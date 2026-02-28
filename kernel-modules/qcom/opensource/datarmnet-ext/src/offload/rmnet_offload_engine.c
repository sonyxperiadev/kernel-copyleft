// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* rmnet_offload core optimization engine */
#include <linux/log2.h>
#include <linux/list.h>
#include <linux/hashtable.h>
#include "rmnet_descriptor.h"
#include "rmnet_module.h"
#include "rmnet_offload_state.h"
#include "rmnet_offload_engine.h"
#include "rmnet_offload_main.h"
#include "rmnet_offload_tcp.h"
#include "rmnet_offload_udp.h"
#include "rmnet_offload_stats.h"
#include "rmnet_offload_knob.h"

#define RMNET_OFFLOAD_ENGINE_HASH_TABLE_BITS \
	(const_ilog2(RMNET_OFFLOAD_ENGINE_NUM_FLOWS))

static DEFINE_HASHTABLE(rmnet_offload_flow_table,
			RMNET_OFFLOAD_ENGINE_HASH_TABLE_BITS);

/* Flushes all active flows of a certain transport protocol */
static u32 rmnet_offload_engine_flush_by_protocol(u8 l4_proto,
						  struct list_head *flush_list)
{
	struct rmnet_offload_flow *flow_cursor;
	int bkt_cursor;
	u32 flushed = 0;

	hash_for_each(rmnet_offload_flow_table, bkt_cursor, flow_cursor,
		      rof_flow_list) {
		if (flow_cursor->rof_pkts_held &&
		    flow_cursor->rof_hdrs.roh_trans_proto == l4_proto) {
			flushed++;
			rmnet_offload_engine_flush_flow(flow_cursor, flush_list);
		}
	}

	return flushed;
}

/* Check if a specific protocol should be optimized */
static bool rmnet_offload_engine_optimize_protocol(u8 l4_proto)
{
	u64 engine_mode;

	engine_mode = rmnet_offload_knob_get(RMNET_OFFLOAD_KNOB_ENGINE_MODE);
	if (engine_mode == RMNET_OFFLOAD_ENGINE_MODE_ALL)
		return true;

	if (engine_mode == RMNET_OFFLOAD_ENGINE_MODE_TCP &&
	    l4_proto == RMNET_OFFLOAD_PROTO_TCP)
		return true;

	if (engine_mode == RMNET_OFFLOAD_ENGINE_MODE_UDP &&
	    l4_proto == RMNET_OFFLOAD_PROTO_UDP)
		return true;

	return false;
}

/* Compare the ip flags of a flow and the incoming packet */
static bool rmnet_offload_engine_ip_mismatch(struct rmnet_offload_flow *flow,
					     struct rmnet_offload_info *pkt)
{
	/* Can't mismatch if the flow is empty */
	if (!flow->rof_pkts_held)
		return false;

	if (pkt->roi_hdrs.roh_ip_proto == 0x4) {
		struct rmnet_offload_header_info *flow_hdr, *pkt_hdr;

		flow_hdr = &flow->rof_hdrs;
		pkt_hdr = &pkt->roi_hdrs;
		if (flow_hdr->roh_ip_ttl ^ pkt_hdr->roh_ip_ttl ||
		    flow_hdr->roh_ip_tos ^ pkt_hdr->roh_ip_tos ||
		    flow_hdr->roh_ip_frag_off ^ pkt_hdr->roh_ip_frag_off ||
		    flow_hdr->roh_ip_len ^ pkt_hdr->roh_ip_len)
			return true;
	} else if (pkt->roi_hdrs.roh_ip_proto == 0x6) {
		__be32 flow_word, pkt_word;
		__be32 word_mismatch;

		flow_word = flow->rof_hdrs.roh_flag_word;
		pkt_word = pkt->roi_hdrs.roh_flag_word;
		word_mismatch = flow_word ^ pkt_word;
		if (word_mismatch & htonl(0x0FF00000))
			return true;
	}

	return false;
}

/* Match an incoming packet against a flow in our table */
static bool rmnet_offload_engine_flow_match(struct rmnet_offload_flow *flow,
					    struct rmnet_offload_info *pkt)
{
	struct rmnet_offload_header_info *flow_hdr, *pkt_hdr;

	flow_hdr = &flow->rof_hdrs;
	pkt_hdr = &pkt->roi_hdrs;
	/* If the flow is empty, it has no header information. We rely on
	 * the hash key to tell us what was there.
	 */
	if (!flow->rof_pkts_held)
		return flow->rof_hash_key == pkt->roi_hash_key;

	/* Transport protocokk must match */
	if (flow_hdr->roh_trans_proto != pkt_hdr->roh_trans_proto)
		return false;

	/* Grab the ports from the L4 header. Fortunately, both TCP and UDP
	 * these in the same location in the header.
	 */
	if (flow_hdr->roh_sport ^ pkt_hdr->roh_sport ||
	    flow_hdr->roh_dport ^ pkt_hdr->roh_dport)
		return false;

	/* Compare the addresses */
	if (pkt_hdr->roh_ip_proto == 0x4) {
		if (flow_hdr->roh_saddr4 ^ pkt_hdr->roh_saddr4 ||
		    flow_hdr->roh_daddr4 ^ pkt_hdr->roh_daddr4)
			return false;
	} else if (pkt_hdr->roh_ip_proto == 0x6) {
		if (memcmp(flow_hdr->roh_saddr6, pkt_hdr->roh_saddr6,
		           sizeof(pkt_hdr->roh_saddr6)) ||
		    memcmp(flow_hdr->roh_daddr6, pkt_hdr->roh_daddr6,
			   sizeof(pkt_hdr->roh_daddr6)))
			return false;
	} else {
		/* This shouldn't ever be hit. But returning false here beats
		 * returning true below and storing the packet somewhere.
		 */
		return false;
	}

	return true;
}

/* Select a flow node to use for a new flow we're going to store */
static struct rmnet_offload_flow *rmnet_offload_engine_recycle(void)
{
	struct rmnet_offload_state *rmnet_offload = rmnet_offload_state_get();
	struct rmnet_offload_engine_state *state;
	struct rmnet_offload_flow *new_flow;
	LIST_HEAD(flush_list);

	state = &rmnet_offload->engine_state;
	if (state->roe_nodes_used < RMNET_OFFLOAD_ENGINE_NUM_FLOWS) {
		/* Still have a few that we've never used, so we can fast
		 * path this flow.
		 */
		new_flow = &state->roe_flow_pool[state->roe_nodes_used];
		state->roe_nodes_used++;
		return new_flow;
	}

	/* Recycle one of the already used flows.
	 * Traditionally, we've just used a modular counter here to
	 * choose which one we replace. Could potentially add a little
	 * more intelligence and check for empty nodes or do a LRU scheme
	 * if we wanted to avoid potentially prematurely flushing an
	 * active flow.
	 */
	new_flow = &state->roe_flow_pool[state->roe_recycle_idx];
	state->roe_recycle_idx++;
	state->roe_recycle_idx %= RMNET_OFFLOAD_ENGINE_NUM_FLOWS;
	hash_del(&new_flow->rof_flow_list);
	if (new_flow->rof_pkts_held) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_FLOW_EVICT);
		rmnet_offload_engine_flush_flow(new_flow, &flush_list);
	}

	rmnet_offload_deliver_descs(&flush_list);
	return new_flow;
}

/* Flush all flows at the end of the packet chain */
static void rmnet_offload_engine_chain_flush(void)
{
	LIST_HEAD(flush_list);

	rmnet_offload_lock();
	if (rmnet_offload_engine_flush_all_flows(&flush_list))
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_CHAIN_FLUSH);

	rmnet_offload_unlock();

	rmnet_offload_deliver_descs(&flush_list);
}

static const struct rmnet_module_hook_register_info
rmnet_offload_engine_hook = {
	.hooknum = RMNET_MODULE_HOOK_OFFLOAD_CHAIN_END,
	.func = rmnet_offload_engine_chain_flush,
};

/* Set hook for flushing on end of chain notifications */
void rmnet_offload_engine_enable_chain_flush(void)
{
	rmnet_module_hook_register(&rmnet_offload_engine_hook, 1);
}

/* Unset hook for flushing on end of chain notifications */
void rmnet_offload_engine_disable_chain_flush(void)
{
	rmnet_module_hook_unregister_no_sync(&rmnet_offload_engine_hook, 1);
}

/* Handle engine mode change notifications */
int rmnet_offload_engine_mode_change(u64 old_mode, u64 new_mode)
{
	LIST_HEAD(flush_list);
	u32 flushed = 0;

	/* If all we did was add a protocol or two to optimize, then nothing
	 * needs to be flushed. O frabjous day!
	 */
	if (old_mode == RMNET_OFFLOAD_ENGINE_MODE_NONE ||
	    new_mode == RMNET_OFFLOAD_ENGINE_MODE_ALL)
		return 0;

	/* Flush any flows belonging to the protocol(s) we're not optimizing */
	switch (new_mode) {
	case RMNET_OFFLOAD_ENGINE_MODE_TCP:
		flushed =
			rmnet_offload_engine_flush_by_protocol(RMNET_OFFLOAD_PROTO_UDP,
							       &flush_list);
		break;
	case RMNET_OFFLOAD_ENGINE_MODE_UDP:
		flushed =
			rmnet_offload_engine_flush_by_protocol(RMNET_OFFLOAD_PROTO_TCP,
							       &flush_list);
		break;
	case RMNET_OFFLOAD_ENGINE_MODE_NONE:
		flushed = rmnet_offload_engine_flush_all_flows(&flush_list);
		break;
	}

	__rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_PROTO_FLUSH, flushed);

	rmnet_offload_deliver_descs(&flush_list);
	return 0;
}

/* Combines packets in a given flow and returns them to the core driver */
void rmnet_offload_engine_flush_flow(struct rmnet_offload_flow *flow,
				     struct list_head *flush_list)
{
	struct rmnet_offload_state *rmnet_offload = rmnet_offload_state_get();
	struct rmnet_frag_descriptor *head_frag, *frag_iter, *tmp;
	struct rmnet_offload_header_info *flow_hdr = &flow->rof_hdrs;
	u32 hlen = flow_hdr->roh_ip_len + flow_hdr->roh_trans_len;

	if (!flow->rof_pkts_held)
		return;

	head_frag = list_first_entry(&flow->rof_pkts,
				     struct rmnet_frag_descriptor, list);
	/* Set GSO segs if it hasn't been initialized yet, e.g. checksum
	 * offload packets.
	 */
	if (!head_frag->gso_segs)
		head_frag->gso_segs = 1;

	head_frag->gso_size = flow->rof_gso_len;
	/* Add subsequent packets to the main one, updating the GSO information
	 * and pulling any unneeded headers along the way.
	 */
	frag_iter = head_frag;
	list_for_each_entry_safe_continue(frag_iter, tmp, &flow->rof_pkts,
					  list) {
		u32 dlen = frag_iter->len - hlen;

		if (!rmnet_frag_descriptor_add_frags_from(head_frag, frag_iter,
							  hlen, dlen)) {
			head_frag->gso_segs += (frag_iter->gso_segs) ?: 1;
			head_frag->coal_bytes += frag_iter->coal_bytes;
			head_frag->coal_bufsize += frag_iter->coal_bufsize;
		}

		rmnet_recycle_frag_descriptor(frag_iter,
					      rmnet_offload->core_port);
	}

	/* Set the hash value and fire it off */
	head_frag->hash = flow->rof_hash_key;
	list_del_init(&head_frag->list);
	list_add_tail(&head_frag->list, flush_list);
	flow->rof_pkts_held = 0;
	flow->rof_len = 0;
}

/* Flush any active flows that match a given hash value */
void rmnet_offload_engine_flush_by_hash(u32 hash_val,
					struct list_head *flush_list)
{
	struct rmnet_offload_flow *flow;

	hash_for_each_possible(rmnet_offload_flow_table, flow, rof_flow_list,
			       hash_val) {
		if (flow->rof_hash_key == hash_val && flow->rof_pkts_held)
			rmnet_offload_engine_flush_flow(flow, flush_list);
	}
}

/* Flush all active flows. Returns the number flushed */
u32 rmnet_offload_engine_flush_all_flows(struct list_head *flush_list)
{
	struct rmnet_offload_flow *flow;
	int bkt_cursor;
	u32 flushed = 0;

	hash_for_each(rmnet_offload_flow_table, bkt_cursor, flow,
		      rof_flow_list) {
		if (flow->rof_pkts_held) {
			flushed++;
			rmnet_offload_engine_flush_flow(flow, flush_list);
		}
	}

	return flushed;
}

/* Add a packet to a flow node */
void rmnet_offload_engine_add_flow_pkt(struct rmnet_offload_flow *flow,
				       struct rmnet_offload_info *pkt)
{
	if (pkt->roi_first_pkt) {
		/* Copy over the flow information */
		memcpy(&flow->rof_hdrs, &pkt->roi_hdrs,
		       sizeof(flow->rof_hdrs));
		flow->rof_hash_key = pkt->roi_hash_key;
		flow->rof_gso_len = (pkt->roi_frag_desc->gso_size) ?:
				    pkt->roi_payload_len;
	}

	/* Set the next sequence number for tcp flows */
	if (pkt->roi_hdrs.roh_trans_proto == RMNET_OFFLOAD_PROTO_TCP)
		flow->rof_hdrs.roh_tcp_seq += pkt->roi_payload_len;

	/* Hold the packet */
	list_add_tail(&pkt->roi_frag_desc->list, &flow->rof_pkts);
	flow->rof_pkts_held++;
	flow->rof_len += pkt->roi_payload_len;
}

/* Main entry point into the core engine framework */
bool rmnet_offload_engine_ingress(struct rmnet_offload_info *pkt,
				  struct list_head *flush_list)
{
	struct rmnet_offload_flow *flow;
	bool flow_node_found = false;
	u8 pkt_proto = pkt->roi_hdrs.roh_trans_proto;

	if (!rmnet_offload_engine_optimize_protocol(pkt_proto)) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_PROTO_SKIPPED);
		return false;
	}

	hash_for_each_possible(rmnet_offload_flow_table, flow, rof_flow_list,
			       pkt->roi_hash_key) {
		bool ip_flush;

		if (!rmnet_offload_engine_flow_match(flow, pkt))
			continue;

node_found:
		ip_flush = rmnet_offload_engine_ip_mismatch(flow, pkt);
		/* Set to true by default. Protocol handlers will handle
		 * adjusting this if needed.
		 */
		pkt->roi_first_pkt = true;
		flow_node_found = true;

		switch (pkt_proto) {
		case RMNET_OFFLOAD_PROTO_TCP:
			return rmnet_offload_engine_tcp_ingress(flow, pkt,
								ip_flush,
								flush_list);
		case RMNET_OFFLOAD_PROTO_UDP:
			return rmnet_offload_engine_udp_ingress(flow, pkt,
								ip_flush,
								flush_list);
		default:
			/* Should never be hit */
			return false;
		}
	}

	if (!flow_node_found) {
		/* This is a new flow. Get a node and retry */
		flow = rmnet_offload_engine_recycle();
		flow->rof_hash_key = pkt->roi_hash_key;
		hash_add(rmnet_offload_flow_table, &flow->rof_flow_list,
			 flow->rof_hash_key);
		goto node_found;
	}

	/* This is never hit, but to keep gcc happy... */
	return false;
}

/* Tears down the internal engine state */
void rmnet_offload_engine_exit(void)
{
	struct rmnet_offload_flow *flow;
	struct hlist_node *tmp;
	int bkt_cursor;

	/* Avoid holding any pointers to memory that will be freed */
	hash_for_each_safe(rmnet_offload_flow_table, bkt_cursor, tmp, flow,
			   rof_flow_list)
		hash_del(&flow->rof_flow_list);
}

/* Initializes the internal engine state */
int rmnet_offload_engine_init(void)
{
	struct rmnet_offload_state *rmnet_offload = rmnet_offload_state_get();
	u8 i;

	/* Initialize the flow nodes */
	for (i = 0; i < RMNET_OFFLOAD_ENGINE_NUM_FLOWS; i++) {
		struct rmnet_offload_flow *flow;

		flow = &rmnet_offload->engine_state.roe_flow_pool[i];
		INIT_LIST_HEAD(&flow->rof_pkts);
		INIT_HLIST_NODE(&flow->rof_flow_list);
	}

	return RMNET_OFFLOAD_MGMT_SUCCESS;
}
