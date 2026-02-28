// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* rmnet_offload TCP optimization engine */
#include <linux/list.h>
#include "rmnet_descriptor.h"
#include "rmnet_offload_main.h"
#include "rmnet_offload_engine.h"
#include "rmnet_offload_stats.h"
#include "rmnet_offload_knob.h"

union rmnet_offload_tcp_opts {
	struct rmnet_offload_tcphdr rotopt_tcp;
	/* Maximum TCP header size. (doff * 4) */
	u8 rotopt_buf[60];
};

/* Check if the TCP flags prevent us from coalescing the packet */
static bool rmnet_offload_tcp_flag_flush(struct rmnet_offload_info *pkt)
{
	struct rmnet_offload_tcphdr *tp, __tp;
	__be32 flush_mask;
	u8 flags;

	tp = rmnet_frag_header_ptr(pkt->roi_frag_desc, pkt->roi_hdrs.roh_ip_len,
				   sizeof(*tp), &__tp);
	if (!tp)
		/* How did you even get this far? Panic and flush everything */
		return true;

	/* OK, being kinda cheeky here to hide this a bit more than it would
	 * be otherwise, but it also cuts down on the number of conditions in
	 * the if statement, so you can check the flags in a single AND.
	 *
	 * TCP flags are as follows:
	 * | C | E | U | A | P | R | S | F |
	 *   ^       ^       ^   ^   ^   ^
	 */
	flush_mask = 0xAF;
	flags = tp->rotcp_flags;
	if (pkt->roi_frag_desc->tcp_flags_set)
		flags = (u8)ntohs(pkt->roi_frag_desc->tcp_flags);

	/* Pure ACKs or any special flags indicated above cause us to flush */
	if ((!pkt->roi_payload_len && (flags & 0x10)) || (flags & flush_mask))
		return true;

	return false;
}

/* Compare the TCP options fields */
static bool rmnet_offload_tcp_option_mismatch(struct rmnet_offload_flow *flow,
					      struct rmnet_offload_info *pkt)
{
	union rmnet_offload_tcp_opts *flow_hdr, __flow_hdr;
	union rmnet_offload_tcp_opts *pkt_hdr, __pkt_hdr;
	struct rmnet_frag_descriptor *flow_desc;
	u32 opt_len, i;

	/* Grab TCP header including options */
	flow_desc = list_first_entry(&flow->rof_pkts,
				     struct rmnet_frag_descriptor, list);
	flow_hdr = rmnet_frag_header_ptr(flow_desc, flow->rof_hdrs.roh_ip_len,
					 flow->rof_hdrs.roh_trans_len,
					 &__flow_hdr);
	if (!flow_hdr)
		/* Uhh, lolwat? Reality is collapsing, so let's flush... */
		return true;

	pkt_hdr = rmnet_frag_header_ptr(pkt->roi_frag_desc,
					pkt->roi_hdrs.roh_ip_len,
					pkt->roi_hdrs.roh_trans_len,
					&__pkt_hdr);
	if (!pkt_hdr)
		return true;

	opt_len = flow_hdr->rotopt_tcp.rotcp_doff * 4;
	/* Obviously, if the lengths are different, something has changed */
	if (pkt_hdr->rotopt_tcp.rotcp_doff * 4 != opt_len)
		return true;

	/* Compare the words. Memcmp is too easy ;). Also, this is how the
	 * kernel does it, so hey.
	 */
	for (i = sizeof(flow_hdr->rotopt_tcp); i < opt_len; i += 4) {
		if (*(u32 *)(flow_hdr->rotopt_buf + i) ^
		    *(u32 *)(pkt_hdr->rotopt_buf + i))
			return true;
	}

	return false;
}

/* Check if we can merge the packet with the flow */
static int rmnet_offload_tcp_merge_check(struct rmnet_offload_flow *flow,
					 struct rmnet_offload_info *pkt)
{
	u64 tcp_byte_limit;
	u32 gso_len;

	/* 1: check the TCP flags to see if this packet can be coalesced */
	if (rmnet_offload_tcp_flag_flush(pkt)) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_TCP_FLAG_FLUSH);
		return RMNET_OFFLOAD_ENGINE_FLUSH_ALL;
	}

	/* 2: check the number of packets held. If we don't have anything
	 * stored right now, we can stop here.
	 */
	if (!flow->rof_pkts_held)
		return RMNET_OFFLOAD_ENGINE_FLUSH_NONE;

	/* 3: Compare the TCP options between the flow header and the new
	 * packet.
	 */
	if (rmnet_offload_tcp_option_mismatch(flow, pkt)) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_TCP_OPTION_FLUSH);
		return RMNET_OFFLOAD_ENGINE_FLUSH_SOME;
	}

	/* 4: Check packet ordering */
	if (pkt->roi_hdrs.roh_tcp_seq ^ flow->rof_hdrs.roh_tcp_seq) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_TCP_OOO_FLUSH);
		return RMNET_OFFLOAD_ENGINE_FLUSH_ALL;
	}

	/* 5: Check packet size */
	gso_len = (pkt->roi_frag_desc->gso_size) ?: pkt->roi_payload_len;
	if (gso_len != flow->rof_gso_len) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_TCP_LEN_FLUSH);
		return RMNET_OFFLOAD_ENGINE_FLUSH_SOME;
	}

	/* 6: Check the byte limit */
	tcp_byte_limit =
		rmnet_offload_knob_get(RMNET_OFFLOAD_KNOB_TCP_BYTE_LIMIT);
	if (pkt->roi_payload_len + flow->rof_len >= tcp_byte_limit) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_TCP_BYTE_FLUSH);
		return RMNET_OFFLOAD_ENGINE_FLUSH_SOME;
	}

	/* Other packets exist in the flow state */
	pkt->roi_first_pkt = false;
	return RMNET_OFFLOAD_ENGINE_FLUSH_NONE;
}

/* Handle a TCP packet */
bool rmnet_offload_engine_tcp_ingress(struct rmnet_offload_flow *flow,
				      struct rmnet_offload_info *pkt,
				      bool force_flush,
				      struct list_head *flush_list)
{
	int rc;

	if (force_flush) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_TCP_FORCE_FLUSH);
		rmnet_offload_engine_flush_flow(flow, flush_list);
		rmnet_offload_flush_current_pkt(pkt, flush_list);
		return true;
	}

	rc = rmnet_offload_tcp_merge_check(flow, pkt);
	if (rc == RMNET_OFFLOAD_ENGINE_FLUSH_NONE) {
		/* Coalesce */
		rmnet_offload_engine_add_flow_pkt(flow, pkt);
	} else if (rc == RMNET_OFFLOAD_ENGINE_FLUSH_SOME) {
		/* Flush flow and insert packet */
		rmnet_offload_engine_flush_flow(flow, flush_list);
		rmnet_offload_engine_add_flow_pkt(flow, pkt);
	} else {
		/* Flush everything */
		rmnet_offload_engine_flush_flow(flow, flush_list);
		rmnet_offload_flush_current_pkt(pkt, flush_list);
	}

	return true;
}
