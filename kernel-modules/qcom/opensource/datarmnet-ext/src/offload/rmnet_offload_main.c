// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* rmnet_offload main handlers and helpers */

#include <linux/compiler.h>
#include <linux/rcupdate.h>
#include <linux/jhash.h>
#include <linux/spinlock.h>
#include "rmnet_descriptor.h"
#include "rmnet_handlers.h"
#include "rmnet_map.h"
#include "rmnet_module.h"
#include "rmnet_offload_main.h"
#include "rmnet_offload_state.h"
#include "rmnet_offload_engine.h"
#include "rmnet_offload_stats.h"

/* Insert newest first, last 4 bytes of the change id */
static char *verinfo[] = {
	"7972254c",
	"36f0d8b1",
	"58aa9bee",
	"c8acaf85",
	"e218f451",
	"2a44f6be",
	"7415921c"};
module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

/* Lock around our operations for synchronization with flushing and mode
 * mode changes.
 */
static DEFINE_SPINLOCK(rmnet_offload_main_lock);

/* Computes the flow hash over the packet's 5 tuple */
static u32 rmnet_offload_compute_flow_hash(struct rmnet_offload_info *pkt_info)
{
	struct rmnet_offload_header_info *pkt_hdr = &pkt_info->roi_hdrs;
	__be32 pkt_five_tuple[11];
	u32 flow_hash_key_len;
	__be16 sport = 0, dport = 0;

	if (pkt_hdr->roh_trans_proto == RMNET_OFFLOAD_PROTO_UDP ||
	    pkt_hdr->roh_trans_proto == RMNET_OFFLOAD_PROTO_TCP) {
		sport = pkt_hdr->roh_sport;
		dport = pkt_hdr->roh_dport;
	}

	if (pkt_hdr->roh_ip_proto == 0x4) {
		pkt_five_tuple[0] = pkt_hdr->roh_daddr4;
		pkt_five_tuple[1] = pkt_hdr->roh_saddr4;
		pkt_five_tuple[2] = pkt_hdr->roh_trans_proto;
		pkt_five_tuple[3] = dport;
		pkt_five_tuple[4] = sport;
		flow_hash_key_len = 5;
	} else {
		memcpy(&pkt_five_tuple[0], &pkt_hdr->roh_daddr6[0],
		       sizeof(pkt_hdr->roh_daddr6));
		memcpy(&pkt_five_tuple[5], &pkt_hdr->roh_saddr6[0],
		       sizeof(pkt_hdr->roh_daddr6));
		pkt_five_tuple[8] = pkt_hdr->roh_trans_proto;
		pkt_five_tuple[9] = dport;
		pkt_five_tuple[10] = sport;
		flow_hash_key_len = 11;
	}

	return jhash2(pkt_five_tuple, flow_hash_key_len, 0);
}

static void rmnet_offload_update_pkt_size_stats(u32 pkt_len)
{
	u32 size_stat;

	if (pkt_len > 50000)
		size_stat = RMNET_OFFLOAD_STAT_SIZE_50000_PLUS;
	else if (pkt_len > 30000)
		size_stat = RMNET_OFFLOAD_STAT_SIZE_30000_PLUS;
	else if (pkt_len > 23000)
		size_stat = RMNET_OFFLOAD_STAT_SIZE_23000_PLUS;
	else if (pkt_len > 14500)
		size_stat = RMNET_OFFLOAD_STAT_SIZE_14500_PLUS;
	else if (pkt_len > 7000)
		size_stat = RMNET_OFFLOAD_STAT_SIZE_7000_PLUS;
	else if (pkt_len > 1400)
		size_stat = RMNET_OFFLOAD_STAT_SIZE_1400_PLUS;
	else
		size_stat = RMNET_OFFLOAD_STAT_SIZE_0_PLUS;

	rmnet_offload_stats_update(size_stat);
}

static bool rmnet_offload_dissect_pkt(struct rmnet_frag_descriptor *frag_desc,
				      struct rmnet_offload_info *pkt_info)
{
	struct rmnet_offload_header_info *pkt_hdr = &pkt_info->roi_hdrs;
	u8 *payload;
	u16 pkt_len;
	u16 ip_pkt_len;

	pkt_len = frag_desc->len;
	/* Guilty until proven innocent */
	pkt_info->roi_skip_hash = true;
	if (unlikely(!(frag_desc->dev->features & NETIF_F_RXCSUM))) {
		/* Sorry, coalescing only makes sense if RX checksum offload
		 * is enabled.
		 */
		goto done;
	}

	/* Check if the core driver already did work for us during RSB/RSC
	 * packet processing. This allows us to skip many sanity checks as well
	 * as some computation.
	 */
	if (frag_desc->hdrs_valid) {
		struct rmnet_offload_udphdr *up, __up;
		struct rmnet_offload_tcphdr *tp, __tp;

		/* Grab header lengths and protocols */
		pkt_hdr->roh_ip_proto = frag_desc->ip_proto;
		pkt_hdr->roh_ip_len = frag_desc->ip_len;
		pkt_hdr->roh_trans_proto = frag_desc->trans_proto;
		pkt_hdr->roh_trans_len = frag_desc->trans_len;
		pkt_len = frag_desc->len;

		/* Grab the IP flags from the header */
		if (pkt_hdr->roh_ip_proto == 0x4) {
			struct rmnet_offload_iphdr *iph, __iph;

			iph = rmnet_frag_header_ptr(frag_desc, 0, sizeof(*iph),
						    &__iph);
			if (!iph)
				goto done;

			pkt_hdr->roh_saddr4 = iph->roip_saddr;
			pkt_hdr->roh_daddr4 = iph->roip_daddr;
			pkt_hdr->roh_ip_ttl = iph->roip_ttl;
			pkt_hdr->roh_ip_tos = iph->roip_tos;
			pkt_hdr->roh_ip_frag_off = iph->roip_frag_off;
		} else {
			struct rmnet_offload_ipv6hdr *ip6h, __ip6h;

			ip6h = rmnet_frag_header_ptr(frag_desc, 0,
						     sizeof(*ip6h), &__ip6h);
			if (!ip6h)
				goto done;

			memcpy(&pkt_hdr->roh_saddr6[0], &ip6h->roipv6_saddr[0],
			       sizeof(ip6h->roipv6_saddr));
			memcpy(&pkt_hdr->roh_daddr6[0], &ip6h->roipv6_daddr[0],
			       sizeof(ip6h->roipv6_daddr));
			pkt_hdr->roh_flag_word = ip6h->roipv6_flow_lbl;

			if (pkt_hdr->roh_ip_len > sizeof(*ip6h)) {
				int dummy_len;
				__be16 roi_frag_off;
				bool roi_frag;
				u8 roi_proto = ip6h->roipv6_nexthdr;

				/* Extension headers are present. And that
				 * means an empty fragment header could ALSO
				 * be present, as IPA can coalesce those.
				 * Kernel can't handle those if we're able to
				 * coalesce past MAX_SKB_FRAGS and move to the
				 * fraglist; defragmentation will trash the
				 * skb.
				 *
				 * Unfortunately, there's not really a good
				 * way of avoiding the reparse.
				 */
				dummy_len =
					rmnet_frag_ipv6_skip_exthdr(frag_desc,
								    sizeof(*ip6h),
								    &roi_proto,
								    &roi_frag_off,
								    &roi_frag);
				if (dummy_len < 0 || roi_frag_off || roi_frag) {
					/* Frag detected */
					if (roi_proto == RMNET_OFFLOAD_PROTO_FRAGMENT)
						pkt_hdr->roh_ip_len += 8;

					rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_FRAG_FLUSH);
					goto done;
				}
			}
		}

		/* Grab ports and tcp sequence number if needed */
		up = rmnet_frag_header_ptr(frag_desc, pkt_hdr->roh_ip_len,
					   sizeof(*up), &__up);
		if (!up)
			goto done;

		pkt_hdr->roh_sport = up->roudp_source;
		pkt_hdr->roh_dport = up->roudp_dest;
		if (pkt_hdr->roh_trans_proto == RMNET_OFFLOAD_PROTO_TCP) {
			__be32 seq_no;

			tp = rmnet_frag_header_ptr(frag_desc,
						   pkt_hdr->roh_ip_len,
						   sizeof(*tp), &__tp);
			if (!tp)
				goto done;

			if (frag_desc->tcp_seq_set)
				seq_no = frag_desc->tcp_seq;
			else
				seq_no = tp->rotcp_seq;

			pkt_hdr->roh_tcp_seq = ntohl(seq_no);
		}

		/* Compute the flow hash since this is guaranteed to be a
		 * valid TCP/UDP non-fragmented packet.
		 */
		pkt_info->roi_hash_key =
			rmnet_offload_compute_flow_hash(pkt_info);
		/* Compute the data length of the packet */
		pkt_info->roi_payload_len = frag_desc->len -
					    frag_desc->ip_len -
					    frag_desc->trans_len;
		/* Store the frag_descriptor and we're in business */
		pkt_info->roi_frag_desc = frag_desc;
		pkt_info->roi_skip_hash = false;
		return false;
	}

	/* This isn't an RSB/RSC packet, so all bets are off. Make sure
	 * everything is valid before we continue.
	 *
	 * We need to go deeper. Grab your totem and let's go!
	 */
	payload = rmnet_frag_data_ptr(frag_desc);
	if (unlikely(!payload))
		return true;

	pkt_hdr->roh_ip_proto = (payload[0] & 0xF0) >> 4;
	if (pkt_hdr->roh_ip_proto == 0x4) {
		struct rmnet_offload_iphdr *iph, __iph;

		iph = rmnet_frag_header_ptr(frag_desc, 0, sizeof(*iph),
					    &__iph);
		if (!iph)
			goto done;

		pkt_hdr->roh_ip_len = iph->roip_ihl * 4;
		pkt_hdr->roh_trans_proto = iph->roip_protocol;
		pkt_hdr->roh_saddr4 = iph->roip_saddr;
		pkt_hdr->roh_daddr4 = iph->roip_daddr;
		pkt_hdr->roh_ip_ttl = iph->roip_ttl;
		pkt_hdr->roh_ip_tos = iph->roip_tos;
		pkt_hdr->roh_ip_frag_off = iph->roip_frag_off;

		/* Flush out any fragment packets immediately.
		 * Mask value is equivalent to IP_MF (0x2000) OR'd
		 * with IP_OFFSET (0x1FFF).
		 */
		if (iph->roip_frag_off & htons(0x3FFF)) {
			rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_FRAG_FLUSH);
			goto done;
		}

		/* Check for length mismatch */
		ip_pkt_len = ntohs(iph->roip_tot_len);
		pkt_info->roi_len_mismatch = ip_pkt_len != pkt_len;
	} else if (pkt_hdr->roh_ip_proto == 0x6) {
		struct rmnet_offload_ipv6hdr *ip6h, __ip6h;
		int roi_v6_len;
		__be16 roi_frag_off;
		bool roi_frag;
		u8 roi_v6_proto;

		ip6h = rmnet_frag_header_ptr(frag_desc, 0, sizeof(*ip6h),
					     &__ip6h);
		if (!ip6h)
			goto done;

		roi_v6_proto = ip6h->roipv6_nexthdr;
		/* Dive down the ipv6 header chain */
		roi_v6_len = rmnet_frag_ipv6_skip_exthdr(frag_desc,
							 sizeof(*ip6h),
							 &roi_v6_proto,
							 &roi_frag_off,
							 &roi_frag);
		if (roi_v6_len < 0) {
			/* Something somewhere has gone horribly wrong. Let
			 * the stack deal with it.
			 */
			goto done;
		}

		pkt_hdr->roh_ip_len = (u16)roi_v6_len;
		pkt_hdr->roh_trans_proto = roi_v6_proto;
		memcpy(&pkt_hdr->roh_saddr6[0], &ip6h->roipv6_saddr[0],
		       sizeof(ip6h->roipv6_saddr));
		memcpy(&pkt_hdr->roh_daddr6[0], &ip6h->roipv6_daddr[0],
		       sizeof(ip6h->roipv6_daddr));
		pkt_hdr->roh_flag_word = ip6h->roipv6_flow_lbl;
		/* Flush out any fragment packets immediately */
		if (roi_frag_off || roi_frag) {
			/* Add in the fragment header length to any non-first
			 * fragment packets.
			 */
			if (pkt_hdr->roh_trans_proto ==
			    RMNET_OFFLOAD_PROTO_FRAGMENT)
				pkt_hdr->roh_ip_len += 8;

			rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_FRAG_FLUSH);
			goto done;
		}

		/* Check for length mismatch */
		ip_pkt_len = ntohs(ip6h->roipv6_payload_len) + sizeof(*ip6h);
		pkt_info->roi_len_mismatch = ip_pkt_len != pkt_len;
	} else {
		/* Not a valid IP packet */
		return true;
	}

	/* Down another level. Leo's gotta be around here somewhere... */
	if (pkt_hdr->roh_trans_proto == RMNET_OFFLOAD_PROTO_TCP) {
		struct rmnet_offload_tcphdr *tp, __tp;

		tp = rmnet_frag_header_ptr(frag_desc, pkt_hdr->roh_ip_len,
					   sizeof(*tp), &__tp);
		if (!tp)
			goto done;

		pkt_hdr->roh_trans_len = tp->rotcp_doff * 4;
		pkt_hdr->roh_sport = tp->rotcp_source;
		pkt_hdr->roh_dport = tp->rotcp_dest;
		pkt_hdr->roh_tcp_seq = ntohl(tp->rotcp_seq);
	} else if (pkt_hdr->roh_trans_proto == RMNET_OFFLOAD_PROTO_UDP) {
		struct rmnet_offload_udphdr *up, __up;

		up = rmnet_frag_header_ptr(frag_desc, pkt_hdr->roh_ip_len,
					   sizeof(*up), &__up);
		if (!up)
			goto done;

		pkt_hdr->roh_trans_len = sizeof(*up);
		pkt_hdr->roh_sport = up->roudp_source;
		pkt_hdr->roh_dport = up->roudp_dest;
	} else {
		/* Not a protocol we can optimize */
		goto done;
	}

	/* Everything seems fine. Go ahead and compute the hash */
	pkt_info->roi_skip_hash = false;
	pkt_info->roi_hash_key = rmnet_offload_compute_flow_hash(pkt_info);
	if (!pkt_info->roi_len_mismatch) {
		/* Copy the header info into the frag descriptor for the core
		 * driver to use later since everything is kosher.
		 */
		frag_desc->ip_proto = pkt_hdr->roh_ip_proto;
		frag_desc->ip_len = pkt_hdr->roh_ip_len;
		frag_desc->trans_proto = pkt_hdr->roh_trans_proto;
		frag_desc->trans_len = pkt_hdr->roh_trans_len;

		/* Now, and ONLY now, do we dare touch this bit */
		frag_desc->hdrs_valid = 1;
	}

done:
	/* Set payload length based on the headers we found */
	pkt_info->roi_payload_len = pkt_len - pkt_hdr->roh_ip_len -
				    pkt_hdr->roh_trans_len;
	if (pkt_info->roi_len_mismatch)
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_LEN_MISMATCH);

	/* Hold on to the descriptor for later */
	pkt_info->roi_frag_desc = frag_desc;
	return false;
}

/* The main entry point into the module from the core driver */
static void __rmnet_offload_ingress(struct rmnet_frag_descriptor *frag_desc,
				    struct rmnet_port *port)
{
	struct rmnet_offload_state *rmnet_offload = rmnet_offload_state_get();
	struct rmnet_offload_info pkt_info;
	LIST_HEAD(flush_list);

	memset(&pkt_info, 0, sizeof(pkt_info));
	rmnet_offload_lock();
	rmnet_offload->core_port = port;

	if (rmnet_offload_dissect_pkt(frag_desc, &pkt_info)) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_NON_IP_COUNT);
		rmnet_recycle_frag_descriptor(frag_desc, port);
		goto out;
	}

	/* We know the packet is an IP packet now */
	rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_PRE_IP_COUNT);
	if (pkt_info.roi_skip_hash) {
		/* We're not optimizing this packet */
		goto flush;
	} else if (pkt_info.roi_len_mismatch) {
		/* Can't optimize this, but we are potentially holding other
		 * packets in this flow. Flush the flow with this has value
		 * to avoid OOO packets.
		 */
		rmnet_offload_engine_flush_by_hash(pkt_info.roi_hash_key,
						   &flush_list);
		goto flush;
	}

	/* Skip any bad checksum packets.
	 * We wait to do this until now to allow any packets that won't be
	 * checksummed by hardware (i.e. non-TCP/UDP, fragments, padding) to
	 * be caught by the above checks. This ensures we report stats
	 * correctly and don't increment the "bad checksum" field for otherwise
	 * valid packets.
	 */
	if (!frag_desc->csum_valid) {
		/* Possible behavior change here. We know that the checksum is
		 * incorrect, so we flush the packet immediately; we do not
		 * flush anything internally. This can potentially make the bad
		 * packet show up in tcpdump as a TCP OOO packet. If we want to
		 * avoid that (even though it doesn't really hurt anything), we
		 * could flush by the hash. Worst case, one of the 5 tuple
		 * components was corrupted so the hash ends up being the same
		 * as another flow we're holding so we flush it prematurely.
		 */
		goto flush;
	}

	if (!rmnet_offload_engine_ingress(&pkt_info, &flush_list))
		goto flush;

	goto out;

flush:
	rmnet_offload_flush_current_pkt(&pkt_info, &flush_list);
out:
	rmnet_offload_unlock();
	rmnet_offload_deliver_descs(&flush_list);
}

static void rmnet_offload_ingress(struct list_head *desc_list,
				  struct rmnet_port *port)
{
	struct rmnet_frag_descriptor *frag, *tmp;

	list_for_each_entry_safe(frag, tmp, desc_list, list) {
		list_del_init(&frag->list);
		__rmnet_offload_ingress(frag, port);
	}
}

void rmnet_offload_lock(void)
{
	spin_lock_bh(&rmnet_offload_main_lock);
}

void rmnet_offload_unlock(void)
{
	spin_unlock_bh(&rmnet_offload_main_lock);
}

static const struct rmnet_module_hook_register_info
rmnet_offload_main_hook = {
	.hooknum = RMNET_MODULE_HOOK_OFFLOAD_INGRESS,
	.func = rmnet_offload_ingress,
};

void rmnet_offload_set_hooks(void)
{
	rmnet_module_hook_register(&rmnet_offload_main_hook, 1);
}

void rmnet_offload_unset_hooks(void)
{
	rmnet_module_hook_unregister_no_sync(&rmnet_offload_main_hook, 1);
}

/* Deliver the final descriptors to the core driver */
void rmnet_offload_deliver_descs(struct list_head *desc_list)
{
	struct rmnet_offload_state *rmnet_offload = rmnet_offload_state_get();
	struct rmnet_frag_descriptor *frag_desc, *tmp;

	list_for_each_entry_safe(frag_desc, tmp, desc_list, list) {
		/* Log the outgoing size */
		rmnet_offload_update_pkt_size_stats(frag_desc->len);
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_POST_IP_COUNT);
		list_del_init(&frag_desc->list);
		rmnet_frag_deliver(frag_desc, rmnet_offload->core_port);
	}
}

/* Flush the packet that we're currently handling back to the core driver */
void rmnet_offload_flush_current_pkt(struct rmnet_offload_info *pkt_info,
				     struct list_head *flush_list)
{
	struct rmnet_frag_descriptor *frag_desc = pkt_info->roi_frag_desc;
	u32 pkt_len = pkt_info->roi_payload_len +
		      pkt_info->roi_hdrs.roh_ip_len +
		      pkt_info->roi_hdrs.roh_trans_len;

	/* Sanity check. Make sure the data will fit in the IP header */
	if (pkt_len > 65536)
		return;

	/* Only set the hash key if we actually calculated it */
	if (!pkt_info->roi_skip_hash)
		frag_desc->hash = pkt_info->roi_hash_key;

	list_add_tail(&frag_desc->list, flush_list);
}

/* Handles entering powersave mode. DL markers are turned off now.
 * This is a no-op for us currently, as we don't need to change anything
 * about our operation.
 */
void rmnet_offload_handle_powersave_on(void *port)
{
}

/* Handles exiting powersave mode. DL markers are turned on again.
 * This is also a no-op for us currently, since we didn't change anything
 * when powersave was enabled.
 */
void rmnet_offload_handle_powersave_off(void *port)
{
}

/* Handles DL maker start notifications from the core driver */
void
rmnet_offload_handle_dl_header(struct rmnet_map_dl_ind_hdr *dlhdr,
			       struct rmnet_map_control_command_header *cmd)
{
	struct rmnet_offload_state *rmnet_offload = rmnet_offload_state_get();
	LIST_HEAD(flush_list);
	(void)cmd;
	rmnet_offload_lock();

	/* If we get multiple starts in a row, assume the end was lost and
	 * flush everything out.
	 */
	if (rmnet_offload->dl_marker_state.dl_marker_start &&
	    rmnet_offload_engine_flush_all_flows(&flush_list))
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_DL_START_FLUSH);

	/* Store away the meta information */
	rmnet_offload->dl_marker_state.dl_marker_start = true;
	rmnet_offload->dl_marker_state.dl_marker_seq = dlhdr->le.seq;
	rmnet_offload->dl_marker_state.dl_marker_pkts = dlhdr->le.pkts;
	rmnet_offload_unlock();
	rmnet_offload_deliver_descs(&flush_list);
}

/* Handles DL maker end notifications from the core driver */
void
rmnet_offload_handle_dl_trailer(struct rmnet_map_dl_ind_trl *dltrl,
				struct rmnet_map_control_command_header *cmd)
{
	struct rmnet_offload_state *rmnet_offload = rmnet_offload_state_get();
	LIST_HEAD(flush_list);

	(void)cmd;
	rmnet_offload_lock();

	/* Check on the sequence number. If they don't match, a marker was lost
	 * somewhere. Log it, but it doesn't change our behavior.
	 */
	if (rmnet_offload->dl_marker_state.dl_marker_seq != dltrl->seq_le)
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_DL_SEQ_MISMATCH);

	/* Flush everything we've got */
	if (rmnet_offload_engine_flush_all_flows(&flush_list))
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_DL_END_FLUSH);

	/* Reset state information */
	rmnet_offload->dl_marker_state.dl_marker_start = false;
	rmnet_offload->dl_marker_state.dl_marker_seq = 0;
	rmnet_offload->dl_marker_state.dl_marker_pkts = 0;
	rmnet_offload_unlock();
	rmnet_offload_deliver_descs(&flush_list);
}
