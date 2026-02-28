// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* rmnet_offload UDP optimization engine */
#include "rmnet_descriptor.h"
#include "rmnet_offload_main.h"
#include "rmnet_offload_engine.h"
#include "rmnet_offload_stats.h"
#include "rmnet_offload_knob.h"

/* Check if we can merge the packet with the flow */
static int rmnet_offload_udp_merge_check(struct rmnet_offload_flow *flow,
					 struct rmnet_offload_info *pkt)
{
	u64 udp_byte_limit;
	u16 gso_len;

	/* 1: If we're not holding anything, the packet can be merged
	 * trivially.
	 */
	if (!flow->rof_pkts_held)
		return RMNET_OFFLOAD_ENGINE_FLUSH_NONE;

	/* 2: Check packet size */
	gso_len = (pkt->roi_frag_desc->gso_size) ?: pkt->roi_payload_len;
	if (gso_len != flow->rof_gso_len) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_UDP_LEN_FLUSH);
		return RMNET_OFFLOAD_ENGINE_FLUSH_SOME;
	}

	/* 3: Check byte limit */
	udp_byte_limit =
		rmnet_offload_knob_get(RMNET_OFFLOAD_KNOB_UDP_BYTE_LIMIT);
	if (pkt->roi_payload_len + flow->rof_len >= udp_byte_limit) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_UDP_BYTE_FLUSH);
		return RMNET_OFFLOAD_ENGINE_FLUSH_SOME;
	}

	/* Other packets exist in the flow state */
	pkt->roi_first_pkt = false;
	return RMNET_OFFLOAD_ENGINE_FLUSH_NONE;
}

/* Handle a UDP packet */
bool rmnet_offload_engine_udp_ingress(struct rmnet_offload_flow *flow,
				      struct rmnet_offload_info *pkt,
				      bool force_flush,
				      struct list_head *flush_list)
{
	int rc;

	if (force_flush) {
		rmnet_offload_stats_update(RMNET_OFFLOAD_STAT_UDP_FORCE_FLUSH);
		rmnet_offload_engine_flush_flow(flow, flush_list);
		rmnet_offload_flush_current_pkt(pkt, flush_list);
		return true;
	}

	rc = rmnet_offload_udp_merge_check(flow, pkt);
	if (rc == RMNET_OFFLOAD_ENGINE_FLUSH_NONE) {
		/* Coalesce */
		rmnet_offload_engine_add_flow_pkt(flow, pkt);
	} else if (rc == RMNET_OFFLOAD_ENGINE_FLUSH_SOME) {
		/* Flush flow and insert packet */
		rmnet_offload_engine_flush_flow(flow, flush_list);
		rmnet_offload_engine_add_flow_pkt(flow, pkt);
	}

	return true;
}
