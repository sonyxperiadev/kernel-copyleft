/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __RMNET_OFFLOAD_STATS_H__
#define __RMNET_OFFLOAD_STATS_H__

#include <linux/types.h>

enum {
	/* Number of inbound IP packets */
	RMNET_OFFLOAD_STAT_PRE_IP_COUNT,
	/* Number of outbound IP packets */
	RMNET_OFFLOAD_STAT_POST_IP_COUNT,
	/* Number of non-IP packets dropped */
	RMNET_OFFLOAD_STAT_NON_IP_COUNT,
	/* Number of flushes caused by 2 start markers in a row */
	RMNET_OFFLOAD_STAT_DL_START_FLUSH,
	/* Number of flushes caused by start-end marker sequence mismatch */
	RMNET_OFFLOAD_STAT_DL_SEQ_MISMATCH,
	/* Number of flushes caused by end markers */
	RMNET_OFFLOAD_STAT_DL_END_FLUSH,
	/* Number of IP fragments received */
	RMNET_OFFLOAD_STAT_FRAG_FLUSH,
	/* Number of QMAP-IP packet length mismatches */
	RMNET_OFFLOAD_STAT_LEN_MISMATCH,
	/* Number of flows evicted to make room for another */
	RMNET_OFFLOAD_STAT_FLOW_EVICT,
	/* Number of flushes caused by end of skb chain */
	RMNET_OFFLOAD_STAT_CHAIN_FLUSH,
	/* Number of outbound TCP/UDP packets not coalesced because of protocol mode */
	RMNET_OFFLOAD_STAT_PROTO_SKIPPED,
	/* Number of outbound TCP/UDP packets flushed because of mode chages */
	RMNET_OFFLOAD_STAT_PROTO_FLUSH,
	/* Number of outbound TCP packets flushed because of IP header changes */
	RMNET_OFFLOAD_STAT_TCP_FORCE_FLUSH,
	/* Number of outbound TCP packets flushed because of TCP flags */
	RMNET_OFFLOAD_STAT_TCP_FLAG_FLUSH,
	/* Number of outbound TCP packets flushed because of TCP option changes */
	RMNET_OFFLOAD_STAT_TCP_OPTION_FLUSH,
	/* Number of outbound TCP packets flushed because of out-of-order sequencing */
	RMNET_OFFLOAD_STAT_TCP_OOO_FLUSH,
	/* Number of outbound TCP packets flushed because of changing data length */
	RMNET_OFFLOAD_STAT_TCP_LEN_FLUSH,
	/* Number of outbound TCP packets flushed because of hitting max byte limit */
	RMNET_OFFLOAD_STAT_TCP_BYTE_FLUSH,
	/* Number of outbound UDP packets flushed because of IP header changes */
	RMNET_OFFLOAD_STAT_UDP_FORCE_FLUSH,
	/* Number of outbound UDP packets flushed because of changing data length */
	RMNET_OFFLOAD_STAT_UDP_LEN_FLUSH,
	/* Number of outbound UDP packets flushed because of hitting max byte limit */
	RMNET_OFFLOAD_STAT_UDP_BYTE_FLUSH,
	/* Outbound packet size distribution */
	RMNET_OFFLOAD_STAT_SIZE_0_PLUS,
	RMNET_OFFLOAD_STAT_SIZE_1400_PLUS,
	RMNET_OFFLOAD_STAT_SIZE_7000_PLUS,
	RMNET_OFFLOAD_STAT_SIZE_14500_PLUS,
	RMNET_OFFLOAD_STAT_SIZE_23000_PLUS,
	RMNET_OFFLOAD_STAT_SIZE_30000_PLUS,
	RMNET_OFFLOAD_STAT_SIZE_50000_PLUS,
	RMNET_OFFLOAD_STAT_MAX,
};

void __rmnet_offload_stats_update(u32 stat, u64 inc);
void rmnet_offload_stats_update(u32 stat);

#endif
