/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __RMNET_OFFLOAD_ENGINE_H__
#define __RMNET_OFFLOAD_ENGINE_H__

#include <linux/types.h>
#include "rmnet_offload_main.h"

#define RMNET_OFFLOAD_ENGINE_NUM_FLOWS 50

enum {
	RMNET_OFFLOAD_ENGINE_FLUSH_ALL,
	RMNET_OFFLOAD_ENGINE_FLUSH_SOME,
	RMNET_OFFLOAD_ENGINE_FLUSH_NONE,
};

enum {
	RMNET_OFFLOAD_ENGINE_MODE_MIN,
	RMNET_OFFLOAD_ENGINE_MODE_ALL = RMNET_OFFLOAD_ENGINE_MODE_MIN,
	RMNET_OFFLOAD_ENGINE_MODE_TCP,
	RMNET_OFFLOAD_ENGINE_MODE_UDP,
	RMNET_OFFLOAD_ENGINE_MODE_NONE,
	RMNET_OFFLOAD_ENGINE_MODE_MAX = RMNET_OFFLOAD_ENGINE_MODE_NONE,
};

struct rmnet_offload_flow {
	/* Lists */
	struct hlist_node rof_flow_list;
	struct list_head rof_pkts;

	/* Flow header information */
	struct rmnet_offload_header_info rof_hdrs;

	/* 5 tuple hash key */
	u32 rof_hash_key;

	/* Total data length */
	u16 rof_len;

	/* TCP sequence number */
	u32 rof_tcp_seq;

	/* GSO segment size */
	u16 rof_gso_len;

	/* Number of packets in the flow */
	u8 rof_pkts_held;
};

struct rmnet_offload_engine_state {
	struct rmnet_offload_flow roe_flow_pool[RMNET_OFFLOAD_ENGINE_NUM_FLOWS];
	u8 roe_nodes_used;
	u8 roe_recycle_idx;
};

void rmnet_offload_engine_enable_chain_flush(void);
void rmnet_offload_engine_disable_chain_flush(void);
int rmnet_offload_engine_mode_change(u64 old_mode, u64 new_mode);
void rmnet_offload_engine_flush_flow(struct rmnet_offload_flow *flow,
				     struct list_head *flush_list);
void rmnet_offload_engine_flush_by_hash(u32 hash_val,
					struct list_head *flush_list);
u32 rmnet_offload_engine_flush_all_flows(struct list_head *flush_list);
void rmnet_offload_engine_add_flow_pkt(struct rmnet_offload_flow *flow,
				       struct rmnet_offload_info *pkt);
bool rmnet_offload_engine_ingress(struct rmnet_offload_info *pkt,
				  struct list_head *flush_list);
void rmnet_offload_engine_exit(void);
int rmnet_offload_engine_init(void);

#endif
