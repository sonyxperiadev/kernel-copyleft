/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _RMNET_SHS_WQ_MEM_H_
#define _RMNET_SHS_WQ_MEM_H_

#include "rmnet_shs.h"
#include "rmnet_shs_config.h"

/* Shared memory files */
#define RMNET_SHS_PROC_DIR      "shs"
#define RMNET_SHS_PROC_CAPS     "rmnet_shs_caps"
#define RMNET_SHS_PROC_G_FLOWS  "rmnet_shs_flows"
#define RMNET_SHS_PROC_SS_FLOWS "rmnet_shs_ss_flows"
#define RMNET_SHS_PROC_FFLOWS   "rmnet_shs_fflows"
#define RMNET_SHS_PROC_LL_FLOWS   "rmnet_shs_ll_flows"
#define RMNET_SHS_PROC_NETDEV   "rmnet_shs_netdev"

#define RMNET_SHS_NUM_TOP_FFLOWS (30)
#define RMNET_SHS_NUM_TOP_LL_FLOWS (RMNET_SHS_NUM_TOP_FFLOWS)

#define RMNET_SHS_MAX_USRFLOWS (100)
#define RMNET_SHS_MAX_NETDEVS (40)
#define RMNET_SHS_IFNAMSIZ (16)
#define RMNET_SHS_READ_VAL (0)

/* NOTE: Make sure these structs fit in one page */
/* 26 bytes * 8 max cpus = 208 bytes < 4096 */
struct __attribute__((__packed__)) rmnet_shs_wq_cpu_cap_usr_s {
	u64 pps_capacity;
	u64 avg_pps_capacity;
	u64 bps;
	u16 cpu_num;
	u8 perf_mask;
};

struct __attribute__((__packed__)) rmnet_shs_wq_additional_stats_s {
	/* Stats from include/net/netns/ipv4.h => struct netns_ipv4 */
	int ipv4_tcp_rmem[3]; /* init_net.ipv4.sysctl_tcp_rmem[] */
	int ipv4_tcp_wmem[3]; /* init_net.ipv4.sysctl_tcp_wmem[] */
};

/* 30 bytes * 128 max = 3840 bytes < 4096 */
struct __attribute__((__packed__)) rmnet_shs_wq_gflows_usr_s {
	u64 rx_pps;
	u64 avg_pps;
	u64 rx_bps;
	u32 hash;
	u16 cpu_num;
};

/* 38 bytes * 100 max = 3800 bytes < 4096 */
struct __attribute__((__packed__)) rmnet_shs_wq_ssflows_usr_s {
	u64 rx_pps;
	u64 avg_pps;
	u64 rx_bps;
	u32 hash;
	u32 bif; /* Bytes in flight */
	u32 ack_thresh;
	u16 cpu_num;
};

/* 30 max < 4096 */
struct __attribute__((__packed__)) rmnet_shs_wq_ll_flows_usr_s {
	union {
		struct iphdr   v4hdr;
		struct ipv6hdr v6hdr;
	} ip_hdr;
	union {
		struct tcphdr tp;
		struct udphdr up;
	} trans_hdr;
	u64 rx_pps;
	u64 avg_pps;
	u64 rx_bps;
	u64 avg_segs;
	u64 hw_coal_bytes_diff;
	u64 hw_coal_bufsize_diff;
	u32 hash;
	u16 cpu_num;
	u16 trans_proto;
	u8  mux_id;
	u8  ll_pipe;
};

/* 30 max < 4096 */
struct __attribute__((__packed__)) rmnet_shs_wq_fflows_usr_s {
	union {
		struct iphdr   v4hdr;
		struct ipv6hdr v6hdr;
	} ip_hdr;
	union {
		struct tcphdr tp;
		struct udphdr up;
	} trans_hdr;
	u64 rx_pps;
	u64 avg_pps;
	u64 rx_bps;
	u64 avg_segs;
	u64 hw_coal_bytes_diff;
	u64 hw_coal_bufsize_diff;
	u32 hash;
	u16 cpu_num;
	u16 trans_proto;
	u8  mux_id;
};

/* 16 + 8*10 + 1 = 97 bytes, 97*40 netdev = 3880 bytes < 4096 */
struct __attribute__((__packed__)) rmnet_shs_wq_netdev_usr_s {
	char name[RMNET_SHS_IFNAMSIZ];
	u64  coal_ip_miss;
	u64  hw_evict;
	u64  coal_rx_pkts;
	u64  coal_tcp;
	u64  coal_tcp_bytes;
	u64  coal_udp;
	u64  coal_udp_bytes;
	u64  udp_rx_bps;
	u64  tcp_rx_bps;
	u64  pb_marker_seq;
	u8   mux_id;
};

extern struct list_head gflows;
extern struct list_head ssflows;
extern struct list_head fflows;
extern struct list_head ll_flows;

extern struct list_head cpu_caps;

/* Buffer size for read and write syscalls */
enum {RMNET_SHS_BUFFER_SIZE = 4096};

struct rmnet_shs_mmap_info {
	char *data;
	refcount_t refcnt;
};

/* Function Definitions */

void rmnet_shs_wq_ssflow_list_add(struct rmnet_shs_wq_hstat_s *hnode,
				  struct list_head *ss_flows);
void rmnet_shs_wq_gflow_list_add(struct rmnet_shs_wq_hstat_s *hnode,
				 struct list_head *gold_flows);
void rmnet_shs_wq_fflow_list_add(struct rmnet_shs_wq_hstat_s *hnode,
				  struct list_head *fflows);
void rmnet_shs_wq_ll_flow_list_add(struct rmnet_shs_wq_hstat_s *hnode,
				 struct list_head *ll_flows);



void rmnet_shs_wq_cleanup_gold_flow_list(struct list_head *gold_flows);
void rmnet_shs_wq_cleanup_ss_flow_list(struct list_head *ss_flows);
void rmnet_shs_wq_cleanup_fflow_list(struct list_head *fflows);
void rmnet_shs_wq_cleanup_ll_flow_list(struct list_head *ll_flows);


void rmnet_shs_wq_cpu_caps_list_add(
				struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p,
				struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_node,
				struct list_head *cpu_caps);

void rmnet_shs_wq_cleanup_cpu_caps_list(struct list_head *cpu_caps);

void rmnet_shs_wq_mem_update_cached_cpu_caps(struct list_head *cpu_caps);

void rmnet_shs_wq_mem_update_cached_sorted_gold_flows(struct list_head *gold_flows);
void rmnet_shs_wq_mem_update_cached_sorted_ss_flows(struct list_head *ss_flows);
void rmnet_shs_wq_mem_update_cached_sorted_fflows(struct list_head *fflows);
void rmnet_shs_wq_mem_update_cached_sorted_ll_flows(struct list_head *ll_flows);
void rmnet_shs_wq_mem_update_cached_netdevs(void);

void rmnet_shs_wq_mem_init(void);

void rmnet_shs_wq_mem_deinit(void);

#endif /*_RMNET_SHS_WQ_GENL_H_*/
