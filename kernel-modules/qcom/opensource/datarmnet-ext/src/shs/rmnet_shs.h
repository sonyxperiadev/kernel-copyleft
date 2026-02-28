/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/skbuff.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>

#include "rmnet_shs_wq.h"

#ifndef _RMNET_SHS_H_
#define _RMNET_SHS_H_

#include "rmnet_shs_freq.h"
#include "rmnet_config.h"
#include "rmnet_map.h"
#include "rmnet_shs_wq_genl.h"
#include "rmnet_private.h"
#include "rmnet_handlers.h"
#include "rmnet_trace.h"

#include "qmi_rmnet.h"

#define RMNET_SHS_HT rmnet_shs_ht
#define RMNET_SHS_HT_SIZE 9
#define RMNET_SHS_MAX_SKB_INACTIVE_TSEC 15
#define MAX_SILVER_CORES rmnet_shs_cfg.max_s_cores
#define MAX_CPUS  8
#define PERF_MASK rmnet_shs_cfg.perf_mask
#define NONPERF_MASK rmnet_shs_cfg.non_perf_mask

/* Duration to acquire perf lock for pings (in ms) */
#define PING_PERF_DURATION (200)
#define INVALID_CPU -1

/* RPS mask change's Default core for orphaned CPU flows */
#define MAIN_CORE 0
#define UPDATE_MASK 0xFF
#define MAX_FLOWS 700
#define DEF_LL_CORE 4

#define DEFAULT_PIN_HASH 0x00AAAAAA
/* Different max inactivity based on # of flows */
#define FLOW_LIMIT1 70
#define INACTIVE_TSEC1  8
#define FLOW_LIMIT2 140
#define INACTIVE_TSEC2  2
#define DEF_PHY_CPU 1

/*Bit field Features */
#define TITANIUM_FEAT 1
#define INST_RX_SWTCH_FEAT 2
#define SILVER_BALANCE_FEAT 4
/* Moves Phy core to gold cluster when cpu 1 is unavailable */
#define PHY_GOLD_SWITCH_FEAT 8

#define SHS_TRACE_ERR(...) \
  do { if (rmnet_shs_debug) trace_rmnet_shs_err(__VA_ARGS__); } while (0)

#define SHS_TRACE_HIGH(...) \
  do { if (rmnet_shs_debug) trace_rmnet_shs_high(__VA_ARGS__); } while (0)

#define SHS_TRACE_LOW(...) \
  do { if (rmnet_shs_debug) trace_rmnet_shs_low(__VA_ARGS__); } while (0)

#define RMNET_SHS_MAX_SILVER_CORE_BURST_CAPACITY  204800

#define RMNET_SHS_TCP_COALESCING_RATIO 23 //Heuristic
#define RMNET_SHS_UDP_PPS_LPWR_CPU_UTHRESH 100000
#define RMNET_SHS_UDP_PPS_LPWR_CPU0_UTHRESH 100000

#define RMNET_SHS_TCP_PPS_LPWR_CPU_UTHRESH (80000*RMNET_SHS_TCP_COALESCING_RATIO)

#define RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH 210000
#define RMNET_SHS_TCP_PPS_PERF_CPU_UTHRESH (210000*RMNET_SHS_TCP_COALESCING_RATIO)

//50% of MAX SILVER THRESHOLD
#define RMNET_SHS_UDP_PPS_LPWR_CPU_LTHRESH 0
#define RMNET_SHS_UDP_PPS_PERF_CPU_LTHRESH 40000
#define RMNET_SHS_TCP_PPS_PERF_CPU_LTHRESH (40000*RMNET_SHS_TCP_COALESCING_RATIO)

#define RMNET_SHS_UDP_PPS_HEADROOM 20000
#define RMNET_SHS_GOLD_BALANCING_THRESH (RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH / 2)

struct core_flush_s {
	struct  hrtimer core_timer;
	struct work_struct work;
	struct timespec64 coretime;
	int coresum;
	u8 core;
};

struct rmnet_shs_cfg_s {
	struct	hrtimer hrtimer_shs;
	struct  hrtimer hrtimer_lpm;
	struct  hrtimer hrtimer_wake;
	struct  hrtimer hrtimer_disable_pb_boost;
	struct rmnet_map_dl_ind dl_mrk_ind_cb;
	struct rmnet_map_pb_ind pb_mrk_ind_cb;
	struct qmi_rmnet_ps_ind rmnet_idl_ind_cb;
	struct rmnet_port *port;
	struct  core_flush_s core_flush[MAX_CPUS];
	long num_bytes_parked;
	long num_pkts_parked;
	atomic_long_t num_flows;
	ktime_t lpm_ring;
	struct wakeup_source *ws;
	u16 max_phy_steer;
	u16 feature_mask;
	u8 num_filters;
	u8 is_reg_dl_mrk_ind;
	u8 is_pkt_parked;
	u8 force_flush_state;
	u8 rmnet_shs_init_complete;
	u8 dl_ind_state;
	u8 ban_mask;
	u8 map_mask;
	u8 map_len;
	/*Target phy CPU*/
	u8 phy_tcpu;
	u8 phy_old_cpu;
	/* Actual Phy CCU*/
	u8 phy_acpu;
	u8 max_s_cores;
	u8 perf_mask;
	u8 non_perf_mask;
	/* Prevents ht add/del ops while phy flushing */
	u8 kfree_stop;
	u32 cpu_freq_boost_val;
};

struct rmnet_shs_skb_list {
	struct sk_buff *head;
	struct sk_buff *tail;
	u64 num_parked_bytes;
	u32 num_parked_skbs;
	u32 skb_load;
};

struct rmnet_shs_skbn_s {
	union {
		struct iphdr   v4hdr;
		struct ipv6hdr v6hdr;
	} ip_hdr;
	union {
		struct tcphdr tp;
		struct udphdr up;
	} trans_hdr;
	struct list_head node_id;
	/*list head for per cpu flow table*/
	struct net_device *dev;
	struct rmnet_shs_wq_hstat_s *hstats;
	/*stats meta data*/
	struct rmnet_shs_skb_list skb_list;
	/*list to park packets*/
	struct hlist_node list;
	/*list head for hash table*/
	u64 num_skb;
	/* num segments of skbs received*/
	u64 num_coal_skb;
	/* num coalesced skbs received*/
	u64 num_skb_bytes;
	/* num bytes received*/
	u64 hw_coal_bytes;
	/* bytes coalesced in HW */
	u64 hw_coal_bufsize;
	u32 num_ll_skb;
	/* coalescing buffer size in HW */
	u32 queue_head;
	/* n/w stack CPU pkt processing queue head */
	u32 hash;
	/*incoming hash*/
	u32 bif;
	/*bytes in flight*/
	u32 ack_thresh;
	/*quickack threshold*/
	u16 map_index;
	/* rps map index assigned*/
	u16 map_cpu;

	u16 custom_map;
	u16 custom_len;
	u8 phy;
	u16 qhead_offset;
	/* rps cpu for this flow*/
	u16 skb_tport_proto;
	/* Transport protocol associated with this flow*/
	u8 is_shs_enabled;
	u8 low_latency;
	u8 ll_flag;
	/*Is SHS enabled for this flow*/
	u8 mux_id;
};

enum rmnet_shs_ll_steer_state_e {
	RMNET_SHS_LL_SAME_CORE_SILVER,
	RMNET_SHS_LL_SILVER_GOLD_NW,
	RMNET_SHS_LL_SAME_CORE_GOLD,
	RMNET_SHS_LL_SPLIT_ALWAYS,
	RMNET_SHS_LL_STEER_MAX
};

enum rmnet_shs_low_latency_state_e {
	RMNET_SHS_NOT_LOW_LATENCY,
	RMNET_SHS_LOW_LATENCY_MATCH,
	RMNET_SHS_LOW_LATENCY_CHECK,
	RMNET_SHS_LL_MAX_STATE
};

enum rmnet_shs_tmr_force_flush_state_e {
	RMNET_SHS_FLUSH_OFF,
	RMNET_SHS_FLUSH_ON,
	RMNET_SHS_FLUSH_DONE
};

enum rmnet_shs_switch_reason_e {
	RMNET_SHS_SWITCH_INSTANT_RATE,
	RMNET_SHS_SWITCH_WQ_RATE,
	RMNET_SHS_OOO_PACKET_SWITCH,
	RMNET_SHS_OOO_PACKET_TOTAL,

	RMNET_SHS_SWITCH_PACKET_BURST,
	RMNET_SHS_SWITCH_CORE_BACKLOG,
	RMNET_SHS_PHY_SWITCH_GOLD_TO_S,
	RMNET_SHS_PHY_SWITCH_SILVER_TO_G,

	RMNET_SHS_PHY_SWITCH_GOLD_TO_S_ACT,
	RMNET_SHS_PHY_SWITCH_SILVER_TO_G_ACT,
	RMNET_SHS_BANNED_CPU_SUGG,
	RMNET_SHS_WQ_FAIL_PHY_DROP,
/* If no interleaved packets come in after phy switch we would be in this state from not moving node->mapcpu */
	RMNET_SHS_PHY_NO_INTERL_QMAP_FF,
	RMNET_SHS_RESERVED_CPU_SUGG,
	RMNET_SHS_RESERVED_PHY_SUGG,
	RMNET_SHS_DUP_SUGG_R2G,

	RMNET_SHS_RESERVED_PHY_MOVE,
	RMNET_SHS_SUGG_R2G_FAIL1,
	RMNET_SHS_SUGG_R2S_FAIL1,
	RMNET_SHS_RM2G_G2G_SWITCH,

	RMNET_SHS_WALT_SWITCH1,
	RMNET_SHS_WALT_SWITCH2,
	RMNET_SHS_CPU_OFFLINE,
	RMNET_SHS_HALT_PHY,

	RMNET_SHS_HALT_MASK_CHANGE,

	RMNET_SHS_SWITCH_MAX_REASON
};

enum rmnet_shs_dl_ind_state {
	RMNET_SHS_HDR_PENDING,
	RMNET_SHS_END_PENDING,
	RMNET_SHS_IND_COMPLETE,
	RMNET_SHS_DL_IND_MAX_STATE
};

enum rmnet_shs_mid_err_e {
	RMNET_SHS_PING_UNOPTIMIZED,
	RMNET_SHS_MALFORM_MOVE,
	RMNET_SHS_SUGG_FAIL1,
	RMNET_SHS_SUGG_FAIL2,

	RMNET_SHS_MID_ERR_MAX
};

enum rmnet_shs_crit_err_e {
	RMNET_SHS_NETDEV_ERR,
	RMNET_SHS_INVALID_CPU_ERR,
	RMNET_SHS_MAIN_SHS_NOT_REQD,
	RMNET_SHS_MAIN_SHS_RPS_INIT_ERR,

	RMNET_SHS_MAIN_MALLOC_ERR,
	RMNET_SHS_MAIN_MAP_LEN_INVALID,
	RMNET_SHS_MAX_FLOWS,
	RMNET_SHS_WQ_ALLOC_WQ_ERR,

	RMNET_SHS_WQ_ALLOC_DEL_WQ_ERR,
	RMNET_SHS_WQ_ALLOC_HSTAT_ERR,
	RMNET_SHS_WQ_ALLOC_EP_TBL_ERR,
	RMNET_SHS_WQ_GET_RMNET_PORT_ERR,

	RMNET_SHS_WQ_EP_ACCESS_ERR,
	RMNET_SHS_WQ_COMSUME_PKTS,
	RMNET_SHS_CPU_PKTLEN_ERR,
	RMNET_SHS_NULL_SKB_HEAD,

	RMNET_SHS_RPS_MASK_CHANGE,
	RMNET_SHS_WQ_INVALID_CPU_ERR,
	RMNET_SHS_WQ_INVALID_PTR_ERR,
	RMNET_SHS_WQ_NODE_MALLOC_ERR,

	RMNET_SHS_WQ_NL_SOCKET_ERR,
	RMNET_SHS_CPU_FLOWS_BNDS_ERR,
	RMNET_SHS_OUT_OF_MEM_ERR,
	RMNET_SHS_UDP_SEGMENT,

	RMNET_SHS_PHY_OOO_SWITCH,
	RMNET_SHS_FAILED_RPS_CHANGE,
	RMNET_SHS_PHY_ON_TCPU,
	RMNET_SHS_PHY_LONG_STEER,

	RMNET_SHS_PHY_INVALID_STATE2,
	RMNET_SHS_PHY_INVALID_STATE3,
	RMNET_SHS_PHY_INVALID_STATE4,
	RMNET_SHS_DL_MKR_SEQ_OFO,

	RMNET_SHS_MAX_LL_FILTERS,
	RMNET_SHS_RESERVE_CPU,
	RMNET_SHS_RESERVE_LIMIT,

	RMNET_SHS_CRIT_ERR_MAX
};

enum rmnet_shs_ff_reason_e {
	RMNET_SHS_REG_NOT_FORCE_FLUSH,
	/* PHY specific FFs */
	RMNET_SHS_FF_PHY_PKT_LIMIT_ETC,
	RMNET_SHS_FF_PHY_INVALID,
	RMNET_SHS_FF_PHY_REG,
	/* GLOBAL specific FFs */
	RMNET_SHS_FF_GLOBAL,
	RMNET_SHS_FF_PKT_LIMIT,
	RMNET_SHS_FF_BYTE_LIMIT,
	RMNET_SHS_FF_CORE_FLUSH,
	RMNET_SHS_FF_BAD_RPS,
	RMNET_SHS_FF_MAX_REASON
};

enum rmnet_shs_flush_reason_e {
	RMNET_SHS_FLUSH_PHY_PKT_LIMIT,
	RMNET_SHS_FLUSH_PKT_LIMIT,
	RMNET_SHS_FLUSH_BYTE_LIMIT,
	RMNET_SHS_FLUSH_TIMER_EXPIRY,

	RMNET_SHS_FLUSH_RX_DL_TRAILER,
	RMNET_SHS_FLUSH_INV_DL_IND,
	RMNET_SHS_FLUSH_WQ_FB_FLUSH,
	RMNET_SHS_FLUSH_WQ_CORE_FLUSH,

	RMNET_SHS_FLUSH_PSH_PKT_FLUSH,
	RMNET_SHS_FLUSH_PHY_FLUSH,
	RMNET_SHS_FLUSH_PHY_FF_FLUSH,
	RMNET_SHS_FLUSH_PHY_WQ_FLUSH,

	RMNET_SHS_FLUSH_Z_QUEUE_FLUSH,
	RMNET_SHS_FLUSH_INV_DL_IND2,
	RMNET_SHS_FLUSH_MAX_REASON
};

struct flow_buff {
	struct sk_buff *skb;
	struct flow_buff *next;
};

struct rmnet_shs_flush_work {
	struct work_struct work;
	struct rmnet_port *port;
};

struct rmnet_shs_cpu_node_s {
	struct list_head node_list_id;
	u32 qhead;
	u32 qtail;
	u32 qdiff;
	u32 parkedlen;
	u32 seg;
	u8 prio;
	u8 wqprio;
	u8 async;
};

enum rmnet_shs_trace_func {
	RMNET_SHS_MODULE,
	RMNET_SHS_CPU_NODE,
	RMNET_SHS_SKB_STAMPING,
	RMNET_SHS_SKB_CAN_GRO,
	RMNET_SHS_DELIVER_SKB,
	RMNET_SHS_CORE_CFG,
	RMNET_SHS_HASH_MAP,
	RMNET_SHS_ASSIGN,
	RMNET_SHS_FLUSH,
	RMNET_SHS_DL_MRK,
	RMNET_SHS_PB_BOOST_CPU,
	RMNET_SHS_WALT,
};

enum rmnet_shs_flush_context {
	RMNET_RX_CTXT,
	RMNET_WQ_CTXT,
	RMNET_MAX_CTXT
};

/* Trace events and functions */
enum rmnet_shs_trace_evt {
	RMNET_SHS_MODULE_INIT,
	RMNET_SHS_MODULE_INIT_WQ,
	RMNET_SHS_MODULE_GOING_DOWN,
	RMNET_SHS_MODULE_EXIT,
	RMNET_SHS_CPU_NODE_FUNC_START,
	RMNET_SHS_CPU_NODE_FUNC_ADD,
	RMNET_SHS_CPU_NODE_FUNC_MOVE,
	RMNET_SHS_CPU_NODE_FUNC_REMOVE,
	RMNET_SHS_CPU_NODE_FUNC_END,
	RMNET_SHS_SKB_STAMPING_START,
	RMNET_SHS_SKB_STAMPING_END,
	RMNET_SHS_SKB_CAN_GRO_START,
	RMNET_SHS_SKB_CAN_GRO_END,
	RMNET_SHS_DELIVER_SKB_START,
	RMNET_SHS_DELIVER_SKB_END,
	RMNET_SHS_CORE_CFG_START,
	RMNET_SHS_CORE_CFG_NUM_LO_CORES,
	RMNET_SHS_CORE_CFG_NUM_HI_CORES,
	RMNET_SHS_CORE_CFG_CHK_HI_CPU,
	RMNET_SHS_CORE_CFG_CHK_LO_CPU,
	RMNET_SHS_CORE_CFG_GET_QHEAD,
	RMNET_SHS_CORE_CFG_GET_QTAIL,
	RMNET_SHS_CORE_CFG_GET_CPU_PROC_PARAMS,
	RMNET_SHS_CORE_CFG_END,
	RMNET_SHS_HASH_MAP_START,
	RMNET_SHS_HASH_MAP_IDX_TO_STAMP,
	RMNET_SHS_HASH_MAP_FORM_HASH,
	RMNET_SHS_HASH_MAP_END,
	RMNET_SHS_ASSIGN_START,
	RMNET_SHS_ASSIGN_GET_NEW_FLOW_CPU,
	RMNET_SHS_ASSIGN_MATCH_FLOW_NODE_START,
	RMNET_SHS_ASSIGN_MATCH_FLOW_COMPLETE,
	RMNET_SHS_ASSIGN_PARK_PKT_COMPLETE,
	RMNET_SHS_ASSIGN_PARK_TMR_START,
	RMNET_SHS_ASSIGN_PARK_TMR_CANCEL,
	RMNET_SHS_ASSIGN_MASK_CHNG,
	RMNET_SHS_ASSIGN_CRIT_ERROR_NO_MSK_SET,
	RMNET_SHS_ASSIGN_CRIT_ERROR_NO_SHS_REQD,
	RMNET_SHS_ASSIGN_END,
	RMNET_SHS_FLUSH_START,
	RMNET_SHS_FLUSH_PARK_TMR_EXPIRY,
	RMNET_SHS_FLUSH_PARK_TMR_RESTART,
	RMNET_SHS_FLUSH_DELAY_WQ_TRIGGER,
	RMNET_SHS_FLUSH_DELAY_WQ_START,
	RMNET_SHS_FLUSH_DELAY_WQ_END,
	RMNET_SHS_FLUSH_FORCE_TRIGGER,
	RMNET_SHS_FLUSH_BYTE_LIMIT_TRIGGER,
	RMNET_SHS_FLUSH_PKT_LIMIT_TRIGGER,
	RMNET_SHS_FLUSH_DL_MRK_TRLR_HDLR_START,
	RMNET_SHS_FLUSH_DL_MRK_TRLR_HDLR_END,
	RMNET_SHS_FLUSH_CHK_AND_FLUSH_NODE_START,
	RMNET_SHS_FLUSH_NODE_START,
	RMNET_SHS_FLUSH_CHK_NODE_CAN_FLUSH,
	RMNET_SHS_FLUSH_NODE_CORE_SWITCH,
	RMNET_SHS_FLUSH_NODE_END,
	RMNET_SHS_FLUSH_CHK_AND_FLUSH_NODE_END,
	RMNET_SHS_FLUSH_END,
	RMNET_SHS_DL_MRK_START,
	RMNET_SHS_DL_MRK_HDR_HDLR_START,
	RMNET_SHS_DL_MRK_HDR_HDLR_END,
	RMNET_SHS_DL_MRK_TRLR_START,
	RMNET_SHS_DL_MRK_TRLR_HDLR_END,
	RMNET_SHS_DL_MRK_TRLR_END,
	RMNET_SHS_DL_MRK_END,
	RMNET_SHS_PB_BOOST_CPU_ENTER,
	RMNET_SHS_PB_BOOST_CPU_UPDATE,
	RMNET_SHS_PB_BOOST_CPU_RESET,
	RMNET_SHS_WALT_TRANSITION,
};

extern struct rmnet_shs_flush_work shs_delayed_work;
extern spinlock_t rmnet_shs_ll_ht_splock;

extern spinlock_t rmnet_shs_ht_splock;
extern spinlock_t rmnet_shs_ep_lock;
extern spinlock_t rmnet_shs_hstat_tbl_lock;
extern struct hlist_head RMNET_SHS_HT[1 << (RMNET_SHS_HT_SIZE)];

void rmnet_shs_skb_entry_disable(void);
void rmnet_shs_skb_entry_enable(void);
void rmnet_shs_switch_disable(void);
void rmnet_shs_switch_enable(void);
int rmnet_shs_is_lpwr_cpu(u16 cpu);
void rmnet_shs_cancel_table(void);
void rmnet_shs_rx_wq_init(void);
unsigned int rmnet_shs_rx_wq_exit(void);
int rmnet_shs_get_mask_len(u8 mask);

int rmnet_shs_chk_and_flush_node(struct rmnet_shs_skbn_s *node,
				 u8 force_flush, u8 ctxt, struct sk_buff **phy_list);
void rmnet_shs_pb_hdr_handler(struct rmnet_map_pb_ind_hdr *pbhdr);
void rmnet_shs_dl_hdr_handler_v2(struct rmnet_map_dl_ind_hdr *dlhdr,
			      struct rmnet_map_control_command_header *qcmd);
void rmnet_shs_dl_trl_handler_v2(struct rmnet_map_dl_ind_trl *dltrl,
			      struct rmnet_map_control_command_header *qcmd);
void rmnet_shs_dl_hdr_handler(struct rmnet_map_dl_ind_hdr *dlhdr);
void rmnet_shs_dl_trl_handler(struct rmnet_map_dl_ind_trl *dltrl);
int rmnet_shs_assign(struct sk_buff *skb, struct rmnet_shs_clnt_s *cfg);
void rmnet_shs_flush_table(u8 is_force_flush, u8 ctxt);
void rmnet_shs_cpu_node_remove(struct rmnet_shs_skbn_s *node);
void rmnet_shs_init(struct net_device *dev, struct net_device *vnd);
void rmnet_shs_exit(unsigned int cpu_switch);
void rmnet_shs_ps_on_hdlr(void *port);
void rmnet_shs_ps_off_hdlr(void *port);
void rmnet_shs_update_cpu_proc_q_all_cpus(void);
void rmnet_shs_clear_node(struct rmnet_shs_skbn_s *node, u8 ctxt);
void rmnet_shs_change_cpu_num_flows(u16 map_cpu, bool inc);

void rmnet_shs_deliver_skb(struct sk_buff *skb);

u32 rmnet_shs_get_cpu_qhead(u8 cpu_num);
#endif /* _RMNET_SHS_H_ */
