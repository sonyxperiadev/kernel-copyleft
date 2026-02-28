/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "rmnet_shs.h"

#ifndef _RMNET_SHS_WQ_GENL_H_
#define _RMNET_SHS_WQ_GENL_H_

#include <net/genetlink.h>

/* Generic Netlink Definitions */
#define RMNET_SHS_GENL_VERSION 1
#define RMNET_SHS_GENL_FAMILY_NAME "RMNET_SHS"
#define RMNET_SHS_MOVE_FAIL_RESP_INT 720
#define RMNET_SHS_MOVE_PASS_RESP_INT 727

#define RMNET_SHS_SYNC_RESP_INT      828
#define RMNET_SHS_SYNC_RESP_INT_LPM_DISABLE      829

#define RMNET_SHS_RMNET_MOVE_DONE_RESP_INT 300
#define RMNET_SHS_RMNET_MOVE_FAIL_RESP_INT 301

#define RMNET_SHS_RMNET_DMA_RESP_INT 400

#define RMNET_SHS_SEG_FAIL_RESP_INT  920
#define RMNET_SHS_SEG_SET_RESP_INT   929

#define RMNET_SHS_BOOT_FAIL_RESP_INT  530
#define RMNET_SHS_BOOT_SET_RESP_INT   539

#define RMNET_SHS_QUICKACK_FAIL_RESP_INT  930
#define RMNET_SHS_QUICKACK_SET_RESP_INT   931
#define RMNET_SHS_SYNC_WQ_EXIT        42

#define MAXCPU 8
extern int rmnet_shs_userspace_connected;

enum {
	RMNET_SHS_GENL_CMD_UNSPEC,
	RMNET_SHS_GENL_CMD_INIT_SHSUSRD,
	RMNET_SHS_GENL_CMD_TRY_TO_MOVE_FLOW,
	RMNET_SHS_GENL_CMD_SET_FLOW_SEGMENTATION,
	RMNET_SHS_GENL_CMD_MEM_SYNC,
	RMNET_SHS_GENL_CMD_LL_FLOW,
	RMNET_SHS_GENL_CMD_QUICKACK,
	RMNET_SHS_GENL_CMD_BOOTUP,
	__RMNET_SHS_GENL_CMD_MAX,
};

enum {
	RMNET_SHS_GENL_ATTR_UNSPEC,
	RMNET_SHS_GENL_ATTR_STR,
	RMNET_SHS_GENL_ATTR_INT,
	RMNET_SHS_GENL_ATTR_SUGG,
	RMNET_SHS_GENL_ATTR_SEG,
	RMNET_SHS_GENL_ATTR_FLOW,
	RMNET_SHS_GENL_ATTR_QUICKACK,
	RMNET_SHS_GENL_ATTR_BOOTUP,
	__RMNET_SHS_GENL_ATTR_MAX,
};
#define RMNET_SHS_GENL_ATTR_MAX (__RMNET_SHS_GENL_ATTR_MAX - 1)

struct rmnet_shs_bootup_info {
	uint32_t feature_mask;
	uint8_t non_perf_mask;
	/*Scaled by 1k on recv */
	uint32_t rx_min_pps_thresh[MAXCPU];
	uint32_t rx_max_pps_thresh[MAXCPU];
	uint32_t cpu_freq_boost_val;
};

struct rmnet_shs_wq_sugg_info {
	uint32_t hash_to_move;
	uint32_t sugg_type;
	uint16_t cur_cpu;
	uint16_t dest_cpu;
};

struct rmnet_shs_wq_seg_info {
	uint32_t hash_to_set;
	uint32_t segs_per_skb;
};

struct rmnet_shs_wq_quickack_info {
	uint32_t hash_to_set;
	uint32_t ack_thresh;
};

struct rmnet_shs_phy_change_payload {
	uint8_t  old_cpu;
	uint8_t  new_cpu;
};

/* rmnet_shs to shsusrd message channel */
#define RMNET_SHS_GENL_MSG_FAMILY_NAME "RMNET_SHS_MSG"

/* Command Types */
enum {
    RMNET_SHS_GENL_MSG_CMD_UNSPEC,
    RMNET_SHS_GENL_MSG_WAIT_CMD,
    __RMNET_SHS_GENL_MSG_CMD_MAX,
};

/* Attribute Types */
enum {
    RMNET_SHS_GENL_MSG_ATTR_UNSPEC,
    RMNET_SHS_GENL_MSG_ATTR_REQ,
    RMNET_SHS_GENL_MSG_ATTR_RESP,
    __RMNET_SHS_GENL_MSG_ATTR_MAX,
};

#define RMNET_SHS_MSG_PAYLOAD_SIZE (98)
#define RMNET_SHS_GENL_MSG_MAX     (1)

struct rmnet_shs_ping_boost_payload {
    uint32_t perf_duration; /* Duration to acquire perf lock */
    uint8_t  perf_acq;      /* Set to 1 to aquire */
};

struct rmnet_shs_pause_payload {
    uint8_t seq; /* Duration to acquire perf lock */
};

enum {
    RMNET_SHS_GENL_MSG_NOP = 0,           /* 0 = No-operation */
    RMNET_SHS_GENL_PING_BOOST_MSG = 1,    /* 1 = Ping boost request */
    RMNET_SHS_GENL_PHY_CHANGE_MSG = 2,    /* 2 = Phy change request */
    RMNET_SHS_GENL_TRAFFIC_PAUSE_MSG = 3, /* 3 = Phy change request */

};

struct rmnet_shs_msg_info {
	char     payload[RMNET_SHS_MSG_PAYLOAD_SIZE];
	uint16_t msg_type;
};

struct rmnet_shs_msg_req {
	int valid;
};

struct rmnet_shs_msg_resp {
	struct rmnet_shs_msg_info list[RMNET_SHS_GENL_MSG_MAX];
	uint64_t timestamp;
	uint16_t list_len;
	uint8_t  valid;
};

struct rmnet_shs_wq_flow_info {
	union {
		__be32	daddr;
		struct in6_addr  v6_daddr;
	} dest_ip_addr;
	union {
		__be32	saddr;
		struct in6_addr  v6_saddr;
	} src_ip_addr;
	u16 src_port;
	u16 src_port_valid;
	u16 dest_port;
	u16 dest_port_valid;
	u8 src_addr_valid;
	u8 dest_addr_valid;
	u8 proto;
	u8 proto_valid;
	u8 ip_version;
	u8 timeout;
	u8 seq;
	u8 opcode;
};

/* Types of suggestions made by shs wq */
enum rmnet_shs_ll_flow_opcode {
	RMNET_SHS_LL_OPCODE_DEL,
	RMNET_SHS_LL_OPCODE_ADD,
	RMNET_SHS_LL_OPCODE_MAX,
};

struct rmnet_shs_wq_flow_node {
	struct list_head filter_head;
	struct hlist_node list;
	struct rmnet_shs_wq_flow_info info;
};

/* Function Prototypes */
int rmnet_shs_genl_dma_init(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_shs_genl_try_to_move_flow(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_shs_genl_set_flow_segmentation(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_shs_genl_mem_sync(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_shs_genl_set_flow_ll(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_shs_genl_set_quickack_thresh(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_shs_genl_set_bootup_config(struct sk_buff *skb_2, struct genl_info *info);

int rmnet_shs_genl_send_int_to_userspace(struct genl_info *info, int val);

int rmnet_shs_genl_send_int_to_userspace_no_info(int val);

int rmnet_shs_genl_send_msg_to_userspace(void);

/* rmnet_shs to shsusrd messaging functionality */
void rmnet_shs_create_ping_boost_msg_resp(uint32_t perf_duration,
					  struct rmnet_shs_msg_resp *msg_resp);
void rmnet_shs_create_pause_msg_resp(uint8_t seq,
				     struct rmnet_shs_msg_resp *msg_resp);

int rmnet_shs_genl_msg_direct_send_to_userspace(struct rmnet_shs_msg_resp *msg_ptr);

/* Handler for message channel to shsusrd */
int rmnet_shs_genl_msg_req_hdlr(struct sk_buff *skb_2,
				struct genl_info *info);

void rmnet_shs_create_phy_msg_resp(struct rmnet_shs_msg_resp *msg_resp,
                                   uint8_t ocpu, uint8_t ncpu);
int rmnet_shs_wq_genl_init(void);

int rmnet_shs_wq_genl_deinit(void);

#endif /*_RMNET_SHS_WQ_GENL_H_*/
