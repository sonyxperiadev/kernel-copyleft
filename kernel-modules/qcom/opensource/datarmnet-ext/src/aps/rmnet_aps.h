/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _RMNET_APS_H_
#define _RMNET_APS_H_

#include <linux/skbuff.h>
#include <net/genetlink.h>

#ifdef RMNET_APS_DEBUG
#define aps_log(...) pr_err(__VA_ARGS__)
#else
#define aps_log(...)
#endif

struct rmnet_aps_flow_req {
	u32 cmd;
	u32 label;
	u32 duration;
	u32 ifindex;
	u8 aps_prio;
	u8 use_llc;
	u8 use_llb;
	u8 reserved;
};

struct rmnet_aps_flow_resp {
	u32 cmd;
	u32 cmd_data;
	u32 label;
};

#define FILTER_MASK_SADDR 1
#define FILTER_MASK_DADDR 2
struct rmnet_aps_filter_req {
	u32 cmd;
	u32 label;
	u32 ifindex;
	s32 ip_type;
	__be32 saddr[4];
	__be32 daddr[4];
	u16 sport;
	u16 dport;
	u32 flow_label;
	u8 tos;
	u8 tos_mask;
	u8 l4_proto;
	u8 filter_masks;
	u8 reserved[68];
};

struct rmnet_aps_filter_resp {
	u32 cmd;
	u32 cmd_data;
	u32 label;
};

struct rmnet_aps_pdn_config_req {
	u32 ifindex;
	u64 apn_mask;
	u32 expire_ms;
	u32 reserved[8];
};

struct rmnet_aps_pdn_config_resp {
	u32 ifindex;
	u32 reserved[7];
};

struct rmnet_aps_data_report {
	u8 mux_id;
	u8 type;
	u8 sum_all_bearers;
	u8 len;
	u32 value[8];
};

int rmnet_aps_genl_flow_hdlr(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_aps_genl_pdn_config_hdlr(struct sk_buff *skb_2,
				   struct genl_info *info);
int rmnet_aps_genl_filter_hdlr(struct sk_buff *skb_2, struct genl_info *info);
int rmnet_aps_genl_data_report_hdlr(struct sk_buff *skb_2,
				    struct genl_info *info);

#endif /* _RMNET_APS_H_ */
