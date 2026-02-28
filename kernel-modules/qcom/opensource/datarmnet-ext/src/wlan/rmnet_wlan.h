/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN handler framework */

#ifndef __RMNET_WLAN_H__
#define __RMNET_WLAN_H__

#include <linux/types.h>
#include <net/genetlink.h>
#include <linux/netdevice.h>
#include <linux/in6.h>

struct rmnet_wlan_tuple {
	union {
	  __be16 port;
	  __be32 spi_val;
	};
	u8 ip_proto;
	u8 trans_proto;
};

struct rmnet_wlan_fwd_info {
	struct net_device *fwd_dev;
	union {
		__be32 v4_addr;
		struct in6_addr v6_addr;
	};
	u8 ip_proto;
	u8 net_type;
};

struct rmnet_wlan_fwd_info_node {
	struct hlist_node hash;
	struct rcu_head rcu;
	struct notifier_block nb;
	char dev_name[IFNAMSIZ];
	struct rmnet_wlan_fwd_info fwd;
};

/* Low Latency Address Info Structure */
struct rmnet_wlan_ll_tuple {
	union {
		__be32 v4_saddr;
		struct in6_addr v6_saddr;
	};
	union {
		__be32 v4_daddr;
		struct in6_addr v6_daddr;
	};
	__be16 sport;
	__be16 dport;
	u8 ip_proto;
};

enum {
	DATA_PATH_PROXY_NET_WLAN,
	DATA_PATH_PROXY_NET_WWAN,
	DATA_PATH_PROXY_NET_LBO,
	__DATA_PATH_PROXY_NET_MAX,
};

rx_handler_result_t rmnet_wlan_rx_handler(struct sk_buff **pskb);

/* TCP clamping api. Clamps mss to 1140 for packets matching the tcp flags */
void rmnet_wlan_tcp_mss_clamp(struct sk_buff *skb, u32 tcp_flags);

/* Pass on an SKB to a FWD device */
int rmnet_wlan_deliver_skb(struct sk_buff *skb,
			   struct rmnet_wlan_fwd_info *fwd_info);

/* Is this tuple present in our list? */
bool rmnet_wlan_tuple_present(struct rmnet_wlan_tuple *tuple);

/* Tuple add/delete interface */
int rmnet_wlan_add_tuples(struct rmnet_wlan_tuple *tuples, u32 tuple_count,
			  struct genl_info *info);
int rmnet_wlan_del_tuples(struct rmnet_wlan_tuple *tuples, u32 tuple_count,
			  struct genl_info *info);
int rmnet_wlan_get_tuples(struct sk_buff **pskb, struct genl_family *fam,
			  struct genl_info *info);

/* Device interface */
int rmnet_wlan_set_device(char *dev_name, struct genl_info *info);
int rmnet_wlan_unset_device(char *dev_name, struct genl_info *info);

int rmnet_wwan_set_device(char *dev_name, struct genl_info *info);
int rmnet_wwan_unset_device(char *dev_name, struct genl_info *info);

/* Forwarding information interface */
int rmnet_wlan_add_fwd_info(struct rmnet_wlan_fwd_info *fwd_info,
			    struct genl_info *info);
int rmnet_wlan_del_fwd_info(struct rmnet_wlan_fwd_info *fwd_info,
			    struct genl_info *info);

/* UDP Encap interface */
int rmnet_wlan_set_encap_port(__be16 port, struct genl_info *info);
int rmnet_wlan_unset_encap_port(__be16 port, struct genl_info *info);
bool rmnet_wlan_udp_encap_check(struct sk_buff *skb,
				struct rmnet_wlan_tuple *tuple,
				int ip_len);
int rmnet_wlan_act_encap_port_pass_through(__be16 port, struct genl_info *info);
int rmnet_wlan_act_encap_port_drop(__be16 port, struct genl_info *info);
bool rmnet_wlan_udp_encap_drop_check(struct rmnet_wlan_tuple *tuple);

/* Pull the plug */
int rmnet_wlan_reset(void);

/* Module teardown */
void rmnet_wlan_deinit(void);

char *rmnet_wlan_get_dev(void);
char *rmnet_wwan_get_dev(void);

struct rmnet_wlan_fwd_info_node * rmnet_wlan_fwd_info_find
				(struct rmnet_wlan_fwd_info *info);

/* Low Latency Tuple Management */
int rmnet_wlan_add_ll_tuple(struct rmnet_wlan_ll_tuple *tuple);
int rmnet_wlan_del_ll_tuple(void);
extern struct rmnet_wlan_ll_tuple * rmnet_wlan_ll_tuple_cache;

int rmnet_wlan_strlcmp(const char *string1, const char *string2,
		       size_t limit_bytes);

#endif
