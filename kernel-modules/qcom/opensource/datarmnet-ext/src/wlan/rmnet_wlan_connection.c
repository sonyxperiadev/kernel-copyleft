// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN connection management framework */

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/hashtable.h>
#include <linux/if_ether.h>
#include <linux/workqueue.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/dst.h>
#include <net/netfilter/nf_conntrack.h>
#include "rmnet_module.h"
#include "rmnet_wlan.h"
#include "rmnet_wlan_connection.h"
#include "rmnet_wlan_stats.h"

/* Time to hold connection entries, in ms. 2 seconds currently */
#define RMNET_WLAN_CONNECTION_TIMEOUT (2000)
/* How often to run the cleaning workqueue while connection info is present,
 * in ms.
 */
#define RMNET_WLAN_CONNECTION_WQ_INTERVAL (500)

#define RMNET_WLAN_CONNECTION_BKTS (16)
#define RMNET_WLAN_CONNECTION_HASH_BITS \
	(const_ilog2(RMNET_WLAN_CONNECTION_BKTS))


struct rmnet_wlan_connection_node {
	struct hlist_node hash;
	struct rcu_head rcu;
	struct rmnet_wlan_connection_info info;
	struct rmnet_wlan_fwd_info *fwd;
	unsigned long ts;
	bool dead;
};

struct rmnet_wlan_connection_work_struct {
	struct delayed_work ws;
	bool force_clean;
};

/* spinlock for connection hashtable protection */
static DEFINE_SPINLOCK(rmnet_wlan_connection_lock);
static DEFINE_HASHTABLE(rmnet_wlan_connection_hash,
			RMNET_WLAN_CONNECTION_HASH_BITS);

/* Thus number of connection objects present in the hash table */
static u32 rmnet_wlan_connection_hash_size;

/* Periodic cleaning work struct for the hashtable */
static struct rmnet_wlan_connection_work_struct rmnet_wlan_connection_work;

static bool
rmnet_wlan_connection_info_match(struct rmnet_wlan_connection_info *i1,
				 struct rmnet_wlan_connection_info *i2)
{
	if (i1->ip_proto != i2->ip_proto)
		return false;

	if (i1->ip_proto == 4)
		return i1->v4_saddr == i2->v4_saddr &&
		       i1->v4_daddr == i2->v4_daddr;

	return !ipv6_addr_cmp(&i1->v6_saddr, &i2->v6_saddr) &&
	       !ipv6_addr_cmp(&i1->v6_daddr, &i2->v6_daddr);
}

static bool
rmnet_wlan_connection_node_expired(struct rmnet_wlan_connection_node *node,
				   unsigned long ts)
{
	unsigned long timeout;

	timeout = msecs_to_jiffies(RMNET_WLAN_CONNECTION_TIMEOUT);
	if (ts - node->ts > timeout)
		return true;

	return false;
}

static bool rmnet_wlan_connection_hash_clean(bool force)
{
	struct rmnet_wlan_connection_node *node;
	struct hlist_node *tmp;
	unsigned long ts;
	int bkt;

	ts = jiffies;
	hash_for_each_safe(rmnet_wlan_connection_hash, bkt, tmp, node, hash) {
		if (node->dead)
			/* Node is already removed, but RCU grace period has
			 * not yet expired.
			 */
			continue;

		if (force || rmnet_wlan_connection_node_expired(node, ts)) {
			node->dead = true;
			hash_del_rcu(&node->hash);
			kfree_rcu(node, rcu);
			rmnet_wlan_connection_hash_size--;
		}
	}

	return !!rmnet_wlan_connection_hash_size;
}

static void rmnet_wlan_connection_work_process(struct work_struct *ws)
{
	struct rmnet_wlan_connection_work_struct *conn_work;
	unsigned long flags;
	bool should_resched;

	conn_work = container_of(to_delayed_work(ws),
				 struct rmnet_wlan_connection_work_struct,
				 ws);
	spin_lock_irqsave(&rmnet_wlan_connection_lock, flags);
	should_resched =
		rmnet_wlan_connection_hash_clean(conn_work->force_clean);
	if (should_resched) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_WLAN_CONNECTION_WQ_INTERVAL);
		schedule_delayed_work(&conn_work->ws, delay);
	}

	spin_unlock_irqrestore(&rmnet_wlan_connection_lock, flags);
}

static rx_handler_result_t rmnet_wlan_receive_skb(struct sk_buff *skb, uint8_t network_type)
{
	/* Only reverse rmnet packets should arrive in this function and match the check */
	if (skb_is_nonlinear(skb) && !skb_headlen(skb)) {
		int header_size = 0;

		if (skb->protocol == htons(ETH_P_IP)) {
			header_size = sizeof(struct iphdr);
		} else if (skb->protocol == htons(ETH_P_IPV6)) {
			header_size = sizeof(struct ipv6hdr);
		} else {
			rmnet_wlan_forward_stats_update(RMNET_F_S_PULL_PROTO_MISMATCH);
			goto drop;
		}

		/* Headroom is already reserved in rmnet core */
		if (!__pskb_pull_tail(skb, header_size)) {
			rmnet_wlan_forward_stats_update(RMNET_F_S_PULL_FAILURE);
			goto drop;
		} else {
			skb_reset_network_header(skb);
			rmnet_wlan_forward_stats_update(RMNET_F_S_PULL_SUCCESS);
		}
	}

	if (skb->dev && (skb->protocol == htons(ETH_P_IP)) &&
	    network_type == DATA_PATH_PROXY_NET_LBO) {
		struct iphdr *iph, __iph;
		struct net_device *wdev = NULL;
		struct flowi4 fl4 = {};
		struct rtable *rt;
		struct neighbour *n;
		int err = 0;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IP_HDRP_FAIL);
			goto drop;
		}

		wdev = dev_get_by_name_rcu(&init_net, rmnet_wlan_get_dev());
		if (!wdev) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IP_DEV_GET_FAIL);
			goto drop;
		}

		skb->dev = wdev;
		memcpy(&fl4.saddr, &iph->saddr, sizeof(__be32));
		memcpy(&fl4.daddr, &iph->daddr, sizeof(__be32));
		fl4.flowi4_oif = wdev->ifindex;
		fl4.flowi4_flags = FLOWI_FLAG_KNOWN_NH;


		rt = ip_route_output_key(&init_net, &fl4);
		if (IS_ERR(rt)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IP_ROUTE_FAIL);
			goto drop;
		}

		n = dst_neigh_lookup(&rt->dst, &fl4.daddr);
		ip_rt_put(rt);
		if (!n) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IP_NEIGH_LOOKUP_FAIL);
			goto drop;
		}

		if (n->dev != skb->dev || !n->dev->header_ops) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IP_HARD_HEADER_FAIL);
			neigh_release(n);
			goto drop;
		}

		err = neigh_resolve_output(n, skb);

		neigh_release(n);

		if (likely(err == NET_XMIT_SUCCESS || err == NET_XMIT_CN)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IP_XMIT_SUCCESS);
		} else {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IP_XMIT_DROP);
		}

		return RX_HANDLER_CONSUMED;
	} else if (skb->dev && (skb->protocol == htons(ETH_P_IPV6)) &&
		  network_type == DATA_PATH_PROXY_NET_LBO) {
		struct ipv6hdr *ip6h, __ip6h;
		struct net_device *wdev = NULL;
		struct flowi6 fl6 = {};
		struct neighbour *n;
		struct dst_entry *dst;
		int err = 0;

		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_HDRP_FAIL);
			goto drop;
		}

		wdev = dev_get_by_name_rcu(&init_net, rmnet_wlan_get_dev());
		if (!wdev) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_DEV_GET_FAIL);
			goto drop;
		}

		skb->dev = wdev;
		memcpy(&fl6.saddr, &ip6h->saddr, sizeof(struct in6_addr));
		memcpy(&fl6.daddr, &ip6h->daddr, sizeof(struct in6_addr));
		fl6.flowi6_oif = wdev->ifindex;
		fl6.flowi6_flags = FLOWI_FLAG_KNOWN_NH;


		dst = ipv6_stub->ipv6_dst_lookup_flow(&init_net, NULL, &fl6, NULL);
		if (IS_ERR(dst)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_ROUTE_FAIL);
			goto drop;
		}

		n = dst_neigh_lookup(dst, &fl6.daddr);
		dst_release(dst);
		if (!n) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_NEIGH_LOOKUP_FAIL);
			goto drop;
		}

		if (n->dev != skb->dev || !n->dev->header_ops) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_HARD_HEADER_FAIL);
			neigh_release(n);
			goto drop;
		}

		err = neigh_resolve_output(n, skb);

		neigh_release(n);

		if (likely(err == NET_XMIT_SUCCESS || err == NET_XMIT_CN)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_XMIT_SUCCESS);
		} else {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_XMIT_DROP);
		}

		return RX_HANDLER_CONSUMED;
	} else if(skb->dev && (skb->protocol == htons(ETH_P_IP)) &&
	          network_type == DATA_PATH_PROXY_NET_WWAN) {
		/* Use xfrm to route packet to rmnet data */
		struct iphdr *iph, __iph;
		struct net_device *wdev = NULL;
		struct flowi4 fl4 = {};
		struct dst_entry *dst_xfrm;
		struct rtable *rt;
		struct net_device *ddev = NULL;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph) {
			rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_R0_IP_HDRP_FAIL);
			goto drop;
		}

		wdev = dev_get_by_name_rcu(&init_net, rmnet_wwan_get_dev());
		if (!wdev) {
			rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_R0_IP_DEV_GET_FAIL);
			goto drop;
		}

		memcpy(&fl4.daddr, &iph->daddr, sizeof(__be32));
		fl4.flowi4_oif = wdev->ifindex;
		fl4.flowi4_flags = FLOWI_FLAG_KNOWN_NH;

		rt = ip_route_output_key(&init_net, &fl4);
		if (IS_ERR(rt)) {
			rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_R0_IP_ROUTE_FAIL);

			ddev = dev_get_by_name_rcu(&init_net, "dummy0");
			if (!ddev) {
				rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_R0_IP_DDEV_GET_FAIL);
				goto drop;
			}

			fl4.flowi4_oif = ddev->ifindex;
			fl4.flowi4_flags = FLOWI_FLAG_KNOWN_NH;

			rt = ip_route_output_key(&init_net, &fl4);
			if (IS_ERR(rt)) {
				rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IP_DRT_LOOKUP_FAIL);
				goto drop;
			}
		}
		memcpy(&fl4.saddr, &iph->saddr, sizeof(__be32));
		dst_xfrm = xfrm_lookup(&init_net, &rt->dst, flowi4_to_flowi(&fl4), NULL, 0);
		rt = (struct rtable*) dst_xfrm;
		if (IS_ERR(rt)) {
			rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IP_XFRM_LOOKUP_FAIL);
			goto drop;
		}

		skb_dst_set(skb, dst_xfrm);
		dst_output(&init_net, NULL, skb);
		rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IP_DST_OUTPUT_SUCCESS);

		return RX_HANDLER_CONSUMED;
	} else if(skb->dev && (skb->protocol == htons(ETH_P_IPV6)) &&
			  network_type == DATA_PATH_PROXY_NET_WWAN) {
		/* Use xfrm to route packet to rmnet data */
		struct ipv6hdr *ip6h, __ip6h;
		struct flowi6 fl6 = {};
		struct dst_entry *dst = NULL, *dst_xfrm;
		struct rtable *rt;
		struct net_device *ddev = NULL;

		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h) {
			rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IPV6_HDRP_FAIL);
			goto drop;
		}

		memcpy(&fl6.saddr, &ip6h->saddr, sizeof(struct in6_addr));
		memcpy(&fl6.daddr, &ip6h->daddr, sizeof(struct in6_addr));

		dst = ipv6_stub->ipv6_dst_lookup_flow(&init_net, NULL, &fl6, NULL);
		if (IS_ERR(dst)) {
			rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IPV6_DST_LOOKUP_FAIL);

			ddev = dev_get_by_name_rcu(&init_net, "dummy0");
			if (!ddev) {
				rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_R0_IPV6_DDEV_GET_FAIL);
				goto drop;
			}

			fl6.flowi6_oif = ddev->ifindex;
			fl6.flowi6_flags = FLOWI_FLAG_KNOWN_NH;

			dst = ipv6_stub->ipv6_dst_lookup_flow(&init_net, NULL, &fl6, NULL);
			if (IS_ERR(dst)) {
				rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IPV6_DDST_LOOKUP_FAIL);
				goto drop;
			}
		}

		dst_xfrm = xfrm_lookup(&init_net, dst, flowi6_to_flowi(&fl6), NULL, 0);
		rt = (struct rtable *)dst_xfrm;
		if (IS_ERR(rt)) {
			rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IPV6_XFRM_LOOKUP_FAIL);
			goto drop;
		}

		skb_dst_set(skb, dst_xfrm);
		dst_output(&init_net, NULL, skb);
		rmnet_wlan_forward_stats_update(RMNET_WWAN_F_S_NON_R0_IPV6_DST_OUTPUT_SUCCESS);

		return RX_HANDLER_CONSUMED;

	} else if(skb->dev && (skb->protocol == htons(ETH_P_IP)) &&
	          network_type == DATA_PATH_PROXY_NET_WLAN) {
		/* Use xfrm to route packet to rmnet data */
		struct iphdr *iph, __iph;
		struct flowi4 fl4 = {};
		struct net_device *wdev = NULL;
		struct dst_entry *dst_xfrm;
		struct net_device *ddev = NULL;
		struct rtable *rt;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IP_HDRP_FAIL);
			goto drop;
		}

		wdev = dev_get_by_name_rcu(&init_net, rmnet_wlan_get_dev());
		if (!wdev) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IP_DEV_GET_FAIL);
			goto drop;
		}

		memcpy(&fl4.daddr, &iph->daddr, sizeof(__be32));
		fl4.flowi4_oif = wdev->ifindex;
		fl4.flowi4_flags = FLOWI_FLAG_KNOWN_NH;

		rt = ip_route_output_key(&init_net, &fl4);
		if (IS_ERR(rt)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IP_RT_LOOKUP_FAIL);

			ddev = dev_get_by_name_rcu(&init_net, "dummy0");
			if (!ddev) {
				rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IP_DDEV_GET_FAIL);
				goto drop;
			}

			fl4.flowi4_oif = ddev->ifindex;
			fl4.flowi4_flags = FLOWI_FLAG_KNOWN_NH;

			rt = ip_route_output_key(&init_net, &fl4);
			if (IS_ERR(rt)) {
				rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IP_DRT_LOOKUP_FAIL);
				goto drop;
			}
		}
		memcpy(&fl4.saddr, &iph->saddr, sizeof(__be32));
		dst_xfrm = xfrm_lookup(&init_net, &rt->dst, flowi4_to_flowi(&fl4), NULL, 0);
		rt = (struct rtable*) dst_xfrm;
		if (IS_ERR(rt)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IP_XFRM_LOOKUP_FAIL);
			goto drop;
		}

		skb_dst_set(skb, dst_xfrm);
		dst_output(&init_net, NULL, skb);
		rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IP_DST_OUTPUT_SUCCESS);

		return RX_HANDLER_CONSUMED;
	} else if(skb->dev && (skb->protocol == htons(ETH_P_IPV6)) &&
			  network_type == DATA_PATH_PROXY_NET_WLAN) {
		/* Use xfrm to route packet to rmnet data */
		struct ipv6hdr *ip6h, __ip6h;
		struct flowi6 fl6 = {};
		struct dst_entry *dst = NULL, *dst_xfrm;
		struct rtable *rt;
		struct net_device *ddev = NULL;

		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IPV6_HDRP_FAIL);
			goto drop;
		}

		memcpy(&fl6.saddr, &ip6h->saddr, sizeof(struct in6_addr));
		memcpy(&fl6.daddr, &ip6h->daddr, sizeof(struct in6_addr));

		dst = ipv6_stub->ipv6_dst_lookup_flow(&init_net, NULL, &fl6, NULL);
		if (IS_ERR(dst)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IPV6_DST_LOOKUP_FAIL);

			ddev = dev_get_by_name_rcu(&init_net, "dummy0");
			if (!ddev) {
				rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_R0_IPV6_DDEV_GET_FAIL);
				goto drop;
			}

			fl6.flowi6_oif = ddev->ifindex;
			fl6.flowi6_flags = FLOWI_FLAG_KNOWN_NH;

			dst = ipv6_stub->ipv6_dst_lookup_flow(&init_net, NULL, &fl6, NULL);
			if (IS_ERR(dst)) {
				rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IPV6_DDST_LOOKUP_FAIL);
				goto drop;
			}
		}

		dst_xfrm = xfrm_lookup(&init_net, dst, flowi6_to_flowi(&fl6), NULL, 0);
		rt = (struct rtable*) dst_xfrm;
		if (IS_ERR(rt)) {
			rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IPV6_XFRM_LOOKUP_FAIL);
			goto drop;
		}

		skb_dst_set(skb, dst_xfrm);
		dst_output(&init_net, NULL, skb);
		rmnet_wlan_forward_stats_update(RMNET_WLAN_F_S_NON_R0_IPV6_DST_OUTPUT_SUCCESS);

		return RX_HANDLER_CONSUMED;
	}

drop:
	kfree_skb(skb);
	return RX_HANDLER_CONSUMED;
}

static rx_handler_result_t rmnet_wlan_connection_handler(struct sk_buff **pskb)
{
	struct rmnet_wlan_connection_info conn = {};
	struct rmnet_wlan_connection_node *node;
	struct sk_buff *skb = *pskb;
	unsigned long flags;
	struct rmnet_wlan_fwd_info fwd_info;
	struct rmnet_wlan_fwd_info_node *fwd_info_node;

	uint8_t network_type = __DATA_PATH_PROXY_NET_MAX;

	if (!skb || skb->pkt_type == PACKET_LOOPBACK)
		return RX_HANDLER_PASS;

	/* Get the source address and IP type */
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, __iph;
		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph)
			goto out;

		fwd_info.v4_addr = iph->saddr;
		fwd_info.ip_proto = 4;
	} else if(skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, __ip6h;
		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			goto out;

		memcpy(&fwd_info.v6_addr, &ip6h->saddr, sizeof(fwd_info.v6_addr));
		fwd_info.ip_proto = 6;
	} else {
		goto out;
	}

	/* Get the registered fwd_node to get the type */
	rcu_read_lock();
	fwd_info_node = rmnet_wlan_fwd_info_find(&fwd_info);
	rcu_read_unlock();
	if (!fwd_info_node)
		goto out;

	network_type = fwd_info_node->fwd.net_type;

	/* Invalid network type given */
	if(network_type == __DATA_PATH_PROXY_NET_MAX) goto out;

	/* replaces raw_before_defrag */
	if (skb->dev)
		nf_ct_set(skb, NULL, IP_CT_UNTRACKED);

	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, __iph;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph)
			goto out;

		if (iph->protocol == IPPROTO_TCP)
			goto clamp;

		if (iph->protocol != IPPROTO_ICMP)
			goto out;

		conn.v4_saddr = iph->saddr;
		conn.v4_daddr = iph->daddr;
		conn.ip_proto = 4;
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, __ip6h;
		__be16 frag_off;
		u8 proto;

		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			goto out;

		proto = ip6h->nexthdr;
		if (ipv6_skip_exthdr(skb, sizeof(*ip6h), &proto, &frag_off) < 0)
			goto out;

		if (frag_off && proto == NEXTHDR_FRAGMENT)
			/* Heckin' KIIIIILL meeeee... */
			goto out;

		if (proto == IPPROTO_TCP)
			goto clamp;

		if (proto != IPPROTO_ICMPV6)
			goto out;

		memcpy(&conn.v6_saddr, &ip6h->saddr, sizeof(conn.v6_saddr));
		memcpy(&conn.v6_daddr, &ip6h->daddr, sizeof(conn.v6_daddr));
		conn.ip_proto = 6;
	} else {
		goto out;
	}

	rcu_read_lock();
	hash_for_each_possible_rcu(rmnet_wlan_connection_hash, node, hash,
				   conn.v4_daddr) {
		if (node->dead)
			continue;

		if (!rmnet_wlan_connection_info_match(&node->info, &conn))
			continue;

		/* Match found. You still alive? *poke poke* */
		/* Ah, ah, ah, ah~ Stayin' alive, stayin' alive! */
		node->ts = jiffies;
		rcu_read_unlock();
		goto out;
	}

	rcu_read_unlock();

	/* Make a new connection entry */
	spin_lock_irqsave(&rmnet_wlan_connection_lock, flags);
	node = kzalloc(sizeof(*node), GFP_ATOMIC);
	if (!node) {
		/* Well, that's unfortunate */
		spin_unlock_irqrestore(&rmnet_wlan_connection_lock, flags);
		goto out;
	}

	INIT_HLIST_NODE(&node->hash);
	memcpy(&node->info, &conn, sizeof(conn));
	node->fwd = &fwd_info_node->fwd;
	hash_add_rcu(rmnet_wlan_connection_hash, &node->hash, conn.v4_daddr);
	if (!rmnet_wlan_connection_hash_size) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_WLAN_CONNECTION_WQ_INTERVAL);
		schedule_delayed_work(&rmnet_wlan_connection_work.ws, delay);
	}

	rmnet_wlan_connection_hash_size++;
	spin_unlock_irqrestore(&rmnet_wlan_connection_lock, flags);

out:
	return rmnet_wlan_receive_skb(skb, network_type);

clamp:
	/* Clamp any received SYNs */
	rmnet_wlan_tcp_mss_clamp(skb, TCP_FLAG_SYN);
	return rmnet_wlan_receive_skb(skb, network_type);
}

struct rmnet_wlan_fwd_info *
rmnet_wlan_connection_find(struct rmnet_wlan_connection_info *info)
	__must_hold(RCU)
{
	struct rmnet_wlan_connection_node *node;

	hash_for_each_possible_rcu(rmnet_wlan_connection_hash, node, hash,
				   info->v4_daddr) {
		if (node->dead)
			continue;

		if (!rmnet_wlan_connection_info_match(&node->info, info))
			continue;

		return node->fwd;
	}

	return NULL;
}

void rmnet_wlan_connection_flush(void)
{
	/* Purge anything old enough... */
	cancel_delayed_work_sync(&rmnet_wlan_connection_work.ws);

	rmnet_wlan_connection_work.force_clean = true;
	schedule_delayed_work(&rmnet_wlan_connection_work.ws, 0);

	/* ... and force remove all the rest. */
	cancel_delayed_work_sync(&rmnet_wlan_connection_work.ws);
}

void rmnet_wlan_ll_tuple_match(struct sk_buff *skb)
{
	int protocol = -1;
	struct rmnet_wlan_ll_tuple * tuple =
		rcu_dereference(rmnet_wlan_ll_tuple_cache);

	/* Check if something valid is cached */
	if (!tuple) return;

	/* IPv4 */
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, __iph;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if(!iph) return;

		if(iph->version != tuple->ip_proto &&
		   iph->saddr != tuple->v4_saddr &&
		   iph->daddr != tuple->v4_daddr)
			return;

		protocol = iph->protocol;
	/* IPv6 */
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, __ip6h;
		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if(!ip6h) return;

		if(ip6h->version != tuple->ip_proto &&
		   ipv6_addr_cmp(&ip6h->saddr, &tuple->v6_saddr) &&
		   ipv6_addr_cmp(&ip6h->daddr, &tuple->v6_daddr))
			return;

		protocol = ip6h->nexthdr;
	}

	/* Check that ports match and is UDP */
	if(protocol == IPPROTO_UDP) {
		if(udp_hdr(skb)->source == tuple->sport &&
		   udp_hdr(skb)->dest == tuple->dport)
			goto tx_priority;
	}

	return;
tx_priority:
	skb->priority = 0x9B6D0100;
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_LL_TX);
}

static rx_handler_result_t rmnet_wlan_ingress_rx_handler(struct sk_buff **pskb)
{
	struct net_device *device;

	if (!pskb || !(*pskb) || !(*pskb)->dev)
		return RX_HANDLER_PASS;

	device = (*pskb)->dev;
	/* Reverse devices this way please */
	if (!rmnet_wlan_strlcmp(device->name, "r_rmnet_data", 12))
		return rmnet_wlan_connection_handler(pskb);

	/* CIWLAN goes over here */
	if (!rmnet_wlan_strlcmp(device->name, rmnet_wwan_get_dev(), IFNAMSIZ))
		return rmnet_wlan_rx_handler(pskb);

	/* OH, you're a wlan device you say? Well, what pranksterful prankster
	 * is naming devices on this logic-forsaken machine...
	 */
	if (!rmnet_wlan_strlcmp(device->name, rmnet_wlan_get_dev(), IFNAMSIZ))
		return rmnet_wlan_rx_handler(pskb);

	/* We have no interest in your devices here */
	return RX_HANDLER_PASS;
}

static const struct rmnet_module_hook_register_info
rmnet_wlan_module_hooks[] = {
	{
		.hooknum = RMNET_MODULE_HOOK_WLAN_FLOW_MATCH,
		.func = rmnet_wlan_ll_tuple_match,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_WLAN_INGRESS_RX_HANDLER,
		.func = rmnet_wlan_ingress_rx_handler,
	},
};

void rmnet_wlan_set_hooks(void)
{
	rmnet_module_hook_register(rmnet_wlan_module_hooks,
				   ARRAY_SIZE(rmnet_wlan_module_hooks));
}

void rmnet_wlan_unset_hooks(void)
{
	rmnet_module_hook_unregister(rmnet_wlan_module_hooks,
				     ARRAY_SIZE(rmnet_wlan_module_hooks));
}

int rmnet_wlan_connection_init(void)
{
	INIT_DELAYED_WORK(&rmnet_wlan_connection_work.ws,
			  rmnet_wlan_connection_work_process);
	return 0;
}

int rmnet_wlan_connection_deinit(void)
{
	rmnet_wlan_connection_flush();
	return 0;
}
