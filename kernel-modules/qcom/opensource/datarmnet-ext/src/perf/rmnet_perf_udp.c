// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET PERF UDP framework */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/hashtable.h>
#include <linux/log2.h>
#include <linux/workqueue.h>
#include <linux/refcount.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/udp.h>
#include <net/sock.h>
#include "rmnet_private.h"

#include "rmnet_perf_udp.h"

/* How long to keep a node, in millisecs */
#define RMNET_PERF_UDP_TRACK_TIMEOUT (2000)
/* How often to run the cleaning workqueue, in millisecs */
#define RMNET_PERF_UDP_TRACK_WQ_INTERVAL (500)

#define RMNET_PERF_UDP_TRACK_HASH_BKTS (16)
#define RMNET_PERF_UDP_TRACK_HASH_BITS \
	(const_ilog2(RMNET_PERF_UDP_TRACK_HASH_BKTS))

enum {
	RMNET_PERF_UDP_TRACK_STAT_NODE_ADD,
	RMNET_PERF_UDP_TRACK_STAT_NODE_ADD_FAIL,
	RMNET_PERF_UDP_TRACK_STAT_NODE_DEL,
	RMNET_PERF_UDP_TRACK_STAT_NO_SK,
	RMNET_PERF_UDP_TRACK_STAT_MAX,
};

struct rmnet_perf_udp_track_tuple {
	union {
		__be32 v4_saddr;
		struct in6_addr v6_saddr;
	};
	union {
		__be32 v4_daddr;
		struct in6_addr v6_daddr;
	};
	union {
		struct {
			__be16 sport;
			__be16 dport;
		};
		u32 hash_key;
	};
	u8 ip_proto;
};

struct rmnet_perf_udp_track_node {
	struct hlist_node hash;
	struct rcu_head rcu;
	struct rmnet_perf_udp_track_tuple info;
	struct sock *node_sk;
	unsigned long ts;
	u8 dead;
};

struct rmnet_perf_udp_work_struct {
	struct delayed_work ws;
	bool force_clean;
};

/* For tracking hash protection */
static DEFINE_SPINLOCK(rmnet_perf_udp_track_lock);
static DEFINE_HASHTABLE(rmnet_perf_udp_track_hash,
			RMNET_PERF_UDP_TRACK_HASH_BITS);
static u32 rmnet_perf_udp_track_hash_size;

/* Periodic cleaning work struct for the hashtable */
static struct rmnet_perf_udp_work_struct rmnet_perf_udp_work;

/* Stats Array */
static u64 rmnet_perf_udp_track_stats[RMNET_PERF_UDP_TRACK_STAT_MAX];
module_param_array_named(rmnet_perf_udp_stat, rmnet_perf_udp_track_stats,
			 ullong, NULL, 0444);

static void rmnet_perf_udp_track_stats_update(u32 stat)
{
	if (stat < RMNET_PERF_UDP_TRACK_STAT_MAX)
		rmnet_perf_udp_track_stats[stat] += 1;
}

static bool
rmnet_perf_udp_track_node_expired(struct rmnet_perf_udp_track_node *node,
				  unsigned long ts)
{
	unsigned long timeout;

	timeout = msecs_to_jiffies(RMNET_PERF_UDP_TRACK_TIMEOUT);
	if (ts - node->ts > timeout)
		return true;

	return false;
}

static void rmnet_perf_udp_track_node_free(struct rcu_head *head)
{
	struct rmnet_perf_udp_track_node *node;

	node = container_of(head, struct rmnet_perf_udp_track_node, rcu);
	if (!IS_ERR_OR_NULL(node->node_sk))
		sock_put(node->node_sk);

	kfree(node);
}

static bool rmnet_perf_udp_track_hash_clean(bool force)
{
	struct rmnet_perf_udp_track_node *node;
	struct hlist_node *tmp;
	unsigned long ts;
	int bkt;

	ts = jiffies;
	hash_for_each_safe(rmnet_perf_udp_track_hash, bkt, tmp, node, hash) {
		if (node->dead)
			/* Node already marked as removed, but not yet
			 * purged after a grace period. Skip it.
			 */
			continue;

		if (force || rmnet_perf_udp_track_node_expired(node, ts)) {
			node->dead = 1;
			hash_del_rcu(&node->hash);
			call_rcu(&node->rcu, rmnet_perf_udp_track_node_free);
			rmnet_perf_udp_track_stats_update(RMNET_PERF_UDP_TRACK_STAT_NODE_DEL);
			rmnet_perf_udp_track_hash_size--;
		}
	}

	return !!rmnet_perf_udp_track_hash_size;
}

static void rmnet_perf_udp_work_process(struct work_struct *ws)
{
	struct rmnet_perf_udp_work_struct *udp_work;
	unsigned long flags;
	bool should_resched;

	udp_work = container_of(to_delayed_work(ws),
				struct rmnet_perf_udp_work_struct, ws);
	spin_lock_irqsave(&rmnet_perf_udp_track_lock, flags);
	should_resched = rmnet_perf_udp_track_hash_clean(udp_work->force_clean);
	if (should_resched) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_PERF_UDP_TRACK_WQ_INTERVAL);
		schedule_delayed_work(&udp_work->ws, delay);
	}

	spin_unlock_irqrestore(&rmnet_perf_udp_track_lock, flags);
}

static bool
rmnet_perf_udp_track_tuple_match(struct rmnet_perf_udp_track_tuple *t1,
				 struct rmnet_perf_udp_track_tuple *t2)
{
	if (t1->ip_proto != t2->ip_proto ||
	    t1->sport != t2->sport ||
	    t1->dport != t2->dport)
		return false;

	if (t1->ip_proto == 4)
		return t1->v4_saddr == t2->v4_saddr &&
		       t1->v4_daddr == t2->v4_daddr;

	return !ipv6_addr_cmp(&t1->v6_saddr, &t2->v6_saddr) &&
	       !ipv6_addr_cmp(&t1->v6_daddr, &t2->v6_daddr);
}

static struct rmnet_perf_udp_track_node *
rmnet_perf_udp_track_node_add(struct rmnet_perf_udp_track_tuple *tuple)
	__must_hold(&rmnet_perf_udp_track_lock)
{
	struct rmnet_perf_udp_track_node *node;

	node = kzalloc(sizeof(*node), GFP_ATOMIC);
	if (!node) {
		rmnet_perf_udp_track_stats_update(RMNET_PERF_UDP_TRACK_STAT_NODE_ADD_FAIL);
		return NULL;
	}

	INIT_HLIST_NODE(&node->hash);
	memcpy(&node->info, tuple, sizeof(*tuple));
	node->ts = jiffies;
	hash_add_rcu(rmnet_perf_udp_track_hash, &node->hash, tuple->hash_key);
	rmnet_perf_udp_track_stats_update(RMNET_PERF_UDP_TRACK_STAT_NODE_ADD);
	if (!rmnet_perf_udp_track_hash_size) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_PERF_UDP_TRACK_WQ_INTERVAL);
		schedule_delayed_work(&rmnet_perf_udp_work.ws, delay);
	}

	rmnet_perf_udp_track_hash_size++;
	return node;
}

static struct rmnet_perf_udp_track_node *
rmnet_perf_udp_track_tuple_find(struct rmnet_perf_udp_track_tuple *tuple)
	__must_hold(RCU)
{
	struct rmnet_perf_udp_track_node *node;
	unsigned long flags;

	spin_lock_irqsave(&rmnet_perf_udp_track_lock, flags);
	hash_for_each_possible_rcu(rmnet_perf_udp_track_hash, node, hash,
				   tuple->hash_key) {
		if (node->dead)
			continue;

		if (rmnet_perf_udp_track_tuple_match(&node->info, tuple))
			goto out;
	}

	/* Make a new one */
	node = rmnet_perf_udp_track_node_add(tuple);
out:
	spin_unlock_irqrestore(&rmnet_perf_udp_track_lock, flags);
	return node;
}

static struct sock *
rmnet_perf_udp_track_sk_lookup(struct rmnet_perf_udp_track_tuple *tuple,
			       struct sk_buff *skb)
{
	struct sock *udp_sock;

	/* UDP socket lookup is surprisingly involved. Fortunately, the
	 * kernel does at least export these helpers. They HAVE nice wrappers,
	 * but those aren't exported, naturally.
	 */
	if (tuple->ip_proto == 4)
		udp_sock = __udp4_lib_lookup(dev_net(skb->dev), tuple->v4_saddr,
					     tuple->sport, tuple->v4_daddr,
					     tuple->dport, inet_iif(skb), 0,
					     &udp_table, NULL);
	else
		udp_sock = __udp6_lib_lookup(dev_net(skb->dev),
					     &tuple->v6_saddr, tuple->sport,
					     &tuple->v6_daddr, tuple->dport,
					     inet6_iif(skb), 0, &udp_table,
					     NULL);

	/* Also, neither of these helpers handle bumping the socket refcount!
	 * We have to do that, in the manner of udp4/6_lib_lookup().
	 */
	if (udp_sock && !refcount_inc_not_zero(&udp_sock->sk_refcnt))
		udp_sock = NULL;

	return udp_sock;
}

static void
rmnet_perf_udp_track_node_update(struct rmnet_perf_udp_track_node *node,
				 struct sk_buff *skb)
	__must_hold(RCU)
{
	struct rmnet_skb_cb *rmnet_cb = RMNET_SKB_CB(skb);

	/* Poke the timestamp since the flow is still active */
	node->ts = jiffies;
	if (IS_ERR(node->node_sk)) {
		/* No socket found */
		rmnet_cb->tethered = true;
		return;
	}

	if (!node->node_sk) {
		/* Perform first-time socket lookup */
		node->node_sk = rmnet_perf_udp_track_sk_lookup(&node->info,
							       skb);
		if (!node->node_sk) {
			rmnet_perf_udp_track_stats_update(RMNET_PERF_UDP_TRACK_STAT_NO_SK);
			node->node_sk = ERR_PTR(-EINVAL);
			rmnet_cb->tethered = true;
			return;
		}
	}

	/* Graft in the socket since we have it? */
}

void rmnet_perf_ingress_handle_udp(struct sk_buff *skb)
{
	struct rmnet_perf_udp_track_tuple tuple = {};
	struct rmnet_perf_udp_track_node *node;
	struct udphdr *uh;

	if (!skb_transport_header_was_set(skb) ||
	    skb->ip_summed == CHECKSUM_NONE)
		return;

	uh = udp_hdr(skb);
	tuple.sport = uh->source;
	tuple.dport = uh->dest;
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph = ip_hdr(skb);

		tuple.v4_saddr = iph->saddr;
		tuple.v4_daddr = iph->daddr;
		tuple.ip_proto = 4;
	} else {
		struct ipv6hdr *ip6h = ipv6_hdr(skb);

		memcpy(&tuple.v6_saddr, &ip6h->saddr, sizeof(ip6h->saddr));
		memcpy(&tuple.v6_daddr, &ip6h->daddr, sizeof(ip6h->daddr));
		tuple.ip_proto = 6;
	}

	rcu_read_lock();
	node = rmnet_perf_udp_track_tuple_find(&tuple);
	if (node) {
		if (likely(!rmnet_perf_udp_track_node_expired(node, jiffies)))
			rmnet_perf_udp_track_node_update(node, skb);
	}

	rcu_read_unlock();
}

void rmnet_perf_egress_handle_udp(struct sk_buff *skb)
{
	struct rmnet_perf_udp_track_tuple tuple = {};
	struct rmnet_perf_udp_track_node *node;
	struct udphdr *uh;

	if (!skb_transport_header_was_set(skb))
		return;

	uh = udp_hdr(skb);
	/* Node tuples are formatted in the DL direction. Swap SRC and DST */
	tuple.sport = uh->dest;
	tuple.dport = uh->source;
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph = ip_hdr(skb);

		tuple.v4_saddr = iph->daddr;
		tuple.v4_daddr = iph->saddr;
		tuple.ip_proto = 4;
	} else {
		struct ipv6hdr *ip6h = ipv6_hdr(skb);

		memcpy(&tuple.v6_saddr, &ip6h->daddr, sizeof(ip6h->daddr));
		memcpy(&tuple.v6_daddr, &ip6h->saddr, sizeof(ip6h->saddr));
		tuple.ip_proto = 6;
	}

	rcu_read_lock();
	node = rmnet_perf_udp_track_tuple_find(&tuple);
	if (node) {
		if (likely(!rmnet_perf_udp_track_node_expired(node, jiffies)))
			rmnet_perf_udp_track_node_update(node, skb);
	}

	rcu_read_unlock();
}

int rmnet_perf_udp_init(void)
{
	INIT_DELAYED_WORK(&rmnet_perf_udp_work.ws,
			  rmnet_perf_udp_work_process);
	return 0;
}

void rmnet_perf_udp_exit(void)
{
	/* Force the current work struct to finish deleting anything old
	 * enough...
	 */
	cancel_delayed_work_sync(&rmnet_perf_udp_work.ws);

	rmnet_perf_udp_work.force_clean = true;
	schedule_delayed_work(&rmnet_perf_udp_work.ws, 0);

	/* ...and force remove all the rest of the nodes */
	cancel_delayed_work_sync(&rmnet_perf_udp_work.ws);
}
