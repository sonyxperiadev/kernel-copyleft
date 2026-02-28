// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET PERF TCP framework */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/hashtable.h>
#include <linux/log2.h>
#include <linux/workqueue.h>
#include <net/ip.h>
#include <net/inet_hashtables.h>
#include <net/ipv6.h>
#include <net/inet6_hashtables.h>
#include <net/tcp.h>
#include <net/sock.h>
#include "rmnet_private.h"

#include "rmnet_perf_tcp.h"

/* How long to hold a node, in millisecs */
#define RMNET_PERF_QUICKACK_TIMEOUT (2000)
/* How often to run the cleaning workqueue, in millisecs */
#define RMNET_PERF_QUICKACK_WQ_INTERVAL (500)

/* Default threshold is 192 KB of data. shsusrd can change this per flow */
#define RMNET_PERF_QUICKACK_THRESH (192000)
#define RMNET_PERF_QUICKACK_HASH_BKTS (16)
#define RMNET_PERF_QUICKACK_HASH_BITS \
	(const_ilog2(RMNET_PERF_QUICKACK_HASH_BKTS))

enum {
	RMNET_PERF_QUICKACK_STAT_NODE_ADD,
	RMNET_PERF_QUICKACK_STAT_NODE_ADD_FAIL,
	RMNET_PERF_QUICKACK_STAT_NODE_DEL,
	RMNET_PERF_QUICKACK_STAT_NO_SK,
	RMNET_PERF_QUICKACK_STAT_FORCE_RX,
	RMNET_PERF_QUICKACK_STAT_FORCE_TX,
	RMNET_PERF_QUICKACK_STAT_MAX,
};

struct rmnet_perf_quickack_tuple {
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

struct rmnet_perf_quickack_node {
	struct hlist_node hash;
	struct rcu_head rcu;
	struct rmnet_perf_quickack_tuple info;
	unsigned long ts;
	u32 tcp_seq;
	u32 tcp_ack;
	u32 offload_hash;
	u32 byte_threshold;
	u32 quickack_count;
	bool no_sock;
	u8 dead;
};

struct rmnet_perf_quickack_work_struct {
	struct delayed_work ws;
	bool force_clean;
};

/* For quickack hash protection */
static DEFINE_SPINLOCK(rmnet_perf_quickack_lock);
static DEFINE_HASHTABLE(rmnet_perf_quickack_hash,
			RMNET_PERF_QUICKACK_HASH_BITS);
static u32 rmnet_perf_quickack_hash_size;

/* Periodic cleaning work struct for the hashtable */
static struct rmnet_perf_quickack_work_struct rmnet_perf_quickack_work;

/* Maximum number of flows to support at a time */
static u32 rmnet_perf_quickack_hash_size_param = 100;
module_param_named(rmnet_perf_tcp_knob0, rmnet_perf_quickack_hash_size_param,
		   uint, 0644);

/* Stats Array */
static u64 rmnet_perf_quickack_stats[RMNET_PERF_QUICKACK_STAT_MAX];
module_param_array_named(rmnet_perf_tcp_stat, rmnet_perf_quickack_stats,
			 ullong, NULL, 0444);

static void rmnet_perf_quickack_stats_update(u32 stat)
{
	if (stat < RMNET_PERF_QUICKACK_STAT_MAX)
		rmnet_perf_quickack_stats[stat] += 1;
}

static bool
rmnet_perf_quickack_node_expired(struct rmnet_perf_quickack_node *node,
				unsigned long ts)
{
	unsigned long timeout;

	timeout = msecs_to_jiffies(RMNET_PERF_QUICKACK_TIMEOUT);
	if (ts - node->ts > timeout)
		return true;

	return false;
}

static void rmnet_perf_quickack_node_free(struct rcu_head *head)
{
	struct rmnet_perf_quickack_node *node;

	node = container_of(head, struct rmnet_perf_quickack_node, rcu);
	kfree(node);
}

static bool rmnet_perf_quickack_hash_clean(bool force)
{
	struct rmnet_perf_quickack_node *node;
	struct hlist_node *tmp;
	unsigned long ts;
	int bkt;

	ts = jiffies;
	hash_for_each_safe(rmnet_perf_quickack_hash, bkt, tmp, node, hash) {
		if (node->dead)
			/* Node already marked as removed, but not yet
			 * purged after a grace period. Skip it.
			 */
			continue;

		if (force || rmnet_perf_quickack_node_expired(node, ts)) {
			node->dead = true;
			hash_del_rcu(&node->hash);
			call_rcu(&node->rcu, rmnet_perf_quickack_node_free);
			rmnet_perf_quickack_stats_update(RMNET_PERF_QUICKACK_STAT_NODE_DEL);
			rmnet_perf_quickack_hash_size--;
		}
	}

	return !!rmnet_perf_quickack_hash_size;
}

static void rmnet_perf_quickack_work_process(struct work_struct *ws)
{
	struct rmnet_perf_quickack_work_struct *quickack_work;
	unsigned long flags;
	bool should_resched;

	quickack_work = container_of(to_delayed_work(ws),
				     struct rmnet_perf_quickack_work_struct,
				     ws);
	spin_lock_irqsave(&rmnet_perf_quickack_lock, flags);
	should_resched =
		rmnet_perf_quickack_hash_clean(quickack_work->force_clean);
	if (should_resched) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_PERF_QUICKACK_WQ_INTERVAL);
		schedule_delayed_work(&quickack_work->ws, delay);
	}

	spin_unlock_irqrestore(&rmnet_perf_quickack_lock, flags);
}

static bool
rmnet_perf_quickack_tuple_match(struct rmnet_perf_quickack_tuple *t1,
				struct rmnet_perf_quickack_tuple *t2)
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

static struct rmnet_perf_quickack_node *
rmnet_perf_quickack_node_add(struct rmnet_perf_quickack_tuple *tuple)
	__must_hold(&rmnet_perf_quickack_lock)
{
	struct rmnet_perf_quickack_node *node;

	if (rmnet_perf_quickack_hash_size >= rmnet_perf_quickack_hash_size_param)
		/* Max flows. Ignore */
		return NULL;

	node = kzalloc(sizeof(*node), GFP_ATOMIC);
	if (!node) {
		rmnet_perf_quickack_stats_update(RMNET_PERF_QUICKACK_STAT_NODE_ADD_FAIL);
		return NULL;
	}

	INIT_HLIST_NODE(&node->hash);
	memcpy(&node->info, tuple, sizeof(*tuple));
	node->byte_threshold = RMNET_PERF_QUICKACK_THRESH;
	node->ts = jiffies;
	hash_add_rcu(rmnet_perf_quickack_hash, &node->hash, tuple->hash_key);
	rmnet_perf_quickack_stats_update(RMNET_PERF_QUICKACK_STAT_NODE_ADD);
	if (!rmnet_perf_quickack_hash_size) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_PERF_QUICKACK_WQ_INTERVAL);
		schedule_delayed_work(&rmnet_perf_quickack_work.ws, delay);
	}

	rmnet_perf_quickack_hash_size++;
	return node;
}

static void rmnet_perf_quickack_node_init(struct rmnet_perf_quickack_node *node,
					  struct sk_buff *skb, bool is_tx)
	__must_hold(RCU)
{
	struct tcphdr *th = tcp_hdr(skb);
	struct rmnet_skb_cb *rmnet_cb = RMNET_SKB_CB(skb);

	node->offload_hash = skb->hash;
	if (is_tx) {
		u32 tcp_ack = ntohl(th->ack_seq);

		WRITE_ONCE(node->tcp_ack, tcp_ack);
		/* If we're initializing on an ACK, assume no data has flowed
		 * yet, as this is very likely the ACK finishing the handshake.
		 * SEQ will be equal to the ACK in this case.
		 */
		WRITE_ONCE(node->tcp_seq, tcp_ack);
	} else {
		u32 tcp_seq = ntohl(th->seq);

		WRITE_ONCE(node->tcp_seq, tcp_seq);
		/* If we're initializing on DATA, assume this is the first
		 * data packet in the flow. The ACK number is 1 less than the
		 * sequence number, as only the handshake is complete.
		 */
		WRITE_ONCE(node->tcp_ack, tcp_seq - 1);
		rmnet_cb->bif = 0;
		rmnet_cb->ack_thresh = READ_ONCE(node->byte_threshold);
	}
}

static struct rmnet_perf_quickack_node *
rmnet_perf_quickack_tuple_find(struct rmnet_perf_quickack_tuple *tuple,
			       struct sk_buff *skb, bool is_tx)
	__must_hold(RCU)
{
	struct rmnet_perf_quickack_node *node;
	unsigned long flags;

	spin_lock_irqsave(&rmnet_perf_quickack_lock, flags);
	hash_for_each_possible_rcu(rmnet_perf_quickack_hash, node, hash,
				   tuple->hash_key) {
		if (node->dead)
			continue;

		if (rmnet_perf_quickack_tuple_match(&node->info, tuple)) {
			spin_unlock_irqrestore(&rmnet_perf_quickack_lock,
					       flags);
			return node;
		}
	}

	/* Make a new one */
	node = rmnet_perf_quickack_node_add(tuple);
	spin_unlock_irqrestore(&rmnet_perf_quickack_lock, flags);
	if (node)
		rmnet_perf_quickack_node_init(node, skb, is_tx);

	return node;
}

static struct sock *
rmnet_perf_sk_lookup(struct rmnet_perf_quickack_tuple *tuple,
		     struct net_device *skb_dev)
{
	struct net *net = dev_net(skb_dev);

	if (tuple->ip_proto == 4)
		return inet_lookup_established(net, &tcp_hashinfo,
					       tuple->v4_saddr,
					       tuple->sport, tuple->v4_daddr,
					       tuple->dport, skb_dev->ifindex);

	/* Interestingly, this one doesn't have a nice wrapper.
	 *
	 * And yes, the ntohs on dport here is intentional. The v4 wrapper
	 * actually handles doing that for us. The lookup code REALLY does want
	 * dport in host order ;)
	 */
	return __inet6_lookup_established(net, &tcp_hashinfo, &tuple->v6_saddr,
					  tuple->sport, &tuple->v6_daddr,
					  ntohs(tuple->dport), skb_dev->ifindex,
					  0);
}

static void rmnet_perf_quickack_force(struct rmnet_perf_quickack_node *node,
				      struct sk_buff *skb)
	__must_hold(RCU)
{
	struct sock *sk;

	if (skb->sk) {
		/* Packet has one! Only possible on the TX path */
		sk = skb->sk;

		if (sk_fullsock(sk)) {
			if (sk->sk_state == TCP_ESTABLISHED &&
			    !sock_flag(sk, SOCK_DEAD) &&
			    !sk_unhashed(sk) &&
			    sk->sk_shutdown != SHUTDOWN_MASK) {
				inet_csk(sk)->icsk_ack.pending |= ICSK_ACK_NOW;
				node->quickack_count++;
				rmnet_perf_quickack_stats_update(RMNET_PERF_QUICKACK_STAT_FORCE_TX);
			}
		}
		return;
	}

	sk = rmnet_perf_sk_lookup(&node->info, skb->dev);
	/* Note that this will take a reference to the socket. */
	if (!sk) {
		struct rmnet_skb_cb *rmnet_cb = RMNET_SKB_CB(skb);

		/* There's no established socket on the host.
		 * Flow is tethered, or something weird happened. Log, mark,
		 * and avoid touching this flow anymore.
		 */
		rmnet_perf_quickack_stats_update(RMNET_PERF_QUICKACK_STAT_NO_SK);
		node->no_sock = true;
		rmnet_cb->tethered = true;
		return;
	}

	if (sk_fullsock(sk)) {
		bh_lock_sock(sk);
		if (sk->sk_state == TCP_ESTABLISHED &&
		    !sock_flag(sk, SOCK_DEAD) &&
		    !sk_unhashed(sk) &&
		    sk->sk_shutdown != SHUTDOWN_MASK) {
			inet_csk(sk)->icsk_ack.pending |= ICSK_ACK_NOW;
			inet_csk_schedule_ack(sk);
			node->quickack_count++;
			rmnet_perf_quickack_stats_update(RMNET_PERF_QUICKACK_STAT_FORCE_RX);
		}

		bh_unlock_sock(sk);
	}
	sock_gen_put(sk);
}

/* Quick and dirty payload length calculation. Note that this requires
 * tcp_hdr(skb) to be valid, so make sure it is ;)
 */
static u32 rmnet_perf_tcp_payload_len(struct sk_buff *skb)
{
	struct tcphdr *th = tcp_hdr(skb);

	return skb->len - ((u8 *)th - skb->data) - th->doff * 4;
}

static void
rmnet_perf_quickack_node_update(struct rmnet_perf_quickack_node *node,
				struct sk_buff *skb, bool is_tx)
	__must_hold(RCU)
{
	struct tcphdr *th = tcp_hdr(skb);
	u32 curr_seq = ntohl(th->seq);
	u32 curr_ack = ntohl(th->ack_seq);
	u32 node_seq = READ_ONCE(node->tcp_seq);
	u32 node_ack = READ_ONCE(node->tcp_ack);
	u32 byte_thresh = READ_ONCE(node->byte_threshold);

	/* First off, poke the timestamp. The flow is still active. */
	node->ts = jiffies;
	if (node->no_sock)
		/* Don't bother, we have nothing to update */
		return;

	if (is_tx) {
		/* Care about the ACK */
		if (after(curr_ack, node_ack)) {
			u32 unacked = 0;

			if (curr_ack > node_seq)
				unacked = curr_ack - node_seq;

			// trace_printk("%s(): curr_ack %lu node_ack %lu node_seq %lu unacked %lu TX %lu\n",
			// 		__func__, curr_ack, node_ack, node_seq, unacked,
			// 		rmnet_perf_quickack_stats[RMNET_PERF_QUICKACK_STAT_FORCE_TX]);
			if (unacked > byte_thresh)
				rmnet_perf_quickack_force(node, skb);

			WRITE_ONCE(node->tcp_ack, curr_ack);
		}
	} else {
		u32 unacked;

		/* Care about the SEQ */
		if (after(curr_seq, node_seq)) {
			unacked = curr_seq - node_ack;
			unacked += rmnet_perf_tcp_payload_len(skb);

			// trace_printk("%s(): curr_seq %lu node_seq %lu node_ack %lu unacked %lu unacked' %lu RX %lu\n",
			// 		__func__, curr_seq, node_seq, node_ack, unacked,
			// 		unacked - rmnet_perf_tcp_payload_len(skb),
			// 		rmnet_perf_quickack_stats[RMNET_PERF_QUICKACK_STAT_FORCE_RX]);

			if (unacked > byte_thresh)
				rmnet_perf_quickack_force(node, skb);

			WRITE_ONCE(node->tcp_seq, curr_seq);
		}
	}
}

static bool
rmnet_perf_ingress_handle_tcp_common(struct sk_buff *skb,
				     struct rmnet_perf_quickack_tuple *tuple)
{
	struct tcphdr *th;
	u32 payload_len;

	/* At this point, both RSC and rmnet_offload have looked at this packet.
	 * If they haven't been able to process this thing successfully, then
	 * there's no use in trying on our end either ;)
	 *
	 * BUT WHAT IF BOTH RSC AND OFFLOAD ARE DISABLED????
	 * Then the socket is only ever getting a stream of 1500 byte packets.
	 * If the kernel can't handle THAT, then we have a bigger problem than
	 * this driver could ever hope to fix.
	 */
	if (!skb_transport_header_was_set(skb) ||
	    skb->ip_summed == CHECKSUM_NONE)
		return false;

	th = tcp_hdr(skb);
	if (th->syn)
		/* SYNs and SYN-ACKs are skipped, as we don't know if there's
		 * even a socket to check yet (and even if there is, how much
		 * data can these packets have~? helllloooo Fast Open that
		 * somehow resulted in 64KB coalescing! ;)
		 */
		return false;

	payload_len = rmnet_perf_tcp_payload_len(skb);
	if (!payload_len && th->ack)
		/* DL ACKs aren't counted. We only care about DL data. */
		return false;

	tuple->sport = th->source;
	tuple->dport = th->dest;
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph = ip_hdr(skb);

		tuple->v4_saddr = iph->saddr;
		tuple->v4_daddr = iph->daddr;
		tuple->ip_proto = 4;
	} else {
		struct ipv6hdr *ip6h = ipv6_hdr(skb);

		memcpy(&tuple->v6_saddr, &ip6h->saddr, sizeof(ip6h->saddr));
		memcpy(&tuple->v6_daddr, &ip6h->daddr, sizeof(ip6h->daddr));
		tuple->ip_proto = 6;
	}

	/* We will process this packet */
	return true;
}

/* Process a TCP packet on the RMNET core */
void rmnet_perf_ingress_handle_tcp(struct sk_buff *skb)
{
	struct rmnet_skb_cb *rmnet_cb = RMNET_SKB_CB(skb);
	struct rmnet_perf_quickack_tuple tuple = {};
	struct rmnet_perf_quickack_node *node;

	if (!rmnet_perf_ingress_handle_tcp_common(skb, &tuple))
		return;

	rcu_read_lock();
	node = rmnet_perf_quickack_tuple_find(&tuple, skb, false);
	if (!node)
		goto out;

	if (unlikely(rmnet_perf_quickack_node_expired(node, jiffies)))
		goto out;

	/* Our one and only job here is to report statistics to shs via the
	 * rmnet_cb struct in the skb. All actual tracking happens on the
	 * network stack core, where the calculations will be far more
	 * accurate as RPS has finished.
	 */
	if (node->no_sock) {
		rmnet_cb->tethered = true;
	} else {
		u32 unacked = READ_ONCE(node->tcp_seq) -
			      READ_ONCE(node->tcp_ack);

		/* A "good enough" estimate of the bytes in flight:
		 * How much outstatding data is there, using only values
		 * for packets the stack has seen.
		 * (i.e. not counting the current data we have yet to
		 * queue to RPS)
		 */
		rmnet_cb->bif = unacked;
		rmnet_cb->ack_thresh = READ_ONCE(node->byte_threshold);
		rmnet_cb->ack_forced = node->quickack_count;
	}

out:
	rcu_read_unlock();
}

/* Process a TCP packet on the Network stack core */
void rmnet_perf_ingress_rx_handler_tcp(struct sk_buff *skb)
{
	struct rmnet_perf_quickack_tuple tuple = {};
	struct rmnet_perf_quickack_node *node;

	if (!rmnet_perf_ingress_handle_tcp_common(skb, &tuple))
		return;

	rcu_read_lock();
	node = rmnet_perf_quickack_tuple_find(&tuple, skb, false);
	if (node) {
		if (likely(!rmnet_perf_quickack_node_expired(node, jiffies)))
			rmnet_perf_quickack_node_update(node, skb, false);
	}

	rcu_read_unlock();
}

void rmnet_perf_egress_handle_tcp(struct sk_buff *skb)
{
	struct rmnet_perf_quickack_tuple tuple = {};
	struct rmnet_perf_quickack_node *node;
	struct tcphdr *th;
	u32 payload_len;

	/* The only case I can see where this would be the case is for
	 * forwarded packets. In which case, we don't even have a socket
	 * to force quickack on, so just skip everything.
	 */
	if (!skb_transport_header_was_set(skb))
		return;

	th = tcp_hdr(skb);
	if (th->syn)
		/* SYNs and SYN-ACKs are skipped for the same reason as the
		 * ingress hook: no data at the socket yet.
		 */
		return;

	payload_len = rmnet_perf_tcp_payload_len(skb);
	if (payload_len || !th->ack)
		/* We don't care about UL data, only UL ACKs */
		return;

	/* Node tuples are formatted in the DL direction. Swap SRC and DST */
	tuple.sport = th->dest;
	tuple.dport = th->source;
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
	node = rmnet_perf_quickack_tuple_find(&tuple, skb, true);
	if (node) {
		if (likely(!rmnet_perf_quickack_node_expired(node, jiffies)))
			rmnet_perf_quickack_node_update(node, skb, true);
	}

	rcu_read_unlock();
}

void rmnet_perf_tcp_update_quickack_thresh(u32 hash_key, u32 byte_thresh)
{
	struct rmnet_perf_quickack_node *node;
	int bkt;

	if (hash_key == 0x0) {
		if (byte_thresh == 0) {
			enable_tcp = false;
		} else if (byte_thresh == 1) {
			enable_tcp = true;
		}
		return;
	}

	rcu_read_lock();
	hash_for_each_rcu(rmnet_perf_quickack_hash, bkt, node, hash) {
		if (node->offload_hash == hash_key)
			WRITE_ONCE(node->byte_threshold, byte_thresh);
	}

	rcu_read_unlock();
}

int rmnet_perf_tcp_init(void)
{
	INIT_DELAYED_WORK(&rmnet_perf_quickack_work.ws,
			  rmnet_perf_quickack_work_process);
	return 0;
}

void rmnet_perf_tcp_exit(void)
{
	/* Force the current work struct to finish deleting anything old
	 * enough...
	 */
	cancel_delayed_work_sync(&rmnet_perf_quickack_work.ws);

	rmnet_perf_quickack_work.force_clean = true;
	schedule_delayed_work(&rmnet_perf_quickack_work.ws, 0);

	/* ...and force remove all the rest of the nodes */
	cancel_delayed_work_sync(&rmnet_perf_quickack_work.ws);
}
