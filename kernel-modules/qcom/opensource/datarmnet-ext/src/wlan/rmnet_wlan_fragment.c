// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN fragment handler framework */

#include <linux/types.h>
#include <linux/skbuff.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <linux/rcupdate.h>
#include <linux/list.h>
#include <linux/rculist.h>
#include <linux/hashtable.h>
#include <linux/workqueue.h>
#include "rmnet_wlan.h"
#include "rmnet_wlan_stats.h"
#include "rmnet_wlan_fragment.h"

#define RMNET_WLAN_FRAGMENT_BKTS (16)
#define RMNET_WLAN_FRAGMENT_HASH_BITS (const_ilog2(RMNET_WLAN_FRAGMENT_BKTS))

/* Period to wait after receiving fragmented packet before declaring no more
 * fragments are coming. 100 ms, currently.
 */
#define RMNET_WLAN_FRAGMENT_TIMEOUT (100)

/* How often to run the cleaning workqueue while framents are present, in ms. */
#define RMNET_WLAN_FRAGMENT_WQ_INTERVAL (50)

struct rmnet_wlan_fragment_info {
	/* Need both addresses to check fragments */
	union {
		__be32 v4_saddr;
		struct in6_addr v6_saddr;
	};
	union {
		__be32 v4_daddr;
		struct in6_addr v6_daddr;
	};
	__be32 id;
	u16 ip_len;
	u16 offset;
	u8 ip_proto;
};

struct rmnet_wlan_fragment_node {
	struct hlist_node hash;
	/* Protects the list of queued fragments */
	spinlock_t pkt_lock;
	struct list_head pkts;
	struct rcu_head rcu;
	struct rmnet_wlan_fragment_info info;
	struct rmnet_wlan_fwd_info *fwd;
	unsigned long ts;
	bool dead;
};

struct rmnet_wlan_fragment_work_struct {
	struct delayed_work ws;
	bool force_clean;
};

/* For fragment hashtable protection */
static DEFINE_SPINLOCK(rmnet_wlan_fragment_lock);
static DEFINE_HASHTABLE(rmnet_wlan_fragment_hash,
			RMNET_WLAN_FRAGMENT_HASH_BITS);
/* Current size of the hashtable. This is purposely a u64 because some
 * places seem to have ways of blasting ridiculous amounts of fragments into
 * the XFRM tunnel at once. If overflow happens here (meaning UINT64_MAX logical
 * packets that have been fragmented within a single RCU grace period), then
 * boy-howdy do we need to have a talk...
 */
static u64 rmnet_wlan_fragment_hash_size;

/* Periodic cleaning work struct for the hashtable */
static struct rmnet_wlan_fragment_work_struct rmnet_wlan_fragment_work;

static int rmnet_wlan_ipv6_find_hdr(const struct sk_buff *skb,
				    unsigned int *offset, int target,
				    unsigned short *fragoff, int *flags)
{
	unsigned int start = skb_network_offset(skb) + sizeof(struct ipv6hdr);
	u8 nexthdr = ipv6_hdr(skb)->nexthdr;
	bool found;

	if (fragoff)
		*fragoff = 0;

	if (*offset) {
		struct ipv6hdr _ip6, *ip6;

		ip6 = skb_header_pointer(skb, *offset, sizeof(_ip6), &_ip6);
		if (!ip6 || (ip6->version != 6))
			return -EBADMSG;

		start = *offset + sizeof(struct ipv6hdr);
		nexthdr = ip6->nexthdr;
	}

	do {
		struct ipv6_opt_hdr _hdr, *hp;
		unsigned int hdrlen;
		found = (nexthdr == target);

		if ((!ipv6_ext_hdr(nexthdr)) || nexthdr == NEXTHDR_NONE) {
			if (target < 0 || found)
				break;
			return -ENOENT;
		}

		hp = skb_header_pointer(skb, start, sizeof(_hdr), &_hdr);
		if (!hp)
			return -EBADMSG;

		if (nexthdr == NEXTHDR_ROUTING) {
			struct ipv6_rt_hdr _rh, *rh;

			rh = skb_header_pointer(skb, start, sizeof(_rh),
						&_rh);
			if (!rh)
				return -EBADMSG;

			if (flags && (*flags & IP6_FH_F_SKIP_RH) &&
			    rh->segments_left == 0)
				found = false;
		}

		if (nexthdr == NEXTHDR_FRAGMENT) {
			unsigned short _frag_off;
			__be16 *fp;

			if (flags)	/* Indicate that this is a fragment */
				*flags |= IP6_FH_F_FRAG;
			fp = skb_header_pointer(skb,
						start+offsetof(struct frag_hdr,
							       frag_off),
						sizeof(_frag_off),
						&_frag_off);
			if (!fp)
				return -EBADMSG;

			_frag_off = ntohs(*fp) & ~0x7;
			if (_frag_off) {
				if (target < 0 &&
				    ((!ipv6_ext_hdr(hp->nexthdr)) ||
				     hp->nexthdr == NEXTHDR_NONE)) {
					if (fragoff)
						*fragoff = _frag_off;
					return hp->nexthdr;
				}
				if (!found)
					return -ENOENT;
				if (fragoff)
					*fragoff = _frag_off;
				break;
			}
			hdrlen = 8;
		} else if (nexthdr == NEXTHDR_AUTH) {
			if (flags && (*flags & IP6_FH_F_AUTH) && (target < 0))
				break;
			hdrlen = ipv6_authlen(hp);
		} else
			hdrlen = ipv6_optlen(hp);

		if (!found) {
			nexthdr = hp->nexthdr;
			start += hdrlen;
		}
	} while (!found);

	*offset = start;
	return nexthdr;
}

static bool
rmnet_wlan_fragment_node_expired(struct rmnet_wlan_fragment_node *node,
				 unsigned long ts)
{
	unsigned long timeout;

	timeout = msecs_to_jiffies(RMNET_WLAN_FRAGMENT_TIMEOUT);
	if (ts - node->ts > timeout)
		return true;

	return false;
}

static void
rmnet_wlan_flush_fragment_node(struct rmnet_wlan_fragment_node *node,
			       bool in_net_rx)
{
	struct rmnet_wlan_fwd_info *info;
	int (*rx_func)(struct sk_buff *skb);
	struct sk_buff *skb, *tmp;
	unsigned long flags;

	rx_func = (in_net_rx) ? netif_receive_skb : __netif_rx;
	info = node->fwd;
	spin_lock_irqsave(&node->pkt_lock, flags);
	list_for_each_entry_safe(skb, tmp, &node->pkts, list) {
		u32 stat;

		list_del(&skb->list);
		skb->next = NULL;
		skb->prev = NULL;
		if (IS_ERR_OR_NULL(info)) {
			rx_func(skb);
			continue;
		}

		/* Forward fragment */
		if (rmnet_wlan_deliver_skb(skb, info)) {
			stat = RMNET_WLAN_STAT_FRAG_FWD_NO_DEV;
			rx_func(skb);
		} else {
			stat = RMNET_WLAN_STAT_FRAG_FWD;
		}

		rmnet_wlan_stats_update(stat);
	}

	spin_unlock_irqrestore(&node->pkt_lock, flags);
}

static bool rmnet_wlan_fragment_hash_clean(bool force)
{
	struct rmnet_wlan_fragment_node *node;
	struct hlist_node *tmp;
	unsigned long ts;
	int bkt;

	ts = jiffies;
	hash_for_each_safe(rmnet_wlan_fragment_hash, bkt, tmp, node, hash) {
		if (node->dead)
			/* Node already marked as removed, but not yet
			 * purged after an RCU grace period. Skip it.
			 */
			continue;

		if (force || rmnet_wlan_fragment_node_expired(node, ts)) {
			node->dead = true;
			hash_del_rcu(&node->hash);
			/* Flush out any fragments we're holding */
			rmnet_wlan_flush_fragment_node(node, false);
			kfree_rcu(node, rcu);
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_FRAG_EXP);
			rmnet_wlan_fragment_hash_size--;
		}
	}

	return !!rmnet_wlan_fragment_hash_size;
}

static void rmnet_wlan_fragment_work_process(struct work_struct *ws)
{
	struct rmnet_wlan_fragment_work_struct *fragment_work;
	unsigned long flags;
	bool should_resched;

	fragment_work = container_of(to_delayed_work(ws),
				     struct rmnet_wlan_fragment_work_struct,
				     ws);
	spin_lock_irqsave(&rmnet_wlan_fragment_lock, flags);
	should_resched =
		rmnet_wlan_fragment_hash_clean(fragment_work->force_clean);
	if (should_resched) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_WLAN_FRAGMENT_WQ_INTERVAL);
		schedule_delayed_work(&fragment_work->ws, delay);
	}

	spin_unlock_irqrestore(&rmnet_wlan_fragment_lock, flags);
}

static bool rmnet_wlan_fragment_match(struct rmnet_wlan_fragment_info *i1,
				      struct rmnet_wlan_fragment_info *i2)
{
	if (i1->ip_proto != i2->ip_proto || i1->id != i2->id)
		return false;

	if (i1->ip_proto == 4)
		return i1->v4_saddr == i2->v4_saddr &&
		       i1->v4_daddr == i2->v4_daddr;

	return !ipv6_addr_cmp(&i1->v6_saddr, &i2->v6_saddr) &&
	       !ipv6_addr_cmp(&i1->v6_daddr, &i2->v6_daddr);
}

static struct rmnet_wlan_fragment_node *
rmnet_wlan_fragment_find(struct rmnet_wlan_fragment_info *info)
{
	struct rmnet_wlan_fragment_node *node;
	unsigned long flags;

	spin_lock_irqsave(&rmnet_wlan_fragment_lock, flags);
	hash_for_each_possible_rcu(rmnet_wlan_fragment_hash, node, hash,
				   info->id) {
		if (node->dead)
			continue;

		if (rmnet_wlan_fragment_match(info, &node->info))
			goto out;
	}

	/* Time to make one */
	node = kzalloc(sizeof(*node), GFP_ATOMIC);
	if (!node)
		goto out;

	spin_lock_init(&node->pkt_lock);
	INIT_LIST_HEAD(&node->pkts);
	memcpy(&node->info, info, sizeof(*info));
	INIT_HLIST_NODE(&node->hash);
	hash_add_rcu(rmnet_wlan_fragment_hash, &node->hash, info->id);
	if (!rmnet_wlan_fragment_hash_size) {
		unsigned long delay;

		delay = msecs_to_jiffies(RMNET_WLAN_FRAGMENT_WQ_INTERVAL);
		schedule_delayed_work(&rmnet_wlan_fragment_work.ws, delay);
	}

	rmnet_wlan_fragment_hash_size++;

out:
	spin_unlock_irqrestore(&rmnet_wlan_fragment_lock, flags);
	return node;
}

static int rmnet_wlan_fragment_handle(struct sk_buff *skb,
				      struct rmnet_wlan_tuple *tuple,
				      struct rmnet_wlan_fragment_info *info,
				      struct rmnet_wlan_fwd_info *fwd_info)
	__must_hold(RCU)
{
	struct skb_shared_info *shinfo = skb_shinfo(skb);
	struct rmnet_wlan_fragment_node *node;
	int ret = 1; /* Pass on by default */

	/* Avoid toching any fragments we've already seen when our rx_handler
	 * has been invoked again after flushing to the network stack.
	 */
	if (shinfo->tskey) {
		/* This is basically unused by the kernel on the RX side, but
		 * we can play nice and reset it to the default value, now
		 * that it can't end up back here.
		 */
		shinfo->tskey = 0;
		goto out;
	}

	rmnet_wlan_stats_update(RMNET_WLAN_STAT_FRAG_RX);
	/* Mark this fragment as having been seen by the rx handler */
	shinfo->tskey = 1;
	/* Check our fragment table */
	node = rmnet_wlan_fragment_find(info);
	if (!node) {
		/* Allocation error */
		ret = (-1);
		goto out;
	}

	/* Poke the timestamp, since there are still fragments happening */
	node->ts = jiffies;

	/* Have we seen the initial frag? */
	if (node->fwd) {
		if (IS_ERR(node->fwd))
			/* We don't need to forward this tuple */
			goto out;

		/* Forward it to the device we used for the others */
		if (!rmnet_wlan_deliver_skb(skb, node->fwd)) {
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_FRAG_FWD);
			ret = 0;
			goto out;
		}

		rmnet_wlan_stats_update(RMNET_WLAN_STAT_FRAG_FWD_NO_DEV);
		goto out;
	}

	if (info->offset) {
		unsigned long flags;

		/* Ah, the worst case scenario. The fragments are arriving
		 * out of order, and we haven't seen the inital fragment to
		 * determine if we care about this packet or not. We have no
		 * choice but to hold it.
		 */
		spin_lock_irqsave(&node->pkt_lock, flags);
		list_add_tail(&skb->list, &node->pkts);
		spin_unlock_irqrestore(&node->pkt_lock, flags);
		ret = 0;
		rmnet_wlan_stats_update(RMNET_WLAN_STAT_FRAG_QUEUE);
		goto out;
	}

	/* We have the first fragment. Time to figure out what to do */
	if (tuple->trans_proto == IPPROTO_TCP ||
	    tuple->trans_proto == IPPROTO_UDP) {
		struct udphdr *udph, __udph;

		udph = skb_header_pointer(skb, info->ip_len, sizeof(*udph), &__udph);
		if (!udph) {
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_HDRP_FAIL);
			goto out;
		}

		tuple->port = udph->dest;
		if (rmnet_wlan_udp_encap_check(skb, tuple, info->ip_len)) {
			if (rmnet_wlan_udp_encap_drop_check(tuple)) {
				kfree_skb(skb);

				ret = 0;
				rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_DROP);
				goto out;
			}
			/* Let the stack handle this packet */
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_DELIVER);
			goto encap;
		}
	} else if (tuple->trans_proto == IPPROTO_ESP) {
		struct ip_esp_hdr *esph, __esph;

		esph = skb_header_pointer(skb, info->ip_len, sizeof(*esph), &__esph);
		if (!esph) {
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_HDRP_FAIL);
			goto out;
		}

		tuple->spi_val = esph->spi;
	}

	if (rmnet_wlan_tuple_present(tuple)) {
		u32 stat;

		/* Match found. Go ahead and pass it on, and store
		 * this decision for the later fragments.
		 */
		node->fwd = fwd_info;
		if (!rmnet_wlan_deliver_skb(skb, fwd_info)) {
			stat = RMNET_WLAN_STAT_FRAG_FWD;
			ret = 0;
		} else {
			stat = RMNET_WLAN_STAT_FRAG_FWD_NO_DEV;
		}

		rmnet_wlan_stats_update(stat);
		/* Now that we know where to forward, forward! */
		rmnet_wlan_flush_fragment_node(node, true);
		goto out;
	}

encap:
	/* Not a fragment we're interested in. Remember that */
	node->fwd = ERR_PTR(-EINVAL);
	/* Flush anything we held before we found this */
	rmnet_wlan_flush_fragment_node(node, true);

out:
	if (ret)
		/* Make sure to reset as we are not requeuing the packet */
		shinfo->tskey = 0;

	return ret;
}

int rmnet_wlan_fragment_v4(struct sk_buff *skb, int ip_len,
			   struct rmnet_wlan_tuple *tuple,
			   struct rmnet_wlan_fwd_info *fwd_info)
	__must_hold(RCU)
{
	struct rmnet_wlan_fragment_info info = {};
	struct iphdr *iph = ip_hdr(skb);

	/* Only deal with this rigmarole if we can't escape it */
	if (tuple->trans_proto != IPPROTO_TCP &&
	    tuple->trans_proto != IPPROTO_UDP &&
	    tuple->trans_proto != IPPROTO_ESP)
		return -1;

	info.ip_proto = 4;
	info.v4_saddr = iph->saddr;
	info.v4_daddr = iph->daddr;
	/* Endian up-casting is messy, ain't it~? */
	info.id = htonl((u32)ntohs(iph->id));
	info.offset = htons(iph->frag_off) & IP_OFFSET;
	info.ip_len = (u16)ip_len;
	return rmnet_wlan_fragment_handle(skb, tuple, &info, fwd_info);
}

int rmnet_wlan_fragment_v6(struct sk_buff *skb, int ip_len,
			   struct rmnet_wlan_tuple *tuple,
			   struct rmnet_wlan_fwd_info *fwd_info)
	__must_hold(RCU)
{
	struct rmnet_wlan_fragment_info info = {};
	struct ipv6hdr *ip6h = ipv6_hdr(skb);
	struct frag_hdr *fragh, __fragh;
	unsigned int ptr;

	/* V6 fragments are harder to deal with, since you won't know the
	 * actual transport protocol in any secondary fragments...
	 */
	if (tuple->trans_proto != IPPROTO_TCP &&
	    tuple->trans_proto != IPPROTO_UDP &&
	    tuple->trans_proto != IPPROTO_ESP &&
	    tuple->trans_proto != NEXTHDR_FRAGMENT)
		return -1;

	/* Grab that frag header! */
	if (rmnet_wlan_ipv6_find_hdr(skb, &ptr, NEXTHDR_FRAGMENT, NULL, NULL)
	    < 0)
		/* ...or not, somehow? */
		return -1;

	fragh = skb_header_pointer(skb, ptr, sizeof(*fragh), &__fragh);
	if (!fragh) {
		rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_HDRP_FAIL);
		return -1;
	}

	info.ip_proto = 6;
	memcpy(&info.v6_saddr, &ip6h->saddr, sizeof(ip6h->saddr));
	memcpy(&info.v6_daddr, &ip6h->daddr, sizeof(ip6h->daddr));
	info.id = fragh->identification;
	info.offset = htons(fragh->frag_off) & IP6_OFFSET;
	info.ip_len = (u16)ip_len;

	/* Account for the the fact that non-secondary fragments won't
	 * handle the fragment header length.
	 */
	if (tuple->trans_proto == NEXTHDR_FRAGMENT)
		info.ip_len += sizeof(*fragh);

	return rmnet_wlan_fragment_handle(skb, tuple, &info, fwd_info);
}

int rmnet_wlan_fragment_init(void)
{
	INIT_DELAYED_WORK(&rmnet_wlan_fragment_work.ws,
			  rmnet_wlan_fragment_work_process);
	return 0;
}

void rmnet_wlan_fragments_remove(void)
{
	/* Force the current work struct to finish deleting anything old
	 * enough...
	 */
	cancel_delayed_work_sync(&rmnet_wlan_fragment_work.ws);

	rmnet_wlan_fragment_work.force_clean = true;
	schedule_delayed_work(&rmnet_wlan_fragment_work.ws, 0);

	/* ...and orce remove all the rest of the nodes */
	cancel_delayed_work_sync(&rmnet_wlan_fragment_work.ws);
}

void rmnet_wlan_fragment_del_fwd_info(struct rmnet_wlan_fwd_info *info)
{
	struct rmnet_wlan_fragment_node *node;
	int bkt;

	rcu_read_lock();
	hash_for_each_rcu(rmnet_wlan_fragment_hash, bkt, node, hash) {
		/* Poison anything that is using the info */
		if (node->fwd == info)
			node->fwd = ERR_PTR(-EINVAL);
	}

	rcu_read_unlock();
}
