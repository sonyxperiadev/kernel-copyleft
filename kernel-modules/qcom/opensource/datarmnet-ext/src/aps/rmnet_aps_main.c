// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/*
 * Apps prioritization service
 */
#include <linux/timer.h>
#include <linux/list_sort.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include "rmnet_config.h"
#include "rmnet_module.h"
#include "rmnet_descriptor.h"
#include "rmnet_qmap.h"
#include "rmnet_aps.h"
#include "rmnet_aps_genl.h"

/* Insert newest first, last 4 bytes of the change id */
static char *verinfo[] = {
	"58aa9bee",
	"b079eb7a",
	"8dedc718",
	"65539b84",
	"e218f451",
	"32a6eba9",
	"6905568e",
	"7415921c",
	"9f6681b4",
};

module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

#define APS_CMD_INIT 1
#define APS_CMD_ADD_FLOW 2
#define APS_CMD_DEL_FLOW 3
#define APS_CMD_UPD_FLOW 4
#define APS_CMD_FLOW_REMOVED 5
#define APS_CMD_ADD_FILTER 6

#define APS_FLOW_REMOVED_EXPIRED 1
#define APS_FLOW_REMOVED_NO_LONGER_VALID 2
#define APS_FLOW_REMOVED_RESET 3

#define APS_PRIORITY_UNSPECIFIED 0
#define APS_MAX_FLOW_CNT 255
#define APS_MAX_PRIO 5

#define APS_RATE_TIMER_SEC (3)
#define APS_RATE_TIMER_JIFF (APS_RATE_TIMER_SEC * HZ)
#define APS_LLC_MAX_BYTES (500 * 1024 * 1024 / 8 * APS_RATE_TIMER_SEC)
#define APS_LLB_MAX_BYTES (200 * 1024 * 1024 / 8 * APS_RATE_TIMER_SEC)

static const char *rmnet_aps_version = "1.4";

static u16 aps_user_cookie;
static struct genl_info aps_client_genl_info;

static bool rmnet_aps_active;
static u32 rmnet_aps_flow_cnt;

/* this is configured so that it works also for pfifo_fast.
 * prio 0-4 maps to band 1, 0, 0, 1, 1 in pfifo_fast,
 * and maps to 3, 0, 1, 2, 3 in rmnet_sch.
 */
static u8 aps2linux_prio_map[APS_MAX_PRIO] = { 0, 6, 7, 8, 9 };

/* mutex for aps flow lists */
DEFINE_MUTEX(rmnet_aps_mutex);

/* spinlock for aps flow update */
static DEFINE_SPINLOCK(rmnet_aps_lock);

/* Access to this list needs aps mutex */
static LIST_HEAD(aps_flow_list);

struct rmnet_aps_filter {
	struct rcu_head rcu;
	struct rmnet_aps_filter_req info;
};

struct rmnet_aps_flow {
	struct rcu_head rcu;
	struct list_head dev_list;
	struct list_head sorted_list;
	struct rmnet_aps_flow_req info;
	struct rmnet_aps_filter __rcu *filter;
	u32 skb_prio;
	u32 duration_jfs;
	unsigned long expires;
	u64 tx_bytes;
	bool use_llc;
	bool use_llb;
};

struct rmnet_aps_flow_list {
	struct rcu_head rcu;
	struct list_head list;
};

/* APS control buffer. The size should be less than rmnet_priv->aps_cb */
struct rmnet_aps_cb {
	char do_tx_tstamp;
	char reserved[7];
	struct rmnet_aps_flow_list __rcu *flow_list;
};

#define IS_RMNET_DEV(dev) (!strncmp(dev->name, "rmnet_data", 10))
#define RMNET_APS_CB(dev)                                                      \
	((struct rmnet_aps_cb *)(((struct rmnet_priv *)netdev_priv(dev))       \
					 ->aps_cb))

extern struct genl_family rmnet_aps_genl_family;

static void rmnet_aps_set_skb_prio(struct net_device *dev, struct sk_buff *skb);
static void rmnet_aps_flow_removed_ind(u32 label, u8 reason);

/* timer for flow expiry */
static void rmnet_aps_timer_fn(struct timer_list *t);
static DEFINE_TIMER(rmnet_aps_timer, rmnet_aps_timer_fn);

/* expire work to remove expired flows */
static void rmnet_aps_flow_expire(struct work_struct *work);
static DECLARE_WORK(rmnet_aps_flow_expire_work, rmnet_aps_flow_expire);

/* rate work to adjust priority */
static void rmnet_aps_adjust_prio(struct work_struct *work);
static DECLARE_DELAYED_WORK(rmnet_aps_rate_work, rmnet_aps_adjust_prio);

static int rmnet_aps_notifier_cb(struct notifier_block *nb,
				 unsigned long action, void *data);

static struct notifier_block rmnet_aps_notifier __read_mostly = {
	.notifier_call = rmnet_aps_notifier_cb,
	.priority = 2,
};

/*
 * Flow dissector
 */
struct aps_dissect_info {
	__u8 ip_ver;
	__u8 is_frag;
	__u8 is_first_frag;
	__u8 l4_proto;
	__be32 saddr[4];
	__be32 daddr[4];
	__u16 sport;
	__u16 dport;
	__u32 flow_label;
	__u8 tos;
};

static int aps_dissect_skb(struct sk_buff *skb, struct aps_dissect_info *di)
{
	struct ipports {
		__be16 sport;
		__be16 dport;
	};

	int offset;

	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, _iph;
		iph = __skb_header_pointer(skb, 0, sizeof(*iph), skb->data,
					   skb_headlen(skb), &_iph);
		if (unlikely(!iph || iph->ihl < 5))
			return -EINVAL;
		if (unlikely(ip_is_fragment(iph))) {
			di->is_frag = 1;
			di->is_first_frag = !(iph->frag_off & htons(IP_OFFSET));
		}
		offset = iph->ihl << 2;
		di->ip_ver = 0x4;
		di->l4_proto = iph->protocol;
		di->saddr[0] = iph->saddr;
		di->daddr[0] = iph->daddr;
		di->tos = iph->tos;
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, _ip6h;
		__be16 frag_off = 0;
		u8 nexthdr;
		ip6h = __skb_header_pointer(skb, 0, sizeof(*ip6h), skb->data,
					    skb_headlen(skb), &_ip6h);
		if (unlikely(!ip6h))
			return -EINVAL;

		nexthdr = ip6h->nexthdr;
		offset = ipv6_skip_exthdr(skb, sizeof(*ip6h), &nexthdr,
					  &frag_off);
		if (unlikely(offset < 0))
			return -EFAULT;
		if (unlikely(frag_off)) {
			di->is_frag = 1;
			di->is_first_frag = !(frag_off & htons(IP6_OFFSET));
		}
		di->ip_ver = 0x6;
		di->l4_proto = nexthdr;
		memcpy(di->saddr, ip6h->saddr.in6_u.u6_addr8, 16);
		memcpy(di->daddr, ip6h->daddr.in6_u.u6_addr8, 16);
		di->tos = ip6_tclass(ip6_flowinfo(ip6h));
		di->flow_label = ntohl(ip6_flowlabel(ip6h));

	} else {
		return -EINVAL;
	}

	if (di->is_frag && !di->is_first_frag)
		return 0;

	if (di->l4_proto == IPPROTO_TCP || di->l4_proto == IPPROTO_UDP) {
		struct ipports *ports, _ports;
		ports = __skb_header_pointer(skb, offset, sizeof(*ports),
					     skb->data, skb_headlen(skb),
					     &_ports);
		if (unlikely(!ports))
			return -EINVAL;
		di->sport = ntohs(ports->sport);
		di->dport = ntohs(ports->dport);
	}

	return 0;
}

/*
 * Dissect rmnet frag descritptor
 */
static int aps_dissect_desc(struct rmnet_frag_descriptor *frag_desc,
			    struct aps_dissect_info *di)
{
	struct ipports {
		__be16 sport;
		__be16 dport;
	};

	int offset;
	u8 *data;

	data = rmnet_frag_data_ptr(frag_desc);
	if (unlikely(!data))
		return -EINVAL;

	if ((data[0] & 0xF0) == 0x40) {
		struct iphdr *iph, _iph;
		iph = rmnet_frag_header_ptr(frag_desc, 0, sizeof(*iph), &_iph);
		if (unlikely(!iph || iph->ihl < 5))
			return -EINVAL;
		if (unlikely(ip_is_fragment(iph))) {
			di->is_frag = 1;
			di->is_first_frag = !(iph->frag_off & htons(IP_OFFSET));
		}
		offset = iph->ihl << 2;
		di->ip_ver = 0x4;
		di->l4_proto = iph->protocol;
		di->saddr[0] = iph->saddr;
		di->daddr[0] = iph->daddr;
		di->tos = iph->tos;
	} else if ((data[0] & 0xF0) == 0x60) {
		struct ipv6hdr *ip6h, _ip6h;
		__be16 frag_off;
		u8 nexthdr;
		bool frag_hdr;

		ip6h = rmnet_frag_header_ptr(frag_desc, 0, sizeof(*ip6h),
					     &_ip6h);
		if (unlikely(!ip6h))
			return -EINVAL;

		nexthdr = ip6h->nexthdr;
		offset = rmnet_frag_ipv6_skip_exthdr(frag_desc, sizeof(*ip6h),
						     &nexthdr, &frag_off,
						     &frag_hdr);
		if (unlikely(offset < 0))
			return -EFAULT;
		if (unlikely(frag_off || frag_hdr)) {
			di->is_frag = 1;
			di->is_first_frag = !(frag_off & htons(IP6_OFFSET));
		}
		di->ip_ver = 0x6;
		di->l4_proto = nexthdr;
		memcpy(di->saddr, ip6h->saddr.in6_u.u6_addr8, 16);
		memcpy(di->daddr, ip6h->daddr.in6_u.u6_addr8, 16);
		di->tos = ip6_tclass(ip6_flowinfo(ip6h));
		di->flow_label = ntohl(ip6_flowlabel(ip6h));

	} else {
		return -EINVAL;
	}

	if (di->is_frag && !di->is_first_frag)
		return 0;

	if (di->l4_proto == IPPROTO_TCP || di->l4_proto == IPPROTO_UDP) {
		struct ipports *ports, _ports;
		ports = rmnet_frag_header_ptr(frag_desc, offset, sizeof(*ports),
					      &_ports);
		if (unlikely(!ports))
			return -EINVAL;
		di->sport = ntohs(ports->sport);
		di->dport = ntohs(ports->dport);
	}

	return 0;
}

/*
 * Return true if skb dissected info matches filter.
 * Assume IP version is already matched by the caller
 */
static bool aps_match_filter(struct rmnet_aps_filter_req *filter,
			     struct aps_dissect_info *di)
{
	/* L4 protocol */
	if (filter->l4_proto) {
		if (filter->l4_proto == 253) {
			if (di->l4_proto != IPPROTO_TCP &&
			    di->l4_proto != IPPROTO_UDP)
				return false;
		} else if (filter->l4_proto != di->l4_proto) {
			return false;
		}
	}

	/* Ports */
	if (di->l4_proto == IPPROTO_TCP || di->l4_proto == IPPROTO_UDP) {
		if ((filter->dport && filter->dport != di->dport) ||
		    (filter->sport && filter->sport != di->sport))
			return false;
	}

	/* Address and others */
	if (filter->ip_type == AF_INET) {
		if (((filter->filter_masks & FILTER_MASK_DADDR) &&
		     filter->daddr[0] != di->daddr[0]) ||
		    ((filter->filter_masks & FILTER_MASK_SADDR) &&
		     filter->saddr[0] != di->saddr[0]))
			return false;
		if (filter->tos && filter->tos != (di->tos & filter->tos_mask))
			return false;
	} else if (filter->ip_type == AF_INET6) {
		if (((filter->filter_masks & FILTER_MASK_DADDR) &&
		     memcmp(filter->daddr, di->daddr, 16)) ||
		    ((filter->filter_masks & FILTER_MASK_SADDR) &&
		     memcmp(filter->saddr, di->saddr, 16)))
			return false;
		if (filter->tos && filter->tos != (di->tos & filter->tos_mask))
			return false;
		if (filter->flow_label && filter->flow_label != di->flow_label)
			return false;
	} else {
		return false;
	}

	return true;
}

static struct rmnet_aps_flow *
rmnet_aps_match_flow(struct list_head *dev_flow_list, struct sk_buff *skb)
{
	struct aps_dissect_info di = {
		0,
	};
	struct rmnet_aps_flow *flow;
	struct rmnet_aps_filter *filter;
	bool skb_dissected = false;

	list_for_each_entry_rcu (flow, dev_flow_list, dev_list) {
		filter = rcu_dereference(flow->filter);
		if (unlikely(!filter))
			continue;

		/* match ip type */
		if ((filter->info.ip_type == AF_INET &&
		     skb->protocol != htons(ETH_P_IP)) ||
		    (filter->info.ip_type == AF_INET6 &&
		     skb->protocol != htons(ETH_P_IPV6)))
			continue;

		if (!skb_dissected) {
			if (aps_dissect_skb(skb, &di) || di.is_frag)
				break;
			skb_dissected = true;
		}

		if (aps_match_filter(&filter->info, &di))
			return flow;
	}

	return NULL;
}

/*
 * Find flow from label
 */
static struct rmnet_aps_flow *
rmnet_aps_find_flow(struct list_head *dev_flow_list, u32 label)
{
	struct rmnet_aps_flow *flow;

	list_for_each_entry_rcu (flow, dev_flow_list, dev_list) {
		if (flow->info.label == label)
			return flow;
	}

	return NULL;
}

/*
 * Add a flow
 */
static void rmnet_aps_add_flow(struct list_head *dev_flow_list,
			       struct rmnet_aps_flow *flow)
{
	list_add_rcu(&flow->dev_list, dev_flow_list);
	list_add(&flow->sorted_list, &aps_flow_list);
	rmnet_aps_flow_cnt++;

	if (flow->expires) {
		if (!timer_pending(&rmnet_aps_timer) ||
		    time_before(flow->expires, rmnet_aps_timer.expires))
			mod_timer(&rmnet_aps_timer, flow->expires);
	}

	aps_log("aps: flow 0x%x added (%u)\n", flow->info.label,
		rmnet_aps_flow_cnt);
}

/*
 * Remove a flow
 */
static void rmnet_aps_remove_flow(struct rmnet_aps_flow *flow)
{
	struct rmnet_aps_filter *filter;

	list_del_rcu(&flow->dev_list);
	list_del(&flow->sorted_list);

	if (likely(rmnet_aps_flow_cnt))
		rmnet_aps_flow_cnt--;

	if (!rmnet_aps_flow_cnt)
		del_timer(&rmnet_aps_timer);

	aps_log("aps: flow 0x%x removing (%u)\n", flow->info.label,
		rmnet_aps_flow_cnt);

	filter = rcu_dereference(flow->filter);
	if (filter) {
		rcu_assign_pointer(flow->filter, NULL);
		kfree_rcu(flow->filter, rcu);
	}
	kfree_rcu(flow, rcu);
}

static void rmnet_aps_remove_all(void)
{
	struct rmnet_aps_flow *flow, *tmp;

	list_for_each_entry_safe (flow, tmp, &aps_flow_list, sorted_list) {
		rmnet_aps_remove_flow(flow);
	}

	aps_log("aps: all flows removed\n");
}

static void rmnet_aps_remove_iface(int ifindex)
{
	struct rmnet_aps_flow *flow, *tmp;

	list_for_each_entry_safe (flow, tmp, &aps_flow_list, sorted_list) {
		if (flow->info.ifindex == ifindex) {
			aps_log("aps: flow 0x%x down\n", flow->info.label);
			rmnet_aps_flow_removed_ind(
				flow->info.label,
				APS_FLOW_REMOVED_NO_LONGER_VALID);
			rmnet_aps_remove_flow(flow);
		}
	}
}

/*
 * Remove all expired flows and set timer to next expiration.
 */
static void rmnet_aps_flow_expire(struct work_struct *work)
{
	struct rmnet_aps_flow *flow, *tmp;
	unsigned long now = jiffies;
	unsigned long expires_next;
	u32 label;

	expires_next = now;

	mutex_lock(&rmnet_aps_mutex);

	if (!rmnet_aps_active)
		goto out;

	spin_lock_bh(&rmnet_aps_lock);

	list_for_each_entry_safe (flow, tmp, &aps_flow_list, sorted_list) {
		if (!flow->expires)
			continue;
		if (time_before_eq(flow->expires, now)) {
			label = flow->info.label;
			aps_log("aps: flow 0x%x expired\n", label);
			rmnet_aps_remove_flow(flow);
			rmnet_aps_flow_removed_ind(label,
						   APS_FLOW_REMOVED_EXPIRED);
		} else if (expires_next == now ||
			   time_before(flow->expires, expires_next)) {
			expires_next = flow->expires;
		}
	}

	if (expires_next != now)
		mod_timer(&rmnet_aps_timer, expires_next);

	spin_unlock_bh(&rmnet_aps_lock);
out:
	mutex_unlock(&rmnet_aps_mutex);
}

static void rmnet_aps_timer_fn(struct timer_list *t)
{
	schedule_work(&rmnet_aps_flow_expire_work);
}

static void rmnet_aps_cal_skb_prio(struct rmnet_aps_flow *flow)
{
	u32 skb_prio;

	/* bits 0 - 7 : linux priority
	 * bit 8: LLC
	 * bit 9: QMAP LL bit
	 * bits 16 - 31: cookie
	*/

	skb_prio = RMNET_APS_MAJOR << 16;

	if (flow->info.aps_prio >= APS_MAX_PRIO)
		skb_prio |= aps2linux_prio_map[0];
	else
		skb_prio |= aps2linux_prio_map[flow->info.aps_prio];

	if (flow->use_llc)
		skb_prio |= RMNET_APS_LLC_MASK;

	if (flow->use_llb)
		skb_prio |= RMNET_APS_LLB_MASK;

	flow->skb_prio = skb_prio;
}

static int rmnet_aps_rate_cmp(void *priv, const struct list_head *a,
			      const struct list_head *b)
{
	struct rmnet_aps_flow *flow_a;
	struct rmnet_aps_flow *flow_b;

	flow_a = list_entry(a, struct rmnet_aps_flow, sorted_list);
	flow_b = list_entry(b, struct rmnet_aps_flow, sorted_list);

	return flow_a->tx_bytes > flow_b->tx_bytes;
}

static void rmnet_aps_adjust_prio(struct work_struct *work)
{
	struct rmnet_aps_flow *flow;
	u64 llc_bytes = 0;
	u64 llb_bytes = 0;
	u64 tmp_bytes;
	bool again = false;

	mutex_lock(&rmnet_aps_mutex);

	if (!rmnet_aps_active)
		goto out;

	spin_lock_bh(&rmnet_aps_lock);

	/* sort the list by acending tx bytes */
	list_sort(NULL, &aps_flow_list, rmnet_aps_rate_cmp);

	list_for_each_entry (flow, &aps_flow_list, sorted_list) {
		if (flow->info.use_llc) {
			tmp_bytes = llc_bytes + flow->tx_bytes;
			if (tmp_bytes < APS_LLC_MAX_BYTES) {
				flow->use_llc = true;
				flow->skb_prio |= RMNET_APS_LLC_MASK;
				llc_bytes = tmp_bytes;
			} else {
				flow->use_llc = false;
				flow->skb_prio &= ~RMNET_APS_LLC_MASK;
			}
			aps_log("aps: flow 0x%x tx_bytes %llu llc %d llc_bytes "
				"%llu\n",
				flow->info.label, flow->tx_bytes, flow->use_llc,
				llc_bytes);
		}
		if (flow->info.use_llb) {
			tmp_bytes = llb_bytes + flow->tx_bytes;
			if (tmp_bytes < APS_LLB_MAX_BYTES) {
				flow->use_llb = true;
				flow->skb_prio |= RMNET_APS_LLB_MASK;
				llb_bytes = tmp_bytes;
			} else {
				flow->use_llb = false;
				flow->skb_prio &= ~RMNET_APS_LLB_MASK;
			}
			aps_log("aps: flow 0x%x tx_bytes %llu llb %d llb_bytes "
				"%llu\n",
				flow->info.label, flow->tx_bytes, flow->use_llb,
				llb_bytes);
		}
		if (flow->tx_bytes) {
			again = true;
			flow->tx_bytes = 0;
		}
	}

	spin_unlock_bh(&rmnet_aps_lock);
out:
	mutex_unlock(&rmnet_aps_mutex);

	if (again)
		schedule_delayed_work(&rmnet_aps_rate_work,
				      APS_RATE_TIMER_JIFF);
}

static int rmnet_aps_change_flow(struct list_head *dev_flow_list,
				 struct rmnet_aps_flow_req *req)
{
	struct rmnet_aps_flow *flow;

	flow = rmnet_aps_find_flow(dev_flow_list, req->label);

	if (req->cmd == APS_CMD_DEL_FLOW) {
		/* delete flow */
		if (flow)
			rmnet_aps_remove_flow(flow);
		return 0;
	}

	if (req->cmd == APS_CMD_ADD_FLOW && flow) {
		/* remove before add */
		rmnet_aps_remove_flow(flow);
		flow = NULL;
	}

	if (!flow) {
		/* add flow */
		if (rmnet_aps_flow_cnt >= APS_MAX_FLOW_CNT) {
			aps_log("aps: flow count %u exceeds max\n",
				rmnet_aps_flow_cnt);
			return -EINVAL;
		}

		flow = kzalloc(sizeof(*flow), GFP_KERNEL);
		if (!flow) {
			aps_log("aps: no memory\n");
			return -ENOMEM;
		}

		memcpy(&flow->info, req, sizeof(flow->info));
		flow->use_llc = flow->info.use_llc;
		flow->use_llb = flow->info.use_llb;
		rmnet_aps_cal_skb_prio(flow);

		if (req->duration) {
			flow->duration_jfs = req->duration * HZ;
			flow->expires = jiffies + flow->duration_jfs;
		}

		rmnet_aps_add_flow(dev_flow_list, flow);

	} else {
		/* update flow */
		spin_lock_bh(&rmnet_aps_lock);
		memcpy(&flow->info, req, sizeof(flow->info));
		flow->use_llc = flow->info.use_llc;
		flow->use_llb = flow->info.use_llb;
		rmnet_aps_cal_skb_prio(flow);
		spin_unlock_bh(&rmnet_aps_lock);

		aps_log("aps: flow 0x%x updated\n", flow->info.label);
	}

	return 0;
}

/*
 * Send NL message
 */
static int rmnet_aps_send_msg(struct genl_info *info, u8 cmd, int attr_type,
			      int attr_data_len, void *attr_data, gfp_t flags)
{
	struct sk_buff *skb = NULL;
	void *msg_head;
	int rc;

	if (unlikely(!info))
		return -EINVAL;

	skb = genlmsg_new(nla_total_size(attr_data_len), flags);
	if (!skb)
		goto out;

	msg_head = genlmsg_put(skb, 0, info->snd_seq + 1,
			       &rmnet_aps_genl_family, 0, cmd);
	if (!msg_head)
		goto out;

	rc = nla_put(skb, attr_type, attr_data_len, attr_data);
	if (rc)
		goto out;

	genlmsg_end(skb, msg_head);

	rc = genlmsg_unicast(genl_info_net(info), skb, info->snd_portid);
	if (rc) {
		skb = NULL;
		goto out;
	}

	return 0;

out:
	aps_log("aps: FAILED to send msg %d\n", cmd);
	if (skb)
		kfree_skb(skb);
	return -EFAULT;
}

static void rmnet_aps_flow_removed_ind(u32 label, u8 reason)
{
	struct rmnet_aps_flow_resp resp;

	if (!aps_client_genl_info.snd_portid) {
		aps_log("aps: client not connected\n");
		return;
	}

	resp.cmd = APS_CMD_FLOW_REMOVED;
	resp.cmd_data = reason;
	resp.label = label;

	if (rmnet_aps_send_msg(&aps_client_genl_info, RMNET_APS_GENL_CMD_FLOW,
			       RMNET_APS_GENL_ATTR_FLOW_RESP, sizeof(resp),
			       &resp, GFP_ATOMIC)) {
		aps_log("aps: client send failed. disable client\n");
		aps_client_genl_info.snd_portid = 0;
	}
}

static void rmnet_aps_set_skb_prio(struct net_device *dev, struct sk_buff *skb)
{
	struct rmnet_aps_flow *flow;
	struct rmnet_aps_cb *aps_cb;
	struct rmnet_aps_flow_list *fl;

	/* if aps_user_cookie is 0, we will do the filtering.
	 * Otherwise, userspace iptables is expected to do the filtering and
	 * put a unique label in the skb->priority field
	 */
	if (aps_user_cookie && (skb->priority >> 16 != aps_user_cookie))
		return;

	aps_cb = RMNET_APS_CB(dev);
	fl = rcu_dereference(aps_cb->flow_list);
	if (!fl)
		return;

	if (aps_user_cookie)
		flow = rmnet_aps_find_flow(&fl->list, skb->priority);
	else
		flow = rmnet_aps_match_flow(&fl->list, skb);

	if (flow) {
		spin_lock_bh(&rmnet_aps_lock);
		skb->priority = flow->skb_prio;
		flow->expires = jiffies + flow->duration_jfs;
		flow->tx_bytes += skb->len;
		spin_unlock_bh(&rmnet_aps_lock);

		schedule_delayed_work(&rmnet_aps_rate_work,
				      APS_RATE_TIMER_JIFF);
	}
}

static int rmnet_aps_notifier_cb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct net_device *dev;
	struct rmnet_aps_cb *aps_cb;
	struct rmnet_aps_flow_list *fl;

	dev = netdev_notifier_info_to_dev(data);
	if (unlikely(!dev))
		return NOTIFY_DONE;

	if (!IS_RMNET_DEV(dev))
		return NOTIFY_DONE;

	aps_cb = RMNET_APS_CB(dev);
	BUILD_BUG_ON(sizeof(*aps_cb) >
		     sizeof(((struct rmnet_priv *)0)->aps_cb));

	switch (action) {
	case NETDEV_DOWN:
		aps_cb->do_tx_tstamp = 0;
		mutex_lock(&rmnet_aps_mutex);
		rmnet_aps_remove_iface(dev->ifindex);
		mutex_unlock(&rmnet_aps_mutex);
		break;

	case NETDEV_UNREGISTER:
		mutex_lock(&rmnet_aps_mutex);
		rmnet_aps_remove_iface(dev->ifindex);
		fl = rcu_dereference(aps_cb->flow_list);
		if (fl) {
			WARN_ON(!list_empty(&fl->list));
			rcu_assign_pointer(aps_cb->flow_list, NULL);
			kfree_rcu(fl, rcu);
		}
		mutex_unlock(&rmnet_aps_mutex);
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

int rmnet_aps_genl_flow_hdlr(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_aps_flow_req req;
	struct rmnet_aps_flow_resp resp;
	struct net_device *dev = NULL;
	struct rmnet_aps_cb *aps_cb;
	struct rmnet_aps_flow_list *fl;
	int rc = -EINVAL;

	aps_log("aps: %s\n", __func__);

	if (!info)
		return -EINVAL;

	na = info->attrs[RMNET_APS_GENL_ATTR_FLOW_REQ];
	if (!na) {
		aps_log("aps: no attrs\n");
		return -EINVAL;
	}

	if (nla_memcpy(&req, na, sizeof(req)) <= 0) {
		aps_log("aps: nla_memcpy failed\n");
		return -EINVAL;
	}

	mutex_lock(&rmnet_aps_mutex);

	if (!rmnet_aps_active)
		goto out;

	switch (req.cmd) {
	case APS_CMD_INIT:
		aps_client_genl_info = *info;
		rmnet_aps_remove_all();
		aps_user_cookie = (u16)req.label;
		aps_log("aps: client init cookie 0x%x\n", aps_user_cookie);
		rc = 0;
		break;

	case APS_CMD_ADD_FLOW:
	case APS_CMD_DEL_FLOW:
	case APS_CMD_UPD_FLOW:
		if (req.ifindex)
			dev = dev_get_by_index(&init_net, req.ifindex);
		if (!dev) {
			aps_log("aps: no dev for ifindex %u\n", req.ifindex);
			break;
		}
		if (IS_RMNET_DEV(dev)) {
			aps_cb = RMNET_APS_CB(dev);
			fl = rcu_dereference(aps_cb->flow_list);
			if (!fl && req.cmd == APS_CMD_ADD_FLOW) {
				fl = kzalloc(sizeof(*fl), GFP_KERNEL);
				if (fl) {
					INIT_LIST_HEAD(&fl->list);
					rcu_assign_pointer(aps_cb->flow_list,
							   fl);
				} else {
					aps_log("aps: no memory\n");
				}
			}
			if (fl)
				rc = rmnet_aps_change_flow(&fl->list, &req);
			else
				aps_log("aps: no flow list for flow req\n");
		}
		dev_put(dev);
		break;

	default:
		break;
	}

out:
	mutex_unlock(&rmnet_aps_mutex);

	resp.cmd = req.cmd;
	resp.cmd_data = rc;
	resp.label = req.label;

	rc = rmnet_aps_send_msg(info, RMNET_APS_GENL_CMD_FLOW,
				RMNET_APS_GENL_ATTR_FLOW_RESP, sizeof(resp),
				&resp, GFP_KERNEL);

	return rc;
}

static int rmnet_aps_change_filter(struct list_head *dev_flow_list,
				   struct rmnet_aps_filter_req *req)
{
	struct rmnet_aps_flow *flow;
	struct rmnet_aps_filter *filter;

	flow = rmnet_aps_find_flow(dev_flow_list, req->label);
	if (!flow) {
		aps_log("aps: flow 0x%x not found\n", req->label);
		return -EINVAL;
	}

	switch (req->cmd) {
	case APS_CMD_ADD_FILTER:
		filter = rcu_dereference(flow->filter);
		if (filter) {
			aps_log("aps: filter for flow 0x%x exists\n",
				req->label);
			return -EINVAL;
		}
		filter = kzalloc(sizeof(*filter), GFP_KERNEL);
		if (!filter) {
			aps_log("aps: no memory\n");
			return -ENOMEM;
		}

		memcpy(&filter->info, req, sizeof(*req));
		if (filter->info.tos) {
			if (!filter->info.tos_mask)
				filter->info.tos_mask = 0xFF;
			filter->info.tos &= filter->info.tos_mask;
		}
		rcu_assign_pointer(flow->filter, filter);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

int rmnet_aps_genl_filter_hdlr(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_aps_filter_req req;
	struct rmnet_aps_filter_resp resp;
	struct net_device *dev = NULL;
	struct rmnet_aps_cb *aps_cb;
	struct rmnet_aps_flow_list *fl;
	int rc = -EINVAL;

	aps_log("aps: %s\n", __func__);

	if (!info)
		return -EINVAL;

	na = info->attrs[RMNET_APS_GENL_ATTR_FILTER_REQ];
	if (!na) {
		aps_log("aps: no attrs\n");
		return -EINVAL;
	}

	if (nla_memcpy(&req, na, sizeof(req)) <= 0) {
		aps_log("aps: nla_memcpy failed\n");
		return -EINVAL;
	}

	mutex_lock(&rmnet_aps_mutex);

	if (!rmnet_aps_active)
		goto out;

	if (req.ifindex)
		dev = dev_get_by_index(&init_net, req.ifindex);
	if (!dev) {
		aps_log("aps: no dev for ifindex %u\n", req.ifindex);
		goto out;
	}

	if (!IS_RMNET_DEV(dev)) {
		dev_put(dev);
		goto out;
	}

	switch (req.cmd) {
	case APS_CMD_ADD_FILTER:
		aps_cb = RMNET_APS_CB(dev);
		fl = rcu_dereference(aps_cb->flow_list);
		if (fl)
			rc = rmnet_aps_change_filter(&fl->list, &req);
		else
			aps_log("aps: no flow list\n");
		break;
	/* Filter is deleted when flow is deleted */
	default:
		break;
	}

	dev_put(dev);

out:
	mutex_unlock(&rmnet_aps_mutex);

	resp.cmd = req.cmd;
	resp.cmd_data = rc;
	resp.label = req.label;

	rc = rmnet_aps_send_msg(info, RMNET_APS_GENL_CMD_FILTER,
				RMNET_APS_GENL_ATTR_FILTER_RESP, sizeof(resp),
				&resp, GFP_KERNEL);

	return rc;
}

/*
 * Request to drop old packets for a PDN
 */

static ktime_t aps_packet_expire_kt;

int rmnet_aps_genl_pdn_config_hdlr(struct sk_buff *skb_2,
				   struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_aps_pdn_config_req req;
	struct rmnet_aps_pdn_config_resp resp;
	struct net_device *dev;
	struct rmnet_aps_cb *aps_cb;
	int rc;

	aps_log("aps: %s\n", __func__);

	if (!info)
		return -EINVAL;

	na = info->attrs[RMNET_APS_GENL_ATTR_PDN_CONFIG_REQ];
	if (!na) {
		aps_log("aps: no attrs\n");
		return -EINVAL;
	}

	if (nla_memcpy(&req, na, sizeof(req)) <= 0) {
		aps_log("aps: nla_memcpy failed\n");
		return -EINVAL;
	}

	aps_log("aps: ifindex %u expire_ms %u\n", req.ifindex, req.expire_ms);

	/* currently expiration time is global not per-PDN */
	if (req.expire_ms) {
		ktime_t kt = ms_to_ktime(req.expire_ms);
		if (aps_packet_expire_kt != kt)
			aps_packet_expire_kt = kt;
	}

	dev = dev_get_by_index(&init_net, req.ifindex);
	if (dev) {
		if (IS_RMNET_DEV(dev)) {
			aps_cb = RMNET_APS_CB(dev);
			aps_cb->do_tx_tstamp = req.expire_ms ? 1 : 0;
		}
		dev_put(dev);
	}

	memset(&resp, 0, sizeof(resp));
	resp.ifindex = req.ifindex;

	rc = rmnet_aps_send_msg(info, RMNET_APS_GENL_CMD_PDN_CONFIG,
				RMNET_APS_GENL_ATTR_PDN_CONFIG_RESP,
				sizeof(resp), &resp, GFP_KERNEL);

	return rc;
}

static void rmnet_aps_do_pre_queue(struct net_device *dev, struct sk_buff *skb)
{
	struct rmnet_aps_cb *aps_cb = RMNET_APS_CB(dev);

	if (aps_cb->do_tx_tstamp)
		skb_hwtstamps(skb)->hwtstamp = ktime_get_boottime();
	else
		skb_hwtstamps(skb)->hwtstamp = 0;

	if (READ_ONCE(rmnet_aps_flow_cnt))
		rmnet_aps_set_skb_prio(dev, skb);
}

static int rmnet_aps_do_post_queue(struct net_device *dev, struct sk_buff *skb)
{
	struct rmnet_aps_cb *aps_cb = RMNET_APS_CB(dev);
	ktime_t curr;

	if (aps_cb->do_tx_tstamp && skb_hwtstamps(skb)->hwtstamp &&
	    aps_packet_expire_kt) {
		curr = ktime_get_boottime();
		if (ktime_sub(curr, skb_hwtstamps(skb)->hwtstamp) >
		    aps_packet_expire_kt)
			return -ETIMEDOUT;
	}

	return 0;
}

static const struct rmnet_module_hook_register_info rmnet_aps_module_hooks[] = {
	{
		.hooknum = RMNET_MODULE_HOOK_APS_PRE_QUEUE,
		.func = rmnet_aps_do_pre_queue,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_APS_POST_QUEUE,
		.func = rmnet_aps_do_post_queue,
	},
};

/*
 * Capture the 1st DL packet that wakes up rmnet
 */

static struct proc_dir_entry *aps_proc_dir;
static struct proc_dir_entry *aps_activity_file;

/* spinlock for activity update */
static DEFINE_SPINLOCK(dl_activity_lock);

struct activity_record {
	ktime_t ts;
	struct aps_dissect_info pkt;
};

#define DL_ACTIVITY_MAX_RECORDS 10
static int next_activity_index;
static struct activity_record dl_activity_records[DL_ACTIVITY_MAX_RECORDS];
static unsigned int inactive_time;
static ktime_t inactive_ktime;
static ktime_t last_inactive_ts;

/* Print all the captured packets to the proc file */
static int aps_activity_show(struct seq_file *s, void *d)
{
	struct activity_record *r;
	s64 sec, dd;
	u8 hh, mm, ss;
	int i;

	spin_lock_bh(&dl_activity_lock);

	/* Start from the oldest record */
	if (!dl_activity_records[next_activity_index].ts)
		i = 0;
	else
		i = next_activity_index;

	do {
		r = &dl_activity_records[i];
		if (!r->ts)
			break;

		sec = ktime_divns(r->ts, NSEC_PER_SEC);
		dd = sec / 86400;
		hh = sec % 86400 / 3600;
		mm = sec % 3600 / 60;
		ss = sec % 60;

		if (r->pkt.l4_proto == IPPROTO_TCP ||
		    r->pkt.l4_proto == IPPROTO_UDP) {
			if (r->pkt.ip_ver == 0x4)
				seq_printf(s, "[%6lld %02d:%02d:%02d] %2u "
					      "%pI4/%u %pI4/%u\n",
					   dd, hh, mm, ss, r->pkt.l4_proto,
					   r->pkt.saddr, r->pkt.sport,
					   r->pkt.daddr, r->pkt.dport);
			else
				seq_printf(s, "[%6lld %02d:%02d:%02d] %2u "
					      "%pI6/%u %pI6/%u\n",
					   dd, hh, mm, ss, r->pkt.l4_proto,
					   r->pkt.saddr, r->pkt.sport,
					   r->pkt.daddr, r->pkt.dport);
		} else {
			if (r->pkt.ip_ver == 0x4)
				seq_printf(s, "[%6lld %02d:%02d:%02d] %2u %pI4 "
					      "%pI4\n",
					   dd, hh, mm, ss, r->pkt.l4_proto,
					   r->pkt.saddr, r->pkt.daddr);
			else
				seq_printf(s, "[%6lld %02d:%02d:%02d] %2u %pI6 "
					      "%pI6\n",
					   dd, hh, mm, ss, r->pkt.l4_proto,
					   r->pkt.saddr, r->pkt.daddr);
		}

		if (++i >= DL_ACTIVITY_MAX_RECORDS)
			i = 0;

	} while (i != next_activity_index);

	spin_unlock_bh(&dl_activity_lock);

	return 0;
}

/* Data inactivity hook */
static void aps_data_inactive(void)
{
	spin_lock_bh(&dl_activity_lock);
	last_inactive_ts = ktime_get_coarse_boottime();
	spin_unlock_bh(&dl_activity_lock);
}

/* Data activity hook */
static void aps_data_active(struct rmnet_frag_descriptor *frag_desc,
			    struct sk_buff *skb)
{
	struct aps_dissect_info di = {
		0,
	};
	struct activity_record *r;
	ktime_t current_ts;
	int ret;

	if (!frag_desc && !skb)
		return;

	spin_lock_bh(&dl_activity_lock);

	if (!inactive_ktime)
		goto out;

	/* Store the dissected packet */
	current_ts = ktime_get_coarse_boottime();
	if (ktime_sub(current_ts, last_inactive_ts) > inactive_ktime) {
		if (frag_desc)
			ret = aps_dissect_desc(frag_desc, &di);
		else if (skb)
			ret = aps_dissect_skb(skb, &di);
		else
			ret = -EINVAL;

		if (ret || (di.is_frag && !di.is_first_frag))
			goto out;

		r = &dl_activity_records[next_activity_index];
		r->ts = current_ts;
		r->pkt = di;
		if (++next_activity_index >= DL_ACTIVITY_MAX_RECORDS)
			next_activity_index = 0;
	}
out:
	spin_unlock_bh(&dl_activity_lock);
}

static const struct rmnet_module_hook_register_info aps_data_activity_hooks[] = {
	{
		.hooknum = RMNET_MODULE_HOOK_APS_DATA_INACTIVE,
		.func = aps_data_inactive,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_APS_DATA_ACTIVE,
		.func = aps_data_active,
	},
};

/* Module params for the inactive time after which packet is captured */
static int aps_set_inactive_time(const char *val, const struct kernel_param *kp)
{
	unsigned int n;

	if (kstrtouint(val, 10, &n))
		return -EINVAL;

	spin_lock_bh(&dl_activity_lock);

	if (n) {
		inactive_ktime = ms_to_ktime(n * 1000);
		last_inactive_ts = ktime_get_coarse_boottime();
		rmnet_module_hook_register(aps_data_activity_hooks,
					   ARRAY_SIZE(aps_data_activity_hooks));
	} else {
		inactive_ktime = 0;
		rmnet_module_hook_unregister_no_sync(
			aps_data_activity_hooks,
			ARRAY_SIZE(aps_data_activity_hooks));
	}

	spin_unlock_bh(&dl_activity_lock);

	return param_set_uint(val, kp);
}

static const struct kernel_param_ops inactive_time_param_ops = {
	.set = aps_set_inactive_time,
	.get = param_get_uint,
};

module_param_cb(inactive_time, &inactive_time_param_ops, &inactive_time, 0644);
MODULE_PARM_DESC(inactive_time, "Inactive time in seconds");

static void aps_create_proc_files(void)
{
	aps_proc_dir = proc_mkdir("aps", NULL);
	if (!aps_proc_dir) {
		pr_err("aps: failed to create proc dir\n");
		return;
	}

	aps_activity_file = proc_create_single_data(
		"dl_activity", 0444, aps_proc_dir, aps_activity_show, NULL);
	if (!aps_activity_file)
		pr_err("aps: failed to create dl_activity entry\n");
}

static void aps_remove_proc_files(void)
{
	proc_remove(aps_activity_file);
	proc_remove(aps_proc_dir);
}

/*
 * Data Report
 */

static bool data_report_enabled = true;
static u32 data_report_count;

/* spinlock for data report transactions */
static DEFINE_SPINLOCK(data_report_lock);

static LIST_HEAD(data_report_req_list);

static void data_report_timer_fn(struct timer_list *t);
static DEFINE_TIMER(data_report_timer, data_report_timer_fn);

struct data_report_req_info {
	struct list_head list;
	struct genl_info info;
	__be32 tx_id;
	unsigned long expires;
};

struct qmap_data_report {
	struct qmap_cmd_hdr hdr;
	u8 cmd_ver;
	u8 type;
	u8 sum_all_bearers;
	u8 len;
	__be32 value[8];
} __aligned(1);

static int qmap_send_data_report_query(u8 type, u8 mux_id, u8 sum, __be32 *txid)
{
	struct sk_buff *skb;
	struct qmap_data_report *data_report;
	unsigned int len;

	len = sizeof(struct qmap_data_report);
	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	skb->protocol = htons(ETH_P_MAP);
	data_report = skb_put(skb, len);
	memset(data_report, 0, len);

	data_report->hdr.cd_bit = 1;
	data_report->hdr.mux_id = sum ? 0xFF : mux_id;
	data_report->hdr.pkt_len = htons(len - QMAP_HDR_LEN);
	data_report->hdr.cmd_name = QMAP_DATA_REPORT;
	data_report->hdr.cmd_type = QMAP_CMD_REQUEST;
	data_report->hdr.tx_id = htonl(rmnet_qmap_next_txid());

	data_report->cmd_ver = 1;
	data_report->type = type;
	data_report->sum_all_bearers = sum ? 1 : 0;

	if (txid)
		*txid = data_report->hdr.tx_id;

	return rmnet_qmap_send(skb, RMNET_CH_CTL, false);
}

void qmap_data_report_handler(struct sk_buff *skb)
{
	struct qmap_data_report *data_report;
	struct data_report_req_info *req_info = NULL;
	struct data_report_req_info *tmp_info;
	struct rmnet_aps_data_report resp;
	int i;

	if (skb->len < sizeof(struct qmap_data_report))
		return;

	data_report = (struct qmap_data_report *)skb->data;

	if (!data_report || data_report->hdr.cmd_name != QMAP_DATA_REPORT ||
	    data_report->hdr.cmd_type != QMAP_CMD_ACK)
		return;

	if (data_report->len > ARRAY_SIZE(data_report->value))
		return;

	spin_lock_bh(&data_report_lock);

	list_for_each_entry (tmp_info, &data_report_req_list, list) {
		if (tmp_info->tx_id == data_report->hdr.tx_id) {
			req_info = tmp_info;
			list_del(&req_info->list);
			data_report_count--;
			break;
		}
	}

	spin_unlock_bh(&data_report_lock);

	if (req_info) {
		resp.mux_id = data_report->hdr.mux_id;
		resp.type = data_report->type;
		resp.sum_all_bearers = data_report->sum_all_bearers;
		resp.len = data_report->len;
		for (i = 0; i < resp.len; i++)
			resp.value[i] = ntohl(data_report->value[i]);

		rmnet_aps_send_msg(&req_info->info,
				   RMNET_APS_GENL_CMD_DATA_REPORT,
				   RMNET_APS_GENL_ATTR_DATA_REPORT,
				   sizeof(resp), &resp, GFP_ATOMIC);

		kfree(req_info);
	}
}

int rmnet_aps_genl_data_report_hdlr(struct sk_buff *skb_2,
				    struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_aps_data_report req;
	__be32 txid;
	struct data_report_req_info *req_info;
	unsigned long start_timer = 0;
	int rc;

	if (!info)
		return -EINVAL;

	na = info->attrs[RMNET_APS_GENL_ATTR_DATA_REPORT];
	if (!na) {
		aps_log("aps: No data report attr\n");
		return -EINVAL;
	}

	if (nla_memcpy(&req, na, sizeof(req)) <= 0) {
		aps_log("aps: copy data report attr failed\n");
		return -EINVAL;
	}

	spin_lock_bh(&data_report_lock);

	if (!data_report_enabled) {
		aps_log("aps: ignore the data report request\n");
		goto send_err;
	}

	if (data_report_count >= 5) {
		aps_log("aps: too many outstanding requests\n");
		goto send_err;
	}

	if (qmap_send_data_report_query(req.type, req.mux_id,
					req.sum_all_bearers, &txid)) {
		aps_log("aps: failed to send data report command");
		goto send_err;
	}

	req_info = kzalloc(sizeof(*req_info), GFP_ATOMIC);
	if (!req_info) {
		aps_log("aps: no memory for data report\n");
		goto send_err;
	}

	req_info->info = *info;
	req_info->tx_id = txid;
	req_info->expires = jiffies + HZ;

	if (list_empty(&data_report_req_list))
		start_timer = req_info->expires;

	list_add(&req_info->list, &data_report_req_list);
	data_report_count++;

	spin_unlock_bh(&data_report_lock);

	if (start_timer) {
		aps_log("aps: starting data report timer\n");
		mod_timer(&data_report_timer, start_timer);
	}

	return 0;

send_err:
	/* Failed to send command. set len to 0 in the resp */
	spin_unlock_bh(&data_report_lock);
	req.len = 0;

	rc = rmnet_aps_send_msg(info, RMNET_APS_GENL_CMD_DATA_REPORT,
				RMNET_APS_GENL_ATTR_DATA_REPORT, sizeof(req),
				&req, GFP_KERNEL);

	return rc;
}

static void data_report_timer_fn(struct timer_list *t)
{
	struct data_report_req_info *req_info;
	struct data_report_req_info *tmp_info;
	unsigned long now = jiffies;
	bool rearm_timer = false;

	aps_log("aps: data report timer fired\n");

	spin_lock_bh(&data_report_lock);

	list_for_each_entry_safe (req_info, tmp_info, &data_report_req_list,
				  list) {
		if (time_before_eq(req_info->expires, now)) {
			list_del(&req_info->list);
			kfree(req_info);
			data_report_count--;
			aps_log("aps: removed one data report request\n");
		}
	}

	if (!list_empty(&data_report_req_list))
		rearm_timer = true;

	spin_unlock_bh(&data_report_lock);

	if (rearm_timer) {
		aps_log("aps: data report timer re-armed\n");
		mod_timer(&data_report_timer, jiffies + HZ);
	}
}

static void data_report_clear(void)
{
	struct data_report_req_info *req_info;
	struct data_report_req_info *tmp_info;

	spin_lock_bh(&data_report_lock);

	list_for_each_entry_safe (req_info, tmp_info, &data_report_req_list,
				  list) {
		list_del(&req_info->list);
		kfree(req_info);
	}

	data_report_enabled = false;
	data_report_count = 0;

	spin_unlock_bh(&data_report_lock);
}

static const struct rmnet_module_hook_register_info aps_data_report_hooks[] = {
	{
		.hooknum = RMNET_MODULE_HOOK_APS_DATA_REPORT,
		.func = qmap_data_report_handler,
	},
};

static int __init rmnet_aps_init(void)
{
	int rc;

	pr_info("aps: init (%s)\n", rmnet_aps_version);

	rc = rmnet_aps_genl_init();
	if (rc) {
		pr_err("aps: failed to register genl family\n");
		return rc;
	}

	register_netdevice_notifier(&rmnet_aps_notifier);

	rmnet_module_hook_register(rmnet_aps_module_hooks,
				   ARRAY_SIZE(rmnet_aps_module_hooks));

	rmnet_module_hook_register(aps_data_report_hooks,
				   ARRAY_SIZE(aps_data_report_hooks));

	mutex_lock(&rmnet_aps_mutex);
	rmnet_aps_active = true;
	mutex_unlock(&rmnet_aps_mutex);

	aps_create_proc_files();

	return 0;
}

static void __exit rmnet_aps_exit(void)
{
	aps_remove_proc_files();

	rmnet_module_hook_unregister_no_sync(
		aps_data_activity_hooks, ARRAY_SIZE(aps_data_activity_hooks));

	rmnet_module_hook_unregister_no_sync(aps_data_report_hooks,
					     ARRAY_SIZE(aps_data_report_hooks));

	rmnet_module_hook_unregister(rmnet_aps_module_hooks,
				     ARRAY_SIZE(rmnet_aps_module_hooks));

	mutex_lock(&rmnet_aps_mutex);
	rmnet_aps_active = false;
	rmnet_aps_remove_all();
	mutex_unlock(&rmnet_aps_mutex);

	rmnet_aps_flow_removed_ind(0, APS_FLOW_REMOVED_RESET);

	cancel_delayed_work_sync(&rmnet_aps_rate_work);
	del_timer_sync(&rmnet_aps_timer);
	cancel_work_sync(&rmnet_aps_flow_expire_work);

	data_report_clear();
	del_timer_sync(&data_report_timer);

	unregister_netdevice_notifier(&rmnet_aps_notifier);
	rmnet_aps_genl_deinit();

	aps_log("aps: exit\n");
}

MODULE_LICENSE("GPL v2");
module_init(rmnet_aps_init);
module_exit(rmnet_aps_exit);
