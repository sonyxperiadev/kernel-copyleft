// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "rmnet_shs.h"
#include "rmnet_shs_wq.h"
#include "rmnet_shs_modules.h"
#include <net/ip.h>
#include <linux/cpu.h>
#include <linux/bitmap.h>
#include <linux/netdevice.h>
#include <linux/kernel.h>

#include <linux/smp.h>

#include <linux/ipv6.h>
#include <linux/netdevice.h>

#define INCREMENT 1
#define DECREMENT 0
/* Helper functions to add and remove entries to the table
 * that maintains a list of all endpoints (vnd's) available on this device.
 */
void rmnet_shs_ep_tbl_add(struct rmnet_shs_wq_ep_s *ep)
{
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_EP_TBL, RMNET_SHS_WQ_EP_TBL_ADD,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, ep, NULL);
	list_add(&ep->ep_list_id, &rmnet_shs_wq_ep_tbl);
}

void rmnet_shs_ep_tbl_remove(struct rmnet_shs_wq_ep_s *ep)
{
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_EP_TBL, RMNET_SHS_WQ_EP_TBL_DEL,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, ep, NULL);
	list_del_init(&ep->ep_list_id);
}

/* Helper functions to add and remove entries to the table
 * that maintains a list of all nodes that maintain statistics per flow
 */
void rmnet_shs_hstat_tbl_add(struct rmnet_shs_wq_hstat_s *hnode)
{
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_HSTAT_TBL,
			       RMNET_SHS_WQ_HSTAT_TBL_ADD,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, hnode, NULL);
    spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
    list_add_rcu(&hnode->hstat_node_id, &rmnet_shs_wq_hstat_tbl);
    spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
}

void rmnet_shs_hstat_tbl_remove(struct rmnet_shs_wq_hstat_s *hnode)
{
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_HSTAT_TBL,
			       RMNET_SHS_WQ_HSTAT_TBL_DEL,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, hnode, NULL);

    spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
    list_del_rcu(&hnode->hstat_node_id);
    spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
}

/* We maintain a list of all flow nodes processed by a cpu.
 * Below helper functions are used to maintain flow<=>cpu
 * association.*
 */
void rmnet_shs_cpu_list_remove(struct rmnet_shs_wq_hstat_s *hnode)
{
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_HSTAT_TBL,
			    RMNET_SHS_WQ_CPU_HSTAT_TBL_DEL,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, hnode, NULL);
    spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
    list_del_init(&hnode->cpu_node_id);
    spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
}

void rmnet_shs_cpu_list_add(struct rmnet_shs_wq_hstat_s *hnode,
			    struct list_head *head)
{
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_HSTAT_TBL,
			    RMNET_SHS_WQ_CPU_HSTAT_TBL_ADD,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, hnode, NULL);

    spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
    list_add(&hnode->cpu_node_id, head);
    spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
}

void rmnet_shs_cpu_list_move(struct rmnet_shs_wq_hstat_s *hnode,
			     struct list_head *head)
{
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_HSTAT_TBL,
			    RMNET_SHS_WQ_CPU_HSTAT_TBL_MOVE,
			    hnode->current_cpu,
			    0xDEF, 0xDEF, 0xDEF, hnode, NULL);
    spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
    list_move(&hnode->cpu_node_id, head);
    spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
}

void rmnet_shs_ep_lock_bh(void)
{
	spin_lock_bh(&rmnet_shs_ep_lock);
}

void rmnet_shs_ep_unlock_bh(void)
{
	spin_unlock_bh(&rmnet_shs_ep_lock);
}

void rmnet_shs_update_cfg_mask(void)
{
	/* Start with most avaible mask all eps could share*/
	u8 mask = UPDATE_MASK;
	u8 rps_enabled = 0;
	struct rmnet_shs_wq_ep_s *ep;

	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {

		if (!ep->is_ep_active)
			continue;
		/* Bitwise and to get common mask from non-null masks.
		 * VNDs with different mask  will have UNDEFINED behavior
		 */
		if (ep->rps_config_msk) {
			mask &= ep->rps_config_msk;
			rps_enabled = 1;
		}
	}

	if (!rps_enabled) {
		rmnet_shs_cfg.map_mask = 0;
		rmnet_shs_cfg.map_len = 0;
		return;
        } else if (rmnet_shs_cfg.map_mask != mask) {
		rmnet_shs_cfg.map_mask = mask;
		rmnet_shs_cfg.map_len = rmnet_shs_get_mask_len(mask);
		pr_info("rmnet_shs:  mask: 0x%x maplen: %d\n", rmnet_shs_cfg.map_mask, rmnet_shs_cfg.map_len);
	}
}

void rmnet_shs_cpu_node_remove(struct rmnet_shs_skbn_s *node)
{
	SHS_TRACE_LOW(RMNET_SHS_CPU_NODE, RMNET_SHS_CPU_NODE_FUNC_REMOVE,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	list_del_init(&node->node_id);
	rmnet_shs_change_cpu_num_flows(node->map_cpu, DECREMENT);

}

void rmnet_shs_cpu_node_add(struct rmnet_shs_skbn_s *node,
			    struct list_head *hd)
{
	SHS_TRACE_LOW(RMNET_SHS_CPU_NODE, RMNET_SHS_CPU_NODE_FUNC_ADD,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	list_add(&node->node_id, hd);
	rmnet_shs_change_cpu_num_flows(node->map_cpu, INCREMENT);
}

void rmnet_shs_cpu_node_move(struct rmnet_shs_skbn_s *node,
			     struct list_head *hd, int oldcpu)
{
	SHS_TRACE_LOW(RMNET_SHS_CPU_NODE, RMNET_SHS_CPU_NODE_FUNC_MOVE,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	list_move(&node->node_id, hd);
	rmnet_shs_change_cpu_num_flows(node->map_cpu, INCREMENT);
	rmnet_shs_change_cpu_num_flows((u16) oldcpu, DECREMENT);
}

void rmnet_shs_cpu_ooo(u8 cpu, int count)
{
	if (cpu < MAX_CPUS)
	{
		rmnet_shs_cpu_ooo_count[cpu]+=count;
	}
}

u64 rmnet_shs_wq_get_max_allowed_pps(u16 cpu)
{

	if (cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return 0;
	}

	return rmnet_shs_cpu_rx_max_pps_thresh[cpu];
}

inline int rmnet_shs_is_lpwr_cpu(u16 cpu)
{
	return !((1 << cpu) & PERF_MASK);
}

u32 rmnet_shs_get_cpu_qhead(u8 cpu_num)
{
	u32 ret = 0;

	if (cpu_num < MAX_CPUS)
		ret = rmnet_shs_cpu_node_tbl[cpu_num].qhead;

	SHS_TRACE_LOW(RMNET_SHS_CORE_CFG, RMNET_SHS_CORE_CFG_GET_QHEAD,
			    cpu_num, ret, 0xDEF, 0xDEF, NULL, NULL);
	return ret;
}

u32 rmnet_shs_get_cpu_qtail(u8 cpu_num)
{
	u32 ret = 0;

	if (cpu_num < MAX_CPUS)
		ret =  rmnet_shs_cpu_node_tbl[cpu_num].qtail;

	SHS_TRACE_LOW(RMNET_SHS_CORE_CFG, RMNET_SHS_CORE_CFG_GET_QTAIL,
			    cpu_num, ret, 0xDEF, 0xDEF, NULL, NULL);

	return ret;
}

u32 rmnet_shs_get_cpu_qdiff(u8 cpu_num)
{
	u32 ret = 0;

	if (cpu_num < MAX_CPUS)
		ret =  rmnet_shs_cpu_node_tbl[cpu_num].qdiff;

	SHS_TRACE_LOW(RMNET_SHS_CORE_CFG, RMNET_SHS_CORE_CFG_GET_QTAIL,
			    cpu_num, ret, 0xDEF, 0xDEF, NULL, NULL);

	return ret;
}

/* Comparison function to sort ll flow loads - based on flow avg_pps
 * return -1 if a is before b, 1 if a is after b, 0 if equal
 */
int cmp_fn_ll_flow_pps(void *priv, const struct list_head *a, const struct list_head *b)
{
	struct rmnet_shs_wq_ll_flow_s *flow_a;
	struct rmnet_shs_wq_ll_flow_s *flow_b;

	if (!a || !b)
		return 0;

	flow_a = list_entry(a, struct rmnet_shs_wq_ll_flow_s, ll_flow_list);
	flow_b = list_entry(b, struct rmnet_shs_wq_ll_flow_s, ll_flow_list);

	if (flow_a->avg_pps > flow_b->avg_pps)
		return -1;
	else if (flow_a->avg_pps < flow_b->avg_pps)
		return 1;

	return 0;
}

/* Comparison function to sort filter flow loads - based on flow avg_pps
 * return -1 if a is before b, 1 if a is after b, 0 if equal
 */
int cmp_fn_filter_flow_pps(void *priv, const struct list_head *a, const struct list_head *b)
{
	struct rmnet_shs_wq_fflow_s *flow_a;
	struct rmnet_shs_wq_fflow_s *flow_b;

	if (!a || !b)
		return 0;

	flow_a = list_entry(a, struct rmnet_shs_wq_fflow_s, fflow_list);
	flow_b = list_entry(b, struct rmnet_shs_wq_fflow_s, fflow_list);

	if (flow_a->avg_pps > flow_b->avg_pps)
		return -1;
	else if (flow_a->avg_pps < flow_b->avg_pps)
		return 1;

	return 0;
}

/* Comparison function to sort gold flow loads - based on flow avg_pps
 * return -1 if a is before b, 1 if a is after b, 0 if equal
 */
int cmp_fn_flow_pps(void *priv, const struct list_head *a, const struct list_head *b)
{
	struct rmnet_shs_wq_gold_flow_s *flow_a;
	struct rmnet_shs_wq_gold_flow_s *flow_b;

	if (!a || !b)
		return 0;

	flow_a = list_entry(a, struct rmnet_shs_wq_gold_flow_s, gflow_list);
	flow_b = list_entry(b, struct rmnet_shs_wq_gold_flow_s, gflow_list);

	if (flow_a->avg_pps > flow_b->avg_pps)
		return -1;
	else if (flow_a->avg_pps < flow_b->avg_pps)
		return 1;

	return 0;
}

/* Comparison function to sort cpu capacities - based on cpu avg_pps capacity
 * return -1 if a is before b, 1 if a is after b, 0 if equal
 */
int cmp_fn_cpu_pps(void *priv, const struct list_head *a, const struct list_head *b)
{
	struct rmnet_shs_wq_cpu_cap_s *cpu_a;
	struct rmnet_shs_wq_cpu_cap_s *cpu_b;

	if (!a || !b)
		return 0;

	cpu_a = list_entry(a, struct rmnet_shs_wq_cpu_cap_s, cpu_cap_list);
	cpu_b = list_entry(b, struct rmnet_shs_wq_cpu_cap_s, cpu_cap_list);

	if (cpu_a->avg_pps_capacity > cpu_b->avg_pps_capacity)
		return -1;
	else if (cpu_a->avg_pps_capacity < cpu_b->avg_pps_capacity)
		return 1;

	return 0;
}

/* Return Invalid core if only pri core available*/
int rmnet_shs_wq_get_lpwr_cpu_new_flow(struct net_device *dev)
{
	u8 lo_idx;
	u8 lo_max;
	int cpu_assigned = -1;
	u8 is_match_found = 0;
	struct rmnet_shs_wq_ep_s *ep = NULL;

	if (!dev) {
		rmnet_shs_crit_err[RMNET_SHS_NETDEV_ERR]++;
		return cpu_assigned;
	}

	spin_lock_bh(&rmnet_shs_ep_lock);
	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {
		if (!ep->is_ep_active)
			continue;

		if (ep->ep == dev) {
			is_match_found = 1;
			break;
		}

	}

	if (!is_match_found) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		spin_unlock_bh(&rmnet_shs_ep_lock);
		return cpu_assigned;
	}

	lo_idx = ep->new_lo_idx;
	lo_max = ep->new_lo_max;

	while (lo_idx < lo_max) {
		if (ep->new_lo_core[lo_idx] >= 0) {
			cpu_assigned = ep->new_lo_core[lo_idx];
			break;
		}
		lo_idx++;
	}

	/* Increment CPU assignment idx to be ready for next flow assignment*/
	if ((cpu_assigned >= 0) || ((ep->new_lo_idx + 1) >= ep->new_lo_max))
		ep->new_lo_idx = ((ep->new_lo_idx + 1) % ep->new_lo_max);
	spin_unlock_bh(&rmnet_shs_ep_lock);

	return cpu_assigned;
}

int rmnet_shs_wq_get_perf_cpu_new_flow(struct net_device *dev)
{
	struct rmnet_shs_wq_ep_s *ep = NULL;
	int cpu_assigned = -1;
	u8 hi_idx;
	u8 hi_max;
	u8 is_match_found = 0;

	if (!dev) {
		rmnet_shs_crit_err[RMNET_SHS_NETDEV_ERR]++;
		return cpu_assigned;
	}

	spin_lock_bh(&rmnet_shs_ep_lock);
	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {

		if (!ep->is_ep_active)
			continue;

		if (ep->ep == dev) {
			is_match_found = 1;
			break;
		}
	}

	if (!is_match_found) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		spin_unlock_bh(&rmnet_shs_ep_lock);
		return cpu_assigned;
	}

	hi_idx = ep->new_hi_idx;
	hi_max = ep->new_hi_max;

	while (hi_idx < hi_max) {
		if (ep->new_hi_core[hi_idx] >= 0) {
			cpu_assigned = ep->new_hi_core[hi_idx];
			break;
		}
		hi_idx++;
	}
	/* Increment CPU assignment idx to be ready for next flow assignment*/
	if (cpu_assigned >= 0)
		ep->new_hi_idx = ((hi_idx + 1) % hi_max);
	spin_unlock_bh(&rmnet_shs_ep_lock);

	return cpu_assigned;
}

void rmnet_shs_ps_on_hdlr(void *port)
{
	rmnet_shs_wq_pause();
}

void rmnet_shs_ps_off_hdlr(void *port)
{
	rmnet_shs_wq_restart();
}

u8 rmnet_shs_mask_from_map(struct rps_map *map)
{
	u8 mask = 0;
	u8 i;

	for (i = 0; i < map->len; i++)
		mask |= 1 << map->cpus[i];

	return mask;
}

int rmnet_shs_get_mask_len(u8 mask)
{
	u8 i;
	u8 sum = 0;

	for (i = 0; i < MAX_CPUS; i++) {
		if (mask & (1 << i))
			sum++;
	}
	return sum;
}

/* Takes a CPU and a CPU mask and computes what index of configured
 * the CPU is in. Returns INVALID_CPU if CPU is not enabled in the mask.
 */
int rmnet_shs_idx_from_cpu(u8 cpu, u8 mask)
{
	int ret = INVALID_CPU;
	u8 idx = 0;
	u8 i;

	/* If not in mask return invalid*/
	if (!(mask & 1 << cpu))
		return ret;

	/* Find idx by counting all other configed CPUs*/
	for (i = 0; i < MAX_CPUS; i++) {
		if (i == cpu  && (mask & (1 << i))) {
			ret = idx;
			break;
		}
		if (mask & (1 << i))
			idx++;
	}
	return ret;
}

/* Assigns a CPU to process packets corresponding to new flow. For flow with
 * small incoming burst a low power core handling least number of packets
 * per second will be assigned.
 *
 * For a flow with a heavy incoming burst, a performance core with the least
 * number of packets processed per second  will be assigned
 *
 * If two or more cores within a cluster are handling the same number of
 * packets per second, the first match will be assigned.
 */
int rmnet_shs_new_flow_cpu(u64 burst_size, struct net_device *dev)
{
	int flow_cpu = INVALID_CPU;

	if (burst_size < RMNET_SHS_MAX_SILVER_CORE_BURST_CAPACITY)
		flow_cpu = rmnet_shs_wq_get_lpwr_cpu_new_flow(dev);
	if (flow_cpu == INVALID_CPU ||
	    burst_size >= RMNET_SHS_MAX_SILVER_CORE_BURST_CAPACITY)
		flow_cpu = rmnet_shs_wq_get_perf_cpu_new_flow(dev);

	SHS_TRACE_HIGH(RMNET_SHS_ASSIGN,
			     RMNET_SHS_ASSIGN_GET_NEW_FLOW_CPU,
			     flow_cpu, burst_size, 0xDEF, 0xDEF, NULL, NULL);

	return flow_cpu;
}

void *rmnet_shs_header_ptr(struct sk_buff *skb, u32 offset, u32 hlen,
				  void *buf)
{
	struct skb_shared_info *shinfo = skb_shinfo(skb);
	skb_frag_t *frag;
	u32 offset_orig = offset;
	int i;

	if (offset > skb->len || hlen > skb->len || offset + hlen > skb->len)
		return NULL;

	/* Linear packets or packets with headers in linear portion */
	if (skb_headlen(skb) >= offset + hlen)
		return skb->data + offset;

	offset -= skb_headlen(skb);
	/* Return pointer to page if contiguous */
	for (i = 0; i < shinfo->nr_frags; i++) {
		u32 frag_size;

		frag = &shinfo->frags[i];
		frag_size = skb_frag_size(frag);
		if (offset >= frag_size) {
			/* Next frag */
			offset -= frag_size;
			continue;
		}

		if (frag_size >= offset + hlen)
			return skb_frag_address(frag) + offset;
	}

	/* The data is split across pages. Use the linear buffer */
	if (skb_copy_bits(skb, (int)offset_orig, buf, (int)hlen))
		return NULL;

	return buf;
}

void rmnet_shs_get_update_skb_hdr_info(struct sk_buff *skb,
				       struct rmnet_shs_skbn_s *node_p)
{
	struct iphdr *ip4h, __ip4h;
	struct ipv6hdr *ip6h, __ip6h;

	struct tcphdr *tp, __tp;
	struct udphdr *up, __up;

	int len = 0;
	u16 ip_len = 0;

	__be16 frag_off;
	u8 protocol;

	switch (skb->protocol) {
	case htons(ETH_P_IP):
		ip4h = rmnet_shs_header_ptr(skb, 0, sizeof(*ip4h), &__ip4h);
		if (!ip4h)
			return;

		node_p->skb_tport_proto = ip4h->protocol;
		memcpy(&(node_p->ip_hdr.v4hdr), ip4h, sizeof(*ip4h));

		ip_len = ip4h->ihl * 4;

		break;
	case htons(ETH_P_IPV6):
		ip6h = rmnet_shs_header_ptr(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			return;

		node_p->skb_tport_proto = ip6h->nexthdr;
		memcpy(&(node_p->ip_hdr.v6hdr), ip6h, sizeof(*ip6h));

		protocol = ip6h->nexthdr;

		len = ipv6_skip_exthdr(skb, sizeof(*ip6h), &protocol,
				       &frag_off);
		if (len < 0) {
			/* Cant find transport header */
			return;
		}

		ip_len = (u16)len;

		break;
	default:
		break;
	}

	if (node_p->skb_tport_proto == IPPROTO_TCP) {
		tp = rmnet_shs_header_ptr(skb, ip_len, sizeof(*tp), &__tp);
		if (!tp)
			return;

		memcpy(&(node_p->trans_hdr.tp),
		       tp,
		       sizeof(struct tcphdr));
	} else if (node_p->skb_tport_proto == IPPROTO_UDP) {
		up = rmnet_shs_header_ptr(skb, ip_len, sizeof(*up), &__up);
		if (!up)
			return;

		memcpy(&(node_p->trans_hdr.up),
		       up,
		       sizeof(struct udphdr));
	} else {
		/* Non TCP or UDP proto, dont copy transport header */
	}

}

/* Forms a new hash from the incoming hash based on the number of cores
 * available for processing. This new hash will be stamped by
 * SHS module (for all the packets arriving with same incoming hash)
 * before delivering them to next layer.
 */
u32 rmnet_shs_form_hash(u32 index, u32 maplen, u32 hash, u8 async)
{
	int offsetmap[MAX_CPUS / 2] = {8, 4, 3, 2};
	u32 ret = 0;

	if (!maplen) {
		rmnet_shs_crit_err[RMNET_SHS_MAIN_MAP_LEN_INVALID]++;
		return ret;
	}

	/* Override MSB of skb hash to steer. Save most of Hash bits
	 * Leave some as 0 to allow for easy debugging.
	 */
	if (maplen < MAX_CPUS)
		ret = ((((index + ((maplen % 2) ? 1 : 0))) << 28)
			* offsetmap[(maplen - 1) >> 1]) | (hash & 0xFFFFFF);
	/*Wipe last 4 bytes and set to magic number if async set*/
	if (async)
		ret = (ret & ~0xFFFFF) | VH_MAGIC_HASH;

	SHS_TRACE_LOW(RMNET_SHS_HASH_MAP, RMNET_SHS_HASH_MAP_FORM_HASH,
			    ret, hash, index, maplen, NULL, NULL);

	return ret;
}
