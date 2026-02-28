// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/ip.h>
#include <linux/cpu.h>
#include <net/ip.h>
#include <linux/cpu.h>
#include <linux/bitmap.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/ipv6.h>
#include <linux/netdevice.h>
#include <linux/percpu-defs.h>

#include "rmnet_shs.h"
#include "rmnet_shs_wq_genl.h"
#include "rmnet_shs_config.h"
#include "rmnet_shs_wq.h"
#include "rmnet_shs_modules.h"
#include "rmnet_shs_common.h"
#include "rmnet_trace.h"
#include <linux/icmp.h>
#include <linux/inet.h>


DEFINE_HASHTABLE(rmnet_shs_ll_ht, RMNET_SHS_HT_SIZE);
DEFINE_HASHTABLE(rmnet_shs_ll_filter_ht, RMNET_SHS_HT_SIZE);
DEFINE_SPINLOCK(rmnet_shs_ll_ht_splock);
struct rmnet_shs_cpu_node_s rmnet_shs_ll_cpu_node_tbl[MAX_CPUS];
#define MAX_LL_FILTERS 100
#define GET_IQUEUE(CPU) (per_cpu(softnet_data, CPU).input_pkt_queue)
#define GET_PQUEUE(CPU) (per_cpu(softnet_data, CPU).process_queue)
#define GET_QLEN(CPU) (GET_IQUEUE(CPU).qlen + GET_PQUEUE(CPU).qlen)
#define GET_QTAIL(SD, CPU) (per_cpu(SD, CPU).input_queue_tail)
#define GET_QHEAD(SD, CPU) (per_cpu(SD, CPU).input_queue_head)
#define GET_QHEADS(CPU) (per_cpu(softnet_data, CPU).input_queue_head)
#define GET_QTAILS(CPU) (per_cpu(softnet_data, CPU).input_queue_tail)
#define MAX_LL_CORE_BACKLOG 20

unsigned int rmnet_shs_ll_pkts = 0;
module_param(rmnet_shs_ll_pkts, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_ll_pkts, "LL packets seen in ll rmnet_shs");

unsigned int rmnet_shs_filter_count = 0;
module_param(rmnet_shs_filter_count, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_filter_count, "LL filter count seen in ll rmnet_shs");

/* Evaluates the incoming transport protocol of the incoming skb. Determines
 * if the skb transport protocol will be supported by SHS module
 */
int rmnet_shs_is_ll_skb_stamping_reqd(struct sk_buff *skb)
{
	int ret_val = 0;
	struct iphdr *ip4h, __ip4h;
	struct ipv6hdr *ip6h, __ip6h;
	const struct ipv6_opt_hdr *ptr;
	struct ipv6_opt_hdr v6hdr;

	/* SHS will ignore ICMP and frag pkts completely */
	switch (skb->protocol) {
	case htons(ETH_P_IP):
		ip4h = rmnet_shs_header_ptr(skb, 0, sizeof(*ip4h), &__ip4h);
		if (!ip4h)
			break;

		if (!ip_is_fragment(ip4h) &&
		    (ip4h->protocol == IPPROTO_TCP ||
		     ip4h->protocol == IPPROTO_UDP)) {
			ret_val = 1;
			break;
		}

		/* RPS logic is skipped if RPS hash is 0 while sw_hash
		 * is set as active and packet is processed on the same
		 * CPU as the initial caller.
		 *
		 * No longer put ICMP on phy core when moving to perf core
		 */
		if (ip4h->protocol == IPPROTO_ICMP) {
			skb->hash = 0;
			skb->sw_hash = 1;

			if (trace_print_icmp_rx_enabled()) {
				char saddr[INET6_ADDRSTRLEN], daddr[INET6_ADDRSTRLEN];
				u16 ip_proto = 0;
				__be16 sequence = 0;
				u8 type = 0;
				struct icmphdr *icmphdr, __icmphdr;

				memset(saddr, 0, INET6_ADDRSTRLEN);
				memset(daddr, 0, INET6_ADDRSTRLEN);

				icmphdr = rmnet_shs_header_ptr(skb, ip4h->ihl * 4,
							       sizeof(*icmphdr), &__icmphdr);
				if (!icmphdr)
					goto skip_trace_print_icmp4_rx;

				if (icmphdr->type != ICMP_ECHOREPLY &&
				    icmphdr->type != ICMP_ECHO)
					goto skip_trace_print_icmp4_rx;

				ip_proto = htons(ETH_P_IP);
				type = icmphdr->type;
				sequence = icmphdr->un.echo.sequence;
				snprintf(saddr, INET6_ADDRSTRLEN, "%pI4", &ip4h->saddr);
				snprintf(daddr, INET6_ADDRSTRLEN, "%pI4", &ip4h->daddr);

				trace_print_icmp_rx(skb, ip_proto, type, sequence, saddr, daddr);
			}
		} else if (ip4h->protocol == IPPROTO_ESP) {
			/* Pin to core 0 if ESP protocol */
			skb->hash = DEFAULT_PIN_HASH;
			skb->sw_hash = 1;
		}
skip_trace_print_icmp4_rx:
		break;

	case htons(ETH_P_IPV6):
		ip6h = rmnet_shs_header_ptr(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			break;

		if (!(ip6h->nexthdr == NEXTHDR_FRAGMENT) &&
		    (ip6h->nexthdr == IPPROTO_TCP ||
		     ip6h->nexthdr == IPPROTO_UDP)) {
				ret_val =  1;
				break;
		}

		/* RPS logic is skipped if RPS hash is 0 while sw_hash
		 * is set as active and packet is processed on the same
		 * CPU as the initial caller.
		 *
		 * No longer put ICMP on phy core when moving to perf core
		 */
		if (ip6h->nexthdr == NEXTHDR_ICMP) {
			skb->hash = 0;
			skb->sw_hash = 1;

			if (trace_print_icmp_rx_enabled()) {
				char saddr[INET6_ADDRSTRLEN], daddr[INET6_ADDRSTRLEN];
				u16 ip_proto = 0;
				__be16 sequence = 0;
				u8 type = 0;
				struct icmp6hdr *icmp6hdr, __icmp6hdr;

				memset(saddr, 0, INET6_ADDRSTRLEN);
				memset(daddr, 0, INET6_ADDRSTRLEN);

				icmp6hdr = rmnet_shs_header_ptr(skb, sizeof(*ip6h),
								sizeof(*icmp6hdr), &__icmp6hdr);
				if (!icmp6hdr)
					goto skip_trace_print_icmp6_rx;

				if (icmp6hdr->icmp6_type != ICMPV6_ECHO_REQUEST &&
				    icmp6hdr->icmp6_type != ICMPV6_ECHO_REPLY)
					goto skip_trace_print_icmp6_rx;

				ip_proto = htons(ETH_P_IPV6);
				type = icmp6hdr->icmp6_type;
				sequence = icmp6hdr->icmp6_sequence;
				snprintf(saddr, INET6_ADDRSTRLEN, "%pI6", &ip6h->saddr);
				snprintf(daddr, INET6_ADDRSTRLEN, "%pI6", &ip6h->daddr);

				trace_print_icmp_rx(skb, ip_proto, type, sequence, saddr, daddr);
			}
		} else  if (ip6h->nexthdr == NEXTHDR_ESP) {
			skb->hash = DEFAULT_PIN_HASH;
			skb->sw_hash = 1;
		} else if (ip6h->nexthdr == NEXTHDR_FRAGMENT) {
			if (skb->len - sizeof(struct ipv6hdr) < (int)sizeof(struct ipv6_opt_hdr)) {
			/* skb too small to contain another header */
				break;
			}
			/* Check if frag header has ESP nextheader. If packet has more headers before
			 * ESP header this would fail. Ex Frag, Encap, ESP Chain */
			ptr = skb_header_pointer(skb, sizeof(struct ipv6hdr), sizeof(v6hdr), &v6hdr);
			if (ptr && ptr->nexthdr == NEXTHDR_ESP) {
				skb->hash = DEFAULT_PIN_HASH;
				skb->sw_hash = 1;
			}
		}

skip_trace_print_icmp6_rx:
		break;

	default:
		break;
	}

	SHS_TRACE_LOW(RMNET_SHS_SKB_STAMPING, RMNET_SHS_SKB_STAMPING_END,
			    ret_val, 0xDEF, 0xDEF, 0xDEF, skb, NULL);

	return ret_val;
}

/* Evaluates if a v6 skb matches the v6 filter passed in *info */
int ipv6_packet_match(struct sk_buff *skb, struct ipv6hdr *skb_ip6h, struct rmnet_shs_wq_flow_info  *info)
{
	struct tcphdr *tp, __tp;
	struct udphdr *up, __up;
	int saddmatch, daddmatch, protomatch, src_port_match, dest_port_match = false;
	int ret = false;
	int v6len = 0;
	u8 protocol;
	__be16 frag_off;

	saddmatch = !info->src_addr_valid || ipv6_addr_equal(&skb_ip6h->saddr,  &info->src_ip_addr.v6_saddr);
	daddmatch = !info->dest_addr_valid || ipv6_addr_equal(&skb_ip6h->daddr,  &info->dest_ip_addr.v6_daddr);
	protomatch = !info->proto_valid || info->proto == skb_ip6h->nexthdr;

	src_port_match = !info->src_port_valid ;
	dest_port_match = !info->dest_port_valid;
	protocol = skb_ip6h->nexthdr;

	if (info->src_port_valid || info->dest_port_valid) {
		v6len = ipv6_skip_exthdr(skb, sizeof(*skb_ip6h), &protocol, &frag_off);
		if (v6len < 0) {
			/* Cant find transport header */
			return false;
		}
		if (skb_ip6h->nexthdr == IPPROTO_TCP) {
			tp = rmnet_shs_header_ptr(skb, v6len, sizeof(*tp), &__tp);
			if (!tp) {
				src_port_match = false;
				dest_port_match = false;
			} else {
				src_port_match = !info->src_port_valid || tp->source ==  (info->src_port);
				dest_port_match = !info->dest_port_valid || tp->dest ==  (info->dest_port);
			}
		} else if (skb_ip6h->nexthdr == IPPROTO_UDP) {
			up = rmnet_shs_header_ptr(skb, v6len, sizeof(*up), &__up);
			if (!up) {
				src_port_match = false;
				dest_port_match = false;
			} else {
				src_port_match = !info->src_port_valid || up->source ==  (info->src_port);
				dest_port_match = !info->dest_port_valid ||  up->dest ==  (info->dest_port);
			}
		}
	}

	if ((saddmatch) && (daddmatch) && (protomatch) && (src_port_match) && (dest_port_match))
	   ret =  true;

	return ret;
}

/* Evaluates if a v4 skb matches the v4 filter passed in *info */
int ipv4_packet_match(struct sk_buff *skb, struct iphdr *skb_ip4h, struct rmnet_shs_wq_flow_info  *info)
{
	int ret = false;
	struct tcphdr *tp, __tp;
	struct udphdr *up, __up;
	u16 v4ip_len = skb_ip4h->ihl * 4;

	int saddmatch = !info->src_addr_valid || skb_ip4h->saddr == info->src_ip_addr.saddr;
	int daddmatch = !info->dest_addr_valid || skb_ip4h->daddr == (info->dest_ip_addr.daddr);
	int protomatch = !info->proto_valid || skb_ip4h->protocol == info->proto;
	int src_port_match = !info->src_port_valid ;
	int dest_port_match = !info->dest_port_valid;

	if (info->src_port_valid || info->dest_port_valid) {
		if (skb_ip4h->protocol == IPPROTO_TCP) {
			tp = rmnet_shs_header_ptr(skb, v4ip_len, sizeof(*tp), &__tp);
			if (!tp) {
				src_port_match = false;
				dest_port_match = false;
			} else {
				src_port_match = !info->src_port_valid || tp->source ==  (info->src_port);
				dest_port_match = !info->dest_port_valid || tp->dest ==  (info->dest_port);
			}
		} else if (skb_ip4h->protocol == IPPROTO_UDP) {
			up = rmnet_shs_header_ptr(skb, v4ip_len, sizeof(*up), &__up);
			if (!up) {
				src_port_match = false;
				dest_port_match = false;
			} else {
				src_port_match = !info->src_port_valid || up->source ==  (info->src_port);
				dest_port_match = !info->dest_port_valid ||  up->dest ==  (info->dest_port);
			}
		}
	}
	if ((saddmatch) && (daddmatch) && (protomatch) && (src_port_match) && (dest_port_match))
	   ret =  true;

	rm_err("SHS_LL: V4 saddr match %u daddr match %u, proto match %u, src port %u, dest port match %u  proto %u\n",
	       saddmatch, daddmatch, protomatch, src_port_match, dest_port_match, skb_ip4h->protocol);
	return ret;
}

/* Evaluates if two filters match identical things, to prevent dup filter additions.*/
int rmnet_shs_is_identical_filter(struct rmnet_shs_wq_flow_node  *node, struct rmnet_shs_wq_flow_node  *node2)
{
	struct rmnet_shs_wq_flow_info  *info = &node->info;
	struct rmnet_shs_wq_flow_info  *info2 = &node2->info;

	int versionmatch = info->ip_version == info2->ip_version;
	int saddmatch, daddmatch, protomatch, src_port_match, dest_port_match = false;

	/* Sequence match superscedes contents */
	if (info->seq && info2->seq && (info->seq == info2->seq)) {
		return true;
	}
	/* Saddr matches between filters if both are not matchign for addr and/or address match*/
	saddmatch = (!info->src_addr_valid  && !info2->src_addr_valid) ||
		 (info->src_addr_valid  && info2->src_addr_valid &&
		 (info->ip_version == 4)?(info->src_ip_addr.saddr == info2->src_ip_addr.saddr) :
		 ipv6_addr_equal(&info2->src_ip_addr.v6_saddr, &info->src_ip_addr.v6_saddr));

	daddmatch = (!info->dest_addr_valid  && !info2->dest_addr_valid) ||
		(info->dest_addr_valid  && info2->dest_addr_valid &&
		(info->ip_version == 4)?(info->dest_ip_addr.daddr == info2->dest_ip_addr.daddr) :
		ipv6_addr_equal(&info2->dest_ip_addr.v6_daddr, &info->dest_ip_addr.v6_daddr));

	protomatch = (!info->proto_valid  && !info2->proto_valid) ||
					(info->proto_valid  && info2->proto_valid && info->proto == info2->proto);

	src_port_match = (!info->src_port_valid  && !info2->src_port_valid) ||
					(info->src_port_valid  && info2->src_port_valid &&
					info->src_port == info2->src_port);

	dest_port_match = (!info->dest_port_valid  && !info2->dest_port_valid) ||
					(info->dest_port_valid  && info2->dest_port_valid &&
					info->dest_port == info2->dest_port);

	rm_err("SHS_LL: match result sadr match %u daddr match %u, proto match %u, src port %u, dest port match %u  versionmatch %u\n",
           saddmatch, daddmatch, protomatch,src_port_match, dest_port_match, versionmatch);
	return (versionmatch && saddmatch && daddmatch  && protomatch && src_port_match && dest_port_match);

}

/* Evaluates the incoming skb against all installed filters
 * if the filter matches with the SKB then true is returned.
 */
int rmnet_shs_is_filter_match(struct sk_buff *skb)
{
	struct iphdr *ip4h, __ip4h;
	struct ipv6hdr *ip6h, __ip6h;
	struct rmnet_shs_wq_flow_node *node_p;
	struct hlist_node *tmp;
	int ret = false;

	spin_lock_bh(&rmnet_shs_ll_ht_splock);
	/* SHS will ignore ICMP and frag pkts completely */
	switch (skb->protocol) {
	case htons(ETH_P_IP):
		ip4h = rmnet_shs_header_ptr(skb, 0, sizeof(*ip4h), &__ip4h);
		if (!ip4h) {
			break;
		}
		hash_for_each_possible_safe(rmnet_shs_ll_filter_ht, node_p, tmp, list, ip4h->saddr) {
			if (ipv4_packet_match(skb, ip4h, &node_p->info)) {
				ret = true;
				break;
			}
		}
		break;
	case htons(ETH_P_IPV6):
		ip6h = rmnet_shs_header_ptr(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h) {
			break;
		}

		hash_for_each_possible_safe(rmnet_shs_ll_filter_ht, node_p, tmp, list, ip6h->saddr.in6_u.u6_addr32[0]) {
			if (ipv6_packet_match(skb, ip6h, &node_p->info)) {
				ret = true;
				break;
			}

		}
		break;
	default:
		break;
	}
	spin_unlock_bh(&rmnet_shs_ll_ht_splock);
	rm_err("SHS_LL: Packet Filter checked analyzed ret: %d", ret);
	return ret;
}

/* Uninstalls a LL flow filter contained by node */
void rmnet_shs_remove_llflow(struct rmnet_shs_wq_flow_node  *node)
{
	struct rmnet_shs_wq_flow_node  *temp_node;
	struct hlist_node *tmp;
	struct rmnet_shs_wq_hstat_s *hnode = NULL;
	unsigned long bkt;
	int i = 0;

	spin_lock_bh(&rmnet_shs_ll_ht_splock);
	hash_for_each_safe(rmnet_shs_ll_filter_ht, bkt, tmp, temp_node, list)
	{
		i++;
		if (rmnet_shs_is_identical_filter(temp_node, node)) {
			rm_err("SHS_LL: %s\n", "Filter already installed, Dup Filter");
			hash_del_rcu(&temp_node->list);
			kfree(temp_node);
			break;
		}
	}
	spin_unlock_bh(&rmnet_shs_ll_ht_splock);

	spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
	list_for_each_entry(hnode, &rmnet_shs_wq_hstat_tbl, hstat_node_id) {
		if (hnode->node && !hnode->node->low_latency) {
			hnode->node->low_latency= RMNET_SHS_LOW_LATENCY_CHECK;
		}
	}
	spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
	/* Free the genl node allocated at genl recv */
	kfree(node);
	rmnet_shs_cfg.num_filters--;
	rmnet_shs_filter_count--;

	rm_err("SHS_LL: %s", " Uninstalled LL filter");
}

void rmnet_shs_print_llflow(struct rmnet_shs_wq_flow_node  *node)
{
	struct rmnet_shs_wq_flow_info  *info = &node->info;

	pr_info("SHS_LL: proto valid %u src addr valid %u, dest addr valid %u, dest port valid %u, srcport valid %u, ip version %u seq %u\n",
		info->proto_valid, info->src_addr_valid, info->dest_addr_valid,
		info->dest_port_valid, info->src_port_valid, info->ip_version, info->seq);

	pr_info("SHS_LL: info->ip_version %u\n", info->ip_version );
	pr_info("SHS_LL: info->proto %u\n", info->proto );
	pr_info("SHS_LL: info->dest_port %u\n", info->dest_port );
	pr_info("SHS_LL: info->src_port %u\n", info->src_port );
	pr_info("SHS_LL: info->dest_addr_valid %u\n", info->dest_addr_valid);
	pr_info("SHS_LL: info->src_addr_valid %u\n", info->src_addr_valid);
	pr_info("SHS_LL: info->seq %u\n", info->seq);

	if (info->ip_version == 4 && (info->dest_addr_valid) && (info->src_addr_valid )) {
		pr_info("New flow info->dest_addr_valid %u\n", info->dest_ip_addr.daddr);
		pr_info("New flow info->src_addr_valid %u\n",  info->src_ip_addr.saddr);
	}
	if (info->ip_version == 6 && (info->dest_addr_valid) && (info->src_addr_valid )) {
		pr_info("New flow info->dest_addr_valid %u %u %u %u\n",
                node->info.dest_ip_addr.v6_daddr.in6_u.u6_addr32[3],
                node->info.dest_ip_addr.v6_daddr.in6_u.u6_addr32[2],
                node->info.dest_ip_addr.v6_daddr.in6_u.u6_addr32[1],
                node->info.dest_ip_addr.v6_daddr.in6_u.u6_addr32[0]);
		pr_info("New flow info->src_addr_valid  %u %u %u %u\n",
                node->info.src_ip_addr.v6_saddr.in6_u.u6_addr32[3],
                node->info.src_ip_addr.v6_saddr.in6_u.u6_addr32[2],
                node->info.src_ip_addr.v6_saddr.in6_u.u6_addr32[1],
                node->info.src_ip_addr.v6_saddr.in6_u.u6_addr32[0]);
	}
}

/* Installs a LL flow filter contained by node */
void rmnet_shs_add_llflow(struct rmnet_shs_wq_flow_node  *node)
{
	struct rmnet_shs_wq_flow_node  *temp_node;
	struct rmnet_shs_wq_hstat_s *hnode = NULL;
	unsigned long bkt;
	int i = 0;

	spin_lock_bh(&rmnet_shs_ll_ht_splock);
	hash_for_each(rmnet_shs_ll_filter_ht, bkt, temp_node, list)
	{
		i++;
		if (rmnet_shs_is_identical_filter(temp_node, node)) {
			kfree(node);
			spin_unlock_bh(&rmnet_shs_ll_ht_splock);
			rm_err("SHS_LL: %s", " Dup filter seen match seen, no install");
			return;
		}
	}

	if (rmnet_shs_cfg.num_filters >= MAX_LL_FILTERS) {
		kfree(node);
		spin_unlock_bh(&rmnet_shs_ll_ht_splock);
        rmnet_shs_crit_err[RMNET_SHS_MAX_LL_FILTERS]++;
        rm_err("SHS_LL: %s\n", "Installed LL filter failed: Max reached");
		return;
	}
	rmnet_shs_cfg.num_filters++;
	rmnet_shs_filter_count++;

	if (rmnet_shs_debug)
		rmnet_shs_print_llflow(node);
	/* Room for improvement if hash is not just the src address 1st int for v6 */
	hash_add(rmnet_shs_ll_filter_ht, &node->list, node->info.src_ip_addr.saddr);
	spin_unlock_bh(&rmnet_shs_ll_ht_splock);

	/* Mark all flows to check if filter is a match on next packet */
	rm_err("SHS_LL: %s\n", "Setting low latency flow check for all flows");
	spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
	list_for_each_entry(hnode, &rmnet_shs_wq_hstat_tbl, hstat_node_id) {
		if (hnode->node && !hnode->node->low_latency) {
			hnode->node->low_latency= RMNET_SHS_LOW_LATENCY_CHECK;
		}

	}
	spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
	rm_err("SHS_LL: %s\n", "Installed LL filter");
}

void rmnet_shs_ll_stamp(struct sk_buff *skb, struct rmnet_shs_skbn_s *node)
{
	u32 hash2stamp = 0; /* the default value of skb->hash*/
	u8 map = 0, maplen = 0;
	u16 index;

	if (!node->custom_map) {
		map = rmnet_shs_cfg.map_mask;
		maplen = rmnet_shs_cfg.map_len;
		index = node->map_index;
	} else {
		map = node->custom_map;
		maplen = node->custom_len;
		index = node->map_index;
	}
	if (map) {
		hash2stamp = rmnet_shs_form_hash(index,
						 maplen,
						 node->hash, 0);
	    skb->hash = hash2stamp;
	}
}

int rmnet_shs_ll_handler(struct sk_buff *skb, struct rmnet_shs_clnt_s *clnt_cfg)
{
	struct rmnet_shs_skbn_s *node_p;
	struct hlist_node *tmp;
	int map = rmnet_shs_cfg.map_mask;
	int ll_cpu = rmnet_shs_ll_flow_cpu;
	int map_cpu;
	u32 hash;
	u8 is_match_found = 0;
	struct rmnet_shs_cpu_node_s *cpu_node_tbl_p;
	struct rmnet_priv *priv;

	rmnet_shs_ll_pkts++;

	hash = skb_get_hash(skb);
	/*deliver non TCP/UDP packets right away*/
	/* If stmp all is set break and don't check reqd */
	if (!(clnt_cfg->config & RMNET_SHS_STMP_ALL) &&
	    !rmnet_shs_is_ll_skb_stamping_reqd(skb)) {
		rmnet_shs_deliver_skb(skb);
		return 0;
	}

	spin_lock_bh(&rmnet_shs_ll_ht_splock);
	do {

		hash_for_each_possible_safe(rmnet_shs_ll_ht, node_p, tmp, list,
					    hash) {

			if (hash != node_p->hash)
				continue;
			is_match_found = 1;

			node_p->map_cpu = rmnet_shs_ll_flow_cpu;

			node_p->map_index = rmnet_shs_idx_from_cpu(node_p->map_cpu, map);
			break;
		}
		if (is_match_found)
			break;

		if (ll_cpu < 0) {
			rmnet_shs_crit_err[RMNET_SHS_RPS_MASK_CHANGE]++;
			break;
		}

		if (atomic_long_read(&rmnet_shs_cfg.num_flows) > MAX_FLOWS) {
			rmnet_shs_crit_err[RMNET_SHS_MAX_FLOWS]++;
			break;
		}

		node_p = kzalloc(sizeof(*node_p), GFP_ATOMIC);

		if (!node_p) {
			rmnet_shs_crit_err[RMNET_SHS_MAIN_MALLOC_ERR]++;
			break;
		}

		atomic_long_inc(&rmnet_shs_cfg.num_flows);

		node_p->custom_map = clnt_cfg->map_mask;
		node_p->custom_len = rmnet_shs_cfg.map_mask;
		node_p->dev = skb->dev;
		node_p->hash = skb->hash;
		node_p->map_cpu = ll_cpu;
		node_p->low_latency = 1;
		node_p->map_index = rmnet_shs_idx_from_cpu(node_p->map_cpu, map);
		node_p->map_cpu = raw_smp_processor_id();
		node_p->map_index = rmnet_shs_idx_from_cpu(node_p->map_cpu, map);

		INIT_LIST_HEAD(&node_p->node_id);
		/* Set ip header / transport header / transport proto */
		rmnet_shs_get_update_skb_hdr_info(skb, node_p);
		/* Workqueue utilizes some of the values from above
		 * initializations . Therefore, we need to request
		 * for memory (to workqueue) after the above initializations
		 */
		rmnet_shs_wq_create_new_flow(node_p);
		map_cpu = node_p->map_cpu;
		cpu_node_tbl_p = &rmnet_shs_ll_cpu_node_tbl[map_cpu];
		/* Set mux id */
		priv = netdev_priv(node_p->dev);
		if (!priv) {
			rm_err("SHS_LL: priv for netdev is null for hash 0x%x", node_p->hash);
			rmnet_shs_crit_err[RMNET_SHS_NETDEV_ERR]++;
		} else {
			node_p->hstats->mux_id = priv->mux_id;
			rm_err("SHS_LL: mux id for hash 0x%x is %d",
			       node_p->hash, node_p->hstats->mux_id);
		}
		rmnet_shs_cpu_node_add(node_p, &cpu_node_tbl_p->node_list_id);
		hash_add_rcu(rmnet_shs_ll_ht, &node_p->list, skb->hash);
		is_match_found = 1;
		break;

	} while (0);
	spin_unlock_bh(&rmnet_shs_ll_ht_splock);

	if (is_match_found) {
        /* Put on same core if backlog is super low until  it isnt then
         * never do that again unless we moved to gold core and backlog
         * is again super small.
         * State 0 RMNET_SHS_LL_SAME_CORE_SILVER = Same core eligible silver
         * State 1 RMNET_SHS_LL_SILVER_GOLD_NW = No longer same core eligible on silver but ok on gold
         * State 0 RMNET_SHS_LL_SAME_CORE_GOLD = Same core eligible gold
         * State 2 RMNET_SHS_LL_SPLIT_ALWAY = No longer same core eligible on silver and gold
          * */
		rmnet_shs_ll_stamp(skb, node_p);
		if (!node_p->ll_flag &&
		    rmnet_shs_is_lpwr_cpu(raw_smp_processor_id()))  {
			if (GET_QLEN(raw_smp_processor_id()) <  MAX_LL_CORE_BACKLOG &&
			    rmnet_shs_cpu_psb_above_thresh(raw_smp_processor_id(), 4000)) {
				skb->hash = 0;
				skb->sw_hash = 1;
			} else if (!node_p->ll_flag) {
				node_p->ll_flag = RMNET_SHS_LL_SILVER_GOLD_NW;
				node_p->map_cpu = ll_cpu;
				node_p->map_index = rmnet_shs_idx_from_cpu(node_p->map_cpu, map);
			}
		} else if (node_p->ll_flag != RMNET_SHS_LL_SAME_CORE_GOLD){
			if (!rmnet_shs_is_lpwr_cpu(raw_smp_processor_id()))  {
				if (GET_QLEN(raw_smp_processor_id()) < MAX_LL_CORE_BACKLOG &&
				    rmnet_shs_cpu_psb_above_thresh(raw_smp_processor_id(), 12000)) {
				skb->hash = 0;
				skb->sw_hash = 1;

				} else {
					node_p->ll_flag = RMNET_SHS_LL_SAME_CORE_GOLD;
					node_p->map_cpu = ll_cpu;
					node_p->map_index = rmnet_shs_idx_from_cpu(node_p->map_cpu, map);
				}
			}
		}

		if (skb_shinfo(skb)->gso_segs) {
			node_p->num_skb += skb_shinfo(skb)->gso_segs;
			rmnet_shs_cpu_node_tbl[node_p->map_cpu].parkedlen++;
			node_p->skb_list.skb_load += skb_shinfo(skb)->gso_segs;
		} else {
			node_p->num_skb += 1;
			rmnet_shs_cpu_node_tbl[node_p->map_cpu].parkedlen++;
			node_p->skb_list.skb_load++;
		}
		node_p->num_coal_skb += 1;
		node_p->hw_coal_bytes += RMNET_SKB_CB(skb)->coal_bytes;
		node_p->hw_coal_bufsize += RMNET_SKB_CB(skb)->coal_bufsize;
		if (skb->priority == 0xda1a)
			node_p->num_ll_skb++;
		node_p->num_skb_bytes += skb->len;
	}

	rmnet_shs_deliver_skb(skb);
	return 0;
}

void rmnet_shs_ll_init(void)
{
	u8 num_cpu;

	for (num_cpu = 0; num_cpu < MAX_CPUS; num_cpu++)
		INIT_LIST_HEAD(&rmnet_shs_ll_cpu_node_tbl[num_cpu].node_list_id);
}

void rmnet_shs_ll_deinit(void)
{
	struct rmnet_shs_wq_flow_node  *node;
	struct hlist_node *tmp;
	unsigned long bkt;

	rm_err("%s", "SHS_LL: De-init LL book-keeping");
	spin_lock_bh(&rmnet_shs_ll_ht_splock);
	hash_for_each_safe(rmnet_shs_ll_ht, bkt, tmp, node, list)
	{
		hash_del_rcu(&node->list);
	}

	hash_for_each_safe(rmnet_shs_ll_filter_ht, bkt, tmp, node, list)
	{
		hash_del_rcu(&node->list);
		kfree(node);
        rmnet_shs_cfg.num_filters--;
        rmnet_shs_filter_count--;
	}
	spin_unlock_bh(&rmnet_shs_ll_ht_splock);
	rm_err("%s", "SHS_LL: De-init LL book-keeping exit");

}
