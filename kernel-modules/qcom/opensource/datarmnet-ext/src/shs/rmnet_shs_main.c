// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/ip.h>
#include <linux/cpu.h>
#include <linux/oom.h>
#include <net/ip.h>
#include <linux/cpu.h>
#include <linux/bitmap.h>
#include <linux/netdevice.h>
#include <linux/kernel.h>

#include <linux/pm_wakeup.h>
#include <linux/smp.h>

#include <linux/ipv6.h>
#include <linux/netdevice.h>
#include <linux/percpu-defs.h>
#include "rmnet_module.h"
#include "rmnet_shs.h"
#include "rmnet_shs_config.h"
#include "rmnet_shs_wq.h"
#include "rmnet_shs_modules.h"
#include "rmnet_shs_common.h"
#include "rmnet_trace.h"
#include "rmnet_shs_wq_genl.h"
#include "rmnet_shs_ll.h"

#include <linux/module.h>
#include <linux/cpumask.h>
#include <linux/icmp.h>
#include <linux/inet.h>
#include <linux/version.h>
#if (KERNEL_VERSION(6, 5, 0) <= LINUX_VERSION_CODE)
#include <net/gso.h>
#endif
#if (KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE)
#include <net/netdev_rx_queue.h>
#endif

/* Insert newest first, last 4 bytes of the change id */
static char *verinfo[] = {
	"77ef4226",
	"7025ca0f",
	"20dd85e1",
	"08116b3c",
	"33d1744f",
	"148a8c9c",
	"1acc8362",
	"f855e9c0",
	"0b140f5e",
	"e619dd08",
	"670244c5",
};
module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

/* Local Macros */
#define RMNET_SHS_FORCE_FLUSH_TIME_NSEC 2000000
#define NS_IN_MS 1000000
#define LPWR_CLUSTER 0
#define PERF_CLUSTER 4
#define DEF_CORE_WAIT 10

#define PERF_CORES 4
#define INVALID_CPU -1

#define WQ_DELAY 2000000
#define MIN_MS 5
#define BACKLOG_CHECK 1
#define PERF_DISABLE 1
#define GET_IQUEUE(CPU) (per_cpu(softnet_data, CPU).input_pkt_queue)
#define GET_PQUEUE(CPU) (per_cpu(softnet_data, CPU).process_queue)
#define GET_QLEN(CPU) (GET_IQUEUE(CPU).qlen + GET_PQUEUE(CPU).qlen)
#define GET_QTAIL(SD, CPU) (per_cpu(SD, CPU).input_queue_tail)
#define GET_QHEAD(SD, CPU) (per_cpu(SD, CPU).input_queue_head)
#define GET_QHEADS(CPU) (per_cpu(softnet_data, CPU).input_queue_head)
#define GET_QTAILS(CPU) (per_cpu(softnet_data, CPU).input_queue_tail)

#define GET_CTIMER(CPU) rmnet_shs_cfg.core_flush[CPU].core_timer

#define BACKLOG1 0
#define BACKLOG2 1
#define EZ_SWITCH 5

/* Specific CPU RMNET runs on */
#define RMNET_CPU 1
#define SKB_FLUSH 0
#define INCREMENT 1
#define DECREMENT 0

/* Local Definitions and Declarations */
DEFINE_SPINLOCK(rmnet_shs_ht_splock);
DEFINE_HASHTABLE(RMNET_SHS_HT, RMNET_SHS_HT_SIZE);
struct rmnet_shs_cpu_node_s rmnet_shs_cpu_node_tbl[MAX_CPUS];
int cpu_num_flows[MAX_CPUS];

/* Maintains a list of flows associated with a core
 * Also keeps track of number of packets processed on that core
 */

unsigned int rmnet_shs_wake __read_mostly = 0;
module_param(rmnet_shs_wake, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_wake, "wake status");

unsigned int rmnet_shs_perf_duration __read_mostly = 200;
module_param(rmnet_shs_perf_duration, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_duration, "up duty cycle in ms");

unsigned int rmnet_shs_max_qmap_pkt __read_mostly = 850;
module_param(rmnet_shs_max_qmap_pkt, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_max_qmap_pkt, "Max interleaved ipa pkts shs will park");

unsigned int rmnet_shs_max_qmap_wait __read_mostly = 140;
module_param(rmnet_shs_max_qmap_wait, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_max_qmap_pkt, "Max interleaved ipa pkts shs will be park for (ms)");

unsigned int rmnet_shs_max_qmap_steer __read_mostly = 7000;
module_param(rmnet_shs_max_qmap_steer, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_max_qmap_steer, "Max packets shs will steer");

unsigned int rmnet_shs_max_phy_backlog __read_mostly = 0;
module_param(rmnet_shs_max_phy_backlog, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_max_phy_backlog, "Max phy backlog seen");

unsigned int rmnet_shs_esp_pkts __read_mostly = 0;
module_param(rmnet_shs_esp_pkts, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_esp_pkts, " Esp packets seen");

unsigned int rmnet_shs_pb_boost_timer_ms __read_mostly = 50;
module_param(rmnet_shs_pb_boost_timer_ms, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_pb_boost_timer_ms, "Duration that PB Boost will be active for before reset");

struct rmnet_shs_cfg_s rmnet_shs_cfg;
/* This flag is set to true after a successful SHS module init*/

struct rmnet_shs_flush_work shs_rx_work;
int rmnet_shs_get_core_prio_flow(u8 mask);
unsigned rmnet_shs_last_seq = 0;

ssize_t change_rps(struct netdev_rx_queue *queue, u8 cpu, u8 ff)
{
	struct rps_map *old_map, *map;
	cpumask_var_t mask;

	map = rcu_dereference(queue->rps_map);
	if (map) {
		map->cpus[0] = cpu;
		map->len = 1;
		if ((1 << cpu) & PERF_MASK)
			rmnet_shs_switch_reason[RMNET_SHS_PHY_SWITCH_SILVER_TO_G_ACT]++;
		else
			rmnet_shs_switch_reason[RMNET_SHS_PHY_SWITCH_GOLD_TO_S_ACT]++;

		return 0;
	}

	if (!alloc_cpumask_var(&mask, GFP_ATOMIC))
		return -ENOMEM;

	cpumask_set_cpu(cpu, (struct cpumask *) &mask);

	map = kzalloc(max_t(unsigned int,
			    RPS_MAP_SIZE(cpumask_weight(mask)), L1_CACHE_BYTES),
		            GFP_ATOMIC);
	if (!map) {
		free_cpumask_var(mask);
		return -ENOMEM;
	}

	map->cpus[0] = cpu;
	map->len = 1;
	if ((1 << cpu) & PERF_MASK)
		rmnet_shs_switch_reason[RMNET_SHS_PHY_SWITCH_SILVER_TO_G_ACT]++;
	else
		rmnet_shs_switch_reason[RMNET_SHS_PHY_SWITCH_GOLD_TO_S_ACT]++;

	rcu_read_lock();
	/* Any backlog when we switch could potentially lead to ooo */
	old_map = rcu_dereference(queue->rps_map);
	rcu_assign_pointer(queue->rps_map, map);

	kfree(old_map);

	free_cpumask_var(mask);
	rcu_read_unlock();
	return 0;
}
/* Evaluates the incoming transport protocol of the incoming skb. Determines
 * if the skb transport protocol will be supported by SHS module
 */
int rmnet_shs_is_skb_stamping_reqd(struct sk_buff *skb, u8 *ret)
{
	int ret_val = 0;
	struct iphdr *ip4h, __ip4h;
	struct ipv6hdr *ip6h, __ip6h;
	const struct ipv6_opt_hdr *ptr;
	struct ipv6_opt_hdr v6hdr;
	s64 mstime;
	struct timespec64 time;
	int perf_rc;

	if (rmnet_module_hook_perf_ingress(&perf_rc, skb)) {
		if (!perf_rc) {
			goto done;
		}
	}

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

		        (void) ktime_get_boottime_ts64(&time);
			mstime = ktime_ms_delta(ktime_set(time.tv_sec, time.tv_nsec), rmnet_shs_cfg.lpm_ring);

			if (hrtimer_active(&rmnet_shs_cfg.hrtimer_wake)) {
				hrtimer_cancel(&rmnet_shs_cfg.hrtimer_wake);
				    hrtimer_start(&rmnet_shs_cfg.hrtimer_wake, ns_to_ktime(1200* NS_IN_MS), HRTIMER_MODE_REL| HRTIMER_MODE_PINNED);
				rmnet_shs_wake = 2;
			} else {
				hrtimer_start(&rmnet_shs_cfg.hrtimer_wake, ns_to_ktime(1200* NS_IN_MS), HRTIMER_MODE_REL| HRTIMER_MODE_PINNED);
				__pm_stay_awake(rmnet_shs_cfg.ws);
				rmnet_shs_wake = 2;
			}

			if (!hrtimer_active(&rmnet_shs_cfg.hrtimer_lpm)) {
			    hrtimer_start(&rmnet_shs_cfg.hrtimer_lpm, ns_to_ktime(950* NS_IN_MS), HRTIMER_MODE_REL| HRTIMER_MODE_PINNED);
				if (mstime > PING_PERF_DURATION && rmnet_shs_cfg.lpm_ring) {
					rmnet_shs_mid_err[RMNET_SHS_PING_UNOPTIMIZED]++;
				} else {
					/* If we are here, a optimized ICMP has occured and we can turn off boost. */
					*ret = 1;
				}
			} else {
				rmnet_shs_mid_err[RMNET_SHS_PING_UNOPTIMIZED]++;
			}

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
			rmnet_shs_esp_pkts++;
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

		        (void) ktime_get_boottime_ts64(&time);
			mstime = ktime_ms_delta(ktime_set(time.tv_sec, time.tv_nsec), rmnet_shs_cfg.lpm_ring);

			if (hrtimer_active(&rmnet_shs_cfg.hrtimer_wake)) {
				hrtimer_cancel(&rmnet_shs_cfg.hrtimer_wake);
				    hrtimer_start(&rmnet_shs_cfg.hrtimer_wake, ns_to_ktime(1200* NS_IN_MS), HRTIMER_MODE_REL| HRTIMER_MODE_PINNED);
				rmnet_shs_wake = 2;
			} else {
				hrtimer_start(&rmnet_shs_cfg.hrtimer_wake, ns_to_ktime(1200* NS_IN_MS), HRTIMER_MODE_REL| HRTIMER_MODE_PINNED);
				__pm_stay_awake(rmnet_shs_cfg.ws);
				rmnet_shs_wake = 2;
			}

			if (!hrtimer_active(&rmnet_shs_cfg.hrtimer_lpm)) {
			    hrtimer_start(&rmnet_shs_cfg.hrtimer_lpm, ns_to_ktime(950* NS_IN_MS), HRTIMER_MODE_REL| HRTIMER_MODE_PINNED);
				if (mstime > PING_PERF_DURATION && rmnet_shs_cfg.lpm_ring) {
					rmnet_shs_mid_err[RMNET_SHS_PING_UNOPTIMIZED]++;
				} else {
					/* If we are here, a optimized ICMP has occured and we can turn off boost. */
					*ret = 1;
				}
			} else {
				rmnet_shs_mid_err[RMNET_SHS_PING_UNOPTIMIZED]++;
			}

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
			rmnet_shs_esp_pkts++;
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
				rmnet_shs_esp_pkts++;
			}
		}

skip_trace_print_icmp6_rx:
		break;

	default:
		break;
	}

done:
	SHS_TRACE_LOW(RMNET_SHS_SKB_STAMPING, RMNET_SHS_SKB_STAMPING_END,
			    ret_val, 0xDEF, 0xDEF, 0xDEF, skb, NULL);

	return ret_val;
}

static void rmnet_shs_update_core_load(int cpu, int burst)
{

	struct  timespec64 time1;
	struct  timespec64 *time2;
	long curinterval;
	int maxinterval = (rmnet_shs_inst_rate_interval < MIN_MS) ? MIN_MS :
			   rmnet_shs_inst_rate_interval;

	ktime_get_boottime_ts64(&time1);
	time2 = &rmnet_shs_cfg.core_flush[cpu].coretime;

	curinterval = RMNET_SHS_SEC_TO_NSEC(time1.tv_sec - time2->tv_sec)  +
		   time1.tv_nsec - time2->tv_nsec;

	if (curinterval >= maxinterval * NS_IN_MS) {
		if (rmnet_shs_cfg.core_flush[cpu].coresum >
			rmnet_shs_cpu_max_coresum[cpu])
			rmnet_shs_cpu_max_coresum[cpu] = rmnet_shs_cfg.core_flush[cpu].coresum;

		rmnet_shs_cfg.core_flush[cpu].coretime.tv_sec = time1.tv_sec;
		rmnet_shs_cfg.core_flush[cpu].coretime.tv_nsec = time1.tv_nsec;
		rmnet_shs_cfg.core_flush[cpu].coresum = burst;

	} else {
		rmnet_shs_cfg.core_flush[cpu].coresum += burst;
	}

}

/* Delivers skb's to the next module */
void rmnet_shs_deliver_skb(struct sk_buff *skb)
{
	SHS_TRACE_LOW(RMNET_SHS_DELIVER_SKB, RMNET_SHS_DELIVER_SKB_START,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, skb, NULL);
	netif_receive_skb(skb);
}

void rmnet_shs_deliver_skb_wq(struct sk_buff *skb)
{

	SHS_TRACE_LOW(RMNET_SHS_DELIVER_SKB, RMNET_SHS_DELIVER_SKB_START,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, skb, NULL);
	netif_rx(skb);
}

static struct sk_buff *rmnet_shs_skb_partial_segment(struct sk_buff *skb,
						     u16 segments_per_skb)
{
	struct skb_shared_info *shinfo = skb_shinfo(skb);
	struct sk_buff *segments, *tmp;
	u16 gso_size = shinfo->gso_size;
	u16 gso_segs = shinfo->gso_segs;
	unsigned int gso_type = shinfo->gso_type;

	if (segments_per_skb >= gso_segs) {
		return NULL;
	}

	/* Update the numbers for the main skb */
	shinfo->gso_segs = DIV_ROUND_UP(gso_segs, segments_per_skb);
	shinfo->gso_size = gso_size * segments_per_skb;
	segments = __skb_gso_segment(skb, NETIF_F_SG, false);
	if (unlikely(IS_ERR_OR_NULL(segments))) {
		/* return to the original state */
		shinfo->gso_size = gso_size;
		shinfo->gso_segs = gso_segs;
		return NULL;
	}

	/* No need to set gso info if single segments */
	if (segments_per_skb <= 1)
		return segments;

	/* Mark correct number of segments, size, and type in the new skbs */
	for (tmp = segments; tmp; tmp = tmp->next) {
		struct skb_shared_info *new_shinfo = skb_shinfo(tmp);

		new_shinfo->gso_type = gso_type;
		new_shinfo->gso_size = gso_size;

		if (gso_segs >= segments_per_skb)
			new_shinfo->gso_segs = segments_per_skb;
		else
			new_shinfo->gso_segs = gso_segs;

		gso_segs -= segments_per_skb;

		if (gso_segs <= 1) {
			break;
		}
	}

	return segments;
}

/* Delivers skbs after segmenting, directly to network stack */
static void rmnet_shs_deliver_skb_segmented(struct sk_buff *in_skb,
					    u8 ctext,
					    u16 segs_per_skb)
{
	struct sk_buff *skb = NULL;
	struct sk_buff *nxt_skb = NULL;
	struct sk_buff *segs = NULL;
	int count = 0;

	SHS_TRACE_LOW(RMNET_SHS_DELIVER_SKB, RMNET_SHS_DELIVER_SKB_START,
			    0x1, 0xDEF, 0xDEF, 0xDEF, in_skb, NULL);

	segs = rmnet_shs_skb_partial_segment(in_skb, segs_per_skb);

	if (segs == NULL) {
		if (ctext == RMNET_RX_CTXT)
			netif_receive_skb(in_skb);
		else
			netif_rx(in_skb);

		return;
	}

	/* Send segmented skb */
	for ((skb = segs); skb != NULL; skb = nxt_skb) {
		nxt_skb = skb->next;

		skb->hash = in_skb->hash;
		skb->dev = in_skb->dev;
		skb->next = NULL;

		if (ctext == RMNET_RX_CTXT)
			netif_receive_skb(skb);
		else
			netif_rx(skb);

		count += 1;
	}

	consume_skb(in_skb);

	return;
}

int rmnet_shs_flow_num_perf_cores(struct rmnet_shs_skbn_s *node_p)
{
	int ret = 0;
	int core = 1;
	u16 idx = 0;

	for (idx = 0; idx < MAX_CPUS; idx++) {
		if (node_p->hstats->pri_core_msk & core)
			ret++;
		core = core << 1;
	}
	return ret;
}

int rmnet_shs_get_core_prio_flow(u8 mask)
{
	int ret = INVALID_CPU;
	int least_flows = INVALID_CPU;
	u8 curr_idx = 0;
	u8 i;

	/* Return 1st free core or the core with least # flows
	 */
	for (i = 0; i < MAX_CPUS; i++) {

		if (!(mask & (1 << i)))
			continue;

		if (mask & (1 << i))
			curr_idx++;

		if (list_empty(&rmnet_shs_cpu_node_tbl[i].node_list_id))
			return i;

		if (cpu_num_flows[i] <= least_flows ||
		    least_flows == INVALID_CPU) {
			ret = i;
			least_flows = cpu_num_flows[i];
		}

	}

	return ret;
}

/* Take a index and a mask and returns what active CPU is
 * in that index.
 */
static int rmnet_shs_cpu_from_idx(u8 index, u8 mask)
{
	int ret = INVALID_CPU;
	u8 curr_idx = 0;
	u8 i;

	for (i = 0; i < MAX_CPUS; i++) {
		/* If core is enabled & is the index'th core
		 * return that CPU
		 */
		if (curr_idx == index && (mask & (1 << i)))
			return i;

		if (mask & (1 << i))
			curr_idx++;
	}
	return ret;
}

int rmnet_shs_get_suggested_cpu(struct rmnet_shs_skbn_s *node)
{
	int cpu = INVALID_CPU;
	u8 cluster;
	u8 cur_cluster;

	if (node->phy)
		return rmnet_shs_cfg.phy_tcpu;
	/* Return same perf core unless moving to gold from silver*/
	else if (rmnet_shs_cpu_node_tbl[node->map_cpu].prio &&
		 rmnet_shs_is_lpwr_cpu(node->map_cpu)) {

		cpu = rmnet_shs_get_core_prio_flow(PERF_MASK & ~rmnet_shs_cfg.ban_mask &
						   ~rmnet_shs_halt_mask & node->custom_map);

		/* If reserved mask PERF_MASK result in no valid cores, ignore reserve mask */
		if (cpu < 0 && node->hstats != NULL) {
			cpu = node->hstats->suggested_cpu;
		}

	} else if (node->hstats != NULL) {

		cpu = node->hstats->suggested_cpu;
		/* if flow is on a banned cpu or move it to most avaible cpu on same cluster */
		if (((1 << node->map_cpu) & (rmnet_shs_cfg.ban_mask | rmnet_shs_halt_mask)) || !cpu_active(node->map_cpu)) {
			cluster = (PERF_MASK & (1 << cpu))? PERF_MASK : NONPERF_MASK;
			/* If flow lands on banned cpu move up a cluster */
			if (!(cluster & ~rmnet_shs_cfg.ban_mask & ~rmnet_shs_halt_mask & node->custom_map & ~node->map_cpu))
				cluster = PERF_MASK;

			cpu = rmnet_shs_get_core_prio_flow(cluster & ~rmnet_shs_cfg.ban_mask &
							   ~rmnet_shs_halt_mask & node->custom_map);

			if ((1 << node->map_cpu) & (rmnet_shs_cfg.ban_mask))
				rmnet_shs_switch_reason[RMNET_SHS_BANNED_CPU_SUGG]++;
			else if (!cpu_active(node->map_cpu))
				rmnet_shs_switch_reason[RMNET_SHS_CPU_OFFLINE]++;

		} else if  ((1 << cpu) & rmnet_shs_cfg.ban_mask) {
			/* if flow if being told to move to banned cpu.
			 * 1) flow is on same cluster
			 *		- Tough luck wait until ban cpu opens up or shsusrd
			 *                changes suggestion, Stay on same core
			 * 2) flow is on different cluster
			 *	        - Ignore wq suggestion and calculate
			 *                free-est cpu on different cluster
			 */
			cluster = (PERF_MASK & (1 << cpu))? PERF_MASK : NONPERF_MASK;
			cur_cluster = (PERF_MASK & (1 << node->map_cpu))? PERF_MASK : NONPERF_MASK;
			if (!(cluster & ~rmnet_shs_cfg.ban_mask & ~rmnet_shs_halt_mask &  node->custom_map))
				cluster = PERF_MASK;

			if (cluster != cur_cluster)
				cpu = rmnet_shs_get_core_prio_flow(cluster & ~rmnet_shs_cfg.ban_mask & node->custom_map);
			else
				cpu = node->map_cpu;

		} else if (rmnet_shs_cpu_node_tbl[cpu].prio) {
			cpu = node->map_cpu;
		}
	}

	return cpu;
}

int rmnet_shs_get_hash_map_idx_to_stamp(struct rmnet_shs_skbn_s *node)
{
	int cpu, idx = INVALID_CPU;

	cpu = rmnet_shs_get_suggested_cpu(node);
	idx = rmnet_shs_idx_from_cpu(cpu, node->custom_map);

	/* If suggested CPU is no longer in mask. Try using current.*/
	if (unlikely(idx < 0))
		idx = rmnet_shs_idx_from_cpu(node->map_cpu,
					     node->custom_map);

	SHS_TRACE_LOW(RMNET_SHS_HASH_MAP,
			    RMNET_SHS_HASH_MAP_IDX_TO_STAMP,
			    node->hash, cpu, idx, 0xDEF, node, NULL);
	return idx;
}

static int rmnet_shs_is_core_loaded(int cpu, int backlog_check, int parked_pkts)
{
	int ret = 0;

	if (rmnet_shs_cfg.core_flush[cpu].coresum >=
            rmnet_shs_cpu_inst_rate_max_pkts[cpu]) {
		ret = RMNET_SHS_SWITCH_PACKET_BURST;
	}
	if (backlog_check && ((rmnet_shs_get_cpu_qdiff(cpu) + parked_pkts) >=
	    rmnet_shs_cpu_backlog_max_pkts[cpu]))
		ret = RMNET_SHS_SWITCH_CORE_BACKLOG;

	return ret;
}

void rmnet_shs_change_cpu_num_flows(u16 map_cpu, bool inc)
{
	if (map_cpu < MAX_CPUS)
		(inc) ? cpu_num_flows[map_cpu]++: cpu_num_flows[map_cpu]--;
	else
		rmnet_shs_crit_err[RMNET_SHS_CPU_FLOWS_BNDS_ERR]++;
}
/* Takes a snapshot of absolute value of the CPU Qhead and Qtail counts for
 * a given core.
 *
 * CPU qhead reports the count of number of packets processed on a core
 * CPU qtail keeps track of total number of pkts on a core
 * qtail - qhead = pkts yet to be processed by next layer
 */
void rmnet_shs_update_cpu_proc_q(u8 cpu_num)
{
	if (cpu_num >= MAX_CPUS)
		return;

	rcu_read_lock();
	rmnet_shs_cpu_node_tbl[cpu_num].qhead =
	   GET_QHEAD(softnet_data, cpu_num);
	rmnet_shs_cpu_node_tbl[cpu_num].qtail =
	   GET_QTAIL(softnet_data, cpu_num);
	rcu_read_unlock();

	rmnet_shs_cpu_node_tbl[cpu_num].qdiff =
	rmnet_shs_cpu_node_tbl[cpu_num].qtail -
	rmnet_shs_cpu_node_tbl[cpu_num].qhead;

	SHS_TRACE_LOW(RMNET_SHS_CORE_CFG,
			    RMNET_SHS_CORE_CFG_GET_CPU_PROC_PARAMS,
			    cpu_num, rmnet_shs_cpu_node_tbl[cpu_num].qhead,
			    rmnet_shs_cpu_node_tbl[cpu_num].qtail,
			    0xDEF, NULL, NULL);
}

/* Takes a snapshot of absolute value of the CPU Qhead and Qtail counts for
 * all cores.
 *
 * CPU qhead reports the count of number of packets processed on a core
 * CPU qtail keeps track of total number of pkts on a core
 * qtail - qhead = pkts yet to be processed by next layer
 */
void rmnet_shs_update_cpu_proc_q_all_cpus(void)
{
	u8 cpu_num;

	rcu_read_lock();
	for (cpu_num = 0; cpu_num < MAX_CPUS; cpu_num++) {
		if (!cpu_active(cpu_num))
			continue;
		rmnet_shs_update_cpu_proc_q(cpu_num);

		SHS_TRACE_LOW(RMNET_SHS_CORE_CFG,
				    RMNET_SHS_CORE_CFG_GET_CPU_PROC_PARAMS,
				    cpu_num,
				    rmnet_shs_cpu_node_tbl[cpu_num].qhead,
				    rmnet_shs_cpu_node_tbl[cpu_num].qtail,
				    0xDEF, NULL, NULL);
	}
	rcu_read_unlock();

}

int rmnet_shs_phy_switch(struct net_device *dev, u8 cpu, u8 ff)
{
    struct rmnet_shs_msg_resp chg_msg;


	if (rmnet_is_real_dev_registered(dev)) {
		if (change_rps(dev->_rx, cpu, ff) == 0) {
			rmnet_shs_cfg.ban_mask = 1 << cpu;
			rmnet_shs_cfg.phy_old_cpu = rmnet_shs_cfg.phy_acpu;
			rmnet_shs_cfg.phy_acpu = cpu;
			rmnet_shs_create_phy_msg_resp(&chg_msg, rmnet_shs_cfg.phy_old_cpu, cpu);
			rmnet_shs_genl_msg_direct_send_to_userspace(&chg_msg);
		} else {
			return -1;
		}
	} else {
		rmnet_shs_crit_err[RMNET_SHS_FAILED_RPS_CHANGE]++;
	}
	return 0;
}

int rmnet_shs_node_can_flush_pkts(struct rmnet_shs_skbn_s *node, u8 force_flush, u8 ctxt)
{
	int cpu_map_index;
	u32 cur_cpu_qhead;
	u32 node_qhead;
	int ret = 0;
	int prev_cpu = -1;
	int ccpu;
	int cpu_num;
	int new_cpu;
	struct rmnet_shs_cpu_node_s *cpun;
	u8 map = node->custom_map;

	cpu_map_index = rmnet_shs_get_hash_map_idx_to_stamp(node);
	do {
		prev_cpu = node->map_cpu;
		if (cpu_map_index < 0) {
			node->is_shs_enabled = 0;
			ret = 1;
			break;
		}
		node->is_shs_enabled = 1;
		if (!map) {
			node->is_shs_enabled = 0;
			ret = 1;
			break;
		}

		/* If the flow is going to the same core itself
		 */
		if (cpu_map_index == node->map_index) {
			ret = 1;
			break;
		}

		if (!node->phy)
			cur_cpu_qhead = rmnet_shs_get_cpu_qhead(node->map_cpu);
		else
			cur_cpu_qhead = rmnet_shs_get_cpu_qhead(rmnet_shs_cfg.phy_acpu);

		node_qhead = node->queue_head;
		cpu_num = node->map_cpu;

		/* OfO packets are avoided here by looking at required qhead of old cpu, if qlen is 0 then qhead/qtail
		 * req should have already been hit.
		 * Qmap flow has to be flushed by qmap packet. Data packet would mean interleaving of dl_trl/hdr
		 */
		if ((!node->phy && cur_cpu_qhead >= (node_qhead + node->qhead_offset))  ||
		   (force_flush >= RMNET_SHS_FF_GLOBAL || (force_flush && node->phy)) ||
		   /* Emtpy old cpu will also cause a switch */
		   /* Phy flows old cpu will be empty but OOO is still posible, focus on */
		    (!node->phy && !rmnet_shs_get_cpu_qdiff(cpu_num) &&
		      ++rmnet_shs_flush_reason[RMNET_SHS_FLUSH_Z_QUEUE_FLUSH]))  {
			if (likely(rmnet_shs_switch_cores)) {
				new_cpu = rmnet_shs_cpu_from_idx(cpu_map_index,
								 node->custom_map);

				if (new_cpu < 0) {
					ret = 1;
					break;
				}
				/* move parked counter stats*/
				rmnet_shs_cpu_node_tbl[new_cpu].parkedlen += node->skb_list.num_parked_skbs;
				rmnet_shs_cpu_node_tbl[node->map_cpu].parkedlen -= node->skb_list.num_parked_skbs;
				/* Chnage node cpus */
				node->map_index = cpu_map_index;
				node->map_cpu = new_cpu;
				ccpu = node->map_cpu;

				/* If current cpu's has not processed pased last enqueued packet's qtail + offset*/
				if (cur_cpu_qhead < (node_qhead + node->qhead_offset)) {

					rmnet_shs_switch_reason[RMNET_SHS_OOO_PACKET_SWITCH]++;
					rmnet_shs_switch_reason[RMNET_SHS_OOO_PACKET_TOTAL] +=
							(node_qhead - cur_cpu_qhead) + node->qhead_offset;
					rmnet_shs_cpu_ooo(cpu_num, (node_qhead - cur_cpu_qhead) + node->qhead_offset);
					pr_info("shs ooo: phy %d new_cpu %d old_cpu %d, offset %d node_qhead %d, cur_qhead %d, ff %d old cpu qdiff %d new qdiff %d \n",
						node->phy, new_cpu, cpu_num, node->qhead_offset, node_qhead, cur_cpu_qhead,
						force_flush, rmnet_shs_get_cpu_qdiff(cpu_num), rmnet_shs_get_cpu_qdiff(new_cpu) );

				}

				/* Mark gold core as prio to prevent
				 * flows from moving in wq
				 */
				if (rmnet_shs_cpu_node_tbl[cpu_num].prio) {
					rmnet_shs_cpu_node_tbl[ccpu].wqprio = 1;
					rmnet_shs_switch_reason[RMNET_SHS_SWITCH_INSTANT_RATE]++;

				} else if ((1 << cpu_num ) & rmnet_shs_halt_mask) {
					rmnet_shs_switch_reason[RMNET_SHS_RESERVED_CPU_SUGG]++;

                }
                else {
					rmnet_shs_switch_reason[RMNET_SHS_SWITCH_WQ_RATE]++;
				}
				cpun = &rmnet_shs_cpu_node_tbl[node->map_cpu];
				rmnet_shs_update_cpu_proc_q_all_cpus();
				/* This is equal to rmnet_shs_get_cpu_qhead, used for < 80 limit*/
				node->queue_head = cpun->qhead;
				node->queue_head += node->skb_list.num_parked_skbs;
				if (node->hstats)
					node->hstats->suggested_cpu = ccpu;
				rmnet_shs_cpu_node_move(node,
							&cpun->node_list_id,
							cpu_num);
				SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
					RMNET_SHS_FLUSH_NODE_CORE_SWITCH,
					node->map_cpu, prev_cpu,
					0xDEF, 0xDEF, node, NULL);
			}
			ret = 1;
		}
	} while (0);

	SHS_TRACE_LOW(RMNET_SHS_FLUSH,
			    RMNET_SHS_FLUSH_CHK_NODE_CAN_FLUSH,
			    ret, node->map_cpu, prev_cpu,
			    0xDEF, node, NULL);
	return ret;
}

void rmnet_shs_flush_core(u8 cpu_num)
{
	struct rmnet_shs_skbn_s *n;
	struct list_head *ptr, *next;
	u32 cpu_tail;
	u32 num_pkts_flush = 0;
	u32 num_bytes_flush = 0;
	u32 total_pkts_flush = 0;
	u32 total_bytes_flush = 0;

	/* Record a qtail + pkts flushed or move if reqd
	 * currently only use qtail for non TCP flows
	 */
	rmnet_shs_update_cpu_proc_q_all_cpus();
	SHS_TRACE_HIGH(RMNET_SHS_FLUSH, RMNET_SHS_FLUSH_START,
			     rmnet_shs_cfg.num_pkts_parked,
			     rmnet_shs_cfg.num_bytes_parked,
			     0xDEF, 0xDEF, NULL, NULL);
	spin_lock_bh(&rmnet_shs_ht_splock);
		cpu_tail = rmnet_shs_get_cpu_qtail(cpu_num);
		list_for_each_safe(ptr, next,
			&rmnet_shs_cpu_node_tbl[cpu_num].node_list_id) {
			n = list_entry(ptr, struct rmnet_shs_skbn_s, node_id);
			if (n->skb_list.num_parked_skbs) {
				num_pkts_flush = n->skb_list.num_parked_skbs;
				num_bytes_flush = n->skb_list.num_parked_bytes;

				rmnet_shs_chk_and_flush_node(n,
							     RMNET_SHS_FF_CORE_FLUSH,
							     RMNET_WQ_CTXT, NULL);

				total_pkts_flush += num_pkts_flush;
				total_bytes_flush += num_bytes_flush;
				if (n->map_cpu == cpu_num) {
					cpu_tail += num_pkts_flush;
					n->queue_head = cpu_tail;
				}
			}

		}

	rmnet_shs_cfg.num_bytes_parked -= total_bytes_flush;
	rmnet_shs_cfg.num_pkts_parked -= total_pkts_flush;
	rmnet_shs_cpu_node_tbl[cpu_num].prio = 0;
	/* Reset coresum in case of instant rate switch */
	rmnet_shs_cfg.core_flush[cpu_num].coresum = 0;
	rmnet_shs_cpu_node_tbl[cpu_num].parkedlen = 0;
	spin_unlock_bh(&rmnet_shs_ht_splock);

	/* This is needed incase no new cpu packets were parked so none
	 * would be flushed and execute below in flush_node
	 */
	if (cpu_num == rmnet_shs_cfg.phy_old_cpu &&
	    rmnet_module_hook_is_set(RMNET_MODULE_HOOK_SHS_SWITCH)) {
		rmnet_shs_switch_disable();
		rmnet_shs_cfg.max_phy_steer = 0;
		/*Just force flushed, change old cpu to current cpu */
		rmnet_shs_cfg.phy_old_cpu = rmnet_shs_cfg.phy_acpu;
	}

	SHS_TRACE_HIGH(RMNET_SHS_FLUSH, RMNET_SHS_FLUSH_END,
	     rmnet_shs_cfg.num_pkts_parked,
			     rmnet_shs_cfg.num_bytes_parked,
			     total_pkts_flush, total_bytes_flush, NULL, NULL);

}

void rmnet_shs_flush_core_work(struct work_struct *work)
{
	struct core_flush_s *core_work = container_of(work,
				 struct core_flush_s, work);

	rmnet_shs_flush_core(core_work->core);
	rmnet_shs_flush_reason[RMNET_SHS_FLUSH_WQ_CORE_FLUSH]++;
}

/* Flushes all the packets parked in order for this flow */
void rmnet_shs_flush_node(struct rmnet_shs_skbn_s *node, u8 ctext, struct sk_buff **phy_list)
{
	struct sk_buff *skb = NULL;
	struct sk_buff *nxt_skb = NULL;

	u32 skbs_delivered = 0;
	u32 skb_bytes_delivered = 0;
	u32 hash2stamp = 0; /* the default value of skb->hash*/
	u8 map = 0, maplen = 0;
	u16 segs_per_skb = 0;
	u16 index;
	u8 asynccore = 0;
	u8 cpu;

	if (!node->skb_list.head)
		return;

	if (!node->custom_map) {
		map = rmnet_shs_cfg.map_mask;
		maplen = rmnet_shs_cfg.map_len;
		index = node->map_index;
	}
	else {
		map = node->custom_map;
		maplen = node->custom_len;
		index = node->map_index;
	}

	cpu  = node->map_cpu;

	if (rmnet_shs_cpu_node_tbl[cpu].async) {
		asynccore = rmnet_shs_cpu_node_tbl[cpu].async && rmnet_shs_no_sync_off;
	}

	if (map) {
		hash2stamp = rmnet_shs_form_hash(index,
						 maplen,
						 node->skb_list.head->hash, asynccore);
	} else {
		node->is_shs_enabled = 0;
	}
	SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
			     RMNET_SHS_FLUSH_NODE_START,
			     node->hash, hash2stamp,
			     node->skb_list.num_parked_skbs,
			     node->skb_list.num_parked_bytes,
			     node, node->skb_list.head);

	segs_per_skb = (u16) node->hstats->segs_per_skb;

	for ((skb = node->skb_list.head); skb != NULL; skb = nxt_skb) {

		nxt_skb = skb->next;
		if (node->is_shs_enabled)
			skb->hash = hash2stamp;

		skb->next = NULL;
		skbs_delivered += 1;
		skb_bytes_delivered += skb->len;
		if(asynccore)
			rmnet_shs_no_sync_packets++;

		if (segs_per_skb > 0 && !node->phy) {
			if (node->skb_tport_proto == IPPROTO_UDP)
				rmnet_shs_crit_err[RMNET_SHS_UDP_SEGMENT]++;
			rmnet_shs_deliver_skb_segmented(skb, ctext,
							segs_per_skb);
		} else {
		/* Don't touch this or rmnet_shs_clear_node  unless
		 * you know what you are doing. QMAP skb need opposite
		 * delivery function than regular skbs.
		 */

			if (ctext == RMNET_RX_CTXT && (!node->phy))
				rmnet_shs_deliver_skb(skb);
			else if (ctext == RMNET_WQ_CTXT && (!node->phy ))
				rmnet_shs_deliver_skb_wq(skb);
			else if (node->phy && ctext == RMNET_RX_CTXT && phy_list) {
				/* Start doubly linked phy list if 1st go else append
				 * Most recently added is saved in phy-prev and updated each iter
				 * */
				if (!*phy_list) {
					skb->prev = skb;
					*phy_list = skb;
				} else {
					(*phy_list)->prev->next = skb;
					(*phy_list)->prev = skb;
				}

			} else {
				rmnet_shs_flush_reason[RMNET_SHS_FLUSH_PHY_WQ_FLUSH]++;
				netif_rx(skb);
			}
		}
	}

	node->skb_list.num_parked_skbs = 0;
	node->skb_list.num_parked_bytes = 0;
	node->skb_list.head = NULL;
	node->skb_list.tail = NULL;

	SHS_TRACE_HIGH(RMNET_SHS_FLUSH, RMNET_SHS_FLUSH_NODE_END,
			     node->hash, hash2stamp,
			     skbs_delivered, skb_bytes_delivered, node, NULL);
}

void rmnet_shs_clear_node(struct rmnet_shs_skbn_s *node, u8 ctxt)
{
	struct sk_buff *skb;
	struct sk_buff *nxt_skb = NULL;
	u32 skbs_delivered = 0;
	u32 skb_bytes_delivered = 0;
	u32 hash2stamp;
	u8 map, maplen;

	if (!node->skb_list.head)
		return;
	map = node->custom_map;
	maplen = node->custom_len;

	if (map) {
		hash2stamp = rmnet_shs_form_hash(node->map_index,
						 maplen,
						 node->skb_list.head->hash, 0);
	} else {
		node->is_shs_enabled = 0;
	}

	for ((skb = node->skb_list.head); skb != NULL; skb = nxt_skb) {
		nxt_skb = skb->next;
		if (node->is_shs_enabled)
			skb->hash = hash2stamp;

		skb->next = NULL;
		skbs_delivered += 1;
		skb_bytes_delivered += skb->len;

		/* Don't touch this or rmnet_shs_clear_node  unless
		 * you know what you are doing. QMAP skb need opposite
		 * delivery function than regular skbs.
		 */
		if (ctxt == RMNET_RX_CTXT && (!node->phy ))
			rmnet_shs_deliver_skb(skb);
		else if (ctxt == RMNET_WQ_CTXT && (!node->phy ))
			rmnet_shs_deliver_skb_wq(skb);
		else {
			netif_rx(skb);
		}

	}
	rmnet_shs_crit_err[RMNET_SHS_WQ_COMSUME_PKTS]++;
	rmnet_shs_cfg.num_bytes_parked -= skb_bytes_delivered;
	rmnet_shs_cfg.num_pkts_parked -= skbs_delivered;
	rmnet_shs_cpu_node_tbl[node->map_cpu].parkedlen -= skbs_delivered;
}

/* Evaluates if all the packets corresponding to a particular flow can
 * be flushed.
 */
int rmnet_shs_chk_and_flush_node(struct rmnet_shs_skbn_s *node,
				 u8 force_flush, u8 ctxt, struct sk_buff **phy_list)
{
	int ret_val = 0;
	/* Shoud stay int for error reporting*/
	int map = node->custom_map;
	int map_idx;

	SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
			     RMNET_SHS_FLUSH_CHK_AND_FLUSH_NODE_START,
			     force_flush, ctxt, 0xDEF, 0xDEF,
			     node, NULL);
	/* Return saved cpu assignment if an entry found */
	if (rmnet_shs_cpu_from_idx(node->map_index, map) != node->map_cpu) {

		/* Keep flow on the same core if possible
		 * or put Orphaned flow on the default 1st core
		 */
		map_idx = rmnet_shs_idx_from_cpu(node->map_cpu,
							map);
		if (map_idx >= 0) {
			node->map_index = map_idx;
			node->map_cpu = rmnet_shs_cpu_from_idx(map_idx, map);

		} else {
			/*Put on default Core if no match*/
			int map_cpu = rmnet_shs_cpu_from_idx(MAIN_CORE, map);

			node->map_index = MAIN_CORE;
			if (map_cpu < 0)
				node->map_cpu = MAIN_CORE;
			else
				node->map_cpu = map_cpu;
		}
		force_flush = 1;
		rmnet_shs_crit_err[RMNET_SHS_RPS_MASK_CHANGE]++;

		SHS_TRACE_ERR(RMNET_SHS_ASSIGN,
					RMNET_SHS_ASSIGN_MASK_CHNG,
					0xDEF, 0xDEF, 0xDEF, 0xDEF,
					NULL, NULL);
	}

	if (rmnet_shs_node_can_flush_pkts(node, force_flush, ctxt)) {
		rmnet_shs_flush_node(node, ctxt, phy_list);
		ret_val = 1;
	}

	SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
			     RMNET_SHS_FLUSH_CHK_AND_FLUSH_NODE_END,
			     ret_val, force_flush, 0xDEF, 0xDEF,
			     node, NULL);
	return ret_val;
}

/* Check if cpu_num should be marked as a priority core and  take care of
 * marking it as priority and configuring  all the changes need for a core
 * switch.
 * Getting marked as prio will divert non-qmap flows to gold core, trigger core flush, Boost silver cpus
 */
static void rmnet_shs_core_prio_check(u8 cpu_num, u8 segmented, u32 parked_pkts)
{
	u32 wait = (!rmnet_shs_max_core_wait) ? 1 : rmnet_shs_max_core_wait;
	int load_reason;

	if ((load_reason = rmnet_shs_is_core_loaded(cpu_num, segmented, parked_pkts)) &&
	    rmnet_shs_is_lpwr_cpu(cpu_num) &&
	    /* Avoid prioritizing phy ramp up core since that will reset the timer */
	    !rmnet_shs_cpu_node_tbl[cpu_num].prio && rmnet_shs_cfg.phy_old_cpu != cpu_num && cpu_num != 1) {

		wait = (!segmented)? DEF_CORE_WAIT: wait;
		rmnet_shs_cpu_node_tbl[cpu_num].prio = 1;
		rmnet_shs_boost_cpus();
		if (hrtimer_active(&GET_CTIMER(cpu_num)))
			hrtimer_cancel(&GET_CTIMER(cpu_num));

		hrtimer_start(&GET_CTIMER(cpu_num),
				ns_to_ktime(wait * NS_IN_MS),
				HRTIMER_MODE_REL);

		rmnet_shs_switch_reason[load_reason]++;
	}
}

/* Flushes all the packets that have been parked so far across all the flows
 * The order of flushing depends on the CPU<=>flow association
 * The flows associated with low power cores are flushed before flushing
 * packets of all the flows associated with perf core.
 *
 * If more than two flows are associated with the same CPU, the packets
 * corresponding to the most recent flow will be flushed first
 *
 * Each time a flushing is invoked we also keep track of the number of
 * packets waiting & have been processed by the next layers.
 */

void rmnet_shs_flush_lock_table(u8 flsh, u8 ctxt)
{
	struct rmnet_shs_skbn_s *n = NULL;
	struct rmnet_shs_skbn_s *phy_node = NULL;
	struct list_head *ptr = NULL, *next = NULL;
	struct sk_buff *phy_list = NULL;
	int cpu_num;
	u32 cpu_tail;
	u32 num_pkts_flush = 0;
	u32 num_bytes_flush = 0;
	u32 skb_seg_pending = 0;
	u32 total_pkts_flush = 0;
	u32 total_bytes_flush = 0;
	u32 total_cpu_gro_flushed = 0;
	u32 total_node_gro_flushed = 0;
	u8 is_flushed = 0;
	struct sk_buff *skb = NULL;
	struct sk_buff *nxt_skb = NULL;
	struct rmnet_shs_cpu_node_s *cpu_node_tbl_p = NULL;

	/* Record a qtail + pkts flushed or move if reqd
	 * currently only use qtail for non TCP flows
	 */
	rmnet_shs_update_cpu_proc_q_all_cpus();
	SHS_TRACE_HIGH(RMNET_SHS_FLUSH, RMNET_SHS_FLUSH_START,
			     rmnet_shs_cfg.num_pkts_parked,
			     rmnet_shs_cfg.num_bytes_parked,
			     0xDEF, 0xDEF, NULL, NULL);

	for (cpu_num = 0; cpu_num < MAX_CPUS; cpu_num++) {

		cpu_tail = rmnet_shs_get_cpu_qtail(cpu_num);
		total_cpu_gro_flushed = 0;
		skb_seg_pending = 0;
		list_for_each_safe(ptr, next,
				   &rmnet_shs_cpu_node_tbl[cpu_num].node_list_id) {
			n = list_entry(ptr, struct rmnet_shs_skbn_s, node_id);
			skb_seg_pending += n->skb_list.skb_load;
		}
		if (rmnet_shs_inst_rate_switch) {
			rmnet_shs_core_prio_check(cpu_num, BACKLOG_CHECK,
						  skb_seg_pending);
		}

		list_for_each_safe(ptr, next,
				   &rmnet_shs_cpu_node_tbl[cpu_num].node_list_id) {
			n = list_entry(ptr, struct rmnet_shs_skbn_s, node_id);

			if (n->skb_list.num_parked_skbs) {
				if (n->phy)
					phy_node = n;
				num_pkts_flush = n->skb_list.num_parked_skbs;
				num_bytes_flush = n->skb_list.num_parked_bytes;
				total_node_gro_flushed = n->skb_list.skb_load;

				is_flushed = rmnet_shs_chk_and_flush_node(n,
									  flsh,
									  ctxt, &phy_list);

				if (is_flushed) {
					total_cpu_gro_flushed += total_node_gro_flushed;
					total_pkts_flush += num_pkts_flush;
					total_bytes_flush += num_bytes_flush;
					rmnet_shs_cpu_node_tbl[n->map_cpu].parkedlen -= num_pkts_flush;
					n->skb_list.skb_load = 0;
					if (n->map_cpu == cpu_num) {
						cpu_tail += num_pkts_flush;
						n->queue_head = cpu_tail;

					}
				}

			}

		}

		/* If core is loaded set core flows as priority and
		 * start a 10ms hard flush timer
		 */
		if (rmnet_shs_inst_rate_switch) {
			/* Update cpu load with prev flush for check */
			if (rmnet_shs_is_lpwr_cpu(cpu_num) &&
			    !rmnet_shs_cpu_node_tbl[cpu_num].prio)
				rmnet_shs_update_core_load(cpu_num,
				total_cpu_gro_flushed);

			rmnet_shs_core_prio_check(cpu_num, BACKLOG_CHECK, 0);

		}

		if (rmnet_shs_cpu_node_tbl[cpu_num].parkedlen < 0)
			rmnet_shs_crit_err[RMNET_SHS_CPU_PKTLEN_ERR]++;

		if (rmnet_shs_get_cpu_qdiff(cpu_num) >=
		    rmnet_shs_cpu_max_qdiff[cpu_num])
			rmnet_shs_cpu_max_qdiff[cpu_num] =
					rmnet_shs_get_cpu_qdiff(cpu_num);
	}

	rmnet_shs_cfg.num_bytes_parked -= total_bytes_flush;
	rmnet_shs_cfg.num_pkts_parked -= total_pkts_flush;

	/* Flushing phy flow */
	if (phy_list && phy_node) {
		/* RPS is on correct core, old core shoudl have been drained into new cpu,
		 * New CPU is detouring into SHS to park until old cpu backlog on new cpu is done.
		 * Parked packets have been drained on this cpu in front of existing cpu backlog.
		 * Turning off Ptr should maintain the order and the detour isn't needed.
		 */
		cpu_node_tbl_p = &rmnet_shs_cpu_node_tbl[phy_node->map_cpu];
		rmnet_shs_cfg.kfree_stop = 1;
		spin_unlock_bh(&rmnet_shs_ht_splock);
		phy_node->qhead_offset = 0;
		for ((skb = phy_list); skb != NULL; skb = nxt_skb) {
			nxt_skb = skb->next;
			skb->next = NULL;
			rmnet_shs_cpu_node_remove(phy_node);
			rmnet_rx_handler(&skb);
			rmnet_shs_cpu_node_add(phy_node, &cpu_node_tbl_p->node_list_id);
		}
		spin_lock_bh(&rmnet_shs_ht_splock);
		rmnet_shs_switch_disable();
		rmnet_shs_cfg.kfree_stop = 0;
		/*Just force flushed, change old cpu to current cpu */
		rmnet_shs_cfg.max_phy_steer = 0;
		rmnet_shs_cfg.phy_old_cpu = rmnet_shs_cfg.phy_acpu;

	}

	SHS_TRACE_HIGH(RMNET_SHS_FLUSH, RMNET_SHS_FLUSH_END,
			     rmnet_shs_cfg.num_pkts_parked,
			     rmnet_shs_cfg.num_bytes_parked,
			     total_pkts_flush, total_bytes_flush, NULL, NULL);

	if ((rmnet_shs_cfg.num_bytes_parked <= 0) ||
	    (rmnet_shs_cfg.num_pkts_parked <= 0)) {

		rmnet_shs_cfg.num_bytes_parked = 0;
		rmnet_shs_cfg.num_pkts_parked = 0;
		rmnet_shs_cfg.is_pkt_parked = 0;
		rmnet_shs_cfg.force_flush_state = RMNET_SHS_FLUSH_DONE;
	}
}

void rmnet_shs_flush_table(u8 flsh, u8 ctxt)
{
	/* Spinlock bh will only block softirqs not hwirqs
	 * This is fine but hrtimers we start can interrutpt us now
	 * But they dont spinlock themselves so it is fine.
	 */
	spin_lock_bh(&rmnet_shs_ht_splock);
	rmnet_shs_flush_lock_table(flsh, ctxt);
	spin_unlock_bh(&rmnet_shs_ht_splock);

	if (ctxt == RMNET_WQ_CTXT) {
		/* If packets remain restart the timer in case there are no
		* more NET_RX flushes coming so pkts are no lost
		*/
		if (rmnet_shs_fall_back_timer &&
		    rmnet_shs_cfg.num_bytes_parked &&
		    rmnet_shs_cfg.num_pkts_parked) {

			if (hrtimer_active(&rmnet_shs_cfg.hrtimer_shs))
				hrtimer_cancel(&rmnet_shs_cfg.hrtimer_shs);

			hrtimer_start(&rmnet_shs_cfg.hrtimer_shs,
				      ns_to_ktime(rmnet_shs_timeout * NS_IN_MS),
				      HRTIMER_MODE_REL);
		}
		rmnet_shs_flush_reason[RMNET_SHS_FLUSH_WQ_FB_FLUSH]++;
	}
}

/* After we have decided to handle the incoming skb we park them in order
 * per flow
 */
void rmnet_shs_chain_to_skb_list(struct sk_buff *skb,
				 struct rmnet_shs_skbn_s *node,
				 struct rmnet_shs_clnt_s *clnt_cfg)
{
	u8 pushflush = 0;

	/* Early flush for TCP if PSH packet.
	 * Flush before parking PSH packet.
	 * Legacy Behavior. Deprecrecate
	 */
#ifdef CONFIG_PSH
	if (!(clnt_cfg->config & RMNET_SHS_NO_PSH) && skb->cb[SKB_FLUSH]) {
		rmnet_shs_flush_lock_table(0, RMNET_RX_CTXT);
		rmnet_shs_flush_reason[RMNET_SHS_FLUSH_PSH_PKT_FLUSH]++;
		pushflush = 1;
	}
#endif
	/* Support for gso marked packets */
	if (skb_shinfo(skb)->gso_segs) {
		node->num_skb += skb_shinfo(skb)->gso_segs;
		rmnet_shs_cpu_node_tbl[node->map_cpu].parkedlen++;
		node->skb_list.skb_load += skb_shinfo(skb)->gso_segs;
	} else {
		node->num_skb += 1;
		rmnet_shs_cpu_node_tbl[node->map_cpu].parkedlen++;
		node->skb_list.skb_load++;
	}
	node->num_coal_skb += 1;
	node->hw_coal_bytes += RMNET_SKB_CB(skb)->coal_bytes;
	node->hw_coal_bufsize += RMNET_SKB_CB(skb)->coal_bufsize;
	node->bif = RMNET_SKB_CB(skb)->bif;
	node->ack_thresh = RMNET_SKB_CB(skb)->ack_thresh;

	node->num_skb_bytes += skb->len;
	node->skb_list.num_parked_bytes += skb->len;
	rmnet_shs_cfg.num_bytes_parked  += skb->len;

	if (node->skb_list.num_parked_skbs > 0) {
		node->skb_list.tail->next = skb;
		node->skb_list.tail = node->skb_list.tail->next;
	} else {
		node->skb_list.head = skb;
		node->skb_list.tail = skb;
	}

	/* skb_list.num_parked_skbs Number of packets are parked for this flow
	 */
	node->skb_list.num_parked_skbs += 1;
	rmnet_shs_cfg.num_pkts_parked  += 1;

	if (unlikely(pushflush))
		rmnet_shs_flush_lock_table(0, RMNET_RX_CTXT);

	SHS_TRACE_HIGH(RMNET_SHS_ASSIGN,
			     RMNET_SHS_ASSIGN_PARK_PKT_COMPLETE,
			     node->skb_list.num_parked_skbs,
			     node->skb_list.num_parked_bytes,
			     rmnet_shs_cfg.num_pkts_parked,
			     rmnet_shs_cfg.num_bytes_parked,
			     skb, node);
}
/* Invoked when all the packets that are parked to be flushed through
 * the workqueue.
 */
static void rmnet_flush_buffered(struct work_struct *work)
{
	SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
			     RMNET_SHS_FLUSH_DELAY_WQ_START, 0,
			     rmnet_shs_cfg.force_flush_state, 0xDEF,
			     0xDEF, NULL, NULL);

	if (rmnet_shs_cfg.num_pkts_parked &&
	   rmnet_shs_cfg.force_flush_state == RMNET_SHS_FLUSH_ON) {
		local_bh_disable();
		rmnet_shs_flush_table(0, RMNET_WQ_CTXT);
		local_bh_enable();
	}
	SHS_TRACE_HIGH(RMNET_SHS_FLUSH, RMNET_SHS_FLUSH_DELAY_WQ_END, 0,
		       0xDEF, 0xDEF, 0xDEF, NULL, NULL);
}
/* Invoked when the flushing timer has expired.
 * Upon first expiry, we set the flag that will trigger force flushing of all
 * packets that have been parked so far. The timer is then restarted
 *
 * Upon the next expiry, if the packets haven't yet been delivered to the
 * next layer, a workqueue will be scheduled to flush all the parked packets.
 */
enum hrtimer_restart rmnet_shs_map_flush_queue(struct hrtimer *t)
{
	enum hrtimer_restart ret = HRTIMER_NORESTART;

	SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
			     RMNET_SHS_FLUSH_PARK_TMR_EXPIRY,
			     rmnet_shs_cfg.force_flush_state, 0xDEF,
			     0xDEF, 0xDEF, NULL, NULL);
	if (rmnet_shs_cfg.num_pkts_parked > 0) {
		if (rmnet_shs_cfg.force_flush_state == RMNET_SHS_FLUSH_OFF) {
			rmnet_shs_cfg.force_flush_state = RMNET_SHS_FLUSH_ON;
			hrtimer_forward(t, hrtimer_cb_get_time(t),
					ns_to_ktime(WQ_DELAY));
			ret = HRTIMER_RESTART;

			SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
					     RMNET_SHS_FLUSH_PARK_TMR_RESTART,
					     rmnet_shs_cfg.num_pkts_parked,
					     0xDEF, 0xDEF, 0xDEF, NULL, NULL);
		} else if (rmnet_shs_cfg.force_flush_state ==
			   RMNET_SHS_FLUSH_DONE) {
			rmnet_shs_cfg.force_flush_state = RMNET_SHS_FLUSH_OFF;

		} else if (rmnet_shs_cfg.force_flush_state ==
			   RMNET_SHS_FLUSH_ON) {
			SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
					     RMNET_SHS_FLUSH_DELAY_WQ_TRIGGER,
					     rmnet_shs_cfg.force_flush_state,
					     0xDEF, 0xDEF, 0xDEF, NULL, NULL);
			schedule_work((struct work_struct *)&shs_rx_work);
		}
	}
	return ret;
}
enum hrtimer_restart rmnet_shs_wake_ring(struct hrtimer *t)
{
	__pm_relax(rmnet_shs_cfg.ws);

	rmnet_shs_wake = 1;
	return HRTIMER_NORESTART;
}

enum hrtimer_restart rmnet_shs_lpm_ring(struct hrtimer *t)
{
	struct rmnet_shs_msg_resp boost_msg;
	struct timespec64 time;

	/* Create boost msg and deliver using direct msg channel to shsusrd */
	rmnet_shs_create_ping_boost_msg_resp(rmnet_shs_perf_duration, &boost_msg);
	rmnet_shs_genl_msg_direct_send_to_userspace(&boost_msg);
	ktime_get_boottime_ts64(&time);

	rmnet_shs_cfg.lpm_ring =  ktime_set(time.tv_sec, time.tv_nsec);

	return HRTIMER_NORESTART;
}

enum hrtimer_restart rmnet_shs_queue_core(struct hrtimer *t)
{
	struct core_flush_s *core_work = container_of(t,
				 struct core_flush_s, core_timer);

	rmnet_shs_reset_cpus();

	schedule_work(&core_work->work);
	return HRTIMER_NORESTART;
}

enum hrtimer_restart rmnet_shs_pb_unboost_cpus(struct hrtimer *t)
{
	SHS_TRACE_LOW(RMNET_SHS_PB_BOOST_CPU, RMNET_SHS_PB_BOOST_CPU_RESET, 0, 0,
				  0xDEF, 0xDEF, NULL, NULL);
	rmnet_shs_pb_reset_cpus();

	return HRTIMER_NORESTART;
}

void rmnet_shs_rx_wq_init(void)
{
	int i;

	/* Initialize a timer/work for each core for switching */
	for (i = 0; i < MAX_CPUS; i++) {
		rmnet_shs_cfg.core_flush[i].core = i;
		INIT_WORK(&rmnet_shs_cfg.core_flush[i].work,
			  rmnet_shs_flush_core_work);

		hrtimer_init(&rmnet_shs_cfg.core_flush[i].core_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		rmnet_shs_cfg.core_flush[i].core_timer.function =
							rmnet_shs_queue_core;
	}
	/* Initialize a fallback/failsafe work for when dl ind fails */
	hrtimer_init(&rmnet_shs_cfg.hrtimer_shs,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rmnet_shs_cfg.hrtimer_shs.function = rmnet_shs_map_flush_queue;
	hrtimer_init(&rmnet_shs_cfg.hrtimer_lpm,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rmnet_shs_cfg.hrtimer_lpm.function = rmnet_shs_lpm_ring;

	hrtimer_init(&rmnet_shs_cfg.hrtimer_wake,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rmnet_shs_cfg.hrtimer_wake.function = rmnet_shs_wake_ring;

	hrtimer_init(&rmnet_shs_cfg.hrtimer_disable_pb_boost,
		     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	rmnet_shs_cfg.hrtimer_disable_pb_boost.function = rmnet_shs_pb_unboost_cpus;

	INIT_WORK(&shs_rx_work.work, rmnet_flush_buffered);
}

unsigned int rmnet_shs_rx_wq_exit(void)
{
	unsigned int cpu_switch = rmnet_shs_inst_rate_switch;
	int i;

	/* Disable any further core_flush timer starts untill cleanup
	 * is complete.
	 */
	rmnet_shs_inst_rate_switch = 0;

	for (i = 0; i < MAX_CPUS; i++) {
		hrtimer_cancel(&GET_CTIMER(i));

		cancel_work_sync(&rmnet_shs_cfg.core_flush[i].work);
	}

	cancel_work_sync(&shs_rx_work.work);

	return cpu_switch;
}

int rmnet_shs_drop_backlog(struct sk_buff_head *list, int cpu)
{
	struct sk_buff *skb;
	struct softnet_data *sd = &per_cpu(softnet_data, cpu);

	rtnl_lock();
	while ((skb = skb_dequeue_tail(list)) != NULL) {
		if (rmnet_is_real_dev_registered(skb->dev)) {
			rmnet_shs_crit_err[RMNET_SHS_OUT_OF_MEM_ERR]++;
			/* Increment sd and netdev drop stats*/
			dev_core_stats_rx_dropped_inc(skb->dev);
#if (KERNEL_VERSION(6, 9, 0) <= LINUX_VERSION_CODE)
			rps_input_queue_head_incr(sd);
			atomic_inc(&sd->dropped);
#else
			input_queue_head_incr(sd);
			sd->dropped++;
#endif
			kfree_skb(skb);
		}
	}
	rtnl_unlock();

	return 0;
}

/* This will run in process context, avoid disabling bh */
static int rmnet_shs_oom_notify(struct notifier_block *self,
			    unsigned long emtpy, void *free)
{
	int input_qlen, process_qlen, cpu;
	int *nfree = (int*)free;
	struct sk_buff_head *process_q;
	struct sk_buff_head *input_q;

	for_each_possible_cpu(cpu) {

		process_q = &GET_PQUEUE(cpu);
		input_q = &GET_IQUEUE(cpu);
		input_qlen = skb_queue_len(process_q);
		process_qlen = skb_queue_len(input_q);

		if (rmnet_oom_pkt_limit &&
		    (input_qlen + process_qlen) >= rmnet_oom_pkt_limit) {
			rmnet_shs_drop_backlog(&per_cpu(softnet_data,
							cpu).input_pkt_queue, cpu);
			input_qlen = skb_queue_len(process_q);
			process_qlen = skb_queue_len(input_q);
			if (process_qlen >= rmnet_oom_pkt_limit) {
				rmnet_shs_drop_backlog(process_q, cpu);
			}
			/* Let oom_killer know memory was freed */
			(*nfree)++;
		}
	}
	return 0;
}

static struct notifier_block rmnet_oom_nb = {
	.notifier_call = rmnet_shs_oom_notify,
};

void rmnet_shs_pb_hdr_handler(struct rmnet_map_pb_ind_hdr *pbhdr)
{
	struct hrtimer *pb_bd_timer;

	SHS_TRACE_LOW(RMNET_SHS_PB_BOOST_CPU, RMNET_SHS_PB_BOOST_CPU_ENTER,
			    pbhdr->le.seq_num, pbhdr->le.start_end_seq_num,
			    pbhdr->le.row_bytes_pending, pbhdr->le.fc_bytes_pending, NULL,
				NULL);

	/* Not yet making use of the contents of the PB qmap command */
	rmnet_shs_pb_boost_cpus();
	pb_bd_timer = &rmnet_shs_cfg.hrtimer_disable_pb_boost;
	if (hrtimer_active(pb_bd_timer))
		hrtimer_cancel(pb_bd_timer);

	hrtimer_start(pb_bd_timer,
				  ns_to_ktime(rmnet_shs_pb_boost_timer_ms * NS_IN_MS),
				  HRTIMER_MODE_REL);

	return;
}

void rmnet_shs_dl_hdr_handler_v2(struct rmnet_map_dl_ind_hdr *dlhdr,
			      struct rmnet_map_control_command_header *qcmd)
{
	rmnet_shs_dl_hdr_handler(dlhdr);
}

void rmnet_shs_dl_hdr_handler(struct rmnet_map_dl_ind_hdr *dlhdr)
{

	SHS_TRACE_LOW(RMNET_SHS_DL_MRK, RMNET_SHS_DL_MRK_HDR_HDLR_START,
			    dlhdr->le.seq, dlhdr->le.pkts,
			    0xDEF, 0xDEF, NULL, NULL);

	if (!rmnet_shs_last_seq || rmnet_shs_last_seq <  dlhdr->le.seq)
		rmnet_shs_last_seq = dlhdr->le.seq;
	else {
		if (rmnet_shs_debug)
			pr_info("rmnet_shs: OFO dl seq %u before %u\n",
				rmnet_shs_last_seq, dlhdr->le.seq);

		rmnet_shs_last_seq = dlhdr->le.seq;
		rmnet_shs_crit_err[RMNET_SHS_DL_MKR_SEQ_OFO]++;
	}

	if (!spin_is_locked(&rmnet_shs_ht_splock)) {
		if (rmnet_shs_cfg.num_pkts_parked > 0 &&
		    rmnet_shs_cfg.dl_ind_state != RMNET_SHS_IND_COMPLETE) {

			rmnet_shs_flush_reason[RMNET_SHS_FLUSH_INV_DL_IND2]++;
			rmnet_shs_flush_table(0, RMNET_RX_CTXT);
		}
		rmnet_shs_cfg.dl_ind_state = RMNET_SHS_END_PENDING;
	}
}

/* Triggers flushing of all packets upon DL trailer
 * receiving a DL trailer marker
 */
void rmnet_shs_dl_trl_handler_v2(struct rmnet_map_dl_ind_trl *dltrl,
			      struct rmnet_map_control_command_header *qcmd)
{

	rmnet_shs_dl_trl_handler(dltrl);
}

void rmnet_shs_dl_trl_handler(struct rmnet_map_dl_ind_trl *dltrl)
{

	SHS_TRACE_HIGH(RMNET_SHS_DL_MRK,
			     RMNET_SHS_FLUSH_DL_MRK_TRLR_HDLR_START,
			     rmnet_shs_cfg.num_pkts_parked, 0,
			     dltrl->seq_le, 0xDEF, NULL, NULL);
	rmnet_shs_cfg.dl_ind_state = RMNET_SHS_IND_COMPLETE;
	if (rmnet_shs_cfg.num_pkts_parked > 0 &&
	    !spin_is_locked(&rmnet_shs_ht_splock)) {
		rmnet_shs_flush_reason[RMNET_SHS_FLUSH_RX_DL_TRAILER]++;
		rmnet_shs_flush_table(0, RMNET_RX_CTXT);
	}
}

void rmnet_shs_init(struct net_device *dev, struct net_device *vnd)
{
	struct rps_map *map;
	int rc;
	u8 num_cpu;
	u8 map_mask;
	u8 map_len;

	if (rmnet_shs_cfg.rmnet_shs_init_complete)
		return;
	map = rcu_dereference(vnd->_rx->rps_map);

	if (!map) {
		map_mask = 0;
		map_len = 0;
	} else {
		map_mask = rmnet_shs_mask_from_map(map);
		map_len = rmnet_shs_get_mask_len(map_mask);
	}

	rmnet_shs_cfg.port = rmnet_get_port(dev);
	rmnet_shs_cfg.map_mask = map_mask;
	rmnet_shs_cfg.map_len = map_len;
	rmnet_shs_cfg.ban_mask = 1 << DEF_PHY_CPU;
	rmnet_shs_cfg.phy_tcpu = DEF_PHY_CPU;
	rmnet_shs_cfg.phy_old_cpu = DEF_PHY_CPU;
	rmnet_shs_cfg.phy_acpu = DEF_PHY_CPU;

	rmnet_shs_cfg.perf_mask = 0x9C;
	rmnet_shs_cfg.non_perf_mask = 0x63;
	rmnet_shs_cfg.max_s_cores = 0x04;



	for (num_cpu = 0; num_cpu < MAX_CPUS; num_cpu++)
		INIT_LIST_HEAD(&rmnet_shs_cpu_node_tbl[num_cpu].node_list_id);

	rmnet_shs_freq_init();
	rmnet_shs_ll_init();

	rc = register_oom_notifier(&rmnet_oom_nb);
	if (rc < 0) {
		pr_info("Rmnet_shs_oom register failure\n");
	}
	pr_info("rmnet_shs init with %x\n", rmnet_shs_cfg.map_mask);

	rmnet_shs_cfg.ws = wakeup_source_register(NULL, "RMNET_SHS");
	rmnet_shs_cfg.rmnet_shs_init_complete = 1;
}

/* Invoked during SHS module exit to gracefully consume all
 * the skb's that are parked and that aren't delivered yet
 */
void rmnet_shs_cancel_table(void)
{
	struct hlist_node *tmp;
	struct rmnet_shs_skbn_s *node;
	struct sk_buff *tmpbuf;
	int bkt;
	struct sk_buff *buf;

	if (!rmnet_shs_cfg.num_pkts_parked)
		return;
	spin_lock_bh(&rmnet_shs_ht_splock);
	hash_for_each_safe(RMNET_SHS_HT, bkt, tmp, node, list) {
		for ((buf = node->skb_list.head); buf != NULL; buf = tmpbuf) {
			tmpbuf = buf->next;
			if (buf)
				consume_skb(buf);
		}
		node->skb_list.num_parked_skbs = 0;
		node->skb_list.num_parked_bytes = 0;
		node->skb_list.head = NULL;
		node->skb_list.tail = NULL;
	}
	rmnet_shs_cfg.num_bytes_parked = 0;
	rmnet_shs_cfg.num_pkts_parked = 0;
	rmnet_shs_cfg.is_pkt_parked = 0;
	rmnet_shs_cfg.force_flush_state = RMNET_SHS_FLUSH_DONE;

	spin_unlock_bh(&rmnet_shs_ht_splock);

}

/* Should only be called in shs_assing with ht_lock */
void rmnet_shs_start_phy_switch(struct rmnet_shs_skbn_s * node_p,
				struct rmnet_shs_clnt_s *clnt_cfg, struct net_device *dev)
{
	u32 temp;
	u32 temp2;

	/* Setting the new queuehead as the qtail of new cpu + backlog*/
	node_p->phy = 1;
	node_p->custom_map = clnt_cfg->map_mask;
	node_p->custom_len = rmnet_shs_get_mask_len(node_p->custom_map);
	/* CPU shouldn't be in a banned cpu */
	node_p->map_cpu = rmnet_shs_cfg.phy_acpu;
	node_p->map_index = node_p->map_cpu;
	node_p->qhead_offset = 0;

	/* Boost silver cpus if on silver and 1 gold cpu. CTIMER will reset both.*/
	if (rmnet_shs_is_lpwr_cpu( node_p->map_cpu ) &&
	    !rmnet_shs_is_lpwr_cpu( rmnet_shs_cfg.phy_tcpu)) {
		rmnet_shs_boost_gold_cpu(rmnet_shs_cfg.phy_tcpu);
		rmnet_shs_boost_cpus();
	}

	rmnet_shs_phy_switch(dev, rmnet_shs_cfg.phy_tcpu, 1);
	/* Old cpu's backlog + new cpus backlog*/
	temp = (GET_PQUEUE(node_p->map_cpu).qlen + GET_IQUEUE(node_p->map_cpu).qlen);
	if (temp > rmnet_shs_max_phy_backlog)
		rmnet_shs_max_phy_backlog = temp;

	node_p->queue_head = rmnet_shs_get_cpu_qtail(rmnet_shs_cfg.phy_tcpu) + temp;
	temp2 = (GET_PQUEUE(rmnet_shs_cfg.phy_tcpu).qlen + GET_IQUEUE(rmnet_shs_cfg.phy_tcpu).qlen);

	/*If we are on silver now use max time but if we are currently on gold we don't
	 * need 60ms to move out of it
	 */
	if (rmnet_shs_is_lpwr_cpu(node_p->map_cpu)) {
		if (!hrtimer_active(&GET_CTIMER(node_p->map_cpu))) {
			hrtimer_start(&GET_CTIMER(node_p->map_cpu),
				      ns_to_ktime((rmnet_shs_max_qmap_wait) * NS_IN_MS),
				      HRTIMER_MODE_REL);
		}
	/* Moving from Gold to silver in a idle scenario*/
	} else if ((temp + temp2) < EZ_SWITCH) {
		if (!hrtimer_active(&GET_CTIMER(node_p->map_cpu))) {
			hrtimer_start(&GET_CTIMER(node_p->map_cpu),
				      ns_to_ktime((rmnet_shs_max_qmap_wait / 20) * NS_IN_MS),
				      HRTIMER_MODE_REL);
		}
	} else {
		if (!hrtimer_active(&GET_CTIMER(node_p->map_cpu))) {
			hrtimer_start(&GET_CTIMER(node_p->map_cpu),
				      ns_to_ktime((rmnet_shs_max_qmap_wait / 2) * NS_IN_MS),
				      HRTIMER_MODE_REL);
		}
	}
}

/* Keeps track of all active flows. Packets reaching SHS are parked in order
 * per flow and then delivered to the next layer upon hitting any of the
 * flushing triggers.
 *
 * Whenever a new hash is observed, cores are chosen round robin so that
 * back to back new flows do not getting assigned to the same core
 */
int rmnet_shs_assign(struct sk_buff *skb, struct rmnet_shs_clnt_s *clnt_cfg)
{
	struct rmnet_shs_skbn_s *node_p;
	struct hlist_node *tmp;
	int map = rmnet_shs_cfg.map_mask;
	int new_cpu;
	int map_cpu;
	u32 cpu_map_index, hash;
	u8 is_match_found = 0;
	u8 is_shs_reqd = 0;
	u8 phy_flush = 0;
	struct rmnet_shs_cpu_node_s *cpu_node_tbl_p;
	struct rmnet_priv *priv;
	u8 boost_rel = 0;
	struct rmnet_shs_msg_resp boost_msg;
	int cluster_mask = 0;
	u8 refresh = 0;
	/*deliver non TCP/UDP packets right away*/
	/* If stmp all is set break and don't check reqd */
	if (!(clnt_cfg->config & RMNET_SHS_STMP_ALL) &&
	    !rmnet_shs_is_skb_stamping_reqd(skb, &boost_rel)) {

		if (boost_rel) {
			/* If we are here we just saw a ICMP packet.*/
			rmnet_shs_deliver_skb(skb);
			rmnet_shs_create_ping_boost_msg_resp(PERF_DISABLE, &boost_msg);
			rmnet_shs_genl_msg_direct_send_to_userspace(&boost_msg);
		}
		else {
			rmnet_shs_deliver_skb(skb);
		}
		return 0;
	}

	if ((unlikely(!map)) || !rmnet_shs_cfg.rmnet_shs_init_complete) {
		rmnet_shs_deliver_skb(skb);
		SHS_TRACE_ERR(RMNET_SHS_ASSIGN,
				    RMNET_SHS_ASSIGN_CRIT_ERROR_NO_SHS_REQD,
				    0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
		rmnet_shs_crit_err[RMNET_SHS_MAIN_SHS_RPS_INIT_ERR]++;
		return 0;
	}

	SHS_TRACE_HIGH(RMNET_SHS_ASSIGN, RMNET_SHS_ASSIGN_START,
			     0xDEF, 0xDEF, 0xDEF, 0xDEF, skb, NULL);

	hash = skb_get_hash(skb);
	/* Phy flows are differentiated by map_mask being non-null.
	 * Better option would be to have a specific field.
	 */
	if (clnt_cfg->map_mask) {
		/*All phy flows will have same shs hash to avoid weird 2 phy core cases */
		skb->hash = 0xDEAD;
		hash = 0xDEAD;
		/* If we are a phy packet and we are going under a phy switch just deliver */
		/* Old cpu packets should only go here if we are here on new cpu it's an error */
		if (raw_smp_processor_id()  ==  rmnet_shs_cfg.phy_old_cpu &&
		    rmnet_shs_cfg.phy_old_cpu !=  rmnet_shs_cfg.phy_acpu) {
			netif_rx(skb);
			rmnet_shs_cfg.max_phy_steer++;
			if (rmnet_shs_cfg.max_phy_steer > rmnet_shs_max_qmap_steer) {
				/* This should flush all the phy cpus and reset the rcu hook*/
				rmnet_shs_flush_table(RMNET_SHS_FF_BAD_RPS, RMNET_RX_CTXT);
				rmnet_shs_switch_disable();
				rmnet_shs_crit_err[RMNET_SHS_PHY_LONG_STEER]++;
				rmnet_shs_cfg.max_phy_steer = 0;
			}

			return 0;
		}
	}
	/*  Using do while to spin lock and unlock only once */
	spin_lock_bh(&rmnet_shs_ht_splock);
	do {
		hash_for_each_possible_safe(RMNET_SHS_HT, node_p, tmp, list,
					    hash) {

			if (hash != node_p->hash)
				continue;

			SHS_TRACE_LOW(RMNET_SHS_ASSIGN,
				RMNET_SHS_ASSIGN_MATCH_FLOW_COMPLETE,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, skb, NULL);

			cpu_map_index = node_p->map_index;

			is_match_found = 1;
			is_shs_reqd = 1;
			/* If flow is marked as a LL flow investigate if fastpath is nessecary */
			/* If flow is not coming in LL irq path then this flow will be */
			if (node_p->low_latency) {
				if (node_p->low_latency == RMNET_SHS_LOW_LATENCY_CHECK) {
						if (rmnet_shs_is_filter_match(skb)) {
							node_p->low_latency = RMNET_SHS_LOW_LATENCY_MATCH;
						} else {
							node_p->low_latency = RMNET_SHS_NOT_LOW_LATENCY;
						}
				}
				spin_unlock_bh(&rmnet_shs_ht_splock);
				/* Does not take coalescing so inaccurate but LL cares about speed */
				node_p->num_skb += 1;
				node_p->num_skb_bytes += skb->len;
				rmnet_shs_ll_handler(skb, clnt_cfg);
				return 0;
			}

			if (node_p->phy) {
				rmnet_shs_update_cpu_proc_q_all_cpus();

				if (raw_smp_processor_id() != rmnet_shs_cfg.phy_acpu) {
					rmnet_shs_crit_err[RMNET_SHS_PHY_INVALID_STATE4]++;
				}
				/* Un accounted gold packet in initial qtail calculations, add as offset.*/
				/* If we are being told to move to new cpu */
				if (rmnet_shs_cfg.phy_tcpu != rmnet_shs_cfg.phy_acpu) {
				/* Later on we expect node to be in cputbl of
				 * node->map, phy switch below will reset
				 * node->map cpu so we should keep it
				 * consistent
				 */
					if (node_p->map_cpu != rmnet_shs_cfg.phy_acpu) {
						cpu_node_tbl_p = &rmnet_shs_cpu_node_tbl[rmnet_shs_cfg.phy_acpu];
						rmnet_shs_cpu_node_move(node_p, &cpu_node_tbl_p->node_list_id,node_p->map_cpu);
						rmnet_shs_switch_reason[RMNET_SHS_PHY_NO_INTERL_QMAP_FF]++;
					}

					rmnet_shs_start_phy_switch(node_p, clnt_cfg, skb->dev);
					netif_rx(skb);
					break;
				}
				node_p->qhead_offset++;
				rmnet_shs_chain_to_skb_list(skb, node_p, clnt_cfg);
				if (!rmnet_module_hook_is_set(RMNET_MODULE_HOOK_SHS_SWITCH)) {
					rmnet_shs_crit_err[RMNET_SHS_PHY_INVALID_STATE2]++;
					phy_flush = RMNET_SHS_FF_PHY_INVALID;
					break;
				}

				/* Reg flush if all silver packets have been
				 * processed as newcpu's qhead has exceeded
				 * last oldcpu's skb qtail @enqtime.
				 */
				if (rmnet_shs_get_cpu_qhead(rmnet_shs_cfg.phy_acpu) >=
				    (node_p->queue_head + node_p->qhead_offset)) {
					phy_flush = RMNET_SHS_FF_PHY_REG;
					break;
				}
				if (rmnet_shs_max_qmap_pkt &&
				    node_p->skb_list.num_parked_skbs > rmnet_shs_max_qmap_pkt) {
					rmnet_shs_flush_reason[RMNET_SHS_FLUSH_PHY_PKT_LIMIT]++;
					phy_flush = RMNET_SHS_FF_PHY_PKT_LIMIT_ETC;
					break;
				}

				spin_unlock_bh(&rmnet_shs_ht_splock);
				return 0;
			}
			else
				rmnet_shs_chain_to_skb_list(skb, node_p, clnt_cfg);

			break;

		}
		if (is_match_found)
			break;

		/* Non-Null mapmask is used as a phy indicator */
		/* We haven't found a hash match upto this point */

		/* If no non-reserved/non banned CPUs move up to gold */
		cluster_mask =  (NONPERF_MASK & ~rmnet_shs_cfg.ban_mask & ~rmnet_shs_halt_mask & rmnet_shs_cfg.map_mask) ?
				NONPERF_MASK : PERF_MASK;

		if (clnt_cfg->map_mask) {
			new_cpu = rmnet_shs_wq_get_least_utilized_core(rmnet_shs_cfg.map_mask &
								       cluster_mask &
								       ~rmnet_shs_cfg.ban_mask &
								       ~rmnet_shs_halt_mask) ;
		} else {
			new_cpu = rmnet_shs_get_core_prio_flow(rmnet_shs_cfg.map_mask &
							       cluster_mask &
							       ~rmnet_shs_cfg.ban_mask &
							       ~rmnet_shs_halt_mask);
		}

		if (new_cpu < 0) {


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
		node_p->dev = skb->dev;
		node_p->hash = skb->hash;
		node_p->map_cpu = new_cpu;
		node_p->map_index = rmnet_shs_idx_from_cpu(node_p->map_cpu,
							   map);
		INIT_LIST_HEAD(&node_p->node_id);
		/* Set ip header / transport header / transport proto */
		rmnet_shs_get_update_skb_hdr_info(skb, node_p);

		/* Workqueue utilizes some of the values from above
		 * initializations . Therefore, we need to request
		 * for memory (to workqueue) after the above initializations
		 */
		rmnet_shs_wq_create_new_flow(node_p);
		if (!node_p->hstats) {
			/* hstat malloc failure, don't add flow to hashtable */
		        atomic_long_dec(&rmnet_shs_cfg.num_flows);
			kfree(node_p);
			break;
		}
		if (clnt_cfg->map_mask) {
			rmnet_shs_update_cpu_proc_q_all_cpus();
			rmnet_shs_start_phy_switch(node_p, clnt_cfg, skb->dev);

		}  else {
			node_p->custom_map = rmnet_shs_cfg.map_mask;
			node_p->custom_len = rmnet_shs_cfg.map_len;
		}
		map_cpu = node_p->map_cpu;
		cpu_node_tbl_p = &rmnet_shs_cpu_node_tbl[map_cpu];

		/* Set mux id */
		priv = netdev_priv(node_p->dev);
		if (!priv) {
			rm_err("priv for netdev is null for hash 0x%x",
			       node_p->hash);
			rmnet_shs_crit_err[RMNET_SHS_NETDEV_ERR]++;
		} else {
			node_p->hstats->mux_id = priv->mux_id;
			rm_err("SHS_MUX: mux id for hash 0x%x is %d",
			       node_p->hash, node_p->hstats->mux_id);
		}
		rmnet_shs_cpu_node_add(node_p, &cpu_node_tbl_p->node_list_id);
		hash_add_rcu(RMNET_SHS_HT, &node_p->list, skb->hash);
		/* If flow is designated as a LL flow then we mark it as such.
		 * Later on we will fastpath flow's skbs.
		 */
		if (rmnet_shs_is_filter_match(skb)) {
			node_p->low_latency = RMNET_SHS_LOW_LATENCY_MATCH;
			spin_unlock_bh(&rmnet_shs_ht_splock);
			rmnet_shs_ll_handler(skb, clnt_cfg);
			return 0;
		}
		/* Chain this pkt to skb list (most likely to skb_list.head)
		 * because this is the first packet for this flow
		 */
		if (!node_p->phy)
			rmnet_shs_chain_to_skb_list(skb, node_p, clnt_cfg);
		else  {
			netif_rx(skb);
			spin_unlock_bh(&rmnet_shs_ht_splock);
			return 0;
		}

		is_shs_reqd = 1;
		break;

	} while (0);

	if (!is_shs_reqd) {
		spin_unlock_bh(&rmnet_shs_ht_splock);
		rmnet_shs_crit_err[RMNET_SHS_MAIN_SHS_NOT_REQD]++;
		rmnet_shs_deliver_skb(skb);
		SHS_TRACE_ERR(RMNET_SHS_ASSIGN,
				    RMNET_SHS_ASSIGN_CRIT_ERROR_NO_SHS_REQD,
				    0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
		return 0;
	}

	/* We got the first packet after a previous successdul flush. Arm the
	 * flushing timer.
	 */
	if (!rmnet_shs_cfg.is_pkt_parked && rmnet_shs_cfg.num_pkts_parked &&
	    rmnet_shs_fall_back_timer) {
		rmnet_shs_cfg.is_pkt_parked = 1;
		rmnet_shs_cfg.force_flush_state = RMNET_SHS_FLUSH_OFF;
		refresh = 1;
	}
	spin_unlock_bh(&rmnet_shs_ht_splock);

	if (refresh) {
		if (hrtimer_active(&rmnet_shs_cfg.hrtimer_shs)) {
			SHS_TRACE_LOW(RMNET_SHS_ASSIGN,
				      RMNET_SHS_ASSIGN_PARK_TMR_CANCEL,
				      RMNET_SHS_FORCE_FLUSH_TIME_NSEC,
				      0xDEF, 0xDEF, 0xDEF, skb, NULL);
			hrtimer_cancel(&rmnet_shs_cfg.hrtimer_shs);
		}
		hrtimer_start(&rmnet_shs_cfg.hrtimer_shs,
			      ns_to_ktime(rmnet_shs_timeout * NS_IN_MS),
					  HRTIMER_MODE_REL);
		SHS_TRACE_LOW(RMNET_SHS_ASSIGN,
			      RMNET_SHS_ASSIGN_PARK_TMR_START,
			      RMNET_SHS_FORCE_FLUSH_TIME_NSEC,
			      0xDEF, 0xDEF, 0xDEF, skb, NULL);
	}

	if (rmnet_shs_cfg.num_pkts_parked >
	    rmnet_shs_pkts_store_limit) {

		if (rmnet_shs_stats_enabled)
			rmnet_shs_flush_reason[RMNET_SHS_FLUSH_PKT_LIMIT]++;

		SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
				     RMNET_SHS_FLUSH_PKT_LIMIT_TRIGGER, 0,
				     0xDEF, 0xDEF, 0xDEF, NULL, NULL);
		rmnet_shs_flush_table(RMNET_SHS_FF_PKT_LIMIT, RMNET_RX_CTXT);

	} else if (rmnet_shs_cfg.num_bytes_parked >
		   rmnet_shs_byte_store_limit) {

		if (rmnet_shs_stats_enabled)
			rmnet_shs_flush_reason[RMNET_SHS_FLUSH_BYTE_LIMIT]++;
		SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
				     RMNET_SHS_FLUSH_BYTE_LIMIT_TRIGGER, 0,
				     0xDEF, 0xDEF, 0xDEF, NULL, NULL);
		rmnet_shs_flush_table(RMNET_SHS_FF_BYTE_LIMIT, RMNET_RX_CTXT);

	} else if (phy_flush) {
		/* Phy was not able to be flushed and moved on time, force flush now */
		SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
			       RMNET_SHS_FLUSH_PKT_LIMIT_TRIGGER, 1,
			       rmnet_shs_cfg.num_pkts_parked, 0xDEF, 0xDEF,
			       NULL, NULL);
		if (rmnet_shs_stats_enabled)
			rmnet_shs_flush_reason[RMNET_SHS_FLUSH_PHY_FF_FLUSH]++;

		rmnet_shs_flush_table(phy_flush, RMNET_RX_CTXT);

	} else if ((clnt_cfg->config & RMNET_SHS_NO_DLMKR) &&
		   rmnet_shs_cfg.num_pkts_parked) {
		if (rmnet_shs_stats_enabled)
			rmnet_shs_flush_reason[RMNET_SHS_FLUSH_PHY_FLUSH]++;
		rmnet_shs_flush_table(0, RMNET_RX_CTXT);
	}

	/* Flushing timer that was armed previously has successfully fired.
	 * Now we trigger force flushing of all packets. If a flow is waiting
	 * to switch to another core, it will be forcefully moved during this
	 * trigger.
	 *
	 * In case the previously delivered packets haven't been processed by
	 * the next layers, the parked packets may be delivered out of order
	 * until all the previously delivered packets have been processed
	 * successully
	 */
	else if (rmnet_shs_cfg.force_flush_state == RMNET_SHS_FLUSH_ON) {
		rmnet_shs_flush_reason[RMNET_SHS_FLUSH_TIMER_EXPIRY]++;
		SHS_TRACE_HIGH(RMNET_SHS_FLUSH,
				     RMNET_SHS_FLUSH_FORCE_TRIGGER, 1,
				     rmnet_shs_cfg.num_pkts_parked,
				     0xDEF, 0xDEF, NULL, NULL);
		rmnet_shs_flush_table(0, RMNET_RX_CTXT);

	} else if (!(clnt_cfg->config & RMNET_SHS_NO_DLMKR) &&
		   rmnet_shs_cfg.num_pkts_parked &&
		   rmnet_shs_cfg.dl_ind_state != RMNET_SHS_END_PENDING) {

		rmnet_shs_flush_reason[RMNET_SHS_FLUSH_INV_DL_IND]++;
		rmnet_shs_flush_table(0, RMNET_RX_CTXT);
	}
	return 0;
}

static const struct rmnet_module_hook_register_info
rmnet_shs_skb_entry_hook = {
	.hooknum = RMNET_MODULE_HOOK_SHS_SKB_ENTRY,
	.func = rmnet_shs_assign,
};

void rmnet_shs_skb_entry_disable(void)
{
	rmnet_module_hook_unregister_no_sync(&rmnet_shs_skb_entry_hook, 1);
}

void rmnet_shs_skb_entry_enable(void)
{
	rmnet_module_hook_register(&rmnet_shs_skb_entry_hook, 1);
}

static int rmnet_shs_switch_hook_entry(struct sk_buff *skb,
				       struct rmnet_shs_clnt_s *cfg)
{
	struct rmnet_skb_cb *cb = RMNET_SKB_CB(skb);

	if (!cb->qmap_steer && skb->priority != 0xda1a) {
		cb->qmap_steer = 1;
		rmnet_shs_assign(skb, cfg);
		return 1;
	}

	return 0;
}

static const struct rmnet_module_hook_register_info
rmnet_shs_switch_hook = {
	.hooknum = RMNET_MODULE_HOOK_SHS_SWITCH,
	.func = rmnet_shs_switch_hook_entry,
};

void rmnet_shs_switch_disable(void)
{
	rmnet_module_hook_unregister_no_sync(&rmnet_shs_switch_hook, 1);
}

void rmnet_shs_switch_enable(void)
{
	rmnet_module_hook_register(&rmnet_shs_switch_hook, 1);
}


/* Cancels the flushing timer if it has been armed
 * Deregisters DL marker indications
 */
void rmnet_shs_exit(unsigned int cpu_switch)
{
	rmnet_shs_freq_exit();
	rmnet_shs_ll_deinit();
	rmnet_shs_cfg.is_reg_dl_mrk_ind = 0;
	unregister_oom_notifier(&rmnet_oom_nb);

	hrtimer_cancel(&rmnet_shs_cfg.hrtimer_shs);
	hrtimer_cancel(&rmnet_shs_cfg.hrtimer_lpm);
	hrtimer_cancel(&rmnet_shs_cfg.hrtimer_wake);
	wakeup_source_unregister(rmnet_shs_cfg.ws);

	memset(&rmnet_shs_cfg, 0, sizeof(rmnet_shs_cfg));
	rmnet_shs_cfg.port = NULL;
	rmnet_shs_cfg.rmnet_shs_init_complete = 0;
	rmnet_shs_inst_rate_switch = cpu_switch;
}
