// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "rmnet_shs.h"
#include "rmnet_shs_wq_genl.h"
#include "rmnet_shs_wq_mem.h"
#include <linux/workqueue.h>
#include <linux/list_sort.h>
#include <net/sock.h>
#include <linux/skbuff.h>
#if IS_ENABLED(CONFIG_SCHED_WALT)
#include <linux/sched/walt.h>
#endif /* CONFIG_SCHED_WALT */
#include "rmnet_shs_modules.h"
#include "rmnet_shs_common.h"
#include <linux/pm_wakeup.h>
#include "rmnet_module.h"
#if (KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE)
#include <net/netdev_rx_queue.h>
#endif

MODULE_LICENSE("GPL v2");
/* Local Macros */
#define RMNET_SHS_FILTER_PKT_LIMIT 200
#define RMNET_SHS_FILTER_FLOW_RATE 100

#define PERIODIC_CLEAN 0
/* FORCE_CLEAN should only used during module de-init.*/
#define FORCE_CLEAN 1
#define MAX_RESERVE_CPU 2
#define TITANIUM_CPU1 5
#define TITANIUM_CPU2 6

#define SYNC_TIME 0x7F
#define ASYNC_LOWTHRESH 15000
#define ASYNC_UPTHRESH 34000
#define BIT_TEST(X,Y) ((1<<Y) & X)
/* Local Definitions and Declarations */
#define PRIO_BACKOFF ((!rmnet_shs_cpu_prio_dur) ? 2 : rmnet_shs_cpu_prio_dur)

#define RMNET_SHS_SEGS_PER_SKB_DEFAULT (2)

DEFINE_SPINLOCK(rmnet_shs_hstat_tbl_lock);
DEFINE_SPINLOCK(rmnet_shs_ep_lock);

static ktime_t rmnet_shs_wq_tnsec;
struct workqueue_struct *rmnet_shs_wq;
static struct rmnet_shs_delay_wq_s *rmnet_shs_delayed_wq;
static struct rmnet_shs_wq_rx_flow_s rmnet_shs_rx_flow_tbl;
struct list_head rmnet_shs_ll_hstat_tbl =
				LIST_HEAD_INIT(rmnet_shs_ll_hstat_tbl);

struct list_head rmnet_shs_wq_hstat_tbl =
				LIST_HEAD_INIT(rmnet_shs_wq_hstat_tbl);
static int rmnet_shs_flow_dbg_stats_idx_cnt;
struct list_head rmnet_shs_wq_ep_tbl = LIST_HEAD_INIT(rmnet_shs_wq_ep_tbl);

static int is_reserved(int cpu)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	cpumask_t rmask = walt_get_cpus_taken();
	return cpumask_test_cpu(cpu, &rmask);
#else
	return 0;
#endif /* CONFIG_SCHED_WALT */
}

static void rmnet_shs_phy_sync(void)
{
	struct rmnet_shs_msg_resp chg_msg;

	rmnet_shs_create_phy_msg_resp(&chg_msg, rmnet_shs_cfg.phy_acpu, rmnet_shs_cfg.phy_acpu);
	rmnet_shs_genl_msg_direct_send_to_userspace(&chg_msg);
}

static void rmnet_shs_get_state(void)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	cpumask_t rmask;
	int j;
	unsigned int dest_mask = 0;

	/* Feature needs to be enabled on target for support*/
	if (!(rmnet_shs_cfg.feature_mask & TITANIUM_FEAT))
		return;

	walt_get_cpus_in_state1(&rmask);


	for (j = 0; j < MAX_CPUS; j++) {
			if (cpumask_test_cpu(j, &rmask)) {
			dest_mask |= 1 << j;
		}
	}

	if ((dest_mask & (1 << TITANIUM_CPU1)) &&
	    (dest_mask & (1 << TITANIUM_CPU2)))
	{
		if (rmnet_shs_cfg.max_s_cores != 4) {
			rmnet_shs_cfg.max_s_cores = 4;
			rmnet_shs_cfg.perf_mask = 0x9C;
			rmnet_shs_cfg.non_perf_mask = 0x63;
			rmnet_shs_cpu_rx_min_pps_thresh[TITANIUM_CPU1] = RMNET_SHS_UDP_PPS_LPWR_CPU_LTHRESH;
			rmnet_shs_cpu_rx_min_pps_thresh[TITANIUM_CPU2] = RMNET_SHS_UDP_PPS_LPWR_CPU_LTHRESH;

			rmnet_shs_cpu_rx_max_pps_thresh[TITANIUM_CPU1] = RMNET_SHS_UDP_PPS_LPWR_CPU_UTHRESH;
			rmnet_shs_cpu_rx_max_pps_thresh[TITANIUM_CPU2] = RMNET_SHS_UDP_PPS_LPWR_CPU_UTHRESH;
			trace_rmnet_shs_wq_low(RMNET_SHS_WALT, RMNET_SHS_WALT_TRANSITION,
						rmnet_shs_cfg.perf_mask, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
			rmnet_shs_switch_reason[RMNET_SHS_WALT_SWITCH1]++;
		}
	} else {
		if (rmnet_shs_cfg.max_s_cores != 2) {
			rmnet_shs_cfg.max_s_cores = 2;
			rmnet_shs_cfg.perf_mask = 0xFC;
			rmnet_shs_cfg.non_perf_mask = 0x03;

			rmnet_shs_cpu_rx_min_pps_thresh[TITANIUM_CPU1] = RMNET_SHS_UDP_PPS_PERF_CPU_LTHRESH;
			rmnet_shs_cpu_rx_min_pps_thresh[TITANIUM_CPU2] = RMNET_SHS_UDP_PPS_PERF_CPU_LTHRESH;

			rmnet_shs_cpu_rx_max_pps_thresh[TITANIUM_CPU1] = RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH;
			rmnet_shs_cpu_rx_max_pps_thresh[TITANIUM_CPU2] = RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH;
			trace_rmnet_shs_wq_low(RMNET_SHS_WALT, RMNET_SHS_WALT_TRANSITION,
						rmnet_shs_cfg.perf_mask, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
			rmnet_shs_switch_reason[RMNET_SHS_WALT_SWITCH2]++;
		}
	}
#endif /* CONFIG_SCHED_WALT */
}

static void rmnet_update_reserve_mask(void)
{
#if IS_ENABLED(CONFIG_SCHED_WALT)
	int j = 0, res_cpus = 0, update_mask = 0, new_cpu = 0;
	int new_phy = 1, aud_res_mask = 0, cluster;
	int old_mask = rmnet_shs_halt_mask;
	cpumask_t rmask = walt_get_cpus_taken();
	cpumask_t halt_mask = walt_get_halted_cpus();

	if (!rmnet_shs_reserve_on)
		return;

	for (j = 0; j < MAX_CPUS; j++) {
		if (BIT_TEST(rmnet_shs_cfg.map_mask, j) && cpumask_test_cpu(j, &halt_mask)) {
			res_cpus++;
			if (res_cpus > MAX_RESERVE_CPU) {
				rmnet_shs_crit_err[RMNET_SHS_RESERVE_LIMIT]++;
				break;
			}
			update_mask |= 1<< j;
		}
	}

	for (j = 0; j < MAX_CPUS && res_cpus < MAX_RESERVE_CPU; j++) {
		if (BIT_TEST(rmnet_shs_cfg.map_mask, j) && cpumask_test_cpu(j, &rmask)) {
			res_cpus++;
			if (res_cpus > MAX_RESERVE_CPU) {
				rmnet_shs_crit_err[RMNET_SHS_RESERVE_LIMIT]++;
				break;
			}
			aud_res_mask |= 1<< j;
			update_mask |= 1<< j;
		}
	}
	/* Halt mask is halted cpus + audio reserved cpus we are honoring up to Max reserve cpus we allow
	 * reserve_mask is a subset of halted mask that shows which audio cpus are honored.
	 */
	if (old_mask != update_mask)
		rmnet_shs_switch_reason[RMNET_SHS_HALT_MASK_CHANGE]++;

	rmnet_shs_reserve_mask = aud_res_mask;
	rmnet_shs_halt_mask = update_mask;

	/* Move to Perf core if silver phy is taken, move to another Perf core if perf phy is taken */
	if ((1 << rmnet_shs_cfg.phy_acpu) & rmnet_shs_halt_mask) {
		cluster = (rmnet_shs_cfg.feature_mask & PHY_GOLD_SWITCH_FEAT)? PERF_MASK : NONPERF_MASK;
		new_cpu = rmnet_shs_wq_get_least_utilized_core(rmnet_shs_cfg.map_mask & cluster &
							       ~rmnet_shs_cfg.ban_mask & ~rmnet_shs_halt_mask);
		if (new_cpu > 0) {
			new_phy = new_cpu;
			rmnet_shs_cfg.phy_tcpu = new_phy;
			rmnet_shs_switch_reason[RMNET_SHS_HALT_PHY]++;
			rmnet_shs_switch_enable();
		}
	}
#endif /* CONFIG_SCHED_WALT */
}

int rmnet_shs_cpu_psb_above_thresh(unsigned cpu_num, unsigned thresh)
{
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_node;
	struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl;

	if (cpu_num >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return 0;
	}

	cpu_node = &rx_flow_tbl_p->cpu_list[cpu_num];

	return cpu_node->rx_pps > thresh;
}

/* Resets all the parameters used to maintain hash statistics */
void rmnet_shs_wq_hstat_reset_node(struct rmnet_shs_wq_hstat_s *hnode)
{
	hnode->c_epoch = 0;
	hnode->l_epoch = 0;
	hnode->node = NULL;
	hnode->inactive_duration = 0;
	hnode->rx_skb = 0;
	hnode->rx_coal_skb = 0;
	hnode->rx_bytes = 0;
	hnode->rx_pps = 0;
	hnode->rx_bps = 0;
	hnode->hw_coal_bytes_diff = 0;
	hnode->hw_coal_bufsize_diff = 0;
	hnode->last_hw_coal_bytes = 0;
	hnode->last_hw_coal_bufsize = 0;
	hnode->hw_coal_bytes = 0;
	hnode->hw_coal_bufsize = 0;
	hnode->last_rx_skb = 0;
	hnode->rx_ll_skb = 0;
	hnode->last_rx_ll_skb = 0;
	hnode->ll_diff = 0;
	hnode->last_rx_bytes = 0;
	hnode->rps_config_msk = 0;
	hnode->current_core_msk = 0;
	hnode->def_core_msk = 0;
	hnode->pri_core_msk = 0;
	hnode->available_core_msk = 0;
	hnode->hash = 0;
	hnode->suggested_cpu = 0;
	hnode->current_cpu = 0;
	hnode->segs_per_skb = 0;
	hnode->skb_tport_proto = 0;
	hnode->stat_idx = (-1);
	hnode->bif = 0;
	hnode->ack_thresh = 0;
	INIT_LIST_HEAD(&hnode->cpu_node_id);
	hnode->is_new_flow = 0;
	/* clear in use flag as a last action. This is required to ensure
	 * the same node does not get allocated until all the paramaeters
	 * are cleared.
	 */
	hnode->in_use = 0;
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_HSTAT_TBL,
			    RMNET_SHS_WQ_HSTAT_TBL_NODE_RESET,
			    hnode->is_perm, 0xDEF, 0xDEF, 0xDEF, hnode, NULL);
}

/* Preallocates a set of flow nodes that maintain flow level statistics*/
void rmnet_shs_wq_hstat_alloc_nodes(u8 num_nodes_to_allocate, u8 is_store_perm)
{
	struct rmnet_shs_wq_hstat_s *hnode = NULL;

	while (num_nodes_to_allocate > 0) {
		hnode = kzalloc(sizeof(*hnode), GFP_ATOMIC);
		if (hnode) {
			hnode->is_perm = is_store_perm;
			rmnet_shs_wq_hstat_reset_node(hnode);
			INIT_LIST_HEAD(&hnode->hstat_node_id);
			INIT_LIST_HEAD(&hnode->cpu_node_id);
			rmnet_shs_hstat_tbl_add(hnode);
		} else {
			rmnet_shs_crit_err[RMNET_SHS_WQ_ALLOC_HSTAT_ERR]++;
		}
		hnode = NULL;
		num_nodes_to_allocate--;
	}

}

/* If there is an already pre-allocated node available and not in use,
 * we will try to re-use them.
 */
struct rmnet_shs_wq_hstat_s *rmnet_shs_wq_get_new_hstat_node(void)
{
	struct rmnet_shs_wq_hstat_s *hnode = NULL;
	struct rmnet_shs_wq_hstat_s *ret_node = NULL;

	rcu_read_lock();
	spin_lock_bh(&rmnet_shs_hstat_tbl_lock);
	list_for_each_entry_rcu(hnode, &rmnet_shs_wq_hstat_tbl, hstat_node_id) {

		if (hnode->in_use == 0) {
			ret_node = hnode;
			ret_node->in_use = 1;
			ret_node->is_new_flow = 1;
			break;
		}
	}
	spin_unlock_bh(&rmnet_shs_hstat_tbl_lock);
	rcu_read_unlock();

	if (ret_node) {
		trace_rmnet_shs_wq_low(RMNET_SHS_WQ_HSTAT_TBL,
				    RMNET_SHS_WQ_HSTAT_TBL_NODE_REUSE,
				    hnode->is_perm, 0xDEF, 0xDEF, 0xDEF,
				    hnode, NULL);
		return ret_node;
	}

	/* We have reached a point where all pre-allocated nodes are in use
	 * Allocating memory to maintain the flow level stats for new flow.
	 * However, this newly allocated memory will be released as soon as we
	 * realize that this flow is inactive
	 */
	ret_node = kzalloc(sizeof(*hnode), GFP_ATOMIC);

	if (!ret_node) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_ALLOC_HSTAT_ERR]++;
		return NULL;
	}

	rmnet_shs_wq_hstat_reset_node(ret_node);
	ret_node->is_perm = 0;
	ret_node->in_use = 1;
	ret_node->is_new_flow = 1;
	INIT_LIST_HEAD(&ret_node->hstat_node_id);
	INIT_LIST_HEAD(&ret_node->cpu_node_id);

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_HSTAT_TBL,
			    RMNET_SHS_WQ_HSTAT_TBL_NODE_DYN_ALLOCATE,
			    ret_node->is_perm, 0xDEF, 0xDEF, 0xDEF,
			    ret_node, NULL);

	rmnet_shs_hstat_tbl_add(ret_node);

	return ret_node;
}

void rmnet_shs_wq_create_new_flow(struct rmnet_shs_skbn_s *node_p)
{
	struct timespec64 time;

	if (!node_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	node_p->hstats = rmnet_shs_wq_get_new_hstat_node();
	if (node_p->hstats != NULL) {
		(void)ktime_get_boottime_ts64(&time);

		node_p->hstats->hash = node_p->hash;
		node_p->hstats->skb_tport_proto = node_p->skb_tport_proto;
		node_p->hstats->current_cpu = node_p->map_cpu;
		node_p->hstats->suggested_cpu = node_p->map_cpu;
		/* Default 0 for segmentation */
		node_p->hstats->segs_per_skb = 0;

		/* Start TCP flows with segmentation if userspace connected */
		if (rmnet_shs_userspace_connected &&
		    node_p->hstats->skb_tport_proto == IPPROTO_TCP)
			node_p->hstats->segs_per_skb = RMNET_SHS_SEGS_PER_SKB_DEFAULT;

		node_p->hstats->node = node_p;
		node_p->hstats->c_epoch = RMNET_SHS_SEC_TO_NSEC(time.tv_sec) +
		   time.tv_nsec;
		node_p->hstats->l_epoch = RMNET_SHS_SEC_TO_NSEC(time.tv_sec) +
		   time.tv_nsec;
	}

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_HSTAT_TBL,
				RMNET_SHS_WQ_HSTAT_TBL_NODE_NEW_REQ,
				0xDEF, 0xDEF, 0xDEF, 0xDEF,
				node_p, node_p->hstats);
}

/* Compute the average pps for a flow based on tuning param
 * Often when we decide to switch from a small cluster core,
 * it is because of the heavy traffic on that core. In such
 * circumstances, we want to switch to a big cluster
 * core as soon as possible. Therefore, we will provide a
 * greater weightage to the most recent sample compared to
 * the previous samples.
 *
 * On the other hand, when a flow which is on a big cluster
 * cpu suddenly starts to receive low traffic we move to a
 * small cluster core after observing low traffic for some
 * more samples. This approach avoids switching back and forth
 * to small cluster cpus due to momentary decrease in data
 * traffic.
 */
static u64 rmnet_shs_wq_get_flow_avg_pps(struct rmnet_shs_wq_hstat_s *hnode)
{
	u64 avg_pps, mov_avg_pps;
	u16 new_weight, old_weight;

	if (!hnode) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return 0;
	}

	old_weight = rmnet_shs_wq_tuning;
	new_weight = 100 - rmnet_shs_wq_tuning;

	/* computing weighted average per flow, if the flow has just started,
	 * there is no past values, so we use the current pps as the avg
	 */
	if (hnode->last_pps == 0) {
		avg_pps = hnode->rx_pps;
	} else {
		mov_avg_pps = (hnode->last_pps + hnode->avg_pps) / 2;
		avg_pps = (((new_weight * hnode->rx_pps) +
			    (old_weight * mov_avg_pps)) /
			    (new_weight + old_weight));
	}

	return avg_pps;
}

static u64 rmnet_shs_wq_get_cpu_avg_pps(u16 cpu_num)
{
	u64 avg_pps, mov_avg_pps;
	u16 new_weight, old_weight;
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_node;
	struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl;

	if (cpu_num >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return 0;
	}

	cpu_node = &rx_flow_tbl_p->cpu_list[cpu_num];
	old_weight = rmnet_shs_wq_tuning;
	new_weight = 100 - rmnet_shs_wq_tuning;

	/* computing weighted average per flow, if the cpu has not past values
	 * for pps, we use the current value as the average
	 */
	if (cpu_node->last_rx_pps == 0) {
		avg_pps = cpu_node->avg_pps;
	} else {
		mov_avg_pps = (cpu_node->last_rx_pps + cpu_node->avg_pps) / 2;
		avg_pps = (((new_weight * cpu_node->rx_pps) +
			    (old_weight * mov_avg_pps)) /
			    (new_weight + old_weight));
	}

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_CPU_STATS,
			   RMNET_SHS_WQ_CPU_STATS_CORE2SWITCH_EVAL_CPU,
			   cpu_num, cpu_node->rx_pps, cpu_node->last_rx_pps,
			   avg_pps, NULL, NULL);

	return avg_pps;
}

/* Refresh the RPS mask associated with this flow */
void rmnet_shs_wq_update_hstat_rps_msk(struct rmnet_shs_wq_hstat_s *hstat_p)
{
	struct rmnet_shs_skbn_s *node_p = NULL;
	struct rmnet_shs_wq_ep_s *ep = NULL;

	if (!hstat_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	node_p = hstat_p->node;

	/*Map RPS mask from the endpoint associated with this flow*/
	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {
		if (node_p->dev == ep->ep) {
			hstat_p->rps_config_msk = ep->rps_config_msk;
			hstat_p->def_core_msk = ep->default_core_msk;
			hstat_p->pri_core_msk = ep->pri_core_msk;

			/* Update ep tput stats while we're here */
			if (hstat_p->skb_tport_proto == IPPROTO_TCP) {
				rm_err("SHS_TCP: adding TCP bps %llu to ep_total %llu ep name %s",
				       hstat_p->rx_bps, ep->tcp_rx_bps, node_p->dev->name);
				ep->tcp_rx_bps += hstat_p->rx_bps;
			} else if (hstat_p->skb_tport_proto == IPPROTO_UDP) {
				rm_err("SHS_UDP: adding UDP rx_bps %llu to ep_total %llu ep name %s",
				       hstat_p->rx_bps, ep->udp_rx_bps, node_p->dev->name);
				ep->udp_rx_bps += hstat_p->rx_bps;
			}
			break;
		}
	}
	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_FLOW_STATS,
				RMNET_SHS_WQ_FLOW_STATS_UPDATE_MSK,
				hstat_p->rps_config_msk,
				hstat_p->def_core_msk,
				hstat_p->pri_core_msk,
				0xDEF, hstat_p, node_p);
}

void rmnet_shs_wq_update_hash_stats_debug(struct rmnet_shs_wq_hstat_s *hstats_p,
					  struct rmnet_shs_skbn_s *node_p)
{
	int idx = rmnet_shs_flow_dbg_stats_idx_cnt;

	if (!rmnet_shs_stats_enabled)
		return;

	if (!hstats_p || !node_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	if (hstats_p->stat_idx < 0) {
		idx = idx % MAX_SUPPORTED_FLOWS_DEBUG;
		hstats_p->stat_idx = idx;
		rmnet_shs_flow_dbg_stats_idx_cnt++;
	}

	rmnet_shs_flow_hash[hstats_p->stat_idx] = hstats_p->hash;
	rmnet_shs_flow_proto[hstats_p->stat_idx] = node_p->skb_tport_proto;
	rmnet_shs_flow_inactive_tsec[hstats_p->stat_idx] =
			RMNET_SHS_NSEC_TO_SEC(hstats_p->inactive_duration);
	rmnet_shs_flow_rx_bps[hstats_p->stat_idx] = hstats_p->rx_bps;
	rmnet_shs_flow_rx_pps[hstats_p->stat_idx] = hstats_p->rx_pps;
	rmnet_shs_flow_rx_bytes[hstats_p->stat_idx] = hstats_p->rx_bytes;
	rmnet_shs_flow_rx_pkts[hstats_p->stat_idx] = hstats_p->rx_skb;
	rmnet_shs_flow_cpu[hstats_p->stat_idx] = hstats_p->current_cpu;
	rmnet_shs_flow_cpu_recommended[hstats_p->stat_idx] =
						hstats_p->suggested_cpu;
	rmnet_shs_flow_silver_to_gold[hstats_p->stat_idx] =
		hstats_p->rmnet_shs_wq_suggs[RMNET_SHS_WQ_SUGG_SILVER_TO_GOLD];
	rmnet_shs_flow_gold_to_silver[hstats_p->stat_idx] =
		hstats_p->rmnet_shs_wq_suggs[RMNET_SHS_WQ_SUGG_GOLD_TO_SILVER];
	rmnet_shs_flow_gold_balance[hstats_p->stat_idx] =
		hstats_p->rmnet_shs_wq_suggs[RMNET_SHS_WQ_SUGG_GOLD_BALANCE];

}

/* Returns TRUE if this flow received a new packet
 *         FALSE otherwise
 */
u8 rmnet_shs_wq_is_hash_rx_new_pkt(struct rmnet_shs_wq_hstat_s *hstats_p,
				   struct rmnet_shs_skbn_s *node_p)
{
	if (!hstats_p || !node_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return 0;
	}

	if (node_p->num_skb == hstats_p->rx_skb)
		return 0;

	return 1;
}

void rmnet_shs_wq_update_hash_tinactive(struct rmnet_shs_wq_hstat_s *hstats_p,
					struct rmnet_shs_skbn_s *node_p)
{
	ktime_t tdiff;

	if (!hstats_p || !node_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	tdiff = rmnet_shs_wq_tnsec - hstats_p->c_epoch;
	hstats_p->inactive_duration = tdiff;

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_FLOW_STATS,
				RMNET_SHS_WQ_FLOW_STATS_FLOW_INACTIVE,
				hstats_p->hash, tdiff, 0xDEF, 0xDEF,
				hstats_p, NULL);
}

void rmnet_shs_wq_update_hash_stats(struct rmnet_shs_wq_hstat_s *hstats_p)
{
	ktime_t tdiff;
	u64 skb_diff, coal_skb_diff, bytes_diff;
	struct rmnet_shs_skbn_s *node_p;

	if (!hstats_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	node_p = hstats_p->node;

	if (!rmnet_shs_wq_is_hash_rx_new_pkt(hstats_p, node_p)) {
		hstats_p->rx_pps = 0;
		hstats_p->avg_pps = 0;
		hstats_p->rx_bps = 0;
		rmnet_shs_wq_update_hash_tinactive(hstats_p, node_p);
		rmnet_shs_wq_update_hash_stats_debug(hstats_p, node_p);
		return;
	}

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_FLOW_STATS,
				RMNET_SHS_WQ_FLOW_STATS_START,
				hstats_p->hash, 0xDEF, hstats_p->rx_pps,
				hstats_p->rx_bps, hstats_p, NULL);

	rmnet_shs_wq_update_hstat_rps_msk(hstats_p);
	hstats_p->inactive_duration = 0;
	hstats_p->l_epoch = node_p->hstats->c_epoch;
	hstats_p->last_rx_skb = node_p->hstats->rx_skb;
	hstats_p->last_rx_ll_skb = node_p->hstats->rx_ll_skb;

	hstats_p->last_rx_coal_skb = node_p->hstats->rx_coal_skb;
	hstats_p->last_hw_coal_bytes = node_p->hstats->hw_coal_bytes;
	hstats_p->last_hw_coal_bufsize = node_p->hstats->hw_coal_bufsize;
	hstats_p->last_rx_bytes = node_p->hstats->rx_bytes;

	hstats_p->c_epoch = rmnet_shs_wq_tnsec;
	hstats_p->rx_skb = node_p->num_skb;
	hstats_p->rx_ll_skb = node_p->num_ll_skb;
	hstats_p->ll_diff = hstats_p->rx_ll_skb !=  hstats_p->last_rx_ll_skb;

	hstats_p->rx_coal_skb = node_p->num_coal_skb;
	hstats_p->hw_coal_bytes = node_p->hw_coal_bytes;
	hstats_p->hw_coal_bufsize = node_p->hw_coal_bufsize;
	hstats_p->rx_bytes = node_p->num_skb_bytes;
	tdiff = (hstats_p->c_epoch - hstats_p->l_epoch);

	/* Under-cap tdff to be 100ms and check wq_interval_ms > 0*/
	tdiff = (tdiff > RMNET_SHS_MSEC_TO_NSC(rmnet_shs_wq_interval_ms) &&
		rmnet_shs_wq_interval_ms > 0)? tdiff : RMNET_SHS_MSEC_TO_NSC(100);
	skb_diff = hstats_p->rx_skb - hstats_p->last_rx_skb;
	coal_skb_diff = hstats_p->rx_coal_skb - hstats_p->last_rx_coal_skb;
	bytes_diff = hstats_p->rx_bytes - hstats_p->last_rx_bytes;

	rm_err1("SHS_SEGS: hash 0x%x coal skb = %llu | last coal skb = %llu | rx skb = %llu | last rx skb %llu",
	       hstats_p->hash,
	       hstats_p->rx_coal_skb,
	       hstats_p->last_rx_coal_skb,
	       hstats_p->rx_skb,
	       hstats_p->last_rx_skb);

	hstats_p->rx_pps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(skb_diff)/(tdiff);
	hstats_p->rx_bps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(bytes_diff)/(tdiff);
	hstats_p->rx_bps = RMNET_SHS_BYTE_TO_BIT(hstats_p->rx_bps);
	hstats_p->avg_pps = rmnet_shs_wq_get_flow_avg_pps(hstats_p);
	if (coal_skb_diff > 0) {
		hstats_p->avg_segs = skb_diff / coal_skb_diff;
		rm_err1("SHS_SEGS: avg segs = %llu skb_diff = %llu coal_skb_diff = %llu",
		       hstats_p->avg_segs, skb_diff, coal_skb_diff);

	} else {
		hstats_p->avg_segs = 0;
	}
	hstats_p->hw_coal_bytes_diff = hstats_p->hw_coal_bytes - hstats_p->last_hw_coal_bytes;
	hstats_p->hw_coal_bufsize_diff = hstats_p->hw_coal_bufsize - hstats_p->last_hw_coal_bufsize;
	rm_err1("SHS_HW_COAL: hw coal bytes = %llu hw coal bufsize = %llu",
		node_p->hw_coal_bytes, node_p->hw_coal_bufsize);
	rm_err1("SHS_HW_COAL: LAST: hw coal bytes = %llu hw coal bufsize = %llu",
		hstats_p->last_hw_coal_bytes, hstats_p->last_hw_coal_bufsize);
	rm_err1("SHS_HW_COAL: hw coal bytes diff = %llu hw coal bufsize diff = %llu",
		hstats_p->hw_coal_bytes_diff, hstats_p->hw_coal_bufsize_diff);

	hstats_p->last_pps = hstats_p->rx_pps;
	rmnet_shs_wq_update_hash_stats_debug(hstats_p, node_p);

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_FLOW_STATS,
				RMNET_SHS_WQ_FLOW_STATS_END,
				hstats_p->hash, hstats_p->rx_pps,
				hstats_p->rx_bps, (tdiff/1000000),
				hstats_p, NULL);

	hstats_p->bif = node_p->bif;
	hstats_p->ack_thresh = node_p->ack_thresh;
	rm_err1("SHS_QUICKACK: bif = %u ack_thresh = %u",
		node_p->bif, node_p->ack_thresh);
}

static void rmnet_shs_wq_refresh_cpu_rates_debug(u16 cpu,
				struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_p)
{
	if (!rmnet_shs_stats_enabled)
		return;

	if (cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return;
	}

	if (!cpu_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	rmnet_shs_cpu_rx_bps[cpu] = cpu_p->rx_bps;
	rmnet_shs_cpu_rx_pps[cpu] = cpu_p->rx_pps;
	rmnet_shs_cpu_rx_flows[cpu] = cpu_p->flows;
	rmnet_shs_cpu_rx_bytes[cpu] = cpu_p->rx_bytes;
	rmnet_shs_cpu_rx_pkts[cpu] = cpu_p->rx_skbs;
	rmnet_shs_cpu_qhead_diff[cpu] = cpu_p->qhead_diff;
	rmnet_shs_cpu_qhead_total[cpu] = cpu_p->qhead_total;
}

static void rmnet_shs_wq_refresh_dl_mrkr_stats(void)
{
	struct rmnet_shs_wq_rx_flow_s *tbl_p = &rmnet_shs_rx_flow_tbl;
	struct rmnet_port *port;
	u64 pkt_diff, byte_diff;
	ktime_t tdiff;

	tbl_p->dl_mrk_last_rx_bytes = tbl_p->dl_mrk_rx_bytes;
	tbl_p->dl_mrk_last_rx_pkts = tbl_p->dl_mrk_rx_pkts;

	port = rmnet_shs_cfg.port;
	if (!port) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_GET_RMNET_PORT_ERR]++;
		return;
	}
	tbl_p->dl_mrk_rx_pkts = port->stats.dl_hdr_total_pkts;
	tbl_p->dl_mrk_rx_bytes = port->stats.dl_hdr_total_bytes;
	tdiff = rmnet_shs_wq_tnsec - tbl_p->l_epoch;
	/* Under-cap tdff to be 100ms and check wq_interval_ms > 0*/
	tdiff = (tdiff > RMNET_SHS_MSEC_TO_NSC(rmnet_shs_wq_interval_ms) &&
		rmnet_shs_wq_interval_ms > 0)? tdiff : RMNET_SHS_MSEC_TO_NSC(100);

	pkt_diff = tbl_p->dl_mrk_rx_pkts - tbl_p->dl_mrk_last_rx_pkts;
	byte_diff = tbl_p->dl_mrk_rx_bytes - tbl_p->dl_mrk_last_rx_bytes;
	tbl_p->dl_mrk_rx_pps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(pkt_diff)/tdiff;
	tbl_p->dl_mrk_rx_bps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(byte_diff)/tdiff;
	tbl_p->dl_mrk_rx_bps = RMNET_SHS_BYTE_TO_BIT(tbl_p->dl_mrk_rx_bps);

}

static void rmnet_shs_wq_refresh_total_stats(void)
{
	struct rmnet_shs_wq_rx_flow_s *tbl_p = &rmnet_shs_rx_flow_tbl;
	u64 pkt_diff, byte_diff, pps, bps;
	ktime_t tdiff;

	tdiff = rmnet_shs_wq_tnsec - tbl_p->l_epoch;
	/* Under-cap tdff to be 100ms and check wq_interval_ms > 0*/
	tdiff = (tdiff > RMNET_SHS_MSEC_TO_NSC(rmnet_shs_wq_interval_ms) &&
		rmnet_shs_wq_interval_ms > 0)? tdiff : RMNET_SHS_MSEC_TO_NSC(100);

	pkt_diff = (tbl_p->rx_skbs -  tbl_p->last_rx_skbs);
	byte_diff = tbl_p->rx_bytes -  tbl_p->last_rx_bytes;
	pps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(pkt_diff)/tdiff;
	bps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(byte_diff)/tdiff;
	tbl_p->last_rx_bps = tbl_p->rx_bps;
	tbl_p->last_rx_pps = tbl_p->rx_pps;
	tbl_p->rx_bps = RMNET_SHS_BYTE_TO_BIT(bps);
	tbl_p->rx_pps = pps;
	tbl_p->l_epoch  = rmnet_shs_wq_tnsec;
	tbl_p->last_rx_bytes = tbl_p->rx_bytes;
	tbl_p->last_rx_skbs = tbl_p->rx_skbs;

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_TOTAL_STATS,
				RMNET_SHS_WQ_TOTAL_STATS_UPDATE,
				tbl_p->rx_pps,
				tbl_p->dl_mrk_rx_pps,
				tbl_p->rx_bps,
				tbl_p->dl_mrk_rx_bps, NULL, NULL);

}

static void rmnet_shs_wq_refresh_cpu_stats(u16 cpu)
{
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_p;
	ktime_t tdiff;
	u64 new_skbs, new_bytes;
	u64 last_rx_bps, last_rx_pps;
	u32 new_qhead;

	if (cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return;
	}

	cpu_p = &rmnet_shs_rx_flow_tbl.cpu_list[cpu];
	new_skbs = cpu_p->rx_skbs - cpu_p->last_rx_skbs;

	new_qhead = rmnet_shs_get_cpu_qhead(cpu);
	if (cpu_p->qhead_start == 0)
		cpu_p->qhead_start = new_qhead;

	cpu_p->last_qhead = cpu_p->qhead;
	cpu_p->qhead = new_qhead;
	cpu_p->qhead_diff = cpu_p->qhead - cpu_p->last_qhead;
	cpu_p->qhead_total = cpu_p->qhead - cpu_p->qhead_start;

	if (rmnet_shs_cpu_node_tbl[cpu].wqprio)
		rmnet_shs_cpu_node_tbl[cpu].wqprio = (rmnet_shs_cpu_node_tbl[cpu].wqprio + 1)
						     % (PRIO_BACKOFF);
	if (new_skbs == 0) {
		cpu_p->l_epoch =  rmnet_shs_wq_tnsec;
		cpu_p->rx_bps = 0;
		cpu_p->rx_pps = 0;
		cpu_p->avg_pps = 0;
		if (rmnet_shs_userspace_connected) {
			rmnet_shs_wq_cpu_caps_list_add(&rmnet_shs_rx_flow_tbl,
						       cpu_p, &cpu_caps);
		}
		rmnet_shs_wq_refresh_cpu_rates_debug(cpu, cpu_p);
		rmnet_shs_cpu_node_tbl[cpu].async = 0;
		return;
	}

	tdiff = rmnet_shs_wq_tnsec - cpu_p->l_epoch;
	new_bytes = cpu_p->rx_bytes - cpu_p->last_rx_bytes;

	/* Under-cap tdff to be 100ms and check wq_interval_ms > 0*/
	tdiff = (tdiff > RMNET_SHS_MSEC_TO_NSC(rmnet_shs_wq_interval_ms) &&
		rmnet_shs_wq_interval_ms > 0)? tdiff : RMNET_SHS_MSEC_TO_NSC(100);

	last_rx_bps = cpu_p->rx_bps;
	last_rx_pps = cpu_p->rx_pps;
	cpu_p->rx_pps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(new_skbs)/tdiff;
	cpu_p->rx_bps = RMNET_SHS_RX_BPNSEC_TO_BPSEC(new_bytes)/tdiff;
	cpu_p->rx_bps = RMNET_SHS_BYTE_TO_BIT(cpu_p->rx_bps);
	cpu_p->avg_pps = rmnet_shs_wq_get_cpu_avg_pps(cpu);
	cpu_p->last_rx_bps = last_rx_bps;
	cpu_p->last_rx_pps = last_rx_pps;

	cpu_p->l_epoch =  rmnet_shs_wq_tnsec;
	cpu_p->last_rx_skbs = cpu_p->rx_skbs;
	cpu_p->last_rx_bytes = cpu_p->rx_bytes;
	cpu_p->rx_bps_est = cpu_p->rx_bps;

	if (rmnet_shs_is_lpwr_cpu(cpu) && cpu_p->rx_pps < ASYNC_UPTHRESH &&
	    cpu_p->rx_pps > ASYNC_LOWTHRESH) {
		rmnet_shs_cpu_node_tbl[cpu].async = 1;
	} else {
		rmnet_shs_cpu_node_tbl[cpu].async = 0;
	}

	if (rmnet_shs_userspace_connected) {
		rmnet_shs_wq_cpu_caps_list_add(&rmnet_shs_rx_flow_tbl,
					       cpu_p, &cpu_caps);
	}

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_CPU_STATS,
				RMNET_SHS_WQ_CPU_STATS_UPDATE, cpu,
				cpu_p->flows, cpu_p->rx_pps,
				cpu_p->rx_bps, NULL, NULL);
	rmnet_shs_wq_refresh_cpu_rates_debug(cpu, cpu_p);

}

static void rmnet_shs_wq_refresh_all_cpu_stats(void)
{
	u16 cpu;

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_CPU_STATS,
				RMNET_SHS_WQ_CPU_STATS_START,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	for (cpu = 0; cpu < MAX_CPUS; cpu++)
		rmnet_shs_wq_refresh_cpu_stats(cpu);

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_CPU_STATS,
				RMNET_SHS_WQ_CPU_STATS_END,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
}

void rmnet_shs_wq_update_cpu_rx_tbl(struct rmnet_shs_wq_hstat_s *hstat_p)
{
	struct rmnet_shs_wq_rx_flow_s *tbl_p = &rmnet_shs_rx_flow_tbl;
	struct rmnet_shs_skbn_s *node_p;
	u64 skb_diff, byte_diff;
	u16 cpu_num;

	if (!hstat_p) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	node_p = hstat_p->node;

	if (hstat_p->inactive_duration > 0)
		return;

	cpu_num = node_p->map_cpu;

	if (cpu_num >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_INVALID_CPU_ERR]++;
		return;
	}
	skb_diff = hstat_p->rx_skb - hstat_p->last_rx_skb;
	byte_diff = hstat_p->rx_bytes - hstat_p->last_rx_bytes;

	/* check if the flow has switched to another CPU*/
	if (cpu_num != hstat_p->current_cpu && !hstat_p->is_new_flow ) {
		rm_err("SHS_FLOW: moving flow 0x%x on cpu[%d] to cpu[%d] "
		       "pps: %llu | avg_pps %llu",
		       hstat_p->hash, hstat_p->current_cpu, cpu_num,
		       hstat_p->rx_pps, hstat_p->avg_pps);
		trace_rmnet_shs_wq_high(RMNET_SHS_WQ_FLOW_STATS,
					RMNET_SHS_WQ_FLOW_STATS_UPDATE_NEW_CPU,
					hstat_p->hash, hstat_p->current_cpu,
					cpu_num, 0xDEF, hstat_p, NULL);

		rmnet_shs_cpu_list_move(hstat_p,
				   &tbl_p->cpu_list[cpu_num].hstat_id);

		rmnet_shs_wq_inc_cpu_flow(cpu_num);
		rmnet_shs_wq_dec_cpu_flow(hstat_p->current_cpu);
		hstat_p->current_cpu = cpu_num;
	} else if (hstat_p->is_new_flow) {
		rmnet_shs_wq_inc_cpu_flow(cpu_num);
		rmnet_shs_cpu_list_add(hstat_p,
				       &tbl_p->cpu_list[cpu_num].hstat_id);
		rm_err("SHS_FLOW: adding flow 0x%x on cpu[%d] "
		       "pps: %llu | avg_pps %llu",
		       hstat_p->hash, hstat_p->current_cpu,
		       hstat_p->rx_pps, hstat_p->avg_pps);
		hstat_p->is_new_flow = 0;
		hstat_p->current_cpu = cpu_num;
	}

	/* Assuming that the data transfers after the last refresh
	 * interval have happened with the newer CPU
	 */
	tbl_p->cpu_list[cpu_num].rx_skbs += skb_diff;
	tbl_p->cpu_list[cpu_num].rx_bytes += byte_diff;
	tbl_p->rx_skbs += skb_diff;
	tbl_p->rx_bytes += byte_diff;

}

void rmnet_shs_wq_chng_suggested_cpu(u16 old_cpu, u16 new_cpu,
					      struct rmnet_shs_wq_ep_s *ep)
{
	struct rmnet_shs_skbn_s *node_p;
	struct rmnet_shs_wq_hstat_s *hstat_p;
	struct hlist_node *tmp;
	u16 bkt;

	spin_lock_bh(&rmnet_shs_ht_splock);
	hash_for_each_safe(RMNET_SHS_HT, bkt, tmp, node_p, list) {

		if (!node_p)
			continue;

		if (!node_p->hstats)
			continue;

		hstat_p = node_p->hstats;

		if ((hstat_p->suggested_cpu == old_cpu) &&
		    (node_p->dev == ep->ep)) {

			trace_rmnet_shs_wq_high(RMNET_SHS_WQ_FLOW_STATS,
				RMNET_SHS_WQ_FLOW_STATS_SUGGEST_NEW_CPU,
				hstat_p->hash, hstat_p->suggested_cpu,
				new_cpu, 0xDEF, hstat_p, NULL);

			node_p->hstats->suggested_cpu = new_cpu;
		}
	}
	spin_unlock_bh(&rmnet_shs_ht_splock);
}

/* Increment the per-flow counter for suggestion type */
static void rmnet_shs_wq_inc_sugg_type(u32 sugg_type,
				       struct rmnet_shs_wq_hstat_s *hstat_p)
{
	if (sugg_type >= RMNET_SHS_WQ_SUGG_MAX || hstat_p == NULL)
		return;

	hstat_p->rmnet_shs_wq_suggs[sugg_type] += 1;
}

/* Change suggested cpu, return 1 if suggestion was made, 0 otherwise */
static int rmnet_shs_wq_chng_flow_cpu(u16 old_cpu, u16 new_cpu,
				      struct rmnet_shs_wq_ep_s *ep,
				      u32 hash_to_move, u32 sugg_type)
{
	struct rmnet_shs_skbn_s *node_p;
	struct rmnet_shs_wq_hstat_s *hstat_p;
	struct hlist_node *tmp;
	int rc = 0;
	u16 bkt;

	if (!ep) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		return 0;
	}

	if (old_cpu >= MAX_CPUS || new_cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return 0;
	}
	spin_lock_bh(&rmnet_shs_ht_splock);
	hash_for_each_safe(RMNET_SHS_HT, bkt, tmp, node_p, list) {
		if (!node_p)
			continue;

		if (!node_p->hstats)
			continue;

		hstat_p = node_p->hstats;

		if (hash_to_move != 0) {
			/* If hash_to_move is given, only move that flow,
			 * otherwise move all the flows on that cpu
			 */
			if (hstat_p->hash != hash_to_move)
				continue;
		}

		rm_err("SHS_HT: >>  sugg cpu %d | old cpu %d | new_cpu %d | "
		       "map_cpu = %d | flow 0x%x",
		       hstat_p->suggested_cpu, old_cpu, new_cpu,
		       node_p->map_cpu, hash_to_move);

		if ((hstat_p->suggested_cpu == old_cpu) &&
		    (node_p->dev == ep->ep)) {

			trace_rmnet_shs_wq_high(RMNET_SHS_WQ_FLOW_STATS,
				RMNET_SHS_WQ_FLOW_STATS_SUGGEST_NEW_CPU,
				hstat_p->hash, hstat_p->suggested_cpu,
				new_cpu, 0xDEF, hstat_p, NULL);

			node_p->hstats->suggested_cpu = new_cpu;
			rmnet_shs_wq_inc_sugg_type(sugg_type, hstat_p);
			if (hash_to_move) { /* Stop after moving one flow */
				rm_err("SHS_CHNG: moving single flow: flow 0x%x "
				       "sugg_cpu changed from %d to %d",
				       hstat_p->hash, old_cpu,
				       node_p->hstats->suggested_cpu);
				rc = 1;
				break;
			}
			rm_err("SHS_CHNG: moving all flows: flow 0x%x "
			       "sugg_cpu changed from %d to %d",
			       hstat_p->hash, old_cpu,
			       node_p->hstats->suggested_cpu);
			rc |= 1;
		}
	}
	spin_unlock_bh(&rmnet_shs_ht_splock);

	return rc;
}

u64 rmnet_shs_wq_get_max_pps_among_cores(u32 core_msk)
{
	int cpu_num;
	u64 max_pps = 0;
	struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl;

	for (cpu_num = 0; cpu_num < MAX_CPUS; cpu_num++) {
		if (((1 << cpu_num) & core_msk) &&
		     (rx_flow_tbl_p->cpu_list[cpu_num].rx_pps > max_pps)) {
			max_pps = rx_flow_tbl_p->cpu_list[cpu_num].rx_pps;
		}
	}
	return max_pps;
}

/* Returns the least utilized core from a core mask
 * In order of priority
 *    1) Returns rightmost core with no flows (Fully Idle)
 *    2) Returns the core with least flows with no pps (Semi Idle)
 *    3) Returns the core with the least pps (Non-Idle)
 */
int rmnet_shs_wq_get_least_utilized_core(u16 core_msk)
{
	struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl;
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *list_p;
	u64 min_pps = U64_MAX;
	u32 min_flows = U32_MAX;
	int ret_val = -1;
	int semi_idle_ret = -1;
	int full_idle_ret = -1;
	int cpu_num = 0;
	u16 is_cpu_in_msk;

	for (cpu_num = MAX_CPUS-1; cpu_num >= 0; cpu_num--) {

		is_cpu_in_msk = ((1 << cpu_num) & core_msk) && cpu_active(cpu_num) && !is_reserved(cpu_num);
		if (!is_cpu_in_msk)
			continue;

		list_p = &rx_flow_tbl_p->cpu_list[cpu_num];
		trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
				       RMNET_SHS_WQ_CPU_STATS_CURRENT_UTIL,
				       cpu_num, list_p->rx_pps, min_pps,
				       0, NULL, NULL);

		/* When there are multiple free CPUs the first free CPU will
		 * be returned
		 */
		if (list_p->flows == 0) {
			full_idle_ret = cpu_num;
			break;
		}
		/* When there are semi-idle CPUs the CPU w/ least flows will
		 * be returned
		 */
		if (list_p->rx_pps == 0 && list_p->flows < min_flows) {
			min_flows = list_p->flows;
			semi_idle_ret = cpu_num;
		}

		/* Found a core that is processing even lower packets */
		if (list_p->rx_pps <= min_pps) {
			min_pps = list_p->rx_pps;
			ret_val = cpu_num;
		}
	}

	if (full_idle_ret >= 0)
		ret_val = full_idle_ret;
	else if (semi_idle_ret >= 0)
		ret_val = semi_idle_ret;

	return ret_val;
}

u16 rmnet_shs_wq_find_cpu_to_move_flows(u16 current_cpu,
					struct rmnet_shs_wq_ep_s *ep)
{
	struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl;
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_list_p, *cur_cpu_list_p;
	u64 cpu_rx_pps, reqd_pps, cur_cpu_rx_pps;
	u64 pps_uthresh, pps_lthresh = 0;
	u16 cpu_to_move = current_cpu;
	u16 cpu_num;
	u8 is_core_in_msk;
	u32 cpu_to_move_util = 0;

	if (!ep) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		return cpu_to_move;
	}

	cur_cpu_list_p = &rx_flow_tbl_p->cpu_list[current_cpu];
	cur_cpu_rx_pps = cur_cpu_list_p->rx_pps;
	pps_uthresh = rmnet_shs_cpu_rx_max_pps_thresh[current_cpu];
	pps_lthresh = rmnet_shs_cpu_rx_min_pps_thresh[current_cpu];

	/* If gold cores are not rps usable or we are already on a perf core
	 * and required pps is beyond beyond the capacity that even perf
	 * cores aren't sufficient there is nothing much we can do. So we
	 * will continue to let flows process packets on same core
	 */
	if (!(rmnet_shs_cfg.map_mask & PERF_MASK) ||
		(!rmnet_shs_is_lpwr_cpu(current_cpu) &&
	    (cur_cpu_rx_pps > pps_lthresh))) {
		return cpu_to_move;
	}
	/* If a core (should only be lpwr was marked prio we don't touch it
	 * for a few ticks and reset it afterwards
	 */

	if (rmnet_shs_cpu_node_tbl[current_cpu].wqprio)
		return current_cpu;

	for (cpu_num = 0; cpu_num < MAX_CPUS; cpu_num++) {

		/* Do not consider invalid CPUs as potential new cpus */
		is_core_in_msk = ((1 << cpu_num) &
		                  (ep->rps_config_msk & ~rmnet_shs_cfg.ban_mask) &
						  ~rmnet_shs_halt_mask);

		/* We are looking for a core that is configured and that
		 * can handle traffic better than the current core
		 */
		if ((cpu_num == current_cpu) || (!is_core_in_msk) ||
		    !cpu_active(cpu_num))
			continue;

		pps_uthresh = rmnet_shs_cpu_rx_max_pps_thresh[cpu_num];
		pps_lthresh = rmnet_shs_cpu_rx_min_pps_thresh[cpu_num];

		cpu_list_p = &rx_flow_tbl_p->cpu_list[cpu_num];
		cpu_rx_pps = cpu_list_p->rx_pps;
		reqd_pps = cpu_rx_pps + cur_cpu_rx_pps;

		trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
				       RMNET_SHS_WQ_CPU_STATS_CORE2SWITCH_FIND,
				       current_cpu, cpu_num, reqd_pps,
				       cpu_rx_pps, NULL, NULL);

		/* Return the most available valid CPU */
		if ((reqd_pps > pps_lthresh) && (reqd_pps < pps_uthresh) &&
			cpu_rx_pps <= cpu_to_move_util) {
			cpu_to_move = cpu_num;
			cpu_to_move_util = cpu_rx_pps;
		}
	}

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_CPU_STATS,
			     RMNET_SHS_WQ_CPU_STATS_CORE2SWITCH_FIND,
			     current_cpu, cpu_to_move, cur_cpu_rx_pps,
			     rx_flow_tbl_p->cpu_list[cpu_to_move].rx_pps,
			     NULL, NULL);
	return cpu_to_move;
}

void rmnet_shs_wq_find_cpu_and_move_flows(u16 cur_cpu)
{
	struct rmnet_shs_wq_ep_s *ep = NULL;
	u16 new_cpu;

	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {
		if (!ep->is_ep_active)
			continue;

		new_cpu = rmnet_shs_wq_find_cpu_to_move_flows(cur_cpu, ep);

		if (new_cpu != cur_cpu)
			rmnet_shs_wq_chng_suggested_cpu(cur_cpu, new_cpu, ep);
	}
}

/* Return 1 if we can move a flow to dest_cpu for this endpoint,
 * otherwise return 0. Basically check rps mask and cpu is online
 */
int rmnet_shs_wq_check_cpu_move_for_ep(u16 current_cpu, u16 dest_cpu,
				       struct rmnet_shs_wq_ep_s *ep)
{
	u16 cpu_in_rps_mask = 0;

	if (!ep) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		return 0;
	}

	if (current_cpu >= MAX_CPUS || dest_cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return 0;
	}

	cpu_in_rps_mask = (((1 << dest_cpu) & ep->rps_config_msk & ~rmnet_shs_cfg.ban_mask &
			  ~rmnet_shs_halt_mask)) && cpu_active(dest_cpu);

	rm_err("SHS_MASK:  cur cpu [%d] | dest_cpu [%d] | "
	       "ep_rps_mask = 0x%x | cpu_active(dest) = %d"
	       "cpu_in_rps_mask = %d, halt_mask %x\n",
	       current_cpu, dest_cpu, ep->rps_config_msk,
	       cpu_active(dest_cpu), cpu_in_rps_mask, rmnet_shs_halt_mask);

	/* We cannot move to dest cpu if the cur cpu is the same,
	 * the dest cpu is offline, dest cpu is not in the rps mask
	 */
	if (!cpu_in_rps_mask) {
		rmnet_shs_mid_err[RMNET_SHS_SUGG_FAIL1]++;
		return 0;
	}


	if (current_cpu == dest_cpu || !cpu_active(dest_cpu)) {
		rmnet_shs_mid_err[RMNET_SHS_SUGG_FAIL2]++;
		return 0;
	}

	return 1;
}

/* rmnet_shs_wq_try_to_move_flow - try to make a flow suggestion
 * return 1 if flow move was suggested, otherwise return 0
 */
int rmnet_shs_wq_try_to_move_flow(u16 cur_cpu, u16 dest_cpu, u32 hash_to_move,
				  u32 sugg_type)
{
	struct rmnet_shs_wq_ep_s *ep;

	if (cur_cpu >= MAX_CPUS || dest_cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return 0;
	}

	/* Traverse end-point list, check if cpu can be used, based
	 * on it if is online, rps mask, etc. then make
	 * suggestion to change the cpu for the flow by passing its hash
	 */
	spin_lock_bh(&rmnet_shs_ep_lock);
	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {
		if (!ep->is_ep_active)
			continue;

		if (!rmnet_shs_wq_check_cpu_move_for_ep(cur_cpu,
							dest_cpu,
							ep)) {
			rm_err("SHS_FDESC: >> Cannot move flow 0x%x on ep"
			       " from cpu[%d] to cpu[%d]",
			       hash_to_move, cur_cpu, dest_cpu);
			continue;
		}

		if (rmnet_shs_wq_chng_flow_cpu(cur_cpu, dest_cpu, ep,
					       hash_to_move, sugg_type)) {
			rm_err("SHS_FDESC: >> flow 0x%x was suggested to"
			       " move from cpu[%d] to cpu[%d] sugg_type [%d]",
			       hash_to_move, cur_cpu, dest_cpu, sugg_type);

			spin_unlock_bh(&rmnet_shs_ep_lock);
            return 1;
		}
	}

	spin_unlock_bh(&rmnet_shs_ep_lock);

	return 0;
}

/* Change flow segmentation, return 1 if set, 0 otherwise */
int rmnet_shs_wq_set_flow_segmentation(u32 hash_to_set, u8 segs_per_skb)
{
	struct rmnet_shs_skbn_s *node_p;
	struct rmnet_shs_wq_hstat_s *hstat_p;
	u16 bkt;

	spin_lock_bh(&rmnet_shs_ht_splock);
	hash_for_each(RMNET_SHS_HT, bkt, node_p, list) {
		if (!node_p)
			continue;

		if (!node_p->hstats)
			continue;

		hstat_p = node_p->hstats;

		if (hstat_p->hash != hash_to_set)
			continue;

		rm_err("SHS_HT: >> segmentation on hash 0x%x segs_per_skb %u",
		       hash_to_set, segs_per_skb);

		trace_rmnet_shs_wq_high(RMNET_SHS_WQ_FLOW_STATS,
				RMNET_SHS_WQ_FLOW_STATS_SET_FLOW_SEGMENTATION,
				hstat_p->hash, segs_per_skb,
				0xDEF, 0xDEF, hstat_p, NULL);

		node_p->hstats->segs_per_skb = segs_per_skb;
		spin_unlock_bh(&rmnet_shs_ht_splock);
		return 1;
	}
	spin_unlock_bh(&rmnet_shs_ht_splock);

	rm_err("SHS_HT: >> segmentation on hash 0x%x segs_per_skb %u not set - hash not found",
	       hash_to_set, segs_per_skb);
	return 0;
}

/* Change quickack threshold, return 1 if set, 0 otherwise */
int rmnet_shs_wq_set_quickack_thresh(u32 hash_to_set, u32 ack_thresh)
{
	/* Call the hoook in rmnet_perf to set the quickack thresh */
	rm_err("Calling quickack thresh hook in rmnet_perf w/ hash 0x%x and thresh %u",
		hash_to_set, ack_thresh);
	if (rmnet_module_hook_perf_set_thresh(hash_to_set, ack_thresh)) {
		rm_err("Successfully changed ack_thresh to %u", ack_thresh);
		return 1;
	}

	rm_err("Failed to change ack_thresh to %u", ack_thresh);
	return 0;
}

/* Prints cpu stats and flows to dmesg for debugging */
void rmnet_shs_wq_debug_print_flows(void)
{
	struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl;
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_node;
	struct rmnet_shs_wq_hstat_s *hnode;
	int flows, i;
	u16 cpu_num = 0;

	if (!RMNET_SHS_DEBUG)
		return;

	for (cpu_num = 0; cpu_num < MAX_CPUS; cpu_num++) {
		cpu_node = &rx_flow_tbl_p->cpu_list[cpu_num];
		flows = rx_flow_tbl_p->cpu_list[cpu_num].flows;

		rm_err("SHS_CPU: cpu[%d]: flows=%d pps=%llu bps=%llu "
		       "qhead_diff %u qhead_total = %u qhead_start = %u "
		       "qhead = %u qhead_last = %u ",
		       cpu_num, flows, cpu_node->rx_pps, cpu_node->rx_bps,
		       cpu_node->qhead_diff, cpu_node->qhead_total,
		       cpu_node->qhead_start,
		       cpu_node->qhead, cpu_node->last_qhead);
		rcu_read_lock();
		list_for_each_entry_rcu(hnode,
				    &rmnet_shs_wq_hstat_tbl,
				    hstat_node_id) {

			if (hnode->in_use == 0)
				continue;

			if (hnode->node) {
				if (hnode->current_cpu == cpu_num)
					rm_err("SHS_CPU:         > flow 0x%x "
					       "with pps %llu avg_pps %llu rx_bps %llu ",
					       hnode->hash, hnode->rx_pps,
					       hnode->avg_pps, hnode->rx_bps);
			}
		} /* loop per flow */
		rcu_read_unlock();

		for (i = 0; i < 3 - flows; i++) {
			rm_err("%s", "SHS_CPU:         > ");
		}
	} /* loop per cpu */
}

/* Prints the sorted gold flow list to dmesg */
void rmnet_shs_wq_debug_print_sorted_gold_flows(struct list_head *gold_flows)
{
	struct rmnet_shs_wq_gold_flow_s *gflow_node;

	if (!RMNET_SHS_DEBUG)
		return;

	if (!gold_flows) {
		rm_err("%s", "SHS_GDMA: Gold Flows List is NULL");
		return;
	}

	rm_err("%s", "SHS_GDMA: List of sorted gold flows:");
	list_for_each_entry(gflow_node, gold_flows, gflow_list) {
		rm_err("SHS_GDMA: > flow 0x%x with pps %llu on cpu[%d]",
		       gflow_node->hash, gflow_node->rx_pps,
		       gflow_node->cpu_num);
	}
}

/* Userspace evaluation. we send userspace the response to the sync message
 * after we update shared memory. shsusr will send a netlink message if
 * flows should be moved around.
 */
void rmnet_shs_wq_eval_cpus_caps_and_flows(struct list_head *cpu_caps,
					   struct list_head *gold_flows,
					   struct list_head *ss_flows,
					   struct list_head *fflows, struct list_head *llflows)
{
	if (!cpu_caps || !gold_flows || !ss_flows || !fflows || !llflows) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_PTR_ERR]++;
		return;
	}

	list_sort(NULL, cpu_caps, &cmp_fn_cpu_pps);
	list_sort(NULL, gold_flows, &cmp_fn_flow_pps);
	list_sort(NULL, fflows, &cmp_fn_filter_flow_pps);
	list_sort(NULL, llflows, &cmp_fn_ll_flow_pps);

	rmnet_shs_wq_mem_update_cached_cpu_caps(cpu_caps);
	rmnet_shs_wq_mem_update_cached_sorted_gold_flows(gold_flows);
	rmnet_shs_wq_mem_update_cached_sorted_ss_flows(ss_flows);
	rmnet_shs_wq_mem_update_cached_sorted_fflows(fflows);
	rmnet_shs_wq_mem_update_cached_sorted_ll_flows(llflows);
	rmnet_shs_wq_mem_update_cached_netdevs();

	rmnet_shs_genl_send_int_to_userspace_no_info(RMNET_SHS_SYNC_RESP_INT);

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR, RMNET_SHS_WQ_SHSUSR_SYNC_END,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
}

/* Default wq evaluation logic, use this if rmnet_shs_userspace_connected is 0 */
void rmnet_shs_wq_eval_suggested_cpu(void)

{
	struct rmnet_shs_wq_rx_flow_s *rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl;
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *cpu_list_p;
	u64 cpu_curr_pps, cpu_last_pps, last_avg_pps;
	u64 moving_avg_pps, avg_pps;
	u64 pps_uthresh, pps_lthresh = 0;
	u16 cpu_num, new_weight, old_weight;
	int flows;

	for (cpu_num = 0; cpu_num < MAX_CPUS; cpu_num++) {
		flows = rx_flow_tbl_p->cpu_list[cpu_num].flows;

		/* Nothing to evaluate if there is no traffic on this cpu */
		if (flows <= 0)
			continue;

		cpu_list_p = &rx_flow_tbl_p->cpu_list[cpu_num];
		cpu_curr_pps = cpu_list_p->rx_pps;
		cpu_last_pps = cpu_list_p->last_rx_pps;
		last_avg_pps = cpu_list_p->avg_pps;
		pps_uthresh = rmnet_shs_cpu_rx_max_pps_thresh[cpu_num];
		pps_lthresh = rmnet_shs_cpu_rx_min_pps_thresh[cpu_num];

		/* Often when we decide to switch from a small cluster core,
		 * it is because of the heavy traffic on that core. In such
		 * circumstances, we want to switch to a big cluster
		 * core as soon as possible. Therefore, we will provide a
		 * greater weightage to the most recent sample compared to
		 * the previous samples.
		 *
		 * On the other hand, when a flow which is on a big cluster
		 * cpu suddenly starts to receive low traffic we move to a
		 * small cluster core after observing low traffic for some
		 * more samples. This approach avoids switching back and forth
		 * to small cluster cpus due to momentary decrease in data
		 * traffic.
		 */
		if (rmnet_shs_is_lpwr_cpu(cpu_num)) {
			new_weight = rmnet_shs_wq_tuning;
			old_weight = 100 - rmnet_shs_wq_tuning;

		} else	{
			old_weight = rmnet_shs_wq_tuning;
			new_weight = 100 - rmnet_shs_wq_tuning;

		}

		/*computing weighted average*/
		moving_avg_pps = (cpu_last_pps + last_avg_pps) / 2;
		avg_pps = ((new_weight * cpu_curr_pps) +
			   (old_weight * moving_avg_pps)) /
			   (new_weight + old_weight);

		cpu_list_p->avg_pps = avg_pps;

		trace_rmnet_shs_wq_high(RMNET_SHS_WQ_CPU_STATS,
				   RMNET_SHS_WQ_CPU_STATS_CORE2SWITCH_EVAL_CPU,
				   cpu_num, cpu_curr_pps, cpu_last_pps,
				   avg_pps, NULL, NULL);

		/* If cpu is now in ban list move flows or offline */
		if ((avg_pps > pps_uthresh) ||
		    ((1 << cpu_num) & (rmnet_shs_cfg.ban_mask | rmnet_shs_halt_mask)) ||
		    !cpu_active(cpu_num) ||
		    ((avg_pps < pps_lthresh) && (cpu_curr_pps < pps_lthresh)))
			rmnet_shs_wq_find_cpu_and_move_flows(cpu_num);
	}

}

void rmnet_shs_wq_refresh_new_flow_list_per_ep(struct rmnet_shs_wq_ep_s *ep)
{
	int lo_core;
	int hi_core;
	u16 rps_msk;
	u16 lo_msk;
	u16 hi_msk;
	u8 lo_core_idx = 0;
	u8 hi_core_idx = 0;

	if (!ep) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		return;
	}

	rps_msk = ep->rps_config_msk;
	lo_msk = ep->default_core_msk;
	hi_msk = ep->pri_core_msk;
	memset(ep->new_lo_core, -1, sizeof(*ep->new_lo_core) * MAX_CPUS);
	memset(ep->new_hi_core, -1, sizeof(*ep->new_hi_core) * MAX_CPUS);
	do {
		lo_core = rmnet_shs_wq_get_least_utilized_core(lo_msk &  ~rmnet_shs_cfg.ban_mask &
							       ~rmnet_shs_halt_mask);
		if (lo_core >= 0) {
			ep->new_lo_core[lo_core_idx] = lo_core;
			lo_msk = lo_msk & ~(1 << lo_core);
			lo_core_idx++;
		} else {
			break;
		}

	} while (lo_msk != 0);

		trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
			    RMNET_SHS_WQ_CPU_STATS_NEW_FLOW_LIST_LO,
			    ep->new_lo_core[0], ep->new_lo_core[1],
			    ep->new_lo_core[2], ep->new_lo_max,
			    ep, NULL);

	do {
		hi_core = rmnet_shs_wq_get_least_utilized_core(hi_msk & ~rmnet_shs_cfg.ban_mask &
							       ~rmnet_shs_halt_mask);
		if (hi_core >= 0) {
			ep->new_hi_core[hi_core_idx] = hi_core;
			hi_msk = hi_msk & ~(1 << hi_core);
			hi_core_idx++;
		} else
			break;

	} while (hi_msk != 0);

	ep->new_lo_max = lo_core_idx;
	ep->new_hi_max = hi_core_idx;
	ep->new_lo_idx = 0;
	ep->new_hi_idx = 0;

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
			    RMNET_SHS_WQ_CPU_STATS_NEW_FLOW_LIST_HI,
			    ep->new_hi_core[0], ep->new_hi_core[1],
			    ep->new_hi_core[2], ep->new_hi_max,
			    ep, NULL);

	return;

}
void rmnet_shs_wq_refresh_new_flow_list(void)
{
	struct rmnet_shs_wq_ep_s *ep = NULL;

	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {
		if (!ep->is_ep_active)
			continue;

		rmnet_shs_wq_refresh_new_flow_list_per_ep(ep);
	}
}

static int rmnet_shs_wq_time_check(ktime_t time, atomic_long_t num_flows)
{

	int ret = false;
	u32 flows = atomic_long_read(&rmnet_shs_cfg.num_flows);

	if (time > rmnet_shs_max_flow_inactivity_sec)
		ret = true;
	else if (flows> FLOW_LIMIT2 && time > INACTIVE_TSEC2)
		ret = true;
	else if (flows > FLOW_LIMIT1 && time > INACTIVE_TSEC1)
		ret = true;

	return ret;
}

void rmnet_shs_wq_cleanup_hash_tbl(u8 force_clean)
{
	struct rmnet_shs_skbn_s *node_p = NULL;
	ktime_t tns2s;
	struct rmnet_shs_wq_hstat_s *hnode = NULL;
	struct list_head *ptr = NULL, *next = NULL;

	rcu_read_lock();
	spin_lock_bh(&rmnet_shs_ht_splock);
	list_for_each_safe(ptr, next, &rmnet_shs_wq_hstat_tbl) {
		hnode = list_entry(ptr, struct rmnet_shs_wq_hstat_s, hstat_node_id);

		if (hnode->node == NULL)
			continue;
		/* If shs just is calling  rmnet_rx_handler prevent cleanup of nodes */
		if (rmnet_shs_cfg.kfree_stop && !force_clean)
			continue;

		node_p = hnode->node;
		tns2s = RMNET_SHS_NSEC_TO_SEC(hnode->inactive_duration);

		/* Flows are cleanup from book keeping faster if
		 * there are a lot of active flows already in memory
		 * Only clear phy node if shs_switch is off.
		 */
		if ((rmnet_shs_wq_time_check(tns2s, rmnet_shs_cfg.num_flows) &&
		    ((node_p->phy && !rmnet_module_hook_is_set(RMNET_MODULE_HOOK_SHS_SWITCH)) || !node_p->phy)) ||
		    force_clean) {
			trace_rmnet_shs_wq_low(RMNET_SHS_WQ_FLOW_STATS,
					       RMNET_SHS_WQ_FLOW_STATS_FLOW_INACTIVE_TIMEOUT,
					       node_p->hash, tns2s, 0xDEF, 0xDEF, node_p, hnode);

			/* Shouldn't be needed for LL flows as no parking is done*/
			rmnet_shs_clear_node(node_p, RMNET_WQ_CTXT);
			rmnet_shs_wq_dec_cpu_flow(hnode->current_cpu);
			if (node_p) {
			/* Low latency nodes need to be cleared from LL ht list with LL locking */
				if(node_p->low_latency) {
					spin_lock_bh(&rmnet_shs_ll_ht_splock);
					rmnet_shs_cpu_node_remove(node_p);
					hash_del_rcu(&node_p->list);
					node_p->node_id.next = NULL;
					node_p->node_id.prev = NULL;
					kfree(node_p);
					spin_unlock_bh(&rmnet_shs_ll_ht_splock);
				}
				else {
					rmnet_shs_cpu_node_remove(node_p);
					hash_del_rcu(&node_p->list);
					node_p->node_id.next = NULL;
					node_p->node_id.prev = NULL;
					kfree(node_p);
				}
			}
			rm_err("SHS_FLOW: removing flow 0x%x on cpu[%d] "
			       "pps: %llu avg_pps: %llu",
			       hnode->hash, hnode->current_cpu,
			       hnode->rx_pps, hnode->avg_pps);
			rmnet_shs_cpu_list_remove(hnode);
			if (hnode->is_perm == 0 || force_clean) {
				rmnet_shs_hstat_tbl_remove(hnode);
				hnode->hstat_node_id.next = NULL;
				hnode->hstat_node_id.prev = NULL;
				kfree(hnode);
			} else {
				rmnet_shs_wq_hstat_reset_node(hnode);
			}
			atomic_long_dec(&rmnet_shs_cfg.num_flows);
		}

	}
	spin_unlock_bh(&rmnet_shs_ht_splock);
	rcu_read_unlock();

}

void rmnet_shs_wq_update_ep_rps_msk(struct rmnet_shs_wq_ep_s *ep)
{
	struct rps_map *map;
	u8 len = 0;

	if (!ep || !ep->ep ) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		return;
	}

	rcu_read_lock();
	if (!ep->ep) {
		pr_info("rmnet_shs invalid state %p\n", ep->ep);
		rmnet_shs_crit_err[RMNET_SHS_WQ_EP_ACCESS_ERR]++;
		return;
	}
	map = rcu_dereference(ep->ep->_rx->rps_map);

	ep->rps_config_msk = 0;
	if (map != NULL) {
		for (len = 0; len < map->len; len++)
			ep->rps_config_msk |= (1 << map->cpus[len]);
	}
	rcu_read_unlock();

	ep->default_core_msk = ep->rps_config_msk & 0x0F;
	ep->pri_core_msk = ep->rps_config_msk & 0xF0;
}

void rmnet_shs_wq_reset_ep_active(struct net_device *dev)
{
	struct rmnet_shs_wq_ep_s *ep = NULL;
	struct rmnet_shs_wq_ep_s *tmp = NULL;

	if (!dev) {
		rmnet_shs_crit_err[RMNET_SHS_NETDEV_ERR]++;
		return;
	}

	spin_lock_bh(&rmnet_shs_ep_lock);
	list_for_each_entry_safe(ep, tmp, &rmnet_shs_wq_ep_tbl, ep_list_id) {
		if (ep->ep == dev){
			ep->is_ep_active = 0;
			rmnet_shs_ep_tbl_remove(ep);
			ep->ep_list_id.next = NULL;
			ep->ep_list_id.prev = NULL;
			kfree(ep);
			break;
		}
	}

	spin_unlock_bh(&rmnet_shs_ep_lock);
}

void rmnet_shs_wq_set_ep_active(struct net_device *dev)
{
	struct rmnet_shs_wq_ep_s *ep = NULL;

	if (!dev) {
		rmnet_shs_crit_err[RMNET_SHS_NETDEV_ERR]++;
		return;
	}

	spin_lock_bh(&rmnet_shs_ep_lock);

	ep = kzalloc(sizeof(*ep), GFP_ATOMIC);

	if (!ep) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_ALLOC_EP_TBL_ERR]++;
		spin_unlock_bh(&rmnet_shs_ep_lock);
		return;
	}
	ep->ep = dev;
	ep->is_ep_active = 1;

	INIT_LIST_HEAD(&ep->ep_list_id);
	rmnet_shs_wq_update_ep_rps_msk(ep);
	rmnet_shs_ep_tbl_add(ep);

	spin_unlock_bh(&rmnet_shs_ep_lock);
}

void rmnet_shs_wq_refresh_ep_masks(void)
{
	struct rmnet_shs_wq_ep_s *ep = NULL;

	list_for_each_entry(ep, &rmnet_shs_wq_ep_tbl, ep_list_id) {
		if (!ep->is_ep_active)
			continue;
		rmnet_shs_wq_update_ep_rps_msk(ep);

		/* These tput totals get re-added as we go through each flow */
		ep->udp_rx_bps = 0;
		ep->tcp_rx_bps = 0;

	}
}

noinline void rmnet_shs_wq_filter(void)
{
	int cpu, cur_cpu;
	int temp;
	struct rmnet_shs_wq_hstat_s *hnode = NULL;

	for (cpu = 0; cpu < MAX_CPUS; cpu++) {
		rmnet_shs_cpu_rx_filter_flows[cpu] = 0;
		rmnet_shs_cpu_node_tbl[cpu].seg = 0;
	}

	rcu_read_lock();
	/* Filter out flows with low pkt count and
	 * mark CPUS with slowstart flows
	 */
	list_for_each_entry_rcu(hnode, &rmnet_shs_wq_hstat_tbl, hstat_node_id) {

		if (hnode->in_use == 0)
			continue;
		if (hnode->avg_pps > RMNET_SHS_FILTER_FLOW_RATE &&
		    hnode->rx_skb > RMNET_SHS_FILTER_PKT_LIMIT)
			if (hnode->current_cpu < MAX_CPUS) {
				temp = hnode->current_cpu;
				rmnet_shs_cpu_rx_filter_flows[temp]++;
			}
		cur_cpu = hnode->current_cpu;
		if (cur_cpu >= MAX_CPUS || cur_cpu < 0) {
			continue;
		}

		if (hnode->segs_per_skb > 0) {
			rmnet_shs_cpu_node_tbl[cur_cpu].seg++;
		}
	}
	rcu_read_unlock();

}

void rmnet_shs_wq_update_stats(void)
{
	struct timespec64 time;
	struct rmnet_shs_wq_hstat_s *hnode = NULL;

	(void) ktime_get_boottime_ts64(&time);
	rmnet_shs_wq_tnsec = RMNET_SHS_SEC_TO_NSEC(time.tv_sec) + time.tv_nsec;
	rmnet_shs_wq_refresh_ep_masks();
	rmnet_shs_update_cfg_mask();
	rmnet_update_reserve_mask();
	rmnet_shs_get_state();


	if ((rmnet_shs_wq_tick & SYNC_TIME) == SYNC_TIME)
		rmnet_shs_phy_sync();

	rcu_read_lock();
	list_for_each_entry_rcu(hnode, &rmnet_shs_wq_hstat_tbl, hstat_node_id) {

		if (hnode->in_use == 0)
			continue;

		if (hnode->node) {
			rmnet_shs_wq_update_hash_stats(hnode);
			rmnet_shs_wq_update_cpu_rx_tbl(hnode);

			if (rmnet_shs_userspace_connected) {
                /* Low latency flows added here */
				if (hnode->node->low_latency) {
					rmnet_shs_wq_ll_flow_list_add(hnode, &ll_flows);
				} else {
					/* Filters can only be installed if dont-fragment is set */
					rmnet_shs_wq_fflow_list_add(hnode, &fflows);
				}
				if ((rmnet_shs_cfg.feature_mask  & SILVER_BALANCE_FEAT) ||
				    !rmnet_shs_is_lpwr_cpu(hnode->current_cpu)) {
					/* Add golds flows to list */
					rmnet_shs_wq_gflow_list_add(hnode, &gflows);
				}
				if (hnode->skb_tport_proto == IPPROTO_TCP) {
					rmnet_shs_wq_ssflow_list_add(hnode, &ssflows);
				}
			} else {
				/* Disable segmentation if userspace gets disconnected connected */
				hnode->node->hstats->segs_per_skb = 0;
			}
		}
	}
	rcu_read_unlock();

	rmnet_shs_wq_refresh_all_cpu_stats();
	rmnet_shs_wq_refresh_total_stats();
	rmnet_shs_wq_refresh_dl_mrkr_stats();

	if (rmnet_shs_userspace_connected) {
		rm_err("%s", "SHS_UPDATE: Userspace connected, relying on userspace evaluation");
		rmnet_shs_wq_eval_cpus_caps_and_flows(&cpu_caps, &gflows, &ssflows, &fflows, &ll_flows);
		rmnet_shs_wq_cleanup_gold_flow_list(&gflows);
		rmnet_shs_wq_cleanup_ss_flow_list(&ssflows);
		rmnet_shs_wq_cleanup_cpu_caps_list(&cpu_caps);
		rmnet_shs_wq_cleanup_fflow_list(&fflows);
		rmnet_shs_wq_cleanup_ll_flow_list(&ll_flows);
	} else {
		rm_err("%s", "SHS_UPDATE: shs userspace not connected, using default logic");
		rmnet_shs_wq_eval_suggested_cpu();
	}
	rmnet_shs_wq_refresh_new_flow_list();
	rmnet_shs_wq_filter();
}

void rmnet_shs_wq_process_wq(struct work_struct *work)
{
	unsigned long jiffies;

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_PROCESS_WQ,
				RMNET_SHS_WQ_PROCESS_WQ_START,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	rmnet_shs_wq_tick++;
	spin_lock_bh(&rmnet_shs_ep_lock);
	rmnet_shs_wq_update_stats();
	spin_unlock_bh(&rmnet_shs_ep_lock);

	/*Invoke after both the locks are released*/
	rmnet_shs_wq_cleanup_hash_tbl(PERIODIC_CLEAN);
	rmnet_shs_wq_debug_print_flows();

	jiffies = msecs_to_jiffies(rmnet_shs_wq_interval_ms);

	queue_delayed_work(rmnet_shs_wq, &rmnet_shs_delayed_wq->wq,
			   jiffies);

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_PROCESS_WQ,
				RMNET_SHS_WQ_PROCESS_WQ_END,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
}

void rmnet_shs_wq_clean_ep_tbl(void)
{
	struct rmnet_shs_wq_ep_s *ep = NULL;
	struct list_head *ptr = NULL, *next = NULL;

	list_for_each_safe(ptr, next, &rmnet_shs_wq_ep_tbl) {
		ep = list_entry(ptr, struct rmnet_shs_wq_ep_s, ep_list_id);

		trace_rmnet_shs_wq_high(RMNET_SHS_WQ_EP_TBL,
					RMNET_SHS_WQ_EP_TBL_CLEANUP,
					0xDEF, 0xDEF, 0xDEF, 0xDEF, ep, NULL);

		rmnet_shs_ep_tbl_remove(ep);
		ep->ep_list_id.next = NULL;
		ep->ep_list_id.prev = NULL;
		kfree(ep);
	}
}

void rmnet_shs_wq_exit(void)
{
	/*If Wq is not initialized, nothing to cleanup */
	if (!rmnet_shs_wq || !rmnet_shs_delayed_wq)
		return;

	rmnet_shs_wq_mem_deinit();
	rmnet_shs_genl_send_int_to_userspace_no_info(RMNET_SHS_SYNC_WQ_EXIT);

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_EXIT, RMNET_SHS_WQ_EXIT_START,
				   0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	rmnet_shs_wq_pause();

	cancel_delayed_work_sync(&rmnet_shs_delayed_wq->wq);
	drain_workqueue(rmnet_shs_wq);
	destroy_workqueue(rmnet_shs_wq);
	kfree(rmnet_shs_delayed_wq);

	rmnet_shs_delayed_wq = NULL;
	rmnet_shs_wq = NULL;
	rmnet_shs_wq_cleanup_hash_tbl(FORCE_CLEAN);
	rmnet_shs_wq_clean_ep_tbl();
	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_EXIT, RMNET_SHS_WQ_EXIT_END,
				   0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
}

void rmnet_shs_wq_init_cpu_rx_flow_tbl(void)
{
	u8 cpu_num;
	struct rmnet_shs_wq_cpu_rx_pkt_q_s *rx_flow_tbl_p;

	for (cpu_num = 0; cpu_num < MAX_CPUS; cpu_num++) {

		trace_rmnet_shs_wq_high(RMNET_SHS_WQ_CPU_HSTAT_TBL,
					RMNET_SHS_WQ_CPU_HSTAT_TBL_INIT,
					cpu_num, 0xDEF, 0xDEF, 0xDEF,
					NULL, NULL);

		rx_flow_tbl_p = &rmnet_shs_rx_flow_tbl.cpu_list[cpu_num];
		INIT_LIST_HEAD(&rx_flow_tbl_p->hstat_id);
		rx_flow_tbl_p->cpu_num = cpu_num;
	}

}

void rmnet_shs_wq_pause(void)
{
	int cpu;
	struct rmnet_shs_msg_resp msg;


	rmnet_shs_pause_count++;

	if (rmnet_shs_wq && rmnet_shs_delayed_wq)
		cancel_delayed_work_sync(&rmnet_shs_delayed_wq->wq);


	for (cpu = 0; cpu < MAX_CPUS; cpu++)
		rmnet_shs_cpu_rx_filter_flows[cpu] = 0;

	/* If shsusrd fails to reset phy do it on a pause
	 * Reset switch pointer if set for some reason
	 * There is a brief window where follwing can happen.
	 * SHS could go flat when suggestion is made tcpu = x, rss = high
	 * If rps isn't moved i.e rps = 1, tcpu = prio, rss = high
         * We can just reset tcpu to 1 and NULL, rss.
	 * If rss is low then change took place and we need to set rss high again.
	 * This should cover the case when that occurs and correct phy and pointer.
	 *
	 * Check core 1 is not in the reserve mask
	 */
	rcu_read_lock();
	if ((rmnet_shs_cfg.phy_acpu != DEF_PHY_CPU) && ((1 << DEF_PHY_CPU ) & ~rmnet_shs_halt_mask)) {
		rmnet_shs_cfg.phy_tcpu = DEF_PHY_CPU;
		rmnet_shs_switch_enable();
		rmnet_shs_switch_reason[RMNET_SHS_WQ_FAIL_PHY_DROP]++;
	 }
	rcu_read_unlock();

	/* Create boost msg and deliver using direct msg channel to shsusrd */
	rmnet_shs_create_pause_msg_resp(0, &msg);
	rmnet_shs_genl_msg_direct_send_to_userspace(&msg);
}

void rmnet_shs_wq_restart(void)
{
	rmnet_shs_restart_count++;

	/* Estimation is off if restart is immediate */
	if (rmnet_shs_wq && rmnet_shs_delayed_wq)
		queue_delayed_work(rmnet_shs_wq, &rmnet_shs_delayed_wq->wq, 0);
}

void rmnet_shs_wq_init(void)
{
	/*If the workqueue is already initialized we should not be
	 *initializing again
	 */
	if (rmnet_shs_wq)
		return;

	rmnet_shs_wq_mem_init();

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_INIT, RMNET_SHS_WQ_INIT_START,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
	rmnet_shs_wq = alloc_workqueue("rmnet_shs_wq", WQ_UNBOUND, 1);
	if (!rmnet_shs_wq) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_ALLOC_WQ_ERR]++;
		return;
	}

	rmnet_shs_delayed_wq = kmalloc(sizeof(struct rmnet_shs_delay_wq_s),
				       GFP_ATOMIC);

	if (!rmnet_shs_delayed_wq) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_ALLOC_DEL_WQ_ERR]++;
		rmnet_shs_wq_exit();
		return;
	}

	/*All hstat nodes allocated during Wq init will be held for ever*/
	rmnet_shs_wq_hstat_alloc_nodes(RMNET_SHS_MIN_HSTAT_NODES_REQD, 1);
	rmnet_shs_wq_init_cpu_rx_flow_tbl();
	INIT_DELAYED_WORK(&rmnet_shs_delayed_wq->wq,
			     rmnet_shs_wq_process_wq);

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_INIT, RMNET_SHS_WQ_INIT_END,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
}

int rmnet_shs_wq_get_num_cpu_flows(u16 cpu)
{
	int flows = -1;

	if (cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_INVALID_CPU_ERR]++;
		return flows;
	}
	flows = rmnet_shs_rx_flow_tbl.cpu_list[cpu].flows;

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
			       RMNET_SHS_WQ_CPU_STATS_GET_CPU_FLOW,
				cpu, flows, 0xDEF, 0xDEF, NULL, NULL);

	return flows;
}

int rmnet_shs_wq_get_max_flows_per_core(void)
{
	u16 cpu;
	int max_flows = -1;
	int cpu_flows;

	for (cpu = 0; cpu < MAX_CPUS; cpu++) {
		cpu_flows = rmnet_shs_wq_get_num_cpu_flows(cpu);
		if (cpu_flows > max_flows)
			max_flows = cpu_flows;

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
			       RMNET_SHS_WQ_CPU_STATS_GET_MAX_CPU_FLOW,
				cpu, cpu_flows, max_flows, 0xDEF, NULL, NULL);
	}

	return max_flows;
}

int rmnet_shs_wq_get_max_flows_per_cluster(u16 cpu)
{
	u32 big_cluster_mask = 1<<4;
	u32 core_mask = 1;
	u16 start_core = 0;
	u16 end_core = 4;
	int max_flows = -1;
	int cpu_flows;

	if (cpu > MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_INVALID_CPU_ERR]++;
		return max_flows;
	}

	core_mask <<= cpu;
	if (core_mask >= big_cluster_mask) {
		start_core = 4;
		end_core = MAX_CPUS;
	}

	for ( ; start_core < end_core; start_core++) {
		cpu_flows = rmnet_shs_wq_get_num_cpu_flows(start_core);
		if (cpu_flows > max_flows)
			max_flows = cpu_flows;
	}

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
			       RMNET_SHS_WQ_CPU_STATS_MAX_FLOW_IN_CLUSTER,
			       start_core, end_core, cpu, max_flows,
			       NULL, NULL);
	return max_flows;
}

void rmnet_shs_wq_inc_cpu_flow(u16 cpu)
{
	if (cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return;
	}

	rmnet_shs_rx_flow_tbl.cpu_list[cpu].flows++;

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
			       RMNET_SHS_WQ_CPU_STATS_INC_CPU_FLOW,
			       cpu, rmnet_shs_rx_flow_tbl.cpu_list[cpu].flows,
			       0xDEF, 0xDEF, NULL, NULL);
}

void rmnet_shs_wq_dec_cpu_flow(u16 cpu)
{
	if (cpu >= MAX_CPUS) {
		rmnet_shs_crit_err[RMNET_SHS_WQ_INVALID_CPU_ERR]++;
		return;
	}

	if (rmnet_shs_rx_flow_tbl.cpu_list[cpu].flows > 0)
		rmnet_shs_rx_flow_tbl.cpu_list[cpu].flows--;

	trace_rmnet_shs_wq_low(RMNET_SHS_WQ_CPU_STATS,
			       RMNET_SHS_WQ_CPU_STATS_DEC_CPU_FLOW,
			       cpu, rmnet_shs_rx_flow_tbl.cpu_list[cpu].flows,
			       0xDEF, 0xDEF, NULL, NULL);
}
