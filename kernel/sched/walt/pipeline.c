// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/sort.h>
#include "walt.h"
#include "trace.h"


static DEFINE_RAW_SPINLOCK(pipeline_lock);
static struct walt_task_struct *pipeline_wts[WALT_NR_CPUS];
int pipeline_nr;
static DEFINE_RAW_SPINLOCK(heavy_lock);
static struct walt_task_struct *heavy_wts[MAX_NR_PIPELINE];
unsigned int pipeline_swap_util_th;
cpumask_t available_gold_cpus = CPU_MASK_NONE;
cpumask_t available_prime_cpus = CPU_MASK_NONE;
int have_heavy_list;
u32 total_util;
u32 least_pipeline_demand;
static bool top_wts_bias;

#define IPC_DEGRADATION_FACTOR 115
#define PIPELINE_2L_FACTOR 95
#define REARRANGE_HYST_MS	100ULL
#define PIPELINE_ENERGY_BAND 6
#define FIND_HEAVY_FAIL -1
#define FIND_HEAVY_WAIT 0
#define FIND_HEAVY_SUCCESS 1
#define CONFIG1 1
#define CONFIG2 2

void pipeline_demand(struct walt_task_struct *wts, u64 *scaled_gold_demand,
		     u64 *scaled_prime_demand)
{
	u64 util =  scale_time_to_util(wts->coloc_demand);
	int cpu = task_cpu(wts_to_ts(wts));

	if (demand_scaling_factor == 100) {
		*scaled_prime_demand = util;
		*scaled_gold_demand = util;
		return;
	}

	/*
	 * TODO:
	 * Assume that a task not on prime is on golds.
	 * This will need to be revisited for a non 2-cluster system.
	 */
	if (cpumask_test_cpu(cpu,  &sched_cluster[num_sched_clusters - 1]->cpus)) {
		*scaled_prime_demand = util;
		*scaled_gold_demand = mult_frac(util, 100, demand_scaling_factor);
	} else {
		*scaled_gold_demand = util;
		*scaled_prime_demand = mult_frac(util, demand_scaling_factor, 100);
	}
}

int add_pipeline(struct walt_task_struct *wts)
{
	int i, pos = -1, ret = -ENOSPC;
	unsigned long flags;
	int max_nr_pipeline = cpumask_weight(&cpus_for_pipeline);

	if (unlikely(walt_disabled))
		return -EAGAIN;

	raw_spin_lock_irqsave(&pipeline_lock, flags);

	for (i = 0; i < max_nr_pipeline; i++) {
		if (wts == pipeline_wts[i]) {
			ret = 0;
			goto out;
		}

		if (pipeline_wts[i] == NULL)
			pos = i;
	}

	if (pos != -1) {
		pipeline_wts[pos] = wts;
		pipeline_nr++;
		ret = 0;
	}
out:
	raw_spin_unlock_irqrestore(&pipeline_lock, flags);
	return ret;
}

int remove_pipeline(struct walt_task_struct *wts)
{
	int i, j, ret = 0;
	unsigned long flags;

	if (unlikely(walt_disabled))
		return -EAGAIN;

	raw_spin_lock_irqsave(&pipeline_lock, flags);

	for (i = 0; i < WALT_NR_CPUS; i++) {
		if (wts == pipeline_wts[i]) {
			wts->low_latency &= ~WALT_LOW_LATENCY_PIPELINE_BIT;
			pipeline_wts[i] = NULL;
			pipeline_nr--;
			for (j = i; j < WALT_NR_CPUS - 1; j++) {
				pipeline_wts[j] = pipeline_wts[j + 1];
				pipeline_wts[j + 1] = NULL;
			}
			goto out;
		}
	}
out:
	raw_spin_unlock_irqrestore(&pipeline_lock, flags);
	return ret;
}

void remove_heavy(struct walt_task_struct *wts)
{
	int i, j;
	unsigned long flags;

	if (unlikely(walt_disabled))
		return;

	raw_spin_lock_irqsave(&heavy_lock, flags);
	if (!(wts->low_latency & WALT_LOW_LATENCY_HEAVY_BIT))
		goto out;

	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		if (wts == heavy_wts[i]) {
			wts->low_latency &= ~WALT_LOW_LATENCY_HEAVY_BIT;
			if (wts->pipeline_cpu > -1)
				have_heavy_list--;
			heavy_wts[i] = NULL;
			for (j = i; j < MAX_NR_PIPELINE - 1; j++) {
				heavy_wts[j] = heavy_wts[j + 1];
				heavy_wts[j + 1] = NULL;
			}
			goto out;
		}
	}
out:
	raw_spin_unlock_irqrestore(&heavy_lock, flags);
}

void remove_special_task(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&heavy_lock, flags);
	/*
	 * Although the pipeline special task designation is removed,
	 * if the task is not dead (i.e. this function was called from sysctl context)
	 * the task will continue to enjoy pipeline privileges until the next update in
	 * find_heaviest_topapp()
	 */
	pipeline_special_task = NULL;
	raw_spin_unlock_irqrestore(&heavy_lock, flags);
}

void set_special_task(struct task_struct *pipeline_special_local)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&heavy_lock, flags);
	pipeline_special_task = pipeline_special_local;
	raw_spin_unlock_irqrestore(&heavy_lock, flags);
}

cpumask_t cpus_for_pipeline = { CPU_BITS_NONE };

/* always set unisolation for max cluster, for pipeline tasks */
/*
 * TODO:
 * This will be refactored later to directly use pipeline cpus
 * to identify the clusters to be isolated/unisolated, that way
 * we are independent of need_cpus/not_preferred configurations
 * of core control (for 2 heavy task we are dependent on the
 * configuration).
 * The call to this can be moved after pipeline cpu assignment.
 */
static inline void pipeline_set_unisolation(bool set, int flag)
{
	static bool unisolation_state;
	struct walt_sched_cluster *cluster;
	static unsigned int enable_pipeline_unisolation;

	if (!set)
		enable_pipeline_unisolation &= ~(1 << flag);
	else
		enable_pipeline_unisolation |= (1 << flag);

	if (unisolation_state && !enable_pipeline_unisolation) {
		unisolation_state = false;

		if (sysctl_single_thread_pipeline) {
			core_ctl_set_cluster_boost(
				cpu_cluster(cpumask_last(&cpus_for_pipeline))->id, false);
		} else {
			for_each_sched_cluster(cluster) {
				if (cpumask_intersects(&cpus_for_pipeline, &cluster->cpus) ||
				    is_max_possible_cluster_cpu(cpumask_first(&cluster->cpus)))
					core_ctl_set_cluster_boost(cluster->id, false);
			}
		}
	} else if (!unisolation_state && enable_pipeline_unisolation) {
		unisolation_state = true;

		if (sysctl_single_thread_pipeline) {
			core_ctl_set_cluster_boost(
				cpu_cluster(cpumask_last(&cpus_for_pipeline))->id, true);
		} else {
			for_each_sched_cluster(cluster) {
				if (cpumask_intersects(&cpus_for_pipeline, &cluster->cpus) ||
				    is_max_possible_cluster_cpu(cpumask_first(&cluster->cpus)))
					core_ctl_set_cluster_boost(cluster->id, true);
			}
		}
	}
}

/*
 * sysctl_sched_heavy_nr or sysctl_sched_pipeline_util_thres can change at any moment in time.
 * as a result, the ability to set/clear unisolation state for a particular type of pipeline, is
 * hindered. Detect a transition and reset the unisolation state of the pipeline method no longer
 * in use.
 */
static inline void pipeline_reset_unisolation_state(void)
{
	static bool last_auto_pipeline;

	if ((sysctl_sched_heavy_nr || sysctl_sched_pipeline_util_thres) && !last_auto_pipeline) {
		pipeline_set_unisolation(false, MANUAL_PIPELINE);
		last_auto_pipeline = true;
	} else if (!sysctl_sched_heavy_nr &&
			!sysctl_sched_pipeline_util_thres && last_auto_pipeline) {
		pipeline_set_unisolation(false, AUTO_PIPELINE);
		last_auto_pipeline = false;
	}
}

static inline bool special_pipeline_thres_valid(void)
{
	struct walt_task_struct *special_wts = (struct walt_task_struct *)
			android_task_vendor_data(pipeline_special_task);
	u64 gold_demand, prime_demand;

	pipeline_demand(special_wts, &gold_demand, &prime_demand);
	if (gold_demand < sysctl_pipeline_special_task_util_thres)
		return true;

	return false;
}

static inline bool should_pipeline_pin_special(void)
{
	if (!pipeline_special_task)
		return false;

	if (!soc_feat(SOC_ENABLE_SINGLE_THREAD_PIPELINE_PINNING) &&
			!heavy_wts[MAX_NR_PIPELINE - 1])
		return false;

	return special_pipeline_thres_valid();
}

#define PIPELINE_BIAS_WINDOW_SIZE 5
int load_cmp_func(const void *tsk1, const void *tsk2)
{
	struct walt_task_struct **wts1 = (struct walt_task_struct **)tsk1;
	struct walt_task_struct **wts2 = (struct walt_task_struct **)tsk2;
	u64 gold_demand1, gold_demand2, prime_demand;

	if (unlikely(!wts1 || !wts2 || !*wts1 || !*wts2))
		return -1;


	pipeline_demand(*wts1, &gold_demand1, &prime_demand);
	pipeline_demand(*wts2, &gold_demand2, &prime_demand);

	return gold_demand2 - (gold_demand1 + pipeline_swap_util_th);
}

#define DEFAULT_PENALTY			10
#define DEFAULT_BONUS			5
#define LST_LOW_DEMAND_PENALTY		5
#define MAX_ACTIVITY_CNT		250
#define LST_PIPELINE_SKIP_THRESHOLD	25
int find_heaviest_topapp(u64 window_start)
{
	struct walt_related_thread_group *grp;
	struct walt_task_struct *wts;
	unsigned long flags;
	static u64 last_rearrange_ns;
	u64 rearrange_target_ns = 0;
	int i, j, start, sort_start, delta = 0;
	struct walt_task_struct *heavy_wts_to_drop[MAX_NR_PIPELINE];
	u64 gold_demand_heavy = 0, prime_demand_heavy = 0;
	static struct walt_task_struct *top_wts;
	static int top_wts_count;
	bool top_wts_miss = false;
	int nr_pipeline_cnt = 0;
	int nr_prime_cpu = cpumask_weight(&sched_cluster[prime_cluster_id]->cpus);

	if (num_sched_clusters < 2)
		return FIND_HEAVY_FAIL;

	/* lazy enabling disabling until 100mS for colocation or heavy_nr change */
	grp = lookup_related_thread_group(DEFAULT_CGROUP_COLOC_ID);
	if (!grp || (!sysctl_sched_heavy_nr && !sysctl_sched_pipeline_util_thres) ||
		!cpumask_weight(&cpus_for_pipeline) || sched_boost_type) {
		if (have_heavy_list) {
			raw_spin_lock_irqsave(&heavy_lock, flags);
			for (i = 0; i < MAX_NR_PIPELINE; i++) {
				if (heavy_wts[i]) {
					heavy_wts[i]->low_latency &= ~WALT_LOW_LATENCY_HEAVY_BIT;
					heavy_wts[i]->pipeline_cpu = -1;
					heavy_wts[i] = NULL;
				}
			}
			have_heavy_list = 0;
			raw_spin_unlock_irqrestore(&heavy_lock, flags);

			pipeline_set_unisolation(false, AUTO_PIPELINE);
		}
		last_rearrange_ns = window_start;
		least_pipeline_demand = 0;
		return FIND_HEAVY_FAIL;
	}

	if (likely(heavy_wts[0]))
		rearrange_target_ns = last_rearrange_ns +
				((u64)sysctl_pipeline_rearrange_delay_ms[0] * MSEC_TO_NSEC);
	else
		rearrange_target_ns = last_rearrange_ns + (250ULL * MSEC_TO_NSEC);

	if (last_rearrange_ns && (window_start < rearrange_target_ns))
		return FIND_HEAVY_WAIT;

	last_rearrange_ns = window_start;
	raw_spin_lock_irqsave(&grp->lock, flags);
	raw_spin_lock(&heavy_lock);

	/* remember the old ones in _to_drop[] */
	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		heavy_wts_to_drop[i] = heavy_wts[i];
		heavy_wts[i] = NULL;
	}

	/* Assign user specified one (if exists) to slot 0*/
	if (pipeline_special_task && special_pipeline_thres_valid()) {
		heavy_wts[0] = (struct walt_task_struct *)android_task_vendor_data(
				pipeline_special_task);
		start = 1;
	} else {
		start = 0;
	}

	/*
	 * Ensure that heavy_wts either contains the top 3 top-app tasks,
	 * or the user defined heavy task followed by the top 2 top-app tasks
	 */
	list_for_each_entry(wts, &grp->tasks, grp_list) {
		struct walt_task_struct *to_be_placed_wts = wts;
		unsigned int win_cnt = 0;
		int penalty = 0;
		u64 gold_demand_to_be, prime_demand_to_be;

		pipeline_demand(to_be_placed_wts, &gold_demand_to_be, &prime_demand_to_be);
		if (have_heavy_list)
			win_cnt = atomic_read(&to_be_placed_wts->event_windows);

		atomic_set(&to_be_placed_wts->event_windows, 0);

		/*
		 * Apply a penalty of DEFAULT_PENALTY to "to_be_placed_wts", assuming it won't
		 * be selected as a pipeline task.
		 * If it is later selected as a pipeline task, this penalty will be removed.
		 */
		if (to_be_placed_wts->pipeline_cpu == -1)
			to_be_placed_wts->pipeline_activity_cnt =
				max(to_be_placed_wts->pipeline_activity_cnt - DEFAULT_PENALTY, 0);

		/*
		 * Penalty is applied on the tasks which have less demand and
		 * were active for less than 4 windows.
		 */
		if ((gold_demand_to_be < min_demand_for_activity_cnt) && (win_cnt < 4)) {
			to_be_placed_wts->pipeline_activity_cnt =
					max(to_be_placed_wts->pipeline_activity_cnt -
									DEFAULT_PENALTY, 0);

			if (to_be_placed_wts->pipeline_cpu == -1)
				continue;
		}

		/*
		 * Calculate penalty for tasks:
		 * This is to ensure older pipeline task, which are currently not active gets
		 * filtered out from pipeline selection.
		 *
		 * If task is big (bigger than smallest pipeline task) then it's active
		 * window count is added to improve it's pipeline selection chances.
		 *
		 * If task is small in demand than the least heavy pipeline tasks then
		 * apply penalty of LST_LOW_DEMAND_PENALTY.
		 *
		 * If task is marked as LST add LST_LOW_DEMAND_PENALTY more to penalty.
		 */
		delta = gold_demand_to_be - least_pipeline_demand;
		if (delta >= 0) {
			if (gold_demand_to_be >= min_demand_for_activity_cnt)
				to_be_placed_wts->pipeline_activity_cnt += win_cnt;
		} else {
			penalty += LST_LOW_DEMAND_PENALTY;
		}

		if (to_be_placed_wts->lst)
			penalty += LST_LOW_DEMAND_PENALTY;

		to_be_placed_wts->pipeline_activity_cnt =
				max(to_be_placed_wts->pipeline_activity_cnt - penalty, 0);

		/*
		 * Ignore any LST task with either small pipeline count or task is not
		 * a pipeline task.
		 */
		if (to_be_placed_wts->lst &&
		    ((to_be_placed_wts->pipeline_activity_cnt < LST_PIPELINE_SKIP_THRESHOLD) ||
						(to_be_placed_wts->pipeline_cpu == -1))) {
			continue;
		}

		/* saturate pipeline count so that we have deterministic decay */
		if (to_be_placed_wts->pipeline_activity_cnt > MAX_ACTIVITY_CNT)
			to_be_placed_wts->pipeline_activity_cnt = MAX_ACTIVITY_CNT;

		/* skip user defined task as it's already part of the list */
		if (pipeline_special_task && (to_be_placed_wts == heavy_wts[0]))
			continue;

		/* skip recently forked thread, at least until next 2 evaluations */
		if ((to_be_placed_wts->mark_start_birth_ts > window_start) ||
			((window_start - to_be_placed_wts->mark_start_birth_ts) <
								(200ULL * MSEC_TO_NSEC)))
			continue;

		/* ensure the task can be placed on any of the potential pipeline cpus */
		if (!cpumask_subset(&cpus_for_pipeline, &(wts_to_ts(wts)->cpus_mask)))
			continue;

		/* skip tasks which are not active since last 2 windows */
		if (to_be_placed_wts->mark_start < window_start - (sched_ravg_window * 2))
			continue;

		/* evaluate task for pipeline based on the pipeline_activity_cnt */
		for (i = start; i < MAX_NR_PIPELINE; i++) {
			if (!heavy_wts[i]) {
				heavy_wts[i] = to_be_placed_wts;
				break;
			} else if (to_be_placed_wts->pipeline_activity_cnt >=
					heavy_wts[i]->pipeline_activity_cnt) {
				struct walt_task_struct *tmp;

				pipeline_demand(heavy_wts[i], &gold_demand_heavy,
						&prime_demand_heavy);
				pipeline_demand(to_be_placed_wts, &gold_demand_to_be,
						&prime_demand_to_be);

				if (to_be_placed_wts->pipeline_activity_cnt ==
							heavy_wts[i]->pipeline_activity_cnt) {
					/*
					 * When evaluating for T0, if the last T0 task has been T0
					 * for 5 or more evaluations, set the top_wts_bias. Once
					 * top_wts_bias has been set, ensure T0 remains at the top
					 * until top_wts_count has been lowered to 0.
					 */
					if (i == 0 && top_wts && top_wts_bias) {
						if (to_be_placed_wts == top_wts) {
							if (prime_demand_heavy >
								(prime_demand_to_be +
								pipeline_swap_util_th))
								top_wts_miss = true;
							prime_demand_heavy = 0;
						} else if (heavy_wts[i] == top_wts) {
							if (prime_demand_to_be >
								prime_demand_heavy)
								top_wts_miss = true;
							continue;
						}
					}

					if (prime_demand_to_be <= prime_demand_heavy)
						continue;
				}
				tmp = heavy_wts[i];
				heavy_wts[i] = to_be_placed_wts;
				to_be_placed_wts = tmp;
			}
		}
	}

	if (heavy_wts[0]) {
		if (heavy_wts[0] == top_wts) {
			if (top_wts_miss) {
				top_wts_count--;
				if (top_wts_count == 0 && top_wts_bias)
					top_wts_bias = false;
			} else {
				top_wts_count++;
				if (top_wts_count >= PIPELINE_BIAS_WINDOW_SIZE) {
					top_wts_count = PIPELINE_BIAS_WINDOW_SIZE;
					top_wts_bias = true;
				}
			}
		} else {
			top_wts = heavy_wts[0];
			top_wts_count = 1;
			top_wts_bias = false;
		}
	}

	/*
	 * Determine how many of the top three pipeline tasks
	 * If "sched_heavy_nr" node is set, the util threshold is ignored.
	 */
	total_util = 0;
	if (sysctl_sched_heavy_nr) {
		for (i = sysctl_sched_heavy_nr; i < MAX_NR_PIPELINE; i++)
			heavy_wts[i] = NULL;
	} else {
		for (i = 0; i < MAX_NR_PIPELINE; i++) {
			if (heavy_wts[i]) {
				pipeline_demand(heavy_wts[i], &gold_demand_heavy,
						&prime_demand_heavy);
				total_util += gold_demand_heavy;
			}
		}

		if (total_util < sysctl_sched_pipeline_util_thres)
			heavy_wts[MAX_NR_PIPELINE - 1] = NULL;
	}

	/* reset tasks that are no longer eligible for pipeline */
	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		bool reset = true;

		if (!heavy_wts_to_drop[i])
			continue;
		for (j = 0; j < MAX_NR_PIPELINE; j++) {
			if (!heavy_wts[j])
				continue;

			if (sysctl_single_thread_pipeline) {
				if (heavy_wts_to_drop[i] == heavy_wts[0]) {
					reset = false;
					break;
				}
			} else {
				if (heavy_wts_to_drop[i] == heavy_wts[j]) {
					reset = false;
					break;
				}
			}
		}
		if (reset) {
			heavy_wts_to_drop[i]->low_latency &= ~WALT_LOW_LATENCY_HEAVY_BIT;
			heavy_wts_to_drop[i]->pipeline_cpu = -1;
			heavy_wts_to_drop[i]->pipeline_activity_cnt =
				max(heavy_wts_to_drop[i]->pipeline_activity_cnt -
									DEFAULT_PENALTY, 0);
		}

	}

	/*
	 * Under single-thread pipeline scenario, it is possible that not all
	 * heavy tasks have WALT_LOW_LATENCY_HEAVY_BIT bit.  Ensure that all
	 * the heavy tasks have WALT_LOW_LATENCY_HEAVY_BIT set.
	 */
	least_pipeline_demand = INT_MAX;
	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		if (heavy_wts[i]) {
			nr_pipeline_cnt++;
			heavy_wts[i]->low_latency |= WALT_LOW_LATENCY_HEAVY_BIT;
			/*
			 * least_pipeline_demand: tracks smallest pipeline task, this is used
			 * for calculation of penalty during pipeline task selection.
			 */
			pipeline_demand(heavy_wts[i], &gold_demand_heavy, &prime_demand_heavy);
			/* pipeline selection count is only applied if task is big enough */
			if (gold_demand_heavy > min_demand_for_activity_cnt)
				heavy_wts[i]->pipeline_activity_cnt += DEFAULT_BONUS;
			if (gold_demand_heavy <= least_pipeline_demand)
				least_pipeline_demand = gold_demand_heavy;

		}
	}

	if (heavy_wts[MAX_NR_PIPELINE - 1] || sysctl_single_thread_pipeline)
		pipeline_set_unisolation(true, AUTO_PIPELINE);
	else
		pipeline_set_unisolation(false, AUTO_PIPELINE);

	/* sort heavy list based on demand */
	sort_start = start ? start : top_wts_bias ? start + 1 : start;
	if (((nr_pipeline_cnt - sort_start) > 1) && (!sort_start || (nr_prime_cpu > 1)))
		sort(&heavy_wts[sort_start], sort_start ? nr_pipeline_cnt - 1 : nr_pipeline_cnt,
			sizeof(heavy_wts[0]), load_cmp_func, NULL);

	raw_spin_unlock(&heavy_lock);
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	return FIND_HEAVY_SUCCESS;
}

static inline void swap_pipeline_with_prime_locked(struct walt_task_struct *prime_wts,
						   struct walt_task_struct *other_wts)
{
	u64 gold_demand_1 = 0, prime_demand_1 = 0, gold_demand_2 = 0, prime_demand_2 = 0;

	if (prime_wts && other_wts) {
		pipeline_demand(prime_wts, &gold_demand_1, &prime_demand_1);
		pipeline_demand(other_wts, &gold_demand_2, &prime_demand_2);
		if (prime_demand_1 < prime_demand_2) {
			int cpu;

			cpu = other_wts->pipeline_cpu;
			other_wts->pipeline_cpu = prime_wts->pipeline_cpu;
			prime_wts->pipeline_cpu = cpu;
			trace_sched_pipeline_swapped(other_wts, prime_wts);
		}
	} else if (!prime_wts && other_wts) {
		/* if prime preferred died promote gold to prime, assumes 1 prime */
		if (have_heavy_list) {
			int cpu = cpumask_first(&available_prime_cpus);

			if (cpu < nr_cpu_ids) {
				cpumask_clear_cpu(other_wts->pipeline_cpu,
						  &available_gold_cpus);
				other_wts->pipeline_cpu = cpu;
				cpumask_clear_cpu(cpu, &available_prime_cpus);
			}
		} else {
			other_wts->pipeline_cpu =
				cpumask_last(&sched_cluster[num_sched_clusters - 1]->cpus);
		}
		trace_sched_pipeline_swapped(other_wts, prime_wts);
	}
}

static inline bool delay_rearrange(u64 window_start, int pipeline_type, bool force)
{
	static u64 last_rearrange_ns[MAX_PIPELINE_TYPES];
	u64 next_rearrange = (u64)sysctl_pipeline_rearrange_delay_ms[1] * (u64)sched_ravg_window;

	if (!force && last_rearrange_ns[pipeline_type] &&
		(window_start < last_rearrange_ns[pipeline_type] + next_rearrange))
		return true;

	last_rearrange_ns[pipeline_type] = window_start;
	return false;
}

static inline void find_prime_and_max_tasks(struct walt_task_struct **wts_list,
					    struct walt_task_struct **prime_wts,
					    struct walt_task_struct **other_wts)
{
	int i;
	int max_demand = 0;
	u64 gold_demand = 0, prime_demand = 0;

	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		struct walt_task_struct *wts = wts_list[i];

		if (wts == NULL)
			continue;

		if (wts->pipeline_cpu < 0)
			continue;

		if (is_max_possible_cluster_cpu(wts->pipeline_cpu)) {
			if (prime_wts)
				*prime_wts = wts;
		} else {
			pipeline_demand(wts, &gold_demand, &prime_demand);
			if (other_wts && prime_demand > max_demand) {
				max_demand = prime_demand;
				*other_wts = wts;
			}
		}
	}
}

static inline bool is_prime_worthy(struct walt_task_struct *wts)
{
	struct task_struct *p;

	if (wts == NULL)
		return false;

	if (num_sched_clusters < 2)
		return true;

	p = wts_to_ts(wts);

	/*
	 * Assume the first row of cpu arrays represents the order of clusters
	 * in magnitude of capacities, where the last column represents prime,
	 * and the second to last column represents golds
	 */
	return !task_fits_max(p, cpumask_last(&cpu_array[0][num_sched_clusters - 2]));
}

void rearrange_heavy(u64 window_start, bool force)
{
	struct walt_task_struct *prime_wts = NULL;
	struct walt_task_struct *other_wts = NULL;
	bool prime_wts_fits_lower = true;
	bool rearranged = false;
	u64 primewts_prime_demand = 0, otherwts_prime_demand = 0, gold_demand = 0;

	if (!pipeline_in_progress())
		return;

	if (sysctl_single_thread_pipeline)
		return;

	if (num_sched_clusters < 2)
		return;

	/* Ensure that special pipeline task remains pinned if it is the only pipeline thread */
	if (should_pipeline_pin_special() && soc_feat(SOC_ENABLE_SINGLE_THREAD_PIPELINE_PINNING))
		return;
	/*
	 * TODO: As primes are isolated under have_heavy_list < 3, and pipeline misfits are also
	 * disabled, setting the prime worthy task's pipeline_cpu as CPU7 could lead to the
	 * pipeline_cpu selection being ignored until the next run of find_heaviest_toppapp(),
	 * and furthermore remove the task's current gold pipeline_cpu, which could cause the
	 * task to start bouncing around on the golds, and ultimately lead to suboptimal behavior.
	 */
	if (have_heavy_list <= 2) {
		for (int i = 0; i < MAX_NR_PIPELINE; i++) {
			int pipeline_cpu;
			int assign_cpu;

			if (!heavy_wts[i])
				continue;

			pipeline_cpu = heavy_wts[i]->pipeline_cpu;
			if (pipeline_cpu < 0) {
				assign_cpu = cpumask_first(&available_gold_cpus);
				if (assign_cpu < nr_cpu_ids) {
					heavy_wts[i]->pipeline_cpu = assign_cpu;
					cpumask_clear_cpu(assign_cpu, &available_gold_cpus);
				}
			} else if (cpumask_test_cpu(pipeline_cpu,
					&sched_cluster[prime_cluster_id]->cpus) &&
					!is_prime_worthy(heavy_wts[i])) {
				assign_cpu = cpumask_first(&available_gold_cpus);
				if (assign_cpu < nr_cpu_ids) {
					cpumask_set_cpu(pipeline_cpu, &available_prime_cpus);
					heavy_wts[i]->pipeline_cpu = assign_cpu;
					cpumask_clear_cpu(assign_cpu, &available_gold_cpus);
				}
			} else if (cpumask_test_cpu(pipeline_cpu,
					&sched_cluster[gold_cluster_id]->cpus) &&
					!rearranged &&
					is_prime_worthy(heavy_wts[i])) {
				assign_cpu = cpumask_first(&available_prime_cpus);
				if (assign_cpu < nr_cpu_ids) {
					cpumask_set_cpu(pipeline_cpu, &available_gold_cpus);
					heavy_wts[i]->pipeline_cpu = assign_cpu;
					cpumask_clear_cpu(assign_cpu, &available_prime_cpus);
					rearranged = true;
				}
			}
		}
		return;
	}

	/* Under pipeline pinning, do not swap for nr = 3*/
	if (should_pipeline_pin_special())
		return;

	if (delay_rearrange(window_start, AUTO_PIPELINE, force))
		return;

	if (!soc_feat(SOC_ENABLE_PIPELINE_SWAPPING_BIT) && !force)
		return;

	/* swap prime for have_heavy_list >= 3 */
	find_prime_and_max_tasks(heavy_wts, &prime_wts, &other_wts);

	if (prime_wts) {
		pipeline_demand(prime_wts, &gold_demand, &primewts_prime_demand);
		prime_wts_fits_lower = task_fits_capacity(wts_to_ts(prime_wts),
					cpumask_last(&cpu_array[0][num_sched_clusters - 2]));
	}
	if (other_wts)
		pipeline_demand(other_wts, &gold_demand, &otherwts_prime_demand);

	/*
	 * default behavior if pipeline_swap_util_th == 0 is to swap gold and prime pipeline
	 * tasks if task running on Gold has higher demand than prime.
	 * But if pipeline_swap_util_th > 0 then swap only under following condition (strict
	 * swapping):
	 * - if task on prime can fit AND
	 *		util for task on Gold > util of task on prime + pipeline_swap_util_th
	 */
	if (!pipeline_swap_util_th || (prime_wts_fits_lower &&
					((primewts_prime_demand + pipeline_swap_util_th) <
					 otherwts_prime_demand)))
		swap_pipeline_with_prime_locked(prime_wts, other_wts);
}

void pipeline_rearrange(struct walt_rq *wrq, int found_topapp)
{
	int i, gold_cpu, prime_cpu, cpu;
	u64 gold_energy, prime_energy;
	u64 config1, config2;
	u64 cur_cpu_util;
	u64 t0_util, t1_util, t2_util;
	u64 t0_gold, t1_gold, t2_gold, t0_prime, t1_prime, t2_prime;
	u64 non_pipeline_cluster_util[MAX_CLUSTERS];
	u64 non_pipeline_cluster_max_util[MAX_CLUSTERS];
	struct walt_sched_cluster *cluster;
	bool t0_is_prime = false, t1_is_prime = false, t2_is_prime = false;
	static int prev_config;
	u64 prev_energy = 0;

	if (found_topapp == FIND_HEAVY_FAIL)
		return;

	raw_spin_lock(&heavy_lock);

	if (found_topapp == FIND_HEAVY_SUCCESS && !sysctl_single_thread_pipeline) {
		cpumask_and(&available_gold_cpus, cpu_online_mask,
			    &sched_cluster[gold_cluster_id]->cpus);
		cpumask_and(&available_prime_cpus, cpu_online_mask,
			    &sched_cluster[prime_cluster_id]->cpus);
		cpumask_and(&available_gold_cpus, &available_gold_cpus, &cpus_for_pipeline);
		cpumask_and(&available_prime_cpus, &available_prime_cpus, &cpus_for_pipeline);
		cpumask_andnot(&available_gold_cpus, &available_gold_cpus, cpu_halt_mask);
		cpumask_andnot(&available_prime_cpus, &available_prime_cpus, cpu_halt_mask);

		have_heavy_list = 0;
		for (i = 0; i < MAX_NR_PIPELINE; i++) {
			if (heavy_wts[i]) {
				have_heavy_list++;
				if (heavy_wts[i]->pipeline_cpu != -1) {
					if (cpumask_test_cpu(heavy_wts[i]->pipeline_cpu,
							     &available_gold_cpus))
						cpumask_clear_cpu(heavy_wts[i]->pipeline_cpu,
								  &available_gold_cpus);
					else
						cpumask_clear_cpu(heavy_wts[i]->pipeline_cpu,
								  &available_prime_cpus);
				}
			}
		}
	}

	if (!heavy_wts[MAX_NR_PIPELINE - 1]) {
		rearrange_heavy(wrq->window_start, false);
		goto out;
	}

	if (found_topapp == FIND_HEAVY_WAIT)
		goto unlock;

	if (cpumask_weight(&sched_cluster[prime_cluster_id]->cpus) == 1)
		sysctl_pipeline_force_config = CONFIG1;

	/* force config skip EAS evaluations */
	if ((sysctl_pipeline_force_config == CONFIG1) || (sysctl_pipeline_force_config == CONFIG2))
		goto assign_pipeline_cpu;

	for_each_sched_cluster(cluster) {
		if (cluster->id != gold_cluster_id && cluster->id != prime_cluster_id)
			continue;

		non_pipeline_cluster_util[cluster->id] = 0;
		non_pipeline_cluster_max_util[cluster->id] = 0;

		for_each_cpu(cpu, &cluster->cpus) {
			if (cpumask_test_cpu(cpu, &cpus_for_pipeline))
				continue;
			wrq = &per_cpu(walt_rq, cpu);
			cur_cpu_util =  wrq->prev_runnable_sum +
				wrq->grp_time.prev_runnable_sum;
			cur_cpu_util = scale_time_to_util(cur_cpu_util);
			non_pipeline_cluster_util[cluster->id] += cur_cpu_util;
			non_pipeline_cluster_max_util[cluster->id] =
				max(non_pipeline_cluster_max_util[cluster->id], cur_cpu_util);
		}
	}

	gold_cpu = cpumask_first(&sched_cluster[gold_cluster_id]->cpus);
	prime_cpu = cpumask_first(&sched_cluster[prime_cluster_id]->cpus);

	/* Config #1 */
	pipeline_demand(heavy_wts[0], &t0_gold, &t0_prime);
	pipeline_demand(heavy_wts[1], &t1_gold, &t1_prime);
	pipeline_demand(heavy_wts[2], &t2_gold, &t2_prime);

	t0_util = t0_prime;
	t1_util = t1_gold;
	t2_util = t2_gold;

	gold_energy = walt_cpu_energy(gold_cpu,
				      max(max(t1_util, t2_util), non_pipeline_cluster_max_util[0]),
				      non_pipeline_cluster_util[0] + t1_util + t2_util);
	prime_energy = walt_cpu_energy(prime_cpu,
				       max(t0_util, non_pipeline_cluster_max_util[1]),
				       non_pipeline_cluster_util[1] + t0_util);
	config1 = gold_energy + prime_energy;

	/* Config #2 */
	if (heavy_wts[0]->pipeline_cpu >= 0 &&
	    cpumask_test_cpu(heavy_wts[0]->pipeline_cpu,
			     &sched_cluster[prime_cluster_id]->cpus))
		t0_is_prime = true;
	if (heavy_wts[1]->pipeline_cpu >= 0 &&
	    cpumask_test_cpu(heavy_wts[1]->pipeline_cpu,
			     &sched_cluster[prime_cluster_id]->cpus))
		t1_is_prime = true;
	if (heavy_wts[2]->pipeline_cpu >= 0 &&
	    cpumask_test_cpu(heavy_wts[2]->pipeline_cpu,
			     &sched_cluster[prime_cluster_id]->cpus))
		t2_is_prime = true;


	pipeline_demand(heavy_wts[0], &t0_gold, &t0_prime);
	pipeline_demand(heavy_wts[1], &t1_gold, &t1_prime);
	pipeline_demand(heavy_wts[2], &t2_gold, &t2_prime);

	t0_util = t0_prime;
	t1_util = t1_prime;
	t2_util = t2_gold;

	if (t0_is_prime && !t1_is_prime)
		t0_util = mult_frac(t0_util, IPC_DEGRADATION_FACTOR, 100);
	if (!t0_is_prime && t1_is_prime)
		t1_util = mult_frac(t1_util, IPC_DEGRADATION_FACTOR, 100);
	if (!t0_is_prime && !t1_is_prime)
		t0_util = mult_frac(t0_util, IPC_DEGRADATION_FACTOR, 100);


	gold_energy = walt_cpu_energy(gold_cpu,
				      max(t2_util, non_pipeline_cluster_max_util[0]),
				      non_pipeline_cluster_util[0] + t2_util);
	prime_energy = walt_cpu_energy(prime_cpu,
				       max(max(t0_util, t1_util), non_pipeline_cluster_max_util[1]),
				       non_pipeline_cluster_util[1] + t0_util + t1_util);

	prime_energy = mult_frac(prime_energy, PIPELINE_2L_FACTOR, 100);

	config2 = gold_energy + prime_energy;

	if (prev_config == CONFIG1)
		prev_energy = config1;
	else
		prev_energy = config2;
	prev_energy = mult_frac(prev_energy, 100 - PIPELINE_ENERGY_BAND, 100);
	if (!prev_energy)
		prev_energy = ULONG_MAX;

assign_pipeline_cpu:
	if ((((prev_config == CONFIG1 && prev_energy < config2) ||
	      (prev_config == CONFIG2 && config1 < prev_energy) ||
	      (!prev_config && config1 < config2) ||
	      should_pipeline_pin_special()) && !sysctl_pipeline_force_config) ||
	     (sysctl_pipeline_force_config == CONFIG1)) {
		if (heavy_wts[2]->pipeline_cpu != -1 &&
		    cpumask_test_cpu(heavy_wts[2]->pipeline_cpu,
				     &sched_cluster[prime_cluster_id]->cpus)) {
			cpumask_set_cpu(heavy_wts[2]->pipeline_cpu, &available_prime_cpus);
			heavy_wts[2]->pipeline_cpu = -1;
		}

		if (heavy_wts[1]->pipeline_cpu != -1 &&
		    cpumask_test_cpu(heavy_wts[1]->pipeline_cpu,
				     &sched_cluster[prime_cluster_id]->cpus)) {
			cpumask_set_cpu(heavy_wts[1]->pipeline_cpu, &available_prime_cpus);
			heavy_wts[1]->pipeline_cpu = -1;
		}

		if (heavy_wts[0]->pipeline_cpu != -1 &&
		    cpumask_test_cpu(heavy_wts[0]->pipeline_cpu,
				     &sched_cluster[gold_cluster_id]->cpus)) {
			cpumask_set_cpu(heavy_wts[0]->pipeline_cpu, &available_gold_cpus);
			heavy_wts[0]->pipeline_cpu = -1;
		}

		if (heavy_wts[2]->pipeline_cpu == -1) {
			heavy_wts[2]->pipeline_cpu = cpumask_first(&available_gold_cpus);
			cpumask_clear_cpu(heavy_wts[2]->pipeline_cpu, &available_gold_cpus);
		}

		if (heavy_wts[1]->pipeline_cpu == -1) {
			heavy_wts[1]->pipeline_cpu = cpumask_first(&available_gold_cpus);
			cpumask_clear_cpu(heavy_wts[1]->pipeline_cpu, &available_gold_cpus);
		}

		if (heavy_wts[0]->pipeline_cpu == -1) {
			heavy_wts[0]->pipeline_cpu = cpumask_first(&available_prime_cpus);
			cpumask_clear_cpu(heavy_wts[0]->pipeline_cpu, &available_prime_cpus);
		}

		prev_config = CONFIG1;
	} else {
		if (heavy_wts[2]->pipeline_cpu != -1 &&
		    cpumask_test_cpu(heavy_wts[2]->pipeline_cpu,
				     &sched_cluster[prime_cluster_id]->cpus)) {
			cpumask_set_cpu(heavy_wts[2]->pipeline_cpu, &available_prime_cpus);
			heavy_wts[2]->pipeline_cpu = -1;
		}

		if (heavy_wts[0]->pipeline_cpu != -1 &&
		    cpumask_test_cpu(heavy_wts[0]->pipeline_cpu,
				     &sched_cluster[gold_cluster_id]->cpus)) {
			cpumask_set_cpu(heavy_wts[0]->pipeline_cpu, &available_gold_cpus);
			heavy_wts[0]->pipeline_cpu = -1;
		}

		if (heavy_wts[1]->pipeline_cpu != -1 &&
		    cpumask_test_cpu(heavy_wts[1]->pipeline_cpu,
				     &sched_cluster[gold_cluster_id]->cpus)) {
			cpumask_set_cpu(heavy_wts[1]->pipeline_cpu, &available_gold_cpus);
			heavy_wts[1]->pipeline_cpu = -1;
		}

		if (heavy_wts[2]->pipeline_cpu == -1) {
			heavy_wts[2]->pipeline_cpu = cpumask_first(&available_gold_cpus);
			cpumask_clear_cpu(heavy_wts[2]->pipeline_cpu, &available_gold_cpus);
		}

		if (heavy_wts[0]->pipeline_cpu == -1) {
			heavy_wts[0]->pipeline_cpu = cpumask_first(&available_prime_cpus);
			cpumask_clear_cpu(heavy_wts[0]->pipeline_cpu, &available_prime_cpus);
		}

		if (heavy_wts[1]->pipeline_cpu == -1) {
			heavy_wts[1]->pipeline_cpu = cpumask_first(&available_prime_cpus);
			cpumask_clear_cpu(heavy_wts[1]->pipeline_cpu, &available_prime_cpus);
		}

		prev_config = CONFIG2;
	}

out:
	if (trace_sched_pipeline_tasks_enabled()) {
		for (i = 0; i < MAX_NR_PIPELINE; i++) {
			if (heavy_wts[i])
				trace_sched_pipeline_tasks(AUTO_PIPELINE, i, heavy_wts[i],
							   have_heavy_list, total_util,
							  should_pipeline_pin_special(),
							  (have_heavy_list == MAX_NR_PIPELINE) ?
							  prev_config : 0, top_wts_bias);
		}
	}

unlock:
	raw_spin_unlock(&heavy_lock);
}

void rearrange_pipeline_preferred_cpus(u64 window_start)
{
	unsigned long flags;
	struct walt_task_struct *wts;
	bool set_unisolation = false;
	u32 max_demand = 0;
	struct walt_task_struct *prime_wts = NULL;
	struct walt_task_struct *other_wts = NULL;
	static int assign_cpu = -1;
	static bool last_set_unisolation;
	int i;
	u64 gold_demand = 0, prime_demand = 0;

	if (sysctl_sched_heavy_nr || sysctl_sched_pipeline_util_thres)
		return;

	if (num_sched_clusters < 2)
		return;

	if (!pipeline_nr || sched_boost_type)
		goto out;

	if (delay_rearrange(window_start, MANUAL_PIPELINE, false))
		goto out;

	raw_spin_lock_irqsave(&pipeline_lock, flags);

	set_unisolation = true;

	for (i = 0; i < WALT_NR_CPUS; i++) {
		wts = pipeline_wts[i];

		if (!wts)
			continue;

		if (!wts->grp)
			wts->pipeline_cpu = -1;

		/*
		 * assumes that if one pipeline doesn't have preferred set,
		 * all pipelines too do not have it set
		 */
		if (wts->pipeline_cpu == -1) {
			assign_cpu = cpumask_next_and(assign_cpu,
						&cpus_for_pipeline, cpu_online_mask);

			if (assign_cpu >= nr_cpu_ids)
				/* reset and rotate the cpus */
				assign_cpu = cpumask_next_and(-1,
						&cpus_for_pipeline, cpu_online_mask);

			if (assign_cpu >= nr_cpu_ids)
				wts->pipeline_cpu = -1;
			else
				wts->pipeline_cpu = assign_cpu;
		}

		if (wts->pipeline_cpu != -1) {
			if (is_max_possible_cluster_cpu(wts->pipeline_cpu)) {
				/* assumes just one prime */
				prime_wts = wts;
			} else {
				pipeline_demand(wts, &gold_demand, &prime_demand);
				if (prime_demand > max_demand) {
					max_demand = prime_demand;
					other_wts = wts;
				}
			}
		}
	}

	if (pipeline_nr <= 2) {
		set_unisolation = false;
		if (prime_wts && !is_prime_worthy(prime_wts)) {
			/* demote prime_wts, it is not worthy */
			assign_cpu = cpumask_next_and(assign_cpu,
						&cpus_for_pipeline, cpu_online_mask);
			if (assign_cpu >= nr_cpu_ids)
				/* reset and rotate the cpus */
				assign_cpu = cpumask_next_and(-1,
							&cpus_for_pipeline, cpu_online_mask);
			if (assign_cpu >= nr_cpu_ids)
				prime_wts->pipeline_cpu = -1;
			else
				prime_wts->pipeline_cpu = assign_cpu;
			prime_wts = NULL;
		}

		if (!prime_wts && is_prime_worthy(other_wts)) {
			/* promote other_wts to prime, it is worthy */
			swap_pipeline_with_prime_locked(NULL, other_wts);
			set_unisolation = true;
		}

		if (prime_wts)
			set_unisolation = true;

		goto release_lock;
	}

	/* swap prime for nr_piprline >= 3 */
	swap_pipeline_with_prime_locked(prime_wts, other_wts);

	if (trace_sched_pipeline_tasks_enabled()) {
		for (i = 0; i < WALT_NR_CPUS; i++) {
			if (pipeline_wts[i] != NULL)
				trace_sched_pipeline_tasks(MANUAL_PIPELINE, i, pipeline_wts[i],
						pipeline_nr, 0, 0, 0, 0);
		}
	}

release_lock:
	raw_spin_unlock_irqrestore(&pipeline_lock, flags);

out:
	if (set_unisolation ^ last_set_unisolation) {
		pipeline_set_unisolation(set_unisolation, MANUAL_PIPELINE);
		last_set_unisolation = set_unisolation;
	}
}

int pipeline_check(struct walt_rq *wrq)
{
	/* found_topapp should force rearrangement */
	int found_topapp = find_heaviest_topapp(wrq->window_start);

	rearrange_pipeline_preferred_cpus(wrq->window_start);
	pipeline_reset_unisolation_state();

	return found_topapp;
}

bool enable_load_sync(int cpu)
{
	if (!cpumask_test_cpu(cpu, &pipeline_sync_cpus))
		return false;

	if (!pipeline_in_progress())
		return false;

	if (sysctl_single_thread_pipeline)
		return false;

	/*
	 * Under manual pipeline, only load sync between the pipeline_sync_cpus, if at least one
	 * of the CPUs userspace has allocated for pipeline tasks corresponds to the
	 * pipeline_sync_cpus
	 */
	if (!sysctl_sched_heavy_nr && !sysctl_sched_pipeline_util_thres &&
			!cpumask_intersects(&pipeline_sync_cpus, &cpus_for_pipeline))
		return false;

	/* Ensure to load sync only if there are 3 auto pipeline tasks */
	if (have_heavy_list)
		return have_heavy_list == MAX_NR_PIPELINE;

	/*
	 * If auto pipeline is disabled, manual must be on. Ensure to load sync under manual
	 * pipeline only if there are 3 or more pipeline tasks
	 */
	return pipeline_nr >= MAX_NR_PIPELINE;
}

/*
 * pipeline_fits_smaller_cpus evaluates if a pipeline task should be treated as a misfit.
 * There are three possible outcomes:
 *	- ret -1: Continue evaluation with task_fits_max().
 *      - ret  0: Task should be treated as a misfit (does not fit on smaller CPUs).
 *      - ret  1: Task cannot be treated as a misfit (fits on smaller CPUs).
 *
 * If the number of pipeline tasks is 2 or fewer, continue evaluation of task_fits_max().
 * If the number of pipeline tasks is 3 or more:
 *	a) If the task is assigned a pipeline CPU which is a prime CPU, ret should be 0,
 *	indicating the task is a misfit.
 *	b) else, ret should be 1, indicating the task fits on the smaller CPUs and is not a
 *	misfit.
 */
int pipeline_fits_smaller_cpus(struct task_struct *p)
{
	struct walt_task_struct *wts = (struct walt_task_struct *)android_task_vendor_data(p);
	int pipeline_cpu = wts->pipeline_cpu;

	if (!pipeline_in_progress())
		return -1;

	if (pipeline_cpu == -1)
		return -1;

	if (have_heavy_list) {
		if (have_heavy_list < MAX_NR_PIPELINE)
			return -1;
	} else {
		if (pipeline_nr < MAX_NR_PIPELINE)
			return -1;
	}

	if (cpumask_test_cpu(pipeline_cpu, &cpu_array[0][num_sched_clusters - 1]))
		return 0;

	return 1;
}

/*
 * single thread pipeline, is to enable pipeline affinity for only one task
 * while the other two heavy task gets benefit of MVP status and avoid
 * pinning, this allows task to be placed on CPU based on energy evaluations.
 */
void walt_configure_single_thread_pipeline(unsigned int val)
{
	unsigned long flags;
	int i;

	raw_spin_lock_irqsave(&heavy_lock, flags);

	if (val) {
		for (i = 1; i < MAX_NR_PIPELINE; i++) {
			struct walt_task_struct *wts = heavy_wts[i];

			if (wts) {
				if (wts->pipeline_cpu > -1)
					have_heavy_list--;

				wts->pipeline_cpu = -1;
			}
		}
	}

	pipeline_set_unisolation(false, AUTO_PIPELINE);
	sysctl_single_thread_pipeline = val;

	/*
	 * only unisolate if we are enabling ST pipeline otherwise
	 * let AUTOPIPELINE flow to decide the core control.
	 */
	if (val && (heavy_wts[0] && is_max_possible_cluster_cpu(cpumask_last(&cpus_for_pipeline))))
		pipeline_set_unisolation(true, AUTO_PIPELINE);


	raw_spin_unlock_irqrestore(&heavy_lock, flags);
}
