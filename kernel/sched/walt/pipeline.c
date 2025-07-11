// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "walt.h"
#include "trace.h"


static DEFINE_RAW_SPINLOCK(pipeline_lock);
static struct walt_task_struct *pipeline_wts[WALT_NR_CPUS];
int pipeline_nr;

static DEFINE_RAW_SPINLOCK(heavy_lock);
static struct walt_task_struct *heavy_wts[MAX_NR_PIPELINE];
bool pipeline_pinning;

static inline int pipeline_demand(struct walt_task_struct *wts)
{
	return scale_time_to_util(wts->coloc_demand);
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

int remove_heavy(struct walt_task_struct *wts)
{
	int i, j, ret = 0;
	unsigned long flags;

	if (unlikely(walt_disabled))
		return -EAGAIN;

	raw_spin_lock_irqsave(&heavy_lock, flags);

	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		if (wts == heavy_wts[i]) {
			wts->low_latency &= ~WALT_LOW_LATENCY_HEAVY_BIT;
			heavy_wts[i] = NULL;
			have_heavy_list--;
			for (j = i; j < MAX_NR_PIPELINE - 1; j++) {
				heavy_wts[j] = heavy_wts[j + 1];
				heavy_wts[j + 1] = NULL;
			}
			goto out;
		}
	}
out:
	raw_spin_unlock_irqrestore(&heavy_lock, flags);
	return ret;
}

void remove_special_task(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&heavy_lock, flags);
	/*
	 * Although the pipeline special task designation is removed,
	 * if the task is not dead (i.e. this function was called from sysctl context)
	 * the task will continue to enjoy pipeline priveleges until the next update in
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

		for_each_sched_cluster(cluster) {
			if (cpumask_intersects(&cpus_for_pipeline, &cluster->cpus) ||
			    is_max_possible_cluster_cpu(cpumask_first(&cluster->cpus)))
				core_ctl_set_cluster_boost(cluster->id, false);
		}
	} else if (!unisolation_state && enable_pipeline_unisolation) {
		unisolation_state = true;

		for_each_sched_cluster(cluster) {
			if (cpumask_intersects(&cpus_for_pipeline, &cluster->cpus) ||
			    is_max_possible_cluster_cpu(cpumask_first(&cluster->cpus)))
				core_ctl_set_cluster_boost(cluster->id, true);
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

static inline bool should_pipeline_pin_special(void)
{
	if (!pipeline_special_task)
		return false;
	if (!heavy_wts[MAX_NR_PIPELINE - 1])
		return false;
	if (pipeline_demand(heavy_wts[0]) <= sysctl_pipeline_special_task_util_thres)
		return true;
	if (pipeline_demand(heavy_wts[1]) <= sysctl_pipeline_non_special_task_util_thres)
		return true;
	if (pipeline_pinning && (pipeline_demand(heavy_wts[0]) <=
		mult_frac(pipeline_demand(heavy_wts[1]), sysctl_pipeline_pin_thres_low_pct, 100)))
		return false;
	if (!pipeline_pinning && (pipeline_demand(heavy_wts[0]) <=
		mult_frac(pipeline_demand(heavy_wts[1]), sysctl_pipeline_pin_thres_high_pct, 100)))
		return false;

	return true;
}

cpumask_t last_available_big_cpus = CPU_MASK_NONE;
int have_heavy_list;
u32 total_util;
bool find_heaviest_topapp(u64 window_start)
{
	struct walt_related_thread_group *grp;
	struct walt_task_struct *wts;
	unsigned long flags;
	static u64 last_rearrange_ns;
	int i, j, start;
	struct walt_task_struct *heavy_wts_to_drop[MAX_NR_PIPELINE];

	if (num_sched_clusters < 2)
		return false;

	/* lazy enabling disabling until 100mS for colocation or heavy_nr change */
	grp = lookup_related_thread_group(DEFAULT_CGROUP_COLOC_ID);
	if (!grp || (!sysctl_sched_heavy_nr && !sysctl_sched_pipeline_util_thres) ||
		sched_boost_type) {
		if (have_heavy_list) {
			raw_spin_lock_irqsave(&heavy_lock, flags);
			for (i = 0; i < MAX_NR_PIPELINE; i++) {
				if (heavy_wts[i]) {
					heavy_wts[i]->low_latency &= ~WALT_LOW_LATENCY_HEAVY_BIT;
					heavy_wts[i]->pipeline_cpu = -1;
					heavy_wts[i] = NULL;
				}
			}
			raw_spin_unlock_irqrestore(&heavy_lock, flags);
			have_heavy_list = 0;

			pipeline_set_unisolation(false, AUTO_PIPELINE);
		}
		return false;
	}

	if (last_rearrange_ns && (window_start < (last_rearrange_ns + 100 * MSEC_TO_NSEC)))
		return false;
	last_rearrange_ns = window_start;

	raw_spin_lock_irqsave(&grp->lock, flags);
	raw_spin_lock(&heavy_lock);

	/* remember the old ones in _to_drop[] */
	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		heavy_wts_to_drop[i] = heavy_wts[i];
		heavy_wts[i] = NULL;
	}

	/* Assign user specified one (if exists) to slot 0*/
	if (pipeline_special_task) {
		heavy_wts[0] = (struct walt_task_struct *)
					pipeline_special_task->android_vendor_data1;
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

		/* if the task hasnt seen action recently skip it */
		if (wts->mark_start < window_start - (sched_ravg_window * 2))
			continue;

		/* skip user defined task as it's already part of the list*/
		if (pipeline_special_task && (wts == heavy_wts[0]))
			continue;

		for (i = start; i < MAX_NR_PIPELINE; i++) {
			if (!heavy_wts[i]) {
				heavy_wts[i] = to_be_placed_wts;
				break;
			} else if (pipeline_demand(to_be_placed_wts) >=
					pipeline_demand(heavy_wts[i])) {
				struct walt_task_struct *tmp;

				tmp = heavy_wts[i];
				heavy_wts[i] = to_be_placed_wts;
				to_be_placed_wts = tmp;
			}
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
			if (heavy_wts[i])
				total_util += pipeline_demand(heavy_wts[i]);
		}

		if (total_util < sysctl_sched_pipeline_util_thres)
			heavy_wts[MAX_NR_PIPELINE - 1] = NULL;
	}

	/* reset heavy for tasks that are no longer heavy */
	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		bool reset = true;

		if (!heavy_wts_to_drop[i])
			continue;
		for (j = 0; j < MAX_NR_PIPELINE; j++) {
			if (!heavy_wts[j])
				continue;
			if (heavy_wts_to_drop[i] == heavy_wts[j]) {
				reset = false;
				break;
			}
		}
		if (reset) {
			heavy_wts_to_drop[i]->low_latency &= ~WALT_LOW_LATENCY_HEAVY_BIT;
			heavy_wts_to_drop[i]->pipeline_cpu = -1;
		}

		if (heavy_wts[i])
			heavy_wts[i]->low_latency |= WALT_LOW_LATENCY_HEAVY_BIT;
	}

	if (heavy_wts[MAX_NR_PIPELINE - 1])
		pipeline_set_unisolation(true, AUTO_PIPELINE);
	else
		pipeline_set_unisolation(false, AUTO_PIPELINE);

	raw_spin_unlock(&heavy_lock);
	raw_spin_unlock_irqrestore(&grp->lock, flags);
	return true;
}

void assign_heaviest_topapp(bool found_topapp)
{
	int i;
	struct walt_task_struct *wts;

	if (!found_topapp)
		return;

	raw_spin_lock(&heavy_lock);

	/* start with non-prime cpus chosen for this chipset (e.g. golds) */
	cpumask_and(&last_available_big_cpus, cpu_online_mask, &cpus_for_pipeline);
	cpumask_andnot(&last_available_big_cpus, &last_available_big_cpus, cpu_halt_mask);

	/*
	 * Ensure the special task is only pinned if there are 3 auto pipeline tasks and
	 * check certain demand conditions between special pipeline task and the largest
	 * non-special pipeline task.
	 */
	if (should_pipeline_pin_special()) {
		pipeline_pinning = true;
		heavy_wts[0]->pipeline_cpu =
			cpumask_last(&sched_cluster[num_sched_clusters - 1]->cpus);
		heavy_wts[0]->low_latency |= WALT_LOW_LATENCY_HEAVY_BIT;
		if (cpumask_test_cpu(heavy_wts[0]->pipeline_cpu, &last_available_big_cpus))
			cpumask_clear_cpu(heavy_wts[0]->pipeline_cpu, &last_available_big_cpus);
	} else {
		pipeline_pinning = false;
	}

	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		wts = heavy_wts[i];
		if (!wts)
			continue;

		if (i == 0 && pipeline_pinning)
			continue;

		if (wts->pipeline_cpu != -1) {
			if (cpumask_test_cpu(wts->pipeline_cpu, &last_available_big_cpus))
				cpumask_clear_cpu(wts->pipeline_cpu, &last_available_big_cpus);
			else
				/* avoid assigning two pipelines to same cpu */
				wts->pipeline_cpu = -1;
		}
	}

	have_heavy_list = 0;
	/* assign cpus and heavy status to the new heavy */
	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		wts = heavy_wts[i];
		if (!wts)
			continue;

		if (wts->pipeline_cpu == -1) {
			wts->pipeline_cpu = cpumask_last(&last_available_big_cpus);
			if (wts->pipeline_cpu >= nr_cpu_ids) {
				/* drop from heavy if it can't be assigned */
				heavy_wts[i]->low_latency &= ~WALT_LOW_LATENCY_HEAVY_BIT;
				heavy_wts[i]->pipeline_cpu = -1;
				heavy_wts[i] = NULL;
			} else {
				/*
				 * clear cpu from the avalilable list of pipeline cpus.
				 * as pipeline_cpu is assigned for the task.
				 */
				cpumask_clear_cpu(wts->pipeline_cpu, &last_available_big_cpus);
			}
		}
		if (wts->pipeline_cpu >= 0)
			have_heavy_list++;
	}

	if (trace_sched_pipeline_tasks_enabled()) {
		for (i = 0; i < MAX_NR_PIPELINE; i++) {
			if (heavy_wts[i] != NULL)
				trace_sched_pipeline_tasks(AUTO_PIPELINE, i, heavy_wts[i],
						have_heavy_list, total_util, pipeline_pinning);
		}
	}

	raw_spin_unlock(&heavy_lock);
}
static inline void swap_pipeline_with_prime_locked(struct walt_task_struct *prime_wts,
						   struct walt_task_struct *other_wts)
{
	if (prime_wts && other_wts) {
		if (pipeline_demand(prime_wts) < pipeline_demand(other_wts)) {
			int cpu;

			cpu = other_wts->pipeline_cpu;
			other_wts->pipeline_cpu = prime_wts->pipeline_cpu;
			prime_wts->pipeline_cpu = cpu;
			trace_sched_pipeline_swapped(other_wts, prime_wts);
		}
	} else if (!prime_wts && other_wts) {
		/* if prime preferred died promote gold to prime, assumes 1 prime */
		other_wts->pipeline_cpu =
			cpumask_last(&sched_cluster[num_sched_clusters - 1]->cpus);
		trace_sched_pipeline_swapped(other_wts, prime_wts);
	}
}

#define WINDOW_HYSTERESIS 4
static inline bool delay_rearrange(u64 window_start, int pipeline_type, bool force)
{
	static u64 last_rearrange_ns[MAX_PIPELINE_TYPES];

	if (!force && last_rearrange_ns[pipeline_type] &&
			(window_start < (last_rearrange_ns[pipeline_type] +
			(sched_ravg_window*WINDOW_HYSTERESIS))))
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

	for (i = 0; i < MAX_NR_PIPELINE; i++) {
		struct walt_task_struct *wts = wts_list[i];

		if (wts == NULL)
			continue;

		if (wts->pipeline_cpu < 0)
			continue;

		if (is_max_possible_cluster_cpu(wts->pipeline_cpu)) {
			if (prime_wts)
				*prime_wts = wts;
		} else if (other_wts && pipeline_demand(wts) > max_demand) {
			max_demand = pipeline_demand(wts);
			*other_wts = wts;
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
	unsigned long flags;

	if (!pipeline_in_progress())
		return;

	if (num_sched_clusters < 2)
		return;

	raw_spin_lock_irqsave(&heavy_lock, flags);
	/*
	 * TODO: As primes are isolated under have_heavy_list < 3, and pipeline misfits are also
	 * disabled, setting the prime worthy task's pipeline_cpu as CPU7 could lead to the
	 * pipeline_cpu selection being ignored until the next run of find_heaviest_toppapp(),
	 * and furthermore remove the task's current gold pipeline_cpu, which could cause the
	 * task to start bouncing around on the golds, and ultimately lead to suboptimal behavior.
	 */
	if (have_heavy_list <= 2) {
		find_prime_and_max_tasks(heavy_wts, &prime_wts, &other_wts);

		if (prime_wts && !is_prime_worthy(prime_wts)) {
			int assign_cpu;

			/* demote prime_wts, it is not worthy */
			assign_cpu = cpumask_first(&last_available_big_cpus);
			if (assign_cpu < nr_cpu_ids) {
				prime_wts->pipeline_cpu = assign_cpu;
				cpumask_clear_cpu(assign_cpu, &last_available_big_cpus);
				prime_wts = NULL;
			}
			/* if no pipeline cpu available to assign, leave task on prime */
		}

		if (!prime_wts && is_prime_worthy(other_wts)) {
			/* promote other_wts to prime, it is worthy */
			swap_pipeline_with_prime_locked(NULL, other_wts);
		}

		goto out;
	}

	if (pipeline_pinning)
		goto out;

	if (delay_rearrange(window_start, AUTO_PIPELINE, force))
		goto out;

	if (!soc_feat(SOC_ENABLE_PIPELINE_SWAPPING_BIT) && !force)
		goto out;

	/* swap prime for have_heavy_list >= 3 */
	find_prime_and_max_tasks(heavy_wts, &prime_wts, &other_wts);
	swap_pipeline_with_prime_locked(prime_wts, other_wts);

out:
	raw_spin_unlock_irqrestore(&heavy_lock, flags);
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
		 * assummes that if one pipeline doesn't have preferred set,
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
			} else if (pipeline_demand(wts) > max_demand) {
				max_demand = pipeline_demand(wts);
				other_wts = wts;
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
						pipeline_nr, 0, 0);
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

bool pipeline_check(struct walt_rq *wrq)
{
	/* found_topapp should force rearrangement */
	bool found_topapp = find_heaviest_topapp(wrq->window_start);

	rearrange_pipeline_preferred_cpus(wrq->window_start);
	pipeline_reset_unisolation_state();

	return found_topapp;
}

void pipeline_rearrange(struct walt_rq *wrq, bool found_topapp)
{
	assign_heaviest_topapp(found_topapp);
	rearrange_heavy(wrq->window_start, found_topapp);
}

bool enable_load_sync(int cpu)
{
	if (!cpumask_test_cpu(cpu, &pipeline_sync_cpus))
		return false;

	if (!pipeline_in_progress())
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
 * If the task is assigned a pipeline CPU which is a prime CPU, ret should be 0, indicating
 * the task is a misfit.
 * If the number of pipeline tasks is 2 or fewer, continue evaluation of task_fits_max().
 * If the number of pipeline tasks is 3 or more, ret should be 1, indicating the task fits on the
 * smaller CPUs and is not a misfit.
 */
int pipeline_fits_smaller_cpus(struct task_struct *p)
{
	struct walt_task_struct *wts = (struct walt_task_struct *) p->android_vendor_data1;
	unsigned int pipeline_cpu = wts->pipeline_cpu;

	if (pipeline_cpu == -1)
		return -1;

	if (cpumask_test_cpu(pipeline_cpu, &cpu_array[0][num_sched_clusters-1]))
		return 0;

	if (have_heavy_list) {
		if (have_heavy_list == MAX_NR_PIPELINE)
			return 1;
		else
			return -1;
	}

	if (pipeline_nr >= MAX_NR_PIPELINE)
		return 1;
	else
		return -1;
}
