/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTE: This file has been modified by Sony Mobile Communications AB.
 * Modifications are licensed under the License.
 */

#define DEBUG_LOWMEMORYKILLER

#ifdef CONFIG_NUMA
#error "Not for NUMA machines"
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/swap.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/ktime.h>

static uint32_t lowmem_debug_level = 1;

static int lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static int lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};

static int lowmem_minfree_size = 4;

/* Uses the minfree array for lowmem/zone normal thresholds after dividing
 * with zone_normal_minfree_ratio */
static const int zone_normal_minfree_ratio = 4;

/* not used, but still here because it is exposed as a parameter */
static int lmk_fast_run = 1;

static ktime_t lowmem_deathpending_timeout;

#define LMK_BUSY (-1)

#define TRIGGER_NO (-1)
#define TRIGGER_ALL_MEM 0
#define TRIGGER_LOW_MEM 1

#ifdef DEBUG_LOWMEMORYKILLER
#define lowmem_debug_print lowmem_print
#else
#define lowmem_debug_print(level, x...)
/* Intentionally empty */
#endif

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			printk(x);			\
	} while (0)


static int test_task_flag(struct task_struct *p, int flag)
{
	struct task_struct *t = p;

	do {
		task_lock(t);
		if (test_tsk_thread_flag(t, flag)) {
			task_unlock(t);
			return 1;
		}
		task_unlock(t);
	} while_each_thread(p, t);

	return 0;
}



static DEFINE_MUTEX(scan_mutex);

int can_use_cma_pages(gfp_t gfp_mask)
{
	int can_use = 0;
	int mtype = allocflags_to_migratetype(gfp_mask);
	int i = 0;
	int *mtype_fallbacks = get_migratetype_fallbacks(mtype);

	if (is_migrate_cma(mtype)) {
		can_use = 1;
	} else {
		for (i = 0;; i++) {
			int fallbacktype = mtype_fallbacks[i];

			if (is_migrate_cma(fallbacktype)) {
				can_use = 1;
				break;
			}

			if (fallbacktype == MIGRATE_RESERVE)
				break;
		}
	}
	return can_use;
}

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	int rem = 0;
	static int same_count;
	static int oldpid;
	static int lastpid;
	int tasksize;
	int i;
	int min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int selected_tasksize = 0;
	int selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free;
	int other_file;
	unsigned long nr_to_scan = sc->nr_to_scan;
	int zone_normal_free;
	int zone_normal_file;
	struct zone *zone;
	struct zonelist *zonelist;
	int use_cma_pages;
	int reason = TRIGGER_NO;
	int lowfree;

	if (nr_to_scan > 0) {
		if (mutex_lock_interruptible(&scan_mutex) < 0)
			return LMK_BUSY;
	}
	lowmem_print(2, "lowmem_shrink() nr_to_scan: %lu by pid: %d (%s)\n",
		     nr_to_scan, current->pid, current->comm);

	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);

	use_cma_pages = can_use_cma_pages(sc->gfp_mask);

	other_free = global_page_state(NR_FREE_PAGES);
	if (!use_cma_pages) {
		other_free -= global_page_state(NR_FREE_CMA_PAGES);
		other_free = max(0, other_free);
	}

	/* conservative estimate of file cache size */
	other_file = global_page_state(NR_FILE_PAGES);
	other_file -= max(global_page_state(NR_FILE_MAPPED),
			  global_page_state(NR_SHMEM));
	other_file = max(0, other_file);

	/* UMA: Only one node (0) and flags don't matter */
	zonelist = node_zonelist(0, 0);
	first_zones_zonelist(zonelist, ZONE_NORMAL, NULL, &zone);

	zone_normal_free = zone_page_state(zone, NR_FREE_PAGES);
	if (!use_cma_pages) {
		zone_normal_free -= zone_page_state(zone, NR_FREE_CMA_PAGES);
		zone_normal_free = max(0, zone_normal_free);
	}

	/* conservative estimate of file cache size */
	zone_normal_file = zone_page_state(zone, NR_FILE_PAGES);
	zone_normal_file -= max(zone_page_state(zone, NR_FILE_MAPPED),
				zone_page_state(zone, NR_SHMEM));
	zone_normal_file = max(0, zone_normal_file);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

	for (i = 0; i < array_size; i++) {
		lowmem_debug_print(3, "lowmem_minfree[i]: %d for lowmem: %d\n",
				   lowmem_minfree[i],
				   lowmem_minfree[i]/zone_normal_minfree_ratio);

		if (other_free + other_file < lowmem_minfree[i]) {
			min_score_adj = lowmem_adj[i];
			reason = TRIGGER_ALL_MEM;
			break;
		}

		lowfree = (zone_normal_free + zone_normal_file) *
			zone_normal_minfree_ratio;
		if (lowfree < lowmem_minfree[i]) {
			min_score_adj = lowmem_adj[i];
			reason = TRIGGER_LOW_MEM;
			break;
		}
	}

	lowmem_debug_print(2, "Free memory: %d (free: %d filecache: %d)\n",
			   other_free + other_file, other_free, other_file);
	lowmem_debug_print(2, "Free lowmem: %d (free: %d filecache: %d)\n",
			   zone_normal_free + zone_normal_file,
			   zone_normal_free, zone_normal_file);
	lowmem_debug_print(2, "min_score_adj: %d\n", min_score_adj);

	if (nr_to_scan > 0)
		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %d\n",
				nr_to_scan, sc->gfp_mask, other_free,
				other_file, min_score_adj);

	if (min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		/* Tell shrinker framework we don't have any memory to free */
		rem = 0;
		lowmem_print(5, "lowmem_shrink %lu, %x, return %d\n",
			     nr_to_scan, sc->gfp_mask, rem);

		if (nr_to_scan > 0)
			mutex_unlock(&scan_mutex);

		return rem;
	}
	selected_oom_score_adj = min_score_adj;

	/* if only queried, return "cache size" */
	if (nr_to_scan <= 0)
		return rem;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		int oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		/* if task no longer has any memory ignore it */
		if (test_task_flag(tsk, TIF_MM_RELEASED))
			continue;

		if (ktime_us_delta(ktime_get(), lowmem_deathpending_timeout) < 0
		    && (test_task_flag(tsk, TIF_MEMDIE))) {
			same_count++;

			lowmem_debug_print(2, "PID:%d already dying. state:%ld"\
					   " exit_state: %d flags: %d\n",
					   tsk->pid, tsk->state,
					   tsk->exit_state, tsk->flags);

			if (tsk->pid != oldpid || same_count > 1000) {
				lowmem_print(1, "terminate loop for pid:%d(%s)"\
					     " oldpid:%d lastpid:%d delta:%ld "\
					     " same_count: %d\n",
					tsk->pid,
					tsk->comm,
					oldpid,
					lastpid,
					(long)ktime_us_delta(
						ktime_get(),
						lowmem_deathpending_timeout),
					same_count);
				lowmem_print(2,
					"state:%ld flag:0x%x "\
					     "oom_killer_disabled: %d\n",
					tsk->state,
					tsk->flags,
					oom_killer_disabled);
				oldpid = tsk->pid;
				same_count = 0;
			}

			rcu_read_unlock();
			mutex_unlock(&scan_mutex);

			/* wait one jiffie */
			schedule_timeout_interruptible(1);

			lowmem_debug_print(3, "LMK_BUSY\n");
			return LMK_BUSY;
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;
		lowmem_debug_print(4, "pid: %d (%s) oom_score_adj: %d\n",
				   tsk->pid, tsk->comm, oom_score_adj);

		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}
		tasksize = get_mm_rss(p->mm);
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(4, "select %d (%s), adj %d, size %d, to kill\n",
			     p->pid, p->comm, oom_score_adj, tasksize);
	}
	if (selected) {
		send_sig(SIGKILL, selected, 0);
		lowmem_print(1, "send sigkill to %d (%s), adj %d, size %d,"\
			     " from %d (%s), trigger: %d\n",
			     selected->pid, selected->comm,
			     selected_oom_score_adj, selected_tasksize,
			     current->pid, current->comm, reason);

		lowmem_deathpending_timeout = ktime_add_ns(ktime_get(),
							   NSEC_PER_SEC/2);
		lowmem_print(2, "state:%ld flag:0x%x %d\n",
			     selected->state, selected->flags,
			     oom_killer_disabled);
		lastpid = selected->pid;
		set_tsk_thread_flag(selected, TIF_MEMDIE);
		rem -= selected_tasksize;
	}
	rcu_read_unlock();
	lowmem_print(4, "lowmem_shrink %lu, %x, return %d\n",
		     nr_to_scan, sc->gfp_mask, rem);
	mutex_unlock(&scan_mutex);

	/* we killed something, give the system some time */
	if (selected && current_is_kswapd())
		schedule_timeout_interruptible(1);

	return rem;
}

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
	register_shrinker(&lowmem_shrinker);
	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static int lowmem_oom_adj_to_oom_score_adj(int oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	int oom_adj;
	int oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_int,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
__module_param_call(MODULE_PARAM_PREFIX, adj,
		    &lowmem_adj_array_ops,
		    .arr = &__param_arr_adj,
		    S_IRUGO | S_IWUSR, -1);
__MODULE_PARM_TYPE(adj, "array of int");
#else
module_param_array_named(adj, lowmem_adj, int, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(lmk_fast_run, lmk_fast_run, int, S_IRUGO | S_IWUSR);

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

