/*
 *  lowmemorykiller_psi
 *
 *  Author: Peter Enderborg <peter.enderborg@sonymobile.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/* add fake print format with original module name */
#define pr_fmt(fmt) "lowmemorykiller: " fmt

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched/loadavg.h>
#include <linux/psi.h>
#include <linux/swap.h>

#include <trace/events/lmk.h>

#include "lowmemorykiller.h"
#include "lowmemorykiller_tng.h"
#include "lowmemorykiller_stats.h"
#include "lowmemorykiller_tasks.h"

static void *tng_private; /* local storage of rcu updated pointer within psi functions */
static struct psi_trigger *tng_trigger;

static void balance_psi(short score)
{
	struct task_struct *selected = NULL;
	struct lmk_rb_watch *lrw;
	struct calculated_params cp;
	gfp_t mask = ___GFP_KSWAPD_RECLAIM |
	  ___GFP_DIRECT_RECLAIM | __GFP_FS | __GFP_IO | __GFP_ATOMIC;
	struct zone *zones = NODE_DATA(node)->node_zones;
	long zone_reserved = zones[ZONE_NORMAL].watermark[WMARK_LOW];

	mask &= ~__GFP_MOVABLE;
	cp.selected_tasksize = 0;
	cp.dynamic_max_queue_len = 3;
	cp.kill_reason = LMK_PSI;
	cp.margin = 0;
	/* other_free not used for calculations but are printed on the kill */
	cp.other_free = zone_page_state(&zones[ZONE_NORMAL], NR_FREE_PAGES) -
		zone_reserved;
	/* other_file not used for calculations but are printed on the kill */
	cp.other_file = global_node_page_state(NR_FILE_PAGES) -
		global_node_page_state(NR_SHMEM) -
		global_node_page_state(NR_UNEVICTABLE) -
		total_swapcache_pages();
	cp.min_score_adj = score;
	cp.minfree = 0;

	spin_lock_bh(&lmk_task_lock);

	lrw = __lmk_task_first();

	if (lrw) {
		struct lmk_death_pending_entry *ldpt;

		if (lrw->tsk->mm) {
			cp.selected_tasksize = get_mm_rss(lrw->tsk->mm);
		} else {
			lowmem_print(3, "pid:%d no mem\n", lrw->tsk->pid);
			lmk_inc_stats(LMK_ERROR);
			goto unlock_out;
		}

		if (score > lrw->tsk->signal->oom_score_adj) {
			lmk_inc_stats(LMK_PSI_LOW);
			goto unlock_out;
		}

		cp.selected_oom_score_adj = lrw->tsk->signal->oom_score_adj;

		if (death_pending_len >= cp.dynamic_max_queue_len)
			__lmk_death_pending_morgue();
		if (death_pending_len >= cp.dynamic_max_queue_len) {
			lmk_inc_stats(LMK_BUSY);
			cp.selected_tasksize = SHRINK_STOP;
			lowmem_print(3, "Queue %d >= %d",
				     death_pending_len,
				     cp.dynamic_max_queue_len);
			goto unlock_out;
		}

		selected = lrw->tsk;

		/* there is a chance that task is locked,
		 * and the case where it locked in oom_score_adj_write
		 * we might have deadlock. There is no macro for it
		 *  and this is the only place there is a try on
		 * the task_lock.
		 */
		if (!spin_trylock(&selected->alloc_lock)) {
			lowmem_print(2, "Failed to lock task.\n");
			lmk_inc_stats(LMK_BUSY);
			cp.selected_tasksize = SHRINK_STOP;
			goto unlock_out;
		}

		/* move to kill pending set */
		ldpt = kmem_cache_alloc(lmk_dp_cache, GFP_ATOMIC);
		if (!ldpt) {
			WARN_ON(1);
			lmk_inc_stats(LMK_MEM_ERROR);
			cp.selected_tasksize = SHRINK_STOP;
			trace_lmk_sigkill(selected->pid, selected->comm,
					  LMK_TRACE_MEMERROR,
					  cp.selected_tasksize,
					  0);
			goto unlock_out;
		}

		ldpt->tsk = selected;

		__lmk_death_pending_add(ldpt);
		if (!__lmk_task_remove(selected, lrw->key))
			WARN_ON(1);

		spin_unlock_bh(&lmk_task_lock);

		send_sig(SIGKILL, selected, 0);
		LMK_TAG_TASK_DIE(selected);
		print_obituary(selected, &cp, 0);
		trace_lmk_sigkill(selected->pid, selected->comm,
				  cp.selected_oom_score_adj,
				  cp.selected_tasksize,
				  0);

		task_unlock(selected);

		lmk_inc_stats(LMK_PSI_KILL);
		goto out;
	} else {
		lowmem_print(2, "Nothing to kill");
		lmk_inc_stats(LMK_NO_KILL);
	}
unlock_out:
	spin_unlock_bh(&lmk_task_lock);
out:
	if (cp.selected_tasksize == 0)
		lowmem_print(2, "list empty nothing to free\n");
}

static int tngpsi(void *arg)
{
	do {
		short cals = OOM_SCORE_ADJ_MAX + LMK_TNG_PSI_SCORE_OFFSET;
		short prcs = LMK_TNG_PSI_PR_THRESHOLD;

		wait_event_interruptible(tng_trigger->event_wait, tng_trigger->event);
		cmpxchg(&tng_trigger->event, 1, 0);

		cals -= LMK_TNG_PSI_PRESSURE_WEIGHT_FULL * psi_system.avg[PSI_MEM_FULL][0] >> FSHIFT;
		cals -= LMK_TNG_PSI_PRESSURE_WEIGHT_SOME * psi_system.avg[PSI_MEM_SOME][0] >> FSHIFT;
		/* limit the kill threshold, psi never kills perceptible
		 */
		if (cals < LMK_TNG_AM_PERCEPTIBLE_LOW_APP)
			cals = LMK_TNG_AM_PERCEPTIBLE_LOW_APP;

		prcs -= LMK_TNG_PSI_PR_WEIGHT_M_FULL * psi_system.avg[PSI_MEM_FULL][1] >> FSHIFT;
		if (prcs < OOM_SCORE_ADJ_MIN)
			prcs = OOM_SCORE_ADJ_MIN;

		prc_recl_min_score_adj = prcs;
		if (cals < OOM_SCORE_ADJ_MAX)
			balance_psi(cals);
		else
			lmk_inc_stats(LMK_PSI_FASTBLOCK);
	} while (1);
	return 0;
}

static struct task_struct *tngpsi_tsk;

static int __init psi_hook_init(void)
{
	if (static_branch_unlikely(&psi_disabled))
		return -EOPNOTSUPP;

	tng_trigger = psi_trigger_create(&psi_system, LMK_TNG_PSI_PARAMETER,
					 strlen(LMK_TNG_PSI_PARAMETER), 0);

	if (IS_ERR(tng_trigger))
		return PTR_ERR(tng_trigger);

	psi_trigger_replace(&tng_private, tng_trigger);

	tngpsi_tsk = kthread_run(tngpsi, NULL, "tngpsi");
	WARN_ON(!tngpsi_tsk);
	return 0;
}

device_initcall(psi_hook_init);
