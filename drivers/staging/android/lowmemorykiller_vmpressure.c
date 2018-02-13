/*
 *  lowmemorykiller_vmpressure
 *
 *  Author: Peter Enderborg <peter.enderborg@sonymobile.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

/* todo: Handle vmpressure notifier directly.
 * It is there on this version since it us hooking
 * into qualcomm vmpressure extension.
 */

/* add fake print format with original module name */
#define pr_fmt(fmt) "lowmemorykiller: " fmt

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/slab.h>
#include <linux/vmpressure.h>

#include "lowmemorykiller.h"
#include "lowmemorykiller_stats.h"
#include "lowmemorykiller_tasks.h"

#define SHRINK_STOP (-1)
void balance_cache(void)
{
	struct task_struct *selected = NULL;
	struct lmk_rb_watch *lrw;
	int do_kill;
	struct calculated_params cp;
	gfp_t mask = __GFP_FS | __GFP_IO;

	cp.selected_tasksize = 0;
	cp.dynamic_max_queue_len = 1;
	spin_lock_bh(&lmk_task_lock);

	lrw = __lmk_task_first();

	if (lrw) {
		if (lrw->tsk->mm) {
			cp.selected_tasksize = get_mm_rss(lrw->tsk->mm);
		} else {
			lowmem_print(3, "pid:%d no mem\n", lrw->tsk->pid);
			lmk_inc_stats(LMK_ERROR);
			goto unlock_out;
		}

		do_kill = kill_needed(lrw->key, mask, &cp);

		if (death_pending_len >= cp.dynamic_max_queue_len) {
			lmk_inc_stats(LMK_BUSY);
			cp.selected_tasksize = SHRINK_STOP;
			lowmem_print(3, "Queue %d >= %d",
				     death_pending_len,
				     cp.dynamic_max_queue_len);
			goto unlock_out;
		}

		if (do_kill) {
			struct lmk_death_pending_entry *ldpt;

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
			ldpt->tsk = selected;

			__lmk_death_pending_add(ldpt);
			if (!__lmk_task_remove(selected, lrw->key))
				WARN_ON(1);

			spin_unlock_bh(&lmk_task_lock);

			send_sig(SIGKILL, selected, 0);
			set_tsk_thread_flag(selected, TIF_MEMDIE);

			print_obituary(selected, &cp, 0);

			task_unlock(selected);
			lmk_inc_stats(LMK_BALANCE_KILL);
			goto out;
		} else {
			lowmem_print(3, "No kill");
			lmk_inc_stats(LMK_BALANCE_WASTE);
		}
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
