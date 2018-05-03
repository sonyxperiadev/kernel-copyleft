/*
 *  lowmemorykiller_oom
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

/* add fake print format with original module name */
#define pr_fmt(fmt) "lowmemorykiller: " fmt

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/oom.h>

#include <trace/events/lmk.h>

#include "lowmemorykiller.h"
#include "lowmemorykiller_stats.h"
#include "lowmemorykiller_tasks.h"

/**
 * lowmemorykiller_oom_notify - OOM notifier
 * @self:	notifier block struct
 * @notused:	not used
 * @parm:	returned - number of pages freed
 *
 * Return value:
 *	NOTIFY_OK
 **/

static int lowmemorykiller_oom_notify(struct notifier_block *self,
				      unsigned long notused, void *param)
{
	struct lmk_rb_watch *lrw;
	unsigned long *nfreed = param;

	lowmem_print(2, "oom notify event\n");
	*nfreed = 0;
	lmk_inc_stats(LMK_OOM_COUNT);
	spin_lock_bh(&lmk_task_lock);
	lrw = __lmk_task_first();
	if (lrw) {
		struct task_struct *selected = lrw->tsk;
		struct lmk_death_pending_entry *ldpt;

		if (!task_trylock_lmk(selected)) {
			lmk_inc_stats(LMK_ERROR);
			lowmem_print(1, "Failed to lock task.\n");
			lmk_inc_stats(LMK_BUSY);
			goto unlock_out;
		}

		/* move to kill pending set */
		ldpt = kmem_cache_alloc(lmk_dp_cache, GFP_ATOMIC);
		ldpt->tsk = selected;

		__lmk_death_pending_add(ldpt);
		if (!__lmk_task_remove(selected, lrw->key))
			WARN_ON(1);

		spin_unlock_bh(&lmk_task_lock);
		*nfreed = get_task_rss(lrw->tsk);
		send_sig(SIGKILL, selected, 0);
		LMK_TAG_TASK_DIE(selected);
		trace_lmk_sigkill(selected->pid, selected->comm,
				  -1, *nfreed,
				  0);

		task_unlock(selected);
		lmk_inc_stats(LMK_OOM_KILL_COUNT);
		goto out;
	}
unlock_out:
	spin_unlock_bh(&lmk_task_lock);
out:
	return NOTIFY_OK;
}

static struct notifier_block lowmemorykiller_oom_nb = {
	.notifier_call = lowmemorykiller_oom_notify
};

int __init lowmemorykiller_register_oom_notifier(void)
{
	register_oom_notifier(&lowmemorykiller_oom_nb);
	return 0;
}
