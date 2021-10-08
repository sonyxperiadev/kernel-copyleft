/*
 *  lowmemorykiller_tng
 *
 *  Author: Peter Enderborg <peter.enderborg@sonymobile.com>
 *  co Author: Snild Dolkow
 *  co Author: Björn Davidsson
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
/* This file contain the logic for new lowmemorykiller (TNG). Parts
 * of the calculations is taken from the original lowmemorykiller
 */

/* add fake print format with original module name */
#define pr_fmt(fmt) "lowmemorykiller: " fmt

#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/mm.h>

#include <linux/oom_score_notifier.h>
#include "lowmemorykiller_tasks.h"
#include "lowmemorykiller_oom.h"
#include "lowmemorykiller_tng.h"

#include <trace/events/lmk.h>
#ifdef CONFIG_ZCACHE
#include <linux/zcache.h>
#else
/*zcache.h has incorrect definition here.*/
static inline u64 zcache_pages(void) { return 0; }
#endif
#define LMK_ZOMBIE_SIZE (4096)

static unsigned long lowmem_count_tng(struct shrinker *s,
				      struct shrink_control *sc);
static unsigned long lowmem_scan_tng(struct shrinker *s,
				     struct shrink_control *sc);

void __init lowmem_init_tng(struct shrinker *shrinker)
{
	lmk_dp_cache = KMEM_CACHE(lmk_death_pending_entry, 0);
	lmk_task_cache = KMEM_CACHE(lmk_rb_watch, 0);
	oom_score_notifier_register(&lmk_oom_score_nb);
	lowmemorykiller_register_oom_notifier();
	shrinker->count_objects = lowmem_count_tng;
	shrinker->scan_objects = lowmem_scan_tng;
}

ssize_t get_task_rss(struct task_struct *tsk)
{
	unsigned long rss = 0;
	struct mm_struct *mm;

	mm = ACCESS_ONCE(tsk->mm);
	if (mm)
		rss = get_mm_rss(mm);
	if (rss < LMK_ZOMBIE_SIZE)
		rss = LMK_ZOMBIE_SIZE;
	return rss;
}

static void calc_params(struct calculated_params *cp, gfp_t mask)
{
	int i;
	int array_size;

	cp->other_free = global_page_state(NR_FREE_PAGES);
	cp->other_file = global_page_state(NR_FILE_PAGES) + zcache_pages() -
		global_page_state(NR_SHMEM) -
		global_page_state(NR_UNEVICTABLE) -
		total_swapcache_pages();
	cp->other_file = (cp->other_file < 0) ? 0 : cp->other_file;

	cp->minfree = 0;
	cp->min_score_adj = SHRT_MAX;
	tune_lmk_param_mask(&cp->other_free, &cp->other_file, mask);
	array_size = lowmem_min_param_size();
	for (i = 0; i < array_size; i++) {
		cp->minfree = lowmem_minfree[i];
		if (cp->other_free < cp->minfree &&
		    cp->other_file < cp->minfree) {
			cp->min_score_adj = lowmem_adj[i];
			break;
		}
	}
	adjust_minadj(&cp->min_score_adj);
	cp->dynamic_max_queue_len = array_size - i + 1;
}

int kill_needed(int level, gfp_t mask,
		struct calculated_params *cp)
{
	calc_params(cp, mask);
	cp->selected_oom_score_adj = level;

	if (level >= cp->min_score_adj)
		return 1;
	return 0;
}

void print_obituary(struct task_struct *doomed,
		    struct calculated_params *cp,
		    gfp_t gfp_mask) {
	long cache_size = cp->other_file * (long)(PAGE_SIZE / 1024);
	long cache_limit = cp->minfree * (long)(PAGE_SIZE / 1024);
	long free = cp->other_free * (long)(PAGE_SIZE / 1024);

	lowmem_print(1, "Killing '%s' (%d), adj %hd,\n"
		     "   to free %ldkB on behalf of '%s' (%d) because\n"
		     "   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n"
		     "   Free memory is %ldkB above reserved.\n"
		     "   Free CMA is %ldkB\n"
		     "   Total reserve is %ldkB\n"
		     "   Total free pages is %ldkB\n"
		     "   Total file cache is %ldkB\n"
		     "   Slab Reclaimable is %ldkB\n"
		     "   Slab UnReclaimable is %ldkB\n"
		     "   Total Slab is %ldkB\n"
		     "   GFP mask is 0x%x\n"
		     "   queue len is %d of max %d\n",
		     doomed->comm, doomed->pid,
		     cp->selected_oom_score_adj,
		     cp->selected_tasksize * (long)(PAGE_SIZE / 1024),
		     current->comm, current->pid,
		     cache_size, cache_limit,
		     cp->min_score_adj,
		     free,
		     global_page_state(NR_FREE_CMA_PAGES) *
		     (long)(PAGE_SIZE / 1024),
		     totalreserve_pages * (long)(PAGE_SIZE / 1024),
		     global_page_state(NR_FREE_PAGES) *
		     (long)(PAGE_SIZE / 1024),
		     global_page_state(NR_FILE_PAGES) *
		     (long)(PAGE_SIZE / 1024),
		     global_page_state(NR_SLAB_RECLAIMABLE) *
		     (long)(PAGE_SIZE / 1024),
		     global_page_state(NR_SLAB_UNRECLAIMABLE) *
		     (long)(PAGE_SIZE / 1024),
		     global_page_state(NR_SLAB_RECLAIMABLE) *
		     (long)(PAGE_SIZE / 1024) +
		     global_page_state(NR_SLAB_UNRECLAIMABLE) *
		     (long)(PAGE_SIZE / 1024),
		     gfp_mask,
		     death_pending_len,
		     cp->dynamic_max_queue_len);
}

static unsigned long lowmem_count_tng(struct shrinker *s,
				      struct shrink_control *sc)
{
	struct lmk_rb_watch *lrw;
	struct calculated_params cp;
	short score;

	if (current_is_kswapd())
		return 0;
	lmk_inc_stats(LMK_COUNT);
	cp.selected_tasksize = 0;
	spin_lock(&lmk_task_lock);
	lrw = __lmk_task_first();
	if (lrw) {
		int rss = get_task_rss(lrw->tsk);

		score = lrw->tsk->signal->oom_score_adj;
		spin_unlock(&lmk_task_lock);
		if (kill_needed(score, sc->gfp_mask, &cp)) {
			if (death_pending_len < cp.dynamic_max_queue_len)
				cp.selected_tasksize = rss;
			else if (lmk_death_pending_morgue())
				if (death_pending_len <
				    cp.dynamic_max_queue_len)
					cp.selected_tasksize = rss;
		}
	} else {
		spin_unlock(&lmk_task_lock);
		lowmem_print(2, "Empty task list in count");
	}
	if (cp.selected_tasksize == 0)
		lmk_inc_stats(LMK_ZERO_COUNT);

	return cp.selected_tasksize;
}

static unsigned long lowmem_scan_tng(struct shrinker *s,
				     struct shrink_control *sc)
{
	struct task_struct *selected = NULL;
	unsigned long nr_to_scan = sc->nr_to_scan;
	struct lmk_rb_watch *lrw;
	int do_kill;
	struct calculated_params cp;

	lmk_inc_stats(LMK_SCAN);

	cp.selected_tasksize = 0;
	spin_lock(&lmk_task_lock);

	lrw = __lmk_task_first();
	if (lrw) {
		cp.selected_tasksize = get_task_rss(lrw->tsk);
		do_kill = kill_needed(lrw->key, sc->gfp_mask, &cp);
		if (death_pending_len >= cp.dynamic_max_queue_len) {
			lmk_inc_stats(LMK_BUSY);
			cp.selected_tasksize = SHRINK_STOP;
			goto unlock_out;
		}

		if (do_kill) {
			struct lmk_death_pending_entry *ldpt;

			selected = lrw->tsk;

			/* there is a chance that task is locked,
			 * and the case where it locked in oom_score_adj_write
			 * we might have deadlock.
			 */
			if (!task_trylock_lmk(selected)) {
				lmk_inc_stats(LMK_ERROR);
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

			spin_unlock(&lmk_task_lock);

			send_sig(SIGKILL, selected, 0);
			LMK_TAG_TASK_DIE(selected);
			trace_lmk_sigkill(selected->pid, selected->comm,
					  cp.selected_oom_score_adj,
					  cp.selected_tasksize,
					  sc->gfp_mask);
			print_obituary(selected, &cp, sc->gfp_mask);

			task_unlock(selected);
			lmk_inc_stats(LMK_KILL);
			goto out;
		} else {
			lmk_inc_stats(LMK_WASTE);
		}
	} else {
		lmk_inc_stats(LMK_NO_KILL);
	}

unlock_out:
	spin_unlock(&lmk_task_lock);
out:
	if (cp.selected_tasksize == 0)
		lowmem_print(2, "list empty nothing to free\n");
	lowmem_print(4, "lowmem_shrink %lu, %x, return %ld\n",
		     nr_to_scan, sc->gfp_mask, cp.selected_tasksize);

	return cp.selected_tasksize;
}

void tune_lmk_param_mask(int *other_free, int *other_file, gfp_t mask)
{
	struct shrink_control fake_shrink_control;

	fake_shrink_control.gfp_mask = mask;
	tune_lmk_param(other_free, other_file, &fake_shrink_control);
}
