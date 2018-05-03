/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#ifndef __LOWMEMORYKILLER_H
#define __LOWMEMORYKILLER_H

#include <linux/version.h>
/* The lowest score LMK is using */
#define LMK_SCORE_THRESHOLD 0

extern u32 lowmem_debug_level;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

int __init lowmemorykiller_register_oom_notifier(void);
struct calculated_params {
	long selected_tasksize;
	long minfree;
	int other_file;
	int other_free;
	int dynamic_max_queue_len;
	short selected_oom_score_adj;
	short min_score_adj;
};

int kill_needed(int level, gfp_t mask,
		struct calculated_params *cp);
void print_obituary(struct task_struct *doomed,
		    struct calculated_params *cp,
		    gfp_t gfp_mask);

void balance_cache(void);
ssize_t get_task_rss(struct task_struct *tsk);

/* kernel does not have a task_trylock and
 * to make it more obvious what the code do
 * we create a help function for it.
 * see sched.h for task_lock and task_unlock
 */
static inline int task_trylock_lmk(struct task_struct *p)
{
	return spin_trylock(&p->alloc_lock);
}

/* kernel has changed how killed process should be marked
 * as killed by lmk. This macros is for versioning so we
 * can use same patches on different kernel versions.
 */

/* maybe not exact version, we need something betwwen 3.18 and 4.4.
 * using LINUX_VERSION_CODE like this will give a warning.
 * it it not OK for mainline but for multiple kernel version patches
 * I think it is OK.
 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 0, 0)
#define LMK_TAG_TASK_DIE(x) set_tsk_thread_flag(x, TIF_MEMDIE)
#else
#define LMK_TAG_TASK_DIE(x) mark_oom_victim(x)
#endif

#endif
