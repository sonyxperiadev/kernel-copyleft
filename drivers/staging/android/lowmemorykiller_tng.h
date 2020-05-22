/*
 *  lowmemorykiller_tng interface
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __LOWMEMORYKILLER_TNG_H
#define __LOWMEMORYKILLER_TNG_H
extern short lowmem_adj[];
extern int lowmem_minfree[];
extern int oom_reaper;

#ifdef CONFIG_PROCESS_RECLAIM /* with SONY extentions! */
extern short prc_recl_min_score_adj;
#endif
#ifdef CONFIG_PSI
extern struct psi_group psi_system;
#endif

/* basic kill reason */
#define LMK_VMPRESSURE		(0x1)
#define LMK_SHRINKER_SCAN	(0x2)
#define LMK_OOM			(0x4)
#define LMK_SHRINKER_COUNT	(0x8)
#define LMK_PSI			(0x10)
/* calc option reason */
#define LMK_LOW_RESERVE		(0x0100)
#define LMK_CANT_SWAP		(0x0200)
#define LMK_LOWIAP_SWAP		(0x0400)

/* function in lowmemorykiller.c */
int lowmem_min_param_size(void);
int adjust_minadj(short *min_score_adj);
/* please dont use tune_lmk_param directly without good reason */
void tune_lmk_param(int *other_free, int *other_file,
		    struct shrink_control *sc);
void tune_lmk_param_mask(int *other_free, int *other_file, gfp_t mask);
void __init lowmem_init_tng(struct shrinker *shrinker);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_TNG_VMPRESSURE
void balance_cache(unsigned long vmpressure);
#else
#define balance_cache(vmpressure) do { } while (0)
#endif
void mark_lmk_victim(struct task_struct *tsk);

#define LMK_TNG_WORKLOAD_OFFSET (250)
#define LMK_TNG_WORKLOAD_MAX (600)
#define LMK_TNG_AM_PERCEPTIBLE_LOW_APP (250)
/* tuning parameters for psi */
#define LMK_TNG_PSI_PARAMETER "full 10000 500000"
#define LMK_TNG_PSI_SCORE_OFFSET (100)
#define LMK_TNG_PSI_PRESSURE_WEIGHT_FULL (120)
#define LMK_TNG_PSI_PRESSURE_WEIGHT_SOME (25)
#define LMK_TNG_PSI_PR_THRESHOLD (600)
#define LMK_TNG_PSI_PR_WEIGHT_M_FULL (128)

#endif
