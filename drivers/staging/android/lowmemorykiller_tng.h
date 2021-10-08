/*
 *  lowmemorykiller_tng interface
 *
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

#ifndef __LOWMEMORYKILLER_TNG_H
#define __LOWMEMORYKILLER_TNG_H

extern short lowmem_adj[];
extern int lowmem_minfree[];
/* function in lowmemorykiller.c */
int lowmem_min_param_size(void);
int adjust_minadj(short *min_score_adj);
/* please dont use tune_lmk_param directly without good reason */
void tune_lmk_param(int *other_free, int *other_file,
		    struct shrink_control *sc);
void tune_lmk_param_mask(int *other_free, int *other_file, gfp_t mask);
void __init lowmem_init_tng(struct shrinker *shrinker);
void balance_cache(void);
#endif
