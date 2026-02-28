// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* rmnet_offload statistics interface */
#include <linux/moduleparam.h>
#include "rmnet_offload_stats.h"

static u64 rmnet_offload_stats[RMNET_OFFLOAD_STAT_MAX];
module_param_array_named(rmnet_offload_stat, rmnet_offload_stats, ullong,
			 NULL, 0444);

void __rmnet_offload_stats_update(u32 stat, u64 inc)
{
	if (stat < RMNET_OFFLOAD_STAT_MAX)
		rmnet_offload_stats[stat] += inc;
}

void rmnet_offload_stats_update(u32 stat)
{
	__rmnet_offload_stats_update(stat, 1);
}
