// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN stats framework */

#include <linux/moduleparam.h>
#include "rmnet_wlan_stats.h"

static u64 rmnet_wlan_stats[RMNET_WLAN_STAT_MAX];
module_param_array_named(rmnet_wlan_stat, rmnet_wlan_stats, ullong, NULL, 0444);

static u64 rmnet_wlan_forward_stats[RMNET_WLAN_F_S_R0_MAX];
module_param_array_named(rmnet_wlan_forward_stat, rmnet_wlan_forward_stats, ullong, NULL, 0444);

void rmnet_wlan_stats_update(u32 stat)
{
	if (stat < RMNET_WLAN_STAT_MAX)
		rmnet_wlan_stats[stat] += 1;
}

void rmnet_wlan_forward_stats_update(u32 stat)
{
	if (stat < RMNET_WLAN_F_S_R0_MAX)
		rmnet_wlan_forward_stats[stat] += 1;
}
