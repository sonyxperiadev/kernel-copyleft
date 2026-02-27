/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __TPG_HW_COMMON_H__
#define __TPG_HW_COMMON_H__

#include <linux/kernel.h>
#include "cam_debug_util.h"
#include "cam_soc_util.h"
#include <cam_cpas_api.h>
#include <media/cam_sensor.h>
#include "tpg_hw.h"

/**
 * @brief  write register settings
 *
 * @param hw   : tpg hw instance
 * @param config : argument for settings config
 * @param settings : argument for register settings
 *
 * @return     : 0 on sucdess
 */
int tpg_hw_write_settings(struct tpg_hw *hw,
		struct tpg_settings_config_t *config, struct tpg_reg_settings *settings);

#endif
