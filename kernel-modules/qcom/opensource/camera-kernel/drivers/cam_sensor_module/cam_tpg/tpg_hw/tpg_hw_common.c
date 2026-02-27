// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "tpg_hw_common.h"

int tpg_hw_write_settings(struct tpg_hw *hw,
		struct tpg_settings_config_t *config, struct tpg_reg_settings *settings)
{
		struct cam_hw_soc_info *soc_info = NULL;
		int setting_index;

		if (!hw || !config || !settings) {
			CAM_ERR(CAM_TPG, "invalid params");
			return -EINVAL;
		}

		soc_info = hw->soc_info;
		for (setting_index = 0; setting_index < config->active_count; setting_index++) {
			cam_io_w_mb(settings->reg_value, soc_info->reg_map[0].mem_base +
						settings->reg_offset);
			settings++;
		}

		return 0;
}
