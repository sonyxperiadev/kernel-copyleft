/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright 2021 Sony Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include "dsi_panel.h"

// y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
int dsi_panel_driver_adjust_brightness_default(struct dsi_panel *panel, u32 bl_lvl)
{
	struct dsi_backlight_config bl = panel->bl_config;
	return bl.bl_min_level + (bl_lvl - 0) * (bl.bl_max_level - bl.bl_min_level) / (bl.brightness_max_level - 0);
}

//For 4K OLED
int dsi_panel_driver_adjust_brightness_type1(struct dsi_panel *panel, u32 bl_lvl)
{
	struct dsi_backlight_config bl = panel->bl_config;

	switch (panel->spec_pdata->hmd_mode) {
		case HMD_MODE1:
			if (bl_lvl <= 286)
				return 4 + (bl_lvl - 0) * (326 - 4) / (286 - 0);

			if (bl_lvl <= 1024)
				return 613 + (bl_lvl - 286) * (2047 - 613) / (1024 - 286);

			if (bl_lvl <= 2662)
				return 2048 + (bl_lvl - 1024) * (bl.bl_max_level - 2048) / (2662 - 1024);

			return bl.bl_max_level;

		case HMD_MODE2:
			if (bl_lvl <= 229)
				return 4 + (bl_lvl - 0) * (484 - 4) / (229 - 0);

			if (bl_lvl <= 819)
				return 624 + (bl_lvl - 229) * (2047 - 624) / (819 - 229);

			if (bl_lvl <= 2252)
				return 2048 + (bl_lvl - 819) * (bl.bl_max_level - 2048) / (2252 - 819);

			return bl.bl_max_level;
		case HMD_OFF:
			if (bl_lvl <= 1720)
				return 4 + (bl_lvl - 0) * (2047 - 4) / (1720 - 0);

			return 2048 + (bl_lvl - 1720) * (bl.bl_max_level - 2048) / (bl.brightness_max_level - 1720);
		default:
			return dsi_panel_driver_adjust_brightness_default(panel, bl_lvl);
	}
}


//For FHD OLED1
int dsi_panel_driver_adjust_brightness_type2(struct dsi_panel *panel, u32 bl_lvl)
{
	return dsi_panel_driver_adjust_brightness_default(panel, bl_lvl);
}

//For FHD OLED2
int dsi_panel_driver_adjust_brightness_type3(struct dsi_panel *panel, u32 bl_lvl)
{
	u32 reg_val = 0;
	int is_need_hbm = 0;
	int cur_hbm_mode = panel->spec_pdata->hbm_mode;

	if(bl_lvl <= 2641)
		reg_val = 4 + (bl_lvl - 0) * (2047-4) / (2641-0);
	else
		reg_val = 2048 + (bl_lvl - 2641) * (4095-2048) / (4095-2641);

	is_need_hbm = reg_val >= 0x800;

	if (cur_hbm_mode != is_need_hbm)
		dsi_panel_set_hbm_mode_core(panel, is_need_hbm);

	return reg_val;
}

