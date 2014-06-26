/* include/linux/input/max1187x.h
 *
 * Copyright (c)2013 Maxim Integrated Products, Inc.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Driver Version: 3.1.8
 * Release Date: May 10, 2013
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MAX1187X_H
#define __MAX1187X_H

#define MAX1187X_NAME   "max1187x"
#define MAX1187X_TOUCH  MAX1187X_NAME "_touchscreen_0"
#define MAX1187X_PEN  MAX1187X_NAME "_pen_0"
#define MAX1187X_KEY    MAX1187X_NAME "_key_0"

#define MXM_NUM_FW_MAPPINGS_MAX    5

struct max1187x_fw_mapping {
	u32	config_id;
	u32	chip_id;
	char	*filename;
	u32	filesize;
	u32	filecrc16;
	u32	file_codesize;
};

struct max1187x_pdata {
	char	*vdd_supply_name;
	u32	gpio_tirq;
	u32	gpio_pwr_en;
	u32	num_fw_mappings;
	struct max1187x_fw_mapping  fw_mapping[MXM_NUM_FW_MAPPINGS_MAX];
	u32	defaults_allow;
	u32	default_config_id;
	u32	default_chip_id;
	u32	i2c_words;
	u32	coordinate_settings;
	u32	panel_margin_xl;
	u32	lcd_x;
	u32	panel_margin_xh;
	u32	panel_margin_yl;
	u32	lcd_y;
	u32	panel_margin_yh;
	u32	num_sensor_x;
	u32	num_sensor_y;
	u32	button_code0;
	u32	button_code1;
	u32	button_code2;
	u32	button_code3;
	u32	enable_resume_por;
};

#endif /* __MAX1187X_H */

