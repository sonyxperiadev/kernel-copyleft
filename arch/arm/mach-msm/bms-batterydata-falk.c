/* copyright (c) 2011, code aurora forum. all rights reserved.
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/mfd/pm8xxx/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x	= {-20, 0, 25, 40, 65},
	.y	= {1500, 1630, 1730, 1750, 1760},
	.cols	= 5,
};

static struct single_row_lut fcc_sf = {
	.x	= {100, 200, 300, 400, 500},
	.y	= {100, 100, 100, 100, 100},
	.cols	= 5,
};

static struct sf_lut pc_sf = {
	.rows		= 10,
	.cols		= 5,
	.row_entries		= {100, 200, 300, 400, 500},
	.percent	= {100, 90, 80, 70, 60, 50, 40, 30, 20, 10},
	.sf		= {
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100},
		{100, 100, 100, 100, 100}
	},
};

static struct pc_temp_ocv_lut  pc_temp_ocv = {
	.rows		= 29,
	.cols		= 5,
	.temp		= {-20, 0, 25, 40, 65},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55,
		50, 45, 40, 35, 30, 25, 20, 15, 10, 9,
		8, 7, 6, 5, 4, 3, 2, 1, 0
	},
	.ocv		= {
		{4180, 4180, 4180, 4180, 4180},
		{4130, 4130, 4130, 4130, 4130},
		{4080, 4080, 4080, 4080, 4080},
		{4040, 4040, 4040, 4040, 4040},
		{4000, 4000, 4000, 4000, 4000},
		{3970, 3970, 3970, 3970, 3970},
		{3940, 3940, 3940, 3940, 3940},
		{3910, 3910, 3910, 3910, 3910},
		{3880, 3880, 3880, 3880, 3880},
		{3860, 3860, 3860, 3860, 3860},
		{3840, 3840, 3840, 3840, 3840},
		{3820, 3820, 3820, 3820, 3820},
		{3805, 3805, 3805, 3805, 3805},
		{3773, 3800, 3800, 3800, 3800},
		{3785, 3785, 3780, 3775, 3765},
		{3765, 3765, 3760, 3755, 3745},
		{3745, 3745, 3740, 3735, 3725},
		{3725, 3725, 3720, 3715, 3705},
		{3705, 3705, 3700, 3695, 3685},
		{3685, 3685, 3680, 3675, 3665},
		{3675, 3675, 3670, 3665, 3655},
		{3659, 3659, 3654, 3649, 3639},
		{3640, 3640, 3635, 3630, 3620},
		{3602, 3602, 3597, 3592, 3582},
		{3555, 3555, 3550, 3545, 3535},
		{3495, 3495, 3490, 3485, 3475},
		{3417, 3417, 3412, 3407, 3397},
		{3300, 3300, 3295, 3290, 3280},
		{3000, 3000, 3000, 3000, 3000}
	},
};

struct bms_battery_data  oem_batt_data __devinitdata = {
	.fcc			= 1730,
	.fcc_temp_lut		= &fcc_temp,
	.fcc_sf_lut		= &fcc_sf,
	.pc_temp_ocv_lut	= &pc_temp_ocv,
	.pc_sf_lut		= &pc_sf,
	.default_rbatt_mohm	= 100,
	.delta_rbatt_mohm	= 0,
};
