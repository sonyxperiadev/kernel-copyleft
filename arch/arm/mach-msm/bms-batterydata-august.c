/* copyright (c) 2011, code aurora forum. all rights reserved.
 * Copyright (C) 2012 Sony Ericsson Mobile Communications AB.
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
	.y	= {1600, 1690, 1760, 1760, 1760},
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
		{4160, 4160, 4160, 4160, 4160},
		{4115, 4115, 4115, 4115, 4115},
		{4062, 4062, 4062, 4062, 4062},
		{4027, 4027, 4027, 4027, 4027},
		{3992, 3992, 3992, 3992, 3992},
		{3964, 3964, 3964, 3964, 3964},
		{3930, 3930, 3930, 3930, 3930},
		{3898, 3898, 3898, 3898, 3898},
		{3870, 3870, 3870, 3870, 3870},
		{3840, 3840, 3840, 3840, 3840},
		{3825, 3825, 3825, 3825, 3825},
		{3807, 3807, 3807, 3807, 3807},
		{3800, 3800, 3800, 3800, 3800},
		{3787, 3787, 3787, 3787, 3787},
		{3780, 3780, 3780, 3780, 3780},
		{3762, 3762, 3762, 3762, 3762},
		{3750, 3750, 3750, 3750, 3750},
		{3718, 3718, 3718, 3718, 3718},
		{3690, 3690, 3690, 3690, 3690},
		{3675, 3675, 3675, 3675, 3675},
		{3660, 3660, 3660, 3660, 3660},
		{3650, 3650, 3650, 3650, 3650},
		{3632, 3632, 3632, 3632, 3632},
		{3615, 3615, 3615, 3615, 3615},
		{3592, 3592, 3592, 3592, 3592},
		{3568, 3568, 3568, 3568, 3568},
		{3530, 3530, 3530, 3530, 3530},
		{3410, 3410, 3410, 3410, 3410},
		{3000, 3000, 3000, 3000, 3000}
	},
};

struct bms_battery_data  oem_batt_data __devinitdata = {
	.fcc			= 1760,
	.fcc_temp_lut		= &fcc_temp,
	.fcc_sf_lut		= &fcc_sf,
	.pc_temp_ocv_lut	= &pc_temp_ocv,
	.pc_sf_lut		= &pc_sf,
	.default_rbatt_mohm	= 100,
	.delta_rbatt_mohm	= 0,
};
