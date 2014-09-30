/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Sony Mobile Communications Inc.
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

#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-10, 0, 25, 40, 65},
	.y		= {3032, 3057, 3084, 3071, 3034},
	.cols	= 5
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 30,
	.cols		= 5,
	.temp		= {-10, 0, 25, 40, 65},
	.percent	= {100, 99, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45,
					40, 35, 30, 25, 20, 15, 10, 9, 8, 7, 6,
					5, 4, 3, 2, 1, 0},
	.ocv		= {
				{4320, 4316, 4314, 4303, 4294},
				{4280, 4280, 4280, 4280, 4280},
				{4243, 4230, 4246, 4241, 4234},
				{4184, 4167, 4190, 4186, 4180},
				{4125, 4110, 4138, 4135, 4129},
				{4077, 4058, 4088, 4086, 4080},
				{4017, 3998, 4040, 4038, 4033},
				{3973, 3953, 3995, 3996, 3992},
				{3925, 3905, 3949, 3951, 3949},
				{3887, 3867, 3906, 3910, 3909},
				{3861, 3841, 3870, 3872, 3869},
				{3837, 3817, 3843, 3844, 3843},
				{3818, 3798, 3820, 3821, 3819},
				{3802, 3782, 3801, 3802, 3800},
				{3788, 3768, 3784, 3786, 3784},
				{3771, 3751, 3768, 3767, 3759},
				{3747, 3727, 3748, 3742, 3725},
				{3724, 3704, 3729, 3722, 3705},
				{3705, 3687, 3703, 3699, 3681},
				{3686, 3667, 3674, 3668, 3653},
				{3688, 3668, 3672, 3666, 3650},
				{3682, 3662, 3668, 3663, 3646},
				{3679, 3659, 3664, 3660, 3641},
				{3663, 3645, 3644, 3636, 3617},
				{3634, 3614, 3606, 3600, 3576},
				{3594, 3574, 3554, 3550, 3522},
				{3536, 3516, 3490, 3487, 3455},
				{3455, 3434, 3411, 3406, 3374},
				{3369, 3348, 3285, 3263, 3248},
				{3000, 3000, 3000, 3000, 3000},
	}
};

static struct sf_lut rbatt_sf = {
	.rows		= 29,
	.cols		= 5,
	/* row_entries are temperature */
	.row_entries	= {-10, 0, 25, 40, 65},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40,
					35, 30, 25, 20, 15, 10, 9, 8, 7, 6, 5,
					4, 3, 2, 1, 0},
	.sf		= {
				{399, 224, 114, 104, 103},
				{399, 224, 114, 104, 103},
				{400, 224, 114, 104, 103},
				{402, 225, 114, 104, 103},
				{405, 226, 114, 104, 103},
				{409, 228, 115, 104, 103},
				{407, 234, 115, 105, 104},
				{409, 237, 115, 105, 104},
				{418, 240, 116, 105, 104},
				{429, 244, 117, 105, 105},
				{435, 253, 117, 106, 105},
				{446, 261, 119, 106, 105},
				{457, 263, 120, 107, 105},
				{460, 274, 120, 107, 106},
				{468, 279, 122, 108, 106},
				{475, 286, 122, 108, 106},
				{486, 293, 123, 108, 107},
				{498, 298, 124, 109, 107},
				{529, 305, 125, 109, 108},
				{525, 319, 125, 110, 108},
				{531, 325, 126, 110, 108},
				{539, 327, 126, 110, 108},
				{548, 332, 127, 110, 108},
				{559, 341, 128, 110, 109},
				{571, 351, 129, 111, 109},
				{585, 346, 130, 112, 110},
				{601, 356, 133, 113, 111},
				{620, 391, 136, 116, 112},
				{641, 420, 159, 132, 129},
	}
};

struct bms_battery_data oem_batt_data = {
	.fcc			= 3084,
	.fcc_temp_lut		= &fcc_temp,
	.pc_temp_ocv_lut	= &pc_temp_ocv,
	.rbatt_sf_lut		= &rbatt_sf,
	.default_rbatt_mohm	= 100,
	.flat_ocv_threshold_uv	= 3800000,
};

struct bms_battery_data oem_batt_data_somc[BATT_VENDOR_NUM] = {
	/* BATT_VENDOR_TDK */
	{
	},
	/* BATT_VENDOR_SEND */
	{
		.fcc			= 3380,
		.fcc_temp_lut		= &fcc_temp,
		.pc_temp_ocv_lut	= &pc_temp_ocv,
		.rbatt_sf_lut		= &rbatt_sf,
		.default_rbatt_mohm	= 100,
		.flat_ocv_threshold_uv	= 3800000,
		.r_sense_uohm		= 10000,
		.ocv_high_threshold_uv	= 3830000,
		.ocv_low_threshold_uv	= 3750000,
	},
	/* BATT_VENDOR_SANYO */
	{
	},
	/* BATT_VENDOR_LG */
	{
	},
	/* BATT_VENDOR_5TH */
	{
	},
};
