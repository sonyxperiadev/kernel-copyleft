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
	.y		= {5942, 6055, 6117, 6114, 6021},
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
				{4310, 4308, 4306, 4294, 4286},
				{4280, 4280, 4280, 4280, 4280},
				{4234, 4225, 4240, 4235, 4230},
				{4180, 4163, 4185, 4182, 4178},
				{4118, 4107, 4134, 4131, 4128},
				{4072, 4052, 4085, 4083, 4080},
				{4018, 4000, 4039, 4038, 4036},
				{3973, 3953, 3994, 3996, 3995},
				{3931, 3911, 3949, 3953, 3955},
				{3894, 3874, 3907, 3908, 3910},
				{3863, 3843, 3872, 3873, 3873},
				{3835, 3815, 3844, 3845, 3846},
				{3812, 3792, 3822, 3823, 3823},
				{3792, 3772, 3803, 3805, 3804},
				{3774, 3754, 3786, 3789, 3788},
				{3757, 3736, 3769, 3769, 3759},
				{3736, 3716, 3750, 3744, 3730},
				{3711, 3691, 3731, 3725, 3710},
				{3690, 3674, 3705, 3700, 3684},
				{3680, 3660, 3682, 3677, 3663},
				{3679, 3659, 3676, 3672, 3657},
				{3674, 3654, 3670, 3668, 3652},
				{3663, 3643, 3658, 3658, 3637},
				{3644, 3624, 3629, 3620, 3606},
				{3610, 3590, 3589, 3580, 3567},
				{3562, 3542, 3538, 3529, 3517},
				{3500, 3480, 3474, 3470, 3455},
				{3418, 3398, 3388, 3380, 3371},
				{3320, 3300, 3255, 3221, 3206},
				{3000, 3000, 3000, 3000, 3000}
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
				{270, 175, 86, 77, 77},
				{271, 175, 86, 77, 77},
				{273, 175, 85, 77, 77},
				{276, 175, 85, 77, 77},
				{278, 176, 85, 77, 77},
				{282, 177, 86, 77, 77},
				{285, 178, 86, 77, 77},
				{289, 180, 86, 77, 77},
				{293, 183, 86, 77, 78},
				{298, 186, 87, 78, 78},
				{303, 190, 87, 78, 78},
				{308, 194, 88, 79, 78},
				{314, 199, 89, 79, 79},
				{320, 204, 89, 79, 79},
				{327, 210, 90, 80, 79},
				{333, 216, 91, 80, 79},
				{341, 224, 91, 80, 80},
				{348, 231, 92, 80, 80},
				{356, 239, 93, 81, 80},
				{358, 241, 93, 81, 80},
				{360, 243, 93, 81, 80},
				{361, 245, 94, 81, 80},
				{363, 246, 94, 82, 80},
				{365, 248, 95, 82, 80},
				{366, 250, 95, 82, 81},
				{368, 252, 96, 83, 81},
				{370, 254, 97, 83, 81},
				{372, 272, 98, 84, 82},
				{375, 286, 99, 85, 83},
	}
};

struct bms_battery_data oem_batt_data = {
	.fcc			= 6117,
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
