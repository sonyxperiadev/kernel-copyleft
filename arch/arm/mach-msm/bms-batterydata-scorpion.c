/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are licensed under the License.
 */

#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-10, 5, 25, 50, 65},
	.y		= {4205, 4430, 4600, 4635, 4600},
	.cols	= 5
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 29,
	.cols		= 5,
	.temp		= {-10, 5, 25, 50, 65},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45,
					40, 35, 30, 25, 20, 15, 10, 9, 8, 7, 6,
					5, 4, 3, 2, 1, 0},
	.ocv		= {
				{4290, 4290, 4290, 4290, 4290},
				{4235, 4250, 4256, 4239, 4232},
				{4171, 4192, 4199, 4183, 4177},
				{4120, 4134, 4145, 4130, 4125},
				{4068, 4078, 4094, 4079, 4074},
				{4020, 4024, 4046, 4032, 4028},
				{3974, 3974, 4001, 3990, 3987},
				{3924, 3930, 3956, 3949, 3948},
				{3888, 3892, 3910, 3904, 3907},
				{3852, 3861, 3873, 3866, 3864},
				{3825, 3834, 3844, 3835, 3833},
				{3802, 3811, 3822, 3815, 3814},
				{3781, 3792, 3803, 3796, 3794},
				{3766, 3778, 3788, 3779, 3776},
				{3746, 3761, 3775, 3755, 3749},
				{3728, 3747, 3755, 3732, 3718},
				{3713, 3726, 3732, 3711, 3695},
				{3686, 3697, 3706, 3680, 3665},
				{3670, 3667, 3671, 3661, 3644},
				{3664, 3654, 3667, 3653, 3639},
				{3653, 3646, 3662, 3643, 3630},
				{3639, 3629, 3655, 3637, 3620},
				{3629, 3613, 3643, 3627, 3610},
				{3608, 3591, 3611, 3616, 3593},
				{3582, 3557, 3568, 3580, 3561},
				{3533, 3498, 3508, 3516, 3502},
				{3470, 3426, 3421, 3431, 3421},
				{3366, 3332, 3297, 3307, 3306},
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
				{289, 158, 83, 78, 78},
				{289, 158, 83, 78, 78},
				{289, 158, 83, 78, 78},
				{289, 158, 84, 78, 78},
				{289, 158, 84, 78, 78},
				{289, 159, 84, 78, 78},
				{290, 160, 84, 78, 78},
				{292, 162, 84, 78, 79},
				{294, 165, 85, 78, 79},
				{300, 169, 85, 79, 79},
				{306, 173, 85, 79, 79},
				{310, 177, 86, 79, 79},
				{318, 183, 86, 79, 79},
				{325, 187, 87, 79, 79},
				{331, 190, 87, 80, 80},
				{340, 194, 88, 80, 80},
				{347, 200, 88, 80, 80},
				{354, 205, 89, 80, 80},
				{357, 211, 89, 81, 80},
				{362, 219, 90, 81, 81},
				{365, 224, 90, 81, 81},
				{369, 225, 90, 81, 81},
				{371, 232, 91, 81, 81},
				{378, 233, 91, 82, 81},
				{379, 246, 92, 82, 81},
				{383, 260, 93, 83, 82},
				{389, 273, 94, 84, 82},
				{396, 352, 98, 86, 83},
				{775, 475, 131, 106, 98},
	}
};

struct bms_battery_data oem_batt_data_somc[BATT_VENDOR_NUM] = {
	/* BATT_VENDOR_TDK */
	{
	},
	/* BATT_VENDOR_SEND */
	{
		.fcc			= 4600,
		.fcc_temp_lut		= &fcc_temp,
		.pc_temp_ocv_lut	= &pc_temp_ocv,
		.rbatt_sf_lut		= &rbatt_sf,
		.default_rbatt_mohm	= 100,
		.flat_ocv_threshold_uv	= 3800000,
		.r_sense_uohm		= 10000,
		.ocv_high_threshold_uv	= 3790000,
		.ocv_low_threshold_uv	= 3720000,
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

struct bms_battery_data *bms_batt_data = &oem_batt_data_somc[0];
int bms_batt_data_num = BATT_VENDOR_NUM;
