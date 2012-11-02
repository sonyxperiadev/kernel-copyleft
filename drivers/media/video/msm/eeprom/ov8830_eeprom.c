/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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
 */

#include <linux/module.h>
#include "msm_camera_eeprom.h"
#include "msm_camera_i2c.h"

DEFINE_MUTEX(ov8830_eeprom_mutex);
static struct msm_eeprom_ctrl_t ov8830_eeprom_t;

static const struct i2c_device_id ov8830_eeprom_i2c_id[] = {
	{"ov8830_eeprom", (kernel_ulong_t)&ov8830_eeprom_t},
	{ }
};

static struct i2c_driver ov8830_eeprom_i2c_driver = {
	.id_table = ov8830_eeprom_i2c_id,
	.probe  = msm_eeprom_i2c_probe,
	.remove = __exit_p(ov8830_eeprom_i2c_remove),
	.driver = {
		.name = "ov8830_eeprom",
	},
};

static int __init ov8830_eeprom_i2c_add_driver(void)
{
	int rc = 0;
	rc = i2c_add_driver(ov8830_eeprom_t.i2c_driver);
	return rc;
}

static struct v4l2_subdev_core_ops ov8830_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops ov8830_eeprom_subdev_ops = {
	.core = &ov8830_eeprom_subdev_core_ops,
};

uint8_t ov8830_otp_data_1[6];
struct msm_calib_af ov8830_calibration_data_1;
uint8_t ov8830_otp_data_2[18];
struct msm_calib_wb_light_info ov8830_calibration_data_2;
uint8_t ov8830_otp_data_3[2652];
struct msm_calib_lsc_light_info ov8830_calibration_data_3;

static struct msm_camera_eeprom_info_t ov8830_calib_supp_info = {
	{TRUE, sizeof(struct msm_calib_af), 0, 1},
	{TRUE, sizeof(struct msm_calib_wb_light_info), 1, 1024},
	{TRUE, sizeof(struct msm_calib_lsc_light_info), 2, 1},
	{FALSE, 0, 0, 1},
};

static struct msm_camera_eeprom_read_t ov8830_eeprom_read_tbl[] = {
	{0x0100, &ov8830_otp_data_1[0], 6, 0},
	{0x0110, &ov8830_otp_data_2[0], ARRAY_SIZE(ov8830_otp_data_2), 0},
	{0x0125, &ov8830_otp_data_3[0], ARRAY_SIZE(ov8830_otp_data_3), 0},
};


static struct msm_camera_eeprom_data_t ov8830_eeprom_data_tbl[] = {
	{&ov8830_calibration_data_1, sizeof(struct msm_calib_af)},
	{&ov8830_calibration_data_2, sizeof(struct msm_calib_wb_light_info)},
	{&ov8830_calibration_data_3, sizeof(struct msm_calib_lsc_light_info)},
};

static void ov8830_format_data_1(void)
{
	ov8830_calibration_data_1.start_dac =
		(uint16_t)(ov8830_otp_data_1[1] << 8) | ov8830_otp_data_1[0];
	ov8830_calibration_data_1.macro_dac =
		(uint16_t)(ov8830_otp_data_1[3] << 8) | ov8830_otp_data_1[2];
	ov8830_calibration_data_1.inf_dac =
		(uint16_t)(ov8830_otp_data_1[5] << 8) | ov8830_otp_data_1[4];
}

static void ov8830_format_data_2(void)
{
	ov8830_calibration_data_2.wb_light_info[0].r_over_g =
		(uint16_t)(ov8830_otp_data_2[1] << 8) |
		ov8830_otp_data_2[0];
	ov8830_calibration_data_2.wb_light_info[0].b_over_g =
		(uint16_t)(ov8830_otp_data_2[3] << 8) |
		ov8830_otp_data_2[2];
	ov8830_calibration_data_2.wb_light_info[0].gr_over_gb =
		(uint16_t)(ov8830_otp_data_2[5] << 8) |
		ov8830_otp_data_2[4];

	ov8830_calibration_data_2.wb_light_info[1].r_over_g =
		(uint16_t)(ov8830_otp_data_2[7] << 8) |
		ov8830_otp_data_2[6];
	ov8830_calibration_data_2.wb_light_info[1].b_over_g =
		(uint16_t)(ov8830_otp_data_2[9] << 8) |
		ov8830_otp_data_2[8];
	ov8830_calibration_data_2.wb_light_info[1].gr_over_gb =
		(uint16_t)(ov8830_otp_data_2[11] << 8) |
		ov8830_otp_data_2[10];

	ov8830_calibration_data_2.wb_light_info[2].r_over_g =
		(uint16_t)(ov8830_otp_data_2[13] << 8) |
		ov8830_otp_data_2[12];
	ov8830_calibration_data_2.wb_light_info[2].b_over_g =
		(uint16_t)(ov8830_otp_data_2[15] << 8) |
		ov8830_otp_data_2[14];
	ov8830_calibration_data_2.wb_light_info[2].gr_over_gb =
		(uint16_t)(ov8830_otp_data_2[17] << 8) |
		ov8830_otp_data_2[16];

	ov8830_calibration_data_2.lightidx[0] = 2;
	ov8830_calibration_data_2.lightidx[1] = 2;
	ov8830_calibration_data_2.lightidx[2] = 0;
	ov8830_calibration_data_2.lightidx[3] = 1;
	ov8830_calibration_data_2.lightidx[4] = 1;
	ov8830_calibration_data_2.lightidx[5] = 0;
	ov8830_calibration_data_2.lightidx[6] = 2;
	ov8830_calibration_data_2.lightidx[7] = 0;
	ov8830_calibration_data_2.lightidx[8] = 2;
	ov8830_calibration_data_2.lightidx[9] = 2;
}

static void ov8830_format_data_3(void)
{
	uint8_t *r_gain, *gr_gain, *gb_gain, *b_gain;
	uint16_t i, j;

	r_gain = &ov8830_otp_data_3[0];
	gr_gain = &ov8830_otp_data_3[221];
	gb_gain = &ov8830_otp_data_3[442];
	b_gain = &ov8830_otp_data_3[663];

	for(i=0; i<3; i++) {
		for(j=0; j<221; j++) {
			ov8830_calibration_data_3.lsc_light_info[i].r_gain[j] =
				r_gain[j];
			ov8830_calibration_data_3.lsc_light_info[i].gr_gain[j] =
				gr_gain[j];
			ov8830_calibration_data_3.lsc_light_info[i].gb_gain[j] =
				gb_gain[j];
			ov8830_calibration_data_3.lsc_light_info[i].b_gain[j] =
				b_gain[j];
		}
		r_gain += 884;
		gr_gain += 884;
		gb_gain += 884;
		b_gain += 884;
	}

	ov8830_calibration_data_3.lightidx[0] = 1;
	ov8830_calibration_data_3.lightidx[1] = 0;
	ov8830_calibration_data_3.lightidx[2] = 2;
	ov8830_calibration_data_3.lightidx[3] = 1;
	ov8830_calibration_data_3.lightidx[4] = 1;
	ov8830_calibration_data_3.lightidx[5] = 0;
	ov8830_calibration_data_3.lightidx[6] = 2;
	ov8830_calibration_data_3.lightidx[7] = 0;
	ov8830_calibration_data_3.lightidx[8] = 2;
	ov8830_calibration_data_3.lightidx[9] = 2;
}

void ov8830_format_calibrationdata(void)
{
	ov8830_format_data_1();
	ov8830_format_data_2();
	ov8830_format_data_3();
}
static struct msm_eeprom_ctrl_t ov8830_eeprom_t = {
	.i2c_driver = &ov8830_eeprom_i2c_driver,
	.i2c_addr = 0xA0,
	.eeprom_v4l2_subdev_ops = &ov8830_eeprom_subdev_ops,

	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	},

	.eeprom_mutex = &ov8830_eeprom_mutex,

	.func_tbl = {
		.eeprom_init = NULL,
		.eeprom_release = NULL,
		.eeprom_get_info = msm_camera_eeprom_get_info,
		.eeprom_get_data = msm_camera_eeprom_get_data,
		.eeprom_set_dev_addr = NULL,
		.eeprom_format_data = ov8830_format_calibrationdata,
	},
	.info = &ov8830_calib_supp_info,
	.info_size = sizeof(struct msm_camera_eeprom_info_t),
	.read_tbl = ov8830_eeprom_read_tbl,
	.read_tbl_size = ARRAY_SIZE(ov8830_eeprom_read_tbl),
	.data_tbl = ov8830_eeprom_data_tbl,
	.data_tbl_size = ARRAY_SIZE(ov8830_eeprom_data_tbl),
};

subsys_initcall(ov8830_eeprom_i2c_add_driver);
MODULE_DESCRIPTION("ov8830 EEPROM");
MODULE_LICENSE("GPL v2");
