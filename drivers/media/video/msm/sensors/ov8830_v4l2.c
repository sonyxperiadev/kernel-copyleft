/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#include "msm_sensor.h"
#define SENSOR_NAME "ov8830"
#define PLATFORM_DRIVER_NAME "msm_camera_ov8830"
#define ov8830_obj ov8830_##obj

DEFINE_MUTEX(ov8830_mut);
static struct msm_sensor_ctrl_t ov8830_s_ctrl;

static struct msm_camera_i2c_reg_conf ov8830_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf ov8830_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf ov8830_groupon_settings[] = {
	{0x3208, 0x00},
};

static struct msm_camera_i2c_reg_conf ov8830_groupoff_settings[] = {
	{0x3208, 0x10},
	{0x3208, 0xA0},
};

static struct msm_camera_i2c_reg_conf ov8830_snap_settings[] = {
	{0x3091, 0x11},
	{0x3093, 0x01},
	{0x30b3, 0x78},
	{0x3501, 0x9a},
	{0x3502, 0x80},
	{0x3601, 0x02},
	{0x3602, 0x1c},
	{0x3708, 0xe2},
	{0x3709, 0x03},
	{0x3802, 0x00},
	{0x3803, 0x0c},
	{0x3806, 0x09},
	{0x3807, 0xa3},
	{0x3808, 0x0c},
	{0x3809, 0xc0},
	{0x380a, 0x09},
	{0x380b, 0x90},
	{0x380e, 0x09},
	{0x380f, 0xb4},
	{0x3a04, 0x09},
	{0x3a05, 0xa9},
	{0x4000, 0x18},
	{0x4004, 0x02},
	{0x4005, 0x1A},
	{0x4006, 0x16},
	{0x404f, 0x7F},
	{0x4103, 0x10},
	{0x4805, 0x01},
	{0x4837, 0x08},
	{0x4d03, 0xE3},
};

static struct msm_camera_i2c_reg_conf ov8830_1080p_settings[] = {
	{0x3091, 0x12},
	{0x3093, 0x00},
	{0x30b3, 0x50},
	{0x3501, 0x7b},
	{0x3502, 0xe0},
	{0x3601, 0x0a},
	{0x3602, 0x9c},
	{0x3708, 0xe3},
	{0x3709, 0xc3},
	{0x3802, 0x01},
	{0x3803, 0x36},
	{0x3806, 0x08},
	{0x3807, 0x77},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x40},
	{0x380e, 0x07},
	{0x380f, 0xcc},
	{0x3a04, 0x07},
	{0x3a05, 0x49},
	{0x4000, 0x10},
	{0x4004, 0x08},
	{0x4005, 0x18},
	{0x4006, 0x20},
	{0x404f, 0xa0},
	{0x4103, 0x00},
	{0x4805, 0x21},
	{0x4837, 0x0d},
	{0x4d03, 0xbb},
};

static struct msm_camera_i2c_reg_conf ov8830_recommended_settings[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x0100, 0x00},
	{0x0100, 0x00},
	{0x0100, 0x00},
	{0x0102, 0x01},
	{0x3001, 0x2a},
	{0x3002, 0x88},
	{0x3005, 0x00},
	{0x3011, 0x21},
	{0x3015, 0xC8},
	{0x301b, 0xb4},
	{0x301d, 0x02},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3081, 0x02},
	{0x3083, 0x01},
	{0x3090, 0x02},
	{0x3091, 0x11},
	{0x3092, 0x00},
	{0x3093, 0x01},
	{0x3098, 0x03},
	{0x3099, 0x1e},
	{0x309a, 0x00},
	{0x309b, 0x00},
	{0x30a2, 0x01},
	{0x30b0, 0x05},
	{0x30b2, 0x00},
	{0x30b3, 0x78},
	{0x30b4, 0x03},
	{0x30b5, 0x04},
	{0x30b6, 0x01},
	{0x3104, 0xa1},
	{0x3106, 0x01},
	{0x3400, 0x04},
	{0x3401, 0x00},
	{0x3402, 0x04},
	{0x3403, 0x00},
	{0x3404, 0x04},
	{0x3405, 0x00},
	{0x3406, 0x01},
	{0x3500, 0x00},
	{0x3501, 0x9a},
	{0x3502, 0x80},
	{0x3503, 0x07},
	{0x3504, 0x00},
	{0x3505, 0x30},
	{0x3506, 0x00},
	{0x3507, 0x10},
	{0x3508, 0x80},
	{0x3509, 0x10},
	{0x350a, 0x00},
	{0x350b, 0x38},
	{0x3600, 0x78},
	{0x3601, 0x02},
	{0x3602, 0x1c},
	{0x3604, 0x38},
	{0x3620, 0x64},
	{0x3621, 0xb5},
	{0x3622, 0x03},
	{0x3625, 0x64},
	{0x3630, 0x55},
	{0x3631, 0xd2},
	{0x3632, 0x00},
	{0x3633, 0x34},
	{0x3634, 0x03},
	{0x3660, 0x80},
	{0x3662, 0x10},
	{0x3665, 0x00},
	{0x3666, 0x00},
	{0x3667, 0x00},
	{0x366a, 0x80},
	{0x366c, 0x00},
	{0x366d, 0x00},
	{0x366e, 0x00},
	{0x366f, 0x20},
	{0x3680, 0xe0},
	{0x3681, 0x00},
	{0x3701, 0x14},
	{0x3702, 0xbf},
	{0x3703, 0x8c},
	{0x3704, 0x78},
	{0x3705, 0x02},
	{0x3708, 0xe2},
	{0x3709, 0x03},
	{0x370a, 0x00},
	{0x370b, 0x20},
	{0x370c, 0x0c},
	{0x370d, 0x11},
	{0x370e, 0x00},
	{0x370f, 0x00},
	{0x3710, 0x00},
	{0x371c, 0x01},
	{0x371f, 0x0c},
	{0x3721, 0x00},
	{0x3724, 0x10},
	{0x3726, 0x00},
	{0x372a, 0x01},
	{0x3730, 0x18},
	{0x3738, 0x22},
	{0x3739, 0x08},
	{0x373a, 0x51},
	{0x373b, 0x02},
	{0x373c, 0x20},
	{0x373f, 0x02},
	{0x3740, 0x42},
	{0x3741, 0x02},
	{0x3742, 0x18},
	{0x3743, 0x01},
	{0x3744, 0x02},
	{0x3747, 0x10},
	{0x374c, 0x04},
	{0x3751, 0xf0},
	{0x3752, 0x00},
	{0x3753, 0x00},
	{0x3754, 0xc0},
	{0x3755, 0x00},
	{0x3756, 0x1a},
	{0x3758, 0x00},
	{0x3759, 0x0f},
	{0x375c, 0x04},
	{0x3767, 0x01},
	{0x376b, 0x44},
	{0x3774, 0x10},
	{0x3776, 0x00},
	{0x377f, 0x08},
	{0x3780, 0x22},
	{0x3781, 0x0c},
	{0x3784, 0x2c},
	{0x3785, 0x1e},
	{0x378f, 0xf5},
	{0x3791, 0xb0},
	{0x3795, 0x00},
	{0x3796, 0x64},
	{0x3797, 0x11},
	{0x3798, 0x30},
	{0x3799, 0x41},
	{0x379a, 0x07},
	{0x379b, 0xb0},
	{0x379c, 0x0c},
	{0x37c5, 0x00},
	{0x37c6, 0xa0},
	{0x37c7, 0x00},
	{0x37c9, 0x00},
	{0x37ca, 0x00},
	{0x37cb, 0x00},
	{0x37cc, 0x00},
	{0x37cd, 0x00},
	{0x37ce, 0x01},
	{0x37cf, 0x00},
	{0x37d1, 0x01},
	{0x37de, 0x00},
	{0x37df, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x0c},
	{0x3802, 0x00},
	{0x3803, 0x0c},
	{0x3804, 0x0c},
	{0x3805, 0xd3},
	{0x3806, 0x09},
	{0x3807, 0xa3},
	{0x3808, 0x0c},
	{0x3809, 0xc0},
	{0x380a, 0x09},
	{0x380b, 0x90},
	{0x380c, 0x0e},
	{0x380d, 0x18},
	{0x380e, 0x09},
	{0x380f, 0xb4},
	{0x3810, 0x00},
	{0x3811, 0x04},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x10},
	{0x3821, 0x0e},
	{0x3823, 0x00},
	{0x3824, 0x00},
	{0x3825, 0x00},
	{0x3826, 0x00},
	{0x3827, 0x00},
	{0x382a, 0x04},
	{0x3a04, 0x09},
	{0x3a05, 0xa9},
	{0x3a06, 0x00},
	{0x3a07, 0xf8},
	{0x3b00, 0x00},
	{0x3b02, 0x00},
	{0x3b03, 0x00},
	{0x3b04, 0x00},
	{0x3b05, 0x00},
	{0x3d00, 0x00},
	{0x3d01, 0x00},
	{0x3d02, 0x00},
	{0x3d03, 0x00},
	{0x3d04, 0x00},
	{0x3d05, 0x00},
	{0x3d06, 0x00},
	{0x3d07, 0x00},
	{0x3d08, 0x00},
	{0x3d09, 0x00},
	{0x3d0a, 0x00},
	{0x3d0b, 0x00},
	{0x3d0c, 0x00},
	{0x3d0d, 0x00},
	{0x3d0e, 0x00},
	{0x3d0f, 0x00},
	{0x3d80, 0x00},
	{0x3d81, 0x00},
	{0x3d84, 0x00},
	{0x4000, 0x18},
	{0x4001, 0x04},
	{0x4002, 0x45},
	{0x4004, 0x02},
	{0x4005, 0x1A},
	{0x4006, 0x16},
	{0x4008, 0x20},
	{0x4009, 0x10},
	{0x4101, 0x12},
	{0x4104, 0x5b},
	{0x4307, 0x30},
	{0x4315, 0x00},
	{0x4511, 0x05},
	{0x4512, 0x01},
	{0x4805, 0x01},
	{0x4806, 0x00},
	{0x481f, 0x36},
	{0x4831, 0x6c},
	{0x4837, 0x08},
	{0x4a00, 0xaa},
	{0x4a03, 0x01},
	{0x4a05, 0x08},
	{0x4a0a, 0x88},
	{0x5000, 0x06},
	{0x5001, 0x01},
	{0x5002, 0x80},
	{0x5003, 0x20},
	{0x5013, 0x00},
	{0x5046, 0x4a},
	{0x5780, 0x1c},
	{0x5786, 0x20},
	{0x5787, 0x10},
	{0x5788, 0x18},
	{0x578a, 0x04},
	{0x578b, 0x02},
	{0x578c, 0x02},
	{0x578e, 0x06},
	{0x578f, 0x02},
	{0x5790, 0x02},
	{0x5791, 0xff},
	{0x5a08, 0x02},
	{0x5e00, 0x00},
	{0x5e10, 0x0c},
};


static struct v4l2_subdev_info ov8830_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array ov8830_init_conf[] = {
	{&ov8830_recommended_settings[0],
	ARRAY_SIZE(ov8830_recommended_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_camera_i2c_conf_array ov8830_confs[] = {
	{&ov8830_snap_settings[0],
	ARRAY_SIZE(ov8830_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov8830_snap_settings[0],
	ARRAY_SIZE(ov8830_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov8830_1080p_settings[0],
	ARRAY_SIZE(ov8830_1080p_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t ov8830_dimensions[] = {
	{
		.x_output = 0xCC0, /* 3264 */
		.y_output = 0x990, /* 2448 */
		.line_length_pclk = 0xE18, /* 3608 */
		.frame_length_lines = 0x9B4, /* 2484 */
		.vt_pixel_clk = 204000000,
		.op_pixel_clk = 240000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0xCC0, /* 3264 */
		.y_output = 0x990, /* 2448 */
		.line_length_pclk = 0xE18, /* 3608 */
		.frame_length_lines = 0x9B4, /* 2484 */
		.vt_pixel_clk = 204000000,
		.op_pixel_clk = 240000000,
		.binning_factor = 1,
	},
	{
		.x_output = 0x780, /* 1920 */
		.y_output = 0x440, /* 1088 */
		.line_length_pclk = 0xE18, /* 3608 */
		.frame_length_lines = 0x7CC, /* 1996 */
		.vt_pixel_clk = 216000000,
		.op_pixel_clk = 240000000,
		.binning_factor = 1,
	},
};

static struct msm_sensor_output_reg_addr_t ov8830_reg_addr = {
	.x_output = 0x3808,
	.y_output = 0x380a,
	.line_length_pclk = 0x380c,
	.frame_length_lines = 0x380e,
};

static struct msm_sensor_id_info_t ov8830_id_info = {
	.sensor_id_reg_addr = 0x300A,
	.sensor_id = 0x8830,
};

static struct msm_sensor_exp_gain_info_t ov8830_exp_gain_info = {
	.coarse_int_time_addr = 0x3500,
	.global_gain_addr = 0x350A,
	.vert_offset = 6,
};

static enum msm_camera_vreg_name_t ov8830_veg_seq[] = {
	CAM_VIO,
	CAM_VANA,
	CAM_VDIG,
	CAM_VAF,
};

static void ov8830_adjust_frame_lines(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t cur_line = 0;
	uint16_t exp_fl_lines = 0;
	uint8_t int_time[3];
	if (s_ctrl->sensor_exp_gain_info) {
		msm_camera_i2c_read_seq(s_ctrl->sensor_i2c_client,
			s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			&int_time[0], 3);
		cur_line = int_time[0] << 12;
		cur_line |= int_time[1] << 4;
		cur_line |= int_time[2] >> 4;

		CDBG("%s: sensor cur_line: %d\n", __func__, cur_line);
		exp_fl_lines = cur_line +
			s_ctrl->sensor_exp_gain_info->vert_offset;
		if (exp_fl_lines > s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].frame_length_lines) {
			exp_fl_lines += (exp_fl_lines & 0x01);
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				s_ctrl->sensor_output_reg_addr->
				frame_length_lines,
				exp_fl_lines,
				MSM_CAMERA_I2C_WORD_DATA);
		}

		CDBG("%s cur_fl_lines %d, exp_fl_lines %d\n", __func__,
			s_ctrl->msm_sensor_reg->
			output_settings[s_ctrl->curr_res].frame_length_lines,
			exp_fl_lines);
	}
	return;
}

static int32_t ov8830_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;
	fl_lines += (fl_lines & 0x1);

	CDBG("%s: gain(0x%x) line(0x%x) fl_lines(0x%x)\n",
		__func__, gain, line, fl_lines);
	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);

	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);

	int_time[0] = line >> 12;
	int_time[1] = line >> 4;
	int_time[2] = line << 4;

	msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
		&int_time[0], 3);

	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);

	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);

	return 0;
}

static const struct i2c_device_id ov8830_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ov8830_s_ctrl},
	{ }
};

static struct i2c_driver ov8830_i2c_driver = {
	.id_table = ov8830_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8830_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&ov8830_i2c_driver);
}

static struct v4l2_subdev_core_ops ov8830_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov8830_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov8830_subdev_ops = {
	.core = &ov8830_subdev_core_ops,
	.video  = &ov8830_subdev_video_ops,
};

static struct msm_sensor_fn_t ov8830_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = ov8830_write_exp_gain,
	.sensor_write_snapshot_exp_gain = ov8830_write_exp_gain,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_adjust_frame_lines = ov8830_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t ov8830_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ov8830_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ov8830_start_settings),
	.stop_stream_conf = ov8830_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ov8830_stop_settings),
	.group_hold_on_conf = ov8830_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ov8830_groupon_settings),
	.group_hold_off_conf = ov8830_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(ov8830_groupoff_settings),
	.init_settings = &ov8830_init_conf[0],
	.init_size = ARRAY_SIZE(ov8830_init_conf),
	.mode_settings = &ov8830_confs[0],
	.output_settings = &ov8830_dimensions[0],
	.num_conf = ARRAY_SIZE(ov8830_confs),
};

static struct msm_sensor_ctrl_t ov8830_s_ctrl = {
	.msm_sensor_reg = &ov8830_regs,
	.sensor_i2c_client = &ov8830_sensor_i2c_client,
	.sensor_i2c_addr = 0x6C,
	.vreg_seq = ov8830_veg_seq,
	.num_vreg_seq = ARRAY_SIZE(ov8830_veg_seq),
	.sensor_output_reg_addr = &ov8830_reg_addr,
	.sensor_id_info = &ov8830_id_info,
	.sensor_exp_gain_info = &ov8830_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &ov8830_mut,
	.sensor_i2c_driver = &ov8830_i2c_driver,
	.sensor_v4l2_subdev_info = ov8830_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8830_subdev_info),
	.sensor_v4l2_subdev_ops = &ov8830_subdev_ops,
	.func_tbl = &ov8830_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Ominivison 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
