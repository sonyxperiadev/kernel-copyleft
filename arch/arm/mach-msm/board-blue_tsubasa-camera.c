/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
 *
 */

#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/camera.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>
#include <mach/gpiomux.h>
#include "devices.h"
#include "board-8960.h"
#ifdef CONFIG_SONY_CAM_V4L2
#include <media/sony_camera_v4l2.h>
#include <linux/v4l2-mediabus.h>
#endif

#ifdef CONFIG_MSM_CAMERA

#if !defined(CONFIG_SONY_CAM_V4L2)
#ifdef CONFIG_MSM_CAMERA_FLASH
#if (defined(CONFIG_GPIO_SX150X) || defined(CONFIG_GPIO_SX150X_MODULE)) && \
	defined(CONFIG_I2C)

static struct i2c_board_info cam_expander_i2c_info[] = {
	{
		I2C_BOARD_INFO("sx1508q", 0x22),
		.platform_data = &msm8960_sx150x_data[SX150X_CAM]
	},
};

static struct msm_cam_expander_info cam_expander_info[] = {
	{
		cam_expander_i2c_info,
		MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	},
};
#endif
#endif
#endif

#if defined(CONFIG_SONY_CAM_V4L2)
static struct gpiomux_setting cam_mclk = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_2ma_no_pull_low = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting cam_mclk1 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_2ma_pull_down_in = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gsbi4 = {
	.func = GPIOMUX_FUNC_1,  /* gsbi4 */
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};
#else
static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 1*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 2*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 3*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_5, /*active 4*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_6, /*active 5*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 6*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_3, /*active 7*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*i2c suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

};
#endif

#if !defined(CONFIG_SONY_CAM_V4L2)
static struct msm_gpiomux_config msm8960_cdp_flash_configs[] = {
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[1],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
};

static struct msm_gpiomux_config msm8960_cam_common_configs[] = {
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[1],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 76,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
};
#endif

static struct msm_gpiomux_config msm8960_cam_2d_configs[] = {
#if defined(CONFIG_SONY_CAM_V4L2)
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_mclk1,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_mclk,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi4,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi4,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
#else
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
#endif
};

#if !defined(CONFIG_SONY_CAM_V4L2)
static struct msm_gpiomux_config msm8960_cam_2d_configs_sglte[] = {
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
};

#define VFE_CAMIF_TIMER1_GPIO 2
#define VFE_CAMIF_TIMER2_GPIO 3
#define VFE_CAMIF_TIMER3_GPIO_INT 4
static struct msm_camera_sensor_strobe_flash_data strobe_flash_xenon = {
	.flash_trigger = VFE_CAMIF_TIMER2_GPIO,
	.flash_charge = VFE_CAMIF_TIMER1_GPIO,
	.flash_charge_done = VFE_CAMIF_TIMER3_GPIO_INT,
	.flash_recharge_duration = 50000,
	.irq = MSM_GPIO_TO_INT(VFE_CAMIF_TIMER3_GPIO_INT),
};

#ifdef CONFIG_MSM_CAMERA_FLASH
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_EXT,
	._fsrc.ext_driver_src.led_en = VFE_CAMIF_TIMER1_GPIO,
	._fsrc.ext_driver_src.led_flash_en = VFE_CAMIF_TIMER2_GPIO,
	._fsrc.ext_driver_src.flash_id = MAM_CAMERA_EXT_LED_FLASH_SC628A,
};
#endif
#endif

static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 139968000,
		.ib = 559872000,
#else
		.ab  = 27648000,
		.ib  = 2656000000UL,
#endif
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 279936000,
		.ib = 699840000,
#else
		.ab  = 0,
		.ib  = 0,
#endif
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 121305600,
		.ib = 485222400,
#else
		.ab  = 600000000,
		.ib  = 2656000000UL,
#endif
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 214617600,
		.ib = 494553600,
#else
		.ab  = 206807040,
		.ib  = 488816640,
#endif
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 295401600,
		.ib = 1181606400,
#else
		.ab  = 600000000,
		.ib  = 2656000000UL,
#endif
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 590803200,
		.ib = 1477008000,
#else
		.ab  = 0,
		.ib  = 0,
#endif
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 590803200,
		.ib = 590803200,
#else
		.ab  = 540000000,
		.ib  = 1350000000,
#endif
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 600000000,
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
#if defined(CONFIG_SONY_CAM_V4L2)
		.ab = 1521190000,
		.ib = 1521190000,
#else
		.ab  = 540000000,
		.ib  = 1350000000,
#endif
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};

static struct msm_bus_vectors cam_video_ls_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 348192000,
		.ib  = 617103360,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_dual_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 600000000,
		.ib  = 2656000000UL,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 206807040,
		.ib  = 488816640,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 540000000,
		.ib  = 1350000000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_MM_IMEM,
		.ab  = 43200000,
		.ib  = 69120000,
	},
};



static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_preview_vectors),
		cam_preview_vectors,
	},
	{
		ARRAY_SIZE(cam_video_vectors),
		cam_video_vectors,
	},
	{
		ARRAY_SIZE(cam_snapshot_vectors),
		cam_snapshot_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_video_ls_vectors),
		cam_video_ls_vectors,
	},
	{
		ARRAY_SIZE(cam_dual_vectors),
		cam_dual_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

static struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
		.csid_core = 0,
#if defined(CONFIG_SONY_VPE)
		.is_vpe = 0,
#else
		.is_vpe    = 1,
#endif
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 1,
#if defined(CONFIG_SONY_VPE)
		.is_vpe = 0,
#else
		.is_vpe    = 1,
#endif
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
	{
		.csid_core = 2,
#if defined(CONFIG_SONY_VPE)
		.is_vpe = 0,
#else
		.is_vpe    = 1,
#endif
		.cam_bus_scale_table = &cam_bus_client_pdata,
	},
};

#ifdef CONFIG_SONY_CAM_V4L2
static struct camera_vreg_t msm_8960_back_cam_vreg[] = {
	{ "cam_vana", REG_LDO, 2600000, 3000000, 85600 },
	{ "cam_vio", REG_VS, 0, 0, 0 },
	{ "cam_vdig", REG_LDO, 1200000, 1200000, 105000 },
	{ "cam_vaf", REG_LDO, 2600000, 3000000, 300000 },
};

static struct camera_vreg_t msm_8960_front_cam_vreg[] = {
	{ "cam_vana", REG_LDO, 2600000, 3000000, 85600 },
	{ "cam_vio", REG_VS, 0, 0, 0 },
};
#else
static struct camera_vreg_t msm_8960_cam_vreg[] = {
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000},
	{"cam_vio", REG_VS, 0, 0, 0},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600},
	{"cam_vaf", REG_LDO, 2800000, 2800000, 300000},
};
#endif

static struct gpio msm8960_common_cam_gpio[] = {
#if defined(CONFIG_SONY_CAM_V4L2)
	{ 4, GPIOF_DIR_IN, "CAMIF_MCLK" },
#endif
	{5, GPIOF_DIR_IN, "CAMIF_MCLK"},
	{20, GPIOF_DIR_IN, "CAMIF_I2C_DATA"},
	{21, GPIOF_DIR_IN, "CAMIF_I2C_CLK"},
};

static struct gpio msm8960_front_cam_gpio[] = {
#ifdef CONFIG_SONY_CAM_V4L2
	{ 18, GPIOF_DIR_OUT, "CAM_RESET" },
#else
	{76, GPIOF_DIR_OUT, "CAM_RESET"},
#endif
};

static struct gpio msm8960_back_cam_gpio[] = {
#ifdef CONFIG_SONY_CAM_V4L2
	{ 1, GPIOF_DIR_OUT, "CAM_RESET" },
#else
	{107, GPIOF_DIR_OUT, "CAM_RESET"},
#endif
};

static struct msm_gpio_set_tbl msm8960_front_cam_gpio_set_tbl[] = {
#ifdef CONFIG_SONY_CAM_V4L2
	{ 18, GPIOF_OUT_INIT_LOW, 1 },
	{ 18, GPIOF_OUT_INIT_HIGH, 1000 },
#else
	{76, GPIOF_OUT_INIT_LOW, 1000},
	{76, GPIOF_OUT_INIT_HIGH, 4000},
#endif
};

static struct msm_gpio_set_tbl msm8960_back_cam_gpio_set_tbl[] = {
#ifdef CONFIG_SONY_CAM_V4L2
	{ 1, GPIOF_OUT_INIT_LOW, 1 },
	{ 1, GPIOF_OUT_INIT_HIGH, 1000 },
#else
	{107, GPIOF_OUT_INIT_LOW, 1000},
	{107, GPIOF_OUT_INIT_HIGH, 4000},
#endif
};

static struct msm_camera_gpio_conf msm_8960_front_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = msm8960_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(msm8960_cam_2d_configs),
	.cam_gpio_common_tbl = msm8960_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(msm8960_common_cam_gpio),
	.cam_gpio_req_tbl = msm8960_front_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(msm8960_front_cam_gpio),
	.cam_gpio_set_tbl = msm8960_front_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(msm8960_front_cam_gpio_set_tbl),
};

static struct msm_camera_gpio_conf msm_8960_back_cam_gpio_conf = {
	.cam_gpiomux_conf_tbl = msm8960_cam_2d_configs,
	.cam_gpiomux_conf_tbl_size = ARRAY_SIZE(msm8960_cam_2d_configs),
	.cam_gpio_common_tbl = msm8960_common_cam_gpio,
	.cam_gpio_common_tbl_size = ARRAY_SIZE(msm8960_common_cam_gpio),
	.cam_gpio_req_tbl = msm8960_back_cam_gpio,
	.cam_gpio_req_tbl_size = ARRAY_SIZE(msm8960_back_cam_gpio),
	.cam_gpio_set_tbl = msm8960_back_cam_gpio_set_tbl,
	.cam_gpio_set_tbl_size = ARRAY_SIZE(msm8960_back_cam_gpio_set_tbl),
};

#if defined(CONFIG_SONY_CAM_V4L2)
static struct msm_camera_sensor_flash_data msm_camera_flash_none = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src = NULL,
};

static struct msm_camera_csi_lane_params msm_camera_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};
#endif

#ifdef CONFIG_SONY_CAM_V4L2
static struct msm_camera_sensor_platform_info
		msm_camera_sony_sensor_main_platform_info = {
	.mount_angle = 90,
	.sensor_reset = 1,
	.cam_vreg = msm_8960_back_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_back_cam_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &msm_camera_csi_lane_params,
};

static struct msm_camera_sensor_info
		msm_camera_sony_sensor_main_sensor_info = {
	.sensor_name = "sony_camera_0",
	.sensor_reset = 1,
	.pdata = &msm_camera_csi_device_data[0],
	.flash_data = &msm_camera_flash_none,
	.sensor_platform_info = &msm_camera_sony_sensor_main_platform_info,
	.csi_if = 1,
	.camera_type = BACK_CAMERA_2D,
};
#else
static struct i2c_board_info msm_act_main_cam_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x11),
};

static struct msm_actuator_info msm_act_main_cam_0_info = {
	.board_info     = &msm_act_main_cam_i2c_info,
	.cam_name   = MSM_ACTUATOR_MAIN_CAM_0,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

static struct i2c_board_info msm_act_main_cam1_i2c_info = {
	I2C_BOARD_INFO("msm_actuator", 0x18),
};

static struct msm_actuator_info msm_act_main_cam_1_info = {
	.board_info     = &msm_act_main_cam1_i2c_info,
	.cam_name       = MSM_ACTUATOR_MAIN_CAM_1,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

static struct msm_camera_sensor_flash_data flash_imx074 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src
#endif
};

static struct msm_camera_csi_lane_params imx074_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx074 = {
	.mount_angle	= 90,
	.cam_vreg = msm_8960_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_cam_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &imx074_csi_lane_params,
};

static struct i2c_board_info imx074_eeprom_i2c_info = {
	I2C_BOARD_INFO("imx074_eeprom", 0x34 << 1),
};

static struct msm_eeprom_info imx074_eeprom_info = {
	.board_info     = &imx074_eeprom_i2c_info,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
};

static struct msm_camera_sensor_info msm_camera_sensor_imx074_data = {
	.sensor_name	= "imx074",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_imx074,
	.strobe_flash_data = &strobe_flash_xenon,
	.sensor_platform_info = &sensor_board_info_imx074,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
	.actuator_info = &msm_act_main_cam_0_info,
	.eeprom_info = &imx074_eeprom_info,
};
#endif

#ifdef CONFIG_SONY_CAM_V4L2
static struct msm_camera_sensor_platform_info
		msm_camera_sony_sensor_sub_platform_info = {
	.mount_angle = 90,
	.sensor_reset = 18,
	.cam_vreg = msm_8960_front_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_front_cam_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &msm_camera_csi_lane_params,
};

static struct msm_camera_sensor_info
		msm_camera_sony_sensor_sub_sensor_info = {
	.sensor_name = "sony_camera_1",
	.sensor_reset = 18,
	.pdata = &msm_camera_csi_device_data[1],
	.flash_data = &msm_camera_flash_none,
	.sensor_platform_info = &msm_camera_sony_sensor_sub_platform_info,
	.csi_if = 1,
	.camera_type = FRONT_CAMERA_2D,
};
#else
static struct msm_camera_sensor_flash_data flash_mt9m114 = {
	.flash_type = MSM_CAMERA_FLASH_NONE
};

static struct msm_camera_csi_lane_params mt9m114_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x1,
};

static struct camera_vreg_t mt9m114_cam_vreg[] = {
	{"cam_vdig", REG_LDO, 1200000, 1200000, 105000, 50},
	{"cam_vio", REG_VS, 0, 0, 0, 50},
	{"cam_vana", REG_LDO, 2800000, 2850000, 85600, 50},
};

static struct msm_camera_sensor_platform_info sensor_board_info_mt9m114 = {
	.mount_angle = 90,
	.cam_vreg = mt9m114_cam_vreg,
	.num_vreg = ARRAY_SIZE(mt9m114_cam_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &mt9m114_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9m114_data = {
	.sensor_name = "mt9m114",
	.pdata = &msm_camera_csi_device_data[1],
	.flash_data = &flash_mt9m114,
	.sensor_platform_info = &sensor_board_info_mt9m114,
	.csi_if = 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = YUV_SENSOR,
};

static struct msm_camera_sensor_flash_data flash_ov2720 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params ov2720_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0x3,
};

static struct msm_camera_sensor_platform_info sensor_board_info_ov2720 = {
	.mount_angle	= 0,
	.cam_vreg = msm_8960_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_cam_vreg),
	.gpio_conf = &msm_8960_front_cam_gpio_conf,
	.csi_lane_params = &ov2720_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov2720_data = {
	.sensor_name	= "ov2720",
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_ov2720,
	.sensor_platform_info = &sensor_board_info_ov2720,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
};
#endif

#if !defined(CONFIG_SONY_CAM_V4L2)
static struct msm_camera_sensor_flash_data flash_s5k3l1yx = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params s5k3l1yx_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_s5k3l1yx = {
	.mount_angle  = 0,
	.cam_vreg = msm_8960_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_cam_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &s5k3l1yx_csi_lane_params,
};

static struct msm_actuator_info msm_act_main_cam_2_info = {
	.board_info     = &msm_act_main_cam_i2c_info,
	.cam_name   = MSM_ACTUATOR_MAIN_CAM_2,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3l1yx_data = {
	.sensor_name          = "s5k3l1yx",
	.pdata                = &msm_camera_csi_device_data[0],
	.flash_data           = &flash_s5k3l1yx,
	.sensor_platform_info = &sensor_board_info_s5k3l1yx,
	.csi_if               = 1,
	.camera_type          = BACK_CAMERA_2D,
	.sensor_type          = BAYER_SENSOR,
	.actuator_info    = &msm_act_main_cam_2_info,
};

static struct msm_camera_csi_lane_params imx091_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_flash_data flash_imx091 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src
#endif
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx091 = {
	.mount_angle	= 0,
	.cam_vreg = msm_8960_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_cam_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &imx091_csi_lane_params,
};

static struct i2c_board_info imx091_eeprom_i2c_info = {
	I2C_BOARD_INFO("imx091_eeprom", 0x21),
};

static struct msm_eeprom_info imx091_eeprom_info = {
	.board_info     = &imx091_eeprom_i2c_info,
	.bus_id         = MSM_8960_GSBI4_QUP_I2C_BUS_ID,
	.eeprom_i2c_slave_addr = 0xA1,
	.eeprom_reg_addr = 0x05,
	.eeprom_read_length = 6,
};

static struct msm_camera_sensor_info msm_camera_sensor_imx091_data = {
	.sensor_name	= "imx091",
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_imx091,
	.sensor_platform_info = &sensor_board_info_imx091,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
	.actuator_info = &msm_act_main_cam_1_info,
	.eeprom_info = &imx091_eeprom_info,
};

static struct msm_camera_sensor_flash_data flash_imx135 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_csi_lane_params imx135_csi_lane_params = {
	.csi_lane_assign = 0xE4,
	.csi_lane_mask = 0xF,
};

static struct msm_camera_sensor_platform_info sensor_board_info_imx135 = {
	.mount_angle = 90,
	.cam_vreg = msm_8960_cam_vreg,
	.num_vreg = ARRAY_SIZE(msm_8960_cam_vreg),
	.gpio_conf = &msm_8960_back_cam_gpio_conf,
	.csi_lane_params = &imx135_csi_lane_params,
};

static struct msm_camera_sensor_info msm_camera_sensor_imx135_data = {
	.sensor_name = "imx135",
	.pdata = &msm_camera_csi_device_data[0],
	.flash_data = &flash_imx135,
	.sensor_platform_info = &sensor_board_info_imx135,
	.csi_if = 1,
	.camera_type = BACK_CAMERA_2D,
	.sensor_type = BAYER_SENSOR,
	.actuator_info = &msm_act_main_cam_1_info,
};
#endif

#if !defined(CONFIG_SONY_CAM_V4L2)
static struct pm8xxx_mpp_config_data privacy_light_on_config = {
	.type		= PM8XXX_MPP_TYPE_SINK,
	.level		= PM8XXX_MPP_CS_OUT_5MA,
	.control	= PM8XXX_MPP_CS_CTRL_MPP_LOW_EN,
};

static struct pm8xxx_mpp_config_data privacy_light_off_config = {
	.type		= PM8XXX_MPP_TYPE_SINK,
	.level		= PM8XXX_MPP_CS_OUT_5MA,
	.control	= PM8XXX_MPP_CS_CTRL_DISABLE,
};

static int32_t msm_camera_8960_ext_power_ctrl(int enable)
{
	int rc = 0;
	if (enable) {
		rc = pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(12),
			&privacy_light_on_config);
	} else {
		rc = pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(12),
			&privacy_light_off_config);
	}
	return rc;
}
#endif

static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

#if defined(CONFIG_SONY_VPE)
struct resource sony_vpe_resources[] = {
	{
		.start = 0x05300000,
		.end = 0x05300000 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = VPE_IRQ,
		.end = VPE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device sony_vpe_device = {
	.name = "sony_vpe",
	.id = 0,
	.num_resources = ARRAY_SIZE(sony_vpe_resources),
	.resource = sony_vpe_resources,
};
#endif

void __init msm8960_init_cam(void)
{
#if defined(CONFIG_SONY_CAM_V4L2)
	msm_gpiomux_install(msm8960_cam_2d_configs,
			ARRAY_SIZE(msm8960_cam_2d_configs));
#else
	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE) {
		msm_8960_front_cam_gpio_conf.cam_gpiomux_conf_tbl =
			msm8960_cam_2d_configs_sglte;
		msm_8960_front_cam_gpio_conf.cam_gpiomux_conf_tbl_size =
			ARRAY_SIZE(msm8960_cam_2d_configs_sglte);
		msm_8960_back_cam_gpio_conf.cam_gpiomux_conf_tbl =
			msm8960_cam_2d_configs_sglte;
		msm_8960_back_cam_gpio_conf.cam_gpiomux_conf_tbl_size =
			ARRAY_SIZE(msm8960_cam_2d_configs_sglte);
	}
	msm_gpiomux_install(msm8960_cam_common_configs,
			ARRAY_SIZE(msm8960_cam_common_configs));

	if (machine_is_msm8960_cdp()) {
		msm_gpiomux_install(msm8960_cdp_flash_configs,
			ARRAY_SIZE(msm8960_cdp_flash_configs));
#ifdef CONFIG_MSM_CAMERA_FLASH
		msm_flash_src._fsrc.ext_driver_src.led_en =
			GPIO_CAM_GP_LED_EN1;
		msm_flash_src._fsrc.ext_driver_src.led_flash_en =
			GPIO_CAM_GP_LED_EN2;
		#if defined(CONFIG_I2C) && (defined(CONFIG_GPIO_SX150X) || \
		defined(CONFIG_GPIO_SX150X_MODULE))
		msm_flash_src._fsrc.ext_driver_src.expander_info =
			cam_expander_info;
		#endif
#endif
	}

	if (machine_is_msm8960_liquid()) {
		struct msm_camera_sensor_info *s_info;
		s_info = &msm_camera_sensor_imx074_data;
		s_info->sensor_platform_info->mount_angle = 180;
		s_info = &msm_camera_sensor_ov2720_data;
		s_info->sensor_platform_info->ext_power_ctrl =
			msm_camera_8960_ext_power_ctrl;
	}

	if (machine_is_msm8960_fluid()) {
		msm_camera_sensor_imx091_data.sensor_platform_info->
			mount_angle = 270;
	}
#endif

	platform_device_register(&msm_camera_server);
	platform_device_register(&msm8960_device_csiphy0);
	platform_device_register(&msm8960_device_csiphy1);
	platform_device_register(&msm8960_device_csiphy2);
	platform_device_register(&msm8960_device_csid0);
	platform_device_register(&msm8960_device_csid1);
	platform_device_register(&msm8960_device_csid2);
	platform_device_register(&msm8960_device_ispif);
	platform_device_register(&msm8960_device_vfe);
#if defined(CONFIG_SONY_VPE)
	platform_device_register(&sony_vpe_device);
#else
	platform_device_register(&msm8960_device_vpe);
#endif
}

#ifdef CONFIG_I2C
static struct i2c_board_info msm8960_camera_i2c_boardinfo[] = {
#if defined(CONFIG_SONY_CAM_V4L2)
	{
		I2C_BOARD_INFO("sony_camera_0", 0x1A),
		.platform_data = &msm_camera_sony_sensor_main_sensor_info,
	},
	{
		I2C_BOARD_INFO("sony_camera_1", 0x3d),
		.platform_data = &msm_camera_sony_sensor_sub_sensor_info,
	},
#else
	{
	I2C_BOARD_INFO("imx074", 0x1A),
	.platform_data = &msm_camera_sensor_imx074_data,
	},
	{
	I2C_BOARD_INFO("imx135", 0x10),
	.platform_data = &msm_camera_sensor_imx135_data,
	},
	{
	I2C_BOARD_INFO("ov2720", 0x6C),
	.platform_data = &msm_camera_sensor_ov2720_data,
	},
	{
	I2C_BOARD_INFO("mt9m114", 0x48),
	.platform_data = &msm_camera_sensor_mt9m114_data,
	},
	{
	I2C_BOARD_INFO("s5k3l1yx", 0x20),
	.platform_data = &msm_camera_sensor_s5k3l1yx_data,
	},
#ifdef CONFIG_MSM_CAMERA_FLASH_SC628A
	{
	I2C_BOARD_INFO("sc628a", 0x6E),
	},
#endif
	{
	I2C_BOARD_INFO("imx091", 0x34),
	.platform_data = &msm_camera_sensor_imx091_data,
	},
#endif
};

struct msm_camera_board_info msm8960_camera_board_info = {
	.board_info = msm8960_camera_i2c_boardinfo,
	.num_i2c_board_info = ARRAY_SIZE(msm8960_camera_i2c_boardinfo),
};
#endif

#if defined(CONFIG_SONY_CAM_V4L2)
static const struct sony_camera_seq sensor_main_power_off[] = {
	{ SONY_I2C_WRITE, 0x0100, 100 },
	{ SONY_CAM_CLK, -1, 1 },
	{ SONY_GPIO_RESET, 0, 1 },
	{ SONY_CAM_VAF, -1, 0 },
	{ SONY_CAM_VANA, -1, 1 },
	{ SONY_CAM_VIO, -1, 1 },
	{ SONY_CAM_VDIG, -1, 15 },
	{ EXIT, 0, 0 },
};

static const struct sony_camera_seq sensor_main_power_on[] = {
	{ SONY_CAM_VDIG, 1200, 1 },
	{ SONY_CAM_VIO, 0, 1 },
	{ SONY_CAM_VANA, 2850, 0 },
	{ SONY_CAM_VAF, 2700, 1 },
	{ SONY_GPIO_RESET, 1, 9 },
	{ SONY_CAM_CLK, 0, 1 },
	{ EXIT, 0, 0 },
};

static const struct sony_camera_seq sensor_sub_power_off[] = {
	{ SONY_GPIO_RESET, 0, 1 },
	{ SONY_CAM_CLK, -1, 1 },
	{ SONY_CAM_VANA, -1, 0 },
	{ SONY_CAM_VIO, -1, 0 },
	{ EXIT, 0, 0 },
};

static const struct sony_camera_seq sensor_sub_power_on[] = {
	{ SONY_CAM_VIO, 0, 1 },
	{ SONY_CAM_VANA, 2850, 1 },
	{ SONY_CAM_CLK, 0, 200 },
	{ EXIT, 0, 0 },
};

static const struct sony_camera_module sensor_main_modules[] = {
	{ "GENERIC",  sensor_main_power_on, sensor_main_power_off },
	{ "SOI13BS0", sensor_main_power_on, sensor_main_power_off },
	{ "KMO13BS0", sensor_main_power_on, sensor_main_power_off },
};

static const struct sony_camera_module sensor_sub_modules[] = {
	{ "GENERIC",  sensor_sub_power_on, sensor_sub_power_off },
	{ "APT00YP1", sensor_sub_power_on, sensor_sub_power_off },
	{ "STW00YP1", sensor_sub_power_on, sensor_sub_power_off },
};

const struct sony_camera_info camera_info[] = {
	{
		.i2c_addr = 0x1A,
		.eeprom_addr = 0x50,
		.eeprom_type = 0,
		.eeprom_max_len = 2048,
		.gpio_af = 89,
		.subdev_code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.modules = sensor_main_modules,
		.modules_num = ARRAY_SIZE(sensor_main_modules),
		.default_module_name = "SOI13BS0",
	},
	{
		.i2c_addr = 0x3d,
		.eeprom_addr = 0x3d,
		.eeprom_type = 1,
		.eeprom_max_len = 1024,
		.gpio_af = 0,
		.subdev_code = V4L2_MBUS_FMT_YUYV8_2X8,
		.modules = sensor_sub_modules,
		.modules_num = ARRAY_SIZE(sensor_sub_modules),
		.default_module_name = "STW01BM0",
	},
};
#endif
#endif
