/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2012 Sony Mobile Communications AB.
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
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>
#include "devices.h"
#include "board-8960.h"

static struct gpiomux_setting gpio_2ma_no_pull_low = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir  = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting gpio_2ma_no_pull_in = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir  = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_2ma_pull_up_in = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir  = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_2ma_pull_down_in = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir  = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_2ma_no_pull_high = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir  = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting cdc_mclk = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi3 = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi4 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi10 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi12 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cam_mclk0 = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cam_mclk1 = {
	.func = GPIOMUX_FUNC_2,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_suspend_1_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir  = GPIOMUX_IN,
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_suspend_2_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hdmi_active_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting debug_uart_tx = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE
};

static struct gpiomux_setting debug_uart_rx = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP
};

static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config semc_viskan_all_cfgs[] __initdata = {
	{ /* RESET_STATUS_N */
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* MCAM_RST_N */
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* FLASH_DR_RST_N */
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* FLASH_TRG */
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_down_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* CAM_MCLK1 */
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_mclk1,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* CAM_MCLK0 */
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_mclk0,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* TP_RESET_N */
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_high,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_high,
		},
	},
	{ /* NC */
		.gpio = 7,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 8,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 9,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* ACCEL_INT */
		.gpio = 10,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* TP_INT */
		.gpio = 11,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_up_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_up_in,
		},
	},
	{ /* HW_ID0 */
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* HW_ID1 */
		.gpio = 13,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* HW_ID2 */
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* HW_ID3 */
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* I2C_DATA_TP */
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi3,
			[GPIOMUX_SUSPENDED] = &gsbi3,
		},
	},
	{ /* I2C_CLK_TP */
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi3,
			[GPIOMUX_SUSPENDED] = &gsbi3,
		},
	},
	{ /* CHAT_CAM_RST_N */
		.gpio = 18,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NFC_DWLD_EN */
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* I2C_DATA_CAM */
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi4,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* I2C_CLK_CAM */
		.gpio = 21,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi4,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* NC */
		.gpio = 22,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 23,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 24,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 25,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},

	/* NOT CONFIGURED: 26-31 */
	/* 26: FM_SSBI (Follow QCT) */
	/* 27: FM_DATA (Follow QCT) */
	/* 28: BT_CTL (Follow QCT) */
	/* 29: BT_DATA (Follow QCT) */
	/* 30: UIM1_DATA_MSM (Follow QCT) */
	/* 31: UIM1_CLK_MSM (Follow QCT) */

	{ /* NC */
		.gpio = 32,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 33,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* UART_TX_DFMS */
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE] = &debug_uart_tx,
			[GPIOMUX_SUSPENDED] = &debug_uart_tx,
		},
	},
	{ /* UART_RX_DTMS */
		.gpio = 35,
		.settings = {
			[GPIOMUX_ACTIVE] = &debug_uart_rx,
			[GPIOMUX_SUSPENDED] = &debug_uart_rx,
		},
	},
	{ /* LCD_DCDC_EN */
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* OTG_OVRCUR_DET_N */
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* NC */
		.gpio = 38,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 39,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 40,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 41,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 42,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 43,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* I2C_DATA_SENS */
		.gpio = 44,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi12,
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},
	{ /* I2C_CLK_SENS */
		.gpio = 45,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi12,
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},
	{ /* DEBUG_GPIO0 */
		.gpio = 46,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* DEBUG_GPIO1 */
		.gpio = 47,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* GYRO_FSYNC */
		.gpio = 48,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* PROX_INT_N */
		.gpio = 49,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_up_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_up_in,
		},
	},
	{ /* NC */
		.gpio = 50,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* USB_OTG_EN */
		.gpio = 51,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NC */
		.gpio = 52,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 53,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* LMU_HW_EN */
		.gpio = 54,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_high,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_high,
		},
	},
	{ /* NC */
		.gpio = 55,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},
	{ /* NC */
		.gpio = 56,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},

	/* 57 not configured */
	/* 57: WTRO_APT_EN (Follow QCT) */

	{ /* BMU_INT_N */
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_up_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_up_in,
		},
	},
	{ /* SLIMBUS1_MCLK (Follow QCT) */
		.gpio = 59,
		.settings = {
			[GPIOMUX_SUSPENDED] = &cdc_mclk,
		},
	},
	{ /* SLIMBUS1_CLK (Follow QCT) */
		.gpio = 60,
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{ /* SLIMBUS1_DATA (Follow QCT) */
		.gpio = 61,
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},

	/* 62 not configured */
	/* 62: CODEC_MAD_INT_N (Follow QCT) */

	{ /* NC */
		.gpio = 63,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* MHL_RST_N */
		.gpio = 64,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_high,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_high,
		},
	},
	{ /* MHL_INT */
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_up_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_up_in,
		},
	},
	{ /* LCD_ID */
		.gpio = 66,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* NC */
		.gpio = 67,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 68,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* GYRO_INT_N */
		.gpio = 69,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* COMPASS_INT */
		.gpio = 70,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_down_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* NC */
		.gpio = 71,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 72,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* I2C_DATA_PERI */
		.gpio = 73,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi10,
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	{ /* I2C_CLK_PERI */
		.gpio = 74,
		.settings = {
			[GPIOMUX_ACTIVE] = &gsbi10,
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	{ /* NC */
		.gpio = 75,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 76,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 77,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC (TP_SWIO)*/
		.gpio = 78,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 79,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 80,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 81,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 82,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},

	/* 83 not configured */
	/* 83: BT_SSBI (Follow QCT) */

	{/* WL_CMD_DATA2 (Follow QCT) */
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{/* WL_CMD_DATA1 (Follow QCT) */
		.gpio = 85,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{/* WL_CMD_DATA0 (Follow QCT) */
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{/* WL_CMD_SET (Follow QCT) */
		.gpio = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{/* WL_CMD_CLK (Follow QCT) */
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{ /* NC */
		.gpio = 89,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 90,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* GSM_TX_MASK */
		.gpio = 91,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},

	{ /* NC (ANT_TUNE0)*/
		.gpio = 92,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},

	{ /* NC */
		.gpio = 93,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 94,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 95,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 96,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 97,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 98,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 99,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* HDMI_DDCSCL */
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_1_cfg,
		},
	},
	{ /* HDMI_DDCSCA */
		.gpio = 101,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_1_cfg,
		},
	},
	{ /* HDMI_HPD (Follow QCT) */
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_2_cfg,
		},
	},

	/* 103-105 not configured */
	/* 103: PM_APC_SEC_IRQ_N (Follow QCT) */
	/* 104: PM_APC_USR_IRQ_N (Follow QCT) */
	/* 105: PM_MDM_IRQ_N (Follow QCT) */

	{ /* NFC_IRQ */
		.gpio = 106,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{ /* SW_SERVICE */
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_up_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_up_in,
		},
	},

	/* 108 not configured */
	/* 108: PS_HOLD (Follow QCT) */

	{ /* NC */
		.gpio = 109,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},
	{ /* NC */
		.gpio = 110,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},

	/* 111-119 not configured */
	/* 111: PRX_SW_SEL/WDOG_DISABLE (Follow QCT) */
	/* 112: ANT_SW_SEL4 (Follow QCT) */
	/* 113: DRX_MODE_SEL2 (Follow QCT) */
	/* 114: DRX_MODE_SEL1 (Follow QCT) */
	/* 115: DRX_MODE_SEL0 (Follow QCT) */
	/* 116: ANT_SW_SEL3 (Follow QCT) */
	/* 117: ANT_SW_SEL2 (Follow QCT) */
	/* 118: ANT_SW_SEL1 (Follow QCT) */
	/* 119: ANT_SW_SEL0 (Follow QCT) */

	{ /* NC (PA1_R0) */
		.gpio = 120,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},
	{ /* NC (PA1_R1) */
		.gpio = 121,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},

	/* 122-123 not configured */
	/* 122: PA0_R0 (Follow QCT) */
	/* 123: PA0_R1 (Follow QCT) */

	{ /* NC */
		.gpio = 124,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},

	/* 125-134 not configured */
	/* 125: WTR0_RF_ON (Follow QCT) */
	/* 126: WTRO_RX_ON (Follow QCT) */
	/* 127: APT0_VCON (Follow QCT) */
	/* 128: PA_ON8 (Follow QCT) */
	/* 129: PA_ON7 (Follow QCT) */
	/* 130: PA_ON6 (Follow QCT) */
	/* 131: PA_ON5 (Follow QCT) */
	/* 132: PA_ON4 (Follow QCT) */
	/* 133: PA_ON3 (Follow QCT) */
	/* 134: PA_ON2 (Follow QCT) */

	{ /* NC */
		.gpio = 135,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},

	/* 136-137 not configured */
	/* 136: PA_ON0 (Follow QCT) */
	/* 137: GPS_LNA_EN (Follow QCT) */

	{ /* NC */
		.gpio = 138,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},

	/* 139 not configured */
	/* 139: DRX_SW_SEL0 (Follow QCT) */

	{ /* NC */
		.gpio = 140,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},
	{ /* NC */
		.gpio = 141,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},

	/* 142-143 not configured */
	/* 142: WTR0_SSBI_PRX_DRX (Follow QCT) */
	/* 143: WTR0_SSBI_TX_GNSS (Follow QCT) */

	{ /* NC */
		.gpio = 144,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 145,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},

	/* 146-148 not configured */
	/* 146: WTR0 GPDATA2 (Follow QCT) */
	/* 147: WTR0 GPDATA1 (Follow QCT) */
	/* 148: WTR0 GPDATA0 (Follow QCT) */

	{ /* NC */
		.gpio = 149,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in, },
	},
	{ /* NC */
		.gpio = 150,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
	{ /* NC */
		.gpio = 151,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},
};



void __init gpiomux_device_install(void)
{
	msm_gpiomux_install(semc_viskan_all_cfgs,
			ARRAY_SIZE(semc_viskan_all_cfgs));
}

int __init msm8960_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return rc;
	}

	gpiomux_device_install();

	return 0;
}
