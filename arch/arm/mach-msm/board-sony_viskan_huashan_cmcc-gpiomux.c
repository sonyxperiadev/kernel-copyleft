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

/* The SPI configurations apply to GSBI 1*/
static struct gpiomux_setting spi_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting spi_suspended_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
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

static struct gpiomux_setting external_vfr[] = {
	/* Suspended state */
	{
		.func = GPIOMUX_FUNC_3,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},
	/* Active state */
	{
		.func = GPIOMUX_FUNC_3,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},
};

static struct gpiomux_setting gsbi_uart = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi8_uart_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi8_uart_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi8_uartdm_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi8_uartdm_suspended_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gsbi9_active_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gsbi9_suspended_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gsbi10 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

#ifdef CONFIG_SONY_QSCFLASHING_UART4
static struct gpiomux_setting gsbi11_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi11_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif
static struct gpiomux_setting gsbi12 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cdc_mclk = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static struct gpiomux_setting gpio_eth_config = {
	.pull = GPIOMUX_PULL_NONE,
	.drv = GPIOMUX_DRV_8MA,
	.func = GPIOMUX_FUNC_GPIO,
};
#endif

static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
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

static struct gpiomux_setting ap2mdm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_errfatal_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ap2mdm_kpdpwr_n_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct gpiomux_setting hdmi_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_active_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

#if defined(CONFIG_FB_MSM_HDMI_MHL_8334) || defined(CONFIG_FB_MSM_HDMI_MHL_9244)
static struct gpiomux_setting hdmi_active_3_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting hdmi_active_4_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};
#endif
#endif

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

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static struct msm_gpiomux_config msm8960_ethernet_configs[] = {
	{
		.gpio = 90,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_eth_config,
		}
	},
	{
		.gpio = 89,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_eth_config,
		}
	},
};
#endif
/* GSBI8 UART GPIOs for Atheros Bluetooth */
static struct msm_gpiomux_config msm8960_gsbi8_uartdm_configs[] = {
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uartdm_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi8_uartdm_active_cfg,
		}
	},
	{
		.gpio = 35,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uartdm_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi8_uartdm_active_cfg,
		}
	},
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uartdm_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi8_uartdm_active_cfg,
		}
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uartdm_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi8_uartdm_active_cfg,
		}
	},
};

static struct msm_gpiomux_config msm8960_fusion_gsbi_configs[] = {
	{
		.gpio = 93,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi9_active_cfg,
		}
	},
	{
		.gpio = 94,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi9_active_cfg,
		}
	},
	{
		.gpio = 95,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi9_active_cfg,
		}
	},
	{
		.gpio = 96,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi9_suspended_cfg,
			[GPIOMUX_ACTIVE] = &gsbi9_active_cfg,
		}
	},
};

static struct msm_gpiomux_config msm8960_gsbi_configs[] __initdata = {
	{
		.gpio      = 6,		/* GSBI1 QUP SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	{
		.gpio      = 7,		/* GSBI1 QUP SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	{
		.gpio      = 8,		/* GSBI1 QUP SPI_CS_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	{
		.gpio      = 9,		/* GSBI1 QUP SPI_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &spi_suspended_config,
			[GPIOMUX_ACTIVE] = &spi_active,
		},
	},
	{
		.gpio      = 16,	/* GSBI3 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3,
			[GPIOMUX_ACTIVE] = &gsbi3,
		},
	},
	{
		.gpio      = 17,	/* GSBI3 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3,
			[GPIOMUX_ACTIVE] = &gsbi3,
		},
	},
#ifdef CONFIG_SONY_QSCFLASHING_UART4
	{
		.gpio	   = 38,	/* GSBI11 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
			[GPIOMUX_ACTIVE] = &gsbi11_1_cfg,
		},
	},
	{
		.gpio	   = 39,	/* GSBI11 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
			[GPIOMUX_ACTIVE] = &gsbi11_2_cfg,
		},
	},
#endif
	{
		.gpio      = 44,	/* GSBI12 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},
	{
		.gpio      = 45,	/* GSBI12 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},
	{
		.gpio      = 73,	/* GSBI10 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	{
		.gpio      = 74,	/* GSBI10 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
};

static struct msm_gpiomux_config msm8960_gsbi5_uart_configs[] __initdata = {
	{
		.gpio      = 22,        /* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi_uart,
		},
	},
	{
		.gpio      = 23,        /* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi_uart,
		},
	},
	{
		.gpio      = 24,        /* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi_uart,
		},
	},
	{
		.gpio      = 25,        /* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi_uart,
		},
	},
};

static struct msm_gpiomux_config msm8960_external_vfr_configs[] __initdata = {
	{
		.gpio      = 23,        /* EXTERNAL VFR */
		.settings = {
			[GPIOMUX_SUSPENDED] = &external_vfr[0],
			[GPIOMUX_ACTIVE] = &external_vfr[1],
		},
	},
};

static struct msm_gpiomux_config msm8960_gsbi8_uart_configs[] __initdata = {
	{
		.gpio      = 34,        /* GSBI8 UART3 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uart_1_cfg,
		},
	},
	{
		.gpio      = 35,        /* GSBI8 UART3 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uart_2_cfg,
		},
	},
	{
		.gpio      = 36,        /* GSBI8 UART3 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uart_2_cfg,
		},
	},
	{
		.gpio      = 37,        /* GSBI8 UART3 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi8_uart_2_cfg,
		},
	},
};

static struct msm_gpiomux_config msm8960_slimbus_config[] __initdata = {
	{
		.gpio	= 60,		/* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 61,		/* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
};

static struct msm_gpiomux_config msm8960_audio_codec_configs[] __initdata = {
	{
		.gpio = 59,
		.settings = {
			[GPIOMUX_SUSPENDED] = &cdc_mclk,
		},
	},
};

static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 85,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
};

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct gpiomux_setting sdcc4_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc4_cmd_data_0_3_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc4_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting sdcc4_data_1_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config msm8960_sdcc4_configs[] __initdata = {
	{
		/* SDC4_DATA_3 */
		.gpio      = 83,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc4_suspend_cfg,
		},
	},
	{
		/* SDC4_DATA_2 */
		.gpio      = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc4_suspend_cfg,
		},
	},
	{
		/* SDC4_DATA_1 */
		.gpio      = 85,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc4_data_1_suspend_cfg,
		},
	},
	{
		/* SDC4_DATA_0 */
		.gpio      = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc4_suspend_cfg,
		},
	},
	{
		/* SDC4_CMD */
		.gpio      = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc4_suspend_cfg,
		},
	},
	{
		/* SDC4_CLK */
		.gpio      = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc4_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc4_suspend_cfg,
		},
	},
};
#endif

static struct msm_gpiomux_config sglte_configs[] __initdata = {
	/* AP2MDM_STATUS */
	{
		.gpio = 77,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* MDM2AP_STATUS */
	{
		.gpio = 24,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_cfg,
		}
	},
	/* MDM2AP_ERRFATAL */
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_errfatal_cfg,
		}
	},
	/* AP2MDM_ERRFATAL */
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		}
	},
	/* AP2MDM_KPDPWR_N */
	{
		.gpio = 79,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		}
	},
	/* AP2MDM_PMIC_PWR_EN */
	{
		.gpio = 22,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_kpdpwr_n_cfg,
		}
	},
	/* AP2MDM_SOFT_RESET */
	{
		.gpio = 78,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	/* NC, reserved for SGLTE_USB_SWITCH */
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		}
	}
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct msm_gpiomux_config msm8960_hdmi_configs[] __initdata = {
	{
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{
		.gpio = 101,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
#ifdef CONFIG_FB_MSM_HDMI_MHL_9244
		{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_4_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
#endif
#ifdef CONFIG_FB_MSM_HDMI_MHL_8334
		{
		.gpio = 4,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_4_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
#endif /* CONFIG_FB_MSM_HDMI_MHL */
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct gpiomux_setting sdcc2_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdcc2_cmd_data_0_3_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdcc2_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting sdcc2_data_1_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config msm8960_sdcc2_configs[] __initdata = {
	{
		/* DATA_3 */
		.gpio      = 92,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_cfg,
		},
	},
	{
		/* DATA_2 */
		.gpio      = 91,
		.settings = {
		[GPIOMUX_ACTIVE]    = &sdcc2_cmd_data_0_3_actv_cfg,
		[GPIOMUX_SUSPENDED] = &sdcc2_suspend_cfg,
		},
	},
	{
		/* DATA_1 */
		.gpio      = 90,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_data_1_suspend_cfg,
		},
	},
	{
		/* DATA_0 */
		.gpio      = 89,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_cfg,
		},
	},
	{
		/* CMD */
		.gpio      = 97,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_cfg,
		},
	},
	{
		/* CLK */
		.gpio      = 98,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdcc2_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdcc2_suspend_cfg,
		},
	},
};
#endif

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

	/* 6: AP2MDM_UART_AUDIO_TX (Follow QCT) */
	/* 7: AP2MDM_UART_AUDIO_RX (Follow QCT) */
	/* 8: AP2MDM_UART_AUDIO_CTS_N (Follow QCT) */
	/* 9: AP2MDM_UART_AUDIO_RFR_N (Follow QCT) */

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

	/* 16: I2C_DATA_TP */
	/* 17: I2C_CLK_TP */
	/* 18: AP2MDM_UART_MOD_TX (Follow QCT), Inter-Modem IPC 2-wire UART */
	/* 19: AP2MDM_UART_MOD_RX (Follow QCT), Inter-Modem IPC 2-wire UART */

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

	/* 22: AP2MDM_PMIC_PWR_EN (Follow QCT), IPC handshaking signals */
	/* 23: MDM2AP_VFR (Follow QCT), VFR signal needed for UART audio */
	/* 24: MDM2AP_STATUS (Follow QCT), IPC handshaking signals */
	/* 25: USB_SW, HS USB switch */
	/* 26: FM_SSBI (Follow QCT) */
	/* 27: FM_DATA (Follow QCT) */
	/* 28: BT_CTL (Follow QCT) */
	/* 29: BT_DATA (Follow QCT) */
	/* 30: UIM1_DATA_MSM (Follow QCT) */
	/* 31: UIM1_CLK_MSM (Follow QCT) */

	{ /* TP_RESET_N */
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_high,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_high,
		},
	},
	{ /* CHAT_CAM_RST_N */
		.gpio = 33,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},

	/* 34: UART_TX_DFMS (Follow QCT) */
	/* 35: UART_RX_DTMS (Follow QCT) */
	/* 36: DEBUG_UART_CTS_N (Follow QCT) */
	/* 37: DEBUG_UART_RFR_N (Follow QCT) */
	/* 38: UART_TX_QSCUART4 (Follow QCT), for QSC UART Boot */
	/* 39: UART_RX_QSCUART4 (Follow QCT), for QSC UART Boot */
	/* 40: MDM2AP_ERRFATAL (Follow QCT) */

	{ /* QSC_EDL_EN */
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* LCD_DCDC_EN */
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* OTG_OVRCUR_DET_N */
		.gpio = 43,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_in,
		},
	},

	/* 44: I2C_DATA_SENS */
	/* 45: I2C_CLK_SENS */

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
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* USB_OTG_EN */
		.gpio = 51,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NFC_DWLD_EN */
		.gpio = 52,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* Reserved TP_SWIO */
		.gpio = 53,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
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
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* NC */
		.gpio = 56,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},

	/* 57: WTRO_APT_EN (Follow QCT) */

	{ /* BMU_INT_N */
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_pull_up_in,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_up_in,
		},
	},

	/* 59: SLIMBUS1_MCLK (Follow QCT) */
	/* 60: SLIMBUS1_CLK (Follow QCT) */
	/* 61: SLIMBUS1_DATA (Follow QCT) */
	/* 62: CODEC_MAD_INT_N (Follow QCT) */

	{ /* NC */
		.gpio = 63,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
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
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},

	/* 68: AP2MDM_VDD_MIN (Follow QCT) */

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
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NC */
		.gpio = 72,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},

	/* 73: I2C_DATA_PERI */
	/* 74: I2C_CLK_PERI */

	{ /* NC */
		.gpio = 75,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NC */
		.gpio = 76,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},

	/* 77: AP2MDM_STATUS (Follow QCT), is IPC handshaking signals. */
	/* 78: AP2MDM_SOFT_RESET (Follow QCT), is IPC handshaking signals. */
	/* 79: AP2MDM_KPDPWR_N, is IPC handshaking signals. */
	/* 80: AP2MDM_ERRFATAL, is IPC handshaking signals. */

	{ /* MDM2AP_WAKE_UP */
		.gpio = 81,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* AP2MDM_WAKE_UP */
		.gpio = 82,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},

	/* 83: BT_SSBI (Follow QCT) */
	/* 84: WL_CMD_DATA2 (Follow QCT) */
	/* 85: WL_CMD_DATA1 (Follow QCT) */
	/* 86: WL_CMD_DATA0 (Follow QCT) */
	/* 87: WL_CMD_SET (Follow QCT) */
	/* 88: WL_CMD_CLK (Follow QCT) */

	{ /* NC */
		.gpio = 89,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NC */
		.gpio = 90,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* GSM_TX_MASK */
		.gpio = 91,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_2ma_no_pull_low,
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NC, reserved for ANT_TUNE0*/
		.gpio = 92,
		.settings = { [GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low, },
	},

	/* 93: AP2MDM_UART_PRIM_TX (Follow QCT) */
	/* 94: AP2MDM_UART_PRIM_RX (Follow QCT) */
	/* 95: AP2MDM_UART_PRIM_CTS_N (Follow QCT) */
	/* 96: AP2MDM_UART_PRIM_RFR_N (Follow QCT) */
	/* 97: MDM2AP_VDD_MIN (Follow QCT), is IPC handshaking signals. */

	{ /* NC, Reserved for RF_ID_EXTENSION */
		.gpio = 98,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NC */
		.gpio = 99,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},

	/* 100: HDMI_DDCSCL */
	/* 101: HDMI_DDCSCA */
	/* 102: HDMI_HPD (Follow QCT) */
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

	/* 108: PS_HOLD (Follow QCT) */

	{ /* NC */
		.gpio = 109,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},

	/* 110: ANT_SW_SEL5 (Follow QCT) */
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
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* NC (PA1_R1) */
		.gpio = 121,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},

	/* 122: PA0_R0 (Follow QCT) */
	/* 123: PA0_R1 (Follow QCT) */

	{ /* NC */
		.gpio = 124,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},

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
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},

	/* 136: PA_ON0 (Follow QCT) */
	/* 137: GPS_BLANKING (Follow QCT) */

	{ /* NC */
		.gpio = 138,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},

	/* 139: DRX_SW_SEL0 (Follow QCT) */

	{ /* NC */
		.gpio = 140,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},
	{ /* NC */
		.gpio = 141,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_pull_down_in,
		},
	},

	/* 142: WTR0_SSBI_PRX_DRX (Follow QCT) */
	/* 143: WTR0_SSBI_TX_GNSS (Follow QCT) */
	/* 144: NC (GSM_TX_CLK) Follow QCT */
	/* 145: NC (GSM_TX_LB_CLK) Follow QCT */
	/* 146: WTR0 GPDATA2 (Follow QCT) */
	/* 147: WTR0 GPDATA1 (Follow QCT) */
	/* 148: WTR0 GPDATA0 (Follow QCT) */
	/* 149: GPS_LNA_EN (Follow QCT) */

	{ /* NC */
		.gpio = 150,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
	{ /* NC */
		.gpio = 151,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_2ma_no_pull_low,
		},
	},
};

void __init gpiomux_device_install(void)
{
	msm_gpiomux_install(semc_viskan_all_cfgs,
			ARRAY_SIZE(semc_viskan_all_cfgs));
}

int __init msm8960_init_gpiomux(void)
{
	int rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err(KERN_ERR "msm_gpiomux_init failed %d\n", rc);
		return rc;
	}

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	msm_gpiomux_install(msm8960_ethernet_configs,
			ARRAY_SIZE(msm8960_ethernet_configs));
#endif

	msm_gpiomux_install(msm8960_gsbi_configs,
			ARRAY_SIZE(msm8960_gsbi_configs));

	msm_gpiomux_install(msm8960_slimbus_config,
			ARRAY_SIZE(msm8960_slimbus_config));

	msm_gpiomux_install(msm8960_audio_codec_configs,
			ARRAY_SIZE(msm8960_audio_codec_configs));

	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	msm_gpiomux_install(msm8960_sdcc4_configs,
		ARRAY_SIZE(msm8960_sdcc4_configs));
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	msm_gpiomux_install(msm8960_hdmi_configs,
			ARRAY_SIZE(msm8960_hdmi_configs));
#endif

	if (socinfo_get_platform_subtype() != PLATFORM_SUBTYPE_SGLTE)
		msm_gpiomux_install(msm8960_gsbi8_uartdm_configs,
			ARRAY_SIZE(msm8960_gsbi8_uartdm_configs));

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE)
		msm_gpiomux_install(msm8960_gsbi8_uart_configs,
			ARRAY_SIZE(msm8960_gsbi8_uart_configs));
	else
		msm_gpiomux_install(msm8960_gsbi5_uart_configs,
			ARRAY_SIZE(msm8960_gsbi5_uart_configs));

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE) {
		/* For 8960 Fusion 2.2 Primary IPC */
		msm_gpiomux_install(msm8960_fusion_gsbi_configs,
			ARRAY_SIZE(msm8960_fusion_gsbi_configs));
		/* For SGLTE 8960 Fusion External VFR */
		msm_gpiomux_install(msm8960_external_vfr_configs,
			ARRAY_SIZE(msm8960_external_vfr_configs));
	}

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	msm_gpiomux_install(msm8960_sdcc2_configs,
		ARRAY_SIZE(msm8960_sdcc2_configs));
#endif

	gpiomux_device_install();

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE)
		msm_gpiomux_install(sglte_configs,
			ARRAY_SIZE(sglte_configs));

	return 0;
}
