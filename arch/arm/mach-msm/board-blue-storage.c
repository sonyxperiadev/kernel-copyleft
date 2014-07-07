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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include "devices.h"
#include "board-8960.h"
#include "board-storage-common-a.h"

#ifdef CONFIG_MMC_MSM_POWER_SUPPLY_SDC3_VIA_MPP04
static uint32_t msm_sdc3_set_mpp04(struct device*, unsigned int);
#endif
static int msm_sdc3_get_wpswitch(struct device *dev);

/* MSM8960 has 5 SDCC controllers */
enum sdcc_controllers {
	SDCC1,
	SDCC2,
	SDCC3,
	SDCC4,
	SDCC5,
	MAX_SDCC_CONTROLLER
};

/* All SDCC controllers require VDD/VCC voltage */
static struct msm_mmc_reg_data mmc_vdd_reg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.name = "sdc_vdd",
		.high_vol_level = 2950000,
		.low_vol_level = 2950000,
		.always_on = 1,
		.lpm_sup = 1,
		.lpm_uA = 9000,
		.hpm_uA = 200000, /* 200mA */
	},
	/* SDCC2 : SDIO slot connected */
	[SDCC2] = {
		.name = "sdc_vdd",
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.always_on = 1,
		.lpm_sup = 1,
		.lpm_uA = 9000,
		.hpm_uA = 200000, /* 200mA */
	},
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
		.name = "sdc_vdd",
		.high_vol_level = 2950000,
		.low_vol_level = 2950000,
		.hpm_uA = 600000, /* 600mA */
	}
};

/* SDCC controllers may require voting for IO operating voltage */
static struct msm_mmc_reg_data mmc_vdd_io_reg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.name = "sdc_vdd_io",
		.always_on = 1,
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.hpm_uA = 200000, /* 200mA */
	},
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
		.name = "sdc_vdd_io",
		.high_vol_level = 2950000,
		.low_vol_level = 1850000,
		.lpm_sup = 1,
		/* Max. Active current required is 16 mA */
		.hpm_uA = 16000,
		/*
		 * Sleep current required is ~300 uA. But min. vote can be
		 * in terms of mA (min. 1 mA). So let's vote for 2 mA
		 * during sleep.
		 */
		.lpm_uA = 2000,
	},
	/* SDCC4 : SDIO slot connected */
	[SDCC4] = {
		.name = "sdc_vdd_io",
		.high_vol_level = 1800000,
		.low_vol_level = 1800000,
		.always_on = 1,
		.lpm_sup = 1,
		.hpm_uA = 200000, /* 200mA */
		.lpm_uA = 2000,
	},
};

static struct msm_mmc_slot_reg_data mmc_slot_vreg_data[MAX_SDCC_CONTROLLER] = {
	/* SDCC1 : eMMC card connected */
	[SDCC1] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC1],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC1],
	},
	/* SDCC2 : SDIO card slot connected */
	[SDCC2] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC2],
	},
	/* SDCC3 : External card slot connected */
	[SDCC3] = {
		.vdd_data = &mmc_vdd_reg_data[SDCC3],
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC3],
	},
	/* SDCC4 : SDIO card slot connected */
	[SDCC4] = {
		.vdd_io_data = &mmc_vdd_io_reg_data[SDCC4],
	},
};

/* SDC1 pad data */
static struct msm_mmc_pad_drv sdc1_pad_drv_on_cfg[] = {
	{TLMM_HDRV_SDC1_CLK, GPIO_CFG_16MA},
	{TLMM_HDRV_SDC1_CMD, GPIO_CFG_6MA},
	{TLMM_HDRV_SDC1_DATA, GPIO_CFG_6MA}
};

static struct msm_mmc_pad_drv sdc1_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC1_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC1_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC1_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc1_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC1_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC1_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC1_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_mmc_pad_pull sdc1_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC1_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC1_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC1_DATA, GPIO_CFG_PULL_UP}
};

/* SDC3 pad data */
static struct msm_mmc_pad_drv sdc3_pad_drv_on_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_8MA}
};

static struct msm_mmc_pad_drv sdc3_pad_drv_off_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_2MA}
};

static struct msm_mmc_pad_pull sdc3_pad_pull_on_cfg[] = {
	{TLMM_PULL_SDC3_CLK, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_mmc_pad_pull sdc3_pad_pull_off_cfg[] = {
	{TLMM_PULL_SDC3_CLK, GPIO_CFG_NO_PULL},
	/*
	 * SDC3 CMD/DATA lines should be NO PULL, because there
	 * is no card and leakage current occurs by the HW
	 * problem of MSM in card power off.
	 */
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_NO_PULL},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_NO_PULL}
};

static struct msm_mmc_pad_pull_data mmc_pad_pull_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.on = sdc1_pad_pull_on_cfg,
		.off = sdc1_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc1_pad_pull_on_cfg)
	},
	[SDCC3] = {
		.on = sdc3_pad_pull_on_cfg,
		.off = sdc3_pad_pull_off_cfg,
		.size = ARRAY_SIZE(sdc3_pad_pull_on_cfg)
	},
};

static struct msm_mmc_pad_drv_data mmc_pad_drv_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.on = sdc1_pad_drv_on_cfg,
		.off = sdc1_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc1_pad_drv_on_cfg)
	},
	[SDCC3] = {
		.on = sdc3_pad_drv_on_cfg,
		.off = sdc3_pad_drv_off_cfg,
		.size = ARRAY_SIZE(sdc3_pad_drv_on_cfg)
	},
};

struct msm_mmc_gpio sdc2_gpio[] = {
	{92, "sdc2_dat_3"},
	{91, "sdc2_dat_2"},
	{90, "sdc2_dat_1"},
	{89, "sdc2_dat_0"},
	{97, "sdc2_cmd"},
	{98, "sdc2_clk"}
};

struct msm_mmc_gpio sdc4_gpio[] = {
	{83, "sdc4_dat_3"},
	{84, "sdc4_dat_2"},
	{85, "sdc4_dat_1"},
	{86, "sdc4_dat_0"},
	{87, "sdc4_cmd"},
	{88, "sdc4_clk"}
};

struct msm_mmc_gpio_data mmc_gpio_data[MAX_SDCC_CONTROLLER] = {
	[SDCC2] = {
		.gpio = sdc2_gpio,
		.size = ARRAY_SIZE(sdc2_gpio),
	},
	[SDCC4] = {
		.gpio = sdc4_gpio,
		.size = ARRAY_SIZE(sdc4_gpio),
	},
};

static struct msm_mmc_pad_data mmc_pad_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.pull = &mmc_pad_pull_data[SDCC1],
		.drv = &mmc_pad_drv_data[SDCC1]
	},
	[SDCC3] = {
		.pull = &mmc_pad_pull_data[SDCC3],
		.drv = &mmc_pad_drv_data[SDCC3]
	},
};

static struct msm_mmc_pin_data mmc_slot_pin_data[MAX_SDCC_CONTROLLER] = {
	[SDCC1] = {
		.pad_data = &mmc_pad_data[SDCC1],
	},
	[SDCC2] = {
		.is_gpio = 1,
		.gpio_data = &mmc_gpio_data[SDCC2],
	},
	[SDCC3] = {
		.pad_data = &mmc_pad_data[SDCC3],
	},
	[SDCC4] = {
		.is_gpio = 1,
		.gpio_data = &mmc_gpio_data[SDCC4],
	},
};

#define MSM_MPM_PIN_SDC1_DAT1	17
#define MSM_MPM_PIN_SDC3_DAT1	21

static unsigned int sdc1_sup_clk_rates[] = {
	400000, 24000000, 48000000, 96000000
};

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static unsigned int sdc3_sup_clk_rates[] = {
	400000, 24000000, 48000000, 96000000, 192000000
};
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm8960_sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.sup_clk_table	= sdc1_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc1_sup_clk_rates),
	.nonremovable	= 1,
	.vreg_data	= &mmc_slot_vreg_data[SDCC1],
	.pin_data	= &mmc_slot_pin_data[SDCC1],
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC1_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
	.uhs_caps	= MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int sdc2_sup_clk_rates[] = {
	400000, 24000000, 48000000
};

static struct mmc_platform_data msm8960_sdc2_data = {
	.ocr_mask       = MMC_VDD_165_195,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table  = sdc2_sup_clk_rates,
	.sup_clk_cnt    = ARRAY_SIZE(sdc2_sup_clk_rates),
	.vreg_data      = &mmc_slot_vreg_data[SDCC2],
	.pin_data       = &mmc_slot_pin_data[SDCC2],
	.sdiowakeup_irq = MSM_GPIO_TO_INT(90),
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm8960_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table	= sdc3_sup_clk_rates,
	.sup_clk_cnt	= ARRAY_SIZE(sdc3_sup_clk_rates),
	.vreg_data	= &mmc_slot_vreg_data[SDCC3],
	.pin_data	= &mmc_slot_pin_data[SDCC3],
	.wpswitch	= msm_sdc3_get_wpswitch,
#ifdef CONFIG_MMC_MSM_POWER_SUPPLY_SDC3_VIA_MPP04
	.translate_vdd	= msm_sdc3_set_mpp04,
#endif
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status_gpio	= PM8921_GPIO_PM_TO_SYS(26),
	.status_irq	= PM8921_GPIO_IRQ(PM8921_IRQ_BASE, 26),
	.irq_flags	= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.is_status_gpio_active_low = true,
#endif
	.xpc_cap	= 1,
	.uhs_caps	= 0,
	.mpm_sdiowakeup_int = MSM_MPM_PIN_SDC3_DAT1,
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static unsigned int sdc4_sup_clk_rates[] = {
	400000, 24000000, 48000000
};

static struct mmc_platform_data msm8960_sdc4_data = {
	.ocr_mask       = MMC_VDD_165_195,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sup_clk_table  = sdc4_sup_clk_rates,
	.sup_clk_cnt    = ARRAY_SIZE(sdc4_sup_clk_rates),
	.vreg_data      = &mmc_slot_vreg_data[SDCC4],
	.pin_data       = &mmc_slot_pin_data[SDCC4],
	.sdiowakeup_irq = MSM_GPIO_TO_INT(85),
	.msm_bus_voting_data = &sps_to_ddr_bus_voting_data,
};
#endif

#ifdef CONFIG_MMC_MSM_POWER_SUPPLY_SDC3_VIA_MPP04
#define PM8921_MPP04 0x4
#define SD_PWR_EN PM8921_MPP_PM_TO_SYS(PM8921_MPP04)

static uint32_t msm_sdc3_set_mpp04(struct device *dv, unsigned int vdd)
{
	static struct regulator *reg_sdc_vdd;
	int rc = 0;
	bool enable = (vdd != 0); /* voltage high */
	struct msm_mmc_reg_data *vreg_data = &mmc_vdd_io_reg_data[SDCC3];

	if (!reg_sdc_vdd)
		reg_sdc_vdd = regulator_get(NULL, "8921_l6");
	if (IS_ERR(reg_sdc_vdd)) {
		pr_err("could not get reg_sdc_vdd, rc = %ld\n",
			PTR_ERR(reg_sdc_vdd));
		rc = ENODEV;
		goto done;
	}
	if (enable) {
		if (vreg_data->is_enabled)
			goto done;
		pr_info("%s: Enable uSD card Load SW\n", __func__);
		rc = regulator_set_optimum_mode(reg_sdc_vdd, 200000);
		if (rc < 0) {
			pr_err("set_optimum_mode sdc_vdd failed, rc=%d\n",
				 rc);
			goto done;
		}
		/* allocate GPIO for SD_PWR_EN */
		rc = gpio_request(SD_PWR_EN, "PM8921_MPP04");
		if (rc) {
			pr_err("gpio_request(%d, %s) failed %d\n",
				SD_PWR_EN,
				"PM8921_MPP04", rc);
			goto done;
		}
		/* GPIO OUTPUT, might sleep, enable GPIO */
		gpio_set_value_cansleep(SD_PWR_EN, 1);
		vreg_data->is_enabled = true;
	} else {
		if (vreg_data->always_on || !vreg_data->is_enabled)
			goto done;
		pr_info("%s: Disable uSD card Load SW\n", __func__);
		/* GPIO OUTPUT, might sleep, disable GPIO */
		gpio_set_value_cansleep(SD_PWR_EN, 0);
		/* release previously-claimed GPIO */
		gpio_free(SD_PWR_EN);
		rc = regulator_set_optimum_mode(reg_sdc_vdd, 1);
		if (rc < 0) {
			pr_err("set_optimum_mode sdc_vdd failed, rc=%d\n",
				 rc);
		} else {
			/* regulator_set_optimum_mode() can return non zero
			 * value even for success case.
			 */
			rc = 0;
		}
		usleep_range(2000, 2000);
		vreg_data->is_enabled = false;
	}
done:
	return rc;
}
#endif

static int msm_sdc3_get_wpswitch(struct device *dev)
{
	/*
	*  Since we don't have an switch on blue for detecting
	*  if sdcard is read only we should always return 0
	*  which means it's never read only.
	*/
	int status = 0;
	return status;
}

void __init msm8960_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	/*
	 * When eMMC runs in DDR mode on CDP platform, we have
	 * seen instability due to DATA CRC errors. These errors are
	 * attributed to long physical path between MSM and eMMC on CDP.
	 * So let's not enable the DDR mode on CDP platform but let other
	 * platforms take advantage of eMMC DDR mode.
	 */
	if (!machine_is_msm8960_cdp())
		msm8960_sdc1_data.uhs_caps |= (MMC_CAP_1_8V_DDR |
					       MMC_CAP_UHS_DDR50);
	/* SDC1 : eMMC card connected */
	msm_add_sdcc(1, &msm8960_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/* SDC2: SDIO slot for WLAN*/
	msm_add_sdcc(2, &msm8960_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	/* SDC3: External card slot */
	msm_add_sdcc(3, &msm8960_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	/* SDC4: SDIO slot for WLAN */
	msm_add_sdcc(4, &msm8960_sdc4_data);
#endif
}
