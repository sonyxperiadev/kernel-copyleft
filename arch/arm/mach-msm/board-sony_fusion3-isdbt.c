/* arch/arm/mach-msm/board-sony_fusion3-isdbt.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Tatsuya Ooka <Tatsuya.Ooka@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/isdbt_tunerpm.h>

#include "board-8064.h"

#define PM_GPIO_ISDBT_POWER     36
#define PM_GPIO_ISDBT_RESET     37
#define PM8821_MPP1            0x1
#define PM8821_MPP2            0x2
#define PM8821_MPP3            0x3
#define PM8821_MPP_ON            1
#define PM8821_MPP_OFF           0

#define D_TUNER_GPIO_POWER_NAME  "ISDB-T tuner power"
#define D_TUNER_GPIO_RESET_NAME  "ISDB-T tuner HW reset"
#define D_TUNER_PMIC_MPP1_NAME   "PM8821_MPP1"
#define D_TUNER_PMIC_MPP2_NAME   "PM8821_MPP2"
#define D_TUNER_PMIC_MPP3_NAME   "PM8821_MPP3"

struct regulator *tuner_avdh;

int isdbt_tunerpm_init(struct device *dev)
{
	int ret;

	ret = gpio_request(PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_POWER),
		D_TUNER_GPIO_POWER_NAME);
	if (ret)
		return ret;

	ret = gpio_request(PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_RESET),
		D_TUNER_GPIO_RESET_NAME);
	if (ret) {
		gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_POWER));
		return ret;
	}

	tuner_avdh = regulator_get(NULL, "8921_l21");
	if (IS_ERR(tuner_avdh))
		tuner_avdh = NULL;
	ret = regulator_set_voltage(tuner_avdh, 1800000, 1800000);
	if (ret) {
		gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_POWER));
		gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_RESET));
		return ret;
	}

	ret = gpio_request(PM8821_MPP_PM_TO_SYS(PM8821_MPP1),
		D_TUNER_PMIC_MPP1_NAME);
	if (ret) {
		gpio_free(PM8821_MPP_PM_TO_SYS(PM8821_MPP1));
		return ret;
	}

	ret = gpio_request(PM8821_MPP_PM_TO_SYS(PM8821_MPP2),
		D_TUNER_PMIC_MPP2_NAME);
	if (ret) {
		gpio_free(PM8821_MPP_PM_TO_SYS(PM8821_MPP2));
		return ret;
	}

	ret = gpio_request(PM8821_MPP_PM_TO_SYS(PM8821_MPP3),
		D_TUNER_PMIC_MPP3_NAME);
	if (ret) {
		gpio_free(PM8821_MPP_PM_TO_SYS(PM8821_MPP3));
		return ret;
	}
	return 0;
}

int isdbt_tunerpm_free(struct device *dev)
{
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_POWER));
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_RESET));
	gpio_free(PM8821_MPP_PM_TO_SYS(PM8821_MPP1));
	gpio_free(PM8821_MPP_PM_TO_SYS(PM8821_MPP2));
	gpio_free(PM8821_MPP_PM_TO_SYS(PM8821_MPP3));
	return 0;
}

int isdbt_tunerpm_power_control(struct device *dev, int on)
{
	gpio_set_value_cansleep(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_POWER), on);

	if (on)
		regulator_enable(tuner_avdh);
	else
		regulator_disable(tuner_avdh);

	return 0;
}

int isdbt_tunerpm_reset_control(struct device *dev, int on)
{
	gpio_set_value_cansleep(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_ISDBT_RESET), on);
	return 0;
}

int isdbt_tunerpm_mpp1_control(int on)
{
	gpio_set_value_cansleep(
		PM8821_MPP_PM_TO_SYS(PM8821_MPP1), on);
	return 0;
}

int isdbt_tunerpm_mpp2_control(int on)
{
	gpio_set_value_cansleep(
		PM8821_MPP_PM_TO_SYS(PM8821_MPP2), on);
	return 0;
}

int isdbt_tunerpm_mpp3_control(int on)
{
	gpio_set_value_cansleep(
		PM8821_MPP_PM_TO_SYS(PM8821_MPP3), on);
	return 0;
}

int isdbt_tunerpm_ant_switch(struct device *dev, int ant_mode)
{
	switch (ant_mode) {
	case ANTMODE_WHIP:
		isdbt_tunerpm_mpp1_control(PM8821_MPP_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP_OFF);
		isdbt_tunerpm_mpp3_control(PM8821_MPP_OFF);
		break;
	case ANTMODE_EARPHONE:
		isdbt_tunerpm_mpp1_control(PM8821_MPP_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP_ON);
		isdbt_tunerpm_mpp3_control(PM8821_MPP_OFF);
		break;
	case ANTMODE_USB:
		isdbt_tunerpm_mpp1_control(PM8821_MPP_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP_OFF);
		isdbt_tunerpm_mpp3_control(PM8821_MPP_ON);
		break;
	case ANTMODE_CHARGER:
		isdbt_tunerpm_mpp1_control(PM8821_MPP_OFF);
		isdbt_tunerpm_mpp2_control(PM8821_MPP_ON);
		isdbt_tunerpm_mpp3_control(PM8821_MPP_ON);
		break;
	case ANTMODE_AUTO:
	case ANTMODE_NOTUSE:
		isdbt_tunerpm_mpp1_control(PM8821_MPP_ON);
		isdbt_tunerpm_mpp2_control(PM8821_MPP_OFF);
		isdbt_tunerpm_mpp3_control(PM8821_MPP_ON);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

struct isdbt_tunerpm_platform_data isdbt_tunerpm_data = {
	.i2c_adapter_id = 0,
	.init = isdbt_tunerpm_init,
	.free = isdbt_tunerpm_free,
	.reset_control = isdbt_tunerpm_reset_control,
	.power_control = isdbt_tunerpm_power_control,
	.ant_switch = isdbt_tunerpm_ant_switch,
};

struct platform_device isdbt_tunerpm_device = {
	.name = D_TUNER_CONFIG_DRIVER_NAME,
	.id = 0,
	.dev = {
		.platform_data = &isdbt_tunerpm_data,
	},
};
