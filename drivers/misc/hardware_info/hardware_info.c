/*
 * Copyright (C) 2011 Huaqin Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*******************************************************************************
* Dependency
*******************************************************************************/
/*
* Copyright 2025 Sony Corporation
* NOTE: This file has been modified by Sony Corporation
* Modifications are licensed under the License.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/hardware_info.h>
#include <linux/regulator/consumer.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/iio/consumer.h>

#define hwinfo_debug  printk

static struct device *lcm_dev = NULL;

static int current_type_value;


/******************************************************************************
 * Hardware Info Driver
*************************`*****************************************************/
static HARDWARE_INFO hwinfo_data;

static int hw_info_parse_dt(struct device_node *np)
{
	int ret = -1;
	int type_gpio_pin0 = -1;
	int type_gpio_pin1 = -1;
	int type_gpio_pin2 = -1;
	if (np) {
		type_gpio_pin0 = of_get_named_gpio(np, "pcb_type_gpios0", 0);
		ret = gpio_request(type_gpio_pin0, "pcb_type_gpios0");
		if (ret) {
			hwinfo_debug
			    ("[HWINFO] pcb_type_gpio not available (ret=%d)\n",
			     ret);
			current_type_value = -1;
			goto err;
		}

		type_gpio_pin1 = of_get_named_gpio(np, "pcb_type_gpios1", 0);
		ret = gpio_request(type_gpio_pin1, "pcb_type_gpios1");
		if (ret) {
			hwinfo_debug
			    ("[HWINFO] pcb_type_gpio not available (ret=%d)\n",
			     ret);
			current_type_value = -1;
			goto err;
		}

		type_gpio_pin2 = of_get_named_gpio(np, "pcb_type_gpios2", 0);
		ret = gpio_request(type_gpio_pin2, "pcb_type_gpios2");
		if (ret) {
			hwinfo_debug
			    ("[HWINFO] pcb_type_gpio not available (ret=%d)\n",
			     ret);
			current_type_value = -1;
			goto err;
		}

		current_type_value =
		    (gpio_get_value(type_gpio_pin0) << 0) |
		    (gpio_get_value(type_gpio_pin1) << 1) |
		    (gpio_get_value(type_gpio_pin2) << 2);
		printk("[HWINFO] current_type_value=%d\n", current_type_value);
	}

err:
	return ret;

	return 0;
}

static int HardwareInfo_driver_probe(struct platform_device *pdev)
{
	int ret = -1;
	printk("HardwareInfo_driver_probe Start!");

	ret = hw_info_parse_dt(pdev->dev.of_node);
	if (ret < 0) {
		hwinfo_debug("[HWINFO] hw_info_parse_dt failed! (ret=%d)\n",
			     ret);
		goto err;
	}

	lcm_dev = &pdev->dev;
	hwinfo_debug("HardwareInfo_driver_probe OK!");
err:
	return ret;
}

static void HardwareInfo_driver_remove(struct platform_device *pdev)
{
	return;
}

static const struct of_device_id hwinfo_dt_match[] = {
	{.compatible = "huaqin,HardwareInfo",},
	{},
};

static struct platform_driver HardwareInfo_driver = {
	.probe = HardwareInfo_driver_probe,
	.remove = HardwareInfo_driver_remove,
	.driver = {
		   .name = "HardwareInfo",
		   .of_match_table = hwinfo_dt_match,
		   },
};

static int __init HardwareInfo_mod_init(void)
{
	int ret = -1;
	memset(&hwinfo_data, 0, sizeof(hwinfo_data));
	ret = platform_driver_register(&HardwareInfo_driver);
	if (ret) {
		hwinfo_debug
		    ("[HWINFO] HardwareInfo_driver registed failed! (ret=%d)\n",
		     ret);
	}

	return ret;
}

static void __exit HardwareInfo_mod_exit(void)
{
	platform_driver_unregister(&HardwareInfo_driver);
}

module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
MODULE_AUTHOR("Oly Peng ");
MODULE_DESCRIPTION("Huaqin Hareware Info driver");
MODULE_LICENSE("GPL");
