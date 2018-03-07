/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/cei_hw_id.h>

static int cei_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pinctrl_state *set_state;
	struct pinctrl *gpio_pinctrl;
	int retval = 0;

	pr_info("%s\n", __func__);
	gpio_pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR(gpio_pinctrl)) {
		if (PTR_ERR(gpio_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		pr_debug("Target does not use pinctrl\n");
		gpio_pinctrl = NULL;
	}

	set_state = pinctrl_lookup_state(gpio_pinctrl,
						"gpio_unused_active");
	if (IS_ERR(set_state)) {
		pr_debug("cannot get <gpio_unused_active>\n");
	} else {
		pr_info("Got pinctrl <gpio_unused_active>\n");
		retval = pinctrl_select_state(gpio_pinctrl, set_state);
		if (retval)
			pr_debug("cannot set <gpio_unused_active>\n");
	}

	set_state = pinctrl_lookup_state(gpio_pinctrl,
						"gpio_init_active");
	if (IS_ERR(set_state)) {
		pr_debug("cannot get <gpio_init_active>\n");
	} else {
		pr_info("Got pinctrl <gpio_init_active>\n");
		retval = pinctrl_select_state(gpio_pinctrl, set_state);
		if (retval)
			pr_debug("cannot set <gpio_init_active>\n");
	}

	ceibootmode_read();

	return retval;
}

static int cei_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id cei_gpio_of_match[] = {
	{ .compatible = "cei-gpio", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_unused_of_match);

static struct platform_driver cei_gpio_device_driver = {
	.probe		= cei_gpio_probe,
	.remove		= cei_gpio_remove,
	.driver		= {
		.name	= "cei-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(cei_gpio_of_match),
	}
};

static int __init cei_gpio_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&cei_gpio_device_driver);
}

static void __exit cei_gpio_exit(void)
{
	platform_driver_unregister(&cei_gpio_device_driver);
}

subsys_initcall(cei_gpio_init);
module_exit(cei_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CEI driver for UNUSED GPIOs");
MODULE_ALIAS("platform:gpio-unused");
