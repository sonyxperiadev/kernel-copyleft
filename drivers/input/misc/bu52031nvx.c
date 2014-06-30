/* drivers/input/misc/bu52031nvx.c
 *
 * Copyright (C) 2012 - 2013 Sony Mobile Communications AB.
 *
 * Author: Takashi Shiina <takashi.shiina@sonymobile.com>
 *         Tadashi Kubo <tadashi.kubo@sonymobile.com>
 *         Shogo Tanaka <shogo.tanaka@sonymobile.com>
 *         Neil Gao <neil.gao@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/types.h>

#define BU52031NVX_DEV_NAME "bu,52031nvx"

#define DETECTION_DELAY 50
#define DETECTION_CYCLES 4

#define DETECT_WORK_DELAY(p)	\
(msecs_to_jiffies(p->detection_delay))

struct bu52031nvx_drvdata {
	struct input_dev *input_dev;

	atomic_t detect_cycle;
	atomic_t detection_in_progress;

	int current_state;

	struct delayed_work det_work;

	const char *name;	/* input device name */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	int gpio;		/* -1 if this key does not support gpio */
	int active_low;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int wakeup;		/* configure the gpio as a wake-up source */
	int detection_delay;	/* debounce ticks interval in msecs */
	int detection_cycles;	/* debounce cycles in msecs */
	unsigned int irq;	/* Irq number in case of interrupt keys */
};

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static int bu52031nvx_get_devtree_pdata(struct device *dev,
			    struct bu52031nvx_drvdata *pdata)
{
	struct device_node *node;
	u32 reg;
	enum of_gpio_flags flags;

	node = dev->of_node;
	if (node == NULL)
		return -ENODEV;

	memset(pdata, 0, sizeof *pdata);

	if (!of_find_property(node, "gpios", NULL)) {
		dev_err(dev, "Found device without gpios\n");
		return -ENODEV;
	}

	pdata->name = of_get_property(node, "input-name", NULL);
	if (pdata->name == NULL)
		pdata->name = BU52031NVX_DEV_NAME;

	pdata->gpio = of_get_gpio_flags(node, 0, &flags);
	pdata->active_low = flags & OF_GPIO_ACTIVE_LOW;

	if (of_property_read_u32(node, "linux,code", &reg) == 0)
		pdata->code = reg;
	else
		pdata->code = SW_LID;

	if (of_property_read_u32(node, "linux,input-type", &reg) == 0)
		pdata->type = reg;
	else
		pdata->type = EV_SW;

	pdata->wakeup = !!of_get_property(node, "gpio-key,wakeup", NULL);

	if (of_property_read_u32(node, "detection_delay", &reg) == 0)
		pdata->detection_delay = reg;
	else
		pdata->detection_delay = DETECTION_DELAY;

	if (of_property_read_u32(node, "detection_cycles", &reg) == 0)
		pdata->detection_cycles = reg;
	else
		pdata->detection_cycles = DETECTION_CYCLES;

	return 0;
}

static struct of_device_id bu52031nvx_of_match[] = {
	{ .compatible = BU52031NVX_DEV_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, bu52031nvx_of_match);

#else

static int bu52031nvx_get_devtree_pdata(struct device *dev,
			    struct bu52031nvx_drvdata *pdata)
{
	return -ENODEV;
}

#define bu52031nvx_of_match NULL

#endif

static int bu52031nvx_gpio_setup(struct platform_device *pdev, int enable)
{
	int ret = 0;
	struct bu52031nvx_drvdata *ddata = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (enable) {
		ret = gpio_request(ddata->gpio, "BU52031NVX_IRQ");
		if (ret < 0)
			dev_err(dev, "failed to request gpio %d\n",
				ddata->gpio);
	} else {
		gpio_free(ddata->gpio);
	}

	return ret;
}

static irqreturn_t bu52031nvx_isr(int irq, void *data)
{
	struct bu52031nvx_drvdata *ddata = (struct bu52031nvx_drvdata *)data;

	atomic_set(&ddata->detect_cycle, 0);
	atomic_set(&ddata->detection_in_progress, 1);
	schedule_delayed_work(&ddata->det_work, DETECT_WORK_DELAY(ddata));

	return IRQ_HANDLED;
}

static void bu52031nvx_det_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bu52031nvx_drvdata *ddata =
		container_of(dwork, struct bu52031nvx_drvdata, det_work);
	struct input_dev *idev = ddata->input_dev;
	int gpio_state = (gpio_get_value(ddata->gpio) ? 1 : 0) ^
					ddata->active_low;

	if (ddata->current_state == gpio_state) {
		atomic_inc(&ddata->detect_cycle);
	} else {
		atomic_set(&ddata->detect_cycle, 0);
		ddata->current_state = gpio_state;
	}

	if (ddata->detection_cycles > atomic_read(&ddata->detect_cycle)) {
		schedule_delayed_work(&ddata->det_work,
					DETECT_WORK_DELAY(ddata));
	} else {
		input_event(idev, ddata->type, ddata->code, gpio_state);
		input_sync(idev);
		atomic_set(&ddata->detect_cycle, 0);
		atomic_set(&ddata->detection_in_progress, 0);
	}
}

static int bu52031nvx_open(struct input_dev *idev)
{
	struct bu52031nvx_drvdata *ddata = input_get_drvdata(idev);

	atomic_set(&ddata->detect_cycle, 0);
	atomic_set(&ddata->detection_in_progress, 1);
	schedule_delayed_work(&ddata->det_work, DETECT_WORK_DELAY(ddata));

	return 0;
}

static int bu52031nvx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bu52031nvx_drvdata *ddata;
	struct input_dev *input;
	int error = 0;

	ddata = kzalloc(sizeof(struct bu52031nvx_drvdata), GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to allocate drvdata (%s)\n", __func__);
		error = -ENOMEM;
		goto fail_kzalloc;
	}

	error = bu52031nvx_get_devtree_pdata(dev, ddata);
	if (error < 0) {
		dev_err(dev, "failed to get_devtree_pdata (%s)\n", __func__);
		goto fail_get_pdata;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(dev, "failed to allocate input_dev (%s)\n", __func__);
		error = -ENOMEM;
		goto fail_input_alloc;
	}

	ddata->input_dev = input;

	platform_set_drvdata(pdev, ddata);

	INIT_DELAYED_WORK(&ddata->det_work, bu52031nvx_det_work);

	input->name = ddata->name;
	input->open = bu52031nvx_open;

	input_set_capability(input, ddata->type, ddata->code);
	input_set_drvdata(input, ddata);

	error = bu52031nvx_gpio_setup(pdev, 1);
	if (error < 0) {
		dev_err(dev, "failed to setup gpio pin (%s)\n", __func__);
		goto fail_gpio_setup;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register device (%s)\n", __func__);
		goto fail_input_register;
	}

	error = request_irq(gpio_to_irq(ddata->gpio),
			bu52031nvx_isr,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			BU52031NVX_DEV_NAME, ddata);

	if (error) {
		dev_err(dev, "failed to request irq (%s)\n", __func__);
		goto fail_request_irq;
	}
	enable_irq_wake(gpio_to_irq(ddata->gpio));

	return 0;

fail_request_irq:
	input_unregister_device(input);
	bu52031nvx_gpio_setup(pdev, 0);
	goto fail_input_alloc;
fail_input_register:
	bu52031nvx_gpio_setup(pdev, 0);
fail_gpio_setup:
	input_free_device(input);
fail_input_alloc:
fail_get_pdata:
	kfree(ddata);
fail_kzalloc:

	return error;
}

static int bu52031nvx_remove(struct platform_device *pdev)
{
	struct bu52031nvx_drvdata *ddata = platform_get_drvdata(pdev);

	free_irq(gpio_to_irq(ddata->gpio), ddata);
	cancel_delayed_work_sync(&ddata->det_work);
	input_unregister_device(ddata->input_dev);

	bu52031nvx_gpio_setup(pdev, 0);

	kfree(ddata);

	return 0;
}

static int bu52031nvx_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bu52031nvx_drvdata *ddata = platform_get_drvdata(pdev);

	if (atomic_read(&ddata->detection_in_progress)) {
		dev_dbg(&pdev->dev, "Lid detection in progress. (%s)\n",
			__func__);
		return -EAGAIN;
	}

	return 0;
}

static struct platform_driver bu52031nvx_driver = {
	.probe = bu52031nvx_probe,
	.remove = bu52031nvx_remove,
	.suspend = bu52031nvx_suspend,
	.driver = {
		.name = BU52031NVX_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bu52031nvx_of_match,
	},
};

static int __init bu52031nvx_init(void)
{
	return platform_driver_register(&bu52031nvx_driver);
}

static void __exit bu52031nvx_exit(void)
{
	platform_driver_unregister(&bu52031nvx_driver);
}

module_init(bu52031nvx_init);
module_exit(bu52031nvx_exit);

MODULE_LICENSE("GPL");

