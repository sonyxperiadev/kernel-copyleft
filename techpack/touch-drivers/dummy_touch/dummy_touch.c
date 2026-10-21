// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>

MODULE_DESCRIPTION("QTI dummy touchscreen driver");
MODULE_LICENSE("GPL v2");

#define DEVICE_COMPATIBLE "qti,dummy_ts"
#define DRIVER_NAME "qti_dummy_ts"

static int dummy_touch_probe(struct platform_device *device)
{
	struct input_dev *touch_dev = NULL;
	int rc;

	touch_dev = input_allocate_device();
	if (!touch_dev) {
		pr_err("%s: input device allocate failed\n", __func__);
		return -ENOMEM;
	}

	touch_dev->name = "qti_dummy_ts";
	touch_dev->id.bustype = BUS_VIRTUAL;
	touch_dev->evbit[0] = BIT_MASK(EV_KEY);
	touch_dev->keybit[BIT_WORD(BTN_0)] = BIT_MASK(BTN_0);
	touch_dev->dev.parent = NULL;

	rc = input_register_device(touch_dev);
	if (rc) {
		pr_err("%s: touch device register failed, rc = %d\n", __func__, rc);
		input_free_device(touch_dev);
		return rc;
	}

	pr_info("qti_dummy_ts device registered\n");

	platform_set_drvdata(device, (void *)touch_dev);

	return 0;
}

static int dummy_touch_remove(struct platform_device *device)
{
	struct input_dev *touch_dev = NULL;

	touch_dev = (struct input_dev *)platform_get_drvdata(device);
	if (touch_dev)
		input_free_device(touch_dev);

	return 0;
}

static const struct of_device_id  dummy_touch_id[] = {
	{.compatible = DEVICE_COMPATIBLE},
	{}
};

static struct platform_driver dummy_touch_driver = {

	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = dummy_touch_id,
	},
	.probe = dummy_touch_probe,
	.remove = dummy_touch_remove,
};

static int __init dummy_touch_init(void)
{
	platform_driver_register(&dummy_touch_driver);

	return 0;
}

static void __exit dummy_touch_exit(void)
{
	platform_driver_unregister(&dummy_touch_driver);
}

late_initcall(dummy_touch_init);
module_exit(dummy_touch_exit);
