/* arch/arm/mach-msm/sim_detect_somc.c
 *
 * Copyright (c) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/input.h>

/*
* -1 means original status
* 0 means sim card removed
* 1 means sim card inserted
*/
static atomic_t	sim_status = ATOMIC_INIT(-1);

static atomic_t	module_ready;
static struct input_dev *input_dev;

static ssize_t attr_simstatus_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&sim_status));
}

/* should can be used in irq handler */
int simstatus_store(long status)
{
	static int init = 1;
	int ret = -EINVAL;

	if (!atomic_read(&module_ready)) {
		ret = -EAGAIN;
		goto err_exit;
	}

	if ((1 < status) || (status < 0))
		goto err_exit;

	atomic_set(&sim_status, status);
	pr_info("%s called, sim status %ld\n", __func__, status);

	if (init) {
		init = 0;
		if (!status)
			__change_bit(SW_JACK_PHYSICAL_INSERT, input_dev->sw);
	}
	input_event(input_dev, EV_SW,
		    SW_JACK_PHYSICAL_INSERT, status);
	input_sync(input_dev);

	return 0;

err_exit:
	pr_err("%s called, error: %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(simstatus_store);

static struct class *sim_class;
static struct class_attribute sim_attributes =
	__ATTR(sim_status, 0444, attr_simstatus_show, NULL);

static int __init sim_detect_init(void)
{
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: input_allocate_device failed\n",
			__func__);
		err = -ENOMEM;
		goto out;
	}

	input_dev->name = "simdetect";
	input_set_capability(input_dev, EV_SW,
					     SW_JACK_PHYSICAL_INSERT);

	err = input_register_device(input_dev);
	if (err) {
		pr_err("%s: input_register_device failed!\n",
			__func__);
		input_free_device(input_dev);
		goto out;
	}

	sim_class = class_create(THIS_MODULE, "sim_card");
	if (IS_ERR(sim_class)) {
		err = PTR_ERR(sim_class);
		pr_err("%s: cannot create sim_card class\n", __func__);
		goto out_input_device;
	}

	err = class_create_file(sim_class, &sim_attributes);
	if (err) {
		pr_err("%s: cannot create sysfs file\n", __func__);
		goto out_class;
	}

	atomic_set(&module_ready, 1);

	return 0;

out_class:
	class_destroy(sim_class);
out_input_device:
	input_unregister_device(input_dev);
out:
	pr_err("%s: cannot initialize, error %d\n", __func__, err);
	return err;
}
module_init(sim_detect_init);

static void __exit sim_detect_exit(void)
{
	atomic_set(&module_ready, 0);
	class_remove_file(sim_class, &sim_attributes);
	class_destroy(sim_class);
	input_unregister_device(input_dev);
}
module_exit(sim_detect_exit);

MODULE_AUTHOR("NEIL GAO <neil.gao@sonyericsson.com>");
MODULE_DESCRIPTION("Detects SIM");
MODULE_ALIAS("SIM DETECT virtual driver");
MODULE_LICENSE("GPL v2");
