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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/types.h>

#define SIM_DETECT_DEV_NAME "sim_detect"
enum sim_report_state {
	NOTHING_HAPPENED = 0,
	SIM_REMOVED = 1,
	SIM_INSERTED = 2,
};

static atomic_t	module_ready;
static struct switch_dev sim_detect;

int simstatus_store(long status)
{
	int ret = -EINVAL;

	if (!atomic_read(&module_ready)) {
		ret = -EAGAIN;
		goto err_exit;
	}

	if ((1 < status) || (status < 0))
		goto err_exit;

	pr_info("%s called, sim status %ld\n", __func__, status);

	if (status)
		switch_set_state(&sim_detect, SIM_INSERTED);
	else
		switch_set_state(&sim_detect, SIM_REMOVED);

	return 0;

err_exit:
	pr_err("%s called, error: %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(simstatus_store);

static int __init sim_detect_init(void)
{
	int err;

	sim_detect.name = SIM_DETECT_DEV_NAME;
	sim_detect.state = 0;
	err = switch_dev_register(&sim_detect);
	if (err) {
		pr_err("%s cannot regist sim detect device(%d)\n",
			__func__, err);
		goto out;
	}

	atomic_set(&module_ready, 1);

	return 0;

out:
	pr_err("%s: cannot initialize, error %d\n", __func__, err);
	return err;
}
module_init(sim_detect_init);

static void __exit sim_detect_exit(void)
{
	atomic_set(&module_ready, 0);
	switch_dev_unregister(&sim_detect);
}
module_exit(sim_detect_exit);

MODULE_AUTHOR("NEIL GAO <neil.gao@sonyericsson.com>");
MODULE_DESCRIPTION("Detects SIM");
MODULE_ALIAS("SIM DETECT virtual driver");
MODULE_LICENSE("GPL v2");
