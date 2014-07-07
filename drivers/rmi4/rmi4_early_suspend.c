/*
 * RMI4 early suspend handler.
 * drivers/rmi4/rmi4_e_suspend.c
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2012 Sony Mobile communications AB
 *
 * Author: Joachim Holst <joachim.holst@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifdef CONFIG_HAS_EARLYSUSPEND

#define DEBUG

#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rmi4/rmi4_early_suspend.h>
#include <linux/rmi4/rmi4.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define E_SUSPEND_LEVEL		(EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1)
#define E_SUSPEND_LEVEL_WAKELOCK	EARLY_SUSPEND_LEVEL_BLANK_SCREEN

struct rmi4_esusp_data {
	struct early_suspend early_suspend;
	struct rmi4_function_device *fdev;
	/* Using a mutex as a semaphore. I chose this because
	 * it is slightly leaner than a semaphore, and the code base
	 * is quite simple to overview */
};

static void rmi4_early_suspend(struct early_suspend *es)
{
	struct rmi4_esusp_data *data =
		container_of(es, struct rmi4_esusp_data,
			     early_suspend);

	dev_dbg(&data->fdev->dev, "Early suspend\n");
	rmi4_bus_notify(data->fdev, RMI4_CHIP_SUSPEND);
}

static void rmi4_early_resume(struct early_suspend *es)
{
	struct rmi4_esusp_data *data =
		container_of(es, struct rmi4_esusp_data,
			     early_suspend);

	dev_dbg(&data->fdev->dev, "Early resume\n");

	rmi4_bus_notify(data->fdev, RMI4_CHIP_WAKEUP);
}

static int rmi4_early_suspend_start(struct rmi4_function_device *fdev)
{
	struct rmi4_esusp_data *data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&fdev->dev, "%s - Failed to allocate driver data\n",
			__func__);
		return -ENOMEM;
	}

	data->fdev = fdev;
	dev_set_drvdata(&fdev->dev, data);

	data->early_suspend.level = E_SUSPEND_LEVEL;
	data->early_suspend.suspend = rmi4_early_suspend;
	data->early_suspend.resume = rmi4_early_resume;
	register_early_suspend(&data->early_suspend);

	dev_info(&fdev->dev, "Successfully probed %s\n", RMI4_E_SUSP_NAME);

	return 0;
}

static int rmi4_early_suspend_stop(struct rmi4_function_device *fdev)
{
	struct rmi4_esusp_data *data = dev_get_drvdata(&fdev->dev);

	unregister_early_suspend(&data->early_suspend);
	dev_set_drvdata(&fdev->dev, NULL);
	kfree(data);

	dev_info(&fdev->dev, "Successfully un-probed %s\n",
		 RMI4_E_SUSP_NAME);

	return 0;
}

static struct rmi4_function_driver rmi4_e_suspend = {
	.drv = {
		.name		= RMI4_E_SUSP_NAME,
	},
	.probe		= rmi4_early_suspend_start,
	.remove	= rmi4_early_suspend_stop,
};

static int __devinit rmi4_early_suspend_init(void)
{
	pr_info("Physically loading driver %s\n", RMI4_E_SUSP_NAME);
	return rmi4_bus_register_function_driver(&rmi4_e_suspend);
}

static void __devexit rmi4_early_suspend_exit(void)
{
	pr_info("Physically unloading driver %s\n", RMI4_E_SUSP_NAME);
	rmi4_bus_unregister_function_driver(&rmi4_e_suspend);
}

module_init(rmi4_early_suspend_init);
module_exit(rmi4_early_suspend_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonymobile.com>");
MODULE_DESCRIPTION("RMI4 Android Early Suspend function driver");
MODULE_LICENSE("GPL");

#endif /* CONFIG_HAS_EARLYSUSPEND */
