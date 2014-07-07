/*
 * RMI4 watchdog driver.
 * driver/rmi4/rmi4_watchdog_34.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonymobile.com>
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
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rmi4/rmi4.h>
#include <linux/rmi4/rmi4_watchdog.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define POLL_INTERVAL HZ

struct rmi4_watchdog_data {
	struct mutex lock;
	bool rearm;
	struct rmi4_function_device *fdev;
	struct delayed_work poll_work;
	int (*reset)(struct device *dev);
	int poll_t_jf;
};

static void rmi4_wd_set_rearm(struct rmi4_watchdog_data *d, bool enable)
{
	mutex_lock(&d->lock);
	d->rearm = enable;
	mutex_unlock(&d->lock);
}

#ifdef CONFIG_PM
static int rmi4_wd_suspend(struct rmi4_function_device *fdev)
{
	struct rmi4_watchdog_data *data = dev_get_drvdata(&fdev->dev);
	dev_dbg(&fdev->dev, "Suspending\n");
	rmi4_wd_set_rearm(data, false);
	cancel_delayed_work_sync(&data->poll_work);
	return 0;
}

static int rmi4_wd_resume(struct rmi4_function_device *fdev)
{
	struct rmi4_watchdog_data *data = dev_get_drvdata(&fdev->dev);
	dev_dbg(&fdev->dev, "Resuming\n");
	rmi4_wd_set_rearm(data, true);
	schedule_delayed_work(&data->poll_work, data->poll_t_jf);
	return 0;
}

static int rmi4_wd_pm_suspend(struct device *dev)
{
	return rmi4_wd_suspend(to_rmi4_func_core(dev));
}

static int rmi4_wd_pm_resume(struct device *dev)
{
	return rmi4_wd_resume(to_rmi4_func_core(dev));
}
#else
#define rmi4_wd_pm_suspend NULL
#define rmi4_wd_pm_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(rmi4_watchdog_pm_ops, rmi4_wd_pm_suspend,
		rmi4_wd_pm_resume);

static void rmi4_wd_irq_handler(int irq, void *data)
{
	struct rmi4_function_device *fdev = data;
	struct rmi4_watchdog_data *ddata = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);
	mutex_lock(&ddata->lock);
	if (cancel_delayed_work(&ddata->poll_work) && ddata->rearm)
		schedule_delayed_work(&ddata->poll_work, ddata->poll_t_jf);
	mutex_unlock(&ddata->lock);
}

static void rmi4_wd_notify_callback(enum rmi4_notification_event event,
				     void *data)
{
	struct rmi4_function_device *fdev = data;
	struct rmi4_watchdog_data *ddata = dev_get_drvdata(&fdev->dev);
	bool cancel = false;

	dev_dbg(&fdev->dev, "%s - Called, events %x\n", __func__, event);

	mutex_lock(&ddata->lock);
	if (event & (RMI4_DRIVER_RESET | RMI4_CHIP_WAKEUP)) {

		dev_dbg(&fdev->dev, "Device has been reset/woken-up\n");
		ddata->rearm = true;
		schedule_delayed_work(&ddata->poll_work, ddata->poll_t_jf);

	} else if (event & (RMI4_CHIP_SUSPEND | RMI4_CHIP_BOOTLOADER)) {

		dev_dbg(&fdev->dev, "Device in suspend/bootloader\n");
		ddata->rearm = false;
		cancel = true;
	} else
		dev_warn(&fdev->dev, "Notification not supported\n");
	mutex_unlock(&ddata->lock);
	if (cancel)
		cancel_delayed_work_sync(&ddata->poll_work);
}

static void rmi4_wd_status_poll(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rmi4_watchdog_data *ddata = container_of(dwork,
			struct rmi4_watchdog_data, poll_work);
	struct rmi4_function_device *fdev = ddata->fdev;
	int err;

	dev_dbg(&fdev->dev, "%s\n", __func__);

	err = rmi4_bus_read(fdev, DATA, 0, (void *)&err, 1);
	mutex_lock(&ddata->lock);
	if (err != 1) {
		dev_err(&fdev->dev, "%s, err %d\n", __func__, err);
		dev_info(&fdev->dev, "Resetting device\n");
		ddata->reset(&fdev->dev);
	}
	schedule_delayed_work(&ddata->poll_work, ddata->poll_t_jf);
	mutex_unlock(&ddata->lock);
}

static int rmi4_wd_probe(struct rmi4_function_device *fdev)
{
	int err;
	struct rmi4_watchdog_data *data;
	struct rmi4_wd_platform_data *pdata = dev_get_platdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	if (!pdata || !pdata->reset_func) {
		dev_err(&fdev->dev, "No reset function pointer provided\n");
		return -EINVAL;
	}
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&fdev->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	data->reset = pdata->reset_func;
	data->poll_t_jf = pdata->poll_t_ms ?
			msecs_to_jiffies(pdata->poll_t_ms) : POLL_INTERVAL;
	data->fdev = fdev;
	mutex_init(&data->lock);
	data->rearm = true;
	dev_set_drvdata(&fdev->dev, data);
	INIT_DELAYED_WORK(&data->poll_work, rmi4_wd_status_poll);
	err = rmi4_bus_request_notification(fdev, RMI4_DRIVER_RESET |
					    RMI4_CHIP_WAKEUP |
					    RMI4_CHIP_SUSPEND |
					    RMI4_CHIP_BOOTLOADER,
					    rmi4_wd_notify_callback, fdev);
	if (err) {
		dev_err(&fdev->dev, "Failed to subscribe to notifications\n");
		goto err_request_notification;
	}
	err = rmi4_bus_request_irq(fdev, fdev, rmi4_wd_irq_handler, 1);
	if (err) {
		dev_err(&fdev->dev, "Failed to subscribe to IRQ\n");
		goto err_request_irq;
	}
	schedule_delayed_work(&data->poll_work, data->poll_t_jf);
	dev_info(&fdev->dev, "Successfully started\n");
	return err;

err_request_irq:
	rmi4_bus_release_notification(fdev, fdev);
err_request_notification:
	dev_set_drvdata(&fdev->dev, NULL);
	kfree(data);
	return err;
}

static int rmi4_wd_stop(struct rmi4_function_device *fdev)
{
	struct rmi4_watchdog_data *data = dev_get_drvdata(&fdev->dev);

	dev_info(&fdev->dev, "%s - Called\n", __func__);
	rmi4_wd_set_rearm(data, false);
	cancel_delayed_work_sync(&data->poll_work);
	rmi4_bus_free_irq(fdev, fdev);
	rmi4_bus_release_notification(fdev, fdev);
	dev_set_drvdata(&fdev->dev, NULL);
	mutex_destroy(&data->lock);
	kfree(data);
	return 0;
}

static struct rmi4_function_driver rmi4_watchdog = {
	.drv = {
		.name = RMI4_WATCHDOG_NAME,
		.pm   = &rmi4_watchdog_pm_ops,
	},
	.probe  = rmi4_wd_probe,
	.remove = rmi4_wd_stop,
};

static int __devinit rmi4_wd_init(void)
{
	pr_info("Registering function %s on RMI4 bus\n",
		rmi4_watchdog.drv.name);
	return rmi4_bus_register_function_driver(&rmi4_watchdog);
}

static void __devexit rmi4_wd_exit(void)
{
	pr_info("Unregistering function %s from RMI4 bus\n",
		rmi4_watchdog.drv.name);
	rmi4_bus_unregister_function_driver(&rmi4_watchdog);
}

module_init(rmi4_wd_init);
module_exit(rmi4_wd_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonymobile.com>");
MODULE_DESCRIPTION("RMI4 watchdog driver");
MODULE_LICENSE("GPL");
