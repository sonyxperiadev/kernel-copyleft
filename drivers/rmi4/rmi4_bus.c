/*
 * RMI4 bus driver.
 * driver/rmi4/rmi4_bus.c
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2012 Sony Mobile Communications AB
 *
 * Author: Joachim Holst <joachim.holst@sonyericsson.com>
 * Author: Joachim Holst <joachim.holst@sonymobile.com>
 *
 * Based on rmi_bus by Synaptics and Unixphere.
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

#ifdef RMI4_CORE_DEBUG
#define DEBUG
#endif

#include <linux/module.h>
#include <linux/rmi4/rmi4.h>

#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/fs.h>

static struct device_type rmi4_core_device;
static struct device_type rmi4_function_device;

/* Used for char devices */
#define RMI4_BUS_CLASS_NAME	"rmi4"
static struct class *rmi4_char_dev_class;
static int rmi4_char_dev_major;
static int rmi4_char_dev_minor;

 /* Used for sensor counting. Will never be decreased */
static int adapter_num;

static int rmi4_bus_match(struct device *dev, struct device_driver *driver);
static int rmi4_bus_probe(struct device *dev);
static int rmi4_bus_remove(struct device *dev);

#ifdef CONFIG_PM
static int rmi4_bus_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	dev_dbg(dev, "%s - Called\n", __func__);

	if (pm)
		return pm_generic_suspend(dev);

	return 0;
}

static int rmi4_bus_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	dev_dbg(dev, "%s - Called\n", __func__);

	if (pm)
		return pm_generic_resume(dev);

	return 0;
}
static int rmi4_bus_freeze(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	dev_dbg(dev, "%s - Called\n", __func__);

	if (pm)
		return pm_generic_freeze(dev);

	return 0;
}

static int rmi4_bus_thaw(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	dev_dbg(dev, "%s - Called\n", __func__);

	if (pm)
		return pm_generic_thaw(dev);

	return 0;
}

static int rmi4_bus_poweroff(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	dev_dbg(dev, "%s - Called\n", __func__);

	if (pm)
		return pm_generic_poweroff(dev);

	return 0;
}

static int rmi4_bus_restore(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	dev_dbg(dev, "%s - Called\n", __func__);

	if (pm)
		return pm_generic_restore(dev);

	return 0;
}
#else
#define rmi4_bus_suspend	NULL
#define rmi4_bus_resume	NULL
#define rmi4_bus_freeze	NULL
#define rmi4_bus_thaw		NULL
#define rmi4_bus_poweroff	NULL
#define rmi4_bus_restore	NULL

#endif /* CONFIG_PM */

/* pm_ops contain a lot more features that should be implemented */
static const struct dev_pm_ops rmi4_bus_pm_ops = {
	.suspend = rmi4_bus_suspend,
	.resume = rmi4_bus_resume,
	.freeze = rmi4_bus_freeze,
	.thaw = rmi4_bus_thaw,
	.poweroff = rmi4_bus_poweroff,
	.restore = rmi4_bus_restore,
	SET_RUNTIME_PM_OPS(
		pm_generic_runtime_suspend,
		pm_generic_runtime_resume,
		pm_generic_runtime_idle
	)
};

static struct bus_type rmi4_bus_type = {
	.name		= "rmi4",
	.match		= rmi4_bus_match,
	.probe		= rmi4_bus_probe,
	.remove	= rmi4_bus_remove,
	.pm		= &rmi4_bus_pm_ops
};

static struct rmi4_function_device *verify_function_device(struct device *dev)
{
	return dev->type == &rmi4_function_device ?
		to_rmi4_func_core(dev) : NULL;
}

static struct rmi4_core_device *verify_core_device(struct device *dev)
{
	return dev->type == &rmi4_core_device ? to_rmi4_core_device(dev) : NULL;
}

static int rmi4_bus_match(struct device *dev, struct device_driver *drv)
{
	int match;
	struct rmi4_core_device *cdev = verify_core_device(dev);
	struct rmi4_function_device *fdev;

	if (cdev) {
		dev_dbg(dev,
			 "%s - rmi4_core_device. Matching '%s' with '%s'\n",
			__func__, cdev->name, drv->name);
		match = !strncmp(cdev->name, drv->name,
				 strnlen(cdev->name, RMI4_NAME_MAXLEN));
		goto exit;
	}

	fdev = verify_function_device(dev);
	if (fdev) {
		dev_dbg(dev,
			"%s - rmi4_function_device. Matching '%s' with '%s'\n",
			__func__, fdev->name, drv->name);
		match = !strncmp(fdev->name, drv->name,
				 strnlen(fdev->name, RMI4_NAME_MAXLEN));
		goto exit;
	}
	match = 0;

exit:
	dev_dbg(dev, "%s: %s matching '%s' driver\n\n\n", __func__,
		match ? "is" : "isn't", drv->name);

	return match;
}

static int rmi4_bus_probe(struct device *dev)
{
	int err;
	struct rmi4_core_device *cdev = verify_core_device(dev);
	struct rmi4_function_device *fdev;
	if (cdev) {
		struct rmi4_core_driver *cdrv =
			to_rmi4_core_driver(cdev->dev.driver);
		dev_dbg(dev, "%s - rmi4_core. Probing '%s'\n",
			__func__, cdev->name);
		if (cdrv && cdrv->probe) {
			err = cdrv->probe(cdev);
			if (!err) {
				dev_dbg(dev, "%s - Successfully probed %s\n",
					__func__, cdrv->drv.name);
				return err;
			}
			goto exit;
		}
	}
	fdev = verify_function_device(dev);
	if (fdev) {
		struct rmi4_function_driver *fdrv =
			to_rmi4_func_driver(fdev->dev.driver);
		dev_dbg(dev, "%s - rmi4_func. Probing '%s'\n",
			__func__, fdev->name);
		if (fdrv && fdrv->probe)
			return fdrv->probe(fdev);
	}

exit:
	return -EINVAL;
}

static int rmi4_bus_remove(struct device *dev)
{

	struct rmi4_core_device *cdev = verify_core_device(dev);
	struct rmi4_function_device *fdev;
	if (cdev) {
		struct rmi4_core_driver *cdrv =
			to_rmi4_core_driver(cdev->dev.driver);
		dev_dbg(dev, "%s - rmi4_core. Removing '%s'\n",
			__func__, cdev->name);
		if (cdrv && cdrv->probe)
			return cdrv->remove(cdev);
		goto exit;
	}
	fdev = verify_function_device(dev);
	if (fdev) {
		struct rmi4_function_driver *fdrv =
			to_rmi4_func_driver(fdev->dev.driver);
		dev_dbg(dev, "%s - rmi4_func. Removing '%s'\n",
			__func__, fdev->name);
		if (fdrv && fdrv->probe)
			return fdrv->remove(fdev);
	}

exit:
	return -ENODEV;
}

static void rmi4_bus_function_core_release(struct device *dev)
{
	struct rmi4_function_device *fdev = verify_function_device(dev);
	struct rmi4_function_driver *fdrv =
		to_rmi4_func_driver(dev->driver);

	if (!fdev) {
		dev_dbg(dev, "%s - Function pointer was no function_device\n",
			__func__);
		return;
	}

	dev_dbg(dev, "%s - Releasing function %s core\n", __func__,
		fdev->name);

	if (fdrv && fdrv->remove)
		fdrv->remove(fdev);
	else
		dev_dbg(dev, "%s - No function core driver registered\n",
			__func__);
}

static int rmi4_bus_function_core_match(struct device *dev, void *data)
{
	int ret;
	struct rmi4_function_data *fdata = data;
	struct rmi4_function_device *fdev = verify_function_device(dev);

	if (!fdev) {
		dev_dbg(dev, "%s - Not a function_device\n", __func__);
		return 0;
	}

	dev_dbg(dev, "%s - Matching '%s' with '%s'\n", __func__,
		 fdev->name, fdata->func_name);

	ret = strncmp(fdata->func_name, fdev->name,
		      strnlen(fdata->func_name, RMI4_NAME_MAXLEN));

	dev_dbg(dev, "%s - Match %s\n", __func__,
		ret ? "not found" : "found");

	return ret == 0;
}

static int rmi4_bus_match_adapter_unregister(struct device *dev, void *data)
{
	struct rmi4_core_device *cdev = verify_core_device(dev);

	dev_dbg(dev, "%s - Called\n", __func__);

	if (!cdev) {
		dev_dbg(dev, "%s - Not a core device\n", __func__);
		return 0;
	}

	if (dev->parent == (struct device *)data) {
		dev_dbg(dev, "%s - Match found\n", __func__);
		return 1;
	}

	dev_dbg(dev, "%s - Match not found\n", __func__);

	return 0;
}

int rmi4_bus_register_function_core(struct device *parent,
				    struct rmi4_function_data *fdata)
{
	int err;
	struct rmi4_function_device *fdev;

	if (IS_ERR_OR_NULL(parent)) {
		pr_err("%s - Need pointer to parent\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(fdata)) {
		dev_err(parent, "%s - Valid function data must be supplied\n",
			__func__);
		return -EINVAL;
	}

	if (!fdata->func_name) {
		dev_err(parent, "%s - Function name must be supplied\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(parent,
		"%s - Registering function F%02X core with name %s\n",
		__func__, fdata->func_id, fdata->func_name);

	fdev = kzalloc(sizeof(*fdev), GFP_KERNEL);
	if (!fdev) {
		dev_err(parent, "%s -Failed to create new function core\n",
			__func__);
		return -ENOMEM;
	}

	fdev->dev.parent = parent;
	fdev->dev.bus = &rmi4_bus_type;
	fdev->dev.type = &rmi4_function_device;
	fdev->dev.release = rmi4_bus_function_core_release;
	fdev->dev.platform_data = fdata->func_data;

	snprintf(fdev->name, sizeof(fdev->name), "%s", fdata->func_name);
	fdev->func_id = fdata->func_id;

	dev_set_name(&fdev->dev, "%s.%s", dev_name(parent), fdev->name);

	err = device_register(&fdev->dev);
	if (err) {
		dev_err(parent,
			"%s - Failed to register function core\n",
			__func__);
		return err;
	}

	dev_dbg(parent, "%s - Registered function core %s\n",
		__func__, fdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(rmi4_bus_register_function_core);

int rmi4_bus_unregister_function_core(struct device *parent,
				      struct rmi4_function_data *fdata)
{
	struct device *dev = NULL;
	struct rmi4_function_device *fdev;

	if (IS_ERR_OR_NULL(fdata)) {
		dev_err(parent, "%s - Function data missing\n", __func__);
		return -EINVAL;
	}

	do {
		dev = bus_find_device(&rmi4_bus_type, dev, fdata,
				      rmi4_bus_function_core_match);

		if (!dev) {
			dev_err(parent, "%s - Failed to find function_core\n",
				__func__);
			return -ENODEV;
		}

		if (parent != dev->parent) {
			dev_dbg(parent, "%s - Wrong parent\n",
				 __func__);
			continue;
		}

		fdev = verify_function_device(dev);
		if (fdev) {
			dev_dbg(parent,
				 "%s - Unregistering function core %s\n",
				 __func__, fdev->name);
			device_unregister(&fdev->dev);
			kfree(fdev);
			return 0;
		}
	} while (dev);

	return 0;
}
EXPORT_SYMBOL_GPL(rmi4_bus_unregister_function_core);

void rmi4_bus_adapter_release(struct device *dev)
{
	dev_dbg(dev, "%s - Called\n", __func__);
}

int rmi4_bus_register_adapter(struct device *parent, struct rmi4_comm_ops *ops,
			      struct rmi4_core_device_data *ddata)
{
	int err;
	struct rmi4_core_device *cdev;

	if (IS_ERR_OR_NULL(parent)) {
		pr_err("%s - Need pointer to parent device\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(ddata)) {
		dev_err(parent, "%s - Core device data must be supplied\n",
			__func__);
		return -EINVAL;
	}

	if (!ddata->core_name) {
		dev_err(parent, "%s - Core name must be supplied\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(parent, "%s - Called\n", __func__);

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev) {
		dev_err(parent,
			"%s - Failed to create a new adapter container\n",
			__func__);
		return -ENOMEM;
	}

	cdev->dev.parent = parent;
	cdev->dev.bus = &rmi4_bus_type;
	cdev->dev.type = &rmi4_core_device;
	cdev->dev.release = rmi4_bus_adapter_release;
	cdev->dev.platform_data = ddata;

	snprintf(cdev->name, sizeof(cdev->name), "%s", ddata->core_name);

	cdev->read = ops->chip_read;
	cdev->write = ops->chip_write;

	dev_set_name(&cdev->dev, "sensor%02d", adapter_num++);

	err = device_register(&cdev->dev);
	if (err)
		dev_err(parent, "%s - Failed to register adapter\n", __func__);
	else
		dev_dbg(parent, "%s - Successfully registered adapter\n",
			__func__);

	return err;
}
EXPORT_SYMBOL_GPL(rmi4_bus_register_adapter);

int rmi4_bus_unregister_adapter(struct device *parent)
{
	int ret = -ENODEV;
	struct device *dev;
	struct rmi4_core_device *cdev;

	if (IS_ERR_OR_NULL(parent)) {
		pr_err("%s - Need parent device of adapter\n", __func__);
		goto done;
	}

	dev_dbg(parent, "%s - Called\n", __func__);

	dev = bus_find_device(&rmi4_bus_type, NULL,
			      parent, rmi4_bus_match_adapter_unregister);

	cdev = verify_core_device(dev);

	if (cdev) {
		device_unregister(&cdev->dev);
		kfree(cdev);
		ret = 0;
		dev_dbg(parent, "%s - Successfully unregistered adapter\n",
			 __func__);
		goto done;
	}

	dev_err(parent, "%s - This adapter has not been registered\n",
		__func__);

done:
	return ret;
}
EXPORT_SYMBOL_GPL(rmi4_bus_unregister_adapter);

int rmi4_bus_register_core_driver(struct rmi4_core_driver *cdrv)
{
	cdrv->drv.bus = &rmi4_bus_type;

	return driver_register(&cdrv->drv);
}
EXPORT_SYMBOL_GPL(rmi4_bus_register_core_driver);

void rmi4_bus_unregister_core_driver(struct rmi4_core_driver *cdrv)
{
	driver_unregister(&cdrv->drv);
}
EXPORT_SYMBOL_GPL(rmi4_bus_unregister_core_driver);

int rmi4_bus_register_function_driver(struct rmi4_function_driver *fdrv)
{
	int err;

	pr_debug("%s - Registering function %s\n", __func__, fdrv->drv.name);

	fdrv->drv.bus = &rmi4_bus_type;

	err = driver_register(&fdrv->drv);
	if (err) {
		pr_err("%s - Failed to register function driver. Error: %d\n",
		       __func__, err);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rmi4_bus_register_function_driver);

void rmi4_bus_unregister_function_driver(struct rmi4_function_driver *fdrv)
{
	pr_debug("%s - Trying to remove function %s\n", __func__,
		 fdrv->drv.name);

	driver_unregister(&fdrv->drv);
}
EXPORT_SYMBOL_GPL(rmi4_bus_unregister_function_driver);

int rmi4_bus_read(struct rmi4_function_device *fdev,
		  enum rmi4_data_command func,  int addr_offset,
		  u8 *data, int data_len)
{
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	struct rmi4_core_driver *cdrv;

	if (!cdev)
		return -ENODEV;

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->read)
		return cdrv->read(fdev, func, addr_offset, data, data_len);

	dev_err(&fdev->dev, "%s - No read pointer initialized\n",
		 __func__);

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(rmi4_bus_read);


int rmi4_bus_write(struct rmi4_function_device *fdev,
		   enum rmi4_data_command func,
		   int addr_offset, u8 *data, int data_len)
{
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	struct rmi4_core_driver *cdrv;

	if (!cdev)
		return -ENODEV;

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdev && cdrv->write)
		return cdrv->write(fdev, func, addr_offset, data, data_len);

	dev_err(&fdev->dev, "%s - No write pointer initialized\n",
		 __func__);

	return -ENODEV;

}
EXPORT_SYMBOL_GPL(rmi4_bus_write);

int rmi4_bus_update_pdt(struct rmi4_function_device *fdev)
{
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	struct rmi4_core_driver *cdrv;

	if (!cdev)
		return -ENODEV;

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->read_pdt)
		return cdrv->read_pdt(fdev);

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(rmi4_bus_update_pdt);

int rmi4_bus_request_irq(struct rmi4_function_device *fdev, void *data,
			 void (*fn)(int, void *), u8 mask)
{
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	struct rmi4_core_driver *cdrv;

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);
	if (!cdev) {
		dev_dbg(&fdev->dev, "%s - Parent is no core device\n",
			 __func__);
		return -ENODEV;
	}

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->request_irq) {
		dev_dbg(&fdev->dev, "%s - Calling IRQ request\n", __func__);
		return cdrv->request_irq(fdev, data, fn, mask);
	}

	dev_dbg(&fdev->dev, "%s - No IRQ request handler registered\n",
		__func__);

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(rmi4_bus_request_irq);

int rmi4_bus_free_irq(struct rmi4_function_device *fdev, void *data)
{
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	struct rmi4_core_driver *cdrv;

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);
	if (!cdev) {
		dev_dbg(&fdev->dev, "%s - No parent device\n", __func__);
		return -ENODEV;
	}

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->free_irq)
		return cdrv->free_irq(fdev, data);

	dev_dbg(&fdev->dev,
		"%s - No IRQ request function registered\n",
		__func__);
	return -ENODEV;

}
EXPORT_SYMBOL_GPL(rmi4_bus_free_irq);

int rmi4_bus_set_non_essential_irq_status(struct rmi4_function_device *fdev,
					  bool disabled)
{
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	struct rmi4_core_driver *cdrv;

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);
	if (!cdev) {
		dev_dbg(&fdev->dev, "%s - No parent device\n", __func__);
		return -ENODEV;
	}

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->disable_non_essential_irqs)
		return cdrv->disable_non_essential_irqs(fdev, disabled);

	dev_dbg(&fdev->dev,
		"%s - No IRQ disable function registered\n",
		__func__);

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(rmi4_bus_set_non_essential_irq_status);

int rmi4_bus_request_notification(
	struct rmi4_function_device *fdev, enum rmi4_notification_event events,
	void (*callback)(enum rmi4_notification_event event, void *data),
	void *data)
{
	struct rmi4_core_driver *cdrv;
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	if (!cdev) {
		dev_err(&fdev->dev, "%s - No parent device\n", __func__);
		return -ENODEV;
	}

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->request_notification)
		return cdrv->request_notification(fdev, events, callback, data);

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(rmi4_bus_request_notification);

void rmi4_bus_release_notification(struct rmi4_function_device *fdev,
				   void *data)
{
	struct rmi4_core_driver *cdrv;
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	if (!cdev) {
		dev_err(&fdev->dev, "%s - No parent device\n", __func__);
		return;
	}

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->release_notification)
		cdrv->release_notification(fdev, data);

}
EXPORT_SYMBOL_GPL(rmi4_bus_release_notification);

void rmi4_bus_notify(struct rmi4_function_device *fdev,
			     enum rmi4_notification_event event)
{
	struct rmi4_core_driver *cdrv;
	struct rmi4_core_device *cdev = verify_core_device(fdev->dev.parent);
	if (!cdev) {
		dev_err(&fdev->dev, "%s - No parent device\n", __func__);
		return;
	}

	cdrv = to_rmi4_core_driver(cdev->dev.driver);
	if (cdrv && cdrv->notify)
		cdrv->notify(fdev, event);

}
EXPORT_SYMBOL_GPL(rmi4_bus_notify);

struct device * __must_check rmi4_bus_create_char_dev(
	const char *name, const struct file_operations *fops, struct cdev *cdev,
	struct device *parent, void *drvdata)
{
	int err;
	dev_t dev_no;
	struct device *dev = NULL;

	if (IS_ERR_OR_NULL(parent)) {
		pr_err("%s - Must supply parent device\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR_OR_NULL(rmi4_char_dev_class)) {
		dev_err(parent,
			"%s - RMI4 class is not initialized. "
			"Can't create char dev\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	if (rmi4_char_dev_major) {
		dev_no = MKDEV(rmi4_char_dev_major, rmi4_char_dev_minor++);
		err = register_chrdev_region(dev_no, 1, name);
	} else {
		err = alloc_chrdev_region(&dev_no, 0, 1, name);
		rmi4_char_dev_major = MAJOR(dev_no);
		dev_dbg(parent, "Major number of rmi_char_dev: %d\n",
			 rmi4_char_dev_major);
	}
	if (err < 0) {
		dev_err(parent,
			"Failed to get minor dev number %d, code %d.\n",
			rmi4_char_dev_minor - 1, err);
		return ERR_PTR(err);
	}

	dev_dbg(parent, "Allocated rmidev %d %d.\n",
		MAJOR(dev_no), MINOR(dev_no));

	cdev_init(cdev, fops);

	err = cdev_add(cdev, dev_no, 1);
	if (err) {
		dev_err(parent, "Error %d adding rmi_char_dev.\n", err);
		goto error;
	}

	dev = device_create(rmi4_char_dev_class, parent, dev_no, drvdata,
			    "%s%d", name, MINOR(dev_no));
	if (IS_ERR(dev)) {
		dev_err(parent, "%s - Failed to create rmi_char_dev\n",
			__func__);
		goto error_cdev_add;
	}

	dev->parent = parent;

	return dev;

error_cdev_add:
	cdev_del(cdev);
error:
	unregister_chrdev_region(dev_no, 1);

	return IS_ERR(dev) ? dev : ERR_PTR(err);

}
EXPORT_SYMBOL_GPL(rmi4_bus_create_char_dev);

void rmi4_bus_destroy_char_dev(struct device *parent, struct cdev *cdev)
{
	device_destroy(rmi4_char_dev_class, cdev->dev);
	unregister_chrdev_region(cdev->dev, 1);
}
EXPORT_SYMBOL_GPL(rmi4_bus_destroy_char_dev);

static char *rmi4_bus_char_devnode(struct device *dev, mode_t *mode)
{
	if (!mode)
		return NULL;

	/**mode = 0666*/
	*mode = (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);

	return kasprintf(GFP_KERNEL, "rmi/%s", dev_name(dev));
}

static int __devinit rmi4_bus_init(void)
{
	int ret;

	/* If this fails, it's not a major error. Just means that it will not
	 * be possible to connect external debug/tuning equipment. This is
	 * something that normally will not be done, so we just warn */
	rmi4_char_dev_class = class_create(THIS_MODULE, RMI4_BUS_CLASS_NAME);
	if (IS_ERR(rmi4_char_dev_class)) {
		pr_err("%s - Failed to create RMI char_dev class. "
		       "Full debug may not be available\n", __func__);
	} else {
		rmi4_char_dev_class->devnode = rmi4_bus_char_devnode;
		pr_info("%s - Successfully created RMI4 class\n", __func__);
	}

	ret = bus_register(&rmi4_bus_type);
	if (ret)
		pr_err("%s: error registering the RMI4 bus: %d\n", __func__,
		       ret);
	else
		pr_info("%s: successfully registered RMI4 bus.\n", __func__);

	return ret;
}

static void __devexit rmi4_bus_exit(void)
{
	pr_info("%s - Unregistering RMI4 bus\n", __func__);
	bus_unregister(&rmi4_bus_type);
}

module_init(rmi4_bus_init);
module_exit(rmi4_bus_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonyerisson.com>");
MODULE_DESCRIPTION("RMI4 bus driver");
MODULE_LICENSE("GPL");
