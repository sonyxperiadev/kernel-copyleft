/*
 * cyttsp4_bus.c
 * Cypress TrueTouch(TM) Standard Product V4 Bus Driver.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2012 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * Author: Aleksej Makarov aleksej.makarov@sonyericsson.com
 * Modified by: Cypress Semiconductor for complete set of TTSP Bus interfaces.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/cyttsp4_bus.h>

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/limits.h>

static DEFINE_MUTEX(core_lock);
static LIST_HEAD(adapter_list);
static LIST_HEAD(core_dev_list);
static LIST_HEAD(cyttsp4_dev_list);

struct bus_type cyttsp4_bus_type;

static void cyttsp4_dev_release(struct device *dev)
{
	dev_vdbg(dev, "%s: Enter\n", __func__);
	put_device(dev->parent);
}

static struct device_type cyttsp4_dev_type = {
	.release = cyttsp4_dev_release
};

static struct device_type cyttsp4_core_type = {
	.release = cyttsp4_dev_release
};

static int _cyttsp4_register_dev(struct cyttsp4_device *pdev,
		struct cyttsp4_core *core)
{
	int ret;

	if (!pdev->dev.parent)
		pdev->dev.parent = get_device(&core->dev);
	/* Assign (new) core */
	pdev->core = core;
	/* Check whether this device is registered before */
	if (pdev->dev.bus == &cyttsp4_bus_type &&
			pdev->dev.type == &cyttsp4_dev_type)
		return 0;

	pdev->dev.bus = &cyttsp4_bus_type;
	pdev->dev.type = &cyttsp4_dev_type;
	dev_set_name(&pdev->dev, "%s.%s", pdev->name,  core->id);

	ret = device_register(&pdev->dev);
	dev_dbg(&pdev->dev, "%s: Registering device '%s'. Parent at '%s', " \
		" err = %d\n",
		 __func__, dev_name(&pdev->dev),
		 dev_name(pdev->dev.parent), ret);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to register device, err %d\n",
			__func__, ret);
		pdev->dev.bus = NULL;
		pdev->dev.type = NULL;
		pdev->core = NULL;
	}
	return ret;
}

static int _cyttsp4_register_core(struct cyttsp4_core *pdev,
		struct cyttsp4_adapter *adap)
{
	int ret;

	if (!pdev->dev.parent)
		pdev->dev.parent = get_device(adap->dev);
	/* Assign (new) adapter */
	pdev->adap = adap;
	/* Check whether this core is registered before */
	if (pdev->dev.bus == &cyttsp4_bus_type &&
			pdev->dev.type == &cyttsp4_core_type)
		return 0;

	pdev->dev.bus = &cyttsp4_bus_type;
	pdev->dev.type = &cyttsp4_core_type;
	dev_set_name(&pdev->dev, "%s.%s", pdev->id,  adap->id);

	ret = device_register(&pdev->dev);
	dev_dbg(&pdev->dev, "%s: Registering device '%s'. Parent at '%s', " \
		" err = %d\n",
		 __func__, dev_name(&pdev->dev),
		 dev_name(pdev->dev.parent), ret);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to register device, err %d\n",
			__func__, ret);
		pdev->dev.bus = NULL;
		pdev->dev.type = NULL;
		pdev->adap = NULL;
	}
	return ret;
}

static struct cyttsp4_adapter *find_adapter(char const *adap_id)
{
	struct cyttsp4_adapter *a;

	list_for_each_entry(a, &adapter_list, node)
		if (!strncmp(a->id, adap_id, NAME_MAX))
			return a;
	return NULL;
}

static struct cyttsp4_core *find_core(char const *core_id)
{
	struct cyttsp4_core *d;

	list_for_each_entry(d, &core_dev_list, node)
		if (!strncmp(d->id, core_id, NAME_MAX) && d->dev.driver)
			return d;
	return NULL;
}

static void rescan_devices(struct cyttsp4_core *core)
{
	struct cyttsp4_device *d;

	list_for_each_entry(d, &cyttsp4_dev_list, node)
		if (!d->core && !strncmp(core->id, d->core_id, NAME_MAX))
			_cyttsp4_register_dev(d, core);
}

static void rescan_cores(struct cyttsp4_adapter *adap)
{
	struct cyttsp4_core *d;

	list_for_each_entry(d, &core_dev_list, node)
		if (!d->adap && !strncmp(adap->id, d->adap_id, NAME_MAX))
			_cyttsp4_register_core(d, adap);
}

int cyttsp4_register_device(struct cyttsp4_device *pdev)
{
	int ret = 0;
	struct cyttsp4_core *core;

	if (!pdev)
		return -EINVAL;
	mutex_lock(&core_lock);
	list_add(&pdev->node, &cyttsp4_dev_list);
	pr_debug("%s: '%s' added to cyttsp4_dev_list\n", __func__, pdev->name);
	core = find_core(pdev->core_id);
	if (core)
		ret = _cyttsp4_register_dev(pdev, core);
	mutex_unlock(&core_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cyttsp4_register_device);

static int cyttsp4_match_dev(struct device *dev, void *data)
{
	return dev == (struct device *)data;
}

void cyttsp4_unregister_device(struct cyttsp4_device *pdev)
{
	if (!pdev)
		return;
	mutex_lock(&core_lock);
	if (bus_find_device(&cyttsp4_bus_type, NULL, &pdev->dev,
			cyttsp4_match_dev)) {
		dev_dbg(&pdev->dev, "%s: Unregistering device '%s'.\n",
			__func__, dev_name(&pdev->dev));
		/* Put reference taken by bus_find_device() */
		put_device(&pdev->dev);
		device_unregister(&pdev->dev);
	}
	list_del(&pdev->node);
	pr_debug("%s: '%s' removed from cyttsp4_dev_list\n", __func__,
		pdev->name);
	mutex_unlock(&core_lock);
}
EXPORT_SYMBOL_GPL(cyttsp4_unregister_device);

int cyttsp4_register_core_device(struct cyttsp4_core *pdev)
{
	int ret = 0;
	struct cyttsp4_adapter *adap;

	if (!pdev)
		return -EINVAL;
	mutex_lock(&core_lock);
	if (find_core(pdev->id)) {
		pr_debug("%s: core id '%s' already exists\n",
				__func__, pdev->id);
		ret = -EINVAL;
		goto fail;
	}
	list_add(&pdev->node, &core_dev_list);
	pr_debug("%s: '%s' added to core_dev_list\n", __func__, pdev->name);
	adap = find_adapter(pdev->adap_id);
	if (adap) {
		pr_debug("%s: adapter for '%s' is '%s'\n", __func__,
				pdev->id, dev_name(adap->dev));
		ret = _cyttsp4_register_core(pdev, adap);
		if (!ret)
			rescan_devices(pdev);
	}
fail:
	mutex_unlock(&core_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cyttsp4_register_core_device);

int cyttsp4_add_adapter(char const *id, struct cyttsp4_ops const *ops,
		struct device *parent)
{
	int rc = 0;
	struct cyttsp4_adapter *a;

	if (!parent) {
		dev_err(parent, "%s: need parent for '%s'\n", __func__, id);
		return -EINVAL;
	}
	mutex_lock(&core_lock);
	if (find_adapter(id)) {
		dev_err(parent, "%s: adapter '%s' already exists\n",
				__func__, id);
		rc = -EINVAL;
		goto fail;
	}
	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a) {
		dev_err(parent, "%s: failed to allocate adapter '%s'\n",
				__func__, id);
		rc = -ENOMEM;
		goto fail;
	}
	memcpy(a->id, id, sizeof(a->id));
	a->id[sizeof(a->id) - 1] = 0;
	a->read = ops->read;
	a->write = ops->write;
	a->dev = parent;
	list_add(&a->node, &adapter_list);
	dev_dbg(parent, "%s: '%s' added to adapter_list\n", __func__, id);
	rescan_cores(a);
fail:
	mutex_unlock(&core_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp4_add_adapter);

int cyttsp4_del_adapter(char const *id)
{
	int rc = 0;
	struct cyttsp4_adapter *adap;
	struct cyttsp4_core *core_dev;

	mutex_lock(&core_lock);
	adap = find_adapter(id);
	if (!adap) {
		pr_err("%s: adapter '%s' does not exist\n",
			__func__, id);
		rc = -EINVAL;
		goto fail;
	}

	list_for_each_entry(core_dev, &core_dev_list, node)
		if (core_dev->adap == adap)
			core_dev->adap = NULL;

	list_del(&adap->node);
	kfree(adap);
	pr_debug("%s: '%s' removed from adapter_list\n", __func__, id);
fail:
	mutex_unlock(&core_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp4_del_adapter);

static struct cyttsp4_device *verify_device_type(struct device *dev)
{
	return dev->type == &cyttsp4_dev_type ? to_cyttsp4_device(dev) : NULL;
}

static struct cyttsp4_core *verify_core_type(struct device *dev)
{
	return dev->type == &cyttsp4_core_type ? to_cyttsp4_core(dev) : NULL;
}

static int cyttsp4_match_device(struct cyttsp4_device *dev, const char *name)
{
	return strncmp(dev->name, name, NAME_MAX) == 0;
}

static int cyttsp4_match_core_device(struct cyttsp4_core *core,
		const char *name)
{
	return strncmp(core->name, name, NAME_MAX) == 0;
}

static int cyttsp4_device_match(struct device *dev, struct device_driver *drv)
{
	struct cyttsp4_device *cyttsp4_dev = verify_device_type(dev);
	struct cyttsp4_core *cyttsp4_core;
	int match;

	if (cyttsp4_dev) {
		match = cyttsp4_match_device(cyttsp4_dev, drv->name);
		goto exit;
	}
	cyttsp4_core = verify_core_type(dev);
	if (cyttsp4_core) {
		match = cyttsp4_match_core_device(cyttsp4_core, drv->name);
		goto exit;
	}
	match = 0;
exit:
	dev_dbg(dev, "%s: %s matching '%s' driver\n", __func__,
			match ? "is" : "isn't", drv->name);
	return match;
}

static ssize_t modalias_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	struct cyttsp4_device *cyttsp4_dev = verify_device_type(dev);
	struct cyttsp4_core *cyttsp4_core;

	char const *name;
	int len;

	if (cyttsp4_dev) {
		name = cyttsp4_dev->name;
		goto exit;
	}
	cyttsp4_core = verify_core_type(dev);
	if (cyttsp4_core) {
		name = cyttsp4_core->id;
		goto exit;
	}
	name = "none";
exit:
	len = snprintf(buf, PAGE_SIZE, "ttsp4:%s\n", name);
	return (len >= PAGE_SIZE) ? (PAGE_SIZE - 1) : len;
}

static struct device_attribute cyttsp4_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};

#ifdef CONFIG_SUSPEND
static int cyttsp4_pm_suspend(struct device *dev)
{
	struct device_driver *drv = dev->driver;

	dev_dbg(dev, "%s\n", __func__);
	if (drv && drv->pm && drv->pm->suspend)
		return drv->pm->suspend(dev);
	return 0;
}

static int cyttsp4_pm_resume(struct device *dev)
{
	struct device_driver *drv = dev->driver;

	dev_dbg(dev, "%s\n", __func__);
	if (drv && drv->pm && drv->pm->resume)
		return drv->pm->resume(dev);
	return 0;
}
#else /* !CONFIG_SUSPEND */
#define cyttsp4_pm_suspend		NULL
#define cyttsp4_pm_resume		NULL
#endif /* !CONFIG_SUSPEND */

#ifdef CONFIG_PM_RUNTIME
#define cyttsp4_pm_rt_suspend		pm_generic_runtime_suspend
#define cyytsp4_pm_rt_resume		pm_generic_runtime_resume
#define cyytsp4_pm_rt_idle		pm_generic_runtime_idle
#else /* !CONFIG_PM_RUNTIME */
#define cyttsp4_pm_rt_suspend		NULL
#define cyytsp4_pm_rt_resume		NULL
#define cyytsp4_pm_rt_idle		NULL
#endif /* !CONFIG_PM_RUNTIME */

static const struct dev_pm_ops cyttsp4_dev_pm_ops = {
	.suspend = cyttsp4_pm_suspend,
	.resume = cyttsp4_pm_resume,
	.runtime_suspend = cyttsp4_pm_rt_suspend,
	.runtime_resume = cyytsp4_pm_rt_resume,
	.runtime_idle = cyytsp4_pm_rt_idle,
};

struct bus_type cyttsp4_bus_type = {
	.name		= "ttsp4",
	.dev_attrs	= cyttsp4_dev_attrs,
	.match		= cyttsp4_device_match,
	.uevent		= NULL,
	.pm		= &cyttsp4_dev_pm_ops,
};
EXPORT_SYMBOL_GPL(cyttsp4_bus_type);

static int cyttsp4_drv_remove(struct device *_dev)
{
	struct cyttsp4_driver *drv = to_cyttsp4_driver(_dev->driver);
	struct cyttsp4_device *dev = to_cyttsp4_device(_dev);
	return drv->remove(dev);
}

static int cyttsp4_core_drv_remove(struct device *_dev)
{
	struct cyttsp4_core_driver *drv = to_cyttsp4_core_driver(_dev->driver);
	struct cyttsp4_core *dev = to_cyttsp4_core(_dev);
	return drv->remove(dev);
}

static int cyttsp4_drv_probe(struct device *_dev)
{
	struct cyttsp4_driver *drv = to_cyttsp4_driver(_dev->driver);
	struct cyttsp4_device *dev = to_cyttsp4_device(_dev);
	struct cyttsp4_core *core = dev->core;
	int rc;

	BUG_ON(!core);
	BUG_ON(!core->dev.driver);

	/* Increase usage count of the core driver*/
	__module_get(core->dev.driver->owner);

	rc = drv->probe(dev);
	if (rc)
		module_put(core->dev.driver->owner);
	dev_dbg(_dev, "%s: for %s = %d\n", __func__, dev->name, rc);
	return rc;
}

static int cyttsp4_core_drv_probe(struct device *_dev)
{
	struct cyttsp4_core_driver *drv = to_cyttsp4_core_driver(_dev->driver);
	struct cyttsp4_core *dev = to_cyttsp4_core(_dev);
	struct cyttsp4_adapter *adap = dev->adap;
	int rc;

	BUG_ON(!adap);
	BUG_ON(!adap->dev->driver);

	/* Increase usage count of the adapter driver*/
	__module_get(adap->dev->driver->owner);

	rc = drv->probe(dev);
	dev_dbg(_dev, "%s: for %s = %d\n", __func__, dev->name, rc);
	if (!rc)
		rescan_devices(dev);
	else
		module_put(adap->dev->driver->owner);
	return rc;
}

int cyttsp4_register_driver(struct cyttsp4_driver *drv)
{
	struct cyttsp4_device *d;
	int ret = 0;

	/*
	 * We need to ensure that the driver of this device's
	 * core device should exist (dependency)
	 * To do so, we traverse through the device, its core
	 * device and the driver of its core device, which requires
	 * the device itself should be registered with the system
	 */
	mutex_lock(&core_lock);
	list_for_each_entry(d, &cyttsp4_dev_list, node) {
		if (!cyttsp4_match_device(d, drv->driver.name))
			continue;
		if (d->core) {
			if (d->core->dev.driver)
				ret = ref_module(drv->driver.owner,
					d->core->dev.driver->owner);
			else
				/* Core device exists but not core driver */
				ret = -ENODEV;
		}
		break;
	}
	mutex_unlock(&core_lock);

	if (ret) {
		if (ret == -ENODEV)
			pr_err("%s: Core driver module does not exist\n",
				__func__);
		else
			pr_err("%s: Error getting ref to core driver module\n",
				__func__);
		goto fail;
	}

	drv->driver.bus = &cyttsp4_bus_type;
	if (drv->probe)
		drv->driver.probe = cyttsp4_drv_probe;
	if (drv->remove)
		drv->driver.remove = cyttsp4_drv_remove;
	ret = driver_register(&drv->driver);
fail:
	pr_debug("%s: '%s' returned %d\n", __func__, drv->driver.name, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(cyttsp4_register_driver);

int cyttsp4_register_core_driver(struct cyttsp4_core_driver *drv)
{
	struct cyttsp4_core *d;
	int ret = 0;

	/*
	 * We need to ensure that the driver of this core device's
	 * adapter should exist (dependency)
	 * To do so, we traverse through the core device, its adapter
	 * and the driver of its adapter, which requires the core
	 * device itself should be registered with the system
	 */
	mutex_lock(&core_lock);
	list_for_each_entry(d, &core_dev_list, node) {
		if (!cyttsp4_match_core_device(d, drv->driver.name))
			continue;
		if (d->adap) {
			if (d->adap->dev && d->adap->dev->driver) {
				ret = ref_module(drv->driver.owner,
					d->adap->dev->driver->owner);
			} else {
				/* Core dev exist but not adap device
				 * Do not let until adap module inserted */
				ret = -ENODEV;
			}
		}
		break;
	}
	mutex_unlock(&core_lock);

	if (ret) {
		if (ret == -ENODEV)
			pr_err("%s: Adapter driver module does not exist\n",
				__func__);
		else
			pr_err("%s: Error get ref to adapter driver module\n",
				__func__);
		goto fail;
	}

	drv->driver.bus = &cyttsp4_bus_type;
	if (drv->probe)
		drv->driver.probe = cyttsp4_core_drv_probe;
	if (drv->remove)
		drv->driver.remove = cyttsp4_core_drv_remove;
	ret = driver_register(&drv->driver);
fail:
	pr_debug("%s: '%s' returned %d\n", __func__, drv->driver.name, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(cyttsp4_register_core_driver);

void cyttsp4_unregister_driver(struct cyttsp4_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(cyttsp4_unregister_driver);

void cyttsp4_unregister_core_driver(struct cyttsp4_core_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(cyttsp4_unregister_core_driver);

int __init cyttsp4_bus_init(void)
{
	int error;
	error =  bus_register(&cyttsp4_bus_type);
	if (error)
		pr_err("%s: error %d\n", __func__, error);
	else
		pr_debug("%s: ok\n", __func__);
	return error;
}

static void __exit cyttsp4_bus_exit(void)
{
	pr_debug("%s: ok\n", __func__);
}

subsys_initcall(cyttsp4_bus_init);
module_exit(cyttsp4_bus_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
