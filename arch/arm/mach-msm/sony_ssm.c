/* kernel/arch/arm/mach-ux500/sony_ssm.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Mattias Larsson <mattias7.larsson@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/suspend.h>
#include <linux/earlysuspend.h>

#define MODULE_NAME "sony_ssm"

enum ssm_state {
	SSM_SUSPEND_PREPARE = 0,
	SSM_LATE_RESUME = 1
};

struct ssm_data {
	struct early_suspend early_suspend;
	struct notifier_block pm_notifier;
	struct kobject *kobj;
	struct mutex lock;
	bool enabled;
	bool notify_next_suspend_prepare;
	bool notify_late_resume;
};

static void ssm_notify(struct ssm_data *sd, enum ssm_state state)
{
	char event[8];
	char *envp[] = {event, NULL};

	snprintf(event, sizeof(event), "EVENT=%d", state);

	pr_debug("%s: Sending uevent EVENT=%d\n", __func__, state);

	kobject_uevent_env(sd->kobj, KOBJ_CHANGE, envp);
}

static void ssm_late_resume(struct early_suspend *h)
{
	struct ssm_data *sd = container_of(h, struct ssm_data, early_suspend);

	pr_debug("%s\n", __func__);

	mutex_lock(&sd->lock);
	if (sd->notify_late_resume)
		ssm_notify(sd, SSM_LATE_RESUME);
	mutex_unlock(&sd->lock);
}

static int ssm_pm_notifier(struct notifier_block *nb, unsigned long event,
		void *ignored)
{
	struct ssm_data *sd = container_of(nb, struct ssm_data, pm_notifier);

	pr_debug("%s: event=%lu\n", __func__, event);

	if (event == PM_SUSPEND_PREPARE) {
		mutex_lock(&sd->lock);
		if (sd->notify_next_suspend_prepare) {
			ssm_notify(sd, SSM_SUSPEND_PREPARE);
			sd->notify_next_suspend_prepare = false;
			mutex_unlock(&sd->lock);
			return NOTIFY_BAD;
		}
		mutex_unlock(&sd->lock);
	}

	return NOTIFY_DONE;
}

static int ssm_enable(struct ssm_data *sd)
{
	int rc;

	pr_debug("%s\n", __func__);

	mutex_lock(&sd->lock);

	if (sd->enabled) {
		pr_err("%s: Already enabled!\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	rc = register_pm_notifier(&sd->pm_notifier);
	if (rc) {
		pr_err("%s: register_pm_notifier failed: %d\n", __func__, rc);
		goto exit;
	}

	register_early_suspend(&sd->early_suspend);

	sd->notify_next_suspend_prepare = false;
	sd->notify_late_resume = false;
	sd->enabled = true;

exit:
	mutex_unlock(&sd->lock);
	return rc;
}

static void ssm_disable(struct ssm_data *sd)
{
	pr_debug("%s\n", __func__);

	mutex_lock(&sd->lock);
	if (sd->enabled) {
		unregister_early_suspend(&sd->early_suspend);
		unregister_pm_notifier(&sd->pm_notifier);
		sd->enabled = false;
	} else {
		pr_warn("%s: Not enabled", __func__);
	}
	mutex_unlock(&sd->lock);
}

static ssize_t ssm_store_enable(struct device *pdev,
		struct device_attribute *attr, const char *pbuf, size_t count)
{
	int rc = 0, val;
	struct ssm_data *sd = dev_get_drvdata(pdev);

	pr_debug("%s: %s\n", __func__, pbuf);

	sscanf(pbuf, "%d", &val);

	if (!!val)
		rc = ssm_enable(sd);
	else
		ssm_disable(sd);

	return rc == 0 ? count : rc;
}

static ssize_t ssm_store_request_next_suspend_prepare_notification(
		struct device *pdev, struct device_attribute *attr,
		const char *pbuf, size_t count)
{
	int rc = 0, val;
	struct ssm_data *sd = dev_get_drvdata(pdev);

	pr_debug("%s: %s\n", __func__, pbuf);

	sscanf(pbuf, "%d", &val);

	mutex_lock(&sd->lock);
	if (sd->enabled) {
		sd->notify_next_suspend_prepare = !!val;
	} else {
		rc = -EINVAL;
		pr_err("%s: Notifications are not enabled!", __func__);
	}
	mutex_unlock(&sd->lock);

	return rc == 0 ? count : rc;
}

static ssize_t ssm_store_set_late_resume_notifications(struct device *pdev,
		struct device_attribute *attr, const char *pbuf, size_t count)
{
	int rc = 0, val;
	struct ssm_data *sd = dev_get_drvdata(pdev);

	pr_debug("%s: %s\n", __func__, pbuf);

	sscanf(pbuf, "%d", &val);

	mutex_lock(&sd->lock);
	if (sd->enabled) {
		sd->notify_late_resume = !!val;
	} else {
		rc = -EINVAL;
		pr_err("%s: Notifications are not enabled!", __func__);
	}
	mutex_unlock(&sd->lock);

	return rc == 0 ? count : rc;
}

static struct device_attribute ssm_attrs[] = {
	__ATTR(enable, 0600, NULL,
			ssm_store_enable),
	__ATTR(set_request_next_suspend_prepare_notification, 0600, NULL,
			ssm_store_request_next_suspend_prepare_notification),
	__ATTR(set_late_resume_notifications, 0600, NULL,
			ssm_store_set_late_resume_notifications),
};

static int ssm_create_attrs(struct device *dev)
{
	unsigned int i;
	for (i = 0; i < ARRAY_SIZE(ssm_attrs); i++)
		if (device_create_file(dev, &ssm_attrs[i]))
			goto err;
	return 0;
err:
	while (i--)
		device_remove_file(dev, &ssm_attrs[i]);
	return -EIO;
}

static void ssm_remove_attrs(struct device *dev)
{
	unsigned int i;
	for (i = 0; i < ARRAY_SIZE(ssm_attrs); i++)
		(void)device_remove_file(dev, &ssm_attrs[i]);
}

static int ssm_probe(struct platform_device *pdev)
{
	int rc;
	struct ssm_data *sd;

	pr_debug("%s\n", __func__);

	sd = kzalloc(sizeof(struct ssm_data), GFP_KERNEL);
	if (!sd) {
		pr_err("%s: OOM for ssm_data\n", __func__);
		return -ENOMEM;
	}

	sd->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	sd->early_suspend.resume = ssm_late_resume;

	sd->kobj = kobject_get(&pdev->dev.kobj);

	mutex_init(&sd->lock);

	sd->enabled = false;
	sd->notify_next_suspend_prepare = false;
	sd->notify_late_resume = false;

	sd->pm_notifier.notifier_call = ssm_pm_notifier;
	sd->pm_notifier.priority = 0;

	rc = ssm_create_attrs(&pdev->dev);
	if (rc) {
		pr_err("%s: ssm_create_attrs failed: %d\n", __func__, rc);
		goto err_create_attrs;
	}

	platform_set_drvdata(pdev, sd);

	return 0;

err_create_attrs:
	kobject_put(sd->kobj);
	kfree(sd);
	return rc;
}

static int ssm_remove(struct platform_device *pdev)
{
	struct ssm_data *sd = platform_get_drvdata(pdev);

	pr_debug("%s\n", __func__);

	ssm_remove_attrs(&pdev->dev);
	kobject_put(sd->kobj);
	kfree(sd);

	return 0;
}

static struct platform_driver ssm_driver = {
	.probe = ssm_probe,
	.remove = ssm_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ssm_init(void)
{
	pr_debug("%s\n", __func__);
	return platform_driver_register(&ssm_driver);
}

static void __exit ssm_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&ssm_driver);
}

late_initcall(ssm_init);
module_exit(ssm_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Power management noticiations for Super Stamina Mode");
