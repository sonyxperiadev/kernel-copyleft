/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTE: This file has been modified by Sony Mobile Communications AB.
 * Modifications are licensed under the License.
 */
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/ratelimit.h>
#include <linux/kernel.h>
#include "mach/ssr_monitor.h"
MODULE_DESCRIPTION("SSR Monitor Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");

static struct ssr_monitor_dev *ssr_md;
static char ssr_monitor_crash_reason[100];


static int ssr_monitor_open(struct inode *inode, struct file *file)
{
	printk(KERN_ERR"ssr_monitor_open\n");
	return 0;
}

static long ssr_monitor_ioctl(struct file *filp, unsigned int iocmd, \
	unsigned long ioarg)
{
	return 0;
}

static int ssr_monitor_release(struct inode *inode, struct file *file)
{
	printk(KERN_ERR "ssr_monitor_release");
	return 0;
}

static int ssr_monitor_read(struct file *file, char __user *buf, \
	size_t count, loff_t *ppos)
{

	if (!buf)
		return -EPERM;

	if (count < sizeof(ssr_monitor_crash_reason)) {
		printk(KERN_ERR "ssr_monitor_read: user buf is too small\n");
		return -EPERM;
	}

	if (!ssr_md->readyet)
		ssr_md->readyet = 1;
	wait_event_interruptible(ssr_md->ssr_monitor_wait_q, \
		ssr_md->data_ready);

	if (copy_to_user(buf, (void *)ssr_monitor_crash_reason, \
		sizeof(ssr_monitor_crash_reason))) {
		printk(KERN_ERR "ssr_monitor_read: copy_to_user failed\n");
		return -EPERM;
	}

	ssr_md->data_ready = 0;
	printk(KERN_ERR "crash reason:%s", ssr_monitor_crash_reason);
	return sizeof(ssr_monitor_crash_reason);
}

static const struct file_operations ssrcharfops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ssr_monitor_ioctl,
	.open = ssr_monitor_open,
	.read = ssr_monitor_read,
	.release = ssr_monitor_release
};

int ssr_monitor_notify(char *name)
{
	int ret = 0;
	if (!name) {
		pr_err("ssr_monitor_notify: no subsystem name!!!\n");
		ret = -1;
		goto EXIT;
	}

	if (ssr_md->initialized != 1) {
		pr_err("ssr_monitor_notify: ssr_monitor no initialized!!!\n");
		ret = -1;
		goto EXIT;
	}

	if (strncmp(name, "ramdump_modem", sizeof("ramdump_modem")) == 0) {
		queue_work(ssr_md->ssr_monitor_wait_wq, \
			&(ssr_md->ssr_monitor_work));
		ret = 0;
	} else{
		pr_err("ssr_monitor_notify: not modem crash, no notify %s\n", \
			name);
		ret = 0;
	}

EXIT:
	return ret;
}
EXPORT_SYMBOL(ssr_monitor_notify);

void ssr_monitor_handle(struct work_struct *work)
{
	pr_err("modem ssr is found in ssr_monitor\n");

	if (!ssr_md->readyet)
		return;

	ssr_md->data_ready = 1;
	wake_up_interruptible(&ssr_md->ssr_monitor_wait_q);
}

void ssr_monitor_store_crashreason(char *reason)
{
	if (!reason)
		return;

	strlcpy(ssr_monitor_crash_reason, reason, strlen(reason)+1);
}
EXPORT_SYMBOL(ssr_monitor_store_crashreason);

static int ssrmonitor_setup_cdev(dev_t devno)
{
	int err;

	cdev_init(ssr_md->cdev, &ssrcharfops);

	ssr_md->cdev->owner = THIS_MODULE;
	ssr_md->cdev->ops = &ssrcharfops;

	err = cdev_add(ssr_md->cdev, devno, 1);

	if (err) {
		printk(KERN_INFO "ssrmonitor cdev registration failed !\n\n");
		return -EPERM;
	}

	ssr_md->ssrmonitor_class = class_create(THIS_MODULE, "ssr_monitor_dev");

	if (IS_ERR(ssr_md->ssrmonitor_class)) {
		printk(KERN_ERR "Error creating ssr_monitor class.\n");
		return -EPERM;
	}

	device_create(ssr_md->ssrmonitor_class, NULL, devno,
				  (void *)ssr_md, "ssr_monitor_dev");

	return 0;

}

static int __init ssr_monitor_init(void)
{
	int ret = 0;
	dev_t dev;
	int error = 0;

	pr_err("ssr_monitor initializing ..\n");
	ssr_md = kzalloc(sizeof(struct ssr_monitor_dev), GFP_KERNEL);

	init_waitqueue_head(&ssr_md->ssr_monitor_wait_q);
	ssr_md->ssr_monitor_wait_wq = \
		create_singlethread_workqueue("ssr_monitor_wait_wq");
	INIT_WORK(&(ssr_md->ssr_monitor_work), ssr_monitor_handle);

	ssr_md->num = 1;
	memcpy(ssr_md->name, "ssr_monitor_dev", sizeof("ssr_monitor_dev"));
	error = alloc_chrdev_region(&dev, ssr_md->minor,
					    ssr_md->num, ssr_md->name);
	if (!error) {
		ssr_md->major = MAJOR(dev);
		ssr_md->minor = MINOR(dev);
	} else {
		printk(KERN_ERR "ssr_monitor_init: major number not allocated\n");
		goto FAILED;
	}

	ssr_md->cdev = cdev_alloc();
	if (!ssr_md->cdev) {
		printk(KERN_ERR "ssr_monitor_init:cdev allocate failed\n");
		goto FAILED;
	}

	error = ssrmonitor_setup_cdev(dev);
	if (error)
		goto FAILED;

	ssr_md->initialized = 1;
	return ret;

FAILED:
	ret = -1;
	return ret;
}

static void ssr_monitor_exit(void)
{
	printk(KERN_ERR "ssr_monitor_exit..\n");
}

module_init(ssr_monitor_init);
module_exit(ssr_monitor_exit);
