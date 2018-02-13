/* bcm_wlan_ramdump.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/current.h>
#include <soc/qcom/ramdump.h>
#include <linux/bcm_wlan_ramdump.h>

static void *bcm_wlan_dev;
static struct dentry *root_dir;
static struct dentry *subsys_dir;
static struct dentry *subsys_dentry;
static char crash_reason[BCM_WLAN_CRASH_REASON_LEN];
static wait_queue_head_t bcm_wlan_debugfs_q;
static int data_ready;

static int enable_ssr_dump;
module_param(enable_ssr_dump, int, S_IRUGO | S_IWUSR);

void bcm_wlan_ramdump(void *addr, int size)
{
	struct ramdump_segment segment;
	segment.v_address = addr;
	segment.size = (unsigned long)size;
	do_ramdump(bcm_wlan_dev, &segment, 1);

}
EXPORT_SYMBOL(bcm_wlan_ramdump);

void bcm_wlan_crash_reason(char *msg)
{
	int ret = 0;
	if (enable_ssr_dump) {
		strlcpy(crash_reason, msg,
					BCM_WLAN_CRASH_REASON_LEN);
		data_ready = 1;
		/*Tell data is ready to the user space */
		wake_up(&bcm_wlan_debugfs_q);
		if (!ret)
			pr_info("%s:Userspace is not ready-timed out\n",
					__func__);
	} else
		panic("Subsystem bcm_wlan crashed during SSR!");
}
EXPORT_SYMBOL(bcm_wlan_crash_reason);

static unsigned int bcm_wlan_poll(struct file *filp,
					struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	if (data_ready)
		mask |= (POLLIN | POLLRDNORM);

	poll_wait(filp, &bcm_wlan_debugfs_q, wait);
	return mask;
}

static ssize_t bcm_wlan_read(struct file *filp, char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	int r = BCM_WLAN_CRASH_REASON_LEN;
	ssize_t size;

	size = simple_read_from_buffer(ubuf, cnt, ppos, crash_reason, r);
	if (*ppos == r) {
		memset(crash_reason, 0, sizeof(crash_reason));
		data_ready = 0;
	}

	return size;
}

static const struct file_operations crash_reason_fops = {
	.open = simple_open,
	.read = bcm_wlan_read,
	.poll = bcm_wlan_poll,
	.llseek = default_llseek,
};

static int bcm_wlan_debugfs_create(void)
{
	subsys_dir = debugfs_create_dir("crash_reason", root_dir);

	if (subsys_dir) {
		subsys_dentry = debugfs_create_file("bcm_wlan",
					S_IRUGO | S_IWUSR, subsys_dir,
					NULL, &crash_reason_fops);
	}

	return !subsys_dentry ? (!subsys_dir ? -ENOMEM : 0) : 0;
}

static int __init bcm_wlan_debugfs_init(void)
{
	int ret = 0;
	root_dir = debugfs_create_dir("sony_subsys", NULL);

	if (root_dir) {
		ret = bcm_wlan_debugfs_create();
		if (ret != 0)
			pr_err("%s: Creating Debugfs file failed\n", __func__);
	}

	return !root_dir ? (!ret ? -ENOMEM : 0) : 0;
}

/**
 * bcm_wlan_ramdump_init() - Registers the root directory
 *
 */
static int __init ramdump_init(void)
{
	int ret = 0;

	bcm_wlan_dev = create_ramdump_device("bcm_wlan", NULL);
	if (!bcm_wlan_dev) {
		pr_err("%s: Unable to create a bcm_wlan ramdump device.\n",
			__func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = bcm_wlan_debugfs_init();
	if (ret != 0) {
		pr_err("%s: Creating Debugfs failed\n", __func__);
		destroy_ramdump_device(bcm_wlan_dev);
	}

	init_waitqueue_head(&bcm_wlan_debugfs_q);

	return ret;
}

static void ramdump_exit(void)
{
	destroy_ramdump_device(bcm_wlan_dev);
	return;
}

late_initcall(ramdump_init);
module_exit(ramdump_exit);

MODULE_DESCRIPTION("bcm wlan ramdump");
MODULE_LICENSE("GPL v2");


