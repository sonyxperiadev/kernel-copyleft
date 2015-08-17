/* Copyright (c) 2013 Sony Mobile Communications Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/sony_subsys_ramdump.h>

#define SONY_DUMP_WAIT_MSECS 10000

/**
 * struct subsys_device - subsystem device
 * @list: device
 * @name: Subsystem device name
 * @subsys_dir: debugfs directory for this device
 * @dentry: debugfs file for this device
 * @restart: debugfs file for this device
 * @data_ready: variable to inform data is ready to user space
 * @crash_reason: Crash message
 * @subsys_event_q: subsystem event wait queue of the debug files
 */
struct subsys_device {
	struct list_head list;
	char name[SUBSYS_NAME_LEN];
	struct dentry *subsys_dir;
	struct dentry *dentry;
	struct dentry *restart;
	char crash_reason[SUBSYS_CRASH_REASON_LEN];
};

struct ramdump_dev {
	int data_ready;
	wait_queue_head_t event_q;
	char buf[SUBSYS_NAME_LEN];
	struct mutex msg_lock;
	struct completion ramdump_complete;
};

static DEFINE_MUTEX(list_lock);
static LIST_HEAD(subsys_subdev_list);
static struct dentry *root_dir;
static struct ramdump_dev rdev;

static int enable_ssr_dump;
module_param(enable_ssr_dump, int, S_IRUGO | S_IWUSR);

static struct subsys_device *sony_subsys_find_device(const char *name)
{
	/*search in the device list and return the matched subsystem*/
	struct list_head *l;
	int device_found = 0;
	struct subsys_device *subsys;
	if (!name)
		return NULL;

	mutex_lock(&list_lock);
	list_for_each(l, &subsys_subdev_list) {
		subsys = list_entry(l, struct subsys_device, list);
		if (!strcmp(name, subsys->name))	{
			device_found = 1;
			break;
		}
	}
	mutex_unlock(&list_lock);

	return device_found ? (struct subsys_device *)subsys : NULL;
}

static unsigned int event_poll(struct file *filp,
					struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	if (rdev.data_ready)
		mask |= (POLLIN | POLLRDNORM);

	poll_wait(filp, &rdev.event_q, wait);
	return mask;
}

static ssize_t event_read(struct file *filp, char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	int r = strnlen(rdev.buf, SUBSYS_NAME_LEN);
	ssize_t size;

	size = simple_read_from_buffer(ubuf, cnt, ppos, rdev.buf, r);
	if (*ppos == r) {
		memset(rdev.buf, 0, sizeof(rdev.buf));
		rdev.data_ready = 0;
		complete(&rdev.ramdump_complete);
	}

	return size;
}
static const struct file_operations event_fops = {
	.open = simple_open,
	.read = event_read,
	.poll = event_poll,
	.llseek = default_llseek,
};

static ssize_t crash_reason_read(struct file *filp, char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	int r;
	char buf[SUBSYS_CRASH_REASON_LEN];
	ssize_t size;
	struct subsys_device *subsys = filp->private_data;

	r = snprintf(buf, sizeof(buf), "%s\n", subsys->crash_reason);

	size = simple_read_from_buffer(ubuf, cnt, ppos, buf, r);
	if (*ppos == r)
		memset(subsys->crash_reason, 0, sizeof(subsys->crash_reason));

	return size;
}

static ssize_t debugfs_restart_write(struct file *filp,
		const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct subsys_device *subsys = filp->private_data;
	char buf[10];
	char *cmp;

	cnt = min(cnt, sizeof(buf) - 1);
	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = '\0';
	cmp = strstrip(buf);

	if (!strcmp(cmp, "1"))
		sony_subsys_notify_crash(subsys->name, "Forced by user");
	else
		return -EINVAL;

	*ppos += cnt;
	return cnt;
}

static const struct file_operations crash_reason_fops = {
	.open = simple_open,
	.read = crash_reason_read,
	.llseek = default_llseek,
};

static const struct file_operations restart_fops = {
	.open = simple_open,
	.write  = debugfs_restart_write,
};

static int __init sony_subsys_debugfs_init(void)
{
	root_dir = debugfs_create_dir("sony_subsys", NULL);

	if (root_dir) {
		debugfs_create_file("event", S_IRUGO | S_IWUSR,
				root_dir, &rdev, &event_fops);
	}
	return !root_dir ? -ENOMEM : 0;
}

static int sony_subsys_debugfs_create(struct subsys_device *subsys)
{
	subsys->subsys_dir = debugfs_create_dir(subsys->name, root_dir);

	if (subsys->subsys_dir) {
		subsys->dentry = debugfs_create_file("crash_reason",
					S_IRUGO | S_IWUSR, subsys->subsys_dir,
					subsys, &crash_reason_fops);
		subsys->restart = debugfs_create_file("restart",
					S_IRUGO | S_IWUSR, root_dir,
					subsys, &restart_fops);
	}
	return (!subsys->dentry && !subsys->restart) ?
				(!subsys->subsys_dir ? -ENOMEM : 0) : 0;
}

/**
 * sony_subsys_notify_crash() - Wake up the dump listeners and dumps the
 * subsystem if enable_ssr_dump is zero
 * @subsys_name: Subsystem name to find out the registered device
 * @msg: Reason for the crash
 * This function wake up the respective ramdump deivce queue
 * and also updates the crash reason the in the debugfs
 */
void sony_subsys_notify_crash(const char *name, char *msg)
{
	int ret = 0;
	struct subsys_device *subsys = sony_subsys_find_device(name);
	if (subsys) {
		if (enable_ssr_dump) {
			mutex_lock(&rdev.msg_lock);

			strlcpy(rdev.buf, subsys->name, SUBSYS_NAME_LEN);
			strlcpy(subsys->crash_reason, msg,
						SUBSYS_CRASH_REASON_LEN);
			rdev.data_ready = 1;

			INIT_COMPLETION(rdev.ramdump_complete);
			/*Tell data is ready to the user space */
			wake_up(&rdev.event_q);
			/*wait here until SONY_DUMP_WAIT_MSECS for the
				user space handle the crash */
			ret = wait_for_completion_timeout(
				&rdev.ramdump_complete,
				msecs_to_jiffies(SONY_DUMP_WAIT_MSECS));
			if (!ret)
				pr_info("%s:Userspace is not ready-timed out\n",
						__func__);
			mutex_unlock(&rdev.msg_lock);

		} else
			panic("Subsystem %s crashed during SSR!", name);
	} else
		pr_err("Subsystem Restart Invalid device %s\n", name);
}
EXPORT_SYMBOL(sony_subsys_notify_crash);

/**
 * register_sony_subsys() - register the subsystem
 * @subsys_name: Subsystem name to find out the registered device
 * 1.Initlizes the subsystem debugfs wait queue
 * 2.Creates the ramdump debug files
 * 3.Creates the respective subsystem debug entry
 * 4.Adds in the subdevices list
 */
int register_sony_subsys(const char *subsys_name)
{
	int ret = 0;

	struct subsys_device *subsys;

	pr_info("Registering subsystem %s\n", subsys_name);
	subsys = kzalloc(sizeof(*subsys), GFP_KERNEL);
	if (!subsys)
		return -ENOMEM;

	strlcpy(subsys->name, subsys_name, SUBSYS_NAME_LEN);
	if (sony_subsys_debugfs_create(subsys))
		goto debugfs_fail;

	mutex_lock(&list_lock);
	list_add(&subsys->list, &subsys_subdev_list);
	mutex_unlock(&list_lock);

	return ret;

debugfs_fail:
	kfree(subsys);

	return ret;

}
EXPORT_SYMBOL(register_sony_subsys);

/**
 * unregister_sony_subsys() - unregister the subsystem
 * @subsys_name: Subsystem name to find out the registered device
 * 1.Removes the ramdump debug files
 * 2.unregisters the device from the list
 * 3.frees the respective subsystem debug entry
 */
void unregister_sony_subsys(const char *subsys_name)
{
	struct subsys_device *subsys = sony_subsys_find_device(subsys_name);
	if (subsys) {
		pr_info("unregistering subsystem %s\n", subsys->name);
		debugfs_remove(subsys->restart);
		debugfs_remove(subsys->dentry);
		debugfs_remove(subsys->subsys_dir);
		mutex_lock(&list_lock);
		list_del(&subsys->list);
		mutex_unlock(&list_lock);
		kfree(subsys);
	}
}
EXPORT_SYMBOL(unregister_sony_subsys);

/**
 * sony_subsys_ramdump_init() - Registers the root directory
 *
 */
static int __init sony_subsys_ramdump_init(void)
{
	int ret = 0;

	ret = sony_subsys_debugfs_init();
	if (ret) {
		pr_err("%s: Debugfs root dir failed\n", __func__);
		return ret;
	}

	mutex_init(&rdev.msg_lock);
	init_completion(&rdev.ramdump_complete);
	init_waitqueue_head(&rdev.event_q);

	return ret;
}

static void sony_subsys_ramdump_exit(void)
{
	debugfs_remove_recursive(root_dir);
	mutex_destroy(&list_lock);
	mutex_destroy(&rdev.msg_lock);
}

module_init(sony_subsys_ramdump_init);
module_exit(sony_subsys_ramdump_exit);

MODULE_DESCRIPTION("Sony subsystem ramdump");
MODULE_LICENSE("GPL v2");


