/* drivers/misc/ramdumplogs.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications Japan.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <mach/subsystem_notif.h>
#include "../../arch/arm/mach-msm/smd_private.h"

struct ramdump_log {
	uint32_t    sig;
	uint32_t    size;
	uint8_t     data[0];
};

#define AMSSLOG_SIG 0x12095015

static char *amsslog;
static size_t amsslog_size;
static void *amsslog_backup_addr;
static size_t amsslog_backup_size;
static struct proc_dir_entry *amsslog_entry;

static ssize_t amsslog_read(struct file *file, char __user *buf,
				size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos >= amsslog_size)
		return 0;

	count = min(len, (size_t)(amsslog_size - pos));
	if (copy_to_user(buf, amsslog + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations amsslog_file_ops = {
	.owner = THIS_MODULE,
	.read = amsslog_read,
};

static void modem_restart_amsslog_backup(void)
{
	struct ramdump_log *buffer = amsslog_backup_addr;
	char *smem_errlog = NULL;
	size_t size;

	smem_errlog = smem_get_entry(SMEM_ERR_CRASH_LOG, &size);
	printk(KERN_INFO
			"amsslog: crash log size=%d\n", size);

	if (smem_errlog && !memcmp(smem_errlog, "ERR", 3)) {
		buffer->sig = AMSSLOG_SIG;
		buffer->size = amsslog_backup_size - 2 * sizeof(unsigned int);
		if (buffer->size > size)
			buffer->size = size;

		memcpy(buffer->data, smem_errlog, buffer->size);
	}
}

static void create_amsslog_proc_entry(struct ramdump_log *buffer, size_t size)
{
	if (amsslog == NULL)
		amsslog = kmalloc(size, GFP_KERNEL);
	if (amsslog == NULL) {
		printk(KERN_ERR
			   "amsslog: failed to allocate buffer\n");
		return;
	}

	amsslog_size = min(buffer->size, size);
	memcpy(amsslog, buffer->data, amsslog_size);

	if (amsslog_entry == NULL) {
		amsslog_entry = create_proc_entry("last_amsslog",
				S_IFREG | S_IRUGO, NULL);
	}

	if (amsslog_entry == NULL) {
		printk(KERN_ERR
			   "amsslog: failed to create proc entry\n");
		kfree(amsslog);
		amsslog = NULL;
	} else {
		amsslog_entry->proc_fops = &amsslog_file_ops;
		amsslog_entry->size = amsslog_size;
	}
}

static int ramdumplog_init(struct resource *res)
{
	struct ramdump_log *buffer;
	size_t size;
	int ret = 0;

	if (res == NULL)
		return -ENODEV;

	size = res->end - res->start + 1;
	buffer = (struct ramdump_log *)ioremap(res->start, size);
	if (buffer == NULL) {
		printk(KERN_ERR "ramdumplog: failed to map memory\n");
		return -ENOMEM;
	}

	if (buffer->sig == AMSSLOG_SIG) {
		printk(KERN_INFO
		       "amsslog:found at 0x%x\n", (unsigned int)buffer);
		create_amsslog_proc_entry(buffer, size);
	}

	memset(buffer, 0, size);
	iounmap(buffer);
	return ret;
}

static int amsslog_backup_handler(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	modem_restart_amsslog_backup();
	return NOTIFY_DONE;
}


static struct notifier_block panic_amsslog_backup = {
	.notifier_call  = amsslog_backup_handler,
};

static int modem_restart_amsslog_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
	struct ramdump_log *buffer = amsslog_backup_addr;
	size_t size = amsslog_backup_size;

	switch (code) {
	case SUBSYS_BEFORE_SHUTDOWN:
		printk(KERN_INFO
				"amsslog:%s->SUBSYS_BEFORE_SHUTDOWN\n",
				 __func__);

		modem_restart_amsslog_backup();
		create_amsslog_proc_entry(buffer, size);
		memset(buffer, 0, size);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block modem_restart_amsslog_nb = {
	.notifier_call = modem_restart_amsslog_notifier_cb,
};

static int ramdumplog_driver_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "amsslog");
	if (!res || !res->start) {
		printk(KERN_ERR "%s: amsslog resource invalid/absent\n",
				__func__);
		return -ENODEV;
	}
	ramdumplog_init(res);

	amsslog_backup_size = res->end - res->start + 1;
	amsslog_backup_addr = ioremap_nocache(res->start, amsslog_backup_size);
	if (!amsslog_backup_addr) {
		printk(KERN_ERR "failed to map amsslog_addr\n");
		return -ENOMEM;
	}

	atomic_notifier_chain_register(&panic_notifier_list,
			&panic_amsslog_backup);

	subsys_notif_register_notifier("modem", &modem_restart_amsslog_nb);

	return 0;
}

static struct platform_driver ramdumplog_driver = {
	.probe = ramdumplog_driver_probe,
	.driver		= {
		.name	= "ramdumplog",
	},
};

static int __init ramdumplog_module_init(void)
{
	int err;
	err = platform_driver_register(&ramdumplog_driver);
	return err;
}

MODULE_AUTHOR("Sony Ericsson Mobile Communications Japan");
MODULE_DESCRIPTION("ramdump crash logs");
MODULE_LICENSE("GPL V2");

module_init(ramdumplog_module_init);
