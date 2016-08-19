/*
 * Author: Anirudh Madnurkar <anirudh.x.madnurkar@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/smem.h>

struct last_amsslog {
	u32 sig;
	u32 size;
	u8 data[0];
};

static struct device *dev;
static void __iomem *amsslog_base;
static char *amsslog_buf;
static u32 amsslog_buf_len;
static size_t amsslog_size;

#define AMSSLOG_SIG 0x48484814

static int last_amsslog_open(struct inode *inode, struct file *file)
{
	struct last_amsslog *buffer = (struct last_amsslog *)amsslog_base;
	u32 max_size = amsslog_size - (buffer->data - (uint8_t *)buffer);

	if (!amsslog_buf && !amsslog_base)
		return -EINVAL;

	memset(amsslog_buf, 0x0, amsslog_size);
	if (readl_relaxed(&buffer->size) > max_size)
		writel_relaxed(max_size, &buffer->size);

	memcpy_fromio(amsslog_buf, buffer->data, buffer->size);
	amsslog_buf_len = readl_relaxed(&buffer->size);

	return 0;
}
static ssize_t last_amsslog_read(struct file *file, char __user *buf,
				size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (!amsslog_buf || pos >= amsslog_buf_len
		|| memcmp(amsslog_buf, "ERR", 3))
		return 0;

	count = min(len, (size_t)(amsslog_buf_len - pos));
	if (copy_to_user(buf, amsslog_buf + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static int last_amsslog_release(struct inode *inode, struct file *file)
{
	memset(amsslog_buf, 0x0, amsslog_size);
	memset_io(amsslog_base, 0x0, amsslog_size);
	amsslog_buf_len = 0;
	return 0;
}

static const struct file_operations last_amsslog_fops = {
	.owner = THIS_MODULE,
	.open = last_amsslog_open,
	.read = last_amsslog_read,
	.release = last_amsslog_release,
};


static int last_amsslog_extract(void)
{
	struct last_amsslog *buffer = (struct last_amsslog *)amsslog_base;
	char *smem_errlog = NULL;
	unsigned size;

	if (!buffer)
		return -EINVAL;

	smem_errlog = smem_get_entry(SMEM_ERR_CRASH_LOG, &size, 0,
			SMEM_ANY_HOST_FLAG);

	memset_io(amsslog_base, 0x0, amsslog_size);
	if (smem_errlog && !memcmp(smem_errlog, "ERR", 3)) {
		if (size > (amsslog_size - (buffer->data - (u8 *)buffer)))
			size = (amsslog_size - (buffer->data -
						(u8 *)buffer));

		memcpy_toio(buffer->data, smem_errlog, size);
		writel_relaxed(size, &buffer->size);
		writel_relaxed(AMSSLOG_SIG, &buffer->sig);

	} else
		return -EINVAL;

	return 0;
}

static int  modem_state_cb(struct notifier_block *nb,
		unsigned long value, void *priv)
{
	int ret;

	if (value == SUBSYS_RAMDUMP_NOTIFICATION ||
			value == SUBSYS_SOC_RESET) {
		ret = last_amsslog_extract();
		if (ret < 0) {
			dev_err(dev, "Failed to extract modem error log\n");
			return NOTIFY_OK;
		}

		dev_info(dev, "logs extracted from smem\n");
	}

	return NOTIFY_OK;
}

static struct notifier_block modem_status_notifier = {
	.notifier_call = modem_state_cb,
	.priority = -INT_MAX,
};

static int last_amsslog_driver_probe(struct platform_device *pdev)
{
	struct resource *amsslog_res;
	struct proc_dir_entry *entry;
	void *subsys;
	int ret = 0;

	dev = &pdev->dev;
	amsslog_res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "amsslog");
	if (!amsslog_res || !amsslog_res->start) {
		dev_err(dev, "last_amsslog resource invalid/absent\n");
		return -ENODEV;
	}

	amsslog_size = amsslog_res->end - amsslog_res->start + 1;
	amsslog_base = (char *)ioremap(amsslog_res->start, amsslog_size);
	if (amsslog_base == NULL) {
		dev_err(dev, "failed to map last_amsslog memory\n");
		return -ENOMEM;
	}

	amsslog_buf = (char *)__get_free_pages(GFP_KERNEL,
			get_order(amsslog_size));
	if (!amsslog_buf) {
		dev_err(dev, "Failed to allocate pages of order %d\n",
			get_order(amsslog_size));
		iounmap(amsslog_base);
		return -ENOMEM;
	}

	subsys = subsys_notif_register_notifier("modem",
			&modem_status_notifier);
	if (IS_ERR(subsys)) {
		dev_err(dev, "Failed to register modem state notifier\n");
		ret = -EINVAL;
		goto out;
	}

	entry = proc_create_data("last_amsslog", S_IRUSR, NULL,
			&last_amsslog_fops, NULL);
	if (!entry) {
		dev_err(dev,
			"failed to create last_amsslog proc entry\n");
		ret = -ENOMEM;
		goto out;
	}

out:
	if (!ret)
		dev_info(dev, "Last modem error log driver probe done !!\n");
	else {
		free_pages((unsigned long)amsslog_buf, get_order(amsslog_size));
		iounmap(amsslog_base);
		amsslog_base = NULL;
	}

	return ret;
}

static struct platform_driver last_amsslog_driver = {
	.probe = last_amsslog_driver_probe,
	.driver = {
		.name   = "last_amsslog",
	},
};

static int __init last_amsslog_module_init(void)
{
	int err;
	err = platform_driver_register(&last_amsslog_driver);
	return err;
}

MODULE_AUTHOR("Anirudh Madnurkar <anirudh.x.madnurkar@sonymobile.com>");
MODULE_DESCRIPTION("modem error log");
MODULE_LICENSE("GPL V2");

module_init(last_amsslog_module_init)
