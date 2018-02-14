/*
 * Author: Anirudh Madnurkar <anirudh.x.madnurkar@sonymobile.com>
 * Author: Sandeep Kumar <sandeepkumar.x.mantrala@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2015 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/smem.h>

#define ERRLOG_SIG 0x48484814

enum {
	SUBSYS_MODEM,
	SUBSYS_ADSP,
	SUBSYS_ID_MAX
};

struct subsys_log {
	size_t log_size;
	void __iomem *log_base;
	char *log_buf;
	u32 log_buf_len;
};

struct subsys_data {
	char name[32];
	char resource_name[32];
	char lastlog_name[32];
	struct notifier_block notifier;
	struct subsys_log subsys_log;
};

struct lastlog_debug_region {
	u32 sig;
	u32 size;
	u8 data[0];
};

static struct device *dev;

static int modem_state_cb(struct notifier_block *nb,
		unsigned long value, void *priv);
static int adsp_state_cb(struct notifier_block *nb,
		unsigned long value, void *priv);

static struct subsys_data subsys_data[] = {
	[SUBSYS_MODEM] = {
		.name = "modem",
		.resource_name = "amsslog",
		.lastlog_name = "last_amsslog",
		.notifier = {
			.notifier_call = modem_state_cb,
			.priority = -INT_MAX,
		},
	},
	[SUBSYS_ADSP] = {
		.name = "adsp",
		.resource_name = "adsplog",
		.lastlog_name = "last_adsplog",
		.notifier = {
			.notifier_call = adsp_state_cb,
			.priority = -INT_MAX,
		},
	},
};

static int lastlog_open(struct inode *inode, struct file *file)
{
	struct lastlog_debug_region __iomem *buffer = NULL;
	struct subsys_data *subsys = PDE_DATA(inode);
	struct subsys_log *subsys_log;
	u32 max_size;

	if (!subsys) {
		dev_err(dev, "subsystem data is NULL.\n");
		return -EINVAL;
	}

	subsys_log = &subsys->subsys_log;
	buffer = (struct lastlog_debug_region *)subsys_log->log_base;
	if (!subsys_log->log_buf || !buffer) {
		dev_err(dev, "Log buffer not initialized.\n");
		return -EINVAL;
	}

	if (ERRLOG_SIG != readl_relaxed(&buffer->sig))
		return 0;

	max_size = subsys_log->log_size - (buffer->data - (uint8_t *)buffer);
	memset(subsys_log->log_buf, 0x0, subsys_log->log_size);
	if (readl_relaxed(&buffer->size) > max_size)
		writel_relaxed(max_size, &buffer->size);

	memcpy_fromio(subsys_log->log_buf, buffer->data, buffer->size);
	subsys_log->log_buf_len = readl_relaxed(&buffer->size);
	return 0;
}

static ssize_t lastlog_read(struct file *file, char __user *buf,
				size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;
	struct subsys_data *subsys = PDE_DATA(file_inode(file));
	struct subsys_log *subsys_log;

	if (!subsys)
		return -EINVAL;

	subsys_log = &subsys->subsys_log;
	if (pos >= subsys_log->log_buf_len)
		return 0;

	count = min(len, (size_t)(subsys_log->log_buf_len - pos));
	if (copy_to_user(buf, subsys_log->log_buf + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static int lastlog_release(struct inode *inode, struct file *file)
{
	struct subsys_data *subsys = PDE_DATA(inode);
	struct subsys_log *subsys_log;

	if (!subsys)
		return 0;

	subsys_log = &subsys->subsys_log;
	if (subsys_log->log_buf) {
		memset(subsys_log->log_buf, 0x0, subsys_log->log_size);
		subsys_log->log_buf_len = 0;
	}

	return 0;
}

static const struct file_operations lastlog_fops = {
	.owner = THIS_MODULE,
	.open = lastlog_open,
	.read = lastlog_read,
	.release = lastlog_release,
};


static int lastlog_extract(unsigned id,
		struct subsys_log *subsys_log,
		char *err_str)
{
	struct lastlog_debug_region __iomem *buffer =
		(struct lastlog_debug_region *)subsys_log->log_base;
	char *smem_errlog = NULL;
	unsigned size;

	if (!buffer) {
		dev_err(dev, "Buffer in debug region not initialized.\n");
		return -EINVAL;
	}

	memset_io(subsys_log->log_base, 0x0, subsys_log->log_size);
	smem_errlog = smem_get_entry(id, &size, 0, SMEM_ANY_HOST_FLAG);
	if (!smem_errlog) {
		dev_err(dev, "Could not read SMEM ID. NULL\n");
		return -EINVAL;
	}

	if (memcmp(smem_errlog, err_str, 3)) {
		dev_err(dev, "SMEM error log is not starting with %s\n",
						err_str);
		return -EINVAL;
	}

	if (size > (subsys_log->log_size -
				(buffer->data - (uint8_t *)buffer)))
		size = (subsys_log->log_size -
				(buffer->data - (uint8_t *)buffer));

	memcpy_toio(buffer->data, smem_errlog, size);
	writel_relaxed(size, &buffer->size);
	writel_relaxed(ERRLOG_SIG, &buffer->sig);
	return 0;
}

static int  modem_state_cb(struct notifier_block *nb,
		unsigned long value, void *priv)
{
	int ret;

	if (value == SUBSYS_RAMDUMP_NOTIFICATION ||
			value == SUBSYS_SOC_RESET) {
		dev_info(dev, "MODEM ramdump notification received.\n");
		ret = lastlog_extract(SMEM_ERR_CRASH_LOG,
				&subsys_data[SUBSYS_MODEM].subsys_log, "ERR");
		if (ret < 0) {
			dev_err(dev, "Failed to extract modem error log\n");
			return NOTIFY_OK;
		}

		dev_info(dev, "Modem logs extracted from smem\n");
	}

	return NOTIFY_OK;
}

static int  adsp_state_cb(struct notifier_block *nb,
		unsigned long value, void *priv)
{
	int ret;

	if (value == SUBSYS_RAMDUMP_NOTIFICATION || value == SUBSYS_SOC_RESET) {
		dev_info(dev, "ADSP ramdump notification received.\n");
		ret = lastlog_extract(SMEM_ERR_CRASH_LOG_ADSP,
				&subsys_data[SUBSYS_ADSP].subsys_log, "err");
		if (ret < 0) {
			dev_err(dev, "Failed to extract adsp error log\n");
			return NOTIFY_OK;
		}

		dev_info(dev, "ADSP logs extracted from smem\n");
	}

	return NOTIFY_OK;
}

static int lastlog_subsys_init(struct platform_device *pdev, int subsys_id)
{
	void *p, *handle;
	int ret;
	struct resource *res;
	struct subsys_data *subsys = NULL;
	struct subsys_log *subsys_log = NULL;

	subsys = &subsys_data[subsys_id];
	subsys_log = &subsys->subsys_log;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
				subsys->resource_name);
	if (!res || !(res->start)) {
		dev_err(dev, "%s resource invalid/absent\n",
					subsys->resource_name);
		return -ENODEV;
	}

	subsys_log->log_size = res->end - res->start + 1;
	subsys_log->log_base = (char *)ioremap(res->start,
						subsys_log->log_size);
	if (subsys_log->log_base == NULL) {
		dev_err(dev, "failed to map %s memory\n", subsys->name);
		return -ENOMEM;
	}

	subsys_log->log_buf = (char *)__get_free_pages(GFP_KERNEL,
			get_order(subsys_log->log_size));
	if (!subsys_log->log_buf) {
		dev_err(dev, "Failed to allocate pages of order %d\n",
			get_order(subsys_log->log_size));
		iounmap(subsys_log->log_base);
		return -ENOMEM;
	}

	memset(subsys_log->log_buf, 0x0, subsys_log->log_size);
	handle = subsys_notif_register_notifier(subsys->name,
				&subsys->notifier);
	if (IS_ERR(handle)) {
		dev_err(dev, "Failed to register %s notifier\n", subsys->name);
		ret = -EINVAL;
		goto out;
	}

	p = proc_create_data(subsys->lastlog_name, S_IRUSR, NULL,
				&lastlog_fops, subsys);
	if (!p) {
		subsys_notif_unregister_notifier(handle, &subsys->notifier);
		dev_err(dev, "failed to create %s proc entry\n",
					subsys->lastlog_name);
		ret = -ENOMEM;
		goto out;
	}

	return 0;
out:
	free_pages((unsigned long)subsys_log->log_buf,
			get_order(subsys_log->log_size));
	iounmap(subsys_log->log_base);
	return ret;
}

static int lastlog_subsys_driver_probe(struct platform_device *pdev)
{
	int ret, subsys_id;

	dev = &pdev->dev;
	for (subsys_id = 0; subsys_id < SUBSYS_ID_MAX; subsys_id++) {
		ret = lastlog_subsys_init(pdev, subsys_id);
		if (ret < 0) {
			dev_err(dev, "lastlogs probe failed for %s subsystem\n",
					subsys_data[subsys_id].name);
			return ret;
		}
	}

	dev_info(dev, "Last subsystem error log driver probe done.\n");
	return 0;
}

static struct platform_driver lastlog_subsys_driver = {
	.probe = lastlog_subsys_driver_probe,
	.driver = {
		.name   = "last_subsyslog",
	},
};

static int __init lastlog_subsys_module_init(void)
{
	return platform_driver_register(&lastlog_subsys_driver);
}

MODULE_AUTHOR("Anirudh Madnurkar <anirudh.x.madnurkar@sonymobile.com>");
MODULE_AUTHOR("Sandeep Mantrala <sandeepkumar.x.mantrala@sonymobile.com>");
MODULE_DESCRIPTION("subsystem error log");
MODULE_LICENSE("GPL V2");

module_init(lastlog_subsys_module_init)
