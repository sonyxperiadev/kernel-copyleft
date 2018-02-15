/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <soc/qcom/last_logs.h>

struct dentry *debugfs_entry;
typedef int (*func_ptr)(void *, size_t, void **, uint32_t*);

static ssize_t last_logs_read(struct file *file, char __user *buf,
	size_t len, loff_t *offp)
{
	struct last_logs_data *priv_data =
			(struct last_logs_data *)file->private_data;

	return simple_read_from_buffer(buf, len, offp,
			priv_data->addr, priv_data->size);
}

static int last_logs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations last_logs_fops = {
	.owner = THIS_MODULE,
	.read = last_logs_read,
	.open = last_logs_open,
};

static int last_logs_init_resource(struct platform_device *pdev,
				struct resource *resource, func_ptr func)
{
	struct last_logs_data *priv_data;
	size_t debug_resource_size;
	int ret = -1;
	void __iomem *last_virt_iobase;
	void *last_logs_addr;

	debug_resource_size = resource->end - resource->start + 1;

	dev_info(&pdev->dev, "log driver initialized %u@%llx\n",
			(unsigned int)debug_resource_size, resource->start);
	/*
	* Map address that stores the physical location diagnostic data
	*/

	last_virt_iobase = devm_ioremap_nocache(&pdev->dev, resource->start,
			debug_resource_size);

	if (!last_virt_iobase) {
		dev_err(&pdev->dev,
			"%s: ERROR could not ioremap: start=%pr, len=%u\n",
			__func__, &resource->start,
			(unsigned int)(debug_resource_size));
		return -ENXIO;
	}

	last_logs_addr = kzalloc(debug_resource_size, GFP_KERNEL);
	if (!last_logs_addr) {
		pr_err("ERROR: %s could not allocate memory", __func__);
		iounmap(last_virt_iobase);
		return -ENOMEM;
	}

	memcpy_fromio(last_logs_addr, last_virt_iobase, debug_resource_size);
	/* clear & unmap last_logs debug memory */
	memset_io(last_virt_iobase, 0, debug_resource_size);
	iounmap(last_virt_iobase);

	priv_data = kzalloc(sizeof(struct last_logs_data), GFP_KERNEL);
	if (!priv_data) {
		pr_err("ERROR: %s could not allocate memory", __func__);
		kzfree(last_logs_addr);
		return -ENOMEM;
	}

	if (func) {
		ret = (*func)(last_logs_addr, debug_resource_size,
			(void **)&(priv_data->addr), (uint32_t *)&(priv_data->size));
		if (!ret) {
			kzfree(last_logs_addr);
			last_logs_addr = NULL;
		} else if (ret) {
			goto exit;
		}
	} else {
		priv_data->addr = last_logs_addr;
		priv_data->size = debug_resource_size;
	}

	if (debugfs_entry) {
		priv_data->debugfs_file = debugfs_create_file(resource->name,
			S_IFREG | S_IRUGO, debugfs_entry, priv_data,
				&last_logs_fops);
		if (!priv_data->debugfs_file) {
			dev_err(&pdev->dev,
				"%s: Failed to create debug file entry %s\n",
				__func__, resource->name);
			ret = -EINVAL;
			goto exit;
		}
	}

	return 0;

exit:
	kzfree(last_logs_addr);
	kzfree(priv_data);
	return ret;
}

#define STR_TZBSP_LOG "tzbsp_log"
static int last_logs_probe(struct platform_device *pdev)
{
	int i;

	if (!debugfs_entry) {
		debugfs_entry = debugfs_create_dir("last_logs", NULL);
		if (!debugfs_entry) {
			dev_err(&pdev->dev,
				"%s: Failed to create last_logs dir\n",
				__func__);
			return -EINVAL;
		}
	}

	for (i = 0; i < pdev->num_resources; i++) {
		struct resource *r = &pdev->resource[i];
		func_ptr func = NULL;

		if (!r->name) {
			dev_err(&pdev->dev, "ERROR device name is invalid");
			continue;
		}

#ifdef CONFIG_MSM_TZ_LOG
		if (!memcmp(r->name, STR_TZBSP_LOG, sizeof(STR_TZBSP_LOG)))
			func = format_tzbsp_log;
#endif
		if (last_logs_init_resource(pdev, r, func) != 0)
			dev_err(&pdev->dev, "ERROR: %s last_logs_init",
				r->name);
	}

	return 0;
}

static struct platform_driver last_logs_driver = {
	.probe		= last_logs_probe,
	.driver		= {
		.name = "rd_last_logs",
		.owner = THIS_MODULE,
	},
};

static int __init last_logs_init(void)
{
	return platform_driver_register(&last_logs_driver);
}

late_initcall(last_logs_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("last_logs driver");
MODULE_ALIAS("platform:last_logs");
MODULE_AUTHOR("Dhamodharan Nallasivam <dhamodharan.x.nallasivam@sonymobile.com>");
