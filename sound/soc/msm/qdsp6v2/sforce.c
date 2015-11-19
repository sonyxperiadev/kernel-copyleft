/* Copyright (C) 2013 Sony Mobile Communications AB.
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
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <sound/apr_audio-v2.h>
#include "sforce.h"


static struct s_force_tuning_params s_force_coefs[SFORCE_TYPE_MAX];

void *sforce_getparam(int type)
{
	pr_debug("%s %d\n", __func__, type);
	return (void *)&s_force_coefs[type];
}

static int sforce_open(struct inode *inode, struct file *f)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static long sforce_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	struct sforce_param_data param;
	pr_debug("%s %d\n", __func__, cmd);

	switch (cmd) {
	case SFORCE_PARAM:
		if (copy_from_user(&param, (void *)arg, sizeof(param))) {
			pr_err("%s: fail to copy memory handle!\n", __func__);
			return -EFAULT;
		}
		memcpy(s_force_coefs[param.sforce_type].coefs,
			param.data, SFORCE_PARAM_SIZE);
		break;

	default:
		return -EFAULT;
	};
	return 0;
}

static int sforce_release(struct inode *inode, struct file *f)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static const struct file_operations sforce_fops = {
	.owner = THIS_MODULE,
	.open = sforce_open,
	.release = sforce_release,
	.unlocked_ioctl = sforce_ioctl,
};

struct miscdevice sforce_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "sforce",
	.fops	= &sforce_fops,
};

static int __init sforce_init(void)
{
	pr_info("%s\n", __func__);
	misc_register(&sforce_misc);
	return 0;
}

static void __exit sforce_exit(void)
{
	pr_info("%s\n", __func__);
}

module_init(sforce_init);
module_exit(sforce_exit);
