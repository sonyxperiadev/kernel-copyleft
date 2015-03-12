/* Copyright (C) 2014 Sony Mobile Communications AB.
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
#include "clearphase_hp.h"


static struct clearphase_hp_tuning_params clearphase_hp_coefs;

void *clearphase_hp_getparam()
{
	return (void *)&clearphase_hp_coefs;
}

static int clearphase_hp_open(struct inode *inode, struct file *f)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static long clearphase_hp_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	pr_debug("%s %d\n", __func__, cmd);

	switch (cmd) {
	case CLEARPHASE_HP_PARAM:
		if (copy_from_user(&clearphase_hp_coefs.coefs, (void *)arg,
			CLEARPHASE_HP_PARAM_SIZE)) {
			pr_err("%s: fail to copy memory handle!\n", __func__);
			return -EFAULT;
		}
		break;

	default:
		return -EFAULT;
	};
	return 0;
}

static int clearphase_hp_release(struct inode *inode, struct file *f)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static const struct file_operations clearphase_hp_fops = {
	.owner = THIS_MODULE,
	.open = clearphase_hp_open,
	.release = clearphase_hp_release,
	.unlocked_ioctl = clearphase_hp_ioctl,
};

struct miscdevice clearphase_hp_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "clearphase_hp",
	.fops	= &clearphase_hp_fops,
};

static int __init clearphase_hp_init(void)
{
	pr_info("%s\n", __func__);
	misc_register(&clearphase_hp_misc);
	return 0;
}

static void __exit clearphase_hp_exit(void)
{
	pr_info("%s\n", __func__);
}

module_init(clearphase_hp_init);
module_exit(clearphase_hp_exit);
