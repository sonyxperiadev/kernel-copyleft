/*
 * Author: Yoshio Yamamoto yoshio.xa.yamamoto@sonymobile.com
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

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <sound/apr_audio-v2.h>
#include "sound/sony-hweffect-params.h"


static struct s_force_tuning_params s_force_coefs;

static struct clearphase_hp_tuning_params clearphase_hp_coefs;

static struct clearphase_sp_tuning_params clearphase_sp_coefs;

static struct xloud_tuning_params xloud_coefs;

void *sony_hweffect_params_getparam(unsigned int effect_id)
{
	void *hw_params;

	pr_debug("%s: effect id %u\n", __func__, effect_id);

	hw_params = NULL;

	switch (effect_id) {
	case SFORCE_PARAM:
		hw_params = (void *)&s_force_coefs;
		break;

	case CLEARPHASE_HP_PARAM:
		hw_params = (void *)&clearphase_hp_coefs;
		break;

	case CLEARPHASE_SP_PARAM:
		hw_params = (void *)&clearphase_sp_coefs;
		break;

	case XLOUD_PARAM:
		hw_params = (void *)&xloud_coefs;
		break;

	default:
		break;
	};

	return hw_params;
}

static int sony_hweffect_params_open(struct inode *inode, struct file *f)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static long sony_hweffect_params_ioctl_shared(struct file *f,
		unsigned int cmd, void __user *arg)
{
	pr_debug("%s %d\n", __func__, cmd);

	switch (cmd) {
	case SFORCE_PARAM:
		if (copy_from_user(&s_force_coefs, (void *)arg,
			sizeof(struct sforce_param_data))) {
			pr_err("%s: fail to copy memory handle!\n", __func__);
			return -EFAULT;
		}
		break;

	case CLEARPHASE_HP_PARAM:
		if (copy_from_user(&clearphase_hp_coefs, (void *)arg,
			sizeof(struct clearphase_hp_param_data))) {
			pr_err("%s: fail to copy memory handle!\n", __func__);
			return -EFAULT;
		}
		break;

	case CLEARPHASE_SP_PARAM:
		if (copy_from_user(&clearphase_sp_coefs, (void *)arg,
			sizeof(struct clearphase_sp_param_data))) {
			pr_err("%s: fail to copy memory handle!\n", __func__);
			return -EFAULT;
		}
		break;

	case XLOUD_PARAM:
		if (copy_from_user(&xloud_coefs, (void *)arg,
			sizeof(struct xloud_param_data))) {
			pr_err("%s: fail to copy memory handle!\n", __func__);
			return -EFAULT;
		}
		break;

	default:
		return -EFAULT;
	};
	return 0;
}

static long sony_hweffect_params_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int result = 0;

	pr_debug("%s %d\n", __func__, cmd);

	result = sony_hweffect_params_ioctl_shared(f, cmd, (void __user *)arg);

	return result;
}

#ifdef CONFIG_COMPAT
#define SFORCE_PARAM_32 \
			_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 0, compat_uptr_t)
#define CLEARPHASE_HP_PARAM_32 \
			_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 1, compat_uptr_t)
#define CLEARPHASE_SP_PARAM_32 \
			_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 2, compat_uptr_t)
#define XLOUD_PARAM_32 \
			_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 3, compat_uptr_t)

static long sony_hweffect_params_compat_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
	unsigned int cmd64;
	int result = 0;

	switch (cmd) {
	case SFORCE_PARAM_32:
		cmd64 = SFORCE_PARAM;
		break;
	case CLEARPHASE_HP_PARAM_32:
		cmd64 = CLEARPHASE_HP_PARAM;
		break;
	case CLEARPHASE_SP_PARAM_32:
		cmd64 = CLEARPHASE_SP_PARAM;
		break;
	case XLOUD_PARAM_32:
		cmd64 = XLOUD_PARAM;
		break;
	default:
		result = -EINVAL;
		pr_err("%s: Invalid IOCTL, command = %d!\n",
		       __func__, cmd);
		break;
	}

	if (!result)
		result = sony_hweffect_params_ioctl_shared(f, cmd64,
					compat_ptr(arg));

	return result;
}
#else
#define sony_hweffect_params_compat_ioctl NULL
#endif

static int sony_hweffect_params_release(struct inode *inode, struct file *f)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static const struct file_operations sony_hweffect_params_fops = {
	.owner = THIS_MODULE,
	.open = sony_hweffect_params_open,
	.release = sony_hweffect_params_release,
	.unlocked_ioctl = sony_hweffect_params_ioctl,
	.compat_ioctl = sony_hweffect_params_compat_ioctl,
};

struct miscdevice sony_hweffect_params_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sony_hweffect_params",
	.fops = &sony_hweffect_params_fops,
};

static int __init sony_hweffect_params_init(void)
{
	pr_info("%s\n", __func__);
	misc_register(&sony_hweffect_params_misc);
	return 0;
}

static void __exit sony_hweffect_params_exit(void)
{
	pr_info("%s\n", __func__);
}

module_init(sony_hweffect_params_init);
module_exit(sony_hweffect_params_exit);
