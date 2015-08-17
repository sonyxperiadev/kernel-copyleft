/* drivers/misc/felica_snfc/felica_snfc.c
 *
 * Copyright (C) 2012-2014 Sony Mobile Communications AB.
 *
 * Author: Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <crypto/hash.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/switch.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include "felica_snfc.h"

#define AUTHENTICATION_LEN 1024
#define AUTH_HASH_LEN 32
#define HASH_ALG "sha256"
#define UDELAY_CEN_WRITE 100

static DEFINE_SPINLOCK(spinlock);

#define AVP_CEN_STS_BIT 0x1
#define AVP_RFS_STS_BIT 0x2
#define AVP_URT_STS_BIT 0x4
#define AVP_ALL_STS_BIT (AVP_CEN_STS_BIT | AVP_RFS_STS_BIT | AVP_URT_STS_BIT)
#define AVP_ACTIVE_VALUE 0x7

struct felica_dev {
	struct felica_data *felica_data;
	struct device *dev;
	struct device sysfs_dev;
	bool irq_shutdown;

	/* FeliCa */
	struct miscdevice device_cen;
	struct miscdevice device_pon;
	struct miscdevice device_rfs;
	struct miscdevice device_rws;
	struct switch_dev swdev;

	unsigned int st_usbcon;
	unsigned int st_airplane;
	unsigned int ta_rwusb;

	/* NFC */
	struct miscdevice device_nfc_cen;
	struct miscdevice device_hsel;
	struct miscdevice device_intu_poll;
	struct miscdevice device_available_poll;
	unsigned int available_poll_nfc;
	wait_queue_head_t available_poll_wait;
	int intu_curr_val;
	wait_queue_head_t intu_wait_queue;
	int hsel_ref;
	bool initialized_rfs_val;
};

enum nfc_avp_type {
	AVP_CEN,
	AVP_RFS,
	AVP_URT,
};

static ssize_t show_usbcon(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct felica_dev *d = dev_get_drvdata(dev);
	unsigned int value;
	unsigned long flags;

	spin_lock_irqsave(&spinlock, flags);
	value = d->st_usbcon;
	spin_unlock_irqrestore(&spinlock, flags);

	return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t set_usbcon(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct felica_dev *d = dev_get_drvdata(dev);
	unsigned long value;
	unsigned long flags;

	if (kstrtoul(buf, 10, &value)) {
		dev_err(dev, "%s: Failed to parse integer\n", __func__);
		return -EINVAL;
	}
	spin_lock_irqsave(&spinlock, flags);
	d->st_usbcon = value;
	spin_unlock_irqrestore(&spinlock, flags);
	dev_dbg(dev, "%s: %u\n", __func__, d->st_usbcon);

	return count;
}

static ssize_t show_airplane(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct felica_dev *d = dev_get_drvdata(dev);
	unsigned int value;
	unsigned long flags;

	spin_lock_irqsave(&spinlock, flags);
	value = d->st_airplane;
	spin_unlock_irqrestore(&spinlock, flags);

	return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t set_airplane(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct felica_dev *d = dev_get_drvdata(dev);
	unsigned long value;
	unsigned long flags;

	if (kstrtoul(buf, 10, &value)) {
		dev_err(dev, "%s: Failed to parse integer\n", __func__);
		return -EINVAL;
	}
	spin_lock_irqsave(&spinlock, flags);
	d->st_airplane = value;
	spin_unlock_irqrestore(&spinlock, flags);
	dev_dbg(dev, "%s: %u\n", __func__, d->st_airplane);

	return count;
}

static ssize_t show_rwusb(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct felica_dev *d = dev_get_drvdata(dev);
	unsigned int value;
	unsigned long flags;

	spin_lock_irqsave(&spinlock, flags);
	value = d->ta_rwusb;
	spin_unlock_irqrestore(&spinlock, flags);

	return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t set_rwusb(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct felica_dev *d = dev_get_drvdata(dev);
	unsigned long value;
	unsigned long flags;

	if (kstrtoul(buf, 10, &value)) {
		dev_err(dev, "%s: Failed to parse integer\n", __func__);
		return -EINVAL;
	}
	spin_lock_irqsave(&spinlock, flags);
	d->ta_rwusb = value;
	dev_dbg(dev, "%s: %u\n", __func__, d->ta_rwusb);
	spin_unlock_irqrestore(&spinlock, flags);

	return count;
}

static ssize_t nfc_ldo_store(
			struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int ret;
	char val = *buf;
	struct felica_dev *d = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (!d->felica_data->flldo->ldo_write) {
		dev_err(dev, "%s: Error. No device.\n", __func__);
		ret = -ENODEV;
		goto err_out;
	}

	if (0 != val && 1 != val) {
		dev_err(dev, "%s: Error. Invalid val(%d).\n", __func__, val);
		ret = -EINVAL;
		goto err_out;
	}

	d->felica_data->flldo->ldo_write(val, d->felica_data->user);

	return count;

err_out:
	return ret;
}

static struct device_attribute felica_attrs[] = {
	__ATTR(st_usbcon, S_IWUSR, show_usbcon, set_usbcon),
	__ATTR(st_airplane, S_IWUSR, show_airplane, set_airplane),
	__ATTR(ta_rwusb, S_IWUSR, show_rwusb, set_rwusb),
};

static struct device_attribute nfc_attrs[] = {
	__ATTR(nfc_ldo, S_IWUSR, NULL, nfc_ldo_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i, j;
	struct felica_dev *d = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(felica_attrs); i++)
		if (device_create_file(dev, felica_attrs + i))
			goto err_felica;

	if (FELICA_SNFC == d->felica_data->type)
		for (j = 0; j < ARRAY_SIZE(nfc_attrs); j++)
			if (device_create_file(dev, nfc_attrs + j))
				goto err_nfc;

	return 0;

err_nfc:
	if (FELICA_SNFC == d->felica_data->type)
		for (j = j - 1; j >= 0; j--)
			device_remove_file(dev, nfc_attrs + j);
err_felica:
	for (i = i - 1; i >= 0; i--)
		device_remove_file(dev, felica_attrs + i);

	dev_err(dev, "Unable to create sysfs interfaces\n");

	return -EIO;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	struct felica_dev *d = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(felica_attrs); i++)
		device_remove_file(dev, felica_attrs + i);

	if (FELICA_SNFC == d->felica_data->type)
		for (i = 0; i < ARRAY_SIZE(nfc_attrs); i++)
			device_remove_file(dev, nfc_attrs + i);
}

struct sdesc {
	struct shash_desc shash;
	char ctx[];
};

/** @ Authentication hash value*/
static const u8 auth_hash[AUTH_HASH_LEN] = {
	0xbf, 0x19, 0x2e, 0xaf, 0x67, 0x0c, 0x90, 0xc8,
	0x12, 0x5f, 0xbb, 0xbc, 0x9e, 0x17, 0x23, 0x39,
	0x69, 0x4a, 0xb7, 0xec, 0x3a, 0x9f, 0x91, 0x57,
	0x80, 0xec, 0xb3, 0xf5, 0xfe, 0x28, 0x8f, 0x33
};

static int calc_hash(const u8 *src, int src_len, u8 *out, struct device *dev)
{
	struct crypto_shash *shash;
	struct sdesc *desc;
	int size;
	int ret = -EFAULT;

	shash = crypto_alloc_shash(HASH_ALG, 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(shash)) {
		dev_err(dev, "%s: Error. crypto_alloc_shash.\n", __func__);
		goto err_shash;
	}

	size = sizeof(struct shash_desc) + crypto_shash_descsize(shash);
	desc = kmalloc(size, GFP_KERNEL);
	if (!desc) {
		dev_err(dev, "%s: Error. No enough mem for Desc.\n", __func__);
		ret = -ENOMEM;
		goto err_desc;
	}

	desc->shash.tfm = shash;
	desc->shash.flags = 0x00;

	if (crypto_shash_digest(&desc->shash, src, src_len, out)) {
		dev_err(dev, "%s: Error. generate hash.\n", __func__);
		goto err_generate;
	}

	ret = 0;

err_generate:
	kfree(desc);
err_desc:
	crypto_free_shash(shash);
err_shash:
	return ret;
}

static int cen_open(struct inode *inode, struct file *file,
			struct felica_dev *d, struct device *dev)
{
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	/* Open CEN */
	ret = d->felica_data->flcen->cen_init(d->felica_data->user);

	if (ret) {
		dev_err(dev, "%s: Cannot enable CEN.\n", __func__);
		return ret;
	}

	dev_dbg(dev, "Successfully enabled CEN\n");

	return 0;
}

static int cen_release(struct inode *inode, struct file *file,
			struct felica_dev *d, struct device *dev)
{
	int ret = 0;

	ret = d->felica_data->flcen->cen_release(d->felica_data->user);

	if (ret) {
		dev_err(dev, "%s: Cannot disable CEN.\n", __func__);
		return ret;
	}

	dev_dbg(dev, "%s\n", __func__);

	return 0;
}

static ssize_t cen_read(struct file *file, char __user *buf,
				size_t count, loff_t *offset,
				struct felica_dev *d, struct device *dev)
{
	int ret;
	char kbuf;

	dev_dbg(dev, "%s\n", __func__);

	/* Read CEN value */
	ret = d->felica_data->flcen->cen_read(&kbuf, d->felica_data->user);
	if (ret) {
		dev_err(dev, "%s: Error. CEN access failed.\n", __func__);
		return -EIO;
	}

	/* Copy CEN value to user space */
	ret = copy_to_user(buf, &kbuf, sizeof(kbuf));
	if (ret) {
		dev_err(dev, "%s: Error. copy_to_user failure.\n", __func__);
		return -EFAULT;
	}

	/* 1 byte read */
	return sizeof(kbuf);
}

static void nfc_available_poll_set_status(struct felica_dev *dev,
			bool status, enum nfc_avp_type type);

static ssize_t cen_write(struct file *file, const char __user *buf,
				size_t count, loff_t *offset,
				struct felica_dev *d, struct device *dev)
{
	int ret;
	char kbuf;
	u8 *src;
	u8 hash[AUTH_HASH_LEN];

	dev_dbg(dev, "%s\n", __func__);

	if ((AUTHENTICATION_LEN+1) != count || NULL == buf) {
		dev_err(dev, "%s: Error. Invalid arg @CEN write.\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	/* Carry out user authentication */
	src = kmalloc(AUTHENTICATION_LEN, GFP_KERNEL);
	if (!src) {
		dev_err(dev, "%s: Error. No enough mem for Auth.\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
	ret = copy_from_user(src, buf, AUTHENTICATION_LEN);
	if (ret) {
		dev_err(dev, "%s: Error. copy_from_user failure.\n", __func__);
		ret = -EFAULT;
		goto end_process;
	}

	if (calc_hash(src, AUTHENTICATION_LEN, hash, dev)) {
		dev_err(dev, "%s: Error. calc hash digest failure.\n",
			__func__);
		ret = -EACCES;
		goto end_process;
	}

	if (memcmp(auth_hash, hash, AUTH_HASH_LEN)) {
		dev_err(dev, "%s: Error. invalid authentication.\n", __func__);
		ret = -EACCES;
		goto end_process;
	}

	/* Copy value from user space */
	ret = copy_from_user(&kbuf, &buf[AUTHENTICATION_LEN], sizeof(kbuf));
	if (ret) {
		dev_err(dev, "%s: Error. copy_from_user failure.\n", __func__);
		ret = -EFAULT;
		goto end_process;
	}

	ret = d->felica_data->flcen->cen_write(kbuf, d->felica_data->user);
	if (ret) {
		dev_err(dev, "%s: Error. Cannot write CEN.\n", __func__);
		goto end_process;
	}

	/* usec delay*/
	udelay(UDELAY_CEN_WRITE);

	ret = AUTHENTICATION_LEN+1;

	if (FELICA_SNFC == d->felica_data->type)
		nfc_available_poll_set_status(d, kbuf, AVP_CEN);

end_process:
	kfree(src);
exit:
	return ret;
}

static int felica_cen_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_cen);

	dev_dbg(d->device_cen.this_device, "%s\n", __func__);

	return cen_open(inode, file, d, d->device_cen.this_device);
}

static int felica_cen_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_cen);

	dev_dbg(d->device_cen.this_device, "%s\n", __func__);

	return cen_release(inode, file, d, d->device_cen.this_device);
}

static ssize_t felica_cen_read(struct file *file, char __user *buf,
					size_t count, loff_t *offset)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_cen);

	dev_dbg(d->device_cen.this_device, "%s\n", __func__);

	return cen_read(file, buf, count, offset, d, d->device_cen.this_device);
}

static ssize_t felica_cen_write(struct file *file, const char __user *buf,
					size_t count, loff_t *offset)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_cen);

	dev_dbg(d->device_cen.this_device, "%s\n", __func__);

	return cen_write(file, buf, count, offset,
					d, d->device_cen.this_device);
}

static const struct file_operations felica_cen_fops = {
	.owner		= THIS_MODULE,
	.read		= felica_cen_read,
	.write		= felica_cen_write,
	.open		= felica_cen_open,
	.release	= felica_cen_release,
	.fsync		= NULL,
};

static int felica_cen_probe_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	if (!d->felica_data->flcen->cen_init ||
		!d->felica_data->flcen->cen_read ||
		!d->felica_data->flcen->cen_write ||
		!d->felica_data->flcen->cen_release) {
		dev_err(d->dev, "%s: Error. Invalid operation.\n", __func__);
		return -EINVAL;
	}

	/* Create CEN character device (/dev/felica_cen) */
	if (misc_register(&d->device_cen)) {
		dev_err(d->dev, "%s: Error. Cannot register CEN.\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static void felica_cen_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_cen);
}

static irqreturn_t felica_int_irq(int irq, void *user)
{
	int state;
	struct felica_dev *d = user;

	/* Read INT GPIO */
	state = d->felica_data->flint->int_read(d->felica_data->user);

	if (0 == state || 1 == state) {
		/* Update value of the switch device */
		switch_set_state(&d->swdev, state);
		dev_dbg(d->dev, "INT state = %d\n", state);
	} else {
		dev_err(d->dev, "%s: Error. Cannot read INT GPIO.\n", __func__);
	}

	return IRQ_HANDLED;
}

static int felica_int_probe_func(struct felica_dev *d)
{
	int ret;

	dev_dbg(d->dev, "%s\n", __func__);

	if (!d->felica_data->flint->int_init ||
		!d->felica_data->flint->int_read ||
		!d->felica_data->flint->int_release) {
		dev_err(d->dev, "%s: Error. Invalid operation.\n", __func__);
		ret = -EINVAL;
		goto err_invalid;
	}

	/* Init INT GPIO */
	ret = d->felica_data->flint->int_init(d->felica_data->user);
	if (ret) {
		dev_err(d->dev, "%s: Error. INT GPIO init failed.\n", __func__);
		ret = -ENODEV;
		goto err_request_int_gpio;
	}

	/* Create INT switch device (felica_push) */
	d->swdev.name = "felica_push";
	if (switch_dev_register(&d->swdev)) {
		dev_err(d->dev, "%s: Error. Cannot create switch dev\n",
					__func__);
		ret = -ENOMEM;
		goto err_create_switch_dev;
	}

	/* Set initial state */
	switch_set_state(&d->swdev, 0);

	return 0;

err_create_switch_dev:
	d->felica_data->flint->int_release(d->felica_data->user);
err_request_int_gpio:
err_invalid:
	return ret;
}

static int felica_int_irq_start(struct felica_dev *d)
{
	int ret, state;

	/* Read INT GPIO */
	state = d->felica_data->flint->int_read(d->felica_data->user);
	if (0 > state) {
		dev_err(d->dev, "%s: Error. Read INT.\n", __func__);
		ret = -EIO;
		goto err_read_state;
	}
	/* Set state of the device */
	switch_set_state(&d->swdev, state);

	/* Request IRQ for INT GPIO */
	ret = request_threaded_irq(d->felica_data->irq_int, NULL,
		felica_int_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"felica_irq", d);
	if (ret) {
		dev_err(d->dev, "%s: Error. Request IRQ failed.\n", __func__);
		ret = -EIO;
		goto err_request_int_irq;
	}

	/* Enable IRQ wake */
	ret = enable_irq_wake(d->felica_data->irq_int);
	if (ret) {
		dev_err(d->dev, "%s: Error. Enabling IRQ wake failed.\n",
					__func__);
		ret = -EIO;
		goto err_enable_irq_wake;
	}

	return 0;

err_enable_irq_wake:
	free_irq(d->felica_data->irq_int, d);
err_request_int_irq:
err_read_state:
	return ret;
}

static void felica_int_irq_shutdown(struct felica_dev *d)
{
	disable_irq_wake(d->felica_data->irq_int);
	free_irq(d->felica_data->irq_int, d);
}

static void felica_int_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	switch_dev_unregister(&d->swdev);
	if (!d->irq_shutdown)
		felica_int_irq_shutdown(d);
	d->felica_data->flint->int_release(d->felica_data->user);
}

static int felica_pon_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_pon);

	dev_dbg(d->device_pon.this_device, "%s\n", __func__);

	return 0;
}

static int felica_pon_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_pon);

	dev_dbg(d->device_pon.this_device, "%s\n", __func__);

	return 0;
}

static ssize_t felica_pon_write(struct file *file, const char __user *buf,
					size_t count, loff_t *offset)
{
	int ret;
	char kbuf;
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_pon);

	dev_dbg(d->device_pon.this_device, "%s\n", __func__);

	if (1 != count || NULL == buf) {
		dev_err(d->device_pon.this_device,
				"%s: Error. Invalid arg @PON write.\n",
					__func__);
		ret = -EINVAL;
		goto exit;
	}

	/* Copy value from user space */
	if (copy_from_user(&kbuf, buf, sizeof(kbuf))) {
		dev_err(d->device_pon.this_device,
			"%s: Error. copy_from_user failure.\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	if (1 == kbuf) {
		/* Write High to PON GPIO */
		d->felica_data->flpon->pon_write(1, d->felica_data->user);
	} else if (0 == kbuf) {
		/* Write LOW to PON GPIO */
		d->felica_data->flpon->pon_write(0, d->felica_data->user);
	} else {
		dev_err(d->device_pon.this_device,
			"%s: Error. Invalid val @PON write.\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	/* 1 byte write */
	ret = sizeof(kbuf);

exit:
	return ret;
}

static const struct file_operations felica_pon_fops = {
	.owner		= THIS_MODULE,
	.read		= NULL,
	.write		= felica_pon_write,
	.open		= felica_pon_open,
	.release	= felica_pon_release,
	.fsync		= NULL,
};

static int felica_pon_probe_func(struct felica_dev *d)
{
	int ret;

	dev_dbg(d->dev, "%s\n", __func__);

	if (!d->felica_data->flpon->pon_init ||
		!d->felica_data->flpon->pon_write ||
		!d->felica_data->flpon->pon_release) {
		dev_err(d->dev, "%s: Error. Invalid operation.\n", __func__);
		ret = -EINVAL;
		goto err_invalid;
	}

	/* Init PON GPIO */
	ret = d->felica_data->flpon->pon_init(d->felica_data->user);
	if (ret) {
		dev_err(d->dev, "%s: Error. PON GPIO init failed.\n", __func__);
		ret = -ENODEV;
		goto err_request_pon_gpio;
	}

	/* Create PON character device (/dev/felica_pon) */
	if (misc_register(&d->device_pon)) {
		dev_err(d->dev, "%s: Error. Cannot register PON.\n",
				__func__);
		ret = -ENODEV;
		goto err_create_pon_dev;
	}

	return 0;

err_create_pon_dev:
	d->felica_data->flpon->pon_release(d->felica_data->user);
err_request_pon_gpio:
err_invalid:
	return ret;
}

static void felica_pon_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_pon);
	d->felica_data->flpon->pon_release(d->felica_data->user);
}

static int felica_rfs_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_rfs);

	dev_dbg(d->device_rfs.this_device, "%s\n", __func__);

	return 0;
}

static int felica_rfs_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_rfs);

	dev_dbg(d->device_rfs.this_device, "%s\n", __func__);

	return 0;
}

static ssize_t felica_rfs_read(struct file *file, char __user *buf,
					size_t count, loff_t *offset)
{
	char kbuf;
	int gpio_val;
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_rfs);

	dev_dbg(d->device_rfs.this_device, "%s\n", __func__);

	/* Read RFS GPIO value */
	gpio_val = d->felica_data->flrfs->rfs_read(d->felica_data->user);

	if (0 == gpio_val) {
		kbuf = 1;
	} else if (1 == gpio_val) {
		kbuf = 0;
	} else {
		dev_err(d->device_rfs.this_device,
				"%s: Error. Invalid GPIO value @RFS read.\n",
					__func__);
		return -EIO;
	}

	/* Copy the value to user space */
	if (copy_to_user(buf, &kbuf, sizeof(kbuf))) {
		dev_err(d->device_rfs.this_device,
				"%s: Error. copy_to_user failure.\n", __func__);
		return -EFAULT;
	}

	/* 1 byte read */
	return sizeof(kbuf);
}

static const struct file_operations felica_rfs_fops = {
	.owner		= THIS_MODULE,
	.read		= felica_rfs_read,
	.write		= NULL,
	.open		= felica_rfs_open,
	.release	= felica_rfs_release,
	.fsync		= NULL,
};

static irqreturn_t nfc_rfs_irq(int irq, void *dev)
{
	int gpio_val;
	struct felica_dev *d = dev;

	dev_dbg(d->dev, "%s\n", __func__);

	gpio_val = d->felica_data->flrfs->rfs_read(d->felica_data->user);

	nfc_available_poll_set_status(d, gpio_val, AVP_RFS);

	return IRQ_HANDLED;
}

static int felica_rfs_probe_func(struct felica_dev *d)
{
	int ret;

	dev_dbg(d->dev, "%s\n", __func__);

	if (!d->felica_data->flrfs->rfs_init ||
		!d->felica_data->flrfs->rfs_read ||
		!d->felica_data->flrfs->rfs_release) {
		dev_err(d->dev, "%s: Error. Invalid operation.\n", __func__);
		ret = -EINVAL;
		goto err_invalid;
	}

	/* Init RFS GPIO */
	ret = d->felica_data->flrfs->rfs_init(d->felica_data->user);
	if (ret) {
		dev_err(d->dev, "%s: Error. RFS GPIO init failed.\n",
				__func__);
		ret = -ENODEV;
		goto err_request_rfs_gpio;
	}

	/* Create RFS character device (/dev/felica_rfs) */
	if (misc_register(&d->device_rfs)) {
		dev_err(d->dev, "%s: Error. Cannot register RFS.\n",
				__func__);
		ret = -ENODEV;
		goto err_create_rfs_dev;
	}

	return 0;

err_create_rfs_dev:
	d->felica_data->flrfs->rfs_release(d->felica_data->user);
err_request_rfs_gpio:
err_invalid:
	return ret;
}

static int felica_rfs_irq_start(struct felica_dev *d)
{
	int ret;
	int gpio_val;
	unsigned long flags;

	if (FELICA_SNFC == d->felica_data->type) {
		d->initialized_rfs_val = false;

		ret = request_threaded_irq(d->felica_data->irq_rfs, NULL,
			nfc_rfs_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT,
			"nfc_rfs_irq", d);
		if (ret) {
			dev_err(d->dev,
				"%s: Error. Request IRQ failed.ret=%d\n",
				__func__, ret);
			goto err_request_irq;
		}

		ret = enable_irq_wake(d->felica_data->irq_rfs);
		if (ret) {
			dev_err(d->dev,
				"%s: Error. Enabling IRQ wake failed.ret=%d\n",
				__func__, ret);
			goto err_enable_irq_wake;
		}

		gpio_val = d->felica_data->flrfs->rfs_read(
			d->felica_data->user);
		if (0 > gpio_val) {
			ret = gpio_val;
			dev_err(d->dev,
				"%s: Error. Invalid GPIO value RFS read.\n",
				__func__);
			goto err_rfs_read;
		}
		spin_lock_irqsave(&spinlock, flags);
		if (!d->initialized_rfs_val) {
			d->available_poll_nfc &=
				(AVP_CEN_STS_BIT | AVP_URT_STS_BIT);
			d->available_poll_nfc |= (gpio_val & 0x1) << 1;
			d->initialized_rfs_val = true;
		}
		spin_unlock_irqrestore(&spinlock, flags);
	}

	return 0;

err_rfs_read:
	if (FELICA_SNFC == d->felica_data->type)
		disable_irq_wake(d->felica_data->irq_rfs);
err_enable_irq_wake:
	if (FELICA_SNFC == d->felica_data->type)
		free_irq(d->felica_data->irq_rfs, d);
err_request_irq:
	return ret;
}

static void felica_rfs_irq_shutdown(struct felica_dev *d)
{
	if (FELICA_SNFC == d->felica_data->type) {
		disable_irq_wake(d->felica_data->irq_rfs);
		free_irq(d->felica_data->irq_rfs, d);
	}
}

static void felica_rfs_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_rfs);
	if (!d->irq_shutdown)
		felica_rfs_irq_shutdown(d);
	d->felica_data->flrfs->rfs_release(d->felica_data->user);
}

static int felica_rws_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_rws);

	dev_dbg(d->device_rws.this_device, "%s\n", __func__);

	return 0;
}

static int felica_rws_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_rws);

	dev_dbg(d->device_rws.this_device, "%s\n", __func__);

	return 0;
}

static ssize_t felica_rws_read(struct file *file, char __user *buf,
					size_t count, loff_t *offset)
{
	int ret;
	char kbuf;
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_rws);
	unsigned long flags;

	dev_dbg(d->device_rws.this_device, "%s\n", __func__);

	dev_dbg(d->device_rws.this_device,
			"st_usbcon = 0x%x\n", d->st_usbcon);
	dev_dbg(d->device_rws.this_device,
			"st_airplane = 0x%x\n", d->st_airplane);
	dev_dbg(d->device_rws.this_device,
			"ta_rwusb = 0x%x\n", d->ta_rwusb);

	/* Check R/W module parameters */
	spin_lock_irqsave(&spinlock, flags);
	if (0 != d->st_usbcon && 1 != d->st_usbcon) {
		spin_unlock_irqrestore(&spinlock, flags);
		dev_err(d->device_rws.this_device,
				"%s: Error. Invalid st_usbcon value.\n",
					__func__);
		ret = -EIO;
		goto exit;
	}
	if (0 != d->st_airplane && 1 != d->st_airplane) {
		spin_unlock_irqrestore(&spinlock, flags);
		dev_err(d->device_rws.this_device,
				"%s: Error. Invalid st_airplane value.\n",
					__func__);
		ret = -EIO;
		goto exit;
	}
	if (0 != d->ta_rwusb && 1 != d->ta_rwusb) {
		spin_unlock_irqrestore(&spinlock, flags);
		dev_err(d->device_rws.this_device,
				"%s: Error. Invalid ta_rwusb value.\n",
					__func__);
		ret = -EIO;
		goto exit;
	}

	/* Determine RWS value */
	if ((1 == d->st_usbcon || 1 == d->st_airplane)
		&& 0 == d->ta_rwusb)
		kbuf = 1;
	else
		kbuf = 0;

	spin_unlock_irqrestore(&spinlock, flags);

	dev_dbg(d->device_rws.this_device, "kbuf = 0x%x\n", kbuf);

	/* Copy the value to user space */
	if (copy_to_user(buf, &kbuf, sizeof(kbuf))) {
		dev_err(d->device_rws.this_device,
				"%s: Error. copy_to_user failure.\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	/* 1 byte read */
	ret = sizeof(kbuf);
exit:
	return ret;
}

static const struct file_operations felica_rws_fops = {
	.owner		= THIS_MODULE,
	.read		= felica_rws_read,
	.write		= NULL,
	.open		= felica_rws_open,
	.release	= felica_rws_release,
};

static int felica_rws_probe_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	/* Create RWS character device (/dev/felica_rws) */
	if (misc_register(&d->device_rws)) {
		dev_err(d->dev, "%s: Error. Cannot register RWS.\n",
				__func__);
		return -ENODEV;
	}

	return 0;
}

static void felica_rws_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_rws);
}

static int nfc_hsel_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_hsel);

	dev_dbg(d->device_hsel.this_device, "%s(%d)\n", __func__, d->hsel_ref);

	d->hsel_ref++;

	return 0;
}

static int nfc_hsel_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_hsel);

	dev_dbg(d->device_hsel.this_device, "%s(%d)\n", __func__, d->hsel_ref);

	d->hsel_ref--;
	if (0 >= d->hsel_ref) {
		d->hsel_ref = 0;
		d->felica_data->flhsel->hsel_write(0, d->felica_data->user);
	}

	return 0;
}

static ssize_t nfc_hsel_write(struct file *file, const char __user *buf,
					size_t count, loff_t *offset)
{
	int ret;
	char kbuf;
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev, device_hsel);

	dev_dbg(d->device_hsel.this_device, "%s\n", __func__);

	if (1 != count || NULL == buf) {
		dev_err(d->device_hsel.this_device,
				"%s: Error. Invalid arg @HSEL write.\n",
				__func__);
		ret = -EINVAL;
		goto exit;
	}

	if (copy_from_user(&kbuf, buf, sizeof(kbuf))) {
		dev_err(d->device_hsel.this_device,
			"%s: Error. copy_from_user failure.\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	if (1 == kbuf) {
		d->felica_data->flhsel->hsel_write(1, d->felica_data->user);
	} else if (0 == kbuf) {
		d->felica_data->flhsel->hsel_write(0, d->felica_data->user);
	} else {
		dev_err(d->device_hsel.this_device,
			"%s: Error. Invalid val @HSEL write.\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	/* 1 byte write */
	ret = sizeof(kbuf);

exit:
	return ret;
}

static const struct file_operations nfc_hsel_fops = {
	.owner		= THIS_MODULE,
	.read		= NULL,
	.write		= nfc_hsel_write,
	.open		= nfc_hsel_open,
	.release	= nfc_hsel_release,
	.fsync		= NULL,
};

static int nfc_hsel_probe_func(struct felica_dev *d)
{
	int ret;

	dev_dbg(d->dev, "%s\n", __func__);

	if (!d->felica_data->flhsel->hsel_init ||
		!d->felica_data->flhsel->hsel_write ||
		!d->felica_data->flhsel->hsel_release) {
		dev_err(d->dev, "%s: Error. Invalid operation.\n", __func__);
		ret = -EINVAL;
		goto err_invalid;
	}

	ret = d->felica_data->flhsel->hsel_init(d->felica_data->user);
	if (ret) {
		dev_err(d->dev, "%s: Error. hsel_init failed.\n", __func__);
		ret = -ENODEV;
		goto err_init;
	}
	if (misc_register(&d->device_hsel)) {
		dev_err(d->dev, "%s: Error. Cannot register NFC HSEL.\n",
				__func__);
		ret = -ENODEV;
		goto err_misc_register;
	}
	d->hsel_ref = 0;

	return 0;

err_misc_register:
	d->felica_data->flhsel->hsel_release(d->felica_data->user);
err_init:
err_invalid:
	return ret;
}

static void nfc_hsel_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_hsel);
	d->felica_data->flhsel->hsel_release(d->felica_data->user);
}

static int nfc_intu_poll_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
						device_intu_poll);

	dev_dbg(d->device_intu_poll.this_device, "%s\n", __func__);

	return 0;
}

static int nfc_intu_poll_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
						device_intu_poll);

	dev_dbg(d->device_intu_poll.this_device, "%s\n", __func__);

	return 0;
}

static ssize_t nfc_intu_poll_read(struct file *file, char __user *buf,
					size_t count, loff_t *offset)
{
	char kbuf;
	int prev_val;
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
						device_intu_poll);
	int ret;

	dev_dbg(d->device_intu_poll.this_device, "%s\n", __func__);

	prev_val = d->intu_curr_val;

	dev_dbg(d->device_intu_poll.this_device,
		"%s: Wait INTU Interrupt.prev_val=%d,curr_val=%d\n",
			__func__, prev_val, d->intu_curr_val);
	ret = wait_event_interruptible(d->intu_wait_queue,
			prev_val != d->intu_curr_val);
	dev_dbg(d->device_intu_poll.this_device,
		"%s: WakeUp INTU Interrupt.prev_val=%d,curr_val=%d\n",
			__func__, prev_val, d->intu_curr_val);
	if (ret) {
		dev_warn(d->device_intu_poll.this_device,
			"%s:"
			" Other factor wakeup of INTU IRQ."
			" ret=%d,prev_val=%d,curr_val=%d\n",
			__func__, ret, prev_val, d->intu_curr_val);
		return -ERESTARTSYS;
	}

	if ((0 == prev_val) && (1 == d->intu_curr_val)) {
		kbuf = 1;
	} else if ((1 == prev_val) && (0 == d->intu_curr_val)) {
		kbuf = 0;
	} else {
		dev_err(d->device_intu_poll.this_device,
				"%s: Error. Invalid GPIO value @INTU read.\n",
					__func__);
		return -EIO;
	}

	dev_dbg(d->device_intu_poll.this_device,
		"nfc intu polled kbuf=%d, prev_val=%d,curr_val=%d\n",
			kbuf, prev_val, d->intu_curr_val);

	if (copy_to_user(buf, &kbuf, sizeof(kbuf))) {
		dev_err(d->device_intu_poll.this_device,
				"%s: Error. copy_to_user failure.\n", __func__);
		return -EFAULT;
	}

	return sizeof(kbuf);
}

static const struct file_operations nfc_intu_poll_fops = {
	.owner		= THIS_MODULE,
	.read		= nfc_intu_poll_read,
	.write		= NULL,
	.open		= nfc_intu_poll_open,
	.release	= nfc_intu_poll_release,
	.fsync		= NULL,
};

static irqreturn_t nfc_intu_poll_irq(int irq, void *dev)
{
	struct felica_dev *d = dev;

	dev_dbg(d->dev, "%s\n", __func__);

	d->intu_curr_val =
		d->felica_data->flintu->intu_read(d->felica_data->user);
	dev_dbg(d->dev, "%s INTU=%d\n", __func__, d->intu_curr_val);

	wake_up_interruptible(&d->intu_wait_queue);

	return IRQ_HANDLED;
}

static int nfc_intu_poll_probe_func(struct felica_dev *d)
{
	int ret;

	dev_dbg(d->dev, "%s\n", __func__);

	if (!d->felica_data->flintu->intu_init ||
		!d->felica_data->flintu->intu_read ||
		!d->felica_data->flintu->intu_release) {
		dev_err(d->dev, "%s: Error. Invalid operation.\n", __func__);
		ret = -EINVAL;
		goto err_invalid;
	}

	ret = d->felica_data->flintu->intu_init(d->felica_data->user);
	if (ret) {
		dev_err(d->dev, "%s: Error. INTU GPIO init failed.\n",
				__func__);
		goto err_init_intu_gpio;
	}

	if (misc_register(&d->device_intu_poll)) {
		dev_err(d->dev, "%s: Error. Cannot register NFC INTU POLL.\n",
			__func__);
		goto err_register;
	}

	return 0;

err_register:
	d->felica_data->flintu->intu_release(d->felica_data->user);
err_init_intu_gpio:
err_invalid:
	return ret;
}

static int nfc_intu_poll_irq_start(struct felica_dev *d)
{

	int ret;

	d->intu_curr_val =
		d->felica_data->flintu->intu_read(d->felica_data->user);

	ret = request_threaded_irq(d->felica_data->irq_intu, NULL,
		nfc_intu_poll_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"nfc_irq", d);
	if (ret) {
		dev_err(d->dev, "%s: Error. Request IRQ failed.ret=%d\n",
			__func__, ret);
		goto err_request_irq;
	}

	ret = enable_irq_wake(d->felica_data->irq_intu);
	if (ret) {
		dev_err(d->dev, "%s: Error. Enabling IRQ wake failed.ret=%d\n",
			__func__, ret);
		goto err_enable_irq_wake;
	}

	return 0;

err_enable_irq_wake:
	free_irq(d->felica_data->irq_intu, d);
err_request_irq:
	return ret;
}

static void nfc_intu_poll_irq_shutdown(struct felica_dev *d)
{
	disable_irq_wake(d->felica_data->irq_intu);
	free_irq(d->felica_data->irq_intu, d);
}

static void nfc_intu_poll_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_intu_poll);
	if (!d->irq_shutdown)
		nfc_intu_poll_irq_shutdown(d);
	d->felica_data->flintu->intu_release(d->felica_data->user);
}

static void nfc_available_poll_set_status(struct felica_dev *d,
			bool status, enum nfc_avp_type type)
{
	switch (type) {
	case AVP_CEN:
		d->available_poll_nfc &= (AVP_RFS_STS_BIT | AVP_URT_STS_BIT);
		d->available_poll_nfc |= status;
		break;
	case AVP_RFS:
		d->available_poll_nfc &= (AVP_CEN_STS_BIT | AVP_URT_STS_BIT);
		d->available_poll_nfc |= status << 1;
		d->initialized_rfs_val = true;
		break;
	case AVP_URT:
		d->available_poll_nfc &= (AVP_CEN_STS_BIT | AVP_RFS_STS_BIT);
		d->available_poll_nfc |= status << 2;
		break;
	default:
		dev_err(d->device_available_poll.this_device,
				"%s: Invalid type.\n", __func__);
		break;
	}

	if (AVP_ACTIVE_VALUE == (d->available_poll_nfc & AVP_ACTIVE_VALUE))
		wake_up_interruptible(&d->available_poll_wait);
}

static int nfc_available_poll_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
						device_available_poll);

	dev_dbg(d->device_available_poll.this_device, "%s\n", __func__);

	return 0;
}

static int nfc_available_poll_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
						device_available_poll);

	dev_dbg(d->device_available_poll.this_device, "%s\n", __func__);

	return 0;
}

static ssize_t nfc_available_poll_read(struct file *file, char __user *buf,
					size_t count, loff_t *offset)
{
	int ret;
	char kbuf = 1;

	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
						device_available_poll);
	dev_dbg(d->device_available_poll.this_device, "%s\n", __func__);

	ret = wait_event_interruptible(d->available_poll_wait,
				(d->available_poll_nfc & AVP_ACTIVE_VALUE) ==
				AVP_ACTIVE_VALUE);
	if (0 > ret)
		goto exit;

	dev_dbg(d->device_available_poll.this_device, "kbuf = 0x%x\n", kbuf);

	if (copy_to_user(buf, &kbuf, sizeof(kbuf))) {
		dev_err(d->device_available_poll.this_device,
				"%s: Error. copy_to_user failure.\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	ret = sizeof(kbuf);
exit:
	return ret;
}


static ssize_t nfc_available_poll_write(struct file *file,
			const char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	char kbuf;
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
						device_available_poll);

	dev_dbg(d->device_available_poll.this_device, "%s\n", __func__);

	if (1 != count || NULL == buf) {
		dev_err(d->device_available_poll.this_device,
				"%s: Error. Invalid arg @available_poll write.\n",
					__func__);
		ret = -EINVAL;
		goto exit;
	}

	if (copy_from_user(&kbuf, buf, sizeof(kbuf))) {
		dev_err(d->device_available_poll.this_device,
			"%s: Error. copy_from_user failure.\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	nfc_available_poll_set_status(d, kbuf == 1 ? 1 : 0, AVP_URT);

	/* 1 byte write */
	ret = sizeof(kbuf);

exit:
	return ret;
}

static const struct file_operations nfc_available_poll_fops = {
	.owner		= THIS_MODULE,
	.read		= nfc_available_poll_read,
	.write		= nfc_available_poll_write,
	.open		= nfc_available_poll_open,
	.release	= nfc_available_poll_release,
	.fsync		= NULL,
};

static int nfc_available_poll_probe_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	if (misc_register(&d->device_available_poll)) {
		dev_err(d->dev,
			"%s: Error. Cannot register NFC AVAILABLE POLL.\n",
			__func__);
		return -ENODEV;
	}

	return 0;
}

static void nfc_available_poll_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_available_poll);
}

static int nfc_cen_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
							device_nfc_cen);

	dev_dbg(d->device_nfc_cen.this_device, "%s\n", __func__);

	return cen_open(inode, file, d, d->device_nfc_cen.this_device);
}

static int nfc_cen_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
							device_nfc_cen);

	dev_dbg(d->device_nfc_cen.this_device, "%s\n", __func__);

	return cen_release(inode, file, d, d->device_nfc_cen.this_device);
}

static ssize_t nfc_cen_read(struct file *file, char __user *buf,
					size_t count, loff_t *offset)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
							device_nfc_cen);

	dev_dbg(d->device_nfc_cen.this_device, "%s\n", __func__);

	return cen_read(file, buf, count, offset,
				d, d->device_nfc_cen.this_device);
}

static ssize_t nfc_cen_write(struct file *file, const char __user *buf,
					size_t count, loff_t *offset)
{
	struct miscdevice *c = file->private_data;
	struct felica_dev *d = container_of(c, struct felica_dev,
							device_nfc_cen);

	dev_dbg(d->device_nfc_cen.this_device, "%s\n", __func__);

	return cen_write(file, buf, count, offset,
				d, d->device_nfc_cen.this_device);
}

static const struct file_operations nfc_cen_fops = {
	.owner		= THIS_MODULE,
	.read		= nfc_cen_read,
	.write		= nfc_cen_write,
	.open		= nfc_cen_open,
	.release	= nfc_cen_release,
	.fsync		= NULL,
};

static int nfc_cen_probe_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	if (!d->felica_data->flcen->cen_init ||
		!d->felica_data->flcen->cen_read ||
		!d->felica_data->flcen->cen_write ||
		!d->felica_data->flcen->cen_release) {
		dev_err(d->dev, "%s: Error. Invalid operation.\n", __func__);
		return -EINVAL;
	}

	/* Create CEN character device (/dev/snfc_cen) */
	if (misc_register(&d->device_nfc_cen)) {
		dev_err(d->dev, "%s: Error. Cannot register CEN.\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static void nfc_cen_remove_func(struct felica_dev *d)
{
	dev_dbg(d->dev, "%s\n", __func__);

	misc_deregister(&d->device_nfc_cen);
}

static struct device *reg_device;

int felica_snfc_register(struct device *dev, struct felica_data *felica_data)
{
	int ret;
	struct felica_dev *d;

	dev_dbg(dev, "%s\n", __func__);

	if (reg_device) {
		dev_err(dev, "%s: felica_snfc was registered.\n",  __func__);
		ret =  -EBUSY;
		goto err_inval;
	}

	if (!dev) {
		dev_err(dev, "%s: device is null.\n",  __func__);
		ret =  -EINVAL;
		goto err_inval;
	}
	if (!felica_data) {
		dev_err(dev, "%s: felica_data is null.\n",  __func__);
		ret =  -EINVAL;
		goto err_inval;
	}
	if (!felica_data->flcen || !felica_data->flpon ||
		!felica_data->flrfs || !felica_data->flint) {
		dev_err(dev, "%s: felica ops is null.\n",  __func__);
		ret =  -EINVAL;
		goto err_inval;
	}
	if (FELICA_SNFC == felica_data->type &&
		(!felica_data->flintu || !felica_data->flhsel ||
		!felica_data->flldo)) {
		dev_err(dev, "%s: nfc ops is null.\n",  __func__);
		ret =  -EINVAL;
		goto err_inval;
	}

	d = kzalloc(sizeof(struct felica_dev), GFP_KERNEL);
	if (!d) {
		dev_err(dev, "%s: no memory\n", __func__);
		ret = -ENOMEM;
		goto err_alloc;
	}
	d->felica_data = felica_data;
	d->irq_shutdown = true;

	d->device_cen.minor = MISC_DYNAMIC_MINOR;
	d->device_cen.name = "felica_cen";
	d->device_cen.fops = &felica_cen_fops;

	d->device_pon.minor = MISC_DYNAMIC_MINOR;
	d->device_pon.name = "felica_pon";
	d->device_pon.fops = &felica_pon_fops;

	d->device_rfs.minor = MISC_DYNAMIC_MINOR;
	d->device_rfs.name = "felica_rfs";
	d->device_rfs.fops = &felica_rfs_fops;

	d->device_rws.minor = MISC_DYNAMIC_MINOR;
	d->device_rws.name = "felica_rws";
	d->device_rws.fops = &felica_rws_fops;

	if (FELICA_SNFC == felica_data->type) {
		d->device_hsel.minor = MISC_DYNAMIC_MINOR;
		d->device_hsel.name = "snfc_hsel";
		d->device_hsel.fops = &nfc_hsel_fops;

		d->device_intu_poll.minor = MISC_DYNAMIC_MINOR;
		d->device_intu_poll.name = "snfc_intu_poll";
		d->device_intu_poll.fops = &nfc_intu_poll_fops;

		d->device_available_poll.minor = MISC_DYNAMIC_MINOR;
		d->device_available_poll.name = "snfc_available_poll";
		d->device_available_poll.fops = &nfc_available_poll_fops;

		d->device_nfc_cen.minor = MISC_DYNAMIC_MINOR;
		d->device_nfc_cen.name = "snfc_cen";
		d->device_nfc_cen.fops = &nfc_cen_fops;
	}

	d->dev = dev;
	dev_set_drvdata(dev, d);

	ret = felica_cen_probe_func(d);
	if (ret) {
		dev_err(dev, "%s: CEN probe failure\n", __func__);
		goto err_cen_probe;
	}

	ret = felica_pon_probe_func(d);
	if (ret) {
		dev_err(dev, "%s: PON probe failure\n", __func__);
		goto err_pon_probe;
	}

	ret = felica_rfs_probe_func(d);
	if (ret) {
		dev_err(dev, "%s: RFS probe failure\n", __func__);
		goto err_rfs_probe;
	}

	ret = felica_int_probe_func(d);
	if (ret) {
		dev_err(dev, "%s: INT probe failure\n", __func__);
		goto err_int_probe;
	}

	ret = felica_rws_probe_func(d);
	if (ret) {
		dev_err(dev, "%s: RWS probe failure\n", __func__);
		goto err_rws_probe;
	}

	if (FELICA_SNFC == felica_data->type) {
		ret = nfc_hsel_probe_func(d);
		if (ret) {
			dev_err(dev, "%s: NFC HSEL probe failure\n", __func__);
			goto err_nfc_hsel_probe;
		}

		ret = nfc_intu_poll_probe_func(d);
		if (ret) {
			dev_err(dev, "%s: NFC INTU POLL probe failure\n",
				__func__);
			goto err_nfc_intu_poll_probe;
		}
		init_waitqueue_head(&d->intu_wait_queue);

		ret = nfc_available_poll_probe_func(d);
		if (ret) {
			dev_err(dev, "%s: NFC AVAILABLE POLL probe failure\n",
				__func__);
			goto err_nfc_available_poll_probe;
		}
		init_waitqueue_head(&d->available_poll_wait);
		d->available_poll_nfc = 0;

		ret = nfc_cen_probe_func(d);
		if (ret) {
			dev_err(dev, "%s: SNFC CEN probe failure\n", __func__);
			goto err_nfc_cen_probe;
		}
	}
	d->sysfs_dev.init_name = "felica_snfc";
	dev_set_drvdata(&d->sysfs_dev, d);
	ret = device_register(&d->sysfs_dev);
	if (ret) {
		dev_err(dev, "%s: failed to register device.\n", __func__);
		goto err_register_device;
	}

	ret = create_sysfs_interfaces(&d->sysfs_dev);
	if (ret) {
		dev_err(dev, "%s: failed to create dev.\n", __func__);
		goto err_create_sysfs;
	}

	reg_device = dev;

	return 0;

err_create_sysfs:
	device_unregister(&d->sysfs_dev);
err_register_device:
	nfc_cen_remove_func(d);
err_nfc_cen_probe:
	nfc_available_poll_remove_func(d);
err_nfc_available_poll_probe:
	nfc_intu_poll_remove_func(d);
err_nfc_intu_poll_probe:
	nfc_hsel_remove_func(d);
err_nfc_hsel_probe:
	felica_rws_remove_func(d);
err_rws_probe:
	felica_int_remove_func(d);
err_int_probe:
	felica_rfs_remove_func(d);
err_rfs_probe:
	felica_pon_remove_func(d);
err_pon_probe:
	felica_cen_remove_func(d);
err_cen_probe:
	kfree(d);
err_alloc:
err_inval:
	return ret;
}
EXPORT_SYMBOL(felica_snfc_register);

int felica_snfc_unregister(struct device *dev)
{
	struct felica_dev *d = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (!dev || !reg_device || reg_device != dev)
		return -EINVAL;

	remove_sysfs_interfaces(&d->sysfs_dev);
	device_unregister(&d->sysfs_dev);

	felica_rws_remove_func(d);
	felica_int_remove_func(d);
	felica_rfs_remove_func(d);
	felica_pon_remove_func(d);
	felica_cen_remove_func(d);

	if (FELICA_SNFC == d->felica_data->type) {
		nfc_cen_remove_func(d);
		nfc_available_poll_remove_func(d);
		nfc_intu_poll_remove_func(d);
		nfc_hsel_remove_func(d);
	}

	kfree(d);

	reg_device = 0;

	return 0;
}
EXPORT_SYMBOL(felica_snfc_unregister);

int felica_snfc_irq_start(struct device *dev)
{
	int ret;
	struct felica_dev *d = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (!dev || !reg_device || reg_device != dev)
		return -EINVAL;

	if (d->irq_shutdown) {
		ret = felica_int_irq_start(d);
		if (ret) {
			dev_err(dev, "%s: failed to start INT irq.\n",
				__func__);
			goto err_int_irq;
		}
		if (FELICA_SNFC == d->felica_data->type) {
			ret = nfc_intu_poll_irq_start(d);
			if (ret) {
				dev_err(dev, "%s: failed to start INTU irq.\n",
					__func__);
				goto err_intu_irq;
			}
		}
		ret = felica_rfs_irq_start(d);
		if (ret) {
			dev_err(dev, "%s: failed to start RFS irq.\n",
				__func__);
			goto err_rfs_irq;
		}
		d->irq_shutdown = false;
	}
	return 0;

err_rfs_irq:
	if (FELICA_SNFC == d->felica_data->type)
		nfc_intu_poll_irq_shutdown(d);
err_intu_irq:
	felica_int_irq_shutdown(d);
err_int_irq:
	return ret;
}
EXPORT_SYMBOL(felica_snfc_irq_start);

int felica_snfc_irq_shutdown(struct device *dev)
{
	struct felica_dev *d = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (!dev || !reg_device || reg_device != dev)
		return -EINVAL;

	if (!d->irq_shutdown) {
		felica_int_irq_shutdown(d);
		if (FELICA_SNFC == d->felica_data->type)
			nfc_intu_poll_irq_shutdown(d);
		felica_rfs_irq_shutdown(d);
		d->irq_shutdown = true;
	}
	return 0;
}
EXPORT_SYMBOL(felica_snfc_irq_shutdown);

MODULE_AUTHOR("Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>");
MODULE_DESCRIPTION("FeliCa and Sony NFC driver");
MODULE_LICENSE("GPL");
