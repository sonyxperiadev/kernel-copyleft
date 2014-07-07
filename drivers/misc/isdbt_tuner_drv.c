/******************************************************************************
 *
 *  file name       : isdbt_tuner_drv.c
 *  brief note      : The Control Layer of Tmm Tuner Driver
 *
 *  creation data   : 2011.07.25
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 322                        $ Revision of Last commit
 *  $Date:: 2011-10-26 13:33:02 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
 *              Copyright (C) 2012 Sony Mobile Communications AB.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/07/25    K.Kitamura(*)
 *                001 new creation
 *              : 2012/12/19    T.Ooka(*)
 *                002 modified
 ******************************************************************************/
#include <asm/irq.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <mach/isdbt_tunerpm.h>

#define TUNER_CONFIG_DRIVER_NAME     "mmtuner_drv"
#define TUNER_CONFIG_DRV_MAJOR           100
#define TUNER_CONFIG_DRV_MINOR           200
#define TUNER_INT                         36
#define TUNER_POWER_ON_WAIT_US         15000
#define TUNER_POWER_ON_WAIT_RANGE_US   15100
#define TUNER_RESET_ON_WAIT_US          2000
#define TUNER_RESET_ON_WAIT_RANGE_US    2100
#define TUNER_RESET_OFF_WAIT_US         1000
#define TUNER_RESET_OFF_WAIT_RANGE_US   1100
#define TUNER_POWER_OFF_WAIT_US         2000
#define TUNER_POWER_OFF_WAIT_RANGE_US   2100

enum TUNER_DRV_CTL {
	TUNER_DRV_CTL_POWON,
	TUNER_DRV_CTL_POWOFF,
	TUNER_DRV_CTL_RESET
};

enum TUNER_DRV_IRQ {
	TUNER_DRV_IRQ_NOT_DETECTED,
	TUNER_DRV_IRQ_DETECTED,
};

struct tuner_drvdata {
	struct platform_device *pdev;
	struct mutex mutex_lock;
	struct i2c_adapter *adap;
};

struct g_tuner_device {
	struct mutex g_tuner_mutex;
	unsigned long open_cnt;
	struct platform_device *mmtuner_device;
	struct class *device_class;
	u32 irq_flag;
	wait_queue_head_t irq_wait_q;
	int irq_num;
} tnr_dev;

int tunerpm_dev_init(struct tuner_drvdata *drvdata)
{
	struct isdbt_tunerpm_platform_data *pfdata =
		drvdata->pdev->dev.platform_data;
	mutex_init(&drvdata->mutex_lock);
	if (pfdata->init)
		return pfdata->init(&drvdata->pdev->dev);
	return 0;
}

int tunerpm_dev_finalize(struct tuner_drvdata *drvdata)
{
	struct isdbt_tunerpm_platform_data *pfdata =
		drvdata->pdev->dev.platform_data;

	if (pfdata->free)
		return pfdata->free(&drvdata->pdev->dev);

	return 0;
}

int tuner_drv_ctl_power(struct tuner_drvdata *drvdata, int data)
{
	struct isdbt_tunerpm_platform_data *pfdata =
		drvdata->pdev->dev.platform_data;

	switch (data) {
	case TUNER_DRV_CTL_POWON:
		mutex_lock(&drvdata->mutex_lock);
		if (pfdata->power_control)
			pfdata->power_control(&drvdata->pdev->dev, 1);
		mutex_unlock(&drvdata->mutex_lock);
		usleep_range(TUNER_POWER_ON_WAIT_US,
			TUNER_POWER_ON_WAIT_RANGE_US);
		break;
	case TUNER_DRV_CTL_RESET:
		i2c_lock_adapter(drvdata->adap);
		mutex_lock(&drvdata->mutex_lock);
		if (pfdata->reset_control)
			pfdata->reset_control(&drvdata->pdev->dev, 1);
		mutex_unlock(&drvdata->mutex_lock);
		usleep_range(TUNER_RESET_ON_WAIT_US,
			TUNER_RESET_ON_WAIT_RANGE_US);
		i2c_unlock_adapter(drvdata->adap);
		break;
	case TUNER_DRV_CTL_POWOFF:
		mutex_lock(&drvdata->mutex_lock);
		usleep_range(TUNER_RESET_OFF_WAIT_US,
			TUNER_RESET_OFF_WAIT_RANGE_US);
		if (pfdata->reset_control)
			pfdata->reset_control(&drvdata->pdev->dev, 0);
		usleep_range(TUNER_POWER_OFF_WAIT_US,
			TUNER_POWER_OFF_WAIT_RANGE_US);
		if (pfdata->power_control)
			pfdata->power_control(&drvdata->pdev->dev, 0);
		gpio_free(TUNER_INT);
		mutex_unlock(&drvdata->mutex_lock);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int tuner_drv_ant_switch(struct tuner_drvdata *drvdata, int ant_mode)
{
	struct isdbt_tunerpm_platform_data *pfdata =
		drvdata->pdev->dev.platform_data;

	if (pfdata->ant_switch)
		pfdata->ant_switch(&drvdata->pdev->dev, ant_mode);

	return 0;
}

irqreturn_t tuner_interrupt(int irq, void *dev_id)
{
	if (tnr_dev.irq_flag == TUNER_DRV_IRQ_NOT_DETECTED) {
		tnr_dev.irq_flag = TUNER_DRV_IRQ_DETECTED;
		wake_up_interruptible(&tnr_dev.irq_wait_q);
	}
	return IRQ_HANDLED;
}

int tuner_drv_set_interrupt(void)
{
	int ret;

	ret = gpio_request(TUNER_INT, "ISDB-T INT");
	if (ret)
		return ret;

	tnr_dev.irq_num = gpio_to_irq(TUNER_INT);

	ret = request_threaded_irq(tnr_dev.irq_num, tuner_interrupt,
		NULL, IRQF_TRIGGER_RISING, "mm_tuner", NULL);
	if (ret) {
		gpio_free(TUNER_INT);
		return -EINVAL;
	}
	return 0;
}

void tuner_drv_release_interrupt(void)
{
	free_irq(tnr_dev.irq_num, NULL);
}

static ssize_t tuner_module_power_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tuner_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 0) {
		ret = tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWON);
		if (ret)
			return -EINVAL;
		ret = tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_RESET);
		if (ret) {
			tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWOFF);
			return -EINVAL;
		}
	} else {
		tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWOFF);
	}
	return count;
}

static ssize_t tuner_module_ant_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct tuner_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	ret = tuner_drv_ant_switch(drvdata, value);
	if (ret)
		return -EINVAL;

	return count;
}

static ssize_t tuner_module_irq_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (value == 0) {
		if (tuner_drv_set_interrupt())
			return -EINVAL;
	} else {
		tuner_drv_release_interrupt();
		tnr_dev.irq_flag = TUNER_DRV_IRQ_NOT_DETECTED;
	}

	return count;
}

static ssize_t tuner_module_irq_detect_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	wait_event_interruptible(tnr_dev.irq_wait_q,
		(tnr_dev.irq_flag == TUNER_DRV_IRQ_DETECTED));
	return snprintf(buf, 1, "%d", tnr_dev.irq_flag);
}

static ssize_t tuner_module_irq_detect_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	tnr_dev.irq_flag = TUNER_DRV_IRQ_NOT_DETECTED;
	return count;
}

static int tuner_module_entry_open(struct inode *inode, struct file *file)
{
	if (tnr_dev.open_cnt > 0)
		return -EBUSY;
	else
		tnr_dev.open_cnt++;

	tnr_dev.irq_flag = TUNER_DRV_IRQ_NOT_DETECTED;

	return 0;
}

static int tuner_module_entry_close(struct inode *inode, struct file *file)
{
	struct devone_data *dev;

	if (tnr_dev.open_cnt <= 0)
		return -ESRCH;
	else
		tnr_dev.open_cnt--;

	if (tnr_dev.open_cnt == 0) {
		tuner_drv_release_interrupt();
		if (!file)
			return -ESRCH;

		dev = file->private_data;
	}
	tnr_dev.irq_flag = TUNER_DRV_IRQ_NOT_DETECTED;

	return 0;
}

static struct device_attribute tuner_sysfs_attrs[] = {
	__ATTR(power_ctrl, S_IWUSR, 0, tuner_module_power_ctrl),
	__ATTR(ant_ctrl, S_IWUSR, 0, tuner_module_ant_ctrl),
	__ATTR(tuner_irq_ctrl, S_IWUSR, 0, tuner_module_irq_ctrl),
	__ATTR(tuner_irq, S_IWUSR | S_IRUSR, tuner_module_irq_detect_read,
		tuner_module_irq_detect_write),
};

static const struct file_operations tuner_file_operations = {
	.owner   = THIS_MODULE,
	.open    = tuner_module_entry_open,
	.release = tuner_module_entry_close
};

static int tuner_probe(struct platform_device *pdev)
{
	int ret;
	int i;

	struct isdbt_tunerpm_platform_data *pfdata;
	struct tuner_drvdata *drvdata;

	pfdata = pdev->dev.platform_data;

	if (!pfdata) {
		dev_err(&pdev->dev, "No platform data.\n");
		ret = -EINVAL;
		goto err_get_platform_data;
	}

	drvdata = kzalloc(sizeof(struct tuner_drvdata), GFP_KERNEL);
	if (!drvdata) {
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	ret = register_chrdev(TUNER_CONFIG_DRV_MAJOR,
		TUNER_CONFIG_DRIVER_NAME, &tuner_file_operations);
	if (ret < 0)
		goto err_register_device;

	drvdata->adap = i2c_get_adapter(pfdata->i2c_adapter_id);
	if (!drvdata->adap)
		goto err_i2c_get_adapter;

	ret = tunerpm_dev_init(drvdata);
	if (ret)
		goto err_gpio_init;

	tnr_dev.open_cnt = 0;

	for (i = 0; i < ARRAY_SIZE(tuner_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &tuner_sysfs_attrs[i]);
		if (ret) {
			for (; i >= 0; --i)
				device_remove_file(&pdev->dev,
					&tuner_sysfs_attrs[i]);
			goto err_create_file;
		}
	}

	mutex_init(&tnr_dev.g_tuner_mutex);
	init_waitqueue_head(&tnr_dev.irq_wait_q);

	return 0;

err_create_file:
err_gpio_init:
	i2c_put_adapter(drvdata->adap);
err_i2c_get_adapter:
err_register_device:
	kfree(drvdata);
err_alloc_data:
err_get_platform_data:
	return ret;
}

static int __devexit tuner_remove(struct platform_device *pdev)
{
	struct tuner_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	tuner_drv_release_interrupt();

	tunerpm_dev_finalize(drvdata);
	kfree(drvdata);
	i2c_put_adapter(drvdata->adap);
	unregister_chrdev(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRIVER_NAME);

	return 0;
}

static struct platform_driver mmtuner_driver = {
	.probe  = tuner_probe,
	.remove = __exit_p(tuner_remove),
	.driver = {
		.name = D_TUNER_CONFIG_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init tuner_drv_start(void)
{
	int ret;
	struct device *dev = NULL;

	ret = platform_driver_register(&mmtuner_driver);

	if (ret)
		goto err_platform_driver_register;

	tnr_dev.mmtuner_device = platform_device_alloc("mmtuner", -1);

	if (!tnr_dev.mmtuner_device) {
		ret = -ENOMEM;
		goto err_platform_device_alloc;
	}

	ret = platform_device_add(tnr_dev.mmtuner_device);
	if (ret)
		goto err_platform_device_add;

	tnr_dev.device_class = class_create(THIS_MODULE, "mmtuner");
	if (IS_ERR(tnr_dev.device_class)) {
		ret = PTR_ERR(tnr_dev.device_class);
		goto err_class_create;
	}

	dev = device_create(tnr_dev.device_class, NULL,
		MKDEV(TUNER_CONFIG_DRV_MAJOR, TUNER_CONFIG_DRV_MINOR),
		NULL, "mmtuner");

	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto err_device_create;
	}

	return 0;
err_device_create:
err_class_create:
err_platform_device_add:
	platform_device_put(tnr_dev.mmtuner_device);
err_platform_device_alloc:
	platform_driver_unregister(&mmtuner_driver);
err_platform_driver_register:
	return ret;
}

static void __exit tuner_drv_end(void)
{
	device_destroy(tnr_dev.device_class, MKDEV(TUNER_CONFIG_DRV_MAJOR,
		TUNER_CONFIG_DRV_MINOR));
	class_destroy(tnr_dev.device_class);
	platform_device_unregister(tnr_dev.mmtuner_device);
	platform_driver_unregister(&mmtuner_driver);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MM Tuner Driver");

module_init(tuner_drv_start);
module_exit(tuner_drv_end);
