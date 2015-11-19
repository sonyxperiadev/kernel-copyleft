/* drivers/misc/oneseg_tuner_drv.c
 *
 * Copyright (C) 2013 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#define D_ONESEG_CONFIG_MODULE_NAME          "Oneseg Tuner Driver"
#define D_ONESEG_CONFIG_MODULE_LICENCE       "GPL"
#define D_ONESEG_CONFIG_DRIVER_NAME          "onesegtuner_drv"
#define D_ONESEG_CONFIG_PLATFORM_DRIVER_NAME "onesegtuner_pdev"
#define D_ONESEG_CONFIG_SYSFS_DEV_NAME       "onesegtuner_pdev"
#define D_ONESEG_CONFIG_CLASS_NAME           "onesegtuner"
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
#define D_ONESEG_CONFIG_MATCH_TABLE          "sony,vj190"

#define D_ONESEG_CONFIG_DRV_MAJOR             110
#define D_ONESEG_CONFIG_DRV_MINOR             210
#define D_ONESEG_POWER_ON_WAIT_US            3000
#define D_ONESEG_POWER_ON_WAIT_RANGE_US      3100
#define D_ONESEG_RESET_ON_WAIT_US            1000
#define D_ONESEG_RESET_ON_WAIT_RANGE_US      1100
#define D_ONESEG_RESET_OFF_WAIT_US           1000
#define D_ONESEG_RESET_OFF_WAIT_RANGE_US     1100
#define D_ONESEG_I2C_ADAPTER_ID                11
#else
#define D_ONESEG_CONFIG_MATCH_TABLE          "sony,vj180"

#define D_ONESEG_CONFIG_DRV_MAJOR             110
#define D_ONESEG_CONFIG_DRV_MINOR             210
#define D_ONESEG_POWER_ON_WAIT_US           15000
#define D_ONESEG_POWER_ON_WAIT_RANGE_US     15100
#define D_ONESEG_RESET_ON_WAIT_US            5000
#define D_ONESEG_RESET_ON_WAIT_RANGE_US      5100
#define D_ONESEG_RESET_OFF_WAIT_US           1000
#define D_ONESEG_RESET_OFF_WAIT_RANGE_US     1100
#define D_ONESEG_POWER_OFF_WAIT_US           5000
#define D_ONESEG_POWER_OFF_WAIT_RANGE_US     5100
#define D_ONESEG_I2C_ADAPTER_ID                11
#endif

enum oneseg_gpio_id {
	ONESEG_POWER_PIN = 0,
	ONESEG_RESET_PIN,
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
	ONESEG_INT_PIN,
#endif
};

static char const * const oneseg_gpio_rsrcs[] = {
	"Oneseg tuner power",
	"Oneseg tuner reset",
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
	"Oneseg tuner int",
#endif
};

enum ONESEG_DRV_CTL {
	ONESEG_DRV_CTL_POWON,
	ONESEG_DRV_CTL_POWOFF,
	ONESEG_DRV_CTL_RESET
};
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
enum ONESEG_DRV_IRQ {
	ONESEG_DRV_IRQ_NOT_DETECTED,
	ONESEG_DRV_IRQ_DETECTED,
};
#endif
struct oneseg_tuner_drvdata {
	struct device *dev;
	struct device sysfs_dev;
	struct mutex mutex_lock;
	unsigned int gpios[ARRAY_SIZE(oneseg_gpio_rsrcs)];
	struct i2c_adapter *adap;
};

struct g_oneseg_tuner_device {
	struct mutex g_tuner_mutex;
	unsigned long open_cnt;
	struct platform_device *onesegtuner_device;
	struct class *device_class;
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
	u32 irq_flag;
	wait_queue_head_t irq_wait_q;
	int irq_num;
#endif
} oneseg_dev;

static enum oneseg_gpio_id req_ids[] = {
	ONESEG_POWER_PIN,
	ONESEG_RESET_PIN,
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
	ONESEG_INT_PIN,
#endif
};

static void oneseg_tunerpm_power_control(struct oneseg_tuner_drvdata
	*drvdata, int on)
{
	gpio_set_value_cansleep(drvdata->gpios[ONESEG_POWER_PIN], on);
}

static void oneseg_tunerpm_reset_control(struct oneseg_tuner_drvdata *drvdata,
	int on)
{
	gpio_set_value_cansleep(drvdata->gpios[ONESEG_RESET_PIN], on);
}

static int oneseg_dev_init(struct platform_device *pdev,
	struct oneseg_tuner_drvdata *drvdata)
{
	int i, ret, gpio;
	unsigned int flags;
	struct device_node *of_node = pdev->dev.of_node;

	mutex_init(&drvdata->mutex_lock);

	for (i = 0; i < ARRAY_SIZE(oneseg_gpio_rsrcs); i++) {
		gpio = of_get_gpio_flags(of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			ret = -EINVAL;
			goto error_gpio;
		}
		drvdata->gpios[i] = gpio;
	}

	for (i = 0; i < ARRAY_SIZE(req_ids); i++) {
		ret = gpio_request(drvdata->gpios[req_ids[i]],
				oneseg_gpio_rsrcs[req_ids[i]]);
		if (ret)
			goto error_gpio_request;
	}

	oneseg_tunerpm_power_control(drvdata, 0);
	oneseg_tunerpm_reset_control(drvdata, 0);

	return 0;

error_gpio_request:
	for (i--; 0 <= i; i--)
		gpio_free(drvdata->gpios[req_ids[i]]);
error_gpio:
	return ret;
}

static void oneseg_dev_finalize(struct oneseg_tuner_drvdata *drvdata)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(req_ids); i++)
		gpio_free(drvdata->gpios[req_ids[i]]);
}

static int tuner_drv_ctl_power(struct oneseg_tuner_drvdata *drvdata, int data)
{
	switch (data) {
	case ONESEG_DRV_CTL_POWON:
		mutex_lock(&drvdata->mutex_lock);
		oneseg_tunerpm_power_control(drvdata, 1);
		mutex_unlock(&drvdata->mutex_lock);
		usleep_range(D_ONESEG_POWER_ON_WAIT_US,
			D_ONESEG_POWER_ON_WAIT_RANGE_US);
		break;
	case ONESEG_DRV_CTL_RESET:
		i2c_lock_adapter(drvdata->adap);
		mutex_lock(&drvdata->mutex_lock);
		oneseg_tunerpm_reset_control(drvdata, 1);
		mutex_unlock(&drvdata->mutex_lock);
		usleep_range(D_ONESEG_RESET_ON_WAIT_US,
			D_ONESEG_RESET_ON_WAIT_RANGE_US);
		i2c_unlock_adapter(drvdata->adap);
		break;
	case ONESEG_DRV_CTL_POWOFF:
		mutex_lock(&drvdata->mutex_lock);
		usleep_range(D_ONESEG_RESET_OFF_WAIT_US,
			D_ONESEG_RESET_OFF_WAIT_RANGE_US);
		oneseg_tunerpm_reset_control(drvdata, 0);
#ifdef CONFIG_ONESEG_TUNER_SMTVJ18X
		usleep_range(D_ONESEG_POWER_OFF_WAIT_US,
			D_ONESEG_POWER_OFF_WAIT_RANGE_US);
#endif
		oneseg_tunerpm_power_control(drvdata, 0);
		mutex_unlock(&drvdata->mutex_lock);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
irqreturn_t tuner_interrupt(int irq, void *dev_id)
{
	if (oneseg_dev.irq_flag == ONESEG_DRV_IRQ_NOT_DETECTED) {
		oneseg_dev.irq_flag = ONESEG_DRV_IRQ_DETECTED;
		wake_up_interruptible(&oneseg_dev.irq_wait_q);
	}
	return IRQ_HANDLED;
}

static int tuner_drv_set_interrupt(int int_num)
{
	int ret;

	oneseg_dev.irq_num = gpio_to_irq(int_num);
	ret = request_threaded_irq(oneseg_dev.irq_num, tuner_interrupt,
		NULL, IRQF_TRIGGER_RISING, D_ONESEG_CONFIG_CLASS_NAME, NULL);
	if (ret) {
		gpio_free(int_num);
		return -EINVAL;
	}
	return 0;
}

static void tuner_drv_release_interrupt(void)
{
	free_irq(oneseg_dev.irq_num, NULL);
}
#endif
static ssize_t tuner_module_power_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct oneseg_tuner_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (!value) {
		ret = tuner_drv_ctl_power(drvdata, ONESEG_DRV_CTL_POWON);
		if (ret)
			return -EINVAL;
		ret = tuner_drv_ctl_power(drvdata, ONESEG_DRV_CTL_RESET);
		if (ret) {
			tuner_drv_ctl_power(drvdata, ONESEG_DRV_CTL_POWOFF);
			return -EINVAL;
		}
	} else {
		tuner_drv_ctl_power(drvdata, ONESEG_DRV_CTL_POWOFF);
	}
	return count;
}
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
static ssize_t tuner_module_irq_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
	struct oneseg_tuner_drvdata *drvdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (!value) {
		if (tuner_drv_set_interrupt(drvdata->gpios[ONESEG_INT_PIN]))
			return -EINVAL;
	} else {
		tuner_drv_release_interrupt();
		oneseg_dev.irq_flag = ONESEG_DRV_IRQ_NOT_DETECTED;
	}

	return count;
}

static ssize_t tuner_module_irq_detect_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	wait_event_interruptible(oneseg_dev.irq_wait_q,
		(oneseg_dev.irq_flag == ONESEG_DRV_IRQ_DETECTED));
	return snprintf(buf, 1, "%d", oneseg_dev.irq_flag);
}

static ssize_t tuner_module_irq_detect_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	oneseg_dev.irq_flag = ONESEG_DRV_IRQ_NOT_DETECTED;
	return count;
}
#endif
static int tuner_module_entry_open(struct inode *inode, struct file *file)
{
	if (oneseg_dev.open_cnt > 0)
		return -EBUSY;
	else
		oneseg_dev.open_cnt++;
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
	oneseg_dev.irq_flag = ONESEG_DRV_IRQ_NOT_DETECTED;
#endif
	return 0;
}

static int tuner_module_entry_close(struct inode *inode, struct file *file)
{
	struct devone_data *dev;

	if (oneseg_dev.open_cnt <= 0)
		return -ESRCH;
	else
		oneseg_dev.open_cnt--;

	if (oneseg_dev.open_cnt == 0) {
		if (!file)
			return -ESRCH;
		dev = file->private_data;
	}

	return 0;
}

static struct device_attribute tuner_sysfs_attrs[] = {
	__ATTR(oneseg_power_ctrl, S_IWUSR, 0, tuner_module_power_ctrl),
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
	__ATTR(oneseg_irq_ctrl, S_IWUSR, 0, tuner_module_irq_ctrl),
	__ATTR(oneseg_irq, S_IWUSR | S_IRUSR, tuner_module_irq_detect_read,
		tuner_module_irq_detect_write),
#endif
};

static const struct file_operations tuner_file_operations = {
	.owner   = THIS_MODULE,
	.open    = tuner_module_entry_open,
	.release = tuner_module_entry_close
};

static int oneseg_tuner_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	struct device *dev = NULL;
	struct oneseg_tuner_drvdata *drvdata;

	oneseg_dev.onesegtuner_device = platform_device_alloc(
		D_ONESEG_CONFIG_CLASS_NAME, -1);

	if (!oneseg_dev.onesegtuner_device) {
		ret = -ENOMEM;
		goto err_platform_device_alloc;
	}

	ret = platform_device_add(oneseg_dev.onesegtuner_device);
	if (ret)
		goto err_platform_device_add;

	oneseg_dev.device_class = class_create(THIS_MODULE,
		D_ONESEG_CONFIG_CLASS_NAME);
	if (IS_ERR(oneseg_dev.device_class)) {
		ret = PTR_ERR(oneseg_dev.device_class);
		goto err_class_create;
	}

	dev = device_create(oneseg_dev.device_class, NULL,
		MKDEV(D_ONESEG_CONFIG_DRV_MAJOR, D_ONESEG_CONFIG_DRV_MINOR),
		NULL, D_ONESEG_CONFIG_CLASS_NAME);

	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto err_device_create;
	}

	drvdata = kzalloc(sizeof(struct oneseg_tuner_drvdata), GFP_KERNEL);
	if (!drvdata) {
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	drvdata->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, drvdata);
	drvdata->sysfs_dev.init_name = D_ONESEG_CONFIG_SYSFS_DEV_NAME;
	dev_set_drvdata(&drvdata->sysfs_dev, drvdata);
	ret = device_register(&drvdata->sysfs_dev);
	if (ret)
		goto err_set_dev;

	ret = register_chrdev(D_ONESEG_CONFIG_DRV_MAJOR,
		D_ONESEG_CONFIG_DRIVER_NAME, &tuner_file_operations);
	if (ret < 0)
		goto err_register_device;

	drvdata->adap = i2c_get_adapter(D_ONESEG_I2C_ADAPTER_ID);
	if (!drvdata->adap)
		goto err_i2c_get_adapter;

	ret = oneseg_dev_init(pdev, drvdata);
	if (ret)
		goto err_gpio_init;

	oneseg_dev.open_cnt = 0;

	for (i = 0; i < ARRAY_SIZE(tuner_sysfs_attrs); i++) {
		ret = device_create_file(&drvdata->sysfs_dev,
			&tuner_sysfs_attrs[i]);
		if (ret) {
			for (; i >= 0; --i)
				device_remove_file(&drvdata->sysfs_dev,
					&tuner_sysfs_attrs[i]);
			goto err_create_file;
		}
	}

	mutex_init(&oneseg_dev.g_tuner_mutex);
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
	init_waitqueue_head(&oneseg_dev.irq_wait_q);

	if (tuner_drv_set_interrupt(drvdata->gpios[ONESEG_INT_PIN]))
		goto err_irq_set;
#endif
	return 0;
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
err_irq_set:
#endif
err_create_file:
err_gpio_init:
	i2c_put_adapter(drvdata->adap);
err_i2c_get_adapter:
	unregister_chrdev(D_ONESEG_CONFIG_DRV_MAJOR,
		D_ONESEG_CONFIG_DRIVER_NAME);
err_register_device:
	device_unregister(&drvdata->sysfs_dev);
err_set_dev:
	kzfree(drvdata);
err_alloc_data:
	device_destroy(oneseg_dev.device_class, MKDEV(D_ONESEG_CONFIG_DRV_MAJOR,
		D_ONESEG_CONFIG_DRV_MINOR));
err_device_create:
	class_destroy(oneseg_dev.device_class);
err_class_create:
	platform_device_del(oneseg_dev.onesegtuner_device);
err_platform_device_add:
	platform_device_put(oneseg_dev.onesegtuner_device);
err_platform_device_alloc:
	return ret;
}

static int __devexit oneseg_tuner_remove(struct platform_device *pdev)
{
	struct oneseg_tuner_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	oneseg_dev_finalize(drvdata);
	i2c_put_adapter(drvdata->adap);
	unregister_chrdev(D_ONESEG_CONFIG_DRV_MAJOR,
		D_ONESEG_CONFIG_DRIVER_NAME);

	return 0;
}

static void oneseg_tuner_shutdown(struct platform_device *pdev)
{
	device_destroy(oneseg_dev.device_class, MKDEV(D_ONESEG_CONFIG_DRV_MAJOR,
		D_ONESEG_CONFIG_DRV_MINOR));
	class_destroy(oneseg_dev.device_class);
	platform_device_unregister(oneseg_dev.onesegtuner_device);
}
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
static struct of_device_id vj190_match_table[] = {
#else
static struct of_device_id vj180_match_table[] = {
#endif
{	.compatible = D_ONESEG_CONFIG_MATCH_TABLE,
},
{}
};

static struct platform_driver onesegtuner_driver = {
	.probe  = oneseg_tuner_probe,
	.remove = __exit_p(oneseg_tuner_remove),
	.shutdown = oneseg_tuner_shutdown,
	.driver = {
		.name = D_ONESEG_CONFIG_PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_ONESEG_TUNER_SMTVJ19X
		.of_match_table = vj190_match_table,
#else
		.of_match_table = vj180_match_table,
#endif
	},
};

static int __init oneseg_drv_start(void)
{
	return platform_driver_register(&onesegtuner_driver);
}

static void __exit oneseg_drv_end(void)
{
	platform_driver_unregister(&onesegtuner_driver);
}

MODULE_LICENSE(D_ONESEG_CONFIG_MODULE_LICENCE);
MODULE_DESCRIPTION(D_ONESEG_CONFIG_MODULE_NAME);

module_init(oneseg_drv_start);
module_exit(oneseg_drv_end);
