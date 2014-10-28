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
 *              Copyright (C) 2012 Sony Mobile Communications Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful
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
 *              : 2013/04/10    T.Ooka(*)
 *                003 modified for SOMC platform
 *              : 2013/07/01    T.Ooka(*)
 *                004 fix power leakages from GPIOs
 *              : 2013/11/27    T.Ooka(*)
 *                005 modified some formats
 *              : 2014/02/10    T.Ooka(*)
 *                006 smt-ej12x adaptation
 *              : 2014/04/21    T.Ooka(*)
 *                007 ANT switch adaptation
 *              : 2014/08/01    T.Ooka(*)
 *                008 Add product switch
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
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#define D_TUNER_CONFIG_MODULE_NAME          "MM Tuner Driver"
#define D_TUNER_CONFIG_MODULE_LICENCE       "GPL"
#define D_TUNER_CONFIG_DRIVER_NAME          "mmtuner_drv"
#define D_TUNER_CONFIG_PLATFORM_DRIVER_NAME "mmtuner_pdev"
#define D_TUNER_CONFIG_SYSFS_DEV_NAME       "mmtuner_pdev"
#define D_TUNER_CONFIG_CLASS_NAME           "mmtuner"
#ifdef CONFIG_ISDBT_TUNER_SMTEJ11X
#define D_TUNER_CONFIG_MATCH_TABLE          "sony,ej113"

#define D_TUNER_CONFIG_DRV_MAJOR              100
#define D_TUNER_CONFIG_DRV_MINOR              200
#define D_TUNER_POWER_CHECK_WAIT_US         10000
#define D_TUNER_POWER_CHECK_WAIT_RANGE_US   10100
#define D_TUNER_POWER_ON_WAIT_US            15000
#define D_TUNER_POWER_ON_WAIT_RANGE_US      15100
#define D_TUNER_RESET_ON_WAIT_US             5000
#define D_TUNER_RESET_ON_WAIT_RANGE_US       5100
#define D_TUNER_RESET_OFF_WAIT_US            1000
#define D_TUNER_RESET_OFF_WAIT_RANGE_US      1100
#define D_TUNER_POWER_OFF_WAIT_US            5000
#define D_TUNER_POWER_OFF_WAIT_RANGE_US      5100
#define D_TUNER_ANT_SWITCH_ON                   1
#define D_TUNER_ANT_SWITCH_OFF                  0
#define D_TUNER_I2C_ADAPTER_ID                 11
#else
#define D_TUNER_CONFIG_MATCH_TABLE          "sony,ej121"

#define D_TUNER_CONFIG_DRV_MAJOR             100
#define D_TUNER_CONFIG_DRV_MINOR             200
#define D_TUNER_POWER_ON_WAIT_US            3000
#define D_TUNER_POWER_ON_WAIT_RANGE_US      3100
#define D_TUNER_RESET_ON_WAIT_US            1000
#define D_TUNER_RESET_ON_WAIT_RANGE_US      1100
#define D_TUNER_RESET_OFF_WAIT_US           1000
#define D_TUNER_RESET_OFF_WAIT_RANGE_US     1100
#define D_TUNER_ANT_SWITCH_ON                  1
#define D_TUNER_ANT_SWITCH_OFF                 0
#define D_TUNER_I2C_ADAPTER_ID                11
#endif

enum gpio_id {
	ANT_POWER_PIN = 0,
	ANT_SWITCH_PIN,
	TUNER_POWER_PIN,
	TUNER_RESET_PIN,
	TUNER_INT_PIN,
};

static char const * const gpio_rsrcs[] = {
	"ISDB-T ant power",
	"ISDB-T ant switch",
	"ISDB-T tuner power",
	"ISDB-T tuner reset",
	"ISDB-T tuner int",
};

enum TUNER_DRV_CTL {
	TUNER_DRV_CTL_POWON,
	TUNER_DRV_CTL_POWOFF,
	TUNER_DRV_CTL_RESET
};

enum TUNER_DRV_IRQ {
	TUNER_DRV_IRQ_NOT_DETECTED,
	TUNER_DRV_IRQ_DETECTED,
};

enum ANTMODE {
	ANTMODE_EARPHONE,
	ANTMODE_USB,
	ANTMODE_AUTO,
	ANTMODE_NOTUSE
};

struct tuner_drvdata {
	struct device *dev;
	struct device sysfs_dev;
	struct mutex mutex_lock;
	unsigned int gpios[ARRAY_SIZE(gpio_rsrcs)];
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

static enum gpio_id req_ids[] = {
	ANT_POWER_PIN,
	ANT_SWITCH_PIN,
	TUNER_POWER_PIN,
	TUNER_RESET_PIN,
	TUNER_INT_PIN,
};

static void isdbt_tunerpm_ant_power_control(struct tuner_drvdata *drvdata,
	int on)
{
	gpio_set_value_cansleep(drvdata->gpios[ANT_POWER_PIN], on);
}

static void isdbt_tunerpm_ant_switch_control(struct tuner_drvdata *drvdata,
	int on)
{
	gpio_set_value_cansleep(drvdata->gpios[ANT_SWITCH_PIN], on);
}

static void isdbt_tunerpm_power_control(struct tuner_drvdata *drvdata, int on)
{
	gpio_set_value_cansleep(drvdata->gpios[TUNER_POWER_PIN], on);
	isdbt_tunerpm_ant_power_control(drvdata, on);
}

static void isdbt_tunerpm_reset_control(struct tuner_drvdata *drvdata, int on)
{
	gpio_set_value_cansleep(drvdata->gpios[TUNER_RESET_PIN], on);
}

static int tunerpm_dev_init(struct platform_device *pdev,
	struct tuner_drvdata *drvdata)
{
	int i, ret, gpio;
	unsigned int flags;
	struct device_node *of_node = pdev->dev.of_node;

	mutex_init(&drvdata->mutex_lock);

	for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++) {
		gpio = of_get_gpio_flags(of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			ret = -EINVAL;
			goto error_gpio;
		}
		drvdata->gpios[i] = gpio;
	}

	for (i = 0; i < ARRAY_SIZE(req_ids); i++) {
		ret = gpio_request(drvdata->gpios[req_ids[i]],
				gpio_rsrcs[req_ids[i]]);
		if (ret)
			goto error_gpio_request;
	}

	isdbt_tunerpm_power_control(drvdata, 0);
	isdbt_tunerpm_reset_control(drvdata, 0);
	isdbt_tunerpm_ant_switch_control(drvdata, 0);

	return 0;

error_gpio_request:
	for (i--; 0 <= i; i--)
		gpio_free(drvdata->gpios[req_ids[i]]);
error_gpio:
	return ret;
}

static void tunerpm_dev_finalize(struct tuner_drvdata *drvdata)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(req_ids); i++)
		gpio_free(drvdata->gpios[req_ids[i]]);
}

static int tuner_drv_ctl_power(struct tuner_drvdata *drvdata, int data)
{
	switch (data) {
	case TUNER_DRV_CTL_POWON:
		mutex_lock(&drvdata->mutex_lock);
		isdbt_tunerpm_power_control(drvdata, 1);
		mutex_unlock(&drvdata->mutex_lock);
		usleep_range(D_TUNER_POWER_ON_WAIT_US,
			D_TUNER_POWER_ON_WAIT_RANGE_US);
		break;
	case TUNER_DRV_CTL_RESET:
		i2c_lock_adapter(drvdata->adap);
		mutex_lock(&drvdata->mutex_lock);
		isdbt_tunerpm_reset_control(drvdata, 1);
		mutex_unlock(&drvdata->mutex_lock);
		usleep_range(D_TUNER_RESET_ON_WAIT_US,
			D_TUNER_RESET_ON_WAIT_RANGE_US);
		i2c_unlock_adapter(drvdata->adap);
		break;
	case TUNER_DRV_CTL_POWOFF:
		mutex_lock(&drvdata->mutex_lock);
		usleep_range(D_TUNER_RESET_OFF_WAIT_US,
			D_TUNER_RESET_OFF_WAIT_RANGE_US);
		isdbt_tunerpm_reset_control(drvdata, 0);
#ifdef CONFIG_ISDBT_TUNER_SMTEJ11X
		usleep_range(D_TUNER_POWER_OFF_WAIT_US,
			D_TUNER_POWER_OFF_WAIT_RANGE_US);
#endif
		isdbt_tunerpm_power_control(drvdata, 0);
		mutex_unlock(&drvdata->mutex_lock);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int tuner_drv_ant_switch(struct tuner_drvdata *drvdata, int ant_mode)
{
	switch (ant_mode) {
	case ANTMODE_EARPHONE:
		isdbt_tunerpm_ant_switch_control(drvdata,
			D_TUNER_ANT_SWITCH_ON);
		break;
	case ANTMODE_USB:
		isdbt_tunerpm_ant_switch_control(drvdata,
			D_TUNER_ANT_SWITCH_OFF);
		break;
	case ANTMODE_AUTO:
	case ANTMODE_NOTUSE:
		isdbt_tunerpm_ant_switch_control(drvdata,
			D_TUNER_ANT_SWITCH_ON);
		break;
	default:
		return -EINVAL;
	}
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

static int tuner_drv_set_interrupt(int int_num)
{
	int ret;

	tnr_dev.irq_num = gpio_to_irq(int_num);
	ret = request_threaded_irq(tnr_dev.irq_num, tuner_interrupt,
		NULL, IRQF_TRIGGER_RISING, D_TUNER_CONFIG_CLASS_NAME, NULL);
	if (ret) {
		gpio_free(int_num);
		return -EINVAL;
	}
	return 0;
}

static void tuner_drv_release_interrupt(void)
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

	if (!value) {
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
	struct tuner_drvdata *drvdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (!value) {
		if (tuner_drv_set_interrupt(drvdata->gpios[TUNER_INT_PIN]))
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
		if (!file)
			return -ESRCH;
		dev = file->private_data;
	}

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
	struct device *dev = NULL;
	struct tuner_drvdata *drvdata;

	tnr_dev.mmtuner_device = platform_device_alloc(
		D_TUNER_CONFIG_CLASS_NAME, -1);

	if (!tnr_dev.mmtuner_device) {
		ret = -ENOMEM;
		goto err_platform_device_alloc;
	}

	ret = platform_device_add(tnr_dev.mmtuner_device);
	if (ret)
		goto err_platform_device_add;

	tnr_dev.device_class = class_create(THIS_MODULE,
		D_TUNER_CONFIG_CLASS_NAME);
	if (IS_ERR(tnr_dev.device_class)) {
		ret = PTR_ERR(tnr_dev.device_class);
		goto err_class_create;
	}

	dev = device_create(tnr_dev.device_class, NULL,
		MKDEV(D_TUNER_CONFIG_DRV_MAJOR, D_TUNER_CONFIG_DRV_MINOR),
		NULL, D_TUNER_CONFIG_CLASS_NAME);

	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto err_device_create;
	}

	drvdata = kzalloc(sizeof(struct tuner_drvdata), GFP_KERNEL);
	if (!drvdata) {
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	drvdata->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, drvdata);
	drvdata->sysfs_dev.init_name = D_TUNER_CONFIG_SYSFS_DEV_NAME;
	dev_set_drvdata(&drvdata->sysfs_dev, drvdata);
	ret = device_register(&drvdata->sysfs_dev);
	if (ret)
		goto err_set_dev;

	ret = register_chrdev(D_TUNER_CONFIG_DRV_MAJOR,
		D_TUNER_CONFIG_DRIVER_NAME, &tuner_file_operations);
	if (ret < 0)
		goto err_register_device;

	drvdata->adap = i2c_get_adapter(D_TUNER_I2C_ADAPTER_ID);
	if (!drvdata->adap)
		goto err_i2c_get_adapter;

	ret = tunerpm_dev_init(pdev, drvdata);
	if (ret)
		goto err_gpio_init;

	tnr_dev.open_cnt = 0;

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

	mutex_init(&tnr_dev.g_tuner_mutex);
	init_waitqueue_head(&tnr_dev.irq_wait_q);

#ifdef CONFIG_ISDBT_TUNER_SMTEJ11X
	ret = tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWON);
	if (ret)
		goto err_check_power;
	ret = tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_RESET);
	if (ret) {
		tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWOFF);
		goto err_check_power;
	}
	usleep_range(D_TUNER_POWER_CHECK_WAIT_US,
		D_TUNER_POWER_CHECK_WAIT_RANGE_US);
	tuner_drv_ctl_power(drvdata, TUNER_DRV_CTL_POWOFF);
#endif

	if (tuner_drv_set_interrupt(drvdata->gpios[TUNER_INT_PIN]))
		goto err_irq_set;

	return 0;

err_irq_set:
#ifdef CONFIG_ISDBT_TUNER_SMTEJ11X
err_check_power:
#endif
err_create_file:
err_gpio_init:
	i2c_put_adapter(drvdata->adap);
err_i2c_get_adapter:
	unregister_chrdev(D_TUNER_CONFIG_DRV_MAJOR,
		D_TUNER_CONFIG_DRIVER_NAME);
err_register_device:
	device_unregister(&drvdata->sysfs_dev);
err_set_dev:
	kzfree(drvdata);
err_alloc_data:
	device_destroy(tnr_dev.device_class, MKDEV(D_TUNER_CONFIG_DRV_MAJOR,
		D_TUNER_CONFIG_DRV_MINOR));
err_device_create:
	class_destroy(tnr_dev.device_class);
err_class_create:
	platform_device_del(tnr_dev.mmtuner_device);
err_platform_device_add:
	platform_device_put(tnr_dev.mmtuner_device);
err_platform_device_alloc:
	return ret;
}

static int __devexit tuner_remove(struct platform_device *pdev)
{
	struct tuner_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	tunerpm_dev_finalize(drvdata);
	tuner_drv_release_interrupt();
	i2c_put_adapter(drvdata->adap);
	unregister_chrdev(D_TUNER_CONFIG_DRV_MAJOR,
		D_TUNER_CONFIG_DRIVER_NAME);

	return 0;
}

static void tuner_shutdown(struct platform_device *pdev)
{
	device_destroy(tnr_dev.device_class, MKDEV(D_TUNER_CONFIG_DRV_MAJOR,
		D_TUNER_CONFIG_DRV_MINOR));
	class_destroy(tnr_dev.device_class);
	platform_device_unregister(tnr_dev.mmtuner_device);
}

#ifdef CONFIG_ISDBT_TUNER_SMTEJ11X
static struct of_device_id ej113_match_table[] = {
#else
static struct of_device_id ej121_match_table[] = {
#endif
{	.compatible = D_TUNER_CONFIG_MATCH_TABLE,
},
{}
};

static struct platform_driver mmtuner_driver = {
	.probe  = tuner_probe,
	.remove = __exit_p(tuner_remove),
	.shutdown = tuner_shutdown,
	.driver = {
		.name = D_TUNER_CONFIG_PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_ISDBT_TUNER_SMTEJ11X
		.of_match_table = ej113_match_table,
#else
		.of_match_table = ej121_match_table,
#endif
	},
};

static int __init tuner_drv_start(void)
{
	return platform_driver_register(&mmtuner_driver);
}

static void __exit tuner_drv_end(void)
{
	platform_driver_unregister(&mmtuner_driver);
}

MODULE_LICENSE(D_TUNER_CONFIG_MODULE_LICENCE);
MODULE_DESCRIPTION(D_TUNER_CONFIG_MODULE_NAME);

module_init(tuner_drv_start);
module_exit(tuner_drv_end);
