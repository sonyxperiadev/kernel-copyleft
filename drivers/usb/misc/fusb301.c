/*
 * fusb301.c (v1.1) -- FUSB301 USB TYPE-C Controller device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Copyright (C) 2016 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/i2c/fusb301.h>

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_DEV_ID			0x01
#define REG_MOD				0x02
#define REG_CON				0x03
#define REG_MAN				0x04
#define REG_RST				0x05
#define REG_MSK				0x10
#define REG_STAT			0x11
#define REG_TYPE			0x12
#define REG_INT				0x13

/******************************************************************************
* Register bits
******************************************************************************/
/*	  REG_DEV_ID (0x01)    */
#define ID_REV				0x0F
#define ID_VER_SHIFT			4
#define ID_VER				(0x0F << ID_VER_SHIFT)

/*    REG_MOD (0x02)    */
#define MOD_SRC				0x01
#define MOD_SRC_ACC_SHIFT		1
#define MOD_SRC_ACC			(0x01 << MOD_SRC_ACC_SHIFT)
#define MOD_SNK_SHIFT			2
#define MOD_SNK				(0x01 << MOD_SNK_SHIFT)
#define MOD_SNK_ACC_SHIFT		3
#define MOD_SNK_ACC			(0x01 << MOD_SNK_ACC_SHIFT)
#define MOD_DRP_SHIFT			4
#define MOD_DRP				(0x01 << MOD_DRP_SHIFT)
#define MOD_DRP_ACC_SHIFT		5
#define MOD_DRP_ACC			(0x01 << MOD_DRP_ACC_SHIFT)

/*    REG_CON (0x03)    */
#define CON_INT_MSK			0x01
#define CON_HOST_CUR_SHIFT		1
#define CON_HOST_CUR			(0x03 << CON_HOST_CUR_SHIFT)
#define CON_DRP_TGL_SHIFT		4
#define CON_DRP_TGL			(0x03 << CON_DRP_TGL_SHIFT)

/*    REG_MAN (0x04)    */
#define MAN_ERR_REC			0x01
#define MAN_DIS_SHIFT			1
#define MAN_DIS				(0x01 << MAN_DIS_SHIFT)
#define MAN_UNATT_SRC_SHIFT		2
#define MAN_UNATT_SRC			(0x01 << MAN_UNATT_SRC_SHIFT)
#define MAN_UNATT_SNK_SHIFT		3
#define MAN_UNATT_SNK			(0x01 << MAN_UNATT_SNK_SHIFT)

/*    REG_RST (0x05)    */
#define RST_SW				0x01

/*    REG_MSK (0x10)    */
#define MSK_ATTACH			0x01
#define MSK_DETACH_SHIFT		1
#define MSK_DETACH			(0x01 << MSK_DETACH_SHIFT)
#define MSK_BC_LVL_SHIFT		2
#define MSK_BC_LVL			(0x01 << MSK_BC_LVL_SHIFT)
#define MSK_ACC_CHG_SHIFT		3
#define MSK_ACC_CHG			(0x01 << MSK_ACC_CHG_SHIFT)

/*    REG_STAT (0x11)    */
#define STAT_ATTACH			0x01
#define STAT_BC_LVL_SHIFT		1
#define STAT_BC_LVL			(0x03 << STAT_BC_LVL_SHIFT)
#define STAT_VBUS_OK_SHIFT		3
#define STAT_VBUS_OK			(0x01 << STAT_VBUS_OK_SHIFT)
#define STAT_ORIENT_SHIFT		3
#define STAT_ORIENT			(0x03 << STAT_ORIENT_SHIFT)

/*    REG_TYPE (0x12)    */
#define TYPE_AUDIO_ACC			0x01
#define TYPE_DBG_ACC_SHIFT		1
#define TYPE_DBG_ACC			(0x01 << TYPE_DBG_ACC_SHIFT)
#define TYPE_PWR_ACC_SHIFT		2
#define TYPE_PWR_ACC			(0x01 << TYPE_PWR_ACC_SHIFT)
#define TYPE_SRC_SHIFT			3
#define TYPE_SRC			(0x01 << TYPE_SRC_SHIFT)
#define TYPE_SNK_SHIFT			4
#define TYPE_SNK			(0x01 << TYPE_SNK_SHIFT)

/*    REG_INT (0x13)    */
#define INT_ATTACH			0x01
#define INT_DETACH_SHIFT		1
#define INT_DETACH			(0x01 << INT_DETACH_SHIFT)
#define INT_BC_LVL_SHIFT		2
#define INT_BC_LVL			(0x01 << INT_BC_LVL_SHIFT)
#define INT_ACC_CHG_SHIFT		3
#define INT_ACC_CHG			(0x01 << INT_ACC_CHG_SHIFT)

/******************************************************************************/
enum fusb301_state {
	FUSB301_UNATTACHED_DRP = 0,	/* with Try.SNK */
	FUSB301_UNATTACHED_SNK,
	FUSB301_UNATTACHED_SRC,

	FUSB301_ATTACHED_SNK,
	FUSB301_ATTACHED_SRC,
	FUSB301_ATTACHED_DEBUG,
	FUSB301_ATTACHED_AUDIO
};

struct fusb301_info {
	struct i2c_client		*i2c;
	struct device			*dev_t;
	struct fusb301_platform_data	*pdata;
	struct mutex			mutex;
	struct class			*fusb_class;
	int				irq;
	enum fusb301_type		fusb_type;
	enum fusb301_state		state;
	struct work_struct		cbl_det_work;
	int				irq_cbl_det;
};

enum fusb301_drp_toggle {
	FUSB301_TOGGLE_SNK35_SRC15 = 0,	/* default */
	FUSB301_TOGGLE_SNK30_SRC20,
	FUSB301_TOGGLE_SNK25_SRC25,
	FUSB301_TOGGLE_SNK20_SRC30,
};

enum fusb301_host_cur {
	FUSB301_HOST_CUR_NO = 0,	/* no current */
	FUSB301_HOST_CUR_80,		/* default USB */
	FUSB301_HOST_CUR_180,		/* 1.5A */
	FUSB301_HOST_CUR_330,		/* 3A */
};

enum fusb301_orient {
	FUSB301_ORIENT_NO_CONN = 0,
	FUSB301_ORIENT_CC1_CC,
	FUSB301_ORIENT_CC2_CC,
	FUSB301_ORIENT_FAULT
};

enum fusb301_config_modes {
	FUSB301_MODE_SRC = 0,
	FUSB301_MODE_SRC_ACC,
	FUSB301_MODE_SNK,
	FUSB301_MODE_SNK_ACC,
	FUSB301_MODE_DRP,
	FUSB301_MODE_DRP_ACC
};

#define TTRYTO_EXP_TIME			600
#define TCCDEBOUNCEMAX_TIME		200

static const char *fusb301_get_state_string(enum fusb301_state state)
{
	const char *ret;

	switch (state) {
	case FUSB301_UNATTACHED_DRP:
		ret = "FUSB301_UNATTACHED_DRP";
		break;
	case FUSB301_UNATTACHED_SNK:
		ret = "FUSB301_UNATTACHED_SNK";
		break;
	case FUSB301_UNATTACHED_SRC:
		ret = "FUSB301_UNATTACHED_SRC";
		break;
	case FUSB301_ATTACHED_SNK:
		ret = "FUSB301_ATTACHED_SNK";
		break;
	case FUSB301_ATTACHED_SRC:
		ret = "FUSB301_ATTACHED_SRC";
		break;
	case FUSB301_ATTACHED_DEBUG:
		ret = "FUSB301_ATTACHED_DEBUG";
		break;
	case FUSB301_ATTACHED_AUDIO:
		ret = "FUSB301_ATTACHED_AUDIO";
		break;
	default:
		ret = "UNKNOWN";
		break;
	}
	return ret;
}

static int fusb301_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct device *cdev = &i2c->dev;
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		dev_err(cdev, "%s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int fusb301_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct device *cdev = &i2c->dev;
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0)
		dev_err(cdev, "%s:reg(0x%x), ret(%d)\n",
				__func__, reg, ret);

	return ret;
}

static int fusb301_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));

		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	return ret;
}

static ssize_t show_current_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fusb301_info *info = dev_get_drvdata(dev);

	switch (info->fusb_type) {
	case FUSB301_TYPE_AUDIO:
		return snprintf(buf, PAGE_SIZE, "FUSB301_TYPE_AUDIO\n");
	case FUSB301_TYPE_DEBUG:
		return snprintf(buf, PAGE_SIZE, "FUSB301_TYPE_DEBUG\n");
	case FUSB301_TYPE_POWER_ACC:
		return snprintf(buf, PAGE_SIZE, "FUSB301_TYPE_POWER_ACC\n");
	case FUSB301_TYPE_SOURCE:
		return snprintf(buf, PAGE_SIZE, "FUSB301_SOURCE\n");
	case FUSB301_TYPE_SINK:
		return snprintf(buf, PAGE_SIZE, "FUSB301_TYPE_SINK\n");
	default:
		return snprintf(buf, PAGE_SIZE, "TYPE ERROR\n");
	}
}

static ssize_t config_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	u8 rdata;
	struct fusb301_info *info = dev_get_drvdata(dev);
	struct device *cdev = &info->i2c->dev;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == FUSB301_MODE_SRC)
		fusb301_write_reg(info->i2c, REG_MOD, MOD_SRC);
	else if (data == FUSB301_MODE_SRC_ACC)
		fusb301_write_reg(info->i2c, REG_MOD, MOD_SRC_ACC);
	else if (data == FUSB301_MODE_SNK)
		fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK);
	else if (data == FUSB301_MODE_SNK_ACC)
		fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK_ACC);
	else if (data == FUSB301_MODE_DRP)
		fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP);
	else if (data == FUSB301_MODE_DRP_ACC)
		fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP_ACC);
	else
		dev_err(cdev, "%s: Argument is not match!!\n", __func__);

	fusb301_read_reg(info->i2c, REG_MOD, &rdata);
	dev_dbg(cdev, "%s: REG_MOD(0x%02x)\n", __func__, rdata);

	return count;
}

static DEVICE_ATTR(type, S_IRUGO, show_current_type, NULL);
static DEVICE_ATTR(mode, S_IWUSR, NULL, config_mode);

static const char *fusb301_get_type_string(enum fusb301_type type)
{
	const char *ret;

	switch (type) {
	case FUSB301_TYPE_NONE:
		ret = "NONE";
		break;
	case FUSB301_TYPE_AUDIO:
		ret = "AUDIO_ACC";
		break;
	case FUSB301_TYPE_DEBUG:
		ret = "DEBUG_ACC";
		break;
	case FUSB301_TYPE_POWER_ACC:
		ret = "POWER_ACC";
		break;
	case FUSB301_TYPE_SOURCE:
		ret = "SOURCE";
		break;
	case FUSB301_TYPE_SINK:
		ret = "SINK";
		break;
	default:
		ret = "UNKNOWN";
	}

	return ret;
}

static enum fusb301_type fusb301_get_type(struct fusb301_info *info)
{
	u8 rdata;
	enum fusb301_type ret;

	fusb301_read_reg(info->i2c, REG_TYPE, &rdata);

	if (rdata & TYPE_AUDIO_ACC)
		ret = FUSB301_TYPE_AUDIO;
	else if (rdata & TYPE_DBG_ACC)
		ret = FUSB301_TYPE_DEBUG;
	else if (rdata & TYPE_PWR_ACC)
		ret = FUSB301_TYPE_POWER_ACC;
	else if (rdata & TYPE_SRC)
		ret = FUSB301_TYPE_SOURCE;
	else if (rdata & TYPE_SNK)
		ret = FUSB301_TYPE_SINK;
	else
		ret = FUSB301_TYPE_NONE;

	return ret;
}

static void fusb301_check_regs(struct fusb301_info *info)
{
	struct device *cdev = &info->i2c->dev;
	u8 type, stat, mod;

	fusb301_read_reg(info->i2c, REG_TYPE, &type);
	fusb301_read_reg(info->i2c, REG_STAT, &stat);
	fusb301_read_reg(info->i2c, REG_MOD, &mod);
	dev_dbg(cdev, "%s: REG_TYPE=0x%02x, REG_STAT=0x%02x, REG_MOD=0x%02x\n",
						__func__, type, stat, mod);
}

static void fusb301_start_snk(struct fusb301_info *info);

static irqreturn_t fusb301_irq_thread(int irq, void *handle)
{
	u8 intr;
	struct fusb301_info *info = (struct fusb301_info *)handle;
	struct device *cdev = &info->i2c->dev;

	mutex_lock(&info->mutex);

	fusb301_read_reg(info->i2c, REG_INT, &intr);
	dev_dbg(cdev, "%s: REG_INT is 0x%02x\n", __func__, intr);

	if (intr & INT_ATTACH) {
		info->fusb_type = fusb301_get_type(info);
		dev_dbg(cdev, "%s: Attached interrupt! TYPE=%d is %s\n",
				__func__, info->fusb_type,
				fusb301_get_type_string(info->fusb_type));

		if (info->fusb_type == FUSB301_TYPE_SOURCE) {
			info->state = FUSB301_ATTACHED_SNK;
			fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK);
		} else if (info->fusb_type == FUSB301_TYPE_SINK) {
			info->state = FUSB301_ATTACHED_SRC;
			/* TO BE DEFINED */
			/* fusb301_write_reg(info->i2c, REG_MOD, MOD_SRC); */
		}
	} else if (intr & INT_DETACH) {
		dev_dbg(cdev, "%s: Detach interrupt!\n", __func__);
		fusb301_start_snk(info);
	} else if (intr & INT_BC_LVL) {
		dev_dbg(cdev, "%s: BC_LVL interrupt!\n", __func__);
	} else if (intr & INT_ACC_CHG) {
		dev_dbg(cdev, "%s: Accessory change interrupt!\n", __func__);
	} else {
		dev_dbg(cdev, "%s: weird interrupt!\n", __func__);
	}

	fusb301_check_regs(info);

	mutex_unlock(&info->mutex);
	return IRQ_HANDLED;
}

static void fusb301_start_drp(struct fusb301_info *info)
{
	info->fusb_type = FUSB301_TYPE_NONE;
	fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP);
	info->state = FUSB301_UNATTACHED_DRP;
}

static void fusb301_start_snk(struct fusb301_info *info)
{
	info->fusb_type = FUSB301_TYPE_NONE;
	fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK);
	info->state = FUSB301_UNATTACHED_SNK;
}

static void fusb301_initialization(struct fusb301_info *info)
{
	struct device *cdev = &info->i2c->dev;
	u8 rdata;

	fusb301_write_reg(info->i2c, REG_MSK, MSK_BC_LVL | MSK_ACC_CHG);
	fusb301_start_snk(info);
	fusb301_read_reg(info->i2c, REG_DEV_ID, &rdata);
	dev_dbg(cdev, "###############%s: REG_DEV_ID ver=0x%02x, rev=0x%02x\n",
		__func__, (rdata & ID_VER) >> ID_VER_SHIFT, rdata & ID_REV);
	/* unmask global interrupts */
	fusb301_update_reg(info->i2c, REG_CON, 0, CON_INT_MSK);
}

static void fusb301_cbl_det_work(struct work_struct *work)
{
	struct fusb301_info *info =
		container_of(work, struct fusb301_info, cbl_det_work);
	struct device *cdev = &info->i2c->dev;
	unsigned int val;

	val = gpio_get_value(info->pdata->cbl_det_gpio);
	dev_info(cdev, "%s : cable det %d, state=%s !!!!!!!!!!\n", __func__,
				val, fusb301_get_state_string(info->state));
	mutex_lock(&info->mutex);
	if (val)
		fusb301_start_snk(info);
	else if (info->state != FUSB301_ATTACHED_SNK)
		fusb301_start_drp(info);
	mutex_unlock(&info->mutex);
}

static irqreturn_t fusb301_irq_cbl_det(int irq, void *handle)
{
	struct fusb301_info *info = (struct fusb301_info *)handle;
	struct device *cdev = &info->i2c->dev;

	if (!info) {
		dev_err(cdev, "%s : called before init.\n", __func__);
		return IRQ_HANDLED;
	}

	schedule_work(&info->cbl_det_work);
	return IRQ_HANDLED;
}

static int fusb301_parse_dt(struct fusb301_info *info)
{
	struct device *cdev = &info->i2c->dev;
	struct device_node *dev_node = cdev->of_node;
	struct fusb301_platform_data *data = info->pdata;
	int rc = 0;

	data->irq_gpio = of_get_named_gpio(dev_node, "fusb301,irq_gpio", 0);
	if (data->irq_gpio < 0) {
		dev_err(cdev, "irq_gpio is not available\n");
		rc = data->irq_gpio;
		goto out;
	}

	data->cbl_det_gpio = of_get_named_gpio(dev_node,
						"fusb301,cbl_det-gpio", 0);
	if (data->cbl_det_gpio < 0) {
		dev_err(cdev, "cbl_det_gpio is not available\n");
		rc = data->cbl_det_gpio;
		goto out;
	}
out:
	return rc;
}

static int fusb301_init_gpio(struct fusb301_info *info)
{
	struct device *cdev = &info->i2c->dev;
	int rc = 0;

	/* Start to enable fusb301 Chip */
	if (gpio_is_valid(info->pdata->irq_gpio)) {
		rc = gpio_request_one(info->pdata->irq_gpio,
					GPIOF_DIR_IN, "fusb301_irq_gpio");
		if (rc)
			dev_err(cdev, "unable to request irq_gpio %d\n",
					info->pdata->irq_gpio);
	} else {
		dev_err(cdev, "irq_gpio %d is not valid\n",
					info->pdata->irq_gpio);
		rc = -EINVAL;
	}

	if (rc)
		return rc;

	/* Start to enable cable detection */
	if (gpio_is_valid(info->pdata->cbl_det_gpio)) {
		rc = gpio_request_one(info->pdata->cbl_det_gpio,
					GPIOF_DIR_IN, "fusb301_cbl_det_gpio");
		if (rc)
			dev_err(cdev, "unable to request cbl_det_gpio %d\n",
					info->pdata->cbl_det_gpio);
	} else {
		dev_err(cdev, "cbl_det_gpio %d is not valid\n",
					info->pdata->cbl_det_gpio);
		rc = -EINVAL;
	}

	return rc;
}

static void fusb301_free_gpio(struct fusb301_info *info)
{
	if (gpio_is_valid(info->pdata->cbl_det_gpio))
		gpio_free(info->pdata->cbl_det_gpio);
	if (gpio_is_valid(info->pdata->irq_gpio))
		gpio_free(info->pdata->irq_gpio);
}

static int fusb301_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret = 0;
	struct fusb301_info *info;
	struct device *cdev = &client->dev;

	info = devm_kzalloc(cdev, sizeof(struct fusb301_info), GFP_KERNEL);
	info->i2c = client;
	if (&client->dev.of_node) {
		struct fusb301_platform_data *data = devm_kzalloc(cdev,
			sizeof(struct fusb301_platform_data), GFP_KERNEL);
		if (!data) {
			ret = -ENOMEM;
			goto platform_data_failed;
		}
		info->pdata = data;
		ret = fusb301_parse_dt(info);
		if (ret) {
			dev_err(cdev, "can't parse dt\n");
			goto parse_dt_failed;
		}
	} else {
		info->pdata = client->dev.platform_data;
	}

	/* GPIO setting */
	ret = fusb301_init_gpio(info);
	if (ret) {
		dev_err(cdev, "fail to init gpio\n");
		goto parse_dt_failed;
	}

	info->irq = gpio_to_irq(info->pdata->irq_gpio);
	if (info->irq < 0) {
		dev_err(cdev, "could not register irq\n");
		ret = -ENXIO;
		goto get_irq_failed;
	}

	info->irq_cbl_det = gpio_to_irq(info->pdata->cbl_det_gpio);
	if (info->irq_cbl_det < 0) {
		dev_err(cdev, "could not register irq_cbl_det\n");
		ret = -ENXIO;
		goto get_irq_failed;
	}

	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	if (!i2c_check_functionality(client->adapter,
						I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(cdev, "%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto check_funcionality_failed;
	}

	info->fusb_class = class_create(THIS_MODULE, "type-c");
	info->dev_t = device_create(info->fusb_class, NULL, 0, NULL, "fusb301");
	device_create_file(info->dev_t, &dev_attr_type);
	device_create_file(info->dev_t, &dev_attr_mode);
	dev_set_drvdata(info->dev_t, info);

	INIT_WORK(&info->cbl_det_work, fusb301_cbl_det_work);

	fusb301_initialization(info);

	ret = devm_request_threaded_irq(cdev, info->irq, NULL,
				fusb301_irq_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"fusb301_irq", info);
	if (ret) {
		dev_err(cdev, "failed to request IRQ\n");
		goto request_irq_failed;
	}

	ret = enable_irq_wake(info->irq);
	if (ret < 0) {
		dev_err(cdev, "failed to enable wakeup src %d\n", ret);
		goto enable_irq_failed;
	}

	ret = devm_request_irq(cdev, info->irq_cbl_det,
				fusb301_irq_cbl_det,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"fusb301_cbl_det_irq", info);
	if (ret) {
		dev_err(cdev, "failed to request IRQ\n");
		goto request_irq_cbl_det_failed;
	}

	ret = enable_irq_wake(info->irq_cbl_det);
	if (ret < 0) {
		dev_err(cdev, "failed to enable wakeup src %d\n", ret);
		goto enable_irq_cbl_det_failed;
	}

	return 0;

enable_irq_cbl_det_failed:
	devm_free_irq(cdev, info->irq_cbl_det, info);
request_irq_cbl_det_failed:
	disable_irq_wake(info->irq);
enable_irq_failed:
	devm_free_irq(cdev, info->irq, info);
request_irq_failed:
	device_destroy(info->fusb_class, 0);
	class_destroy(info->fusb_class);
	device_remove_file(info->dev_t, &dev_attr_type);
check_funcionality_failed:
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
get_irq_failed:
	fusb301_free_gpio(info);
parse_dt_failed:
	if (&client->dev.of_node && info->pdata)
		devm_kfree(cdev, info->pdata);
platform_data_failed:
	devm_kfree(cdev, info);

	return ret;

}


static int fusb301_remove(struct i2c_client *client)
{
	struct fusb301_info *info = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;

	if (info->irq_cbl_det) {
		disable_irq_wake(info->irq_cbl_det);
		devm_free_irq(cdev, info->irq_cbl_det, info);
	}
	if (info->irq) {
		disable_irq_wake(info->irq);
		devm_free_irq(cdev, info->irq, info);
	}
	device_remove_file(info->dev_t, &dev_attr_type);
	device_destroy(info->fusb_class, 0);
	class_destroy(info->fusb_class);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
	fusb301_free_gpio(info);
	if (&client->dev.of_node && info->pdata)
		devm_kfree(cdev, info->pdata);
	devm_kfree(cdev, info);

	return 0;
}


static int fusb301_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int fusb301_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id fusb301_i2c_id[] = {
	{ "fusb301", 0 },
	{ }
};

static struct i2c_driver fusb301_i2c_driver = {
	.driver = {
		.name = "fusb301",
		.owner = THIS_MODULE,
	},
	.probe		= fusb301_probe,
	.remove		= fusb301_remove,
	.suspend	= fusb301_suspend,
	.resume		= fusb301_resume,
	.id_table	= fusb301_i2c_id,
};

static __init int fusb301_i2c_init(void)
{
	return i2c_add_driver(&fusb301_i2c_driver);
}

static __exit void fusb301_i2c_exit(void)
{
	i2c_del_driver(&fusb301_i2c_driver);
}

module_init(fusb301_i2c_init);
module_exit(fusb301_i2c_exit);

MODULE_AUTHOR("chris.jeong@fairchildsemi.com");
MODULE_DESCRIPTION("I2C bus driver for FUSB301 USB Type-C");
MODULE_LICENSE("GPL v2");
