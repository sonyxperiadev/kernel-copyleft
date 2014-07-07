/* drivers/input/misc/lsm303dlhc_acc_lt.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 * Copyright (c) 2012 Sony Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *         Stefan Karlsson <stefan3.karlsson@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

/*#define DEBUG*/

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/lsm303dlhc_acc_lt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/workqueue.h>

enum scale_adjust {
	SCALE_ADJ_2G  = 1,
	SCALE_ADJ_4G  = 2,
	SCALE_ADJ_8G  = 4,
	SCALE_ADJ_16G = 12,
};

enum reg_ctl_1 {
	REG_CTL1_ADDR = 0x20,
	PWR_OFF = 0,
	LOW_PWR_MODE = (1 << 3),
	ODR_1000_MS =  (1 << 4),
	ODR__100_MS =  (2 << 4),
	ODR___40_MS =  (3 << 4),
	ODR___20_MS =  (4 << 4),
	ODR___10_MS =  (5 << 4),
	ODR____5_MS =  (6 << 4),
	ODR_2500_US =  (7 << 4),
	ODR__617_US = ((8 << 4) | LOW_PWR_MODE),
	ODR__744_US =  (9 << 4),
	ODR__186_US = ((9 << 4) | LOW_PWR_MODE),
	ODR_MASK = (0x0f << 4),
	ENABLE_ALL_AXES = 7,
};

enum reg_ctl_2 {
	REG_CTL2_ADDR = 0x21,
	HP_NORMAL_RESET = (0 << 6),
	HP_REF          = (1 << 6),
	HP_NORAML       = (2 << 6),
	HP_AOUTORESET   = (3 << 6),
	HP_MASK = (3 << 6),
	HPCF1 = (1 << 5),
	HPCF2 = (1 << 4),
	FDS   = (1 << 3),
	HP_CLICK = (1 << 2),
	HPIS2 = (1 << 1),
	HPIS1 = (1 << 0),
};

enum reg_ctl_3 {
	REG_CTL3_ADDR = 0x22,
	I1_CLICK   = (1 << 7),
	I1_AOI1    = (1 << 6),
	I1_AOI2    = (1 << 5),
	I1_DRDY1   = (1 << 4),
	I1_DRDY2   = (1 << 3),
	I1_WTM     = (1 << 2),
	I1_OVERRUN = (1 << 1),
};

enum reg_ctl_4 {
	REG_CTL4_ADDR = 0x23,
	BDU       = (1 << 7),
	BLE_BE    = (1 << 6),
	BLE_LE    = (0 << 6),
	FSC__2G   = (0 << 4),
	FSC__4G   = (1 << 4),
	FSC__8G   = (2 << 4),
	FSC_16G   = (3 << 4),
	FSC_MASK  = (3 << 4),
	HRES      = (1 << 3),
	ST_NORMAL = (0 << 1),
	ST_TEST0  = (1 << 1),
	ST_TEST1  = (2 << 1),
	ST_MASK   = (3 << 1),
};

enum reg_ctl_5 {
	REG_CTL5_ADDR = 0x24,
	BOOT    = (1 << 7),
	FIFO_EN = (1 << 6),
	LIR_1   = (1 << 3),
	D4D_1   = (1 << 2),
};

enum reg_ctl_6 {
	REG_CTL6_ADDR = 0x25,
	I2_CLICK_EN   = (1 << 7),
	I2_INT1       = (1 << 6),
	BOOT_I2       = (1 << 4),
	INT_ACTIVE_LO = (1 << 1),
	INT_ACTIVE_HI = (0 << 1),
};

enum reg_status {
	REG_STATUS_ADDR = 0x27,
	ZYXOR = (1 << 7),
	ZOR   = (1 << 6),
	YOR   = (1 << 5),
	XOR   = (1 << 4),
	ZYXDA = (1 << 3),
	ZDA   = (1 << 2),
	YDA   = (1 << 1),
	XDA   = (1 << 0),
};

enum reg_out {
	AUTO_INCREMENT = 0x80,
	OUT_X = 0x28,
	OUT_Y = 0x2a,
	OUT_Z = 0x2c,
	OUT_XYZ = (OUT_X | AUTO_INCREMENT),
};

enum reg_fifo_ctl {
	REG_FIFO_CTL_ADDR = 0x2e,
	FIFO_BYPASS = (0 << 6),
	FIFO_FIFO   = (1 << 6),
	FIFO_STREAM = (2 << 6),
	FIFO_TRIG   = (3 << 6),
	FIFO_MASK   = (3 << 6),
	TRIGGER_I1  = (0 << 5),
	TRIGGER_I2  = (1 << 5),
	FIFO_THRES_MASK = (0x1f << 0),
};

enum reg_fifo_src {
	REG_FIFO_SRC_ADDR = 0x2f,
	FIFO_WTM        = (1 << 7),
	FIFO_OVR        = (1 << 6),
	FIFO_EMPTY      = (1 << 5),
	FIFO_LEVEL_MASK = (0x1f << 0),
};

enum reg_int1_cfg {
	REG_INT1_CFG_ADDR = 0x30,
	INT1_OR        = (0 << 6),
	INT1_6D_MOTION = (1 << 6),
	INT1_AND       = (2 << 6),
	INT1_6D_POS    = (3 << 6),
	INT1_TYPE_MASK = (3 << 6),
	INT1_6D_BIT    = (1 << 6),
	INT1_ZHE = (1 << 5),
	INT1_ZLE = (1 << 4),
	INT1_YHE = (1 << 3),
	INT1_YLE = (1 << 2),
	INT1_XHE = (1 << 1),
	INT1_XLE = (1 << 0),
	INT1_ALL_LO = (INT1_ZLE | INT1_YLE | INT1_XLE),
	INT1_ALL_HI = (INT1_ZHE | INT1_YHE | INT1_XHE),
};

enum reg_int1_src {
	REG_INT1_SRC_ADDR = 0x31,
	I1_SRC_IRQ = (1 << 6),
	I1_SRC_ZH  = (1 << 5),
	I1_SRC_ZL  = (1 << 4),
	I1_SRC_YH  = (1 << 3),
	I1_SRC_YL  = (1 << 2),
	I1_SRC_XH  = (1 << 1),
	I1_SRC_XL  = (1 << 0),
};

enum reg_int1_thres {
	REG_INT1_THRES_ADDR = 0x32,
	THS_MASK = 0x7f,
};

enum reg_int1_dur {
	REG_INT1_DUR_ADDR = 0x33,
	DUR_MASK = 0x7f,
};

enum setup_regs {
	SR_BASE_ADDR,
	SR_ODR_PWR,
	SR_HIPASS,
	SR_I1_CFG,
	SR_FSC,
	SR_PAD1,
	SR_PAD2,
};

#define MIN_POLLING_RATE_MS 1
#define ACC_RANGE_MG        16000
#define DSHIFT              4
#define AUTOSUSPEND_MS      50

struct lsm303dlhc_acc_lt_data {
	struct i2c_client *client;
	struct delayed_work work;
	struct workqueue_struct *wq;
	struct mutex lock;
	struct input_dev *input_dev;
	u8 scale;
	u8 ctl_regs_setup[5];
	int range;
	atomic_t poll_interval_ms;
	bool powered;
	int (*power)(struct device *dev,
			enum lsm303dlhc_acc_lt_power_state pwr_state);
};

#define LOCK(p) do { \
	dev_dbg(&(p)->client->dev, "%s: lock\n", __func__); \
	mutex_lock(&p->lock); \
} while (0)

#define UNLOCK(p) do { \
	dev_dbg(&(p)->client->dev, "%s: unlock\n", __func__); \
	mutex_unlock(&p->lock); \
} while (0)

#define INIT_LOCK(p) mutex_init(&p->lock);

/**
 * lsm303dlhc_acc_lt_i2c_read - read data from chip
 * @acc: Pointer to driver data structure
 * @buf: Byte array into which data will be read;
 * @len: number of bytes to be read from chip
*/
static int lsm303dlhc_acc_lt_i2c_read(struct lsm303dlhc_acc_lt_data *acc,
						u8 reg, void *buf, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	err = i2c_transfer(acc->client->adapter, msgs, 2);
	if (err != 2) {
		dev_err(&acc->client->dev, "%s: i2c read error %d (reg 0x%x)\n",
		       __func__, err, reg);
		return -EIO;
	}
	return 0;
}

/**
 * lsm303dlhc_acc_lt_i2c_write - write data to chip
 * @acc: Pointer to driver data structure
 * @buf: Byte array which will be written including register offset
 *       in the first byte;
 * @len: number of bytes to be written including register offset
*/
static int lsm303dlhc_acc_lt_i2c_write(struct lsm303dlhc_acc_lt_data *acc,
				   void *buf, int len)
{
	int err;
	struct i2c_msg msg = {
		.addr = acc->client->addr,
		.flags = acc->client->flags & I2C_M_TEN,
		.len = len,
		.buf = buf,
	};

	err = i2c_transfer(acc->client->adapter, &msg, 1);
	if (err != 1) {
		dev_err(&acc->client->dev,
			"%s: i2c write error %d (reg 0x%x)\n",
		       __func__, err, *(u8 *)buf & ~AUTO_INCREMENT);
		return -EIO;
	}
	return 0;
}

static int lsm303dlhc_acc_lt_wr_reg(struct lsm303dlhc_acc_lt_data *acc,
				   u8 reg, u8 data)
{
	u8 x[2] = {reg, data};
	int rc = lsm303dlhc_acc_lt_i2c_write(acc, &x, sizeof(x));
	if (!rc)
		dev_dbg(&acc->client->dev, "%s: 0x%02x -> reg[%02x]\n",
			   __func__, data, reg);
	return rc;
}

static void lsm303dlhc_acc_lt_set_range(struct lsm303dlhc_acc_lt_data *acc)
{
	u8 fsc;

	switch (acc->range) {
	case 16:
		fsc = FSC_16G;
		acc->scale = SCALE_ADJ_16G;
		break;
	case 8:
		fsc = FSC__8G;
		acc->scale = SCALE_ADJ_8G;
		break;
	case 4:
		fsc = FSC__4G;
		acc->scale = SCALE_ADJ_4G;
		break;
	default:
		fsc = FSC__2G;
		acc->scale = SCALE_ADJ_2G;
		break;
	}
	acc->ctl_regs_setup[SR_FSC] = (acc->ctl_regs_setup[SR_FSC] & ~FSC_MASK)
									 | fsc;
}

static void lsm303dlhc_acc_lt_report(struct lsm303dlhc_acc_lt_data *acc)
{
	int err;
	u8 data[6];
	s16 *accel = (s16 *)data;
	u8 scale = acc->scale;

	err = lsm303dlhc_acc_lt_i2c_read(acc, OUT_XYZ, data, sizeof(data));
	if (err)
		return;

	dev_dbg(&acc->client->dev, "%s: ACCEL: %6d, %6d, %6d\n",
			__func__,  (accel[0] >> DSHIFT) * scale,
			(accel[1] >> DSHIFT) * scale,
			(accel[2] >> DSHIFT) * scale);

	input_report_abs(acc->input_dev, ABS_X, (accel[0] >> DSHIFT) * scale);
	input_report_abs(acc->input_dev, ABS_Y, (accel[1] >> DSHIFT) * scale);
	input_report_abs(acc->input_dev, ABS_Z, (accel[2] >> DSHIFT) * scale);
	input_sync(acc->input_dev);
}

static void lsm303dlhc_acc_lt_poll_func(struct work_struct *work)
{
	struct lsm303dlhc_acc_lt_data *acc =
			container_of((struct delayed_work *)work,
			struct lsm303dlhc_acc_lt_data, work);
	struct timespec start, end;
	int rc, ms;

	getnstimeofday(&start);


	rc = pm_runtime_get_sync(&acc->client->dev);
	if (rc < 0) {
		dev_err(&acc->client->dev, "Failed to runtime resume; rc = %d",
									rc);
		pm_runtime_mark_last_busy(&acc->client->dev);
		pm_runtime_put_noidle(&acc->client->dev);
		return;
	}
	lsm303dlhc_acc_lt_report(acc);
	pm_runtime_mark_last_busy(&acc->client->dev);
	pm_runtime_put_autosuspend(&acc->client->dev);

	getnstimeofday(&end);
	ms = atomic_read(&acc->poll_interval_ms) - (end.tv_sec -
				start.tv_sec)*MSEC_PER_SEC - (end.tv_nsec -
						start.tv_nsec)/NSEC_PER_MSEC;
	if (ms < 0)
		ms = 0;
	queue_delayed_work(acc->wq, &acc->work, msecs_to_jiffies(ms));
}

static void lsm303dlhc_acc_lt_device_stop(struct lsm303dlhc_acc_lt_data *acc)
{
	dev_dbg(&acc->client->dev, "%s: internal power-off\n", __func__);

	lsm303dlhc_acc_lt_wr_reg(acc, REG_CTL1_ADDR, ENABLE_ALL_AXES | PWR_OFF);
}

static int lsm303dlhc_acc_lt_device_start(struct lsm303dlhc_acc_lt_data *acc)
{
	int e;
	dev_dbg(&acc->client->dev, "%s: internal power-on\n", __func__);

	e = lsm303dlhc_acc_lt_i2c_write(acc, acc->ctl_regs_setup,
		ARRAY_SIZE(acc->ctl_regs_setup));

	if (e)
		dev_err(&acc->client->dev, "%s: unable to start, err %d\n",
								__func__, e);
	return e;
}

#ifdef CONFIG_PM_RUNTIME
static int lsm303dlhc_acc_lt_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_acc_lt_data *acc = i2c_get_clientdata(client);

	LOCK(acc);
	if (acc->powered) {
		lsm303dlhc_acc_lt_device_stop(acc);
		if (!acc->power(&client->dev, LSM303DLHC_LT_PWR_OFF))
			acc->powered = false;
	}
	UNLOCK(acc);
	return 0;
}

static int lsm303dlhc_acc_lt_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_acc_lt_data *acc = i2c_get_clientdata(client);

	LOCK(acc);
	if (!acc->powered) {
		if (!acc->power(&client->dev, LSM303DLHC_LT_PWR_ON))
			acc->powered = true;
	}
	lsm303dlhc_acc_lt_device_start(acc);
	UNLOCK(acc);
	usleep_range(12000, 12000);
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int lsm303dlhc_acc_lt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_acc_lt_data *acc = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&acc->work);
	return 0;
}

static int lsm303dlhc_acc_lt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_acc_lt_data *acc = i2c_get_clientdata(client);

	if (acc->input_dev->users)
		queue_delayed_work(acc->wq, &acc->work,
			msecs_to_jiffies(atomic_read(&acc->poll_interval_ms)));
	return 0;
}
#endif

static int lsm303dlhc_acc_lt_open(struct input_dev *dev)
{
	struct lsm303dlhc_acc_lt_data *acc = input_get_drvdata(dev);

	queue_delayed_work(acc->wq, &acc->work,
		msecs_to_jiffies(atomic_read(&acc->poll_interval_ms)));
	return 0;
}

static void lsm303dlhc_acc_lt_close(struct input_dev *dev)
{
	struct lsm303dlhc_acc_lt_data *acc = input_get_drvdata(dev);

	cancel_delayed_work_sync(&acc->work);
}

static ssize_t attr_set_poll_rate(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int e = 0;
	struct lsm303dlhc_acc_lt_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (interval_ms < 1 || interval_ms > 10000)
		return -EINVAL;

	atomic_set(&acc->poll_interval_ms, interval_ms);
	return e ? e : size;
}

static ssize_t attr_get_poll_rate(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct lsm303dlhc_acc_lt_data *acc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
					atomic_read(&acc->poll_interval_ms));
}

static ssize_t attr_set_filt_config(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lsm303dlhc_acc_lt_data *acc = dev_get_drvdata(dev);
	unsigned long hpf;
	int e = 0;

	if (kstrtoul(buf, 16, &hpf))
		return -EINVAL;

	hpf &= 0xff;
	LOCK(acc);
	acc->ctl_regs_setup[SR_HIPASS] = hpf;
	if (acc->powered)
		e = lsm303dlhc_acc_lt_wr_reg(acc, REG_CTL2_ADDR, hpf);
	UNLOCK(acc);
	return e ? e : size;
}


static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0644, attr_get_poll_rate, attr_set_poll_rate),
	__ATTR(hpf_config, 0200, NULL, attr_set_filt_config),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;
error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: Unable to create interface\n", __func__);
	return -EIO;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int lsm303dlhc_acc_lt_power_stub(struct device *dev,
				enum lsm303dlhc_acc_lt_power_state pwr_state)
{
	dev_dbg(dev, "%s: power state %d\n", __func__, pwr_state);
	return 0;
}

static int __devinit lsm303dlhc_acc_lt_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct lsm303dlhc_acc_lt_platform_data *pdata =
					client->dev.platform_data;
	struct lsm303dlhc_acc_lt_data *acc;
	int result = 0;

	dev_info(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		result = -EIO;
		goto err_check_functionality;
	}
	if (!pdata) {
		result = -EINVAL;
		dev_err(&client->dev, "%s: platform data required.\n",
			__func__);
		goto err_no_platform_data;
	}
	acc = kzalloc(sizeof(*acc), GFP_KERNEL);
	if (!acc) {
		result = -ENOMEM;
		goto err_alloc_data_failed;
	}
	acc->client = client;
	acc->range = pdata->range ? pdata->range : 8;
	if (pdata->poll_interval_ms)
		atomic_set(&acc->poll_interval_ms, pdata->poll_interval_ms);
	else
		atomic_set(&acc->poll_interval_ms, 100);

	acc->ctl_regs_setup[SR_BASE_ADDR] = REG_CTL1_ADDR | AUTO_INCREMENT;
	acc->ctl_regs_setup[SR_ODR_PWR] = ENABLE_ALL_AXES | ODR___10_MS;
	acc->ctl_regs_setup[SR_HIPASS] = HP_NORMAL_RESET | HPCF1 | HPCF2 |
								 HPIS2 | HPIS1;
	acc->ctl_regs_setup[SR_I1_CFG] = 0;
	acc->ctl_regs_setup[SR_FSC] = HRES | FSC__2G | BLE_LE | BDU;

	acc->wq = create_singlethread_workqueue("lsm303dlhc_acc_lt");
	if (!acc->wq) {
		dev_err(&client->dev, "%s: workqueue alloc error\n", __func__);
		goto err_alloc_wq_failed;
	}
	INIT_DELAYED_WORK(&acc->work, lsm303dlhc_acc_lt_poll_func);
	INIT_LOCK(acc);

	lsm303dlhc_acc_lt_set_range(acc);

	if (pdata->power)
		acc->power = pdata->power;
	else
		acc->power = lsm303dlhc_acc_lt_power_stub;

	i2c_set_clientdata(client, acc);

	if (pdata->power_config) {
		result = pdata->power_config(&client->dev, true);
		if (result)
			goto err_not_responding;
	}

	acc->power(&client->dev, LSM303DLHC_LT_PWR_ON);
	result = lsm303dlhc_acc_lt_wr_reg(acc, REG_CTL1_ADDR, 0);
	lsm303dlhc_acc_lt_device_stop(acc);
	acc->power(&client->dev, LSM303DLHC_LT_PWR_OFF);

	if (result) {
		dev_err(&client->dev, "%s: device not responding\n", __func__);
		goto err_disable_power_config;
	}
	dev_info(&client->dev, "%s: device access ok.\n", __func__);

	pm_runtime_enable(&client->dev);

	result = create_sysfs_interfaces(&client->dev);
	if (result)
		goto sys_attr_err;

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		dev_err(&client->dev, "%s: input_allocate_device failed\n",
			__func__);
		result = -ENOMEM;
		goto err_allocate_device;
	}
	input_set_drvdata(acc->input_dev, acc);

	acc->input_dev->open = lsm303dlhc_acc_lt_open;
	acc->input_dev->close = lsm303dlhc_acc_lt_close;
	acc->input_dev->name = LSM303DLHC_ACC_LT_DEV_NAME;
	set_bit(EV_ABS, acc->input_dev->evbit);
	set_bit(ABS_X, acc->input_dev->absbit);
	set_bit(ABS_Y, acc->input_dev->absbit);
	set_bit(ABS_Z, acc->input_dev->absbit);
	input_set_abs_params(acc->input_dev, ABS_X, -ACC_RANGE_MG,
			ACC_RANGE_MG - 1, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -ACC_RANGE_MG,
			ACC_RANGE_MG - 1, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -ACC_RANGE_MG,
			ACC_RANGE_MG - 1, 0, 0);

	result = input_register_device(acc->input_dev);
	if (result) {
		dev_err(&client->dev, "%s: input_register_device failed!",
			__func__);
		input_free_device(acc->input_dev);
		goto err_register_device;
	}

	pm_runtime_set_autosuspend_delay(&client->dev, AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_mark_last_busy(&client->dev);

	dev_info(&client->dev, "%s completed.\n", __func__);

	return 0;

err_register_device:
err_allocate_device:
	remove_sysfs_interfaces(&client->dev);
sys_attr_err:
	pm_runtime_disable(&client->dev);
err_disable_power_config:
	if (pdata->power_config)
		pdata->power_config(&client->dev, false);
err_not_responding:
	destroy_workqueue(acc->wq);
err_alloc_wq_failed:
	kfree(acc);
err_alloc_data_failed:
err_no_platform_data:
err_check_functionality:
	dev_err(&client->dev, "%s failed.\n", __func__);
	return result;
}

static int __devexit lsm303dlhc_acc_lt_remove(struct i2c_client *client)
{
	struct lsm303dlhc_acc_lt_data *acc = i2c_get_clientdata(client);
	struct lsm303dlhc_acc_lt_platform_data *pdata =
					client->dev.platform_data;

	cancel_delayed_work_sync(&acc->work);
	input_unregister_device(acc->input_dev);
	remove_sysfs_interfaces(&client->dev);
	if (!pm_runtime_suspended(&client->dev)) {
		lsm303dlhc_acc_lt_device_stop(acc);
		acc->power(&client->dev, LSM303DLHC_LT_PWR_OFF);
	}
	pm_runtime_disable(&client->dev);

	if (pdata->power_config)
		pdata->power_config(&client->dev, false);
	destroy_workqueue(acc->wq);
	kfree(acc);
	return 0;
}

static const struct i2c_device_id lsm303dlhc_acc_lt_id[] = {
	{LSM303DLHC_ACC_LT_DEV_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lsm303dlhc_acc_lt_id);

static const struct dev_pm_ops lsm303dlhc_acc_lt_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303dlhc_acc_lt_suspend,
					lsm303dlhc_acc_lt_resume)
	SET_RUNTIME_PM_OPS(lsm303dlhc_acc_lt_runtime_suspend,
				lsm303dlhc_acc_lt_runtime_resume, NULL)
};

static struct i2c_driver lsm303dlhc_acc_lt_driver = {
	.driver = {
		.name = LSM303DLHC_ACC_LT_DEV_NAME,
		.owner = THIS_MODULE,
		.pm = &lsm303dlhc_acc_lt_pm,
	},
	.probe = lsm303dlhc_acc_lt_probe,
	.remove = __devexit_p(lsm303dlhc_acc_lt_remove),
	.id_table = lsm303dlhc_acc_lt_id,
};

static int __init lsm303dlhc_acc_lt_init(void)
{
	int err = i2c_add_driver(&lsm303dlhc_acc_lt_driver);
	pr_info("%s: LSM303DLHC Accelerometer driver, built %s @ %s\n",
		 __func__, __DATE__, __TIME__);

	return err;
}

static void __exit lsm303dlhc_acc_lt_exit(void)
{
	i2c_del_driver(&lsm303dlhc_acc_lt_driver);
}

module_init(lsm303dlhc_acc_lt_init);
module_exit(lsm303dlhc_acc_lt_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
MODULE_AUTHOR("Stefan Karlsson <stefan3.karlsson@sonymobile.com>");
MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("lsm303dlhc light accelerometer driver");
