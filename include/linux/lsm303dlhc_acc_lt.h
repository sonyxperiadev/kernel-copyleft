/*
 * lsm303dlhc_acc_lt.h -  lsm303dlh accelerometer driver API
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 * Copyright (c) 2012 Sony Mobile Communications AB.
 * License terms: GNU General Public License (GPL) version 2
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *         Stefan Karlsson <stefan3.karlsson@sonymobile.com>
 *
 */
#ifndef _LSM303DLHC_ACC_LT_H_
#define _LSM303DLHC_ACC_LT_H_
#include <linux/device.h>
#include <linux/kernel.h>

#define LSM303DLHC_ACC_LT_DEV_NAME "lsm303dlhc_acc_lt"

enum lsm303dlhc_acc_lt_power_state {
	LSM303DLHC_LT_PWR_ON,
	LSM303DLHC_LT_PWR_OFF,
	LSM303DLHC_LT_STANDBY,
};

struct lsm303dlhc_acc_lt_platform_data {
	int range;
	int poll_interval_ms;
	int (*power)(struct device *dev,
			enum lsm303dlhc_acc_lt_power_state pwr_state);
	int (*power_config)(struct device *dev, bool value);
};
#endif

