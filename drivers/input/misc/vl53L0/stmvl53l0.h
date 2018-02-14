/*
 *  stmvl53l0.h - Linux kernel modules for STM VL53L0 FlightSense TOF sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division
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
 */
/*
 * Copyright (C) 2016 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
/*
 * Defines
 */
#ifndef STMVL53L0_H
#define STMVL53L0_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>


#define STMVL53L0_DRV_NAME	"stmvl53l0"
#define STMVL53L0_SLAVE_ADDR	(0x52>>1)

#define DRIVER_VERSION		"1.0.0"
#define I2C_M_WR			0x00
/* #define INT_POLLING_DELAY	20 */

/* if don't want to have output from vl53l0_dbgmsg, comment out #DEBUG macro */
//#define DEBUG
#define vl53l0_dbgmsg(str, args...)	\
	pr_err("%s: " str, __func__, ##args)
#define vl53l0_errmsg(str, args...) \
	pr_err("%s: " str, __func__, ##args)

#define VL53L0_VDD_MIN      2600000
#define VL53L0_VDD_MAX      3000000

typedef enum {
	NORMAL_MODE = 0,
	OFFSETCALIB_MODE = 1,
	XTALKCALIB_MODE = 2,
} init_mode_e;

typedef enum {
	OFFSET_PAR = 0,
	XTALKRATE_PAR = 1,
	XTALKENABLE_PAR = 2,
	GPIOFUNC_PAR = 3,
	LOWTHRESH_PAR = 4,
	HIGHTHRESH_PAR = 5,
	DEVICEMODE_PAR = 6,
	INTERMEASUREMENT_PAR = 7,
} parameter_name_e;

enum {
	CCI_BUS = 0,
	I2C_BUS = 1,
};

/*
 *  IOCTL register data structs
 */
struct stmvl53l0_register {
	uint32_t is_read; /*1: read 0: write*/
	uint32_t reg_index;
	uint32_t reg_bytes;
	uint32_t reg_data;
	int32_t status;
};

/*
 *  IOCTL parameter structs
 */
struct stmvl53l0_parameter {
	uint32_t is_read; /*1: Get 0: Set*/
	parameter_name_e name;
	int32_t value;
	int32_t status;
};

/*
 *  driver data structs
 */
struct stmvl53l0_data {

	VL53L0_DevData_t Data;	/* !<embed ST VL53L0 Dev data as
								"dev_data" */
	uint8_t   I2cDevAddr;	/*!< i2c device address user specific field
							*/
	uint8_t   comms_type;	/*!< Type of comms : VL53L0_COMMS_I2C
							or VL53L0_COMMS_SPI */
	uint16_t  comms_speed_khz;	/*!< Comms speed [kHz] :
						typically 400kHz for I2C */
	uint8_t   bus_type;		/* CCI_BUS; I2C_BUS */

	void *client_object; /* cci or i2c client */

	struct mutex update_lock;
	struct delayed_work	dwork;		/* for PS  work handler */
	struct input_dev *input_dev_ps;
	struct kobject *range_kobj;

	const char *dev_name;
	/* function pointer */

	/* misc device */
	struct miscdevice miscdev;

	int irq;
	unsigned int reset;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;

	/* PS parameters */
	//unsigned int ps_is_singleshot;
	//unsigned int ps_is_started;
	unsigned int ps_data;			/* to store PS data */

	/* Calibration parameters */
	unsigned int offsetCalDistance;
	unsigned int xtalkCalDistance;


	/* Range Data */
	VL53L0_RangingMeasurementData_t rangeData;

	/* Device parameters */
	VL53L0_DeviceModes 			deviceMode;
	uint32_t					interMeasurems;
	VL53L0_GpioFunctionality gpio_function;
	VL53L0_InterruptPolarity gpio_polarity;
	FixPoint1616_t low_threshold;
	FixPoint1616_t high_threshold;

	/* delay time in miniseconds*/
	uint8_t delay_ms;

	struct mutex work_mutex;

	/* Debug */
	unsigned int enableDebug;
	uint8_t interrupt_received;
};

/*
 *  function pointer structs
 */
struct stmvl53l0_module_fn_t {
	int (*init)(void);
	void (*deinit)(void *);
	int (*power_up)(void *, unsigned int *);
	int (*power_down)(void *);
};



int stmvl53l0_setup(struct stmvl53l0_data *data);

#endif /* STMVL53L0_H */
