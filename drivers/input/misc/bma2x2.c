/*
 *  Date: 2013/04/18 17:30:00
 *  Revision: 1.8.3
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011-2013 Bosch Sensortec GmbH
 * Copyright (C) 2012, 2013 Sony Mobile Communications AB.
 * All Rights Reserved
 */


/* file BMA2X2.c
   brief This file contains all function implementations for the BMA2X2 in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include "bstclass.h"

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#endif

#define BMA2X2_SENSOR_IDENTIFICATION_ENABLE

#define ACC_NAME  "bma"

#define buff_size 256
#define MSC_RX  MSC_SERIAL
#define MSC_RY  MSC_PULSELED
#define MSC_RZ  MSC_GESTURE


#define SENSOR_NAME			"bma2x2"
#define ABSMIN				-512
#define ABSMAX				512
#define SLOPE_THRESHOLD_VALUE		32
#define SLOPE_DURATION_VALUE		1
#define INTERRUPT_LATCH_MODE		13
#define INTERRUPT_ENABLE		1
#define INTERRUPT_DISABLE		0
#define MAP_SLOPE_INTERRUPT		2
#define SLOPE_X_INDEX			5
#define SLOPE_Y_INDEX			6
#define SLOPE_Z_INDEX			7
#define BMA2X2_MAX_DELAY		200
#define BMA2X2_RANGE_SET		3  /* +/- 2G */
#define BMA2X2_BW_SET			12 /* 125HZ  */


#define BMA2X2_EEP_OFFSET                       0x16
#define BMA2X2_IMAGE_BASE                       0x38
#define BMA2X2_IMAGE_LEN                        22

#define BMA2X2_CHIP_ID_REG                      0x00
#define BMA2X2_VERSION_REG                      0x01
#define BMA2X2_X_AXIS_LSB_REG                   0x02
#define BMA2X2_X_AXIS_MSB_REG                   0x03
#define BMA2X2_Y_AXIS_LSB_REG                   0x04
#define BMA2X2_Y_AXIS_MSB_REG                   0x05
#define BMA2X2_Z_AXIS_LSB_REG                   0x06
#define BMA2X2_Z_AXIS_MSB_REG                   0x07
#define BMA2X2_TEMPERATURE_REG                  0x08
#define BMA2X2_STATUS1_REG                      0x09
#define BMA2X2_STATUS2_REG                      0x0A
#define BMA2X2_STATUS_TAP_SLOPE_REG             0x0B
#define BMA2X2_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA2X2_STATUS_FIFO_REG                  0x0E
#define BMA2X2_RANGE_SEL_REG                    0x0F
#define BMA2X2_BW_SEL_REG                       0x10
#define BMA2X2_MODE_CTRL_REG                    0x11
#define BMA2X2_LOW_NOISE_CTRL_REG               0x12
#define BMA2X2_DATA_CTRL_REG                    0x13
#define BMA2X2_RESET_REG                        0x14
#define BMA2X2_INT_ENABLE1_REG                  0x16
#define BMA2X2_INT_ENABLE2_REG                  0x17
#define BMA2X2_INT_SLO_NO_MOT_REG               0x18
#define BMA2X2_INT1_PAD_SEL_REG                 0x19
#define BMA2X2_INT_DATA_SEL_REG                 0x1A
#define BMA2X2_INT2_PAD_SEL_REG                 0x1B
#define BMA2X2_INT_SRC_REG                      0x1E
#define BMA2X2_INT_SET_REG                      0x20
#define BMA2X2_INT_CTRL_REG                     0x21
#define BMA2X2_OFFSET_CTRL_REG                  0x36
#define BMA2X2_OFFSET_PARAMS_REG                0x37
#define BMA2X2_OFFSET_X_AXIS_REG                0x38
#define BMA2X2_OFFSET_Y_AXIS_REG                0x39
#define BMA2X2_OFFSET_Z_AXIS_REG                0x3A

#define BMA2X2_CHIP_ID__POS             0
#define BMA2X2_CHIP_ID__MSK             0xFF
#define BMA2X2_CHIP_ID__LEN             8
#define BMA2X2_CHIP_ID__REG             BMA2X2_CHIP_ID_REG

#define BMA2X2_VERSION__POS          0
#define BMA2X2_VERSION__LEN          8
#define BMA2X2_VERSION__MSK          0xFF
#define BMA2X2_VERSION__REG          BMA2X2_VERSION_REG

#define BMA2x2_SLO_NO_MOT_DUR__POS	2
#define BMA2x2_SLO_NO_MOT_DUR__LEN	6
#define BMA2x2_SLO_NO_MOT_DUR__MSK	0xFC
#define BMA2x2_SLO_NO_MOT_DUR__REG	BMA2X2_SLOPE_DURN_REG

#define BMA2X2_NEW_DATA_X__POS          0
#define BMA2X2_NEW_DATA_X__LEN          1
#define BMA2X2_NEW_DATA_X__MSK          0x01
#define BMA2X2_NEW_DATA_X__REG          BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X14_LSB__POS           2
#define BMA2X2_ACC_X14_LSB__LEN           6
#define BMA2X2_ACC_X14_LSB__MSK           0xFC
#define BMA2X2_ACC_X14_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X12_LSB__POS           4
#define BMA2X2_ACC_X12_LSB__LEN           4
#define BMA2X2_ACC_X12_LSB__MSK           0xF0
#define BMA2X2_ACC_X12_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X10_LSB__POS           6
#define BMA2X2_ACC_X10_LSB__LEN           2
#define BMA2X2_ACC_X10_LSB__MSK           0xC0
#define BMA2X2_ACC_X10_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X8_LSB__POS           0
#define BMA2X2_ACC_X8_LSB__LEN           0
#define BMA2X2_ACC_X8_LSB__MSK           0x00
#define BMA2X2_ACC_X8_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X_MSB__POS           0
#define BMA2X2_ACC_X_MSB__LEN           8
#define BMA2X2_ACC_X_MSB__MSK           0xFF
#define BMA2X2_ACC_X_MSB__REG           BMA2X2_X_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Y__POS          0
#define BMA2X2_NEW_DATA_Y__LEN          1
#define BMA2X2_NEW_DATA_Y__MSK          0x01
#define BMA2X2_NEW_DATA_Y__REG          BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y14_LSB__POS           2
#define BMA2X2_ACC_Y14_LSB__LEN           6
#define BMA2X2_ACC_Y14_LSB__MSK           0xFC
#define BMA2X2_ACC_Y14_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y12_LSB__POS           4
#define BMA2X2_ACC_Y12_LSB__LEN           4
#define BMA2X2_ACC_Y12_LSB__MSK           0xF0
#define BMA2X2_ACC_Y12_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y10_LSB__POS           6
#define BMA2X2_ACC_Y10_LSB__LEN           2
#define BMA2X2_ACC_Y10_LSB__MSK           0xC0
#define BMA2X2_ACC_Y10_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y8_LSB__POS           0
#define BMA2X2_ACC_Y8_LSB__LEN           0
#define BMA2X2_ACC_Y8_LSB__MSK           0x00
#define BMA2X2_ACC_Y8_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y_MSB__POS           0
#define BMA2X2_ACC_Y_MSB__LEN           8
#define BMA2X2_ACC_Y_MSB__MSK           0xFF
#define BMA2X2_ACC_Y_MSB__REG           BMA2X2_Y_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Z__POS          0
#define BMA2X2_NEW_DATA_Z__LEN          1
#define BMA2X2_NEW_DATA_Z__MSK          0x01
#define BMA2X2_NEW_DATA_Z__REG          BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z14_LSB__POS           2
#define BMA2X2_ACC_Z14_LSB__LEN           6
#define BMA2X2_ACC_Z14_LSB__MSK           0xFC
#define BMA2X2_ACC_Z14_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z12_LSB__POS           4
#define BMA2X2_ACC_Z12_LSB__LEN           4
#define BMA2X2_ACC_Z12_LSB__MSK           0xF0
#define BMA2X2_ACC_Z12_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z10_LSB__POS           6
#define BMA2X2_ACC_Z10_LSB__LEN           2
#define BMA2X2_ACC_Z10_LSB__MSK           0xC0
#define BMA2X2_ACC_Z10_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z8_LSB__POS           0
#define BMA2X2_ACC_Z8_LSB__LEN           0
#define BMA2X2_ACC_Z8_LSB__MSK           0x00
#define BMA2X2_ACC_Z8_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z_MSB__POS           0
#define BMA2X2_ACC_Z_MSB__LEN           8
#define BMA2X2_ACC_Z_MSB__MSK           0xFF
#define BMA2X2_ACC_Z_MSB__REG           BMA2X2_Z_AXIS_MSB_REG

#define BMA2X2_DATA_INT_S__POS           7
#define BMA2X2_DATA_INT_S__LEN           1
#define BMA2X2_DATA_INT_S__MSK           0x80
#define BMA2X2_DATA_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_RANGE_SEL__POS             0
#define BMA2X2_RANGE_SEL__LEN             4
#define BMA2X2_RANGE_SEL__MSK             0x0F
#define BMA2X2_RANGE_SEL__REG             BMA2X2_RANGE_SEL_REG

#define BMA2X2_BANDWIDTH__POS             0
#define BMA2X2_BANDWIDTH__LEN             5
#define BMA2X2_BANDWIDTH__MSK             0x1F
#define BMA2X2_BANDWIDTH__REG             BMA2X2_BW_SEL_REG

#define BMA2X2_SLEEP_DUR__POS             1
#define BMA2X2_SLEEP_DUR__LEN             4
#define BMA2X2_SLEEP_DUR__MSK             0x1E
#define BMA2X2_SLEEP_DUR__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_MODE_CTRL__POS             5
#define BMA2X2_MODE_CTRL__LEN             3
#define BMA2X2_MODE_CTRL__MSK             0xE0
#define BMA2X2_MODE_CTRL__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_DEEP_SUSPEND__POS          5
#define BMA2X2_DEEP_SUSPEND__LEN          1
#define BMA2X2_DEEP_SUSPEND__MSK          0x20
#define BMA2X2_DEEP_SUSPEND__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_LOW_POWER__POS          6
#define BMA2X2_EN_LOW_POWER__LEN          1
#define BMA2X2_EN_LOW_POWER__MSK          0x40
#define BMA2X2_EN_LOW_POWER__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_SUSPEND__POS            7
#define BMA2X2_EN_SUSPEND__LEN            1
#define BMA2X2_EN_SUSPEND__MSK            0x80
#define BMA2X2_EN_SUSPEND__REG            BMA2X2_MODE_CTRL_REG

#define BMA2X2_LOW_POWER_MODE__POS          6
#define BMA2X2_LOW_POWER_MODE__LEN          1
#define BMA2X2_LOW_POWER_MODE__MSK          0x40
#define BMA2X2_LOW_POWER_MODE__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_EN_NEW_DATA_INT__POS        4
#define BMA2X2_EN_NEW_DATA_INT__LEN        1
#define BMA2X2_EN_NEW_DATA_INT__MSK        0x10
#define BMA2X2_EN_NEW_DATA_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_INT1_PAD_NEWDATA__POS     0
#define BMA2X2_EN_INT1_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT1_PAD_NEWDATA__MSK     0x01
#define BMA2X2_EN_INT1_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_NEWDATA__POS     7
#define BMA2X2_EN_INT2_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT2_PAD_NEWDATA__MSK     0x80
#define BMA2X2_EN_INT2_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_INT_MODE_SEL__POS                0
#define BMA2X2_INT_MODE_SEL__LEN                4
#define BMA2X2_INT_MODE_SEL__MSK                0x0F
#define BMA2X2_INT_MODE_SEL__REG                BMA2X2_INT_CTRL_REG

#define BMA2X2_FAST_CAL_RDY_S__POS             4
#define BMA2X2_FAST_CAL_RDY_S__LEN             1
#define BMA2X2_FAST_CAL_RDY_S__MSK             0x10
#define BMA2X2_FAST_CAL_RDY_S__REG             BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_CAL_TRIGGER__POS                5
#define BMA2X2_CAL_TRIGGER__LEN                2
#define BMA2X2_CAL_TRIGGER__MSK                0x60
#define BMA2X2_CAL_TRIGGER__REG                BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_COMP_CUTOFF__POS                 0
#define BMA2X2_COMP_CUTOFF__LEN                 1
#define BMA2X2_COMP_CUTOFF__MSK                 0x01
#define BMA2X2_COMP_CUTOFF__REG                 BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_X__POS        1
#define BMA2X2_COMP_TARGET_OFFSET_X__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA2X2_COMP_TARGET_OFFSET_X__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Y__POS        3
#define BMA2X2_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA2X2_COMP_TARGET_OFFSET_Y__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Z__POS        5
#define BMA2X2_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA2X2_COMP_TARGET_OFFSET_Z__REG        BMA2X2_OFFSET_PARAMS_REG


#define BMA2X2_RANGE_2G                 3
#define BMA2X2_RANGE_4G                 5
#define BMA2X2_RANGE_8G                 8
#define BMA2X2_RANGE_16G                12

#define BMA2X2_BW_7_81HZ        0x08
#define BMA2X2_BW_15_63HZ       0x09
#define BMA2X2_BW_31_25HZ       0x0A
#define BMA2X2_BW_62_50HZ       0x0B
#define BMA2X2_BW_125HZ         0x0C
#define BMA2X2_BW_250HZ         0x0D
#define BMA2X2_BW_500HZ         0x0E
#define BMA2X2_BW_1000HZ        0x0F

#define BMA2X2_MODE_NORMAL             0
#define BMA2X2_MODE_LOWPOWER1          1
#define BMA2X2_MODE_SUSPEND            2
#define BMA2X2_MODE_DEEP_SUSPEND       3
#define BMA2X2_MODE_LOWPOWER2          4
#define BMA2X2_MODE_STANDBY            5

#define BMA2X2_X_AXIS           0
#define BMA2X2_Y_AXIS           1
#define BMA2X2_Z_AXIS           2

#define BMA2X2_DATA_EN               4

#define BMA2X2_INT1_NDATA        0
#define BMA2X2_INT2_NDATA        1

#define BMA2X2_INT1_OUTPUT      0
#define BMA2X2_INT2_OUTPUT      1
#define BMA2X2_INT1_LEVEL       0
#define BMA2X2_INT2_LEVEL       1

#define BMA2X2_CUT_OFF                  0
#define BMA2X2_OFFSET_TRIGGER_X         1
#define BMA2X2_OFFSET_TRIGGER_Y         2
#define BMA2X2_OFFSET_TRIGGER_Z         3


#define BMA2X2_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA2X2_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


#define BMA255_CHIP_ID 0XFA
#define BMA250E_CHIP_ID 0XF9
#define BMA222E_CHIP_ID 0XF8
#define BMA280_CHIP_ID 0XFB

#define BMA255_TYPE 0
#define BMA250E_TYPE 1
#define BMA222E_TYPE 2
#define BMA280_TYPE 3

#define MAX_FIFO_F_LEVEL 32
#define MAX_FIFO_F_BYTES 6
#define BMA_MAX_RETRY_I2C_XFER (100)

unsigned char *sensor_name[] = { "BMA255", "BMA250E", "BMA222E", "BMA280" };

struct bma2x2acc {
	s16	x,
		y,
		z;
} ;

struct bma2x2_data {
	struct i2c_client *bma2x2_client;
	atomic_t delay;
	atomic_t enable;
	atomic_t selftest_result;
	unsigned int chip_id;
	unsigned char mode;
	signed char sensor_type;
	struct input_dev *input;

	struct bst_dev *bst_acc;

	struct bma2x2acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	int IRQ;
	int irq_gpio;
	struct bosch_sensor_specific *bst_pd;
};

struct bma2x2_regulator_data {
	struct regulator *reg_vdd;
	struct regulator *reg_vio;
	char *id_vdd;
	char *id_vio;
};

static struct bma2x2_regulator_data bma2x2_regu_data;

static int bma2x2_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -EPERM;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma2x2_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -EPERM;
	return 0;
}

static int bma2x2_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -EPERM;
	return 0;
}

#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
static int bma2x2_set_newdata(struct i2c_client *client,
			unsigned char channel, unsigned char int_newdata)
{

	unsigned char data;
	int comres = 0;

	switch (channel) {
	case BMA2X2_INT1_NDATA:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_NEWDATA__REG, &data);
		data = BMA2X2_SET_BITSLICE(data,
				BMA2X2_EN_INT1_PAD_NEWDATA, int_newdata);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_NEWDATA__REG, &data);
		break;
	case BMA2X2_INT2_NDATA:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_NEWDATA__REG, &data);
		data = BMA2X2_SET_BITSLICE(data,
				BMA2X2_EN_INT2_PAD_NEWDATA, int_newdata);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_NEWDATA__REG, &data);
		break;
	default:
		comres = -1;
		break;
	}

	return comres;

}

static int bma2x2_set_Int_Enable(struct i2c_client *client, unsigned char
		InterruptType , unsigned char value)
{
	int comres;
	unsigned char data1, data2;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_INT_ENABLE1_REG, &data1);
	comres = bma2x2_smbus_read_byte(client, BMA2X2_INT_ENABLE2_REG, &data2);

	value = value & 1;
	switch (InterruptType) {

	case 4:
		/* New Data Interrupt  */

		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_EN_NEW_DATA_INT,
				value);
		break;

	default:
		break;
	}
	comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE1_REG,
			&data1);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE2_REG,
			&data2);

	return comres;
}
#endif /* CONFIG_BMA_ENABLE_NEWDATA_INT */

static int bma2x2_set_Int_Mode(struct i2c_client *client, unsigned char Mode)
{
	int comres;
	unsigned char data;


	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_INT_MODE_SEL__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_INT_MODE_SEL, Mode);
	comres = bma2x2_smbus_write_byte(client,
			BMA2X2_INT_MODE_SEL__REG, &data);


	return comres;
}
#endif

static int bma2x2_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int comres;
	unsigned char data1 = 0, data2 = 0;

	if (Mode < 6) {
		comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG,
				&data1);
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_LOW_NOISE_CTRL_REG,
				&data2);
		switch (Mode) {
		case BMA2X2_MODE_NORMAL:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 0);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 0);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
				usleep_range(1000, 1100);
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_LOWPOWER1:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 2);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 0);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
				usleep_range(1000, 1100);
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_SUSPEND:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 4);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 0);
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				usleep_range(1000, 1100);
				bma2x2_smbus_write_byte(client,
					BMA2X2_MODE_CTRL_REG, &data1);
				break;
		case BMA2X2_MODE_DEEP_SUSPEND:
				data1  = BMA2X2_SET_BITSLICE(data1,
							BMA2X2_MODE_CTRL, 1);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 1);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
				usleep_range(1000, 1100);
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_LOWPOWER2:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 2);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 1);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
				usleep_range(1000, 1100);
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_STANDBY:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 4);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 1);
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				usleep_range(1000, 1100);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
		break;
		}
	} else {
		comres = -1 ;
	}

	return comres;
}

static int bma2x2_get_mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres;
	unsigned char data1 = 0, data2 = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG, &data1);
	comres = bma2x2_smbus_read_byte(client, BMA2X2_LOW_NOISE_CTRL_REG,
			&data2);

	data1  = (data1 & 0xE0) >> 5;
	data2  = (data2 & 0x40) >> 6;


	if ((data1 == 0x00) && (data2 == 0x00))
		*Mode  = BMA2X2_MODE_NORMAL;
	else if ((data1 == 0x02) && (data2 == 0x00))
		*Mode  = BMA2X2_MODE_LOWPOWER1;
	else if ((data1 == 0x04 || data1 == 0x06) && (data2 == 0x00))
		*Mode  = BMA2X2_MODE_SUSPEND;
	else if (((data1 & 0x01) == 0x01))
		*Mode  = BMA2X2_MODE_DEEP_SUSPEND;
	else if ((data1 == 0x02) && (data2 == 0x01))
		*Mode  = BMA2X2_MODE_LOWPOWER2;
	else if ((data1 == 0x04) && (data2 == 0x01))
		*Mode  = BMA2X2_MODE_STANDBY;
	else
		*Mode = BMA2X2_MODE_DEEP_SUSPEND;

	return comres;
}

static int bma2x2_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres;
	unsigned char data1 = 0;

	if ((Range == 3) || (Range == 5) || (Range == 8) || (Range == 12)) {
		comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL_REG,
				&data1);
		switch (Range) {
		case BMA2X2_RANGE_2G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 3);
			break;
		case BMA2X2_RANGE_4G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 5);
			break;
		case BMA2X2_RANGE_8G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 8);
			break;
		case BMA2X2_RANGE_16G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 12);
			break;
		}
		comres += bma2x2_smbus_write_byte(client, BMA2X2_RANGE_SEL_REG,
				&data1);
	} else {
		comres = -1 ;
	}

	return comres;
}

static int bma2x2_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres;
	unsigned char data;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL__REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_RANGE_SEL);
	*Range = data;

	return comres;
}

static int bma2x2_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres;
	unsigned char data = 0;
	int Bandwidth = 0;

	if (BW > 7 && BW < 16) {
		switch (BW) {
		case BMA2X2_BW_7_81HZ:
			Bandwidth = BMA2X2_BW_7_81HZ;

			/*  7.81 Hz      64000 uS   */
			break;
		case BMA2X2_BW_15_63HZ:
			Bandwidth = BMA2X2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
		case BMA2X2_BW_31_25HZ:
			Bandwidth = BMA2X2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
		case BMA2X2_BW_62_50HZ:
			Bandwidth = BMA2X2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
		case BMA2X2_BW_125HZ:
			Bandwidth = BMA2X2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
		case BMA2X2_BW_250HZ:
			Bandwidth = BMA2X2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
		case BMA2X2_BW_500HZ:
			Bandwidth = BMA2X2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
		case BMA2X2_BW_1000HZ:
			Bandwidth = BMA2X2_BW_1000HZ;

			/*  1000 Hz      500 uS   */
			break;
		}
		comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_BANDWIDTH, Bandwidth);
		comres += bma2x2_smbus_write_byte(client, BMA2X2_BANDWIDTH__REG,
				&data);
	} else {
		comres = -1 ;
	}

	return comres;
}

static int bma2x2_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres;
	unsigned char data;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_BANDWIDTH);
	*BW = data ;

	return comres;
}

static int bma2x2_set_offset_x(struct i2c_client *client, unsigned char
		offsetfilt)
{
	int comres;
	unsigned char data;

	data =  offsetfilt;
	comres = bma2x2_smbus_write_byte(client, BMA2X2_OFFSET_X_AXIS_REG,
						&data);

	return comres;
}


static int bma2x2_get_offset_x(struct i2c_client *client, unsigned char
						*offsetfilt)
{
	int comres;
	unsigned char data;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_OFFSET_X_AXIS_REG,
						&data);
	*offsetfilt = data;

	return comres;
}

static int bma2x2_set_offset_y(struct i2c_client *client, unsigned char
						offsetfilt)
{
	int comres;
	unsigned char data;

	data =  offsetfilt;
	comres = bma2x2_smbus_write_byte(client, BMA2X2_OFFSET_Y_AXIS_REG,
						&data);

	return comres;
}

static int bma2x2_get_offset_y(struct i2c_client *client, unsigned char
						*offsetfilt)
{
	int comres;
	unsigned char data;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_OFFSET_Y_AXIS_REG,
						&data);
	*offsetfilt = data;

	return comres;
}

static int bma2x2_set_offset_z(struct i2c_client *client, unsigned char
						offsetfilt)
{
	int comres;
	unsigned char data;

	data =  offsetfilt;
	comres = bma2x2_smbus_write_byte(client, BMA2X2_OFFSET_Z_AXIS_REG,
						&data);

	return comres;
}

static int bma2x2_get_offset_z(struct i2c_client *client, unsigned char
						*offsetfilt)
{
	int comres;
	unsigned char data;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_OFFSET_Z_AXIS_REG,
						&data);
	*offsetfilt = data;

	return comres;
}

static int bma2x2_read_accel_xyz(struct i2c_client *client,
		signed char sensor_type, struct bma2x2acc *acc)
{
	int comres = 0;
	unsigned char data[6];

#ifdef BMA2X2_SENSOR_IDENTIFICATION_ENABLE
	comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X12_LSB__REG, data, 6);
	acc->x = (data[1]<<8)|data[0];
	acc->y = (data[3]<<8)|data[2];
	acc->z = (data[5]<<8)|data[4];

#else
	switch (sensor_type) {
	case 0:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X12_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X12_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X12_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y12_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y12_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z12_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z12_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 1:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X10_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X10_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X10_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y10_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y10_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z10_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z10_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 2:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X8_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X8_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X8_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y8_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y8_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z8_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z8_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 3:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X14_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X14_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X14_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y14_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y14_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z14_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z14_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	default:
		break;
	}
#endif

	return comres;
}

#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
static void bma2x2_work_func(struct work_struct *work)
{
	struct bma2x2_data *bma2x2 = container_of((struct delayed_work *)work,
			struct bma2x2_data, work);
	static struct bma2x2acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma2x2->delay));

	bma2x2_read_accel_xyz(bma2x2->bma2x2_client, bma2x2->sensor_type,
									 &acc);
	input_event(bma2x2->input, EV_MSC, MSC_RX, acc.x);
	input_event(bma2x2->input, EV_MSC, MSC_RY, acc.y);
	input_event(bma2x2->input, EV_MSC, MSC_RZ, acc.z);
	input_sync(bma2x2->input);
	mutex_lock(&bma2x2->value_mutex);
	bma2x2->value = acc;
	mutex_unlock(&bma2x2->value_mutex);
	schedule_delayed_work(&bma2x2->work, delay);
}
#endif

static ssize_t bma2x2_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	if (bma2x2_get_range(bma2x2->bma2x2_client, &data) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%d\n", data);
}

static ssize_t bma2x2_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_range(bma2x2->bma2x2_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	if (bma2x2_get_bandwidth(bma2x2->bma2x2_client, &data) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%d\n", data);
}

static ssize_t bma2x2_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2->sensor_type == BMA280_TYPE)
		if ((unsigned char) data > 14)
			return -EINVAL;

	if (bma2x2_set_bandwidth(bma2x2->bma2x2_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	if (bma2x2_get_mode(bma2x2->bma2x2_client, &data) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%d\n", data);
}

static ssize_t bma2x2_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_mode(bma2x2->bma2x2_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma2x2_data *bma2x2 = input_get_drvdata(input);
	struct bma2x2acc acc_value;

	bma2x2_read_accel_xyz(bma2x2->bma2x2_client, bma2x2->sensor_type,
								&acc_value);

	return scnprintf(buf, buff_size, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma2x2_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	return scnprintf(buf, buff_size, "%d\n", atomic_read(&bma2x2->delay));
}

static ssize_t bma2x2_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > BMA2X2_MAX_DELAY)
		data = BMA2X2_MAX_DELAY;
	atomic_set(&bma2x2->delay, (unsigned int) data);

	return count;
}


static ssize_t bma2x2_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	return scnprintf(buf, buff_size, "%d\n", atomic_read(&bma2x2->enable));
}

static void bma2x2_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&bma2x2->enable);

	mutex_lock(&bma2x2->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			bma2x2_set_mode(bma2x2->bma2x2_client,
					BMA2X2_MODE_NORMAL);
			usleep_range(10000, 11000);
#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
			enable_irq(bma2x2->IRQ);
			/* enable new data interrupt */
			bma2x2_set_Int_Enable(bma2x2->bma2x2_client, 4, 1);
#else
			schedule_delayed_work(&bma2x2->work,
				msecs_to_jiffies(atomic_read(&bma2x2->delay)));
#endif
			atomic_set(&bma2x2->enable, 1);
		}

	} else {
		if (pre_enable == 1) {
#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
			/* disable new data interrupt */
			bma2x2_set_Int_Enable(bma2x2->bma2x2_client, 4, 0);
			disable_irq(bma2x2->IRQ);
#else
			cancel_delayed_work_sync(&bma2x2->work);
#endif
			bma2x2_set_mode(bma2x2->bma2x2_client,
					BMA2X2_MODE_SUSPEND);
			usleep_range(10000, 11000);
			atomic_set(&bma2x2->enable, 0);
		}
	}
	mutex_unlock(&bma2x2->enable_mutex);

}

static ssize_t bma2x2_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		bma2x2_set_enable(dev, data);

	return count;
}

static ssize_t bma2x2_offset_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	if (bma2x2_get_offset_x(bma2x2->bma2x2_client, &data) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%hhd\n", (char)data);
}

static ssize_t bma2x2_offset_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = kstrtol(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_offset_x(bma2x2->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_offset_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	if (bma2x2_get_offset_y(bma2x2->bma2x2_client, &data) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%hhd\n", (char)data);
}

static ssize_t bma2x2_offset_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = kstrtol(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_offset_y(bma2x2->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_offset_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	if (bma2x2_get_offset_z(bma2x2->bma2x2_client, &data) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%hhd\n", (char)data);
}

static ssize_t bma2x2_offset_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = kstrtol(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_offset_z(bma2x2->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}


static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_range_show, bma2x2_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_bandwidth_show, bma2x2_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_mode_show, bma2x2_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
		bma2x2_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_delay_show, bma2x2_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_enable_show, bma2x2_enable_store);
static DEVICE_ATTR(offset_x, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_offset_x_show,
		bma2x2_offset_x_store);
static DEVICE_ATTR(offset_y, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_offset_y_show,
		bma2x2_offset_y_store);
static DEVICE_ATTR(offset_z, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma2x2_offset_z_show,
		bma2x2_offset_z_store);

static struct attribute *bma2x2_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_offset_x.attr,
	&dev_attr_offset_y.attr,
	&dev_attr_offset_z.attr,
	NULL
};

static struct attribute_group bma2x2_attribute_group = {
	.attrs = bma2x2_attributes
};


#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
static irqreturn_t bma2x2_interrupt_thread(int irq, void *handle)
{
	struct bma2x2_data *bma2x2 = handle;
	static struct bma2x2acc acc;

	bma2x2_read_accel_xyz(bma2x2->bma2x2_client,
				bma2x2->sensor_type, &acc);
	input_event(bma2x2->input, EV_MSC, MSC_RX, acc.x);
	input_event(bma2x2->input, EV_MSC, MSC_RY, acc.y);
	input_event(bma2x2->input, EV_MSC, MSC_RZ, acc.z);
	input_sync(bma2x2->input);

	return IRQ_HANDLED;
}
#endif /* defined(BMA2X2_ENABLE_INT1)||defined(BMA2X2_ENABLE_INT2) */

static int bma2x2_regulator_init(struct device *dev)
{
	int rc = 0;

	struct bma2x2_regulator_data *regdata = &bma2x2_regu_data;

	if (!regdata->reg_vdd)
		regdata->reg_vdd = devm_regulator_get(dev, regdata->id_vdd);
	if (IS_ERR_OR_NULL(regdata->reg_vdd)) {
		rc = PTR_ERR(regdata->reg_vdd);
		dev_err(dev, "%s: regulator_get failed on %s. rc=%d\n",
					__func__, regdata->id_vdd, rc);
		rc = rc ? rc : -ENODEV;
		goto err_vdd;
	} else {
		rc = regulator_set_voltage(regdata->reg_vdd, 2850000, 2850000);
		if (rc)
			goto err_vdd_set;
	}

	if (!regdata->reg_vio)
		regdata->reg_vio = devm_regulator_get(dev, regdata->id_vio);
	if (IS_ERR_OR_NULL(regdata->reg_vio)) {
		rc = PTR_ERR(regdata->reg_vio);
		dev_err(dev, "%s: regulator_get failed on %s. rc=%d\n",
					__func__, regdata->id_vio, rc);
		rc = rc ? rc : -ENODEV;
		goto err_vio;
	}
	return rc;
err_vio:
	regdata->reg_vio = NULL;
err_vdd_set:
	regulator_put(regdata->reg_vdd);
err_vdd:
	regdata->reg_vdd = NULL;
	return rc;
}

static int  bma2x2_regulator_control(bool enable, struct device *dev)
{
	int rc = 0;

	struct bma2x2_regulator_data *regdata = &bma2x2_regu_data;

	if (enable) {
		rc = regulator_enable(regdata->reg_vdd);
		if (rc) {
			dev_err(dev,
				"%s: regulator_enable failed on %s. rc=%d\n",
						__func__, regdata->id_vdd, rc);
			return rc;
		}
		rc = regulator_enable(regdata->reg_vio);
		if (rc)
			dev_err(dev,
				"%s: regulator_enable failed on %s. rc=%d\n",
						__func__, regdata->id_vio, rc);
	} else {
		rc = regulator_disable(regdata->reg_vio);
		if (rc) {
			dev_err(dev,
				"%s: regulator_disable failed on %s. rc=%d\n",
						__func__, regdata->id_vio, rc);
			return rc;
		}
		rc = regulator_disable(regdata->reg_vdd);
		if (rc) {
			dev_err(dev,
				"%s: regulator_disable failed on %s. rc=%d\n",
						__func__, regdata->id_vdd, rc);
			return rc;
		}
	}

	/* Wait untile device stable */
	usleep_range(3000, 3100);

	return rc;
}

static int bma2x2_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	unsigned char tmp_chip_id;
#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
	unsigned int irq_gpio_flags;
#endif
	struct bma2x2_data *data;
	struct input_dev *dev;
	struct bst_dev  *dev_acc;
	struct bma2x2_regulator_data *regdata = &bma2x2_regu_data;

	dev_info(&client->dev, "%s: BMA2x2 start probing.\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_info(&client->dev, "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma2x2_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
	data->irq_gpio = of_get_named_gpio_flags(client->dev.of_node,
				"bma2x2,irq_gpio", 0, &irq_gpio_flags);
	if (data->irq_gpio < 0) {
		dev_err(&client->dev,
			"%s: negative gpio number ", __func__);
		err = -EIO;
		goto kfree_exit;
	}

	err = gpio_request(data->irq_gpio, SENSOR_NAME);
	if (err) {
		dev_err(&client->dev, "%s: gpio_request failed, err=%d\n",
					__func__, err);
		goto kfree_exit;
	}
#endif

	/* init regulator */
	regdata->reg_vdd = NULL;
	regdata->reg_vio = NULL;
	regdata->id_vdd = "bma2x2_vdd";
	regdata->id_vio = "bma2x2_vio";
	bma2x2_regulator_init(&client->dev);

	/* power on */
	bma2x2_regulator_control(1, &client->dev);

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, BMA2X2_CHIP_ID_REG);
	tmp_chip_id = tempvalue&0x00ff;

	switch (tmp_chip_id) {
	case BMA255_CHIP_ID:
		data->sensor_type = BMA255_TYPE;
		break;
	case BMA250E_CHIP_ID:
		data->sensor_type = BMA250E_TYPE;
		break;
	case BMA222E_CHIP_ID:
		data->sensor_type = BMA222E_TYPE;
		break;
	case BMA280_CHIP_ID:
		data->sensor_type = BMA280_TYPE;
		break;
	default:
		data->sensor_type = -1;
	}
	if (data->sensor_type != -1) {
		data->chip_id = tmp_chip_id;
		dev_info(&client->dev, "Bosch Sensortec Device detected!\n" \
				"%s registered I2C driver!\n", \
						sensor_name[data->sensor_type]);
	} else{
		dev_info(&client->dev, "Bosch Sensortec Device not found" \
				"i2c error %d\n", tempvalue);
		err = -ENODEV;
		goto kfree_exit;
	}
	i2c_set_clientdata(client, data);
	data->bma2x2_client = client;
	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);
	bma2x2_set_bandwidth(client, BMA2X2_BW_SET);
	if (bma2x2_set_range(client, BMA2X2_RANGE_SET) < 0)
		dev_err(&client->dev, "%s: bma2x2_set_range failed\n",
			__func__);

#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)

#ifdef BMA2X2_ENABLE_INT1
	/* maps interrupt to INT1 pin */
#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
	bma2x2_set_newdata(client, BMA2X2_INT1_NDATA, 1);
	bma2x2_set_newdata(client, BMA2X2_INT2_NDATA, 0);
#endif
#endif

#ifdef BMA2X2_ENABLE_INT2
	/* maps interrupt to INT2 pin */
#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
	bma2x2_set_newdata(client, BMA2X2_INT1_NDATA, 0);
	bma2x2_set_newdata(client, BMA2X2_INT2_NDATA, 1);
#endif
#endif
	bma2x2_set_Int_Mode(client, 1);/*latch interrupt 250ms*/

	data->IRQ = client->irq;
	err = request_threaded_irq(data->IRQ, NULL, bma2x2_interrupt_thread,
			IRQF_TRIGGER_RISING, "bma_int", data);
	if (err)
		dev_err(&client->dev, "could not request irq\n");

	disable_irq(data->IRQ);
#endif

#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
	INIT_DELAYED_WORK(&data->work, bma2x2_work_func);

#endif

	atomic_set(&data->delay, BMA2X2_MAX_DELAY);
	atomic_set(&data->enable, 0);

	bma2x2_set_mode(client, BMA2X2_MODE_SUSPEND);
	dev = input_allocate_device();
	if (!dev) {
		err = -ENOMEM;
		goto kfree_exit;
	}
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_MSC, ABS_MISC);
	input_set_capability(dev, EV_MSC, MSC_RX);
	input_set_capability(dev, EV_MSC, MSC_RY);
	input_set_capability(dev, EV_MSC, MSC_RZ);

	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		goto kfree_exit;
	}

	data->input = dev;

	dev_acc = bst_allocate_device();
	if (!dev_acc) {
		err = -ENOMEM;
		goto error_sysfs;
	}
	dev_acc->name = ACC_NAME;

	bst_set_drvdata(dev_acc, data);

	err = bst_register_device(dev_acc);
	if (err < 0)
		goto bst_free_acc_exit;

	data->bst_acc = dev_acc;

	err = sysfs_create_group(&data->bst_acc->dev.kobj,
			&bma2x2_attribute_group);
	if (err < 0)
		goto bst_free_exit;

	return 0;

bst_free_exit:
	(void)bst_unregister_device(dev_acc);

bst_free_acc_exit:
	bst_free_device(dev_acc);

error_sysfs:
	input_unregister_device(data->input);

kfree_exit:
#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
	if (NULL != data)
		gpio_free(data->irq_gpio);
#endif
	if ((NULL != data) && (NULL != data->bst_pd)) {
		kfree(data->bst_pd);
		data->bst_pd = NULL;
	}
	kfree(data);
exit:
	return err;
}

static int __devexit bma2x2_remove(struct i2c_client *client)
{
	struct bma2x2_data *data = i2c_get_clientdata(client);

	bma2x2_set_enable(&client->dev, 0);
	sysfs_remove_group(&data->input->dev.kobj, &bma2x2_attribute_group);
	input_unregister_device(data->input);

	if (NULL != data->bst_pd) {
		kfree(data->bst_pd);
		data->bst_pd = NULL;
	}

#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
	gpio_free(data->irq_gpio);
#endif
	kfree(data);
	return 0;
}

static int bma2x2_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bma2x2_data *data = i2c_get_clientdata(client);

	if (atomic_read(&data->enable) == 1) {
		bma2x2_set_mode(data->bma2x2_client, BMA2X2_MODE_SUSPEND);
#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
		cancel_delayed_work_sync(&data->work);
#endif
	}

	return 0;
}

static int bma2x2_resume(struct i2c_client *client)
{
	struct bma2x2_data *data = i2c_get_clientdata(client);
	unsigned char op_mode;

	if (atomic_read(&data->enable) == 1) {
		bma2x2_set_mode(data->bma2x2_client, BMA2X2_MODE_NORMAL);
#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
		schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
#endif
	} else {
		bma2x2_get_mode(data->bma2x2_client, &op_mode);
		if (op_mode == BMA2X2_MODE_NORMAL)
			bma2x2_set_mode(data->bma2x2_client,
						BMA2X2_MODE_SUSPEND);
	}

	return 0;
}

static const struct i2c_device_id bma2x2_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma2x2_id);

#ifdef CONFIG_OF
static struct of_device_id bma2x2_match_table[] = {
	{ .compatible = "bosch,bma2x2", },
	{ },
};
#endif

static struct i2c_driver bma2x2_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = bma2x2_match_table,
#endif
	},
	.suspend	= bma2x2_suspend,
	.resume		= bma2x2_resume,
	.id_table	= bma2x2_id,
	.probe		= bma2x2_probe,
	.remove		= __devexit_p(bma2x2_remove),

};

static int __init BMA2X2_init(void)
{
	return i2c_add_driver(&bma2x2_driver);
}

static void __exit BMA2X2_exit(void)
{
	i2c_del_driver(&bma2x2_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA2X2 accelerometer sensor driver");
MODULE_LICENSE("GPL v2");

module_init(BMA2X2_init);
module_exit(BMA2X2_exit);
