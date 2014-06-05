/*
 * Last modified: April 18th, 2013
 * Revision: V1.4.5
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2012, 2013 Bosch Sensortec GmbH
 * Copyright (C) 2012, 2013 Sony Mobile Communications AB.
 * All Rights Reserved
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "bstclass.h"

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#endif

/* sensor specific */
#define SENSOR_NAME "bmg160"

#define SENSOR_CHIP_ID_BMG (0x0f)

#define BMG_REG_NAME(name) BMG160_##name
#define BMG_VAL_NAME(name) BMG160_##name
#define BMG_CALL_API(name) bmg160_##name

#define BMG_I2C_WRITE_DELAY_TIME 1

/* generic */
#define BMG_MAX_RETRY_I2C_XFER (100)
#define BMG_MAX_RETRY_WAKEUP (5)
#define BMG_MAX_RETRY_WAIT_DRDY (100)

#define BMG_DELAY_MIN (1)
#define BMG_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (32767)
#define MAG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define buff_size 256

#define MSC_RX  MSC_SERIAL
#define MSC_RY  MSC_PULSELED
#define MSC_RZ  MSC_GESTURE

#define INT_PIN_PUSH_PULL     0
#define INT_PIN_OPEN_DRAIN    1
#define INT_DISABLE           0
#define INT_ENABLE            1

#define BMG160_ENABLE_INT1    1
#define BMG160_ENABLE_INT2    2

#define BMG160_U16 unsigned short       /* 16 bit achieved with short */
#define BMG160_S16 signed short
#define BMG160_S32 signed int           /* 32 bit achieved with int   */

/* never change this line */
#define BMG160_BUS_WRITE_FUNC(device_addr, register_addr,\
register_data, wr_len) bus_write(device_addr, register_addr,\
register_data, wr_len)

/* never change this line */
#define BMG160_BUS_READ_FUNC(device_addr, register_addr,\
register_data, rd_len) bus_read(device_addr, register_addr,\
register_data, rd_len)

#define	BMG160_I2C_ADDR1				0x68
#define	BMG160_I2C_ADDR					BMG160_I2C_ADDR1
#define	BMG160_I2C_ADDR2				0x69

/*Define of registers*/

/* Hard Wired */
#define BMG160_CHIP_ID_ADDR						0x00
/**<Address of Chip ID Register*/


/* Data Register */
#define BMG160_RATE_X_LSB_ADDR                   0x02
/**<        Address of X axis Rate LSB Register       */
#define BMG160_RATE_X_MSB_ADDR                   0x03
/**<        Address of X axis Rate MSB Register       */
#define BMG160_RATE_Y_LSB_ADDR                   0x04
/**<        Address of Y axis Rate LSB Register       */
#define BMG160_RATE_Y_MSB_ADDR                   0x05
/**<        Address of Y axis Rate MSB Register       */
#define BMG160_RATE_Z_LSB_ADDR                   0x06
/**<        Address of Z axis Rate LSB Register       */
#define BMG160_RATE_Z_MSB_ADDR                   0x07
/**<        Address of Z axis Rate MSB Register       */

/* Status Register */
#define BMG160_INT_STATUS0_ADDR                 0x09
/**<        Address of Interrupt status Register 0    */
#define BMG160_INT_STATUS1_ADDR                 0x0A
/**<        Address of Interrupt status Register 1    */
#define BMG160_INT_STATUS2_ADDR                 0x0B
/**<        Address of Interrupt status Register 2    */
#define BMG160_INT_STATUS3_ADDR                 0x0C
/**<        Address of Interrupt status Register 3    */
#define BMG160_FIFO_STATUS_ADDR                 0x0E
/**<        Address of FIFO status Register           */

/* Control Register */
#define BMG160_RANGE_ADDR                  0x0F
/**<        Address of Range address Register     */
#define BMG160_BW_ADDR                     0x10
/**<        Address of Bandwidth Register         */
#define BMG160_MODE_LPM1_ADDR              0x11
/**<        Address of Mode LPM1 Register         */
#define BMG160_MODE_LPM2_ADDR              0x12
/**<        Address of Mode LPM2 Register         */
#define BMG160_RATED_HBW_ADDR              0x13
/**<        Address of Rate HBW Register          */
#define BMG160_BGW_SOFTRESET_ADDR          0x14
/**<        Address of BGW Softreset Register      */
#define BMG160_INT_ENABLE0_ADDR            0x15
/**<        Address of Interrupt Enable 0             */
#define BMG160_INT_ENABLE1_ADDR            0x16
/**<        Address of Interrupt Enable 1             */
#define BMG160_INT_MAP_0_ADDR              0x17
/**<        Address of Interrupt MAP 0                */
#define BMG160_INT_MAP_1_ADDR              0x18
/**<        Address of Interrupt MAP 1                */
#define BMG160_INT_MAP_2_ADDR              0x19
/**<        Address of Interrupt MAP 2                */
#define BMG160_INT_0_ADDR                  0x1A
/**<        Address of Interrupt 0 register   */
#define BMG160_INT_1_ADDR                  0x1B
/**<        Address of Interrupt 1 register   */
#define BMG160_INT_2_ADDR                  0x1C
/**<        Address of Interrupt 2 register   */
#define BMG160_INT_4_ADDR                  0x1E
/**<        Address of Interrupt 4 register   */
#define BMG160_A_FOC_ADDR                      0x32
/**<        Address of A_FOC Register        */

/* Trim Register */
#define BMG160_OFC1_ADDR                   0x36
/**<        Address of OFC1 Register          */
#define BMG160_OFC2_ADDR                       0x37
/**<        Address of OFC2 Register          */
#define BMG160_OFC3_ADDR                   0x38
/**<        Address of OFC3 Register          */
#define BMG160_OFC4_ADDR                   0x39
/**<        Address of OFC4 Register          */
#define BMG160_TRIM_GP0_ADDR               0x3A
/**<        Address of Trim GP0 Register              */
#define BMG160_TRIM_GP1_ADDR               0x3B
/**<        Address of Trim GP1 Register              */

/* Rate X LSB Register */
#define BMG160_RATE_X_LSB_VALUEX__POS        0

/**< Last 8 bits of RateX LSB Registers */
#define BMG160_RATE_X_LSB_VALUEX__LEN        8
#define BMG160_RATE_X_LSB_VALUEX__MSK        0xFF
#define BMG160_RATE_X_LSB_VALUEX__REG        BMG160_RATE_X_LSB_ADDR

/* Rate Y LSB Register */
/**<  Last 8 bits of RateY LSB Registers */
#define BMG160_RATE_Y_LSB_VALUEY__POS        0
#define BMG160_RATE_Y_LSB_VALUEY__LEN        8
#define BMG160_RATE_Y_LSB_VALUEY__MSK        0xFF
#define BMG160_RATE_Y_LSB_VALUEY__REG        BMG160_RATE_Y_LSB_ADDR

/* Rate Z LSB Register */
/**< Last 8 bits of RateZ LSB Registers */
#define BMG160_RATE_Z_LSB_VALUEZ__POS        0
#define BMG160_RATE_Z_LSB_VALUEZ__LEN        8
#define BMG160_RATE_Z_LSB_VALUEZ__MSK        0xFF
#define BMG160_RATE_Z_LSB_VALUEZ__REG        BMG160_RATE_Z_LSB_ADDR

/**< First 3 bits of range Registers */
#define BMG160_RANGE_ADDR_RANGE__POS           0
#define BMG160_RANGE_ADDR_RANGE__LEN           3
#define BMG160_RANGE_ADDR_RANGE__MSK           0x07
#define BMG160_RANGE_ADDR_RANGE__REG           BMG160_RANGE_ADDR

/**< Last bit of Bandwidth Registers */
#define BMG160_BW_ADDR_HIGH_RES__POS       7
#define BMG160_BW_ADDR_HIGH_RES__LEN       1
#define BMG160_BW_ADDR_HIGH_RES__MSK       0x80
#define BMG160_BW_ADDR_HIGH_RES__REG       BMG160_BW_ADDR

/**< First 3 bits of Bandwidth Registers */
#define BMG160_BW_ADDR__POS             0
#define BMG160_BW_ADDR__LEN             3
#define BMG160_BW_ADDR__MSK             0x07
#define BMG160_BW_ADDR__REG             BMG160_BW_ADDR

/**< 6th bit of Bandwidth Registers */
#define BMG160_BW_ADDR_IMG_STB__POS             6
#define BMG160_BW_ADDR_IMG_STB__LEN             1
#define BMG160_BW_ADDR_IMG_STB__MSK             0x40
#define BMG160_BW_ADDR_IMG_STB__REG             BMG160_BW_ADDR

/**< 5th and 7th bit of LPM1 Register */
#define BMG160_MODE_LPM1__POS             5
#define BMG160_MODE_LPM1__LEN             3
#define BMG160_MODE_LPM1__MSK             0xA0
#define BMG160_MODE_LPM1__REG             BMG160_MODE_LPM1_ADDR

/**< 1st to 3rd bit of LPM1 Register */
#define BMG160_MODELPM1_ADDR_SLEEPDUR__POS              1
#define BMG160_MODELPM1_ADDR_SLEEPDUR__LEN              3
#define BMG160_MODELPM1_ADDR_SLEEPDUR__MSK              0x0E
#define BMG160_MODELPM1_ADDR_SLEEPDUR__REG              BMG160_MODE_LPM1_ADDR

/**< 7th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__POS         7
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__LEN         1
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__MSK         0x80
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__REG         BMG160_MODE_LPM2_ADDR

/**< 6th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__POS      6
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__LEN      1
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__MSK      0x40
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__REG      BMG160_MODE_LPM2_ADDR

/**< 4th & 5th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SEL__POS          4
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SEL__LEN          2
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SEL__MSK          0x30
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SEL__REG          BMG160_MODE_LPM2_ADDR

/**< 0th to 2nd bit of LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR__POS  0
#define BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR__LEN  3
#define BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR__MSK  0x07
#define BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR__REG  BMG160_MODE_LPM2_ADDR

/**< 7th bit of HBW Register */
#define BMG160_RATED_HBW_ADDR_DATA_HIGHBW__POS         7
#define BMG160_RATED_HBW_ADDR_DATA_HIGHBW__LEN         1
#define BMG160_RATED_HBW_ADDR_DATA_HIGHBW__MSK         0x80
#define BMG160_RATED_HBW_ADDR_DATA_HIGHBW__REG         BMG160_RATED_HBW_ADDR

/**< 6th bit of HBW Register */
#define BMG160_RATED_HBW_ADDR_SHADOW_DIS__POS          6
#define BMG160_RATED_HBW_ADDR_SHADOW_DIS__LEN          1
#define BMG160_RATED_HBW_ADDR_SHADOW_DIS__MSK          0x40
#define BMG160_RATED_HBW_ADDR_SHADOW_DIS__REG          BMG160_RATED_HBW_ADDR

/**< 7th bit of Interrupt Enable 0 Registers */
#define BMG160_INT_ENABLE0_DATAEN__POS               7
#define BMG160_INT_ENABLE0_DATAEN__LEN               1
#define BMG160_INT_ENABLE0_DATAEN__MSK               0x80
#define BMG160_INT_ENABLE0_DATAEN__REG               BMG160_INT_ENABLE0_ADDR

/**< 3rd bit of Interrupt Enable 1 Registers */
#define BMG160_INT_ENABLE1_IT2_OD__POS               3
#define BMG160_INT_ENABLE1_IT2_OD__LEN               1
#define BMG160_INT_ENABLE1_IT2_OD__MSK               0x08
#define BMG160_INT_ENABLE1_IT2_OD__REG               BMG160_INT_ENABLE1_ADDR

/**< 2nd bit of Interrupt Enable 1 Registers */
#define BMG160_INT_ENABLE1_IT2_LVL__POS              2
#define BMG160_INT_ENABLE1_IT2_LVL__LEN              1
#define BMG160_INT_ENABLE1_IT2_LVL__MSK              0x04
#define BMG160_INT_ENABLE1_IT2_LVL__REG              BMG160_INT_ENABLE1_ADDR

/**< 1st bit of Interrupt Enable 1 Registers */
#define BMG160_INT_ENABLE1_IT1_OD__POS               1
#define BMG160_INT_ENABLE1_IT1_OD__LEN               1
#define BMG160_INT_ENABLE1_IT1_OD__MSK               0x02
#define BMG160_INT_ENABLE1_IT1_OD__REG               BMG160_INT_ENABLE1_ADDR

/**< 0th bit of Interrupt Enable 1 Registers */
#define BMG160_INT_ENABLE1_IT1_LVL__POS              0
#define BMG160_INT_ENABLE1_IT1_LVL__LEN              1
#define BMG160_INT_ENABLE1_IT1_LVL__MSK              0x01
#define BMG160_INT_ENABLE1_IT1_LVL__REG              BMG160_INT_ENABLE1_ADDR

/**< 7th bit of MAP_1Registers */
#define BMG160_MAP_1_INT2_DATA__POS                  7
#define BMG160_MAP_1_INT2_DATA__LEN                  1
#define BMG160_MAP_1_INT2_DATA__MSK                  0x80
#define BMG160_MAP_1_INT2_DATA__REG                  BMG160_INT_MAP_1_ADDR

/**< 0th bit of MAP_1Registers */
#define BMG160_MAP_1_INT1_DATA__POS                  0
#define BMG160_MAP_1_INT1_DATA__LEN                  1
#define BMG160_MAP_1_INT1_DATA__MSK                  0x01
#define BMG160_MAP_1_INT1_DATA__REG                  BMG160_INT_MAP_1_ADDR

/**< 3nd bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_EN__POS        3
#define BMG160_FAST_OFFSET_EN__LEN        1
#define BMG160_FAST_OFFSET_EN__MSK        0x08
#define BMG160_FAST_OFFSET_EN__REG        BMG160_A_FOC_ADDR

/**< 2nd bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_EN_Z__POS      2
#define BMG160_FAST_OFFSET_EN_Z__LEN      1
#define BMG160_FAST_OFFSET_EN_Z__MSK      0x04
#define BMG160_FAST_OFFSET_EN_Z__REG      BMG160_A_FOC_ADDR

/**< 1st bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_EN_Y__POS      1
#define BMG160_FAST_OFFSET_EN_Y__LEN      1
#define BMG160_FAST_OFFSET_EN_Y__MSK      0x02
#define BMG160_FAST_OFFSET_EN_Y__REG      BMG160_A_FOC_ADDR

/**< 0th bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_EN_X__POS      0
#define BMG160_FAST_OFFSET_EN_X__LEN      1
#define BMG160_FAST_OFFSET_EN_X__MSK      0x01
#define BMG160_FAST_OFFSET_EN_X__REG      BMG160_A_FOC_ADDR

/**< 0 to 2 bits of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_EN_XYZ__POS      0
#define BMG160_FAST_OFFSET_EN_XYZ__LEN      3
#define BMG160_FAST_OFFSET_EN_XYZ__MSK      0x07
#define BMG160_FAST_OFFSET_EN_XYZ__REG      BMG160_A_FOC_ADDR

 /**< Last 2 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_X__POS       4
#define BMG160_OFC1_ADDR_OFFSET_X__LEN       2
#define BMG160_OFC1_ADDR_OFFSET_X__MSK       0xC0
#define BMG160_OFC1_ADDR_OFFSET_X__REG       BMG160_OFC1_ADDR

/**< 3 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_Y__POS       2
#define BMG160_OFC1_ADDR_OFFSET_Y__LEN       3
#define BMG160_OFC1_ADDR_OFFSET_Y__MSK       0x38
#define BMG160_OFC1_ADDR_OFFSET_Y__REG       BMG160_OFC1_ADDR

/**< First 3 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_Z__POS       1
#define BMG160_OFC1_ADDR_OFFSET_Z__LEN       3
#define BMG160_OFC1_ADDR_OFFSET_Z__MSK       0x07
#define BMG160_OFC1_ADDR_OFFSET_Z__REG       BMG160_OFC1_ADDR

/**< 2 bits of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__POS       2
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__LEN       2
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__MSK       0x0C
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__REG       BMG160_TRIM_GP0_ADDR

/**< 1st bit of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__POS       1
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__LEN       1
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__MSK       0x02
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG       BMG160_TRIM_GP0_ADDR

/**< First bit of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__POS       0
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__LEN       1
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__MSK       0x01
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG       BMG160_TRIM_GP0_ADDR

/* For Axis Selection   */
/**< It refers BMG160 X-axis */
#define BMG160_X_AXIS           0
/**< It refers BMG160 Y-axis */
#define BMG160_Y_AXIS           1
/**< It refers BMG160 Z-axis */
#define BMG160_Z_AXIS           2

/* For Mode Settings    */
#define BMG160_MODE_NORMAL              0
#define BMG160_MODE_DEEPSUSPEND         1
#define BMG160_MODE_SUSPEND             2
#define BMG160_MODE_FASTPOWERUP			3
#define BMG160_MODE_ADVANCEDPOWERSAVING 4

/* get bit slice  */
#define BMG160_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)
#define BMG160_GET_BITSLICE_LEFT(regvar, bitname)\
((regvar & bitname##__MSK) << bitname##__POS)

/* Set bit slice */
#define BMG160_SET_BITSLICE(regvar, bitname, val)\
((regvar&~bitname##__MSK)|((val<<bitname##__POS)&bitname##__MSK))
#define BMG160_SET_BITSLICE_RIGHT(regvar, bitname, val)\
((regvar&~bitname##__MSK)|((val>>bitname##__POS)&bitname##__MSK))
/* Constants */

#define BMG160_NULL                             0
/**< constant declaration of NULL */
#define BMG160_DISABLE                          0
/**< It refers BMG160 disable */
#define BMG160_ENABLE                           1
/**< It refers BMG160 enable */
#define BMG160_OFF                              0
/**< It refers BMG160 OFF state */
#define BMG160_ON                               1
/**< It refers BMG160 ON state  */


#define BMG160_TURN1                            0
/**< It refers BMG160 TURN1 */
#define BMG160_TURN2                            1
/**< It refers BMG160 TURN2 */

#define BMG160_INT1                             0
/**< It refers BMG160 INT1 */
#define BMG160_INT2                             1
/**< It refers BMG160 INT2 */

#define BMG160_INT1_DATA                        0
/**< It refers BMG160 Int1 Data */
#define BMG160_INT2_DATA                        1
/**< It refers BMG160 Int2 Data */

#define C_BMG160_SUCCESS						0
/**< It refers BMG160 operation is success */
#define C_BMG160_FAILURE						1
/**< It refers BMG160 operation is Failure */

#define BMG160_SPI_RD_MASK                      0x80
/**< Read mask **/
#define BMG160_READ_SET                         0x01
/**< Setting for rading data **/

#define BMG160_SHIFT_1_POSITION                 1
/**< Shift bit by 1 Position **/
#define BMG160_SHIFT_2_POSITION                 2
/**< Shift bit by 2 Position **/
#define BMG160_SHIFT_3_POSITION                 3
/**< Shift bit by 3 Position **/
#define BMG160_SHIFT_4_POSITION                 4
/**< Shift bit by 4 Position **/
#define BMG160_SHIFT_5_POSITION                 5
/**< Shift bit by 5 Position **/
#define BMG160_SHIFT_6_POSITION                 6
/**< Shift bit by 6 Position **/
#define BMG160_SHIFT_7_POSITION                 7
/**< Shift bit by 7 Position **/
#define BMG160_SHIFT_8_POSITION                 8
/**< Shift bit by 8 Position **/
#define BMG160_SHIFT_12_POSITION                12
/**< Shift bit by 12 Position **/

#define         C_BMG160_Null_U8X                              0
#define         C_BMG160_Zero_U8X                              0
#define         C_BMG160_One_U8X                               1
#define         C_BMG160_Two_U8X                               2
#define         C_BMG160_Three_U8X                             3
#define         C_BMG160_Four_U8X                              4
#define         C_BMG160_Five_U8X                              5
#define         C_BMG160_Six_U8X                               6
#define         C_BMG160_Seven_U8X                             7
#define         C_BMG160_Eight_U8X                             8
#define         C_BMG160_Nine_U8X                              9
#define         C_BMG160_Ten_U8X                               10

#define E_BMG160_NULL_PTR               (char)(-127)
#define E_BMG160_COMM_RES               (char)(-1)
#define E_BMG160_OUT_OF_RANGE           (signed char)(-2)

#define C_BMG160_No_Filter_U8X			0
#define	C_BMG160_BW_230Hz_U8X			1
#define	C_BMG160_BW_116Hz_U8X			2
#define	C_BMG160_BW_47Hz_U8X			3
#define	C_BMG160_BW_23Hz_U8X			4
#define	C_BMG160_BW_12Hz_U8X			5
#define	C_BMG160_BW_64Hz_U8X			6
#define	C_BMG160_BW_32Hz_U8X			7

#define C_BMG160_No_AutoSleepDur_U8X	0
#define	C_BMG160_4ms_AutoSleepDur_U8X	1
#define	C_BMG160_5ms_AutoSleepDur_U8X	2
#define	C_BMG160_8ms_AutoSleepDur_U8X	3
#define	C_BMG160_10ms_AutoSleepDur_U8X	4
#define	C_BMG160_15ms_AutoSleepDur_U8X	5
#define	C_BMG160_20ms_AutoSleepDur_U8X	6
#define	C_BMG160_40ms_AutoSleepDur_U8X	7

#define BMG160_WR_FUNC_PTR char (*bus_write)\
(unsigned char, unsigned char, unsigned char *, unsigned char)
#define BMG160_RD_FUNC_PTR char (*bus_read)\
(unsigned char, unsigned char, unsigned char *, unsigned char)
#define BMG160_BRD_FUNC_PTR char (*burst_read)\
(unsigned char, unsigned char, unsigned char *, BMG160_S32)
#define BMG160_MDELAY_DATA_TYPE BMG160_S32

/*user defined Structures*/
struct bmg160_data_t {
		BMG160_S16 datax;
		BMG160_S16 datay;
		BMG160_S16 dataz;
		char intstatus[5];
};

struct bmg160_offset_t {
		BMG160_U16 datax;
		BMG160_U16 datay;
		BMG160_U16 dataz;
};

struct bmg160_t {
		unsigned char chip_id;
		unsigned char dev_addr;
		BMG160_BRD_FUNC_PTR;
		BMG160_WR_FUNC_PTR;
		BMG160_RD_FUNC_PTR;
		void(*delay_msec)(BMG160_MDELAY_DATA_TYPE);
};

struct bmg160_t *p_bmg160;

struct op_mode_map {
	char *op_mode_name;
	long op_mode;
};

struct bmg_client_data {
	struct bmg160_t device;
	struct i2c_client *client;
	struct input_dev *input;
	struct bst_dev *bst_gyro;
	struct delayed_work work;

	atomic_t delay;

	struct bmg160_data_t value;
	u8 enable:1;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;

	struct work_struct irq_work;
	int sample_count;
	int IRQ;
	int irq_gpio;
	int irq_enable;
};

struct bmg160_regulator_data {
	struct regulator *reg_vdd;
	struct regulator *reg_vio;
	char *id_vdd;
	char *id_vio;
};

static const struct op_mode_map op_mode_maps[] = {
	{"normal", BMG_VAL_NAME(MODE_NORMAL)},
	{"deepsuspend", BMG_VAL_NAME(MODE_DEEPSUSPEND)},
	{"suspend", BMG_VAL_NAME(MODE_SUSPEND)},
	{"fastpowerup", BMG_VAL_NAME(MODE_FASTPOWERUP)},
	{"advancedpowersav", BMG_VAL_NAME(MODE_ADVANCEDPOWERSAVING)},
};

static struct i2c_client *bmg_client;
static struct bmg160_regulator_data bmg160_regu_data;

/* i2c operation for API */
static void bmg_i2c_delay(BMG160_S32 msec);
static char bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);
static char bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len);

static void bmg_dump_reg(struct i2c_client *client);
static int bmg_check_chip_id(struct i2c_client *client);

static int bmg160_get_mode(unsigned char *Mode);
static int bmg160_set_autosleepdur(unsigned char duration, \
		unsigned char bandwith);
static int bmg160_get_autosleepdur(unsigned char *duration);


static int bmg160_init(struct bmg160_t *bmg160)
{
	int comres = 0;
	unsigned char a_data_u8r;
	p_bmg160 = bmg160;

	p_bmg160->dev_addr = BMG160_I2C_ADDR;

	/*Read CHIP_ID */
	comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,\
	 BMG160_CHIP_ID_ADDR, &a_data_u8r, 1);
	p_bmg160->chip_id = a_data_u8r;
	return comres;
}

static int bmg160_get_dataXYZ(struct bmg160_data_t *data)
{
	int comres;
	unsigned char a_data_u8r[6];
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,\
		 BMG160_RATE_X_LSB_VALUEX__REG, a_data_u8r, 6);
		/* Data X */
		a_data_u8r[0] = \
		BMG160_GET_BITSLICE(a_data_u8r[0], BMG160_RATE_X_LSB_VALUEX);
		data->datax = (BMG160_S16)\
		((((BMG160_S16)((signed char)a_data_u8r[1]))\
		<< BMG160_SHIFT_8_POSITION) | (a_data_u8r[0]));
		/* Data Y */
		a_data_u8r[2] = BMG160_GET_BITSLICE(a_data_u8r[2],\
		BMG160_RATE_Y_LSB_VALUEY);
		data->datay = (BMG160_S16)\
		((((BMG160_S16)((signed char)a_data_u8r[3]))\
		<< BMG160_SHIFT_8_POSITION) | (a_data_u8r[2]));
		/* Data Z */
		a_data_u8r[4] = BMG160_GET_BITSLICE(a_data_u8r[4],\
		BMG160_RATE_Z_LSB_VALUEZ);
		data->dataz = (BMG160_S16)\
		((((BMG160_S16)((signed char)a_data_u8r[5]))\
		<< BMG160_SHIFT_8_POSITION) | (a_data_u8r[4]));
	}
	return comres;
}

static int bmg160_get_range_reg(unsigned char *range)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC\
		(p_bmg160->dev_addr,\
		BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8r, 1);
		*range =\
		BMG160_GET_BITSLICE(v_data_u8r, BMG160_RANGE_ADDR_RANGE);
	}
	return comres;
}

static int bmg160_set_range_reg(unsigned char range)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		if (range < C_BMG160_Five_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
			(p_bmg160->dev_addr,\
			BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8r, 1);
			v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
			BMG160_RANGE_ADDR_RANGE,\
			range);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
			(p_bmg160->dev_addr,\
			BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8r, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}

static int bmg160_get_bw(unsigned char *bandwidth)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC\
		(p_bmg160->dev_addr, BMG160_BW_ADDR__REG, &v_data_u8r, 1);
		*bandwidth = BMG160_GET_BITSLICE(v_data_u8r,\
			BMG160_BW_ADDR);
	}
	return comres;
}

static int bmg160_set_bw(unsigned char bandwidth)
{
	int comres;
	unsigned char v_data_u8r;
	unsigned char v_mode_u8r;
	unsigned char v_autosleepduration = 0;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		if (bandwidth < C_BMG160_Eight_U8X) {
			bmg160_get_mode(&v_mode_u8r);
			if (v_mode_u8r == BMG160_MODE_ADVANCEDPOWERSAVING) {
				bmg160_get_autosleepdur(&v_autosleepduration);
				bmg160_set_autosleepdur(v_autosleepduration, \
				bandwidth);
			}
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
			(p_bmg160->dev_addr,\
			BMG160_BW_ADDR__REG, &v_data_u8r, 1);
			v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
				BMG160_BW_ADDR, bandwidth);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_BW_ADDR__REG, &v_data_u8r, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}

static int bmg160_set_data_enable(unsigned char data_en)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC\
			(p_bmg160->dev_addr,\
		BMG160_INT_ENABLE0_DATAEN__REG, &v_data_u8r, 1);
		v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
		BMG160_INT_ENABLE0_DATAEN, data_en);
		comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
			(p_bmg160->dev_addr,\
		BMG160_INT_ENABLE0_DATAEN__REG, &v_data_u8r, 1);
	}
	return comres;
}

static int bmg160_set_int_od(unsigned char param, unsigned char int_od)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		switch (param) {
		case BMG160_INT1:
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_INT_ENABLE1_IT1_OD__REG, &v_data_u8r, 1);
			v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
			BMG160_INT_ENABLE1_IT1_OD, int_od);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_INT_ENABLE1_IT1_OD__REG, &v_data_u8r, 1);
			break;
		case BMG160_INT2:
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_INT_ENABLE1_IT2_OD__REG, &v_data_u8r, 1);
			v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
			BMG160_INT_ENABLE1_IT2_OD, int_od);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_INT_ENABLE1_IT2_OD__REG, &v_data_u8r, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

static int bmg160_set_int_data(unsigned char axis, unsigned char int_data)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		switch (axis) {
		case BMG160_INT1_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MAP_1_INT1_DATA__REG, &v_data_u8r, 1);
			v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
			BMG160_MAP_1_INT1_DATA, int_data);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MAP_1_INT1_DATA__REG, &v_data_u8r, 1);
			break;
		case BMG160_INT2_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MAP_1_INT2_DATA__REG, &v_data_u8r, 1);
			v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
			BMG160_MAP_1_INT2_DATA, int_data);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MAP_1_INT2_DATA__REG, &v_data_u8r, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

static int bmg160_get_mode(unsigned char *Mode)
{
	int comres = C_BMG160_Zero_U8X ;
	unsigned char data1 = C_BMG160_Zero_U8X;
	unsigned char data2 = C_BMG160_Zero_U8X;
	unsigned char data3 = C_BMG160_Zero_U8X;
	if (p_bmg160 == C_BMG160_Zero_U8X) {
		comres = E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr, \
		BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_One_U8X);
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr, \
		BMG160_MODE_LPM2_ADDR, &data2, C_BMG160_One_U8X);
		data1  = (data1 & 0xA0) >> 5;
		data3  = (data2 & 0x40) >> 6;
		data2  = (data2 & 0x80) >> 7;
		if (data3 == 0x01) {
			*Mode  = BMG160_MODE_ADVANCEDPOWERSAVING;
		} else {
			if ((data1 == 0x00) && (data2 == 0x00)) {
				*Mode  = BMG160_MODE_NORMAL;
				} else {
				if ((data1 == 0x01) || (data1 == 0x05)) {
					*Mode  = BMG160_MODE_DEEPSUSPEND;
					} else {
					if ((data1 == 0x04) &&\
					(data2 == 0x00)) {
						*Mode  = BMG160_MODE_SUSPEND;
					} else {
					if ((data1 == 0x04) &&\
						(data2 == 0x01))
						*Mode  =\
						BMG160_MODE_FASTPOWERUP;
					}
					}
				}
			}
		}
	return comres;
}

static int bmg160_set_mode(unsigned char Mode)
{
	int comres = C_BMG160_Zero_U8X ;
	unsigned char data1;
	unsigned char data2;
	unsigned char data3;
	unsigned char v_autosleepduration = 0;
	unsigned char v_bw_u8r = 0;
	if (p_bmg160 == C_BMG160_Zero_U8X) {
		comres = E_BMG160_NULL_PTR;
	} else {
		if (Mode < C_BMG160_Five_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_One_U8X);
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR, &data2, C_BMG160_One_U8X);
			switch (Mode) {
			case BMG160_MODE_NORMAL:
				data1  = BMG160_SET_BITSLICE(data1,\
				BMG160_MODE_LPM1, C_BMG160_Zero_U8X);
				data2  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,\
				C_BMG160_Zero_U8X);
				data3  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,\
				C_BMG160_Zero_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_One_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_One_U8X);
				break;
			case BMG160_MODE_DEEPSUSPEND:
				data1  = BMG160_SET_BITSLICE(data1,\
				BMG160_MODE_LPM1, C_BMG160_One_U8X);
				data2  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,\
				C_BMG160_Zero_U8X);
				data3  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,\
				C_BMG160_Zero_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_One_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_One_U8X);
				break;
			case BMG160_MODE_SUSPEND:
				data1  = BMG160_SET_BITSLICE(data1,\
				BMG160_MODE_LPM1, C_BMG160_Four_U8X);
				data2  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,\
				C_BMG160_Zero_U8X);
				data3  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,\
				C_BMG160_Zero_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_One_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_One_U8X);
				break;
			case BMG160_MODE_FASTPOWERUP:
				data1  = BMG160_SET_BITSLICE(data1,\
				BMG160_MODE_LPM1, C_BMG160_Four_U8X);
				data2  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP, \
				C_BMG160_One_U8X);
				data3  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,\
				C_BMG160_Zero_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_One_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_One_U8X);
				break;
			case BMG160_MODE_ADVANCEDPOWERSAVING:
				/* Configuring the proper settings for auto
				sleep duration */
				bmg160_get_bw(&v_bw_u8r);
				bmg160_get_autosleepdur(&v_autosleepduration);
				bmg160_set_autosleepdur(v_autosleepduration, \
				v_bw_u8r);
				comres += p_bmg160->BMG160_BUS_READ_FUNC\
					(p_bmg160->dev_addr,\
				BMG160_MODE_LPM2_ADDR, &data2,\
				C_BMG160_One_U8X);
				/* Configuring the advanced power saving mode*/
				data1  = BMG160_SET_BITSLICE(data1,\
				BMG160_MODE_LPM1, C_BMG160_Zero_U8X);
				data2  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP, \
				C_BMG160_Zero_U8X);
				data3  = BMG160_SET_BITSLICE(data2,\
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,\
				C_BMG160_One_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_One_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_One_U8X);
				break;
				}
		} else {
		comres = E_BMG160_OUT_OF_RANGE ;
		}
	}
	return comres;
}

static int bmg160_get_autosleepdur(unsigned char *duration)
{
	int comres;
	unsigned char v_data_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,\
		 BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR__REG, &v_data_u8r, 1);
		*duration = BMG160_GET_BITSLICE(v_data_u8r,\
		BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR);
	}
	return comres;
}

static int bmg160_set_autosleepdur(unsigned char duration,
				unsigned char bandwith)
{
	int comres;
	unsigned char v_data_u8r;
	unsigned char v_autosleepduration_u8r = 0;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC\
			(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR__REG,\
			&v_data_u8r, 1);
			if (duration < C_BMG160_Eight_U8X) {
				switch (bandwith) {
				case C_BMG160_No_Filter_U8X:
					if (duration >
					C_BMG160_4ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
						v_autosleepduration_u8r =\
						C_BMG160_4ms_AutoSleepDur_U8X;
					break;
				case C_BMG160_BW_230Hz_U8X:
					if (duration >
					C_BMG160_4ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
						v_autosleepduration_u8r =\
						C_BMG160_4ms_AutoSleepDur_U8X;
					break;
				case C_BMG160_BW_116Hz_U8X:
					if (duration >
					C_BMG160_4ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
						v_autosleepduration_u8r =\
						C_BMG160_4ms_AutoSleepDur_U8X;
					break;
				case C_BMG160_BW_47Hz_U8X:
					if (duration >
					C_BMG160_5ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
						v_autosleepduration_u8r =\
						C_BMG160_5ms_AutoSleepDur_U8X;
					break;
				case C_BMG160_BW_23Hz_U8X:
					if (duration >
					C_BMG160_10ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
						v_autosleepduration_u8r =\
						C_BMG160_10ms_AutoSleepDur_U8X;
					break;
				case C_BMG160_BW_12Hz_U8X:
					if (duration >
					C_BMG160_20ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
					v_autosleepduration_u8r =\
					C_BMG160_20ms_AutoSleepDur_U8X;
					break;
				case C_BMG160_BW_64Hz_U8X:
					if (duration >
					C_BMG160_10ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
						v_autosleepduration_u8r =\
						C_BMG160_10ms_AutoSleepDur_U8X;
					break;
				case C_BMG160_BW_32Hz_U8X:
					if (duration >
					C_BMG160_20ms_AutoSleepDur_U8X)
						v_autosleepduration_u8r =\
						duration;
					else
						v_autosleepduration_u8r =\
						C_BMG160_20ms_AutoSleepDur_U8X;
					break;
				}
			v_data_u8r = BMG160_SET_BITSLICE(v_data_u8r,\
			BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR,\
			v_autosleepduration_u8r);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
			BMG160_MODE_LPM2_ADDR_AUTOSLEEPDUR__REG,\
			&v_data_u8r, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}

static int bmg160_get_offset(unsigned char axis, short *offset)
{
	int comres;
	unsigned char v_data1_u8r, v_data2_u8r;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		switch (axis) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_X__REG,\
				&v_data1_u8r, 1);
			v_data1_u8r = BMG160_GET_BITSLICE(v_data1_u8r,\
				BMG160_TRIM_GP0_ADDR_OFFSET_X);

			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_X__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_GET_BITSLICE(v_data2_u8r,\
				BMG160_OFC1_ADDR_OFFSET_X);

			v_data2_u8r = (v_data2_u8r | v_data1_u8r);

			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr, BMG160_OFC2_ADDR,\
				&v_data1_u8r, 1);
			*offset = (BMG160_S16)((((BMG160_S16)\
				((signed char)v_data1_u8r))\
				<< BMG160_SHIFT_4_POSITION) | (v_data2_u8r));
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG,\
				&v_data1_u8r, 1);
			v_data1_u8r = BMG160_GET_BITSLICE(v_data1_u8r,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Y);

			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_Y__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_GET_BITSLICE(v_data2_u8r,\
				BMG160_OFC1_ADDR_OFFSET_Y);

			v_data2_u8r = (v_data2_u8r | v_data1_u8r);

			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC3_ADDR, &v_data1_u8r, 1);
			*offset = (BMG160_S16)((((BMG160_S16)\
				((signed char)v_data1_u8r))\
				<< BMG160_SHIFT_4_POSITION) | (v_data2_u8r));
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG,\
				&v_data1_u8r, 1);
			v_data1_u8r = BMG160_GET_BITSLICE(v_data1_u8r,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Z);

			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_Z__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_GET_BITSLICE_LEFT(v_data2_u8r,\
				BMG160_OFC1_ADDR_OFFSET_Z);

			v_data2_u8r = (v_data2_u8r | v_data1_u8r);

			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC4_ADDR, &v_data1_u8r, 1);
			*offset = (BMG160_S16)((((BMG160_S16)\
				((signed char)v_data1_u8r))\
				<< BMG160_SHIFT_4_POSITION) | (v_data2_u8r));
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

static int bmg160_set_offset(unsigned char axis, short offset)
{
	int comres;
	unsigned char v_data1_u8r = 0, v_data2_u8r = 0;
	if (p_bmg160 == BMG160_NULL) {
		comres = E_BMG160_NULL_PTR;
	} else {
		switch (axis) {
		case BMG160_X_AXIS:
			v_data1_u8r = (signed char) ((offset & 0x0FF0)\
			>> BMG160_SHIFT_4_POSITION);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC2_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x000C);
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_X__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,\
				BMG160_OFC1_ADDR_OFFSET_X, v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_X__REG,\
				&v_data2_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x0003);
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_X__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,\
				BMG160_TRIM_GP0_ADDR_OFFSET_X, v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_X__REG,\
				&v_data2_u8r, 1);
			break;
		case BMG160_Y_AXIS:
			v_data1_u8r = (signed char) ((offset & 0x0FF0) >>\
			BMG160_SHIFT_4_POSITION);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC3_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x000E);
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_Y__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,\
				BMG160_OFC1_ADDR_OFFSET_Y, v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_Y__REG,\
				&v_data2_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x0001);
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Y, v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG,\
				&v_data2_u8r, 1);
			break;
		case BMG160_Z_AXIS:
			v_data1_u8r = (signed char) ((offset & 0x0FF0) >>\
			BMG160_SHIFT_4_POSITION);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC4_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x000E);
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_Z__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_SET_BITSLICE_RIGHT(v_data2_u8r,\
				BMG160_OFC1_ADDR_OFFSET_Z, v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_OFC1_ADDR_OFFSET_Z__REG,\
				&v_data2_u8r, 1);

			v_data1_u8r = (unsigned char) (offset & 0x0001);
			comres = p_bmg160->BMG160_BUS_READ_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG,\
				&v_data2_u8r, 1);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Z, v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC\
				(p_bmg160->dev_addr,\
				BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG,\
				&v_data2_u8r, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

static int bmg_check_chip_id(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;

	bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), &chip_id, 1);
	dev_info(&client->dev, "read chip id result: %#x", chip_id);

	if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMG)
		err = -EPERM;

	return err;
}

static void bmg_i2c_delay(BMG160_S32 msec)
{
	msleep(msec);
}

static void bmg_dump_reg(struct i2c_client *client)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		scnprintf(dbg_buf_str + i * 3, buff_size, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);

	bmg_i2c_read(client, BMG_REG_NAME(CHIP_ID_ADDR), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		scnprintf(dbg_buf_str + i * 3, buff_size, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	dev_dbg(&client->dev, "%s\n", dbg_buf_str);
}

/*	i2c read routine for API*/
static char bmg_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -EPERM;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			dev_err(&client->dev, "i2c bus read error");
			return -EPERM;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
			return -EPERM;

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
			return -EPERM;
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			msleep(BMG_I2C_WRITE_DELAY_TIME);
	}

	if (BMG_MAX_RETRY_I2C_XFER <= retry) {
		dev_err(&client->dev, "I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}


/*	i2c write routine for */
static char bmg_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMG_USE_BASIC_I2C_FUNC
	s32 dummy;

#ifndef BMG_SMBUS
	u8 buffer[2];
#endif

	if (NULL == client)
		return -EPERM;

	while (0 != len--) {
#ifdef BMG_SMBUS
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
#endif
		reg_addr++;
		data++;
		if (dummy < 0) {
			dev_err(&client->dev, "error writing i2c bus");
			return -EPERM;
		}

	}
	return 0;
#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
		 },
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < BMG_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				msleep(BMG_I2C_WRITE_DELAY_TIME);
			}
		}
		if (BMG_MAX_RETRY_I2C_XFER <= retry) {
			dev_err(&client->dev, "I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
#endif
}

static char bmg_i2c_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err;
	err = bmg_i2c_read(bmg_client, reg_addr, data, len);
	return err;
}

static char bmg_i2c_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	char err;
	err = bmg_i2c_write(bmg_client, reg_addr, data, len);
	return err;
}

static void bmg_work_func(struct work_struct *work)
{
	struct bmg_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bmg_client_data, work);

	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));

	BMG_CALL_API(get_dataXYZ)(&client_data->value);

	input_event(client_data->input, EV_MSC, MSC_RX,
					client_data->value.datax);
	input_event(client_data->input, EV_MSC, MSC_RY,
					client_data->value.datay);
	input_event(client_data->input, EV_MSC, MSC_RZ,
					client_data->value.dataz);
	input_sync(client_data->input);

	schedule_delayed_work(&client_data->work, delay);

}

static ssize_t bmg_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	u8 op_mode = 0xff;

	mutex_lock(&client_data->mutex_op_mode);
	BMG_CALL_API(get_mode)(&op_mode);
	mutex_unlock(&client_data->mutex_op_mode);

	ret = scnprintf(buf, buff_size, "%d\n", op_mode);

	return ret;
}

static ssize_t bmg_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	dev_info(&client_data->client->dev, "%s:set op-mode[%ld]",
		__func__, op_mode);

	mutex_lock(&client_data->mutex_op_mode);

	err = BMG_CALL_API(set_mode)(op_mode);

	mutex_unlock(&client_data->mutex_op_mode);

	if (err)
		return err;
	else
		return count;
}

static ssize_t bmg_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int count;

	BMG_CALL_API(get_dataXYZ)(&client_data->value);

	count = scnprintf(buf, buff_size, "%hd %hd %hd\n",
			client_data->value.datax,
			client_data->value.datay,
			client_data->value.dataz);

	return count;
}

static ssize_t bmg_show_offset_x(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	short offset = 0;

	if (bmg160_get_offset(BMG160_X_AXIS, &offset) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%hd\n", offset);
}

static ssize_t bmg_store_offset_x(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int error;
	long offset;

	error = kstrtol(buf, 10, &offset);
	if (error)
		return error;

	if (bmg160_set_offset(BMG160_X_AXIS, (short)offset) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bmg_show_offset_y(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	short offset = 0;

	if (bmg160_get_offset(BMG160_Y_AXIS, &offset) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%hd\n", offset);
}

static ssize_t bmg_store_offset_y(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int error;
	long offset;

	error = kstrtol(buf, 10, &offset);
	if (error)
		return error;

	if (bmg160_set_offset(BMG160_Y_AXIS, (short)offset) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bmg_show_offset_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	short offset = 0;

	if (bmg160_get_offset(BMG160_Z_AXIS, &offset) < 0)
		return scnprintf(buf, buff_size, "Read error\n");

	return scnprintf(buf, buff_size, "%hd\n", offset);
}

static ssize_t bmg_store_offset_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int error;
	long offset;

	error = kstrtol(buf, 10, &offset);
	if (error)
		return error;

	if (bmg160_set_offset(BMG160_Z_AXIS, (short)offset) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bmg_show_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char range = 0;
	BMG_CALL_API(get_range_reg)(&range);
	err = scnprintf(buf, buff_size, "%d\n", range);
	return err;
}

static ssize_t bmg_store_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long range;
	err = kstrtoul(buf, 10, &range);
	if (err)
		return err;
	BMG_CALL_API(set_range_reg)(range);
	return count;
}

static ssize_t bmg_show_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	unsigned char bandwidth = 0;
	BMG_CALL_API(get_bw)(&bandwidth);
	err = scnprintf(buf, buff_size, "%d\n", bandwidth);
	return err;
}

static ssize_t bmg_store_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	unsigned long bandwidth;
	err = kstrtoul(buf, 10, &bandwidth);
	if (err)
		return err;
	BMG_CALL_API(set_bw)(bandwidth);
	return count;
}

static ssize_t bmg_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->mutex_enable);
	err = scnprintf(buf, buff_size, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return err;
}

static ssize_t bmg_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (!client_data->irq_enable) {
			if (data) {
				BMG_CALL_API(set_mode)(
					BMG_VAL_NAME(MODE_NORMAL));
				schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
						&client_data->delay)));
				msleep(40);
			} else {
				cancel_delayed_work_sync(&client_data->work);
				BMG_CALL_API(set_mode)(
					BMG_VAL_NAME(MODE_SUSPEND));
			}
		} else {
			if (data) {
				BMG_CALL_API(set_mode)(
					BMG_VAL_NAME(MODE_NORMAL));
				usleep_range(10000, 11000);
				enable_irq(client_data->IRQ);
				BMG_CALL_API(set_data_enable)(INT_ENABLE);
			} else {
				BMG_CALL_API(set_data_enable)(INT_DISABLE);
				disable_irq(client_data->IRQ);
				BMG_CALL_API(set_mode)(
					BMG_VAL_NAME(MODE_SUSPEND));
			}
		}
		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);

	return count;
}

static ssize_t bmg_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, buff_size, "%d\n",
			atomic_read(&client_data->delay));

}

static ssize_t bmg_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmg_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data <= 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMG_DELAY_MIN)
		data = BMG_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}


static DEVICE_ATTR(op_mode, S_IRUSR|S_IWUSR,
		bmg_show_op_mode, bmg_store_op_mode);
static DEVICE_ATTR(value, S_IRUSR,
		bmg_show_value, NULL);
static DEVICE_ATTR(offset_x, S_IRUSR|S_IWUSR,
		bmg_show_offset_x, bmg_store_offset_x);
static DEVICE_ATTR(offset_y, S_IRUSR|S_IWUSR,
		bmg_show_offset_y, bmg_store_offset_y);
static DEVICE_ATTR(offset_z, S_IRUSR|S_IWUSR,
		bmg_show_offset_z, bmg_store_offset_z);
static DEVICE_ATTR(range, S_IRUSR|S_IWUSR,
		bmg_show_range, bmg_store_range);
static DEVICE_ATTR(bandwidth, S_IRUSR|S_IWUSR,
		bmg_show_bandwidth, bmg_store_bandwidth);
static DEVICE_ATTR(enable, S_IRUSR|S_IWUSR,
		bmg_show_enable, bmg_store_enable);
static DEVICE_ATTR(delay, S_IRUSR|S_IWUSR,
		bmg_show_delay, bmg_store_delay);

static struct attribute *bmg_attributes[] = {
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_offset_x.attr,
	&dev_attr_offset_y.attr,
	&dev_attr_offset_z.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	NULL
};

static struct attribute_group bmg_attribute_group = {
	.attrs = bmg_attributes
};

static int bmg_input_init(struct bmg_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_MSC, ABS_MISC);
	input_set_capability(dev, EV_MSC, MSC_RX);
	input_set_capability(dev, EV_MSC, MSC_RY);
	input_set_capability(dev, EV_MSC, MSC_RZ);
	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	client_data->input = dev;

	return 0;
}

static void bmg_input_destroy(struct bmg_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static void bmg_irq_work_func(struct work_struct *work)
{
	struct bmg_client_data *client_data =
			container_of((struct work_struct *)work,
			struct bmg_client_data, irq_work);

	mutex_lock(&client_data->mutex_enable);

	if (client_data->enable) {
		BMG_CALL_API(get_dataXYZ)(&client_data->value);
		input_event(client_data->input, EV_MSC, MSC_RX,
				client_data->value.datax);
		input_event(client_data->input, EV_MSC, MSC_RY,
				client_data->value.datay);
		input_event(client_data->input, EV_MSC, MSC_RZ,
				client_data->value.dataz);
		input_sync(client_data->input);
	}
	mutex_unlock(&client_data->mutex_enable);
	return;
}

static irqreturn_t bmg_interrupt_thread(int irq, void *handle)
{
	struct bmg_client_data *client_data = handle;

	/* downsampling here */
	if (((atomic_read(&client_data->delay) == 5)
		&& !(client_data->sample_count % 2))
	|| ((atomic_read(&client_data->delay) == 10)
		&& !(client_data->sample_count % 4))) {
		BMG_CALL_API(get_dataXYZ)(&client_data->value);
		input_event(client_data->input, EV_MSC, MSC_RX,
				client_data->value.datax);
		input_event(client_data->input, EV_MSC, MSC_RY,
				client_data->value.datay);
		input_event(client_data->input, EV_MSC, MSC_RZ,
				client_data->value.dataz);
		input_sync(client_data->input);
	}
	client_data->sample_count++;

	return IRQ_HANDLED;
}

static int bmg160_regulator_init(struct device *dev)
{

	int rc = 0;

	struct bmg160_regulator_data *regdata = &bmg160_regu_data;

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

static int  bmg160_regulator_control(bool enable, struct device *dev)
{
	int rc = 0;

	struct bmg160_regulator_data *regdata = &bmg160_regu_data;

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
	usleep_range(30000, 31000);

	return rc;
}

static int bmg_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned int irq_gpio_flags;
	unsigned int irq_enable;
	int err = 0;
	struct bmg_client_data *client_data = NULL;
	struct bst_dev  *dev_gyro;

	struct bmg160_regulator_data *regdata = &bmg160_regu_data;

	dev_info(&client->dev, "function entrance");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error!");
		err = -EIO;
		goto exit_err;
	}

	if (NULL == bmg_client) {
		bmg_client = client;
	} else {
		dev_err(&client->dev,
			"this driver does not support multiple clients");
		err = -EINVAL;
		goto exit_err;
	}

	client_data = kzalloc(sizeof(struct bmg_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		dev_err(&client->dev, "no memory available");
		err = -ENOMEM;
		goto exit_err;
	}

	err = of_property_read_u32(client->dev.of_node,
					"bmg160,irq_enable", &irq_enable);
	if (err) {
		dev_err(&client->dev,
			"%s: irq_enable get from device tree fail\n", __func__);
		irq_enable = 0;
	}
	client_data->irq_enable = irq_enable;

	if ((client_data->irq_enable == BMG160_ENABLE_INT1) ||
		(client_data->irq_enable == BMG160_ENABLE_INT2)) {
		client_data->irq_gpio = of_get_named_gpio_flags(
			client->dev.of_node, "bmg160,irq_gpio",
			0, &irq_gpio_flags);
		if (client_data->irq_gpio < 0) {
			dev_err(&client->dev,
				"%s: negative gpio number ", __func__);
			err = -EIO;
			goto exit_err_clean;
		}
		err = gpio_request(client_data->irq_gpio, SENSOR_NAME);
		if (err) {
			dev_err(&client->dev, "%s: gpio_request failed, err=%d\n",
					__func__, err);
			goto exit_err_clean;
		}
	}

	regdata->reg_vdd = NULL;
	regdata->reg_vio = NULL;
	regdata->id_vdd = "bmg160_vdd";
	regdata->id_vio = "bmg160_vio";

	/* init regulator */
	bmg160_regulator_init(&client->dev);

	/* power on */
	bmg160_regulator_control(1, &client->dev);

	/* check chip id */
	err = bmg_check_chip_id(client);
	if (!err) {
		dev_notice(&client->dev,
			"Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		dev_err(&client->dev,
			"Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}

	i2c_set_clientdata(client, client_data);
	client_data->client = client;

	mutex_init(&client_data->mutex_op_mode);
	mutex_init(&client_data->mutex_enable);

	/* input device init */
	err = bmg_input_init(client_data);
	if (err < 0)
		goto exit_err_sysfs;

	/* sysfs node creation */
	dev_gyro = bst_allocate_device();
	if (!dev_gyro) {
		err = -ENOMEM;
		goto exit_err_sysfs;
	}
	dev_gyro->name = "bmg";

	bst_set_drvdata(dev_gyro, client_data);

	err = bst_register_device(dev_gyro);
	if (err < 0)
		goto bst_free_gyro_exit;

	client_data->bst_gyro = dev_gyro;

	err = sysfs_create_group(&client_data->bst_gyro->dev.kobj,
			&bmg_attribute_group);
	if (err < 0)
		goto bst_free_exit;

	/* h/w init */
	client_data->device.bus_read = bmg_i2c_read_wrapper;
	client_data->device.bus_write = bmg_i2c_write_wrapper;
	client_data->device.delay_msec = bmg_i2c_delay;
	BMG_CALL_API(init)(&client_data->device);

	/* INT1/2 Pin setting. (Reset value = open-darin) */
	bmg160_set_int_od(BMG160_INT1, 0);
	bmg160_set_int_od(BMG160_INT2, 0);

	if (client_data->irq_enable == BMG160_ENABLE_INT1) {
		/* maps interrupt to INT1 pin */
		BMG_CALL_API(set_int_od)(BMG160_INT1, INT_PIN_PUSH_PULL);
		BMG_CALL_API(set_int_data)(BMG160_INT1, INT_ENABLE);
	} else if (client_data->irq_enable == BMG160_ENABLE_INT2) {
		/* maps interrupt to INT2 pin */
		BMG_CALL_API(set_int_data)(BMG160_INT2, INT_ENABLE);
		BMG_CALL_API(set_int_od)(BMG160_INT2, INT_PIN_PUSH_PULL);
	}

	if ((client_data->irq_enable == BMG160_ENABLE_INT1) ||
		(client_data->irq_enable == BMG160_ENABLE_INT2)) {
		client_data->IRQ = client->irq;

		err = request_threaded_irq(client_data->IRQ,
				     NULL, bmg_interrupt_thread,
				     IRQF_TRIGGER_RISING,
				     "bmg_int", client_data);
		if (err) {
			dev_err(&client->dev,
				"could not request irq %d err = %d\n",
				client_data->IRQ, err);
			goto bst_free_exit;
		}

		INIT_WORK(&client_data->irq_work, bmg_irq_work_func);
		disable_irq(client_data->IRQ);
	} else {
		/* soft workqueue init */
		INIT_DELAYED_WORK(&client_data->work, bmg_work_func);
		atomic_set(&client_data->delay, BMG_DELAY_DEFAULT);
	}

	bmg_dump_reg(client);

	client_data->enable = 0;
	client_data->sample_count = 0;
	/* now it's power on which is considered as resuming from suspend */

	BMG_CALL_API(set_bw)(C_BMG160_BW_47Hz_U8X); /*set bandwidth to 47Hz*/
	err = BMG_CALL_API(set_mode)(
			BMG_VAL_NAME(MODE_SUSPEND));

	if (err < 0)
		goto bst_free_exit;

	dev_notice(&client->dev, "sensor %s probed successfully", SENSOR_NAME);

	dev_dbg(&client->dev,
		"i2c_client: %p client_data: %p i2c_device: %p input: %p",
		client, client_data, &client->dev, client_data->input);

	return 0;

bst_free_exit:
	(void)bst_unregister_device(dev_gyro);

bst_free_gyro_exit:
	bst_free_device(dev_gyro);

exit_err_sysfs:
	if (err)
		bmg_input_destroy(client_data);

exit_err_clean:
	if (err) {
		if (client_data != NULL) {
			if ((client_data->irq_enable ==
				BMG160_ENABLE_INT1) ||
				(client_data->irq_enable ==
				BMG160_ENABLE_INT2)) {
				gpio_free(client_data->irq_gpio);
			}
			kfree(client_data);
			client_data = NULL;
		}
		bmg_client = NULL;
	}

exit_err:
	return err;
}

static int bmg_pre_suspend(struct i2c_client *client)
{
	int err = 0;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);
	dev_info(&client->dev, "function entrance:%s", __func__);

	if (client_data->enable) {
		if (!client_data->irq_enable) {
			cancel_delayed_work_sync(&client_data->work);
			dev_info(&client->dev, "cancel work");
		}
	}
	return err;
}

static int bmg_post_resume(struct i2c_client *client)
{
	int err = 0;
	u8 op_mode;
	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	dev_info(&client->dev, "function entrance:%s", __func__);

	if (client_data->enable) {
		err = BMG_CALL_API(set_mode)(BMG_VAL_NAME(MODE_NORMAL));
		if (err)
			goto err_exit;
		dev_info(&client->dev, "%s:NORMAL Mode", __func__);
		if (!client_data->irq_enable) {
			schedule_delayed_work(&client_data->work,
					msecs_to_jiffies(
					atomic_read(&client_data->delay)));
		}

	} else {
		BMG_CALL_API(get_mode)(&op_mode);
		if (op_mode == BMG_VAL_NAME(MODE_NORMAL))
			err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		dev_info(&client->dev, "%s:SUSPEND Mode", __func__);
	}

err_exit:
	return err;
}


static int bmg_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	u8 op_mode;

	dev_info(&client->dev, "function entrance:%s", __func__);

	BMG_CALL_API(get_mode)(&op_mode);
	if (op_mode == BMG_VAL_NAME(MODE_NORMAL)) {
		err = bmg_pre_suspend(client);
		if (err)
			goto err_exit;
		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		if (err)
			goto err_exit;
	}

err_exit:
	return err;
}

static int bmg_resume(struct i2c_client *client)
{
	int err = 0;

	dev_info(&client->dev, "function entrance:%s", __func__);

	/* post resume operation */
	err = bmg_post_resume(client);

	return err;
}


static int bmg_remove(struct i2c_client *client)
{
	int err = 0;
	u8 op_mode;

	struct bmg_client_data *client_data =
		(struct bmg_client_data *)i2c_get_clientdata(client);

	if (NULL != client_data) {
		mutex_lock(&client_data->mutex_op_mode);
		BMG_CALL_API(get_mode)(&op_mode);
		if (BMG_VAL_NAME(MODE_NORMAL) == op_mode) {
			if (!client_data->irq_enable) {
				cancel_delayed_work_sync(&client_data->work);
				dev_info(&client->dev, "cancel work");
			}
		}
		mutex_unlock(&client_data->mutex_op_mode);

		err = BMG_CALL_API(set_mode)(
				BMG_VAL_NAME(MODE_SUSPEND));
		msleep(BMG_I2C_WRITE_DELAY_TIME);

		sysfs_remove_group(&client_data->bst_gyro->dev.kobj,
				&bmg_attribute_group);
		bmg_input_destroy(client_data);
		if ((client_data->irq_enable == BMG160_ENABLE_INT1) ||
			(client_data->irq_enable == BMG160_ENABLE_INT2)) {
			gpio_free(client_data->irq_gpio);
		}
		kfree(client_data);

		bmg_client = NULL;
	}

	return err;
}

static const struct i2c_device_id bmg_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bmg_id);

#ifdef CONFIG_OF
static struct of_device_id bmg160_match_table[] = {
	{ .compatible = "bosch,bmg160", },
	{ },
};
#endif

static struct i2c_driver bmg_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = bmg160_match_table,
#endif
	},
	.class = I2C_CLASS_HWMON,
	.id_table = bmg_id,
	.probe = bmg_probe,
	.remove = bmg_remove,
	.suspend = bmg_suspend,
	.resume = bmg_resume,
};

static int __init BMG_init(void)
{
	return i2c_add_driver(&bmg_driver);
}

static void __exit BMG_exit(void)
{
	i2c_del_driver(&bmg_driver);
}

MODULE_AUTHOR("Ji.Chen <ji.chen@bosch-sensortec.com>");
MODULE_DESCRIPTION("driver for " SENSOR_NAME);
MODULE_LICENSE("GPL v2");

module_init(BMG_init);
module_exit(BMG_exit);
