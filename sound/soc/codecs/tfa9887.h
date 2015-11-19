#ifndef __tfa9887_H__
#define __tfa9887_H__

#define tfa9887_U16 unsigned short       /* 16 bit achieved with short */
#define tfa9887_S16 signed short
#define tfa9887_S32 signed int           /* 32 bit achieved with int   */


/*
 ********************************************************************
 *
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 ********************************************************************/
/*  Date: 2010/05/31
 *  Revision: 1.0
 *  Date: 2011/03/15
 *  Revision: 1.1
 *  Date: 2011/06/27
 *  Revision: 1.2
 *
 *
 */

/*******************************************************************
* Copyright (C) 2007 Bosch Sensortec GmbH
*
* tfa9887.h
*
* Usage:    This header file which includes all function declaration
*
* Author:       gokul.rajendran@in.bosch.com, yogananth.k@in.bosch.com
********************************************************************/
/*******************************************************************/
/*  Disclaimer
*
* Common:
* Bosch Sensortec products are developed for the consumer goods industry.
* They may only be used within the parameters of the respective valid
* product data sheet.  Bosch Sensortec products areprovided with the
* express understanding that there is no warranty of fitness for a
* particular purpose.They are not fit for use in life-sustaining,
* safety or security sensitive systems or any system or device
* that may lead to bodily harm or property damage if the system
* or device malfunctions. In addition,Bosch Sensortec products are
* not fit for use in products which interact with motor vehicle systems.
* The resale and or use of products are at the purchasers own risk and
* his own responsibility. The examination of fitness for the intended use
* is the sole responsibility of the Purchaser.
*
* The purchaser shall indemnify Bosch Sensortec from all third party
* claims, including any claims for incidental, or consequential damages,
* arising from any product use not covered by the parameters of
* the respective valid product data sheet or not approved by
* Bosch Sensortec and reimburse Bosch Sensortec for all costs in
* connection with such claims.
*
* The purchaser must monitor the market for the purchased products,
* particularly with regard to product safety and inform Bosch Sensortec
* without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e).
* Samples may vary from the valid technical specifications of the product
* series. They are therefore not intended or fit for resale to third
* parties or for use in end products. Their sole purpose is internal
* client testing. The testing of an engineering sample may in no way
* replace the testing of a product series. Bosch Sensortec assumes
* no liability for the use of engineering samples.
* By accepting the engineering samples, the Purchaser agrees to indemnify
* Bosch Sensortec from all claims arising from the use of engineering
* samples.
*
* Special:
* This software module (hereinafter called "Software") and any information
* on application-sheets (hereinafter called "Information") is provided
* free of charge for the sole purpose to support your application work.
* The Software and Information is subject to the following
* terms and conditions:
*
* The Software is specifically designed for the exclusive use for
* Bosch Sensortec products by personnel who have special experience
* and training. Do not use this Software if you do not have the
* proper experience or training.
*
* This Software package is provided `` as is `` and without any expressed
* or implied warranties,including without limitation, the implied warranties
* of merchantability and fitness for a particular purpose.
*
* Bosch Sensortec and their representatives and agents deny any liability
* for the functional impairment
* of this Software in terms of fitness, performance and safety.
* Bosch Sensortec and their representatives and agents shall not be liable
* for any direct or indirect damages or injury, except as
* otherwise stipulated in mandatory applicable law.
*
* The Information provided is believed to be accurate and reliable.
* Bosch Sensortec assumes no responsibility for the consequences of use
* of such Information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of Bosch. Specifications mentioned in the Information are
* subject to change without notice.
*
* It is not allowed to deliver the source code of the Software
* to any third party without permission of
* Bosch Sensortec.
*/
/*****************************************************************************/
/*! \file tfa9887.h
    \brief Header for tfa9887 API */
/**\brief defines the calling parameter types of the tfa9887_WR_FUNCTION */
#define tfa9887_BUS_WR_RETURN_TYPE char

/**\brief links the order of parameters defined in
tfa9887_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define tfa9887_BUS_WR_PARAM_TYPES unsigned char, unsigned char,\
unsigned char *, unsigned char

/**\brief links the order of parameters defined in
tfa9887_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define tfa9887_BUS_WR_PARAM_ORDER(device_addr, register_addr,\
register_data, wr_len)

/* never change this line */
#define tfa9887_BUS_WRITE_FUNC(device_addr, register_addr,\
register_data, wr_len) bus_write(device_addr, register_addr,\
register_data, wr_len)
/**\brief defines the return parameter type of the tfa9887_RD_FUNCTION
*/
#define tfa9887_BUS_RD_RETURN_TYPE char
/**\brief defines the calling parameter types of the tfa9887_RD_FUNCTION
*/
#define tfa9887_BUS_RD_PARAM_TYPES unsigned char, unsigned char,\
unsigned char *, unsigned char
/**\brief links the order of parameters defined in \
tfa9887_BUS_RD_PARAM_TYPE to function calls used inside the API
*/
#define tfa9887_BUS_RD_PARAM_ORDER device_addr, register_addr,\
register_data
/* never change this line */
#define tfa9887_BUS_READ_FUNC(device_addr, register_addr,\
register_data, rd_len)bus_read(device_addr, register_addr,\
register_data, rd_len)
/**\brief defines the return parameter type of the tfa9887_RD_FUNCTION
*/
#define tfa9887_BURST_RD_RETURN_TYPE char
/**\brief defines the calling parameter types of the tfa9887_RD_FUNCTION
*/
#define tfa9887_BURST_RD_PARAM_TYPES unsigned char,\
unsigned char, unsigned char *, signed int
/**\brief links the order of parameters defined in \
tfa9887_BURST_RD_PARAM_TYPE to function calls used inside the API
*/
#define tfa9887_BURST_RD_PARAM_ORDER device_addr, register_addr,\
register_data
/* never change this line */
#define tfa9887_BURST_READ_FUNC(device_addr, register_addr,\
register_data, rd_len)burst_read(device_addr, \
register_addr, register_data, rd_len)
/**\brief defines the return parameter type of the tfa9887_DELAY_FUNCTION
*/
#define tfa9887_DELAY_RETURN_TYPE void
/**\brief defines the calling parameter types of the tfa9887_DELAY_FUNCTION
*/
#define tfa9887_DELAY_PARAM_TYPES tfa9887_U16
/* never change this line */
#define tfa9887_DELAY_FUNC(delay_in_msec)\
		delay_func(delay_in_msec)
#define tfa9887_RETURN_FUNCTION_TYPE			int
/**< This refers tfa9887 return type as char */

#define	tfa9887_I2C_ADDR1				0x68
#define	tfa9887_I2C_ADDR					tfa9887_I2C_ADDR1
#define	tfa9887_I2C_ADDR2				0x69


/* For Axis Selection   */
/**< It refers tfa9887 X-axis */
#define tfa9887_X_AXIS           0
/**< It refers tfa9887 Y-axis */
#define tfa9887_Y_AXIS           1
/**< It refers tfa9887 Z-axis */
#define tfa9887_Z_AXIS           2

/* For Mode Settings    */
#define tfa9887_MODE_NORMAL              0
#define tfa9887_MODE_DEEPSUSPEND         1
#define tfa9887_MODE_SUSPEND             2
#define tfa9887_MODE_FASTPOWERUP			3
#define tfa9887_MODE_ADVANCEDPOWERSAVING 4

/* get bit slice  */
#define tfa9887_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)

/* Set bit slice */
#define tfa9887_SET_BITSLICE(regvar, bitname, val)\
((regvar&~bitname##__MSK)|((val<<bitname##__POS)&bitname##__MSK))
/* Constants */

#define tfa9887_NULL                             0
/**< constant declaration of NULL */
#define tfa9887_DISABLE                          0
/**< It refers tfa9887 disable */
#define tfa9887_ENABLE                           1
/**< It refers tfa9887 enable */
#define tfa9887_OFF                              0
/**< It refers tfa9887 OFF state */
#define tfa9887_ON                               1
/**< It refers tfa9887 ON state  */


#define tfa9887_TURN1                            0
/**< It refers tfa9887 TURN1 */
#define tfa9887_TURN2                            1
/**< It refers tfa9887 TURN2 */

#define tfa9887_INT1                             0
/**< It refers tfa9887 INT1 */
#define tfa9887_INT2                             1
/**< It refers tfa9887 INT2 */

#define tfa9887_SLOW_OFFSET                      0
/**< It refers tfa9887 Slow Offset */
#define tfa9887_AUTO_OFFSET                      1
/**< It refers tfa9887 Auto Offset */
#define tfa9887_FAST_OFFSET                      2
/**< It refers tfa9887 Fast Offset */
#define tfa9887_S_TAP                            0
/**< It refers tfa9887 Single Tap */
#define tfa9887_D_TAP                            1
/**< It refers tfa9887 Double Tap */
#define tfa9887_INT1_DATA                        0
/**< It refers tfa9887 Int1 Data */
#define tfa9887_INT2_DATA                        1
/**< It refers tfa9887 Int2 Data */
#define tfa9887_TAP_UNFILT_DATA                   0
/**< It refers tfa9887 Tap unfilt data */
#define tfa9887_HIGH_UNFILT_DATA                  1
/**< It refers tfa9887 High unfilt data */
#define tfa9887_CONST_UNFILT_DATA                 2
/**< It refers tfa9887 Const unfilt data */
#define tfa9887_ANY_UNFILT_DATA                   3
/**< It refers tfa9887 Any unfilt data */
#define tfa9887_SHAKE_UNFILT_DATA                 4
/**< It refers tfa9887 Shake unfilt data */
#define tfa9887_SHAKE_TH                         0
/**< It refers tfa9887 Shake Threshold */
#define tfa9887_SHAKE_TH2                        1
/**< It refers tfa9887 Shake Threshold2 */
#define tfa9887_AUTO_OFFSET_WL                   0
/**< It refers tfa9887 Auto Offset word length */
#define tfa9887_FAST_OFFSET_WL                   1
/**< It refers tfa9887 Fast Offset word length */
#define tfa9887_I2C_WDT_EN                       0
/**< It refers tfa9887 I2C WDT En */
#define tfa9887_I2C_WDT_SEL                      1
/**< It refers tfa9887 I2C WDT Sel */
#define tfa9887_EXT_MODE                         0
/**< It refers tfa9887 Ext Mode */
#define tfa9887_EXT_PAGE                         1
/**< It refers tfa9887 Ext page */
#define tfa9887_START_ADDR                       0
/**< It refers tfa9887 Start Address */
#define tfa9887_STOP_ADDR                        1
/**< It refers tfa9887 Stop Address */
#define tfa9887_SLOW_CMD                         0
/**< It refers tfa9887 Slow Command */
#define tfa9887_FAST_CMD                         1
/**< It refers tfa9887 Fast Command */
#define tfa9887_TRIM_VRA                         0
/**< It refers tfa9887 Trim VRA */
#define tfa9887_TRIM_VRD                         1
/**< It refers tfa9887 Trim VRD */
#define tfa9887_LOGBIT_EM                        0
/**< It refers tfa9887 LogBit Em */
#define tfa9887_LOGBIT_VM                        1
/**< It refers tfa9887 LogBit VM */
#define tfa9887_GP0                              0
/**< It refers tfa9887 GP0 */
#define tfa9887_GP1                              1
/**< It refers tfa9887 GP1*/
#define tfa9887_LOW_SPEED                        0
/**< It refers tfa9887 Low Speed Oscilator */
#define tfa9887_HIGH_SPEED                       1
/**< It refers tfa9887 High Speed Oscilator */
#define tfa9887_DRIVE_OFFSET_P                   0
/**< It refers tfa9887 Drive Offset P */
#define tfa9887_DRIVE_OFFSET_N                   1
/**< It refers tfa9887 Drive Offset N */
#define tfa9887_TEST_MODE_EN                     0
/**< It refers tfa9887 Test Mode Enable */
#define tfa9887_TEST_MODE_REG                    1
/**< It refers tfa9887 Test Mode reg */
#define tfa9887_IBIAS_DRIVE_TRIM                 0
/**< It refers tfa9887 IBIAS Drive Trim */
#define tfa9887_IBIAS_RATE_TRIM                  1
/**< It refers tfa9887 IBIAS Rate Trim */
#define tfa9887_BAA_MODE                         0
/**< It refers tfa9887 BAA Mode Trim */
#define tfa9887_BMA_MODE                         1
/**< It refers tfa9887 BMA Mode Trim */
#define tfa9887_PI_KP                            0
/**< It refers tfa9887 PI KP */
#define tfa9887_PI_KI                            1
/**< It refers tfa9887 PI KI */


#define C_tfa9887_SUCCESS						0
/**< It refers tfa9887 operation is success */
#define C_tfa9887_FAILURE						1
/**< It refers tfa9887 operation is Failure */

#define tfa9887_SPI_RD_MASK                      0x80
/**< Read mask **/
#define tfa9887_READ_SET                         0x01
/**< Setting for rading data **/

#define tfa9887_SHIFT_1_POSITION                 1
/**< Shift bit by 1 Position **/
#define tfa9887_SHIFT_2_POSITION                 2
/**< Shift bit by 2 Position **/
#define tfa9887_SHIFT_3_POSITION                 3
/**< Shift bit by 3 Position **/
#define tfa9887_SHIFT_4_POSITION                 4
/**< Shift bit by 4 Position **/
#define tfa9887_SHIFT_5_POSITION                 5
/**< Shift bit by 5 Position **/
#define tfa9887_SHIFT_6_POSITION                 6
/**< Shift bit by 6 Position **/
#define tfa9887_SHIFT_7_POSITION                 7
/**< Shift bit by 7 Position **/
#define tfa9887_SHIFT_8_POSITION                 8
/**< Shift bit by 8 Position **/
#define tfa9887_SHIFT_12_POSITION                12
/**< Shift bit by 12 Position **/

#define         C_tfa9887_Null_U8X                              0
#define         C_tfa9887_Zero_U8X                              0
#define         C_tfa9887_One_U8X                               1
#define         C_tfa9887_Two_U8X                               2
#define         C_tfa9887_Three_U8X                             3
#define         C_tfa9887_Four_U8X                              4
#define         C_tfa9887_Five_U8X                              5
#define         C_tfa9887_Six_U8X                               6
#define         C_tfa9887_Seven_U8X                             7
#define         C_tfa9887_Eight_U8X                             8
#define         C_tfa9887_Nine_U8X                              9
#define         C_tfa9887_Ten_U8X                               10
#define         C_tfa9887_Eleven_U8X                            11
#define         C_tfa9887_Twelve_U8X                            12
#define         C_tfa9887_Thirteen_U8X                          13
#define         C_tfa9887_Fifteen_U8X                           15
#define         C_tfa9887_Sixteen_U8X                           16
#define         C_tfa9887_TwentyTwo_U8X                         22
#define         C_tfa9887_TwentyThree_U8X                       23
#define         C_tfa9887_TwentyFour_U8X                        24
#define         C_tfa9887_TwentyFive_U8X                        25
#define         C_tfa9887_ThirtyTwo_U8X                         32
#define         C_tfa9887_Hundred_U8X                           100
#define         C_tfa9887_OneTwentySeven_U8X                    127
#define         C_tfa9887_OneTwentyEight_U8X                    128
#define         C_tfa9887_TwoFiftyFive_U8X                      255
#define         C_tfa9887_TwoFiftySix_U16X                      256

#define E_tfa9887_NULL_PTR               (char)(-127)
#define E_tfa9887_COMM_RES               (char)(-1)
#define E_tfa9887_OUT_OF_RANGE           (signed char)(-2)

#define C_tfa9887_No_Filter_U8X			0
#define	C_tfa9887_BW_230Hz_U8X			1
#define	C_tfa9887_BW_116Hz_U8X			2
#define	C_tfa9887_BW_47Hz_U8X			3
#define	C_tfa9887_BW_23Hz_U8X			4
#define	C_tfa9887_BW_12Hz_U8X			5
#define	C_tfa9887_BW_64Hz_U8X			6
#define	C_tfa9887_BW_32Hz_U8X			7

#define C_tfa9887_No_AutoSleepDur_U8X	0
#define	C_tfa9887_4ms_AutoSleepDur_U8X	1
#define	C_tfa9887_5ms_AutoSleepDur_U8X	2
#define	C_tfa9887_8ms_AutoSleepDur_U8X	3
#define	C_tfa9887_10ms_AutoSleepDur_U8X	4
#define	C_tfa9887_15ms_AutoSleepDur_U8X	5
#define	C_tfa9887_20ms_AutoSleepDur_U8X	6
#define	C_tfa9887_40ms_AutoSleepDur_U8X	7

#define tfa9887_WR_FUNC_PTR char (*bus_write)\
(unsigned char, unsigned char, unsigned char *, unsigned char)
#define tfa9887_RD_FUNC_PTR char (*bus_read)\
(unsigned char, unsigned char, unsigned char *, unsigned char)
#define tfa9887_BRD_FUNC_PTR char (*burst_read)\
(unsigned char, unsigned char, unsigned char *, tfa9887_S32)
#define tfa9887_MDELAY_DATA_TYPE tfa9887_S32

/*user defined Structures*/
struct tfa9887_t {
		unsigned char chip_id;
		unsigned char dev_addr;
		tfa9887_BRD_FUNC_PTR;
		tfa9887_WR_FUNC_PTR;
		tfa9887_RD_FUNC_PTR;
		void(*delay_msec)(tfa9887_MDELAY_DATA_TYPE);
};

int bmg_set_power(int Mode);	/* PERI-FG-REGULATOR_SET-00+ */
#endif
