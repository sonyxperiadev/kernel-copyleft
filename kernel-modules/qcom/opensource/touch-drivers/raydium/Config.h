/* config.h
 *
 * Raydium TouchScreen driver.
 *
 * Copyright (c) 2021  Raydium tech Ltd.
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

#ifndef __CONFIG_H
#define __CONFIG_H


/*****************************************************************************
**                            GLOBAL MARCO DEFINITION
******************************************************************************/
#define MINIMUM(x, y) (((x) < (y)) ? (x) : (y)) //  Compares two parameters, return minimum.
#define MAXIMUM(x, y) (((x) > (y)) ? (x) : (y)) //  Compares two parameters, return maximum.
#define ABS(x) ((x) >= 0 ? (x) : -(x))          // return absolute value

/*****************************************************************************
**                            GLOBAL FUNCTIONAL DEFINITION
******************************************************************************/

//Bootloader
#define BOOTLOADER                          1  //1: with bootloader, 0: without bootloader
#define SELFTEST                       	     1  //1: For System Selftest, 0:For Dongle Open/Short Tool
#if SELFTEST
#define SELFTEST_3X	1
#define	SELFTEST_2X	0
#else
#define SELFTEST_3X	0
#define	SELFTEST_2X	0
#endif
#define ENABLE_TEST_TIME_MEASURMENT		1
#define ENABLE_TEST_TIME_MEASURMENT_CC		0
#define ENABLE_WDT 				0
#define ENABLE_TEST_RSU_DATA_SHOW		0
#define ENABLE_AUO_VERIFY_LOG			0
#define ENABLE_CONTROL_OPENSHORT_WDT		0
#define ENABLE_TEST_GPIO_MEASURMENT		0
#endif /* end __CONFIG_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
