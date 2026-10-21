/* raydium_selftest.h
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

#define DISABLE_PDA2_MODE_AND_INT		0
#define ENABLE_PDA2_MODE_AND_INT		1
#define DISABLE_TOUCH_INT				2
#define ENABLE_TOUCH_INT				3

#define ENABLE_TIME_MEASURMENT			1
/*Raydium system flag*/
#define RAYDIUM_INTERRUPT_FLAG			0x01


#define RM_SELF_TEST_THRESHOLD_FILE		"/data/selftest/tpselftest.ini"
#define RM_SELF_TEST_LOGFILE			"/sdcard/selftest.csv"


extern const unsigned char u8_rad_testpara_20[];

