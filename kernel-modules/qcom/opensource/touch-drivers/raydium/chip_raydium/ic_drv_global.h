/* i2c_drv_global.h
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

#ifndef _DRVGLOBAL_H_
#define _DRVGLOBAL_H_
#include "../Config.h"
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#if (SELFTEST_2X)
#include "f302_ic_control.h"
#endif

#if !SELFTEST
#include "f302_ic_control.h"
#include "f303_ic_control.h"
#include "usb.h"
#endif
//********* Basic Parameter Definition ***********//
/*****************************************************************************
**
**                  Declared Global Variable
**
*****************************************************************************/


#define MAX_IMAGE_BUFFER_SIZE			64
#define MAX_SENSING_PIN_NUM				50	// (wearable)
#define F302_MAX_IMAGE_BUFFER_SIZE		36
#define DATA_BUFFER_SIZE				(0x100)

#define F302_DONGLE_FLASH_INI_ADDR		0xF000
#define F302_INI_THRESHOLD_ADDR			(F302_DONGLE_FLASH_INI_ADDR + 16)
#define F302_INI_PARA_ADDR			(F302_INI_THRESHOLD_ADDR + 36)
#define F302_INI_RAW_DATA2_BL_ADDR		(F302_INI_PARA_ADDR + 48)
#define F302_INI_RAW_DATA_3_CC_ADDR		(F302_INI_RAW_DATA2_BL_ADDR + 72)
#define F302_INI_UC_CC_ADDR			(F302_INI_RAW_DATA_3_CC_ADDR + 72) // INI_RAW_DATA2_ADDR
#define F302_NA_P						36

#define F303_DONGLE_FLASH_INI_ADDR		0x10000
#define F303_INI_THRESHOLD_ADDR			(F303_DONGLE_FLASH_INI_ADDR + 16)
#define F303_INI_PARA_ADDR			(F303_INI_THRESHOLD_ADDR + 36)
#define F303_INI_RAW_DATA2_BL_ADDR		(F303_INI_PARA_ADDR + 48)
#define F303_INI_RAW_DATA_3_CC_ADDR		(F303_INI_RAW_DATA2_BL_ADDR + 128)
#define F303_INI_UC_CC_ADDR			(F303_INI_RAW_DATA_3_CC_ADDR + 128) // INI_RAW_DATA2_ADDR
#define F303_MAX_SENSING_PIN_NUM		48
#define F303_NA_P						65

#define F302_INITIAL_CODE_START_ADDR	0xF400
#define F303_INITIAL_CODE_START_ADDR	0x10400

#define PRINT_DEBUG_MSG_TYPE_1 		0x01
#define PRINT_DEBUG_MSG_TYPE_2 		0x02
#define PRINT_DEBUG_MSG_TYPE_3 		0x04
#define PRINT_DEBUG_MSG_TYPE_4 		0x08
#define PRINT_DEBUG_MSG_TYPE_5 		0x10
#define PRINT_DEBUG_MSG_TYPE_6 		0x20
#define PRINT_DEBUG_MSG_TYPE_7 		0x40
#define PRINT_DEBUG_MSG_TYPE_8 		0x80

#define FW_SYS_CMD_ADDR         	0x20000288

#define SYS_CMD_TP_MANUAL_MODE					0x34
#define SYS_CMD_DO_BL_CAL                   	0x5A
#define SYS_CMD_DO_CC_CAL                   	0x5C

#define FW_TP_SEQ_NUM_ADDR          0x20000290
#define FW_TP_POINT_DATA_ADDR       0x20000294
#define REG_SYSCON_MISCIER_ADDR		0x40000014
#define FW_FT_IMG_ADDR				0x2000019C
#define RM_DATAMEM0_BASE			0x20000000

#define F303_DRAM_FT_DBG_START		0x2000004C

//======================= Basic Hardware Define ==============================


//==================== End of Basic Hardware Define ==========================

#if SELFTEST

#ifdef __KERNEL__
typedef enum {
	error = -1,
	success = 1
} STATUS;

#else
typedef enum {
	ERROR = 0,
	SUCCESS = 1
} STATUS;
typedef enum {
	DISABLE = 0,
	ENABLE = 1
} FunctionalState;

typedef enum {
	false = 0,
	true = 1
} bool;
#endif
#endif

//========== Basic Parameter Information ==========
//********************* Global *********************//

// Global variable for dongle
extern unsigned char g_u8_drv_interface;

extern volatile unsigned char g_u8_gpio_irq_trigger;
extern unsigned short g_u16_dev_id;


extern unsigned char g_u8_raw_data_buffer[MAX_IMAGE_BUFFER_SIZE * 2];
extern unsigned short g_u16_raw_data_tmp[MAX_IMAGE_BUFFER_SIZE];

extern short g_i16_raw_data_1_short_buf[MAX_SENSING_PIN_NUM];
extern short g_i16_raw_data_2_open_buf[MAX_SENSING_PIN_NUM];
extern unsigned short g_u16_raw_data3_cc_buf[MAX_SENSING_PIN_NUM];
extern unsigned short g_u16_uc_buf[MAX_SENSING_PIN_NUM];


extern unsigned short g_u16_raw_data3_golden_cc_buf[MAX_SENSING_PIN_NUM];
extern unsigned short g_u16_uc_golden_cc_buf[MAX_SENSING_PIN_NUM];
extern unsigned int g_u32_test_result[MAX_SENSING_PIN_NUM]; // each node test result (open ng, short ng, uniformity ng..etc)
extern unsigned char g_u8_wearable_pin_map[MAX_IMAGE_BUFFER_SIZE];
extern 	unsigned char g_u8_test_result[MAX_SENSING_PIN_NUM];

extern volatile unsigned short g_u16_test_items_host_cmd;
extern unsigned short g_u16_test_items_tool_cmd;
extern volatile unsigned short g_u16_panel_jig_set_test_items;

extern unsigned int g_u32_wearable_test_result;
extern unsigned char g_u8_channel_x;
extern unsigned char g_u8_channel_y;
extern unsigned char g_u8_is_normal_fw;
extern unsigned char g_u8_data_buf[DATA_BUFFER_SIZE];
extern unsigned char g_u8_mipi_read_buf[56];

extern unsigned int g_u32_fw_cc_version;
extern unsigned char g_u8_print_debug_msg;

extern char g_i8_test_baseline_msg[30];
extern bool b_is_auo_jig_testing;
#if ENABLE_TEST_TIME_MEASURMENT || ENABLE_TEST_TIME_MEASURMENT_CC
extern unsigned int g_u32_spend_time;
#endif


#if !SELFTEST
extern unsigned char g_u8_ic_power_on_ng;
#endif

extern unsigned char g_u8_display_interface;
extern unsigned char g_u8_PAGE_ADDR;

extern int g_u32_dongle_flash_ini_addr;//       0xF000
extern int  g_u32_ini_threshold_addr;//          DONGLE_FLASH_INI_ADDR + 16
extern int  g_u32_ini_para_addr;//               INI_THRESHOLD_ADDR + 36
//int  u32_ini_raw_data_2_bl_addr;//		INI_PARA_ADDR+48
extern int  g_u32_ini_raw_data_3_cc_addr;//      INI_RAW_DATA2_BL_ADDR+72
extern int  g_u32_ini_uc_cc_addr;//              INI_RAW_DATA_3_CC_ADDR + 72// INI_RAW_DATA2_ADDR

extern int g_u32_initial_code_start_addr;
extern void ic_drv_init(void);

#endif  /* end _DRVGLOBAL_H_*/

/******************************************************************************
**                            End Of File
******************************************************************************/
