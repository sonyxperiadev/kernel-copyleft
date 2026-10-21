/* i2c_drv_interface.h
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

#include "ic_drv_global.h"

#define DEVICE_ID_2X             				 0xF302
#define DEVICE_ID_3X             				 0xF303

#define I2C_PDA2_BYTE_MODE  								0x03
#define I2C_PDA2_WORD_MODE  								0x43
#define I2C_BYTE_MODE 										(0x80|0x20)
#define I2C_WORD_MODE 										(0xc0|0x20)
#define I2C_MCU_MODE  										0x40
#define I2C_PDA2_MODE  										0x10
#define I2C_PDA_MODE  										0x08

#define SPI_BYTE_MODE 										(0x80|0x03)
#define SPI_WORD_MODE 										(0xc0|0x03)
#define SPI_MCU_MODE  										0x40

#define I2C_INTERFACE										0
#define SPI_INTERFACE										1

#define DISPLAY_SPI_MODE									0
#define DISPLAY_MIPI_MODE									1
#define DISPLAY_TOUCH_2_DRIVER_MODE						2

#define DATA_REMAP_TO_IC_PIN    							1
#define DATA_REMAP_TO_SENSOR_PAD 							2

#define IC_TEST_RETRY_TIME               					3
#define IC_TEST_TIME_OUT                   					(-1)
#define FT_TEST_STUATUS_PATTERN            					0x5A00

#define FT_CMD_INIT                         				0x11
#define FT_CMD_DO_FT_TEST                   				0x14

#define IC_TEST_ITEMS_SYSTEM                                0x0001
#define IC_TEST_ITEMS_BURN_FW                               0x0002
#define IC_TEST_ITEMS_OPEN                                  0x0004
#define IC_TEST_ITEMS_SHORT                                 0x0008
#define IC_TEST_ITEMS_UC                                    0x0010
#define IC_TEST_ITEMS_UB                                    0x0020
#define IC_TEST_ITEMS_BURN_CC                               0x0040
#define IC_TEST_ITEMS_PANEL_TEST_1                          0x0080
#define IC_TEST_ITEMS_PANEL_TEST_2                          0x0100
#define IC_TEST_ITEMS_PANEL_TEST_3                          0x0200
#define IC_TEST_ITEMS_RESV_0                                0x0400
#define IC_TEST_ITEMS_RESV_1                                0x0800
#define IC_TEST_ITEMS_RESV_2                                0x1000
#define IC_TEST_ITEMS_RESV_3                                0x2000
#define IC_TEST_ITEMS_RESV_4                                0x4000
#define IC_TEST_ITEMS_RESV_5                                0x8000

#define IC_TEST_ENG_ITEMS_RESV_1                            0x0001
#define IC_TEST_ENG_ITEMS_RESV_2                            0x0002
#define IC_TEST_ENG_ITEMS_RESV_3                            0x0004
#define IC_TEST_ENG_ITEMS_TEST_ALL                          0x0008
#define IC_TEST_ENG_ITEMS_RESV_5                            0x0010
#define IC_TEST_ENG_ITEMS_RESV_6                            0x0020
#define IC_TEST_ENG_ITEMS_RESV_7                            0x0040
#define IC_TEST_ENG_ITEMS_RESV_8                            0x0080
#define IC_TEST_ENG_ITEMS_PANEL_TEST_JIG                    0x0100
#define IC_TEST_ENG_ITEMS_RESV_10                           0x0200
#define IC_TEST_ENG_ITEMS_RESV_11                           0x0400
#define IC_TEST_ENG_ITEMS_RESV_12                           0x0800
#define IC_TEST_ENG_ITEMS_RESV_13                           0x1000
#define IC_TEST_ENG_ITEMS_RESV_14                           0x2000
#define IC_TEST_ENG_ITEMS_RESV_15                           0x4000
#define IC_TEST_ENG_ITEMS_RESV_16                           0x8000

#define WEARABLE_FT_TEST_RESULT_IC_INIT_STATE 	                0xDFFFE5DF
#define WEARABLE_FT_TEST_RESULT_PANEL_INIT_STATE                0x20001A20
#define WEARABLE_FT_TEST_RESULT_PASS        					0x00000000
#define WEARABLE_FT_TEST_RESULT_OPEN_NG     					0x00000001
#define WEARABLE_FT_TEST_RESULT_SHORT_NG    					0x00000002
#define WEARABLE_FT_TEST_RESULT_UB_NG	      					0x00000004
#define WEARABLE_FT_TEST_RESULT_I2C_NG      					0x00000008
#define WEARABLE_FT_TEST_RESULT_INT_NG      					0x00000010
#define WEARABLE_FT_TEST_RESULT_PANEL_TEST_1_NG					0x00000020
#define WEARABLE_FT_TEST_RESULT_RESET_NG    					0x00000040
#define WEARABLE_FT_TEST_RESULT_CB_NG       					0x00000080
#define WEARABLE_FT_TEST_RESULT_PRAM_NG     					0x00000100
#define WEARABLE_FT_TEST_RESULT_NORMAL_FW_NG     				0x00000200
#define WEARABLE_FT_TEST_RESULT_BURN_CC_NG     					0x00000400
#define WEARABLE_FT_TEST_RESULT_PANEL_TEST_3_NG     			0x00000800
#define WEARABLE_FT_TEST_RESULT_GET_DATA_NG					    0x00001000
#define WEARABLE_FT_TEST_RESULT_FLASH_ID_NG    					0x00002000
#define WEARABLE_FT_TEST_RESULT_NORMAL_FW_VER_NG    			0x00004000
#define WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG      			0x00008000
#define WEARABLE_FT_TEST_RESULT_UC_NG  	    					0x00010000
#define WEARABLE_FT_TEST_RESULT_SINGLE_CC_OPEN_NG				0x00020000
#define WEARABLE_FT_TEST_RESULT_SINGLE_CC_SHORT_NG				0x00040000
#define WEARABLE_FT_TEST_RESULT_CC_CALIBRATION_NG				0x00080000
#define WEARABLE_FT_TEST_RESULT_TEST_INIT_NG				    0x00100000
#define WEARABLE_FT_TEST_RESULT_LOAD_TESTFW_NG 				    0x00200000
#define WEARABLE_FT_TEST_RESULT_BURN_FW_NG     					0x00400000
#define WEARABLE_FT_TEST_RESULT_IC_FW_VERIFY_NG					0x00800000
#define WEARABLE_FT_TEST_RESULT_EXT_FLASH_EMPTY_NG				0x01000000
#define WEARABLE_FT_TEST_RESULT_MCU_HOLD_NG						0x02000000
#if SELFTEST
#define WEARABLE_FT_TEST_RESULT_SYSFS_NG						0x04000000
#else
#define WEARABLE_FT_TEST_RESULT_AUO_JIG_NG						0x04000000
#endif
#define WEARABLE_FT_TEST_RESULT_NORMAL_FW_RESET_NG              0x08000000
#define WEARABLE_FT_TEST_RESULT_POWER_ON_NG						0x10000000
#define WEARABLE_FT_TEST_RESULT_CMD_NG                          0x20000000
#define WEARABLE_FT_TEST_RESULT_IC_SUB_VERSION_NG               0x40000000
#define WEARABLE_FT_TEST_RESULT_IC_VERSION_NG                   0x80000000

#define WEARBLE_FT_BURN_CC_SYSTEM_NG_CASE						1
#define WEARBLE_FT_FW_VER_SYSTEM_NG_CASE						2
#define WEARBLE_FT_I2C_SYSTEM_NG_CASE							3
#define WEARBLE_FT_INT_SYSTEM_NG_CASE							4
#define WEARBLE_FT_LOAD_TEST_FW_SYSTEM_NG_CASE					5
#define WEARBLE_FT_PRAM_SYSTEM_NG_CASE							6
#define WEARBLE_FT_RESET_PIN_SYSTEM_NG_CASE						7


// Define ic test state
typedef enum {
	IC_TEST_INIT = 0,
	IC_TEST_SYSTEM,					//1
	IC_TEST_BURN_FW,				//2
	IC_TEST_BURN_CC,				//3
	IC_TEST_LOAD_TEST_FW,			//4
	IC_TEST_ENTER_FW_TEST_MODE,		//5
	IC_TEST_OPEN_SHORT,				//6
	IC_TEST_READ_TEST_FW_DATA,	    //7
	IC_ENTER_NORMAL_FW,				//8
	IC_TEST_UC_UB,					//9
	IC_TEST_PANEL_PATTERN_TEST,     //10
	IC_TEST_EXIT,					//11
} IC_TEST_STATE;

#pragma pack(push)
#pragma pack(1)
typedef struct {
	//-------- FT Test Info (Byte 16)-----------------//
	unsigned char u8_device_id;
	unsigned char u8_company_id;
	unsigned char u8_project_id;
	unsigned char u8_station_id;
	unsigned short u16_ft_test_item;
	unsigned short u16_ft_eng_item;
	unsigned short u16_ft_test_info_1;
	unsigned short u16_ft_test_info_2;
	unsigned short u16_ft_test_info_3;
	unsigned short u16_ft_test_info_4;
} st_test_info;
extern st_test_info g_st_test_info;

typedef struct {
	//-------- FT Test Threshold (Byte 26)----------------//
	unsigned char u8_ft_test_company_id;
	unsigned char u8_ft_test_station_id;
	signed short i16_ft_test_open_lower_thd;              //TEST_THD_01
	signed short i16_ft_test_short_upper_thd;             //TEST_THD_02
	signed short i16_ft_test_short_lower_thd;             //TEST_THD_03
	signed short i16_ft_test_single_cc_upper_thd;         //TEST_THD_04
	signed short i16_ft_test_single_cc_lower_thd;	      //TEST_THD_05
	signed short i16_ft_test_uniformity_bl_upper_thd;     //TEST_THD_06
	signed short i16_ft_test_uniformity_bl_lower_thd;     //TEST_THD_07
	signed short i16_ft_test_uniformity_cc_upper_thd;	  //TEST_THD_08
	signed short i16_ft_test_uniformity_cc_lower_thd;     //TEST_THD_09
	signed short i16_ft_test_panel_test_1_thd;            //TEST_THD_10
	signed short i16_ft_test_panel_test_3_thd;            //TEST_THD_11
	signed short i16_ft_test_panel_test_2_thd;            //TEST_THD_12
	//-------- reserve (Byte 10)-----------------//
	signed short i16_ft_test_panel_test_2_s2_thd;         //TEST_THD_13
	signed short i16_ft_test_thd_14;
	signed short i16_ft_test_thd_15;
	signed short i16_ft_test_thd_16;
	signed short i16_ft_test_thd_resv;
} st_test_threshold;
extern st_test_threshold g_st_test_thd;

typedef struct {
	//-------- FT Test Parameter (Byte 48)-----------------//
	unsigned char u8_para_resv_0[40];
	unsigned int u32_normal_fw_version;
	unsigned int u32_test_fw_version;
} st_test_para_resv;
extern st_test_para_resv g_st_test_para_resv;

#pragma pack(pop)

extern unsigned char g_u8_ic_test_state;
extern STATUS wearable_ic_test_read_info(void);
extern STATUS wearable_ic_test_init(void);
extern void handle_ic_test(void);
extern void wearable_ic_test_init_buffer(void);
extern STATUS handle_burn_log_to_flash(void);
extern void handle_set_display_interface(unsigned char u8_interface);

