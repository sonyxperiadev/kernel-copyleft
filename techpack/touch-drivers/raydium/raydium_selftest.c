/* raydium_selftest.c
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

#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "drv_interface.h"
#include "chip_raydium/ic_drv_interface.h"
#include "raydium_selftest.h"
#include "chip_raydium/ic_drv_global.h"
#include "tpselftest_30.h"
#if defined(FW_MAPPING_EN)
#include "tpselftest_21.h"
#endif

#define RM_SELF_TEST_CUSTOMER_VERSION	0x01
#define RM_SELF_TEST_PLATFORM_VERSION	0x01
#define RM_SELF_TEST_PROJECT_VERSION		0x40
#define RM_SELF_TEST_MAIN_VERSION		1
#define RM_SELF_TEST_SUB_VERSION		0

#define RESULT_SUCCESS 0
#define RESULT_NG 1

#define RELATIVE_PATH 0

#define RM_SELF_TEST_MAX_STR_LENGTH		1000

unsigned char g_u8_normal_fw_version_buf[4];
char str_ini_path[100];

static int self_test_all(void)
{
	int ret = 0;

	g_u8_raydium_flag |= ENG_MODE;
	handle_ic_test();
	ret = g_u32_wearable_test_result;

	/*g_u8_raydium_flag &= ~ENG_MODE;*/
	DEBUGOUT("%s\r\n", __func__);

	return ret;
}

int self_test_save_to_file(char *file_name, char *p_string, short len)
{
	struct file *filp = NULL;
	mm_segment_t old_fs;

	filp = filp_open(file_name, O_RDWR | O_CREAT | O_APPEND, 0666);
	if (IS_ERR(filp)) {
		DEBUGOUT("can't open file:%s\n", RM_SELF_TEST_LOGFILE);
		return 0;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	filp->f_op->write(filp, p_string, len, &filp->f_pos);
	set_fs(old_fs);
	filp_close(filp, NULL);

	return 1;
}

#if 1
static int raydium_check_ini_version(void)
{
	int ret = 0;
	unsigned int u32_test_version;

	memcpy(&u32_test_version, &g_rad_testpara_image[4], 4);

	if (u32_test_version != g_st_test_para_resv.u32_test_fw_version) {
		DEBUGOUT("test fw versio 0x%X != ini version 0x%X\n"
			 , u32_test_version, g_st_test_para_resv.u32_test_fw_version);
		ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;
	}
	if (g_u16_dev_id == DEVICE_ID_3X) {

		g_u32_dongle_flash_ini_addr = F303_DONGLE_FLASH_INI_ADDR;
		g_u32_ini_threshold_addr = F303_INI_THRESHOLD_ADDR;
		g_u32_ini_para_addr = F303_INI_PARA_ADDR;
		g_u32_ini_raw_data_3_cc_addr = F303_INI_RAW_DATA_3_CC_ADDR;
		g_u32_ini_uc_cc_addr = F303_INI_UC_CC_ADDR;
		g_u32_initial_code_start_addr = F303_INITIAL_CODE_START_ADDR;
		DEBUGOUT("[out_set_ic_version] F303 Set INI ADDR!\r\n");
	} else if (g_u16_dev_id == DEVICE_ID_2X) {
		g_u32_dongle_flash_ini_addr = F302_DONGLE_FLASH_INI_ADDR;
		g_u32_ini_threshold_addr = F302_INI_THRESHOLD_ADDR;
		g_u32_ini_para_addr = F302_INI_PARA_ADDR;
		g_u32_ini_raw_data_3_cc_addr = F302_INI_RAW_DATA_3_CC_ADDR;
		g_u32_ini_uc_cc_addr = F302_INI_UC_CC_ADDR;
		g_u32_initial_code_start_addr = F302_INITIAL_CODE_START_ADDR;
		DEBUGOUT("[out_set_ic_version] F302 Set INI ADDR!\r\n");
	}
	return ret;
}
#else
static int raydium_check_ini_version(void)
{
	int ret = 0;
	unsigned int u32_test_version, u32_version_20, u32_version_21;

	memcpy(&u32_test_version, &g_st_test_para_resv.u32_test_fw_version, 4);

	if (g_u16_dev_id == DEVICE_ID_2X) {
		switch (g_raydium_ts->id) {
		case RAD_SELFTEST_20:
			memcpy(&u32_version_20, &u8_rad_testpara_20[4], 4);
			DEBUGOUT("ini version 0x%X, 20 version 0x%X\n"
				 , u32_test_version, u32_version_20);

			if (u32_test_version == u32_version_20)
				DEBUGOUT("map version= 0x%X\r\n", u32_version_20);
			else
				ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;
		case RAD_SELFTEST_21:
			memcpy(&u32_version_21, &u8_rad_testpara_21[4], 4);
			DEBUGOUT("ini version 0x%X, 21 version 0x%X\n"
				 , u32_test_version, u32_version_21);

			if (u32_test_version == u32_version_21)
				DEBUGOUT("map version= 0x%X\r\n", u32_version_21);
			else
				ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;
		}
	}

	return ret;
}
#endif
static int self_test_init(void)
{
	int ret = 0;
	unsigned int u32_dev_id;

	g_u8_drv_interface = I2C_INTERFACE;
	g_u16_dev_id = DEVICE_ID_3X;

	if (handle_ic_read(RAYDIUM_CHK_I2C_CMD, 4, (unsigned char *)&u32_dev_id, g_u8_drv_interface, I2C_WORD_MODE) == ERROR) {
		ret = WEARABLE_FT_TEST_RESULT_SYSFS_NG;
		return ret;
	}
	g_u16_dev_id = ((u32_dev_id & 0xFFFF0000) >> 16);

	if (g_u16_dev_id == DEVICE_ID_2X) {
		handle_ic_read(0x00006a04, 4, g_u8_normal_fw_version_buf, g_u8_drv_interface, I2C_WORD_MODE);
		DEBUGOUT("FW Version=0x%.2X%.2X%.2X%.2X\n", g_u8_normal_fw_version_buf[0], g_u8_normal_fw_version_buf[1],
			 g_u8_normal_fw_version_buf[3], g_u8_normal_fw_version_buf[2]);
	} else if (g_u16_dev_id == DEVICE_ID_3X) {
		handle_ic_read(0x00007b04, 4, g_u8_normal_fw_version_buf, g_u8_drv_interface, I2C_WORD_MODE);
		DEBUGOUT("FW Version=0x%.2X%.2X%.2X%.2X\n", g_u8_normal_fw_version_buf[0], g_u8_normal_fw_version_buf[1],
			 g_u8_normal_fw_version_buf[3], g_u8_normal_fw_version_buf[2]);
	} else {
		DEBUGOUT("read ic namd fail\n");
		ret = WEARABLE_FT_TEST_RESULT_TEST_INIT_NG;
		return ret;
	}

	if (raydium_check_ini_version() != 0)
		ret = WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG;

	return ret;
}

int self_test_save_test_raw_data_to_file(int i32_ng_type)
{
	/*struct tm *time_infor;*/
	/*time_t raw_time;*/
	char write_string[1000];
	unsigned char u8_i, u8_j;
	short *p_i16_buf = (short *)g_u16_raw_data_tmp;

	/*FW Version*/
	memset(write_string, 0, strlen(write_string));
	if (g_u16_dev_id != 0) {
		snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH,
			"FW Version=0x%.2X%.2X%.2X%.2X\n",
			g_u8_normal_fw_version_buf[0], g_u8_normal_fw_version_buf[1],
			g_u8_normal_fw_version_buf[3], g_u8_normal_fw_version_buf[2]);
	}
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	/*Version*/
	memset(write_string, 0, strlen(write_string));
	snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "Selftest Version=%x.%x.%x.%x.%x\n\n",
		RM_SELF_TEST_CUSTOMER_VERSION, RM_SELF_TEST_PLATFORM_VERSION,
		RM_SELF_TEST_PROJECT_VERSION, RM_SELF_TEST_MAIN_VERSION,
		RM_SELF_TEST_SUB_VERSION);
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	/*Test result*/
	memset(write_string, 0, strlen(write_string));
	snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "Test Result = 0x%08X\n\n",
		i32_ng_type);
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	if (i32_ng_type == 0) {
		memset(write_string, 0, strlen(write_string));
		snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "All pass\n\n\n");
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	} else {
		memset(write_string, 0, strlen(write_string));

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SYSFS_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "System NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_I2C_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "I2C NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_INT_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "INT NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_RESET_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "RESET NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_PRAM_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "PRAM NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_NORMAL_FW_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "NORMAL_FW_NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_OPEN_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "OPEN NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SHORT_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "SHORT NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_BURN_CC_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "BURN CC NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_GET_DATA_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "GET DATA NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_FLASH_ID_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "FLASH ID NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_NORMAL_FW_VER_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "NORMAL FW VER NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_TEST_FW_VER_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "TEST FW VER NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_TEST_INIT_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "TEST INIT NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_LOAD_TESTFW_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "LOAD TESTFW NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_BURN_FW_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "BURN FW NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SINGLE_CC_OPEN_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "Open NG (Single Pin CC) ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_SINGLE_CC_SHORT_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "Short NG (Single Pin CC) ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_UB_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "Uniformity Baseline NG ");
		}

		if (i32_ng_type & WEARABLE_FT_TEST_RESULT_UC_NG) {
			snprintf(write_string + strlen(write_string),
				RM_SELF_TEST_MAX_STR_LENGTH, "Uniformity CC NG ");
		}

		snprintf(write_string + strlen(write_string), RM_SELF_TEST_MAX_STR_LENGTH, "\n");
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	}

	/*Threshold*/
	memset(write_string, 0, strlen(write_string));
	/*snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "0x%02X, 0x%02X\n0x%02X,
		0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n0x%02X,
		0x%02X\n0x%02X, 0x%02X\n0x%02X, 0x%02X\n",
		(g_st_test_thd.i16_ft_test_open_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_open_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_short_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_short_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_short_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_short_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_single_cc_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_single_cc_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_single_cc_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_single_cc_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_bl_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_bl_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_bl_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_bl_lower_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_cc_upper_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_cc_upper_thd & 0xFF),
		(g_st_test_thd.i16_ft_test_uniformity_cc_lower_thd >> 8) & 0xFF, (g_st_test_thd.i16_ft_test_uniformity_cc_lower_thd & 0xFF));*/

	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));


	for (u8_i = 0; u8_i < MAX_SENSING_PIN_NUM; u8_i++) {
		memset(write_string, 0, strlen(write_string));
		snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "0x%2X,",
			g_u16_uc_golden_cc_buf[u8_i]);
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	}

	memset(write_string, 0, strlen(write_string));
	snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\n");
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	for (u8_i = 0; u8_i < MAX_SENSING_PIN_NUM; u8_i++) {
		memset(write_string, 0, strlen(write_string));
		snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "0x%2X,",
			g_u16_raw_data3_golden_cc_buf[u8_i]);
		self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
	}

	memset(write_string, 0, strlen(write_string));
	snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\n\n\n\n\n\n\n\n\n\n\n\n\n");
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	if ((i32_ng_type & 0xFFF0FBF8) < 4) {

		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_SHORT)) != 0) {
			/*Raw data*/
			/*Raw data slow*/
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "Raw Data 1\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data1*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_i16_raw_data_1_short_buf[u8_i];

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH,
						"%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_OPEN)) != 0) {
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			/*Raw data slow*/
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "Raw Data 2\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data2*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_i16_raw_data_2_open_buf[u8_i];

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH,
						"%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_OPEN | IC_TEST_ITEMS_SHORT)) != 0) {
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			/*Raw data 3*/
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "Raw Data 3\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data3*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_u16_raw_data3_cc_buf[u8_i];

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH,
						"%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
		if ((g_st_test_info.u16_ft_test_item & (IC_TEST_ITEMS_UC)) != 0) {
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n\n\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			/*Raw data Uniformity CC*/
			memset(write_string, 0, strlen(write_string));
			snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "Raw Data_UC\n");
			self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
			/*Raw data uc*/
			memset(p_i16_buf, 0, MAX_IMAGE_BUFFER_SIZE * 2);
			for (u8_i = 0; u8_i < MAX_IMAGE_BUFFER_SIZE; u8_i++)
				if (g_u8_wearable_pin_map[u8_i] != F303_NA_P)
					p_i16_buf[g_u8_wearable_pin_map[u8_i]] = g_u16_uc_buf[u8_i];

			DEBUGOUT("Image:0x%x\r\n", p_i16_buf[0]);

			for (u8_j = 0; u8_j < g_u8_channel_y; u8_j++) {
				for (u8_i = 0; u8_i < g_u8_channel_x; u8_i++) {
					memset(write_string, 0, strlen(write_string));
					snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH,
						"%05d,", p_i16_buf[u8_i + u8_j * g_u8_channel_x]);
					self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));
				}

				memset(write_string, 0, strlen(write_string));
				snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n");
				self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

			}
		}
	}

	memset(write_string, 0, strlen(write_string));
	snprintf(write_string, RM_SELF_TEST_MAX_STR_LENGTH, "\r\n\n\n");
	self_test_save_to_file(RM_SELF_TEST_LOGFILE, write_string, strlen(write_string));

	return 1;
}

int self_test_read_setting_from_file(void)
{
	unsigned short u16_offset = 0;

	DEBUGOUT("[touch]g_raydium_ts->id = 0x%x\r\n", g_raydium_ts->id);
	switch (g_raydium_ts->id) {
	case RAD_SELFTEST_30:
		u16_offset = 0;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_info_30, sizeof(u8_test_info_30));
		u16_offset += 16;
		memcpy(&g_u8_ini_flash[u16_offset], &i8_ft_test_thd_30, sizeof(i8_ft_test_thd_30));
		u16_offset += 36;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_para_30, sizeof(u8_test_para_30));
		u16_offset += 48;
		u16_offset += 128;/*reserve for BL*/
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_data_3_cc_30, sizeof(u8_raw_data_3_cc_30));
		u16_offset += 128;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_uc_cc_30, sizeof(u8_raw_uc_cc_30));
		u16_offset += 128;

		memcpy((void *)(&g_st_test_para_resv), &u8_test_para_30[0], sizeof(g_st_test_para_resv));
		DEBUGOUT("ini length = %d\r\n", u16_offset);
		break;
#if defined(FW_MAPPING_EN)
	case RAD_SELFTEST_31:
		u16_offset = 0;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_info_31, sizeof(u8_test_info_31));
		u16_offset += 16;
		memcpy(&g_u8_ini_flash[u16_offset], &i8_ft_test_thd_31, sizeof(i8_ft_test_thd_31));
		u16_offset += 36;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_test_para_31, sizeof(u8_test_para_31));
		u16_offset += 48;
		u16_offset += 128;/*reserve for BL*/
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_data_3_cc_31, sizeof(u8_raw_data_3_cc_31));
		u16_offset += 128;
		memcpy(&g_u8_ini_flash[u16_offset], &u8_raw_uc_cc_31, sizeof(u8_raw_uc_cc_31));
		u16_offset += 128;

		memcpy((void *)(&g_st_test_para_resv), &u8_test_para_31[0], sizeof(g_st_test_para_resv));
		DEBUGOUT("ini length = %d\r\n", u16_offset);
		break;
#endif
	}

	return 0;
}

int raydium_do_selftest(struct raydium_ts_data *ts)
{
	int ret = RESULT_SUCCESS;
	unsigned int time_start, time_end, time_start2, time_end2;

	time_start = get_system_time();

	pr_info("Selftest Version=%x.%x.%x.%x.%x\n", RM_SELF_TEST_CUSTOMER_VERSION, RM_SELF_TEST_PLATFORM_VERSION,
		RM_SELF_TEST_PROJECT_VERSION, RM_SELF_TEST_MAIN_VERSION, RM_SELF_TEST_SUB_VERSION);

	self_test_read_setting_from_file();
	ic_drv_init();
	set_raydium_ts_data(ts);

	ret = self_test_init();
	if (ret != 0)
		DEBUGOUT("mapping ic fw fail\n");
	else {
		DEBUGOUT("Test all\n");
		ret |= self_test_all();
	}

	if (ret != WEARABLE_FT_TEST_RESULT_SYSFS_NG) {
		gpio_touch_hw_reset();
		g_u8_raydium_flag &= ~ENG_MODE;
	}

	raydium_i2c_mode_control(ts->client, ENABLE_TOUCH_INT);
#if ENABLE_TIME_MEASURMENT
	time_start2 = get_system_time();
#endif
	self_test_save_test_raw_data_to_file(ret);

#if ENABLE_TIME_MEASURMENT
	time_end2 = get_system_time();
	DEBUGOUT("Write log Finish(%ums)\n", time_end2 - time_start2);
#endif
	if (ret != 0) {
		DEBUGOUT("Selftest Test Result=0x%x\n", ret);
		ret = RESULT_NG;
		DEBUGOUT("Selftest Result=%d\n", ret);
	} else {
		DEBUGOUT("Selftest Pass ^_^!!!\n");
		ret = RESULT_SUCCESS;
	}

	time_end = get_system_time();
	DEBUGOUT("All Test Finish(%ums)\n", time_end - time_start);


	return ret;
}
