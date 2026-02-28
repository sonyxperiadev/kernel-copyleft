/* i2c_drv_interface.c
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
#ifdef __KERNEL__
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#else
#include <stdint.h>
#include <string.h>
#endif
#include "ic_drv_interface.h"
#include "../drv_interface.h"
#include "ic_drv_global.h"
#if SELFTEST_2X
#include "f302_ic_control.h"
#include "f302_ic_test.h"
#endif
#if SELFTEST_3X
#include "f303_ic_control.h"
#include "f303_ic_test.h"
#endif
#if !SELFTEST
#include "f302_ic_test.h"
#include "f302_ic_test_ft.h"
#include "f303_ic_test.h"
#include "f303_ic_test_ft.h"
#include "ic_drv_global_ft.h"
#include "main.h"
#endif

st_test_threshold g_st_test_thd;
st_test_info g_st_test_info;
st_test_para_resv g_st_test_para_resv;
unsigned char g_u8_ic_test_state;

STATUS wearable_ic_test_read_info(void)
{
	if (read_flash_data(g_u32_dongle_flash_ini_addr, 16) != ERROR) {
		memcpy(&g_st_test_info, g_u8_data_buf, sizeof(g_st_test_info));
	} else {
		DEBUGOUT("Read Flash Data ERROR\r\n");
		return ERROR;
	}

	return SUCCESS;
}

STATUS wearable_ic_test_info_init(void)
{
#if SELFTEST
	g_u16_test_items_host_cmd = IC_TEST_ITEMS_SYSTEM | IC_TEST_ITEMS_OPEN | IC_TEST_ITEMS_SHORT |  IC_TEST_ITEMS_UC | IC_TEST_ITEMS_UB | IC_TEST_ITEMS_BURN_CC;
#endif

	if (!wearable_ic_test_read_info()) {
		DEBUGOUT("Read Info ERROR\r\n");
		return ERROR;
	}

	if (g_u16_test_items_host_cmd != 0)
		g_st_test_info.u16_ft_test_item &= g_u16_test_items_host_cmd;
	g_u16_panel_jig_set_test_items |= g_st_test_info.u16_ft_test_item;

	DEBUGOUT("TestItem:0x%x:0x%x\r\n", g_st_test_info.u16_ft_test_item, g_st_test_info.u16_ft_eng_item);
	DEBUGOUT("g_u16_panel_jig_set_test_items:0x%x\r\n", g_u16_panel_jig_set_test_items);
	if (read_flash_data(g_u32_ini_threshold_addr, 36)) {
		memcpy(&g_st_test_thd, g_u8_data_buf, sizeof(g_st_test_thd));
	} else {
		DEBUGOUT("Read THD Data ERROR\r\n");
		return ERROR;
	}

	DEBUGOUT("THD:\r\n%d,%d,%d,%d,\r\n%d,%d,%d,%d,\r\n%d,%d,%d,%d\r\n",
		 g_st_test_thd.i16_ft_test_open_lower_thd,
		 g_st_test_thd.i16_ft_test_short_upper_thd,
		 g_st_test_thd.i16_ft_test_short_lower_thd,
		 g_st_test_thd.i16_ft_test_single_cc_upper_thd,
		 g_st_test_thd.i16_ft_test_single_cc_lower_thd,
		 g_st_test_thd.i16_ft_test_uniformity_bl_upper_thd,
		 g_st_test_thd.i16_ft_test_uniformity_bl_lower_thd,
		 g_st_test_thd.i16_ft_test_uniformity_cc_upper_thd,
		 g_st_test_thd.i16_ft_test_uniformity_cc_lower_thd,
		 g_st_test_thd.i16_ft_test_panel_test_1_thd,
		 g_st_test_thd.i16_ft_test_panel_test_3_thd,
		 g_st_test_thd.i16_ft_test_panel_test_2_thd);

	if (read_flash_data(g_u32_ini_para_addr, 48)) {
		memcpy(&g_st_test_para_resv, g_u8_data_buf, sizeof(g_st_test_para_resv));
	} else {
		DEBUGOUT("Read INI Para ERROR\r\n");
		return ERROR;
	}
/*
 *	DEBUGOUT(" g_st_test_para_resv.u32_normal_fw_version = %X ,g_st_test_para_resv.u32_test_fw_version= %X \r\n",
 *		 g_st_test_para_resv.u32_normal_fw_version,
 *		 g_st_test_para_resv.u32_test_fw_version
 *		);
 */

	if (g_u16_dev_id == DEVICE_ID_3X) {
		if (read_flash_data(g_u32_ini_raw_data_3_cc_addr, 128)) {
			memcpy(g_u16_raw_data3_golden_cc_buf, g_u8_data_buf, sizeof(g_u16_raw_data3_golden_cc_buf));
		} else {
			DEBUGOUT("read raw data 3 cc ERROR\r\n");
			return ERROR;
		}
/*
 *		DEBUGOUT(" g_u16_raw_data3_golden_cc_buf[0] = %d,g_u16_raw_data3_golden_cc_buf[1 =%d g_u16_raw_data3_golden_cc_buf[2]=%d,\r\n",
 *			 g_u16_raw_data3_golden_cc_buf[0],
 *			 g_u16_raw_data3_golden_cc_buf[1],
 *			 g_u16_raw_data3_golden_cc_buf[2]
 *			);
 */
		if (read_flash_data(g_u32_ini_uc_cc_addr, 128)) {
			memcpy(g_u16_uc_golden_cc_buf, g_u8_data_buf, sizeof(g_u16_uc_golden_cc_buf));
		} else {
			DEBUGOUT("read uc cc ERROR\r\n");
			return ERROR;
		}
	} else if (g_u16_dev_id == DEVICE_ID_2X) {
		if (read_flash_data(g_u32_ini_raw_data_3_cc_addr, 72)) {
			memcpy(g_u16_raw_data3_golden_cc_buf, g_u8_data_buf, sizeof(g_u16_raw_data3_golden_cc_buf));
		} else {
			DEBUGOUT("read raw data 3 cc ERROR\r\n");
			return ERROR;
		}
/*
 *		DEBUGOUT(" g_u16_raw_data3_golden_cc_buf[0] = %d,g_u16_raw_data3_golden_cc_buf[1 =%d g_u16_raw_data3_golden_cc_buf[2]=%d,\r\n",
 *			 g_u16_raw_data3_golden_cc_buf[0],
 *			 g_u16_raw_data3_golden_cc_buf[1],
 *			 g_u16_raw_data3_golden_cc_buf[2]
 *			);
 */
		if (read_flash_data(g_u32_ini_uc_cc_addr, 72)) {
			memcpy(g_u16_uc_golden_cc_buf, g_u8_data_buf, sizeof(g_u16_uc_golden_cc_buf));
		} else {
			DEBUGOUT("read uc cc ERROR\r\n");
			return ERROR;
		}
	}
/*	if (read_flash_data(INI_RAW_DATA2_BL_ADDR, 72))
 *		memcpy(g_i16_raw_data2_golden_bl_buf, g_u8_data_buf, sizeof(g_i16_raw_data2_golden_bl_buf));
 */
	return SUCCESS;
}

STATUS wearable_ic_test_init(void)
{
	if (!wearable_ic_test_info_init())
		return ERROR;
	if (!(g_st_test_info.u16_ft_eng_item & IC_TEST_ENG_ITEMS_PANEL_TEST_JIG))
		wearable_ic_test_init_buffer();

	return SUCCESS;
}

void wearable_ic_test_init_buffer(void)
{
	g_u32_wearable_test_result = WEARABLE_FT_TEST_RESULT_IC_INIT_STATE;
	memset(g_u32_test_result, 0, sizeof(g_u32_test_result));
	memset(g_u8_test_result, 0, sizeof(g_u8_test_result));
	g_u8_is_normal_fw = FALSE;
}

STATUS handle_burn_log_to_flash(void)
{
	STATUS u8_status = ERROR;

	switch (g_u16_dev_id) {
#if (!SELFTEST_2X && (!SELFTEST))
	case DEVICE_ID_2X:
		burn_data_log_to_flash_2x();
		break;
#endif
#if (!SELFTEST_3X && (!SELFTEST))
	case DEVICE_ID_3X:
		b_is_auo_jig_testing = false;
		burn_header_log_to_flash_3x(TRUE, TRUE);
		memset(g_u32_save_config, 0, sizeof(g_u32_save_config));
		if (burn_data_log_to_flash_3x())
			u8_status = SUCCESS;
		break;
#endif
	}
	return u8_status;
}

void handle_ic_test(void)
{

	switch (g_u16_dev_id) {
#if (SELFTEST_2X | (!SELFTEST))
	case DEVICE_ID_2X:
		do_ic_test_2x();
		break;
#endif
#if (SELFTEST_3X | (!SELFTEST))
	case DEVICE_ID_3X:
		do_ic_test_3x();
		break;
#endif
	}
}

void handle_set_display_interface(unsigned char u8_interface)
{
	g_u8_display_interface = u8_interface;
	DEBUGOUT("%s: 0x%x \r\n", __func__, u8_interface);
}

