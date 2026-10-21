/* f303_ic_test.h
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

/****************************************************************************
 *                            Defined Const Value
 *****************************************************************************/
#define FT_CMD_DO_FT_TEST					0x14

extern STATUS turn_on_flash_3x(void);
extern STATUS read_fpc_flash_3x(unsigned int u32_addr, unsigned int *p_u32_data);
extern STATUS set_test_info_thd_para_3x(void);
extern STATUS check_test_fw_status_3x(unsigned char u8_target_status, unsigned char *p_u8_result);
extern STATUS ft_test_do_fw_test_3x(unsigned short u16_test_items);
extern STATUS enter_fw_test_mode_3x(void);
extern STATUS system_test_3x(void);
extern STATUS ft_test_ctrl_mbist_fun_3x(unsigned char u8_enable);
extern STATUS ft_test_ram_test_3x(unsigned char u8_is_stop_mcu);
extern STATUS ft_test_connect_test_3x(void);
extern STATUS ft_test_reset_pin_test_3x(void);
extern STATUS ft_raw_data_checksum_check_3x(unsigned short *u16_buffer);
extern STATUS ft_test_result_checksum_check_3x(unsigned char *u8_buffer);
extern STATUS burn_cc_to_ic_flash_3x(void);
extern STATUS check_cc_bl_flag_3x(void);
extern STATUS read_test_fw_data_3x(unsigned short u16_test_items);
extern STATUS load_test_fw_3x(void);

extern void dump_image_data_3x(short *p_image_buf, unsigned char u8_remap);
extern void dump_image_hex_data_3x(short *p_image_buf);
extern STATUS ft_test_read_used_pin_infor_3x(unsigned char *p_u8_infor);
extern void ft_raw_data_checksum_cal_3x(unsigned short *u16_buffer);
extern void ft_test_result_checksum_cal_3x(unsigned char *u8_buffer);
extern STATUS baseline_update_control_3x(bool b_enable_baseline_update);
extern STATUS enter_normal_fw_3x(void);
extern STATUS do_calibration_3x(unsigned char u8_do_calibration_cmd, unsigned char u8_burn_flash);
extern STATUS hw_int_pin_Test_3x(void);
extern void test_item_message_3x(void);
extern STATUS burn_cc_3x(unsigned short u16_test_items);
extern void do_ic_test_3x(void);
extern STATUS burn_data_log_to_flash_3x(void);
extern STATUS burn_header_log_to_flash_3x(bool is_test_finish,  bool is_in_header_data_page);

//-----------------------------extern FT function ------------------------------------------
extern STATUS load_test_fw_ft_3x(void);
extern void do_ic_panel_test_3x(void);
extern STATUS burn_fw_3x(void);
extern STATUS burn_to_ic_flash_3x(unsigned char u8_type);
extern unsigned char notify_panel_jig_start_test_3x(unsigned char u8_cmd);

