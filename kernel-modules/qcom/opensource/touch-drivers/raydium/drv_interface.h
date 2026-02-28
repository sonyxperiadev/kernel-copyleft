/* drv_interface.h
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

#include "Config.h"
#include "raydium_driver.h"

#define DEBUGOUT pr_info
#define delay_ms msleep

#define I2C_INTERFACE			0
#define SPI_INTERFACE			1

#define M_RX_BUF				1
#define M_TX_BUF				0

#define     __I     volatile
#define     __O     volatile
#define     __IO    volatile


#define WORD					4

#define RAD_CMD_START_BURN		0x01
#define RAD_CMD_LOAD_TESTFW		0x04
#define RAD_CMD_UPDATE_BIN		0x80
#define RAD_CMD_UPDATE_END		0x81
#define RAD_CMD_BURN_FINISH		0x82

#define WEARABLE_FT_TEST_RESULT_AUO_JIG_NG WEARABLE_FT_TEST_RESULT_SYSFS_NG


extern unsigned char g_u8_m_buf[2][128];
extern struct timeval timer;
extern unsigned char g_rad_fw_flash[32768], g_rad_testfw_flash[32768];
extern unsigned char g_u8_ini_flash[0x400];
extern unsigned long g_u32_save_config[64];
extern unsigned char g_u8_mute_i2c_err_log;
extern short g_i16_raw_data_frame_buffer[101][50];
extern unsigned char g_u8_test_log_burn_times;

extern unsigned char read_flash_data(unsigned int u32_addr, unsigned short u16_lenth);
extern unsigned int get_system_time(void);

extern unsigned char sysfs_mode_control(unsigned char u8_type);
//extern unsigned char sysfs_read_int_flag(unsigned char *p_u8_flag, unsigned char u8_is_clear_flag);
extern unsigned char sysfs_write_int_flag(char *p_cmd);
//extern unsigned char sysfs_i2c_read(char *p_u8_addr, char u8_len, unsigned char *p_u8_data);
//extern unsigned char sysfs_i2c_write(char *p_u8_addr, char u8_len, unsigned char *p_u8_data);
extern unsigned char sysfs_do_cal(void);
extern unsigned char sysfs_burn_cc_bl(void);

extern void gpio_touch_reset_pin_control(unsigned char u8_high);
extern void gpio_touch_hw_reset(void);
extern unsigned char fw_upgrade(unsigned char type);
extern unsigned char gpio_touch_int_access(unsigned char u8_is_clear_flag);
extern unsigned char gpio_touch_int_pin_state_access(void);

extern unsigned char handle_read_word(unsigned int addr, unsigned int *p_data);
extern unsigned char handle_write_word(unsigned int u32_addr, unsigned int u32_data);
extern unsigned char handle_read_pda(unsigned int u32_addr, unsigned char u8_read_len, unsigned char *p_u8_output_buf);
extern unsigned char handle_write_pda(unsigned int u32_addr, unsigned char u8_write_len, unsigned char *bValue, unsigned char u8_trans_mode);
extern unsigned char handle_ic_read(unsigned int u32_addr, unsigned short u8_read_len, unsigned char *p_u8_output_buf, unsigned char u8_interface, unsigned char u8_trans_mode);
extern unsigned char handle_ic_write(unsigned int u32_addr, unsigned char u8_write_len, unsigned char *bValue, unsigned char u8_interface, unsigned char u8_trans_mode);

extern unsigned char  raydium_upgrade_test_fw(unsigned long ul_fw_addr);
extern unsigned char  raydium_upgrade_test_fw_3x(unsigned long ul_fw_addr);
extern unsigned char  raydium_upgrade_test_fw_2x(unsigned long ul_fw_addr);
extern void set_raydium_ts_data(struct raydium_ts_data *ts_old);

