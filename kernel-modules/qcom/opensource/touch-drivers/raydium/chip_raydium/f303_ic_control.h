/* f303_ic_control.h
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

#define HEADER_LENGTH                            (4)//length + checksum

extern unsigned char enable_ic_block_3x(void);
extern unsigned char stop_mcu_3x(unsigned char u8_is_tp_reset);
extern unsigned char hardware_reset_3x(unsigned char u8_enable_ic_block);
extern unsigned char check_dev_id_3x(unsigned short u16_dev_id);
extern unsigned char check_dev_sub_version_3x(unsigned char u8_version);
extern unsigned char set_fw_system_cmd_3x(unsigned int u32_sysm_cmd);
extern unsigned char  wait_fw_state_3x(unsigned int u32_addr, unsigned int u32_state, unsigned short u16_delay, unsigned short u16_retry);
extern unsigned char disable_i2c_deglitch_3x(void);
extern unsigned char WriteDriverByTouchMode(const unsigned char *p_u8_data, uint16_t u16DataLength);
