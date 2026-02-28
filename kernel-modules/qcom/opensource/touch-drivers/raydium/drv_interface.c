/* drv_interface.c
 *
 * Raydium TouchScreen driver.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 * Qualcomm Innovation Center, Inc. chooses to use it under GPLv2
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
 * BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Google Inc. or Linaro Ltd. nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/timer.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/unistd.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/version.h>
#include "chip_raydium/ic_drv_global.h"
#include "chip_raydium/ic_drv_interface.h"
#include "drv_interface.h"
#include "raydium_selftest.h"
#include "raydium_driver.h"
#include "chip_raydium/f303_ic_control.h"

unsigned char g_u8_m_buf[2][128];
unsigned char g_u8_ini_flash[0x400];
struct raydium_ts_data *ts;
unsigned char g_u8_mute_i2c_err_log;


STATUS i2c_burst_read_pda2(unsigned char u8_addr, unsigned short u16ReadLen, unsigned char *p_u8_output_buf)
{
	return ERROR;
}
STATUS i2c_burst_write_pda2(unsigned char u8_addr, unsigned char bWriteLen, unsigned char *bValue)
{
	return ERROR;
}

unsigned char spi_write_pda(unsigned int u32_addr, unsigned char u8_write_len, unsigned char *bValue, unsigned char u8_trans_mode)
{
	return ERROR;
}
unsigned char spi_read_pda(unsigned int u32_addr, unsigned char u8_read_len, unsigned char *p_u8_output_buf)
{
	return ERROR;
}
STATUS burn_fw_3x(void)
{
	g_u32_wearable_test_result &= ~(WEARABLE_FT_TEST_RESULT_CMD_NG | WEARABLE_FT_TEST_RESULT_BURN_FW_NG);
	return SUCCESS;
}
unsigned char fw_upgrade(unsigned char type)
{
	int i32_ret = ERROR;

	i32_ret = raydium_burn_fw(g_raydium_ts->client);
	if (i32_ret < 0)
		pr_err("[touch]FW update fail:%d\n", i32_ret);
	return i32_ret;

}
unsigned char read_flash_data(unsigned int u32_addr, unsigned short u16_lenth)
{
	unsigned int u32_data_offset;

	if (g_u16_dev_id == DEVICE_ID_2X) {
		u32_data_offset = u32_addr - 0x800;

		if (u32_addr < 0x8000) {
			if (u32_data_offset >= 0x8000 && u32_addr < F302_DONGLE_FLASH_INI_ADDR) {
				u32_data_offset = u32_data_offset - 0x6200;
				memcpy(g_u8_data_buf, g_rad_para_image + u32_data_offset, u16_lenth);
			} else if (u32_addr >= F302_DONGLE_FLASH_INI_ADDR) {
				u32_data_offset = u32_addr - F302_DONGLE_FLASH_INI_ADDR;
				memcpy(g_u8_data_buf, g_u8_ini_flash + u32_data_offset, u16_lenth);
			}  else
				memcpy(g_u8_data_buf, g_rad_fw_image + u32_data_offset, u16_lenth);
		} else {
			u32_data_offset -= 0x8000;
			if (u32_data_offset >= 0x6200 && u32_addr < F302_DONGLE_FLASH_INI_ADDR) {
				u32_data_offset = u32_data_offset - 0x6200;
				memcpy(g_u8_data_buf, g_rad_testpara_image + u32_data_offset, u16_lenth);
			} else if (u32_addr >= F302_DONGLE_FLASH_INI_ADDR) {
				u32_data_offset = u32_addr - F302_DONGLE_FLASH_INI_ADDR;
				memcpy(g_u8_data_buf, g_u8_ini_flash + u32_data_offset, u16_lenth);
			} else {
				memcpy(g_u8_data_buf, g_rad_testfw_image + u32_data_offset, u16_lenth);
			}
		}
		return TRUE;
	} else if (g_u16_dev_id == DEVICE_ID_3X) {
		u32_data_offset = u32_addr - 0x800;

		if (u32_addr < 0x7800) {
			if (u32_data_offset >= 0x7300 && u32_addr < F303_DONGLE_FLASH_INI_ADDR) {
				u32_data_offset = u32_data_offset - 0x7300;
				memcpy(g_u8_data_buf, g_rad_para_image + u32_data_offset, u16_lenth);
			} else
				memcpy(g_u8_data_buf, g_rad_fw_image + u32_data_offset, u16_lenth);
		} else {
			u32_data_offset -= 0x7800;
			if (u32_data_offset >= 0x7300 && u32_addr < F303_DONGLE_FLASH_INI_ADDR) {
				u32_data_offset = u32_data_offset - 0x7300;
				memcpy(g_u8_data_buf, g_rad_testpara_image + u32_data_offset, u16_lenth);
			} else if (u32_addr >= F303_DONGLE_FLASH_INI_ADDR) {
				u32_data_offset = u32_addr - F303_DONGLE_FLASH_INI_ADDR;
				pr_info("ini addr 0x%x offset %d\r\n", u32_addr, u32_data_offset);
				memcpy(g_u8_data_buf, g_u8_ini_flash + u32_data_offset, u16_lenth);
			} else {
				memcpy(g_u8_data_buf, g_rad_testfw_image + u32_data_offset, u16_lenth);
			}
		}
		return TRUE;
	}
	return FALSE;
}

unsigned int get_system_time(void)
{
/*	unsigned int u32_timer;
 *	struct timeval timer;
 *	do_gettimeofday(&timer);
 *	u32_timer = (timer.tv_sec % 1000) * 1000 + (timer.tv_usec / 1000);
 *	return u32_timer;
 */
	#if (KERNEL_VERSION(5, 0, 0) <= LINUX_VERSION_CODE)
	struct timespec64 ts;

	ktime_get_ts64(&ts);
	return (ts.tv_sec*1000 + ts.tv_nsec/1000000);
	#else
	struct timeval tv;

	do_gettimeofday(&tv);
	return (tv.tv_sec*1000 + tv.tv_usec/1000);
	#endif
}

unsigned char gpio_touch_int_pin_state_access(void)
{
	return gpio_get_value(ts->irq_gpio);
}

unsigned char gpio_touch_int_access(unsigned char u8_is_clear_flag)
{
	unsigned char u8_flag = g_u8_raydium_flag;

	if (u8_is_clear_flag && u8_flag)
		g_u8_raydium_flag &= ~INT_FLAG;

	return u8_flag;
}

unsigned char sysfs_burn_cc_bl(void)
{
	unsigned char ret = ERROR;

	DEBUGOUT("%s\r\n", __func__);
	ret = raydium_burn_comp(ts->client);
	return ret;
}

unsigned char raydium_upgrade_test_fw_2x(unsigned long ul_fw_addr)
{
	int ret = ERROR;
	unsigned char u8_retry = 2;
	unsigned int u32_read;
	unsigned int u32_write;

RETRY:
	gpio_touch_reset_pin_control(0);
	delay_ms(10);
	gpio_touch_reset_pin_control(1);
	delay_ms(2);
	u32_write = 0x00000030;
	handle_ic_write(0x50000918, 4, (unsigned char *)&u32_write, g_u8_drv_interface, I2C_WORD_MODE);

	if (raydium_load_test_fw(ts->client) == SUCCESS) {
		ret = SUCCESS;
		DEBUGOUT("### Raydium Load test FW SUCCESS ###\n");
	}

	handle_ic_read(0x6A04, 4, (unsigned char *)&u32_read, g_u8_drv_interface, I2C_WORD_MODE);

	if (u32_read != g_st_test_para_resv.u32_test_fw_version) {
		DEBUGOUT("Read FW version NG=0x%x:0x%x!!\r\n", u32_read, g_st_test_para_resv.u32_test_fw_version);
		goto ERROR_EXIT;
	}

	return ret;

ERROR_EXIT:
	if (u8_retry) {
		u8_retry--;
		goto RETRY;
	}

	return ERROR;
}

unsigned char raydium_upgrade_test_fw_3x(unsigned long ul_fw_addr)
{
	int ret = ERROR;
	unsigned char u8_retry = 2;
	unsigned int u32_read;

RETRY:

	if (raydium_load_test_fw(ts->client) == SUCCESS) {
		ret = SUCCESS;
		DEBUGOUT("### Raydium Load test FW SUCCESS ###\n");
	}

	handle_ic_read(0x7B04, 4, (unsigned char *)&u32_read, g_u8_drv_interface, I2C_WORD_MODE);
	if (u32_read != g_st_test_para_resv.u32_test_fw_version) {
		DEBUGOUT("Read FW version NG=0x%x:0x%x!!\r\n", u32_read, g_st_test_para_resv.u32_test_fw_version);
		goto ERROR_EXIT;
	}

	return ret;

ERROR_EXIT:
	if (u8_retry) {
		u8_retry--;
		goto RETRY;
	}

	return ERROR;
}

void gpio_touch_reset_pin_control(unsigned char u8_high)
{

	if (u8_high)
		gpio_set_value(ts->rst_gpio, 1);
	else
		gpio_set_value(ts->rst_gpio, 0);

	return;

}


void gpio_touch_hw_reset(void)
{
	gpio_touch_reset_pin_control(0);
	delay_ms(10);
	gpio_touch_reset_pin_control(1);

}
void set_raydium_ts_data(struct raydium_ts_data *ts_old)
{
	ts = ts_old;
}

/******************************************************************************
 **Function name:handle_ic_read
 **
 **Descriptions:handle read data from ic
 **
 **parameters:u32_addr,Address
 **	      u8_read_len,Read datalength
 **	      p_u8_output_buf,Data buffer
 **	      u8_interface,SPI or I2C
 **	      u8_trans_modePDA2_MODE, PDA_WORD_MODE, PDA_BYTE_MODE, MCU_MODE
 **
 **Returned value:ERROR,SUCCESS
 **
 ******************************************************************************
 */
unsigned char handle_ic_read(
	unsigned int u32_addr,
	unsigned short u8_read_len,
	unsigned char *p_u8_output_buf,
	unsigned char u8_interface,
	unsigned char u8_trans_mode)
{
	if (u8_trans_mode == I2C_PDA2_MODE) {
		/*PDA2 MODE */
		if (raydium_i2c_pda2_read(g_raydium_ts->client, (unsigned char)u32_addr, p_u8_output_buf, u8_read_len) == ERROR) {
			DEBUGOUT("%s\r\n", __func__);
			return ERROR;
		}
	} else {
		/*PDA MODE*/
		if ((u8_trans_mode == I2C_WORD_MODE) && (u32_addr & 0x00000003)) {
			DEBUGOUT("[HRW] Handle Read Word ADDR Not Word Align!!\r\n");
			return ERROR;
		}

		if (u8_interface == SPI_INTERFACE) {
			if (spi_read_pda(u32_addr, u8_read_len, p_u8_output_buf) == ERROR) {
				DEBUGOUT("%s\r\n", __func__);
				return ERROR;
			}
		} else {
			if (g_u16_dev_id == DEVICE_ID_3X) {
				if ((u8_interface & I2C_PDA_MODE) != 0)  {
					if (raydium_i2c_pda_read(g_raydium_ts->client, u32_addr, p_u8_output_buf, u8_read_len) == ERROR) {
						DEBUGOUT("%s\r\n", __func__);
						return ERROR;
					}
				} else {
					if (raydium_i2c_read_pda_via_pda2(g_raydium_ts->client, u32_addr, p_u8_output_buf, u8_read_len) == ERROR) {
						DEBUGOUT("%s\r\n", __func__);
						return ERROR;
					}
				}
			} else {
				if (raydium_i2c_pda_read(g_raydium_ts->client, u32_addr, p_u8_output_buf, u8_read_len) == ERROR) {
					DEBUGOUT("%s\r\n", __func__);
					return ERROR;
				}
			}
		}
	}

	return SUCCESS;
}

/******************************************************************************
 ** Function name:handle_ic_write
 **
 ** Descriptions:handle write data to ic
 **
 ** parameters:u32_addr,Address
 **	       u8_write_len,datalength
 **	       bValue,Data
 **	       u8_interface,SPI or I2C
 **	       u8_trans_modePDA2_MODE, PDA_WORD_MODE, PDA_BYTE_MODE, MCU_MODE
 **
 ** Returned value:ERROR,SUCCESS
 **
 ******************************************************************************/
unsigned char handle_ic_write(
	unsigned int u32_addr,
	unsigned char u8_write_len,
	unsigned char *bValue,
	unsigned char u8_interface,
	unsigned char u8_trans_mode)
{
	if (u8_trans_mode == I2C_PDA2_MODE) {
		/*PDA2 MODE*/
		if (raydium_i2c_pda2_write(g_raydium_ts->client, (unsigned char)u32_addr, bValue, u8_write_len) == ERROR) {
			DEBUGOUT("%s\r\n", __func__);
			return ERROR;
		}
	} else {
		/*PDA MODE*/
		if ((u8_trans_mode == I2C_WORD_MODE) && (u32_addr & 0x00000003)) {
			DEBUGOUT("[I2CRW] Handle Write Word ADDR Not Word Align!!\r\n");
			return ERROR;
		}

		if (u8_interface == SPI_INTERFACE) {
			switch (u8_trans_mode) {
			case I2C_BYTE_MODE:
				u8_trans_mode = SPI_BYTE_MODE;
				break;
			case I2C_WORD_MODE:
				u8_trans_mode = SPI_WORD_MODE;
				break;
			/*			case I2C_MCU_MODE:
			 *
			 *				u8_trans_mode = SPI_MCU_MODE;
			 *
			 *				break;
			 */
			case SPI_BYTE_MODE:
			case SPI_WORD_MODE:
				break;
			default:
				DEBUGOUT("%s%d\r\n", __func__, u8_trans_mode);
				return ERROR;
			}

			if (spi_write_pda(u32_addr, u8_write_len, bValue, u8_trans_mode) == ERROR) {
				DEBUGOUT("%s\r\n", __func__);
				return ERROR;
			}
		} else {
			if (g_u16_dev_id == DEVICE_ID_3X) {
				if ((u8_interface & I2C_PDA_MODE) != 0)  {
					if (raydium_i2c_pda_write(g_raydium_ts->client, u32_addr, bValue, u8_write_len) == ERROR) {
						DEBUGOUT("%s\r\n", __func__);
						return ERROR;
					}
				} else {
					if (raydium_i2c_write_pda_via_pda2(g_raydium_ts->client, u32_addr, bValue, u8_write_len) == ERROR) {
						DEBUGOUT("%s\r\n", __func__);
						return ERROR;
					}
				}
			} else {
				if (raydium_i2c_pda_write(g_raydium_ts->client, u32_addr, bValue, u8_write_len) == ERROR) {
					DEBUGOUT("%s\r\n", __func__);
					return ERROR;
				}
			}
		}
	}

	return SUCCESS;
}

/******************************************************************************
 ** Function name:handle_display_write
 **
 ** Descriptions:handle write data to display
 **
 ** parameters:p_u8_data,Data
 **	       u16DataLength,datalength
 **
 **
 ** Returned value:ERROR,SUCCESS
 **
 ******************************************************************************/
unsigned char handle_display_write(
	unsigned char *p_u8_data,
	unsigned short u16DataLength)
{

	if (WriteDriverByTouchMode(p_u8_data, u16DataLength) == ERROR) {
		DEBUGOUT("[HDW] WriteDriverByTouchMode NG!\r\n");
		return ERROR;
	}

	return SUCCESS;
}
