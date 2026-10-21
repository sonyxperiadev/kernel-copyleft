/* raydium_fw_update.c
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

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <asm/traps.h>
#include <linux/firmware.h>
#include "raydium_driver.h"
#if !ENABLE_FW_LOADER
#include "rad_fw_image_30.h"
#endif
#if defined(FW_MAPPING_EN)
#include "rad_fw_image_31.h"
#endif

void raydium_mem_table_init(unsigned short u16_id)
{
	LOGD(LOG_INFO, "[touch]Raydium table init 0x%x\n", u16_id);

	g_rad_boot_image = kzalloc(RAD_BOOT_3X_SIZE, GFP_KERNEL);
	g_rad_init_image = kzalloc(RAD_INIT_3X_SIZE, GFP_KERNEL);
	g_rad_fw_image = kzalloc(RAD_FW_3X_SIZE, GFP_KERNEL);
	g_rad_para_image = kzalloc(RAD_PARA_3X_SIZE + 4, GFP_KERNEL);
	g_rad_testfw_image = kzalloc(RAD_TESTFW_3X_SIZE, GFP_KERNEL);
	g_rad_testpara_image = kzalloc(RAD_PARA_3X_SIZE + 4,
				       GFP_KERNEL);
	g_u8_table_init = SUCCESS;
}
#if ENABLE_FW_LOADER
static void raydium_cb(const struct firmware *fw, void *ctx)
{
	unsigned int u32_offset = 0;
	unsigned int u32_image_version;
#ifdef FW_UPDATE_EN
	int i32_ret = ERROR;
#endif
	if (fw && (fw->size == RAD_ALLFW_3X_SIZE)) {
		LOGD(LOG_DEBUG, "[touch]get firmware success size:%x\n", fw->size);

		memcpy(g_rad_boot_image, fw->data, RAD_BOOT_3X_SIZE);
		u32_offset += RAD_BOOT_3X_SIZE;
		memcpy(g_rad_init_image, fw->data + u32_offset, RAD_INIT_3X_SIZE);
		u32_offset += RAD_INIT_3X_SIZE;
		memcpy(g_rad_fw_image, fw->data + u32_offset, RAD_FW_3X_SIZE);
		u32_offset += RAD_FW_3X_SIZE;
		memcpy(g_rad_para_image, fw->data + u32_offset, RAD_PARA_3X_SIZE + 4);
		u32_offset += RAD_PARA_3X_SIZE + 4;
		memcpy(g_rad_testfw_image, fw->data + u32_offset, RAD_FW_3X_SIZE);
		u32_offset += RAD_FW_3X_SIZE;
		memcpy(g_rad_testpara_image, fw->data + u32_offset, RAD_PARA_3X_SIZE + 4);

		memcpy(g_rad_testfw_image + RAD_FW_3X_SIZE, g_rad_testpara_image
		       , RAD_PARA_3X_SIZE + 4);
		u32_image_version = (g_rad_para_image[PARA_FW_VERSION_OFFSET] << 24) |
				    (g_rad_para_image[PARA_FW_VERSION_OFFSET + 1] << 16) |
				    (g_rad_para_image[PARA_FW_VERSION_OFFSET + 2] << 8) |
				    g_rad_para_image[PARA_FW_VERSION_OFFSET + 3];

		LOGD(LOG_INFO, "[touch]RAD Image FW ver : 0x%x\n", u32_image_version);
#ifdef FW_UPDATE_EN
		i32_ret = raydium_fw_update_check(u32_image_version);
		if (i32_ret < 0)
			LOGD(LOG_ERR, "[touch]fw update fail!\n");
#endif
	} else {
		LOGD(LOG_ERR, "[touch]get firmware file fail!\n");
	}
}
int raydium_load_image(unsigned char *u8_buf, char *name)
{
	int i32_ret = SUCCESS;
	struct i2c_client *client = g_raydium_ts->client;

	i32_ret = request_firmware_nowait(THIS_MODULE, true, FW_NAME, &client->dev,
					GFP_KERNEL, g_raydium_ts, raydium_cb);

	if (i32_ret) {
		LOGD(LOG_ERR, "[touch]failed to get firmware %s %d\n",
					name, i32_ret);
		return i32_ret;
	}

	return i32_ret;
}
int raydium_mem_table_setting(void)
{
	int i32_ret = SUCCESS;
	char name[RAYDIUM_FW_BIN_PATH_LENGTH];

	snprintf((char *)name, RAYDIUM_FW_BIN_PATH_LENGTH, "%s", FW_NAME);
	LOGD(LOG_DEBUG, "[touch]firmware path %s\n", name);
	i32_ret = raydium_load_image(g_rad_fw_image, name);

	if (i32_ret < 0)
		return ERROR;

	i32_ret = SUCCESS;
	return i32_ret;
}

#else
int raydium_mem_table_setting(void)
{
	int i32_ret = SUCCESS;

	LOGD(LOG_INFO, "[touch]Raydium ID is 0x%x\n", g_raydium_ts->id);
	switch (g_raydium_ts->id) {
	case RAD_30:
		memcpy(g_rad_boot_image, u8_rad_boot_30, RAD_BOOT_3X_SIZE);
		memcpy(g_rad_init_image, u8_rad_init_30, RAD_INIT_3X_SIZE);
		memcpy(g_rad_fw_image, u8_rad_fw_30, RAD_FW_3X_SIZE);
		memcpy(g_rad_testfw_image, u8_rad_testfw_30, RAD_FW_3X_SIZE);
		memcpy(g_rad_testfw_image + RAD_FW_3X_SIZE, u8_rad_testpara_30
		       , RAD_PARA_3X_SIZE + 4);
		memcpy(g_rad_para_image, u8_rad_para_30, RAD_PARA_3X_SIZE + 4);
		memcpy(g_rad_testpara_image, u8_rad_testpara_30,
		       RAD_PARA_3X_SIZE + 4);
		break;
#if defined(FW_MAPPING_EN)
	case RAD_31:
		memcpy(g_rad_boot_image, u8_rad_boot_31, RAD_BOOT_3X_SIZE);
		memcpy(g_rad_init_image, u8_rad_init_31, RAD_INIT_3X_SIZE);
		memcpy(g_rad_fw_image, u8_rad_fw_31, RAD_FW_3X_SIZE);
		memcpy(g_rad_testfw_image, u8_rad_testfw_31, RAD_FW_3X_SIZE);
		memcpy(g_rad_testfw_image + RAD_FW_3X_SIZE, u8_rad_testpara_31,
		       RAD_PARA_3X_SIZE + 4);
		memcpy(g_rad_para_image, u8_rad_para_31, RAD_PARA_3X_SIZE + 4);
		memcpy(g_rad_testpara_image, u8_rad_testpara_31,
		       RAD_PARA_3X_SIZE + 4);
		break;
#endif
	default:
		LOGD(LOG_DEBUG, "[touch]mapping ic setting use default fw\n");
		memcpy(g_rad_boot_image, u8_rad_boot_30, RAD_BOOT_3X_SIZE);
		memcpy(g_rad_init_image, u8_rad_init_30, RAD_INIT_3X_SIZE);
		memcpy(g_rad_fw_image, u8_rad_fw_30, RAD_FW_3X_SIZE);
		memcpy(g_rad_testfw_image, u8_rad_testfw_30, RAD_FW_3X_SIZE);
		memcpy(g_rad_testfw_image + RAD_FW_3X_SIZE, u8_rad_testpara_30
		       , RAD_PARA_3X_SIZE + 4);
		memcpy(g_rad_para_image, u8_rad_para_30, RAD_PARA_3X_SIZE + 4);
		memcpy(g_rad_testpara_image, u8_rad_testpara_30,
		       RAD_PARA_3X_SIZE + 4);
		g_raydium_ts->id = RAD_30;
		i32_ret = SUCCESS;
		break;
	}

	g_u8_table_setting = 0;
	return i32_ret;
}
#endif

#if !ENABLE_FW_LOADER
int raydium_id_init(unsigned char u8_type)
{
	int i32_ret = ERROR;

	i32_ret = 0;
	switch (u8_type) {
	case 0:
		g_raydium_ts->id = RAD_30;
		i32_ret = SUCCESS;
		break;
#if defined(FW_MAPPING_EN)
	case 1:
		g_raydium_ts->id = RAD_31;
		i32_ret = SUCCESS;
		break;
#endif
	}
	return i32_ret;
}
#endif
unsigned int bits_reverse(unsigned int u32_num, unsigned int bit_num)
{
	unsigned int reverse = 0, u32_i;

	for (u32_i = 0; u32_i < bit_num; u32_i++) {
		if (u32_num & (1 << u32_i))
			reverse |= 1 << ((bit_num - 1) - u32_i);
	}
	return reverse;
}

unsigned int rc_crc32(const char *buf, unsigned int u32_len,
		      unsigned int u32_crc)
{
	unsigned int u32_i;
	unsigned char u8_flash_byte, u8_current, u8_j;

	for (u32_i = 0; u32_i < u32_len; u32_i++) {
		u8_flash_byte = buf[u32_i];
		u8_current = (unsigned char)bits_reverse(u8_flash_byte, 8);
		for (u8_j = 0; u8_j < 8; u8_j++) {
			if ((u32_crc ^ u8_current) & 0x01)
				u32_crc = (u32_crc >> 1) ^ 0xedb88320;
			else
				u32_crc >>= 1;
			u8_current >>= 1;
		}
	}
	return u32_crc;
}

int wait_fw_state(struct i2c_client *client, unsigned int u32_addr,
		  unsigned int u32_state, unsigned long u32_delay_us,
		  unsigned short u16_retry)
{
	unsigned char u8_buf[4];
	unsigned int u32_read_data;
	unsigned int u32_min_delay_us = u32_delay_us - 500;
	unsigned int u32_max_delay_us = u32_delay_us + 500;

	do {
		if (handle_i2c_pda_read(client, u32_addr, u8_buf, 4) == ERROR)
			return ERROR;

		memcpy(&u32_read_data, u8_buf, 4);
		u16_retry--;
		usleep_range(u32_min_delay_us, u32_max_delay_us);
	} while ((u32_read_data != u32_state) && (u16_retry != 0));

	if (u32_read_data != u32_state) {
		LOGD(LOG_ERR, "[touch]confirm data error : 0x%x\n", u32_read_data);
		return ERROR;
	}

	return SUCCESS;
}

int wait_irq_state(struct i2c_client *client, unsigned int retry_time,
		   unsigned int u32_delay_us)
{
	int i32_ret = SUCCESS;
	unsigned int u32_retry;
	unsigned int u32_irq_value;
	unsigned int u32_min_delay_us = u32_delay_us - 500;
	unsigned int u32_max_delay_us = u32_delay_us + 500;

	u32_retry = retry_time;
	u32_irq_value = 0;
	while (u32_retry != 0 && u32_irq_value != 1) {
		u32_irq_value = gpio_get_value(g_raydium_ts->irq_gpio);
		usleep_range(u32_min_delay_us, u32_max_delay_us);
		u32_retry--;
	}
	LOGD(LOG_DEBUG, "[touch]irq_value is %d\n", u32_irq_value);

	if (u32_retry == 0) {
		LOGD(LOG_ERR, "[touch]%s, FW not ready, retry error!\n", __func__);
		i32_ret = ERROR;
	}

	return i32_ret;
}

int raydium_do_software_reset(struct i2c_client *client)
{
	int i32_ret = SUCCESS;

	unsigned char u8_buf[4];

	/*SW reset*/
	g_u8_resetflag = true;
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	LOGD(LOG_INFO, "[touch]SW reset\n");
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BLKRST, u8_buf, 4);
	if (i32_ret < 0)
		goto exit;

	msleep(35);
	if (raydium_disable_i2c_deglitch() == ERROR) {
		LOGD(LOG_ERR, "[touch] 1.disable_i2c_deglitch_3x NG!\r\n");
		goto exit;
	}
exit:
	return i32_ret;
}

int set_skip_load(struct i2c_client *client)
{
	int i32_ret = SUCCESS;
	unsigned int u32_retry_time = 1000;
	unsigned char u8_buf[4];

	/*Skip load*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x10;
	u8_buf[1] = 0x08;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTREG, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	i32_ret = raydium_do_software_reset(client);
	if (i32_ret < 0)
		LOGD(LOG_ERR, "[touch]%s, SW reset error!\n", __func__);
	msleep(35);

	i32_ret = wait_fw_state(client, RAYDIUM_PDA_BOOTSTATE, 0x82, 2000, u32_retry_time);
	if (i32_ret < 0)
		LOGD(LOG_ERR, "[touch]%s, wait_fw_state error!\n", __func__);
exit_upgrade:
	return i32_ret;
}

/*check pram crc32*/
static int raydium_check_pram_crc_3X(struct i2c_client *client,
				     unsigned int u32_addr,
				     unsigned int u32_len)
{
	int i32_ret = SUCCESS;
	unsigned int u32_crc_addr = u32_addr + u32_len;
	unsigned int u32_end_addr = u32_crc_addr - 1;
	unsigned int u32_crc_result, u32_read_data;
	unsigned int u32_retry = 1000;
	unsigned char u8_buf[4], u8_retry = 3;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = (unsigned char)(u32_addr & 0xFF);
	u8_buf[1] = (unsigned char)((u32_addr & 0xFF00) >> 8);
	u8_buf[2] = (unsigned char)(u32_end_addr & 0xFF);
	u8_buf[3] = (unsigned char)((u32_end_addr & 0xFF00) >> 8);

	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_PRGCHKSUMADDR, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	i32_ret = handle_i2c_pda_read(client, RAYDIUM_PDA_PRGCHKSUMENG, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	u8_buf[3] |= 0x81;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_PRGCHKSUMENG, u8_buf, 4);

	while (u8_buf[3] != 0x80 && u32_retry != 0) {
		i32_ret = handle_i2c_pda_read(client, RAYDIUM_PDA_PRGCHKSUMENG, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		u32_retry--;
		usleep_range(4500, 5500);
	}
	if (u32_retry == 0) {
		LOGD(LOG_ERR, "[touch]%s, Cal CRC not ready, retry error!\n",
		     __func__);
		i32_ret = ERROR;
	}

	i32_ret = handle_i2c_pda_read(client, RAYDIUM_PDA_PRGCHKSUMRESULT, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	memcpy(&u32_crc_result, u8_buf, 4);
	i32_ret = handle_i2c_pda_read(client, u32_crc_addr, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;
	memcpy(&u32_read_data, u8_buf, 4);

	while (u32_read_data != u32_crc_result && u8_retry > 0) {
		i32_ret = handle_i2c_pda_read(client, RAYDIUM_PDA_PRGCHKSUMRESULT, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		memcpy(&u32_crc_result, u8_buf, 4);
		usleep_range(1500, 2500);
		u8_retry--;
	}
	if (u32_read_data != u32_crc_result) {
		LOGD(LOG_ERR, "[touch]check pram crc fail!!\n");
		LOGD(LOG_ERR, "[touch]u32_read_data 0x%x\n", u32_read_data);
		LOGD(LOG_ERR, "[touch]u32_crc_result 0x%x\n", u32_crc_result);
		i32_ret = ERROR;
		goto exit_upgrade;
	} else if (u8_retry != 3) {
		LOGD(LOG_DEBUG, "[touch]check pram crc pass!!\n");
		LOGD(LOG_DEBUG, "[touch]u8_retry : %d\n", u8_retry);
		LOGD(LOG_DEBUG, "[touch]u32_read_data 0x%x\n", u32_read_data);
		LOGD(LOG_DEBUG, "[touch]u32_crc_result 0x%x\n", u32_crc_result);
		i32_ret = ERROR;
		goto exit_upgrade;
	} else {
		LOGD(LOG_DEBUG, "[touch]check pram crc pass!!\n");
		LOGD(LOG_DEBUG, "[touch]u8_retry : %d\n", u8_retry);
		LOGD(LOG_DEBUG, "[touch]u32_read_data 0x%x\n", u32_read_data);
		LOGD(LOG_DEBUG, "[touch]u32_crc_result 0x%x\n", u32_crc_result);
		i32_ret = SUCCESS;
	}

exit_upgrade:
	return i32_ret;
}

/* upgrade firmware with image file */
static int raydium_write_to_pram_3X(struct i2c_client *client,
				    unsigned int u32_fw_addr,
				    unsigned char u8_type)
{
	int i32_ret = ERROR;
	unsigned int u32_fw_size = 0;
	unsigned char *p_u8_firmware_data = NULL;
	unsigned int u32_write_offset = 0;
	unsigned short u16_write_length = 0;

	switch (u8_type) {
	case RAYDIUM_INIT:
		u32_fw_size = 0x80;
		p_u8_firmware_data = g_rad_init_image;
		break;
	case RAYDIUM_BOOTLOADER:
		u32_fw_size = 0x800;
		p_u8_firmware_data = g_rad_boot_image;
		break;

	case RAYDIUM_PARA:
		u32_fw_size = RAYDIUM_PDA_PARALENGTH;
		p_u8_firmware_data = g_rad_para_image;
		break;

	case RAYDIUM_FIRMWARE:
		u32_fw_size = RAYDIUM_PDA_FIRMWARELENGTH;
		p_u8_firmware_data = g_rad_fw_image;
		break;
	case RAYDIUM_TEST_FW:
		u32_fw_size = RAYDIUM_PDA_FIRMWARELENGTH + RAYDIUM_PDA_PARALENGTH;
		p_u8_firmware_data = g_rad_testfw_image;
		break;

	default:
		goto exit_upgrate;
	}

	u32_write_offset = 0;
	while (u32_write_offset < u32_fw_size) {
		if ((u32_write_offset + MAX_WRITE_PACKET_SIZE) < u32_fw_size)
			u16_write_length = MAX_WRITE_PACKET_SIZE;
		else
			u16_write_length =
				(unsigned short)(u32_fw_size - u32_write_offset);

		i32_ret = handle_i2c_pda_write(
				  client,
				  (u32_fw_addr + u32_write_offset),
				  (p_u8_firmware_data + u32_write_offset),
				  u16_write_length);
		if (i32_ret < 0)
			goto exit_upgrate;

		u32_write_offset += (unsigned long)u16_write_length;
	}
	u32_fw_addr += u32_write_offset;

exit_upgrate:
	if (i32_ret < 0) {
		LOGD(LOG_ERR, "[touch]upgrade failed\n");
		return i32_ret;
	}
	LOGD(LOG_INFO, "[touch]upgrade success\n");
	return SUCCESS;
}

unsigned char raydium_stop_mcu_3x(unsigned char u8_is_tp_reset)
{
	unsigned short u16_time_out = 100;
	unsigned int u32_read_data;
	unsigned int u32_write = 0;

	if (u8_is_tp_reset) {
		gpio_set_value(g_raydium_ts->rst_gpio, 1);
		gpio_set_value(g_raydium_ts->rst_gpio, 0);
		msleep(RAYDIUM_RESET_INTERVAL_MSEC);
		gpio_set_value(g_raydium_ts->rst_gpio, 1);

		g_u8_i2c_mode = PDA2_MODE;
		msleep(35);
		if (raydium_disable_i2c_deglitch() == ERROR) {
			LOGD(LOG_ERR, "[touch] disable_i2c_deglitch_3x NG!\r\n");
			return ERROR;
		}
	}
	LOGD(LOG_INFO, "[touch]%s\r\n", __func__);

	u32_write = 0x30;
	if (handle_i2c_pda_write(g_raydium_ts->client, RAYDIUM_PDA_BOOTREG
				 , (unsigned char *)&u32_write, 4) == ERROR) {
		return ERROR;
	}

	u32_write = 0x01;
	if (handle_i2c_pda_write(g_raydium_ts->client, RAYDIUM_PDA_BLKRST, (unsigned char *)&u32_write, 4
				) == ERROR) {
		return ERROR;
	}
	msleep(100);

	if (raydium_disable_i2c_deglitch() == ERROR) {
		LOGD(LOG_ERR, "[touch] disable_i2c_deglitch_3x NG!\r\n");
		return ERROR;
	}
	LOGD(LOG_INFO, "[touch] stop_mcu 2\r\n");

	if (handle_i2c_pda_read(g_raydium_ts->client, RAYDIUM_PDA_BOOTREG
				, (unsigned char *)(&u32_read_data), 4) == ERROR) {
		return ERROR;
	}

	while ((u32_read_data & 0x2000) == 0 && u16_time_out-- > 0) {
		msleep(20);
		if (handle_i2c_pda_read(g_raydium_ts->client, RAYDIUM_PDA_BOOTREG
					, (unsigned char *)(&u32_read_data), 4) == ERROR) {
			return ERROR;
		}
	}

	LOGD(LOG_DEBUG, "[touch]Stop MCU=0x%X(0x%x)(%d)!!\r\n", u32_read_data, (u32_read_data & 0x2000), u16_time_out);

	if ((u32_read_data & 0x2000) == 0)
		return ERROR;

	return SUCCESS;
}

/* Raydium fireware upgrade flow */
static int raydium_fw_upgrade_3X(struct i2c_client *client,
				 unsigned char u8_type,
				 unsigned char u8_check_crc)
{
	int i32_ret = 0;
	unsigned char u8_buf[4];
	unsigned short u16_retry = 1000;
	unsigned int u32_write = 0;

	/*##### wait for boot-loader start #####*/
	LOGD(LOG_INFO, "[touch]Type is %x\n", u8_type);

	/*read Boot version*/
	if (handle_i2c_pda_read(client, RAYDIUM_PDA_BOOTVERSION, u8_buf, 4) == ERROR)
		return ERROR;
	u32_write = (u8_buf[3] << 24) | (u8_buf[2] << 16) | (u8_buf[1] << 8) | u8_buf[0];
	LOGD(LOG_INFO, "[touch]bootloader ver: 0x%x\r\n", u32_write);

	if (u8_type != RAYDIUM_COMP) {
		if (raydium_stop_mcu_3x(0) == ERROR)
			return ERROR;
	}

	/*#start write data to PRAM*/
	if (u8_type == RAYDIUM_FIRMWARE) {
		/* unlock PRAM */
		u8_buf[0] = (BOTLR_LOCK | COMP_LOCK | BASEL_LOCK | INICO_LOCK);
		i32_ret = handle_i2c_pda_write(client,
					       RAYDIUM_PDA_PRAMLOCK,
					       u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_write_to_pram_3X(client, RAYDIUM_PDA_FIRMWAREADDR,
						   RAYDIUM_FIRMWARE);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_write_to_pram_3X(client, RAYDIUM_PDA_PARAADDR,
						   RAYDIUM_PARA);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_check_pram_crc_3X(client, RAYDIUM_PDA_FIRMWAREADDR,
						    RAYDIUM_PDA_CRCLENGTH);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_BOOTLOADER) {
		/* unlock PRAM */
		u8_buf[0] = (BOTLR_LOCK | CONFIG_LOCK | COMP_LOCK | BASEL_LOCK | INICO_LOCK);

		i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_PRAMLOCK,
					       u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_write_to_pram_3X(client, 0x0800,
						   RAYDIUM_BOOTLOADER);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_write_to_pram_3X(client, 0x1000,
						   RAYDIUM_INIT);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_check_pram_crc_3X(client, 0x800,
						    0x7FC);
		if (i32_ret < 0)
			goto exit_upgrade;
		i32_ret = raydium_check_pram_crc_3X(client, 0x1000,
						    0x7C);
		if (i32_ret < 0)
			goto exit_upgrade;
	}

	if (u8_type != RAYDIUM_COMP) {
		/*release mcu hold*/
		/*Skip load*/
		i32_ret = set_skip_load(client);
		if (i32_ret < 0)
			LOGD(LOG_ERR, "[touch]%s, set skip_load error!\n", __func__);
		/*#confirm in burn mode*/
		if (wait_fw_state(client, RAYDIUM_PDA_BOOTSTATE, 0x82,
				  2000, u16_retry) == ERROR) {
			LOGD(LOG_ERR, "[touch]Error, confirm in burn mode 1\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}

	}

	/*#setting burning mode*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTENG1, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTENG2, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTENG3, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x01;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTMODE, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#confirm in burn mode*/
	if (wait_fw_state(client, RAYDIUM_PDA_BOOTSTATE, 0xFF,
			  2000, u16_retry) == ERROR) {
		LOGD(LOG_ERR, "[touch]bootloader wrt empty flash fail\r\n");
		LOGD(LOG_ERR, "[touch]bootloader confirm in burn mode fail, Type=%d\r\n", u8_type);
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/* burning setting */
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x10;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTREG, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = u8_type;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_PRAMTYPE, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#set PRAM length (at 'h5000_090C)*/
	if (u8_type == RAYDIUM_COMP) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x78;
		u8_buf[1] = 0x7c;
		i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_PRAMADDR,
					       u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_FIRMWARE) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x08;
		i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_PRAMADDR,
					       u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

	} else if (u8_type == RAYDIUM_BOOTLOADER) {
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x00;
		u8_buf[1] = 0x08;
		i32_ret = handle_i2c_pda_write(client, 0x50000908,
					       u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;
	}


	/*#set sync_data(RAYDIUM_PDA_SYNCDATA) = 0 as WRT data finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_SYNCDATA, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#wait for sync_data finish*/
	if (wait_fw_state(client, RAYDIUM_PDA_BOOTENG4, 168, 1000,
			  u16_retry) == ERROR) {
		LOGD(LOG_ERR, "[touch]Error, wait for input unlock key\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}

	/*#flash unlock key*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xd7;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_FLKEY2, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa5;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_FLKEY1, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_FLKEY1, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa5;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_FLKEY1, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_FLKEY2, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_FLASHPRO, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#ready to burn flash*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0xa8;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTSTATE, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*#clr sync_data(RAYDIUM_PDA_SYNCDATA) = 0 as finish*/
	memset(u8_buf, 0, sizeof(u8_buf));
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_SYNCDATA, u8_buf, 4);
	if (i32_ret < 0)
		goto exit_upgrade;

	/* wait erase/wrt finish
	 * confirm burning_state result (gu8I2CSyncData.burning_state =
	 * BURNING_WRT_FLASH_FINISH at RAYDIUM_PDA_BOOTENG3)
	 */
	if (wait_fw_state(client, RAYDIUM_PDA_BOOTENG3, 6, 8000,
			  u16_retry) == ERROR) {
		LOGD(LOG_ERR, "[touch]Error, wait erase/wrt finish\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}
	LOGD(LOG_INFO, "[touch]Burn flash ok\n");

	if (u8_check_crc) {
		memset(u8_buf, 0, sizeof(u8_buf));
		i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_SYNCDATA,
					       u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		/*#wait software reset finish*/
		msleep(35);

		/* wait sw reset finished RAYDIUM_PDA_BOOTSTATE = 0x82 */
		if (wait_fw_state(client, RAYDIUM_PDA_BOOTSTATE, 0x82, 2000,
				  u16_retry) == ERROR) {
			LOGD(LOG_ERR, "[touch]Error, wait sw reset finished\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}

		/*#set test_mode = 1 start to check crc*/
		memset(u8_buf, 0, sizeof(u8_buf));
		u8_buf[0] = 0x01;
		i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTMODE,
					       u8_buf, 4);
		if (i32_ret < 0)
			goto exit_upgrade;

		/*#wait crc check finish*/
		if (wait_fw_state(client, RAYDIUM_PDA_BOOTENG2, 2,
				  2000, u16_retry)
		    == ERROR) {
			LOGD(LOG_ERR, "[touch]Error, wait crc check finish\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}

		/*#crc check pass RAYDIUM_PDA_BOOTSTATE = 0x81*/
		if (wait_fw_state(client, RAYDIUM_PDA_BOOTSTATE, 0x81,
				  2000, u16_retry)
		    == ERROR) {
			LOGD(LOG_ERR, "[touch]Error, confirm crc result\n");
			i32_ret = ERROR;
			goto exit_upgrade;
		}
		/*#run to next step*/
		LOGD(LOG_INFO, "[touch]Type 0x%x => Pass\n", u8_type);

		/* sw rest */

		i32_ret = raydium_do_software_reset(client);
		if (i32_ret < 0)
			goto exit_upgrade;

		g_u8_i2c_mode = PDA2_MODE;


		LOGD(LOG_INFO, "[touch]Burn FW finish!\n");
	}

exit_upgrade:
	return i32_ret;
}

static int raydium_boot_upgrade_3X(struct i2c_client *client, unsigned char u8_is_tp_reset)
{
	int i32_ret = SUCCESS;
	unsigned char u8_buf[4];
	unsigned short u16_retry = 1000;

	/*set mcu hold*/
	memset(u8_buf, 0, sizeof(u8_buf));
	if (raydium_stop_mcu_3x(u8_is_tp_reset) == ERROR)
		return ERROR;
	/*WRT boot-loader to PRAM first*/
	memset(u8_buf, 0, sizeof(u8_buf));
	u8_buf[0] = 0x1F;
	i32_ret = handle_i2c_pda_write(client, RAYDIUM_PDA_PRAMLOCK, u8_buf, 4);

	/*Sending bootloader*/
	i32_ret = raydium_write_to_pram_3X(client, 0x0000,
					   RAYDIUM_BOOTLOADER);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = raydium_check_pram_crc_3X(client, 0x000, 0x7FC);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*Sending initial code*/
	i32_ret = raydium_write_to_pram_3X(client, 0x7f80,
					   RAYDIUM_INIT);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = raydium_check_pram_crc_3X(client, 0x7F80, 0x7C);
	if (i32_ret < 0)
		goto exit_upgrade;

	/*release mcu hold*/
	/*Skip load*/
	i32_ret = set_skip_load(client);
	if (i32_ret < 0)
		LOGD(LOG_ERR, "[touch]%s, set skip_load error!\n", __func__);
	msleep(35);
	if (wait_fw_state(client, RAYDIUM_PDA_BOOTSTATE, 0x82, 1000,
			  u16_retry) == ERROR) {
		LOGD(LOG_ERR, "[touch]Error, wait for input unlock key\n");
		i32_ret = ERROR;
		goto exit_upgrade;
	}
exit_upgrade:
	return i32_ret;
}

int raydium_fw_update_check(unsigned int u32_check_version)
{
	int i32_ret = ERROR;
	unsigned int u32_fw_version;
	unsigned char u8_rbuffer[4];

	if (g_raydium_ts->fw_version != u32_check_version) {

		LOGD(LOG_INFO, "[touch]FW need update.\n");
		g_u8_raydium_flag |= ENG_MODE;

		i32_ret = raydium_burn_fw(g_raydium_ts->client);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]FW update fail:%d\n", i32_ret);
			goto exit_error;
		}
		g_u8_raydium_flag &= ~ENG_MODE;

		i32_ret = raydium_i2c_pda2_set_page(g_raydium_ts->client,
						    g_raydium_ts->is_suspend,
						    RAYDIUM_PDA2_PAGE_0);
		if (i32_ret < 0)
			goto exit_error;

		i32_ret = raydium_i2c_pda2_read(g_raydium_ts->client,
						RAYDIUM_PDA2_FW_VERSION_ADDR,
						u8_rbuffer,
						4);
		if (i32_ret < 0)
			goto exit_error;

		u32_fw_version = (u8_rbuffer[0] << 24) |
				 (u8_rbuffer[1] << 16) |
				 (u8_rbuffer[2] << 8) |
				 u8_rbuffer[3];
		LOGD(LOG_INFO, "[touch]RAD FW ver is 0x%x\n",
		     u32_fw_version);
		g_raydium_ts->fw_version = u32_fw_version;
	} else
		LOGD(LOG_INFO, "[touch]FW is the latest version.\n");

	i32_ret = SUCCESS;
	return i32_ret;

exit_error:
	return i32_ret;
}

int raydium_burn_fw(struct i2c_client *client)
{
	int i32_ret = ERROR;

	g_u8_resetflag = true;
	if ((g_raydium_ts->id & 0x3000) != 0) {
		LOGD(LOG_INFO, "[touch]start burn function!\n");
		i32_ret = raydium_boot_upgrade_3X(client, 1);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_fw_upgrade_3X(client, RAYDIUM_BOOTLOADER, 0);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_ret = raydium_fw_upgrade_3X(client, RAYDIUM_FIRMWARE, 1);
		if (i32_ret < 0)
			goto exit_upgrade;
	} else {
		LOGD(LOG_ERR, "[touch]FW ID ERROR!\n");
	}

exit_upgrade:
	return i32_ret;
}

int raydium_fw_update_init(unsigned short u16_i2c_data)
{

	unsigned char u8_rbuffer[4];

	unsigned int u32_fw_version;
#if !ENABLE_FW_LOADER
	unsigned int u32_image_version;
#endif
	int i32_ret = ERROR;

	mutex_lock(&g_raydium_ts->lock);
	i32_ret = raydium_i2c_pda2_set_page(g_raydium_ts->client,
					    g_raydium_ts->is_suspend,
					    RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0)
		goto exit_error;

	i32_ret = raydium_i2c_pda2_read(g_raydium_ts->client,
					RAYDIUM_PDA2_FW_VERSION_ADDR,
					u8_rbuffer,
					4);
	if (i32_ret < 0)
		goto exit_error;

	mutex_unlock(&g_raydium_ts->lock);

	u32_fw_version = (u8_rbuffer[0] << 24) | (u8_rbuffer[1] << 16) |
			 (u8_rbuffer[2] << 8) | u8_rbuffer[3];
	LOGD(LOG_INFO, "[touch]RAD FW ver 0x%.8x\n", u32_fw_version);

	g_raydium_ts->fw_version = u32_fw_version;

	g_raydium_ts->id = ((u16_i2c_data & 0xF) << 12) |
			   ((u8_rbuffer[0] & 0xF) << 8) | u8_rbuffer[1];

	raydium_mem_table_init(g_raydium_ts->id);
	if (raydium_mem_table_setting() == SUCCESS) {
#if !ENABLE_FW_LOADER

		u32_image_version = (g_rad_para_image[PARA_FW_VERSION_OFFSET] << 24) |
				    (g_rad_para_image[PARA_FW_VERSION_OFFSET + 1] << 16) |
				    (g_rad_para_image[PARA_FW_VERSION_OFFSET + 2] << 8) |
				    g_rad_para_image[PARA_FW_VERSION_OFFSET + 3];

		LOGD(LOG_INFO, "[touch]RAD Image FW ver : 0x%x\n", u32_image_version);
#endif
	} else {
		LOGD(LOG_ERR, "[touch]Mem setting failed, Stop fw upgrade!\n");
		return i32_ret;
	}

#ifdef FW_UPDATE_EN

#if !ENABLE_FW_LOADER
	i32_ret = raydium_fw_update_check(u32_image_version);
	if (i32_ret < 0)
		LOGD(LOG_ERR, "[touch]fw update fail!\n");
#endif

#endif


	return i32_ret;

exit_error:
	mutex_unlock(&g_raydium_ts->lock);
	return i32_ret;
}
int raydium_burn_comp(struct i2c_client *client)
{
	int i32_ret = ERROR;

	i32_ret = raydium_boot_upgrade_3X(client, 0);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = set_skip_load(client);
	if (i32_ret < 0)
		goto exit_upgrade;


	i32_ret = raydium_fw_upgrade_3X(client, RAYDIUM_COMP, 1);
	if (i32_ret < 0)
		goto exit_upgrade;

	i32_ret = SUCCESS;

exit_upgrade:
	return i32_ret;
}

int raydium_check_fw_ready(struct i2c_client *client)
{
	int i32_ret = SUCCESS;
	unsigned int u32_retry = 400;
	unsigned char u8_buf[4];

	u8_buf[1] = 0;
	while (u8_buf[1] != 0x40 && u32_retry != 0) {
		i32_ret = handle_i2c_pda_read(client, RAYDIUM_PDA_BOOTREG, u8_buf, 4);
		if (i32_ret < 0)
			goto exit;

		u32_retry--;
		usleep_range(4500, 5500);
	}

	if (u32_retry == 0) {
		LOGD(LOG_ERR, "[touch]%s, FW not ready, retry error!\n", __func__);
		i32_ret = ERROR;
	} else {
		LOGD(LOG_INFO, "[touch]%s, FW is ready!!\n", __func__);
		usleep_range(4500, 5500);
	}

exit:
	return i32_ret;
}
/* upgrade firmware with image file */
int raydium_fw_upgrade_with_image(struct i2c_client *client,
				  unsigned int u32_fw_addr,
				  unsigned char u8_type)
{
	int i32_ret = ERROR;
	unsigned int u32_fw_size = 0;
	unsigned char *p_u8_firmware_data = NULL;
	unsigned int u32_write_offset = 0;
	unsigned short u16_write_length = 0;
	unsigned int u32_checksum = 0xFFFFFFFF;

	switch (u8_type) {
	case RAYDIUM_INIT:
		u32_fw_size = 0x1fc;
		p_u8_firmware_data = g_rad_init_image;
		break;
	case RAYDIUM_PARA:
		u32_fw_size = 0x158;
		p_u8_firmware_data = g_rad_para_image;
		break;
	case RAYDIUM_FIRMWARE:
		u32_fw_size = 0x61fc;
		p_u8_firmware_data = g_rad_fw_image;
		break;
	case RAYDIUM_BOOTLOADER:
		u32_fw_size = 0x7FC;
		p_u8_firmware_data = g_rad_boot_image;
		break;
	case RAYDIUM_TEST_FW:
		u32_fw_size = 0x635C;
		p_u8_firmware_data = g_rad_testfw_image;
		break;
	}

	LOGD(LOG_DEBUG, "[touch]CRC 0x%08X\n",
	     *(unsigned int *)(p_u8_firmware_data + u32_fw_size));

	u32_checksum = rc_crc32(p_u8_firmware_data,
				u32_fw_size, u32_checksum);
	u32_checksum = bits_reverse(u32_checksum, 32);
	memcpy((p_u8_firmware_data + u32_fw_size), &u32_checksum, 4);
	LOGD(LOG_DEBUG, "[touch]CRC result 0x%08X\n", u32_checksum);

	u32_fw_size += 4;

	u32_write_offset = 0;
	while (u32_write_offset < u32_fw_size) {
		if ((u32_write_offset + MAX_WRITE_PACKET_SIZE) < u32_fw_size)
			u16_write_length = MAX_WRITE_PACKET_SIZE;
		else
			u16_write_length =
				(unsigned short)
				(u32_fw_size - u32_write_offset);

		i32_ret = handle_i2c_pda_write(
				  client,
				  (u32_fw_addr + u32_write_offset),
				  (p_u8_firmware_data + u32_write_offset),
				  u16_write_length);
		if (i32_ret < 0)
			goto exit_upgrate;

		u32_write_offset += (unsigned long)u16_write_length;
	}
	u32_fw_addr += u32_write_offset;

exit_upgrate:
	if (i32_ret < 0) {
		LOGD(LOG_ERR, "[touch]upgrade failed\n");
		return i32_ret;
	}
	LOGD(LOG_INFO, "[touch]upgrade success\n");
	return SUCCESS;
}
int raydium_load_test_fw(struct i2c_client *client)
{
	unsigned int u32_read_data;
	unsigned int u32_write = 0;
	int i32_ret = ERROR;

	/* set skip load & MCU hold then SW reset*/
	/* sync_data:210;cmd_type:210h;ret_data:214h;test_mode:218h*/
	if (raydium_stop_mcu_3x(1) == ERROR)
		goto ERROR_EXIT;

	/*WRT boot-loader to PRAM first*/
	u32_write = 1;
	if (handle_i2c_pda_write(client, RAYDIUM_PDA_PRAMLOCK, (unsigned char *)&u32_write, 4) == ERROR)
		return ERROR;

	/*Sending bootloader*/

	i32_ret = raydium_write_to_pram_3X(client, 0x0000,
					   RAYDIUM_BOOTLOADER);
	if (i32_ret < 0)
		goto ERROR_EXIT;

	i32_ret = raydium_check_pram_crc_3X(client, 0x000, 0x7FC);
	if (i32_ret < 0)
		goto ERROR_EXIT;

	i32_ret = raydium_write_to_pram_3X(client, RAYDIUM_PDA_FIRMWAREADDR,
					   RAYDIUM_TEST_FW);
	if (i32_ret < 0)
		goto ERROR_EXIT;

	i32_ret = raydium_check_pram_crc_3X(client, RAYDIUM_PDA_FIRMWAREADDR,
					    RAYDIUM_PDA_CRCLENGTH);
	if (i32_ret < 0)
		goto ERROR_EXIT;

	/*clear CC & BL info*/
	u32_write = 0x00000000;
	if (handle_i2c_pda_write(client, 0x7F78, (unsigned char *)&u32_write, 4) == ERROR)
		return ERROR;

	/* set skip load & MCU hold then SW reset*/
	/* sync_data:210;cmd_type:210h;ret_data:214h;test_mode:218h*/
	/* read bootloader version*/
	if (handle_i2c_pda_read(client, 0x80, (unsigned char *)(&u32_read_data), 4) == ERROR)
		goto ERROR_EXIT;
	LOGD(LOG_INFO, "[touch]Read bootloader ver=0x%x!!\r\n", u32_read_data);

	u32_write = 0x0410;
	if (handle_i2c_pda_write(client, RAYDIUM_PDA_BOOTREG, (unsigned char *)&u32_write, 4) == ERROR)
		return ERROR;
	if (handle_i2c_pda_read(client, RAYDIUM_PDA_BOOTREG, (unsigned char *)(&u32_read_data), 4) == ERROR)
		goto ERROR_EXIT;
	LOGD(LOG_DEBUG, "[touch]write MCU STATUS=0x%x!!\r\n", u32_read_data);
	/*wait sw rst finish*/
	u32_write = 1;

	handle_i2c_pda_write(client, RAYDIUM_PDA_BLKRST, (unsigned char *)&u32_write, 4);
	msleep(35);

	if (handle_i2c_pda_read(client, RAYDIUM_PDA_BOOTREG, (unsigned char *)(&u32_read_data), 4) == ERROR)
		goto ERROR_EXIT;
	LOGD(LOG_INFO, "[touch]Read MCU STATUS=0x%x!!\r\n", u32_read_data);
	if (handle_i2c_pda_read(client, 0x200006E4, (unsigned char *)(&u32_read_data), 4) == ERROR)
		goto ERROR_EXIT;
	LOGD(LOG_INFO, "[touch]Read test fw version=0x%x!!\r\n", u32_read_data);

	return SUCCESS;

ERROR_EXIT:

	return ERROR;
}



