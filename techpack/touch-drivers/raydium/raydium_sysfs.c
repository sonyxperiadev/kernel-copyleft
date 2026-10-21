/*raydium_sysfs.c
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

#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include "raydium_driver.h"

static ssize_t raydium_touch_calibration_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	unsigned char u8_rbuffer[1];
	unsigned short u16_len = 0;
	int i32_ret = -1;
	unsigned char u8_retry = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (g_raydium_ts->is_suspend)
		LOGD(LOG_DEBUG, "[touch]RAD is_suspend at %s\n", __func__);

	g_u8_raydium_flag |= ENG_MODE;

	mutex_lock(&g_raydium_ts->lock);

	i32_ret = raydium_i2c_pda2_set_page(client,
					    g_raydium_ts->is_suspend,
					    RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0)
		goto exit_i2c_error;

	u8_rbuffer[0] = RAYDIUM_HOST_CMD_CALIBRATION;
	i32_ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR,
					 u8_rbuffer, 1);
	if (i32_ret < 0)
		goto exit_i2c_error;

	do {
		if (u8_rbuffer[0] == RAYDIUM_HOST_CMD_NO_OP)
			break;

		msleep(1000);

		i32_ret = raydium_i2c_pda2_read(client,
						RAYDIUM_PDA2_HOST_CMD_ADDR,
						u8_rbuffer, 1);
		if (i32_ret < 0)
			goto exit_i2c_error;

		LOGD(LOG_INFO, "[touch]RAD %s return 0x%02x!!\n",
		     __func__, u8_rbuffer[0]);
	} while (u8_retry++ < (SYN_I2C_RETRY_TIMES * 2));

	memcpy(p_i8_buf, u8_rbuffer, 1);

	u16_len = strlen(p_i8_buf);
	i32_ret = u16_len + 1;

exit_i2c_error:
	mutex_unlock(&g_raydium_ts->lock);
	g_u8_raydium_flag &= ~ENG_MODE;
	return i32_ret;
}

static ssize_t raydium_i2c_pda_access_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	unsigned char u8_rbuffer[4];
	unsigned short u16_len = 0;
	int i32_ret = -1;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (g_u32_length > 4)
		return -EINVAL;
	memset(u8_rbuffer, 0x00, 4);
	mutex_lock(&g_raydium_ts->lock);
	i32_ret = raydium_i2c_pda_read(client,
				       g_u32_addr,
				       u8_rbuffer,
				       g_u32_length);
	mutex_unlock(&g_raydium_ts->lock);
	if (i32_ret < 0)
		return i32_ret;

	snprintf(p_i8_buf, PAGE_SIZE, "0x%08X : 0x%02X%02X%02X%02X\n",
		 (unsigned int)g_u32_addr, u8_rbuffer[3], u8_rbuffer[2],
		 u8_rbuffer[1], u8_rbuffer[0]);
	u16_len = strlen(p_i8_buf);

	return u16_len + 1;
}


static ssize_t raydium_i2c_pda_access_via_pda2_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	unsigned char u8_rbuffer[4];
	unsigned short u16_len = 0;
	int i32_ret = -1;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (g_u32_length > 4)
		return -EINVAL;
	memset(u8_rbuffer, 0x00, 4);
	mutex_lock(&g_raydium_ts->lock);
	i32_ret = handle_i2c_pda_read(client,
				      g_u32_addr,
				      u8_rbuffer,
				      g_u32_length);
	mutex_unlock(&g_raydium_ts->lock);
	if (i32_ret < 0)
		return i32_ret;

	snprintf(p_i8_buf, PAGE_SIZE, "0x%08X : 0x%02X%02X%02X%02X\n",
		 (unsigned int)g_u32_addr, u8_rbuffer[3], u8_rbuffer[2],
		 u8_rbuffer[1], u8_rbuffer[0]);
	u16_len = strlen(p_i8_buf);

	return u16_len + 1;
}

static ssize_t raydium_check_i2c_show(struct device *dev,
				      struct device_attribute *attr,
				      char *p_i8_buf)
{
	unsigned char u8_rbuffer[4];
	unsigned short u16_len = 0;
	int i32_ret = -1;

	if (g_raydium_ts->is_suspend)
		LOGD(LOG_DEBUG, "[touch]RAD is_suspend at %s\n", __func__);

	mutex_lock(&g_raydium_ts->lock);
	i32_ret = handle_i2c_pda_read(g_raydium_ts->client,
				      RAYDIUM_CHK_I2C_CMD, u8_rbuffer, 4);
	if (i32_ret < 0)
		goto exit_i2c_error;

	snprintf(p_i8_buf, PAGE_SIZE, "[touch]RAD Touch check i2c: %02X%02X\n",
		 u8_rbuffer[3], u8_rbuffer[2]);
	u16_len = strlen(p_i8_buf);
	i32_ret = u16_len + 1;

exit_i2c_error:
	mutex_unlock(&g_raydium_ts->lock);
	return i32_ret;
}

static ssize_t raydium_hw_reset_show(struct device *dev,
				     struct device_attribute *attr,
				     char *p_i8_buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	int i32_ret = SUCCESS;


	LOGD(LOG_INFO, "[touch]HW reset\n");
	g_u8_resetflag = true;
	g_u8_raydium_flag |= ENG_MODE;

	/*HW reset*/
	if (gpio_is_valid(g_raydium_ts->rst_gpio)) {
		gpio_set_value(g_raydium_ts->rst_gpio, 1);
		gpio_set_value(g_raydium_ts->rst_gpio, 0);
		msleep(RAYDIUM_RESET_INTERVAL_MSEC);
		gpio_set_value(g_raydium_ts->rst_gpio, 1);
	}

	g_u8_i2c_mode = PDA2_MODE;
	if (raydium_disable_i2c_deglitch() == ERROR)
		LOGD(LOG_ERR, "[touch]disable_i2c_deglitch_3x NG!\r\n");

	i32_ret = wait_irq_state(client, 1000, 2000);
	if (i32_ret != ERROR)
		msleep(35);

	g_u8_raydium_flag &= ~ENG_MODE;

	snprintf(p_i8_buf, PAGE_SIZE, "Raydium HW Reset : %d\n", i32_ret);
	LOGD(LOG_INFO, "[touch]%s\n", p_i8_buf);
	return strlen(p_i8_buf) + 1;
}

static ssize_t raydium_palm_status_show(struct device *dev,
					struct device_attribute *attr,
					char *p_i8_buf)
{
	unsigned short u16_len = 0;

	unsigned char u8_tp_status[MAX_TCH_STATUS_PACKET_SIZE];
	unsigned char u8_tp_buf[MAX_REPORT_PACKET_SIZE];

	raydium_read_touchdata(u8_tp_status, u8_tp_buf);
	snprintf(p_i8_buf, PAGE_SIZE, "[touch] palm_status : %d\n",
		 u8_tp_status[POS_GES_STATUS]);

	u16_len = strlen(p_i8_buf);
	return u16_len + 1;
}
static ssize_t raydium_touch_lock_store(struct device *dev,
					struct device_attribute *attr,
					const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	unsigned char u8_mode;
	unsigned char u8_wbuffer[1];

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (g_raydium_ts->is_suspend)
		LOGD(LOG_DEBUG, "[touch]RAD is_suspend at %s\n", __func__);

	/* receive command line arguments string */
	if (count > 2)
		return -EINVAL;

	i32_ret = kstrtou8(p_i8_buf, 16, &u8_mode);
	if (i32_ret < 0)
		return i32_ret;
	g_u8_raydium_flag |= ENG_MODE;
	mutex_lock(&g_raydium_ts->lock);

	switch (u8_mode) {
	case 0: /* Disable Touch lock */

		if (g_raydium_ts->is_sleep != 1)
			break;
		g_u8_resetflag = true;
		if (gpio_is_valid(g_raydium_ts->rst_gpio)) {
			gpio_set_value(g_raydium_ts->rst_gpio, 1);
			gpio_set_value(g_raydium_ts->rst_gpio, 0);
			msleep(RAYDIUM_RESET_INTERVAL_MSEC);/*5ms*/
			gpio_set_value(g_raydium_ts->rst_gpio, 1);
			msleep(RAYDIUM_RESET_DELAY_MSEC);/*100ms*/
		}
		LOGD(LOG_INFO, "[touch]RAD %s disable touch lock!!\n", __func__);

		g_raydium_ts->is_sleep = 0;
		break;

	case 1: /* Enable Touch lock */

		if (g_raydium_ts->is_sleep == 1)
			break;
		i32_ret = raydium_i2c_pda2_set_page(client,
						    g_raydium_ts->is_suspend,
						    RAYDIUM_PDA2_PAGE_0);
		if (i32_ret < 0)
			goto exit_i2c_error;
		/*fw enter sleep mode*/
		u8_wbuffer[0] = RAYDIUM_HOST_CMD_PWR_SLEEP;
		i32_ret = raydium_i2c_pda2_write(client,
						 RAYDIUM_PDA2_HOST_CMD_ADDR,
						 u8_wbuffer,
						 1);
		if (i32_ret < 0)
			goto exit_i2c_error;

		LOGD(LOG_INFO, "[touch]RAD %s enable touch lock!!\n", __func__);
		g_raydium_ts->is_sleep = 1;
		break;
	}

exit_i2c_error:
	mutex_unlock(&g_raydium_ts->lock);
	g_u8_raydium_flag &= ~ENG_MODE;
	return count;
}

static ssize_t raydium_check_driver_version_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	/*unsigned char rbuffer[4];*/
	unsigned short u16_len = 0;
	int i32_ret = -1;

	snprintf(p_i8_buf, PAGE_SIZE, "RAD Driver Ver: 0x%X\n",
		 g_u32_driver_version);

	u16_len = strlen(p_i8_buf);
	i32_ret = u16_len + 1;
	return i32_ret;
}

static ssize_t raydium_check_fw_version_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	unsigned char u8_rbuffer[4];
	unsigned short u16_len = 0;
	int i32_ret = -1;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	unsigned int fw_version, image_version;

	if (g_raydium_ts->is_suspend)
		LOGD(LOG_DEBUG, "[touch]RAD is_suspend at %s\n", __func__);

	g_u8_raydium_flag |= ENG_MODE;
	mutex_lock(&g_raydium_ts->lock);

	i32_ret = raydium_i2c_pda2_set_page(client,
					    g_raydium_ts->is_suspend,
					    RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0)
		goto exit_i2c_error;
	i32_ret = raydium_i2c_pda2_read(client, RAYDIUM_PDA2_FW_VERSION_ADDR,
					u8_rbuffer, 4);
	if (i32_ret < 0)
		goto exit_i2c_error;
	snprintf(p_i8_buf, PAGE_SIZE, "RAD Touch FW Ver : %02X%02X%02X%02X\n",
		 u8_rbuffer[0], u8_rbuffer[1], u8_rbuffer[2], u8_rbuffer[3]);

	fw_version = (u8_rbuffer[0] << 24)
		     | (u8_rbuffer[1] << 16)
		     | (u8_rbuffer[2] << 8)
		     | u8_rbuffer[3];
	LOGD(LOG_INFO, "[touch]RAD FW ver : 0x%x\n", fw_version);

	image_version = (g_rad_para_image[PARA_FW_VERSION_OFFSET] << 24) |
			(g_rad_para_image[PARA_FW_VERSION_OFFSET + 1] << 16) |
			(g_rad_para_image[PARA_FW_VERSION_OFFSET + 2] << 8) |
			g_rad_para_image[PARA_FW_VERSION_OFFSET + 3];

	LOGD(LOG_INFO, "[touch]RAD Image FW ver : 0x%x\n", image_version);

	mutex_unlock(&g_raydium_ts->lock);
	g_u8_raydium_flag &= ~ENG_MODE;

	if (fw_version != image_version)
		LOGD(LOG_INFO, "[touch]%s, FW need upgrade.\n", __func__);

	u16_len = strlen(p_i8_buf);
	i32_ret = u16_len + 1;
	goto exit_upgrade;

exit_i2c_error:
	mutex_unlock(&g_raydium_ts->lock);
	g_u8_raydium_flag &= ~ENG_MODE;

exit_upgrade:
	return i32_ret;
}

static ssize_t raydium_check_panel_version_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	unsigned char u8_rbuffer[8];
	unsigned short u16_len = 0;
	int i32_ret = -1;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (g_raydium_ts->is_suspend)
		LOGD(LOG_DEBUG, "[touch]RAD is_suspend at %s\n", __func__);
	g_u8_raydium_flag |= ENG_MODE;
	mutex_lock(&g_raydium_ts->lock);

	i32_ret = raydium_i2c_pda2_set_page(client,
					    g_raydium_ts->is_suspend,
					    RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0)
		goto exit_i2c_error;
	i32_ret = raydium_i2c_pda2_read(client,
					RAYDIUM_PDA2_PANEL_VERSION_ADDR,
					u8_rbuffer, 8);
	if (i32_ret < 0)
		goto exit_i2c_error;
	snprintf(p_i8_buf, PAGE_SIZE,
		 "RAD Touch Panel Version : %02X%02X%02X%02X%02X%02X\n",
		 u8_rbuffer[0], u8_rbuffer[1], u8_rbuffer[2],
		 u8_rbuffer[3], u8_rbuffer[4], u8_rbuffer[5]);
	mutex_unlock(&g_raydium_ts->lock);
	g_u8_raydium_flag &= ~ENG_MODE;

	u16_len = strlen(p_i8_buf);
	i32_ret = u16_len + 1;
	goto exit_upgrade;

exit_i2c_error:
	mutex_unlock(&g_raydium_ts->lock);
	g_u8_raydium_flag &= ~ENG_MODE;
exit_upgrade:
	return i32_ret;
}

static ssize_t raydium_fw_upgrade_store(struct device *dev,
					struct device_attribute *attr,
					const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;

	/* receive command line arguments string */
	if (count > 2)
		return -EINVAL;

	i32_ret = kstrtou8(p_i8_buf, 16, &g_u8_upgrade_type);
	if (i32_ret < 0)
		return i32_ret;

	return count;
}

static ssize_t raydium_fw_upgrade_show(struct device *dev,
				       struct device_attribute *attr,
				       char *p_i8_buf)
{
	int i32_ret = 0, i32_result = FAIL;
	unsigned short u16_len = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	LOGD(LOG_INFO, "[touch]RAD burn type is %d\n", g_u8_upgrade_type);

	g_u8_raydium_flag |= ENG_MODE;

	if ((g_u8_table_setting == 1) && (g_u8_table_init == 1))
		raydium_mem_table_setting();

	if (g_u8_upgrade_type == 1) {
		i32_ret = raydium_burn_fw(client);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_result = SUCCESS;
	} else if (g_u8_upgrade_type == 2) {
		i32_ret = raydium_burn_comp(client);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_result = SUCCESS;
	}
#ifdef RAD_SELFTEST
	else if (g_u8_upgrade_type == 4) {
		i32_ret = raydium_load_test_fw(client);
		if (i32_ret < 0)
			goto exit_upgrade;

		i32_result = SUCCESS;
	}
#endif

exit_upgrade:
	LOGD(LOG_DEBUG, "[touch]g_u8_raydium_flag : %d", g_u8_raydium_flag);
	g_u8_raydium_flag &= ~ENG_MODE;
	g_u8_upgrade_type = 0;

	snprintf(p_i8_buf, PAGE_SIZE, "FW Upgrade result : %d\n", i32_result);
	u16_len = strlen(p_i8_buf);
	return u16_len + 1;
}
static ssize_t raydium_i2c_pda2_page_store(struct device *dev,
		struct device_attribute *attr,
		const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	unsigned char u8_page = 0;
	char *temp_buf, *token, *free_temp_buf, *free_token;
	const char *delim = " ,";
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	/* receive command line arguments string */
	if (count < 2)
		return -EINVAL;
	temp_buf = kzalloc(count + 1, GFP_KERNEL);
	if (temp_buf == NULL)
		return -ENOMEM;

	token = kzalloc(count + 1, GFP_KERNEL);
	if (token == NULL) {
		kfree(temp_buf);
		return -ENOMEM;
	}

	free_temp_buf = temp_buf;
	free_token = token;

	strlcpy(temp_buf, p_i8_buf, count);

	token = strsep(&temp_buf, delim);

	if (temp_buf) {
		LOGD(LOG_ERR, "[touch]input error, extra auguments!n");
		i32_ret = -EINVAL;
		goto exit_error;
	}
	i32_ret = kstrtou8(token, 16, &u8_page);

	if (i32_ret < 0)
		goto exit_error;

	mutex_lock(&g_raydium_ts->lock);

	i32_ret = raydium_i2c_pda2_set_page(client, g_raydium_ts->is_suspend, u8_page);
	if (i32_ret < 0)
		goto exit_set_error;

	/* TODO: Page check, Due to ISR will change page back to Page_0.
	 *  Or disable IRQ during PDA2 access period
	 */

exit_set_error:
	mutex_unlock(&g_raydium_ts->lock);

exit_error:
	kfree(free_token);
	kfree(free_temp_buf);
	return count;
}

static ssize_t raydium_mem_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *p_i8_buf, size_t count)
{
#if !ENABLE_FW_LOADER
	int i32_ret = 0;
	unsigned char u8_type = 0;
	unsigned int u32_image_version;

	/* receive command line arguments string */
	if (count > 2)
		return -EINVAL;

	LOGD(LOG_INFO, "[touch]%s\n", __func__);

	i32_ret = kstrtou8(p_i8_buf, 16, &u8_type);
	if (i32_ret < 0)
		return i32_ret;

	if (u8_type > 3) {
		LOGD(LOG_ERR, "[touch]Input invalid value!!\n");
		return ERROR;
	}

	if (g_rad_boot_image) {
		kfree(g_rad_boot_image);
		g_rad_boot_image = NULL;
	}
	if (g_rad_init_image) {
		kfree(g_rad_init_image);
		g_rad_init_image = NULL;
	}
	if (g_rad_fw_image) {
		kfree(g_rad_fw_image);
		g_rad_fw_image = NULL;
	}

	kfree(g_rad_para_image);
	g_rad_para_image = NULL;

	if (g_rad_testfw_image) {
		kfree(g_rad_testfw_image);
		g_rad_testfw_image = NULL;
	}

	kfree(g_rad_testpara_image);
	g_rad_testpara_image = NULL;

	if (!raydium_id_init(u8_type)) {
		LOGD(LOG_ERR, "[touch]Set Raydium id failed!\n");
		return count;
	}

	raydium_mem_table_init(g_raydium_ts->id);
	if (raydium_mem_table_setting()) {
		u32_image_version = (g_rad_para_image[PARA_FW_VERSION_OFFSET] << 24) |
				    (g_rad_para_image[PARA_FW_VERSION_OFFSET + 1] << 16) |
				    (g_rad_para_image[PARA_FW_VERSION_OFFSET + 2] << 8) |
				    g_rad_para_image[PARA_FW_VERSION_OFFSET + 3];

		LOGD(LOG_INFO, "[touch]RAD Image FW ver : 0x%x\n", u32_image_version);
	} else
		LOGD(LOG_ERR, "[touch]Mem init failed!\n");
	if (g_rad_para_image) {
		kfree(g_rad_para_image);
		g_rad_para_image = NULL;
	}
#endif
	return count;
}

static ssize_t raydium_i2c_raw_data_store(struct device *dev,
		struct device_attribute *attr,
		const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	char *temp_buf, *token, *free_temp_buf, *free_token;
	const char *delim = " ,";
	unsigned char u8_w_data[RAD_FT_CMD_LENGTH];

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	/* receive command line arguments string */
	if (count < 2)
		return -EINVAL;
	temp_buf = kzalloc(count + 1, GFP_KERNEL);
	if (temp_buf  == NULL) {
		LOGD(LOG_ERR, "[touch]kzalloc temp_buf failed\n");
		return -ENOMEM;
	}

	token = kzalloc(count + 1, GFP_KERNEL);
	if (token  == NULL) {
		kfree(temp_buf);
		return -ENOMEM;
	}

	free_temp_buf = temp_buf;
	free_token = token;

	strlcpy(temp_buf, p_i8_buf, count);

	token = strsep(&temp_buf, delim);

	i32_ret = kstrtou8(token, 16, &g_u8_raw_data_type);

	token = strsep(&temp_buf, delim);
	if (token) {
		i32_ret = kstrtouint(token, 16, &g_u32_raw_data_len);
		if (i32_ret < 0)
			goto exit_error;


	} else { /* without length info*/
		i32_ret = -EINVAL;
		goto exit_error;
	}

	if (temp_buf) { /* too much arguments*/
		i32_ret = -E2BIG;
		goto exit_error;
	}

	memset(u8_w_data, 0x00, RAD_FT_CMD_LENGTH);

	mutex_lock(&g_raydium_ts->lock);
	i32_ret = raydium_i2c_pda2_set_page(client,
					    g_raydium_ts->is_suspend,
					    RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0) {
		mutex_unlock(&g_raydium_ts->lock);
		goto exit_error;
	}

	g_u8_resetflag = true;

	u8_w_data[RAD_HOST_CMD_POS] = RAYDIUM_HOST_CMD_NO_OP;
	u8_w_data[RAD_FT_CMD_POS] = g_u8_raw_data_type;

	i32_ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR,
					 u8_w_data, RAD_FT_CMD_LENGTH);
	mutex_unlock(&g_raydium_ts->lock);
	if (i32_ret < 0)
		goto exit_error;

	if (g_u8_raw_data_type == 0) {
		msleep(20);
		g_u8_resetflag = false;
	}
exit_error:
	kfree(free_token);
	kfree(free_temp_buf);

	return count;
}

static ssize_t raydium_i2c_raw_data_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	unsigned char u8_rbuffer[MAX_READ_PACKET_SIZE];
	unsigned int u32_target_addr;
	unsigned int u32_offset;
	unsigned short u16_read_length;

	int i32_ret = -1;
	int i32_retry = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct raydium_ts_data *ts =
		(struct raydium_ts_data *)i2c_get_clientdata(client);
	unsigned char u8_retry_limit = (ts->is_suspend) ? 100 : 30;

	memset(u8_rbuffer, 0x00, MAX_READ_PACKET_SIZE);

	/* make sure update flag was set*/
	for (i32_retry = 0; i32_retry < u8_retry_limit; i32_retry++) {
		mutex_lock(&ts->lock);
		i32_ret = raydium_i2c_pda2_set_page(client,
						    ts->is_suspend,
						    RAYDIUM_PDA2_PAGE_0);
		if (i32_ret < 0) {
			mutex_unlock(&ts->lock);
			goto exit_i2c_error;
		}

		i32_ret = raydium_i2c_pda2_read(client,
						RAYDIUM_PDA2_HOST_CMD_ADDR,
						u8_rbuffer,
						RAD_FT_CMD_LENGTH);
		mutex_unlock(&ts->lock);
		if (i32_ret < 0)
			goto exit_flag_error;

		if ((u8_rbuffer[RAD_FT_CMD_POS] & RAYDIUM_FT_UPDATE) ==
		    RAYDIUM_FT_UPDATE)
			break;

		usleep_range(500, 1500);
	}

	if (i32_retry == u8_retry_limit) {
		i32_ret = -EAGAIN;
		goto exit_flag_error;
	}

	u32_offset = 0;
	u16_read_length = 0;
	while (u32_offset < g_u32_raw_data_len) {
		if ((u32_offset + MAX_READ_PACKET_SIZE) <
		    g_u32_raw_data_len)
			u16_read_length = MAX_READ_PACKET_SIZE;
		else
			u16_read_length =
				(unsigned short)(g_u32_raw_data_len - u32_offset);

		u32_target_addr = RAD_READ_FT_DATA_CMD + u32_offset;

		mutex_lock(&(ts->lock));

		/*using byte mode to read 4 bytes*/
		i32_ret = handle_i2c_pda_read(client,
					      u32_target_addr,
					      u8_rbuffer,
					      u16_read_length);

		mutex_unlock(&(ts->lock));
		if (i32_ret < 0)
			goto exit_flag_error;

		memcpy((p_i8_buf + u32_offset), u8_rbuffer, u16_read_length);

		u32_offset += u16_read_length;
	}

	/* clear update flag to get next one*/
	u8_rbuffer[RAD_HOST_CMD_POS] = RAYDIUM_HOST_CMD_NO_OP;
	u8_rbuffer[RAD_FT_CMD_POS] = g_u8_raw_data_type;
	mutex_lock(&ts->lock);
	i32_ret = raydium_i2c_pda2_write(client, RAYDIUM_PDA2_HOST_CMD_ADDR,
					 u8_rbuffer, RAD_FT_CMD_LENGTH);
	mutex_unlock(&ts->lock);
	if (i32_ret < 0)
		goto exit_flag_error;

	return g_u32_raw_data_len;
exit_i2c_error:
	mutex_unlock(&(ts->lock));
exit_flag_error:
	return i32_ret;
}

static ssize_t raydium_i2c_pda_access_store(struct device *dev,
		struct device_attribute *attr,
		const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	char *temp_buf, *token, *free_temp_buf, *free_token;
	const char *delim = " ,";
	unsigned char u8_w_data[MAX_WRITE_PACKET_SIZE];
	unsigned int u32_data_count = 0;
	unsigned int u32_data_index = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	/* receive command line arguments string */
	if (count < 2)
		return -EINVAL;

	temp_buf = kzalloc(count + 1, GFP_KERNEL);
	if (temp_buf == NULL)
		return -ENOMEM;

	token = kzalloc(count + 1, GFP_KERNEL);
	if (token == NULL) {
		kfree(temp_buf);
		return -ENOMEM;
	}

	free_temp_buf = temp_buf;
	free_token = token;

	strlcpy(temp_buf, p_i8_buf, count);

	token = strsep(&temp_buf, delim);

	i32_ret = kstrtoul(token, 16, &g_u32_addr);

	token = strsep(&temp_buf, delim);
	if (token)
		i32_ret = kstrtouint(token, 16, &u32_data_count);
	else
		goto exit_error;
	if (g_u32_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;
	g_u32_length = u32_data_count;

	memset(u8_w_data, 0x00, MAX_WRITE_PACKET_SIZE);

	if (temp_buf && u32_data_count) {
		u32_data_index = 0;
		while (u32_data_count) {
			token = strsep(&temp_buf, delim);
			i32_ret = kstrtou8(token, 16,
					   &u8_w_data[u32_data_index++]);
			if (i32_ret < 0)
				goto exit_error;
			u32_data_count--;
		}
		mutex_lock(&g_raydium_ts->lock);
		i32_ret = raydium_i2c_pda_write(client, g_u32_addr,
						u8_w_data, g_u32_length);
		mutex_unlock(&g_raydium_ts->lock);
	}

exit_error:
	kfree(free_token);
	kfree(free_temp_buf);
	return count;
}
static ssize_t raydium_i2c_pda_access_via_pda2_store(struct device *dev,
		struct device_attribute *attr,
		const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	char *temp_buf, *token, *free_temp_buf, *free_token;
	const char *delim = " ,";
	unsigned char u8_w_data[MAX_WRITE_PACKET_SIZE];
	unsigned int u32_data_count = 0;
	unsigned int u32_data_index = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	/* receive command line arguments string */
	if (count < 2)
		return -EINVAL;

	temp_buf = kzalloc(count + 1, GFP_KERNEL);
	if (temp_buf == NULL)
		return -ENOMEM;

	token = kzalloc(count + 1, GFP_KERNEL);
	if (token == NULL) {
		kfree(temp_buf);
		return -ENOMEM;
	}

	free_temp_buf = temp_buf;
	free_token = token;

	strlcpy(temp_buf, p_i8_buf, count);

	token = strsep(&temp_buf, delim);

	i32_ret = kstrtoul(token, 16, &g_u32_addr);

	token = strsep(&temp_buf, delim);
	if (token)
		i32_ret = kstrtouint(token, 16, &u32_data_count);
	else
		goto exit_error;
	if (g_u32_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;
	g_u32_length = u32_data_count;

	memset(u8_w_data, 0x00, MAX_WRITE_PACKET_SIZE);

	if (temp_buf && u32_data_count) {
		u32_data_index = 0;
		while (u32_data_count) {
			token = strsep(&temp_buf, delim);
			i32_ret = kstrtou8(token, 16,
					   &u8_w_data[u32_data_index++]);
			if (i32_ret < 0)
				goto exit_error;
			u32_data_count--;
		}
		mutex_lock(&g_raydium_ts->lock);
		i32_ret = handle_i2c_pda_write(client, g_u32_addr,
					       u8_w_data, g_u32_length);
		mutex_unlock(&g_raydium_ts->lock);
	}

exit_error:
	kfree(free_token);
	kfree(free_temp_buf);
	return count;
}

static ssize_t raydium_i2c_pda2_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	unsigned char u8_mode;

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (g_raydium_ts->is_suspend)
		LOGD(LOG_DEBUG, "[touch]RAD is_suspend at %s\n", __func__);

	/* receive command line arguments string */
	if (count > 2)
		return -EINVAL;

	i32_ret = kstrtou8(p_i8_buf, 16, &u8_mode);
	if (i32_ret < 0)
		return i32_ret;
	i32_ret = raydium_i2c_mode_control(client, u8_mode);
	if (i32_ret < 0)
		return i32_ret;

	return count;
}

static ssize_t raydium_i2c_pda2_access_show(struct device *dev,
		struct device_attribute *attr,
		char *p_i8_buf)
{
	unsigned char u8_rbuffer[4];
	unsigned short u16_len = 0;
	int i32_ret = -1;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (g_u32_length > 4)
		return -EINVAL;
	memset(u8_rbuffer, 0x00, 4);

	mutex_lock(&g_raydium_ts->lock);
	i32_ret = raydium_i2c_pda2_read(client, g_u8_addr,
					u8_rbuffer, g_u32_length);
	mutex_unlock(&g_raydium_ts->lock);
	if (i32_ret < 0)
		return i32_ret;

	snprintf(p_i8_buf, PAGE_SIZE, "0x%04X : 0x%02X%02X%02X%02X\n",
		 g_u8_addr, u8_rbuffer[3], u8_rbuffer[2],
		 u8_rbuffer[1], u8_rbuffer[0]);
	u16_len = strlen(p_i8_buf);

	return u16_len + 1;
}

static ssize_t raydium_i2c_pda2_access_store(struct device *dev,
		struct device_attribute *attr,
		const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	char *temp_buf, *token, *free_temp_buf, *free_token;
	const char *delim = " ,";
	unsigned char u8_w_data[MAX_WRITE_PACKET_SIZE];
	unsigned int u32_data_count = 0;
	unsigned int u32_data_index = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	/* receive command line arguments string */
	if (count < 2)
		return -EINVAL;
	temp_buf = kzalloc(count + 1, GFP_KERNEL);
	if (temp_buf  == NULL) {
		LOGD(LOG_ERR, "[touch]kzalloc temp_buf failed\n");
		return -ENOMEM;
	}

	token = kzalloc(count + 1, GFP_KERNEL);
	if (token  == NULL) {
		kfree(temp_buf);
		return -ENOMEM;
	}

	free_temp_buf = temp_buf;
	free_token = token;

	strlcpy(temp_buf, p_i8_buf, count);

	token = strsep(&temp_buf, delim);

	i32_ret = kstrtou8(token, 16, &g_u8_addr);

	token = strsep(&temp_buf, delim);
	if (token)
		i32_ret = kstrtouint(token, 16, &u32_data_count);
	else {
		i32_ret = -EINVAL;
		goto exit_error;
	}

	if (u32_data_count > MAX_WRITE_PACKET_SIZE) {
		i32_ret = -EINVAL;
		goto exit_error;
	}

	memset(u8_w_data, 0x00, MAX_WRITE_PACKET_SIZE);

	g_u32_length = u32_data_count;

	if (temp_buf && u32_data_count) {
		u32_data_index = 0;
		while (u32_data_count) {
			token = strsep(&temp_buf, delim);
			i32_ret = kstrtou8(token, 16,
					   &u8_w_data[u32_data_index++]);
			if (i32_ret < 0)
				goto exit_error;
			u32_data_count--;
		}

		mutex_lock(&g_raydium_ts->lock);
		i32_ret = raydium_i2c_pda2_write(client, g_u8_addr,
						 u8_w_data, g_u32_length);
		mutex_unlock(&g_raydium_ts->lock);
		if (i32_ret < 0)
			goto exit_error;
	}

exit_error:
	kfree(free_token);
	kfree(free_temp_buf);
	return count;
}

static ssize_t raydium_receive_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	const char *delim = " ,";
	char *token, *temp_buf, *free_token = NULL, *free_temp_buf = NULL;
	static unsigned char *p_u8_firmware_data;

	unsigned char u8_cmd;
	unsigned long u32_len;
	static unsigned char u8_type;
	static unsigned int u32_index;

	if (count == 20) { /*check FW type*/
		temp_buf = kzalloc(32, GFP_KERNEL);
		if (temp_buf  == NULL) {
			LOGD(LOG_ERR, "[touch]kzalloc temp_buf failed\n");
			return -ENOMEM;
		}

		token = kzalloc(32, GFP_KERNEL);
		if (token == NULL) {
			kfree(temp_buf);
			return -ENOMEM;
		}

		free_token = token;
		free_temp_buf = temp_buf;

		snprintf(temp_buf, PAGE_SIZE, "%s", p_i8_buf);
		token = strsep(&temp_buf, delim);
		i32_ret = kstrtou8(token, 16, &u8_cmd);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]kstrtou8 failed\n");
			kfree(free_token);
			free_token = NULL;
			kfree(free_temp_buf);
			free_temp_buf = NULL;
		}

		token = strsep(&temp_buf, delim);
		i32_ret = kstrtou8(token, 16, &u8_type);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]kstrtou8 failed\n");
			kfree(temp_buf);
			kfree(token);
		}

		token = strsep(&temp_buf, delim);
		i32_ret = kstrtoul(token, 16, &u32_len);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]kstrtou8 failed\n");
			kfree(temp_buf);
			kfree(token);
		}

		LOGD(LOG_INFO, "[touch]uc_cmd=0x%x, uc_type=0x%x, u16_len=0x%x\n",
		     u8_cmd, u8_type, (unsigned int)u32_len);

		if (u8_cmd == RAD_CMD_UPDATE_BIN) { /*check FW length*/
			u32_index = 0;
			if (u8_type == RAYDIUM_BOOTLOADER) {
				memset(g_rad_boot_image, 0, u32_len);
				p_u8_firmware_data = g_rad_boot_image;
			} else if (u8_type == RAYDIUM_INIT) {
				memset(g_rad_init_image, 0, u32_len);
				p_u8_firmware_data = g_rad_init_image;
			} else if (u8_type == RAYDIUM_PARA) {
				memset(g_rad_para_image, 0, u32_len);
				p_u8_firmware_data = g_rad_para_image;
			} else if (u8_type == RAYDIUM_FIRMWARE) {
				memset(g_rad_fw_image, 0, u32_len);
				p_u8_firmware_data = g_rad_fw_image;
			} else if (u8_type == RAYDIUM_TEST_PARA) {
				memset(g_rad_testpara_image, 0, u32_len);
				p_u8_firmware_data = g_rad_testpara_image;
			} else if (u8_type == RAYDIUM_TEST_FW) {
				memset(g_rad_testfw_image, 0, u32_len);
				p_u8_firmware_data = g_rad_testfw_image;
			}

		} else if (u8_cmd == RAD_CMD_UPDATE_END) { /*set buffer finish*/
			if (u8_type == RAYDIUM_TEST_FW) {
				memcpy((g_rad_testfw_image + RAD_FW_3X_SIZE),
				       g_rad_testpara_image, RAD_PARA_3X_SIZE + 4);
			}

			u32_index = 0;
			g_u8_table_setting = 0;

		} else if (u8_cmd == RAD_CMD_BURN_FINISH) { /*free buffer*/
			u8_type = 0;
			u32_index = 0;
			g_u8_table_setting = 1;
		}

		if (free_temp_buf) {
			kfree(free_temp_buf);
			free_temp_buf = NULL;
		}
		if (free_token) {
			kfree(free_token);
			free_token = NULL;
		}
	} else  if (count > 10) {	/*start copy FW to array*/
		memcpy((p_u8_firmware_data + u32_index), p_i8_buf, count);
		u32_index += count;
	} else
		LOGD(LOG_ERR, "[touch]other case, count=%d\n", count);

	return count;
}

static ssize_t raydium_log_level_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	unsigned char u8_level = 0;

	/* receive command line arguments string */
	if (count > 2)
		return -EINVAL;
	i32_ret = kstrtou8(p_i8_buf, 16, &u8_level);
	if (i32_ret < 0)
		return i32_ret;
	g_u8_log_level = u8_level;
	LOGD(LOG_ERR, "[touch]g_u8_log_level = %d\r\n", g_u8_log_level);
	return count;
}


#ifdef RAD_SELFTEST
static ssize_t raydium_reset_control_store(struct device *dev,
		struct device_attribute *attr,
		const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	unsigned char u8_high;

	if (g_raydium_ts->is_suspend)
		LOGD(LOG_DEBUG, "[touch]RAD is_suspend at %s\n", __func__);

	/* receive command line arguments string */
	if (count > 2)
		return -EINVAL;

	i32_ret = kstrtou8(p_i8_buf, 16, &u8_high);
	if (i32_ret < 0)
		return i32_ret;

	g_u8_i2c_mode = PDA2_MODE;
	g_u8_resetflag = true;

	if (u8_high) {
		LOGD(LOG_INFO, "[touch]RAD %s set reset gpio to high!!\n",
		     __func__);
		gpio_set_value(g_raydium_ts->rst_gpio, 1);
	} else {
		LOGD(LOG_INFO, "[touch]RAD %s set reset gpio to low!!\n",
		     __func__);
		gpio_set_value(g_raydium_ts->rst_gpio, 0);
	}
	return count;
}


static ssize_t raydium_irq_state_show(struct device *dev,
				      struct device_attribute *attr,
				      char *p_i8_buf)
{
	unsigned int u32_irq_value;

	u32_irq_value = gpio_get_value(g_raydium_ts->irq_gpio);

	snprintf(p_i8_buf, PAGE_SIZE, "%d", u32_irq_value);
	LOGD(LOG_DEBUG, "%s\n", p_i8_buf);
	return strlen(p_i8_buf) + 1;
}


static ssize_t raydium_flag_show(struct device *dev,
				 struct device_attribute *attr,
				 char *p_i8_buf)
{
	unsigned short u16_len = 0;

	snprintf(p_i8_buf, PAGE_SIZE, "%d", g_u8_raydium_flag);
	LOGD(LOG_DEBUG, "[touch]RAD flag : %d\n", g_u8_raydium_flag);
	u16_len = strlen(p_i8_buf);

	return u16_len + 1;
}

static ssize_t raydium_flag_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *p_i8_buf, size_t count)
{
	int i32_ret = 0;
	unsigned char u8_flag = 0;

	/* receive command line arguments string */
	if (count > 2)
		return -EINVAL;
	i32_ret = kstrtou8(p_i8_buf, 16, &u8_flag);
	if (i32_ret < 0)
		return i32_ret;
	g_u8_raydium_flag = u8_flag;
	return count;
}

static ssize_t raydium_selftest_show(struct device *dev,
				     struct device_attribute *attr,
				     char *p_i8_buf)
{
	int i32_ret = SUCCESS;

	LOGD(LOG_INFO, "[touch]do selftest\n");

	i32_ret = raydium_do_selftest(g_raydium_ts);


	snprintf(p_i8_buf, PAGE_SIZE, "Raydium do selftest : %d\n", i32_ret);

	return strlen(p_i8_buf) + 1;
}

#endif
/* panel calibration cmd (R)
 *  example:cat raydium_ic_verion
 */
static DEVICE_ATTR(raydium_touch_calibration, 0644,
		   raydium_touch_calibration_show,
		   NULL);

/* check the i2c (R)
 *  example:cat raydium_check_i2c
 */
static DEVICE_ATTR(raydium_check_i2c, 0644,
		   raydium_check_i2c_show,
		   NULL);

/* upgrade configurate and algo firmware from app.bin (W)
 *  example:echo "offset num_of_bin length *_app.bin [length *_app.bin]"
 *  > raydium_fw_upgrade_mode
 */
static DEVICE_ATTR(raydium_fw_upgrade, 0644,
		   raydium_fw_upgrade_show,
		   raydium_fw_upgrade_store);

/* change I2C communication mode (W)
 *  example:echo 1 > raydium_i2c_pda2_mode ==> enable pda2 mode
 *        echo 0 > raydium_i2c_pda2_mode ==> disable pda2 mode
 */
static DEVICE_ATTR(raydium_i2c_pda2_mode, 0644,
		   NULL,
		   raydium_i2c_pda2_mode_store);

/* I2C pda mode (R/W)
 *  example:    cat raydium_i2c_pda_access ==> read pda address provided by the
 *		following cmd
 *		echo ADDRinHEX [DATAinHEX] > raydium_i2c_pda_access ==> write
 *		pda address [data]
 */
static DEVICE_ATTR(raydium_i2c_pda_access, 0644,
		   raydium_i2c_pda_access_show,
		   raydium_i2c_pda_access_store);
/* I2C pda mode via pda2(R/W)
 *  example:    cat raydium_i2c_pda_access ==> read pda address provided by the
 *		following cmd
 *		echo ADDRinHEX [DATAinHEX] > raydium_i2c_pda_access ==> write
 *		pda address [data]
 */
static DEVICE_ATTR(raydium_i2c_pda_access_via_pda2, 0644,
		   raydium_i2c_pda_access_via_pda2_show,
		   raydium_i2c_pda_access_via_pda2_store);

/* I2C pda2 mode (R/W)
 *  example:    cat raydium_i2c_pda2_access ==> read pda2 address provided by
 *		the following cmd
 *		echo ADDRinHEX [DATAinHEX] > raydium_i2c_pda2_access ==>
 *		write pda2 address [data]
 */
static DEVICE_ATTR(raydium_i2c_pda2_access, 0644,
		   raydium_i2c_pda2_access_show,
		   raydium_i2c_pda2_access_store);

/* I2C pda2 mode page (W)
 *  example:    echo PAGEinHEX > raydium_i2c_pda2_page ==> write pda2 page
 */
static DEVICE_ATTR(raydium_i2c_pda2_page, 0644,
		   NULL,
		   raydium_i2c_pda2_page_store);

/* I2C read/set FT raw data (R/W)
 *  example:    cat raydium_i2c_raw_data ==> read raw data with specific length
 *		of corresponding type provided by the following cmd
 *              echo DataTypeinHEX RawDataLengthinHEX > raydium_i2c_raw_data
 *		==> set raw data type and its length
 */
static DEVICE_ATTR(raydium_i2c_raw_data, 0644,
		   raydium_i2c_raw_data_show,
		   raydium_i2c_raw_data_store);

/* Touch lock (W)
 *  example:    echo 1 > raydium_i2c_touch_lock ==> enable touch lock
 *            echo 0 > raydium_i2c_touch_lock ==> disable touch lock
 */
static DEVICE_ATTR(raydium_i2c_touch_lock, 0644,
		   NULL,
		   raydium_touch_lock_store);

/* Log level (W)
 *  example:    echo 1 > raydium_log_level ==> modify log level
 */
static DEVICE_ATTR(raydium_log_level, 0644,
		   NULL,
		   raydium_log_level_store);
/* show the fw version (R)
 *  example:cat raydium_fw_version
 */
static DEVICE_ATTR(raydium_check_fw_version, 0644,
		   raydium_check_fw_version_show,
		   NULL);

/* show the driver version (R)
 *  example:cat raydium_check_driver_version
 */
static DEVICE_ATTR(raydium_check_driver_version, 0644,
		   raydium_check_driver_version_show,
		   NULL);
/* show the panel version (R)
 *  example:cat raydium_panel_version
 */
static DEVICE_ATTR(raydium_check_panel_version, 0644,
		   raydium_check_panel_version_show,
		   NULL);

static DEVICE_ATTR(raydium_hw_reset, 0644,
		   raydium_hw_reset_show,
		   NULL);

static DEVICE_ATTR(raydium_palm_status, 0644,
		   raydium_palm_status_show,
		   NULL);

static DEVICE_ATTR(raydium_receive_fw_control, 0644,
		   NULL,
		   raydium_receive_fw_store);

static DEVICE_ATTR(raydium_mem_setting, 0644,
		   NULL,
		   raydium_mem_store);

#ifdef RAD_SELFTEST

/* Read interrupt flag cmd (R)
 *  example:cat raydium_int_flag
 */
static DEVICE_ATTR(raydium_int_flag, 0644,
		   raydium_flag_show,
		   raydium_flag_store);

static DEVICE_ATTR(raydium_reset_control, 0644,
		   NULL,
		   raydium_reset_control_store);

static DEVICE_ATTR(raydium_irq_state, 0644,
		   raydium_irq_state_show,
		   NULL);
static DEVICE_ATTR(raydium_do_selftest, 0644,
		   raydium_selftest_show,
		   NULL);
#endif
/*add your attr in here*/
struct attribute *raydium_attributes[] = {
	&dev_attr_raydium_touch_calibration.attr,
	&dev_attr_raydium_check_i2c.attr,
	&dev_attr_raydium_i2c_pda2_mode.attr,
	&dev_attr_raydium_i2c_pda_access.attr,
	&dev_attr_raydium_i2c_pda_access_via_pda2.attr,
	&dev_attr_raydium_i2c_pda2_access.attr,
	&dev_attr_raydium_i2c_pda2_page.attr,
	&dev_attr_raydium_i2c_raw_data.attr,
	&dev_attr_raydium_i2c_touch_lock.attr,
	&dev_attr_raydium_fw_upgrade.attr,
	&dev_attr_raydium_check_fw_version.attr,
	&dev_attr_raydium_check_panel_version.attr,
	&dev_attr_raydium_hw_reset.attr,
	&dev_attr_raydium_palm_status.attr,
	&dev_attr_raydium_check_driver_version.attr,
	&dev_attr_raydium_receive_fw_control.attr,
	&dev_attr_raydium_mem_setting.attr,
	&dev_attr_raydium_log_level.attr,
#ifdef RAD_SELFTEST
	&dev_attr_raydium_irq_state.attr,
	&dev_attr_raydium_int_flag.attr,
	&dev_attr_raydium_reset_control.attr,
	&dev_attr_raydium_do_selftest.attr,
#endif
	NULL
};
