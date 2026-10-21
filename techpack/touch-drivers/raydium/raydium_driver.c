/* raydium_driver.c
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
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/kprobes.h>
#include <asm/traps.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/of_device.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif /*end of CONFIG_FB*/

#include "raydium_driver.h"

struct raydium_slot_status {
	unsigned char pt_id;      /*Occupied point ID*/
	unsigned char need_update;         /*Mark as info need to be updated*/
	unsigned char pt_report_offset;    /*point info offset in report*/
};
/*The first 3 elements are currently occupied. therest is new coming points*/
struct raydium_slot_status gst_slot[MAX_TOUCH_NUM * 2];
struct raydium_slot_status gst_slot_init = {0xFF, 0, 0};

static int raydium_enable_regulator(struct raydium_ts_data *cd, bool en);


#if (defined(CONFIG_RM_SYSFS_DEBUG))
const struct attribute_group raydium_attr_group;
#endif /*end of CONFIG_RM_SYSFS_DEBUG*/

#if defined(CONFIG_DRM)
	static struct drm_panel *active_panel;
#endif

unsigned char g_u8_addr;
unsigned char g_u8_raydium_flag;
unsigned char g_u8_i2c_mode;
unsigned char g_u8_upgrade_type;
unsigned char g_u8_raw_data_type;
unsigned int g_u32_raw_data_len;    /* 128 bytes*/
unsigned long g_u32_addr;
unsigned int g_u32_length;
unsigned int g_u32_driver_version;
unsigned char *g_rad_fw_image, *g_rad_init_image;
unsigned char *g_rad_boot_image, *g_rad_para_image;
unsigned char *g_rad_testfw_image, *g_rad_testpara_image;
unsigned char g_u8_table_setting, g_u8_table_init;
unsigned char g_u8_resetflag;
unsigned char g_u8_wakeup_flag;
#ifdef ESD_SOLUTION_EN
unsigned char g_u8_checkflag;
#endif
unsigned char g_u8_log_level;
struct raydium_ts_data *g_raydium_ts;
/*******************************************************************************
 *  Name: raydium_variable_init
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *******************************************************************************/
static void raydium_variable_init(void)
{
	g_u8_addr = RAYDIUM_PDA2_PDA_CFG_ADDR;
	g_u8_raydium_flag = NORMAL_MODE;
	g_u8_i2c_mode = PDA2_MODE;
	g_u8_upgrade_type = 0;
	g_u8_raw_data_type = RAYDIUM_FT_UPDATE;
	g_u32_raw_data_len = 64 * 2;    /* 128 bytes*/
	g_u32_addr = RAYDIUM_CHK_I2C_CMD;
	g_u32_length = 1;
	g_u8_table_setting = 0;
	g_u8_table_init = 0;
	g_rad_fw_image = NULL;
	g_rad_init_image = NULL;
	g_rad_boot_image = NULL;
	g_rad_para_image = NULL;
	g_rad_testfw_image = NULL;
	g_rad_testpara_image = NULL;
	g_u32_driver_version = ((RAD_MAIN_VERSION <<  24) |
				(RAD_MINOR_VERSION << 16) |
				(RAD_CUSTOMER_VERSION));
	g_u8_resetflag = false;
	g_u8_wakeup_flag = false;
#ifdef ESD_SOLUTION_EN
	g_u8_checkflag = false;
#endif
	g_u8_log_level = LOG_INFO;
}


/*******************************************************************************
 *  Name: raydium_gpio_configure
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *******************************************************************************/

static int raydium_gpio_configure(bool on)
{
	int i32_err = 0;

	if (on) {
		if (gpio_is_valid(g_raydium_ts->irq_gpio)) {
			i32_err = gpio_request(g_raydium_ts->irq_gpio,
					       "raydium_irq_gpio");
			if (i32_err) {
				LOGD(LOG_ERR, "[touch]irq gpio request failed");
				goto err_irq_gpio_req;
			}

			i32_err = gpio_direction_input(g_raydium_ts->irq_gpio);
			if (i32_err) {
				LOGD(LOG_ERR, "[touch]set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}
		if (gpio_is_valid(g_raydium_ts->rst_gpio)) {
			i32_err = gpio_request(g_raydium_ts->rst_gpio,
					       "raydium_rst_gpio");
			if (i32_err) {
				LOGD(LOG_ERR,  "[touch]rst gpio request failed");
				goto err_irq_gpio_req;
			}

			i32_err = gpio_direction_output(g_raydium_ts->rst_gpio, 0);
			msleep(RAYDIUM_RESET_INTERVAL_10MSEC);
			if (i32_err) {
				LOGD(LOG_ERR,
				     "[touch]set_direction for rst gpio failed\n");
				goto err_rst_gpio_dir;
			}

			i32_err = gpio_direction_output(g_raydium_ts->rst_gpio, 1);
			if (i32_err) {
				LOGD(LOG_ERR,
				     "[touch]set_direction for irq gpio failed\n");
				goto err_rst_gpio_dir;
			}
		}
	} else {
		if (gpio_is_valid(g_raydium_ts->irq_gpio))
			gpio_free(g_raydium_ts->irq_gpio);
	}
	return 0;
err_rst_gpio_dir:
	if (gpio_is_valid(g_raydium_ts->rst_gpio))
		gpio_free(g_raydium_ts->rst_gpio);
	return i32_err;
err_irq_gpio_dir:
	if (gpio_is_valid(g_raydium_ts->irq_gpio))
		gpio_free(g_raydium_ts->irq_gpio);
err_irq_gpio_req:
	return i32_err;
}

/*******************************************************************************
 *  Name: raydium_ts_pinctrl_init
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *******************************************************************************/
#ifdef MSM_NEW_VER
static int raydium_ts_pinctrl_init(void)
{
	int i32_ret;

	/* Get pinctrl if target uses pinctrl */
	g_raydium_ts->ts_pinctrl = devm_pinctrl_get(&(g_raydium_ts->client->dev));
	if (IS_ERR_OR_NULL(g_raydium_ts->ts_pinctrl)) {
		i32_ret = PTR_ERR(g_raydium_ts->ts_pinctrl);
		LOGD(LOG_ERR, "[touch]target does not use pinctrl %d\n", i32_ret);
		goto err_pinctrl_get;
	}

	g_raydium_ts->pinctrl_state_active
		= pinctrl_lookup_state(g_raydium_ts->ts_pinctrl, PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(g_raydium_ts->pinctrl_state_active)) {
		i32_ret = PTR_ERR(g_raydium_ts->pinctrl_state_active);
		LOGD(LOG_ERR, "[touch]Can not lookup %s pinstate %d\n",
		     PINCTRL_STATE_ACTIVE, i32_ret);
		goto err_pinctrl_lookup;
	}

	g_raydium_ts->pinctrl_state_suspend
		= pinctrl_lookup_state(g_raydium_ts->ts_pinctrl,
				       PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(g_raydium_ts->pinctrl_state_suspend)) {
		i32_ret = PTR_ERR(g_raydium_ts->pinctrl_state_suspend);
		LOGD(LOG_ERR, "[touch]Can not lookup %s pinstate %d\n",
		     PINCTRL_STATE_SUSPEND, i32_ret);
		goto err_pinctrl_lookup;
	}

	g_raydium_ts->pinctrl_state_release
		= pinctrl_lookup_state(g_raydium_ts->ts_pinctrl,
				       PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(g_raydium_ts->pinctrl_state_release)) {
		i32_ret = PTR_ERR(g_raydium_ts->pinctrl_state_release);
		LOGD(LOG_ERR, "[touch]Can not lookup %s pinstate %d\n",
		     PINCTRL_STATE_RELEASE, i32_ret);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(g_raydium_ts->ts_pinctrl);
err_pinctrl_get:
	g_raydium_ts->ts_pinctrl = NULL;
	return i32_ret;
}
#endif/*end of MSM_NEW_VER*/
#ifdef ESD_SOLUTION_EN
static int raydium_hw_reset_fun(struct i2c_client *client)
{
	int i32_ret = SUCCESS;

	LOGD(LOG_INFO, "[touch]HW reset\n");
	g_u8_raydium_flag |= ENG_MODE;

	g_u8_resetflag = true;
	/*HW reset*/
	gpio_set_value(g_raydium_ts->rst_gpio, 1);
	gpio_set_value(g_raydium_ts->rst_gpio, 0);
	msleep(RAYDIUM_RESET_INTERVAL_MSEC);
	gpio_set_value(g_raydium_ts->rst_gpio, 1);

	g_u8_i2c_mode = PDA2_MODE;

	i32_ret = wait_irq_state(client, 300, 2000);
	if (i32_ret != ERROR)
		msleep(35);

	g_u8_raydium_flag &= ~ENG_MODE;

	LOGD(LOG_INFO, "[touch]Raydium HW reset : %d\n", i32_ret);
	return i32_ret;
}
#endif

int raydium_i2c_write_pda_via_pda2(struct i2c_client *client,
				   unsigned int u32_addr, unsigned char *u8_w_data,
				   unsigned short u16_length)
{
	int i32_ret = -1;
	/*unsigned char u8_retry;*/
	unsigned char u8_mode = 0x00;
	unsigned char u8_buf[MAX_WRITE_PACKET_SIZE + 6];

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = u16_length + 6,
			.buf = u8_buf,
		},
	};
	if (u16_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;


	/*I2C access for register need to word mode*/

	if ((u16_length == 4) &&
	    ((u32_addr & 0x50000000) || (u32_addr & 0x40000000)))
		u8_mode = I2C_PDA2_WORD_MODE;
	else
		u8_mode = I2C_PDA2_BYTE_MODE;

	if (u16_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;
	g_u8_i2c_mode = PDA2_MODE;

	u8_buf[0] = RAYDIUM_I2C_PDA_CMD;
	u8_buf[1] = (unsigned char)u32_addr;
	u8_buf[2] = (unsigned char)(u32_addr >> 8);
	u8_buf[3] = (unsigned char)(u32_addr >> 16);
	u8_buf[4] = (unsigned char)(u32_addr >> 24);
	u8_buf[5] = u8_mode;

	memcpy(&u8_buf[6], u8_w_data, u16_length);

	/*for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {*/
	if (i2c_transfer(client->adapter, msg, 1) == 1) {
		i32_ret = u16_length;
		/*break;*/
	}
	i32_ret = u16_length;
	/*usleep_range(500, 1500);*/
	/*}*/

	/*if (u8_retry == SYN_I2C_RETRY_TIMES) {*/
	/*	LOGD(LOG_ERR, "[touch]%s: I2C write over retry limit\n", __func__);*/
	/*	i32_ret = -EIO;*/
	/*}*/

	return i32_ret;

}
int raydium_i2c_read_pda_via_pda2(struct i2c_client *client,
				  unsigned int u32_addr, unsigned char *u8_r_data,
				  unsigned short u16_length)
{
	int i32_ret;
	/*unsigned char u8_retry;*/
	unsigned char u8_mode = 0x00;
	unsigned char u8_buf[6];

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = 6,
			.buf = u8_buf,
		},
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_READ,
			.len = u16_length,
			.buf = u8_r_data,
		},
	};

	if ((u32_addr & 0x50000000) || (u32_addr & 0x40000000))
		u8_mode = I2C_PDA2_WORD_MODE;
	else
		u8_mode = I2C_PDA2_BYTE_MODE;

	g_u8_i2c_mode = PDA2_MODE;
	u8_buf[0] = RAYDIUM_I2C_PDA_CMD;
	u8_buf[1] = (unsigned char)u32_addr;
	u8_buf[2] = (unsigned char)(u32_addr >> 8);
	u8_buf[3] = (unsigned char)(u32_addr >> 16);
	u8_buf[4] = (unsigned char)(u32_addr >> 24);
	u8_buf[5] = u8_mode;

	/*for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {*/
	if (i2c_transfer(g_raydium_ts->client->adapter, msg, 2) == 2) {
		i32_ret = u16_length;
		/*break;*/
	}
	i32_ret = u16_length;
	/*usleep_range(500, 1500);*/
	/*}*/

	/*if (u8_retry == SYN_I2C_RETRY_TIMES) {*/
	/*	LOGD(LOG_ERR, "[touch]%s: I2C read over retry limit\n", __func__);*/
	/*	i32_ret = -EIO;*/
	/*}*/

	return i32_ret;
}
int raydium_i2c_pda_set_address(unsigned int u32_address,
				unsigned char u8_mode)
{
	int i32_ret = 0;
	unsigned char u8_retry;
	unsigned char u8_buf[RAD_I2C_PDA_ADDRESS_LENGTH];
	struct i2c_client *client = g_raydium_ts->client;

	client->addr = RAYDIUM_I2C_EID;
	u8_buf[0] = (u32_address & 0x0000FF00) >> 8;
	u8_buf[1] = (u32_address & 0x00FF0000) >> 16;
	u8_buf[2] = (u32_address & 0xFF000000) >> 24;
	u8_buf[3] = u8_mode;

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		i32_ret = i2c_master_send(client, u8_buf,
					  RAD_I2C_PDA_ADDRESS_LENGTH);
		if (i32_ret != RAD_I2C_PDA_ADDRESS_LENGTH) {
			LOGD(LOG_ERR, "[touch]%s: I2C retry %d\n",
			     __func__, u8_retry + 1);
			usleep_range(500, 1500);
		} else {
			break;
		}
	}

	return (i32_ret == RAD_I2C_PDA_ADDRESS_LENGTH) ? i32_ret : -EIO;
}

/*device attribute raydium_i2c_pda2_mode used*/
int raydium_i2c_pda_read(struct i2c_client *client,
			 unsigned int u32_addr, unsigned char *u8_r_data,
			 unsigned short u16_length)
{
	int i32_ret;
	unsigned char u8_retry;
	unsigned char u8_mode = 0x00;
	unsigned char u8_buf;

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = 1,
			.buf = &u8_buf,
		},
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_READ,
			.len = u16_length,
			.buf = u8_r_data,
		},
	};

	if (u16_length == 4)
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			   RAD_I2C_PDA_2_MODE_DISABLE |
			   RAD_I2C_PDA_MODE_WORD_MODE;
	else
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			   RAD_I2C_PDA_2_MODE_DISABLE;

	u8_mode |= 0x03;

	u8_buf = u32_addr & MASK_8BIT;

	i32_ret = raydium_i2c_pda_set_address(u32_addr, u8_mode);
	if (i32_ret != RAD_I2C_PDA_ADDRESS_LENGTH)
		goto exit;
	usleep_range(50, 80);

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(g_raydium_ts->client->adapter, msg, 2) == 2) {
			i32_ret = u16_length;
			break;
		}
		LOGD(LOG_ERR, "%s: I2C retry %d\n", __func__, u8_retry + 1);
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		LOGD(LOG_ERR, "%s: I2C read over retry limit\n", __func__);
		i32_ret = -EIO;
	}
exit:
	return i32_ret;
}

int raydium_i2c_pda_write(struct i2c_client *client,
			  unsigned int u32_addr, unsigned char *u8_w_data,
			  unsigned short u16_length)
{
	int i32_ret;
	unsigned char u8_retry;
	unsigned char u8_mode = 0x00;
	unsigned char u8_buf[MAX_WRITE_PACKET_SIZE + 1];

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = u16_length + 1,
			.buf = u8_buf,
		},
	};

	if (u16_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;

	if (u16_length == 4)
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			   RAD_I2C_PDA_2_MODE_DISABLE |
			   RAD_I2C_PDA_MODE_WORD_MODE;
	else
		u8_mode |= RAD_I2C_PDA_MODE_ENABLE |
			   RAD_I2C_PDA_2_MODE_DISABLE;

	u8_buf[0] = u32_addr & MASK_8BIT;
	memcpy(&u8_buf[1], u8_w_data, u16_length);

	i32_ret = raydium_i2c_pda_set_address(u32_addr, u8_mode);
	if (i32_ret != RAD_I2C_PDA_ADDRESS_LENGTH)
		goto exit;
	usleep_range(50, 80);

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			i32_ret = u16_length;
			break;
		}
		LOGD(LOG_ERR, "[touch]%s: I2C retry %d\n", __func__, u8_retry + 1);
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		LOGD(LOG_ERR, "[touch]%s: I2C write over retry limit\n", __func__);
		i32_ret = -EIO;
	}
exit:
	return i32_ret;
}

int handle_i2c_pda_read(struct i2c_client *client,
			unsigned int u32_addr, unsigned char *u8_r_data,
			unsigned short u16_length)
{
	if ((g_u8_i2c_mode & PDA_MODE) != 0)  {
		if (raydium_i2c_pda_read(client, u32_addr, u8_r_data, u16_length) == ERROR) {
			LOGD(LOG_ERR, "[touch] handle_ic_write I2C NG!\r\n");
			return ERROR;
		}
	} else {
		if (raydium_i2c_read_pda_via_pda2(client, u32_addr, u8_r_data, u16_length) == ERROR) {
			LOGD(LOG_ERR, "[touch] handle_ic_write I2C via_pda2 NG!\r\n");
			return ERROR;
		}
	}
	return SUCCESS;
}

int handle_i2c_pda_write(struct i2c_client *client,
			 unsigned int u32_addr, unsigned char *u8_w_data,
			 unsigned short u16_length)
{
	if ((g_u8_i2c_mode & PDA_MODE) != 0)  {
		if (raydium_i2c_pda_write(client, u32_addr, u8_w_data, u16_length) == ERROR) {
			LOGD(LOG_ERR, "[touch] handle_ic_write I2C NG!\r\n");
			return ERROR;
		}
	} else {
		if (raydium_i2c_write_pda_via_pda2(client, u32_addr, u8_w_data, u16_length) == ERROR) {
			LOGD(LOG_ERR, "[touch] handle_ic_write I2C via_pda2 NG!\r\n");
			return ERROR;
		}
	}
	return SUCCESS;
}
int raydium_i2c_pda2_set_page(struct i2c_client *client,
			      unsigned int is_suspend,
			      unsigned char u8_page)
{
	int i32_ret = -1;
	unsigned char u8_retry;
	unsigned int u8_ret = (is_suspend) ? 10 : 2;
	unsigned char u8_buf[RAYDIUM_I2C_PDA2_PAGE_LENGTH];

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = RAYDIUM_I2C_PDA2_PAGE_LENGTH,
			.buf = u8_buf,
		},
	};

	u8_buf[0] = RAYDIUM_PDA2_PAGE_ADDR;
	u8_buf[1] = u8_page;
	for (; u8_ret > 0; u8_ret--) {
		for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
			if (i2c_transfer(client->adapter, msg, 1) == 1) {
				i32_ret = RAYDIUM_I2C_PDA2_PAGE_LENGTH;
				break;
			}
			usleep_range(500, 1500);
		}
		if (i32_ret == RAYDIUM_I2C_PDA2_PAGE_LENGTH)
			break;
		usleep_range(2000, 5000);
	}

	if (u8_ret == 0) {
		LOGD(LOG_ERR, "[touch]%s: I2C write over retry limit\n", __func__);
		i32_ret = -EIO;
	}

	return i32_ret;
}

int raydium_i2c_pda2_read(struct i2c_client *client,
			  unsigned char u8_addr,
			  unsigned char *u8_r_data,
			  unsigned short u16_length)
{
	int i32_ret = -1;
	unsigned char u8_retry;

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = 1,
			.buf = &u8_addr,
		},
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_READ,
			.len = u16_length,
			.buf = u8_r_data,
		},
	};
	g_u8_i2c_mode = PDA2_MODE;
	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(g_raydium_ts->client->adapter, msg, 2) == 2) {
			i32_ret = u16_length;
			break;
		}
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		LOGD(LOG_ERR, "[touch]%s: I2C read over retry limit\n", __func__);
		i32_ret = -EIO;
	}

	return i32_ret;
}

int raydium_i2c_pda2_write(struct i2c_client *client,
			   unsigned char u8_addr,
			   unsigned char *u8_w_data,
			   unsigned short u16_length)
{
	int i32_ret = -1;
	unsigned char u8_retry;
	unsigned char u8_buf[MAX_WRITE_PACKET_SIZE + 1];

	struct i2c_msg msg[] = {
		{
			.addr = RAYDIUM_I2C_NID,
			.flags = RAYDIUM_I2C_WRITE,
			.len = u16_length + 1,
			.buf = u8_buf,
		},
	};

	if (u16_length > MAX_WRITE_PACKET_SIZE)
		return -EINVAL;
	g_u8_i2c_mode = PDA2_MODE;
	u8_buf[0] = u8_addr;
	memcpy(&u8_buf[1], u8_w_data, u16_length);

	for (u8_retry = 0; u8_retry < SYN_I2C_RETRY_TIMES; u8_retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			i32_ret = u16_length;
			break;
		}
		usleep_range(500, 1500);
	}

	if (u8_retry == SYN_I2C_RETRY_TIMES) {
		LOGD(LOG_ERR, "[touch]%s: I2C write over retry limit\n", __func__);
		i32_ret = -EIO;
	}

	return i32_ret;
}

void raydium_irq_control(bool enable)
{
	if (enable) {
		if (g_raydium_ts->irq_enabled) {
			/*mutex_unlock(&ts->lock);*/
			LOGD(LOG_INFO, "[touch]Already enable irq\n");
			return;
		}

		/* Clear interrupts first */
		if (g_raydium_ts->blank != DRM_PANEL_BLANK_POWERDOWN) {
			if (g_u8_i2c_mode == PDA2_MODE) {
				mutex_lock(&g_raydium_ts->lock);
				if (raydium_i2c_pda2_set_page(g_raydium_ts->client,
								g_raydium_ts->is_suspend,
								RAYDIUM_PDA2_PAGE_0) < 0)
					LOGD(LOG_ERR, "[touch]set page fail%s\n", __func__);
				mutex_unlock(&g_raydium_ts->lock);
				usleep_range(500, 1500);
			}
		}
		while (g_raydium_ts->irq_desc->depth > 0) {
			LOGD(LOG_INFO, "[touch]irq enable\n");
			g_raydium_ts->irq_enabled = true;
			enable_irq(g_raydium_ts->irq);
		}
	} else {
		if (g_raydium_ts->irq_enabled) {
			if (g_raydium_ts->irq_desc->depth == 0) {
				disable_irq(g_raydium_ts->irq);
				g_raydium_ts->irq_enabled = false;
				LOGD(LOG_INFO, "[touch]irq disable\n");
			}
		}
	}
}

unsigned char raydium_disable_i2c_deglitch(void)
{
	unsigned int u32_buf = 0;
	unsigned char u8_retry = 3, u8_comfirm_time = 3;
	unsigned char u8_check = 0, u8_i = 0;
	unsigned int u32_i2c_deglitch = 0x07060000;
	unsigned char u8_buf[4];

	while (u8_retry--) {
		u32_buf = 0;
		handle_i2c_pda_read(g_raydium_ts->client, RAYDIUM_CHK_I2C_CMD,
				    (unsigned char *)(&u32_buf), 4);
		if ((u32_buf & 0xFFFF0000) == 0xF3030000)
			u8_check++;
	}
	if (u8_check == 3) {
		LOGD(LOG_INFO, "[touch]PDA2 OK\r\n");
		return SUCCESS;
	}

	g_u8_i2c_mode = PDA_MODE;
	u8_retry = 100;
	while (u8_retry--) {
		u8_check = 0;
		for (u8_i = 0; u8_i < u8_comfirm_time; u8_i++) {
			/*check I2C*/
			u32_buf = 0;
			if (handle_i2c_pda_read(g_raydium_ts->client,
						RAYDIUM_PDA_I2CENG,
						(unsigned char *)(&u32_buf), 4) == ERROR) {
				LOGD(LOG_ERR, "[touch]%s: 1.handle_ic_read I2C NG!\r\n", __func__);
				break;
			}

			if (u32_buf == u32_i2c_deglitch)
				u8_check++;
			else
				break;
		}

		if (u8_check == u8_comfirm_time)
			break;

		if (handle_i2c_pda_write(g_raydium_ts->client, RAYDIUM_PDA_I2CENG,
					 (unsigned char *)(&u32_i2c_deglitch), 4) == ERROR) {
			LOGD(LOG_ERR, "[touch]%s:handle_ic_write I2C NG!\r\n", __func__);
			continue;
		}

		u8_check = 0;
		for (u8_i = 0; u8_i < u8_comfirm_time; u8_i++) {
			/*check I2C*/
			u32_buf = 0;
			if (handle_i2c_pda_read(g_raydium_ts->client,
						RAYDIUM_PDA_I2CENG,
						(unsigned char *)(&u32_buf), 4) == ERROR) {
				LOGD(LOG_ERR, "[touch]%s:2.handle_ic_read I2C NG!\r\n", __func__);
				break;
			}

			if (u32_buf == u32_i2c_deglitch)
				u8_check++;
			else
				break;
		}

		if (u8_check == u8_comfirm_time)
			break;
	}

	if (u8_retry == 0)
		return ERROR;

	u32_buf = 0x03;
	if (handle_i2c_pda_write(g_raydium_ts->client, RAYDIUM_REG_GPIO_DEGLITCH,
				 (unsigned char *)(&u32_buf), 4) == ERROR) {
		LOGD(LOG_ERR, "[touch]%s:3.handle_ic_write I2C NG!\r\n", __func__);
		return ERROR;
	}

	/*Disable PDA*/
	handle_i2c_pda_read(g_raydium_ts->client, RAYDIUM_PDA_I2CREG, u8_buf, 4);
	u8_buf[0] |= RAD_ENABLE_PDA2 | RAD_ENABLE_SI2;
	handle_i2c_pda_write(g_raydium_ts->client, RAYDIUM_PDA_I2CREG, u8_buf, 4);
	raydium_i2c_pda_set_address(0x50000628, DISABLE);

	g_u8_i2c_mode = PDA2_MODE;

	return SUCCESS;
}

#ifdef CONFIG_RM_SYSFS_DEBUG

int raydium_i2c_mode_control(struct i2c_client *client,
			     unsigned char u8_mode)
{
	unsigned char u8_buf[4];

	switch (u8_mode) {
	case 0:	/* Disable INT flag */
		LOGD(LOG_INFO, "[touch]RAD INT flag : %d\n", g_raydium_ts->irq_enabled);
		disable_irq(g_raydium_ts->irq);
		g_raydium_ts->irq_enabled = false;
		LOGD(LOG_INFO, "[touch]RAD irq disable\n");
		break;
	case 1:	/* Enable INT flag */
		LOGD(LOG_INFO, "[touch]RAD INT flag : %d\n", g_raydium_ts->irq_enabled);
		enable_irq(g_raydium_ts->irq);
		g_raydium_ts->irq_enabled = true;
		LOGD(LOG_INFO, "[touch]RAD irq enable\n");
		break;
	case 2: /* Disable INT by raydium_irq_control */
		raydium_irq_control(DISABLE);
		break;
	case 3: /* Enable INT by raydium_irq_control */
		raydium_irq_control(ENABLE);
		break;
	case 4: /* Show RAD INT depth */
		LOGD(LOG_INFO, "[touch]RAD INT depth : %d\n", g_raydium_ts->irq_desc->depth);
		break;
	case 7:
		raydium_i2c_pda2_set_page(client,
					  g_raydium_ts->is_suspend, RAYDIUM_PDA2_2_PDA);
		g_u8_i2c_mode = PDA_MODE;
		LOGD(LOG_INFO, "[touch]Disable PDA2_MODE\n");
		break;
	case 8:
		raydium_i2c_pda_read(client, RAYDIUM_PDA_I2CREG, u8_buf, 4);
		u8_buf[0] |= RAD_ENABLE_PDA2 | RAD_ENABLE_SI2;
		raydium_i2c_pda_write(client, RAYDIUM_PDA_I2CREG, u8_buf, 4);
		raydium_i2c_pda_set_address(RAYDIUM_PDA_I2CREG, DISABLE);

		g_u8_i2c_mode = PDA2_MODE;
		LOGD(LOG_INFO, "[touch]Enable PDA2_MODE\n");
		break;
	}
	return 0;
}


const struct attribute_group raydium_attr_group = {
	.attrs = raydium_attributes
};

/*create sysfs for debug update firmware*/
static int raydium_create_sysfs(struct i2c_client *client)
{
	int ret = -1;

	ret = sysfs_create_group(&(client->dev.kobj), &raydium_attr_group);
	if (ret) {
		LOGD(LOG_ERR, "[touch]failed to register sysfs\n");
		sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
		ret = -EIO;
	} else {
		LOGD(LOG_DEBUG, "[touch]create raydium sysfs attr_group successful\n");
	}
	return ret;
}

static void raydium_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &raydium_attr_group);
}
#endif /*end of CONFIG_RM_SYSFS_DEBUG*/

#ifdef ESD_SOLUTION_EN
int raydium_esd_check(void)
{
	int i32_ret = 0;
	unsigned char u8_esd_status[MAX_TCH_STATUS_PACKET_SIZE];

	mutex_lock(&g_raydium_ts->lock);
	if (g_u8_i2c_mode == PDA2_MODE) {
		i32_ret = raydium_i2c_pda2_set_page(g_raydium_ts->client,
						    g_raydium_ts->is_suspend,
						    RAYDIUM_PDA2_PAGE_0);
		if (i32_ret < 0)
			goto exit;
		/*read esd status*/
		i32_ret = raydium_i2c_pda2_read(g_raydium_ts->client,
						RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR,
						u8_esd_status, MAX_TCH_STATUS_PACKET_SIZE);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]%s: failed to read data: %d\n",
			     __func__, __LINE__);
			goto exit;
		}

		if (u8_esd_status[POS_FW_STATE] != 0x1A &&
		    u8_esd_status[POS_FW_STATE] != 0xAA) {
			if (g_u8_resetflag == true) {
				LOGD(LOG_ERR, "[touch]%s -> filter abnormal irq\n"
				     , __func__);
				goto exit;
			}
			LOGD(LOG_ERR, "[touch]%s -> abnormal irq, FW state = 0x%x\n",
			     __func__, u8_esd_status[POS_FW_STATE]);
			g_u8_resetflag = false;
			i32_ret = -1;
			goto exit;

		}
		g_u8_resetflag = false;
	}
exit:
	mutex_unlock(&g_raydium_ts->lock);
	LOGD(LOG_INFO, "[touch]%s\n", __func__);
	return i32_ret;
}
#endif




static int raydium_touch_report(unsigned char *p_u8_buf,
				unsigned char u8_points_amount)
{
	unsigned char u8_i, u8_j, u8_offset = 0, u8_pt_id;
	signed short i16_wx, i16_wy;
	/* number of touch points */
	unsigned char u8_touch_count = 0;
	//DECLARE_BITMAP(ids, g_raydium_ts->u8_max_touchs);
	unsigned long *ids = NULL;

	ids = kzalloc(sizeof(*ids)*BITS_TO_LONGS(g_raydium_ts->u8_max_touchs), GFP_KERNEL);
	if (!ids)
		return -ENOMEM;

	bitmap_zero(ids, g_raydium_ts->u8_max_touchs);

	for (u8_i = 0; u8_i < (g_raydium_ts->u8_max_touchs * 2); u8_i++) {
		gst_slot[u8_i].need_update = 0;
		gst_slot[u8_i].pt_report_offset = 0;
	}

	/*Check incoming point info*/
	for (u8_i = 0; u8_i < u8_points_amount; u8_i++) {
		u8_pt_id = p_u8_buf[POS_PT_ID + u8_i * LEN_PT];
		/* Current*/
		for (u8_j = 0; u8_j < g_raydium_ts->u8_max_touchs; u8_j++) {
			if (u8_pt_id == gst_slot[u8_j].pt_id) {
				gst_slot[u8_j].need_update = 1;
				gst_slot[u8_j].pt_report_offset = u8_i;
				break;
			}
		}
		/* New coming*/
		if (u8_j == g_raydium_ts->u8_max_touchs) {
			for (u8_j = g_raydium_ts->u8_max_touchs;
			     u8_j < (g_raydium_ts->u8_max_touchs * 2); u8_j++) {
				if (!gst_slot[u8_j].need_update) {
					gst_slot[u8_j].pt_id = u8_pt_id;
					gst_slot[u8_j].need_update = 1;
					gst_slot[u8_j].pt_report_offset = u8_i;
					LOGD(LOG_DEBUG, "[touch]x:%d,y:%d\n",
					     p_u8_buf[POS_X_L + u8_offset] |
					     p_u8_buf[POS_X_H + u8_offset] << 8,
					     p_u8_buf[POS_Y_L + u8_offset] |
					     p_u8_buf[POS_Y_H + u8_offset] << 8);
					break;
				}
			}
		}
	}

	/*Release slot with non-occupied point*/
	for (u8_i = 0; u8_i < g_raydium_ts->u8_max_touchs; u8_i++) {
		if (!gst_slot[u8_i].need_update) {
			input_mt_slot(g_raydium_ts->input_dev, u8_i);
			input_mt_report_slot_state(g_raydium_ts->input_dev,
						   MT_TOOL_FINGER, false);
			gst_slot[u8_i].pt_id = 0xFF;
			gst_slot[u8_i].pt_report_offset = 0;
			gst_slot[u8_i].need_update = 0;
		}
	}
	/*Assign new one to non-occupied slot*/
	for (u8_i = g_raydium_ts->u8_max_touchs;
	     u8_i < (g_raydium_ts->u8_max_touchs * 2); u8_i++) {
		if (gst_slot[u8_i].need_update) {
			for (u8_j = 0; u8_j < g_raydium_ts->u8_max_touchs; u8_j++) {
				if (!gst_slot[u8_j].need_update) {
					gst_slot[u8_j] = gst_slot[u8_i];
					gst_slot[u8_i] = gst_slot_init;
					break;
				}
			}
		} else {
			break;
		}
	}
	for (u8_i = 0; u8_i < g_raydium_ts->u8_max_touchs; u8_i++) {
		if (gst_slot[u8_i].need_update) {
			u8_offset = gst_slot[u8_i].pt_report_offset * LEN_PT;
			g_raydium_ts->x_pos[u8_i] = p_u8_buf[POS_X_L + u8_offset] |
						    p_u8_buf[POS_X_H + u8_offset] << BYTE_SHIFT;
			g_raydium_ts->y_pos[u8_i] = p_u8_buf[POS_Y_L + u8_offset] |
						    p_u8_buf[POS_Y_H + u8_offset] << BYTE_SHIFT;
			g_raydium_ts->pressure = p_u8_buf[POS_PRESSURE_L + u8_offset] |
						 p_u8_buf[POS_PRESSURE_H + u8_offset] << BYTE_SHIFT;
			i16_wx = p_u8_buf[POS_WX_L + u8_offset] |
				 p_u8_buf[POS_WX_H + u8_offset] << BYTE_SHIFT;
			i16_wy = p_u8_buf[POS_WY_L + u8_offset] |
				 p_u8_buf[POS_WY_H + u8_offset] << BYTE_SHIFT;

			input_mt_slot(g_raydium_ts->input_dev, u8_i);
			input_mt_report_slot_state(g_raydium_ts->input_dev,
						   MT_TOOL_FINGER, true);
			__set_bit(u8_i, ids);

			input_report_abs(g_raydium_ts->input_dev,
					 ABS_MT_POSITION_X, g_raydium_ts->x_pos[u8_i]);
			input_report_abs(g_raydium_ts->input_dev,
					 ABS_MT_POSITION_Y, g_raydium_ts->y_pos[u8_i]);
			input_report_abs(g_raydium_ts->input_dev,
					 ABS_MT_PRESSURE, g_raydium_ts->pressure);
			input_report_abs(g_raydium_ts->input_dev,
					 ABS_MT_TOUCH_MAJOR, max(i16_wx, i16_wy));
			input_report_abs(g_raydium_ts->input_dev,
					 ABS_MT_TOUCH_MINOR, min(i16_wx, i16_wy));
			LOGD(LOG_DEBUG, "[touch:%d]x:%d,y:%d\n",
			     u8_i,
			     p_u8_buf[POS_X_L + u8_offset] |
			     p_u8_buf[POS_X_H + u8_offset] << 8,
			     p_u8_buf[POS_Y_L + u8_offset] |
			     p_u8_buf[POS_Y_H + u8_offset] << 8);
			u8_touch_count++;
		}
	}
	input_report_key(g_raydium_ts->input_dev,
			 BTN_TOUCH, u8_touch_count > 0);
	input_report_key(g_raydium_ts->input_dev,
			 BTN_TOOL_FINGER, u8_touch_count > 0);

	for (u8_i = 0; u8_i < g_raydium_ts->u8_max_touchs; u8_i++) {
		if (test_bit(u8_i, ids))
			continue;
		input_mt_slot(g_raydium_ts->input_dev, u8_i);
		input_mt_report_slot_state(g_raydium_ts->input_dev,
					   MT_TOOL_FINGER, false);
	}

	input_sync(g_raydium_ts->input_dev);
	kfree(ids);

	return 0;
}

int raydium_read_touchdata(unsigned char *p_u8_tp_status,  unsigned char *p_u8_buf)
{
	int i32_ret = 0;
	unsigned char u8_points_amount;
	static unsigned char u8_seq_no;
	unsigned char u8_retry;
	unsigned char u8_read_size;
	unsigned char u8_read_buf[MAX_REPORT_PACKET_SIZE];
	u8_retry = 3;

	mutex_lock(&g_raydium_ts->lock);
	while (u8_retry != 0) {
		i32_ret = raydium_i2c_pda2_set_page(g_raydium_ts->client,
						    g_raydium_ts->is_suspend, RAYDIUM_PDA2_PAGE_0);
		if (i32_ret < 0) {
			msleep(250);
			u8_retry--;
		} else
			break;
	}
	if (u8_retry == 0) {
		LOGD(LOG_ERR, "[touch]%s: failed to set page\n", __func__);

		goto reset_error;
	}

	memset(u8_read_buf, 0, MAX_REPORT_PACKET_SIZE);
	memset(p_u8_buf, 0, MAX_REPORT_PACKET_SIZE);
	memset(p_u8_tp_status, 0, MAX_TCH_STATUS_PACKET_SIZE);
	u8_read_size = 4 + MAX_TOUCH_NUM * LEN_PT + 1;
	/*read touch point information*/
	i32_ret = raydium_i2c_pda2_read(g_raydium_ts->client,
					RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR,
					u8_read_buf, u8_read_size);
	if (i32_ret < 0) {
		LOGD(LOG_ERR, "[touch]%s: failed to read data: %d\n",
		     __func__, __LINE__);
		goto exit_error;
	}
	memcpy(p_u8_tp_status, &u8_read_buf[0], MAX_TCH_STATUS_PACKET_SIZE);

#ifdef ESD_SOLUTION_EN
	if (p_u8_tp_status[POS_FW_STATE] != 0x1A &&
	    p_u8_tp_status[POS_FW_STATE] != 0xAA) {
		if (g_u8_resetflag == true) {
			LOGD(LOG_ERR, "[touch]%s -> filter irq, FW state = 0x%x\n",
			     __func__, p_u8_tp_status[POS_FW_STATE]);
			i32_ret = -1;
			g_u8_resetflag = false;
			goto exit_error;
		}
		LOGD(LOG_ERR, "[touch]%s -> abnormal irq, FW state = 0x%x\n",
		     __func__, p_u8_tp_status[POS_FW_STATE]);
		i32_ret = -1;
		goto reset_error;

	}
#endif
	/* inform IC to prepare next report*/
	if (u8_seq_no == p_u8_tp_status[POS_SEQ]) {
		p_u8_tp_status[POS_SEQ] = 0;
		i32_ret = raydium_i2c_pda2_write(g_raydium_ts->client,
						 RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, p_u8_tp_status, 1);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]%s: write data failed: %d\n", __func__, i32_ret);
			goto exit_error;
		}
		LOGD(LOG_DEBUG, "[touch]%s -> report not updated.\n", __func__);
		goto exit_error;
	}
	u8_seq_no = p_u8_tp_status[POS_SEQ];
	p_u8_tp_status[POS_SEQ] = 0;
	i32_ret = raydium_i2c_pda2_write(g_raydium_ts->client,
					 RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, p_u8_tp_status, 1);
	if (i32_ret < 0) {
		LOGD(LOG_ERR, "[touch]%s: write data failed: %d\n", __func__, i32_ret);
		goto exit_error;
	}

	u8_points_amount = p_u8_tp_status[POS_PT_AMOUNT];

	if (u8_points_amount > MAX_TOUCH_NUM)
		goto exit_error;
	memcpy(p_u8_buf, &u8_read_buf[4], u8_points_amount * LEN_PT);

	raydium_touch_report(p_u8_buf, u8_points_amount);

exit_error:
	mutex_unlock(&g_raydium_ts->lock);

	return i32_ret;

reset_error:
	mutex_unlock(&g_raydium_ts->lock);
#ifdef ESD_SOLUTION_EN
	u8_retry = 3;
	while (u8_retry != 0) {
		i32_ret = raydium_hw_reset_fun(g_raydium_ts->client);
		LOGD(LOG_ERR, "[touch]%s: HW reset\n", __func__);
		if (i32_ret < 0) {
			msleep(100);
			u8_retry--;
		} else
			break;
	}
#endif
	return i32_ret;
}

static void raydium_work_handler(struct work_struct *work)
{
	int i32_ret = 0;
	unsigned char u8_tp_status[MAX_TCH_STATUS_PACKET_SIZE];
	unsigned char u8_buf[MAX_REPORT_PACKET_SIZE];

#ifdef GESTURE_EN
	unsigned char u8_i;
	LOGD(LOG_DEBUG, "[touch]ts->blank:%x, g_u8_i2c_mode:%x\n",
			g_raydium_ts->blank, g_u8_i2c_mode);
	LOGD(LOG_DEBUG, "[touch]u8_tp_status:%x, g_raydium_ts->is_palm:%x\n",
			u8_tp_status[POS_GES_STATUS], g_raydium_ts->is_palm);

	if (g_u8_i2c_mode == PDA2_MODE) {
		i32_ret = raydium_read_touchdata(u8_tp_status, u8_buf);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]%s, read_touchdata error, ret:%d\n",
			     __func__, i32_ret);
			return;
		}
	}

	if (g_raydium_ts->blank == DRM_PANEL_BLANK_LP ||
	g_raydium_ts->blank == DRM_PANEL_BLANK_POWERDOWN || g_raydium_ts->fb_state == FB_OFF) {
		LOGD(LOG_DEBUG, "[touch] elseif u8_tp_status:%x\n", u8_tp_status[POS_GES_STATUS]);
		/*need check small area*/
		/*if (u8_tp_status[POS_GES_STATUS] == RAD_WAKE_UP */
		 /*&& g_u8_wakeup_flag == false) { */
		if (u8_tp_status[POS_GES_STATUS] == 0)	{
			input_report_key(g_raydium_ts->input_dev, KEY_WAKEUP, true);
			usleep_range(9500, 10500);
			input_sync(g_raydium_ts->input_dev);

			input_report_key(g_raydium_ts->input_dev, KEY_WAKEUP, false);
			input_sync(g_raydium_ts->input_dev);
			LOGD(LOG_DEBUG, "[touch]display wake up with g_u8_resetflag true\n");
			/*goto exit;*/
		}
	}
	/*when display on can use palm to suspend*/
	else if (g_raydium_ts->blank == DRM_PANEL_BLANK_UNBLANK) {
		if (u8_tp_status[POS_GES_STATUS] == RAD_PALM_ENABLE) {
			if (g_raydium_ts->is_palm == 0) {
				/* release all touches*/
				for (u8_i = 0; u8_i < g_raydium_ts->u8_max_touchs;
				     u8_i++) {
					input_mt_slot(g_raydium_ts->input_dev,
						      u8_i);
					input_mt_report_slot_state(
						g_raydium_ts->input_dev,
						MT_TOOL_FINGER, false);
				}
				input_mt_report_pointer_emulation(
					g_raydium_ts->input_dev,
					false);
				/*press sleep key*/
				input_report_key(g_raydium_ts->input_dev,
						 KEY_SLEEP, true);
				input_sync(g_raydium_ts->input_dev);

				LOGD(LOG_INFO, "[touch]palm_status = %d.\n",
				     u8_tp_status[POS_GES_STATUS]);

				g_raydium_ts->is_palm = 1;
				/*goto exit;*/
			}
		} else if ((u8_tp_status[POS_GES_STATUS]
			    == RAD_PALM_DISABLE)
			   && (g_raydium_ts->is_palm == 1)) {
			LOGD(LOG_INFO, "[touch]leave palm mode.\n");
			input_report_key(g_raydium_ts->input_dev,
					 KEY_SLEEP, false);
			input_sync(g_raydium_ts->input_dev);

			/*raydium_irq_control(raydium_ts, DISABLE);*/
			g_raydium_ts->is_palm = 0;
			/*goto exit;*/
		}
	}
#else
	if (g_u8_i2c_mode == PDA2_MODE) {
		i32_ret = raydium_read_touchdata(u8_tp_status, u8_buf);
		if (i32_ret < 0) {
			LOGD(LOG_ERR, "[touch]%s, read_touchdata error, ret:%d\n",
			     __func__, i32_ret);
		}
	}
#endif
}


/*The raydium device will signal the host about TRIGGER_FALLING.
 *Processed when the interrupt is asserted.
 */
static irqreturn_t raydium_ts_interrupt(int irq, void *dev_id)
{
	bool result = false;

	LOGD(LOG_DEBUG, "[touch]%s\n", __func__);

	/*For bootloader wrt/erase flash and software reset interrupt*/
	if ((g_u8_raydium_flag & ENG_MODE) != 0) {
		LOGD(LOG_INFO, "[touch]RAD_ENG_MODE\n");
		g_u8_raydium_flag |= INT_FLAG;
	} else {
		if (!work_pending(&g_raydium_ts->work)) {
			/* Clear interrupts*/
			result = queue_work(g_raydium_ts->workqueue,
					    &g_raydium_ts->work);

			if (!result) {
				/*queue_work fail*/
				LOGD(LOG_ERR, "[touch]queue_work fail.\n");
			}


		} else {
			/*work pending*/
			mutex_lock(&g_raydium_ts->lock);
			if (raydium_i2c_pda2_set_page(g_raydium_ts->client,
						      g_raydium_ts->is_suspend,
						      RAYDIUM_PDA2_PAGE_0) < 0) {

				LOGD(LOG_ERR, "[touch]%s: failed to set page in work_pending\n",
				     __func__);
			}
			mutex_unlock(&g_raydium_ts->lock);

			LOGD(LOG_DEBUG, "[touch]work_pending\n");
		}
	}
	return IRQ_HANDLED;
}

static int raydium_check_i2c_ready(unsigned short *u16_i2c_data)
{
	unsigned char u8_buf[4];
	int i32_ret = ERROR;

	mutex_lock(&g_raydium_ts->lock);

	if (g_u8_i2c_mode == PDA2_MODE) {
		i32_ret = handle_i2c_pda_read(g_raydium_ts->client,
					      RAYDIUM_CHK_I2C_CMD, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_error;

		if (u8_buf[3] != 0xF3) {
			LOGD(LOG_ERR, "[touch]PDA2 read i2c fail\n");
			g_u8_i2c_mode = PDA_MODE;
			i32_ret = handle_i2c_pda_read(g_raydium_ts->client,
						      RAYDIUM_CHK_I2C_CMD, u8_buf, 4);
			if (i32_ret < 0)
				goto exit_error;
		}
	} else {
		i32_ret = handle_i2c_pda_read(g_raydium_ts->client,
					      RAYDIUM_CHK_I2C_CMD, u8_buf, 4);
		if (i32_ret < 0)
			goto exit_error;

	}

	*u16_i2c_data = u8_buf[3] << 8 | u8_buf[2];

	LOGD(LOG_DEBUG, "[touch]RAD check I2C : 0x%02X%02X\n", u8_buf[3], u8_buf[2]);

exit_error:
	mutex_unlock(&g_raydium_ts->lock);
	return i32_ret;
}

#if defined(CONFIG_PM)
static void raydium_ts_do_suspend(void)
{
	unsigned char u8_i = 0;

	LOGD(LOG_INFO, "[touch]%s.\n", __func__);

	if (g_u8_raw_data_type == 0)
		g_u8_resetflag = false;
	if (g_raydium_ts->is_suspend == 1) {
		LOGD(LOG_WARNING, "[touch]Already in suspend state\n");
		return;
	}
	/*#ifndef GESTURE_EN*/
	raydium_irq_control(DISABLE);
	/*#endif*/

	/*clear workqueue*/
	if (!cancel_work_sync(&g_raydium_ts->work))
		LOGD(LOG_DEBUG, "[touch]workqueue is empty!\n");

	/* release all touches */
	for (u8_i = 0; u8_i < g_raydium_ts->u8_max_touchs; u8_i++) {
		input_mt_slot(g_raydium_ts->input_dev, u8_i);
		input_mt_report_slot_state(g_raydium_ts->input_dev,
					   MT_TOOL_FINGER,
					   false);
	}
	input_mt_report_pointer_emulation(g_raydium_ts->input_dev, false);
	input_sync(g_raydium_ts->input_dev);

#ifdef GESTURE_EN
	if (device_may_wakeup(&g_raydium_ts->client->dev)) {
		LOGD(LOG_INFO, "[touch]Device may wakeup\n");
		if (!enable_irq_wake(g_raydium_ts->irq))
			g_raydium_ts->irq_wake = true;

	} else {
		LOGD(LOG_INFO, "[touch]Device not wakeup\n");
	}
	raydium_irq_control(ENABLE);
#endif

	g_raydium_ts->is_suspend = 1;
}

static void raydium_ts_do_resume(void)
{
#ifdef ESD_SOLUTION_EN
	int i32_ret = 0;
	unsigned char u8_retry = 0;
#endif

	LOGD(LOG_INFO, "[touch]%s, %d.\n", __func__, g_raydium_ts->is_suspend);
	if (g_raydium_ts->is_suspend == 0) {
		LOGD(LOG_WARNING, "[touch]Already in resume state\n");
		return;
	}

	/* clear interrupts*/
	mutex_lock(&g_raydium_ts->lock);
	if (raydium_i2c_pda2_set_page(g_raydium_ts->client,
				      g_raydium_ts->is_suspend, RAYDIUM_PDA2_PAGE_0) < 0) {
		LOGD(LOG_ERR, "[touch]%s: failed to set page\n", __func__);
		mutex_unlock(&g_raydium_ts->lock);
		return;
	}
	mutex_unlock(&g_raydium_ts->lock);

	/* clear workqueue*/
	if (!cancel_work_sync(&g_raydium_ts->work))
		LOGD(LOG_DEBUG, "[ raydium ]workqueue is empty!\n");
#ifdef ESD_SOLUTION_EN
	if (g_u8_checkflag == true) {
		i32_ret = raydium_esd_check();
		if (i32_ret < 0) {
			u8_retry = 3;
			while (u8_retry != 0) {
				i32_ret = raydium_hw_reset_fun(g_raydium_ts->client);
				if (i32_ret < 0) {
					msleep(100);
					u8_retry--;
				} else
					break;
			}

		}
		g_u8_checkflag = false;
	}
#endif
	raydium_irq_control(ENABLE);
#ifdef GESTURE_EN
	if (device_may_wakeup(&g_raydium_ts->client->dev)) {
		LOGD(LOG_INFO, "[touch]Device may wakeup\n");
		if (g_raydium_ts->irq_wake) {
			disable_irq_wake(g_raydium_ts->irq);
			g_raydium_ts->irq_wake = false;
		}
	} else
		LOGD(LOG_INFO, "[touch]Device not wakeup\n");
#endif

	g_raydium_ts->is_suspend = 0;
}

static int raydium_ts_suspend(struct device *dev)
{
	raydium_ts_do_suspend();
	return 0;
}

static int raydium_ts_resume(struct device *dev)
{
	raydium_ts_do_resume();
	return 0;
}


static const struct dev_pm_ops raydium_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend	= raydium_ts_suspend,
	.resume		= raydium_ts_resume,
#endif /*end of CONFIG_PM*/
};

/*used for touch lock feature*/
static int raydium_ts_open(struct input_dev *input_dev)
{
	//int i32_ret = 0;

	LOGD(LOG_DEBUG, "[touch]%s()+\n", __func__);

	LOGD(LOG_DEBUG, "[touch]ts->blank:%x\n", g_raydium_ts->blank);

	if (g_raydium_ts->is_sleep == 1) {
		mutex_lock(&g_raydium_ts->lock);
		if (gpio_is_valid(g_raydium_ts->rst_gpio)) {

			g_u8_resetflag = true;
			gpio_set_value(g_raydium_ts->rst_gpio, 1);
			gpio_set_value(g_raydium_ts->rst_gpio, 0);
			msleep(RAYDIUM_RESET_INTERVAL_MSEC);/*5ms*/
			gpio_set_value(g_raydium_ts->rst_gpio, 1);
			msleep(RAYDIUM_RESET_DELAY_MSEC);/*100ms*/
			g_u8_i2c_mode = PDA2_MODE;
		}
		mutex_unlock(&g_raydium_ts->lock);
		raydium_irq_control(ENABLE);
		g_raydium_ts->is_sleep = 0;
		LOGD(LOG_INFO, "[touch]disable touch lock.\n");
	}
	//return i32_ret;
	return 0;
}

static void raydium_ts_close(struct input_dev *input_dev)
{
	int i32_ret = 0;
	unsigned char u8_i = 0;
	unsigned char u8_wbuffer[1];

	LOGD(LOG_INFO, "[touch]%s()+\n", __func__);

	if (g_raydium_ts->is_sleep == 1) {
		LOGD(LOG_INFO, "[touch]touch lock already enabled.\n");
		return;
	}

	raydium_irq_control(DISABLE);

	for (u8_i = 0; u8_i < g_raydium_ts->u8_max_touchs; u8_i++) {
		input_mt_slot(g_raydium_ts->input_dev, u8_i);
		input_mt_report_slot_state(g_raydium_ts->input_dev,
					   MT_TOOL_FINGER,
					   false);
	}
	input_mt_report_pointer_emulation(g_raydium_ts->input_dev, false);
	input_sync(g_raydium_ts->input_dev);
	mutex_lock(&g_raydium_ts->lock);
	i32_ret = raydium_i2c_pda2_set_page(g_raydium_ts->client,
					    g_raydium_ts->is_suspend, RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0) {
		LOGD(LOG_ERR, "[touch]ret:%d\n", i32_ret);
		goto exit_i2c_error;
	}
	u8_wbuffer[0] = RAYDIUM_HOST_CMD_PWR_SLEEP;
	i32_ret = raydium_i2c_pda2_write(g_raydium_ts->client,
					 RAYDIUM_PDA2_HOST_CMD_ADDR,
					 u8_wbuffer,
					 1);
	if (i32_ret < 0) {
		LOGD(LOG_ERR, "[touch]ret:%d\n", i32_ret);
		goto exit_i2c_error;
	}

	mutex_unlock(&g_raydium_ts->lock);
	g_raydium_ts->is_sleep = 1;
	LOGD(LOG_INFO, "[touch]enable touch lock.\n");
	return;

exit_i2c_error:
	mutex_unlock(&g_raydium_ts->lock);
	raydium_irq_control(ENABLE);
}

#else
static int raydium_ts_suspend(struct device *dev)
{
	return 0;
}

static int raydium_ts_resume(struct device *dev)
{
	return 0;
}
#endif /*end of CONFIG_FB*/


#if defined(CONFIG_DRM)
/*******************************************************************************
 * FUNCTION: drm_notifier_callback
 *
 * SUMMARY: Call back function for DRM notifier to allow to call
 * resume/suspend attention list.
 *
 * RETURN:
 *   0 = success
 *
 * PARAMETERS:
 * self   - pointer to notifier_block structure
 * event  - event type of fb notifier
 * data   - pointer to fb_event structure
 ******************************************************************************/
static int drm_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct raydium_ts_data *g_raydium_ts =
		container_of(self, struct raydium_ts_data, fb_notif);
	struct drm_panel_notifier *evdata = data;
	int *blank;

	LOGD(LOG_INFO, "%s: DRM notifier called!\n", __func__);

	if (!evdata)
		goto exit;

	if (!(event == DRM_PANEL_EARLY_EVENT_BLANK ||
			event == DRM_PANEL_EVENT_BLANK)) {
		LOGD(LOG_INFO, "%s: Event(%lu) do not need process\n",
			__func__, event);
		goto exit;
	}

	blank = evdata->data;
	g_raydium_ts->blank = (*blank);
	LOGD(LOG_INFO, "%s: DRM event:%lu,blank:%d fb_state %d ",
		__func__, event, *blank, g_raydium_ts->fb_state);
	LOGD(LOG_INFO, "%s: DRM Power - %s - FB state %d ",
		__func__, (*blank == DRM_PANEL_BLANK_UNBLANK)?"UP":"DOWN", g_raydium_ts->fb_state);

	if (*blank == DRM_PANEL_BLANK_UNBLANK) {
		LOGD(LOG_INFO, "%s: UNBLANK!\n", __func__);

		if (event == DRM_PANEL_EARLY_EVENT_BLANK) {
			LOGD(LOG_INFO, "%s: resume: event = %lu, not care\n",
				__func__, event);
		} else if (event == DRM_PANEL_EVENT_BLANK) {
			if (g_raydium_ts->fb_state != FB_ON) {
				LOGD(LOG_INFO, "%s: Resume notifier called!\n",
					__func__);
#ifdef GESTURE_EN
			/* clear palm status */
			g_raydium_ts->is_palm = 0;
#endif

#if defined(CONFIG_PM)
				raydium_ts_resume(&g_raydium_ts->client->dev);
#endif
				g_raydium_ts->fb_state = FB_ON;
			LOGD(LOG_INFO, "%s: Resume notified!\n", __func__);
			}
		}
	} else if (*blank == DRM_PANEL_BLANK_LP || *blank == DRM_PANEL_BLANK_POWERDOWN
		|| *blank == DRM_PANEL_BLANK_FPS_CHANGE) {
		LOGD(LOG_INFO, "%s: LOWPOWER!\n", __func__);
		if (event == DRM_PANEL_EARLY_EVENT_BLANK) {
			if (g_raydium_ts->fb_state != FB_OFF) {
				LOGD(LOG_INFO, "%s: Suspend notifier called!\n",
					__func__);
#ifdef GESTURE_EN
			/* clear palm status */
			g_raydium_ts->is_palm = 0;
#endif

#if defined(CONFIG_PM)
				raydium_ts_suspend(&g_raydium_ts->client->dev);
#endif
				g_raydium_ts->fb_state = FB_OFF;
				LOGD(LOG_INFO, "%s: Suspend notified!\n", __func__);
			}
		} else if (event == DRM_PANEL_EVENT_BLANK) {
			LOGD(LOG_INFO, "%s: suspend: event = %lu, not care\n",
				__func__, event);
		}
	} else {
		LOGD(LOG_INFO, "%s: DRM BLANK(%d) do not need process\n",
			__func__, *blank);
	}
exit:
	return 0;
}

/*******************************************************************************
 * FUNCTION: raydium_setup_drm_notifier
 *
 * SUMMARY: Set up call back function into drm notifier.
 *
 * PARAMETERS:
 * g_raydium_ts   - pointer to core data
 *******************************************************************************/
static void raydium_setup_drm_notifier(struct raydium_ts_data *g_raydium_ts)
{
	g_raydium_ts->fb_state = FB_ON;
	g_raydium_ts->fb_notif.notifier_call = drm_notifier_callback;
	LOGD(LOG_INFO, "[touch]%s: Setting up drm notifier\n", __func__);

	if (!active_panel)
		LOGD(LOG_ERR, "[touch]%s: Active panel not registered!\n", __func__);

	if (active_panel && drm_panel_notifier_register(active_panel,
		&g_raydium_ts->fb_notif) < 0)
		LOGD(LOG_ERR, "[touch]%s: Register notifier failed!\n", __func__);
}
#endif /*end of CONFIG_DRM*/

/*******************************************************************************
 * FUNCTION: raydium_setup_drm_notifier
 *
 * SUMMARY: Set up call back function into fb notifier.
 *
 * PARAMETERS:
 * g_raydium_ts   - pointer to core data
 *******************************************************************************/
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event,
				void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    g_raydium_ts && g_raydium_ts->client) {
		blank = evdata->data;
		g_raydium_ts->blank = (*blank);
		switch (*blank) {

		/*screen on*/
		case FB_BLANK_UNBLANK:
			LOGD(LOG_INFO, "[touch]FB_BLANK_UNBLANK\n");
#ifdef GESTURE_EN

			/* clear palm status */

			g_raydium_ts->is_palm = 0;
#endif
			raydium_ts_resume(&g_raydium_ts->client->dev);
			break;

		/*screen off*/
		case FB_BLANK_POWERDOWN:
			LOGD(LOG_INFO, "[touch]FB_BLANK_POWERDOWN\n");
#ifdef GESTURE_EN

			/* clear palm status */

			g_raydium_ts->is_palm = 0;
#endif
			raydium_ts_suspend(&g_raydium_ts->client->dev);
			break;

		/*ambient mode*/
		case FB_BLANK_VSYNC_SUSPEND:
			LOGD(LOG_INFO, "[touch]FB_BLANK_VSYNC_SUSPEND\n");
#ifdef GESTURE_EN

			/* clear palm status */

			g_raydium_ts->is_palm = 0;
#endif

			raydium_ts_suspend(&g_raydium_ts->client->dev);
			break;

		default:
			break;
		}
	}

	return 0;
}

static void raydium_register_notifier(void)
{
	memset(&g_raydium_ts->fb_notif, 0, sizeof(g_raydium_ts->fb_notif));
	g_raydium_ts->fb_notif.notifier_call = fb_notifier_callback;

	/* register on the fb notifier and work with fb*/
	if (fb_register_client(&g_raydium_ts->fb_notif))
		LOGD(LOG_ERR, "[touch]register notifier failed\n");
}

static void raydium_unregister_notifier(void)
{
	fb_unregister_client(&g_raydium_ts->fb_notif);
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void raydium_ts_early_suspend(struct early_suspend *handler)
{

	raydium_ts_do_suspend();
}

static void raydium_ts_late_resume(struct early_suspend *handler)
{
	raydium_ts_do_resume();
}
#endif /*end of CONFIG_FB*/

#ifdef CONFIG_OF
static int raydium_get_dt_coords(struct device *dev, char *name,
				 struct raydium_ts_platform_data *pdata)
{
	u32 coords[COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;

	if (!prop->value)
		return -ENODATA;


	coords_size = prop->length / sizeof(u32);
	if (coords_size != COORDS_ARR_SIZE) {
		LOGD(LOG_ERR, "[touch]invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		LOGD(LOG_ERR, "[touch]unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "raydium,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		LOGD(LOG_ERR, "[touch]unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

/*******************************************************************************
 * FUNCTION: raydium_check_dsi_panel_dt
 *
 * SUMMARY: Get the DSI active panel information from dtsi
 *
 * RETURN:
 *   0 = success
 *   !0 = fail
 *
 * PARAMETERS:
 * np            -  pointer to device_node structure
 * active_panel  -  name of active DSI panel
 ******************************************************************************/

static int raydium_check_dsi_panel_dt(struct device_node *np, struct drm_panel **active_panel)
{
	int i = 0, rc = 0;
	int count = 0;
	struct device_node *node = NULL;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "panel", NULL);
	LOGD(LOG_INFO, "[touch]%s: Active panel count: %d\n", __func__, count);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);

		if (node != NULL)
			LOGD(LOG_ERR, "[touch]%s: Node handle successfully parsed !\n", __func__);
		panel = of_drm_find_panel(node);
		of_node_put(node);

		if (!IS_ERR(panel)) {
			LOGD(LOG_ERR, "[touch]%s: Active panel selected !\n", __func__);
			*active_panel = panel;
			return 0;
		}
	}
	LOGD(LOG_ERR, "[touch]%s: Active panel NOT selected !\n", __func__);
	rc = PTR_ERR(panel);
	return rc;
}

static int raydium_parse_dt(struct device *dev,
			    struct raydium_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct drm_panel *active_panel = NULL;
	int rc = 0;
	u32 temp_val = 0;

	pdata->name = RAYDIUM_NAME;

	rc = raydium_get_dt_coords(dev, "raydium,display-coords", pdata);
	if (rc)
		return rc;


	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np,
			    "raydium,reset-gpio",
			    0,
			    &pdata->reset_gpio_flags);
	//if (pdata->reset_gpio < 0)
	if ((s32)(pdata->reset_gpio) < 0)
		return pdata->reset_gpio;


	pdata->irq_gpio = of_get_named_gpio_flags(np,
			  "raydium,irq-gpio",
			  0,
			  &pdata->irq_gpio_flags);
	//if (pdata->irq_gpio < 0)
	if ((s32)(pdata->irq_gpio) < 0)
		return pdata->irq_gpio;

	rc = raydium_check_dsi_panel_dt(np, &active_panel);
	LOGD(LOG_ERR, "[touch]%s: Panel not selected, rc=%d\n", __func__, rc);
	if (rc) {
		LOGD(LOG_ERR, "[touch]%s: Panel not selected, rc=%d\n", __func__, rc);
		if (rc == -EPROBE_DEFER) {
			LOGD(LOG_ERR, "[touch]%s: Probe defer selected, rc=%d\n", __func__, rc);
			return rc;
		}
	}
	pdata->active_panel = active_panel;
	LOGD(LOG_ERR, "[touch]%s: Successful insert of active panel in core data\n", __func__);

	rc = of_property_read_u32(np,
				  "raydium,hard-reset-delay-ms", &temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;


	rc = of_property_read_u32(np,
				  "raydium,soft-reset-delay-ms", &temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;


	rc = of_property_read_u32(np, "raydium,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;
#ifdef FW_MAPPING_BYID_EN
	rc = of_property_read_u32(np, "raydium,fw_id", &temp_val);
	if (!rc)
		pdata->fw_id = temp_val;
	else
		return rc;
#endif
	return 0;
}
#else
static int raydium_parse_dt(struct device *dev,
			    struct raydium_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif /*end of CONFIG_OF*/

static void raydium_input_set(struct input_dev *input_dev)
{
	int ret = 0;
	unsigned char i;

	input_dev->name = "raydium_ts";/*name need same with .idc*/
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &g_raydium_ts->client->dev;
	input_dev->open = raydium_ts_open;/*touch lock*/
	input_dev->close = raydium_ts_close;
	input_set_drvdata(input_dev, g_raydium_ts);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	LOGD(LOG_INFO, "[touch]set abs prarams x[%d], y[%d]\n",
	     g_raydium_ts->x_max, g_raydium_ts->y_max);

	/* Multitouch input params setup */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     g_raydium_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     g_raydium_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, WIDTH_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MINOR, 0, WIDTH_MAX, 0, 0);

	ret = input_mt_init_slots(input_dev, MAX_TOUCH_NUM,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		LOGD(LOG_ERR, "[touch]failed to initialize MT slots: %d\n", ret);

	for (i = 0; i < (MAX_TOUCH_NUM * 2); i++)
		gst_slot[i] = gst_slot_init;

}
static int raydium_set_resolution(void)
{
	unsigned char u8_buf[4];
	int i32_ret = -1;
	unsigned int u32_x, u32_y;

	mutex_lock(&g_raydium_ts->lock);

	i32_ret = raydium_i2c_pda2_set_page(g_raydium_ts->client,
					    g_raydium_ts->is_suspend,
					    RAYDIUM_PDA2_PAGE_0);
	if (i32_ret < 0)
		goto exit_error;

	i32_ret = raydium_i2c_pda2_read(g_raydium_ts->client,
					RAYDIUM_PDA2_DISPLAY_INFO_ADDR,
					u8_buf, 4);
	if (i32_ret < 0)
		goto exit_error;

	u32_x = u8_buf[3] << 8 | u8_buf[2];
	u32_y = u8_buf[1] << 8 | u8_buf[0];

	LOGD(LOG_DEBUG, "[touch]RAD display info x:%d, y:%d\n", u32_x, u32_y);

	if (u32_x > 100 && u32_y > 100 &&
	    u32_x < 600 && u32_y < 600) {
		g_raydium_ts->x_max = u32_x - 1;
		g_raydium_ts->y_max = u32_y - 1;
	}

exit_error:
	mutex_unlock(&g_raydium_ts->lock);
	return i32_ret;
}
static int raydium_get_regulator(struct raydium_ts_data *cd, bool get)
{
	int rc;

	if (!get) {
		rc = 0;
		goto regulator_put;
	}
#ifdef VDD_ANALOG_ENABLE
	cd->vdd = regulator_get(&cd->client->dev, "vdd");
	if (IS_ERR(cd->vdd)) {
		rc = PTR_ERR(cd->vdd);
		dev_err(&cd->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		goto regulator_put;
	}
#endif

	cd->vcc_i2c = regulator_get(&cd->client->dev, "vcc_i2c");
	if (IS_ERR(cd->vcc_i2c)) {
		rc = PTR_ERR(cd->vcc_i2c);
		dev_err(&cd->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto regulator_put;
	}

	return 0;

regulator_put:
#ifdef VDD_ANALOG_ENABLE
	if (!IS_ERR(cd->vdd)) {
		dev_err(&cd->client->dev, "Regulator put vdd\n");
		regulator_put(cd->vdd);
		cd->vdd = NULL;
	}
#endif

	if (!IS_ERR(cd->vcc_i2c)) {
		dev_err(&cd->client->dev, "Regulator put vcc_i2c\n");
		regulator_put(cd->vcc_i2c);
		cd->vcc_i2c = NULL;
	}

	return rc;
}

static int raydium_enable_regulator(struct raydium_ts_data *cd, bool en)
{
	int rc;

	if (!en) {
		rc = 0;
		goto disable_vcc_i2c_reg;
	}
#ifdef VDD_ANALOG_ENABLE
	if (cd->vdd) {
		if (regulator_count_voltages(cd->vdd) > 0) {
			rc = regulator_set_voltage(cd->vdd, VTG_MIN_UV,
						VTG_MAX_UV);
			if (rc) {
				dev_err(&cd->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
				goto exit;
			}
		}

		rc = regulator_enable(cd->vdd);
		if (rc) {
			dev_err(&cd->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto exit;
		}
	}
#endif

	if (cd->vcc_i2c) {
		if (regulator_count_voltages(cd->vcc_i2c) > 0) {
			rc = regulator_set_voltage(cd->vcc_i2c, I2C_VTG_MIN_UV,
							I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&cd->client->dev,
					"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				goto disable_vdd_reg;
			}
		}

		rc = regulator_enable(cd->vcc_i2c);
		if (rc) {
			dev_err(&cd->client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto disable_vdd_reg;
		}
	}

	return 0;

disable_vcc_i2c_reg:
	if (cd->vcc_i2c) {
		if (regulator_count_voltages(cd->vcc_i2c) > 0)
			regulator_set_voltage(cd->vcc_i2c, I2C_VTG_MIN_UV,
						I2C_VTG_MAX_UV);

		regulator_disable(cd->vcc_i2c);
	}

disable_vdd_reg:
#ifdef VDD_ANALOG_ENABLE
	if (cd->vdd) {
		if (regulator_count_voltages(cd->vdd) > 0)
			regulator_set_voltage(cd->vdd, VTG_MIN_UV,
						VTG_MAX_UV);

		regulator_disable(cd->vdd);
	}
#endif

#ifdef VDD_ANALOG_ENABLE
exit:
#endif
	return rc;
}

static int raydium_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct raydium_ts_platform_data *pdata =
		(struct raydium_ts_platform_data *)client->dev.platform_data;

	struct input_dev *input_dev;
	unsigned short u16_i2c_data;
	int ret = 0;

	LOGD(LOG_INFO, "[touch] probe\n");

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct raydium_ts_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			LOGD(LOG_ERR, "[touch]failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = raydium_parse_dt(&client->dev, pdata);
		if (ret) {
			LOGD(LOG_ERR, "[touch]device tree parsing failed\n");
			goto parse_dt_failed;
		}
	} else
		pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ret = -ENODEV;
		goto exit_check_functionality_failed;
	}

	g_raydium_ts = devm_kzalloc(&client->dev,
				    sizeof(struct raydium_ts_data),
				    GFP_KERNEL);
	if (!g_raydium_ts) {
		LOGD(LOG_ERR, "[touch]failed to allocate input driver data\n");
		return -ENOMEM;
	}

	raydium_variable_init();

	mutex_init(&g_raydium_ts->lock);

	i2c_set_clientdata(client, g_raydium_ts);
	g_raydium_ts->irq_enabled = false;
	g_raydium_ts->irq_wake = false;

	g_raydium_ts->irq_gpio = pdata->irq_gpio;
	g_raydium_ts->rst_gpio = pdata->reset_gpio;
	client->irq = g_raydium_ts->irq_gpio;
	g_raydium_ts->u8_max_touchs = pdata->num_max_touches;
	g_raydium_ts->client = client;
	g_raydium_ts->x_max = pdata->x_max - 1;
	g_raydium_ts->y_max = pdata->y_max - 1;
	g_raydium_ts->is_suspend = 0;
	g_raydium_ts->is_sleep = 0;


#ifdef GESTURE_EN
	g_raydium_ts->is_palm = 0;
#endif
	g_raydium_ts->fw_version = 0;
	device_init_wakeup(&client->dev, 1);

#ifdef MSM_NEW_VER
	ret = raydium_ts_pinctrl_init();
	if (!ret && g_raydium_ts->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret = pinctrl_select_state(g_raydium_ts->ts_pinctrl,
					   g_raydium_ts->pinctrl_state_active);
		if (ret < 0)
			LOGD(LOG_ERR, "[touch]failed to set pin to active state\n");
	}
#endif /*end of MSM_NEW_VER*/

	ret = raydium_get_regulator(g_raydium_ts, true);
	if (ret) {
		dev_err(&client->dev, "Failed to get voltage regulators\n");
		goto error_alloc_data;
	}

	ret = raydium_enable_regulator(g_raydium_ts, true);
	if (ret) {
		dev_err(&client->dev, "Failed to enable regulators: rc=%d\n", ret);
		goto error_get_regulator;
	}
	ret = raydium_gpio_configure(true);
	if (ret < 0) {
		LOGD(LOG_ERR, "[touch]failed to configure the gpios\n");
		goto err_gpio_req;
	}
	/*modify dtsi to 360*/
	msleep(pdata->soft_rst_dly);
	if (raydium_disable_i2c_deglitch() == ERROR) {
		LOGD(LOG_ERR, "[touch]disable i2c deglicth NG!\r\n");
		ret = -ENODEV;
		goto exit_check_i2c;
	}

	/*print touch i2c ready*/
	ret = raydium_check_i2c_ready(&u16_i2c_data);
	if (ret < 0 || (u16_i2c_data == 0)) {
		LOGD(LOG_ERR, "[touch]Check I2C failed\n");
		ret = -EPROBE_DEFER;
		goto exit_check_i2c;
	}
#ifdef CONFIG_DRM
	/* Setup active dsi panel */
	active_panel = pdata->active_panel;
#endif
	/*input device initialization*/
	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		LOGD(LOG_ERR, "[touch]failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	raydium_set_resolution();

	g_raydium_ts->input_dev = input_dev;
	raydium_input_set(input_dev);

	ret = input_register_device(input_dev);
	if (ret) {
		LOGD(LOG_ERR, "[touch]failed to register input device: %s\n",
		     dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef GESTURE_EN
	input_set_capability(input_dev, EV_KEY, KEY_SLEEP);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
#endif

	/*suspend/resume routine*/
#if defined(CONFIG_FB)
	raydium_register_notifier();
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	/*Early-suspend level*/
	g_raydium_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_raydium_ts->early_suspend.suspend = raydium_ts_early_suspend;
	g_raydium_ts->early_suspend.resume = raydium_ts_late_resume;
	register_early_suspend(&g_raydium_ts->early_suspend);
#endif/*end of CONFIG_FB*/

#ifdef CONFIG_RM_SYSFS_DEBUG
	raydium_create_sysfs(client);
#endif/*end of CONFIG_RM_SYSFS_DEBUG*/

	INIT_WORK(&g_raydium_ts->work, raydium_work_handler);

	g_raydium_ts->workqueue = create_singlethread_workqueue("raydium_ts");
	/*irq_gpio = 13 irqflags = 108*/
	LOGD(LOG_DEBUG, "[touch]pdata irq : %d\n", g_raydium_ts->irq_gpio);
	LOGD(LOG_DEBUG, "[touch]client irq : %d, pdata flags : %d\n",
	     client->irq, pdata->irqflags);

#if defined(CONFIG_DRM)
	LOGD(LOG_DEBUG, "%s: Probe: Setup drm notifier\n", __func__);
	raydium_setup_drm_notifier(g_raydium_ts);
#endif/*end of CONFIG_DRM*/

	g_raydium_ts->irq = gpio_to_irq(pdata->irq_gpio);
	ret = request_threaded_irq(g_raydium_ts->irq, NULL, raydium_ts_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				   client->dev.driver->name, g_raydium_ts);
	if (ret < 0) {
		LOGD(LOG_ERR, "[touch]raydium_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	g_raydium_ts->irq_desc = irq_to_desc(g_raydium_ts->irq);
	g_raydium_ts->irq_enabled = true;

	/*disable_irq then enable_irq for avoid Unbalanced enable for IRQ */

	/*raydium_irq_control(ts, ENABLE);*/

	LOGD(LOG_DEBUG, "[touch]RAD Touch driver ver :0x%X\n", g_u32_driver_version);

	/*fw update check*/
	ret = raydium_fw_update_init(u16_i2c_data);
	if (ret < 0) {
		LOGD(LOG_ERR, "[touch]FW update init failed\n");
		ret = -ENODEV;
		goto exit_irq_request_failed;
	}

	LOGD(LOG_INFO, "[touch] probe: done\n");
	return 0;

exit_irq_request_failed:
#if defined(CONFIG_FB)
	raydium_unregister_notifier();
#endif/*end of CONFIG_FB*/

	cancel_work_sync(&g_raydium_ts->work);
	input_unregister_device(input_dev);

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
exit_check_i2c:
#ifdef MSM_NEW_VER
	if (g_raydium_ts->ts_pinctrl) {
		if (IS_ERR_OR_NULL(g_raydium_ts->pinctrl_state_release)) {
			devm_pinctrl_put(g_raydium_ts->ts_pinctrl);
			g_raydium_ts->ts_pinctrl = NULL;
		} else {
			if (pinctrl_select_state(g_raydium_ts->ts_pinctrl,
					g_raydium_ts->pinctrl_state_release))
				LOGD(LOG_ERR,
				    "[touch]pinctrl_select_state failed\n");
		}
	}
#endif/*end of MSM_NEW_VER*/

	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);

	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);

err_gpio_req:
	raydium_enable_regulator(g_raydium_ts, false);

error_get_regulator:
	raydium_get_regulator(g_raydium_ts, false);
error_alloc_data:
parse_dt_failed:
exit_check_functionality_failed:
	return ret;

}


void raydium_ts_shutdown(struct i2c_client *client)
{

#if defined(CONFIG_FB)
	raydium_unregister_notifier();
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&g_raydium_ts->early_suspend);
#elif defined(CONFIG_DRM)
	if (active_panel)
		drm_panel_notifier_unregister(active_panel, &g_raydium_ts->fb_notifier);
#endif/*end of CONFIG_FB*/
	input_unregister_device(g_raydium_ts->input_dev);
	input_free_device(g_raydium_ts->input_dev);
	gpio_free(g_raydium_ts->rst_gpio);

#ifdef CONFIG_RM_SYSFS_DEBUG
	raydium_release_sysfs(client);
#endif /*end of CONFIG_RM_SYSFS_DEBUG*/

	free_irq(client->irq, g_raydium_ts);

	if (gpio_is_valid(g_raydium_ts->rst_gpio))
		gpio_free(g_raydium_ts->rst_gpio);

	if (gpio_is_valid(g_raydium_ts->irq_gpio))
		gpio_free(g_raydium_ts->irq_gpio);

	cancel_work_sync(&g_raydium_ts->work);
	destroy_workqueue(g_raydium_ts->workqueue);

	raydium_enable_regulator(g_raydium_ts, false);
	raydium_get_regulator(g_raydium_ts, false);

	i2c_set_clientdata(client, NULL);
}

static int raydium_ts_remove(struct i2c_client *client)
{

#if defined(CONFIG_FB)
	raydium_unregister_notifier();
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&g_raydium_ts->early_suspend);
#elif defined(CONFIG_DRM)
	if (active_panel)
		drm_panel_notifier_unregister(active_panel, &g_raydium_ts->fb_notifier);
#endif/*end of CONFIG_FB*/
	input_unregister_device(g_raydium_ts->input_dev);
	input_free_device(g_raydium_ts->input_dev);
	gpio_free(g_raydium_ts->rst_gpio);

#ifdef CONFIG_RM_SYSFS_DEBUG
	raydium_release_sysfs(client);
#endif /*end of CONFIG_RM_SYSFS_DEBUG*/

	free_irq(client->irq, g_raydium_ts);

	if (gpio_is_valid(g_raydium_ts->rst_gpio))
		gpio_free(g_raydium_ts->rst_gpio);

	if (gpio_is_valid(g_raydium_ts->irq_gpio))
		gpio_free(g_raydium_ts->irq_gpio);

	cancel_work_sync(&g_raydium_ts->work);
	destroy_workqueue(g_raydium_ts->workqueue);

	raydium_enable_regulator(g_raydium_ts, false);
	raydium_get_regulator(g_raydium_ts, false);

	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id raydium_ts_id[] = {
	{RAYDIUM_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, raydium_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id raydium_match_table[] = {
	{ .compatible = "raydium,raydium-ts",},
	{ },
};
#else
#define raydium_match_table NULL
#endif/*end of CONFIG_OF*/

static struct i2c_driver raydium_ts_driver = {
	.probe = raydium_ts_probe,
	.remove = raydium_ts_remove,
	.shutdown = raydium_ts_shutdown,
	.id_table = raydium_ts_id,
	.driver = {
		.name = RAYDIUM_NAME,
		.owner = THIS_MODULE,
		.of_match_table = raydium_match_table,
#if defined(CONFIG_PM)
		.pm    = &raydium_ts_pm_ops,
#endif/*end of CONFIG_PM*/
	},
};

static int __init raydium_ts_init(void)
{
	int ret;

	ret = i2c_add_driver(&raydium_ts_driver);
	return ret;
}

static void __exit raydium_ts_exit(void)
{
	i2c_del_driver(&raydium_ts_driver);
}

module_init(raydium_ts_init);
module_exit(raydium_ts_exit);

MODULE_AUTHOR("<Rejion>");
MODULE_DESCRIPTION("Raydium TouchScreen driver");
MODULE_LICENSE("GPL");
