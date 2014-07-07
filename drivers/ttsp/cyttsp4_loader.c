/*
 * cyttsp4_loader.c
 * Cypress TrueTouch(TM) Standard Product V4 FW loader module.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/cyttsp4_bus.h>
#include <linux/cyttsp4_core.h>
#include "cyttsp4_regs.h"

#define CYTTSP4_LOADER_NAME "cyttsp4_loader"
#define CYTTSP4_AUTO_LOAD_FOR_CORRUPTED_FW 1

static const u8 cyttsp4_security_key[] = {
	0xA5, 0x01, 0x02, 0x03, 0xFF, 0xFE, 0xFD, 0x5A
};
#define CY_MAX_NUM_CORE_DEVS				5
#define CY_CMD_I2C_ADDR					0
	/* Timeout value in ms. */
#define CY_CMD_TIMEOUT					500
	/* Timeout in ms. */
#define CY_CMD_LDR_INIT_TIMEOUT				10000
#define CY_STATUS_SIZE_BYTE				1
#define CY_STATUS_TYP_DELAY				2
#define CY_CMD_TAIL_LEN					3
#define CY_CMD_BYTE					1
#define CY_STATUS_BYTE					1
#define CY_MAX_STATUS_SIZE				32
#define CY_MIN_STATUS_SIZE				5
#define CY_START_OF_PACKET				0x01
#define CY_END_OF_PACKET				0x17
#define CY_DATA_ROW_SIZE				288
#define CY_DATA_ROW_SIZE_TMA400				128
#define CY_PACKET_DATA_LEN				96
#define CY_MAX_PACKET_LEN				512
#define CY_COMM_BUSY					0xFF
#define CY_CMD_BUSY					0xFE
#define CY_SEPARATOR_OFFSET				0
#define CY_ARRAY_ID_OFFSET				0
#define CY_ROW_NUM_OFFSET				1
#define CY_ROW_SIZE_OFFSET				3
#define CY_ROW_DATA_OFFSET				5
#define CY_FILE_SILICON_ID_OFFSET			0
#define CY_FILE_REV_ID_OFFSET				4
#define CY_CMD_LDR_HOST_SYNC				0xFF /* tma400 */
#define CY_CMD_LDR_EXIT					0x3B
#define CY_CMD_LDR_EXIT_CMD_SIZE			7
#define CY_CMD_LDR_EXIT_STAT_SIZE			7
#define CY_CMD_LDR_ENTER				0x38
#define CY_CMD_LDR_ENTER_CMD_SIZE			7
#define CY_CMD_LDR_ENTER_STAT_SIZE			15
#define CY_CMD_LDR_INIT					0x48
#define CY_CMD_LDR_INIT_CMD_SIZE			15
#define CY_CMD_LDR_INIT_STAT_SIZE			7
#define CY_CMD_LDR_ERASE_ROW				0x34
#define CY_CMD_LDR_ERASE_ROW_CMD_SIZE			10
#define CY_CMD_LDR_ERASE_ROW_STAT_SIZE			7
#define CY_CMD_LDR_SEND_DATA				0x37
#define CY_CMD_LDR_SEND_DATA_CMD_SIZE			4 /* hdr bytes only */
#define CY_CMD_LDR_SEND_DATA_STAT_SIZE			8
#define CY_CMD_LDR_PROG_ROW				0x39
#define CY_CMD_LDR_PROG_ROW_CMD_SIZE			7 /* hdr bytes only */
#define CY_CMD_LDR_PROG_ROW_STAT_SIZE			7
#define CY_CMD_LDR_VERIFY_ROW				0x3A
#define CY_CMD_LDR_VERIFY_ROW_STAT_SIZE			8
#define CY_CMD_LDR_VERIFY_ROW_CMD_SIZE			10
#define CY_CMD_LDR_VERIFY_CHKSUM			0x31
#define CY_CMD_LDR_VERIFY_CHKSUM_CMD_SIZE		7
#define CY_CMD_LDR_VERIFY_CHKSUM_STAT_SIZE		8

struct cyttsp4_loader_data {
	struct cyttsp4_device *ttsp;
	struct cyttsp4_sysinfo *si;
	u8 status_buf[CY_MAX_STATUS_SIZE];
	struct completion int_running;
};

struct cyttsp4_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

enum ldr_status {
	ERROR_SUCCESS = 0,
	ERROR_COMMAND = 1,
	ERROR_FLASH_ARRAY = 2,
	ERROR_PACKET_DATA = 3,
	ERROR_PACKET_LEN = 4,
	ERROR_PACKET_CHECKSUM = 5,
	ERROR_FLASH_PROTECTION = 6,
	ERROR_FLASH_CHECKSUM = 7,
	ERROR_VERIFY_IMAGE = 8,
	ERROR_UKNOWN1 = 9,
	ERROR_UKNOWN2 = 10,
	ERROR_UKNOWN3 = 11,
	ERROR_UKNOWN4 = 12,
	ERROR_UKNOWN5 = 13,
	ERROR_UKNOWN6 = 14,
	ERROR_INVALID_COMMAND = 15,
	ERROR_INVALID
};

static u16 _cyttsp4_compute_crc(struct cyttsp4_device *ttsp, u8 *buf, int size)
{
	u16 crc = 0xffff;
	u16 tmp;
	int i;

	if (size == 0)
		crc = ~crc;
	else {

		do {
			for (i = 0, tmp = 0x00ff & *buf++; i < 8;
				i++, tmp >>= 1) {
				if ((crc & 0x0001) ^ (tmp & 0x0001))
					crc = (crc >> 1) ^ 0x8408;
				else
					crc >>= 1;
			}
		} while (--size);

		crc = ~crc;
		tmp = crc;
		crc = (crc << 8) | (tmp >> 8 & 0xFF);
	}

	return crc;
}

static u16 _cyttsp4_get_short(u8 *buf)
{
	return ((u16)(*buf) << 8) + *(buf+1);
}

static u8 *_cyttsp4_get_row(struct cyttsp4_device *ttsp,
			    u8 *row_buf, u8 *image_buf, int size)
{
	memcpy(row_buf, image_buf, size);
	image_buf = image_buf + size;
	return image_buf;
}

static int _cyttsp4_get_status(struct cyttsp4_device *ttsp,
			       u8 *buf, int size, unsigned long timeout_ms)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_loader_data *data = dev_get_drvdata(dev);

	unsigned long uretval;
	int tries;
	int retval = 0;

	if (timeout_ms != 0) {
		/* wait until status ready interrupt or timeout occurs */
		uretval = wait_for_completion_timeout(
			&data->int_running, msecs_to_jiffies(timeout_ms));

		/* TODO: Reconsider purpose of having retries here */
		for (tries = 0; tries < 2; tries++) {
			retval = cyttsp4_read(ttsp, CY_MODE_BOOTLOADER,
					      CY_REG_BASE, buf, size);
			/*
			 * retry if bus read error or
			 * status byte shows not ready
			 */
			if (buf[1] == CY_COMM_BUSY || buf[1] == CY_CMD_BUSY)
				msleep(20); /* TODO: Constant if code kept */
			else
				break;
		}
		dev_vdbg(dev,
			"%s: tries=%d ret=%d status=%02X\n",
			__func__, tries, retval, buf[1]);
	}

	return retval;
}

static int _cyttsp4_send_cmd(struct cyttsp4_device *ttsp, const u8 *cmd_buf,
			     int cmd_size, u8 *stat_ret, size_t num_stat_byte,
			     size_t status_size, unsigned long timeout_ms)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_loader_data *data = dev_get_drvdata(dev);

	u8 *status_buf = data->status_buf;
	int retval = 0;

	if (cmd_buf == NULL)
		goto _cyttsp4_send_cmd_exit;

	if (cmd_size == 0)
		goto _cyttsp4_send_cmd_exit;

	if (timeout_ms > 0)
		INIT_COMPLETION(data->int_running);
	retval = cyttsp4_write(ttsp, CY_MODE_BOOTLOADER,
			       CY_REG_BASE, cmd_buf, cmd_size);
	if (retval < 0) {
		dev_err(dev,
			"%s: Fail writing command=%02X\n",
			__func__, cmd_buf[CY_CMD_BYTE]);
		goto _cyttsp4_send_cmd_exit;
	}

	if (timeout_ms > 0) {
		memset(status_buf, 0, sizeof(data->status_buf));
		retval = _cyttsp4_get_status(ttsp, status_buf,
			status_size, timeout_ms);
		if (retval < 0 || status_buf[0] != CY_START_OF_PACKET) {
			dev_err(dev,
				"%s: Error getting status r=%d"
				" status_buf[0]=%02X\n",
				__func__, retval, status_buf[0]);
			if (!(retval < 0))
				retval = -EIO;
			goto _cyttsp4_send_cmd_exit;
		} else {
			if (status_buf[CY_STATUS_BYTE] != ERROR_SUCCESS) {
				dev_err(dev,
					"%s: Status=0x%02X error\n",
					__func__, status_buf[CY_STATUS_BYTE]);
				retval = -EIO;
			} else if (stat_ret != NULL) {
				if (num_stat_byte < status_size)
					*stat_ret = status_buf[num_stat_byte];
				else
					*stat_ret = 0;
			}
		}
	} else {
		if (stat_ret != NULL)
			*stat_ret = ERROR_SUCCESS;
	}

_cyttsp4_send_cmd_exit:
	return retval;
}

static int _cyttsp4_ldr_enter(struct cyttsp4_device *ttsp,
		struct cyttsp4_dev_id *dev_id)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_loader_data *data = dev_get_drvdata(dev);

	u16 crc = 0;
	int i = 0;
	size_t cmd_size = 0;
	u8 *status_buf = &data->status_buf[0];
	u8 status = 0;
	int retval;
	/* +1 for TMA400 host sync byte */
	u8 ldr_enter_cmd[CY_CMD_LDR_ENTER_CMD_SIZE+1];
	memset(status_buf, 0, sizeof(data->status_buf));
	dev_id->bl_ver = 0;
	dev_id->rev_id = 0;
	dev_id->silicon_id = 0;

	ldr_enter_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
	ldr_enter_cmd[i++] = CY_START_OF_PACKET;
	ldr_enter_cmd[i++] = CY_CMD_LDR_ENTER;
	ldr_enter_cmd[i++] = 0x00;	/* data len lsb */
	ldr_enter_cmd[i++] = 0x00;	/* data len msb */
	crc = _cyttsp4_compute_crc(ttsp, &ldr_enter_cmd[1], i - 1);
	cmd_size = sizeof(ldr_enter_cmd);
	ldr_enter_cmd[i++] = (u8)crc;
	ldr_enter_cmd[i++] = (u8)(crc >> 8);
	ldr_enter_cmd[i++] = CY_END_OF_PACKET;

	INIT_COMPLETION(data->int_running);

	retval = cyttsp4_write(ttsp, CY_MODE_BOOTLOADER,
		CY_REG_BASE, ldr_enter_cmd, cmd_size);
	if (retval < 0) {
		dev_err(dev,
			"%s: write block failed %d\n", __func__, retval);
		return retval;
	}
	retval = _cyttsp4_get_status(ttsp, status_buf,
		CY_CMD_LDR_ENTER_STAT_SIZE, CY_CMD_TIMEOUT);

	if (retval < 0) {
		dev_err(dev,
			"%s: Fail get status to Enter Loader command r=%d\n",
			__func__, retval);
		return retval;
	}
	status = status_buf[CY_STATUS_BYTE];
	if (status == ERROR_SUCCESS) {
		dev_id->bl_ver =
			status_buf[11] << 16 |
			status_buf[10] <<  8 |
			status_buf[9] <<  0;
		dev_id->rev_id =
			status_buf[8] <<  0;
		dev_id->silicon_id =
			status_buf[7] << 24 |
			status_buf[6] << 16 |
			status_buf[5] <<  8 |
			status_buf[4] <<  0;
		retval = 0;
	} else
		retval = -EIO;
	dev_vdbg(dev,
		 "%s: status=%d "
		 "bl_ver=%08X rev_id=%02X silicon_id=%08X\n",
		 __func__, status,
		 dev_id->bl_ver, dev_id->rev_id, dev_id->silicon_id);

	return retval;
}

static int _cyttsp4_ldr_init(struct cyttsp4_device *ttsp)
{
	u16 crc;
	int i = 0;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_init_cmd[CY_CMD_LDR_INIT_CMD_SIZE+1];

	ldr_init_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
	ldr_init_cmd[i++] = CY_START_OF_PACKET;
	ldr_init_cmd[i++] = CY_CMD_LDR_INIT;
	ldr_init_cmd[i++] = 0x08;	/* data len lsb */
	ldr_init_cmd[i++] = 0x00;	/* data len msb */
	memcpy(&ldr_init_cmd[i], cyttsp4_security_key,
		sizeof(cyttsp4_security_key));
	i += sizeof(cyttsp4_security_key);
	crc = _cyttsp4_compute_crc(ttsp, &ldr_init_cmd[1], i - 1);
	ldr_init_cmd[i++] = (u8)crc;
	ldr_init_cmd[i++] = (u8)(crc >> 8);
	ldr_init_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ttsp, ldr_init_cmd, i, NULL, 0,
				   CY_CMD_LDR_INIT_STAT_SIZE,
				   CY_CMD_LDR_INIT_TIMEOUT);
	if (retval < 0) {
		dev_err(&ttsp->dev,
			"%s: Fail ldr init r=%d\n",
			__func__, retval);
	}

	return retval;
}

struct cyttsp4_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[CY_DATA_ROW_SIZE];
} __packed;

static int _cyttsp4_ldr_parse_row(struct cyttsp4_device *ttsp, u8 *row_buf,
	struct cyttsp4_hex_image *row_image)
{
	int retval = 0;

	row_image->array_id = row_buf[CY_ARRAY_ID_OFFSET];
	row_image->row_num = _cyttsp4_get_short(&row_buf[CY_ROW_NUM_OFFSET]);
	row_image->row_size = _cyttsp4_get_short(&row_buf[CY_ROW_SIZE_OFFSET]);

	if (row_image->row_size > ARRAY_SIZE(row_image->row_data)) {
		dev_err(&ttsp->dev,
			"%s: row data buffer overflow\n", __func__);
		retval = -EOVERFLOW;
		goto cyttsp4_ldr_parse_row_exit;
	}

	memcpy(row_image->row_data, &row_buf[CY_ROW_DATA_OFFSET],
	       row_image->row_size);
cyttsp4_ldr_parse_row_exit:
	return retval;
}

static int _cyttsp4_ldr_prog_row(struct cyttsp4_device *ttsp,
				 struct cyttsp4_hex_image *row_image)
{
	u16 crc;
	int next;
	int data;
	int row_data;
	u16 row_sum;
	size_t data_len;
	int retval = 0;

	u8 *cmd = kzalloc(CY_MAX_PACKET_LEN, GFP_KERNEL);

	if (cmd != NULL) {
		row_data = 0;
		row_sum = 0;
		next = 0;
		cmd[next++] = CY_CMD_LDR_HOST_SYNC;
		cmd[next++] = CY_START_OF_PACKET;
		cmd[next++] = CY_CMD_LDR_PROG_ROW;
		/*
		 * include array id size and row id size in CY_PACKET_DATA_LEN
		 */
		data_len = CY_DATA_ROW_SIZE_TMA400;
		cmd[next++] = (u8)(data_len+3);
		cmd[next++] = (u8)((data_len+3) >> 8);
		cmd[next++] = row_image->array_id;
		cmd[next++] = (u8)row_image->row_num;
		cmd[next++] = (u8)(row_image->row_num >> 8);

		for (data = 0;
			data < data_len; data++) {
			cmd[next] = row_image->row_data[row_data++];
			row_sum += cmd[next];
			next++;
		}

		crc = _cyttsp4_compute_crc(ttsp, &cmd[1], next - 1);
		cmd[next++] = (u8)crc;
		cmd[next++] = (u8)(crc >> 8);
		cmd[next++] = CY_END_OF_PACKET;

		retval = _cyttsp4_send_cmd(ttsp, cmd, next, NULL, 0,
					   CY_CMD_LDR_PROG_ROW_STAT_SIZE,
					   CY_CMD_TIMEOUT);

		if (retval < 0) {
			dev_err(&ttsp->dev,
			"%s: prog row=%d fail r=%d\n",
				__func__, row_image->row_num, retval);
			goto cyttsp4_ldr_prog_row_exit;
		}

	} else {
		dev_err(&ttsp->dev,
			"%s prog row error - cmd buf is NULL\n", __func__);
		retval = -EIO;
	}

cyttsp4_ldr_prog_row_exit:
	kfree(cmd);
	return retval;
}

static int _cyttsp4_ldr_verify_row(struct cyttsp4_device *ttsp,
	struct cyttsp4_hex_image *row_image)
{
	u16 crc = 0;
	int i = 0;
	u8 verify_checksum;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_verify_row_cmd[CY_CMD_LDR_VERIFY_ROW_CMD_SIZE+1];

	ldr_verify_row_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
	ldr_verify_row_cmd[i++] = CY_START_OF_PACKET;
	ldr_verify_row_cmd[i++] = CY_CMD_LDR_VERIFY_ROW;
	ldr_verify_row_cmd[i++] = 0x03;	/* data len lsb */
	ldr_verify_row_cmd[i++] = 0x00;	/* data len msb */
	ldr_verify_row_cmd[i++] = row_image->array_id;
	ldr_verify_row_cmd[i++] = (u8)row_image->row_num;
	ldr_verify_row_cmd[i++] = (u8)(row_image->row_num >> 8);
	crc = _cyttsp4_compute_crc(ttsp, &ldr_verify_row_cmd[1], i - 1);
	ldr_verify_row_cmd[i++] = (u8)crc;
	ldr_verify_row_cmd[i++] = (u8)(crc >> 8);
	ldr_verify_row_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ttsp, ldr_verify_row_cmd, i,
				   &verify_checksum, 4,
				   CY_CMD_LDR_VERIFY_ROW_STAT_SIZE,
				   CY_CMD_TIMEOUT);

	if (retval < 0) {
		dev_err(&ttsp->dev,
			"%s: verify row=%d fail r=%d\n",
			__func__, row_image->row_num, retval);
	}

	return retval;
}

static int _cyttsp4_ldr_verify_chksum(struct cyttsp4_device *ttsp,
	u8 *app_chksum)
{
	u16 crc = 0;
	int i = 0;
	int retval = 0;
	/* +1 for TMA400 host sync byte */
	u8 ldr_verify_chksum_cmd[CY_CMD_LDR_VERIFY_CHKSUM_CMD_SIZE+1];

	ldr_verify_chksum_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
	ldr_verify_chksum_cmd[i++] = CY_START_OF_PACKET;
	ldr_verify_chksum_cmd[i++] = CY_CMD_LDR_VERIFY_CHKSUM;
	ldr_verify_chksum_cmd[i++] = 0x00;	/* data len lsb */
	ldr_verify_chksum_cmd[i++] = 0x00;	/* data len msb */
	crc = _cyttsp4_compute_crc(ttsp, &ldr_verify_chksum_cmd[1], i - 1);
	ldr_verify_chksum_cmd[i++] = (u8)crc;
	ldr_verify_chksum_cmd[i++] = (u8)(crc >> 8);
	ldr_verify_chksum_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ttsp, ldr_verify_chksum_cmd, i,
				   app_chksum, 4,
				   CY_CMD_LDR_VERIFY_CHKSUM_STAT_SIZE,
				   CY_CMD_TIMEOUT);

	if (retval < 0) {
		dev_err(&ttsp->dev,
			"%s: verify checksum fail r=%d\n",
			__func__, retval);
	}

	return retval;
}

/* Constructs loader exit command and sends via _cyttsp4_send_cmd() */
static int _cyttsp4_ldr_exit(struct cyttsp4_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	u16 crc = 0;
	int i = 0;
	int retval;
	/* +1 for TMA400 host sync byte */
	u8 ldr_exit_cmd[CY_CMD_LDR_EXIT_CMD_SIZE+1];

	ldr_exit_cmd[i++] = CY_CMD_LDR_HOST_SYNC;
	ldr_exit_cmd[i++] = CY_START_OF_PACKET;
	ldr_exit_cmd[i++] = CY_CMD_LDR_EXIT;
	ldr_exit_cmd[i++] = 0x00;	/* data len lsb */
	ldr_exit_cmd[i++] = 0x00;	/* data len msb */
	crc = _cyttsp4_compute_crc(ttsp, &ldr_exit_cmd[1], i - 1);
	ldr_exit_cmd[i++] = (u8)crc;
	ldr_exit_cmd[i++] = (u8)(crc >> 8);
	ldr_exit_cmd[i++] = CY_END_OF_PACKET;

	retval = _cyttsp4_send_cmd(ttsp, ldr_exit_cmd, i, NULL, 0,
				   CY_CMD_LDR_EXIT_STAT_SIZE, 0);

	if (retval < 0) {
		dev_err(dev,
			"%s: BL Loader exit fail r=%d\n",
			__func__, retval);
	}

	dev_vdbg(dev,
		"%s: Exit BL Loader r=%d\n", __func__, retval);

	return retval;
}

static int _cyttsp4_load_app(struct cyttsp4_device *ttsp, const u8 *fw,
			     int fw_size)
{
	struct device *dev = &ttsp->dev;
	u8 *p;
	int ret;
	int retval;	/* need separate return value at exit stage */
	struct cyttsp4_dev_id *file_id = NULL;
	struct cyttsp4_dev_id *dev_id = NULL;
	struct cyttsp4_hex_image *row_image = NULL;
	u8 app_chksum;

	u8 *row_buf = NULL;
	/* Prevent loading if TMA ver not defined. */
	size_t image_rec_size = fw_size + 1;
	size_t row_buf_size = 1024 > CY_MAX_PRBUF_SIZE ?
		1024 : CY_MAX_PRBUF_SIZE;
	int row_count = 0;

	image_rec_size = CY_DATA_ROW_SIZE_TMA400 +
		(sizeof(struct cyttsp4_hex_image) - CY_DATA_ROW_SIZE);
	if (fw_size % image_rec_size != 0) {
		dev_err(dev,
			"%s: Firmware image is misaligned\n", __func__);
		retval = -EINVAL;
		goto _cyttsp4_load_app_exit;
	}

	dev_info(dev, "%s: start load app\n", __func__);

	row_buf = kzalloc(row_buf_size, GFP_KERNEL);
	row_image = kzalloc(sizeof(struct cyttsp4_hex_image), GFP_KERNEL);
	file_id = kzalloc(sizeof(struct cyttsp4_dev_id), GFP_KERNEL);
	dev_id = kzalloc(sizeof(struct cyttsp4_dev_id), GFP_KERNEL);
	if (row_buf == NULL || row_image == NULL ||
	    file_id == NULL || dev_id == NULL) {
		dev_err(dev,
			"%s: Unable to alloc row buffers(%p %p %p %p)\n",
			__func__, row_buf, row_image, file_id, dev_id);
		retval = -ENOMEM;
		goto _cyttsp4_load_app_exit;
	}

	cyttsp4_request_stop_wd(ttsp);

	p = (u8 *)fw;
	/* Enter Loader and return Silicon ID and Rev */

	retval = cyttsp4_request_reset(ttsp);
	if (retval < 0) {
		dev_err(dev,
			"%s: Fail reset device r=%d\n", __func__, retval);
		goto _cyttsp4_load_app_exit;
	}

	dev_info(dev,
			"%s: Send BL Loader Enter\n", __func__);
	retval = _cyttsp4_ldr_enter(ttsp, dev_id);
	if (retval < 0) {
		dev_err(dev,
			"%s: Error cannot start Loader (ret=%d)\n",
			__func__, retval);
		goto _cyttsp4_load_app_exit;
	}

	dev_vdbg(dev,
		"%s: dev: silicon id=%08X rev=%02X bl=%08X\n",
		__func__, dev_id->silicon_id,
		dev_id->rev_id, dev_id->bl_ver);

	udelay(1000);
	retval = _cyttsp4_ldr_init(ttsp);
	if (retval < 0) {
		dev_err(dev,
			"%s: Error cannot init Loader (ret=%d)\n",
			__func__, retval);
		goto _cyttsp4_load_app_exit;
	}

	dev_info(dev,
			"%s: Send BL Loader Blocks\n", __func__);
	while (p < (fw + fw_size)) {
		/* Get row */
		dev_dbg(dev,
			"%s: read row=%d\n", __func__, ++row_count);
		memset(row_buf, 0, row_buf_size);
		p = _cyttsp4_get_row(ttsp, row_buf, p, image_rec_size);

		/* Parse row */
		dev_vdbg(dev,
			"%s: p=%p buf=%p buf[0]=%02X\n", __func__,
			p, row_buf, row_buf[0]);
		retval = _cyttsp4_ldr_parse_row(ttsp, row_buf, row_image);
		dev_vdbg(dev,
			"%s: array_id=%02X row_num=%04X(%d)"
				" row_size=%04X(%d)\n", __func__,
			row_image->array_id,
			row_image->row_num, row_image->row_num,
			row_image->row_size, row_image->row_size);
		if (retval < 0) {
			dev_err(dev,
			"%s: Parse Row Error "
				"(a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num,
				retval);
			goto bl_exit;
		} else {
			dev_vdbg(dev,
				"%s: Parse Row "
				"(a=%d r=%d ret=%d\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
		}

		/* program row */
		retval = _cyttsp4_ldr_prog_row(ttsp, row_image);
		if (retval < 0) {
			dev_err(dev,
			"%s: Program Row Error "
				"(array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
			goto _cyttsp4_load_app_exit;
		}

		/* verify row */
		retval = _cyttsp4_ldr_verify_row(ttsp, row_image);
		if (retval < 0) {
			dev_err(dev,
			"%s: Verify Row Error "
				"(array=%d row=%d ret=%d)\n",
				__func__, row_image->array_id,
				row_image->row_num, retval);
			goto _cyttsp4_load_app_exit;
		}

		dev_vdbg(dev,
			"%s: array=%d row_cnt=%d row_num=%04X\n",
			__func__, row_image->array_id, row_count,
			row_image->row_num);
	}

	/* verify app checksum */
	retval = _cyttsp4_ldr_verify_chksum(ttsp, &app_chksum);
	dev_dbg(dev,
		"%s: Application Checksum = %02X r=%d\n",
		__func__, app_chksum, retval);
	if (retval < 0) {
		dev_err(dev,
			"%s: ldr_verify_chksum fail r=%d\n", __func__, retval);
		retval = 0;
	}

	/* exit loader */
bl_exit:
	dev_info(dev,
			"%s: Send BL Loader Terminate\n", __func__);
	ret = _cyttsp4_ldr_exit(ttsp);
	if (ret) {
		dev_err(dev,
			"%s: Error on exit Loader (ret=%d)\n",
			__func__, ret);
		retval = ret;
	}

_cyttsp4_load_app_exit:
	kfree(row_buf);
	kfree(row_image);
	kfree(file_id);
	kfree(dev_id);
	return retval;
}

static int _start_fw_class(struct cyttsp4_device *ttsp,
		void (*cont) (const struct firmware *fw, void *context))
{
	int retval;

	dev_vdbg(&ttsp->dev,
		"%s: Enabling firmware class loader\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_NOHOTPLUG, "", &ttsp->dev,
		GFP_KERNEL, ttsp, cont);

	if (retval < 0) {
		dev_err(&ttsp->dev,
			"%s: Fail request firmware class file load\n",
			__func__);
		return retval;
	}

	return 0;
}

static void _cyttsp4_firmware_cont(const struct firmware *fw, void *context)
{
	struct cyttsp4_device *ttsp = context;
	struct device *dev = &ttsp->dev;
	struct cyttsp4_core_platform_data *core_pdata =
				dev_get_platdata(&ttsp->core->dev);
	int retval = 0;
	u8 header_size = 0;

	if (fw == NULL)
		goto cyttsp4_firmware_cont_exit;

	if (fw->data == NULL || fw->size == 0) {
		dev_err(dev,
			"%s: No firmware received\n", __func__);
		goto cyttsp4_firmware_cont_release_exit;
	}

	header_size = fw->data[0];
	if (header_size >= (fw->size + 1)) {
		dev_err(dev,
			"%s: Firmware format is invalid\n", __func__);
		goto cyttsp4_firmware_cont_release_exit;
	}

	pm_runtime_get_sync(dev);

	retval = cyttsp4_request_exclusive(ttsp, 5000);
	if (retval < 0)
		goto cyttsp4_firmware_cont_release_exit;

	retval = _cyttsp4_load_app(ttsp, &(fw->data[header_size + 1]),
		fw->size - (header_size + 1));
	if (retval < 0) {
		dev_err(dev,
			"%s: Firmware update failed with error code %d\n",
			__func__, retval);
	}
	dev_info(dev, "%s: Firmware update finished.\n",  __func__);
	cyttsp4_release_exclusive(ttsp);
	if (core_pdata->use_auto_calibration) {
		cyttsp4_request_calibration(ttsp);
	}
	cyttsp4_request_restart(ttsp);

cyttsp4_firmware_cont_release_exit:
	pm_runtime_put(dev);
	release_firmware(fw);

cyttsp4_firmware_cont_exit:
	return;
}

/*
 * return code
 *  0: Do not upgrade firmware
 * !0: Do a firmware upgrade
 *
 */
static int cyttsp4_check_version(struct cyttsp4_device *ttsp,
		const struct firmware *fw)
{
	struct cyttsp4_loader_data *data = dev_get_drvdata(&ttsp->dev);
	struct device *dev = &ttsp->dev;
	u32 fw_ver_img;
	u32 fw_revctrl_img_h;
	u32 fw_revctrl_img_l;
	u32 fw_ver_new;
	u32 fw_revctrl_new_h;
	u32 fw_revctrl_new_l;

	if (!data->si) {
		dev_dbg(dev, "%s: No firmware infomation found," \
			" device FW may be corrupted\n", __func__);
		return CYTTSP4_AUTO_LOAD_FOR_CORRUPTED_FW;
	}

	fw_ver_img = data->si->si_ptrs.cydata->fw_ver_major << 8;
	fw_ver_img += data->si->si_ptrs.cydata->fw_ver_minor;
	fw_ver_new = (u8)fw->data[3] << 8;
	fw_ver_new += (u8)fw->data[4];

	dev_info(dev, "%s: img vers:0x%04X new vers:0x%04X\n", __func__,
			fw_ver_img, fw_ver_new);

	if (fw_ver_new > fw_ver_img) {
		dev_dbg(dev, "%s: Image is newer, will upgrade\n",
				__func__);
		goto cyttsp4_firmware_upgrade;
	}

	if (fw_ver_new < fw_ver_img) {
		dev_dbg(dev, "%s: Image is older, will NOT upgrade\n",
				__func__);
		goto cyttsp4_firmware_no_upgrade;
	}

	fw_revctrl_img_h = be32_to_cpu(
		*(u32 *)(data->si->si_ptrs.cydata->revctrl + 0));
	fw_revctrl_new_h = be32_to_cpu(*(u32 *)(fw->data + 5));

	dev_info(dev, "%s: img revctrl_h:0x%04X new revctrl_h:0x%04X\n",
			__func__, fw_revctrl_img_h, fw_revctrl_new_h);

	if (fw_revctrl_new_h > fw_revctrl_img_h) {
		dev_dbg(dev, "%s: Image is newer, will upgrade\n",
				__func__);
		goto cyttsp4_firmware_upgrade;
	}

	if (fw_revctrl_new_h < fw_revctrl_img_h) {
		dev_dbg(dev, "%s: Image is older, will NOT upgrade\n",
				__func__);
		goto cyttsp4_firmware_no_upgrade;
	}

	fw_revctrl_img_l = be32_to_cpu(
		*(u32 *)(data->si->si_ptrs.cydata->revctrl + 4));
	fw_revctrl_new_l = be32_to_cpu(*(u32 *)(fw->data + 9));

	dev_info(dev, "%s: img revctrl_l:0x%04X new revctrl_l:0x%04X\n",
			__func__, fw_revctrl_img_l, fw_revctrl_new_l);

	if (fw_revctrl_new_l > fw_revctrl_img_l) {
		dev_dbg(dev, "%s: Image is newer, will upgrade\n",
				__func__);
		goto cyttsp4_firmware_upgrade;
	}

	if (fw_revctrl_new_l < fw_revctrl_img_l) {
		dev_dbg(dev, "%s: Image is older, will NOT upgrade\n",
				__func__);
		goto cyttsp4_firmware_no_upgrade;
	}

	/* equal */
	dev_dbg(dev, "%s: Image is same, will NOT upgrade\n", __func__);
cyttsp4_firmware_no_upgrade:
	return 0;

cyttsp4_firmware_upgrade:
	return 1;
}

static void _cyttsp4_firmware_cont_builtin(const struct firmware *fw,
		void *context)
{
	struct cyttsp4_device *ttsp = context;
	struct device *dev = &ttsp->dev;
	int upgrade;

	if (fw == NULL) {
		dev_err(dev,
			"%s: NULL firmware received\n", __func__);
		goto _cyttsp4_firmware_cont_builtin_exit;
	}

	if (fw->data == NULL || fw->size == 0) {
		dev_err(dev,
			"%s: No firmware received\n", __func__);
		goto _cyttsp4_firmware_cont_builtin_exit;
	}
	dev_dbg(dev, "%s: Found firmware\n", __func__);

	upgrade = cyttsp4_check_version(ttsp, fw);
	if (upgrade) {
		_cyttsp4_firmware_cont(fw, ttsp);
		return;
	}
_cyttsp4_firmware_cont_builtin_exit:
	release_firmware(fw);
}

static int _start_fw_builtin(struct cyttsp4_device *ttsp,
		void (*cont) (const struct firmware *fw, void *context))
{
	struct device *dev = &ttsp->dev;
	int retval;

	dev_vdbg(dev,
		"%s: Enabling firmware class loader built-in\n", __func__);

	retval = request_firmware_nowait(THIS_MODULE,
		FW_ACTION_HOTPLUG, CY_FW_FILE_NAME, dev,
		GFP_KERNEL, ttsp, cont);
	if (retval < 0) {
		dev_err(dev,
			"%s: Fail request firmware class file load\n",
			__func__);
	}

	return retval;
}

static int cyttsp4_loader_attention(struct cyttsp4_device *ttsp)
{
	struct cyttsp4_loader_data *data = dev_get_drvdata(&ttsp->dev);
	complete(&data->int_running);
	return 0;
}

static ssize_t cyttsp4_manual_upgrade_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4_loader_data *data = dev_get_drvdata(dev);
	int rc;

	rc = _start_fw_class(data->ttsp, _cyttsp4_firmware_cont);
	if (rc < 0)
		return rc;

	return size;
}

static DEVICE_ATTR(manual_upgrade, S_IRUSR | S_IWUSR,
	NULL, cyttsp4_manual_upgrade_store);

static int cyttsp4_loader_probe(struct cyttsp4_device *ttsp)
{
	struct cyttsp4_loader_data *data;
	struct device *dev = &ttsp->dev;
	int rc;

	dev_dbg(dev, "%s\n", __func__);
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	rc = device_create_file(dev, &dev_attr_manual_upgrade);
	if (rc) {
		dev_err(dev, "%s: Error, could not create manual_upgrade\n",
				__func__);
		goto error_attr_create_failed;
	}

	data->ttsp = ttsp;
	dev_set_drvdata(dev, data);
	init_completion(&(data->int_running));

	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);
	data->si = cyttsp4_request_sysinfo(ttsp);
	if (data->si == NULL)
		dev_warn(dev, "%s: Fail get sysinfo pointer from core\n",
			__func__);
	cyttsp4_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp4_loader_attention, CY_MODE_BOOTLOADER);
	rc = _start_fw_builtin(ttsp, _cyttsp4_firmware_cont_builtin);
	if (rc < 0) {
		dev_err(dev, "%s: Error, could not start firmware loader.\n",
				__func__);
		goto error_builtin_start_failed;
	}
	pm_runtime_put(dev);

	dev_info(dev, "%s: Successful probe %s\n", __func__, ttsp->name);
	return 0;

error_builtin_start_failed:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
error_attr_create_failed:
	kfree(data);
error_alloc_data_failed:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

static int cyttsp4_loader_release(struct cyttsp4_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_loader_data *data = dev_get_drvdata(dev);
	int retval;

	dev_dbg(dev, "%s\n", __func__);
	retval = cyttsp4_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp4_loader_attention, CY_MODE_BOOTLOADER);
	if (retval < 0) {
		dev_err(dev,
			"%s: Failed to restart IC with error code %d\n",
			__func__, retval);
	}
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	device_remove_file(dev, &dev_attr_manual_upgrade);
	dev_set_drvdata(dev, NULL);
	kfree(data);
	return retval;
}

struct cyttsp4_driver cyttsp4_loader_driver = {
	.probe = cyttsp4_loader_probe,
	.remove = cyttsp4_loader_release,
	.driver = {
		.name = CYTTSP4_LOADER_NAME,
		.bus = &cyttsp4_bus_type,
		.owner = THIS_MODULE,
	},
};

static const char cyttsp4_loader_name[] = CYTTSP4_LOADER_NAME;
static struct cyttsp4_device cyttsp4_loader_devices[CY_MAX_NUM_CORE_DEVS];

static char *core_ids[CY_MAX_NUM_CORE_DEVS] = {
	"main_ttsp_core",
	NULL,
	NULL,
	NULL,
	NULL
};

static int num_core_ids = 1;

module_param_array(core_ids, charp, &num_core_ids, 0);
MODULE_PARM_DESC(core_ids, "Core id list of cyttsp4 core devices");

static int __init cyttsp4_loader_init(void)
{
	int rc = 0;
	int i = 0;

	for (i = 0; i < num_core_ids && i < CY_MAX_NUM_CORE_DEVS; i++) {
		cyttsp4_loader_devices[i].name = cyttsp4_loader_name;
		cyttsp4_loader_devices[i].core_id = core_ids[i];
		pr_info("%s: Registering loader device for core_id: %s\n",
			__func__, cyttsp4_loader_devices[i].core_id);
		rc = cyttsp4_register_device(&cyttsp4_loader_devices[i]);
		if (rc < 0) {
			pr_err("%s: Error, failed registering device\n",
				__func__);
			goto cyttsp4_loader_init_exit;
		}
	}
	rc = cyttsp4_register_driver(&cyttsp4_loader_driver);
	pr_info("%s: Cypress TTSP FW loader (Built %s @ %s) rc=%d\n",
		 __func__, __DATE__, __TIME__, rc);
cyttsp4_loader_init_exit:
	return rc;
}
module_init(cyttsp4_loader_init);

static void __exit cyttsp4_loader_exit(void)
{
	int i = 0;

	cyttsp4_unregister_driver(&cyttsp4_loader_driver);
	for (i = 0; i < num_core_ids && i < CY_MAX_NUM_CORE_DEVS; i++) {
		cyttsp4_unregister_device(&cyttsp4_loader_devices[i]);
		pr_info("%s: Unregistering loader device for core_id: %s\n",
			__func__, cyttsp4_loader_devices[i].core_id);
	}
	pr_info("%s: module exit\n", __func__);
}
module_exit(cyttsp4_loader_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen FW loader");
MODULE_AUTHOR("Cypress Semiconductor");
