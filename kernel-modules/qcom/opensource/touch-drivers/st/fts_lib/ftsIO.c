// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2016-2019, STMicroelectronics Limited.
 * Authors: AMG(Analog Mems Group) <marco.cali@st.com>
 *
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *                     I2C/SPI Communication				  *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsIO.c
  * \brief Contains all the functions which handle with the I2C/SPI
  *communication
  */


#include "ftsSoftware.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#if (KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE)
#include <linux/stdarg.h>
#else
#include <stdarg.h>
#endif
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsHardware.h"
#include "ftsIO.h"


static u16 I2CSAD[MAX_SUPPORTED_TOUCH_PANELS];	/* /< slave address of the IC in the i2c bus */
struct client_info gClient[MAX_SUPPORTED_TOUCH_PANELS];	/* /< bus client retrieved by the OS and
				 * used to execute the bus transfers
				 */

/**
  * Initialize the static client variable of the fts_lib library in order
  * to allow any i2c/spi transaction in the driver (Must be called in the probe)
  * @param clt pointer to i2c_client or spi_device struct which identify the bus
  * slave device
  * @return OK
  */
int openChannel(void *ctl)
{
	struct fts_ts_info *info = (struct fts_ts_info *)ctl;
	int idx = 0;

	if (info->is_primary)
		idx = PRIMARY_TOUCH_IDX;
	else
		idx = SECONDARY_TOUCH_IDX;

	gClient[idx].irq = info->irq;
	gClient[idx].dev = info->dev;
	gClient[idx].bus_type = info->bus_type;

	if (info->bus_type == BUS_I2C) {
		gClient[idx].client = info->i2c_client;
		I2CSAD[idx] = info->i2c_client->addr;
		logError(1, "%s %s: SAD: %02X\n", tag, __func__, I2CSAD[idx]);
	} else if (info->bus_type == BUS_SPI) {
		gClient[idx].client = info->spi_client;
		logError(1, "%s %s: spi_master: flags = %04X !\n", tag, __func__,
#if	(LINUX_VERSION_CODE < KERNEL_VERSION(6, 9, 0))
			 info->spi_client->master->flags);
#else
			 info->spi_client->controller->flags);
#endif

		logError(1,
			 "%s %s: spi_device: max_speed = %d chip select = %02X bits_per_words = %d mode = %04X !\n",
			 tag, __func__, info->spi_client->max_speed_hz,
			 info->spi_client->chip_select,
			 info->spi_client->bits_per_word,
			 info->spi_client->mode);
		logError(1, "%s %s: completed!\n", tag, __func__);
	}

	return OK;
}

/**
  * Change the I2C slave address which will be used during the transaction
  * (For Debug Only)
  * @param sad new slave address id
  * @return OK
  */
int changeSAD(struct fts_ts_info *info, u8 sad)
{
	int idx = 0;

	if (info->is_primary)
		idx = PRIMARY_TOUCH_IDX;
	else
		idx = SECONDARY_TOUCH_IDX;

	I2CSAD[idx] = sad;
	return OK;
}

u16 getI2CSAD(struct fts_ts_info *info)
{
	int idx = 0;

	if (info->is_primary)
		idx = PRIMARY_TOUCH_IDX;
	else
		idx = SECONDARY_TOUCH_IDX;

	return I2CSAD[idx];
}

struct device *getPrimaryDev(void)
{
	struct fts_ts_info *info = NULL;

	for (int i = 0; i < MAX_SUPPORTED_TOUCH_PANELS; i++) {
		info = dev_get_drvdata((&gClient[i])->dev);
		if (info->is_primary)
			return (&gClient[i])->dev;
	}
	return NULL;
}

/**
  * Retrieve the pointer to the device struct of the IC
  * @return a the device struct pointer if client was previously set
  * or NULL in all the other cases
  */
struct device *getDev(struct fts_ts_info *info)
{
	if (getClient(info))
		return getClient(info)->dev;

	return NULL;
}

struct client_info *getClient(struct fts_ts_info *info)
{
	int idx = 0;

	if (info->is_primary)
		idx = PRIMARY_TOUCH_IDX;
	else
		idx = SECONDARY_TOUCH_IDX;

	if (gClient[idx].client == NULL ||
		(gClient[idx].bus_type != BUS_I2C &&
		gClient[idx].bus_type != BUS_SPI))
		return NULL;
	else
		return &gClient[idx];
}

struct i2c_client *toI2CClient(struct fts_ts_info *info)
{
	if (getClient(info) && (getClient(info)->bus_type == BUS_I2C))
		return (struct i2c_client *)getClient(info)->client;
	return NULL;
}

struct spi_device *toSPIClient(struct fts_ts_info *info)
{
	if (getClient(info) && (getClient(info)->bus_type == BUS_SPI))
		return (struct spi_device *)getClient(info)->client;
	return NULL;
}


/****************** New I2C API *********************/

u8 remap_reg(u8 reg, u16 bus_type)
{
	if (bus_type != BUS_I2C)
		return reg;

	if (reg == FTS_CMD_HW_REG_R)
		return FTS_CMD_I2C_HW_REG_R;
	if (reg == FTS_CMD_HW_REG_W)
		return FTS_CMD_I2C_HW_REG_W;
	if (reg == FTS_CMD_FRAMEBUFFER_W)
		return FTS_CMD_I2C_FRAMEBUFFER_W;
	if (reg == FTS_CMD_FRAMEBUFFER_R)
		return FTS_CMD_I2C_FRAMEBUFFER_R;
	if (reg == FTS_CMD_CONFIG_R)
		return FTS_CMD_I2C_CONFIG_R;
	if (reg == FTS_CMD_CONFIG_W)
		return FTS_CMD_I2C_CONFIG_W;
	if (reg == FIFO_CMD_READONE)
		return FIFO_CMD_I2C_READONE;

	return reg;
}

int remap_dummy_byte(int dymmy_byte, u16 bus_type)
{
	if (bus_type != BUS_I2C)
		return dymmy_byte;

	if (dymmy_byte == DUMMY_HW_REG)
		return DUMMY_I2C_HW_REG;
	else if (dymmy_byte == DUMMY_FRAMEBUFFER)
		return DUMMY_I2C_FRAMEBUFFER;
	else if (dymmy_byte == DUMMY_CONFIG)
		return DUMMY_I2C_CONFIG;
	else if (dymmy_byte == DUMMY_FIFO)
		return DUMMY_I2C_FIFO;

	return dymmy_byte;
}


/**
  * Perform a direct bus read
  * @param outBuf pointer of a byte array which should contain the byte read
  *from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int fts_read(struct fts_ts_info *info, u8 *outBuf, int byteToRead)
{
	int ret = -1;
	int retry = 0;
	u8 *rx_buf = NULL;

	struct i2c_msg I2CMsg[1];
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };

	rx_buf = kzalloc(byteToRead, GFP_KERNEL);

	if (!rx_buf) {
		logError(1, "alloc rx_buf failed\n");
		return ERROR_ALLOC;
	}

	if (getClient(info) == NULL) {
		kfree(rx_buf);
		return ERROR_BUS_O;
	}

	if (getClient(info)->bus_type == BUS_I2C) {
		I2CMsg[0].addr = (__u16)getI2CSAD(info);
		I2CMsg[0].flags = (__u16)I2C_M_RD;
		I2CMsg[0].len = (__u16)byteToRead;
		I2CMsg[0].buf = (__u8 *)rx_buf;
	} else if (getClient(info)->bus_type == BUS_SPI) {
		spi_message_init(&msg);
		transfer[0].len = byteToRead;
		transfer[0].delay.value = SPI_DELAY_CS;
		transfer[0].tx_buf = NULL;
		transfer[0].rx_buf = rx_buf;
		spi_message_add_tail(&transfer[0], &msg);
	}

	while (retry < I2C_RETRY && ret < OK) {
		if (getClient(info)->bus_type == BUS_I2C)
			ret = i2c_transfer(toI2CClient(info)->adapter, I2CMsg, 1);
		else if (getClient(info)->bus_type == BUS_SPI)
			ret = spi_sync(toSPIClient(info), &msg);
		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
		/* logError(1,"%s fts_writeCmd: attempt %d\n", tag, retry); */
	}
	if (ret < 0) {
		kfree(rx_buf);
		logError(1, "%s %s: ERROR %08X\n", tag, __func__, ERROR_BUS_R);
		return ERROR_BUS_R;
	}

	memcpy(outBuf, rx_buf, byteToRead);

	kfree(rx_buf);
	return OK;
}


/**
  * Perform a bus write followed by a bus read without a stop condition
  * @param cmd byte array containing the command to write
  * @param cmdLength size of cmd
  * @param outBuf pointer of a byte array which should contain the bytes read
  *from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int fts_writeRead(struct fts_ts_info *info, u8 *cmd, int cmdLength, u8 *outBuf, int byteToRead)
{
	int ret = -1;
	int retry = 0;
	u8 *tx_buf = NULL;
	u8 *rx_buf = NULL;
	struct i2c_msg I2CMsg[2];
	struct spi_message msg;
	struct spi_transfer transfer[2] = { { 0 }, { 0 } };


	tx_buf = kzalloc(cmdLength, GFP_KERNEL);
	rx_buf = kzalloc(byteToRead, GFP_KERNEL);

	if (!rx_buf || !tx_buf) {
		logError(1, "alloc tx/rx_buf failed\n");
		return ERROR_ALLOC;
	}

	if (getClient(info) == NULL) {
		kfree(tx_buf);
		kfree(rx_buf);
		return ERROR_BUS_O;
	}

	memcpy(tx_buf, cmd, cmdLength);

	if (getClient(info)->bus_type == BUS_I2C) {
		/* write msg */
		I2CMsg[0].addr = (__u16)getI2CSAD(info);
		I2CMsg[0].flags = (__u16)0;
		I2CMsg[0].len = (__u16)cmdLength;
		I2CMsg[0].buf = (__u8 *)tx_buf;

		/* read msg */
		I2CMsg[1].addr = (__u16)getI2CSAD(info);
		I2CMsg[1].flags = I2C_M_RD;
		I2CMsg[1].len = byteToRead;
		I2CMsg[1].buf = (__u8 *)rx_buf;
	} else if (getClient(info)->bus_type == BUS_SPI) {
		spi_message_init(&msg);

		transfer[0].len = cmdLength;
		transfer[0].tx_buf = tx_buf;
		transfer[0].rx_buf = NULL;
		spi_message_add_tail(&transfer[0], &msg);

		transfer[1].len = byteToRead;
		transfer[1].delay.value = SPI_DELAY_CS;
		transfer[1].tx_buf = NULL;
		transfer[1].rx_buf = rx_buf;
		spi_message_add_tail(&transfer[1], &msg);
	}

	while (retry < I2C_RETRY && ret < OK) {
		if (getClient(info)->bus_type == BUS_I2C)
			ret = i2c_transfer(toI2CClient(info)->adapter, I2CMsg, 2);
		else if (getClient(info)->bus_type == BUS_SPI)
			ret = spi_sync(toSPIClient(info), &msg);
		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
	}

	if (ret < 0) {
		kfree(tx_buf);
		kfree(rx_buf);
		logError(1, "%s %s: ERROR %08X\n", tag, __func__, ERROR_BUS_WR);
		return ERROR_BUS_WR;
	}
	if (rx_buf)
		memcpy(outBuf, rx_buf, byteToRead);

	kfree(tx_buf);
	kfree(rx_buf);
	return OK;
}


/**
  * Perform a bus write
  * @param cmd byte array containing the command to write
  * @param cmdLength size of cmd
  * @return OK if success or an error code which specify the type of error
  */
int fts_write(struct fts_ts_info *info, u8 *cmd, int cmdLength)
{
	int ret = -1;
	int retry = 0;
	u8 *tx_buf = NULL;

	struct i2c_msg I2CMsg[1];
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };

	tx_buf = kzalloc(cmdLength, GFP_KERNEL);

	if (!tx_buf) {
		logError(1, "alloc tx failed\n");
		return ERROR_ALLOC;
	}

	if (getClient(info) == NULL) {
		kfree(tx_buf);
		return ERROR_BUS_O;
	}

	memcpy(tx_buf, cmd, cmdLength);

	if (getClient(info)->bus_type == BUS_I2C) {
		I2CMsg[0].addr = (__u16)getI2CSAD(info);
		I2CMsg[0].flags = (__u16)0;
		I2CMsg[0].len = (__u16)cmdLength;
		I2CMsg[0].buf = (__u8 *)tx_buf;
	} else if (getClient(info)->bus_type == BUS_SPI) {
		spi_message_init(&msg);
		transfer[0].len = cmdLength;
		transfer[0].delay.value = SPI_DELAY_CS;
		transfer[0].tx_buf = tx_buf;
		transfer[0].rx_buf = NULL;
		spi_message_add_tail(&transfer[0], &msg);
	}

	while (retry < I2C_RETRY && ret < OK) {
		if (getClient(info)->bus_type == BUS_I2C)
			ret = i2c_transfer(toI2CClient(info)->adapter, I2CMsg, 1);
		else if (getClient(info)->bus_type == BUS_SPI)
			ret = spi_sync(toSPIClient(info), &msg);

		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
		/* logError(1,"%s fts_writeCmd: attempt %d\n", tag, retry); */
	}
	if (ret < 0) {
		kfree(tx_buf);
		logError(1, "%s %s: ERROR %08X\n", tag, __func__, ERROR_BUS_W);
		return ERROR_BUS_W;
	}
	kfree(tx_buf);
	return OK;
}

/**
  * Write a FW command to the IC and check automatically the echo event
  * @param cmd byte array containing the command to send
  * @param cmdLength size of cmd
  * @return OK if success, or an error code which specify the type of error
  */
int fts_writeFwCmd(struct fts_ts_info *info, u8 *cmd, int cmdLength)
{
	int ret = -1;
	int ret2 = -1;
	int retry = 0;
	u8 *tx_buf = NULL;

	struct i2c_msg I2CMsg[1];
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };

	tx_buf = kzalloc(cmdLength, GFP_KERNEL);

	if (!tx_buf) {
		logError(1, "alloc tx_buf failed\n");
		return ERROR_ALLOC;
	}

	if (getClient(info) == NULL) {
		kfree(tx_buf);
		return ERROR_BUS_O;
	}

	memcpy(tx_buf, cmd, cmdLength);

	if (getClient(info)->bus_type == BUS_I2C) {
		I2CMsg[0].addr = (__u16)getI2CSAD(info);
		I2CMsg[0].flags = (__u16)0;
		I2CMsg[0].len = (__u16)cmdLength;
		I2CMsg[0].buf = (__u8 *)tx_buf;
	} else if (getClient(info)->bus_type == BUS_SPI) {
		spi_message_init(&msg);
		transfer[0].len = cmdLength;
		transfer[0].delay.value = SPI_DELAY_CS;
		transfer[0].tx_buf = tx_buf;
		transfer[0].rx_buf = NULL;
		spi_message_add_tail(&transfer[0], &msg);
	}

	resetErrorList();
	while (retry < I2C_RETRY && (ret < OK || ret2 < OK)) {
		if (getClient(info)->bus_type == BUS_I2C)
			ret = i2c_transfer(toI2CClient(info)->adapter, I2CMsg, 1);
		else if (getClient(info)->bus_type == BUS_SPI)
			ret = spi_sync(toSPIClient(info), &msg);

		retry++;
		if (ret >= 0)
			ret2 = checkEcho(info, cmd, cmdLength);
		if (ret < OK || ret2 < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
		/* logError(1,"%s fts_writeCmd: attempt %d\n", tag, retry); */
	}
	if (ret < 0) {
		kfree(tx_buf);
		logError(1, "%s fts_writeFwCmd: ERROR %08X\n", tag,
			 ERROR_BUS_W);
		return ERROR_BUS_W;
	}
	if (ret2 < OK) {
		kfree(tx_buf);
		logError(1, "%s fts_writeFwCmd: check echo ERROR %08X\n", tag,
			 ret2);
		return ret2;
	}
	kfree(tx_buf);
	return OK;
}


/**
 * Perform two bus write and one bus read without any stop condition
 * In case of FTI this function is not supported and the same sequence
 * can be achieved calling fts_write followed by an fts_writeRead.
 * @param writeCmd1 byte array containing the first command to write
 * @param writeCmdLength size of writeCmd1
 * @param readCmd1 byte array containing the second command to write
 * @param readCmdLength size of readCmd1
 * @param outBuf pointer of a byte array which should contain the bytes read
 * from the IC
 * @param byteToRead number of bytes to read
 * @return OK if success or an error code which specify the type of error
 */
int fts_writeThenWriteRead(struct fts_ts_info *info,
				u8 *writeCmd1,
				int writeCmdLength,
				u8 *readCmd1,
				int readCmdLength,
				u8 *outBuf,
				int byteToRead)
{
	int ret = -1;
	int retry = 0;

	u8 *tx_buf = NULL;
	u8 *tx_buf1 = NULL;
	u8 *rx_buf = NULL;

	struct i2c_msg I2CMsg[3];
	struct spi_message msg;
	struct spi_transfer transfer[3] = { { 0 }, { 0 }, { 0 } };

	tx_buf = kzalloc(writeCmdLength, GFP_KERNEL);
	tx_buf1 = kzalloc(readCmdLength, GFP_KERNEL);
	rx_buf = kzalloc(byteToRead, GFP_KERNEL);

	if (!tx_buf || !tx_buf1 || !rx_buf) {
		logError(1, "alloc tx/rx_buf failed\n");
		return ERROR_ALLOC;
	}

	if (getClient(info) == NULL) {
		kfree(tx_buf);
		kfree(tx_buf1);
		kfree(rx_buf);
		return ERROR_BUS_O;
	}

	memcpy(tx_buf, writeCmd1, writeCmdLength);
	memcpy(tx_buf1, readCmd1, readCmdLength);


	if (getClient(info)->bus_type == BUS_I2C) {
		/* write msg */
		I2CMsg[0].addr = (__u16)getI2CSAD(info);
		I2CMsg[0].flags = (__u16)0;
		I2CMsg[0].len = (__u16)writeCmdLength;
		I2CMsg[0].buf = (__u8 *)tx_buf;

		/* write msg */
		I2CMsg[1].addr = (__u16)getI2CSAD(info);
		I2CMsg[1].flags = (__u16)0;
		I2CMsg[1].len = (__u16)readCmdLength;
		I2CMsg[1].buf = (__u8 *)tx_buf1;

		/* read msg */
		I2CMsg[2].addr = (__u16)getI2CSAD(info);
		I2CMsg[2].flags = I2C_M_RD;
		I2CMsg[2].len = byteToRead;
		I2CMsg[2].buf = (__u8 *)rx_buf;
	} else if (getClient(info)->bus_type == BUS_SPI) {
		spi_message_init(&msg);

		transfer[0].len = writeCmdLength;
		transfer[0].tx_buf = tx_buf;
		transfer[0].rx_buf = NULL;
		spi_message_add_tail(&transfer[0], &msg);

		transfer[1].len = readCmdLength;
		transfer[1].tx_buf = tx_buf1;
		transfer[1].rx_buf = NULL;
		spi_message_add_tail(&transfer[1], &msg);

		transfer[2].len = byteToRead;
		transfer[2].delay.value = SPI_DELAY_CS;
		transfer[2].tx_buf = NULL;
		transfer[2].rx_buf = rx_buf;
		spi_message_add_tail(&transfer[2], &msg);
	}

	while (retry < I2C_RETRY && ret < OK) {

		if (getClient(info)->bus_type == BUS_I2C)
			ret = i2c_transfer(toI2CClient(info)->adapter, I2CMsg, 3);
		else if (getClient(info)->bus_type == BUS_SPI)
			ret = spi_sync(toSPIClient(info), &msg);

		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
	}

	if (ret < 0) {
		kfree(tx_buf);
		kfree(tx_buf1);
		kfree(rx_buf);
		logError(1, "%s %s: ERROR %08X\n", tag, __func__, ERROR_BUS_WR);
		return ERROR_BUS_WR;
	}

	memcpy(outBuf, rx_buf, byteToRead);
	kfree(tx_buf);
	kfree(tx_buf1);
	kfree(rx_buf);
	return OK;
}

/**
  * Perform a chunked write with one byte op code and 1 to 8 bytes address
  * @param cmd byte containing the op code to write
  * @param addrSize address size in byte
  * @param address the starting address
  * @param data pointer of a byte array which contain the bytes to write
  * @param dataSize size of data
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the address is max 8 bytes */
int fts_writeU8UX(struct fts_ts_info *info, u8 cmd, AddrSize addrSize, u64 address, u8 *data, int
		  dataSize)
{
	u8 finalCmd[FTS_ADDR_SIZE_MAX + WRITE_CHUNK];
	int remaining = dataSize;
	int toWrite = 0, i = 0;

	cmd = remap_reg(cmd, getClient(info)->bus_type);

	if (addrSize <= sizeof(u64)) {
		while (remaining > 0) {
			if (remaining >= WRITE_CHUNK) {
				toWrite = WRITE_CHUNK;
				remaining -= WRITE_CHUNK;
			} else {
				toWrite = remaining;
				remaining = 0;
			}

			finalCmd[0] = cmd;
			logError(0, "%s %s: addrSize = %d\n", tag, __func__,
				 addrSize);
			for (i = 0; i < addrSize; i++) {
				finalCmd[i + 1] = (u8)((address >> ((addrSize -
								     1 - i) *
								    8)) & 0xFF);
				logError(1, "%s %s: cmd[%d] = %02X\n", tag,
					 __func__, i + 1, finalCmd[i + 1]);
			}

			memcpy(&finalCmd[addrSize + 1], data, toWrite);

			if (fts_write(info, finalCmd, 1 + addrSize + toWrite) < OK) {
				logError(1, "%s %s: ERROR %08X\n", tag,
					 __func__, ERROR_BUS_W);
				return ERROR_BUS_W;
			}

			address += toWrite;

			data += toWrite;
		}
	} else
		logError(1,
			 "%s %s: address size bigger than max allowed %ld... ERROR %08X\n",
			 tag, __func__, sizeof(u64), ERROR_OP_NOT_ALLOW);

	return OK;
}

/**
  * Perform a chunked write read with one byte op code and 1 to 8 bytes address
  * and dummy byte support.
  * @param cmd byte containing the op code to write
  * @param addrSize address size in byte
  * @param address the starting address
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param byteToRead number of bytes to read
  * @param hasDummyByte  if the first byte of each reading is dummy (must be
  * skipped)
  * set to 1, otherwise if it is valid set to 0 (or any other value)
  * @return OK if success or an error code which specify the type of error
  */
int fts_writeReadU8UX(struct fts_ts_info *info,
			u8 cmd,
			AddrSize addrSize,
			u64 address,
			u8 *outBuf,
			int byteToRead,
			int hasDummyByte)
{
	u8 finalCmd[FTS_ADDR_SIZE_MAX];
	u8 buff[READ_CHUNK + 1];/* worst case has dummy byte */
	int remaining = byteToRead;
	int toRead = 0, i = 0;

	hasDummyByte = remap_dummy_byte(hasDummyByte, getClient(info)->bus_type);
	cmd = remap_reg(cmd, getClient(info)->bus_type);

	while (remaining > 0) {
		if (remaining >= READ_CHUNK) {
			toRead = READ_CHUNK;
			remaining -= READ_CHUNK;
		} else {
			toRead = remaining;
			remaining = 0;
		}

		finalCmd[0] = cmd;
		for (i = 0; i < addrSize; i++)
			finalCmd[i + 1] = (u8)((address >> ((addrSize - 1 - i) *
							    8)) & 0xFF);

		if (hasDummyByte == 1) {
			if (fts_writeRead(info, finalCmd, 1 + addrSize, buff, toRead +
					  1) < OK) {
				logError(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff + 1, toRead);
		} else {
			if (fts_writeRead(info, finalCmd, 1 + addrSize, buff,
					  toRead) < OK) {
				logError(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff, toRead);
		}

		address += toRead;

		outBuf += toRead;
	}

	return OK;
}

/**
 * Perform a chunked write followed by a second write with one byte op code
 * for each write and 1 to 8 bytes address (the sum of the 2 address size of
 * the two writes can not exceed 8 bytes)
 * @param cmd1 byte containing the op code of first write
 * @param addrSize1 address size in byte of first write
 * @param cmd2 byte containing the op code of second write
 * @param addrSize2 address size in byte of second write
 * @param address the starting address
 * @param data pointer of a byte array which contain the bytes to write
 * @param dataSize size of data
 * @return OK if success or an error code which specify the type of error
 */
/* this function works only if the sum of two addresses in the two commands is
 * max 8 bytes
 */
int fts_writeU8UXthenWriteU8UX(struct fts_ts_info *info,
				u8 cmd1,
				AddrSize addrSize1,
				u8 cmd2,
				AddrSize addrSize2,
				u64 address,
				u8 *data,
				int dataSize)
{
	u8 finalCmd1[FTS_ADDR_SIZE_MAX];
	u8 finalCmd2[FTS_ADDR_SIZE_MAX + WRITE_CHUNK];

	int remaining = dataSize;
	int toWrite = 0, i = 0;

	while (remaining > 0) {
		if (remaining >= WRITE_CHUNK) {
			toWrite = WRITE_CHUNK;
			remaining -= WRITE_CHUNK;
		} else {
			toWrite = remaining;
			remaining = 0;
		}

		finalCmd1[0] = cmd1;
		for (i = 0; i < addrSize1; i++)
			finalCmd1[i + 1] = (u8)((address >> ((addrSize1 +
							      addrSize2 - 1 -
							      i) * 8)) & 0xFF);

		finalCmd2[0] = cmd2;
		for (i = addrSize1; i < addrSize1 + addrSize2; i++)
			finalCmd2[i - addrSize1 + 1] = (u8)((address >>
							     ((addrSize1 +
							       addrSize2 - 1 -
							       i) * 8)) & 0xFF);

		memcpy(&finalCmd2[addrSize2 + 1], data, toWrite);

		if (fts_write(info, finalCmd1, 1 + addrSize1) < OK) {
			logError(1, "%s %s: first write error... ERROR %08X\n",
				 tag, __func__, ERROR_BUS_W);
			return ERROR_BUS_W;
		}

		if (fts_write(info, finalCmd2, 1 + addrSize2 + toWrite) < OK) {
			logError(1,
				 "%s %s: second write error... ERROR %08X\n",
				 tag,
				 __func__, ERROR_BUS_W);
			return ERROR_BUS_W;
		}

		address += toWrite;

		data += toWrite;
	}

	return OK;
}

/**
  * Perform a chunked write  followed by a write read with one byte op code
  * and 1 to 8 bytes address for each write and dummy byte support.
  * @param cmd1 byte containing the op code of first write
  * @param addrSize1 address size in byte of first write
  * @param cmd2 byte containing the op code of second write read
  * @param addrSize2 address size in byte of second write	read
  * @param address the starting address
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param byteToRead number of bytes to read
  * @param hasDummyByte  if the first byte of each reading is dummy (must be
  * skipped) set to 1,
  *  otherwise if it is valid set to 0 (or any other value)
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the sum of two addresses in the two commands is
 * max 8 bytes */
int fts_writeU8UXthenWriteReadU8UX(struct fts_ts_info *info, u8 cmd1, AddrSize addrSize1, u8 cmd2,
				   AddrSize addrSize2, u64 address, u8 *outBuf,
				   int byteToRead, int hasDummyByte)
{
	u8 finalCmd1[FTS_ADDR_SIZE_MAX];
	u8 finalCmd2[FTS_ADDR_SIZE_MAX];


	u8 buff[READ_CHUNK + 1];/* worst case has dummy byte */
	int remaining = byteToRead;
	int toRead = 0, i = 0;

	while (remaining > 0) {
		if (remaining >= READ_CHUNK) {
			toRead = READ_CHUNK;
			remaining -= READ_CHUNK;
		} else {
			toRead = remaining;
			remaining = 0;
		}


		finalCmd1[0] = cmd1;
		for (i = 0; i < addrSize1; i++)
			finalCmd1[i + 1] = (u8)((address >> ((addrSize1 +
							      addrSize2 - 1 -
							      i) * 8)) & 0xFF);
		/* logError(1, "%s %s: finalCmd1[%d] =  %02X\n",
		  *	tag, __func__, i+1, finalCmd1[i + 1]); */

		finalCmd2[0] = cmd2;
		for (i = addrSize1; i < addrSize1 + addrSize2; i++)
			finalCmd2[i - addrSize1 + 1] = (u8)((address >>
							     ((addrSize1 +
							       addrSize2 - 1 -
							       i) * 8)) & 0xFF);

		if (fts_write(info, finalCmd1, 1 + addrSize1) < OK) {
			logError(1, "%s %s: first write error... ERROR %08X\n",
				 tag, __func__, ERROR_BUS_W);
			return ERROR_BUS_W;
		}

		if (hasDummyByte == 1) {
			if (fts_writeRead(info, finalCmd2, 1 + addrSize2, buff,
					  toRead + 1) < OK) {
				logError(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff + 1, toRead);
		} else {
			if (fts_writeRead(info, finalCmd2, 1 + addrSize2, buff,
					  toRead) < OK) {
				logError(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff, toRead);
		}

		address += toRead;
		outBuf += toRead;
	}

	return OK;
}
