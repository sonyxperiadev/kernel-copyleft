/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SONY_CAMERA_H
#define __LINUX_SONY_CAMERA_H

#include <linux/time.h>

#define SONY_CAMERA_MAX_I2C_DATA       (256)
#define SONY_CAMERA_MAX_EEPROM_DATA    (8192)

#define SONY_CAMERA_CMD_BASE           (0x10000)
#define SONY_CAMERA_CMD_POWER_UP       (SONY_CAMERA_CMD_BASE + 0)
#define SONY_CAMERA_CMD_POWER_DOWN     (SONY_CAMERA_CMD_BASE + 1)
#define SONY_CAMERA_CMD_I2C_READ       (SONY_CAMERA_CMD_BASE + 2)
#define SONY_CAMERA_CMD_I2C_WRITE      (SONY_CAMERA_CMD_BASE + 3)
#define SONY_CAMERA_CMD_SPI_READ       (SONY_CAMERA_CMD_BASE + 4)
#define SONY_CAMERA_CMD_SPI_WRITE      (SONY_CAMERA_CMD_BASE + 5)
#define SONY_CAMERA_CMD_GET_EEPROM     (SONY_CAMERA_CMD_BASE + 6)
#define SONY_CAMERA_CMD_GET_EVENT      (SONY_CAMERA_CMD_BASE + 7)
#define SONY_CAMERA_CMD_SET_THERMAL    (SONY_CAMERA_CMD_BASE + 8)
#define SONY_CAMERA_CMD_DOWNLOAD_FW    (SONY_CAMERA_CMD_BASE + 9)
#define SONY_CAMERA_CMD_GET_LOT_ID     (SONY_CAMERA_CMD_BASE + 10)
#define SONY_CAMERA_CMD_CLEAR          (SONY_CAMERA_CMD_BASE + 11)
#define SONY_CAMERA_CMD_BUG_ON         (SONY_CAMERA_CMD_BASE + 12)

#define SONY_CAMERA_EVT_BASE           (0x20000)
#define SONY_CAMERA_EVT_SOF            (SONY_CAMERA_EVT_BASE + 0)
#define SONY_CAMERA_EVT_FATAL          (SONY_CAMERA_EVT_BASE + 1)
#define SONY_CAMERA_EVT_GP_STATUS      (SONY_CAMERA_EVT_BASE + 2)

enum sony_camera_i2c_type {
	SONY_CAMERA_I2C_TYPE_INVALID,
	SONY_CAMERA_I2C_TYPE_BYTE,
	SONY_CAMERA_I2C_TYPE_WORD,
	SONY_CAMERA_I2C_TYPE_3B,
	SONY_CAMERA_I2C_TYPE_MAX,
};

struct sony_camera_i2c_data {
	uint32_t	slave_addr;
	uint32_t	addr;
	uint32_t	addr_type;
	uint32_t	len;
	uint8_t		data[SONY_CAMERA_MAX_I2C_DATA];
};

struct sony_camera_spi_data {
	uint32_t	header_size;
	void		*header;
	uint32_t	payload_size;
	void		*payload;
};

struct sony_camera_firmware_download_data {
	char fw_path[PATH_MAX];
};

enum sony_camera_spi_status {
	SPI_ASSERT,
	SPI_DEASSERT,
	SPI_STATUS_MAX,
};

struct sony_camera_eeprom_data {
	uint8_t		data[SONY_CAMERA_MAX_EEPROM_DATA];
};

struct sony_camera_event_sof_data {
	uint32_t	sof_count;
	struct		timeval mono_timestamp;
};

struct sony_camera_event_data {
	uint32_t	type;
	union {
		struct	sony_camera_event_sof_data sof_data;
		uint8_t				gp_status;
		uint8_t				err_code;
	} data;
};

struct sony_camera_thermal_data {
	int32_t		main_sensor;
	int32_t		sub_sensor;
	int32_t		sensor_isp;
};

#endif /* __LINUX_SONY_CAMERA_H */
