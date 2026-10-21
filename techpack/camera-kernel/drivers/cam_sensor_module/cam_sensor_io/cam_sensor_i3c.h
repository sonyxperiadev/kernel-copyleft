/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_SENSOR_I3C_H_
#define _CAM_SENSOR_I3C_H_

#include <linux/delay.h>
#include <media/v4l2-subdev.h>
#include <media/cam_sensor.h>
#include "cam_sensor_io.h"

/**
 * cam_qup_i3c_read : This API handles QUP I3C read operations
 * @client    : QUP IeC client structure
 * @data      : I3C data
 * @addr_type : I3C address type
 * @data_type : I3C data type
 */

int cam_qup_i3c_read(struct i3c_device *client,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type);

/**
 * cam_qup_i3c_read_seq : This API handles QUP I3C Sequential read operations
 * @client    : QUP I3C client structure
 * @data      : I3C data
 * @addr_type : I3C address type
 * @num_bytes : Number of bytes to read
 */

int cam_qup_i3c_read_seq(struct i3c_device *client,
	uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type,
	uint32_t num_byte);

/**
 * cam_qup_i3c_poll : This API handles QUP based I3C poll operation
 * @client    : QUP I3C client structure
 * @addr      : I3C address
 * @data      : I3C data
 * @data_mask : I3C data mask
 * @data_type : I3C data type
 * @addr_type : I3C addr type
 * @delay_ms  : Delay in milli seconds
 */

int cam_qup_i3c_poll(struct i3c_device *client,
	uint32_t addr, uint16_t data, uint16_t data_mask,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type,
	uint32_t delay_ms);

/**
 * cam_qup_i3c_write_table : This API Handles QUP based I3C write random operations
 * @client        : QUP I3C client structure
 * @write_setting : I3C register settings
 */

int cam_qup_i3c_write_table(
	struct camera_io_master *client,
	struct cam_sensor_i2c_reg_setting *write_setting);

/**
 * cam_qup_i3c_write_continuous_write: This API Handles QUP based I3C write continuous(Burst/Seq)
 * @client: QUP I3C client structure
 * @write_setting: I3C register setting
 * @cam_sensor_i3c_write_flag: burst or seq write
 */
int cam_qup_i3c_write_continuous_table(
	struct camera_io_master *client,
	struct cam_sensor_i2c_reg_setting *write_setting,
	uint8_t cam_sensor_i3c_write_flag);

#endif /*_CAM_SENSOR_I3C_H_*/
