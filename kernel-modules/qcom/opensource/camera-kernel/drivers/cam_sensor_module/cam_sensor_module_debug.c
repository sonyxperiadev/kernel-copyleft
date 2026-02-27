// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <media/cam_sensor.h>
#include "cam_debug_util.h"
#include "cam_sensor_cmn_header.h"
#include "cam_sensor_io.h"
#include "cam_sensor_i2c.h"
#include "cam_actuator_dev.h"
#include "cam_eeprom_dev.h"
#include "cam_flash_dev.h"
#include "cam_ois_dev.h"
#include "cam_sensor_dev.h"

#define RW_BUFFER_SIZE          3000
#define LINE_BUFFER_SIZE        150
#define NUM_OF_READ_PARAMS      5
#define NUM_OF_WRITE_PARAMS     7
#define USAGE_STRING   "Read format: r/R, reg_addr(hex), addr_type, "\
			       "data_type, device_type, instance_id (ID pair)\n"\
			       "Write format: w/W, reg_addr(hex), addr_type, "\
			       "reg_value(hex), delay, addr_type, data_type, device_type, "\
				   "instance_id (ID pair)\n"

struct cam_sensor_i2c_devices {
	struct cam_actuator_ctrl_t *actuator[MAX_CAMERAS];
	struct cam_eeprom_ctrl_t   *eeprom[MAX_CAMERAS];
	struct cam_flash_ctrl      *flash[MAX_CAMERAS];
	struct cam_ois_ctrl_t      *ois[MAX_CAMERAS];
	struct cam_sensor_ctrl_t   *sensor[MAX_CAMERAS];
	int num_actuator;
	int num_eeprom;
	int num_flash;
	int num_ois;
	int num_sensor;
};

static struct dentry *debugfs_root;
static char in_buffer[RW_BUFFER_SIZE], out_buffer[RW_BUFFER_SIZE];
static struct cam_sensor_i2c_devices devices = {0};

struct camera_io_master *cam_sensor_module_get_io_master(
	int device_type, int instance_number, bool *is_on)
{
	struct camera_io_master *io_master = NULL;
	*is_on = false;

	switch (device_type) {
	case CAM_SENSOR_ACTUATOR:
		io_master = instance_number >= devices.num_actuator ? NULL :
			&devices.actuator[instance_number]->io_master_info;
		if (io_master)
			*is_on = devices.actuator[instance_number]->cam_act_state ==
				CAM_ACTUATOR_CONFIG ? true : false;
		break;
	case CAM_SENSOR_EEPROM:
		io_master = instance_number >= devices.num_eeprom ? NULL :
			&devices.eeprom[instance_number]->io_master_info;
		if (io_master)
			*is_on = devices.eeprom[instance_number]->cam_eeprom_state ==
				CAM_EEPROM_CONFIG ? true : false;
		break;
	case CAM_SENSOR_FLASH:
		io_master = instance_number >= devices.num_flash ? NULL :
			&devices.flash[instance_number]->io_master_info;
		if (io_master)
			*is_on = devices.flash[instance_number]->flash_state >=
				CAM_FLASH_STATE_CONFIG ? true : false;
		break;
	case CAM_SENSOR_OIS:
		io_master = instance_number >= devices.num_ois ? NULL :
			&devices.ois[instance_number]->io_master_info;
		if (io_master)
			*is_on = devices.ois[instance_number]->cam_ois_state >=
				CAM_OIS_CONFIG ? true : false;
		break;
	case CAM_SENSOR_DEVICE:
		io_master = instance_number >= devices.num_sensor ? NULL :
			&devices.sensor[instance_number]->io_master_info;
		if (io_master)
			*is_on = devices.sensor[instance_number]->sensor_state >=
				CAM_SENSOR_ACQUIRE ? true : false;
		break;
	default:
		CAM_WARN(CAM_SENSOR, "Unrecognized sensor device type: %d", device_type);
		break;
	}

	return io_master;
}

static int cam_sensor_module_parse_line(const char *p_line,
	struct cam_sensor_i2c_reg_setting *reg_list,
	struct camera_io_master **io_master, bool *is_read,
	bool *power_state)
{
	int device_type, instance_number, rc;
	struct cam_sensor_i2c_reg_array *reg_array = reg_list->reg_setting;

	if (!strlen(p_line))
		return -EINVAL;

	CAM_DBG(CAM_SENSOR, "Sensor debugfs string: %s", p_line);

	if (p_line[0] == 'r' || p_line[0] == 'R') {
		*is_read = true;

		rc = sscanf(p_line+2, "%x,%d,%d,%d,%d",
			&reg_array->reg_addr, &reg_list->addr_type, &reg_list->data_type,
			&device_type, &instance_number);
		if (rc == NUM_OF_READ_PARAMS) {
			*io_master = cam_sensor_module_get_io_master(
				device_type, instance_number, power_state);
			return 0;
		}
	} else if (p_line[0] == 'w'  || p_line[0] == 'W') {
		*is_read = false;

		rc = sscanf(p_line+2, "%x,%x,%u,%d,%d,%d,%d",
			&reg_array->reg_addr,  &reg_array->reg_data, &reg_array->delay,
			&reg_list->addr_type, &reg_list->data_type,
			&device_type, &instance_number);
		if (rc == NUM_OF_WRITE_PARAMS) {
			*io_master = cam_sensor_module_get_io_master(
				device_type, instance_number, power_state);
			return 0;
		}
	}

	return -EINVAL;
}

static void cam_sensor_get_device_status(int offset)
{
	int i;
	char line_buffer[LINE_BUFFER_SIZE], *start = out_buffer + offset;

	for (i = 0; i < devices.num_actuator; i++) {
		snprintf(line_buffer, LINE_BUFFER_SIZE,
			"Device: %s, \t %u, state: %d, ID(device_type, instance_number): %d, %d\n",
		devices.actuator[i]->device_name, devices.actuator[i]->soc_info.index,
		devices.actuator[i]->cam_act_state, CAM_SENSOR_ACTUATOR, i);
		strlcat(start, line_buffer, RW_BUFFER_SIZE - offset);
	}
	memset(line_buffer, '\0', LINE_BUFFER_SIZE);
	for (i = 0; i < devices.num_eeprom; i++) {
		snprintf(line_buffer, LINE_BUFFER_SIZE,
			"Device: %s, \t\t %u, state: %d, ID(device_type, instance_number): %d, %d\n",
		devices.eeprom[i]->device_name, devices.eeprom[i]->soc_info.index,
		devices.eeprom[i]->cam_eeprom_state, CAM_SENSOR_EEPROM, i);
		strlcat(start, line_buffer, RW_BUFFER_SIZE - offset);
	}
	memset(line_buffer, '\0', LINE_BUFFER_SIZE);
	for (i = 0; i < devices.num_flash; i++) {
		snprintf(line_buffer, LINE_BUFFER_SIZE,
			"Device: %s, \t\t %u, state: %d, ID(device_type, instance_number): %d, %d\n",
		devices.flash[i]->device_name, devices.flash[i]->soc_info.index,
		devices.flash[i]->flash_state, CAM_SENSOR_FLASH, i);
		strlcat(start, line_buffer, RW_BUFFER_SIZE - offset);
	}
	memset(line_buffer, '\0', LINE_BUFFER_SIZE);
	for (i = 0; i < devices.num_ois; i++) {
		snprintf(line_buffer, LINE_BUFFER_SIZE,
			"Device: %s, \t\t %u, state: %d, ID(device_type, instance_number): %d, %d\n",
		devices.ois[i]->device_name, devices.ois[i]->soc_info.index,
		devices.ois[i]->cam_ois_state, CAM_SENSOR_OIS, i);
		strlcat(start, line_buffer, RW_BUFFER_SIZE - offset);
	}
	memset(line_buffer, '\0', LINE_BUFFER_SIZE);
	for (i = 0; i  < devices.num_sensor; i++) {
		snprintf(line_buffer, LINE_BUFFER_SIZE,
			"Device: %s, \t\t %u, state: %d, ID(device_type, instance_number): %d, %d\n",
		devices.sensor[i]->sensor_name, devices.sensor[i]->soc_info.index,
		devices.sensor[i]->sensor_state, CAM_SENSOR_DEVICE, i);
		strlcat(start, line_buffer, RW_BUFFER_SIZE - offset);
	}
	memset(line_buffer, '\0', LINE_BUFFER_SIZE);
}

static ssize_t i2c_read(struct file *t_file, char __user *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	ssize_t count;

	count = simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, RW_BUFFER_SIZE);
	memset(out_buffer, '\0', RW_BUFFER_SIZE);

	return count;
}

static ssize_t i2c_write(struct file *t_file, const char __user *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	ssize_t bytes_written = 0, rc = 0;
	struct cam_sensor_i2c_reg_setting read_write;
	struct cam_sensor_i2c_reg_array   reg_array;
	struct camera_io_master *io_master = NULL;
	char line_buffer[LINE_BUFFER_SIZE];
	bool is_read, power_state;

	memset(out_buffer, '\0', RW_BUFFER_SIZE);
	memset(line_buffer, '\0', LINE_BUFFER_SIZE);
	read_write.reg_setting = &reg_array;

	bytes_written = simple_write_to_buffer(in_buffer, RW_BUFFER_SIZE - 1,
		t_loff_t, t_char, t_size_t);

	/* Turn it into a C string */
	in_buffer[bytes_written + 1] = '\0';

	rc = cam_sensor_module_parse_line(in_buffer, &read_write,
		&io_master, &is_read, &power_state);

	if (!rc) {
		if (!power_state) {
			snprintf(line_buffer, LINE_BUFFER_SIZE, "Dev not on");
			goto end;
		}
		if (!is_read) {
			read_write.size = 1;
			rc = camera_io_dev_write(io_master, &read_write);
			if (rc) {
				snprintf(line_buffer, LINE_BUFFER_SIZE,
					"Error: 0x%X, 0x%X, rc: %d\n",
					read_write.reg_setting->reg_addr,
					read_write.reg_setting->reg_data, rc);
				strlcat(out_buffer, line_buffer, RW_BUFFER_SIZE);
			}
		} else {
			rc = camera_io_dev_read(io_master,
				read_write.reg_setting->reg_addr,
				&read_write.reg_setting->reg_data, read_write.addr_type,
				read_write.data_type, false);
			if (!rc)
				snprintf(line_buffer, LINE_BUFFER_SIZE, "Read data: 0x%X\n",
					read_write.reg_setting->reg_data);
			else
				snprintf(line_buffer, LINE_BUFFER_SIZE, "Error, rc: %d\n", rc);
			strlcat(out_buffer, line_buffer, RW_BUFFER_SIZE);
		}
	} else {
		strscpy(out_buffer, USAGE_STRING, RW_BUFFER_SIZE);
		cam_sensor_get_device_status(strlen(USAGE_STRING));
	}

end:
	return bytes_written;
}

const struct file_operations i2c_operations = {
	.open = simple_open,
	.read = i2c_read,
	.write = i2c_write,
};

void cam_sensor_module_add_i2c_device(void *ctrl_struct, int device_type)
{
	if (!cam_debugfs_available())
		return;

	CAM_INFO(CAM_SENSOR, "Adding device type: %d", device_type);

	switch (device_type) {
	case CAM_SENSOR_ACTUATOR:
		devices.actuator[devices.num_actuator++] =
			(struct cam_actuator_ctrl_t *) ctrl_struct;
		break;
	case CAM_SENSOR_EEPROM:
		devices.eeprom[devices.num_eeprom++] =
			(struct cam_eeprom_ctrl_t *) ctrl_struct;
		break;
	case CAM_SENSOR_FLASH:
		devices.flash[devices.num_flash++] =
			(struct cam_flash_ctrl *) ctrl_struct;
		break;
	case CAM_SENSOR_OIS:
		devices.ois[devices.num_ois++] =
			(struct cam_ois_ctrl_t *) ctrl_struct;
		break;
	case CAM_SENSOR_DEVICE:
		devices.sensor[devices.num_sensor++] =
			(struct cam_sensor_ctrl_t *) ctrl_struct;
		break;
	default:
		CAM_WARN(CAM_SENSOR, "Unrecognized sensor device type: %d", device_type);
		break;
	}
}

int cam_sensor_module_debug_register(void)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	if (!cam_debugfs_available())
		return 0;

	rc = cam_debugfs_create_subdir("i2c", &dbgfileptr);
	if (rc) {
		CAM_ERR(CAM_MEM, "DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}

	debugfs_root =  dbgfileptr;
	debugfs_create_file("i2c-rw", 0644, debugfs_root,
		NULL, &i2c_operations);

end:
	return rc;
}

void cam_sensor_module_debug_deregister(void)
{
	debugfs_root = NULL;
}
