/*
 * RMI4 bus driver.
 * drivers/rmi4/rmi4_function_01.c
 *
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (C) 2012 Sony Mobile Communications AB
 *
 * Author: Joachim Holst <joachim.holst@sonymobile.com>
 *
 * Based on rmi_bus by Synaptics and Unixphere.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/rmi4/rmi4.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/mutex.h>
#include <linux/errno.h>

#define RMI4_VERSION_SIZE	2
#define RMI4_PRODUCT_SIZE	10

#define RMI4_VERSION_OFFSET	2
#define RMI4_PROD_NAME_OFFSET	11

#define RMI4_PRODUCT_NAME_LENGTH	10
#define RMI4_PRODUCT_INFO_LENGTH	2
#define RMI4_DATE_CODE_LENGTH		3

#define RMI4_SLEEP_MODE_NORMAL (0x00)
#define RMI4_SLEEP_MODE_SENSOR_SLEEP (0x01)
#define RMI4_SLEEP_MODE_RESERVED0 (0x02)
#define RMI4_SLEEP_MODE_RESERVED1 (0x03)

#define RMI4_IS_VALID_SLEEPMODE(mode) \
	(mode <= RMI4_SLEEP_MODE_RESERVED1)

union f01_device_commands {
	struct {
		u8 reset:1;
		u8 reserved:1;
	};
	u8 reg;
};

struct rmi4_f01_device_control {
	union {
		struct {
			u8 sleep_mode:2;
			u8 nosleep:1;
			u8 reserved:2;
			u8 charger_input:1;
			u8 report_rate:1;
			u8 configured:1;
		};
		u8 reg;
	};
};

struct rmi4_f01_device_status {
	union {
		struct {
			u8 status_code:4;
			u8 reserved:2;
			u8 flash_prog:1;
			u8 unconfigured:1;
		};
		u8 reg;
	};
};

struct rmi4_f01_basic_queries {
	union {
		struct {
			u8 manufacturer_id:8;

			u8 custom_map:1;
			u8 non_compliant:1;
			u8 q1_bit_2:1;
			u8 has_sensor_id:1;
			u8 has_charger_input:1;
			u8 has_adjustable_doze:1;
			u8 has_adjustable_doze_holdoff:1;
			u8 q1_bit_7:1;

			u8 productinfo_1:7;
			u8 q2_bit_7:1;
			u8 productinfo_2:7;
			u8 q3_bit_7:1;

			u8 year:5;
			u8 month:4;
			u8 day:5;
			u8 cp1:1;
			u8 cp2:1;
			u8 wafer_id1_lsb:8;
			u8 wafer_id1_msb:8;
			u8 wafer_id2_lsb:8;
			u8 wafer_id2_msb:8;
			u8 wafer_id3_lsb:8;
		};
		u8 reg[11];
	};
};


struct rmi4_f01_data {
	struct mutex lock;

	struct rmi4_f01_device_control control;
	struct rmi4_f01_basic_queries queries;
	struct rmi4_f01_device_status status;
	char product_name[RMI4_PRODUCT_NAME_LENGTH + 1];

	bool suspended;
	bool reset_forced;
	atomic_t chip_sleeping;
};

static int rmi4_f01_update_control(struct rmi4_function_device *fdev,
					  bool read)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(&fdev->dev);
	struct rmi4_f01_device_control *c = &data->control;

	dev_dbg(&fdev->dev, "%s - %s control data\n", __func__,
		 read ? "Reading" : "Writing");

	dev_dbg(&fdev->dev, "%s - Sleep_mode: %d\n", __func__, c->sleep_mode);
	dev_dbg(&fdev->dev, "%s - nosleep: %d\n", __func__, c->nosleep);
	dev_dbg(&fdev->dev, "%s - charger_input: %d\n", __func__,
		 c->charger_input);
	dev_dbg(&fdev->dev, "%s - report_rate: %d\n", __func__,
		 c->report_rate);
	dev_dbg(&fdev->dev, "%s - configured: %d\n", __func__, c->configured);

	if (read)
		err = rmi4_bus_read(fdev, CONTROL, 0, &c->reg,
				    sizeof(c->reg));
	else
		err = rmi4_bus_write(fdev, CONTROL, 0, &c->reg,
				     sizeof(c->reg));
	return err;
}

static int rmi4_f01_update_queries(struct rmi4_function_device *fdev, bool read)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(&fdev->dev);
	struct rmi4_f01_basic_queries *q = &data->queries;

	dev_dbg(&fdev->dev, "%s - %s query data\n", __func__,
		 read ? "Reading" : "Writing");

	if (read)
		err = rmi4_bus_read(fdev, QUERY, 0, (u8 *)&q->reg,
				    sizeof(q->reg));
	else
		err = rmi4_bus_write(fdev, QUERY, 0, (u8 *)&q->reg,
				     sizeof(q->reg));

	return err;
}

static int rmi4_f01_update_status(struct rmi4_function_device *fdev, bool read)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(&fdev->dev);
	struct rmi4_f01_device_status *s = &data->status;

	dev_dbg(&fdev->dev, "%s - %s status data\n", __func__,
		 read ? "Reading" : "Writing");

	dev_dbg(&fdev->dev, "%s - Status code: %d\n", __func__,
		 data->status.status_code);
	dev_dbg(&fdev->dev, "%s - flash_prog: %d\n", __func__,
		 data->status.flash_prog);
	dev_dbg(&fdev->dev, "%s - Unconfigured: %d\n", __func__,
		 data->status.unconfigured);

	if (read)
		err = rmi4_bus_read(fdev, DATA, 0, (u8 *)&s->reg,
				    sizeof(s->reg));
	else
		err = rmi4_bus_write(fdev, DATA, 0, (u8 *)&s->reg,
				     sizeof(s->reg));

	return err;
}

static ssize_t rmi4_f01_productinfo_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);
	err = rmi4_f01_update_queries(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to read version\n", __func__);
		goto exit;
	}

	err = snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
		       data->queries.productinfo_1,
		       data->queries.productinfo_2);

exit:
	mutex_unlock(&data->lock);

	return err;
}

static ssize_t rmi4_f01_productid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi4_f01_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->product_name);
}

static ssize_t rmi4_f01_manufacturer_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_queries(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to read version\n", __func__);
		goto exit;
	}

	err = snprintf(buf, PAGE_SIZE, "0x%02x\n",
		       data->queries.manufacturer_id);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_datecode_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_queries(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to read version\n", __func__);
		goto exit;
	}

	err =  snprintf(buf, PAGE_SIZE, "20%02u-%02u-%02u\n",
			data->queries.year, data->queries.month,
			data->queries.day);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_reset_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int err = -EINVAL;
	unsigned int reset;
	/* Command register always reads as 0, so we can just use a local. */
	union f01_device_commands commands;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	if (1 != sscanf(buf, "%u", &reset) || 1 != reset)
		return -EINVAL;

	mutex_lock(&data->lock);
	commands.reset = 1;
	err = rmi4_bus_write(fdev, COMMAND, 0, &commands.reg,
			     sizeof(commands.reg));
	if (0 > err) {
		dev_err(dev, "%s: failed to issue reset command, "
			"error = %d.", __func__, err);
		goto exit;
	}
	dev_dbg(dev, "%s - Successfully reset chip\n", __func__);
	data->reset_forced = true;
exit:
	mutex_unlock(&data->lock);
	return err ? err : count;
}

static ssize_t rmi4_f01_sleepmode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_queries(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to read version\n", __func__);
		goto exit;
	}

	err = snprintf(buf, PAGE_SIZE, "%d\n", data->control.sleep_mode);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_sleepmode_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	u8 old;
	int err;
	unsigned long new_value;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &new_value);
	if (err < 0 || !RMI4_IS_VALID_SLEEPMODE(new_value)) {
		dev_err(dev, "%s: Invalid sleep mode %s.", __func__, buf);
		return -EINVAL;
	}

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to read version\n", __func__);
		goto exit;
	}

	old = data->control.sleep_mode;
	data->control.sleep_mode = new_value;

	err = rmi4_f01_update_control(fdev, false);
	if (0 > err) {
		dev_err(dev, "Failed to write sleep mode.\n");
		data->control.sleep_mode = old;
		goto exit;
	}

exit:
	mutex_unlock(&data->lock);
	return (0 > err) ? err : count;
}

static ssize_t rmi4_f01_nosleep_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err) {
		dev_err(dev, "%s - Failed to read nosleep. Err: %d\n",
			__func__, err);
		goto exit;
	}

	err = snprintf(buf, PAGE_SIZE, "%d\n", data->control.nosleep);

exit:
	mutex_unlock(&data->lock);

	return (0 > err) ? err : 0;
}

static ssize_t rmi4_f01_nosleep_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	u8 old;
	int err;
	unsigned long new_value;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &new_value);
	if (err < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid nosleep bit %s.", __func__, buf);
		return -EINVAL;
	}

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err)
		goto exit;

	old = data->control.nosleep;
	data->control.nosleep = new_value;

	err = rmi4_f01_update_control(fdev, false);
	if (0 > err) {
		dev_err(dev, "Failed to write nosleep bit.\n");
		data->control.nosleep = old;
		goto exit;
	}

exit:
	mutex_unlock(&data->lock);
	return (0 > err) ? err : count;
}

static ssize_t rmi4_f01_chargerinput_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err)
		goto exit;

	err = snprintf(buf, PAGE_SIZE, "%d\n", data->control.charger_input);
exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_chargerinput_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	u8 old;
	int err;
	unsigned long new_value;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &new_value);
	if (0 > err || new_value > 1) {
		dev_err(dev, "%s: Invalid chargerinput bit %s.", __func__, buf);
		return -EINVAL;
	}

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err)
		goto exit;

	old = data->control.charger_input;
	data->control.charger_input = new_value;

	err = rmi4_f01_update_control(fdev, false);
	if (0 > err) {
		data->control.charger_input = old;
		dev_err(dev, "Failed to write chargerinput bit.\n");
	}

exit:
	mutex_unlock(&data->lock);
	return (0 > err) ? err : 0;
}

static ssize_t rmi4_f01_reportrate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err)
		goto exit;

	err =  snprintf(buf, PAGE_SIZE, "%d\n", data->control.report_rate);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_reportrate_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	u8 old;
	int err;
	unsigned long new_value;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &new_value);
	if (0 > err || new_value > 1) {
		dev_err(dev, "%s: Invalid reportrate bit %s.", __func__, buf);
		return -EINVAL;
	}

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err)
		goto exit;

	old = data->control.report_rate;
	data->control.report_rate = new_value;

	err = rmi4_f01_update_control(fdev, false);
	if (0 > err) {
		data->control.report_rate = old;
		dev_err(dev, "Failed to write chargerinput bit.\n");
	}

exit:
	mutex_unlock(&data->lock);
	return (0 > err) ? err : 0;
}

static ssize_t rmi4_f01_configured_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device  *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err)
		goto exit;

	err =  snprintf(buf, PAGE_SIZE, "%d\n", data->control.configured);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_unconfigured_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_status(fdev, true);
	if (0 > err)
		goto exit;

	err =  snprintf(buf, PAGE_SIZE, "%d\n", data->status.unconfigured);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_flashprog_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_status(fdev, true);
	if (0 > err)
		goto exit;

	err =  snprintf(buf, PAGE_SIZE, "%d\n", data->status.flash_prog);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static ssize_t rmi4_f01_statuscode_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&data->lock);

	err = rmi4_f01_update_status(fdev, true);
	if (0 > err)
		goto exit;

	err =  snprintf(buf, PAGE_SIZE, "0x%02x\n", data->status.status_code);

exit:
	mutex_unlock(&data->lock);
	return err;
}

static void irq_handler(int irq_id, void *data)
{
	int err;

	struct rmi4_function_device *fdev = data;
	struct rmi4_f01_data *ddata = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	mutex_lock(&ddata->lock);

	err = rmi4_f01_update_status(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to update current status\n",
			__func__);
		goto exit;
	}

	if (ddata->status.unconfigured) {
		dev_dbg(&fdev->dev, "Reset detected\n");
		ddata->control.configured = 1;
		err = rmi4_f01_update_control(fdev, true);
		if (0 > err) {
			dev_err(&fdev->dev, "%s - Failed to update control\n", __func__);
		}

		if (ddata->reset_forced) {
			dev_dbg(&fdev->dev, "%s - Reset has been forced\n",
				 __func__);
			ddata->reset_forced = false;
			rmi4_bus_notify(fdev, RMI4_DRIVER_RESET);
		} else {
			dev_dbg(&fdev->dev, "%s - Spurious reset detected\n",
				 __func__);
		}
	}
exit:
	mutex_unlock(&ddata->lock);
}

static int rmi4_f01_set_sleep_mode(struct rmi4_function_device *fdev,
				   bool suspend)
{
	u8 old;
	int err;
	struct rmi4_f01_data *data = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s %s\n", suspend ? "Suspending" : "Resuming",
		 dev_name(&fdev->dev));

	mutex_lock(&data->lock);
	if (data->suspended == suspend) {
		dev_warn(&fdev->dev, "Device %s Already %s\n",
			 dev_name(&fdev->dev),
			 suspend ? "suspended" : "awoken");
		err = 0;
		goto done;
	}

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev,
			"%s - Failed to read current sleep status\n",
			__func__);
		goto done;
	}

	dev_dbg(&fdev->dev, "%s - Current sleep mode = 0x%02X\n", __func__,
		 data->control.sleep_mode);

	old = data->control.sleep_mode;
	if (suspend)
		data->control.sleep_mode = RMI4_SLEEP_MODE_SENSOR_SLEEP;
	else
		data->control.sleep_mode = RMI4_SLEEP_MODE_NORMAL;

	dev_dbg(&fdev->dev, "%s - New sleep mode = 0x%02X\n", __func__,
		 data->control.sleep_mode);

	err = rmi4_f01_update_control(fdev, false);
	if (0 > err) {
		data->control.sleep_mode = old;
		dev_err(&fdev->dev, "%s - Failed to %s\n", __func__,
			suspend ? "suspend" : "wake up");
		goto done;
	}

	data->suspended = suspend;

	dev_dbg(&fdev->dev, "Successfully %s %s\n",
		 suspend ? "suspended" : "woke up", dev_name(&fdev->dev));

done:
	mutex_unlock(&data->lock);
	return (0 > err) ? err : 0;
}

static int rmi4_f01_suspend(struct rmi4_function_device *fdev)
{
	int err = 0;

	struct rmi4_f01_data *data = dev_get_drvdata(&fdev->dev);

	if (0 == atomic_cmpxchg(&data->chip_sleeping, 0, 1)) {
		dev_dbg(&fdev->dev, "Suspending\n");
		err = rmi4_f01_set_sleep_mode(fdev, true);
	} else {
		dev_dbg(&fdev->dev, "Chip already in suspend mode\n");
		atomic_inc(&data->chip_sleeping);
	}

	return err;
}

static int rmi4_f01_resume(struct rmi4_function_device *fdev)
{
	int err = 0;
	struct rmi4_f01_data *data = dev_get_drvdata(&fdev->dev);

	if (1 == atomic_cmpxchg(&data->chip_sleeping, 1, 0)) {
		dev_dbg(&fdev->dev, "Resuming\n");
		err = rmi4_f01_set_sleep_mode(fdev, false);
	} else {
		dev_dbg(&fdev->dev, "Chip still sleeping by other request\n");
		atomic_dec(&data->chip_sleeping);
	}

	return err;
}

#ifdef CONFIG_PM
static int rmi4_f01_pm_suspend(struct device *dev)
{
	return rmi4_f01_suspend(to_rmi4_func_core(dev));
}

static int rmi4_f01_pm_resume(struct device *dev)
{
	return rmi4_f01_resume(to_rmi4_func_core(dev));
}
#else
#define rmi4_f01_pm_suspend NULL
#define rmi4_f01_pm_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(rmi4_f01_pm_ops, rmi4_f01_pm_suspend,
			 rmi4_f01_pm_resume);

static struct device_attribute rmi4_f01_attrs[] = {
	__ATTR(productinfo, S_IRUGO, rmi4_f01_productinfo_show, NULL),
	__ATTR(productid, S_IRUGO, rmi4_f01_productid_show, NULL),
	__ATTR(manufacturer, S_IRUGO, rmi4_f01_manufacturer_show, NULL),
	__ATTR(datecode, S_IRUGO, rmi4_f01_datecode_show, NULL),

	/* control register access */
	__ATTR(sleepmode, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	       rmi4_f01_sleepmode_show, rmi4_f01_sleepmode_store),
	__ATTR(nosleep, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	       rmi4_f01_nosleep_show, rmi4_f01_nosleep_store),
	__ATTR(chargerinput, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	       rmi4_f01_chargerinput_show, rmi4_f01_chargerinput_store),
	__ATTR(reportrate, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
	       rmi4_f01_reportrate_show, rmi4_f01_reportrate_store),
	__ATTR(configured, S_IRUGO, rmi4_f01_configured_show, NULL),

	/* Command register access. */
	__ATTR(reset, S_IWUSR, NULL, rmi4_f01_reset_store),

	/* Status register access. */
	__ATTR(unconfigured, S_IRUGO, rmi4_f01_unconfigured_show, NULL),
	__ATTR(flashprog, S_IRUGO, rmi4_f01_flashprog_show, NULL),
	__ATTR(statuscode, S_IRUGO, rmi4_f01_statuscode_show, NULL),
};

static int rmi4_f01_create_sysfs_files(struct rmi4_function_device *fdev,
				       bool create)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(rmi4_f01_attrs); i++) {
		if (create) {
			err = sysfs_create_file(&fdev->dev.kobj,
						&rmi4_f01_attrs[i].attr);
			if (err) {
				dev_err(&fdev->dev,
					"%s - Failed to create sysfs files\n",
					__func__);
				goto fail;
			}
		} else {
			sysfs_remove_file(&fdev->dev.kobj,
					  &rmi4_f01_attrs[i].attr);
		}
	}

	return err;

fail:
	for (; i >= 0; i--)
		sysfs_remove_file(&fdev->dev.kobj, &rmi4_f01_attrs[i].attr);

	return err;
}

static void rmi4_f01_notify_callback(enum rmi4_notification_event event,
				     void *data)
{
	struct rmi4_function_device *fdev = data;

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	if (event & RMI4_DRIVER_RESET) {
		dev_dbg(&fdev->dev, "%s - Driver has been reset\n", __func__);
	} else if (event & RMI4_CHIP_SUSPEND) {
		dev_dbg(&fdev->dev, "%s - Got external suspend event\n",
			 __func__);
		rmi4_f01_suspend(fdev);
	} else if (event & RMI4_CHIP_WAKEUP) {
		dev_dbg(&fdev->dev, "%s - Got external resume event\n",
			 __func__);
		rmi4_f01_resume(fdev);
	} else {
		dev_warn(&fdev->dev, "%s - Notification not supported\n",
			 __func__);
	}
}

static int rmi4_f01_start(struct rmi4_function_device *fdev)
{
	int err;
	struct rmi4_f01_data *data = kzalloc(sizeof(*data), GFP_KERNEL);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	if (!data) {
		dev_err(&fdev->dev, "%s - Failed to allocate local data\n",
			__func__);
		return -ENOMEM;
	}

	mutex_init(&data->lock);
	dev_set_drvdata(&fdev->dev, data);

	err = rmi4_bus_read(fdev, QUERY, RMI4_PROD_NAME_OFFSET,
			    &data->product_name[0], RMI4_PRODUCT_SIZE);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to product ID\n", __func__);
		goto err_clear_loc_data;
	}

	err = rmi4_f01_update_control(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s Failed to read initial control state\n",
			__func__);
		goto err_clear_loc_data;
	}

	err = rmi4_f01_update_queries(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s Failed to read initial query state\n",
			__func__);
		goto err_clear_loc_data;
	}

	err = rmi4_f01_update_status(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s Failed to read initial status\n",
			__func__);
		goto err_clear_loc_data;
	}

	data->control.configured = 1;
	err = rmi4_f01_update_control(fdev, false);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to enable chip\n", __func__);
		goto err_clear_loc_data;
	}

	err = rmi4_f01_create_sysfs_files(fdev, true);
	if (err) {
		dev_err(&fdev->dev, "%s - Failed to create sysfs files\n",
			__func__);
		goto err_clear_loc_data;
	}

	if (rmi4_bus_request_irq(fdev, fdev, irq_handler, 1)) {
		dev_err(&fdev->dev, "%s - Failed to subscribe to IRQ\n",
			__func__);
		goto err_free_sysfs;
	}

	err = rmi4_bus_request_notification(fdev,
					    RMI4_DRIVER_RESET |
					    RMI4_CHIP_SUSPEND |
					    RMI4_CHIP_WAKEUP,
					    rmi4_f01_notify_callback, fdev);
	if (err) {
		dev_err(&fdev->dev,
			"%s - Failed to subscribe to notification events\n",
			__func__);
		goto err_notify;
	}

	dev_info(&fdev->dev, "Product type: %s\n", data->product_name);
	dev_info(&fdev->dev, "Version ID : = 0x%02X 0x%02X (%u.%u)\n",
		 data->queries.productinfo_1, data->queries.productinfo_2,
		 data->queries.productinfo_1, data->queries.productinfo_2);
	dev_info(&fdev->dev, "Production date: 20%02u-%02u-%02u\n",
		 data->queries.year, data->queries.month, data->queries.day);

	dev_info(&fdev->dev, "Successfully initialized %s\n",
		 dev_name(&fdev->dev));

	return err;

err_notify:
	rmi4_bus_free_irq(fdev, fdev);
err_free_sysfs:
	rmi4_f01_create_sysfs_files(fdev, false);
err_clear_loc_data:
	kfree(data);
	return err;
}

static int rmi4_f01_stop(struct rmi4_function_device *fdev)
{
	struct rmi4_f01_data *data = dev_get_drvdata(&fdev->dev);
	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	rmi4_bus_free_irq(fdev, fdev);
	rmi4_f01_create_sysfs_files(fdev, false);
	kfree(data);
	dev_info(&fdev->dev, "%s - Released\n", dev_name(&fdev->dev));
	return 0;
}

static struct rmi4_function_driver rmi4_f01 = {
	.drv = {
		.name		= "f01",
		.pm		= &rmi4_f01_pm_ops,
	},
	.probe		= rmi4_f01_start,
	.remove	= rmi4_f01_stop,
};

static int __devinit rmi4_f01_init(void)
{
	pr_info("Registering function %s on RMI4 bus\n",
		rmi4_f01.drv.name);
	return rmi4_bus_register_function_driver(&rmi4_f01);
}

static void __devexit rmi4_f01_exit(void)
{
	pr_info("Unregistering function %s from RMI4 bus\n",
		rmi4_f01.drv.name);
	rmi4_bus_unregister_function_driver(&rmi4_f01);
}

module_init(rmi4_f01_init);
module_exit(rmi4_f01_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonyericsson.com>");
MODULE_DESCRIPTION("RMI4 F01 function driver");
MODULE_LICENSE("GPL");
