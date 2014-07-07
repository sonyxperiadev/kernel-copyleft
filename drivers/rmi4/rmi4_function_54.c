/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2012 SonyEricsson Mobile communications AB
 * Copyright (C) 2012 Sony Mobile Communications AB
 *
 * Authors: Daniel Rosenberg <daniel.rosenberg@synaptics.com>
 *	    Joachim Holst <joachim.holst@sonymobile.com>
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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/rmi4/rmi4.h>
#include <linux/rmi4/rmi4_function_54.h>
#include <linux/stat.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/errno.h>

enum rmi4_f54_status {
	STATUS_IDLE,
	STATUS_BUSY,
};

enum rmi4_f54_commands {
	CMD_GET_REPORT	= 1,
	CMD_FORCE_CAL,
};

/* Offsets for data */
#define RMI4_F54_REPORT_DATA_OFFSET	3
#define RMI4_F54_FIFO_OFFSET		1
#define RMI4_F54_NUM_TX_OFFSET		1
#define RMI4_F54_NUM_RX_OFFSET		0

/* Fixed sizes of reports */
#define RMI4_54_FULL_RAW_CAP_MIN_MAX_SIZE	4
#define RMI4_54_HIGH_RESISTANCE_SIZE		6

enum rmi4_f54_report_types {
	/* Manually indexing this sine it's not completely in series */
	F54_8BIT_IMAGE				= 1,
	F54_16BIT_IMAGE			= 2,
	F54_RAW_16BIT_IMAGE			= 3,
	F54_HIGH_RESISTANCE			= 4,
	F54_TX_TO_TX_SHORT			= 5,

	F54_RX_TO_RX1				= 7,

	F54_TRUE_BASELINE			= 9,

	F54_FULL_RAW_CAP_MIN_MAX		= 13,
	F54_RX_OPENS1				= 14,
	F54_TX_OPEN				= 15,
	F54_TX_TO_GROUND			= 16,
	F54_RX_TO_RX2				= 17,
	F54_RX_OPENS2				= 18,
	F54_FULL_RAW_CAP			= 19,
	F54_FULL_RAW_CAP_RX_COUPLING_COMP	= 20
};

struct rmi4_f54_ad_query {
	/* query 0 */
	u8 number_of_receiver_electrodes;

	/* query 1 */
	u8 number_of_transmitter_electrodes;

	union {
		struct {
			/* query2 */
			u8 f54_ad_query2_b0__1:2;
			u8 has_baseline:1;
			u8 has_image8:1;
			u8 f54_ad_query2_b4__5:2;
			u8 has_image16:1;
			u8 f54_ad_query2_b7:1;
		};
		u8 f54_ad_query2;
	};

	/* query 3.0 and 3.1 */
	u16 clock_rate;

	/* query 4 */
	u8 touch_controller_family;

	/* query 5 */
	union {
		struct {
			u8 has_pixel_touch_threshold_adjustment:1;
			u8 f54_ad_query5_b1__7:7;
		};
		u8 f54_ad_query5;
	};

	/* query 6 */
	union {
		struct {
		u8 has_sensor_assignment:1;
		u8 has_interference_metric:1;
		u8 has_sense_frequency_control:1;
		u8 has_firmware_noise_mitigation:1;
		u8 f54_ad_query6_b4:1;
		u8 has_two_byte_report_rate:1;
		u8 has_one_byte_report_rate:1;
		u8 has_relaxation_control:1;
		};
		u8 f54_ad_query6;
	};

	/* query 7 */
	union {
		struct {
			u8 curve_compensation_mode:2;
			u8 f54_ad_query7_b2__7:6;
		};
		u8 f54_ad_query7;
	};

	/* query 8 */
	union {
		struct {
		u8 f54_ad_query2_b0:1;
		u8 has_iir_filter:1;
		u8 has_cmn_removal:1;
		u8 has_cmn_maximum:1;
		u8 has_pixel_threshold_hysteresis:1;
		u8 has_edge_compensation:1;
		u8 has_perf_frequency_noisecontrol:1;
		u8 f54_ad_query8_b7:1;
		};
		u8 f54_ad_query8;
	};

	u8 f54_ad_query9;
	u8 f54_ad_query10;
	u8 f54_ad_query11;

	/* query 12 */
	union {
		struct {
			u8 number_of_sensing_frequencies:4;
			u8 f54_ad_query12_b4__7:4;
		};
		u8 f54_ad_query12;
	};
};

struct rmi4_f54_drvdata {
	struct rmi4_f54_ad_query query;
	struct rmi4_function_device *fdev;

	u8 cmd;
	enum rmi4_f54_report_types report_type;
	u16 fifoindex;
	u8 status;
	bool no_auto_cal;

	unsigned int report_size;
	u8 *report_data;
	unsigned int bufsize;

	struct mutex data_mutex;
	struct mutex status_mutex;

#ifdef CONFIG_RMI4_F54_WATCHDOG
	struct hrtimer watchdog;
	struct work_struct work;
#endif

	u8 num_rx_electrodes;
	u8 num_tx_electrodes;
};

static int rmi4_f54_update_query(struct rmi4_function_device *fdev, bool read)
{
	int err;
	struct rmi4_f54_drvdata *data = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - %s query data\n", __func__,
		read ? "Reading" : "Writing");

	if (read)
		err = rmi4_bus_read(fdev, QUERY, 0, (u8 *)&data->query,
				    sizeof(data->query));
	else
		err = rmi4_bus_write(fdev, QUERY, 0, (u8 *)&data->query,
				     sizeof(data->query));

	return err;
}

static void set_report_size(struct rmi4_f54_drvdata *data)
{
	u8 rx = data->num_rx_electrodes;
	u8 tx = data->num_tx_electrodes;

	dev_dbg(&data->fdev->dev, "%s - Called\n", __func__);
	switch (data->report_type) {
	case F54_8BIT_IMAGE:
		data->report_size = rx * tx;
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		data->report_size = 2 * rx * tx;
		break;
	case F54_HIGH_RESISTANCE:
		data->report_size = RMI4_54_HIGH_RESISTANCE_SIZE;
		break;
	case F54_FULL_RAW_CAP_MIN_MAX:
		data->report_size = RMI4_54_FULL_RAW_CAP_MIN_MAX_SIZE;
		break;
	case F54_TX_TO_TX_SHORT:
	case F54_TX_OPEN:
	case F54_TX_TO_GROUND:
		data->report_size =  (tx + 7) / 8;
		break;
	case F54_RX_TO_RX1:
	case F54_RX_OPENS1:
		if (rx < tx)
			data->report_size = 2 * rx * rx;
		else
			data->report_size = 2 * rx * tx;
		break;
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
		if (rx <= tx)
			data->report_size = 0;
		else
			data->report_size = 2 * rx * (rx - tx);
		break;
	default:
		data->report_size = 0;
	}

	dev_dbg(&data->fdev->dev, "%s - Report size set to: %u\n", __func__,
		data->report_size);
}

#ifdef CONFIG_RMI4_F54_DBG_HUMAN_READABLE
static void rmi4_f54_report_hr_data(struct rmi4_function_device *fdev)
{
	/* TODO: This human redable printout is most likely completely
	 *	 incorrect. Since SEMC currently doesn't have any specification
	 *	 on the F54 RMI4 function, we don't really know if the data is
	 *	 signed or unsigned. We also don't know between which values
	 *	 the data should be, so we can't really fix this function.
	 *	 Would appreciate if Synaptics could update this function and
	 *	 the values to get a correct readout of the data that is
	 *	 understandable by humans. */
	int i;
	int j;
	/* We use s16 here in order to avoid build warnings. This is most
	 * likely not correct though */
	s16 tmp;
	int count = 0;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(&fdev->dev);
	int size = ddata->num_rx_electrodes * ddata->num_tx_electrodes;
	u16 *data = (u16 *)ddata->report_data;
	/* Add space for data and additional chars (EOF\n + \0) */
	char buf[size  + ddata->num_tx_electrodes + 6];

	memset(buf, 0, sizeof(buf));

	dev_dbg(&fdev->dev, "Data size = %d\n", size);
	dev_dbg(&fdev->dev, "Buf size = %u\n", sizeof(buf));
	dev_dbg(&fdev->dev, "Num TX electrodes: %d\n",
		 ddata->num_tx_electrodes);
	dev_dbg(&fdev->dev, "Num RX electrodes: %d\n",
		 ddata->num_rx_electrodes);

	for (i = 0; i < ddata->num_tx_electrodes && count < sizeof(buf); i++) {
		/* The ranges used below are probably too low or at least
		 * incorrect. Needs update */
		for (j = 0; j < ddata->num_rx_electrodes &&
			     count < sizeof(buf); j++) {
			tmp = le16_to_cpu(data[j]);
			if (tmp < -64)
				count += snprintf(buf + count, sizeof(buf),
						 "%s", ".");
			else if (tmp < 0)
				count += snprintf(buf + count, sizeof(buf),
						 "%s", "-");
			else if (tmp > 64)
				count += snprintf(buf + count, sizeof(buf),
						 "%s", "*");
			else if (tmp > 0)
				count += snprintf(buf + count, sizeof(buf),
						 "%s", "+");
			else
				count += snprintf(buf + count, sizeof(buf),
						 "%s", "0");
			dev_dbg(&fdev->dev, "rec: count = %d\n", count);
			dev_dbg(&fdev->dev, "j = %d\n", j);
		}
		count += snprintf(buf + count, sizeof(buf),
				 "%s", "\n");
		dev_dbg(&fdev->dev, "trans: count = %d\n", count);
		dev_dbg(&fdev->dev, "i = %d\n", i);
	}
	count += snprintf(buf + count, sizeof(buf),
			 "%s", "EOF\n");

	dev_info(&fdev->dev, "Report data (Image):\n%s", buf);
}
#endif

static void rmi4_f54_dump_raw_hex(struct rmi4_function_device *fdev)
{
#ifdef CONFIG_RMI4_F54_DBG_RAW_HEX
	int i;
	u16 tmp;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(&fdev->dev);
	u16 *data = (u16 *)ddata->report_data;
	int len = ddata->report_size / 2;

	for (i = 0; i < len; i++) {
		tmp = le16_to_cpu(data[i]);
		dev_info(&fdev->dev, "%03d: 0x%04X\n", i, tmp);
	}
#endif
}

static void rmi4_f54_irqhandler(int irq_id, void *data)
{
	u8 fifo[2];
	int error;
	struct rmi4_function_device *fdev = data;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	set_report_size(ddata);

	if (ddata->report_size == 0) {
		dev_err(&fdev->dev, "%s - Invalid report type set.\n",
			__func__);
		return;
	}

	if (ddata->bufsize < ddata->report_size) {
		mutex_lock(&ddata->data_mutex);
		if (ddata->bufsize > 0)
			kfree(ddata->report_data);

		ddata->report_data = kzalloc(ddata->report_size, GFP_KERNEL);
		if (!ddata->report_data) {
			dev_err(&fdev->dev,
				"Failed to allocate report_data.\n");
			ddata->bufsize = 0;
			mutex_unlock(&ddata->data_mutex);
			error = -ENOMEM;
			goto error_exit;
		}
		ddata->bufsize = ddata->report_size;
		mutex_unlock(&ddata->data_mutex);
	}

	dev_dbg(&fdev->dev, "F54 Interrupt handler is running.\nSize: %d\n",
		ddata->report_size);

	fifo[0] = 0;
	fifo[1] = 0;
	error = rmi4_bus_write(fdev, DATA, RMI4_F54_FIFO_OFFSET, fifo,
			       sizeof(fifo));
	if (error < 0)
		dev_err(&fdev->dev, "Failed to write fifo to zero!\n");
	else
		error = rmi4_bus_read(fdev, DATA, RMI4_F54_REPORT_DATA_OFFSET,
				      ddata->report_data, ddata->report_size);

	if (error < 0  || error != ddata->report_size) {
		dev_err(&fdev->dev, "F54 data read failed. Code: %d.\n", error);
		error = -EINVAL;
		goto error_exit;
	}

	rmi4_f54_dump_raw_hex(fdev);

#ifdef CONFIG_RMI4_F54_DBG_HUMAN_READABLE
	switch (ddata->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		rmi4_f54_report_hr_data(fdev);
		break;
	default:
		dev_info(&fdev->dev, "Report type %d debug image not supported",
			 ddata->report_type);
	}
#endif

	error = 0;

error_exit:
	mutex_lock(&ddata->status_mutex);
	error = rmi4_bus_set_non_essential_irq_status(fdev, false);
	if (error)
		dev_err(&fdev->dev,
			"%s - Failed to re-enable non essential IRQ's\n",
			__func__);

	ddata->status = STATUS_IDLE;
	mutex_unlock(&ddata->status_mutex);
}


#ifdef CONFIG_RMI4_F54_WATCHDOG
static void rmi4_f54_clear_status_worker(struct work_struct *work)
{
	int result;
	u8 command;
	struct rmi4_f54_drvdata *data =
		container_of(work, struct rmi4_f54_drvdata, work);

	mutex_lock(&data->status_mutex);
	if (data->status == STATUS_BUSY) {
		dev_dbg(&data->fdev->dev,
			"F54 Timout Occured: Determining status.\n");
		result = rmi4_bus_read(data->fdev, COMMAND, 0, &command, 1);
		if (result < 0) {
			dev_err(&data->fdev->dev,
				"Could not read get_report register\n");
			goto done;
		}

		dev_dbg(&data->fdev->dev, "%s - read command = %u\n", __func__,
			command);

		if (command & CMD_GET_REPORT)
			dev_warn(&data->fdev->dev,
				 "%s - Report type unsupported!",
				__func__);
		else
			dev_dbg(&data->fdev->dev, "%s - Timeout\n",
				__func__);

		dev_dbg(&data->fdev->dev,
			 "%s - Restoring interrupts\n", __func__);
		if (rmi4_bus_set_non_essential_irq_status(data->fdev,
							      false))
			dev_err(&data->fdev->dev,
				"%s - Failed to restore interrupts\n",
				__func__);
	}

done:
	mutex_unlock(&data->status_mutex);
	data->status = STATUS_IDLE;

}

static enum hrtimer_restart rmi4_f54_clear_status(struct hrtimer *timer)
{
	struct rmi4_f54_drvdata *data =
		container_of(timer, struct rmi4_f54_drvdata, watchdog);

	schedule_work(&data->work);
	return HRTIMER_NORESTART;
}
#endif

/* Check if report_type is valid */
static bool rmi4_f54_is_report_type_valid(enum rmi4_f54_report_types reptype)
{
	/* TODO: Check Query3 to see if some specific reports are
	 * available. This is currently listed as a reserved register.
	 */
	switch (reptype) {
	case F54_8BIT_IMAGE:
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_HIGH_RESISTANCE:
	case F54_TX_TO_TX_SHORT:
	case F54_RX_TO_RX1:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_RX_OPENS1:
	case F54_TX_OPEN:
	case F54_TX_TO_GROUND:
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		return true;
		break;
	default:
		return false;
	}
}

static ssize_t rmi4_f54_report_type_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct rmi4_f54_drvdata *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->report_type);
}

static ssize_t rmi4_f54_report_type_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int err;
	u8 data;
	unsigned long val;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &val);
	if (err)
		return err;

	if (!rmi4_f54_is_report_type_valid(val)) {
		dev_err(dev, "%s : Report type %u is invalid.\n",
			__func__, (u8)val);
		return -EINVAL;
	}

	mutex_lock(&ddata->status_mutex);
	if (ddata->status != STATUS_BUSY) {
		data = val;
		err = rmi4_bus_write(fdev, DATA, 0, &data, 1);
		if (err < 0) {
			dev_err(dev, "%s : Could not write report type\n",
				__func__);
			goto done;
		}
		ddata->report_type = data;
		dev_dbg(&fdev->dev, "%s - Requested report type %u\n", __func__,
			data);
	} else {
		dev_err(dev, "%s : Report type cannot be changed in the middle"
				" of command.\n", __func__);
		err = -EINVAL;
	}
done:
	mutex_unlock(&ddata->status_mutex);
	return (0 > err) ? err : count;
}

static ssize_t rmi4_f54_get_report_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count) {
	int err;
	u8 command;
	unsigned long val;
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	dev_dbg(dev, "%s - Called\n", __func__);

	err = strict_strtoul(buf, 10, &val);
	if (err)
		return err;

	if (1 != val) {
		dev_warn(&fdev->dev, "%s - value %lu not valid. Only 1 is\n",
			 __func__, val);
		return -EINVAL;
	}

	command = CMD_GET_REPORT;

	mutex_lock(&ddata->status_mutex);
	if (ddata->status != STATUS_IDLE && ddata->status != STATUS_BUSY) {
		dev_err(dev, "F54 status is in an abnormal state: 0x%x",
			ddata->status);
		err = -EFAULT;
		goto done;
	}

	/* Store interrupts */
	/* Do not exit if we fail to turn off interupts. We are likely
	 * to still get useful data. The report data can, however, be
	 * corrupted, and there may be unexpected behavior.
	 */
	dev_dbg(dev, "Storing and overriding interupts\n");
	rmi4_bus_set_non_essential_irq_status(fdev, true);
	ddata->status = STATUS_BUSY;

	/* small delay to avoid race condition in firmare. This value is a bit
	 * higher than absolutely necessary. Should be removed once issue is
	 * resolved in firmware. */
	mdelay(2);

	err = rmi4_bus_write(fdev, COMMAND, 0, &command, 1);
	if (err < 0) {
		dev_err(dev, "%s : Could not write command\n", __func__);
		goto done;
	}

#ifdef CONFIG_RMI4_F54_WATCHDOG
	hrtimer_start(&ddata->watchdog, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif

done:
	mutex_unlock(&ddata->status_mutex);
	return (err < 0) ? err : count;
}

static ssize_t rmi4_f54_force_cal_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	u8 command;
	unsigned long val;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &val);
	if (err)
		return err;

	if (val != 1)
		return count;

	command = CMD_FORCE_CAL;

	if (ddata->status == STATUS_BUSY)
		return -EBUSY;

	err = rmi4_bus_write(fdev, COMMAND, 0, &command, 1);
	if (err < 0) {
		dev_err(dev, "%s : Could not write force cal command\n",
			__func__);
		return err;
	}
	return count;
}

static ssize_t rmi4_f54_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ddata->status);
}

static ssize_t rmi4_f54_num_rx_electrodes_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->num_rx_electrodes);
}

static ssize_t rmi4_f54_num_tx_electrodes_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->num_tx_electrodes);
}

static ssize_t rmi4_f54_has_image16_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_image16);
}

static ssize_t rmi4_f54_has_image8_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", ddata->query.has_image8);
}

static ssize_t rmi4_f54_has_baseline_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", ddata->query.has_baseline);
}

static ssize_t rmi4_f54_clock_rate_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", ddata->query.clock_rate);
}


static ssize_t rmi4_f54_touch_controller_family_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.touch_controller_family);
}


static ssize_t rmi4_f54_has_pixel_touch_threshold_adjustment_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_pixel_touch_threshold_adjustment);
}

static ssize_t rmi4_f54_has_sensor_assignment_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_sensor_assignment);
}

static ssize_t rmi4_f54_has_interference_metric_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_interference_metric);
}

static ssize_t rmi4_f54_has_sense_frequency_control_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_sense_frequency_control);
}

static ssize_t rmi4_f54_has_firmware_noise_mitigation_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_firmware_noise_mitigation);
}

static ssize_t rmi4_f54_has_two_byte_report_rate_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_two_byte_report_rate);
}

static ssize_t rmi4_f54_has_one_byte_report_rate_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_one_byte_report_rate);
}

static ssize_t rmi4_f54_has_relaxation_control_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_relaxation_control);
}

static ssize_t rmi4_f54_curve_compensation_mode_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.curve_compensation_mode);
}

static ssize_t rmi4_f54_has_iir_filter_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_iir_filter);
}

static ssize_t rmi4_f54_has_cmn_removal_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_cmn_removal);
}

static ssize_t rmi4_f54_has_cmn_maximum_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_cmn_maximum);
}

static ssize_t rmi4_f54_has_pixel_threshold_hysteresis_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_pixel_threshold_hysteresis);
}

static ssize_t rmi4_f54_has_edge_compensation_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_edge_compensation);
}

static ssize_t rmi4_f54_has_perf_frequency_noisecontrol_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.has_perf_frequency_noisecontrol);
}

static ssize_t rmi4_f54_number_of_sensing_frequencies_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->query.number_of_sensing_frequencies);
}


static ssize_t rmi4_f54_no_auto_cal_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			ddata->no_auto_cal ? 1 : 0);
}

static ssize_t rmi4_f54_no_auto_cal_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int err;
	unsigned long val;
	unsigned char data;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &val);
	if (err)
		return err;

	if (val > 1)
		return count;

	err = rmi4_bus_read(fdev, CONTROL, 0, &data, 1);
	if (0 > err) {
		dev_err(dev, "%s - Failed to read initial status", __func__);
		return err;
	}

	if ((data & 1) == val)
		return count;

	data = (data & ~1) | (val & 0x01);
	err = rmi4_bus_write(fdev, CONTROL, 0, &data, 1);
	if (err < 0) {
		dev_err(dev, "%s : Could not write control\n", __func__);
		return err;
	}

	ddata->no_auto_cal = (val == 1);
	return count;
}

static ssize_t rmi4_f54_fifoindex_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int err;
	u16 data;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = rmi4_bus_read(fdev, DATA, RMI4_F54_FIFO_OFFSET,
			    (u8 *)&data, sizeof(data));
	if (0 > err) {
		dev_err(dev, "%s - Could not read fifoindex\n", __func__);
		return err;
	}
	ddata->fifoindex = le16_to_cpu(data);

	return snprintf(buf, PAGE_SIZE, "%u\n", ddata->fifoindex);
}

static ssize_t rmi4_f54_fifoindex_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	int err;
	u16 data;
	unsigned long val;
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	err = strict_strtoul(buf, 10, &val);
	if (err)
		return err;

	data = cpu_to_le16((u16)val);

	err = rmi4_bus_write(fdev, DATA, RMI4_F54_FIFO_OFFSET, (u8 *)&data,
			     sizeof(data));
	if (0 > err) {
		dev_err(dev, "%s : Could not write fifoindex\n", __func__);
		return err;
	}

	ddata->fifoindex = val;

	return count;
}

static ssize_t rmi4_f54_data_read(struct file *data_file, struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count)
{
	int err = 0;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->data_mutex);
	if (count < ddata->report_size) {
		dev_err(dev,
			"%s: F54 report size too large for buffer: %d."
			" Need at least: %d for Report type: %d.\n",
			__func__, count, ddata->report_size,
			ddata->report_type);
		err = -EINVAL;
		goto done;
	}
	if (ddata->report_data) {
		memcpy(buf, ddata->report_data,
		       ddata->report_size);
		dev_dbg(dev, "%s: Presumably successful.", __func__);
		err = ddata->report_size;
		goto done;
	} else {
		dev_err(dev, "%s: F54 report_data does not exist!\n", __func__);
		err = -EINVAL;
	}

done:
	mutex_unlock(&ddata->data_mutex);
	return err;
}

static struct device_attribute rmi4_f54_attrs[] = {
	__ATTR(report_type, S_IWUGO | S_IRUGO, rmi4_f54_report_type_show,
	       rmi4_f54_report_type_store),
	__ATTR(get_report, S_IWUGO, NULL, rmi4_f54_get_report_store),
	__ATTR(force_cal, S_IWUGO, NULL, rmi4_f54_force_cal_store),
	__ATTR(status, S_IRUGO, rmi4_f54_status_show, NULL),
	__ATTR(num_rx_electrodes, S_IRUGO, rmi4_f54_num_rx_electrodes_show,
	       NULL),
	__ATTR(num_tx_electrodes, S_IRUGO, rmi4_f54_num_tx_electrodes_show,
	       NULL),
	__ATTR(has_image16, S_IRUGO, rmi4_f54_has_image16_show, NULL),
	__ATTR(has_image8, S_IRUGO, rmi4_f54_has_image8_show, NULL),
	__ATTR(has_baseline, S_IRUGO, rmi4_f54_has_baseline_show, NULL),
	__ATTR(clock_rate, S_IRUGO, rmi4_f54_clock_rate_show, NULL),
	__ATTR(touch_controller_family, S_IRUGO,
	       rmi4_f54_touch_controller_family_show, NULL),
	__ATTR(has_pixel_touch_threshold_adjustment, S_IRUGO,
	       rmi4_f54_has_pixel_touch_threshold_adjustment_show, NULL),
	__ATTR(has_sensor_assignment, S_IRUGO,
	       rmi4_f54_has_sensor_assignment_show, NULL),
	__ATTR(has_interference_metric, S_IRUGO,
	       rmi4_f54_has_interference_metric_show, NULL),
	__ATTR(has_sense_frequency_control, S_IRUGO,
	       rmi4_f54_has_sense_frequency_control_show, NULL),
	__ATTR(has_firmware_noise_mitigation, S_IRUGO,
	       rmi4_f54_has_firmware_noise_mitigation_show, NULL),
	__ATTR(has_two_byte_report_rate, S_IRUGO,
	       rmi4_f54_has_two_byte_report_rate_show, NULL),
	__ATTR(has_one_byte_report_rate, S_IRUGO,
	       rmi4_f54_has_one_byte_report_rate_show, NULL),
	__ATTR(has_relaxation_control, S_IRUGO,
	       rmi4_f54_has_relaxation_control_show, NULL),
	__ATTR(curve_compensation_mode, S_IRUGO,
	       rmi4_f54_curve_compensation_mode_show, NULL),
	__ATTR(has_iir_filter, S_IRUGO, rmi4_f54_has_iir_filter_show, NULL),
	__ATTR(has_cmn_removal, S_IRUGO, rmi4_f54_has_cmn_removal_show, NULL),
	__ATTR(has_cmn_maximum, S_IRUGO, rmi4_f54_has_cmn_maximum_show, NULL),
	__ATTR(has_pixel_threshold_hysteresis, S_IRUGO,
	       rmi4_f54_has_pixel_threshold_hysteresis_show, NULL),
	__ATTR(has_edge_compensation, S_IRUGO,
	       rmi4_f54_has_edge_compensation_show, NULL),
	__ATTR(has_perf_frequency_noisecontrol, S_IRUGO,
	       rmi4_f54_has_perf_frequency_noisecontrol_show, NULL),
	__ATTR(number_of_sensing_frequencies, S_IRUGO,
	       rmi4_f54_number_of_sensing_frequencies_show, NULL),
	__ATTR(no_auto_cal, S_IWUGO | S_IRUGO, rmi4_f54_no_auto_cal_show,
	       rmi4_f54_no_auto_cal_store),
	__ATTR(fifoindex, S_IWUGO | S_IRUGO, rmi4_f54_fifoindex_show,
	       rmi4_f54_fifoindex_store),
};

static struct bin_attribute rmi4_f54_rep_data = {
	.attr = {
		.name = "rep_data",
		.mode = S_IRUGO,
	},
	.size = 0,
	.read = rmi4_f54_data_read,
};

static int rmi4_f54_create_sysfs_files(struct rmi4_function_device *fdev,
				       bool create)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(rmi4_f54_attrs); i++) {
		if (create)
			err = sysfs_create_file(&fdev->dev.kobj,
						&rmi4_f54_attrs[i].attr);
		else
			sysfs_remove_file(&fdev->dev.kobj,
					  &rmi4_f54_attrs[i].attr);
		if (err) {
			dev_err(&fdev->dev, "Failed to create sysfs files\n");
			goto fail;
		}
	}

	if (create) {
		err = sysfs_create_bin_file(&fdev->dev.kobj,
					    &rmi4_f54_rep_data);
		if (err) {
			dev_err(&fdev->dev,
				"Failed to create binary sysfs files\n");
			goto fail;
		}
	} else {
		sysfs_remove_bin_file(&fdev->dev.kobj, &rmi4_f54_rep_data);
	}

	return err;

fail:
	for (i--; i >= 0; i--)
		sysfs_remove_file(&fdev->dev.kobj,
				  &rmi4_f54_attrs[i].attr);

	return err;
}

static int rmi4_f54_start(struct rmi4_function_device *fdev)
{
	int err;
	struct rmi4_f54_drvdata *data = kzalloc(sizeof(*data), GFP_KERNEL);
	struct rmi4_function_54_platform_data *pdata =
		dev_get_platdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	if (!data) {
		dev_err(&fdev->dev, "%s - Failed to allocate local data\n",
			__func__);
		return -ENOMEM;
	}
	dev_set_drvdata(&fdev->dev, data);
	data->fdev = fdev;

#ifdef CONFIG_RMI4_F54_WATCHDOG
	/* Set up watchdog timer to catch unanswered get_report commands */
	hrtimer_init(&data->watchdog, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->watchdog.function = rmi4_f54_clear_status;
	INIT_WORK(&data->work, rmi4_f54_clear_status_worker);
#endif

	mutex_init(&data->data_mutex);
	mutex_init(&data->status_mutex);

	err = rmi4_f54_update_query(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to update query\n",
			__func__);
		goto err_query;
	}

	if (pdata) {
		/* This check doesn't work with rev 0x41 of Odin FW.
		 * According to info from Synaptics, this should report the
		 * number of available electrodes. Not the number of used
		 * electrodes. Oding has 23 RX and 13TX electrodes, but we only
		 * manage to read out 6 RX and 1 TX which is completely wrong.
		 * Disabling this until future time.
		 *
		if (pdata->num_rx_electrodes >
		    data->query.number_of_receiver_electrodes) {
			dev_err(&fdev->dev, "Chip supports %d RX electrodes. "
				"%d is out of range.\n",
				data->query.number_of_receiver_electrodes,
				pdata->num_rx_electrodes);
			goto err_query;
		}

		if (pdata->num_tx_electrodes >
		    data->query.number_of_transmitter_electrodes) {
			dev_err(&fdev->dev, "Chip supports %d TX electrodes. "
				"%d is out of range.\n",
				data->query.number_of_transmitter_electrodes,
				pdata->num_tx_electrodes);
			goto err_query;
		}
		* End of disabled code */
		data->num_rx_electrodes = pdata->num_rx_electrodes;
		data->num_tx_electrodes = pdata->num_tx_electrodes;
	} else {
		dev_info(&fdev->dev,
			 "%s - No platform data found. Using defaults.\n",
			 __func__);
		data->num_rx_electrodes =
			data->query.number_of_receiver_electrodes;
		data->num_tx_electrodes =
			data->query.number_of_transmitter_electrodes;
	}

	err = rmi4_f54_create_sysfs_files(fdev, true);
	if (err) {
		dev_err(&fdev->dev, "%s - Failed to create sysfs files\n",
			__func__);
		goto err_query;
	}

	err = rmi4_bus_request_irq(fdev, fdev, rmi4_f54_irqhandler, 1);
	if (err) {
		dev_err(&fdev->dev, "%s - Failed to subscribe to IRQ\n",
			__func__);
		goto err_irq_request;
	}

	dev_info(&fdev->dev, "Successfully registered\n");
	goto done;

err_irq_request:
	rmi4_f54_create_sysfs_files(fdev, false);
err_query:
	dev_info(&fdev->dev, "Failed to register\n");
	dev_set_drvdata(&fdev->dev, NULL);
	mutex_destroy(&data->data_mutex);
	mutex_destroy(&data->status_mutex);
	kfree(data);

done:
	return err;
}

static int rmi4_f54_stop(struct rmi4_function_device *fdev)
{
	struct rmi4_f54_drvdata *ddata = dev_get_drvdata(&fdev->dev);

#ifdef CONFIG_RMI4_F54_WATCHDOG
	hrtimer_cancel(&ddata->watchdog);
#endif

	rmi4_bus_free_irq(fdev, fdev);
	rmi4_f54_create_sysfs_files(fdev, false);
	dev_set_drvdata(&fdev->dev, NULL);
	mutex_destroy(&ddata->data_mutex);
	mutex_destroy(&ddata->status_mutex);
	kfree(ddata);
	return 0;
}

static struct rmi4_function_driver rmi4_f54 = {
	.drv = {
		.name		= "f54",
	},
	.probe		= rmi4_f54_start,
	.remove	= __devexit_p(rmi4_f54_stop),
};

static int __devinit rmi4_f54_init(void)
{
	pr_info("Registering function %s on RMI4 bus\n", rmi4_f54.drv.name);
	return rmi4_bus_register_function_driver(&rmi4_f54);
}

static void __devexit rmi4_f54_exit(void)
{
	pr_info("Unregistering function %s from RMI4 bus\n",
		rmi4_f54.drv.name);
	rmi4_bus_unregister_function_driver(&rmi4_f54);
}


module_init(rmi4_f54_init);
module_exit(rmi4_f54_exit);

MODULE_AUTHOR("Daniel Rosenberg <daniel.rosenberg@synaptics.com>," \
	      "Joachim Holst <joachim.holst@sonyericsson.com>");
MODULE_DESCRIPTION("RMI F54 module");
MODULE_LICENSE("GPL");
