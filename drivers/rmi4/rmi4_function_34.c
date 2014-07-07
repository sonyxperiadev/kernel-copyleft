/*
 * RMI4 bus driver.
 * driver/rmi4/rmi4_function_34.c
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
#include <linux/rmi4/rmi4_function_34.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/swab.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#define RMI4_F34_BLK_NUM_OFFSET	0x02
#define RMI4_F34_BLK_SIZE_OFFSET	0x03
#define RMI4_F34_IMG_BLK_CNT_OFFSET	0x05
#define RMI4_F34_CFG_BLK_CNT_OFFSET	0x07

/* Status report */
#define RMI4_F34_STATUS_IDLE		0x00
#define RMI4_F34_STATUS_OK		0x80
#define RMI4_F34_STATUS_FAILED		0xFF

#define RMI4_F34_MODESWITCH_TIMEOUT	msecs_to_jiffies(200)
#define RMI4_F34_ERASE_TIMEOUT		msecs_to_jiffies(10000)
#define RMI4_F34_PROGRAM_TIMEOUT	msecs_to_jiffies(50)
#define RMI4_F34_FLASH_POLL_DELAY	msecs_to_jiffies(1)

#define RMI4_F34_FLASH_COMMAND_MASK	0x0F

#define RMI4_F34_FLASH_WRITE_FW_BLOCK		0x02
#define RMI4_F34_FLASH_ERASE_ALL		0x03
#define RMI4_F34_READ_CONFIG_BLOCK		0x05
#define RMI4_F34_FLASH_WRITE_CONFIG_BLOCK	0x06
#define RMI4_F34_ERASE_CONFIG			0x07
#define RMI4_F34_FLASH_PROG_START		0x0F


static void rmi4_f34_irqhandler(int irq_id, void *data);

static ssize_t rmi4_f34_data_write(struct file *file, struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count);

struct rmi4_f34_status_reg {
	union {
		struct {
			u8 flash_command:4;
			u8 flash_status:3;
			u8 flash_enabled:1;
		};
		u8 reg;
	};
};

struct rmi4_f34_ctrl0_3_reg {
	union {
		struct {
			u8 module_id;
			u8 revision_id;
			u8 developer_id;
			u8 personal_test_id;
		};
		u8 reg[4];
	};
};

struct rmi4_f34_query2_reg {
	union {
		struct {
			u8 reg_map:1;
			u8 unlocked:1;
			u8 has_config_id:1;
			u8 reserved:5;
		};
		u8 reg;
	};
};

struct rmi4_f34_data {
	struct mutex f34_lock;
	struct rmi4_f34_status_reg stat;
	struct rmi4_f34_ctrl0_3_reg ctrl0_3;
	struct rmi4_f34_query2_reg q2_reg;

	struct rmi4_function_device *fdev;

	u8 cmd;
	u8 status;
	u8 state;
	u16 bootloader_id;
	u16 block_size;
	u16 image_block_count;
	u16 config_block_count;
	u16 block_num;
	bool in_flash_prog_mode;

	/* Workaround for bad bootloaders */
	struct delayed_work poll_work;
	bool use_polling;
	bool do_polling;

	struct completion action_complete;
};

struct bin_attribute rmi4_f34_bin_attr = {
	.attr = {
		.name = "data",
		.mode = 0700
	},
	.size = 0,
	.write = rmi4_f34_data_write,
};

static int rmi4_f34_refresh_ctrl0_reg(struct rmi4_function_device *fdev)
{
	struct rmi4_f34_data *data = dev_get_drvdata(&fdev->dev);

	return rmi4_bus_read(fdev, CONTROL, 0, &data->ctrl0_3.reg[0],
			     sizeof(data->ctrl0_3.reg));
}

static int rmi4_f34_refresh_query2_reg(struct rmi4_function_device *fdev)
{
	struct rmi4_f34_data *data = dev_get_drvdata(&fdev->dev);

	return rmi4_bus_read(fdev, QUERY, 2, &data->q2_reg.reg,
			     sizeof(data->q2_reg.reg));
}

static int rmi4_f34_update_extra_data(struct rmi4_function_device *fdev)
{
	int err;
	struct rmi4_f34_data *data = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	err = rmi4_f34_refresh_query2_reg(fdev);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to update query2 register\n",
			__func__);
		goto done;
	}

	if (data->q2_reg.has_config_id) {
		dev_info(&fdev->dev, "Chip has config ID\n");
		err = rmi4_f34_refresh_ctrl0_reg(fdev);
		if (0 > err) {
			dev_err(&fdev->dev,
				"%s - Failed to update control0_3 register\n",
				__func__);
			goto done;
		}
		dev_info(&fdev->dev,
			 "Module ID: 0x%02X (%d), revision ID: 0x%02X (%d)\n",
			 data->ctrl0_3.module_id, data->ctrl0_3.module_id,
			 data->ctrl0_3.revision_id, data->ctrl0_3.revision_id);
	} else {
		dev_info(&fdev->dev, "Chip doesn't have config ID\n");
	}

done:
	return (0 > err) ? err : 0;
}

static int rmi4_f34_update_status(struct rmi4_function_device *fdev, bool read)
{
	int err;
	struct rmi4_f34_data *data = dev_get_drvdata(&fdev->dev);

	if (read)
		err = rmi4_bus_read(fdev, DATA, data->block_size +
				    RMI4_F34_BLK_NUM_OFFSET,
				    &data->stat.reg,
				    sizeof(data->stat.reg));
	else
		err = rmi4_bus_write(fdev, DATA, data->block_size +
				     RMI4_F34_BLK_NUM_OFFSET,
				     &data->stat.reg,
				     sizeof(data->stat.reg));

	return err;
}

static int rmi4_f34_create_bin_dev(struct rmi4_function_device *fdev,
				   bool create)
{
	int err = 0;

	if (create)
		err = sysfs_create_bin_file(&fdev->dev.kobj,
					    &rmi4_f34_bin_attr);
	else
		sysfs_remove_bin_file(&fdev->dev.kobj,
				      &rmi4_f34_bin_attr);

	return err;
}

static ssize_t rmi4_f34_image_block_count_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->f34_lock);
	err = snprintf(buf, PAGE_SIZE, "%u\n", ddata->image_block_count);
	mutex_unlock(&ddata->f34_lock);

	return err;
}

static ssize_t rmi4_f34_config_block_count_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->f34_lock);
	err = snprintf(buf, PAGE_SIZE, "%u\n", ddata->config_block_count);
	mutex_unlock(&ddata->f34_lock);

	return err;
}

static ssize_t rmi4_f34_has_config_id_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->f34_lock);
	err = snprintf(buf, PAGE_SIZE, "%d\n", ddata->q2_reg.has_config_id);
	mutex_unlock(&ddata->f34_lock);

	return err;
}

static ssize_t rmi4_f34_module_id_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->f34_lock);
	if (!ddata->q2_reg.has_config_id) {
		mutex_unlock(&ddata->f34_lock);
		return snprintf(buf, PAGE_SIZE, "N/A\n");
	}

	err = snprintf(buf, PAGE_SIZE, "0x%02X\n", ddata->ctrl0_3.module_id);
	mutex_unlock(&ddata->f34_lock);

	return err;
}

static ssize_t rmi4_f34_revision_id_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->f34_lock);
	if (!ddata->q2_reg.has_config_id) {
		mutex_unlock(&ddata->f34_lock);
		return snprintf(buf, PAGE_SIZE, "N/A\n");
	}

	err = snprintf(buf, PAGE_SIZE, "0x%02X\n", ddata->ctrl0_3.revision_id);
	mutex_unlock(&ddata->f34_lock);

	return err;
}
static ssize_t rmi4_f34_developer_id_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->f34_lock);
	if (!ddata->q2_reg.has_config_id) {
		mutex_unlock(&ddata->f34_lock);
		return snprintf(buf, PAGE_SIZE, "N/A\n");
	}

	err = snprintf(buf, PAGE_SIZE, "%c\n", ddata->ctrl0_3.developer_id);
	mutex_unlock(&ddata->f34_lock);

	return err;
}

static ssize_t rmi4_f34_personal_test_id_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);

	mutex_lock(&ddata->f34_lock);

	if (!ddata->q2_reg.has_config_id) {
		mutex_unlock(&ddata->f34_lock);
		return snprintf(buf, PAGE_SIZE, "N/A\n");
	}

	err = snprintf(buf, PAGE_SIZE, "0x%02X\n",
		       ddata->ctrl0_3.personal_test_id);
	mutex_unlock(&ddata->f34_lock);

	return err;
}



static int rmi4_f34_parse_flash_status(struct rmi4_function_device *fdev,
				       u8 status)
{
	switch (status) {
	case 0:
		dev_dbg(&fdev->dev, "Flash status: Success\n");
		return 0;
	case 1:
		dev_dbg(&fdev->dev, "Flash status: Reserved\n");
		return -EINVAL;
	case 2:
		dev_dbg(&fdev->dev,
			 "Flash status: Not enabled / bad command\n");
		return -ENOSYS;
	case 3:
		dev_dbg(&fdev->dev, "Flash status: Invalid block number\n");
		return -EINVAL;
	case 4:
		dev_dbg(&fdev->dev, "Flash status: Block not erased\n");
		return -EFAULT;
	case 5:
		dev_dbg(&fdev->dev, "Flash status: Erase key incorrect\n");
		return -EINVAL;
	case 6:
		dev_dbg(&fdev->dev,
			 "Flash status: Unknown erase/program failure");
		return -EFAULT;
	case 7:
		dev_dbg(&fdev->dev, "Flash status: Device has been reset\n");
		return -EREMCHG;
	default:
		dev_dbg(&fdev->dev, "Invalid flash status found\n");
		return -EFAULT;
	}
}

static void rmi4_f34_status_poll(struct work_struct *work)
{
	int err;
	struct delayed_work *dwork = to_delayed_work(work);
	struct rmi4_f34_data *ddata = container_of(dwork, struct rmi4_f34_data,
						  poll_work);

	err = rmi4_f34_update_status(ddata->fdev, true);
	if (0 > err) {
		dev_err(&ddata->fdev->dev,
			"%s - Failed to update current status\n",
			__func__);
		return;
	}

	if (ddata->stat.reg != 0x80 && ddata->do_polling)
		schedule_delayed_work(&ddata->poll_work, 0);
	else
		complete(&ddata->action_complete);
}

static int rmi4_f34_run_combined_bl_id_command(
	struct rmi4_function_device *fdev, unsigned long timeout)
{
	int err;
	struct rmi4_f34_data *ddata = dev_get_drvdata(&fdev->dev);
	u16 blid = cpu_to_le16(ddata->bootloader_id);

	err = rmi4_bus_write(fdev, DATA, RMI4_F34_BLK_NUM_OFFSET,
			     (u8 *)&blid, sizeof(blid));
	if (0 > err) {
		dev_err(&fdev->dev,
			"%s : Failed to write bootloader id\n", __func__);
		return err;
	} else {
		dev_dbg(&fdev->dev, "%s : Successfully wrote BL ID\n\n",
			__func__);
	}

	ddata->stat.flash_command = ddata->cmd;
	err = rmi4_f34_update_status(fdev, false);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to write flash enable\n",
			__func__);
		return err;
	}

retry:
	init_completion(&ddata->action_complete);
	if (ddata->use_polling) {
		ddata->do_polling = true;
		schedule_delayed_work(&ddata->poll_work,
				      RMI4_F34_FLASH_POLL_DELAY);
	}

	err = wait_for_completion_timeout(&ddata->action_complete, timeout);
	if (!err) {
		if (!ddata->use_polling) {
			dev_warn(&fdev->dev,
				 "Bootloader bad. Swiched to polling mode\n");
			ddata->use_polling = true;
			rmi4_bus_free_irq(fdev, fdev);
			goto retry;
		}
		dev_err(&fdev->dev, "Command 0x%02X timed out\n",
			ddata->stat.flash_command &
			RMI4_F34_FLASH_COMMAND_MASK);
		ddata->do_polling = false;
		return -ETIMEDOUT;
	}

	return rmi4_f34_parse_flash_status(fdev, ddata->stat.flash_status);
}

static ssize_t rmi4_f34_cmd_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err = 0;
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	mutex_lock(&ddata->f34_lock);

	if (strnstr(buf, RMI4_ENTER_BL_MODE_CMD,
		    sizeof(RMI4_ENTER_BL_MODE_CMD))) {
		ddata->cmd = RMI4_F34_FLASH_PROG_START;
		dev_info(&fdev->dev, "Enabling FW update functionality\n");

		rmi4_bus_notify(fdev, RMI4_CHIP_BOOTLOADER);

		err = rmi4_bus_request_irq(fdev, fdev, rmi4_f34_irqhandler, 1);
		if (err) {
			dev_err(&fdev->dev, "%s - Failed to subscribe to IRQ\n",
				__func__);
			goto exit;
		}

		err = rmi4_f34_run_combined_bl_id_command(fdev,
						  RMI4_F34_MODESWITCH_TIMEOUT);
		/* All other errors are allowed at this stage */
		if (-ETIMEDOUT == err)
			goto exit;

		err = rmi4_bus_update_pdt(fdev);
		if (err) {
			dev_err(&fdev->dev, "%s - Failed to rescan PDT table\n",
				__func__);
			goto exit;
		}

		err = rmi4_f34_update_status(fdev, true);
		if (0 > err) {
			dev_err(&fdev->dev,
				"%s : Could not read status\n", __func__);
			goto exit;
		}

		err = rmi4_f34_parse_flash_status(fdev,
						  ddata->stat.flash_status);
		if (err) {
			dev_err(&fdev->dev,
				"%s - Failed to enable flash mode\n",
				__func__);
			goto exit;
		}

		if (!ddata->in_flash_prog_mode) {
			err = rmi4_f34_create_bin_dev(fdev, true);
			if (err) {
				dev_err(&fdev->dev,
					"Failed to create sysfs file\n");
				goto exit;
			}
		}

		ddata->in_flash_prog_mode = true;
		dev_info(&fdev->dev, "%s - Successfully entered BL mode\n",
			 __func__);
	} else if (strnstr(buf, RMI4_ERASE_CONFIG_CMD,
			   sizeof(RMI4_ERASE_CONFIG_CMD))) {
		ddata->cmd = RMI4_F34_ERASE_CONFIG;
		err = rmi4_f34_run_combined_bl_id_command(fdev,
						 RMI4_F34_ERASE_TIMEOUT);
		if (err)
			dev_err(&fdev->dev, "%s - Failed to erase config\n",
				__func__);
		else
			dev_info(&fdev->dev,
				 "%s - Successfully erased config\n",
				 __func__);
	} else if (strnstr(buf, RMI4_ERASE_ALL_CMD,
			   sizeof(RMI4_ERASE_ALL_CMD))) {
		ddata->cmd = RMI4_F34_FLASH_ERASE_ALL;
		err = rmi4_f34_run_combined_bl_id_command(fdev,
						 RMI4_F34_ERASE_TIMEOUT);
		if (err)
			dev_err(&fdev->dev, "%s - Failed to erase chip\n",
				__func__);
		else
			dev_info(&fdev->dev, "%s - Successfully erased chip\n",
				 __func__);
	} else if (strnstr(buf, RMI4_WRITE_FW_IMG_CMD,
			   sizeof(RMI4_WRITE_FW_IMG_CMD))) {
		ddata->cmd = RMI4_F34_FLASH_WRITE_FW_BLOCK;
		dev_info(&fdev->dev, "%s - Writing FW image\n",
			 __func__);
	} else if (strnstr(buf, RMI4_WRITE_CFG_IMG_CMD,
			   sizeof(RMI4_WRITE_CFG_IMG_CMD))) {
		ddata->cmd = RMI4_F34_FLASH_WRITE_CONFIG_BLOCK;
		dev_info(&fdev->dev,
			 "%s - Writing config image\n",
			 __func__);
	} else if (strnstr(buf, RMI4_FLASH_PROG_DONE_CMD,
			   sizeof(RMI4_FLASH_PROG_DONE_CMD))) {
		if (ddata->in_flash_prog_mode) {
			dev_info(&fdev->dev, "FW update done\n");
			sysfs_remove_bin_file(&fdev->dev.kobj,
					      &rmi4_f34_bin_attr);
			ddata->status = RMI4_F34_STATUS_IDLE;
			ddata->in_flash_prog_mode = false;
			ddata->do_polling = false;
			if (!ddata->use_polling)
				rmi4_bus_free_irq(fdev, fdev);
			ddata->use_polling = false;
			err = rmi4_f34_update_extra_data(fdev);
		} else {
			dev_info(&fdev->dev,
				 "%s - Flash mode not enabled. Can't disable\n",
				 __func__);
		}
	} else {
		dev_warn(dev, "%s: Unsupported command \"%s\"\n",
			 __func__, buf);
		err = -EINVAL;
	}

exit:
	mutex_unlock(&ddata->f34_lock);

	return (0 > err) ? err : count;
}

static ssize_t rmi4_f34_data_write(struct file *file, struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count)
{
	int err = -EINVAL;
	int i = count;
	u16 blocknum = 0;
	int bytes_written = 0;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi4_f34_data *ddata = dev_get_drvdata(dev);
	struct rmi4_function_device *fdev = to_rmi4_func_core(dev);

	dev_dbg(dev, "%s - pos = %d\n", __func__, (int)pos);

	if (RMI4_F34_FLASH_WRITE_FW_BLOCK != ddata->cmd &&
	    RMI4_F34_FLASH_WRITE_CONFIG_BLOCK != ddata->cmd) {
		dev_err(dev,
			"Not in fw image or cfg image state. No write data\n");
		return -EINVAL;
	}

	mutex_lock(&ddata->f34_lock);
	if (0 == pos) {
		err = rmi4_bus_write(fdev, DATA, 0, (u8 *)&blocknum,
				     sizeof(blocknum));
		if (0 > err) {
			dev_err(dev, "%s : Failed to write block data\n",
				__func__);
			goto exit;
		}
	}

	while (0 < i) {
		int num = ddata->block_size > i ? i : ddata->block_size;
		err = rmi4_bus_write(fdev, DATA, RMI4_F34_BLK_NUM_OFFSET,
				     buf, num);
		if (0 > err) {
			dev_err(dev, "%s : Failed to write block data\n",
				__func__);
			goto exit;
		}

		ddata->stat.flash_command = ddata->cmd;
		err = rmi4_f34_update_status(fdev, false);
		if (0 > err) {
			dev_err(&fdev->dev,
				"%s - Failed to write data block\n",
				__func__);
			goto exit;
		}

retry:
		init_completion(&ddata->action_complete);
		if (ddata->use_polling) {
			ddata->do_polling = true;
			schedule_delayed_work(&ddata->poll_work, 0);
		}

		err = wait_for_completion_timeout(&ddata->action_complete,
					    RMI4_F34_PROGRAM_TIMEOUT);
		if (!err) {
			if (!ddata->use_polling) {
				ddata->use_polling = true;
				rmi4_bus_free_irq(fdev, fdev);
				goto retry;
			}
			dev_err(&fdev->dev, "%s - Command 0x%02X timed out\n",
				__func__, ddata->cmd &
				RMI4_F34_FLASH_COMMAND_MASK);
			ddata->do_polling = false;
			err = -ETIMEDOUT;
			goto exit;
		}

		if (RMI4_F34_STATUS_FAILED == ddata->status) {
			dev_err(&fdev->dev,
				"%s - Flashing process failed. Unknonw error\n",
				__func__);
			err = -EFAULT;
			goto exit;
		}

		bytes_written += num;
		i -= num;
		buf += num;
	}

	dev_dbg(dev, "%s - Finished writing %d bytes\n", __func__,
		 bytes_written);

exit:
	mutex_unlock(&ddata->f34_lock);

	return (0 > err) ? err : count;
}


static struct device_attribute rmi4_f34_attrs[] = {
	__ATTR(cmd, S_IWUSR, NULL, rmi4_f34_cmd_store),
	__ATTR(imageblockcount, S_IRUGO, rmi4_f34_image_block_count_show,
	NULL),
	__ATTR(configblockcount, S_IRUGO, rmi4_f34_config_block_count_show,
	       NULL),
	__ATTR(has_config_id, S_IRUGO, rmi4_f34_has_config_id_show, NULL),
	__ATTR(module_id, S_IRUGO, rmi4_f34_module_id_show, NULL),
	__ATTR(revision_id, S_IRUGO, rmi4_f34_revision_id_show, NULL),
	__ATTR(developer_id, S_IRUGO, rmi4_f34_developer_id_show, NULL),
	__ATTR(personal_test_id, S_IRUGO, rmi4_f34_personal_test_id_show,
	       NULL),
};

static void rmi4_f34_irqhandler(int irq_id, void *data)
{
	int err;
	struct rmi4_function_device *fdev = data;
	struct rmi4_f34_data *ddata = dev_get_drvdata(&fdev->dev);

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	err = rmi4_f34_update_status(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev,
			"%s : Could not read status\n", __func__);
	}

	if (RMI4_F34_FLASH_WRITE_CONFIG_BLOCK == ddata->cmd &&
	    RMI4_F34_STATUS_IDLE == ddata->stat.reg) {
		dev_info(&fdev->dev, "Flashing complete\n");
	} else if (ddata->stat.reg != RMI4_F34_STATUS_OK) {
		ddata->status = RMI4_F34_STATUS_FAILED;
		dev_info(&fdev->dev, "reg = 0x%02X\n",
			 ddata->stat.reg);
		dev_err(&fdev->dev, "Flash failed\n");
	} else {
		ddata->status = RMI4_F34_STATUS_OK;
	}

	complete(&ddata->action_complete);

	dev_dbg(&fdev->dev, "%s - Status = 0x%02X\n", __func__,
		ddata->stat.reg & 0xF0);
}

static int rmi4_f34_start(struct rmi4_function_device *fdev)
{
	int err;
	int i;
	struct rmi4_f34_data *data;

	dev_dbg(&fdev->dev, "%s - Called\n", __func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&fdev->dev, "%s - Failed to allocate local data\n",
			__func__);
		return -ENOMEM;
	}
	init_completion(&data->action_complete);
	mutex_init(&data->f34_lock);
	INIT_DELAYED_WORK(&data->poll_work, rmi4_f34_status_poll);
	data->fdev = fdev;
	dev_set_drvdata(&fdev->dev, data);

	err = rmi4_bus_read(fdev, QUERY, 0,
			    (u8 *)&data->bootloader_id,
			    sizeof(data->bootloader_id));
	if (0 > err) {
		dev_err(&fdev->dev,
			"%s : Failed to read bootloader id from\n", __func__);
		goto error_boot_load_id;
	}

	data->bootloader_id = le16_to_cpu(data->bootloader_id);

	dev_info(&fdev->dev, "%s - BootloaderID = %d (0x%X)\n", __func__,
		 data->bootloader_id, data->bootloader_id);

	err = rmi4_bus_read(fdev, QUERY, RMI4_F34_BLK_SIZE_OFFSET,
			   (u8 *) &data->block_size,
			   sizeof(data->block_size));
	if (0 > err) {
		dev_err(&fdev->dev,
			"%s: Could not read block size\n", __func__);
		goto error_boot_load_id;
	}
	data->block_size = le16_to_cpu(data->block_size);
	dev_info(&fdev->dev, "%s - Block size = %d (0x%X)\n", __func__,
		 data->block_size, data->block_size);

	err = rmi4_bus_read(fdev, QUERY, RMI4_F34_IMG_BLK_CNT_OFFSET,
			   (u8 *) &data->image_block_count,
			   sizeof(data->image_block_count));
	if (0 > err) {
		dev_err(&fdev->dev,
			"%s: Could not read image block count\n", __func__);
		goto error_boot_load_id;
	}
	data->image_block_count = le16_to_cpu(data->image_block_count);
	dev_info(&fdev->dev, "%s - Image block count = %d (0x%X)\n", __func__,
		 data->image_block_count, data->image_block_count);

	err = rmi4_bus_read(fdev, QUERY, RMI4_F34_CFG_BLK_CNT_OFFSET,
			   (u8 *) &data->config_block_count,
			   sizeof(data->config_block_count));
	if (0 > err) {
		dev_err(&fdev->dev,
			"%s: Could not read config block count\n", __func__);
		goto error_boot_load_id;
	}
	data->config_block_count = le16_to_cpu(data->config_block_count);
	dev_info(&fdev->dev, "%s - Config block count = %d (0x%X)\n", __func__,
		 data->config_block_count, data->config_block_count);

	err = rmi4_f34_update_status(fdev, true);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to update current status\n",
			__func__);
		goto error_boot_load_id;
	}

	err = rmi4_f34_update_extra_data(fdev);
	if (0 > err) {
		dev_err(&fdev->dev, "%s - Failed to update query2 register\n",
			__func__);
		goto error_boot_load_id;
	}

	for (i = 0; i < ARRAY_SIZE(rmi4_f34_attrs); i++) {
		err = sysfs_create_file(&fdev->dev.kobj,
					&rmi4_f34_attrs[i].attr);
		if (err) {
			dev_err(&fdev->dev,
				"%s: Failed to create sysfs file for  %s.",
				__func__, rmi4_f34_attrs[i].attr.name);
			goto error_sysfs_create;
		}
	}

	if (data->stat.flash_enabled) {
		dev_info(&fdev->dev, "%s - Already in flash mode\n", __func__);
		data->in_flash_prog_mode = true;
		err = rmi4_f34_create_bin_dev(fdev, true);
		if (err) {
			dev_err(&fdev->dev, "%s - Failed to create bin file\n",
				__func__);
			goto error_sysfs_create;
		}
	}

	dev_info(&fdev->dev, "Successfully initialized RMI4 F34 Driver\n");

	return err;

error_sysfs_create:
	for (i--; i >= 0; i--)
		sysfs_remove_file(&fdev->dev.kobj, &rmi4_f34_attrs[i].attr);
error_boot_load_id:
	dev_set_drvdata(&fdev->dev, NULL);
	kfree(data);
	return err;
}

static int rmi4_f34_stop(struct rmi4_function_device *fdev)
{
	int i;
	struct rmi4_f34_data *data = dev_get_drvdata(&fdev->dev);

	dev_info(&fdev->dev, "%s - Called\n", __func__);

	sysfs_remove_bin_file(&fdev->dev.kobj, &rmi4_f34_bin_attr);

	for (i = 0; i < ARRAY_SIZE(rmi4_f34_attrs); i++)
		sysfs_remove_file(&fdev->dev.kobj, &rmi4_f34_attrs[i].attr);

	rmi4_bus_free_irq(fdev, fdev);

	dev_set_drvdata(&fdev->dev, NULL);
	mutex_destroy(&data->f34_lock);
	kfree(data);

	dev_info(&fdev->dev, "Successfully removed\n");

	return 0;
}

static struct rmi4_function_driver rmi4_f34 = {
	.drv = {
		.name = RMI4_DEFAULT_F34_NAME,
	},
	.probe		= rmi4_f34_start,
	.remove	= rmi4_f34_stop,
};

static int __devinit rmi4_f34_init(void)
{
	pr_info("Registering function %s on RMI4 bus\n",
		rmi4_f34.drv.name);
	return rmi4_bus_register_function_driver(&rmi4_f34);
}

static void __devexit rmi4_f34_exit(void)
{
	pr_info("Unregistering function %s from RMI4 bus\n",
		rmi4_f34.drv.name);
	rmi4_bus_unregister_function_driver(&rmi4_f34);
}

module_init(rmi4_f34_init);
module_exit(rmi4_f34_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonyericsson.com>");
MODULE_DESCRIPTION("RMI4 F01 function driver");
MODULE_LICENSE("GPL");
