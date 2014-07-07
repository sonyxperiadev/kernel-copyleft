/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2012 Sony Ericsson mobile communications AB
 * Copyright (c) 2012 Sony Mobile Communications AB
 *
 * Author: Unknown
 * Author: Joachim Holst <joachim.holst@sonymobile.com>
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
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/rmi4/rmi4.h>
#include <linux/gpio.h>

#define REG_ADDR_LIMIT 0xFFFF
#define RMI4_CHAR_DEVICE_NAME "rmi"

struct rmi4_char_dev_drv_data {
	struct mutex fop_lock;
	struct cdev cdev;
	struct rmi4_function_device *fdev;
	int ref_count;

	struct rmi4_char_dev_func_data *fdata;

};

/*
 * rmi_char_dev_llseek: - use to setup register address
 *
 * @filp: file structure for seek
 * @off: offset
 *       if whence == SEEK_SET,
 *       high 16 bits: page address
 *       low 16 bits: register address
 *
 *       if whence == SEEK_CUR,
 *       offset from current position
 *
 *       if whence == SEEK_END,
 *       offset from END(0xFFFF)
 *
 * @whence: SEEK_SET , SEEK_CUR or SEEK_END
 */
static loff_t rmi4_char_dev_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;
	struct rmi4_char_dev_drv_data *ddata = filp->private_data;
	if (IS_ERR_OR_NULL(ddata)) {
		pr_err("%s: pointer of char device is invalid", __func__);
		return -EBADF;
	}

	mutex_lock(&ddata->fop_lock);

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;

	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;

	case SEEK_END:
		newpos = REG_ADDR_LIMIT + off;
		break;

	default:/* can't happen. Just to keep compiler happy */
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > REG_ADDR_LIMIT) {
		dev_err(&ddata->fdev->dev, "newpos 0x%04x is invalid.\n",
			(unsigned int)newpos);
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:
	mutex_unlock(&ddata->fop_lock);
	return newpos;
}

/*
 *  rmi_char_dev_read: - use to read data from RMI stream
 *
 *  @filp: file structure for read
 *  @buf: user-level buffer pointer
 *
 *  @count: number of byte read
 *  @f_pos: offset (starting register address)
 *
 *	@return number of bytes read into user buffer (buf) if succeeds
 *          negative number if error occurs.
 */
static ssize_t rmi4_char_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *f_pos)
{
	ssize_t retval;
	unsigned char tmpbuf[count+1];
	struct rmi4_char_dev_drv_data *ddata = filp->private_data;
	if (IS_ERR_OR_NULL(ddata)) {
		pr_err("%s: pointer of char device is invalid", __func__);
		return -EBADF;
	}

	/* limit offset to REG_ADDR_LIMIT-1 */
	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0) {
		dev_info(&ddata->fdev->dev, "%s - Tried to read 0 bytes\n",
			 __func__);
		return 0;
	}

	mutex_lock(&ddata->fop_lock);

	retval = ddata->fdata->read(ddata->fdev, *f_pos, tmpbuf, count);

	if (retval < 0)
		goto clean_up;

	*f_pos += retval;

	if (copy_to_user(buf, tmpbuf, count))
		retval = -EFAULT;

clean_up:

	mutex_unlock(&ddata->fop_lock);

	return retval;
}

/*
 * rmi_char_dev_write: - use to write data into RMI stream
 *
 * @filep : file structure for write
 * @buf: user-level buffer pointer contains data to be written
 * @count: number of byte be be written
 * @f_pos: offset (starting register address)
 *
 * @return number of bytes written from user buffer (buf) if succeeds
 *         negative number if error occurs.
 */
static ssize_t rmi4_char_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *f_pos)
{
	ssize_t retval;
	unsigned char tmpbuf[count+1];
	struct rmi4_char_dev_drv_data *ddata = filp->private_data;
	if (IS_ERR_OR_NULL(ddata)) {
		pr_err("%s: pointer of char device is invalid", __func__);
		return -EBADF;
	}

	/* limit offset to REG_ADDR_LIMIT-1 */
	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0) {
		dev_info(&ddata->fdev->dev, "%s - Tried to write 0 bytes\n",
			 __func__);
		return 0;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		dev_err(&ddata->fdev->dev,
			"%s - Failed to copy incoming data\n",
			__func__);
		return -EFAULT;
	}

	mutex_lock(&ddata->fop_lock);

	retval = ddata->fdata->write(ddata->fdev, *f_pos, tmpbuf, count);

	if (retval >= 0)
		*f_pos += count;

	mutex_unlock(&ddata->fop_lock);

	return retval;
}

static int rmi4_char_dev_open(struct inode *inp, struct file *filp)
{
	int retval = 0;
	struct rmi4_char_dev_drv_data *ddata =
		container_of(inp->i_cdev, struct rmi4_char_dev_drv_data,
			     cdev);


	filp->private_data = ddata;

	mutex_lock(&ddata->fop_lock);
	if (ddata->ref_count < 1)
		ddata->ref_count++;
	else
		retval = -EACCES;

	mutex_unlock(&ddata->fop_lock);

	return retval;
}

static int rmi4_char_dev_release(struct inode *inp, struct file *filp)
{
	struct rmi4_char_dev_drv_data *ddata =
		container_of(inp->i_cdev, struct rmi4_char_dev_drv_data,
			     cdev);

	mutex_lock(&ddata->fop_lock);

	ddata->ref_count--;
	if (ddata->ref_count < 0)
		ddata->ref_count = 0;

	filp->private_data = NULL;

	mutex_unlock(&ddata->fop_lock);

	return 0;
}

static const struct file_operations rmi4_char_dev_fops = {
	.owner =    THIS_MODULE,
	.llseek =   rmi4_char_dev_llseek,
	.read =     rmi4_char_dev_read,
	.write =    rmi4_char_dev_write,
	.open =     rmi4_char_dev_open,
	.release =  rmi4_char_dev_release,
};

static int rmi4_char_dev_register(struct rmi4_function_device *fdev)
{
	int err;
	struct device *device_ptr;
	struct rmi4_char_dev_drv_data *ddata = kzalloc(sizeof(*ddata),
						       GFP_KERNEL);

	if (!ddata) {
		dev_err(&fdev->dev, "Failed to allocate driver data.\n");
		return -ENOMEM;
	}

	mutex_init(&ddata->fop_lock);
	ddata->fdev = fdev;
	dev_set_drvdata(&fdev->dev, ddata);
	ddata->fdata = dev_get_platdata(&fdev->dev);

	device_ptr = rmi4_bus_create_char_dev(RMI4_CHAR_DEVICE_NAME,
					      &rmi4_char_dev_fops,
					      &ddata->cdev, &fdev->dev,
					      ddata);
	if (IS_ERR(device_ptr)) {
		dev_err(&fdev->dev, "%s - Failed to create char dev\n",
			__func__);
		err = PTR_ERR(device_ptr);
		goto error;
	}

	err = gpio_export(ddata->fdata->gpio, false);
	if (err) {
		dev_warn(&fdev->dev, "WARNING: Failed to export ATTN gpio!\n");
		goto error;
	}

	err = gpio_export_link(&fdev->dev, "attn", ddata->fdata->gpio);
	if (err)
		dev_warn(&fdev->dev,
			 "WARNING: Failed to symlink ATTN gpio!\n");
	else
		dev_info(&fdev->dev,
			 "%s: Exported GPIO %d.", __func__, ddata->fdata->gpio);

	return 0;
error:
	dev_set_drvdata(&fdev->dev, NULL);
	mutex_destroy(&ddata->fop_lock);
	kfree(ddata);
	return err;
}

static int rmi4_char_dev_unregister(struct rmi4_function_device *fdev)
{
	struct rmi4_char_dev_drv_data *ddata = dev_get_drvdata(&fdev->dev);

	rmi4_bus_destroy_char_dev(&fdev->dev, &ddata->cdev);

	dev_set_drvdata(&fdev->dev, NULL);
	mutex_destroy(&ddata->fop_lock);
	kfree(ddata);

	return 0;
}

static struct rmi4_function_driver rmi4_char_dev = {
	.drv = {
		.name		= RMI4_CHAR_DEV_NAME,
	},
	.probe		= rmi4_char_dev_register,
	.remove	= __devexit_p(rmi4_char_dev_unregister),
};

static int __devinit rmi4_char_dev_init(void)
{
	pr_info("Registering function %s on RMI4 bus\n",
		rmi4_char_dev.drv.name);
	return rmi4_bus_register_function_driver(&rmi4_char_dev);
}

static void __devexit rmi4_char_dev_exit(void)
{
	pr_info("Unregistering function %s from RMI4 bus\n",
		rmi4_char_dev.drv.name);
	rmi4_bus_unregister_function_driver(&rmi4_char_dev);
}

module_init(rmi4_char_dev_init);
module_exit(rmi4_char_dev_exit);

MODULE_AUTHOR("Synaptics, Inc,."					\
	      "Joachim Holst <joachim.holst@sonymobile.com>");
MODULE_DESCRIPTION("RMI4 char driver");
MODULE_LICENSE("GPL");
