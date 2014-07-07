/*
 * RMI4 bus driver.
 * drivers/rmi4/rmi4_i2c_adaptor.c
 *
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 * Copyright (C) 2011 Sony Ericsson mobile communications AB
 * Copyright (c) 2012 Sony Mobile Communications AB
 *
 * Author: Christopher Heiny <cheiny@synaptics.com>"
 * Author: Eric Andersson <eric.andersson@unixphere.com>"
 * Author: Joakim Wesslen <joakim.wesslen@sonyericsson.com>
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

/* #define DEBUG */

#include <linux/module.h>
#include <linux/rmi4/rmi4.h>
#include <linux/rmi4/rmi4_i2c.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#define RMI4_I2C_RETRIES	5

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)
#define RMI4_ADDR_SIZE	2
#define RMI4_HEADER_SIZE	4
#define RMI4_PACKET_SIZE	(RMI4_HEADER_SIZE + RMI4_ADDR_SIZE)


struct rmi4_i2c_driver_data {
	struct mutex page_mutex;
	int page;
	int (*set_page) (struct rmi4_core_device *cdev, unsigned int  page);
};

static void rmi4_i2c_buf_debug(struct i2c_client *client,
		const char *info, u16 addr, u8 *buf, int len, unsigned int page)
{
#ifdef DEBUG
	char b[len + 1];
	unsigned i, k;

	dev_dbg(&client->dev, "%s: -------------------\n", __func__);
	dev_dbg(&client->dev, "%s: %s: page:%d, addr:%.2x, len:%.2d\n",
					__func__, info, page, addr, len);
	for (i = k = 0; i < len; i++)
		k += snprintf(b + k, sizeof(b) - k, "%02x ", buf[i]);

	dev_dbg(&client->dev, "%s: %s\n", __func__, b);
	dev_dbg(&client->dev, "%s: -------------------\n", __func__);
#endif
}

static int rmi4_i2c_do_write(struct i2c_client *client, u8 *tx_buf,
			    int tx_buf_len)
{
	int retval;
	int retries = 0;

retry:
	retval = i2c_master_send(client, tx_buf, tx_buf_len);
	if (0 > retval || retval != tx_buf_len) {
		if (RMI4_I2C_RETRIES > retries) {
			dev_err(&client->dev, "Retrying write operation\n");
			retries++;
			goto retry;
		}
	}
	return retval;
}

static int rmi4_i2c_do_read(struct i2c_client *client, u8 *recv_buf,
			    int recv_buf_len)
{

	int retval;
	int retries = 0;

retry:
	retval = i2c_master_recv(client, recv_buf, recv_buf_len);
	if (0 > retval) {
		if (RMI4_I2C_RETRIES > retries) {
			dev_err(&client->dev, "Retrying read operation\n");
			retries++;
			goto retry;
		}
	}

	return retval;
}

static int rmi4_i2c_set_page(struct i2c_client *client, unsigned int page)
{
	int retval;
	struct rmi4_i2c_driver_data *ddata = dev_get_drvdata(&client->dev);
	char txbuf[2];
	int txlen = sizeof(txbuf);
	txbuf[0] = 0xff;
	txbuf[1] = page;

	dev_dbg(&client->dev, "%s - called\n", __func__);

	retval = rmi4_i2c_do_write(client, txbuf, txlen);
	if (retval < 0) {
		dev_err(&client->dev, "%s: set page failed, write: %d.",
							__func__, retval);
		goto exit;
	}
	if (retval != txlen) {
		dev_err(&client->dev,
				"%s: set page failed, size:%d, retval:%d.",
						__func__, txlen, retval);
		retval = -ENXIO;
		goto exit;
	}

	retval = 0;
	ddata->page = page;
exit:
	return retval;
}

static int rmi4_i2c_adapter_set_page(struct rmi4_core_device *cdev,
							unsigned int page)
{
	int retval = 0;
	struct device *dev = cdev->dev.parent;
	struct i2c_client *client = to_i2c_client(dev);

	dev_dbg(dev, "%s - Called\n", __func__);

	retval = rmi4_i2c_set_page(client, page);
	if (retval) {
		dev_err(dev, "%s: Set Page failed: %d.", __func__, retval);
		goto exit;
	}

exit:
	return (retval < 0) ? retval : 0;
}

static int rmi4_i2c_write(struct i2c_client *client,
						u16 addr, u8 *buf, int len)
{
	int retval;
	u8 txbuf[len + 1];
	int txlen = sizeof(txbuf);


	dev_dbg(&client->dev, "%s - called\n", __func__);

	txbuf[0] = addr & 0xff;
	memcpy(txbuf + 1, buf, len);

	rmi4_i2c_buf_debug(client, "write", addr, txbuf, txlen,
							RMI_I2C_PAGE(addr));

	retval = rmi4_i2c_do_write(client, txbuf, txlen);
	if (retval < 0) {
		dev_err(&client->dev, "%s: i2c write failed: %d.",
							__func__, retval);
		goto exit;
	}
	if (retval != txlen) {
		dev_err(&client->dev,
				"%s: i2c write failed, txbuf size: %d.",
							__func__, retval);
		retval = -ENXIO;
	}

exit:
	return retval;
}

static int rmi4_i2c_adapter_write(struct rmi4_core_device *cdev,
						u16 addr, u8 *buf, int len)
{
	int retval;
	struct device *dev = cdev->dev.parent;
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi4_i2c_driver_data *ddata = dev_get_drvdata(&client->dev);

	dev_dbg(dev, "%s - Called\n", __func__);

	mutex_lock(&ddata->page_mutex);

	if (RMI_I2C_PAGE(addr) != ddata->page) {
		retval = rmi4_i2c_adapter_set_page(cdev, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

	retval = rmi4_i2c_write(client, addr, buf, len);
	if (!retval) {
		dev_err(&client->dev, "%s: i2c write failed: %d.",
							__func__, retval);
		goto exit;
	}

exit:
	mutex_unlock(&ddata->page_mutex);
	return retval;
}

static int rmi4_i2c_read(struct i2c_client *client,
						u16 addr, u8 *buf, int len)
{
	int retval;
	u8 *txbuf = NULL; /* Only need address, no data */
	int txlen = 0;

	dev_dbg(&client->dev, "%s - called\n", __func__);

	retval = rmi4_i2c_write(client, addr, txbuf, txlen);
	if (!retval) {
		dev_err(&client->dev, "%s: i2c read (write) failed: %d.",
							__func__, retval);
		goto exit;
	}

	retval = rmi4_i2c_do_read(client, buf, len);
	if (retval < 0)
		dev_err(&client->dev, "%s: i2c read failed: %d.",
							__func__, retval);

	rmi4_i2c_buf_debug(client, "read", addr, buf, len, RMI_I2C_PAGE(addr));

exit:
	return retval;
}

static int rmi4_i2c_adapter_read(struct rmi4_core_device *cdev,
						u16 addr, u8 *buf, int len)
{
	int retval;
	struct device *dev = cdev->dev.parent;
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi4_i2c_driver_data *ddata = dev_get_drvdata(&client->dev);

	dev_dbg(dev, "%s - Called\n", __func__);

	mutex_lock(&ddata->page_mutex);

	if (RMI_I2C_PAGE(addr) != ddata->page) {
		retval = rmi4_i2c_adapter_set_page(cdev, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

	retval = rmi4_i2c_read(client, addr, buf, len);
	if (!retval) {
		dev_err(&client->dev, "%s: i2c set page failed: %d.",
							__func__, retval);
		goto exit;
	}

exit:
	mutex_unlock(&ddata->page_mutex);
	return retval;
}

static int rmi4_i2c_check_device(struct i2c_client *client)
{
	int rc = -ENXIO;
	struct device *dev = &client->dev;
	struct rmi4_i2c_driver_data *ddata = dev_get_drvdata(&client->dev);
	u16 addr = PDT_START_SCAN_LOCATION;
	u8 buf[RMI4_PACKET_SIZE];
	int len = sizeof(buf);
	int i;

	if (RMI_I2C_PAGE(addr) != ddata->page) {
		rc = rmi4_i2c_set_page(client, RMI_I2C_PAGE(addr));
		if (rc < 0)
			goto exit;
	}

	rc = rmi4_i2c_read(client, addr, buf, len);
	if (rc < 0) {
		dev_err(dev, "%s - check device failed with %d.\n",
								__func__, rc);
		goto exit;
	}

	for (i = 0; i < len; i++) {
		if (buf[i] != 0x00 && buf[i] != 0xFF) {
			dev_dbg(dev, "%s - Got valid response from chip.\n",
				__func__);
			rc = 0;
			goto exit;
		}
	}

exit:
	return rc;
}

static int __devinit rmi4_i2c_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int rc = 0;
	struct device *dev = &client->dev;
	struct rmi4_i2c_driver_data *ddata;
	struct rmi4_i2c_adapter_platform_data *pdata =
						dev_get_platdata(&client->dev);
	struct rmi4_comm_ops ops = {
		.chip_read = rmi4_i2c_adapter_read,
		.chip_write = rmi4_i2c_adapter_write,
	};

	dev_dbg(dev, "%s - Called\n", __func__);

	if (!pdata) {
		dev_err(dev, "%s - no platform data\n", __func__);
		return -EINVAL;
	}

	if (pdata->vreg_config) {
		rc = pdata->vreg_config(dev, 1);
		if (rc < 0) {
			if (rc == -EAGAIN) {
				rc = -EPROBE_DEFER;
				dev_warn(dev, "Vreg not ready yet\n");
			} else {
				dev_err(dev, "failed to setup vreg\n");
			}
			goto err_driver_data;
		}
	}
	msleep(200);

	ddata = kzalloc(sizeof(*ddata), GFP_KERNEL);
	if (!ddata) {
		rc = -ENOMEM;
		goto err_driver_data;
	}

	mutex_init(&ddata->page_mutex);
	ddata->set_page = rmi4_i2c_adapter_set_page;

	dev_set_drvdata(dev, ddata);

	rc = rmi4_i2c_check_device(client);
	if (rc < 0) {
		dev_err(dev, "%s: check device failed!\n", __func__);
		goto err_data_free;
	}

	rc = rmi4_bus_register_adapter(dev, &ops, pdata->cdev_data);
	if (rc) {
		dev_err(dev,
			"%s - failed to register physical driver\n", __func__);
		goto err_vreg;
	}

	dev_info(dev,
		"%s - Successfully registered RMI4 I2C adapter\n", __func__);

	return 0;

err_vreg:


err_data_free:
	dev_set_drvdata(&client->dev, NULL);
	kfree(ddata);
err_driver_data:
	return rc;
}


static int __devexit rmi4_i2c_remove(struct i2c_client *client)
{
	struct rmi4_i2c_driver_data *data = dev_get_drvdata(&client->dev);
	struct rmi4_i2c_adapter_platform_data *d = client->dev.platform_data;

	dev_dbg(&client->dev, "%s - Called\n", __func__);

	if (d && data) {
		dev_dbg(&client->dev, "%s - Unregistering adapter\n", __func__);
		rmi4_bus_unregister_adapter(&client->dev);
	}

	return 0;
}

static const struct i2c_device_id rmi_id[] = {
	{ RMI4_I2C_ADAPTER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= RMI4_I2C_ADAPTER_NAME
	},
	.id_table	= rmi_id,
	.probe		= rmi4_i2c_probe,
	.remove		= __devexit_p(rmi4_i2c_remove),
};

static int __devinit rmi4_i2c_adapter_init(void)
{
	pr_info("%s: RMI4 I2C Driver (Built %s @ %s)\n",
						__func__, __DATE__, __TIME__);
	return i2c_add_driver(&rmi_i2c_driver);
}

static void __devexit rmi4_i2c_adapter_exit(void)
{
	i2c_del_driver(&rmi_i2c_driver);
}

module_init(rmi4_i2c_adapter_init);
module_exit(rmi4_i2c_adapter_exit);

MODULE_AUTHOR("Joakim Wesslen <joakim.wesslen@sonyerisson.com>");
MODULE_DESCRIPTION("RMI4 i2c adapter");
MODULE_LICENSE("GPL");
