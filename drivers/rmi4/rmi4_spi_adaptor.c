/*
 * RMI4 bus driver.
 * drivers/rmi4/rmi4_spi_adaptor.c
 *
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 * Copyright (c) 2011 Sony Ericsson mobile communications AB
 *
 * Authors:	Synaptics INC
 *		Unixphere
 *		Joachim Holst <joachim.holst@sonyericsson.com>
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

#include <linux/rmi4/rmi4.h>
#include <linux/rmi4/rmi4_spi.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/ktime.h>
#include <linux/mutex.h>

#define RMI4_SPI_ADDR_SIZE	2
#define RMI4_SPI_HEADER_SIZE	4
#define RMI4_SPI_PACKET_SIZE	(RMI4_SPI_HEADER_SIZE + RMI4_SPI_ADDR_SIZE)

#define RMI4_SPI_RX_LEN_INDEX	1

#define RMI4_SPI_VERSION_1	0
#define RMI4_SPI_VERSION_2	1

#define RMI_SPI_PROT_VER_INDEX	0

#define RMI_PROTOCOL_VERSION_ADDRESS	0xA0FD
#define SPI_V2_UNIFIED_READ		0xC0
#define SPI_V2_WRITE			0x40
#define SPI_V2_PREPARE_SPLIT_READ	0xC8
#define SPI_V2_EXECUTE_SPLIT_READ	0xCa

#define RMI_SPI_BLOCK_DELAY_US		65
#define RMI_SPI_BYTE_DELAY_US		65
#define RMI_SPI_WRITE_DELAY_US		0

#define DUMMY_READ_SLEEP_US		10
#define RMI4_SPI_WAIT_TIMEOUT_MS	500
#define RMI4_SPI_NUM_BITS_PER_WORD	8

#define RMI_PAGE_SELECT_REGISTER 0x00FF
#define RMI_SPI_PAGE(addr) (((addr) >> 8) & 0x80)
#define RMI_SPI_PAGE_SELECT_WRITE_LENGTH 1

#define SPI_V1_PROTO_NAME	"spi"
#define SPI_V2_PROTO_NAME	"spiv2"

struct rmi4_spi_comm_info {
	char *proto;
	long tx_count;
	long tx_bytes;
	long tx_errs;
	long rx_count;
	long rx_bytes;
	long rx_errs;
	long attn_count;
};

struct rmi4_spi_driver_data {
	bool split_read_pending;

	struct completion irq_comp;

	struct rmi4_spi_comm_info info;
	struct mutex page_lock;

	int (*set_page) (struct rmi4_core_device *dev, u8 page);

	int page;

	int irq;
};

static void rmi4_spi_adapter_dbg_dump(struct device *dev, const u8 *buf,
				      int size, char const *info)
{
#ifdef VERBOSE_DEBUG
	char b[size * 3 + 1];
	unsigned i, k;

	for (i = k = 0; i < size; i++)
		k += snprintf(b + k, sizeof(b) - k, "%02x ", buf[i]);
	dev_dbg(dev, "%s: %s\n", info, b);
#endif
}

static irqreturn_t rmi4_spi_adapter_hard_irq(int irq, void *p)
{
	struct rmi4_spi_driver_data *data = p;

	if (data->split_read_pending)
		complete(&data->irq_comp);

	return IRQ_HANDLED;
}

static int rmi4_spi_adapter_xfer(struct device *dev, const u8 *txbuf,
				 unsigned n_tx, u8 *rxbuf, unsigned n_rx)
{
	int status;
	struct spi_message message;
	struct spi_transfer *xfers;
	int total_bytes = n_tx + n_rx;
	u8 local_buf[total_bytes];
	int xfer_count = 0;
	int xfer_index = 0;
	struct spi_device *client = to_spi_device(dev);
	struct rmi4_spi_driver_data *ddata = dev_get_drvdata(dev);
	struct rmi4_spi_adapter_platform_data *pdata = dev_get_platdata(dev);
	int block_delay = n_rx > 0 ? pdata->spi_v2.block_delay_us : 0;
	int byte_delay = n_rx > 1 ? pdata->spi_v2.read_delay_us : 0;
	int write_delay = n_tx > 1 ? pdata->spi_v2.write_delay_us : 0;

	if (ddata->split_read_pending) {
		block_delay =
		    n_rx > 0 ? pdata->spi_v2.split_read_block_delay_us : 0;
		byte_delay =
		    n_tx > 1 ? pdata->spi_v2.split_read_byte_delay_us : 0;
		write_delay = 0;
	}

	if (n_tx) {
		ddata->info.tx_count++;
		ddata->info.tx_bytes += n_tx;
		if (write_delay)
			xfer_count += n_tx;
		else
			xfer_count += 1;
	}

	if (n_rx) {
		ddata->info.rx_count++;
		ddata->info.rx_bytes += n_rx;
		if (byte_delay)
			xfer_count += n_rx;
		else
			xfer_count += 1;
	}

	xfers = kcalloc(xfer_count, sizeof(struct spi_transfer), GFP_KERNEL);
	if (!xfers)
		return -ENOMEM;

	spi_message_init(&message);

	if (n_tx) {
		if (write_delay) {
			for (xfer_index = 0; xfer_index < n_tx;
			     xfer_index++) {
				memset(&xfers[xfer_index], 0,
				       sizeof(struct spi_transfer));
				xfers[xfer_index].len = 1;
				xfers[xfer_index].delay_usecs = write_delay;
				xfers[xfer_index].tx_buf = txbuf + xfer_index;
				spi_message_add_tail(&xfers[xfer_index],
						     &message);
			}
		} else {
			memset(&xfers[0], 0, sizeof(struct spi_transfer));
			xfers[0].len = n_tx;
			spi_message_add_tail(&xfers[0], &message);
			memcpy(local_buf, txbuf, n_tx);
			xfers[0].tx_buf = local_buf;
			xfer_index++;
		}
		xfers[xfer_index-1].delay_usecs = block_delay;
	}

	if (n_rx) {
		if (byte_delay) {
			int buffer_offset = n_tx;
			for (; xfer_index < xfer_count; xfer_index++) {
				memset(&xfers[xfer_index], 0,
				       sizeof(struct spi_transfer));
				xfers[xfer_index].len = 1;
				xfers[xfer_index].delay_usecs = byte_delay;
				xfers[xfer_index].rx_buf =
					local_buf + buffer_offset;
				buffer_offset++;
				spi_message_add_tail(&xfers[xfer_index],
						     &message);
			}
		} else {
			memset(&xfers[xfer_index], 0,
			       sizeof(struct spi_transfer));
			xfers[xfer_index].len = n_rx;
			xfers[xfer_index].rx_buf = local_buf + n_tx;
			spi_message_add_tail(&xfers[xfer_index], &message);
			xfer_index++;
		}
	}

	if (n_tx)
		rmi4_spi_adapter_dbg_dump(dev, txbuf, n_tx, "SPI sent bytes:");

	if (pdata->spi_v2.cs_assert) {
		status = pdata->spi_v2.cs_assert(dev, true);
		if (status) {
			dev_err(dev, "%s - Failed to assert CS.", __func__);
			goto error_exit;
		}
	}

	udelay(pdata->spi_v2.pre_delay_us);

	status = spi_sync(client, &message);
	if (status == 0) {
		memcpy(rxbuf, local_buf + n_tx, n_rx);
		status = message.status;
	} else {
		ddata->info.tx_errs++;
		ddata->info.rx_errs++;
		dev_err(dev, "spi_sync failed with error code %d.",
			status);
	}

	udelay(pdata->spi_v2.post_delay_us);

	if (pdata->spi_v2.cs_assert) {
		status = pdata->spi_v2.cs_assert(dev, false);
		if (status) {
			dev_err(dev, "%s - Failed to deassert CS.", __func__);
			goto error_exit;
		}
	}

	if (n_rx)
		rmi4_spi_adapter_dbg_dump(dev, txbuf, n_tx,
					  "SPI received bytes:");

error_exit:
	kfree(xfers);
	return status;
}

/* The set_page functions are always called when a mutex is locked. */
static int rmi_spi_v1_set_page(struct rmi4_core_device *dev, u8 page)
{
	int error;
	struct rmi4_spi_driver_data *data = dev_get_drvdata(dev->dev.parent);
	u8 txbuf[] = {RMI_PAGE_SELECT_REGISTER >> 8,
		      RMI_PAGE_SELECT_REGISTER & 0xFF, page};

	error = rmi4_spi_adapter_xfer(dev->dev.parent, txbuf,
				      sizeof(txbuf), NULL, 0);
	if (error < 0) {
		dev_err(dev->dev.parent,
			"Failed to set page select, code: %d.\n",
			error);
		goto error;
	}

	data->page = page;
error:
	return (error < 0) ? error : 0;
}

static int rmi_spi_v2_set_page(struct rmi4_core_device *cdev, u8 page)
{
	int error;
	struct rmi4_spi_driver_data *data = dev_get_drvdata(cdev->dev.parent);
	u8 txbuf[] = {SPI_V2_WRITE, RMI_PAGE_SELECT_REGISTER >> 8,
		      RMI_PAGE_SELECT_REGISTER & 0xFF,
		      RMI_SPI_PAGE_SELECT_WRITE_LENGTH, page};

	error = rmi4_spi_adapter_xfer(cdev->dev.parent, txbuf,
				      sizeof(txbuf), NULL, 0);
	if (error < 0) {
		dev_err(&cdev->dev,
			"Failed to set page select, code: %d.\n",
			error);
		goto error;
	}

	data->page = page;
error:
	return (error < 0) ? error : 0;
}

static int rmi4_spi_adapter_v1_read(struct rmi4_core_device *cdev, u16 addr,
				    u8 *buf, int len)
{
	struct rmi4_spi_driver_data *data;
	u8 txbuf[RMI4_SPI_ADDR_SIZE];
	int error;

	if (!cdev->dev.parent) {
		pr_err("%s - rmi4_core_device not initialized\n", __func__);
		return -EINVAL;
	}
	data = dev_get_drvdata(cdev->dev.parent);

	if (!data) {
		dev_err(&cdev->dev, "%s - No local driver data found\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(cdev->dev.parent, "%s - Reading addr: 0x%X\n", __func__, addr);

	txbuf[0] = (addr >> 8) | 0x80;
	txbuf[1] = addr;

	mutex_lock(&data->page_lock);

	if (RMI_SPI_PAGE(addr) != data->page) {
		if (data->set_page) {
			error = data->set_page(cdev, RMI_SPI_PAGE(addr));
			if (error < 0)
				goto exit;
		} else {
			dev_err(&cdev->dev,
				"%s - page set function not defined\n",
				__func__);
			error = -EFAULT;
			goto exit;
		}
	}

	error = rmi4_spi_adapter_xfer(cdev->dev.parent, txbuf,
				      sizeof(txbuf), buf, len);

exit:
	mutex_unlock(&data->page_lock);
	return error ? error : len;
}

static int rmi4_spi_adapter_v2_read(struct rmi4_core_device *cdev, u16 addr,
				    u8 *buf, int len)
{
	struct rmi4_spi_driver_data *data = dev_get_drvdata(cdev->dev.parent);
	u8 txbuf[RMI4_SPI_HEADER_SIZE];
	int error;

	txbuf[0] = SPI_V2_UNIFIED_READ;
	txbuf[1] = (addr >> 8) & 0x00FF;
	txbuf[2] = addr & 0x00ff;
	txbuf[3] = len;

	mutex_lock(&data->page_lock);

	if (RMI_SPI_PAGE(addr) != data->page) {
		error = data->set_page(cdev, RMI_SPI_PAGE(addr));
		if (error < 0)
			goto exit;
	}

	error = rmi4_spi_adapter_xfer(cdev->dev.parent, txbuf, sizeof(txbuf),
				      buf, len);

exit:
	mutex_unlock(&data->page_lock);

	return error ? error : len;
}

static int rmi4_spi_adapter_v2_split_read(struct rmi4_core_device *cdev,
					  u16 addr, u8 *buf, int len)
{
	int error;
	u8 txbuf[RMI4_SPI_HEADER_SIZE];
	u8 rxbuf[len + 1]; /* one extra byte for read length */
	unsigned long timeout;
	unsigned long remain;
	struct rmi4_spi_driver_data *data = dev_get_drvdata(cdev->dev.parent);

	txbuf[0] = SPI_V2_PREPARE_SPLIT_READ;
	txbuf[1] = (addr >> 8) & 0x00FF;
	txbuf[2] = addr & 0x00ff;
	txbuf[3] = len;

	mutex_lock(&data->page_lock);

	if (RMI_SPI_PAGE(addr) != data->page) {
		error = data->set_page(cdev, RMI_SPI_PAGE(addr));
		if (error < 0)
			goto exit;
	}

	data->split_read_pending = true;

	error = rmi4_spi_adapter_xfer(cdev->dev.parent, txbuf,
				      sizeof(txbuf), NULL, 0);
	if (error < 0) {
		data->split_read_pending = false;
		goto exit;
	}

	timeout = msecs_to_jiffies(RMI4_SPI_WAIT_TIMEOUT_MS);
	remain = wait_for_completion_timeout(&data->irq_comp, timeout);
	if (!remain) {
		dev_err(&cdev->dev, "%s - Split read timed out. Aborting\n",
			__func__);
		goto exit;
	}

	txbuf[0] = SPI_V2_EXECUTE_SPLIT_READ;
	txbuf[1] = 0;

	error = rmi4_spi_adapter_xfer(cdev->dev.parent, txbuf, 2,
				      rxbuf, sizeof(rxbuf));
	data->split_read_pending = false;
	if (error < 0)
		goto exit;

	if (rxbuf[RMI4_SPI_RX_LEN_INDEX] != len) {
		error = -EIO;
		goto exit;
	}

	memcpy(buf, rxbuf + 1, len);

exit:
	mutex_unlock(&data->page_lock);
	return error ? error : len;
}

static int rmi4_spi_adapter_v1_write(struct rmi4_core_device *cdev, u16 addr,
				    u8 *buf, int len)
{
	struct rmi4_spi_driver_data *data = dev_get_drvdata(cdev->dev.parent);
	unsigned char txbuf[len + RMI4_SPI_ADDR_SIZE];
	int error;

	txbuf[0] = addr >> 8;
	txbuf[1] = addr;
	memcpy(txbuf + RMI4_SPI_ADDR_SIZE, buf, len);

	mutex_lock(&data->page_lock);

	if (RMI_SPI_PAGE(addr) != data->page) {
		error = data->set_page(cdev, RMI_SPI_PAGE(addr));
		if (error < 0)
			goto exit;
	}

	error = rmi4_spi_adapter_xfer(cdev->dev.parent, txbuf,
				      sizeof(txbuf), NULL, 0);

exit:
	mutex_unlock(&data->page_lock);

	return error ? error : len;
}

static int rmi4_spi_adapter_v2_write(struct rmi4_core_device *cdev, u16 addr,
				    u8 *buf, int len)
{
	struct rmi4_spi_driver_data *data = dev_get_drvdata(cdev->dev.parent);
	u8 txbuf[len + RMI4_SPI_HEADER_SIZE];
	int error;

	txbuf[0] = SPI_V2_WRITE;
	txbuf[1] = (addr >> 8) & 0x00FF;
	txbuf[2] = addr & 0x00FF;
	txbuf[3] = len;

	memcpy(&txbuf[RMI4_SPI_HEADER_SIZE], buf, len);

	mutex_lock(&data->page_lock);

	if (RMI_SPI_PAGE(addr) != data->page) {
		error = data->set_page(cdev, RMI_SPI_PAGE(addr));
		if (error < 0)
			goto exit;
	}

	error = rmi4_spi_adapter_xfer(cdev->dev.parent, buf,
				      sizeof(txbuf), NULL, 0);

exit:
	mutex_unlock(&data->page_lock);

	return error ? error : len;
}

static int rmi4_spi_adapter_setup_comms(struct device *dev,
					struct rmi4_comm_ops *ops)
{
	int error;
	u8 buf[RMI4_SPI_HEADER_SIZE + RMI4_SPI_ADDR_SIZE];
	struct rmi4_core_device cdev;
	struct rmi4_spi_driver_data *data = dev_get_drvdata(dev);
	struct rmi4_spi_adapter_platform_data *pd = dev_get_platdata(dev);

	if (!data) {
		dev_err(dev, "%s - No device data found. Will crash\n",
			__func__);
		return -EINVAL;
	}

	cdev.dev.parent = dev;

	error = rmi4_spi_adapter_v1_read(&cdev, RMI_PROTOCOL_VERSION_ADDRESS,
					 buf, sizeof(buf));
	if (error < 0) {
		dev_err(dev, "%s - failed to get SPI version number!\n",
			__func__);
		error = -ENODEV;
		goto out;
	}

	if (RMI4_SPI_VERSION_2 == buf[RMI_SPI_PROT_VER_INDEX]) {
		dev_info(dev, "%s - SPI Version 2\n", __func__);
		ops->chip_write = rmi4_spi_adapter_v2_write;
		data->set_page = rmi_spi_v2_set_page;
		data->info.proto = SPI_V2_PROTO_NAME;

		if (pd && pd->attn_gpio >= 0)
			ops->chip_read = rmi4_spi_adapter_v2_split_read;
		else
			ops->chip_read = rmi4_spi_adapter_v2_read;
	} else if (RMI4_SPI_VERSION_1 != buf[RMI_SPI_PROT_VER_INDEX]) {
		dev_err(dev, "%s - Unrecognized SPI version %d.\n",
			__func__, buf[0]);
		error = -EINVAL;
	} else {
		dev_info(dev, "%s - SPI Version 1\n", __func__);
		ops->chip_write = rmi4_spi_adapter_v1_write;
		ops->chip_read = rmi4_spi_adapter_v1_read;
		data->set_page = rmi_spi_v1_set_page;
		data->info.proto = SPI_V1_PROTO_NAME;
	}

out:
	return (error >= 0) ? 0 : error;
}

static int rmi4_spi_adapter_set_comms_delay(struct device *dev,
	struct rmi4_spi_v2_platform_data *spi_v2)
{
	if (spi_v2) {
		spi_v2->block_delay_us = spi_v2->block_delay_us ?
			spi_v2->block_delay_us : RMI_SPI_BLOCK_DELAY_US;
		spi_v2->split_read_block_delay_us =
			spi_v2->split_read_block_delay_us ?
			spi_v2->split_read_block_delay_us :
			RMI_SPI_BLOCK_DELAY_US;
		spi_v2->read_delay_us = spi_v2->read_delay_us ?
			spi_v2->read_delay_us : RMI_SPI_BYTE_DELAY_US;
		spi_v2->write_delay_us = spi_v2->write_delay_us ?
			spi_v2->write_delay_us : RMI_SPI_BYTE_DELAY_US;
		spi_v2->split_read_byte_delay_us =
			spi_v2->split_read_byte_delay_us ?
			spi_v2->split_read_byte_delay_us :
			RMI_SPI_BYTE_DELAY_US;
		spi_v2->pre_delay_us = spi_v2->pre_delay_us ?
			spi_v2->pre_delay_us : 0;
		spi_v2->post_delay_us = spi_v2->post_delay_us ?
			spi_v2->post_delay_us : 0;
	} else {
		dev_err(dev,
			"%s - Invalid input. spi_platform_data uninitialized\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

static int rmi4_spi_adapter_check_device(struct device *dev)
{
	struct rmi4_core_device cdev;
	u8 buf[RMI4_SPI_PACKET_SIZE];
	int error;
	int i;

	/* Some SPI subsystems return 0 for the very first read you do.  So
	 * we use this dummy read to get that out of the way.
	 */
	cdev.dev.parent = dev;
	error = rmi4_spi_adapter_v1_read(&cdev, PDT_START_SCAN_LOCATION,
					 buf, sizeof(buf));
	if (error < 0) {
		dev_err(dev, "%s - dummy read failed with %d.\n",
			__func__, error);
		return error;
	}
	udelay(DUMMY_READ_SLEEP_US);

	/* Now read the first PDT entry.  We know where this is, and if the
	 * RMI4 device is out there, these 6 bytes will be something other
	 * than all 0x00 or 0xFF.  We need to check for 0x00 and 0xFF,
	 * because many (maybe all) SPI implementations will return all 0x00
	 * or all 0xFF on read if the device is not connected.
	 */
	error = rmi4_spi_adapter_v1_read(&cdev, PDT_START_SCAN_LOCATION,
					 buf, sizeof(buf));
	if (error < 0) {
		dev_err(dev, "%s - Read failed with %d.\n",
			__func__, error);
		return error;
	}

	for (i = 0; i < sizeof(buf); i++) {
		if (buf[i] != 0x00 && buf[i] != 0xFF) {
			dev_dbg(dev, "%s - Got valid response from chip.\n",
				__func__);
			return 0;
		}
	}

	dev_err(dev, "%s - Read returned invalid block.\n",
		__func__);

	return -ENODEV;
}

static int __devinit rmi4_spi_adapter_probe(struct spi_device *spi)
{
	int error;
	int irq = -1;
	struct rmi4_spi_driver_data *data;
	struct rmi4_spi_adapter_platform_data *pdata =
		dev_get_platdata(&spi->dev);


	struct rmi4_comm_ops ops = {
		.chip_read = rmi4_spi_adapter_v1_read,
		.chip_write = rmi4_spi_adapter_v1_write,
	};

	dev_dbg(&spi->dev, "%s - Called\n", __func__);

	if (!pdata) {
		dev_err(&spi->dev, "%s - no platform data\n", __func__);
		return -EINVAL;
	}

	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		dev_err(&spi->dev,
			"%s - SPI_MASTER_HALF_DUPLEX flag is not supported\n",
			__func__);
		return -EINVAL;
	}

	spi->bits_per_word = RMI4_SPI_NUM_BITS_PER_WORD;
	spi->mode = SPI_MODE_3;

	error = spi_setup(spi);
	if (error < 0) {
		dev_err(&spi->dev, "%s - spi_setup failed!\n", __func__);
		return error;
	}

	dev_dbg(&spi->dev, "%s - SPI configured\n", __func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		error = -ENOMEM;
		goto err_driver_data;
	}

	init_completion(&data->irq_comp);
	mutex_init(&data->page_lock);
	data->set_page = rmi_spi_v1_set_page;

	dev_set_drvdata(&spi->dev, data);

	rmi4_spi_adapter_set_comms_delay(&spi->dev, &pdata->spi_v2);

	dev_dbg(&spi->dev, "%s - Successfully setup comms delay\n", __func__);

	if (pdata->attn_gpio) {
		error = gpio_request(pdata->attn_gpio, "RMI4_SPI IRQ GPIO");
		if (error) {
			dev_err(&spi->dev,
				"%s - Failed to request GPIO. Error: %d\n",
				__func__, error);
			goto err_data_free;
		}

		gpio_direction_input(pdata->attn_gpio);
	}

	if (pdata->gpio_config) {
		error = pdata->gpio_config(&spi->dev, 1);
		if (error < 0) {
			dev_err(&spi->dev, "Failed to setup GPIOs, code: %d.\n",
				error);
			goto err_data_free;
		}
	}
	dev_dbg(&spi->dev, "%s - Successfully setup GPIO config\n", __func__);

	error = rmi4_spi_adapter_check_device(&spi->dev);
	if (error)
		goto err_data;

	dev_dbg(&spi->dev, "%s - Successfully setup device\n", __func__);

	error = rmi4_spi_adapter_setup_comms(&spi->dev, &ops);
	if (error) {
		dev_err(&spi->dev,
			"%s - Failed to set communication parameters\n",
			__func__);
		goto err_data;
	}

	if (pdata->irq_is_shared)
		pdata->irq_polarity |= IRQF_SHARED;

	if (pdata->attn_gpio > 0) {
		irq = gpio_to_irq(pdata->attn_gpio);
		if (0 > irq) {
			dev_err(&spi->dev,
				"%s - Failed to retrieve correct IRQ\n",
				__func__);
			error = -ENODEV;
			goto err_data;
		}
		data->irq = irq;
		error = request_threaded_irq(irq, rmi4_spi_adapter_hard_irq,
					     NULL, pdata->irq_polarity,
					     dev_name(&spi->dev), data);
		if (error) {
			dev_err(&spi->dev, "request_threaded_irq failed %d\n",
				pdata->attn_gpio);
			goto err_data;
		}
	}

	error = rmi4_bus_register_adapter(&spi->dev, &ops, pdata->cdev_data);
	if (error) {
		dev_err(&spi->dev, "%s - failed to register physical driver\n",
			__func__);
		goto err_irq;
	}

	dev_info(&spi->dev, "%s - Successfully registered RMI4 SPI adapter\n",
		 __func__);

	return 0;

err_irq:
	free_irq(irq, data);
err_data:
	if (pdata->gpio_config) {
		error = pdata->gpio_config(&spi->dev, 0);
		if (error < 0) {
			dev_err(&spi->dev,
				"Failed to release GPIOs, code: %d.\n",
				error);
		}
	}
err_data_free:
	dev_set_drvdata(&spi->dev, NULL);
	kfree(data);
err_driver_data:
	return error;
}

static int __devexit rmi4_spi_adapter_remove(struct spi_device *spi)
{
	int irq;
	struct rmi4_spi_driver_data *data = dev_get_drvdata(&spi->dev);
	struct rmi4_spi_adapter_platform_data *d = spi->dev.platform_data;

	if (d && data) {
		dev_dbg(&spi->dev, "%s - Unregistering adapter\n", __func__);
		rmi4_bus_unregister_adapter(&spi->dev);

		irq = gpio_to_irq(d->attn_gpio);
		if (0 <= irq) {
			dev_dbg(&spi->dev, "%s - Freeing IRQ\n", __func__);
			free_irq(irq, data);
		} else {
			dev_dbg(&spi->dev, "%s - Failed to get IRQ number\n",
				 __func__);
		}

		gpio_free(d->attn_gpio);

		if (d->gpio_config) {
			int error = d->gpio_config(&spi->dev, 0);
			dev_dbg(&spi->dev, "%s - Releasing GPIO's\n",
				 __func__);
			if (error < 0) {
				dev_dbg(&spi->dev,
					"Failed to release GPIOs, code: %d.\n",
					error);
			} else {
				dev_dbg(&spi->dev,
					 "%s - Successfully released GPIO's\n",
					 __func__);
			}
		} else {
			dev_dbg(&spi->dev, "%s - gpio_config not allocated\n",
				 __func__);
		}

		dev_dbg(&spi->dev, "%s - Freeing driver data\n", __func__);
		dev_set_drvdata(&spi->dev, NULL);
		mutex_destroy(&data->page_lock);
		kfree(data);
	} else {
		dev_info(&spi->dev, "%s - System already freed\n", __func__);
	}

	dev_info(&spi->dev, "%s - Unregistered RMI4 SPI Adapter\n", __func__);

	return 0;
}

static const struct spi_device_id rmi_id[] = {
	{ "rmi", 0 },
	{ "rmi-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, rmi_id);

#ifdef CONFIG_PM
static int rmi4_spi_adapter_suspend(struct device *dev)
{
	struct rmi4_spi_driver_data *data = dev_get_drvdata(dev);
	dev_info(dev, "%s - Called\n", __func__);
		free_irq(data->irq, data);
	return 0;
}

static int rmi4_spi_adapter_resume(struct device *dev)
{
	int error;
	struct rmi4_spi_driver_data *data = dev_get_drvdata(dev);
	struct rmi4_spi_adapter_platform_data *pdata =
		dev_get_platdata(dev);

	dev_info(dev, "%s - Called\n", __func__);
	error = request_threaded_irq(data->irq, rmi4_spi_adapter_hard_irq,
				     NULL, pdata->irq_polarity,
				     dev_name(dev), data);
	if (error < 0)
		dev_err(dev, "request_threaded_irq failed %d\n",
			data->irq);

	return error;
}
#else
#define rmi4_spi_adapter_resume	NULL
#define rmi4_spi_adapter_suspend	NULL
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(rmi4_spi_adapter_pm_ops,
			 rmi4_spi_adapter_suspend,
			 rmi4_spi_adapter_resume);


static struct spi_driver rmi_spi_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= RMI4_SPI_ADAPTER_NAME,
		.pm		= &rmi4_spi_adapter_pm_ops,
	},
	.id_table	= rmi_id,
	.probe		= rmi4_spi_adapter_probe,
	.remove	= __devexit_p(rmi4_spi_adapter_remove),
};

static int __devinit rmi4_spi_adapter_init(void)
{
	return spi_register_driver(&rmi_spi_driver);
}

static void __devexit rmi4_spi_adapter_exit(void)
{
	spi_unregister_driver(&rmi_spi_driver);
}

module_init(rmi4_spi_adapter_init);
module_exit(rmi4_spi_adapter_exit);

MODULE_AUTHOR("Joachim Holst <joachim.holst@sonyerisson.com>");
MODULE_DESCRIPTION("RMI4 spi adapter");
MODULE_LICENSE("GPL");
