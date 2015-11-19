/*
 * cyttsp4_i2c.c
 * Cypress TrueTouch(TM) Standard Product V4 I2C Driver module.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2012 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 * Modified by: Cypress Semiconductor for test with device
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/input/cypress-gen4/cyttsp4_bus.h>
#include <linux/input/cypress-gen4/cyttsp4_core.h>
#include "cyttsp4_i2c.h"

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <linux/qpnp/pin.h>
#include <linux/qpnp/qpnp-adc.h>

#include <linux/fih_sw_info.h>

#include "cyttsp4_devtree.h"
#include "file_node_control/file_node_control.h"

#define CY_I2C_DATA_SIZE  (3 * 256)

struct cyttsp4_i2c {
	struct i2c_client *client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	char const *id;
	struct mutex lock;
	struct qpnp_vadc_chip *touch_sensor;
};

extern bool cyttsp_i2c_driver;	/* PERI-FG-TOUCH_CHECK_HW-01+ */

int cyttsp4_i2c_read_block_data(struct cyttsp4_i2c *ts_i2c, u16 addr,
		int length, void *values, int max_xfer)
{
	int rc = -EINVAL;
	int trans_len;
	u8 client_addr;
	u8 addr_lo;
	struct i2c_msg msgs[2];

	while (length > 0) {
		client_addr = ts_i2c->client->addr | ((addr >> 8) & 0x1);
		addr_lo = addr & 0xFF;
		trans_len = min(length, max_xfer);

		memset(msgs, 0, sizeof(msgs));
		msgs[0].addr = client_addr;
		msgs[0].flags = 0;
		msgs[0].len = 1;
		msgs[0].buf = &addr_lo;

		msgs[1].addr = client_addr;
		msgs[1].flags = I2C_M_RD;
		msgs[1].len = trans_len;
		msgs[1].buf = values;

		rc = i2c_transfer(ts_i2c->client->adapter, msgs, 2);
		if (rc != 2)
			goto exit;

		length -= trans_len;
		values += trans_len;
		addr += trans_len;
	}

exit:
	return (rc < 0) ? rc : rc != ARRAY_SIZE(msgs) ? -EIO : 0;
}
EXPORT_SYMBOL_GPL( cyttsp4_i2c_read_block_data );

int cyttsp4_i2c_write_block_data(struct cyttsp4_i2c *ts_i2c, u16 addr,
		int length, const void *values, int max_xfer)
{
	int rc = -EINVAL;
	u8 client_addr;
	u8 addr_lo;
	int trans_len;
	struct i2c_msg msg;

	if (sizeof(ts_i2c->wr_buf) < (length + 1))
		return -ENOMEM;

	while (length > 0) {
		client_addr = ts_i2c->client->addr | ((addr >> 8) & 0x1);
		addr_lo = addr & 0xFF;
		trans_len = min(length, max_xfer);

		memset(&msg, 0, sizeof(msg));
		msg.addr = client_addr;
		msg.flags = 0;
		msg.len = trans_len + 1;
		msg.buf = ts_i2c->wr_buf;

		ts_i2c->wr_buf[0] = addr_lo;
		memcpy(&ts_i2c->wr_buf[1], values, trans_len);

		/* write data */
		rc = i2c_transfer(ts_i2c->client->adapter, &msg, 1);
		if (rc != 1)
			goto exit;

		length -= trans_len;
		values += trans_len;
		addr += trans_len;
	}

exit:
	return (rc < 0) ? rc : rc != 1 ? -EIO : 0;
}
EXPORT_SYMBOL_GPL( cyttsp4_i2c_write_block_data );

static int cyttsp4_i2c_write(struct cyttsp4_adapter *adap, u16 addr,
	const void *buf, int size, int max_xfer)
{
	struct cyttsp4_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_write_block_data(ts, addr, size, buf, max_xfer);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

static int cyttsp4_i2c_read(struct cyttsp4_adapter *adap, u16 addr,
	void *buf, int size, int max_xfer)
{
	struct cyttsp4_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp4_i2c_read_block_data(ts, addr, size, buf, max_xfer);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);

	return rc;
}

static struct cyttsp4_ops ops = {
	.write = cyttsp4_i2c_write,
	.read = cyttsp4_i2c_read,
};

static struct of_device_id cyttsp4_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp4_i2c_adapter", }, { }
};
MODULE_DEVICE_TABLE(of, cyttsp4_i2c_of_match);

/* PERI-FG-TOUCH_CHECK_HW-00+[ */
static int cyttsp4_ping_hw(struct cyttsp4_i2c *ts_i2c)
{
    int rc, retry = 3;
    char buf;

    mutex_lock(&ts_i2c->lock);
    while (retry--)
    {
        rc = cyttsp4_i2c_read_block_data(ts_i2c, 0x00, 1, &buf, 1);
        if (rc)
            printk("%s: Read unsuccessful, try=%d\n", __func__, 3 - retry);
        else
            break;
        msleep(100);
    }
    mutex_unlock(&ts_i2c->lock);

    return rc;
}
/* PERI-FG-TOUCH_CHECK_HW-00+] */

static int cyttsp4_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct cyttsp4_i2c *ts_i2c;
	struct device *dev = &client->dev;
	const struct of_device_id *match;
	char const *adap_id;
	int rc;

	dev_info(dev, "%s: Starting %s probe...\n", __func__, CYTTSP4_I2C_NAME);

	dev_dbg(dev, "%s: debug on\n", __func__);
	dev_vdbg(dev, "%s: verbose debug on\n", __func__);

	{ // Enable power source

		struct	power_source_data
		{

			char	*name;

			unsigned	min_voltage, max_voltage, load_current;

		};

		struct power_source_data	power_source_EVM[] =
		{

			{ "vdda", 2700000, 3300000, 15000 },
			{ "vddio_EVM", 1800000, 1800000, 15000 }

		};

		struct power_source_data	power_source_PD1[] =
		{

			{ "vdda", 2700000, 3300000, 15000 },
			{ "vddio_PD1", 1800000, 1800000, 15000 }

		};

		struct power_source_data	power_source_DEFAULT[] =
		{

			{ "vdda", 2700000, 3300000, 15000 },
			{ "vddio_DEFAULT", 1800000, 1800000, 15000 }

		};

		struct regulator	*power;

		struct power_source_data	*power_source;

		unsigned	loop, power_source_count;

		switch( fih_get_product_phase () )
		{

			case PHASE_EVM :

				power_source = power_source_EVM;

				power_source_count = sizeof( power_source_EVM ) / sizeof( *power_source_EVM );

				printk( "ITUCH : Load EVM power source\n" );

				break;

			case PHASE_PD1 :

				power_source = power_source_PD1;

				power_source_count = sizeof( power_source_PD1 ) / sizeof( *power_source_PD1 );

				printk( "ITUCH : Load PD1 power source\n" );

				break;

			default :

				power_source = power_source_DEFAULT;

				power_source_count = sizeof( power_source_DEFAULT ) / sizeof( *power_source_DEFAULT );

				printk( "ITUCH : Load default power source\n" );

				break;

		}

		for( loop = 0 ; loop < power_source_count ; ++loop )
		{ // enable loop

			struct power_source_data	*source = power_source + loop;

			power	= regulator_get( dev, source->name );

			if( IS_ERR( power ) )
			{ // Failed

				printk( "PTUCH : Failed to get %s regulator\n", source->name );

				return	-1;

			} // Failed

			if( regulator_count_voltages( power ) > 0 )
			{ // Set voltage & current

				if( regulator_set_voltage( power, source->min_voltage, source->max_voltage ) )
				{ // Failed

					printk( "PTUCH : reg set %s vtg failed\n", source->name );

					return	-1;

				} // Failed

				if( regulator_set_optimum_mode( power, source->load_current ) < 0 )
				{ // Failed

					printk( "PTUCH : Regulator %s set_opt failed\n", source->name );

					return	-1;

				} // Failed

			} // Set voltage & current

			if( regulator_enable( power ) )
			{ // Failed

				printk( "PTUCH : Regulator %s enable failed\n", source->name );

				return	-1;

			} // Failed

		} // enable loop

	} // Enable power source

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "%s: fail check I2C functionality\n", __func__);
		rc = -EIO;
		goto error_alloc_data_failed;
	}

	ts_i2c = kzalloc(sizeof(struct cyttsp4_i2c), GFP_KERNEL);
	if (ts_i2c == NULL) {
		dev_err(dev, "%s: Error, kzalloc.\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	mutex_init(&ts_i2c->lock);
	ts_i2c->client = client;
	ts_i2c->id = (adap_id) ? adap_id : CYTTSP4_I2C_NAME;
	client->dev.bus = &i2c_bus_type;
	i2c_set_clientdata(client, ts_i2c);
	dev_set_drvdata(&client->dev, ts_i2c);

	dev_dbg(dev, "%s: add adap='%s' (CYTTSP4_I2C_NAME=%s)\n", __func__,
		ts_i2c->id, CYTTSP4_I2C_NAME);

	pm_runtime_enable(&client->dev);

    /* PERI-FG-TOUCH_CHECK_HW-00+[ */
    rc = cyttsp4_ping_hw(ts_i2c);
    if (rc) {
        dev_err(dev, "%s: No HW detected\n", __func__);
        goto error_free_data;
    }
    /* PERI-FG-TOUCH_CHECK_HW-00+] */

	match = of_match_device(of_match_ptr(cyttsp4_i2c_of_match), dev);
	if (match) {
		rc = of_property_read_string(dev->of_node, "cy,adapter_id",
				&adap_id);
		if (rc) {
			dev_err(dev, "%s: OF error rc=%d\n", __func__, rc);
			goto error_free_data;
		}
		cyttsp4_devtree_register_devices(dev);
	} else {
		adap_id = dev_get_platdata(dev);
	}
	ts_i2c->id = (adap_id) ? adap_id : CYTTSP4_I2C_NAME;

	rc = cyttsp4_add_adapter(ts_i2c->id, &ops, dev);
	if (rc) {
		dev_err(dev, "%s: Error on probe %s\n", __func__,
			CYTTSP4_I2C_NAME);
		goto add_adapter_err;
	}

	dev_info(dev, "%s: Successful probe %s\n", __func__, CYTTSP4_I2C_NAME);

	printk( "ITUCH : I2C driver is ready\n" );

//	ts_i2c->touch_sensor	= qpnp_get_vadc( dev, "module_adc" );
//
//	if( IS_ERR( ts_i2c->touch_sensor ) )
//		printk( "ITUCH : Can't get touch_sensor for touch senosr module!!!\n" );

	// REGISTER_FILE_NODE_CONTROL_DRIVER( ts_i2c );

	cyttsp_i2c_driver = true;	/* PERI-FG-TOUCH_CHECK_HW-01+ */

	return 0;

add_adapter_err:
error_free_data:
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
error_alloc_data_failed:
	return rc;
}

/* registered in driver struct */
static int cyttsp4_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cyttsp4_i2c *ts_i2c = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);
	cyttsp4_del_adapter(ts_i2c->id);
	pm_runtime_disable(&client->dev);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
	return 0;
}

static const struct i2c_device_id cyttsp4_i2c_id[] = {
	{ CYTTSP4_I2C_NAME, 0 },  { }
};
MODULE_DEVICE_TABLE(i2c, cyttsp4_i2c_id);

static struct i2c_driver cyttsp4_i2c_driver = {
	.driver = {
		.name = CYTTSP4_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cyttsp4_i2c_of_match,
	},
	.probe = cyttsp4_i2c_probe,
	.remove = cyttsp4_i2c_remove,
	.id_table = cyttsp4_i2c_id,
};

static int __init cyttsp4_i2c_init(void)
{
	/* PERI-FG-TOUCH_CHECK_HW-01*[ */
	int rc = 0;

	if (cyttsp_i2c_driver)
		return rc;

	rc = i2c_add_driver(&cyttsp4_i2c_driver);
	/* PERI-FG-TOUCH_CHECK_HW-01*] */

	pr_info("%s: Cypress TTSP I2C Touchscreen Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
module_init(cyttsp4_i2c_init);

static void __exit cyttsp4_i2c_exit(void)
{
	i2c_del_driver(&cyttsp4_i2c_driver);
	pr_info("%s: module exit\n", __func__);
}
module_exit(cyttsp4_i2c_exit);

MODULE_ALIAS(CYTTSP4_I2C_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");
