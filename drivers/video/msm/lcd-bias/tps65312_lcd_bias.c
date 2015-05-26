/*
 * drivers/video/msm/lcd-bias/tps65312_lcd_bias.c
 *
 * Copyright (C) 2014 Sony Mobile Communications AB.
 *
 * Raghavendra Alevoor <raghavendra.alevoor@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#define TPS65312_DEV_NAME "tps65312_lcd_bias"

enum tps65312_register {
	TPS65312_REG_CONTROL = 0xff,
	TPS65312_REG_VPOS = 0x00,
	TPS65312_REG_VNEG = 0x01,
	TPS65312_REG_FREQ = 0x03,
};

enum tps65312_eprom_write {
	TPS65312_WRITE_EEPROM = 0x80,
};

struct tps65312_data {
	struct i2c_client *i2c;
	u8 bias_voltage[2];
};

static struct tps65312_data *get_platdata_dt(struct device *dev)
{
	struct tps65312_data *tps_data = NULL;
	struct device_node *devnode = dev->of_node;
	u32 temp_read;

	if (!devnode)
		return NULL;

	tps_data = kzalloc(sizeof(*tps_data), GFP_KERNEL);

	if (!tps_data) {
		dev_err(dev, "Failed to allocate memory for pdata\n");
		return NULL;
	}
	/* Parse VSP bias voltage */
	if (of_property_read_u32(devnode, "vsp_bias", &temp_read)) {
		dev_err(dev, "Failed to get property: vsp_bias\n");
		goto err_parse_dt;
	}
	tps_data->bias_voltage[0] = (u8)temp_read;

	/* Parse VSN bias voltage */
	if (of_property_read_u32(devnode, "vsn_bias", &temp_read)) {
		dev_err(dev, "Failed to get property: vsn_bias\n");
		goto err_parse_dt;
	}
	tps_data->bias_voltage[1] = (u8)temp_read;

	return tps_data;
err_parse_dt:
	kfree(tps_data);
	return NULL;
}

static int __devinit tps65312_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct tps65312_data *tpsd = NULL;
	int i;
	int rc;

	dev_info(dev, "%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: SMBUS byte data not supported\n", __func__);
		return -EIO;
	}

	tpsd = get_platdata_dt(dev);
	if (!tpsd) {
		rc = -ENOMEM;
		goto err_alloc_data_failed;
	}

	tpsd->i2c = client;
	i2c_set_clientdata(client, tpsd);

	for (i = TPS65312_REG_VPOS; i <= (int)TPS65312_REG_VNEG; ++i) {
		rc = i2c_smbus_write_byte_data(tpsd->i2c, i,
					tpsd->bias_voltage[i]);
		if (rc) {
			dev_err(dev, "Error Writing (0x%02X):%d", i, rc);
			goto err_config;
		}
	}
	dev_dbg(dev, "%s: completed.\n", __func__);
	return 0;

err_config:
	kfree(tpsd);
err_alloc_data_failed:
	dev_err(dev, "%s: failed.\n", __func__);
	return rc;
}

static int __devexit tps65312_remove(struct i2c_client *client)
{
	struct tps65312_data *tpsd = i2c_get_clientdata(client);
	pr_info("%s: Removing the driver\n", __func__);
	i2c_set_clientdata(client, NULL);
	kfree(tpsd);
	return 0;
}

static struct of_device_id tps65312_bl_match_table[] = {
	{ .compatible = "ti,tps65312_bias" },
	{},
};

static const struct i2c_device_id tps65312_id[] = {
	{TPS65312_DEV_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps65312_id);

static struct i2c_driver tps65312_driver = {
	.driver = {
		.name = TPS65312_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tps65312_bl_match_table,
		.pm = NULL,
	},
	.probe = tps65312_probe,
	.remove = __devexit_p(tps65312_remove),
	.id_table = tps65312_id,
};

static int __init tps65312_init(void)
{
	int err = i2c_add_driver(&tps65312_driver);
	pr_info("%s: tps65312 LCD BIAS IC driver, built %s @ %s\n",
		 __func__, __DATE__, __TIME__);
	return err;
}

static void __exit tps65312_exit(void)
{
	pr_info("%s:Module EXIT\n", __func__);
	i2c_del_driver(&tps65312_driver);
}

module_init(tps65312_init);
module_exit(tps65312_exit);

MODULE_AUTHOR("Raghavendra Alevoor <raghavendra.alevoor@sonymobile.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TPS65312 LCD Bias IC driver");
