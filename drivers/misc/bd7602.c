/* drivers/misc/bd7602.c
 *
 * Author: Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#define BD7602_DEVICE_NAME "bd7602"
#define BD7602_MODE_MAX 0x04

struct bd7602_dev {
	struct mutex mutex;
	struct i2c_client *client;
	u8 mode;
};

static ssize_t bd7602_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret;
	u8 mode;
	struct bd7602_dev *bd7602_dev = dev_get_drvdata(dev);

	if (!bd7602_dev) {
		ret = -ENODEV;
		goto err;
	}

	if (sizeof(mode) != size) {
		dev_err(dev, "%s: Invalid size %ld\n", __func__, size);
		ret = -EINVAL;
		goto err;
	}

	memcpy(&mode, buf, size);
	if (BD7602_MODE_MAX < mode) {
		dev_err(dev, "%s: Invalid mode 0x%02x\n", __func__, mode);
		ret = -EINVAL;
		goto err;
	}

	mutex_lock(&bd7602_dev->mutex);
	bd7602_dev->mode = mode;
	mutex_unlock(&bd7602_dev->mutex);
	dev_info(dev, "%s: mode: 0x%02x\n", __func__, bd7602_dev->mode);

	return size;

err:
	return ret;
}

static ssize_t bd7602_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct bd7602_dev *bd7602_dev = dev_get_drvdata(dev);

	if (!bd7602_dev) {
		ret = -ENODEV;
		goto err;
	}

	mutex_lock(&bd7602_dev->mutex);
	memcpy(buf, &bd7602_dev->mode, sizeof(bd7602_dev->mode));
	mutex_unlock(&bd7602_dev->mutex);
	dev_info(dev, "%s: mode: 0x%02x\n", __func__, bd7602_dev->mode);

	return sizeof(bd7602_dev->mode);

err:
	return ret;
}

static ssize_t bd7602_value_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u8 data[2];
	int ret;
	int len = 2;
	struct bd7602_dev *bd7602_dev = dev_get_drvdata(dev);

	if (!bd7602_dev) {
		ret = -ENODEV;
		goto err;
	}

	if (sizeof(u8) != size) {
		dev_err(dev, "%s: Invalid size %ld\n", __func__, size);
		ret = -EINVAL;
		goto err;
	}

	mutex_lock(&bd7602_dev->mutex);
	data[0] = bd7602_dev->mode;
	memcpy(&data[1], buf, size);
	ret = i2c_master_send(bd7602_dev->client, data, len);
	if (ret != len) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(dev, "%s: Failed to write %d\n", __func__, ret);
		ret = -EIO;
		goto err;
	}
	mutex_unlock(&bd7602_dev->mutex);

	dev_info(dev, "%s: mode:0x%02x value:0x%02x\n",
			__func__, data[0], data[1]);

	return size;

err:
	return ret;
}

static ssize_t bd7602_value_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 data;
	int ret;
	struct bd7602_dev *bd7602_dev = dev_get_drvdata(dev);

	if (!bd7602_dev) {
		ret = -ENODEV;
		goto err;
	}

	mutex_lock(&bd7602_dev->mutex);
	ret = i2c_master_send(bd7602_dev->client, &bd7602_dev->mode,
				sizeof(bd7602_dev->mode));
	if (sizeof(bd7602_dev->mode) != ret) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(dev, "%s: Failed to write %d\n", __func__, ret);
		ret = -EIO;
		goto err;
	}

	ret = i2c_master_recv(bd7602_dev->client, &data, sizeof(data));
	if (sizeof(data) != ret) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(dev, "%s: Failed to read %d\n", __func__, ret);
		ret = -EIO;
		goto err;
	}

	memcpy(buf, &data, sizeof(data));
	mutex_unlock(&bd7602_dev->mutex);

	dev_info(dev, "%s: mode:0x%02x value:0x%02x\n",
			__func__, bd7602_dev->mode, data);

	return sizeof(data);

err:
	return ret;
}

static struct device_attribute bd7602_sysfs_attrs[] = {
	__ATTR(mode, S_IRUSR | S_IWUSR, bd7602_mode_show, bd7602_mode_store),
	__ATTR(value, S_IRUSR | S_IWUSR, bd7602_value_show, bd7602_value_store),
};

static int bd7602_create_sysfs_entries(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bd7602_sysfs_attrs); i++)
		if (device_create_file(dev, bd7602_sysfs_attrs + i))
			goto err_create_file;
	return 0;

err_create_file:
	for (i = i - 1; i >= 0; i--)
		device_remove_file(dev, bd7602_sysfs_attrs + i);

	dev_err(dev, "Unable to create sysfs interfaces\n");

	return -EIO;
}

static void bd7602_remove_sysfs_entries(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bd7602_sysfs_attrs); i++)
		device_remove_file(dev, bd7602_sysfs_attrs + i);
}

static int bd7602_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	struct bd7602_dev *bd7602_dev;

	dev_info(&client->dev, "%s, probing bd7602 driver flags = %x\n",
			__func__, client->flags);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_exit;
	}

	bd7602_dev = kzalloc(sizeof(*bd7602_dev), GFP_KERNEL);
	if (bd7602_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	bd7602_dev->client = client;
	i2c_set_clientdata(client, bd7602_dev);

	mutex_init(&bd7602_dev->mutex);

	ret = bd7602_create_sysfs_entries(&client->dev);
	if (ret) {
		dev_err(&client->dev, "bd7602_create_sysfs_entries failed\n");
		goto err_create_sysfs_entries;
	}

	dev_info(&client->dev,
		"%s, probing bd7602 driver exited successfully\n",
		__func__);
	return 0;

err_create_sysfs_entries:
	mutex_destroy(&bd7602_dev->mutex);
	i2c_set_clientdata(client, NULL);
	kzfree(bd7602_dev);
err_kzalloc:
err_exit:
	return ret;
}

static int bd7602_remove(struct i2c_client *client)
{
	struct bd7602_dev *bd7602_dev = i2c_get_clientdata(client);

	bd7602_remove_sysfs_entries(&client->dev);
	mutex_destroy(&bd7602_dev->mutex);
	i2c_set_clientdata(client, NULL);
	kzfree(bd7602_dev);

	return 0;
}

static const struct i2c_device_id bd7602_id[] = {
	{ BD7602_DEVICE_NAME, 0 },
	{ }
};

static struct of_device_id bd7602_match_table[] = {
	{ .compatible = "rohm,bd7602", },
	{ },
};

static struct i2c_driver bd7602_driver = {
	.id_table = bd7602_id,
	.probe = bd7602_probe,
	.remove = bd7602_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = BD7602_DEVICE_NAME,
		.of_match_table = bd7602_match_table,
	},
};

static int __init bd7602_dev_init(void)
{
	return i2c_add_driver(&bd7602_driver);
}
module_init(bd7602_dev_init);

static void __exit bd7602_dev_exit(void)
{
	i2c_del_driver(&bd7602_driver);
}
module_exit(bd7602_dev_exit);

MODULE_AUTHOR("Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>");
MODULE_DESCRIPTION("ROHM BD7602 Power IC Driver");
MODULE_LICENSE("GPL v2");
