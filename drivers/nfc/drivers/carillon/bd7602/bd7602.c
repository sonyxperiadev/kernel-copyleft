/* drivers/misc/bd7602.c
 *
 * Author: Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "bd7602.h"

#define BD7602_DEVICE_NAME "bd7602"
#define BD7602_MODE_MAX 0x04

struct bd7602_dev {
	struct mutex mutex;
	struct i2c_client *client;
	struct miscdevice bd7602_device;
	u8 mode;
};

static ssize_t bd7602_mode_store(struct bd7602_dev *bd7602_dev, void __user *buf)
{
	u8 mode;
	int ret;

	ret = copy_from_user(&mode, buf, sizeof(mode));
	if (ret != 0) {
		dev_err(&bd7602_dev->client->dev, "%s: Failed to copy data from user %d\n",
				__func__, ret);
		ret = -EIO;
		goto out;
	}

	if (BD7602_MODE_MAX < mode) {
		dev_err(&bd7602_dev->client->dev, "%s: Invalid mode 0x%02x\n", __func__, mode);
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&bd7602_dev->mutex);
	bd7602_dev->mode = mode;
	mutex_unlock(&bd7602_dev->mutex);
	dev_info(&bd7602_dev->client->dev, "%s: mode: 0x%02x\n", __func__, bd7602_dev->mode);

out:
	return ret;
}

static ssize_t bd7602_mode_show(struct bd7602_dev *bd7602_dev, void __user *buf)
{
	int ret;

	if (!buf) {
		dev_err(&bd7602_dev->client->dev, "%s: invalid address\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&bd7602_dev->mutex);
	ret = copy_to_user(buf, &bd7602_dev->mode, sizeof(bd7602_dev->mode));
	if (ret != 0) {
		dev_err(&bd7602_dev->client->dev, "%s: Failed to copy data to user %d\n",
				__func__, ret);
		ret = -EIO;
	}
	mutex_unlock(&bd7602_dev->mutex);
	dev_info(&bd7602_dev->client->dev, "%s: mode: 0x%02x\n", __func__, bd7602_dev->mode);

	return ret;
}

static ssize_t bd7602_value_store(struct bd7602_dev *bd7602_dev, void __user *buf)
{
	u8 data[2];
	int ret;
	int len = 2;

	mutex_lock(&bd7602_dev->mutex);
	data[0] = bd7602_dev->mode;
	ret = copy_from_user(&data[1], buf, sizeof(u8));
	if (ret != 0) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(&bd7602_dev->client->dev, "%s: Failed to copy data from user %d\n",
				__func__, ret);
		ret = -EIO;
		goto out;
	}
	ret = i2c_master_send(bd7602_dev->client, data, len);
	if (ret != len) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(&bd7602_dev->client->dev, "%s: Failed to write %d\n", __func__, ret);
		ret = -EIO;
		goto out;
	}
	mutex_unlock(&bd7602_dev->mutex);

	dev_info(&bd7602_dev->client->dev, "%s: mode:0x%02x value:0x%02x\n",
			__func__, data[0], data[1]);

out:
	return ret > 0 ? 0 : ret;
}

static ssize_t bd7602_value_show(struct bd7602_dev *bd7602_dev, void __user *buf)
{
	u8 data;
	int ret;

	if (!buf) {
		dev_err(&bd7602_dev->client->dev, "%s: invalid address\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&bd7602_dev->mutex);
	ret = i2c_master_send(bd7602_dev->client, &bd7602_dev->mode,
				sizeof(bd7602_dev->mode));
	if (sizeof(bd7602_dev->mode) != ret) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(&bd7602_dev->client->dev, "%s: Failed to write %d\n", __func__, ret);
		ret = -EIO;
		goto out;
	}

	ret = i2c_master_recv(bd7602_dev->client, &data, sizeof(data));
	if (sizeof(data) != ret) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(&bd7602_dev->client->dev, "%s: Failed to read %d\n", __func__, ret);
		ret = -EIO;
		goto out;
	}

	ret = copy_to_user(buf, &data, sizeof(data));
	if (ret != 0) {
		mutex_unlock(&bd7602_dev->mutex);
		dev_err(&bd7602_dev->client->dev, "%s: Failed to copy data to user %d\n",
				__func__, ret);
		ret = -EIO;
		goto out;
	}
	mutex_unlock(&bd7602_dev->mutex);

	dev_info(&bd7602_dev->client->dev, "%s: mode:0x%02x value:0x%02x\n",
			__func__, bd7602_dev->mode, data);

out:
	return ret > 0 ? 0 : ret;
}

static long bd7602_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct bd7602_dev *bd7602_dev = filp->private_data;
	int ret = -EINVAL;
	void __user *data = (void __user *)arg;

	if (!bd7602_dev) {
		return -ENODEV;
	}

	switch (cmd) {
	case BD7602_MODE_STORE:
		ret = bd7602_mode_store(bd7602_dev, data);
		break;

	case BD7602_MODE_SHOW:
		ret = bd7602_mode_show(bd7602_dev, data);
		break;

	case BD7602_VALUE_STORE:
		ret = bd7602_value_store(bd7602_dev, data);
		break;

	case BD7602_VALUE_SHOW:
		ret = bd7602_value_show(bd7602_dev, data);
		break;

	default:
		dev_err(&bd7602_dev->client->dev,
			"%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
	}

	return ret;
}

static int bd7602_dev_open(struct inode *inode, struct file *filp)
{
	struct bd7602_dev *bd7602_dev = container_of(filp->private_data,
                                                       struct bd7602_dev,
                                                       bd7602_device);
	filp->private_data = bd7602_dev;

	dev_info(&bd7602_dev->client->dev, "%s, open\n", __func__);

	return 0;
}

const struct file_operations bd7602_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = bd7602_dev_open,
	.unlocked_ioctl	= bd7602_dev_unlocked_ioctl
};

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

	bd7602_dev->bd7602_device.minor = MISC_DYNAMIC_MINOR;
	bd7602_dev->bd7602_device.name = "bd7602-i2c";
	bd7602_dev->bd7602_device.fops = &bd7602_dev_fops;

	ret = misc_register(&bd7602_dev->bd7602_device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}

	dev_info(&client->dev,
		"%s, probing bd7602 driver exited successfully\n",
		__func__);
	return 0;

err_misc_register:
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

int bd7602_dev_init(void)
{
	return i2c_add_driver(&bd7602_driver);
}

void bd7602_dev_exit(void)
{
	i2c_del_driver(&bd7602_driver);
}

MODULE_AUTHOR("Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>");
MODULE_DESCRIPTION("ROHM BD7602 Power IC Driver");
MODULE_LICENSE("GPL v2");
