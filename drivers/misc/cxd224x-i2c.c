/* drivers/misc/cxd224x-i2c.c
 *
 * Copyright (C) 2013 Sony Corporation.
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

#include <linux/clk.h>
#include <linux/cxd224x.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>

/* do not change below */
#define MAX_BUFFER_SIZE		780

/* Read data */
#define PACKET_HEADER_SIZE_NCI	(3)
#define PACKET_HEADER_SIZE_HCI	(3)
#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)

#define CXD224X_WAKE_LOCK_TIMEOUT	3
#define CXD224X_WAKE_LOCK_NAME	CXD224X_DEVICE_NAME

#define CXD224X_PINCTRL_ACTIVE	"felica_active"
#define CXD224X_PINCTRL_SUSPEND	"felica_suspend"

struct cxd224x_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice cxd224x_device;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	unsigned int irq_gpio;
	unsigned int wake_gpio;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	unsigned int count_irq;
	struct wake_lock wakelock;
};

static void cxd224x_init_stat(struct cxd224x_dev *cxd224x_dev)
{
	cxd224x_dev->count_irq = 0;
}

static void cxd224x_disable_irq(struct cxd224x_dev *cxd224x_dev)
{
	unsigned long flags;
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->irq_enabled) {
		disable_irq_nosync(cxd224x_dev->client->irq);
		cxd224x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
}

static void cxd224x_enable_irq(struct cxd224x_dev *cxd224x_dev)
{
	unsigned long flags;
	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (!cxd224x_dev->irq_enabled) {
		cxd224x_dev->irq_enabled = true;
		enable_irq(cxd224x_dev->client->irq);
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
}

static irqreturn_t cxd224x_dev_irq_handler(int irq, void *dev_id)
{
	struct cxd224x_dev *cxd224x_dev = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	cxd224x_dev->count_irq++;
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
	wake_up(&cxd224x_dev->read_wq);

	dev_info(&cxd224x_dev->client->dev, "%s\n", __func__);

	return IRQ_HANDLED;
}

static unsigned int cxd224x_dev_poll(struct file *filp, poll_table *wait)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(filp, &cxd224x_dev->read_wq, wait);

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->count_irq > 0) {
		cxd224x_dev->count_irq--;
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

	if (mask)
		wake_lock_timeout(&cxd224x_dev->wakelock,
				CXD224X_WAKE_LOCK_TIMEOUT * HZ);
	return mask;
}

static ssize_t cxd224x_dev_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int total, len, ret;

	total = 0;
	len = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&cxd224x_dev->read_mutex);

	ret = i2c_master_recv(cxd224x_dev->client, tmp, 3);
	if (ret == 3 && (tmp[0] != 0xff)) {
		total = ret;

		len = tmp[PACKET_HEADER_SIZE_NCI-1];

		/* make sure full packet fits in the buffer */
		if (len > 0 && (len + total) <= count) {
			/* read the remainder of the packet */
			ret = i2c_master_recv(cxd224x_dev->client, tmp+total,
						len);
			if (ret == len)
				total += len;
		}
	}

	mutex_unlock(&cxd224x_dev->read_mutex);

	if (total > count || copy_to_user(buf, tmp, total)) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to copy to user space, total = %d\n", total);
		total = -EFAULT;
	}

	return total;
}

static ssize_t cxd224x_dev_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE) {
		dev_err(&cxd224x_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to copy from user space\n");
		return -EFAULT;
	}

	mutex_lock(&cxd224x_dev->read_mutex);
	/* Write data */

	ret = i2c_master_send(cxd224x_dev->client, tmp, count);
	if (ret != count) {
		dev_err(&cxd224x_dev->client->dev,
			"failed to write %d\n", ret);
		ret = -EIO;
	}
	mutex_unlock(&cxd224x_dev->read_mutex);

	return ret;
}

static int cxd224x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct cxd224x_dev *cxd224x_dev = container_of(filp->private_data,
							struct cxd224x_dev,
							cxd224x_device);
	filp->private_data = cxd224x_dev;
	cxd224x_init_stat(cxd224x_dev);
	cxd224x_enable_irq(cxd224x_dev);
	dev_info(&cxd224x_dev->client->dev,
		"%d,%d\n", imajor(inode), iminor(inode));

	return ret;
}

static const struct file_operations cxd224x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = cxd224x_dev_poll,
	.read = cxd224x_dev_read,
	.write = cxd224x_dev_write,
	.open = cxd224x_dev_open,
};

static ssize_t cxd224x_dev_wake_ctl_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u8 mode;
	int ret, size;
	struct cxd224x_dev *cxd224x_dev = dev_get_drvdata(dev);


	if (!cxd224x_dev) {
		ret = -ENODEV;
		goto err;
	}
	ret = gpio_get_value_cansleep(cxd224x_dev->wake_gpio);
	if (ret < 0) {
		dev_err(dev, "%s: Unable to read WAKE GPIO\n", __func__);
		goto err;
	}
	mode = !ret;
	size = sizeof(mode);
	memcpy(buf, &mode, size);

	return size;

err:
	return ret;
}

static ssize_t cxd224x_dev_wake_ctl_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	u8 mode;
	int ret, value;
	struct cxd224x_dev *cxd224x_dev = dev_get_drvdata(dev);

	if (!cxd224x_dev) {
		ret = -ENODEV;
		goto err;
	}

	if (sizeof(mode) != size) {
		dev_err(dev, "%s: Invalid size %ld\n", __func__, size);
		ret = -EINVAL;
		goto err;
	}

	memcpy(&mode, buf, size);
	value = mode ? 0 : 1;
	gpio_set_value_cansleep(cxd224x_dev->wake_gpio, value);

	return size;

err:
	return ret;
}

static struct device_attribute cxd224x_sysfs_attrs[] = {
	__ATTR(wake_ctl, S_IRUSR | S_IWUSR, cxd224x_dev_wake_ctl_show,
		cxd224x_dev_wake_ctl_store),
};

static int cxd224x_pinctrl_init(struct device *dev, struct cxd224x_dev *cxd224x_dev)
{
	int ret = 0;

	cxd224x_dev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(cxd224x_dev->pinctrl)) {
		dev_err(dev, "error devm_pinctrl_get() failed err:%ld\n",
			PTR_ERR(cxd224x_dev->pinctrl));
		ret = PTR_ERR(cxd224x_dev->pinctrl);
		goto out;
	}

	cxd224x_dev->gpio_state_active = pinctrl_lookup_state(
		cxd224x_dev->pinctrl, CXD224X_PINCTRL_ACTIVE);

	if (IS_ERR_OR_NULL(cxd224x_dev->gpio_state_active)) {
		ret = PTR_ERR(cxd224x_dev->gpio_state_active);
		dev_info(dev, "note pinctrl_lookup_state(%s) err:%d\n",
				CXD224X_PINCTRL_ACTIVE, ret);
		goto out;
	}

	cxd224x_dev->gpio_state_suspend = pinctrl_lookup_state(
		cxd224x_dev->pinctrl, CXD224X_PINCTRL_SUSPEND);

	if (IS_ERR_OR_NULL(cxd224x_dev->gpio_state_suspend)) {
		ret = PTR_ERR(cxd224x_dev->gpio_state_suspend);
		dev_info(dev, "note pinctrl_lookup_state(%s) err:%d\n",
				CXD224X_PINCTRL_ACTIVE, ret);
		goto out;
	}

out:
	return ret;
}

static void cxd224x_pinctrl_select_state(struct device *dev, bool active)
{
	struct cxd224x_dev *cxd224x_dev = dev_get_drvdata(dev);
	struct pinctrl_state *pins_state;
	const char *pins_state_name;

	if (active) {
		pins_state = cxd224x_dev->gpio_state_active;
		pins_state_name = CXD224X_PINCTRL_ACTIVE;
	} else {
		pins_state = cxd224x_dev->gpio_state_suspend;
		pins_state_name = CXD224X_PINCTRL_SUSPEND;
	}

	if (!IS_ERR_OR_NULL(pins_state)) {
		int ret = pinctrl_select_state(cxd224x_dev->pinctrl,
						pins_state);
		if (ret)
			dev_err(dev, "error pinctrl_select_state(%s) err:%d\n",
				pins_state_name, ret);
	} else {
		dev_err(dev,
			"error pinctrl state-name:'%s' is not configured\n",
			pins_state_name);
	}
}

static int cxd224x_create_sysfs_entries(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cxd224x_sysfs_attrs); i++)
		if (device_create_file(dev, cxd224x_sysfs_attrs + i))
			goto err_create_file;
	return 0;

err_create_file:
	for (i = i - 1; i >= 0; i--)
		device_remove_file(dev, cxd224x_sysfs_attrs + i);

	dev_err(dev, "Unable to create sysfs interfaces\n");

	return -EIO;
}

static void cxd224x_remove_sysfs_entries(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cxd224x_sysfs_attrs); i++)
		device_remove_file(dev, cxd224x_sysfs_attrs + i);
}

static struct cxd224x_platform_data *cxd224x_dt_to_pdata(struct device *dev)
{
	int ret;
	struct device_node *node = dev->of_node;
	struct cxd224x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "unable to allocate memory for platform data\n");
		ret = -ENOMEM;
		goto err;
	}
	pdata->irq_gpio = of_get_named_gpio(node, "cxd224x,irq_gpio", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		ret = -EINVAL;
		dev_err(dev,
			"failed to of_get_named_gpio(cxd224x,irq_gpio)\n");
		goto err;
	}
	pdata->wake_gpio = of_get_named_gpio(node, "cxd224x,wake_gpio", 0);
	if (!gpio_is_valid(pdata->wake_gpio)) {
		ret = -EINVAL;
		dev_err(dev,
			"failed to of_get_named_gpio(cxd224x,wake_gpio)\n");
		goto err;
	}

	return pdata;
err:
	if (pdata)
		devm_kfree(dev, pdata);

	return ERR_PTR(ret);
}

static int cxd224x_gpio_request(struct device *dev,
				struct cxd224x_platform_data *pdata)
{
	int ret;

	ret = gpio_request(pdata->irq_gpio, "cxd224x_irq");
	if (ret)
		goto err_irq;
	ret = gpio_request(pdata->wake_gpio, "cxd224x_wake");
	if (ret)
		goto err_wake;

	return 0;

err_wake:
	gpio_free(pdata->wake_gpio);
err_irq:
	gpio_free(pdata->irq_gpio);

	dev_err(dev, "%s: failed to gpio request %d\n", __func__, ret);
	return ret;
}

static void cxd224x_gpio_free(struct device *dev,
				struct cxd224x_platform_data *pdata)
{
	gpio_free(pdata->wake_gpio);
	gpio_free(pdata->irq_gpio);
}

static int cxd224x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	struct cxd224x_platform_data *platform_data;
	struct cxd224x_dev *cxd224x_dev;
	struct clk *felica_clk = NULL;

	platform_data = client->dev.platform_data;

	dev_info(&client->dev, "%s, probing cxd224x driver flags = %x\n",
			__func__, client->flags);

	if (client->dev.of_node) {
		platform_data = cxd224x_dt_to_pdata(&client->dev);
		if (IS_ERR(platform_data)) {
			ret =  PTR_ERR(platform_data);
			dev_err(&client->dev, "failed to dt_to_pdata\n");
			goto err_exit;
		}
		client->dev.platform_data = platform_data;
	}

	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc probe fail\n");
		ret = -ENODEV;
		goto err_exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_exit;
	}

	ret = cxd224x_gpio_request(&client->dev, platform_data);
	if (ret) {
		dev_err(&client->dev, "failed to gpio_request\n");
		goto err_exit;
	}

	cxd224x_dev = kzalloc(sizeof(*cxd224x_dev), GFP_KERNEL);
	if (cxd224x_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	cxd224x_dev->irq_gpio = platform_data->irq_gpio;
	cxd224x_dev->wake_gpio = platform_data->wake_gpio;
	cxd224x_dev->client = client;
	wake_lock_init(&cxd224x_dev->wakelock, WAKE_LOCK_SUSPEND,
			CXD224X_WAKE_LOCK_NAME);

	/* init mutex and queues */
	init_waitqueue_head(&cxd224x_dev->read_wq);
	mutex_init(&cxd224x_dev->read_mutex);
	spin_lock_init(&cxd224x_dev->irq_enabled_lock);

	ret = cxd224x_pinctrl_init(&client->dev, cxd224x_dev);
	if (ret) {
		dev_err(&client->dev, "pinctrl_init failed\n");
		goto err_pinctrl_init;
	}

	cxd224x_dev->cxd224x_device.minor = MISC_DYNAMIC_MINOR;
	cxd224x_dev->cxd224x_device.name = CXD224X_DEVICE_NAME;
	cxd224x_dev->cxd224x_device.fops = &cxd224x_dev_fops;

	ret = misc_register(&cxd224x_dev->cxd224x_device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}

	felica_clk = clk_get(&client->dev, "felica_clk");
	if (IS_ERR(felica_clk)) {
		dev_err(&client->dev, "Couldn't get felica_clk\n");
		goto err_clk;
	}
	ret = clk_prepare_enable(felica_clk);
	if (ret) {
		dev_err(&client->dev, "failed to enable felica_clk\n");
		goto err_clk_enable;
	}

	ret = cxd224x_create_sysfs_entries(&client->dev);
	if (ret) {
		dev_err(&client->dev, "create_sysfs_entries failed\n");
		goto err_create_sysfs_entries;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "requesting IRQ %d\n", client->irq);
	cxd224x_dev->irq_enabled = true;
	ret = request_threaded_irq(client->irq, NULL, cxd224x_dev_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				client->name, cxd224x_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}

	cxd224x_disable_irq(cxd224x_dev);
	i2c_set_clientdata(client, cxd224x_dev);
	cxd224x_pinctrl_select_state(&client->dev, true);
	device_init_wakeup(&client->dev, 1);

	dev_info(&client->dev,
		"%s, probing cxd224x driver exited successfully\n",
		__func__);

	return 0;

err_request_irq_failed:
	cxd224x_remove_sysfs_entries(&client->dev);
err_create_sysfs_entries:
	misc_deregister(&cxd224x_dev->cxd224x_device);
err_clk_enable:
	clk_put(felica_clk);
err_clk:
err_misc_register:
err_pinctrl_init:
	mutex_destroy(&cxd224x_dev->read_mutex);
	wake_lock_destroy(&cxd224x_dev->wakelock);
	kzfree(cxd224x_dev);
err_kzalloc:
	cxd224x_gpio_free(&client->dev, platform_data);
err_exit:
	return ret;
}

static int cxd224x_remove(struct i2c_client *client)
{
	struct cxd224x_dev *cxd224x_dev = i2c_get_clientdata(client);

	free_irq(client->irq, cxd224x_dev);
	cxd224x_remove_sysfs_entries(&client->dev);
	misc_deregister(&cxd224x_dev->cxd224x_device);
	mutex_destroy(&cxd224x_dev->read_mutex);
	wake_lock_destroy(&cxd224x_dev->wakelock);
	i2c_set_clientdata(client, NULL);
	kzfree(cxd224x_dev);
	cxd224x_gpio_free(&client->dev, client->dev.platform_data);

	return 0;
}

#ifdef CONFIG_PM
static int cxd224x_suspend(struct device *dev)
{
	struct cxd224x_dev *cxd224x_dev = dev_get_drvdata(dev);

	if (device_may_wakeup(&cxd224x_dev->client->dev))
		enable_irq_wake(cxd224x_dev->client->irq);

	cxd224x_pinctrl_select_state(dev, false);

	return 0;
}

static int cxd224x_resume(struct device *dev)
{
	struct cxd224x_dev *cxd224x_dev = dev_get_drvdata(dev);

	if (device_may_wakeup(&cxd224x_dev->client->dev))
		disable_irq_wake(cxd224x_dev->client->irq);

	cxd224x_pinctrl_select_state(dev, true);

	return 0;
}

static const struct dev_pm_ops cxd224x_pm_ops = {
	.suspend	= cxd224x_suspend,
	.resume		= cxd224x_resume,
};
#endif

static const struct i2c_device_id cxd224x_id[] = {
	{ CXD224X_DEVICE_NAME, 0 },
	{ }
};

static struct of_device_id cxd224x_match_table[] = {
	{ .compatible = "sony,cxd224x", },
	{ },
};

static struct i2c_driver cxd224x_driver = {
	.id_table = cxd224x_id,
	.probe = cxd224x_probe,
	.remove = cxd224x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = CXD224X_DEVICE_NAME,
#ifdef CONFIG_PM
		.pm = &cxd224x_pm_ops,
#endif
		.of_match_table = cxd224x_match_table,
	},
};

static int __init cxd224x_dev_init(void)
{
	return i2c_add_driver(&cxd224x_driver);
}
module_init(cxd224x_dev_init);

static void __exit cxd224x_dev_exit(void)
{
	i2c_del_driver(&cxd224x_driver);
}
module_exit(cxd224x_dev_exit);

MODULE_AUTHOR("Sony");
MODULE_DESCRIPTION("NFC cxd224x driver");
MODULE_LICENSE("GPL v2");
