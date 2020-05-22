/*
 *  cxd224x-i2c.c - cxd224x NFC i2c driver
 *
 * Copyright (C) 2013- Sony Corporation.
 * Copyright (C) 2012 Broadcom Corporation.
 * Copyright (C) 2017 Sony Mobile Communications Inc.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are licensed under the License.
 *
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/of_gpio.h>

#include "cxd224x.h"

#define CXD224X_WAKE_LOCK_TIMEOUT	1		/* wake timeout for HOSTINT (sec) */

/* do not change below */
#define MAX_BUFFER_SIZE		780

/* Read data */
#define PACKET_HEADER_SIZE_NCI	(3)
#define PACKET_HEADER_SIZE_HCI	(3)
#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)

/* RESET */
#define RESET_ASSERT_MS         (1)

#define CXD224X_PINCTRL_ACTIVE  "felica_active"
#define CXD224X_PINCTRL_SUSPEND "felica_suspend"

struct cxd224x_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice cxd224x_device;
        struct cxd224x_platform_data *gpio;
	bool irq_enabled;
	struct mutex lock;
	spinlock_t irq_enabled_lock;
	unsigned int users;
	unsigned int count_irq;
	/* Driver message queue */
	struct workqueue_struct	*wqueue;
	struct work_struct qmsg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

static struct cxd224x_dev *p_cxd224x_dev;
static void cxd224x_enable_pon(struct cxd224x_dev *cxd224x_dev);
static void cxd224x_conditional_disable_pon(struct cxd224x_dev *cxd224x_dev);

#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
static void cxd224x_workqueue(struct work_struct *work)
{
	struct cxd224x_dev *cxd224x_dev = container_of(work, struct cxd224x_dev, qmsg);
	unsigned long flags;

	dev_info(&cxd224x_dev->client->dev, "%s, xrst assert\n", __func__);
        spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	gpio_set_value(cxd224x_dev->gpio->rst_gpio, CXDNFC_RST_ACTIVE);
        cxd224x_dev->count_irq=0; /* clear irq */
        spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

	msleep(RESET_ASSERT_MS);
	dev_info(&cxd224x_dev->client->dev, "%s, xrst deassert\n", __func__);
	gpio_set_value(cxd224x_dev->gpio->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);
}

static int __init init_wqueue(struct cxd224x_dev *cxd224x_dev)
{
	INIT_WORK(&cxd224x_dev->qmsg, cxd224x_workqueue);
	cxd224x_dev->wqueue = create_workqueue("cxd224x-i2c_wrokq");
	if (cxd224x_dev->wqueue == NULL)
		return -EBUSY;
	return 0;
}
#endif /* CONFIG_NFC_CXD224X_RST */

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

	dev_info(&cxd224x_dev->client->dev, "%s\n", __func__);

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	cxd224x_dev->count_irq++;
	if (cxd224x_dev->irq_enabled) {
		cxd224x_enable_pon(cxd224x_dev);
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

	wake_up(&cxd224x_dev->read_wq);

	return IRQ_HANDLED;
}

static unsigned int cxd224x_dev_poll(struct file *filp, poll_table *wait)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(filp, &cxd224x_dev->read_wq, wait);

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->count_irq > 0)
	{
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

	if(mask) 
		pm_wakeup_event(&cxd224x_dev->client->dev,
			jiffies_to_msecs(CXD224X_WAKE_LOCK_TIMEOUT*HZ));

	return mask;
}

static ssize_t cxd224x_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int total, len, ret;
	unsigned long flags;

	total = 0;
	len = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
	if (cxd224x_dev->count_irq > 0)
		cxd224x_dev->count_irq--;
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

	mutex_lock(&cxd224x_dev->read_mutex);

	ret = i2c_master_recv(cxd224x_dev->client, tmp, 3);
	if (ret == 3 && (tmp[0] != 0xff)) {
		total = ret;

		len = tmp[PACKET_HEADER_SIZE_NCI-1];

		/** make sure full packet fits in the buffer
		**/
		if (len > 0 && (len + total) <= count) {
			/** read the remainder of the packet.
			**/
			ret = i2c_master_recv(cxd224x_dev->client, tmp+total, len);
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
	int call_enable = 0;
	struct cxd224x_dev *cxd224x_dev = container_of(filp->private_data,
                                                       struct cxd224x_dev,
                                                       cxd224x_device);
	filp->private_data = cxd224x_dev;
	mutex_lock(&cxd224x_dev->lock);
	if (!cxd224x_dev->users)
	{
		cxd224x_init_stat(cxd224x_dev);
		call_enable = 1;
	}
	cxd224x_dev->users++;
	if (call_enable)
		cxd224x_enable_irq(cxd224x_dev);
	mutex_unlock(&cxd224x_dev->lock);

	dev_info(&cxd224x_dev->client->dev,
		 "open %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

	return ret;
}

static int cxd224x_dev_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	int call_disable = 0;
	struct cxd224x_dev *cxd224x_dev = filp->private_data;

	mutex_lock(&cxd224x_dev->lock);
	cxd224x_dev->users--;
	if (!cxd224x_dev->users)
	{
		call_disable = 1;
	}
	if (call_disable)
        {
		cxd224x_disable_irq(cxd224x_dev);
		gpio_set_value(cxd224x_dev->gpio->wake_gpio, 0);
        }
	mutex_unlock(&cxd224x_dev->lock);

	dev_info(&cxd224x_dev->client->dev,
		 "release %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

	return ret;
}

static void cxd224x_enable_pon(struct cxd224x_dev *cxd224x_dev)
{
	gpio_set_value(cxd224x_dev->gpio->wake_gpio, 1);
}

static void cxd224x_conditional_disable_pon(struct cxd224x_dev *cxd224x_dev)
{
	unsigned long flag;

	spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flag);
	/* Do not disable PON when data is available to be read */
	if (cxd224x_dev->count_irq == 0)
		gpio_set_value(cxd224x_dev->gpio->wake_gpio, 0);
	spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flag);
}

static long cxd224x_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct cxd224x_dev *cxd224x_dev = filp->private_data;

	switch (cmd) {
	case CXDNFC_RST_CTL:
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
		dev_info(&cxd224x_dev->client->dev, "%s, rst arg=%d\n", __func__, (int)arg);
		return (queue_work(cxd224x_dev->wqueue, &cxd224x_dev->qmsg) ? 0 : 1);
#endif
		break;
	case CXDNFC_POWER_CTL:
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
		if (arg == 0) {
			gpio_set_value(cxd224x_dev->en_gpio, 1);
		} else if (arg == 1) {
			gpio_set_value(cxd224x_dev->en_gpio, 0);  
		} else {
			/* do nothing */
		}
#else
                return 1; /* not support */
#endif
		break;
	case CXDNFC_WAKE_CTL:
		if (arg == 0) {
			/* PON HIGH (normal power mode)*/
			cxd224x_enable_pon(cxd224x_dev);
		} else if (arg == 1) {
			/* PON LOW (low power mode) */
			cxd224x_conditional_disable_pon(cxd224x_dev);
		} else {
			/* do nothing */
		}
		break;
	default:
		dev_err(&cxd224x_dev->client->dev,
			"%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
		return 0;
	}

	return 0;
}

static const struct file_operations cxd224x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = cxd224x_dev_poll,
	.read = cxd224x_dev_read,
	.write = cxd224x_dev_write,
	.open = cxd224x_dev_open,
	.release = cxd224x_dev_release,
	.unlocked_ioctl = cxd224x_dev_unlocked_ioctl
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

#if defined(CONFIG_OF)
static int cxd224x_parse_dt(struct device *dev,
                            struct cxd224x_platform_data *pdata)
{
        int ret=0;

        /*nfc_int*/
	pdata->irq_gpio =  of_get_named_gpio_flags(dev->of_node, "sony,nfc_int", 0,NULL);
	if (pdata->irq_gpio < 0) {
		pr_err( "failed to get \"nfc_int\"\n");
		goto dt_err;
	}

#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
        pdata->en_gpio = of_get_named_gpio_flags(dev->of_node, "sony,nfc_ven", 0,NULL);
	if (pdata->en_gpio< 0) {
		pr_err( "failed to get \"nfc_ven\"\n");
		goto dt_err;
	}
#endif

#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	pdata->rst_gpio = of_get_named_gpio_flags(dev->of_node, "sony,nfc_rst", 0,NULL);
	if (pdata->rst_gpio< 0) {
		pr_err( "failed to get \"nfc_rst\"\n");
		goto dt_err;
	}
#endif

	pdata->wake_gpio = of_get_named_gpio_flags(dev->of_node, "sony,nfc_wake", 0,NULL);
	if (pdata->wake_gpio< 0) {
		pr_err( "failed to get \"nfc_wake\"\n");
		goto dt_err;
	}
        return 0;

dt_err:
        return ret;        
}
#endif

static int cxd224x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct cxd224x_platform_data *platform_data=NULL;
	struct cxd224x_dev *cxd224x_dev;
	struct clk *felica_clk = NULL;
	int irq_gpio_ok  = 0;
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	int en_gpio_ok   = 0;
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	int rst_gpio_ok = 0;
#endif
	int wake_gpio_ok = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
        
#if defined(CONFIG_OF)
        platform_data = kzalloc(sizeof(struct cxd224x_platform_data),
				GFP_KERNEL);
	if (platform_data == NULL) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
        ret = cxd224x_parse_dt(&client->dev, platform_data);
        if (ret) {
                dev_err(&client->dev, "failed to parse device tree\n");
                kfree(platform_data);
                return -ENODEV;
        }
#else
	platform_data = client->dev.platform_data;

	dev_info(&client->dev, "%s, probing cxd224x driver flags = %x\n", __func__, client->flags);
	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc probe fail\n");
		return -ENODEV;
	}
#endif
	dev_info(&client->dev, "%s, rst_gpio(%d)\n", __func__, platform_data->rst_gpio);
        dev_info(&client->dev, "%s, ven_gpio(%d)\n", __func__, platform_data->en_gpio);
        dev_info(&client->dev, "%s, irq_gpio(%d)\n", __func__, platform_data->irq_gpio);
        dev_info(&client->dev, "%s, wake_gpio(%d)\n", __func__, platform_data->wake_gpio);
        
	irq_gpio_ok=1;
        client->irq = gpio_to_irq(platform_data->irq_gpio);
        if (client->irq<0)
        {
                dev_err(&client->dev, "%s, failed to allocate irq=%d\n", __func__, client->irq);
		return -ENODEV;
        }
        dev_info(&client->dev, "%s, irq(%d)\n", __func__, client->irq);

#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	ret = gpio_request_one(platform_data->en_gpio, GPIOF_OUT_INIT_LOW, "nfc_cen");
	if (ret)
		goto err_exit;
	en_gpio_ok=1;
        ret = gpio_direction_output(platform_data->en_gpio, 0);
        if (ret)
            return -ENODEV;
#endif

#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	ret = gpio_request_one(platform_data->rst_gpio, GPIOF_OUT_INIT_HIGH, "nfc_rst");
	if (ret)
		goto err_exit;
	rst_gpio_ok=1;
        ret = gpio_direction_output(platform_data->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);
        if (ret)
            return -ENODEV;
	dev_info(&client->dev, "%s, xrst deassert\n", __func__);
#endif

	ret = gpio_request_one(platform_data->wake_gpio, GPIOF_OUT_INIT_LOW, "nfc_wake");
	if (ret)
		goto err_exit;
	wake_gpio_ok=1;
        ret = gpio_direction_output(platform_data->wake_gpio,0);

	cxd224x_dev = kzalloc(sizeof(*cxd224x_dev), GFP_KERNEL);
	if (cxd224x_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	cxd224x_dev->client = client;
        cxd224x_dev->gpio = platform_data;
	cxd224x_dev->users =0;

	/* init mutex and queues */
	init_waitqueue_head(&cxd224x_dev->read_wq);
	mutex_init(&cxd224x_dev->read_mutex);
	mutex_init(&cxd224x_dev->lock);
	spin_lock_init(&cxd224x_dev->irq_enabled_lock);

#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	if (init_wqueue(cxd224x_dev) != 0) {
		dev_err(&client->dev, "init workqueue failed\n");
		goto err_exit;
	}
#endif

	ret = cxd224x_pinctrl_init(&client->dev, cxd224x_dev);
	if (ret) {
		dev_err(&client->dev, "pinctrl_init failed\n");
		goto err_pinctrl_init;
	}

	cxd224x_dev->cxd224x_device.minor = MISC_DYNAMIC_MINOR;
	cxd224x_dev->cxd224x_device.name = "cxd224x-i2c";
	cxd224x_dev->cxd224x_device.fops = &cxd224x_dev_fops;

	ret = misc_register(&cxd224x_dev->cxd224x_device);
	if (ret) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}

	felica_clk = clk_get(&client->dev, "felica_clk");
	if (IS_ERR(felica_clk)) {
		dev_info(&client->dev, "Couldn't get felica_clk, XTAL should be used\n");
	} else {
		ret = clk_prepare_enable(felica_clk);
		if (ret) {
			dev_err(&client->dev, "failed to enable felica_clk\n");
			goto err_clk_enable;
		}
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	dev_info(&client->dev, "requesting IRQ %d\n", client->irq);
	cxd224x_dev->irq_enabled = true;
	ret = request_irq(client->irq, cxd224x_dev_irq_handler,
			  IRQF_TRIGGER_FALLING, client->name, cxd224x_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	cxd224x_disable_irq(cxd224x_dev);
	i2c_set_clientdata(client, cxd224x_dev);
	cxd224x_pinctrl_select_state(&client->dev, true);
	device_init_wakeup(&client->dev, 1);
	p_cxd224x_dev = cxd224x_dev;

	dev_info(&client->dev,
		 "%s, probing cxd224x driver exited successfully\n",
		 __func__);
	return 0;

err_request_irq_failed:
	misc_deregister(&cxd224x_dev->cxd224x_device);
err_clk_enable:
	clk_put(felica_clk);
err_pinctrl_init:
err_misc_register:
	mutex_destroy(&cxd224x_dev->read_mutex);
	kfree(cxd224x_dev);
err_exit:
	if(irq_gpio_ok)
		gpio_free(platform_data->irq_gpio);
#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
	if(en_gpio_ok)
		gpio_free(platform_data->en_gpio);
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
	if(rst_gpio_ok)
		gpio_free(platform_data->rst_gpio);
#endif
	if(wake_gpio_ok)
		gpio_free(platform_data->wake_gpio);
        
#if defined(CONFIG_OF)
        if(platform_data)
                kfree(platform_data);
#endif
	return ret;
}

static int cxd224x_remove(struct i2c_client *client)
{
	struct cxd224x_dev *cxd224x_dev;

	cxd224x_dev = i2c_get_clientdata(client);

	device_wakeup_disable(&client->dev);

	free_irq(client->irq, cxd224x_dev);
	misc_deregister(&cxd224x_dev->cxd224x_device);
	mutex_destroy(&cxd224x_dev->read_mutex);
        if(cxd224x_dev->gpio)
        {
                gpio_free(cxd224x_dev->gpio->irq_gpio);
                gpio_free(cxd224x_dev->gpio->wake_gpio);

#if defined(CONFIG_NFC_CXD224X_VEN) || defined(CONFIG_NFC_CXD224X_VEN_MODULE)
                gpio_free(cxd224x_dev->gpio->en_gpio);
#endif
#if defined(CONFIG_NFC_CXD224X_RST) || defined(CONFIG_NFC_CXD224X_RST_MODULE)
                gpio_free(cxd224x_dev->gpio->rst_gpio);
#endif

#if defined(CONFIG_OF)

                kfree(cxd224x_dev->gpio);
#endif
        }
	kfree(cxd224x_dev);
        
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cxd224x_suspend(struct device *dev)
{
	struct cxd224x_dev *cxd224x_dev = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);

	if (device_may_wakeup(&cxd224x_dev->client->dev)) {
		int irq = gpio_to_irq(cxd224x_dev->gpio->irq_gpio);
		enable_irq_wake(irq);
	}

	cxd224x_pinctrl_select_state(dev, false);

	return 0;
}

static int cxd224x_resume(struct device *dev)
{
	struct cxd224x_dev *cxd224x_dev = dev_get_drvdata(dev);

	dev_info(dev, "%s\n", __func__);

	if (device_may_wakeup(&cxd224x_dev->client->dev)) {
		int irq = gpio_to_irq(cxd224x_dev->gpio->irq_gpio);
		disable_irq_wake(irq);
	}

	cxd224x_pinctrl_select_state(dev, true);

	return 0;
}

static const struct dev_pm_ops cxd224x_pm_ops = {
	.suspend	= cxd224x_suspend,
	.resume		= cxd224x_resume,
};
#endif

static const struct i2c_device_id cxd224x_id[] = {
	{"cxd224x-i2c", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cxd224x_id);

static struct i2c_driver cxd224x_driver = {
	.id_table = cxd224x_id,
	.probe = cxd224x_probe,
	.remove = cxd224x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cxd224x-i2c",
	},
};

static const struct platform_device_id cxd224x_pm_ops_id[] = {
	{ "cxd224x-pm-ops", 0 },
	{ }
};

static const struct of_device_id cxd224x_pm_ops_match_table[] = {
	{ .compatible = "sony,cxd224x-pm-ops" },
	{ }
};

static int cxd224x_pm_ops_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s, probing cxd224x PM OPS driver\n", __func__);

	if (!p_cxd224x_dev)
		return -ENODEV;

	platform_set_drvdata(pdev, p_cxd224x_dev);

	dev_info(&pdev->dev,
		"%s, probing cxd224x PM OPS driver successfully\n", __func__);

	return 0;
}

static struct platform_driver cxd224x_pm_ops_driver = {
	.id_table = cxd224x_pm_ops_id,
	.probe = cxd224x_pm_ops_probe,
	.driver		= {
		.name		= "cxd224x-pm-ops",
		.of_match_table	= cxd224x_pm_ops_match_table,
#ifdef CONFIG_PM
		.pm		= &cxd224x_pm_ops,
#endif
	},
};

/*
 * module load/unload record keeping
 */

int cxd224x_dev_init(void)
{
	int ret;

	ret = i2c_add_driver(&cxd224x_driver);
	if (ret)
		goto exit;

	ret = platform_driver_register(&cxd224x_pm_ops_driver);
	if (ret)
		goto exit_del_i2c_driver;

	return 0;

exit_del_i2c_driver:
	i2c_del_driver(&cxd224x_driver);

exit:
	return ret;
}

void cxd224x_dev_exit(void)
{
	if (p_cxd224x_dev)
		p_cxd224x_dev = NULL;

	i2c_del_driver(&cxd224x_driver);
	platform_driver_unregister(&cxd224x_pm_ops_driver);
}

MODULE_AUTHOR("Sony");
MODULE_DESCRIPTION("NFC cxd224x driver");
MODULE_LICENSE("GPL v2");
