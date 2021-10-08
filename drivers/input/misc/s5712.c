/*
 *
 * Author: Grace Chang <grace_chang@compal.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#define LID_DEV_NAME    "s5712_sensor"
#define s5712_INPUT      "/dev/input/s5712_dev"

struct s5712_data {
	int gpio;       /* device use gpio number */
	int irq;        /* device request irq number */
	int active_low; /* gpio active high or low for valid value */
	bool wakeup;    /* device can wakeup system or not */
	struct input_dev *s5712_dev;
	struct regulator *vddio;
	u32 min_uv;     /* device allow minimum voltage */
	u32 max_uv;     /* device allow max voltage */
	/* pinctrl data */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
};

static int s5712_gpio_num;
static int s5712_out_status_show(struct seq_file *s, void *unused)
{
	int state;

	state = gpio_get_value(s5712_gpio_num);
	pr_info("[Sensor] %s , s5712_gpio_num=%d , state=%d\n",
		__func__, s5712_gpio_num, state);
	seq_printf(s, "%d\n", state);
	return 0;
}
static int s5712_out_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, s5712_out_status_show, NULL);
}
static const struct file_operations s5712_out_status_fops = {
	.owner		= THIS_MODULE,
	.open		= s5712_out_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static irqreturn_t s5712_interrupt_handler(int irq, void *dev)
{
	int value;
	struct s5712_data *data = dev;

	value = (gpio_get_value_cansleep(data->gpio) ? 1 : 0)
		^ data->active_low;
	pr_info("[s5712 Sensor] %s , value=%d\n", __func__, value);
	if (value) {
		input_report_switch(data->s5712_dev, SW_LID, 0);
		dev_dbg(&data->s5712_dev->dev, "far\n");
		pr_info("[s5712 Sensor] %s , far\n", __func__);
	} else {
		input_report_switch(data->s5712_dev, SW_LID, 1);
		dev_dbg(&data->s5712_dev->dev, "near\n");
		pr_info("[s5712 Sensor] %s , near\n", __func__);
	}
	input_sync(data->s5712_dev);

	return IRQ_HANDLED;
}

static int s5712_input_init(
	struct platform_device *pdev, struct s5712_data *data)
{
	int err = -1;

	data->s5712_dev = devm_input_allocate_device(&pdev->dev);
	if (!data->s5712_dev) {
		dev_err(&data->s5712_dev->dev,
			"input device allocation failed\n");
		return -EINVAL;
	}
	data->s5712_dev->name = LID_DEV_NAME;
	data->s5712_dev->phys = s5712_INPUT;
	__set_bit(EV_SW, data->s5712_dev->evbit);
	__set_bit(SW_LID, data->s5712_dev->swbit);

	err = input_register_device(data->s5712_dev);
	if (err < 0) {
		dev_err(&data->s5712_dev->dev,
			"unable to register input device %s\n",
			LID_DEV_NAME);
		return err;
	}

	return 0;
}
static int s5712_config_regulator(struct platform_device *dev, bool on)
{
	struct s5712_data *data = dev_get_drvdata(&dev->dev);
	int rc = 0;

	if (on) {
		data->vddio = devm_regulator_get(&dev->dev, "vddio");
		if (IS_ERR(data->vddio)) {
			rc = PTR_ERR(data->vddio);
			dev_err(&dev->dev,
				"vddio get failed rc=%d\n", rc);
			data->vddio = NULL;
			return rc;
		}

		if (regulator_count_voltages(data->vddio) > 0) {
			rc = regulator_set_voltage(data->vddio,
				data->min_uv, data->max_uv);
			if (rc) {
				dev_err(&dev->dev,
					"vddio set failed rc=%d\n", rc);
				goto deinit_vregs;
			}
		}
		return rc;
	} else {
		goto deinit_vregs;
	}

deinit_vregs:
	if (regulator_count_voltages(data->vddio) > 0)
		regulator_set_voltage(data->vddio, 0, data->max_uv);

	return rc;
}

static int s5712_set_regulator(struct platform_device *dev, bool on)
{
	struct s5712_data *data = dev_get_drvdata(&dev->dev);
	int rc = 0;

	if (on) {
		if (!IS_ERR_OR_NULL(data->vddio)) {
			rc = regulator_enable(data->vddio);
			if (rc) {
				dev_err(&dev->dev,
					"Enable vddio failed rc=%d\n", rc);
				goto disable_regulator;
			}
		}
		return rc;
	} else {
		if (!IS_ERR_OR_NULL(data->vddio)) {
			rc = regulator_disable(data->vddio);
			if (rc) {
				dev_err(&dev->dev,
					"Disable vddio failed rc=%d\n", rc);
			}
		}
		return 0;
	}

disable_regulator:
	if (!IS_ERR_OR_NULL(data->vddio))
		regulator_disable(data->vddio);
	return rc;
}

#ifdef CONFIG_OF
static int s5712_parse_dt(struct device *dev, struct s5712_data *data)
{
	unsigned int tmp;
	u32 tempval;
	int rc;
	struct device_node *np = dev->of_node;

	pr_info("[Sensor] %s , enter\n", __func__);

	data->gpio = of_get_named_gpio_flags(dev->of_node,
			"linux,gpio-int", 0, &tmp);
	if (!gpio_is_valid(data->gpio)) {
		dev_err(dev, "s5712 gpio is not valid\n");
		return -EINVAL;
	}
	data->active_low = tmp & OF_GPIO_ACTIVE_LOW ? 0 : 1;

	s5712_gpio_num = data->gpio;
	pr_info("[Sensor] %s , gpio=%d , tmp=%d , active_low=%d\n",
		__func__, data->gpio, tmp, data->active_low);

	data->wakeup = of_property_read_bool(np, "linux,wakeup");

	rc = of_property_read_u32(np, "linux,max-uv", &tempval);
	if (rc) {
		dev_err(dev, "unable to read max-uv\n");
		return -EINVAL;
	}
	data->max_uv = tempval;

	rc = of_property_read_u32(np, "linux,min-uv", &tempval);
	if (rc) {
		dev_err(dev, "unable to read min-uv\n");
		return -EINVAL;
	}
	data->min_uv = tempval;

	pr_info("[Sensor] %s , exit\n", __func__);
	return 0;
}
#else
static int s5712_parse_dt(struct device *dev, struct s5712_data *data)
{
	return -EINVAL;
}
#endif

static int s5712_driver_probe(struct platform_device *dev)
{
	struct s5712_data *data;
	int err = 0;
	int irq_flags;

	pr_info("[Sensor] %s , enter\n", __func__);
	dev_dbg(&dev->dev, "s5712_driver probe\n");
	data = devm_kzalloc(&dev->dev, sizeof(struct s5712_data), GFP_KERNEL);
	if (data == NULL) {
		err = -ENOMEM;
		dev_err(&dev->dev, "failed to allocate memory %d\n", err);
		goto exit;
	}
	dev_set_drvdata(&dev->dev, data);
	if (dev->dev.of_node) {
		err = s5712_parse_dt(&dev->dev, data);
		if (err < 0) {
			dev_err(&dev->dev, "Failed to parse device tree\n");
			goto exit;
		}
	} else if (dev->dev.platform_data != NULL) {
		memcpy(data, dev->dev.platform_data, sizeof(*data));
	} else {
		dev_err(&dev->dev, "No valid platform data.\n");
		err = -ENODEV;
		goto exit;
	}

	err = s5712_input_init(dev, data);
	if (err < 0) {
		dev_err(&dev->dev, "input init failed\n");
		goto exit;
	}

	if (!gpio_is_valid(data->gpio)) {
		dev_err(&dev->dev, "gpio is not valid\n");
		err = -EINVAL;
		goto exit;
	}

	irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	err = gpio_request_one(data->gpio, GPIOF_DIR_IN, "s5712_sensor_irq");
	if (err) {
		dev_err(&dev->dev, "unable to request gpio %d\n", data->gpio);
		goto exit;
	}

	data->irq = gpio_to_irq(data->gpio);
	err = devm_request_threaded_irq(&dev->dev, data->irq, NULL,
			s5712_interrupt_handler,
			irq_flags, "s5712_sensor", data);
	if (err < 0) {
		dev_err(&dev->dev, "request irq failed : %d\n", data->irq);
		goto free_gpio;
	}

	device_init_wakeup(&dev->dev, data->wakeup);
	enable_irq_wake(data->irq);

	err = s5712_config_regulator(dev, true);
	if (err < 0) {
		dev_err(&dev->dev, "Configure power failed: %d\n", err);
		goto free_irq;
	}

	err = s5712_set_regulator(dev, true);
	if (err < 0) {
		dev_err(&dev->dev, "power on failed: %d\n", err);
		goto err_regulator_init;
	}

	proc_create_data("s5712_out_status",
			0444, NULL, &s5712_out_status_fops, NULL);

	pr_info("[Sensor] %s , exit - Success\n", __func__);
	return 0;

err_regulator_init:
	/*s5712_config_regulator(dev, false);*/
free_irq:
	disable_irq_wake(data->irq);
	device_init_wakeup(&dev->dev, 0);
free_gpio:
	gpio_free(data->gpio);
exit:
	pr_info("[Sensor] %s , exit - Fail\n", __func__);
	return err;
}

static int s5712_driver_remove(struct platform_device *dev)
{
	struct s5712_data *data = dev_get_drvdata(&dev->dev);

	disable_irq_wake(data->irq);
	device_init_wakeup(&dev->dev, 0);
	if (data->gpio)
		gpio_free(data->gpio);
	s5712_set_regulator(dev, false);
	s5712_config_regulator(dev, false);

	return 0;
}

static struct platform_device_id s5712_id[] = {
	{LID_DEV_NAME, 0 },
	{ },
};

#ifdef CONFIG_OF
static struct of_device_id s5712_match_table[] = {
	{.compatible = "s5712-switch", },
	{ },
};
#endif

static struct platform_driver s5712_driver = {
	.driver = {
		.name = LID_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(s5712_match_table),
	},
	.probe = s5712_driver_probe,
	.remove = s5712_driver_remove,
	.id_table = s5712_id,
};

static int __init s5712_init(void)
{
	return platform_driver_register(&s5712_driver);
}

static void __exit s5712_exit(void)
{
	platform_driver_unregister(&s5712_driver);
}

module_init(s5712_init);
module_exit(s5712_exit);
MODULE_DESCRIPTION("S5712 driver");
MODULE_LICENSE("GPL v2");