/* drivers/misc/rpm975h16e4a.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <mach/gpiomux.h>
#include <mach/irqs-8974.h>

#define PM_QOS_IRDA_TIMEOUT (1200 * USEC_PER_SEC)
#define PM_QOS_IRDA_LAT_VALUE 0

#define PWDOWN_GPIO_RSRC "pwdown_gpio"

static char const * const gpio_rsrcs[] = {
	"tx-gpio",
	"rx-gpio",
};

struct rpm975h16e4a_drvdata {
	int pwdown_gpio;
	int gpios[ARRAY_SIZE(gpio_rsrcs)];
	struct platform_device *pdev;
	struct device sysfs_dev;
};

static struct pm_qos_request qos_req;

static struct gpiomux_setting gpio_setting[][ARRAY_SIZE(gpio_rsrcs)] = {
	{ /* Active */
		{
			.func = GPIOMUX_FUNC_2,
			.drv = GPIOMUX_DRV_2MA,
			.pull = GPIOMUX_PULL_NONE,
		},
		{
			.func = GPIOMUX_FUNC_2,
			.drv = GPIOMUX_DRV_2MA,
			.pull = GPIOMUX_PULL_NONE,
		},
	},
	{ /* Suspend */
		{
			.func = GPIOMUX_FUNC_GPIO,
			.drv = GPIOMUX_DRV_2MA,
			.pull = GPIOMUX_PULL_NONE,
			.dir = GPIOMUX_OUT_LOW,
		},
		{
			.func = GPIOMUX_FUNC_GPIO,
			.drv = GPIOMUX_DRV_2MA,
			.pull = GPIOMUX_PULL_NONE,
			.dir = GPIOMUX_IN,
		},
	},
};

static int pwdown_high(struct device *dev, bool high)
{
	int value = high ? 1 : 0;
	struct rpm975h16e4a_drvdata *ddata = dev_get_drvdata(dev);

	gpio_set_value_cansleep(ddata->pwdown_gpio, value);

	return 0;
};

static int rpm975h16e4a_power(struct device *dev, bool enable)
{
	return pwdown_high(dev, !enable);
}

static int rpm975h16e4a_gpio_set_active(struct device *dev, bool active)
{
	int ret;
	int i;
	struct rpm975h16e4a_drvdata *ddata = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++) {
		ret = msm_gpiomux_write(ddata->gpios[i], GPIOMUX_ACTIVE,
					&gpio_setting[!active][i], NULL);
		if (ret) {
			dev_err(dev,
				"%s: msm_gpiomux_write to GPIO %d failed %d\n",
				__func__, ddata->gpios[i], ret);
			goto error_end;
		}
	}

	return 0;

error_end:
	for (; i >= 0; i--)
		msm_gpiomux_write(ddata->gpios[i], GPIOMUX_ACTIVE,
				&gpio_setting[active][i], NULL);
	return ret;
}

static int rpm975h16e4a_reg_power(bool enable, struct device *dev,
			struct regulator **vdd,	struct regulator **vio,
			const char *id_vdd, const char *id_vio)
{
	int ret;

	if (!*vdd)
		*vdd = regulator_get(dev, id_vdd);
	if (IS_ERR_OR_NULL(*vdd)) {
		ret = PTR_ERR(*vdd);
		dev_err(dev, "%s: regulator_get failed on %s. ret=%d\n",
			__func__, id_vdd, ret);
		ret = ret ? ret : -ENODEV;
		goto err_vdd;
	} else {
		ret = regulator_set_voltage(*vdd, 2850000, 2850000);
		if (ret)
			goto err_vdd_set;
	}

	if (!*vio)
		*vio = regulator_get(dev, id_vio);
	if (IS_ERR_OR_NULL(*vio)) {
		ret = PTR_ERR(*vio);
		dev_err(dev, "%s: regulator_get failed on %s. ret=%d\n",
			__func__, id_vio, ret);
		ret = ret ? ret : -ENODEV;
		goto err_vio;
	}

	if (enable) {
		ret = regulator_enable(*vdd);
		if (ret) {
			dev_err(dev,
				"%s: regulator_enable failed on %s. ret=%d\n",
				__func__, id_vdd, ret);
			return ret;
		}
		ret = regulator_enable(*vio);
		if (ret)
			dev_err(dev,
				"%s: regulator_enable failed on %s. ret=%d\n",
				__func__, id_vio, ret);
	} else {
		ret = regulator_disable(*vio);
		if (ret) {
			dev_err(dev,
				"%s: regulator_disable failed on %s. ret=%d\n",
				__func__, id_vio, ret);
			return ret;
		}
		ret = regulator_disable(*vdd);
		if (ret) {
			dev_err(dev,
				"%s: regulator_disable failed on %s. ret=%d\n",
				__func__, id_vdd, ret);
			return ret;
		}
	}
	return ret;
err_vio:
	*vio = NULL;
err_vdd_set:
	regulator_put(*vdd);
err_vdd:
	*vdd = NULL;
	return ret;
}

static int rpm975h16e4a_setup_regulator(struct device *dev, bool enable)
{
	static bool vreg_is_enable;
	static struct regulator *vcc;
	static struct regulator *vio;
	int ret = 0;

	if (enable && !vreg_is_enable) {
		ret = rpm975h16e4a_reg_power(true, dev, &vcc, &vio,
					"rpm975h16e4a_vcc", "rpm975h16e4a_vio");
		if (ret) {
			dev_err(dev, "%s: regulator_enable failed\n", __func__);
			goto out;
		}
		vreg_is_enable = true;
	} else if (!enable && vreg_is_enable) {
		vreg_is_enable = false;
		ret = rpm975h16e4a_reg_power(false, dev, &vcc, &vio,
					"rpm975h16e4a_vcc", "rpm975h16e4a_vio");
		if (ret)
			dev_err(dev,
				"%s: regulator_disable failed\n", __func__);
	} else {
		dev_warn(dev, "%s: regulator was already set to %d\n",
			 __func__, enable);
	}

out:
	return ret;
}

static int rpm975h16e4a_gpios_request(struct device *dev, bool enable)
{
	struct rpm975h16e4a_drvdata *ddata = dev_get_drvdata(dev);
	int i;
	int ret = 0;

	if (enable) {
		for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++) {
			ret = gpio_request(ddata->gpios[i], gpio_rsrcs[i]);
			if (ret) {
				dev_err(dev, "%s: gpio request failed for:%d\n",
					__func__, ddata->gpios[i]);
				goto gpios_free;
			}
		}
		return ret;
	}

gpios_free:
	rpm975h16e4a_gpio_set_active(dev, false);
	for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++)
		gpio_free(ddata->gpios[i]);

	return ret;
}

static int rpm975h16e4a_disable(struct device *dev)
{
	int ret;

	ret = pwdown_high(dev, false);
	if (ret) {
		dev_err(dev, "%s: rpm975h16e4a power low failed\n", __func__);
		goto error_pwdown_high;
	}
	ret = rpm975h16e4a_gpios_request(dev, false);
	if (ret) {
		dev_err(dev, "%s: rpm975h16e4a_gpios_request failed\n",
			__func__);
		goto error_gpio_request;
	}
	ret = rpm975h16e4a_setup_regulator(dev, false);
	if (ret) {
		dev_err(dev, "%s: rpm975h16e4a_setup_regulator failed\n",
			__func__);
		goto error_setup_regulator;
	}
	return 0;

error_setup_regulator:
	rpm975h16e4a_gpios_request(dev, true);
error_gpio_request:
	pwdown_high(dev, true);
error_pwdown_high:
	return ret;
}

static int rpm975h16e4a_enable(struct device *dev)
{
	int ret;

	ret = rpm975h16e4a_setup_regulator(dev, true);
	if (ret) {
		dev_err(dev, "%s: rpm975h16e4a_setup_regulator failed\n",
			__func__);
		goto error_setup_regulator;
	}
	ret = rpm975h16e4a_gpios_request(dev, true);
	if (ret) {
		dev_err(dev, "%s: rpm975h16e4a_gpios_request failed\n",
			__func__);
		goto error_setup_gpio;
	}
	ret = pwdown_high(dev, true);
	if (ret) {
		dev_err(dev, "%s: rpm975h16e4a power high failed\n", __func__);
		goto error_pwdown_high;
	}

	return 0;

error_pwdown_high:
	rpm975h16e4a_gpios_request(dev, false);
error_setup_gpio:
	rpm975h16e4a_setup_regulator(dev, false);
error_setup_regulator:
	return ret;
}

static ssize_t rpm975h16e4a_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long value;
	int ret = 0;

	if (kstrtoul(buf, 0, &value)) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		ret = -EINVAL;
		goto err_out;
	}
	if (value) {
		ret = rpm975h16e4a_power(dev, true);
		if (!ret) {
			ret = rpm975h16e4a_gpio_set_active(dev, true);
			if (!ret)
				pm_qos_update_request_timeout(&qos_req,
							PM_QOS_IRDA_LAT_VALUE,
							PM_QOS_IRDA_TIMEOUT);
			else
				rpm975h16e4a_power(dev, false);
		}
	} else {
		rpm975h16e4a_gpio_set_active(dev, false);
		rpm975h16e4a_power(dev, false);
		pm_qos_update_request(&qos_req, PM_QOS_DEFAULT_VALUE);
	}
	if (ret) {
		dev_err(dev, "%s: power control failed\n", __func__);
		goto err_out;
	}

	return count;

err_out:
	return ret;
}

static struct device_attribute rpm975h16e4a_attr =
	__ATTR(irda, S_IRUSR | S_IWUSR, NULL, rpm975h16e4a_store);

static int __devinit rpm975h16e4a_probe(struct platform_device *pdev)
{
	int gpio;
	int ret;
	int i;
	unsigned int flags;
	struct rpm975h16e4a_drvdata *ddata;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node) {
		ret = -EPERM;
		goto error_init;
	}
	ddata = kzalloc(sizeof(struct rpm975h16e4a_drvdata), GFP_KERNEL);
	if (!ddata) {
		dev_err(&pdev->dev,
			"%s: kmallock failed\n", __func__);
		ret = -ENOMEM;
		goto error_init;
	}
	for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++) {
		gpio = of_get_gpio_flags(of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(&pdev->dev, "%s: invalid gpio #%d: %d\n",
				__func__, i, gpio);
			ret = -EINVAL;
			goto error_gpio;
		}
		ddata->gpios[i] = gpio;
	}
	gpio = of_get_named_gpio_flags(of_node, PWDOWN_GPIO_RSRC, 0, &flags);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "%s: invalid gpio #%s: %d\n",
			__func__, PWDOWN_GPIO_RSRC, gpio);
		ret = -EINVAL;
		goto error_gpio;
	}
	ddata->pwdown_gpio = gpio;
	ret = gpio_request(gpio, PWDOWN_GPIO_RSRC);
	if (ret) {
		dev_err(&pdev->dev, "%s: GPIO %d: gpio_request failed %d\n",
			__func__, gpio, ret);
		goto error_gpio;
	}
	ddata->pdev = pdev;
	platform_set_drvdata(pdev, ddata);
	ddata->sysfs_dev.init_name = "irda";
	dev_set_drvdata(&ddata->sysfs_dev, ddata);
	ret = device_register(&ddata->sysfs_dev);
	if (ret) {
		dev_err(&pdev->dev, "%s: device_register failed %d\n",
			__func__, ret);
		goto error_device_register;
	}
	ret = device_create_file(&ddata->sysfs_dev, &rpm975h16e4a_attr);
	if (ret) {
		dev_err(&pdev->dev, "%s: device_create_file failed %d\n",
			__func__, ret);
		goto error_device_create_file;
	}
	ret = rpm975h16e4a_enable(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "%s: rpm975h16e4a_enable failed %d\n",
			__func__, ret);
		goto error_rpm975h16e4a_enable;
	}

	return 0;

error_rpm975h16e4a_enable:
	device_remove_file(&ddata->sysfs_dev, &rpm975h16e4a_attr);
error_device_create_file:
	device_unregister(&ddata->sysfs_dev);
error_device_register:
	gpio_free(ddata->pwdown_gpio);
error_gpio:
	kfree(ddata);
error_init:
	return ret;
}

static int __devexit rpm975h16e4a_remove(struct platform_device *pdev)
{
	struct rpm975h16e4a_drvdata *ddata = platform_get_drvdata(pdev);

	rpm975h16e4a_disable(&pdev->dev);
	device_remove_file(&ddata->sysfs_dev, &rpm975h16e4a_attr);
	device_unregister(&ddata->sysfs_dev);
	gpio_free(ddata->pwdown_gpio);
	kfree(ddata);

	return 0;
}

static struct of_device_id rpm975h16e4a_match_table[] = {
	{	.compatible = "rohm,rpm975h16e4a",
	},
	{}
};

static struct platform_driver rpm975h16e4a_platform_driver = {
	.probe = rpm975h16e4a_probe,
	.remove = __devexit_p(rpm975h16e4a_remove),
	.driver = {
		.name = "rpm975h16e4a",
		.owner = THIS_MODULE,
		.of_match_table = rpm975h16e4a_match_table,
	},
};

static int __init rpm975h16e4a_init(void)
{
	int error;

	error = platform_driver_register(&rpm975h16e4a_platform_driver);
	if (error) {
		pr_err("%s: platform_driver_register failed\n", __func__);
		goto error_platform_driver_register;
	}
	pm_qos_add_request(&qos_req, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	return 0;

error_platform_driver_register:
	return error;
}

static void __exit rpm975h16e4a_exit(void)
{
	pm_qos_remove_request(&qos_req);
	platform_driver_unregister(&rpm975h16e4a_platform_driver);
}

module_init(rpm975h16e4a_init);
module_exit(rpm975h16e4a_exit);

MODULE_AUTHOR("Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>");
MODULE_DESCRIPTION("Rohm RPM975-H16E4A IrDA driver");
MODULE_LICENSE("GPL");
