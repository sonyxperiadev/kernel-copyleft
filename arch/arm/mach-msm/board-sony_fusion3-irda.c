/* arch/arm/mach-msm/board-sony_fusion3-irda.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/err.h>
#include <linux/pm_qos.h>
#include <linux/serial_core.h>
#include <mach/gpiomux.h>
#include <mach/msm_serial_hs_lite.h>

#include "devices.h"
#include "board-8064.h"
#include "board-sony_fusion3.h"

#define PM_GPIO_IRDA_PWDOWN PM8921_GPIO_PM_TO_SYS(17)
#define PM_QOS_IRDA_TIMEOUT (1200 * USEC_PER_SEC)
#define PM_QOS_IRDA_LAT_VALUE 0

#define UART_TX_IRDA_GPIO 22
#define UART_RX_IRDA_GPIO 23

struct irda_vreg_data {
	bool enable;
	struct regulator *irda_vcc;
	struct regulator *irda_vio;
};

struct irda_platform_data {
	int pwdown_gpio;
	int uart_tx_gpio;
	int uart_rx_gpio;
};

static struct pm_qos_request qos_req;

static int pwdown_high(struct device *dev, bool high)
{
	struct irda_platform_data *irda_data = dev->platform_data;

	gpio_set_value_cansleep(irda_data->pwdown_gpio, high);

	return 0;
};

static int irda_power(struct device *dev, bool enable)
{
	struct irda_platform_data *irda_data = dev->platform_data;

	if (enable)
		if (!gpio_get_value_cansleep(irda_data->pwdown_gpio))
			return -EBUSY;

	gpio_set_value_cansleep(irda_data->pwdown_gpio, !enable);

	return 0;
}

static struct gpiomux_setting trx_active = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting rx_inactive = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting tx_inactive = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static int irda_trx_active(struct device *dev, bool active)
{
	struct irda_platform_data *irda_data = dev->platform_data;
	int ret;

	if (active) {
		ret = msm_gpiomux_write(irda_data->uart_rx_gpio,
				GPIOMUX_ACTIVE, &trx_active, NULL);
		if (ret) {
			dev_err(dev, "%s: msm_gpiomux_write to GPIO %d failed.",
				__func__, irda_data->uart_rx_gpio);
			goto irda_trx_active_end;
		}
		ret = msm_gpiomux_write(irda_data->uart_tx_gpio,
				GPIOMUX_ACTIVE, &trx_active, NULL);
		if (ret) {
			dev_err(dev, "%s: msm_gpiomux_write to GPIO %d failed.",
				__func__, irda_data->uart_tx_gpio);
			msm_gpiomux_write(irda_data->uart_rx_gpio,
				GPIOMUX_ACTIVE, &rx_inactive, NULL);
			goto irda_trx_active_end;
		}
	} else {
		ret = msm_gpiomux_write(irda_data->uart_rx_gpio,
				GPIOMUX_ACTIVE, &rx_inactive, NULL);
		if (ret)
			dev_err(dev, "%s: msm_gpiomux_write to GPIO %d failed.",
				__func__, irda_data->uart_rx_gpio);
		ret = msm_gpiomux_write(irda_data->uart_tx_gpio,
				GPIOMUX_ACTIVE, &tx_inactive, NULL);
		if (ret)
			dev_err(dev, "%s: msm_gpiomux_write to GPIO %d failed.",
				__func__, irda_data->uart_tx_gpio);
		ret = 0;
	}

irda_trx_active_end:
	return ret;
}

static int irda_setup_regulator(struct device *dev, bool enable)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct irda_vreg_data *irda_vregs = platform_get_drvdata(pdev);
	int ret = 0;

	if (enable) {
		if (!irda_vregs->enable) {
			ret = sensor_power(true, dev,
						&irda_vregs->irda_vcc,
						&irda_vregs->irda_vio,
						"irda_vcc", "irda_vio");
			if (ret) {
				dev_err(dev,
					"%s: regulator_enable failed\n",
					__func__);
				return ret;
			}
			irda_vregs->enable = true;
		}
	} else {
		if (irda_vregs->enable) {
			ret = sensor_power(false, dev,
						&irda_vregs->irda_vcc,
						&irda_vregs->irda_vio,
						"irda_vcc", "irda_vio");
			if (ret)
				dev_err(dev,
					"%s: regulator_disable failed\n",
					__func__);
			irda_vregs->enable = false;
		}
	}

	return ret;
}

static int irda_setup_gpio(struct device *dev, bool enable)
{
	struct irda_platform_data *irda_data = dev->platform_data;
	int ret = 0;

	if (enable) {
		ret = gpio_request(irda_data->uart_rx_gpio, "UART_RX_GPIO");
		if (ret) {
			dev_err(dev, "%s: gpio request failed for:%d\n",
				__func__, irda_data->uart_rx_gpio);
			return ret;
		}
		ret = gpio_request(irda_data->uart_tx_gpio, "UART_TX_GPIO");
		if (ret) {
			dev_err(dev, "%s: gpio request failed for:%d\n",
				__func__, irda_data->uart_tx_gpio);
			gpio_free(irda_data->uart_rx_gpio);
			return ret;
		}
	} else {
		irda_trx_active(dev, false);
		gpio_free(irda_data->uart_tx_gpio);
		gpio_free(irda_data->uart_rx_gpio);
	}

	return ret;
}

static int irda_disable(struct device *dev)
{
	int ret;

	ret = pwdown_high(dev, false);
	if (ret) {
		dev_err(dev, "%s: irda power low failed\n", __func__);
		goto error_pwdown_high;
	}
	ret = irda_setup_gpio(dev, false);
	if (ret) {
		dev_err(dev, "%s: irda_setup_gpio failed\n", __func__);
		goto error_setup_gpio;
	}
	ret = irda_setup_regulator(dev, false);
	if (ret) {
		dev_err(dev, "%s: irda_setup_regulator failed\n", __func__);
		goto error_setup_regulator;
	}
	return 0;

error_setup_regulator:
	irda_setup_gpio(dev, true);
error_setup_gpio:
	pwdown_high(dev, true);
error_pwdown_high:
	return ret;
}

static int irda_enable(struct device *dev)
{
	int ret;

	ret = irda_setup_regulator(dev, true);
	if (ret) {
		dev_err(dev, "%s: irda_setup_regulator failed\n", __func__);
		goto error_setup_regulator;
	}
	ret = irda_setup_gpio(dev, true);
	if (ret) {
		dev_err(dev, "%s: irda_setup_gpio failed\n", __func__);
		goto error_setup_gpio;
	}
	ret = pwdown_high(dev, true);
	if (ret) {
		dev_err(dev, "%s: irda power high failed\n", __func__);
		goto error_pwdown_high;
	}

	return 0;

error_pwdown_high:
	irda_setup_gpio(dev, false);
error_setup_gpio:
	irda_setup_regulator(dev, false);
error_setup_regulator:
	return ret;
}

static ssize_t irda_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long value;
	int ret = 0;

	if (strict_strtoul(buf, 0, &value)) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		ret = -EINVAL;
		goto err_out;
	}
	if (value) {
		ret = irda_power(dev, true);
		if (!ret) {
			ret = irda_trx_active(dev, true);
			if (!ret)
				pm_qos_update_request_timeout(&qos_req,
							PM_QOS_IRDA_LAT_VALUE,
							PM_QOS_IRDA_TIMEOUT);
			else
				irda_power(dev, false);
		}
	} else {
		irda_trx_active(dev, false);
		irda_power(dev, false);
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

static struct irda_vreg_data irda_vreg_data;

static struct device_attribute irda_attr =
	__ATTR(irda, S_IRUSR | S_IWUSR, NULL, irda_store);

static int irda_suspend(struct device *dev)
{
	return irda_disable(dev);
}

static int irda_resume(struct device *dev)
{
	return irda_enable(dev);
}

static int __devinit irda_probe(struct platform_device *pdev)
{
	int ret;
	struct irda_platform_data *irda_data = pdev->dev.platform_data;

	if (!irda_data)
		return -EPERM;

	ret = gpio_request(irda_data->pwdown_gpio, "IRDA_PWDOWN");
	if (ret) {
		dev_err(&pdev->dev, "%s: GPIO %d: gpio_request failed\n",
			__func__, irda_data->pwdown_gpio);
		goto error_gpio_request;
	}
	platform_set_drvdata(pdev, &irda_vreg_data);
	ret = device_create_file(&pdev->dev, &irda_attr);
	if (ret) {
		dev_err(&pdev->dev, "%s: device_create_file failed\n",
			__func__);
		goto error_device_create_file;
	}
	ret = irda_enable(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "%s: irda_enable failed\n", __func__);
		goto error_irda_enable;
	}

	return 0;

error_irda_enable:
	device_remove_file(&pdev->dev, &irda_attr);
error_device_create_file:
	gpio_free(irda_data->pwdown_gpio);
error_gpio_request:
	return ret;
}

static int __devexit irda_remove(struct platform_device *pdev)
{
	struct irda_platform_data *irda_data = pdev->dev.platform_data;

	irda_disable(&pdev->dev);
	device_remove_file(&pdev->dev, &irda_attr);
	if (irda_data)
		gpio_free(irda_data->pwdown_gpio);

	return 0;
}

static const struct dev_pm_ops irda_pm_ops = {
	.suspend = irda_suspend,
	.resume = irda_resume,
};

static struct platform_driver irda_platform_driver = {
	.probe = irda_probe,
	.remove = __devexit_p(irda_remove),
	.driver = {
		.name = "irda",
		.owner = THIS_MODULE,
		.pm = &irda_pm_ops,
	},
};

static struct irda_platform_data irda_platform_data = {
	.pwdown_gpio = PM_GPIO_IRDA_PWDOWN,
	.uart_tx_gpio = UART_TX_IRDA_GPIO,
	.uart_rx_gpio = UART_RX_IRDA_GPIO,
};

static struct platform_device this_device = {
	.name = "irda",
	.id = -1,
	.dev = {
		.platform_data = &irda_platform_data,
	},
};

static struct platform_device *irda_devices[] = {
	&apq8064_device_uart_gsbi2,
	&this_device,
};

static struct msm_serial_hslite_platform_data msm_uart_gsbi2_pdata = {
	.line = 2,
	.type = PORT_IRDA,
};

static int __init irda_init(void)
{
	int error;

	error = platform_driver_register(&irda_platform_driver);
	if (error) {
		pr_err("%s: platform_driver_register failed\n", __func__);
		goto error_platform_driver_register;
	}
	apq8064_device_uart_gsbi2.dev.platform_data = &msm_uart_gsbi2_pdata;
	error = platform_add_devices(irda_devices, ARRAY_SIZE(irda_devices));
	if (error) {
		pr_err("%s: platform_add_devices failed\n", __func__);
		goto error_platform_add_devices;
	}
	pm_qos_add_request(&qos_req, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	return 0;

error_platform_add_devices:
	platform_driver_unregister(&irda_platform_driver);
error_platform_driver_register:
	return error;
}

static void __exit irda_exit(void)
{
	pm_qos_remove_request(&qos_req);
	platform_device_unregister(&this_device);
	platform_driver_unregister(&irda_platform_driver);
}

module_init(irda_init);
module_exit(irda_exit);
