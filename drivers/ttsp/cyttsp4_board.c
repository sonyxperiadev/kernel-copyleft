/*
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/msm_ssbi.h>
#include <linux/regulator/msm-gpio-regulator.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slimbus/slimbus.h>
#include <linux/bootmem.h>
#include <linux/delay.h>

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4
#include <linux/cyttsp4_bus.h>
#include <linux/cyttsp4_core.h>
#include <linux/cyttsp4_btn.h>
#include <linux/cyttsp4_mt.h>
#endif


#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_I2C
#define CYTTSP4_I2C_NAME "cyttsp4_i2c_adapter"
#define CYTTSP4_I2C_TCH_ADR 0x24
#define CYTTSP4_LDR_TCH_ADR 0x24

#define CYTTSP4_I2C_IRQ_GPIO 17
#ifdef CONFIG_MACH_VISKAN_HUASHAN_CT
#define CYTTSP4_I2C_RST_GPIO 16
#else
#define CYTTSP4_I2C_RST_GPIO 16
#endif
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_SPI
#define CYTTSP4_SPI_NAME "cyttsp4_spi_adapter"
#define CYTTSP4_SPI_IRQ_GPIO 38
#define CYTTSP4_SPI_RST_GPIO 37
#endif

/* Check GPIO numbers if both I2C and SPI are enabled */
#if defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_I2C) && \
	defined(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_SPI)
#if CYTTSP4_I2C_IRQ_GPIO == CYTTSP4_SPI_IRQ_GPIO || \
	CYTTSP4_I2C_RST_GPIO == CYTTSP4_SPI_RST_GPIO
#error "GPIO numbers should be different when both I2C and SPI are on!"
#endif
#endif

#define CY_MAXX 720
#define CY_MAXY 1280
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255
#define CY_ABS_MIN_T 0
#define CY_ABS_MAX_T 15

#define CY_IGNORE_VALUE 0xFFFF

#define VREG_VDDA_2P85 "8226_l19"
#define VREG_VDDD_1P8 "8226_lvs1"

static int cyttsp4_regulator_get(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(pdata->vreg_touch_vdda)) {
		pdata->vreg_touch_vdda = regulator_get(NULL, VREG_VDDA_2P85);

		if (IS_ERR(pdata->vreg_touch_vdda)) {
			dev_err(dev, "%s: get regulator %s failed.\n",
					__func__, VREG_VDDA_2P85);
			rc = -ENODEV;
			goto touch_vdda_err;
		}
	}

	if (IS_ERR_OR_NULL(pdata->vreg_i2c_vddd)) {
		pdata->vreg_i2c_vddd = regulator_get(NULL, VREG_VDDD_1P8);

		if (IS_ERR(pdata->vreg_i2c_vddd)) {
			dev_err(dev, "%s: get regulator %s failed.\n",
					__func__, VREG_VDDD_1P8);
			rc = -ENODEV;
			goto i2c_vddd_err;
		}
	}

	return rc;

i2c_vddd_err:
	regulator_put(pdata->vreg_touch_vdda);
	pdata->vreg_touch_vdda = NULL;
touch_vdda_err:
	return rc;
}

static int cyttsp4_vreg_configure(struct cyttsp4_core_platform_data *pdata,
		int enable, struct device *dev)
{
	int rc;

	rc = cyttsp4_regulator_get(pdata, dev);
	if (rc < 0)
		goto regulator_get_err;

	if (enable) {
		rc = regulator_set_voltage(pdata->vreg_touch_vdda,
						2850000, 2850000);
		if (rc < 0) {
			dev_err(dev, "%s: set vreg_touch_vdda fail, rc=%d\n",
					__func__, rc);
			goto touch_vdda_err;
		}

		rc = regulator_enable(pdata->vreg_touch_vdda);
		if (rc < 0) {
			dev_err(dev, "%s: enable vreg_touch_vdda fail, rc=%d\n",
					__func__, rc);
			goto touch_vdda_err;
		}

		rc = regulator_enable(pdata->vreg_i2c_vddd);
		if (rc < 0) {
			dev_err(dev, "%s: enable vreg_i2c_vddd fail, rc=%d\n",
					__func__, rc);
			goto i2c_vddd_err;
		}

	} else {
		rc = regulator_set_voltage(pdata->vreg_touch_vdda, 0, 2850000);
		if (rc < 0)
			dev_err(dev, "%s: set vreg_touch_vdda fail, rc=%d\n",
					__func__, rc);

		rc = regulator_disable(pdata->vreg_touch_vdda);
		if (rc < 0)
			dev_err(dev, "%s: disable vreg_touch_vdda fail rc=%d\n",
					__func__, rc);

		rc = regulator_disable(pdata->vreg_i2c_vddd);
		if (rc < 0)
			dev_err(dev, "%s: disable vreg_i2c_vddd fail, rc=%d\n",
					__func__, rc);
	}
	return rc;

i2c_vddd_err:
	regulator_disable(pdata->vreg_touch_vdda);
touch_vdda_err:
	regulator_put(pdata->vreg_touch_vdda);
	pdata->vreg_touch_vdda = NULL;
	regulator_put(pdata->vreg_i2c_vddd);
	pdata->vreg_i2c_vddd = NULL;
regulator_get_err:
	return rc;
}

static int cyttsp4_xres(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;

	gpio_set_value(rst_gpio, 1);
	msleep(20);
	gpio_set_value(rst_gpio, 0);
	msleep(40);
	gpio_set_value(rst_gpio, 1);
	msleep(20);
	dev_info(dev, "%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
			pdata->rst_gpio, rc);
	return rc;
}

static int cyttsp4_init(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (on) {
		rc = cyttsp4_vreg_configure(pdata, 1, dev);
		if (rc < 0) {
			dev_err(dev, "%s: enable touch vreg failed, rc=%d\n",
					__func__, rc);
			goto vreg_error;
		}

		rc = gpio_request(rst_gpio, "cyttsp4_reset");
		if (rc < 0) {
			dev_err(dev, "%s: Fail request gpio=%d\n",
					__func__, rst_gpio);
			goto gpio_error;
		}

		rc = gpio_direction_output(rst_gpio, 1);
		if (rc < 0) {
			dev_err(dev, "%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
			goto gpio_rst_error;
		}

		rc = gpio_request(irq_gpio, "cyttsp4_irq");
		if (rc < 0) {
			dev_err(dev, "%s: Fail request gpio=%d\n",
					__func__, irq_gpio);
			goto gpio_rst_error;
		}

		rc = gpio_direction_input(irq_gpio);
		if (rc < 0) {
			dev_err(dev, "%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
				goto gpio_irq_error;
		}
	} else {
		gpio_free(rst_gpio);
		gpio_free(irq_gpio);
		cyttsp4_vreg_configure(pdata, 0, dev);
	}

	dev_info(dev,
		"%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return 0;

gpio_irq_error:
	gpio_free(irq_gpio);
gpio_rst_error:
	gpio_free(rst_gpio);
gpio_error:
	cyttsp4_vreg_configure(pdata, 0, dev);
vreg_error:
	return rc;
}

static int cyttsp4_wakeup(struct cyttsp4_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (IS_ERR_OR_NULL(pdata->vreg_touch_vdda))
		return -ENODEV;

	rc = regulator_set_optimum_mode(pdata->vreg_touch_vdda, 15000);
	if (rc < 0) {
		dev_err(dev, "%s: set vreg_touch_vdda mode fail, rc=%d\n",
				__func__, rc);
		goto set_vdd_error;
	}

	if (ignore_irq)
		atomic_set(ignore_irq, 1);
	rc = gpio_direction_output(irq_gpio, 0);
	if (rc < 0) {
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		dev_err(dev,
			"%s: Fail set output gpio=%d\n",
			__func__, irq_gpio);
	} else {
		udelay(2000);
		rc = gpio_direction_input(irq_gpio);
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail set input gpio=%d\n",
				__func__, irq_gpio);
		}
	}

	dev_dbg(dev,
		"%s: WAKEUP CYTTSP gpio=%d r=%d\n", __func__,
		irq_gpio, rc);
set_vdd_error:
	return rc;
}

static int cyttsp4_sleep(struct cyttsp4_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int rc = 0;
	if (IS_ERR_OR_NULL(pdata->vreg_touch_vdda))
		return -ENODEV;

	rc = regulator_set_optimum_mode(pdata->vreg_touch_vdda, 1000);
	if (rc < 0)
		dev_err(dev, "%s: set vreg_touch_vdda mode fail, rc=%d\n",
				__func__, rc);

	return rc < 0 ? rc : 0;
}

static int cyttsp4_power(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp4_wakeup(pdata, dev, ignore_irq);
	else
		return cyttsp4_sleep(pdata, dev, ignore_irq);
}

static int cyttsp4_irq_stat(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

/* Button to keycode conversion */
static u16 cyttsp4_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_HOME,		/* 102 */
	KEY_MENU,		/* 139 */
	KEY_BACK,		/* 158 */
	KEY_SEARCH,		/* 217 */
	KEY_VOLUMEDOWN,		/* 114 */
	KEY_VOLUMEUP,		/* 115 */
	KEY_CAMERA,		/* 212 */
	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp4_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp4_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp4_btn_keys),
	.tag = 0,
};

static struct cyttsp4_core_platform_data _cyttsp4_core_platform_data = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_I2C
	.irq_gpio = CYTTSP4_I2C_IRQ_GPIO,
	.rst_gpio = CYTTSP4_I2C_RST_GPIO,
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_SPI
	.irq_gpio = CYTTSP4_SPI_IRQ_GPIO,
	.rst_gpio = CYTTSP4_SPI_RST_GPIO,
#endif
	.use_configure_sensitivity = 1,
	.use_auto_calibration = 1,
	.xres = cyttsp4_xres,
	.init = cyttsp4_init,
	.power = cyttsp4_power,
	.irq_stat = cyttsp4_irq_stat,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL, /* &cyttsp4_sett_param_regs, */
		NULL, /* &cyttsp4_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp4_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp4_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp4_sett_btn_keys,	/* button-to-keycode table */
	},
};

static struct cyttsp4_core cyttsp4_core_device = {
	.name = CYTTSP4_CORE_NAME,
	.id = "main_ttsp_core",
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_I2C
	.adap_id = CYTTSP4_I2C_NAME,
#endif
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_SPI
	.adap_id = CYTTSP4_SPI_NAME,
#endif
	.dev = {
		.platform_data = &_cyttsp4_core_platform_data,
	},
};

static const uint16_t cyttsp4_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -128, 127, 0, 0,
};

struct touch_framework cyttsp4_framework = {
	.abs = (uint16_t *)&cyttsp4_abs[0],
	.size = ARRAY_SIZE(cyttsp4_abs),
	.enable_vkeys = 0,
};

static struct cyttsp4_mt_platform_data _cyttsp4_mt_platform_data = {
	.frmwrk = &cyttsp4_framework,
	.flags = 0,
	.inp_dev_name = CYTTSP4_MT_NAME,
};

struct cyttsp4_device cyttsp4_mt_device = {
	.name = CYTTSP4_MT_NAME,
	.core_id = "main_ttsp_core",
	.dev = {
		.platform_data = &_cyttsp4_mt_platform_data,
	}
};

static struct cyttsp4_btn_platform_data _cyttsp4_btn_platform_data = {
	.inp_dev_name = CYTTSP4_BTN_NAME,
};

struct cyttsp4_device cyttsp4_btn_device = {
	.name = CYTTSP4_BTN_NAME,
	.core_id = "main_ttsp_core",
	.dev = {
		.platform_data = &_cyttsp4_btn_platform_data,
	}
};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_VIRTUAL_KEYS
static ssize_t cyttps4_virtualkeys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf,
		__stringify(EV_KEY) ":"
		__stringify(KEY_BACK) ":1360:90:160:180:"
		__stringify(EV_KEY) ":"
		__stringify(KEY_MENU) ":1360:270:160:180:"
		__stringify(EV_KEY) ":"
		__stringify(KEY_HOME) ":1360:450:160:180:"
		__stringify(EV_KEY) ":"
		__stringify(KEY_SEARCH) ":1360:630:160:180");
}

static struct kobj_attribute cyttsp4_virtualkeys_attr = {
	.attr = {
		.name = "virtualkeys.cyttsp4_mt",
		.mode = S_IRUGO,
	},
	.show = &cyttps4_virtualkeys_show,
};

static struct attribute *cyttsp4_properties_attrs[] = {
	&cyttsp4_virtualkeys_attr.attr,
	NULL
};

static struct attribute_group cyttsp4_properties_attr_group = {
	.attrs = cyttsp4_properties_attrs,
};
#endif

static int cyttsp4_board_probe(struct platform_device *pdev)
{
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_VIRTUAL_KEYS
	struct kobject *properties_kobj;
	int ret;
#endif

	/* Register core and devices */
	cyttsp4_register_core_device(&cyttsp4_core_device);
	cyttsp4_register_device(&cyttsp4_mt_device);
	cyttsp4_register_device(&cyttsp4_btn_device);

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_VIRTUAL_KEYS
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				&cyttsp4_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: failed to create board_properties\n", __func__);
#endif

	return 0;
}

static int cyttsp4_board_remove(struct platform_device *pdev)
{
	return 0;
	}

static struct of_device_id cyttsp4_board_match[] = {
	{.compatible = "cypress,cyttsp4_core"},
};

static struct platform_driver cyttsp4_board_driver = {
	.probe = cyttsp4_board_probe,
	.remove = cyttsp4_board_remove,
	.driver = {
		.name = "cyttsp4",
		.owner = THIS_MODULE,
		.of_match_table = cyttsp4_board_match,
	},
};

static int __init cyttsp4_board_init(void)
{
	int rc = 0;

	rc = platform_driver_register(&cyttsp4_board_driver);

	pr_info("%s: Cypress Board (Built %s @ %s) rc=%d\n",
		 __func__, __DATE__, __TIME__, rc);
	return rc;
}
module_init(cyttsp4_board_init);

static void __exit cyttsp4_board_exit(void)
{
	platform_driver_unregister(&cyttsp4_board_driver);

	pr_info("%s: module exit\n", __func__);
}
module_exit(cyttsp4_board_exit);
#endif
