/*
 * Copyright 2025 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Willsemi Co. Ltd.
 *
 * Author: Ray Deng <ray.deng@corp.ovt.com>
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include "wl28681c-regulator.h"
#include <linux/sysfs.h>
#include <linux/mutex.h>

#define WL28681_DEVICE_D1		0x00
#define WL28681_DEVICE_D2		0x01
#define WL28681_ILIM			0x02
#define WL28681_LDO_EN			0x03

#define WL28681_LDO1_VOUT		0x04
#define WL28681_LDO2_VOUT		0x05
#define WL28681_LDO3_VOUT		0x06
#define WL28681_LDO4_VOUT		0x07
#define WL28681_LDO5_VOUT		0x08
#define WL28681_LDO6_VOUT		0x09
#define WL28681_LDO7_VOUT		0x0a

#define WL28681_LDO1_LDO2_SEQ		0x0b
#define WL28681_LDO3_LDO4_SEQ		0x0c
#define WL28681_LDO5_LDO6_SEQ		0x0d
#define WL28681_LDO7_SEQ			0x0e
#define WL28681_SEQ_STATUS			0x0f

#define WL28681_DISCHARGE_RESISTORS		0x10
#define WL28681_RESET					0x11
#define WL28681_REPROGRAMMABLE_I2C_ADDR	0x12

#define WL28681_LDO1234_COMP	0x13
#define WL28681_LDO567_COMP		0x14

#define WL28681_UVP_INT			0x15
#define WL28681_OCP_INT			0x16
#define WL28681_TSD_UVLO_INT	0x17
#define WL28681_UVP_INT_STATUS			0x18
#define WL28681_OCP_INT_STATUS			0x19
#define WL28681_TSD_UVLO_INT_STATUS		0x1a
#define WL28681_SUSD_STATUS				0x1b
#define WL28681_UVP_INT_MASK			0x1c
#define WL28681_OCP_INT_MASK			0x1d
#define WL28681_TSD_UVLO_INT_MASK		0x1e
#define WL28681_MAX_REG_NO				0x1f

#define WL28681_VSEL_MASK		0xff

struct mutex i2c_control_mutex;

enum wl28681_regulators {
	WL28681_REGULATOR_LDO1 = 0,
	WL28681_REGULATOR_LDO2,
	WL28681_REGULATOR_LDO3,
	WL28681_REGULATOR_LDO4,
	WL28681_REGULATOR_LDO5,
	WL28681_REGULATOR_LDO6,
	WL28681_REGULATOR_LDO7,
	WL28681_MAX_REGULATORS,
};

/* ldo1~7  0-current,1-current */
/*
 *	LDO1	1460000, 2000000
 *	LDO2	1460000, 2000000
 *	LDO3	500000, 700000
 *	LDO4	500000, 700000
 *	LDO5	740000, 950000
 *	LDO6	500000, 700000
 *	LDO7	740000, 950000
 */
static const unsigned int wl28681_crtable1[] = {1460000, 2000000};
static const unsigned int wl28681_crtable2[] = {1460000, 2000000};
static const unsigned int wl28681_crtable3[] = {500000, 700000};
static const unsigned int wl28681_crtable4[] = {500000, 700000};
static const unsigned int wl28681_crtable5[] = {740000, 950000};
static const unsigned int wl28681_crtable6[] = {500000, 700000};
static const unsigned int wl28681_crtable7[] = {740000, 950000};

struct wl28681 {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_dev *rdev;
	struct gpio_desc *reset_gpio;
	int min_dropout_uv;
	int ldo_vout[7];
	int ldo_en;
};

static int wl28681_list_voltage_linear(struct regulator_dev *rdev,
				  unsigned int selector)
{
	const struct regulator_desc *desc = rdev->desc;

	pr_debug("%s:[selector,n_voltages,linear_min_sel,min_uV,uV_step]=\
			[%d,%d,%d,%d,%d]\n", __func__, selector, desc->n_voltages, 
			desc->linear_min_sel, desc->min_uV, desc->uV_step);
	if (selector >= desc->n_voltages)
		return -EINVAL;

	if (selector < desc->linear_min_sel)
		return 0;

	selector -= desc->linear_min_sel;

	return (desc->min_uV + (desc->uV_step * selector));
}

static int wl28681_map_voltage_linear(struct regulator_dev *rdev,
				 int min_uV, int max_uV)
{
	int ret, voltage;
	const struct regulator_desc *desc = rdev->desc;

	pr_info("%s:[n_voltages,uV_step,min_uV,max_uV,d_min_uV,linear_min_sel]=\
			[%d,%d,%d,%d,%d,%d]\n", __func__, desc->n_voltages,
			desc->uV_step, min_uV, max_uV, desc->min_uV,desc->linear_min_sel);
	/* Allow uV_step to be 0 for fixed voltage */
	if (desc->n_voltages == 1 && desc->uV_step == 0) {
		if (min_uV <= desc->min_uV && desc->min_uV <= max_uV)
			return 0;
		else
			return -EINVAL;
	}

	if (!desc->uV_step) {
		BUG_ON(!desc->uV_step);
		return -EINVAL;
	}

	if (min_uV < desc->min_uV)
		min_uV = desc->min_uV;

	ret = DIV_ROUND_UP(min_uV - desc->min_uV, desc->uV_step);
	if (ret < 0)
		return ret;

	ret += desc->linear_min_sel;

	/* Map back into a voltage to verify we're still in bounds */
	voltage = desc->ops->list_voltage(rdev, ret);
	if (voltage < min_uV || voltage > max_uV)
		return -EINVAL;

	return ret;
}

static int wl28681_get_voltage_sel_regmap(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;
	const struct regulator_desc *desc = rdev->desc;

	ret = regmap_read(rdev->regmap, desc->vsel_reg, &val);
	pr_info("%s:[vsel_reg,val,vsel_mask]=[%d,%d,%d] ret=%d\n",
			 __func__, desc->vsel_reg, val, desc->vsel_mask, ret);
	if (ret != 0)
		return ret;

	val &= desc->vsel_mask;
	val >>= ffs(desc->vsel_mask) - 1;

	return val;
}

static int wl28681_set_voltage_sel_regmap(struct regulator_dev *rdev, unsigned sel)
{
	int ret;
	const struct regulator_desc *desc = rdev->desc;

	pr_info("%s:[sel,vsel_mask,vsel_reg,apply_bit,apply_reg]=\
			[%d,%d,%d,%d,%d]\n", __func__, sel, desc->vsel_mask,
			desc->vsel_reg, desc->apply_bit, desc->apply_reg);

	sel <<= ffs(desc->vsel_mask) - 1;

	ret = regmap_update_bits(rdev->regmap, desc->vsel_reg,
				  desc->vsel_mask, sel);
	if (ret)
		return ret;

	if (desc->apply_bit)
		ret = regmap_update_bits(rdev->regmap, desc->apply_reg,
					 desc->apply_bit,
					 desc->apply_bit);
	return ret;
}

static int wl28681_set_current_limit_regmap(struct regulator_dev *rdev,
				       int min_uA, int max_uA)
{
	int i, sel = -1;
	const struct regulator_desc *desc = rdev->desc;
	unsigned int n_currents = desc->n_current_limits;

	pr_info("%s:[n_currents,min_uA,max_uA,csel_reg,csel_mask]=\
			[%d,%d,%d,%d,%d]\n", __func__, n_currents, min_uA, max_uA,
			rdev->desc->vsel_reg, rdev->desc->vsel_mask);

	if (n_currents == 0)
		return -EINVAL;

	if (rdev->desc->curr_table) {
		const unsigned int *curr_table = rdev->desc->curr_table;
		bool ascend = curr_table[n_currents - 1] > curr_table[0];

		/* search for closest to maximum */
		if (ascend) {
			for (i = n_currents - 1; i >= 0; i--) {
				if (min_uA <= curr_table[i] &&
				    curr_table[i] <= max_uA) {
					sel = i;
					break;
				}
			}
		} else {
			for (i = 0; i < n_currents; i++) {
				if (min_uA <= curr_table[i] &&
				    curr_table[i] <= max_uA) {
					sel = i;
					break;
				}
			}
		}
	}

	if (sel < 0)
		return -EINVAL;

	sel <<= ffs(rdev->desc->csel_mask) - 1;

	return regmap_update_bits(rdev->regmap, rdev->desc->csel_reg,
				  rdev->desc->csel_mask, sel);
}

static int wl28681_enable_regmap(struct regulator_dev *rdev)
{
	unsigned int val, ret;

	mutex_lock(&i2c_control_mutex);
	if (rdev->desc->enable_is_inverted) {
		val = rdev->desc->disable_val;
	} else {
		val = rdev->desc->enable_val;
		if (!val)
			val = rdev->desc->enable_mask;
	}

	pr_info("%s:[enable_val,enable_reg,enable_mask,val]=\
			[%d,%d,%d,%d]\n", __func__, rdev->desc->enable_val,
			rdev->desc->enable_reg, rdev->desc->vsel_mask, val);

	ret = regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask, val);
	mutex_unlock(&i2c_control_mutex);
	return ret;
}

static int wl28681_disable_regmap(struct regulator_dev *rdev)
{
	unsigned int val, ret;

	mutex_lock(&i2c_control_mutex);
	if (rdev->desc->enable_is_inverted) {
		val = rdev->desc->enable_val;
		if (!val)
			val = rdev->desc->enable_mask;
	} else {
		val = rdev->desc->disable_val;
	}

	pr_info("%s:[disable_val,enable_reg,enable_mask,val]=\
			[%d,%d,%d,%d]\n", __func__, rdev->desc->disable_val,
			rdev->desc->enable_reg, rdev->desc->vsel_mask, val);

	ret = regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask, val);
	mutex_unlock(&i2c_control_mutex);
	return ret;
}

static int wl28681_is_enabled_regmap(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, rdev->desc->enable_reg, &val);
	if (ret != 0)
		return ret;
	pr_info("%s:[enable_reg,enable_mask,val]=[%d,%d,%d]\n",
			__func__, rdev->desc->enable_reg, rdev->desc->vsel_mask, val);
	val &= rdev->desc->enable_mask;

	if (rdev->desc->enable_is_inverted) {
		if (rdev->desc->enable_val)
			return val != rdev->desc->enable_val;
		return val == 0;
	} else {
		if (rdev->desc->enable_val)
			return val == rdev->desc->enable_val;
		return val != 0;
	}
}

static const struct regulator_ops wl28681_reg_ops = {
	.list_voltage		= wl28681_list_voltage_linear,
	.map_voltage		= wl28681_map_voltage_linear,
	.get_voltage_sel	= wl28681_get_voltage_sel_regmap,
	.set_voltage_sel	= wl28681_set_voltage_sel_regmap,
	.set_current_limit  = wl28681_set_current_limit_regmap,
	.enable			= wl28681_enable_regmap,
	.disable		= wl28681_disable_regmap,
	.is_enabled		= wl28681_is_enabled_regmap,
};

#define WL28681_DESC(_id, _match, _supply, _min, _max, _step, _vreg, _vmask,	\
	 _ereg, _emask, _enval, _disval, _curr_table, _creg, _cmask, _minsel)		\
	{								\
		.id		= (_id),			\
		.name		= (_match),		\
		.of_match	= of_match_ptr(_match),		\
		.supply_name	= (_supply),			\
		.min_uV		= (_min) * 1000,			\
		.uV_step	= (_step) * 1000,			\
		.n_voltages	= (((_max) - (_min)) / (_step) + 1),	\
		.n_current_limits = ARRAY_SIZE(_curr_table),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.type		= REGULATOR_VOLTAGE,	\
		.vsel_reg	= (_vreg),				\
		.vsel_mask	= (_vmask),				\
		.csel_reg	= (_creg),				\
		.csel_mask	= (_cmask),				\
		.enable_reg	= (_ereg),				\
		.enable_mask	= (_emask),				\
		.enable_val     = (_enval),				\
		.disable_val     = (_disval),			\
		.ops		= &wl28681_reg_ops,			\
		.curr_table = _curr_table,				\
		.linear_min_sel = _minsel,				\
		.owner		= THIS_MODULE,				\
	}

static const struct regulator_desc wl28681_reg[] = {
	WL28681_DESC(WL28681_REGULATOR_LDO1, "wl28681c_l1", "vin12", 496, 2048, 8,
		     WL28681_LDO1_VOUT, WL28681_VSEL_MASK, WL28681_LDO_EN, BIT(0),
			 BIT(0), 0,	wl28681_crtable1, WL28681_ILIM, BIT(0), 61),
	WL28681_DESC(WL28681_REGULATOR_LDO2, "wl28681c_l2", "vin12", 496, 2048, 8,
		     WL28681_LDO2_VOUT, WL28681_VSEL_MASK, WL28681_LDO_EN, BIT(1),
			 BIT(1), 0, wl28681_crtable2, WL28681_ILIM, BIT(1), 61),
	WL28681_DESC(WL28681_REGULATOR_LDO3, "wl28681c_l3", "vin34", 1372, 3412, 8,
		     WL28681_LDO3_VOUT, WL28681_VSEL_MASK, WL28681_LDO_EN, BIT(2),
			 BIT(2), 0, wl28681_crtable3, WL28681_ILIM, BIT(2), 1),
	WL28681_DESC(WL28681_REGULATOR_LDO4, "wl28681c_l4", "vin34", 1372, 3412, 8,
		     WL28681_LDO4_VOUT, WL28681_VSEL_MASK, WL28681_LDO_EN, BIT(3),
			 BIT(3), 0, wl28681_crtable4, WL28681_ILIM, BIT(3), 1),
	WL28681_DESC(WL28681_REGULATOR_LDO5, "wl28681c_l5", "vin5", 1372, 3412, 8,
		     WL28681_LDO5_VOUT, WL28681_VSEL_MASK, WL28681_LDO_EN, BIT(4),
			 BIT(4), 0, wl28681_crtable5, WL28681_ILIM, BIT(4), 1),
	WL28681_DESC(WL28681_REGULATOR_LDO6, "wl28681c_l6", "vin6", 1372, 3412, 8,
		     WL28681_LDO6_VOUT, WL28681_VSEL_MASK, WL28681_LDO_EN, BIT(5),
			 BIT(5), 0, wl28681_crtable6, WL28681_ILIM, BIT(5), 1),
	WL28681_DESC(WL28681_REGULATOR_LDO7, "wl28681c_l7", "vin7", 1372, 3412, 8,
		     WL28681_LDO7_VOUT, WL28681_VSEL_MASK, WL28681_LDO_EN, BIT(6),
			 BIT(6), 0, wl28681_crtable7, WL28681_ILIM, BIT(6), 1),
};

static const struct regmap_range wl28681_writeable_ranges[] = {
	regmap_reg_range(WL28681_ILIM, WL28681_LDO567_COMP),
};

static const struct regmap_range wl28681_readable_ranges[] = {
	regmap_reg_range(WL28681_DEVICE_D1, WL28681_TSD_UVLO_INT_MASK),
};

static const struct regmap_range wl28681_volatile_ranges[] = {
	regmap_reg_range(WL28681_DEVICE_D1, WL28681_TSD_UVLO_INT_MASK),
};

static const struct regmap_access_table wl28681_writeable_table = {
	.yes_ranges   = wl28681_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(wl28681_writeable_ranges),
};

static const struct regmap_access_table wl28681_readable_table = {
	.yes_ranges   = wl28681_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(wl28681_readable_ranges),
};

static const struct regmap_access_table wl28681_volatile_table = {
	.yes_ranges   = wl28681_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(wl28681_volatile_ranges),
};

static const struct regmap_config wl28681_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL28681_MAX_REG_NO,
	.wr_table = &wl28681_writeable_table,
	.rd_table = &wl28681_readable_table,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &wl28681_volatile_table,
};

static void wl28681_reset(struct wl28681 *wl28681)
{
	pr_debug("wl28681 reset gpio low\n");
	gpiod_set_value_cansleep(wl28681->reset_gpio, 0);	//set H
	usleep_range(10000, 11000);
	pr_debug("wl28681 reset gpio high\n");
	gpiod_set_value_cansleep(wl28681->reset_gpio, 1);	//set L
	usleep_range(10000, 11000);
}

static  struct i2c_client *pclient;
static ssize_t moveVCM_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t chipid = -1;

	chipid = i2c_smbus_read_byte_data(pclient, 0x00);
	return sprintf(buf, "device addr:%x, current id:%d\n", pclient->addr, chipid);
}

static ssize_t moveVCM_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	//todo, for debug
	int ret = 0;
	uint32_t value = 0;
	ret = kstrtou32(buf, 0, &value);
	if (ret < 0) {
		dev_err(dev, "cam_actuator_power_on Number of parameters error");
		return ret;
	}

	return count;
}

static DEVICE_ATTR(moveVCM, 0664, moveVCM_show, moveVCM_store);

static int wl28681_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	const struct regulator_desc *regulators;
	struct wl28681 *wl28681;
	int ret, i;
	int chipid = 0;
	struct pinctrl *pinctrl;

	pclient = client;
	wl28681 = devm_kzalloc(dev, sizeof(struct wl28681), GFP_KERNEL);
	if (!wl28681)
		return -ENOMEM;

	wl28681->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(wl28681->reset_gpio)) {
		ret = PTR_ERR(wl28681->reset_gpio);
		dev_err(dev, "failed to request reset GPIO: %d\n", ret);
		return ret;
	}
	ret = sysfs_create_file(&dev->kobj, &dev_attr_moveVCM.attr);

	pinctrl = devm_pinctrl_get_select(dev, "reset_low");
	if (IS_ERR(pinctrl)) {
  		ret = PTR_ERR(pinctrl);
  		dev_err(dev, "%s failed to select default state %d\n",
  			__func__, ret);
	} else {
		dev_info(dev, "wl28681 reset_low\n");
	}
	usleep_range(10000, 11000);

	pinctrl = devm_pinctrl_get_select(dev, "reset_high");
	if (IS_ERR(pinctrl)) {
  		ret = PTR_ERR(pinctrl);
  		dev_err(dev, "%s failed to select default state %d\n",
  			__func__, ret);
	} else {
		dev_info(dev, "wl28681 reset_high\n");
	}
	usleep_range(10000, 11000);

	//wl28681_reset(wl28681);

	chipid = i2c_smbus_read_byte_data(client, 0x00);
	dev_info(dev, "wl28681 probe read id=0x%d\n", chipid);

	i2c_set_clientdata(client, wl28681);
	wl28681->dev = dev;
	wl28681->regmap = devm_regmap_init_i2c(client, &wl28681_regmap_config);
	if (IS_ERR(wl28681->regmap)) {
		ret = PTR_ERR(wl28681->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	mutex_init(&i2c_control_mutex);
	config.dev = &client->dev;
	config.regmap = wl28681->regmap;
	regulators = wl28681_reg;

	/* Instantiate the regulators */
	for (i = 0; i < WL28681_MAX_REGULATORS; i++) {
		rdev = devm_regulator_register(&client->dev,
					       &regulators[i], &config);
		if (IS_ERR(rdev)) {
			dev_err(&client->dev, "register ldo %d failed\n", i);
			//return PTR_ERR(rdev);
		} else {
			dev_info(&client->dev, "register ldo %d sucessed\n", i);
		}

	}

	/*Inital related mask for interrupt here*/
	regmap_write(wl28681->regmap, WL28681_UVP_INT_MASK, 0);
	regmap_write(wl28681->regmap, WL28681_OCP_INT_MASK, 0);
	regmap_write(wl28681->regmap, WL28681_TSD_UVLO_INT_MASK, 0);
	regmap_write(wl28681->regmap, WL28681_RESET, 0x07);

	return 0;
}

static void wl28681_regulator_shutdown(struct i2c_client *client)
{
	struct wl28681 *wl28681 = i2c_get_clientdata(client);

	if (system_state == SYSTEM_POWER_OFF)
		regmap_write(wl28681->regmap, WL28681_LDO_EN, 0x80);
}

static int __maybe_unused wl28681_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wl28681 *wl28681 = i2c_get_clientdata(client);
	int i;

	regmap_read(wl28681->regmap, WL28681_LDO_EN, &wl28681->ldo_en);
	for (i = 0; i < ARRAY_SIZE(wl28681->ldo_vout); i++)
		regmap_read(wl28681->regmap, WL28681_LDO1_VOUT + i,
			    &wl28681->ldo_vout[i]);

	return 0;
}

static int __maybe_unused wl28681_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wl28681 *wl28681 = i2c_get_clientdata(client);
	int i;

	wl28681_reset(wl28681);
	for (i = 0; i < ARRAY_SIZE(wl28681->ldo_vout); i++)
		regmap_write(wl28681->regmap, WL28681_LDO1_VOUT + i,
			     wl28681->ldo_vout[i]);
	regmap_write(wl28681->regmap, WL28681_LDO_EN, wl28681->ldo_en);

	return 0;
}

static SIMPLE_DEV_PM_OPS(wl28681_pm_ops, wl28681_suspend, wl28681_resume);

static const struct i2c_device_id wl28681_i2c_id[] = {
	{ "wl28681", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, wl28681_i2c_id);

static const struct of_device_id wl28681_of_match[] = {
	{ .compatible = "willsemi,wl28681" },
	{}
};
MODULE_DEVICE_TABLE(of, wl28681_of_match);

static struct i2c_driver wl28681_regulator_driver = {
	.driver = {
		.name = "wl28681",
		.of_match_table = of_match_ptr(wl28681_of_match),
		.pm = &wl28681_pm_ops,
	},
	.id_table = wl28681_i2c_id,
	.probe	= wl28681_i2c_probe,
	.shutdown = wl28681_regulator_shutdown,
};
/*
int __init wl28681c_chip_init_module(void)
{
	if (i2c_add_driver(&wl28681_i2c_driver)) {
		pr_err("i2c driver register failed");
		return -EINVAL;
	}
	return 0;
}

void __exit wl28681c_chip_exit_module(void)
{
	i2c_del_driver(&wl28681_i2c_driver);
}
*/
module_i2c_driver(wl28681_regulator_driver);

MODULE_DESCRIPTION("WL28681 regulator driver");
MODULE_AUTHOR("willsemi");
MODULE_LICENSE("GPL");
