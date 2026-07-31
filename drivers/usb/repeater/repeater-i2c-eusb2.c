// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/qti-regmap-debugfs.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/usb/repeater.h>

#define NUM_SUPPLIES			2

/* NXP eUSB2 repeater registers */
#define RESET_CONTROL			0x01
#define LINK_CONTROL1			0x02
#define LINK_CONTROL2			0x03
#define eUSB2_RX_CONTROL		0x04
#define eUSB2_TX_CONTROL		0x05
#define USB2_RX_CONTROL			0x06
#define USB2_TX_CONTROL1		0x07
#define USB2_TX_CONTROL2		0x08
#define USB2_HS_TERMINATION		0x09
#define RAP_SIGNATURE			0x0D
#define VDX_CONTROL			0x0E
#define DEVICE_STATUS			0x0F
#define LINK_STATUS			0x10
#define REVISION_ID			0x13
#define CHIP_ID_0			0x14
#define CHIP_ID_1			0x15
#define CHIP_ID_2			0x16

/* TI eUSB2 repeater registers */
#define GPIO0_CONFIG			0x00
#define GPIO1_CONFIG			0x40
#define UART_PORT1			0x50
#define EXTRA_PORT1			0x51
#define U_TX_ADJUST_PORT1		0x70
#define U_HS_TX_PRE_EMPHASIS_P1		0x71
#define U_RX_ADJUST_PORT1		0x72
#define U_DISCONNECT_SQUELCH_PORT1	0x73
#define E_HS_TX_PRE_EMPHASIS_P1		0x77
#define E_TX_ADJUST_PORT1		0x78
#define E_RX_ADJUST_PORT1		0x79
#define REV_ID				0xB0
#define GLOBAL_CONFIG			0xB2
#define INT_ENABLE_1			0xB3
#define INT_ENABLE_2			0xB4
#define BC_CONTROL			0xB6
#define BC_STATUS_1			0xB7
#define INT_STATUS_1			0xA3
#define INT_STATUS_2			0xA4

enum eusb2_repeater_type {
	TI_REPEATER,
	NXP_REPEATER,
};

struct i2c_repeater_chip {
	enum eusb2_repeater_type repeater_type;
};

struct eusb2_repeater {
	struct device			*dev;
	struct usb_repeater		ur;
	struct regmap			*regmap;
	const struct i2c_repeater_chip	*chip;
	u16				reg_base;
	struct regulator_bulk_data	*supplies;
	bool				power_enabled;

	struct gpio_desc		*reset_gpiod;
	u32				*param_override_seq;
	u8				param_override_seq_cnt;
};

static const struct regmap_config eusb2_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int eusb2_i2c_read_reg(struct eusb2_repeater *er, u8 reg, u8 *val)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(er->regmap, reg, &reg_val);
	if (ret < 0) {
		dev_err(er->dev, "Failed to read reg:0x%02x ret=%d\n", reg, ret);
		return ret;
	}

	*val = reg_val;
	dev_dbg(er->dev, "read reg:0x%02x val:0x%02x\n", reg, *val);

	return 0;
}

static int eusb2_i2c_write_reg(struct eusb2_repeater *er, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(er->regmap, reg, val);
	if (ret < 0) {
		dev_err(er->dev, "failed to write 0x%02x to reg: 0x%02x ret=%d\n", val, reg, ret);
		return ret;
	}

	dev_dbg(er->dev, "write reg:0x%02x val:0x%02x\n", reg, val);

	return 0;
}

static void eusb2_repeater_update_seq(struct eusb2_repeater *er, u32 *seq, u8 cnt)
{
	int i;

	dev_dbg(er->ur.dev, "param override seq count:%d\n", cnt);
	for (i = 0; i < cnt; i = i+2) {
		dev_dbg(er->ur.dev, "write 0x%02x to 0x%02x\n", seq[i], seq[i+1]);
		eusb2_i2c_write_reg(er, seq[i+1], seq[i]);
	}
}

static int eusb2_repeater_power(struct eusb2_repeater *er, bool on)
{
	int ret = 0;

	dev_dbg(er->ur.dev, "%s turn %s regulators. power_enabled:%d\n",
			__func__, on ? "on" : "off", er->power_enabled);

	if (er->power_enabled == on) {
		dev_dbg(er->ur.dev, "regulators are already ON.\n");
		return 0;
	}

	if (!on) {
		ret = regulator_bulk_disable(NUM_SUPPLIES, er->supplies);
		if (ret < 0) {
			dev_err(er->ur.dev, "Unable to turn off regulators:%d\n", ret);
			return ret;
		}

		er->power_enabled = false;
		pr_debug("%s(): eUSB2 repeater regulators are turned OFF.\n", __func__);
		return ret;
	}

	ret = regulator_bulk_enable(NUM_SUPPLIES, er->supplies);
	if (ret < 0) {
		dev_err(er->ur.dev, "Unable to turn on regulators:%d\n", ret);
		return ret;
	}

	er->power_enabled = true;
	pr_debug("%s(): eUSB2 repeater egulators are turned ON.\n", __func__);
	return ret;
}

static int eusb2_repeater_init(struct usb_repeater *ur)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);
	const struct i2c_repeater_chip *chip = er->chip;
	u8 reg_val;

	switch (chip->repeater_type) {
	case TI_REPEATER:
		eusb2_i2c_read_reg(er, REV_ID, &reg_val);
		/* If the repeater revision is B1 disable auto-resume WA */
		if (reg_val == 0x03)
			ur->flags |= UR_AUTO_RESUME_SUPPORTED;
		break;
	case NXP_REPEATER:
		eusb2_i2c_read_reg(er, REVISION_ID, &reg_val);
		break;
	default:
		dev_err(er->ur.dev, "Invalid repeater\n");
		return -EINVAL;
	}

	dev_dbg(er->ur.dev, "eUSB2 repeater version = 0x%x ur->flags:0x%x\n", reg_val, ur->flags);

	/* override init sequence using devicetree based values */
	if (er->param_override_seq_cnt)
		eusb2_repeater_update_seq(er, er->param_override_seq,
					er->param_override_seq_cnt);

	dev_dbg(er->ur.dev, "eUSB2 repeater init\n");

	return 0;
}

static int eusb2_repeater_reset(struct usb_repeater *ur, bool bring_out_of_reset)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);

	dev_dbg(ur->dev, "reset gpio:%s\n",
			bring_out_of_reset ? "assert" : "deassert");
	gpiod_set_value_cansleep(er->reset_gpiod, bring_out_of_reset);
	return 0;
}

static int eusb2_repeater_powerup(struct usb_repeater *ur)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);

	return eusb2_repeater_power(er, true);
}

static int eusb2_repeater_powerdown(struct usb_repeater *ur)
{
	struct eusb2_repeater *er =
			container_of(ur, struct eusb2_repeater, ur);

	return eusb2_repeater_power(er, false);
}

static const struct regulator_bulk_data eusb2_repeater_supplies[NUM_SUPPLIES] = {
	{
		.supply = "vdd3v3",
		.init_load_uA = 3500,
	}, {
		.supply = "vdd1v8",
		.init_load_uA = 32000,
	}
};

static struct i2c_repeater_chip repeater_chip[] = {
	[NXP_REPEATER] = {
		.repeater_type = NXP_REPEATER,
	},
	[TI_REPEATER] = {
		.repeater_type = TI_REPEATER,
	}
};

static const struct of_device_id eusb2_repeater_id_table[] = {
	{
		.compatible = "nxp,eusb2-repeater",
		.data = &repeater_chip[NXP_REPEATER]
	},
	{
		.compatible = "ti,eusb2-repeater",
		.data = &repeater_chip[TI_REPEATER]
	},
	{ }
};
MODULE_DEVICE_TABLE(of, eusb2_repeater_id_table);

static int eusb2_repeater_i2c_probe(struct i2c_client *client)
{
	struct eusb2_repeater *er;
	struct device *dev = &client->dev;
	const struct of_device_id *match;
	int ret, num_elem;

	er = devm_kzalloc(dev, sizeof(*er), GFP_KERNEL);
	if (!er)
		return -ENOMEM;

	er->dev = dev;
	match = of_match_node(eusb2_repeater_id_table, dev->of_node);
	er->chip = match->data;

	er->regmap = devm_regmap_init_i2c(client, &eusb2_i2c_regmap);
	if (IS_ERR(er->regmap))
		return dev_err_probe(dev, PTR_ERR(er->regmap),
					"failed to allocate register map\n");

	devm_regmap_qti_debugfs_register(er->dev, er->regmap);
	i2c_set_clientdata(client, er);

	ret = of_property_read_u16(dev->of_node, "reg", &er->reg_base);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get reg base address\n");

	ret = devm_regulator_bulk_get_const(dev, NUM_SUPPLIES,
			eusb2_repeater_supplies, &er->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "unable to parse regulators from dt\n");

	er->reset_gpiod = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(er->reset_gpiod))
		return PTR_ERR(er->reset_gpiod);

	num_elem = of_property_count_elems_of_size(dev->of_node, "qcom,param-override-seq",
				sizeof(*er->param_override_seq));
	if (num_elem > 0) {
		if (num_elem % 2)
			return dev_err_probe(dev, -EINVAL, "invalid param_override_seq_len\n");

		er->param_override_seq_cnt = num_elem;
		er->param_override_seq = devm_kcalloc(dev,
				er->param_override_seq_cnt,
				sizeof(*er->param_override_seq), GFP_KERNEL);
		if (!er->param_override_seq)
			return -ENOMEM;

		ret = of_property_read_u32_array(dev->of_node,
				"qcom,param-override-seq",
				er->param_override_seq,
				er->param_override_seq_cnt);
		if (ret)
			return dev_err_probe(dev, ret,
					"qcom,param-override-seq read failed\n");
	}


	er->ur.dev = dev;

	er->ur.init		= eusb2_repeater_init;
	er->ur.reset		= eusb2_repeater_reset;
	er->ur.powerup		= eusb2_repeater_powerup;
	er->ur.powerdown	= eusb2_repeater_powerdown;

	return usb_add_repeater_dev(&er->ur);
}

static void eusb2_repeater_i2c_remove(struct i2c_client *client)
{
	struct eusb2_repeater *er = i2c_get_clientdata(client);

	usb_remove_repeater_dev(&er->ur);
	eusb2_repeater_power(er, false);
}

static struct i2c_driver eusb2_i2c_repeater_driver = {
	.probe		= eusb2_repeater_i2c_probe,
	.remove		= eusb2_repeater_i2c_remove,
	.driver = {
		.name	= "eusb2-repeater",
		.of_match_table = of_match_ptr(eusb2_repeater_id_table),
	},
};

module_i2c_driver(eusb2_i2c_repeater_driver);

MODULE_DESCRIPTION("eUSB2 i2c repeater driver");
MODULE_LICENSE("GPL");
