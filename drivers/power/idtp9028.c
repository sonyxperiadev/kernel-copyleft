/*
 * Authors: Tomohiko Sato <tomohiko.sato@sonymobile.com>>
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

#define pr_fmt(fmt) "IDTP: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/device.h>

/* IDT P9028 Registers */
#define IDTP9028_REG0			0x40
#define IDTP9028_REG1			0x41
#define IDTP9028_REG2			0x42
#define IDTP9028_REG3			0x43
#define IDTP9028_REG4			0x44
#define IDTP9028_REG5			0x45
#define IDTP9028_REG7			0x47
#define IDTP9028_REG8			0x48
#define IDTP9028_REG30			0x5E

#define IDTP9028_RXID_OFFSET		IDTP9028_REG30
#define IDTP9028_RXID_SIZE		6

enum {
	ATTR_VRECT_MV = 0,
	ATTR_IOUT_MA,
	ATTR_FREQ_HZ,
	ATTR_UVLO,
	ATTR_CLAMP_ON,
	ATTR_LDO_CL,
	ATTR_TS_DETECT,
	ATTR_CHARGE_COMPLETE,
	ATTR_TS_LEVEL,
	ATTR_DIE_TEMP,
	ATTR_TX_TYPE,
	ATTR_RXID,
};

struct idtp9028 {
	struct device *dev;
	struct i2c_client *i2c_handle;
	int wpc_en1;
	int wpc_en2;
	struct delayed_work detect_work;
	struct power_supply wireless;
	int is_dc_charging;
	int vrect_mv;
	int iout_ma;
	int freq_hz;
	u8 uvlo;
	u8 clamp_on;
	u8 ldo_cl;
	u8 ts_detect;
	u8 charge_complete;
	u8 ts_level;
	u8 die_temp;
	u8 tx_type;
	u8 rxid[6];
	bool is_rxid_registerd;
};

#define IDTP9028_ADC_TO_MV(adc)	((adc * 1800 * 5) >> 12)
#define IDTP9028_ADC_TO_MA(adc)	((adc * 1800) >> 12)

#define IDTP9028_DETECTION_1ST_DELAY_MS		100
#define IDTP9028_DETECTION_INTERVAL_MS		0
#define IDTP9028_DETECTION_INTERVAL_MAX_MS	30000

static int idt9028_detection_interval = IDTP9028_DETECTION_INTERVAL_MS;
module_param_named(
	polling_interval, idt9028_detection_interval,
	int, S_IRUSR | S_IWUSR
);

static ssize_t idt9028_param_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

static struct device_attribute wireless_attrs[] = {
	__ATTR(vrect_mv,	S_IRUGO, idt9028_param_show, NULL),
	__ATTR(iout_ma,		S_IRUGO, idt9028_param_show, NULL),
	__ATTR(freq_hz,		S_IRUGO, idt9028_param_show, NULL),
	__ATTR(uvlo,		S_IRUGO, idt9028_param_show, NULL),
	__ATTR(clamp_on,	S_IRUGO, idt9028_param_show, NULL),
	__ATTR(ldo_cl,		S_IRUGO, idt9028_param_show, NULL),
	__ATTR(ts_detect,	S_IRUGO, idt9028_param_show, NULL),
	__ATTR(charge_complete,	S_IRUGO, idt9028_param_show, NULL),
	__ATTR(ts_level,	S_IRUGO, idt9028_param_show, NULL),
	__ATTR(die_temp,	S_IRUGO, idt9028_param_show, NULL),
	__ATTR(tx_type,		S_IRUGO, idt9028_param_show, NULL),
	__ATTR(rxid,		S_IRUGO, idt9028_param_show, NULL),
};

static enum power_supply_property wireless_props[] = {
	POWER_SUPPLY_PROP_WIRELESS_DET,
};

static ssize_t idt9028_param_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t size = 0;
	const ptrdiff_t off = attr - wireless_attrs;
	struct idtp9028 *chip = dev_get_drvdata(dev);

	switch (off) {
	case ATTR_VRECT_MV:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->vrect_mv);
		break;

	case ATTR_IOUT_MA:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->iout_ma);
		break;

	case ATTR_FREQ_HZ:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->freq_hz);
		break;

	case ATTR_UVLO:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->uvlo);
		break;

	case ATTR_CLAMP_ON:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->clamp_on);
		break;

	case ATTR_LDO_CL:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->ldo_cl);
		break;

	case ATTR_TS_DETECT:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->ts_detect);
		break;

	case ATTR_CHARGE_COMPLETE:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->charge_complete);
		break;

	case ATTR_TS_LEVEL:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->ts_level);
		break;

	case ATTR_DIE_TEMP:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->die_temp);
		break;

	case ATTR_TX_TYPE:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->tx_type);
		break;

	case ATTR_RXID:
		size = scnprintf(buf, PAGE_SIZE, "%02x%02x%02x%02x%02x%02x\n",
				chip->rxid[0], chip->rxid[1], chip->rxid[2],
				chip->rxid[3], chip->rxid[4], chip->rxid[5]);
		break;

	default:
		size = 0;
		break;
	}

	return size;
}

static int idtp9028_parse_dt(struct device *dev, struct device_node *node)
{
	struct idtp9028 *chip = dev_get_drvdata(dev);

	if (!node) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	chip->wpc_en1 = of_get_named_gpio(node, "wpc_en1", 0);
	if (chip->wpc_en1 < 0) {
		dev_err(dev, "Can't get wpc_en1\n");
		return -EINVAL;
	}

	chip->wpc_en2 = of_get_named_gpio(node, "wpc_en2", 0);
	if (chip->wpc_en2 < 0) {
		dev_err(dev, "Can't get wpc_en2\n");
		return -EINVAL;
	}

	return 0;
}

static void idtp9028_control_enable(struct idtp9028 *chip, bool enable)
{
	if (!chip->wpc_en1)
		return;

	if (enable)
		gpio_direction_output(chip->wpc_en1, 1);
	else
		gpio_direction_output(chip->wpc_en1, 0);

	return;
}

static void idtp9028_control_chg_end(struct idtp9028 *chip, bool enable)
{
	if (!chip->wpc_en2)
		return;

	if (enable)
		gpio_direction_output(chip->wpc_en2, 1);
	else
		gpio_direction_output(chip->wpc_en2, 0);

	return;
}

static void idtp9028_detect_work(struct work_struct *work)
{
	int ret;
	struct idtp9028 *chip = container_of(work,
				struct idtp9028,
				detect_work.work);

	u8 i;
	u8 reg;
	u8 reg_buf[2];
	int freq_cnt, vrect_adc, iout_adc;

	if (!chip->is_dc_charging)
		return;

	/* read RXID (one time) */
	if (!chip->is_rxid_registerd) {
		for (i = 0; i < IDTP9028_RXID_SIZE; i++) {
			reg = IDTP9028_RXID_OFFSET +  i;
			ret = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
			if (ret < 0) {
				dev_err(chip->dev,
				"failed to read i2c addr=0x%02x\n", reg);
				goto err;
			} else {
				chip->rxid[i] = ret;
			}
		}
		chip->is_rxid_registerd = true;
	}

	/* read RX STATUS (during charging) */
	reg = IDTP9028_REG0;
	reg_buf[0] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[0] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}

	reg = IDTP9028_REG1;
	reg_buf[1] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[1] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}
	vrect_adc = (reg_buf[0] << 4) | (reg_buf[1] >> 4);
	chip->vrect_mv = IDTP9028_ADC_TO_MV(vrect_adc);

	reg = IDTP9028_REG2;
	reg_buf[0] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[0] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}

	reg = IDTP9028_REG3;
	reg_buf[1] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[1] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}
	iout_adc = ((reg_buf[0] << 4) | (reg_buf[1] >> 4));
	chip->iout_ma = IDTP9028_ADC_TO_MA(iout_adc);

	reg = IDTP9028_REG4;
	reg_buf[0] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[0] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}

	reg = IDTP9028_REG5;
	reg_buf[1] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[1] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}
	freq_cnt = (reg_buf[0] << 2) | (reg_buf[1] >> 6);
	if (freq_cnt)
		chip->freq_hz = 65536 / freq_cnt;

	reg = IDTP9028_REG7;
	reg_buf[0] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[0] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}
	chip->uvlo = (reg_buf[0] & 0x08) >> 3;
	chip->clamp_on = (reg_buf[0] & 0x04) >> 2;
	chip->ldo_cl = (reg_buf[0] & 0x02) >> 1;
	chip->ts_detect = reg_buf[0] & 0x01;

	reg = IDTP9028_REG8;
	reg_buf[0] = i2c_smbus_read_byte_data(chip->i2c_handle, reg);
	if (reg_buf[0] < 0) {
		dev_err(chip->dev, "failed to read i2c addr=0x%02x\n", reg);
		goto err;
	}
	chip->charge_complete = (reg_buf[0] & 0x80) >> 7;
	switch ((reg_buf[0] & 0x7C) >> 2) {
	case 0x10:
		chip->ts_level = 1;
		break;
	case 0x08:
		chip->ts_level = 2;
		break;
	case 0x04:
		chip->ts_level = 3;
		break;
	case 0x02:
		chip->ts_level = 4;
		break;
	case 0x01:
		chip->ts_level = 5;
		break;
	default:
		chip->ts_level = 0;
		break;
	}
	chip->die_temp = (reg_buf[0] & 0x02) >> 1;
	chip->tx_type = reg_buf[0] & 0x01;

	pr_info("Vrect=%4dmV Iout=%4dmA Freq=%4dHz CHG_COMPLETE=%1d, TX_TYPE=%1d\n",
					chip->vrect_mv,
					chip->iout_ma,
					chip->freq_hz,
					chip->charge_complete,
					chip->tx_type);
err:
	if (idt9028_detection_interval > 0 &&
		idt9028_detection_interval < IDTP9028_DETECTION_INTERVAL_MAX_MS)
		schedule_delayed_work(&chip->detect_work,
				msecs_to_jiffies(idt9028_detection_interval));
}

static int idtp9028_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_WIRELESS_DET:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int idtp9028_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct idtp9028 *chip = dev_get_drvdata(psy->dev->parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_WIRELESS_DET:
		if (val->intval) {
			pr_info("wireless charging is detected.\n");
			chip->is_dc_charging = 1;
			schedule_delayed_work(&chip->detect_work,
			msecs_to_jiffies(IDTP9028_DETECTION_1ST_DELAY_MS));
		} else {
			chip->is_dc_charging = 0;
			cancel_delayed_work_sync(&chip->detect_work);
			pr_info("wireless charging is stopped.\n");
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int idtp9028_property_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_WIRELESS_DET:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}

	return rc;
}

static int idtp9028_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct idtp9028 *chip;
	struct device_node *node = cl->dev.of_node;
	int rc;
	int i;

	chip = devm_kzalloc(&cl->dev, sizeof(struct idtp9028), GFP_KERNEL);
	if (!chip) {
		rc = -ENOMEM;
		goto err_kzalloc;
	}

	/* device name resistration */
	dev_set_name(&cl->dev, "wireless-chg");

	/* i2c resistration */
	chip->i2c_handle = cl;
	chip->dev = &cl->dev;
	i2c_set_clientdata(cl, chip);

	/* get device tree */
	rc = idtp9028_parse_dt(&cl->dev, node);
	if (rc < 0)
		goto err_parse_dt;

	/* GPIO initialize */
	if (gpio_is_valid(chip->wpc_en1))
		gpio_request_one(chip->wpc_en1, GPIOF_OUT_INIT_LOW,
						"gpio_wpc_en1");

	if (gpio_is_valid(chip->wpc_en2))
		gpio_request_one(chip->wpc_en2, GPIOF_OUT_INIT_LOW,
						"gpio_wpc_en2");
	idtp9028_control_enable(chip, 0);
	idtp9028_control_chg_end(chip, 0);

	/* sysfs resistration */
	for (i = 0; i < ARRAY_SIZE(wireless_attrs); i++) {
		rc = device_create_file(&cl->dev, &wireless_attrs[i]);
		if (rc < 0) {
			dev_err(chip->dev, "can't create sysfs (%d)\n", rc);
			goto err_sysfs;
		}
	}

	/* power supply resistration */
	memset(&chip->wireless, 0x00, sizeof(struct power_supply));
	chip->wireless.properties = wireless_props;
	chip->wireless.num_properties = ARRAY_SIZE(wireless_props);
	chip->wireless.get_property = idtp9028_get_property;
	chip->wireless.set_property = idtp9028_set_property;
	chip->wireless.name = "wireless";
	chip->wireless.type = POWER_SUPPLY_TYPE_UNKNOWN;
	chip->wireless.property_is_writeable = idtp9028_property_is_writeable;
	rc = power_supply_register(&cl->dev, &chip->wireless);
	if (rc < 0) {
		dev_err(chip->dev, "can't regist power_supply (%d)\n", rc);
		goto err_sysfs;
	}

	/* worker resistration */
	INIT_DELAYED_WORK(&chip->detect_work, idtp9028_detect_work);

	return 0;

err_sysfs:
err_parse_dt:
	i2c_set_clientdata(cl, NULL);
	kzfree(chip);
err_kzalloc:
	return rc;
}

static int idtp9028_remove(struct i2c_client *cl)
{
	struct idtp9028 *chip = i2c_get_clientdata(cl);
	int i;

	cancel_delayed_work_sync(&chip->detect_work);

	power_supply_unregister(&chip->wireless);

	/* sysfs unresistration */
	for (i = 0; i < ARRAY_SIZE(wireless_attrs); i++)
		device_remove_file(&cl->dev, &wireless_attrs[i]);

	i2c_set_clientdata(cl, NULL);

	return 0;
}

static const struct of_device_id idtp9028_dt_ids[] = {
	{ .compatible = "idtp,idtp9028", },
	{ }
};
MODULE_DEVICE_TABLE(of, idtp9028_dt_ids);

static const struct i2c_device_id idtp9028_ids[] = {
	{"idtp9028a", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, idtp9028_ids);

static struct i2c_driver idtp9028_driver = {
	.driver = {
		   .name = "idtp9028",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(idtp9028_dt_ids),
		   },
	.probe = idtp9028_probe,
	.remove = idtp9028_remove,
	.id_table = idtp9028_ids,
};

module_i2c_driver(idtp9028_driver);
