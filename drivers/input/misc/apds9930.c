/* drivers/input/misc/apds9930.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonymobile.com>
 *         Takashi Shiina <Takashi.Shiina@sonymobile.com>
 *         Masashi Shimizu <Masashi.X.Shimizu@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unistd.h>
#include "apds9930.h"

#define APDS_IDDIO_HCURRENT	3000
#define APDS_IDDIO_LCURRENT	4

#define APDS9930_THRESH_MIN	0x0000
#define APDS9930_THRESH_MAX	0xFFFF
#define APDS9930_PERS_MAX       0xFF
#define APDS9930_PERS_MIN       0x00
#define APDS9930_PPULSE_MAX     0xFF
#define APDS9930_PPULSE_MIN     0x00

#define APDS9930_P_INIT_MS	3
#define APDS9930_A_INIT_MS	3

#define APDS9930_VAL_ATIME	238
#define APDS9930_VAL_PTIME	255
#define APDS9930_VAL_WTIME	0

#define APDS9930_VAL_HI_THRES	180
#define APDS9930_VAL_LO_THRES	150
#define APDS9930_VAL_PPULSE	2

#define APDS9930_NONE_VAL	0

enum apds9930_regs {
	APDS9930_ENABLE,
	APDS9930_ALS_TIME,
	APDS9930_PRX_TIME,
	APDS9930_WAIT_TIME,
	APDS9930_ALS_MINTHRESHLO,
	APDS9930_ALS_MINTHRESHHI,
	APDS9930_ALS_MAXTHRESHLO,
	APDS9930_ALS_MAXTHRESHHI,
	APDS9930_PRX_MINTHRESHLO,
	APDS9930_PRX_MINTHRESHHI,
	APDS9930_PRX_MAXTHRESHLO,
	APDS9930_PRX_MAXTHRESHHI,
	APDS9930_PERSISTENCE,
	APDS9930_CONFIG,
	APDS9930_PRX_PULSE_COUNT,
	APDS9930_CONTROL,

	APDS9930_REVID = 0x11,
	APDS9930_CHIPID,
	APDS9930_STATUS,
	APDS9930_ALS_CHAN0LO,
	APDS9930_ALS_CHAN0HI,
	APDS9930_ALS_CHAN1LO,
	APDS9930_ALS_CHAN1HI,
	APDS9930_PRX_LO,
	APDS9930_PRX_HI,

	APDS9930_REG_PRX_OFFS = 0x1e,
	APDS9930_REG_MAX,
};

enum apds9930_cmd_reg {
	APDS9930_CMD_REG           = (1 << 7),
	APDS9930_CMD_INCR          = (0x1 << 5),
	APDS9930_CMD_SPL_FN        = (0x3 << 5),
	APDS9930_CMD_PROX_INT_CLR  = (0x5 << 0),
	APDS9930_CMD_ALS_INT_CLR   = (0x6 << 0),
};

enum apds9930_en_reg {
	APDS9930_EN_PWR_ON   = (1 << 0),
	APDS9930_EN_ALS      = (1 << 1),
	APDS9930_EN_PRX      = (1 << 2),
	APDS9930_EN_WAIT     = (1 << 3),
	APDS9930_EN_ALS_IRQ  = (1 << 4),
	APDS9930_EN_PRX_IRQ  = (1 << 5),
	APDS9930_EN_SAI      = (1 << 6),
};

enum apds9930_status {
	APDS9930_ST_ALS_VALID  = (1 << 0),
	APDS9930_ST_PRX_VALID  = (1 << 1),
	APDS9930_ST_ALS_IRQ    = (1 << 4),
	APDS9930_ST_PRX_IRQ    = (1 << 5),
	APDS9930_ST_PRX_SAT    = (1 << 6),
};

enum apds9930_bit_op {
	APDS9930_ALS_GAIN_MASK = (3 << 0),
	APDS9930_ALS_AGL_MASK  = (1 << 2),
	APDS9930_ALS_AGL_SHIFT = 2,
	APDS9930_ATIME_PER_100 = 273,
	APDS9930_ATIME_DEFAULT_MS = 50,
	APDS9930_SCALE_SHIFT = 11,
	APDS9930_RATIO_SHIFT = 10,
	APDS9930_MAX_ALS_VALUE = 0xffff,
	APDS9930_MIN_ALS_VALUE = 10,
	APDS9930_GAIN_SWITCH_LEVEL = 100,
	APDS9930_GAIN_AUTO_INIT_VALUE = 16,
};

static u8 const apds9930_ids[] = {
	0x39,
};

static char const *apds9930_names[] = {
	"apds9930",
};

static u8 const apds9930_restorable_regs[] = {
	APDS9930_ALS_TIME,
	APDS9930_PRX_TIME,
	APDS9930_WAIT_TIME,
	APDS9930_PERSISTENCE,
	APDS9930_CONFIG,
	APDS9930_PRX_PULSE_COUNT,
	APDS9930_CONTROL,
	APDS9930_REG_PRX_OFFS,
};

static u8 const apds9930_als_gains[] = {
	1,
	8,
	16,
	120
};

struct apds9930_prox_info {
	int raw;
	int detected;
};

struct apds9930_regulator_data {
	struct regulator *reg_vdd;
	struct regulator *reg_vio;
};

struct apds9930_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct apds9930_prox_info prx_inf;
	struct apds9930_parameters params;
	u8 shadow[APDS9930_REG_MAX];
	struct input_dev *p_idev;
	struct input_dev *a_idev;
	struct apds9930_regulator_data regudata;
	int in_suspend;
	int wake_irq;
	int irq_pending;
	bool unpowered;
	bool als_enabled;
	bool prx_enabled;
};

static int apds9930_power_init(struct device *dev)
{
	int rc;
	struct apds9930_chip *chip = dev_get_drvdata(dev);

	if (!(chip->regudata.reg_vdd))
		chip->regudata.reg_vdd = devm_regulator_get(dev,
							"apds9930_vdd");
	if (IS_ERR_OR_NULL(chip->regudata.reg_vdd)) {
		rc = PTR_ERR(chip->regudata.reg_vdd);
		dev_err(dev, "%s: regulator_get failed on %s. rc=%d\n",
						__func__, "apds9930_vdd", rc);
		rc = rc ? rc : -ENODEV;
		goto err_vdd;
	} else {
		rc = regulator_set_voltage(chip->regudata.reg_vdd,
						2850000, 2850000);
		if (rc)
			goto err_vdd_set;
	}

	if (!chip->regudata.reg_vio)
		chip->regudata.reg_vio = devm_regulator_get(dev,
							"apds9930_vio");
	if (IS_ERR_OR_NULL(chip->regudata.reg_vio)) {
		rc = PTR_ERR(chip->regudata.reg_vio);
		dev_err(dev, "%s: regulator_get failed on %s. rc=%d\n",
						__func__, "apds9930_vio", rc);
		rc = rc ? rc : -ENODEV;
		goto err_vio;
	}
	return rc;
err_vio:
	chip->regudata.reg_vio = NULL;
err_vdd_set:
	regulator_put(chip->regudata.reg_vio);
err_vdd:
	chip->regudata.reg_vio = NULL;
	return rc;
}

static int apds9930_irq_init(struct device *dev, int request)
{
	int rc;
	static int irq_gpio;
	u32 irq_gpio_flags;

	irq_gpio = of_get_named_gpio_flags(dev->of_node,
				"apds9930,irq_gpio",
				0, &irq_gpio_flags);
	if (irq_gpio < 0) {
		dev_err(dev, "%s: failed to get irq_gpio %d\n",
					__func__, irq_gpio);
		rc = irq_gpio;
		goto err_irq_gpio;
	}
	if (request) {
		rc = gpio_request(irq_gpio, "apds9930");
		if (rc) {
			dev_err(dev, "%s: failed to request gpio %d\n",
					__func__, irq_gpio);
		}
	} else {
		rc = 0;
		gpio_free(irq_gpio);
	}
err_irq_gpio:
	return rc;
}

static int apds9930_power_cotrol(struct device *dev, int enable)
{
	int rc;
	struct apds9930_chip *chip = dev_get_drvdata(dev);

	if (enable) {
		rc = regulator_enable(chip->regudata.reg_vdd);
		if (rc) {
			dev_err(dev,
				"%s: regulator_enable failed on vdd. rc=%d\n",
				__func__, rc);
			goto err_pwr_ctrl;
		}
		rc = regulator_enable(chip->regudata.reg_vio);
		if (rc) {
			dev_err(dev,
				"%s: regulator_enable failed on vio. rc=%d\n",
				__func__, rc);
			goto err_pwr_ctrl;
		}
		usleep_range(10000, 11000);
	} else {
		rc = regulator_disable(chip->regudata.reg_vio);
		if (rc) {
			dev_err(dev,
				"%s: regulator_disable failed on vio. rc=%d\n",
				__func__, rc);
			goto err_pwr_ctrl;
		}
		rc = regulator_disable(chip->regudata.reg_vdd);
		if (rc) {
			dev_err(dev,
				"%s: regulator_disable failed on vdd. rc=%d\n",
				__func__, rc);
			goto err_pwr_ctrl;
		}
	}
err_pwr_ctrl:
	return rc;
}

static int apds9930_i2c_read(struct apds9930_chip *chip, u8 reg, u8 *val)
{
	int ret;
	s32 read;
	struct i2c_client *client = chip->client;

	ret = i2c_smbus_write_byte(client, (APDS9930_CMD_REG | reg));
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
		return ret;
	}
	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		dev_err(&client->dev, "%s: failed to read from register %x\n",
				__func__, reg);
		return ret;
	}
	*val = read;
	return 0;
}

static int apds9930_i2c_blk_read(struct apds9930_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	ret =  i2c_smbus_read_i2c_block_data(client,
			APDS9930_CMD_REG | APDS9930_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}

static int apds9930_i2c_write(struct apds9930_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	ret = i2c_smbus_write_byte_data(client, APDS9930_CMD_REG | reg, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
	return ret;
}

static int apds9930_i2c_blk_write(struct apds9930_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	ret =  i2c_smbus_write_i2c_block_data(client,
			APDS9930_CMD_REG | APDS9930_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}

static int apds9930_irq_clr(struct apds9930_chip *chip, u8 bits)
{
	int ret = i2c_smbus_write_byte(chip->client, APDS9930_CMD_REG |
			APDS9930_CMD_SPL_FN | bits);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: failed, bits %x\n",
				__func__, bits);
	return ret;
}

static void apds9930_get_prox(struct apds9930_chip *chip)
{
	u8 *buf = &chip->shadow[APDS9930_PRX_LO];
	bool d = chip->prx_inf.detected;

	chip->prx_inf.raw = (buf[1] << 8) | buf[0];
	chip->prx_inf.detected =
			(d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
			(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
	dev_dbg(&chip->client->dev, "%s: raw %d, detected %d\n", __func__,
			chip->prx_inf.raw, chip->prx_inf.detected);
}

static int apds9930_read_all(struct apds9930_chip *chip)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = apds9930_i2c_blk_read(chip, APDS9930_STATUS,
			&chip->shadow[APDS9930_STATUS],
			APDS9930_PRX_HI - APDS9930_STATUS + 1);
	return (ret < 0) ? ret : 0;
}


static int update_prox_thresh(struct apds9930_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[APDS9930_PRX_MINTHRESHLO];
	u16 from, to;

	if (on_enable) {
		/* zero gate to force irq */
		from = to = 0;
	} else {
		if (chip->prx_inf.detected) {
			from = chip->params.prox_th_min;
			to = 0xffff;
		} else {
			from = 0;
			to = chip->params.prox_th_max;
		}
	}
	dev_dbg(&chip->client->dev, "%s: %u - %u\n", __func__, from, to);
	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = apds9930_i2c_blk_write(chip, APDS9930_PRX_MINTHRESHLO,
		&chip->shadow[APDS9930_PRX_MINTHRESHLO],
		APDS9930_PRX_MAXTHRESHHI - APDS9930_PRX_MINTHRESHLO + 1);
	return (ret < 0) ? ret : 0;
}

static void report_prox(struct apds9930_chip *chip)
{
	if (chip->p_idev) {
		input_report_abs(chip->p_idev, ABS_DISTANCE,
				chip->prx_inf.detected ? 0 : 1);
		input_sync(chip->p_idev);
	}
}

static int apds9930_check_and_report(struct apds9930_chip *chip)
{
	u8 status;
	int ret = apds9930_read_all(chip);
	if (ret)
		goto exit_clr;

	status = chip->shadow[APDS9930_STATUS];
	dev_dbg(&chip->client->dev, "%s: status 0x%02x\n", __func__, status);

	if ((status & (APDS9930_ST_PRX_VALID | APDS9930_ST_PRX_IRQ)) ==
			(APDS9930_ST_PRX_VALID | APDS9930_ST_PRX_IRQ)) {
		apds9930_get_prox(chip);
		report_prox(chip);
		ret = update_prox_thresh(chip, 0);
		if (ret)
			dev_err(&chip->client->dev,
				"%s: prox threshold update fail\n", __func__);
	}
exit_clr:
	(void)apds9930_irq_clr(chip,
		APDS9930_CMD_PROX_INT_CLR | APDS9930_CMD_ALS_INT_CLR);
	return ret;
}

static irqreturn_t apds9930_irq(int irq, void *handle)
{
	struct apds9930_chip *chip = handle;
	struct device *dev = &chip->client->dev;

	mutex_lock(&chip->lock);
	if (chip->in_suspend) {
		dev_dbg(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		disable_irq_nosync(chip->client->irq);
		goto bypass;
	}
	dev_dbg(dev, "%s\n", __func__);
	(void)apds9930_check_and_report(chip);
bypass:
	mutex_unlock(&chip->lock);
	return IRQ_HANDLED;
}

static void set_pltf_settings(struct apds9930_chip *chip)
{
	int ret;
	u32 pulse_cnt, th_max, th_min;
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	dev_dbg(dev, "%s: use defaults\n", __func__);
	sh[APDS9930_ALS_TIME] = APDS9930_VAL_ATIME; /* ~50 ms */
	sh[APDS9930_PRX_TIME] = APDS9930_VAL_PTIME;
	sh[APDS9930_WAIT_TIME] = APDS9930_VAL_WTIME;
	sh[APDS9930_PERSISTENCE] = PRX_PERSIST(2) | ALS_PERSIST(2);
	sh[APDS9930_CONFIG] = APDS9930_NONE_VAL;
	sh[APDS9930_CONTROL] = AGAIN_1 | PGAIN_1 |
				PDIOD_CH1 | PDRIVE_100MA;
	sh[APDS9930_REG_PRX_OFFS] = APDS9930_NONE_VAL;

	ret = of_property_read_u32(dev->of_node, "ppulse_cnt", &pulse_cnt);
	if (ret < 0)
		dev_err(dev, "%s: ppulse_cnt get from device tree fail\n",
			__func__);
	sh[APDS9930_PRX_PULSE_COUNT] =
			(ret < 0) ? APDS9930_VAL_PPULSE : pulse_cnt;

	ret = of_property_read_u32(dev->of_node, "prox_th_max", &th_max);
	if (ret < 0)
		dev_err(dev, "%s: prox_th_max get from device tree fail\n",
			__func__);
	chip->params.prox_th_max =
			(ret < 0) ? APDS9930_VAL_HI_THRES : (u16)th_max;

	ret = of_property_read_u32(dev->of_node, "prox_th_min", &th_min);
	if (ret < 0)
		dev_err(dev, "%s: prox_th_min get from device tree fail\n",
			__func__);
	chip->params.prox_th_min =
			(ret < 0) ? APDS9930_VAL_LO_THRES : (u16)th_min;
}

static int flush_regs(struct apds9930_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(apds9930_restorable_regs); i++) {
		reg = apds9930_restorable_regs[i];
		rc = apds9930_i2c_write(chip, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}
	return rc;
}

static int update_enable_reg(struct apds9930_chip *chip)
{
	dev_dbg(&chip->client->dev, "%s: %02x\n", __func__,
			chip->shadow[APDS9930_ENABLE]);
	return apds9930_i2c_write(chip, APDS9930_ENABLE,
			chip->shadow[APDS9930_ENABLE]);
}

static int apds9930_prox_enable(struct apds9930_chip *chip, int on)
{
	int rc;

	dev_dbg(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		(void)apds9930_irq_clr(chip, APDS9930_CMD_PROX_INT_CLR);
		rc = update_prox_thresh(chip, 1);
		if (rc)
			dev_err(&chip->client->dev,
				"%s: prox threshold update fail\n", __func__);
		chip->shadow[APDS9930_ENABLE] |=
				(APDS9930_EN_PWR_ON | APDS9930_EN_PRX |
				APDS9930_EN_PRX_IRQ);
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		msleep(APDS9930_P_INIT_MS);
	} else {
		chip->shadow[APDS9930_ENABLE] &=
				~(APDS9930_EN_PRX_IRQ | APDS9930_EN_PRX);
		if (!(chip->shadow[APDS9930_ENABLE] & APDS9930_EN_ALS))
			chip->shadow[APDS9930_ENABLE] &= ~APDS9930_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		(void)apds9930_irq_clr(chip, APDS9930_CMD_PROX_INT_CLR);
	}
	if (!rc)
		chip->prx_enabled = on;
	return rc;
}

static int apds9930_als_enable(struct apds9930_chip *chip, int on)
{
	int rc;

	dev_dbg(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		chip->shadow[APDS9930_ENABLE] |=
				(APDS9930_EN_PWR_ON | APDS9930_EN_ALS);
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		msleep(APDS9930_A_INIT_MS);
	} else {
		chip->shadow[APDS9930_ENABLE] &= ~APDS9930_EN_ALS;
		if (!(chip->shadow[APDS9930_ENABLE] & APDS9930_EN_PRX))
			chip->shadow[APDS9930_ENABLE] &= ~APDS9930_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
	}
	if (!rc)
		chip->als_enabled = on;
	return rc;
}

static ssize_t apds9930_device_als_adc_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct apds9930_chip *chip = dev_get_drvdata(dev);
	u32 ch0, ch1;
	u8 *shadow_buf = &chip->shadow[APDS9930_ALS_CHAN0LO];

	(void)apds9930_read_all(chip);
	ch0 = le16_to_cpup((const __le16 *)&shadow_buf[0]);
	ch1 = le16_to_cpup((const __le16 *)&shadow_buf[2]);
	return snprintf(buf, PAGE_SIZE, "ch0:0x%04x,ch1:0x%04x\n", ch0, ch1);
}

static ssize_t attr_threshold_lo_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	int err = 0;
	long th;
	struct apds9930_chip *chip = dev_get_drvdata(dev);

	err = kstrtoul(buf, 0, &th);
	if (!err && th >= APDS9930_THRESH_MIN) {
		mutex_lock(&chip->lock);
		chip->params.prox_th_min = th;
		dev_dbg(dev, "%s threshold is %ld\n", __func__, th);
		if (chip->prx_enabled) {
			err = update_prox_thresh(chip, 1);
			if (err < 0)
				dev_err(dev, "%s: I2C write error = %d\n",
				__func__, err);
		}
		mutex_unlock(&chip->lock);
		ret = (err == 0) ? size : err;
	}
	return ret;
}

static ssize_t attr_threshold_hi_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	int err = 0;
	long th;
	struct apds9930_chip *chip = dev_get_drvdata(dev);

	err = kstrtoul(buf, 0, &th);
	if (!err && th <= APDS9930_THRESH_MAX) {
		mutex_lock(&chip->lock);
		chip->params.prox_th_max = th;
		dev_dbg(dev, "%s threshold is %ld\n", __func__, th);
		if (chip->prx_enabled) {
			err = update_prox_thresh(chip, 1);
			if (err < 0)
				dev_err(dev, "%s: I2C write error = %d\n",
				__func__, err);
		}
		mutex_unlock(&chip->lock);
		ret = (err == 0) ? size : err;
	}
	return ret;
}

static ssize_t attr_ppulse_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	int err = 0;
	long pp;
	struct apds9930_chip *chip = dev_get_drvdata(dev);

	err = kstrtoul(buf, 10, &pp);
	if (!err && pp <= APDS9930_PPULSE_MAX && pp >= APDS9930_PPULSE_MIN) {
		mutex_lock(&chip->lock);
		chip->shadow[APDS9930_PRX_PULSE_COUNT] = (unsigned char)pp;
		dev_dbg(dev, "%s ppulse is %ld\n", __func__, pp);
		if (chip->prx_enabled) {
			err = apds9930_i2c_write(chip, APDS9930_PRX_PULSE_COUNT,
					chip->shadow[APDS9930_PRX_PULSE_COUNT]);
			if (err)
				dev_err(dev, "%s: I2C write error = %d\n",
								__func__, err);
		}
		mutex_unlock(&chip->lock);
		ret = (err == 0) ? size : err;
	}
	return ret;
}

static struct device_attribute prox_attrs[] = {
	__ATTR(prx_hith, S_IWUSR, NULL, attr_threshold_hi_set),
	__ATTR(prx_loth, S_IWUSR, NULL, attr_threshold_lo_set),
	__ATTR(prx_ppulse, S_IWUSR, NULL, attr_ppulse_set),
};

static struct device_attribute als_attrs[] = {
	__ATTR(als_adc_data, S_IRUGO, apds9930_device_als_adc_data, NULL),
};

static int add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int apds9930_get_id(struct apds9930_chip *chip, u8 *id, u8 *rev)
{
	int rc = apds9930_i2c_read(chip, APDS9930_REVID, rev);
	if (rc)
		return rc;
	return apds9930_i2c_read(chip, APDS9930_CHIPID, id);
}

static int power_on(struct apds9930_chip *chip)
{
	int rc = 0;

	rc = apds9930_power_cotrol(&chip->client->dev, 1);
	chip->unpowered = rc != 0;
	if (rc)
		return rc;
	dev_dbg(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return flush_regs(chip);
}

static void power_off(struct apds9930_chip *chip)
{
	int rc = 0;
	if (!(chip->prx_enabled || chip->als_enabled)) {
		rc = apds9930_power_cotrol(&chip->client->dev, 0);
		chip->unpowered = rc == 0;
	}
}

static int apds9930_prox_idev_open(struct input_dev *idev)
{
	struct apds9930_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;
	bool als = chip->a_idev && chip->a_idev->users;

	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		rc = power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = apds9930_prox_enable(chip, 1);
	if (rc && !als)
		power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return rc;
}

static void apds9930_prox_idev_close(struct input_dev *idev)
{
	struct apds9930_chip *chip = dev_get_drvdata(&idev->dev);

	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	(void)apds9930_prox_enable(chip, 0);
	if (!chip->a_idev || !chip->a_idev->users)
		power_off(chip);
	mutex_unlock(&chip->lock);
}

static int apds9930_als_idev_open(struct input_dev *idev)
{
	struct apds9930_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;
	bool prox = chip->p_idev && chip->p_idev->users;

	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		rc = power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = apds9930_als_enable(chip, 1);
	if (rc && !prox)
		power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return rc;
}

static void apds9930_als_idev_close(struct input_dev *idev)
{
	struct apds9930_chip *chip = dev_get_drvdata(&idev->dev);

	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	(void)apds9930_als_enable(chip, 0);
	if (!chip->p_idev || !chip->p_idev->users)
		power_off(chip);
	mutex_unlock(&chip->lock);
}

static int __devinit apds9930_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct apds9930_chip *chip;
	char const *prox_name = NULL;
	char const *als_name = NULL;

	dev_info(dev, "%s\n", __func__);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev,
			"%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	ret = of_property_read_string(dev->of_node, "prox_name", &prox_name);
	if (ret < 0)
		dev_err(dev, "%s: prox_name get from device tree fail\n",
			__func__);
	ret = of_property_read_string(dev->of_node, "als_name", &als_name);
	if (ret < 0)
		dev_err(dev, "%s: als_name get from device tree fail\n",
			__func__);
	if (!(prox_name || als_name) || client->irq < 0) {
		dev_err(dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	ret = apds9930_irq_init(dev, 1);
	if (ret)
		goto init_failed;

	chip = kzalloc(sizeof(struct apds9930_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	chip->client = client;
	i2c_set_clientdata(client, chip);

	ret = apds9930_power_init(dev);

	ret = apds9930_power_cotrol(dev, 1);
	if (ret) {
		dev_err(dev, "%s: power on failed\n", __func__);
		goto pon_failed;
	}
	chip->unpowered = false;

	ret = apds9930_get_id(chip, &id, &rev);
	if (ret)
		goto id_failed;
	for (i = 0; i < ARRAY_SIZE(apds9930_ids); i++) {
		if (id == apds9930_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(apds9930_names)) {
		dev_info(dev, "%s: '%s rev. %d' detected\n", __func__,
			apds9930_names[i], rev);
	} else {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}
	mutex_init(&chip->lock);
	set_pltf_settings(chip);
	ret = flush_regs(chip);
	if (ret)
		goto flush_regs_failed;

	(void)apds9930_power_cotrol(dev, 0);
	chip->unpowered = true;

	if (!prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
	chip->p_idev->open = apds9930_prox_idev_open;
	chip->p_idev->close = apds9930_prox_idev_close;
	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, prox_name);
		goto input_p_alloc_failed;
	}
	ret = add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;
bypass_prox_idev:
	if (!als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->a_idev->open = apds9930_als_idev_open;
	chip->a_idev->close = apds9930_als_idev_close;
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, als_name);
		goto input_a_alloc_failed;
	}
	ret = add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;

bypass_als_idev:
	ret = request_threaded_irq(client->irq, NULL, apds9930_irq,
		      IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		      dev_name(dev), chip);
	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

	dev_info(dev, "Probe ok.\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
		remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
	}
input_p_alloc_failed:
flush_regs_failed:
id_failed:
pon_failed:
	i2c_set_clientdata(client, NULL);

	kfree(chip);
malloc_failed:
	if (!chip->unpowered)
		(void)apds9930_power_cotrol(dev, 0);
	(void)apds9930_irq_init(dev, 0);
init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int apds9930_suspend(struct device *dev)
{
	struct apds9930_chip *chip = dev_get_drvdata(dev);
	int rc = 0;

	dev_dbg(dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;
	if (chip->p_idev && chip->p_idev->users) {
		dev_dbg(dev, "set wake on proximity\n");
		chip->wake_irq = 1;
	}
	if (chip->a_idev && chip->a_idev->users) {
		dev_dbg(dev, "als off\n");
		rc = apds9930_als_enable(chip, 0);
		if (rc) {
			dev_err(dev, "%s: als disab;e fail\n", __func__);
			goto err_als_enable;
		}
	}
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_dbg(dev, "powering off\n");
		power_off(chip);
	}
err_als_enable:
	mutex_unlock(&chip->lock);
	return rc;
}

static int apds9930_resume(struct device *dev)
{
	struct apds9930_chip *chip = dev_get_drvdata(dev);
	bool als_on, prx_on;
	int rc = 0;

	mutex_lock(&chip->lock);
	prx_on = chip->p_idev && chip->p_idev->users;
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;
	dev_dbg(dev, "%s: powerd %d, als: needed %d  enabled %d,"
			" prox: needed %d  enabled %d\n", __func__,
			!chip->unpowered, als_on, chip->als_enabled,
			prx_on, chip->prx_enabled);
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && als_on) {
		dev_dbg(dev, "powering on\n");
		rc = power_on(chip);
		if (rc)
			goto err_power;
	}
	if (als_on && !chip->als_enabled)
		rc = apds9930_als_enable(chip, 1);
		if (rc)
			dev_err(dev, "%s: als enable fail\n", __func__);
	if (chip->irq_pending) {
		dev_dbg(dev, "%s: pending interrupt\n", __func__);
		chip->irq_pending = 0;
		(void)apds9930_check_and_report(chip);
		enable_irq(chip->client->irq);
	}
err_power:
	mutex_unlock(&chip->lock);
	return rc;
}

static int __devexit apds9930_remove(struct i2c_client *client)
{
	struct apds9930_chip *chip = i2c_get_clientdata(client);
	free_irq(client->irq, chip);
	if (chip->a_idev) {
		remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
		input_unregister_device(chip->p_idev);
	}
	(void)apds9930_power_cotrol(&client->dev, 0);
	(void)apds9930_irq_init(&client->dev, 0);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

static struct i2c_device_id apds9930_idtable[] = {
	{ "apds9930", 0 },
	{}
};

static struct of_device_id apds9930_match_table[] = {
	{ .compatible = "avago,apds9930", },
	{ }
};

MODULE_DEVICE_TABLE(i2c, apds9930_idtable);

static const struct dev_pm_ops apds9930_pm_ops = {
	.suspend = apds9930_suspend,
	.resume  = apds9930_resume,
};

static struct i2c_driver apds9930_driver = {
	.driver = {
		.name = "apds9930",
		.pm = &apds9930_pm_ops,
		.of_match_table = apds9930_match_table,
	},
	.id_table = apds9930_idtable,
	.probe = apds9930_probe,
	.remove = __devexit_p(apds9930_remove),
};

static int __init apds9930_init(void)
{
	return i2c_add_driver(&apds9930_driver);
}

static void __exit apds9930_exit(void)
{
	i2c_del_driver(&apds9930_driver);
}

module_init(apds9930_init);
module_exit(apds9930_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonymobile.com>,"
		"Takashi Shiina <Takashi.Shiina@sonymobile.com>,"
		"Masashi Shimizu <Masashi.X.Shimizu@sonymobile.com>");
MODULE_DESCRIPTION("avago apds9930 ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");
