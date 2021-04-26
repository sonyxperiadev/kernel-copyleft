/*
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are Copyright (c) 2018 Sony Mobile Communications Inc,
 * and licensed under the license of the file.
 */
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define TRILED_REG_TYPE			0x04
#define TRILED_REG_SUBTYPE		0x05
#define TRILED_REG_EN_CTL		0x46

/* TRILED_REG_EN_CTL */
#define TRILED_EN_CTL_MASK		GENMASK(7, 5)
#define TRILED_EN_CTL_MAX_BIT		7

#define TRILED_TYPE			0x19
#define TRILED_SUBTYPE_LED3H0L12	0x02
#define TRILED_SUBTYPE_LED2H0L12	0x03
#define TRILED_SUBTYPE_LED1H2L12	0x04

#define TRILED_NUM_MAX			3

#define PWM_PERIOD_DEFAULT_NS		1000000

#define RGB_MAX_LEVEL			512

#define RGB_LED_MAX_PAUSE_REG		254
#define RGB_LED_MAX_PAUSE_TIME		130048
#define RGB_LED_LUT_MAX_PAUSE		255
#define RGB_CURR_DEFAULT_PATTERN	1
#define RGB_CURR_UNIT_NUM		3
#define RGB_LED_DISABLE			0x00
#define PWM_NS				(1000 * NSEC_PER_USEC)
#define RAMP_STEP_DURATION_MAX		511
#define RGB_LED_INDEX_OFFSET		1
#define RGB_LED_INDEX_MAX_EACH_LED	8

struct pwm_setting {
	u64	pre_period_ns;
	u64	period_ns;
	u64	duty_ns;
	bool	enabled;
	bool	use_duration;
	struct ramp_config ramp_config;
};

struct led_setting {
	u64			on_ms;
	u64			off_ms;
	enum led_brightness	brightness;
	bool			blink;
	bool			breath;
	u16			single_pwm_value;
	u16			mix_pwm_value;
};

enum rgb_sync_state {
	RGBSYNC_STATE_NOT_BLINK,
	RGBSYNC_STATE_CONFIGURE_TO_BLINK,
	RGBSYNC_STATE_BLINKING,
};

struct qpnp_led_dev {
	struct led_classdev	cdev;
	struct pwm_device	*pwm_dev;
	struct pwm_setting	pwm_setting;
	struct led_setting	led_setting;
	struct qpnp_tri_led_chip	*chip;
	struct mutex		lock;
	const char		*label;
	const char		*default_trigger;
	u8			id;
	bool			blinking;
	bool			breathing;
	u16			max_current;
};

struct qpnp_tri_led_chip {
	struct device		*dev;
	struct regmap		*regmap;
	struct qpnp_led_dev	*leds;
	struct nvmem_device	*pbs_nvmem;
	struct mutex		bus_lock;
	int			num_leds;
	u16			reg_base;
	u8			subtype;
	u8			bitmap;
	struct led_classdev	rgb_cdev;
	enum rgb_sync_state	sync_state;
};

static int rgb_current_index;
static int __init rgb_current_setup(char *str)
{
	unsigned int res;

	if (str == NULL)
		return 0;
	if (!kstrtouint(str, 16, &res))
		rgb_current_index = res;
	return 1;
}
__setup("oemandroidboot.babe137e=", rgb_current_setup);

static int qpnp_tri_led_read(struct qpnp_tri_led_chip *chip, u16 addr, u8 *val)
{
	int rc;
	unsigned int tmp;

	mutex_lock(&chip->bus_lock);
	rc = regmap_read(chip->regmap, chip->reg_base + addr, &tmp);
	if (rc < 0)
		dev_err(chip->dev, "Read addr 0x%x failed, rc=%d\n", addr, rc);
	else
		*val = (u8)tmp;
	mutex_unlock(&chip->bus_lock);

	return rc;
}

static int qpnp_tri_led_masked_write(struct qpnp_tri_led_chip *chip,
				u16 addr, u8 mask, u8 val)
{
	int rc;

	mutex_lock(&chip->bus_lock);
	rc = regmap_update_bits(chip->regmap, chip->reg_base + addr, mask, val);
	if (rc < 0)
		dev_err(chip->dev, "Update addr 0x%x to val 0x%x with mask 0x%x failed, rc=%d\n",
					addr, val, mask, rc);
	mutex_unlock(&chip->bus_lock);

	return rc;
}

static int __tri_led_config_pwm(struct qpnp_led_dev *led,
				struct pwm_setting *pwm)
{
	struct pwm_state pstate;
	int rc;

	pwm_get_state(led->pwm_dev, &pstate);
	pstate.enabled = !!(pwm->duty_ns != 0);
	pstate.period = pwm->period_ns;
	pstate.duty_cycle = pwm->duty_ns;
	pstate.output_type = led->led_setting.breath ?
		PWM_OUTPUT_MODULATED : PWM_OUTPUT_FIXED;
	/* Use default pattern in PWM device */
	pstate.output_pattern = NULL;
	rc = pwm_apply_state(led->pwm_dev, &pstate);

	if (rc < 0)
		dev_err(led->chip->dev, "Apply PWM state for %s led failed, rc=%d\n",
					led->cdev.name, rc);

	return rc;
}

#define PBS_ENABLE	1
#define PBS_DISABLE	2
#define PBS_ARG		0x42
#define PBS_TRIG_CLR	0xE6
#define PBS_TRIG_SET	0xE5
static int __tri_led_set(struct qpnp_led_dev *led)
{
	int rc = 0;
	u8 val = 0, mask = 0, pbs_val;
	u8 prev_bitmap;

	rc = __tri_led_config_pwm(led, &led->pwm_setting);
	if (rc < 0) {
		dev_err(led->chip->dev, "Configure PWM for %s led failed, rc=%d\n",
					led->cdev.name, rc);
		return rc;
	}

	mask |= 1 << (TRILED_EN_CTL_MAX_BIT - led->id);

	if (led->pwm_setting.duty_ns == 0)
		val = 0;
	else
		val = mask;

	if (led->chip->subtype == TRILED_SUBTYPE_LED2H0L12 &&
		led->chip->pbs_nvmem) {
		/*
		 * Control BOB_CONFIG_EXT_CTRL2_FORCE_EN for HR_LED through
		 * PBS trigger. PBS trigger for enable happens if any one of
		 * LEDs are turned on. PBS trigger for disable happens only
		 * if both LEDs are turned off.
		 */

		prev_bitmap = led->chip->bitmap;
		if (val)
			led->chip->bitmap |= (1 << led->id);
		else
			led->chip->bitmap &= ~(1 << led->id);

		if (!(led->chip->bitmap & prev_bitmap)) {
			pbs_val = led->chip->bitmap ? PBS_ENABLE : PBS_DISABLE;
			rc = nvmem_device_write(led->chip->pbs_nvmem, PBS_ARG,
				1, &pbs_val);
			if (rc < 0) {
				dev_err(led->chip->dev, "Couldn't set PBS_ARG, rc=%d\n",
					rc);
				return rc;
			}

			pbs_val = 1;
			rc = nvmem_device_write(led->chip->pbs_nvmem,
				PBS_TRIG_CLR, 1, &pbs_val);
			if (rc < 0) {
				dev_err(led->chip->dev, "Couldn't set PBS_TRIG_CLR, rc=%d\n",
					rc);
				return rc;
			}

			pbs_val = 1;
			rc = nvmem_device_write(led->chip->pbs_nvmem,
				PBS_TRIG_SET, 1, &pbs_val);
			if (rc < 0) {
				dev_err(led->chip->dev, "Couldn't set PBS_TRIG_SET, rc=%d\n",
					rc);
				return rc;
			}
		}
	}

	rc = qpnp_tri_led_masked_write(led->chip, TRILED_REG_EN_CTL,
							mask, val);
	if (rc < 0)
		dev_err(led->chip->dev, "Update addr 0x%x failed, rc=%d\n",
					TRILED_REG_EN_CTL, rc);

	return rc;
}

static int qpnp_tri_led_set(struct qpnp_led_dev *led)
{
	u64 on_ms, off_ms, period_ns, duty_ns;
	enum led_brightness brightness = led->led_setting.brightness;
	int rc = 0;

	if (led->led_setting.blink) {
		on_ms = led->led_setting.on_ms;
		off_ms = led->led_setting.off_ms;

		duty_ns = on_ms * NSEC_PER_MSEC;
		period_ns = (on_ms + off_ms) * NSEC_PER_MSEC;

		if (period_ns < duty_ns && duty_ns != 0)
			period_ns = duty_ns + 1;
	} else {
		/* Use initial period if no blinking is required */
		period_ns = led->pwm_setting.pre_period_ns;

		if (brightness == LED_OFF)
			duty_ns = 0;

		duty_ns = period_ns * brightness;
		do_div(duty_ns, LED_FULL);

		if (period_ns < duty_ns && duty_ns != 0)
			period_ns = duty_ns + 1;
	}
	dev_dbg(led->chip->dev, "PWM settings for %s led: period = %lluns, duty = %lluns\n",
				led->cdev.name, period_ns, duty_ns);

	led->pwm_setting.duty_ns = duty_ns;
	led->pwm_setting.period_ns = period_ns;

	rc = __tri_led_set(led);
	if (rc < 0) {
		dev_err(led->chip->dev, "__tri_led_set %s failed, rc=%d\n",
				led->cdev.name, rc);
		return rc;
	}

	if (led->led_setting.blink) {
		led->cdev.brightness = LED_FULL;
		led->blinking = true;
		led->breathing = false;
	} else if (led->led_setting.breath) {
		led->cdev.brightness = LED_FULL;
		led->blinking = false;
		led->breathing = true;
	} else {
		led->cdev.brightness = led->led_setting.brightness;
		led->blinking = false;
		led->breathing = false;
	}

	return rc;
}

static int qpnp_rgb_set_direct(struct qpnp_led_dev *led)
{
	int rc;
	int duty_ns;
	u8 mask = 0;

	mask |= 1 << (TRILED_EN_CTL_MAX_BIT - led->id);

	led->cdev.brightness = led->led_setting.brightness;
	led->pwm_dev->state.output_type = PWM_OUTPUT_FIXED;
	duty_ns = (PWM_NS / (RGB_MAX_LEVEL-1)) * led->cdev.brightness;
	rc = pwm_config(led->pwm_dev, duty_ns, PWM_NS);
	if (rc < 0) {
		dev_err(led->chip->dev,
			"pwm config failed\n");
		return rc;
	}

	if (led->cdev.brightness > 0) {
		rc = qpnp_tri_led_masked_write(led->chip,
			TRILED_REG_EN_CTL,
			mask, mask);
		if (rc != 0) {
			dev_err(led->chip->dev,
				"Failed to write led enable reg\n");
			return rc;
		}
		rc = pwm_enable(led->pwm_dev);
		if (rc <= 0)
			led->pwm_setting.enabled = true;
	} else {
		pwm_disable(led->pwm_dev);
		led->pwm_setting.enabled = false;
		rc = qpnp_tri_led_masked_write(led->chip,
			TRILED_REG_EN_CTL,
			mask, RGB_LED_DISABLE);
		if (rc != 0) {
			dev_err(led->chip->dev,
				"Failed to write led enable reg\n");
			return rc;
		}
	}

	return 0;
}

static int qpnp_tri_led_set_brightness(struct led_classdev *led_cdev,
		enum led_brightness brightness)
{
	struct qpnp_led_dev *led;
	int rc = 0;

	if (led_cdev == NULL) {
		pr_err("%s parameter error NULL\n", __func__);
		return 0;
	}

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	mutex_lock(&led->lock);
	if (brightness > led->cdev.max_brightness)
		brightness = led->cdev.max_brightness;

	if (brightness != 0 && brightness == led->led_setting.brightness &&
			!led->blinking && !led->breathing) {
		mutex_unlock(&led->lock);
		return 0;
	}

	led->led_setting.brightness = brightness;
	if (!!brightness)
		led->led_setting.off_ms = 0;
	else
		led->led_setting.on_ms = 0;
	led->led_setting.blink = false;
	led->led_setting.breath = false;

	rc = qpnp_rgb_set_direct(led);
	if (rc)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

	mutex_unlock(&led->lock);

	return rc;
}

static enum led_brightness qpnp_tri_led_get_brightness(
			struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static int qpnp_tri_led_set_blink(struct led_classdev *led_cdev,
		unsigned long *on_ms, unsigned long *off_ms)
{
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);
	int rc = 0;

	mutex_lock(&led->lock);
	if (led->blinking && *on_ms == led->led_setting.on_ms &&
			*off_ms == led->led_setting.off_ms) {
		dev_dbg(led_cdev->dev, "Ignore, on/off setting is not changed: on %lums, off %lums\n",
						*on_ms, *off_ms);
		mutex_unlock(&led->lock);
		return 0;
	}

	if (*on_ms == 0) {
		led->led_setting.blink = false;
		led->led_setting.breath = false;
		led->led_setting.brightness = LED_OFF;
	} else if (*off_ms == 0) {
		led->led_setting.blink = false;
		led->led_setting.breath = false;
		led->led_setting.brightness = led->cdev.brightness;
	} else {
		led->led_setting.on_ms = *on_ms;
		led->led_setting.off_ms = *off_ms;
		led->led_setting.blink = true;
		led->led_setting.breath = false;
	}

	rc = qpnp_tri_led_set(led);
	if (rc)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

	mutex_unlock(&led->lock);
	return rc;
}

static ssize_t breath_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", led->led_setting.breath);
}

static ssize_t breath_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int rc;
	bool breath;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led =
		container_of(led_cdev, struct qpnp_led_dev, cdev);

	rc = kstrtobool(buf, &breath);
	if (rc < 0)
		return rc;

	cancel_work_sync(&led_cdev->set_brightness_work);

	mutex_lock(&led->lock);
	if (led->breathing == breath)
		goto unlock;

	led->led_setting.blink = false;
	led->led_setting.breath = breath;
	led->led_setting.brightness = breath ? LED_FULL : LED_OFF;
	rc = qpnp_tri_led_set(led);
	if (rc < 0)
		dev_err(led->chip->dev, "Set led failed for %s, rc=%d\n",
				led->label, rc);

unlock:
	mutex_unlock(&led->lock);
	return (rc < 0) ? rc : count;
}

static DEVICE_ATTR_RW(breath);
static const struct attribute *breath_attrs[] = {
	&dev_attr_breath.attr,
	NULL
};

static void qpnp_rgb_set_duration(struct qpnp_led_dev *led)
{
	u32 calc_temp, duration;
	u32 *hi_count = &led->pwm_setting.ramp_config.pause_hi_count;
	u32 *lo_count = &led->pwm_setting.ramp_config.pause_lo_count;

	if (*hi_count > RGB_LED_MAX_PAUSE_TIME)
		*hi_count = RGB_LED_MAX_PAUSE_TIME;
	if (*lo_count > RGB_LED_MAX_PAUSE_TIME)
		*lo_count = RGB_LED_MAX_PAUSE_TIME;

	calc_temp = ((*hi_count >= *lo_count) ? *hi_count : *lo_count);
	duration = (calc_temp + (RGB_LED_MAX_PAUSE_REG - 1)) / RGB_LED_MAX_PAUSE_REG;
	*hi_count = (*hi_count / duration) + 1;
	*lo_count = (*lo_count / duration) + 1;
	led->pwm_setting.ramp_config.step_ms = (duration - 1);
	led->pwm_setting.use_duration = false;

	pr_debug("%s : pause_hi=%d pause_lo=%d step_ms=%d\n", __func__, *hi_count, *lo_count,
		led->pwm_setting.ramp_config.step_ms);

	return;
}

static int rgbled_set_lut_to_register(struct qpnp_led_dev *led)
{
	int rc;
	int duty_ns;

	led->pwm_dev->state.output_type = PWM_OUTPUT_MODULATED;
	duty_ns = (PWM_NS / (RGB_MAX_LEVEL-1)) * led->cdev.brightness;
	rc = pwm_config(led->pwm_dev, duty_ns, PWM_NS);
	if (rc < 0) {
		dev_err(led->chip->dev,
			"pwm config failed\n");
		return rc;
	}

	rc = pwm_config_lut(led->pwm_dev, &led->pwm_setting.ramp_config);
	if (rc < 0) {
		dev_err(led->chip->dev,
			"pwm_config_lut failed\n");
		return rc;
	}

	return rc;
}

#define MAX_DUTY_PATTERN 16

static ssize_t rgbled_lut_pwm_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct qpnp_led_dev *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned int lut[MAX_DUTY_PATTERN];

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		lut, lut+1, lut+2, lut+3, lut+4, lut+5, lut+6, lut+7,
		lut+8, lut+9, lut+10, lut+11, lut+12, lut+13, lut+14, lut+15);
	if (ret == 0) {
		lut[0] = 0;
		ret = 1;
	}

	led->pwm_setting.ramp_config.pattern_length = ret;
	if (led->pwm_setting.ramp_config.pattern == 0)
		led->pwm_setting.ramp_config.pattern = devm_kzalloc(dev,
			sizeof(*led->pwm_setting.ramp_config.pattern) *
			led->pwm_setting.ramp_config.pattern_length,
			GFP_KERNEL);
	if (led->pwm_setting.ramp_config.pattern_length > 0) {
		led->led_setting.blink = true;
		led->pwm_setting.ramp_config.lo_idx =
			(led->id * RGB_LED_INDEX_MAX_EACH_LED) +
			RGB_LED_INDEX_OFFSET;
		led->pwm_setting.ramp_config.hi_idx =
			led->pwm_setting.ramp_config.lo_idx +
			led->pwm_setting.ramp_config.pattern_length -
			1;
		led->pwm_setting.ramp_config.ramp_dir_low_to_hi = true;
		led->pwm_setting.ramp_config.pattern_repeat = true;
		led->pwm_setting.ramp_config.toggle = false;
		memcpy(led->pwm_setting.ramp_config.pattern, lut,
			led->pwm_setting.ramp_config.pattern_length);
	}

	return count;
}

static ssize_t rgbled_lut_pwm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led;
	int *lut;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);
	lut = led->pwm_setting.ramp_config.pattern;

	return scnprintf(buf, PAGE_SIZE, "len:%u,[0]:%d,[1]:%d\n",
		led->pwm_setting.ramp_config.pattern_length, lut[0], lut[1]);
}

static ssize_t rgbled_step_duration_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct qpnp_led_dev *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long duration;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtoul(buf, 10, &duration);
	if (ret != 0)
		return ret;

	if (duration > RAMP_STEP_DURATION_MAX)
		duration = RAMP_STEP_DURATION_MAX;
	led->pwm_setting.ramp_config.step_ms = duration;

	return count;
}

static ssize_t rgbled_step_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		led->pwm_setting.ramp_config.step_ms);
}

static ssize_t rgbled_pause_hi_multi_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct qpnp_led_dev *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long lut_pause;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtoul(buf, 10, &lut_pause);
	if (ret != 0)
		return ret;

	if (lut_pause > RGB_LED_LUT_MAX_PAUSE)
		led->pwm_setting.use_duration = true;

	led->pwm_setting.ramp_config.pause_hi_count = lut_pause;

	return count;
}

static ssize_t rgbled_pause_hi_multi_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		led->pwm_setting.ramp_config.pause_hi_count);
}

static ssize_t rgbled_pause_lo_multi_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct qpnp_led_dev *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long lut_pause;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	ret = kstrtoul(buf, 10, &lut_pause);
	if (ret != 0)
		return ret;

	if (lut_pause > RGB_LED_LUT_MAX_PAUSE)
		led->pwm_setting.use_duration = true;

	led->pwm_setting.ramp_config.pause_lo_count = lut_pause;
	return count;
}

static ssize_t rgbled_pause_lo_multi_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		led->pwm_setting.ramp_config.pause_lo_count);
}

static ssize_t rgbled_single_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		led->led_setting.single_pwm_value);
}

static ssize_t rgbled_mix_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led;

	led = container_of(led_cdev, struct qpnp_led_dev, cdev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		led->led_setting.mix_pwm_value);
}

static DEVICE_ATTR(lut_pwm, S_IRUGO | S_IWUSR | S_IWGRP,
		rgbled_lut_pwm_show, rgbled_lut_pwm_store);
static DEVICE_ATTR(step_duration, S_IRUGO | S_IWUSR | S_IWGRP,
		rgbled_step_duration_show, rgbled_step_duration_store);
static DEVICE_ATTR(pause_hi_multi, S_IRUGO | S_IWUSR | S_IWGRP,
		rgbled_pause_hi_multi_show, rgbled_pause_hi_multi_store);
static DEVICE_ATTR(pause_lo_multi, S_IRUGO | S_IWUSR | S_IWGRP,
		rgbled_pause_lo_multi_show, rgbled_pause_lo_multi_store);
static DEVICE_ATTR(max_single_brightness, S_IRUGO,
		rgbled_single_value_show, NULL);
static DEVICE_ATTR(max_mix_brightness, S_IRUGO,
		rgbled_mix_value_show, NULL);
static const struct attribute *rgbled_attrs[] = {
	&dev_attr_lut_pwm.attr,
	&dev_attr_step_duration.attr,
	&dev_attr_pause_hi_multi.attr,
	&dev_attr_pause_lo_multi.attr,
	&dev_attr_max_single_brightness.attr,
	&dev_attr_max_mix_brightness.attr,
	NULL
};

static int rgbcommon_turn_off_force(struct qpnp_tri_led_chip *chip)
{
	int i;
	int rc = 0;
	struct qpnp_led_dev *led;

	for (i = 0; i < chip->num_leds; ++i) {
		led = &chip->leds[i];
		if (led->pwm_setting.ramp_config.pattern) {
			devm_kfree(led->cdev.dev, led->pwm_setting.ramp_config.pattern);
			led->pwm_setting.ramp_config.pattern = NULL;
			led->pwm_setting.ramp_config.pattern_length = 0;
		}
		led->led_setting.blink = false;
		led->led_setting.brightness = LED_OFF;
		rc = qpnp_rgb_set_direct(led);
		if (rc != 0)
			break;
	}

	return rc;
}

static ssize_t rgbcommon_sync_state_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long state;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	struct qpnp_tri_led_chip *chip;

	ret = kstrtoul(buf, 10, &state);
	if (ret != 0)
		return ret;
	if (state > RGBSYNC_STATE_BLINKING) {
		dev_err(dev, "%s out of range\n", __func__);
		return -EFAULT;
	}
	chip = container_of(led_cdev, struct qpnp_tri_led_chip, rgb_cdev);
	if (chip == NULL) {
		dev_err(dev, "%s chip NULL\n", __func__);
		return -EFAULT;
	}

	if (chip->sync_state == state) {
		dev_err(dev, "%s do nothing. same state", __func__);
		return count;
	} else if ((chip->sync_state == RGBSYNC_STATE_BLINKING)
			|| ((chip->sync_state == RGBSYNC_STATE_NOT_BLINK)
			&& (state == RGBSYNC_STATE_CONFIGURE_TO_BLINK))) {
		dev_err(dev, "%s turn off force. state changed\n", __func__);
		ret = rgbcommon_turn_off_force(chip);
		if (ret != 0) {
			dev_err(dev, "Failed to turn off force\n");
			return ret;
		}
	}
	chip->sync_state = state;

	return count;
}

static ssize_t rgbcommon_sync_state_show(struct device *ldev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(ldev);
	struct device *dev = led_cdev->dev->parent;
	struct qpnp_tri_led_chip *chip;

	chip = container_of(led_cdev, struct qpnp_tri_led_chip, rgb_cdev);
	if (chip == NULL) {
		dev_err(dev, "%s chip NULL\n", __func__);
		return -EFAULT;
	}
	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->sync_state);
}

static ssize_t rgbcommon_start_blink_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_led_dev *led = NULL;
	struct qpnp_tri_led_chip *chip;
	u8 mask = 0;
	int i, rc;

	chip = container_of(led_cdev, struct qpnp_tri_led_chip, rgb_cdev);
	if (chip == NULL) {
		dev_err(dev, "%s chip NULL\n", __func__);
		return -EFAULT;
	}

	if (chip->sync_state != RGBSYNC_STATE_CONFIGURE_TO_BLINK) {
		dev_err(dev, "%s ignore. not configure to blink state\n",
			__func__);
		return count;
	}

	chip->sync_state = RGBSYNC_STATE_BLINKING;
	/* must set only led which is blinked */
	for (i = 0; i < chip->num_leds; ++i) {
		led = &chip->leds[i];
		if (led->pwm_setting.use_duration)
			qpnp_rgb_set_duration(led);
		mask |= 1 << (TRILED_EN_CTL_MAX_BIT - led->id);
		if (led->led_setting.blink) {
			rgbled_set_lut_to_register(led);
			rc = qpnp_tri_led_masked_write(led->chip,
				TRILED_REG_EN_CTL,
				mask, mask);
			if (rc != 0) {
				dev_err(led->chip->dev,
					"Failed to write led enable reg\n");
				return rc;
			}
			if (led->pwm_setting.enabled) {
				pwm_disable(led->pwm_dev);
				led->pwm_setting.enabled = false;
			}
			rc = pwm_enable(led->pwm_dev);
			if (rc <= 0)
				led->pwm_setting.enabled = true;
		}
	}

	return count;
}

static DEVICE_ATTR(sync_state, S_IRUGO | S_IWUSR | S_IWGRP,
		rgbcommon_sync_state_show, rgbcommon_sync_state_store);
static DEVICE_ATTR(start_blink, S_IWUSR | S_IWGRP,
		NULL, rgbcommon_start_blink_store);
static const struct attribute *rgbcommon_attr[] = {
	&dev_attr_sync_state.attr,
	&dev_attr_start_blink.attr,
	NULL
};

static int qpnp_tri_led_register(struct qpnp_tri_led_chip *chip)
{
	struct qpnp_led_dev *led;
	int rc, i, j;

	for (i = 0; i < chip->num_leds; i++) {
		led = &chip->leds[i];
		mutex_init(&led->lock);
		led->cdev.name = led->label;
		led->cdev.max_brightness = led->max_current;
		led->cdev.brightness_set_blocking = qpnp_tri_led_set_brightness;
		led->cdev.brightness_get = qpnp_tri_led_get_brightness;
		led->cdev.blink_set = qpnp_tri_led_set_blink;
		led->cdev.default_trigger = led->default_trigger;
		led->cdev.brightness = LED_OFF;
		led->cdev.flags |= LED_KEEP_TRIGGER;

		rc = devm_led_classdev_register(chip->dev, &led->cdev);
		if (rc < 0) {
			dev_err(chip->dev, "%s led class device registering failed, rc=%d\n",
							led->label, rc);
			goto err_out;
		}

		if (pwm_get_output_type_supported(led->pwm_dev)
				& PWM_OUTPUT_MODULATED) {
			rc = sysfs_create_files(&led->cdev.dev->kobj,
					breath_attrs);
			if (rc < 0) {
				dev_err(chip->dev, "Create breath file for %s led failed, rc=%d\n",
						led->label, rc);
				goto err_out;
			}
			rc = sysfs_create_files(&led->cdev.dev->kobj,
					rgbled_attrs);
			if (rc < 0) {
				dev_err(chip->dev, "Create breath file for %s led failed, rc=%d\n",
						led->label, rc);
				goto err_out;
			}
		}
	}

	return 0;

err_out:
	for (j = 0; j <= i; j++) {
		if (j < i) {
			sysfs_remove_files(&chip->leds[j].cdev.dev->kobj,
					breath_attrs);
			sysfs_remove_files(&chip->leds[j].cdev.dev->kobj,
					rgbled_attrs);
		}
		mutex_destroy(&chip->leds[j].lock);
	}
	return rc;
}

static int qpnp_tri_led_hw_init(struct qpnp_tri_led_chip *chip)
{
	int rc = 0;
	u8 val;

	rc = qpnp_tri_led_read(chip, TRILED_REG_TYPE, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Read REG_TYPE failed, rc=%d\n", rc);
		return rc;
	}

	if (val != TRILED_TYPE) {
		dev_err(chip->dev, "invalid subtype(%d)\n", val);
		return -ENODEV;
	}

	rc = qpnp_tri_led_read(chip, TRILED_REG_SUBTYPE, &val);
	if (rc < 0) {
		dev_err(chip->dev, "Read REG_SUBTYPE failed, rc=%d\n", rc);
		return rc;
	}

	chip->subtype = val;

	return 0;
}

static int qpnp_tri_led_parse_dt(struct qpnp_tri_led_chip *chip)
{
	struct device_node *node = chip->dev->of_node, *child_node;
	struct qpnp_led_dev *led;
	struct pwm_args pargs;
	const __be32 *addr;
	int rc = 0, id, i = 0;
	u32 prepare_rgb_sync;
	u32 color_variation_max_num;
	u32 *rgb_current_table;
	int current_index = 0;
	int max_pwm_value;
	int j;

	addr = of_get_address(chip->dev->of_node, 0, NULL, NULL);
	if (!addr) {
		dev_err(chip->dev, "Getting address failed\n");
		return -EINVAL;
	}
	chip->reg_base = be32_to_cpu(addr[0]);

	chip->num_leds = of_get_available_child_count(node);
	if (chip->num_leds == 0) {
		dev_err(chip->dev, "No led child node defined\n");
		return -ENODEV;
	}

	if (chip->num_leds > TRILED_NUM_MAX) {
		dev_err(chip->dev, "can't support %d leds(max %d)\n",
				chip->num_leds, TRILED_NUM_MAX);
		return -EINVAL;
	}

	if (of_find_property(chip->dev->of_node, "nvmem", NULL)) {
		chip->pbs_nvmem = devm_nvmem_device_get(chip->dev, "pbs_sdam");
		if (IS_ERR_OR_NULL(chip->pbs_nvmem)) {
			rc = PTR_ERR(chip->pbs_nvmem);
			if (rc != -EPROBE_DEFER) {
				dev_err(chip->dev, "Couldn't get nvmem device, rc=%d\n",
					rc);
				return -ENODEV;
			}
			chip->pbs_nvmem = NULL;
			return rc;
		}
	}

	chip->leds = devm_kcalloc(chip->dev, chip->num_leds,
			sizeof(struct qpnp_led_dev), GFP_KERNEL);
	if (!chip->leds)
		return -ENOMEM;

	rc = of_property_read_u32(node, "somc,rgb_sync", &prepare_rgb_sync);
	if (rc < 0)
		prepare_rgb_sync = 0;
	if (prepare_rgb_sync != 0) {
		chip->rgb_cdev.name = "rgb";
		rc = led_classdev_register(chip->dev, &chip->rgb_cdev);
		if (rc != 0)
			dev_err(chip->dev, "unable to register rgb %d\n", rc);
		else {
			rc = sysfs_create_files(&chip->rgb_cdev.dev->kobj, rgbcommon_attr);
			if (rc < 0)
				dev_err(chip->dev, "unable to create sysfs %d\n", rc);
		}
	}
	dev_info(chip->dev, "rgb_sync prepare %d\n", prepare_rgb_sync);

	for_each_available_child_of_node(node, child_node) {
		rc = of_property_read_u32(child_node, "led-sources", &id);
		if (rc) {
			dev_err(chip->dev, "Get led-sources failed, rc=%d\n",
							rc);
			return rc;
		}

		if (id >= TRILED_NUM_MAX) {
			dev_err(chip->dev, "only support 0~%d current source\n",
					TRILED_NUM_MAX - 1);
			return -EINVAL;
		}

		led = &chip->leds[i++];
		led->chip = chip;
		led->id = id;
		led->label =
			of_get_property(child_node, "label", NULL) ? :
							child_node->name;

		led->pwm_dev =
			devm_of_pwm_get(chip->dev, child_node, NULL);
		if (IS_ERR(led->pwm_dev)) {
			rc = PTR_ERR(led->pwm_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev, "Get pwm device for %s led failed, rc=%d\n",
							led->label, rc);
			return rc;
		}

		pwm_get_args(led->pwm_dev, &pargs);
		if (pargs.period == 0)
			led->pwm_setting.pre_period_ns = PWM_PERIOD_DEFAULT_NS;
		else
			led->pwm_setting.pre_period_ns = pargs.period;

		led->default_trigger = of_get_property(child_node,
				"linux,default-trigger", NULL);

		if ((strncmp(led->label, "red", strlen("red")) == 0) ||
			(strncmp(led->label, "green", strlen("green")) == 0) ||
			(strncmp(led->label, "blue", strlen("blue")) == 0)) {
			rc = of_property_read_u32(child_node,
				"somc,color_variation_max_num",
				&color_variation_max_num);
			if (rc < 0) {
				dev_err(chip->dev, "Unable to read color_variation_max_num\n");
				color_variation_max_num = RGB_CURR_DEFAULT_PATTERN;
				rc = 0;
			}
			dev_info(chip->dev, "color_variation_max_num[%d] rgb_current_index[%d]\n",
				color_variation_max_num, rgb_current_index);

			rgb_current_table = devm_kzalloc(chip->dev,
				sizeof(u32) * color_variation_max_num * RGB_CURR_UNIT_NUM,
				GFP_KERNEL);
			if (rgb_current_table != 0) {
				rc = of_property_read_u32_array(child_node, "somc,max_current",
					rgb_current_table,
					color_variation_max_num * RGB_CURR_UNIT_NUM);
				if (rc < 0) {
					dev_err(chip->dev, "Unable to read max_mix_current\n");
					led->led_setting.single_pwm_value = RGB_MAX_LEVEL - 1;
					led->led_setting.mix_pwm_value = RGB_MAX_LEVEL - 1;
					rc = 0;
				} else {
					for (j = 0; j < color_variation_max_num; j++) {
						current_index = j * RGB_CURR_UNIT_NUM;
						if (rgb_current_index == rgb_current_table[current_index]) {
							break;
						}
					}
					led->led_setting.single_pwm_value =
						rgb_current_table[current_index + 1];
					led->led_setting.mix_pwm_value =
						rgb_current_table[current_index + 2];
				}
				devm_kfree(chip->dev, rgb_current_table);
			} else {
				dev_err(chip->dev, "Unable to allocate memory\n");
				led->led_setting.single_pwm_value = RGB_MAX_LEVEL - 1;
				led->led_setting.mix_pwm_value = RGB_MAX_LEVEL - 1;
			}

			led->max_current = (led->led_setting.single_pwm_value >
				led->led_setting.mix_pwm_value ?
				led->led_setting.single_pwm_value :
				led->led_setting.mix_pwm_value);
			pwm_set_max_pwm_value(led->pwm_dev, led->max_current);
			max_pwm_value = pwm_get_max_pwm_value(led->pwm_dev);
			led->max_current
				= max_pwm_value ? max_pwm_value : RGB_MAX_LEVEL;
		}
	}

	return rc;
}

static int qpnp_tri_led_probe(struct platform_device *pdev)
{
	struct qpnp_tri_led_chip *chip;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Getting regmap failed\n");
		return -EINVAL;
	}

	rc = qpnp_tri_led_parse_dt(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Devicetree properties parsing failed, rc=%d\n",
								rc);
		return rc;
	}

	mutex_init(&chip->bus_lock);

	rc = qpnp_tri_led_hw_init(chip);
	if (rc) {
		dev_err(chip->dev, "HW initialization failed, rc=%d\n", rc);
		goto destroy;
	}

	dev_set_drvdata(chip->dev, chip);
	rc = qpnp_tri_led_register(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Registering LED class devices failed, rc=%d\n",
								rc);
		goto destroy;
	}

	dev_dbg(chip->dev, "Tri-led module with subtype 0x%x is detected\n",
					chip->subtype);
	return 0;
destroy:
	mutex_destroy(&chip->bus_lock);
	dev_set_drvdata(chip->dev, NULL);

	return rc;
}

static int qpnp_tri_led_remove(struct platform_device *pdev)
{
	int i;
	struct qpnp_tri_led_chip *chip = dev_get_drvdata(&pdev->dev);

	mutex_destroy(&chip->bus_lock);
	for (i = 0; i < chip->num_leds; i++) {
		sysfs_remove_files(&chip->leds[i].cdev.dev->kobj, breath_attrs);
		mutex_destroy(&chip->leds[i].lock);
	}
	dev_set_drvdata(chip->dev, NULL);
	return 0;
}

static const struct of_device_id qpnp_tri_led_of_match[] = {
	{ .compatible = "qcom,tri-led",},
	{ },
};

static struct platform_driver qpnp_tri_led_driver = {
	.driver		= {
		.name		= "qcom,tri-led",
		.of_match_table	= qpnp_tri_led_of_match,
	},
	.probe		= qpnp_tri_led_probe,
	.remove		= qpnp_tri_led_remove,
};
module_platform_driver(qpnp_tri_led_driver);

MODULE_DESCRIPTION("QTI TRI_LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:qpnp-tri-led");
