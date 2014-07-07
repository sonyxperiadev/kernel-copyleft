/*
 * as3677.c - Led dimmer
 *
 * Version:
 * 2011-12-23: v1.4 : - group check is sometimes too strict, disable it
 * 2011-12-15: v1.3 : - fixed pattern starting/stopping
 *                    - fixed step reduced ALS if dimming was used before
 *                    - added startup brightness
 * 2011-09-07: v1.2 : - added support for using PWM to reduce step size in ALS
 *                    - incoroporate review comments
 *                    - fsm may not be reconfigured while in use
 *                    - fixing style issues
 *                    - fixed deadlock in suspend function introduced with
 *                      previous version
 * 2010-12-15: v1.1 : - patterns were restarted too often
 *                    - added max_current for leds
 *                    - DCDC was not correctly enabled
 *                    - fixed chargepump enabling
 *                    - fixed wrong cross checking constraining when dimming
 *                    - fixed adc reading
 *                    - fix for not disabling adder,
 *                    - fixed ALS (potential oops, falling slopes not working)
 *                    - added locking
 * 2010-06-11: v1.0 : first feature complete release
 *
 * Copyright (C) 2011 Ulrich Herrmann <ulrich.herrmann@austriamicrosystems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/leds-as3677.h>

/* Current group check function is too strict in some cases.
   Setting this define to 0 disables it */
#define AS3677_ENABLE_GROUP_CHECK 0

#define AS3677_NUM_LEDS  6

#define AS3677_REG_Control                  0x00
#define AS3677_REG_curr12_control           0x01
#define AS3677_REG_curr_rgb_control         0x02
#define AS3677_REG_LDO_Voltage              0x07
#define AS3677_REG_Curr1_current            0x09
#define AS3677_REG_Curr2_current            0x0A
#define AS3677_REG_Rgb1_current             0x0B
#define AS3677_REG_Rgb2_current             0x0C
#define AS3677_REG_Rgb3_current             0x0D
#define AS3677_REG_Pwm_control              0x16
#define AS3677_REG_Pwm_code                 0x17
#define AS3677_REG_Pattern_control          0x18
#define AS3677_REG_Pattern_data0            0x19
#define AS3677_REG_Pattern_data1            0x1A
#define AS3677_REG_Pattern_data2            0x1B
#define AS3677_REG_Pattern_data3            0x1C
#define AS3677_REG_GPIO_control             0x1E
#define AS3677_REG_GPIO_control_3           0x1F
#define AS3677_REG_GPIO_driving_cap         0x20
#define AS3677_REG_DCDC_control1            0x21
#define AS3677_REG_DCDC_control2            0x22
#define AS3677_REG_CP_control               0x23
#define AS3677_REG_CP_mode_Switch1          0x24
#define AS3677_REG_CP_mode_Switch2          0x25
#define AS3677_REG_ADC_control              0x26
#define AS3677_REG_ADC_MSB_result           0x27
#define AS3677_REG_ADC_LSB_result           0x28
#define AS3677_REG_Overtemp_control         0x29
#define AS3677_REG_Curr_low_voltage_status1 0x2A
#define AS3677_REG_Curr_low_voltage_status2 0x2B
#define AS3677_REG_Gpio_current             0x2C
#define AS3677_REG_Curr6_current            0x2F
#define AS3677_REG_Adder_Current_1          0x30
#define AS3677_REG_Adder_Current_2          0x31
#define AS3677_REG_Adder_Current_3          0x32
#define AS3677_REG_Adder_Enable_1           0x33
#define AS3677_REG_Adder_Enable_2           0x34
#define AS3677_REG_ASIC_ID1                 0x3E
#define AS3677_REG_ASIC_ID2                 0x3F
#define AS3677_REG_GPIO_output_2            0x50
#define AS3677_REG_GPIO_signal_2            0x51
#define AS3677_REG_Pattern_End              0x54
#define AS3677_REG_DLS_mode_control1        0x56
#define AS3677_REG_DLS_mode_control2        0x57
#define AS3677_REG_ALS_control              0x90
#define AS3677_REG_ALS_filter               0x91
#define AS3677_REG_ALS_offset               0x92
#define AS3677_REG_ALS_result               0x93
#define AS3677_REG_ALS_curr12_group         0x94
#define AS3677_REG_ALS_rgb_group            0x95
#define AS3677_REG_ALS_group_1_Y0           0x98
#define AS3677_REG_ALS_group_1_Y3           0x99
#define AS3677_REG_ALS_group_1_X1           0x9A
#define AS3677_REG_ALS_group_1_K1           0x9B
#define AS3677_REG_ALS_group_1_X2           0x9C
#define AS3677_REG_ALS_group_1_K2           0x9D
#define AS3677_REG_ALS_group_2_Y0           0x9E
#define AS3677_REG_ALS_group_2_Y3           0x9F
#define AS3677_REG_ALS_group_2_X1           0xA0
#define AS3677_REG_ALS_group_2_K1           0xA1
#define AS3677_REG_ALS_group_2_X2           0xA2
#define AS3677_REG_ALS_group_2_K2           0xA3
#define AS3677_REG_ALS_group_3_Y0           0xA4
#define AS3677_REG_ALS_group_3_Y3           0xA5
#define AS3677_REG_ALS_group_3_X1           0xA6
#define AS3677_REG_ALS_group_3_K1           0xA7
#define AS3677_REG_ALS_group_3_X2           0xA8
#define AS3677_REG_ALS_group_3_K2           0xA9
#define AS3677_REG_ALS_group_output_1       0xAA
#define AS3677_REG_ALS_group_output_2       0xAB
#define AS3677_REG_ALS_group_output_3       0xAC
#define AS3677_REG_ALS_range_high_threshold 0xAD
#define AS3677_REG_ALS_range_low_threshold  0xAE
#define AS3677_REG_Interrupt_Status         0xAF

#define ldev_to_led(c)       container_of(c, struct as3677_led, ldev)

#define AS3677_WRITE_REG(a, b) as3677_write_reg(data, (a), (b))
#define AS3677_READ_REG(a) as3677_read_reg(data, (a))
#define AS3677_MODIFY_REG(a, b, c) as3677_modify_reg(data, (a), (b), (c))
#define AS3677_WRITE_PATTERN(a) do { \
	u32 __d = a; \
	AS3677_WRITE_REG(AS3677_REG_Pattern_data0, __d & 0xff); __d >>= 8; \
	AS3677_WRITE_REG(AS3677_REG_Pattern_data1, __d & 0xff); __d >>= 8; \
	AS3677_WRITE_REG(AS3677_REG_Pattern_data2, __d & 0xff); __d >>= 8; \
	AS3677_WRITE_REG(AS3677_REG_Pattern_data3, __d & 0xff); \
	} while (0)

#define AS3677_LOCK()   mutex_lock(&(data)->update_lock)
#define AS3677_UNLOCK() mutex_unlock(&(data)->update_lock)

struct as3677_reg {
	const char *name;
	u8 value;
};

struct as3677_led {
	u8 reg;
	u8 mode_reg; /* fixed offset 0x93 to amb_mode */
	u8 mode_shift; /* same offsets for amb_mode */
	u8 adder_reg;
	u8 adder_on_reg;
	u8 adder_on_shift;
	u8 dls_mode_reg; /* fixed offset -0x32 to cp_mode reg */
	u8 dls_mode_shift; /* same for cp_mode_shift */
	u8 dls2shift; /* only for CURR2 and CURR6, if 0 not enabled */
	u8 mode; /* 0: off, 1: pattern/on, 2: up/down dimming */
	int dim_brightness;
	int dim_value;
	struct i2c_client *client;
	char *name;
	struct led_classdev ldev;
	struct as3677_platform_led *pled;
	bool use_pattern;
};

struct as3677_data {
	struct i2c_client *client;
	struct as3677_led leds[AS3677_NUM_LEDS];
	struct mutex update_lock;
	struct work_struct isr_work;
	struct delayed_work dim_work;
	struct as3677_reg regs[255];
	int curr_state;
	struct as3677_als_fsm_state *fsm;
	int fsm_num_states;
	enum dim_states {
		DIM_NONE = 0,
		DIM_UPDOWN,
		DIM_PATTERN,
		DIM_ALS}
	dimming_in_progress;
	int pattern_running;
	u32 pattern_data;
	u8 als_control_backup;
	u8 als_result_backup;
	u8 als_gpio_backup;
	u8 ss[4]; /* step size reduction = pwm value, used for ALS */
};

struct as3677_als_group {
	u8 gn;
	u8 y0;
	u8 x1;
	s8 k1;
	u8 x2;
	s8 k2;
	u8 y3;
	u8 ss;
};

struct as3677_als_fsm_state {
	u8 range_high;
	u8 range_low;
	u8 gpio;
	s32 sensor2lux;
	u8 gain;
	u8 offset;
	u8 ss;
	struct as3677_als_group groups[3];
};

static u8 as3677_read_reg(struct as3677_data *data, u8 reg);
static s32 as3677_write_reg(struct as3677_data *data, u8 reg, u8 value);
static s32 as3677_modify_reg(struct as3677_data *data, u8 reg, u8 reset,
		u8 set);
static void as3677_set_brightness(struct as3677_data *data,
		struct as3677_led *led, enum led_brightness value);
static void as3677_switch_als(struct as3677_data *data, int als_on);



#define AS3677_REG(NAME, VAL)[AS3677_REG_##NAME] = \
	{.name = __stringify(NAME), .value = (VAL)}

static const struct as3677_data as3677_default_data = {
	.client = NULL,
	.leds = {{
			.name = "as3677::curr1",
			.reg = AS3677_REG_Curr1_current,
			.mode_reg = AS3677_REG_curr12_control,
			.mode_shift = 0,
			.adder_reg = AS3677_REG_Adder_Current_1,
			.adder_on_reg = AS3677_REG_Adder_Enable_2,
			.adder_on_shift = 0,
			.dls_mode_reg = AS3677_REG_DLS_mode_control2,
			.dls_mode_shift = 0,
			.dls2shift = 0,
		},
		{
			.name = "as3677::curr2",
			.reg = AS3677_REG_Curr2_current,
			.mode_reg = AS3677_REG_curr12_control,
			.mode_shift = 2,
			.adder_reg = AS3677_REG_Adder_Current_2,
			.adder_on_reg = AS3677_REG_Adder_Enable_2,
			.adder_on_shift = 1,
			.dls_mode_reg = AS3677_REG_DLS_mode_control2,
			.dls_mode_shift = 1,
			.dls2shift = 5,
		},
		{
			.name = "as3677::curr6",
			.reg = AS3677_REG_Curr6_current,
			.mode_reg = AS3677_REG_curr_rgb_control,
			.mode_shift = 6,
			.adder_reg = AS3677_REG_Adder_Current_3,
			.adder_on_reg = AS3677_REG_Adder_Enable_2,
			.adder_on_shift = 2,
			.dls_mode_reg = AS3677_REG_DLS_mode_control2,
			.dls_mode_shift = 7,
			.dls2shift = 6,
		},
		{
			.name = "as3677::rgb1",
			.reg = AS3677_REG_Rgb1_current,
			.mode_reg = AS3677_REG_curr_rgb_control,
			.mode_shift = 0,
			.adder_reg = AS3677_REG_Adder_Current_1,
			.adder_on_reg = AS3677_REG_Adder_Enable_1,
			.adder_on_shift = 0,
			.dls_mode_reg = AS3677_REG_DLS_mode_control1,
			.dls_mode_shift = 4,
			.dls2shift = 0,
		},
		{
			.name = "as3677::rgb2",
			.reg = AS3677_REG_Rgb2_current,
			.mode_reg = AS3677_REG_curr_rgb_control,
			.mode_shift = 2,
			.adder_reg = AS3677_REG_Adder_Current_2,
			.adder_on_reg = AS3677_REG_Adder_Enable_1,
			.adder_on_shift = 1,
			.dls_mode_reg = AS3677_REG_DLS_mode_control1,
			.dls_mode_shift = 5,
			.dls2shift = 0,
		},
		{
			.name = "as3677::rgb3",
			.reg = AS3677_REG_Rgb3_current,
			.mode_reg = AS3677_REG_curr_rgb_control,
			.mode_shift = 4,
			.adder_reg = AS3677_REG_Adder_Current_3,
			.adder_on_reg = AS3677_REG_Adder_Enable_1,
			.adder_on_shift = 2,
			.dls_mode_reg = AS3677_REG_DLS_mode_control1,
			.dls_mode_shift = 6,
			.dls2shift = 0,
		},
	},
	.regs = {
		AS3677_REG(Control                 , 0),
		AS3677_REG(Control                 , 0),
		AS3677_REG(curr12_control          , 0),
		AS3677_REG(curr_rgb_control        , 0),
		AS3677_REG(LDO_Voltage             , 0),
		AS3677_REG(Curr1_current           , 0),
		AS3677_REG(Curr2_current           , 0),
		AS3677_REG(Rgb1_current            , 0),
		AS3677_REG(Rgb2_current            , 0),
		AS3677_REG(Rgb3_current            , 0),
		AS3677_REG(Pwm_control             , 0),
		AS3677_REG(Pwm_code                , 0),
		AS3677_REG(Pattern_control         , 0),
		AS3677_REG(Pattern_data0           , 0),
		AS3677_REG(Pattern_data1           , 0),
		AS3677_REG(Pattern_data2           , 0),
		AS3677_REG(Pattern_data3           , 0),
		AS3677_REG(GPIO_control            , 0x4c),
		AS3677_REG(GPIO_control_3          , 0x04),
		AS3677_REG(GPIO_driving_cap        , 0),
		AS3677_REG(DCDC_control1           , 0x02),
		AS3677_REG(DCDC_control2           , 0x88),
		/* to have automatic charge pump mode : */
		AS3677_REG(CP_control              , 0x40),
		AS3677_REG(CP_mode_Switch1         , 0x70),
		AS3677_REG(CP_mode_Switch2         , 0),
		AS3677_REG(ADC_control             , 0x02),
		AS3677_REG(ADC_MSB_result          , 0),
		AS3677_REG(ADC_LSB_result          , 0),
		AS3677_REG(Overtemp_control        , 0x11),
		AS3677_REG(Curr_low_voltage_status1, 0),
		AS3677_REG(Curr_low_voltage_status2, 0),
		AS3677_REG(Gpio_current            , 0x80),
		AS3677_REG(Curr6_current           , 0),
		AS3677_REG(Adder_Current_1         , 0),
		AS3677_REG(Adder_Current_2         , 0),
		AS3677_REG(Adder_Current_3         , 0),
		AS3677_REG(Adder_Enable_1          , 0),
		AS3677_REG(Adder_Enable_2          , 0),
		AS3677_REG(ASIC_ID1                , 0),
		AS3677_REG(ASIC_ID2                , 0),
		AS3677_REG(GPIO_output_2           , 0),
		AS3677_REG(GPIO_signal_2           , 0),
		AS3677_REG(Pattern_End             , 0),
		AS3677_REG(DLS_mode_control1       , 0),
		AS3677_REG(DLS_mode_control2       , 0),
		AS3677_REG(ALS_control             , 0),
		AS3677_REG(ALS_filter              , 0),
		AS3677_REG(ALS_offset              , 0),
		AS3677_REG(ALS_result              , 0),
		AS3677_REG(ALS_curr12_group        , 0),
		AS3677_REG(ALS_rgb_group           , 0),
		AS3677_REG(ALS_group_1_Y0          , 0),
		AS3677_REG(ALS_group_1_Y3          , 0),
		AS3677_REG(ALS_group_1_X1          , 0),
		AS3677_REG(ALS_group_1_K1          , 0),
		AS3677_REG(ALS_group_1_X2          , 0),
		AS3677_REG(ALS_group_1_K2          , 0),
		AS3677_REG(ALS_group_2_Y0          , 0),
		AS3677_REG(ALS_group_2_Y3          , 0),
		AS3677_REG(ALS_group_2_X1          , 0),
		AS3677_REG(ALS_group_2_K1          , 0),
		AS3677_REG(ALS_group_2_X2          , 0),
		AS3677_REG(ALS_group_2_K2          , 0),
		AS3677_REG(ALS_group_3_Y0          , 0),
		AS3677_REG(ALS_group_3_Y3          , 0),
		AS3677_REG(ALS_group_3_X1          , 0),
		AS3677_REG(ALS_group_3_K1          , 0),
		AS3677_REG(ALS_group_3_X2          , 0),
		AS3677_REG(ALS_group_3_K2          , 0),
		AS3677_REG(ALS_group_output_1      , 0),
		AS3677_REG(ALS_group_output_2      , 0),
		AS3677_REG(ALS_group_output_3      , 0),
		AS3677_REG(ALS_range_high_threshold, 0),
		AS3677_REG(ALS_range_low_threshold , 0),
		AS3677_REG(Interrupt_Status        , 0),
	},
	.curr_state = -1,
};

static int as3677_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int as3677_remove(struct i2c_client *client);

static const struct i2c_device_id as3677_id[] = {
	{ "as3677", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, as3677_id);

#ifdef CONFIG_PM
static int as3677_suspend(struct device *dev)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	dev_info(dev, "Suspending AS3677\n");

	AS3677_LOCK();
	data->als_control_backup = AS3677_READ_REG(AS3677_REG_ALS_control);
	data->als_gpio_backup    = AS3677_READ_REG(AS3677_REG_GPIO_output_2);
	data->als_result_backup = i2c_smbus_read_byte_data(data->client,
			AS3677_REG_ALS_result);
	as3677_switch_als(data, 0);
	AS3677_UNLOCK();

	return 0;
}

static int as3677_resume(struct device *dev)
{
	struct as3677_data *data = dev_get_drvdata(dev);

	dev_info(dev, "Resuming AS3677\n");

	AS3677_LOCK();
	AS3677_WRITE_REG(AS3677_REG_ALS_result, data->als_result_backup);
	if (data->als_control_backup & 1)
		as3677_switch_als(data, 1);
	AS3677_MODIFY_REG(AS3677_REG_GPIO_output_2, 0x06,
			data->als_gpio_backup & 0x06);
	AS3677_UNLOCK();

	return 0;
}

static int as3677_dev_suspend(struct device *dev)
{
	return as3677_suspend(dev);
}

static int as3677_dev_resume(struct device *dev)
{
	return as3677_resume(dev);
}

static const struct dev_pm_ops as3677_pm = {
	.suspend  = as3677_dev_suspend,
	.resume   = as3677_dev_resume,
};
#endif


static void as3677_shutdown(struct i2c_client *client)
{
	struct as3677_data *data = i2c_get_clientdata(client);
	int i;

	dev_info(&client->dev, "Shutting down AS3677\n");

	for (i = 0; i < AS3677_NUM_LEDS; i++)
		as3677_set_brightness(data, data->leds+i, 0);

	AS3677_LOCK();
	AS3677_WRITE_REG(AS3677_REG_Control, 0x00);
	AS3677_WRITE_REG(AS3677_REG_ALS_control, 0x00);
	AS3677_UNLOCK();
}

static struct i2c_driver as3677_driver = {
	.driver = {
		.name   = "as3677",
#ifdef CONFIG_PM
		.pm     = &as3677_pm,
#endif
	},
	.probe  = as3677_probe,
	.remove = as3677_remove,
	.shutdown = as3677_shutdown,
	.id_table = as3677_id,
};

static int device_add_attributes(struct device *dev,
				 struct device_attribute *attrs)
{
	int error = 0;
	int i;

	if (attrs) {
		for (i = 0; attr_name(attrs[i]); i++) {
			error = device_create_file(dev, &attrs[i]);
			if (error)
				break;
		}
		if (error)
			while (--i >= 0)
				device_remove_file(dev, &attrs[i]);
	}
	return error;
}

static void device_remove_attributes(struct device *dev,
				     struct device_attribute *attrs)
{
	int i;

	if (attrs)
		for (i = 0; attr_name(attrs[i]); i++)
			device_remove_file(dev, &attrs[i]);
}

static u8 as3677_read_reg(struct as3677_data *data, u8 reg)
{
	return data->regs[reg].value;
}

static s32 as3677_write_reg(struct as3677_data *data, u8 reg, u8 value)
{
	s32 ret = i2c_smbus_write_byte_data(data->client, reg, value);
	if (ret == 0)
		data->regs[reg].value = value;
	return ret;
}

static s32 as3677_modify_reg(struct as3677_data *data, u8 reg, u8 reset, u8 set)
{
	s32 ret;
	u8 val = (data->regs[reg].value & ~reset) | set;

	ret = i2c_smbus_write_byte_data(data->client, reg, val);

	if (ret == 0)
		data->regs[reg].value = val;
	return ret;
}

static void as3677_switch_led(struct as3677_data *data,
		struct as3677_led *led,
		int mode)
{
	u8 pattern_running = data->pattern_running;
	u8 old_mode = (AS3677_READ_REG(led->mode_reg) >> led->mode_shift) & 0x3;
	u8 gid;

	led->mode = mode;

	if (old_mode == 0x3 && (mode != 1 || led->use_pattern == 0))
		pattern_running--;

	if (old_mode != 0x3 && mode == 1 && led->use_pattern)
		pattern_running++;

	/* If the last led is going to leave the pattern generator stop it
	   avoiding artefacts */
	if (pattern_running == 0 && data->pattern_running == 1) {
		/* Stop pattern generator avoiding flashes at next start */
		AS3677_WRITE_PATTERN(0);
		/* Kickstart pattern generator */
		AS3677_WRITE_REG(AS3677_REG_Pattern_control,
				AS3677_READ_REG(AS3677_REG_Pattern_control));
	}

	switch (mode) {
	case 0: /* off */
		AS3677_MODIFY_REG(led->mode_reg,
				0x03 << led->mode_shift, 0);
		break;
	case 1: { /* on and pattern */
			u8 on_val =  1;
			if (led->use_pattern)
				on_val = 3;
			gid = (AS3677_READ_REG(led->mode_reg + 0x93) >>
				led->mode_shift) & 0x3;
			if (data->ss[gid])
				on_val = 2;
			AS3677_MODIFY_REG(led->mode_reg,
					0x03 << led->mode_shift,
					on_val << led->mode_shift);
			break;
		}
	case 2: /* dimming / pwm */
		AS3677_MODIFY_REG(led->mode_reg, 0x3<<led->mode_shift,
				0x2<<led->mode_shift);
		break;

	}

	/* If we have now the first led running on a pattern,
	   make sure values are correct */
	if (pattern_running == 1 && data->pattern_running == 0) {
		AS3677_WRITE_PATTERN(data->pattern_data);
		/* Kickstart pattern generator to correctly use pattern_delay */
		AS3677_WRITE_REG(AS3677_REG_Pattern_control,
				AS3677_READ_REG(AS3677_REG_Pattern_control));
	}
	data->pattern_running = pattern_running;
}

static void as3677_retrigger_led(struct as3677_data *data,
		struct as3677_led *led)
{
	as3677_switch_led(data, led, led->mode);
}

static void as3677_retrigger_group_leds(struct as3677_data *data, u8 gn)
{
	int i;
	u8 gid;

	for (i = 0; i < AS3677_NUM_LEDS; i++) {
		struct as3677_led *led = data->leds + i;
		gid = (AS3677_READ_REG(led->mode_reg + 0x93)
				>> led->mode_shift) & 0x3;
		if (gid == gn)
			as3677_retrigger_led(data, led);
	}
}

static void as3677_check_DCDC(struct as3677_data *data)
{ /* Here we check if one or more leds are connected to DCDC.
     Doing the same for charge pump is not necessary due to cp_auto_on */
	int i, on_dcdc = 0;
	struct as3677_led *led;
	for (i = 0; i < 3; i++) {
		led = data->leds + i;
		if (led->pled && led->pled->on_charge_pump)
			continue;
		if (AS3677_READ_REG(led->reg) != LED_OFF) {
			on_dcdc++;
			continue;
		}
		if (AS3677_READ_REG(led->adder_on_reg)
				& (1<<led->adder_on_shift))
			on_dcdc++;
	}
	AS3677_MODIFY_REG(AS3677_REG_Control, 0x8, on_dcdc ? 8 : 0);
}

static void as3677_set_led_brightness(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct device *dev = led_cdev->dev->parent;
	struct as3677_data *data = dev_get_drvdata(dev);
	struct as3677_led *led = ldev_to_led(led_cdev);
	if (led->dim_brightness != -1 &&
			data->dimming_in_progress == DIM_UPDOWN) {
		dev_warn(dev, "LED %s is currently being dimmed, changing"
				"brightness not possible!", led->name);
		return;
	}
	as3677_set_brightness(data, led, value);
}
static void as3677_set_brightness(struct as3677_data *data,
		struct as3677_led *led, enum led_brightness value)
{
	u8 prev_value = AS3677_READ_REG(led->reg);

	AS3677_LOCK();
	value = (value * led->pled->max_current_uA + 25500/2) / 25500;
	if ((prev_value == LED_OFF) != (value == LED_OFF)) {
		/* we must switch on/off */
		as3677_switch_led(data, led, value != LED_OFF);
		AS3677_WRITE_REG(led->reg, value);
		as3677_check_DCDC(data);
	} else {
		AS3677_WRITE_REG(led->reg, value);
	}
	AS3677_UNLOCK();
}

#define MSNPRINTF(...) do { act = snprintf(buf, rem, __VA_ARGS__);	\
				if (act > rem)				\
					goto exit;			\
				buf += act;				\
				rem -= act;				\
			} while (0)

static ssize_t as3677_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	size_t ps = PAGE_SIZE, cw = 0;
	u8 cr;
	int i = 0;
	struct as3677_reg *reg;

	while (cw < ps && i < ARRAY_SIZE(data->regs)) {
		ps -= cw;
		buf += cw;
		cw = 0;
		reg = data->regs + i;
		if (reg->name) {
			cr = i2c_smbus_read_byte_data(data->client, i);
			if (cr == reg->value)
				cw = snprintf(buf, ps, "%34s %02x: %#04x\n",
					reg->name, i, reg->value);
			else
				cw = snprintf(buf, ps,
						"%34s %02x: %#04x -> %#04x\n",
						reg->name, i, reg->value, cr);
		}
		i++;
	}
	ps -= cw;
	buf += cw;
	cw = snprintf(buf, ps, "pattern_running=%d\n", data->pattern_running);
	ps -= cw;
	buf += cw;

	return PAGE_SIZE - ps;
}

static ssize_t as3677_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int i;
	u8 reg, val;
	i = sscanf(buf, "0x%hhx=0x%hhx", &reg, &val);
	if (i != 2)
		return -EINVAL;
	AS3677_LOCK();
	AS3677_WRITE_REG(reg, val);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_use_pattern_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));

	snprintf(buf, PAGE_SIZE, "%d\n", led->use_pattern);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_use_pattern_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	int i, use_pattern;

	i = sscanf(buf, "%d", &use_pattern);
	if (i != 1)
		return -EINVAL;

	AS3677_LOCK();

	led->use_pattern = use_pattern;

	if (AS3677_READ_REG(led->reg) == LED_OFF)
		goto exit;

	as3677_switch_led(data, led, 1);
exit:
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_dim_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));

	snprintf(buf, PAGE_SIZE, "%d\n", led->dim_brightness);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_dim_brightness_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	int i;
	int dim_brightness;
	ssize_t err = strnlen(buf, PAGE_SIZE);

	i = sscanf(buf, "%d", &dim_brightness);
	if (i != 1)
		return -EINVAL;
	if (dim_brightness > 255 || dim_brightness < -1)
		return -EINVAL;

	AS3677_LOCK();
	if (data->dimming_in_progress) {
		dev_warn(dev, "Cannot change dimming parameters while dimming"
			     " is in progress %d\n", data->dimming_in_progress);
		if (data->dimming_in_progress == DIM_UPDOWN) {
			err = -EAGAIN;
			goto exit;
		}
		err = -EBUSY;
		goto exit;
	}
	led->dim_value =
		(dim_brightness * led->pled->max_current_uA + 25500/2) / 25500;
	led->dim_brightness = dim_brightness;
exit:
	AS3677_UNLOCK();
	return err;
}

static ssize_t as3677_pattern_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int pdata, p, i;
	ssize_t act, rem = PAGE_SIZE;
	AS3677_LOCK();
	pdata  = data->pattern_data;
	AS3677_UNLOCK();
	p = pdata;
	if (AS3677_READ_REG(AS3677_REG_Pattern_control) & 1) {
		int r = 0, g = 0, b = 0, i;
		/* 10 rgb bits in 32 bits */
		for (i = 0; i < 10; i++) {
			r |= (pdata & 1) << i;
			pdata >>= 1;
			g |= (pdata & 1) << i;
			pdata >>= 1;
			b |= (pdata & 1) << i;
			pdata >>= 1;
		}

		MSNPRINTF("use_color == 1: 30 bit pattern "
				"data %#010x, curr1/rgb1=%#05x, "
				"curr2/rgb2=%#05x, curr6/rgb3=%#05x\n"
				       , p, r, g, b);
		for (i = 0; i < 10; i++) {
			if (r & (1 << i))
				MSNPRINTF("-");
			else
				MSNPRINTF("_");
		}
		MSNPRINTF("\n");
		for (i = 0; i < 10; i++) {
			if (g & (1 << i))
				MSNPRINTF("-");
			else
				MSNPRINTF("_");
		}
		MSNPRINTF("\n");
		for (i = 0; i < 10; i++) {
			if (b & (1 << i))
				MSNPRINTF("-");
			else
				MSNPRINTF("_");
		}
	} else {
		MSNPRINTF("use_color == 0: 32 bit pattern "
				"data %#010x\n", p);
		for (i = 0; i < 32; i++) {
			if (p & (1 << i))
				MSNPRINTF("-");
			else
				MSNPRINTF("_");
		}
	}
	MSNPRINTF("\n");
exit:
	return PAGE_SIZE - rem;
}

static ssize_t as3677_pattern_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int r, g, b, pdata, i;
	i = sscanf(buf, "%x,%x,%x", &pdata, &g, &b);
	if (i != 1 && i != 3)
		return -EINVAL;
	if (i == 3) {
		r = pdata;
		pdata = 0;
		/* Read in 10 rgb bits */
		for (i = 9; i >= 0 ; i--) {
			pdata <<= 1;
			pdata |= (b >> i) & 1;
			pdata <<= 1;
			pdata |= (g >> i) & 1;
			pdata <<= 1;
			pdata |= (r >> i) & 1;
		}

	}
	AS3677_LOCK();
	data->pattern_data = pdata;
	if (data->pattern_running)
		AS3677_WRITE_PATTERN(pdata);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_pattern_duration_secs_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int pattern_duration = 1, creg;
	creg = AS3677_READ_REG(AS3677_REG_Gpio_current);
	if (creg & 0x40)
		pattern_duration *= 8;

	snprintf(buf, PAGE_SIZE, "%d, possible 1,8\n", pattern_duration);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_pattern_duration_secs_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int pattern_duration, i;
	i = sscanf(buf, "%u", &pattern_duration);
	if (i != 1)
		return -EINVAL;
	if ((pattern_duration != 8) && (pattern_duration != 1))
		return -EINVAL;
	AS3677_LOCK();
	AS3677_MODIFY_REG(AS3677_REG_Gpio_current, 0x40,
			  (pattern_duration == 8) ? 0x40 : 0);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_pattern_use_color_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int pattern_use_color = 42;
	pattern_use_color =  AS3677_READ_REG(AS3677_REG_Pattern_control) & 1;
	snprintf(buf, PAGE_SIZE, "%d\n", pattern_use_color);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_pattern_use_color_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int pattern_use_color, i;
	i = sscanf(buf, "%u", &pattern_use_color);
	pattern_use_color = !!pattern_use_color;
	if (i != 1)
		return -EINVAL;
	AS3677_LOCK();
	AS3677_MODIFY_REG(AS3677_REG_Pattern_control, 1, pattern_use_color);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_pattern_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int pattern_delay, creg, greg, pdsecs;
	creg = AS3677_READ_REG(AS3677_REG_Pattern_control);
	greg = AS3677_READ_REG(AS3677_REG_Gpio_current);
	pdsecs = pattern_delay = ((creg & 0x6) >> 1) | ((greg & 0x10) >> 2);
	if (greg & 0x40)
		pdsecs *= 8;
	snprintf(buf, PAGE_SIZE, "%d pattern_durations = %d secs\n",
			pattern_delay, pdsecs);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_pattern_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int pattern_delay, i;
	i = sscanf(buf, "%u", &pattern_delay);
	if (i != 1)
		return -EINVAL;
	if (pattern_delay > 7)
		return -EINVAL;
	pattern_delay = pattern_delay << 1;
	pattern_delay = (pattern_delay & 0x6) | ((pattern_delay & 0x8) << 1);
	AS3677_LOCK();
	AS3677_MODIFY_REG(AS3677_REG_Pattern_control,
			0x06, pattern_delay & 0x6);
	AS3677_MODIFY_REG(AS3677_REG_Gpio_current, 0x10, pattern_delay & 0x10);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_filter_speed_mHz_show(struct device *dev,
				char *buf, int offset)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int filter_speed_mHz, filter_speed;
	filter_speed = (AS3677_READ_REG(AS3677_REG_ALS_filter) >> offset) & 0x7;
	filter_speed_mHz = 250 << filter_speed;
	snprintf(buf, PAGE_SIZE, "%d\n", filter_speed_mHz);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_filter_speed_mHz_store(struct device *dev,
				const char *buf, size_t size, int offset)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int filter_speed_mHz, filter_speed, i, highest_bit;
	i = sscanf(buf, "%d", &filter_speed_mHz);
	if (i != 1)
		return -EINVAL;

	/* The following code rounds to the neares power of two */
	/* division while preserving round bit */
	filter_speed = (filter_speed_mHz << 1) / 250;
	highest_bit = fls(filter_speed);
	/* now check the second most significant bit */
	if (highest_bit < 2)
		filter_speed = filter_speed & 1;
	else if (filter_speed & (1 << (highest_bit - 2)))
		filter_speed = highest_bit - 1;
	else
		filter_speed = highest_bit - 2;

	if (filter_speed > 7)
		filter_speed = 7;

	AS3677_LOCK();
	AS3677_MODIFY_REG(AS3677_REG_ALS_filter, 0x7 << offset,
			filter_speed<<offset);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_filter_up_speed_mHz_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return as3677_als_filter_speed_mHz_show(dev, buf, 0);
}

static ssize_t as3677_als_filter_up_speed_mHz_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return as3677_als_filter_speed_mHz_store(dev, buf, size, 0);
}

static ssize_t as3677_als_filter_down_speed_mHz_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return as3677_als_filter_speed_mHz_show(dev, buf, 4);
}

static ssize_t as3677_als_filter_down_speed_mHz_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return as3677_als_filter_speed_mHz_store(dev, buf, size, 4);
}

static ssize_t as3677_pattern_use_softdim_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int use_softdim = AS3677_READ_REG(AS3677_REG_Pattern_control) & 0x8;
	use_softdim >>= 3;
	snprintf(buf, PAGE_SIZE, "%d\n", use_softdim);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_pattern_use_softdim_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int use_softdim, i;
	ssize_t err = strnlen(buf, PAGE_SIZE);

	i = sscanf(buf, "%d", &use_softdim);
	if (i != 1)
		return -EINVAL;
	use_softdim = !!use_softdim;

	AS3677_LOCK();
	if (((AS3677_READ_REG(AS3677_REG_Pattern_control) & 0x8) >> 3)
		== use_softdim)
		goto exit;

	if (use_softdim && data->dimming_in_progress == DIM_UPDOWN) {
		dev_warn(dev, "Cannot use softdim while dimming is in"
				" progress, try again later\n");
		err = -EAGAIN;
		goto exit;
	}
	if (use_softdim && data->dimming_in_progress == DIM_ALS) {
		dev_warn(dev, "Cannot use softdim while ALS uses PWM\n");
		err = -EBUSY;
		goto exit;
	}
	data->dimming_in_progress = (use_softdim) ? (DIM_PATTERN) : (DIM_NONE);
	AS3677_MODIFY_REG(AS3677_REG_Pattern_control, 0x08, use_softdim<<3);
exit:
	AS3677_UNLOCK();
	return err;
}

static ssize_t as3677_als_group_n_show(u8 gid, struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	u8 group_reg, ss;
	s32 y0, y3, x1, x2, x, y;
	s32 k1, k2;
	const uint inc = 8;
	int i;
	ssize_t act, rem = PAGE_SIZE;

	AS3677_LOCK();
	group_reg = AS3677_REG_ALS_group_1_Y0 + (gid - 1) * 6;
	y0 = AS3677_READ_REG(group_reg);
	y3 = AS3677_READ_REG(group_reg + 1);
	x1 = AS3677_READ_REG(group_reg + 2);
	k1 = (s8) AS3677_READ_REG(group_reg + 3);
	x2 = AS3677_READ_REG(group_reg + 4);
	k2 = (s8) AS3677_READ_REG(group_reg + 5);
	ss = data->ss[gid];
	AS3677_UNLOCK();
	MSNPRINTF("%d,%d,%d,%d,%d,%d,%d"
			"(y0=%d, x1=%d, k1=%d, x2=%d, k2=%d, y3=%d, ss=%d)\n",
			y0, x1, k1, x2, k2, y3, ss,
			y0, x1, k1, x2, k2, y3, ss);

	MSNPRINTF("       ");
	for (i = 0; i < 255; i += inc)
		MSNPRINTF("y");
	MSNPRINTF("\n");
	/* Print first segment, straight line */
	for (x = 0; x < x1; x += inc) {
		MSNPRINTF("x0=%3d,y=%3d:", x, y0);
		for (i = 0; i < y0; i += inc)
			MSNPRINTF(" ");
		MSNPRINTF("|\n");
	}
	/* Second segment, slope k1 */
	for (; x < x2; x += inc) {
		if (gid == 3)
			y = ((k1 * (x - x1))) + y0;
		else
			y = ((k1 * (x - x1)) / 32) + y0;
		MSNPRINTF("x1=%3d,y=%3d:", x, y);
		for (i = 0; i < y; i += inc)
			MSNPRINTF(" ");
		if (k1 < 0)
			MSNPRINTF("/\n");
		else
			MSNPRINTF("\\\n");
	}
	/* Third segment, slope k2 */
	for (; x < 256; x += inc) {
		if (gid == 3) {
			y = (k1 * (x2 - x1))
				+ ((k2 * (x - x2)) / 32)
				+ y0;
		} else {
			y = (((k1 * (x2 - x1))) / 32)
				+ ((k2 * (x - x2)) / 32)
				+ y0;
		}
		if ((k2 >= 0 && y > y3) ||
			(k2 < 0 && y < y3))
			break;
		MSNPRINTF("x2=%3d,y=%3d:", x, y);
		for (i = 0; i < y; i += inc)
			MSNPRINTF(" ");
		if (k2 < 0)
			MSNPRINTF("/\n");
		else
			MSNPRINTF("\\\n");
	}
	/* Fourth and last segment straight line */
	for (; x < 256; x += inc) {
		MSNPRINTF("x3=%3d,y=%3d:", x, y3);
		for (i = 0; i < y3; i += inc)
			MSNPRINTF(" ");
		MSNPRINTF("|\n");
	}

exit:
	return PAGE_SIZE - rem;
}

#if AS3677_ENABLE_GROUP_CHECK
static int as3677_als_group_check(struct device *dev,
		struct as3677_als_group *g)
{
	s32 y2, x3;
	if (g->gn == 0 || g->gn > 3) {
			dev_warn(dev, "invalid gn must be 1-3, is %d\n",
					g->gn);
			return -EINVAL;
	}
	if (g->y0 > g->y3) {
		if (g->k1 > 0 || g->k2 > 0) {
			dev_warn(dev, "k1=%d and k2=%d must be negative with"
					"this y0,y3\n", g->k1, g->k2);
			return -EINVAL;
		}
	} else {
		if (g->k1 < 0 || g->k2 < 0) {
			dev_warn(dev, "k1=%d and k2=%d must be positive with"
					"this given y0,y3\n", g->k1, g->k2);
			return -EINVAL;
		}
	}
	if (g->x1 == 0 || g->k1 == 0) {
		dev_warn(dev, "k1=%d and x1=%d must not be zero\n",
				g->k1, g->x1);
		return -EINVAL;
	}
	if (g->x1 > g->x2) {
		dev_warn(dev, "x1=%d must be smaller than x2=%d\n",
				g->x1, g->x2);
		return -EINVAL;
	}

	/* Test y2 */
	if (g->gn == 3)
		y2 = g->y0 + ((s32)g->k1) * (g->x2 - g->x1);
	else
		y2 = g->y0 + ((s32)g->k1) * (g->x2 - g->x1) / 32;

	if (y2 < 0 || y2 > 255 ||
		((g->k2 >= 0) && (g->y3 < y2)) ||
		((g->k2 < 0) && (g->y3 > y2))) {
		dev_warn(dev, "Invalid y2 would be %d\n", y2);
		return -EINVAL;
	}

	/* Test x3 */
	if (g->k2 == 0) {
		if (y2 != g->y3) {
			dev_warn(dev, "Invalid y3, must equal y2 for k2==0!\n");
			return -EINVAL;
		}
	} else {
		x3 = g->x2 + ((g->y3 - y2)) * 32 / g->k2;
		if (x3 < 0 || x3 > 255 || x3 < g->x2) {
			dev_warn(dev, "Invalid x3 would be %d, y2=%d\n",
					x3, y2);
			return -EINVAL;
		}
	}
	return 0;
}
#endif

static ssize_t as3677_als_group_n_store(u8 gid, struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int i;
	u8 group_reg = AS3677_REG_ALS_group_1_Y0 + (gid - 1) * 6;
	struct as3677_als_group g;
	ssize_t err = strnlen(buf, PAGE_SIZE);

	g.gn = gid;
	i = sscanf(buf, "%hhu,%hhu,%hhd,%hhu,%hhd,%hhu,%hhu",
			&g.y0, &g.x1, &g.k1, &g.x2, &g.k2, &g.y3, &g.ss);

	if (i < 7)
		return -EINVAL;

#if AS3677_ENABLE_GROUP_CHECK
	i = as3677_als_group_check(dev, &g);

	if (i != 0)
		return i;
#endif

	AS3677_LOCK();

	if (g.ss && data->dimming_in_progress == DIM_UPDOWN) {
		dev_warn(dev, "ALS group would use PWM but PWM is currently"
				" by up/down dimmer\n");
		err = -EAGAIN;
		goto exit;
	}
	if (g.ss && data->dimming_in_progress == DIM_PATTERN) {
		dev_warn(dev, "pattern_use_softdim is set, setting this group"
			       " not possible\n");
		err = -EBUSY;
		goto exit;
	}
	if (data->dimming_in_progress == DIM_ALS &&
			!data->ss[1] && !data->ss[2] && !data->ss[3])
		data->dimming_in_progress = DIM_NONE;
	if (data->ss[1] || data->ss[2] || data->ss[3]) {
		data->dimming_in_progress = DIM_ALS;
		/* no up/down dimming */
		AS3677_WRITE_REG(AS3677_REG_Pwm_control, 0);
	}
	AS3677_WRITE_REG(group_reg + 0, g.y0);
	AS3677_WRITE_REG(group_reg + 1, g.y3);
	AS3677_WRITE_REG(group_reg + 2, g.x1);
	AS3677_WRITE_REG(group_reg + 3, g.k1);
	AS3677_WRITE_REG(group_reg + 4, g.x2);
	AS3677_WRITE_REG(group_reg + 5, g.k2);
	if (data->ss[gid] != g.ss) {
		data->ss[gid] = g.ss;
		if (g.ss)
			AS3677_WRITE_REG(AS3677_REG_Pwm_code, g.ss);
		as3677_retrigger_group_leds(data, gid);
	}
exit:
	AS3677_UNLOCK();
	return err;
}

static ssize_t as3677_als_group1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return as3677_als_group_n_show(1, dev, attr, buf);
}

static ssize_t as3677_als_group1_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return as3677_als_group_n_store(1, dev, attr, buf, size);
}
static ssize_t as3677_als_group2_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return as3677_als_group_n_show(2, dev, attr, buf);
}

static ssize_t as3677_als_group2_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return as3677_als_group_n_store(2, dev, attr, buf, size);
}
static ssize_t as3677_als_group3_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return as3677_als_group_n_show(3, dev, attr, buf);
}

static ssize_t as3677_als_group3_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return as3677_als_group_n_store(3, dev, attr, buf, size);
}

#define DELTA(A, B) (((A) < (B)) ? ((B)-(A)) : ((A)-(B)))
static u16 dim_speed2time[8] = { 1000, 1900, 500, 950, 100, 190, 50, 95 };

static ssize_t as3677_dim_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int dim_speed = (AS3677_READ_REG(AS3677_REG_Pwm_control) >> 3) & 0x7;
	u16 dim_time = dim_speed2time[dim_speed];
	snprintf(buf, PAGE_SIZE, "%d (50,95,190,500,950,1900)\n", dim_time);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_dim_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int dim_time, i, curr_best = 3, delta;
	i = sscanf(buf, "%d", &dim_time);
	if (i != 1 || dim_time > 10000)
		return -EINVAL;
	delta = 65535;
	for (i = 0; i < ARRAY_SIZE(dim_speed2time) ; i++) {
		/* Omit settings where there exist better ones */
		if (i == 0 || i == 4)
			continue;
		if (DELTA(dim_time, dim_speed2time[i]) < delta) {
			delta = DELTA(dim_time, dim_speed2time[i]);
			curr_best = i;
		}
	}
	AS3677_LOCK();
	AS3677_MODIFY_REG(AS3677_REG_Pwm_control, 0x38, curr_best<<3);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_dim_start_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	if (data->dimming_in_progress)
		snprintf(buf, PAGE_SIZE, "%d (in progress)\n",
				data->dimming_in_progress);
	else
		snprintf(buf, PAGE_SIZE, "0 (ready)\n");
	return strnlen(buf, PAGE_SIZE);
}

static void as3677_clear_dimming(struct as3677_data *data)
{
	int i;
	for (i = 0; i < AS3677_NUM_LEDS; i++) {
		struct as3677_led *led = data->leds + i;
		led->dim_brightness = -1;
		led->dim_value = -1;
	}
	if (data->dimming_in_progress == DIM_PATTERN)
		data->dimming_in_progress = DIM_NONE;
}

static int as3677_get_dimming_direction(struct as3677_data *data)
{
	int i, dir = 0, err = 0;
	for (i = 0; i < AS3677_NUM_LEDS; i++) {
		struct as3677_led *led = data->leds + i;
		if (led->dim_brightness == -1)
			continue;
		if (AS3677_READ_REG(led->reg) == led->dim_value)
			continue;
		switch (dir) {
		case 0:
			if (AS3677_READ_REG(led->reg)
					> led->dim_value)
				dir = -1;
			else
				dir = 1;
			break;
		case 1:
			if (AS3677_READ_REG(led->reg)
					> led->dim_value)
				err = -EINVAL;
			break;
		case -1:
			if (AS3677_READ_REG(led->reg)
					< led->dim_value)
				err = -EINVAL;
			break;
		}
	}
	if (err)
		return err;
	return dir;
}

static int as3677_check_cross_dimming(struct device *dev,
struct as3677_data *data, int dir)
{
	int i, use_group;
	for (i = 0; i < 3; i++) {
		struct as3677_led *led = data->leds + i;
		int adder1, adder2;
		if (led->dim_brightness == -1 || led[3].dim_brightness == -1)
			continue; /* one curr and one rgb use the same adder */

		use_group = AS3677_READ_REG(led->mode_reg + 0x93)
			& (0x3<<led->mode_shift);
			if (use_group) {
				dev_warn(dev, "Led %s uses ALS, cannot be "
					      "dimmed together with led %s\n",
					      led->name, led[3].name);
				return -EINVAL;
			}
		if (dir == 1)
			adder1 = AS3677_READ_REG(led->reg);
		else
			adder1 = led->dim_value;

		if (dir == 1)
			adder2 = AS3677_READ_REG(led[3].reg);
		else
			adder2 = led[3].dim_value;

		if (adder1 != adder2)
			return -EINVAL;

		use_group = AS3677_READ_REG(led[3].mode_reg + 0x93)
			& (0x3<<led->mode_shift);
		if (use_group) {
			dev_warn(dev, "Led %s uses ALS, cannot be "
					"dimmed together with led %s\n",
					led[3].name, led->name);
			return -EINVAL;
		}
	}
	return 0;
}

static u8 as3677_als_corrected_adder(u8 value, struct as3677_data *data,
		struct as3677_led *led)
{
	int gid = AS3677_READ_REG(led->mode_reg + 0x93) &
		(0x3<<led->mode_shift);
	int amb_result;
	u8 group_reg = AS3677_REG_ALS_group_1_Y0 + (gid - 1) * 6;
	s32 y0, y3, x1, x2, x3, out;
	s32 k1, k2;

	if (gid == 0)
		return value;

	amb_result = i2c_smbus_read_byte_data(data->client,
			AS3677_REG_ALS_result);

	y0 = AS3677_READ_REG(group_reg);
	y3 = AS3677_READ_REG(group_reg + 1);
	x1 = AS3677_READ_REG(group_reg + 2);
	k1 = (s8) AS3677_READ_REG(group_reg + 3);
	x2 = AS3677_READ_REG(group_reg + 4);
	k2 = (s8) AS3677_READ_REG(group_reg + 5);
	if (gid == 3)
		x3 = x2 + (y3 - (x2 - x1) * k1 - y0) * 32 / k2;
	else
		x3 = x2 + (y3 - (x2 - x1) * k1 / 32 - y0) * 32 / k2;

	if (amb_result < x1) /* first part */
		out = y0;
	else if (amb_result < x2) /* second part */ {
		if (gid == 3)
			out = k1 * (amb_result - x1) + y0;
		else
			out = k1 * (amb_result - x1) / 32 + y0;
	} else if (amb_result < x3) /* third part */
		out = k1 * (x2 - x1) / 32 + k2 * (amb_result - x2) / 32 + y0;
	else /* fourth part */
		out = y3;

	return value * out / 256;
}

static ssize_t as3677_dim_start_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int dim_start, i, dir, err = 0, dim_speed;
	u16 dim_time;

	i = sscanf(buf, "%d", &dim_start);
	if (i != 1)
		return -EINVAL;
	if (dim_start != 1 && dim_start != 0)
		return -EINVAL;

	AS3677_LOCK();

	if (!dim_start)
		goto exit_clr;

	if (data->dimming_in_progress) {
		switch (data->dimming_in_progress) {
		case DIM_PATTERN:
			dev_warn(dev, "pattern_use_softdim is set,"
					" dimming not possible\n");
			err = -EBUSY;
		break;
		case DIM_UPDOWN:
			dev_warn(dev, "Dimming in progress, try again later\n");
			err = -EAGAIN;
		break;
		case DIM_ALS:
			dev_warn(dev, "ALS is using PWM,"
					" dimming not possible\n");
			err = -EBUSY;
		break;
		default:
			dev_warn(dev, "Unknown dimming\n");
			err = -EBUSY;
		}
		goto exit;
	}

	data->dimming_in_progress = DIM_UPDOWN;

	/* Before continuing remove dimming request which don't require
	   dimming */
	for (i = 0; i < AS3677_NUM_LEDS; i++) {
		struct as3677_led *led = data->leds + i;
		if (AS3677_READ_REG(led->reg) == led->dim_value)
			led->dim_brightness = -1;
	}

	dir = as3677_get_dimming_direction(data);
	if (dir != 1 && dir != -1) {
		dev_warn(dev, "Inconsistent dimming directions, "
				"resetting dim values\n");
		goto exit_clr;
	}

	err = as3677_check_cross_dimming(dev, data, dir);
	if (err) {
		dev_warn(dev, "Inconsistent cross dimming, "
				"each pair curr1/rgb1, curr2/rgb2 and "
				"curr6/rgb3 must either dim up starting from"
				" the same value or dim down to the same "
				"value!\n");
		goto exit_clr;
	}

	dim_speed = (AS3677_READ_REG(AS3677_REG_Pwm_control) >> 3) & 0x7;
	dim_time = dim_speed2time[dim_speed];
	if (dir == -1) { /* down dimming */
		/* To be safe disable adder and pwm, should be disabled already
		   anyway */
		AS3677_MODIFY_REG(AS3677_REG_Pwm_control, 0x6, 0x0);
		AS3677_WRITE_REG(AS3677_REG_Pwm_code, 0xff);

		for (i = 0; i < AS3677_NUM_LEDS; i++) {
			struct as3677_led *led = data->leds + i;
			if (led->dim_brightness == -1)
				continue;
			/* turn off adder */
			AS3677_MODIFY_REG(led->adder_on_reg,
					1<<led->adder_on_shift, 0);
			AS3677_WRITE_REG(led->adder_reg,
					as3677_als_corrected_adder(
					led->dim_value, data, led));
			AS3677_WRITE_REG(led->reg, AS3677_READ_REG(led->reg) -
					led->dim_value);
			/* FIXME Here the LED is at a lower value until the next
			   i2c command hits. Alternatively we could do something
			   like pumping the value over or having one i2c request
			   for the two registers and good i2c driver */
			AS3677_MODIFY_REG(led->adder_on_reg, 0,
					1<<led->adder_on_shift);
			/* Switch led to pwm */
			as3677_switch_led(data, led, 2);
		}

		/* Enable down dimming */
		AS3677_MODIFY_REG(AS3677_REG_Pwm_control, 0x6, 0x4);
		/* Wait for dimming to be finished */
		schedule_delayed_work(&data->dim_work,
				msecs_to_jiffies(dim_time));
	} else { /* up dimming */
		/* To be safe disable adder and pwm, should be disabled already
		   anyway */
		AS3677_MODIFY_REG(AS3677_REG_Pwm_control, 0x6, 0x0);
		AS3677_WRITE_REG(AS3677_REG_Pwm_code, 0x00);

		for (i = 0; i < AS3677_NUM_LEDS; i++) {
			struct as3677_led *led = data->leds + i;
			if (led->dim_brightness == -1)
				continue;
			AS3677_MODIFY_REG(led->adder_on_reg,
					1<<led->adder_on_shift, 0);
			AS3677_WRITE_REG(led->adder_reg,
				as3677_als_corrected_adder(
				AS3677_READ_REG(led->reg), data, led));
			/* Switch to pwm */
			as3677_switch_led(data, led, 2);
			/* FIXME Here the LED is for a short while off until
			   the next i2c command hits */
			/* Alternatively we could do something like pumping the
			   value over */
			/* or having one i2c request for the two registers and
			   good i2c driver */
			AS3677_MODIFY_REG(led->adder_on_reg, 0,
					1<<led->adder_on_shift);
			AS3677_WRITE_REG(led->reg, led->dim_value -
					AS3677_READ_REG(led->reg));
			as3677_check_DCDC(data);
		}
		/* Now start the updimming */
		AS3677_MODIFY_REG(AS3677_REG_Pwm_control, 0x6, 0x2);
		/* Wait for dimming to be finished */
		schedule_delayed_work(&data->dim_work,
				msecs_to_jiffies(dim_time));
	}
	goto exit;
exit_clr:
	as3677_clear_dimming(data);
	if (data->dimming_in_progress == DIM_UPDOWN)
		data->dimming_in_progress = DIM_NONE;
exit:
	AS3677_UNLOCK();
	if (err)
		return err;
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_use_dls_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	int use_dls = AS3677_READ_REG(led->dls_mode_reg);

	use_dls >>= led->dls_mode_shift;
	use_dls &= 1;
	snprintf(buf, PAGE_SIZE, "%d\n", use_dls);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_use_dls_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	int i;
	u8 use_dls;
	i = sscanf(buf, "%hhu", &use_dls);
	if (i != 1)
		return -EINVAL;
	use_dls = !!use_dls;
	AS3677_LOCK();
	AS3677_MODIFY_REG(led->dls_mode_reg,
			1 << led->dls_mode_shift,
			use_dls << led->dls_mode_shift);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_dls_pin_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	int dls_pin = AS3677_READ_REG(led->dls_mode_reg);

	if (!led->dls2shift) {
		snprintf(buf, PAGE_SIZE,
			"1 (fixed, dls2 pin not available for this source)\n");
		return strnlen(buf, PAGE_SIZE);
	}

	dls_pin >>= led->dls2shift;
	dls_pin &= 1;
	snprintf(buf, PAGE_SIZE, "%d (1/2), for curr2 and curr6 dls2 pin can "
				"be used, all others use dls1\n", dls_pin+1);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_dls_pin_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	int i;
	u8 dls_pin;
	if (!led->dls2shift)
		return -EINVAL;
	i = sscanf(buf, "%hhu", &dls_pin);
	if (i != 1 || ((dls_pin != 1) && (dls_pin != 2)))
		return -EINVAL;
	dls_pin--;

	AS3677_LOCK();
	AS3677_MODIFY_REG(led->dls_mode_reg,
			1 << led->dls2shift,
			dls_pin << led->dls2shift);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}


static ssize_t as3677_als_group_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	int als_group = AS3677_READ_REG(led->mode_reg + 0x93);

	als_group >>= led->mode_shift;
	als_group &= 0x3;
	snprintf(buf, PAGE_SIZE, "%d (0=no ambient control, N = groupN)\n"
				, als_group);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_group_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev->parent);
	struct as3677_led *led = ldev_to_led(dev_get_drvdata(dev));
	int i;
	u8 als_group;
	i = sscanf(buf, "%hhu", &als_group);
	if (i != 1 || als_group > 3)
		return -EINVAL;
	AS3677_LOCK();
	AS3677_MODIFY_REG(led->mode_reg + 0x93,
			3 << led->mode_shift,
			als_group << led->mode_shift);
	as3677_retrigger_led(data, led);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_on_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int als_on = AS3677_READ_REG(AS3677_REG_ALS_control) & 1;
	snprintf(buf, PAGE_SIZE, "%d\n", als_on);
	return strnlen(buf, PAGE_SIZE);
}

static void as3677_switch_als(struct as3677_data *data, int als_on)
{
	if ((AS3677_READ_REG(AS3677_REG_ALS_control) & 1) == als_on)
		return;

	AS3677_WRITE_REG(AS3677_REG_LDO_Voltage, 0xff);
	AS3677_MODIFY_REG(AS3677_REG_Control, 1, als_on);
	AS3677_MODIFY_REG(AS3677_REG_ALS_control, 1, als_on);
	AS3677_MODIFY_REG(AS3677_REG_GPIO_control, 0xf0,
			als_on ? 0x10 : 0x00);
	AS3677_MODIFY_REG(AS3677_REG_GPIO_control_3, 0x0f,
			als_on ? 0x01 : 0x00);
	AS3677_MODIFY_REG(AS3677_REG_GPIO_output_2, 0x06,
			als_on ? 0x06 : 0x00);
}

static ssize_t as3677_als_on_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int als_on, i;
	i = sscanf(buf, "%d", &als_on);
	if (i != 1)
		return -EINVAL;
	if (als_on != 1 && als_on != 0)
		return -EINVAL;
	AS3677_LOCK();
	as3677_switch_als(data, als_on);
	AS3677_UNLOCK();
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_use_fsm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%d\n", (data->curr_state >= 0));
	return strnlen(buf, PAGE_SIZE);
}

static int as3677_result2lux(u8 result, struct as3677_als_fsm_state *s)
{
	u32 lx;
	u8 amb_gain = 1 << s->gain; /* have gain as amb_gain/4 */

	/* multiply always before doing divisions to preserve precision.
	   Overflows should not happen with the values since values go
	   up to 200.000 lx and amb_gain is a maximum of 4 */
	lx = (result * s->sensor2lux * 4 / amb_gain
			- s->sensor2lux * s->offset * 4);
#if 0
	printk(KERN_INFO "r2l %d (sl=%d, o=%d g=%d/4) -> %d\n"
			, result, s->sensor2lux, s->offset, amb_gain, lx);
#endif
	return lx;
}
static int as3677_lux2result(u32 lux, struct as3677_als_fsm_state *s)
{
	int result;
	u8 amb_gain = 1 << s->gain; /* have gain as amb_gain/4 */

	/* multiply always before doing divisions to preserve precision.
	   Overflows should not happen with the values since values go
	   up to 200.000 lx and amb_gain is a maximum of 4 */

	result = (lux + s->sensor2lux * s->offset * 4)
		* amb_gain
		/ 4
		/ s->sensor2lux;
#if 0
	printk(KERN_INFO "l2r %d (sl=%d, o=%d g=%d/4) -> %d\n"
			, lux, s->sensor2lux, s->offset, amb_gain, result);
#endif
	return result;
}
static ssize_t as3677_als_lx_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int i;
	s32 als_lx;
	u32 adc_result;
	s32 amb_result = i2c_smbus_read_byte_data(data->client,
			AS3677_REG_ALS_result);
	struct as3677_als_fsm_state *s = data->fsm + data->curr_state;

	/* Start measuring GPIO2/LIGHT */
	AS3677_WRITE_REG(AS3677_REG_ADC_control, 0x82);
	for (i = 0; i < 10; i++) {
		adc_result = i2c_smbus_read_byte_data(data->client,
				AS3677_REG_ADC_MSB_result);
		if (!(adc_result & 0x80))
			break;
		udelay(10);
	}
	adc_result <<= 3;
	adc_result |= i2c_smbus_read_byte_data(data->client,
			AS3677_REG_ADC_LSB_result);

	if (data->curr_state == -1 ||
			!(AS3677_READ_REG(AS3677_REG_ALS_control) & 1)) {
		snprintf(buf, PAGE_SIZE,
				"-1 lx (amb_result=n/a,"
				"offset=n/a,amb_gain=n/a,adc=%d\n", adc_result);
		return strnlen(buf, PAGE_SIZE);
	}

	als_lx = as3677_result2lux(amb_result, s);

	snprintf(buf, PAGE_SIZE, "%d lx (amb_result=%d,offset=%d,amb_gain%d/4,"
			"sensor2lux=%d,adc=%d)\n",
			als_lx, amb_result, s->offset, s->gain, s->sensor2lux,
			adc_result);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_lx_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return -EINVAL;
}

static int as3677_set_state(struct as3677_data *data,
		struct as3677_als_fsm_state *s)
{
	int i;
	u8 ss = 0;
	AS3677_WRITE_REG(AS3677_REG_ALS_range_high_threshold, s->range_high);
	AS3677_WRITE_REG(AS3677_REG_ALS_range_low_threshold, s->range_low);
	AS3677_WRITE_REG(AS3677_REG_ALS_offset, s->offset);
	AS3677_MODIFY_REG(AS3677_REG_ALS_control, 0x06, s->gain << 1);
	AS3677_MODIFY_REG(AS3677_REG_GPIO_output_2, 0x06, s->gpio << 1);

	/* We should wait 1 ms for light sensor to adapt.
	   Jiffy is usually way above 1ms*/
	schedule_timeout_uninterruptible(1);

	/* Write group values */
	for (i = 0; i < 3; i++) {
		struct as3677_als_group *g = s->groups + i;
		u8 group_reg = AS3677_REG_ALS_group_1_Y0 +
			(g->gn - 1) * 6;

		if (g->gn == 0)
			continue;
		if (g->ss)
			ss = g->ss;

		AS3677_WRITE_REG(group_reg + 0, g->y0);
		AS3677_WRITE_REG(group_reg + 1, g->y3);
		AS3677_WRITE_REG(group_reg + 2, g->x1);
		AS3677_WRITE_REG(group_reg + 3, g->k1);
		AS3677_WRITE_REG(group_reg + 4, g->x2);
		AS3677_WRITE_REG(group_reg + 5, g->k2);
	}

	if (ss)
		AS3677_WRITE_REG(AS3677_REG_Pwm_code, ss);

	/* Retrigger leds going to PWM */
	for (i = 0; i < 3; i++) {
		struct as3677_als_group *g = s->groups + i;
		if (g->gn == 0)
			continue;
		if (g->ss && !data->ss[g->gn]) { /* retrigger the leds going to
		    PWM, the others have to be done after setting amb_keep */
			data->ss[g->gn] = g->ss;
			as3677_retrigger_group_leds(data, g->gn);
		}
	}

	/* let ALS run again, i.e. set amb_keep */
	AS3677_MODIFY_REG(AS3677_REG_ALS_control, 0x08, 0x00);

	/* Retrigger leds leaving PWM mode */
	for (i = 0; i < 3; i++) {
		struct as3677_als_group *g = s->groups + i;
		if (g->gn == 0)
			continue;
		if (!g->ss && data->ss[g->gn]) {
			/* retrigger the leds leaving PWM */
			data->ss[g->gn] = g->ss;
			as3677_retrigger_group_leds(data, g->gn);
		}
	}
	return 0;
}

static void as3677_dim_work(struct work_struct *work)
{
	struct as3677_data *data =
		container_of(work, struct as3677_data, dim_work.work);

	int i, dir;

	dir = as3677_get_dimming_direction(data);

	AS3677_LOCK();
	if (dir == -1) { /* down dimming */
		for (i = 0; i < AS3677_NUM_LEDS; i++) {
			struct as3677_led *led = data->leds + i;
			if (led->dim_brightness == -1)
				continue;
			AS3677_WRITE_REG(led->reg, led->dim_value);
			AS3677_MODIFY_REG(led->adder_on_reg,
					1<<led->adder_on_shift, 0);
			/* Here the LED is turned off until the next i2c command
			   hits. Alternatively we could do something like
			   pumping the value over. Switch from pwm to desired
			   mode */
			as3677_switch_led(data, led, led->dim_value != 0);
			led->ldev.brightness = led->dim_brightness;
		}
		/* Turn off Pwm */
		AS3677_MODIFY_REG(AS3677_REG_Pwm_control, 0x6, 0x0);
		/* Possibly DCDC can be switched off now */
		as3677_check_DCDC(data);
	} else { /* up dimming */
		for (i = 0; i < AS3677_NUM_LEDS; i++) {
			struct as3677_led *led = data->leds + i;
			if (led->dim_brightness == -1)
				continue;
			/* Switch off the adder */
			AS3677_MODIFY_REG(led->adder_on_reg,
					1<<led->adder_on_shift, 0);
			/* Here the LED is at a lower value until the next
			   command hits. Alternatively we could do something
			   like pumping the value over */
			AS3677_WRITE_REG(led->reg, led->dim_value);
			/* Switch to desired mode */
			as3677_switch_led(data, led, 1);
			led->ldev.brightness = led->dim_brightness;
		}
		/* Turn off Pwm */
		AS3677_MODIFY_REG(AS3677_REG_Pwm_control, 0x6, 0x0);
	}
	as3677_clear_dimming(data);
	data->dimming_in_progress = DIM_NONE;
	AS3677_UNLOCK();
}

static void as3677_fsm_transition(struct work_struct *work)
{
	struct as3677_data *data =
		container_of(work, struct as3677_data, isr_work);
	struct device *dev = &data->client->dev;
	u8 ireg, als_res;
	int nState, lx, new_res;

	nState = data->curr_state;

	AS3677_LOCK();
	ireg = i2c_smbus_read_byte_data(data->client,
			AS3677_REG_Interrupt_Status);

	if (data->curr_state == -1) {
		AS3677_WRITE_REG(AS3677_REG_ALS_range_high_threshold, 0);
		AS3677_WRITE_REG(AS3677_REG_ALS_range_low_threshold, 0);
		dev_err(dev, "Unexptected as3677 interrupt,"
				"ireg was %x\n", ireg);
		goto exit;
	}

	als_res = i2c_smbus_read_byte_data(data->client,
			AS3677_REG_ALS_result);
	if (ireg & 1) /* high interrupt */
		nState++;
	else if (ireg & 2)
		nState--;
	else
		dev_err(dev, "AS3677 no interrupt\n");

	if (nState < 0) {
		dev_err(dev, "AS3677 already lowest state\n");
		nState = 0;
		goto exit;
	}
	if (nState > data->fsm_num_states) {
		nState = data->fsm_num_states - 1;
		dev_err(dev, "AS3677 already highest state\n");
		goto exit;
	}

	/* Calculate the new expected result to be stored into AS3677.
	   Filter will then start with this value. This avoids immediately
	   changing back. */
	lx = as3677_result2lux(als_res, data->fsm + data->curr_state);
	new_res = as3677_lux2result(lx, data->fsm + nState);

	/* disable interrupts inside AS3677 */
	AS3677_WRITE_REG(AS3677_REG_ALS_range_high_threshold, 0);
	AS3677_WRITE_REG(AS3677_REG_ALS_range_low_threshold, 0);

	/* Read interrupt register one more time to clear */
	i2c_smbus_read_byte_data(data->client,
			AS3677_REG_Interrupt_Status);


	dev_info(dev, "transition %d(%d) --(%d lx)>> %d(%d)\n",
			data->curr_state,
			als_res,
			lx,
			nState,
			new_res);

	/* set amb_keep to stop ALS operation */
	AS3677_MODIFY_REG(AS3677_REG_ALS_control, 0x08, 0x08);
	AS3677_WRITE_REG(AS3677_REG_ALS_result, new_res);
	data->curr_state = nState;

	/* set_state will reset amb_keep */
	as3677_set_state(data, data->fsm + nState);

exit:
	AS3677_UNLOCK();
	enable_irq(data->client->irq);
}

static ssize_t as3677_als_use_fsm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int als_use_fsm, i;
	i = sscanf(buf, "%d", &als_use_fsm);
	if (i != 1 && i != 0)
		return -EINVAL;

	if (data->curr_state == ((als_use_fsm) ? (0) : (-1)))
		goto exit;
	/* State changed */
	if (als_use_fsm) {
		if (data->fsm == NULL) {
			dev_warn(dev,
				"starting fsm not possible without fsm\n");
			return -EINVAL;
		}
		AS3677_LOCK();
		as3677_set_state(data, data->fsm);
		data->curr_state = 0;
		AS3677_UNLOCK();
		enable_irq(data->client->irq);
	} else {
		AS3677_LOCK();
		disable_irq(data->client->irq);
		data->curr_state = -1;
		/* Disable both interrupts */
		AS3677_WRITE_REG(AS3677_REG_ALS_range_high_threshold, 0);
		AS3677_WRITE_REG(AS3677_REG_ALS_range_low_threshold, 0);
		AS3677_UNLOCK();
	}

exit:
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_als_fsm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	ssize_t act = 0, rem = PAGE_SIZE;
	int i, j;
	struct as3677_als_fsm_state *s;
	struct as3677_als_group *g;
	/* no lock here, it is just informational to not have the lock
	   allocated for too long */
	for (i = 0; i < data->fsm_num_states; i++) {
		s = data->fsm + i;
		MSNPRINTF("%u,%u,%u,%d,%u,%u",
				s->range_high, s->range_low, s->gpio,
				s->sensor2lux, s->gain, s->offset);
		for (j = 0; j < 3; j++) {
			g = s->groups + j;
			if (!g->gn)
				break;
			MSNPRINTF(":%u:%u,%u,%d,%u,%d,%u,%u",
					g->gn, g->y0, g->x1, g->k1,
					g->x2, g->k2, g->y3, g->ss);
		}
		MSNPRINTF("\n");
	}
exit:
	return PAGE_SIZE - rem;
}

static int as3677_als_extract_state(struct device *dev,
		struct as3677_als_fsm_state *s,
		const char *buf)
{
	int i, num_groups, ret;
	u8 *rh = &s->range_high, *rl = &s->range_low, *gp = &s->gpio;
	u8 *ga = &s->gain, *o = &s->offset;
	s32 *sg = &s->sensor2lux;
	struct as3677_als_group *x = s->groups, *y = s->groups + 1,
				*z = s->groups + 2;

	i = sscanf(buf,
		" %3hhu, %3hhu, %3hhu,   %4d, %3hhu, %3hhu"
		":%3hhu: %3hhu, %3hhu, %3hhd, %3hhu, %3hhd, %3hhu, %3hhu"
		":%3hhu: %3hhu, %3hhu, %3hhd, %3hhu, %3hhd, %3hhu, %3hhu"
		":%3hhu: %3hhu, %3hhu, %3hhd, %3hhu, %3hhd, %3hhu, %3hhu"
		, rh, rl, gp, sg, ga, o
		, &x->gn, &x->y0, &x->x1, &x->k1, &x->x2, &x->k2, &x->y3, &x->ss
		, &y->gn, &y->y0, &y->x1, &y->k1, &y->x2, &y->k2, &y->y3, &y->ss
		, &z->gn, &z->y0, &z->x1, &z->k1, &z->x2, &z->k2, &z->y3, &z->ss
		);
	if (s->gpio > 3) {
		dev_warn(dev, "gp value out of range: %d\n", s->gpio);
		return -EINVAL;
	}
	switch (i) {
	case 14:
		num_groups = 1;
		break;
	case 22:
		num_groups = 2;
		break;
	case 30:
		num_groups = 3;
		break;
	default:
		dev_warn(dev, "Invalid FSM line, has to have"
				"14, 22 or 29 values (actual %d)\n"
				"Recall no spaces before separators allowed\n"
				, i);
		return -EINVAL;
	}

	for (i = 0; i < num_groups; i++) {
#if AS3677_ENABLE_GROUP_CHECK
		ret = as3677_als_group_check(dev, s->groups + i);
		if (ret < 0) {
			dev_warn(dev, "error was in group %d\n",
					s->groups[i].gn);
			return ret;
		}
#endif
		if (s->ss && s->groups[i].ss && s->ss != s->groups[i].ss) {
			ret = -EINVAL;
			dev_warn(dev, "All groups inside one state must have "
					"same ss value or zero");
		}
		if (s->groups[i].ss)
			s->ss = s->groups[i].ss;
	}

	return 0;
}

static int extract_line(const char *buf, int max, char *str)
{
	const char *b;
	b = strchr(buf, '\n');
	if (b == NULL)
		return 0;
	if ((b - buf) > max)
		return 0;
	memcpy(str, buf, b - buf);
	str[b - buf] = '\0';
	return b - buf;
}

static ssize_t as3677_als_fsm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int i;
	int line;
	struct as3677_als_fsm_state *s = NULL;
	const char *b;
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int num_states;
	char str[200];
	bool usesPWM = false;

	if (data->curr_state != -1) {
		dev_warn(dev, "setting fsm not possible while in use\n");
		return -EACCES;
	}

	/* Count lines of buffer */
	b = buf - 1;
	num_states = 0;
	do {
		b++;
		if (*b != '#' && *b != '\0')
			num_states++;
		b = strchr(b, '\n');
	} while (b);

	s = kzalloc(sizeof(struct as3677_als_fsm_state) * num_states, GFP_USER);

	b = buf;
	line = 0;
	i = 0;
	while (extract_line(b, 200, str)) {
		line++;
		b = strchr(b, '\n') + 1;
		if (str[0] == '#')
			continue;
		ret = as3677_als_extract_state(dev, s + i, str);
		if (ret != 0) {
			dev_warn(dev, "error was in state %d in line %d\n",
					i, line);
			goto exit;
		}
		if (s[i].ss)
			usesPWM = true;
		i++;
	}
	if (i != num_states) {
		dev_warn(dev, "Could not parse all states %d!=%d\n",
				i, num_states);
		ret = -EINVAL;
		goto exit;
	}

	/* Have proper start and end states */
	s[0].range_low = 0;
	s[num_states - 1].range_high = 0;

	/* check state gains and sensor2lx values to avoid interrupt jamming */
	for  (i = 1; i < num_states; i++) {
		if (as3677_lux2result(
					as3677_result2lux(s[i - 1].range_high,
						s + i - 1)
					, s + i)
				< s[i].range_low) {
			dev_warn(dev, "Transition from state %d to %d would "
					"cause immediate return\n", i - 1, i);
			ret = -EINVAL;
			goto exit;
		}
		if (as3677_lux2result(
					as3677_result2lux(s[i].range_low, s + i)
					, s + i - 1)
				> s[i - 1].range_high) {
			dev_warn(dev, "Transition from state %d to %d would "
					"cause immediate return\n", i, i - 1);
			ret = -EINVAL;
			goto exit;
		}
	}
	AS3677_LOCK();
	if (usesPWM && !(data->dimming_in_progress == DIM_NONE ||
			 data->dimming_in_progress == DIM_ALS)) {
		AS3677_UNLOCK();
		dev_warn(dev, "FSM would need Pwm/dimmer but it is in use\n");
		ret = -EBUSY;
		goto exit;

	}
	if (!usesPWM && (data->dimming_in_progress == DIM_ALS))
		data->dimming_in_progress = DIM_NONE;

	if (usesPWM) {
		data->dimming_in_progress = DIM_ALS;
		/* no up/down dimming */
		AS3677_WRITE_REG(AS3677_REG_Pwm_control, 0);
	}

	if (data->fsm != NULL)
		kfree(data->fsm);

	data->fsm = s;
	data->fsm_num_states = num_states;
	AS3677_UNLOCK();
	s = NULL;
exit:
	kfree(s);
	return ret;
}

static ssize_t as3677_als_sample_fsm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t act = 0, rem = PAGE_SIZE;

	MSNPRINTF(" 200, 0,1,  2,3, 0:  1:  1, 10,  1,128,  3,  5, 0\n");
	MSNPRINTF(" 200,17,1,  2,0, 0:  1:  5, 10,  1,128,  3, 10, 0\n");
	MSNPRINTF(" 200,17,2, 19,0, 0:  1: 10, 10,  3,128,  3, 25, 0\n");
	MSNPRINTF("   0,17,3,195,0, 0:  1: 25, 10, 30,128, 38,250, 0\n");
	MSNPRINTF("#rh,rl,gp,sgm,ga, o"
			" [:gn:y0, x1, k1, x2, k2, y3, ss]{1-3}\n");
	MSNPRINTF("#|\n");
	MSNPRINTF("#rh: range high value, next line used if higher\n");
	MSNPRINTF("#rl: range low value, next line used if lower\n");
	MSNPRINTF("#gp: gpio2 and gpio3 setting (gain of sensor)\n");
	MSNPRINTF("#sgm: sensor to lux conversion factor, e.g.\n");
	MSNPRINTF("#    0.00244 / (0.57E-06 * 2200)\n");
	MSNPRINTF("#ga: gain - 0=1/4, 1=1/2, 2=1, 3=2\n");
	MSNPRINTF("# o: amb_offset\n");
	MSNPRINTF("#gn: number of als group following after (1,2,3)\n");
	MSNPRINTF("#y0,x1,k1,x2,k2,y3, ss values for a proper als group\n");
	MSNPRINTF("#with ss: step size reduction using PWM: 1,8,16\n");
	MSNPRINTF("#at least one group needs to follow, at most 3.\n");
	MSNPRINTF("#all values are u8 type except k1 and k2 which are s8\n");

exit:
	return PAGE_SIZE - rem;
}

static ssize_t as3677_als_sample_fsm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	return -EINVAL;
}

#if 0
static ssize_t as3677_foo_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int foo = 42;
	snprintf(buf, PAGE_SIZE, "%d\n", foo);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t as3677_foo_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct as3677_data *data = dev_get_drvdata(dev);
	int foo, i;
	int reg, reset, set;
	i = sscanf(buf, "%d", &foo);
	if (i != 1)
		return -EINVAL;
	AS3677_MODIFY_REG(reg, reset, set);
	return strnlen(buf, PAGE_SIZE);
}
#endif
#define AS3677_ATTR(_name)  \
	__ATTR(_name, 0644, as3677_##_name##_show, as3677_##_name##_store)
static struct device_attribute as3677_attributes[] = {
	AS3677_ATTR(debug),
	AS3677_ATTR(dim_start),
	AS3677_ATTR(dim_time),
	AS3677_ATTR(pattern_data),
	AS3677_ATTR(pattern_duration_secs),
	AS3677_ATTR(pattern_use_color),
	AS3677_ATTR(pattern_delay),
	AS3677_ATTR(pattern_use_softdim),
	AS3677_ATTR(als_filter_up_speed_mHz),
	AS3677_ATTR(als_filter_down_speed_mHz),
	AS3677_ATTR(als_fsm),
	AS3677_ATTR(als_group1),
	AS3677_ATTR(als_group2),
	AS3677_ATTR(als_group3),
	AS3677_ATTR(als_lx),
	AS3677_ATTR(als_on),
	AS3677_ATTR(als_sample_fsm),
	AS3677_ATTR(als_use_fsm),
	__ATTR_NULL
};

static struct device_attribute as3677_led_attributes[] = {
	AS3677_ATTR(als_group),
	AS3677_ATTR(dim_brightness),
	AS3677_ATTR(dls_pin),
	AS3677_ATTR(use_dls),
	AS3677_ATTR(use_pattern),
	__ATTR_NULL
};

static int as3677_configure(struct i2c_client *client,
		struct as3677_data *data, struct as3677_platform_data *pdata)
{
	int i, err = 0;

	AS3677_MODIFY_REG(AS3677_REG_DCDC_control1, 0x18,
			pdata->step_up_vmax << 3);

	AS3677_MODIFY_REG(AS3677_REG_DLS_mode_control1, 0x80,
			pdata->dls_analog << 7);

	for (i = 0; i < 6; i++) {
		struct as3677_led *led = &data->leds[i];
		if (pdata != NULL) {
			struct as3677_platform_led *pled = &pdata->leds[i];
			led->pled = pled;
			if (pled->name && strlen(pled->name) > 0)
				led->name = pled->name;
			if (pled->on_charge_pump) {
				AS3677_MODIFY_REG(led->dls_mode_reg - 0x32,
						1 << led->dls_mode_shift,
						1 << led->dls_mode_shift);
			}
		}
		if (led->pled->max_current_uA > 25500)
			led->pled->max_current_uA = 25500;
		if (led->pled->startup_current_uA > led->pled->max_current_uA)
			led->pled->startup_current_uA =
				led->pled->max_current_uA;
		/* Round down max current to next step */
		led->pled->max_current_uA =
			led->pled->max_current_uA / 100 * 100;
		led->client = client;
		led->ldev.name = led->name;
		led->ldev.brightness = LED_OFF;
		led->ldev.brightness_set = as3677_set_led_brightness;
		led->ldev.blink_set = NULL;
		led->dim_brightness = -1;
		err = led_classdev_register(&client->dev, &led->ldev);
		if (err < 0) {
			dev_err(&client->dev,
					"couldn't register LED %s\n",
					led->name);
			goto exit;
		}
		err = device_add_attributes(led->ldev.dev,
				as3677_led_attributes);
		if (err < 0) {
			dev_err(&client->dev,
					"couldn't add attributes %s\n",
					led->name);
			goto exit;
		}
	}

	err = device_add_attributes(&client->dev, as3677_attributes);

	if (err)
		goto exit;

	for (i = 0; i < 6; i++) {
		struct as3677_led *led = &data->leds[i];
		if (led->pled->startup_current_uA) {
			as3677_set_brightness(data, led,
				led->pled->startup_current_uA * 255 /
				led->pled->max_current_uA);

		}
	}
	return 0;

exit:
	device_remove_attributes(&client->dev, as3677_attributes);
	if (i > 0)
		for (i = i - 1; i >= 0; i--)
			led_classdev_unregister(&data->leds[i].ldev);
	return err;
}

/*static irqreturn_t
as3677_isr(int this_irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct as3677_data *data = dev_get_drvdata(dev);

	disable_irq(this_irq);
	schedule_work(&data->isr_work);
	return IRQ_HANDLED;
}*/

static int as3677_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct as3677_data *data;
	struct as3677_platform_data *as3677_pdata = client->dev.platform_data;
	int id1, id2, i;
	int err = 0;

	if (!as3677_pdata)
		return -EIO;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	data = kzalloc(sizeof(*data), GFP_USER);
	if (!data)
		return -ENOMEM;

	/* initialize with meaningful data ( register names, etc.) */
	*data = as3677_default_data;

	dev_set_drvdata(&client->dev, data);

	data->client = client;
	mutex_init(&data->update_lock);

	id1 = i2c_smbus_read_byte_data(client, AS3677_REG_ASIC_ID1);
	id2 = i2c_smbus_read_byte_data(client, AS3677_REG_ASIC_ID2);
	if (id1 < 0) {
		err = id1;
		goto exit;
	}
	if (id2 < 0) {
		err = id2;
		goto exit;
	}
	if ((id1 != 0xa6) || (id2&0xf0) != 0x50) {
		err = -ENXIO;
		dev_err(&client->dev, "wrong chip detected, ids %x %x",
				id1, id2);
		goto exit;
	}
	dev_info(&client->dev, "AS3677 driver v1.4: detected AS3677"
			"compatible chip with ids %x %x\n",
			id1, id2);
	/* all voltages on */
	data->client = client;

	for (i = 0; i < ARRAY_SIZE(data->regs); i++) {
		if (data->regs[i].name)
			i2c_smbus_write_byte_data(client,
					i, data->regs[i].value);
	}

	for (i = 0; i < ARRAY_SIZE(data->regs); i++) {
		if (!data->regs[i].name)
			continue;
		data->regs[i].value = i2c_smbus_read_byte_data(client, i);
	}

	AS3677_WRITE_REG(AS3677_REG_Control, 0x00);

	INIT_WORK(&data->isr_work, as3677_fsm_transition);
	INIT_DELAYED_WORK(&data->dim_work, as3677_dim_work);
/*
	err = request_irq(client->irq, as3677_isr,
			IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_LOW,
			"as3677isr", &client->dev);
	disable_irq(client->irq);

	if (err) {
		dev_err(&client->dev, "Could not register isr!\n");
		return err;
	}
*/
	i2c_set_clientdata(client, data);


	err = as3677_configure(client, data, as3677_pdata);
exit:
	if (err) {
		kfree(data);
		/*free_irq(client->irq, &client->dev);*/
		dev_err(&client->dev, "could not configure %x", err);
		i2c_set_clientdata(client, NULL);
	}

	return err;
}

static int as3677_remove(struct i2c_client *client)
{
	struct as3677_data *data = i2c_get_clientdata(client);
	int i;
	for (i = 0; i < AS3677_NUM_LEDS; i++)
		led_classdev_unregister(&data->leds[i].ldev);

	kfree(data);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int __init as3677_init(void)
{
	return i2c_add_driver(&as3677_driver);
}

static void __exit as3677_exit(void)
{
	i2c_del_driver(&as3677_driver);
}

MODULE_AUTHOR("Ulrich Herrmann <ulrich.herrmann@austriamicrosystems.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AS3677 LED dimmer");

module_init(as3677_init);
module_exit(as3677_exit);

