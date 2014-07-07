/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/leds.h>
#include <linux/leds-pm8xxx.h>
#include <linux/msm_ssbi.h>
#include <linux/gpio_event.h>
#include <linux/gpio_keys.h>
#include <asm/mach-types.h>
#include <mach/msm_bus_board.h>
#include <mach/restart.h>
#include "devices.h"
#include "board-8960.h"

struct pm8xxx_gpio_init {
	unsigned			gpio;
	struct pm_gpio			config;
};

struct pm8xxx_mpp_init {
	unsigned			mpp;
	struct pm8xxx_mpp_config_data	config;
};

#define PM8XXX_GPIO_INIT(_gpio, _dir, _buf, _val, _pull, _vin, _out_strength, \
			_func, _inv, _disable) \
{ \
	.gpio	= PM8921_GPIO_PM_TO_SYS(_gpio), \
	.config	= { \
		.direction	= _dir, \
		.output_buffer	= _buf, \
		.output_value	= _val, \
		.pull		= _pull, \
		.vin_sel	= _vin, \
		.out_strength	= _out_strength, \
		.function	= _func, \
		.inv_int_pol	= _inv, \
		.disable_pin	= _disable, \
	} \
}

#define PM8XXX_MPP_INIT(_mpp, _type, _level, _control) \
{ \
	.mpp	= PM8921_MPP_PM_TO_SYS(_mpp), \
	.config	= { \
		.type		= PM8XXX_MPP_TYPE_##_type, \
		.level		= _level, \
		.control	= PM8XXX_MPP_##_control, \
	} \
}

#define PM8XXX_GPIO_DISABLE(_gpio) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, 0, 0, 0, PM_GPIO_VIN_S4, \
			 0, 0, 0, 1)

#define PM8XXX_GPIO_OUTPUT(_gpio, _val) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8XXX_GPIO_INPUT(_gpio, _pull) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_IN, PM_GPIO_OUT_BUF_CMOS, 0, \
			_pull, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_NO, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8XXX_GPIO_OUTPUT_FUNC(_gpio, _val, _func) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			PM_GPIO_STRENGTH_HIGH, \
			_func, 0, 0)

#define PM8XXX_GPIO_OUTPUT_VIN(_gpio, _val, _vin) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, _vin, \
			PM_GPIO_STRENGTH_HIGH, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

#define PM8XXX_GPIO_OUTPUT_STRENGTH(_gpio, _val, _out_strength) \
	PM8XXX_GPIO_INIT(_gpio, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, _val, \
			PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
			_out_strength, \
			PM_GPIO_FUNC_NORMAL, 0, 0)

/* Initial PM8921 GPIO configurations */
static struct pm8xxx_gpio_init pm8921_gpios[] __initdata = {

	PM8XXX_GPIO_INPUT(1, PM_GPIO_PULL_UP_30), /* CAMERA_A_FOCUS  */
	PM8XXX_GPIO_INPUT(2, PM_GPIO_PULL_UP_30), /* CAMERA_SNAPSHOT */

	PM8XXX_GPIO_OUTPUT(10, 0), /* KYPD_DRV2 */

	/* UIM1_RST_CONN (GPIO_27),
	   UIM1_CLK_MSM  (GPIO_29) and
	   UIM1_CLK_CONN (GPIO_30) is set by UIM driver in AMSS */

	/* CLK_MSM_FWD (GPIO_39) is set by QCT code */

	/* NC */
	PM8XXX_GPIO_DISABLE(3),
	PM8XXX_GPIO_DISABLE(4),
	PM8XXX_GPIO_DISABLE(5),
	PM8XXX_GPIO_DISABLE(6),
	PM8XXX_GPIO_DISABLE(7),
	PM8XXX_GPIO_DISABLE(8),
	PM8XXX_GPIO_DISABLE(9),
	PM8XXX_GPIO_DISABLE(11),
	PM8XXX_GPIO_DISABLE(12),
	PM8XXX_GPIO_DISABLE(13),
	PM8XXX_GPIO_DISABLE(14),
	PM8XXX_GPIO_DISABLE(15),
	PM8XXX_GPIO_DISABLE(16),
	PM8XXX_GPIO_DISABLE(17),
	PM8XXX_GPIO_DISABLE(18),
	PM8XXX_GPIO_DISABLE(23),
	PM8XXX_GPIO_DISABLE(24),
	PM8XXX_GPIO_DISABLE(25),
	PM8XXX_GPIO_DISABLE(28),
	PM8XXX_GPIO_DISABLE(31),
	PM8XXX_GPIO_DISABLE(32),
	PM8XXX_GPIO_DISABLE(35),
	PM8XXX_GPIO_DISABLE(37),
	PM8XXX_GPIO_DISABLE(40),
	PM8XXX_GPIO_DISABLE(41),
	PM8XXX_GPIO_DISABLE(44),

	/* OTHERS */
	/* Right speaker enable */
	PM8XXX_GPIO_OUTPUT_STRENGTH(19, 0, PM_GPIO_STRENGTH_MED),
	PM8XXX_GPIO_INPUT(20,	PM_GPIO_PULL_UP_30),	/* VOLUME_DOWN_KEY */
	PM8XXX_GPIO_INPUT(21,	PM_GPIO_PULL_UP_30),	/* VOLUME_UP_KEY */
	PM8XXX_GPIO_OUTPUT(22,  0),			/* RF_ID_EN */
	PM8XXX_GPIO_INPUT(26,	PM_GPIO_PULL_NO),	/* SD_CARD_DET_N */
#if defined(CONFIG_NFC_PN544)
	/* NFC_EN */
	PM8XXX_GPIO_INIT(33, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_OPEN_DRAIN, 0, \
				PM_GPIO_PULL_NO, PM_GPIO_VIN_VPH, \
				PM_GPIO_STRENGTH_LOW, \
				PM_GPIO_FUNC_NORMAL, 0, 0),
#else
	PM8XXX_GPIO_DISABLE(33),
#endif
	/* WCD9310_RESET_N */
	PM8XXX_GPIO_INIT(34, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 1, \
				PM_GPIO_PULL_NO, PM_GPIO_VIN_S4, \
				PM_GPIO_STRENGTH_MED, \
				PM_GPIO_FUNC_NORMAL, 0, 0),
	PM8XXX_GPIO_INPUT(36,	PM_GPIO_PULL_UP_30),	/* SIM_DET_N */
	PM8XXX_GPIO_INPUT(38,	PM_GPIO_PULL_NO),	/* PLUG_DET */
	/* OTG_OVP_CNTL */
	PM8XXX_GPIO_INIT(42, PM_GPIO_DIR_OUT, PM_GPIO_OUT_BUF_CMOS, 1, \
				PM_GPIO_PULL_NO, PM_GPIO_VIN_VPH, \
				PM_GPIO_STRENGTH_LOW, \
				PM_GPIO_FUNC_NORMAL, 0, 0),

	/* MLCD_RST_N (GPIO_43) is set by LCD driver */
};

/* Initial PM8921 MPP configurations */
static struct pm8xxx_mpp_init pm8921_mpps[] __initdata = {
	/* External 5V regulator enable; shared by HDMI and USB_OTG switches. */
	PM8XXX_MPP_INIT(4, D_OUTPUT, PM8921_MPP_DIG_LEVEL_S4, DOUT_CTRL_LOW),
	PM8XXX_MPP_INIT(PM8XXX_AMUX_MPP_8, A_INPUT, PM8XXX_MPP_AIN_AMUX_CH8,
								DOUT_CTRL_LOW),
};

void __init msm8960_pm8921_gpio_mpp_init(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(pm8921_gpios); i++) {
		rc = pm8xxx_gpio_config(pm8921_gpios[i].gpio,
					&pm8921_gpios[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_gpio_config: rc=%d\n", __func__, rc);
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(pm8921_mpps); i++) {
		rc = pm8xxx_mpp_config(pm8921_mpps[i].mpp,
					&pm8921_mpps[i].config);
		if (rc) {
			pr_err("%s: pm8xxx_mpp_config: rc=%d\n", __func__, rc);
			break;
		}
	}
}

static struct pm8xxx_adc_amux pm8xxx_adc_channels_data[] = {
	{"vcoin", CHANNEL_VCOIN, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vbat", CHANNEL_VBAT, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"dcin", CHANNEL_DCIN, CHAN_PATH_SCALING4, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ichg", CHANNEL_ICHG, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"vph_pwr", CHANNEL_VPH_PWR, CHAN_PATH_SCALING2, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"ibat", CHANNEL_IBAT, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"m4", CHANNEL_MPP_1, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"batt_therm", CHANNEL_BATT_THERM, CHAN_PATH_SCALING1, AMUX_RSV2,
		ADC_DECIMATION_TYPE2, ADC_SCALE_BATT_THERM},
	{"batt_id", CHANNEL_BATT_ID, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"usbin", CHANNEL_USBIN, CHAN_PATH_SCALING3, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pmic_therm", CHANNEL_DIE_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PMIC_THERM},
	{"625mv", CHANNEL_625MV, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"125v", CHANNEL_125V, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"chg_temp", CHANNEL_CHG_TEMP, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_DEFAULT},
	{"pa_therm1", ADC_MPP_1_AMUX8, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PA_THERM},
	{"xo_therm", CHANNEL_MUXOFF, CHAN_PATH_SCALING1, AMUX_RSV0,
		ADC_DECIMATION_TYPE2, ADC_SCALE_XOTHERM},
	{"pa_therm0", ADC_MPP_1_AMUX3, CHAN_PATH_SCALING1, AMUX_RSV1,
		ADC_DECIMATION_TYPE2, ADC_SCALE_PA_THERM},
};

static struct pm8xxx_adc_properties pm8xxx_adc_data = {
	.adc_vdd_reference	= 1800, /* milli-voltage for this adc */
	.bitresolution		= 15,
	.bipolar                = 0,
};

static struct pm8xxx_adc_platform_data pm8xxx_adc_pdata = {
	.adc_channel            = pm8xxx_adc_channels_data,
	.adc_num_board_channel  = ARRAY_SIZE(pm8xxx_adc_channels_data),
	.adc_prop               = &pm8xxx_adc_data,
	.adc_mpp_base		= PM8921_MPP_PM_TO_SYS(1),
};

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata __devinitdata = {
	.irq_base		= PM8921_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(104),
	.irq_trigger_flag	= IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata __devinitdata = {
	.gpio_base	= PM8921_GPIO_PM_TO_SYS(1),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata __devinitdata = {
	.mpp_base	= PM8921_MPP_PM_TO_SYS(1),
};

static struct pm8xxx_rtc_platform_data pm8xxx_rtc_pdata __devinitdata = {
	.rtc_write_enable       = false,
	.rtc_alarm_powerup	= false,
};

static struct pm8xxx_pwrkey_platform_data pm8xxx_pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us	= 15625,
	.wakeup			= 1,
};

static struct gpio_keys_button gpio_keys_buttons[] = {
	{
		.code = KEY_CAMERA_FOCUS,
		.gpio = PM8921_GPIO_PM_TO_SYS(1),
		.active_low = 1,
		.desc = "camera a.focus",
		.type = EV_KEY,
		.wakeup = 0,
		.debounce_interval = 10,
	},
	{
		.code = KEY_CAMERA_SNAPSHOT,
		.gpio = PM8921_GPIO_PM_TO_SYS(2),
		.active_low = 1,
		.desc = "camera snapshot",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 10,
	},
	{
		.code = KEY_VOLUMEDOWN,
		.gpio = PM8921_GPIO_PM_TO_SYS(20),
		.active_low = 1,
		.desc = "volume down",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 10,
	},
	{
		.code = KEY_VOLUMEUP,
		.gpio = PM8921_GPIO_PM_TO_SYS(21),
		.active_low = 1,
		.desc = "volume up",
		.type = EV_KEY,
		.wakeup = 1,
		.debounce_interval = 10,
	},
};

static struct gpio_keys_platform_data gpio_keys_pdata = {
	.buttons = gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(gpio_keys_buttons),
};

static struct platform_device gpio_keys_device = {
	.name = "gpio-keys",
	.id = 0,
	.dev = { .platform_data = &gpio_keys_pdata },
	.num_resources = 0,
	.resource = NULL,
};

#define GPIO_SW_SIM_DETECTION		36

static struct gpio_event_direct_entry gpio_sw_gpio_map[] = {
	{PM8921_GPIO_PM_TO_SYS(GPIO_SW_SIM_DETECTION), SW_JACK_PHYSICAL_INSERT},

};

static struct gpio_event_input_info gpio_sw_gpio_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = 1,
	.flags = 0,
	.type = EV_SW,
	.keymap = gpio_sw_gpio_map,
	.keymap_size = ARRAY_SIZE(gpio_sw_gpio_map),
	.debounce_time.tv64 = 100 * NSEC_PER_MSEC,

};

static struct gpio_event_info *pmic_keypad_info[] = {
	&gpio_sw_gpio_info.info,
};

static struct gpio_event_platform_data pmic_keypad_data = {
	.name = "sim-detection",
	.info = pmic_keypad_info,
	.info_count = ARRAY_SIZE(pmic_keypad_info),
};

static struct platform_device pmic_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {.platform_data = &pmic_keypad_data},
};

static int __init input_devices_init(void)
{
	platform_device_register(&gpio_keys_device);
	platform_device_register(&pmic_keypad_device);
	return 0;
}
static void __exit input_devices_exit(void)
{
	platform_device_unregister(&gpio_keys_device);
	platform_device_unregister(&pmic_keypad_device);
}

module_init(input_devices_init);
module_exit(input_devices_exit);

static int pm8921_therm_mitigation[] = {
	1425,
	825,
	475,
	325,
};

#define MAX_VOLTAGE_MV		4200
#define V_CUTOFF_MV		3200
#define CHG_TERM_MA		70
static struct pm8921_charger_platform_data pm8921_chg_pdata __devinitdata = {
	.update_time		= 30000,
	.max_voltage		= MAX_VOLTAGE_MV,
	.min_voltage		= V_CUTOFF_MV,
	.uvd_thresh_voltage	= 4050,
	.alarm_low_mv		= V_CUTOFF_MV,
	.alarm_high_mv		= V_CUTOFF_MV + 100,
	.resume_voltage_delta	= 60,
	.resume_charge_percent	= 99,
	.term_current		= CHG_TERM_MA,
	.cool_temp		= 10,
	.warm_temp		= 45,
	.hysteresis_temp	= 3,
	.temp_check_period	= 1,
	.max_bat_chg_current	= 1425,
	.cool_bat_chg_current	= 1425,
	.warm_bat_chg_current	= 350,
	.cool_bat_voltage	= 4200,
	.warm_bat_voltage	= 4000,
	.thermal_mitigation	= pm8921_therm_mitigation,
	.thermal_levels		= ARRAY_SIZE(pm8921_therm_mitigation),
	.rconn_mohm		= 18,
	.soc_scaling		= 1,
	.btc_override		= 1,
	.btc_override_cold_degc	= 5,
	.btc_override_hot_degc	= 55,
	.btc_delay_ms		= 10000,
	.btc_panic_if_cant_stop_chg	= 1,
	.safety_time		= 512,
};

static struct pm8xxx_misc_platform_data pm8xxx_misc_pdata = {
	.priority		= 0,
};

static struct pm8921_bms_platform_data pm8921_bms_pdata __devinitdata = {
	.battery_type			= BATT_OEM,
	.r_sense_uohm			= 10000,
	.v_cutoff			= V_CUTOFF_MV,
	.max_voltage_uv			= MAX_VOLTAGE_MV * 1000,
	.rconn_mohm			= 30,
	.shutdown_soc_valid_limit	= 20,
	.adjust_soc_low_threshold	= 25,
	.chg_term_ua			= CHG_TERM_MA * 1000,
	.normal_voltage_calc_ms		= 20000,
	.low_voltage_calc_ms		= 1000,
	.alarm_low_mv			= V_CUTOFF_MV,
	.alarm_high_mv			= V_CUTOFF_MV + 100,
	.hold_soc_est			= 3,
	.low_voltage_detect		= 1,
	.vbatt_cutoff_retries		= 5,
	.enable_fcc_learning		= 1,
	.min_fcc_learning_soc		= 20,
	.min_fcc_ocv_pc			= 30,
	.min_fcc_learning_samples	= 5,
};

#define	PM8921_LC_LED_MAX_CURRENT	4	/* I = 4mA */
#define	PM8921_LC_LED_LOW_CURRENT	1	/* I = 1mA */
#define PM8XXX_LED_PWM_PERIOD		1000
#define PM8XXX_LED_PWM_DUTY_MS		20
/**
 * PM8XXX_PWM_CHANNEL_NONE shall be used when LED shall not be
 * driven using PWM feature.
 */
#define PM8XXX_PWM_CHANNEL_NONE		-1

static struct led_info pm8921_led_info[] = {
	[0] = {
		.name			= "led:battery_charging",
		.default_trigger	= "battery-charging",
	},
	[1] = {
		.name			= "led:battery_full",
		.default_trigger	= "battery-full",
	},
};

static struct led_platform_data pm8921_led_core_pdata = {
	.num_leds = ARRAY_SIZE(pm8921_led_info),
	.leds = pm8921_led_info,
};

static int pm8921_led0_pwm_duty_pcts[56] = {
		1, 4, 8, 12, 16, 20, 24, 28, 32, 36,
		40, 44, 46, 52, 56, 60, 64, 68, 72, 76,
		80, 84, 88, 92, 96, 100, 100, 100, 98, 95,
		92, 88, 84, 82, 78, 74, 70, 66, 62, 58,
		58, 54, 50, 48, 42, 38, 34, 30, 26, 22,
		14, 10, 6, 4, 1
};

/*
 * Note: There is a bug in LPG module that results in incorrect
 * behavior of pattern when LUT index 0 is used. So effectively
 * there are 63 usable LUT entries.
 */
static struct pm8xxx_pwm_duty_cycles pm8921_led0_pwm_duty_cycles = {
	.duty_pcts = (int *)&pm8921_led0_pwm_duty_pcts,
	.num_duty_pcts = ARRAY_SIZE(pm8921_led0_pwm_duty_pcts),
	.duty_ms = PM8XXX_LED_PWM_DUTY_MS,
	.start_idx = 1,
};

static struct pm8xxx_led_config pm8921_led_configs[] = {
	[0] = {
		.id = PM8XXX_ID_LED_0,
		.mode = PM8XXX_LED_MODE_PWM2,
		.max_current = PM8921_LC_LED_MAX_CURRENT,
		.pwm_channel = 5,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
		.pwm_duty_cycles = &pm8921_led0_pwm_duty_cycles,
	},
	[1] = {
		.id = PM8XXX_ID_LED_1,
		.mode = PM8XXX_LED_MODE_PWM1,
		.max_current = PM8921_LC_LED_MAX_CURRENT,
		.pwm_channel = 4,
		.pwm_period_us = PM8XXX_LED_PWM_PERIOD,
	},
};

static struct pm8xxx_led_platform_data pm8xxx_leds_pdata = {
		.led_core = &pm8921_led_core_pdata,
		.configs = pm8921_led_configs,
		.num_configs = ARRAY_SIZE(pm8921_led_configs),
};

static struct pm8xxx_ccadc_platform_data pm8xxx_ccadc_pdata = {
	.r_sense_uohm		= 10000,
	.calib_delay_ms		= 600000,
};

/**
 * PM8XXX_PWM_DTEST_CHANNEL_NONE shall be used when no LPG
 * channel should be in DTEST mode.
 */

#define PM8XXX_PWM_DTEST_CHANNEL_NONE   (-1)

static struct pm8xxx_pwm_platform_data pm8xxx_pwm_pdata = {
	.dtest_channel	= PM8XXX_PWM_DTEST_CHANNEL_NONE,
};

static struct pm8921_platform_data pm8921_platform_data __devinitdata = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.rtc_pdata              = &pm8xxx_rtc_pdata,
	.pwrkey_pdata		= &pm8xxx_pwrkey_pdata,
	.misc_pdata		= &pm8xxx_misc_pdata,
	.regulator_pdatas	= msm_pm8921_regulator_pdata,
	.charger_pdata		= &pm8921_chg_pdata,
	.bms_pdata		= &pm8921_bms_pdata,
	.adc_pdata		= &pm8xxx_adc_pdata,
	.leds_pdata		= &pm8xxx_leds_pdata,
	.ccadc_pdata		= &pm8xxx_ccadc_pdata,
	.pwm_pdata		= &pm8xxx_pwm_pdata,
};

static struct msm_ssbi_platform_data msm8960_ssbi_pm8921_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8921-core",
		.platform_data		= &pm8921_platform_data,
	},
};

void __init msm8960_init_pmic(void)
{
	pmic_reset_irq = PM8921_IRQ_BASE + PM8921_RESOUT_IRQ;
	msm8960_device_ssbi_pmic.dev.platform_data =
				&msm8960_ssbi_pm8921_pdata;
	pm8921_platform_data.num_regulators = msm_pm8921_regulator_pdata_len;

	if (machine_is_msm8960_mtp())
		pm8921_platform_data.bms_pdata->battery_type = BATT_PALLADIUM;
}
