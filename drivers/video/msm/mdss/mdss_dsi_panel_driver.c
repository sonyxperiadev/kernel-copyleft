/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * Copyright (C) 2015 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/leds-qpnp-wled.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/pm_qos.h>
#include <linux/mdss_io_util.h>
#include <linux/regulator/qpnp-labibb-regulator.h>

#include "mdss.h"
#include "mdss_panel.h"
#include "mdss_dsi.h"
#include "mdss_debug.h"
#include "mdss_dba_utils.h"
#include "mdss_dsi_panel_driver.h"
#include "mdss_dsi_panel_debugfs.h"

static bool gpio_req;
static u32 down_period;

static struct fps_data fpsd, vpsd;

#define ADC_PNUM		2
#define ADC_RNG_MIN		0
#define ADC_RNG_MAX		1

#define CHANGE_FPS_MIN 36
#define CHANGE_FPS_MAX 63

#define NODE_OF_HYBRID "/soc/dsi_panel_pwr_supply_hybrid_incell"
#define NODE_OF_FULL "/soc/dsi_panel_pwr_supply_full_incell"

#define PSEC ((u64)1000000000000)
#define KSEC ((u64)1000)
#define CHANGE_PAYLOAD(a, b) (spec_pdata->fps_cmds.cmds[a].payload[b])

static unsigned long lcdid_adc = 1505000;
static void vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t t);

static struct mdss_mdp_vsync_handler vs_handle;
static bool display_onoff_state;

static int __init lcdid_adc_setup(char *str)
{
	unsigned long res;

	if (!*str)
		return 0;
	if (!kstrtoul(str, 0, &res)) {
		lcdid_adc = res;
	}

	return 1;
}
__setup("lcdid_adc=", lcdid_adc_setup);

void mdss_dsi_panel_driver_detection(
		struct platform_device *pdev,
		struct device_node **np)
{
	u32 res[ADC_PNUM];
	int rc = 0;
	struct device_node *parent;
	struct device_node *next;
	u32 dev_index = 0;
	u32 dsi_index = 0;
	u32 adc_uv = 0;

	rc = of_property_read_u32(pdev->dev.of_node, "cell-index", &dev_index);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: Cell-index not specified, rc=%d\n",
						__func__, rc);
		return;
	}

	parent = of_get_parent(*np);

	adc_uv = lcdid_adc;
	pr_info("%s: physical:%d\n", __func__, adc_uv);

	for_each_child_of_node(parent, next) {
		rc = of_property_read_u32(next, "somc,dsi-index", &dsi_index);
		if (rc)
			dsi_index = 0;
		if (dsi_index != dev_index)
			continue;

		rc = of_property_read_u32_array(next,
				"somc,lcd-id-adc", res, ADC_PNUM);
		if (rc)
			continue;
		if (adc_uv < res[ADC_RNG_MIN] || res[ADC_RNG_MAX] < adc_uv)
			continue;

		*np = next;
		break;
	}
}

static int mdss_dsi_panel_driver_vreg_name_to_config(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		struct dss_vreg *config, char *name)
{
	struct dss_vreg *vreg_config = ctrl_pdata->panel_power_data.vreg_config;
	int num_vreg = ctrl_pdata->panel_power_data.num_vreg;
	int i = 0;
	int valid = -EINVAL;

	for (i = 0; i < num_vreg; i++) {
		if (!strcmp(name, vreg_config[i].vreg_name)) {
			*config = vreg_config[i];
			valid = 0;
			break;
		}
	}

	return valid;
}

static int mdss_dsi_panel_driver_vreg_ctrl(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata, char *vreg, bool enable)
{
	struct dss_vreg vreg_config;
	struct mdss_panel_power_seq *pw_seq = NULL;
	int valid = 0;
	int wait = 0;
	int ret = 0;

	valid = mdss_dsi_panel_driver_vreg_name_to_config(
			ctrl_pdata, &vreg_config, vreg);

	if (!valid) {
		if (enable) {
			ret = msm_dss_enable_vreg(&vreg_config, 1, 1);
			pw_seq = &ctrl_pdata->spec_pdata->on_seq;
		} else {
			ret = msm_dss_enable_vreg(&vreg_config, 1, 0);
			pw_seq = &ctrl_pdata->spec_pdata->off_seq;
		}

		if (!strcmp(vreg, "vdd"))
			wait = pw_seq->disp_vdd;
		else if (!strcmp(vreg, "vddio"))
			wait = pw_seq->disp_vddio;
		else if (!strcmp(vreg, "lab"))
			wait = pw_seq->disp_vsp;
		else if (!strcmp(vreg, "ibb"))
			wait = pw_seq->disp_vsn;
		else if (!strcmp(vreg, "touch-avdd"))
			wait = pw_seq->touch_avdd;
		else
			wait = 0;

		if (!ret && wait) {
			usleep_range(wait * 1000, wait * 1000 + 100);
		}
	}

	return ret;
}

bool mdss_dsi_panel_driver_is_power_on(unsigned char state)
{
	bool ret = false;

	if (state & INCELL_POWER_STATE_ON)
		ret = true;

	pr_debug("%s: In-Cell %s state\n", __func__, (ret ? "on" : "off"));

	return ret;
}

bool mdss_dsi_panel_driver_is_power_lock(unsigned char state)
{
	bool ret = false;

	if (state & INCELL_LOCK_STATE_ON)
		ret = true;

	pr_debug("%s: In-Cell I/F %s state\n", __func__,
		(ret ? "Lock" : "Unlock"));

	return ret;
}

bool mdss_dsi_panel_driver_is_ewu(unsigned char state)
{
	bool ret = false;

	if (state & INCELL_EWU_STATE_ON)
		ret = true;

	pr_debug("%s: In-Cell I/F %s state\n", __func__,
		(ret ? "EWU" : "NORMAL"));

	return ret;
}

bool mdss_dsi_panel_driver_is_system_on(unsigned char state)
{
	bool ret = false;

	if (state & INCELL_SYSTEM_STATE_ON)
		ret = true;

	pr_debug("%s: In-Cell system %s state\n", __func__,
		(ret ? "resume" : "suspend"));

	return ret;
}

static bool mdss_dsi_panel_driver_is_seq_for_ewu(void)
{
	struct incell_ctrl *incell = incell_get_info();

	if (incell &&
		(incell->seq == POWER_ON_EWU_SEQ))
		return true;

	return false;
}

static bool mdss_dsi_panel_driver_is_incell_operation(void)
{
	struct incell_ctrl *incell = incell_get_info();

	if (incell &&
		(incell->incell_intf_operation == INCELL_TOUCH_RUN))
		return true;

	return false;
}

static int mdss_dsi_panel_calculation_sleep(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		int gpio, bool enable)
{
	struct mdss_panel_specific_pdata *spec_pdata = ctrl_pdata->spec_pdata;
	struct mdss_panel_power_seq *pw_seq = NULL;
	int wait = 0;

	if (mdss_dsi_panel_driver_is_seq_for_ewu() &&
		(gpio == spec_pdata->touch_reset_gpio) &&
		!enable) {
		if (&spec_pdata->ewu_seq)
			pw_seq = &spec_pdata->ewu_seq;
		else
			pw_seq = &spec_pdata->on_seq;
	} else {
		if (enable)
			pw_seq = &spec_pdata->on_seq;
		else
			pw_seq = &spec_pdata->off_seq;
	}

	if (gpio == spec_pdata->touch_vddio_gpio)
		wait = pw_seq->touch_vddio;
	else if (gpio == spec_pdata->touch_reset_gpio)
		wait = pw_seq->touch_reset;
	else if (gpio == spec_pdata->touch_int_gpio)
		wait = pw_seq->touch_intn;
	else
		wait = 0;

	wait = wait * 1000;
	return wait;
}

static void mdss_dsi_panel_driver_gpio_output(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		int gpio, bool enable, int value)
{
	int wait;

	wait = mdss_dsi_panel_calculation_sleep(ctrl_pdata, gpio, enable);

	if (gpio_is_valid(gpio))
		gpio_direction_output(gpio, value);

	if (wait)
		usleep_range(wait, wait + 100);
}

static void mdss_dsi_panel_driver_set_gpio(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		int gpio, bool enable, int value)
{
	int wait = 0;

	wait = mdss_dsi_panel_calculation_sleep(ctrl_pdata, gpio, enable);

	if (gpio_is_valid(gpio))
		gpio_set_value(gpio, value);

	if (wait)
		usleep_range(wait, wait + 100);
}

int mdss_dsi_panel_driver_pinctrl_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	ctrl_pdata->pin_res.touch_state_active
		= pinctrl_lookup_state(ctrl_pdata->pin_res.pinctrl,
				MDSS_PINCTRL_STATE_TOUCH_ACTIVE);
	if (IS_ERR_OR_NULL(ctrl_pdata->pin_res.touch_state_active))
		pr_warn("%s: can not get touch active pinstate\n", __func__);

	ctrl_pdata->pin_res.touch_state_suspend
		= pinctrl_lookup_state(ctrl_pdata->pin_res.pinctrl,
				MDSS_PINCTRL_STATE_TOUCH_SUSPEND);
	if (IS_ERR_OR_NULL(ctrl_pdata->pin_res.touch_state_suspend))
		pr_warn("%s: can not get touch suspend pinstate\n", __func__);

	return 0;
}

int mdss_dsi_panel_driver_touch_pinctrl_set_state(
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool active)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	struct pinctrl_state *pin_state;
	int rc = -EFAULT;
	int wait = 0;

	if (IS_ERR_OR_NULL(ctrl_pdata->pin_res.pinctrl))
		return PTR_ERR(ctrl_pdata->pin_res.pinctrl);

	spec_pdata = ctrl_pdata->spec_pdata;

	pin_state = active ? ctrl_pdata->pin_res.touch_state_active
				: ctrl_pdata->pin_res.touch_state_suspend;

	if (!IS_ERR_OR_NULL(pin_state)) {
		rc = pinctrl_select_state(ctrl_pdata->pin_res.pinctrl,
				pin_state);
		if (!rc) {
			wait = mdss_dsi_panel_calculation_sleep(ctrl_pdata,
					spec_pdata->touch_int_gpio, active);
			if (wait)
				usleep_range(wait, wait + 100);
		} else {
			pr_err("%s: can not set %s pins\n", __func__,
			       active ? MDSS_PINCTRL_STATE_TOUCH_ACTIVE
			       : MDSS_PINCTRL_STATE_TOUCH_SUSPEND);
		}
	} else {
		pr_err("%s: invalid '%s' pinstate\n", __func__,
		       active ? MDSS_PINCTRL_STATE_TOUCH_ACTIVE
		       : MDSS_PINCTRL_STATE_TOUCH_SUSPEND);
	}

	return rc;
}

void mdss_dsi_panel_driver_state_change_off(struct incell_ctrl *incell)
{
	incell_state_change change_state = incell->change_state;
	unsigned char *state = &incell->state;

	pr_debug("%s: status:0x%x --->\n", __func__, (*state));

	switch (change_state) {
	case INCELL_STATE_NONE:
		pr_debug("%s: Not change off status\n", __func__);
		break;
	case INCELL_STATE_S_OFF:
		*state &= INCELL_SYSTEM_STATE_OFF;
		break;
	case INCELL_STATE_P_OFF:
		*state &= INCELL_POWER_STATE_OFF;
		break;
	case INCELL_STATE_SP_OFF:
		*state &= INCELL_POWER_STATE_OFF;
		*state &= INCELL_SYSTEM_STATE_OFF;
		break;
	default:
		pr_err("%s: offmode unknown\n", __func__);
		break;
	}

	pr_debug("%s: ---> status:0x%x\n", __func__, (*state));
}

void mdss_dsi_panel_driver_power_off_ctrl(struct incell_ctrl *incell)
{
	incell_pw_seq seq = POWER_OFF_EXECUTE;
	incell_state_change change_state = INCELL_STATE_NONE;
	unsigned char state = incell->state;
	incell_intf_mode intf_mode = incell->intf_mode;
	incell_worker_state worker_state = incell->worker_state;
	bool incell_intf_operation = incell->incell_intf_operation;

	if (worker_state == INCELL_WORKER_ON) {
		change_state = INCELL_STATE_P_OFF;
	} else if (incell_intf_operation == INCELL_TOUCH_RUN) {
		/* touch I/F running mode */
		if (intf_mode == INCELL_DISPLAY_HW_RESET) {
			if (!mdss_dsi_panel_driver_is_power_on(state)) {
				pr_err("%s: Already power off. state:0x%x\n",
						__func__, state);
				seq = POWER_OFF_SKIP;
			} else {
				change_state = INCELL_STATE_P_OFF;
			}
		} else {
			if (!mdss_dsi_panel_driver_is_power_on(state)) {
				pr_err("%s: Power off status. state:0x%x\n",
						__func__, state);
				seq = POWER_OFF_SKIP;
			} else if (mdss_dsi_panel_driver_is_ewu(state)) {
				pr_debug("%s: Skip power off for EWU seq\n",
						__func__);
				seq = POWER_OFF_SKIP;
			} else {
				change_state = INCELL_STATE_P_OFF;
			}
		}
	} else {
		if (worker_state == INCELL_WORKER_PENDING)
			incell_panel_power_worker_canceling(incell);

		/* touch I/F idling mode */
		if (mdss_dsi_panel_driver_is_power_lock(state)) {
			change_state = INCELL_STATE_S_OFF;
			seq = POWER_OFF_SKIP;
		} else if (!mdss_dsi_panel_driver_is_power_on(state)) {
			change_state = INCELL_STATE_S_OFF;
			seq = POWER_OFF_SKIP;
		} else if (mdss_dsi_panel_driver_is_ewu(state)) {
			change_state = INCELL_STATE_S_OFF;
			seq = POWER_OFF_SKIP;
		} else {
			change_state = INCELL_STATE_SP_OFF;
		}
	}

	pr_debug("%s: incell change state seq:%d change_state:%d\n",
				__func__, (int)seq, (int)change_state);
	incell->seq = seq;
	incell->change_state = change_state;
}

int mdss_dsi_panel_driver_power_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	struct mdss_panel_power_seq *pw_seq = NULL;
	struct incell_ctrl *incell = incell_get_info();
	int ret = 0;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto end;
	}

	if (incell)
		if (incell->seq == POWER_OFF_SKIP)
			return ret;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	spec_pdata = ctrl_pdata->spec_pdata;
	pw_seq = &spec_pdata->off_seq;

	if (spec_pdata->off_seq.rst_b_seq) {
		rc = mdss_dsi_panel_reset(pdata, 0);
		if (rc)
			pr_warn("%s: Panel reset failed. rc=%d\n",
					__func__, rc);
	}

	ret += mdss_dsi_panel_driver_reset_touch(pdata, 0);
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "ibb", false);
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "lab", false);

	if (!spec_pdata->off_seq.rst_b_seq) {
		rc = mdss_dsi_panel_reset(pdata, 0);
		if (rc)
			pr_warn("%s: Panel reset failed. rc=%d\n",
					__func__, rc);
	}

	mdss_dsi_panel_driver_touch_pinctrl_set_state(ctrl_pdata, false);

	mdss_dsi_panel_driver_set_gpio(ctrl_pdata,
		(spec_pdata->touch_vddio_gpio), false, 1);

	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "vddio", false);
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "touch-avdd", false);
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "vdd", false);

	if (ret)
		pr_err("%s: failed to disable vregs for %s\n",
				__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
	else
		pr_notice("@@@@ panel power off @@@@\n");

	if (mdss_dsi_pinctrl_set_state(ctrl_pdata, false))
		pr_debug("reset disable: pinctrl not enabled\n");

	if (spec_pdata->down_period)
		down_period = (u32)ktime_to_ms(ktime_get());

end:
	return ret;
}

void mdss_dsi_panel_driver_state_change_on(struct incell_ctrl *incell)
{
	incell_state_change change_state = incell->change_state;
	unsigned char *state = &incell->state;

	pr_debug("%s: status:0x%x --->\n", __func__, (*state));

	switch (change_state) {
	case INCELL_STATE_NONE:
		pr_debug("%s: Not change on status\n", __func__);
		break;
	case INCELL_STATE_S_ON:
		*state |= INCELL_SYSTEM_STATE_ON;
		break;
	case INCELL_STATE_P_ON:
		*state |= INCELL_POWER_STATE_ON;
		break;
	case INCELL_STATE_SP_ON:
		*state |= INCELL_SYSTEM_STATE_ON;
		*state |= INCELL_POWER_STATE_ON;
		break;
	default:
		pr_err("%s: onmode unknown\n", __func__);
		break;
	}

	pr_debug("%s: ---> status:0x%x\n", __func__, (*state));
}

void mdss_dsi_panel_driver_power_on_ctrl(struct incell_ctrl *incell)
{
	incell_pw_seq seq = POWER_ON_EXECUTE;
	incell_state_change change_state = INCELL_STATE_NONE;
	unsigned char state = incell->state;
	incell_intf_mode intf_mode = incell->intf_mode;
	incell_worker_state worker_state = incell->worker_state;
	bool incell_intf_operation = incell->incell_intf_operation;

	if (worker_state == INCELL_WORKER_ON) {
		change_state = INCELL_STATE_P_ON;
	} else if (incell_intf_operation == INCELL_TOUCH_RUN) {
		/* touch I/F running mode */
		if (intf_mode != INCELL_DISPLAY_HW_RESET) {
			pr_err("%s: Unknown I/F: %d\n",
					__func__, (int)intf_mode);
			seq = POWER_ON_SKIP;
		} else if (mdss_dsi_panel_driver_is_power_on(state)) {
			pr_err("%s: Already power on status. state:0x%x\n",
					__func__, state);
			seq = POWER_ON_SKIP;
		} else {
			change_state = INCELL_STATE_P_ON;
		}
	} else {
		/* touch I/F idling mode */
		if (worker_state == INCELL_WORKER_PENDING) {
			incell_panel_power_worker_canceling(incell);
			change_state = INCELL_STATE_S_ON;
			seq = POWER_ON_EWU_SEQ;
		} else if (mdss_dsi_panel_driver_is_ewu(state)) {
			if (mdss_dsi_panel_driver_is_power_on(state)) {
				change_state = INCELL_STATE_S_ON;
				seq = POWER_ON_EWU_SEQ;
			} else {
				change_state = INCELL_STATE_SP_ON;
				seq = POWER_ON_EXECUTE;
			}
		} else if (mdss_dsi_panel_driver_is_power_on(state)) {
			change_state = INCELL_STATE_S_ON;
			seq = POWER_ON_EWU_SEQ;
		} else {
			change_state = INCELL_STATE_SP_ON;
			seq = POWER_ON_EXECUTE;
		}
	}

	pr_debug("%s: incell change state seq:%d change_state:%d\n",
				__func__, (int)seq, (int)change_state);
	incell->seq = seq;
	incell->change_state = change_state;
}

static int mdss_dsi_panel_driver_ewu_seq(struct mdss_panel_data *pdata)
{
	int ret = 0;

	ret += mdss_dsi_panel_driver_reset_touch(pdata, 0);
	ret += mdss_dsi_panel_driver_reset_panel(pdata, 1);

	return ret;
}

int mdss_dsi_panel_driver_power_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	struct mdss_panel_power_seq *pw_seq = NULL;
	struct incell_ctrl *incell = incell_get_info();
	unsigned char state;
	int ret = 0;
	int wait;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (incell) {
		mdss_dsi_panel_driver_power_on_ctrl(incell);
		if (incell->seq != POWER_ON_EXECUTE) {
			if (incell->seq == POWER_ON_EWU_SEQ)
				ret = mdss_dsi_panel_driver_ewu_seq(pdata);
			return ret;
		}
		state = incell->state;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	spec_pdata = ctrl_pdata->spec_pdata;
	pw_seq = &spec_pdata->on_seq;

	if (!gpio_req) {
		ret = mdss_dsi_request_gpios(ctrl_pdata);
		if (ret) {
			pr_err("gpio request failed\n");
			return ret;
		}
		gpio_req = true;
	}

	if (spec_pdata->down_period) {
		u32 kt = (u32)ktime_to_ms(ktime_get());
		kt = (kt < down_period) ? kt + ~down_period : kt - down_period;
		if (kt < spec_pdata->down_period)
			usleep_range((spec_pdata->down_period - kt) *
				1000,
				(spec_pdata->down_period - kt) *
				1000 + 100);
	}

	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "vdd", true);
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "touch-avdd", true);
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "vddio", true);

	mdss_dsi_panel_driver_gpio_output(ctrl_pdata,
		(spec_pdata->touch_vddio_gpio), true, 0);

	mdss_dsi_panel_driver_touch_pinctrl_set_state(ctrl_pdata, true);

	if ((spec_pdata->panel_type == HYBRID_INCELL) &&
	    (!mdss_dsi_panel_driver_is_power_on(state))) {
		ret += mdss_dsi_panel_driver_reset_touch(pdata, 1);
		wait = pw_seq->touch_reset_first;
		usleep_range(wait * 1000, wait * 1000 + 100);
	}
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "lab", true);
	ret += mdss_dsi_panel_driver_vreg_ctrl(ctrl_pdata, "ibb", true);

	if (ret) {
		pr_err("%s: failed to enable vregs for %s\n",
			__func__, __mdss_dsi_pm_name(DSI_PANEL_PM));
		return ret;
	}

	pr_notice("@@@@ panel power on @@@@\n");

	/*
	 * If continuous splash screen feature is enabled, then we need to
	 * request all the GPIOs that have already been configured in the
	 * bootloader. This needs to be done irresepective of whether
	 * the lp11_init flag is set or not.
	 */
	if (!pdata->panel_info.cont_splash_enabled &&
		!pdata->panel_info.mipi.lp11_init) {
		if (mdss_dsi_pinctrl_set_state(ctrl_pdata, true))
			pr_debug("reset enable: pinctrl not enabled\n");

		ret = mdss_dsi_panel_reset(pdata, 1);
		if (ret)
			pr_err("%s: Panel reset failed. rc=%d\n",
					__func__, ret);
	}

	return ret;
}

int mdss_dsi_panel_driver_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	int rc = 0;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	spec_pdata = ctrl_pdata->spec_pdata;

	if (gpio_is_valid(spec_pdata->touch_vddio_gpio)) {
		rc = gpio_request(spec_pdata->touch_vddio_gpio,
						"touch_vddio");
		if (rc) {
			pr_err("request touch vddio gpio failed, rc=%d\n",
				       rc);
			goto touch_vddio_gpio_err;
		}
	}

	if (gpio_is_valid(spec_pdata->touch_reset_gpio)) {
		rc = gpio_request(spec_pdata->touch_reset_gpio,
						"touch_reset");
		if (rc) {
			pr_err("request touch reset gpio failed,rc=%d\n",
								rc);
			goto touch_reset_gpio_err;
		}
	}

	if (gpio_is_valid(spec_pdata->touch_int_gpio)) {
		rc = gpio_request(spec_pdata->touch_int_gpio,
						"touch_int");
		if (rc) {
			pr_err("request touch int gpio failed,rc=%d\n",
								rc);
			goto touch_int_gpio_err;
		}
	}

	return rc;

touch_int_gpio_err:
	if (gpio_is_valid(spec_pdata->touch_reset_gpio))
		gpio_free(spec_pdata->touch_reset_gpio);
touch_reset_gpio_err:
	if (gpio_is_valid(spec_pdata->touch_vddio_gpio))
		gpio_free(spec_pdata->touch_vddio_gpio);
touch_vddio_gpio_err:
	return rc;
}

void mdss_dsi_panel_driver_gpio_free(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	spec_pdata = ctrl_pdata->spec_pdata;

	if (gpio_is_valid(spec_pdata->touch_int_gpio))
		gpio_free(spec_pdata->touch_int_gpio);

	if (gpio_is_valid(spec_pdata->touch_reset_gpio))
		gpio_free(spec_pdata->touch_reset_gpio);

	if (gpio_is_valid(spec_pdata->touch_vddio_gpio))
		gpio_free(spec_pdata->touch_vddio_gpio);
}

void mdss_dsi_panel_driver_parse_gpio_params(struct platform_device *ctrl_pdev,
		struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	spec_pdata = ctrl_pdata->spec_pdata;

	spec_pdata->touch_vddio_gpio = of_get_named_gpio(
			ctrl_pdev->dev.of_node,
			"qcom,platform-touch-vddio-gpio", 0);

	if (!gpio_is_valid(spec_pdata->touch_vddio_gpio))
		pr_err("%s:%d, touch vddio gpio not specified\n",
						__func__, __LINE__);

	spec_pdata->touch_reset_gpio = of_get_named_gpio(
			ctrl_pdev->dev.of_node,
			"qcom,platform-touch-reset-gpio", 0);

	if (!gpio_is_valid(spec_pdata->touch_reset_gpio))
		pr_err("%s:%d, touch reset gpio not specified\n",
						__func__, __LINE__);

	spec_pdata->touch_int_gpio = of_get_named_gpio(
			ctrl_pdev->dev.of_node,
			"qcom,platform-touch-int-gpio", 0);

	if (!gpio_is_valid(spec_pdata->touch_int_gpio))
		pr_err("%s:%d, touch int gpio not specified\n",
						__func__, __LINE__);
}


static void mdss_dsi_panel_set_gpio_seq(
		int gpio, int seq_num, const int *seq)
{
	int i;

	for (i = 0; i + 1 < seq_num; i += 2) {
		gpio_set_value(gpio, seq[i]);
		usleep_range(seq[i + 1] * 1000, seq[i + 1] * 1000 + 100);
		pr_debug("%s: enable=%d, wait=%dms\n",
			__func__, seq[i], seq[i+1]);
	}
}

int mdss_dsi_panel_driver_reset_panel(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	struct mdss_panel_power_seq *pw_seq = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, panel reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);


	if (!gpio_req) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		gpio_req = true;
	}

	if (mdss_dsi_panel_driver_is_seq_for_ewu() && enable)
		pw_seq = &ctrl_pdata->spec_pdata->ewu_seq ?
				&ctrl_pdata->spec_pdata->ewu_seq :
				&ctrl_pdata->spec_pdata->on_seq;
	else
		pw_seq = (enable) ? &ctrl_pdata->spec_pdata->on_seq :
					&ctrl_pdata->spec_pdata->off_seq;

	mdss_dsi_panel_set_gpio_seq(ctrl_pdata->rst_gpio,
				pw_seq->seq_num, pw_seq->rst_seq);

	return rc;
}

int mdss_dsi_panel_driver_reset_touch(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	spec_pdata = ctrl_pdata->spec_pdata;

	if (!gpio_is_valid(spec_pdata->touch_reset_gpio)) {
		pr_debug("%s:%d, touch reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		if (!gpio_req) {
			rc = mdss_dsi_request_gpios(ctrl_pdata);
			if (rc) {
				pr_err("gpio request failed\n");
				return rc;
			}
			gpio_req = true;
		}

		mdss_dsi_panel_driver_gpio_output(ctrl_pdata,
			(spec_pdata->touch_reset_gpio), true, 1);
	} else {
		mdss_dsi_panel_driver_set_gpio(ctrl_pdata,
			(spec_pdata->touch_reset_gpio), false, 0);
	}

	return rc;
}

static int mdss_dsi_property_read_u32_var(struct device_node *np,
		char *name, u32 **out_data, int *num)
{
	struct property *prop = of_find_property(np, name, NULL);
	const __be32 *val;
	u32 *out;
	int s;

	if (!prop) {
		pr_debug("%s:%d, unable to read %s", __func__, __LINE__, name);
		return -EINVAL;
	}
	if (!prop->value) {
		pr_debug("%s:%d, no data of %s", __func__, __LINE__, name);
		return -ENODATA;
	}

	*num = prop->length / sizeof(u32);
	if (!*num || *num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, *num);
		return -ENODATA;
	}
	*out_data = kzalloc(prop->length, GFP_KERNEL);
	if (!*out_data) {
		pr_err("%s:no mem assigned: kzalloc fail\n", __func__);
		*num = 0;
		return -ENOMEM;
	}

	val = prop->value;
	out = *out_data;
	s = *num;
	while (s--)
		*out++ = be32_to_cpup(val++);
	return 0;
}

int mdss_dsi_panel_driver_parse_dt(struct device_node *np,
		struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	u32 tmp = 0;
	u32 res[2];
	int rc = 0;
	struct device_node *panel_np;
	const char *panel_type_name;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);
	struct mdss_panel_labibb_data *labibb = NULL;

	if (ctrl_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		goto error;
	}

	spec_pdata = ctrl_pdata->spec_pdata;

	spec_pdata->pcc_enable = of_property_read_bool(np, "somc,mdss-dsi-pcc-enable");
	if (spec_pdata->pcc_enable) {
		mdss_dsi_parse_dcs_cmds(np, &spec_pdata->pre_uv_read_cmds,
			"somc,mdss-dsi-pre-uv-command", NULL);

		mdss_dsi_parse_dcs_cmds(np, &spec_pdata->uv_read_cmds,
			"somc,mdss-dsi-uv-command", NULL);

		rc = of_property_read_u32(np,
			"somc,mdss-dsi-uv-param-type", &tmp);
		spec_pdata->pcc_data.param_type =
			(!rc ? tmp : CLR_DATA_UV_PARAM_TYPE_NONE);

		rc = of_property_read_u32(np,
			"somc,mdss-dsi-pcc-table-size", &tmp);
		spec_pdata->pcc_data.tbl_size =
			(!rc ? tmp : 0);

		spec_pdata->pcc_data.color_tbl =
			kzalloc(spec_pdata->pcc_data.tbl_size *
				sizeof(struct mdss_pcc_color_tbl),
				GFP_KERNEL);
		if (!spec_pdata->pcc_data.color_tbl) {
			pr_err("no mem assigned: kzalloc fail\n");
			return -ENOMEM;
		}
		rc = of_property_read_u32_array(np,
			"somc,mdss-dsi-pcc-table",
			(u32 *)spec_pdata->pcc_data.color_tbl,
			spec_pdata->pcc_data.tbl_size *
			sizeof(struct mdss_pcc_color_tbl) /
			sizeof(u32));
		if (rc) {
			spec_pdata->pcc_data.tbl_size = 0;
			kzfree(spec_pdata->pcc_data.color_tbl);
			spec_pdata->pcc_data.color_tbl = NULL;
			pr_err("%s:%d, Unable to read pcc table",
				__func__, __LINE__);
		}

		rc = of_property_read_u32_array(np,
			"somc,mdss-dsi-u-rev", res, 2);
		if (rc) {
			spec_pdata->rev_u[0] = 0;
			spec_pdata->rev_u[1] = 0;
		} else {
			spec_pdata->rev_u[0] = res[0];
			spec_pdata->rev_u[1] = res[1];
		}
		rc = of_property_read_u32_array(np,
			"somc,mdss-dsi-v-rev", res, 2);
		if (rc) {
			spec_pdata->rev_v[0] = 0;
			spec_pdata->rev_v[1] = 0;
		} else {
			spec_pdata->rev_v[0] = res[0];
			spec_pdata->rev_v[1] = res[1];
		}

		spec_pdata->pcc_data.pcc_sts |= PCC_STS_UD;
	}

	(void)mdss_dsi_property_read_u32_var(np,
		"somc,pw-on-rst-seq",
		(u32 **)&spec_pdata->on_seq.rst_seq,
		&spec_pdata->on_seq.seq_num);


	if (of_find_property(np, "somc,pw-off-rst-b-seq", NULL)) {
		spec_pdata->off_seq.rst_b_seq = true;

		(void)mdss_dsi_property_read_u32_var(np,
			"somc,pw-off-rst-b-seq",
			(u32 **)&spec_pdata->off_seq.rst_seq,
			&spec_pdata->off_seq.seq_num);
	} else {
		(void)mdss_dsi_property_read_u32_var(np,
			"somc,pw-off-rst-seq",
			(u32 **)&spec_pdata->off_seq.rst_seq,
			&spec_pdata->off_seq.seq_num);
	}


	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-vddio", &tmp);
	spec_pdata->on_seq.disp_vddio = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-vsp", &tmp);
	spec_pdata->on_seq.disp_vsp = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-vsn", &tmp);
	spec_pdata->on_seq.disp_vsn = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-off-vddio", &tmp);
	spec_pdata->off_seq.disp_vddio = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-off-vsp", &tmp);
	spec_pdata->off_seq.disp_vsp = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-off-vsn", &tmp);
	spec_pdata->off_seq.disp_vsn = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-touch-avdd", &tmp);
	spec_pdata->on_seq.touch_avdd = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-touch-vddio", &tmp);
	spec_pdata->on_seq.touch_vddio = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-touch-reset", &tmp);
	spec_pdata->on_seq.touch_reset = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-touch-reset-first", &tmp);
	spec_pdata->on_seq.touch_reset_first = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-on-touch-int-n", &tmp);
	spec_pdata->on_seq.touch_intn = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-off-touch-avdd", &tmp);
	spec_pdata->off_seq.touch_avdd = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-off-touch-vddio", &tmp);
	spec_pdata->off_seq.touch_vddio = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-off-touch-reset", &tmp);
	spec_pdata->off_seq.touch_reset = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-wait-after-off-touch-int-n", &tmp);
	spec_pdata->off_seq.touch_intn = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,pw-down-period", &tmp);
	spec_pdata->down_period = !rc ? tmp : 0;

	rc = mdss_dsi_property_read_u32_var(np,
		"somc,ewu-rst-seq",
		(u32 **)&spec_pdata->ewu_seq.rst_seq,
		&spec_pdata->ewu_seq.seq_num);
	if (rc) {
		spec_pdata->ewu_seq.rst_seq = NULL;
		spec_pdata->ewu_seq.seq_num = 0;
		pr_debug("%s: Unable to read ewu sequence\n", __func__);
	}

	rc = of_property_read_u32(np,
		"somc,ewu-wait-after-touch-reset", &tmp);
	spec_pdata->ewu_seq.touch_reset = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
			"somc,mdss-dsi-disp-on-in-hs", &tmp);
	spec_pdata->disp_on_in_hs = !rc ? tmp : 0;

	rc = of_property_read_u32(np,
		"somc,mdss-dsi-wait-time-before-post-on-cmd", &tmp);
	spec_pdata->wait_time_before_post_on_cmd = !rc ? tmp : 0;

	panel_np = of_parse_phandle(np, "qcom,panel-supply-entries", 0);

	panel_type_name = of_node_full_name(panel_np);
	if (!strcmp(panel_type_name, NODE_OF_HYBRID))
			spec_pdata->panel_type = HYBRID_INCELL;
	if (!strcmp(panel_type_name, NODE_OF_FULL))
			spec_pdata->panel_type = FULL_INCELL;

	spec_pdata->chg_fps.enable = of_property_read_bool(np,
					"somc,change-fps-enable");
	if (spec_pdata->chg_fps.enable) {

		spec_pdata->input_fpks = pinfo->mipi.frame_rate * 1000;
		mdss_dsi_parse_dcs_cmds(np, &spec_pdata->fps_cmds,
					"somc,change-fps-command", NULL);

		rc = of_property_read_u32(np,
					"somc,driver-ic-vdisp", &tmp);
		if (rc) {
			pr_err("%s: Display vdisp not specified\n", __func__);
			goto error;
		}
		spec_pdata->chg_fps.dric_vdisp = tmp;

		spec_pdata->chg_fps.susres_mode = of_property_read_bool(np,
			"somc,change-fps-suspend-resume-mode");

		if (spec_pdata->panel_type == HYBRID_INCELL) {
			(void)mdss_dsi_property_read_u32_var(np,
				"somc,change-fps-send-pos",
				(u32 **)&spec_pdata->chg_fps.send_pos.pos,
				&spec_pdata->chg_fps.send_pos.num);

			rc = of_property_read_u32(np,
					"somc,driver-ic-rtn",  &tmp);
			if (rc) {
				pr_err("%s: DrIC rtn not specified\n",
								__func__);
				goto error;
			}
			spec_pdata->chg_fps.dric_rtn = tmp;

			(void)mdss_dsi_property_read_u32_var(np,
				"somc,change-fps-send-pos",
				(u32 **)&spec_pdata->chg_fps.send_pos.pos,
				&spec_pdata->chg_fps.send_pos.num);

			rc = of_property_read_u32(np,
						"somc,driver-ic-mclk", &tmp);
			if (rc) {
				pr_err("%s: DrIC mclk not specified\n", __func__);
				goto error;
			}
			spec_pdata->chg_fps.dric_mclk = tmp;

			rc = of_property_read_u32(np,
						"somc,driver-ic-vtouch", &tmp);
			if (rc) {
				pr_err("%s: DrIC vtouch not specified\n", __func__);
				goto error;
			}
			spec_pdata->chg_fps.dric_vtouch = tmp;

			rc = of_property_read_u32(np,
					"somc,change-fps-send-byte", &tmp);
			if (rc) {
				pr_err("%s: fps bytes send not specified\n",
								__func__);
				goto error;
			}
			spec_pdata->chg_fps.send_byte = tmp;

			rc = of_property_read_u32(np,
				"somc,change-fps-porch-mask-pos", &tmp);
			if (rc) {
				pr_warn("%s: fps mask position not specified\n",
								__func__);
				spec_pdata->chg_fps.mask_pos = 0;
			} else {
				spec_pdata->chg_fps.mask_pos = tmp;
				rc = of_property_read_u32(np,
					"somc,change-fps-porch-mask", &tmp);
				if (rc) {
					pr_warn("%s: fps mask not specified\n",
								__func__);
					spec_pdata->chg_fps.mask = 0x0;
				} else {
					spec_pdata->chg_fps.mask = tmp;
				}
			}

			rc = of_property_read_u32_array(np,
					"somc,change-fps-porch-range",
					spec_pdata->chg_fps.porch_range,
					FPS_PORCH_RNG_NUM);
			if (rc) {
				spec_pdata->chg_fps.porch_range[FPS_PORCH_RNG_MIN] = 0;
				spec_pdata->chg_fps.porch_range[FPS_PORCH_RNG_MAX] = 0;
			}
		} else if (spec_pdata->panel_type == FULL_INCELL) {
			(void)mdss_dsi_property_read_u32_var(np,
				"somc,change-fps-rtn-pos",
				(u32 **)&spec_pdata->chg_fps.send_pos.pos,
				&spec_pdata->chg_fps.send_pos.num);

			rc = of_property_read_u32(np,
					"somc,driver-ic-total-porch", &tmp);
			if (rc) {
				pr_err("%s: DrIC total_porch not specified\n",
								__func__);
				goto error;
			}
			spec_pdata->chg_fps.dric_total_porch = tmp;

			rc = of_property_read_u32(np,
						"somc,driver-ic-rclk", &tmp);
			if (rc) {
				pr_err("%s: DrIC rclk not specified\n", __func__);
				goto error;
			}
			spec_pdata->chg_fps.dric_rclk = tmp;

			rc = of_property_read_u32(np,
						"somc,driver-ic-vtp", &tmp);
			if (rc) {
				pr_err("%s: DrIC vtp not specified\n", __func__);
				goto error;
			}
			spec_pdata->chg_fps.dric_tp = tmp;
		} else {
			pr_err("%s:Read panel-type name failed. \n", __func__);
			goto error;
		}
	}

	/* Parsing lab/ibb register settings */
	labibb = &spec_pdata->labibb;

	memset(labibb, 0, sizeof(*labibb));
	if (of_find_property(np, "somc,lab-output-voltage", &tmp))
		labibb->labibb_ctrl_state |= OVR_LAB_VOLTAGE;

	if (of_find_property(np, "somc,ibb-output-voltage", &tmp))
		labibb->labibb_ctrl_state |= OVR_IBB_VOLTAGE;

	if (of_find_property(np, "somc,qpnp-lab-limit-maximum-current", &tmp))
		labibb->labibb_ctrl_state |= OVR_LAB_CURRENT_MAX;

	if (of_find_property(np, "somc,qpnp-ibb-limit-maximum-current", &tmp))
		labibb->labibb_ctrl_state |= OVR_IBB_CURRENT_MAX;

	if (of_find_property(np, "somc,qpnp-lab-max-precharge-time", &tmp))
		labibb->labibb_ctrl_state |= OVR_LAB_PRECHARGE_CTL;

	if (of_find_property(np, "somc,qpnp-lab-soft-start", &tmp))
		labibb->labibb_ctrl_state |= OVR_LAB_SOFT_START_CTL;

	if (of_find_property(np, "somc,qpnp-ibb-discharge-resistor", &tmp))
		labibb->labibb_ctrl_state |= OVR_IBB_SOFT_START_CTL;

	if (of_find_property(np, "somc,qpnp-lab-pull-down-enable", &tmp))
		labibb->labibb_ctrl_state |= OVR_LAB_PD_CTL;

	if (of_find_property(np, "somc,qpnp-ibb-pull-down-enable", &tmp))
		labibb->labibb_ctrl_state |= OVR_IBB_PD_CTL;

	labibb->lab_output_voltage = QPNP_REGULATOR_VSP_V_5P4V;
	if (((labibb->labibb_ctrl_state) & OVR_LAB_VOLTAGE)) {
		rc = of_property_read_u32(np, "somc,lab-output-voltage", &tmp);
		if (!rc)
			labibb->lab_output_voltage = tmp;
	}

	labibb->ibb_output_voltage = QPNP_REGULATOR_VSN_V_M5P4V;
	if (((labibb->labibb_ctrl_state) & OVR_IBB_VOLTAGE)) {
		rc = of_property_read_u32(np, "somc,ibb-output-voltage", &tmp);
		if (!rc)
			labibb->ibb_output_voltage = tmp;
	}

	labibb->lab_current_max = LAB_CURRENT_MAX;
	if (((labibb->labibb_ctrl_state) & OVR_LAB_CURRENT_MAX)) {
		rc = of_property_read_u32(np,
			"somc,qpnp-lab-limit-maximum-current", &tmp);
		if (!rc)
			labibb->lab_current_max = tmp;
	}

	labibb->ibb_current_max = IBB_CURRENT_MAX;
	if (((labibb->labibb_ctrl_state) & OVR_IBB_CURRENT_MAX)) {
		rc = of_property_read_u32(np,
			"somc,qpnp-ibb-limit-maximum-current", &tmp);
		if (!rc)
			labibb->ibb_current_max = tmp;
	}

	labibb->lab_fast_precharge_time = LAB_FAST_PRECHARGE_TIME;
	labibb->lab_fast_precharge_en = false;
	if (((labibb->labibb_ctrl_state) & OVR_LAB_PRECHARGE_CTL)) {
		rc = of_property_read_u32(np,
			"somc,qpnp-lab-max-precharge-time", &tmp);
		if (!rc)
			labibb->lab_fast_precharge_time = tmp;

		labibb->lab_fast_precharge_en = of_property_read_bool(np,
					"somc,qpnp-lab-max-precharge-enable");
	}

	labibb->lab_soft_start = LAB_SOFT_START_TIME;
	if (((labibb->labibb_ctrl_state) & OVR_LAB_SOFT_START_CTL)) {
		rc = of_property_read_u32(np,
			"somc,qpnp-lab-soft-start", &tmp);
		if (!rc)
			labibb->lab_soft_start = tmp;
	}

	labibb->ibb_soft_start = IBB_SOFT_START_RESISTOR;
	if (((labibb->labibb_ctrl_state) & OVR_IBB_SOFT_START_CTL)) {
		rc = of_property_read_u32(np,
			"somc,qpnp-ibb-discharge-resistor", &tmp);
		if (!rc)
			labibb->ibb_soft_start = tmp;
	}

	labibb->lab_pd_full = false;
	if (((labibb->labibb_ctrl_state) & OVR_LAB_PD_CTL))
		labibb->lab_pd_full = of_property_read_bool(np,
					"somc,qpnp-lab-full-pull-down");

	labibb->ibb_pd_full = false;
	if (((labibb->labibb_ctrl_state) & OVR_IBB_PD_CTL))
		labibb->ibb_pd_full = of_property_read_bool(np,
					"somc,qpnp-ibb-full-pull-down");

	return 0;

error:
	return -EINVAL;
}

static void conv_uv_data(char *data, int param_type, int *u_data, int *v_data)
{
	switch (param_type) {
	case CLR_DATA_UV_PARAM_TYPE_RENE_DEFAULT:
		*u_data = ((data[0] & 0x0F) << 2) |
			/* 4bit of data[0] higher data. */
			((data[1] >> 6) & 0x03);
			/* 2bit of data[1] lower data. */
		*v_data = (data[1] & 0x3F);
			/* Remainder 6bit of data[1] is effective as v_data. */
		break;
	case CLR_DATA_UV_PARAM_TYPE_NOVA_DEFAULT:
	case CLR_DATA_UV_PARAM_TYPE_RENE_SR:
		/* 6bit is effective as u_data */
		*u_data = data[0] & 0x3F;
		/* 6bit is effective as v_data */
		*v_data = data[1] & 0x3F;
		break;
	case CLR_DATA_UV_PARAM_TYPE_NOVA_AUO:
		/* 6bit is effective as u_data */
		*u_data = data[0] & 0x3F;
		/* 6bit is effective as v_data */
		*v_data = data[2] & 0x3F;
		break;
	default:
		pr_err("%s: Failed to conv type:%d\n", __func__, param_type);
		break;
	}
}

static int get_uv_param_len(int param_type, bool *short_response)
{
	int ret = 0;

	*short_response = false;
	switch (param_type) {
	case CLR_DATA_UV_PARAM_TYPE_RENE_DEFAULT:
		ret = CLR_DATA_REG_LEN_RENE_DEFAULT;
		break;
	case CLR_DATA_UV_PARAM_TYPE_NOVA_DEFAULT:
		ret = CLR_DATA_REG_LEN_NOVA_DEFAULT;
		break;
	case CLR_DATA_UV_PARAM_TYPE_NOVA_AUO:
		ret = CLR_DATA_REG_LEN_NOVA_AUO;
		break;
	case CLR_DATA_UV_PARAM_TYPE_RENE_SR:
		ret = CLR_DATA_REG_LEN_RENE_SR;
		*short_response = true;
		break;
	default:
		pr_err("%s: Failed to get param len\n", __func__);
		break;
	}

	return ret;
}

static void get_uv_data(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		int *u_data, int *v_data)
{
	struct dsi_cmd_desc *cmds = ctrl_pdata->spec_pdata->uv_read_cmds.cmds;
	int param_type = ctrl_pdata->spec_pdata->pcc_data.param_type;
	char buf[MDSS_DSI_LEN];
	char *pos = buf;
	int len;
	int i;
	bool short_response;
	struct dcs_cmd_req cmdreq;

	len = get_uv_param_len(param_type, &short_response);

	for (i = 0; i < ctrl_pdata->spec_pdata->uv_read_cmds.cmd_cnt; i++) {
		memset(&cmdreq, 0, sizeof(cmdreq));
		cmdreq.cmds = cmds;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
		cmdreq.rlen = short_response ? 1 : len;
		cmdreq.rbuf = ctrl_pdata->rx_buf.data;
		cmdreq.cb = NULL;

		mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

		memcpy(pos, ctrl_pdata->rx_buf.data, len);
		pos += len;
		cmds++;
	}
	conv_uv_data(buf, param_type, u_data, v_data);
}

static int find_color_area(struct mdp_pcc_cfg_data *pcc_config,
	struct mdss_pcc_data *pcc_data)
{
	int i;
	int ret = 0;

	for (i = 0; i < pcc_data->tbl_size; i++) {
		if (pcc_data->u_data < pcc_data->color_tbl[i].u_min)
			continue;
		if (pcc_data->u_data > pcc_data->color_tbl[i].u_max)
			continue;
		if (pcc_data->v_data < pcc_data->color_tbl[i].v_min)
			continue;
		if (pcc_data->v_data > pcc_data->color_tbl[i].v_max)
			continue;
		break;
	}
	pcc_data->tbl_idx = i;
	if (i >= pcc_data->tbl_size) {
		ret = -EINVAL;
		goto exit;
	}

	pcc_config->r.r = pcc_data->color_tbl[i].r_data;
	pcc_config->g.g = pcc_data->color_tbl[i].g_data;
	pcc_config->b.b = pcc_data->color_tbl[i].b_data;
exit:
	return ret;
}

static int mdss_dsi_panel_driver_calibrate_uv_data(
		u32 *rev_data, u32 *uv_data)
{
	int ret = 0;

	if (rev_data[1] != 0) {
		if (rev_data[0] == 0) {
			*uv_data += rev_data[1];
		 } else if (rev_data[0] == 1) {
			if (*uv_data < rev_data[1])
				*uv_data = 0;
			else
				*uv_data -= rev_data[1];
		} else {
			pr_err("%s: Invalid rev data.\n", __func__);
			ret = -EINVAL;
		}
	}

	return ret;
}

int mdss_dsi_panel_pcc_setup(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_pcc_data *pcc_data = NULL;
	int ret;
	u32 copyback;
	struct mdp_pcc_cfg_data pcc_config;
	u32 raw_u_data = 0, raw_v_data = 0;
	struct mdss_panel_specific_pdata *spec_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	spec_pdata = ctrl_pdata->spec_pdata;

	if (!spec_pdata->pcc_enable) {
		if (pdata->panel_info.dsi_master == pdata->panel_info.pdest)
			pr_info("%s (%d): pcc isn't enabled.\n",
				__func__, __LINE__);
		goto exit;
	}

	pcc_data = &ctrl_pdata->spec_pdata->pcc_data;

	mdss_dsi_op_mode_config(DSI_CMD_MODE, pdata);
	if (spec_pdata->pre_uv_read_cmds.cmds)
		mdss_dsi_panel_cmds_send(
				ctrl_pdata, &spec_pdata->pre_uv_read_cmds);
	if (spec_pdata->uv_read_cmds.cmds) {
		get_uv_data(ctrl_pdata, &pcc_data->u_data, &pcc_data->v_data);
		raw_u_data = pcc_data->u_data;
		raw_v_data = pcc_data->v_data;
	}

	ret = mdss_dsi_panel_driver_calibrate_uv_data(
			spec_pdata->rev_u, &pcc_data->u_data);
	if (ret)
		goto exit;

	ret = mdss_dsi_panel_driver_calibrate_uv_data(
			spec_pdata->rev_v, &pcc_data->v_data);
	if (ret)
		goto exit;

	if (pcc_data->u_data == 0 && pcc_data->v_data == 0) {
		pr_info("%s (%d): u,v is flashed 0.\n",
			__func__, __LINE__);
		goto exit;
	}
	if (!pcc_data->color_tbl) {
		if (pdata->panel_info.dsi_master == pdata->panel_info.pdest)
			pr_info("%s (%d): color_tbl isn't found.\n",
				__func__, __LINE__);
		goto exit;
	}

	memset(&pcc_config, 0, sizeof(struct mdp_pcc_cfg_data));
	ret = find_color_area(&pcc_config, pcc_data);
	if (ret) {
		pr_err("%s: failed to find color area.\n", __func__);
		goto exit;
	}

	if (pcc_data->color_tbl[pcc_data->tbl_idx].color_type != UNUSED) {
		pcc_config.block = MDP_LOGICAL_BLOCK_DISP_0;
		pcc_config.ops = MDP_PP_OPS_ENABLE | MDP_PP_OPS_WRITE;
		ret = mdss_mdp_pcc_config(&pcc_config, &copyback);
		if (ret != 0)
			pr_err("failed by settings of pcc data.\n");
	}

	pr_notice("%s (%d): raw_ud=%d raw_vd=%d "
		"ct=%d area=%d ud=%d vd=%d r=0x%08X g=0x%08X b=0x%08X",
		__func__, __LINE__,
		raw_u_data, raw_v_data,
		pcc_data->color_tbl[pcc_data->tbl_idx].color_type,
		pcc_data->color_tbl[pcc_data->tbl_idx].area_num,
		pcc_data->u_data, pcc_data->v_data,
		pcc_data->color_tbl[pcc_data->tbl_idx].r_data,
		pcc_data->color_tbl[pcc_data->tbl_idx].g_data,
		pcc_data->color_tbl[pcc_data->tbl_idx].b_data);

exit:
	return 0;
}

struct mdss_panel_specific_pdata *mdss_panel2spec_pdata(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	return ctrl_pdata->spec_pdata;
}

static u32 ts_diff_ms(struct timespec lhs, struct timespec rhs)
{
	struct timespec tdiff;
	s64 nsec;
	u32 msec;

	tdiff = timespec_sub(lhs, rhs);
	nsec = timespec_to_ns(&tdiff);
	msec = (u32)nsec;
	do_div(msec, NSEC_PER_MSEC);

	return msec;
}

static struct fps_data *mdss_dsi_panel_driver_get_fps_address(fps_type type)
{
	switch (type) {
	case FPSD:
		return &fpsd;
	case VPSD:
		return &vpsd;
	default:
		pr_err("%s: select Failed!\n", __func__);
		return NULL;
	}
}

static void update_fps_data(struct fps_data *fps)
{
	if (mutex_trylock(&fps->fps_lock)) {
		u32 fpks = 0;
		u32 ms_since_last = 0;
		u32 num_frames;
		struct timespec tlast = fps->timestamp_last;
		struct timespec tnow;
		u32 msec;

		getrawmonotonic(&tnow);
		msec = ts_diff_ms(tnow, tlast);
		fps->timestamp_last = tnow;

		fps->interval_ms = msec;
		fps->frame_counter++;
		num_frames = fps->frame_counter - fps->frame_counter_last;

		fps->fa[fps->fps_array_cnt].frame_nbr = fps->frame_counter;
		fps->fa[fps->fps_array_cnt].time_delta = msec;
		fps->fa_last_array_pos = fps->fps_array_cnt;
		(fps->fps_array_cnt)++;
		if (fps->fps_array_cnt >= DEFAULT_FPS_ARRAY_SIZE)
			fps->fps_array_cnt = 0;

		ms_since_last = ts_diff_ms(tnow, fps->fpks_ts_last);

		if (num_frames > 1 && ms_since_last >= fps->log_interval) {
			fpks = (num_frames * 1000000) / ms_since_last;
			fps->fpks_ts_last = tnow;
			fps->frame_counter_last = fps->frame_counter;
			fps->fpks = fpks;
		}
		mutex_unlock(&fps->fps_lock);
	}
}

static void mdss_dsi_panel_driver_fps_data_init(fps_type type)
{
	struct fps_data *fps = mdss_dsi_panel_driver_get_fps_address(type);

	if (!fps) {
		pr_err("%s: select Failed!\n", __func__);
		return;
	}

	fps->frame_counter = 0;
	fps->frame_counter_last = 0;
	fps->log_interval = DEFAULT_FPS_LOG_INTERVAL;
	fps->fpks = 0;
	fps->fa_last_array_pos = 0;
	fps->vps_en = false;
	getrawmonotonic(&fps->timestamp_last);
	mutex_init(&fps->fps_lock);
}

void mdss_dsi_panel_driver_fps_data_update(
		struct msm_fb_data_type *mfd, fps_type type)
{
	struct fps_data *fps = mdss_dsi_panel_driver_get_fps_address(type);

	if (!fps) {
		pr_err("%s: select Failed!\n", __func__);
		return;
	}

	if (mfd->index == 0)
		update_fps_data(fps);
}

struct fps_data mdss_dsi_panel_driver_get_fps_data(void)
{
	return fpsd;
}

struct fps_data mdss_dsi_panel_driver_get_vps_data(void)
{
	return vpsd;
}

static void vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t t)
{
	struct msm_fb_data_type *mfd = ctl->mfd;

	mdss_dsi_panel_driver_fps_data_update(mfd, VPSD);
}

static void mdss_dsi_panel_driver_vsync_handler_init(void)
{
	vs_handle.vsync_handler = NULL;
}

ssize_t mdss_dsi_panel_driver_vsyncs_per_ksecs_store(struct device *dev,
			 const char *buf, size_t count)
{
	int ret = count;
	long vps_en;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_mdp_ctl *ctl = mdata->ctl_off;

	if (kstrtol(buf, 10, &vps_en)) {
		dev_err(dev, "%s: Error, buf = %s\n", __func__, buf);
		ret = -EINVAL;
		goto exit;
	}

	vs_handle.vsync_handler = (mdp_vsync_handler_t)vsync_handler;
	vs_handle.cmd_post_flush = false;

	if (vps_en) {
		vs_handle.enabled = false;
		if (!vpsd.vps_en && (ctl->ops.add_vsync_handler)) {
			ctl->ops.add_vsync_handler(ctl, &vs_handle);
			vpsd.vps_en = true;
			pr_info("%s: vsyncs_per_ksecs is valid\n", __func__);
		}
	} else {
		vs_handle.enabled = true;
		if (vpsd.vps_en && (ctl->ops.remove_vsync_handler)) {
			ctl->ops.remove_vsync_handler(ctl, &vs_handle);
			vpsd.vps_en = false;
			fpsd.fpks = 0;
			pr_info("%s: vsyncs_per_ksecs is invalid\n", __func__);
		}
	}
exit:
	return ret;
}

void mdss_dsi_panel_driver_labibb_vreg_init(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	bool cont_splash_enabled = false;
	int ret;
	int min_uV, max_uV = 0;
	struct mdss_panel_info *pinfo = NULL;
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	struct mdss_panel_labibb_data *labibb = NULL;
	struct dss_vreg lab_vreg_config, ibb_vreg_config;

	pinfo = &ctrl_pdata->panel_data.panel_info;
	if (!pinfo) {
		pr_err("%s: Invalid panel data\n", __func__);
		return;
	}

	cont_splash_enabled = ctrl_pdata->mdss_util->panel_intf_status(
					pinfo->pdest, MDSS_PANEL_INTF_DSI);
	if (cont_splash_enabled) {
		pr_notice("%s: lab/ibb vreg already initialized\n", __func__);
		return;
	}

	spec_pdata = ctrl_pdata->spec_pdata;
	if (!spec_pdata) {
		pr_err("%s: Invalid specific panel data\n", __func__);
		return;
	}

	labibb = &(spec_pdata->labibb);
	if (!labibb) {
		pr_err("%s: Invalid regulator settings\n", __func__);
		return;
	}

	/*
	 * Get lab/ibb info.
	 */
	ret = mdss_dsi_panel_driver_vreg_name_to_config(ctrl_pdata,
						&lab_vreg_config, "lab");
	if (ret) {
		pr_err("%s: lab not registered\n", __func__);
		return;
	}

	ret = mdss_dsi_panel_driver_vreg_name_to_config(ctrl_pdata,
						&ibb_vreg_config, "ibb");
	if (ret) {
		pr_err("%s: ibb not registered\n", __func__);
		return;
	}

	/*
	 * Set lab/ibb voltage.
	 */
	if (((labibb->labibb_ctrl_state) & OVR_LAB_VOLTAGE)) {
		min_uV = labibb->lab_output_voltage;
		max_uV = min_uV;
		ret = regulator_set_voltage(lab_vreg_config.vreg,
							min_uV, max_uV);
		if (ret)
			pr_err("%s: Unable to configure of lab voltage.\n", __func__);
	}

	if (((labibb->labibb_ctrl_state) & OVR_IBB_VOLTAGE)) {
		min_uV = labibb->ibb_output_voltage;
		max_uV = min_uV;
		ret = regulator_set_voltage(ibb_vreg_config.vreg, min_uV, max_uV);
		if (ret)
			pr_err("%s: Unable to configure of ibb voltage.\n",
								__func__);
	}

	/*
	 * Set lab/ibb current max
	 */
	if (((labibb->labibb_ctrl_state) & OVR_LAB_CURRENT_MAX)) {
		ret = qpnp_lab_set_current_max(lab_vreg_config.vreg,
						labibb->lab_current_max);
		if (ret)
			pr_err("%s: Unable to configure of lab current_max.\n",
								__func__);
	}

	if (((labibb->labibb_ctrl_state) & OVR_IBB_CURRENT_MAX)) {
		ret = qpnp_ibb_set_current_max(ibb_vreg_config.vreg,
						labibb->ibb_current_max);
		if (ret)
			pr_err("%s: Unable to configure of ibb current_max.\n",
								__func__);
	}

	/*
	 * Set lab precharge
	 */
	if (((labibb->labibb_ctrl_state) & OVR_LAB_PRECHARGE_CTL)) {
		ret = qpnp_lab_set_precharge(lab_vreg_config.vreg,
					labibb->lab_fast_precharge_time,
					labibb->lab_fast_precharge_en);
		if (ret)
			pr_err("%s: Unable to configure of lab precharge.\n",
								__func__);
	}

	/*
	 * Set lab/ibb soft-start control
	 */
	if (((labibb->labibb_ctrl_state) & OVR_LAB_SOFT_START_CTL)) {
		ret = qpnp_lab_set_soft_start(lab_vreg_config.vreg,
						labibb->lab_soft_start);
		if (ret)
			pr_err("%s: Unable to configure of lab soft-start.\n",
								__func__);
	}

	if (((labibb->labibb_ctrl_state) & OVR_IBB_SOFT_START_CTL)) {
		ret = qpnp_ibb_set_soft_start(ibb_vreg_config.vreg,
						labibb->ibb_soft_start);
		if (ret)
			pr_err("%s: Unable to configure of ibb soft-start.\n",
							__func__);
	}

	/*
	 * Set lab/ibb pull-down control
	 */
	if (((labibb->labibb_ctrl_state) & OVR_LAB_PD_CTL)) {
		ret = qpnp_lab_set_pull_down(lab_vreg_config.vreg,
						labibb->lab_pd_full);
		if (ret)
			pr_err("%s: Unable to configure of lab pull-down.\n",
								__func__);
	}

	if (((labibb->labibb_ctrl_state) & OVR_IBB_PD_CTL)) {
		ret = qpnp_ibb_set_pull_down(ibb_vreg_config.vreg,
						labibb->ibb_pd_full);
		if (ret)
			pr_err("%s: Unable to configure of ibb pull-down.\n",
								__func__);
	}
}

static void mdss_dsi_panel_driver_fps_cmd_send(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		u32 dfpks_rev, int dfpks) {
	char dfps = (char)(dfpks_rev / KSEC);
	struct mdss_panel_specific_pdata *spec_pdata = ctrl_pdata->spec_pdata;
	struct mdss_panel_info *pinfo = &ctrl_pdata->panel_data.panel_info;

	pinfo->mipi.frame_rate = dfps;

	if (!spec_pdata->chg_fps.susres_mode) {
		pr_debug("%s: fps change sequence\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl_pdata,
				&ctrl_pdata->spec_pdata->fps_cmds);
	}
	pr_info("%s: change fpks=%d\n", __func__, dfpks);

	pinfo->new_fps		= dfps;
	spec_pdata->input_fpks	= dfpks;
}

static int mdss_dsi_panel_driver_fps_calc_rtn(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata, int dfpks) {
	u32 dfpks_rev;
	u32 vtotal_porch, vdisp;
	u32 vrclk, vtp;
	u32 cmds, payload;
	struct mdss_panel_specific_pdata *spec_pdata = ctrl_pdata->spec_pdata;
	u16 rtn;
	int i, j, byte_cnt;
	char send_rtn[sizeof(u16)] = {0};

	vtotal_porch = spec_pdata->chg_fps.dric_total_porch;
	vdisp = spec_pdata->chg_fps.dric_vdisp;

	vrclk = spec_pdata->chg_fps.dric_rclk;
	vtp = spec_pdata->chg_fps.dric_tp;

	rtn = (u16)(
		(vrclk * KSEC) /
		(dfpks * (vdisp + vtotal_porch + vtp))
	);
	dfpks_rev = (u32)(
		(vrclk * KSEC) /
		(rtn * (vdisp + vtotal_porch + vtp))
	);

	pr_debug("%s: porch=%d vdisp=%d vtp=%d vrclk=%d rtn=0x%x\n",
		__func__, vtotal_porch, vdisp, vtp, vrclk, rtn);

	for (i = 0; i < sizeof(send_rtn) ; i++) {
		send_rtn[i] = (char)(rtn & 0x00FF);
		pr_debug("%s: send_rtn[%d]=0x%x\n",
				__func__, i, send_rtn[i]);
		if (rtn > 0xFF) {
			rtn = (rtn >> 8);
		} else {
			byte_cnt = i;
			break;
		}
	}

	for (i = 0; i < (spec_pdata->chg_fps.send_pos.num / 2); i++) {
		cmds = spec_pdata->chg_fps.send_pos.pos[(i * 2)];
		payload = spec_pdata->chg_fps.send_pos.pos[(i * 2) + 1];
		for (j = 0; j <= byte_cnt ; j++)
			CHANGE_PAYLOAD(cmds, payload + j) = send_rtn[byte_cnt - j];
	}

	mdss_dsi_panel_driver_fps_cmd_send(ctrl_pdata, dfpks_rev, dfpks);

	return 0;
}

static int mdss_dsi_panel_driver_fps_calc_porch
		(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int dfpks) {
	u64 vmclk;
	u64 vtouch;
	u32 dfpks_rev;
	u32 vdisp;
	u32 cmds, payload;
	u32 porch_range_max = 0;
	u32 porch_range_min = 0;

	int i, j;
	u16 porch_calc = 0;
	u16 send_byte;
	u16 rtn;
	u8 mask_pos;
	char mask;
	char porch[2] = {0};
	char send[10] = {0};
	struct mdss_panel_specific_pdata *spec_pdata = ctrl_pdata->spec_pdata;

	rtn = spec_pdata->chg_fps.dric_rtn;
	vdisp = spec_pdata->chg_fps.dric_vdisp;
	vtouch = spec_pdata->chg_fps.dric_vtouch;
	vmclk = (u64)spec_pdata->chg_fps.dric_mclk;
	send_byte = spec_pdata->chg_fps.send_byte;
	mask_pos = spec_pdata->chg_fps.mask_pos;
	mask = spec_pdata->chg_fps.mask;
	porch_range_max = spec_pdata->chg_fps.porch_range[FPS_PORCH_RNG_MAX];
	porch_range_min = spec_pdata->chg_fps.porch_range[FPS_PORCH_RNG_MIN];

	porch_calc = (u16)((
			(((PSEC * KSEC) - (KSEC * vtouch * (u64)dfpks)) /
			((u64)dfpks * vmclk * (u64)rtn))
			- (u64)vdisp) / 2);

	if (porch_range_max > 0) {
		if ((porch_calc < porch_range_min)
		||  (porch_calc > porch_range_max)) {
			pr_err("%s: Not supported. porch:%d\n",
				__func__, porch_calc);
			return -EINVAL;
		}
	}

	dfpks_rev = (u32)(
		(PSEC * KSEC) /
		(vmclk * rtn * (vdisp + porch_calc) + vtouch));

	pr_debug("%s: porch=%d vdisp=%d vtouch=%llu vmclk=%llu rtn=0x%x\n",
		__func__, porch_calc, vdisp, vtouch, vmclk, rtn);

	for (i = 0; i < 2 ; i++) {
		porch[i] = (char)(porch_calc & 0x00FF);
		pr_debug("%s: porch[%d]=0x%x\n", __func__, i, porch[i]);
		porch_calc = (porch_calc >> 8);
	}

	for (i = 0; i < send_byte; i = i + 2) {
		memcpy(send + i, porch, sizeof(char));
		memcpy(send + i + 1, porch + 1, sizeof(char));
	}

	for (i = 0; i < (spec_pdata->chg_fps.send_pos.num / 2); i++) {
		cmds = spec_pdata->chg_fps.send_pos.pos[(i * 2)];
		payload = spec_pdata->chg_fps.send_pos.pos[(i * 2) + 1];
		for (j = 0; j < send_byte ; j++) {
			if (j == mask_pos)
				send[j] = (mask | send[j]);
			CHANGE_PAYLOAD(cmds, payload + j) = send[j];
			pr_debug("%s: fps_cmds.cmds[%d].payload[%d]) = 0x%x\n",
				__func__, cmds, payload + j,
				spec_pdata->fps_cmds.cmds[cmds].payload[payload + j]);
		}
	}

	mdss_dsi_panel_driver_fps_cmd_send(ctrl_pdata, dfpks_rev, dfpks);

	return 0;
}

static int mdss_dsi_panel_chg_fps_calc
		(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int dfpks) {
	int ret = -EINVAL;
	struct mdss_panel_specific_pdata *spec_pdata = NULL;

	spec_pdata = ctrl_pdata->spec_pdata;
	if (!spec_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		return ret;
	}

	if (spec_pdata->panel_type == HYBRID_INCELL)
		ret = mdss_dsi_panel_driver_fps_calc_porch(ctrl_pdata, dfpks);
	else if (spec_pdata->panel_type == FULL_INCELL)
		ret = mdss_dsi_panel_driver_fps_calc_rtn(ctrl_pdata, dfpks);
	else
		pr_err("%s: Invalid type data\n", __func__);

	return ret;
}

static int mdss_dsi_panel_chg_fps_check_state
		(struct mdss_dsi_ctrl_pdata *ctrl, int dfpks) {
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct msm_fb_data_type *mfd = mdata->ctl_off->mfd;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;
	struct mdss_dsi_ctrl_pdata *sctrl = NULL;
	int rc = 0;

	if (!mdp5_data->ctl || !mdp5_data->ctl->power_state)
		goto error;

	if (dfpks == ctrl->spec_pdata->input_fpks) {
		pr_info("%s: fpks is already %d\n", __func__, dfpks);
		goto end;
	}

	if ((pinfo->mipi.mode == DSI_CMD_MODE) &&
			(!ctrl->spec_pdata->fps_cmds.cmd_cnt))
		goto cmd_cnt_err;

	if (!display_onoff_state)
		goto disp_onoff_state_err;

	if (mdss_dsi_sync_wait_enable(ctrl)) {
		sctrl = mdss_dsi_get_other_ctrl(ctrl);
		if (sctrl) {
			if ((pinfo->mipi.mode == DSI_CMD_MODE) &&
				(!sctrl->spec_pdata->fps_cmds.cmd_cnt))
				goto cmd_cnt_err;

			if (!display_onoff_state)
				goto disp_onoff_state_err;

			if (mdss_dsi_sync_wait_trigger(ctrl)) {
				rc = mdss_dsi_panel_chg_fps_calc(sctrl, dfpks);
				if (rc < 0)
					goto end;
				rc = mdss_dsi_panel_chg_fps_calc(ctrl, dfpks);
			} else {
				rc = mdss_dsi_panel_chg_fps_calc(ctrl, dfpks);
				if (rc < 0)
					goto end;
				rc = mdss_dsi_panel_chg_fps_calc(sctrl, dfpks);
			}
		} else {
			rc = mdss_dsi_panel_chg_fps_calc(ctrl, dfpks);
		}
	} else {
		rc = mdss_dsi_panel_chg_fps_calc(ctrl, dfpks);
	}
end:
	return rc;
cmd_cnt_err:
	pr_err("%s: change fps isn't supported\n", __func__);
	return -EINVAL;
disp_onoff_state_err:
	pr_err("%s: Disp-On is not yet completed. Please retry\n", __func__);
	return -EINVAL;
error:
	return -EINVAL;
}

ssize_t mdss_dsi_panel_driver_change_fpks_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	int dfpks, rc;

	rc = kstrtoint(buf, 10, &dfpks);
	if (rc < 0) {
		pr_err("%s: Error, buf = %s\n", __func__, buf);
		return rc;
	}

	if (dfpks < 1000 * CHANGE_FPS_MIN
			|| dfpks > 1000 * CHANGE_FPS_MAX) {
		pr_err("%s: invalid value for change_fpks buf = %s\n",
				 __func__, buf);
		return -EINVAL;
	}

	rc = mdss_dsi_panel_chg_fps_check_state(ctrl_pdata, dfpks);
	if (rc) {
		pr_err("%s: Error, rc = %d\n", __func__, rc);
		return rc;
	}
	return count;
}

ssize_t mdss_dsi_panel_driver_change_fpks_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct msm_fb_data_type *mfd = mdata->ctl_off->mfd;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);

	if (!mdp5_data->ctl || !mdp5_data->ctl->power_state)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		ctrl_pdata->spec_pdata->input_fpks);
}

ssize_t mdss_dsi_panel_driver_change_fps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	int dfps, dfpks, rc;

	rc = kstrtoint(buf, 10, &dfps);
	if (rc < 0) {
		pr_err("%s: Error, buf = %s\n", __func__, buf);
		return rc;
	}

	if (dfps >= 1000 * CHANGE_FPS_MIN
			&& dfps <= 1000 * CHANGE_FPS_MAX) {
		dfpks = dfps;
	} else if (dfps >= CHANGE_FPS_MIN && dfps <= CHANGE_FPS_MAX) {
		dfpks = dfps * 1000;
	} else {
		pr_err("%s: invalid value for change_fps buf = %s\n",
				__func__, buf);
		return -EINVAL;
	}

	rc = mdss_dsi_panel_chg_fps_check_state(ctrl_pdata, dfpks);
	if (rc) {
		pr_err("%s: Error, rc = %d\n", __func__, rc);
		return rc;
	}
	return count;
}

ssize_t mdss_dsi_panel_driver_change_fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct msm_fb_data_type *mfd = mdata->ctl_off->mfd;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);

	if (!mdp5_data->ctl || !mdp5_data->ctl->power_state)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
		ctrl_pdata->spec_pdata->input_fpks / 1000);
}

void mdss_dsi_panel_driver_check_splash_enable(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->panel_data.panel_info.cont_splash_enabled)
		display_onoff_state = true;
	else
		display_onoff_state = false;
}

static void mdss_dsi_panel_driver_chg_fps_cmds_send
			(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	u32 fps_cmds, fps_payload;
	char rtn;

	spec_pdata = ctrl_pdata->spec_pdata;

	if (ctrl_pdata->panel_data.panel_info.mipi.mode == DSI_CMD_MODE) {
		if (spec_pdata->fps_cmds.cmd_cnt) {
			fps_cmds = spec_pdata->chg_fps.send_pos.pos[0];
			fps_payload = spec_pdata->chg_fps.send_pos.pos[1];
			rtn = CHANGE_PAYLOAD(fps_cmds, fps_payload);
			pr_debug("%s: change fps sequence --- rtn = 0x%x\n",
				__func__, rtn);
			mdss_dsi_panel_cmds_send(ctrl_pdata,
						&ctrl_pdata->spec_pdata->fps_cmds);
		}
	}
}

void mdss_dsi_panel_driver_wait_before_post_on_cmds(
			struct mdss_dsi_ctrl_pdata *ctrl_pdata) {
	if (ctrl_pdata->spec_pdata->wait_time_before_post_on_cmd)
		msleep(ctrl_pdata->spec_pdata->wait_time_before_post_on_cmd);
}

void mdss_dsi_panel_driver_post_on_event(struct mdss_mdp_ctl *ctl)
{
	int rc = 0;

	rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_POST_PANEL_ON, NULL);
	WARN(rc, "intf %d post panel on error (%d)\n",
		ctl->intf_num, rc);

	rc = mdss_mdp_tearcheck_enable(ctl, true);
	WARN(rc, "intf %d tearcheck enable error (%d)\n",
		ctl->intf_num, rc);
}

bool mdss_dsi_panel_driver_check_disp_on_in_hs(struct mdss_mdp_ctl *ctl)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	bool res = false;

	spec_pdata = mdss_panel2spec_pdata(ctl->panel_data);
	if (spec_pdata) {
		if (spec_pdata->disp_on_in_hs)
			res = true;
		else
			mdss_dsi_panel_driver_post_on_event(ctl);
	} else {
		mdss_dsi_panel_driver_post_on_event(ctl);
	}

	return res;
}

void mdss_dsi_panel_driver_fb_notifier_call_chain(
		struct msm_fb_data_type *mfd, int blank, bool type)
{
	struct fb_event event;

	if ((mfd->panel_info->type == MIPI_VIDEO_PANEL) ||
	    (mfd->panel_info->type == MIPI_CMD_PANEL)) {
		if (!mdss_dsi_panel_driver_is_incell_operation()) {
			event.info = mfd->fbi;
			event.data = &blank;

			if (type == FB_NOTIFIER_PRE) {
				fb_notifier_call_chain(FB_EXT_EARLY_EVENT_BLANK, &event);
			} else {
				fb_notifier_call_chain(FB_EXT_EVENT_BLANK, &event);
			}
		}
	}
}

void mdss_dsi_panel_driver_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	ctrl_pdata->spec_pdata->pcc_setup = mdss_dsi_panel_pcc_setup;
	mdss_dsi_panel_driver_fps_data_init(FPSD);
	mdss_dsi_panel_driver_fps_data_init(VPSD);
	mdss_dsi_panel_driver_vsync_handler_init();
}

void mdss_dsi_panel_driver_off(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_mdp_ctl *ctl = mdata->ctl_off;

	ctrl_pdata->spec_pdata->black_screen_off(ctrl_pdata);

	vs_handle.vsync_handler = (mdp_vsync_handler_t)vsync_handler;
	vs_handle.cmd_post_flush = false;
	vs_handle.enabled = true;
	if (vpsd.vps_en && (ctl->ops.remove_vsync_handler)) {
		ctl->ops.remove_vsync_handler(ctl, &vs_handle);
		vpsd.vps_en = false;
		fpsd.fpks = 0;
		pr_info("%s: vsyncs_per_ksecs is invalid\n", __func__);
	}
	display_onoff_state = false;
}

void mdss_dsi_panel_driver_post_on(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	mdss_dsi_panel_driver_chg_fps_cmds_send(ctrl_pdata);

	display_onoff_state = true;
}
