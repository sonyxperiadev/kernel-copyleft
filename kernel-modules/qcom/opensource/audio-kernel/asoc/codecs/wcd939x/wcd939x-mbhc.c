// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asoc/wcdcal-hwdep.h>
#include <asoc/wcd-mbhc-v2-api.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include "wcd939x-registers.h"
#include "internal.h"
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
#include <linux/soc/qcom/wcd939x-i2c.h>
#endif

#define WCD939X_ZDET_SUPPORTED          true
/* Z value defined in milliohm */
#define WCD939X_ZDET_VAL_32             32000
#define WCD939X_ZDET_VAL_400            400000
#define WCD939X_ZDET_VAL_1200           1200000
#define WCD939X_ZDET_VAL_100K           100000000
/* Z floating defined in ohms */
#define WCD939X_ZDET_FLOATING_IMPEDANCE 0x0FFFFFFE

#define WCD939X_ZDET_NUM_MEASUREMENTS   900
#define WCD939X_MBHC_GET_C1(c)          ((c & 0xC000) >> 14)
#define WCD939X_MBHC_GET_X1(x)          (x & 0x3FFF)
/* Z value compared in milliOhm */
#define WCD939X_MBHC_IS_SECOND_RAMP_REQUIRED(z) false
#define WCD939X_MBHC_ZDET_CONST         (1071 * 1024)
#define WCD939X_MBHC_MOISTURE_RREF      R_24_KOHM

#define OHMS_TO_MILLIOHMS 1000
#define SLOPE_FACTOR_SCALER 10000
#define FLOAT_TO_FIXED_XTALK (1UL << 16)
#define MAX_XTALK_ALPHA 255
#define MIN_RL_EFF_MOHMS 1
#define MAX_RL_EFF_MOHMS 900000
#define HD2_CODE_BASE_VALUE 0x1D
#define HD2_CODE_INV_RESOLUTION 4201025
#define FLOAT_TO_FIXED_LINEARIZER (1UL << 12)
#define MIN_TAP_OFFSET -1023
#define MAX_TAP_OFFSET 1023
#define MIN_TAP 0
#define MAX_TAP 1023
#define RDOWN_TIMER_PERIOD_MSEC 100

#define WCD_USBSS_WRITE true
#define WCD_USBSS_READ false
#define ZDET_SE 0
#define ZDET_DIFF 1
#define WCD_USBSS_EXT_LIN_EN 0x3D
#define WCD_USBSS_EXT_SW_CTRL_1 0x43
#define WCD_USBSS_MG1_BIAS 0x25
#define WCD_USBSS_MG2_BIAS 0x29

#define SE_SLOPE_MEAS_BIAS 10000
#define DIFF_SLOPE_MEAS_BIAS 20000
#define XTALK_CH_REG_ADDR_DELTA 4
#define NUM_DIFF_MEAS 2
#define ZDET_SE_MAX_MOHMS 600000
#define ZDET_ACC_LMT_MOHMS 100000
#define R_CONN_PAR_LOAD_POS_MOHMS 7895
#define LINEARIZER_DEFAULT_TAP 0xE8
#define GND_EXT_FET_MAX_MOHMS 2000

struct zdet_dnl_entry {
	u8  base_val_ohms;
	s16 se_corr_mohms;
	s16 diff_corr_mohms;
};

static const struct zdet_dnl_entry zdet_dnl_table[] = {
	{  0,    0,    0},
	{  5,   56,   13},
	{ 10,   60,   34},
	{ 15,   13,   -4},
	{ 20,   21,   14},
	{ 25,  -16,  -20},
	{ 30,    5,    7},
	{ 35,  -46,  -90},
	{ 40,   -4,  -17},
	{ 45,  -74,  -40},
	{ 50,  -52,    3},
	{ 55,  -37,   -5},
	{ 60,    4,  -79},
	{ 65,  -34,  -82},
	{ 70, -105,  -33},
	{ 75,  -81,  -55},
	{ 80,  -39,   34},
	{ 85,  -37,   46},
	{ 90,  -51,   81},
	{ 95,   14,  132},
	{100,  101,  197},
	{105,  150,  247},
	{110,  217,  245},
	{115,  232, -189},
	{120,  201, -146},
	{125, -152, -121},
	{130, -157,  -69},
	{135, -118,  -72},
	{140,  -54,  -24},
	{145,  -55,   51},
};

static struct wcd_mbhc_register
	wcd_mbhc_registers[WCD_MBHC_REG_FUNC_MAX] = {
	WCD_MBHC_REGISTER("WCD_MBHC_L_DET_EN",
			  WCD939X_MBHC_MECH, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_GND_DET_EN",
			  WCD939X_MBHC_MECH, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MECH_DETECTION_TYPE",
			  WCD939X_MBHC_MECH, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MIC_CLAMP_CTL",
			  WCD939X_PLUG_DETECT_CTL, 0x30, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_DETECTION_TYPE",
			  WCD939X_MBHC_ELECT, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_L_DET_PULL_UP_CTRL",
			  WCD939X_MECH_DET_CURRENT, 0x1F, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_L_DET_PULL_UP_COMP_CTRL",
			  WCD939X_MBHC_MECH, 0x04, 2, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_PLUG_TYPE",
			  WCD939X_MBHC_MECH, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_GND_PLUG_TYPE",
			  WCD939X_MBHC_MECH, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_SW_HPH_LP_100K_TO_GND",
			  WCD939X_MBHC_MECH, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_SCHMT_ISRC",
			  WCD939X_MBHC_ELECT, 0x06, 1, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_FSM_EN",
			  WCD939X_MBHC_ELECT, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_INSREM_DBNC",
			  WCD939X_PLUG_DETECT_CTL, 0x0F, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_BTN_DBNC",
			  WCD939X_CTL_1, 0x03, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_VREF",
			  WCD939X_CTL_2, 0x03, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HS_COMP_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_IN2P_CLAMP_STATE",
			  WCD939X_MBHC_RESULT_3, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MIC_SCHMT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_SCHMT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_SCHMT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_OCP_FSM_EN",
			  WCD939X_HPH_OCP_CTL, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_BTN_RESULT",
			  WCD939X_MBHC_RESULT_3, 0x07, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_BTN_ISRC_CTL",
			  WCD939X_MBHC_ELECT, 0x70, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_RESULT",
			  WCD939X_MBHC_RESULT_3, 0xFF, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MICB_CTRL",
			  WCD939X_MICB2, 0xC0, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPH_CNP_WG_TIME",
			  WCD939X_CNP_WG_TIME, 0xFF, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_PA_EN",
			  WCD939X_HPH, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_PA_EN",
			  WCD939X_HPH, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPH_PA_EN",
			  WCD939X_HPH, 0xC0, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_SWCH_LEVEL_REMOVE",
			  WCD939X_MBHC_RESULT_3, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_PULLDOWN_CTRL",
			  0, 0, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ANC_DET_EN",
			  WCD939X_CTL_BCS, 0x02, 1, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_FSM_STATUS",
			  WCD939X_FSM_STATUS, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MUX_CTL",
			  WCD939X_CTL_2, 0x70, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MOISTURE_STATUS",
			  WCD939X_FSM_STATUS, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_GND",
			  WCD939X_PA_CTL2, 0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_GND",
			  WCD939X_PA_CTL2, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_OCP_DET_EN",
			  WCD939X_L_TEST, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_OCP_DET_EN",
			  WCD939X_R_TEST, 0x01, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHL_OCP_STATUS",
			  WCD939X_INTR_STATUS_0, 0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_HPHR_OCP_STATUS",
			  WCD939X_INTR_STATUS_0, 0x20, 5, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_EN",
			  WCD939X_CTL_1, 0x08, 3, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_COMPLETE", WCD939X_FSM_STATUS,
			  0x40, 6, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_TIMEOUT", WCD939X_FSM_STATUS,
			  0x80, 7, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_RESULT", WCD939X_ADC_RESULT,
			  0xFF, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_MICB2_VOUT", WCD939X_MICB2, 0x3F, 0, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ADC_MODE",
			  WCD939X_CTL_1, 0x10, 4, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_DETECTION_DONE",
			  WCD939X_CTL_1, 0x04, 2, 0),
	WCD_MBHC_REGISTER("WCD_MBHC_ELECT_ISRC_EN",
			  WCD939X_MBHC_ZDET, 0x02, 1, 0),
};

static const struct wcd_mbhc_intr intr_ids = {
	.mbhc_sw_intr =  WCD939X_IRQ_MBHC_SW_DET,
	.mbhc_btn_press_intr = WCD939X_IRQ_MBHC_BUTTON_PRESS_DET,
	.mbhc_btn_release_intr = WCD939X_IRQ_MBHC_BUTTON_RELEASE_DET,
	.mbhc_hs_ins_intr = WCD939X_IRQ_MBHC_ELECT_INS_REM_LEG_DET,
	.mbhc_hs_rem_intr = WCD939X_IRQ_MBHC_ELECT_INS_REM_DET,
	.hph_left_ocp = WCD939X_IRQ_HPHL_OCP_INT,
	.hph_right_ocp = WCD939X_IRQ_HPHR_OCP_INT,
};

struct wcd939x_mbhc_zdet_param {
	u16 ldo_ctl;
	u16 noff;
	u16 nshift;
	u16 btn5;
	u16 btn6;
	u16 btn7;
};

static int wcd939x_mbhc_request_irq(struct snd_soc_component *component,
				  int irq, irq_handler_t handler,
				  const char *name, void *data)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	return wcd_request_irq(&wcd939x->irq_info, irq, name, handler, data);
}

static void wcd939x_mbhc_irq_control(struct snd_soc_component *component,
				   int irq, bool enable)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	if (enable)
		wcd_enable_irq(&wcd939x->irq_info, irq);
	else
		wcd_disable_irq(&wcd939x->irq_info, irq);
}

static int wcd939x_mbhc_free_irq(struct snd_soc_component *component,
			       int irq, void *data)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	wcd_free_irq(&wcd939x->irq_info, irq, data);

	return 0;
}

static void wcd939x_mbhc_clk_setup(struct snd_soc_component *component,
				 bool enable)
{
	if (enable)
		snd_soc_component_update_bits(component, WCD939X_CTL_1,
				    0x80, 0x80);
	else
		snd_soc_component_update_bits(component, WCD939X_CTL_1,
				    0x80, 0x00);
}

static int wcd939x_mbhc_btn_to_num(struct snd_soc_component *component)
{
	return snd_soc_component_read(component, WCD939X_MBHC_RESULT_3) & 0x7;
}

static void wcd939x_mbhc_mbhc_bias_control(struct snd_soc_component *component,
					 bool enable)
{
	if (enable)
		snd_soc_component_update_bits(component, WCD939X_MBHC_ELECT,
				    0x01, 0x01);
	else
		snd_soc_component_update_bits(component, WCD939X_MBHC_ELECT,
				    0x01, 0x00);
}

static void wcd939x_mbhc_program_btn_thr(struct snd_soc_component *component,
				       s16 *btn_low, s16 *btn_high,
				       int num_btn, bool is_micbias)
{
	int i;
	int vth;

	if (num_btn > WCD_MBHC_DEF_BUTTONS) {
		dev_err_ratelimited(component->dev, "%s: invalid number of buttons: %d\n",
			__func__, num_btn);
		return;
	}

	for (i = 0; i < num_btn; i++) {
		vth = ((btn_high[i] * 2) / 25) & 0x3F;
		snd_soc_component_update_bits(component, WCD939X_MBHC_BTN0 + i,
				    0xFC, vth << 2);
		dev_dbg(component->dev, "%s: btn_high[%d]: %d, vth: %d\n",
			__func__, i, btn_high[i], vth);
	}
}

static bool wcd939x_mbhc_lock_sleep(struct wcd_mbhc *mbhc, bool lock)
{
	struct snd_soc_component *component = mbhc->component;
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);

	wcd939x->wakeup((void*)wcd939x, lock);

	return true;
}

static int wcd939x_mbhc_register_notifier(struct wcd_mbhc *mbhc,
					struct notifier_block *nblock,
					bool enable)
{
	struct wcd939x_mbhc *wcd939x_mbhc;

	wcd939x_mbhc = container_of(mbhc, struct wcd939x_mbhc, wcd_mbhc);

	if (enable)
		return blocking_notifier_chain_register(&wcd939x_mbhc->notifier,
							nblock);
	else
		return blocking_notifier_chain_unregister(
				&wcd939x_mbhc->notifier, nblock);
}

static bool wcd939x_mbhc_micb_en_status(struct wcd_mbhc *mbhc, int micb_num)
{
	u8 val = 0;

	if (micb_num == MIC_BIAS_2) {
		val = ((snd_soc_component_read(mbhc->component,
								WCD939X_MICB2) & 0xC0)
			>> 6);
		if (val == 0x01)
			return true;
	}
	return false;
}

static bool wcd939x_mbhc_hph_pa_on_status(struct snd_soc_component *component)
{
	return (snd_soc_component_read(component, WCD939X_HPH) & 0xC0) ?
									true : false;
}

static void wcd939x_mbhc_hph_l_pull_up_control(
							struct snd_soc_component *component,
							int pull_up_cur)
{
	/* Default pull up current to 2uA */
	if (pull_up_cur > HS_PULLUP_I_OFF || pull_up_cur < HS_PULLUP_I_3P0_UA ||
	    pull_up_cur == HS_PULLUP_I_DEFAULT)
		pull_up_cur = HS_PULLUP_I_2P0_UA;

	dev_dbg(component->dev, "%s: HS pull up current:%d\n",
		__func__, pull_up_cur);

	snd_soc_component_update_bits(component,
				WCD939X_MECH_DET_CURRENT,
			    0x1F, pull_up_cur);
}

static int wcd939x_mbhc_request_micbias(struct snd_soc_component *component,
					int micb_num, int req)
{
	int ret = 0;

	ret = wcd939x_micbias_control(component, micb_num, req, false);

	return ret;
}

static void wcd939x_mbhc_micb_ramp_control(struct snd_soc_component *component,
					   bool enable)
{
	if (enable) {
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x1C, 0x0C);
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x80, 0x80);
	} else {
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x80, 0x00);
		snd_soc_component_update_bits(component, WCD939X_MICB2_RAMP,
				    0x1C, 0x00);
	}
}

static struct firmware_cal *wcd939x_get_hwdep_fw_cal(struct wcd_mbhc *mbhc,
						   enum wcd_cal_type type)
{
	struct wcd939x_mbhc *wcd939x_mbhc;
	struct firmware_cal *hwdep_cal;
	struct snd_soc_component *component = mbhc->component;

	wcd939x_mbhc = container_of(mbhc, struct wcd939x_mbhc, wcd_mbhc);

	if (!component) {
		pr_err_ratelimited("%s: NULL component pointer\n", __func__);
		return NULL;
	}
	hwdep_cal = wcdcal_get_fw_cal(wcd939x_mbhc->fw_data, type);
	if (!hwdep_cal)
		dev_err_ratelimited(component->dev, "%s: cal not sent by %d\n",
			__func__, type);

	return hwdep_cal;
}

static int wcd939x_mbhc_micb_ctrl_threshold_mic(
							struct snd_soc_component *component,
							int micb_num, bool req_en)
{
	struct wcd939x_pdata *pdata = dev_get_platdata(component->dev);
	int rc, micb_mv;

	if (micb_num != MIC_BIAS_2)
		return -EINVAL;
	/*
	 * If device tree micbias level is already above the minimum
	 * voltage needed to detect threshold microphone, then do
	 * not change the micbias, just return.
	 */
	if (pdata->micbias.micb2_mv >= WCD_MBHC_THR_HS_MICB_MV)
		return 0;

	micb_mv = req_en ? WCD_MBHC_THR_HS_MICB_MV : pdata->micbias.micb2_mv;

	rc = wcd939x_mbhc_micb_adjust_voltage(component, micb_mv, MIC_BIAS_2);

	return rc;
}

static inline void wcd939x_mbhc_get_result_params(struct wcd939x_priv *wcd939x,
						s16 *d1_a, u16 noff,
						int32_t *zdet)
{
	int i;
	int val, val1;
	s16 c1;
	s32 x1, d1;
	int32_t denom;
	int minCode_param[] = {
			3277, 1639, 820, 410, 205, 103, 52, 26
	};
	struct wcd939x_mbhc *wcd939x_mbhc = wcd939x->mbhc;

	regmap_update_bits(wcd939x->regmap, WCD939X_MBHC_ZDET, 0x20, 0x20);
	for (i = 0; i < WCD939X_ZDET_NUM_MEASUREMENTS; i++) {
		regmap_read(wcd939x->regmap, WCD939X_MBHC_RESULT_2, &val);
		if (val & 0x80)
			break;
	}
	val = val << 0x8;
	regmap_read(wcd939x->regmap, WCD939X_MBHC_RESULT_1, &val1);
	val |= val1;
	wcd939x_mbhc->rdown_prev_iter = val;
	regmap_update_bits(wcd939x->regmap, WCD939X_MBHC_ZDET, 0x20, 0x00);
	x1 = WCD939X_MBHC_GET_X1(val);
	c1 = WCD939X_MBHC_GET_C1(val);
	/* If ramp is not complete, give additional 5ms */
	if ((c1 < 2) && x1)
		usleep_range(5000, 5050);

	if (!c1 || !x1) {
		dev_dbg(wcd939x->dev,
			"%s: Impedance detect ramp error, c1=%d, x1=0x%x\n",
			__func__, c1, x1);
		goto ramp_down;
	}
	d1 = d1_a[c1];
	denom = (x1 * d1) - (1 << (14 - noff));
	if (denom > 0)
		*zdet = (WCD939X_MBHC_ZDET_CONST * 1000) / denom;
	else if (x1 < minCode_param[noff])
		*zdet = WCD939X_ZDET_FLOATING_IMPEDANCE;

	dev_dbg(wcd939x->dev, "%s: d1=%d, c1=%d, x1=0x%x, z_val=%d(milliOhm)\n",
		__func__, d1, c1, x1, *zdet);
ramp_down:
	i = 0;
	wcd939x_mbhc->rdown_timer_complete = false;
	mod_timer(&wcd939x_mbhc->rdown_timer, jiffies + msecs_to_jiffies(RDOWN_TIMER_PERIOD_MSEC));
	while (x1) {
		regmap_read(wcd939x->regmap,
				 WCD939X_MBHC_RESULT_1, &val);
		regmap_read(wcd939x->regmap,
				 WCD939X_MBHC_RESULT_2, &val1);
		val = val << 0x08;
		val |= val1;
		x1 = WCD939X_MBHC_GET_X1(val);
		i++;
		if (i == WCD939X_ZDET_NUM_MEASUREMENTS)
			break;
		if (wcd939x_mbhc->rdown_timer_complete && wcd939x_mbhc->rdown_prev_iter == val)
			break;
		wcd939x_mbhc->rdown_prev_iter = val;
	}
	del_timer(&wcd939x_mbhc->rdown_timer);
}

static void wcd939x_mbhc_zdet_ramp(struct snd_soc_component *component,
				 struct wcd939x_mbhc_zdet_param *zdet_param,
				 int32_t *zl, int32_t *zr, s16 *d1_a)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);
	int32_t zdet = 0;

	snd_soc_component_update_bits(component, WCD939X_ZDET_ANA_CTL, 0xF0,
				      0x80 | (zdet_param->ldo_ctl << 4));
	snd_soc_component_update_bits(component, WCD939X_MBHC_BTN5, 0xFC,
			    zdet_param->btn5);
	snd_soc_component_update_bits(component, WCD939X_MBHC_BTN6, 0xFC,
			    zdet_param->btn6);
	snd_soc_component_update_bits(component, WCD939X_MBHC_BTN7, 0xFC,
			    zdet_param->btn7);
	snd_soc_component_update_bits(component, WCD939X_ZDET_ANA_CTL,
				0x0F, zdet_param->noff);
	snd_soc_component_update_bits(component, WCD939X_ZDET_RAMP_CTL,
				0x0F, zdet_param->nshift);
	snd_soc_component_update_bits(component, WCD939X_ZDET_RAMP_CTL,
				0x70, 0x60); /*acc1_min_63 */

	if (!zl)
		goto z_right;
	/* Start impedance measurement for HPH_L */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x80, 0x80);
	dev_dbg(wcd939x->dev, "%s: ramp for HPH_L, noff = %d\n",
		__func__, zdet_param->noff);
	wcd939x_mbhc_get_result_params(wcd939x, d1_a, zdet_param->noff, &zdet);
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x80, 0x00);

	*zl = zdet;

z_right:
	if (!zr)
		return;
	/* Start impedance measurement for HPH_R */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x40, 0x40);
	dev_dbg(wcd939x->dev, "%s: ramp for HPH_R, noff = %d\n",
		__func__, zdet_param->noff);
	wcd939x_mbhc_get_result_params(wcd939x, d1_a, zdet_param->noff, &zdet);
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_ZDET, 0x40, 0x00);

	*zr = zdet;
}

static inline void wcd939x_wcd_mbhc_qfuse_cal(
					struct snd_soc_component *component,
					int32_t *z_val, int flag_l_r)
{
	s16 q1;
	int q1_cal;

	q1 = snd_soc_component_read(component,
			WCD939X_EFUSE_REG_21 + flag_l_r);
	if (q1 & 0x80)
		q1_cal = (10000 - ((q1 & 0x7F) * 10));
	else
		q1_cal = (10000 + (q1 * 10));

	if (q1_cal > 0) {
		if (*z_val < 200 * OHMS_TO_MILLIOHMS)
			*z_val = ((*z_val) * 10000) / q1_cal;
		else if (*z_val < 2000 * OHMS_TO_MILLIOHMS)
			*z_val = ((*z_val) * 1000) / q1_cal * 10;
		else if (*z_val < 20000 * OHMS_TO_MILLIOHMS)
			*z_val = ((*z_val) * 100) / q1_cal * 100;
	}
}

static void rdown_timer_callback(struct timer_list *timer)
{
	struct wcd939x_mbhc *wcd939x_mbhc = container_of(timer, struct wcd939x_mbhc, rdown_timer);

	wcd939x_mbhc->rdown_timer_complete = true;
}

static void update_hd2_codes(struct regmap *regmap, u32 r_gnd_res_tot_mohms, u32 r_load_eff_mohms)
{
	u64 hd2_delta = 0;

	if (!regmap)
		return;
	hd2_delta = (HD2_CODE_INV_RESOLUTION * (u64) r_gnd_res_tot_mohms +
		    FLOAT_TO_FIXED_XTALK * (u64) ((r_gnd_res_tot_mohms + r_load_eff_mohms) / 2)) /
		    (FLOAT_TO_FIXED_XTALK * (u64) (r_gnd_res_tot_mohms + r_load_eff_mohms));
	if (hd2_delta >= HD2_CODE_BASE_VALUE) {
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_L, 0x1F, 0x00);
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_R, 0x1F, 0x00);
	} else {
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_L, 0x1F,
				   HD2_CODE_BASE_VALUE - hd2_delta);
		regmap_update_bits(regmap, WCD939X_RDAC_HD2_CTL_R, 0x1F,
				   HD2_CODE_BASE_VALUE - hd2_delta);
	}
}

static u8 get_xtalk_scale(u32 gain)
{
	u8 i;
	int target, residue;

	if (gain == 0)
		return MAX_XTALK_SCALE;

	target = FLOAT_TO_FIXED_XTALK / ((int) gain);
	residue = target;

	for (i = 0; i <= MAX_XTALK_SCALE; i++) {
		residue = target - (1 << ((int)((u32) i)));
		if (residue < 0)
			return i;
	}
	return MAX_XTALK_SCALE;
}

static u8 get_xtalk_alpha(u32 gain, u8 scale)
{
	u32 two_exp_scale, round_offset, alpha;

	if (gain == 0)
		return MIN_XTALK_ALPHA;

	two_exp_scale = 1 << ((u32) scale);
	round_offset = FLOAT_TO_FIXED_XTALK / 2;
	alpha = (((gain * two_exp_scale - FLOAT_TO_FIXED_XTALK) * 255) + round_offset)
		    / FLOAT_TO_FIXED_XTALK;
	return (alpha <= MAX_XTALK_ALPHA) ? ((u8) alpha) : MAX_XTALK_ALPHA;
}

static void update_xtalk_scale_and_alpha(struct wcd939x_priv *wcd939x)
{
	u32 r_gnd_res_tot_mohms = 0, r_gnd_int_fet_mohms = 0, r_gnd_par_route1_mohms = 0;
	u32 xtalk_gain_l = 0, xtalk_gain_r = 0, r_load_eff_mohms = 0;
	u32 xtalk_gain_denom_l = 0, xtalk_gain_denom_r = 0, r7 = 0;
	struct wcd939x_pdata *pdata = dev_get_platdata(wcd939x->dev);

	if (!pdata || pdata->usbcss_hs.xtalk.xtalk_config == XTALK_NONE)
		return;

	/* Default xtalk values */
	pdata->usbcss_hs.xtalk.scale_l = MAX_XTALK_SCALE;
	pdata->usbcss_hs.xtalk.alpha_l = MIN_XTALK_ALPHA;
	pdata->usbcss_hs.xtalk.scale_r = MAX_XTALK_SCALE;
	pdata->usbcss_hs.xtalk.alpha_r = MIN_XTALK_ALPHA;

	/* Orientation-dependent ground impedance parameters */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU2_ORIENTATION_A) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_int_fet_mohms;
		r_gnd_par_route1_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_par_route1_mohms;
		r7 = pdata->usbcss_hs.gnd.sbu2.r7;
	} else if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU1_ORIENTATION_B) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_int_fet_mohms;
		r_gnd_par_route1_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_par_route1_mohms;
		r7 = pdata->usbcss_hs.gnd.sbu1.r7;
	} else {
		dev_dbg(wcd939x->dev, "%s: Using default scale and alpha values\n", __func__);
		return;
	}
#endif

	r_load_eff_mohms = (pdata->usbcss_hs.aud.l.r_load_eff_mohms +
			    pdata->usbcss_hs.aud.r.r_load_eff_mohms) / 2;

	if (pdata->usbcss_hs.xtalk.xtalk_config == XTALK_ANALOG) {
		/* Update HD2 codes for analog xtalk */
		update_hd2_codes(wcd939x->regmap, r_gnd_res_tot_mohms, r_load_eff_mohms);
	}

	/* Left channel */
	xtalk_gain_denom_l = pdata->usbcss_hs.aud.l.zval - r_gnd_int_fet_mohms -
			     r_gnd_par_route1_mohms + pdata->usbcss_hs.aud.l.r1;
	if (xtalk_gain_denom_l == 0) {
		dev_dbg(wcd939x->dev,
			"%s: Using default scale and alpha values for the left channel\n",
			__func__);
	} else {
		xtalk_gain_l = FLOAT_TO_FIXED_XTALK * pdata->usbcss_hs.gnd.r_common_gnd_mohms /
			       xtalk_gain_denom_l;
		/* Store scale and alpha values */
		pdata->usbcss_hs.xtalk.scale_l = get_xtalk_scale(xtalk_gain_l);
		pdata->usbcss_hs.xtalk.alpha_l = get_xtalk_alpha(xtalk_gain_l,
								 pdata->usbcss_hs.xtalk.scale_l);
	}

	/* Right channel */
	xtalk_gain_denom_r = pdata->usbcss_hs.aud.r.zval - r_gnd_int_fet_mohms -
			     r_gnd_par_route1_mohms + pdata->usbcss_hs.aud.r.r1;
	if (xtalk_gain_denom_r == 0) {
		dev_dbg(wcd939x->dev,
			"%s: Using default scale and alpha values for the right channel\n",
			__func__);
	} else {
		xtalk_gain_r = FLOAT_TO_FIXED_XTALK * pdata->usbcss_hs.gnd.r_common_gnd_mohms /
			       xtalk_gain_denom_r;
		pdata->usbcss_hs.xtalk.scale_r = get_xtalk_scale(xtalk_gain_r);
		pdata->usbcss_hs.xtalk.alpha_r = get_xtalk_alpha(xtalk_gain_r,
								 pdata->usbcss_hs.xtalk.scale_r);
	}

	/* Print relevant values */
	dev_dbg(wcd939x->dev, "%s: %s = %dmohms, %s = %dmohms, %s = %dmohms\n", __func__,
		"Left SE measurement", pdata->usbcss_hs.aud.l.zval,
		"right SE measurment", pdata->usbcss_hs.aud.r.zval,
		"differential measurement", pdata->usbcss_hs.zdiffval);
	dev_dbg(wcd939x->dev,
		"%s: %s = %dmohms, %s = %dmohms, %s = %dmohms, %s = %dmohms, %s = %dmohms\n",
		__func__, "R1_L", pdata->usbcss_hs.aud.l.r1, "R1_R", pdata->usbcss_hs.aud.r.r1,
		"R7", r7, "r_gnd_int_fet_mohms", r_gnd_int_fet_mohms, "r_common_gnd_mohms",
		pdata->usbcss_hs.gnd.r_common_gnd_mohms);
	dev_dbg(wcd939x->dev, "%s: %s = %d, %s = %d %s %d\n", __func__,
		"Xtalk gain (L->R)", xtalk_gain_l, "xtalk gain (R->L)", xtalk_gain_r,
		". To convert xtalk gain to floating point, divide by", FLOAT_TO_FIXED_XTALK);
}

static void update_ext_fet_res(struct wcd939x_pdata *pdata, u32 r_aud_ext_fet_mohms,
			       u32 r_gnd_ext_fet_mohms)
{
	if (!pdata)
		return;

	pdata->usbcss_hs.gnd.r_gnd_ext_fet_mohms = (r_gnd_ext_fet_mohms >
						    MAX_USBCSS_HS_IMPEDANCE_MOHMS)
						   ? MAX_USBCSS_HS_IMPEDANCE_MOHMS
						   : r_gnd_ext_fet_mohms;
	pdata->usbcss_hs.aud.l.r_aud_ext_fet_mohms = (r_aud_ext_fet_mohms >
						      MAX_USBCSS_HS_IMPEDANCE_MOHMS)
						     ? MAX_USBCSS_HS_IMPEDANCE_MOHMS
						     : r_aud_ext_fet_mohms;
	pdata->usbcss_hs.aud.r.r_aud_ext_fet_mohms = pdata->usbcss_hs.aud.l.r_aud_ext_fet_mohms;
	pdata->usbcss_hs.gnd.sbu1.r_gnd_res_tot_mohms = get_r_gnd_res_tot_mohms(
						     pdata->usbcss_hs.gnd.sbu1.r_gnd_int_fet_mohms,
						     pdata->usbcss_hs.gnd.r_gnd_ext_fet_mohms,
						     pdata->usbcss_hs.gnd.sbu1.r_gnd_par_tot_mohms);
	pdata->usbcss_hs.gnd.sbu2.r_gnd_res_tot_mohms = get_r_gnd_res_tot_mohms(
						     pdata->usbcss_hs.gnd.sbu2.r_gnd_int_fet_mohms,
						     pdata->usbcss_hs.gnd.r_gnd_ext_fet_mohms,
						     pdata->usbcss_hs.gnd.sbu2.r_gnd_par_tot_mohms);
	pdata->usbcss_hs.aud.l.r_aud_res_tot_mohms = get_r_aud_res_tot_mohms(
						  pdata->usbcss_hs.aud.l.r_aud_int_fet_mohms,
						  pdata->usbcss_hs.aud.l.r_aud_ext_fet_mohms,
						  pdata->usbcss_hs.aud.l.r_load_eff_mohms);
	pdata->usbcss_hs.aud.r.r_aud_res_tot_mohms = get_r_aud_res_tot_mohms(
						  pdata->usbcss_hs.aud.r.r_aud_int_fet_mohms,
						  pdata->usbcss_hs.aud.r.r_aud_ext_fet_mohms,
						  pdata->usbcss_hs.aud.r.r_load_eff_mohms);
}

static void get_linearizer_taps(struct wcd939x_pdata *pdata, u32 *aud_tap)
{
	u32 r_gnd_int_fet_mohms = 0, r_gnd_par_tot_mohms = 0;
	u32 v_aud1 = 0, v_aud2 = 0, aud_denom = 0;
	u32 r_load_eff_mohms = 0, r3 = 0, r_aud_ext_fet_mohms = 0, r_aud_int_fet_mohms = 0;
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	u32 r_gnd_res_tot_mohms = 0;
#endif

	if (!pdata)
		goto err_data;

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Orientation-dependent ground impedance parameters */
	if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU2_ORIENTATION_A) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_int_fet_mohms;
	} else if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU1_ORIENTATION_B) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_res_tot_mohms;
		r_gnd_int_fet_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_int_fet_mohms;
	} else {
		goto err_data;
	}
#endif

	r_load_eff_mohms = (pdata->usbcss_hs.aud.l.r_load_eff_mohms +
			    pdata->usbcss_hs.aud.r.r_load_eff_mohms) / 2;
	r3 = (pdata->usbcss_hs.aud.l.r3 + pdata->usbcss_hs.aud.r.r3) / 2;
	r_aud_ext_fet_mohms = (pdata->usbcss_hs.aud.l.r_aud_ext_fet_mohms +
			       pdata->usbcss_hs.aud.l.r_aud_ext_fet_mohms) / 2;
	r_aud_int_fet_mohms = (pdata->usbcss_hs.aud.l.r_aud_int_fet_mohms +
			       pdata->usbcss_hs.aud.l.r_aud_int_fet_mohms) / 2;

	aud_denom = (u32) (FLOAT_TO_FIXED_LINEARIZER +
			  (FLOAT_TO_FIXED_LINEARIZER * pdata->usbcss_hs.aud.k_aud_times_100 / 100));

	v_aud2 = r_load_eff_mohms - r3 + r_gnd_int_fet_mohms +
		 pdata->usbcss_hs.gnd.r_gnd_ext_fet_mohms + r_gnd_par_tot_mohms;
	v_aud1 = v_aud2 + r_aud_ext_fet_mohms;
	v_aud1 = FLOAT_TO_FIXED_LINEARIZER * v_aud1 / (v_aud1 + r_aud_int_fet_mohms);
	v_aud2 = FLOAT_TO_FIXED_LINEARIZER * v_aud2 /
					     (v_aud2 + r_aud_ext_fet_mohms + r_aud_int_fet_mohms);

	*aud_tap = (u32) ((s32) ((1000 * v_aud1 + 10 * pdata->usbcss_hs.aud.k_aud_times_100 * v_aud2
				  + aud_denom / 2) / aud_denom) +
				 pdata->usbcss_hs.aud.aud_tap_offset);
	if (*aud_tap > MAX_TAP)
		*aud_tap = MAX_TAP;
	else if (*aud_tap < MIN_TAP)
		*aud_tap = MIN_TAP;

	return;

err_data:
	*aud_tap = LINEARIZER_DEFAULT_TAP;
}

static void interpolate_zdet_val(uint32_t *z, s64 z_meas_bias_removed, s64 z_val_slope_corrected,
				 int lb, int flag_se_diff)
{
	s64 lb_to_z = 0, lb_to_ub = 0, z_to_ub = 0, lb_corr = 0, ub_corr = 0, z_interp = 0;

	/* If lb is the table upper bound, no interpolation needed, just use the lb corr factor */
	if ((lb + 1) >= ARRAY_SIZE(zdet_dnl_table)) {
		z_interp = (s64) ((flag_se_diff) ? (zdet_dnl_table[lb].diff_corr_mohms) :
						   (zdet_dnl_table[lb].se_corr_mohms));
		goto apply_interpolated_bias;
	}

	/* Set up interpolation */
	lb_to_ub = OHMS_TO_MILLIOHMS *
		   (s64) (u64) ((zdet_dnl_table[lb + 1].base_val_ohms -
				 zdet_dnl_table[lb].base_val_ohms));
	z_to_ub = (OHMS_TO_MILLIOHMS *
		   ((s64) (u64) (zdet_dnl_table[lb + 1].base_val_ohms))) - z_meas_bias_removed;
	lb_to_z = z_meas_bias_removed - (OHMS_TO_MILLIOHMS *
			      ((s64) (u64) (zdet_dnl_table[lb].base_val_ohms)));
	lb_corr = (s64) ((flag_se_diff) ? (zdet_dnl_table[lb].diff_corr_mohms) :
					  (zdet_dnl_table[lb].se_corr_mohms));
	ub_corr = (s64) ((flag_se_diff) ? (zdet_dnl_table[lb + 1].diff_corr_mohms) :
					  (zdet_dnl_table[lb + 1].se_corr_mohms));
	/* Linear interpolation */
	z_interp = (lb_corr * z_to_ub + ub_corr * lb_to_z) / lb_to_ub;

apply_interpolated_bias:
	/* Subtract interpolated bias to correct error */
	if (z_interp < z_val_slope_corrected)
		*z = (u32) (s32) (z_val_slope_corrected - z_interp);
}

static int get_lb_zdet_base_val_index(uint32_t z_val)
{
	int i;

	/* Find the lower bound index, whose base value is the smallest value that is still higher
	 * than the load
	 */
	for (i = 1; i < ARRAY_SIZE(zdet_dnl_table); i++) {
		if (z_val < (OHMS_TO_MILLIOHMS * (u32) zdet_dnl_table[i].base_val_ohms))
			return i - 1;
	}

	/* Return the last index if the load is larger than all base values */
	return ARRAY_SIZE(zdet_dnl_table) - 1;
}

static void apply_zdet_correction(uint32_t *z, int flag_se_diff, u32 se_slope_factor_times_1000,
				  u32 diff_slope_factor_times_1000)
{
	s64 z_val_slope_corrected = 0, slope_corr = 0;
	uint32_t z_meas_bias_removed = 0;
	int lb;

	/* Apply slope correction */
	slope_corr = (s64) ((flag_se_diff) ? diff_slope_factor_times_1000 :
					     se_slope_factor_times_1000);
	z_val_slope_corrected = ((s64) (u64) *z) * (FLOAT_TO_FIXED_XTALK) * slope_corr /
				 SLOPE_FACTOR_SCALER / (FLOAT_TO_FIXED_XTALK);

	/* Interpolate correction term to bias out and apply correction */
	z_meas_bias_removed = (flag_se_diff) ? *z - DIFF_SLOPE_MEAS_BIAS : *z - SE_SLOPE_MEAS_BIAS;
	lb = get_lb_zdet_base_val_index(z_meas_bias_removed);
	interpolate_zdet_val(z, (s64) (u64) z_meas_bias_removed, z_val_slope_corrected, lb,
			     flag_se_diff);
}

static void get_r_common_gnd(struct wcd939x_priv *wcd939x, u32 r_gnd_res_tot_mohms,
			     u32 r_gnd_int_fet_mohms, u32 r_gnd_par_route1_mohms)
{
	u32 r_common_gnd_mohms = 0, r_accum = 0, r_avg = 0;
	size_t i;
	struct wcd939x_pdata *pdata = dev_get_platdata(wcd939x->dev);
	size_t index = pdata->usbcss_hs.gnd.r_cm_gnd_buffer.write_index;

	r_common_gnd_mohms = r_gnd_res_tot_mohms - r_gnd_int_fet_mohms - r_gnd_par_route1_mohms;

	/* Compute average from r_common_gnd buffer */
	for (i = 0; i < R_COMMON_GND_BUFFER_SIZE; i++)
		r_accum += pdata->usbcss_hs.gnd.r_cm_gnd_buffer.data[i];
	r_avg = r_accum / R_COMMON_GND_BUFFER_SIZE;

	/* If r_common_gnd_mohms is OOB, use the average of the buffer values instead */
	if (r_common_gnd_mohms > (r_avg + pdata->usbcss_hs.gnd.r_common_gnd_margin) ||
	    (r_avg >= pdata->usbcss_hs.gnd.r_common_gnd_margin &&
	     r_common_gnd_mohms < (r_avg - pdata->usbcss_hs.gnd.r_common_gnd_margin))) {
		dev_dbg(wcd939x->dev, "%s: %s %d %s %d %s\n", __func__,
			"The average of the r_common_gnd buffer,", r_avg,
			"mohms, is being used instead of the calculated r_common_gnd value of",
			r_common_gnd_mohms, "mohms");
		pdata->usbcss_hs.gnd.r_common_gnd_mohms = r_avg;
		return;
	}

	/* Otherwise, use the computed value and store it in the buffer, updating the write index */
	pdata->usbcss_hs.gnd.r_common_gnd_mohms = r_common_gnd_mohms;
	pdata->usbcss_hs.gnd.r_cm_gnd_buffer.data[index] = r_common_gnd_mohms;
	pdata->usbcss_hs.gnd.r_cm_gnd_buffer.write_index = (index + 1) % R_COMMON_GND_BUFFER_SIZE;
	dev_dbg(wcd939x->dev, "%s: %s %d %s\n", __func__, "The calculated r_common_gnd value,",
		r_common_gnd_mohms, "mohms, is being used");
}

struct usbcss_hs_attr {
	struct wcd939x_priv *priv;
	struct kobj_attribute attr;
	int index;
};

static char *usbcss_sysfs_files[] = {
	"rdson_3p6v",
	"rdson_6v",
	"r1_l",
	"r1_r",
	"r3_l",
	"r3_r",
	"r4_sbu1",
	"r4_sbu2",
	"r5_sbu1",
	"r5_sbu2",
	"r6_sbu1",
	"r6_sbu2",
	"r7_sbu1",
	"r7_sbu2",
	"r_common_gnd_offset",
	"rcom_margin",
	"se_slope_factor_times_1000",
	"diff_slope_factor_times_1000",
	"lin_k_aud",
	"xtalk_config",
};

static ssize_t usbcss_sysfs_store(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	struct usbcss_hs_attr *usbc_attr;
	struct wcd939x_priv *wcd939x;
	struct wcd939x_pdata *pdata;
	struct wcd939x_usbcss_hs_params *usbcss_hs;
	long val;
	int rc;
	u32 aud_tap = 0;
	bool update_xtalk = false, update_linearizer = false;

	usbc_attr = container_of(attr, struct usbcss_hs_attr, attr);
	wcd939x = usbc_attr->priv;
	pdata = dev_get_platdata(wcd939x->dev);

	if (!wcd939x || !pdata)
		return -EINVAL;

	usbcss_hs = &pdata->usbcss_hs;

	rc = kstrtol(buf, 0, &val);
	if (rc)
		return rc;

	if (strcmp(attr->attr.name, "rdson_3p6v") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.rdson_3p6v_mohms = val;
		usbcss_hs->gnd.gnd_ext_fet_delta_mohms = (s32) (usbcss_hs->gnd.rdson_3p6v_mohms -
								usbcss_hs->gnd.rdson_mohms);
		update_linearizer = usbcss_hs->xtalk.xtalk_config == XTALK_ANALOG;
	} else if (strcmp(attr->attr.name, "rdson_6v") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.rdson_mohms = val;
		update_linearizer = usbcss_hs->xtalk.xtalk_config == XTALK_ANALOG;
	} else if (strcmp(attr->attr.name, "r1_l") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->aud.l.r1 = val;
		update_xtalk = true;
	} else if (strcmp(attr->attr.name, "r1_r") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->aud.r.r1 = val;
		update_xtalk = true;
	} else if (strcmp(attr->attr.name, "r3_l") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->aud.l.r3 = val;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "r3_r") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->aud.r.r3 = val;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "r4_sbu1") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu1.r4 = val;
		update_xtalk = true;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "r4_sbu2") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu2.r4 = val;
		update_xtalk = true;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "r5_sbu1") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu1.r5 = val;

		switch (usbcss_hs->xtalk.xtalk_config) {
		case XTALK_ANALOG:
			update_xtalk = true;
			update_linearizer = true;
			break;
		case XTALK_DIGITAL:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r5_sbu2") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu2.r5 = val;

		switch (usbcss_hs->xtalk.xtalk_config) {
		case XTALK_ANALOG:
			update_xtalk = true;
			update_linearizer = true;
			break;
		case XTALK_DIGITAL:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r6_sbu1") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu1.r6 = val;

		switch (usbcss_hs->xtalk.xtalk_config) {
		case XTALK_DIGITAL:
			update_xtalk = true;
			update_linearizer = true;
			break;
		case XTALK_ANALOG:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r6_sbu2") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu2.r6 = val;

		switch (usbcss_hs->xtalk.xtalk_config) {
		case XTALK_DIGITAL:
			update_xtalk = true;
			update_linearizer = true;
			break;
		case XTALK_ANALOG:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r7_sbu1") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu1.r7 = val;

		switch (usbcss_hs->xtalk.xtalk_config) {
		case XTALK_DIGITAL:
			update_xtalk = true;
			update_linearizer = true;
			break;
		case XTALK_ANALOG:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r7_sbu2") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.sbu2.r7 = val;

		switch (usbcss_hs->xtalk.xtalk_config) {
		case XTALK_DIGITAL:
			update_xtalk = true;
			update_linearizer = true;
			break;
		case XTALK_ANALOG:
			fallthrough;
		case XTALK_NONE:
			fallthrough;
		default:
			return count;
		}
	} else if (strcmp(attr->attr.name, "r_common_gnd_offset") == 0) {
		if (val < -MAX_USBCSS_HS_IMPEDANCE_MOHMS || val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of bounds. Min: %d, Max: %d\n",
			__func__, val, -MAX_USBCSS_HS_IMPEDANCE_MOHMS,
			MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.r_common_gnd_offset = val;
		update_xtalk = true;
	} else if (strcmp(attr->attr.name, "rcom_margin") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->gnd.r_common_gnd_margin = val;
	} else if (strcmp(attr->attr.name, "se_slope_factor_times_1000") == 0) {
		if (val > MAX_USBCSS_HS_IMPEDANCE_MOHMS) {
			dev_err(wcd939x->dev, "%s: Value %d out of HS impedance range %d\n",
			__func__, val, MAX_USBCSS_HS_IMPEDANCE_MOHMS);
			return count;
		}
		usbcss_hs->se_slope_factor_times_1000 = val;
	} else if (strcmp(attr->attr.name, "diff_slope_factor_times_1000") == 0) {
		if (val > MAX_DIFF_SLOPE_FACTOR || val < MIN_DIFF_SLOPE_FACTOR) {
			dev_err(wcd939x->dev, "%s: Value %d out of range of %d to %d\n",
			__func__, val, MIN_DIFF_SLOPE_FACTOR, MAX_DIFF_SLOPE_FACTOR);
			return count;
		}
		usbcss_hs->diff_slope_factor_times_1000 = val;
	} else if (strcmp(attr->attr.name, "lin_k_aud") == 0) {
		if (val < MIN_K_TIMES_100 || val > MAX_K_TIMES_100) {
			dev_err(wcd939x->dev, "%s: Value %d out of bounds. Min: %d, Max: %d\n",
			__func__, val, MIN_K_TIMES_100, MAX_K_TIMES_100);
			return count;
		}
		usbcss_hs->aud.k_aud_times_100 = val;
		update_linearizer = true;
	} else if (strcmp(attr->attr.name, "xtalk_config") == 0) {
		pdata->usbcss_hs.xtalk.xtalk_config = val;
		update_xtalk = true;

		switch (val) {
		case XTALK_NONE:
			usbcss_hs->xtalk.scale_l = MAX_XTALK_SCALE;
			usbcss_hs->xtalk.scale_r = MAX_XTALK_SCALE;
			usbcss_hs->xtalk.alpha_l = MIN_XTALK_ALPHA;
			usbcss_hs->xtalk.alpha_r = MIN_XTALK_ALPHA;
			break;
		case XTALK_DIGITAL:
			update_linearizer = true;
			break;
		case XTALK_ANALOG:
			update_linearizer = true;
			break;
		default:
			return count;
		}
	}

	/* Update parastics */
	switch (pdata->usbcss_hs.xtalk.xtalk_config) {
	case XTALK_NONE:
		fallthrough;
	case XTALK_DIGITAL:
		usbcss_hs->gnd.sbu1.r_gnd_par_route1_mohms = usbcss_hs->gnd.sbu1.r7;
		usbcss_hs->gnd.sbu2.r_gnd_par_route1_mohms = usbcss_hs->gnd.sbu2.r7;
		usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms = usbcss_hs->gnd.sbu1.r6 +
							     usbcss_hs->gnd.sbu1.r4;
		usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms = usbcss_hs->gnd.sbu2.r6 +
							     usbcss_hs->gnd.sbu2.r4;
		break;
	case XTALK_ANALOG:
		usbcss_hs->gnd.sbu1.r_gnd_par_route1_mohms = usbcss_hs->gnd.sbu1.r5 +
							     usbcss_hs->gnd.sbu1.r4;
		usbcss_hs->gnd.sbu2.r_gnd_par_route1_mohms = usbcss_hs->gnd.sbu2.r5 +
							     usbcss_hs->gnd.sbu2.r4;
		usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms = 1;
		usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms = 1;
		break;
	default:
		return count;
	}

	/* Update parastics total */
	usbcss_hs->gnd.sbu1.r_gnd_par_tot_mohms = usbcss_hs->gnd.sbu1.r_gnd_par_route1_mohms +
						  usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms;
	usbcss_hs->gnd.sbu2.r_gnd_par_tot_mohms = usbcss_hs->gnd.sbu2.r_gnd_par_route1_mohms +
						  usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms;

	if (update_xtalk) {
		/* Apply r_common_gnd offset */
		usbcss_hs->gnd.r_common_gnd_mohms = (usbcss_hs->gnd.r_common_gnd_offset >= 0) ?
						    usbcss_hs->gnd.r_common_gnd_mohms +
						    (u32) usbcss_hs->gnd.r_common_gnd_offset :
						    usbcss_hs->gnd.r_common_gnd_mohms -
						    (u32) (-1 * usbcss_hs->gnd.r_common_gnd_offset);
		/* Compute and store xtalk values */
		update_xtalk_scale_and_alpha(wcd939x);
		/* Revert r_common_gnd offset */
		usbcss_hs->gnd.r_common_gnd_mohms = (usbcss_hs->gnd.r_common_gnd_offset >= 0) ?
						    usbcss_hs->gnd.r_common_gnd_mohms -
						    (u32) usbcss_hs->gnd.r_common_gnd_offset :
						    usbcss_hs->gnd.r_common_gnd_mohms +
						    (u32) (-1 * usbcss_hs->gnd.r_common_gnd_offset);
		/* Apply xtalk scale and alpha values */
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC0, 0x1F,
				   pdata->usbcss_hs.xtalk.scale_l);
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC1, 0xFF,
				   pdata->usbcss_hs.xtalk.alpha_l);
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC0 +
						    XTALK_CH_REG_ADDR_DELTA, 0x1F,
				   pdata->usbcss_hs.xtalk.scale_r);
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHL_RX_PATH_SEC1 +
						    XTALK_CH_REG_ADDR_DELTA, 0xFF,
				   pdata->usbcss_hs.xtalk.alpha_r);
		dev_err(wcd939x->dev, "%s: Updated xtalk thru sysfs\n",
			__func__);
		dev_dbg(wcd939x->dev, "%s: Left-channel: Xtalk scale is 0x%x and alpha is 0x%x\n",
			__func__, pdata->usbcss_hs.xtalk.scale_l, pdata->usbcss_hs.xtalk.alpha_l);
		dev_dbg(wcd939x->dev, "%s: Right-channel: Xtalk scale is 0x%x and alpha is 0x%x\n",
			__func__, pdata->usbcss_hs.xtalk.scale_r, pdata->usbcss_hs.xtalk.alpha_r);
	}

	if (update_linearizer) {
		get_linearizer_taps(pdata, &aud_tap);
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		wcd_usbss_set_linearizer_sw_tap(aud_tap, LINEARIZER_DEFAULT_TAP);
#endif
		dev_err(wcd939x->dev, "%s: Updated linearizer thru sysfs\n",
			__func__);
		dev_dbg(wcd939x->dev, "%s: Linearizer aud_tap is 0x%x\n",
			__func__, aud_tap);
	}

	return count;
}

static ssize_t usbcss_sysfs_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct usbcss_hs_attr *usbc_attr;
	struct wcd939x_priv *wcd939x;
	struct wcd939x_pdata *pdata;

	usbc_attr = container_of(attr, struct usbcss_hs_attr, attr);
	wcd939x = usbc_attr->priv;
	pdata = dev_get_platdata(wcd939x->dev);

	if (strcmp(attr->attr.name, "rdson_3p6v") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.rdson_3p6v_mohms);
	else if (strcmp(attr->attr.name, "rdson_6v") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.rdson_mohms);
	else if (strcmp(attr->attr.name, "r1_l") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.aud.l.r1);
	else if (strcmp(attr->attr.name, "r1_r") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.aud.r.r1);
	else if (strcmp(attr->attr.name, "r3_l") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.aud.l.r3);
	else if (strcmp(attr->attr.name, "r3_r") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.aud.r.r3);
	else if (strcmp(attr->attr.name, "r4_sbu1") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu1.r4);
	else if (strcmp(attr->attr.name, "r4_sbu2") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu2.r4);
	else if (strcmp(attr->attr.name, "r5_sbu1") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu1.r5);
	else if (strcmp(attr->attr.name, "r5_sbu2") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu2.r5);
	else if (strcmp(attr->attr.name, "r6_sbu1") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu1.r6);
	else if (strcmp(attr->attr.name, "r6_sbu2") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu2.r6);
	else if (strcmp(attr->attr.name, "r7_sbu1") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu1.r7);
	else if (strcmp(attr->attr.name, "r7_sbu2") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.sbu2.r7);
	else if (strcmp(attr->attr.name, "r_common_gnd_offset") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.r_common_gnd_offset);
	else if (strcmp(attr->attr.name, "rcom_margin") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.gnd.r_common_gnd_margin);
	else if (strcmp(attr->attr.name, "se_slope_factor_times_1000") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.se_slope_factor_times_1000);
	else if (strcmp(attr->attr.name, "diff_slope_factor_times_1000") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.diff_slope_factor_times_1000);
	else if (strcmp(attr->attr.name, "lin_k_aud") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.aud.k_aud_times_100);
	else if (strcmp(attr->attr.name, "xtalk_config") == 0)
		return scnprintf(buf, 10, "%d\n", pdata->usbcss_hs.xtalk.xtalk_config);
	return 0;
}

static int create_sysfs_entry_file(struct wcd939x_priv *wcd939x, char *name, int mode,
		int index, struct kobject *parent)
{
	struct usbcss_hs_attr *usbc_attr;
	char *name_copy;

	usbc_attr = devm_kmalloc(wcd939x->dev, sizeof(*usbc_attr), GFP_KERNEL);
	if (!usbc_attr)
		return -ENOMEM;

	name_copy = devm_kstrdup(wcd939x->dev, name, GFP_KERNEL);
	if (!name_copy)
		return -ENOMEM;

	usbc_attr->priv = wcd939x;
	usbc_attr->index = index;
	usbc_attr->attr.attr.name = name_copy;
	usbc_attr->attr.attr.mode = mode;
	usbc_attr->attr.show = usbcss_sysfs_show;
	usbc_attr->attr.store = usbcss_sysfs_store;
	sysfs_attr_init(&usbc_attr->attr.attr);

	return sysfs_create_file(parent, &usbc_attr->attr.attr);
}

static int usbcss_hs_sysfs_init(struct wcd939x_priv *wcd939x)
{
	int rc = 0;
	int i = 0;
	struct kobject *kobj = NULL;

	if (!wcd939x || !wcd939x->dev) {
		pr_err("%s: Invalid wcd939x private data.\n", __func__);
		return -EINVAL;
	}

	kobj = kobject_create_and_add("usbcss_hs", kernel_kobj);
	if (!kobj) {
		dev_err(wcd939x->dev, "%s: Could not create the USBC-SS HS kobj.\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(usbcss_sysfs_files); i++) {
		rc = create_sysfs_entry_file(wcd939x, usbcss_sysfs_files[i],
				0644, i, kobj);
	}

	return 0;
}

static void wcd939x_wcd_mbhc_calc_impedance(struct wcd_mbhc *mbhc, uint32_t *zl, uint32_t *zr)
{
	struct snd_soc_component *component = mbhc->component;
	struct wcd939x_priv *wcd939x = dev_get_drvdata(component->dev);
	struct wcd939x_pdata *pdata = dev_get_platdata(wcd939x->dev);
	s16 reg0, reg1, reg2, reg3, reg4;
	uint32_t zdiff_val = 0, r_gnd_int_fet_mohms = 0, rl_eff_l_mohms = 0, rl_eff_r_mohms = 0;
	uint32_t r_gnd_ext_fet_mohms = 0, r_aud_ext_fet_mohms = 0, r_gnd_res_tot_mohms = 0;
	uint32_t r_gnd_par_tot_mohms = 0, r_gnd_par_route1_mohms = 0;
	uint32_t aud_tap = LINEARIZER_DEFAULT_TAP, zdiff_counter = 0, zdiff_sum = 0;
	uint32_t *zdiff = &zdiff_val;
	s32 z_L_R_delta_mohms = 0;
	int32_t z1L, z1R, z1Ls, z1Diff;
	int zMono, z_diff1, z_diff2;
	size_t i;
	bool is_fsm_disable = false, calculate_lin_aud_tap = false, gnd_ext_fet_updated = false;
	struct wcd939x_mbhc_zdet_param zdet_param = {4, 0, 6, 0x18, 0x60, 0x78};
	struct wcd939x_mbhc_zdet_param *zdet_param_ptr = &zdet_param;
	s16 d1[] = {0, 30, 30, 6};
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	uint32_t cached_regs[4][2] = {{WCD_USBSS_EXT_LIN_EN, 0}, {WCD_USBSS_EXT_SW_CTRL_1, 0},
				      {WCD_USBSS_MG1_BIAS, 0}, {WCD_USBSS_MG2_BIAS, 0}};
	uint32_t l_3_6V_regs[4][2] = {{WCD_USBSS_EXT_LIN_EN, 0x00}, {WCD_USBSS_EXT_SW_CTRL_1, 0x00},
				      {WCD_USBSS_MG1_BIAS, 0x0E}, {WCD_USBSS_MG2_BIAS, 0x0E}};
	uint32_t diff_regs[2][2] = {{WCD_USBSS_EXT_LIN_EN, 0x00}, {WCD_USBSS_EXT_SW_CTRL_1, 0x00}};
#endif
	WCD_MBHC_RSC_ASSERT_LOCKED(mbhc);

	/* Turn on RX supplies */
	if (wcd939x->version == WCD939X_VERSION_2_0) {
		/* Start up Buck/Flyback, Enable RX bias, Use MBHC RCO for MBHC Zdet, Enable Vneg */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x4E, 0x4E);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Enable VNEGDAC_LDO */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x10, 0x10);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Keep PA left/right channels disabled */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x01, 0x01);
		/* Enable VPOS */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x20, 0x20);
		/* Wait 500us for settling */
		usleep_range(500, 510);
	}

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Cache relevant USB-SS registers */
	wcd_usbss_register_update(cached_regs, WCD_USBSS_READ, ARRAY_SIZE(cached_regs));
	/* Disable 2k pulldown on MG for improved measurement */
	wcd_usbss_register_update(l_3_6V_regs, WCD_USBSS_WRITE, ARRAY_SIZE(l_3_6V_regs));
#endif

	/* Store register values */
	reg0 = snd_soc_component_read(component, WCD939X_MBHC_BTN5);
	reg1 = snd_soc_component_read(component, WCD939X_MBHC_BTN6);
	reg2 = snd_soc_component_read(component, WCD939X_MBHC_BTN7);
	reg3 = snd_soc_component_read(component, WCD939X_CTL_CLK);
	reg4 = snd_soc_component_read(component, WCD939X_ZDET_ANA_CTL);

	/* Disable the detection FSM */
	if (snd_soc_component_read(component, WCD939X_MBHC_ELECT) & 0x80) {
		is_fsm_disable = true;
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_ELECT, 0x80, 0x00);
	}

	/* For NO-jack, disable L_DET_EN before Z-det measurements */
	if (mbhc->hphl_swh)
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_MECH, 0x80, 0x00);

	/* Turn off 100k pull down on HPHL */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_MECH, 0x01, 0x00);

	/* Disable surge protection before impedance detection.
	 * This is done to give correct value for high impedance.
	 */
	regmap_update_bits(wcd939x->regmap,
                          WCD939X_HPHLR_SURGE_EN, 0xC0, 0x00);
	/* 1ms delay needed after disable surge protection */
	usleep_range(1000, 1010);

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Disable sense switch and MIC for USB-C analog platforms */
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		wcd_usbss_set_switch_settings_enable(SENSE_SWITCHES, USBSS_SWITCH_DISABLE);
		wcd_usbss_set_switch_settings_enable(MIC_SWITCHES, USBSS_SWITCH_DISABLE);
	}
#endif

	/* L-channel impedance */
	wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, &z1L, NULL, d1);
	if ((z1L == WCD939X_ZDET_FLOATING_IMPEDANCE) || (z1L > WCD939X_ZDET_VAL_100K)) {
		*zl = WCD939X_ZDET_FLOATING_IMPEDANCE;
	} else {
		*zl = z1L;
		wcd939x_wcd_mbhc_qfuse_cal(component, zl, 0);
		dev_dbg(component->dev, "%s: Calibrated left SE measurement is %d(mohms)\n",
			__func__, *zl);
		apply_zdet_correction(zl, ZDET_SE, pdata->usbcss_hs.se_slope_factor_times_1000,
				      pdata->usbcss_hs.diff_slope_factor_times_1000);
	}
	pdata->usbcss_hs.aud.l.zval = *zl;

	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		dev_dbg(component->dev,
			"%s: Calibrated and adjusted left SE measurement is %d(mohms)\n", __func__,
			*zl);
	} else {
		dev_dbg(component->dev, "%s: impedance on HPH_L = %d(mohms)\n",
			__func__, *zl);
	}

	/* R-channel impedance */
	wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, NULL, &z1R, d1);
	if ((z1R == WCD939X_ZDET_FLOATING_IMPEDANCE) || (z1R > WCD939X_ZDET_VAL_100K)) {
		*zr = WCD939X_ZDET_FLOATING_IMPEDANCE;
	} else {
		*zr = z1R;
		wcd939x_wcd_mbhc_qfuse_cal(component, zr, 4);
		dev_dbg(component->dev, "%s: Calibrated right SE measurement is %d(mohms)\n",
			__func__, *zr);
		apply_zdet_correction(zr, ZDET_SE, pdata->usbcss_hs.se_slope_factor_times_1000,
				      pdata->usbcss_hs.diff_slope_factor_times_1000);
	}
	pdata->usbcss_hs.aud.r.zval = *zr;


	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		dev_dbg(component->dev,
			"%s: Calibrated and adjusted right SE measurement is %d(mohms)\n", __func__,
			*zr);
	} else {
		dev_dbg(component->dev, "%s: impedance on HPH_R = %d(mohms)\n",
			__func__, *zr);
		/* Convert from mohms to ohms (rounded) */
		*zl = (*zl + OHMS_TO_MILLIOHMS / 2) / OHMS_TO_MILLIOHMS;
		*zr = (*zr + OHMS_TO_MILLIOHMS / 2) / OHMS_TO_MILLIOHMS;
		goto mono_stereo_detection;
	}

	/* Differential measurement L to R */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Disable AGND switch */
	wcd_usbss_set_switch_settings_enable(AGND_SWITCHES, USBSS_SWITCH_DISABLE);
	wcd_usbss_register_update(diff_regs, WCD_USBSS_WRITE, ARRAY_SIZE(diff_regs));
#endif
	/* Enable HPHR NCLAMP */
	regmap_update_bits(wcd939x->regmap, WCD939X_HPHLR_SURGE_MISC1, 0x08, 0x08);
	/* Wait 3ms for settling */
	usleep_range(3000, 3010);
	/* Differential impedance */
	for (i = 0; i < NUM_DIFF_MEAS; i++) {
		wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, &z1Diff, NULL, d1);
		if ((z1Diff == WCD939X_ZDET_FLOATING_IMPEDANCE) ||
		    (z1Diff > WCD939X_ZDET_VAL_100K)) {
		} else {
			*zdiff = z1Diff;
			wcd939x_wcd_mbhc_qfuse_cal(component, zdiff, 0);
			dev_dbg(component->dev,
				"%s: Calibrated differential measurement %d is %d(mohms)\n",
				__func__, i + 1, *zdiff);
			apply_zdet_correction(zdiff, ZDET_DIFF,
					      pdata->usbcss_hs.se_slope_factor_times_1000,
					      pdata->usbcss_hs.diff_slope_factor_times_1000);
			zdiff_sum += *zdiff;
			zdiff_counter++;
		}
		dev_dbg(component->dev,
			"%s: Calibrated and adjusted differential measurement %d is %d(mohms)\n",
			__func__, i + 1, *zdiff);
	}
	/* Take average of measurements */
	if (zdiff_counter == 0)
		*zdiff = WCD939X_ZDET_FLOATING_IMPEDANCE;
	else
		*zdiff = zdiff_sum / zdiff_counter;
	/* Store the average of the measurements */
	pdata->usbcss_hs.zdiffval = *zdiff;
	dev_dbg(component->dev, "%s: %s %d(mohms)\n", __func__,
		"Average of the calibrated and adjusted differential measurement(s) is", *zdiff);
	/* Disable HPHR NCLAMP */
	regmap_update_bits(wcd939x->regmap, WCD939X_HPHLR_SURGE_MISC1, 0x08, 0x00);
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Enable AGND switch */
	wcd_usbss_set_switch_settings_enable(AGND_SWITCHES, USBSS_SWITCH_ENABLE);
	/* Get ground internal resistance based on orientation */
	if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU2_ORIENTATION_A) {
		r_gnd_int_fet_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_int_fet_mohms;
		r_gnd_par_route1_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_par_route1_mohms;
		r_gnd_par_tot_mohms = pdata->usbcss_hs.gnd.sbu2.r_gnd_par_tot_mohms;
	} else if (wcd_usbss_get_sbu_switch_orientation() == GND_SBU1_ORIENTATION_B) {
		r_gnd_int_fet_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_int_fet_mohms;
		r_gnd_par_route1_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_par_route1_mohms;
		r_gnd_par_tot_mohms = pdata->usbcss_hs.gnd.sbu1.r_gnd_par_tot_mohms;
	} else {
		dev_dbg(component->dev, "%s: Invalid SBU switch orientation\n", __func__);
		*zl = 0;
		*zr = 0;
		goto default_vals;
	}
#endif
	z_L_R_delta_mohms = *zl - *zr;
	dev_dbg(component->dev, "%s: %s : %d mohms\n", __func__,
		"The difference between the L and R SE measurements (L - R) is", z_L_R_delta_mohms);

	/* Ground path resistance */
	/* Use DTSI params for high zdet SE measurements */
	if (pdata->usbcss_hs.aud.l.zval > ZDET_ACC_LMT_MOHMS ||
	    pdata->usbcss_hs.aud.r.zval > ZDET_ACC_LMT_MOHMS) {
		r_gnd_res_tot_mohms = pdata->usbcss_hs.gnd.rdson_mohms + r_gnd_par_tot_mohms +
				      r_gnd_int_fet_mohms;
		pdata->usbcss_hs.gnd.r_common_gnd_mohms = r_gnd_res_tot_mohms -
							  r_gnd_int_fet_mohms -
							  r_gnd_par_route1_mohms;
		dev_dbg(component->dev, "%s: %s %d %s\n", __func__,
			"The r_common_gnd value determined by DTSI parameters,",
			pdata->usbcss_hs.gnd.r_common_gnd_mohms,
			"mohms, is being used instead of calculating r_common_gnd");
		calculate_lin_aud_tap = false;
	} else {
		r_gnd_res_tot_mohms = (*zl + *zr - *zdiff + pdata->usbcss_hs.aud.r_surge_mohms) / 2;
		/* Offset to account for using 3.6V SE measurements */
		r_gnd_res_tot_mohms = (pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms >= 0) ?
				r_gnd_res_tot_mohms -
				(u32) (s32) pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms :
				r_gnd_res_tot_mohms +
				(u32) (s32) (-1 * pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms);
		/* Compute r_common_gnd */
		get_r_common_gnd(wcd939x, r_gnd_res_tot_mohms, r_gnd_int_fet_mohms,
				 r_gnd_par_route1_mohms);
		/* Re-calculate ground path resistance based on r_common_gnd */
		r_gnd_res_tot_mohms = pdata->usbcss_hs.gnd.r_common_gnd_mohms +
				      r_gnd_int_fet_mohms + r_gnd_par_route1_mohms;
		calculate_lin_aud_tap = true;
	}
	dev_dbg(component->dev, "%s: r_gnd_res_tot_mohms is : %d mohms\n", __func__,
		r_gnd_res_tot_mohms);
	/* Print r_common_gnd buffer */
	for (i = 0; i < R_COMMON_GND_BUFFER_SIZE; i++) {
		dev_dbg(component->dev, "%s: Element %d in r_common_gnd_buffer is : %d mohms\n",
			__func__, i + 1, pdata->usbcss_hs.gnd.r_cm_gnd_buffer.data[i]);
	}
	/* Apply r_common_gnd offset */
	pdata->usbcss_hs.gnd.r_common_gnd_mohms =
				(pdata->usbcss_hs.gnd.r_common_gnd_offset >= 0) ?
					pdata->usbcss_hs.gnd.r_common_gnd_mohms +
					(u32) pdata->usbcss_hs.gnd.r_common_gnd_offset :
					pdata->usbcss_hs.gnd.r_common_gnd_mohms -
					(u32) (-1 * pdata->usbcss_hs.gnd.r_common_gnd_offset);

	/* Ground external FET */
	r_gnd_ext_fet_mohms = r_gnd_res_tot_mohms - r_gnd_par_tot_mohms - r_gnd_int_fet_mohms;
	dev_dbg(component->dev, "%s: r_gnd_ext_fet_mohms is : %d mohms\n", __func__,
		r_gnd_ext_fet_mohms);

	/* Audio external FET */
	r_aud_ext_fet_mohms = (pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms >= 0) ?
	      r_gnd_ext_fet_mohms + (u32) (s32) pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms :
	      r_gnd_ext_fet_mohms - (u32) (s32) (-1 * pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms);
	dev_dbg(component->dev, "%s: r_aud_ext_fet_mohms is : %d mohms\n", __func__,
		r_aud_ext_fet_mohms);

	/* Compute effective load resistance */
	rl_eff_l_mohms = *zl - pdata->usbcss_hs.aud.l.r_aud_int_fet_mohms - r_aud_ext_fet_mohms -
			 r_gnd_res_tot_mohms;
	rl_eff_r_mohms = *zr - pdata->usbcss_hs.aud.r.r_aud_int_fet_mohms - r_aud_ext_fet_mohms -
			 r_gnd_res_tot_mohms;

	/* Store z values */
	*zl = (rl_eff_l_mohms - R_CONN_PAR_LOAD_POS_MOHMS - pdata->usbcss_hs.aud.l.r3 +
	       OHMS_TO_MILLIOHMS / 2) / OHMS_TO_MILLIOHMS;
	dev_dbg(component->dev, "%s: rload_l is : %d mohms\n", __func__,
		rl_eff_l_mohms - R_CONN_PAR_LOAD_POS_MOHMS - pdata->usbcss_hs.aud.l.r3);
	*zr = (rl_eff_r_mohms - R_CONN_PAR_LOAD_POS_MOHMS - pdata->usbcss_hs.aud.r.r3 +
	       OHMS_TO_MILLIOHMS / 2) / OHMS_TO_MILLIOHMS;
	dev_dbg(component->dev, "%s: rload_r is : %d mohms\n", __func__,
		rl_eff_r_mohms - R_CONN_PAR_LOAD_POS_MOHMS - pdata->usbcss_hs.aud.r.r3);

	/* Check bounds on effective load values and store the value */
	if (rl_eff_l_mohms > MAX_RL_EFF_MOHMS)
		rl_eff_l_mohms = MAX_RL_EFF_MOHMS;
	else if (rl_eff_l_mohms < MIN_RL_EFF_MOHMS)
		rl_eff_l_mohms = MIN_RL_EFF_MOHMS;
	pdata->usbcss_hs.aud.l.r_load_eff_mohms = rl_eff_l_mohms;
	if (rl_eff_r_mohms > MAX_RL_EFF_MOHMS)
		rl_eff_r_mohms = MAX_RL_EFF_MOHMS;
	else if (rl_eff_r_mohms < MIN_RL_EFF_MOHMS)
		rl_eff_r_mohms = MIN_RL_EFF_MOHMS;
	pdata->usbcss_hs.aud.r.r_load_eff_mohms = rl_eff_r_mohms;

	/* Update FET values and resistances */
	update_ext_fet_res(pdata, r_aud_ext_fet_mohms, r_gnd_ext_fet_mohms);

	/* Update xtalk params */
	/* For SE measurements greater than ZDET_SE_MAX_MOHMS, use default xtalk values */
	if (pdata->usbcss_hs.aud.l.zval > ZDET_SE_MAX_MOHMS ||
	    pdata->usbcss_hs.aud.r.zval > ZDET_SE_MAX_MOHMS) {
		pdata->usbcss_hs.xtalk.scale_l = MAX_XTALK_SCALE;
		pdata->usbcss_hs.xtalk.scale_r = MAX_XTALK_SCALE;
		pdata->usbcss_hs.xtalk.alpha_l = MIN_XTALK_ALPHA;
		pdata->usbcss_hs.xtalk.alpha_r = MIN_XTALK_ALPHA;
		dev_dbg(component->dev, "%s: %s %d, %s\n",
			__func__, "The SE zdet measurement is greater than ZDET_SE_MAX_MOHMS,",
			ZDET_SE_MAX_MOHMS,
			"so the default xtalk scale and alpha values will be used");
	} else {
		update_xtalk_scale_and_alpha(wcd939x);
		/* Compute updated linearizer tap */
		if (calculate_lin_aud_tap) {
			if (r_gnd_ext_fet_mohms < pdata->usbcss_hs.gnd.gnd_ext_fet_min_mohms) {
				r_gnd_ext_fet_mohms = pdata->usbcss_hs.gnd.gnd_ext_fet_min_mohms;
				gnd_ext_fet_updated = true;
			}
			if (r_gnd_ext_fet_mohms > GND_EXT_FET_MAX_MOHMS) {
				r_gnd_ext_fet_mohms = GND_EXT_FET_MAX_MOHMS;
				gnd_ext_fet_updated = true;
			}
			if (gnd_ext_fet_updated) {
				dev_dbg(component->dev, "%s: %s %d mohms\n", __func__,
					"Updated (for linearizer) r_gnd_ext_fet_mohms is :",
					r_gnd_ext_fet_mohms);
				/* Audio external FET */
				    r_aud_ext_fet_mohms =
				    (pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms >= 0) ?
				    r_gnd_ext_fet_mohms +
				    (u32) (s32) pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms :
				    r_gnd_ext_fet_mohms -
				    (u32) (s32) (-1 * pdata->usbcss_hs.gnd.gnd_ext_fet_delta_mohms);
				dev_dbg(component->dev, "%s: %s %d mohms\n", __func__,
					"Updated (for linearizer) r_aud_ext_fet_mohms is :",
					r_aud_ext_fet_mohms);
				/* Update FET values and resistances */
				update_ext_fet_res(pdata, r_aud_ext_fet_mohms, r_gnd_ext_fet_mohms);
			}
			get_linearizer_taps(pdata, &aud_tap);
		}
	}

	/* Print xtalk params */
	dev_dbg(component->dev, "%s: Left-channel: Xtalk scale is 0x%x and alpha is 0x%x\n",
		__func__, pdata->usbcss_hs.xtalk.scale_l, pdata->usbcss_hs.xtalk.alpha_l);
	dev_dbg(component->dev, "%s: Right-channel: Xtalk scale is 0x%x and alpha is 0x%x\n",
		__func__, pdata->usbcss_hs.xtalk.scale_r, pdata->usbcss_hs.xtalk.alpha_r);

	/* Revert r_common_gnd offset */
	pdata->usbcss_hs.gnd.r_common_gnd_mohms = (pdata->usbcss_hs.gnd.r_common_gnd_offset >= 0) ?
					pdata->usbcss_hs.gnd.r_common_gnd_mohms -
					(u32) pdata->usbcss_hs.gnd.r_common_gnd_offset :
					pdata->usbcss_hs.gnd.r_common_gnd_mohms +
					(u32) (-1 * pdata->usbcss_hs.gnd.r_common_gnd_offset);

mono_stereo_detection:
	/* Mono/stereo detection */
	if ((*zl == WCD939X_ZDET_FLOATING_IMPEDANCE) && (*zr == WCD939X_ZDET_FLOATING_IMPEDANCE)) {
		dev_dbg(component->dev,
			"%s: plug type is invalid or extension cable\n",
			__func__);
		goto zdet_complete;
	}
	if ((*zl == WCD939X_ZDET_FLOATING_IMPEDANCE) ||
	    (*zr == WCD939X_ZDET_FLOATING_IMPEDANCE) ||
	    ((*zl < WCD_MONO_HS_MIN_THR) && (*zr > WCD_MONO_HS_MIN_THR)) ||
	    ((*zl > WCD_MONO_HS_MIN_THR) && (*zr < WCD_MONO_HS_MIN_THR))) {
		dev_dbg(component->dev,
			"%s: Mono plug type with one ch floating or shorted to GND\n",
			__func__);
		mbhc->hph_type = WCD_MBHC_HPH_MONO;
		goto zdet_complete;
	}
	snd_soc_component_update_bits(component, WCD939X_R_ATEST, 0x02, 0x02);
	snd_soc_component_update_bits(component, WCD939X_PA_CTL2, 0x40, 0x01);
	wcd939x_mbhc_zdet_ramp(component, zdet_param_ptr, &z1Ls, NULL, d1);
	snd_soc_component_update_bits(component, WCD939X_PA_CTL2, 0x40, 0x00);
	snd_soc_component_update_bits(component, WCD939X_R_ATEST, 0x02, 0x00);
	z1Ls /= 1000;
	wcd939x_wcd_mbhc_qfuse_cal(component, &z1Ls, 0);
	/* Parallel of left Z and 9 ohm pull down resistor */
	zMono = ((*zl) * 9) / ((*zl) + 9);
	z_diff1 = (z1Ls > zMono) ? (z1Ls - zMono) : (zMono - z1Ls);
	z_diff2 = ((*zl) > z1Ls) ? ((*zl) - z1Ls) : (z1Ls - (*zl));
	if ((z_diff1 * (*zl + z1Ls)) > (z_diff2 * (z1Ls + zMono))) {
		dev_dbg(component->dev, "%s: stereo plug type detected\n",
			__func__);
		mbhc->hph_type = WCD_MBHC_HPH_STEREO;
	} else {
		dev_dbg(component->dev, "%s: MONO plug type detected\n",
			__func__);
		mbhc->hph_type = WCD_MBHC_HPH_MONO;
	}
	goto zdet_complete;
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
default_vals:
	pdata->usbcss_hs.xtalk.scale_l = MAX_XTALK_SCALE;
	pdata->usbcss_hs.xtalk.scale_r = MAX_XTALK_SCALE;
	pdata->usbcss_hs.xtalk.alpha_l = MIN_XTALK_ALPHA;
	pdata->usbcss_hs.xtalk.alpha_r = MIN_XTALK_ALPHA;
	/* Print xtalk params */
	dev_dbg(component->dev,
		"%s: Left-channel: Xtalk scale is 0x%x and alpha is 0x%x\n", __func__,
		pdata->usbcss_hs.xtalk.scale_l, pdata->usbcss_hs.xtalk.alpha_l);
	dev_dbg(component->dev,
		"%s: Right-channel: Xtalk scale is 0x%x and alpha is 0x%x\n", __func__,
		pdata->usbcss_hs.xtalk.scale_r, pdata->usbcss_hs.xtalk.alpha_r);
#endif
zdet_complete:
	/* Configure linearizer */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_set_linearizer_sw_tap(aud_tap, LINEARIZER_DEFAULT_TAP);
#endif
	/* Print linearizer values */
	dev_dbg(component->dev, "%s: Linearizer aud_tap is 0x%x\n",
		__func__, aud_tap);
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Enable sense switch and MIC for USB-C analog platforms */
	if (mbhc->mbhc_cfg->enable_usbc_analog) {
		wcd_usbss_set_switch_settings_enable(SENSE_SWITCHES, USBSS_SWITCH_ENABLE);
		wcd_usbss_set_switch_settings_enable(MIC_SWITCHES, USBSS_SWITCH_ENABLE);
	}
#endif
	/* Enable surge protection again after impedance detection for platforms other than USB-C
	 * analog platforms
	 */
	if (!(mbhc->mbhc_cfg->enable_usbc_analog))
		regmap_update_bits(wcd939x->regmap, WCD939X_HPHLR_SURGE_EN, 0xC0, 0xC0);

	snd_soc_component_write(component, WCD939X_MBHC_BTN5, reg0);
	snd_soc_component_write(component, WCD939X_MBHC_BTN6, reg1);
	snd_soc_component_write(component, WCD939X_MBHC_BTN7, reg2);
	/* Turn on 100k pull down on HPHL */
	regmap_update_bits(wcd939x->regmap,
			   WCD939X_MBHC_MECH, 0x01, 0x01);

	/* For NO-jack, re-enable L_DET_EN after Z-det measurements */
	if (mbhc->hphl_swh)
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_MECH, 0x80, 0x80);

	snd_soc_component_write(component, WCD939X_ZDET_ANA_CTL, reg4);
	snd_soc_component_write(component, WCD939X_CTL_CLK, reg3);
	if (is_fsm_disable)
		regmap_update_bits(wcd939x->regmap,
				   WCD939X_MBHC_ELECT, 0x80, 0x80);

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_register_update(cached_regs, WCD_USBSS_WRITE, ARRAY_SIZE(cached_regs));
#endif

	/* Turn off RX supplies */
	if (wcd939x->version == WCD939X_VERSION_2_0) {
		/* Set VPOS to be controlled by RX */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x20, 0x00);
		/* Wait 500us for settling */
		usleep_range(500, 510);
		/* Set PA Left/Right channels and VNEGDAC_LDO to be controlled by RX */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x11, 0x00);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Set Vneg mode and enable to be controlled by RX */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x06, 0x00);
		/* Wait 100us for settling */
		usleep_range(100, 110);
		/* Set RX bias to be controlled by RX and set Buck/Flyback back to SWR Rx clock */
		regmap_update_bits(wcd939x->regmap, WCD939X_ZDET_VNEG_CTL, 0x48, 0x00);
	}
}

static void wcd939x_mbhc_gnd_det_ctrl(struct snd_soc_component *component,
			bool enable)
{
	if (enable) {
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x02, 0x02);
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x40, 0x40);
	} else {
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x40, 0x00);
		snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				    0x02, 0x00);
	}
}

static void wcd939x_mbhc_hph_pull_down_ctrl(struct snd_soc_component *component,
					  bool enable)
{
	if (enable) {
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x40, 0x40);
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x10, 0x10);
	} else {
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x40, 0x00);
		snd_soc_component_update_bits(component, WCD939X_PA_CTL2,
				    0x10, 0x00);
	}
}

static void wcd939x_mbhc_moisture_config(struct wcd_mbhc *mbhc)
{
	struct snd_soc_component *component = mbhc->component;

	if ((mbhc->moist_rref == R_OFF) ||
	    (mbhc->mbhc_cfg->enable_usbc_analog)) {
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		return;
	}

	/* Do not enable moisture detection if jack type is NC */
	if (!mbhc->hphl_swh) {
		dev_dbg(component->dev, "%s: disable moisture detection for NC\n",
			__func__);
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		return;
	}

	snd_soc_component_update_bits(component, WCD939X_CTL_2,
			    0x0C, mbhc->moist_rref << 2);
}

static void wcd939x_mbhc_moisture_detect_en(struct wcd_mbhc *mbhc, bool enable)
{
	struct snd_soc_component *component = mbhc->component;

	if (enable)
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
					0x0C, mbhc->moist_rref << 2);
	else
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
}

static bool wcd939x_mbhc_get_moisture_status(struct wcd_mbhc *mbhc)
{
	struct snd_soc_component *component = mbhc->component;
	bool ret = false;

	if ((mbhc->moist_rref == R_OFF) ||
	    (mbhc->mbhc_cfg->enable_usbc_analog)) {
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		goto done;
	}

	/* Do not enable moisture detection if jack type is NC */
	if (!mbhc->hphl_swh) {
		dev_dbg(component->dev, "%s: disable moisture detection for NC\n",
			__func__);
		snd_soc_component_update_bits(component, WCD939X_CTL_2,
				    0x0C, R_OFF << 2);
		goto done;
	}

	/*
	 * If moisture_en is already enabled, then skip to plug type
	 * detection.
	 */
	if ((snd_soc_component_read(component, WCD939X_CTL_2) & 0x0C))
		goto done;

	wcd939x_mbhc_moisture_detect_en(mbhc, true);
	/* Read moisture comparator status */
	ret = ((snd_soc_component_read(component, WCD939X_FSM_STATUS)
				& 0x20) ? 0 : 1);

done:
	return ret;

}

static void wcd939x_mbhc_moisture_polling_ctrl(struct wcd_mbhc *mbhc,
						bool enable)
{
	struct snd_soc_component *component = mbhc->component;

	snd_soc_component_update_bits(component,
			WCD939X_MOISTURE_DET_POLLING_CTRL,
			0x04, (enable << 2));
}

static void wcd939x_mbhc_bcs_enable(struct wcd_mbhc *mbhc,
						  bool bcs_enable)
{
	if (bcs_enable)
		wcd939x_disable_bcs_before_slow_insert(mbhc->component, false);
	else
		wcd939x_disable_bcs_before_slow_insert(mbhc->component, true);
}

static void wcd939x_surge_reset_routine(struct wcd_mbhc *mbhc)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(mbhc->component);

	regcache_mark_dirty(wcd939x->regmap);
	regcache_sync(wcd939x->regmap);
}

static void wcd939x_mbhc_zdet_leakage_resistance(struct wcd_mbhc *mbhc,
							bool enable)
{
	if (enable)
		snd_soc_component_update_bits(mbhc->component, WCD939X_ZDET_BIAS_CTL,
				0x80, 0x80); /* disable 1M pull-up */
	else
		snd_soc_component_update_bits(mbhc->component, WCD939X_ZDET_BIAS_CTL,
				0x80, 0x00); /* enable 1M pull-up */
}

static const struct wcd_mbhc_cb mbhc_cb = {
	.request_irq = wcd939x_mbhc_request_irq,
	.irq_control = wcd939x_mbhc_irq_control,
	.free_irq = wcd939x_mbhc_free_irq,
	.clk_setup = wcd939x_mbhc_clk_setup,
	.map_btn_code_to_num = wcd939x_mbhc_btn_to_num,
	.mbhc_bias = wcd939x_mbhc_mbhc_bias_control,
	.set_btn_thr = wcd939x_mbhc_program_btn_thr,
	.lock_sleep = wcd939x_mbhc_lock_sleep,
	.register_notifier = wcd939x_mbhc_register_notifier,
	.micbias_enable_status = wcd939x_mbhc_micb_en_status,
	.hph_pa_on_status = wcd939x_mbhc_hph_pa_on_status,
	.hph_pull_up_control_v2 = wcd939x_mbhc_hph_l_pull_up_control,
	.mbhc_micbias_control = wcd939x_mbhc_request_micbias,
	.mbhc_micb_ramp_control = wcd939x_mbhc_micb_ramp_control,
	.get_hwdep_fw_cal = wcd939x_get_hwdep_fw_cal,
	.mbhc_micb_ctrl_thr_mic = wcd939x_mbhc_micb_ctrl_threshold_mic,
	.compute_impedance = wcd939x_wcd_mbhc_calc_impedance,
	.mbhc_gnd_det_ctrl = wcd939x_mbhc_gnd_det_ctrl,
	.hph_pull_down_ctrl = wcd939x_mbhc_hph_pull_down_ctrl,
	.mbhc_moisture_config = wcd939x_mbhc_moisture_config,
	.mbhc_get_moisture_status = wcd939x_mbhc_get_moisture_status,
	.mbhc_moisture_polling_ctrl = wcd939x_mbhc_moisture_polling_ctrl,
	.mbhc_moisture_detect_en = wcd939x_mbhc_moisture_detect_en,
	.bcs_enable = wcd939x_mbhc_bcs_enable,
	.surge_reset_routine = wcd939x_surge_reset_routine,
	.zdet_leakage_resistance = wcd939x_mbhc_zdet_leakage_resistance,
};

static int wcd939x_get_hph_type(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_mbhc *wcd939x_mbhc = wcd939x_soc_get_mbhc(component);
	struct wcd_mbhc *mbhc;

	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}

	mbhc = &wcd939x_mbhc->wcd_mbhc;

	ucontrol->value.integer.value[0] = (u32) mbhc->hph_type;
	dev_dbg(component->dev, "%s: hph_type = %u\n", __func__, mbhc->hph_type);

	return 0;
}

static int wcd939x_hph_impedance_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	uint32_t zl, zr;
	bool hphr;
	struct soc_multi_mixer_control *mc;
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_mbhc *wcd939x_mbhc = wcd939x_soc_get_mbhc(component);

	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}

	mc = (struct soc_multi_mixer_control *)(kcontrol->private_value);
	hphr = mc->shift;
	wcd_mbhc_get_impedance(&wcd939x_mbhc->wcd_mbhc, &zl, &zr);
	dev_dbg(component->dev, "%s: zl=%u(ohms), zr=%u(ohms)\n", __func__, zl, zr);
	ucontrol->value.integer.value[0] = hphr ? zr : zl;

	return 0;
}

static const struct snd_kcontrol_new hph_type_detect_controls[] = {
	SOC_SINGLE_EXT("HPH Type", 0, 0, UINT_MAX, 0,
		       wcd939x_get_hph_type, NULL),
};

static const struct snd_kcontrol_new impedance_detect_controls[] = {
	SOC_SINGLE_EXT("HPHL Impedance", 0, 0, UINT_MAX, 0,
		       wcd939x_hph_impedance_get, NULL),
	SOC_SINGLE_EXT("HPHR Impedance", 0, 1, UINT_MAX, 0,
		       wcd939x_hph_impedance_get, NULL),
};

/*
 * wcd939x_mbhc_get_impedance: get impedance of headphone
 * left and right channels
 * @wcd939x_mbhc: handle to struct wcd939x_mbhc *
 * @zl: handle to left-ch impedance
 * @zr: handle to right-ch impedance
 * return 0 for success or error code in case of failure
 */
int wcd939x_mbhc_get_impedance(struct wcd939x_mbhc *wcd939x_mbhc,
			     uint32_t *zl, uint32_t *zr)
{
	if (!wcd939x_mbhc) {
		pr_err_ratelimited("%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}
	if (!zl || !zr) {
		pr_err_ratelimited("%s: zl or zr null!\n", __func__);
		return -EINVAL;
	}

	return wcd_mbhc_get_impedance(&wcd939x_mbhc->wcd_mbhc, zl, zr);
}
EXPORT_SYMBOL(wcd939x_mbhc_get_impedance);

/*
 * wcd939x_mbhc_hs_detect: starts mbhc insertion/removal functionality
 * @codec: handle to snd_soc_component *
 * @mbhc_cfg: handle to mbhc configuration structure
 * return 0 if mbhc_start is success or error code in case of failure
 */
int wcd939x_mbhc_hs_detect(struct snd_soc_component *component,
			 struct wcd_mbhc_config *mbhc_cfg)
{
	struct wcd939x_priv *wcd939x = NULL;
	struct wcd939x_mbhc *wcd939x_mbhc = NULL;

	if (!component) {
		pr_err_ratelimited("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x) {
		pr_err_ratelimited("%s: wcd939x is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x_mbhc = wcd939x->mbhc;
	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return -EINVAL;
	}

	return wcd_mbhc_start(&wcd939x_mbhc->wcd_mbhc, mbhc_cfg);
}
EXPORT_SYMBOL(wcd939x_mbhc_hs_detect);

/*
 * wcd939x_mbhc_hs_detect_exit: stop mbhc insertion/removal functionality
 * @component: handle to snd_soc_component *
 */
void wcd939x_mbhc_hs_detect_exit(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = NULL;
	struct wcd939x_mbhc *wcd939x_mbhc = NULL;

	if (!component) {
		pr_err_ratelimited("%s: component is NULL\n", __func__);
		return;
	}

	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x) {
		pr_err_ratelimited("%s: wcd939x is NULL\n", __func__);
		return;
	}

	wcd939x_mbhc = wcd939x->mbhc;
	if (!wcd939x_mbhc) {
		dev_err_ratelimited(component->dev, "%s: mbhc not initialized!\n", __func__);
		return;
	}
	wcd_mbhc_stop(&wcd939x_mbhc->wcd_mbhc);
}
EXPORT_SYMBOL(wcd939x_mbhc_hs_detect_exit);

/*
 * wcd939x_mbhc_ssr_down: stop mbhc during
 * wcd939x subsystem restart
 * mbhc: pointer to wcd937x_mbhc structure
 * component: handle to snd_soc_component *
 */
void wcd939x_mbhc_ssr_down(struct wcd939x_mbhc *mbhc,
			struct snd_soc_component *component)
{
	struct wcd_mbhc *wcd_mbhc = NULL;

	if (!mbhc || !component)
		return;

	wcd_mbhc = &mbhc->wcd_mbhc;
	if (!wcd_mbhc) {
		dev_err_ratelimited(component->dev, "%s: wcd_mbhc is NULL\n", __func__);
		return;
	}

	wcd939x_mbhc_hs_detect_exit(component);
	wcd_mbhc_deinit(wcd_mbhc);
}
EXPORT_SYMBOL(wcd939x_mbhc_ssr_down);

/*
 * wcd939x_mbhc_post_ssr_init: initialize mbhc for
 * wcd939x post subsystem restart
 * @mbhc: poniter to wcd939x_mbhc structure
 * @component: handle to snd_soc_component *
 *
 * return 0 if mbhc_init is success or error code in case of failure
 */
int wcd939x_mbhc_post_ssr_init(struct wcd939x_mbhc *mbhc,
			     struct snd_soc_component *component)
{
	int ret = 0;
	struct wcd_mbhc *wcd_mbhc = NULL;

	if (!mbhc || !component)
		return -EINVAL;

	wcd_mbhc = &mbhc->wcd_mbhc;
	if (wcd_mbhc == NULL) {
		pr_err("%s: wcd_mbhc is NULL\n", __func__);
		return -EINVAL;
	}

	/* Reset detection type to insertion after SSR recovery */
	snd_soc_component_update_bits(component, WCD939X_MBHC_MECH,
				0x20, 0x20);
	ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb, &intr_ids,
			    wcd_mbhc_registers, WCD939X_ZDET_SUPPORTED);
	if (ret) {
		dev_err(component->dev, "%s: mbhc initialization failed\n",
			__func__);
		goto done;
	}

done:
	return ret;
}
EXPORT_SYMBOL(wcd939x_mbhc_post_ssr_init);

/*
 * wcd939x_mbhc_init: initialize mbhc for wcd939x
 * @mbhc: poniter to wcd939x_mbhc struct pointer to store the configs
 * @codec: handle to snd_soc_component *
 * @fw_data: handle to firmware data
 *
 * return 0 if mbhc_init is success or error code in case of failure
 */
int wcd939x_mbhc_init(struct wcd939x_mbhc **mbhc,
				struct snd_soc_component *component,
				struct fw_info *fw_data)
{
	struct wcd939x_mbhc *wcd939x_mbhc = NULL;
	struct wcd_mbhc *wcd_mbhc = NULL;
	int ret = 0;
	struct wcd939x_pdata *pdata;
	struct wcd939x_priv *wcd939x;

	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x_mbhc = devm_kzalloc(component->dev, sizeof(struct wcd939x_mbhc),
				    GFP_KERNEL);
	if (!wcd939x_mbhc)
		return -ENOMEM;

	wcd939x_mbhc->fw_data = fw_data;
	BLOCKING_INIT_NOTIFIER_HEAD(&wcd939x_mbhc->notifier);
	wcd_mbhc = &wcd939x_mbhc->wcd_mbhc;
	if (wcd_mbhc == NULL) {
		pr_err("%s: wcd_mbhc is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	/* Setting default mbhc detection logic to ADC */
	wcd_mbhc->mbhc_detection_logic = WCD_DETECTION_ADC;

	/* Down ramp timer set-up */
	timer_setup(&wcd939x_mbhc->rdown_timer, rdown_timer_callback, 0);
	wcd939x_mbhc->rdown_prev_iter = 0;
	wcd939x_mbhc->rdown_timer_complete = false;

	pdata = dev_get_platdata(component->dev);
	if (!pdata) {
		dev_err(component->dev, "%s: pdata pointer is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err;
	}
	wcd_mbhc->micb_mv = pdata->micbias.micb2_mv;

	ret = wcd_mbhc_init(wcd_mbhc, component, &mbhc_cb,
				&intr_ids, wcd_mbhc_registers,
				WCD939X_ZDET_SUPPORTED);
	if (ret) {
		dev_err(component->dev, "%s: mbhc initialization failed\n",
			__func__);
		goto err;
	}

	(*mbhc) = wcd939x_mbhc;
	snd_soc_add_component_controls(component, impedance_detect_controls,
				   ARRAY_SIZE(impedance_detect_controls));
	snd_soc_add_component_controls(component, hph_type_detect_controls,
				   ARRAY_SIZE(hph_type_detect_controls));

	wcd939x = dev_get_drvdata(component->dev);
	if (!wcd939x) {
		dev_err(component->dev, "%s: wcd939x pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	usbcss_hs_sysfs_init(wcd939x);

	return 0;
err:
	if (wcd939x_mbhc)
		del_timer(&wcd939x_mbhc->rdown_timer);
	devm_kfree(component->dev, wcd939x_mbhc);
	return ret;
}
EXPORT_SYMBOL(wcd939x_mbhc_init);

/*
 * wcd939x_mbhc_deinit: deinitialize mbhc for wcd939x
 * @codec: handle to snd_soc_component *
 */
void wcd939x_mbhc_deinit(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x;
	struct wcd939x_mbhc *wcd939x_mbhc;

	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return;
	}

	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x) {
		pr_err("%s: wcd939x is NULL\n", __func__);
		return;
	}

	wcd939x_mbhc = wcd939x->mbhc;
	if (wcd939x_mbhc) {
		del_timer(&wcd939x_mbhc->rdown_timer);
		wcd_mbhc_deinit(&wcd939x_mbhc->wcd_mbhc);
		devm_kfree(component->dev, wcd939x_mbhc);
	}
}
EXPORT_SYMBOL(wcd939x_mbhc_deinit);
