
/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include "mdss_mdp.h"
#include "mdss_dsi.h"

#define DT_CMD_HDR 6

#define DEFAULT_FPS_LOG_INTERVAL 100
#define DEFAULT_FPS_ARRAY_SIZE 120

#define DSI_PCLK_MIN 3300000
#define DSI_PCLK_MAX 223000000
#define DSI_PCLK_DEFAULT 35000000

struct device virtdev;

struct fps_array {
	u32 frame_nbr;
	u32 time_delta;
};
static u16 fps_array_cnt;

static struct fps_data {
	struct mutex fps_lock;
	u32 log_interval;
	u32 interval_ms;
	struct timespec timestamp_last;
	u32 frame_counter_last;
	u32 frame_counter;
	u32 fpks;
	struct timespec fpks_ts_last;
	u16 fa_last_array_pos;
	struct fps_array fa[DEFAULT_FPS_ARRAY_SIZE];
} fpsd;

DEFINE_LED_TRIGGER(bl_led_trigger);

static struct mdss_dsi_phy_ctrl phy_params;

static int lcd_id;
static int mdss_dsi_panel_detect(struct mdss_panel_data *pdata);
static int mdss_panel_parse_panel_dt(struct device_node *np,
	struct device_node *next,
	struct mdss_panel_common_pdata *panel_data,
	int driver_ic, char *id_data);

/* pcc data infomation */
#define PANEL_SKIP_ID		0xff
#define CLR_NUM_REG_DATA_REN	2	/* read registor bytes */
#define CLR_RESOLUTION		60	/* u/v resolution */
#define CLR_SUB_V_BLOCK_HEIGHT	4
#define CLR_SUB_COL_MAX		6
#define PCC_COLUMNS		3
#ifdef CONFIG_FB_MSM_MDSS_SUB_AREA_TABLE_51
#define CLR_NUM_PART		6	/* matrix partition */
#define CLR_NUM_REG_DATA_NOV	1	/* read registor bytes */
#define CLR_REG_DATA_NOV_V	1
#define CLR_SUB_AREA_V_START	4
#define PCC_WIDE_ROWS		25
#define PCC_SUB_ROWS		51
#else
#define CLR_NUM_PART		3	/* matrix partition */
#define CLR_NUM_REG_DATA_NOV	3	/* read registor bytes */
#define CLR_REG_DATA_NOV_V	2
#define CLR_SUB_AREA_V_START	20
#define PCC_WIDE_ROWS		7
#define PCC_SUB_ROWS		42
#endif
#define CLR_REG_DATA_NOV_U	0

#define PCC_WIDE_SIZE	(sizeof(struct mdss_pcc_cfg_rgb) * PCC_WIDE_ROWS)
#define PCC_SUB_SIZE	(sizeof(struct mdss_pcc_cfg_rgb) * PCC_SUB_ROWS)

#define calc_coltype_num(val, numpart, maxval)\
	((val >= maxval) ? numpart - 1 : val / (maxval / numpart))

#ifdef CONFIG_FB_MSM_MDSS_SUB_AREA_TABLE_51
/* pcc color matrix type */
enum color_type {
	CLR01_GRN,
	CLR02_GRN,
	CLR03_LG,	/* light green */
	CLR04_LG,
	CLR05_YEL,
	CLR06_YEL,
	CLR07_GRN,
	CLR08_GRN,
	CLR09_LG,
	CLR10_LG,
	CLR11_YEL,
	CLR12_YEL,
	CLR13_LB,	/* light blue */
	CLR14_LB,
	CLR15_WHT,
	CLR16_ORG,
	CLR17_ORG,
	CLR18_BLE,
	CLR19_BLE,
	CLR20_PUR,
	CLR21_PUR,
	CLR22_RED,
	CLR23_RED,
	CLR24_RED,
	CLR25_RED,
	UNUSED = 0xff,
};

/* pcc color matrix  */
static const enum color_type clr_wide_tbl[CLR_NUM_PART][CLR_NUM_PART] = {
	{CLR18_BLE, CLR19_BLE, CLR21_PUR, CLR21_PUR, CLR24_RED, CLR25_RED},
	{CLR19_BLE, CLR19_BLE, CLR20_PUR, CLR20_PUR, CLR22_RED, CLR23_RED},
	{CLR13_LB,  CLR14_LB,  UNUSED, UNUSED, CLR16_ORG, CLR17_ORG},
	{CLR13_LB,  CLR14_LB,  UNUSED, UNUSED, CLR16_ORG, CLR17_ORG},
	{CLR07_GRN, CLR08_GRN, CLR09_LG,  CLR10_LG,  CLR11_YEL, CLR12_YEL},
	{CLR01_GRN, CLR02_GRN, CLR03_LG,  CLR04_LG,  CLR05_YEL, CLR06_YEL},
};
#else
/* pcc color matrix type */
enum color_type {
	CLR01_GRN,
	CLR02_YEL,
	CLR03_LB,	/* light blue */
	CLR04_ORG,
	CLR05_BLE,
	CLR06_PUR,
	CLR07_RED,
	UNUSED = 0xff,
};

/* pcc color matrix  */
static const enum color_type clr_wide_tbl[CLR_NUM_PART][CLR_NUM_PART] = {
	{CLR05_BLE, CLR06_PUR, CLR07_RED},
	{CLR03_LB,  UNUSED,    CLR04_ORG},
	{CLR01_GRN, UNUSED,    CLR02_YEL},
};
#endif

static const struct {
	uint8_t u_min;
	uint8_t u_max;
	uint8_t sub_area;
} clr_sub_tbl[][CLR_SUB_COL_MAX] = {
#ifdef CONFIG_FB_MSM_MDSS_SUB_AREA_TABLE_51
	{{22, 25, 50} },
	{{22, 25, 48}, {26, 29, 49} },
	{{22, 25, 45}, {26, 29, 46}, {30, 33, 47} },
	{{22, 25, 41}, {26, 29, 42}, {30, 33, 43}, {34, 37, 44} },
	{{22, 25, 37}, {26, 29, 38}, {30, 33, 39}, {34, 37, 40}, {38, 41, 51} },
	{{22, 23, 31}, {24, 27, 32}, {28, 31, 33}, {32, 35, 34}, {36, 39, 35},
	 {40, 41, 36} },
	{{22, 23, 26}, {24, 27, 27}, {32, 35, 28}, {36, 39, 29}, {40, 41, 30} },
	{{22, 23, 20}, {24, 27, 21}, {28, 31, 22}, {32, 35, 23}, {36, 39, 24},
	 {40, 41, 25} },
	{{24, 27, 15}, {28, 31, 16}, {32, 35, 17}, {36, 39, 18}, {40, 41, 19} },
	{{26, 29, 11}, {30, 33, 12}, {34, 37, 13}, {38, 41, 14} },
	{{28, 31,  7}, {32, 35,  8}, {36, 39,  9}, {40, 43, 10} },
	{{31, 34,  4}, {35, 38,  5}, {39, 42,  6} },
	{{34, 37,  2}, {38, 41,  3} },
	{{38, 41,  1} },			/* Area1 */
#else
	{{20, 23, 33}, {24, 27, 34}, {28, 35, 35}, {36, 39, 36}, {40, 43, 42} },
	{{20, 23, 29}, {24, 27, 30}, {28, 31, 31}, {32, 35, 32}, {36, 39, 36},
	 {40, 43, 42} },
	{{20, 23, 24}, {24, 27, 25}, {32, 35, 26}, {36, 39, 27}, {40, 43, 28} },
	{{20, 23, 18}, {24, 27, 19}, {28, 31, 20}, {32, 35, 21}, {36, 39, 22},
	 {40, 43, 23} },
	{{20, 23, 37}, {24, 27, 13}, {28, 31, 14}, {32, 35, 15}, {36, 39, 16},
	 {40, 43, 17} },
	{{20, 23, 38}, {24, 27,  8}, {28, 31,  9}, {32, 35, 10}, {36, 39, 11},
	 {40, 43, 12} },
	{{20, 23, 38}, {24, 27,  3}, {28, 31,  4}, {32, 35,  5}, {36, 39,  6},
	 {40, 43,  7} },
	{{20, 23, 38}, {24, 35, 39}, {36, 39,  1}, {40, 43,  2} },
	{{20, 39, 40}, {40, 43, 41} },
	{{20, 39, 40}, {40, 43, 41} },
#endif
};

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;

	if (!gpio_is_valid(ctrl->pwm_pmic_gpio)) {
		pr_err("%s: pwm_pmic_gpio=%d Invalid\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ret = gpio_request(ctrl->pwm_pmic_gpio, "disp_pwm");
	if (ret) {
		pr_err("%s: pwm_pmic_gpio=%d request failed\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: lpg_chan=%d pwm request failed", __func__,
				ctrl->pwm_lpg_chan);
		gpio_free(ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
	}
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	ret = pwm_config(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_dcs_read(struct mdss_dsi_ctrl_pdata *ctrl,
			char cmd0, char cmd1)
{
	struct dcs_cmd_req cmdreq;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: level=%d\n", __func__, level);

	led_pwm1[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

static void mdss_dsi_panel_fps_array_clear(struct fps_data *fps)
{
	memset(fps->fa, 0, sizeof(fps->fa));
	fps_array_cnt = 0;
}

static ssize_t mdss_dsi_panel_frame_counter(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%i\n", fpsd.frame_counter);
}

static ssize_t mdss_dsi_panel_frames_per_ksecs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%i\n", fpsd.fpks);
}

static ssize_t mdss_dsi_panel_interval_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%i\n", fpsd.interval_ms);
}

static ssize_t mdss_dsi_panel_log_interval_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%i\n", fpsd.log_interval);
}

static ssize_t mdss_dsi_panel_log_interval_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = count;

	if (sscanf(buf, "%4i", &fpsd.log_interval) != 1) {
		pr_err("%s: Error, buf = %s\n", __func__, buf);
		ret = -EINVAL;
	}
	return ret;
}

#define DEBUG_INTERVAL_ARRAY
static ssize_t mdss_dsi_panel_interval_array_ms(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u16 i, len, rc = 0;
	char *tmp = buf;

	mutex_lock(&fpsd.fps_lock);
	len = fpsd.fa_last_array_pos;
	/* Get the first frames from the buffer */
	for (i = len + 1; i < DEFAULT_FPS_ARRAY_SIZE; i++) {
		if (fpsd.fa[i].time_delta) {
#ifdef DEBUG_INTERVAL_ARRAY
			/* FrameNumber, buf idx and delta time */
			rc += scnprintf(tmp + rc, PAGE_SIZE - rc ,
							"%03i[%03i]: %i,\n",
							fpsd.fa[i].frame_nbr, i,
							fpsd.fa[i].time_delta);
#else
			rc += scnprintf(tmp + rc, PAGE_SIZE - rc ,
						"%i, ", fpsd.fa[i].time_delta);
#endif
		}
	}
	/* Get the rest frames from the buffer */
	if (len) {
		for (i = 0; i <= len; i++) {
			if (fpsd.fa[i].time_delta) {
#ifdef DEBUG_INTERVAL_ARRAY
				/* FrameNumber, buf idx and delta time */
				rc += scnprintf(tmp + rc, PAGE_SIZE - rc ,
							"%03i[%03i]: %i,\n",
							fpsd.fa[i].frame_nbr, i,
							fpsd.fa[i].time_delta);
#else
				rc += scnprintf(tmp + rc, PAGE_SIZE - rc ,
						"%i, ", fpsd.fa[i].time_delta);
#endif
			}
		}
	}
	rc += scnprintf(tmp + rc, PAGE_SIZE - rc , "\n");

	/* Clear the buffer once it is read */
	mdss_dsi_panel_fps_array_clear(&fpsd);
	mutex_unlock(&fpsd.fps_lock);

	return rc;
}

static int mdss_dsi_panel_read_cabc(struct device *dev)
{
	struct platform_device *pdev;
	struct mdss_dsi_ctrl_pdata *ctrl;

	pdev = container_of(dev, struct platform_device, dev);
	if (!pdev) {
		dev_err(dev, "%s(%d): no panel connected\n",
							__func__, __LINE__);
		goto exit;
	}

	ctrl = platform_get_drvdata(pdev);
	if (!ctrl) {
		dev_err(dev, "%s(%d): no panel connected\n",
							__func__, __LINE__);
		goto exit;
	}

	return ctrl->spec_pdata->cabc_enabled;
exit:
	return -EINVAL;
}

static void mdss_dsi_panel_write_cabc(struct device *dev, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl;

	ctrl = dev_get_drvdata(dev);
	if (!ctrl) {
		dev_err(dev, "%s(%d): no panel connected\n",
							__func__, __LINE__);
		goto exit;
	}

	ctrl->spec_pdata->cabc_enabled = enable;

exit:
	return;
}

static ssize_t mdss_dsi_panel_cabc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int tmp;
	tmp = mdss_dsi_panel_read_cabc(dev);
	return scnprintf(buf, PAGE_SIZE, "%i\n", tmp);
}

static ssize_t mdss_dsi_panel_cabc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = count;
	long tmp;

	if (kstrtol(buf, 10, &tmp)) {
		dev_err(dev, "%s: Error, buf = %s\n", __func__, buf);
		ret = -EINVAL;
		goto exit;
	}
	mdss_dsi_panel_write_cabc(dev, tmp);
exit:
	return ret;
}

static struct device_attribute panel_attributes[] = {
	__ATTR(frame_counter, S_IRUGO, mdss_dsi_panel_frame_counter, NULL),
	__ATTR(frames_per_ksecs, S_IRUGO,
				mdss_dsi_panel_frames_per_ksecs, NULL),
	__ATTR(interval_ms, S_IRUGO, mdss_dsi_panel_interval_ms_show, NULL),
	__ATTR(log_interval, S_IRUGO|S_IWUSR|S_IWGRP,
					mdss_dsi_panel_log_interval_show,
					mdss_dsi_panel_log_interval_store),
	__ATTR(interval_array, S_IRUGO,
					mdss_dsi_panel_interval_array_ms, NULL),
	__ATTR(cabc, S_IRUGO|S_IWUSR|S_IWGRP, mdss_dsi_panel_cabc_show,
						mdss_dsi_panel_cabc_store),
};

static int register_attributes(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(panel_attributes); i++)
		if (device_create_file(dev, panel_attributes + i))
			goto error;
	return 0;
error:
	dev_err(dev, "%s: Unable to create interface\n", __func__);
	for (--i; i >= 0 ; i--)
		device_remove_file(dev, panel_attributes + i);
	return -ENODEV;
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!ctrl->spec_pdata->detected && !ctrl->spec_pdata->init_from_begin)
		return 0;

	pr_info("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi = &pdata->panel_info.mipi;

	if (ctrl->einit_cmds.cmd_cnt) {
		struct mdss_panel_common_pdata *vendor_pdata
			= dev_get_platdata(&pdata->panel_pdev->dev);

		pr_debug("%s: early init sequence\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->einit_cmds);
		if (vendor_pdata->reset)
			vendor_pdata->reset(pdata, 1);
	}

	if (ctrl->cabc_early_on_cmds.cmd_cnt &&
					ctrl->spec_pdata->cabc_enabled) {
		pr_debug("%s: early CABC-on sequence\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->cabc_early_on_cmds);
		ctrl->cabc_active = 1;
	}

	if (ctrl->init_cmds.cmd_cnt) {
		pr_debug("%s: init (exit sleep) sequence\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->init_cmds);
	}

	if (ctrl->on_cmds.cmd_cnt && !ctrl->spec_pdata->disp_on_in_hs) {
		pr_debug("%s: panel on sequence (in low speed)\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
	}

	pr_debug("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!ctrl->spec_pdata->detected) {
		ctrl->spec_pdata->detected = true;
		if (!ctrl->spec_pdata->init_from_begin)
			return 0;
	}

	pr_info("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (ctrl->cabc_deferred_on_cmds.cmd_cnt && ctrl->cabc_active) {
		pr_debug("%s: killing CABC deferred\n", __func__);
		cancel_work_sync(&ctrl->cabc_work);
	}

	mipi = &pdata->panel_info.mipi;

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

	if (ctrl->cabc_active && (ctrl->spec_pdata->cabc_enabled == 0)) {
		pr_debug("%s: sending display off\n", __func__);
		if (ctrl->cabc_off_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl, &ctrl->cabc_off_cmds);
		if (ctrl->cabc_late_off_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl,
						&ctrl->cabc_late_off_cmds);
		ctrl->cabc_active = 0;
	}
	pr_debug("%s: Done\n", __func__);

	return 0;
}

static void cabc_work_fn(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl =
			container_of(work, struct mdss_dsi_ctrl_pdata,
			cabc_work);
	pr_debug("%s: CABC deferred sequence\n", __func__);
	mdss_dsi_panel_cmds_send(ctrl, &ctrl->cabc_deferred_on_cmds);
	ctrl->cabc_active = 1;
	pr_debug("%s: CABC deferred sequence sent\n", __func__);
}

static int mdss_dsi_panel_disp_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!ctrl->spec_pdata->detected)
		return 0;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi = &pdata->panel_info.mipi;

	if ((ctrl->cabc_on_cmds.cmd_cnt && ctrl->spec_pdata->cabc_enabled) ||
		(ctrl->on_cmds.cmd_cnt && ctrl->spec_pdata->disp_on_in_hs)) {
		pr_debug("%s: delay after entering video mode\n", __func__);
		msleep(ctrl->spec_pdata->wait_time_before_on_cmd);
	}

	if (ctrl->cabc_on_cmds.cmd_cnt && ctrl->spec_pdata->cabc_enabled) {
		pr_debug("%s: CABC on sequence\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->cabc_on_cmds);
		ctrl->cabc_active = 1;
	}

	if (ctrl->on_cmds.cmd_cnt && ctrl->spec_pdata->disp_on_in_hs) {
		pr_debug("%s: panel on sequence (in high speed)\n", __func__);
		mdss_set_tx_power_mode(0, pdata);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
	}
	if (ctrl->cabc_deferred_on_cmds.cmd_cnt &&
				ctrl->spec_pdata->cabc_enabled) {
		pr_debug("%s: posting CABC deferred\n", __func__);
		schedule_work(&ctrl->cabc_work);
	}
	pr_debug("%s: done\n", __func__);

	return 0;
}

static int mdss_dsi_panel_detect(struct mdss_panel_data *pdata)
{
	struct device_node *np;
	struct device_node *next = NULL;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	struct mdss_panel_common_pdata *panel_data;
	int rc;

	panel_data = dev_get_platdata(&pdata->panel_pdev->dev);
	if (!panel_data) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (!panel_data->panel_detect)
		return 0;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	spec_pdata = ctrl_pdata->spec_pdata;
	if (!spec_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	mdss_dsi_op_mode_config(DSI_CMD_MODE, pdata);
	mdss_dsi_cmds_rx(ctrl_pdata,
			 ctrl_pdata->id_read_cmds.cmds, 10, 0);

	pr_debug("%s: Panel ID", __func__);
	pr_debug("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
		ctrl_pdata->rx_buf.data[0], ctrl_pdata->rx_buf.data[1],
		ctrl_pdata->rx_buf.data[2], ctrl_pdata->rx_buf.data[3],
		ctrl_pdata->rx_buf.data[4], ctrl_pdata->rx_buf.data[5],
		ctrl_pdata->rx_buf.data[6], ctrl_pdata->rx_buf.data[7]);

	np = pdata->panel_pdev->dev.of_node;
	if (!np) {
		pr_err("%s np=NULL\n", __func__);
		return -EINVAL;
	}

	rc = mdss_panel_parse_panel_dt(np, next, panel_data,
		spec_pdata->driver_ic, ctrl_pdata->rx_buf.data);

	return 0;
}

static int mdss_dsi_panel_update_panel_info(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_common_pdata *panel_data;
	struct mipi_panel_info *mipi;
	int rc;
	u32 h_period, v_period, dsi_pclk_rate;
	u8 lanes = 0, bpp;

	panel_data = dev_get_platdata(&pdata->panel_pdev->dev);
	if (!panel_data) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (!panel_data->panel_detect)
		return 0;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	h_period = ((panel_data->panel_info.lcdc.h_pulse_width)
			+ (panel_data->panel_info.lcdc.h_back_porch)
			+ (panel_data->panel_info.xres)
			+ (panel_data->panel_info.lcdc.h_front_porch));

	v_period = ((panel_data->panel_info.lcdc.v_pulse_width)
			+ (panel_data->panel_info.lcdc.v_back_porch)
			+ (panel_data->panel_info.yres)
			+ (panel_data->panel_info.lcdc.v_front_porch));

	mipi = &panel_data->panel_info.mipi;

	panel_data->panel_info.type =
		((mipi->mode == DSI_VIDEO_MODE)
			? MIPI_VIDEO_PANEL : MIPI_CMD_PANEL);

	if (mipi->data_lane3)
		lanes += 1;
	if (mipi->data_lane2)
		lanes += 1;
	if (mipi->data_lane1)
		lanes += 1;
	if (mipi->data_lane0)
		lanes += 1;

	if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB666_LOOSE))
		bpp = 3;
	else if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
		 || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB565))
		bpp = 2;
	else
		bpp = 3;		/* Default format set to RGB888 */

	if (panel_data->panel_info.type == MIPI_VIDEO_PANEL &&
		!panel_data->panel_info.clk_rate) {
		h_period += panel_data->panel_info.lcdc.xres_pad;
		v_period += panel_data->panel_info.lcdc.yres_pad;

		if (lanes > 0) {
			panel_data->panel_info.clk_rate =
			((h_period * v_period * (mipi->frame_rate) * bpp * 8)
			   / lanes);
		} else {
			pr_err("%s: forcing mdss_dsi lanes to 1\n", __func__);
			panel_data->panel_info.clk_rate =
				(h_period * v_period
					 * (mipi->frame_rate) * bpp * 8);
		}
	}
	pll_divider_config.clk_rate = panel_data->panel_info.clk_rate;

	rc = mdss_dsi_clk_div_config(bpp, lanes, &dsi_pclk_rate);
	if (rc) {
		pr_err("%s: unable to initialize the clk dividers\n", __func__);
		return rc;
	}

	if ((dsi_pclk_rate < DSI_PCLK_MIN) || (dsi_pclk_rate > DSI_PCLK_MAX)) {
		pr_err("%s: Pixel clock not supported\n", __func__);
		dsi_pclk_rate = DSI_PCLK_DEFAULT;
	}

	mipi->dsi_pclk_rate = dsi_pclk_rate;
	ctrl_pdata->pclk_rate = dsi_pclk_rate;
	ctrl_pdata->byte_clk_rate = panel_data->panel_info.clk_rate / 8;

	ctrl_pdata->einit_cmds = panel_data->einit_cmds;
	ctrl_pdata->init_cmds = panel_data->init_cmds;
	ctrl_pdata->on_cmds = panel_data->on_cmds;
	ctrl_pdata->off_cmds = panel_data->off_cmds;
	ctrl_pdata->id_read_cmds = panel_data->id_read_cmds;

	ctrl_pdata->cabc_early_on_cmds = panel_data->cabc_early_on_cmds;
	ctrl_pdata->cabc_on_cmds = panel_data->cabc_on_cmds;
	ctrl_pdata->cabc_off_cmds = panel_data->cabc_off_cmds;
	ctrl_pdata->cabc_late_off_cmds = panel_data->cabc_late_off_cmds;
	ctrl_pdata->cabc_deferred_on_cmds = panel_data->cabc_deferred_on_cmds;
	if (ctrl_pdata->cabc_deferred_on_cmds.cmd_cnt)
		INIT_WORK(&ctrl_pdata->cabc_work, cabc_work_fn);
	ctrl_pdata->cabc_active = 0;

	panel_data->panel_info.panel_power_on =
			ctrl_pdata->panel_data.panel_info.panel_power_on;

	memcpy(&((ctrl_pdata->panel_data).panel_info),
		&(panel_data->panel_info),
			sizeof(struct mdss_panel_info));
	return 0;
}

#define RENESAS_VREG_WAIT_MS	25
#define RENESAS_DISP_EN_CHANGE_WAIT_MS	25

static int mdss_dsi_panel_reset_renesas(
		struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	pr_debug("%s: enable=%d\n", __func__, enable);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!ctrl->disp_en_gpio)
		pr_debug("%s:%d, disp_en line not configured\n",
			   __func__, __LINE__);

	if (!ctrl->rst_gpio)
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);

	if (enable) {
		msleep(RENESAS_VREG_WAIT_MS);
		gpio_set_value(ctrl->disp_en_gpio, 1);
		gpio_set_value(ctrl->rst_gpio, 0);
		msleep(RENESAS_DISP_EN_CHANGE_WAIT_MS);
		gpio_set_value(ctrl->rst_gpio, 1);
		usleep_range(10000, 11000);
	} else {
		gpio_set_value(ctrl->rst_gpio, 0);
		usleep_range(10000, 11000);
		gpio_set_value(ctrl->disp_en_gpio, 0);
		msleep(RENESAS_DISP_EN_CHANGE_WAIT_MS);
	}

	return 0;
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

		fps->fa[fps_array_cnt].frame_nbr = fps->frame_counter;
		fps->fa[fps_array_cnt].time_delta = msec;
		fps->fa_last_array_pos = fps_array_cnt;
		fps_array_cnt++;
		if (fps_array_cnt >= DEFAULT_FPS_ARRAY_SIZE)
			fps_array_cnt = 0;

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

static void mdss_dsi_panel_fps_data_init(struct fps_data *fps)
{
	fps->frame_counter = 0;
	fps->frame_counter_last = 0;
	fps->log_interval = DEFAULT_FPS_LOG_INTERVAL;
	fps->fpks = 0;
	fps->fa_last_array_pos = 0;
	getrawmonotonic(&fps->timestamp_last);
	mutex_init(&fps->fps_lock);
}

int mdss_dsi_panel_fps_data_update(struct msm_fb_data_type *mfd)
{
	/* Only count fps on primary display */
	if (mfd->index == 0)
		update_fps_data(&fpsd);

	return 0;
}

static int mdss_dsi_panel_power_on_renesas(
		struct mdss_panel_data *pdata, int enable)
{
	int ret;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	static int display_power_on;
	static int skip_first_off = 1;
	struct mdss_panel_common_pdata *panel_data;

	pr_debug("%s: enable=%d\n", __func__, enable);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	panel_data = dev_get_platdata(&pdata->panel_pdev->dev);
	if (panel_data == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (enable && !display_power_on) {
		ret = regulator_set_optimum_mode
		  ((ctrl->shared_pdata).vdd_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdd_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdd_io_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdd_io_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdda_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdda_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_enable((ctrl->shared_pdata).vdd_io_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_enable((ctrl->shared_pdata).vdd_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_enable((ctrl->shared_pdata).vdda_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}

		ret = mdss_dsi_panel_reset_renesas(pdata, 1);
		if (ret) {
			pr_err("%s: Failed to enable gpio.\n", __func__);
			return ret;
		}
		display_power_on = 1;
	} else if (!enable && skip_first_off && panel_data->panel_detect) {
		skip_first_off = 0;
	} else if (!enable) {

		ret = mdss_dsi_panel_reset_renesas(pdata, 0);
		if (ret) {
			pr_err("%s: Failed to disable gpio.\n", __func__);
			return ret;
		}

		ret = regulator_disable((ctrl->shared_pdata).vdd_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_disable((ctrl->shared_pdata).vdda_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_disable((ctrl->shared_pdata).vdd_io_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode
		  ((ctrl->shared_pdata).vdd_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdd_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdd_io_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdd_io_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdda_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdda_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}
		display_power_on = 0;
	}
	return 0;
}

#define NOVATEK_DISP_RST_WAIT_MS	25

static int mdss_dsi_panel_reset_novatek(
		struct mdss_panel_data *pdata, int enable)

{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	pr_debug("%s: enable=%d\n", __func__, enable);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!ctrl->disp_en_gpio)
		pr_debug("%s:%d, disp_en line not configured\n",
			   __func__, __LINE__);

	if (!ctrl->rst_gpio)
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);

	if (enable) {
		usleep_range(10000, 11000);
		gpio_set_value(ctrl->disp_en_gpio, 1);
		usleep_range(10000, 11000);
		gpio_set_value(ctrl->rst_gpio, 1);
		usleep_range(10000, 11000);
		gpio_set_value(ctrl->rst_gpio, 0);
		usleep_range(10000, 11000);
		gpio_set_value(ctrl->rst_gpio, 1);
		msleep(NOVATEK_DISP_RST_WAIT_MS);
	} else {
		usleep_range(10000, 11000);
		gpio_set_value(ctrl->rst_gpio, 0);
	}

	return 0;
}

static int mdss_dsi_panel_power_on_novatek(
		struct mdss_panel_data *pdata, int enable)
{
	int ret;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	static int display_power_on;
	static int skip_first_off = 1;
	struct mdss_panel_common_pdata *panel_data;

	pr_debug("%s: enable=%d\n", __func__, enable);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	panel_data = dev_get_platdata(&pdata->panel_pdev->dev);
	if (panel_data == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (enable && !display_power_on) {
		ret = regulator_set_optimum_mode
		  ((ctrl->shared_pdata).vdd_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdd_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdd_io_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdd_io_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdda_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdda_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_enable((ctrl->shared_pdata).vdd_io_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_enable((ctrl->shared_pdata).vdd_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_enable((ctrl->shared_pdata).vdda_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}

		ret = mdss_dsi_panel_reset_novatek(pdata, 1);
		if (ret) {
			pr_err("%s: Failed to enable gpio.\n", __func__);
			return ret;
		}
		display_power_on = 1;
	} else if (!enable && skip_first_off && panel_data->panel_detect) {
		skip_first_off = 0;
	} else if (!enable) {

		gpio_set_value(ctrl->disp_en_gpio, 0);
		usleep_range(10000, 11000);

		ret = regulator_disable((ctrl->shared_pdata).vdd_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_disable((ctrl->shared_pdata).vdda_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_disable((ctrl->shared_pdata).vdd_io_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode
		  ((ctrl->shared_pdata).vdd_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdd_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdd_io_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdd_io_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(
			(ctrl->shared_pdata).vdda_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdda_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = mdss_dsi_panel_reset_novatek(pdata, 0);
		if (ret) {
			pr_err("%s: Failed to disable gpio.\n", __func__);
			return ret;
		}
		display_power_on = 0;
	}
	return 0;
}

static void conv_uv_data(char *data, int drv_ic, int *u_data, int *v_data)
{
	if (drv_ic == PANEL_DRIVER_IC_NOVATEK) {
		/* 6bit is effective as u_data */
		*u_data = data[CLR_REG_DATA_NOV_U] & 0x3F;
		/* 6bit is effective as v_data */
		*v_data = data[CLR_REG_DATA_NOV_V] & 0x3F;
	} else {
		*u_data = ((data[0] & 0x0F) << 2) |
			/* 4bit of data[0] higher data. */
			((data[1] >> 6) & 0x03);
			/* 2bit of data[1] lower data. */
		*v_data = (data[1] & 0x3F);
			/* Remainder 6bit of data[1] is effective as v_data. */
	}
}

static void get_uv_data(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		struct mdss_panel_common_pdata *vendor_pdata,
		int *u_data, int *v_data)
{
	struct dsi_cmd_desc *cmds = vendor_pdata->uv_read_cmds.cmds;
	char buf[MDSS_DSI_LEN];
	char *pos = buf;
	int len;
	int i;

	len = (vendor_pdata->driver_ic == PANEL_DRIVER_IC_NOVATEK) ?
		CLR_NUM_REG_DATA_NOV : CLR_NUM_REG_DATA_REN;
	for (i = 0; i < vendor_pdata->uv_read_cmds.cmd_cnt; i++) {
		mdss_dsi_cmds_rx(ctrl_pdata, cmds, len, 0);
		memcpy(pos, ctrl_pdata->rx_buf.data, len);
		pos += len;
		cmds++;
	}
	conv_uv_data(buf, vendor_pdata->driver_ic, u_data, v_data);
}

static int find_subdivision_area(int u_data, int v_data)
{
	int row, col;
	int num_area = 0;

	if (v_data < CLR_SUB_AREA_V_START || CLR_RESOLUTION <= v_data)
		goto exit;

	row = (v_data - CLR_SUB_AREA_V_START) / CLR_SUB_V_BLOCK_HEIGHT;
	if (ARRAY_SIZE(clr_sub_tbl) <= row)
		goto exit;
	for (col = 0; col < CLR_SUB_COL_MAX; col++) {
		if (clr_sub_tbl[row][col].sub_area == 0)
			break;
		if (clr_sub_tbl[row][col].u_min <= u_data
			&& u_data <= clr_sub_tbl[row][col].u_max) {
			num_area = clr_sub_tbl[row][col].sub_area;
			break;
		}
	}
exit:
	return num_area;
}

int mdss_dsi_panel_pcc_setup(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_panel_common_pdata *vendor_pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	const struct mdss_pcc_cfg_rgb *cfg_rgb = NULL;
	enum color_type color_type = 0;
	int u_data = 0, v_data = 0;
	int u, v;
	int num_area = 0;
	int ret;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	vendor_pdata = dev_get_platdata(&pdata->panel_pdev->dev);
	if (vendor_pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	if (!vendor_pdata->wide_tbl)
		goto exit;
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_op_mode_config(DSI_CMD_MODE, pdata);
	if (vendor_pdata->uv_read_cmds.cmds)
		get_uv_data(ctrl_pdata, vendor_pdata, &u_data, &v_data);
	if (u_data == 0 && v_data == 0)
		goto exit;

	if (vendor_pdata->sub_tbl)
		num_area = find_subdivision_area(u_data, v_data);
	if (num_area > 0) {
		cfg_rgb = &vendor_pdata->sub_tbl[num_area - 1];
	} else {
		u = calc_coltype_num(u_data, CLR_NUM_PART, CLR_RESOLUTION);
		v = calc_coltype_num(v_data, CLR_NUM_PART, CLR_RESOLUTION);
		color_type = clr_wide_tbl[v][u];
		if (color_type != UNUSED)
			cfg_rgb = &vendor_pdata->wide_tbl[color_type];
	}
	if (cfg_rgb) {
		u32 copyback;
		struct mdp_pcc_cfg_data pcc_config;
		struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);

		if (!mdp5_data->ctl) {
			pr_err("%s: ctl not initialized\n", __func__);
			return -ENODEV;
		}
		memset(&pcc_config, 0, sizeof(struct mdp_pcc_cfg_data));
		pcc_config.block = mfd->index + MDP_LOGICAL_BLOCK_DISP_0;
		pcc_config.ops = MDP_PP_OPS_ENABLE | MDP_PP_OPS_WRITE;
		pcc_config.r.r = cfg_rgb->r;
		pcc_config.g.g = cfg_rgb->g;
		pcc_config.b.b = cfg_rgb->b;
		ret = mdss_mdp_pcc_config(mdp5_data->ctl, &pcc_config,
			&copyback);
		if (ret != 0)
			pr_err("failed by settings of pcc data.\n");
	}
	pr_info("%s (%d):area=%d ct=%d ud=%d vd=%d",
		__func__, __LINE__, num_area, color_type, u_data, v_data);

exit:
	return 0;
}

static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	pcmds->cmd_cnt = 0;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return 0;
	}

	if (pcmds->buf != NULL)
		kfree(pcmds->buf);

	if (pcmds->cmds != NULL)
		kfree(pcmds->cmds);

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			return -ENOMEM;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		kfree(buf);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		return -ENOMEM;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	pcmds->link_state = DSI_LP_MODE; /* default */

	if (link_key != NULL) {
		data = of_get_property(np, link_key, NULL);
		if (!strncmp(data, "DSI_HS_MODE", 11))
			pcmds->link_state = DSI_HS_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;
}

#define PANEL_FULL_HD_DEFAULT_XRES	1080
#define PANEL_FULL_HD_DEFAULT_YRES	1920

static int mdss_panel_parse_panel_dt(struct device_node *np,
			struct device_node *next,
			struct mdss_panel_common_pdata *panel_data,
			int driver_ic, char *id_data)
{
	u32 res[6], tmp;
	int rc, i, len;
	const char *data;
	const char *panel_name;
	static const char *pdest;
	struct mdss_panel_info *panel_info = &panel_data->panel_info;

	for_each_child_of_node(np, next) {
		rc = of_property_read_u32(next, "somc,driver-ic", &tmp);
		if (rc) {
			pr_err("%s:%d, Unable to read driver ic value",
				__func__, __LINE__);
			goto error;
		}
		if (driver_ic != tmp)
			continue;

		data = of_get_property(next, "somc,panel-id", &len);
		if (!data) {
			pr_err("%s:%d, panel not read\n",
				__func__, __LINE__);
			goto error;
		}
		if (data && id_data) {
			if ((len == 1) && (data[0] == PANEL_SKIP_ID))
				continue;
			for (i = 0; i < len; i++) {
				pr_debug("read data:0x%02X dtsi data:0x%02X",
						id_data[i], data[i]);
				if ((id_data[i] != data[i]) &&
					(data[i] != PANEL_SKIP_ID)) {
						rc = -ENODEV;
						break;
					}
				rc = 0;
			}
			if (rc)
				continue;
		} else if (data && !id_data) {
			if ((len != 1) || (data[0] != PANEL_SKIP_ID))
				continue;
			else {
				rc = of_property_read_u32(next
					, "somc,panel-detect", &tmp);
				panel_data->panel_detect = !rc ? tmp : 0;
			}
		}

		panel_name = of_get_property(next, "label", NULL);
		if (!panel_name)
			pr_info("%s:%d, panel name not specified\n",
							__func__, __LINE__);
		else
			pr_info("%s: Panel Name = %s\n", __func__, panel_name);

		rc = of_property_read_u32_array(next,
			"qcom,mdss-pan-res", res, 2);
		if (rc) {
			pr_err("%s:%d, panel resolution not specified\n",
							__func__, __LINE__);
			goto error;
		}
		panel_info->xres = (!rc ? res[0] : PANEL_FULL_HD_DEFAULT_XRES);
		panel_info->yres = (!rc ? res[1] : PANEL_FULL_HD_DEFAULT_YRES);

		rc = of_property_read_u32_array(next,
			"somc,mdss-phy-size-mm", res, 2);
		if (rc)
			pr_err("%s:%d, panel physical size not specified\n",
							__func__, __LINE__);

		panel_info->width = (!rc ? res[0] : 0);
		panel_info->height = (!rc ? res[1] : 0);

		rc = of_property_read_u32_array(next,
			"qcom,mdss-pan-active-res", res, 2);
		if (rc == 0) {
			panel_info->lcdc.xres_pad =
				panel_info->xres - res[0];
			panel_info->lcdc.yres_pad =
				panel_info->yres - res[1];
		}

		rc = of_property_read_u32(next, "qcom,mdss-pan-bpp", &tmp);
		if (rc) {
			pr_err("%s:%d, panel bpp not specified\n",
							__func__, __LINE__);
			goto error;
		}
		panel_info->bpp = (!rc ? tmp : 24);

		pdest = of_get_property(next,
					"qcom,mdss-pan-dest", NULL);
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strncmp(pdest, "display_1", 9))
			panel_info->pdest = DISPLAY_1;
		else if (!strncmp(pdest, "display_2", 9))
			panel_info->pdest = DISPLAY_2;
		else {
			pr_err("%s: pdest not specified. Set Default\n",
								__func__);
			panel_info->pdest = DISPLAY_1;
		}

		rc = of_property_read_u32_array(next,
			"qcom,mdss-pan-porch-values", res, 6);
		panel_info->lcdc.h_back_porch = (!rc ? res[0] : 6);
		panel_info->lcdc.h_pulse_width = (!rc ? res[1] : 2);
		panel_info->lcdc.h_front_porch = (!rc ? res[2] : 6);
		panel_info->lcdc.v_back_porch = (!rc ? res[3] : 6);
		panel_info->lcdc.v_pulse_width = (!rc ? res[4] : 2);
		panel_info->lcdc.v_front_porch = (!rc ? res[5] : 6);

		rc = of_property_read_u32(next, "qcom,mdss-pan-dsi-mode", &tmp);
		panel_info->mipi.mode = (!rc ? tmp : DSI_VIDEO_MODE);

		rc = of_property_read_u32(next, "qcom,mdss-vsync-enable", &tmp);
		panel_info->mipi.vsync_enable = (!rc ? tmp : 0);

		rc = of_property_read_u32(next,
			"qcom,mdss-hw-vsync-mode", &tmp);
		panel_info->mipi.hw_vsync_mode = (!rc ? tmp : 0);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-h-pulse-mode", &tmp);
		panel_info->mipi.pulse_mode_hsa_he =
						(!rc ? tmp : false);

		rc = of_property_read_u32_array(next,
			"qcom,mdss-pan-dsi-h-power-stop", res, 3);
		panel_info->mipi.hbp_power_stop =
						(!rc ? res[0] : false);
		panel_info->mipi.hsa_power_stop =
						(!rc ? res[1] : false);
		panel_info->mipi.hfp_power_stop =
						(!rc ? res[2] : false);

		rc = of_property_read_u32_array(next,
			"qcom,mdss-pan-dsi-bllp-power-stop", res, 2);
		panel_info->mipi.bllp_power_stop =
						(!rc ? res[0] : false);
		panel_info->mipi.eof_bllp_power_stop =
						(!rc ? res[1] : false);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-traffic-mode", &tmp);
		panel_info->mipi.traffic_mode =
				(!rc ? tmp : DSI_NON_BURST_SYNCH_PULSE);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-insert-dcs-cmd", &tmp);
		panel_info->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-wr-mem-continue", &tmp);
		panel_info->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-wr-mem-start", &tmp);
		panel_info->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-te-sel", &tmp);
		panel_info->mipi.te_sel =
			(!rc ? tmp : 1);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-dst-format", &tmp);
		panel_info->mipi.dst_format =
				(!rc ? tmp : DSI_VIDEO_DST_FORMAT_RGB888);

		rc = of_property_read_u32(next, "qcom,mdss-pan-dsi-vc", &tmp);
		panel_info->mipi.vc = (!rc ? tmp : 0);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-rgb-swap", &tmp);
		panel_info->mipi.rgb_swap =
						(!rc ? tmp : DSI_RGB_SWAP_RGB);

		rc = of_property_read_u32_array(next,
			"qcom,mdss-pan-dsi-data-lanes", res, 4);
		panel_info->mipi.data_lane0 = (!rc ? res[0] : true);
		panel_info->mipi.data_lane1 = (!rc ? res[1] : false);
		panel_info->mipi.data_lane2 = (!rc ? res[2] : false);
		panel_info->mipi.data_lane3 = (!rc ? res[3] : false);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-dlane-swap", &tmp);
		panel_info->mipi.dlane_swap = (!rc ? tmp : 0);

		rc = of_property_read_u32_array(next,
			"qcom,mdss-pan-dsi-t-clk", res, 2);
		panel_info->mipi.t_clk_pre = (!rc ? res[0] : 0x24);
		panel_info->mipi.t_clk_post = (!rc ? res[1] : 0x03);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-stream", &tmp);
		panel_info->mipi.stream = (!rc ? tmp : 0);

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-mdp-tr", &tmp);
		panel_info->mipi.mdp_trigger =
				(!rc ? tmp : DSI_CMD_TRIGGER_SW);
		if (panel_info->mipi.mdp_trigger > 6) {
			pr_err("%s:%d, Invalid mdp trigger. Use sw trigger",
							 __func__, __LINE__);
			panel_info->mipi.mdp_trigger =
						DSI_CMD_TRIGGER_SW;
		}

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-dma-tr", &tmp);
		panel_info->mipi.dma_trigger =
				(!rc ? tmp : DSI_CMD_TRIGGER_SW);
		if (panel_info->mipi.dma_trigger > 6) {
			pr_err("%s:%d, Invalid dma trigger. Use sw trigger",
							 __func__, __LINE__);
			panel_info->mipi.dma_trigger =
						DSI_CMD_TRIGGER_SW;
		}

		rc = of_property_read_u32(next,
			"qcom,mdss-pan-dsi-frame-rate", &tmp);
		panel_info->mipi.frame_rate = (!rc ? tmp : 60);

		rc = of_property_read_u32(next, "qcom,mdss-pan-clk-rate", &tmp);
		panel_info->clk_rate = (!rc ? tmp : 0);

		data = of_get_property(next,
			"qcom,panel-phy-regulatorSettings", &len);
		if ((!data) || (len != 7)) {
			pr_err("%s:%d, Unable to read Phy regulator settings",
			       __func__, __LINE__);
			goto error;
		}
		for (i = 0; i < len; i++)
			phy_params.regulator[i] = data[i];

		data = of_get_property(next,
			"qcom,panel-phy-timingSettings", &len);
		if ((!data) || (len != 12)) {
			pr_err("%s:%d, Unable to read Phy timing settings",
			       __func__, __LINE__);
			goto error;
		}
		for (i = 0; i < len; i++)
			phy_params.timing[i] = data[i];

		data = of_get_property(next,
			"qcom,panel-phy-strengthCtrl", &len);
		if ((!data) || (len != 2)) {
			pr_err("%s:%d, Unable to read Phy Strength ctrl",
			       __func__, __LINE__);
			goto error;
		}
		phy_params.strength[0] = data[0];
		phy_params.strength[1] = data[1];

		data = of_get_property(next, "qcom,panel-phy-bistCtrl", &len);
		if ((!data) || (len != 6)) {
			pr_err("%s:%d, Unable to read Phy Bist Ctrl",
			       __func__, __LINE__);
			goto error;
		}
		for (i = 0; i < len; i++)
			phy_params.bistCtrl[i] = data[i];

		data = of_get_property(next, "qcom,panel-phy-laneConfig", &len);
		if ((!data) || (len != 45)) {
			pr_err("%s:%d, Unable to read Phy lane configure",
			       __func__, __LINE__);
			goto error;
		}
		for (i = 0; i < len; i++)
			phy_params.laneCfg[i] = data[i];

		panel_info->mipi.dsi_phy_db = &phy_params;

		mdss_dsi_parse_dcs_cmds(next, &panel_data->einit_cmds,
			"somc,panel-early-init-cmds", NULL);

		mdss_dsi_parse_dcs_cmds(next, &panel_data->init_cmds,
			"somc,panel-init-cmds", NULL);

		mdss_dsi_parse_dcs_cmds(next, &panel_data->on_cmds,
			"qcom,panel-on-cmds", "qcom,on-cmds-dsi-state");

		mdss_dsi_parse_dcs_cmds(next, &panel_data->off_cmds,
			"qcom,panel-off-cmds", "qcom,off-cmds-dsi-state");

		rc = of_property_read_u32(next, "somc,wait-time-before-on-cmd",
			&tmp);
		panel_data->wait_time_before_on_cmd = !rc ? tmp : 0;

		rc = of_property_read_u32(next, "somc,disp-on-in-hs", &tmp);
		panel_data->disp_on_in_hs = (!rc ? tmp : 0);

		rc = of_property_read_u32(next, "somc,init-from-begin", &tmp);
		panel_data->init_from_begin = (!rc ? tmp : 0);

		mdss_dsi_parse_dcs_cmds(next, &panel_data->uv_read_cmds,
			"somc,panel-uv-cmds", NULL);

		if (of_property_read_bool(next, "somc,panel-pcc-table")) {
			panel_data->wide_tbl = kzalloc(PCC_WIDE_SIZE,
							GFP_KERNEL);
			if (!panel_data->wide_tbl) {
				pr_err("no mem assigned: kzalloc fail\n");
				return -ENOMEM;
			}
			rc = of_property_read_u32_array(next,
				"somc,panel-pcc-table",
				(u32 *)panel_data->wide_tbl,
				PCC_COLUMNS * PCC_WIDE_ROWS);
			if (rc) {
				pr_err("%s:%d, Unable to read pcc table",
					__func__, __LINE__);
				goto error;
			}
		}
		if (of_property_read_bool(next, "somc,panel-pcc-sub-table")) {
			panel_data->sub_tbl = kzalloc(PCC_SUB_SIZE, GFP_KERNEL);
			if (!panel_data->sub_tbl) {
				pr_err("no mem assigned: kzalloc fail\n");
				return -ENOMEM;
			}
			rc = of_property_read_u32_array(next,
				"somc,panel-pcc-sub-table",
				(u32 *)panel_data->sub_tbl,
				PCC_COLUMNS * PCC_SUB_ROWS);
			if (rc) {
				pr_err("%s:%d, Unable to read pcc table",
					__func__, __LINE__);
				goto error;
			}
		}

		mdss_dsi_parse_dcs_cmds(next,
				&panel_data->cabc_early_on_cmds,
				"somc,panel-cabc-early-on-cmds", NULL);

		mdss_dsi_parse_dcs_cmds(next,
				&panel_data->cabc_on_cmds,
					"somc,panel-cabc-on-cmds", NULL);

		mdss_dsi_parse_dcs_cmds(next,
				&panel_data->cabc_off_cmds,
					"somc,panel-cabc-off-cmds", NULL);

		mdss_dsi_parse_dcs_cmds(next,
				&panel_data->cabc_late_off_cmds,
				"somc,panel-cabc-late-off-cmds", NULL);

		mdss_dsi_parse_dcs_cmds(next,
				&panel_data->cabc_deferred_on_cmds,
				"somc,panel-cabc-deferred-on-cmds", NULL);

		rc = of_property_read_u32(next, "somc,cabc-enabled", &tmp);
		panel_data->cabc_enabled = !rc ? tmp : 0;

		break;
	}

	return 0;
error:
	return -EINVAL;
}

static int mdss_panel_parse_dt_with_child(struct platform_device *pdev,
			struct mdss_panel_common_pdata *panel_data,
			int driver_ic)
{
	struct device_node *np;
	struct device_node *next = NULL;
	u32 res[2], tmp;
	int rc;
	static const char *bl_ctrl_type;

	np = pdev->dev.of_node;

	rc = of_property_read_u32(np, "qcom,mdss-pan-underflow-clr", &tmp);
	panel_data->panel_info.lcdc.underflow_clr = (!rc ? tmp : 0xff);

	bl_ctrl_type = of_get_property(np,
				  "qcom,mdss-pan-bl-ctrl", NULL);
	if ((bl_ctrl_type) && (!strncmp(bl_ctrl_type, "bl_ctrl_wled", 12))) {
		led_trigger_register_simple("bkl-trigger", &bl_led_trigger);
		pr_debug("%s: SUCCESS-> WLED TRIGGER register\n", __func__);

		panel_data->panel_info.bklt_ctrl = BL_WLED;
	} else if (!strncmp(bl_ctrl_type, "bl_ctrl_pwm", 11)) {
		panel_data->panel_info.bklt_ctrl = BL_PWM;

		rc = of_property_read_u32(np, "qcom,pwm-period", &tmp);
		if (rc) {
			pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
			return -EINVAL;
		}
		panel_data->panel_info.pwm_period = tmp;

		rc = of_property_read_u32(np, "qcom,pwm-lpg-channel", &tmp);
		if (rc) {
			pr_err("%s:%d, Error, dsi lpg channel\n",
						__func__, __LINE__);
			return -EINVAL;
		}
		panel_data->panel_info.pwm_lpg_chan = tmp;

		tmp = of_get_named_gpio(np, "qcom,pwm-pmic-gpio", 0);
		panel_data->panel_info.pwm_pmic_gpio =  tmp;
	} else if (!strncmp(bl_ctrl_type, "bl_ctrl_dcs", 11)) {
		panel_data->panel_info.bklt_ctrl = BL_DCS_CMD;
	} else {
		pr_debug("%s: Unknown backlight control\n", __func__);
		panel_data->panel_info.bklt_ctrl = UNKNOWN_CTRL;
	}

	rc = of_property_read_u32_array(np, "qcom,mdss-pan-bl-levels", res, 2);
	panel_data->panel_info.bl_min = (!rc ? res[0] : 0);
	panel_data->panel_info.bl_max = (!rc ? res[1] : 255);

	mdss_dsi_parse_dcs_cmds(np, &panel_data->id_read_cmds,
		"somc,panel-id-read-cmds", NULL);

	rc = mdss_panel_parse_panel_dt(np, next, panel_data, driver_ic, NULL);
	if (rc) {
		pr_err("%s:%d, cannot read panel dt\n",
						__func__, __LINE__);
		goto error;
	}
	return 0;
error:

	return -EINVAL;
}

static int __devinit mdss_dsi_panel_probe(struct platform_device *pdev)
{
	int rc = 0;
	static struct mdss_panel_common_pdata vendor_pdata;
	char *path_name = "mdss_dsi_panel";
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct device_node *dsi_ctrl_np = NULL;
	struct platform_device *ctrl_pdev = NULL;

	dev_set_name(&virtdev, "%s", path_name);
	rc = device_register(&virtdev);
	if (rc) {
		pr_err("%s: device_register rc = %d\n", __func__, rc);
		return rc;
	}

	rc = register_attributes(&virtdev);
	if (rc) {
		pr_err("%s: register_attributes rc = %d\n", __func__, rc);
		goto error;
	}

	pr_debug("%s:%d, debug info id=%d", __func__, __LINE__, pdev->id);
	if (!pdev->dev.of_node) {
		rc = -ENODEV;
		goto error;
	}

	dsi_ctrl_np = of_parse_phandle(pdev->dev.of_node,
				       "qcom,dsi-ctrl-phandle", 0);
	if (!dsi_ctrl_np) {
		dev_err(&pdev->dev, "%s: Dsi controller node not initialized\n",
								__func__);
		rc = -EPROBE_DEFER;
		goto error;
	}

	ctrl_pdev = of_find_device_by_node(dsi_ctrl_np);

	ctrl_pdata = platform_get_drvdata(ctrl_pdev);
	if (!ctrl_pdata) {
		dev_err(&pdev->dev, "%s: no dsi ctrl driver data\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	rc = dev_set_drvdata(&virtdev, ctrl_pdata);

	lcd_id = of_get_named_gpio(pdev->dev.of_node, "somc,dric-gpio", 0);
	if (!gpio_is_valid(lcd_id)) {
		pr_err("%s:%d, Disp_en gpio not specified\n",
						__func__, __LINE__);
		vendor_pdata.driver_ic = PANEL_DRIVER_IC_NONE;
	}

	if (!vendor_pdata.driver_ic) {
		rc = gpio_request(lcd_id, "lcd_id");
		if (rc) {
			pr_err("request lcd id gpio failed, rc=%d\n",
				rc);
			vendor_pdata.driver_ic = PANEL_DRIVER_IC_NONE;
		}
	}

	if (!vendor_pdata.driver_ic) {
		rc = gpio_direction_input(lcd_id);
		if (rc) {
			pr_err("set_direction for lcd_id gpio failed, rc=%d\n",
				rc);
			gpio_free(lcd_id);
			vendor_pdata.driver_ic = PANEL_DRIVER_IC_NONE;
		}
	}

	if (!vendor_pdata.driver_ic) {
		mdss_dsi_panel_power_detect(pdev, 1);
		vendor_pdata.driver_ic = gpio_get_value(lcd_id);
		pr_err("gpio=%d\n", vendor_pdata.driver_ic);
		mdss_dsi_panel_power_detect(pdev, 0);
	}

	rc = mdss_panel_parse_dt_with_child(pdev, &vendor_pdata,
		vendor_pdata.driver_ic);
	if (rc)
		goto error;

	if (vendor_pdata.driver_ic == PANEL_DRIVER_IC_RENESAS) {
		vendor_pdata.panel_power_on = mdss_dsi_panel_power_on_renesas;
		vendor_pdata.reset = mdss_dsi_panel_reset_renesas;
	} else if (vendor_pdata.driver_ic == PANEL_DRIVER_IC_NOVATEK) {
		vendor_pdata.panel_power_on = mdss_dsi_panel_power_on_novatek;
		vendor_pdata.reset = mdss_dsi_panel_reset_novatek;
	} else {
		vendor_pdata.panel_power_on = mdss_dsi_panel_power_on_renesas;
		vendor_pdata.reset = mdss_dsi_panel_reset_renesas;
	}
	vendor_pdata.detect = mdss_dsi_panel_detect;
	vendor_pdata.update_panel = mdss_dsi_panel_update_panel_info;
	vendor_pdata.disp_on = mdss_dsi_panel_disp_on;
	vendor_pdata.update_fps = mdss_dsi_panel_fps_data_update;
	vendor_pdata.on = mdss_dsi_panel_on;
	vendor_pdata.off = mdss_dsi_panel_off;
	vendor_pdata.bl_fnc = mdss_dsi_panel_bl_ctrl;
	vendor_pdata.pcc_setup = mdss_dsi_panel_pcc_setup;

	mdss_dsi_panel_fps_data_init(&fpsd);

	rc = dsi_panel_device_register(pdev, &vendor_pdata);
	if (rc)
		goto error;

	pdev->dev.platform_data = &vendor_pdata;

	return 0;
error:
	device_unregister(&virtdev);
	return rc;
}

static int __devexit mdss_dsi_panel_remove(struct platform_device *pdev)
{
	struct mdss_panel_common_pdata *vendor_pdata;

	vendor_pdata = dev_get_platdata(&pdev->dev);
	kfree(vendor_pdata->wide_tbl);
	vendor_pdata->wide_tbl = NULL;
	kfree(vendor_pdata->sub_tbl);
	vendor_pdata->sub_tbl = NULL;

	return 0;
}

static const struct of_device_id mdss_dsi_panel_match[] = {
	{.compatible = "qcom,mdss-dsi-panel"},
	{}
};

static struct platform_driver this_driver = {
	.probe = mdss_dsi_panel_probe,
	.remove = __devexit_p(mdss_dsi_panel_remove),
	.driver = {
		.name = "dsi_panel",
		.of_match_table = mdss_dsi_panel_match,
	},
};

static int __init mdss_dsi_panel_init(void)
{
	return platform_driver_register(&this_driver);
}
module_init(mdss_dsi_panel_init);
