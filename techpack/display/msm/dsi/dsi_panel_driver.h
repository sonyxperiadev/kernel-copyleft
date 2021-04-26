/* Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
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

#ifndef DSI_PANEL_DRIVER_H
#define DSI_PANEL_DRIVER_H

#include <linux/of_gpio.h>
#include <linux/incell.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include "dsi_panel.h"
#include "dsi_display.h"

/* touch I/F or not */
#define INCELL_TOUCH_RUN	((bool)true)
#define INCELL_TOUCH_IDLE	((bool)false)

/* incell status */
#define INCELL_S000 ((unsigned char)	0x00)
#define INCELL_S001 ((unsigned char)	0x01)
#define INCELL_S010 ((unsigned char)	0x02)
#define INCELL_S011 ((unsigned char)	0x03)
#define INCELL_S100 ((unsigned char)	0x04)
#define INCELL_S101 ((unsigned char)	0x05)
#define INCELL_S110 ((unsigned char)	0x06)
#define INCELL_S111 ((unsigned char)	0x07)

/* incell change status trigger */
#define INCELL_POWER_STATE_ON	BIT(0)
#define INCELL_LOCK_STATE_ON	BIT(1)
#define INCELL_SYSTEM_STATE_ON	BIT(2)
#define INCELL_POWER_STATE_OFF	~INCELL_POWER_STATE_ON
#define INCELL_LOCK_STATE_OFF	~INCELL_LOCK_STATE_ON
#define INCELL_SYSTEM_STATE_OFF	~INCELL_SYSTEM_STATE_ON

/* How to send power sequence */
typedef enum {
	POWER_EXECUTE,
	POWER_SKIP,
} incell_pw_seq;

/* dcs read/write */
#define DTYPE_DCS_WRITE		0x05	/* short write, 0 parameter */
#define DTYPE_DCS_WRITE1	0x15	/* short write, 1 parameter */
#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_LWRITE	0x39	/* long write */

/* generic read/write */
#define DTYPE_GEN_WRITE		0x03	/* short write, 0 parameter */
#define DTYPE_GEN_WRITE1	0x13	/* short write, 1 parameter */
#define DTYPE_GEN_WRITE2	0x23	/* short write, 2 parameter */
#define DTYPE_GEN_LWRITE	0x29	/* long write */
#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_READ1		0x14	/* long read, 1 parameter */
#define DTYPE_GEN_READ2		0x24	/* long read, 2 parameter */

#define DTYPE_COMPRESSION_MODE	0x07	/* compression mode */
#define DTYPE_PPS		0x0a	/* pps */
#define DTYPE_MAX_PKTSIZE	0x37	/* set max packet size */
#define DTYPE_NULL_PKT		0x09	/* null packet, no data */
#define DTYPE_BLANK_PKT		0x19	/* blankiing packet, no data */

#define DTYPE_CM_ON		0x02	/* color mode off */
#define DTYPE_CM_OFF		0x12	/* color mode on */
#define DTYPE_PERIPHERAL_OFF	0x22
#define DTYPE_PERIPHERAL_ON	0x32

/*
 * dcs response
 */
#define DTYPE_ACK_ERR_RESP      0x02
#define DTYPE_EOT_RESP          0x08    /* end of tx */
#define DTYPE_GEN_READ1_RESP    0x11    /* 1 parameter, short */
#define DTYPE_GEN_READ2_RESP    0x12    /* 2 parameter, short */
#define DTYPE_GEN_LREAD_RESP    0x1a
#define DTYPE_DCS_LREAD_RESP    0x1c
#define DTYPE_DCS_READ1_RESP    0x21    /* 1 parameter, short */
#define DTYPE_DCS_READ2_RESP    0x22    /* 2 parameter, short */

enum dbg_cmd_type {
	DCS,
	GEN,
};

struct dsi_reset_cfg {
	struct dsi_reset_seq *seq;
	u32 count;
};

/* control parameters for incell panel */
struct incell_ctrl {
	unsigned char state;
	incell_pw_seq seq;
	bool incell_intf_operation;
	incell_intf_mode intf_mode;
	struct msm_drm_private *priv;
};

/* short flag control default data */
#define SHORT_CHATTER_CNT_START		1
#define SHORT_DEFAULT_TARGET_CHATTER_CNT	3
#define SHORT_DEFAULT_TARGET_CHATTER_INTERVAL	500
#define SHORT_POWER_OFF_RETRY_INTERVAL	500

#define SHORT_WORKER_ACTIVE		true
#define SHORT_WORKER_PASSIVE		false
#define SHORT_IRQF_FLAGS	(IRQF_ONESHOT | IRQF_TRIGGER_RISING)

struct short_detection_ctrl {
	struct delayed_work check_work;
	int current_chatter_cnt;
	int target_chatter_cnt;
	int target_chatter_check_interval;
	int irq_num;
	bool short_check_working;
	bool irq_enable;
};

struct panel_specific_pdata {
	bool cont_splash_enabled;

	struct dsi_reset_seq *sequence_touch;
	int reset_touch_gpio;
	int disp_err_fg_gpio;
	u32 count_touch;

	int lp11_on;
	int panel_reset_on;

	int lp11_off;
	int touch_vddh_off;
	int down_period;

	int touch_reset_off;

	struct dsi_reset_cfg on_seq;
	struct dsi_reset_cfg off_seq;
	bool rst_b_seq;
	bool rst_after_pon;

	int aod_mode;
	int hbm_mode;
	int pre_sod_mode;
	int opec_mode;

	bool display_onoff_state;

	u32 *area_count;
	int *area_count_table;
	int area_count_table_size;
	u32 now_area;
	u32 start_jiffies;
	struct short_detection_ctrl short_det;
};

/* Somc Original function */
int dsi_panel_driver_pinctrl_init(struct dsi_panel *panel);
int dsi_panel_driver_gpio_request(struct dsi_panel *panel);
int dsi_panel_driver_gpio_release(struct dsi_panel *panel);
int dsi_panel_driver_touch_gpio_release(struct dsi_panel *panel);
int dsi_panel_driver_touch_reset(struct dsi_panel *panel);
int dsi_panel_driver_pre_power_on(struct dsi_panel *panel);
int dsi_panel_driver_power_on(struct dsi_panel *panel);
int dsi_panel_driver_power_off(struct dsi_panel *panel);
int dsi_panel_driver_post_power_off(struct dsi_panel *panel);
int dsi_panel_driver_parse_dt(struct dsi_panel *panel,
				      struct device_node *of_node);
int dsi_panel_driver_parse_gpios(struct dsi_panel *panel);
int dsi_panel_driver_reset_panel(struct dsi_panel *panel, bool en);
void dsi_panel_driver_post_enable(struct dsi_panel *panel);
void dsi_panel_driver_pre_disable(struct dsi_panel *panel);
void dsi_panel_driver_disable(struct dsi_panel *panel);
int dsi_panel_driver_create_fs(struct dsi_display *display);
int dsi_panel_tx_cmd(struct dsi_panel *panel, struct dsi_cmd_desc *cmds);
int dsi_panel_rx_cmd(struct dsi_display *display, struct dsi_cmd_desc *cmds,
			struct dsi_display_ctrl *ctrl, char *rbuf, int len);
void dsi_panel_driver_panel_update_area(struct dsi_panel *panel, u32 level);
void dsi_panel_driver_init_area_count(struct dsi_panel *panel);
void dsi_panel_driver_deinit_area_count(struct dsi_panel *panel);
void dsi_panel_driver_oled_short_det_init_works(struct dsi_display *display);
void dsi_panel_driver_oled_short_check_worker(struct work_struct *work);
void dsi_panel_driver_oled_short_det_enable(
			struct panel_specific_pdata *spec_pdata, bool inWork);
void dsi_panel_driver_oled_short_det_disable(
			struct panel_specific_pdata *spec_pdata);

/* For incell driver */
struct incell_ctrl *incell_get_info(void);
int dsi_panel_driver_touch_reset_ctrl(struct dsi_panel *panel, bool en);
bool dsi_panel_driver_is_power_on(unsigned char state);
bool dsi_panel_driver_is_power_lock(unsigned char state);
void incell_driver_init(struct msm_drm_private *priv);

/* Qualcomm Original function */
int dsi_panel_reset(struct dsi_panel *panel);
int dsi_panel_set_pinctrl_state(struct dsi_panel *panel, bool enable);
int dsi_display_cmd_engine_enable(struct dsi_display *display);
int dsi_display_cmd_engine_disable(struct dsi_display *display);

#endif /* DSI_PANEL_DRIVER_H */
