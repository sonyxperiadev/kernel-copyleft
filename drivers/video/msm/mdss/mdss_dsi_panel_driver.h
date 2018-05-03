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

#ifndef MDSS_DSI_PANEL_DRIVER_H
#define MDSS_DSI_PANEL_DRIVER_H

#include <linux/incell.h>

#include "mdss_dsi.h"
#include "mdss_fb.h"
#include "mdss_mdp.h"

/* pcc data infomation */
#define PCC_STS_UD	0x01	/* update request */
#define UNUSED				0xff
#define CLR_DATA_REG_LEN_RENE_DEFAULT	2
#define CLR_DATA_REG_LEN_NOVA_DEFAULT	1
#define CLR_DATA_REG_LEN_NOVA_AUO	3
#define CLR_DATA_REG_LEN_RENE_SR	1
enum {
	CLR_DATA_UV_PARAM_TYPE_NONE,
	CLR_DATA_UV_PARAM_TYPE_RENE_DEFAULT,
	CLR_DATA_UV_PARAM_TYPE_NOVA_DEFAULT,
	CLR_DATA_UV_PARAM_TYPE_NOVA_AUO,
	CLR_DATA_UV_PARAM_TYPE_RENE_SR
};

/* fb_notifier call type */
#define FB_NOTIFIER_PRE		((bool)true)
#define FB_NOTIFIER_POST	((bool)false)

/* touch I/F data information for incell */
/* touch I/F or not */
#define INCELL_TOUCH_RUN	((bool)true)
#define INCELL_TOUCH_IDLE	((bool)false)

/* incell status */
#define INCELL_POWER_STATE_ON	BIT(0)
#define INCELL_EWU_STATE_ON	BIT(1)
#define INCELL_LOCK_STATE_ON	BIT(2)
#define INCELL_SYSTEM_STATE_ON	BIT(3)

#define INCELL_POWER_STATE_OFF	~INCELL_POWER_STATE_ON
#define INCELL_EWU_STATE_OFF	~INCELL_EWU_STATE_ON
#define INCELL_LOCK_STATE_OFF	~INCELL_LOCK_STATE_ON
#define INCELL_SYSTEM_STATE_OFF	~INCELL_SYSTEM_STATE_ON

/* SLE000-P0: Initial setting the case of booting by Kernel */
#define INCELL_INIT_STATE_KERNEL	(0x0f \
					& INCELL_POWER_STATE_OFF \
					& INCELL_EWU_STATE_OFF \
					& INCELL_LOCK_STATE_OFF \
					& INCELL_SYSTEM_STATE_OFF)

/* SLE100-P1: Initial setting the case of booting by LK */
#define INCELL_INIT_STATE_LK	(INCELL_POWER_STATE_ON \
			       | INCELL_SYSTEM_STATE_ON)

/* The conditions of status if incell_work needed or not */
#define INCELL_WORK_NEED_P_OFF		INCELL_POWER_STATE_ON
#define INCELL_WORK_NEED_P_ON		INCELL_SYSTEM_STATE_ON
#define INCELL_WORK_NEED_P_ON_EWU	(INCELL_SYSTEM_STATE_ON \
				       | INCELL_EWU_STATE_ON)

/* status to adjust power for incell panel or not */
typedef enum {
	INCELL_WORKER_OFF,
	INCELL_WORKER_PENDING,
	INCELL_WORKER_ON,
} incell_worker_state;

/*
 * Incell status change mode
 *
 * SP means the below.
 * S : System
 * P : Power
 */
typedef enum {
	INCELL_STATE_NONE,
	INCELL_STATE_S_OFF,
	INCELL_STATE_P_OFF,
	INCELL_STATE_SP_OFF,
	INCELL_STATE_S_ON,
	INCELL_STATE_P_ON,
	INCELL_STATE_SP_ON,
} incell_state_change;

/* How to send power sequence */
typedef enum {
	POWER_OFF_EXECUTE,
	POWER_OFF_SKIP,
	POWER_ON_EXECUTE,
	POWER_ON_SKIP,
	POWER_ON_EWU_SEQ,
} incell_pw_seq;

/* control parameters for incell panel */
struct incell_ctrl {
	unsigned char state;
	incell_state_change change_state;
	incell_pw_seq seq;

	bool incell_intf_operation;
	incell_intf_mode intf_mode;

	incell_worker_state worker_state;
	struct work_struct incell_work;
};

#define DEFAULT_FPS_LOG_INTERVAL 100
#define DEFAULT_FPS_ARRAY_SIZE 120

#define HYBRID_INCELL ((bool)true)
#define FULL_INCELL   ((bool)false)

typedef enum {
	FPS_PORCH_RNG_MIN = 0,
	FPS_PORCH_RNG_MAX,
	FPS_PORCH_RNG_NUM,
} fps_porch_rng_index;

struct fps_array {
	u32 frame_nbr;
	u32 time_delta;
};

struct fps_data {
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
	u16 fps_array_cnt;
	bool vps_en;
};

typedef enum FPS_TYPE {
	FPSD,
	VPSD
} fps_type;

struct change_fps_send_pos {
	int num;
	int *pos;
};

struct change_fps {
	/* common */
	bool enable;
	bool susres_mode;
	u32 dric_vdisp;
	struct change_fps_send_pos send_pos;

	/* hybred */
	u32 dric_mclk;
	u32 dric_vtouch;
	u32 porch_range[FPS_PORCH_RNG_NUM];
	u16 dric_rtn;
	u16 send_byte;
	u16 mask_pos;
	char mask;

	/* full */
	u32 dric_rclk;
	u32 dric_tp;
	u32 dric_total_porch;
};

struct mdss_panel_power_seq {
	int seq_num;
	int *rst_seq;
	bool rst_b_seq;

	int disp_vdd;
	int disp_vddio;
	int disp_vsp;
	int disp_vsn;
	int touch_avdd;
	int touch_vddio;
	int touch_reset;
	int touch_reset_first;
	int touch_intn;
};

struct mdss_pcc_color_tbl {
	u32 color_type;
	u32 area_num;
	u32 u_min;
	u32 u_max;
	u32 v_min;
	u32 v_max;
	u32 r_data;
	u32 g_data;
	u32 b_data;
} __packed;

struct mdss_pcc_data {
	struct mdss_pcc_color_tbl *color_tbl;
	u32 tbl_size;
	u8 tbl_idx;
	u8 pcc_sts;
	u32 u_data;
	u32 v_data;
	int param_type;
};

/* lab/ibb control default data */
#define OVR_LAB_VOLTAGE			BIT(0)
#define OVR_IBB_VOLTAGE			BIT(1)
#define OVR_LAB_CURRENT_MAX		BIT(2)
#define OVR_IBB_CURRENT_MAX		BIT(3)
#define OVR_LAB_PRECHARGE_CTL		BIT(4)
#define OVR_LAB_SOFT_START_CTL		BIT(5)
#define OVR_IBB_SOFT_START_CTL		BIT(6)
#define OVR_LAB_PD_CTL			BIT(7)
#define OVR_IBB_PD_CTL			BIT(8)

#define QPNP_REGULATOR_VSP_V_5P4V	((u32)5600000)
#define QPNP_REGULATOR_VSN_V_M5P4V	((u32)5600000)
#define LAB_CURRENT_MAX			((u32)200)
#define IBB_CURRENT_MAX			((u32)800)
#define LAB_FAST_PRECHARGE_TIME		((u32)500)
#define LAB_SOFT_START_TIME		((u32)300)
#define IBB_SOFT_START_RESISTOR		((u32)32)

struct mdss_panel_labibb_data {
	u16 labibb_ctrl_state;

	u32 lab_output_voltage;
	u32 ibb_output_voltage;
	u32 lab_current_max;
	u32 ibb_current_max;
	u32 lab_fast_precharge_time;
	u32 lab_soft_start;
	u32 ibb_soft_start;

	bool lab_fast_precharge_en;
	bool lab_pd_full;
	bool ibb_pd_full;
};

struct mdss_panel_specific_pdata {
	int touch_vddio_gpio;
	int touch_reset_gpio;
	int touch_int_gpio;

	bool pcc_enable;
	struct dsi_panel_cmds pre_uv_read_cmds;
	struct dsi_panel_cmds uv_read_cmds;
	struct mdss_pcc_data pcc_data;
	u32 rev_u[2], rev_v[2];

	struct mdss_panel_power_seq on_seq;
	struct mdss_panel_power_seq off_seq;
	u32 down_period;

	struct mdss_panel_labibb_data labibb;

	struct mdss_panel_power_seq ewu_seq;

	int (*pcc_setup)(struct mdss_panel_data *pdata);
	void (*crash_counter_reset)(void);
	void (*blackscreen_det)(void);
	void (*fff_time_update)(struct mdss_panel_specific_pdata *spec_pdata);
	void (*black_screen_off)(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
	bool resume_started;

	bool panel_type;
	struct change_fps chg_fps;
	int input_fpks;
	struct dsi_panel_cmds fps_cmds;

	u32 disp_on_in_hs;
	u32 wait_time_before_post_on_cmd;
};

void mdss_dsi_panel_driver_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void mdss_dsi_panel_driver_fps_data_update(
		struct msm_fb_data_type *mfd, fps_type type);
struct fps_data mdss_dsi_panel_driver_get_fps_data(void);
struct fps_data mdss_dsi_panel_driver_get_vps_data(void);
ssize_t mdss_dsi_panel_driver_vsyncs_per_ksecs_store(struct device *dev,
			 const char *buf, size_t count);

ssize_t mdss_dsi_panel_driver_change_fpks_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
ssize_t mdss_dsi_panel_driver_change_fpks_show(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t mdss_dsi_panel_driver_change_fps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
ssize_t mdss_dsi_panel_driver_change_fps_show(struct device *dev,
		struct device_attribute *attr, char *buf);

void mdss_dsi_panel_driver_detection(struct platform_device *pdev,
		struct device_node **np);
int mdss_dsi_panel_driver_pinctrl_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int mdss_dsi_panel_driver_touch_pinctrl_set_state(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool active);
int mdss_dsi_panel_driver_power_off(struct mdss_panel_data *pdata);
int mdss_dsi_panel_driver_power_on(struct mdss_panel_data *pdata);
void mdss_dsi_panel_driver_off(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void mdss_dsi_panel_driver_post_on(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int mdss_dsi_panel_driver_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void mdss_dsi_panel_driver_gpio_free(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void mdss_dsi_panel_driver_parse_gpio_params(struct platform_device *ctrl_pdev,
		struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int mdss_dsi_panel_driver_reset_panel(struct mdss_panel_data *pdata,
		int enable);
int mdss_dsi_panel_driver_reset_touch(struct mdss_panel_data *pdata,
		int enable);
int mdss_dsi_panel_driver_parse_dt(struct device_node *np,
		struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int mdss_dsi_panel_pcc_setup(struct mdss_panel_data *pdata);
struct mdss_panel_specific_pdata *mdss_panel2spec_pdata(
	struct mdss_panel_data *pdata);
void mdss_dsi_panel_driver_labibb_vreg_init(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void mdss_dsi_panel_driver_wait_before_post_on_cmds(
			struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void mdss_dsi_panel_driver_post_on_event(struct mdss_mdp_ctl *ctl);
bool mdss_dsi_panel_driver_check_disp_on_in_hs(struct mdss_mdp_ctl *ctl);
void mdss_dsi_panel_driver_check_splash_enable(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
void mdss_dsi_panel_driver_fb_notifier_call_chain(
		struct msm_fb_data_type *mfd, int blank, bool type);

/* For incell driver */
struct incell_ctrl *incell_get_info(void);
void incell_panel_power_worker_canceling(struct incell_ctrl *incell);
void incell_driver_init(void);
bool mdss_dsi_panel_driver_is_power_lock(unsigned char state);
bool mdss_dsi_panel_driver_is_power_on(unsigned char state);
bool mdss_dsi_panel_driver_is_ewu(unsigned char state);
bool mdss_dsi_panel_driver_is_system_on(unsigned char state);
void mdss_dsi_panel_driver_state_change_off(struct incell_ctrl *incell);
void mdss_dsi_panel_driver_power_off_ctrl(struct incell_ctrl *incell);
void mdss_dsi_panel_driver_state_change_on(struct incell_ctrl *incell);
void mdss_dsi_panel_driver_power_on_ctrl(struct incell_ctrl *incell);

/* Qualcomm original function */
int mdss_dsi_pinctrl_set_state(struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		bool active);
int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int mdss_dsi_parse_dcs_cmds(struct device_node *np,
				   struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);
void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
				     struct dsi_panel_cmds *pcmds, u32 flags);
int mdss_mdp_tearcheck_enable(struct mdss_mdp_ctl *ctl, bool enable);

static inline struct mdss_dsi_ctrl_pdata *mdss_dsi_get_master_ctrl(
					struct mdss_panel_data *pdata)
{
	int dsi_master = DSI_CTRL_0;

	if (pdata->panel_info.dsi_master == DISPLAY_2)
		dsi_master = DSI_CTRL_1;
	else
		dsi_master = DSI_CTRL_0;

	return mdss_dsi_get_ctrl_by_index(dsi_master);
}

#endif /* MDSS_DSI_PANEL_DRIVER_H */
