/* drivers/video/msm/mipi_r63306_panels/mipi_sharp_ls043k3sx04.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#include "../msm_fb.h"
#include "../mipi_dsi.h"
#include "../mipi_dsi_panel.h"

/* Initial Sequence */
static char mcap[] = {
	0xB0, 0x00
};

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
static char mcap_unlock_reg[] = {
	0xB0, 0x00
};
#endif

static char ltps_if_ctrl[] = {
	0xC4, 0xC3, 0x29
};
static char gamma_ctrl[] = {
	0xC8, 0x10, 0x00, 0x1F, 0x00
};
static char gamma_ctrl_set_r_pos[] = {
	0xC9, 0x08, 0x03, 0x01, 0x01, 0x02, 0x05, 0x11,
	0x18, 0x10, 0x0C, 0x1F, 0x10, 0x20
};
static char gamma_ctrl_set_r_neg[] = {
	0xCA, 0x26, 0x2B, 0x50, 0x4F, 0x4E, 0x49, 0x3F,
	0x36, 0x3E, 0x41, 0x34, 0x29, 0x19
};
static char gamma_ctrl_set_g_pos[] = {
	0xCB, 0x29, 0x21, 0x1F, 0x1E, 0x1A, 0x13, 0x19,
	0x1C, 0x15, 0x11, 0x24, 0x14, 0x20
};
static char gamma_ctrl_set_g_neg[] = {
	0xCC, 0x09, 0x11, 0x32, 0x33, 0x36, 0x3D, 0x36,
	0x33, 0x39, 0x3C, 0x2C, 0x21, 0x19
};
static char gamma_ctrl_set_b_pos[] = {
	0xCD, 0x3C, 0x33, 0x32, 0x2A, 0x26, 0x14, 0x1B,
	0x1F, 0x19, 0x16, 0x26, 0x10, 0x20
};
static char gamma_ctrl_set_b_neg[] = {
	0xCE, 0x00, 0x00, 0x1F, 0x26, 0x2A, 0x3B, 0x34,
	0x2E, 0x34, 0x38, 0x28, 0x22, 0x19
};
static char auto_cmd_refresh[] = {
	0xB2, 0x00
};
static char panel_driving[] = {
	0xC0, 0x41, 0x02, 0x7F, 0xC9, 0x07
};
static char mcap_lock[] = {
	0xB0, 0x03
};

/* Display ON Sequence */
static char exit_sleep[] = {
	0x11
};
static char display_on[] = {
	0x29
};

/* Display OFF Sequence */
static char display_off[] = {
	0x28
};

static char enter_sleep[] = {
	0x10
};

/* Reading DDB Sequence */
static char read_ddb_start[] = {
	0xA1, 0x00
};

static struct dsi_cmd_desc sharp_display_init_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(mcap), mcap},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ltps_if_ctrl), ltps_if_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl), gamma_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_r_pos), gamma_ctrl_set_r_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_r_neg), gamma_ctrl_set_r_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_g_pos), gamma_ctrl_set_g_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_g_neg), gamma_ctrl_set_g_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_b_pos), gamma_ctrl_set_b_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_b_neg), gamma_ctrl_set_b_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(auto_cmd_refresh), auto_cmd_refresh},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(panel_driving), panel_driving},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(mcap_lock), mcap_lock},
};

static struct dsi_cmd_desc sharp_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc sharp_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc read_ddb_start_cmds[] = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_ddb_start), read_ddb_start},
};

#ifdef CONFIG_FB_MSM_RECOVER_PANEL

static char nvm_rsp1[] = {
	0xE7, 0x07, 0x0E, 0x0E, 0x35  /* default data */
};
static char dev_code[] = {
	0xBF, 0x01, 0x22, 0x33, 0x06, 0xA4
};
static char test_mode1[] = {
	0xE4, 0x00, 0x00, 0x00, 0xF0, 0xFF
};
static char test_mode2[] = {
	0xE4, 0x39, 0x87
};
static char test_mode3[] = {
	0xE4, 0x00, 0x00, 0x00, 0x00, 0x00
};
static char test_mode4[] = {
	0xE4, 0xB9, 0x47
};
static char test_mode5[] = {
	0xE4, 0xBD
};
static char nvm_read_e4_reg[] = {
	0xE4
};
static char nvm_read_e1_reg[] = {
	0xE1
};
static char test_mode7[] = {
	0xFD, 0x04, 0x55, 0x53, 0x00, 0x70, 0xFF, 0x10, 0x33, 0x22, 0x22, 0x22,
	0x37, 0x00
};

static char nvm_erase_and_write_rsp[] = {
	0xE0
};
static char nvm_erase_and_write_user[] = {
	0xE0, 0x00
};
static char pix_fmt[] = {
	0xB4, 0x02
};
static char dsi_ctl[] = {
	0xB6, 0x51, 0xE3
};
static char dsp_h_timming[] = {
	0xC1, 0x00, 0xB6, 0x00, 0x00, 0x9F, 0x00, 0x00, 0x9F,
	0x09, 0x23, 0x09, 0x00, 0x00, 0x00, 0x01
};
static char src_output[] = {
	0xC2, 0x00, 0x09, 0x09, 0x00, 0x00
};
static char gate_drv_if_ctl[] = {
	0xC3, 0x04
};
static char pbctrl_ctl[] = {
	0xC5, 0x00, 0x02
};
static char dsp_rgb_sw_odr[] = {
	0xC6, 0x13, 0x20, 0x20, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static char ltps_if_ctl[] = {
	0xC7, 0x00
};
static char pow_set1[] = {
	0xD0, 0x6C, 0x65, 0x09, 0x18, 0xA0, 0x00, 0x20, 0x00
};
static char pow_set2[] = {
	0xD1, 0x77, 0xD4
};
static char pow_internal[] = {
	0xD3, 0x33
};
static char vol_set[] = {
	0xD5, 0x07, 0x07
};
static char vcomdc_set[] = {
	0xDE, 0x00, 0x00, 0x31, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00  /* default data */
};
static char nvm_ld_ctl[] = {
	0xE2, 0x03
};
static char reg_wri_ctl[] = {
	0xE5, 0x01
};
static char ddb_wri_ctl[] = {
	0xE6, 0x12, 0x61, 0x41, 0x46, 0x03, 0x01, 0x00
};
static char nvm_read_e6_reg[] = {
	0xE6
};
static char nvm_read_e7_reg[] = {
	0xE7
};
static char nvm_read_de_reg[] = {
	0xDE
};
static char nvm_read_bf_reg[] = {
	0xBF
};

static struct dsi_cmd_desc nvm_enter_sleep_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 80, sizeof(enter_sleep), enter_sleep}
};
static struct dsi_cmd_desc nvm_exit_sleep_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 80, sizeof(exit_sleep), exit_sleep}
};
static struct dsi_cmd_desc nvm_mcap_unlock_cmds[] = {
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mcap_unlock_reg),
							mcap_unlock_reg},
};
static struct dsi_cmd_desc nvm_mcap_lock_cmds[] = {
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mcap_lock), mcap_lock},
};
static struct dsi_cmd_desc nvm_read_e6_cmds[] = {
	{DTYPE_GEN_READ, 1, 0, 0, 0, sizeof(nvm_read_e6_reg), nvm_read_e6_reg},
};
static struct dsi_cmd_desc nvm_read_e7_cmds[] = {
	{DTYPE_GEN_READ, 1, 0, 0, 0, sizeof(nvm_read_e7_reg), nvm_read_e7_reg},
};
static struct dsi_cmd_desc nvm_read_de_cmds[] = {
	{DTYPE_GEN_READ, 1, 0, 0, 0, sizeof(nvm_read_de_reg), nvm_read_de_reg},
};
static struct dsi_cmd_desc nvm_read_bf_cmds[] = {
	{DTYPE_GEN_READ, 1, 0, 0, 0, sizeof(nvm_read_bf_reg), nvm_read_bf_reg},
};
static struct dsi_cmd_desc nvm_read_e4_cmds[] = {
	{DTYPE_GEN_READ, 1, 0, 0, 0, sizeof(nvm_read_e4_reg), nvm_read_e4_reg},
};
static struct dsi_cmd_desc nvm_erase_finish_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode3), test_mode3},
};
static struct dsi_cmd_desc nvm_read_e1_cmds[] = {
	{DTYPE_GEN_READ, 1, 0, 0, 0, sizeof(nvm_read_e1_reg), nvm_read_e1_reg},
};

static struct dsi_cmd_desc nvm_erase_all_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode1), test_mode1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_rsp1), nvm_rsp1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dev_code), dev_code},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(auto_cmd_refresh),
							auto_cmd_refresh},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pix_fmt), pix_fmt},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dsi_ctl), dsi_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(panel_driving), panel_driving},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dsp_h_timming), dsp_h_timming},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(src_output), src_output},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gate_drv_if_ctl),
							gate_drv_if_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ltps_if_ctrl), ltps_if_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pbctrl_ctl), pbctrl_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dsp_rgb_sw_odr), dsp_rgb_sw_odr},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(ltps_if_ctl), ltps_if_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pow_set1), pow_set1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pow_set2), pow_set2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pow_internal), pow_internal},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(vol_set), vol_set},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(vcomdc_set), vcomdc_set},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ld_ctl), nvm_ld_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(reg_wri_ctl), reg_wri_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ddb_wri_ctl), ddb_wri_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl), gamma_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_r_pos),
							gamma_ctrl_set_r_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_r_neg),
							gamma_ctrl_set_r_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_g_pos),
							gamma_ctrl_set_g_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_g_neg),
							gamma_ctrl_set_g_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_b_pos),
							gamma_ctrl_set_b_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_b_neg),
							gamma_ctrl_set_b_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode7), test_mode7},
	{DTYPE_DCS_WRITE, 1, 0, 0, 200, sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode4), test_mode4},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1300, sizeof(test_mode5), test_mode5},
};

static struct dsi_cmd_desc nvm_write_trim_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode1), test_mode1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_rsp1), nvm_rsp1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dev_code), dev_code},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode2), test_mode2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1300, sizeof(nvm_erase_and_write_rsp),
						nvm_erase_and_write_rsp},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode3), test_mode3},
};

static struct dsi_cmd_desc nvm_write_user_cmds[] = {
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(auto_cmd_refresh),
						auto_cmd_refresh},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pix_fmt), pix_fmt},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dsi_ctl), dsi_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(panel_driving), panel_driving},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dsp_h_timming), dsp_h_timming},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(src_output), src_output},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gate_drv_if_ctl),
						gate_drv_if_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ltps_if_ctrl), ltps_if_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pbctrl_ctl), pbctrl_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(dsp_rgb_sw_odr), dsp_rgb_sw_odr},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(ltps_if_ctl), ltps_if_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pow_set1), pow_set1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pow_set2), pow_set2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(pow_internal), pow_internal},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(vol_set), vol_set},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(vcomdc_set), vcomdc_set},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ld_ctl), nvm_ld_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(reg_wri_ctl), reg_wri_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ddb_wri_ctl), ddb_wri_ctl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl), gamma_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_r_pos),
						gamma_ctrl_set_r_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_r_neg),
						gamma_ctrl_set_r_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_g_pos),
						gamma_ctrl_set_g_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_g_neg),
						gamma_ctrl_set_g_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_b_pos),
						gamma_ctrl_set_b_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_ctrl_set_b_neg),
						gamma_ctrl_set_b_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(test_mode7), test_mode7},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 2500, sizeof(nvm_erase_and_write_user),
						nvm_erase_and_write_user},
};

static struct dsi_nvm_rewrite_ctl dsi_nvrw_ctl = {
	.nvm_erase_all		= nvm_erase_all_cmds,
	.nvm_read_e4		= nvm_read_e4_cmds,
	.nvm_erase_finish	= nvm_erase_finish_cmds,
	.nvm_read_e1		= nvm_read_e1_cmds,
	.nvm_write_trim		= nvm_write_trim_cmds,
	.nvm_write_user		= nvm_write_user_cmds,
	.nvm_enter_sleep	= nvm_enter_sleep_cmds,
	.nvm_exit_sleep		= nvm_exit_sleep_cmds,
	.nvm_mcap_unlock	= nvm_mcap_unlock_cmds,
	.nvm_mcap_lock		= nvm_mcap_lock_cmds,
	.nvm_read_e6		= nvm_read_e6_cmds,
	.nvm_read_e7		= nvm_read_e7_cmds,
	.nvm_read_de		= nvm_read_de_cmds,
	.nvm_read_bf		= nvm_read_bf_cmds,
	.nvm_bf			= dev_code,
	.nvm_e6			= ddb_wri_ctl,
	.nvm_e7			= nvm_rsp1,
	.nvm_de			= vcomdc_set,
	.nvm_e7_nbr_params	= ARRAY_SIZE(nvm_rsp1) - 1,
	.nvm_de_nbr_params	= ARRAY_SIZE(vcomdc_set) - 1,
	.nvm_erase_all_size	= ARRAY_SIZE(nvm_erase_all_cmds),
	.nvm_read_e4_size	= ARRAY_SIZE(nvm_read_e4_cmds),
	.nvm_erase_finish_size	= ARRAY_SIZE(nvm_erase_finish_cmds),
	.nvm_read_e1_size	= ARRAY_SIZE(nvm_read_e1_cmds),
	.nvm_write_trim_size	= ARRAY_SIZE(nvm_write_trim_cmds),
	.nvm_write_user_size	= ARRAY_SIZE(nvm_write_user_cmds),
	.nvm_enter_sleep_size	= ARRAY_SIZE(nvm_enter_sleep_cmds),
	.nvm_exit_sleep_size	= ARRAY_SIZE(nvm_exit_sleep_cmds),
	.nvm_mcap_unlock_size	= ARRAY_SIZE(nvm_mcap_unlock_cmds),
	.nvm_mcap_lock_size	= ARRAY_SIZE(nvm_mcap_lock_cmds),
	.nvm_restore_e7		= true,
	.nvm_restore_de		= true,
	.nvm_de_valid_param_start	= 2,
	.nvm_de_valid_param_end		= 3,
};

#endif /* CONFIG_FB_MSM_RECOVER_PANEL */

static const struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db[] = {
	/* 720*1280, RGB888, 4 Lane 60 fps video mode */
	{
		/* regulator */
		{0x03, 0x0a, 0x04, 0x00, 0x20},
		/* timing   */
		{0x7b, 0x1b, 0x12, 0x00, 0x40, 0x49, 0x17, 0x1e,
		 0x1e, 0x03, 0x04, 0xa0},
		/* phy ctrl */
		{0x5f, 0x00, 0x00, 0x10},
		/* strength */
		{0xff, 0x00, 0x06, 0x00},
		/* pll control */
		{0x00, 0x9e, 0x31, 0xd9, 0x00, 0x50, 0x48, 0x63,
		 0x41, 0x0f, 0x03,
		 0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
	},
};
static const struct mipi_dsi_lane_cfg  lncfg = {
	/* DSI1_DSIPHY_LN_CFG */
	.ln_cfg = {
		{0x80, 0xEF, 0x00},
		{0x80, 0xEF, 0x00},
		{0x80, 0xEF, 0x00},
		{0x80, 0xEF, 0x00}
	},
	/* DSI1_DSIPHY_LN_TEST_DATAPATH */
	.ln_dpath = {0x00, 0x00, 0x00, 0x00},
	/* DSI1_DSIPHY_LN_TEST_STR */
	.ln_str = {
		{0x01, 0x66},
		{0x01, 0x66},
		{0x01, 0x66},
		{0x01, 0x66}
	},
	/* DSI1_DSIPHY_LNCK_CFG */
	.lnck_cfg = {0x40, 0xEF, 0x00},
	/* DSI1_DSIPHY_LNCK_TEST_DATAPATH */
	.lnck_dpath = 0x0,
	/* DSI1_DSIPHY_LNCK_TEST_STR */
	.lnck_str = {0x01, 0x88},
};

static struct msm_panel_info pinfo;

static struct msm_panel_info *get_panel_info(void)
{
	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 40;
	pinfo.lcdc.h_front_porch = 160;
	pinfo.lcdc.h_pulse_width = 4;
	pinfo.lcdc.v_back_porch = 6;
	pinfo.lcdc.v_front_porch = 8;
	pinfo.lcdc.v_pulse_width = 2;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0;	/* black */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 15;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 431000000;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.r_sel = 0;
	pinfo.mipi.g_sel = 0;
	pinfo.mipi.b_sel = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x04;
	pinfo.mipi.t_clk_pre = 0x1B;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate  = 60;
	pinfo.mipi.dsi_phy_db =
		(struct mipi_dsi_phy_ctrl *)dsi_video_mode_phy_db;

	return &pinfo;
}

static struct dsi_controller dsi_video_controller_panel = {
	.get_panel_info = get_panel_info,
	.display_init_cmds = sharp_display_init_cmds,
	.display_on_cmds = sharp_display_on_cmds,
	.display_off_cmds = sharp_display_off_cmds,
	.read_id_cmds = read_ddb_start_cmds,
	.display_init_cmds_size = ARRAY_SIZE(sharp_display_init_cmds),
	.display_on_cmds_size = ARRAY_SIZE(sharp_display_on_cmds),
	.display_off_cmds_size = ARRAY_SIZE(sharp_display_off_cmds),
};

static char ddb_val_1a[] = {
	0x12, 0x61, 0x41, 0x46, 0x1a, 0x01, 0x00, 0xff
};

static char ddb_val[] = {
	0x12, 0x61, 0x41, 0x46, 0xff, 0x01, 0x00, 0xff
};

static char default_ddb_val[] = {
	0x12, 0x61, 0x41, 0x46
};

const struct panel_id sharp_ls043k3sx04_panel_id_1a = {
	.name = "mipi_video_sharp_wxga_ls043k3sx04_id_1a",
	.pctrl = &dsi_video_controller_panel,
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	.pnvrw_ctl = &dsi_nvrw_ctl,
#endif
	.id = ddb_val_1a,
	.id_num = ARRAY_SIZE(ddb_val_1a),
	.width = 53,
	.height = 95,
	.plncfg = &lncfg,
};

const struct panel_id sharp_ls043k3sx04_panel_id = {
	.name = "mipi_video_sharp_wxga_ls043k3sx04",
	.pctrl = &dsi_video_controller_panel,
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	.pnvrw_ctl = &dsi_nvrw_ctl,
	.use_if_nv_fail = true,
#endif
	.id = ddb_val,
	.id_num = ARRAY_SIZE(ddb_val),
	.width = 53,
	.height = 95,
	.plncfg = &lncfg,
};

const struct panel_id sharp_ls043k3sx04_panel_default = {
	.name = "mipi_sharp_panel",
	.pctrl = &dsi_video_controller_panel,
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	.pnvrw_ctl = &dsi_nvrw_ctl,
#endif
	.id = default_ddb_val,
	.id_num = ARRAY_SIZE(default_ddb_val),
	.width = 53,
	.height = 95,
	.plncfg = &lncfg,
};

