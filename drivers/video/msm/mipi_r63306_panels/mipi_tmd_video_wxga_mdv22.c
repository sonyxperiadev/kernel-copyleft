/* drivers/video/msm/mipi_r63306_panels/mipi_tmd_video_wxga_mdv22.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_video_panel.h"

/* Initial Sequence */
static char mcap[] = {
	0xB0, 0x00
};
static char acr[] = {
	0xB2, 0x00
};
static char clk_and_if[] = {
	0xB3, 0x0C
};
static char pixform[] = {
	0xB4, 0x02
};
static char pfm_pwm_ctrl[] = {
	0xB9, 0x01, 0x00, 0x75, 0x00, 0xBA, 0xBB, 0xBF,
	0xCC, 0xE3, 0x46, 0x81, 0xC2
};
static char cabc_on_off[] = {
	0xBB, 0x00
};
static char cabc_user_param[] = {
	0xBE, 0xFF, 0x0F, 0x1A, 0x18, 0x02, 0x40, 0x00,
	0x5D, 0x00, 0x00, 0x80, 0x32
};
static char panel_driving[] = {
	0xC0, 0x40, 0x02, 0x7F, 0xC8, 0x08
};
static char h_timing[] = {
	0xC1, 0x00, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x9C, 0x08, 0x24, 0x0B, 0x00, 0x00, 0x00, 0x00
};
static char src_out[] = {
	0xC2, 0x00, 0x00, 0x0B, 0x00, 0x00
};
static char gate_ic_ctrl[] = {
	0xC3, 0x04
};
static char ltps_if_ctrl_1[] = {
	0xC4, 0x4D, 0x83, 0x00
};
static char src_out_mode[] = {
	0xC6, 0x12, 0x00, 0x08, 0x71, 0x00, 0x00, 0x00,
	0x80, 0x00, 0x04
};
static char ltps_if_ctrl_2[] = {
	0xC7, 0x22
};
static char gamma_ctrl[] = {
	0xC8, 0x4C, 0x0C, 0x0C, 0x0C
};
static char gamma_ctrl_set_a_pos[] = {
	0xC9, 0x00, 0x63, 0x3B, 0x3A, 0x33, 0x25, 0x2A,
	0x30, 0x2B, 0x2F, 0x47, 0x79, 0x3F
};
static char gamma_ctrl_set_a_neg[] = {
	0xCA, 0x00, 0x46, 0x1A, 0x31, 0x35, 0x2F, 0x36,
	0x3A, 0x2D, 0x25, 0x28, 0x5C, 0x3F
};
static char gamma_ctrl_set_b_pos[] = {
	0xCB, 0x00, 0x63, 0x3B, 0x3A, 0x33, 0x25, 0x2A,
	0x30, 0x2B, 0x2F, 0x47, 0x79, 0x3F
};
static char gamma_ctrl_set_b_neg[] = {
	0xCC, 0x00, 0x46, 0x1A, 0x31, 0x35, 0x2F, 0x36,
	0x3A, 0x2D, 0x25, 0x28, 0x5C, 0x3F
};
static char gamma_ctrl_set_c_pos[] = {
	0xCD, 0x00, 0x63, 0x3B, 0x3A, 0x33, 0x25, 0x2A,
	0x30, 0x2B, 0x2F, 0x47, 0x79, 0x3F
};
static char gamma_ctrl_set_c_neg[] = {
	0xCE, 0x00, 0x46, 0x1A, 0x31, 0x35, 0x2F, 0x36,
	0x3A, 0x2D, 0x25, 0x28, 0x5C, 0x3F
};
static char power_setting_1[] = {
	0xD0, 0x6A, 0x64, 0x01
};
static char power_setting_2[] = {
	0xD1, 0x77, 0xD4
};
static char power_setting_internal[] = {
	0xD3, 0x33
};
static char vplvl_vnlvl_setting[] = {
	0xD5, 0x0C, 0x0C
};
static char vcom_dc_setting_1[] = {
	0xD8, 0x34, 0x64, 0x23, 0x25, 0x62, 0x32
};
static char vcom_dc_setting_2[] = {
	0xDE, 0x10, 0xA8, 0x11, 0x08, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};
static char nvm_load_ctrl[] = {
	0xE2, 0x00
};

/* Display ON Sequence */
static char exit_sleep[] = {
	0x11
};
static char display_on[] = {
	0x29
};

/* Display OFF Sequence */
static char enter_sleep[] = {
	0x10
};

/* Reading DDB Sequence */
static char read_ddb_start[] = {
	0xA1, 0x00
};

static struct dsi_cmd_desc tmd_display_init_cmds[] = {
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(mcap), mcap},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(acr), acr},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(clk_and_if), clk_and_if},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(pixform), pixform},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(pfm_pwm_ctrl), pfm_pwm_ctrl},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(cabc_on_off), cabc_on_off},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cabc_user_param), cabc_user_param},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(panel_driving), panel_driving},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(h_timing), h_timing},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(src_out), src_out},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(gate_ic_ctrl), gate_ic_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(ltps_if_ctrl_1), ltps_if_ctrl_1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(src_out_mode), src_out_mode},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(ltps_if_ctrl_2), ltps_if_ctrl_2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl), gamma_ctrl},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_a_pos), gamma_ctrl_set_a_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_a_neg), gamma_ctrl_set_a_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_b_pos), gamma_ctrl_set_b_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_b_neg), gamma_ctrl_set_b_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_c_pos), gamma_ctrl_set_c_pos},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(gamma_ctrl_set_c_neg), gamma_ctrl_set_c_neg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(power_setting_1), power_setting_1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(power_setting_2), power_setting_2},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(power_setting_internal), power_setting_internal},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(vplvl_vnlvl_setting), vplvl_vnlvl_setting},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(vcom_dc_setting_1), vcom_dc_setting_1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(vcom_dc_setting_2), vcom_dc_setting_2},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(nvm_load_ctrl), nvm_load_ctrl},
};

static struct dsi_cmd_desc tmd_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc tmd_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 80,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc read_ddb_start_cmds[] = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_ddb_start), read_ddb_start},
};

static const struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db[] = {
	/* 720*1280, RGB888, 4 Lane 60 fps video mode */
	{
		{0x13, 0x0b, 0x05, 0x50},	/* regulator */
		/* timing   */
		{0xb0, 0x8c, 0x1a, 0x00, 0x1d, 0x92, 0x1e,
		 0x8e, 0x1d, 0x03, 0x04},
		{0x7f, 0x00, 0x00, 0x00},	/* phy ctrl */
		{0xdd, 0x02, 0x86, 0x00},	/* strength */
		/* pll control */
		{0x40, 0xa1, 0xb1, 0xda, 0x00, 0x2f, 0x48, 0x63,
		0x31, 0x0f, 0x03,
		0x05, 0x14, 0x03, 0x00, 0x00, 0x54, 0x06, 0x10, 0x04, 0x00 },
	},
};

static struct msm_panel_info mipi_video_tmd_panel;

static struct msm_panel_info *get_panel_info(void)
{
	mipi_video_tmd_panel.xres = 720;
	mipi_video_tmd_panel.yres = 1280;
	mipi_video_tmd_panel.type = MIPI_VIDEO_PANEL;
	mipi_video_tmd_panel.pdest = DISPLAY_1;
	mipi_video_tmd_panel.wait_cycle = 0;
	mipi_video_tmd_panel.bpp = 24;
	mipi_video_tmd_panel.lcdc.h_back_porch = 45;
	mipi_video_tmd_panel.lcdc.h_front_porch = 128;
	mipi_video_tmd_panel.lcdc.h_pulse_width = 3;
	mipi_video_tmd_panel.lcdc.v_back_porch = 3;
	mipi_video_tmd_panel.lcdc.v_front_porch = 9;
	mipi_video_tmd_panel.lcdc.v_pulse_width = 4;
	mipi_video_tmd_panel.lcdc.border_clr = 0;	/* blk */
	mipi_video_tmd_panel.lcdc.underflow_clr = 0xff;	/* blue */
	mipi_video_tmd_panel.lcdc.hsync_skew = 0;
	mipi_video_tmd_panel.bl_max = 15;
	mipi_video_tmd_panel.bl_min = 1;
	mipi_video_tmd_panel.fb_num = 2;
	mipi_video_tmd_panel.clk_rate = 416000000;

	mipi_video_tmd_panel.mipi.mode = DSI_VIDEO_MODE;
	mipi_video_tmd_panel.mipi.pulse_mode_hsa_he = TRUE;
	mipi_video_tmd_panel.mipi.hfp_power_stop = FALSE;
	mipi_video_tmd_panel.mipi.hbp_power_stop = FALSE;
	mipi_video_tmd_panel.mipi.hsa_power_stop = FALSE;
	mipi_video_tmd_panel.mipi.eof_bllp_power_stop = TRUE;
	mipi_video_tmd_panel.mipi.bllp_power_stop = TRUE;
	mipi_video_tmd_panel.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	mipi_video_tmd_panel.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	mipi_video_tmd_panel.mipi.vc = 0;
	mipi_video_tmd_panel.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	mipi_video_tmd_panel.mipi.r_sel = 0;
	mipi_video_tmd_panel.mipi.g_sel = 0;
	mipi_video_tmd_panel.mipi.b_sel = 0;
	mipi_video_tmd_panel.mipi.data_lane0 = TRUE;
	mipi_video_tmd_panel.mipi.data_lane1 = TRUE;
	mipi_video_tmd_panel.mipi.data_lane2 = TRUE;
	mipi_video_tmd_panel.mipi.data_lane3 = TRUE;
	mipi_video_tmd_panel.mipi.tx_eot_append = TRUE;
	mipi_video_tmd_panel.mipi.t_clk_post = 34;
	mipi_video_tmd_panel.mipi.t_clk_pre = 62;
	mipi_video_tmd_panel.mipi.stream = 0; /* dma_p */
	mipi_video_tmd_panel.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	mipi_video_tmd_panel.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	mipi_video_tmd_panel.mipi.frame_rate  = 60;
	mipi_video_tmd_panel.mipi.dsi_phy_db =
		(struct mipi_dsi_phy_ctrl *)dsi_video_mode_phy_db;

	return &mipi_video_tmd_panel;
}

static struct dsi_video_controller dsi_video_controller_panel = {
	.get_panel_info = get_panel_info,
	.display_init_cmds = tmd_display_init_cmds,
	.display_on_cmds = tmd_display_on_cmds,
	.display_off_cmds = tmd_display_off_cmds,
	.read_id_cmds = read_ddb_start_cmds,
	.display_init_cmds_size = ARRAY_SIZE(tmd_display_init_cmds),
	.display_on_cmds_size = ARRAY_SIZE(tmd_display_on_cmds),
	.display_off_cmds_size = ARRAY_SIZE(tmd_display_off_cmds),
};

static char ddb_val[] = {
	0x01, 0x00, 0x03
};

const struct panel_id tmd_video_wxga_mdv22_panel_id = {
	.name = "mipi_video_tmd_wxga_mdv22",
	.pctrl = &dsi_video_controller_panel,
	.id = ddb_val,
	.id_num = ARRAY_SIZE(ddb_val),
};

