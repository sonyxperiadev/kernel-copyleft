/* drivers/video/msm/mipi_s6d6aa0_panels/mipi_sony_acx439akm.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "../msm_fb.h"
#include "../mipi_dsi.h"
#include "../mipi_dsi_panel.h"

/* Display ON Sequence */
static char exit_sleep[] = {
	0x11
};
static char nvm_reg_access_enable1[] = {
	0xF0, 0x5A, 0x5A
};
static char nvm_reg_access_enable2[] = {
	0xF1, 0x5A, 0x5A
};
static char panel_ctrl_1[] = {
	0xF6, 0x02, 0x0E, 0x0D, 0x1F, 0x08, 0x00, 0x0F,
	0x1C, 0x17, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x22, 0x1E, 0x3C, 0x30, 0x51
};
static char display_scan_ctrl_id_00[] = {
	0x36, 0x40
};
static char display_scan_ctrl[] = {
	0x36, 0x80
};
static char write_ctrl_display[] = {
	0x53, 0x2C
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

static struct dsi_cmd_desc sony_display_on_cmds_id_00[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 140,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(display_scan_ctrl_id_00), display_scan_ctrl_id_00},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc sony_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 140,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(nvm_reg_access_enable1), nvm_reg_access_enable1},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(nvm_reg_access_enable2), nvm_reg_access_enable2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(panel_ctrl_1), panel_ctrl_1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(write_ctrl_display), write_ctrl_display},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(display_scan_ctrl), display_scan_ctrl},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc sony_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 150,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc read_ddb_start_cmds[] = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_ddb_start), read_ddb_start},
};

static const struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db_id_00[] = {
	/* 720*1280, RGB888, 4 Lane 60 fps video mode */
	{
		/* regulator */
		{0x03, 0x0a, 0x04, 0x00, 0x20},
		/* timing */
		{0xa7, 0x89, 0x16, 0x00, 0x18, 0x90, 0x19, 0x8b,
		 0x18, 0x03, 0x04, 0xa0},
		/* phy ctrl */
		{0x5f, 0x00, 0x00, 0x10},
		/* strength */
		{0xff, 0x00, 0x06, 0x00},
		/* pll control */
		{0x01, 0x4f, 0x31, 0xd9, 0x00, 0x40, 0x03, 0x62,
		 0x41, 0x0f, 0x01,
		 0x00, 0x1a, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
	},
};

static const struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db[] = {
	/* 720*1280, RGB888, 4 Lane 60 fps video mode */
	{
		/* regulator */
		{0x03, 0x0a, 0x04, 0x00, 0x20},
		/* timing */
		{0xb0, 0x23, 0x1a, 0x00, 0x94, 0x93, 0x1e, 0x25,
		 0x1d, 0x03, 0x04, 0xa0},
		/* phy ctrl */
		{0x5f, 0x00, 0x00, 0x10},
		/* strength */
		{0xff, 0x00, 0x06, 0x00},
		/* pll control */
		{0x01, 0x91, 0x01, 0x19, 0x00, 0x40, 0x03, 0x62,
		 0x41, 0x0f, 0x01,
		 0x00, 0x1a, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
	},
};

static struct msm_panel_info pinfo;

static struct msm_panel_info *get_panel_info_id_00(void)
{
	/* should fix porch, pulse widht and so on */
	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 10;
	pinfo.lcdc.h_front_porch = 21;
	pinfo.lcdc.h_pulse_width = 3;
	pinfo.lcdc.v_back_porch = 2;
	pinfo.lcdc.v_front_porch = 2;
	pinfo.lcdc.v_pulse_width = 2;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 15;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 350000000;

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
	pinfo.mipi.dlane_swap = 0x01;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.r_sel = 0;
	pinfo.mipi.g_sel = 0;
	pinfo.mipi.b_sel = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x22;
	pinfo.mipi.t_clk_pre = 0x3c;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate  = 60;
	pinfo.mipi.dsi_phy_db =
		(struct mipi_dsi_phy_ctrl *)dsi_video_mode_phy_db_id_00;

	return &pinfo;
}

static struct msm_panel_info *get_panel_info(void)
{
	/* should fix porch, pulse widht and so on */
	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 64;
	pinfo.lcdc.h_front_porch = 64;
	pinfo.lcdc.h_pulse_width = 16;
	pinfo.lcdc.v_back_porch = 30;
	pinfo.lcdc.v_front_porch = 30;
	pinfo.lcdc.v_pulse_width = 4;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 15;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 418000000;

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
	pinfo.mipi.dlane_swap = 0x01;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.r_sel = 0;
	pinfo.mipi.g_sel = 0;
	pinfo.mipi.b_sel = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x19;
	pinfo.mipi.t_clk_pre = 0x2f;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate  = 60;
	pinfo.mipi.dsi_phy_db =
		(struct mipi_dsi_phy_ctrl *)dsi_video_mode_phy_db;

	return &pinfo;
}

static struct dsi_controller dsi_video_controller_panel_id_00 = {
	.get_panel_info = get_panel_info_id_00,
	.display_on_cmds = sony_display_on_cmds_id_00,
	.display_off_cmds = sony_display_off_cmds,
	.read_id_cmds = read_ddb_start_cmds,
	.display_on_cmds_size = ARRAY_SIZE(sony_display_on_cmds_id_00),
	.display_off_cmds_size = ARRAY_SIZE(sony_display_off_cmds),
};

static struct dsi_controller dsi_video_controller_panel = {
	.get_panel_info = get_panel_info,
	.display_on_cmds = sony_display_on_cmds,
	.display_off_cmds = sony_display_off_cmds,
	.read_id_cmds = read_ddb_start_cmds,
	.display_on_cmds_size = ARRAY_SIZE(sony_display_on_cmds),
	.display_off_cmds_size = ARRAY_SIZE(sony_display_off_cmds),
};

static char ddb_val_id_00[] = {
	0x03, 0x00, 0x05
};

static char ddb_val[] = {
	0x81, 0x73, 0x1a, 0x01, 0xff
};

const struct panel_id sony_acx439akm_panel_id_00 = {
	.name = "mipi_video_sony_wxga_acx439akm id 00",
	.pctrl = &dsi_video_controller_panel_id_00,
	.id = ddb_val_id_00,
	.id_num = ARRAY_SIZE(ddb_val_id_00),
};

const struct panel_id sony_acx439akm_panel_id = {
	.name = "mipi_video_sony_wxga_acx439akm",
	.pctrl = &dsi_video_controller_panel,
	.id = ddb_val,
	.id_num = ARRAY_SIZE(ddb_val),
};
