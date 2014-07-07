/* drivers/video/msm/mipi_nt35565_panels/mipi_sharp_ls038k3sx01.c
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
static char exit_sleep[] = {
	0x11
};
static char set_horizontal_address[] = {
	0x2A, 0x00, 0x00, 0x02, 0x1B
};
static char set_vertical_address[] = {
	0x2B, 0x00, 0x00, 0x03, 0xBF
};
static char set_address_mode[] = {
	0x36, 0x00
};
static char set_pixel_format[] = {
	0x3A, 0x77
};
static char set_tear_on[] = {
	0x35, 0x00
};
static char set_tear_scanline[] = {
	0x44, 0x00, 0x01
};
static char wrctrld[] = {
	0x53, 0x2C
};
static char wrdisbv[] = {
	0x51, 0xFF
};
static char wrctrld_cabc[] = {
	0x55, 0x03
};
static char brightness_setting2[] = {
	0x19, 0x81
};
static char brightness_setting10[] = {
	0x22, 0x01
};
static char page1_ctrl[] = {
	0x00, 0x01
};
static char cmd2_p1_lock[] = {
	0x7F, 0xAA
};
static char cmd2_unlock[] = {
	0xF3, 0xAA
};
static char page0_ctrl[] = {
	0x00, 0x00
};
static char display_ctrl[] = {
	0xA2, 0x03
};
static char cmd2_p0_lock[] = {
	0xFF, 0xAA
};
static char reg_ctrl1[] = {
	0x15, 0x3B
};
static char reg_ctrl2[] = {
	0x16, 0x48
};

static char red_gamma_setting_00[] = { 0x24, 0x2D };
static char red_gamma_setting_01[] = { 0x25, 0x3E };
static char red_gamma_setting_02[] = { 0x26, 0x4A };
static char red_gamma_setting_03[] = { 0x27, 0x42 };
static char red_gamma_setting_04[] = { 0x28, 0x18 };
static char red_gamma_setting_05[] = { 0x29, 0x23 };
static char red_gamma_setting_06[] = { 0x2A, 0x4C };
static char red_gamma_setting_07[] = { 0x2B, 0x35 };
static char red_gamma_setting_08[] = { 0x2D, 0x20 };
static char red_gamma_setting_09[] = { 0x2F, 0x29 };
static char red_gamma_setting_10[] = { 0x30, 0x62 };
static char red_gamma_setting_11[] = { 0x31, 0x27 };
static char red_gamma_setting_12[] = { 0x32, 0x4F };
static char red_gamma_setting_13[] = { 0x33, 0x5F };
static char red_gamma_setting_14[] = { 0x34, 0x40 };
static char red_gamma_setting_15[] = { 0x35, 0x4C };
static char red_gamma_setting_16[] = { 0x36, 0x75 };
static char red_gamma_setting_17[] = { 0x37, 0x6F };
static char red_gamma_setting_18[] = { 0x38, 0x50 };
static char red_gamma_setting_19[] = { 0x39, 0x64 };
static char red_gamma_setting_20[] = { 0x3A, 0x70 };
static char red_gamma_setting_21[] = { 0x3B, 0x6A };
static char red_gamma_setting_22[] = { 0x3D, 0x16 };
static char red_gamma_setting_23[] = { 0x3F, 0x22 };
static char red_gamma_setting_24[] = { 0x40, 0x4A };
static char red_gamma_setting_25[] = { 0x41, 0x3B };
static char red_gamma_setting_26[] = { 0x42, 0x1F };
static char red_gamma_setting_27[] = { 0x43, 0x2C };
static char red_gamma_setting_28[] = { 0x44, 0x90 };
static char red_gamma_setting_29[] = { 0x45, 0x27 };
static char red_gamma_setting_30[] = { 0x46, 0x50 };
static char red_gamma_setting_31[] = { 0x47, 0x61 };
static char red_gamma_setting_32[] = { 0x48, 0x5E };
static char red_gamma_setting_33[] = { 0x49, 0x67 };
static char red_gamma_setting_34[] = { 0x4A, 0x88 };
static char red_gamma_setting_35[] = { 0x4B, 0x6D };

static char green_gamma_setting_00[] = { 0x4C, 0x27 };
static char green_gamma_setting_01[] = { 0x4D, 0x48 };
static char green_gamma_setting_02[] = { 0x4E, 0x50 };
static char green_gamma_setting_03[] = { 0x4F, 0x4A };
static char green_gamma_setting_04[] = { 0x50, 0x12 };
static char green_gamma_setting_05[] = { 0x51, 0x26 };
static char green_gamma_setting_06[] = { 0x52, 0x57 };
static char green_gamma_setting_07[] = { 0x53, 0x2B };
static char green_gamma_setting_08[] = { 0x54, 0x15 };
static char green_gamma_setting_09[] = { 0x55, 0x27 };
static char green_gamma_setting_10[] = { 0x56, 0x69 };
static char green_gamma_setting_11[] = { 0x57, 0x1F };
static char green_gamma_setting_12[] = { 0x58, 0x49 };
static char green_gamma_setting_13[] = { 0x59, 0x5E };
static char green_gamma_setting_14[] = { 0x5A, 0x4E };
static char green_gamma_setting_15[] = { 0x5B, 0x5D };
static char green_gamma_setting_16[] = { 0x5C, 0x8A };
static char green_gamma_setting_17[] = { 0x5D, 0x6F };
static char green_gamma_setting_18[] = { 0x5E, 0x6E };
static char green_gamma_setting_19[] = { 0x5F, 0x75 };
static char green_gamma_setting_20[] = { 0x60, 0x7C };
static char green_gamma_setting_21[] = { 0x61, 0x76 };
static char green_gamma_setting_22[] = { 0x62, 0x12 };
static char green_gamma_setting_23[] = { 0x63, 0x27 };
static char green_gamma_setting_24[] = { 0x64, 0x58 };
static char green_gamma_setting_25[] = { 0x65, 0x5D };
static char green_gamma_setting_26[] = { 0x66, 0x1A };
static char green_gamma_setting_27[] = { 0x67, 0x2D };
static char green_gamma_setting_28[] = { 0x68, 0x94 };
static char green_gamma_setting_29[] = { 0x69, 0x21 };
static char green_gamma_setting_30[] = { 0x6A, 0x49 };
static char green_gamma_setting_31[] = { 0x6B, 0x5E };
static char green_gamma_setting_32[] = { 0x6C, 0x67 };
static char green_gamma_setting_33[] = { 0x6D, 0x73 };
static char green_gamma_setting_34[] = { 0x6E, 0x97 };
static char green_gamma_setting_35[] = { 0x6F, 0x6D };

static char blue_gamma_setting_00[] = { 0x70, 0x5F };
static char blue_gamma_setting_01[] = { 0x71, 0x64 };
static char blue_gamma_setting_02[] = { 0x72, 0x72 };
static char blue_gamma_setting_03[] = { 0x73, 0x7C };
static char blue_gamma_setting_04[] = { 0x74, 0x1F };
static char blue_gamma_setting_05[] = { 0x75, 0x29 };
static char blue_gamma_setting_06[] = { 0x76, 0x5F };
static char blue_gamma_setting_07[] = { 0x77, 0x40 };
static char blue_gamma_setting_08[] = { 0x78, 0x1C };
static char blue_gamma_setting_09[] = { 0x79, 0x2B };
static char blue_gamma_setting_10[] = { 0x7A, 0x75 };
static char blue_gamma_setting_11[] = { 0x7B, 0x21 };
static char blue_gamma_setting_12[] = { 0x7C, 0x42 };
static char blue_gamma_setting_13[] = { 0x7D, 0x5D };
static char blue_gamma_setting_14[] = { 0x7E, 0x50 };
static char blue_gamma_setting_15[] = { 0x7F, 0x5E };
static char blue_gamma_setting_16[] = { 0x80, 0x93 };
static char blue_gamma_setting_17[] = { 0x81, 0x6F };
static char blue_gamma_setting_18[] = { 0x82, 0x7F };
static char blue_gamma_setting_19[] = { 0x83, 0x85 };
static char blue_gamma_setting_20[] = { 0x84, 0x94 };
static char blue_gamma_setting_21[] = { 0x85, 0x8D };
static char blue_gamma_setting_22[] = { 0x86, 0x20 };
static char blue_gamma_setting_23[] = { 0x87, 0x2B };
static char blue_gamma_setting_24[] = { 0x88, 0x62 };
static char blue_gamma_setting_25[] = { 0x89, 0x68 };
static char blue_gamma_setting_26[] = { 0x8A, 0x1C };
static char blue_gamma_setting_27[] = { 0x8B, 0x2C };
static char blue_gamma_setting_28[] = { 0x8C, 0x99 };
static char blue_gamma_setting_29[] = { 0x8D, 0x20 };
static char blue_gamma_setting_30[] = { 0x8E, 0x3F };
static char blue_gamma_setting_31[] = { 0x8F, 0x5C };
static char blue_gamma_setting_32[] = { 0x90, 0x6A };
static char blue_gamma_setting_33[] = { 0x91, 0x77 };
static char blue_gamma_setting_34[] = { 0x92, 0xA1 };
static char blue_gamma_setting_35[] = { 0x93, 0x6D };

/* Display ON Sequence */
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
	{DTYPE_DCS_WRITE, 1, 0, 0, 150,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_horizontal_address), set_horizontal_address},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_vertical_address), set_vertical_address},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(set_address_mode), set_address_mode},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(set_pixel_format), set_pixel_format},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(set_tear_on), set_tear_on},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0,
		sizeof(set_tear_scanline), set_tear_scanline},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(wrctrld), wrctrld},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(wrdisbv), wrdisbv},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(wrctrld_cabc), wrctrld_cabc},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(cmd2_unlock), cmd2_unlock},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(page1_ctrl), page1_ctrl},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(brightness_setting2), brightness_setting2},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(brightness_setting10), brightness_setting10},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(cmd2_p1_lock), cmd2_p1_lock},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(cmd2_unlock), cmd2_unlock},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(page0_ctrl), page0_ctrl},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(display_ctrl), display_ctrl},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(cmd2_p0_lock), cmd2_p0_lock},

	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(cmd2_unlock), cmd2_unlock},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(page0_ctrl), page0_ctrl},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(reg_ctrl1), reg_ctrl1},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(reg_ctrl2), reg_ctrl2},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_00), red_gamma_setting_00},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_01), red_gamma_setting_01},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_02), red_gamma_setting_02},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_03), red_gamma_setting_03},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_04), red_gamma_setting_04},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_05), red_gamma_setting_05},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_06), red_gamma_setting_06},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_07), red_gamma_setting_07},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_08), red_gamma_setting_08},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_09), red_gamma_setting_09},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_10), red_gamma_setting_10},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_11), red_gamma_setting_11},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_12), red_gamma_setting_12},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_13), red_gamma_setting_13},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_14), red_gamma_setting_14},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_15), red_gamma_setting_15},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_16), red_gamma_setting_16},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_17), red_gamma_setting_17},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_18), red_gamma_setting_18},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_19), red_gamma_setting_19},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_20), red_gamma_setting_20},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_21), red_gamma_setting_21},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_22), red_gamma_setting_22},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_23), red_gamma_setting_23},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_24), red_gamma_setting_24},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_25), red_gamma_setting_25},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_26), red_gamma_setting_26},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_27), red_gamma_setting_27},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_28), red_gamma_setting_28},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_29), red_gamma_setting_29},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_30), red_gamma_setting_30},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_31), red_gamma_setting_31},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_32), red_gamma_setting_32},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_33), red_gamma_setting_33},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_34), red_gamma_setting_34},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(red_gamma_setting_35), red_gamma_setting_35},

	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_00), green_gamma_setting_00},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_01), green_gamma_setting_01},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_02), green_gamma_setting_02},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_03), green_gamma_setting_03},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_04), green_gamma_setting_04},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_05), green_gamma_setting_05},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_06), green_gamma_setting_06},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_07), green_gamma_setting_07},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_08), green_gamma_setting_08},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_09), green_gamma_setting_09},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_10), green_gamma_setting_10},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_11), green_gamma_setting_11},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_12), green_gamma_setting_12},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_13), green_gamma_setting_13},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_14), green_gamma_setting_14},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_15), green_gamma_setting_15},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_16), green_gamma_setting_16},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_17), green_gamma_setting_17},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_18), green_gamma_setting_18},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_19), green_gamma_setting_19},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_20), green_gamma_setting_20},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_21), green_gamma_setting_21},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_22), green_gamma_setting_22},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_23), green_gamma_setting_23},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_24), green_gamma_setting_24},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_25), green_gamma_setting_25},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_26), green_gamma_setting_26},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_27), green_gamma_setting_27},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_28), green_gamma_setting_28},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_29), green_gamma_setting_29},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_30), green_gamma_setting_30},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_31), green_gamma_setting_31},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_32), green_gamma_setting_32},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_33), green_gamma_setting_33},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_34), green_gamma_setting_34},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(green_gamma_setting_35), green_gamma_setting_35},

	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_00), blue_gamma_setting_00},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_01), blue_gamma_setting_01},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_02), blue_gamma_setting_02},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_03), blue_gamma_setting_03},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_04), blue_gamma_setting_04},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_05), blue_gamma_setting_05},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_06), blue_gamma_setting_06},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_07), blue_gamma_setting_07},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_08), blue_gamma_setting_08},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_09), blue_gamma_setting_09},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_10), blue_gamma_setting_10},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_11), blue_gamma_setting_11},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_12), blue_gamma_setting_12},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_13), blue_gamma_setting_13},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_14), blue_gamma_setting_14},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_15), blue_gamma_setting_15},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_16), blue_gamma_setting_16},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_17), blue_gamma_setting_17},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_18), blue_gamma_setting_18},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_19), blue_gamma_setting_19},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_20), blue_gamma_setting_20},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_21), blue_gamma_setting_21},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_22), blue_gamma_setting_22},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_23), blue_gamma_setting_23},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_24), blue_gamma_setting_24},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_25), blue_gamma_setting_25},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_26), blue_gamma_setting_26},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_27), blue_gamma_setting_27},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_28), blue_gamma_setting_28},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_29), blue_gamma_setting_29},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_30), blue_gamma_setting_30},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_31), blue_gamma_setting_31},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_32), blue_gamma_setting_32},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_33), blue_gamma_setting_33},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_34), blue_gamma_setting_34},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(blue_gamma_setting_35), blue_gamma_setting_35},

	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(cmd2_p0_lock), cmd2_p0_lock},
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

static const struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db[] = {
	/* 540*960, RGB888, 2 Lane 60 fps command mode */
	{
		/* regulator */
		{0x03, 0x0a, 0x04, 0x00, 0x20},
		/* timing */
		{0x73, 0x19, 0x11, 0x00, 0x3c, 0x46, 0x14, 0x1c,
		 0x1c, 0x03, 0x04, 0xa0},
		/* phy ctrl */
		{0x5f, 0x00, 0x00, 0x10},
		/* strength */
		{0xff, 0x00, 0x06, 0x00},
		/* pll control */
		{0x00, 0x7d, 0x31, 0xd9, 0x00, 0x50, 0x48, 0x63,
		 0x41, 0x0f, 0x03,
		 0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
	},
};

static struct msm_panel_info pinfo;

static struct msm_panel_info *get_panel_info(void)
{
	/* should fix porch, pulse widht and so on */
	pinfo.xres = 540;
	pinfo.yres = 960;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 4;
	pinfo.lcdc.h_front_porch = 18;
	pinfo.lcdc.h_pulse_width = 2;
	pinfo.lcdc.v_back_porch = 11;
	pinfo.lcdc.v_front_porch = 4;
	pinfo.lcdc.v_pulse_width = 2;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 397000000;
	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.lcd.refx100 = 6000; /* adjust refx100 to prevent tearing */
	pinfo.lcd.v_back_porch = 11;
	pinfo.lcd.v_front_porch = 4;
	pinfo.lcd.v_pulse_width = 2;

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.dlane_swap = 0x00;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.t_clk_post = 0x04;
	pinfo.mipi.t_clk_pre = 0x1a;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.te_sel = 1; /* TE from vsycn gpio */
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.dsi_phy_db =
		(struct mipi_dsi_phy_ctrl *)dsi_cmd_mode_phy_db;

	return &pinfo;
}

static struct dsi_controller dsi_cmd_controller_panel = {
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
	0x12, 0x57, 0x77, 0x75, 0x1a, 0x01, 0xff
};

static char ddb_val_1b[] = {
	0x12, 0x57, 0x77, 0x75, 0x1b, 0x01, 0xff
};

static char ddb_val[] = {
	0x12, 0x57, 0x77, 0x75, 0xff, 0x01, 0xff
};

static char default_ddb_val[] = {
	0x12, 0x57, 0x77, 0x75
};

const struct panel_id sharp_ls038k3sx01_panel_id_1a = {
	.name = "mipi_cmd_sharp_qhd_ls038k3sx01_id_1a",
	.pctrl = &dsi_cmd_controller_panel,
	.id = ddb_val_1a,
	.id_num = ARRAY_SIZE(ddb_val_1a),
	.width = 46,
	.height = 81,
};

const struct panel_id sharp_ls038k3sx01_panel_id_1b = {
	.name = "mipi_cmd_sharp_qhd_ls038k3sx01_id_1b",
	.pctrl = &dsi_cmd_controller_panel,
	.id = ddb_val_1b,
	.id_num = ARRAY_SIZE(ddb_val_1b),
	.width = 46,
	.height = 81,
};

const struct panel_id sharp_ls038k3sx01_panel_id = {
	.name = "mipi_cmd_sharp_qhd_ls038k3sx01",
	.pctrl = &dsi_cmd_controller_panel,
	.id = ddb_val,
	.id_num = ARRAY_SIZE(ddb_val),
	.width = 46,
	.height = 81,
};

const struct panel_id sharp_ls038k3sx01_panel_default = {
	.name = "mipi_sharp_panel",
	.pctrl = &dsi_cmd_controller_panel,
	.id = default_ddb_val,
	.id_num = ARRAY_SIZE(default_ddb_val),
	.width = 46,
	.height = 81,
};
