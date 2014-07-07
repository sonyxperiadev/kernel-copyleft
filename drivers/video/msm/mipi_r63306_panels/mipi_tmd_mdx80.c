/* drivers/video/msm/mipi_r63306_panels/mipi_tmd_mdx80.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Yosuke Hatanaka <yosuke.hatanaka@sonyericsson.com>
 *         Johan Olson <johan.olson@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "../msm_fb.h"
#include "../mipi_dsi.h"
#include "../mipi_dsi_panel.h"


static char mcap_unlock_reg[] = {
	0xB0, 0x00
};

static char mcap_lock_reg[] = {
	0xB0, 0x03
};

/* Display ON Sequence */
static char cabc_on_off[] = {
	0xBB, 0x0B
};

static char pwm_dimming_control[] = {
	0xBC, 0x00
};

static char cabc_user_param[] = {
	0xBE, 0xFF, 0x0F, 0x00, 0x0C, 0x10, 0x02,
	0x00, 0x5D, 0x00, 0x00, 0x80, 0x32
};

static char pwm_setting_1[] = {
	0xB7, 0x18, 0x00, 0x18, 0x18, 0x0C, 0x14,
	0xAC, 0x14, 0x6C, 0x14, 0x0C, 0x14, 0x00,
	0x10, 0x00
};

static char pwm_setting_2[] = {
	0xB8, 0xF8, 0xDA, 0x6D, 0xFB, 0xFF, 0xFF,
	0xCF, 0x1F, 0x37, 0x5A, 0x87, 0xBE, 0xFF
};

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
};

static struct dsi_cmd_desc tmd_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(mcap_unlock_reg), mcap_unlock_reg},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(cabc_on_off), cabc_on_off},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(pwm_dimming_control), pwm_dimming_control},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(pwm_setting_1), pwm_setting_1},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(pwm_setting_2), pwm_setting_2},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(cabc_user_param), cabc_user_param},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mcap_lock_reg), mcap_lock_reg},
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc tmd_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc read_ddb_start_cmds[] = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_ddb_start), read_ddb_start},
};

#ifdef CONFIG_FB_MSM_RECOVER_PANEL

static char nvm_e4_00_00_00_f0_ff_reg[] = {
	0xE4, 0x00, 0x00, 0x00, 0xF0, 0xFF
};
static char nvm_e7_reg[] = {
	0xE7, 0x0F, 0x0C, 0x0B, 0x34 /* default data according to SoMC */
};
static char nvm_bf_reg[] = {
	0xBF, 0x01, 0x22, 0x33, 0x06, 0xB0
};
static char nvm_b2_reg[] = {
	0xB2, 0x00
};
static char nvm_b4_reg[] = {
	0xB4, 0x02
};
static char nvm_b6_reg[] = {
	0xB6, 0x51, 0xE3
};
static char nvm_c0_reg[] = {
	0xC0, 0x40, 0x02, 0x7F, 0xC8, 0x08
};
static char nvm_c1_reg[] = {
	0xC1, 0x00, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x08, 0x27,
	0x09, 0x00, 0x00, 0x00, 0x00
};
static char nvm_c2_reg[] = {
	0xC2, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00
};
static char nvm_c3_reg[] = {
	0xC3, 0x04
};
static char nvm_c4_reg[] = {
	0xC4, 0x4D, 0x83, 0x00
};
static char nvm_c6_reg[] = {
	0xC6, 0x12, 0x00, 0x08, 0x71, 0x00, 0x00, 0x00, 0x80, 0x00, 0x04
};
static char nvm_c7_reg[] = {
	0xC7, 0x22
};
static char nvm_c8_reg[] = {
	0xC8, 0x4C, 0x0C, 0x0C, 0x0C
};
static char nvm_c9_reg[] = {
	0xC9, 0x00, 0x06, 0x10, 0x2E, 0x36, 0x34, 0x3F, 0x44, 0x38, 0x32,
	0x31, 0x6A, 0x35
};
static char nvm_ca_reg[] = {
	0xCA, 0x0A, 0x15, 0x2E, 0x2D, 0x27, 0x1B, 0x20, 0x2B, 0x29, 0x31,
	0x4F, 0x79, 0x3F
};
static char nvm_cb_reg[] = {
	0xCB, 0x00, 0x11, 0x29, 0x41, 0x45, 0x3F, 0x46, 0x4A, 0x3D, 0x36,
	0x37, 0x2E, 0x39
};
static char nvm_cc_reg[] = {
	0xCC, 0x06, 0x11, 0x28, 0x29, 0x22, 0x15, 0x19, 0x20, 0x1A, 0x1E,
	0x36, 0x2E, 0x3F
};
static char nvm_cd_reg[] = {
	0xCD, 0x10, 0x1F, 0x59, 0x5D, 0x5A, 0x4E, 0x50, 0x51, 0x41, 0x39,
	0x3A, 0x30, 0x3F
};
static char nvm_ce_reg[] = {
	0xCE, 0x00, 0x0F, 0x25, 0x26, 0x1E, 0x0E, 0x0F, 0x11, 0x05, 0x02,
	0x06, 0x20, 0x2F
};
static char nvm_d0_reg[] = {
	0xD0, 0x6A, 0x64, 0x01, 0x18, 0xA1, 0x00, 0x21
};
static char nvm_d1_reg[] = {
	0xD1, 0x77, 0xD4, 0x5C, 0x21, 0x97, 0x06, 0x00, 0x00
};
static char nvm_d3_reg[] = {
	0xD3, 0x33
};
static char nvm_d5_reg[] = {
	0xD5, 0x0C, 0x0C
};
static char nvm_d7_reg[] = {
	0xD7, 0x22, 0x21, 0x55, 0x46, 0x44, 0x34, 0x02, 0x65, 0x04
};
static char nvm_d8_reg[] = {
	0xD8, 0x34, 0x64, 0x23, 0x25, 0x62, 0x32
};
static char nvm_d9_reg[] = {
	0xD9, 0xDF, 0xDD, 0x3F
};
static char nvm_da_reg[] = {
	0xDA, 0x01
};
static char nvm_db_reg[] = {
	0xDB, 0x46, 0x62, 0x42
};
static char nvm_de_reg[] = {
	0xDE, 0x10, 0x80, 0x11, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00
};
static char nvm_f3_reg[] = {
	0xF3, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00
};
static char nvm_e2_reg[] = {
	0xE2, 0x01
};
static char nvm_fd_reg[] = {
	0xFD, 0x04, 0x55, 0x53, 0x00, 0x70, 0xFF, 0x10, 0x73, 0x22, 0x22,
	0x22, 0x33, 0x00
};
static char nvm_e6_reg[] = {
	0xE6, 0x12, 0x57, 0x12, 0x35, 0x50, 0x01
};
static char nvm_fe_reg[] = {
	0xFE, 0x00, 0x00
};
static char nvm_e4_b9_47_reg[] = {
	0xE4, 0xB9, 0x47
};
static char nvm_e4_bd_reg[] = {
	0xE4, 0xBD
};
static char nvm_read_e4_reg[] = {
	0xE4
};
static char nvm_e4_00_00_00_00_00_reg[] = {
	0xE4, 0x00, 0x00, 0x00, 0x00, 0x00
};
static char nvm_read_e1_reg[] = {
	0xE1
};
static char nvm_e4_39_87_reg[] = {
	0xE4, 0x39, 0x87
};
static char nvm_e0_reg[] = {
	0xE0
};
static char nvm_e5_reg[] = {
	0xE5, 0x01
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
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mcap_lock_reg), mcap_lock_reg},
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
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e4_00_00_00_00_00_reg),
						nvm_e4_00_00_00_00_00_reg},
};
static struct dsi_cmd_desc nvm_read_e1_cmds[] = {
	{DTYPE_GEN_READ, 1, 0, 0, 0, sizeof(nvm_read_e1_reg), nvm_read_e1_reg},
};

static struct dsi_cmd_desc nvm_erase_all_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e4_00_00_00_f0_ff_reg),
						nvm_e4_00_00_00_f0_ff_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e7_reg), nvm_e7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_bf_reg), nvm_bf_reg},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(nvm_b2_reg), nvm_b2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_b4_reg), nvm_b4_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_b6_reg), nvm_b6_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c0_reg), nvm_c0_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c1_reg), nvm_c1_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c2_reg), nvm_c2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c3_reg), nvm_c3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c4_reg), nvm_c4_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c6_reg), nvm_c6_reg},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(nvm_c7_reg), nvm_c7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c8_reg), nvm_c8_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c9_reg), nvm_c9_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ca_reg), nvm_ca_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cb_reg), nvm_cb_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cc_reg), nvm_cc_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cd_reg), nvm_cd_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ce_reg), nvm_ce_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d0_reg), nvm_d0_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d1_reg), nvm_d1_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d3_reg), nvm_d3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d5_reg), nvm_d5_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d7_reg), nvm_d7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d8_reg), nvm_d8_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d9_reg), nvm_d9_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_da_reg), nvm_da_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_db_reg), nvm_db_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_de_reg), nvm_de_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e2_reg), nvm_e2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_f3_reg), nvm_f3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_fd_reg), nvm_fd_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e5_reg), nvm_e5_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e6_reg), nvm_e6_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_fe_reg), nvm_fe_reg},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(exit_sleep), exit_sleep},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e4_b9_47_reg),
							nvm_e4_b9_47_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1350, sizeof(nvm_e4_bd_reg),
							nvm_e4_bd_reg},
};

static struct dsi_cmd_desc nvm_write_trim_cmds[] = {
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e4_00_00_00_f0_ff_reg),
						nvm_e4_00_00_00_f0_ff_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e7_reg), nvm_e7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_bf_reg), nvm_bf_reg},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(nvm_b2_reg), nvm_b2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_b4_reg), nvm_b4_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_b6_reg), nvm_b6_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c0_reg), nvm_c0_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c1_reg), nvm_c1_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c2_reg), nvm_c2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c3_reg), nvm_c3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c4_reg), nvm_c4_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c6_reg), nvm_c6_reg},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(nvm_c7_reg), nvm_c7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c8_reg), nvm_c8_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c9_reg), nvm_c9_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ca_reg), nvm_ca_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cb_reg), nvm_cb_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cc_reg), nvm_cc_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cd_reg), nvm_cd_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ce_reg), nvm_ce_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d0_reg), nvm_d0_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d1_reg), nvm_d1_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d3_reg), nvm_d3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d5_reg), nvm_d5_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d7_reg), nvm_d7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d8_reg), nvm_d8_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d9_reg), nvm_d9_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_da_reg), nvm_da_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_db_reg), nvm_db_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_de_reg), nvm_de_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e2_reg), nvm_e2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_f3_reg), nvm_f3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_fd_reg), nvm_fd_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e5_reg), nvm_e5_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e6_reg), nvm_e6_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_fe_reg), nvm_fe_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e4_39_87_reg),
							nvm_e4_39_87_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1350, sizeof(nvm_e0_reg), nvm_e0_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e4_00_00_00_00_00_reg),
						nvm_e4_00_00_00_00_00_reg},
};

static struct dsi_cmd_desc nvm_write_user_cmds[] = {
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(nvm_b2_reg), nvm_b2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_b4_reg), nvm_b4_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_b6_reg), nvm_b6_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c0_reg), nvm_c0_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c1_reg), nvm_c1_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c2_reg), nvm_c2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c3_reg), nvm_c3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c4_reg), nvm_c4_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c6_reg), nvm_c6_reg},
	{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(nvm_c7_reg), nvm_c7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c8_reg), nvm_c8_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_c9_reg), nvm_c9_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ca_reg), nvm_ca_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cb_reg), nvm_cb_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cc_reg), nvm_cc_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_cd_reg), nvm_cd_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_ce_reg), nvm_ce_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d0_reg), nvm_d0_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d1_reg), nvm_d1_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d3_reg), nvm_d3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d5_reg), nvm_d5_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d7_reg), nvm_d7_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d8_reg), nvm_d8_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_d9_reg), nvm_d9_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_da_reg), nvm_da_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_db_reg), nvm_db_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_de_reg), nvm_de_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e2_reg), nvm_e2_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_f3_reg), nvm_f3_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_fd_reg), nvm_fd_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e5_reg), nvm_e5_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_e6_reg), nvm_e6_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(nvm_fe_reg), nvm_fe_reg},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 1300, sizeof(nvm_e0_reg), nvm_e0_reg},
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
	.nvm_read_bf		= nvm_read_bf_cmds,
	.nvm_read_e6		= nvm_read_e6_cmds,
	.nvm_read_e7		= nvm_read_e7_cmds,
	.nvm_read_de		= nvm_read_de_cmds,
	.nvm_bf			= nvm_bf_reg,
	.nvm_e6			= nvm_e6_reg,
	.nvm_e7			= nvm_e7_reg,
	.nvm_de			= nvm_de_reg,
	.nvm_e7_nbr_params	= ARRAY_SIZE(nvm_e7_reg) - 1,
	.nvm_de_nbr_params	= ARRAY_SIZE(nvm_de_reg) - 1,
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
	.nvm_restore_de		= false,
};

#endif /* CONFIG_FB_MSM_RECOVER_PANEL */

static const struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db[] = {
	/* 720*1280, RGB888, 4 Lane 60 fps video mode */
	{
		/* regulator */
		{0x03, 0x0a, 0x04, 0x00, 0x20},
		/* timing */
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

static struct msm_panel_info pinfo;

static struct msm_panel_info *get_panel_info(void)
{
	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 45;
	pinfo.lcdc.h_front_porch = 156;
	pinfo.lcdc.h_pulse_width = 3;
	pinfo.lcdc.v_back_porch = 3;
	pinfo.lcdc.v_front_porch = 9;
	pinfo.lcdc.v_pulse_width = 4;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
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
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.r_sel = 0;
	pinfo.mipi.g_sel = 0;
	pinfo.mipi.b_sel = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x04;
	pinfo.mipi.t_clk_pre = 0x1b;
	pinfo.mipi.esc_byte_ratio = 2;
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
	.display_init_cmds = tmd_display_init_cmds,
	.display_on_cmds = tmd_display_on_cmds,
	.display_off_cmds = tmd_display_off_cmds,
	.read_id_cmds = read_ddb_start_cmds,
	.display_init_cmds_size = ARRAY_SIZE(tmd_display_init_cmds),
	.display_on_cmds_size = ARRAY_SIZE(tmd_display_on_cmds),
	.display_off_cmds_size = ARRAY_SIZE(tmd_display_off_cmds),
};

static char ddb_val_1a[] = {
	0x12, 0x57, 0x12, 0x35, 0x1a, 0x01, 0x00, 0xff
};

static char ddb_val[] = {
	0x12, 0x57, 0x12, 0x35, 0xff, 0x01, 0x00, 0xff
};

static char ddb_val_att[] = {
	0x12, 0x61, 0x73, 0x24, 0xff, 0x01, 0x00, 0xff
};

static char default_ddb_val[] = {
	0x12, 0x57, 0x12, 0x35
};

const struct panel_id tmd_mdx80_panel_id_1a = {
	.name = "mipi_video_tmd_wxga_mdx80_id_1a",
	.pctrl = &dsi_video_controller_panel,
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	.pnvrw_ctl = &dsi_nvrw_ctl,
#endif
	.id = ddb_val_1a,
	.id_num = ARRAY_SIZE(ddb_val_1a),
	.width = 57,
	.height = 101,
};

const struct panel_id tmd_mdx80_panel_id = {
	.name = "mipi_video_tmd_wxga_mdx80",
	.pctrl = &dsi_video_controller_panel,
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	.pnvrw_ctl = &dsi_nvrw_ctl,
	.use_if_nv_fail = true,
#endif
	.id = ddb_val,
	.id_num = ARRAY_SIZE(ddb_val),
	.width = 57,
	.height = 101,
};

const struct panel_id tmd_mdx80_panel_id_att = {
	.name = "mipi_video_tmd_wxga_mdx80_att",
	.pctrl = &dsi_video_controller_panel,
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	.pnvrw_ctl = &dsi_nvrw_ctl,
#endif
	.id = ddb_val_att,
	.id_num = ARRAY_SIZE(ddb_val_att),
	.width = 57,
	.height = 101,
};

const struct panel_id tmd_mdx80_panel_default = {
	.name = "mipi_tmd_panel",
	.pctrl = &dsi_video_controller_panel,
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	.pnvrw_ctl = &dsi_nvrw_ctl,
#endif
	.id = default_ddb_val,
	.id_num = ARRAY_SIZE(default_ddb_val),
	.width = 57,
	.height = 101,
};
