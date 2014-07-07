/* include/video/mipi_dsi_panel.h
 *
 * Copyright (c) 2012-2013 Sony Mobile Communications AB.
 *
 * Author: Johan Olson <johan.olson@sonymobile.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef MIPI_DSI_PANEL_H
#define MIPI_DSI_PANEL_H

#include <linux/types.h>

#define MIPI_DSI_PANEL_NAME "mipi_dsi_panel"

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
enum {
	NVRW_DRV_NONE,
	NVRW_DRV_RENESAS,
	NVRW_DRV_SAMSUNG,
	NVRW_DRV_NOVATEK
};
#define NVRW_NUM_E7_PARAM	4
#define NVRW_NUM_DE_PARAM	12
#endif

enum panel_cmd_type {
	CMD_END,
	CMD_WAIT_MS,
	CMD_DSI,
	CMD_RESET,
	CMD_PLATFORM
};

struct dsi_cmd_payload {
	struct dsi_cmd_desc *dsi;
	int cnt;
};

struct panel_cmd {
	enum panel_cmd_type type;
	union {
		char data;
		struct dsi_cmd_payload dsi_payload;
	} payload;
};

struct dsi_controller {
	struct msm_panel_info *(*get_panel_info)(void);
	const struct panel_cmd *display_init;
	const struct panel_cmd *display_on;
	const struct panel_cmd *display_off;
	const struct panel_cmd *read_id;
};

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
struct dsi_nvm_rewrite_ctl {
	struct dsi_cmd_desc *nvm_disp_off;
	struct dsi_cmd_desc *nvm_mcap;
	struct dsi_cmd_desc *nvm_mcap_lock;
	struct dsi_cmd_desc *nvm_open;
	struct dsi_cmd_desc *nvm_close;
	struct dsi_cmd_desc *nvm_status;
	struct dsi_cmd_desc *nvm_erase;
	struct dsi_cmd_desc *nvm_erase_res;
	struct dsi_cmd_desc *nvm_read;
	struct dsi_cmd_desc *nvm_write_rsp;
	struct dsi_cmd_desc *nvm_flash_rsp;
	struct dsi_cmd_desc *nvm_write_user;
	struct dsi_cmd_desc *nvm_flash_user;

	int nvm_disp_off_size;
	int nvm_mcap_size;
	int nvm_mcap_lock_size;
	int nvm_open_size;
	int nvm_close_size;
	int nvm_erase_size;
	int nvm_read_size;
	int nvm_write_rsp_size;
	int nvm_flash_rsp_size;
	int nvm_write_user_size;
	int nvm_flash_user_size;
};
#endif

struct panel {
	const char			*name;
	const char			*panel_id;
	const char			*panel_rev;
	struct dsi_controller		*pctrl;
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	struct dsi_nvm_rewrite_ctl	*pnvrw_ctl;
#endif
	const u32			width;	/* in mm */
	const u32			height;	/* in mm */
	const char			*id;
	const int			id_num;
	const bool			send_video_data_before_display_on;
	const bool			esd_failed_check;
};

struct panel_platform_data {
	/* TODO: add data regarding gpio pin, level, regulators, voltages */
	int (*platform_power)(bool enable);
	int (*platform_reset)(bool high);
	const struct panel **panels;
	const struct panel **default_panels;
	int (*vreg_power)(int on);
	int ic_vendor;
};

#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01
extern const struct panel sharp_ls046k3sy01_panel_id_1a;
extern const struct panel sharp_ls046k3sy01_panel_id;
extern const struct panel sharp_ls046k3sy01_panel_default;
#endif /*CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01*/
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04
extern const struct panel sharp_ls043k3sx04_panel_default;
extern const struct panel sharp_ls043k3sx04_panel_id_1a;
extern const struct panel sharp_ls043k3sx04_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01
extern const struct panel sharp_ls043k3sx01_panel_default_old;
extern const struct panel sharp_ls043k3sx01_panel_default;
extern const struct panel sharp_ls043k3sx01_panel_id_1a;
extern const struct panel sharp_ls043k3sx01_panel_id;
extern const struct panel sharp_ls043k3sx01_panel_id_old;
#endif /*CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01*/
#ifdef CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01
extern const struct panel auo_h455tvn01_panel_default;
extern const struct panel auo_h455tvn01_panel_id_1a;
extern const struct panel auo_h455tvn01_panel_id;
#endif /* CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01 */
#endif /* MIPI_DSI_PANEL_H */
