/* drivers/video/msm/mipi_dsi_panel.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 * Author: Yosuke Hatanaka <yosuke.hatanaka@sonyericsson.com>
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#ifndef MIPI_DSI_PANEL_H
#define MIPI_DSI_PANEL_H

#include <linux/types.h>
#include "mipi_dsi.h"

#define ONE_FRAME_TRANSMIT_WAIT_MS 20

/* These can be hardcoded since they do not vary from panel to panel */
#define NVRW_NUM_E6_PARAM	6
#define NVRW_NUM_E6_VALID_PARAM	4
#define NVRW_NUM_BF_PARAM	5
#define NVRW_NUM_DE_VALID_PARAM	8  /* We can only read 8 bytes */

struct dsi_controller {
	struct msm_panel_info *(*get_panel_info) (void);
	struct dsi_cmd_desc *display_init_cmds;
	struct dsi_cmd_desc *display_on_eco_cmds;
	struct dsi_cmd_desc *display_on_cmds;
	struct dsi_cmd_desc *display_off_cmds;
	struct dsi_cmd_desc *read_id_cmds;
	struct dsi_cmd_desc *eco_mode_gamma_cmds;
	struct dsi_cmd_desc *normal_gamma_cmds;
	int display_init_cmds_size;
	int display_on_eco_cmds_size;
	int display_on_cmds_size;
	int display_off_cmds_size;
	int eco_mode_gamma_cmds_size;
	int normal_gamma_cmds_size;
};

struct dsi_nvm_rewrite_ctl {
	struct dsi_cmd_desc *nvm_erase_all;
	struct dsi_cmd_desc *nvm_read_e4;
	struct dsi_cmd_desc *nvm_erase_finish;
	struct dsi_cmd_desc *nvm_read_e1;
	struct dsi_cmd_desc *nvm_write_trim;
	struct dsi_cmd_desc *nvm_write_user;
	struct dsi_cmd_desc *nvm_enter_sleep;
	struct dsi_cmd_desc *nvm_exit_sleep;
	struct dsi_cmd_desc *nvm_mcap_unlock;
	struct dsi_cmd_desc *nvm_mcap_lock;
	struct dsi_cmd_desc *nvm_read_bf;
	struct dsi_cmd_desc *nvm_read_e6;
	struct dsi_cmd_desc *nvm_read_e7;
	struct dsi_cmd_desc *nvm_read_de;
	char *nvm_bf;
	char *nvm_e6;
	char *nvm_e7;
	char *nvm_de;
	int nvm_e7_nbr_params;
	int nvm_de_nbr_params;
	int nvm_erase_all_size;
	int nvm_read_e4_size;
	int nvm_erase_finish_size;
	int nvm_read_e1_size;
	int nvm_write_trim_size;
	int nvm_write_user_size;
	int nvm_enter_sleep_size;
	int nvm_exit_sleep_size;
	int nvm_mcap_unlock_size;
	int nvm_mcap_lock_size;
	const bool nvm_restore_e7;
	const bool nvm_restore_de;
	int nvm_de_valid_param_start;
	int nvm_de_valid_param_end;
};

struct mipi_dsi_lane_cfg {
	uint32_t	ln_cfg[4][3];
	uint32_t	ln_dpath[4];
	uint32_t	ln_str[4][2];
	uint32_t	lnck_cfg[3];
	uint32_t	lnck_dpath;
	uint32_t	lnck_str[2];
};

struct panel_id {
	const char			*name;
	struct dsi_controller		*pctrl;
	struct dsi_nvm_rewrite_ctl	*pnvrw_ctl;
	const bool			use_if_nv_fail;
	const u32			width;	/* in mm */
	const u32			height;	/* in mm */
	const char			*id;
	const int			id_num;
	const struct mipi_dsi_lane_cfg	*plncfg;
	const bool			esd_failed_check;
};

struct mipi_dsi_data {
	struct dsi_buf tx_buf;
	struct dsi_buf rx_buf;
	struct msm_fb_panel_data panel_data;
	const struct panel_id *panel;
	const struct panel_id **default_panels;
	const struct panel_id **panels;
	int (*lcd_power)(int on);
	int (*lcd_reset)(int on);
	int (*vreg_power)(int on);
	int (*eco_mode_switch)(struct msm_fb_data_type *mfd);
#ifdef CONFIG_DEBUG_FS
	struct dentry *panel_driver_ic_dir;
#endif
	int panel_detecting;
	int eco_mode_on;

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	bool	panel_nvm_ok;
	bool	panel_nvm_backup_ok;
	bool	enable_resume_check;
	int	(*override_nvm_data)(struct msm_fb_data_type *mfd,
					const char *buf, int count);
	ssize_t	(*get_nvm_backup)(struct mipi_dsi_data *dsi_data, char *buf);
	bool	(*is_nvm_ok)(struct msm_fb_data_type *mfd);
	void	(*backup_nvm_to_ram)(struct msm_fb_data_type *mfd);
	int	(*nvm_erase_all)(struct msm_fb_data_type *mfd);
	int	(*nvm_write_trim_area)(struct msm_fb_data_type *mfd);
	int	(*nvm_write_user_area)(struct msm_fb_data_type *mfd);
#endif
	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;
	void (*esd_check)(struct mipi_dsi_data *dsi_data);
	bool esd_check_enable;
};

void mipi_dsi_set_default_panel(struct mipi_dsi_data *dsi_data);
struct msm_panel_info *mipi_dsi_detect_panel(
	struct msm_fb_data_type *mfd);
int __devinit mipi_dsi_need_detect_panel(
	const struct panel_id **panels);
int mipi_dsi_update_panel(struct platform_device *pdev);
int mipi_dsi_eco_mode_switch(struct msm_fb_data_type *mfd);
void mipi_dsi_update_lane_cfg(const struct mipi_dsi_lane_cfg *plncfg);

int eco_mode_sysfs_register(struct device *dev);

#if defined(CONFIG_FB_MSM_RECOVER_PANEL) || defined(CONFIG_DEBUG_FS)
int prepare_for_reg_access(struct msm_fb_data_type *mfd);
void post_reg_access(struct msm_fb_data_type *mfd);
#endif
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
int create_sysfs_interfaces(struct device *dev);
void remove_sysfs_interfaces(struct device *dev);
#endif

#ifdef CONFIG_DEBUG_FS
extern void mipi_dsi_panel_create_debugfs(struct platform_device *pdev,
					  const char *sub_name);
extern void mipi_dsi_panel_remove_debugfs(struct platform_device *pdev);
#endif

#endif /* MIPI_DSI_PANEL_H */
