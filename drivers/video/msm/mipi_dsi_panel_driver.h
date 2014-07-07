/* drivers/video/msm/mipi_dsi_panel_driver.h
 *
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Author: Yosuke Hatanaka <yosuke.hatanaka@sonyericsson.com>
 * Author: Johan Olson <johan.olson@sonymobile.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#ifndef MIPI_DSI_PANEL_DRIVER_H
#define MIPI_DSI_PANEL_DRIVER_H

#include <linux/types.h>
#include "mipi_dsi.h"
#include <video/mipi_dsi_panel.h>

#define ONE_FRAME_TRANSMIT_WAIT_MS 20

enum power_state {
	PANEL_OFF,
	DEBUGFS_POWER_OFF,
	DEBUGFS_POWER_ON,
	PANEL_INITIALIZED,
	PANEL_SLEEP_OUT,
	PANEL_ON
};

struct mipi_dsi_data {
	struct dsi_buf tx_buf;
	struct dsi_buf rx_buf;
	struct msm_fb_panel_data panel_data;
	const struct panel *panel;
	const struct panel **default_panels;
	const struct panel **panels;
	int (*lcd_power)(bool on);
	int (*lcd_reset)(bool on);
	int (*vreg_power)(int on);
#ifdef CONFIG_DEBUG_FS
	struct dentry *dir;
#endif
	bool panel_detected;
	enum power_state panel_state;
	struct mutex lock;
	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;
	void (*esd_check)(struct mipi_dsi_data *dsi_data);
	bool esd_check_enable;
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	int nvrw_ic_vendor;
	bool nvrw_panel_detective;
	int nvrw_result;
	int nvrw_retry_cnt;

	int (*override_nvm_data)(struct msm_fb_data_type *mfd,
					const char *buf, int count);
	int (*seq_nvm_read)(struct msm_fb_data_type *mfd, char *buf);
	int (*seq_nvm_erase)(struct msm_fb_data_type *mfd);
	int (*seq_nvm_rsp_write)(struct msm_fb_data_type *mfd);
	int (*seq_nvm_user_write)(struct msm_fb_data_type *mfd);
#endif
};

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
int drv_ic_sysfs_register(struct device *dev);
void drv_ic_sysfs_unregister(struct device *dev);
#endif

#ifdef CONFIG_DEBUG_FS
extern void mipi_dsi_panel_create_debugfs(struct platform_device *pdev);
extern void mipi_dsi_panel_remove_debugfs(struct platform_device *pdev);
#endif

#endif /* MIPI_DSI_PANEL_DRIVER_H */
