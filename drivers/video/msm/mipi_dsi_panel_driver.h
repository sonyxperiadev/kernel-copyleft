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
};

#ifdef CONFIG_DEBUG_FS
extern void mipi_dsi_panel_create_debugfs(struct platform_device *pdev);
extern void mipi_dsi_panel_remove_debugfs(struct platform_device *pdev);
#endif

#endif /* MIPI_DSI_PANEL_DRIVER_H */
