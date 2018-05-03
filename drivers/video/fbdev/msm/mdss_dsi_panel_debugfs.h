/*
 * drivers/video/fbdev/msm/mdss_dsi_panel_debugfs.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2016 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef MDSS_DSI_PANEL_DEBUGFS_H
#define MDSS_DSI_PANEL_DEBUGFS_H

#include "mdss_dsi.h"

struct blackscreen_det {
	u16 threshold;
	u16 cnt_timeout;
	u16 cnt_crash;
	ktime_t timestamp;
	int done;
	wait_queue_head_t wait_queue;
};

struct first_frame_flushed_det {
	ktime_t timestamp;
	int done;
	wait_queue_head_t wait_queue;
};

int mdss_dsi_panel_create_fs(struct mdss_dsi_ctrl_pdata *ctrl_pdata);
int mdss_dsi_create_debugfs(struct msm_fb_data_type *mfd);
#endif /* MDSS_DSI_PANEL_DEBUGFS_H */
