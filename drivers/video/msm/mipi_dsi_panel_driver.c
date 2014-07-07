/* drivers/video/msm/mipi_dsi_panel_driver.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Johan Olson <johan.olson@sonymobile.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

/* #define DEBUG */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>

#include <video/mipi_dsi_panel.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_panel_driver.h"

#ifdef CONFIG_FB_MSM_MDP303
#define DSI_VIDEO_BASE	0xF0000
#else
#define DSI_VIDEO_BASE	0xE0000
#endif

#define PANEL_ESD_CHECK_PERIOD		msecs_to_jiffies(1000)

static struct msm_fb_data_type *mipi_dsi_panel_mfd;

static bool is_read_cmd(int dtype)
{
	return (dtype == DTYPE_DCS_READ || dtype == DTYPE_GEN_READ ||
		dtype == DTYPE_GEN_READ1 || dtype == DTYPE_GEN_READ2);
}

static int panel_execute_cmd(struct msm_fb_data_type *mfd,
			     struct mipi_dsi_data *dsi_data,
			     const struct panel_cmd *pcmd,
			     int nbr_bytes_to_read)
{
	struct device *dev = &mfd->panel_pdev->dev;
	int n;
	int ret = 0;

	if (!pcmd) {
		dev_err(dev, "%s: no command\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	for (n = 0; ; ++n)
		switch (pcmd[n].type) {
		case CMD_END:
			dev_dbg(dev, "CMD_END\n");
			goto exit;
			break;
		case CMD_WAIT_MS:
			dev_dbg(dev, "%s: CMD_WAIT_MS = %d ms\n",
						__func__, pcmd[n].payload.data);
			msleep(pcmd[n].payload.data);
			break;
		case CMD_DSI:
			dev_dbg(dev, "%s: CMD_DSI\n", __func__);
			/* Sufficient to check if first cmd is a read, then
			 * it is impled that all are of the same type */
			if (is_read_cmd(
				pcmd[n].payload.dsi_payload.dsi[0].dtype)) {

				dev_dbg(dev, "%s: CMD_DSI READ\n", __func__);
				mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf,
					&dsi_data->rx_buf,
					pcmd[n].payload.dsi_payload.dsi,
					nbr_bytes_to_read);
			} else {
				mipi_dsi_cmds_tx(&dsi_data->tx_buf,
					pcmd[n].payload.dsi_payload.dsi,
					pcmd[n].payload.dsi_payload.cnt);
			}
			break;
		case CMD_RESET:
			dev_dbg(dev, "%s: CMD_RESET lvl=%d\n", __func__,
							pcmd[n].payload.data);
			if (dsi_data->lcd_reset)
				ret = dsi_data->lcd_reset(pcmd[n].payload.data);
			if (ret)
				goto exit;
			break;
		case CMD_PLATFORM:
			dev_dbg(dev, "%s: CMD_PLATFORM enable=%d\n", __func__,
							pcmd[n].payload.data);
			if (dsi_data->lcd_power)
				ret = dsi_data->lcd_power(pcmd[n].payload.data);
			if (ret)
				goto exit;
			break;
		default:
			dev_err(dev, "%s: Unknown command type!\n",
								__func__);
		}
exit:
	return ret;
}

static int mipi_dsi_set_default_panel(struct mipi_dsi_data *dsi_data)
{
	if (dsi_data->default_panels[0] != NULL)
		dsi_data->panel = dsi_data->default_panels[0];
	else if (dsi_data->panels[0] != NULL)
		dsi_data->panel = dsi_data->panels[0];
	else
		return -ENODEV;

	pr_info("%s: default panel: %s\n", __func__, dsi_data->panel->name);
	dsi_data->panel_data.panel_info =
		*dsi_data->panel->pctrl->get_panel_info();
	dsi_data->panel_data.panel_info.width =
		dsi_data->panel->width;
	dsi_data->panel_data.panel_info.height =
		dsi_data->panel->height;

	return 0;
}

static int panel_id_reg_check(struct msm_fb_data_type *mfd,
			      struct mipi_dsi_data *dsi_data,
			      const struct panel *panel)
{
	struct device *dev = &mfd->panel_pdev->dev;
	int i;
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__);

	mutex_lock(&mfd->dma->ov_mutex);
	ret = panel_execute_cmd(mfd, dsi_data, panel->pctrl->read_id,
								panel->id_num);
	mutex_unlock(&mfd->dma->ov_mutex);
	if (ret) {
		dev_err(dev, "%s: read id failed\n", __func__);
		goto exit;
	}

	for (i = 0; i < panel->id_num; i++) {
		if ((i >= dsi_data->rx_buf.len) ||
			((dsi_data->rx_buf.data[i] != panel->id[i]) &&
				(panel->id[i] != 0xff))) {
			ret = -ENODEV;
			goto exit;
		}
		dev_dbg(dev, "panel %s, id[%d]: 0x%x\n",
			panel->name, i, dsi_data->rx_buf.data[i]);
	}
exit:
	return ret;
}

static void mipi_dsi_clk_toggle(struct msm_fb_data_type *mfd)
{
	mutex_lock(&mfd->dma->ov_mutex);

	if (mfd->panel_info.mipi.mode == DSI_VIDEO_MODE) {
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
		mipi_dsi_controller_cfg(0);
		mipi_dsi_op_mode_config(DSI_CMD_MODE);

		mipi_dsi_op_mode_config(DSI_VIDEO_MODE);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		mipi_dsi_sw_reset();
		mipi_dsi_controller_cfg(1);
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 1);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	}

	mutex_unlock(&mfd->dma->ov_mutex);
}

static void mipi_dsi_panel_esd_failed_check(struct mipi_dsi_data *dsi_data)
{
	struct device *dev = &mipi_dsi_panel_mfd->panel_pdev->dev;

	dev_dbg(dev, "%s: enter...", __func__);

	mutex_lock(&mipi_dsi_panel_mfd->power_lock);
	/*if the panel was power off, it's no need to check ESD failed*/
	if (!mipi_dsi_panel_mfd->panel_power_on)
		goto unlock_exit;

	/*ESD Failed check*/
	mipi_dsi_clk_toggle(mipi_dsi_panel_mfd);

unlock_exit:
	mutex_unlock(&mipi_dsi_panel_mfd->power_lock);
}

static void panel_esd_start_check(struct mipi_dsi_data *dsi_data)
{
	queue_delayed_work(dsi_data->esd_wq, &dsi_data->esd_work,
			   PANEL_ESD_CHECK_PERIOD);
}

static void panel_esd_check_work(struct work_struct *work)
{
	struct mipi_dsi_data *dsi_data = container_of(to_delayed_work(work),
						struct mipi_dsi_data, esd_work);

	mutex_lock(&dsi_data->lock);
	if (!dsi_data->esd_check_enable) {
		mutex_unlock(&dsi_data->lock);
		return;
	}
	mutex_unlock(&dsi_data->lock);

	if (dsi_data->esd_check != NULL) {
		dsi_data->esd_check(dsi_data);
		panel_esd_start_check(dsi_data);
	}
}

static int __devinit mipi_dsi_need_detect_panel(const struct panel **panels)
{
	return panels[0] && panels[1];
}

static int panel_update_config(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct msm_panel_info *pinfo;
	struct mipi_panel_info *mipi;
	uint8 lanes = 0, bpp;
	uint32 h_period, v_period, dsi_pclk_rate;

	dev_dbg(&pdev->dev, "%s: pdev = %p, pdev->name = %s\n", __func__,
							pdev, pdev->name);

	mfd = platform_get_drvdata(pdev);
	pinfo = &mfd->panel_info;
	fbi = mfd->fbi;
	fbi->var.pixclock = pinfo->clk_rate;
	fbi->var.left_margin = pinfo->lcdc.h_back_porch;
	fbi->var.right_margin = pinfo->lcdc.h_front_porch;
	fbi->var.upper_margin = pinfo->lcdc.v_back_porch;
	fbi->var.lower_margin = pinfo->lcdc.v_front_porch;
	fbi->var.hsync_len = pinfo->lcdc.h_pulse_width;
	fbi->var.vsync_len = pinfo->lcdc.v_pulse_width;

	mipi = &pinfo->mipi;

	if (mipi->data_lane3)
		lanes += 1;
	if (mipi->data_lane2)
		lanes += 1;
	if (mipi->data_lane1)
		lanes += 1;
	if (mipi->data_lane0)
		lanes += 1;

	switch (mipi->dst_format) {
	case DSI_CMD_DST_FORMAT_RGB888:
	case DSI_VIDEO_DST_FORMAT_RGB888:
	case DSI_VIDEO_DST_FORMAT_RGB666_LOOSE:
		bpp = 3;
		break;
	case DSI_CMD_DST_FORMAT_RGB565:
	case DSI_VIDEO_DST_FORMAT_RGB565:
		bpp = 2;
		break;
	default:
		bpp = 3;	/* Default format set to RGB888 */
		break;
	}

	if (pinfo->type == MIPI_VIDEO_PANEL && !pinfo->clk_rate) {
		h_period = pinfo->lcdc.h_pulse_width
				+ pinfo->lcdc.h_back_porch
				+ pinfo->xres
				+ pinfo->lcdc.h_front_porch;

		v_period = pinfo->lcdc.v_pulse_width
				+ pinfo->lcdc.v_back_porch
				+ pinfo->yres
				+ pinfo->lcdc.v_front_porch;

		if (lanes > 0) {
			pinfo->clk_rate =
			((h_period * v_period * (mipi->frame_rate) * bpp * 8)
			   / lanes);
		} else {
			dev_err(&pdev->dev,
				"%s: forcing mipi_dsi lanes to 1\n", __func__);
			pinfo->clk_rate =
				(h_period * v_period
					 * (mipi->frame_rate) * bpp * 8);
		}
	}
	pll_divider_config.clk_rate = pinfo->clk_rate;

	mipi_dsi_clk_div_config(bpp, lanes, &dsi_pclk_rate);

	if ((dsi_pclk_rate < 3300000) || (dsi_pclk_rate > 103300000))
		dsi_pclk_rate = 35000000;
	mipi->dsi_pclk_rate = dsi_pclk_rate;

	return 0;
}

static int panel_sleep_out_display_off(struct msm_fb_data_type *mfd,
				       struct mipi_dsi_data *dsi_data)
{
	struct device *dev = &mfd->panel_pdev->dev;
	int ret = 0;

	dev_dbg(dev, "%s: Execute display_init\n", __func__);
	ret = panel_execute_cmd(mfd, dsi_data,
				dsi_data->panel->pctrl->display_init, 0);
	if (ret)
		dev_err(&mfd->panel_pdev->dev, "display_init failed\n");
	return ret;
}

static int panel_display_on(struct msm_fb_data_type *mfd,
			    struct mipi_dsi_data *dsi_data)
{
	struct device *dev = &mfd->panel_pdev->dev;
	int ret = 0;

	dev_dbg(dev, "%s: Execute display on\n", __func__);
	mipi_set_tx_power_mode(0);
	ret = panel_execute_cmd(mfd, dsi_data,
					dsi_data->panel->pctrl->display_on, 0);
	if (ret) {
		dev_err(dev, "%s: display_on failed\n", __func__);
		goto exit;
	}
	dev_info(dev, "%s: DISPLAY_ON sent\n", __func__);
exit:
	return ret;
}

static int panel_on(struct platform_device *pdev)
{
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_data_type *mfd;
	struct device *dev;
	bool skip_display_on = false;
	int ret = 0;

	mfd = platform_get_drvdata(pdev);
	if (!mfd) {
		ret = -ENODEV;
		goto exit;
	}

	if (mfd->key != MFD_KEY) {
		ret = -EINVAL;
		goto exit;
	}

	mipi_dsi_panel_mfd = mfd;

	dev = &mfd->panel_pdev->dev;
	dev_dbg(dev, "%s\n", __func__);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data) {
		ret = -ENODEV;
		goto exit;
	}

	mutex_lock(&dsi_data->lock);
	dev_dbg(dev, "%s: state = %d\n", __func__, dsi_data->panel_state);

	if (!dsi_data->panel_detected)
		goto unlock_and_exit;

	if (dsi_data->panel_state == PANEL_OFF ||
	    dsi_data->panel_state == DEBUGFS_POWER_OFF) {
		dev_dbg(dev, "%s: call power on\n", __func__);
		if (dsi_data->lcd_power) {
			ret = dsi_data->lcd_power(TRUE);
			if (ret)
				goto unlock_and_exit;
		}
		if (dsi_data->panel_state == DEBUGFS_POWER_OFF)
			dsi_data->panel_state = DEBUGFS_POWER_ON;
	}

	if (dsi_data->panel_state == DEBUGFS_POWER_ON) {
		dev_dbg(dev, "%s: debugfs state, don't exit sleep\n", __func__);
		goto unlock_and_exit;
	}

	if (dsi_data->panel_state == PANEL_OFF) {
		ret = panel_sleep_out_display_off(mfd, dsi_data);
		if (ret)
			goto unlock_and_exit;
		if (dsi_data->panel_data.controller_on_panel_on)
			skip_display_on = true;

		dsi_data->panel_state = PANEL_SLEEP_OUT;
	}

	/* Call display on depending on if it is setup to receive video */
	/* data data before display on or not */
	if (!skip_display_on && dsi_data->panel_state == PANEL_SLEEP_OUT) {
		ret = panel_display_on(mfd, dsi_data);
		if (ret)
			goto unlock_and_exit;
		dsi_data->panel_state = PANEL_ON;
	}

	/*Queue the ESD check work*/
	if (dsi_data->panel->esd_failed_check) {
		dsi_data->esd_check_enable = true;
		panel_esd_start_check(dsi_data);
	}

unlock_and_exit:
	mutex_unlock(&dsi_data->lock);
exit:
	return ret;
}

static int panel_off(struct platform_device *pdev)
{
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_data_type *mfd;
	struct device *dev;
	int ret = 0;

	mfd = platform_get_drvdata(pdev);
	if (!mfd) {
		ret = -ENODEV;
		goto exit;
	}
	if (mfd->key != MFD_KEY) {
		ret = -EINVAL;
		goto exit;
	}

	dev = &mfd->panel_pdev->dev;
	dev_dbg(dev, "%s\n", __func__);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data) {
		ret = -ENODEV;
		goto exit;
	}

	mutex_lock(&dsi_data->lock);

	/*Stop the ESD check when panel off*/
	if (dsi_data->panel->esd_failed_check)
		dsi_data->esd_check_enable = false;

	if (!dsi_data->panel_detected) {
		/* We rely on that we have detected a panel the first time
		   this function is called */
		dsi_data->panel_detected = true;
		goto unlock_and_exit;
	}

	if (dsi_data->panel_state == DEBUGFS_POWER_ON) {
		dev_dbg(dev, "%s: debugfs state, power off\n", __func__);
		goto power_off;
	}

	dev_dbg(dev, "%s: Execute display off\n", __func__);
	/* Set to OFF even if commands fail below */
	dsi_data->panel_state = PANEL_OFF;
	/* Turn off video timing generator and switch to command mode
	 * Otherwise command transmission during BLLP fails sometime
	 */
	if (mfd->panel_info.mipi.mode == DSI_VIDEO_MODE) {
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0);
		msleep(20);
		mipi_dsi_op_mode_config(DSI_CMD_MODE);
	}
	ret = panel_execute_cmd(mfd, dsi_data,
					dsi_data->panel->pctrl->display_off, 0);
	if (ret) {
		dev_err(dev, "%s: display_off failed\n", __func__);
		goto unlock_and_exit;
	}
	dev_info(dev, "%s: DISPLAY_OFF sent\n", __func__);
power_off:
	if (dsi_data->lcd_power)
		ret = dsi_data->lcd_power(FALSE);
unlock_and_exit:
	mutex_unlock(&dsi_data->lock);
exit:
	return ret;
}

static struct msm_panel_info *detect_panel(struct msm_fb_data_type *mfd)
{
	int i;
	int ret;
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_panel_data *pdata = NULL;
	struct device *dev;

	dev = &mfd->panel_pdev->dev;
	dev_dbg(dev, "%s\n", __func__);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data || !dsi_data->panels[0]) {
		dev_err(dev, "%s: Failed to detect panel, no panel data\n",
								__func__);
		return NULL;
	}

	mutex_lock(&dsi_data->lock);
	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	for (i = 0; dsi_data->panels[i]; i++) {
		ret = panel_id_reg_check(mfd, dsi_data, dsi_data->panels[i]);
		if (!ret)
			break;
	}

	if (dsi_data->panels[i]) {
		dsi_data->panel = dsi_data->panels[i];
		dev_info(dev, "%s: Found panel: %s\n", __func__,
							dsi_data->panel->name);
	} else {
		dev_warn(dev, "%s: Failed to detect panel!\n", __func__);
		mutex_unlock(&dsi_data->lock);
		return NULL;
	}

	if (dsi_data->panel->send_video_data_before_display_on) {
		dev_info(dev, "%s: send_video_data_before_display_on\n",
								__func__);
		dsi_data->panel_data.controller_on_panel_on = panel_on;
		dsi_data->panel_data.power_on_panel_at_pan = 0;
		/* Also need to update platform data since that is */
		/* used in msm_fb */
		pdata = mfd->pdev->dev.platform_data;
		pdata->controller_on_panel_on = panel_on;
		pdata->power_on_panel_at_pan = 0;
	}

	dsi_data->panel_data.panel_info =
				*dsi_data->panel->pctrl->get_panel_info();
	dsi_data->panel_data.panel_info.width = dsi_data->panel->width;
	dsi_data->panel_data.panel_info.height = dsi_data->panel->height;
	dsi_data->panel_data.panel_info.mipi.dsi_pclk_rate =
				mfd->panel_info.mipi.dsi_pclk_rate;

	mipi_dsi_op_mode_config(dsi_data->panel_data.panel_info.mipi.mode);

	mutex_unlock(&dsi_data->lock);
	return &dsi_data->panel_data.panel_info;
}

static ssize_t mipi_dsi_panel_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_data *dsi_data = dev_get_drvdata(dev);
	char const *id =  dsi_data->panel->panel_id ?
			dsi_data->panel->panel_id : "generic";
	return scnprintf(buf, PAGE_SIZE, "%s", id);
}

static ssize_t mipi_dsi_panel_rev_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_data *dsi_data = dev_get_drvdata(dev);
	char const *rev =  dsi_data->panel->panel_rev ?
			dsi_data->panel->panel_rev : "generic";
	return scnprintf(buf, PAGE_SIZE, "%s", rev);
}

static struct device_attribute panel_attributes[] = {
	__ATTR(panel_id, S_IRUGO, mipi_dsi_panel_id_show, NULL),
	__ATTR(panel_rev, S_IRUGO, mipi_dsi_panel_rev_show, NULL),
};

static int register_attributes(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(panel_attributes); i++)
		if (device_create_file(dev, panel_attributes + i))
			goto error;
	return 0;
error:
	dev_err(dev, "%s: Unable to create interface\n", __func__);
	for (--i; i >= 0 ; i--)
		device_remove_file(dev, panel_attributes + i);
	return -ENODEV;
}

static void remove_attributes(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(panel_attributes); i++)
		device_remove_file(dev, panel_attributes + i);
}

static int __devexit mipi_dsi_panel_remove(struct platform_device *pdev)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	remove_attributes(&pdev->dev);

#ifdef CONFIG_DEBUG_FS
	mipi_dsi_panel_remove_debugfs(pdev);
#endif
	if (dsi_data->panel->esd_failed_check)
		destroy_workqueue(dsi_data->esd_wq);
	platform_set_drvdata(pdev, NULL);
	mipi_dsi_buf_release(&dsi_data->tx_buf);
	mipi_dsi_buf_release(&dsi_data->rx_buf);
	kfree(dsi_data);
	return 0;
}

static int __devinit mipi_dsi_panel_probe(struct platform_device *pdev)
{
	int ret;
	struct panel_platform_data *platform_data;
	struct mipi_dsi_data *dsi_data;
	struct platform_device *mipi_dsi_pdev;

	pr_info("%s: pdev = %p, pdev->name = %s\n", __func__, pdev, pdev->name);

	platform_data = pdev->dev.platform_data;
	if (platform_data == NULL)
		return -EINVAL;

	dsi_data = kzalloc(sizeof(struct mipi_dsi_data), GFP_KERNEL);
	if (dsi_data == NULL)
		return -ENOMEM;

	dsi_data->default_panels = platform_data->default_panels;

	mutex_init(&dsi_data->lock);
	/* TODO: Create panel_record and remove unnecessary copying of */
	/* variables */
	dsi_data->panels = platform_data->panels;
	dsi_data->lcd_power = platform_data->platform_power;
	dsi_data->lcd_reset = platform_data->platform_reset;
	if (mipi_dsi_need_detect_panel(dsi_data->panels)) {
		dsi_data->panel_data.panel_detect = detect_panel;
		dsi_data->panel_data.update_panel = panel_update_config;
		dsi_data->panel_detected = false;
	} else {
		dev_info(&pdev->dev, "no need to detect panel\n");
	}

	ret = mipi_dsi_buf_alloc(&dsi_data->tx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(tx) failed!\n");
		ret = -ENOMEM;
		goto out_free;
	}

	ret = mipi_dsi_buf_alloc(&dsi_data->rx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(rx) failed!\n");
		ret = -ENOMEM;
		goto out_rx_release;
	}

	platform_set_drvdata(pdev, dsi_data);

	mipi_dsi_set_default_panel(dsi_data);

	dsi_data->panel_data.on = panel_on;
	dsi_data->panel_data.off = panel_off;

	if (dsi_data->panel->esd_failed_check) {
		dsi_data->esd_wq =
			create_singlethread_workqueue("panel_esd_check");
		if (dsi_data->esd_wq == NULL) {
			dev_err(&pdev->dev, "can't create ESD workqueue\n");
			goto out_tx_release;
		}
		INIT_DELAYED_WORK(&dsi_data->esd_work, panel_esd_check_work);
		dsi_data->esd_check = mipi_dsi_panel_esd_failed_check;
	}

	ret = platform_device_add_data(pdev, &dsi_data->panel_data,
						sizeof(dsi_data->panel_data));
	if (ret) {
		dev_err(&pdev->dev,
			"platform_device_add_data failed!\n");
		goto out_wq_release;
	}

	ret = register_attributes(&pdev->dev);
	if (ret)
		goto out_wq_release;

	mipi_dsi_pdev = msm_fb_add_device(pdev);

#ifdef CONFIG_DEBUG_FS
	mipi_dsi_panel_create_debugfs(mipi_dsi_pdev);
#endif
	dev_info(&pdev->dev, "%s: Probe success\n", __func__);
	return 0;
out_wq_release:
	if (dsi_data->panel->esd_failed_check)
		destroy_workqueue(dsi_data->esd_wq);
out_tx_release:
	mipi_dsi_buf_release(&dsi_data->rx_buf);
out_rx_release:
	mipi_dsi_buf_release(&dsi_data->tx_buf);
out_free:
	platform_set_drvdata(pdev, NULL);
	kfree(dsi_data);
	return ret;
}

static struct platform_driver this_driver = {
	.probe  = mipi_dsi_panel_probe,
	.remove = mipi_dsi_panel_remove,
	.driver = {
		.name = MIPI_DSI_PANEL_NAME,
	},
};

static int __init mipi_dsi_panel_init(void)
{
	return platform_driver_register(&this_driver);
}

static void __exit mipi_dsi_panel_exit(void)
{
	platform_driver_unregister(&this_driver);
}

module_init(mipi_dsi_panel_init);
module_exit(mipi_dsi_panel_exit);

