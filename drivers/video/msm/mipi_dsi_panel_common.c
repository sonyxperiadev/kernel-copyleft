/* drivers/video/msm/mipi_dsi_panel_common.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_panel.h"

#if defined(CONFIG_FB_MSM_RECOVER_PANEL) || defined(CONFIG_DEBUG_FS)
#define DSI_VIDEO_BASE	0xE0000
#endif

void mipi_dsi_set_default_panel(struct mipi_dsi_data *dsi_data)
{
	if (dsi_data->default_panels[0] != NULL)
		dsi_data->panel = dsi_data->default_panels[0];
	else
		dsi_data->panel = dsi_data->panels[0];

	MSM_FB_INFO("default panel: %s\n", dsi_data->panel->name);
	dsi_data->panel_data.panel_info =
		*dsi_data->panel->pctrl->get_panel_info();
	dsi_data->panel_data.panel_info.width =
		dsi_data->panel->width;
	dsi_data->panel_data.panel_info.height =
		dsi_data->panel->height;
	esc_byte_ratio =
			dsi_data->panel_data.panel_info.mipi.esc_byte_ratio;
}

static int panel_id_reg_check(struct msm_fb_data_type *mfd, struct dsi_buf *ptx,
			      struct dsi_buf *prx, const struct panel_id *panel)
{
	int i;

	mutex_lock(&mfd->dma->ov_mutex);
	mipi_dsi_buf_init(prx);
	mipi_dsi_buf_init(ptx);
	mipi_dsi_cmds_rx(mfd, ptx, prx, panel->pctrl->read_id_cmds,
			 panel->id_num);
	mutex_unlock(&mfd->dma->ov_mutex);

	for (i = 0; i < panel->id_num; i++) {
		if ((i >= prx->len) ||
			((prx->data[i] != panel->id[i]) &&
				(panel->id[i] != 0xff)))
			return -ENODEV;
	}
	return 0;
}

struct msm_panel_info *mipi_dsi_detect_panel(
	struct msm_fb_data_type *mfd)
{
	int i;
	int ret;
	struct mipi_dsi_data *dsi_data;
	struct device *dev = &mfd->panel_pdev->dev;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	if (dsi_data->default_panels[0] != NULL) {
		for (i = 0; dsi_data->default_panels[i]; i++) {
			ret = panel_id_reg_check(mfd, &dsi_data->tx_buf,
						 &dsi_data->rx_buf,
						 dsi_data->default_panels[i]);
			if (!ret)
				break;
		}

		if (dsi_data->default_panels[i]) {
			dsi_data->panel = dsi_data->default_panels[i];
			dev_info(dev, "found panel vendor: %s\n",
							dsi_data->panel->name);
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
		} else {
			/* The recovery can only be done if we have one default
			panel defined, which means i will be 1 here */
			if (i == 1) {
				dev_warn(dev, "Unable to detect panel vendor, "
							"try to recover.\n");
				dsi_data->panel_nvm_ok = false;
				dsi_data->panel = dsi_data->default_panels[0];
				dev_info(dev, "Assume panel vendor: %s\n",
							dsi_data->panel->name);
#endif
			} else {
				dev_warn(dev, "cannot detect panel vendor!\n");
				return NULL;
			}
		}
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	}
	if (dsi_data->panel_nvm_ok) {
#endif
		for (i = 0; dsi_data->panels[i]; i++) {
			ret = panel_id_reg_check(mfd, &dsi_data->tx_buf,
					 &dsi_data->rx_buf,
					 dsi_data->panels[i]);
			if (!ret)
				break;
		}
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	} else {
		for (i = 0; dsi_data->panels[i]; i++) {
			if (dsi_data->panels[i]->use_if_nv_fail) {
				dev_info(dev, "Assume panel name: %s\n",
						dsi_data->panels[i]->name);
				break;
			}
		}
	}
#endif

	if (dsi_data->panels[i]) {
		dsi_data->panel = dsi_data->panels[i];
		dev_info(dev, "found panel: %s\n", dsi_data->panel->name);
	} else {
		dev_warn(dev, "cannot detect panel!\n");
		return NULL;
	}

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	/* Verify NVM even if we have managed to detect panel */
	if (dsi_data->panel_nvm_ok && dsi_data->panel->pnvrw_ctl &&
			dsi_data->backup_nvm_to_ram && dsi_data->is_nvm_ok) {
		dsi_data->backup_nvm_to_ram(mfd);
		if (dsi_data->is_nvm_ok(mfd))
			dsi_data->panel_nvm_backup_ok = true;
		else
			dsi_data->panel_nvm_ok = false;
	}
#endif

	dsi_data->panel_data.panel_info =
		*dsi_data->panel->pctrl->get_panel_info();
	dsi_data->panel_data.panel_info.width =
		dsi_data->panel->width;
	dsi_data->panel_data.panel_info.height =
		dsi_data->panel->height;
	dsi_data->panel_data.panel_info.mipi.dsi_pclk_rate =
		mfd->panel_info.mipi.dsi_pclk_rate;
	mipi_dsi_op_mode_config
		(dsi_data->panel_data.panel_info.mipi.mode);

	return &dsi_data->panel_data.panel_info;
}

int __devinit mipi_dsi_need_detect_panel(
	const struct panel_id **panels)
{
	int num = 0;
	int i;

	for (i = 0; panels[i]; i++)
		num++;

	return (num > 1) ? 1 : 0;
}

int mipi_dsi_update_panel(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct msm_panel_info *pinfo;
	struct mipi_panel_info *mipi;
	uint8 lanes = 0, bpp;
	uint32 h_period, v_period, dsi_pclk_rate;

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

	h_period = ((pinfo->lcdc.h_pulse_width)
			+ (pinfo->lcdc.h_back_porch)
			+ (pinfo->xres)
			+ (pinfo->lcdc.h_front_porch));

	v_period = ((pinfo->lcdc.v_pulse_width)
			+ (pinfo->lcdc.v_back_porch)
			+ (pinfo->yres)
			+ (pinfo->lcdc.v_front_porch));

	mipi  = &pinfo->mipi;

	if (mipi->data_lane3)
		lanes += 1;
	if (mipi->data_lane2)
		lanes += 1;
	if (mipi->data_lane1)
		lanes += 1;
	if (mipi->data_lane0)
		lanes += 1;

	if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB666_LOOSE))
		bpp = 3;
	else if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
		 || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB565))
		bpp = 2;
	else
		bpp = 3;		/* Default format set to RGB888 */

	if (pinfo->type == MIPI_VIDEO_PANEL) {
		if (lanes > 0) {
			pinfo->clk_rate =
			((h_period * v_period * (mipi->frame_rate) * bpp * 8)
			   / lanes);
		} else {
			pr_err("%s: forcing mipi_dsi lanes to 1\n", __func__);
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

void mipi_dsi_update_lane_cfg(const struct mipi_dsi_lane_cfg *plncfg)
{
	int i, j, ln_offset;

	ln_offset = 0x300;
	for (i = 0; i < 4; i++) {
		/* DSI1_DSIPHY_LN_CFG */
		for (j = 0; j < 3; j++)
			MIPI_OUTP(MIPI_DSI_BASE + ln_offset + j * 4,
				plncfg->ln_cfg[i][j]);
		/* DSI1_DSIPHY_LN_TEST_DATAPATH */
		MIPI_OUTP(MIPI_DSI_BASE + ln_offset + 0x0c,
			plncfg->ln_dpath[i]);
		/* DSI1_DSIPHY_LN_TEST_STR */
		for (j = 0; j < 2; j++)
			MIPI_OUTP(MIPI_DSI_BASE + ln_offset + 0x14 + j * 4,
				plncfg->ln_str[i][j]);

		ln_offset += 0x40;
	}

	/* DSI1_DSIPHY_LNCK_CFG */
	for (i = 0; i < 3; i++)
		MIPI_OUTP(MIPI_DSI_BASE + 0x0400 + i * 4,
			plncfg->lnck_cfg[i]);
	/* DSI1_DSIPHY_LNCK_TEST_DATAPATH */
	MIPI_OUTP(MIPI_DSI_BASE + 0x040c, plncfg->lnck_dpath);
	/* DSI1_DSIPHY_LNCK_TEST_STR */
	for (i = 0; i < 2; i++)
		MIPI_OUTP(MIPI_DSI_BASE + 0x0414 + i * 4,
			plncfg->lnck_str[i]);
}

int mipi_dsi_eco_mode_switch(struct msm_fb_data_type *mfd)
{
	int ret = 0;
	struct mipi_dsi_data *dsi_data;
	struct dsi_controller *pctrl;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data || !dsi_data->lcd_power) {
		ret = -ENODEV;
		goto eco_mode_switch_fail;
	}
	pctrl = dsi_data->panel->pctrl;

	mipi_set_tx_power_mode(0);

	if (dsi_data->eco_mode_on && pctrl->eco_mode_gamma_cmds) {
		mipi_dsi_buf_init(&dsi_data->tx_buf);
		mipi_dsi_cmds_tx(&dsi_data->tx_buf,
			pctrl->eco_mode_gamma_cmds,
			pctrl->eco_mode_gamma_cmds_size);
		dev_info(&mfd->panel_pdev->dev, "ECO MODE ON\n");
	} else if (pctrl->normal_gamma_cmds) {
		mipi_dsi_buf_init(&dsi_data->tx_buf);
		mipi_dsi_cmds_tx(&dsi_data->tx_buf,
			pctrl->normal_gamma_cmds,
			pctrl->normal_gamma_cmds_size);
		dev_info(&mfd->panel_pdev->dev, "ECO MODE OFF\n");
	}

	mipi_set_tx_power_mode(1);

	return ret;

eco_mode_switch_fail:
	return ret;
}

static ssize_t show_eco_mode(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct platform_device *pdev;
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_data_type *mfd;

	pdev = container_of(dev, struct platform_device, dev);
	mfd = platform_get_drvdata(pdev);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	return snprintf(buf, PAGE_SIZE, "%i\n", dsi_data->eco_mode_on);
}

static ssize_t store_eco_mode(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	ssize_t ret;
	struct platform_device *pdev;
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_data_type *mfd;
	struct msm_fb_panel_data *pdata = NULL;

	pdev = container_of(dev, struct platform_device, dev);
	mfd = platform_get_drvdata(pdev);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if (sscanf(buf, "%i", &ret) != 1) {
		printk(KERN_ERR"Invalid flag for eco_mode\n");
		goto exit;
	}

	if (ret)
		dsi_data->eco_mode_on = true;
	else
		dsi_data->eco_mode_on = false;

	if (mfd->panel_power_on)
		dsi_data->eco_mode_switch(mfd);

exit:
	ret = strnlen(buf, count);

	return ret;
}

static struct device_attribute eco_mode_attributes[] = {
	__ATTR(eco_mode, 0644, show_eco_mode, store_eco_mode),
};

int eco_mode_sysfs_register(struct device *dev)
{
	int i;

	dev_dbg(dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(eco_mode_attributes); i++)
		if (device_create_file(dev, eco_mode_attributes + i))
			goto error;

	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, eco_mode_attributes + i);

	dev_err(dev, "%s: Unable to create interface\n", __func__);

	return -ENODEV;
}

#ifdef CONFIG_FB_MSM_RECOVER_PANEL

static ssize_t nvm_is_ok_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_data_type *mfd;
	ssize_t ret;

	pdev = container_of(dev, struct platform_device, dev);
	mfd = platform_get_drvdata(pdev);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	if (!dsi_data || !dsi_data->panel || !dsi_data->panel->pnvrw_ctl) {
		ret = scnprintf(buf, PAGE_SIZE, "skip");
	} else if (dsi_data->panel_nvm_ok) {
		dsi_data->enable_resume_check = true;
		ret = scnprintf(buf, PAGE_SIZE, "OK");
	} else {
		dsi_data->enable_resume_check = true;
		ret = scnprintf(buf, PAGE_SIZE, "NG");
	}
	return ret;
}

static ssize_t nvm_data_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_data_type *mfd;
	ssize_t ret;

	pdev = container_of(dev, struct platform_device, dev);
	mfd = platform_get_drvdata(pdev);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	if (!dsi_data || !dsi_data->panel || !dsi_data->panel->pnvrw_ctl ||
			!dsi_data->get_nvm_backup)
		ret = scnprintf(buf, PAGE_SIZE, "NG");
	else
		ret = dsi_data->get_nvm_backup(dsi_data, buf);

	return ret;
}

static ssize_t nvm_data_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct platform_device *pdev;
	struct mipi_dsi_data *dsi_data;
	struct msm_fb_data_type *mfd;
	int rc;

	pdev = container_of(dev, struct platform_device, dev);
	mfd = platform_get_drvdata(pdev);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	if (!dsi_data->panel->pnvrw_ctl)
		goto exit;

	if (dsi_data->override_nvm_data) {
		rc = dsi_data->override_nvm_data(mfd, buf, count);
		if (rc == 0) {
			dev_err(dev, "%s : nvm data format error.<%s>\n",
							__func__, buf);
			goto exit;
		}
	}

	if (prepare_for_reg_access(mfd))
		goto exit;

	mipi_set_tx_power_mode(1);

	if (dsi_data->nvm_erase_all && dsi_data->nvm_write_trim_area &&
					dsi_data->nvm_write_user_area) {
		(void)dsi_data->nvm_erase_all(mfd);
		(void)dsi_data->nvm_write_trim_area(mfd);
		(void)dsi_data->nvm_write_user_area(mfd);
	}
	if (dsi_data->panel_data.on)
		dsi_data->panel_data.on(mfd->pdev);

	post_reg_access(mfd);

exit:
	return count;
}

static struct device_attribute panel_nvm_attributes[] = {
	__ATTR(nvm_is_ok, S_IRUGO, nvm_is_ok_show, NULL),
	__ATTR(nvm_data, S_IRUGO | S_IWUSR, nvm_data_show, nvm_data_store),
};

int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(panel_nvm_attributes); i++)
		if (device_create_file(dev, panel_nvm_attributes + i))
			goto failed;
	return 0;

failed:
	for (--i; i >= 0; i--)
		device_remove_file(dev, panel_nvm_attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(panel_nvm_attributes); i++)
		device_remove_file(dev, panel_nvm_attributes + i);

}
#endif /* CONFIG_FB_MSM_RECOVER_PANEL */

#if defined(CONFIG_FB_MSM_RECOVER_PANEL) || defined(CONFIG_DEBUG_FS)
int prepare_for_reg_access(struct msm_fb_data_type *mfd)
{
	struct device *dev = &mfd->panel_pdev->dev;
	struct mipi_dsi_data *dsi_data;
	int ret = 0;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	mutex_lock(&mfd->nvrw_prohibit_draw);
#endif
	/* Needed to make sure the display stack isn't powered on/off while */
	/* we are executing. Also locks in msm_fb.c */
	mutex_lock(&mfd->power_lock);
	if (!mfd->panel_power_on) {
		dev_err(dev, "%s: panel is OFF, not supported\n", __func__);
		mutex_unlock(&mfd->power_lock);
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
		mutex_unlock(&mfd->nvrw_prohibit_draw);
#endif
		ret = -EINVAL;
		goto exit;
	}

	mutex_lock(&mfd->dma->ov_mutex);
	if (mfd->panel_info.mipi.mode == DSI_VIDEO_MODE) {
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
		msleep(ONE_FRAME_TRANSMIT_WAIT_MS);
		mipi_dsi_controller_cfg(0);
		mipi_dsi_op_mode_config(DSI_CMD_MODE);
	}
exit:
	return ret;
}

void post_reg_access(struct msm_fb_data_type *mfd)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (mfd->panel_info.mipi.mode == DSI_VIDEO_MODE) {
		mipi_dsi_op_mode_config(DSI_VIDEO_MODE);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		mipi_dsi_sw_reset();
		mipi_dsi_controller_cfg(1);
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 1);
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	}
	mutex_unlock(&mfd->dma->ov_mutex);
	mutex_unlock(&mfd->power_lock);
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	mutex_unlock(&mfd->nvrw_prohibit_draw);
#endif
}

#endif /* defined(CONFIG_FB_MSM_RECOVER_PANEL) || defined(CONFIG_DEBUG_FS) */



