/* drivers/video/msm/mipi_samsung_s6d6aa0.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/workqueue.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_panel.h"

#define PANEL_ESD_CHECK_PERIOD		msecs_to_jiffies(1000)
#define DSI_VIDEO_BASE	0xE0000

static struct mutex esd_lock;
static struct msm_fb_data_type *mipi_dsi_panel_mfd;

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

	mutex_lock(&esd_lock);
	if (!dsi_data->esd_check_enable) {
		mutex_unlock(&esd_lock);
		return;
	}
	mutex_unlock(&esd_lock);

	if (dsi_data->esd_check != NULL) {
		dsi_data->esd_check(dsi_data);
		panel_esd_start_check(dsi_data);
	}
}

static int mipi_s6d6aa0_disp_on(struct msm_fb_data_type *mfd)
{
	struct mipi_dsi_data *dsi_data;
	struct dsi_controller *pctrl;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data)
		return -ENODEV;
	pctrl = dsi_data->panel->pctrl;

	if (!dsi_data->panel_detecting) {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);

		if (pctrl->display_init_cmds) {
			mipi_dsi_buf_init(&dsi_data->tx_buf);
			mipi_dsi_cmds_tx(&dsi_data->tx_buf,
				pctrl->display_init_cmds,
				pctrl->display_init_cmds_size);
		}
		if (dsi_data->eco_mode_on && pctrl->display_on_eco_cmds) {
			mipi_dsi_buf_init(&dsi_data->tx_buf);
			mipi_dsi_cmds_tx(&dsi_data->tx_buf,
				pctrl->display_on_eco_cmds,
				pctrl->display_on_eco_cmds_size);
			dev_info(&mfd->panel_pdev->dev, "ECO MODE ON\n");
		} else {
			mipi_dsi_buf_init(&dsi_data->tx_buf);
			mipi_dsi_cmds_tx(&dsi_data->tx_buf,
				pctrl->display_on_cmds,
				pctrl->display_on_cmds_size);
			dev_info(&mfd->panel_pdev->dev, "ECO MODE OFF\n");
		}
	}

	return 0;
}

static int mipi_s6d6aa0_disp_off(struct msm_fb_data_type *mfd)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	if (!dsi_data)
		return -ENODEV;

	if (!dsi_data->panel_detecting) {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);

		mipi_dsi_buf_init(&dsi_data->tx_buf);
		mipi_dsi_cmds_tx(&dsi_data->tx_buf,
			dsi_data->panel->pctrl->display_off_cmds,
			dsi_data->panel->pctrl->display_off_cmds_size);
	} else {
		dsi_data->panel_detecting = false;
	}

	return 0;
}
static int mipi_s6d6aa0_lcd_on(struct platform_device *pdev)
{
	int ret;
	struct msm_fb_data_type *mfd;
	struct mipi_dsi_data *dsi_data;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mutex_lock(&esd_lock);
	mipi_dsi_panel_mfd = mfd;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (dsi_data->panel && dsi_data->panel->plncfg)
		mipi_dsi_update_lane_cfg(dsi_data->panel->plncfg);
	ret = mipi_s6d6aa0_disp_on(mfd);
	if (ret) {
		dev_err(&pdev->dev, "%s: Display on failed\n", __func__);
	} else if (dsi_data->panel->esd_failed_check) {
		dsi_data->esd_check_enable = true;
		panel_esd_start_check(dsi_data);
	}
	mutex_unlock(&esd_lock);
	return ret;
}

static int mipi_s6d6aa0_lcd_off(struct platform_device *pdev)
{
	int ret;
	struct msm_fb_data_type *mfd;
	struct mipi_dsi_data *dsi_data;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	mutex_lock(&esd_lock);
	/*Stop the ESD check when panel off*/
	dsi_data->esd_check_enable = false;

	ret = mipi_s6d6aa0_disp_off(mfd);
	if (ret)
		dev_err(&pdev->dev, "%s: Display off failed\n", __func__);
	mutex_unlock(&esd_lock);
	return ret;
}

static int __devexit mipi_s6d6aa0_lcd_remove(struct platform_device *pdev)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	if (!dsi_data)
		return -ENODEV;

	if (dsi_data->panel->esd_failed_check)
		destroy_workqueue(dsi_data->esd_wq);

#ifdef CONFIG_DEBUG_FS
	mipi_dsi_panel_remove_debugfs(pdev);
#endif
	platform_set_drvdata(pdev, NULL);
	mipi_dsi_buf_release(&dsi_data->tx_buf);
	mipi_dsi_buf_release(&dsi_data->rx_buf);
	kfree(dsi_data);
	return 0;
}

static int __devinit mipi_s6d6aa0_lcd_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;
	struct lcd_panel_platform_data *platform_data;
	struct mipi_dsi_data *dsi_data;
	struct platform_device *fb_pdev;

	platform_data = pdev->dev.platform_data;
	if (platform_data == NULL)
		return -EINVAL;

	dsi_data = kzalloc(sizeof(struct mipi_dsi_data), GFP_KERNEL);
	if (dsi_data == NULL)
		return -ENOMEM;

	dsi_data->panel_data.on = mipi_s6d6aa0_lcd_on;
	dsi_data->panel_data.off = mipi_s6d6aa0_lcd_off;
	dsi_data->default_panels = platform_data->default_panels;
	dsi_data->panels = platform_data->panels;
	dsi_data->lcd_power = platform_data->lcd_power;
	dsi_data->lcd_reset = platform_data->lcd_reset;
	dsi_data->eco_mode_switch = mipi_dsi_eco_mode_switch;

	if (mipi_dsi_need_detect_panel(dsi_data->panels)) {
		dsi_data->panel_data.panel_detect = mipi_dsi_detect_panel;
		dsi_data->panel_data.update_panel = mipi_dsi_update_panel;
		dsi_data->panel_detecting = true;
	} else {
		dev_info(&pdev->dev, "no need to detect panel\n");
	}

	ret = mipi_dsi_buf_alloc(&dsi_data->tx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(tx) failed!\n");
		goto out_free;
	}

	ret = mipi_dsi_buf_alloc(&dsi_data->rx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(rx) failed!\n");
		goto out_rx_release;
	}

	platform_set_drvdata(pdev, dsi_data);

	mipi_dsi_set_default_panel(dsi_data);

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

	mutex_init(&esd_lock);

	ret = platform_device_add_data(pdev, &dsi_data->panel_data,
		sizeof(dsi_data->panel_data));
	if (ret) {
		dev_err(&pdev->dev,
			"platform_device_add_data failed!\n");
		goto out_wq_release;
	}
	fb_pdev = msm_fb_add_device(pdev);
#ifdef CONFIG_FB_MSM_PANEL_ECO_MODE
	eco_mode_sysfs_register(&fb_pdev->dev);
#endif
#ifdef CONFIG_DEBUG_FS
	mipi_dsi_panel_create_debugfs(fb_pdev, "mipi_s6d6aa0");
#endif

	return 0;
out_wq_release:
	if (dsi_data->panel->esd_failed_check)
		destroy_workqueue(dsi_data->esd_wq);
out_tx_release:
	mipi_dsi_buf_release(&dsi_data->rx_buf);
out_rx_release:
	mipi_dsi_buf_release(&dsi_data->tx_buf);
out_free:
	kfree(dsi_data);
	return ret;
}

static struct platform_driver this_driver = {
	.probe  = mipi_s6d6aa0_lcd_probe,
	.remove = mipi_s6d6aa0_lcd_remove,
	.driver = {
		.name   = "mipi_samsung_s6d6aa0",
	},
};

static int __init mipi_s6d6aa0_lcd_init(void)
{
	return platform_driver_register(&this_driver);
}

static void __exit mipi_s6d6aa0_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

module_init(mipi_s6d6aa0_lcd_init);
module_exit(mipi_s6d6aa0_lcd_exit);

