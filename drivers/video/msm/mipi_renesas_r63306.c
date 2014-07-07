/* drivers/video/msm/mipi_renesas_r63306.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_panel.h"

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
#define NVRW_RETRY		10
#define NVRW_SEPARATOR_POS	2
#define NVRW_NUM_ONE_PARAM	3
#define NVRW_PANEL_OFF_MSLEEP	100
#endif

static int mipi_r63306_disp_on(struct msm_fb_data_type *mfd)
{
	int ret = 0;
	struct mipi_dsi_data *dsi_data;
	struct dsi_controller *pctrl;
	struct device *dev = &mfd->panel_pdev->dev;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data || !dsi_data->lcd_power) {
		ret = -ENODEV;
		goto disp_on_fail;
	}
	pctrl = dsi_data->panel->pctrl;

	if (!dsi_data->panel_detecting) {
		ret = dsi_data->lcd_power(TRUE);

		if (ret)
			goto disp_on_fail;

		mipi_dsi_op_mode_config(DSI_CMD_MODE);
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
		mipi_set_tx_power_mode(0);
		if (dsi_data->enable_resume_check) {
			if (!dsi_data->is_nvm_ok(mfd)) {
				dev_info(dev, "NVM corrupted. Recover\n");
				mipi_set_tx_power_mode(1);
				(void)dsi_data->nvm_erase_all(mfd);
				(void)dsi_data->nvm_write_trim_area(mfd);
				(void)dsi_data->nvm_write_user_area(mfd);
				mipi_set_tx_power_mode(0);
			}
		}
#endif
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
			dev_info(dev, "ECO MODE ON\n");
		} else {
			mipi_dsi_buf_init(&dsi_data->tx_buf);
			mipi_dsi_cmds_tx(&dsi_data->tx_buf,
				pctrl->display_on_cmds,
				pctrl->display_on_cmds_size);
			dev_info(dev, "ECO MODE OFF\n");
		}
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
		mipi_set_tx_power_mode(1);
#endif
	}

disp_on_fail:
	return ret;
}

static int mipi_r63306_disp_off(struct msm_fb_data_type *mfd)
{
	int ret = 0;
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	if (!dsi_data || !dsi_data->lcd_power)
		return -ENODEV;

	if (!dsi_data->panel_detecting) {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);

		mipi_dsi_buf_init(&dsi_data->tx_buf);
		mipi_dsi_cmds_tx(&dsi_data->tx_buf,
			dsi_data->panel->pctrl->display_off_cmds,
			dsi_data->panel->pctrl->display_off_cmds_size);
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
		mipi_set_tx_power_mode(1);
#endif
		ret = dsi_data->lcd_power(FALSE);
	} else {
		dsi_data->panel_detecting = false;
		ret = 0;
	}

	return ret;
}

static int mipi_r63306_lcd_on(struct platform_device *pdev)
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
	if (dsi_data->panel && dsi_data->panel->plncfg)
		mipi_dsi_update_lane_cfg(dsi_data->panel->plncfg);
	ret = mipi_r63306_disp_on(mfd);
	if (ret)
		dev_err(&pdev->dev, "%s: Display on failed\n", __func__);

	return ret;
}

static int mipi_r63306_lcd_off(struct platform_device *pdev)
{
	int ret;
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	ret = mipi_r63306_disp_off(mfd);
	if (ret)
		dev_err(&pdev->dev, "%s: Display off failed\n", __func__);

	return ret;
}

#ifdef CONFIG_FB_MSM_RECOVER_PANEL

static void mipi_r63306_backup_nvm_to_ram(struct msm_fb_data_type *mfd)
{
	struct mipi_dsi_data *dsi_data;
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;
	int n;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_unlock,
					pnvrw_ctl->nvm_mcap_unlock_size);

	/* Backup E7 register */
	if (pnvrw_ctl->nvm_read_e7 && pnvrw_ctl->nvm_restore_e7) {
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
			pnvrw_ctl->nvm_read_e7, pnvrw_ctl->nvm_e7_nbr_params);
		for (n = 0; n < pnvrw_ctl->nvm_e7_nbr_params; n++) {
			pnvrw_ctl->nvm_e7[n + 1] = dsi_data->rx_buf.data[n];
		}
	}

	/* Backup DE register */
	if (pnvrw_ctl->nvm_read_de && pnvrw_ctl->nvm_restore_de) {
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
			pnvrw_ctl->nvm_read_de, pnvrw_ctl->nvm_de_nbr_params);
		for (n = 0; n < pnvrw_ctl->nvm_de_nbr_params; n++) {
			if (pnvrw_ctl->nvm_de_valid_param_start <= n &&
					n <= pnvrw_ctl->nvm_de_valid_param_end)
				pnvrw_ctl->nvm_de[n + 1] =
						dsi_data->rx_buf.data[n];
		}
	}
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_lock,
						pnvrw_ctl->nvm_mcap_lock_size);
}

static bool mipi_r63303_is_nvm_ok(struct msm_fb_data_type *mfd)
{
	struct mipi_dsi_data *dsi_data;
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;
	struct device *dev = &mfd->panel_pdev->dev;
	int n;
	bool verify_ok = true;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_unlock,
					pnvrw_ctl->nvm_mcap_unlock_size);

	/* Check E6 register data */
	if (pnvrw_ctl->nvm_read_e6) {
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
				pnvrw_ctl->nvm_read_e6, NVRW_NUM_E6_PARAM);
		for (n = 0; n < NVRW_NUM_E6_PARAM; n++) {
			if ((n < NVRW_NUM_E6_VALID_PARAM) &&
					(dsi_data->rx_buf.data[n] !=
						pnvrw_ctl->nvm_e6[n + 1])) {
				dev_warn(dev, "%s: E6 (r 0x%02x, 0x%02x)\n",
					__func__, dsi_data->rx_buf.data[n],
					pnvrw_ctl->nvm_e6[n + 1]);
				verify_ok = false;
				goto exit;
			}
		}
	}

	/* Check BF register data */
	 if (pnvrw_ctl->nvm_read_bf) {
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
				pnvrw_ctl->nvm_read_bf, NVRW_NUM_BF_PARAM);
		for (n = 0; n < NVRW_NUM_BF_PARAM; n++) {
			if (dsi_data->rx_buf.data[n] !=
					pnvrw_ctl->nvm_bf[n + 1]) {
				dev_warn(dev, "%s: BF (r 0x%02x, 0x%02x)\n",
					__func__, dsi_data->rx_buf.data[n],
					pnvrw_ctl->nvm_bf[n + 1]);
				verify_ok = false;
				goto exit;
			}
		}
	}
exit:
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_lock,
						pnvrw_ctl->nvm_mcap_lock_size);
	return verify_ok;
}

static ssize_t mipi_r63303_get_nvm_backup(struct mipi_dsi_data *dsi_data,
						char *buf)
{
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;
	int n;
	ssize_t ret = 0;

	if (!dsi_data || !dsi_data->panel || !dsi_data->panel->pnvrw_ctl)
		goto exit;

	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;

	for (n = 0; n < pnvrw_ctl->nvm_e7_nbr_params + 1; n++)
		ret += scnprintf(&buf[0 + ret], PAGE_SIZE - ret, " %02x",
					pnvrw_ctl->nvm_e7[n]);

	for (n = 0; n < pnvrw_ctl->nvm_de_nbr_params + 1; n++)
		ret += scnprintf(&buf[0 + ret], PAGE_SIZE - ret, " %02x",
					pnvrw_ctl->nvm_de[n]);
exit:
	return ret;
}

static int mipi_r63306_nvm_override_data(struct msm_fb_data_type *mfd,
					const char *buf, int count)
{
	struct mipi_dsi_data *dsi_data;
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;
	char work[count + 1];
	char *pos = work;
	ulong dat;
	int /*i,*/ n;
	int rc;
	int cnt = 0;
	int total_count = 0;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data || !dsi_data->panel || !dsi_data->panel->pnvrw_ctl)
		return 0;
	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;

	if (pnvrw_ctl->nvm_restore_e7)
		total_count += pnvrw_ctl->nvm_e7_nbr_params;

	if (pnvrw_ctl->nvm_restore_de)
		total_count += pnvrw_ctl->nvm_de_nbr_params;

	if (count < total_count * NVRW_NUM_ONE_PARAM - 1)
		return 0;
	memcpy(work, buf, count);
	work[count] = 0;

	if (pnvrw_ctl->nvm_restore_e7) {
		/* override E7 register data */
		for (n = 0; n < pnvrw_ctl->nvm_e7_nbr_params;
					n++, pos += NVRW_NUM_ONE_PARAM, cnt++) {
			*(pos + NVRW_SEPARATOR_POS) = 0;
			rc = strict_strtoul(pos, 16, &dat);
			if (rc < 0)
				return 0;
			pnvrw_ctl->nvm_e7[n + 1] = dat & 0xff;
		}
	}

	if (pnvrw_ctl->nvm_restore_de) {
		/* override DE register data */
		for (n = 0; n < pnvrw_ctl->nvm_de_nbr_params;
					n++, pos += NVRW_NUM_ONE_PARAM, cnt++) {
			*(pos + NVRW_SEPARATOR_POS) = 0;
			rc = strict_strtoul(pos, 16, &dat);
			if (rc < 0)
				return 0;
			pnvrw_ctl->nvm_de[n + 1] = dat & 0xff;
		}
	}
	return cnt;
}

static void nvm_retry(struct mipi_dsi_data *dsi_data,
			struct dsi_nvm_rewrite_ctl *pnvrw_ctl)
{
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_lock,
						pnvrw_ctl->nvm_mcap_lock_size);
	dsi_data->lcd_power(0);
	dsi_data->lcd_reset(0);
	dsi_data->lcd_reset(1);
	dsi_data->lcd_power(1);
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_unlock,
					pnvrw_ctl->nvm_mcap_unlock_size);
}

static void nvm_power(bool enable, struct mipi_dsi_data *dsi_data)
{
	if (enable) {
		dsi_data->lcd_reset(1);
		dsi_data->lcd_power(1);
	} else {
		dsi_data->lcd_power(0);
		dsi_data->lcd_reset(0);
		msleep(NVRW_PANEL_OFF_MSLEEP);
	}
}

static void end_nvm_command(struct mipi_dsi_data *dsi_data,
			    struct dsi_nvm_rewrite_ctl *pnvrw_ctl)
{
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_lock,
					pnvrw_ctl->nvm_mcap_lock_size);
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_enter_sleep,
					pnvrw_ctl->nvm_enter_sleep_size);
	nvm_power(false, dsi_data);
}

static int mipi_r63306_nvm_erase_all(struct msm_fb_data_type *mfd)
{
	struct device *dev = &mfd->panel_pdev->dev;
	struct mipi_dsi_data *dsi_data;
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;
	int ret = 0;
	int i;

	dev_dbg(dev, "%s: start\n", __func__);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data && !dsi_data->panel && !dsi_data->panel->pnvrw_ctl) {
		ret = -1;
		goto exit;
	}

	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;
	/* Enter sleep not in spec, but we come get here when panel is on */
	mipi_dsi_cmds_tx(&dsi_data->tx_buf,
		pnvrw_ctl->nvm_enter_sleep, pnvrw_ctl->nvm_enter_sleep_size);
	mipi_dsi_cmds_tx(&dsi_data->tx_buf,
		pnvrw_ctl->nvm_mcap_unlock, pnvrw_ctl->nvm_mcap_unlock_size);

	mipi_set_tx_power_mode(0);

	for (i = 0; i < NVRW_RETRY; i++) {
		mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_erase_all,
						pnvrw_ctl->nvm_erase_all_size);
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
					pnvrw_ctl->nvm_read_e4, 1);
		if (dsi_data->rx_buf.data[0] == 0xBD) {
			dev_warn(dev, "%s: E4 failed (0x%02x), retry %d\n",
				__func__, dsi_data->rx_buf.data[0], i + 1);
			nvm_retry(dsi_data, pnvrw_ctl);
			continue;
		}
		mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_erase_finish,
					pnvrw_ctl->nvm_erase_finish_size);
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
					pnvrw_ctl->nvm_read_e1, 1);
		if (dsi_data->rx_buf.data[0] == 0x00) {
			dev_warn(dev, "%s: E1 failed (0x%02x), retry %d\n",
				__func__, dsi_data->rx_buf.data[0], i + 1);
			nvm_retry(dsi_data, pnvrw_ctl);
			continue;
		}
		break;
	}
	if (i >= NVRW_RETRY) {
		dev_err(&mfd->panel_pdev->dev, "%s: failed.\n", __func__);
		ret = -1;
	}
	end_nvm_command(dsi_data, pnvrw_ctl);
exit:
	dev_dbg(dev, "%s: end, ret = %d\n", __func__, ret);
	return ret;
}

static int mipi_r63306_nvm_write_trim_area(struct msm_fb_data_type *mfd)
{
	struct device *dev = &mfd->panel_pdev->dev;
	struct mipi_dsi_data *dsi_data;
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;
	int i;
	int ret = 0;

	dev_dbg(dev, "%s: start\n", __func__);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data && !dsi_data->panel && !dsi_data->panel->pnvrw_ctl) {
		ret = -1;
		goto exit;
	}

	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;
	nvm_power(true, dsi_data);
	mipi_dsi_cmds_tx(&dsi_data->tx_buf,
		pnvrw_ctl->nvm_mcap_unlock, pnvrw_ctl->nvm_mcap_unlock_size);
	for (i = 0; i < NVRW_RETRY; i++) {
		mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_write_trim,
						pnvrw_ctl->nvm_write_trim_size);
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
					pnvrw_ctl->nvm_read_e1, 1);
		if (dsi_data->rx_buf.data[0] == 0x00) {
			dev_warn(dev, "%s: E1 failed (0x%02x), retry %d\n",
				__func__, dsi_data->rx_buf.data[0], i + 1);
			nvm_retry(dsi_data, pnvrw_ctl);
			continue;
		}
		break;
	}
	if (i >= NVRW_RETRY) {
		dev_err(&mfd->panel_pdev->dev, "%s : failed.\n", __func__);
		ret = -1;
	}

	end_nvm_command(dsi_data, pnvrw_ctl);
exit:
	dev_dbg(dev, "%s: end, ret = %d\n", __func__, ret);
	return ret;
}

static int mipi_r63306_nvm_write_user_area(struct msm_fb_data_type *mfd)
{
	struct device *dev = &mfd->panel_pdev->dev;
	struct mipi_dsi_data *dsi_data;
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;
	int i;
	int ret = 0;

	dev_dbg(dev, "%s: start\n", __func__);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data && !dsi_data->panel && !dsi_data->panel->pnvrw_ctl) {
		ret = -1;
		goto exit;
	}

	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;
	nvm_power(true, dsi_data);
	mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_mcap_unlock,
					pnvrw_ctl->nvm_mcap_unlock_size);
	for (i = 0; i < NVRW_RETRY; i++) {
		mipi_dsi_cmds_tx(&dsi_data->tx_buf, pnvrw_ctl->nvm_write_user,
						pnvrw_ctl->nvm_write_user_size);
		mipi_dsi_cmds_rx(mfd, &dsi_data->tx_buf, &dsi_data->rx_buf,
					pnvrw_ctl->nvm_read_e1, 1);
		if (dsi_data->rx_buf.data[0] != 0x03) {
			dev_warn(dev, "%s: E1 failed (0x%02x), retry %d\n",
				__func__, dsi_data->rx_buf.data[0], i + 1);
			nvm_retry(dsi_data, pnvrw_ctl);
			continue;
		}
		break;
	}
	if (i >= NVRW_RETRY) {
		dev_err(&mfd->panel_pdev->dev, "%s: failed.\n", __func__);
		ret = -1;
	}

	end_nvm_command(dsi_data, pnvrw_ctl);
	nvm_power(true, dsi_data);
exit:
	dev_dbg(dev, "%s: end, ret = %d\n", __func__, ret);
	return ret;
}
#endif /* CONFIG_FB_MSM_RECOVER_PANEL */

static int __devexit mipi_r63306_lcd_remove(struct platform_device *pdev)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	if (!dsi_data)
		return -ENODEV;

#ifdef CONFIG_DEBUG_FS
	mipi_dsi_panel_remove_debugfs(pdev);
#endif
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	remove_sysfs_interfaces(&pdev->dev);
#endif
	platform_set_drvdata(pdev, NULL);
	mipi_dsi_buf_release(&dsi_data->tx_buf);
	mipi_dsi_buf_release(&dsi_data->rx_buf);
	kfree(dsi_data);
	return 0;
}

static int __devinit mipi_r63306_lcd_probe(struct platform_device *pdev)
{
	int ret;
	struct lcd_panel_platform_data *platform_data;
	struct mipi_dsi_data *dsi_data;
	struct platform_device *fb_pdev;

	platform_data = pdev->dev.platform_data;
	if (platform_data == NULL)
		return -EINVAL;

	dsi_data = kzalloc(sizeof(struct mipi_dsi_data), GFP_KERNEL);
	if (dsi_data == NULL)
		return -ENOMEM;
	dsi_data->panel_data.on = mipi_r63306_lcd_on;
	dsi_data->panel_data.off = mipi_r63306_lcd_off;
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
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	dsi_data->is_nvm_ok = mipi_r63303_is_nvm_ok;
	dsi_data->backup_nvm_to_ram = mipi_r63306_backup_nvm_to_ram;
	dsi_data->get_nvm_backup = mipi_r63303_get_nvm_backup;
	dsi_data->override_nvm_data = mipi_r63306_nvm_override_data;
	dsi_data->nvm_erase_all = mipi_r63306_nvm_erase_all;
	dsi_data->nvm_write_trim_area = mipi_r63306_nvm_write_trim_area;
	dsi_data->nvm_write_user_area = mipi_r63306_nvm_write_user_area;
	dsi_data->panel_nvm_ok = true;
	dsi_data->panel_nvm_backup_ok = false;
#endif
	ret = mipi_dsi_buf_alloc(&dsi_data->tx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(tx) failed!\n");
		goto err_dsibuf_free;
	}

	ret = mipi_dsi_buf_alloc(&dsi_data->rx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(rx) failed!\n");
		goto err_txbuf_free;
	}

	platform_set_drvdata(pdev, dsi_data);

	mipi_dsi_set_default_panel(dsi_data);

	ret = platform_device_add_data(pdev, &dsi_data->panel_data,
		sizeof(dsi_data->panel_data));
	if (ret) {
		dev_err(&pdev->dev,
			"platform_device_add_data failed!\n");
		goto err_rxbuf_free;
	}
	fb_pdev = msm_fb_add_device(pdev);
#ifdef CONFIG_FB_MSM_PANEL_ECO_MODE
	eco_mode_sysfs_register(&fb_pdev->dev);
#endif
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
	create_sysfs_interfaces(&fb_pdev->dev);
#endif
#ifdef CONFIG_DEBUG_FS
	mipi_dsi_panel_create_debugfs(fb_pdev, "mipi_r63306");
#endif

	return 0;
err_rxbuf_free:
	mipi_dsi_buf_release(&dsi_data->rx_buf);
err_txbuf_free:
	mipi_dsi_buf_release(&dsi_data->tx_buf);
err_dsibuf_free:
	kfree(dsi_data);
	return ret;
}

static struct platform_driver this_driver = {
	.probe  = mipi_r63306_lcd_probe,
	.remove = mipi_r63306_lcd_remove,
	.driver = {
		.name   = "mipi_renesas_r63306",
	},
};

static int __init mipi_r63306_lcd_init(void)
{
	return platform_driver_register(&this_driver);
}

static void __exit mipi_r63306_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

module_init(mipi_r63306_lcd_init);
module_exit(mipi_r63306_lcd_exit);

