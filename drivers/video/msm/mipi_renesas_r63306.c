/* drivers/video/msm/mipi_renesas_r63306.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <mach/mipi_dsi_renesas.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_video_panel.h"

struct mipi_r63306_data {
	struct lcd_panel_platform_data *pdata;
	struct dsi_buf tx_buf;
	struct dsi_buf rx_buf;
	struct msm_fb_panel_data panel_data;
	const struct panel_id *panel;
#ifdef CONFIG_DEBUG_FS
	struct dentry *r63306_dir;
	char *debug_buf;
#endif
};

#ifdef CONFIG_DEBUG_FS

#define MIPI_R63306_DEBUG_BUF	2048

#define MSNPRINTF(buf, rsize, ...)			\
do {							\
	ssize_t act = 0;					\
							\
	if (rsize > 0)					\
		act = snprintf(buf, rsize, __VA_ARGS__);	\
	buf += act;					\
	rsize -= act;					\
} while (0)

static void print_cmds2buf(struct dsi_cmd_desc *cmds, int cnt,
			 char **buf, int *rem_size)
{
	int i, j;

	if (!cmds) {
		MSNPRINTF(*buf, *rem_size, "cmds NULL\n");
		goto exit;
	}

	for (i = 0; i < cnt; i++) {
		switch (cmds[i].dtype) {
		case DTYPE_DCS_WRITE:
		case DTYPE_DCS_WRITE1:
			MSNPRINTF(*buf, *rem_size, "DCS_WRITE: ");
			break;
		case DTYPE_DCS_LWRITE:
			MSNPRINTF(*buf, *rem_size, "DCS_LONG_WRITE: ");
			break;
		case DTYPE_GEN_WRITE:
		case DTYPE_GEN_WRITE1:
		case DTYPE_GEN_WRITE2:
			MSNPRINTF(*buf, *rem_size, "GEN_WRITE: ");
			break;
		case DTYPE_GEN_LWRITE:
			MSNPRINTF(*buf, *rem_size, "GEN_LONG_WRITE: ");
			break;
		case DTYPE_DCS_READ:
			MSNPRINTF(*buf, *rem_size, "DCS_READ: ");
			break;
		case DTYPE_GEN_READ:
		case DTYPE_GEN_READ1:
		case DTYPE_GEN_READ2:
			MSNPRINTF(*buf, *rem_size, "GEN_READ: ");
			break;
		case DTYPE_MAX_PKTSIZE:
			MSNPRINTF(*buf, *rem_size, "SET_MAX_PACKET_SIZE: ");
			break;
		case DTYPE_NULL_PKT:
			MSNPRINTF(*buf, *rem_size, "NULL_PACKET: ");
			break;
		case DTYPE_BLANK_PKT:
			MSNPRINTF(*buf, *rem_size, "BLANK_PACKET: ");
			break;
		case DTYPE_PERIPHERAL_ON:
			MSNPRINTF(*buf, *rem_size, "PERIPHERAL_ON: ");
			break;
		case DTYPE_PERIPHERAL_OFF:
			MSNPRINTF(*buf, *rem_size, "PERIPHERAL_OFF: ");
			break;
		default:
			MSNPRINTF(*buf, *rem_size, "UnknownData: ");
			break;
		}
		for (j = 0; j < cmds[i].dlen; j++)
			MSNPRINTF(*buf, *rem_size, "0x%.2x ",
				  cmds[i].payload[j]);
		MSNPRINTF(*buf, *rem_size, "\n");
	}
	MSNPRINTF(*buf, *rem_size, "---------\n");
exit:
	return;
}

static int mipi_r63306_cmd_seq_open(struct inode *inode, struct file *file)
{
	struct mipi_r63306_data *dsi_data;

	dsi_data = inode->i_private;
	if (dsi_data->debug_buf != NULL)
		return -EBUSY;
	dsi_data->debug_buf = kzalloc(MIPI_R63306_DEBUG_BUF, GFP_KERNEL);
	file->private_data = dsi_data;
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_r63306_cmd_seq_release(struct inode *inode, struct file *file)
{
	struct mipi_r63306_data *dsi_data;

	dsi_data = file->private_data;
	kfree(dsi_data->debug_buf);
	dsi_data->debug_buf = NULL;
	return 0;
}

static ssize_t mipi_r63306_cmd_seq_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	char *bp;
	int len = 0;
	int tot = 0;
	int dlen;
	struct mipi_r63306_data *dsi_data;

	if (*ppos)
		return 0;

	dsi_data = file->private_data;

	bp = dsi_data->debug_buf;
	if (bp == NULL)
		return 0;

	dlen = MIPI_R63306_DEBUG_BUF;

	if (dsi_data->pdata->panels[0]) {
		/* show panel info */
		MSNPRINTF(bp, dlen, "Register data for panel %s\n",
			  dsi_data->pdata->panels[0]->name);
		MSNPRINTF(bp, dlen, "xres = %d, yres = %d\n",
			  dsi_data->panel_data.panel_info.xres,
			  dsi_data->panel_data.panel_info.yres);
		MSNPRINTF(bp, dlen, "width = %d mm, height = %d mm\n",
			  dsi_data->panel_data.panel_info.width,
			  dsi_data->panel_data.panel_info.height);
		/* show commands */
		MSNPRINTF(bp, dlen, "init cmds:\n");
		print_cmds2buf(dsi_data->panel->pctrl->display_init_cmds,
			     dsi_data->panel->pctrl->display_init_cmds_size,
			       &bp, &dlen);
		MSNPRINTF(bp, dlen, "display_on cmds:\n");
		print_cmds2buf(dsi_data->panel->pctrl->display_on_cmds,
			     dsi_data->panel->pctrl->display_on_cmds_size,
			       &bp, &dlen);
		MSNPRINTF(bp, dlen, "display_off cmds:\n");
		print_cmds2buf(dsi_data->panel->pctrl->display_off_cmds,
			     dsi_data->panel->pctrl->display_off_cmds_size,
			       &bp, &dlen);
	} else {
		len = snprintf(bp, dlen, "No panel name\n");
		bp += len;
		dlen -= len;
	}

	tot = (uint32)bp - (uint32)dsi_data->debug_buf;
	*bp = 0;
	tot++;

	if (tot < 0)
		return 0;
	if (copy_to_user(buff, dsi_data->debug_buf, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations mipi_r63306_cmd_seq_fops = {
	.open = mipi_r63306_cmd_seq_open,
	.release = mipi_r63306_cmd_seq_release,
	.read = mipi_r63306_cmd_seq_read,
};

static void mipi_r63306_debugfs_init(struct platform_device *pdev)
{
	struct dentry *root;
	struct dentry *file;
	const char *sub_name = "mipi_r63306";
	struct mipi_r63306_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	root = msm_fb_get_debugfs_root();
	if (root != NULL) {
		dsi_data->r63306_dir = debugfs_create_dir(sub_name, root);

		if (IS_ERR(dsi_data->r63306_dir) ||
			(dsi_data->r63306_dir == NULL)) {
			dev_err(&pdev->dev,
				"debugfs_create_dir fail, error %ld\n",
				PTR_ERR(dsi_data->r63306_dir));
		} else {
			file = debugfs_create_file("cmd_seq", 0444,
				dsi_data->r63306_dir, dsi_data,
				&mipi_r63306_cmd_seq_fops);
			if (file == NULL)
				dev_err(&pdev->dev,
					"debugfs_create_file: index fail\n");
		}
	}
}

static void mipi_r63306_debugfs_exit(struct platform_device *pdev)
{
	struct mipi_r63306_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	debugfs_remove_recursive(dsi_data->r63306_dir);
}

#endif

static void mipi_r63306_set_default_panel(struct mipi_r63306_data *dsi_data)
{
	/* Set panel[0] as default */
	dsi_data->panel = dsi_data->pdata->panels[0];
	MSM_FB_INFO("default panel: %s\n", dsi_data->panel->name);
	dsi_data->panel_data.panel_info =
		*dsi_data->panel->pctrl->get_panel_info();
}

static int panel_id_reg_check(struct msm_fb_data_type *mfd, struct dsi_buf *ptx,
			      struct dsi_buf *prx, const struct panel_id* panel)
{
	int i;

	mutex_lock(&mfd->dma->ov_mutex);
	MSM_FB_INFO("%s: mutex_locked\n", __func__);
	mipi_dsi_buf_init(prx);
	mipi_dsi_buf_init(ptx);
	mipi_dsi_cmds_rx(mfd, ptx, prx, panel->pctrl->read_id_cmds,
			 panel->id_num);
	mutex_unlock(&mfd->dma->ov_mutex);
	MSM_FB_INFO("%s: mutex_unlocked\n", __func__);

	for (i = 0; i < panel->id_num; i++) {
		if (i >= prx->len || prx->data[i] != panel->id[i])
			return -ENODEV;
	}
	return 0;
}

static int mipi_r63306_disp_on(struct msm_fb_data_type *mfd)
{
	int ret;
	struct mipi_r63306_data *dsi_data;

	MSM_FB_INFO("%s: disp on\n", __func__);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data || !dsi_data->pdata ||
	    !dsi_data->pdata->lcd_power)
		return -ENODEV;

	ret = dsi_data->pdata->lcd_power(TRUE);

	if (!ret) {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);

		mipi_dsi_buf_init(&dsi_data->tx_buf);

		mipi_dsi_cmds_tx(&dsi_data->tx_buf,
			 dsi_data->panel->pctrl->display_init_cmds,
			 dsi_data->panel->pctrl->display_init_cmds_size);

		mipi_dsi_buf_init(&dsi_data->tx_buf);

		mipi_dsi_cmds_tx(&dsi_data->tx_buf,
			 dsi_data->panel->pctrl->display_on_cmds,
			 dsi_data->panel->pctrl->display_on_cmds_size);
	}

	MSM_FB_INFO("%s: disp on end\n", __func__);

	return ret;
}

static int mipi_r63306_disp_off(struct msm_fb_data_type *mfd)
{
	struct mipi_r63306_data *dsi_data;

	MSM_FB_INFO("%s: disp_off\n", __func__);

	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	if (!dsi_data || !dsi_data->pdata ||
	    !dsi_data->pdata->lcd_power)
		return -ENODEV;

	mipi_dsi_op_mode_config(DSI_CMD_MODE);

	mipi_dsi_buf_init(&dsi_data->tx_buf);
	mipi_dsi_cmds_tx(&dsi_data->tx_buf,
			dsi_data->panel->pctrl->display_off_cmds,
			dsi_data->panel->pctrl->display_off_cmds_size);

	MSM_FB_INFO("%s: disp_off end\n", __func__);

	return dsi_data->pdata->lcd_power(FALSE);
}

static int mipi_r63306_lcd_on(struct platform_device *pdev)
{
	int ret;
	struct msm_fb_data_type *mfd;

	MSM_FB_INFO("%s: lcd on\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	ret = mipi_r63306_disp_on(mfd);
	if (ret)
		dev_err(&pdev->dev, "%s: Display on failed\n", __func__);

	MSM_FB_INFO("%s: lcd on end\n", __func__);

	return ret;
}

static int mipi_r63306_lcd_off(struct platform_device *pdev)
{
	int ret;
	struct msm_fb_data_type *mfd;

	MSM_FB_INFO("%s: lcd off\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	ret = mipi_r63306_disp_off(mfd);
	if (ret)
		dev_err(&pdev->dev, "%s: Display off failed\n", __func__);

	MSM_FB_INFO("%s: lcd off end\n", __func__);

	return ret;
}

static struct msm_panel_info *mipi_r63306_detect_panel
(struct msm_fb_data_type *mfd)
{
	int i;
	int ret;
	struct mipi_r63306_data *dsi_data;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	mipi_dsi_op_mode_config(DSI_CMD_MODE);
	for (i = 0; dsi_data->pdata->panels[i]; i++) {
		ret = panel_id_reg_check(mfd, &dsi_data->tx_buf,
					 &dsi_data->rx_buf,
					 dsi_data->pdata->panels[i]);
		if (!ret)
			break;
	}

	if (dsi_data->pdata->panels[i]) {
		dsi_data->panel = dsi_data->pdata->panels[i];
		dsi_data->panel_data.panel_info =
			*dsi_data->panel->pctrl->get_panel_info();
		mipi_dsi_op_mode_config
			(dsi_data->panel_data.panel_info.mipi.mode);
		dev_info(&mfd->panel_pdev->dev, "found panel: %s\n",
			 dsi_data->panel->name);
	} else {
		dev_warn(&mfd->panel_pdev->dev, "cannot detect panel!\n");
		return NULL;
	}

	return &dsi_data->panel_data.panel_info;
}

static int __devexit mipi_r63306_lcd_remove(struct platform_device *pdev)
{
	struct mipi_r63306_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	if (!dsi_data)
		return -ENODEV;

#ifdef CONFIG_DEBUG_FS
	mipi_r63306_debugfs_exit(pdev);
#endif

	platform_set_drvdata(pdev, NULL);
	mipi_dsi_buf_release(&dsi_data->tx_buf);
	mipi_dsi_buf_release(&dsi_data->rx_buf);
	kfree(dsi_data);
	return 0;
}

static int __devinit mipi_r63306_need_detect_panel
(const struct panel_id **panels)
{
	int num = 0;
	int i;

	for (i = 0; panels[i]; i++)
		num++;

	return (num > 1) ? 1 : 0;
}

static int __devinit mipi_r63306_lcd_probe(struct platform_device *pdev)
{
	int ret;
	struct lcd_panel_platform_data *platform_data;
	struct mipi_r63306_data *dsi_data;

	MSM_FB_INFO("%s: lcd probe\n", __func__);

	platform_data = pdev->dev.platform_data;
	pdev->dev.platform_data = NULL;

	if (platform_data == NULL)
		return -EINVAL;

	dsi_data = kzalloc(sizeof(struct mipi_r63306_data), GFP_KERNEL);
	if (dsi_data == NULL)
		return -ENOMEM;

	dsi_data->pdata = platform_data;
	dsi_data->panel_data.on = mipi_r63306_lcd_on;
	dsi_data->panel_data.off = mipi_r63306_lcd_off;
	if (mipi_r63306_need_detect_panel(dsi_data->pdata->panels))
		dsi_data->panel_data.panel_detect = mipi_r63306_detect_panel;
	else
		dev_info(&pdev->dev, "no need to detect panel\n");

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client(dsi_data->pdata->pdata->name)) {
		kfree(dsi_data);
		return -ENODEV;
	}
#endif

	ret = mipi_dsi_buf_alloc(&dsi_data->tx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(tx) failed!\n");
		kfree(dsi_data);
		return ret;
	}

	ret = mipi_dsi_buf_alloc(&dsi_data->rx_buf, DSI_BUF_SIZE);
	if (ret <= 0) {
		dev_err(&pdev->dev, "mipi_dsi_buf_alloc(rx) failed!\n");
		mipi_dsi_buf_release(&dsi_data->tx_buf);
		kfree(dsi_data);
		return ret;
	}

	platform_set_drvdata(pdev, dsi_data);

	mipi_r63306_set_default_panel(dsi_data);

	ret = platform_device_add_data(pdev, &dsi_data->panel_data,
		sizeof(dsi_data->panel_data));
	if (ret) {
		dev_err(&pdev->dev,
			"platform_device_add_data failed!\n");
		mipi_dsi_buf_release(&dsi_data->tx_buf);
		mipi_dsi_buf_release(&dsi_data->rx_buf);
		kfree(dsi_data);
		return ret;
	}

	msm_fb_add_device(pdev);
#ifdef CONFIG_DEBUG_FS
	mipi_r63306_debugfs_init(pdev);
#endif

	MSM_FB_INFO("%s: lcd probe end\n", __func__);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_r63306_lcd_probe,
	.remove = mipi_r63306_lcd_remove,
	.driver = {
		.name   = MIPI_DEVICE_NAME,
	},
};

static int __init mipi_r63306_lcd_init(void)
{
	MSM_FB_INFO("%s: mipi_init\n", __func__);
	return platform_driver_register(&this_driver);
}

static void __exit mipi_r63306_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

module_init(mipi_r63306_lcd_init);
module_exit(mipi_r63306_lcd_exit);

