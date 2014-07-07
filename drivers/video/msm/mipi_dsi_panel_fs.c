/* drivers/video/msm/mipi_dsi_panel_fs.c
 *
 * Copyright (C) [2011] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2; as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_panel_driver.h"

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

	if (mfd->panel_power_on) {
		int curr_pwr_state;

		mfd->op_enable = FALSE;
		curr_pwr_state = mfd->panel_power_on;
		mfd->panel_power_on = FALSE;

		msleep(ONE_FRAME_TRANSMIT_WAIT_MS);
		ret = pdata->off(mfd->pdev);
		if (ret)
			mfd->panel_power_on = curr_pwr_state;

		mfd->op_enable = TRUE;
	}

	if (!mfd->panel_power_on) {
		pdata->on(mfd->pdev);
		if (ret == 0)
			mfd->panel_power_on = TRUE;
	}

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

#ifdef CONFIG_DEBUG_FS

#define MIPI_PANEL_DEBUG_BUF	2048

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

static int mipi_dsi_cmd_seq_open(struct inode *inode,
	struct file *file)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = inode->i_private;
	if (dsi_data->debug_buf != NULL)
		return -EBUSY;
	dsi_data->debug_buf = kzalloc(MIPI_PANEL_DEBUG_BUF, GFP_KERNEL);
	file->private_data = dsi_data;
	/* non-seekable */
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}

static int mipi_dsi_cmd_seq_release(struct inode *inode,
	struct file *file)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = file->private_data;
	kfree(dsi_data->debug_buf);
	dsi_data->debug_buf = NULL;
	return 0;
}

static ssize_t mipi_dsi_cmd_seq_read(struct file *file,
	char __user *buff, size_t count, loff_t *ppos)
{
	char *bp;
	int len = 0;
	int tot = 0;
	int dlen;
	struct mipi_dsi_data *dsi_data;

	if (*ppos)
		return 0;

	dsi_data = file->private_data;

	bp = dsi_data->debug_buf;
	if (bp == NULL)
		return 0;

	dlen = MIPI_PANEL_DEBUG_BUF;

	if (dsi_data->panel) {
		/* show panel info */
		MSNPRINTF(bp, dlen, "Register data for panel %s\n",
			  dsi_data->panel->name);
		MSNPRINTF(bp, dlen, "xres = %d, yres = %d\n",
			  dsi_data->panel_data.panel_info.xres,
			  dsi_data->panel_data.panel_info.yres);
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

static const struct file_operations mipi_dsi_cmd_seq_fops = {
	.open = mipi_dsi_cmd_seq_open,
	.release = mipi_dsi_cmd_seq_release,
	.read = mipi_dsi_cmd_seq_read,
};

void mipi_dsi_debugfs_init(struct platform_device *pdev,
	const char *sub_name)
{
	struct dentry *root;
	struct dentry *file;
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	root = msm_fb_get_debugfs_root();
	if (root != NULL) {
		dsi_data->panel_driver_ic_dir =
			debugfs_create_dir(sub_name, root);

		if (IS_ERR(dsi_data->panel_driver_ic_dir) ||
			(dsi_data->panel_driver_ic_dir == NULL)) {
			dev_err(&pdev->dev,
				"debugfs_create_dir fail, error %ld\n",
				PTR_ERR(dsi_data->panel_driver_ic_dir));
		} else {
			file = debugfs_create_file("cmd_seq", 0444,
				dsi_data->panel_driver_ic_dir, dsi_data,
				&mipi_dsi_cmd_seq_fops);
			if (file == NULL)
				dev_err(&pdev->dev,
					"debugfs_create_file: index fail\n");
		}
	}
}

void mipi_dsi_debugfs_exit(struct platform_device *pdev)
{
	struct mipi_dsi_data *dsi_data;

	dsi_data = platform_get_drvdata(pdev);
	debugfs_remove_recursive(dsi_data->panel_driver_ic_dir);
}

#endif /* CONFIG_DEBUG_FS */
