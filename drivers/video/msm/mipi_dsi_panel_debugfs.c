/* drivers/video/msm/mipi_dsi_panel_debugfs.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Sony Mobile DSI display driver debug fs
 *
 * Author: Johan Olson <johan.olson@sonymobile.com>
 * Author: Joakim Wesslen <joakim.wesslen@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_dsi_panel.h"

static char *res_buf;
static int buf_sz;

#define TMP_BUF_SZ 128
#define MAX_WRITE_DATA 100

enum dbg_cmd_type {
	DCS,
	GEN,
};

static void update_res_buf(char *string)
{
	res_buf = krealloc(res_buf, buf_sz + strnlen(string, TMP_BUF_SZ) + 1,
								GFP_KERNEL);
	if (!res_buf) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		return;
	}

	memcpy(res_buf + buf_sz, string, strnlen(string, TMP_BUF_SZ) + 1);
	buf_sz += strnlen(string, TMP_BUF_SZ); /* Exclude NULL termination */
}

static void reset_res_buf(void)
{
	kfree(res_buf);
	res_buf = NULL;
	buf_sz = 0;
}

static void print_cmds2file(const struct dsi_cmd_desc *dsi, int cmds_size,
				struct seq_file *s)
{
	int i=0, j;

	if (!dsi) {
		seq_printf(s, "---------\n");
		goto exit;
	}

	for (i = 0; i < cmds_size; i++) {
		switch (dsi[i].dtype) {
		case DTYPE_GEN_WRITE:
			seq_printf(s, "DTYPE_GEN_WRITE: ");
			break;
		case DTYPE_GEN_WRITE1:
			seq_printf(s, "DTYPE_GEN_WRITE1: ");
			break;
		case DTYPE_GEN_WRITE2:
			seq_printf(s, "DTYPE_GEN_WRITE2: ");
			break;
		case DTYPE_GEN_LWRITE:
			seq_printf(s, "DTYPE_GEN_LWRITE: ");
			break;
		case DTYPE_GEN_READ:
			seq_printf(s, "DTYPE_GEN_READ: ");
			break;
		case DTYPE_GEN_READ1:
			seq_printf(s, "DTYPE_GEN_READ1: ");
			break;
		case DTYPE_GEN_READ2:
			seq_printf(s, "DTYPE_GEN_READ2: ");
			break;
		case DTYPE_DCS_LWRITE:
			seq_printf(s, "DTYPE_DCS_LWRITE: ");
			break;
		case DTYPE_DCS_WRITE:
			seq_printf(s, "DTYPE_DCS_WRITE: ");
			break;
		case DTYPE_DCS_WRITE1:
			seq_printf(s, "DTYPE_DCS_WRITE1: ");
			break;
		case DTYPE_DCS_READ:
			seq_printf(s, "DTYPE_DCS_READ: ");
			break;
		case DTYPE_MAX_PKTSIZE:
			seq_printf(s, "DTYPE_MAX_PKTSIZE: ");
			break;
		case DTYPE_NULL_PKT:
			seq_printf(s, "DTYPE_NULL_PKT: ");
			break;
		case DTYPE_BLANK_PKT:
			seq_printf(s, "DTYPE_BLANK_PKT: ");
			break;
		case DTYPE_CM_ON:
			seq_printf(s, "DTYPE_CM_ON: ");
			break;
		case DTYPE_CM_OFF:
			seq_printf(s, "DTYPE_CM_OFF: ");
			break;
		case DTYPE_PERIPHERAL_ON:
			seq_printf(s, "DTYPE_PERIPHERAL_ON: ");
			break;
		case DTYPE_PERIPHERAL_OFF:
			seq_printf(s, "DTYPE_PERIPHERAL_OFF: ");
			break;
		default:
			seq_printf(s, "Unknown dtype: 0x%x: ",
						dsi[i].dtype);
		}
		seq_printf(s, "last = %d, vc = %d, ack = %d, ",
				dsi[i].last, dsi[i].vc, dsi[i].ack);
		seq_printf(s, "wait = %d, dlen = %d\npayload: ",
				dsi[i].wait, dsi[i].dlen);

		for (j = 0; j < dsi[i].dlen; j++)
			seq_printf(s, "0x%.2x ", dsi[i].payload[j]);
		seq_printf(s, "\n");
	}
exit:
	return;
}

static int info_show(struct seq_file *s, void *unused)
{
	struct msm_fb_data_type *mfd = s->private;
	struct mipi_dsi_data *dsi_data;

	dev_dbg(&mfd->panel_pdev->dev, "%s\n", __func__);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data || !dsi_data->panel) {
		seq_printf(s, "No panel\n");
		pr_err("%s: no panels\n", __func__);
		return 0;
	}

	if (!dsi_data->panel->name) {
		seq_printf(s, "No panel name\n");
		goto exit;
	}

	seq_printf(s, "Panel: %s\n", dsi_data->panel->name);
	seq_printf(s, "xres = %d, yres = %d\n",
				dsi_data->panel_data.panel_info.xres,
				dsi_data->panel_data.panel_info.yres);
	seq_printf(s, "width = %d mm, height = %d mm\n",
				dsi_data->panel->width,
				dsi_data->panel->height);
	seq_printf(s, "---------\n");

	if (!dsi_data->panel->pctrl) {
		seq_printf(s, "No pctrl\n");
		goto exit;
	}
	seq_printf(s, "display_init commands:\n");
	print_cmds2file(dsi_data->panel->pctrl->display_init_cmds,
			dsi_data->panel->pctrl->display_init_cmds_size, s);
	seq_printf(s, "display_on_eco_cmds commands:\n");
	print_cmds2file(dsi_data->panel->pctrl->display_on_eco_cmds,
			dsi_data->panel->pctrl->display_on_eco_cmds_size, s);
	seq_printf(s, "display_on commands:\n");
	print_cmds2file(dsi_data->panel->pctrl->display_on_cmds,
			dsi_data->panel->pctrl->display_on_cmds_size, s);
	seq_printf(s, "display_off commands:\n");
	print_cmds2file(dsi_data->panel->pctrl->display_off_cmds,
			dsi_data->panel->pctrl->display_off_cmds_size, s);
	seq_printf(s, "read_id commands (size 1):\n");
	print_cmds2file(dsi_data->panel->pctrl->read_id_cmds, 1, s);
	seq_printf(s, "eco_mode_gamma commands:\n");
	print_cmds2file(dsi_data->panel->pctrl->eco_mode_gamma_cmds,
			dsi_data->panel->pctrl->eco_mode_gamma_cmds_size, s);
	seq_printf(s, "normal_gamma commands:\n");
	print_cmds2file(dsi_data->panel->pctrl->normal_gamma_cmds,
			dsi_data->panel->pctrl->normal_gamma_cmds_size, s);
exit:
	return 0;
}

static void print_params(int dtype, u8 reg, int len, u8 *data)
{
	int i = 0;
	char tmp[TMP_BUF_SZ];

	switch (dtype) {
	case DTYPE_GEN_WRITE:
		update_res_buf("GEN_WRITE\n");
		break;
	case DTYPE_GEN_WRITE1:
		update_res_buf("GEN_WRITE1\n");
		break;
	case DTYPE_GEN_WRITE2:
		update_res_buf("GEN_WRITE2\n");
		break;
	case DTYPE_GEN_LWRITE:
		update_res_buf("GEN_LWRITE\n");
		break;
	case DTYPE_GEN_READ:
		update_res_buf("GEN_READ\n");
		break;
	case DTYPE_GEN_READ1:
		update_res_buf("GEN_READ1\n");
		break;
	case DTYPE_GEN_READ2:
		update_res_buf("GEN_READ2\n");
		break;
	case DTYPE_DCS_LWRITE:
		update_res_buf("DCS_LWRITE\n");
		break;
	case DTYPE_DCS_WRITE:
		update_res_buf("DCS_WRITE\n");
		break;
	case DTYPE_DCS_WRITE1:
		update_res_buf("DCS_WRITE1\n");
		break;
	case DTYPE_DCS_READ:
		update_res_buf("DCS_READ\n");
		break;
	default:
		snprintf(tmp, sizeof(tmp), "Unknown dtype = 0x%x\n", dtype);
		update_res_buf(tmp);
	}

	if (len > 0) {
		snprintf(tmp, sizeof(tmp), "reg=0x%.2X\n", reg);
		update_res_buf(tmp);
		snprintf(tmp, sizeof(tmp), "len=%d\n", len);
		update_res_buf(tmp);
		for (i = 0; i < len; i++) {
			snprintf(tmp, sizeof(tmp), "data[%d]=0x%.2X\n", i,
								data[i]);
			update_res_buf(tmp);
		}
	} else {
		update_res_buf("Something went wrong, length is zero.\n");
		snprintf(tmp, sizeof(tmp),
				"reg=0x%.2X, len=%d, data[0]=0x%.2X\n",
				reg, len, data[0]);
		update_res_buf(tmp);
	}
}

static int setup_reg_access(struct device *dev, struct dsi_buf *rx_buf,
			    struct dsi_buf *tx_buf, char **buf,
			    const char __user *ubuf, size_t count)
{
	int ret = 0;

	reset_res_buf();

	if (rx_buf) {
		ret = mipi_dsi_buf_alloc(rx_buf, DSI_BUF_SIZE);
		if (ret <= 0) {
			dev_err(dev, "mipi_dsi_buf_alloc(rx) failed!\n");
			ret = -ENOMEM;
			goto exit;
		}
	}

	if (tx_buf) {
		ret = mipi_dsi_buf_alloc(tx_buf, DSI_BUF_SIZE);
		if (ret <= 0) {
			dev_err(dev, "mipi_dsi_buf_alloc(tx) failed!\n");
			ret = -ENOMEM;
			goto fail_free_rx;
		}
	}

	*buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!*buf) {
		dev_err(dev, "%s: Failed to allocate buffer\n", __func__);
		ret = -ENOMEM;
		goto fail_free_all;
	}

	if (copy_from_user(*buf, ubuf, count)) {
		ret = -EFAULT;
		goto fail_free_all;
	}
	return 0;

fail_free_all:
	if (tx_buf)
		mipi_dsi_buf_release(tx_buf);
	kfree(*buf);
fail_free_rx:
	if (rx_buf)
		mipi_dsi_buf_release(rx_buf);
exit:
	return ret;
}

static int get_cmd_type(char *buf, enum dbg_cmd_type *cmd)
{
	int ret = 0;

	if (!strncmp(buf, "dcs", 3))
		*cmd = DCS;
	else if (!strncmp(buf, "gen", 3))
		*cmd = GEN;
	else
		ret = -EFAULT;
	return ret;
}

static int get_parameters(const char *p, u8 *par_buf, int par_buf_size,
								int *nbr_params)
{
	int ret = 0;

	while (true) {
		if (isspace(*p)) {
			p++;
		} else {
			if (sscanf(p, "%4hhx", &par_buf[*nbr_params]) == 1) {
				(*nbr_params)++;
				while (isxdigit(*p) || (*p == 'x')) {
					p++;
				}
			}
		}
		if (*nbr_params > par_buf_size) {
			update_res_buf("Too many parameters\n");
			ret = -EINVAL;
			goto exit;
		}
		if (iscntrl(*p))
			break;
	}
exit:
	return ret;
}

static ssize_t reg_read(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	struct device *dev;
	u8 params[3]; /* No more than reg + two parameters is allowed */
	char *buf;
	const char *p;
	int ret;
	int nbr_bytes_to_read;
	int i;
	int j;
	enum dbg_cmd_type cmd;
	struct dsi_buf tx_buf;
	struct dsi_buf rx_buf;
	struct dsi_cmd_desc dsi;

	dev = &mfd->panel_pdev->dev;
	dev_dbg(dev, "%s\n", __func__);

	ret = setup_reg_access(dev, &rx_buf, &tx_buf, &buf, ubuf, count);
	if (ret)
		goto exit;

	ret = get_cmd_type(buf, &cmd);
	if (ret) {
		update_res_buf("Read - unknown type\n");
		goto fail_free_all;
	}

	p = buf;
	p = p+4;

	/* Get nbr_bytes_to_read */
	if (sscanf(p, "%d", &nbr_bytes_to_read) != 1) {
		update_res_buf("Read - parameter error\n");
		ret = -EINVAL;
		goto fail_free_all;
	}

	while (isxdigit(*p) || (*p == 'x'))
		p++;

	dev_dbg(dev, "nbr_bytes_to_read = %d\n", nbr_bytes_to_read);
	i = 0;

	ret = get_parameters(p, params, ARRAY_SIZE(params), &i);
	if (ret)
		goto fail_free_all;

	ret = prepare_for_reg_access(mfd);
	if (ret)
		goto fail_free_all;

	if (cmd == DCS) {
		dsi.dtype = DTYPE_DCS_READ;
	} else {
		if (i == 1) { /* 0 parameters */
			dsi.dtype = DTYPE_GEN_READ;
		} else if (i == 2) { /* 1 parameter */
			dsi.dtype = DTYPE_GEN_READ1;
		} else { /* 2 paramters */
			dsi.dtype = DTYPE_GEN_READ2;
		}
	}
	dsi.last = 1;
	dsi.vc = 0;
	dsi.ack = 1;
	dsi.wait = 5; /* why 5? */
	dsi.dlen = i;
	dsi.payload = params;

	dev_dbg(dev, "dtype = %d, last = %d, vc = %d, ack = %d, wait = %d, "
		"dlen = %d\n", dsi.dtype, dsi.last, dsi.vc, dsi.ack, dsi.wait,
		dsi.dlen);
	for (j = 0; j < i; j++)
		dev_dbg(dev, "payload[%d] = 0x%x\n", j, dsi.payload[j]);

	mipi_dsi_cmds_rx(mfd, &tx_buf, &rx_buf, &dsi, nbr_bytes_to_read);
	post_reg_access(mfd);
	print_params(dsi.dtype, params[0], rx_buf.len, rx_buf.data);

fail_free_all:
	kfree(buf);
	mipi_dsi_buf_release(&rx_buf);
	mipi_dsi_buf_release(&tx_buf);
exit:
	return count;
}

static ssize_t reg_write(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	struct device *dev;
	char *buf;
	const char *p;
	enum dbg_cmd_type cmd;
	u8 data[MAX_WRITE_DATA];
	int i = 0;
	int j;
	int ret;
	struct dsi_buf tx_buf;
	struct dsi_cmd_desc dsi;

	dev = &mfd->panel_pdev->dev;
	dev_dbg(dev, "%s\n", __func__);
	ret = setup_reg_access(dev, NULL, &tx_buf, &buf, ubuf, count);
	if (ret)
		goto exit;

	ret = get_cmd_type(buf, &cmd);
	if (ret) {
		update_res_buf("Write - unknown type\n");
		goto fail_free_all;
	}

	p = buf;
	p = p+4;

	/* Get first param, Register */
	if (sscanf(p, "%4hhx", &data[0]) != 1) {
		update_res_buf("Write - parameter error\n");
		ret = -EINVAL;
		goto fail_free_all;
	}
	i++;

	while (isxdigit(*p) || (*p == 'x'))
		p++;

	ret = get_parameters(p, data, ARRAY_SIZE(data) - 1, &i);
	if (ret)
		goto fail_free_all;

	ret = prepare_for_reg_access(mfd);
	if (ret)
		goto fail_free_all;

	if (cmd == DCS) {
		if (i == 1) { /* 0 parameters */
			dsi.dtype = DTYPE_DCS_WRITE;
		} else if (i == 2) { /* 1 parameter */
			dsi.dtype = DTYPE_DCS_WRITE1;
		} else { /* Many parameters */
			dsi.dtype = DTYPE_DCS_LWRITE;
		}
	} else {
		if (i == 1) { /* 0 parameters */
			dsi.dtype = DTYPE_GEN_WRITE;
		} else if (i == 2) { /* 1 parameter */
			dsi.dtype = DTYPE_GEN_WRITE1;
		} else if (i == 3) { /* 2 parameters */
			dsi.dtype = DTYPE_GEN_WRITE2;
		} else { /* Many parameters */
			dsi.dtype = DTYPE_GEN_LWRITE;
		}
	}
	dsi.last = 1;
	dsi.vc = 0;
	dsi.ack = 0;
	dsi.wait = 0;
	dsi.dlen = i;
	dsi.payload = data;

	dev_dbg(dev, "last = %d, vc = %d, ack = %d, wait = %d, dlen = %d\n",
		dsi.last, dsi.vc, dsi.ack, dsi.wait, dsi.dlen);
	for (j = 0; j < i; j++)
		dev_dbg(dev, "payload[%d] = 0x%x\n", j, dsi.payload[j]);
	mipi_dsi_cmds_tx(&tx_buf, &dsi, 1);

	post_reg_access(mfd);
	print_params(dsi.dtype, data[0], i, dsi.payload);

fail_free_all:
	kfree(buf);
	mipi_dsi_buf_release(&tx_buf);
exit:
	return count;
}

static int panels_show(struct seq_file *s, void *unused)
{
	struct msm_fb_data_type *mfd = s->private;
	int n = 0;
	struct mipi_dsi_data *dsi_data;

	dev_dbg(&mfd->panel_pdev->dev, "%s\n", __func__);
	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data || !dsi_data->panels) {
		seq_printf(s, "No panel platform data\n");
		pr_err("%s: no panels\n", __func__);
		return 0;
	}

	seq_printf(s, "Supported display panels:\n");
	while (dsi_data->panels[n] != NULL) {
		seq_printf(s, "DSI panel[%d]=%s\n", n,
						dsi_data->panels[n]->name);
		n++;
	}
	return 0;
}

static int result_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s", res_buf);
	if (!res_buf)
		seq_printf(s, "\n");
	return 0;
}

#ifdef CONFIG_FB_MSM_RECOVER_PANEL
static int recover_nvm_write(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	struct device *dev;
	int ret = 0;
	char *buf;
	struct mipi_dsi_data *dsi_data;
	const char *p;
	u8 data[MAX_WRITE_DATA];
	int i = 0, j;
	struct dsi_nvm_rewrite_ctl *pnvrw_ctl;

	reset_res_buf();
	dsi_data = platform_get_drvdata(mfd->panel_pdev);

	dev = &mfd->panel_pdev->dev;

	if (!dsi_data && !dsi_data->panel && !dsi_data->panel->pnvrw_ctl) {
		ret = -EFAULT;
		goto exit;
	}
	pnvrw_ctl = dsi_data->panel->pnvrw_ctl;

	buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto exit;
	}

	if (copy_from_user(buf, ubuf, count)) {
		ret = -EFAULT;
		goto fail_free;
	}

	p = buf;
	ret = get_parameters(p, data, ARRAY_SIZE(data) - 1, &i);
	if (ret)
		goto fail_free;

	if (i != 0) {
		if (i == pnvrw_ctl->nvm_e7_nbr_params + 1) {
			if (data[0] != 0xE7) {
				update_res_buf("recover_nvm, param not E7\n");
				ret = -EINVAL;
				goto fail_free;
			}
			/* override E7 register data */
			for (j = 0; j < pnvrw_ctl->nvm_e7_nbr_params; j++)
				pnvrw_ctl->nvm_e7[j + 1] = data[j + 1];
		} else if (i == pnvrw_ctl->nvm_de_nbr_params + 1) {
			if (data[0] != 0xDE) {
				update_res_buf("recover_nvm, param not DE\n");
				ret = -EINVAL;
				goto fail_free;
			}
			/* override DE register data */
			for (j = 0; j < pnvrw_ctl->nvm_de_nbr_params; j++)
				pnvrw_ctl->nvm_de[j + 1] = data[j + 1];
		} else if (i == pnvrw_ctl->nvm_e7_nbr_params +
					pnvrw_ctl->nvm_de_nbr_params + 2) {
			if ((data[pnvrw_ctl->nvm_e7_nbr_params + 1] != 0xDE) &&
							(data[0] != 0xE7)) {
				update_res_buf("recover_nvm, param error\n");
				ret = -EINVAL;
				goto fail_free;
			}
			/* override E7 register data */
			for (j = 0; j < pnvrw_ctl->nvm_e7_nbr_params; j++)
				pnvrw_ctl->nvm_e7[j + 1] = data[j + 1];
			/* override DE register data */
			for (j = 0; j < pnvrw_ctl->nvm_de_nbr_params; j++)
				pnvrw_ctl->nvm_de[j + 1] =
					data[1 + pnvrw_ctl->nvm_e7_nbr_params +
									1 + j];
		} else {
			update_res_buf("recover_nvm, wrong nbr of params\n");
			ret = -EINVAL;
			goto fail_free;
		}
	}

	ret = prepare_for_reg_access(mfd);
	if (ret)
		goto fail_free;

	mipi_set_tx_power_mode(1);

	if (dsi_data->nvm_erase_all)
		if (dsi_data->nvm_erase_all(mfd))
			ret = -1;
	if (dsi_data->nvm_write_trim_area)
		if (dsi_data->nvm_write_trim_area(mfd))
			ret = -1;
	if (dsi_data->nvm_write_user_area)
		if (dsi_data->nvm_write_user_area(mfd))
			ret = -1;

	if (dsi_data->panel_data.on)
		dsi_data->panel_data.on(mfd->pdev);

	post_reg_access(mfd);
fail_free:
	kfree(buf);
exit:
	if (ret)
		update_res_buf("recover_nvm failed\n");
	else
		update_res_buf("recover_nvm sucess\n");
	return count;
}

static int break_nvm_write(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	int ret = 0;
	struct mipi_dsi_data *dsi_data;

	reset_res_buf();
	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data) {
		ret = -EFAULT;
		goto exit;
	}

	ret = prepare_for_reg_access(mfd);
	if (ret)
		goto exit;

	mipi_set_tx_power_mode(1);

	if (dsi_data->nvm_erase_all) {
		ret = dsi_data->nvm_erase_all(mfd);
	}
	post_reg_access(mfd);
	dsi_data->lcd_reset(1);
	dsi_data->lcd_power(1);
exit:
	if (ret)
		update_res_buf("break_nvm failed\n");
	else
		update_res_buf("break_nvm sucess\n");
	return count;
}
#endif /* CONFIG_FB_MSM_RECOVER_PANEL */

static int info_open(struct inode *inode, struct file *file)
{
	return single_open(file, info_show, inode->i_private);
}

static const struct file_operations info_fops = {
	.owner		= THIS_MODULE,
	.open		= info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int read_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations read_fops = {
	.owner			= THIS_MODULE,
	.open			= read_open,
	.write			= reg_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int write_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations write_fops = {
	.owner			= THIS_MODULE,
	.open			= write_open,
	.write			= reg_write,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int panels_open(struct inode *inode, struct file *file)
{
	return single_open(file, panels_show, inode->i_private);
}

static const struct file_operations panels_fops = {
	.owner		= THIS_MODULE,
	.open		= panels_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int result_open(struct inode *inode, struct file *file)
{
	return single_open(file, result_show, inode->i_private);
}

static const struct file_operations result_fops = {
	.owner		= THIS_MODULE,
	.open		= result_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_FB_MSM_RECOVER_PANEL

static int recover_nvm_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations recover_nvm_fops = {
	.owner			= THIS_MODULE,
	.open			= recover_nvm_open,
	.write			= recover_nvm_write,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int break_nvm_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, inode->i_private);
}

static const struct file_operations break_nvm_fops = {
	.owner			= THIS_MODULE,
	.open			= break_nvm_open,
	.write			= break_nvm_write,
	.llseek			= seq_lseek,
	.release		= single_release,
};
#endif /* CONFIG_FB_MSM_RECOVER_PANEL */

void __devinit mipi_dsi_panel_create_debugfs(struct platform_device *pdev,
						const char *sub_name)
{
	struct mipi_dsi_data *dsi_data;
	struct device *dev;
	struct msm_fb_data_type *mfd;
	struct dentry *root;

	if (!pdev) {
		pr_err("%s: no device\n", __func__);
		return;
	}

	mfd = platform_get_drvdata(pdev);
	if (!mfd) {
		pr_err("%s: no mfd\n", __func__);
		return;
	}

	if (!mfd->panel_pdev) {
		pr_err("%s: no panel device\n", __func__);
		return;
	}
	dev = &mfd->panel_pdev->dev;

	dsi_data = platform_get_drvdata(mfd->panel_pdev);
	if (!dsi_data) {
		pr_err("%s: no dsi_data\n", __func__);
		return;
	}
	if (!&dev->kobj) {
		pr_err("%s: no &dev->kobj\n", __func__);
		return;
	}

	root = msm_fb_get_debugfs_root();
	dsi_data->panel_driver_ic_dir = debugfs_create_dir(sub_name, root);
	if (!dsi_data->panel_driver_ic_dir) {
		dev_err(dev, "%s: dbgfs create dir failed\n", __func__);
	} else {
		if (!debugfs_create_file("info", S_IRUGO,
					dsi_data->panel_driver_ic_dir, mfd,
					&info_fops)) {
			dev_err(dev, "%s: failed to create dbgfs info file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("read", S_IWUSR,
					dsi_data->panel_driver_ic_dir, mfd,
					&read_fops)) {
			dev_err(dev, "%s: failed to create dbgfs read file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("write", S_IWUSR,
					dsi_data->panel_driver_ic_dir, mfd,
					&write_fops)) {
			dev_err(dev, "%s: failed to create dbgfs write file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("panels", S_IRUGO,
					dsi_data->panel_driver_ic_dir, mfd,
					&panels_fops)) {
			dev_err(dev, "%s: failed to create dbgfs panels file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("result", S_IRUGO,
					dsi_data->panel_driver_ic_dir, mfd,
					&result_fops)) {
			dev_err(dev, "%s: failed to create dbgfs result file\n",
								__func__);
			return;
		}
#ifdef CONFIG_FB_MSM_RECOVER_PANEL
		if (!debugfs_create_file("recover_nvm", S_IWUSR,
					dsi_data->panel_driver_ic_dir, mfd,
					&recover_nvm_fops)) {
			dev_err(&pdev->dev,
				"%s: failed to create dbgfs recover_nvm\n",
				__func__);
			return;
		}
		if (!debugfs_create_file("break_nvm", S_IWUSR,
					dsi_data->panel_driver_ic_dir, mfd,
					&break_nvm_fops)) {
			dev_err(&pdev->dev,
				"%s: failed to create dbgfs break_nvm\n",
				__func__);
			return;
		}
#endif
	}
}

void __devexit mipi_dsi_panel_remove_debugfs(struct platform_device *pdev)
{
	struct mipi_dsi_data *dsi_data;

	if (!pdev || !&pdev->dev) {
		pr_err("%s: no device\n", __func__);
		return;
	}

	dsi_data = platform_get_drvdata(pdev);
	if (!dsi_data) {
		pr_err("%s: no dsi_data\n", __func__);
		return;
	}
	debugfs_remove_recursive(dsi_data->panel_driver_ic_dir);
}
