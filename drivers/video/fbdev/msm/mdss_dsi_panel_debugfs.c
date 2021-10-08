/*
 * drivers/video/fbdev/msm/mdss_dsi_panel_debugfs.c
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

#include <linux/incell.h>
#include <linux/fb.h>

#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_dsi.h"
#include "mdss_dsi_panel_driver.h"
#include "mdss_dsi_panel_debugfs.h"

struct device virtdev;
struct device incelldev;

static char *res_buf;
static int buf_sz;

static struct dsi_buf debug_tx_buf;
static struct dsi_buf debug_rx_buf;

static struct blackscreen_det bs_det;
static struct first_frame_flushed_det fff_det;

#define DSI_BUF_SIZE	1024
#define TMP_BUF_SZ 128
#define MAX_WRITE_DATA 100

#define DEFAULT_BS_THRESHOLD 500

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
	kzfree(res_buf);
	res_buf = NULL;
	buf_sz = 0;
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

static int setup_reg_access(char **buf, const char __user *ubuf, size_t count)
{
	int ret = 0;

	reset_res_buf();

	*buf = kzalloc(sizeof(char) * count, GFP_KERNEL);
	if (!*buf) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (copy_from_user(*buf, ubuf, count)) {
		ret = -EFAULT;
		goto exit;
	}
	return 0;

exit:
	kzfree(*buf);
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
				while (isxdigit(*p) || (*p == 'x'))
					p++;
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

static u32 panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_cmd_desc *cmds, void (*fxn)(int),
		char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

static void panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_cmd_desc *cmds, int cmd_cnt,
			int link_state)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmds;
	cmdreq.cmds_cnt = cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/* Panel ON/Off commands should be sent in DSI Low Power Mode */
	if (link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
}

static ssize_t reg_read(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	struct mdss_panel_data *pdata;
	struct mdss_mdp_ctl *ctl;
	u8 params[3]; /* No more than reg + two parameters is allowed */
	char *buf, *rbuf;
	const char *p;
	int ret;
	int nbr_bytes_to_read;
	int i;
	int j;
	enum dbg_cmd_type cmd;
	struct dsi_cmd_desc dsi;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (!mfd->panel_power_state) {
		pr_err("%s: panel is NOT on\n", __func__);
		goto exit;
	}

	ctl = mfd_to_ctl(mfd);
	if (!ctl)
		goto exit;

	if (mutex_lock_interruptible(&ctl->lock))
		goto exit;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("%s: no panel connected\n", __func__);
		goto exit;
	}

	ctrl_pdata = mdss_dsi_get_master_ctrl(pdata);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		goto exit;
	}

	pr_err("%s\n", __func__);

	ret = setup_reg_access(&buf, ubuf, count);
	if (ret)
		goto exit;

	ret = get_cmd_type(buf, &cmd);
	if (ret) {
		update_res_buf("Read - unknown type\n");
		goto fail_free_buf;
	}

	p = buf;
	p = p+4;

	/* Get nbr_bytes_to_read */
	if (sscanf(p, "%d", &nbr_bytes_to_read) != 1) {
		update_res_buf("Read - parameter error\n");
		goto fail_free_buf;
	}

	while (isxdigit(*p) || (*p == 'x'))
		p++;

	pr_err("%s: nbr_bytes_to_read = %d\n", __func__, nbr_bytes_to_read);
	i = 0;

	ret = get_parameters(p, params, ARRAY_SIZE(params), &i);
	if (ret)
		goto fail_free_buf;

	if (cmd == DCS) {
		dsi.dchdr.dtype = DTYPE_DCS_READ;
	} else {
		if (i == 1) { /* 0 parameters */
			dsi.dchdr.dtype = DTYPE_GEN_READ;
		} else if (i == 2) { /* 1 parameter */
			dsi.dchdr.dtype = DTYPE_GEN_READ1;
		} else { /* 2 paramters */
			dsi.dchdr.dtype = DTYPE_GEN_READ2;
		}
	}
	dsi.dchdr.last = 1;
	dsi.dchdr.vc = 0;
	dsi.dchdr.ack = 1;
	dsi.dchdr.wait = 5;
	dsi.dchdr.dlen = i;
	dsi.payload = params;

	pr_err("%s: dtype=%d, last=%d, vc=%d, ack=%d, wait=%d, dlen=%d\n",
		__func__,
		dsi.dchdr.dtype, dsi.dchdr.last, dsi.dchdr.vc, dsi.dchdr.ack,
		dsi.dchdr.wait, dsi.dchdr.dlen);
	for (j = 0; j < i; j++)
		pr_err("%s: payload[%d] = 0x%x\n",
			__func__, j, dsi.payload[j]);

	rbuf = kzalloc(sizeof(char) * nbr_bytes_to_read, GFP_KERNEL);
	if (!rbuf) {
		pr_err("%s: Failed to allocate buffer\n", __func__);
		goto fail_free_buf;
	}

	panel_cmd_read(ctrl_pdata, &dsi, NULL, rbuf, nbr_bytes_to_read);

	mutex_unlock(&ctl->lock);

	print_params(dsi.dchdr.dtype, params[0], nbr_bytes_to_read,
			rbuf);

	if (rbuf)
		kzfree(rbuf);
fail_free_buf:
	if (buf)
		kzfree(buf);
exit:
	return count;
}

static ssize_t reg_write(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	struct mdss_panel_data *pdata;
	struct mdss_mdp_ctl *ctl;
	char *buf;
	const char *p;
	enum dbg_cmd_type cmd;
	u8 data[MAX_WRITE_DATA];
	int i = 0;
	int j;
	int ret;
	struct dsi_cmd_desc dsi;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (!mfd->panel_power_state) {
		pr_err("%s: panel is NOT on\n", __func__);
		goto exit;
	}

	ctl = mfd_to_ctl(mfd);
	if (!ctl)
		goto exit;

	if (mutex_lock_interruptible(&ctl->lock))
		goto exit;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("%s: no panel connected\n", __func__);
		goto exit;
	}

	ctrl_pdata = mdss_dsi_get_master_ctrl(pdata);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		goto exit;
	}

	pr_err("%s\n", __func__);
	ret = setup_reg_access(&buf, ubuf, count);
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
		goto fail_free_all;
	}
	i++;

	while (isxdigit(*p) || (*p == 'x'))
		p++;

	ret = get_parameters(p, data, ARRAY_SIZE(data) - 1, &i);
	if (ret)
		goto fail_free_all;

	if (cmd == DCS) {
		if (i == 1) { /* 0 parameters */
			dsi.dchdr.dtype = DTYPE_DCS_WRITE;
		} else if (i == 2) { /* 1 parameter */
			dsi.dchdr.dtype = DTYPE_DCS_WRITE1;
		} else { /* Many parameters */
			dsi.dchdr.dtype = DTYPE_DCS_LWRITE;
		}
	} else {
		if (i == 1) { /* 0 parameters */
			dsi.dchdr.dtype = DTYPE_GEN_WRITE;
		} else if (i == 2) { /* 1 parameter */
			dsi.dchdr.dtype = DTYPE_GEN_WRITE1;
		} else if (i == 3) { /* 2 parameters */
			dsi.dchdr.dtype = DTYPE_GEN_WRITE2;
		} else { /* Many parameters */
			dsi.dchdr.dtype = DTYPE_GEN_LWRITE;
		}
	}
	dsi.dchdr.last = 1;
	dsi.dchdr.vc = 0;
	dsi.dchdr.ack = 0;
	dsi.dchdr.wait = 0;
	dsi.dchdr.dlen = i;
	dsi.payload = data;

	pr_err("%s: last = %d, vc = %d, ack = %d, wait = %d, dlen = %d\n",
		__func__,
		dsi.dchdr.last, dsi.dchdr.vc, dsi.dchdr.ack, dsi.dchdr.wait,
		dsi.dchdr.dlen);
	for (j = 0; j < i; j++)
		pr_err("%s: payload[%d] = 0x%x\n",
			__func__, j, dsi.payload[j]);

	panel_cmds_send(ctrl_pdata, &dsi, 1, DSI_LP_MODE);

	print_params(dsi.dchdr.dtype, data[0], i, dsi.payload);

	mutex_unlock(&ctl->lock);

fail_free_all:
	kzfree(buf);
exit:
	return count;
}

static int result_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s", res_buf);
	if (!res_buf)
		seq_printf(s, "\n");
	return 0;
}

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

static ssize_t incell_panel_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	incell_pw_status power_status;
	bool display = false;
	bool touch = false;

	power_status.touch_power = false;
	power_status.display_power = false;
	ret = incell_get_power_status(&power_status);

	if (ret == INCELL_OK) {
		display = power_status.display_power;
		touch = power_status.touch_power;
	} else {
		return scnprintf(buf, PAGE_SIZE, "%s\n", "Get failed");
	}

	return scnprintf(buf, PAGE_SIZE, "touch = %d, display = %d\n",
				touch, display);
}

static ssize_t incell_intf_type_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = INCELL_OK, mode;
	incell_intf_mode intf_mode;

	rc = kstrtoint(buf, 10, &mode);
	if (rc < 0) {
		pr_err("%s: Error, buf = %s\n", __func__, buf);
		return rc;
	}

	intf_mode = (incell_intf_mode)(mode);
	pr_notice("%s: buf = %s\n", __func__, buf);
	rc = incell_control_mode(intf_mode, INCELL_FORCE);
	pr_notice("%s: Incell execution %s\n", __func__,
			rc == INCELL_OK ? "suscess" : "fail");
	pr_notice("%s: return %d\n", __func__, rc);
	return count;
}

static ssize_t incell_power_lock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = INCELL_OK, mode;
	incell_pw_lock intf_lock;
	incell_pw_status power_status;

	rc = kstrtoint(buf, 10, &mode);
	if (rc < 0) {
		pr_err("%s: Error, buf = %s\n", __func__, buf);
		return rc;
	}

	intf_lock = (incell_pw_lock)(mode);
	pr_notice("%s: buf = %s\n", __func__, buf);
	rc = incell_power_lock_ctrl(mode, &power_status);
	pr_notice("%s: Incell execution %s\n", __func__,
			rc == INCELL_OK ? "suscess" : "fail");
	pr_notice("%s: return %d\n", __func__, rc);
	return count;
}

static ssize_t incell_ewu_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = INCELL_OK, mode;
	incell_ewu_mode intf_ewu;

	rc = kstrtoint(buf, 10, &mode);
	if (rc < 0) {
		pr_err("%s: Error, buf = %s\n", __func__, buf);
		return rc;
	}

	intf_ewu = (incell_ewu_mode)(mode);
	pr_notice("%s: buf = %s\n", __func__, buf);
	incell_ewu_mode_ctrl(intf_ewu);
	pr_notice("%s: Incell execution %s\n", __func__,
			rc == INCELL_OK ? "suscess" : "fail");
	pr_notice("%s: return %d\n", __func__, rc);
	return count;
}

static struct device_attribute incell_attributes[] = {
	__ATTR(incell_status, S_IRUGO, incell_panel_status_show, NULL),
	__ATTR(incell_intf, S_IWUSR|S_IWGRP, NULL, incell_intf_type_store),
	__ATTR(incell_lock, S_IWUSR|S_IWGRP, NULL, incell_power_lock_store),
	__ATTR(incell_ewu, S_IWUSR|S_IWGRP, NULL, incell_ewu_store),
};

static int incell_register_attributes(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(incell_attributes); i++)
		if (device_create_file(dev, incell_attributes + i))
			goto err;

	return 0;

err:
	dev_err(dev, "%s: Unable to create interface\n", __func__);
	for (--i; i >= 0 ; i--)
		device_remove_file(dev, incell_attributes + i);

	return -ENODEV;
}

static int incell_create_debugfs(struct msm_fb_data_type *mfd)
{
	int rc = 0;
	char *path_name = "incell_dsi_panel";
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_mdp_ctl *ctl = mdata->ctl_off;
	struct mdss_panel_data *pdata = ctl->panel_data;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(pdata,
				struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	dev_set_name(&incelldev, "%s", path_name);
	rc = device_register(&incelldev);
	if (rc) {
		pr_err("%s: device_register rc = %d\n", __func__, rc);
		goto out;
	}

	rc = incell_register_attributes(&incelldev);
	if (rc) {
		pr_err("%s: register_attributes rc = %d\n", __func__, rc);
		goto err;
	}

	dev_set_drvdata(&incelldev, ctrl_pdata);
	goto out;

err:
	device_unregister(&incelldev);
out:
	return rc;
}

static ssize_t mdss_dsi_panel_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	char const *id = ctrl_pdata->panel_data.panel_info.panel_id_name ?
		ctrl_pdata->panel_data.panel_info.panel_id_name : "default";

	return scnprintf(buf, PAGE_SIZE, "%s\n", id);
}

static ssize_t mdss_dsi_panel_pcc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	struct mdss_pcc_data *pcc_data;
	u32 r, g, b;

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	pcc_data = &ctrl_pdata->spec_pdata->pcc_data;

	r = g = b = 0;
	if (!pcc_data->color_tbl) {
		pr_err("%s: Panel has no color table\n", __func__);
		goto exit;
	}
	if (pcc_data->u_data == 0 && pcc_data->v_data == 0) {
		pr_err("%s: u,v are 0.\n", __func__);
		goto exit;
	}
	if (pcc_data->tbl_idx >= pcc_data->tbl_size) {
		pr_err("%s: Invalid color area(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	if (pcc_data->color_tbl[pcc_data->tbl_idx].color_type == UNUSED) {
		pr_err("%s: Unsupported color type(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	r = pcc_data->color_tbl[pcc_data->tbl_idx].r_data;
	g = pcc_data->color_tbl[pcc_data->tbl_idx].g_data;
	b = pcc_data->color_tbl[pcc_data->tbl_idx].b_data;
exit:
	return scnprintf(buf, PAGE_SIZE, "0x%x 0x%x 0x%x ", r, g, b);
}

static ssize_t mdss_dsi_panel_srgb_pcc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	struct mdss_pcc_data *pcc_data;
	u32 r, g, b;

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	pcc_data = &ctrl_pdata->spec_pdata->srgb_pcc_data;

	r = g = b = 0;
	if (!pcc_data->color_tbl) {
		pr_err("%s: Panel has no color table\n", __func__);
		goto exit;
	}
	if (pcc_data->u_data == 0 && pcc_data->v_data == 0) {
		pr_err("%s: u,v are 0.\n", __func__);
		goto exit;
	}
	if (pcc_data->tbl_idx >= pcc_data->tbl_size) {
		pr_err("%s: Invalid color area(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	if (pcc_data->color_tbl[pcc_data->tbl_idx].color_type == UNUSED) {
		pr_err("%s: Unsupported color type(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	r = pcc_data->color_tbl[pcc_data->tbl_idx].r_data;
	g = pcc_data->color_tbl[pcc_data->tbl_idx].g_data;
	b = pcc_data->color_tbl[pcc_data->tbl_idx].b_data;
exit:
	return scnprintf(buf, PAGE_SIZE, "0x%x 0x%x 0x%x ", r, g, b);
}

static ssize_t mdss_dsi_panel_vivid_pcc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	struct mdss_pcc_data *pcc_data;
	u32 r, g, b;

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	pcc_data = &ctrl_pdata->spec_pdata->vivid_pcc_data;

	r = g = b = 0;
	if (!pcc_data->color_tbl) {
		pr_err("%s: Panel has no color table\n", __func__);
		goto exit;
	}
	if (pcc_data->u_data == 0 && pcc_data->v_data == 0) {
		pr_err("%s: u,v are 0.\n", __func__);
		goto exit;
	}
	if (pcc_data->tbl_idx >= pcc_data->tbl_size) {
		pr_err("%s: Invalid color area(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	if (pcc_data->color_tbl[pcc_data->tbl_idx].color_type == UNUSED) {
		pr_err("%s: Unsupported color type(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	r = pcc_data->color_tbl[pcc_data->tbl_idx].r_data;
	g = pcc_data->color_tbl[pcc_data->tbl_idx].g_data;
	b = pcc_data->color_tbl[pcc_data->tbl_idx].b_data;
exit:
	return scnprintf(buf, PAGE_SIZE, "0x%x 0x%x 0x%x ", r, g, b);
}

static ssize_t mdss_dsi_panel_hdr_pcc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	struct mdss_pcc_data *pcc_data;
	u32 r, g, b;

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	pcc_data = &ctrl_pdata->spec_pdata->hdr_pcc_data;

	r = g = b = 0;
	if (!pcc_data->color_tbl) {
		pr_err("%s: Panel has no color table\n", __func__);
		goto exit;
	}
	if (pcc_data->u_data == 0 && pcc_data->v_data == 0) {
		pr_err("%s: u,v are 0.\n", __func__);
		goto exit;
	}
	if (pcc_data->tbl_idx >= pcc_data->tbl_size) {
		pr_err("%s: Invalid color area(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	if (pcc_data->color_tbl[pcc_data->tbl_idx].color_type == UNUSED) {
		pr_err("%s: Unsupported color type(idx=%d)\n",
			__func__, pcc_data->tbl_idx);
		goto exit;
	}
	r = pcc_data->color_tbl[pcc_data->tbl_idx].r_data;
	g = pcc_data->color_tbl[pcc_data->tbl_idx].g_data;
	b = pcc_data->color_tbl[pcc_data->tbl_idx].b_data;
exit:
	return scnprintf(buf, PAGE_SIZE, "0x%x 0x%x 0x%x ", r, g, b);
}

static ssize_t mdss_dsi_panel_color_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_panel_specific_pdata *spec_pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	spec_pdata = ctrl_pdata->spec_pdata;

	return scnprintf(buf, PAGE_SIZE, "%d", spec_pdata->color_mode);
}

static ssize_t mdss_dsi_panel_color_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdss_panel_specific_pdata *spec_pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);
	int mode, rc = 0;

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	spec_pdata = ctrl_pdata->spec_pdata;

	if (sscanf(buf, "%d", &mode) < 0) {
		pr_err("sscanf failed to set mode. keep current mode=%d\n",
			spec_pdata->color_mode);
		rc = -EINVAL;
		goto exit;
	}

	switch (mode) {
	case CLR_MODE_SELECT_SRGB:
	case CLR_MODE_SELECT_DCIP3:
	case CLR_MODE_SELECT_PANELNATIVE:
		spec_pdata->color_mode = mode;
		break;
	default:
		pr_err("failed set mode:invalid mode=%d. keep current mode"
			"=%d\n", mode, spec_pdata->color_mode);
		rc = -EINVAL;
		break;
	}

exit:
	return !rc ? count : rc;

}

static ssize_t mdss_dsi_panel_frame_counter(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fps_data fpsd = mdss_dsi_panel_driver_get_fps_data();

	return scnprintf(buf, PAGE_SIZE, "%i\n", fpsd.frame_counter);
}

static ssize_t mdss_dsi_panel_frames_per_ksecs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fps_data fpsd = mdss_dsi_panel_driver_get_fps_data();

	return scnprintf(buf, PAGE_SIZE, "%i\n", fpsd.fpks);
}

static ssize_t mdss_dsi_panel_vsyncs_per_ksecs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return mdss_dsi_panel_driver_vsyncs_per_ksecs_store(dev, buf, count);
}

static ssize_t mdss_dsi_panel_vsyncs_per_ksecs_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct fps_data vpsd = mdss_dsi_panel_driver_get_vps_data();

	if (vpsd.vps_en)
		return scnprintf(buf, PAGE_SIZE, "%i\n", vpsd.fpks);
	else
		return scnprintf(buf, PAGE_SIZE,
		"This function is invalid now.\n"
		"Please read again after writing ON.\n");
}

static ssize_t mdss_dsi_panel_change_fpks_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return mdss_dsi_panel_driver_change_fpks_store(dev, attr, buf, count);
}

static ssize_t mdss_dsi_panel_change_fpks_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return mdss_dsi_panel_driver_change_fpks_show(dev, attr, buf);
}

static ssize_t mdss_dsi_panel_change_fps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return mdss_dsi_panel_driver_change_fps_store(dev, attr, buf, count);
}

static ssize_t mdss_dsi_panel_change_fps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return mdss_dsi_panel_driver_change_fps_show(dev, attr, buf);
}

static ssize_t mdss_dsi_panel_esd_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdss_panel_specific_pdata *spec_pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	spec_pdata = ctrl_pdata->spec_pdata;

	return scnprintf(buf, PAGE_SIZE, "esd_enable = %u\n",
				spec_pdata->esd_enable_without_xlog);
}

static ssize_t mdss_dsi_panel_esd_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	struct mdss_panel_specific_pdata *spec_pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = dev_get_drvdata(dev);

	ctrl_pdata = mdss_dsi_get_master_ctrl(&ctrl_pdata->panel_data);
	spec_pdata = ctrl_pdata->spec_pdata;

	rc = kstrtouint(buf, 10, &spec_pdata->esd_enable_without_xlog);
	if (rc < 0) {
		pr_err("%s: Error, buf = %s\n", __func__, buf);
		return rc;
	}
	pr_notice("%s: esd_enable = %u\n",
			__func__, spec_pdata->esd_enable_without_xlog);
	return count;
}

static struct device_attribute panel_attributes[] = {
	__ATTR(panel_id, S_IRUSR, mdss_dsi_panel_id_show, NULL),
	__ATTR(cc, S_IRUGO, mdss_dsi_panel_pcc_show, NULL),
	__ATTR(srgb_cc, S_IRUGO, mdss_dsi_panel_srgb_pcc_show, NULL),
	__ATTR(vivid_cc, S_IRUGO, mdss_dsi_panel_vivid_pcc_show, NULL),
	__ATTR(hdr_cc, S_IRUGO, mdss_dsi_panel_hdr_pcc_show, NULL),
	__ATTR(c_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		mdss_dsi_panel_color_mode_show,
		mdss_dsi_panel_color_mode_store),
	__ATTR(frame_counter, S_IRUGO, mdss_dsi_panel_frame_counter, NULL),
	__ATTR(frames_per_ksecs, S_IRUGO,
		mdss_dsi_panel_frames_per_ksecs, NULL),
	__ATTR(vsyncs_per_ksecs, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
		mdss_dsi_panel_vsyncs_per_ksecs_show,
		mdss_dsi_panel_vsyncs_per_ksecs_store),
	__ATTR(change_fps, S_IRUGO|S_IWUSR|S_IWGRP,
					mdss_dsi_panel_change_fps_show,
					mdss_dsi_panel_change_fps_store),
	__ATTR(change_fpks, S_IRUGO|S_IWUSR|S_IWGRP,
					mdss_dsi_panel_change_fpks_show,
					mdss_dsi_panel_change_fpks_store),
	__ATTR(esd_enable_wo_xlog, S_IRUSR|S_IRGRP|S_IWUSR|S_IWGRP,
		mdss_dsi_panel_esd_enable_show,
		mdss_dsi_panel_esd_enable_store),
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

int mdss_dsi_panel_create_fs(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	char *path_name = "mdss_dsi_panel";

	dev_set_name(&virtdev, "%s", path_name);
	rc = device_register(&virtdev);
	if (rc) {
		pr_err("%s: device_register rc = %d\n", __func__, rc);
		goto err;
	}

	rc = register_attributes(&virtdev);
	if (rc) {
		pr_err("%s: register_attributes rc = %d\n", __func__, rc);
		device_unregister(&virtdev);
		goto err;
	}

	dev_set_drvdata(&virtdev, ctrl_pdata);

err:
	return rc;
}

static void blackscreen_det_init(void)
{
	bs_det.threshold = DEFAULT_BS_THRESHOLD;
	bs_det.cnt_crash = 0;
	bs_det.cnt_timeout = 0;
	memset(&bs_det.timestamp, 0x00, sizeof(ktime_t));
	bs_det.done = 0;
	init_waitqueue_head(&bs_det.wait_queue);
}

static void first_frame_flushed_det_init(void)
{
	memset(&fff_det.timestamp, 0x00, sizeof(ktime_t));
	fff_det.done = 0;
	init_waitqueue_head(&fff_det.wait_queue);
}

static void crash_counter_reset(void)
{
	bs_det.cnt_crash = 0;
}

static void mdss_dsi_panel_blackscreen_det(void)
{
	bs_det.cnt_timeout++;
	bs_det.cnt_crash++;
	bs_det.timestamp = ktime_get_boottime();
	bs_det.done = 1;
	wake_up(&bs_det.wait_queue);

	if (bs_det.cnt_crash > bs_det.threshold)
		panic("[BScreenD] : panel command timeout error expired");
}

static void mdss_dsi_panel_fff_time_update(
	struct mdss_panel_specific_pdata *spec_pdata)
{
	fff_det.timestamp = ktime_get_boottime();
	spec_pdata->resume_started = false;
	fff_det.done = 1;
	wake_up(&fff_det.wait_queue);
}

static char *format_timestamp(ktime_t timestamp)
{
	static char buf[20];

	scnprintf(buf, sizeof(buf), "%lld", ktime_to_ms(timestamp));
	return &buf[0];
}

static int bs_threshold_show(struct seq_file *s, void *unused)
{
	struct msm_fb_data_type *mfd = s->private;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_panel_info *pinfo;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("%s: no panel connected\n", __func__);
		goto exit;
	}

	ctrl_pdata = mdss_dsi_get_master_ctrl(pdata);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		goto exit;
	}
	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		seq_printf(s, "%d\n",
			bs_det.threshold);
	} else {
		seq_printf(s, "VideoMode isn't supported\n");
	}
exit:
	return 0;
}

static int timeout_counter_show(struct seq_file *s, void *unused)
{
	struct msm_fb_data_type *mfd = s->private;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_panel_info *pinfo;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("%s: no panel connected\n", __func__);
		goto exit;
	}

	ctrl_pdata = mdss_dsi_get_master_ctrl(pdata);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		goto exit;
	}
	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		seq_printf(s, "%s %d\n",
			format_timestamp(bs_det.timestamp), bs_det.cnt_timeout);
	} else {
		seq_printf(s, "VideoMode isn't supported\n");
	}
exit:
	return 0;
}

static int crash_counter_show(struct seq_file *s, void *unused)
{
	struct msm_fb_data_type *mfd = s->private;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_panel_info *pinfo;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("%s: no panel connected\n", __func__);
		goto exit;
	}

	ctrl_pdata = mdss_dsi_get_master_ctrl(pdata);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		goto exit;
	}
	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		seq_printf(s, "%d\n",
			bs_det.cnt_crash);
	} else {
		seq_printf(s, "VideoMode isn't supported\n");
	}
exit:
	return 0;
}

static int fff_time_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s\n",
		format_timestamp(fff_det.timestamp));
	return 0;
}

static int bs_threshold_open(struct inode *inode, struct file *file)
{
	return single_open(file, bs_threshold_show, inode->i_private);
}

static ssize_t bs_threshold_write(struct file *file, const char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_fb_data_type *mfd = s->private;
	struct mdss_panel_data *pdata;
	int ret;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;
	char *buf = NULL;
	u16 threshold;

	if (!mfd->panel_power_state) {
		pr_err("%s: panel is NOT on\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("%s: no panel connected\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	ctrl_pdata = mdss_dsi_get_master_ctrl(pdata);
	if (!ctrl_pdata) {
		pr_err("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		ret = setup_reg_access(&buf, ubuf, count);
		if (ret) {
			pr_err("%s: Error, setup_reg_access\n", __func__);
			ret = -EINVAL;
			goto exit;
		}
		if (kstrtou16(buf, 10, &threshold)) {
			pr_err("%s: Error, buf = %s\n", __func__, buf);
			ret = -EINVAL;
			goto fail_free_all;
		}
		pr_err("%s: threshold = %d\n", __func__, threshold);
		ret = threshold;
		bs_det.threshold = threshold;

		if (bs_det.cnt_crash > bs_det.threshold)
			pr_notice("crash_counter is already beyond threshold\n");
	} else {
		pr_err("VideoMode isn't supported\n");
		ret = -EINVAL;
		goto exit;
	}

fail_free_all:
	kzfree(buf);
exit:
	return ret;
}

static void mdss_dsi_panel_black_screen_off(
	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	ctrl_pdata->spec_pdata->resume_started = true;
}

static const struct file_operations bs_threshold_fops = {
	.owner		= THIS_MODULE,
	.open		= bs_threshold_open,
	.read		= seq_read,
	.write		= bs_threshold_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int timeout_counter_open(struct inode *inode, struct file *file)
{
	return single_open(file, timeout_counter_show, inode->i_private);
}

static unsigned int timeout_counter_poll(struct file *filp,
			struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &bs_det.wait_queue, wait);
	if (bs_det.done) {
		mask |= (POLLIN | POLLRDNORM);
		bs_det.done = 0;
	}

	return mask;
}

static const struct file_operations timeout_counter_fops = {
	.owner		= THIS_MODULE,
	.open		= timeout_counter_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.poll		= timeout_counter_poll,
	.release	= single_release,
};

static int crash_counter_open(struct inode *inode, struct file *file)
{
	return single_open(file, crash_counter_show, inode->i_private);
}

static const struct file_operations crash_counter_fops = {
	.owner		= THIS_MODULE,
	.open		= crash_counter_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int fff_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, fff_time_show, inode->i_private);
}

static unsigned int fff_time_poll(struct file *filp,
			struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &fff_det.wait_queue, wait);
	if (fff_det.done) {
		mask |= (POLLIN | POLLRDNORM);
		fff_det.done = 0;
	}

	return mask;
}

static const struct file_operations fff_time_fops = {
	.owner		= THIS_MODULE,
	.open		= fff_time_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.poll		= fff_time_poll,
	.release	= single_release,
};
static void mipi_dsi_panel_create_debugfs(struct msm_fb_data_type *mfd)
{
	struct device *dev;
	struct dentry *root;
	struct mdss_panel_data *pdata;
	struct platform_device *pdev;
	struct mdss_panel_specific_pdata *spec_pdata;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	if (!pdata) {
		pr_err("%s: no panel connected\n", __func__);
		return;
	}

	blackscreen_det_init();
	first_frame_flushed_det_init();
	spec_pdata = mdss_panel2spec_pdata(pdata);
	spec_pdata->crash_counter_reset = crash_counter_reset;
	spec_pdata->blackscreen_det = mdss_dsi_panel_blackscreen_det;
	spec_pdata->fff_time_update = mdss_dsi_panel_fff_time_update;
	spec_pdata->black_screen_off = mdss_dsi_panel_black_screen_off;
	spec_pdata->resume_started = false;

	pdev = pdata->panel_pdev;
	if (!pdev) {
		pr_err("%s: no device\n", __func__);
		return;
	}

	dev = &pdev->dev;

	if (!&dev->kobj) {
		pr_err("%s: no &dev->kobj\n", __func__);
		return;
	}

	mdss_dsi_buf_alloc(dev, &debug_tx_buf, ALIGN(DSI_BUF_SIZE, SZ_4K));
	mdss_dsi_buf_alloc(dev, &debug_rx_buf, ALIGN(DSI_BUF_SIZE, SZ_4K));

	dev_notice(dev, "%s: create folder %s\n", __func__,
						kobject_name(&dev->kobj));

	root = debugfs_create_dir("mdss_dsi_panel", 0);

	if (!root) {
		dev_err(dev, "%s: dbgfs create dir failed\n", __func__);
	} else {
		if (!debugfs_create_file("read", S_IWUSR, root, mfd,
								&read_fops)) {
			dev_err(dev, "%s: failed to create dbgfs read file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("write", S_IWUSR, root, mfd,
								&write_fops)) {
			dev_err(dev, "%s: failed to create dbgfs write file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("result", S_IRUGO, root, mfd,
								&result_fops)) {
			dev_err(dev, "%s: failed to create dbgfs result file\n",
								__func__);
			return;
		}
		if (!debugfs_create_file("bs_threshold", S_IRUGO|S_IWUSR, root,
					 mfd, &bs_threshold_fops)) {
			dev_err(dev,
				"%s: failed to create dbgfs bs_threshold file\n",
				__func__);
			return;
		}
		if (!debugfs_create_file("timeout_counter", S_IRUGO, root, mfd,
					 &timeout_counter_fops)) {
			dev_err(dev,
				"%s: failed to create dbgfs timeout_counter file\n",
				__func__);
			return;
		}
		if (!debugfs_create_file("crash_counter", S_IRUGO, root, mfd,
					 &crash_counter_fops)) {
			dev_err(dev,
				"%s: failed to create dbgfs crash_counter file\n",
				__func__);
			return;
		}
		if (!debugfs_create_file("fff_time", S_IRUGO, root, mfd,
					 &fff_time_fops)) {
			dev_err(dev, "%s: failed to create dbgfs fff_time file\n",
				__func__);
			return;
		}
	}
}

int mdss_dsi_create_debugfs(struct msm_fb_data_type *mfd)
{
	int rc;

	mipi_dsi_panel_create_debugfs(mfd);

	rc = incell_create_debugfs(mfd);
	if (rc) {
		pr_err("%s: incell_create_debugfs rc = %d\n", __func__, rc);
		goto out;
	}
out:
	return rc;
}
