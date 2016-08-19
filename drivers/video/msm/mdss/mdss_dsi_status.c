/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are Copyright (c) 2015 Sony Mobile Communications Inc,
 * and licensed under the license of the file.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/iopoll.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_panel.h"
#include "mdss_mdp.h"
#include "mdss_dsi_panel_debugfs.h"

#define STATUS_CHECK_INTERVAL_MS 5000
#define STATUS_CHECK_INTERVAL_MIN_MS 50
#define DSI_STATUS_CHECK_DISABLE 0

static uint32_t interval = STATUS_CHECK_INTERVAL_MS;
static uint32_t dsi_status_disable = DSI_STATUS_CHECK_DISABLE;
struct dsi_status_data *pstatus_data;

struct poll_ctrl *polling;

/*
 * send_panel_on_seq() - Sends on sequence.
 */
static int send_panel_on_seq(struct mdss_mdp_ctl *ctl,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int ret;
	struct mdss_panel_data *pdata = ctl->panel_data;

	if (!ctl || !pdata) {
		pr_err("%s: Invalid panel data\n", __func__);
		return -EINVAL;
	}

	ret = ctrl_pdata->on(pdata);

	if (pdata->panel_info.disp_on_in_hs) {
		if (ctrl_pdata->spec_pdata->disp_on)
			ret = ctrl_pdata->spec_pdata->disp_on(pdata);
	}

	if (ret)
		pr_err("%s: Failed to send display on sequence\n", __func__);

	pr_debug("%s: Display on sequence completed\n", __func__);
	return ret;
}

/*
 * check_dric_reg_status() - Checks value correct or not
 */
static int check_esd_status(void)
{
	int rc = 0;
	char rbuf;

	panel_cmd_read(polling->ctrl_pdata, &(polling->esd.dsi),
			NULL, &rbuf, polling->esd.nbr_bytes_to_read);

	if (polling->esd.correct_val != rbuf) {
		pr_err("%s:Target reg value isn't correct rbuf = 0x%02x\n",
			__func__, rbuf);

		rc = -EINVAL;
	}

	return rc;
}

/*
 * check_poll_status() - Checks polling status
 */
static int check_poll_status(void)
{
	if (!polling) {
		pr_err("%s: Invalid poll worker\n", __func__);
		return -EINVAL;
	}

	if (!polling->ctrl_pdata) {
		pr_err("%s: Polling Disable\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/*
 * poll_panel_status() - Reads panel register.
 * @work  : driver ic status data
 */
static void poll_panel_status(struct work_struct *work)
{
	int rc = 0;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_mdp_ctl *ctl = mdata->ctl_off;

	rc = check_poll_status();
	if (rc)
		return;

	rc = check_esd_status();
	if (rc)
		send_panel_on_seq(ctl,
				polling->ctrl_pdata);
	else
		schedule_delayed_work(&polling->poll_working,
				msecs_to_jiffies(polling->intervals));
}

/*
 * check_dsi_ctrl_status() - Reads MFD structure and
 * calls platform specific DSI ctrl Status function.
 * @work  : dsi controller status data
 */
static void check_dsi_ctrl_status(struct work_struct *work)
{
	struct dsi_status_data *pdsi_status = NULL;

	pdsi_status = container_of(to_delayed_work(work),
		struct dsi_status_data, check_status);

	if (!pdsi_status) {
		pr_err("%s: DSI status data not available\n", __func__);
		return;
	}

	if (!pdsi_status->mfd) {
		pr_err("%s: FB data not available\n", __func__);
		return;
	}

	if (mdss_panel_is_power_off(pdsi_status->mfd->panel_power_state) ||
			pdsi_status->mfd->shutdown_pending) {
		pr_err("%s: panel off\n", __func__);
		return;
	}

	pdsi_status->mfd->mdp.check_dsi_status(work, interval);
}

/*
 * hw_vsync_handler() - Interrupt handler for HW VSYNC signal.
 * @irq		: irq line number
 * @data	: Pointer to the device structure.
 *
 * This function is called whenever a HW vsync signal is received from the
 * panel. This resets the timer of ESD delayed workqueue back to initial
 * value.
 */
irqreturn_t hw_vsync_handler(int irq, void *data)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata =
			(struct mdss_dsi_ctrl_pdata *)data;
	if (!ctrl_pdata) {
		pr_err("%s: DSI ctrl not available\n", __func__);
		return IRQ_HANDLED;
	}

	if (pstatus_data)
		mod_delayed_work(system_wq, &pstatus_data->check_status,
			msecs_to_jiffies(interval));
	else
		pr_err("Pstatus data is NULL\n");

	if (!atomic_read(&ctrl_pdata->te_irq_ready))
		atomic_inc(&ctrl_pdata->te_irq_ready);

	return IRQ_HANDLED;
}

/*
 * fb_event_callback() - Call back function for the fb_register_client()
 *			 notifying events
 * @self  : notifier block
 * @event : The event that was triggered
 * @data  : Of type struct fb_event
 *
 * This function listens for FB_BLANK_UNBLANK and FB_BLANK_POWERDOWN events
 * from frame buffer. DSI status check work is either scheduled again after
 * PANEL_STATUS_CHECK_INTERVAL or cancelled based on the event.
 */
static int fb_event_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct dsi_status_data *pdata = container_of(self,
				struct dsi_status_data, fb_notifier);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;
	struct msm_fb_data_type *mfd;

	if (!evdata) {
		pr_err("%s: event data not available\n", __func__);
		return NOTIFY_BAD;
	}

	mfd = evdata->info->par;
	ctrl_pdata = container_of(dev_get_platdata(&mfd->pdev->dev),
				struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: DSI ctrl not available\n", __func__);
		return NOTIFY_BAD;
	}

	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (!(pinfo->esd_check_enabled)) {
		pr_debug("ESD check is not enaled in panel dtsi\n");
		return NOTIFY_DONE;
	}

	if (dsi_status_disable) {
		pr_debug("%s: DSI status disabled\n", __func__);
		return NOTIFY_DONE;
	}

	pdata->mfd = evdata->info->par;
	if (event == FB_EVENT_BLANK) {
		int *blank = evdata->data;
		struct dsi_status_data *pdata = container_of(self,
				struct dsi_status_data, fb_notifier);
		pdata->mfd = evdata->info->par;

		switch (*blank) {
		case FB_BLANK_UNBLANK:
			schedule_delayed_work(&pdata->check_status,
				msecs_to_jiffies(interval));
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			cancel_delayed_work(&pdata->check_status);
			break;
		default:
			pr_err("Unknown case in FB_EVENT_BLANK event\n");
			break;
		}
	}
	return 0;
}

static int param_dsi_status_disable(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	int int_val;

	ret = kstrtos32(val, 0, &int_val);
	if (ret)
		return ret;

	pr_info("%s: Set DSI status disable to %d\n",
			__func__, int_val);
	*((int *)kp->arg) = int_val;
	return ret;
}

static int param_set_interval(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	int int_val;

	ret = kstrtos32(val, 0, &int_val);
	if (ret)
		return ret;
	if (int_val < STATUS_CHECK_INTERVAL_MIN_MS) {
		pr_err("%s: Invalid value %d used, ignoring\n",
						__func__, int_val);
		ret = -EINVAL;
	} else {
		pr_info("%s: Set check interval to %d msecs\n",
						__func__, int_val);
		*((int *)kp->arg) = int_val;
	}
	return ret;
}

#define POLL_REG_BYTE_TO_READ 1
static struct dsi_ctrl_hdr poll_reg_dchdr = {
	DTYPE_DCS_READ, 1, 0, 1, 5, 1};
int mdss_dsi_panel_poll_init(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	polling = NULL;

	if (ctrl_pdata) {
		polling = &(ctrl_pdata->spec_pdata->polling);
		INIT_DELAYED_WORK(&(polling->poll_working), poll_panel_status);

		polling->esd.dsi.payload = &polling->esd.reg;
		polling->esd.nbr_bytes_to_read = POLL_REG_BYTE_TO_READ;
		polling->esd.dsi.dchdr = poll_reg_dchdr;

	} else {
		pr_err("%s: Invalid panel data\n", __func__);
	}

	return 0;
}

int __init mdss_dsi_status_init(void)
{
	int rc = 0;

	pstatus_data = kzalloc(sizeof(struct dsi_status_data), GFP_KERNEL);
	if (!pstatus_data) {
		pr_err("%s: can't allocate memory\n", __func__);
		return -ENOMEM;
	}

	pstatus_data->fb_notifier.notifier_call = fb_event_callback;

	rc = fb_register_client(&pstatus_data->fb_notifier);
	if (rc < 0) {
		pr_err("%s: fb_register_client failed, returned with rc=%d\n",
								__func__, rc);
		kfree(pstatus_data);
		return -EPERM;
	}

	pr_info("%s: DSI status check interval:%d\n", __func__,	interval);

	INIT_DELAYED_WORK(&pstatus_data->check_status, check_dsi_ctrl_status);

	pr_debug("%s: DSI ctrl status work queue initialized\n", __func__);

	return rc;
}

void __exit mdss_dsi_status_exit(void)
{
	fb_unregister_client(&pstatus_data->fb_notifier);
	cancel_delayed_work_sync(&pstatus_data->check_status);
	kfree(pstatus_data);
	pr_debug("%s: DSI ctrl status work queue removed\n", __func__);
}

module_param_call(interval, param_set_interval, param_get_uint,
						&interval, 0644);
MODULE_PARM_DESC(interval,
		"Duration in milliseconds to send BTA command for checking"
		"DSI status periodically");

module_param_call(dsi_status_disable, param_dsi_status_disable, param_get_uint,
						&dsi_status_disable, 0644);
MODULE_PARM_DESC(dsi_status_disable,
		"Disable DSI status check");

module_init(mdss_dsi_status_init);
module_exit(mdss_dsi_status_exit);

MODULE_LICENSE("GPL v2");
