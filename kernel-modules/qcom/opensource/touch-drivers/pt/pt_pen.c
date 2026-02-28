#ifndef TTDL_KERNEL_SUBMISSION
/*
 * pt_pen.c
 * Parade TrueTouch(TM) Standard Product CapSense Reports Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * TMA5XX
 * TMA448
 * TMA445A
 * TT21XXX
 * TT31XXX
 * TT4XXXX
 * TT7XXX
 * TC3XXX
 *
 * Copyright (C) 2015-2021 Parade Technologies
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 */

#include "pt_regs.h"

/*******************************************************************************
 * FUNCTION: pt_pen_lift_all
 *
 * SUMMARY: Reports pen liftoff action
 *
 * PARAMETERS:
 *     *pend  - pointer to pen data structure
 ******************************************************************************/
static void pt_pen_lift_all(struct pt_pen_data *pend)
{
	input_report_key(pend->input, BTN_STYLUS, 0);
	input_report_key(pend->input, BTN_STYLUS2, 0);
	input_report_key(pend->input, BTN_TOUCH, 0);
	input_report_key(pend->input, BTN_TOOL_PEN, 0);
	input_sync(pend->input);
}

/*******************************************************************************
 * FUNCTION: pt_get_pen_data
 *
 * SUMMARY: Gets axis of pen report
 *
 * PARAMETERS:
 *     *pend    - pointer to pen data structure
 *     *xy_data - pointer to touch data
 ******************************************************************************/
static void pt_get_pen(struct pt_pen_data *pend,
	struct pt_pen *pen, u8 *xy_data)
{
	struct device *dev = pend->dev;
	struct pt_sysinfo *si = pend->si;
	enum pt_pen_abs abs;

	for (abs = PT_PEN_X; abs < PT_PEN_NUM_ABS; abs++) {
		if (!si->pen_abs[abs].report)
			continue;
		pt_get_touch_field(dev, &pen->abs[abs],
			si->pen_abs[abs].size,
			si->pen_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->pen_abs[abs].bofs);
		pt_debug(dev, DL_DEBUG, "%s: get %s=%04X(%d)\n",
			__func__, pt_pen_abs_string[abs],
			pen->abs[abs], pen->abs[abs]);
	}
}


/*******************************************************************************
 * FUNCTION: pt_xy_worker
 *
 * SUMMARY: Read xy_data for current pen touch
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *pend  - pointer to pen data structure
 ******************************************************************************/
static int pt_xy_worker(struct pt_pen_data *pend)
{
	struct pt_sysinfo *si = pend->si;
	struct pt_pen pen;
	bool tool;

	pt_get_pen(pend, &pen, si->xy_data + 3);

	tool = pen.abs[PT_PEN_IV] ?
		BTN_TOOL_RUBBER : BTN_TOOL_PEN;

	input_report_abs(pend->input, ABS_X, pen.abs[PT_PEN_X]);
	input_report_abs(pend->input, ABS_Y, pen.abs[PT_PEN_Y]);
	input_report_abs(pend->input, ABS_PRESSURE, pen.abs[PT_PEN_P]);
	input_report_key(pend->input, BTN_STYLUS, pen.abs[PT_PEN_BS]);

	if (si->pen_abs[PT_PEN_2ND_BS].report)
		input_report_key(pend->input, BTN_STYLUS2,
			pen.abs[PT_PEN_2ND_BS]);

	input_report_key(pend->input, BTN_TOUCH, pen.abs[PT_PEN_TS]);
	input_report_key(pend->input, tool, pen.abs[PT_PEN_IR]);

	if (si->pen_abs[PT_PEN_X_TILT].report)
		input_report_abs(pend->input, ABS_TILT_X,
			pen.abs[PT_PEN_X_TILT]);

	if (si->pen_abs[PT_PEN_Y_TILT].report)
		input_report_abs(pend->input, ABS_TILT_Y,
			pen.abs[PT_PEN_Y_TILT]);

	input_sync(pend->input);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pen_attention
 *
 * SUMMARY: Wrapper function for pt_xy_worker() that register to TTDL attention
 *  list.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_pen_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;
	int rc;

	if (pend->si->xy_mode[2] != pend->si->desc.pen_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&pend->pen_lock);
	rc = pt_xy_worker(pend);
	mutex_unlock(&pend->pen_lock);
	if (rc < 0)
		pt_debug(dev, DL_ERROR,
			"%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_startup_attention
 *
 * SUMMARY: Wrapper function for pt_pen_lift_all() that register to TTDL
 *  attention list.
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_startup_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;

	mutex_lock(&pend->pen_lock);
	pt_pen_lift_all(pend);
	mutex_unlock(&pend->pen_lock);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pen_suspend_attention
 *
 * SUMMARY: Function for pen to enter suspend state that as following steps:
 *          1) Lift pen touch
 *          2) Set flag with suspend state
 *          3) Decrese pm system count
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_pen_suspend_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;

	mutex_lock(&pend->pen_lock);
	pt_pen_lift_all(pend);
	pend->is_suspended = true;
	mutex_unlock(&pend->pen_lock);

	pm_runtime_put(dev);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pen_resume_attention
 *
 * SUMMARY: Function for pen to leave suspend state that as following steps:
 *          1) Increse pm system count
 *          2) Clear suspend state flag
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_pen_resume_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;

	pm_runtime_get(dev);

	mutex_lock(&pend->pen_lock);
	pend->is_suspended = false;
	mutex_unlock(&pend->pen_lock);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pen_open
 *
 * SUMMARY: Open method for input device(pen) that sets up call back
 *  functions to TTDL attention list
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *     *input - pointer to input_dev structure
 ******************************************************************************/
static int pt_pen_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;

	pm_runtime_get_sync(dev);

	mutex_lock(&pend->pen_lock);
	pend->is_suspended = false;
	mutex_unlock(&pend->pen_lock);

	pt_debug(dev, DL_INFO, "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	_pt_subscribe_attention(dev, PT_ATTEN_IRQ, PT_PEN_NAME,
		pt_pen_attention, PT_MODE_OPERATIONAL);

	/* set up startup call back */
	_pt_subscribe_attention(dev, PT_ATTEN_STARTUP, PT_PEN_NAME,
		pt_startup_attention, 0);

	/* set up suspend call back */
	_pt_subscribe_attention(dev, PT_ATTEN_SUSPEND, PT_PEN_NAME,
		pt_pen_suspend_attention, 0);

	/* set up resume call back */
	_pt_subscribe_attention(dev, PT_ATTEN_RESUME, PT_PEN_NAME,
		pt_pen_resume_attention, 0);

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_pen_close
 *
 * SUMMARY: Close method for input device(pen) that clears call back
 *  functions from TTDL attention list.
 *
 * PARAMETERS:
 *     *input - pointer to input_dev structure
 ******************************************************************************/
static void pt_pen_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;

	_pt_unsubscribe_attention(dev, PT_ATTEN_IRQ, PT_PEN_NAME,
		pt_pen_attention, PT_MODE_OPERATIONAL);

	_pt_unsubscribe_attention(dev, PT_ATTEN_STARTUP, PT_PEN_NAME,
		pt_startup_attention, 0);

	_pt_unsubscribe_attention(dev, PT_ATTEN_SUSPEND, PT_PEN_NAME,
		pt_pen_suspend_attention, 0);

	_pt_unsubscribe_attention(dev, PT_ATTEN_RESUME, PT_PEN_NAME,
		pt_pen_resume_attention, 0);

	mutex_lock(&pend->pen_lock);
	if (!pend->is_suspended) {
		pm_runtime_put(dev);
		pend->is_suspended = true;
	}
	mutex_unlock(&pend->pen_lock);
}

/*******************************************************************************
 * FUNCTION: pt_setup_input_device
 *
 * SUMMARY: Set up resolution, event signal capabilities and register input
 *  device for pen.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_setup_input_device(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;
	int i;
	int rc;
	u32 usage;
	int max_x, max_y, max_p;

	pt_debug(dev, DL_INFO, "%s: Initialize event signals\n",
		__func__);
	__set_bit(EV_ABS, pend->input->evbit);
	__set_bit(EV_KEY, pend->input->evbit);

	for (i = PT_PEN_X; i < PT_PEN_NUM_ABS; i++) {
		usage = pt_pen_abs_field_map[i];

		switch (usage) {
		case HID_GD_X:
			max_x = pend->si->sensing_conf_data.res_x;
			input_set_abs_params(pend->input,
				ABS_X, 0, max_x, 0, 0);
			break;
		case HID_GD_Y:
			max_y = pend->si->sensing_conf_data.res_y;
			input_set_abs_params(pend->input,
				ABS_Y, 0, max_y, 0, 0);
			break;
		case HID_DG_TIPPRESSURE:
			max_p = pend->si->sensing_conf_data.max_z;
			input_set_abs_params(pend->input,
				ABS_PRESSURE, 0, max_p, 0, 0);
			break;
		case HID_DG_INRANGE:
			input_set_capability(pend->input,
				EV_KEY, BTN_TOOL_PEN);
			break;
		case HID_DG_INVERT:
			input_set_capability(pend->input,
				EV_KEY, BTN_TOOL_RUBBER);
			break;
		case HID_DG_TILT_X:
			max_x = pend->si->pen_abs[i].max;
			input_set_abs_params(pend->input,
				ABS_TILT_X, 0, max_x, 0, 0);
			break;
		case HID_DG_TILT_Y:
			max_x = pend->si->pen_abs[i].max;
			input_set_abs_params(pend->input,
				ABS_TILT_Y, 0, max_x, 0, 0);
			break;
		case HID_DG_ERASER:
		case HID_DG_TIPSWITCH:
			input_set_capability(pend->input,
				EV_KEY, BTN_TOUCH);
			break;
		case HID_DG_BARRELSWITCH:
			input_set_capability(pend->input,
				EV_KEY, BTN_STYLUS);
			break;
		case HID_DG_BARRELSWITCH2:
			input_set_capability(pend->input,
				EV_KEY, BTN_STYLUS2);
			break;
		}
	}

	rc = input_register_device(pend->input);
	if (rc < 0)
		pt_debug(dev, DL_ERROR,
			"%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		pend->input_device_registered = true;

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_setup_input_attention
 *
 * SUMMARY: Wrapper function for pt_setup_input_device() register to TTDL
 *  attention list.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *     *dev - pointer to device structure
 ******************************************************************************/
static int pt_setup_input_attention(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;
	int rc;

	pend->si = _pt_request_sysinfo(dev);
	if (!pend->si)
		return -1;

	rc = pt_setup_input_device(dev);

	_pt_unsubscribe_attention(dev, PT_ATTEN_STARTUP, PT_PEN_NAME,
		pt_setup_input_attention, 0);

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pen_probe
 *
 * SUMMARY: The probe function for pen input device
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 *
 * PARAMETERS:
 *	*dev   - pointer to device structure
 ******************************************************************************/
int pt_pen_probe(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_pen_data *pend = &cd->pend;
	struct pt_platform_data *pdata = dev_get_platdata(dev);
	struct pt_pen_platform_data *pen_pdata;
	int rc = 0;

	if (!pdata || !pdata->pen_pdata) {
		pt_debug(dev, DL_ERROR,
			"%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}
	pen_pdata = pdata->pen_pdata;

	mutex_init(&pend->pen_lock);
	pend->dev = dev;
	pend->pdata = pen_pdata;

	/* Create the input device and register it. */
	pt_debug(dev, DL_INFO,
		"%s: Create the input device and register it\n", __func__);
	pend->input = input_allocate_device();
	if (!pend->input) {
		pt_debug(dev, DL_ERROR,
			"%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENODEV;
		goto error_alloc_failed;
	} else
		pend->input_device_allocated = true;

	if (pend->pdata->inp_dev_name)
		pend->input->name = pend->pdata->inp_dev_name;
	else
		pend->input->name = PT_PEN_NAME;
	scnprintf(pend->phys, sizeof(pend->phys), "%s/input%d", dev_name(dev),
			cd->phys_num++);
	pend->input->phys = pend->phys;
	pend->input->dev.parent = pend->dev;
	pend->input->open = pt_pen_open;
	pend->input->close = pt_pen_close;
	input_set_drvdata(pend->input, pend);

	/* get sysinfo */
	pend->si = _pt_request_sysinfo(dev);

	if (pend->si) {
		rc = pt_setup_input_device(dev);
		if (rc)
			goto error_init_input;
	} else {
		pt_debug(dev, DL_ERROR,
			"%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, pend->si);
		_pt_subscribe_attention(dev, PT_ATTEN_STARTUP,
			PT_PEN_NAME, pt_setup_input_attention, 0);
	}

	return 0;

error_init_input:
	input_free_device(pend->input);
	pend->input_device_allocated = false;
error_alloc_failed:
error_no_pdata:
	pt_debug(dev, DL_ERROR, "%s failed.\n", __func__);
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_pen_release
 *
 * SUMMARY: The release function for pen input device
 *
 * RETURN:
 *	 0 = success
 *
 * PARAMETERS:
 *	*dev   - pointer to device structure
 ******************************************************************************/
int pt_pen_release(struct device *dev)
{
	struct pt_core_data *cd;
	struct pt_pen_data *pend;

	/* Ensure valid pointers before de-referencing them */
	if (dev) {
		cd = dev_get_drvdata(dev);
		if (cd)
			pend = &cd->pend;
		else
			return 0;
	} else {
		return 0;
	}

	/*
	 * Second call this function may cause kernel panic if probe fail.
	 * Use input_device_registered & input_device_allocated variable to
	 * avoid unregister or free unavailable devive.
	 */
	if (pend && pend->input_device_registered) {
		pend->input_device_registered = false;
		input_unregister_device(pend->input);
		/* Unregistering device will free the device too */
		pend->input_device_allocated = false;
	} else if (pend && pend->input_device_allocated) {
		pend->input_device_allocated = false;
		input_free_device(pend->input);
		_pt_unsubscribe_attention(dev, PT_ATTEN_STARTUP,
			PT_PEN_NAME, pt_setup_input_attention, 0);
	}

	return 0;
}
#endif /*!TTDL_KERNEL_SUBMISSION */
