/*
 * cyttsp4_debug.c
 * Cypress TrueTouch(TM) Standard Product V4 Core driver module.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2012 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 * Modified by: Cypress Semiconductor to add device functions
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/cyttsp4_bus.h>

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "cyttsp4_regs.h"

#define CYTTSP4_DEBUG_NAME "cyttsp4_debug"

struct cyttsp4_debug_data {
	struct cyttsp4_device *ttsp;
	struct cyttsp4_debug_platform_data *pdata;
	struct cyttsp4_sysinfo *si;
	uint32_t interrupt_count;
	uint32_t formated_output;
	struct mutex sysfs_lock;
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
};

struct cyttsp4_debug_platform_data {
	char const *debug_dev_name;
};

/*
 * This function provide output of combined xy_mode and xy_data.
 * Required by TTHE.
 */
void cyttsp4_pr_buf_op_mode(struct device *dev, u8 *pr_buf,
		struct cyttsp4_sysinfo *si, u8 curTouch)
{
	int i, k;
	const char fmt[] = "%02X ";
	int max = (CY_MAX_PRBUF_SIZE - 1) - sizeof(CY_PR_TRUNCATED);
	int totalSize = si->si_ofs.mode_size
			+ (curTouch * si->si_ofs.tch_rec_size);
	u8 num_btns = si->si_ofs.num_btns;

	pr_buf[0] = 0;
	for (i = k = 0; i < si->si_ofs.mode_size && i < max; i++, k += 3)
		scnprintf(pr_buf + k, CY_MAX_PRBUF_SIZE, fmt, si->xy_mode[i]);

	for (i = 0; i < (curTouch * si->si_ofs.tch_rec_size) && i < max;
			i++, k += 3)
		scnprintf(pr_buf + k, CY_MAX_PRBUF_SIZE, fmt, si->xy_data[i]);

	if (num_btns) {
		/* print btn diff data for TTHE */
		scnprintf(pr_buf + k, CY_MAX_PRBUF_SIZE, "%s", "=");
		k++;
		for (i = 0; i < (num_btns * si->si_ofs.btn_rec_size) && i < max;
				i++, k += 3)
			scnprintf(pr_buf + k, CY_MAX_PRBUF_SIZE, fmt,
					si->btn_rec_data[i]);
		totalSize += num_btns * si->si_ofs.btn_rec_size + 1;
	}
	pr_info("%s=%s%s\n", "cyttsp4_OpModeData", pr_buf,
			totalSize <= max ? "" : CY_PR_TRUNCATED);
}

static void cyttsp4_debug_print(struct device *dev, u8 *pr_buf, u8 *sptr,
		int size, const char *data_name)
{
	int i, j;
	int elem_size = sizeof("XX ") - 1;
	int max = (CY_MAX_PRBUF_SIZE - 1) / elem_size;
	int limit = size < max ? size : max;

	if (limit < 0)
		limit = 0;

	pr_buf[0] = 0;
	for (i = j = 0; i < limit; i++, j += elem_size)
		scnprintf(pr_buf + j, CY_MAX_PRBUF_SIZE - j, "%02X ", sptr[i]);

	pr_info("%s[0..%d]=%s%s\n", data_name, size - 1, pr_buf,
			size <= max ? "" : CY_PR_TRUNCATED);
}

static void cyttsp4_debug_formated(struct device *dev, u8 *pr_buf,
		struct cyttsp4_sysinfo *si, u8 num_cur_tch)
{
	u8 mode_size = si->si_ofs.mode_size;
	u8 rep_len = si->xy_mode[si->si_ofs.rep_ofs];
	u8 tch_rec_size = si->si_ofs.tch_rec_size;
	u8 num_btns = si->si_ofs.num_btns;
	u8 num_btn_regs = (num_btns + CY_NUM_BTN_PER_REG - 1)
			/ CY_NUM_BTN_PER_REG;
	u8 num_btn_tch;
	u8 data_name[] = "touch[99]";
	int max_print_length = 18;
	int i;

	/* xy_mode */
	cyttsp4_debug_print(dev, pr_buf, si->xy_mode, mode_size, "xy_mode");

	/* xy_data */
	if (rep_len > max_print_length) {
		pr_info("xy_data[0..%d]:\n", rep_len);
		for (i = 0; i < rep_len - max_print_length;
				i += max_print_length) {
			cyttsp4_debug_print(dev, pr_buf, si->xy_data + i,
					max_print_length, " ");
		}
		if (rep_len - i)
			cyttsp4_debug_print(dev, pr_buf, si->xy_data + i,
					rep_len - i, " ");
	} else {
		cyttsp4_debug_print(dev, pr_buf, si->xy_data, rep_len,
				"xy_data");
	}

	/* touches */
	for (i = 0; i < num_cur_tch; i++) {
		scnprintf(data_name, sizeof(data_name) - 1, "touch[%u]", i);
		cyttsp4_debug_print(dev, pr_buf,
				si->xy_data + (i * tch_rec_size),
				tch_rec_size, data_name);
	}

	/* buttons */
	if (num_btns) {
		num_btn_tch = 0;
		for (i = 0; i < num_btn_regs; i++) {
			if (si->xy_mode[si->si_ofs.rep_ofs + 2 + i]) {
				num_btn_tch++;
				break;
			}
		}
		if (num_btn_tch)
			cyttsp4_debug_print(dev, pr_buf,
					&si->xy_mode[si->si_ofs.rep_ofs + 2],
					num_btn_regs, "button");
	}
}

/* read xy_data for all touches for debug */
static int cyttsp4_xy_worker(struct cyttsp4_debug_data *dd)
{
	struct device *dev = &dd->ttsp->dev;
	struct cyttsp4_sysinfo *si = dd->si;
	u8 tt_stat = si->xy_mode[si->si_ofs.tt_stat_ofs];
	u8 num_cur_tch = GET_NUM_TOUCHES(tt_stat);
	uint32_t formated_output;
	int rc;

	mutex_lock(&dd->sysfs_lock);
	dd->interrupt_count++;
	formated_output = dd->formated_output;
	mutex_unlock(&dd->sysfs_lock);

	if (si->si_ofs.num_btns > 0) {
		/* read button diff data */
		rc = cyttsp4_read(dd->ttsp, CY_MODE_OPERATIONAL,
				/*  replace with btn_diff_ofs when that field
				 *  becomes supported in the firmware */
				si->si_ofs.tt_stat_ofs + 1 +
				si->si_ofs.max_tchs * si->si_ofs.tch_rec_size,
				si->btn_rec_data,
				si->si_ofs.num_btns * si->si_ofs.btn_rec_size);
		if (rc < 0) {
			dev_err(dev, "%s: read fail on button regs r=%d\n",
					__func__, rc);
		}
	}

	/* Interrupt */
	pr_info("Interrupt(%u)\n", dd->interrupt_count);

	if (formated_output)
		cyttsp4_debug_formated(dev, dd->pr_buf, si, num_cur_tch);
	else
		/* print data for TTHE */
		cyttsp4_pr_buf_op_mode(dev, dd->pr_buf, si, num_cur_tch);

#ifdef SHOK_SENSOR_DATA_MODE
	/* print data for the sensor monitor */
	if (si->monitor.mntr_status == CY_MNTR_STARTED) {
		cyttsp4_debug_print(dev, dd->pr_buf, si->monitor.sensor_data,
				150, "cyttsp4_sensor_monitor");
	}
#endif

	pr_info("\n");

	dev_vdbg(dev, "%s: done\n", __func__);

	return 0;
}

static int cyttsp4_debug_attention(struct cyttsp4_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_debug_data *dd = dev_get_drvdata(dev);
	int rc = 0;

	dev_vdbg(dev, "%s\n", __func__);

	/* core handles handshake */
	rc = cyttsp4_xy_worker(dd);
	if (rc < 0)
		dev_err(dev, "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static ssize_t cyttsp4_interrupt_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4_debug_data *dd = dev_get_drvdata(dev);
	int val;

	mutex_lock(&dd->sysfs_lock);
	val = dd->interrupt_count;
	mutex_unlock(&dd->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE, "Interrupt Count: %d\n", val);
}

static ssize_t cyttsp4_interrupt_count_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4_debug_data *dd = dev_get_drvdata(dev);
	mutex_lock(&dd->sysfs_lock);
	dd->interrupt_count = 0;
	mutex_unlock(&dd->sysfs_lock);
	return size;
}

static DEVICE_ATTR(int_count, S_IRUSR | S_IWUSR,
	cyttsp4_interrupt_count_show, cyttsp4_interrupt_count_store);

static ssize_t cyttsp4_formated_output_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4_debug_data *dd = dev_get_drvdata(dev);
	int val;

	mutex_lock(&dd->sysfs_lock);
	val = dd->formated_output;
	mutex_unlock(&dd->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Formated debug output: %x\n", val);
}

static ssize_t cyttsp4_formated_output_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4_debug_data *dd = dev_get_drvdata(dev);
	unsigned long value;
	int rc;

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		return size;
	}

	/* Expecting only 0 or 1 */
	if (value != 0 && value != 1) {
		dev_err(dev, "%s: Invalid value %lu\n", __func__, value);
		return size;
	}

	mutex_lock(&dd->sysfs_lock);
	dd->formated_output = value;
	mutex_unlock(&dd->sysfs_lock);
	return size;
}

static DEVICE_ATTR(formated_output, S_IRUSR | S_IWUSR,
	cyttsp4_formated_output_show, cyttsp4_formated_output_store);

static int cyttsp4_debug_probe(struct cyttsp4_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_debug_data *dd;
	struct cyttsp4_debug_platform_data *pdata = dev_get_platdata(dev);
	int rc;

	dev_info(dev, "%s: startup\n", __func__);
	dev_dbg(dev, "%s: debug on\n", __func__);
	dev_vdbg(dev, "%s: verbose debug on\n", __func__);

	/* get context and debug print buffers */
	dd = kzalloc(sizeof(*dd), GFP_KERNEL);
	if (dd == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto cyttsp4_debug_probe_alloc_failed;
	}

	rc = device_create_file(dev, &dev_attr_int_count);
	if (rc) {
		dev_err(dev, "%s: Error, could not create int_count\n",
				__func__);
		goto cyttsp4_debug_probe_create_int_count_failed;
	}

	rc = device_create_file(dev, &dev_attr_formated_output);
	if (rc) {
		dev_err(dev, "%s: Error, could not create formated_output\n",
				__func__);
		goto cyttsp4_debug_probe_create_formated_failed;
	}

	mutex_init(&dd->sysfs_lock);
	dd->ttsp = ttsp;
	dd->pdata = pdata;
	dev_set_drvdata(dev, dd);

	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);
	dd->si = cyttsp4_request_sysinfo(ttsp);
	if (dd->si == NULL) {
		dev_err(dev, "%s: Fail get sysinfo pointer from core\n",
				__func__);
		rc = -ENODEV;
		goto cyttsp4_debug_probe_sysinfo_failed;
	}

	rc = cyttsp4_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp4_debug_attention, CY_MODE_OPERATIONAL);
	if (rc < 0) {
		dev_err(dev, "%s: Error, could not subscribe attention cb\n",
				__func__);
		goto cyttsp4_debug_probe_subscribe_failed;
	}
	pm_runtime_put(dev);

	return 0;

cyttsp4_debug_probe_subscribe_failed:
cyttsp4_debug_probe_sysinfo_failed:
	pm_runtime_put(dev);
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	dev_set_drvdata(dev, NULL);
	device_remove_file(dev, &dev_attr_formated_output);
cyttsp4_debug_probe_create_formated_failed:
	device_remove_file(dev, &dev_attr_int_count);
cyttsp4_debug_probe_create_int_count_failed:
	kfree(dd);
cyttsp4_debug_probe_alloc_failed:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

static int cyttsp4_debug_release(struct cyttsp4_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_debug_data *dd = dev_get_drvdata(dev);
	int rc = 0;
	dev_dbg(dev, "%s\n", __func__);

	if (dev_get_drvdata(&ttsp->core->dev) == NULL) {
		dev_err(dev, "%s: Error, core driver does not exist. " \
			       "Unable to un-subscribe attention\n",
				__func__);
		goto cyttsp4_debug_release_exit;
	}

	rc = cyttsp4_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp4_debug_attention, CY_MODE_OPERATIONAL);
	if (rc < 0) {
		dev_err(dev, "%s: Error, could not un-subscribe attention\n",
				__func__);
		goto cyttsp4_debug_release_exit;
	}

cyttsp4_debug_release_exit:
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	device_remove_file(dev, &dev_attr_int_count);
	dev_set_drvdata(dev, NULL);
	kfree(dd);

	return rc;
}

struct cyttsp4_driver cyttsp4_debug_driver = {
	.probe = cyttsp4_debug_probe,
	.remove = cyttsp4_debug_release,
	.driver = {
		.name = CYTTSP4_DEBUG_NAME,
		.bus = &cyttsp4_bus_type,
		.owner = THIS_MODULE,
	},
};

struct cyttsp4_debug_platform_data
	_cyttsp4_debug_platform_data = {
	.debug_dev_name = CYTTSP4_DEBUG_NAME,
};

struct cyttsp4_device cyttsp4_debug_device = {
	.name = CYTTSP4_DEBUG_NAME,
	.core_id = "main_ttsp_core",
	.dev = {
		.platform_data = &_cyttsp4_debug_platform_data,
	}
};

static int __init cyttsp4_debug_init(void)
{
	int rc1;
	int rc2;

	rc1 = cyttsp4_register_device(&cyttsp4_debug_device);
	if (rc1)
		return -ENODEV;

	rc2 = cyttsp4_register_driver(&cyttsp4_debug_driver);
	if (rc2) {
		cyttsp4_unregister_device(&cyttsp4_debug_device);
		return -ENODEV;
	}

	pr_info("%s: %s (Built %s @ %s),rc1=%d rc2=%d\n", __func__,
			"Cypress TTSP Debug", __DATE__, __TIME__,
			rc1, rc2);
	return 0;
}
module_init(cyttsp4_debug_init);

static void __exit cyttsp4_debug_exit(void)
{
	cyttsp4_unregister_device(&cyttsp4_debug_device);
	cyttsp4_unregister_driver(&cyttsp4_debug_driver);
	pr_info("%s: module exit\n", __func__);
}
module_exit(cyttsp4_debug_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen debug driver");
MODULE_AUTHOR("Cypress Semiconductor");
