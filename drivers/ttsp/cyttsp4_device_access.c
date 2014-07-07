/*
 * cyttsp4_device_access.c
 * Cypress TrueTouch(TM) Standard Product V4 Device Access module.
 * Configuration and Test command/status user interface.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2012 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
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
#include <linux/cyttsp4_core.h>
#include <linux/cyttsp4_mt.h>

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
#include <linux/wait.h>
#include <linux/workqueue.h>
#include "cyttsp4_device_access.h"
#include "cyttsp4_regs.h"

#define CY_MAX_CONFIG_BYTES    256
#define CY_CMD_INDEX             0
#define CY_NULL_CMD_INDEX        1
#define CY_NULL_CMD_MODE_INDEX   2
#define CY_NULL_CMD_SIZE_INDEX   3
#define CY_NULL_CMD_SIZEL_INDEX  2
#define CY_NULL_CMD_SIZEH_INDEX  3

#define HI_BYTE(x)  (u8)(((x) >> 8) & 0xFF)
#define LOW_BYTE(x) (u8)((x) & 0xFF)

struct heatmap_param {
	bool scan_start;
	enum scanDataTypeList dataType; /* raw, base, diff */
	int numElement;
};

struct cyttsp4_device_access_data {
	struct cyttsp4_device *ttsp;
	struct cyttsp4_device_access_platform_data *pdata;
	struct cyttsp4_sysinfo *si;
	struct cyttsp4_test_mode_params test;
	struct mutex sysfs_lock;
	uint32_t ic_grpnum;
	uint32_t ic_grpoffset;
	bool own_exclusive;
	uint32_t ebid_row_size;
#ifdef VERBOSE_DEBUG
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
#endif
	wait_queue_head_t wait_q;
	u8 ic_buf[CY_MAX_PRBUF_SIZE];
	struct heatmap_param heatmap;
};

/*
 * cyttsp4_is_awakening_grpnum
 * Returns true if a grpnum requires being awake
 */
static bool cyttsp4_is_awakening_grpnum(int grpnum)
{
	int i;

	/* Array that lists which grpnums require being awake */
	static const int awakening_grpnums[] = {
		CY_IC_GRPNUM_CMD_REGS,
		CY_IC_GRPNUM_TEST_REGS,
	};

	for (i = 0; i < ARRAY_SIZE(awakening_grpnums); i++)
		if (awakening_grpnums[i] == grpnum)
			return true;

	return false;
}

/*
 * Show function prototype.
 * Returns response length or Linux error code on error.
 */
typedef int (*cyttsp4_show_function) (struct device *dev, u8 *ic_buf,
		size_t length);

/*
 * Store function prototype.
 * Returns Linux error code on error.
 */
typedef int (*cyttsp4_store_function) (struct device *dev, u8 *ic_buf,
		size_t length);

/*
 * grpdata show function to be used by
 * reserved and not implemented ic group numbers.
 */
int cyttsp4_grpdata_show_void (struct device *dev, u8 *ic_buf, size_t length)
{
	return -ENOSYS;
}

/*
 * grpdata store function to be used by
 * reserved and not implemented ic group numbers.
 */
int cyttsp4_grpdata_store_void (struct device *dev, u8 *ic_buf, size_t length)
{
	return -ENOSYS;
}

/*
 * SysFs group number entry show function.
 */
static ssize_t cyttsp4_ic_grpnum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int val = 0;

	mutex_lock(&dad->sysfs_lock);
	val = dad->ic_grpnum;
	mutex_unlock(&dad->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE, "Current Group: %d\n", val);
}

/*
 * SysFs group number entry store function.
 */
static ssize_t cyttsp4_ic_grpnum_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	unsigned long value;
	int prev_grpnum;
	int rc;

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		return size;
	}

	if (value >= CY_IC_GRPNUM_NUM) {
		dev_err(dev, "%s: Group %lu does not exist.\n",
				__func__, value);
		return size;
	}

	if (value > 0xFF)
		value = 0xFF;

	mutex_lock(&dad->sysfs_lock);
	/*
	 * Block grpnum change when own_exclusive flag is set
	 * which means the current grpnum implementation requires
	 * running exclusively on some consecutive grpdata operations
	 */
	if (dad->own_exclusive) {
		mutex_unlock(&dad->sysfs_lock);
		dev_err(dev, "%s: own_exclusive\n", __func__);
		return -EBUSY;
	}
	prev_grpnum = dad->ic_grpnum;
	dad->ic_grpnum = (int) value;
	mutex_unlock(&dad->sysfs_lock);

	/* Check whether the new grpnum requires being awake */
	if (cyttsp4_is_awakening_grpnum(prev_grpnum) !=
		cyttsp4_is_awakening_grpnum(value)) {
		if (cyttsp4_is_awakening_grpnum(value))
			pm_runtime_get(dev);
		else
			pm_runtime_put(dev);
	}

	dev_vdbg(dev, "%s: ic_grpnum=%d, return size=%d\n",
			__func__, (int)value, (int)size);
	return size;
}

static DEVICE_ATTR(ic_grpnum, S_IRUSR | S_IWUSR,
		   cyttsp4_ic_grpnum_show, cyttsp4_ic_grpnum_store);

/*
 * SysFs group offset entry show function.
 */
static ssize_t cyttsp4_ic_grpoffset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int val = 0;

	mutex_lock(&dad->sysfs_lock);
	val = dad->ic_grpoffset;
	mutex_unlock(&dad->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE, "Current Offset: %d\n", val);
}

/*
 * SysFs group offset entry store function.
 */
static ssize_t cyttsp4_ic_grpoffset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		return size;
	}

	if (value > 0xFFFF)
		value = 0xFFFF;

	mutex_lock(&dad->sysfs_lock);
	dad->ic_grpoffset = (int)value;
	mutex_unlock(&dad->sysfs_lock);

	dev_vdbg(dev, "%s: ic_grpoffset=%d, return size=%d\n", __func__,
			(int)value, (int)size);
	return size;
}

static DEVICE_ATTR(ic_grpoffset, S_IRUSR | S_IWUSR,
		   cyttsp4_ic_grpoffset_show, cyttsp4_ic_grpoffset_store);

/*
 * Prints part of communication registers.
 */
static int cyttsp4_grpdata_show_registers(struct device *dev, u8 *ic_buf,
		size_t length, int num_read, int offset, int mode)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int rc;

	if (dad->ic_grpoffset >= num_read)
		return -EINVAL;

	num_read -= dad->ic_grpoffset;

	if (length < num_read) {
		dev_err(dev, "%s: not sufficient buffer req_bug_len=%d, length=%d\n",
				__func__, num_read, length);
		return -EINVAL;
	}

	rc = cyttsp4_read(dad->ttsp, mode, offset + dad->ic_grpoffset, ic_buf,
			num_read);
	if (rc < 0)
		return rc;

	return num_read;
}

/*
 * SysFs grpdata show function implementation of group 1.
 * Prints status register contents of Operational mode registers.
 */
static int cyttsp4_grpdata_show_operational_regs(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.rep_ofs - dad->si->si_ofs.cmd_ofs;
	int offset = dad->si->si_ofs.cmd_ofs;

	return cyttsp4_grpdata_show_registers(dev, ic_buf, length, num_read,
			offset, CY_MODE_OPERATIONAL);
}

/*
 * SysFs grpdata show function implementation of group 2.
 * Prints current contents of the touch registers (full set).
 */
static int cyttsp4_grpdata_show_touch_regs(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.rep_sz;
	int offset = dad->si->si_ofs.rep_ofs;

	return cyttsp4_grpdata_show_registers(dev, ic_buf, length, num_read,
			offset, CY_MODE_OPERATIONAL);
}

/*
 * Prints some content of the system information
 */
static int cyttsp4_grpdata_show_sysinfo(struct device *dev, u8 *ic_buf,
		size_t length, int num_read, int offset)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int rc = 0, rc2 = 0, rc3 = 0;

	if (dad->ic_grpoffset >= num_read)
		return -EINVAL;

	num_read -= dad->ic_grpoffset;

	if (length < num_read) {
		dev_err(dev, "%s: not sufficient buffer req_bug_len=%d, length=%d\n",
				__func__, num_read, length);
		return -EINVAL;
	}

	rc = cyttsp4_request_exclusive(dad->ttsp,
			CY_DA_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		return rc;
	}

	rc = cyttsp4_request_set_mode(dad->ttsp, CY_MODE_SYSINFO);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request set mode r=%d\n",
				__func__, rc);
		goto cyttsp4_grpdata_show_sysinfo_err_release;
	}

	rc = cyttsp4_read(dad->ttsp, CY_MODE_SYSINFO,
			offset + dad->ic_grpoffset,
			ic_buf, num_read);
	if (rc < 0)
		dev_err(dev, "%s: Fail read cmd regs r=%d\n",
				__func__, rc);

	rc2 = cyttsp4_request_set_mode(dad->ttsp, CY_MODE_OPERATIONAL);
	if (rc2 < 0)
		dev_err(dev, "%s: Error on request set mode 2 r=%d\n",
				__func__, rc2);

cyttsp4_grpdata_show_sysinfo_err_release:
	rc3 = cyttsp4_release_exclusive(dad->ttsp);
	if (rc3 < 0) {
		dev_err(dev, "%s: Error on release exclusive r=%d\n",
				__func__, rc3);
		return rc3;
	}

	if (rc < 0)
		return rc;
	if (rc2 < 0)
		return rc2;

	return num_read;
}

/*
 * SysFs grpdata show function implementation of group 3.
 * Prints content of the system information DATA record.
 */
static int cyttsp4_grpdata_show_sysinfo_data_rec(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.cydata_size;
	int offset = dad->si->si_ofs.cydata_ofs;

	return cyttsp4_grpdata_show_sysinfo(dev, ic_buf, length, num_read,
			offset);
}

/*
 * SysFs grpdata show function implementation of group 4.
 * Prints content of the system information TEST record.
 */
static int cyttsp4_grpdata_show_sysinfo_test_rec(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.test_size;
	int offset = dad->si->si_ofs.test_ofs;

	return cyttsp4_grpdata_show_sysinfo(dev, ic_buf, length, num_read,
			offset);
}

/*
 * SysFs grpdata show function implementation of group 5.
 * Prints content of the system information PANEL data.
 */
static int cyttsp4_grpdata_show_sysinfo_panel(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.pcfg_size;
	int offset = dad->si->si_ofs.pcfg_ofs;

	return cyttsp4_grpdata_show_sysinfo(dev, ic_buf, length, num_read,
			offset);
}

/*
 * Get EBID Row Size is a Config mode command
 */
static int _cyttsp4_get_ebid_row_size(struct device *dev)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	u8 cmd_buf[CY_CMD_CAT_GET_CFG_BLK_SZ_CMD_SZ];
	u8 return_buf[CY_CMD_CAT_GET_CFG_BLK_SZ_RET_SZ];
	int rc;

	cmd_buf[0] = CY_CMD_CAT_GET_CFG_BLK_SZ;
	rc = cyttsp4_request_exec_cmd(dad->ttsp, CY_MODE_CAT,
			cmd_buf, CY_CMD_CAT_GET_CFG_BLK_SZ_CMD_SZ,
			return_buf, CY_CMD_CAT_GET_CFG_BLK_SZ_RET_SZ,
			CY_DA_COMMAND_COMPLETE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Unable to read EBID row size.\n", __func__);
		return rc;
	}

	dad->ebid_row_size = (return_buf[0] << 8) + return_buf[1];
	return rc;
}

/*
 * SysFs grpdata show function implementation of group 6.
 * Prints contents of the touch parameters a row at a time.
 */
static int cyttsp4_grpdata_show_touch_params(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	u8 cmd_buf[CY_CMD_CAT_READ_CFG_BLK_CMD_SZ];
	int return_buf_size = CY_CMD_CAT_READ_CFG_BLK_RET_SZ;
	int row_offset;
	int offset_in_single_row = 0;
	int rc;
	int rc2 = 0;
	int rc3;
	int i, j;

	rc = cyttsp4_request_exclusive(dad->ttsp,
			CY_DA_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		return rc;
	}

	rc = cyttsp4_request_set_mode(dad->ttsp, CY_MODE_CAT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request set mode r=%d\n",
				__func__, rc);
		goto cyttsp4_grpdata_show_touch_params_err_release;
	}

	if (dad->ebid_row_size == 0) {
		rc = _cyttsp4_get_ebid_row_size(dev);
		if (rc < 0)
			goto cyttsp4_grpdata_show_touch_params_err_change_mode;
	}

	/* Perform buffer size check since we have just acquired row size */
	return_buf_size += dad->ebid_row_size;

	if (length < return_buf_size) {
		dev_err(dev, "%s: not sufficient buffer " \
				"req_buf_len=%d, length=%d\n",
				__func__, return_buf_size, length);
		rc = -EINVAL;
		goto cyttsp4_grpdata_show_touch_params_err_change_mode;
	}

	row_offset = dad->ic_grpoffset / dad->ebid_row_size;

	cmd_buf[0] = CY_CMD_CAT_READ_CFG_BLK;
	cmd_buf[1] = HI_BYTE(row_offset);
	cmd_buf[2] = LOW_BYTE(row_offset);
	cmd_buf[3] = HI_BYTE(dad->ebid_row_size);
	cmd_buf[4] = LOW_BYTE(dad->ebid_row_size);
	cmd_buf[5] = CY_EBID;
	rc = cyttsp4_request_exec_cmd(dad->ttsp, CY_MODE_CAT,
			cmd_buf, CY_CMD_CAT_READ_CFG_BLK_CMD_SZ,
			ic_buf, return_buf_size,
			CY_DA_COMMAND_COMPLETE_TIMEOUT);

	offset_in_single_row = dad->ic_grpoffset % dad->ebid_row_size;

	/* Remove Header data from return buffer */
	for (i = 0, j = CY_CMD_CAT_READ_CFG_BLK_RET_HDR_SZ
				+ offset_in_single_row;
			i < (dad->ebid_row_size - offset_in_single_row);
			i++, j++)
		ic_buf[i] = ic_buf[j];

cyttsp4_grpdata_show_touch_params_err_change_mode:
	rc2 = cyttsp4_request_set_mode(dad->ttsp, CY_MODE_OPERATIONAL);
	if (rc2 < 0)
		dev_err(dev, "%s: Error on request set mode r=%d\n",
				__func__, rc2);

cyttsp4_grpdata_show_touch_params_err_release:
	rc3 = cyttsp4_release_exclusive(dad->ttsp);
	if (rc3 < 0) {
		dev_err(dev, "%s: Error on release exclusive r=%d\n",
				__func__, rc3);
		return rc3;
	}

	if (rc < 0)
		return rc;
	if (rc2 < 0)
		return rc2;

	return dad->ebid_row_size - offset_in_single_row;
}

/*
 * SysFs grpdata show function implementation of group 7.
 * Prints contents of the touch parameters sizes.
 */
static int cyttsp4_grpdata_show_touch_params_sizes(struct device *dev,
		u8 *ic_buf, size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	struct cyttsp4_core_platform_data *pdata =
			dev_get_platdata(&dad->ttsp->core->dev);
	int max_size;
	int block_start;
	int block_end;
	int num_read;

	if (pdata->sett[CY_IC_GRPNUM_TCH_PARM_SIZE] == NULL) {
		dev_err(dev, "%s: Missing platform data Touch Parameters" \
				" Sizes table\n", __func__);
		return -EINVAL;
	}

	if (pdata->sett[CY_IC_GRPNUM_TCH_PARM_SIZE]->data == NULL) {
		dev_err(dev, "%s: Missing platform data Touch Parameters" \
				" Sizes table data\n", __func__);
		return -EINVAL;
	}

	max_size = pdata->sett[CY_IC_GRPNUM_TCH_PARM_SIZE]->size;
	max_size *= sizeof(uint16_t);
	if (dad->ic_grpoffset >= max_size)
		return -EINVAL;

	block_start = (dad->ic_grpoffset / CYTTSP4_TCH_PARAM_SIZE_BLK_SZ)
			* CYTTSP4_TCH_PARAM_SIZE_BLK_SZ;
	block_end = CYTTSP4_TCH_PARAM_SIZE_BLK_SZ + block_start;
	if (block_end > max_size)
		block_end = max_size;
	num_read = block_end - dad->ic_grpoffset;
	if (length < num_read) {
		dev_err(dev, "%s: not sufficient buffer %s=%d, %s=%d\n",
				__func__, "req_buf_len", num_read, "length",
				length);
		return -EINVAL;
	}

	memcpy(ic_buf, (u8 *)pdata->sett[CY_IC_GRPNUM_TCH_PARM_SIZE]->data
			+ dad->ic_grpoffset, num_read);

	return num_read;
}

/*
 * SysFs grpdata show function implementation of group 10.
 * Prints content of the system information Operational Configuration data.
 */
static int cyttsp4_grpdata_show_sysinfo_opcfg(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.opcfg_size;
	int offset = dad->si->si_ofs.opcfg_ofs;

	return cyttsp4_grpdata_show_sysinfo(dev, ic_buf, length, num_read,
			offset);
}

/*
 * SysFs grpdata show function implementation of group 11.
 * Prints content of the system information Design data.
 */
static int cyttsp4_grpdata_show_sysinfo_design(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.ddata_size;
	int offset = dad->si->si_ofs.ddata_ofs;

	return cyttsp4_grpdata_show_sysinfo(dev, ic_buf, length, num_read,
			offset);
}

/*
 * SysFs grpdata show function implementation of group 12.
 * Prints content of the system information Manufacturing data.
 */
static int cyttsp4_grpdata_show_sysinfo_manufacturing(struct device *dev,
		u8 *ic_buf, size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int num_read = dad->si->si_ofs.mdata_size;
	int offset = dad->si->si_ofs.mdata_ofs;

	return cyttsp4_grpdata_show_sysinfo(dev, ic_buf, length, num_read,
			offset);
}

/*
 * SysFs grpdata show function implementation of group 13.
 * Prints status register contents of Configuration and
 * Test registers.
 */
static int cyttsp4_grpdata_show_test_regs(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	u8 mode;
	int rc = 0;
	int num_read = 0;

	dev_vdbg(dev, "%s: test.cur_cmd=%d test.cur_mode=%d\n",
			__func__, dad->test.cur_cmd, dad->test.cur_mode);

	if (dad->test.cur_cmd == CY_CMD_CAT_NULL) {
		num_read = 1;
		if (length < num_read) {
			dev_err(dev, "%s: not sufficient buffer %s=%d, %s=%d\n",
					__func__, "req_buf_len", num_read,
					"length", length);
			return -EINVAL;
		}

		dev_vdbg(dev, "%s: GRP=TEST_REGS: NULL CMD: host_mode=%02X\n",
				__func__, ic_buf[0]);
		rc = cyttsp4_read(dad->ttsp,
				dad->test.cur_mode == CY_TEST_MODE_CAT ?
					CY_MODE_CAT : CY_MODE_OPERATIONAL,
				CY_REG_BASE, &mode, sizeof(mode));
		if (rc < 0) {
			ic_buf[0] = 0xFF;
			dev_err(dev, "%s: failed to read host mode r=%d\n",
					__func__, rc);
		} else {
			ic_buf[0] = mode;
		}
	} else if (dad->test.cur_mode == CY_TEST_MODE_CAT) {
		num_read = dad->test.cur_status_size;
		if (length < num_read) {
			dev_err(dev, "%s: not sufficient buffer %s=%d, %s=%d\n",
					__func__, "req_buf_len", num_read,
					"length", length);
			return -EINVAL;
		}
		dev_vdbg(dev, "%s: GRP=TEST_REGS: num_rd=%d at ofs=%d + " \
				"grpofs=%d\n", __func__, num_read,
				dad->si->si_ofs.cmd_ofs, dad->ic_grpoffset);
		rc = cyttsp4_read(dad->ttsp, CY_MODE_CAT,
				dad->ic_grpoffset + dad->si->si_ofs.cmd_ofs,
				ic_buf, num_read);
		if (rc < 0)
			return rc;
	} else {
		dev_err(dev, "%s: Not in Config/Test mode\n", __func__);
	}

	return num_read;
}

/*
 * SysFs grpdata show function implementation of group 14.
 * Prints CapSense button keycodes.
 */
static int cyttsp4_grpdata_show_btn_keycodes(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	struct cyttsp4_btn *btn = dad->si->btn;
	int num_btns = dad->si->si_ofs.num_btns - dad->ic_grpoffset;
	int n;

	if (num_btns <= 0 || btn == NULL || length < num_btns)
		return -EINVAL;

	for (n = 0; n < num_btns; n++)
		ic_buf[n] = (u8) btn[dad->ic_grpoffset + n].key_code;

	return n;
}

/*
 * SysFs grpdata show function implementation of group 15.
 * Prints status register contents of Configuration and
 * Test registers.
 */
static int cyttsp4_grpdata_show_tthe_test_regs(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int rc = 0;
	int num_read = 0;

	dev_vdbg(dev, "%s: test.cur_cmd=%d test.cur_mode=%d\n",
			__func__, dad->test.cur_cmd, dad->test.cur_mode);

	if (dad->test.cur_cmd == CY_CMD_CAT_NULL) {
		num_read = dad->test.cur_status_size;
		if (length < num_read) {
			dev_err(dev, "%s: not sufficient buffer %s=%d, %s=%d\n",
					__func__, "req_buf_len", num_read,
					"length", length);
			return -EINVAL;
		}

		dev_vdbg(dev, "%s: GRP=TEST_REGS: NULL CMD: host_mode=%02X\n",
				__func__, ic_buf[0]);
		rc = cyttsp4_read(dad->ttsp,
				(dad->test.cur_mode == CY_TEST_MODE_CAT)
					? CY_MODE_CAT :
				(dad->test.cur_mode == CY_TEST_MODE_SYSINFO)
					? CY_MODE_SYSINFO : CY_MODE_OPERATIONAL,
				CY_REG_BASE, ic_buf, num_read);
		if (rc < 0) {
			ic_buf[0] = 0xFF;
			dev_err(dev, "%s: failed to read host mode r=%d\n",
					__func__, rc);
		}
	} else if (dad->test.cur_mode == CY_TEST_MODE_CAT
			|| dad->test.cur_mode == CY_TEST_MODE_SYSINFO) {
		num_read = dad->test.cur_status_size;
		if (length < num_read) {
			dev_err(dev, "%s: not sufficient buffer %s=%d, %s=%d\n",
					__func__, "req_buf_len", num_read,
					"length", length);
			return -EINVAL;
		}
		dev_vdbg(dev, "%s: GRP=TEST_REGS: num_rd=%d at ofs=%d + " \
				"grpofs=%d\n", __func__, num_read,
				dad->si->si_ofs.cmd_ofs, dad->ic_grpoffset);
		rc = cyttsp4_read(dad->ttsp,
				(dad->test.cur_mode == CY_TEST_MODE_CAT)
					? CY_MODE_CAT : CY_MODE_SYSINFO,
				CY_REG_BASE, ic_buf, num_read);
		if (rc < 0)
			return rc;
	} else {
		dev_err(dev, "%s: In unsupported mode\n", __func__);
	}

	return num_read;
}

static cyttsp4_show_function
		cyttsp4_grpdata_show_functions[CY_IC_GRPNUM_NUM] = {
	[CY_IC_GRPNUM_RESERVED] = cyttsp4_grpdata_show_void,
	[CY_IC_GRPNUM_CMD_REGS] = cyttsp4_grpdata_show_operational_regs,
	[CY_IC_GRPNUM_TCH_REP] = cyttsp4_grpdata_show_touch_regs,
	[CY_IC_GRPNUM_DATA_REC] = cyttsp4_grpdata_show_sysinfo_data_rec,
	[CY_IC_GRPNUM_TEST_REC] = cyttsp4_grpdata_show_sysinfo_test_rec,
	[CY_IC_GRPNUM_PCFG_REC] = cyttsp4_grpdata_show_sysinfo_panel,
	[CY_IC_GRPNUM_TCH_PARM_VAL] = cyttsp4_grpdata_show_touch_params,
	[CY_IC_GRPNUM_TCH_PARM_SIZE] = cyttsp4_grpdata_show_touch_params_sizes,
	[CY_IC_GRPNUM_RESERVED1] = cyttsp4_grpdata_show_void,
	[CY_IC_GRPNUM_RESERVED2] = cyttsp4_grpdata_show_void,
	[CY_IC_GRPNUM_OPCFG_REC] = cyttsp4_grpdata_show_sysinfo_opcfg,
	[CY_IC_GRPNUM_DDATA_REC] = cyttsp4_grpdata_show_sysinfo_design,
	[CY_IC_GRPNUM_MDATA_REC] = cyttsp4_grpdata_show_sysinfo_manufacturing,
	[CY_IC_GRPNUM_TEST_REGS] = cyttsp4_grpdata_show_test_regs,
	[CY_IC_GRPNUM_BTN_KEYS] = cyttsp4_grpdata_show_btn_keycodes,
	[CY_IC_GRPNUM_TTHE_REGS] = cyttsp4_grpdata_show_tthe_test_regs,
};

static ssize_t cyttsp4_ic_grpdata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int i;
	ssize_t num_read;
	int index;

	mutex_lock(&dad->sysfs_lock);
	dev_vdbg(dev, "%s: grpnum=%d grpoffset=%u\n",
			__func__, dad->ic_grpnum, dad->ic_grpoffset);

	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Group %d, Offset %u:\n", dad->ic_grpnum,
			dad->ic_grpoffset);

	pm_runtime_get_sync(dev);
	num_read = cyttsp4_grpdata_show_functions[dad->ic_grpnum] (dev,
			dad->ic_buf, CY_MAX_PRBUF_SIZE);
	pm_runtime_put(dev);
	if (num_read < 0) {
		index = num_read;
		if (num_read == -ENOSYS) {
			dev_err(dev, "%s: Group %d is not implemented.\n",
				__func__, dad->ic_grpnum);
			goto cyttsp4_ic_grpdata_show_error;
		}
		dev_err(dev, "%s: Cannot read Group %d Data.\n",
				__func__, dad->ic_grpnum);
		goto cyttsp4_ic_grpdata_show_error;
	}

	for (i = 0; i < num_read; i++) {
		index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
				"0x%02X\n", dad->ic_buf[i]);
	}

	index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
			"(%d bytes)\n", num_read);

cyttsp4_ic_grpdata_show_error:
	mutex_unlock(&dad->sysfs_lock);
	return index;
}

static int _cyttsp4_cmd_handshake(struct cyttsp4_device_access_data *dad)
{
	struct device *dev = &dad->ttsp->dev;
	u8 mode;
	int rc;

	rc = cyttsp4_read(dad->ttsp, CY_MODE_CAT,
			CY_REG_BASE, &mode, sizeof(mode));
	if (rc < 0) {
		dev_err(dev, "%s: Fail read host mode r=%d\n", __func__, rc);
		return rc;
	}

	rc = cyttsp4_request_handshake(dad->ttsp, mode);
	if (rc < 0)
		dev_err(dev, "%s: Fail cmd handshake r=%d\n", __func__, rc);

	return rc;
}

static int _cyttsp4_cmd_toggle_lowpower(struct cyttsp4_device_access_data *dad)
{
	struct device *dev = &dad->ttsp->dev;
	u8 mode;
	int rc = cyttsp4_read(dad->ttsp,
			(dad->test.cur_mode == CY_TEST_MODE_CAT)
				? CY_MODE_CAT : CY_MODE_OPERATIONAL,
			CY_REG_BASE, &mode, sizeof(mode));
	if (rc < 0) {
		dev_err(dev, "%s: Fail read host mode r=%d\n",
				__func__, rc);
		return rc;
	}

	rc = cyttsp4_request_toggle_lowpower(dad->ttsp, mode);
	if (rc < 0)
		dev_err(dev, "%s: Fail cmd handshake r=%d\n",
				__func__, rc);
	return rc;
}

static int cyttsp4_test_cmd_mode(struct cyttsp4_device_access_data *dad,
		u8 *ic_buf, size_t length)
{
	struct device *dev = &dad->ttsp->dev;
	int rc = -ENOSYS;
	u8 mode;

	if (length < CY_NULL_CMD_MODE_INDEX + 1)  {
		dev_err(dev, "%s: %s length=%d\n", __func__,
				"Buffer length is not valid", length);
		return -EINVAL;
	}
	mode = ic_buf[CY_NULL_CMD_MODE_INDEX];

	if (mode == CY_HST_CAT) {
		rc = cyttsp4_request_exclusive(dad->ttsp,
				CY_REQUEST_EXCLUSIVE_TIMEOUT);
		if (rc < 0) {
			dev_err(dev, "%s: Fail rqst exclusive r=%d\n",
					__func__, rc);
			goto cyttsp4_test_cmd_mode_exit;
		}
		rc = cyttsp4_request_set_mode(dad->ttsp, CY_MODE_CAT);
		if (rc < 0) {
			dev_err(dev, "%s: Fail rqst set mode=%02X r=%d\n",
					__func__, mode, rc);
			rc = cyttsp4_release_exclusive(dad->ttsp);
			if (rc < 0)
				dev_err(dev, "%s: %s r=%d\n", __func__,
						"Fail release exclusive", rc);
			goto cyttsp4_test_cmd_mode_exit;
		}
		dad->test.cur_mode = CY_TEST_MODE_CAT;
		dad->own_exclusive = true;
		dev_vdbg(dev, "%s: %s=%d %s=%02X %s=%d(CaT)\n", __func__,
				"own_exclusive", dad->own_exclusive == true,
				"mode", mode, "test.cur_mode",
				dad->test.cur_mode);
	} else if (mode == CY_HST_OPERATE) {
		if (dad->own_exclusive) {
			rc = cyttsp4_request_set_mode(dad->ttsp,
					CY_MODE_OPERATIONAL);
			if (rc < 0)
				dev_err(dev, "%s: %s=%02X r=%d\n", __func__,
						"Fail rqst set mode", mode, rc);
				/* continue anyway */

			rc = cyttsp4_release_exclusive(dad->ttsp);
			if (rc < 0) {
				dev_err(dev, "%s: %s r=%d\n", __func__,
						"Fail release exclusive", rc);
				/* continue anyway */
				rc = 0;
			}
			dad->test.cur_mode = CY_TEST_MODE_NORMAL_OP;
			dad->own_exclusive = false;
			dev_vdbg(dev, "%s: %s=%d %s=%02X %s=%d(Operate)\n",
					__func__, "own_exclusive",
					dad->own_exclusive == true,
					"mode", mode,
					"test.cur_mode", dad->test.cur_mode);
		} else
			dev_vdbg(dev, "%s: %s mode=%02X(Operate)\n", __func__,
					"do not own exclusive; cannot switch",
					mode);
	} else
		dev_vdbg(dev, "%s: unsupported mode switch=%02X\n",
				__func__, mode);

cyttsp4_test_cmd_mode_exit:
	return rc;
}

static int cyttsp4_test_tthe_cmd_mode(struct cyttsp4_device_access_data *dad,
		u8 *ic_buf, size_t length)
{
	struct device *dev = &dad->ttsp->dev;
	int rc = -ENOSYS;
	u8 mode;
	enum cyttsp4_test_mode test_mode;
	int new_mode;

	if (length < CY_NULL_CMD_MODE_INDEX + 1)  {
		dev_err(dev, "%s: %s length=%d\n", __func__,
				"Buffer length is not valid", length);
		return -EINVAL;
	}
	mode = ic_buf[CY_NULL_CMD_MODE_INDEX];

	switch (mode) {
	case CY_HST_CAT:
		new_mode = CY_MODE_CAT;
		test_mode = CY_TEST_MODE_CAT;
		break;
	case CY_HST_OPERATE:
		new_mode = CY_MODE_OPERATIONAL;
		test_mode = CY_TEST_MODE_NORMAL_OP;
		break;
	case CY_HST_SYSINFO:
		new_mode = CY_MODE_SYSINFO;
		test_mode = CY_TEST_MODE_SYSINFO;
		break;
	default:
		dev_vdbg(dev, "%s: unsupported mode switch=%02X\n",
				__func__, mode);
		goto cyttsp4_test_tthe_cmd_mode_exit;
	}

	rc = cyttsp4_request_exclusive(dad->ttsp, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Fail rqst exclusive r=%d\n", __func__, rc);
		goto cyttsp4_test_tthe_cmd_mode_exit;
	}
	rc = cyttsp4_request_set_mode(dad->ttsp, new_mode);
	if (rc < 0)
		dev_err(dev, "%s: Fail rqst set mode=%02X r=%d\n",
				__func__, mode, rc);
	rc = cyttsp4_release_exclusive(dad->ttsp);
	if (rc < 0) {
		dev_err(dev, "%s: %s r=%d\n", __func__,
				"Fail release exclusive", rc);
		if (mode == CY_HST_OPERATE)
			rc = 0;
		else
			goto cyttsp4_test_tthe_cmd_mode_exit;
	}
	dad->test.cur_mode = test_mode;
	dev_vdbg(dev, "%s: %s=%d %s=%02X %s=%d\n", __func__,
			"own_exclusive", dad->own_exclusive == true,
			"mode", mode,
			"test.cur_mode", dad->test.cur_mode);

cyttsp4_test_tthe_cmd_mode_exit:
	return rc;
}

/*
 * SysFs grpdata store function implementation of group 1.
 * Stores to command and parameter registers of Operational mode.
 */
static int cyttsp4_grpdata_store_operational_regs(struct device *dev,
		u8 *ic_buf, size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	size_t cmd_ofs = dad->si->si_ofs.cmd_ofs;
	int rc, rc2;

	if ((cmd_ofs + dad->ic_grpoffset + length) > dad->si->si_ofs.rep_ofs)
		return -EINVAL;

	rc = cyttsp4_request_exclusive(dad->ttsp,
			CY_DA_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		return rc;
	}

	rc = cyttsp4_write(dad->ttsp, CY_MODE_OPERATIONAL,
			cmd_ofs + dad->ic_grpoffset, ic_buf, length);
	if (rc < 0)
		dev_err(dev, "%s: Fail write cmd regs r=%d\n", __func__, rc);

	rc2 = cyttsp4_release_exclusive(dad->ttsp);
	if (rc2 < 0) {
		dev_err(dev, "%s: Error on release exclusive r=%d\n",
				__func__, rc2);
		return rc2;
	}

	return rc;
}

/*
 * SysFs store function of Test Regs group.
 */
static int cyttsp4_grpdata_store_test_regs(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int rc;

	/* Caller function guaranties, length is not bigger than ic_buf size */
	if (length < CY_CMD_INDEX + 1) {
		dev_err(dev, "%s: %s length=%d\n", __func__,
				"Buffer length is not valid", length);
		return -EINVAL;
	}

	dad->test.cur_cmd = ic_buf[CY_CMD_INDEX];
	if (dad->test.cur_cmd == CY_CMD_CAT_NULL) {
		if (length < CY_NULL_CMD_INDEX + 1) {
			dev_err(dev, "%s: %s length=%d\n", __func__,
					"Buffer length is not valid", length);
			return -EINVAL;
		}
		dev_vdbg(dev, "%s: test-cur_cmd=%d null-cmd=%d\n", __func__,
				dad->test.cur_cmd, ic_buf[CY_NULL_CMD_INDEX]);
		switch (ic_buf[CY_NULL_CMD_INDEX]) {
		case CY_NULL_CMD_NULL:
			dev_err(dev, "%s: empty NULL cmd\n", __func__);
			break;
		case CY_NULL_CMD_MODE:
			if (length < CY_NULL_CMD_MODE_INDEX + 1) {
				dev_err(dev, "%s: %s length=%d\n", __func__,
						"Buffer length is not valid",
						length);
				return -EINVAL;
			}
			dev_vdbg(dev, "%s: Set cmd mode=%02X\n", __func__,
					ic_buf[CY_NULL_CMD_MODE_INDEX]);
			cyttsp4_test_cmd_mode(dad, ic_buf, length);
			break;
		case CY_NULL_CMD_STATUS_SIZE:
			if (length < CY_NULL_CMD_SIZE_INDEX + 1) {
				dev_err(dev, "%s: %s length=%d\n", __func__,
						"Buffer length is not valid",
						length);
				return -EINVAL;
			}
			dad->test.cur_status_size =
				ic_buf[CY_NULL_CMD_SIZEL_INDEX]
				+ (ic_buf[CY_NULL_CMD_SIZEH_INDEX] << 8);
			dev_vdbg(dev, "%s: test-cur_status_size=%d\n",
					__func__, dad->test.cur_status_size);
			break;
		case CY_NULL_CMD_HANDSHAKE:
			dev_vdbg(dev, "%s: try null cmd handshake\n",
					__func__);
			rc = _cyttsp4_cmd_handshake(dad);
			if (rc < 0)
				dev_err(dev, "%s: %s r=%d\n", __func__,
						"Fail test cmd handshake", rc);
		default:
			break;
		}
	} else {
		dev_dbg(dev, "%s: TEST CMD=0x%02X length=%d %s%d\n",
				__func__, ic_buf[0], length, "cmd_ofs+grpofs=",
				dad->ic_grpoffset + dad->si->si_ofs.cmd_ofs);
		cyttsp4_pr_buf(dev, dad->pr_buf, ic_buf, length, "test_cmd");
		rc = cyttsp4_write(dad->ttsp, CY_MODE_CAT,
				   dad->ic_grpoffset + dad->si->si_ofs.cmd_ofs,
				   ic_buf, length);
		if (rc < 0)
			dev_err(dev, "%s: Fail write cmd regs r=%d\n",
					__func__, rc);
	}
	return 0;
}

/*
 * SysFs store function of Test Regs group.
 */
static int cyttsp4_grpdata_store_tthe_test_regs(struct device *dev, u8 *ic_buf,
		size_t length)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	int rc;

	/* Caller function guaranties, length is not bigger than ic_buf size */
	if (length < CY_CMD_INDEX + 1) {
		dev_err(dev, "%s: %s length=%d\n", __func__,
				"Buffer length is not valid", length);
		return -EINVAL;
	}

	dad->test.cur_cmd = ic_buf[CY_CMD_INDEX];
	if (dad->test.cur_cmd == CY_CMD_CAT_NULL) {
		if (length < CY_NULL_CMD_INDEX + 1) {
			dev_err(dev, "%s: %s length=%d\n", __func__,
					"Buffer length is not valid", length);
			return -EINVAL;
		}
		dev_vdbg(dev, "%s: test-cur_cmd=%d null-cmd=%d\n", __func__,
				dad->test.cur_cmd, ic_buf[CY_NULL_CMD_INDEX]);
		switch (ic_buf[CY_NULL_CMD_INDEX]) {
		case CY_NULL_CMD_NULL:
			dev_err(dev, "%s: empty NULL cmd\n", __func__);
			break;
		case CY_NULL_CMD_MODE:
			if (length < CY_NULL_CMD_MODE_INDEX + 1) {
				dev_err(dev, "%s: %s length=%d\n", __func__,
						"Buffer length is not valid",
						length);
				return -EINVAL;
			}
			dev_vdbg(dev, "%s: Set cmd mode=%02X\n", __func__,
					ic_buf[CY_NULL_CMD_MODE_INDEX]);
			cyttsp4_test_tthe_cmd_mode(dad, ic_buf, length);
			break;
		case CY_NULL_CMD_STATUS_SIZE:
			if (length < CY_NULL_CMD_SIZE_INDEX + 1) {
				dev_err(dev, "%s: %s length=%d\n", __func__,
						"Buffer length is not valid",
						length);
				return -EINVAL;
			}
			dad->test.cur_status_size =
				ic_buf[CY_NULL_CMD_SIZEL_INDEX]
				+ (ic_buf[CY_NULL_CMD_SIZEH_INDEX] << 8);
			dev_vdbg(dev, "%s: test-cur_status_size=%d\n",
					__func__, dad->test.cur_status_size);
			break;
		case CY_NULL_CMD_HANDSHAKE:
			dev_vdbg(dev, "%s: try null cmd handshake\n",
					__func__);
			rc = _cyttsp4_cmd_handshake(dad);
			if (rc < 0)
				dev_err(dev, "%s: %s r=%d\n", __func__,
						"Fail test cmd handshake", rc);
		case CY_NULL_CMD_LOW_POWER:
			dev_vdbg(dev, "%s: try null cmd low power\n", __func__);
			rc = _cyttsp4_cmd_toggle_lowpower(dad);
			if (rc < 0)
				dev_err(dev, "%s: %s r=%d\n", __func__,
					"Fail test cmd toggle low power", rc);
		default:
			break;
		}
	} else {
		dev_dbg(dev, "%s: TEST CMD=0x%02X length=%d %s%d\n",
				__func__, ic_buf[0], length, "cmd_ofs+grpofs=",
				dad->ic_grpoffset + dad->si->si_ofs.cmd_ofs);
		cyttsp4_pr_buf(dev, dad->pr_buf, ic_buf, length, "test_cmd");
		/* Support Operating mode command. */
		rc = cyttsp4_write(dad->ttsp,
				(dad->test.cur_mode == CY_TEST_MODE_CAT)
					?  CY_MODE_CAT : CY_MODE_OPERATIONAL,
				dad->ic_grpoffset + dad->si->si_ofs.cmd_ofs,
				ic_buf, length);
		if (rc < 0)
			dev_err(dev, "%s: Fail write cmd regs r=%d\n",
					__func__, rc);
	}
	return 0;
}

/*
 * Gets user input from sysfs and parse it
 * return size of parsed output buffer
 */
static int cyttsp4_ic_parse_input(struct device *dev, const char *buf,
		size_t buf_size, u8 *ic_buf, size_t ic_buf_size)
{
	const char *pbuf = buf;
	unsigned long value;
	char scan_buf[CYTTSP4_INPUT_ELEM_SZ];
	int i = 0;
	int j;
	int last = 0;
	int ret;

	dev_dbg(dev, "%s: pbuf=%p buf=%p size=%d %s=%d buf=%s\n", __func__,
			pbuf, buf, (int) buf_size, "scan buf size",
			CYTTSP4_INPUT_ELEM_SZ, buf);

	while (pbuf <= (buf + buf_size)) {
		if (i >= CY_MAX_CONFIG_BYTES) {
			dev_err(dev, "%s: %s size=%d max=%d\n", __func__,
					"Max cmd size exceeded", i,
					CY_MAX_CONFIG_BYTES);
			return -EINVAL;
		}
		if (i >= ic_buf_size) {
			dev_err(dev, "%s: %s size=%d buf_size=%d\n", __func__,
					"Buffer size exceeded", i, ic_buf_size);
			return -EINVAL;
		}
		while (((*pbuf == ' ') || (*pbuf == ','))
				&& (pbuf < (buf + buf_size))) {
			last = *pbuf;
			pbuf++;
		}

		if (pbuf >= (buf + buf_size))
			break;

		memset(scan_buf, 0, CYTTSP4_INPUT_ELEM_SZ);
		if ((last == ',') && (*pbuf == ',')) {
			dev_err(dev, "%s: %s \",,\" not allowed.\n", __func__,
					"Invalid data format.");
			return -EINVAL;
		}
		for (j = 0; j < (CYTTSP4_INPUT_ELEM_SZ - 1)
				&& (pbuf < (buf + buf_size))
				&& (*pbuf != ' ')
				&& (*pbuf != ','); j++) {
			last = *pbuf;
			scan_buf[j] = *pbuf++;
		}

		ret = kstrtoul(scan_buf, 16, &value);
		if (ret < 0) {
			dev_err(dev, "%s: %s '%s' %s%s i=%d r=%d\n", __func__,
					"Invalid data format. ", scan_buf,
					"Use \"0xHH,...,0xHH\"", " instead.",
					i, ret);
			return ret;
		}

		ic_buf[i] = value;
		i++;
	}

	return i;
}

/*
 * SysFs store functions of each group member.
 */
static cyttsp4_store_function
		cyttsp4_grpdata_store_functions[CY_IC_GRPNUM_NUM] = {
	[CY_IC_GRPNUM_RESERVED] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_CMD_REGS] = cyttsp4_grpdata_store_operational_regs,
	[CY_IC_GRPNUM_TCH_REP] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_DATA_REC] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_TEST_REC] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_PCFG_REC] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_TCH_PARM_VAL] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_TCH_PARM_SIZE] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_RESERVED1] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_RESERVED2] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_OPCFG_REC] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_DDATA_REC] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_MDATA_REC] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_TEST_REGS] = cyttsp4_grpdata_store_test_regs,
	[CY_IC_GRPNUM_BTN_KEYS] = cyttsp4_grpdata_store_void,
	[CY_IC_GRPNUM_TTHE_REGS] = cyttsp4_grpdata_store_tthe_test_regs,
};

static ssize_t cyttsp4_ic_grpdata_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	ssize_t length;
	int rc;

	mutex_lock(&dad->sysfs_lock);
	length = cyttsp4_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		dev_err(dev, "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto cyttsp4_ic_grpdata_store_exit;
	}

	dev_vdbg(dev, "%s: grpnum=%d grpoffset=%u\n",
			__func__, dad->ic_grpnum, dad->ic_grpoffset);

	if (dad->ic_grpnum >= CY_IC_GRPNUM_NUM) {
		dev_err(dev, "%s: Group %d does not exist.\n",
				__func__, dad->ic_grpnum);
		goto cyttsp4_ic_grpdata_store_exit;
	}

	/* write ic_buf to log */
	cyttsp4_pr_buf(dev, dad->pr_buf, dad->ic_buf, length, "ic_buf");

	pm_runtime_get_sync(dev);
	/* Call relevant store handler. */
	rc = cyttsp4_grpdata_store_functions[dad->ic_grpnum] (dev, dad->ic_buf,
			length);
	pm_runtime_put(dev);
	if (rc < 0)
		dev_err(dev, "%s: Failed to store for grpmun=%d.\n",
				__func__, dad->ic_grpnum);

cyttsp4_ic_grpdata_store_exit:
	mutex_unlock(&dad->sysfs_lock);
	dev_vdbg(dev, "%s: return size=%d\n", __func__, size);
	return size;
}

static DEVICE_ATTR(ic_grpdata, S_IRUSR | S_IWUSR,
	cyttsp4_ic_grpdata_show, cyttsp4_ic_grpdata_store);

/*
 * Execute scan command
 */
static int _cyttsp4_exec_scan_cmd(struct device *dev)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	u8 cmd_buf[CY_CMD_CAT_GET_CFG_BLK_SZ_CMD_SZ];
	u8 return_buf[CY_CMD_CAT_GET_CFG_BLK_SZ_RET_SZ];
	int rc;

	cmd_buf[0] = CY_CMD_CAT_EXEC_PANEL_SCAN;
	rc = cyttsp4_request_exec_cmd(dad->ttsp, CY_MODE_CAT,
			cmd_buf, CY_CMD_CAT_EXEC_SCAN_CMD_SZ,
			return_buf, CY_CMD_CAT_EXEC_SCAN_RET_SZ,
			CY_DA_COMMAND_COMPLETE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Unable to send execute panel scan command.\n",
				__func__);
		return rc;
	}

	if (return_buf[0] != 0)
		return -EINVAL;
	return rc;
}

/*
 * Retrieve panel data command
 */
static int _cyttsp4_ret_scan_data_cmd(struct device *dev, int readOffset,
		int numElement, u8 dataType, u8 *return_buf)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	u8 cmd_buf[CY_CMD_CAT_READ_CFG_BLK_CMD_SZ];
	int rc;

	cmd_buf[0] = CY_CMD_CAT_RETRIEVE_PANEL_SCAN;
	cmd_buf[1] = HI_BYTE(readOffset);
	cmd_buf[2] = LOW_BYTE(readOffset);
	cmd_buf[3] = HI_BYTE(numElement);
	cmd_buf[4] = LOW_BYTE(numElement);
	cmd_buf[5] = dataType;
	rc = cyttsp4_request_exec_cmd(dad->ttsp, CY_MODE_CAT,
			cmd_buf, CY_CMD_CAT_RET_PANEL_DATA_CMD_SZ,
			return_buf, CY_CMD_CAT_RET_PANEL_DATA_RET_SZ,
			CY_DA_COMMAND_COMPLETE_TIMEOUT);
	if (rc < 0)
		return rc;

	if (return_buf[0] != 0)
		return -EINVAL;
	return rc;
}

/*
 * SysFs grpdata show function implementation of group 6.
 * Prints contents of the touch parameters a row at a time.
 */
static ssize_t cyttsp4_get_panel_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	u8 return_buf[CY_CMD_CAT_RET_PANEL_DATA_RET_SZ];

	int rc = 0;
	int rc1 = 0;
	int dataIdx = -1;
	int i = 0;
	int printIdx = -1;
	u8 cmdParam_ofs = dad->si->si_ofs.cmd_ofs + 1;
	int readByte = CY_CMD_CAT_RET_PANEL_DATA_RET_SZ
			+ (dad->si->si_ofs.cmd_ofs + 1);
	int leftOverElement = 0;
	int returnedElement = 0;
	int readElementOffset = 0;
	u8 elementStartOffset = dad->si->si_ofs.cmd_ofs + 1
			+ CY_CMD_CAT_RET_PANEL_DATA_RET_SZ;
	u8 elementSize = 1;
	int maxElmtSize = I2C_BUF_MAX_SIZE - elementStartOffset;

	rc = cyttsp4_request_exclusive(dad->ttsp,
			CY_DA_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto cyttsp4_get_panel_data_show_err_release;
	}

	if (dad->heatmap.scan_start)	{
		/* Start scan */
		rc = _cyttsp4_exec_scan_cmd(dev);
		if (rc < 0)
			goto cyttsp4_get_panel_data_show_err_release;
	}
	/* retrieve scan data */
	rc = _cyttsp4_ret_scan_data_cmd(dev, CY_CMD_IN_DATA_OFFSET_VALUE,
			dad->heatmap.numElement, dad->heatmap.dataType,
			return_buf);

	if (rc < 0)
		goto cyttsp4_get_panel_data_show_err_release;
	if (return_buf[CY_CMD_OUT_STATUS_OFFSET] != CY_CMD_STATUS_SUCCESS)
		goto cyttsp4_get_panel_data_show_err_release;

	/* read data */
	elementSize = (return_buf[CY_CMD_RET_PNL_OUT_DATA_FORMAT_OFFS] &
			CY_CMD_RET_PANEL_ELMNT_SZ_MASK);
	readByte += (dad->heatmap.numElement *
			elementSize);

	if (readByte >= I2C_BUF_MAX_SIZE) {
		rc = cyttsp4_read(dad->ttsp, CY_MODE_CAT, 0, dad->ic_buf,
				I2C_BUF_MAX_SIZE);
		dataIdx = I2C_BUF_MAX_SIZE;
	} else {
		rc = cyttsp4_read(dad->ttsp, CY_MODE_CAT, 0, dad->ic_buf,
				readByte);
		dataIdx = readByte;
	}
	if (rc < 0) {
		dev_err(dev, "%s: Error on read r=%d\n", __func__, dataIdx);
		goto cyttsp4_get_panel_data_show_err_release;
	}

	if (readByte < I2C_BUF_MAX_SIZE)
		goto cyttsp4_get_panel_data_show_err_release;

	maxElmtSize = maxElmtSize / elementSize;
	leftOverElement = dad->heatmap.numElement;

	returnedElement =
			return_buf[CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_H] * 256
			+ return_buf[CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_L];
	returnedElement = (returnedElement > maxElmtSize) ?
				maxElmtSize : returnedElement;

	leftOverElement -= returnedElement;
	readElementOffset += returnedElement;
	do {
		/* get the data */
		rc = _cyttsp4_ret_scan_data_cmd(dev, readElementOffset,
				leftOverElement, dad->heatmap.dataType,
				return_buf);
		if (rc < 0) {
			dev_err(dev, "%s: Error on reading scanData r=%d\n",
					__func__, rc);
			goto cyttsp4_get_panel_data_show_err_release;
		}

		if (return_buf[CY_CMD_OUT_STATUS_OFFSET]
				!= CY_CMD_STATUS_SUCCESS)
			goto cyttsp4_get_panel_data_show_err_release;

		/* DO read */
		readByte = leftOverElement *
			(elementSize);

		if (readByte >= (I2C_BUF_MAX_SIZE - elementStartOffset)) {
			rc = cyttsp4_read(dad->ttsp, CY_MODE_CAT,
					elementStartOffset,
					dad->ic_buf + dataIdx,
					I2C_BUF_MAX_SIZE - elementStartOffset);
			dataIdx += (I2C_BUF_MAX_SIZE - elementStartOffset);
		} else {
			rc = cyttsp4_read(dad->ttsp, CY_MODE_CAT,
					elementStartOffset,
					dad->ic_buf + dataIdx, readByte);
			dataIdx += readByte;
		}
		if (rc < 0) {
			dev_err(dev, "%s: Error on read r=%d\n", __func__, rc);
			goto cyttsp4_get_panel_data_show_err_release;
		}
		returnedElement =
			return_buf[CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_H] * 256
			+ return_buf[CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_L];
		returnedElement = (returnedElement > maxElmtSize) ?
				maxElmtSize : returnedElement;
		/* Update element status */
		leftOverElement -= returnedElement;
		readElementOffset += returnedElement;
	} while (leftOverElement > 0);
	/* update on the buffer */
	dad->ic_buf[CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_H + cmdParam_ofs] =
		HI_BYTE(readElementOffset);
	dad->ic_buf[CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_L + cmdParam_ofs] =
		LOW_BYTE(readElementOffset);

cyttsp4_get_panel_data_show_err_release:
	rc1 = cyttsp4_release_exclusive(dad->ttsp);
	if (rc1 < 0) {
		dev_err(dev, "%s: Error on release exclusive r=%d\n",
				__func__, rc1);
		goto cyttsp4_get_panel_data_show_err_sysfs;
	}

	if (rc < 0)
		goto cyttsp4_get_panel_data_show_err_sysfs;

	printIdx = 0;
	printIdx += scnprintf(buf, CY_MAX_PRBUF_SIZE, "CY_DATA:");
	for (i = 0; i < dataIdx; i++) {
		printIdx += scnprintf(buf + printIdx,
				CY_MAX_PRBUF_SIZE - printIdx,
				"%02X ", dad->ic_buf[i]);
	}
	printIdx += scnprintf(buf + printIdx, CY_MAX_PRBUF_SIZE - printIdx,
			":(%d bytes)\n", dataIdx);

cyttsp4_get_panel_data_show_err_sysfs:
	return printIdx;
}

/*
 * SysFs grpdata show function implementation of group 6.
 * Prints contents of the touch parameters a row at a time.
 */
static int cyttsp4_get_panel_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	ssize_t length;

	mutex_lock(&dad->sysfs_lock);

	length = cyttsp4_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		dev_err(dev, "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto cyttsp4_get_panel_data_store_exit;
	}

	dev_vdbg(dev, "%s: grpnum=%d grpoffset=%u\n",
			__func__, dad->ic_grpnum, dad->ic_grpoffset);

	if (dad->ic_grpnum >= CY_IC_GRPNUM_NUM) {
		dev_err(dev, "%s: Group %d does not exist.\n",
				__func__, dad->ic_grpnum);
		goto cyttsp4_get_panel_data_store_exit;
	}

	pm_runtime_get_sync(dev);
	/*update parameter value */
	dad->heatmap.numElement = dad->ic_buf[4] + (dad->ic_buf[3] * 256);
	dad->heatmap.dataType = dad->ic_buf[5];

	if (dad->ic_buf[6] > 0)
		dad->heatmap.scan_start = true;
	else
		dad->heatmap.scan_start = false;
	pm_runtime_put(dev);

cyttsp4_get_panel_data_store_exit:
	mutex_unlock(&dad->sysfs_lock);
	dev_vdbg(dev, "%s: return size=%d\n", __func__, size);
	return size;
}

static DEVICE_ATTR(get_panel_data, S_IRUSR | S_IWUSR,
	cyttsp4_get_panel_data_show, cyttsp4_get_panel_data_store);

#ifdef CONFIG_PM_SLEEP
static int cyttsp4_device_access_suspend(struct device *dev)
{
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (!mutex_trylock(&dad->sysfs_lock))
		return -EBUSY;

	mutex_unlock(&dad->sysfs_lock);
	return 0;
}

static int cyttsp4_device_access_resume(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);

	return 0;
}
#endif

static const struct dev_pm_ops cyttsp4_device_access_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp4_device_access_suspend,
			cyttsp4_device_access_resume)
};

static int cyttsp4_device_access_probe(struct cyttsp4_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_device_access_data *dad;
	struct cyttsp4_device_access_platform_data *pdata =
			dev_get_platdata(dev);
	int rc = 0;

	dev_info(dev, "%s\n", __func__);
	dev_dbg(dev, "%s: debug on\n", __func__);
	dev_vdbg(dev, "%s: verbose debug on\n", __func__);

	dad = kzalloc(sizeof(*dad), GFP_KERNEL);
	if (dad == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto cyttsp4_device_access_probe_data_failed;
	}

	mutex_init(&dad->sysfs_lock);
	init_waitqueue_head(&dad->wait_q);
	dad->ttsp = ttsp;
	dad->pdata = pdata;
	dad->ic_grpnum = CY_IC_GRPNUM_TEST_REGS;
	dad->test.cur_cmd = -1;
	dad->heatmap.numElement = 200;
	dev_set_drvdata(dev, dad);

	pm_runtime_enable(dev);

	pm_runtime_get_sync(dev);
	/* get sysinfo */
	dad->si = cyttsp4_request_sysinfo(ttsp);
	pm_runtime_put(dev);
	if (dad->si == NULL) {
		dev_err(dev, "%s: Fail get sysinfo pointer from core p=%p\n",
				__func__, dad->si);
		rc = -ENODEV;
		goto cyttsp4_device_access_probe_sysinfo_failed;
	}

	rc = device_create_file(dev, &dev_attr_ic_grpnum);
	if (rc) {
		dev_err(dev, "%s: Error, could not create ic_grpnum\n",
				__func__);
		goto cyttsp4_device_access_probe_sysinfo_failed;
	}

	rc = device_create_file(dev, &dev_attr_ic_grpoffset);
	if (rc) {
		dev_err(dev, "%s: Error, could not create ic_grpoffset\n",
				__func__);
		goto cyttsp4_device_access_probe_unregister_grpnum;
	}

	rc = device_create_file(dev, &dev_attr_ic_grpdata);
	if (rc) {
		dev_err(dev, "%s: Error, could not create ic_grpdata\n",
				__func__);
		goto cyttsp4_device_access_probe_unregister_grpoffset;
	}

	rc = device_create_file(dev, &dev_attr_get_panel_data);
	if (rc) {
		dev_err(dev, "%s: Error, could not create get_panel_data\n",
				__func__);
		goto cyttsp4_device_access_probe_unregister_get_panel_data;
	}

	/* Stay awake if the current grpnum requires */
	if (cyttsp4_is_awakening_grpnum(dad->ic_grpnum))
		pm_runtime_get(dev);

	dev_dbg(dev, "%s: ok\n", __func__);
	return 0;

cyttsp4_device_access_probe_unregister_get_panel_data:
	device_remove_file(dev, &dev_attr_get_panel_data);
 cyttsp4_device_access_probe_unregister_grpoffset:
	device_remove_file(dev, &dev_attr_ic_grpoffset);
 cyttsp4_device_access_probe_unregister_grpnum:
	device_remove_file(dev, &dev_attr_ic_grpnum);
 cyttsp4_device_access_probe_sysinfo_failed:
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	dev_set_drvdata(dev, NULL);
	kfree(dad);
 cyttsp4_device_access_probe_data_failed:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

static int cyttsp4_device_access_release(struct cyttsp4_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp4_device_access_data *dad = dev_get_drvdata(dev);
	u8 ic_buf[CY_NULL_CMD_MODE_INDEX + 1];
	dev_dbg(dev, "%s\n", __func__);

	/* If the current grpnum required being awake, release it */
	mutex_lock(&dad->sysfs_lock);
	if (cyttsp4_is_awakening_grpnum(dad->ic_grpnum))
		pm_runtime_put(dev);
	mutex_unlock(&dad->sysfs_lock);

	if (dad->own_exclusive) {
		dev_err(dev, "%s: Can't unload in CAT mode. " \
				"First switch back to Operational mode\n",
				__func__);
		ic_buf[CY_NULL_CMD_MODE_INDEX] = CY_HST_OPERATE;
		cyttsp4_test_cmd_mode(dad, ic_buf, CY_NULL_CMD_MODE_INDEX + 1);
	}

	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	device_remove_file(dev, &dev_attr_ic_grpnum);
	device_remove_file(dev, &dev_attr_ic_grpoffset);
	device_remove_file(dev, &dev_attr_ic_grpdata);
	device_remove_file(dev, &dev_attr_get_panel_data);
	dev_set_drvdata(dev, NULL);
	kfree(dad);
	return 0;
}

struct cyttsp4_driver cyttsp4_device_access_driver = {
	.probe = cyttsp4_device_access_probe,
	.remove = cyttsp4_device_access_release,
	.driver = {
		.name = CYTTSP4_DEVICE_ACCESS_NAME,
		.bus = &cyttsp4_bus_type,
		.owner = THIS_MODULE,
		.pm = &cyttsp4_device_access_pm_ops,
	},
};

struct cyttsp4_device_access_platform_data
	_cyttsp4_device_access_platform_data = {
	.device_access_dev_name = CYTTSP4_DEVICE_ACCESS_NAME,
};

struct cyttsp4_device cyttsp4_device_access_device = {
	.name = CYTTSP4_DEVICE_ACCESS_NAME,
	.core_id = "main_ttsp_core",
	.dev = {
		.platform_data = &_cyttsp4_device_access_platform_data,
	}
};

static int __init cyttsp4_device_access_init(void)
{
	int rc1;
	int rc2;

	rc1 = cyttsp4_register_device(&cyttsp4_device_access_device);
	if (rc1)
		return -ENODEV;

	rc2 = cyttsp4_register_driver(&cyttsp4_device_access_driver);
	if (rc2) {
		cyttsp4_unregister_device(&cyttsp4_device_access_device);
		return -ENODEV;
	}

	pr_info("%s: %s (Built %s @ %s),rc1=%d rc2=%d\n", __func__,
			"Cypress TTSP Device Access", __DATE__, __TIME__,
			rc1, rc2);
	return 0;
}
module_init(cyttsp4_device_access_init);

static void __exit cyttsp4_device_access_exit(void)
{
	cyttsp4_unregister_device(&cyttsp4_device_access_device);
	cyttsp4_unregister_driver(&cyttsp4_device_access_driver);
	pr_info("%s: module exit\n", __func__);
}
module_exit(cyttsp4_device_access_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Device Access Driver");
MODULE_AUTHOR("Cypress Semiconductor");
