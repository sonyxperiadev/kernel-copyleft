// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/spinlock_types.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/rwsem.h>
#include <media/cam_defs.h>
#include <media/cam_icp.h>
#include <media/cam_cpas.h>

#include "cam_sync_api.h"
#include "cam_packet_util.h"
#include "cam_hw.h"
#include "cam_hw_mgr_intf.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_icp_hw_mgr.h"
#include "cam_bps_hw_intf.h"
#include "cam_ipe_hw_intf.h"
#include "cam_smmu_api.h"
#include "cam_mem_mgr.h"
#include "hfi_intf.h"
#include "hfi_reg.h"
#include "hfi_session_defs.h"
#include "hfi_sys_defs.h"
#include "cam_req_mgr_workq.h"
#include "hfi_sys_defs.h"
#include "cam_debug_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_cpas_api.h"
#include "cam_common_util.h"
#include "cam_mem_mgr_api.h"
#include "cam_presil_hw_access.h"
#include "cam_icp_proc.h"

#define ICP_WORKQ_TASK_CMD_TYPE 1
#define ICP_WORKQ_TASK_MSG_TYPE 2

#define ICP_DEVICE_IDLE_TIMEOUT 400

/*
 * If synx fencing is enabled, send FW memory mapping
 * for synx hw_mutex, ipc hw_mutex, synx global mem
 * and global cntr for qtimer
 */
#define ICP_NUM_MEM_REGIONS_FOR_SYNX 4

DECLARE_RWSEM(frame_in_process_sem);

static struct cam_icp_hw_mgr *g_icp_hw_mgr[CAM_ICP_SUBDEV_MAX];

uint32_t icp_cpas_mask[CAM_ICP_SUBDEV_MAX] = {CPAS_ICP_BIT, CPAS_ICP1_BIT};

static void cam_icp_mgr_process_dbg_buf(struct cam_icp_hw_mgr *hw_mgr);

static int cam_icp_dump_io_cfg(struct cam_icp_hw_ctx_data *ctx_data,
	int32_t buf_handle, uint32_t size)
{
	uintptr_t vaddr_ptr;
	uint32_t  *ptr;
	uint32_t  io_size;
	size_t    len;
	int       rc, i;
	char      buf[512];
	int       used = 0;

	rc = cam_mem_get_cpu_buf(buf_handle, &vaddr_ptr, &len);
	if (rc) {
		CAM_ERR(CAM_ICP, "%s: Unable to get io_cfg buf address",
			ctx_data->ctx_id_string);
		return rc;
	}

	io_size = size / sizeof(uint32_t);
	ptr = (uint32_t *)vaddr_ptr;
	for (i = 0; i < io_size; i++) {
		used += snprintf(buf + used,
			sizeof(buf) - used, "0X%08X-", ptr[i]);
		if (!(i % 8)) {
			CAM_DBG(CAM_ICP, "%s: %s", __func__, buf);
			used = 0;
		}
	}
	cam_mem_put_cpu_buf(buf_handle);
	return rc;
}

static const char *cam_icp_dev_type_to_name(uint32_t dev_type)
{
	switch (dev_type) {
	case CAM_ICP_RES_TYPE_BPS:
		return "BPS";
	case CAM_ICP_RES_TYPE_BPS_RT:
		return "BPS_RT";
	case CAM_ICP_RES_TYPE_BPS_SEMI_RT:
		return "BPS_SEMI_RT";
	case CAM_ICP_RES_TYPE_IPE:
		return "IPE";
	case CAM_ICP_RES_TYPE_IPE_RT:
		return "IPE_RT";
	case CAM_ICP_RES_TYPE_IPE_SEMI_RT:
		return "IPE_SEMI_RT";
	case CAM_ICP_RES_TYPE_OFE:
		return "OFE";
	case CAM_ICP_RES_TYPE_OFE_RT:
		return "OFE_RT";
	case CAM_ICP_RES_TYPE_OFE_SEMI_RT:
		return "OFE_SEMI_RT";
	default:
		return "Invalid dev type";
	}
}

static inline void cam_icp_dump_debug_info(struct cam_icp_hw_mgr *hw_mgr,
	bool skip_dump)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	uint32_t dump_type;
	int rc;

	if (skip_dump)
		return;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL",
			hw_mgr->hw_mgr_name);
		return;
	}

	dump_type = CAM_ICP_DUMP_STATUS_REGISTERS;
	cam_icp_mgr_process_dbg_buf(hw_mgr);
	cam_hfi_queue_dump(hw_mgr->hfi_handle, false);

	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
		CAM_ICP_CMD_HW_REG_DUMP, &dump_type, sizeof(dump_type));
	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Fail to dump debug info", hw_mgr->hw_mgr_name);
}

static int cam_icp_send_ubwc_cfg(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	struct cam_icp_ubwc_cfg_cmd ubwc_cmd;
	int rc;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL",
			hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	ubwc_cmd.ubwc_cfg_dev_mask = hw_mgr->hw_cap_mask;
	ubwc_cmd.disable_ubwc_comp = hw_mgr->disable_ubwc_comp;

	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv, CAM_ICP_CMD_UBWC_CFG,
		&ubwc_cmd, sizeof(ubwc_cmd));
	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Fail to submit UBWC config", hw_mgr->hw_mgr_name);

	return rc;
}

static void cam_icp_hw_mgr_clk_info_update(struct cam_icp_hw_ctx_data *ctx_data)
{
	struct cam_icp_clk_info *dev_clk_info;

	dev_clk_info = &ctx_data->device_info->clk_info;

	if (dev_clk_info->base_clk >= ctx_data->clk_info.base_clk)
		dev_clk_info->base_clk -= ctx_data->clk_info.base_clk;
}

static void cam_icp_hw_mgr_reset_clk_info(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_icp_clk_info *clk_info;
	int i;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		clk_info = &hw_mgr->dev_info[i].clk_info;

		clk_info->base_clk = 0;
		clk_info->curr_clk = hw_mgr->icp_svs_clk;
		clk_info->threshold = ICP_OVER_CLK_THRESHOLD;
		clk_info->over_clked = 0;
		clk_info->uncompressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
		clk_info->compressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
	}

	hw_mgr->icp_default_clk = hw_mgr->icp_svs_clk;
}

static int cam_icp_get_actual_clk_rate_idx(
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	for (i = 0; i < CAM_MAX_VOTE; i++)
		if (ctx_data->clk_info.clk_rate[i] >= base_clk)
			return i;

	/*
	 * Caller has to ensure returned index is within array
	 * size bounds while accessing that index.
	 */

	return i;
}

static bool cam_icp_is_over_clk(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info)
{
	int base_clk_idx;
	int curr_clk_idx;

	base_clk_idx = cam_icp_get_actual_clk_rate_idx(ctx_data,
		hw_mgr_clk_info->base_clk);

	curr_clk_idx = cam_icp_get_actual_clk_rate_idx(ctx_data,
		hw_mgr_clk_info->curr_clk);

	CAM_DBG(CAM_PERF, "%s: bc_idx = %d cc_idx = %d %d %d",
		ctx_data->ctx_id_string, base_clk_idx, curr_clk_idx,
		hw_mgr_clk_info->base_clk, hw_mgr_clk_info->curr_clk);

	if (curr_clk_idx > base_clk_idx)
		return true;

	return false;
}

static int cam_icp_get_lower_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	i = cam_icp_get_actual_clk_rate_idx(ctx_data, base_clk);

	if (i > 0)
		return ctx_data->clk_info.clk_rate[i - 1];

	CAM_DBG(CAM_PERF, "%s: Already clk at lower level", ctx_data->ctx_id_string);
	return base_clk;
}

static int cam_icp_get_next_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	i = cam_icp_get_actual_clk_rate_idx(ctx_data, base_clk);

	if (i < CAM_MAX_VOTE - 1)
		return ctx_data->clk_info.clk_rate[i + 1];

	CAM_DBG(CAM_PERF, "%s: Already clk at higher level", ctx_data->ctx_id_string);

	return base_clk;
}

static int cam_icp_get_actual_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, uint32_t base_clk)
{
	int i;

	for (i = 0; i < CAM_MAX_VOTE; i++)
		if (ctx_data->clk_info.clk_rate[i] >= base_clk)
			return ctx_data->clk_info.clk_rate[i];

	return base_clk;
}

static int cam_icp_get_supported_clk_rates(struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;
	struct cam_hw_soc_info *soc_info;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_hw_info *dev = NULL;

	dev_intf = ctx_data->device_info->dev_intf[0];
	if (!dev_intf) {
		CAM_ERR(CAM_ICP, "%s Invalid device intf for %s",
			ctx_data->ctx_id_string, ctx_data->device_info->dev_name);
		return -EINVAL;
	}

	dev = (struct cam_hw_info *)dev_intf->hw_priv;
	soc_info = &dev->soc_info;

	for (i = 0; i < CAM_MAX_VOTE; i++) {
		ctx_data->clk_info.clk_rate[i] =
			soc_info->clk_rate[i][soc_info->src_clk_idx];
		CAM_DBG(CAM_PERF, "%s: clk_info[%d] = %d",
			ctx_data->ctx_id_string, i, ctx_data->clk_info.clk_rate[i]);
	}

	return 0;
}

static int cam_icp_get_frame_process_idx_from_req_id(struct cam_icp_hw_ctx_data *ctx_data,
	uint64_t req_id)
{
	struct hfi_frame_process_info *frame_process;
	int i;

	frame_process = &ctx_data->hfi_frame_process;

	for (i = 0; i < CAM_FRAME_CMD_MAX; i++)
		if (frame_process->request_id[i] == req_id)
			break;

	return i;
}

static int cam_icp_ctx_clk_info_init(struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;

	ctx_data->clk_info.curr_fc = 0;
	ctx_data->clk_info.base_clk = 0;
	ctx_data->clk_info.uncompressed_bw = 0;
	ctx_data->clk_info.compressed_bw = 0;
	for (i = 0; i < CAM_ICP_MAX_PER_PATH_VOTES; i++) {
		ctx_data->clk_info.axi_path[i].camnoc_bw = 0;
		ctx_data->clk_info.axi_path[i].mnoc_ab_bw = 0;
		ctx_data->clk_info.axi_path[i].mnoc_ib_bw = 0;
	}

	cam_icp_get_supported_clk_rates(ctx_data);

	return 0;
}

static bool cam_icp_frame_pending(struct cam_icp_hw_ctx_data *ctx_data)
{
	return !bitmap_empty(ctx_data->hfi_frame_process.bitmap,
		CAM_FRAME_CMD_MAX);
}

static int cam_icp_ctx_timer_reset(struct cam_icp_hw_ctx_data *ctx_data)
{
	if (ctx_data && ctx_data->watch_dog) {
		ctx_data->watch_dog_reset_counter++;
		CAM_DBG(CAM_PERF, "%s: reset timer : counter=%d",
			ctx_data->ctx_id_string, ctx_data->watch_dog_reset_counter);
		crm_timer_reset(ctx_data->watch_dog);
	}

	return 0;
}

static void cam_icp_device_timer_reset(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_device_info *dev_info)
{
	struct cam_icp_clk_info *clk_info = &dev_info->clk_info;

	if (clk_info->watch_dog) {
		CAM_DBG(CAM_PERF, "[%s] reset timer for device: %s",
			hw_mgr->hw_mgr_name, dev_info->dev_name);
		crm_timer_reset(clk_info->watch_dog);
		clk_info->watch_dog_reset_counter++;
	}
}

static int32_t cam_icp_deinit_idle_clk(void *priv, void *data)
{
	struct cam_icp_hw_mgr *hw_mgr = (struct cam_icp_hw_mgr *)priv;
	struct clk_work_data *task_data = (struct clk_work_data *)data;
	struct cam_icp_hw_device_info *dev_info = task_data->data;
	struct cam_icp_clk_info *clk_info = &dev_info->clk_info;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_icp_dev_clk_update_cmd clk_upd_cmd;
	int rc = 0, i;
	bool busy = false;

	mutex_lock(&hw_mgr->hw_mgr_mutex);

	clk_info->base_clk = 0;
	clk_info->curr_clk = 0;
	clk_info->over_clked = 0;

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx_data = &hw_mgr->ctx_data[i];
		mutex_lock(&ctx_data->ctx_mutex);
		if (ctx_data->state == CAM_ICP_CTX_STATE_ACQUIRED) {
			if (ctx_data->device_info->hw_dev_type == dev_info->hw_dev_type) {
				busy = cam_icp_frame_pending(ctx_data);
				if (busy) {
					mutex_unlock(&ctx_data->ctx_mutex);
					break;
				}
				cam_icp_ctx_clk_info_init(ctx_data);
			}
		}
		mutex_unlock(&ctx_data->ctx_mutex);
	}

	if (busy) {
		cam_icp_device_timer_reset(hw_mgr, dev_info);
		rc = -EBUSY;
		goto done;
	}

	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "[%s] No acquired ctx data found", hw_mgr->hw_mgr_name);
		rc = -EFAULT;
		goto done;
	}

	clk_upd_cmd.dev_pc_enable = hw_mgr->dev_pc_flag;

	CAM_DBG(CAM_PERF, "[%s] Disable %d",
		hw_mgr->hw_mgr_name, dev_info->dev_name);

	for (i = 0; i < dev_info->hw_dev_cnt; i++) {
		dev_intf = dev_info->dev_intf[i];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "[%s] Device intf for %s[%u] is NULL",
				hw_mgr->hw_mgr_name, dev_info->dev_name, i);
			rc = -EINVAL;
			goto done;
		}
		dev_intf->hw_ops.process_cmd(dev_intf->hw_priv, CAM_ICP_DEV_CMD_DISABLE_CLK,
			&clk_upd_cmd, sizeof(clk_upd_cmd));
	}

done:
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static inline bool cam_icp_validate_bw_path_idx(
	int path_idx, uint32_t path_data_type)
{
	if (path_idx < 0) {
		return true;
	} else if (path_idx >= CAM_ICP_MAX_PER_PATH_VOTES) {
		CAM_WARN(CAM_PERF,
			"Invalid path: %u IPE start offset: %d, OFE start offset: %d max: %d",
			path_data_type,
			CAM_AXI_PATH_DATA_IPE_START_OFFSET,
			CAM_AXI_PATH_DATA_OFE_START_OFFSET,
			CAM_ICP_MAX_PER_PATH_VOTES);
		return true;
	} else {
		return false;
	}
}

static inline int cam_icp_get_axi_path_index(struct cam_cpas_axi_per_path_bw_vote *axi_path,
	enum cam_icp_hw_type hw_dev_type)
{
	switch (hw_dev_type) {
	case CAM_ICP_DEV_BPS:
		/* By assuming BPS has Read-All, Write-All votes only */
		return axi_path->transac_type - CAM_AXI_TRANSACTION_READ;
	case CAM_ICP_DEV_IPE:
		return axi_path->path_data_type - CAM_AXI_PATH_DATA_IPE_START_OFFSET;
	case CAM_ICP_DEV_OFE:
		return axi_path->path_data_type - CAM_AXI_PATH_DATA_OFE_START_OFFSET;
	default:
		CAM_ERR(CAM_ICP, "Invalid hw dev type not supported: %u",
			hw_dev_type);
		return -EINVAL;
	}
}

static inline int cam_icp_get_bw_device_share_ratio(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_device_info *dev_info)
{
	uint32_t device_share_ratio = 1, num_dev;

	/*
	 * Since there are 2 devices, we assume the load is evenly shared
	 * between HWs and corresponding AXI paths. So divide total bw by half
	 * to vote on each device
	 */

	num_dev = dev_info->hw_dev_cnt;
	if (num_dev > 1) {
		device_share_ratio = ICP_TWO_DEV_BW_SHARE_RATIO;
		if (num_dev > 2) {
			CAM_ERR(CAM_ICP,
				"[%s] Number of devices %u not supported for geting bw device share ratio",
				hw_mgr->hw_mgr_name, num_dev);
			return -EINVAL;
		}
	}

	return device_share_ratio;
}

static int cam_icp_remove_ctx_bw(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	struct cam_icp_hw_device_info *dev_info = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	uint64_t temp, total_ab_bw = 0;
	struct cam_icp_clk_info *clk_info;
	struct cam_icp_cpas_vote clk_update;
	int i = 0, device_share_ratio;
	enum cam_icp_hw_type hw_dev_type;

	if (!ctx_data->icp_dev_acquire_info) {
		CAM_WARN(CAM_ICP, "%s: NULL acquire info", ctx_data->ctx_id_string);
		return -EINVAL;
	}

	CAM_DBG(CAM_PERF,
		"%s: ubw = %lld cbw = %lld curr_fc = %u bc = %u",
		ctx_data->ctx_id_string,
		ctx_data->clk_info.uncompressed_bw,
		ctx_data->clk_info.compressed_bw,
		ctx_data->clk_info.curr_fc, ctx_data->clk_info.base_clk);

	if (!ctx_data->clk_info.bw_included) {
		CAM_DBG(CAM_PERF, "%s: BW vote already removed",
			ctx_data->ctx_id_string);
		return 0;
	}

	dev_info = ctx_data->device_info;
	clk_info = &dev_info->clk_info;
	hw_dev_type = dev_info->hw_dev_type;

	device_share_ratio = cam_icp_get_bw_device_share_ratio(hw_mgr, dev_info);
	if (device_share_ratio < 0) {
		CAM_ERR(CAM_ICP, "%s: Fail to get device share ratio",
			ctx_data->ctx_id_string);
		return -EINVAL;
	}

	if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V1) {
		clk_update.axi_vote.num_paths = 1;
		if (hw_dev_type == CAM_ICP_DEV_BPS) {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_BPS_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_BPS_DEFAULT_AXI_TRANSAC;
		} else if (hw_dev_type == CAM_ICP_DEV_IPE) {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_IPE_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_IPE_DEFAULT_AXI_TRANSAC;
		} else {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_OFE_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_OFE_DEFAULT_AXI_TRANSAC;
		}

		clk_info->compressed_bw -= ctx_data->clk_info.compressed_bw;
		clk_info->uncompressed_bw -= ctx_data->clk_info.uncompressed_bw;

		total_ab_bw = clk_info->compressed_bw;

		ctx_data->clk_info.uncompressed_bw = 0;
		ctx_data->clk_info.compressed_bw = 0;
		ctx_data->clk_info.curr_fc = 0;
		ctx_data->clk_info.base_clk = 0;

		clk_update.axi_vote.num_paths = 1;

		temp = clk_info->uncompressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].camnoc_bw = temp;

		temp = clk_info->compressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].mnoc_ab_bw = temp;
		clk_update.axi_vote.axi_path[0].mnoc_ib_bw = temp;
	} else {
		int path_index;

		/*
		 * Remove previous vote of this context from hw mgr first.
		 * hw_mgr_clk_info has all valid paths, with each path in its
		 * own index. BW that we wanted to vote now is after removing
		 * current context's vote from hw mgr consolidated vote
		 */
		for (i = 0; i < ctx_data->clk_info.num_paths; i++) {
			path_index = cam_icp_get_axi_path_index(&ctx_data->clk_info.axi_path[i],
				hw_dev_type);

			if (cam_icp_validate_bw_path_idx(path_index,
				ctx_data->clk_info.axi_path[i].path_data_type))
				continue;

			clk_info->axi_path[path_index].camnoc_bw -=
				ctx_data->clk_info.axi_path[i].camnoc_bw;
			clk_info->axi_path[path_index].mnoc_ab_bw -=
				ctx_data->clk_info.axi_path[i].mnoc_ab_bw;
			clk_info->axi_path[path_index].mnoc_ib_bw -=
				ctx_data->clk_info.axi_path[i].mnoc_ib_bw;

			total_ab_bw +=
				clk_info->axi_path[path_index].mnoc_ab_bw;

			CAM_DBG(CAM_PERF,
				"%s: Removing ctx bw from path_type: %s, transac_type: %s, camnoc_bw = %lld mnoc_ab_bw = %lld, mnoc_ib_bw = %lld",
				ctx_data->ctx_id_string,
				cam_cpas_axi_util_path_type_to_string(
				ctx_data->clk_info.axi_path[i].path_data_type),
				cam_cpas_axi_util_trans_type_to_string(
				ctx_data->clk_info.axi_path[i].transac_type),
				ctx_data->clk_info.axi_path[i].camnoc_bw,
				ctx_data->clk_info.axi_path[i].mnoc_ab_bw,
				ctx_data->clk_info.axi_path[i].mnoc_ib_bw);

			CAM_DBG(CAM_PERF,
				"%s: Final HW bw for path_type: %s, transac_type: %s, camnoc_bw = %lld mnoc_ab_bw = %lld, mnoc_ib_bw = %lld",
				ctx_data->ctx_id_string,
				cam_cpas_axi_util_path_type_to_string(
				clk_info->axi_path[i].path_data_type),
				cam_cpas_axi_util_trans_type_to_string(
				clk_info->axi_path[i].transac_type),
				clk_info->axi_path[i].camnoc_bw,
				clk_info->axi_path[i].mnoc_ab_bw,
				clk_info->axi_path[i].mnoc_ib_bw);
		}

		memset(&ctx_data->clk_info.axi_path[0], 0,
			CAM_ICP_MAX_PER_PATH_VOTES *
			sizeof(struct cam_cpas_axi_per_path_bw_vote));
		ctx_data->clk_info.curr_fc = 0;
		ctx_data->clk_info.base_clk = 0;

		clk_update.axi_vote.num_paths = clk_info->num_paths;
		memcpy(&clk_update.axi_vote.axi_path[0],
			&clk_info->axi_path[0],
			clk_update.axi_vote.num_paths *
			sizeof(struct cam_cpas_axi_per_path_bw_vote));

		if (device_share_ratio > 1) {
			for (i = 0; i < clk_update.axi_vote.num_paths; i++) {
				do_div(
				clk_update.axi_vote.axi_path[i].camnoc_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ab_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ib_bw,
					device_share_ratio);
			}
		}
	}

	clk_update.axi_vote_valid = true;

	if (total_ab_bw == 0) {
		/* If no more contexts are active, reduce AHB vote to minimum */
		clk_update.ahb_vote.type = CAM_VOTE_ABSOLUTE;
		clk_update.ahb_vote.vote.level = CAM_LOWSVS_D1_VOTE;
		clk_update.ahb_vote_valid = true;
	} else {
		clk_update.ahb_vote_valid = false;
	}

	/*
	 * If two devices, vote half bandwidth each on both devices.
	 * Total bw at mnoc - CPAS will take care of adding up.
	 * camnoc clk calculate is more accurate this way.
	 */

	for (i = 0; i < dev_info->hw_dev_cnt; i++) {
		dev_intf = dev_info->dev_intf[i];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "%s: Device intf for %s[%u] is NULL",
				ctx_data->ctx_id_string, dev_info->dev_name, i);
			return -EINVAL;
		}
		rc = dev_intf->hw_ops.process_cmd(dev_intf->hw_priv,
			CAM_ICP_DEV_CMD_VOTE_CPAS, &clk_update, sizeof(clk_update));
		if (rc) {
			CAM_ERR(CAM_PERF,
				"%s: Failed in updating cpas vote for %s cell idx: %u, rc=%d",
				ctx_data->ctx_id_string, dev_info->dev_name, i, rc);
		}
	}

	ctx_data->clk_info.bw_included = false;

	CAM_DBG(CAM_PERF, "%s: X :curr_fc = %u bc = %u", ctx_data->ctx_id_string,
		ctx_data->clk_info.curr_fc, ctx_data->clk_info.base_clk);

	return rc;
}


static int32_t cam_icp_ctx_timer(void *priv, void *data)
{
	struct clk_work_data *task_data = (struct clk_work_data *)data;
	struct cam_icp_hw_mgr *hw_mgr = priv;
	struct cam_icp_hw_ctx_data *ctx_data =
		(struct cam_icp_hw_ctx_data *)task_data->data;

	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "ctx_data is NULL, failed to update clk");
		return -EINVAL;
	}

	mutex_lock(&ctx_data->ctx_mutex);

	CAM_DBG(CAM_PERF,
		"%s: ubw = %lld cbw = %lld curr_fc = %u bc = %u",
		ctx_data->ctx_id_string,
		ctx_data->clk_info.uncompressed_bw,
		ctx_data->clk_info.compressed_bw,
		ctx_data->clk_info.curr_fc,
		ctx_data->clk_info.base_clk);

	if ((ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) ||
		(ctx_data->watch_dog_reset_counter == 0)) {
		CAM_DBG(CAM_PERF, "%s: state %d, counter=%d",
			ctx_data->ctx_id_string, ctx_data->state,
			ctx_data->watch_dog_reset_counter);
		mutex_unlock(&ctx_data->ctx_mutex);
		return 0;
	}

	if (cam_icp_frame_pending(ctx_data)) {
		cam_icp_ctx_timer_reset(ctx_data);
		mutex_unlock(&ctx_data->ctx_mutex);
		return -EBUSY;
	}

	cam_icp_remove_ctx_bw(hw_mgr, ctx_data);

	mutex_unlock(&ctx_data->ctx_mutex);

	return 0;
}

static void cam_icp_ctx_timer_cb(struct timer_list *timer_data)
{
	unsigned long flags;
	struct crm_workq_task *task;
	struct clk_work_data *task_data;
	struct cam_req_mgr_timer *timer =
		container_of(timer_data, struct cam_req_mgr_timer, sys_timer);
	struct cam_icp_hw_ctx_data *ctx_data = timer->parent;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;

	spin_lock_irqsave(&hw_mgr->hw_mgr_lock, flags);
	task = cam_req_mgr_workq_get_task(hw_mgr->timer_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "%s: empty task", ctx_data->ctx_id_string);
		spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);
		return;
	}

	task_data = (struct clk_work_data *)task->payload;
	task_data->data = ctx_data;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_ctx_timer;
	cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);
	spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);
}

static void cam_icp_device_timer_cb(struct timer_list *timer_data)
{
	unsigned long flags;
	struct crm_workq_task *task;
	struct clk_work_data *task_data;
	struct cam_req_mgr_timer *timer =
		container_of(timer_data, struct cam_req_mgr_timer, sys_timer);
	struct cam_icp_hw_device_info *dev_info = timer->parent;
	struct cam_icp_hw_mgr *hw_mgr = dev_info->clk_info.timeout_cb_data;

	spin_lock_irqsave(&hw_mgr->hw_mgr_lock, flags);
	task = cam_req_mgr_workq_get_task(hw_mgr->timer_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "[%s] empty task", hw_mgr->hw_mgr_name);
		spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);
		return;
	}

	task_data = (struct clk_work_data *)task->payload;
	task_data->data = timer->parent;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_deinit_idle_clk;
	cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);
	spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);
}

static int cam_icp_get_svs_clk_info(struct cam_icp_hw_mgr *hw_mgr)
{
	int32_t src_clk_idx;
	struct cam_hw_soc_info *soc_info;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_hw_info *dev = NULL;

	if (CAM_ICP_IS_DEV_HW_EXIST(hw_mgr->hw_cap_mask, CAM_ICP_DEV_IPE)) {
		dev_intf = hw_mgr->dev_info[hw_mgr->dev_info_idx[CAM_ICP_DEV_IPE]].dev_intf[0];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "[%s] IPE dev intf is invalid", hw_mgr->hw_mgr_name);
			return -EINVAL;
		}
	} else if (CAM_ICP_IS_DEV_HW_EXIST(hw_mgr->hw_cap_mask, CAM_ICP_DEV_OFE)) {
		dev_intf = hw_mgr->dev_info[hw_mgr->dev_info_idx[CAM_ICP_DEV_OFE]].dev_intf[0];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "[%s] OFE dev inf is invalid", hw_mgr->hw_mgr_name);
			return -EINVAL;
		}
	} else {
		CAM_ERR(CAM_ICP, "[%s] No supported device to get svs clock info",
			hw_mgr->hw_mgr_name);
		return -ENODEV;
	}

	dev = (struct cam_hw_info *)dev_intf->hw_priv;
	soc_info = &dev->soc_info;
	src_clk_idx = soc_info->src_clk_idx;
	hw_mgr->icp_svs_clk = soc_info->clk_rate[CAM_SVS_VOTE][src_clk_idx];

	if (hw_mgr->icp_svs_clk <= 0)
		hw_mgr->icp_svs_clk = ICP_CLK_SVS_HZ;

	CAM_DBG(CAM_PERF, "[%s] icp_svs_clk = %lld", hw_mgr->hw_mgr_name, hw_mgr->icp_svs_clk);
	return 0;
}

static int cam_icp_clk_info_init(struct cam_icp_hw_mgr *hw_mgr)
{
	int i;
	struct cam_icp_clk_info *clk_info;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		clk_info = &hw_mgr->dev_info[i].clk_info;

		clk_info->base_clk = hw_mgr->icp_svs_clk;
		clk_info->curr_clk = hw_mgr->icp_svs_clk;
		clk_info->threshold = ICP_OVER_CLK_THRESHOLD;
		clk_info->over_clked = 0;
		clk_info->uncompressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
		clk_info->compressed_bw = CAM_CPAS_DEFAULT_AXI_BW;
		memset(clk_info->axi_path, 0,
			CAM_ICP_MAX_PER_PATH_VOTES * sizeof(struct cam_cpas_axi_per_path_bw_vote));

		clk_info->watch_dog_reset_counter = 0;
	}

	hw_mgr->icp_default_clk = hw_mgr->icp_svs_clk;

	return 0;
}

static int cam_icp_ctx_timer_start(struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;

	rc = crm_timer_init(&ctx_data->watch_dog,
		200, ctx_data, &cam_icp_ctx_timer_cb);
	if (rc)
		CAM_ERR(CAM_ICP, "%s: Failed to start timer", ctx_data->ctx_id_string);

	ctx_data->watch_dog_reset_counter = 0;

	CAM_DBG(CAM_PERF, "%s: start timer", ctx_data->ctx_id_string);
	return rc;
}

static int cam_icp_device_timer_start(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0, i;
	struct cam_icp_clk_info *clk_info = NULL;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		clk_info = &hw_mgr->dev_info[i].clk_info;
		if (!clk_info->watch_dog) {
			clk_info->timeout_cb_data = hw_mgr;
			rc = crm_timer_init(&clk_info->watch_dog,
				ICP_DEVICE_IDLE_TIMEOUT, &hw_mgr->dev_info[i],
					&cam_icp_device_timer_cb);
			if (rc)
				CAM_ERR(CAM_ICP, "[%s] Failed to start %s timer",
					hw_mgr->hw_mgr_name, hw_mgr->dev_info[i].dev_name);
			clk_info->watch_dog_reset_counter = 0;
		}
	}

	return rc;
}

static int cam_icp_ctx_timer_stop(struct cam_icp_hw_ctx_data *ctx_data)
{
	if (ctx_data->watch_dog) {
		CAM_DBG(CAM_PERF, "%s: stop timer", ctx_data->ctx_id_string);
		ctx_data->watch_dog_reset_counter = 0;
		crm_timer_exit(&ctx_data->watch_dog);
		ctx_data->watch_dog = NULL;
	}

	return 0;
}

static void cam_icp_device_timer_stop(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_icp_hw_device_info *dev_info = NULL;
	struct cam_icp_clk_info *clk_info = NULL;
	int i;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		dev_info = &hw_mgr->dev_info[i];
		clk_info = &dev_info->clk_info;
		if (!dev_info->dev_ctx_info.dev_ctxt_cnt && clk_info->watch_dog) {
			clk_info->watch_dog_reset_counter = 0;
			crm_timer_exit(&clk_info->watch_dog);
			clk_info->watch_dog = NULL;
		}
	}
}

static uint32_t cam_icp_mgr_calc_base_clk(uint32_t frame_cycles,
	uint64_t budget)
{
	uint64_t base_clk;
	uint64_t mul = 1000000000;

	base_clk = frame_cycles * mul;
	do_div(base_clk, budget);

	CAM_DBG(CAM_PERF, "budget = %lld fc = %d ib = %lld base_clk = %lld",
		budget, frame_cycles,
		(long long)(frame_cycles * mul), base_clk);

	return base_clk;
}

static bool cam_icp_busy_prev_reqs(struct hfi_frame_process_info *frm_process,
	uint64_t req_id)
{
	int i;
	int cnt;

	for (i = 0, cnt = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if (frm_process->request_id[i] && frm_process->fw_process_flag[i]) {
			CAM_DBG(CAM_PERF, "r id = %lld busy = %d",
				frm_process->request_id[i],
				frm_process->fw_process_flag[i]);
			cnt++;
		}
	}
	if (cnt > 1)
		return true;

	return false;
}

static int cam_icp_calc_total_clk(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_clk_info *dev_clk_info, enum cam_icp_hw_type hw_dev_type)
{
	int i;
	struct cam_icp_hw_ctx_data *ctx_data;

	dev_clk_info->base_clk = 0;
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx_data = &hw_mgr->ctx_data[i];
		if (ctx_data->state == CAM_ICP_CTX_STATE_ACQUIRED &&
			(ctx_data->device_info->hw_dev_type == hw_dev_type))
			dev_clk_info->base_clk += ctx_data->clk_info.base_clk;
	}

	return 0;
}

static bool cam_icp_update_clk_busy(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *dev_clk_info,
	struct cam_icp_clk_bw_request *request_clk_info,
	uint32_t base_clk)
{
	uint32_t next_clk_level;
	uint32_t actual_clk;
	bool rc = false;

	/* 1. if current request frame cycles(fc) are more than previous
	 *      frame fc
	 *      Calculate the new base clock.
	 *      if sum of base clocks are more than next available clk level
	 *       Update clock rate, change curr_clk_rate to sum of base clock
	 *       rates and make over_clked to zero
	 *      else
	 *       Update clock rate to next level, update curr_clk_rate and make
	 *       overclked cnt to zero
	 * 2. if current fc is less than or equal to previous  frame fc
	 *      Still Bump up the clock to next available level
	 *      if it is available, then update clock, make overclk cnt to
	 *      zero. If the clock is already at highest clock rate then
	 *      no need to update the clock
	 */
	ctx_data->clk_info.base_clk = base_clk;
	dev_clk_info->over_clked = 0;
	if (request_clk_info->frame_cycles > ctx_data->clk_info.curr_fc) {
		cam_icp_calc_total_clk(hw_mgr, dev_clk_info,
			ctx_data->device_info->hw_dev_type);
		actual_clk = cam_icp_get_actual_clk_rate(hw_mgr,
			ctx_data, base_clk);
		if (dev_clk_info->base_clk > actual_clk) {
			dev_clk_info->curr_clk = dev_clk_info->base_clk;
		} else {
			next_clk_level = cam_icp_get_next_clk_rate(hw_mgr,
				ctx_data, dev_clk_info->curr_clk);
			dev_clk_info->curr_clk = next_clk_level;
		}
		rc = true;
	} else {
		next_clk_level =
			cam_icp_get_next_clk_rate(hw_mgr, ctx_data,
			dev_clk_info->curr_clk);
		if (dev_clk_info->curr_clk < next_clk_level) {
			dev_clk_info->curr_clk = next_clk_level;
			rc = true;
		}
	}
	ctx_data->clk_info.curr_fc = request_clk_info->frame_cycles;

	return rc;
}

static bool cam_icp_update_clk_overclk_free(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, struct cam_icp_clk_info *dev_clk_info,
	struct cam_icp_clk_bw_request *request_clk_info, uint32_t base_clk)
{
	int rc = false;

	/*
	 * In caseof no pending packets case
	 *    1. In caseof overclk cnt is less than threshold, increase
	 *       overclk count and no update in the clock rate
	 *    2. In caseof overclk cnt is greater than or equal to threshold
	 *       then lower clock rate by one level and update hw_mgr current
	 *       clock value.
	 *        a. In case of new clock rate greater than sum of clock
	 *           rates, reset overclk count value to zero if it is
	 *           overclock
	 *        b. if it is less than sum of base clocks then go to next
	 *           level of clock and make overclk count to zero
	 *        c. if it is same as sum of base clock rates update overclock
	 *           cnt to 0
	 */
	if (dev_clk_info->over_clked < dev_clk_info->threshold) {
		dev_clk_info->over_clked++;
		rc = false;
	} else {
		dev_clk_info->curr_clk =
			cam_icp_get_lower_clk_rate(hw_mgr, ctx_data,
			dev_clk_info->curr_clk);
		if (dev_clk_info->curr_clk > dev_clk_info->base_clk) {
			if (cam_icp_is_over_clk(hw_mgr, ctx_data,
				dev_clk_info))
				dev_clk_info->over_clked = 0;
		} else if (dev_clk_info->curr_clk <
			dev_clk_info->base_clk) {
			dev_clk_info->curr_clk =
				cam_icp_get_next_clk_rate(hw_mgr, ctx_data,
				dev_clk_info->curr_clk);
				dev_clk_info->over_clked = 0;
		} else if (dev_clk_info->curr_clk ==
			dev_clk_info->base_clk) {
			dev_clk_info->over_clked = 0;
		}
		rc = true;
	}

	return rc;
}

static bool cam_icp_update_clk_free(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_request *clk_info,
	uint32_t base_clk)
{
	int rc = false;
	bool over_clocked = false;

	ctx_data->clk_info.curr_fc = clk_info->frame_cycles;
	ctx_data->clk_info.base_clk = base_clk;
	cam_icp_calc_total_clk(hw_mgr, hw_mgr_clk_info, ctx_data->device_info->hw_dev_type);

	/*
	 * Current clock is not always sum of base clocks, due to
	 * clock scales update to next higher or lower levels, it
	 * equals to one of discrete clock values supported by hardware.
	 * So even current clock is higher than sum of base clocks, we
	 * can not consider it is over clocked. if it is greater than
	 * discrete clock level then only it is considered as over clock.
	 * 1. Handle over clock case
	 * 2. If current clock is less than sum of base clocks
	 *    update current clock
	 * 3. If current clock is same as sum of base clocks no action
	 */

	over_clocked = cam_icp_is_over_clk(hw_mgr, ctx_data,
		hw_mgr_clk_info);

	if (hw_mgr_clk_info->curr_clk > hw_mgr_clk_info->base_clk &&
		over_clocked) {
		rc = cam_icp_update_clk_overclk_free(hw_mgr, ctx_data,
			hw_mgr_clk_info, clk_info, base_clk);
	} else if (hw_mgr_clk_info->curr_clk > hw_mgr_clk_info->base_clk) {
		hw_mgr_clk_info->over_clked = 0;
		rc = false;
	}  else if (hw_mgr_clk_info->curr_clk < hw_mgr_clk_info->base_clk) {
		hw_mgr_clk_info->curr_clk = cam_icp_get_actual_clk_rate(hw_mgr,
			ctx_data, hw_mgr_clk_info->base_clk);
		rc = true;
	}

	return rc;
}

static bool cam_icp_debug_clk_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_clk_info *hw_mgr_clk_info)
{
	if (hw_mgr->icp_debug_clk &&
		hw_mgr->icp_debug_clk != hw_mgr_clk_info->curr_clk) {
		hw_mgr_clk_info->base_clk = hw_mgr->icp_debug_clk;
		hw_mgr_clk_info->curr_clk = hw_mgr->icp_debug_clk;
		hw_mgr_clk_info->uncompressed_bw = hw_mgr->icp_debug_clk;
		hw_mgr_clk_info->compressed_bw = hw_mgr->icp_debug_clk;
		CAM_DBG(CAM_PERF, "[%s] bc = %d cc = %d",
			hw_mgr->hw_mgr_name, hw_mgr_clk_info->base_clk,
			hw_mgr_clk_info->curr_clk);
		return true;
	}

	return false;
}

static bool cam_icp_default_clk_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_clk_info *hw_mgr_clk_info)
{
	if (hw_mgr->icp_default_clk != hw_mgr_clk_info->curr_clk) {
		hw_mgr_clk_info->base_clk = hw_mgr->icp_default_clk;
		hw_mgr_clk_info->curr_clk = hw_mgr->icp_default_clk;
		hw_mgr_clk_info->uncompressed_bw = hw_mgr->icp_default_clk;
		hw_mgr_clk_info->compressed_bw = hw_mgr->icp_default_clk;
		CAM_DBG(CAM_PERF, "[%s] bc = %d cc = %d",
			hw_mgr->hw_mgr_name, hw_mgr_clk_info->base_clk,
			hw_mgr_clk_info->curr_clk);
		return true;
	}

	return false;
}

static bool cam_icp_update_bw_v2(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_req_internal_v2 *clk_info,
	bool busy)
{
	int i, path_index;
	bool update_required = true;

	/*
	 * If current request bandwidth is different from previous frames, then
	 * recalculate bandwidth of all contexts of same hardware and update
	 * voting of bandwidth
	 */

	for (i = 0; i < clk_info->num_paths; i++)
		CAM_DBG(CAM_PERF, "%s: clk_info camnoc = %lld busy = %d",
			ctx_data->ctx_id_string, clk_info->axi_path[i].camnoc_bw, busy);

	if (clk_info->num_paths == ctx_data->clk_info.num_paths) {
		update_required = false;
		for (i = 0; i < clk_info->num_paths; i++) {
			if ((clk_info->axi_path[i].transac_type ==
				ctx_data->clk_info.axi_path[i].transac_type) &&
				(clk_info->axi_path[i].path_data_type ==
				ctx_data->clk_info.axi_path[i].path_data_type) &&
				(clk_info->axi_path[i].camnoc_bw ==
				ctx_data->clk_info.axi_path[i].camnoc_bw) &&
				(clk_info->axi_path[i].mnoc_ab_bw ==
				ctx_data->clk_info.axi_path[i].mnoc_ab_bw)) {
				continue;
			} else {
				update_required = true;
				break;
			}
		}
	}

	if (!update_required) {
		CAM_DBG(CAM_PERF,
			"%s: Incoming BW hasn't changed, no update required, num_paths=%d",
			ctx_data->ctx_id_string, clk_info->num_paths);
		return false;
	}

	if (busy) {
		for (i = 0; i < clk_info->num_paths; i++) {
			if (ctx_data->clk_info.axi_path[i].camnoc_bw >
				clk_info->axi_path[i].camnoc_bw)
				return false;
		}
	}

	/*
	 * Remove previous vote of this context from hw mgr first.
	 * hw_mgr_clk_info has all valid paths, with each path in its own index
	 */
	for (i = 0; i < ctx_data->clk_info.num_paths; i++) {

		path_index = cam_icp_get_axi_path_index(&ctx_data->clk_info.axi_path[i],
			ctx_data->device_info->hw_dev_type);

		if (cam_icp_validate_bw_path_idx(path_index,
			ctx_data->clk_info.axi_path[i].path_data_type))
			continue;

		hw_mgr_clk_info->axi_path[path_index].camnoc_bw -=
			ctx_data->clk_info.axi_path[i].camnoc_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ab_bw -=
			ctx_data->clk_info.axi_path[i].mnoc_ab_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ib_bw -=
			ctx_data->clk_info.axi_path[i].mnoc_ib_bw;
	}

	ctx_data->clk_info.num_paths = clk_info->num_paths;

	memcpy(&ctx_data->clk_info.axi_path[0],
		&clk_info->axi_path[0],
		clk_info->num_paths * sizeof(struct cam_cpas_axi_per_path_bw_vote));

	/*
	 * Add new vote of this context in hw mgr.
	 * hw_mgr_clk_info has all paths, with each path in its own index
	 */
	for (i = 0; i < ctx_data->clk_info.num_paths; i++) {
		path_index = cam_icp_get_axi_path_index(&ctx_data->clk_info.axi_path[i],
			ctx_data->device_info->hw_dev_type);

		if (cam_icp_validate_bw_path_idx(path_index,
			ctx_data->clk_info.axi_path[i].path_data_type))
			continue;

		hw_mgr_clk_info->axi_path[path_index].path_data_type =
			ctx_data->clk_info.axi_path[i].path_data_type;
		hw_mgr_clk_info->axi_path[path_index].transac_type =
			ctx_data->clk_info.axi_path[i].transac_type;
		hw_mgr_clk_info->axi_path[path_index].camnoc_bw +=
			ctx_data->clk_info.axi_path[i].camnoc_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ab_bw +=
			ctx_data->clk_info.axi_path[i].mnoc_ab_bw;
		hw_mgr_clk_info->axi_path[path_index].mnoc_ib_bw +=
			ctx_data->clk_info.axi_path[i].mnoc_ib_bw;
		CAM_DBG(CAM_PERF,
			"%s: Consolidate Path Vote: i[%d] path_idx[%d] : [%s %s] [%lld %lld]",
			ctx_data->ctx_id_string,
			i, path_index,
			cam_cpas_axi_util_trans_type_to_string(
			hw_mgr_clk_info->axi_path[path_index].transac_type),
			cam_cpas_axi_util_path_type_to_string(
			hw_mgr_clk_info->axi_path[path_index].path_data_type),
			hw_mgr_clk_info->axi_path[path_index].camnoc_bw,
			hw_mgr_clk_info->axi_path[path_index].mnoc_ab_bw);
	}

	ctx_data->clk_info.bw_included = true;

	if (hw_mgr_clk_info->num_paths < ctx_data->clk_info.num_paths)
		hw_mgr_clk_info->num_paths = ctx_data->clk_info.num_paths;

	return true;
}

static bool cam_icp_update_bw(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_icp_clk_info *hw_mgr_clk_info,
	struct cam_icp_clk_bw_request *clk_info,
	bool busy)
{
	int i;
	struct cam_icp_hw_ctx_data *ctx;

	/*
	 * If current request bandwidth is different from previous frames, then
	 * recalculate bandwidth of all contexts of same hardware and update
	 * voting of bandwidth
	 */
	CAM_DBG(CAM_PERF, "%s: ubw ctx = %lld clk_info ubw = %lld busy = %d",
		ctx_data->ctx_id_string, ctx_data->clk_info.uncompressed_bw,
		clk_info->uncompressed_bw, busy);

	if ((clk_info->uncompressed_bw == ctx_data->clk_info.uncompressed_bw) &&
		(ctx_data->clk_info.uncompressed_bw ==
		hw_mgr_clk_info->uncompressed_bw)) {
		CAM_DBG(CAM_PERF, "%s: Update not required bw=%lld",
			ctx_data->ctx_id_string, ctx_data->clk_info.uncompressed_bw);
		return false;
	}

	if (busy &&
		(ctx_data->clk_info.uncompressed_bw >
		clk_info->uncompressed_bw)) {
		CAM_DBG(CAM_PERF,
			"%s: Busy, Update not required existing=%lld, new=%lld",
			ctx_data->ctx_id_string,
			ctx_data->clk_info.uncompressed_bw,
			clk_info->uncompressed_bw);
		return false;
	}

	ctx_data->clk_info.uncompressed_bw = clk_info->uncompressed_bw;
	ctx_data->clk_info.compressed_bw = clk_info->compressed_bw;
	hw_mgr_clk_info->uncompressed_bw = 0;
	hw_mgr_clk_info->compressed_bw = 0;

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx = &hw_mgr->ctx_data[i];
		if (ctx->state == CAM_ICP_CTX_STATE_ACQUIRED &&
			(ctx_data->device_info->hw_dev_type == ctx->device_info->hw_dev_type)) {
			hw_mgr_clk_info->uncompressed_bw +=
				ctx->clk_info.uncompressed_bw;
			hw_mgr_clk_info->compressed_bw +=
				ctx->clk_info.compressed_bw;
			CAM_DBG(CAM_PERF,
				"%s: Current context=[%lld %lld] Total=[%lld %lld]",
				ctx_data->ctx_id_string,
				ctx->clk_info.uncompressed_bw,
				ctx->clk_info.compressed_bw,
				hw_mgr_clk_info->uncompressed_bw,
				hw_mgr_clk_info->compressed_bw);
		}
	}

	ctx_data->clk_info.bw_included = true;

	return true;
}

static bool cam_icp_check_clk_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int idx)
{
	bool busy, rc = false;
	uint32_t base_clk;
	struct cam_icp_clk_bw_request *request_clk_info;
	struct hfi_frame_process_info *frame_info;
	uint64_t req_id;
	struct cam_icp_hw_device_info *dev_info = ctx_data->device_info;
	struct cam_icp_clk_info *dev_clk_info;

	cam_icp_ctx_timer_reset(ctx_data);
	cam_icp_device_timer_reset(hw_mgr, dev_info);
	CAM_DBG(CAM_ICP, "Reset %s device timer", dev_info->dev_name);

	dev_clk_info = &dev_info->clk_info;

	if (hw_mgr->icp_debug_clk)
		return cam_icp_debug_clk_update(hw_mgr, dev_clk_info);

	/* Check is there any pending frames in this context */
	frame_info = &ctx_data->hfi_frame_process;
	req_id = frame_info->request_id[idx];
	busy = cam_icp_busy_prev_reqs(frame_info, req_id);
	CAM_DBG(CAM_PERF, "%s: busy = %d req_id = %lld",
		ctx_data->ctx_id_string, busy, req_id);

	request_clk_info = &ctx_data->hfi_frame_process.clk_info[idx];
	if (!request_clk_info->frame_cycles)
		return cam_icp_default_clk_update(hw_mgr, dev_clk_info);

	ctx_data->clk_info.rt_flag = request_clk_info->rt_flag;

	/* Override base clock to max or calculate base clk rate */
	if (!ctx_data->clk_info.rt_flag &&
		(dev_info->hw_dev_type != CAM_ICP_DEV_BPS))
		base_clk = ctx_data->clk_info.clk_rate[CAM_MAX_VOTE-1];
	else
		base_clk = cam_icp_mgr_calc_base_clk(request_clk_info->frame_cycles,
			request_clk_info->budget_ns);

	if (busy)
		rc = cam_icp_update_clk_busy(hw_mgr, ctx_data,
			dev_clk_info, request_clk_info, base_clk);
	else
		rc = cam_icp_update_clk_free(hw_mgr, ctx_data,
			dev_clk_info, request_clk_info, base_clk);

	CAM_DBG(CAM_PERF, "%s: bc = %d cc = %d busy = %d overclk = %d uc = %d",
		ctx_data->ctx_id_string, dev_clk_info->base_clk, dev_clk_info->curr_clk,
		busy, dev_clk_info->over_clked, rc);

	return rc;
}

static bool cam_icp_check_bw_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int idx)
{
	bool busy, bw_updated = false;
	int i;
	struct cam_icp_clk_bw_request *request_clk_info;
	struct cam_icp_clk_bw_req_internal_v2 *request_clk_info_v2;
	struct cam_icp_clk_info *dev_clk_info;
	struct hfi_frame_process_info *frame_info;
	uint64_t req_id;

	dev_clk_info = &ctx_data->device_info->clk_info;
	frame_info = &ctx_data->hfi_frame_process;
	req_id = frame_info->request_id[idx];
	busy = cam_icp_busy_prev_reqs(frame_info, req_id);

	if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V1) {
		request_clk_info = &ctx_data->hfi_frame_process.clk_info[idx];

		CAM_DBG(CAM_PERF, "%s: Req[%lld] Current camno=%lld, mnoc=%lld",
			ctx_data->ctx_id_string, req_id, dev_clk_info->uncompressed_bw,
			dev_clk_info->compressed_bw);

		bw_updated = cam_icp_update_bw(hw_mgr, ctx_data,
			dev_clk_info, request_clk_info, busy);
	} else if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V2) {
		request_clk_info_v2 = &ctx_data->hfi_frame_process.clk_info_v2[idx];

		CAM_DBG(CAM_PERF, "%s: index=%d, num_paths=%d, ctx_data=%pK",
			ctx_data->ctx_id_string, idx, request_clk_info_v2->num_paths, ctx_data);

		bw_updated = cam_icp_update_bw_v2(hw_mgr, ctx_data,
			dev_clk_info, request_clk_info_v2, busy);

		for (i = 0; i < dev_clk_info->num_paths; i++) {
			CAM_DBG(CAM_PERF,
				"%s: Final path_type: %s, transac_type: %s, camnoc_bw = %lld mnoc_ab_bw = %lld, mnoc_ib_bw = %lld",
				ctx_data->ctx_id_string,
				cam_cpas_axi_util_path_type_to_string(
				dev_clk_info->axi_path[i].path_data_type),
				cam_cpas_axi_util_trans_type_to_string(
				dev_clk_info->axi_path[i].transac_type),
				dev_clk_info->axi_path[i].camnoc_bw,
				dev_clk_info->axi_path[i].mnoc_ab_bw,
				dev_clk_info->axi_path[i].mnoc_ib_bw);
		}
	} else {
		CAM_ERR(CAM_PERF, "%s: Invalid bw config version: %d",
			hw_mgr->hw_mgr_name, ctx_data->bw_config_version);
		return false;
	}

	return bw_updated;
}

static int cam_icp_update_clk_rate(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	uint32_t i, curr_clk_rate;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_icp_clk_info *dev_clk_info = NULL;
	struct cam_icp_dev_clk_update_cmd clk_upd_cmd;
	char tmp_buff[64];

	dev_clk_info = &ctx_data->device_info->clk_info;
	scnprintf(tmp_buff, sizeof(tmp_buff), "%s Before clk update rate=%u",
		ctx_data->ctx_id_string,
		dev_clk_info->prev_clk);
	cam_cpas_notify_event(tmp_buff, dev_clk_info->prev_clk);

	curr_clk_rate = dev_clk_info->curr_clk;
	dev_clk_info->prev_clk = curr_clk_rate;
	scnprintf(tmp_buff, sizeof(tmp_buff), "%s After clk update rate=%u",
		ctx_data->ctx_id_string,
		curr_clk_rate);
	cam_cpas_notify_event(tmp_buff, curr_clk_rate);

	CAM_DBG(CAM_PERF, "%s: clk_rate %u",
		ctx_data->ctx_id_string, curr_clk_rate);

	clk_upd_cmd.curr_clk_rate = curr_clk_rate;
	clk_upd_cmd.dev_pc_enable = hw_mgr->dev_pc_flag;
	clk_upd_cmd.clk_level = -1;

	for (i = 0; i < ctx_data->device_info->hw_dev_cnt; i++) {
		dev_intf = ctx_data->device_info->dev_intf[i];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "Device intf for %s[%u] is NULL",
				ctx_data->device_info->dev_name, i);
			return -EINVAL;
		}
		dev_intf->hw_ops.process_cmd(dev_intf->hw_priv, CAM_ICP_DEV_CMD_UPDATE_CLK,
			&clk_upd_cmd, sizeof(clk_upd_cmd));
	}

	/* Scale ICP clock to IPE clk rate or OFE clk rate */
	if (ctx_data->device_info->hw_dev_type != CAM_ICP_DEV_BPS) {
		/* update ICP Proc clock */
		CAM_DBG(CAM_PERF, "%s: Update ICP clk to level [%d]",
			ctx_data->ctx_id_string, clk_upd_cmd.clk_level);
		dev_intf = hw_mgr->icp_dev_intf;
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "Device interface is NULL");
			return -EINVAL;
		}
		dev_intf->hw_ops.process_cmd(dev_intf->hw_priv, CAM_ICP_CMD_CLK_UPDATE,
			&clk_upd_cmd.clk_level, sizeof(clk_upd_cmd.clk_level));
	}

	return 0;
}

static int cam_icp_update_cpas_vote(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0, i = 0, device_share_ratio;
	uint64_t temp;
	struct cam_hw_intf *dev_intf = NULL;
	struct cam_icp_clk_info *dev_clk_info;
	struct cam_icp_cpas_vote clk_update = {0};
	enum cam_icp_hw_type hw_dev_type;

	dev_clk_info = &ctx_data->device_info->clk_info;
	hw_dev_type = ctx_data->device_info->hw_dev_type;

	device_share_ratio = cam_icp_get_bw_device_share_ratio(hw_mgr,
		ctx_data->device_info);
	if (device_share_ratio < 0) {
		CAM_ERR(CAM_ICP, "%s: Fail to get device share ratio",
			ctx_data->ctx_id_string);
		return -EINVAL;
	}

	clk_update.ahb_vote.type = CAM_VOTE_DYNAMIC;
	clk_update.ahb_vote.vote.freq = 0;
	clk_update.ahb_vote_valid = false;

	if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_V1) {
		clk_update.axi_vote.num_paths = 1;
		if (hw_dev_type == CAM_ICP_DEV_BPS) {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_BPS_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_BPS_DEFAULT_AXI_TRANSAC;
		} else if (hw_dev_type == CAM_ICP_DEV_IPE) {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_IPE_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_IPE_DEFAULT_AXI_TRANSAC;
		} else {
			clk_update.axi_vote.axi_path[0].path_data_type =
				CAM_OFE_DEFAULT_AXI_PATH;
			clk_update.axi_vote.axi_path[0].transac_type =
				CAM_OFE_DEFAULT_AXI_TRANSAC;
		}

		temp = dev_clk_info->uncompressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].camnoc_bw = temp;

		temp = dev_clk_info->compressed_bw;
		do_div(temp, device_share_ratio);
		clk_update.axi_vote.axi_path[0].mnoc_ab_bw = temp;
		clk_update.axi_vote.axi_path[0].mnoc_ib_bw = temp;
	} else {
		clk_update.axi_vote.num_paths = dev_clk_info->num_paths;
		memcpy(&clk_update.axi_vote.axi_path[0],
			&dev_clk_info->axi_path[0],
			clk_update.axi_vote.num_paths *
			sizeof(struct cam_cpas_axi_per_path_bw_vote));

		if (device_share_ratio > 1) {
			for (i = 0; i < clk_update.axi_vote.num_paths; i++) {
				do_div(
				clk_update.axi_vote.axi_path[i].camnoc_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ab_bw,
					device_share_ratio);
				do_div(
				clk_update.axi_vote.axi_path[i].mnoc_ib_bw,
					device_share_ratio);
			}
		}
	}

	clk_update.axi_vote_valid = true;

	/*
	 * If two devices, vote half bandwidth each on both devices.
	 * Total bw at mnoc - CPAS will take care of adding up.
	 * camnoc clk calculate is more accurate this way.
	 */

	for (i = 0; i < ctx_data->device_info->hw_dev_cnt; i++) {
		dev_intf = ctx_data->device_info->dev_intf[i];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "Device intf for %s[%u] is NULL",
				ctx_data->device_info->dev_name, i);
			return -EINVAL;
		}
		rc = dev_intf->hw_ops.process_cmd(dev_intf->hw_priv,
			CAM_ICP_DEV_CMD_VOTE_CPAS, &clk_update, sizeof(clk_update));
		if (rc) {
			CAM_ERR(CAM_PERF,
				"%s: Failed in updating cpas vote for %s cell idx: %u, rc=%d",
				ctx_data->ctx_id_string, ctx_data->device_info->dev_name, i, rc);
		}
	}

	return rc;
}

static int cam_icp_mgr_dev_clk_update(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int idx)
{
	int rc = 0;

	if (cam_icp_check_clk_update(hw_mgr, ctx_data, idx))
		rc = cam_icp_update_clk_rate(hw_mgr, ctx_data);

	if (cam_icp_check_bw_update(hw_mgr, ctx_data, idx))
		rc |= cam_icp_update_cpas_vote(hw_mgr, ctx_data);

	return rc;
}

static inline int cam_icp_mgr_get_core_info_mask(enum cam_icp_hw_type hw_dev_type,
	uint32_t dev_num, uint32_t *core_info_mask)
{
	if (dev_num > 2) {
		CAM_ERR(CAM_ICP, "%u devices core info mask is not supported", dev_num);
		return -EINVAL;
	}

	switch (hw_dev_type) {
	case CAM_ICP_DEV_BPS:
		*core_info_mask = ICP_PWR_CLP_BPS;
		break;
	case CAM_ICP_DEV_IPE:
		*core_info_mask = ICP_PWR_CLP_IPE0;
		if (dev_num > 1)
			*core_info_mask |= ICP_PWR_CLP_IPE1;
		break;
	case CAM_ICP_DEV_OFE:
		*core_info_mask = ICP_PWR_CLP_OFE;
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid hw device type: %u",
			hw_dev_type);
		return -EINVAL;
	}

	return 0;
}

static int cam_icp_mgr_device_resume(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	struct cam_icp_hw_device_info *dev_info = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	struct hfi_cmd_prop *dbg_prop = NULL;
	uint32_t core_info_mask = 0, size;
	int rc = 0, i;
	enum cam_icp_hw_type hw_dev_type;

	hw_dev_type = ctx_data->device_info->hw_dev_type;
	dev_info = ctx_data->device_info;

	if (dev_info->dev_ctx_info.dev_ctxt_cnt++)
		goto end;

	for (i = 0; i < dev_info->hw_dev_cnt; i++) {
		dev_intf = dev_info->dev_intf[i];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "Device intf for %s[%u] is NULL",
				dev_info->dev_name, i);
			rc = -EINVAL;
			goto end;
		}
		if (!dev_info->dev_ctx_info.dev_clk_state) {
			rc = dev_intf->hw_ops.init(dev_intf->hw_priv, NULL, 0);
			if (rc) {
				CAM_ERR(CAM_ICP, "Fail to resume device %s[%u]",
					dev_info->dev_name, i);
			}
			dev_info->dev_ctx_info.dev_clk_state = true;
		}
		if (hw_mgr->dev_pc_flag) {
			dev_intf->hw_ops.process_cmd(dev_intf->hw_priv,
				CAM_ICP_DEV_CMD_POWER_RESUME, NULL, 0);
		}
	}

	rc = cam_icp_mgr_get_core_info_mask(hw_dev_type, dev_info->hw_dev_cnt, &core_info_mask);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"%s Fail to get core info mask for hw dev: %s ctx id: %u rc:%d",
			ctx_data->ctx_id_string, dev_info->dev_name, ctx_data->ctx_id, rc);
	}

	CAM_DBG(CAM_PERF, "%s core_info 0x%x",
		ctx_data->ctx_id_string, core_info_mask);

	size = sizeof(struct hfi_cmd_prop) + sizeof(struct hfi_dev_pc);
	dbg_prop = kzalloc(size, GFP_KERNEL);
	if (!dbg_prop) {
		CAM_ERR(CAM_ICP, "%s Allocate command prop failed",
			ctx_data->ctx_id_string);
		rc = -ENOMEM;
		goto end;
	}

	dbg_prop->size = size;
	dbg_prop->pkt_type = HFI_CMD_SYS_SET_PROPERTY;
	dbg_prop->num_prop = 1;

	switch (hw_dev_type) {
	case CAM_ICP_DEV_IPE:
		fallthrough;
	case CAM_ICP_DEV_BPS:
		dbg_prop->prop_data[0] = HFI_PROP_SYS_IPEBPS_PC;
		break;
	case CAM_ICP_DEV_OFE:
		dbg_prop->prop_data[0] = HFI_PROP_SYS_OFE_PC;
		break;
	default:
		CAM_ERR(CAM_ICP, "%s Invalid hw dev type: %u",
			ctx_data->ctx_id_string, hw_dev_type);
		rc = -EINVAL;
		goto free_dbg_prop;
	}

	dbg_prop->prop_data[1] = hw_mgr->dev_pc_flag;
	dbg_prop->prop_data[2] = core_info_mask;

	hfi_write_cmd(hw_mgr->hfi_handle, dbg_prop);

free_dbg_prop:
	kfree(dbg_prop);

end:
	return rc;
}

static int cam_icp_mgr_dev_power_collapse(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data, int dev_type)
{
	int rc = 0, i;
	struct cam_icp_hw_device_info *dev_info = NULL;
	struct cam_hw_intf *dev_intf = NULL;

	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Invalid ctx data is NULL");
		return -EINVAL;
	}

	dev_info = ctx_data->device_info;

	CAM_DBG(CAM_PERF, "%s: device %s ctx cnt: %u",
		ctx_data->ctx_id_string, dev_info->dev_name,
		dev_info->dev_ctx_info.dev_ctxt_cnt);

	if (--dev_info->dev_ctx_info.dev_ctxt_cnt)
		goto end;

	for (i = 0; i < dev_info->hw_dev_cnt; i++) {
		dev_intf = dev_info->dev_intf[i];
		if (!dev_intf) {
			CAM_ERR(CAM_ICP, "%s Device intf for %s[%u] is NULL",
				ctx_data->ctx_id_string, dev_info->dev_name, i);
			return -EINVAL;
		}
		if (hw_mgr->dev_pc_flag && !atomic_read(&hw_mgr->recovery)) {
			dev_intf->hw_ops.process_cmd(dev_intf->hw_priv,
				CAM_ICP_DEV_CMD_POWER_COLLAPSE, NULL, 0);
		}
		if (dev_info->dev_ctx_info.dev_clk_state)
			dev_intf->hw_ops.deinit(dev_intf->hw_priv, NULL, 0);
	}

	dev_info->dev_ctx_info.dev_clk_state = false;

end:
	return rc;
}

static int cam_icp_mgr_dev_get_gdsc_control(
	struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0, i, j;
	struct cam_hw_intf *dev_intf;

	if (!hw_mgr->dev_pc_flag)
		return rc;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		for (j = 0; j < hw_mgr->dev_info[i].hw_dev_cnt; j++) {
			if (!hw_mgr->dev_info[i].dev_ctx_info.dev_clk_state)
				continue;

			dev_intf = hw_mgr->dev_info[i].dev_intf[j];
			if (!dev_intf) {
				CAM_ERR(CAM_ICP, "Device intf for %s[%u] is NULL",
					hw_mgr->dev_info[i].dev_name, j);
				return -EINVAL;
			}
			rc = dev_intf->hw_ops.process_cmd(dev_intf->hw_priv,
				CAM_ICP_DEV_CMD_POWER_COLLAPSE, NULL, 0);
		}
	}

	return rc;
}

static int cam_icp_set_dbg_default_clk(void *data, u64 val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	hw_mgr->icp_debug_clk = val;
	return 0;
}

static int cam_icp_get_dbg_default_clk(void *data, u64 *val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	*val = hw_mgr->icp_debug_clk;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_icp_debug_default_clk,
	cam_icp_get_dbg_default_clk,
	cam_icp_set_dbg_default_clk, "%16llu");

static int cam_icp_set_icp_dbg_lvl(void *data, u64 val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	hw_mgr->icp_dbg_lvl = val;
	return 0;
}

static int cam_icp_get_icp_dbg_lvl(void *data, u64 *val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	*val = hw_mgr->icp_dbg_lvl;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_icp_debug_fs, cam_icp_get_icp_dbg_lvl,
	cam_icp_set_icp_dbg_lvl, "%08llu");

static int cam_icp_set_icp_dbg_type(void *data, u64 val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	if (val <= NUM_HFI_DEBUG_MODE)
		hw_mgr->icp_debug_type = val;

	return 0;
}

static int cam_icp_get_icp_dbg_type(void *data, u64 *val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	*val = hw_mgr->icp_debug_type;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_icp_debug_type_fs, cam_icp_get_icp_dbg_type,
	cam_icp_set_icp_dbg_type, "%08llu");

static int cam_icp_set_icp_fw_hang_dump_lvl(void *data, u64 val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	if (val < NUM_HFI_DUMP_LVL)
		hw_mgr->icp_fw_dump_lvl = val;

	return 0;
}

static int cam_icp_get_icp_fw_hang_dump_lvl(void *data, u64 *val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	*val = hw_mgr->icp_fw_dump_lvl;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(cam_icp_debug_fw_dump, cam_icp_get_icp_fw_hang_dump_lvl,
	cam_icp_set_icp_fw_hang_dump_lvl, "%08llu");

static int cam_icp_set_icp_fw_ramdump_lvl(void *data, u64 val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	if (val < NUM_HFI_RAMDUMP_LVLS)
		hw_mgr->icp_fw_ramdump_lvl = (uint32_t)val;
	return 0;
}

static int cam_icp_get_icp_fw_ramdump_lvl(void *data, u64 *val)
{
	struct cam_icp_hw_mgr *hw_mgr = data;

	*val = hw_mgr->icp_fw_ramdump_lvl;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(cam_icp_debug_fw_ramdump, cam_icp_get_icp_fw_ramdump_lvl,
	cam_icp_set_icp_fw_ramdump_lvl, "%08llu");

#ifdef CONFIG_CAM_TEST_ICP_FW_DOWNLOAD
static ssize_t cam_icp_hw_mgr_fw_load_unload(
	struct file *file, const char __user *ubuf,
	size_t size, loff_t *loff_t)
{
	int rc = 0;
	char input_buf[16];
	struct cam_icp_hw_mgr *hw_mgr = file->private_data;

	if (copy_from_user(input_buf, ubuf, sizeof(input_buf)))
		return -EFAULT;

	if (strcmp(input_buf, "load\n") == 0) {
		rc = cam_mem_mgr_init();
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] memmgr init failed rc: %d",
				hw_mgr->hw_mgr_name, rc);
			goto end;
		}
		cam_icp_mgr_hw_open(hw_mgr, NULL);
	} else if (strcmp(input_buf, "unload\n") == 0) {
		cam_icp_mgr_hw_close(hw_mgr, NULL);
		cam_mem_mgr_deinit();
	} else {
		CAM_WARN(CAM_ICP, "[%s] Invalid input: %s", hw_mgr->hw_mgr_name, input_buf);
	}

end:
	return size;
}

static const struct file_operations cam_icp_hw_mgr_fw_load_options = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = cam_icp_hw_mgr_fw_load_unload,
};
#endif

#ifdef CONFIG_CAM_TEST_IRQ_LINE

static int cam_icp_test_irq_line(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = -EINVAL;
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	if (icp_dev_intf->hw_ops.test_irq_line)
		rc = icp_dev_intf->hw_ops.test_irq_line(icp_dev_intf->hw_priv);

	if (rc)
		CAM_ERR(CAM_ICP, "[%s] failed to verify IRQ line", hw_mgr->hw_mgr_name);

	return 0;
}

#else

static int cam_icp_test_irq_line(struct cam_icp_hw_mgr *hw_mgr)
{
	CAM_ERR(CAM_ICP, "[%s] IRQ line verification disabled", hw_mgr->hw_mgr_name);
	return -EPERM;
}

#endif

#if (defined(CONFIG_CAM_TEST_IRQ_LINE) && defined(CONFIG_CAM_TEST_IRQ_LINE_AT_PROBE))

static int cam_icp_test_irq_line_at_probe(struct cam_icp_hw_mgr *hw_mgr)
{
	return cam_icp_test_irq_line(hw_mgr);
}

#else

static int cam_icp_test_irq_line_at_probe(struct cam_icp_hw_mgr *hw_mgr)
{
	return 0;
}

#endif

static int cam_icp_set_irq_line_test(void *data, u64 val)
{
	cam_icp_test_irq_line(data);
	return 0;
}

static int cam_icp_get_irq_line_test(void *data, u64 *val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_icp_irq_line_test, cam_icp_get_irq_line_test,
	cam_icp_set_irq_line_test, "%08llu");

static int cam_icp_hw_mgr_create_debugfs_entry(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	if (!cam_debugfs_available())
		return 0;

	rc = cam_debugfs_create_subdir(hw_mgr->hw_mgr_name, &dbgfileptr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] DebugFS could not create directory", hw_mgr->hw_mgr_name);
		rc = -ENOENT;
		goto end;
	}

	/* Store parent inode for cleanup in caller */
	hw_mgr->dentry = dbgfileptr;

	debugfs_create_bool("icp_pc", 0644, hw_mgr->dentry,
		&hw_mgr->icp_pc_flag);

	debugfs_create_bool("dev_interframe_pc", 0644, hw_mgr->dentry,
		&hw_mgr->dev_pc_flag);

	debugfs_create_file("icp_debug_clk", 0644,
		hw_mgr->dentry, hw_mgr, &cam_icp_debug_default_clk);

	debugfs_create_bool("icp_jtag_debug", 0644,
		hw_mgr->dentry, &hw_mgr->icp_jtag_debug);

	debugfs_create_file("icp_debug_type", 0644,
		hw_mgr->dentry, hw_mgr, &cam_icp_debug_type_fs);

	debugfs_create_file("icp_debug_lvl", 0644,
		hw_mgr->dentry, hw_mgr, &cam_icp_debug_fs);

	debugfs_create_file("icp_fw_dump_lvl", 0644,
		hw_mgr->dentry, hw_mgr, &cam_icp_debug_fw_dump);

	debugfs_create_file("icp_fw_ramdump_lvl", 0644,
		hw_mgr->dentry, hw_mgr, &cam_icp_debug_fw_ramdump);

	debugfs_create_bool("disable_ubwc_comp", 0644,
		hw_mgr->dentry, &hw_mgr->disable_ubwc_comp);

	#ifdef CONFIG_CAM_TEST_ICP_FW_DOWNLOAD
		debugfs_create_file("icp_fw_load_unload", 0644,
			hw_mgr->dentry, hw_mgr, &cam_icp_hw_mgr_fw_load_options);
	#endif
	debugfs_create_file("test_irq_line", 0644,
		hw_mgr->dentry, hw_mgr, &cam_icp_irq_line_test);

end:

	/* Set default hang dump lvl */
	hw_mgr->icp_fw_dump_lvl = HFI_FW_DUMP_ON_FAILURE;
	hw_mgr->icp_fw_ramdump_lvl = HFI_FW_RAMDUMP_ENABLED;
	return rc;
}

static int cam_icp_mgr_process_cmd(void *priv, void *data)
{
	int rc;
	struct hfi_cmd_work_data *task_data = NULL;
	struct cam_icp_hw_mgr *hw_mgr;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid params%pK %pK", data, priv);
		return -EINVAL;
	}

	hw_mgr = priv;
	task_data = (struct hfi_cmd_work_data *)data;

	rc = hfi_write_cmd(hw_mgr->hfi_handle, task_data->data);

	return rc;
}

static int cam_icp_mgr_cleanup_ctx(struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;
	struct hfi_frame_process_info *hfi_frame_process;
	struct cam_icp_hw_buf_done_evt_data icp_evt_data;
	struct cam_hw_done_event_data buf_data = {0};

	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (i = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if (!hfi_frame_process->request_id[i])
			continue;
		buf_data.request_id = hfi_frame_process->request_id[i];
		icp_evt_data.evt_id = CAM_CTX_EVT_ID_SUCCESS;
		icp_evt_data.buf_done_data = &buf_data;
		ctx_data->ctxt_event_cb(ctx_data->context_priv,
			CAM_ICP_EVT_ID_BUF_DONE, &icp_evt_data);
		hfi_frame_process->request_id[i] = 0;
		if (ctx_data->hfi_frame_process.in_resource[i] > 0) {
			CAM_DBG(CAM_ICP, "%s: Delete merged sync in object: %d",
				ctx_data->ctx_id_string,
				ctx_data->hfi_frame_process.in_resource[i]);
			cam_sync_destroy(
				ctx_data->hfi_frame_process.in_resource[i]);
			ctx_data->hfi_frame_process.in_resource[i] = 0;
		}
		hfi_frame_process->fw_process_flag[i] = false;
		clear_bit(i, ctx_data->hfi_frame_process.bitmap);
	}

	for (i = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if (!hfi_frame_process->in_free_resource[i])
			continue;

		CAM_DBG(CAM_ICP, "%s: Delete merged sync in object: %d",
			ctx_data->ctx_id_string,
			ctx_data->hfi_frame_process.in_free_resource[i]);
		cam_sync_destroy(
			ctx_data->hfi_frame_process.in_free_resource[i]);
		ctx_data->hfi_frame_process.in_free_resource[i] = 0;
	}

	ctx_data->abort_timed_out = false;
	return 0;
}

static uint32_t cam_icp_handle_err_type_to_evt_param(uint32_t error_type)
{
	switch (error_type) {
	case CAMERAICP_ENOMEMORY:
		return CAM_SYNC_ICP_EVENT_NO_MEMORY;
	case CAMERAICP_EFAILED:
		return CAM_SYNC_ICP_EVENT_FRAME_PROCESS_FAILURE;
	case CAMERAICP_EBADSTATE:
		return CAM_SYNC_ICP_EVENT_BAD_STATE;
	case CAMERAICP_EBADPARM:
		return CAM_SYNC_ICP_EVENT_BAD_PARAM;
	case CAMERAICP_EBADITEM:
		return CAM_SYNC_ICP_EVENT_BAD_ITEM;
	case CAMERAICP_EINVALIDFORMAT:
		return CAM_SYNC_ICP_EVENT_INVALID_FORMAT;
	case CAMERAICP_EUNSUPPORTED:
		return CAM_SYNC_ICP_EVENT_UNSUPPORTED;
	case CAMERAICP_EOUTOFBOUND:
		return CAM_SYNC_ICP_EVENT_OUT_OF_BOUND;
	case CAMERAICP_ETIMEDOUT:
		return CAM_SYNC_ICP_EVENT_TIME_OUT;
	case CAMERAICP_EABORTED:
		return CAM_SYNC_ICP_EVENT_ABORTED;
	case CAMERAICP_EHWVIOLATION:
		return CAM_SYNC_ICP_EVENT_HW_VIOLATION;
	case CAMERAICP_ECDMERROR:
		return CAM_SYNC_ICP_EVENT_CMD_ERROR;
	case CAMERAICP_HFI_ERR_COMMAND_SIZE:
		return CAM_SYNC_ICP_EVENT_HFI_ERR_COMMAND_SIZE;
	case CAMERAICP_HFI_ERR_MESSAGE_SIZE:
		return CAM_SYNC_ICP_EVENT_HFI_ERR_MESSAGE_SIZE;
	case CAMERAICP_HFI_QUEUE_EMPTY:
		return CAM_SYNC_ICP_EVENT_HFI_ERR_QUEUE_EMPTY;
	case CAMERAICP_HFI_QUEUE_FULL:
		return CAM_SYNC_ICP_EVENT_HFI_ERR_QUEUE_FULL;
	default:
		return CAM_SYNC_ICP_EVENT_UNKNOWN;
	}
}

static const char *cam_icp_error_handle_id_to_type(
	uint32_t error_handle)
{
	const char *name = NULL;

	switch (error_handle) {
	case CAMERAICP_SUCCESS:
		name = "SUCCESS";
		break;
	case CAMERAICP_EFAILED:
		name = "EFAILED";
		break;
	case CAMERAICP_ENOMEMORY:
		name = "ENOMEMORY";
		break;
	case CAMERAICP_EBADSTATE:
		name = "EBADSTATE";
		break;
	case CAMERAICP_EBADPARM:
		name = "EBADPARM";
		break;
	case CAMERAICP_EBADITEM:
		name = "EBADITEM";
		break;
	case CAMERAICP_EINVALIDFORMAT:
		name = "EINVALIDFORMAT";
		break;
	case CAMERAICP_EUNSUPPORTED:
		name = "EUNSUPPORTED";
		break;
	case CAMERAICP_EOUTOFBOUND:
		name = "EOUTOFBOUND";
		break;
	case CAMERAICP_ETIMEDOUT:
		name = "ETIMEDOUT";
		break;
	case CAMERAICP_EABORTED:
		name = "EABORTED";
		break;
	case CAMERAICP_EHWVIOLATION:
		name = "EHWVIOLATION";
		break;
	case CAMERAICP_ECDMERROR:
		name = "ECDMERROR";
		break;
	case CAMERAICP_HFI_ERR_COMMAND_SIZE:
		name = "HFI_ERR_COMMAND_SIZE";
		break;
	case CAMERAICP_HFI_ERR_MESSAGE_SIZE:
		name = "HFI_ERR_MESSAGE_SIZE";
		break;
	case CAMERAICP_HFI_QUEUE_EMPTY:
		name = "HFI_QUEUE_EMPTY";
		break;
	case CAMERAICP_HFI_QUEUE_FULL:
		name = "HFI_QUEUE_FULL";
		break;
	default:
		name = NULL;
		break;
	}
	return name;
}

static inline void cam_icp_mgr_apply_evt_injection(struct cam_hw_done_event_data *buf_done_data,
	struct cam_icp_hw_ctx_data *ctx_data, bool *signal_fence_buffer)
{
	struct cam_hw_inject_evt_param *evt_params = &ctx_data->evt_inject_params;
	struct cam_common_evt_inject_data inject_evt;

	inject_evt.buf_done_data = buf_done_data;
	inject_evt.evt_params = evt_params;

	if (ctx_data->ctxt_event_cb)
		ctx_data->ctxt_event_cb(ctx_data->context_priv, CAM_ICP_EVT_ID_INJECT_EVENT,
			&inject_evt);

	if (ctx_data->evt_inject_params.inject_id ==
		CAM_COMMON_EVT_INJECT_BUFFER_ERROR_TYPE)
		*signal_fence_buffer = false;

	evt_params->is_valid = false;
}

static void cam_icp_mgr_dump_active_req_info(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	char log_info[256];
	size_t buf_size, len;
	uint32_t total_active_streams = 0, total_active_requests = 0;
	int i, j;

	buf_size = sizeof(log_info);

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx_data = &hw_mgr->ctx_data[i];
		mutex_lock(&ctx_data->ctx_mutex);
		if (ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) {
			mutex_unlock(&ctx_data->ctx_mutex);
			continue;
		}

		len = 0;
		for (j = 0; j < CAM_FRAME_CMD_MAX; j++) {
			if (!ctx_data->hfi_frame_process.request_id[j])
				continue;

			len += scnprintf(log_info + len, (buf_size - len), " %llu",
				ctx_data->hfi_frame_process.request_id[j]);
			total_active_requests++;
		}
		total_active_streams++;
		CAM_INFO(CAM_ICP, "%s: Active Requests IDs: %s",
			ctx_data->ctx_id_string, len ? log_info : " None");
		mutex_unlock(&ctx_data->ctx_mutex);
	}
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	CAM_INFO(CAM_ICP, "%s: Total Active streams: %u, Total active requests: %u",
		hw_mgr->hw_mgr_name, total_active_streams, total_active_requests);
}

static void cam_icp_mgr_compute_fw_avg_response_time(struct cam_icp_hw_ctx_data *ctx_data,
	uint32_t request_idx)
{
	struct cam_icp_ctx_perf_stats *perf_stats;
	uint64_t delta;

	delta = ktime_ms_delta(ktime_get(),
		ctx_data->hfi_frame_process.submit_timestamp[request_idx]);

	perf_stats = &ctx_data->perf_stats;
	perf_stats->total_resp_time += delta;
	perf_stats->total_requests++;

	CAM_DBG(CAM_PERF,
		"%s: Avg response time: current_req: %llu total_processed_requests: %llu avg_time: %llums",
		ctx_data->ctx_id_string,
		ctx_data->hfi_frame_process.request_id[request_idx], perf_stats->total_requests,
		(perf_stats->total_resp_time / perf_stats->total_requests));
}

static int cam_icp_mgr_handle_frame_process(uint32_t *msg_ptr, int flag)
{
	int i;
	uint32_t idx, event_id;
	uint64_t request_id;
	struct cam_icp_hw_mgr *hw_mgr = NULL;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct hfi_msg_dev_async_ack *ioconfig_ack = NULL;
	struct hfi_frame_process_info *hfi_frame_process;
	struct cam_hw_done_event_data buf_data = {0};
	struct cam_icp_hw_buf_done_evt_data icp_done_evt;
	struct cam_icp_hw_error_evt_data icp_err_evt = {0};
	struct cam_hangdump_mem_regions *mem_regions = NULL;
	bool signal_fence_buffer = true;

	ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
	request_id = ioconfig_ack->user_data2;
	ctx_data = (struct cam_icp_hw_ctx_data *)
		U64_TO_PTR(ioconfig_ack->user_data1);
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Invalid Context req %llu", request_id);
		return -EINVAL;
	}

	mutex_lock(&ctx_data->ctx_mutex);
	hw_mgr = ctx_data->hw_mgr_priv;
	cam_icp_ctx_timer_reset(ctx_data);
	if (ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) {
		CAM_DBG(CAM_ICP, "%s: is in %d state",
			ctx_data->ctx_id_string, ctx_data->state);
		mutex_unlock(&ctx_data->ctx_mutex);
		return 0;
	}

	CAM_DBG(CAM_REQ,
		"%s: request_id: %lld",
		ctx_data->ctx_id_string, request_id);

	cam_icp_device_timer_reset(hw_mgr, ctx_data->device_info);

	hfi_frame_process = &ctx_data->hfi_frame_process;

	idx = cam_icp_get_frame_process_idx_from_req_id(ctx_data, request_id);
	if (idx >= CAM_FRAME_CMD_MAX) {
		CAM_ERR(CAM_ICP, "%s: pkt not found for req_id =%lld",
			ctx_data->ctx_id_string, request_id);
		mutex_unlock(&ctx_data->ctx_mutex);
		return -EINVAL;
	}

	cam_icp_mgr_compute_fw_avg_response_time(ctx_data, idx);

	if (flag == ICP_FRAME_PROCESS_FAILURE) {
		if (ioconfig_ack->err_type == CAMERAICP_EABORTED) {
			CAM_WARN(CAM_ICP,
				"%s: req %llu has been aborted[flushed]",
				ctx_data->ctx_id_string, request_id);
			event_id = CAM_CTX_EVT_ID_CANCEL;
		} else {
			CAM_ERR(CAM_ICP,
				"%s: Done with error: %u err_type= [%s] for req %llu",
				ctx_data->ctx_id_string, ioconfig_ack->err_type,
				cam_icp_error_handle_id_to_type(ioconfig_ack->err_type),
				request_id);
			event_id = CAM_CTX_EVT_ID_ERROR;
		}
		buf_data.evt_param = cam_icp_handle_err_type_to_evt_param(ioconfig_ack->err_type);
	} else {
		event_id = CAM_CTX_EVT_ID_SUCCESS;
	}

	if (cam_presil_mode_enabled()) {
		mem_regions = &hfi_frame_process->hangdump_mem_regions[idx];
		CAM_INFO(CAM_ICP, "Hangdump Num Regions %d",
			mem_regions->num_mem_regions);
		for (i = 0; i < mem_regions->num_mem_regions; i++) {
			CAM_INFO(CAM_PRESIL, "Hangdump Mem %d handle 0x%08x offset 0x%08x len %u",
				i, mem_regions->mem_info_array[i].mem_handle,
				mem_regions->mem_info_array[i].offset,
				mem_regions->mem_info_array[i].size);
			cam_mem_mgr_retrieve_buffer_from_presil(
				mem_regions->mem_info_array[i].mem_handle,
				mem_regions->mem_info_array[i].size,
				mem_regions->mem_info_array[i].offset,
				hw_mgr->iommu_hdl);
		}
	}

	CAM_TRACE(CAM_ICP,
		"%s: BufDone Req %llu event_id %d",
		ctx_data->ctx_id_string, hfi_frame_process->request_id[idx], event_id);

	buf_data.request_id = hfi_frame_process->request_id[idx];
	if ((ctx_data->evt_inject_params.is_valid) &&
		(ctx_data->evt_inject_params.req_id == request_id))
		cam_icp_mgr_apply_evt_injection(&buf_data, ctx_data, &signal_fence_buffer);

	if (signal_fence_buffer) {
		icp_done_evt.evt_id = event_id;
		icp_done_evt.buf_done_data = &buf_data;
		if (ctx_data->ctxt_event_cb)
			ctx_data->ctxt_event_cb(ctx_data->context_priv, CAM_ICP_EVT_ID_BUF_DONE,
				&icp_done_evt);
	}

	hfi_frame_process->request_id[idx] = 0;
	if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
		CAM_DBG(CAM_ICP, "%s: Delete merged sync in object: %d",
			ctx_data->ctx_id_string,
			ctx_data->hfi_frame_process.in_resource[idx]);
		cam_sync_destroy(ctx_data->hfi_frame_process.in_resource[idx]);
		ctx_data->hfi_frame_process.in_resource[idx] = 0;
	}
	clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
	hfi_frame_process->fw_process_flag[idx] = false;

	mutex_unlock(&ctx_data->ctx_mutex);

	/* report recovery to userspace if FW encounters no memory */
	if (ioconfig_ack->err_type == CAMERAICP_ENOMEMORY) {
		cam_icp_mgr_dump_active_req_info(hw_mgr);

		icp_err_evt.err_type = CAM_ICP_HW_ERROR_NO_MEM;
		icp_err_evt.req_id = request_id;

		mutex_lock(&ctx_data->ctx_mutex);
		if (ctx_data->ctxt_event_cb)
			ctx_data->ctxt_event_cb(ctx_data->context_priv, CAM_ICP_EVT_ID_ERROR,
				&icp_err_evt);
		mutex_unlock(&ctx_data->ctx_mutex);
	}

	if (cam_presil_mode_enabled()) {
		if (!atomic_read(&hw_mgr->frame_in_process)) {
			CAM_ERR(CAM_PRESIL, "%s: presil: frame_in_process not set",
				ctx_data->ctx_id_string);
		} else {
			hw_mgr->frame_in_process_ctx_id = -1;
			atomic_set(&hw_mgr->frame_in_process, 0);
			up_write(&frame_in_process_sem);
			CAM_DBG(CAM_PRESIL, "%s: presil: unlocked frame_in_process",
				ctx_data->ctx_id_string);
		}
	}

	return 0;
}

static int cam_icp_mgr_process_msg_frame_process(uint32_t *msg_ptr)
{
	struct hfi_msg_dev_async_ack *ioconfig_ack = NULL;
	struct hfi_msg_frame_process_done *frame_done;

	if (!msg_ptr) {
		CAM_ERR(CAM_ICP, "msg ptr is NULL");
		return -EINVAL;
	}

	ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
	if (ioconfig_ack->err_type != CAMERAICP_SUCCESS) {
		cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_FAILURE);
		return -EIO;
	}

	frame_done =
		(struct hfi_msg_frame_process_done *)ioconfig_ack->msg_data;
	if (!frame_done) {
		cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_FAILURE);
		return -EINVAL;
	}

	if (frame_done->result)
		return cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_FAILURE);
	else
		return cam_icp_mgr_handle_frame_process(msg_ptr,
			ICP_FRAME_PROCESS_SUCCESS);
}

static int cam_icp_mgr_process_msg_config_io(uint32_t *msg_ptr)
{
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct hfi_msg_dev_async_ack *ioconfig_ack = NULL;

	if (!msg_ptr) {
		CAM_ERR(CAM_ICP, "msg ptr is NULL");
		return -EINVAL;
	}

	ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;

	if (ioconfig_ack->opcode == HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO) {
		struct hfi_msg_ipe_config *ipe_config_ack = NULL;

		ipe_config_ack =
			(struct hfi_msg_ipe_config *)(ioconfig_ack->msg_data);
		if (ipe_config_ack->rc) {
			CAM_ERR(CAM_ICP, "rc = %d failed with\n"
				"err_no = [%u] err_type = [%s]",
				ipe_config_ack->rc,
				ioconfig_ack->err_type,
				cam_icp_error_handle_id_to_type(
				ioconfig_ack->err_type));

			return -EIO;
		}
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if (!ctx_data) {
			CAM_ERR(CAM_ICP, "wrong ctx data from IPE config io response");
			return -EINVAL;
		}
		CAM_DBG(CAM_ICP, "%s: received IPE config io response",
			ctx_data->ctx_id_string);
		ctx_data->scratch_mem_size = ipe_config_ack->scratch_mem_size;
	} else if (ioconfig_ack->opcode == HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO) {
		struct hfi_msg_bps_config *bps_config_ack = NULL;

		bps_config_ack =
			(struct hfi_msg_bps_config *)(ioconfig_ack->msg_data);
		if (bps_config_ack->rc) {
			CAM_ERR(CAM_ICP, "rc : %u, opcode :%u",
				bps_config_ack->rc, ioconfig_ack->opcode);
			return -EIO;
		}
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if (!ctx_data) {
			CAM_ERR(CAM_ICP, "wrong ctx data from BPS config io response");
			return -EINVAL;
		}
		CAM_DBG(CAM_ICP, "%s: received BPS config io response",
			ctx_data->ctx_id_string);
	} else {
		CAM_ERR(CAM_ICP, "Invalid OPCODE: %u", ioconfig_ack->opcode);
		return -EINVAL;
	}

	complete(&ctx_data->wait_complete);

	return 0;
}

static int cam_icp_mgr_process_msg_create_handle(uint32_t *msg_ptr)
{
	struct hfi_msg_create_handle_ack *create_handle_ack = NULL;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	int rc = 0;

	create_handle_ack = (struct hfi_msg_create_handle_ack *)msg_ptr;
	if (!create_handle_ack) {
		CAM_ERR(CAM_ICP, "Invalid create_handle_ack");
		return -EINVAL;
	}

	ctx_data =
		(struct cam_icp_hw_ctx_data *)(uintptr_t)
		create_handle_ack->user_data1;
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Invalid ctx_data");
		return -EINVAL;
	}

	if (ctx_data->state == CAM_ICP_CTX_STATE_IN_USE) {
		ctx_data->fw_handle = create_handle_ack->fw_handle;
		CAM_DBG(CAM_ICP, "%s: fw_handle = %x",
			ctx_data->ctx_id_string, ctx_data->fw_handle);
	} else {
		CAM_WARN(CAM_ICP,
			"%s: This ctx is no longer in use current state: %d",
			ctx_data->ctx_id_string, ctx_data->state);
		ctx_data->fw_handle = 0;
		rc = -EPERM;
	}
	complete(&ctx_data->wait_complete);
	return rc;
}

static int cam_icp_mgr_process_msg_ping_ack(uint32_t *msg_ptr)
{
	struct hfi_msg_ping_ack *ping_ack = NULL;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;

	ping_ack = (struct hfi_msg_ping_ack *)msg_ptr;
	if (!ping_ack) {
		CAM_ERR(CAM_ICP, "Empty ping ack message");
		return -EINVAL;
	}

	ctx_data = (struct cam_icp_hw_ctx_data *)
		U64_TO_PTR(ping_ack->user_data);
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Invalid ctx_data");
		return -EINVAL;
	}

	if (ctx_data->state == CAM_ICP_CTX_STATE_IN_USE)
		complete(&ctx_data->wait_complete);

	return 0;
}

static int cam_icp_mgr_process_ipebps_indirect_ack_msg(uint32_t *msg_ptr)
{
	int rc = 0;

	switch (msg_ptr[ICP_PACKET_OPCODE]) {
	case HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO:
	case HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO:
		CAM_DBG(CAM_ICP, "received IPE/BPS_CONFIG_IO:");
		rc = cam_icp_mgr_process_msg_config_io(msg_ptr);
		if (rc)
			return rc;
		break;
	case HFI_IPEBPS_CMD_OPCODE_IPE_FRAME_PROCESS:
	case HFI_IPEBPS_CMD_OPCODE_BPS_FRAME_PROCESS:
		CAM_DBG(CAM_ICP, "received IPE/BPS_FRAME_PROCESS:");
		rc = cam_icp_mgr_process_msg_frame_process(msg_ptr);
		if (rc)
			return rc;
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid opcode : %u",
			msg_ptr[ICP_PACKET_OPCODE]);
		return -EINVAL;
	}

	return rc;
}

static int cam_icp_mgr_process_ofe_indirect_ack_msg(uint32_t *msg_ptr)
{
	int rc = 0;

	switch (msg_ptr[ICP_PACKET_OPCODE]) {
	case HFI_OFE_CMD_OPCODE_CONFIG_IO: {
		struct hfi_msg_dev_async_ack *ioconfig_ack =
			(struct hfi_msg_dev_async_ack *)msg_ptr;
		struct hfi_msg_ofe_config *ofe_config_ack =
			(struct hfi_msg_ofe_config *)(ioconfig_ack->msg_data);
		struct cam_icp_hw_ctx_data *ctx_data = NULL;

		if (ofe_config_ack->rc) {
			CAM_ERR(CAM_ICP, "rc : %u, error type: %u error: [%s] opcode :%u",
				ofe_config_ack->rc, ioconfig_ack->err_type,
				cam_icp_error_handle_id_to_type(ioconfig_ack->err_type),
				ioconfig_ack->opcode);
			return -EIO;
		}
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if (!ctx_data) {
			CAM_ERR(CAM_ICP, "wrong ctx data from OFE config io response");
			return -EINVAL;
		}
		CAM_DBG(CAM_ICP, "%s: received OFE config io response",
			ctx_data->ctx_id_string);

		complete(&ctx_data->wait_complete);
		break;
	}
	case HFI_OFE_CMD_OPCODE_FRAME_PROCESS:
		CAM_DBG(CAM_ICP, "received OFE_FRAME_PROCESS:");
		rc = cam_icp_mgr_process_msg_frame_process(msg_ptr);
		if (rc)
			return rc;
		break;
	case HFI_OFE_CMD_OPCODE_ABORT: {
		struct hfi_msg_dev_async_ack *ioconfig_ack = NULL;
		struct cam_icp_hw_ctx_data *ctx_data = NULL;
		struct cam_icp_hw_mgr *hw_mgr;

		ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
		ctx_data = U64_TO_PTR(ioconfig_ack->user_data1);
		if (cam_presil_mode_enabled()) {
			hw_mgr = ctx_data->hw_mgr_priv;
			if (atomic_read(&hw_mgr->frame_in_process)) {
				if (hw_mgr->frame_in_process_ctx_id == ctx_data->ctx_id) {
					CAM_DBG(CAM_PRESIL, "presil: frame process abort ctx %d",
						hw_mgr->frame_in_process_ctx_id);
						hw_mgr->frame_in_process_ctx_id = -1;
					atomic_set(&hw_mgr->frame_in_process, 0);
					up_write(&frame_in_process_sem);
				} else {
					CAM_WARN(CAM_PRESIL, "presil: abort mismatch %d %d",
						hw_mgr->frame_in_process_ctx_id,
						ctx_data->ctx_id);
				}
			}
		}
		if (ctx_data->state != CAM_ICP_CTX_STATE_FREE)
			complete(&ctx_data->wait_complete);

		CAM_DBG(CAM_ICP, "received OFE Abort done msg ctx_state: %u",
			ctx_data->state);
		break;
	}
	case HFI_OFE_CMD_OPCODE_DESTROY: {
		struct hfi_msg_dev_async_ack *ioconfig_ack = NULL;
		struct cam_icp_hw_ctx_data *ctx_data = NULL;

		ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
		ctx_data = U64_TO_PTR(ioconfig_ack->user_data1);
		if ((ctx_data->state == CAM_ICP_CTX_STATE_RELEASE) ||
			(ctx_data->state == CAM_ICP_CTX_STATE_IN_USE))
			complete(&ctx_data->wait_complete);

		CAM_DBG(CAM_ICP, "received OFE destroy done msg: %u",
			ctx_data->state);
		break;
	}
	default:
		CAM_ERR(CAM_ICP, "Invalid opcode : %u",
			msg_ptr[ICP_PACKET_OPCODE]);
		return -EINVAL;
	}

	return rc;
}

static int cam_icp_mgr_process_direct_ack_msg(uint32_t *msg_ptr)
{
	struct cam_icp_hw_mgr *hw_mgr = NULL;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct hfi_msg_dev_async_ack *ioconfig_ack = NULL;
	int rc = 0;

	if (!msg_ptr)
		return -EINVAL;

	switch (msg_ptr[ICP_PACKET_OPCODE]) {
	case HFI_IPEBPS_CMD_OPCODE_IPE_ABORT:
	case HFI_IPEBPS_CMD_OPCODE_BPS_ABORT:
		ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);

		if (cam_presil_mode_enabled()) {
			hw_mgr = ctx_data->hw_mgr_priv;
			if (atomic_read(&hw_mgr->frame_in_process)) {
				if (hw_mgr->frame_in_process_ctx_id == ctx_data->ctx_id) {
					CAM_DBG(CAM_PRESIL, "%s: presil: frame process abort",
						ctx_data->ctx_id_string);
					hw_mgr->frame_in_process_ctx_id = -1;
					atomic_set(&hw_mgr->frame_in_process, 0);
					up_write(&frame_in_process_sem);
				} else {
					CAM_WARN(CAM_PRESIL,
						"%s: presil: abort mismatch frame_in_process_ctx_id: %d",
						ctx_data->ctx_id_string,
						hw_mgr->frame_in_process_ctx_id);
				}
			}
		}

		if (ctx_data->state != CAM_ICP_CTX_STATE_FREE)
			complete(&ctx_data->wait_complete);
		CAM_DBG(CAM_ICP, "%s: received ABORT: ctx_state =%d",
			ctx_data->ctx_id_string, ctx_data->state);
		break;
	case HFI_IPEBPS_CMD_OPCODE_IPE_DESTROY:
	case HFI_IPEBPS_CMD_OPCODE_BPS_DESTROY:
		ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
		ctx_data = (struct cam_icp_hw_ctx_data *)
			U64_TO_PTR(ioconfig_ack->user_data1);
		if ((ctx_data->state == CAM_ICP_CTX_STATE_RELEASE) ||
			(ctx_data->state == CAM_ICP_CTX_STATE_IN_USE)) {
			complete(&ctx_data->wait_complete);
		}
		CAM_DBG(CAM_ICP, "%s: received DESTROY: ctx_state =%d",
			ctx_data->ctx_id_string, ctx_data->state);
		break;
	case HFI_IPEBPS_CMD_OPCODE_MEM_MAP:
		ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
		ctx_data =
			(struct cam_icp_hw_ctx_data *)ioconfig_ack->user_data1;
		if (ctx_data->state != CAM_ICP_CTX_STATE_FREE)
			complete(&ctx_data->wait_complete);
		CAM_DBG(CAM_ICP, "%s: received\n"
			"MAP ACK:ctx_state =%d\n"
			"failed with err_no = [%u] err_type = [%s]",
			ctx_data->ctx_id_string, ctx_data->state, ioconfig_ack->err_type,
			cam_icp_error_handle_id_to_type(ioconfig_ack->err_type));
		break;
	case HFI_IPEBPS_CMD_OPCODE_MEM_UNMAP:
		ioconfig_ack = (struct hfi_msg_dev_async_ack *)msg_ptr;
		ctx_data =
			(struct cam_icp_hw_ctx_data *)ioconfig_ack->user_data1;
		if (ctx_data->state != CAM_ICP_CTX_STATE_FREE)
			complete(&ctx_data->wait_complete);
		CAM_DBG(CAM_ICP,
			"%s: received UNMAP ACK:ctx_state =%d\n"
			"failed with err_no = [%u] err_type = [%s]",
			ctx_data->ctx_id_string, ctx_data->state, ioconfig_ack->err_type,
			cam_icp_error_handle_id_to_type(ioconfig_ack->err_type));
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid opcode : %u",
			msg_ptr[ICP_PACKET_OPCODE]);
		rc = -EINVAL;
	}
	return rc;
}

static int cam_icp_dev_reset(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0, i, j;
	struct cam_icp_hw_device_info *dev_info = NULL;
	struct cam_hw_intf *dev_intf = NULL;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		dev_info = &hw_mgr->dev_info[i];
		if (!dev_info->dev_ctx_info.dev_ctxt_cnt)
			continue;

		for (j = 0; j < dev_info->hw_dev_cnt; j++) {
			dev_intf = dev_info->dev_intf[j];
			if (!dev_intf) {
				CAM_ERR(CAM_ICP, "Device intf for %s[%u] is NULL",
					dev_info->dev_name, j);
				return -EINVAL;
			}
			rc = dev_intf->hw_ops.process_cmd(dev_intf->hw_priv,
				CAM_ICP_DEV_CMD_RESET, NULL, 0);
			if (rc)
				CAM_ERR(CAM_ICP, "[%s] %s[%u] reset failed rc: %d",
					hw_mgr->hw_mgr_name, dev_info->dev_name, j, rc);
		}
	}

	return rc;
}

static int cam_icp_mgr_trigger_recovery(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0, i;
	struct sfr_buf *sfr_buffer = NULL;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct cam_icp_hw_error_evt_data icp_err_evt = {0};

	CAM_DBG(CAM_ICP, "[%s] Enter", hw_mgr->hw_mgr_name);

	if (atomic_read(&hw_mgr->recovery)) {
		CAM_ERR(CAM_ICP, "%s Recovery is set", hw_mgr->hw_mgr_name);
		return rc;
	}

	sfr_buffer = (struct sfr_buf *)hw_mgr->hfi_mem.sfr_buf.kva;
	CAM_WARN(CAM_ICP, "[%s] hw_mgr[%u] SFR:%s", hw_mgr->hw_mgr_name,
		hw_mgr->hw_mgr_id, sfr_buffer->msg);
	cam_icp_dump_debug_info(hw_mgr, false);
	cam_icp_mgr_dump_active_req_info(hw_mgr);

	cam_icp_mgr_dev_get_gdsc_control(hw_mgr);
	cam_icp_dev_reset(hw_mgr);

	atomic_set(&hw_mgr->recovery, 1);

	/* Find any active context and notify userspace of system failure */
	icp_err_evt.err_type = CAM_ICP_HW_ERROR_SYSTEM_FAILURE;
	mutex_lock(&hw_mgr->hw_mgr_mutex);
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx_data = &hw_mgr->ctx_data[i];
		mutex_lock(&ctx_data->ctx_mutex);
		if (ctx_data->state != CAM_CTX_ACQUIRED) {
			mutex_unlock(&ctx_data->ctx_mutex);
			continue;
		}
		ctx_data->ctxt_event_cb(ctx_data->context_priv, CAM_ICP_EVT_ID_ERROR,
			&icp_err_evt);
		mutex_unlock(&ctx_data->ctx_mutex);
		break;
	}
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	if (i == CAM_ICP_CTX_MAX)
		CAM_ERR(CAM_ICP,
			"[%s] Fail to report system failure to userspace due to no active ctx",
			hw_mgr->hw_mgr_name);

	complete(&hw_mgr->icp_complete);
	CAM_DBG(CAM_ICP, "[%s] Done", hw_mgr->hw_mgr_name);
	return rc;
}

static int cam_icp_mgr_process_fatal_error(
	struct cam_icp_hw_mgr *hw_mgr, uint32_t *msg_ptr)
{
	struct hfi_msg_event_notify *event_notify;
	int rc = 0;

	CAM_DBG(CAM_ICP, "%s: Enter", hw_mgr->hw_mgr_name);

	event_notify = (struct hfi_msg_event_notify *)msg_ptr;
	if (!event_notify) {
		CAM_ERR(CAM_ICP, "[%s] Empty event message", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "[%s] evt_id: %u evt_data1: %u evt_data2: %u",
		hw_mgr->hw_mgr_name, event_notify->event_id, event_notify->event_data1,
		event_notify->event_data2);

	if (event_notify->event_id == HFI_EVENT_SYS_ERROR) {
		CAM_INFO(CAM_ICP, "[%s] received HFI_EVENT_SYS_ERROR",
			hw_mgr->hw_mgr_name);
		if (event_notify->event_data1 == HFI_ERR_SYS_FATAL) {
			CAM_ERR(CAM_ICP, "[%s] received HFI_ERR_SYS_FATAL",
				hw_mgr->hw_mgr_name);
			BUG();
		}
		rc = cam_icp_mgr_trigger_recovery(hw_mgr);
	}

	return rc;
}

static void cam_icp_mgr_process_dbg_buf(struct cam_icp_hw_mgr *hw_mgr)
{
	uint32_t *msg_ptr = NULL, *pkt_ptr = NULL;
	struct hfi_msg_debug *dbg_msg;
	uint32_t read_len, size_processed = 0, debug_lvl;
	uint64_t timestamp = 0;
	char *dbg_buf;
	int rc = 0;

	rc = hfi_read_message(hw_mgr->hfi_handle, hw_mgr->dbg_buf, Q_DBG,
		ICP_DBG_BUF_SIZE_IN_WORDS, &read_len);
	if (rc)
		return;

	msg_ptr = (uint32_t *)hw_mgr->dbg_buf;
	debug_lvl = hw_mgr->icp_dbg_lvl;
	while (true) {
		pkt_ptr = msg_ptr;
		if (pkt_ptr[ICP_PACKET_TYPE] == HFI_MSG_SYS_DEBUG) {
			dbg_msg = (struct hfi_msg_debug *)pkt_ptr;
			dbg_buf = (char *)&dbg_msg->msg_data;
			timestamp = ((((uint64_t)(dbg_msg->timestamp_hi) << 32)
				| dbg_msg->timestamp_lo) >> 16);
			trace_cam_icp_fw_dbg(dbg_buf, timestamp/2,
				hw_mgr->hw_mgr_name);
			if (!debug_lvl)
				CAM_INFO(CAM_ICP, "[%s]: FW_DBG:%s",
					hw_mgr->hw_mgr_name, dbg_buf);
		}
		size_processed += (pkt_ptr[ICP_PACKET_SIZE] >>
			BYTE_WORD_SHIFT);
		if (size_processed >= read_len)
			return;
		msg_ptr += (pkt_ptr[ICP_PACKET_SIZE] >>
		BYTE_WORD_SHIFT);
	}
}

static int cam_icp_process_msg_pkt_type(
	struct cam_icp_hw_mgr *hw_mgr,
	uint32_t *msg_ptr,
	uint32_t *msg_processed_len)
{
	int rc = 0;
	int size_processed = 0;

	switch (msg_ptr[ICP_PACKET_TYPE]) {
	case HFI_MSG_SYS_INIT_DONE:
		CAM_DBG(CAM_ICP, "[%s] received SYS_INIT_DONE", hw_mgr->hw_mgr_name);
		complete(&hw_mgr->icp_complete);
		size_processed = (
			(struct hfi_msg_init_done *)msg_ptr)->size;
		break;

	case HFI_MSG_SYS_PC_PREP_DONE:
		CAM_DBG(CAM_ICP, "[%s] HFI_MSG_SYS_PC_PREP_DONE is received\n",
			hw_mgr->hw_mgr_name);
		complete(&hw_mgr->icp_complete);
		size_processed = sizeof(struct hfi_msg_pc_prep_done);
		break;

	case HFI_MSG_SYS_PING_ACK:
		CAM_DBG(CAM_ICP, "[%s] received SYS_PING_ACK", hw_mgr->hw_mgr_name);
		rc = cam_icp_mgr_process_msg_ping_ack(msg_ptr);
		size_processed = sizeof(struct hfi_msg_ping_ack);
		break;

	case HFI_MSG_IPEBPS_CREATE_HANDLE_ACK:
	case HFI_MSG_OFE_CREATE_HANDLE_ACK:
		CAM_DBG(CAM_ICP, "[%s] received IPE/BPS/OFE CREATE_HANDLE_ACK",
			hw_mgr->hw_mgr_name);
		rc = cam_icp_mgr_process_msg_create_handle(msg_ptr);
		size_processed = sizeof(struct hfi_msg_create_handle_ack);
		break;

	case HFI_MSG_IPEBPS_ASYNC_COMMAND_INDIRECT_ACK:
		CAM_DBG(CAM_ICP, "[%s] received IPE/BPS ASYNC_INDIRECT_ACK",
			hw_mgr->hw_mgr_name);
		rc = cam_icp_mgr_process_ipebps_indirect_ack_msg(msg_ptr);
		size_processed = (
			(struct hfi_msg_dev_async_ack *)msg_ptr)->size;
		break;

	case HFI_MSG_OFE_ASYNC_COMMAND_ACK:
		CAM_DBG(CAM_ICP, "[%s] received OFE ASYNC COMMAND ACK",
			hw_mgr->hw_mgr_name);
		rc = cam_icp_mgr_process_ofe_indirect_ack_msg(msg_ptr);
		size_processed = (
			(struct hfi_msg_dev_async_ack *)msg_ptr)->size;
		break;

	case HFI_MSG_IPEBPS_ASYNC_COMMAND_DIRECT_ACK:
		CAM_DBG(CAM_ICP, "[%s] received ASYNC_DIRECT_ACK", hw_mgr->hw_mgr_name);
		rc = cam_icp_mgr_process_direct_ack_msg(msg_ptr);
		size_processed = (
			(struct hfi_msg_dev_async_ack *)msg_ptr)->size;
		break;

	case HFI_MSG_EVENT_NOTIFY:
		CAM_DBG(CAM_ICP, "[%s] received EVENT_NOTIFY", hw_mgr->hw_mgr_name);
		size_processed = (
			(struct hfi_msg_event_notify *)msg_ptr)->size;
		rc = cam_icp_mgr_process_fatal_error(hw_mgr, msg_ptr);
		if (rc)
			CAM_ERR(CAM_ICP, "[%s] failed in processing evt notify",
				hw_mgr->hw_mgr_name);

		break;

	case HFI_MSG_DBG_SYNX_TEST:
		CAM_DBG(CAM_ICP, "received DBG_SYNX_TEST");
		size_processed = sizeof(struct hfi_cmd_synx_test_payload);
		complete(&hw_mgr->icp_complete);
		break;
	default:
		CAM_ERR(CAM_ICP, "[%s] invalid msg : %u",
			hw_mgr->hw_mgr_name, msg_ptr[ICP_PACKET_TYPE]);
		rc = -EINVAL;
	}

	*msg_processed_len = size_processed;
	return rc;
}

static int32_t cam_icp_mgr_process_msg(void *priv, void *data)
{
	uint32_t read_len, msg_processed_len;
	uint32_t *msg_ptr = NULL;
	struct hfi_msg_work_data *task_data;
	struct cam_icp_hw_mgr *hw_mgr;
	int rc = 0;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid data");
		return -EINVAL;
	}

	task_data = data;
	hw_mgr = priv;

	rc = hfi_read_message(hw_mgr->hfi_handle, hw_mgr->msg_buf, Q_MSG,
		ICP_MSG_BUF_SIZE_IN_WORDS, &read_len);
	if (rc) {
		CAM_DBG(CAM_ICP, "Unable to read msg q rc %d", rc);
	} else {
		read_len = read_len << BYTE_WORD_SHIFT;
		msg_ptr = (uint32_t *)hw_mgr->msg_buf;
		while (true) {
			cam_icp_process_msg_pkt_type(hw_mgr, msg_ptr,
				&msg_processed_len);

			if (!msg_processed_len) {
				CAM_ERR(CAM_ICP, "Failed to read");
				rc = -EINVAL;
				break;
			}

			read_len -= msg_processed_len;
			if (read_len > 0) {
				msg_ptr += (msg_processed_len >>
				BYTE_WORD_SHIFT);
				msg_processed_len = 0;
			} else {
				break;
			}
		}
	}

	cam_icp_mgr_process_dbg_buf(hw_mgr);

	if (task_data->recover) {
		CAM_ERR_RATE_LIMIT(CAM_ICP, "issuing device recovery...");

		rc = cam_icp_mgr_trigger_recovery(hw_mgr);
	}

	return rc;
}

static int32_t cam_icp_hw_mgr_cb(void *data, bool recover)
{
	int rc = 0;
	unsigned long flags;
	struct cam_icp_hw_mgr *hw_mgr = data;
	struct crm_workq_task *task;
	struct hfi_msg_work_data *task_data;

	if (!data) {
		CAM_ERR(CAM_ICP, "irq cb data is NULL");
		return rc;
	}

	spin_lock_irqsave(&hw_mgr->hw_mgr_lock, flags);
	task = cam_req_mgr_workq_get_task(hw_mgr->msg_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "no empty task");
		spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);
		return -ENOMEM;
	}

	task_data = (struct hfi_msg_work_data *)task->payload;
	task_data->data = hw_mgr;
	task_data->recover = recover;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_mgr_process_msg;
	rc = cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);
	spin_unlock_irqrestore(&hw_mgr->hw_mgr_lock, flags);

	return rc;
}

static void cam_icp_free_fw_mem(struct cam_icp_hw_mgr *hw_mgr)
{
	/* Skip freeing FW memory if not allocated */
	if (hw_mgr->icp_use_pil)
		return;

	cam_smmu_dealloc_firmware(hw_mgr->iommu_hdl);
}

static void cam_icp_free_hfi_mem(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;

	cam_icp_free_fw_mem(hw_mgr);

	if (hw_mgr->hfi_mem.fw_uncached_region) {
		rc = cam_mem_mgr_free_memory_region(
			&hw_mgr->hfi_mem.fw_uncached_generic);
		if (rc)
			CAM_ERR(CAM_ICP,
				"[%s] failed to unreserve fwuncached region", hw_mgr->hw_mgr_name);
			hw_mgr->hfi_mem.fw_uncached_region = false;
	} else {
		rc = cam_mem_mgr_free_memory_region(
			&hw_mgr->hfi_mem.sec_heap);
		if (rc)
			CAM_ERR(CAM_ICP, "[%s] failed to unreserve sec heap", hw_mgr->hw_mgr_name);

		cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.qtbl);
		cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.cmd_q);
		cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.msg_q);
		cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.dbg_q);
		cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.sfr_buf);
	}

	cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl, CAM_SMMU_REGION_QDSS, 0);

	/* Skip freeing if not mapped */
	if (hw_mgr->synx_signaling_en) {
		cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl, CAM_SMMU_REGION_FWUNCACHED,
			CAM_SMMU_SUBREGION_GLOBAL_SYNC_MEM);
		cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl, CAM_SMMU_REGION_DEVICE,
			CAM_SMMU_SUBREGION_SYNX_HWMUTEX);
		cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl, CAM_SMMU_REGION_DEVICE,
			CAM_SMMU_SUBREGION_IPC_HWMUTEX);
		cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl, CAM_SMMU_REGION_DEVICE,
			CAM_SMMU_SUBREGION_GLOBAL_CNTR);
	}
}

static int cam_icp_alloc_secheap_mem(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_mem_mgr_memory_desc *secheap)
{
	int rc;
	struct cam_mem_mgr_request_desc alloc;
	struct cam_mem_mgr_memory_desc out;
	struct cam_smmu_region_info secheap_info;

	rc = cam_smmu_get_region_info(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_SECHEAP, &secheap_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Unable to get secheap memory info", hw_mgr->hw_mgr_name);
		return rc;
	}

	alloc.size = secheap_info.iova_len;
	alloc.align = 0;
	alloc.flags = 0;
	alloc.smmu_hdl = hw_mgr->iommu_hdl;
	rc = cam_mem_mgr_reserve_memory_region(&alloc,
		CAM_SMMU_REGION_SECHEAP,
		&out);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Unable to reserve secheap memory", hw_mgr->hw_mgr_name);
		return rc;
	}

	*secheap = out;
	CAM_DBG(CAM_ICP, "[%s] kva: %llX, iova: %x, hdl: %x, len: %lld",
		hw_mgr->hw_mgr_name, out.kva, out.iova, out.mem_handle, out.len);

	return rc;
}

static int cam_icp_alloc_shared_mem(
	struct cam_icp_hw_mgr *hw_mgr, size_t size_requested,
	struct cam_mem_mgr_memory_desc *alloc_out)
{
	int rc;
	struct cam_mem_mgr_request_desc alloc;
	struct cam_mem_mgr_memory_desc out;

	alloc.size = size_requested;
	alloc.align = 0;
	alloc.flags = CAM_MEM_FLAG_HW_READ_WRITE |
		CAM_MEM_FLAG_HW_SHARED_ACCESS;
	alloc.smmu_hdl = hw_mgr->iommu_hdl;
	rc = cam_mem_mgr_request_mem(&alloc, &out);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in alloc shared mem rc %d",
		hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	*alloc_out = out;
	CAM_DBG(CAM_ICP, "[%s] kva: %llX, iova: %x, hdl: %x, len: %lld",
		hw_mgr->hw_mgr_name,
		out.kva, out.iova, out.mem_handle, out.len);

	return rc;
}

static int cam_icp_allocate_fw_mem(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	uintptr_t kvaddr;
	size_t len;
	dma_addr_t iova;

	if (hw_mgr->icp_use_pil)
		return 0;

	rc = cam_smmu_alloc_firmware(hw_mgr->iommu_hdl,
		&iova, &kvaddr, &len);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in alloc firmware rc %d",
		hw_mgr->hw_mgr_name, rc);
		return -ENOMEM;
	}

	hw_mgr->hfi_mem.fw_buf.len = len;
	hw_mgr->hfi_mem.fw_buf.kva = kvaddr;
	hw_mgr->hfi_mem.fw_buf.iova = iova;
	hw_mgr->hfi_mem.fw_buf.smmu_hdl = hw_mgr->iommu_hdl;

	CAM_DBG(CAM_ICP, "[%s], kva: %zX, iova: %llx, len: %zu",
		hw_mgr->hw_mgr_name,
		kvaddr, iova, len);

	return rc;
}

static int cam_icp_allocate_qdss_mem(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	size_t len;
	dma_addr_t iova;

	rc = cam_smmu_map_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_QDSS, 0, &iova, &len);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in alloc qdss mem rc %d",
		hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	hw_mgr->hfi_mem.qdss_buf.len = len;
	hw_mgr->hfi_mem.qdss_buf.iova = iova;
	hw_mgr->hfi_mem.qdss_buf.smmu_hdl = hw_mgr->iommu_hdl;

	CAM_DBG(CAM_ICP, "[%s] iova: %llx, len: %zu",
		hw_mgr->hw_mgr_name, iova, len);

	return rc;
}

static int cam_icp_allocate_global_sync_mem(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	size_t len;
	dma_addr_t iova;

	rc = cam_smmu_map_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_FWUNCACHED,
		CAM_SMMU_SUBREGION_GLOBAL_SYNC_MEM, &iova, &len);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"[%s] Failed in allocating global sync ipc mem rc %d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	hw_mgr->hfi_mem.fw_uncached_global_sync.len = len;
	hw_mgr->hfi_mem.fw_uncached_global_sync.iova = iova;
	hw_mgr->hfi_mem.fw_uncached_global_sync.smmu_hdl = hw_mgr->iommu_hdl;

	CAM_DBG(CAM_ICP, "[%s] iova: %llx, len: %zu",
		hw_mgr->hw_mgr_name, iova, len);

	return rc;
}

static int cam_icp_allocate_device_synx_hwmutex_mem(
	struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	size_t len;
	dma_addr_t iova;

	rc = cam_smmu_map_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_DEVICE, CAM_SMMU_SUBREGION_SYNX_HWMUTEX, &iova, &len);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"Failed in allocating hwmutex mem rc %d", rc);
		return rc;
	}

	hw_mgr->hfi_mem.synx_hwmutex.len = len;
	hw_mgr->hfi_mem.synx_hwmutex.iova = iova;
	hw_mgr->hfi_mem.synx_hwmutex.smmu_hdl = hw_mgr->iommu_hdl;

	CAM_DBG(CAM_ICP, "[%s] iova: %llx, len: %zu",
		hw_mgr->hw_mgr_name, iova, len);

	return rc;
}

static int cam_icp_allocate_device_global_cnt_mem(
	struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	size_t len;
	dma_addr_t iova;

	rc = cam_smmu_map_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_DEVICE, CAM_SMMU_SUBREGION_GLOBAL_CNTR,
		&iova, &len);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"Failed in allocating global cntr mem rc %d", rc);
		return rc;
	}

	hw_mgr->hfi_mem.global_cntr.len = len;
	hw_mgr->hfi_mem.global_cntr.iova = iova;
	hw_mgr->hfi_mem.global_cntr.smmu_hdl = hw_mgr->iommu_hdl;

	CAM_DBG(CAM_ICP, "[%s] iova: %llx, len: %zu",
		hw_mgr->hw_mgr_name, iova, len);

	return rc;
}

static int cam_icp_allocate_device_ipc_hwmutex_mem(
	struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	size_t len;
	dma_addr_t iova;

	rc = cam_smmu_map_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_DEVICE, CAM_SMMU_SUBREGION_IPC_HWMUTEX,
		&iova, &len);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"Failed in allocating hwmutex mem rc %d", rc);
		return rc;
	}

	hw_mgr->hfi_mem.ipc_hwmutex.len = len;
	hw_mgr->hfi_mem.ipc_hwmutex.iova = iova;
	hw_mgr->hfi_mem.ipc_hwmutex.smmu_hdl = hw_mgr->iommu_hdl;

	CAM_DBG(CAM_ICP, "[%s] iova: %llx, len: %zu",
		hw_mgr->hw_mgr_name, iova, len);

	return rc;
}

static int cam_icp_allocate_mem_for_fence_signaling(
	struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;

	rc = cam_smmu_get_region_info(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_DEVICE, &hw_mgr->hfi_mem.device);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"[%s] Unable to get device memory info rc %d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	rc = cam_icp_allocate_global_sync_mem(hw_mgr);
	if (rc)
		return rc;

	rc = cam_icp_allocate_device_synx_hwmutex_mem(hw_mgr);
	if (rc)
		goto unmap_global_sync;

	rc = cam_icp_allocate_device_ipc_hwmutex_mem(hw_mgr);
	if (rc)
		goto unmap_synx_hwmutex;

	rc = cam_icp_allocate_device_global_cnt_mem(hw_mgr);
	if (rc)
		goto unmap_ipc_mutex;

	return 0;

unmap_ipc_mutex:
	cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_DEVICE,
		CAM_SMMU_SUBREGION_IPC_HWMUTEX);
unmap_synx_hwmutex:
	cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_DEVICE,
		CAM_SMMU_SUBREGION_SYNX_HWMUTEX);
unmap_global_sync:
	cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_FWUNCACHED,
		CAM_SMMU_SUBREGION_GLOBAL_SYNC_MEM);

	return rc;
}

static int cam_icp_get_io_mem_info(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	size_t len, discard_iova_len;
	dma_addr_t iova, discard_iova_start;

	rc = cam_smmu_get_io_region_info(hw_mgr->iommu_hdl,
		&iova, &len, &discard_iova_start, &discard_iova_len);
	if (rc)
		return rc;

	hw_mgr->hfi_mem.io_mem.iova_len = len;
	hw_mgr->hfi_mem.io_mem.iova_start = iova;
	hw_mgr->hfi_mem.io_mem.discard_iova_start = discard_iova_start;
	hw_mgr->hfi_mem.io_mem.discard_iova_len = discard_iova_len;

	CAM_DBG(CAM_ICP, "[%s] iova: %llx, len: %zu discard iova %llx len %llx",
		hw_mgr->hw_mgr_name, iova, len, discard_iova_start, discard_iova_len);

	return rc;
}

static int cam_icp_allocate_hfi_mem(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	struct cam_smmu_region_info fwuncached_region_info = {0};
	bool fwuncached_region_exists = false;
	size_t qtbl_size, cmdq_size, msgq_size, dbgq_size, sfr_size, sec_heap_size;

	rc = cam_smmu_get_region_info(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_SHARED, &hw_mgr->hfi_mem.shmem);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Unable to get shared memory info rc %d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	rc = cam_icp_allocate_fw_mem(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Unable to allocate FW memory rc %d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	rc = cam_icp_allocate_qdss_mem(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Unable to allocate qdss memory rc %d",
			hw_mgr->hw_mgr_name, rc);
		goto fw_alloc_failed;
	}

	/*
	 * Compute sizes aligned to page size, and add a padding
	 * of a page between regions
	 */
	qtbl_size = ALIGN(ICP_QTBL_SIZE_IN_BYTES, PAGE_SIZE) + PAGE_SIZE;
	cmdq_size = ALIGN(ICP_CMD_Q_SIZE_IN_BYTES, PAGE_SIZE) + PAGE_SIZE;
	msgq_size = ALIGN(ICP_MSG_Q_SIZE_IN_BYTES, PAGE_SIZE) + PAGE_SIZE;
	dbgq_size = ALIGN(ICP_DBG_Q_SIZE_IN_BYTES, PAGE_SIZE) + PAGE_SIZE;
	sfr_size = ALIGN(ICP_MSG_SFR_SIZE_IN_BYTES, PAGE_SIZE) + PAGE_SIZE;
	sec_heap_size = ALIGN(ICP_SEC_HEAP_SIZE_IN_BYTES, PAGE_SIZE) + PAGE_SIZE;

	rc = cam_smmu_get_region_info(hw_mgr->iommu_hdl,
		CAM_SMMU_REGION_FWUNCACHED, &fwuncached_region_info);
	if (!rc)
		fwuncached_region_exists = true;

	if (fwuncached_region_exists) {
		struct cam_mem_mgr_request_desc alloc;
		struct cam_mem_mgr_memory_desc out;
		uint32_t offset;

		/*
		 * FW uncached consists of the qtbl, HFI queues, SFR buffer
		 * and secondary heap
		 */
		alloc.size = qtbl_size + cmdq_size + msgq_size + dbgq_size +
			sfr_size + sec_heap_size;
		alloc.align = 0;
		alloc.flags = CAM_MEM_FLAG_KMD_ACCESS;
		alloc.smmu_hdl = hw_mgr->iommu_hdl;
		rc = cam_mem_mgr_reserve_memory_region(&alloc,
			CAM_SMMU_REGION_FWUNCACHED, &out);
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] Unable to reserve fw uncached memory rc %d",
				hw_mgr->hw_mgr_name, rc);
			goto qtbl_alloc_failed;
		}

		hw_mgr->hfi_mem.fw_uncached_region = true;
		/*
		 * If the FW uncached region is carved into different subregions
		 * send the whole region to FW, resulting in one ICP MMU entry.
		 * If reserving for FW uncached the SMMU will map the FW uncached
		 * generic, and return the appropriate va
		 *
		 */
		hw_mgr->hfi_mem.fw_uncached = fwuncached_region_info;
		hw_mgr->hfi_mem.fw_uncached_generic = out;

		offset = 0;

		hw_mgr->hfi_mem.sec_heap.iova       = out.iova + offset;
		hw_mgr->hfi_mem.sec_heap.kva        = out.kva + offset;
		hw_mgr->hfi_mem.sec_heap.len        = sec_heap_size;
		hw_mgr->hfi_mem.sec_heap.smmu_hdl   = out.smmu_hdl;
		hw_mgr->hfi_mem.sec_heap.mem_handle = out.mem_handle;
		hw_mgr->hfi_mem.sec_heap.region     = out.region;
		offset += (uint32_t)sec_heap_size;

		hw_mgr->hfi_mem.qtbl.iova       = out.iova + offset;
		hw_mgr->hfi_mem.qtbl.kva        = out.kva + offset;
		hw_mgr->hfi_mem.qtbl.len        = qtbl_size;
		hw_mgr->hfi_mem.qtbl.smmu_hdl   = out.smmu_hdl;
		hw_mgr->hfi_mem.qtbl.mem_handle = out.mem_handle;
		hw_mgr->hfi_mem.qtbl.region     = out.region;
		offset += (uint32_t)qtbl_size;

		hw_mgr->hfi_mem.cmd_q.iova       = out.iova + offset;
		hw_mgr->hfi_mem.cmd_q.kva        = out.kva + offset;
		hw_mgr->hfi_mem.cmd_q.len        = cmdq_size;
		hw_mgr->hfi_mem.cmd_q.smmu_hdl   = out.smmu_hdl;
		hw_mgr->hfi_mem.cmd_q.mem_handle = out.mem_handle;
		hw_mgr->hfi_mem.cmd_q.region     = out.region;
		offset += (uint32_t)cmdq_size;

		hw_mgr->hfi_mem.msg_q.iova       = out.iova + offset;
		hw_mgr->hfi_mem.msg_q.kva        = out.kva + offset;
		hw_mgr->hfi_mem.msg_q.len        = msgq_size;
		hw_mgr->hfi_mem.msg_q.smmu_hdl   = out.smmu_hdl;
		hw_mgr->hfi_mem.msg_q.mem_handle = out.mem_handle;
		hw_mgr->hfi_mem.msg_q.region     = out.region;
		offset += (uint32_t)msgq_size;

		hw_mgr->hfi_mem.dbg_q.iova       = out.iova + offset;
		hw_mgr->hfi_mem.dbg_q.kva        = out.kva + offset;
		hw_mgr->hfi_mem.dbg_q.len        = dbgq_size;
		hw_mgr->hfi_mem.dbg_q.smmu_hdl   = out.smmu_hdl;
		hw_mgr->hfi_mem.dbg_q.mem_handle = out.mem_handle;
		hw_mgr->hfi_mem.dbg_q.region     = out.region;
		offset += (uint32_t)dbgq_size;

		hw_mgr->hfi_mem.sfr_buf.iova       = out.iova + offset;
		hw_mgr->hfi_mem.sfr_buf.kva        = out.kva + offset;
		hw_mgr->hfi_mem.sfr_buf.len        = sfr_size;
		hw_mgr->hfi_mem.sfr_buf.smmu_hdl   = out.smmu_hdl;
		hw_mgr->hfi_mem.sfr_buf.mem_handle = out.mem_handle;
		hw_mgr->hfi_mem.sfr_buf.region     = out.region;
		offset += (uint32_t)sfr_size;

		if (offset > out.len) {
			CAM_ERR(CAM_ICP,
				"[%s] FW uncached region size %lld not enough, required %lld",
				hw_mgr->hw_mgr_name, offset, out.len);
			cam_mem_mgr_free_memory_region(&hw_mgr->hfi_mem.fw_uncached_generic);
			goto qtbl_alloc_failed;
		}
	} else {
		rc = cam_icp_alloc_shared_mem(hw_mgr, qtbl_size, &hw_mgr->hfi_mem.qtbl);
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] Unable to allocate qtbl memory, rc %d",
				hw_mgr->hw_mgr_name, rc);
			goto qtbl_alloc_failed;
		}

		rc = cam_icp_alloc_shared_mem(hw_mgr, cmdq_size, &hw_mgr->hfi_mem.cmd_q);
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] Unable to allocate cmd q memory rc %d",
				hw_mgr->hw_mgr_name, rc);
			goto cmd_q_alloc_failed;
		}

		rc = cam_icp_alloc_shared_mem(hw_mgr, msgq_size, &hw_mgr->hfi_mem.msg_q);
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] Unable to allocate msg q memory rc %d",
				hw_mgr->hw_mgr_name, rc);
			goto msg_q_alloc_failed;
		}

		rc = cam_icp_alloc_shared_mem(hw_mgr, dbgq_size, &hw_mgr->hfi_mem.dbg_q);
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] Unable to allocate dbg q memory rc %d",
				hw_mgr->hw_mgr_name, rc);
			goto dbg_q_alloc_failed;
		}

		rc = cam_icp_alloc_shared_mem(hw_mgr, sfr_size, &hw_mgr->hfi_mem.sfr_buf);
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] Unable to allocate sfr buffer rc %d",
				hw_mgr->hw_mgr_name, rc);
			goto sfr_buf_alloc_failed;
		}

		rc = cam_icp_alloc_secheap_mem(hw_mgr, &hw_mgr->hfi_mem.sec_heap);
		if (rc) {
			CAM_ERR(CAM_ICP, "[%s] Unable to allocate sec heap memory rc %d",
				hw_mgr->hw_mgr_name, rc);
			goto sec_heap_alloc_failed;
		}
	}

	CAM_DBG(CAM_ICP, "[%s] Shared Region [0x%x %lld] FW Uncached nested Region [0x%x %lld]",
		hw_mgr->hw_mgr_name,
		hw_mgr->hfi_mem.shmem.iova_start,
		hw_mgr->hfi_mem.shmem.iova_len,
		fwuncached_region_info.iova_start,
		fwuncached_region_info.iova_len);

	CAM_DBG(CAM_ICP,
		"[%s] FwUncached[0x%x %lld] FwUncached_Generic[0x%x %p %lld] QTbl[0x%x %p %lld] CmdQ[0x%x %p %lld] MsgQ[0x%x %p %lld]",
		hw_mgr->hw_mgr_name,
		hw_mgr->hfi_mem.fw_uncached.iova_start,
		hw_mgr->hfi_mem.fw_uncached.iova_len,
		hw_mgr->hfi_mem.fw_uncached_generic.iova,
		hw_mgr->hfi_mem.fw_uncached_generic.kva,
		hw_mgr->hfi_mem.fw_uncached_generic.len,
		hw_mgr->hfi_mem.qtbl.iova,
		hw_mgr->hfi_mem.qtbl.kva,
		hw_mgr->hfi_mem.qtbl.len,
		hw_mgr->hfi_mem.cmd_q.iova,
		hw_mgr->hfi_mem.cmd_q.kva,
		hw_mgr->hfi_mem.cmd_q.len,
		hw_mgr->hfi_mem.msg_q.iova,
		hw_mgr->hfi_mem.msg_q.kva,
		hw_mgr->hfi_mem.msg_q.len);

	CAM_DBG(CAM_ICP,
		"[%s] DbgQ[0x%x %p %lld] SFR[0x%x %p %lld] SecHeap[0x%x %p %lld]",
		hw_mgr->hw_mgr_name,
		hw_mgr->hfi_mem.dbg_q.iova,
		hw_mgr->hfi_mem.dbg_q.kva,
		hw_mgr->hfi_mem.dbg_q.len,
		hw_mgr->hfi_mem.sfr_buf.iova,
		hw_mgr->hfi_mem.sfr_buf.kva,
		hw_mgr->hfi_mem.sfr_buf.len,
		hw_mgr->hfi_mem.sec_heap.iova,
		hw_mgr->hfi_mem.sec_heap.kva,
		hw_mgr->hfi_mem.sec_heap.len);

	rc = cam_icp_get_io_mem_info(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Unable to get I/O region info rc %d",
			hw_mgr->hw_mgr_name, rc);
		if (fwuncached_region_exists) {
			cam_mem_mgr_free_memory_region(
				&hw_mgr->hfi_mem.fw_uncached_generic);
			goto qtbl_alloc_failed;
		} else {
			goto get_io_mem_failed;
		}
	}

	/* Allocate sync global mem & hwmutex for IPC */
	if (hw_mgr->synx_signaling_en) {
		rc = cam_icp_allocate_mem_for_fence_signaling(hw_mgr);
		if (rc) {
			if (fwuncached_region_exists) {
				cam_mem_mgr_free_memory_region(
					&hw_mgr->hfi_mem.fw_uncached_generic);
				goto qtbl_alloc_failed;
			} else {
				goto get_io_mem_failed;
			}
		}
	}

	return rc;
get_io_mem_failed:
	cam_mem_mgr_free_memory_region(&hw_mgr->hfi_mem.sec_heap);
sec_heap_alloc_failed:
	cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.sfr_buf);
sfr_buf_alloc_failed:
	cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.dbg_q);
dbg_q_alloc_failed:
	cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.msg_q);
msg_q_alloc_failed:
	cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.cmd_q);
cmd_q_alloc_failed:
	cam_mem_mgr_release_mem(&hw_mgr->hfi_mem.qtbl);
qtbl_alloc_failed:
	cam_smmu_unmap_phy_mem_region(hw_mgr->iommu_hdl, CAM_SMMU_REGION_QDSS, 0);
fw_alloc_failed:
	cam_icp_free_fw_mem(hw_mgr);
	return rc;
}

static int cam_icp_mgr_get_free_ctx(struct cam_icp_hw_mgr *hw_mgr)
{
	int i = 0;

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		mutex_lock(&hw_mgr->ctx_data[i].ctx_mutex);
		if (hw_mgr->ctx_data[i].state == CAM_ICP_CTX_STATE_FREE) {
			hw_mgr->ctx_data[i].state = CAM_ICP_CTX_STATE_IN_USE;
			mutex_unlock(&hw_mgr->ctx_data[i].ctx_mutex);
			break;
		}
		mutex_unlock(&hw_mgr->ctx_data[i].ctx_mutex);
	}

	return i;
}

static void cam_icp_mgr_put_ctx(struct cam_icp_hw_ctx_data *ctx_data)
{
	ctx_data->state = CAM_ICP_CTX_STATE_FREE;
}

static int cam_icp_mgr_send_pc_prep(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	unsigned long rem_jiffies;
	int timeout = 5000;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] device interface is NULL",
			hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	reinit_completion(&hw_mgr->icp_complete);

	CAM_DBG(CAM_ICP, "[%s] Sending pc prep command", hw_mgr->hw_mgr_name);
	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
		CAM_ICP_CMD_PC_PREP, NULL, 0);
	if (rc)
		return rc;

	CAM_DBG(CAM_ICP, "[%s] Wait for PC_PREP_DONE Message\n", hw_mgr->hw_mgr_name);
	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
		&hw_mgr->icp_complete,
		msecs_to_jiffies((timeout)), CAM_ICP,
		"[%s] FW response timeout for PC PREP handle command", hw_mgr->hw_mgr_name);
	if (!rem_jiffies)
		rc = -ETIMEDOUT;
	CAM_DBG(CAM_ICP, "[%s] Done Waiting for PC_PREP Message\n", hw_mgr->hw_mgr_name);

	return rc;
}

static int cam_icp_device_deint(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_icp_hw_device_info *dev_info = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	int rc = 0, i, j;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		dev_info = &hw_mgr->dev_info[i];
		for (j = 0; j < dev_info->hw_dev_cnt; j++) {
			if (dev_info->dev_ctx_info.dev_clk_state) {
				dev_intf = dev_info->dev_intf[j];
				if (!dev_intf) {
					CAM_ERR(CAM_ICP, "[%s] Device intf for %s[%u] is NULL",
						hw_mgr->hw_mgr_name, dev_info->dev_name, j);
					return -EINVAL;
				}
				rc = dev_intf->hw_ops.deinit(dev_intf->hw_priv, NULL, 0);
				if (rc) {
					CAM_ERR(CAM_ICP, "[%s] %s[%u] failed to deinit rc: %d",
						hw_mgr->hw_mgr_name, dev_info->dev_name, j, rc);
				}
				dev_info->dev_ctx_info.dev_clk_state = false;
			}
		}
	}

	return rc;
}

static int cam_icp_mgr_hw_close_u(void *hw_priv, void *hw_close_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;
	int rc = 0;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	CAM_DBG(CAM_ICP, "[%s] UMD calls close", hw_mgr->hw_mgr_name);

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	rc = cam_icp_mgr_hw_close(hw_mgr, NULL);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	return rc;
}

static int cam_icp_mgr_hw_close_k(void *hw_priv, void *hw_close_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	CAM_DBG(CAM_ICP, "[%s] KMD calls close", hw_mgr->hw_mgr_name);

	return cam_icp_mgr_hw_close(hw_mgr, NULL);

}

static int cam_icp_mgr_proc_resume(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	int rc;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return -EINVAL;
	}

	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv, CAM_ICP_CMD_POWER_RESUME,
		&hw_mgr->icp_jtag_debug, sizeof(hw_mgr->icp_jtag_debug));
	if (!rc)
		hw_mgr->icp_resumed = true;

	return rc;
}

static void cam_icp_mgr_proc_suspend(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	int rc;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "ICP device interface is NULL");
		return;
	}

	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv, CAM_ICP_CMD_POWER_COLLAPSE,
		NULL, 0);
	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Fail to suspend processor rc %d",
			hw_mgr->hw_mgr_name, rc);

	hw_mgr->icp_resumed = false;
}

static int __power_collapse(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;

	if (!hw_mgr->icp_pc_flag || atomic_read(&hw_mgr->recovery)) {
		cam_icp_mgr_proc_suspend(hw_mgr);

		rc = cam_icp_mgr_hw_close_k(hw_mgr, NULL);
		if (rc)
			CAM_ERR(CAM_ICP, "[%s] Failed in hw close rc %d",
				hw_mgr->hw_mgr_name, rc);
	} else {
		CAM_DBG(CAM_PERF, "[%s] Sending PC prep ICP PC enabled",
			hw_mgr->hw_mgr_name);

		rc = cam_icp_mgr_send_pc_prep(hw_mgr);
		if (rc)
			CAM_ERR(CAM_ICP, "[%s] Failed in send pc prep rc %d",
				hw_mgr->hw_mgr_name, rc);

		cam_icp_mgr_proc_suspend(hw_mgr);
	}

	return rc;
}

static int cam_icp_mgr_icp_power_collapse(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	int rc;
	bool send_freq_info = true;

	CAM_DBG(CAM_PERF, "[%s] ENTER", hw_mgr->hw_mgr_name);
	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	rc = __power_collapse(hw_mgr);
	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Fail to power collapse ICP rc: %d",
			hw_mgr->hw_mgr_name, rc);

	rc = icp_dev_intf->hw_ops.deinit(icp_dev_intf->hw_priv, &send_freq_info,
		sizeof(send_freq_info));
	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Fail to deinit ICP", hw_mgr->hw_mgr_name);

	CAM_DBG(CAM_PERF, "[%s] EXIT", hw_mgr->hw_mgr_name);

	return rc;
}

static int cam_icp_mgr_proc_boot(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	struct cam_icp_boot_args args;
	int rc;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] invalid device interface", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	if (!hw_mgr->icp_use_pil) {
		/* We handle the iommu mapping */
		args.firmware.iova = hw_mgr->hfi_mem.fw_buf.iova;
		args.firmware.kva = hw_mgr->hfi_mem.fw_buf.kva;
		args.firmware.len = hw_mgr->hfi_mem.fw_buf.len;
		args.use_sec_pil = false;
	} else {
		args.use_sec_pil = true;
	}

	args.irq_cb.data = hw_mgr;
	args.irq_cb.cb = cam_icp_hw_mgr_cb;

	args.debug_enabled = hw_mgr->icp_jtag_debug;
	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
		CAM_ICP_CMD_PROC_BOOT, &args, sizeof(args));
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] processor boot failed rc=%d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	hw_mgr->icp_resumed = true;

	return rc;
}

static void cam_icp_mgr_proc_shutdown(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	bool send_freq_info = false;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL",
			hw_mgr->hw_mgr_name);
		return;
	}

	icp_dev_intf->hw_ops.init(icp_dev_intf->hw_priv,
		&send_freq_info, sizeof(send_freq_info));

	icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
		CAM_ICP_CMD_PROC_SHUTDOWN, NULL, 0);

	icp_dev_intf->hw_ops.deinit(icp_dev_intf->hw_priv,
		&send_freq_info, sizeof(send_freq_info));

	if (hw_mgr->synx_signaling_en)
		cam_sync_synx_core_recovery(hw_mgr->synx_core_id);

	hw_mgr->icp_resumed = false;
}

static void cam_icp_mgr_populate_hfi_mem_info(struct cam_icp_hw_mgr *hw_mgr,
	struct hfi_mem_info *hfi_mem)
{
	hfi_mem->qtbl.kva  = hw_mgr->hfi_mem.qtbl.kva;
	hfi_mem->qtbl.iova = hw_mgr->hfi_mem.qtbl.iova;
	hfi_mem->qtbl.len  = hw_mgr->hfi_mem.qtbl.len;

	CAM_DBG(CAM_ICP, "[%s] qtbl kva = %llx IOVA = %x length = %lld\n",
		hw_mgr->hw_mgr_name, hfi_mem->qtbl.kva, hfi_mem->qtbl.iova, hfi_mem->qtbl.len);

	hfi_mem->cmd_q.kva  = hw_mgr->hfi_mem.cmd_q.kva;
	hfi_mem->cmd_q.iova = hw_mgr->hfi_mem.cmd_q.iova;
	hfi_mem->cmd_q.len  = hw_mgr->hfi_mem.cmd_q.len;
	CAM_DBG(CAM_ICP, "[%s] cmd_q kva = %llx IOVA = %x length = %lld\n",
		hw_mgr->hw_mgr_name, hfi_mem->cmd_q.kva, hfi_mem->cmd_q.iova, hfi_mem->cmd_q.len);

	hfi_mem->msg_q.kva  = hw_mgr->hfi_mem.msg_q.kva;
	hfi_mem->msg_q.iova = hw_mgr->hfi_mem.msg_q.iova;
	hfi_mem->msg_q.len  = hw_mgr->hfi_mem.msg_q.len;
	CAM_DBG(CAM_ICP, "[%s] msg_q kva = %llx IOVA = %x length = %lld\n",
		hw_mgr->hw_mgr_name, hfi_mem->msg_q.kva, hfi_mem->msg_q.iova, hfi_mem->msg_q.len);

	hfi_mem->dbg_q.kva  = hw_mgr->hfi_mem.dbg_q.kva;
	hfi_mem->dbg_q.iova = hw_mgr->hfi_mem.dbg_q.iova;
	hfi_mem->dbg_q.len  = hw_mgr->hfi_mem.dbg_q.len;
	CAM_DBG(CAM_ICP, "[%s] dbg_q kva = %llx IOVA = %x length = %lld\n",
		hw_mgr->hw_mgr_name, hfi_mem->dbg_q.kva, hfi_mem->dbg_q.iova, hfi_mem->dbg_q.len);

	hfi_mem->sfr_buf.kva  = hw_mgr->hfi_mem.sfr_buf.kva;
	hfi_mem->sfr_buf.iova = hw_mgr->hfi_mem.sfr_buf.iova;
	hfi_mem->sfr_buf.len  = hw_mgr->hfi_mem.sfr_buf.len;
	CAM_DBG(CAM_ICP, "[%s] sfr kva = %llx IOVA = %x length = %lld\n",
		hw_mgr->hw_mgr_name, hfi_mem->sfr_buf.kva, hfi_mem->sfr_buf.iova,
		hfi_mem->sfr_buf.len);

	hfi_mem->sec_heap.kva  = hw_mgr->hfi_mem.sec_heap.kva;
	hfi_mem->sec_heap.iova = hw_mgr->hfi_mem.sec_heap.iova;
	hfi_mem->sec_heap.len  = hw_mgr->hfi_mem.sec_heap.len;

	hfi_mem->shmem.iova = hw_mgr->hfi_mem.shmem.iova_start;
	hfi_mem->shmem.len  = hw_mgr->hfi_mem.shmem.iova_len;

	hfi_mem->qdss.iova = hw_mgr->hfi_mem.qdss_buf.iova;
	hfi_mem->qdss.len = hw_mgr->hfi_mem.qdss_buf.len;

	if (hw_mgr->synx_signaling_en) {
		hfi_mem->device_mem.iova = hw_mgr->hfi_mem.device.iova_start;
		hfi_mem->device_mem.len = hw_mgr->hfi_mem.device.iova_len;
		CAM_DBG(CAM_ICP,
			"device memory [iova = 0x%llx len = 0x%llx]",
			hfi_mem->device_mem.iova, hfi_mem->device_mem.len);
	}

	if (hw_mgr->hfi_mem.io_mem.discard_iova_start &&
		hw_mgr->hfi_mem.io_mem.discard_iova_len) {
		/* IO Region 1 */
		hfi_mem->io_mem.iova = hw_mgr->hfi_mem.io_mem.iova_start;
		hfi_mem->io_mem.len =
			hw_mgr->hfi_mem.io_mem.discard_iova_start -
			hw_mgr->hfi_mem.io_mem.iova_start;

		/* IO Region 2 */
		hfi_mem->io_mem2.iova =
			hw_mgr->hfi_mem.io_mem.discard_iova_start +
			hw_mgr->hfi_mem.io_mem.discard_iova_len;
		hfi_mem->io_mem2.len =
			hw_mgr->hfi_mem.io_mem.iova_start +
			hw_mgr->hfi_mem.io_mem.iova_len   -
			hfi_mem->io_mem2.iova;
	} else {
		/* IO Region 1 */
		hfi_mem->io_mem.iova = hw_mgr->hfi_mem.io_mem.iova_start;
		hfi_mem->io_mem.len = hw_mgr->hfi_mem.io_mem.iova_len;

		/* IO Region 2 */
		hfi_mem->io_mem2.iova = 0x0;
		hfi_mem->io_mem2.len = 0x0;
	}

	hfi_mem->fw_uncached.iova = hw_mgr->hfi_mem.fw_uncached.iova_start;
	hfi_mem->fw_uncached.len = hw_mgr->hfi_mem.fw_uncached.iova_len;

	CAM_DBG(CAM_ICP,
		"[%s] IO region1 IOVA = %X length = %lld, IO region2 IOVA = %X length = %lld",
		hw_mgr->hw_mgr_name,
		hfi_mem->io_mem.iova,
		hfi_mem->io_mem.len,
		hfi_mem->io_mem2.iova,
		hfi_mem->io_mem2.len);
}

static int cam_icp_mgr_hfi_resume(struct cam_icp_hw_mgr *hw_mgr)
{
	return cam_hfi_resume(hw_mgr->hfi_handle);
}

static int cam_icp_mgr_populate_abort_cmd(struct cam_icp_hw_ctx_data *ctx_data,
	struct hfi_cmd_dev_async **abort_cmd_ptr)
{
	struct hfi_cmd_dev_async *abort_cmd = NULL;
	uint32_t opcode, pkt_type;
	size_t packet_size;

	packet_size = sizeof(struct hfi_cmd_dev_async);

	switch (ctx_data->device_info->hw_dev_type) {
	case CAM_ICP_DEV_BPS:
		pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
		packet_size = packet_size + sizeof(struct hfi_cmd_abort) -
			sizeof(((struct hfi_cmd_dev_async *)0)->payload.direct);
		opcode = HFI_IPEBPS_CMD_OPCODE_BPS_ABORT;
		break;
	case CAM_ICP_DEV_IPE:
		pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
		packet_size = packet_size + sizeof(struct hfi_cmd_abort) -
			sizeof(((struct hfi_cmd_dev_async *)0)->payload.direct);
		opcode = HFI_IPEBPS_CMD_OPCODE_IPE_ABORT;
		break;
	case CAM_ICP_DEV_OFE:
		pkt_type = HFI_CMD_OFE_ASYNC_COMMAND;
		opcode = HFI_OFE_CMD_OPCODE_ABORT;
		break;
	default:
		CAM_ERR(CAM_ICP,
			"%s: Invalid device type not supported: %u",
			ctx_data->ctx_id_string, ctx_data->device_info->hw_dev_type);
		return -EINVAL;
	}

	abort_cmd = kzalloc(packet_size, GFP_KERNEL);
	if (!abort_cmd)
		return -ENOMEM;

	CAM_DBG(CAM_ICP, "%s: abort pkt size = %d",
		ctx_data->ctx_id_string, (int)packet_size);

	abort_cmd->size = packet_size;
	abort_cmd->pkt_type = pkt_type;
	abort_cmd->opcode = opcode;
	abort_cmd->num_fw_handles = 1;
	abort_cmd->fw_handles[0] = ctx_data->fw_handle;
	abort_cmd->user_data1 = PTR_TO_U64(ctx_data);
	abort_cmd->user_data2 = (uint64_t)0x0;

	*abort_cmd_ptr = abort_cmd;

	return 0;
}

static int cam_icp_mgr_abort_handle_wq(
	void *priv, void *data)
{
	int rc = 0;
	struct cam_icp_hw_mgr      *hw_mgr;
	struct hfi_cmd_work_data   *task_data = NULL;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct hfi_cmd_dev_async   *abort_cmd;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid params %pK %pK", data, priv);
		return -EINVAL;
	}

	task_data = (struct hfi_cmd_work_data *)data;
	ctx_data =
		(struct cam_icp_hw_ctx_data *)task_data->data;
	hw_mgr = ctx_data->hw_mgr_priv;

	rc = cam_icp_mgr_populate_abort_cmd(ctx_data, &abort_cmd);
	if (rc)
		return rc;

	rc = hfi_write_cmd(hw_mgr->hfi_handle, abort_cmd);
	if (rc) {
		kfree(abort_cmd);
		return rc;
	}
	CAM_DBG(CAM_ICP, "%s: fw_handle = 0x%x ctx_data = %pK",
		ctx_data->ctx_id_string, ctx_data->fw_handle, ctx_data);

	kfree(abort_cmd);
	return rc;
}

static int cam_icp_mgr_abort_handle(struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	unsigned long rem_jiffies = 0;
	int timeout = 2000;
	struct hfi_cmd_dev_async *abort_cmd;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;

	rc = cam_icp_mgr_populate_abort_cmd(ctx_data, &abort_cmd);
	if (rc)
		return rc;

	reinit_completion(&ctx_data->wait_complete);

	rc = hfi_write_cmd(hw_mgr->hfi_handle, abort_cmd);
	if (rc) {
		kfree(abort_cmd);
		return rc;
	}
	CAM_DBG(CAM_ICP, "%s: fw_handle = 0x%x ctx_data = %pK",
		ctx_data->ctx_id_string, ctx_data->fw_handle, ctx_data);
	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
			&ctx_data->wait_complete,
			msecs_to_jiffies(timeout), CAM_ICP,
			"%s: FW response timeout for Abort handle command",
			ctx_data->ctx_id_string);
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		cam_icp_dump_debug_info(ctx_data->hw_mgr_priv, false);
		ctx_data->abort_timed_out = true;
	}

	kfree(abort_cmd);
	return rc;
}

static int cam_icp_mgr_destroy_handle(
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0, opcode, pkt_type;
	int timeout = 1000;
	unsigned long rem_jiffies;
	size_t packet_size;
	struct hfi_cmd_dev_async *destroy_cmd;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;

	packet_size = sizeof(struct hfi_cmd_dev_async);

	switch (ctx_data->device_info->hw_dev_type) {
	case CAM_ICP_DEV_BPS:
		pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
		packet_size = packet_size + sizeof(struct hfi_cmd_destroy) -
			sizeof(((struct hfi_cmd_dev_async *)0)->payload.direct);
		opcode = HFI_IPEBPS_CMD_OPCODE_BPS_DESTROY;
		break;
	case CAM_ICP_DEV_IPE:
		pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
		packet_size = packet_size + sizeof(struct hfi_cmd_destroy) -
			sizeof(((struct hfi_cmd_dev_async *)0)->payload.direct);
		opcode = HFI_IPEBPS_CMD_OPCODE_IPE_DESTROY;
		break;
	case CAM_ICP_DEV_OFE:
		pkt_type = HFI_CMD_OFE_ASYNC_COMMAND;
		opcode = HFI_OFE_CMD_OPCODE_DESTROY;
		break;
	default:
		CAM_ERR(CAM_ICP, "%s: Invalid hw dev type not supported: %u",
			ctx_data->ctx_id_string, ctx_data->device_info->hw_dev_type);
		return -EINVAL;
	}

	destroy_cmd = kzalloc(packet_size, GFP_KERNEL);
	if (!destroy_cmd) {
		rc = -ENOMEM;
		return rc;
	}

	destroy_cmd->size = packet_size;
	destroy_cmd->pkt_type = pkt_type;
	destroy_cmd->opcode = opcode;
	destroy_cmd->num_fw_handles = 1;
	destroy_cmd->fw_handles[0] = ctx_data->fw_handle;
	destroy_cmd->user_data1 = PTR_TO_U64(ctx_data);
	destroy_cmd->user_data2 = (uint64_t)0x0;

	reinit_completion(&ctx_data->wait_complete);

	rc = hfi_write_cmd(hw_mgr->hfi_handle, destroy_cmd);
	if (rc) {
		kfree(destroy_cmd);
		return rc;
	}
	CAM_DBG(CAM_ICP, "%s: fw_handle = 0x%x ctx_data = %pK",
		ctx_data->ctx_id_string, ctx_data->fw_handle, ctx_data);
	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
			&ctx_data->wait_complete,
			msecs_to_jiffies(timeout), CAM_ICP,
			"%s: FW response timeout for Destroy handle command",
			ctx_data->ctx_id_string);
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		cam_icp_dump_debug_info(hw_mgr, ctx_data->abort_timed_out);
	}
	kfree(destroy_cmd);
	return rc;
}

static int cam_icp_mgr_release_ctx(struct cam_icp_hw_mgr *hw_mgr, int ctx_id)
{
	struct cam_icp_ctx_perf_stats *perf_stats;
	int i = 0;

	if (ctx_id >= CAM_ICP_CTX_MAX) {
		CAM_ERR(CAM_ICP, "[%s] ctx_id is wrong: %d",
			hw_mgr->hw_mgr_name, ctx_id);
		return -EINVAL;
	}

	mutex_lock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
	perf_stats = &hw_mgr->ctx_data[ctx_id].perf_stats;
	CAM_DBG(CAM_PERF,
		"Avg response time on %s: total_processed_requests: %llu avg_time: %llums",
		hw_mgr->ctx_data[ctx_id].ctx_id_string, perf_stats->total_requests,
		perf_stats->total_requests ?
		(perf_stats->total_resp_time / perf_stats->total_requests) : 0);

	memset(&hw_mgr->ctx_data[ctx_id].evt_inject_params, 0,
		sizeof(struct cam_hw_inject_evt_param));
	cam_icp_remove_ctx_bw(hw_mgr, &hw_mgr->ctx_data[ctx_id]);
	if (hw_mgr->ctx_data[ctx_id].state !=
		CAM_ICP_CTX_STATE_ACQUIRED) {
		mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
		CAM_DBG(CAM_ICP,
			"%s: Not in right state to release: %d",
			hw_mgr->ctx_data[ctx_id].ctx_id_string,
			hw_mgr->ctx_data[ctx_id].state);
		return 0;
	}
	cam_icp_mgr_dev_power_collapse(hw_mgr,
		&hw_mgr->ctx_data[ctx_id], 0);
	hw_mgr->ctx_data[ctx_id].state = CAM_ICP_CTX_STATE_RELEASE;
	CAM_DBG(CAM_ICP, "%s: E: recovery = %d",
		hw_mgr->ctx_data[ctx_id].ctx_id_string,
		atomic_read(&hw_mgr->recovery));

	if (!atomic_read(&hw_mgr->recovery)) {
		cam_icp_mgr_abort_handle(&hw_mgr->ctx_data[ctx_id]);
		cam_icp_mgr_destroy_handle(&hw_mgr->ctx_data[ctx_id]);
	}

	cam_icp_mgr_cleanup_ctx(&hw_mgr->ctx_data[ctx_id]);

	hw_mgr->ctx_data[ctx_id].fw_handle = 0;
	hw_mgr->ctx_data[ctx_id].scratch_mem_size = 0;
	hw_mgr->ctx_data[ctx_id].last_flush_req = 0;
	for (i = 0; i < CAM_FRAME_CMD_MAX; i++)
		clear_bit(i, hw_mgr->ctx_data[ctx_id].hfi_frame_process.bitmap);
	kfree(hw_mgr->ctx_data[ctx_id].hfi_frame_process.bitmap);
	hw_mgr->ctx_data[ctx_id].hfi_frame_process.bitmap = NULL;
	cam_icp_hw_mgr_clk_info_update(&hw_mgr->ctx_data[ctx_id]);
	hw_mgr->ctx_data[ctx_id].clk_info.curr_fc = 0;
	hw_mgr->ctx_data[ctx_id].clk_info.base_clk = 0;
	hw_mgr->ctxt_cnt--;
	kfree(hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info);
	hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info = NULL;
	hw_mgr->ctx_data[ctx_id].state = CAM_ICP_CTX_STATE_FREE;
	cam_icp_ctx_timer_stop(&hw_mgr->ctx_data[ctx_id]);
	hw_mgr->ctx_data[ctx_id].hw_mgr_priv = NULL;
	mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);

	CAM_DBG(CAM_ICP, "[%s] X: ctx_id = %d", hw_mgr->hw_mgr_name, ctx_id);
	return 0;
}

static unsigned long cam_icp_hw_mgr_mini_dump_cb(void *dst, unsigned long len,
	void *priv_data)
{
	struct cam_icp_hw_mini_dump_info   *md;
	struct cam_icp_hw_ctx_mini_dump    *ctx_md;
	struct cam_icp_hw_ctx_data         *ctx;
	struct cam_icp_hw_mgr              *hw_mgr = NULL;
	struct cam_hw_mini_dump_args        hw_dump_args;
	struct cam_icp_hw_dump_args         icp_dump_args;
	struct cam_hw_intf                 *icp_dev_intf = NULL;
	uint32_t                            i = 0, j = 0;
	unsigned long                       dumped_len = 0;
	unsigned long                       remain_len = len;
	int                                 rc = 0;
	uint32_t                            hw_mgr_idx;

	if (!dst || len < sizeof(*md)) {
		CAM_ERR(CAM_ICP, "Invalid params dst %pk len %lu", dst, len);
		return 0;
	}

	hw_mgr_idx = *((uint32_t *)priv_data);
	if (hw_mgr_idx >= CAM_ICP_SUBDEV_MAX) {
		CAM_ERR(CAM_ICP, "Invalid index to hw mgr: %u", hw_mgr_idx);
		return -EINVAL;
	}

	hw_mgr = g_icp_hw_mgr[hw_mgr_idx];
	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Uninitialized hw mgr for subdev: %u", hw_mgr_idx);
		return -EINVAL;
	}

	md = (struct cam_icp_hw_mini_dump_info *)dst;
	md->num_context = 0;
	cam_hfi_mini_dump(hw_mgr->hfi_handle, &md->hfi_info);
	memcpy(md->hw_mgr_name, hw_mgr->hw_mgr_name, strlen(md->hw_mgr_name));
	memcpy(&md->hfi_mem_info, &hw_mgr->hfi_mem,
		sizeof(struct icp_hfi_mem_info));
	memcpy(&md->dev_info, hw_mgr->dev_info,
		hw_mgr->num_dev_info * sizeof(struct cam_icp_hw_device_info));
	md->num_device_info = hw_mgr->num_dev_info;
	md->recovery = atomic_read(&hw_mgr->recovery);
	md->icp_booted = hw_mgr->icp_booted;
	md->icp_resumed = hw_mgr->icp_resumed;
	md->disable_ubwc_comp = hw_mgr->disable_ubwc_comp;
	md->icp_pc_flag = hw_mgr->icp_pc_flag;
	md->icp_use_pil = hw_mgr->icp_use_pil;

	dumped_len += sizeof(*md);
	remain_len = len -  dumped_len;
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		ctx = &hw_mgr->ctx_data[i];
		if (ctx->state == CAM_ICP_CTX_STATE_FREE ||
			ctx->state == CAM_ICP_CTX_STATE_RELEASE)
			continue;

		if (remain_len < sizeof(*ctx_md))
			goto end;

		md->num_context++;
		ctx_md = (struct cam_icp_hw_ctx_mini_dump *)
			    ((uint8_t *)dst + dumped_len);
		md->ctx[i] = ctx_md;
		ctx_md->state = ctx->state;
		ctx_md->ctx_id = ctx->ctx_id;
		memcpy(ctx_md->ctx_id_string, ctx->ctx_id_string,
			strlen(ctx->ctx_id_string));
		if (ctx->icp_dev_acquire_info) {
			ctx_md->acquire.secure_mode =
				ctx->icp_dev_acquire_info->secure_mode;
			ctx_md->acquire.dev_type =
				ctx->icp_dev_acquire_info->dev_type;
			ctx_md->acquire.num_out_res =
				ctx->icp_dev_acquire_info->num_out_res;
			memcpy(&ctx_md->acquire.in_res,
				&ctx->icp_dev_acquire_info->in_res,
				sizeof(struct cam_icp_res_info));
			memcpy(ctx_md->acquire.out_res,
				ctx->icp_dev_acquire_info->out_res,
				sizeof(ctx->icp_dev_acquire_info->out_res));
		} else {
			memset(&ctx_md->acquire, 0,
				sizeof(struct cam_icp_mini_dump_acquire_info));
		}

		memcpy(ctx_md->hfi_frame.request_id, ctx->hfi_frame_process.request_id,
			sizeof(ctx->hfi_frame_process.request_id));
		memcpy(ctx_md->hfi_frame.in_resource, ctx->hfi_frame_process.in_resource,
			sizeof(ctx->hfi_frame_process.in_resource));
		memcpy(ctx_md->hfi_frame.submit_timestamp,
			ctx->hfi_frame_process.submit_timestamp,
			sizeof(ctx->hfi_frame_process.submit_timestamp));
		memcpy(ctx_md->hfi_frame.num_out_res, ctx->hfi_frame_process.num_out_resources,
			sizeof(ctx->hfi_frame_process.num_out_resources));
		memcpy(ctx_md->hfi_frame.out_res, ctx->hfi_frame_process.out_resource,
			sizeof(ctx->hfi_frame_process.out_resource));
		for (j = 0; j < CAM_FRAME_CMD_MAX; j++) {
			ctx_md->hfi_frame.fw_process_flag[j] =
				ctx->hfi_frame_process.fw_process_flag[j];
		}

		dumped_len += sizeof(*ctx_md);
		remain_len = len - dumped_len;
		hw_dump_args.len = remain_len;
		hw_dump_args.bytes_written = 0;
		hw_dump_args.start_addr = (void *)((uint8_t *)dst + dumped_len);
		hw_mgr->mini_dump_cb(ctx->context_priv, &hw_dump_args);
		if (dumped_len + hw_dump_args.bytes_written >= len)
			goto end;

		ctx_md->hw_ctx = hw_dump_args.start_addr;
		dumped_len += hw_dump_args.bytes_written;
		remain_len = len - dumped_len;
	}

	/* Dump fw image */
	if (!hw_mgr->icp_use_pil) {
		icp_dump_args.cpu_addr = (uintptr_t)((uint8_t *)dst + dumped_len);
		icp_dump_args.offset = 0;
		icp_dump_args.buf_len = remain_len;
		icp_dev_intf = hw_mgr->icp_dev_intf;
		rc = icp_dev_intf->hw_ops.process_cmd(
			icp_dev_intf->hw_priv,
			CAM_ICP_CMD_HW_MINI_DUMP, &icp_dump_args,
			sizeof(struct cam_icp_hw_dump_args));
		if (rc)
			goto end;

		dumped_len += icp_dump_args.offset;
		md->fw_img = (void *)icp_dump_args.cpu_addr;
	}
end:
	return dumped_len;
}

static void cam_icp_mgr_device_deinit(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_icp_hw_device_info *dev_info = NULL;
	struct cam_hw_intf *dev_intf = NULL;
	int i, j;
	bool send_freq_info = false;

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		dev_info = &hw_mgr->dev_info[i];
		for (j = 0; j < dev_info->hw_dev_cnt; j++) {
			dev_intf = dev_info->dev_intf[j];
			if (!dev_intf) {
				CAM_ERR(CAM_ICP, "[%s] Device intf for %s[%u] is NULL",
					hw_mgr->hw_mgr_name, dev_info->dev_name, j);
				return;
			}
			dev_intf->hw_ops.deinit(dev_intf->hw_priv, NULL, 0);
			dev_info->dev_ctx_info.dev_clk_state = false;
		}
	}

	dev_intf = hw_mgr->icp_dev_intf;
	if (!dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL",
			hw_mgr->hw_mgr_name);
		return;
	}
	dev_intf->hw_ops.deinit(dev_intf->hw_priv, &send_freq_info, sizeof(send_freq_info));
}

static int cam_icp_mgr_hw_close(void *hw_priv, void *hw_close_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;
	int rc = 0;

	CAM_DBG(CAM_ICP, "[%s] Enter", hw_mgr->hw_mgr_name);
	atomic_set(&hw_mgr->recovery, 0);
	if (!hw_mgr->icp_booted) {
		CAM_DBG(CAM_ICP, "[%s] hw mgr is already closed", hw_mgr->hw_mgr_name);
		return 0;
	}

	if (!hw_mgr->icp_dev_intf) {
		CAM_DBG(CAM_ICP, "[%s] ICP device interface is NULL", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	cam_icp_mgr_proc_shutdown(hw_mgr);

	cam_hfi_deinit(hw_mgr->hfi_handle);
	cam_icp_free_hfi_mem(hw_mgr);

	hw_mgr->icp_booted = false;

	CAM_DBG(CAM_ICP, "[%s] Exit", hw_mgr->hw_mgr_name);
	return rc;
}

static int cam_icp_mgr_device_init(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0, i, j;
	struct cam_icp_hw_device_info *dev_info;
	struct cam_hw_intf *dev_intf = NULL;
	bool send_freq_info = false;

	dev_intf = hw_mgr->icp_dev_intf;
	if (!dev_intf) {
		CAM_ERR(CAM_ICP, "Invalid ICP device interface");
		return -EINVAL;
	}
	rc = dev_intf->hw_ops.init(dev_intf->hw_priv, &send_freq_info, sizeof(send_freq_info));
	if (rc) {
		CAM_ERR(CAM_ICP, "Failed in ICP init rc=%d", rc);
		return rc;
	}

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		dev_info = &hw_mgr->dev_info[i];
		for (j = 0; j < dev_info->hw_dev_cnt; j++) {
			dev_intf = dev_info->dev_intf[j];
			if (!dev_intf) {
				CAM_ERR(CAM_ICP, "Device intf for %s[%u] is NULL",
					dev_info->dev_name, j);
				rc = -EINVAL;
				goto hw_dev_deinit;
			}

			rc = dev_intf->hw_ops.init(dev_intf->hw_priv, NULL, 0);
			if (rc) {
				CAM_ERR(CAM_ICP, "[%s] Failed to init %s[%u]",
					hw_mgr->hw_mgr_name, dev_info->dev_name, j);
				goto hw_dev_deinit;
			}
			dev_info->dev_ctx_info.dev_clk_state = true;
		}
	}

	return rc;

hw_dev_deinit:
	for (; i >= 0; i--) {
		dev_info = &hw_mgr->dev_info[i];
		j = (j == -1) ? dev_info->hw_dev_cnt : (j - 1);
		for (; j >= 0; j--) {
			dev_intf = dev_info->dev_intf[j];
			dev_intf->hw_ops.deinit(dev_intf->hw_priv, NULL, 0);
		}
		dev_info->dev_ctx_info.dev_clk_state = false;
	}

	dev_intf = hw_mgr->icp_dev_intf;
	dev_intf->hw_ops.deinit(dev_intf->hw_priv, &send_freq_info, sizeof(send_freq_info));

	return rc;
}

static int cam_icp_mgr_hfi_init(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	struct hfi_mem_info hfi_mem = {0};
	const struct hfi_ops *hfi_ops;
	int rc;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is invalid", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	cam_icp_mgr_populate_hfi_mem_info(hw_mgr, &hfi_mem);
	rc = cam_icp_get_hfi_device_ops(icp_dev_intf->hw_type, &hfi_ops);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Fail to get HFI device ops rc: %d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	if (hw_mgr->synx_signaling_en) {
		/* Expect global sync to be at the start of FW uncached region */
		if (hw_mgr->hfi_mem.fw_uncached_global_sync.iova >=
			hw_mgr->hfi_mem.fw_uncached_generic.iova) {
			CAM_ERR(CAM_ICP,
				"global sync memory [start: 0x%x] expected to be at the start of FW uncached [uncached_generic start: 0x%x]",
				hw_mgr->hfi_mem.fw_uncached_global_sync.iova,
				hw_mgr->hfi_mem.fw_uncached_generic.iova);
			return -EINVAL;
		}

		/*
		 * Global sync memory is part of FW uncached region, but to FW remove this entry
		 * from FW uncached to avoid it being mapped with FW uncached. Global sync
		 * mem will be mapped with sharable attributes for IPC access, and hence
		 * an independent mapping of it's own.
		 */
		hfi_mem.fw_uncached.iova += hw_mgr->hfi_mem.fw_uncached_global_sync.len;
		hfi_mem.fw_uncached.len -= hw_mgr->hfi_mem.fw_uncached_global_sync.len;
	}

	rc = cam_hfi_init(hw_mgr->hfi_handle, &hfi_mem, hfi_ops,
		icp_dev_intf->hw_priv, 0);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed to init HFI rc=%d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	CAM_DBG(CAM_ICP, "[%s] hw mgr succeed hfi init with hfi handle: %d",
		hw_mgr->hw_mgr_name, hw_mgr->hfi_handle);

	return rc;
}

static int cam_icp_mgr_send_fw_init(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc;
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	unsigned long rem_jiffies;
	int timeout = 5000;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "%s: ICP device interface is NULL", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	reinit_completion(&hw_mgr->icp_complete);

	CAM_DBG(CAM_ICP, "[%s] Sending HFI init command", hw_mgr->hw_mgr_name);
	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv,
		CAM_ICP_SEND_INIT, NULL, 0);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in sending HFI init command rc %d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
		&hw_mgr->icp_complete,
		msecs_to_jiffies(timeout), CAM_ICP,
		"[%s] FW response timeout for FW init handle command",
		hw_mgr->hw_mgr_name);
	if (!rem_jiffies || atomic_read(&hw_mgr->recovery)) {
		rc = -ETIMEDOUT;
		cam_icp_dump_debug_info(hw_mgr, false);
	}
	CAM_DBG(CAM_ICP, "[%s] Done Waiting for INIT DONE Message",
		hw_mgr->hw_mgr_name);

	return rc;
}

static int cam_icp_mgr_send_memory_region_info(
	struct cam_icp_hw_mgr *hw_mgr)
{
	struct hfi_cmd_prop *set_prop = NULL;
	struct hfi_cmd_config_mem_regions *region_info = NULL;
	uint32_t num_regions = 0;
	size_t payload_size;

	if (hw_mgr->synx_signaling_en)
		num_regions += ICP_NUM_MEM_REGIONS_FOR_SYNX;

	if (!num_regions)
		return 0;

	payload_size = sizeof(struct hfi_cmd_prop) +
		(sizeof(struct hfi_cmd_config_mem_regions)) +
		(sizeof(struct hfi_cmd_mem_region_info) * (num_regions - 1));

	set_prop = kzalloc(payload_size, GFP_KERNEL);
	if (!set_prop)
		return -ENOMEM;

	set_prop->size = payload_size;
	set_prop->pkt_type = HFI_CMD_SYS_SET_PROPERTY;
	set_prop->num_prop = 1;
	set_prop->prop_data[0] = HFI_PROP_SYS_MEM_REGIONS;

	region_info = (struct hfi_cmd_config_mem_regions *)&set_prop->prop_data[1];
	if (hw_mgr->synx_signaling_en) {
		/* Update synx global mem */
		region_info->region_info[region_info->num_valid_regions].region_id =
			HFI_MEM_REGION_ID_IPCLITE_SHARED_MEM;
		region_info->region_info[region_info->num_valid_regions].region_type =
			HFI_MEM_REGION_TYPE_UNCACHED;
		region_info->region_info[region_info->num_valid_regions].start_addr =
			hw_mgr->hfi_mem.fw_uncached_global_sync.iova;
		region_info->region_info[region_info->num_valid_regions].size =
			hw_mgr->hfi_mem.fw_uncached_global_sync.len;

		region_info->num_valid_regions++;

		/* Update synx hw_mutex mem */
		region_info->region_info[region_info->num_valid_regions].region_id =
			HFI_MEM_REGION_ID_SYNX_HW_MUTEX;
		region_info->region_info[region_info->num_valid_regions].region_type =
			HFI_MEM_REGION_TYPE_DEVICE;
		region_info->region_info[region_info->num_valid_regions].start_addr =
			hw_mgr->hfi_mem.synx_hwmutex.iova;
		region_info->region_info[region_info->num_valid_regions].size =
			hw_mgr->hfi_mem.synx_hwmutex.len;

		region_info->num_valid_regions++;

		/* Update ipc hw_mutex mem */
		region_info->region_info[region_info->num_valid_regions].region_id =
			HFI_MEM_REGION_ID_GLOBAL_ATOMIC_HW_MUTEX;
		region_info->region_info[region_info->num_valid_regions].region_type =
			HFI_MEM_REGION_TYPE_DEVICE;
		region_info->region_info[region_info->num_valid_regions].start_addr =
			hw_mgr->hfi_mem.ipc_hwmutex.iova;
		region_info->region_info[region_info->num_valid_regions].size =
			hw_mgr->hfi_mem.ipc_hwmutex.len;

		region_info->num_valid_regions++;

		/* Update global cntr mem */
		region_info->region_info[region_info->num_valid_regions].region_id =
			HFI_MEM_REGION_ID_GLOBAL_CNTR;
		region_info->region_info[region_info->num_valid_regions].region_type =
			HFI_MEM_REGION_TYPE_DEVICE;
		region_info->region_info[region_info->num_valid_regions].start_addr =
			hw_mgr->hfi_mem.global_cntr.iova;
		region_info->region_info[region_info->num_valid_regions].size =
			hw_mgr->hfi_mem.global_cntr.len;

		region_info->num_valid_regions++;
		CAM_DBG(CAM_ICP,
			"Synx mem regions global_sync[0x%x:0x%x] synx_hw_mutex[0x%x:0x%x] ipc_hw_mutex[0x%x:0x%x] global_cntr[0x%x:0x%x]",
			hw_mgr->hfi_mem.fw_uncached_global_sync.iova,
			hw_mgr->hfi_mem.fw_uncached_global_sync.len,
			hw_mgr->hfi_mem.synx_hwmutex.iova, hw_mgr->hfi_mem.synx_hwmutex.len,
			hw_mgr->hfi_mem.ipc_hwmutex.iova, hw_mgr->hfi_mem.ipc_hwmutex.len,
			hw_mgr->hfi_mem.global_cntr.iova, hw_mgr->hfi_mem.global_cntr.len);
	}

	CAM_DBG(CAM_ICP,
		"Mem region property payload size: %zu num_regions: %u",
		payload_size, region_info->num_valid_regions);

	hfi_write_cmd(hw_mgr->hfi_handle, set_prop);
	kfree(set_prop);

	return 0;
}

static int cam_icp_mgr_hw_open_u(void *hw_mgr_priv, void *download_fw_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	int rc = 0;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	if (cam_presil_mode_enabled()) {
		CAM_DBG(CAM_PRESIL, "[%s] hw_open from umd skipped for presil",
			hw_mgr->hw_mgr_name);
		return 0;
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	rc = cam_icp_mgr_hw_open(hw_mgr, download_fw_args);
	if (rc) {
		CAM_WARN(CAM_ICP, "Retry ICP initialization");
		rc = cam_icp_mgr_hw_close(hw_mgr, NULL);
		if (rc) {
			CAM_ERR(CAM_ICP, "cam_icp_mgr_hw_close() rc:%d", rc);
		} else {
			rc = cam_icp_mgr_hw_open(hw_mgr, download_fw_args);
			if (rc) {
				CAM_ERR(CAM_ICP, "cam_icp_mgr_hw_open() rc:%d", rc);
			}
		}
	}
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	return rc;
}

static int cam_icp_mgr_hw_open_k(void *hw_mgr_priv, void *download_fw_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	int rc;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Null hw mgr");
		return 0;
	}

	rc = cam_icp_mgr_hw_open(hw_mgr, download_fw_args);
	CAM_DBG(CAM_ICP, "[%s] hw_open from kmd done %d",
		hw_mgr->hw_mgr_name, rc);

	return rc;
}

static int cam_icp_mgr_icp_resume(struct cam_icp_hw_mgr *hw_mgr)
{
	int rc = 0;
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	bool downloadFromResume = true, send_freq_info;

	CAM_DBG(CAM_ICP, "[%s] Enter", hw_mgr->hw_mgr_name);

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	if (!hw_mgr->icp_booted) {
		CAM_DBG(CAM_ICP, "[%s] booting ICP processor", hw_mgr->hw_mgr_name);
		return cam_icp_mgr_hw_open_k(hw_mgr, &downloadFromResume);
	}

	send_freq_info = true;
	rc = icp_dev_intf->hw_ops.init(icp_dev_intf->hw_priv, &send_freq_info,
		sizeof(send_freq_info));
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed to init ICP hw rc: %d", hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	rc = cam_icp_mgr_proc_resume(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed to resume proc rc: %d", hw_mgr->hw_mgr_name, rc);
		goto hw_deinit;
	}

	rc = cam_icp_mgr_hfi_resume(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed to resume HFI rc: %d", hw_mgr->hw_mgr_name, rc);
		goto power_collapse;
	}

	CAM_DBG(CAM_ICP, "[%s] Exit", hw_mgr->hw_mgr_name);
	return rc;

power_collapse:
	__power_collapse(hw_mgr);
hw_deinit:
	send_freq_info = false;
	icp_dev_intf->hw_ops.deinit(icp_dev_intf->hw_priv, &send_freq_info,
		sizeof(send_freq_info));

	return rc;
}

static int cam_icp_mgr_hw_open(void *hw_mgr_priv, void *download_fw_args)
{
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	bool icp_pc = false;
	uint32_t dump_type;
	int rc = 0;

	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "hw_mgr is NULL");
		return -EINVAL;
	}

	if (hw_mgr->icp_booted) {
		CAM_DBG(CAM_ICP, "[%s] ICP already booted", hw_mgr->hw_mgr_name);
		return rc;
	}

	if (!hw_mgr->icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is invalid",
			hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "[%s] Start icp hw open", hw_mgr->hw_mgr_name);

	rc = cam_icp_allocate_hfi_mem(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in alloc hfi mem, rc %d",
			hw_mgr->hw_mgr_name, rc);
		goto alloc_hfi_mem_failed;
	}

	rc = cam_icp_mgr_device_init(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in device init, rc %d",
			hw_mgr->hw_mgr_name, rc);
		goto dev_init_fail;
	}

	rc = cam_icp_mgr_proc_boot(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in proc boot, rc %d",
			hw_mgr->hw_mgr_name, rc);
		goto boot_failed;
	}

	rc = cam_icp_mgr_hfi_init(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in hfi init, rc %d",
			hw_mgr->hw_mgr_name, rc);
		dump_type = (CAM_ICP_DUMP_STATUS_REGISTERS | CAM_ICP_DUMP_CSR_REGISTERS);
		hw_mgr->icp_dev_intf->hw_ops.process_cmd(hw_mgr->icp_dev_intf->hw_priv,
			CAM_ICP_CMD_HW_REG_DUMP, &dump_type, sizeof(dump_type));
		goto hfi_init_failed;
	}

	rc = cam_icp_mgr_send_fw_init(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in sending fw init, rc %d",
			hw_mgr->hw_mgr_name, rc);
		goto fw_init_failed;
	}

	rc = cam_icp_mgr_send_memory_region_info(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed in sending mem region info, rc %d",
			hw_mgr->hw_mgr_name, rc);
		goto fw_init_failed;
	}

	hw_mgr->ctxt_cnt = 0;
	hw_mgr->icp_booted = true;
	atomic_set(&hw_mgr->recovery, 0);

	CAM_INFO(CAM_ICP, "[%s] FW download done successfully", hw_mgr->hw_mgr_name);

	rc = cam_icp_device_deint(hw_mgr);
	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Failed in ipe bps deinit rc %d",
			hw_mgr->hw_mgr_name, rc);

	if (download_fw_args)
		icp_pc = *((bool *)download_fw_args);

	if (icp_pc && hw_mgr->icp_pc_flag) {
		rc = cam_icp_device_deint(hw_mgr);
		if (rc)
			CAM_ERR(CAM_ICP, "[%s] Failed in ipe bps deinit with icp_pc rc %d",
				hw_mgr->hw_mgr_name, rc);

		CAM_DBG(CAM_ICP, "[%s] deinit all clocks", hw_mgr->hw_mgr_name);
	}

	if (icp_pc)
		return rc;

	rc = cam_icp_mgr_icp_power_collapse(hw_mgr);
	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Failed in icp power collapse rc %d",
			hw_mgr->hw_mgr_name, rc);

	CAM_DBG(CAM_ICP, "[%s] deinit all clocks at boot up", hw_mgr->hw_mgr_name);

	return rc;

fw_init_failed:
	cam_hfi_deinit(hw_mgr->hfi_handle);
hfi_init_failed:
	cam_icp_mgr_proc_shutdown(hw_mgr);
boot_failed:
	cam_icp_mgr_device_deinit(hw_mgr);
dev_init_fail:
	cam_icp_free_hfi_mem(hw_mgr);
alloc_hfi_mem_failed:
	return rc;
}

static int cam_icp_mgr_handle_config_err(
	struct cam_hw_config_args *config_args,
	struct cam_icp_hw_ctx_data *ctx_data,
	int idx)
{
	struct cam_icp_hw_buf_done_evt_data icp_evt_data;
	struct cam_hw_done_event_data buf_data = {0};

	buf_data.request_id = *(uint64_t *)config_args->priv;
	buf_data.evt_param = CAM_SYNC_ICP_EVENT_CONFIG_ERR;
	icp_evt_data.evt_id = CAM_CTX_EVT_ID_ERROR;
	icp_evt_data.buf_done_data = &buf_data;
	ctx_data->ctxt_event_cb(ctx_data->context_priv, CAM_ICP_EVT_ID_BUF_DONE,
		&icp_evt_data);

	ctx_data->hfi_frame_process.request_id[idx] = 0;
	ctx_data->hfi_frame_process.fw_process_flag[idx] = false;
	clear_bit(idx, ctx_data->hfi_frame_process.bitmap);

	return 0;
}

static int cam_icp_mgr_enqueue_config(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_hw_config_args *config_args)
{
	int rc = 0;
	uint64_t request_id = 0;
	struct crm_workq_task *task;
	struct hfi_cmd_work_data *task_data;
	struct cam_hw_update_entry *hw_update_entries;
	struct icp_frame_info *frame_info = NULL;

	frame_info = (struct icp_frame_info *)config_args->priv;
	request_id = frame_info->request_id;
	hw_update_entries = config_args->hw_update_entries;
	CAM_DBG(CAM_ICP, "[%s] req_id = %lld %pK",
		hw_mgr->hw_mgr_name, request_id, config_args->priv);

	task = cam_req_mgr_workq_get_task(hw_mgr->cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "[%s] no empty task", hw_mgr->hw_mgr_name);
		return -ENOMEM;
	}

	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)hw_update_entries->addr;
	task_data->request_id = request_id;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	rc = cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);

	return rc;
}

static int cam_icp_mgr_send_config_io(struct cam_icp_hw_ctx_data *ctx_data,
	uint32_t io_buf_addr)
{
	int rc = 0;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;
	struct hfi_cmd_work_data *task_data;
	struct hfi_cmd_dev_async ioconfig_cmd;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct crm_workq_task *task;
	uint32_t size_in_words;

	task = cam_req_mgr_workq_get_task(hw_mgr->cmd_work);
	if (!task) {
		CAM_ERR_RATE_LIMIT(CAM_ICP, "%s: No free cmd task", ctx_data->ctx_id_string);
		return -ENOMEM;
	}

	switch (ctx_data->device_info->hw_dev_type) {
	case CAM_ICP_DEV_BPS:
		ioconfig_cmd.opcode = HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO;
		ioconfig_cmd.pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
		break;
	case CAM_ICP_DEV_IPE:
		ioconfig_cmd.opcode = HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO;
		ioconfig_cmd.pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
		break;
	case CAM_ICP_DEV_OFE:
		ioconfig_cmd.opcode = HFI_OFE_CMD_OPCODE_CONFIG_IO;
		ioconfig_cmd.pkt_type = HFI_CMD_OFE_ASYNC_COMMAND;
		break;
	default:
		CAM_ERR(CAM_ICP, "%s Invalid hw dev type not supported: %u",
			ctx_data->ctx_id_string, ctx_data->device_info->hw_dev_type);
		return -EINVAL;
	}

	reinit_completion(&ctx_data->wait_complete);

	ioconfig_cmd.size = sizeof(struct hfi_cmd_dev_async);
	ioconfig_cmd.num_fw_handles = 1;
	ioconfig_cmd.fw_handles[0] = ctx_data->fw_handle;
	ioconfig_cmd.payload.indirect = io_buf_addr;
	ioconfig_cmd.user_data1 = PTR_TO_U64(ctx_data);
	ioconfig_cmd.user_data2 = (uint64_t)0x0;
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)&ioconfig_cmd;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_MSG_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	size_in_words = (*(uint32_t *)task_data->data) >> 2;
	CAM_DBG(CAM_ICP, "%s: size_in_words %u",
		ctx_data->ctx_id_string, size_in_words);
	rc = cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc) {
		CAM_ERR_RATE_LIMIT(CAM_ICP, "%s: Failed to enqueue io config task",
			ctx_data->ctx_id_string);
		return rc;
	}

	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
			&ctx_data->wait_complete,
			msecs_to_jiffies(timeout), CAM_ICP,
			"%s: FW response timeout for send IO cfg handle command on",
			ctx_data->ctx_id_string);
	if (!rem_jiffies) {
		/* send specific error for io config failure */
		rc = -EREMOTEIO;
		cam_icp_dump_debug_info(hw_mgr, false);
	}

	return rc;
}

static int cam_icp_mgr_send_recfg_io(struct cam_icp_hw_ctx_data *ctx_data,
	struct hfi_cmd_dev_async *ioconfig_cmd, uint64_t req_id)
{
	int rc = 0;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;
	struct hfi_cmd_work_data *task_data;
	struct crm_workq_task *task;

	task = cam_req_mgr_workq_get_task(hw_mgr->cmd_work);
	if (!task)
		return -ENOMEM;

	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)ioconfig_cmd;
	task_data->request_id = req_id;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	rc = cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc)
		return rc;

	return rc;
}

static int cam_icp_mgr_config_hw(void *hw_mgr_priv, void *config_hw_args)
{
	int rc = 0;
	int idx;
	uint64_t req_id;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_hw_config_args *config_args = config_hw_args;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct icp_frame_info *frame_info = NULL;

	if (!hw_mgr || !config_args) {
		CAM_ERR(CAM_ICP, "Invalid arguments %pK %pK",
			hw_mgr, config_args);
		return -EINVAL;
	}

	ctx_data = config_args->ctxt_to_hw_map;

	if (!config_args->num_hw_update_entries) {
		CAM_ERR(CAM_ICP, "%s: No hw update enteries are available",
			ctx_data->ctx_id_string);
		return -EINVAL;
	}

	if (cam_presil_mode_enabled()) {
		CAM_DBG(CAM_PRESIL, "%s: presil: locking frame_in_process %d req id %u",
			ctx_data->ctx_id_string, atomic_read(&hw_mgr->frame_in_process),
			config_args->request_id);
		down_write(&frame_in_process_sem);
		atomic_set(&hw_mgr->frame_in_process, 1);
		hw_mgr->frame_in_process_ctx_id = ctx_data->ctx_id;
		CAM_DBG(CAM_PRESIL, "%s: presil: locked frame_in_process req id %u ctx_id %d",
			ctx_data->ctx_id_string, config_args->request_id,
			hw_mgr->frame_in_process_ctx_id);
		msleep(100);
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	mutex_lock(&ctx_data->ctx_mutex);
	if (ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) {
		mutex_unlock(&ctx_data->ctx_mutex);
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
		CAM_ERR(CAM_ICP, "%s: is not in use", ctx_data->ctx_id_string);
		return -EINVAL;
	}

	frame_info = (struct icp_frame_info *)config_args->priv;
	req_id = frame_info->request_id;
	idx = cam_icp_get_frame_process_idx_from_req_id(ctx_data, req_id);
	if (idx >= CAM_FRAME_CMD_MAX) {
		CAM_ERR(CAM_ICP, "%s: frame process index not found for req_id: %llu",
			ctx_data->ctx_id_string, req_id);
		mutex_unlock(&ctx_data->ctx_mutex);
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
		return -EINVAL;
	}

	if (cam_presil_mode_enabled()) {
		CAM_INFO(CAM_ICP, "%s: Sending relevant buffers for request: %llu to presil",
			ctx_data->ctx_id_string, config_args->request_id);
		cam_presil_send_buffers_from_packet(frame_info->pkt, hw_mgr->iommu_hdl,
			hw_mgr->iommu_hdl);
	}

	cam_icp_mgr_dev_clk_update(hw_mgr, ctx_data, idx);
	ctx_data->hfi_frame_process.fw_process_flag[idx] = true;
	ctx_data->hfi_frame_process.submit_timestamp[idx] = ktime_get();

	CAM_DBG(CAM_ICP, "%s: req_id %llu, io config %llu",
		ctx_data->ctx_id_string, req_id, frame_info->io_config);

	if (frame_info->io_config != 0) {
		CAM_INFO(CAM_ICP, "%s: Send recfg io", ctx_data->ctx_id_string);
		rc = cam_icp_mgr_send_recfg_io(ctx_data,
			&frame_info->hfi_cfg_io_cmd, req_id);
		if (rc)
			CAM_ERR(CAM_ICP, "%s: Fail to send reconfig io cmd",
				ctx_data->ctx_id_string);
	}

	if (req_id <= ctx_data->last_flush_req)
		CAM_WARN(CAM_ICP,
			"%s: Anomaly submitting flushed req %llu [last_flush %llu]",
			ctx_data->ctx_id_string, req_id, ctx_data->last_flush_req);

	cam_cpas_notify_event(ctx_data->ctx_id_string, req_id);

	rc = cam_icp_mgr_enqueue_config(hw_mgr, config_args);
	if (rc)
		goto config_err;

	CAM_DBG(CAM_REQ, "%s: req_id = %lld queued to FW",
		ctx_data->ctx_id_string, req_id);
	mutex_unlock(&ctx_data->ctx_mutex);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	return 0;
config_err:
	cam_icp_mgr_handle_config_err(config_args, ctx_data, idx);
	mutex_unlock(&ctx_data->ctx_mutex);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_mgr_prepare_frame_process_cmd(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct hfi_cmd_dev_async *hfi_cmd,
	uint64_t request_id,
	uint32_t fw_cmd_buf_iova_addr)
{
	switch (ctx_data->device_info->hw_dev_type) {
	case CAM_ICP_DEV_BPS:
		hfi_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_BPS_FRAME_PROCESS;
		hfi_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
		break;
	case CAM_ICP_DEV_IPE:
		hfi_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_IPE_FRAME_PROCESS;
		hfi_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
		break;
	case CAM_ICP_DEV_OFE:
		hfi_cmd->opcode = HFI_OFE_CMD_OPCODE_FRAME_PROCESS;
		hfi_cmd->pkt_type = HFI_CMD_OFE_ASYNC_COMMAND;
		break;
	default:
		CAM_ERR(CAM_ICP, "%s: Invalid hw dev type not supported: %u",
			ctx_data->ctx_id_string, ctx_data->device_info->hw_dev_type);
		return -EINVAL;
	}

	hfi_cmd->size = sizeof(struct hfi_cmd_dev_async);
	hfi_cmd->num_fw_handles = 1;
	hfi_cmd->fw_handles[0] = ctx_data->fw_handle;
	hfi_cmd->payload.indirect = fw_cmd_buf_iova_addr;
	hfi_cmd->user_data1 = PTR_TO_U64(ctx_data);
	hfi_cmd->user_data2 = request_id;

	CAM_DBG(CAM_ICP, "%s: ctx_data : %pK, request_id :%lld cmd_buf %x",
		ctx_data->ctx_id_string, (void *)ctx_data->context_priv, request_id,
		fw_cmd_buf_iova_addr);

	return 0;
}

static bool cam_icp_mgr_is_valid_inconfig(struct cam_packet *packet)
{
	int i, num_in_map_entries = 0;
	bool in_config_valid = false;
	struct cam_buf_io_cfg *io_cfg_ptr = NULL;

	io_cfg_ptr = (struct cam_buf_io_cfg *) ((uint32_t *) &packet->payload +
		packet->io_configs_offset/4);

	for (i = 0 ; i < packet->num_io_configs; i++)
		if (io_cfg_ptr[i].direction == CAM_BUF_INPUT)
			num_in_map_entries++;

	if (num_in_map_entries <= CAM_MAX_IN_RES)
		in_config_valid = true;
	else
		CAM_ERR(CAM_ICP, "In config entries(%u) more than allowed(%u)",
			num_in_map_entries, CAM_MAX_IN_RES);

	CAM_DBG(CAM_ICP,
		"number of io config: %u in config: %u max in res: %u",
		packet->num_io_configs,
		num_in_map_entries, CAM_MAX_IN_RES);

	return in_config_valid;
}

static bool cam_icp_mgr_is_valid_outconfig(struct cam_packet *packet)
{
	int i, num_out_map_entries = 0;
	bool out_config_valid = false;
	struct cam_buf_io_cfg *io_cfg_ptr = NULL;

	io_cfg_ptr = (struct cam_buf_io_cfg *) ((uint32_t *) &packet->payload +
		packet->io_configs_offset/4);

	for (i = 0 ; i < packet->num_io_configs; i++)
		if (io_cfg_ptr[i].direction == CAM_BUF_OUTPUT)
			num_out_map_entries++;

	if (num_out_map_entries <= CAM_MAX_OUT_RES) {
		out_config_valid = true;
	} else {
		CAM_ERR(CAM_ICP, "Out config entries(%u) more than allowed(%u)",
			num_out_map_entries, CAM_MAX_OUT_RES);
	}

	CAM_DBG(CAM_ICP,
		"number of io config: %u out config: %u max out res: %u",
		packet->num_io_configs,
		num_out_map_entries, CAM_MAX_OUT_RES);

	return out_config_valid;
}

static int cam_icp_mgr_pkt_validation(struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_packet *packet)
{
	uint32_t op_code;
	enum cam_icp_hw_type hw_dev_type;

	op_code = packet->header.op_code & 0xff;
	hw_dev_type = ctx_data->device_info->hw_dev_type;

	if (((op_code != CAM_ICP_OPCODE_IPE_UPDATE) &&
		hw_dev_type == CAM_ICP_DEV_IPE) ||
		((op_code != CAM_ICP_OPCODE_BPS_UPDATE) &&
		hw_dev_type == CAM_ICP_DEV_BPS) ||
		((op_code != CAM_ICP_OPCODE_OFE_UPDATE) &&
		hw_dev_type == CAM_ICP_DEV_OFE)) {
		CAM_ERR(CAM_ICP, "%s: Invalid Opcode in pkt: %d device: %s",
			ctx_data->ctx_id_string, op_code, ctx_data->device_info->dev_name);
		return -EINVAL;
	}

	if (!packet->num_io_configs) {
		CAM_ERR(CAM_ICP, "%s: Invalid number of io configs: %d",
			ctx_data->ctx_id_string, packet->num_io_configs);
		return -EINVAL;
	}

	if (!packet->num_cmd_buf || packet->num_cmd_buf > CAM_ICP_CTX_MAX_CMD_BUFFERS) {
		CAM_ERR(CAM_ICP, "%s: Invalid number of cmd buffers: %d max cmd buf: %d",
			ctx_data->ctx_id_string, packet->num_cmd_buf,
			CAM_ICP_CTX_MAX_CMD_BUFFERS);
		return -EINVAL;
	}

	if (!cam_icp_mgr_is_valid_inconfig(packet) ||
		!cam_icp_mgr_is_valid_outconfig(packet)) {
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "%s: number of cmd/patch info: %u %u %u",
		ctx_data->ctx_id_string, packet->num_cmd_buf, packet->num_io_configs,
		packet->num_patches);

	return 0;
}

static int cam_icp_mgr_process_cmd_desc(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_packet *packet, struct cam_icp_hw_ctx_data *ctx_data,
	uint32_t *fw_cmd_buf_iova_addr, struct list_head *buf_tracker)
{
	int rc = 0;
	int i;
	dma_addr_t addr;
	size_t len;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	uintptr_t cpu_addr = 0;

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *) &packet->payload + packet->cmd_buf_offset/4);
	rc = cam_packet_util_validate_cmd_desc(cmd_desc);
	if (rc)
		return rc;

	*fw_cmd_buf_iova_addr = 0;
	for (i = 0; i < packet->num_cmd_buf; i++) {
		if (cmd_desc[i].type == CAM_CMD_BUF_FW) {
			rc = cam_mem_get_io_buf(cmd_desc[i].mem_handle,
				hw_mgr->iommu_hdl, &addr, &len, NULL, buf_tracker);
			if (rc) {
				CAM_ERR(CAM_ICP, "%s: get cmd buf failed %x",
					ctx_data->ctx_id_string, hw_mgr->iommu_hdl);
				return rc;
			}

			/* FW buffers are expected to be within 32-bit address range */
			*fw_cmd_buf_iova_addr = addr;

			if ((cmd_desc[i].offset >= len) ||
				((len - cmd_desc[i].offset) <
				cmd_desc[i].size)) {
				CAM_ERR(CAM_ICP,
					"%s: Invalid offset, i: %d offset: %u len: %zu size: %zu",
					ctx_data->ctx_id_string, i, cmd_desc[i].offset,
					len, cmd_desc[i].size);
				return -EINVAL;
			}

			*fw_cmd_buf_iova_addr =
				(*fw_cmd_buf_iova_addr + cmd_desc[i].offset);
			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&cpu_addr, &len);
			if (rc || !cpu_addr) {
				CAM_ERR(CAM_ICP, "%s: get cmd buf failed %x",
					ctx_data->ctx_id_string, hw_mgr->iommu_hdl);
				*fw_cmd_buf_iova_addr = 0;
				return rc;
			}
			if ((len <= cmd_desc[i].offset) ||
				(cmd_desc[i].size < cmd_desc[i].length) ||
				((len - cmd_desc[i].offset) <
				cmd_desc[i].length)) {
				CAM_ERR(CAM_ICP, "%s: Invalid offset or length",
					ctx_data->ctx_id_string);
				cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
				return -EINVAL;
			}
			cpu_addr = cpu_addr + cmd_desc[i].offset;

			cam_mem_put_cpu_buf(cmd_desc[i].mem_handle);
		}
	}

	if (!cpu_addr) {
		CAM_ERR(CAM_ICP, "%s: invalid number of cmd buf", ctx_data->ctx_id_string);
		return -EINVAL;
	}

	return rc;
}

static int cam_icp_mgr_process_io_cfg(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_packet *packet,
	struct cam_hw_prepare_update_args *prepare_args,
	int32_t index)
{
	int i, j, k, rc = 0;
	struct cam_buf_io_cfg *io_cfg_ptr = NULL;
	int32_t sync_in_obj[CAM_MAX_IN_RES];
	int32_t merged_sync_in_obj;

	io_cfg_ptr = (struct cam_buf_io_cfg *) ((uint32_t *) &packet->payload +
		packet->io_configs_offset/4);
	prepare_args->num_out_map_entries = 0;
	prepare_args->num_in_map_entries = 0;

	for (i = 0, j = 0, k = 0; i < packet->num_io_configs; i++) {
		if (io_cfg_ptr[i].direction == CAM_BUF_INPUT) {
			sync_in_obj[j++] = io_cfg_ptr[i].fence;
			prepare_args->num_in_map_entries++;
		} else {
			prepare_args->out_map_entries[k].sync_id =
				io_cfg_ptr[i].fence;
			prepare_args->out_map_entries[k].resource_handle =
				io_cfg_ptr[i].resource_type;
			k++;
			prepare_args->num_out_map_entries++;
		}

		CAM_DBG(CAM_REQ,
			"%s: req_id: %llu dir[%d]: %u, fence: %u resource_type = %u memh %x",
			ctx_data->ctx_id_string, packet->header.request_id, i,
			io_cfg_ptr[i].direction, io_cfg_ptr[i].fence,
			io_cfg_ptr[i].resource_type,
			io_cfg_ptr[i].mem_handle[0]);
	}

	if (prepare_args->num_in_map_entries > 1)
		prepare_args->num_in_map_entries =
			cam_common_util_remove_duplicate_arr(
			sync_in_obj, prepare_args->num_in_map_entries);

	if (prepare_args->num_in_map_entries > 1) {
		rc = cam_sync_merge(&sync_in_obj[0],
			prepare_args->num_in_map_entries, &merged_sync_in_obj);
		if (rc) {
			prepare_args->num_out_map_entries = 0;
			prepare_args->num_in_map_entries = 0;
			return rc;
		}

		ctx_data->hfi_frame_process.in_resource[index] =
			merged_sync_in_obj;
		prepare_args->in_map_entries[0].sync_id = merged_sync_in_obj;
		prepare_args->num_in_map_entries = 1;
		CAM_DBG(CAM_REQ, "%s: req_id: %llu Merged Sync obj: %d",
			ctx_data->ctx_id_string, packet->header.request_id,
			merged_sync_in_obj);
	} else if (prepare_args->num_in_map_entries == 1) {
		prepare_args->in_map_entries[0].sync_id = sync_in_obj[0];
		prepare_args->num_in_map_entries = 1;
		ctx_data->hfi_frame_process.in_resource[index] = 0;
	} else {
		CAM_DBG(CAM_ICP, "%s: No input fences for req id: %llu",
			ctx_data->ctx_id_string, packet->header.request_id);
		prepare_args->num_in_map_entries = 0;
		ctx_data->hfi_frame_process.in_resource[index] = 0;
	}

	return rc;
}

static int cam_icp_process_stream_settings(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_cmd_mem_regions *cmd_mem_regions,
	bool map_unmap)
{
	int rc = 0, i = 0;
	size_t packet_size, map_cmd_size, len;
	dma_addr_t iova;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct hfi_cmd_ipe_bps_map  *map_cmd;
	struct hfi_cmd_dev_async *async_direct;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;

	if (ctx_data->device_info->hw_dev_type == CAM_ICP_DEV_OFE) {
		CAM_DBG(CAM_ICP, "%s OFE FW does not support map/unmap operations",
			ctx_data->ctx_id_string);
		return 0;
	}

	map_cmd_size =
		sizeof(struct hfi_cmd_ipe_bps_map) +
		((cmd_mem_regions->num_regions - 1) *
		sizeof(struct mem_map_region_data));

	map_cmd = kzalloc(map_cmd_size, GFP_KERNEL);
	if (!map_cmd)
		return -ENOMEM;

	for (i = 0; i < cmd_mem_regions->num_regions; i++) {
		rc = cam_mem_get_io_buf(
			cmd_mem_regions->map_info_array[i].mem_handle,
			hw_mgr->iommu_hdl, &iova, &len, NULL, NULL);
		if (rc) {
			CAM_ERR(CAM_ICP,
				"%s: Failed to get cmd region iova for handle %u",
				ctx_data->ctx_id_string,
				cmd_mem_regions->map_info_array[i].mem_handle);
			kfree(map_cmd);
			return -EINVAL;
		}

		/* FW/CDM buffers are expected to be mapped in 32-bit address range */
		map_cmd->mem_map_region_sets[i].start_addr = (uint32_t)iova +
			(cmd_mem_regions->map_info_array[i].offset);
		map_cmd->mem_map_region_sets[i].len = (uint32_t) len;

		CAM_DBG(CAM_ICP, "%s: Region %u mem_handle %d iova %pK len %u",
			ctx_data->ctx_id_string, (i+1),
			cmd_mem_regions->map_info_array[i].mem_handle,
			(uint32_t)iova, (uint32_t)len);
	}

	map_cmd->mem_map_request_num = cmd_mem_regions->num_regions;
	map_cmd->user_data = 0;

	packet_size =
		sizeof(struct hfi_cmd_dev_async) +
		(sizeof(struct hfi_cmd_ipe_bps_map) +
		((cmd_mem_regions->num_regions - 1) *
		sizeof(struct mem_map_region_data))) -
		sizeof(((struct hfi_cmd_dev_async *)0)->payload.direct);

	async_direct = kzalloc(packet_size, GFP_KERNEL);
	if (!async_direct) {
		kfree(map_cmd);
		return -ENOMEM;
	}

	async_direct->size = packet_size;
	async_direct->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT;
	if (map_unmap)
		async_direct->opcode = HFI_IPEBPS_CMD_OPCODE_MEM_MAP;
	else
		async_direct->opcode = HFI_IPEBPS_CMD_OPCODE_MEM_UNMAP;
	async_direct->num_fw_handles = 1;
	async_direct->fw_handles[0] = ctx_data->fw_handle;
	async_direct->user_data1 = (uint64_t)ctx_data;
	async_direct->user_data2 = (uint64_t)0x0;
	memcpy(async_direct->payload.direct, map_cmd,
		map_cmd_size);

	reinit_completion(&ctx_data->wait_complete);
	rc = hfi_write_cmd(hw_mgr->hfi_handle, async_direct);
	if (rc) {
		CAM_ERR(CAM_ICP, "%s: hfi write failed  rc %d",
			ctx_data->ctx_id_string, rc);
		goto end;
	}

	CAM_DBG(CAM_ICP, "%s: Sent FW %s cmd", ctx_data->ctx_id_string,
		map_unmap ? "Map" : "Unmap");

	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
			&ctx_data->wait_complete,
			msecs_to_jiffies(timeout), CAM_ICP,
			"%s: FW response timeout for process stream setting handle command",
			ctx_data->ctx_id_string);
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		cam_icp_dump_debug_info(ctx_data->hw_mgr_priv, false);
	}

end:
	kfree(map_cmd);
	kfree(async_direct);
	return rc;
}

static int cam_icp_process_presil_hangdump_info(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_cmd_mem_regions *cmd_mem_regions,
	uint32_t index)
{
	int i = 0;
	struct cam_hangdump_mem_regions *mem_regions = NULL;

	if (!ctx_data || !cmd_mem_regions) {
		CAM_ERR(CAM_ICP, "Invalid hangdump info blob ctx %pK mem_region %pK",
			ctx_data, cmd_mem_regions);
		return -EINVAL;
	}

	if ((cmd_mem_regions->num_regions == 0) ||
		(cmd_mem_regions->num_regions > HANG_DUMP_REGIONS_MAX)) {
		CAM_ERR(CAM_ICP, "%s Invalid num hangdump mem regions %d ",
			ctx_data->ctx_id_string, cmd_mem_regions->num_regions);
		return -EINVAL;
	}

	mem_regions = &ctx_data->hfi_frame_process.hangdump_mem_regions[index];
	CAM_INFO(CAM_ICP, "%s Hangdump Mem Num Regions %d index %d  mem_regions 0x%pK",
		ctx_data->ctx_id_string, cmd_mem_regions->num_regions, index, mem_regions);

	for (i = 0; i < cmd_mem_regions->num_regions; i++) {
		mem_regions->mem_info_array[i].mem_handle =
			cmd_mem_regions->map_info_array[i].mem_handle;
		mem_regions->mem_info_array[i].offset =
			cmd_mem_regions->map_info_array[i].offset;
		mem_regions->mem_info_array[i].size =
			cmd_mem_regions->map_info_array[i].size;
		CAM_INFO(CAM_ICP, "%s Hangdump Mem Region %u mem_handle 0x%08x iova 0x%08x len %u",
			ctx_data->ctx_id_string, i, cmd_mem_regions->map_info_array[i].mem_handle,
			(uint32_t)cmd_mem_regions->map_info_array[i].offset,
			(uint32_t)cmd_mem_regions->map_info_array[i].size);
	}
	mem_regions->num_mem_regions = cmd_mem_regions->num_regions;

	return 0;
}

static int cam_icp_packet_generic_blob_handler(void *user_data,
	uint32_t blob_type, uint32_t blob_size, uint8_t *blob_data)
{
	struct cam_icp_clk_bw_request *soc_req;
	struct cam_icp_clk_bw_request *clk_info;
	struct cam_icp_clk_bw_request_v2 *soc_req_v2;
	struct cam_icp_clk_bw_req_internal_v2 *clk_info_v2;
	struct cam_cmd_mem_regions *cmd_mem_regions;
	struct icp_cmd_generic_blob *blob;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct cam_icp_hw_mgr *hw_mgr;
	struct cam_icp_acquire_dev_info dev_io_info;
	uint32_t index;
	size_t io_buf_size, clk_update_size;
	int rc = 0;
	uintptr_t pResource;
	uint32_t i = 0;

	if (!blob_data || (blob_size == 0)) {
		CAM_ERR(CAM_ICP, "Invalid blob info %pK %d", blob_data,
			blob_size);
		return -EINVAL;
	}

	blob = (struct icp_cmd_generic_blob *)user_data;
	ctx_data = blob->ctx;
	hw_mgr = ctx_data->hw_mgr_priv;
	index = blob->frame_info_idx;

	switch (blob_type) {
	case CAM_ICP_CMD_GENERIC_BLOB_CLK:
		CAM_WARN_RATE_LIMIT_CUSTOM(CAM_PERF, 300, 1,
			"Using deprecated blob type GENERIC_BLOB_CLK");
		if (blob_size != sizeof(struct cam_icp_clk_bw_request)) {
			CAM_ERR(CAM_ICP, "%s: Mismatch blob size %d expected %lu",
				ctx_data->ctx_id_string, blob_size,
				sizeof(struct cam_icp_clk_bw_request));
			return -EINVAL;
		}

		if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_UNKNOWN) {
			ctx_data->bw_config_version = CAM_ICP_BW_CONFIG_V1;
		} else if (ctx_data->bw_config_version !=
			CAM_ICP_BW_CONFIG_V1) {
			CAM_ERR(CAM_ICP,
				"%s: Mismatch blob versions %d expected v1 %d, blob_type=%d",
				ctx_data->ctx_id_string, ctx_data->bw_config_version,
				CAM_ICP_BW_CONFIG_V1, blob_type);
			return -EINVAL;
		}

		clk_info = &ctx_data->hfi_frame_process.clk_info[index];

		soc_req = (struct cam_icp_clk_bw_request *)blob_data;
		*clk_info = *soc_req;
		CAM_DBG(CAM_PERF, "%s: budget:%llu fc: %llu %d BW %lld %lld",
			ctx_data->ctx_id_string, clk_info->budget_ns, clk_info->frame_cycles,
			clk_info->rt_flag, clk_info->uncompressed_bw,
			clk_info->compressed_bw);
		break;

	case CAM_ICP_CMD_GENERIC_BLOB_CLK_V2:
		if (blob_size < sizeof(struct cam_icp_clk_bw_request_v2)) {
			CAM_ERR(CAM_ICP, "%s: Mismatch blob size %d expected %lu",
				ctx_data->ctx_id_string,
				blob_size,
				sizeof(struct cam_icp_clk_bw_request_v2));
			return -EINVAL;
		}

		if (ctx_data->bw_config_version == CAM_ICP_BW_CONFIG_UNKNOWN) {
			ctx_data->bw_config_version = CAM_ICP_BW_CONFIG_V2;
		} else if (ctx_data->bw_config_version !=
			CAM_ICP_BW_CONFIG_V2) {
			CAM_ERR(CAM_ICP,
				"%s: Mismatch blob versions %d expected v2 %d, blob_type=%d",
				ctx_data->ctx_id_string, ctx_data->bw_config_version,
				CAM_ICP_BW_CONFIG_V2, blob_type);
			return -EINVAL;
		}

		soc_req_v2 = (struct cam_icp_clk_bw_request_v2 *)blob_data;
		if (soc_req_v2->num_paths > CAM_ICP_MAX_PER_PATH_VOTES) {
			CAM_ERR(CAM_PERF, "%s: Invalid num paths: %d",
				ctx_data->ctx_id_string, soc_req_v2->num_paths);
			return -EINVAL;
		}

		/* Check for integer overflow */
		if (soc_req_v2->num_paths != 1) {
			if (sizeof(struct cam_axi_per_path_bw_vote) >
				((UINT_MAX -
				sizeof(struct cam_icp_clk_bw_request_v2)) /
				(soc_req_v2->num_paths - 1))) {
				CAM_ERR(CAM_ICP,
					"%s: Size exceeds limit paths:%u size per path:%lu",
					ctx_data->ctx_id_string, soc_req_v2->num_paths - 1,
					sizeof(struct cam_axi_per_path_bw_vote));
				return -EINVAL;
			}
		}

		clk_update_size = sizeof(struct cam_icp_clk_bw_request_v2) +
			((soc_req_v2->num_paths - 1) * sizeof(struct cam_axi_per_path_bw_vote));
		if (blob_size < clk_update_size) {
			CAM_ERR(CAM_ICP, "%s: Invalid blob size: %u",
				ctx_data->ctx_id_string, blob_size);
			return -EINVAL;
		}

		clk_info = &ctx_data->hfi_frame_process.clk_info[index];
		clk_info_v2 = &ctx_data->hfi_frame_process.clk_info_v2[index];
		clk_info_v2->budget_ns = soc_req_v2->budget_ns;
		clk_info_v2->frame_cycles = soc_req_v2->frame_cycles;
		clk_info_v2->rt_flag = soc_req_v2->rt_flag;
		clk_info_v2->num_paths = soc_req_v2->num_paths;

		for (i = 0; i < soc_req_v2->num_paths; i++) {
			clk_info_v2->axi_path[i].usage_data = soc_req_v2->axi_path[i].usage_data;
			clk_info_v2->axi_path[i].transac_type =
				soc_req_v2->axi_path[i].transac_type;
			clk_info_v2->axi_path[i].path_data_type =
				soc_req_v2->axi_path[i].path_data_type;
			clk_info_v2->axi_path[i].vote_level = 0;
			clk_info_v2->axi_path[i].camnoc_bw = soc_req_v2->axi_path[i].camnoc_bw;
			clk_info_v2->axi_path[i].mnoc_ab_bw = soc_req_v2->axi_path[i].mnoc_ab_bw;
			clk_info_v2->axi_path[i].mnoc_ib_bw = soc_req_v2->axi_path[i].mnoc_ib_bw;
		}

		/* Use v1 structure for clk fields */
		clk_info->budget_ns = clk_info_v2->budget_ns;
		clk_info->frame_cycles = clk_info_v2->frame_cycles;
		clk_info->rt_flag = clk_info_v2->rt_flag;

		CAM_DBG(CAM_PERF,
			"%s: budget=%llu, frame_cycle=%llu, rt_flag=%d, num_paths=%d, index=%d, ctx_data=%pK",
			ctx_data->ctx_id_string, clk_info_v2->budget_ns,
			clk_info_v2->frame_cycles, clk_info_v2->rt_flag, clk_info_v2->num_paths,
			index, ctx_data);

		for (i = 0; i < clk_info_v2->num_paths; i++) {
			CAM_DBG(CAM_PERF,
				"%s: [%d] : path_type=%d, trans_type=%d, camnoc=%lld, mnoc_ab=%lld, mnoc_ib=%lld",
				ctx_data->ctx_id_string,
				i,
				clk_info_v2->axi_path[i].path_data_type,
				clk_info_v2->axi_path[i].transac_type,
				clk_info_v2->axi_path[i].camnoc_bw,
				clk_info_v2->axi_path[i].mnoc_ab_bw,
				clk_info_v2->axi_path[i].mnoc_ib_bw);
		}

		break;

	case CAM_ICP_CMD_GENERIC_BLOB_CFG_IO:
		CAM_DBG(CAM_ICP, "%s: CAM_ICP_CMD_GENERIC_BLOB_CFG_IO", ctx_data->ctx_id_string);
		pResource = *((uint32_t *)blob_data);
		if (copy_from_user(&dev_io_info,
			(void __user *)pResource,
			sizeof(struct cam_icp_acquire_dev_info))) {
			CAM_ERR(CAM_ICP, "%s: Failed in copy from user", ctx_data->ctx_id_string);
			return -EFAULT;
		}
		CAM_DBG(CAM_ICP, "%s: buf handle %d", ctx_data->ctx_id_string,
			dev_io_info.io_config_cmd_handle);
		rc = cam_mem_get_io_buf(dev_io_info.io_config_cmd_handle, hw_mgr->iommu_hdl,
			blob->io_buf_addr, &io_buf_size, NULL, NULL);
		if (rc)
			CAM_ERR(CAM_ICP, "%s: Failed in blob update", ctx_data->ctx_id_string);
		else
			CAM_DBG(CAM_ICP, "%s: io buf addr %llu",
				ctx_data->ctx_id_string, *blob->io_buf_addr);
		break;

	case CAM_ICP_CMD_GENERIC_BLOB_FW_MEM_MAP:
		cmd_mem_regions =
			(struct cam_cmd_mem_regions *)blob_data;
		if (cmd_mem_regions->num_regions <= 0) {
			rc = -EINVAL;
			CAM_ERR(CAM_ICP,
				"%s: Invalid number of regions for FW map %u",
				ctx_data->ctx_id_string, cmd_mem_regions->num_regions);
		} else {
			CAM_DBG(CAM_ICP,
				"%s: Processing blob for mapping %u regions",
				ctx_data->ctx_id_string, cmd_mem_regions->num_regions);
			rc = cam_icp_process_stream_settings(ctx_data,
				cmd_mem_regions, true);
		}
		break;

	case CAM_ICP_CMD_GENERIC_BLOB_FW_MEM_UNMAP:
		cmd_mem_regions =
			(struct cam_cmd_mem_regions *)blob_data;
		if (cmd_mem_regions->num_regions <= 0) {
			rc = -EINVAL;
			CAM_ERR(CAM_ICP,
				"%s: Invalid number of regions for FW unmap %u",
				ctx_data->ctx_id_string, cmd_mem_regions->num_regions);
		} else {
			CAM_DBG(CAM_ICP,
				"%s: Processing blob for unmapping %u regions",
				ctx_data->ctx_id_string, cmd_mem_regions->num_regions);
			rc = cam_icp_process_stream_settings(ctx_data,
				cmd_mem_regions, false);
		}
		break;

	case CAM_ICP_CMD_GENERIC_BLOB_PRESIL_HANGDUMP:
		if (cam_presil_mode_enabled()) {
			cmd_mem_regions = (struct cam_cmd_mem_regions *)blob_data;
			if (cmd_mem_regions->num_regions <= 0) {
				CAM_INFO(CAM_ICP, "%s: Pre-sil Hangdump disabled %u",
					ctx_data->ctx_id_string, cmd_mem_regions->num_regions);
			} else {
				CAM_INFO(CAM_ICP,
					"%s: Pre-sil Hangdump enabled %u entries index %d",
					ctx_data->ctx_id_string, cmd_mem_regions->num_regions,
					index);
				rc = cam_icp_process_presil_hangdump_info(ctx_data,
					cmd_mem_regions, index);
			}
		}
		break;

	default:
		CAM_WARN(CAM_ICP, "%s: Invalid blob type %d", ctx_data->ctx_id_string,
			blob_type);
		break;
	}
	return rc;
}

static int cam_icp_process_generic_cmd_buffer(
	struct cam_packet *packet,
	struct cam_icp_hw_ctx_data *ctx_data,
	int32_t index,
	dma_addr_t *io_buf_addr)
{
	int i, rc = 0;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct icp_cmd_generic_blob cmd_generic_blob;

	cmd_generic_blob.ctx = ctx_data;
	cmd_generic_blob.frame_info_idx = index;
	cmd_generic_blob.io_buf_addr = io_buf_addr;

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *) &packet->payload + packet->cmd_buf_offset/4);
	for (i = 0; i < packet->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		if (!cmd_desc[i].length)
			continue;

		if (cmd_desc[i].meta_data != CAM_ICP_CMD_META_GENERIC_BLOB)
			continue;

		rc = cam_packet_util_process_generic_cmd_buffer(&cmd_desc[i],
			cam_icp_packet_generic_blob_handler, &cmd_generic_blob);
		if (rc)
			CAM_ERR(CAM_ICP, "Failed in processing blobs %d", rc);
	}

	return rc;
}

static int cam_icp_mgr_process_cfg_io_cmd(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct hfi_cmd_dev_async *ioconfig_cmd,
	uint64_t request_id,
	uint64_t io_config)
{
	switch (ctx_data->device_info->hw_dev_type) {
	case CAM_ICP_DEV_BPS:
		ioconfig_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_BPS_CONFIG_IO;
		ioconfig_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
		break;
	case CAM_ICP_DEV_IPE:
		ioconfig_cmd->opcode = HFI_IPEBPS_CMD_OPCODE_IPE_CONFIG_IO;
		ioconfig_cmd->pkt_type = HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT;
		break;
	case CAM_ICP_DEV_OFE:
		ioconfig_cmd->opcode = HFI_OFE_CMD_OPCODE_CONFIG_IO;
		ioconfig_cmd->pkt_type = HFI_CMD_OFE_ASYNC_COMMAND;
		break;
	default:
		CAM_ERR(CAM_ICP, "%s: Invalid device type %u not supported",
			ctx_data->ctx_id_string, ctx_data->device_info->hw_dev_type);
		return -EINVAL;
	}

	ioconfig_cmd->size = sizeof(struct hfi_cmd_dev_async);
	ioconfig_cmd->num_fw_handles = 1;
	ioconfig_cmd->fw_handles[0] = ctx_data->fw_handle;
	ioconfig_cmd->payload.indirect = io_config;
	ioconfig_cmd->user_data1 = PTR_TO_U64(ctx_data);
	ioconfig_cmd->user_data2 = request_id;

	return 0;
}

static int cam_icp_mgr_update_hfi_frame_process(
	struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_packet *packet,
	struct cam_hw_prepare_update_args *prepare_args,
	int32_t *idx)
{
	int32_t index, rc;
	struct hfi_cmd_dev_async *hfi_cmd = NULL;

	index = find_first_zero_bit(ctx_data->hfi_frame_process.bitmap,
		ctx_data->hfi_frame_process.bits);
	if (index < 0 || index >= CAM_FRAME_CMD_MAX) {
		CAM_ERR(CAM_ICP, "%s request idx is wrong: %d",
			ctx_data->ctx_id_string, index);
		return -EINVAL;
	}
	set_bit(index, ctx_data->hfi_frame_process.bitmap);

	ctx_data->hfi_frame_process.request_id[index] =
		packet->header.request_id;
	ctx_data->hfi_frame_process.frame_info[index].request_id =
		packet->header.request_id;
	ctx_data->hfi_frame_process.frame_info[index].io_config = 0;
	ctx_data->hfi_frame_process.frame_info[index].pkt = packet;
	rc = cam_icp_process_generic_cmd_buffer(packet, ctx_data, index,
		&ctx_data->hfi_frame_process.frame_info[index].io_config);
	if (rc) {
		clear_bit(index, ctx_data->hfi_frame_process.bitmap);
		ctx_data->hfi_frame_process.request_id[index] = -1;
		return rc;
	}

	if (ctx_data->hfi_frame_process.frame_info[index].io_config) {
		hfi_cmd = (struct hfi_cmd_dev_async *)
		&ctx_data->hfi_frame_process.frame_info[index].hfi_cfg_io_cmd;
		rc = cam_icp_mgr_process_cfg_io_cmd(ctx_data, hfi_cmd,
			packet->header.request_id,
			ctx_data->hfi_frame_process.frame_info[index].io_config);
	}
	*idx = index;

	return rc;
}

static int cam_icp_mgr_config_stream_settings(
	void *hw_mgr_priv, void *hw_stream_settings)
{
	int        rc = 0;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct cam_packet *packet = NULL;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct icp_cmd_generic_blob cmd_generic_blob;
	struct cam_hw_stream_setttings *config_args =
		hw_stream_settings;

	if ((!hw_stream_settings) ||
		(!hw_mgr) || (!config_args->packet)) {
		CAM_ERR(CAM_ICP, "Invalid input arguments");
		return -EINVAL;
	}

	ctx_data = config_args->ctxt_to_hw_map;
	mutex_lock(&ctx_data->ctx_mutex);
	packet = config_args->packet;

	cmd_generic_blob.ctx = ctx_data;
	cmd_generic_blob.frame_info_idx = -1;
	cmd_generic_blob.io_buf_addr = NULL;

	cmd_desc = (struct cam_cmd_buf_desc *)
		((uint32_t *) &packet->payload + packet->cmd_buf_offset/4);

	rc = cam_packet_util_validate_cmd_desc(cmd_desc);
	if (rc)
		return rc;

	if (!cmd_desc[0].length ||
		cmd_desc[0].meta_data != CAM_ICP_CMD_META_GENERIC_BLOB) {
		CAM_ERR(CAM_ICP, "%s: Invalid cmd buffer length/metadata",
			ctx_data->ctx_id_string);
		rc = -EINVAL;
		goto end;
	}

	rc = cam_packet_util_process_generic_cmd_buffer(&cmd_desc[0],
		cam_icp_packet_generic_blob_handler, &cmd_generic_blob);
	if (rc)
		CAM_ERR(CAM_ICP, "%s: Failed in processing cmd mem blob %d",
		ctx_data->ctx_id_string, rc);

end:
	mutex_unlock(&ctx_data->ctx_mutex);
	return rc;
}

static int cam_icp_mgr_prepare_hw_update(void *hw_mgr_priv,
	void *prepare_hw_update_args)
{
	int        rc = 0;
	int32_t    idx;
	uint32_t   fw_cmd_buf_iova_addr;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct cam_packet *packet = NULL;
	struct hfi_cmd_dev_async *hfi_cmd = NULL;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_hw_prepare_update_args *prepare_args =
		prepare_hw_update_args;

	if ((!prepare_args) || (!hw_mgr) || (!prepare_args->packet)) {
		CAM_ERR(CAM_ICP, "Invalid args");
		return -EINVAL;
	}

	ctx_data = prepare_args->ctxt_to_hw_map;
	mutex_lock(&ctx_data->ctx_mutex);
	if (ctx_data->state != CAM_ICP_CTX_STATE_ACQUIRED) {
		mutex_unlock(&ctx_data->ctx_mutex);
		CAM_ERR(CAM_ICP, "%s: is not in use", ctx_data->ctx_id_string);
		return -EINVAL;
	}

	packet = prepare_args->packet;

	if (cam_packet_util_validate_packet(packet, prepare_args->remain_len))
		return -EINVAL;

	rc = cam_icp_mgr_pkt_validation(ctx_data, packet);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	rc = cam_icp_mgr_process_cmd_desc(hw_mgr, packet,
		ctx_data, &fw_cmd_buf_iova_addr, prepare_args->buf_tracker);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	CAM_DBG(CAM_REQ, "%s: req id = %lld", ctx_data->ctx_id_string,
		packet->header.request_id);
	/* Update Buffer Address from handles and patch information */
	rc = cam_packet_util_process_patches(packet, prepare_args->buf_tracker,
		hw_mgr->iommu_hdl, hw_mgr->iommu_sec_hdl, true);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	rc = cam_icp_mgr_update_hfi_frame_process(ctx_data, packet,
		prepare_args, &idx);
	if (rc) {
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	rc = cam_icp_mgr_process_io_cfg(hw_mgr, ctx_data,
		packet, prepare_args, idx);
	if (rc) {
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0)
			cam_sync_destroy(
				ctx_data->hfi_frame_process.in_resource[idx]);
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
		ctx_data->hfi_frame_process.request_id[idx] = -1;
		mutex_unlock(&ctx_data->ctx_mutex);
		return rc;
	}

	hfi_cmd = (struct hfi_cmd_dev_async *)
			&ctx_data->hfi_frame_process.hfi_frame_cmd[idx];
	cam_icp_mgr_prepare_frame_process_cmd(
		ctx_data, hfi_cmd, packet->header.request_id,
		fw_cmd_buf_iova_addr);

	prepare_args->num_hw_update_entries = 1;
	prepare_args->hw_update_entries[0].addr = (uintptr_t)hfi_cmd;
	prepare_args->priv = &ctx_data->hfi_frame_process.frame_info[idx];

	CAM_DBG(CAM_ICP, "%s: X: req id = %lld", ctx_data->ctx_id_string,
		packet->header.request_id);
	mutex_unlock(&ctx_data->ctx_mutex);
	return rc;
}

static int cam_icp_mgr_send_abort_status(struct cam_icp_hw_ctx_data *ctx_data)
{
	struct hfi_frame_process_info *hfi_frame_process;
	struct cam_icp_hw_buf_done_evt_data icp_evt_data;
	struct cam_hw_done_event_data buf_data = {0};
	int idx;

	mutex_lock(&ctx_data->ctx_mutex);
	hfi_frame_process = &ctx_data->hfi_frame_process;
	buf_data.evt_param = CAM_SYNC_ICP_EVENT_ABORTED;
	icp_evt_data.evt_id = CAM_CTX_EVT_ID_CANCEL;
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->request_id[idx])
			continue;

		buf_data.request_id = hfi_frame_process->request_id[idx];
		icp_evt_data.buf_done_data = &buf_data;
		ctx_data->ctxt_event_cb(ctx_data->context_priv,
			CAM_ICP_EVT_ID_BUF_DONE,
			&icp_evt_data);

		/* now release memory for hfi frame process command */
		hfi_frame_process->request_id[idx] = 0;
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
			CAM_DBG(CAM_ICP, "%s: Delete merged sync in object: %d",
				ctx_data->ctx_id_string,
				ctx_data->hfi_frame_process.in_resource[idx]);
			cam_sync_destroy(
				ctx_data->hfi_frame_process.in_resource[idx]);
			ctx_data->hfi_frame_process.in_resource[idx] = 0;
		}
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
	}
	mutex_unlock(&ctx_data->ctx_mutex);
	return 0;
}

static int cam_icp_mgr_delete_sync(void *priv, void *data)
{
	struct hfi_cmd_work_data *task_data = NULL;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct hfi_frame_process_info *hfi_frame_process;
	int idx;

	if (!data || !priv) {
		CAM_ERR(CAM_ICP, "Invalid params%pK %pK", data, priv);
		return -EINVAL;
	}

	task_data = (struct hfi_cmd_work_data *)data;
	ctx_data = task_data->data;

	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "Null Context");
		return -EINVAL;
	}

	mutex_lock(&ctx_data->ctx_mutex);
	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->in_free_resource[idx])
			continue;
		//cam_sync_destroy(
			//ctx_data->hfi_frame_process.in_free_resource[idx]);
		ctx_data->hfi_frame_process.in_resource[idx] = 0;
	}
	mutex_unlock(&ctx_data->ctx_mutex);
	return 0;
}

static int cam_icp_mgr_delete_sync_obj(struct cam_icp_hw_ctx_data *ctx_data)
{
	int rc = 0;
	struct crm_workq_task *task;
	struct hfi_cmd_work_data *task_data;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;

	task = cam_req_mgr_workq_get_task(hw_mgr->cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "%s: no empty task", ctx_data->ctx_id_string);
		return -ENOMEM;
	}

	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)ctx_data;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_delete_sync;
	rc = cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);

	return rc;
}

static int cam_icp_mgr_flush_all(struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_hw_flush_args *flush_args)
{
	struct hfi_frame_process_info *hfi_frame_process;
	int idx;
	bool clear_in_resource = false;

	hfi_frame_process = &ctx_data->hfi_frame_process;
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->request_id[idx])
			continue;

		/* now release memory for hfi frame process command */
		hfi_frame_process->request_id[idx] = 0;
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
			ctx_data->hfi_frame_process.in_free_resource[idx] =
				ctx_data->hfi_frame_process.in_resource[idx];
			ctx_data->hfi_frame_process.in_resource[idx] = 0;
		}
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
		clear_in_resource = true;
	}

	if (clear_in_resource)
		cam_icp_mgr_delete_sync_obj(ctx_data);

	return 0;
}

static int cam_icp_mgr_flush_req(struct cam_icp_hw_ctx_data *ctx_data,
	struct cam_hw_flush_args *flush_args)
{
	int64_t request_id;
	struct hfi_frame_process_info *hfi_frame_process;
	int idx;
	bool clear_in_resource = false;

	hfi_frame_process = &ctx_data->hfi_frame_process;
	request_id = *(int64_t *)flush_args->flush_req_pending[0];
	for (idx = 0; idx < CAM_FRAME_CMD_MAX; idx++) {
		if (!hfi_frame_process->request_id[idx])
			continue;

		if (hfi_frame_process->request_id[idx] != request_id)
			continue;

		/* now release memory for hfi frame process command */
		hfi_frame_process->request_id[idx] = 0;
		if (ctx_data->hfi_frame_process.in_resource[idx] > 0) {
			ctx_data->hfi_frame_process.in_free_resource[idx] =
				ctx_data->hfi_frame_process.in_resource[idx];
			ctx_data->hfi_frame_process.in_resource[idx] = 0;
		}
		clear_bit(idx, ctx_data->hfi_frame_process.bitmap);
		clear_in_resource = true;
	}

	if (clear_in_resource)
		cam_icp_mgr_delete_sync_obj(ctx_data);

	return 0;
}

static void cam_icp_mgr_flush_info_dump(
	struct cam_hw_flush_args *flush_args, uint32_t ctx_id)
{
	int i;

	for (i = 0; i < flush_args->num_req_active; i++) {
		CAM_DBG(CAM_ICP, "Flushing active request %lld in ctx %u",
			*(int64_t *)flush_args->flush_req_active[i],
			ctx_id);
	}

	for (i = 0; i < flush_args->num_req_pending; i++) {
		CAM_DBG(CAM_ICP, "Flushing pending request %lld in ctx %u",
			*(int64_t *)flush_args->flush_req_pending[i],
			ctx_id);
	}
}

static int cam_icp_mgr_enqueue_abort(
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int timeout = 2000, rc;
	unsigned long rem_jiffies = 0;
	struct cam_icp_hw_mgr *hw_mgr = ctx_data->hw_mgr_priv;
	struct hfi_cmd_work_data *task_data;
	struct crm_workq_task *task;

	task = cam_req_mgr_workq_get_task(hw_mgr->cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "%s: no empty task", ctx_data->ctx_id_string);
		return -ENOMEM;
	}

	reinit_completion(&ctx_data->wait_complete);
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)ctx_data;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_abort_handle_wq;
	cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);

	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
		&ctx_data->wait_complete,
		msecs_to_jiffies(timeout), CAM_ICP,
		"%s FW response timeout for Abort handle command",
		ctx_data->ctx_id_string);
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		cam_icp_dump_debug_info(hw_mgr, false);
		ctx_data->abort_timed_out = true;
		return rc;
	}

	CAM_DBG(CAM_ICP, "%s: Abort after flush is success", ctx_data->ctx_id_string);
	return 0;
}

static int cam_icp_mgr_hw_dump(void *hw_priv, void *hw_dump_args)
{
	int                              rc;
	int                              i, j;
	size_t                           remain_len;
	uint8_t                         *dst;
	uint32_t                         min_len;
	uint64_t                        *addr, *start;
	uint64_t                        *clk_addr, *clk_start;
	uint32_t                        *mgr_addr, *mgr_start;
	struct timespec64                cur_ts;
	struct timespec64                req_ts;
	ktime_t                          cur_time;
	struct cam_hw_intf              *icp_dev_intf;
	struct cam_icp_hw_mgr           *hw_mgr;
	struct cam_hw_dump_args         *dump_args;
	struct cam_icp_hw_ctx_data      *ctx_data;
	struct cam_icp_dump_header      *hdr;
	struct cam_icp_hw_dump_args      icp_dump_args;
	struct hfi_frame_process_info   *frm_process;
	int                              frm_idx = -1;

	if ((!hw_priv) || (!hw_dump_args)) {
		CAM_ERR(CAM_ICP, "Invalid params %pK %pK",
			hw_priv, hw_dump_args);
		return -EINVAL;
	}

	memset(&req_ts, 0, sizeof(struct timespec64));

	dump_args = (struct cam_hw_dump_args *)hw_dump_args;
	hw_mgr = hw_priv;
	ctx_data = dump_args->ctxt_to_hw_map;
	CAM_DBG(CAM_ICP, "[%s] Req %lld",
		hw_mgr->hw_mgr_name,
		dump_args->request_id);
	frm_process = &ctx_data->hfi_frame_process;
	for (i = 0; i < CAM_FRAME_CMD_MAX; i++) {
		if ((frm_process->request_id[i] ==
			dump_args->request_id) &&
			frm_process->fw_process_flag[i]) {
			frm_idx = i;
			break;
		}
	}

	cur_time = ktime_get();
	cur_ts = ktime_to_timespec64(cur_time);
	if (frm_idx >= 0) {
		req_ts = ktime_to_timespec64(frm_process->submit_timestamp[frm_idx]);
	}

	CAM_INFO(CAM_ICP,
		"[%s] Error req %lld req timestamp:[%lld.%06lld] curr timestamp:[%lld.%06lld]",
		hw_mgr->hw_mgr_name, dump_args->request_id,
		req_ts.tv_sec,
		req_ts.tv_nsec/NSEC_PER_USEC,
		cur_ts.tv_sec,
		cur_ts.tv_nsec/NSEC_PER_USEC);
	rc  = cam_mem_get_cpu_buf(dump_args->buf_handle,
		&icp_dump_args.cpu_addr, &icp_dump_args.buf_len);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Invalid addr %u rc %d",
			hw_mgr->hw_mgr_name, dump_args->buf_handle, rc);
		return rc;
	}
	if (icp_dump_args.buf_len <= dump_args->offset) {
		CAM_WARN(CAM_ICP, "[%s] dump buffer overshoot len %zu offset %zu",
			hw_mgr->hw_mgr_name, icp_dump_args.buf_len, dump_args->offset);
		rc = -ENOSPC;
		goto put_cpu_buf;
	}

	remain_len = icp_dump_args.buf_len - dump_args->offset;
	min_len = sizeof(struct cam_icp_dump_header) +
			(CAM_ICP_DUMP_NUM_WORDS * sizeof(uint64_t));

	if (remain_len < min_len) {
		CAM_WARN(CAM_ICP, "[%s] dump buffer exhaust remain %zu min %u",
			hw_mgr->hw_mgr_name, remain_len, min_len);
		rc = -ENOSPC;
		goto put_cpu_buf;
	}

	/* Dumping clock and bandwidth info */
	dst = (uint8_t *)icp_dump_args.cpu_addr + dump_args->offset;
	hdr = (struct cam_icp_dump_header *)dst;
	scnprintf(hdr->tag, CAM_ICP_DUMP_TAG_MAX_LEN, "ICP_HW_CLK:");
	hdr->word_size = sizeof(uint64_t);
	clk_addr = (uint64_t *)(dst + sizeof(struct cam_icp_dump_header));
	clk_start = clk_addr;
	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		*clk_addr++ = hw_mgr->dev_info[i].clk_info.prev_clk;
		*clk_addr++ = hw_mgr->dev_info[i].clk_info.curr_clk;
	}
	for (j = 0; j < ctx_data->clk_info.num_paths; j++) {
		*clk_addr++ = ctx_data->clk_info.axi_path[j].camnoc_bw;
		*clk_addr++ = ctx_data->clk_info.axi_path[j].mnoc_ab_bw;
		*clk_addr++ = ctx_data->clk_info.axi_path[j].mnoc_ib_bw;
	}
	hdr->size = hdr->word_size * (clk_addr - clk_start);
	dump_args->offset += (hdr->size + sizeof(struct cam_icp_dump_header));

	/* Dumping hw mgr info */
	dst = (uint8_t *)icp_dump_args.cpu_addr + dump_args->offset;
	hdr = (struct cam_icp_dump_header *)dst;
	scnprintf(hdr->tag, CAM_ICP_DUMP_TAG_MAX_LEN, "ICP_HW_MGR.%s:",
		cam_icp_dev_type_to_name(ctx_data->icp_dev_acquire_info->dev_type));
	hdr->word_size = sizeof(uint32_t);
	mgr_addr = (uint32_t *)(dst + sizeof(struct cam_icp_dump_header));
	mgr_start = mgr_addr;
	*mgr_addr++ = atomic_read(&hw_mgr->recovery);
	*mgr_addr++ = hw_mgr->icp_booted;
	*mgr_addr++ = hw_mgr->icp_resumed;
	*mgr_addr++ = hw_mgr->disable_ubwc_comp;
	memcpy(mgr_addr, &hw_mgr->dev_info, sizeof(hw_mgr->dev_info));
	mgr_addr += sizeof(hw_mgr->dev_info);
	*mgr_addr++ = hw_mgr->icp_pc_flag;
	*mgr_addr++ = hw_mgr->dev_pc_flag;
	*mgr_addr++ = hw_mgr->icp_use_pil;
	hdr->size = hdr->word_size * (mgr_addr - mgr_start);
	dump_args->offset += (hdr->size + sizeof(struct cam_icp_dump_header));

	/* Dumping time info */
	dst = (uint8_t *)icp_dump_args.cpu_addr + dump_args->offset;
	hdr = (struct cam_icp_dump_header *)dst;
	scnprintf(hdr->tag, CAM_ICP_DUMP_TAG_MAX_LEN, "ICP_REQ:");
	hdr->word_size = sizeof(uint64_t);
	addr = (uint64_t *)(dst + sizeof(struct cam_icp_dump_header));
	start = addr;
	*addr++ = dump_args->request_id;
	*addr++ = req_ts.tv_sec;
	*addr++ = req_ts.tv_nsec / NSEC_PER_USEC;
	*addr++ = cur_ts.tv_sec;
	*addr++ = cur_ts.tv_nsec / NSEC_PER_USEC;
	hdr->size = hdr->word_size * (addr - start);
	dump_args->offset += (hdr->size + sizeof(struct cam_icp_dump_header));

	/* Dumping the fw image*/
	icp_dump_args.offset = dump_args->offset;
	icp_dev_intf = hw_mgr->icp_dev_intf;
	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL",
			hw_mgr->hw_mgr_name);
		rc = -EINVAL;
		goto put_cpu_buf;
	}
	rc = icp_dev_intf->hw_ops.process_cmd(
		icp_dev_intf->hw_priv,
		CAM_ICP_CMD_HW_DUMP, &icp_dump_args,
		sizeof(struct cam_icp_hw_dump_args));
	CAM_DBG(CAM_ICP, "[%s] Offset before %zu after %zu",
		hw_mgr->hw_mgr_name, dump_args->offset, icp_dump_args.offset);
	dump_args->offset = icp_dump_args.offset;

put_cpu_buf:
	cam_mem_put_cpu_buf(dump_args->buf_handle);
	return rc;
}

static int cam_icp_mgr_synx_core_control(
	struct cam_icp_hw_mgr *hw_mgr,
	struct cam_synx_core_control *synx_core_ctrl)
{
	int rc;

	if (synx_core_ctrl->core_control) {
		rc = cam_icp_mgr_icp_resume(hw_mgr);
		if (!rc)
			/* Set FW log level for synx */
			if (hw_mgr->icp_debug_type)
				hfi_set_debug_level(hw_mgr->hfi_handle,
					hw_mgr->icp_debug_type, hw_mgr->icp_dbg_lvl);
	} else {
		rc = cam_icp_mgr_icp_power_collapse(hw_mgr);
	}

	if (rc)
		CAM_ERR(CAM_ICP, "[%s] Failed to process core control resume: %s",
			hw_mgr->hw_mgr_name, CAM_BOOL_TO_YESNO(synx_core_ctrl->core_control));

	CAM_INFO(CAM_ICP, "Synx test core control: %s done rc: %d",
		CAM_BOOL_TO_YESNO(synx_core_ctrl->core_control), rc);
	return rc;
}

static int cam_icp_mgr_synx_send_test_cmd(
	struct cam_icp_hw_mgr *hw_mgr,
	struct cam_synx_test_cmd *synx_test_params)
{
	int rc = 0;
	size_t size;
	dma_addr_t iova;
	struct hfi_cmd_synx_test_payload synx_test_cmd;
	unsigned long rem_jiffies;
	int timeout = 5000;

	if (!hw_mgr->icp_resumed) {
		rc = cam_icp_mgr_icp_resume(hw_mgr);
		if (rc) {
			CAM_ERR(CAM_ICP, "Failed to resume ICP rc: %d", rc);
			goto end;
		}

		/* Set FW log level for synx */
		if (hw_mgr->icp_debug_type)
			hfi_set_debug_level(hw_mgr->hfi_handle,
				hw_mgr->icp_debug_type, hw_mgr->icp_dbg_lvl);
	}

	synx_test_cmd.pkt_type = HFI_CMD_DBG_SYNX_TEST;
	synx_test_cmd.size = sizeof(synx_test_cmd);

	rc = cam_mem_get_io_buf(synx_test_params->ip_mem_hdl, hw_mgr->iommu_hdl,
		&iova, &size, NULL, NULL);
	if (rc) {
		CAM_ERR(CAM_ICP, "Failed to get buf for hdl: %d rc: %d",
			synx_test_params->ip_mem_hdl, rc);
		goto end;
	}

	synx_test_cmd.input_iova = (uint32_t)iova;
	synx_test_cmd.input_size = (uint32_t)size;

	rc = cam_mem_get_io_buf(synx_test_params->op_mem_hdl, hw_mgr->iommu_hdl,
		&iova, &size, NULL, NULL);
	if (rc) {
		CAM_ERR(CAM_ICP, "Failed to get buf for hdl: %d rc: %d",
			synx_test_params->ip_mem_hdl, rc);
		goto end;
	}

	synx_test_cmd.output_iova = (uint32_t)iova;
	synx_test_cmd.output_size = (uint32_t)size;

	CAM_DBG(CAM_ICP,
		"Input (hdl: 0x%x iova: 0x%x size: 0x%x) output (hdl: 0x%x iova: 0x%x size: 0x%x)",
		synx_test_params->ip_mem_hdl, synx_test_cmd.input_iova, synx_test_cmd.input_size,
		synx_test_params->op_mem_hdl, synx_test_cmd.output_iova, synx_test_cmd.output_size);

	reinit_completion(&hw_mgr->icp_complete);
	rc = hfi_write_cmd(hw_mgr->hfi_handle, &synx_test_cmd);
	if (rc)
		goto end;

	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
			&hw_mgr->icp_complete, msecs_to_jiffies(timeout), CAM_ICP,
			"FW response timeout for synx test cmd");
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		goto end;
	}

	CAM_INFO(CAM_ICP, "Synx test cmd done rc: %d", rc);

end:
	return rc;
}

static int cam_icp_mgr_service_synx_test_cmds(void *hw_priv, void *synx_args)
{
	int rc;
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;
	struct cam_synx_test_params *synx_params;

	if ((!hw_priv) || (!synx_args)) {
		CAM_ERR(CAM_ICP, "Input params are Null:");
		return -EINVAL;
	}

	synx_params = (struct cam_synx_test_params *)synx_args;
	mutex_lock(&hw_mgr->hw_mgr_mutex);
	switch (synx_params->cmd_type) {
	case CAM_SYNX_TEST_CMD_TYPE_CORE_CTRL: {
		rc = cam_icp_mgr_synx_core_control(hw_mgr, &synx_params->u.core_ctrl);
	}
		break;
	case CAM_SYNX_TEST_CMD_TYPE_SYNX_CMD: {
		rc = cam_icp_mgr_synx_send_test_cmd(hw_mgr, &synx_params->u.test_cmd);
	}
		break;
	default:
		rc = -EINVAL;
		goto end;
	}

end:
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_mgr_hw_flush(void *hw_priv, void *hw_flush_args)
{
	struct cam_hw_flush_args *flush_args = hw_flush_args;
	struct cam_icp_hw_ctx_data *ctx_data;
	struct cam_icp_hw_mgr *hw_mgr = hw_priv;

	if ((!hw_priv) || (!hw_flush_args)) {
		CAM_ERR(CAM_ICP, "Input params are Null:");
		return -EINVAL;
	}

	ctx_data = flush_args->ctxt_to_hw_map;
	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "[%s] ctx data is NULL", hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	if ((flush_args->flush_type >= CAM_FLUSH_TYPE_MAX) ||
		(flush_args->flush_type < CAM_FLUSH_TYPE_REQ)) {
		CAM_ERR(CAM_ICP, "%s: Invalid lush type: %d",
			ctx_data->ctx_id_string, flush_args->flush_type);
		return -EINVAL;
	}

	ctx_data->last_flush_req = flush_args->last_flush_req;
	CAM_DBG(CAM_REQ, "%s: Flush type %d last_flush_req %u",
		ctx_data->ctx_id_string, flush_args->flush_type,
		ctx_data->last_flush_req);
	switch (flush_args->flush_type) {
	case CAM_FLUSH_TYPE_ALL:
		mutex_lock(&hw_mgr->hw_mgr_mutex);
		if (!atomic_read(&hw_mgr->recovery)
			&& flush_args->num_req_active) {
			mutex_unlock(&hw_mgr->hw_mgr_mutex);
			cam_icp_mgr_flush_info_dump(flush_args,
				ctx_data->ctx_id);
			cam_icp_mgr_enqueue_abort(ctx_data);
		} else {
			mutex_unlock(&hw_mgr->hw_mgr_mutex);
		}
		mutex_lock(&ctx_data->ctx_mutex);
		cam_icp_mgr_flush_all(ctx_data, flush_args);
		mutex_unlock(&ctx_data->ctx_mutex);
		break;
	case CAM_FLUSH_TYPE_REQ:
		mutex_lock(&ctx_data->ctx_mutex);
		if (flush_args->num_req_active) {
			CAM_ERR(CAM_ICP,
				"%s: Flush a specific active request id: %lld is not supported",
				ctx_data->ctx_id_string,
				*(int64_t *)flush_args->flush_req_active[0]);
			mutex_unlock(&ctx_data->ctx_mutex);
			return -EINVAL;
		}
		if (flush_args->num_req_pending)
			cam_icp_mgr_flush_req(ctx_data, flush_args);
		mutex_unlock(&ctx_data->ctx_mutex);
		break;
	default:
		CAM_ERR(CAM_ICP, "%s: Invalid flush type: %d",
			ctx_data->ctx_id_string, flush_args->flush_type);
		return -EINVAL;
	}

	return 0;
}

static int cam_icp_mgr_release_hw(void *hw_mgr_priv, void *release_hw_args)
{
	int rc = 0, i;
	int ctx_id = 0;
	struct cam_hw_release_args *release_hw = release_hw_args;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;

	if (!release_hw || !hw_mgr) {
		CAM_ERR(CAM_ICP, "Invalid args: %pK %pK", release_hw, hw_mgr);
		return -EINVAL;
	}

	ctx_data = release_hw->ctxt_to_hw_map;

	if (!ctx_data) {
		CAM_ERR(CAM_ICP, "NULL ctx data");
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "%s: Enter recovery set %d",
		ctx_data->ctx_id_string, atomic_read(&hw_mgr->recovery));

	ctx_id = ctx_data->ctx_id;
	if (ctx_id < 0 || ctx_id >= CAM_ICP_CTX_MAX) {
		CAM_ERR(CAM_ICP, "%s: Invalid ctx id: %d",
			ctx_data->ctx_id_string, ctx_id);
		return -EINVAL;
	}

	mutex_lock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
	if (hw_mgr->ctx_data[ctx_id].state != CAM_ICP_CTX_STATE_ACQUIRED) {
		CAM_DBG(CAM_ICP, "%s: is not in use",
			hw_mgr->ctx_data[ctx_id].ctx_id_string);
		mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);
		return -EINVAL;
	}
	mutex_unlock(&hw_mgr->ctx_data[ctx_id].ctx_mutex);

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	if (!atomic_read(&hw_mgr->recovery) && release_hw->active_req) {
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
		cam_icp_mgr_abort_handle(ctx_data);
		cam_icp_mgr_send_abort_status(ctx_data);
	} else {
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	rc = cam_icp_mgr_release_ctx(hw_mgr, ctx_id);
	if (!hw_mgr->ctxt_cnt) {
		CAM_DBG(CAM_ICP, "[%s] Last Release", hw_mgr->hw_mgr_name);
		cam_icp_mgr_icp_power_collapse(hw_mgr);
		cam_icp_hw_mgr_reset_clk_info(hw_mgr);
		rc = cam_icp_device_deint(hw_mgr);
	}

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		if (hw_mgr->dev_info[i].dev_ctx_info.dev_ctxt_cnt)
			break;
	}

	if (i == hw_mgr->num_dev_info)
		cam_icp_device_timer_stop(hw_mgr);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	CAM_DBG(CAM_ICP, "[%s] Release done.", hw_mgr->hw_mgr_name);
	return rc;
}

static int cam_icp_mgr_create_handle(struct cam_icp_hw_mgr *hw_mgr,
	uint32_t dev_type, struct cam_icp_hw_ctx_data *ctx_data)
{
	struct hfi_cmd_create_handle create_handle;
	struct hfi_cmd_work_data *task_data;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct crm_workq_task *task;
	int rc = 0;
	uint32_t handle_type;

	if (ctx_data->device_info->hw_dev_type == CAM_ICP_DEV_OFE) {
		create_handle.pkt_type = HFI_CMD_OFE_CREATE_HANDLE;
		switch (dev_type) {
		case CAM_ICP_RES_TYPE_OFE_RT:
			handle_type = HFI_OFE_HANDLE_TYPE_OFE_RT;
			break;
		case CAM_ICP_RES_TYPE_OFE:
			handle_type = HFI_OFE_HANDLE_TYPE_OFE_NON_RT;
			break;
		case CAM_ICP_RES_TYPE_OFE_SEMI_RT:
			handle_type = HFI_OFE_HANDLE_TYPE_OFE_SEMI_RT;
			break;
		default:
			CAM_ERR(CAM_ICP, "Invalid OFE stream type: %u", dev_type);
			return -EINVAL;
		}
	} else {
		create_handle.pkt_type = HFI_CMD_IPEBPS_CREATE_HANDLE;
		switch (dev_type) {
		case CAM_ICP_RES_TYPE_BPS:
			handle_type = HFI_IPEBPS_HANDLE_TYPE_BPS_NON_RT;
			break;
		case CAM_ICP_RES_TYPE_IPE_RT:
			handle_type = HFI_IPEBPS_HANDLE_TYPE_IPE_RT;
			break;
		case CAM_ICP_RES_TYPE_IPE:
			handle_type = HFI_IPEBPS_HANDLE_TYPE_IPE_NON_RT;
			break;
		case CAM_ICP_RES_TYPE_IPE_SEMI_RT:
			handle_type = HFI_IPEBPS_HANDLE_TYPE_IPE_SEMI_RT;
			break;
		case CAM_ICP_RES_TYPE_BPS_RT:
			handle_type = HFI_IPEBPS_HANDLE_TYPE_BPS_RT;
			break;
		case CAM_ICP_RES_TYPE_BPS_SEMI_RT:
			handle_type = HFI_IPEBPS_HANDLE_TYPE_BPS_SEMI_RT;
			break;
		default:
			CAM_ERR(CAM_ICP, "Invalid IPE/BPS stream type: %u", dev_type);
			return -EINVAL;
		}
	}
	task = cam_req_mgr_workq_get_task(hw_mgr->cmd_work);
	if (!task)
		return -ENOMEM;

	create_handle.size = sizeof(struct hfi_cmd_create_handle);
	create_handle.handle_type = handle_type;
	create_handle.user_data1 = PTR_TO_U64(ctx_data);
	reinit_completion(&ctx_data->wait_complete);
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)&create_handle;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;
	rc = cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc)
		return rc;

	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
			&ctx_data->wait_complete,
			msecs_to_jiffies(timeout), CAM_ICP,
			"%s: FW response timeout for create handle command",
			ctx_data->ctx_id_string);
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		cam_icp_dump_debug_info(hw_mgr, false);
	}

	if (ctx_data->fw_handle == 0) {
		CAM_ERR(CAM_ICP, "%s: Invalid handle created", ctx_data->ctx_id_string);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_icp_mgr_send_ping(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	struct hfi_cmd_ping_pkt ping_pkt;
	struct hfi_cmd_work_data *task_data;
	unsigned long rem_jiffies;
	int timeout = 5000;
	struct crm_workq_task *task;
	int rc = 0;

	task = cam_req_mgr_workq_get_task(hw_mgr->cmd_work);
	if (!task) {
		CAM_ERR(CAM_ICP, "%s: No free task to send ping command",
			ctx_data->ctx_id_string);
		return -ENOMEM;
	}

	ping_pkt.size = sizeof(struct hfi_cmd_ping_pkt);
	ping_pkt.pkt_type = HFI_CMD_SYS_PING;
	ping_pkt.user_data = PTR_TO_U64(ctx_data);
	init_completion(&ctx_data->wait_complete);
	task_data = (struct hfi_cmd_work_data *)task->payload;
	task_data->data = (void *)&ping_pkt;
	task_data->request_id = 0;
	task_data->type = ICP_WORKQ_TASK_CMD_TYPE;
	task->process_cb = cam_icp_mgr_process_cmd;

	rc = cam_req_mgr_workq_enqueue_task(task, hw_mgr,
		CRM_TASK_PRIORITY_0);
	if (rc)
		return rc;

	rem_jiffies = CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(
			&ctx_data->wait_complete,
			msecs_to_jiffies(timeout), CAM_ICP,
			"%s: FW response timeout for Ping handle command",
			ctx_data->ctx_id_string);
	if (!rem_jiffies) {
		rc = -ETIMEDOUT;
		cam_icp_dump_debug_info(hw_mgr, false);
	}

	return rc;
}

static int cam_icp_get_acquire_info(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_hw_acquire_args *args,
	struct cam_icp_hw_ctx_data *ctx_data)
{
	int i;
	int acquire_size;
	struct cam_icp_acquire_dev_info icp_dev_acquire_info;
	struct cam_icp_res_info *p_icp_out = NULL;

	if (copy_from_user(&icp_dev_acquire_info,
		(void __user *)args->acquire_info,
		sizeof(struct cam_icp_acquire_dev_info))) {
		CAM_ERR(CAM_ICP, "%s: Failed in acquire", ctx_data->ctx_id_string);
		return -EFAULT;
	}

	if (icp_dev_acquire_info.secure_mode > CAM_SECURE_MODE_SECURE) {
		CAM_ERR(CAM_ICP, "%s: Invalid mode: %d",
			ctx_data->ctx_id_string, icp_dev_acquire_info.secure_mode);
		return -EINVAL;
	}

	if ((icp_dev_acquire_info.num_out_res > ICP_MAX_OUTPUT_SUPPORTED) ||
		(icp_dev_acquire_info.num_out_res <= 0)) {
		CAM_ERR(CAM_ICP, "%s: Invalid num of out resources: %u",
			ctx_data->ctx_id_string, icp_dev_acquire_info.num_out_res);
		return -EINVAL;
	}

	if (icp_dev_acquire_info.dev_type < CAM_ICP_RES_TYPE_BPS ||
		icp_dev_acquire_info.dev_type > CAM_ICP_RES_TYPE_OFE_SEMI_RT) {
		CAM_ERR(CAM_ICP, "%s Invalid device type",
			ctx_data->ctx_id_string);
		return -EFAULT;
	}

	acquire_size = sizeof(struct cam_icp_acquire_dev_info) +
		((icp_dev_acquire_info.num_out_res - 1) *
		sizeof(struct cam_icp_res_info));
	ctx_data->icp_dev_acquire_info = kzalloc(acquire_size, GFP_KERNEL);
	if (!ctx_data->icp_dev_acquire_info)
		return -ENOMEM;

	if (copy_from_user(ctx_data->icp_dev_acquire_info,
		(void __user *)args->acquire_info, acquire_size)) {
		CAM_ERR(CAM_ICP, "%s: Failed in acquire: size = %d",
			ctx_data->ctx_id_string, acquire_size);
		kfree(ctx_data->icp_dev_acquire_info);
		ctx_data->icp_dev_acquire_info = NULL;
		return -EFAULT;
	}

	CAM_DBG(CAM_ICP, "%s: %x %x %x %x %x %x",
		ctx_data->ctx_id_string,
		ctx_data->icp_dev_acquire_info->in_res.format,
		ctx_data->icp_dev_acquire_info->in_res.width,
		ctx_data->icp_dev_acquire_info->in_res.height,
		ctx_data->icp_dev_acquire_info->in_res.fps,
		ctx_data->icp_dev_acquire_info->num_out_res,
		ctx_data->icp_dev_acquire_info->scratch_mem_size);

	p_icp_out = ctx_data->icp_dev_acquire_info->out_res;
	for (i = 0; i < icp_dev_acquire_info.num_out_res; i++)
		CAM_DBG(CAM_ICP, "%s: out[i] %x %x %x %x",
			ctx_data->ctx_id_string,
			p_icp_out[i].format,
			p_icp_out[i].width,
			p_icp_out[i].height,
			p_icp_out[i].fps);

	return 0;
}

static inline enum cam_icp_hw_type cam_icp_get_hw_dev_type(uint32_t dev_type)
{
	switch (dev_type) {
	case CAM_ICP_RES_TYPE_BPS:
	case CAM_ICP_RES_TYPE_BPS_RT:
	case CAM_ICP_RES_TYPE_BPS_SEMI_RT:
		return CAM_ICP_DEV_BPS;
	case CAM_ICP_RES_TYPE_IPE:
	case CAM_ICP_RES_TYPE_IPE_RT:
	case CAM_ICP_RES_TYPE_IPE_SEMI_RT:
		return CAM_ICP_DEV_IPE;
	case CAM_ICP_RES_TYPE_OFE:
	case CAM_ICP_RES_TYPE_OFE_RT:
	case CAM_ICP_RES_TYPE_OFE_SEMI_RT:
		return CAM_ICP_DEV_OFE;
	default:
		CAM_ERR(CAM_ICP, "Invalid resource type: %u", dev_type);
	}

	return CAM_ICP_HW_MAX;
}

static int cam_icp_mgr_acquire_hw(void *hw_mgr_priv, void *acquire_hw_args)
{
	int rc = 0, bitmap_size = 0, i;
	uint32_t ctx_id = 0;
	dma_addr_t io_buf_addr;
	size_t io_buf_size;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_icp_hw_ctx_data *ctx_data = NULL;
	struct cam_hw_acquire_args *args = acquire_hw_args;
	struct cam_icp_acquire_dev_info *icp_dev_acquire_info;
	struct cam_cmd_mem_regions cmd_mem_region;
	enum cam_icp_hw_type hw_dev_type;

	if ((!hw_mgr_priv) || (!acquire_hw_args)) {
		CAM_ERR(CAM_ICP, "Invalid params: %pK %pK", hw_mgr_priv,
			acquire_hw_args);
		return -EINVAL;
	}

	if (args->num_acq > 1) {
		CAM_ERR(CAM_ICP, "[%s] number of resources are wrong: %u",
			hw_mgr->hw_mgr_name, args->num_acq);
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "[%s] ENTER", hw_mgr->hw_mgr_name);
	mutex_lock(&hw_mgr->hw_mgr_mutex);
	ctx_id = cam_icp_mgr_get_free_ctx(hw_mgr);
	if (ctx_id >= CAM_ICP_CTX_MAX) {
		CAM_ERR(CAM_ICP, "[%s] No free ctx space", hw_mgr->hw_mgr_name);
		mutex_unlock(&hw_mgr->hw_mgr_mutex);
		return -ENOSPC;
	}
	ctx_data = &hw_mgr->ctx_data[ctx_id];
	ctx_data->ctx_id = ctx_id;
	ctx_data->hw_mgr_priv = hw_mgr_priv;

	mutex_lock(&ctx_data->ctx_mutex);
	rc = cam_icp_get_acquire_info(hw_mgr, args, ctx_data);
	if (rc)
		goto acquire_info_failed;

	icp_dev_acquire_info = ctx_data->icp_dev_acquire_info;
	hw_dev_type = cam_icp_get_hw_dev_type(icp_dev_acquire_info->dev_type);
	if (!CAM_ICP_IS_VALID_HW_DEV_TYPE(hw_dev_type)) {
		CAM_ERR(CAM_ICP, "[%s] Fail to get hw device type from dev type: %u",
			hw_mgr->hw_mgr_name, icp_dev_acquire_info->dev_type);
		rc = -EINVAL;
		goto get_io_buf_failed;
	}

	if (!CAM_ICP_IS_DEV_HW_EXIST(hw_mgr->hw_cap_mask, hw_dev_type)) {
		CAM_ERR(CAM_ICP,
			"Attempt to acquire device %s not supported by [%s]",
			cam_icp_dev_type_to_name(icp_dev_acquire_info->dev_type),
			hw_mgr->hw_mgr_name);
		rc = -EINVAL;
		goto get_io_buf_failed;
	}

	ctx_data->device_info = &hw_mgr->dev_info[hw_mgr->dev_info_idx[hw_dev_type]];

	scnprintf(ctx_data->ctx_id_string, sizeof(ctx_data->ctx_id_string),
		"[%s]_%s_ctx[%d]_hwmgr_ctx[%d]",
		hw_mgr->hw_mgr_name,
		cam_icp_dev_type_to_name(
		ctx_data->icp_dev_acquire_info->dev_type),
		args->ctx_id, ctx_data->ctx_id);

	CAM_DBG(CAM_ICP, "%s: acquire io buf handle %d",
		ctx_data->ctx_id_string, icp_dev_acquire_info->io_config_cmd_handle);
	/* FW/CDM buffers are expected to be mapped in 32-bit address range */
	rc = cam_mem_get_io_buf(
		icp_dev_acquire_info->io_config_cmd_handle,
		hw_mgr->iommu_hdl,
		&io_buf_addr, &io_buf_size, NULL, NULL);
	if (rc) {
		CAM_ERR(CAM_ICP, "%s: unable to get src buf info from io desc",
			ctx_data->ctx_id_string);
		goto get_io_buf_failed;
	}

	CAM_DBG(CAM_ICP, "%s: hdl: %d, addr: %pK, size: %zu",
		ctx_data->ctx_id_string, icp_dev_acquire_info->io_config_cmd_handle,
		(void *)io_buf_addr, io_buf_size);

	if (!hw_mgr->ctxt_cnt) {
		rc = cam_icp_clk_info_init(hw_mgr);
		if (rc)
			goto get_io_buf_failed;

		rc = cam_icp_mgr_icp_resume(hw_mgr);
		if (rc)
			goto get_io_buf_failed;

		if (hw_mgr->icp_debug_type)
			hfi_set_debug_level(hw_mgr->hfi_handle, hw_mgr->icp_debug_type,
				hw_mgr->icp_dbg_lvl);

		hfi_set_fw_dump_levels(hw_mgr->hfi_handle, hw_mgr->icp_fw_dump_lvl,
			hw_mgr->icp_fw_ramdump_lvl);

		rc = cam_icp_send_ubwc_cfg(hw_mgr);
		if (rc)
			goto ubwc_cfg_failed;
	}

	rc = cam_icp_mgr_device_resume(hw_mgr, ctx_data);
	if (rc)
		goto icp_dev_resume_failed;

	rc = cam_icp_mgr_send_ping(hw_mgr, ctx_data);
	if (rc) {
		CAM_ERR(CAM_ICP, "%s: ping ack not received", ctx_data->ctx_id_string);
		goto send_ping_failed;
	}
	CAM_DBG(CAM_ICP, "%s: ping ack received", ctx_data->ctx_id_string);

	rc = cam_icp_mgr_create_handle(hw_mgr, icp_dev_acquire_info->dev_type,
		ctx_data);
	if (rc) {
		CAM_ERR(CAM_ICP, "%s: create handle failed", ctx_data->ctx_id_string);
		goto create_handle_failed;
	}

	CAM_DBG(CAM_ICP,
		"%s: created stream handle",
		ctx_data->ctx_id_string);

	cmd_mem_region.num_regions = 1;
	cmd_mem_region.map_info_array[0].mem_handle =
		icp_dev_acquire_info->io_config_cmd_handle;
	cmd_mem_region.map_info_array[0].offset = 0;
	cmd_mem_region.map_info_array[0].size =
		icp_dev_acquire_info->io_config_cmd_size;
	cmd_mem_region.map_info_array[0].flags = 0;

	rc = cam_icp_process_stream_settings(ctx_data,
		&cmd_mem_region, true);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"%s: sending config io mapping failed rc %d",
				ctx_data->ctx_id_string, rc);
		goto send_map_info_failed;
	}

	if (cam_presil_mode_enabled()) {
		CAM_INFO(CAM_PRESIL, "%s: Sending IO Config buffers to presil: FD %d ",
			ctx_data->ctx_id_string,
			(icp_dev_acquire_info->io_config_cmd_handle >> 16));
		cam_mem_mgr_send_buffer_to_presil(hw_mgr->iommu_hdl,
			icp_dev_acquire_info->io_config_cmd_handle);
	}

	rc = cam_icp_mgr_send_config_io(ctx_data, io_buf_addr);
	if (rc) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"%s: IO Config command failed %d size:%d",
			ctx_data->ctx_id_string, rc, icp_dev_acquire_info->io_config_cmd_size);
		cam_icp_dump_io_cfg(ctx_data,
			icp_dev_acquire_info->io_config_cmd_handle,
			icp_dev_acquire_info->io_config_cmd_size);
		goto ioconfig_failed;
	}

	rc = cam_icp_process_stream_settings(ctx_data,
		&cmd_mem_region, false);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"%s: sending config io unmapping failed %d",
			ctx_data->ctx_id_string, rc);
		goto send_map_info_failed;
	}

	ctx_data->context_priv = args->context_data;
	args->ctxt_to_hw_map = ctx_data;
	args->hw_mgr_ctx_id = ctx_data->ctx_id;

	bitmap_size = BITS_TO_LONGS(CAM_FRAME_CMD_MAX) * sizeof(long);
	ctx_data->hfi_frame_process.bitmap =
			kzalloc(bitmap_size, GFP_KERNEL);
	if (!ctx_data->hfi_frame_process.bitmap) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"%s: failed to allocate hfi frame bitmap", ctx_data->ctx_id_string);
			rc = -ENOMEM;
		goto ioconfig_failed;
	}

	ctx_data->perf_stats.total_resp_time = 0;
	ctx_data->perf_stats.total_requests = 0;

	ctx_data->hfi_frame_process.bits = bitmap_size * BITS_PER_BYTE;
	hw_mgr->ctx_data[ctx_id].ctxt_event_cb = args->event_cb;
	icp_dev_acquire_info->scratch_mem_size = ctx_data->scratch_mem_size;

	if (copy_to_user((void __user *)args->acquire_info,
		icp_dev_acquire_info,
		sizeof(struct cam_icp_acquire_dev_info))) {
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"%s: copy from user failed", ctx_data->ctx_id_string);
		goto copy_to_user_failed;
	}

	cam_icp_ctx_clk_info_init(ctx_data);
	ctx_data->state = CAM_ICP_CTX_STATE_ACQUIRED;
	mutex_unlock(&ctx_data->ctx_mutex);
	CAM_DBG(CAM_ICP, "%s: scratch size = %x fw_handle = %x",
		ctx_data->ctx_id_string,
		(unsigned int)icp_dev_acquire_info->scratch_mem_size,
		(unsigned int)ctx_data->fw_handle);

	/* Start device timer*/
	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		if (hw_mgr->dev_info[i].dev_ctx_info.dev_ctxt_cnt == 1)
			break;
	}

	if (i != hw_mgr->num_dev_info)
		cam_icp_device_timer_start(hw_mgr);

	/* Start context timer*/
	cam_icp_ctx_timer_start(ctx_data);
	hw_mgr->ctxt_cnt++;
	mutex_unlock(&hw_mgr->hw_mgr_mutex);

	CAM_DBG(CAM_ICP, "%s: Acquire Done",
		ctx_data->ctx_id_string);

	CAM_TRACE(CAM_ICP,
		"%s: Acquired, in_res : format=%d, widht=%d, height=%d, fps=%d",
		ctx_data->ctx_id_string,
		ctx_data->icp_dev_acquire_info->in_res.format,
		ctx_data->icp_dev_acquire_info->in_res.width,
		ctx_data->icp_dev_acquire_info->in_res.height,
		ctx_data->icp_dev_acquire_info->in_res.fps);

	if (ctx_data->icp_dev_acquire_info->num_out_res > 0) {
		CAM_TRACE(CAM_ICP,
			"%s: Acquired, out_res[0] : format=%d, widht=%d, height=%d, fps=%d",
			ctx_data->ctx_id_string,
			ctx_data->icp_dev_acquire_info->out_res[0].format,
			ctx_data->icp_dev_acquire_info->out_res[0].width,
			ctx_data->icp_dev_acquire_info->out_res[0].height,
			ctx_data->icp_dev_acquire_info->out_res[0].fps);
	}

	if (ctx_data->icp_dev_acquire_info->num_out_res > 1) {
		CAM_TRACE(CAM_ICP,
			"%s: Acquired, out_res[1] : format=%d, widht=%d, height=%d, fps=%d",
			ctx_data->ctx_id_string,
			ctx_data->icp_dev_acquire_info->out_res[1].format,
			ctx_data->icp_dev_acquire_info->out_res[1].width,
			ctx_data->icp_dev_acquire_info->out_res[1].height,
			ctx_data->icp_dev_acquire_info->out_res[1].fps);
	}

	return 0;

copy_to_user_failed:
	kfree(ctx_data->hfi_frame_process.bitmap);
	ctx_data->hfi_frame_process.bitmap = NULL;
ioconfig_failed:
	cam_icp_process_stream_settings(ctx_data,
		&cmd_mem_region, false);
send_map_info_failed:
	cam_icp_mgr_destroy_handle(ctx_data);
create_handle_failed:
send_ping_failed:
	cam_icp_mgr_dev_power_collapse(hw_mgr, ctx_data, 0);
icp_dev_resume_failed:
ubwc_cfg_failed:
	if (!hw_mgr->ctxt_cnt)
		cam_icp_mgr_icp_power_collapse(hw_mgr);
get_io_buf_failed:
	kfree(hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info);
	hw_mgr->ctx_data[ctx_id].icp_dev_acquire_info = NULL;
	hw_mgr->ctx_data[ctx_id].device_info = NULL;
acquire_info_failed:
	cam_icp_mgr_put_ctx(ctx_data);
	cam_icp_mgr_process_dbg_buf(hw_mgr);
	mutex_unlock(&ctx_data->ctx_mutex);
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_mgr_get_hw_caps(void *hw_mgr_priv, void *hw_caps_args)
{
	int rc = 0;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_query_cap_cmd *query_cap = hw_caps_args;
	struct cam_icp_query_cap_cmd icp_caps;

	if ((!hw_mgr_priv) || (!hw_caps_args)) {
		CAM_ERR(CAM_ICP, "Invalid params: %pK %pK", hw_mgr_priv, hw_caps_args);
		return -EINVAL;
	}

	if (sizeof(struct cam_icp_query_cap_cmd) != query_cap->size) {
		CAM_ERR(CAM_ICP,
			"[%s] Input query cap size:%u does not match expected query cap size: %u",
			hw_mgr->hw_mgr_name, query_cap->size,
			sizeof(struct cam_icp_query_cap_cmd));
		return -EFAULT;
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	if (copy_from_user(&icp_caps,
		u64_to_user_ptr(query_cap->caps_handle),
		sizeof(struct cam_icp_query_cap_cmd))) {
		CAM_ERR(CAM_ICP, "[%s] copy from_user failed", hw_mgr->hw_mgr_name);
		rc = -EFAULT;
		goto end;
	}

	rc = hfi_get_hw_caps(&icp_caps);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Fail to get hfi caps", hw_mgr->hw_mgr_name);
		goto end;
	}

	icp_caps.dev_iommu_handle.non_secure = hw_mgr->iommu_hdl;
	icp_caps.dev_iommu_handle.secure = hw_mgr->iommu_sec_hdl;

	if (copy_to_user(u64_to_user_ptr(query_cap->caps_handle),
		&icp_caps, sizeof(struct cam_icp_query_cap_cmd))) {
		CAM_ERR(CAM_ICP, "[%s] copy_to_user failed", hw_mgr->hw_mgr_name);
		rc = -EFAULT;
	}
end:
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static int cam_icp_mgr_get_hw_caps_v2(void *hw_mgr_priv, void *hw_caps_args)
{
	int rc = 0;
	struct cam_icp_hw_mgr *hw_mgr = hw_mgr_priv;
	struct cam_icp_hw_device_info *dev_info;
	struct cam_query_cap_cmd *query_cap = hw_caps_args;
	struct cam_icp_query_cap_cmd_v2 query_cmd;
	uint32_t supported_hw_dev, num_supported_device = 0;
	int i;

	if ((!hw_mgr_priv) || (!hw_caps_args)) {
		CAM_ERR(CAM_ICP, "Invalid params: %pK %pK",
			hw_mgr_priv, hw_caps_args);
		return -EINVAL;
	}

	if (sizeof(struct cam_icp_query_cap_cmd_v2) != query_cap->size) {
		CAM_ERR(CAM_ICP,
			"[%s] Input query cap size:%u does not match expected query cap size: %ud",
			hw_mgr->hw_mgr_name, query_cap->size,
			sizeof(struct cam_icp_query_cap_cmd_v2));
		return -EFAULT;
	}

	mutex_lock(&hw_mgr->hw_mgr_mutex);
	if (copy_from_user(&query_cmd, u64_to_user_ptr(query_cap->caps_handle),
		sizeof(struct cam_icp_query_cap_cmd_v2))) {
		CAM_ERR(CAM_ICP, "[%s] copy from user failed", hw_mgr->hw_mgr_name);
		rc = -EFAULT;
		goto end;
	}

	memset(&query_cmd.dev_info, 0,
		(CAM_ICP_MAX_NUM_OF_DEV_TYPES * sizeof(struct cam_icp_device_info)));

	for (i = 0; i < CAM_ICP_MAX_ICP_HW_TYPE; i++) {
		if (!CAM_ICP_IS_DEV_HW_EXIST(hw_mgr->hw_cap_mask, i))
			continue;

		query_cmd.dev_info[num_supported_device].dev_type = CAM_ICP_DEV_TYPE_ICP;
		query_cmd.dev_info[num_supported_device].num_devices = CAM_ICP_MAX_ICP_PROC_PER_DEV;
		num_supported_device++;
		break;
	}

	if (!num_supported_device) {
		CAM_ERR(CAM_ICP, "No ICP HW binded to %s", hw_mgr->hw_mgr_name);
		rc = -ENODEV;
		goto end;
	}

	for (i = 0; i < hw_mgr->num_dev_info; i++) {
		dev_info = &hw_mgr->dev_info[i];

		switch (dev_info->hw_dev_type) {
		case CAM_ICP_DEV_IPE:
			supported_hw_dev = CAM_ICP_DEV_TYPE_IPE;
			break;
		case CAM_ICP_DEV_BPS:
			supported_hw_dev = CAM_ICP_DEV_TYPE_BPS;
			break;
		case CAM_ICP_DEV_OFE:
			supported_hw_dev = CAM_ICP_DEV_TYPE_OFE;
			break;
		default:
			CAM_ERR(CAM_ICP, "[%s] Invalid icp hw type: %u",
				hw_mgr->hw_mgr_name, i);
			rc = -EBADSLT;
			goto end;
		}

		if (num_supported_device >= CAM_ICP_MAX_NUM_OF_DEV_TYPES) {
			CAM_ERR(CAM_ICP,
				"[%s] number of supported hw devices:%u exceeds maximum number of devices supported by query cap struct: %u",
				hw_mgr->hw_mgr_name, num_supported_device,
				CAM_ICP_MAX_NUM_OF_DEV_TYPES);
			rc = -EFAULT;
			goto end;
		}

		query_cmd.dev_info[num_supported_device].dev_type = supported_hw_dev;
		query_cmd.dev_info[num_supported_device].num_devices = dev_info->hw_dev_cnt;

		num_supported_device++;
	}

	query_cmd.num_dev_info = num_supported_device;

	rc = hfi_get_hw_caps_v2(hw_mgr->hfi_handle, &query_cmd);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Fail to get hfi caps rc:%d",
			hw_mgr->hw_mgr_name, rc);
		goto end;
	}

	query_cmd.dev_iommu_handle.non_secure = hw_mgr->iommu_hdl;
	query_cmd.dev_iommu_handle.secure = hw_mgr->iommu_sec_hdl;

	if (copy_to_user(u64_to_user_ptr(query_cap->caps_handle),
		&query_cmd, sizeof(struct cam_icp_query_cap_cmd_v2))) {
		CAM_ERR(CAM_ICP, "[%s] copy_to_user failed", hw_mgr->hw_mgr_name);
		rc = -EFAULT;
	}
end:
	mutex_unlock(&hw_mgr->hw_mgr_mutex);
	return rc;
}

static void cam_icp_mgr_free_hw_devs(struct cam_icp_hw_mgr *hw_mgr)
{
	int i;

	for (i = 0; i < hw_mgr->num_dev_info; i++)
		kfree(hw_mgr->dev_info[i].dev_intf);

	kfree(hw_mgr->dev_info);
	hw_mgr->dev_info = NULL;
	hw_mgr->icp_dev_intf = NULL;
}

static int cam_icp_mgr_verify_hw_caps(struct cam_icp_hw_mgr *hw_mgr, uint32_t *icp_dev_mask,
	uint32_t num_icp_dev_mask)
{
	struct cam_cpas_query_cap query;
	int rc, i;
	uint32_t *cam_caps, num_cpas_cap_mask;

	rc = cam_cpas_get_hw_info(&query.camera_family, &query.camera_version,
			&query.cpas_version, &cam_caps, &num_cpas_cap_mask,
			NULL, NULL);
	if (rc)
		return rc;

	if (num_icp_dev_mask > num_cpas_cap_mask) {
		CAM_ERR(CAM_ICP,
			"[%s] Number of found icp device caps mask %u exceeds cpas cap mask %u",
			hw_mgr->hw_mgr_name, num_icp_dev_mask, num_cpas_cap_mask);
		return -EINVAL;
	}

	for (i = 0; i < num_icp_dev_mask; i++) {
		if ((icp_dev_mask[i] & cam_caps[i]) != icp_dev_mask[i]) {
			CAM_ERR(CAM_ICP,
				"[%s] Found unsupported HW, cpas caps mask: %u icp device mask: %u",
				hw_mgr->hw_mgr_name, cam_caps[i], icp_dev_mask[i]);
			return -ENODEV;
		}
	}

	return 0;
}

static int cam_icp_mgr_alloc_devs(struct device_node *np, struct cam_icp_hw_mgr *hw_mgr,
	struct cam_hw_intf ***devices, uint32_t *hw_dev_cnt)
{
	struct cam_hw_intf **alloc_devices = NULL;
	int rc, i;
	enum cam_icp_hw_type icp_hw_type;
	uint32_t num = 0, num_cpas_mask = 0, cpas_hw_mask[MAX_HW_CAPS_MASK] = {0};

	rc = cam_icp_alloc_processor_devs(np, &icp_hw_type, &alloc_devices, hw_dev_cnt);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] proc devices allocation failed rc=%d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	if (!CAM_ICP_IS_VALID_HW_DEV_TYPE(icp_hw_type)) {
		CAM_ERR(CAM_ICP, "[%s] Invalid hw dev type: %u",
			hw_mgr->hw_mgr_name, icp_hw_type);
		rc = -EINVAL;
		kfree(devices);
		kfree(alloc_devices);
		return rc;
	}

	if (hw_dev_cnt[icp_hw_type] > CAM_ICP_MAX_ICP_PROC_PER_DEV) {
		CAM_ERR(CAM_ICP,
			"Invalid number of ICP device: %u allocated exceeds ICP device supported per hw mgr: %u",
			hw_dev_cnt[icp_hw_type], CAM_ICP_MAX_ICP_PROC_PER_DEV);
		rc = -EINVAL;
		goto free_devs;
	}

	devices[icp_hw_type] = alloc_devices;
	hw_mgr->hw_cap_mask |= BIT(icp_hw_type);
	num_cpas_mask = max(num_cpas_mask, (uint32_t)(ICP_CAPS_MASK_IDX + 1));
	cpas_hw_mask[ICP_CAPS_MASK_IDX] |= icp_cpas_mask[hw_mgr->hw_mgr_id];

	rc = of_property_read_u32(np, "num-ipe", &num);
	if (!rc) {
		alloc_devices = kcalloc(num, sizeof(*alloc_devices), GFP_KERNEL);
		if (!alloc_devices) {
			CAM_ERR(CAM_ICP, "[%s] ipe device allocation failed",
				hw_mgr->hw_mgr_name);
			goto free_devs;
		}

		hw_dev_cnt[CAM_ICP_DEV_IPE] = num;
		devices[CAM_ICP_DEV_IPE] = alloc_devices;
		hw_mgr->hw_cap_mask |= BIT(CAM_ICP_DEV_IPE);
		num_cpas_mask = max(num_cpas_mask, (uint32_t)(IPE_CAPS_MASK_IDX + 1));
		cpas_hw_mask[IPE_CAPS_MASK_IDX] |= CPAS_TITAN_IPE0_CAP_BIT;
	}

	rc = of_property_read_u32(np, "num-bps", &num);
	if (!rc) {
		alloc_devices = kcalloc(num, sizeof(*alloc_devices), GFP_KERNEL);
		if (!alloc_devices) {
			CAM_ERR(CAM_ICP, "[%s] bps device allocation failed",
				hw_mgr->hw_mgr_name);
			goto free_devs;
		}

		hw_dev_cnt[CAM_ICP_DEV_BPS] = num;
		devices[CAM_ICP_DEV_BPS] = alloc_devices;
		hw_mgr->hw_cap_mask |= BIT(CAM_ICP_DEV_BPS);
		num_cpas_mask = max(num_cpas_mask, (uint32_t)(BPS_CAPS_MASK_IDX + 1));
		cpas_hw_mask[BPS_CAPS_MASK_IDX] |= CPAS_BPS_BIT;
	}

	rc = of_property_read_u32(np, "num-ofe", &num);
	if (!rc) {
		alloc_devices = kcalloc(num, sizeof(*alloc_devices), GFP_KERNEL);
		if (!alloc_devices) {
			CAM_ERR(CAM_ICP, "[%s] OFE device allocation failed",
				hw_mgr->hw_mgr_name);
			goto free_devs;
		}

		hw_dev_cnt[CAM_ICP_DEV_OFE] = num;
		devices[CAM_ICP_DEV_OFE] = alloc_devices;
		hw_mgr->hw_cap_mask |= BIT(CAM_ICP_DEV_OFE);
		num_cpas_mask = max(num_cpas_mask, (uint32_t)(OFE_CAPS_MASK_IDX + 1));
		cpas_hw_mask[OFE_CAPS_MASK_IDX] |= CPAS_OFE_BIT;
	}

	rc = cam_icp_mgr_verify_hw_caps(hw_mgr, cpas_hw_mask, num_cpas_mask);
	if (rc) {
		CAM_ERR(CAM_ICP, "CPAS ICP HW capability verification fails rc=%d", rc);
		goto free_devs;
	}

	hw_mgr->dev_pc_flag = of_property_read_bool(np, "ipe_bps_pc_en");
	hw_mgr->icp_pc_flag = of_property_read_bool(np, "icp_pc_en");
	hw_mgr->icp_use_pil = of_property_read_bool(np, "icp_use_pil");
	hw_mgr->synx_signaling_en = of_property_read_bool(np, "synx_signaling_en");

	return 0;

free_devs:
	kfree(alloc_devices);
	for (i = 0; i < CAM_ICP_HW_MAX; i++)
		kfree(devices[i]);

	return rc;
}

static char *cam_icp_hw_dev_type_to_name(enum cam_icp_hw_type hw_dev_type)
{
	switch (hw_dev_type) {
	case CAM_ICP_HW_ICP_V1:
		return "ICP_V1";
	case CAM_ICP_HW_ICP_V2:
		return "ICP_V2";
	case CAM_ICP_DEV_IPE:
		return "IPE";
	case CAM_ICP_DEV_BPS:
		return "BPS";
	case CAM_ICP_DEV_OFE:
		return "OFE";
	default:
		return "Invalid hw dev type";
	}
}

static int cam_icp_mgr_set_up_dev_info(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_hw_intf ***devices, uint32_t *hw_dev_cnt)
{
	int i, j;

	hw_mgr->icp_dev_intf = CAM_ICP_GET_PROC_DEV_INTF(devices);
	if (!hw_mgr->icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] Invalid ICP dev interface is NULL",
			hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	for (i = CAM_ICP_DEV_START_IDX; i < CAM_ICP_HW_MAX; i++) {
		if (hw_dev_cnt[i] > 0)
			hw_mgr->num_dev_info++;
	}

	hw_mgr->dev_info = kcalloc(hw_mgr->num_dev_info, sizeof(struct cam_icp_hw_device_info),
		GFP_KERNEL);
	if (!hw_mgr->dev_info)
		return -ENOMEM;

	for (i = CAM_ICP_DEV_START_IDX, j = 0; i < CAM_ICP_HW_MAX; i++) {
		if (hw_dev_cnt[i] > 0) {
			hw_mgr->dev_info_idx[i] = j;
			hw_mgr->dev_info[j].dev_intf = devices[i];
			hw_mgr->dev_info[j].dev_name = cam_icp_hw_dev_type_to_name(i);
			hw_mgr->dev_info[j].hw_dev_type = i;
			hw_mgr->dev_info[j].hw_dev_cnt = hw_dev_cnt[i];
			j++;
		} else {
			hw_mgr->dev_info_idx[i] = -1;
		}
	}

	return 0;
}

static int cam_icp_mgr_init_devs(struct device_node *np, struct cam_icp_hw_mgr *hw_mgr)
{
	int rc, i, j, count, num_hw;
	struct cam_hw_intf **devices[CAM_ICP_HW_MAX] = {0};
	uint32_t hw_dev_cnt[CAM_ICP_HW_MAX] = {0};

	rc = cam_icp_mgr_alloc_devs(np, hw_mgr, devices, hw_dev_cnt);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] devices allocation failed rc=%d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	count = of_property_count_strings(np, "compat-hw-name");
	if (count < 0) {
		CAM_ERR(CAM_ICP, "[%s] Invalid compat-hw-name count=%d", hw_mgr->hw_mgr_name,
			count);
		rc = count;
		goto free_devices;
	}

	for (i = 0; i < count; i++) {
		const char *name = NULL;
		struct platform_device *pdev;
		struct device_node *node;
		struct cam_hw_intf *iface;

		rc = of_property_read_string_index(np, "compat-hw-name",
			i, &name);
		if (rc) {
			CAM_ERR(CAM_ICP,
				"[%s] unable to get property name: idx=%d rc=%d",
				hw_mgr->hw_mgr_name, i, rc);
			goto free_devices;
		}

		node = of_find_node_by_name(NULL, name);
		if (!node) {
			CAM_ERR(CAM_ICP, "[%s] missing node %s",
				hw_mgr->hw_mgr_name, name);
			rc = -ENODEV;
			goto free_devices;
		}

		pdev = of_find_device_by_node(node);
		of_node_put(node);
		if (!pdev) {
			CAM_ERR(CAM_ICP,
				"[%s] platform device not found for %s",
				hw_mgr->hw_mgr_name, name);
			rc = -ENODEV;
			goto free_devices;
		}

		iface = platform_get_drvdata(pdev);
		if (!iface || !iface->hw_ops.process_cmd) {
			CAM_ERR(CAM_ICP,
				"[%s] invalid interface: iface=%pK process_cmd=%pK",
				hw_mgr->hw_mgr_name, iface,
				(iface ? iface->hw_ops.process_cmd : NULL));

			rc = -EINVAL;
			goto free_devices;
		}

		if (!CAM_ICP_IS_VALID_HW_DEV_TYPE(iface->hw_type)) {
			CAM_ERR(CAM_ICP, "[%s] Invalid HW type:%u",
				hw_mgr->hw_mgr_name, iface->hw_type);
			rc = -EINVAL;
			goto free_devices;
		}

		num_hw = iface->hw_idx + 1;
		for (j = 0; j < num_hw; j++) {
			if (!devices[iface->hw_type][j]) {
				devices[iface->hw_type][j] = iface;
				break;
			}
		}
	}

	rc = cam_icp_mgr_set_up_dev_info(hw_mgr, devices, hw_dev_cnt);
	if (rc) {
		CAM_ERR(CAM_ICP, "Failed to set up hw device info rc=%d", rc);
		goto free_devices;
	}

	return 0;

free_devices:
	for (i = 0; i < CAM_ICP_HW_MAX; i++)
		kfree(devices[i]);

	return rc;
}

static void cam_req_mgr_process_workq_icp_command_queue(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static void cam_req_mgr_process_workq_icp_message_queue(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static void cam_req_mgr_process_workq_icp_timer_queue(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static int cam_icp_mgr_create_wq(struct cam_icp_hw_mgr *hw_mgr)
{
	char q_name[32];
	int rc, i;

	scnprintf(q_name, sizeof(q_name), "%s_command_queue", hw_mgr->hw_mgr_name);
	rc = cam_req_mgr_workq_create("icp_command_queue", ICP_WORKQ_NUM_TASK,
		&hw_mgr->cmd_work, CRM_WORKQ_USAGE_NON_IRQ, 0,
		cam_req_mgr_process_workq_icp_command_queue);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] unable to create a command worker for %s",
			hw_mgr->hw_mgr_name, q_name);
		goto cmd_work_failed;
	}

	scnprintf(q_name, sizeof(q_name), "%s_message_queue", hw_mgr->hw_mgr_name);
	rc = cam_req_mgr_workq_create(q_name, ICP_WORKQ_NUM_TASK,
		&hw_mgr->msg_work, CRM_WORKQ_USAGE_IRQ, 0,
		cam_req_mgr_process_workq_icp_message_queue);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] unable to create a message worker for %s",
			hw_mgr->hw_mgr_name, q_name);
		goto msg_work_failed;
	}

	scnprintf(q_name, sizeof(q_name), "%s_timer_queue", hw_mgr->hw_mgr_name);
	rc = cam_req_mgr_workq_create(q_name, ICP_WORKQ_NUM_TASK,
		&hw_mgr->timer_work, CRM_WORKQ_USAGE_IRQ, 0,
		cam_req_mgr_process_workq_icp_timer_queue);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] unable to create a timer worker for %s",
			hw_mgr->hw_mgr_name, q_name);
		goto timer_work_failed;
	}

	hw_mgr->cmd_work_data =
		kzalloc(sizeof(struct hfi_cmd_work_data) * ICP_WORKQ_NUM_TASK, GFP_KERNEL);
	if (!hw_mgr->cmd_work_data) {
		CAM_ERR(CAM_ICP, "[%s] Mem reservation fail for cmd_work_data",
			hw_mgr->hw_mgr_name);
		goto cmd_work_data_failed;
	}
	hw_mgr->msg_work_data =
		kzalloc(sizeof(struct hfi_msg_work_data) * ICP_WORKQ_NUM_TASK, GFP_KERNEL);
	if (!hw_mgr->msg_work_data) {
		CAM_ERR(CAM_ICP, "[%s] Mem reservation fail for msg_work_data",
			hw_mgr->hw_mgr_name);
		goto msg_work_data_failed;
	}

	hw_mgr->timer_work_data =
		kzalloc(sizeof(struct hfi_msg_work_data) * ICP_WORKQ_NUM_TASK, GFP_KERNEL);
	if (!hw_mgr->timer_work_data) {
		CAM_ERR(CAM_ICP, "[%s] Mem reservation fail for timer_work_data",
			hw_mgr->hw_mgr_name);
		goto timer_work_data_failed;
	}

	for (i = 0; i < ICP_WORKQ_NUM_TASK; i++)
		hw_mgr->msg_work->task.pool[i].payload =
			&hw_mgr->msg_work_data[i];

	for (i = 0; i < ICP_WORKQ_NUM_TASK; i++)
		hw_mgr->cmd_work->task.pool[i].payload =
			&hw_mgr->cmd_work_data[i];

	for (i = 0; i < ICP_WORKQ_NUM_TASK; i++)
		hw_mgr->timer_work->task.pool[i].payload =
			&hw_mgr->timer_work_data[i];
	return 0;

timer_work_data_failed:
	kfree(hw_mgr->msg_work_data);
msg_work_data_failed:
	kfree(hw_mgr->cmd_work_data);
cmd_work_data_failed:
	cam_req_mgr_workq_destroy(&hw_mgr->timer_work);
timer_work_failed:
	cam_req_mgr_workq_destroy(&hw_mgr->msg_work);
msg_work_failed:
	cam_req_mgr_workq_destroy(&hw_mgr->cmd_work);
cmd_work_failed:
	return rc;
}

void cam_icp_mgr_destroy_wq(struct cam_icp_hw_mgr *hw_mgr)
{
	cam_req_mgr_workq_destroy(&hw_mgr->timer_work);
	cam_req_mgr_workq_destroy(&hw_mgr->msg_work);
	cam_req_mgr_workq_destroy(&hw_mgr->cmd_work);
}

static void cam_icp_mgr_dump_pf_data(struct cam_icp_hw_mgr *hw_mgr,
	struct cam_hw_cmd_pf_args *pf_cmd_args)
{
	struct cam_packet          *packet;
	struct cam_hw_dump_pf_args *pf_args;
	int                         rc;

	pf_args = pf_cmd_args->pf_args;

	rc = cam_packet_util_get_packet_addr(&packet, pf_cmd_args->pf_req_info->packet_handle,
		pf_cmd_args->pf_req_info->packet_offset);
	if (rc)
		return;

	/*
	 * res_id_support is false since ICP doesn't have knowledge
	 * of res_id. FW submits packet to HW
	 */
	cam_packet_util_dump_io_bufs(packet, hw_mgr->iommu_hdl,
		hw_mgr->iommu_sec_hdl, pf_args, false);

	cam_packet_util_dump_patch_info(packet, hw_mgr->iommu_hdl,
		hw_mgr->iommu_sec_hdl, pf_args);

	cam_packet_util_put_packet_addr(pf_cmd_args->pf_req_info->packet_handle);
}

static int cam_icp_mgr_cmd(void *hw_mgr_priv, void *cmd_args)
{
	int rc = 0;
	struct cam_hw_cmd_args *hw_cmd_args = cmd_args;
	struct cam_icp_hw_mgr  *hw_mgr = hw_mgr_priv;

	if (!hw_mgr_priv || !cmd_args) {
		CAM_ERR(CAM_ICP, "Invalid arguments");
		return -EINVAL;
	}

	switch (hw_cmd_args->cmd_type) {
	case CAM_HW_MGR_CMD_DUMP_PF_INFO:
		cam_icp_mgr_dump_pf_data(hw_mgr, hw_cmd_args->u.pf_cmd_args);
		break;
	default:
		CAM_ERR(CAM_ICP, "[%s] Invalid cmd", hw_mgr->hw_mgr_name);
	}

	return rc;
}

static void cam_icp_mgr_inject_evt(void *hw_mgr_priv, void *evt_args)
{
	struct cam_icp_hw_ctx_data *ctx_data       = hw_mgr_priv;
	struct cam_hw_inject_evt_param *evt_params = evt_args;

	if (!ctx_data || !evt_params) {
		CAM_ERR(CAM_ICP, "Invalid params ctx data %s event params %s",
			CAM_IS_NULL_TO_STR(ctx_data), CAM_IS_NULL_TO_STR(evt_params));
		return;
	}

	memcpy(&ctx_data->evt_inject_params, evt_params, sizeof(struct cam_hw_inject_evt_param));

	ctx_data->evt_inject_params.is_valid = true;
}

static int cam_icp_mgr_register_hfi_client(struct cam_icp_hw_mgr *hw_mgr)
{

	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	int hfi_handle;
	int rc;

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP device interface is NULL",
			hw_mgr->hw_mgr_name);
		return -EINVAL;
	}

	hw_mgr->hfi_handle = HFI_HANDLE_INIT_VALUE;
	rc = cam_hfi_register(&hw_mgr->hfi_handle, hw_mgr->hw_mgr_name);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Fail to register hw mgr as hfi client rc=%d",
			hw_mgr->hw_mgr_name, rc);
		return rc;
	}

	hfi_handle = hw_mgr->hfi_handle;
	rc = icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv, CAM_ICP_CMD_SET_HFI_HANDLE,
		&hfi_handle, sizeof(hfi_handle));
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Fail to share hfi handle to ICP core rc=%d hfi hdl: %d",
			hw_mgr->hw_mgr_name, rc, hfi_handle);
		cam_hfi_unregister(&hw_mgr->hfi_handle);
		return rc;
	}

	CAM_DBG(CAM_ICP, "[%s] successfully registered as hfi client with handle: %d",
		hw_mgr->hw_mgr_name, hfi_handle);

	return 0;
}

static void cam_icp_mgr_unregister_hfi_client(struct cam_icp_hw_mgr *hw_mgr)
{
	struct cam_hw_intf *icp_dev_intf = hw_mgr->icp_dev_intf;
	int rc;

	rc = cam_hfi_unregister(&hw_mgr->hfi_handle);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Failed to unregister hfi client hdl: %d rc: %d",
			hw_mgr->hw_mgr_name, hw_mgr->hfi_handle, rc);
		return;
	}

	if (!icp_dev_intf) {
		CAM_ERR(CAM_ICP, "[%s] ICP dev intf is NULL", hw_mgr->hw_mgr_name);
		return;
	}

	icp_dev_intf->hw_ops.process_cmd(icp_dev_intf->hw_priv, CAM_ICP_CMD_SET_HFI_HANDLE,
		&hw_mgr->hfi_handle, sizeof(hw_mgr->hfi_handle));
}

static int cam_icp_mgr_get_hw_mgr_name(uint32_t device_idx, char *hw_mgr_name)
{
	if (!hw_mgr_name) {
		CAM_ERR(CAM_ICP, "Invalid output parameter hw mgr id NULL");
		return -EINVAL;
	}

	if (device_idx)
		scnprintf(hw_mgr_name, CAM_ICP_HW_MGR_NAME_SIZE, "icp%u", device_idx);
	else
		strscpy(hw_mgr_name, "icp", CAM_ICP_HW_MGR_NAME_SIZE);

	return 0;
}

int cam_icp_hw_mgr_init(struct device_node *of_node, uint64_t *hw_mgr_hdl,
	int *iommu_hdl, cam_icp_mini_dump_cb mini_dump_cb, int device_idx)
{
	int i, rc = 0;
	struct cam_icp_hw_mgr  *hw_mgr = NULL;
	struct cam_hw_mgr_intf *hw_mgr_intf;
	uint32_t size = 0;

	hw_mgr_intf = (struct cam_hw_mgr_intf *)hw_mgr_hdl;
	if (!of_node || !hw_mgr_intf) {
		CAM_ERR(CAM_ICP, "Invalid args of_node %pK hw_mgr %pK",
			of_node, hw_mgr_intf);
		return -EINVAL;
	}

	if (g_icp_hw_mgr[device_idx]) {
		CAM_ERR(CAM_ICP, "HW mgr for device idx: %u already initialized", device_idx);
		return -EPERM;
	}

	memset(hw_mgr_intf, 0, sizeof(struct cam_hw_mgr_intf));

	hw_mgr = kzalloc(sizeof(struct cam_icp_hw_mgr), GFP_KERNEL);
	if (!hw_mgr)
		return -ENOMEM;

	hw_mgr->hw_mgr_id = device_idx;

	rc = cam_icp_mgr_get_hw_mgr_name(device_idx, hw_mgr->hw_mgr_name);
	if (rc) {
		CAM_ERR(CAM_ICP, "Fail to get hw mgr name rc: %d for icp dev[%u]",
			rc, device_idx);
		goto free_hw_mgr;
	}

	CAM_DBG(CAM_ICP, "Initailize hw mgr[%u] with name: %s",
		device_idx, hw_mgr->hw_mgr_name);

	hw_mgr_intf->hw_mgr_priv = hw_mgr;
	hw_mgr_intf->hw_get_caps = cam_icp_mgr_get_hw_caps;
	hw_mgr_intf->hw_get_caps_v2 = cam_icp_mgr_get_hw_caps_v2;
	hw_mgr_intf->hw_acquire = cam_icp_mgr_acquire_hw;
	hw_mgr_intf->hw_release = cam_icp_mgr_release_hw;
	hw_mgr_intf->hw_prepare_update = cam_icp_mgr_prepare_hw_update;
	hw_mgr_intf->hw_config_stream_settings =
		cam_icp_mgr_config_stream_settings;
	hw_mgr_intf->hw_config = cam_icp_mgr_config_hw;
	hw_mgr_intf->hw_open = cam_icp_mgr_hw_open_u;
	hw_mgr_intf->hw_close = cam_icp_mgr_hw_close_u;
	hw_mgr_intf->hw_flush = cam_icp_mgr_hw_flush;
	hw_mgr_intf->hw_cmd = cam_icp_mgr_cmd;
	hw_mgr_intf->hw_dump = cam_icp_mgr_hw_dump;
	hw_mgr_intf->hw_inject_evt = cam_icp_mgr_inject_evt;
	hw_mgr->secure_mode = CAM_SECURE_MODE_NON_SECURE;
	hw_mgr->mini_dump_cb = mini_dump_cb;
	hw_mgr_intf->synx_trigger = cam_icp_mgr_service_synx_test_cmds;

	mutex_init(&hw_mgr->hw_mgr_mutex);
	spin_lock_init(&hw_mgr->hw_mgr_lock);

	atomic_set(&hw_mgr->frame_in_process, 0);
	hw_mgr->frame_in_process_ctx_id = -1;

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		mutex_init(&hw_mgr->ctx_data[i].ctx_mutex);
		if (cam_presil_mode_enabled()) {
			size = CAM_FRAME_CMD_MAX * sizeof(struct cam_hangdump_mem_regions);
			hw_mgr->ctx_data[i].hfi_frame_process.hangdump_mem_regions =
				kzalloc(size, GFP_KERNEL);
		}
	}

	rc = cam_icp_mgr_init_devs(of_node, hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] cam_icp_mgr_init_devs fail: rc: %d",
			hw_mgr->hw_mgr_name, rc);
		goto destroy_mutex;
	}

	rc = cam_smmu_get_handle(hw_mgr->hw_mgr_name, &hw_mgr->iommu_hdl);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] get mmu handle failed: %d",
		hw_mgr->hw_mgr_name, rc);
		goto icp_get_hdl_failed;
	}

	rc = cam_smmu_get_handle("cam-secure", &hw_mgr->iommu_sec_hdl);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] get secure mmu handle failed: %d",
		hw_mgr->hw_mgr_name, rc);
		goto secure_hdl_failed;
	}

	rc = cam_icp_mgr_create_wq(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] cam_icp_mgr_create_wq fail: rc=%d",
		hw_mgr->hw_mgr_name, rc);
		goto icp_wq_create_failed;
	}

	rc = cam_icp_hw_mgr_create_debugfs_entry(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Fail to create debugfs entry",
			hw_mgr->hw_mgr_name);
		goto icp_debugfs_create_failed;
	}

	if (iommu_hdl)
		*iommu_hdl = hw_mgr->iommu_hdl;

	rc = cam_icp_mgr_register_hfi_client(hw_mgr);
	if (rc) {
		CAM_ERR(CAM_ICP, "[%s] Fail to register hw mgr as hfi handle",
			hw_mgr->hw_mgr_name);
		goto icp_hfi_register_failed;
	}

	init_completion(&hw_mgr->icp_complete);
	cam_common_register_mini_dump_cb(cam_icp_hw_mgr_mini_dump_cb, hw_mgr->hw_mgr_name,
		&hw_mgr->hw_mgr_id);

	cam_icp_test_irq_line_at_probe(hw_mgr);

	rc = cam_icp_get_svs_clk_info(hw_mgr);
	if (rc)
		goto icp_get_svs_clk_failed;

	if (hw_mgr->synx_signaling_en) {
		switch (hw_mgr->hw_mgr_id) {
		case 0:
			hw_mgr->synx_core_id = CAM_ICP_0_SYNX_CORE;
			break;
		case 1:
			hw_mgr->synx_core_id = CAM_ICP_1_SYNX_CORE;
			break;
		default:
			hw_mgr->synx_core_id = CAM_INVALID_SYNX_CORE;
			break;
		}
	}

	g_icp_hw_mgr[device_idx] = hw_mgr;

	CAM_DBG(CAM_ICP, "Done hw mgr[%u] init: icp name:%s",
		device_idx, hw_mgr->hw_mgr_name);

	return rc;

icp_get_svs_clk_failed:
	cam_hfi_unregister(&hw_mgr->hfi_handle);
icp_hfi_register_failed:
	debugfs_remove_recursive(hw_mgr->dentry);
	hw_mgr->dentry = NULL;
icp_debugfs_create_failed:
	cam_icp_mgr_destroy_wq(hw_mgr);
icp_wq_create_failed:
	cam_smmu_destroy_handle(hw_mgr->iommu_sec_hdl);
	hw_mgr->iommu_sec_hdl = -1;
secure_hdl_failed:
	cam_smmu_destroy_handle(hw_mgr->iommu_hdl);
	hw_mgr->iommu_hdl = -1;
icp_get_hdl_failed:
	cam_icp_mgr_free_hw_devs(hw_mgr);
destroy_mutex:
	mutex_destroy(&hw_mgr->hw_mgr_mutex);
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		mutex_destroy(&hw_mgr->ctx_data[i].ctx_mutex);
		if (cam_presil_mode_enabled())
			kfree(hw_mgr->ctx_data[i].hfi_frame_process.hangdump_mem_regions);
	}
free_hw_mgr:
	kfree(hw_mgr);

	return rc;
}

void cam_icp_hw_mgr_deinit(int device_idx)
{
	struct cam_icp_hw_mgr *hw_mgr = NULL;
	int i = 0;

	hw_mgr = g_icp_hw_mgr[device_idx];
	if (!hw_mgr) {
		CAM_ERR(CAM_ICP, "Uninitialized hw mgr for subdev: %u", device_idx);
		return;
	}

	CAM_DBG(CAM_ICP, "hw mgr deinit: %u icp name: %s", device_idx, hw_mgr->hw_mgr_name);

	hw_mgr->dentry = NULL;

	cam_icp_mgr_unregister_hfi_client(hw_mgr);
	cam_icp_mgr_destroy_wq(hw_mgr);
	cam_icp_mgr_free_hw_devs(hw_mgr);
	mutex_destroy(&hw_mgr->hw_mgr_mutex);
	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		mutex_destroy(&hw_mgr->ctx_data[i].ctx_mutex);
		if (cam_presil_mode_enabled())
			kfree(hw_mgr->ctx_data[i].hfi_frame_process.hangdump_mem_regions);
	}

	kfree(hw_mgr);
	g_icp_hw_mgr[device_idx] = NULL;
}
