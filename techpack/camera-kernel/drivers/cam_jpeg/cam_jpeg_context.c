// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <media/cam_sync.h>
#include "cam_mem_mgr.h"
#include "cam_jpeg_context.h"
#include "cam_context_utils.h"
#include "cam_debug_util.h"
#include "cam_packet_util.h"

static const char jpeg_dev_name[] = "cam-jpeg";

static int cam_jpeg_context_dump_active_request(void *data, void *args)
{

	struct cam_context         *ctx = (struct cam_context *)data;
	struct cam_ctx_request     *req = NULL;
	struct cam_ctx_request     *req_temp = NULL;
	struct cam_hw_dump_pf_args *pf_args = (struct cam_hw_dump_pf_args *)args;
	int rc = 0;

	if (!ctx || !pf_args) {
		CAM_ERR(CAM_JPEG, "Invalid ctx %pK or pf args %pK",
			ctx, pf_args);
		return -EINVAL;
	}

	CAM_INFO(CAM_JPEG, "iommu fault for jpeg ctx %d state %d",
		ctx->ctx_id, ctx->state);

	list_for_each_entry_safe(req, req_temp,
			&ctx->active_req_list, list) {
		CAM_INFO(CAM_JPEG, "Active req_id: %lld, ctx_id: %u",
			req->request_id, ctx->ctx_id);

		rc = cam_context_dump_pf_info_to_hw(ctx, pf_args, &req->pf_data);
		if (rc)
			CAM_ERR(CAM_JPEG, "Failed to dump pf info ctx_id: %u state: %d",
				ctx->ctx_id, ctx->state);
	}

	if (pf_args->pf_context_info.ctx_found) {
		/* Send PF notification to UMD if PF found on current CTX */
		rc = cam_context_send_pf_evt(ctx, pf_args);
		if (rc)
			CAM_ERR(CAM_JPEG,
				"Failed to notify PF event to userspace rc: %d", rc);
	}

	return rc;
}

static int cam_jpeg_context_mini_dump(void *priv, void *args)
{
	int rc;
	struct cam_context *ctx;

	if (!priv || args) {
		CAM_ERR(CAM_ICP, "Invalid param priv %pK args %pK", priv, args);
		return -EINVAL;
	}

	ctx = (struct cam_context *)priv;
	rc = cam_context_mini_dump(ctx, args);
	if (rc)
		CAM_ERR(CAM_JPEG, "Mini Dump failed %d", rc);

	return rc;
}

static int __cam_jpeg_ctx_acquire_dev_in_available(struct cam_context *ctx,
	struct cam_acquire_dev_cmd *cmd)
{
	int rc;

	rc = cam_context_acquire_dev_to_hw(ctx, cmd);
	if (rc)
		CAM_ERR(CAM_JPEG, "Unable to Acquire device %d", rc);
	else
		ctx->state = CAM_CTX_ACQUIRED;

	return rc;
}

static int __cam_jpeg_ctx_release_dev_in_acquired(struct cam_context *ctx,
	struct cam_release_dev_cmd *cmd)
{
	int rc;

	rc = cam_context_release_dev_to_hw(ctx, cmd);
	if (rc)
		CAM_ERR(CAM_JPEG, "Unable to release device %d", rc);

	cam_common_release_err_params(ctx->dev_hdl);

	ctx->state = CAM_CTX_AVAILABLE;

	return rc;
}

static int __cam_jpeg_ctx_dump_dev_in_acquired(
	struct cam_context      *ctx,
	struct cam_dump_req_cmd *cmd)
{
	int rc;

	rc = cam_context_dump_dev_to_hw(ctx, cmd);
	if (rc)
		CAM_ERR(CAM_JPEG, "Failed to dump device, rc=%d", rc);

	return rc;
}

static int __cam_jpeg_ctx_flush_dev_in_acquired(struct cam_context *ctx,
	struct cam_flush_dev_cmd *cmd)
{
	int rc;

	rc = cam_context_flush_dev_to_hw(ctx, cmd);
	if (rc)
		CAM_ERR(CAM_ICP, "Failed to flush device");

	return rc;
}

static int __cam_jpeg_ctx_config_dev_in_acquired(struct cam_context *ctx,
	struct cam_config_dev_cmd *cmd)
{
	return cam_context_prepare_dev_to_hw(ctx, cmd);
}

static int __cam_jpeg_ctx_handle_buf_done_in_acquired(void *ctx,
	uint32_t evt_id, void *done)
{
	return cam_context_buf_done_from_hw(ctx, done, evt_id);
}

static int __cam_jpeg_ctx_stop_dev_in_acquired(struct cam_context *ctx,
	struct cam_start_stop_dev_cmd *cmd)
{
	int rc;

	rc = cam_context_stop_dev_to_hw(ctx);
	if (rc) {
		CAM_ERR(CAM_JPEG, "Failed in Stop dev, rc=%d", rc);
		return rc;
	}

	return rc;
}

static int cam_jpeg_context_inject_error(void *context, void *err_param)
{
	int rc = 0;
	struct cam_context *ctx = (struct cam_context *)context;
	uint64_t req_id;
	uint32_t err_code;
	uint32_t err_type;

	if (!err_param) {
		CAM_ERR(CAM_ISP, "err_param not valid");
		return -EINVAL;
	}

	req_id     = ((struct cam_err_inject_param *)err_param)->req_id;
	err_code   = ((struct cam_err_inject_param *)err_param)->err_code;
	err_type   = ((struct cam_err_inject_param *)err_param)->err_type;

	switch (err_type) {
	case CAM_REQ_MGR_RETRY_EVENT:
		switch (err_code) {
		case CAM_REQ_MGR_JPEG_THUBNAIL_SIZE_ERROR:
			break;
		default:
			CAM_ERR(CAM_ISP, "err code not supported %d", err_code);
			return -EINVAL;
		}
		break;
	case CAM_SYNC_STATE_SIGNALED_ERROR:
		switch (err_code) {
		case CAM_SYNC_JPEG_EVENT_INVLD_CMD:
		case CAM_SYNC_JPEG_EVENT_SET_IRQ_CB:
		case CAM_SYNC_JPEG_EVENT_HW_RESET_FAILED:
		case CAM_SYNC_JPEG_EVENT_CDM_CHANGE_BASE_ERR:
		case CAM_SYNC_JPEG_EVENT_CDM_CONFIG_ERR:
		case CAM_SYNC_JPEG_EVENT_START_HW_ERR:
			break;
		default:
			CAM_ERR(CAM_ISP, "err code not supported %d", err_code);
			return -EINVAL;
		}
		break;
	default:
		CAM_ERR(CAM_ISP, "err type not supported %d", err_type);
		return -EINVAL;
	}

	rc = cam_context_err_to_hw(ctx, err_param);

	return rc;
}

/* top state machine */
static struct cam_ctx_ops
	cam_jpeg_ctx_state_machine[CAM_CTX_STATE_MAX] = {
	/* Uninit */
	{
		.ioctl_ops = { },
		.crm_ops = { },
		.irq_ops = NULL,
	},
	/* Available */
	{
		.ioctl_ops = {
			.acquire_dev = __cam_jpeg_ctx_acquire_dev_in_available,
		},
		.crm_ops = { },
		.irq_ops = NULL,
		.mini_dump_ops = cam_jpeg_context_mini_dump,
	},
	/* Acquired */
	{
		.ioctl_ops = {
			.release_dev = __cam_jpeg_ctx_release_dev_in_acquired,
			.config_dev = __cam_jpeg_ctx_config_dev_in_acquired,
			.stop_dev = __cam_jpeg_ctx_stop_dev_in_acquired,
			.flush_dev = __cam_jpeg_ctx_flush_dev_in_acquired,
			.dump_dev = __cam_jpeg_ctx_dump_dev_in_acquired,
		},
		.crm_ops = { },
		.irq_ops = __cam_jpeg_ctx_handle_buf_done_in_acquired,
		.pagefault_ops = cam_jpeg_context_dump_active_request,
		.mini_dump_ops = cam_jpeg_context_mini_dump,
		.err_inject_ops = cam_jpeg_context_inject_error,
	},
	/* Ready */
	{
		.ioctl_ops = {},
	},
	/* Flushed */
	{
		.ioctl_ops = {},
	},
	/* Activated */
	{
		.ioctl_ops = {},
	},
};

int cam_jpeg_context_init(struct cam_jpeg_context *ctx,
	struct cam_context *ctx_base,
	struct cam_hw_mgr_intf *hw_intf,
	uint32_t ctx_id,
	int img_iommu_hdl)
{
	int rc;
	int i;

	if (!ctx || !ctx_base) {
		CAM_ERR(CAM_JPEG, "Invalid Context");
		rc = -EFAULT;
		goto err;
	}

	memset(ctx, 0, sizeof(*ctx));

	ctx->base = ctx_base;

	for (i = 0; i < CAM_CTX_REQ_MAX; i++)
		ctx->req_base[i].req_priv = &ctx->jpeg_req[i];

	rc = cam_context_init(ctx_base, jpeg_dev_name, CAM_JPEG, ctx_id,
		NULL, hw_intf, ctx->req_base, CAM_CTX_REQ_MAX, img_iommu_hdl);
	if (rc) {
		CAM_ERR(CAM_JPEG, "Camera Context Base init failed");
		goto err;
	}

	ctx_base->state_machine = cam_jpeg_ctx_state_machine;
	ctx_base->ctx_priv = ctx;

	ctx_base->max_hw_update_entries = CAM_CTX_CFG_MAX;
	ctx_base->max_in_map_entries = CAM_CTX_CFG_MAX;
	ctx_base->max_out_map_entries = CAM_CTX_CFG_MAX;
err:
	return rc;
}

int cam_jpeg_context_deinit(struct cam_jpeg_context *ctx)
{
	if (!ctx || !ctx->base) {
		CAM_ERR(CAM_JPEG, "Invalid params: %pK", ctx);
		return -EINVAL;
	}

	cam_context_deinit(ctx->base);

	memset(ctx, 0, sizeof(*ctx));

	return 0;
}
