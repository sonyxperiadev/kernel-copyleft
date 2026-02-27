// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
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
#include "cam_jpeg_hw_intf.h"

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

	cam_common_release_evt_params(ctx->dev_hdl);

	rc = cam_context_release_dev_to_hw(ctx, cmd);
	if (rc)
		CAM_ERR(CAM_JPEG, "Unable to release device %d", rc);

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
	struct cam_context_utils_flush_args flush_args;

	flush_args.cmd = cmd;
	flush_args.flush_active_req = true;

	rc = cam_context_flush_dev_to_hw(ctx, &flush_args);
	if (rc)
		CAM_ERR(CAM_ICP, "Failed to flush device");

	return rc;
}

static int __cam_jpeg_ctx_config_dev_in_acquired(struct cam_context *ctx,
	struct cam_config_dev_cmd *cmd)
{
	return cam_context_prepare_dev_to_hw(ctx, cmd);
}

static int __cam_jpeg_ctx_handle_buf_done_in_acquired(void *ctx, void *done_evt_data)
{
	struct cam_jpeg_hw_buf_done_evt_data *buf_done = done_evt_data;

	return cam_context_buf_done_from_hw(ctx, buf_done->buf_done_data,
		 buf_done->evt_id);
}

static int __cam_jpeg_ctx_handle_evt_inducement(void *ctx, void *inject_evt_arg)
{
	return cam_context_apply_evt_injection(ctx, inject_evt_arg);
}

static int __cam_jpeg_ctx_handle_hw_event(void *ctx,
	uint32_t evt_id, void *evt_data)
{
	int rc;

	if (!ctx || !evt_data) {
		CAM_ERR(CAM_JPEG, "Invalid parameters ctx %s evt_data: %s",
			CAM_IS_NULL_TO_STR(ctx), CAM_IS_NULL_TO_STR(evt_data));
		return -EINVAL;
	}

	switch (evt_id) {
	case CAM_JPEG_EVT_ID_BUF_DONE:
		rc = __cam_jpeg_ctx_handle_buf_done_in_acquired(ctx, evt_data);
		break;
	case CAM_JPEG_EVT_ID_INDUCE_ERR:
		rc = __cam_jpeg_ctx_handle_evt_inducement(ctx, evt_data);
		break;
	default:
		CAM_ERR(CAM_JPEG, "Invalid event id: %u", evt_id);
		rc = -EINVAL;
	}

	return rc;
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

static int cam_jpeg_context_validate_event_notify_injection(struct cam_context *ctx,
	struct cam_hw_inject_evt_param *evt_params)
{
	int rc = 0;
	uint32_t evt_type;
	uint64_t req_id;

	req_id   = evt_params->req_id;
	evt_type = evt_params->u.evt_notify.evt_notify_type;

	switch (evt_type) {
	case V4L_EVENT_CAM_REQ_MGR_NODE_EVENT: {
		struct cam_hw_inject_node_evt_param *node_evt_params =
			&evt_params->u.evt_notify.u.node_evt_params;

		switch (node_evt_params->event_type) {
		case CAM_REQ_MGR_RETRY_EVENT:
			break;
		default:
			CAM_ERR(CAM_JPEG,
				"Invalid event type %u for node event injection event cause: %u req id: %llu ctx id: %u dev hdl: %d",
				node_evt_params->event_type, node_evt_params->event_cause,
				req_id, ctx->ctx_id, ctx->dev_hdl);
			return -EINVAL;
		}

		CAM_INFO(CAM_JPEG,
			"Inject Node evt: event type: %u event cause: %u req id: %llu ctx id: %u dev hdl: %d",
			node_evt_params->event_type, node_evt_params->event_cause,
			req_id, ctx->ctx_id, ctx->dev_hdl);
		break;
	}
	case V4L_EVENT_CAM_REQ_MGR_PF_ERROR: {
		struct cam_hw_inject_pf_evt_param *pf_evt_params =
			&evt_params->u.evt_notify.u.pf_evt_params;
		bool non_fatal_en;

		rc = cam_smmu_is_cb_non_fatal_fault_en(ctx->img_iommu_hdl, &non_fatal_en);
		if (rc) {
			CAM_ERR(CAM_JPEG,
				"Fail to query whether device's cb has non-fatal enabled rc: %d",
				rc);
			return rc;
		}

		if (!non_fatal_en) {
			CAM_ERR(CAM_JPEG,
				"Fail to inject page fault event notification. Page fault is fatal for JPEG");
			return -EINVAL;
		}

		CAM_INFO(CAM_JPEG,
			"Inject PF evt: req_id: %llu ctx id: %u dev hdl: %d ctx found: %hhu",
			req_id, ctx->ctx_id, ctx->dev_hdl, pf_evt_params->ctx_found);
		break;
	}
	default:
		CAM_ERR(CAM_JPEG, "Event notification type not supported: %u", evt_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_jpeg_context_inject_evt(void *context, void *evt_args)
{
	struct cam_context *ctx = context;
	struct cam_hw_inject_evt_param *evt_params = NULL;
	struct cam_hw_inject_buffer_error_param *buf_err_params = NULL;
	int rc = 0;

	if (!ctx || !evt_args) {
		CAM_ERR(CAM_JPEG,
			"invalid params ctx %s event args %s",
			CAM_IS_NULL_TO_STR(ctx), CAM_IS_NULL_TO_STR(evt_args));
		return -EINVAL;
	}

	evt_params = (struct cam_hw_inject_evt_param *)evt_args;

	if (evt_params->inject_id == CAM_COMMON_EVT_INJECT_BUFFER_ERROR_TYPE) {
		buf_err_params = &evt_params->u.buf_err_evt;
		if (buf_err_params->sync_error > CAM_SYNC_JPEG_EVENT_START ||
			buf_err_params->sync_error < CAM_SYNC_JPEG_EVENT_END) {
			CAM_INFO(CAM_JPEG, "Inject buffer sync error %u ctx id: %u req id %llu",
				buf_err_params->sync_error, ctx->ctx_id, evt_params->req_id);
		} else {
			CAM_ERR(CAM_JPEG, "Invalid buffer sync error %u",
				buf_err_params->sync_error);
			return -EINVAL;
		}
	} else {
		rc = cam_jpeg_context_validate_event_notify_injection(ctx, evt_params);
		if (rc) {
			CAM_ERR(CAM_JPEG,
				"Event notification injection failed validation rc: %d", rc);
			return rc;
		}
	}

	if (ctx->hw_mgr_intf->hw_inject_evt)
		ctx->hw_mgr_intf->hw_inject_evt(ctx->ctxt_to_hw_map, evt_args);

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
		.irq_ops = __cam_jpeg_ctx_handle_hw_event,
		.pagefault_ops = cam_jpeg_context_dump_active_request,
		.mini_dump_ops = cam_jpeg_context_mini_dump,
		.evt_inject_ops = cam_jpeg_context_inject_evt,
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
