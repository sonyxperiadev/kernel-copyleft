/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_CONTEXT_UTILS_H_
#define _CAM_CONTEXT_UTILS_H_

#include <linux/types.h>
#include "cam_smmu_api.h"

/**
 * struct cam_context_utils_flush_args - arguments for flush context util
 *
 * @cmd: flush dev command from userland
 * @flush_active_req: flag to indicate if the device supports flushing a particular
 *                    active request or not
 */
struct cam_context_utils_flush_args {
	struct cam_flush_dev_cmd *cmd;
	bool flush_active_req;
};

int cam_context_buf_done_from_hw(struct cam_context *ctx,
	void *done_event_data, uint32_t evt_id);
int32_t cam_context_release_dev_to_hw(struct cam_context *ctx,
	struct cam_release_dev_cmd *cmd);
int32_t cam_context_prepare_dev_to_hw(struct cam_context *ctx,
	struct cam_config_dev_cmd *cmd);
int32_t cam_context_config_dev_to_hw(
	struct cam_context *ctx, struct cam_config_dev_cmd *cmd);
int32_t cam_context_acquire_dev_to_hw(struct cam_context *ctx,
	struct cam_acquire_dev_cmd *cmd);
int32_t cam_context_start_dev_to_hw(struct cam_context *ctx,
	struct cam_start_stop_dev_cmd *cmd);
int32_t cam_context_stop_dev_to_hw(struct cam_context *ctx);
int32_t cam_context_flush_dev_to_hw(struct cam_context *ctx,
	struct cam_context_utils_flush_args *args);
int32_t cam_context_flush_ctx_to_hw(struct cam_context *ctx);
int32_t cam_context_flush_req_to_hw(struct cam_context *ctx,
	struct cam_context_utils_flush_args *args);
int32_t cam_context_send_pf_evt(struct cam_context *ctx,
	struct cam_hw_dump_pf_args *pf_args);
int32_t cam_context_dump_pf_info_to_hw(struct cam_context *ctx,
	struct cam_hw_dump_pf_args *pf_args,
	struct cam_hw_mgr_pf_request_info *pf_req_info);
int32_t cam_context_dump_hw_acq_info(struct cam_context *ctx);
int32_t cam_context_dump_dev_to_hw(struct cam_context *ctx,
	struct cam_dump_req_cmd *cmd);
size_t cam_context_parse_config_cmd(struct cam_context *ctx, struct cam_config_dev_cmd *cmd,
	struct cam_packet **packet);
int cam_context_mini_dump(struct cam_context *ctx, void *args);
int cam_context_apply_evt_injection(struct cam_context *ctx, void *inject_evt_arg);
#endif /* _CAM_CONTEXT_UTILS_H_ */
