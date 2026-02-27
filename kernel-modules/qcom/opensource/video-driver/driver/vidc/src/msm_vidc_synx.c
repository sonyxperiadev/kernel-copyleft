// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <msm_hw_fence_synx_translation.h>
#include "msm_vidc_core.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_fence.h"
#include "msm_vidc_debug.h"
#include <synx_api.h>

#define MSM_VIDC_SYNX_FENCE_CLIENT_ID      SYNX_CLIENT_HW_FENCE_VID_CTX0
#define MSM_VIDC_SYNX_CREATE_DMA_FENCE     SYNX_CREATE_DMA_FENCE
#define MAX_SYNX_FENCE_SESSION_NAME        64

static const char *msm_vidc_synx_dma_fence_get_driver_name(struct dma_fence *df)
{
	struct msm_vidc_fence *fence;

	if (df) {
		fence = container_of(df, struct msm_vidc_fence, dma_fence);
		return fence->name;
	}
	return "msm_vidc_synx_dma_fence_get_driver_name: invalid fence";
}

static const char *msm_vidc_synx_dma_fence_get_timeline_name(struct dma_fence *df)
{
	struct msm_vidc_fence *fence;

	if (df) {
		fence = container_of(df, struct msm_vidc_fence, dma_fence);
		return fence->name;
	}
	return "msm_vidc_synx_dma_fence_get_timeline_name: invalid fence";
}

static void msm_vidc_synx_fence_release(struct dma_fence *df)
{
	struct msm_vidc_fence *fence;
	int rc = 0;

	if (!df) {
		d_vpr_e("%s: invalid dma fence\n", __func__);
		return;
	}

	fence = container_of(df, struct msm_vidc_fence, dma_fence);
	if (!fence) {
		d_vpr_e("%s: invalid fence\n", __func__);
		return;
	}
	d_vpr_l("%s: name %s\n", __func__, fence->name);

	/* destroy associated synx fence */
	if (fence->session) {
		rc = synx_hwfence_release((struct synx_session *)fence->session,
			(u32)fence->fence_id);
		if (rc)
			d_vpr_e("%s: failed to destroy synx fence for %s\n",
				__func__, fence->name);
	}

	vfree(fence);
	return;
}

static const struct dma_fence_ops msm_vidc_synx_dma_fence_ops = {
	.get_driver_name = msm_vidc_synx_dma_fence_get_driver_name,
	.get_timeline_name = msm_vidc_synx_dma_fence_get_timeline_name,
	.release = msm_vidc_synx_fence_release,
};

static struct msm_vidc_fence *msm_vidc_get_synx_fence_from_id(
	struct msm_vidc_inst *inst, u64 fence_id)
{
	struct msm_vidc_fence *fence, *dummy_fence;
	bool found = false;

	list_for_each_entry_safe(fence, dummy_fence, &inst->fence_list, list) {
		if (fence->fence_id == fence_id) {
			found = true;
			break;
		}
	}

	if (!found) {
		i_vpr_l(inst, "%s: no fence available for id: %u\n",
			__func__, fence_id);
		return NULL;
	}

	return fence;
}

static void msm_vidc_synx_fence_destroy(struct msm_vidc_inst *inst, u64 fence_id)
{
	struct msm_vidc_fence *fence;

	fence = msm_vidc_get_synx_fence_from_id(inst, fence_id);
	if (!fence)
		return;

	i_vpr_e(inst, "%s: fence %s\n", __func__, fence->name);
	list_del_init(&fence->list);

	dma_fence_set_error(&fence->dma_fence, -EINVAL);
	dma_fence_signal(&fence->dma_fence);
	dma_fence_put(&fence->dma_fence);
}

static int msm_vidc_synx_fence_register(struct msm_vidc_core *core)
{
	struct synx_initialization_params params;
	struct synx_session *session = NULL;
	char synx_session_name[MAX_SYNX_FENCE_SESSION_NAME];
	struct synx_queue_desc queue_desc;

	if (!core->capabilities[SUPPORTS_SYNX_FENCE].value)
		return 0;

	/* fill synx_initialization_params */
	memset(&params, 0, sizeof(struct synx_initialization_params));
	memset(&queue_desc, 0, sizeof(struct synx_queue_desc));

	params.id = (enum synx_client_id)MSM_VIDC_SYNX_FENCE_CLIENT_ID;
	snprintf(synx_session_name, MAX_SYNX_FENCE_SESSION_NAME,
		"video synx fence");
	params.name = synx_session_name;
	params.ptr = &queue_desc;
	params.flags = SYNX_INIT_MAX; /* unused */

	session =
		(struct synx_session *)synx_hwfence_initialize(&params);
	if (IS_ERR_OR_NULL(session)) {
		d_vpr_e("%s: invalid synx fence session\n", __func__);
		return -EINVAL;
	}

	/* fill core synx fence data */
	core->synx_fence_data.client_id = (u32)params.id;
	core->synx_fence_data.client_flags = (u32)params.flags;
	core->synx_fence_data.session = (void *)session;
	core->synx_fence_data.queue.size = (u32)queue_desc.size;
	core->synx_fence_data.queue.kvaddr = queue_desc.vaddr;
	core->synx_fence_data.queue.phys_addr = (phys_addr_t)queue_desc.dev_addr;

	core->synx_fence_data.queue.type = MSM_VIDC_BUF_INTERFACE_QUEUE;
	core->synx_fence_data.queue.region = MSM_VIDC_NON_SECURE;
	core->synx_fence_data.queue.direction = DMA_BIDIRECTIONAL;

	d_vpr_h("%s: successfully registered synx fence\n", __func__);
	return 0;
}

static int msm_vidc_synx_fence_deregister(struct msm_vidc_core *core)
{
	int rc = 0;

	if (!core->capabilities[SUPPORTS_SYNX_FENCE].value)
		return 0;

	rc = synx_hwfence_uninitialize(
		(struct synx_session *)core->synx_fence_data.session);
	if (rc) {
		d_vpr_e("%s: failed to deregister synx fence\n", __func__);
		/* ignore error */
		rc = 0;
	} else {
		d_vpr_l("%s: successfully deregistered synx fence\n", __func__);
	}

	return rc;
}

static struct msm_vidc_fence *msm_vidc_synx_dma_fence_create(struct msm_vidc_inst *inst)
{
	struct msm_vidc_fence *fence = NULL;

	fence = vzalloc(sizeof(*fence));
	if (!fence) {
		i_vpr_e(inst, "%s: allocation failed\n", __func__);
		return NULL;
	}

	fence->fd = INVALID_FD;
	spin_lock_init(&fence->lock);
	dma_fence_init(&fence->dma_fence, &msm_vidc_synx_dma_fence_ops,
		&fence->lock, inst->fence_context.ctx_num,
		++inst->fence_context.seq_num);
	snprintf(fence->name, sizeof(fence->name), "synx %s: %llu",
		inst->fence_context.name, inst->fence_context.seq_num);

	fence->fence_id = fence->dma_fence.seqno;

	INIT_LIST_HEAD(&fence->list);
	list_add_tail(&fence->list, &inst->fence_list);
	i_vpr_l(inst, "%s: created %s\n", __func__, fence->name);

	return fence;
}

static struct msm_vidc_fence *msm_vidc_synx_fence_create(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_fence *fence = NULL;
	struct msm_vidc_core *core = NULL;
	struct synx_create_params params;
	u32 fence_id = 0;

	core = inst->core;

	/* return if synx fence is not supported */
	if (!core->capabilities[SUPPORTS_SYNX_FENCE].value)
		return NULL;

	/* create dma fence */
	fence = msm_vidc_synx_dma_fence_create(inst);
	if (!fence) {
		i_vpr_e(inst, "%s: failed to create dma fence\n", __func__);
		return NULL;
	}

	if (!core->synx_fence_data.session) {
		i_vpr_e(inst, "%s: invalid synx fence session\n", __func__);
		goto destroy_dma_fence;
	}

	/* fill synx fence params structure */
	memset(&params, 0, sizeof(struct synx_create_params));
	params.name = fence->name;
	params.fence = (void *)&fence->dma_fence;
	params.h_synx = &fence_id;
	params.flags = MSM_VIDC_SYNX_CREATE_DMA_FENCE;

	/* create hw fence */
	rc = synx_hwfence_create(
		(struct synx_session *)core->synx_fence_data.session,
		&params);
	if (rc) {
		i_vpr_e(inst, "%s: failed to create hw fence for %s",
			__func__, fence->name);
		goto destroy_dma_fence;
	}

	fence->fence_id = (u64)(*(params.h_synx));
	/* this copy of hw fence client handle is req. to destroy synx fence */
	fence->session = core->synx_fence_data.session;
	i_vpr_l(inst, "%s: successfully created synx fence with id: %llu",
		__func__, fence->fence_id);

	return fence;

destroy_dma_fence:
	msm_vidc_synx_fence_destroy(inst, fence->fence_id);
	return NULL;
}

int msm_vidc_synx_fence_create_fd(struct msm_vidc_inst *inst,
	struct msm_vidc_fence *fence)
{
	int rc = 0;

	fence->fd = get_unused_fd_flags(0);
	if (fence->fd < 0) {
		i_vpr_e(inst, "%s: getting fd (%d) failed\n", __func__,
			fence->fd);
		rc = -EINVAL;
		goto err_fd;
	}
	fence->sync_file = sync_file_create(&fence->dma_fence);
	if (!fence->sync_file) {
		i_vpr_e(inst, "%s: sync_file_create failed\n", __func__);
		rc = -EINVAL;
		goto err_sync_file;
	}
	fd_install(fence->fd, fence->sync_file->file);

	i_vpr_l(inst, "%s: created fd %d for fence %s\n", __func__,
		fence->fd, fence->name);

	return 0;

err_sync_file:
	put_unused_fd(fence->fd);
err_fd:
	return rc;
}

static int msm_vidc_synx_fence_signal(struct msm_vidc_inst *inst, u64 fence_id)
{
	int rc = 0;
	struct msm_vidc_fence *fence;
	struct msm_vidc_core *core;

	core = inst->core;

	fence = msm_vidc_get_synx_fence_from_id(inst, fence_id);
	if (!fence) {
		i_vpr_e(inst, "%s: no fence available to signal with id: %u\n",
			__func__, fence_id);
		rc = -EINVAL;
		goto exit;
	}

	i_vpr_l(inst, "%s: fence %s\n", __func__, fence->name);
	list_del_init(&fence->list);

	dma_fence_signal(&fence->dma_fence);
	dma_fence_put(&fence->dma_fence);

exit:
	return rc;
}

static void msm_vidc_synx_fence_recover(struct msm_vidc_core *core)
{
	int rc = 0;

	rc = synx_hwfence_recover(
		(enum synx_client_id)core->synx_fence_data.client_id);
	if (rc)
		d_vpr_e("%s: failed to recover synx fences for client id: %d",
			__func__,
			(enum synx_client_id)core->synx_fence_data.client_id);

	return;
}

const struct msm_vidc_fence_ops *get_synx_fence_ops(void)
{
	static struct msm_vidc_fence_ops synx_ops;

	synx_ops.fence_register      = msm_vidc_synx_fence_register;
	synx_ops.fence_deregister    = msm_vidc_synx_fence_deregister;
	synx_ops.fence_create        = msm_vidc_synx_fence_create;
	synx_ops.fence_create_fd     = msm_vidc_synx_fence_create_fd;
	synx_ops.fence_destroy       = msm_vidc_synx_fence_destroy;
	synx_ops.fence_signal        = msm_vidc_synx_fence_signal;
	synx_ops.fence_recover       = msm_vidc_synx_fence_recover;

	return &synx_ops;
}
