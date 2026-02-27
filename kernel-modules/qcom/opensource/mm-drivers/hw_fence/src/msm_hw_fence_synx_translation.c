// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/soc/qcom/msm_hw_fence.h>
#include "msm_hw_fence_synx_translation.h"
#include "hw_fence_drv_priv.h"
#include "hw_fence_drv_debug.h"

/**
 * MAX_SUPPORTED_DPU0:
 * Maximum number of dpu clients supported
 */
#define MAX_SUPPORTED_DPU0 (HW_FENCE_CLIENT_ID_CTL5 - HW_FENCE_CLIENT_ID_CTL0)

static int to_synx_status(int hw_fence_status_code)
{
	int synx_status_code;

	switch (hw_fence_status_code) {
	case 0:
		synx_status_code = SYNX_SUCCESS;
		break;
	case -ENOMEM:
		synx_status_code = -SYNX_NOMEM;
		break;
	case -EPERM:
		synx_status_code = -SYNX_NOPERM;
		break;
	case -ETIMEDOUT:
		synx_status_code = -SYNX_TIMEOUT;
		break;
	case -EALREADY:
		synx_status_code = -SYNX_ALREADY;
		break;
	case -ENOENT:
		synx_status_code = -SYNX_NOENT;
		break;
	case -EINVAL:
		synx_status_code = -SYNX_INVALID;
		break;
	case -EBUSY:
		synx_status_code = -SYNX_BUSY;
		break;
	default:
		synx_status_code = hw_fence_status_code;
		break;
	}

	return synx_status_code;
}

static enum hw_fence_client_id _get_hw_fence_client_id(enum synx_client_id synx_client_id)
{
	enum hw_fence_client_id hw_fence_client_id;

	switch ((int)synx_client_id) {
	case SYNX_CLIENT_HW_FENCE_GFX_CTX0:
		hw_fence_client_id = HW_FENCE_CLIENT_ID_CTX0;
		break;
	case SYNX_CLIENT_HW_FENCE_IPE_CTX0 ... SYNX_CLIENT_HW_FENCE_IPE_CTX0 +
			SYNX_MAX_SIGNAL_PER_CLIENT - 1:
		hw_fence_client_id = synx_client_id - SYNX_CLIENT_HW_FENCE_IPE_CTX0 +
			HW_FENCE_CLIENT_ID_IPE;
		break;
	case SYNX_CLIENT_HW_FENCE_VID_CTX0 ... SYNX_CLIENT_HW_FENCE_VID_CTX0 +
			SYNX_MAX_SIGNAL_PER_CLIENT - 1:
		hw_fence_client_id = synx_client_id - SYNX_CLIENT_HW_FENCE_VID_CTX0 +
			HW_FENCE_CLIENT_ID_VPU;
		break;
	case SYNX_CLIENT_HW_FENCE_DPU0_CTL0 ... SYNX_CLIENT_HW_FENCE_DPU0_CTL0 + MAX_SUPPORTED_DPU0:
		hw_fence_client_id = synx_client_id - SYNX_CLIENT_HW_FENCE_DPU0_CTL0 +
			HW_FENCE_CLIENT_ID_CTL0;
		break;
	case SYNX_CLIENT_HW_FENCE_IFE0_CTX0 ... SYNX_CLIENT_HW_FENCE_IFE7_CTX0 +
			SYNX_MAX_SIGNAL_PER_CLIENT - 1:
		hw_fence_client_id = synx_client_id - SYNX_CLIENT_HW_FENCE_IFE0_CTX0 +
			HW_FENCE_CLIENT_ID_IFE0;
		break;
	default:
		HWFNC_ERR("Unsupported hw-fence client for synx_id:%d\n", synx_client_id);
		hw_fence_client_id = HW_FENCE_CLIENT_MAX;
		break;
	}

	return hw_fence_client_id;
}

static bool is_hw_fence_client(enum synx_client_id synx_client_id)
{
	return synx_client_id >= SYNX_HW_FENCE_CLIENT_START
		&& synx_client_id < SYNX_HW_FENCE_CLIENT_END;
}

struct synx_session *synx_hwfence_initialize(struct synx_initialization_params *params)
{
	struct synx_session *session = NULL;
	enum hw_fence_client_id client_id;
	void *client_handle;

	if (!hw_fence_driver_enable)
		return ERR_PTR(-SYNX_INVALID);

	if (IS_ERR_OR_NULL(params) || IS_ERR_OR_NULL(params->ptr)) {
		HWFNC_ERR("invalid params:0x%pK params->ptr:0x%pK\n", params,
			IS_ERR_OR_NULL(params) ? NULL : params->ptr);
		return ERR_PTR(-SYNX_INVALID);
	}

	client_id = _get_hw_fence_client_id(params->id);
	if (!is_hw_fence_client(params->id) || client_id == HW_FENCE_CLIENT_MAX) {
		HWFNC_ERR("Initializing session for invalid synx_id:%d\n", params->id);
		return ERR_PTR(-SYNX_INVALID);
	}

	session = kzalloc(sizeof(struct synx_session), GFP_KERNEL);
	if (!session)
		return ERR_PTR(-SYNX_NOMEM);

	client_handle = msm_hw_fence_register(client_id,
		(struct msm_hw_fence_mem_addr *)params->ptr);
	if (IS_ERR_OR_NULL(client_handle)) {
		kfree(session);
		HWFNC_ERR("failed to initialize synx_id:%d ret:%d\n", params->id,
			PTR_ERR(client_handle));
		return ERR_PTR(to_synx_status(PTR_ERR(client_handle)));
	}
	session->client = client_handle;
	session->type = params->id;
	HWFNC_DBG_INIT("initialized session synx_id:%d hw_fence_id:%d\n", params->id, client_id);

	return session;
}
EXPORT_SYMBOL(synx_hwfence_initialize);

int synx_hwfence_uninitialize(struct synx_session *session)
{
	int ret;

	if (IS_ERR_OR_NULL(session) || !is_hw_fence_client(session->type)) {
		HWFNC_ERR("invalid session:0x%pK synx_id:%d\n", session,
			IS_ERR_OR_NULL(session) ? -1 : session->type);
		return -SYNX_INVALID;
	}

	ret = msm_hw_fence_deregister(session->client);
	if (ret)
		HWFNC_ERR("Failed to deregister synx_id:%d ret:%d\n", session->type, ret);
	else
		kfree(session);

	return to_synx_status(ret);
}
EXPORT_SYMBOL(synx_hwfence_uninitialize);

int synx_hwfence_create(struct synx_session *session, struct synx_create_params *params)
{
	int ret = 0;
	struct msm_hw_fence_create_params hwfence_params;
	u64 handle;

	if (IS_ERR_OR_NULL(session) || !is_hw_fence_client(session->type) ||
			IS_ERR_OR_NULL(params)) {
		HWFNC_ERR("invalid session:0x%pK synx_id:%d params:0x%pK\n", session,
			IS_ERR_OR_NULL(session) ? -1 : session->type, params);
		return -SYNX_INVALID;
	}

	if (IS_ERR_OR_NULL(params->h_synx) || (params->flags > SYNX_CREATE_MAX_FLAGS) ||
			!(params->flags & SYNX_CREATE_DMA_FENCE) ||
			(params->flags & SYNX_CREATE_CSL_FENCE) ||
			IS_ERR_OR_NULL(params->fence)) {
		HWFNC_ERR("synx_id:%d invalid create params h_synx:0x%pK flags:0x%x fence:0x%pK\n",
			session->type, params->h_synx, params->flags, params->fence);
		return -SYNX_INVALID;
	}

	hwfence_params.fence = params->fence;
	hwfence_params.handle = &handle;
	ret = msm_hw_fence_create(session->client, &hwfence_params);
	if (ret) {
		HWFNC_ERR("synx_id:%d failed create fence:0x%pK flags:0x%x ret:%d\n", session->type,
			params->fence, params->flags, ret);
		return to_synx_status(ret);
	}
	if (handle > U32_MAX) {
		HWFNC_ERR("synx_id:%d fence handle:%llu would overflow h_synx\n", session->type,
			handle);
		msm_hw_fence_destroy_with_handle(session->client, handle);
		return -SYNX_INVALID;
	}
	*params->h_synx = handle;

	return SYNX_SUCCESS;
}
EXPORT_SYMBOL(synx_hwfence_create);

int synx_hwfence_release(struct synx_session *session, u32 h_synx)
{
	int ret;

	if (IS_ERR_OR_NULL(session) || !is_hw_fence_client(session->type)) {
		HWFNC_ERR("invalid session:0x%pK synx_id:%d\n", session,
			IS_ERR_OR_NULL(session) ? -1 : session->type);
		return -SYNX_INVALID;
	}

	ret = msm_hw_fence_destroy_with_handle(session->client, h_synx);
	if (ret)
		HWFNC_ERR("synx_id:%d failed to destroy fence h_synx:%u ret:%d\n", session->type,
			h_synx, ret);

	return to_synx_status(ret);
}
EXPORT_SYMBOL(synx_hwfence_release);

int synx_hwfence_signal(struct synx_session *session, u32 h_synx, enum synx_signal_status status)
{
	int ret;

	if (IS_ERR_OR_NULL(session) || !is_hw_fence_client(session->type)) {
		HWFNC_ERR("invalid session:0x%pK synx_id:%d\n", session,
			IS_ERR_OR_NULL(session) ? -1 : session->type);
		return -SYNX_INVALID;
	}

	ret = msm_hw_fence_update_txq(session->client, h_synx, 0, (u32)status);
	if (ret)
		HWFNC_ERR("synx_id:%d failed to signal fence h_synx:%u status:%d ret:%d\n",
			session->type, h_synx, status, ret);

	return to_synx_status(ret);
}
EXPORT_SYMBOL(synx_hwfence_signal);

int synx_hwfence_recover(enum synx_client_id id)
{
	int ret;

	if (!is_hw_fence_client(id)) {
		HWFNC_ERR("invalid synx_id:%d\n", id);
		return -SYNX_INVALID;
	}

	ret = msm_hw_fence_reset_client_by_id(_get_hw_fence_client_id(id),
		MSM_HW_FENCE_RESET_WITHOUT_DESTROY);
	if (ret)
		HWFNC_ERR("synx_id:%d failed to recover ret:%d\n", id, ret);

	return to_synx_status(ret);
}
EXPORT_SYMBOL(synx_hwfence_recover);

static int synx_hwfence_import_indv(void *client, struct synx_import_indv_params *params)
{
	u64 handle;
	int ret;

	if (IS_ERR_OR_NULL(client) || IS_ERR_OR_NULL(params) ||
			IS_ERR_OR_NULL(params->new_h_synx) ||
			!(params->flags & SYNX_IMPORT_DMA_FENCE) ||
			(params->flags & SYNX_IMPORT_SYNX_FENCE) || IS_ERR_OR_NULL(params->fence)) {
		HWFNC_ERR("invalid client:0x%pK params:0x%pK h_synx:0x%pK flags:0x%x fence:0x%pK\n",
			client, params, IS_ERR_OR_NULL(params) ? NULL : params->new_h_synx,
			IS_ERR_OR_NULL(params) ? 0 : params->flags,
			IS_ERR_OR_NULL(params) ? NULL : params->fence);
		return -SYNX_INVALID;
	}

	ret = msm_hw_fence_wait_update_v2(client, (struct dma_fence **)&params->fence, &handle,
		NULL, 1, true);
	if (ret) {
		HWFNC_ERR("failed to import fence:0x%pK flags:0x%x ret:%d\n", params->fence,
			params->flags, ret);
		return to_synx_status(ret);
	}
	if (handle > U32_MAX) {
		HWFNC_ERR("fence handle:%llu would overflow new_h_synx\n", handle);
		msm_hw_fence_wait_update_v2(client, (struct dma_fence **)&params->fence, &handle,
			NULL, 1, false);
		return -SYNX_INVALID;
	}
	*params->new_h_synx = handle;

	return SYNX_SUCCESS;
}

static int synx_hwfence_import_arr(void *client, struct synx_import_arr_params *params)
{
	int i, ret;

	if (IS_ERR_OR_NULL(client) || IS_ERR_OR_NULL(params) || !params->num_fences) {
		HWFNC_ERR("invalid import arr client:0x%pK params:0x%pK num_fences:%u\n", client,
			params, IS_ERR_OR_NULL(params) ? -1 : params->num_fences);
		return -SYNX_INVALID;
	}

	for (i = 0; i < params->num_fences; i++) {
		ret = synx_hwfence_import_indv(client, &params->list[i]);
		if (ret) {
			HWFNC_ERR("importing fence[%u] 0x%pK failed ret:%d\n", i,
				params->list[i].fence, ret);
			return ret;
		}
	}

	return SYNX_SUCCESS;
}

int synx_hwfence_import(struct synx_session *session, struct synx_import_params *params)
{
	int ret;

	if (IS_ERR_OR_NULL(session) || !is_hw_fence_client(session->type)
			|| IS_ERR_OR_NULL(params)) {
		HWFNC_ERR("invalid session:0x%pK synx_id:%d params:0x%pK\n", session,
			IS_ERR_OR_NULL(session) ? -1 : session->type, params);
		return -SYNX_INVALID;
	}

	if (params->type == SYNX_IMPORT_ARR_PARAMS)
		ret = synx_hwfence_import_arr(session->client, &params->arr);
	else
		ret = synx_hwfence_import_indv(session->client, &params->indv);

	if (ret)
		HWFNC_ERR("synx_id:%d failed to import type:%s fences ret:%d\n", session->type,
			(params->type == SYNX_IMPORT_ARR_PARAMS) ? "arr" : "indv", ret);

	return ret;
}
EXPORT_SYMBOL(synx_hwfence_import);
