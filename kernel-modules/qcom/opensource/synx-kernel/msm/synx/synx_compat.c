// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/types.h>
#include "synx_api.h"
#include "synx_hwfence.h"
#include "synx_private.h"
#include "synx_debugfs.h"

struct synx_ops synx_hwfence_ops = {
	.uninitialize = NULL,
	.create = NULL,
	.release = NULL,
	.signal = NULL,
	.async_wait = NULL,
	.get_fence = NULL,
	.import = NULL,
	.get_status = NULL,
	.merge = NULL,
	.wait = NULL,
	.cancel_async_wait = NULL
};

static struct synx_ops synx_internal_ops = {
	.uninitialize = synx_internal_uninitialize,
	.create = synx_internal_create,
	.release = synx_internal_release,
	.signal = synx_internal_signal,
	.async_wait = synx_internal_async_wait,
	.get_fence = synx_internal_get_fence,
	.import = synx_internal_import,
	.get_status = synx_internal_get_status,
	.merge = synx_internal_merge,
	.wait = synx_internal_wait,
	.cancel_async_wait = synx_internal_cancel_async_wait
};

static bool is_hw_fence_client(enum synx_client_id synx_client_id)
{
	return synx_client_id >= SYNX_HW_FENCE_CLIENT_START
		&& synx_client_id < SYNX_HW_FENCE_CLIENT_END;
}

struct synx_session *synx_initialize(struct synx_initialization_params *params)
{
	struct synx_session *session = NULL;

	if (IS_ERR_OR_NULL(params))
		return ERR_PTR(-SYNX_INVALID);

	if (is_hw_fence_client(params->id)) {
		session = synx_hwfence_initialize(params);
		if (IS_ERR_OR_NULL(session))
			return session;
		session->ops = &synx_hwfence_ops;
	} else {
		session = synx_internal_initialize(params);
		if (IS_ERR_OR_NULL(session))
			return session;
		session->ops = &synx_internal_ops;
	}
	return session;
}
EXPORT_SYMBOL(synx_initialize);

int synx_uninitialize(struct synx_session *session)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->uninitialize)
		return -SYNX_INVALID;
	return session->ops->uninitialize(session);
}
EXPORT_SYMBOL(synx_uninitialize);

int synx_create(struct synx_session *session, struct synx_create_params *params)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->create)
		return -SYNX_INVALID;
	return session->ops->create(session, params);
}
EXPORT_SYMBOL(synx_create);

int synx_release(struct synx_session *session, u32 h_synx)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->release)
		return -SYNX_INVALID;
	return session->ops->release(session, h_synx);
}
EXPORT_SYMBOL(synx_release);

int synx_signal(struct synx_session *session, u32 h_synx, enum synx_signal_status status)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->signal)
		return -SYNX_INVALID;
	return session->ops->signal(session, h_synx, status);
}
EXPORT_SYMBOL(synx_signal);

int synx_async_wait(struct synx_session *session, struct synx_callback_params *params)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->async_wait)
		return -SYNX_INVALID;
	return session->ops->async_wait(session, params);
}
EXPORT_SYMBOL(synx_async_wait);

int synx_recover(enum synx_client_id id)
{
	int ret = 0;

	dprintk(SYNX_WARN, "Subsystem restart for core_id: %d\n", id);

	if (is_hw_fence_client(id))
		ret = synx_hwfence_recover(id);
	else
		ret = synx_internal_recover(id);
	return ret;
}
EXPORT_SYMBOL(synx_recover);

void *synx_get_fence(struct synx_session *session, u32 h_synx)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->get_fence)
		return ERR_PTR(-SYNX_INVALID);
	return session->ops->get_fence(session, h_synx);
}
EXPORT_SYMBOL(synx_get_fence);

int synx_import(struct synx_session *session, struct synx_import_params *params)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->import)
		return -SYNX_INVALID;
	return session->ops->import(session, params);
}
EXPORT_SYMBOL(synx_import);

int synx_get_status(struct synx_session *session, u32 h_synx)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->get_status)
		return -SYNX_INVALID;
	return session->ops->get_status(session, h_synx);
}
EXPORT_SYMBOL(synx_get_status);

int synx_merge(struct synx_session *session, struct synx_merge_params *params)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->merge)
		return -SYNX_INVALID;
	return session->ops->merge(session, params);
}
EXPORT_SYMBOL(synx_merge);

int synx_wait(struct synx_session *session, u32 h_synx, u64 timeout_ms)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->wait)
		return -SYNX_INVALID;
	return session->ops->wait(session, h_synx, timeout_ms);
}
EXPORT_SYMBOL(synx_wait);

int synx_cancel_async_wait(struct synx_session *session,
	struct synx_callback_params *params)
{
	if (IS_ERR_OR_NULL(session) || !session->ops || !session->ops->cancel_async_wait)
		return -SYNX_INVALID;
	return session->ops->cancel_async_wait(session, params);
}
EXPORT_SYMBOL(synx_cancel_async_wait);

int synx_enable_resources(enum synx_client_id id, enum synx_resource_type resource, bool enable)
{
	int ret = 0;

	if (is_hw_fence_client(id))
		ret = synx_hwfence_enable_resources(id, resource, enable);
	else
		ret = SYNX_SUCCESS; /* no resources to enable for native synx clients */
	return ret;
}
EXPORT_SYMBOL_GPL(synx_enable_resources);
