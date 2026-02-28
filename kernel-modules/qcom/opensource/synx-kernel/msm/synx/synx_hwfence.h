/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SYNX_HW_FENCE_H
#define __SYNX_HW_FENCE_H

#if IS_ENABLED(CONFIG_QTI_HW_FENCE)
/**
 * synx_hwfence_init_ops - Initialize hw-fence operations in synx ops
 *
 * Populates function pointers in synx_ops struct
 *
 * @param ops : Pointer to synx ops
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_hwfence_init_ops(struct synx_ops *ops);

/**
 * synx_hwfence_initialize - Initializes a new client session
 *
 * @param params : Pointer to session init params
 *
 * @return Client session pointer on success. NULL or error in case of failure.
 */
struct synx_session *synx_hwfence_initialize(struct synx_initialization_params *params);

/**
 * synx_hwfence_recover - Recover any possible handle leaks
 *
 * Function should be called on HW hang/reset to
 * recover the Synx handles shared. This cleans up
 * Synx handles held by the rest HW, and avoids
 * potential resource leaks.
 *
 * Function does not destroy the session, but only
 * recover synx handles belonging to the session.
 * Synx session would still be active and clients
 * need to destroy the session explicitly through
 * synx_uninitialize API.
 *
 * @param id : Client ID of core to recover
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_hwfence_recover(enum synx_client_id id);

/**
 * synx_hwfence_enable_resources - enable any resources needed
 * for the synx client
 *
 * Function should be called with enable=true when
 * client is using fences and with enable=false when
 * client is not using fences, e.g. at use-case boundary.
 *
 * @param id : Client ID of core for which resources are enabled
 * @param resource : type of synx resource to enable
 * @param enable : true if enabling resources, false to disable resources
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_hwfence_enable_resources(enum synx_client_id id, enum synx_resource_type resource,
	bool enable);

#else /* CONFIG_QTI_HW_FENCE */
static inline int synx_hwfence_init_ops(struct synx_ops *ops)
{
	return -SYNX_INVALID;
}

static inline struct synx_session *synx_hwfence_initialize(
	struct synx_initialization_params *params)
{
	return ERR_PTR(-SYNX_INVALID);
}

static inline int synx_hwfence_recover(enum synx_client_id id)
{
	return -SYNX_INVALID;
}

static inline int synx_hwfence_enable_resources(enum synx_client_id id,
	enum synx_resource_type resource, bool enable)
{
	return -SYNX_INVALID;
}

#endif /* CONFIG_QTI_HW_FENCE */
#endif /* __SYNX_HW_FENCE_H */
