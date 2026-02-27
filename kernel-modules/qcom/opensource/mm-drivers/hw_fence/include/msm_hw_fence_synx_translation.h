/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __MSM_HW_FENCE_SYNX_TRANSLATION_H
#define __MSM_HW_FENCE_SYNX_TRANSLATION_H

#include <synx_api.h>

extern bool hw_fence_driver_enable;

#ifndef SYNX_HW_FENCE_CLIENT_START
#define SYNX_HW_FENCE_CLIENT_START 1024
#define SYNX_HW_FENCE_CLIENT_END 4096
#define SYNX_MAX_SIGNAL_PER_CLIENT 64

/**
 * enum synx_client_id : Unique identifier of the supported clients
 *
 * @SYNX_CLIENT_HW_FENCE_GFX_CTX0 : HW Fence GFX Client 0
 * @SYNX_CLIENT_HW_FENCE_IPE_CTX0 : HW Fence IPE Client 0
 * @SYNX_CLIENT_HW_FENCE_VID_CTX0 : HW Fence Video Client 0
 * @SYNX_CLIENT_HW_FENCE_DPU0_CTL0 : HW Fence DPU0 Client 0
 * @SYNX_CLIENT_HW_FENCE_DPU1_CTL0 : HW Fence DPU1 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE0_CTX0 : HW Fence IFE0 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE1_CTX0 : HW Fence IFE1 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE2_CTX0 : HW Fence IFE2 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE3_CTX0 : HW Fence IFE3 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE4_CTX0 : HW Fence IFE4 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE5_CTX0 : HW Fence IFE5 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE6_CTX0 : HW Fence IFE6 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE7_CTX0 : HW Fence IFE7 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE8_CTX0 : HW Fence IFE8 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE9_CTX0 : HW Fence IFE9 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE10_CTX0 : HW Fence IFE10 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE11_CTX0 : HW Fence IFE11 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE12_CTX0 : HW Fence IFE12 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE13_CTX0 : HW Fence IFE13 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE14_CTX0 : HW Fence IFE14 Client 0
 * @SYNX_CLIENT_HW_FENCE_IFE15_CTX0 : HW Fence IFE15 Client 0
 */
enum synx_hwfence_client_id {
	SYNX_CLIENT_HW_FENCE_GFX_CTX0 = SYNX_HW_FENCE_CLIENT_START,
	SYNX_CLIENT_HW_FENCE_IPE_CTX0 = SYNX_CLIENT_HW_FENCE_GFX_CTX0 + SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_VID_CTX0 = SYNX_CLIENT_HW_FENCE_IPE_CTX0 + SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_DPU0_CTL0 = SYNX_CLIENT_HW_FENCE_VID_CTX0 + SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_DPU1_CTL0 = SYNX_CLIENT_HW_FENCE_DPU0_CTL0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE0_CTX0 = SYNX_CLIENT_HW_FENCE_DPU1_CTL0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE1_CTX0 = SYNX_CLIENT_HW_FENCE_IFE0_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE2_CTX0 = SYNX_CLIENT_HW_FENCE_IFE1_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE3_CTX0 = SYNX_CLIENT_HW_FENCE_IFE2_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE4_CTX0 = SYNX_CLIENT_HW_FENCE_IFE3_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE5_CTX0 = SYNX_CLIENT_HW_FENCE_IFE4_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE6_CTX0 = SYNX_CLIENT_HW_FENCE_IFE5_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE7_CTX0 = SYNX_CLIENT_HW_FENCE_IFE6_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE8_CTX0 = SYNX_CLIENT_HW_FENCE_IFE7_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE9_CTX0 = SYNX_CLIENT_HW_FENCE_IFE8_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE10_CTX0 = SYNX_CLIENT_HW_FENCE_IFE9_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE11_CTX0 = SYNX_CLIENT_HW_FENCE_IFE10_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE12_CTX0 = SYNX_CLIENT_HW_FENCE_IFE11_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE13_CTX0 = SYNX_CLIENT_HW_FENCE_IFE12_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE14_CTX0 = SYNX_CLIENT_HW_FENCE_IFE13_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_IFE15_CTX0 = SYNX_CLIENT_HW_FENCE_IFE14_CTX0 +
		SYNX_MAX_SIGNAL_PER_CLIENT,
	SYNX_CLIENT_HW_FENCE_MAX = SYNX_HW_FENCE_CLIENT_END,
};
#endif

#if IS_ENABLED(CONFIG_QTI_HW_FENCE)
/**
 * synx_hwfence_initialize - Initializes a new client session
 *
 * @param params : Pointer to session init params
 *
 * @return Client session pointer on success. NULL or error in case of failure.
 */
struct synx_session *synx_hwfence_initialize(struct synx_initialization_params *params);

/**
 * synx_hwfence_uninitialize - Destroys the client session
 *
 * @param session : Session ptr (returned from synx_initialize)
 *
 * @return Status of operation. SYNX_SUCCESS in case of success.
 */
int synx_hwfence_uninitialize(struct synx_session *session);

/**
 * synx_hwfence_create - Creates a synx object
 *
 *  Creates a new synx obj and returns the handle to client.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param params  : Pointer to create params
 *
 * @return Status of operation. SYNX_SUCCESS in case of success.
 * -SYNX_INVALID will be returned if params were invalid.
 * -SYNX_NOMEM will be returned if the kernel can't allocate space for
 * synx object.
 */
int synx_hwfence_create(struct synx_session *session, struct synx_create_params *params);

/**
 * synx_hwfence_release - Release the synx object
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param h_synx  : Synx object handle to be destroyed
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_hwfence_release(struct synx_session *session, u32 h_synx);

/**
 * synx_hwfence_signal - Signals a synx object with the status argument.
 *
 * This function will signal the synx object referenced by h_synx
 * and invoke any external binding synx objs.
 * The status parameter will indicate whether the entity
 * performing the signaling wants to convey an error case or a success case.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param h_synx  : Synx object handle
 * @param status  : Status of signaling.
 *                  Clients can send custom signaling status
 *                  beyond SYNX_STATE_SIGNALED_MAX.
 *
 * @return Status of operation. Negative in case of error. SYNX_SUCCESS otherwise.
 */
int synx_hwfence_signal(struct synx_session *session, u32 h_synx, enum synx_signal_status status);

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
 * synx_hwfence_import - Imports (looks up) synx object from given handle/fence
 *
 * Import subscribes the client session for notification on signal
 * of handles/fences.
 *
 * @param session : Session ptr (returned from synx_initialize)
 * @param params  : Pointer to import params
 *
 * @return SYNX_SUCCESS upon success, -SYNX_INVAL if synx object is bad state
 */
int synx_hwfence_import(struct synx_session *session, struct synx_import_params *params);

#else /* CONFIG_QTI_HW_FENCE */
static inline struct synx_session *synx_hwfence_initialize(
	struct synx_initialization_params *params)
{
	return ERR_PTR(-SYNX_INVALID);
}

static inline int synx_hwfence_uninitialize(struct synx_session *session)
{
	return -SYNX_INVALID;
}

static inline int synx_hwfence_create(struct synx_session *session,
	struct synx_create_params *params)
{
	return -SYNX_INVALID;
}

static inline int synx_hwfence_release(struct synx_session *session, u32 h_synx)
{
	return -SYNX_INVALID;
}

static inline int synx_hwfence_signal(struct synx_session *session, u32 h_synx,
	enum synx_signal_status status)
{
	return -SYNX_INVALID;
}

static inline int synx_hwfence_recover(enum synx_client_id id)
{
	return -SYNX_INVALID;
}

static inline int synx_hwfence_import(struct synx_session *session,
	struct synx_import_params *params)
{
	return -SYNX_INVALID;
}

#endif /* CONFIG_QTI_HW_FENCE */
#endif /* __MSM_HW_FENCE_SYNX_TRANSLATION_H */
