/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __CAM_SYNC_SYNX_H__
#define __CAM_SYNC_SYNX_H__

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/bitmap.h>
#include <synx_api.h>

#include "cam_sync_api.h"
#include "cam_sync.h"
#include "cam_debug_util.h"

#define CAM_SYNX_MAX_OBJS 256
#define CAM_SYNX_OBJ_NAME_LEN 64
#define CAM_SYNX_TABLE_SZ (CAM_SYNX_MAX_OBJS / CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ)

/* Synx obj state */
enum cam_synx_obj_state {
	CAM_SYNX_OBJ_STATE_INVALID,
	CAM_SYNX_OBJ_STATE_ACTIVE,
	CAM_SYNX_OBJ_STATE_SIGNALED,
};

/**
 * struct cam_synx_obj_release_params - Synx release payload
 *                   Based on the flag row_idx or synx_obj is consumed
 *
 * @synx_row_idx   : Synx obj row idx
 * @synx_obj       : Synx object handle
 * @use_row_idx    : Use row idx
 */
struct cam_synx_obj_release_params {
	union {
		int32_t  synx_row_idx;
		uint32_t synx_obj;
	} u;
	bool use_row_idx;
};

/**
 * struct cam_synx_obj_fence_signal_sync_obj - SYNX -> sync signal info
 *                           Payload to signal sync on a synx fence being signaled
 *
 * @synx_obj               : Synx object handle
 * @status                 : Sync signal status
 */
struct cam_synx_obj_signal_sync_obj {
	int32_t synx_obj;
	int32_t status;
};

/* Synx obj callback function type */
typedef int (*cam_sync_callback_for_synx_obj)(int32_t sync_obj,
	struct cam_synx_obj_signal_sync_obj *signal_sync_obj);

/**
 * @brief Find the synx obj in the device's table
 *
 * @param synx_obj       : Synx obj
 * @param idx            : Synx object table index
 *
 * @return Status of operation. Zero in case of success.
 */
int cam_synx_obj_find_obj_in_table(uint32_t synx_obj, int32_t *idx);

/**
 * @brief Create a synx object
 *
 * @param name     : Synx obj name
 * @param flags    : Creation flags
 * @param synx_obj : Created synx obj handle
 * @param row_idx  : Created synx obj table row idx
 *
 * @return Status of operation. Zero in case of success.
 * -EINVAL will be returned if params were invalid.
 * -ENOMEM will be returned if the kernel can't allocate space for
 * synx object.
 */
int cam_synx_obj_create(const char *name, uint32_t flags, uint32_t *synx_obj,
	int32_t *row_idx);

/**
 * @brief Signal a synx obj when sync obj is signaled
 *
 * @param row_idx         : Synx obj table row index
 * @param signal_synx_obj : Info on synx obj to be signaled
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_synx_obj_internal_signal(int32_t row_idx,
	struct cam_synx_obj_signal *signal_synx_obj);

/**
 * @brief Import a synx obj for synchronization
 *
 * @param name     : Synx obj name
 * @param flags    : Import flags
 * @param fence    : DMA fence ptr
 * @param synx_obj : New synx obj handle
 * @param row_idx  : Imported obj table row idx
 *
 * @return Status of operation. Zero in case of success
 * -EINVAL if synx object is bad state
 */
int cam_synx_obj_import_dma_fence(const char *name, uint32_t flags, void *fence,
	uint32_t *synx_obj, int32_t *row_idx);

/**
 * @brief Release a synx object
 *
 * @param release_params : Synx obj release info
 *
 * @return Status of operation. Zero upon success. Negative value otherwise
 */
int cam_synx_obj_release(struct cam_synx_obj_release_params *release_params);

/**
 * @brief Signal a synx obj [userspace API]
 *
 * @param signal_synx_obj : Signal info
 *
 * @return Status of operation. Zero upon success. Negative value otherwise
 */
int cam_synx_obj_signal_obj(struct cam_synx_obj_signal *signal_synx_obj);

/**
 * @brief Synx obj register callback
 *
 * @param sync_obj : Sync object
 * @param row_idx  : Synx obj table row idx
 * @param sync_cb  : Sync object callback
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_synx_obj_register_cb(int32_t *sync_obj, int32_t row_idx,
	cam_sync_callback_for_synx_obj sync_cb);

/**
 * @brief: Synx recovery for a given core
 *
 * @param core_id: Core ID we want to recover for
 *
 * @return Status of operation. Zero in case of success
 * -EINVAL if core_id is invalid
 */
int cam_synx_core_recovery(
	enum cam_sync_synx_supported_cores core_id);

/**
 * @brief: cam synx driver open
 *
 */
void cam_synx_obj_open(void);

/**
 * @brief: cam synx driver close
 *
 */
void cam_synx_obj_close(void);

/**
 * @brief: cam synx driver initialize
 *
 */
int cam_synx_obj_driver_init(void);

/**
 * @brief: cam synx driver deinit
 *
 */
void cam_synx_obj_driver_deinit(void);

#endif /* __CAM_SYNC_SYNX_H__ */
