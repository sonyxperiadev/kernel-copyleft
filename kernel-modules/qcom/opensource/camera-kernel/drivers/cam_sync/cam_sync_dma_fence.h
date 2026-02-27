/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __CAM_SYNC_DMA_FENCE_H__
#define __CAM_SYNC_DMA_FENCE_H__

#include <linux/dma-fence.h>
#include <linux/spinlock_types.h>
#include <linux/sync_file.h>
#include <linux/file.h>
#include <linux/bitmap.h>

#include "cam_sync.h"
#include "cam_debug_util.h"

#define CAM_DMA_FENCE_MAX_FENCES  128
#define CAM_DMA_FENCE_NAME_LEN    128
#define CAM_DMA_FENCE_TABLE_SZ (CAM_DMA_FENCE_MAX_FENCES / CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ)

/* DMA fence state */
enum cam_dma_fence_state {
	CAM_DMA_FENCE_STATE_INVALID,
	CAM_DMA_FENCE_STATE_ACTIVE,
	CAM_DMA_FENCE_STATE_SIGNALED,
};

/**
 * struct cam_dma_fence_release_params - DMA release payload
 *                     Based on the flag row_idx or fd is consumed
 *
 * @dma_row_idx      : DMA fence row idx
 * @dma_fence_fd     : DMA fence fd
 * @use_row_idx      : Use row idx
 */
struct cam_dma_fence_release_params {
	union {
		int32_t dma_row_idx;
		int32_t dma_fence_fd;
	} u;
	bool use_row_idx;
};

/**
 * struct cam_dma_fence_signal_sync_obj - DMA -> sync signal info
 *                           Payload to signal sync on a dma fence
 *                           being signaled
 *
 * @status                 : DMA fence status
 * @fd                     : DMA fence fd if any
 */
struct cam_dma_fence_signal_sync_obj {
	int32_t status;
	int32_t fd;
};

/* DMA fence callback function type */
typedef int (*cam_sync_callback_for_dma_fence)(int32_t sync_obj,
	struct cam_dma_fence_signal_sync_obj *signal_sync_obj);

/**
 * struct cam_dma_fence_create_sync_obj_payload -
 *                           Payload to create sync for a dma fence
 *
 * @dma_fence_row_idx      : DMA fence row idx
 * @fd                     : DMA fence fd
 * @sync_created_with_dma  : Set if dma fence and sync obj are being
 *                           created in a single IOCTL call
 */
struct cam_dma_fence_create_sync_obj_payload {
	int32_t dma_fence_row_idx;
	int32_t fd;
	bool    sync_created_with_dma;
};

/**
 * @brief: Signal a dma fence fd [userspace API]
 *
 * @param signal_dma_fence: Info on DMA fence to be signaled
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_dma_fence_signal_fd(
	struct cam_dma_fence_signal *signal_dma_fence);

/**
 * @brief: Signal a dma fence when sync obj is signaled
 *
 * @param dma_fence_row_idx : DMA fence row idx
 * @param signal_dma_fence  : Info on DMA fence to be signaled
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_dma_fence_internal_signal(int32_t dma_fence_row_idx,
	struct cam_dma_fence_signal *signal_dma_fence);

/**
 * @brief: Create a dma fence fd
 *
 * @param  name              : DMA fence name, optional param
 *                             will accommodate names of length
 *                             CAM_DMA_FENCE_NAME_LEN
 * @output dma_fence_fd      : DMA fence fd
 * @output dma_fence_row_idx : Row Idx corresponding to DMA fence in the table
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_dma_fence_create_fd(
	int32_t *dma_fence_fd, int32_t *dma_fence_row_idx, const char *name);

/**
 * @brief: Release a dma fence
 *
 * @param release_params : dma fence info to be released
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_dma_fence_release(
	struct cam_dma_fence_release_params *release_params);

/**
 * @brief: Gets the dma fence from a fd, increments refcnt
 *
 * @param  fd                : File descriptor
 * @output dma_fence_row_idx : Row idx pertaining to this dma fence
 *
 * @return Status of operation. Error or valid fence.
 */
struct dma_fence *cam_dma_fence_get_fence_from_fd(int32_t fd,
	int32_t *dma_fence_row_idx);

/**
 * @brief: DMA fence register cb
 *
 * @param sync_obj      : Sync object
 * @param dma_fence_idx : DMA fence row idx
 * @param sync_cb       : Sync object callback
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_dma_fence_register_cb(int32_t *sync_obj,
	int32_t *dma_fence_row_idx, cam_sync_callback_for_dma_fence sync_cb);

/**
 * @brief: get/put on dma fence
 *
 * @get_or_put              : True for get, false for put
 * @param dma_fence_row_idx : Idx in the dma fence table pertaining to
 *                            the dma fence on which get/put ref is invoked
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_dma_fence_get_put_ref(bool get_or_put, int32_t dma_fence_row_idx);

/**
 * @brief: dma fence driver open
 *
 */
void cam_dma_fence_open(void);

/**
 * @brief: dma fence driver close
 *
 */
void cam_dma_fence_close(void);

/**
 * @brief: dma fence driver initialize
 *
 */
int cam_dma_fence_driver_init(void);

/**
 * @brief: dma fence driver deinit
 *
 */
void cam_dma_fence_driver_deinit(void);

#endif /* __CAM_SYNC_DMA_FENCE_H__ */
