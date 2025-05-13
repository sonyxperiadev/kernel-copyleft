/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __CAM_SYNC_UTIL_H__
#define __CAM_SYNC_UTIL_H__


#include "cam_sync_private.h"
#include "cam_debug_util.h"

extern struct sync_device *sync_dev;

/**
 * struct cam_sync_check_for_dma_release -
 *                          Checks if the dma fence being released
 *                          was created with the sync obj
 *
 * @dma_fence_row_idx     : Get DMA fence row idx that is associated with
 *                          the sync obj
 * @dma_fence_fd          : Check if DMA fence fd is associated with
 *                          sync obj
 * @sync_created_with_dma : Set if the dma fence fd was created
 *                          with sync obj
 */
struct cam_sync_check_for_dma_release {
	int32_t dma_fence_row_idx;
	int32_t dma_fence_fd;
	bool sync_created_with_dma;
};

/**
 * struct cam_sync_check_for_synx_release -
 *                          Checks if the synx obj being released
 *                          was created with the sync obj
 *
 * @synx_obj               : Check if synx obj is associated with
 *                           sync obj
 * @synx_obj_row_idx       : Get synx obj row idx that is associated with
 *                           the sync obj
 * @sync_created_with_synx : Set if the dma fence fd was created
 *                           with sync obj
 */
struct cam_sync_check_for_synx_release {
	int32_t synx_obj;
	int32_t synx_obj_row_idx;
	bool sync_created_with_synx;
};

/**
 * @brief: Finds an empty row in the sync table and sets its corresponding bit
 * in the bit array
 *
 * @param sync_dev : Pointer to the sync device instance
 * @param idx      : Pointer to an long containing the index found in the bit
 *                   array
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_util_find_and_set_empty_row(struct sync_device *sync_dev,
	long *idx);

/**
 * @brief: Function to initialize an empty row in the sync table. This should be
 *         called only for individual sync objects.
 *
 * @param table : Pointer to the sync objects table
 * @param idx   : Index of row to initialize
 * @param name  : Optional string representation of the sync object. Should be
 *                63 characters or less
 * @param type  : type of row to be initialized
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_init_row(struct sync_table_row *table,
	uint32_t idx, const char *name, uint32_t type);

/**
 * @brief: Function to uninitialize a row in the sync table
 *
 * @param table                           : Pointer to the sync objects table
 * @param idx                             : Index of row to initialize
 * @optional param check_for_dma_release  : checks for dma fence release
 * @optional param check_for_synx_release : checks for synx obj release
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_deinit_object(struct sync_table_row *table, uint32_t idx,
	struct cam_sync_check_for_dma_release *check_for_dma_release,
	struct cam_sync_check_for_synx_release *check_for_synx_release);

/**
 * @brief: Function to initialize a row in the sync table when the object is a
 *         group object, also known as a merged sync object
 *
 * @param table     : Pointer to the sync objects table
 * @param idx       : Index of row to initialize
 * @param sync_objs : Array of sync objects which will merged
 *                    or grouped together
 * @param num_objs  : Number of sync objects in the array
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_init_group_object(struct sync_table_row *table,
	uint32_t idx,
	uint32_t *sync_objs,
	uint32_t num_objs);

/**
 * @brief: Function to dispatch a kernel callback for a sync callback
 *
 * @param cb_dispatch_work : Pointer to the work_struct that needs to be
 *                           dispatched
 *
 * @return None
 */
void cam_sync_util_cb_dispatch(struct work_struct *cb_dispatch_work);

/**
 * @brief: Function to dispatch callbacks for a signaled sync object
 *
 * @sync_obj    : Sync object that is signaled
 * @status      : Status of the signaled object
 * @evt_param   : Event paramaeter
 *
 * @return None
 */
void cam_sync_util_dispatch_signaled_cb(int32_t sync_obj,
	uint32_t status, uint32_t evt_param);

/**
 * @brief: Function to send V4L event to user space
 * @param id       : V4L event id to send
 * @param sync_obj : Sync obj for which event needs to be sent
 * @param status   : Status of the event
 * @payload        : Payload that needs to be sent to user space
 * @len            : Length of the payload
 * @evt_param      : Event Paramenter
 *
 * @return None
 */
void cam_sync_util_send_v4l2_event(uint32_t id,
	uint32_t sync_obj,
	int status,
	void *payload,
	int len,
	uint32_t evt_param);

/**
 * @brief: Function which gets the next state of the sync object based on the
 *         current state and the new state
 *
 * @param current_state : Current state of the sync object
 * @param new_state     : New state of the sync object
 *
 * @return Next state of the sync object
 */
int cam_sync_util_update_parent_state(struct sync_table_row *parent_row,
	int new_state);

/**
 * @brief: Function to clean up the children of a sync object
 * @row                 : Row whose child list to clean
 * @list_clean_type     : Clean specific object or clean all objects
 * @sync_obj            : Sync object to be clean if list clean type is
 *                          SYNC_LIST_CLEAN_ONE
 *
 * @return None
 */
void cam_sync_util_cleanup_children_list(struct sync_table_row *row,
	uint32_t list_clean_type, uint32_t sync_obj);

/**
 * @brief: Function to clean up the parents of a sync object
 * @row                 : Row whose parent list to clean
 * @list_clean_type     : Clean specific object or clean all objects
 * @sync_obj            : Sync object to be clean if list clean type is
 *                          SYNC_LIST_CLEAN_ONE
 *
 * @return None
 */
void cam_sync_util_cleanup_parents_list(struct sync_table_row *row,
	uint32_t list_clean_type, uint32_t sync_obj);

/**
 * @brief: Function to dump sync obj & monitor data
 * @row                 : Row whose data to dump
 *
 * @return None
 */
void cam_sync_dump_monitor_array(struct sync_table_row *row);

/**
 * @brief: Function to add a new entry to the monitor table
 * @idx                 : Index of row to update
 * @mutex               : Mutex lock when expand monitor table
 * @mon_data            : Pointer to the monitor data array
 * @op                  : Operation id
 *
 * @return None
 */
void cam_generic_fence_update_monitor_array(int idx,
	struct mutex *lock,
	struct cam_generic_fence_monitor_data **mon_data,
	enum cam_fence_op                op);

/**
 * @brief: Function to dump monitor array for sync/dma/synx
 * @obj_info             : Monitor object that needs to be dumped
 *
 * @return None
 */
void cam_generic_fence_dump_monitor_array(
	struct cam_generic_fence_monitor_obj_info *obj_info);

#endif /* __CAM_SYNC_UTIL_H__ */
