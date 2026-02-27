/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __CAM_SYNC_PRIVATE_H__
#define __CAM_SYNC_PRIVATE_H__

#include <linux/bitmap.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include "cam_sync_api.h"
#include "cam_sync_dma_fence.h"

#if IS_ENABLED(CONFIG_TARGET_SYNX_ENABLE)
#include "cam_sync_synx.h"
#endif

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX) || IS_ENABLED(CONFIG_TARGET_SYNX_ENABLE)
#include <synx_api.h>
#endif

#ifdef CONFIG_CAM_SYNC_DBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define CAM_SYNC_OBJ_NAME_LEN           64
#define CAM_SYNC_MAX_OBJS               2048
#define CAM_GENERIC_FENCE_BATCH_MAX     10
#define CAM_SYNC_MAX_V4L2_EVENTS        250
#define CAM_SYNC_DEBUG_FILENAME         "cam_debug"
#define CAM_SYNC_DEBUG_BASEDIR          "cam"
#define CAM_SYNC_DEBUG_BUF_SIZE         32
#define CAM_SYNC_PAYLOAD_WORDS          2
#define CAM_SYNC_NAME                   "cam_sync"
#define CAM_SYNC_WORKQUEUE_NAME         "HIPRIO_SYNC_WORK_QUEUE"

#define CAM_SYNC_TYPE_INDV              0
#define CAM_SYNC_TYPE_GROUP             1

/* Number of monitor table elements */
#define CAM_SYNC_MONITOR_TABLE_SIZE     16
/* Number of monitored objects per table entry */
#define CAM_SYNC_MONITOR_TABLE_ENTRY_SZ (CAM_SYNC_MAX_OBJS / CAM_SYNC_MONITOR_TABLE_SIZE)
#define CAM_SYNC_MONITOR_MAX_ENTRIES    30
#define CAM_SYNC_INC_MONITOR_HEAD(head, ret) \
	div_u64_rem(atomic64_add_return(1, head),\
	CAM_SYNC_MONITOR_MAX_ENTRIES, (ret))
#define CAM_SYNC_MONITOR_GET_DATA(idx)  \
	(sync_dev->mon_data[idx / CAM_SYNC_MONITOR_TABLE_ENTRY_SZ] + \
	(idx % CAM_SYNC_MONITOR_TABLE_ENTRY_SZ))
#define CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ CAM_SYNC_MONITOR_TABLE_ENTRY_SZ
#define CAM_GENERIC_MONITOR_GET_DATA(mon_data, idx) \
	((mon_data)[idx / CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ] + \
	(idx % CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ))

/**
 * Feature is enabled by setting BIT(fence_type), this will trigger the fence
 * dumps on any error, to explicitly trigger a dump on every fence release
 * below BIT(fence_type_dump) needs to be used at the same time
 */
#define CAM_GENERIC_FENCE_DUMP_ALWAYS   0x10
#define CAM_GENERIC_FENCE_TYPE_SYNC_OBJ_DUMP \
	(CAM_GENERIC_FENCE_TYPE_SYNC_OBJ + (CAM_GENERIC_FENCE_DUMP_ALWAYS))
#define CAM_GENERIC_FENCE_TYPE_DMA_FENCE_DUMP \
	(CAM_GENERIC_FENCE_TYPE_DMA_FENCE + (CAM_GENERIC_FENCE_DUMP_ALWAYS))
#define CAM_GENERIC_FENCE_TYPE_SYNX_OBJ_DUMP \
	(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ + (CAM_GENERIC_FENCE_DUMP_ALWAYS))

/**
 * enum sync_type - Enum to indicate the type of sync object,
 * i.e. individual or group.
 *
 * @SYNC_TYPE_INDV  : Object is an individual sync object
 * @SYNC_TYPE_GROUP : Object is a group sync object
 */
enum sync_type {
	SYNC_TYPE_INDV,
	SYNC_TYPE_GROUP
};

/**
 * enum sync_list_clean_type - Enum to indicate the type of list clean action
 * to be peformed, i.e. specific sync ID or all list sync ids.
 *
 * @SYNC_CLEAN_ONE : Specific object to be cleaned in the list
 * @SYNC_CLEAN_ALL : Clean all objects in the list
 */
enum sync_list_clean_type {
	SYNC_LIST_CLEAN_ONE,
	SYNC_LIST_CLEAN_ALL
};

/**
 * struct sync_parent_info - Single node of information about a parent
 * of a sync object, usually part of the parents linked list
 *
 * @sync_id  : Sync object id of parent
 * @list     : List member used to append this node to a linked list
 */
struct sync_parent_info {
	int32_t sync_id;
	struct list_head list;
};

/**
 * struct sync_parent_info - Single node of information about a child
 * of a sync object, usually part of the children linked list
 *
 * @sync_id  : Sync object id of child
 * @list     : List member used to append this node to a linked list
 */
struct sync_child_info {
	int32_t sync_id;
	struct list_head list;
};


/**
 * struct sync_callback_info - Single node of information about a kernel
 * callback registered on a sync object
 *
 * @callback_func      : Callback function, registered by client driver
 * @cb_data            : Callback data, registered by client driver
 * @status             : Status with which callback will be invoked in client
 * @sync_obj           : Sync id of the object for which callback is registered
 * @workq_scheduled_ts : workqueue scheduled timestamp
 * @cb_dispatch_work   : Work representing the call dispatch
 * @list               : List member used to append this node to a linked list
 */
struct sync_callback_info {
	sync_callback callback_func;
	void *cb_data;
	int status;
	int32_t sync_obj;
	ktime_t workq_scheduled_ts;
	struct work_struct cb_dispatch_work;
	struct list_head list;
};

/**
 * struct sync_user_payload - Single node of information about a user space
 * payload registered from user space
 *
 * @payload_data    : Payload data, opaque to kernel
 * @list            : List member used to append this node to a linked list
 */
struct sync_user_payload {
	uint64_t payload_data[CAM_SYNC_PAYLOAD_WORDS];
	struct list_head list;
};

/**
 * struct sync_dma_fence_info - DMA fence info associated with this sync obj
 *
 * @dma_fence_fd          : DMA fence fd
 * @dma_fence_row_idx     : Index of the row corresponding to this dma fence
 *                          in the dma fence table
 * @sync_created_with_dma : If sync obj and dma fence are created together
 */
struct sync_dma_fence_info {
	int32_t dma_fence_fd;
	int32_t dma_fence_row_idx;
	bool    sync_created_with_dma;
};

/**
 * enum cam_fence_op - Enum to indicate the type of operation performed
 *
 * @CAM_FENCE_OP_CREATE                : Created obj
 * @CAM_FENCE_OP_REGISTER_CB           : Successful callback registration
 * @CAM_FENCE_OP_SKIP_REGISTER_CB      : Callback registration skipped
 * @CAM_FENCE_OP_ALREADY_REGISTERED_CB : Callback already registered
 * @CAM_FENCE_OP_SIGNAL                : Signaled obj
 * @CAM_FENCE_OP_UNREGISTER_ON_SIGNAL  : Callback unregistered after signaling
 * @CAM_FENCE_OP_UNREGISTER_CB         : Callback unregistered
 * @CAM_FENCE_OP_DESTROY               : Destroyed obj
 */
enum cam_fence_op {
	CAM_FENCE_OP_CREATE,
	CAM_FENCE_OP_REGISTER_CB,
	CAM_FENCE_OP_SKIP_REGISTER_CB,
	CAM_FENCE_OP_ALREADY_REGISTERED_CB,
	CAM_FENCE_OP_SIGNAL,
	CAM_FENCE_OP_UNREGISTER_ON_SIGNAL,
	CAM_FENCE_OP_UNREGISTER_CB,
	CAM_FENCE_OP_DESTROY,
};

/**
 * struct cam_generic_fence_monitor_entry - Single operation sync data
 *
 * @timestamp     : Timestamp of op
 * @op            : Operation id
 */
struct cam_generic_fence_monitor_entry {
	struct timespec64 timestamp;
	enum cam_fence_op op;
};

/**
 * struct cam_generic_fence_monitor_data - All operations data from current &
 *                          previous use of a fence object
 *
 * @monitor_head         : Executed operations count
 * @prev_name            : Previous name of this fence obj
 * @prev_type            : Previous type of this fence obj
 * @prev_obj_id          : Previous handle of this fence obj
 * @prev_sync_id         : Previous handle of this fence's associated sync obj
 * @prev_remaining       : Previous count of remaining children that not been
 *                         signaled
 * @prev_state           : Previous state (INVALID, ACTIVE, SIGNALED_SUCCESS or
 *                         SIGNALED_ERROR)
 * @prev_monitor_head    : Previous executed ops count
 * @swap_monitor_entries : Flag indicating which entry table should be used
 *                         as current/previous. Used to avoid copying.
 * @monitor_entries      : Op info entry table
 * @prev_monitor_entries : Previous op info entry table
 */
struct cam_generic_fence_monitor_data {
	atomic64_t                        monitor_head;
	char                              prev_name[CAM_DMA_FENCE_NAME_LEN];
	enum sync_type                    prev_type;
	int32_t                           prev_obj_id;
	int32_t                           prev_sync_id;
	uint32_t                          prev_remaining;
	uint32_t                          prev_state;
	uint64_t                          prev_monitor_head;
	bool                              swap_monitor_entries;
	struct cam_generic_fence_monitor_entry monitor_entries[
		CAM_SYNC_MONITOR_MAX_ENTRIES];
	struct cam_generic_fence_monitor_entry prev_monitor_entries[
		CAM_SYNC_MONITOR_MAX_ENTRIES];
};

/**
 * struct cam_generic_fence_monitor_obj_info - Single object monitor info
 *
 * @name                 : Name of this fence obj
 * @sync_type            : Type of this fence obj
 * @obj_id               : Handle of this fence obj
 * @sync_id              : Handle of this fence's associated sync obj
 * @state                : Previous state (INVALID, ACTIVE, SIGNALED_SUCCESS or
 *                         SIGNALED_ERROR)
 * @remaining            : Count of remaining children that not been signaled
 * @ref_cnt              : Ref count of the number of usage of the fence.
 * @fence_type           : Fence type - DMA/Sync/Synx
 * @monitor_data         : Fence operations data
 * @monitor_entries      : Op info entry table
 * @prev_monitor_entries : Previous op info entry table
 */
struct cam_generic_fence_monitor_obj_info {
	char *name;
	enum sync_type sync_type;
	int32_t obj_id;
	int32_t sync_id;
	uint32_t state;
	uint32_t remaining;
	uint32_t ref_cnt;
	uint32_t fence_type;
	struct cam_generic_fence_monitor_data *monitor_data;
	struct cam_generic_fence_monitor_entry *monitor_entries;
	struct cam_generic_fence_monitor_entry *prev_monitor_entries;
};

/**
 * struct sync_synx_obj_info - Synx object info associated with this sync obj
 *
 * @synx_obj               : Synx object handle
 * @synx_obj_row_idx       : Index of the row corresponding to this synx obj
 *                           in the synx obj table
 * @sync_created_with_synx : If sync obj and synx obj are created together
 */
struct sync_synx_obj_info {
	uint32_t synx_obj;
	int32_t  synx_obj_row_idx;
	bool     sync_created_with_synx;
};

/**
 * struct sync_table_row - Single row of information about a sync object, used
 * for internal book keeping in the sync driver
 *
 * @name               : Optional string representation of the sync object
 * @type               : Type of the sync object (individual or group)
 * @sync_id            : Integer id representing this sync object
 * @parents_list       : Linked list of parents of this sync object
 * @children_list      : Linked list of children of this sync object
 * @state              : State (INVALID, ACTIVE, SIGNALED_SUCCESS or
 *                       SIGNALED_ERROR)
 * @remaining          : Count of remaining children that not been signaled
 * @signaled           : Completion variable on which block calls will wait
 * @callback_list      : Linked list of kernel callbacks registered
 * @user_payload_list  : LInked list of user space payloads registered
 * @ref_cnt            : ref count of the number of usage of the fence.
 * @ext_fence_mask     : Mask to indicate associated external fence types
 * @dma_fence_info     : dma fence info if associated
 * @synx_obj_info      : synx obj info if associated
 */
struct sync_table_row {
	char name[CAM_SYNC_OBJ_NAME_LEN];
	enum sync_type type;
	int32_t sync_id;
	/* List of parents, which are merged objects */
	struct list_head parents_list;
	/* List of children, which constitute the merged object */
	struct list_head children_list;
	uint32_t state;
	uint32_t remaining;
	struct completion signaled;
	struct list_head callback_list;
	struct list_head user_payload_list;
	atomic_t ref_cnt;
	unsigned long ext_fence_mask;
	struct sync_dma_fence_info dma_fence_info;
	struct sync_synx_obj_info synx_obj_info;
};

/**
 * struct cam_signalable_info - Information for a single sync object that is
 * ready to be signaled
 *
 * @sync_obj : Sync object id of signalable object
 * @status   : Status with which to signal
 * @list     : List member used to append this node to a linked list
 */
struct cam_signalable_info {
	int32_t sync_obj;
	uint32_t status;
	struct list_head list;
};

/**
 * struct sync_device - Internal struct to book keep sync driver details
 *
 * @vdev            : Video device
 * @v4l2_dev        : V4L2 device
 * @sync_table      : Table of all sync objects allocated when driver initializes
 * @row_spinlocks   : Spinlock array, one for each row in the table
 * @table_lock      : Mutex used to lock the table
 * @open_cnt        : Count of file open calls made on the sync driver
 * @dentry          : Debugfs entry
 * @work_queue      : Work queue used for dispatching kernel callbacks
 * @cam_sync_eventq : Event queue used to dispatch user payloads to user space
 * @bitmap          : Bitmap representation of all sync objects
 * @mon_data        : Objects monitor data
 * @params          : Parameters for synx call back registration
 * @version         : version support
 */
struct sync_device {
	struct video_device *vdev;
	struct v4l2_device v4l2_dev;
	struct sync_table_row *sync_table;
	spinlock_t row_spinlocks[CAM_SYNC_MAX_OBJS];
	struct mutex table_lock;
	int open_cnt;
	struct dentry *dentry;
	struct workqueue_struct *work_queue;
	struct v4l2_fh *cam_sync_eventq;
	spinlock_t cam_sync_eventq_lock;
	DECLARE_BITMAP(bitmap, CAM_SYNC_MAX_OBJS);
	struct cam_generic_fence_monitor_data **mon_data;
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
	struct synx_register_params params;
#endif
	uint32_t version;
};


#endif /* __CAM_SYNC_PRIVATE_H__ */
