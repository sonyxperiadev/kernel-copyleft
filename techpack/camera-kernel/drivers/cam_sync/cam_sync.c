// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/irqflags.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
#include <synx_api.h>
#endif
#include "cam_sync_util.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "cam_compat.h"
#include "camera_main.h"
#include "cam_req_mgr_workq.h"

struct sync_device *sync_dev;

/*
 * Flag to determine whether to enqueue cb of a
 * signaled fence onto the workq or invoke it
 * directly in the same context
 */
static bool trigger_cb_without_switch;

static void cam_sync_print_fence_table(void)
{
	int idx;

	for (idx = 0; idx < CAM_SYNC_MAX_OBJS; idx++) {
		spin_lock_bh(&sync_dev->row_spinlocks[idx]);
		CAM_INFO(CAM_SYNC,
			"index[%u]: sync_id=%d, name=%s, type=%d, state=%d, ref_cnt=%d",
			idx,
			sync_dev->sync_table[idx].sync_id,
			sync_dev->sync_table[idx].name,
			sync_dev->sync_table[idx].type,
			sync_dev->sync_table[idx].state,
			atomic_read(&sync_dev->sync_table[idx].ref_cnt));
		spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
	}
}

static int cam_sync_create_util(
	int32_t *sync_obj, const char *name,
	struct cam_dma_fence_create_sync_obj_payload *dma_sync_create_info)
{
	int rc;
	long idx;
	bool bit;
	struct sync_table_row *row = NULL;

	do {
		idx = find_first_zero_bit(sync_dev->bitmap, CAM_SYNC_MAX_OBJS);
		if (idx >= CAM_SYNC_MAX_OBJS) {
			CAM_ERR(CAM_SYNC,
				"Error: Unable to create sync idx = %d sync name = %s reached max!",
				idx, name);
			cam_sync_print_fence_table();
			return -ENOMEM;
		}

		CAM_DBG(CAM_SYNC, "Index location available at idx: %ld", idx);
		bit = test_and_set_bit(idx, sync_dev->bitmap);
	} while (bit);

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	rc = cam_sync_init_row(sync_dev->sync_table, idx, name,
		CAM_SYNC_TYPE_INDV);

	if (rc) {
		CAM_ERR(CAM_SYNC, "Error: Unable to init row at idx = %ld",
			idx);
		clear_bit(idx, sync_dev->bitmap);
		spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
		return -EINVAL;
	}

	*sync_obj = idx;

	/* Associate sync obj with dma fence if any holding sync lock */
	if (dma_sync_create_info) {
		row = sync_dev->sync_table + idx;
		row->dma_fence_info.dma_fence_fd = dma_sync_create_info->fd;
		row->dma_fence_info.dma_fence_row_idx = dma_sync_create_info->dma_fence_row_idx;
		row->dma_fence_info.sync_created_with_dma =
			dma_sync_create_info->sync_created_with_dma;
		set_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask);

		/* Association refcnt for non-import cases */
		if (dma_sync_create_info->sync_created_with_dma) {
			rc = cam_dma_fence_get_put_ref(true, row->dma_fence_info.dma_fence_row_idx);
			if (rc)
				CAM_ERR(CAM_SYNC,
					"Failed to getref on dma fence idx: %u fd: %d sync_obj: %d rc: %d",
					row->dma_fence_info.dma_fence_row_idx,
					row->dma_fence_info.dma_fence_fd,
					*sync_obj, rc);
			goto end;
		}

		CAM_DBG(CAM_SYNC, "sync_obj: %s[%d] associated with dma fence fd: %d",
			name, *sync_obj, dma_sync_create_info->fd);
		goto end;
	}

	CAM_DBG(CAM_SYNC, "sync_obj: %s[%i]", name, *sync_obj);

end:
	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
	return rc;
}

int cam_sync_create(int32_t *sync_obj, const char *name)
{
	return cam_sync_create_util(sync_obj, name, NULL);
}

int cam_sync_register_callback(sync_callback cb_func,
	void *userdata, int32_t sync_obj)
{
	struct sync_callback_info *sync_cb;
	struct sync_table_row *row = NULL;
	int status = 0;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0 || !cb_func)
		return -EINVAL;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	sync_cb = kzalloc(sizeof(*sync_cb), GFP_ATOMIC);
	if (!sync_cb) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -ENOMEM;
	}

	/* Trigger callback if sync object is already in SIGNALED state */
	if (((row->state == CAM_SYNC_STATE_SIGNALED_SUCCESS) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_ERROR) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_CANCEL)) &&
		(!row->remaining)) {
		if (trigger_cb_without_switch) {
			CAM_DBG(CAM_SYNC, "Invoke callback for sync object:%s[%d]",
				row->name,
				sync_obj);
			status = row->state;
			kfree(sync_cb);
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			cb_func(sync_obj, status, userdata);
		} else {
			sync_cb->callback_func = cb_func;
			sync_cb->cb_data = userdata;
			sync_cb->sync_obj = sync_obj;
			INIT_WORK(&sync_cb->cb_dispatch_work,
				cam_sync_util_cb_dispatch);
			sync_cb->status = row->state;
			CAM_DBG(CAM_SYNC, "Enqueue callback for sync object:%s[%d]",
				row->name,
				sync_cb->sync_obj);
			sync_cb->workq_scheduled_ts = ktime_get();
			queue_work(sync_dev->work_queue,
				&sync_cb->cb_dispatch_work);
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		}

		return 0;
	}

	sync_cb->callback_func = cb_func;
	sync_cb->cb_data = userdata;
	sync_cb->sync_obj = sync_obj;
	INIT_WORK(&sync_cb->cb_dispatch_work, cam_sync_util_cb_dispatch);
	list_add_tail(&sync_cb->list, &row->callback_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	return 0;
}

int cam_sync_deregister_callback(sync_callback cb_func,
	void *userdata, int32_t sync_obj)
{
	struct sync_table_row *row = NULL;
	struct sync_callback_info *sync_cb, *temp;
	bool found = false;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	CAM_DBG(CAM_SYNC, "deregistered callback for sync object:%s[%d]",
		row->name,
		sync_obj);
	list_for_each_entry_safe(sync_cb, temp, &row->callback_list, list) {
		if (sync_cb->callback_func == cb_func &&
			sync_cb->cb_data == userdata) {
			list_del_init(&sync_cb->list);
			kfree(sync_cb);
			found = true;
		}
	}

	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return found ? 0 : -ENOENT;
}

static inline int cam_sync_signal_dma_fence_util(
	struct sync_table_row *row, uint32_t status)
{
	struct cam_dma_fence_signal signal_dma_fence;

	signal_dma_fence.dma_fence_fd = row->dma_fence_info.dma_fence_fd;

	switch (status) {
	case CAM_SYNC_STATE_SIGNALED_SUCCESS:
		signal_dma_fence.status = 0;
		break;
	case CAM_SYNC_STATE_SIGNALED_ERROR:
		/* Advertise error */
		signal_dma_fence.status = -EADV;
		break;
	case CAM_SYNC_STATE_SIGNALED_CANCEL:
		signal_dma_fence.status = -ECANCELED;
		break;
	default:
		CAM_ERR(CAM_SYNC,
			"Signaling undefined status: %d for sync obj: %d",
			status, row->sync_id);
		return -EINVAL;
	}

	return cam_dma_fence_internal_signal(row->dma_fence_info.dma_fence_row_idx,
		&signal_dma_fence);
}

static void cam_sync_signal_parent_util(int32_t status,
	uint32_t event_cause, struct list_head *parents_list)
{
	int rc;
	struct sync_table_row *parent_row = NULL;
	struct sync_parent_info *parent_info, *temp_parent_info;

	/*
	 * Now iterate over all parents of this object and if they too need to
	 * be signaled dispatch cb's
	 */
	 list_for_each_entry_safe(parent_info, temp_parent_info,
		parents_list, list) {
		parent_row = sync_dev->sync_table + parent_info->sync_id;
		spin_lock_bh(&sync_dev->row_spinlocks[parent_info->sync_id]);
		parent_row->remaining--;

		rc = cam_sync_util_update_parent_state(
			parent_row,
			status);
		if (rc) {
			CAM_ERR(CAM_SYNC, "Invalid parent state %d",
				parent_row->state);
			spin_unlock_bh(
				&sync_dev->row_spinlocks[parent_info->sync_id]);
			kfree(parent_info);
			continue;
		}

		if (!parent_row->remaining)
			cam_sync_util_dispatch_signaled_cb(
				parent_info->sync_id, parent_row->state,
				event_cause);

		spin_unlock_bh(&sync_dev->row_spinlocks[parent_info->sync_id]);
		list_del_init(&parent_info->list);
		kfree(parent_info);
	}
}

static int cam_sync_signal_validate_util(
	int32_t sync_obj, int32_t status)
{
	struct sync_table_row *row = sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name, sync_obj);
		return -EINVAL;
	}

	if (row->type == CAM_SYNC_TYPE_GROUP) {
		CAM_ERR(CAM_SYNC,
			"Error: Signaling a GROUP sync object = %s[%d]",
			row->name, sync_obj);
		return -EINVAL;
	}

	if (row->state != CAM_SYNC_STATE_ACTIVE) {
		CAM_ERR(CAM_SYNC,
			"Error: Sync object already signaled sync_obj = %s[%d]",
			row->name, sync_obj);
		return -EALREADY;
	}

	if ((status != CAM_SYNC_STATE_SIGNALED_SUCCESS) &&
		(status != CAM_SYNC_STATE_SIGNALED_ERROR) &&
		(status != CAM_SYNC_STATE_SIGNALED_CANCEL)) {
		CAM_ERR(CAM_SYNC,
			"Error: signaling with undefined status = %d", status);
		return -EINVAL;
	}

	return 0;
}

int cam_sync_signal(int32_t sync_obj, uint32_t status, uint32_t event_cause)
{
	struct sync_table_row *row = NULL;
	struct list_head parents_list;
	int rc = 0;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0) {
		CAM_ERR(CAM_SYNC, "Error: Out of range sync obj (0 <= %d < %d)",
			sync_obj, CAM_SYNC_MAX_OBJS);
		return -EINVAL;
	}

	row = sync_dev->sync_table + sync_obj;
	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);

	rc = cam_sync_signal_validate_util(sync_obj, status);
	if (rc) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC,
			"Error: Failed to validate signal info for sync_obj = %s[%d] with status = %d rc = %d",
			row->name, sync_obj, status, rc);
		return rc;
	}

	if (!atomic_dec_and_test(&row->ref_cnt)) {
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return 0;
	}

	row->state = status;
	/*
	 * Signal associated dma fence first - external entities
	 * waiting on this fence can start processing
	 */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask)) {
		rc =  cam_sync_signal_dma_fence_util(row, status);
		if (rc)
			CAM_ERR(CAM_SYNC,
				"Error: Failed to signal associated dma fencefd = %d for sync_obj = %s[%d]",
				row->dma_fence_info.dma_fence_fd, row->name, sync_obj);
	}

	cam_sync_util_dispatch_signaled_cb(sync_obj, status, event_cause);

	/* copy parent list to local and release child lock */
	INIT_LIST_HEAD(&parents_list);
	list_splice_init(&row->parents_list, &parents_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	if (list_empty(&parents_list))
		return 0;

	cam_sync_signal_parent_util(status, event_cause, &parents_list);

	return 0;
}

int cam_sync_merge(int32_t *sync_obj, uint32_t num_objs, int32_t *merged_obj)
{
	int rc;
	long idx = 0;
	bool bit;
	int i = 0;

	if (!sync_obj || !merged_obj) {
		CAM_ERR(CAM_SYNC, "Invalid pointer(s)");
		return -EINVAL;
	}

	if (num_objs <= 1) {
		CAM_ERR(CAM_SYNC, "Single object merge is not allowed");
		return -EINVAL;
	}

	if (cam_common_util_remove_duplicate_arr(sync_obj, num_objs)
		!= num_objs) {
		CAM_ERR(CAM_SYNC, "The obj list has duplicate fence");
		return -EINVAL;
	}

	for (i = 0; i < num_objs; i++) {
		rc = cam_sync_check_valid(sync_obj[i]);
		if (rc) {
			CAM_ERR(CAM_SYNC, "Sync_obj[%d] %d valid check fail",
				i, sync_obj[i]);
			return rc;
		}
	}
	do {
		idx = find_first_zero_bit(sync_dev->bitmap, CAM_SYNC_MAX_OBJS);
		if (idx >= CAM_SYNC_MAX_OBJS)
			return -ENOMEM;
		bit = test_and_set_bit(idx, sync_dev->bitmap);
	} while (bit);

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	rc = cam_sync_init_group_object(sync_dev->sync_table,
		idx, sync_obj,
		num_objs);
	if (rc < 0) {
		CAM_ERR(CAM_SYNC, "Error: Unable to init row at idx = %ld",
			idx);
		clear_bit(idx, sync_dev->bitmap);
		spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
		return -EINVAL;
	}
	CAM_DBG(CAM_SYNC, "Init row at idx:%ld to merge objects", idx);
	*merged_obj = idx;
	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);

	return 0;
}

int cam_sync_get_obj_ref(int32_t sync_obj)
{
	struct sync_table_row *row = NULL;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;

	spin_lock(&sync_dev->row_spinlocks[sync_obj]);

	if (row->state != CAM_SYNC_STATE_ACTIVE) {
		spin_unlock(&sync_dev->row_spinlocks[sync_obj]);
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		return -EINVAL;
	}

	atomic_inc(&row->ref_cnt);
	spin_unlock(&sync_dev->row_spinlocks[sync_obj]);
	CAM_DBG(CAM_SYNC, "get ref for obj %d", sync_obj);

	return 0;
}

int cam_sync_put_obj_ref(int32_t sync_obj)
{
	struct sync_table_row *row = NULL;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;
	atomic_dec(&row->ref_cnt);
	CAM_DBG(CAM_SYNC, "put ref for obj %d", sync_obj);

	return 0;
}

int cam_sync_destroy(int32_t sync_obj)
{
	return cam_sync_deinit_object(sync_dev->sync_table, sync_obj, NULL);
}

int cam_sync_check_valid(int32_t sync_obj)
{
	struct sync_table_row *row = NULL;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;

	if (!test_bit(sync_obj, sync_dev->bitmap)) {
		CAM_ERR(CAM_SYNC, "Error: Released sync obj received %s[%d]",
			row->name,
			sync_obj);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		return -EINVAL;
	}
	return 0;
}

int cam_sync_wait(int32_t sync_obj, uint64_t timeout_ms)
{
	unsigned long timeleft;
	int rc = -EINVAL;
	struct sync_table_row *row = NULL;

	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	row = sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		return -EINVAL;
	}

	timeleft = cam_common_wait_for_completion_timeout(&row->signaled,
		msecs_to_jiffies(timeout_ms));

	if (!timeleft) {
		CAM_ERR(CAM_SYNC,
			"Error: timed out for sync obj = %s[%d]", row->name, sync_obj);
		rc = -ETIMEDOUT;
	} else {
		switch (row->state) {
		case CAM_SYNC_STATE_INVALID:
		case CAM_SYNC_STATE_ACTIVE:
		case CAM_SYNC_STATE_SIGNALED_ERROR:
		case CAM_SYNC_STATE_SIGNALED_CANCEL:
			CAM_ERR(CAM_SYNC,
				"Error: Wait on invalid state = %d, obj = %d, name = %s",
				row->state, sync_obj, row->name);
			rc = -EINVAL;
			break;
		case CAM_SYNC_STATE_SIGNALED_SUCCESS:
			rc = 0;
			break;
		default:
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

static int cam_sync_handle_create(struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_info sync_create;
	int result;

	if (k_ioctl->size != sizeof(struct cam_sync_info))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_create,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;
	sync_create.name[SYNC_DEBUG_NAME_LEN] = '\0';

	result = cam_sync_create(&sync_create.sync_obj,
		sync_create.name);

	if (!result)
		if (copy_to_user(
			u64_to_user_ptr(k_ioctl->ioctl_ptr),
			&sync_create,
			k_ioctl->size))
			return -EFAULT;

	return result;
}

static int cam_sync_handle_signal(struct cam_private_ioctl_arg *k_ioctl)
{
	int rc = 0;
	struct cam_sync_signal sync_signal;

	if (k_ioctl->size != sizeof(struct cam_sync_signal))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_signal,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	/* need to get ref for UMD signaled fences */
	rc = cam_sync_get_obj_ref(sync_signal.sync_obj);
	if (rc) {
		CAM_DBG(CAM_SYNC,
			"Error: cannot signal an uninitialized sync obj = %d",
			sync_signal.sync_obj);
		return rc;
	}

	return cam_sync_signal(sync_signal.sync_obj,
		sync_signal.sync_state,
		CAM_SYNC_COMMON_SYNC_SIGNAL_EVENT);
}

static int cam_sync_handle_merge(struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_merge sync_merge;
	uint32_t *sync_objs;
	uint32_t num_objs;
	uint32_t size;
	int result;

	if (k_ioctl->size != sizeof(struct cam_sync_merge))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_merge,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	if (sync_merge.num_objs >= CAM_SYNC_MAX_OBJS)
		return -EINVAL;

	size = sizeof(uint32_t) * sync_merge.num_objs;
	sync_objs = kzalloc(size, GFP_ATOMIC);

	if (!sync_objs)
		return -ENOMEM;

	if (copy_from_user(sync_objs,
		u64_to_user_ptr(sync_merge.sync_objs),
		sizeof(uint32_t) * sync_merge.num_objs)) {
		kfree(sync_objs);
		return -EFAULT;
	}

	num_objs = sync_merge.num_objs;

	result = cam_sync_merge(sync_objs,
		num_objs,
		&sync_merge.merged);

	if (!result)
		if (copy_to_user(
			u64_to_user_ptr(k_ioctl->ioctl_ptr),
			&sync_merge,
			k_ioctl->size)) {
			kfree(sync_objs);
			return -EFAULT;
	}

	kfree(sync_objs);

	return result;
}

static int cam_sync_handle_wait(struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_wait sync_wait;

	if (k_ioctl->size != sizeof(struct cam_sync_wait))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_wait,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	k_ioctl->result = cam_sync_wait(sync_wait.sync_obj,
		sync_wait.timeout_ms);

	return 0;
}

static int cam_sync_handle_destroy(struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_info sync_create;

	if (k_ioctl->size != sizeof(struct cam_sync_info))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&sync_create,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	return cam_sync_destroy(sync_create.sync_obj);
}

static int cam_sync_handle_register_user_payload(
	struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_userpayload_info userpayload_info;
	struct sync_user_payload *user_payload_kernel;
	struct sync_user_payload *user_payload_iter;
	struct sync_user_payload *temp_upayload_kernel;
	uint32_t sync_obj;
	struct sync_table_row *row = NULL;

	if (k_ioctl->size != sizeof(struct cam_sync_userpayload_info))
		return -EINVAL;

	if (!k_ioctl->ioctl_ptr)
		return -EINVAL;

	if (copy_from_user(&userpayload_info,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	sync_obj = userpayload_info.sync_obj;
	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	user_payload_kernel = kzalloc(sizeof(*user_payload_kernel), GFP_KERNEL);
	if (!user_payload_kernel)
		return -ENOMEM;

	memcpy(user_payload_kernel->payload_data,
		userpayload_info.payload,
		CAM_SYNC_PAYLOAD_WORDS * sizeof(__u64));

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row =  sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		kfree(user_payload_kernel);
		return -EINVAL;
	}

	if ((row->state == CAM_SYNC_STATE_SIGNALED_SUCCESS) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_ERROR) ||
		(row->state == CAM_SYNC_STATE_SIGNALED_CANCEL)) {

		cam_sync_util_send_v4l2_event(CAM_SYNC_V4L_EVENT_ID_CB_TRIG,
			sync_obj,
			row->state,
			user_payload_kernel->payload_data,
			CAM_SYNC_USER_PAYLOAD_SIZE * sizeof(__u64),
			CAM_SYNC_COMMON_REG_PAYLOAD_EVENT);

		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		kfree(user_payload_kernel);
		return 0;
	}

	list_for_each_entry_safe(user_payload_iter,
		temp_upayload_kernel,
		&row->user_payload_list,
		list) {
		if (user_payload_iter->payload_data[0] ==
				user_payload_kernel->payload_data[0] &&
			user_payload_iter->payload_data[1] ==
				user_payload_kernel->payload_data[1]) {

			spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
			kfree(user_payload_kernel);
			return -EALREADY;
		}
	}

	list_add_tail(&user_payload_kernel->list, &row->user_payload_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return 0;
}

static int cam_sync_handle_deregister_user_payload(
	struct cam_private_ioctl_arg *k_ioctl)
{
	struct cam_sync_userpayload_info userpayload_info;
	struct sync_user_payload *user_payload_kernel, *temp;
	uint32_t sync_obj;
	struct sync_table_row *row = NULL;

	if (k_ioctl->size != sizeof(struct cam_sync_userpayload_info)) {
		CAM_ERR(CAM_SYNC, "Incorrect ioctl size");
		return -EINVAL;
	}

	if (!k_ioctl->ioctl_ptr) {
		CAM_ERR(CAM_SYNC, "Invalid embedded ioctl ptr");
		return -EINVAL;
	}

	if (copy_from_user(&userpayload_info,
		u64_to_user_ptr(k_ioctl->ioctl_ptr),
		k_ioctl->size))
		return -EFAULT;

	sync_obj = userpayload_info.sync_obj;
	if (sync_obj >= CAM_SYNC_MAX_OBJS || sync_obj <= 0)
		return -EINVAL;

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row =  sync_dev->sync_table + sync_obj;

	if (row->state == CAM_SYNC_STATE_INVALID) {
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj = %s[%d]",
			row->name,
			sync_obj);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
		return -EINVAL;
	}

	list_for_each_entry_safe(user_payload_kernel, temp,
				&row->user_payload_list, list) {
		if (user_payload_kernel->payload_data[0] ==
				userpayload_info.payload[0] &&
				user_payload_kernel->payload_data[1] ==
				userpayload_info.payload[1]) {
			list_del_init(&user_payload_kernel->list);
			kfree(user_payload_kernel);
		}
	}

	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return 0;
}

static int cam_sync_dma_fence_cb(
	int32_t sync_obj,
	struct cam_dma_fence_signal_sync_obj *signal_sync_obj)
{
	int32_t rc = 0;
	int32_t status = CAM_SYNC_STATE_SIGNALED_SUCCESS;
	struct sync_table_row *row = NULL;
	struct list_head parents_list;

	if (!signal_sync_obj) {
		CAM_ERR(CAM_SYNC, "Invalid signal info args");
		return -EINVAL;
	}

	/* Validate sync object range */
	if (!(sync_obj > 0 && sync_obj < CAM_SYNC_MAX_OBJS)) {
		CAM_ERR(CAM_SYNC, "Invalid sync obj: %d", sync_obj);
		return -EINVAL;
	}

	spin_lock_bh(&sync_dev->row_spinlocks[sync_obj]);
	row = sync_dev->sync_table + sync_obj;

	/* Validate if sync obj has a dma fence association */
	if (!test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask)) {
		CAM_ERR(CAM_SYNC,
			"sync obj = %d[%s] has no associated dma fence ext_fence_mask = 0x%x",
			sync_obj, row->name, row->ext_fence_mask);
		rc = -EINVAL;
		goto end;
	}

	/* Validate if we are signaling the right sync obj based on dma fence fd */
	if (row->dma_fence_info.dma_fence_fd != signal_sync_obj->fd) {
		CAM_ERR(CAM_SYNC,
			"sync obj: %d[%s] is associated with a different fd: %d, signaling for fd: %d",
			sync_obj, row->name, row->dma_fence_info.dma_fence_fd, signal_sync_obj->fd);
		rc = -EINVAL;
		goto end;
	}

	/* Check for error status */
	if (signal_sync_obj->status < 0) {
		if (signal_sync_obj->status == -ECANCELED)
			status = CAM_SYNC_STATE_SIGNALED_CANCEL;
		else
			status = CAM_SYNC_STATE_SIGNALED_ERROR;
	}

	rc = cam_sync_signal_validate_util(sync_obj, status);
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"Error: Failed to validate signal info for sync_obj = %d[%s] with status = %d rc = %d",
			sync_obj, row->name, status, rc);
		goto end;
	}

	if (!atomic_dec_and_test(&row->ref_cnt))
		goto end;

	row->state = status;

	cam_sync_util_dispatch_signaled_cb(sync_obj, status, 0);

	INIT_LIST_HEAD(&parents_list);
	list_splice_init(&row->parents_list, &parents_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);

	if (list_empty(&parents_list))
		return 0;

	cam_sync_signal_parent_util(status, 0x0, &parents_list);
	return 0;

end:
	spin_unlock_bh(&sync_dev->row_spinlocks[sync_obj]);
	return rc;
}

static int cam_generic_fence_alloc_validate_input_info_util(
	struct cam_generic_fence_cmd_args    *fence_cmd_args,
	struct cam_generic_fence_input_info **fence_input_info)
{
	int rc = 0;
	struct cam_generic_fence_input_info *fence_input = NULL;
	uint32_t num_fences;
	size_t expected_size;

	*fence_input_info = NULL;

	if (fence_cmd_args->input_data_size <
		sizeof(struct cam_generic_fence_input_info)) {
		CAM_ERR(CAM_SYNC, "Size is invalid expected: 0x%llx actual: 0x%llx",
			sizeof(struct cam_generic_fence_input_info),
			fence_cmd_args->input_data_size);
		return -EINVAL;
	}

	fence_input = memdup_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_cmd_args->input_data_size);
	if (IS_ERR_OR_NULL(fence_input)) {
		CAM_ERR(CAM_SYNC, "memdup failed for hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		return -ENOMEM;
	}

	/* Validate num fences */
	num_fences = fence_input->num_fences_requested;
	if ((num_fences == 0) || (num_fences > CAM_GENERIC_FENCE_BATCH_MAX)) {
		CAM_ERR(CAM_SYNC, "Invalid number of fences: %u for batching",
			num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	/* Validate sizes */
	expected_size = sizeof(struct cam_generic_fence_input_info) +
		((num_fences - 1) * sizeof(struct cam_generic_fence_config));
	if ((uint32_t)expected_size != fence_cmd_args->input_data_size) {
		CAM_ERR(CAM_SYNC, "Invalid input size expected: 0x%x actual: 0x%x for fences: %u",
			expected_size, fence_cmd_args->input_data_size, num_fences);
		rc = -EINVAL;
		goto free_mem;
	}

	*fence_input_info = fence_input;
	return rc;

free_mem:
	kfree(fence_input);
	return rc;
}

static void cam_generic_fence_free_input_info_util(
	struct cam_generic_fence_input_info **fence_input_info)
{
	struct cam_generic_fence_input_info *fence_input = *fence_input_info;

	kfree(fence_input);
	*fence_input_info = NULL;
}

static int cam_generic_fence_handle_dma_create(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = 0, i, dma_fence_row_idx;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_DMA_FENCE,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		rc = cam_dma_fence_create_fd(&fence_cfg->dma_fence_fd,
			&dma_fence_row_idx, fence_cfg->name);
		if (rc) {
			CAM_ERR(CAM_DMA_FENCE,
				"Failed to create dma fence at index: %d rc: %d num fences [requested: %u processed: %u]",
				i, rc, fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
			fence_cfg->reason_code = rc;
			goto out_copy;
		}

		CAM_DBG(CAM_DMA_FENCE,
			"Created dma_fence @ i: %d fence fd: %d[%s] num fences [requested: %u processed: %u] ",
			i, fence_cfg->dma_fence_fd, fence_cfg->name,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		CAM_ERR(CAM_DMA_FENCE, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		rc = -EFAULT;
	}

	cam_generic_fence_free_input_info_util(&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_dma_release(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = 0, i;
	bool failed = false;
	struct cam_dma_fence_release_params release_params;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_DMA_FENCE,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		release_params.use_row_idx = false;
		release_params.u.dma_fence_fd = fence_cfg->dma_fence_fd;
		rc = cam_dma_fence_release(&release_params);
		if (rc) {
			CAM_ERR(CAM_DMA_FENCE,
				"Failed to destroy dma fence at index: %d fd: %d rc: %d num fences [requested: %u processed: %u]",
				i, fence_cfg->dma_fence_fd, rc,
				fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
			fence_cfg->reason_code = rc;
			/* Continue to release other fences, but mark the call as failed */
			failed = true;
			continue;
		}

		CAM_DBG(CAM_DMA_FENCE,
			"Released dma_fence @ i: %d fd: %d num fences [requested: %u processed: %u]",
			i, fence_cfg->dma_fence_fd,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

	if (failed)
		rc = -ENOMSG;

	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		CAM_ERR(CAM_DMA_FENCE, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
		rc = -EFAULT;
	}

	cam_generic_fence_free_input_info_util(&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_dma_import(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int32_t rc = 0, i, dma_fence_row_idx;
	struct dma_fence *fence = NULL;
	struct cam_dma_fence_create_sync_obj_payload dma_sync_create;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_DMA_FENCE,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		/* Check if fd is for a valid dma fence */
		fence = cam_dma_fence_get_fence_from_fd(fence_cfg->dma_fence_fd,
			&dma_fence_row_idx);
		if (IS_ERR_OR_NULL(fence)) {
			CAM_ERR(CAM_DMA_FENCE,
				"Invalid dma fence for fd: %d", fence_cfg->dma_fence_fd);
			fence_cfg->reason_code = -EINVAL;
			goto out_copy;
		}

		dma_sync_create.dma_fence_row_idx = dma_fence_row_idx;
		dma_sync_create.fd = fence_cfg->dma_fence_fd;
		dma_sync_create.sync_created_with_dma = false;

		/* Create new sync object and associate dma fence */
		rc = cam_sync_create_util(&fence_cfg->sync_obj, fence_cfg->name,
			&dma_sync_create);
		if (rc) {
			fence_cfg->reason_code = rc;

			/* put on the import refcnt */
			cam_dma_fence_get_put_ref(false, dma_fence_row_idx);
			goto out_copy;
		}

		/* Register a cb for dma fence */
		rc = cam_dma_fence_register_cb(&fence_cfg->sync_obj,
			&dma_fence_row_idx, cam_sync_dma_fence_cb);
		if (rc) {
			CAM_ERR(CAM_DMA_FENCE,
				"Failed to register cb for dma fence fd: %d sync_obj: %d rc: %d",
				fence_cfg->dma_fence_fd, fence_cfg->sync_obj, rc);
			cam_sync_deinit_object(sync_dev->sync_table, fence_cfg->sync_obj, NULL);
			fence_cfg->reason_code = rc;
			goto out_copy;
		}

		CAM_DBG(CAM_DMA_FENCE,
			"dma fence fd = %d imported for sync_obj = %d[%s] num fences [requested: %u processed: %u]",
			fence_cfg->dma_fence_fd, fence_cfg->sync_obj, fence_cfg->name,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_DMA_FENCE, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
	}

	cam_generic_fence_free_input_info_util(&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_dma_signal(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	struct cam_dma_fence_signal signal_dma_fence;

	if (fence_cmd_args->input_data_size < sizeof(struct cam_dma_fence_signal)) {
		CAM_ERR(CAM_DMA_FENCE, "Size is invalid expected: 0x%llx actual: 0x%llx",
			sizeof(struct cam_dma_fence_signal),
			fence_cmd_args->input_data_size);
		return -EINVAL;
	}

	if (copy_from_user(&signal_dma_fence, (void __user *)fence_cmd_args->input_handle,
		fence_cmd_args->input_data_size))
		return -EFAULT;

	return cam_dma_fence_signal_fd(&signal_dma_fence);
}

static int cam_generic_fence_process_dma_fence_cmd(
	uint32_t id,
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = -EINVAL;

	switch (id) {
	case CAM_GENERIC_FENCE_CREATE:
		rc = cam_generic_fence_handle_dma_create(fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_RELEASE:
		rc = cam_generic_fence_handle_dma_release(fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_IMPORT:
		rc = cam_generic_fence_handle_dma_import(fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_SIGNAL:
		rc = cam_generic_fence_handle_dma_signal(fence_cmd_args);
		break;
	default:
		CAM_ERR(CAM_DMA_FENCE, "IOCTL cmd: %u not supported for dma fence", id);
		break;
	}

	return rc;
}

static int cam_generic_fence_handle_sync_create(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = 0, i, dma_fence_row_idx;
	bool dma_fence_created;
	unsigned long fence_sel_mask;
	struct cam_dma_fence_release_params release_params;
	struct cam_dma_fence_create_sync_obj_payload dma_sync_create;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNC,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		fence_cfg->reason_code = 0;

		/* Reset flag */
		dma_fence_created = false;

		fence_sel_mask = fence_cfg->fence_sel_mask;
		if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &fence_sel_mask)) {
			rc = cam_dma_fence_create_fd(&fence_cfg->dma_fence_fd,
				&dma_fence_row_idx, fence_cfg->name);
			if (rc) {
				CAM_ERR(CAM_SYNC,
				"Failed to create dma fence at index: %d rc: %d num_fences: %u",
				i, rc, fence_input_info->num_fences_requested);
				fence_cfg->reason_code = rc;
				goto out_copy;
			}

			dma_sync_create.dma_fence_row_idx = dma_fence_row_idx;
			dma_sync_create.fd = fence_cfg->dma_fence_fd;
			dma_sync_create.sync_created_with_dma = true;
			dma_fence_created = true;
		}

		rc = cam_sync_create_util(&fence_cfg->sync_obj, fence_cfg->name,
			(dma_fence_created ? &dma_sync_create : NULL));
		if (rc) {
			fence_cfg->reason_code = rc;
			if (dma_fence_created) {
				release_params.use_row_idx = true;
				release_params.u.dma_row_idx = dma_fence_row_idx;

				cam_dma_fence_release(&release_params);
			}

			CAM_ERR(CAM_SYNC,
				"Failed to create sync obj at index: %d rc: %d num_fences: %u",
				i, rc, fence_input_info->num_fences_requested);
			goto out_copy;
		}

		/* Register dma fence cb */
		if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &fence_sel_mask)) {
			rc = cam_dma_fence_register_cb(&fence_cfg->sync_obj,
				&dma_fence_row_idx, cam_sync_dma_fence_cb);
			if (rc) {
				CAM_ERR(CAM_SYNC,
					"Failed to register cb for dma fence fd: %d sync_obj: %d rc: %d",
					fence_cfg->dma_fence_fd, fence_cfg->sync_obj, rc);

				/* Destroy sync obj */
				cam_sync_deinit_object(
					sync_dev->sync_table, fence_cfg->sync_obj, NULL);

				/* Release dma fence */
				release_params.use_row_idx = true;
				release_params.u.dma_row_idx = dma_fence_row_idx;
				cam_dma_fence_release(&release_params);

				fence_cfg->reason_code = rc;
				goto out_copy;
			}
		}

		CAM_DBG(CAM_SYNC,
			"Created sync_obj = %d[%s] with fence_sel_mask: 0x%x dma_fence_fd: %d num fences [requested: %u processed: %u]",
			fence_cfg->sync_obj, fence_cfg->name,
			fence_cfg->fence_sel_mask, fence_cfg->dma_fence_fd,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

out_copy:
	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNC, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
	}

	cam_generic_fence_free_input_info_util(&fence_input_info);
	return rc;
}

static int cam_generic_fence_handle_sync_release(
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	bool failed = false;
	int rc = 0, i;
	unsigned long fence_sel_mask;
	struct cam_sync_check_for_dma_release check_for_dma_release;
	struct cam_dma_fence_release_params release_params;
	struct cam_generic_fence_input_info *fence_input_info = NULL;
	struct cam_generic_fence_config *fence_cfg = NULL;

	rc = cam_generic_fence_alloc_validate_input_info_util(fence_cmd_args, &fence_input_info);
	if (rc || !fence_input_info) {
		CAM_ERR(CAM_SYNC,
			"Fence input info validation failed rc: %d fence_input_info: %pK",
			rc, fence_input_info);
		return -EINVAL;
	}

	for (i = 0; i < fence_input_info->num_fences_requested; i++) {
		fence_cfg = &fence_input_info->fence_cfg[i];
		fence_input_info->num_fences_processed++;
		/* Reset fields */
		fence_cfg->reason_code = 0;
		check_for_dma_release.sync_created_with_dma = false;
		check_for_dma_release.dma_fence_fd = fence_cfg->dma_fence_fd;

		rc = cam_sync_deinit_object(sync_dev->sync_table, fence_cfg->sync_obj,
			&check_for_dma_release);
		if (rc) {
			fence_cfg->reason_code = rc;
			failed = true;
			CAM_ERR(CAM_SYNC,
				"Failed to release sync obj at index: %d rc: %d num_fences [requested: %u processed: %u]",
				i, rc, fence_input_info->num_fences_requested,
				fence_input_info->num_fences_processed);
		}

		fence_sel_mask = fence_cfg->fence_sel_mask;
		if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &fence_sel_mask)) {
			if (!check_for_dma_release.sync_created_with_dma) {
				CAM_ERR(CAM_SYNC,
					"Failed to release dma fence fd: %d with sync_obj: %d, not created together",
					fence_cfg->dma_fence_fd, fence_cfg->sync_obj);
				failed = true;
				fence_cfg->reason_code = -EPERM;
				continue;
			}

			release_params.use_row_idx = true;
			release_params.u.dma_row_idx = check_for_dma_release.dma_fence_row_idx;
			rc = cam_dma_fence_release(&release_params);
			if (rc) {
				CAM_ERR(CAM_SYNC,
					"Failed to destroy dma fence at index: %d rc: %d num fences [requested: %u processed: %u]",
					i, rc, fence_input_info->num_fences_requested,
					fence_input_info->num_fences_processed);
				fence_cfg->reason_code = rc;
				failed = true;
				continue;
			}
		}

		CAM_DBG(CAM_SYNC,
			"Released sync_obj = %d[%s] with fence_sel_mask: 0x%x dma_fence_fd: %d  num fences [requested: %u processed: %u]",
			fence_cfg->sync_obj, fence_cfg->name,
			fence_cfg->fence_sel_mask, fence_cfg->dma_fence_fd,
			fence_input_info->num_fences_requested,
			fence_input_info->num_fences_processed);
	}

	if (failed)
		rc = -ENOMSG;

	if (copy_to_user(u64_to_user_ptr(fence_cmd_args->input_handle),
		fence_input_info, fence_cmd_args->input_data_size)) {
		rc = -EFAULT;
		CAM_ERR(CAM_SYNC, "copy to user failed hdl: %d size: 0x%x",
			fence_cmd_args->input_handle, fence_cmd_args->input_data_size);
	}

	cam_generic_fence_free_input_info_util(&fence_input_info);
	return rc;
}

static int cam_generic_fence_process_sync_obj_cmd(
	uint32_t id,
	struct cam_generic_fence_cmd_args *fence_cmd_args)
{
	int rc = -EINVAL;

	switch (id) {
	case CAM_GENERIC_FENCE_CREATE:
		rc = cam_generic_fence_handle_sync_create(fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_RELEASE:
		rc = cam_generic_fence_handle_sync_release(fence_cmd_args);
		break;
	default:
		CAM_ERR(CAM_SYNC, "IOCTL cmd: %u not supported for sync object", id);
		break;
	}

	return rc;
}

static int cam_generic_fence_parser(
	struct cam_private_ioctl_arg *k_ioctl)
{
	int rc;
	struct cam_generic_fence_cmd_args fence_cmd_args;

	if (!k_ioctl->ioctl_ptr) {
		CAM_ERR(CAM_SYNC, "Invalid args input ptr: %p",
			k_ioctl->ioctl_ptr);
		return -EINVAL;
	}

	if (k_ioctl->size != sizeof(struct cam_generic_fence_cmd_args)) {
		CAM_ERR(CAM_SYNC, "Size mismatch expected: 0x%llx actual: 0x%llx",
			sizeof(struct cam_generic_fence_cmd_args), k_ioctl->size);
		return -EINVAL;
	}

	if (copy_from_user(&fence_cmd_args, u64_to_user_ptr(k_ioctl->ioctl_ptr),
		sizeof(fence_cmd_args))) {
		CAM_ERR(CAM_SYNC, "copy from user failed for input ptr: %pK",
			k_ioctl->ioctl_ptr);
		return -EFAULT;
	}

	if (fence_cmd_args.input_handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SYNC, "Invalid handle type: %u",
			fence_cmd_args.input_handle_type);
		return -EINVAL;
	}

	switch (fence_cmd_args.fence_type) {
	case CAM_GENERIC_FENCE_TYPE_SYNC_OBJ:
		rc = cam_generic_fence_process_sync_obj_cmd(k_ioctl->id, &fence_cmd_args);
		break;
	case CAM_GENERIC_FENCE_TYPE_DMA_FENCE:
		rc = cam_generic_fence_process_dma_fence_cmd(k_ioctl->id, &fence_cmd_args);
		break;
	default:
		rc = -EINVAL;
		CAM_ERR(CAM_SYNC, "fence type: 0x%x handling not supported",
			fence_cmd_args.fence_type);
		break;
	}

	return rc;
}

static long cam_sync_dev_ioctl(struct file *filep, void *fh,
		bool valid_prio, unsigned int cmd, void *arg)
{
	int32_t rc;
	struct sync_device *sync_dev = video_drvdata(filep);
	struct cam_private_ioctl_arg k_ioctl;

	if (!sync_dev) {
		CAM_ERR(CAM_SYNC, "sync_dev NULL");
		return -EINVAL;
	}

	if (!arg)
		return -EINVAL;

	if (cmd != CAM_PRIVATE_IOCTL_CMD)
		return -ENOIOCTLCMD;

	k_ioctl = *(struct cam_private_ioctl_arg *)arg;

	switch (k_ioctl.id) {
	case CAM_SYNC_CREATE:
		rc = cam_sync_handle_create(&k_ioctl);
		break;
	case CAM_SYNC_DESTROY:
		rc = cam_sync_handle_destroy(&k_ioctl);
		break;
	case CAM_SYNC_REGISTER_PAYLOAD:
		rc = cam_sync_handle_register_user_payload(
			&k_ioctl);
		break;
	case CAM_SYNC_DEREGISTER_PAYLOAD:
		rc = cam_sync_handle_deregister_user_payload(
			&k_ioctl);
		break;
	case CAM_SYNC_SIGNAL:
		rc = cam_sync_handle_signal(&k_ioctl);
		break;
	case CAM_SYNC_MERGE:
		rc = cam_sync_handle_merge(&k_ioctl);
		break;
	case CAM_SYNC_WAIT:
		rc = cam_sync_handle_wait(&k_ioctl);
		((struct cam_private_ioctl_arg *)arg)->result =
			k_ioctl.result;
		break;
	case CAM_GENERIC_FENCE_CREATE:
	case CAM_GENERIC_FENCE_RELEASE:
	case CAM_GENERIC_FENCE_IMPORT:
	case CAM_GENERIC_FENCE_SIGNAL:
		rc = cam_generic_fence_parser(&k_ioctl);
		break;
	default:
		rc = -ENOIOCTLCMD;
	}

	return rc;
}

static unsigned int cam_sync_poll(struct file *f,
	struct poll_table_struct *pll_table)
{
	int rc = 0;
	struct v4l2_fh *eventq = f->private_data;

	if (!eventq)
		return -EINVAL;

	poll_wait(f, &eventq->wait, pll_table);

	if (v4l2_event_pending(eventq))
		rc = POLLPRI;

	return rc;
}

static int cam_sync_open(struct file *filep)
{
	int rc;
	struct sync_device *sync_dev = video_drvdata(filep);

	if (!sync_dev) {
		CAM_ERR(CAM_SYNC, "Sync device NULL");
		return -ENODEV;
	}

	mutex_lock(&sync_dev->table_lock);
	if (sync_dev->open_cnt >= 1) {
		mutex_unlock(&sync_dev->table_lock);
		return -EALREADY;
	}

	rc = v4l2_fh_open(filep);
	if (!rc) {
		sync_dev->open_cnt++;
		cam_dma_fence_open();
		spin_lock_bh(&sync_dev->cam_sync_eventq_lock);
		sync_dev->cam_sync_eventq = filep->private_data;
		spin_unlock_bh(&sync_dev->cam_sync_eventq_lock);
	} else {
		CAM_ERR(CAM_SYNC, "v4l2_fh_open failed : %d", rc);
	}
	mutex_unlock(&sync_dev->table_lock);

	return rc;
}

static int cam_sync_close(struct file *filep)
{
	int rc = 0;
	int i;
	struct sync_device *sync_dev = video_drvdata(filep);

	if (!sync_dev) {
		CAM_ERR(CAM_SYNC, "Sync device NULL");
		rc = -ENODEV;
		return rc;
	}
	mutex_lock(&sync_dev->table_lock);
	sync_dev->open_cnt--;
	if (!sync_dev->open_cnt) {
		for (i = 1; i < CAM_SYNC_MAX_OBJS; i++) {
			struct sync_table_row *row =
			sync_dev->sync_table + i;

			/*
			 * Signal all ACTIVE objects as ERR, but we don't
			 * care about the return status here apart from logging
			 * it.
			 */
			if (row->state == CAM_SYNC_STATE_ACTIVE) {
				rc = cam_sync_signal(i,
					CAM_SYNC_STATE_SIGNALED_ERROR,
					CAM_SYNC_COMMON_RELEASE_EVENT);
				if (rc < 0)
					CAM_ERR(CAM_SYNC,
					  "Cleanup signal fail idx:%d", i);
			}
		}

		/*
		 * Flush the work queue to wait for pending signal callbacks to
		 * finish
		 */
		flush_workqueue(sync_dev->work_queue);

		/*
		 * Now that all callbacks worker threads have finished,
		 * destroy the sync objects
		 */
		for (i = 1; i < CAM_SYNC_MAX_OBJS; i++) {
			struct sync_table_row *row =
			sync_dev->sync_table + i;

			if (row->state != CAM_SYNC_STATE_INVALID) {
				rc = cam_sync_destroy(i);
				if (rc < 0)
					CAM_ERR(CAM_SYNC,
					  "Cleanup destroy fail:idx:%d\n", i);
			}
		}
	}

	/* Clean dma fence table */
	cam_dma_fence_close();
	mutex_unlock(&sync_dev->table_lock);

	spin_lock_bh(&sync_dev->cam_sync_eventq_lock);
	sync_dev->cam_sync_eventq = NULL;
	spin_unlock_bh(&sync_dev->cam_sync_eventq_lock);
	v4l2_fh_release(filep);

	return rc;
}

static void cam_sync_event_queue_notify_error(const struct v4l2_event *old,
	struct v4l2_event *new)
{
	if (sync_dev->version == CAM_SYNC_V4L_EVENT_V2) {
		struct cam_sync_ev_header_v2 *ev_header;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V2((*old));
		CAM_ERR(CAM_CRM,
			"Failed to notify event id %d fence %d statue %d reason %u %u %u %u",
			old->id, ev_header->sync_obj, ev_header->status,
			ev_header->evt_param[0], ev_header->evt_param[1],
			ev_header->evt_param[2], ev_header->evt_param[3]);

	} else {
		struct cam_sync_ev_header *ev_header;

		ev_header = CAM_SYNC_GET_HEADER_PTR((*old));
		CAM_ERR(CAM_CRM,
			"Failed to notify event id %d fence %d statue %d",
			old->id, ev_header->sync_obj, ev_header->status);
	}
}

static struct v4l2_subscribed_event_ops cam_sync_v4l2_ops = {
	.merge = cam_sync_event_queue_notify_error,
};

int cam_sync_subscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub)
{
	if (!((sub->type == CAM_SYNC_V4L_EVENT) ||
	(sub->type == CAM_SYNC_V4L_EVENT_V2))) {
		CAM_ERR(CAM_SYNC, "Non supported event type 0x%x", sub->type);
		return -EINVAL;
	}

	sync_dev->version = sub->type;
	CAM_DBG(CAM_SYNC, "Sync event verion type 0x%x", sync_dev->version);
	return v4l2_event_subscribe(fh, sub, CAM_SYNC_MAX_V4L2_EVENTS,
		&cam_sync_v4l2_ops);
}

int cam_sync_unsubscribe_event(struct v4l2_fh *fh,
		const struct v4l2_event_subscription *sub)
{
	if (!((sub->type == CAM_SYNC_V4L_EVENT) ||
	(sub->type == CAM_SYNC_V4L_EVENT_V2))) {
		CAM_ERR(CAM_SYNC, "Non supported event type 0x%x", sub->type);
		return -EINVAL;
	}

	return v4l2_event_unsubscribe(fh, sub);
}

static const struct v4l2_ioctl_ops g_cam_sync_ioctl_ops = {
	.vidioc_subscribe_event = cam_sync_subscribe_event,
	.vidioc_unsubscribe_event = cam_sync_unsubscribe_event,
	.vidioc_default = cam_sync_dev_ioctl,
};

static struct v4l2_file_operations cam_sync_v4l2_fops = {
	.owner = THIS_MODULE,
	.open  = cam_sync_open,
	.release = cam_sync_close,
	.poll = cam_sync_poll,
	.unlocked_ioctl   = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
};

#if IS_REACHABLE(CONFIG_MEDIA_CONTROLLER)
static int cam_sync_media_controller_init(struct sync_device *sync_dev,
	struct platform_device *pdev)
{
	int rc;

	sync_dev->v4l2_dev.mdev = kzalloc(sizeof(struct media_device),
		GFP_KERNEL);
	if (!sync_dev->v4l2_dev.mdev)
		return -ENOMEM;

	media_device_init(sync_dev->v4l2_dev.mdev);
	strlcpy(sync_dev->v4l2_dev.mdev->model, CAM_SYNC_DEVICE_NAME,
			sizeof(sync_dev->v4l2_dev.mdev->model));
	sync_dev->v4l2_dev.mdev->dev = &(pdev->dev);

	rc = media_device_register(sync_dev->v4l2_dev.mdev);
	if (rc < 0)
		goto register_fail;

	rc = media_entity_pads_init(&sync_dev->vdev->entity, 0, NULL);
	if (rc < 0)
		goto entity_fail;

	return 0;

entity_fail:
	media_device_unregister(sync_dev->v4l2_dev.mdev);
register_fail:
	media_device_cleanup(sync_dev->v4l2_dev.mdev);
	return rc;
}

static void cam_sync_media_controller_cleanup(struct sync_device *sync_dev)
{
	media_entity_cleanup(&sync_dev->vdev->entity);
	media_device_unregister(sync_dev->v4l2_dev.mdev);
	media_device_cleanup(sync_dev->v4l2_dev.mdev);
	kfree(sync_dev->v4l2_dev.mdev);
}

static void cam_sync_init_entity(struct sync_device *sync_dev)
{
	sync_dev->vdev->entity.function = CAM_SYNC_DEVICE_TYPE;
	sync_dev->vdev->entity.name =
				video_device_node_name(sync_dev->vdev);
}
#else
static int cam_sync_media_controller_init(struct sync_device *sync_dev,
	struct platform_device *pdev)
{
	return 0;
}

static void cam_sync_media_controller_cleanup(struct sync_device *sync_dev)
{
}

static void cam_sync_init_entity(struct sync_device *sync_dev)
{
}
#endif

static int cam_sync_create_debugfs(void)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	if (!cam_debugfs_available())
		return 0;

	rc = cam_debugfs_create_subdir("sync", &dbgfileptr);
	if (rc) {
		CAM_ERR(CAM_SYNC,"DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}
	/* Store parent inode for cleanup in caller */
	sync_dev->dentry = dbgfileptr;

	debugfs_create_bool("trigger_cb_without_switch", 0644,
		sync_dev->dentry, &trigger_cb_without_switch);

end:
	return rc;
}

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
int cam_synx_sync_signal(int32_t sync_obj, uint32_t synx_status)
{
	int rc = 0;
	uint32_t sync_status = synx_status;

	switch (synx_status) {
	case SYNX_STATE_ACTIVE:
		sync_status = CAM_SYNC_STATE_ACTIVE;
		break;
	case SYNX_STATE_SIGNALED_SUCCESS:
		sync_status = CAM_SYNC_STATE_SIGNALED_SUCCESS;
		break;
	case SYNX_STATE_SIGNALED_ERROR:
		sync_status = CAM_SYNC_STATE_SIGNALED_ERROR;
		break;
	case 4: /* SYNX_STATE_SIGNALED_CANCEL: */
		sync_status = CAM_SYNC_STATE_SIGNALED_CANCEL;
		break;
	default:
		CAM_ERR(CAM_SYNC, "Invalid synx status %d for obj %d",
			synx_status, sync_obj);
		sync_status = CAM_SYNC_STATE_SIGNALED_ERROR;
		break;
	}

	rc = cam_sync_signal(sync_obj, sync_status, CAM_SYNC_COMMON_EVENT_SYNX);
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"synx signal failed with %d, sync_obj=%d, synx_status=%d, sync_status=%d",
			sync_obj, synx_status, sync_status, rc);
	}

	return rc;
}

static int cam_sync_register_synx_bind_ops(
	struct synx_register_params *object)
{
	int rc = 0;

	rc = synx_register_ops(object);
	if (rc)
		CAM_ERR(CAM_SYNC, "synx registration fail with rc=%d", rc);

	return rc;
}

static void cam_sync_unregister_synx_bind_ops(
	struct synx_register_params *object)
{
	int rc = 0;

	rc = synx_deregister_ops(object);
	if (rc)
		CAM_ERR(CAM_SYNC, "sync unregistration fail with %d", rc);
}

static void cam_sync_configure_synx_obj(struct synx_register_params *object)
{
	struct synx_register_params *params = object;

	params->name = CAM_SYNC_NAME;
	params->type = SYNX_TYPE_CSL;
	params->ops.register_callback = cam_sync_register_callback;
	params->ops.deregister_callback = cam_sync_deregister_callback;
	params->ops.enable_signaling = cam_sync_get_obj_ref;
	params->ops.signal = cam_synx_sync_signal;
}
#endif

static int cam_sync_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc;
	int idx;
	struct platform_device *pdev = to_platform_device(dev);

	sync_dev = kzalloc(sizeof(*sync_dev), GFP_KERNEL);
	if (!sync_dev)
		return -ENOMEM;

	mutex_init(&sync_dev->table_lock);
	spin_lock_init(&sync_dev->cam_sync_eventq_lock);

	for (idx = 0; idx < CAM_SYNC_MAX_OBJS; idx++)
		spin_lock_init(&sync_dev->row_spinlocks[idx]);

	sync_dev->vdev = video_device_alloc();
	if (!sync_dev->vdev) {
		rc = -ENOMEM;
		goto vdev_fail;
	}

	rc = cam_sync_media_controller_init(sync_dev, pdev);
	if (rc < 0)
		goto mcinit_fail;

	sync_dev->vdev->v4l2_dev = &sync_dev->v4l2_dev;

	rc = v4l2_device_register(&(pdev->dev), sync_dev->vdev->v4l2_dev);
	if (rc < 0)
		goto register_fail;

	strlcpy(sync_dev->vdev->name, CAM_SYNC_NAME,
				sizeof(sync_dev->vdev->name));
	sync_dev->vdev->release  = video_device_release_empty;
	sync_dev->vdev->fops     = &cam_sync_v4l2_fops;
	sync_dev->vdev->ioctl_ops = &g_cam_sync_ioctl_ops;
	sync_dev->vdev->minor     = -1;
	sync_dev->vdev->device_caps |= V4L2_CAP_VIDEO_CAPTURE;
	sync_dev->vdev->vfl_type  = VFL_TYPE_VIDEO;
	rc = video_register_device(sync_dev->vdev, VFL_TYPE_VIDEO, -1);
	if (rc < 0) {
		CAM_ERR(CAM_SYNC,
			"video device registration failure rc = %d, name = %s, device_caps = %d",
			rc, sync_dev->vdev->name, sync_dev->vdev->device_caps);
		goto v4l2_fail;
	}

	cam_sync_init_entity(sync_dev);
	video_set_drvdata(sync_dev->vdev, sync_dev);
	bitmap_zero(sync_dev->bitmap, CAM_SYNC_MAX_OBJS);

	/*
	 * We treat zero as invalid handle, so we will keep the 0th bit set
	 * always
	 */
	set_bit(0, sync_dev->bitmap);

	sync_dev->work_queue = alloc_workqueue(CAM_SYNC_WORKQUEUE_NAME,
		WQ_HIGHPRI | WQ_UNBOUND, 1);

	if (!sync_dev->work_queue) {
		CAM_ERR(CAM_SYNC,
			"Error: high priority work queue creation failed");
		rc = -ENOMEM;
		goto v4l2_fail;
	}

	/* Initialize dma fence driver */
	rc = cam_dma_fence_driver_init();
	if (rc) {
		CAM_ERR(CAM_SYNC,
			"DMA fence driver initialization failed rc: %d", rc);
		goto workq_destroy;
	}

	trigger_cb_without_switch = false;
	cam_sync_create_debugfs();
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
	CAM_DBG(CAM_SYNC, "Registering with synx driver");
	cam_sync_configure_synx_obj(&sync_dev->params);
	rc = cam_sync_register_synx_bind_ops(&sync_dev->params);
	if (rc)
		goto dma_driver_deinit;
#endif
	CAM_DBG(CAM_SYNC, "Component bound successfully");
	return rc;

#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
dma_driver_deinit:
	cam_dma_fence_driver_deinit();
#endif
workq_destroy:
	destroy_workqueue(sync_dev->work_queue);
v4l2_fail:
	v4l2_device_unregister(sync_dev->vdev->v4l2_dev);
register_fail:
	cam_sync_media_controller_cleanup(sync_dev);
mcinit_fail:
	video_unregister_device(sync_dev->vdev);
	video_device_release(sync_dev->vdev);
vdev_fail:
	mutex_destroy(&sync_dev->table_lock);
	kfree(sync_dev);
	return rc;
}

static void cam_sync_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int i;

	v4l2_device_unregister(sync_dev->vdev->v4l2_dev);
	cam_sync_media_controller_cleanup(sync_dev);
#if IS_REACHABLE(CONFIG_MSM_GLOBAL_SYNX)
	cam_sync_unregister_synx_bind_ops(&sync_dev->params);
#endif
	video_unregister_device(sync_dev->vdev);
	video_device_release(sync_dev->vdev);
	sync_dev->dentry = NULL;

	cam_dma_fence_driver_deinit();
	for (i = 0; i < CAM_SYNC_MAX_OBJS; i++)
		spin_lock_init(&sync_dev->row_spinlocks[i]);

	kfree(sync_dev);
	sync_dev = NULL;
}

const static struct component_ops cam_sync_component_ops = {
	.bind = cam_sync_component_bind,
	.unbind = cam_sync_component_unbind,
};

static int cam_sync_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_SYNC, "Adding Sync component");
	rc = component_add(&pdev->dev, &cam_sync_component_ops);
	if (rc)
		CAM_ERR(CAM_SYNC, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_sync_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_sync_component_ops);
	return 0;
}

static const struct of_device_id cam_sync_dt_match[] = {
	{.compatible = "qcom,cam-sync"},
	{}
};

MODULE_DEVICE_TABLE(of, cam_sync_dt_match);

struct platform_driver cam_sync_driver = {
	.probe = cam_sync_probe,
	.remove = cam_sync_remove,
	.driver = {
		.name = "cam_sync",
		.owner = THIS_MODULE,
		.of_match_table = cam_sync_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_sync_init(void)
{
	return platform_driver_register(&cam_sync_driver);
}

void cam_sync_exit(void)
{
	platform_driver_unregister(&cam_sync_driver);
}

MODULE_DESCRIPTION("Camera sync driver");
MODULE_LICENSE("GPL v2");
