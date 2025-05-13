// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2018, 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_sync_util.h"
#include "cam_req_mgr_workq.h"
#include "cam_common_util.h"

extern unsigned long cam_sync_monitor_mask;

static int cam_generic_expand_monitor_table(int idx, struct mutex *lock,
	struct cam_generic_fence_monitor_data **mon_data)
{
	struct cam_generic_fence_monitor_data *row_mon_data;

	mutex_lock(lock);
	row_mon_data = mon_data[(idx / CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ)];
	if (!row_mon_data) {
		row_mon_data = kzalloc(
			sizeof(struct cam_generic_fence_monitor_data) *
			CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ, GFP_KERNEL);
		mon_data[(idx / CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ)] = row_mon_data;
	}
	if (!row_mon_data) {
		CAM_ERR(CAM_SYNC, "Error allocating memory %d, idx %d",
			sizeof(struct cam_generic_fence_monitor_data) *
			CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ, idx);
		mutex_unlock(lock);
		return -ENOMEM;
	}

	mutex_unlock(lock);

	return 0;
}

static inline struct cam_generic_fence_monitor_entry *__cam_sync_get_monitor_entries(int idx)
{
	struct cam_generic_fence_monitor_data *mon_data;

	mon_data = CAM_SYNC_MONITOR_GET_DATA(idx);
	if (mon_data->swap_monitor_entries)
		return mon_data->prev_monitor_entries;
	else
		return mon_data->monitor_entries;
}

static inline struct cam_generic_fence_monitor_entry *__cam_sync_get_prev_monitor_entries(int idx)
{
	struct cam_generic_fence_monitor_data *mon_data;

	mon_data = CAM_SYNC_MONITOR_GET_DATA(idx);
	if (mon_data->swap_monitor_entries)
		return mon_data->monitor_entries;
	else
		return mon_data->prev_monitor_entries;
}

const char *cam_fence_op_to_string(
	enum cam_fence_op op)
{
	switch (op) {
	case CAM_FENCE_OP_CREATE:
		return "CREATE";
	case CAM_FENCE_OP_REGISTER_CB:
		return "REGISTER_CB";
	case CAM_FENCE_OP_SIGNAL:
		return "SIGNAL";
	case CAM_FENCE_OP_UNREGISTER_ON_SIGNAL:
		return "UNREGISTER_ON_SIGNAL";
	case CAM_FENCE_OP_UNREGISTER_CB:
		return "UNREGISTER_CB";
	case CAM_FENCE_OP_SKIP_REGISTER_CB:
		return "SKIP_REGISTER_CB";
	case CAM_FENCE_OP_ALREADY_REGISTERED_CB:
		return "ALREADY_REGISTERED_CB";
	case CAM_FENCE_OP_DESTROY:
		return "DESTROY";
	default:
		return "INVALID";
	}
}

static void __cam_sync_save_previous_monitor_data(
	struct sync_table_row *row)
{
	struct cam_generic_fence_monitor_data *row_mon_data;

	if (!sync_dev->mon_data)
		return;

	row_mon_data = CAM_SYNC_MONITOR_GET_DATA(row->sync_id);

	/* save current usage details into prev variables */
	strscpy(row_mon_data->prev_name, row->name, SYNC_DEBUG_NAME_LEN);
	row_mon_data->prev_type         = row->type;
	row_mon_data->prev_obj_id       = row->sync_id;
	row_mon_data->prev_state        = row->state;
	row_mon_data->prev_remaining    = row->remaining;
	row_mon_data->prev_monitor_head = atomic64_read(&row_mon_data->monitor_head);

	/* Toggle swap flag. Avoid copying and just read/write using correct table idx */
	row_mon_data->swap_monitor_entries = !row_mon_data->swap_monitor_entries;
}

void cam_generic_fence_update_monitor_array(int idx,
	struct mutex *lock,
	struct cam_generic_fence_monitor_data **mon_data,
	enum cam_fence_op op)
{
	int iterator, rc;
	struct cam_generic_fence_monitor_data *row_mon_data;
	struct cam_generic_fence_monitor_entry *row_mon_entries;

	/* Validate inputs */
	if (!lock || !mon_data)
		return;

	row_mon_data = mon_data[(idx / CAM_GENERIC_MONITOR_TABLE_ENTRY_SZ)];
	if (!row_mon_data) {
		rc = cam_generic_expand_monitor_table(idx, lock, mon_data);
		if (rc) {
			CAM_ERR(CAM_SYNC, "Failed to expand monitor table");
			return;
		}
	}

	row_mon_data = CAM_GENERIC_MONITOR_GET_DATA(mon_data, idx);
	if (op == CAM_FENCE_OP_CREATE)
		atomic64_set(&row_mon_data->monitor_head, -1);
	if (row_mon_data->swap_monitor_entries)
		row_mon_entries = row_mon_data->monitor_entries;
	else
		row_mon_entries = row_mon_data->prev_monitor_entries;

	CAM_SYNC_INC_MONITOR_HEAD(&row_mon_data->monitor_head, &iterator);
	CAM_GET_TIMESTAMP(row_mon_entries[iterator].timestamp);
	row_mon_entries[iterator].op = op;
}

static void __cam_generic_fence_dump_monitor_entries(
	struct cam_generic_fence_monitor_entry *monitor_entries,
	uint32_t index, uint32_t num_entries)
{
	int i = 0;
	uint64_t ms, hrs, min, sec;

	for (i = 0; i < num_entries; i++) {
		CAM_CONVERT_TIMESTAMP_FORMAT(monitor_entries[index].timestamp,
			hrs, min, sec, ms);

		CAM_INFO(CAM_SYNC,
			"**** %llu:%llu:%llu.%llu : Index[%d] Op[%s]",
			hrs, min, sec, ms,
			index,
			cam_fence_op_to_string(monitor_entries[index].op));

		index = (index + 1) % CAM_SYNC_MONITOR_MAX_ENTRIES;
	}
}

static int __cam_generic_fence_get_monitor_entries_info(uint64_t  state_head,
	uint32_t *oldest_entry, uint32_t *num_entries)
{
	*oldest_entry = 0;
	*num_entries  = 0;

	if (state_head == -1) {
		return -EINVAL;
	} else if (state_head < CAM_SYNC_MONITOR_MAX_ENTRIES) {
		/* head starts from -1 */
		*num_entries = state_head + 1;
		*oldest_entry = 0;
	} else {
		*num_entries = CAM_SYNC_MONITOR_MAX_ENTRIES;
		div_u64_rem(state_head + 1,
			CAM_SYNC_MONITOR_MAX_ENTRIES, oldest_entry);
	}

	return 0;
}

void cam_generic_fence_dump_monitor_array(
	struct cam_generic_fence_monitor_obj_info *obj_info)
{
	int rc;
	uint32_t num_entries, oldest_entry;
	uint64_t ms, hrs, min, sec;
	struct timespec64 current_ts;
	struct cam_generic_fence_monitor_data *mon_data = obj_info->monitor_data;

	/* Check if there are any current entries in the monitor data */
	rc = __cam_generic_fence_get_monitor_entries_info(
		atomic64_read(&mon_data->monitor_head),
		&oldest_entry, &num_entries);

	if (rc)
		return;

	/* Print current monitor entries */
	CAM_GET_TIMESTAMP(current_ts);
	CAM_CONVERT_TIMESTAMP_FORMAT(current_ts, hrs, min, sec, ms);
	switch (obj_info->fence_type) {
	case CAM_GENERIC_FENCE_TYPE_SYNC_OBJ:
		CAM_INFO(CAM_SYNC,
			"======== %llu:%llu:%llu:%llu Dumping monitor information for sync obj %s, type %d, sync_id %d state %d remaining %d ref_cnt %d num_entries %u ===========",
			hrs, min, sec, ms, obj_info->name, obj_info->sync_type,
			obj_info->obj_id, obj_info->state, obj_info->remaining,
			obj_info->ref_cnt, num_entries);
		break;
	case CAM_GENERIC_FENCE_TYPE_DMA_FENCE:
		CAM_INFO(CAM_DMA_FENCE,
			"======== %llu:%llu:%llu:%llu Dumping monitor information for dma obj %s, fd %d sync_id %d state %d ref_cnt %d num_entries %u ===========",
			hrs, min, sec, ms, obj_info->name, obj_info->obj_id,
			obj_info->sync_id, obj_info->state, obj_info->ref_cnt,
			num_entries);
		break;
	case CAM_GENERIC_FENCE_TYPE_SYNX_OBJ:
		CAM_INFO(CAM_SYNX,
			"======== %llu:%llu:%llu:%llu Dumping monitor information for synx obj %s, synx_id %d sync_id %d state %d ref_cnt %d num_entries %u ===========",
			hrs, min, sec, ms, obj_info->name, obj_info->obj_id,
			obj_info->sync_id, obj_info->state, obj_info->ref_cnt,
			num_entries);
		break;
	default:
		break;
	}

	__cam_generic_fence_dump_monitor_entries(obj_info->monitor_entries,
		oldest_entry, num_entries);


	/* Check if there are any previous entries in the monitor data */
	rc = __cam_generic_fence_get_monitor_entries_info(
		mon_data->prev_monitor_head,
		&oldest_entry, &num_entries);

	if (rc)
		return;

	/* Print previous monitor entries */
	CAM_GET_TIMESTAMP(current_ts);
	CAM_CONVERT_TIMESTAMP_FORMAT(current_ts, hrs, min, sec, ms);
	switch (obj_info->fence_type) {
	case CAM_GENERIC_FENCE_TYPE_SYNC_OBJ:
		CAM_INFO(CAM_SYNC,
			"======== %llu:%llu:%llu:%llu Dumping previous monitor information for sync obj %s, type %d, sync_id %d state %d remaining %d num_entries %u ===========",
			hrs, min, sec, ms, mon_data->prev_name, mon_data->prev_type,
			mon_data->prev_obj_id, mon_data->prev_state, mon_data->prev_remaining,
			num_entries);
		break;
	case CAM_GENERIC_FENCE_TYPE_DMA_FENCE:
		CAM_INFO(CAM_DMA_FENCE,
			"======== %llu:%llu:%llu:%llu Dumping previous monitor information for dma obj %s, fd %d sync_id %d state %d num_entries %u ===========",
			hrs, min, sec, ms, mon_data->prev_name, mon_data->prev_obj_id,
			mon_data->prev_sync_id, mon_data->prev_state,
			num_entries);
		break;
	case CAM_GENERIC_FENCE_TYPE_SYNX_OBJ:
		CAM_INFO(CAM_SYNX,
			"======== %llu:%llu:%llu:%llu Dumping previous monitor information for synx obj %s, synx_id %d sync_id %d state %d num_entries %u ===========",
			hrs, min, sec, ms, mon_data->prev_name, mon_data->prev_obj_id,
			mon_data->prev_sync_id, mon_data->prev_state,
			num_entries);
		break;
	default:
		break;
	}

	__cam_generic_fence_dump_monitor_entries(obj_info->prev_monitor_entries,
		oldest_entry, num_entries);

}

void cam_sync_dump_monitor_array(struct sync_table_row *row)
{
	struct cam_generic_fence_monitor_obj_info obj_info;

	if (!sync_dev->mon_data ||
		!test_bit(CAM_GENERIC_FENCE_TYPE_SYNC_OBJ, &cam_sync_monitor_mask) ||
		!(CAM_GENERIC_MONITOR_GET_DATA(sync_dev->mon_data, row->sync_id)->prev_obj_id))
		return;

	obj_info.name = row->name;
	obj_info.sync_type = row->type;
	obj_info.obj_id = row->sync_id;
	obj_info.state = row->state;
	obj_info.remaining = row->remaining;
	obj_info.ref_cnt = atomic_read(&row->ref_cnt);
	obj_info.monitor_data = CAM_SYNC_MONITOR_GET_DATA(row->sync_id);
	obj_info.fence_type = CAM_GENERIC_FENCE_TYPE_SYNC_OBJ;
	obj_info.monitor_entries =
		__cam_sync_get_monitor_entries(row->sync_id);
	obj_info.prev_monitor_entries =
		__cam_sync_get_prev_monitor_entries(row->sync_id);
	cam_generic_fence_dump_monitor_array(&obj_info);
}

int cam_sync_util_find_and_set_empty_row(struct sync_device *sync_dev,
	long *idx)
{
	int rc = 0;

	mutex_lock(&sync_dev->table_lock);

	*idx = find_first_zero_bit(sync_dev->bitmap, CAM_SYNC_MAX_OBJS);

	if (*idx < CAM_SYNC_MAX_OBJS)
		set_bit(*idx, sync_dev->bitmap);
	else
		rc = -1;

	mutex_unlock(&sync_dev->table_lock);

	return rc;
}

int cam_sync_init_row(struct sync_table_row *table,
	uint32_t idx, const char *name, uint32_t type)
{
	struct sync_table_row *row = table + idx;

	if (!table || idx <= 0 || idx >= CAM_SYNC_MAX_OBJS)
		return -EINVAL;

	strlcpy(row->name, name, SYNC_DEBUG_NAME_LEN);
	INIT_LIST_HEAD(&row->parents_list);
	INIT_LIST_HEAD(&row->children_list);
	row->type = type;
	row->sync_id = idx;
	row->state = CAM_SYNC_STATE_ACTIVE;
	row->remaining = 0;
	atomic_set(&row->ref_cnt, 0);
	init_completion(&row->signaled);
	INIT_LIST_HEAD(&row->callback_list);
	INIT_LIST_HEAD(&row->user_payload_list);
	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNC_OBJ, &cam_sync_monitor_mask)) {
		cam_generic_fence_update_monitor_array(idx, &sync_dev->table_lock,
			sync_dev->mon_data,
			CAM_FENCE_OP_CREATE);
	}
	CAM_DBG(CAM_SYNC,
		"row name:%s sync_id:%i [idx:%u] row_state:%u ",
		row->name, row->sync_id, idx, row->state);

	return 0;
}

int cam_sync_init_group_object(struct sync_table_row *table,
	uint32_t idx,
	uint32_t *sync_objs,
	uint32_t num_objs)
{
	int i, rc;
	struct sync_child_info *child_info;
	struct sync_parent_info *parent_info;
	struct sync_table_row *row = table + idx;
	struct sync_table_row *child_row = NULL;

	cam_sync_init_row(table, idx, "merged_fence", CAM_SYNC_TYPE_GROUP);

	/*
	 * While traversing for children, parent's row list is updated with
	 * child info and each child's row is updated with parent info.
	 * If any child state is ERROR or SUCCESS, it will not be added to list.
	 */
	for (i = 0; i < num_objs; i++) {
		child_row = table + sync_objs[i];
		spin_lock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);

		/* validate child */
		if ((child_row->type == CAM_SYNC_TYPE_GROUP) ||
			(child_row->state == CAM_SYNC_STATE_INVALID)) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
			CAM_ERR(CAM_SYNC,
				"Invalid child fence:%i state:%u type:%u",
				child_row->sync_id, child_row->state,
				child_row->type);
			rc = -EINVAL;
			goto clean_children_info;
		}

		/* check for child's state */
		if ((child_row->state == CAM_SYNC_STATE_SIGNALED_ERROR) ||
			(child_row->state == CAM_SYNC_STATE_SIGNALED_CANCEL)) {
			row->state = child_row->state;
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
			continue;
		}
		if (child_row->state != CAM_SYNC_STATE_ACTIVE) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
			continue;
		}

		row->remaining++;

		/* Add child info */
		child_info = kzalloc(sizeof(*child_info), GFP_ATOMIC);
		if (!child_info) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
			rc = -ENOMEM;
			goto clean_children_info;
		}
		child_info->sync_id = sync_objs[i];
		list_add_tail(&child_info->list, &row->children_list);

		/* Add parent info */
		parent_info = kzalloc(sizeof(*parent_info), GFP_ATOMIC);
		if (!parent_info) {
			spin_unlock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
			rc = -ENOMEM;
			goto clean_children_info;
		}
		parent_info->sync_id = idx;
		list_add_tail(&parent_info->list, &child_row->parents_list);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
	}

	if (!row->remaining) {
		if ((row->state != CAM_SYNC_STATE_SIGNALED_ERROR) &&
			(row->state != CAM_SYNC_STATE_SIGNALED_CANCEL))
			row->state = CAM_SYNC_STATE_SIGNALED_SUCCESS;
		complete_all(&row->signaled);
	}

	return 0;

clean_children_info:
	row->state = CAM_SYNC_STATE_INVALID;
	for (i = i-1; i >= 0; i--) {
		spin_lock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
		child_row = table + sync_objs[i];
		cam_sync_util_cleanup_parents_list(child_row,
			SYNC_LIST_CLEAN_ONE, idx);
		spin_unlock_bh(&sync_dev->row_spinlocks[sync_objs[i]]);
	}

	cam_sync_util_cleanup_children_list(row, SYNC_LIST_CLEAN_ALL, 0);
	return rc;
}

int cam_sync_deinit_object(struct sync_table_row *table, uint32_t idx,
	struct cam_sync_check_for_dma_release *check_for_dma_release,
	struct cam_sync_check_for_synx_release *check_for_synx_release)
{
	struct sync_table_row      *row = table + idx;
	struct sync_child_info     *child_info, *temp_child;
	struct sync_callback_info  *sync_cb, *temp_cb;
	struct sync_parent_info    *parent_info, *temp_parent;
	struct sync_user_payload   *upayload_info, *temp_upayload;
	struct sync_table_row      *child_row = NULL, *parent_row = NULL;
	struct list_head            temp_child_list, temp_parent_list;

	if (!table || (idx <= 0) || (idx >= CAM_SYNC_MAX_OBJS))
		return -EINVAL;

	CAM_DBG(CAM_SYNC,
		"row name:%s sync_id:%i [idx:%u] row_state:%u",
		row->name, row->sync_id, idx, row->state);

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	if (row->state == CAM_SYNC_STATE_INVALID) {
		spin_unlock_bh(&sync_dev->row_spinlocks[idx]);
		CAM_ERR(CAM_SYNC,
			"Error: accessing an uninitialized sync obj: idx = %d name = %s",
			idx,
			row->name);
		return -EINVAL;
	}

	if (row->state == CAM_SYNC_STATE_ACTIVE)
		CAM_DBG(CAM_SYNC,
			"Destroying an active sync object name:%s id:%i",
			row->name, row->sync_id);

	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNC_OBJ, &cam_sync_monitor_mask)) {
		cam_generic_fence_update_monitor_array(idx, &sync_dev->table_lock,
			sync_dev->mon_data,
			CAM_FENCE_OP_DESTROY);
		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNC_OBJ_DUMP, &cam_sync_monitor_mask))
			cam_sync_dump_monitor_array(row);
		__cam_sync_save_previous_monitor_data(row);
	}

	row->state = CAM_SYNC_STATE_INVALID;

	/* Object's child and parent objects will be added into this list */
	INIT_LIST_HEAD(&temp_child_list);
	INIT_LIST_HEAD(&temp_parent_list);

	list_for_each_entry_safe(child_info, temp_child, &row->children_list,
		list) {
		if (child_info->sync_id <= 0)
			continue;

		list_del_init(&child_info->list);
		list_add_tail(&child_info->list, &temp_child_list);
	}

	list_for_each_entry_safe(parent_info, temp_parent, &row->parents_list,
		list) {
		if (parent_info->sync_id <= 0)
			continue;

		list_del_init(&parent_info->list);
		list_add_tail(&parent_info->list, &temp_parent_list);
	}

	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);

	/* Cleanup the child to parent link from child list */
	while (!list_empty(&temp_child_list)) {
		child_info = list_first_entry(&temp_child_list,
			struct sync_child_info, list);
		child_row = sync_dev->sync_table + child_info->sync_id;

		spin_lock_bh(&sync_dev->row_spinlocks[child_info->sync_id]);

		if (child_row->state == CAM_SYNC_STATE_INVALID) {
			list_del_init(&child_info->list);
			spin_unlock_bh(&sync_dev->row_spinlocks[
				child_info->sync_id]);
			kfree(child_info);
			continue;
		}

		if (child_row->state == CAM_SYNC_STATE_ACTIVE)
			CAM_DBG(CAM_SYNC,
				"Warning: destroying active child sync obj = %s[%d]",
				child_row->name,
				child_info->sync_id);

		cam_sync_util_cleanup_parents_list(child_row,
			SYNC_LIST_CLEAN_ONE, idx);

		list_del_init(&child_info->list);
		spin_unlock_bh(&sync_dev->row_spinlocks[child_info->sync_id]);
		kfree(child_info);
	}

	/* Cleanup the parent to child link */
	while (!list_empty(&temp_parent_list)) {
		parent_info = list_first_entry(&temp_parent_list,
			struct sync_parent_info, list);
		parent_row = sync_dev->sync_table + parent_info->sync_id;

		spin_lock_bh(&sync_dev->row_spinlocks[parent_info->sync_id]);

		if (parent_row->state == CAM_SYNC_STATE_INVALID) {
			list_del_init(&parent_info->list);
			spin_unlock_bh(&sync_dev->row_spinlocks[
				parent_info->sync_id]);
			kfree(parent_info);
			continue;
		}

		if (parent_row->state == CAM_SYNC_STATE_ACTIVE)
			CAM_DBG(CAM_SYNC,
				"Warning: destroying active parent sync obj = %s[%d]",
				parent_row->name,
				parent_info->sync_id);

		cam_sync_util_cleanup_children_list(parent_row,
			SYNC_LIST_CLEAN_ONE, idx);

		list_del_init(&parent_info->list);
		spin_unlock_bh(&sync_dev->row_spinlocks[parent_info->sync_id]);
		kfree(parent_info);
	}

	spin_lock_bh(&sync_dev->row_spinlocks[idx]);
	list_for_each_entry_safe(upayload_info, temp_upayload,
			&row->user_payload_list, list) {
		list_del_init(&upayload_info->list);
		kfree(upayload_info);
	}

	list_for_each_entry_safe(sync_cb, temp_cb,
			&row->callback_list, list) {
		list_del_init(&sync_cb->list);
		kfree(sync_cb);
	}

	/* Decrement ref cnt for imported dma fence */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_DMA_FENCE, &row->ext_fence_mask)) {
		cam_dma_fence_get_put_ref(false, row->dma_fence_info.dma_fence_row_idx);

		/* Check if same dma fence is being released with the sync obj */
		if (check_for_dma_release) {
			if (row->dma_fence_info.dma_fence_fd ==
				check_for_dma_release->dma_fence_fd) {
				check_for_dma_release->sync_created_with_dma =
					row->dma_fence_info.sync_created_with_dma;
				check_for_dma_release->dma_fence_row_idx =
					row->dma_fence_info.dma_fence_row_idx;
			}
		}
	}

	/* Check if same synx obj is being released with the sync obj */
	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &row->ext_fence_mask)) {
		if (check_for_synx_release) {
			if (row->synx_obj_info.synx_obj ==
				check_for_synx_release->synx_obj) {
				check_for_synx_release->synx_obj_row_idx =
					row->synx_obj_info.synx_obj_row_idx;
				check_for_synx_release->sync_created_with_synx =
					row->synx_obj_info.sync_created_with_synx;
			}
		}
	}

	memset(row, 0, sizeof(*row));
	clear_bit(idx, sync_dev->bitmap);
	INIT_LIST_HEAD(&row->callback_list);
	INIT_LIST_HEAD(&row->parents_list);
	INIT_LIST_HEAD(&row->children_list);
	INIT_LIST_HEAD(&row->user_payload_list);
	spin_unlock_bh(&sync_dev->row_spinlocks[idx]);

	return 0;
}

void cam_sync_util_cb_dispatch(struct work_struct *cb_dispatch_work)
{
	struct sync_callback_info *cb_info = container_of(cb_dispatch_work,
		struct sync_callback_info,
		cb_dispatch_work);
	sync_callback sync_data = cb_info->callback_func;
	void *cb = cb_info->callback_func;

	cam_common_util_thread_switch_delay_detect(
		"cam_sync_workq", "schedule", cb,
		cb_info->workq_scheduled_ts,
		CAM_WORKQ_SCHEDULE_TIME_THRESHOLD);
	sync_data(cb_info->sync_obj, cb_info->status, cb_info->cb_data);

	kfree(cb_info);
}

void cam_sync_util_dispatch_signaled_cb(int32_t sync_obj,
	uint32_t status, uint32_t event_cause)
{
	struct sync_callback_info  *sync_cb;
	struct sync_user_payload   *payload_info;
	struct sync_callback_info  *temp_sync_cb;
	struct sync_table_row      *signalable_row;
	struct sync_user_payload   *temp_payload_info;

	signalable_row = sync_dev->sync_table + sync_obj;
	if (signalable_row->state == CAM_SYNC_STATE_INVALID) {
		CAM_DBG(CAM_SYNC,
			"Accessing invalid sync object:%s[%i]", signalable_row->name,
			sync_obj);
		return;
	}

	/* Dispatch kernel callbacks if any were registered earlier */
	list_for_each_entry_safe(sync_cb,
		temp_sync_cb, &signalable_row->callback_list, list) {
		sync_cb->status = status;
		list_del_init(&sync_cb->list);
		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNC_OBJ,
			&cam_sync_monitor_mask))
			cam_generic_fence_update_monitor_array(sync_obj,
				&sync_dev->table_lock, sync_dev->mon_data,
				CAM_FENCE_OP_UNREGISTER_ON_SIGNAL);
		queue_work(sync_dev->work_queue,
			&sync_cb->cb_dispatch_work);
	}

	/* Dispatch user payloads if any were registered earlier */
	list_for_each_entry_safe(payload_info, temp_payload_info,
		&signalable_row->user_payload_list, list) {
		spin_lock_bh(&sync_dev->cam_sync_eventq_lock);
		if (!sync_dev->cam_sync_eventq) {
			spin_unlock_bh(
				&sync_dev->cam_sync_eventq_lock);
			break;
		}
		spin_unlock_bh(&sync_dev->cam_sync_eventq_lock);
		cam_sync_util_send_v4l2_event(
			CAM_SYNC_V4L_EVENT_ID_CB_TRIG,
			sync_obj,
			status,
			payload_info->payload_data,
			CAM_SYNC_PAYLOAD_WORDS * sizeof(__u64),
			event_cause);

		list_del_init(&payload_info->list);

		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNC_OBJ,
			&cam_sync_monitor_mask))
			cam_generic_fence_update_monitor_array(sync_obj,
				&sync_dev->table_lock, sync_dev->mon_data,
				CAM_FENCE_OP_UNREGISTER_ON_SIGNAL);
		/*
		 * We can free the list node here because
		 * sending V4L event will make a deep copy
		 * anyway
		 */
		kfree(payload_info);
	}

	/*
	 * This needs to be done because we want to unblock anyone
	 * who might be blocked and waiting on this sync object
	 */
	complete_all(&signalable_row->signaled);
}

void cam_sync_util_send_v4l2_event(uint32_t id,
	uint32_t sync_obj,
	int status,
	void *payload,
	int len, uint32_t event_cause)
{
	struct v4l2_event event;
	__u64 *payload_data = NULL;

	if (sync_dev->version == CAM_SYNC_V4L_EVENT_V2) {
		struct cam_sync_ev_header_v2 *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT_V2;

		ev_header = CAM_SYNC_GET_HEADER_PTR_V2(event);
		ev_header->sync_obj = sync_obj;
		ev_header->status = status;
		ev_header->version = sync_dev->version;
		ev_header->evt_param[CAM_SYNC_EVENT_REASON_CODE_INDEX] =
			event_cause;
		payload_data = CAM_SYNC_GET_PAYLOAD_PTR_V2(event, __u64);
	} else {
		struct cam_sync_ev_header *ev_header = NULL;

		event.id = id;
		event.type = CAM_SYNC_V4L_EVENT;

		ev_header = CAM_SYNC_GET_HEADER_PTR(event);
		ev_header->sync_obj = sync_obj;
		ev_header->status = status;
		payload_data = CAM_SYNC_GET_PAYLOAD_PTR(event, __u64);
	}

	memcpy(payload_data, payload, len);
	v4l2_event_queue(sync_dev->vdev, &event);
	CAM_DBG(CAM_SYNC, "send v4l2 event version %d for sync_obj :%d",
		sync_dev->version,
		sync_obj);
}

int cam_sync_util_update_parent_state(struct sync_table_row *parent_row,
	int new_state)
{
	int rc = 0;

	switch (parent_row->state) {
	case CAM_SYNC_STATE_ACTIVE:
	case CAM_SYNC_STATE_SIGNALED_SUCCESS:
		parent_row->state = new_state;
		break;

	case CAM_SYNC_STATE_SIGNALED_ERROR:
	case CAM_SYNC_STATE_SIGNALED_CANCEL:
		break;

	case CAM_SYNC_STATE_INVALID:
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

void cam_sync_util_cleanup_children_list(struct sync_table_row *row,
	uint32_t list_clean_type, uint32_t sync_obj)
{
	struct sync_child_info *child_info = NULL;
	struct sync_child_info *temp_child_info = NULL;
	uint32_t                curr_sync_obj;

	list_for_each_entry_safe(child_info,
			temp_child_info, &row->children_list, list) {
		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(child_info->sync_id != sync_obj))
			continue;

		curr_sync_obj = child_info->sync_id;
		list_del_init(&child_info->list);
		kfree(child_info);

		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(curr_sync_obj == sync_obj))
			break;
	}
}

void cam_sync_util_cleanup_parents_list(struct sync_table_row *row,
	uint32_t list_clean_type, uint32_t sync_obj)
{
	struct sync_parent_info *parent_info = NULL;
	struct sync_parent_info *temp_parent_info = NULL;
	uint32_t                 curr_sync_obj;

	list_for_each_entry_safe(parent_info,
			temp_parent_info, &row->parents_list, list) {
		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(parent_info->sync_id != sync_obj))
			continue;

		curr_sync_obj = parent_info->sync_id;
		list_del_init(&parent_info->list);
		kfree(parent_info);

		if ((list_clean_type == SYNC_LIST_CLEAN_ONE) &&
			(curr_sync_obj == sync_obj))
			break;
	}
}
