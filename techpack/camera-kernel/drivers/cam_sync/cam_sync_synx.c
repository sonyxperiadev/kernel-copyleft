// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_sync_synx.h"
#include "cam_sync_util.h"

extern unsigned long cam_sync_monitor_mask;

/**
 * struct cam_synx_obj_row - Synx obj row
 */
struct cam_synx_obj_row {
	char                            name[CAM_SYNX_OBJ_NAME_LEN];
	uint32_t                        synx_obj;
	enum cam_synx_obj_state         state;
	cam_sync_callback_for_synx_obj  sync_cb;
	bool                            cb_registered_for_sync;
	bool                            sync_signal_synx;
	int32_t                         sync_obj;
};

/**
 * struct cam_synx_obj_device - Synx obj device
 */
struct cam_synx_obj_device {
	struct cam_synx_obj_row rows[CAM_SYNX_MAX_OBJS];
	spinlock_t row_spinlocks[CAM_SYNX_MAX_OBJS];
	struct synx_session *session_handle;
	struct mutex dev_lock;
	DECLARE_BITMAP(bitmap, CAM_SYNX_MAX_OBJS);
	struct cam_generic_fence_monitor_data **monitor_data;
};

static struct cam_synx_obj_device *g_cam_synx_obj_dev;
static char cam_synx_session_name[64] = "Camera_Generic_Synx_Session";


static inline struct cam_generic_fence_monitor_entry *
	__cam_synx_obj_get_monitor_entries(int idx)
{
	struct cam_generic_fence_monitor_data *monitor_data;

	monitor_data = CAM_GENERIC_MONITOR_GET_DATA(
		g_cam_synx_obj_dev->monitor_data, idx);
	if (monitor_data->swap_monitor_entries)
		return monitor_data->prev_monitor_entries;
	else
		return monitor_data->monitor_entries;
}

static inline struct cam_generic_fence_monitor_entry *
	__cam_synx_obj_get_prev_monitor_entries(int idx)
{
	struct cam_generic_fence_monitor_data *monitor_data;

	monitor_data = CAM_GENERIC_MONITOR_GET_DATA(
		g_cam_synx_obj_dev->monitor_data, idx);
	if (monitor_data->swap_monitor_entries)
		return monitor_data->monitor_entries;
	else
		return monitor_data->prev_monitor_entries;
}

static int __cam_synx_obj_map_sync_status_util(uint32_t sync_status,
	uint32_t *out_synx_status)
{
	if (!out_synx_status)
		return -EINVAL;

	switch (sync_status) {
	case CAM_SYNC_STATE_SIGNALED_SUCCESS:
		*out_synx_status = SYNX_STATE_SIGNALED_SUCCESS;
		break;
	case CAM_SYNC_STATE_SIGNALED_CANCEL:
	default:
		*out_synx_status = SYNX_STATE_SIGNALED_CANCEL;
		break;
	}

	return 0;
}

static void __cam_synx_obj_save_previous_monitor_data(int32_t row_idx)
{
	struct cam_generic_fence_monitor_data *row_mon_data;
	struct cam_synx_obj_row *row;

	if (!g_cam_synx_obj_dev->monitor_data)
		return;

	row = &g_cam_synx_obj_dev->rows[row_idx];
	row_mon_data = CAM_GENERIC_MONITOR_GET_DATA(
		g_cam_synx_obj_dev->monitor_data, row_idx);

	/* save current usage details into prev variables */
	strscpy(row_mon_data->prev_name, row->name, CAM_SYNX_OBJ_NAME_LEN);
	row_mon_data->prev_obj_id          = row->synx_obj;
	row_mon_data->prev_sync_id         = row->sync_obj;
	row_mon_data->prev_state           = row->state;
	row_mon_data->prev_monitor_head    = atomic64_read(&row_mon_data->monitor_head);
	row_mon_data->swap_monitor_entries = !row_mon_data->swap_monitor_entries;
}

static void __cam_synx_obj_dump_monitor_array(int32_t row_idx)
{
	struct cam_generic_fence_monitor_obj_info obj_info;
	struct cam_synx_obj_row *row;

	if (!g_cam_synx_obj_dev->monitor_data ||
		!test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask))
		return;

	if (!CAM_GENERIC_MONITOR_GET_DATA(g_cam_synx_obj_dev->monitor_data,
		row_idx)->prev_obj_id)
		return;

	row = &g_cam_synx_obj_dev->rows[row_idx];

	obj_info.name = row->name;
	obj_info.obj_id = row->synx_obj;
	obj_info.state = row->state;
	obj_info.monitor_data = CAM_GENERIC_MONITOR_GET_DATA(
		g_cam_synx_obj_dev->monitor_data, row_idx);
	obj_info.fence_type = CAM_GENERIC_FENCE_TYPE_SYNX_OBJ;
	obj_info.sync_id = row->sync_obj;
	obj_info.monitor_entries =
		__cam_synx_obj_get_monitor_entries(row_idx);
	obj_info.prev_monitor_entries =
		__cam_synx_obj_get_prev_monitor_entries(row_idx);
	cam_generic_fence_dump_monitor_array(&obj_info);
}

static void __cam_synx_obj_signal_cb(u32 h_synx, int status, void *data)
{
	struct cam_synx_obj_signal_sync_obj signal_sync_obj;
	struct cam_synx_obj_row *synx_obj_row = NULL;
	int32_t idx;

	if (!data) {
		CAM_ERR(CAM_SYNX,
			"Invalid data passed to synx obj : No callback function set.");
		return;
	}

	synx_obj_row = (struct cam_synx_obj_row *)data;

	/* If this synx obj is signaled by sync obj, skip cb */
	if (synx_obj_row->sync_signal_synx)
		return;

	if (synx_obj_row->synx_obj != h_synx) {
		CAM_ERR(CAM_SYNX,
			"Synx obj: %d callback does not match synx obj: %d in sync table.",
			h_synx, synx_obj_row->synx_obj);
		return;
	}

	if (synx_obj_row->state == CAM_SYNX_OBJ_STATE_INVALID) {
		CAM_ERR(CAM_SYNX,
			"Synx obj :%d is in invalid state: %d",
			synx_obj_row->synx_obj, synx_obj_row->state);
		return;
	}

	CAM_DBG(CAM_SYNX, "Synx obj: %d signaled, signal sync obj: %d",
		 synx_obj_row->synx_obj, synx_obj_row->sync_obj);

	if ((synx_obj_row->cb_registered_for_sync) && (synx_obj_row->sync_cb)) {
		signal_sync_obj.synx_obj = synx_obj_row->synx_obj;
		switch (status) {
		case SYNX_STATE_SIGNALED_SUCCESS:
			signal_sync_obj.status = CAM_SYNC_STATE_SIGNALED_SUCCESS;
			break;
		case SYNX_STATE_SIGNALED_CANCEL:
			signal_sync_obj.status = CAM_SYNC_STATE_SIGNALED_CANCEL;
			break;
		default:
			CAM_WARN(CAM_SYNX,
				"Synx signal status %d is neither SUCCESS nor CANCEL, custom code?",
				status);
			signal_sync_obj.status = CAM_SYNC_STATE_SIGNALED_ERROR;
			break;
		}
		synx_obj_row->state = CAM_SYNX_OBJ_STATE_SIGNALED;
		synx_obj_row->sync_cb(synx_obj_row->sync_obj, &signal_sync_obj);
		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ,
			&cam_sync_monitor_mask)) {
			cam_synx_obj_find_obj_in_table(synx_obj_row->synx_obj, &idx);
			cam_generic_fence_update_monitor_array(idx,
				&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
				CAM_FENCE_OP_UNREGISTER_ON_SIGNAL);
		}
	}

}

/*
 * Synx APIs need to be invoked in non atomic context,
 * all these utils invoke synx driver
 */
static inline int __cam_synx_signal_util(
	uint32_t synx_hdl, uint32_t signal_status)
{
	return synx_signal(g_cam_synx_obj_dev->session_handle, synx_hdl, signal_status);
}

static inline int __cam_synx_deregister_cb_util(
	uint32_t synx_hdl, void *data)
{
	struct synx_callback_params cb_params;

	cb_params.userdata = data;
	cb_params.cancel_cb_func = NULL;
	cb_params.h_synx = synx_hdl;
	cb_params.cb_func = __cam_synx_obj_signal_cb;

	return synx_cancel_async_wait(g_cam_synx_obj_dev->session_handle, &cb_params);
}

static inline int __cam_synx_create_hdl_util(
	struct synx_create_params *params)
{
	return synx_create(g_cam_synx_obj_dev->session_handle, params);
}

static inline int __cam_synx_release_hdl_util(uint32_t synx_hdl)
{
	return synx_release(g_cam_synx_obj_dev->session_handle, synx_hdl);
}

static inline int __cam_synx_import_hdl_util(
	struct synx_import_params *params)
{
	return synx_import(g_cam_synx_obj_dev->session_handle, params);
}

static inline int __cam_synx_register_cb_util(
	struct synx_callback_params *cb_params)
{
	return synx_async_wait(g_cam_synx_obj_dev->session_handle, cb_params);
}

static int __cam_synx_obj_release(int32_t row_idx)
{
	int rc;
	bool deregister_cb = false;
	uint32_t synx_hdl = 0;
	struct cam_synx_obj_row *row = NULL;

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	row = &g_cam_synx_obj_dev->rows[row_idx];
	synx_hdl = row->synx_obj;

	if (row->state == CAM_SYNX_OBJ_STATE_ACTIVE) {
		CAM_DBG(CAM_SYNX,
			"Unsignaled synx obj being released name: %s synx_obj:%d",
			row->name, row->synx_obj);

		if (row->cb_registered_for_sync) {
			if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask))
				cam_generic_fence_update_monitor_array(row_idx,
					&g_cam_synx_obj_dev->dev_lock,
					g_cam_synx_obj_dev->monitor_data,
					CAM_FENCE_OP_UNREGISTER_ON_SIGNAL);
			deregister_cb = true;
		}

		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask))
			cam_generic_fence_update_monitor_array(row_idx,
				&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
				CAM_FENCE_OP_SIGNAL);

		if (deregister_cb) {
			spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
			rc = __cam_synx_deregister_cb_util(synx_hdl, row);
			spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
			if (rc) {
				CAM_DBG(CAM_SYNX,
					"Failed to deregister cb for synx hdl: %u rc: %d",
					synx_hdl, rc);
				__cam_synx_obj_dump_monitor_array(row_idx);
			}
		}
	}

	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask)) {
		/* Update monitor entries & save data before row memset to 0 */
		cam_generic_fence_update_monitor_array(row_idx,
			&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
			CAM_FENCE_OP_DESTROY);

		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ_DUMP, &cam_sync_monitor_mask))
			__cam_synx_obj_dump_monitor_array(row_idx);
		__cam_synx_obj_save_previous_monitor_data(row_idx);
	}

	CAM_DBG(CAM_SYNX,
		"Releasing synx_obj: %d[%s] row_idx: %u", row->synx_obj, row->name, row_idx);

	/* deinit row */
	memset(row, 0, sizeof(struct cam_synx_obj_row));
	clear_bit(row_idx, g_cam_synx_obj_dev->bitmap);
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);

	return __cam_synx_release_hdl_util(synx_hdl);
}

static int __cam_synx_obj_find_free_idx(uint32_t *idx)
{
	int rc = 0;
	bool bit;

	do {
		*idx = find_first_zero_bit(g_cam_synx_obj_dev->bitmap, CAM_SYNX_MAX_OBJS);
		if (*idx >= CAM_SYNX_MAX_OBJS) {
			CAM_ERR(CAM_SYNC,
				"Error: Unable to create synx, no free index");
			rc = -ENOMEM;
			break;
		}

		bit = test_and_set_bit(*idx, g_cam_synx_obj_dev->bitmap);
	} while (bit);

	return rc;
}

static void __cam_synx_obj_init_row(uint32_t idx, const char *name,
	uint32_t synx_obj)
{
	struct cam_synx_obj_row *row;

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
	row = &g_cam_synx_obj_dev->rows[idx];
	row->synx_obj = synx_obj;
	row->state = CAM_SYNX_OBJ_STATE_ACTIVE;
	strscpy(row->name, name, CAM_SYNX_OBJ_NAME_LEN);
	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask)) {
		cam_generic_fence_update_monitor_array(idx,
			&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
			CAM_FENCE_OP_CREATE);
	}
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[idx]);
}

static int __cam_synx_obj_release_row(int32_t row_idx)
{
	if ((row_idx < 0) || (row_idx >= CAM_SYNX_MAX_OBJS)) {
		CAM_ERR(CAM_SYNX, "synx row idx: %d is invalid", row_idx);
		return -EINVAL;
	}

	return __cam_synx_obj_release(row_idx);
}

int cam_synx_obj_find_obj_in_table(uint32_t synx_obj, int32_t *idx)
{
	int i, rc = -EINVAL;
	struct cam_synx_obj_row *row = NULL;

	for (i = 0; i < CAM_SYNX_MAX_OBJS; i++) {
		spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
		row = &g_cam_synx_obj_dev->rows[i];
		if ((row->state != CAM_SYNX_OBJ_STATE_INVALID) &&
			(row->synx_obj == synx_obj)) {
			*idx = i;
			spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
			rc = 0;
			break;
		}
		spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[i]);
	}

	return rc;
}

static int __cam_synx_obj_release_obj(uint32_t synx_obj, int32_t *idx)
{
	if (cam_synx_obj_find_obj_in_table(synx_obj, idx)) {
		CAM_ERR(CAM_SYNX, "Failed to find synx obj: %d", synx_obj);
		return -EINVAL;
	}

	return __cam_synx_obj_release(*idx);
}

static int __cam_synx_obj_import(const char *name,
	struct synx_import_params *params, int32_t *row_idx)
{
	int rc = -1;
	uint32_t idx;

	if (__cam_synx_obj_find_free_idx(&idx))
		goto end;

	rc = __cam_synx_import_hdl_util(params);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Synx import failed for fence : %p",
			params->indv.fence);
		goto free_idx;
	}

	*row_idx = idx;
	__cam_synx_obj_init_row(idx, name, *params->indv.new_h_synx);

	CAM_DBG(CAM_SYNX, "Imported synx obj handle: %d[%s] row_idx: %u",
		*params->indv.new_h_synx, name, idx);

	return rc;

free_idx:
	clear_bit(idx, g_cam_synx_obj_dev->bitmap);
end:
	return rc;
}

static int __cam_synx_map_generic_flags_to_create(uint32_t generic_flags,
	struct synx_create_params *params)
{
	if (!params) {
		CAM_ERR(CAM_SYNX, "Create parameters missing");
		return -EINVAL;
	}

	/*
	 * Create Global Always - remove after userspace optimizes and
	 * determines when global Vs local is needed
	 */
	params->flags |= SYNX_CREATE_GLOBAL_FENCE;

	return 0;
}

static int __cam_synx_map_generic_flags_to_import(uint32_t generic_flags,
	struct synx_import_indv_params *params)
{
	if (!params) {
		CAM_ERR(CAM_SYNX, "Import parameters missing");
		return -EINVAL;
	}

	/*
	 * Create Global Always - remove after userspace optimizes and
	 * determines when global Vs local is needed
	 */
	params->flags |= SYNX_IMPORT_GLOBAL_FENCE;

	return 0;
}

int cam_synx_obj_create(const char *name, uint32_t flags, uint32_t *synx_obj,
	int32_t *row_idx)
{
	int rc = -1;
	uint32_t idx;
	struct synx_create_params params;

	if (__cam_synx_obj_find_free_idx(&idx))
		goto end;

	params.fence = NULL;
	params.name = name;
	params.flags = 0;
	params.h_synx = synx_obj;

	rc = __cam_synx_map_generic_flags_to_create(flags, &params);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to generate create flags");
		goto free_idx;
	}

	rc = __cam_synx_create_hdl_util(&params);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to create new synx handle rc: %d", rc);
		goto free_idx;
	}

	*row_idx = idx;
	__cam_synx_obj_init_row(idx, name, *synx_obj);

	CAM_DBG(CAM_SYNX, "Created synx obj handle: %d[%s] row_idx: %u",
		*synx_obj, name, idx);

	return rc;

free_idx:
	clear_bit(idx, g_cam_synx_obj_dev->bitmap);
end:
	return rc;
}

int cam_synx_obj_import_dma_fence(const char *name, uint32_t flags, void *fence,
	uint32_t *synx_obj, int32_t *row_idx)
{
	struct synx_import_params params;

	if (!fence) {
		CAM_ERR(CAM_SYNX,
			"Importing DMA fence failed - fence pointer is NULL");
		return -EINVAL;
	}

	params.indv.flags = 0;
	params.indv.fence = fence;
	params.indv.new_h_synx = synx_obj;
	params.type = SYNX_IMPORT_INDV_PARAMS;
	params.indv.flags |= SYNX_IMPORT_DMA_FENCE;

	if (__cam_synx_map_generic_flags_to_import(flags, &params.indv)) {
		CAM_ERR(CAM_SYNX,
			"Importing DMA fence failed - invalid synx import flags");
		return -EINVAL;
	}

	return __cam_synx_obj_import(name, &params, row_idx);
}

int cam_synx_obj_internal_signal(int32_t row_idx,
	struct cam_synx_obj_signal *signal_synx_obj)
{
	int rc;
	uint32_t signal_status;
	bool deregister_cb = false;
	struct cam_synx_obj_row *row = NULL;

	if ((row_idx < 0) || (row_idx >= CAM_SYNX_MAX_OBJS)) {
		CAM_ERR(CAM_SYNX, "synx obj row idx: %d is invalid",
			row_idx);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	row = &g_cam_synx_obj_dev->rows[row_idx];

	/* Ensures sync obj cb is not invoked */
	row->sync_signal_synx = true;

	if (row->state != CAM_SYNX_OBJ_STATE_ACTIVE) {
		CAM_ERR(CAM_SYNX, "synx obj: %u not in right state: %d to signal",
			signal_synx_obj->synx_obj, row->state);
		rc = -EINVAL;
		goto monitor_dump;
	}

	if (row->synx_obj != signal_synx_obj->synx_obj) {
		CAM_WARN(CAM_SYNX,
			"Trying to signal synx obj: %u in row: %u having a different synx obj: %u",
			signal_synx_obj->synx_obj, row_idx, row->synx_obj);
		rc = 0;
		goto monitor_dump;
	}

	rc = __cam_synx_obj_map_sync_status_util(signal_synx_obj->status, &signal_status);
	if (rc) {
		CAM_WARN(CAM_SYNX,
			"Signaling undefined status: %d for synx obj: %d",
			signal_synx_obj->status, signal_synx_obj->synx_obj);
	}

	if (row->cb_registered_for_sync) {
		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask))
			cam_generic_fence_update_monitor_array(row_idx,
				&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
				CAM_FENCE_OP_UNREGISTER_ON_SIGNAL);
		deregister_cb = true;
	}

	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask))
		cam_generic_fence_update_monitor_array(row_idx,
			&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
			CAM_FENCE_OP_SIGNAL);

	row->state = CAM_SYNX_OBJ_STATE_SIGNALED;
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);

	if (deregister_cb) {
		rc = __cam_synx_deregister_cb_util(signal_synx_obj->synx_obj, row);
		if (rc) {
			spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
			CAM_ERR(CAM_SYNX, "Failed to deregister cb for synx: %u rc: %d",
				signal_synx_obj->synx_obj, rc);
			goto monitor_dump;
		}
	}

	rc = __cam_synx_signal_util(signal_synx_obj->synx_obj, signal_status);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to signal synx hdl: %u with status: %u rc: %d",
			signal_synx_obj->synx_obj, signal_status, rc);
		goto end;
	}

	CAM_DBG(CAM_SYNX, "synx obj: %d signaled with status: %d rc: %d",
		signal_synx_obj->synx_obj, signal_status, rc);

	return rc;

monitor_dump:
	__cam_synx_obj_dump_monitor_array(row_idx);
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
end:
	return rc;
}

int cam_synx_obj_release(struct cam_synx_obj_release_params *release_params)
{
	int rc;
	int32_t idx = -1;

	if (release_params->use_row_idx) {
		rc = __cam_synx_obj_release_row(release_params->u.synx_row_idx);
		if (rc < 0)
			__cam_synx_obj_dump_monitor_array(release_params->u.synx_row_idx);
	} else {
		rc = __cam_synx_obj_release_obj(release_params->u.synx_obj, &idx);
		if ((rc < 0) && (idx >= 0))
			__cam_synx_obj_dump_monitor_array(idx);
	}
	return rc;
}

int cam_synx_obj_signal_obj(struct cam_synx_obj_signal *signal_synx_obj)
{
	int rc;
	uint32_t idx;

	rc = cam_synx_obj_find_obj_in_table(signal_synx_obj->synx_obj, &idx);
	if (rc) {
		CAM_ERR(CAM_SYNX, "Failed to find synx obj: %u", signal_synx_obj->synx_obj);
		return -EINVAL;
	}

	return cam_synx_obj_internal_signal(idx, signal_synx_obj);
}

int cam_synx_obj_register_cb(int32_t *sync_obj, int32_t row_idx,
	cam_sync_callback_for_synx_obj sync_cb)
{
	int rc = 0;
	uint32_t synx_obj = 0;
	struct cam_synx_obj_row *row = NULL;
	struct synx_callback_params cb_params;

	if (!sync_obj || !sync_cb) {
		CAM_ERR(CAM_SYNX, "Invalid args sync_obj: %p sync_cb: %p",
			sync_obj, sync_cb);
		return -EINVAL;
	}

	if ((row_idx < 0) || (row_idx >= CAM_SYNX_MAX_OBJS)) {
		CAM_ERR(CAM_SYNX, "synx obj idx: %d is invalid",
			row_idx);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	row = &g_cam_synx_obj_dev->rows[row_idx];
	synx_obj = row->synx_obj;

	if (row->state != CAM_SYNX_OBJ_STATE_ACTIVE) {
		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ,
			&cam_sync_monitor_mask))
			cam_generic_fence_update_monitor_array(row_idx,
				&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
				CAM_FENCE_OP_SKIP_REGISTER_CB);
		CAM_ERR(CAM_SYNX,
			"synx obj at idx: %d handle: %d is not active, current state: %d",
			row_idx, row->synx_obj, row->state);
		rc = -EINVAL;
		goto monitor_dump;
	}

	/**
	 * If the cb is already registered, return
	 */
	if (row->cb_registered_for_sync) {
		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ,
			&cam_sync_monitor_mask))
			cam_generic_fence_update_monitor_array(row_idx,
				&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
				CAM_FENCE_OP_ALREADY_REGISTERED_CB);
		CAM_WARN(CAM_SYNX,
			"synx obj at idx: %d handle: %d has already registered a cb for sync: %d",
			row_idx, row->synx_obj, row->sync_obj);
		goto monitor_dump;
	}

	row->sync_cb = sync_cb;
	row->sync_obj = *sync_obj;
	row->cb_registered_for_sync = true;

	cb_params.userdata = row;
	cb_params.cancel_cb_func = NULL;
	cb_params.h_synx = synx_obj;
	cb_params.timeout_ms = SYNX_NO_TIMEOUT;
	cb_params.cb_func = __cam_synx_obj_signal_cb;

	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask))
		cam_generic_fence_update_monitor_array(row_idx,
			&g_cam_synx_obj_dev->dev_lock, g_cam_synx_obj_dev->monitor_data,
			CAM_FENCE_OP_REGISTER_CB);

	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);

	rc = __cam_synx_register_cb_util(&cb_params);
	if (rc) {
		CAM_ERR(CAM_SYNX,
			"Failed to register cb for synx obj: %d rc: %d", synx_obj, rc);
		return rc;
	}

	CAM_DBG(CAM_SYNX,
		"CB successfully registered for synx obj: %d for sync_obj: %d",
		synx_obj, *sync_obj);

	return rc;

monitor_dump:
	__cam_synx_obj_dump_monitor_array(row_idx);
	spin_unlock_bh(&g_cam_synx_obj_dev->row_spinlocks[row_idx]);
	return rc;
}

int cam_synx_core_recovery(
	enum cam_sync_synx_supported_cores cam_core_id)
{
	int rc;
	enum synx_client_id client_id = SYNX_CLIENT_MAX;

	switch (cam_core_id) {
	case CAM_ICP_0_SYNX_CORE:
		client_id = SYNX_CLIENT_ICP_CTX0;
		break;
	default:
		rc = -EINVAL;
		goto err;
	}

	rc = synx_recover(client_id);
	if (rc)
		goto err;

	CAM_DBG(CAM_SYNX, "Synx recovery for synx_client: %d[%d] success",
		client_id, cam_core_id);

	return rc;

err:
	CAM_ERR(CAM_SYNX, "Failed to recover for synx_client: %d rc: %d",
			client_id, rc);
	return rc;
}

int __cam_synx_init_session(void)
{
	struct synx_queue_desc queue_desc;
	struct synx_initialization_params params;

	params.name = cam_synx_session_name;
	params.ptr = &queue_desc;
	params.flags = SYNX_INIT_MAX;
	params.id = SYNX_CLIENT_NATIVE;
	g_cam_synx_obj_dev->session_handle = synx_initialize(&params);

	if (!g_cam_synx_obj_dev->session_handle) {
		CAM_ERR(CAM_SYNX, "Synx session initialization failed");
		return -EINVAL;
	}

	CAM_DBG(CAM_SYNX, "Synx session initialized: %p",
		g_cam_synx_obj_dev->session_handle);

	return 0;
}

void cam_synx_obj_open(void)
{
	mutex_lock(&g_cam_synx_obj_dev->dev_lock);
	if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ, &cam_sync_monitor_mask)) {
		g_cam_synx_obj_dev->monitor_data = kzalloc(
			sizeof(struct cam_generic_fence_monitor_data *) *
			CAM_SYNX_TABLE_SZ, GFP_KERNEL);
		if (!g_cam_synx_obj_dev->monitor_data) {
			CAM_WARN(CAM_DMA_FENCE, "Failed to allocate memory %d",
				sizeof(struct cam_generic_fence_monitor_data *) *
				CAM_SYNX_TABLE_SZ);
		}
	}
	mutex_unlock(&g_cam_synx_obj_dev->dev_lock);
}

void cam_synx_obj_close(void)
{
	int i;
	struct cam_synx_obj_row *row = NULL;

	mutex_lock(&g_cam_synx_obj_dev->dev_lock);
	for (i = 0; i < CAM_SYNX_MAX_OBJS; i++) {
		row = &g_cam_synx_obj_dev->rows[i];
		if (row->state == CAM_SYNX_OBJ_STATE_INVALID)
			continue;

		CAM_DBG(CAM_SYNX, "Releasing synx_obj: %d[%s]",
			row->synx_obj, row->name);

		/* If registered for cb, remove cb */
		if (row->cb_registered_for_sync) {
			if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ,
				&cam_sync_monitor_mask))
				cam_generic_fence_update_monitor_array(i,
					&g_cam_synx_obj_dev->dev_lock,
					g_cam_synx_obj_dev->monitor_data,
					CAM_FENCE_OP_UNREGISTER_CB);

			__cam_synx_deregister_cb_util(row->synx_obj, row);
		}

		/* Signal and release the synx obj */
		if (row->state != CAM_SYNX_OBJ_STATE_SIGNALED) {
			if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ,
				&cam_sync_monitor_mask))
				cam_generic_fence_update_monitor_array(i,
					&g_cam_synx_obj_dev->dev_lock,
					g_cam_synx_obj_dev->monitor_data,
					CAM_FENCE_OP_SIGNAL);

			__cam_synx_signal_util(row->synx_obj, SYNX_STATE_SIGNALED_CANCEL);
		}

		if (test_bit(CAM_GENERIC_FENCE_TYPE_SYNX_OBJ,
			&cam_sync_monitor_mask))
			cam_generic_fence_update_monitor_array(i,
				&g_cam_synx_obj_dev->dev_lock,
				g_cam_synx_obj_dev->monitor_data,
				CAM_FENCE_OP_DESTROY);

		__cam_synx_release_hdl_util(row->synx_obj);
		memset(row, 0, sizeof(struct cam_synx_obj_row));
		clear_bit(i, g_cam_synx_obj_dev->bitmap);
	}

	if (g_cam_synx_obj_dev->monitor_data) {
		for (i = 0; i < CAM_SYNX_TABLE_SZ; i++) {
			kfree(g_cam_synx_obj_dev->monitor_data[i]);
			g_cam_synx_obj_dev->monitor_data[i] = NULL;
		}
	}
	kfree(g_cam_synx_obj_dev->monitor_data);
	g_cam_synx_obj_dev->monitor_data = NULL;

	mutex_unlock(&g_cam_synx_obj_dev->dev_lock);
	CAM_DBG(CAM_SYNX, "Close on Camera SYNX driver");
}

int cam_synx_obj_driver_init(void)
{
	int i;

	g_cam_synx_obj_dev = kzalloc(sizeof(struct cam_synx_obj_device), GFP_KERNEL);
	if (!g_cam_synx_obj_dev)
		return -ENOMEM;

	if (__cam_synx_init_session())
		goto deinit_driver;

	mutex_init(&g_cam_synx_obj_dev->dev_lock);
	for (i = 0; i < CAM_SYNX_MAX_OBJS; i++)
		spin_lock_init(&g_cam_synx_obj_dev->row_spinlocks[i]);

	memset(&g_cam_synx_obj_dev->rows, 0, sizeof(g_cam_synx_obj_dev->rows));
	memset(&g_cam_synx_obj_dev->bitmap, 0, sizeof(g_cam_synx_obj_dev->bitmap));
	bitmap_zero(g_cam_synx_obj_dev->bitmap, CAM_SYNX_MAX_OBJS);

	/* zero will be considered an invalid slot */
	set_bit(0, g_cam_synx_obj_dev->bitmap);

	CAM_DBG(CAM_SYNX, "Camera synx obj driver initialized");
	return 0;

deinit_driver:
	CAM_ERR(CAM_SYNX, "Camera synx obj driver initialization failed");
	kfree(g_cam_synx_obj_dev);
	g_cam_synx_obj_dev = NULL;
	return -EINVAL;
}

void cam_synx_obj_driver_deinit(void)
{
	int rc;

	if (g_cam_synx_obj_dev->session_handle) {
		rc = synx_uninitialize(g_cam_synx_obj_dev->session_handle);
		if (rc) {
			CAM_ERR(CAM_SYNX,
				"Synx failed to uninitialize session: %p, rc: %d",
				g_cam_synx_obj_dev->session_handle, rc);
		}
	}

	kfree(g_cam_synx_obj_dev);
	g_cam_synx_obj_dev = NULL;
	CAM_DBG(CAM_SYNX, "Camera synx obj driver deinitialized");
}
