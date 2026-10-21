// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include "cam_sync_dma_fence.h"

/**
 * struct cam_dma_fence_row - DMA fence row
 */
struct cam_dma_fence_row {
	char                            name[CAM_DMA_FENCE_NAME_LEN];
	struct dma_fence               *fence;
	int32_t                         fd;
	enum cam_dma_fence_state        state;
	struct dma_fence_cb             fence_cb;
	int32_t                         sync_obj;
	cam_sync_callback_for_dma_fence sync_cb;
	bool                            cb_registered_for_sync;
	bool                            ext_dma_fence;
	bool                            sync_signal_dma;
};

/**
 * struct cam_dma_fence_device - DMA fence device
 */
struct cam_dma_fence_device {
	uint64_t dma_fence_context;
	struct cam_dma_fence_row rows[CAM_DMA_FENCE_MAX_FENCES];
	spinlock_t row_spinlocks[CAM_DMA_FENCE_MAX_FENCES];
	struct mutex dev_lock;
	DECLARE_BITMAP(bitmap, CAM_DMA_FENCE_MAX_FENCES);
};

static atomic64_t g_cam_dma_fence_seq_no;
static struct cam_dma_fence_device *g_cam_dma_fence_dev;

bool __cam_dma_fence_enable_signaling(
	struct dma_fence *fence)
{
	return true;
}

const char *__cam_dma_fence_get_driver_name(
	struct dma_fence *fence)
{
	return "Camera DMA fence driver";
}

void __cam_dma_fence_free(struct dma_fence *fence)
{
	CAM_DBG(CAM_DMA_FENCE,
		"Free memory for dma fence seqno: %llu", fence->seqno);
	kfree(fence->lock);
	kfree(fence);
}

static struct dma_fence_ops cam_sync_dma_fence_ops = {
	.enable_signaling = __cam_dma_fence_enable_signaling,
	.get_driver_name = __cam_dma_fence_get_driver_name,
	.get_timeline_name = __cam_dma_fence_get_driver_name,
	.release = __cam_dma_fence_free,
};

static void __cam_dma_fence_print_table(void)
{
	int i;
	struct cam_dma_fence_row *row;
	struct dma_fence *fence;

	for (i = 0; i < CAM_DMA_FENCE_MAX_FENCES; i++) {
		spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[i]);
		row = &g_cam_dma_fence_dev->rows[i];
		fence = row->fence;
		CAM_INFO(CAM_DMA_FENCE,
			"Idx: %d seqno: %llu name: %s state: %d",
			i, fence->seqno, row->name, row->state);
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[i]);
	}
}

static int __cam_dma_fence_find_free_idx(uint32_t *idx)
{
	int rc = 0;

	*idx = find_first_zero_bit(g_cam_dma_fence_dev->bitmap, CAM_DMA_FENCE_MAX_FENCES);
	if (*idx < CAM_DMA_FENCE_MAX_FENCES)
		set_bit(*idx, g_cam_dma_fence_dev->bitmap);
	else
		rc = -ENOMEM;

	if (rc) {
		CAM_ERR(CAM_DMA_FENCE, "No free idx, printing dma fence table......");
		__cam_dma_fence_print_table();
	}

	return rc;
}

static struct dma_fence *__cam_dma_fence_find_fence_in_table(
	int32_t fd, int32_t *idx)
{
	int i;
	struct dma_fence *fence = NULL;
	struct cam_dma_fence_row *row = NULL;

	for (i = 0; i < CAM_DMA_FENCE_MAX_FENCES; i++) {
		spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[i]);

		row = &g_cam_dma_fence_dev->rows[i];
		if ((row->state != CAM_DMA_FENCE_STATE_INVALID) && (row->fd == fd)) {
			*idx = i;
			fence = row->fence;
			spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[i]);
			break;
		}
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[i]);
	}

	return fence;
}

static void __cam_dma_fence_init_row(const char *name,
	struct dma_fence *dma_fence, int32_t fd, uint32_t idx,
	bool ext_dma_fence)
{
	struct cam_dma_fence_row *row;

	spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[idx]);
	row = &g_cam_dma_fence_dev->rows[idx];
	memset(row, 0, sizeof(*row));
	row->fence = dma_fence;
	row->fd = fd;
	row->state = CAM_DMA_FENCE_STATE_ACTIVE;
	row->ext_dma_fence = ext_dma_fence;
	strscpy(row->name, name, CAM_DMA_FENCE_NAME_LEN);
	spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[idx]);
}

void __cam_dma_fence_signal_cb(
	struct dma_fence *fence, struct dma_fence_cb *cb)
{
	struct cam_dma_fence_signal_sync_obj signal_sync_obj;
	struct cam_dma_fence_row *dma_fence_row =
		container_of(cb, struct cam_dma_fence_row, fence_cb);

	if (dma_fence_row->state == CAM_DMA_FENCE_STATE_INVALID) {
		CAM_ERR(CAM_DMA_FENCE, "dma fence seqno: %llu is in invalid state: %d",
			fence->seqno, dma_fence_row->state);
		return;
	}

	/* If this dma fence is signaled by sync obj, skip cb */
	if (dma_fence_row->sync_signal_dma)
		return;

	CAM_DBG(CAM_DMA_FENCE, "dma fence seqno: %llu fd: %d signaled, signal sync obj: %d",
		fence->seqno, dma_fence_row->fd, dma_fence_row->sync_obj);
	if ((dma_fence_row->cb_registered_for_sync) && (dma_fence_row->sync_cb)) {
		signal_sync_obj.fd = dma_fence_row->fd;

		/*
		 * Signal is invoked with the fence lock held,
		 * lock not needed to query status
		 */
		signal_sync_obj.status = dma_fence_get_status_locked(fence);
		dma_fence_row->state = CAM_DMA_FENCE_STATE_SIGNALED;
		dma_fence_row->sync_cb(dma_fence_row->sync_obj, &signal_sync_obj);
	}
}

int cam_dma_fence_get_put_ref(
	bool get_or_put, int32_t dma_fence_row_idx)
{
	struct dma_fence *dma_fence;
	struct cam_dma_fence_row *row;

	if ((dma_fence_row_idx < 0) ||
		(dma_fence_row_idx >= CAM_DMA_FENCE_MAX_FENCES)) {
		CAM_ERR(CAM_DMA_FENCE, "dma fence idx: %d is invalid",
			dma_fence_row_idx);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);
	row = &g_cam_dma_fence_dev->rows[dma_fence_row_idx];

	if (row->state == CAM_DMA_FENCE_STATE_INVALID) {
		CAM_ERR(CAM_DMA_FENCE,
			"dma fence at idx: %d is in invalid state: %d",
			dma_fence_row_idx, row->state);
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);
		return -EINVAL;
	}

	dma_fence = row->fence;

	if (get_or_put)
		dma_fence_get(dma_fence);
	else
		dma_fence_put(dma_fence);

	spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);

	CAM_DBG(CAM_DMA_FENCE, "Refcnt: %u after %s for dma fence with seqno: %llu",
		kref_read(&dma_fence->refcount), (get_or_put ? "getref" : "putref"),
		dma_fence->seqno);

	return 0;
}

static struct dma_fence *cam_dma_fence_get_fence_from_sync_file(
	int32_t fd, int32_t *dma_fence_row_idx)
{
	uint32_t idx;
	struct dma_fence *dma_fence = NULL;

	dma_fence = sync_file_get_fence(fd);
	if (IS_ERR_OR_NULL(dma_fence)) {
		CAM_ERR(CAM_DMA_FENCE, "Invalid fd: %d no dma fence found", fd);
		return ERR_PTR(-EINVAL);
	}

	if (__cam_dma_fence_find_free_idx(&idx)) {
		CAM_ERR(CAM_DMA_FENCE, "No free idx");
		goto end;
	}

	__cam_dma_fence_init_row(dma_fence->ops->get_driver_name(dma_fence),
		dma_fence, fd, idx, true);
	*dma_fence_row_idx = idx;
	CAM_DBG(CAM_DMA_FENCE,
		"External dma fence with fd: %d seqno: %llu ref_cnt: %u updated in tbl",
		fd, dma_fence->seqno, kref_read(&dma_fence->refcount));

	return dma_fence;

end:
	dma_fence_put(dma_fence);
	return NULL;
}

struct dma_fence *cam_dma_fence_get_fence_from_fd(
	int32_t fd, int32_t *dma_fence_row_idx)
{
	struct dma_fence *dma_fence = NULL;

	dma_fence = __cam_dma_fence_find_fence_in_table(fd, dma_fence_row_idx);
	if (IS_ERR_OR_NULL(dma_fence)) {
		CAM_WARN(CAM_DMA_FENCE,
			"dma fence with fd: %d is an external fence, querying sync file",
			fd);
		return cam_dma_fence_get_fence_from_sync_file(fd, dma_fence_row_idx);
	}

	dma_fence_get(dma_fence);

	CAM_DBG(CAM_DMA_FENCE, "dma fence found for fd: %d with seqno: %llu ref_cnt: %u",
		fd, dma_fence->seqno, kref_read(&dma_fence->refcount));

	return dma_fence;
}

int cam_dma_fence_register_cb(int32_t *sync_obj, int32_t *dma_fence_idx,
	cam_sync_callback_for_dma_fence sync_cb)
{
	int rc = 0;
	int dma_fence_row_idx = 0;
	struct cam_dma_fence_row *row = NULL;
	struct dma_fence *dma_fence = NULL;

	if (!sync_obj || !dma_fence_idx || !sync_cb) {
		CAM_ERR(CAM_DMA_FENCE,
			"Invalid args sync_obj: %p dma_fence_idx: %p sync_cb: %p",
			sync_obj, dma_fence_idx, sync_cb);
		return -EINVAL;
	}

	dma_fence_row_idx = *dma_fence_idx;
	if ((dma_fence_row_idx < 0) ||
		(dma_fence_row_idx >= CAM_DMA_FENCE_MAX_FENCES)) {
		CAM_ERR(CAM_DMA_FENCE, "dma fence idx: %d is invalid",
			dma_fence_row_idx);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);
	row = &g_cam_dma_fence_dev->rows[dma_fence_row_idx];
	dma_fence = row->fence;

	if (row->state != CAM_DMA_FENCE_STATE_ACTIVE) {
		CAM_ERR(CAM_DMA_FENCE,
			"dma fence at idx: %d fd: %d seqno: %llu is not active, current state: %d",
			dma_fence_row_idx, row->fd, dma_fence->seqno, row->state);
		rc = -EINVAL;
		goto end;
	}

	/**
	 * If the cb is already registered, return
	 * If a fd is closed by userspace without releasing the dma fence, it is
	 * possible that same fd is returned to a new fence.
	 */
	if (row->cb_registered_for_sync) {
		CAM_WARN(CAM_DMA_FENCE,
			"dma fence at idx: %d fd: %d seqno: %llu has already registered a cb for sync: %d - same fd for 2 fences?",
			dma_fence_row_idx, row->fd, dma_fence->seqno, row->sync_obj);
		goto end;
	}

	rc = dma_fence_add_callback(row->fence, &row->fence_cb,
		__cam_dma_fence_signal_cb);
	if (rc) {
		CAM_ERR(CAM_DMA_FENCE,
			"Failed to register cb for dma fence fd: %d seqno: %llu rc: %d",
			row->fd, dma_fence->seqno, rc);
		goto end;
	}

	row->cb_registered_for_sync = true;
	row->sync_obj = *sync_obj;
	row->sync_cb = sync_cb;

	CAM_DBG(CAM_DMA_FENCE,
		"CB successfully registered for dma fence fd: %d seqno: %llu for sync_obj: %d",
		row->fd, dma_fence->seqno, *sync_obj);

end:
	spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);
	return rc;
}

static int __cam_dma_fence_signal_fence(
	struct dma_fence *dma_fence,
	int32_t status)
{
	bool fence_signaled = false;

	fence_signaled = dma_fence_is_signaled(dma_fence);
	if (fence_signaled) {
		CAM_WARN(CAM_DMA_FENCE,
			"dma fence seqno: %llu is already signaled",
			dma_fence->seqno);
		return 0;
	}

	if (status)
		dma_fence_set_error(dma_fence, status);

	return dma_fence_signal(dma_fence);
}

int cam_dma_fence_internal_signal(
	int32_t dma_fence_row_idx,
	struct cam_dma_fence_signal *signal_dma_fence)
{
	int rc = 0;
	struct dma_fence *dma_fence = NULL;
	struct cam_dma_fence_row *row = NULL;

	if ((dma_fence_row_idx < 0) ||
		(dma_fence_row_idx >= CAM_DMA_FENCE_MAX_FENCES)) {
		CAM_ERR(CAM_DMA_FENCE, "dma fence idx: %d is invalid",
			dma_fence_row_idx);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);
	row = &g_cam_dma_fence_dev->rows[dma_fence_row_idx];

	/* Ensures sync obj cb is not invoked */
	row->sync_signal_dma = true;
	dma_fence = row->fence;

	if (IS_ERR_OR_NULL(dma_fence)) {
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);
		CAM_ERR(CAM_DMA_FENCE, "DMA fence in row: %d is invalid",
			dma_fence_row_idx);
		return -EINVAL;
	}

	if (row->state == CAM_DMA_FENCE_STATE_SIGNALED) {
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);
		CAM_WARN(CAM_DMA_FENCE,
			"dma fence fd: %d[seqno: %llu] already in signaled state",
			signal_dma_fence->dma_fence_fd, dma_fence->seqno);
		return 0;
	}

	rc = __cam_dma_fence_signal_fence(dma_fence, signal_dma_fence->status);
	if (rc)
		CAM_WARN(CAM_DMA_FENCE,
			"dma fence seqno: %llu fd: %d already signaled rc: %d",
			dma_fence->seqno, row->fd, rc);

	row->state = CAM_DMA_FENCE_STATE_SIGNALED;
	spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_fence_row_idx]);

	CAM_DBG(CAM_DMA_FENCE,
		"dma fence fd: %d[seqno: %llu] signaled with status: %d rc: %d",
		signal_dma_fence->dma_fence_fd, dma_fence->seqno,
		signal_dma_fence->status, rc);

	return rc;
}

int cam_dma_fence_signal_fd(struct cam_dma_fence_signal *signal_dma_fence)
{
	int rc = 0;
	uint32_t idx;
	struct dma_fence *dma_fence = NULL;
	struct cam_dma_fence_row *row = NULL;

	dma_fence = __cam_dma_fence_find_fence_in_table(
		signal_dma_fence->dma_fence_fd, &idx);

	if (IS_ERR_OR_NULL(dma_fence)) {
		CAM_ERR(CAM_DMA_FENCE, "Failed to find dma fence for fd: %d",
			signal_dma_fence->dma_fence_fd);
		return -EINVAL;
	}

	spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[idx]);
	row = &g_cam_dma_fence_dev->rows[idx];
	/*
	 * Check for invalid state again, there could be a contention
	 * between signal and release
	 */
	if (row->state == CAM_DMA_FENCE_STATE_INVALID) {
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[idx]);
		CAM_ERR(CAM_DMA_FENCE,
			"dma fence fd: %d is invalid row_idx: %u, failed to signal",
			signal_dma_fence->dma_fence_fd, idx);
		return -EINVAL;
	}

	if (row->state == CAM_DMA_FENCE_STATE_SIGNALED) {
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[idx]);
		CAM_WARN(CAM_DMA_FENCE,
			"dma fence fd: %d[seqno: %llu] already in signaled state",
			signal_dma_fence->dma_fence_fd, dma_fence->seqno);
		return 0;
	}

	rc = __cam_dma_fence_signal_fence(dma_fence, signal_dma_fence->status);
	if (rc)
		CAM_WARN(CAM_DMA_FENCE,
			"dma fence seqno: %llu fd: %d already signaled rc: %d",
			dma_fence->seqno, row->fd, rc);

	row->state = CAM_DMA_FENCE_STATE_SIGNALED;
	spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[idx]);

	CAM_DBG(CAM_DMA_FENCE,
		"dma fence fd: %d[seqno: %llu] signaled with status: %d rc: %d",
		signal_dma_fence->dma_fence_fd, dma_fence->seqno,
		signal_dma_fence->status, rc);

	return rc;
}

static int __cam_dma_fence_get_fd(int32_t *row_idx,
	const char *name)
{
	int fd = -1;
	uint32_t idx;
	struct dma_fence *dma_fence = NULL;
	spinlock_t       *dma_fence_lock = NULL;
	struct sync_file *sync_file = NULL;

	if (__cam_dma_fence_find_free_idx(&idx))
		goto end;

	dma_fence_lock = kzalloc(sizeof(spinlock_t), GFP_KERNEL);
	if (!dma_fence_lock)
		goto free_idx;

	dma_fence = kzalloc(sizeof(struct dma_fence), GFP_KERNEL);
	if (!dma_fence) {
		kfree(dma_fence_lock);
		goto free_idx;
	}

	spin_lock_init(dma_fence_lock);
	dma_fence_init(dma_fence, &cam_sync_dma_fence_ops, dma_fence_lock,
		g_cam_dma_fence_dev->dma_fence_context,
		atomic64_inc_return(&g_cam_dma_fence_seq_no));
	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0) {
		CAM_ERR(CAM_DMA_FENCE, "failed to get a unused fd: %d", fd);
		dma_fence_put(dma_fence);
		goto free_idx;
	}

	sync_file = sync_file_create(dma_fence);
	if (!sync_file) {
		put_unused_fd(fd);
		fd = -1;
		dma_fence_put(dma_fence);
		goto free_idx;
	}

	fd_install(fd, sync_file->file);

	*row_idx = idx;
	__cam_dma_fence_init_row(name, dma_fence, fd, idx, false);

	CAM_DBG(CAM_DMA_FENCE, "Created dma fence fd: %d[%s] seqno: %llu row_idx: %u ref_cnt: %u",
		fd, name, dma_fence->seqno, idx, kref_read(&dma_fence->refcount));

	return fd;

free_idx:
	clear_bit(idx, g_cam_dma_fence_dev->bitmap);
end:
	return fd;
}

int cam_dma_fence_create_fd(
	int32_t *dma_fence_fd, int32_t *dma_fence_row_idx, const char *name)
{
	int fd = -1, rc = 0;

	if (!dma_fence_fd || !dma_fence_row_idx) {
		CAM_ERR(CAM_DMA_FENCE, "Invalid args fd: %pK dma_fence_row_idx: %pK",
			dma_fence_fd, dma_fence_row_idx);
		return -EINVAL;
	}

	fd = __cam_dma_fence_get_fd(dma_fence_row_idx, name);
	if (fd < 0) {
		rc = -EBADFD;
		goto end;
	}

	*dma_fence_fd = fd;

end:
	return rc;
}

static int __cam_dma_fence_release(int32_t dma_row_idx)
{
	struct dma_fence *dma_fence = NULL;
	struct cam_dma_fence_row *row = NULL;

	spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_row_idx]);
	row = &g_cam_dma_fence_dev->rows[dma_row_idx];
	dma_fence = row->fence;

	if (row->state == CAM_DMA_FENCE_STATE_INVALID) {
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_row_idx]);
		CAM_ERR(CAM_DMA_FENCE, "Invalid row index: %u, state: %u",
			dma_row_idx, row->state);
		return -EINVAL;
	}

	if (row->state == CAM_DMA_FENCE_STATE_ACTIVE) {
		CAM_WARN(CAM_DMA_FENCE,
			"Unsignaled fence being released name: %s seqno: %llu fd:%d",
			row->name, dma_fence->seqno, row->fd);
		__cam_dma_fence_signal_fence(dma_fence, -ECANCELED);
	}

	CAM_DBG(CAM_DMA_FENCE,
		"Releasing dma fence with fd: %d[%s] row_idx: %u current ref_cnt: %u",
		row->fd, row->name, dma_row_idx, kref_read(&dma_fence->refcount));

	/* putref on dma fence */
	dma_fence_put(dma_fence);

	/* deinit row */
	memset(row, 0, sizeof(struct cam_dma_fence_row));
	clear_bit(dma_row_idx, g_cam_dma_fence_dev->bitmap);
	spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[dma_row_idx]);
	return 0;
}

static int __cam_dma_fence_release_fd(int fd)
{
	int32_t idx;
	struct dma_fence *dma_fence = NULL;

	dma_fence = __cam_dma_fence_find_fence_in_table(fd, &idx);
	if (IS_ERR_OR_NULL(dma_fence)) {
		CAM_ERR(CAM_DMA_FENCE, "Failed to find dma fence for fd: %d", fd);
		return -EINVAL;
	}

	return __cam_dma_fence_release(idx);
}

static int __cam_dma_fence_release_row(
	int32_t dma_fence_row_idx)
{
	if ((dma_fence_row_idx < 0) ||
		(dma_fence_row_idx >= CAM_DMA_FENCE_MAX_FENCES)) {
		CAM_ERR(CAM_DMA_FENCE, "dma fence idx: %d is invalid",
			dma_fence_row_idx);
		return -EINVAL;
	}

	return __cam_dma_fence_release(dma_fence_row_idx);
}

int cam_dma_fence_release(
	struct cam_dma_fence_release_params *release_params)
{
	if (release_params->use_row_idx)
		return __cam_dma_fence_release_row(release_params->u.dma_row_idx);
	else
		return __cam_dma_fence_release_fd(release_params->u.dma_fence_fd);
}

void cam_dma_fence_close(void)
{
	int i;
	struct cam_dma_fence_row *row = NULL;

	mutex_lock(&g_cam_dma_fence_dev->dev_lock);
	for (i = 0; i < CAM_DMA_FENCE_MAX_FENCES; i++) {
		spin_lock_bh(&g_cam_dma_fence_dev->row_spinlocks[i]);

		row = &g_cam_dma_fence_dev->rows[i];
		if (row->state != CAM_DMA_FENCE_STATE_INVALID) {
			CAM_DBG(CAM_DMA_FENCE,
				"Releasing dma fence seqno: %llu associated with fd: %d[%s] ref_cnt: %u",
				row->fence->seqno, row->fd, row->name,
				kref_read(&row->fence->refcount));

			/* If registered for cb, remove cb */
			if (row->cb_registered_for_sync)
				dma_fence_remove_callback(row->fence, &row->fence_cb);

			/* Signal and put if the dma fence is created from camera */
			if (!row->ext_dma_fence) {
				if (row->state != CAM_DMA_FENCE_STATE_SIGNALED)
					__cam_dma_fence_signal_fence(row->fence, -EADV);
				dma_fence_put(row->fence);
			}

			memset(row, 0, sizeof(struct cam_dma_fence_row));
			clear_bit(i, g_cam_dma_fence_dev->bitmap);
		}
		spin_unlock_bh(&g_cam_dma_fence_dev->row_spinlocks[i]);
	}

	mutex_unlock(&g_cam_dma_fence_dev->dev_lock);
	CAM_DBG(CAM_DMA_FENCE, "Close on Camera DMA fence driver");
}

void cam_dma_fence_open(void)
{
	mutex_lock(&g_cam_dma_fence_dev->dev_lock);

	/* DMA fence seqno reset */
	atomic64_set(&g_cam_dma_fence_seq_no, 0);
	mutex_unlock(&g_cam_dma_fence_dev->dev_lock);
	CAM_DBG(CAM_DMA_FENCE, "Camera DMA fence driver opened");
}

int cam_dma_fence_driver_init(void)
{
	int i;

	g_cam_dma_fence_dev = kzalloc(sizeof(struct cam_dma_fence_device), GFP_KERNEL);
	if (!g_cam_dma_fence_dev)
		return -ENOMEM;

	mutex_init(&g_cam_dma_fence_dev->dev_lock);
	for (i = 0; i < CAM_DMA_FENCE_MAX_FENCES; i++)
		spin_lock_init(&g_cam_dma_fence_dev->row_spinlocks[i]);

	bitmap_zero(g_cam_dma_fence_dev->bitmap, CAM_DMA_FENCE_MAX_FENCES);
	g_cam_dma_fence_dev->dma_fence_context = dma_fence_context_alloc(1);

	CAM_DBG(CAM_DMA_FENCE, "Camera DMA fence driver initialized");
	return 0;
}

void cam_dma_fence_driver_deinit(void)
{
	kfree(g_cam_dma_fence_dev);
	g_cam_dma_fence_dev = NULL;
	CAM_DBG(CAM_DMA_FENCE, "Camera DMA fence driver deinitialized");
}
