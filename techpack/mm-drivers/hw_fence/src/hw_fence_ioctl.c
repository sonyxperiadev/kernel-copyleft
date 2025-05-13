// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/ktime.h>
#include <linux/types.h>
#include <linux/sync_file.h>

#include "hw_fence_drv_priv.h"
#include "hw_fence_drv_utils.h"
#include "hw_fence_drv_ipc.h"
#include "hw_fence_drv_debug.h"

#define HW_SYNC_IOCTL_COUNT		ARRAY_SIZE(hw_sync_debugfs_ioctls)
#define HW_FENCE_ARRAY_SIZE		10
#define HW_SYNC_IOC_MAGIC		'W'
#define HW_SYNC_IOC_REG_CLIENT	_IOWR(HW_SYNC_IOC_MAGIC, 10, unsigned long)
#define HW_SYNC_IOC_UNREG_CLIENT	_IOWR(HW_SYNC_IOC_MAGIC, 11, unsigned long)
#define HW_SYNC_IOC_CREATE_FENCE	_IOWR(HW_SYNC_IOC_MAGIC, 12,\
						struct hw_fence_sync_create_data)
#define HW_SYNC_IOC_CREATE_FENCE_ARRAY	_IOWR(HW_SYNC_IOC_MAGIC, 14,\
						struct hw_fence_array_sync_create_data)
#define HW_SYNC_IOC_REG_FOR_WAIT	_IOWR(HW_SYNC_IOC_MAGIC, 16, int)
#define HW_SYNC_IOC_FENCE_SIGNAL	_IOWR(HW_SYNC_IOC_MAGIC, 17, unsigned long)
#define HW_SYNC_IOC_FENCE_WAIT	_IOWR(HW_SYNC_IOC_MAGIC, 18, int)
#define HW_SYNC_IOC_RESET_CLIENT	_IOWR(HW_SYNC_IOC_MAGIC, 19, unsigned long)
#define HW_FENCE_IOCTL_NR(n)			(_IOC_NR(n) - 2)
#define HW_IOCTL_DEF(ioctl, _func)	\
	[HW_FENCE_IOCTL_NR(ioctl)] = {		\
		.cmd = ioctl,			\
		.func = _func,			\
		.name = #ioctl			\
	}

#define ktime_compare_safe(A, B) ktime_compare(ktime_sub((A), (B)), ktime_set(0, 0))

/**
 * struct hw_sync_obj - per client hw sync object.
 * @context: context id used to create fences.
 * @client_id: to uniquely represent client.
 * @client_handle: Pointer to the structure holding the resources
 *                 allocated to the client.
 * @mem_descriptor: Memory descriptor of the queue allocated by the
 *                  hardware fence driver for each client during register.
 */
struct hw_sync_obj {
	u64 context;
	int client_id;
	void *client_handle;
	struct msm_hw_fence_mem_addr mem_descriptor;
};

/**
 * struct hw_fence_sync_create_data - data used in creating fences.
 * @seqno: sequence number.
 * @incr_context: if set, then the context would be incremented.
 * @fence: returns the fd of the new sync_file with the created fence.
 * @hash: fence hash
 */
struct hw_fence_sync_create_data {
	u64 seqno;
	bool incr_context;
	__s32 fence;
	u64 hash;
};

/**
 * struct hw_fence_array_sync_create_data - data used in creating multiple fences.
 * @seqno: sequence number used to create fence array.
 * @num_fences: number of fence fds received.
 * @fences: array of fence fds.
 * @fence_array_fd: fd of fence array.
 */
struct hw_fence_array_sync_create_data {
	u64 seqno;
	int num_fences;
	u64 fences[HW_FENCE_ARRAY_SIZE];
	__s32 fence_array_fd;
};

/**
 * struct hw_fence_sync_signal_data - data used to signal fences.
 * @hash: hash of the fence.
 * @error_flag: error flag
 */
struct hw_fence_sync_signal_data {
	u64 hash;
	u32 error_flag;
};

/**
 * struct hw_fence_sync_wait_data - data used to wait on fences.
 * @fence: fence fd.
 * @timeout_ms: fence wait time out.
 */
struct hw_fence_sync_wait_data {
	__s32 fence;
	u64 timeout_ms;
};

/**
 * struct hw_fence_sync_reset_data - data used to reset client.
 * @client_id: client id.
 * @reset_flag: reset flag
 */
struct hw_fence_sync_reset_data {
	int client_id;
	u32 reset_flag;
};

typedef long hw_fence_ioctl_t(struct hw_sync_obj *obj, unsigned long arg);

/**
 * struct hw_sync_ioctl_def - hw_sync driver ioctl entry
 * @cmd: ioctl command number, without flags
 * @func: handler for this ioctl
 * @name: user-readable name for debug output
 */
struct hw_sync_ioctl_def {
	unsigned int cmd;
	hw_fence_ioctl_t *func;
	const char *name;
};

static bool _is_valid_client(struct hw_sync_obj *obj)
{
	if (!obj)
		return false;

	if (obj->client_id < HW_FENCE_CLIENT_ID_VAL0 || obj->client_id > HW_FENCE_CLIENT_ID_VAL6) {
		HWFNC_ERR("invalid client_id:%d min:%d max:%d\n", obj->client_id,
				HW_FENCE_CLIENT_ID_VAL0, HW_FENCE_CLIENT_ID_VAL6);
		return false;
	}

	return true;
}

static int _get_client_id(struct hw_sync_obj *obj, unsigned long arg)
{
	int client_id;

	if (copy_from_user(&client_id, (void __user *)arg, sizeof(client_id)))
		return -EFAULT;

	if (!obj)
		return -EINVAL;

	if (client_id < HW_FENCE_CLIENT_ID_VAL0 || client_id > HW_FENCE_CLIENT_ID_VAL6) {
		HWFNC_ERR("invalid client_id:%d min:%d max:%d\n", client_id,
				HW_FENCE_CLIENT_ID_VAL0, HW_FENCE_CLIENT_ID_VAL6);
		return -EINVAL;
	}

	return client_id;
}

static void *_hw_sync_get_fence(int fd)
{
	return fd >= 0 ? sync_file_get_fence(fd) : NULL;
}

static int hw_sync_debugfs_open(struct inode *inode, struct file *file)
{
	struct hw_sync_obj *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return -ENOMEM;

	obj->context = dma_fence_context_alloc(1);
	file->private_data = obj;

	return 0;
}

static int hw_sync_debugfs_release(struct inode *inode, struct file *file)
{
	struct hw_sync_obj *obj = file->private_data;

	if (!obj)
		return -EINVAL;

	kfree(obj);

	return 0;
}

static long hw_sync_ioctl_reg_client(struct hw_sync_obj *obj, unsigned long arg)
{
	int client_id = _get_client_id(obj, arg);

	if (IS_ERR(&client_id)) {
		return client_id;
	} else if (obj->client_handle) {
		HWFNC_ERR("client:%d already registered as validation client\n", client_id);
		return -EINVAL;
	}

	obj->client_id = client_id;
	obj->client_handle = msm_hw_fence_register(obj->client_id, &obj->mem_descriptor);
	if (IS_ERR_OR_NULL(obj->client_handle))
		return -EINVAL;

	return 0;
}

static long hw_sync_ioctl_unreg_client(struct hw_sync_obj *obj, unsigned long arg)
{
	int client_id = _get_client_id(obj, arg);

	if (IS_ERR(&client_id)) {
		return client_id;
	} else if (client_id != obj->client_id) {
		HWFNC_ERR("deregistering hw-fence client %d with invalid client_id arg:%d\n",
			obj->client_id, client_id);
		return -EINVAL;
	}

	return msm_hw_fence_deregister(obj->client_handle);
}

static long hw_sync_ioctl_create_fence(struct hw_sync_obj *obj, unsigned long arg)
{
	struct msm_hw_fence_create_params params;
	struct hw_fence_sync_create_data data;
	struct hw_dma_fence *fence;
	spinlock_t *fence_lock;
	u64 hash;
	struct sync_file *sync_file;
	int fd, ret;

	if (!_is_valid_client(obj)) {
		return -EINVAL;
	} else if (IS_ERR_OR_NULL(obj->client_handle)) {
		HWFNC_ERR("client:%d is not register as validation client\n", obj->client_id);
		return -EINVAL;
	}

	if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
		return -EFAULT;

	/* create dma fence */
	fence_lock = kzalloc(sizeof(*fence_lock), GFP_KERNEL);
	if (!fence_lock)
		return -ENOMEM;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence) {
		kfree(fence_lock);
		return -ENOMEM;
	}

	snprintf(fence->name, HW_FENCE_NAME_SIZE, "hwfence:id:%d:ctx=%lu:seqno:%lu",
			obj->client_id, obj->context, data.seqno);

	spin_lock_init(fence_lock);
	dma_fence_init(&fence->base, &hw_fence_dbg_ops, fence_lock, obj->context, data.seqno);

	HWFNC_DBG_H("creating hw_fence for client:%d ctx:%llu seqno:%llu\n", obj->client_id,
				obj->context, data.seqno);
	params.fence = &fence->base;
	params.handle = &hash;

	/* create hw fence */
	ret = msm_hw_fence_create(obj->client_handle, &params);
	if (ret) {
		HWFNC_ERR("failed to create hw_fence for client:%d ctx:%llu seqno:%llu\n",
			obj->client_id, obj->context, data.seqno);
		dma_fence_put(&fence->base);
		return -EINVAL;
	}

	/* keep handle in dma_fence, to destroy hw-fence during release */
	fence->client_handle = obj->client_handle;

	if (data.incr_context)
		obj->context = dma_fence_context_alloc(1);

	/* create fd */
	fd = get_unused_fd_flags(0);
	if (fd < 0) {
		HWFNC_ERR("failed to get fd for client:%d\n", obj->client_id);
		dma_fence_put(&fence->base);
		return fd;
	}

	sync_file = sync_file_create(&fence->base);
	if (sync_file == NULL) {
		HWFNC_ERR("couldn't create fence fd, %d\n", fd);
		dma_fence_put(&fence->base);
		ret = -EINVAL;
		goto exit;
	}

	/* Decrement the refcount that sync_file_create increments */
	dma_fence_put(&fence->base);

	data.fence = fd;
	data.hash = hash;
	if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
		dma_fence_put(&fence->base);
		fput(sync_file->file);
		ret = -EFAULT;
		goto exit;
	}

	fd_install(fd, sync_file->file);

	return 0;

exit:
	put_unused_fd(fd);
	return ret;
}

static void _put_child_fences(int i, struct dma_fence **fences)
{
	int fence_idx;

	for (fence_idx = i; fence_idx >= 0 ; fence_idx--)
		dma_fence_put(fences[i]);
}

static long hw_sync_ioctl_create_fence_array(struct hw_sync_obj *obj, unsigned long arg)
{
	struct dma_fence_array *fence_array;
	struct hw_fence_array_sync_create_data data;
	struct dma_fence **fences = NULL;
	struct sync_file *sync_file;
	int num_fences, i, fd, ret;
	struct hw_dma_fence *fence;

	if (!_is_valid_client(obj)) {
		return -EINVAL;
	} else if (IS_ERR_OR_NULL(obj->client_handle)) {
		HWFNC_ERR("client:%d is not register as validation client\n", obj->client_id);
		return -EINVAL;
	}

	if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
		return -EFAULT;

	num_fences = data.num_fences;
	if (num_fences > HW_FENCE_ARRAY_SIZE) {
		HWFNC_ERR("Number of fences: %d is greater than allowed size: %d\n",
					num_fences, HW_FENCE_ARRAY_SIZE);
		return -EINVAL;
	}

	fences = kcalloc(num_fences, sizeof(*fences), GFP_KERNEL);
	if (!fences) {
		return -ENOMEM;
	}

	for (i = 0; i < num_fences; i++) {
		fd = data.fences[i];
		if (fd <= 0) {
			kfree(fences);
			return -EINVAL;
		}
		fence = (struct hw_dma_fence *)_hw_sync_get_fence(fd);
		if (!fence) {
			_put_child_fences(i-1, fences);
			kfree(fences);
			return -EINVAL;
		}
		fences[i] = &fence->base;
	}

	/* create the fence array from array of dma fences */
	fence_array = dma_fence_array_create(num_fences, fences, obj->context, data.seqno, 0);
	if (!fence_array) {
		HWFNC_ERR("Error creating fence_array\n");
		/* decrease the refcount incremented for each child fences */
		for (i = 0; i < num_fences; i++)
			dma_fence_put(fences[i]);
		kfree(fences);
		return -EINVAL;
	}

	/* create fd */
	fd = get_unused_fd_flags(0);
	if (fd <= 0) {
		HWFNC_ERR("failed to get fd for client:%d\n", obj->client_id);
		dma_fence_put(&fence_array->base);
		return fd;
	}

	sync_file = sync_file_create(&fence_array->base);
	if (sync_file == NULL) {
		HWFNC_ERR("couldn't create fence fd, %d\n", fd);
		dma_fence_put(&fence_array->base);
		kfree(fence_array);
		ret = -EINVAL;
		goto exit;
	}

	/* Decrement the refcount that sync_file_create increments */
	dma_fence_put(&fence_array->base);

	data.fence_array_fd = fd;
	if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
		fput(sync_file->file);
		dma_fence_put(&fence_array->base);
		ret = -EFAULT;
		goto exit;
	}

	fd_install(fd, sync_file->file);

	return 0;

exit:
	put_unused_fd(fd);
	return ret;
}

/*
 * this IOCTL only supports receiving one fence as input-parameter, which can be
 * either a "dma_fence" or a "dma_fence_array", but eventually we would expand
 * this API to receive more fences
 */
static long hw_sync_ioctl_reg_for_wait(struct hw_sync_obj *obj, unsigned long arg)
{
	struct dma_fence *fence;
	int ret, fd, num_fences = 1;

	if (!_is_valid_client(obj))
		return -EINVAL;

	if (copy_from_user(&fd, (void __user *)arg, sizeof(fd)))
		return -EFAULT;

	fence = (struct dma_fence *)_hw_sync_get_fence(fd);
	if (!fence) {
		HWFNC_ERR("Invalid fence fd: %d\n", fd);
		return -EINVAL;
	}

	ret = msm_hw_fence_wait_update_v2(obj->client_handle, &fence, NULL, NULL, num_fences, 1);

	/* Decrement the refcount that hw_sync_get_fence increments */
	dma_fence_put(fence);

	return ret;
}

static long hw_sync_ioctl_fence_signal(struct hw_sync_obj *obj, unsigned long arg)
{
	struct msm_hw_fence_client *hw_fence_client;
	struct hw_fence_sync_signal_data data;
	int ret, tx_client, rx_client, signal_id;

	if (!_is_valid_client(obj)) {
		return -EINVAL;
	} else if (IS_ERR_OR_NULL(obj->client_handle)) {
		HWFNC_ERR("invalid client handle for the client_id: %d\n", obj->client_id);
		return -EINVAL;
	}

	hw_fence_client = (struct msm_hw_fence_client *)obj->client_handle;
	if (!hw_fence_client) {
		HWFNC_ERR("invalid client handle\n");
		return -EINVAL;
	}

	if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
		return -EFAULT;

	ret = msm_hw_fence_update_txq(obj->client_handle, data.hash, 0, data.error_flag);
	if (ret) {
		HWFNC_ERR("hw fence update txq has failed client_id: %d\n", obj->client_id);
		return ret;
	}

	signal_id = dbg_out_clients_signal_map_no_dpu[obj->client_id].ipc_signal_id;
	if (signal_id < 0)
		return -EINVAL;

	tx_client = hw_fence_client->ipc_client_pid;
	rx_client = hw_fence_client->ipc_client_vid;
	ret = msm_hw_fence_trigger_signal(obj->client_handle, tx_client, rx_client, signal_id);
	if (ret) {
		HWFNC_ERR("hw fence trigger signal has failed\n");
		return ret;
	}

	return 0;
}

static long hw_sync_ioctl_fence_wait(struct hw_sync_obj *obj, unsigned long arg)
{
	struct msm_hw_fence_client *hw_fence_client;
	struct msm_hw_fence_queue_payload payload;
	struct hw_fence_sync_wait_data data;
	struct dma_fence *fence;
	ktime_t cur_ktime, exp_ktime;
	int fd, ret, read = 1, queue_type = HW_FENCE_RX_QUEUE - 1;  /* rx queue index */

	if (!_is_valid_client(obj))
		return -EINVAL;

	if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
		return -EFAULT;

	fd = data.fence;
	fence = (struct dma_fence *)_hw_sync_get_fence(fd);
	if (!fence) {
		HWFNC_ERR("Invalid fence fd: %d\n", fd);
		return -EINVAL;
	}

	hw_fence_client = (struct msm_hw_fence_client *)obj->client_handle;
	if (!hw_fence_client) {
		HWFNC_ERR("invalid client handle for fd:%d\n", fd);
		/* Decrement the refcount that hw_sync_get_fence increments */
		dma_fence_put(fence);
		return -EINVAL;
	}

	exp_ktime = ktime_add_ms(ktime_get(), data.timeout_ms);
	do {
		ret = wait_event_timeout(hw_fence_client->wait_queue,
				atomic_read(&hw_fence_client->val_signal) > 0,
				msecs_to_jiffies(data.timeout_ms));
		cur_ktime = ktime_get();
	} while ((atomic_read(&hw_fence_client->val_signal) <= 0) && (ret == 0) &&
		ktime_compare_safe(exp_ktime, cur_ktime) > 0);

	if (!ret) {
		HWFNC_ERR("timed out waiting for the client signal %d\n", data.timeout_ms);
		/* Decrement the refcount that hw_sync_get_fence increments */
		dma_fence_put(fence);
		return -ETIMEDOUT;
	}

	/* clear doorbell signal flag */
	atomic_set(&hw_fence_client->val_signal, 0);

	while (read) {
		read = hw_fence_read_queue(obj->client_handle, &payload, queue_type);
		if (read < 0) {
			HWFNC_ERR("unable to read client rxq client_id:%d\n", obj->client_id);
			break;
		}
		HWFNC_DBG_L("rxq read: hash:%llu, flags:%llu, error:%lu\n",
			payload.hash, payload.flags, payload.error);
		if (payload.ctxt_id == fence->context && payload.seqno == fence->seqno) {
			/* Decrement the refcount that hw_sync_get_fence increments */
			dma_fence_put(fence);
			return 0;
		}
	}

	/* Decrement the refcount that hw_sync_get_fence increments */
	dma_fence_put(fence);

	HWFNC_ERR("fence received did not match the fence expected\n");
	HWFNC_ERR("fence received: context:%d seqno:%d fence expected: context:%d seqno:%d\n",
				payload.ctxt_id, payload.seqno, fence->context, fence->seqno);

	return read;
}

static long hw_sync_ioctl_reset_client(struct hw_sync_obj *obj, unsigned long arg)
{
	int ret;
	struct hw_fence_sync_reset_data data;

	if (!_is_valid_client(obj)) {
		return -EINVAL;
	} else if (IS_ERR_OR_NULL(obj->client_handle)) {
		HWFNC_ERR("client:%d handle doesn't exists\n", obj->client_id);
		return -EINVAL;
	}

	if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
		return -EFAULT;

	ret = msm_hw_fence_reset_client(obj->client_handle, data.reset_flag);
	if (ret) {
		HWFNC_ERR("hw fence reset client has failed\n");
		return ret;
	}

	return 0;
}

static const struct hw_sync_ioctl_def hw_sync_debugfs_ioctls[] = {
	HW_IOCTL_DEF(HW_SYNC_IOC_REG_CLIENT, hw_sync_ioctl_reg_client),
	HW_IOCTL_DEF(HW_SYNC_IOC_UNREG_CLIENT, hw_sync_ioctl_unreg_client),
	HW_IOCTL_DEF(HW_SYNC_IOC_CREATE_FENCE, hw_sync_ioctl_create_fence),
	HW_IOCTL_DEF(HW_SYNC_IOC_CREATE_FENCE_ARRAY, hw_sync_ioctl_create_fence_array),
	HW_IOCTL_DEF(HW_SYNC_IOC_REG_FOR_WAIT, hw_sync_ioctl_reg_for_wait),
	HW_IOCTL_DEF(HW_SYNC_IOC_FENCE_SIGNAL, hw_sync_ioctl_fence_signal),
	HW_IOCTL_DEF(HW_SYNC_IOC_FENCE_WAIT, hw_sync_ioctl_fence_wait),
	HW_IOCTL_DEF(HW_SYNC_IOC_RESET_CLIENT, hw_sync_ioctl_reset_client)
};

static long hw_sync_debugfs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct hw_sync_obj *obj = file->private_data;
	int num = HW_FENCE_IOCTL_NR(cmd);
	hw_fence_ioctl_t *func;

	if (num >= HW_SYNC_IOCTL_COUNT) {
		HWFNC_ERR("invalid ioctl num = %d\n", num);
		return -EINVAL;
	}

	func = (&hw_sync_debugfs_ioctls[num])->func;
	if (unlikely(!func)) {
		HWFNC_ERR("no function num = %d\n", num);
		return -ENOTTY;
	}

	return func(obj, arg);
}

const struct file_operations hw_sync_debugfs_fops = {
	.open           = hw_sync_debugfs_open,
	.release        = hw_sync_debugfs_release,
	.unlocked_ioctl = hw_sync_debugfs_ioctl,
};
