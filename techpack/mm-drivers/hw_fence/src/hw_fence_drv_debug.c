// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#include "hw_fence_drv_priv.h"
#include "hw_fence_drv_debug.h"
#include "hw_fence_drv_ipc.h"
#include "hw_fence_drv_utils.h"

#define HW_FENCE_DEBUG_MAX_LOOPS 200

#define HFENCE_TBL_MSG \
	"[%d]hfence[%d] v:%d err:%lu ctx:%llu seq:%llu wait:0x%llx alloc:%d f:0x%llx child_cnt:%d" \
	"%s ct:%llu tt:%llu wt:%llu\n"

/* each hwfence parent includes one "32-bit" element + "," separator */
#define HW_FENCE_MAX_PARENTS_SUBLIST_DUMP (MSM_HW_FENCE_MAX_JOIN_PARENTS * 9)
#define HW_FENCE_MAX_PARENTS_DUMP (sizeof("parent_list[] ") + HW_FENCE_MAX_PARENTS_SUBLIST_DUMP)

/* event dump data includes one "32-bit" element + "|" separator */
#define HW_FENCE_MAX_DATA_PER_EVENT_DUMP (HW_FENCE_EVENT_MAX_DATA * 9)

#define HFENCE_EVT_MSG "[%d][cpu:%d][%lu] data[%d]:%s\n"

u32 msm_hw_fence_debug_level = HW_FENCE_PRINTK;

/**
 * struct client_data - Structure holding the data of the debug clients.
 *
 * @client_id: client id.
 * @dma_context: context id to create the dma-fences for the client.
 * @seqno_cnt: sequence number, this is a counter to simulate the seqno for debugging.
 * @client_handle: handle for the client, this is returned by the hw-fence driver after
 *                 a successful registration of the client.
 * @mem_descriptor: memory descriptor for the client-queues. This is populated by the hw-fence
 *                 driver after a successful registration of the client.
 * @list: client node.
 */
struct client_data {
	int client_id;
	u64 dma_context;
	u64 seqno_cnt;
	void *client_handle;
	struct msm_hw_fence_mem_addr mem_descriptor;
	struct list_head list;
};

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int _get_debugfs_input_client(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos,
	struct hw_fence_driver_data **drv_data)
{
	char buf[10];
	int client_id;

	if (!file || !file->private_data) {
		HWFNC_ERR("unexpected data %d\n", !file);
		return -EINVAL;
	}
	*drv_data = file->private_data;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0; /* end of string */

	if (kstrtouint(buf, 0, &client_id))
		return -EFAULT;

	if (client_id < HW_FENCE_CLIENT_ID_CTX0 || client_id >= HW_FENCE_CLIENT_MAX) {
		HWFNC_ERR("invalid client_id:%d min:%d max:%d\n", client_id,
			HW_FENCE_CLIENT_ID_CTX0, HW_FENCE_CLIENT_MAX);
		return -EINVAL;
	}

	return client_id;
}

static int _debugfs_ipcc_trigger(struct file *file, const char __user *user_buf,
	size_t count, loff_t *ppos, u32 tx_client, u32 rx_client)
{
	struct hw_fence_driver_data *drv_data;
	int client_id, signal_id;

	client_id = _get_debugfs_input_client(file, user_buf, count, ppos, &drv_data);
	if (client_id < 0)
		return -EINVAL;

	/* Get signal-id that hw-fence driver would trigger for this client */
	signal_id = hw_fence_ipcc_get_signal_id(drv_data, client_id);
	if (signal_id < 0)
		return -EINVAL;

	HWFNC_DBG_IRQ("client_id:%d ipcc write tx_client:%d rx_client:%d signal_id:%d qtime:%llu\n",
		client_id, tx_client, rx_client, signal_id, hw_fence_get_qtime(drv_data));
	hw_fence_ipcc_trigger_signal(drv_data, tx_client, rx_client, signal_id);

	return count;
}

/**
 * hw_fence_dbg_ipcc_write() - debugfs write to trigger an ipcc irq.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs receives as parameter a hw-fence driver client_id, and triggers an ipcc signal
 * from apps to apps for that client id.
 */
static ssize_t hw_fence_dbg_ipcc_write(struct file *file, const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct hw_fence_driver_data *drv_data = file->private_data;

	return _debugfs_ipcc_trigger(file, user_buf, count, ppos, drv_data->ipcc_client_pid,
		drv_data->ipcc_client_vid);
}

/**
 * hw_fence_dbg_ipcc_dpu_write() - debugfs write to trigger an ipcc irq to dpu core.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs receives as parameter a hw-fence driver client_id, and triggers an ipcc signal
 * from apps to dpu for that client id.
 */
static ssize_t hw_fence_dbg_ipcc_dpu_write(struct file *file, const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct hw_fence_driver_data *drv_data = file->private_data;

	return _debugfs_ipcc_trigger(file, user_buf, count, ppos, drv_data->ipcc_client_pid,
		hw_fence_ipcc_get_client_virt_id(drv_data, HW_FENCE_CLIENT_ID_CTL0));

}

static const struct file_operations hw_fence_dbg_ipcc_dpu_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_ipcc_dpu_write,
};

static const struct file_operations hw_fence_dbg_ipcc_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_ipcc_write,
};

struct client_data *_get_client_node(struct hw_fence_driver_data *drv_data, u32 client_id)
{
	struct client_data *node = NULL;
	bool found = false;

	mutex_lock(&drv_data->debugfs_data.clients_list_lock);
	list_for_each_entry(node, &drv_data->debugfs_data.clients_list, list) {
		if (node->client_id == client_id) {
			found = true;
			break;
		}
	}
	mutex_unlock(&drv_data->debugfs_data.clients_list_lock);

	return found ? node : NULL;
}

/**
 * hw_fence_dbg_reset_client_wr() - debugfs write to trigger reset in a debug hw-fence client.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs receives as parameter a hw-fence driver client_id, and triggers a reset for
 * this client. Note that this operation will only perform on hw-fence clients created through
 * the debug framework.
 */
static ssize_t hw_fence_dbg_reset_client_wr(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	int client_id, ret;
	struct client_data *client_info;
	struct hw_fence_driver_data *drv_data;

	client_id = _get_debugfs_input_client(file, user_buf, count, ppos, &drv_data);
	if (client_id < 0)
		return -EINVAL;

	client_info = _get_client_node(drv_data, client_id);
	if (!client_info || IS_ERR_OR_NULL(client_info->client_handle)) {
		HWFNC_ERR("client:%d not registered as debug client\n", client_id);
		return -EINVAL;
	}

	HWFNC_DBG_H("resetting client: %d\n", client_id);
	ret = msm_hw_fence_reset_client(client_info->client_handle, 0);
	if (ret)
		HWFNC_ERR("failed to reset client:%d\n", client_id);

	return count;
}

/**
 * hw_fence_dbg_register_clients_wr() - debugfs write to register a client with the hw-fence
 *                                      driver for debugging.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs receives as parameter a hw-fence driver client_id to register for debug.
 * Note that if the client_id received was already registered by any other driver, the
 * registration here will fail.
 */
static ssize_t hw_fence_dbg_register_clients_wr(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	int client_id;
	struct client_data *client_info;
	struct hw_fence_driver_data *drv_data;

	client_id = _get_debugfs_input_client(file, user_buf, count, ppos, &drv_data);
	if (client_id < 0)
		return -EINVAL;

	/* we cannot create same debug client twice */
	if (_get_client_node(drv_data, client_id)) {
		HWFNC_ERR("client:%d already registered as debug client\n", client_id);
		return -EINVAL;
	}

	client_info = kzalloc(sizeof(*client_info), GFP_KERNEL);
	if (!client_info)
		return -ENOMEM;

	HWFNC_DBG_H("register client %d\n", client_id);
	client_info->client_handle = msm_hw_fence_register(client_id,
		&client_info->mem_descriptor);
	if (IS_ERR_OR_NULL(client_info->client_handle)) {
		HWFNC_ERR("error registering as debug client:%d\n", client_id);
		client_info->client_handle = NULL;
		return -EFAULT;
	}

	client_info->dma_context = dma_fence_context_alloc(1);
	client_info->client_id = client_id;

	mutex_lock(&drv_data->debugfs_data.clients_list_lock);
	list_add(&client_info->list, &drv_data->debugfs_data.clients_list);
	mutex_unlock(&drv_data->debugfs_data.clients_list_lock);

	return count;
}

/**
 * hw_fence_dbg_tx_and_signal_clients_wr() - debugfs write to simulate the lifecycle of a hw-fence.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs receives as parameter the number of iterations that the simulation will run,
 * each iteration will: create, signal, register-for-signal and destroy a hw-fence.
 * Note that this simulation relies in the user first registering the clients as debug-clients
 * through the debugfs 'hw_fence_dbg_register_clients_wr'. If the clients are not previously
 * registered as debug-clients, this simulation will fail and won't run.
 */
static ssize_t hw_fence_dbg_tx_and_signal_clients_wr(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	u32 input_data, client_id_src, client_id_dst, tx_client, rx_client;
	struct client_data *client_info_src, *client_info_dst;
	struct hw_fence_driver_data *drv_data;
	struct msm_hw_fence_client *hw_fence_client, *hw_fence_client_dst;
	u64 context, seqno, hash;
	char buf[10];
	int signal_id, ret;

	if (!file || !file->private_data) {
		HWFNC_ERR("unexpected data %d\n", file);
		return -EINVAL;
	}
	drv_data = file->private_data;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0; /* end of string */

	if (kstrtouint(buf, 0, &input_data))
		return -EFAULT;

	if (input_data <= 0) {
		HWFNC_ERR("won't do anything, write value greather than 0 to start..\n");
		return 0;
	} else if (input_data > HW_FENCE_DEBUG_MAX_LOOPS) {
		HWFNC_ERR("requested loops:%d exceed max:%d, setting max\n", input_data,
			HW_FENCE_DEBUG_MAX_LOOPS);
		input_data = HW_FENCE_DEBUG_MAX_LOOPS;
	}

	client_id_src = HW_FENCE_CLIENT_ID_CTL0;
	client_id_dst = HW_FENCE_CLIENT_ID_CTL1;

	client_info_src = _get_client_node(drv_data, client_id_src);
	client_info_dst = _get_client_node(drv_data, client_id_dst);

	if (!client_info_src || IS_ERR_OR_NULL(client_info_src->client_handle) ||
			!client_info_dst || IS_ERR_OR_NULL(client_info_dst->client_handle)) {
		/* Make sure we registered this client through debugfs */
		HWFNC_ERR("client_id_src:%d or client_id_dst:%d not registered as debug client!\n",
			client_id_src, client_id_dst);
		return -EINVAL;
	}

	hw_fence_client = (struct msm_hw_fence_client *)client_info_src->client_handle;
	hw_fence_client_dst = (struct msm_hw_fence_client *)client_info_dst->client_handle;

	while (drv_data->debugfs_data.create_hw_fences && input_data > 0) {

		/***********************************************************/
		/***** SRC CLIENT - CREATE HW FENCE & TX QUEUE UPDATE ******/
		/***********************************************************/

		/* we will use the context and the seqno of the source client */
		context = client_info_src->dma_context;
		seqno = client_info_src->seqno_cnt;

		/* linear increment of the seqno for the src client*/
		client_info_src->seqno_cnt++;

		/* Create hw fence for src client */
		ret = hw_fence_create(drv_data, hw_fence_client, context, seqno, &hash);
		if (ret) {
			HWFNC_ERR("Error creating HW fence\n");
			goto exit;
		}

		/* Write to Tx queue */
		hw_fence_update_queue(drv_data, hw_fence_client, context, seqno, hash,
			0, 0, 0, HW_FENCE_TX_QUEUE - 1); /* no flags and no error */

		/**********************************************/
		/***** DST CLIENT - REGISTER WAIT CLIENT ******/
		/**********************************************/
		/* use same context and seqno that src client used to create fence */
		ret = hw_fence_register_wait_client(drv_data, NULL, hw_fence_client_dst, context,
			seqno, &hash, 0);
		if (ret) {
			HWFNC_ERR("failed to register for wait\n");
			return -EINVAL;
		}

		/*********************************************/
		/***** SRC CLIENT - TRIGGER IPCC SIGNAL ******/
		/*********************************************/

		/* AFTER THIS IS WHEN SVM WILL GET CALLED AND WILL PROCESS SRC AND DST CLIENTS */

		/* Trigger IPCC for SVM to read the queue */

		/* Get signal-id that hw-fence driver would trigger for this client */
		signal_id = dbg_out_clients_signal_map_no_dpu[client_id_src].ipc_signal_id;
		if (signal_id < 0)
			return -EINVAL;

		/*  Write to ipcc to trigger the irq */
		tx_client = drv_data->ipcc_client_pid;
		rx_client = drv_data->ipcc_client_vid;
		HWFNC_DBG_IRQ("client:%d tx_client:%d rx_client:%d signal:%d delay:%d in_data%d\n",
			client_id_src, tx_client, rx_client, signal_id,
			drv_data->debugfs_data.hw_fence_sim_release_delay, input_data);

		hw_fence_ipcc_trigger_signal(drv_data, tx_client, rx_client, signal_id);

		/********************************************/
		/******** WAIT ******************************/
		/********************************************/

		/* wait between iterations */
		usleep_range(drv_data->debugfs_data.hw_fence_sim_release_delay,
			(drv_data->debugfs_data.hw_fence_sim_release_delay + 5));

		/******************************************/
		/***** SRC CLIENT - CLEANUP HW FENCE ******/
		/******************************************/

		/* cleanup hw fence for src client */
		ret = hw_fence_destroy(drv_data, hw_fence_client, context, seqno);
		if (ret) {
			HWFNC_ERR("Error destroying HW fence\n");
			goto exit;
		}

		input_data--;
	} /* LOOP.. */

exit:
	return count;
}

/**
 * hw_fence_dbg_create_wr() - debugfs write to simulate the creation of a hw-fence.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs receives as parameter the client-id, for which the hw-fence will be created.
 * Note that this simulation relies in the user first registering the client as a debug-client
 * through the debugfs 'hw_fence_dbg_register_clients_wr'. If the client is not previously
 * registered as debug-client, this simulation will fail and won't run.
 */
static ssize_t hw_fence_dbg_create_wr(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct msm_hw_fence_create_params params;
	struct hw_fence_driver_data *drv_data;
	struct client_data *client_info;
	struct hw_dma_fence *dma_fence;
	spinlock_t *fence_lock;
	static u64 hw_fence_dbg_seqno = 1;
	int client_id, ret;
	u64 hash;

	client_id = _get_debugfs_input_client(file, user_buf, count, ppos, &drv_data);
	if (client_id < 0)
		return -EINVAL;

	client_info = _get_client_node(drv_data, client_id);
	if (!client_info || IS_ERR_OR_NULL(client_info->client_handle)) {
		HWFNC_ERR("client:%d not registered as debug client\n", client_id);
		return -EINVAL;
	}

	/* create debug dma_fence */
	fence_lock = kzalloc(sizeof(*fence_lock), GFP_KERNEL);
	if (!fence_lock)
		return -ENOMEM;

	dma_fence = kzalloc(sizeof(*dma_fence), GFP_KERNEL);
	if (!dma_fence) {
		kfree(fence_lock);
		return -ENOMEM;
	}

	snprintf(dma_fence->name, HW_FENCE_NAME_SIZE, "hwfence:id:%d:ctx=%lu:seqno:%lu",
		client_id, client_info->dma_context, hw_fence_dbg_seqno);

	spin_lock_init(fence_lock);
	dma_fence_init(&dma_fence->base, &hw_fence_dbg_ops, fence_lock,
		client_info->dma_context, hw_fence_dbg_seqno);

	HWFNC_DBG_H("creating hw_fence for client:%d ctx:%llu seqno:%llu\n", client_id,
		client_info->dma_context, hw_fence_dbg_seqno);
	params.fence = &dma_fence->base;
	params.handle = &hash;
	ret = msm_hw_fence_create(client_info->client_handle, &params);
	if (ret) {
		HWFNC_ERR("failed to create hw_fence for client:%d ctx:%llu seqno:%llu\n",
			client_id, client_info->dma_context, hw_fence_dbg_seqno);
		dma_fence_put(&dma_fence->base);
		return -EINVAL;
	}
	hw_fence_dbg_seqno++;

	/* keep handle in dma_fence, to destroy hw-fence during release */
	dma_fence->client_handle = client_info->client_handle;

	return count;
}

static void _dump_fence_helper(enum hw_fence_drv_prio prio, struct msm_hw_fence *hw_fence,
	char *parents_dump, u64 hash, u32 count)
{
	char sublist[HW_FENCE_MAX_PARENTS_SUBLIST_DUMP];
	u32 parents_cnt;
	int i, len = 0;

	if (!hw_fence || !parents_dump) {
		HWFNC_ERR("invalid params hw_fence:0x%pK parents_dump:0x%pK\n", hw_fence,
			parents_dump);
		return;
	}

	memset(parents_dump, 0, sizeof(char) * HW_FENCE_MAX_PARENTS_DUMP);
	if (hw_fence->parents_cnt) {
		if (hw_fence->parents_cnt > MSM_HW_FENCE_MAX_JOIN_PARENTS) {
			HWFNC_ERR("hfence[%d] has invalid parents_cnt:%d greater than max:%d\n",
				hash, hw_fence->parents_cnt, MSM_HW_FENCE_MAX_JOIN_PARENTS);
			parents_cnt = MSM_HW_FENCE_MAX_JOIN_PARENTS;
		} else {
			parents_cnt = hw_fence->parents_cnt;
		}

		memset(sublist, 0, sizeof(sublist));
		for (i = 0; i < parents_cnt; i++)
			len += scnprintf(sublist + len, HW_FENCE_MAX_PARENTS_SUBLIST_DUMP - len,
				"%lu,", hw_fence->parent_list[i]);
		scnprintf(parents_dump, HW_FENCE_MAX_PARENTS_DUMP, " p:[%s]", sublist);
	}

	HWFNC_DBG_DUMP(prio, HFENCE_TBL_MSG,
		count, hash, hw_fence->valid, hw_fence->error, hw_fence->ctx_id, hw_fence->seq_id,
		hw_fence->wait_client_mask, hw_fence->fence_allocator, hw_fence->flags,
		hw_fence->pending_child_cnt, parents_dump, hw_fence->fence_create_time,
		hw_fence->fence_trigger_time, hw_fence->fence_wait_time);
}

void hw_fence_debug_dump_fence(enum hw_fence_drv_prio prio, struct msm_hw_fence *hw_fence, u64 hash,
	u32 count)
{
	char parents_dump[HW_FENCE_MAX_PARENTS_DUMP];

	return _dump_fence_helper(prio, hw_fence, parents_dump, hash, count);
}

static inline int _dump_fence(struct msm_hw_fence *hw_fence, char *buf, int len, int max_size,
		u32 index, u32 cnt)
{
	int ret;
	char parents_dump[HW_FENCE_MAX_PARENTS_DUMP];

	_dump_fence_helper(HW_FENCE_INFO, hw_fence, parents_dump, index, cnt);

	ret = scnprintf(buf + len, max_size - len, HFENCE_TBL_MSG,
		cnt, index, hw_fence->valid, hw_fence->error, hw_fence->ctx_id, hw_fence->seq_id,
		hw_fence->wait_client_mask, hw_fence->fence_allocator, hw_fence->flags,
		hw_fence->pending_child_cnt, parents_dump, hw_fence->fence_create_time,
		hw_fence->fence_trigger_time, hw_fence->fence_wait_time);

	return ret;
}

void hw_fence_debug_dump_table(enum hw_fence_drv_prio prio, struct hw_fence_driver_data *drv_data)
{
	u32 i, cnt = 0;
	struct msm_hw_fence *hw_fence;

	for (i = 0; i < drv_data->hw_fences_tbl_cnt; i++) {
		hw_fence = &drv_data->hw_fences_tbl[i];
		if (!hw_fence->valid)
			continue;
		hw_fence_debug_dump_fence(prio, hw_fence, i, cnt);
		cnt++;
	}
}

static int dump_single_entry(struct hw_fence_driver_data *drv_data, char *buf, u32 *index,
	int max_size)
{
	struct msm_hw_fence *hw_fence;
	u64 context, seqno, hash = 0;
	int len = 0;

	context = drv_data->debugfs_data.context_rd;
	seqno = drv_data->debugfs_data.seqno_rd;

	hw_fence = msm_hw_fence_find(drv_data, NULL, context, seqno, &hash);
	if (!hw_fence) {
		HWFNC_ERR("no valid hfence found for context:%lu seqno:%lu hash:%lu",
				context, seqno, hash);
		len = scnprintf(buf + len, max_size - len,
			"no valid hfence found for context:%lu seqno:%lu hash:%lu\n",
			context, seqno, hash);

		goto exit;
	}

	len = _dump_fence(hw_fence, buf, len, max_size, hash, 0);

exit:
	/* move idx to end of table to stop the dump */
	*index = drv_data->hw_fences_tbl_cnt;

	return len;
}

static int dump_full_table(struct hw_fence_driver_data *drv_data, char *buf, u32 *index,
	u32 *cnt, int max_size, int entry_size)
{
	struct msm_hw_fence *hw_fence;
	int len = 0;

	while (((*index)++ < drv_data->hw_fences_tbl_cnt) && (len < (max_size - entry_size))) {
		hw_fence = &drv_data->hw_fences_tbl[*index];

		if (!hw_fence->valid)
			continue;

		len += _dump_fence(hw_fence, buf, len, max_size, *index, *cnt);
		(*cnt)++;
	}

	return len;
}

static void _find_earliest_event(struct hw_fence_driver_data *drv_data, u32 *start_index,
	u64 *start_time)
{
	u32 i;

	if (!start_index || !start_time) {
		HWFNC_ERR("invalid params start_index:0x%pK start_time:0x%pK\n", start_index,
			start_time);
		return;
	}

	mb(); /* make sure data is ready before read */
	for (i = 0; i < drv_data->total_events; i++) {
		u64 time = drv_data->events[i].time;

		if (time && (!*start_time || time < *start_time)) {
			*start_time = time;
			*start_index = i;
		}
	}
}

static void _dump_event(enum hw_fence_drv_prio prio, struct msm_hw_fence_event *event,
	char *data, u32 index)
{
	u32 data_cnt;
	int i, len = 0;

	if (!event || !data) {
		HWFNC_ERR("invalid params event:0x%pK data:0x%pK\n", event, data);
		return;
	}

	memset(data, 0, sizeof(char) * HW_FENCE_MAX_DATA_PER_EVENT_DUMP);
	if (event->data_cnt > HW_FENCE_EVENT_MAX_DATA) {
		HWFNC_ERR("event[%d] has invalid data_cnt:%lu greater than max_data_cnt:%lu\n",
			index, event->data_cnt, HW_FENCE_EVENT_MAX_DATA);
		data_cnt = HW_FENCE_EVENT_MAX_DATA;
	} else {
		data_cnt = event->data_cnt;
	}

	for (i = 0; i < data_cnt; i++)
		len += scnprintf(data + len, HW_FENCE_MAX_DATA_PER_EVENT_DUMP - len,
			"%lx|", event->data[i]);

	HWFNC_DBG_DUMP(prio, HFENCE_EVT_MSG, index, event->cpu, event->time, event->data_cnt, data);
}

void hw_fence_debug_dump_events(enum hw_fence_drv_prio prio, struct hw_fence_driver_data *drv_data)
{
	char data[HW_FENCE_MAX_DATA_PER_EVENT_DUMP];
	u32 start_index;
	u64 start_time;
	int i;

	if (!drv_data->events) {
		HWFNC_ERR("events not supported\n");
		return;
	}

	_find_earliest_event(drv_data, &start_index, &start_time);
	for (i = start_index; i < drv_data->total_events && drv_data->events[i].time; i++)
		_dump_event(prio, &drv_data->events[i], data, i);
	for (i = 0; i < start_index; i++)
		_dump_event(prio, &drv_data->events[i], data, i);
}

/**
 * hw_fence_dbg_dump_events_rd() - debugfs read to dump the fctl events.
 * @file: file handler.
 * @user_buf: user buffer content for debugfs.
 * @user_buf_size: size of the user buffer.
 * @ppos: position offset of the user buffer.
 */
static ssize_t hw_fence_dbg_dump_events_rd(struct file *file, char __user *user_buf,
	size_t user_buf_size, loff_t *ppos)
{
	struct hw_fence_driver_data *drv_data;
	u32 entry_size = sizeof(struct msm_hw_fence_event), max_size = SZ_4K;
	char *buf = NULL;
	int len = 0;
	static u64 start_time;
	static int index, start_index;
	static bool wraparound;

	if (!file || !file->private_data) {
		HWFNC_ERR("unexpected data %d\n", file);
		return -EINVAL;
	}
	drv_data = file->private_data;

	if (!drv_data->events) {
		HWFNC_ERR("events not supported\n");
		return -EINVAL;
	}

	if (wraparound && index >= start_index) {
		HWFNC_DBG_H("no more data index:%d total_events:%d\n", index,
			drv_data->total_events);
		start_time = 0;
		index = 0;
		wraparound = false;
		return 0;
	}

	if (user_buf_size < entry_size) {
		HWFNC_ERR("Not enough buff size:%d to dump entries:%d\n", user_buf_size,
			entry_size);
		return -EINVAL;
	}

	buf = kzalloc(max_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* find index of earliest event */
	if (!start_time) {
		_find_earliest_event(drv_data, &start_index, &start_time);
		index = start_index;
		HWFNC_DBG_H("events:0x%pK start_index:%d start_time:%llu total_events:%d\n",
			drv_data->events, start_index, start_time, drv_data->total_events);
	}

	HWFNC_DBG_H("++ dump_events index:%d qtime:%llu\n", index, hw_fence_get_qtime(drv_data));
	while ((!wraparound || index < start_index) && len < (max_size - entry_size)) {
		char data[HW_FENCE_MAX_DATA_PER_EVENT_DUMP];

		if (drv_data->events[index].time) {
			_dump_event(HW_FENCE_INFO, &drv_data->events[index], data, index);
			len += scnprintf(buf + len, max_size - len, HFENCE_EVT_MSG, index,
				drv_data->events[index].cpu, drv_data->events[index].time,
				drv_data->events[index].data_cnt, data);
		}

		index++;
		if (index >= drv_data->total_events) {
			index = 0;
			wraparound = true;
		}
	}
	HWFNC_DBG_H("-- dump_events: index:%d qtime:%llu\n", index, hw_fence_get_qtime(drv_data));

	if (len <= 0 || len > user_buf_size) {
		HWFNC_ERR("len:%d invalid buff size:%d\n", len, user_buf_size);
		len = 0;
		goto exit;
	}

	if (copy_to_user(user_buf, buf, len)) {
		HWFNC_ERR("failed to copy to user!\n");
		len = -EFAULT;
		goto exit;
	}
	*ppos += len;
exit:
	kfree(buf);
	return len;
}

static void _dump_queue(enum hw_fence_drv_prio prio, struct msm_hw_fence_client *hw_fence_client,
	int queue_type)
{
	struct msm_hw_fence_queue *queue;
	struct msm_hw_fence_hfi_queue_header *hfi_header;
	struct msm_hw_fence_queue_payload *payload;
	u64 timestamp;
	u32 *read_ptr, queue_entries;
	int i;

	queue = &hw_fence_client->queues[queue_type - 1];

	if ((queue_type > hw_fence_client->queues_num) || !queue || !queue->va_header
			|| !queue->va_queue) {
		HWFNC_ERR("Cannot dump client:%d q_type:%s q_ptr:0x%pK q_header:0x%pK q_va:0x%pK\n",
			hw_fence_client->client_id,
			(queue_type == HW_FENCE_TX_QUEUE) ? "TX QUEUE" : "RX QUEUE",
			queue, queue ? queue->va_header : NULL, queue ? queue->va_queue : NULL);
		return;
	}
	hfi_header = (struct msm_hw_fence_hfi_queue_header *)queue->va_header;

	mb(); /* make sure data is ready before read */
	HWFNC_DBG_DUMP(prio, "%s va:0x%pK rd_idx:%lu wr_idx:%lu tx_wm:%lu q_size_bytes:%lu\n",
		(queue_type == HW_FENCE_TX_QUEUE) ? "TX QUEUE" : "RX QUEUE", queue->va_queue,
		hfi_header->read_index, hfi_header->write_index, hfi_header->tx_wm,
		queue->q_size_bytes);
	queue_entries = queue->q_size_bytes / HW_FENCE_CLIENT_QUEUE_PAYLOAD;

	for (i = 0; i < queue_entries; i++) {
		read_ptr = ((u32 *)queue->va_queue +
			(i * (sizeof(struct msm_hw_fence_queue_payload) / sizeof(u32))));
		payload = (struct msm_hw_fence_queue_payload *)read_ptr;
		timestamp = (u64)payload->timestamp_lo | ((u64)payload->timestamp_hi << 32);

		HWFNC_DBG_DUMP(prio,
			"%s[%d]: hash:%d ctx:%llu seqno:%llu f:%llu d:%llu err:%u time:%llu\n",
			(queue_type == HW_FENCE_TX_QUEUE) ? "tx" : "rx", i, payload->hash,
			payload->ctxt_id, payload->seqno, payload->flags, payload->client_data,
			payload->error, timestamp);
	}
}

void hw_fence_debug_dump_queues(enum hw_fence_drv_prio prio,
	struct msm_hw_fence_client *hw_fence_client)
{
	if (!hw_fence_client) {
		HWFNC_ERR("Invalid params client:0x%pK\n", hw_fence_client);
		return;
	}

	HWFNC_DBG_DUMP(prio, "Queues for client %d\n", hw_fence_client->client_id);
	if (hw_fence_client->queues_num == HW_FENCE_CLIENT_QUEUES)
		_dump_queue(prio, hw_fence_client, HW_FENCE_RX_QUEUE);
	_dump_queue(prio, hw_fence_client, HW_FENCE_TX_QUEUE);
}

/**
 * hw_fence_dbg_dump_queues_wr() - debugfs wr to dump the hw-fences queues.
 * @file: file handler.
 * @user_buf: user buffer content for debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs dumps the hw-fence queues. Takes as input the desired client to dump.
 * Dumps to debug msgs the contents of the TX and RX queues for that client, if they exist.
 */
static ssize_t hw_fence_dbg_dump_queues_wr(struct file *file, const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct hw_fence_driver_data *drv_data;
	int client_id;

	if (!file || !file->private_data) {
		HWFNC_ERR("unexpected data %d\n", file);
		return -EINVAL;
	}
	drv_data = file->private_data;

	client_id = _get_debugfs_input_client(file, user_buf, count, ppos, &drv_data);
	if (client_id < 0)
		return -EINVAL;

	if (!drv_data->clients[client_id]) {
		HWFNC_ERR("client %d not initialized\n", client_id);
		return -EINVAL;
	}
	hw_fence_debug_dump_queues(HW_FENCE_PRINTK, drv_data->clients[client_id]);

	return count;
}

/**
 * hw_fence_dbg_dump_table_rd() - debugfs read to dump the hw-fences table.
 * @file: file handler.
 * @user_buf: user buffer content for debugfs.
 * @user_buf_size: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs dumps the hw-fence table. By default debugfs will dump all the valid entries of the
 * whole table. However, if user only wants to dump only one particular entry, user can provide the
 * context-id and seqno of the dma-fence of interest by writing to this debugfs node (see
 * documentation for the write in 'hw_fence_dbg_dump_table_wr').
 */
static ssize_t hw_fence_dbg_dump_table_rd(struct file *file, char __user *user_buf,
	size_t user_buf_size, loff_t *ppos)
{
	struct hw_fence_driver_data *drv_data;
	int entry_size = sizeof(struct msm_hw_fence);
	char *buf = NULL;
	int len = 0, max_size = SZ_4K;
	static u32 index, cnt;

	if (!file || !file->private_data) {
		HWFNC_ERR("unexpected data %d\n", file);
		return -EINVAL;
	}
	drv_data = file->private_data;

	if (!drv_data->hw_fences_tbl) {
		HWFNC_ERR("Failed to dump table: Null fence table\n");
		return -EINVAL;
	}

	if (index >= drv_data->hw_fences_tbl_cnt) {
		HWFNC_DBG_H("no more data index:%d cnt:%d\n", index, drv_data->hw_fences_tbl_cnt);
		index = cnt = 0;
		return 0;
	}

	if (user_buf_size < entry_size) {
		HWFNC_ERR("Not enough buff size:%d to dump entries:%d\n", user_buf_size,
			entry_size);
		return -EINVAL;
	}

	buf = kzalloc(max_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len = drv_data->debugfs_data.entry_rd ?
		dump_single_entry(drv_data, buf, &index, max_size) :
		dump_full_table(drv_data, buf, &index, &cnt, max_size, entry_size);

	if (len <= 0 || len > user_buf_size) {
		HWFNC_ERR("len:%d invalid buff size:%d\n", len, user_buf_size);
		len = 0;
		goto exit;
	}

	if (copy_to_user(user_buf, buf, len)) {
		HWFNC_ERR("failed to copy to user!\n");
		len = -EFAULT;
		goto exit;
	}
	*ppos += len;
exit:
	kfree(buf);
	return len;
}

/**
 * hw_fence_dbg_dump_table_wr() - debugfs write to control the dump of the hw-fences table.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @user_buf_size: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs receives as parameters the settings to dump either the whole hw-fences table
 * or only one element on the table in the next read of the same debugfs node.
 * If this debugfs receives two input values, it will interpret them as the 'context-id' and the
 * 'sequence-id' to dump from the hw-fence table in the subsequent reads of the debugfs.
 * Otherwise, if the debugfs receives only one input value, the next read from the debugfs, will
 * dump the whole hw-fences table.
 */
static ssize_t hw_fence_dbg_dump_table_wr(struct file *file,
		const char __user *user_buf, size_t user_buf_size, loff_t *ppos)
{
	struct hw_fence_driver_data *drv_data;
	u64 param_0, param_1;
	char buf[24];
	int num_input_params;

	if (!file || !file->private_data) {
		HWFNC_ERR("unexpected data %d\n", file);
		return -EINVAL;
	}
	drv_data = file->private_data;

	if (user_buf_size >= sizeof(buf)) {
		HWFNC_ERR("wrong size:%d size:%d\n", user_buf_size, sizeof(buf));
		return -EFAULT;
	}

	if (copy_from_user(buf, user_buf, user_buf_size))
		return -EFAULT;

	buf[user_buf_size] = 0; /* end of string */

	/* read the input params */
	num_input_params = sscanf(buf, "%lu %lu", &param_0, &param_1);

	if (num_input_params == 2) { /* if debugfs receives two input params */
		drv_data->debugfs_data.context_rd = param_0;
		drv_data->debugfs_data.seqno_rd = param_1;
		drv_data->debugfs_data.entry_rd = true;
	} else if (num_input_params == 1) { /* if debugfs receives one param */
		drv_data->debugfs_data.context_rd = 0;
		drv_data->debugfs_data.seqno_rd = 0;
		drv_data->debugfs_data.entry_rd = false;
	} else {
		HWFNC_ERR("invalid num params:%d\n", num_input_params);
		return -EFAULT;
	}

	return user_buf_size;
}



/**
 * hw_fence_dbg_create_join_fence() - debugfs write to simulate the lifecycle of a join hw-fence.
 * @file: file handler.
 * @user_buf: user buffer content from debugfs.
 * @count: size of the user buffer.
 * @ppos: position offset of the user buffer.
 *
 * This debugfs will: create, signal, register-for-signal and destroy a join hw-fence.
 * Note that this simulation relies in the user first registering the clients as debug-clients
 * through the debugfs 'hw_fence_dbg_register_clients_wr'. If the clients are not previously
 * registered as debug-clients, this simulation will fail and won't run.
 */
static ssize_t hw_fence_dbg_create_join_fence(struct file *file,
			const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct dma_fence_array *fence_array;
	struct hw_fence_driver_data *drv_data;
	struct dma_fence *fence_array_fence;
	struct client_data *client_info_src, *client_info_dst;
	u64 hw_fence_dbg_seqno = 1;
	int client_id_src, client_id_dst;
	struct msm_hw_fence_create_params params;
	int i, ret = 0;
	u64 hash;
	struct msm_hw_fence_client *hw_fence_client;
	int tx_client, rx_client, signal_id;

	/* creates 3 fences and a parent fence */
	int num_fences = 3;
	struct dma_fence **fences = NULL;
	spinlock_t **fences_lock = NULL;

	if (!file || !file->private_data) {
		HWFNC_ERR("unexpected data %d\n", file);
		return -EINVAL;
	}
	drv_data = file->private_data;
	client_id_src = HW_FENCE_CLIENT_ID_CTL0;
	client_id_dst = HW_FENCE_CLIENT_ID_CTL1;
	client_info_src = _get_client_node(drv_data, client_id_src);
	client_info_dst = _get_client_node(drv_data, client_id_dst);
	if (!client_info_src || IS_ERR_OR_NULL(client_info_src->client_handle) ||
			!client_info_dst || IS_ERR_OR_NULL(client_info_dst->client_handle)) {
		HWFNC_ERR("client_src:%d or client:%d is not register as debug client\n",
			client_id_src, client_id_dst);
		return -EINVAL;
	}
	hw_fence_client = (struct msm_hw_fence_client *)client_info_src->client_handle;

	fences_lock = kcalloc(num_fences, sizeof(*fences_lock), GFP_KERNEL);
	if (!fences_lock)
		return -ENOMEM;

	fences = kcalloc(num_fences, sizeof(*fences), GFP_KERNEL);
	if (!fences) {
		kfree(fences_lock);
		return -ENOMEM;
	}

	/* Create the array of dma fences */
	for (i = 0; i < num_fences; i++) {
		struct hw_dma_fence *dma_fence;

		fences_lock[i] = kzalloc(sizeof(spinlock_t), GFP_KERNEL);
		if (!fences_lock[i]) {
			_cleanup_fences(i, fences, fences_lock);
			return -ENOMEM;
		}

		dma_fence = kzalloc(sizeof(*dma_fence), GFP_KERNEL);
		if (!dma_fence) {
			_cleanup_fences(i, fences, fences_lock);
			return -ENOMEM;
		}
		fences[i] = &dma_fence->base;

		spin_lock_init(fences_lock[i]);
		dma_fence_init(fences[i], &hw_fence_dbg_ops, fences_lock[i],
			client_info_src->dma_context, hw_fence_dbg_seqno + i);
	}

	/* create the fence array from array of dma fences */
	fence_array = dma_fence_array_create(num_fences, fences,
				client_info_src->dma_context, hw_fence_dbg_seqno + num_fences, 0);
	if (!fence_array) {
		HWFNC_ERR("Error creating fence_array\n");
		_cleanup_fences(num_fences - 1, fences, fences_lock);
		return -EINVAL;
	}

	/* create hw fence and write to tx queue for each dma fence */
	for (i = 0; i < num_fences; i++) {
		params.fence = fences[i];
		params.handle = &hash;

		ret = msm_hw_fence_create(client_info_src->client_handle, &params);
		if (ret) {
			HWFNC_ERR("Error creating HW fence\n");
			count = -EINVAL;
			goto error;
		}

		/* Write to Tx queue */
		hw_fence_update_queue(drv_data, hw_fence_client, client_info_src->dma_context,
			hw_fence_dbg_seqno + i, hash, 0, 0, 0,
			HW_FENCE_TX_QUEUE - 1);
	}

	/* wait on the fence array */
	fence_array_fence = &fence_array->base;
	msm_hw_fence_wait_update_v2(client_info_dst->client_handle, &fence_array_fence, NULL, NULL,
		1, 1);

	signal_id = dbg_out_clients_signal_map_no_dpu[client_id_src].ipc_signal_id;
	if (signal_id < 0) {
		count = -EINVAL;
		goto error;
	}

	/* write to ipcc to trigger the irq */
	tx_client = drv_data->ipcc_client_pid;
	rx_client = drv_data->ipcc_client_vid;
	hw_fence_ipcc_trigger_signal(drv_data, tx_client, rx_client, signal_id);

	usleep_range(drv_data->debugfs_data.hw_fence_sim_release_delay,
		(drv_data->debugfs_data.hw_fence_sim_release_delay + 5));

error:
	/* this frees the memory for the fence-array and each dma-fence */
	dma_fence_put(&fence_array->base);

	/*
	 * free array of pointers, no need to call kfree in 'fences', since that is released
	 * from the fence-array release api
	 */
	kfree(fences_lock);

	return count;
}

int process_validation_client_loopback(struct hw_fence_driver_data *drv_data,
		int client_id)
{
	struct msm_hw_fence_client *hw_fence_client;

	if (client_id < HW_FENCE_CLIENT_ID_VAL0 || client_id > HW_FENCE_CLIENT_ID_VAL6) {
		HWFNC_ERR("invalid client_id: %d min: %d max: %d\n", client_id,
				HW_FENCE_CLIENT_ID_VAL0, HW_FENCE_CLIENT_ID_VAL6);
		return -EINVAL;
	}

	mutex_lock(&drv_data->clients_register_lock);

	if (!drv_data->clients[client_id]) {
		mutex_unlock(&drv_data->clients_register_lock);
		HWFNC_ERR("Processing workaround for unregistered val client:%d\n", client_id);
		return -EINVAL;
	}

	hw_fence_client = drv_data->clients[client_id];

	HWFNC_DBG_IRQ("Processing validation client workaround client_id:%d\n", client_id);

	/* set the atomic flag, to signal the client wait */
	atomic_set(&hw_fence_client->val_signal, 1);

	/* wake-up waiting client */
	wake_up_all(&hw_fence_client->wait_queue);

	mutex_unlock(&drv_data->clients_register_lock);

	return 0;
}

static const struct file_operations hw_fence_reset_client_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_reset_client_wr,
};

static const struct file_operations hw_fence_register_clients_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_register_clients_wr,
};

static const struct file_operations hw_fence_tx_and_signal_clients_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_tx_and_signal_clients_wr,
};

static const struct file_operations hw_fence_create_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_create_wr,
};

static const struct file_operations hw_fence_dump_table_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_dump_table_wr,
	.read = hw_fence_dbg_dump_table_rd,
};

static const struct file_operations hw_fence_dump_queues_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_dump_queues_wr,
};

static const struct file_operations hw_fence_dump_events_fops = {
	.open = simple_open,
	.read = hw_fence_dbg_dump_events_rd,
};

static const struct file_operations hw_fence_create_join_fence_fops = {
	.open = simple_open,
	.write = hw_fence_dbg_create_join_fence,
};

int hw_fence_debug_debugfs_register(struct hw_fence_driver_data *drv_data)
{
	struct dentry *debugfs_root;

	debugfs_root = debugfs_create_dir("hw_fence", NULL);
	if (IS_ERR_OR_NULL(debugfs_root)) {
		HWFNC_ERR("debugfs_root create_dir fail, error %ld\n",
			PTR_ERR(debugfs_root));
		drv_data->debugfs_data.root = NULL;
		return -EINVAL;
	}

	mutex_init(&drv_data->debugfs_data.clients_list_lock);
	INIT_LIST_HEAD(&drv_data->debugfs_data.clients_list);
	drv_data->debugfs_data.root = debugfs_root;
	drv_data->debugfs_data.create_hw_fences = true;
	drv_data->debugfs_data.hw_fence_sim_release_delay = 8333; /* uS */

	debugfs_create_file("ipc_trigger", 0600, debugfs_root, drv_data,
		&hw_fence_dbg_ipcc_fops);
	debugfs_create_file("dpu_trigger", 0600, debugfs_root, drv_data,
		&hw_fence_dbg_ipcc_dpu_fops);
	debugfs_create_file("hw_fence_reset_client", 0600, debugfs_root, drv_data,
		&hw_fence_reset_client_fops);
	debugfs_create_file("hw_fence_register_clients", 0600, debugfs_root, drv_data,
		&hw_fence_register_clients_fops);
	debugfs_create_file("hw_fence_tx_and_signal", 0600, debugfs_root, drv_data,
		&hw_fence_tx_and_signal_clients_fops);
	debugfs_create_file("hw_fence_create_join_fence", 0600, debugfs_root, drv_data,
		&hw_fence_create_join_fence_fops);
	debugfs_create_bool("create_hw_fences", 0600, debugfs_root,
		&drv_data->debugfs_data.create_hw_fences);
	debugfs_create_u32("sleep_range_us", 0600, debugfs_root,
		&drv_data->debugfs_data.hw_fence_sim_release_delay);
	debugfs_create_file("hw_fence_create", 0600, debugfs_root, drv_data,
		&hw_fence_create_fops);
	debugfs_create_u32("hw_fence_debug_level", 0600, debugfs_root, &msm_hw_fence_debug_level);
	debugfs_create_file("hw_fence_dump_table", 0600, debugfs_root, drv_data,
		&hw_fence_dump_table_fops);
	debugfs_create_file("hw_fence_dump_queues", 0600, debugfs_root, drv_data,
		&hw_fence_dump_queues_fops);
	debugfs_create_file("hw_sync", 0600, debugfs_root, NULL, &hw_sync_debugfs_fops);
	debugfs_create_u64("hw_fence_lock_wake_cnt", 0600, debugfs_root,
		&drv_data->debugfs_data.lock_wake_cnt);
	debugfs_create_file("hw_fence_dump_events", 0600, debugfs_root, drv_data,
		&hw_fence_dump_events_fops);

	return 0;
}

#else
int hw_fence_debug_debugfs_register(struct hw_fence_driver_data *drv_data)
{
	return 0;
}
#endif /* CONFIG_DEBUG_FS */
