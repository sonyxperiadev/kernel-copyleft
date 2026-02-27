// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/dma-direction.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include "msm_cvp_core.h"
#include "msm_cvp_internal.h"
#include "msm_cvp_debug.h"
#include "msm_cvp.h"
#include "msm_cvp_common.h"
#include <linux/delay.h>
#include "cvp_hfi_api.h"
#include "msm_cvp_clocks.h"
#include <linux/dma-buf.h>

#define MAX_EVENTS 30
#define NUM_CYCLES16X16_HCD_FRAME 95
#define NUM_CYCLES16X16_DMM_FRAME 600
#define NUM_CYCLES16X16_NCC_FRAME 400
#define NUM_CYCLES16X16_DS_FRAME  80
#define NUM_CYCLESFW_FRAME  1680000
#define NUM_DMM_MAX_FEATURE_POINTS 500
#define CYCLES_MARGIN_IN_POWEROF2 3

static atomic_t nr_insts;

void *cvp_kmem_cache_zalloc(struct cvp_kmem_cache *k, gfp_t flags)
{
	atomic_inc(&k->nr_objs);
	return kmem_cache_zalloc(k->cache, flags);
}

void cvp_kmem_cache_free(struct cvp_kmem_cache *k, void *obj)
{
	atomic_dec(&k->nr_objs);
	kmem_cache_free(k->cache, obj);
}

int msm_cvp_poll(void *instance, struct file *filp,
		struct poll_table_struct *wait)
{
	return 0;
}
EXPORT_SYMBOL(msm_cvp_poll);

int msm_cvp_private(void *cvp_inst, unsigned int cmd,
		struct eva_kmd_arg *arg)
{
	int rc = 0;
	struct msm_cvp_inst *inst = (struct msm_cvp_inst *)cvp_inst;

	if (!inst || !arg) {
		dprintk(CVP_ERR, "%s: invalid args\n", __func__);
		return -EINVAL;
	}

	rc = msm_cvp_handle_syscall(inst, arg);

	return rc;
}
EXPORT_SYMBOL(msm_cvp_private);

static bool msm_cvp_check_for_inst_overload(struct msm_cvp_core *core,
		u32 *instance_count)
{
	u32 secure_instance_count = 0;
	struct msm_cvp_inst *inst = NULL;
	bool overload = false;

	mutex_lock(&core->lock);
	list_for_each_entry(inst, &core->instances, list) {
		(*instance_count)++;
		/* This flag is not updated yet for the current instance */
		if (inst->flags & CVP_SECURE)
			secure_instance_count++;
	}
	mutex_unlock(&core->lock);

	/* Instance count includes current instance as well. */

	if ((*instance_count >= core->resources.max_inst_count) ||
		(secure_instance_count >=
			core->resources.max_secure_inst_count))
		overload = true;
	return overload;
}

static int __init_session_queue(struct msm_cvp_inst *inst)
{
	spin_lock_init(&inst->session_queue.lock);
	INIT_LIST_HEAD(&inst->session_queue.msgs);
	inst->session_queue.msg_count = 0;
	init_waitqueue_head(&inst->session_queue.wq);
	inst->session_queue.state = QUEUE_ACTIVE;
	return 0;
}

static void __init_fence_queue(struct msm_cvp_inst *inst)
{
	mutex_init(&inst->fence_cmd_queue.lock);
	INIT_LIST_HEAD(&inst->fence_cmd_queue.wait_list);
	INIT_LIST_HEAD(&inst->fence_cmd_queue.sched_list);
	init_waitqueue_head(&inst->fence_cmd_queue.wq);
	inst->fence_cmd_queue.state = QUEUE_ACTIVE;
	inst->fence_cmd_queue.mode = OP_NORMAL;

	spin_lock_init(&inst->session_queue_fence.lock);
	INIT_LIST_HEAD(&inst->session_queue_fence.msgs);
	inst->session_queue_fence.msg_count = 0;
	init_waitqueue_head(&inst->session_queue_fence.wq);
	inst->session_queue_fence.state = QUEUE_ACTIVE;
}

static void __deinit_fence_queue(struct msm_cvp_inst *inst)
{
	mutex_destroy(&inst->fence_cmd_queue.lock);
	inst->fence_cmd_queue.state = QUEUE_INVALID;
	inst->fence_cmd_queue.mode = OP_INVALID;
}

static void __deinit_session_queue(struct msm_cvp_inst *inst)
{
	struct cvp_session_msg *msg, *tmpmsg;

	/* free all messages */
	spin_lock(&inst->session_queue.lock);
	list_for_each_entry_safe(msg, tmpmsg, &inst->session_queue.msgs, node) {
		list_del_init(&msg->node);
		cvp_kmem_cache_free(&cvp_driver->msg_cache, msg);
	}
	inst->session_queue.msg_count = 0;
	inst->session_queue.state = QUEUE_INVALID;
	spin_unlock(&inst->session_queue.lock);

	wake_up_all(&inst->session_queue.wq);
}

struct msm_cvp_inst *msm_cvp_open(int session_type, struct task_struct *task)
{
	struct msm_cvp_inst *inst = NULL;
	struct msm_cvp_core *core = NULL;
	int rc = 0;
	int i = 0;
	u32 instance_count;

	core = cvp_driver->cvp_core;
	if (!core) {
		dprintk(CVP_ERR, "%s CVP core not initialized\n", __func__);
		goto err_invalid_core;
	}

	if (!msm_cvp_auto_pil && session_type == MSM_CVP_BOOT) {
		dprintk(CVP_SESS, "Auto PIL disabled, bypass CVP init at boot");
		goto err_invalid_core;
	}

	core->resources.max_inst_count = MAX_SUPPORTED_INSTANCES;
	if (msm_cvp_check_for_inst_overload(core, &instance_count)) {
		dprintk(CVP_ERR, "Instance num reached Max, rejecting session");
		mutex_lock(&core->lock);
		list_for_each_entry(inst, &core->instances, list)
			cvp_print_inst(CVP_ERR, inst);
		mutex_unlock(&core->lock);

		return NULL;
	}

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst) {
		dprintk(CVP_ERR, "Failed to allocate memory\n");
		rc = -ENOMEM;
		goto err_invalid_core;
	}

	pr_info(CVP_DBG_TAG "%s opening cvp instance: %pK type %d cnt %d\n",
		"sess", task->comm, inst, session_type, instance_count);
	mutex_init(&inst->sync_lock);
	mutex_init(&inst->lock);
	spin_lock_init(&inst->event_handler.lock);

	INIT_MSM_CVP_LIST(&inst->persistbufs);
	INIT_DMAMAP_CACHE(&inst->dma_cache);
	INIT_MSM_CVP_LIST(&inst->cvpdspbufs);
	INIT_MSM_CVP_LIST(&inst->cvpwnccbufs);
	INIT_MSM_CVP_LIST(&inst->frames);

	inst->cvpwnccbufs_num = 0;
	inst->cvpwnccbufs_table = NULL;

	init_waitqueue_head(&inst->event_handler.wq);

	kref_init(&inst->kref);

	inst->session_type = session_type;
	inst->state = MSM_CVP_CORE_UNINIT_DONE;
	inst->core = core;
	inst->clk_data.min_freq = 0;
	inst->clk_data.curr_freq = 0;
	inst->clk_data.ddr_bw = 0;
	inst->clk_data.sys_cache_bw = 0;
	inst->clk_data.bitrate = 0;

	for (i = SESSION_MSG_INDEX(SESSION_MSG_START);
		i <= SESSION_MSG_INDEX(SESSION_MSG_END); i++) {
		init_completion(&inst->completions[i]);
	}

	msm_cvp_session_init(inst);

	__init_fence_queue(inst);
	mutex_lock(&core->lock);
	mutex_lock(&core->clk_lock);
	list_add_tail(&inst->list, &core->instances);
	atomic_inc(&nr_insts);
	mutex_unlock(&core->clk_lock);
	mutex_unlock(&core->lock);

	rc = __init_session_queue(inst);
	if (rc)
		goto fail_init;

	rc = msm_cvp_comm_try_state(inst, MSM_CVP_CORE_INIT_DONE);
	if (rc) {
		dprintk(CVP_ERR,
			"Failed to move cvp instance to init state\n");
		goto fail_init;
	}

	inst->debugfs_root =
		msm_cvp_debugfs_init_inst(inst, core->debugfs_root);
	strlcpy(inst->proc_name, task->comm, TASK_COMM_LEN);

	return inst;
fail_init:
	__deinit_session_queue(inst);
	__deinit_fence_queue(inst);
	mutex_lock(&core->lock);
	list_del(&inst->list);
	mutex_unlock(&core->lock);
	mutex_destroy(&inst->sync_lock);
	mutex_destroy(&inst->lock);

	DEINIT_MSM_CVP_LIST(&inst->persistbufs);
	DEINIT_DMAMAP_CACHE(&inst->dma_cache);
	DEINIT_MSM_CVP_LIST(&inst->cvpdspbufs);
	DEINIT_MSM_CVP_LIST(&inst->cvpwnccbufs);
	DEINIT_MSM_CVP_LIST(&inst->frames);

	kfree(inst);
	inst = NULL;
err_invalid_core:
	return inst;
}
EXPORT_SYMBOL(msm_cvp_open);

static void msm_cvp_clean_sess_queue(struct msm_cvp_inst *inst,
		struct cvp_session_queue *sq)
{
	struct cvp_session_msg *mptr, *dummy;
	u64 ktid  = 0LL;

check_again:
	spin_lock(&sq->lock);
	if (sq->msg_count && sq->state != QUEUE_ACTIVE) {
		list_for_each_entry_safe(mptr, dummy, &sq->msgs, node) {
			ktid = mptr->pkt.client_data.kdata;
			if (ktid) {
				list_del_init(&mptr->node);
				sq->msg_count--;
				break;
			}
		}
	}
	spin_unlock(&sq->lock);

	if (ktid) {
		msm_cvp_unmap_frame(inst, ktid);
		cvp_kmem_cache_free(&cvp_driver->msg_cache, mptr);
		mptr = NULL;
		ktid = 0LL;
		goto check_again;
	}
}

static int msm_cvp_cleanup_instance(struct msm_cvp_inst *inst)
{
	bool empty;
	int rc, max_retries;
	struct msm_cvp_frame *frame;
	struct cvp_session_queue *sq, *sqf;
	struct cvp_hfi_ops *ops_tbl;
	struct msm_cvp_inst *tmp;

	if (!inst) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	sqf = &inst->session_queue_fence;
	sq = &inst->session_queue;

	max_retries =  inst->core->resources.msm_cvp_hw_rsp_timeout >> 5;
	msm_cvp_session_queue_stop(inst);

wait_dsp:
	mutex_lock(&inst->cvpdspbufs.lock);
	empty = list_empty(&inst->cvpdspbufs.list);
	if (!empty && max_retries > 0) {
		mutex_unlock(&inst->cvpdspbufs.lock);
		usleep_range(2000, 3000);
		max_retries--;
		goto wait_dsp;
	}
	mutex_unlock(&inst->cvpdspbufs.lock);

	if (!empty) {
		dprintk(CVP_WARN, "Failed sess %pK DSP frame pending\n", inst);
		/*
		 * A session is either DSP session or CPU session, cannot have both
		 * DSP and frame buffers
		 */
		goto stop_session;
	}

	max_retries =  inst->core->resources.msm_cvp_hw_rsp_timeout >> 1;
wait_frame:
	mutex_lock(&inst->frames.lock);
	empty = list_empty(&inst->frames.list);
	if (!empty && max_retries > 0) {
		mutex_unlock(&inst->frames.lock);
		usleep_range(1000, 2000);
		msm_cvp_clean_sess_queue(inst, sqf);
		msm_cvp_clean_sess_queue(inst, sq);
		max_retries--;
		goto wait_frame;
	}
	mutex_unlock(&inst->frames.lock);

	if (!empty) {
		dprintk(CVP_WARN,
			"Failed to process frames before session %pK close\n",
			inst);
		mutex_lock(&inst->frames.lock);
		list_for_each_entry(frame, &inst->frames.list, list)
			dprintk(CVP_WARN, "Unprocessed frame %08x ktid %llu\n",
				frame->pkt_type, frame->ktid);
		mutex_unlock(&inst->frames.lock);
		inst->core->synx_ftbl->cvp_dump_fence_queue(inst);
	}

stop_session:
	tmp = cvp_get_inst_validate(inst->core, inst);
	if (!tmp) {
		dprintk(CVP_ERR, "%s has a invalid session %llx\n",
			__func__, inst);
		goto exit;
	}
	if (!empty) {
		/* STOP SESSION to avoid SMMU fault after releasing ARP */
		ops_tbl = inst->core->dev_ops;
		rc = call_hfi_op(ops_tbl, session_stop, (void *)inst->session);
		if (rc) {
			dprintk(CVP_WARN, "%s: cannot stop session rc %d\n",
				__func__, rc);
			goto release_arp;
		}

		/*Fail stop session, release arp later may cause smmu fault*/
		rc = wait_for_sess_signal_receipt(inst, HAL_SESSION_STOP_DONE);
		if (rc)
			dprintk(CVP_WARN, "%s: wait for sess_stop fail, rc %d\n",
					__func__, rc);
		/* Continue to release ARP anyway */
	}
release_arp:
	cvp_put_inst(tmp);
exit:
	if (cvp_release_arp_buffers(inst))
		dprintk_rl(CVP_WARN,
			"Failed to release persist buffers\n");

	if (inst->prop.type == HFI_SESSION_FD
		|| inst->prop.type == HFI_SESSION_DMM) {
		spin_lock(&inst->core->resources.pm_qos.lock);
		if (inst->core->resources.pm_qos.off_vote_cnt > 0)
			inst->core->resources.pm_qos.off_vote_cnt--;
		else
			dprintk(CVP_INFO, "%s Unexpected pm_qos off vote %d\n",
				__func__,
				inst->core->resources.pm_qos.off_vote_cnt);
		spin_unlock(&inst->core->resources.pm_qos.lock);
		ops_tbl = inst->core->dev_ops;
		call_hfi_op(ops_tbl, pm_qos_update, ops_tbl->hfi_device_data);
	}
	return 0;
}

int msm_cvp_destroy(struct msm_cvp_inst *inst)
{
	struct msm_cvp_core *core;

	if (!inst || !inst->core) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	core = inst->core;

	if (inst->session_type == MSM_CVP_DSP) {
		cvp_dsp_del_sess(inst->dsp_handle, inst);
		inst->task = NULL;
	}

	/* Ensure no path has core->clk_lock and core->lock sequence */
	mutex_lock(&core->lock);
	mutex_lock(&core->clk_lock);
	/* inst->list lives in core->instances */
	list_del(&inst->list);
	atomic_dec(&nr_insts);
	mutex_unlock(&core->clk_lock);
	mutex_unlock(&core->lock);

	DEINIT_MSM_CVP_LIST(&inst->persistbufs);
	DEINIT_DMAMAP_CACHE(&inst->dma_cache);
	DEINIT_MSM_CVP_LIST(&inst->cvpdspbufs);
	DEINIT_MSM_CVP_LIST(&inst->cvpwnccbufs);
	DEINIT_MSM_CVP_LIST(&inst->frames);

	kfree(inst->cvpwnccbufs_table);
	inst->cvpwnccbufs_table = NULL;

	mutex_destroy(&inst->sync_lock);
	mutex_destroy(&inst->lock);

	msm_cvp_debugfs_deinit_inst(inst);

	__deinit_session_queue(inst);
	__deinit_fence_queue(inst);
	core->synx_ftbl->cvp_sess_deinit_synx(inst);

	pr_info(CVP_DBG_TAG
		"closed cvp instance: %pK session_id = %d type %d %d\n",
		inst->proc_name, inst, hash32_ptr(inst->session),
		inst->session_type, core->smem_leak_count);
	inst->session = (void *)0xdeadbeef;
	if (atomic_read(&inst->smem_count) > 0) {
		dprintk(CVP_WARN, "Session closed with %d unmapped smems\n",
			atomic_read(&inst->smem_count));
		core->smem_leak_count += atomic_read(&inst->smem_count);
	}
	kfree(inst);
	inst = NULL;
	dprintk(CVP_SESS,
		"sys-stat: nr_insts %d msgs %d, frames %d, bufs %d, smems %d\n",
		atomic_read(&nr_insts),
		atomic_read(&cvp_driver->msg_cache.nr_objs),
		atomic_read(&cvp_driver->frame_cache.nr_objs),
		atomic_read(&cvp_driver->buf_cache.nr_objs),
		atomic_read(&cvp_driver->smem_cache.nr_objs));
	return 0;
}

static void close_helper(struct kref *kref)
{
	struct msm_cvp_inst *inst;

	if (!kref)
		return;
	inst = container_of(kref, struct msm_cvp_inst, kref);

	msm_cvp_destroy(inst);
}

int msm_cvp_close(void *instance)
{
	struct msm_cvp_inst *inst = instance;
	int rc = 0;

	if (!inst || !inst->core) {
		dprintk_rl(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	pr_info(CVP_DBG_TAG
		"to close instance: %pK session_id = %d type %d state %d\n",
		inst->proc_name, inst, hash32_ptr(inst->session),
		inst->session_type, inst->state);

	if (inst->session == 0) {
		if (inst->state >= MSM_CVP_CORE_INIT_DONE &&
			inst->state < MSM_CVP_OPEN_DONE) {
			/* Session is not created, no ARP */
			inst->state = MSM_CVP_CORE_UNINIT;
			goto exit;
		}
		if (inst->state == MSM_CVP_CORE_UNINIT)
			return -EINVAL;
	}

	if (inst->session_type != MSM_CVP_BOOT) {
		rc = msm_cvp_cleanup_instance(inst);
		if (rc)
			return -EINVAL;
		msm_cvp_session_deinit(inst);
	}

	rc = msm_cvp_comm_try_state(inst, MSM_CVP_CORE_UNINIT);
	if (rc) {
		dprintk(CVP_ERR,
			"Failed to move inst %pK to uninit state\n", inst);
		rc = msm_cvp_deinit_core(inst);
	}

	msm_cvp_comm_session_clean(inst);
exit:
	kref_put(&inst->kref, close_helper);
	return 0;
}
EXPORT_SYMBOL(msm_cvp_close);

int msm_cvp_suspend(void)
{
	return msm_cvp_comm_suspend();
}
EXPORT_SYMBOL(msm_cvp_suspend);
