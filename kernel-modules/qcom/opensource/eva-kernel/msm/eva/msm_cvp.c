// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "msm_cvp.h"
#include "cvp_hfi.h"
#include "cvp_core_hfi.h"
#include "msm_cvp_buf.h"
#include "cvp_comm_def.h"
#include "cvp_power.h"
#include "cvp_hfi_api.h"
static int cvp_enqueue_pkt(struct msm_cvp_inst* inst,
	struct eva_kmd_hfi_packet *in_pkt,
	unsigned int in_offset,
	unsigned int in_buf_num);

int msm_cvp_get_session_info(struct msm_cvp_inst *inst, u32 *session)
{
	int rc = 0;
	struct msm_cvp_inst *s;

	if (!inst || !inst->core || !session) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	s = cvp_get_inst_validate(inst->core, inst);
	if (!s)
		return -ECONNRESET;

	*session = hash32_ptr(inst->session);
	dprintk(CVP_SESS, "%s: id 0x%x\n", __func__, *session);

	cvp_put_inst(s);
	return rc;
}



static bool cvp_msg_pending(struct cvp_session_queue *sq,
				struct cvp_session_msg **msg, u64 *ktid)
{
	struct cvp_session_msg *mptr = NULL, *dummy;
	bool result = false;

	if (!sq)
		return false;
	spin_lock(&sq->lock);
	if (sq->state == QUEUE_INIT || sq->state == QUEUE_INVALID) {
		/* The session is being deleted */
		spin_unlock(&sq->lock);
		*msg = NULL;
		return true;
	}
	result = list_empty(&sq->msgs);
	if (!result) {
		mptr = list_first_entry(&sq->msgs,
				struct cvp_session_msg,
				node);
		if (!ktid) {
			if (mptr) {
				list_del_init(&mptr->node);
				sq->msg_count--;
			}
		} else {
			result = true;
			list_for_each_entry_safe(mptr, dummy, &sq->msgs, node) {
				if (*ktid == mptr->pkt.client_data.kdata) {
					list_del_init(&mptr->node);
					sq->msg_count--;
					result = false;
					break;
				}
			}
			if (result)
				mptr = NULL;
		}
	}
	spin_unlock(&sq->lock);
	*msg = mptr;
	return !result;
}

static int cvp_wait_process_message(struct msm_cvp_inst *inst,
				struct cvp_session_queue *sq, u64 *ktid,
				unsigned long timeout,
				struct eva_kmd_hfi_packet *out)
{
	struct cvp_session_msg *msg = NULL;
	struct cvp_hfi_msg_session_hdr *hdr;
	int rc = 0;

	if (wait_event_timeout(sq->wq,
		cvp_msg_pending(sq, &msg, ktid), timeout) == 0) {
		dprintk(CVP_WARN, "session queue wait timeout\n");
		if (inst && inst->core && inst->core->dev_ops &&
				inst->state != MSM_CVP_CORE_INVALID)
			print_hfi_queue_info(inst->core->dev_ops);
		rc = -ETIMEDOUT;
		goto exit;
	}

	if (msg == NULL) {
		dprintk(CVP_WARN, "%s: queue state %d, msg cnt %d\n", __func__,
					sq->state, sq->msg_count);

		if (inst->state >= MSM_CVP_CLOSE_DONE ||
				(sq->state != QUEUE_ACTIVE &&
				sq->state != QUEUE_START)) {
			rc = -ECONNRESET;
			goto exit;
		}

		msm_cvp_comm_kill_session(inst);
		goto exit;
	}

	if (!out) {
		cvp_kmem_cache_free(&cvp_driver->msg_cache, msg);
		goto exit;
	}

	hdr = (struct cvp_hfi_msg_session_hdr *)&msg->pkt;
	memcpy(out, &msg->pkt, get_msg_size(hdr));
	if (hdr->client_data.kdata >= ARRAY_SIZE(cvp_hfi_defs))
		msm_cvp_unmap_frame(inst, hdr->client_data.kdata);
	cvp_kmem_cache_free(&cvp_driver->msg_cache, msg);

exit:
	return rc;
}

static int msm_cvp_session_receive_hfi(struct msm_cvp_inst *inst,
			struct eva_kmd_hfi_packet *out_pkt)
{
	unsigned long wait_time;
	struct cvp_session_queue *sq;
	struct msm_cvp_inst *s;
	int rc = 0;

	if (!inst) {
		dprintk(CVP_ERR, "%s invalid session\n", __func__);
		return -EINVAL;
	}

	s = cvp_get_inst_validate(inst->core, inst);
	if (!s)
		return -ECONNRESET;

	wait_time = msecs_to_jiffies(
		inst->core->resources.msm_cvp_hw_rsp_timeout);
	sq = &inst->session_queue;

	rc = cvp_wait_process_message(inst, sq, NULL, wait_time, out_pkt);

	cvp_put_inst(inst);
	return rc;
}

static int msm_cvp_session_process_hfi(
	struct msm_cvp_inst *inst,
	struct eva_kmd_hfi_packet *in_pkt,
	unsigned int in_offset,
	unsigned int in_buf_num)
{
	int pkt_idx, rc = 0;

	unsigned int offset = 0, buf_num = 0, signal;
	struct cvp_session_queue *sq;
	struct msm_cvp_inst *s;
	struct cvp_hfi_cmd_session_hdr *pkt_hdr;
	bool is_config_pkt;


	if (!inst || !inst->core || !in_pkt) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (inst->state == MSM_CVP_CORE_INVALID) {
		dprintk(CVP_ERR, "sess %pK INVALIDim reject new HFIs\n", inst);
		return -ECONNRESET;
	}

	s = cvp_get_inst_validate(inst->core, inst);
	if (!s)
		return -ECONNRESET;

	pkt_hdr = (struct cvp_hfi_cmd_session_hdr *)in_pkt;
	dprintk(CVP_CMD, "%s: "
		"pkt_type %08x sess_id %08x trans_id %u ktid %llu\n",
		__func__, pkt_hdr->packet_type,
		pkt_hdr->session_id,
		pkt_hdr->client_data.transaction_id,
		pkt_hdr->client_data.kdata & (FENCE_BIT - 1));

	pkt_idx = get_pkt_index((struct cvp_hal_session_cmd_pkt *)in_pkt);
	if (pkt_idx < 0) {
		dprintk(CVP_ERR, "%s incorrect packet %d, %x\n", __func__,
				in_pkt->pkt_data[0],
				in_pkt->pkt_data[1]);
		goto exit;
	} else {
		signal = cvp_hfi_defs[pkt_idx].resp;
		is_config_pkt = cvp_hfi_defs[pkt_idx].is_config_pkt;
	}

	if (is_config_pkt)
		pr_info_ratelimited(CVP_DBG_TAG "inst %pK config %s\n", "sess",
			inst, cvp_hfi_defs[pkt_idx].name);

	if (signal == HAL_NO_RESP) {
		/* Frame packets are not allowed before session starts*/
		sq = &inst->session_queue;
		spin_lock(&sq->lock);
		if ((sq->state != QUEUE_START && !is_config_pkt) ||
			(sq->state >= QUEUE_INVALID)) {
			/*
			 * A init packet is allowed in case of
			 * QUEUE_ACTIVE, QUEUE_START, QUEUE_STOP
			 * A frame packet is only allowed in case of
			 * QUEUE_START
			 */
			spin_unlock(&sq->lock);
			dprintk(CVP_ERR, "%s: invalid queue state %d\n",
				__func__, sq->state);
			rc = -EINVAL;
			goto exit;
		}
		spin_unlock(&sq->lock);
	}

	if (in_offset && in_buf_num) {
		offset = in_offset;
		buf_num = in_buf_num;
	}
	if (!is_buf_param_valid(buf_num, offset)) {
		dprintk(CVP_ERR, "Incorrect buffer num and offset in cmd\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = msm_cvp_proc_oob(inst, in_pkt);
	if (rc) {
		dprintk(CVP_ERR, "%s: failed to process OOB buffer", __func__);
		goto exit;
	}

	rc = cvp_enqueue_pkt(inst, in_pkt, offset, buf_num);
	if (rc) {
		dprintk(CVP_ERR, "Failed to enqueue pkt, inst %pK "
			"pkt_type %08x ktid %llu transaction_id %u\n",
			inst, pkt_hdr->packet_type,
			pkt_hdr->client_data.kdata,
			pkt_hdr->client_data.transaction_id);
	}

exit:
	cvp_put_inst(inst);
	return rc;
}

static bool cvp_fence_wait(struct cvp_fence_queue *q,
			struct cvp_fence_command **fence,
			enum queue_state *state)
{
	struct cvp_fence_command *f;

	if (!q)
		return false;

	*fence = NULL;

	while (!mutex_trylock(&q->lock))
		usleep_range(100, 200);
	*state = q->state;
	if (*state != QUEUE_START) {
		mutex_unlock(&q->lock);
		return true;
	}

	if (list_empty(&q->wait_list)) {
		mutex_unlock(&q->lock);
		return false;
	}

	f = list_first_entry(&q->wait_list, struct cvp_fence_command, list);
	list_del_init(&f->list);
	list_add_tail(&f->list, &q->sched_list);

	mutex_unlock(&q->lock);
	*fence = f;

	return true;
}

static int cvp_fence_proc(struct msm_cvp_inst *inst,
			struct cvp_fence_command *fc,
			struct cvp_hfi_cmd_session_hdr *pkt)
{
	int rc = 0;
	unsigned long timeout;
	u64 ktid;
	int synx_state = SYNX_STATE_SIGNALED_SUCCESS;
	struct cvp_hfi_ops *ops_tbl;
	struct cvp_session_queue *sq;
	u32 hfi_err = HFI_ERR_NONE;
	struct cvp_hfi_msg_session_hdr_ext hdr;
	struct iris_hfi_device *device;

	dprintk(CVP_SYNX, "%s %s\n", current->comm, __func__);

	if (!inst || !inst->core)
		return -EINVAL;

	ops_tbl = inst->core->dev_ops;
	sq = &inst->session_queue_fence;
	ktid = pkt->client_data.kdata;

	rc = inst->core->synx_ftbl->cvp_synx_ops(inst, CVP_INPUT_SYNX,
			fc, &synx_state);
	if (rc) {
		msm_cvp_unmap_frame(inst, pkt->client_data.kdata);
		goto exit;
	}

	rc = call_hfi_op(ops_tbl, session_send, (void *)inst->session,
			(struct eva_kmd_hfi_packet *)pkt);
	if (rc) {
		dprintk(CVP_ERR, "%s %s: Failed in call_hfi_op %d, %x\n",
			current->comm, __func__, pkt->size, pkt->packet_type);
		synx_state = SYNX_STATE_SIGNALED_CANCEL;
		goto exit;
	}

	timeout = msecs_to_jiffies(
			inst->core->resources.msm_cvp_hw_rsp_timeout);
	rc = cvp_wait_process_message(inst, sq, &ktid, timeout,
				(struct eva_kmd_hfi_packet *)&hdr);

	hfi_err = hdr.error_type;
	if (rc) {
		dprintk(CVP_ERR, "%s %s: cvp_wait_process_message rc %d\n",
			current->comm, __func__, rc);
		synx_state = SYNX_STATE_SIGNALED_CANCEL;
		goto exit;
	}
	if (hfi_err == HFI_ERR_SESSION_FLUSHED) {
		dprintk(CVP_SYNX, "%s %s: cvp_wait_process_message flushed\n",
			current->comm, __func__);
		synx_state = SYNX_STATE_SIGNALED_CANCEL;
	} else if (hfi_err == HFI_ERR_SESSION_STREAM_CORRUPT) {
		dprintk(CVP_INFO, "%s %s: cvp_wait_process_msg non-fatal %d\n",
		current->comm, __func__, hfi_err);
		synx_state = SYNX_STATE_SIGNALED_SUCCESS;
	} else if (hfi_err == HFI_ERR_SESSION_HW_HANG_DETECTED) {
		dprintk(CVP_ERR, "%s %s: cvp_wait_process_message hfi HW hang err %d\n",
			current->comm, __func__, hfi_err);
		synx_state = SYNX_STATE_SIGNALED_CANCEL;
		device = ops_tbl->hfi_device_data;
		cvp_dump_csr(device);
	} else if (hfi_err != HFI_ERR_NONE) {
		dprintk(CVP_ERR, "%s %s: cvp_wait_process_message hfi err %d\n",
			current->comm, __func__, hfi_err);
		synx_state = SYNX_STATE_SIGNALED_CANCEL;
	}

exit:
	rc = inst->core->synx_ftbl->cvp_synx_ops(inst, CVP_OUTPUT_SYNX,
			fc, &synx_state);
	return rc;
}

static int cvp_alloc_fence_data(struct cvp_fence_command **f, u32 size)
{
	struct cvp_fence_command *fcmd;
	int alloc_size = sizeof(struct cvp_hfi_msg_session_hdr_ext);

	fcmd = kzalloc(sizeof(struct cvp_fence_command), GFP_KERNEL);
	if (!fcmd)
		return -ENOMEM;

	alloc_size = (alloc_size >= size) ? alloc_size : size;
	fcmd->pkt = kzalloc(alloc_size, GFP_KERNEL);
	if (!fcmd->pkt) {
		kfree(fcmd);
		return -ENOMEM;
	}

	*f = fcmd;
	return 0;
}

static void cvp_free_fence_data(struct cvp_fence_command *f)
{
	kfree(f->pkt);
	f->pkt = NULL;
	kfree(f);
	f = NULL;
}

static int cvp_fence_thread(void *data)
{
	int rc = 0, num_fences;
	struct msm_cvp_inst *inst;
	struct cvp_fence_queue *q;
	enum queue_state state;
	struct cvp_fence_command *f;
	struct cvp_hfi_cmd_session_hdr *pkt;
	u32 *synx;
	u64 ktid = 0;

	dprintk(CVP_SYNX, "Enter %s\n", current->comm);

	inst = (struct msm_cvp_inst *)data;
	if (!inst || !inst->core || !inst->core->dev_ops) {
		dprintk(CVP_ERR, "%s invalid inst %pK\n", current->comm, inst);
		rc = -EINVAL;
		goto exit;
	}

	q = &inst->fence_cmd_queue;

wait:
	dprintk(CVP_SYNX, "%s starts wait\n", current->comm);

	f = NULL;
	wait_event_interruptible(q->wq, cvp_fence_wait(q, &f, &state));
	if (state != QUEUE_START)
		goto exit;

	if (!f) {
		usleep_range(100, 200);
		goto wait;
	}

	pkt = f->pkt;
	synx = (u32 *)f->synx;

	num_fences = f->num_fences - f->output_index;
	/*
	 * If there is output fence, go through fence path
	 * Otherwise, go through non-fenced path
	 */
	if (num_fences)
		ktid = pkt->client_data.kdata & (FENCE_BIT - 1);

	dprintk(CVP_SYNX, "%s pkt type %d on ktid %llu frameID %llu\n",
		current->comm, pkt->packet_type, ktid, f->frame_id);

	rc = cvp_fence_proc(inst, f, pkt);

	mutex_lock(&q->lock);
	inst->core->synx_ftbl->cvp_release_synx(inst, f);
	list_del_init(&f->list);
	state = q->state;
	mutex_unlock(&q->lock);

	dprintk(CVP_SYNX, "%s done with %d ktid %llu frameID %llu rc %d\n",
		current->comm, pkt->packet_type, ktid, f->frame_id, rc);

	cvp_free_fence_data(f);

	if (rc && state != QUEUE_START)
		goto exit;

	goto wait;

exit:
	dprintk(CVP_SYNX, "%s exit\n", current->comm);
	cvp_put_inst(inst);
	return rc;
}

static int msm_cvp_session_process_hfi_fence(struct msm_cvp_inst *inst,
					struct eva_kmd_arg *arg)
{
	dprintk(CVP_WARN, "Deprecated IOCTL command %s\n", __func__);
	return -EINVAL;
}


static int cvp_populate_fences( struct eva_kmd_hfi_packet *in_pkt,
	unsigned int offset, unsigned int num, struct msm_cvp_inst *inst)
{
	u32 i, buf_offset, fence_cnt;
	struct eva_kmd_fence fences[MAX_HFI_FENCE_SIZE];
	struct cvp_fence_command *f;
	struct cvp_hfi_cmd_session_hdr *cmd_hdr;
	struct cvp_fence_queue *q;
	enum op_mode mode;
	struct cvp_buf_type *buf;
	bool override;

	int rc = 0;

	override = get_pkt_fenceoverride((struct cvp_hal_session_cmd_pkt*)in_pkt);

	dprintk(CVP_SYNX, "%s:Fence Override is %d\n",__func__, override);
	dprintk(CVP_SYNX, "%s:Kernel Fence is %d\n", __func__, cvp_kernel_fence_enabled);

	q = &inst->fence_cmd_queue;

	mutex_lock(&q->lock);
	mode = q->mode;
	mutex_unlock(&q->lock);

	if (mode == OP_DRAINING) {
		dprintk(CVP_SYNX, "%s: flush in progress\n", __func__);
		rc = -EBUSY;
		goto exit;
	}

	cmd_hdr = (struct cvp_hfi_cmd_session_hdr *)in_pkt;
	rc = cvp_alloc_fence_data((&f), cmd_hdr->size);
	if (rc) {
		dprintk(CVP_ERR,"%s: Failed to alloc fence data", __func__);
		goto exit;
	}

	f->type = cmd_hdr->packet_type;
	f->mode = OP_NORMAL;
	f->signature = 0xFEEDFACE;
	f->num_fences = 0;
	f->output_index = 0;
	buf_offset = offset;

	if (cvp_kernel_fence_enabled == 0)
	{
		goto soc_fence;
	}
	else if (cvp_kernel_fence_enabled == 1)
	{
		goto kernel_fence;
	}
	else if (cvp_kernel_fence_enabled == 2)
	{
		if (override == true)
			goto kernel_fence;
		else if (override == false)
			goto soc_fence;
		else
		{
			dprintk(CVP_ERR, "%s: invalid params", __func__);
			rc = -EINVAL;
			goto free_exit;
		}
	}
	else
	{
		dprintk(CVP_ERR, "%s: invalid params", __func__);
		rc = -EINVAL;
		goto free_exit;
	}

soc_fence:
	for (i = 0; i < num; i++) {
		buf = (struct cvp_buf_type*)&in_pkt->pkt_data[buf_offset];
		buf_offset += sizeof(*buf) >> 2;

		if (buf->input_handle || buf->output_handle) {
			f->num_fences++;
			if (buf->input_handle)
				f->output_index++;
		}
	}
	f->signature = 0xB0BABABE;
	if (f->num_fences)
		goto fence_cmd_queue;

	goto free_exit;


kernel_fence:
	/* First pass to find INPUT synx handles */
	for (i = 0; i < num; i++) {
		buf = (struct cvp_buf_type *)&in_pkt->pkt_data[buf_offset];
		buf_offset += sizeof(*buf) >> 2;

		if (buf->input_handle) {
			/* Check fence_type? */
			fences[f->num_fences].h_synx = buf->input_handle;
			f->num_fences++;
			buf->fence_type &= ~INPUT_FENCE_BITMASK;
			buf->input_handle = 0;
		}
	}
	f->output_index = f->num_fences;

	dprintk(CVP_SYNX, "%s:Input Fence passed - Number of Fences is %d\n",
			__func__, f->num_fences);

	/*
	 * Second pass to find OUTPUT synx handle
	 * If no of fences is 0 dont execute the below portion until line 911, return 0
	 */
	buf_offset = offset;
	for (i = 0; i < num; i++) {
		buf = (struct cvp_buf_type*)&in_pkt->pkt_data[buf_offset];
		buf_offset += sizeof(*buf) >> 2;

		if (buf->output_handle) {
			/* Check fence_type? */
			fences[f->num_fences].h_synx = buf->output_handle;
			f->num_fences++;
			buf->fence_type &= ~OUTPUT_FENCE_BITMASK;
			buf->output_handle = 0;
		}
	}
	dprintk(CVP_SYNX, "%s:Output Fence passed - Number of Fences is %d\n",
			__func__, f->num_fences);

	if (f->num_fences == 0)
		goto free_exit;

	rc = inst->core->synx_ftbl->cvp_import_synx(inst, f, (u32*)fences);
	if (rc) {
		dprintk(CVP_ERR,"%s: Failed to import fences", __func__);
		goto free_exit;
	}

fence_cmd_queue:
	fence_cnt = f->num_fences;
	memcpy(f->pkt, cmd_hdr, cmd_hdr->size);
	f->pkt->client_data.kdata |= FENCE_BIT;

	mutex_lock(&q->lock);
	list_add_tail(&f->list, &inst->fence_cmd_queue.wait_list);
	mutex_unlock(&q->lock);

	wake_up(&inst->fence_cmd_queue.wq);

	return fence_cnt;

free_exit:
	cvp_free_fence_data(f);
exit:
	return rc;
}


static int cvp_enqueue_pkt(struct msm_cvp_inst* inst,
	struct eva_kmd_hfi_packet *in_pkt,
	unsigned int in_offset,
	unsigned int in_buf_num)
{
	struct cvp_hfi_ops *ops_tbl;
	struct cvp_hfi_cmd_session_hdr *cmd_hdr;
	int pkt_type, rc = 0;
	enum buf_map_type map_type;

	ops_tbl = inst->core->dev_ops;

	pkt_type = in_pkt->pkt_data[1];
	map_type = cvp_find_map_type(pkt_type);

	cmd_hdr = (struct cvp_hfi_cmd_session_hdr *)in_pkt;
	/* The kdata will be overriden by transaction ID if the cmd has buf */
	cmd_hdr->client_data.kdata = 0;
	dprintk(CVP_CMD, "%s: "
		"pkt_type %08x sess_id %08x trans_id %u ktid %llu\n",
		__func__, cmd_hdr->packet_type,
		cmd_hdr->session_id,
		cmd_hdr->client_data.transaction_id,
		cmd_hdr->client_data.kdata & (FENCE_BIT - 1));

	if (map_type == MAP_PERSIST)
		rc = msm_cvp_map_user_persist(inst, in_pkt, in_offset, in_buf_num);
	else if (map_type == UNMAP_PERSIST)
		rc = msm_cvp_unmap_user_persist(inst, in_pkt, in_offset, in_buf_num);
	else
		rc = msm_cvp_map_frame(inst, in_pkt, in_offset, in_buf_num);

	if (rc)
		return rc;

	rc = cvp_populate_fences(in_pkt, in_offset, in_buf_num, inst);
	if (rc == 0) {
		rc = call_hfi_op(ops_tbl, session_send, (void *)inst->session,
			in_pkt);
		if (rc) {
			dprintk(CVP_ERR,"%s: Failed in call_hfi_op %d, %x\n",
					__func__, in_pkt->pkt_data[0],
					in_pkt->pkt_data[1]);
			if (map_type == MAP_FRAME)
				msm_cvp_unmap_frame(inst,
					cmd_hdr->client_data.kdata);
		}
	} else if (rc > 0) {
		dprintk(CVP_SYNX, "Going fenced path\n");
		rc = 0;
	} else {
		dprintk(CVP_ERR,"%s: Failed to populate fences\n",
			__func__);
		if (map_type == MAP_FRAME)
			msm_cvp_unmap_frame(inst, cmd_hdr->client_data.kdata);
	}

	return rc;
}

static inline int div_by_1dot5(unsigned int a)
{
	unsigned long i = a << 1;

	return (unsigned int) i/3;
}

int msm_cvp_session_delete(struct msm_cvp_inst *inst)
{
	return 0;
}

int msm_cvp_session_create(struct msm_cvp_inst *inst)
{
	int rc = 0, rc1 = 0;
	struct cvp_session_queue *sq;

	if (!inst || !inst->core)
		return -EINVAL;

	if (inst->state >= MSM_CVP_CLOSE_DONE)
		return -ECONNRESET;

	if (inst->state != MSM_CVP_CORE_INIT_DONE ||
		inst->state > MSM_CVP_OPEN_DONE) {
		dprintk(CVP_ERR,
			"%s Incorrect CVP state %d to create session\n",
			__func__, inst->state);
		return -EINVAL;
	}

	rc = msm_cvp_comm_try_state(inst, MSM_CVP_OPEN_DONE);
	if (rc) {
		dprintk(CVP_ERR,
			"Failed to move instance to open done state\n");
		goto fail_create;
	}

	rc = cvp_comm_set_arp_buffers(inst);
	if (rc) {
		dprintk(CVP_ERR,
				"Failed to set ARP buffers\n");
		goto fail_init;
	}

	inst->core->synx_ftbl->cvp_sess_init_synx(inst);
	sq = &inst->session_queue;
	spin_lock(&sq->lock);
	sq->state = QUEUE_ACTIVE;
	spin_unlock(&sq->lock);
	return rc;

fail_init:
	rc1 = msm_cvp_comm_try_state(inst, MSM_CVP_CLOSE_DONE);
	if (rc1)
		dprintk(CVP_ERR, "%s: close failed\n", __func__);
fail_create:
	return rc;
}

static int session_state_check_init(struct msm_cvp_inst *inst)
{
	mutex_lock(&inst->lock);
	if (inst->state == MSM_CVP_OPEN || inst->state == MSM_CVP_OPEN_DONE) {
		mutex_unlock(&inst->lock);
		return 0;
	}
	mutex_unlock(&inst->lock);

	return msm_cvp_session_create(inst);
}

static int cvp_fence_thread_start(struct msm_cvp_inst *inst)
{
	u32 tnum = 0;
	u32 i = 0;
	int rc = 0;
	char tname[16];
	struct task_struct *thread;
	struct cvp_fence_queue *q;
	struct cvp_session_queue *sq;

	if (!inst->prop.fthread_nr)
		return 0;

	q = &inst->fence_cmd_queue;
	mutex_lock(&q->lock);
	q->state = QUEUE_START;
	mutex_unlock(&q->lock);

	for (i = 0; i < inst->prop.fthread_nr; ++i) {
		if (!cvp_get_inst_validate(inst->core, inst)) {
			rc = -ECONNRESET;
			goto exit;
		}

		snprintf(tname, sizeof(tname), "fthread_%d", tnum++);
		thread = kthread_run(cvp_fence_thread, inst, tname);
		if (!thread) {
			dprintk(CVP_ERR, "%s create %s fail", __func__, tname);
			rc = -ECHILD;
			goto exit;
		}
	}

	sq = &inst->session_queue_fence;
	spin_lock(&sq->lock);
	sq->state = QUEUE_START;
	spin_unlock(&sq->lock);

exit:
	if (rc) {
		mutex_lock(&q->lock);
		q->state = QUEUE_STOP;
		mutex_unlock(&q->lock);
		wake_up_all(&q->wq);
	}
	return rc;
}

static int cvp_fence_thread_stop(struct msm_cvp_inst *inst)
{
	struct cvp_fence_queue *q;
	struct cvp_session_queue *sq;

	if (!inst->prop.fthread_nr)
		return 0;

	q = &inst->fence_cmd_queue;

	mutex_lock(&q->lock);
	q->state = QUEUE_STOP;
	mutex_unlock(&q->lock);

	sq = &inst->session_queue_fence;
	spin_lock(&sq->lock);
	sq->state = QUEUE_STOP;
	spin_unlock(&sq->lock);

	wake_up_all(&q->wq);
	wake_up_all(&sq->wq);

	return 0;
}

int msm_cvp_session_start(struct msm_cvp_inst *inst,
		struct eva_kmd_arg *arg)
{
	struct cvp_session_queue *sq;
	struct cvp_hfi_ops *ops_tbl;
	int rc;
	enum queue_state old_state;

	if (!inst || !inst->core) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	sq = &inst->session_queue;
	spin_lock(&sq->lock);
	if (sq->msg_count) {
		dprintk(CVP_ERR, "session start failed queue not empty%d\n",
			sq->msg_count);
		spin_unlock(&sq->lock);
		rc = -EINVAL;
		goto exit;
	}
	old_state = sq->state;
	sq->state = QUEUE_START;
	spin_unlock(&sq->lock);

	ops_tbl = inst->core->dev_ops;
	if (inst->prop.type == HFI_SESSION_FD
		|| inst->prop.type == HFI_SESSION_DMM) {
		spin_lock(&inst->core->resources.pm_qos.lock);
		inst->core->resources.pm_qos.off_vote_cnt++;
		spin_unlock(&inst->core->resources.pm_qos.lock);
		call_hfi_op(ops_tbl, pm_qos_update, ops_tbl->hfi_device_data);
	}
	/*
	 * cvp_fence_thread_start will increment reference to instance.
	 * It guarantees the EVA session won't be deleted. Use of session
	 * functions, such as session_start requires the session to be valid.
	 */
	rc = cvp_fence_thread_start(inst);
	if (rc)
		goto restore_state;

	/* Send SESSION_START command */
	rc = call_hfi_op(ops_tbl, session_start, (void *)inst->session);
	if (rc) {
		dprintk(CVP_WARN, "%s: session start failed rc %d\n",
				__func__, rc);
		goto stop_thread;
	}

	/* Wait for FW response */
	rc = wait_for_sess_signal_receipt(inst, HAL_SESSION_START_DONE);
	if (rc) {
		dprintk(CVP_WARN, "%s: wait for signal failed, rc %d\n",
				__func__, rc);
		goto stop_thread;
	}

	pr_info_ratelimited(CVP_DBG_TAG "session %llx (%#x) started\n",
		"sess", inst, hash32_ptr(inst->session));

	return 0;

stop_thread:
	cvp_fence_thread_stop(inst);
restore_state:
	spin_lock(&sq->lock);
	sq->state = old_state;
	spin_unlock(&sq->lock);
exit:
	return rc;
}

int msm_cvp_session_stop(struct msm_cvp_inst *inst,
		struct eva_kmd_arg *arg)
{
	struct cvp_session_queue *sq;
	struct eva_kmd_session_control *sc = NULL;
	struct msm_cvp_inst *s;
	struct cvp_hfi_ops *ops_tbl;
	int rc;

	if (!inst || !inst->core) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (arg)
		sc = &arg->data.session_ctrl;

	s = cvp_get_inst_validate(inst->core, inst);
	if (!s)
		return -ECONNRESET;

	sq = &inst->session_queue;

	spin_lock(&sq->lock);
	if (sq->msg_count) {
		dprintk(CVP_ERR, "session stop incorrect: queue not empty%d\n",
			sq->msg_count);
		if (sc)
			sc->ctrl_data[0] = sq->msg_count;
		spin_unlock(&sq->lock);
		rc =  -EUCLEAN;
		goto exit;
	}
	sq->state = QUEUE_STOP;

	pr_info_ratelimited(CVP_DBG_TAG "Stop session: %pK session_id = %#x\n",
			"sess", inst, hash32_ptr(inst->session));
	spin_unlock(&sq->lock);

	ops_tbl = inst->core->dev_ops;
	/* Send SESSION_STOP command */
	rc = call_hfi_op(ops_tbl, session_stop, (void *)inst->session);
	if (rc) {
		dprintk(CVP_WARN, "%s: session stop failed rc %d\n",
				__func__, rc);
		goto stop_thread;
	}

	/* Wait for FW response */
	rc = wait_for_sess_signal_receipt(inst, HAL_SESSION_STOP_DONE);
	if (rc) {
		dprintk(CVP_WARN, "%s: wait for signal failed, rc %d\n",
				__func__, rc);
		goto stop_thread;
	}

stop_thread:
	wake_up_all(&inst->session_queue.wq);

	cvp_fence_thread_stop(inst);
exit:
	cvp_put_inst(s);
	return rc;
}

int msm_cvp_session_queue_stop(struct msm_cvp_inst *inst)
{
	struct cvp_session_queue *sq;

	sq = &inst->session_queue;

	spin_lock(&sq->lock);

	if (sq->state == QUEUE_STOP) {
		spin_unlock(&sq->lock);
		return 0;
	}

	sq->state = QUEUE_STOP;

	dprintk(CVP_SESS, "Stop session queue: %pK session_id = %#x\n",
			inst, hash32_ptr(inst->session));
	spin_unlock(&sq->lock);

	wake_up_all(&inst->session_queue.wq);

	return cvp_fence_thread_stop(inst);
}

static int msm_cvp_session_ctrl(struct msm_cvp_inst *inst,
		struct eva_kmd_arg *arg)
{
	struct eva_kmd_session_control *ctrl = &arg->data.session_ctrl;
	int rc = 0;
	unsigned int ctrl_type;

	ctrl_type = ctrl->ctrl_type;

	if (!inst && ctrl_type != SESSION_CREATE) {
		dprintk(CVP_ERR, "%s invalid session\n", __func__);
		return -EINVAL;
	}

	switch (ctrl_type) {
	case SESSION_STOP:
		rc = msm_cvp_session_stop(inst, arg);
		break;
	case SESSION_START:
		rc = msm_cvp_session_start(inst, arg);
		break;
	case SESSION_CREATE:
		rc = msm_cvp_session_create(inst);
		break;
	case SESSION_DELETE:
		rc = msm_cvp_session_delete(inst);
		break;
	case SESSION_INFO:
	default:
		dprintk(CVP_ERR, "%s Unsupported session ctrl%d\n",
			__func__, ctrl->ctrl_type);
		rc = -EINVAL;
	}
	return rc;
}

static int msm_cvp_get_sysprop(struct msm_cvp_inst *inst,
		struct eva_kmd_arg *arg)
{
	struct eva_kmd_sys_properties *props = &arg->data.sys_properties;
	struct cvp_hfi_ops *ops_tbl;
	struct iris_hfi_device *hfi;
	struct cvp_session_prop *session_prop;
	int i, rc = 0;

	if (!inst || !inst->core || !inst->core->dev_ops) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	ops_tbl = inst->core->dev_ops;
	hfi = ops_tbl->hfi_device_data;

	if (props->prop_num > MAX_KMD_PROP_NUM_PER_PACKET) {
		dprintk(CVP_ERR, "Too many properties %d to get\n",
			props->prop_num);
		return -E2BIG;
	}

	session_prop = &inst->prop;

	for (i = 0; i < props->prop_num; i++) {
		switch (props->prop_data[i].prop_type) {
		case EVA_KMD_PROP_HFI_VERSION:
		{
			props->prop_data[i].data = hfi->version;
			break;
		}
		case EVA_KMD_PROP_SESSION_DUMPOFFSET:
		{
			props->prop_data[i].data =
				session_prop->dump_offset;
			break;
		}
		case EVA_KMD_PROP_SESSION_DUMPSIZE:
		{
			props->prop_data[i].data =
				session_prop->dump_size;
			break;
		}
		case EVA_KMD_PROP_SESSION_ERROR:
		{
			get_dma_buf(hfi->sfr.mem_data.dma_buf);
			rc = dma_buf_fd(hfi->sfr.mem_data.dma_buf, O_RDONLY | O_CLOEXEC);
			if (rc < 0) {
				dprintk(CVP_WARN, "Failed get dma_buf fd %d\n", rc);
				break;
			}

			props->prop_data[i].data = rc;
			rc = 0;
			break;
		}
		case EVA_KMD_PROP_PWR_FDU:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_FDU);
			break;
		}
		case EVA_KMD_PROP_PWR_ICA:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_ICA);
			break;
		}
		case EVA_KMD_PROP_PWR_OD:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_OD);
			break;
		}
		case EVA_KMD_PROP_PWR_MPU:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_MPU);
			break;
		}
		case EVA_KMD_PROP_PWR_VADL:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_VADL);
			break;
		}
		case EVA_KMD_PROP_PWR_TOF:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_TOF);
			break;
		}
		case EVA_KMD_PROP_PWR_RGE:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_RGE);
			break;
		}
		case EVA_KMD_PROP_PWR_XRA:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_XRA);
			break;
		}
		case EVA_KMD_PROP_PWR_LSR:
		{
			props->prop_data[i].data =
				msm_cvp_get_hw_aggregate_cycles(HFI_HW_LSR);
			break;
		}
		default:
			dprintk(CVP_ERR, "unrecognized sys property %d\n",
				props->prop_data[i].prop_type);
			rc = -EFAULT;
		}
	}
	return rc;
}

static int msm_cvp_set_sysprop(struct msm_cvp_inst *inst,
		struct eva_kmd_arg *arg)
{
	struct eva_kmd_sys_properties *props = &arg->data.sys_properties;
	struct eva_kmd_sys_property *prop_array;
	struct cvp_session_prop *session_prop;
	int i, rc = 0;

	if (!inst) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (props->prop_num > MAX_KMD_PROP_NUM_PER_PACKET) {
		dprintk(CVP_ERR, "Too many properties %d to set\n",
			props->prop_num);
		return -E2BIG;
	}

	prop_array = &arg->data.sys_properties.prop_data[0];
	session_prop = &inst->prop;

	for (i = 0; i < props->prop_num; i++) {
		switch (prop_array[i].prop_type) {
		case EVA_KMD_PROP_SESSION_TYPE:
			session_prop->type = prop_array[i].data;
			break;
		case EVA_KMD_PROP_SESSION_KERNELMASK:
			session_prop->kernel_mask = prop_array[i].data;
			break;
		case EVA_KMD_PROP_SESSION_PRIORITY:
			session_prop->priority = prop_array[i].data;
			break;
		case EVA_KMD_PROP_SESSION_SECURITY:
			session_prop->is_secure = prop_array[i].data;
			break;
		case EVA_KMD_PROP_SESSION_DSPMASK:
			session_prop->dsp_mask = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FDU:
			session_prop->cycles[HFI_HW_FDU] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_ICA:
			session_prop->cycles[HFI_HW_ICA] =
				div_by_1dot5(prop_array[i].data);
			break;
		case EVA_KMD_PROP_PWR_OD:
			session_prop->cycles[HFI_HW_OD] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_MPU:
			session_prop->cycles[HFI_HW_MPU] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_VADL:
			session_prop->cycles[HFI_HW_VADL] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_TOF:
			session_prop->cycles[HFI_HW_TOF] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_RGE:
			session_prop->cycles[HFI_HW_RGE] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_XRA:
			session_prop->cycles[HFI_HW_XRA] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_LSR:
			session_prop->cycles[HFI_HW_LSR] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FW:
			session_prop->fw_cycles =
				div_by_1dot5(prop_array[i].data);
			break;
		case EVA_KMD_PROP_PWR_DDR:
			session_prop->ddr_bw = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_SYSCACHE:
			session_prop->ddr_cache = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FDU_OP:
			session_prop->op_cycles[HFI_HW_FDU] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_ICA_OP:
			session_prop->op_cycles[HFI_HW_ICA] =
				div_by_1dot5(prop_array[i].data);
			break;
		case EVA_KMD_PROP_PWR_OD_OP:
			session_prop->op_cycles[HFI_HW_OD] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_MPU_OP:
			session_prop->op_cycles[HFI_HW_MPU] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_VADL_OP:
			session_prop->op_cycles[HFI_HW_VADL] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_TOF_OP:
			session_prop->op_cycles[HFI_HW_TOF] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_RGE_OP:
			session_prop->op_cycles[HFI_HW_RGE] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_XRA_OP:
			session_prop->op_cycles[HFI_HW_XRA] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_LSR_OP:
			session_prop->op_cycles[HFI_HW_LSR] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FW_OP:
			session_prop->fw_op_cycles =
				div_by_1dot5(prop_array[i].data);
			break;
		case EVA_KMD_PROP_PWR_DDR_OP:
			session_prop->ddr_op_bw = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_SYSCACHE_OP:
			session_prop->ddr_op_cache = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_FDU:
			session_prop->fps[HFI_HW_FDU] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_MPU:
			session_prop->fps[HFI_HW_MPU] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_OD:
			session_prop->fps[HFI_HW_OD] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_ICA:
			session_prop->fps[HFI_HW_ICA] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_VADL:
			session_prop->fps[HFI_HW_VADL] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_TOF:
			session_prop->fps[HFI_HW_TOF] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_RGE:
			session_prop->fps[HFI_HW_RGE] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_XRA:
			session_prop->fps[HFI_HW_XRA] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_PWR_FPS_LSR:
			session_prop->fps[HFI_HW_LSR] = prop_array[i].data;
			break;
		case EVA_KMD_PROP_SESSION_DUMPOFFSET:
			session_prop->dump_offset = prop_array[i].data;
			break;
		case EVA_KMD_PROP_SESSION_DUMPSIZE:
			session_prop->dump_size = prop_array[i].data;
			break;
		default:
			dprintk(CVP_ERR,
				"unrecognized sys property to set %d\n",
				prop_array[i].prop_type);
			rc = -EFAULT;
		}
	}
	return rc;
}

static int cvp_drain_fence_sched_list(struct msm_cvp_inst *inst)
{
	unsigned long wait_time;
	struct cvp_fence_queue *q;
	struct cvp_fence_command *f;
	int rc = 0;
	int count = 0, max_count = 0;
	u64 ktid;

	q = &inst->fence_cmd_queue;

	if (!q)
		return -EINVAL;

	if (list_empty(&q->sched_list))
		return rc;

	mutex_lock(&q->lock);
	list_for_each_entry(f, &q->sched_list, list) {
		ktid = f->pkt->client_data.kdata & (FENCE_BIT - 1);
		dprintk(CVP_SYNX, "%s: frame %llu %llu is in sched_list\n",
			__func__, ktid, f->frame_id);
		++count;
	}
	mutex_unlock(&q->lock);
	wait_time = count * 1000;
	wait_time *= inst->core->resources.msm_cvp_hw_rsp_timeout;

	dprintk(CVP_SYNX, "%s: wait %d us for %d fence command\n",
			__func__, wait_time, count);

	count = 0;
	max_count = wait_time / 100;

retry:
	mutex_lock(&q->lock);
	if (list_empty(&q->sched_list)) {
		mutex_unlock(&q->lock);
		return rc;
	}

	mutex_unlock(&q->lock);
	usleep_range(100, 200);
	++count;
	if (count < max_count) {
		goto retry;
	} else {
		rc = -ETIMEDOUT;
		dprintk(CVP_ERR, "%s: timed out!\n", __func__);
	}

	return rc;
}

static void cvp_clean_fence_queue(struct msm_cvp_inst *inst, int synx_state)
{
	struct cvp_fence_queue *q;
	struct cvp_fence_command *f, *d;
	u64 ktid;

	q = &inst->fence_cmd_queue;

	if (!q)
		return;

	mutex_lock(&q->lock);
	q->mode = OP_DRAINING;

	if (list_empty(&q->wait_list))
		goto check_sched;

	list_for_each_entry_safe(f, d, &q->wait_list, list) {
		ktid = f->pkt->client_data.kdata & (FENCE_BIT - 1);

		dprintk(CVP_SYNX, "%s: (%#x) flush frame %llu %llu wait_list\n",
			__func__, hash32_ptr(inst->session), ktid, f->frame_id);

		list_del_init(&f->list);
		msm_cvp_unmap_frame(inst, f->pkt->client_data.kdata);
		inst->core->synx_ftbl->cvp_cancel_synx(inst, CVP_OUTPUT_SYNX,
			f, synx_state);
		inst->core->synx_ftbl->cvp_release_synx(inst, f);
		cvp_free_fence_data(f);
	}

check_sched:
	if (list_empty(&q->sched_list)) {
		mutex_unlock(&q->lock);
		return;
	}

	list_for_each_entry(f, &q->sched_list, list) {
		ktid = f->pkt->client_data.kdata & (FENCE_BIT - 1);

		dprintk(CVP_SYNX, "%s: (%#x)flush frame %llu %llu sched_list\n",
			__func__, hash32_ptr(inst->session), ktid, f->frame_id);
		inst->core->synx_ftbl->cvp_cancel_synx(inst, CVP_INPUT_SYNX,
			f, synx_state);
	}

	mutex_unlock(&q->lock);
}

int cvp_clean_session_queues(struct msm_cvp_inst *inst)
{
	struct cvp_fence_queue *q;
	u32 count = 0, max_retries = 100;

	q = &inst->fence_cmd_queue;
	mutex_lock(&q->lock);
	if (q->state == QUEUE_START || q->state == QUEUE_ACTIVE) {
		mutex_unlock(&q->lock);
		cvp_clean_fence_queue(inst, SYNX_STATE_SIGNALED_CANCEL);
	} else {
		dprintk(CVP_WARN, "Incorrect fence cmd queue state %d\n",
			q->state);
		mutex_unlock(&q->lock);
	}

	cvp_fence_thread_stop(inst);

	/* Waiting for all output synx sent */
retry:
	mutex_lock(&q->lock);
	if (list_empty(&q->sched_list)) {
		mutex_unlock(&q->lock);
		return 0;
	}
	mutex_unlock(&q->lock);
	usleep_range(500, 1000);
	if (++count > max_retries)
		return -EBUSY;

	goto retry;
}

static int cvp_flush_all(struct msm_cvp_inst *inst)
{
	int rc = 0;
	struct msm_cvp_inst *s;
	struct cvp_fence_queue *q;
	struct cvp_hfi_ops *ops_tbl;

	if (!inst || !inst->core) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	s = cvp_get_inst_validate(inst->core, inst);
	if (!s)
		return -ECONNRESET;

	dprintk(CVP_SESS, "session %llx (%#x)flush all starts\n",
			inst, hash32_ptr(inst->session));
	q = &inst->fence_cmd_queue;
	ops_tbl = inst->core->dev_ops;

	cvp_clean_fence_queue(inst, SYNX_STATE_SIGNALED_CANCEL);

	dprintk(CVP_SESS, "%s: (%#x) send flush to fw\n",
			__func__, hash32_ptr(inst->session));

	/* Send flush to FW */
	rc = call_hfi_op(ops_tbl, session_flush, (void *)inst->session);
	if (rc) {
		dprintk(CVP_WARN, "%s: continue flush without fw. rc %d\n",
		__func__, rc);
		goto exit;
	}

	/* Wait for FW response */
	rc = wait_for_sess_signal_receipt(inst, HAL_SESSION_FLUSH_DONE);
	if (rc)
		dprintk(CVP_WARN, "%s: wait for signal failed, rc %d\n",
		__func__, rc);

	dprintk(CVP_SESS, "%s: (%#x) received flush from fw\n",
			__func__, hash32_ptr(inst->session));

exit:
	rc = cvp_drain_fence_sched_list(inst);

	mutex_lock(&q->lock);
	q->mode = OP_NORMAL;
	mutex_unlock(&q->lock);

	cvp_put_inst(s);
	return rc;
}

int msm_cvp_handle_syscall(struct msm_cvp_inst *inst, struct eva_kmd_arg *arg)
{
	int rc = 0;

	if (!inst || !arg) {
		dprintk(CVP_ERR, "%s: invalid args\n", __func__);
		return -EINVAL;
	}
	dprintk(CVP_HFI, "%s: arg->type = %x", __func__, arg->type);

	if (arg->type != EVA_KMD_SESSION_CONTROL &&
		arg->type != EVA_KMD_SET_SYS_PROPERTY &&
		arg->type != EVA_KMD_GET_SYS_PROPERTY) {

		rc = session_state_check_init(inst);
		if (rc) {
			dprintk(CVP_ERR,
				"Incorrect session state %d for command %#x",
				inst->state, arg->type);
			return rc;
		}
	}

	switch (arg->type) {
	case EVA_KMD_GET_SESSION_INFO:
	{
		struct eva_kmd_session_info *session =
			(struct eva_kmd_session_info *)&arg->data.session;

		rc = msm_cvp_get_session_info(inst, &session->session_id);
		break;
	}
	case EVA_KMD_UPDATE_POWER:
	{
		rc = msm_cvp_update_power(inst);
		break;
	}
	case EVA_KMD_REGISTER_BUFFER:
	{
		struct eva_kmd_buffer *buf =
			(struct eva_kmd_buffer *)&arg->data.regbuf;

		rc = msm_cvp_register_buffer(inst, buf);
		break;
	}
	case EVA_KMD_UNREGISTER_BUFFER:
	{
		struct eva_kmd_buffer *buf =
			(struct eva_kmd_buffer *)&arg->data.unregbuf;

		rc = msm_cvp_unregister_buffer(inst, buf);
		break;
	}
	case EVA_KMD_RECEIVE_MSG_PKT:
	{
		struct eva_kmd_hfi_packet *out_pkt =
			(struct eva_kmd_hfi_packet *)&arg->data.hfi_pkt;
		rc = msm_cvp_session_receive_hfi(inst, out_pkt);
		break;
	}
	case EVA_KMD_SEND_CMD_PKT:
	{
		struct eva_kmd_hfi_packet *in_pkt =
			(struct eva_kmd_hfi_packet *)&arg->data.hfi_pkt;

		rc = msm_cvp_session_process_hfi(inst, in_pkt,
				arg->buf_offset, arg->buf_num);
		break;
	}
	case EVA_KMD_SEND_FENCE_CMD_PKT:
	{
		rc = msm_cvp_session_process_hfi_fence(inst, arg);
		break;
	}
	case EVA_KMD_SESSION_CONTROL:
		rc = msm_cvp_session_ctrl(inst, arg);
		break;
	case EVA_KMD_GET_SYS_PROPERTY:
		rc = msm_cvp_get_sysprop(inst, arg);
		break;
	case EVA_KMD_SET_SYS_PROPERTY:
		rc = msm_cvp_set_sysprop(inst, arg);
		break;
	case EVA_KMD_FLUSH_ALL:
		rc = cvp_flush_all(inst);
		break;
	case EVA_KMD_FLUSH_FRAME:
		dprintk(CVP_WARN, "EVA_KMD_FLUSH_FRAME IOCTL deprecated\n");
		rc = 0;
		break;
	default:
		dprintk(CVP_HFI, "%s: unknown arg type %#x\n",
				__func__, arg->type);
		rc = -ENOTSUPP;
		break;
	}

	return rc;
}

int msm_cvp_session_deinit(struct msm_cvp_inst *inst)
{
	int rc = 0;
	struct cvp_hal_session *session;

	if (!inst || !inst->core) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}
	dprintk(CVP_SESS, "%s: inst %pK (%#x)\n", __func__,
		inst, hash32_ptr(inst->session));

	session = (struct cvp_hal_session *)inst->session;
	if (!session || session == (void *)0xdeadbeef)
		return rc;

	rc = msm_cvp_comm_try_state(inst, MSM_CVP_CLOSE_DONE);
	if (rc)
		dprintk(CVP_ERR, "%s: close failed\n", __func__);

	rc = msm_cvp_session_deinit_buffers(inst);
	return rc;
}

int msm_cvp_session_init(struct msm_cvp_inst *inst)
{
	int rc = 0;

	if (!inst) {
		dprintk(CVP_ERR, "%s: invalid params\n", __func__);
		return -EINVAL;
	}

	dprintk(CVP_SESS, "%s: inst %pK (%#x)\n", __func__,
		inst, hash32_ptr(inst->session));

	/* set default frequency */
	inst->clk_data.min_freq = 1000;
	inst->clk_data.ddr_bw = 1000;
	inst->clk_data.sys_cache_bw = 1000;

	inst->prop.type = 1;
	inst->prop.kernel_mask = 0xFFFFFFFF;
	inst->prop.priority = 0;
	inst->prop.is_secure = 0;
	inst->prop.dsp_mask = 0;
	inst->prop.fthread_nr = 3;

	return rc;
}
