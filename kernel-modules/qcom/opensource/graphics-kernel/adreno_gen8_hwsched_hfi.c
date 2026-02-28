// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/iommu.h>
#include <soc/qcom/msm_performance.h>

#include "adreno.h"
#include "adreno_gen8.h"
#include "adreno_gen8_hwsched.h"
#include "adreno_hfi.h"
#include "adreno_pm4types.h"
#include "adreno_trace.h"
#include "kgsl_device.h"
#include "kgsl_eventlog.h"
#include "kgsl_pwrctrl.h"
#include "kgsl_util.h"

#define HFI_QUEUE_MAX (HFI_QUEUE_DEFAULT_CNT)

#define DEFINE_QHDR(gmuaddr, id, prio) \
	{\
		.status = 1, \
		.start_addr = GMU_QUEUE_START_ADDR(gmuaddr, id), \
		.type = QUEUE_HDR_TYPE(id, prio, 0, 0), \
		.queue_size = SZ_4K >> 2, \
		.msg_size = 0, \
		.unused0 = 0, \
		.unused1 = 0, \
		.unused2 = 0, \
		.unused3 = 0, \
		.unused4 = 0, \
		.read_index = 0, \
		.write_index = 0, \
}

struct pending_cmd gen8_hw_fence_ack;

struct gen8_hwsched_hfi *to_gen8_hwsched_hfi(
	struct adreno_device *adreno_dev)
{
	struct gen8_device *gen8_dev = container_of(adreno_dev,
					struct gen8_device, adreno_dev);
	struct gen8_hwsched_device *gen8_hwsched = container_of(gen8_dev,
					struct gen8_hwsched_device, gen8_dev);

	return &gen8_hwsched->hwsched_hfi;
}

int gen8_hfi_send_lpac_feature_ctrl(struct adreno_device *adreno_dev)
{
	if (!adreno_dev->lpac_enabled)
		return 0;

	return gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_LPAC, 1, 0);
}

static void add_waiter(struct gen8_hwsched_hfi *hfi, u32 hdr,
	struct pending_cmd *ack)
{
	memset(ack, 0x0, sizeof(*ack));

	init_completion(&ack->complete);
	write_lock_irq(&hfi->msglock);
	list_add_tail(&ack->node, &hfi->msglist);
	write_unlock_irq(&hfi->msglock);

	ack->sent_hdr = hdr;
}

static void del_waiter(struct gen8_hwsched_hfi *hfi, struct pending_cmd *ack)
{
	write_lock_irq(&hfi->msglock);
	list_del(&ack->node);
	write_unlock_irq(&hfi->msglock);
}

static void gen8_receive_ack_async(struct adreno_device *adreno_dev, void *rcvd)
{
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct pending_cmd *cmd = NULL;
	u32 waiters[64], num_waiters = 0, i;
	u32 *ack = rcvd;
	u32 hdr = ack[0];
	u32 req_hdr = ack[1];
	u32 size_bytes = MSG_HDR_GET_SIZE(hdr) << 2;

	if (size_bytes > sizeof(cmd->results))
		dev_err_ratelimited(gmu_pdev_dev,
			"Ack result too big: %d Truncating to: %ld\n",
			size_bytes, sizeof(cmd->results));

	read_lock(&hfi->msglock);

	list_for_each_entry(cmd, &hfi->msglist, node) {
		if (CMP_HFI_ACK_HDR(cmd->sent_hdr, req_hdr)) {
			memcpy(cmd->results, ack,
				min_t(u32, size_bytes,
					sizeof(cmd->results)));
			complete(&cmd->complete);
			read_unlock(&hfi->msglock);
			return;
		}

		if (num_waiters < ARRAY_SIZE(waiters))
			waiters[num_waiters++] = cmd->sent_hdr;
	}

	read_unlock(&hfi->msglock);

	/* Didn't find the sender, list the waiter */
	dev_err_ratelimited(gmu_pdev_dev,
		"Unexpectedly got id %d seqnum %d. Total waiters: %d Top %d Waiters:\n",
		MSG_HDR_GET_ID(req_hdr), MSG_HDR_GET_SEQNUM(req_hdr),
		num_waiters, min_t(u32, num_waiters, 5));

	for (i = 0; i < num_waiters && i < 5; i++)
		dev_err_ratelimited(gmu_pdev_dev,
			" id %d seqnum %d\n",
			MSG_HDR_GET_ID(waiters[i]),
			MSG_HDR_GET_SEQNUM(waiters[i]));
}

static void _retire_inflight_hw_fences(struct adreno_device *adreno_dev,
	struct kgsl_context *context)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct adreno_hw_fence_entry *entry, *tmp;

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &adreno_dev->hwsched.flags))
		return;

	spin_lock(&drawctxt->lock);

	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_inflight_list, node) {
		struct gmu_context_queue_header *hdr = drawctxt->gmu_context_queue.hostptr;

		/*
		 * Since this list is sorted by timestamp, abort on the first fence that hasn't
		 * yet been sent to TxQueue
		 */
		if (timestamp_cmp((u32)entry->cmd.ts, hdr->out_fence_ts) > 0)
			break;

		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}
	spin_unlock(&drawctxt->lock);
}

static void log_profiling_info(struct adreno_device *adreno_dev, u32 *rcvd)
{
	struct hfi_ts_retire_cmd *cmd = (struct hfi_ts_retire_cmd *)rcvd;
	struct kgsl_context *context;
	struct retire_info info = {0};
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	context = kgsl_context_get(device, cmd->ctxt_id);
	if (context == NULL)
		return;

	info.timestamp = cmd->ts;
	info.rb_id = adreno_get_level(context);
	info.gmu_dispatch_queue = context->gmu_dispatch_queue;
	info.submitted_to_rb = cmd->submitted_to_rb;
	info.sop = cmd->sop;
	info.eop = cmd->eop;
	if (GMU_VER_MINOR(gmu->ver.hfi) < 4)
		info.active = cmd->eop - cmd->sop;
	else
		info.active = cmd->active;
	info.retired_on_gmu = cmd->retired_on_gmu;

	/* protected GPU work must not be reported */
	if  (!(context->flags & KGSL_CONTEXT_SECURE))
		kgsl_work_period_update(device, context->proc_priv->period,
					     info.active);

	trace_adreno_cmdbatch_retired(context, &info, 0, 0, 0);

	log_kgsl_cmdbatch_retired_event(context->id, cmd->ts,
		context->priority, 0, cmd->sop, cmd->eop);

	_retire_inflight_hw_fences(adreno_dev, context);

	kgsl_context_put(context);
}

struct syncobj_flags {
	unsigned long mask;
	const char *name;
};

static void _get_syncobj_string(char *str, u32 max_size, struct hfi_syncobj *syncobj, u32 index)
{
	u32 count = scnprintf(str, max_size, "syncobj[%d] ctxt_id:%llu seqno:%llu flags:", index,
			syncobj->ctxt_id, syncobj->seq_no);
	u32 i;
	bool first = true;
	static const struct syncobj_flags _flags[] = {
		GMU_SYNCOBJ_FLAGS, { -1, NULL }};

	for (i = 0; _flags[i].name; i++) {
		if (!(syncobj->flags & _flags[i].mask))
			continue;

		if (first) {
			count += scnprintf(str + count, max_size - count, "%s", _flags[i].name);
			first = false;
		} else {
			count += scnprintf(str + count, max_size - count, "|%s", _flags[i].name);
		}
	}
}

static void log_syncobj(struct gen8_gmu_device *gmu, struct adreno_context *drawctxt,
		struct hfi_submit_syncobj *cmd, u32 syncobj_read_idx)
{
	struct kgsl_device *device = KGSL_DEVICE(gen8_gmu_to_adreno(gmu));
	struct gmu_context_queue_header *hdr = drawctxt->gmu_context_queue.hostptr;
	struct hfi_syncobj syncobj;
	char str[128];
	u32 i = 0;

	for (i = 0; i < cmd->num_syncobj; i++) {
		if (adreno_gmu_context_queue_read(drawctxt, (u32 *) &syncobj, syncobj_read_idx,
			sizeof(syncobj) >> 2))
			break;

		_get_syncobj_string(str, sizeof(str), &syncobj, i);
		dev_err(GMU_PDEV_DEV(device), "%s\n", str);
		syncobj_read_idx = (syncobj_read_idx + (sizeof(syncobj) >> 2)) % hdr->queue_size;
	}
}

static void find_timeout_syncobj(struct adreno_device *adreno_dev, u32 ctxt_id, u32 ts)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct kgsl_context *context = NULL;
	struct adreno_context *drawctxt;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gmu_context_queue_header *hdr;
	struct hfi_submit_syncobj cmd;
	u32 *queue, i;
	int ret;

	/* We want to get the context even if it is detached */
	read_lock(&device->context_lock);
	context = idr_find(&device->context_idr, ctxt_id);
	ret = _kgsl_context_get(context);
	read_unlock(&device->context_lock);

	if (!ret)
		return;

	drawctxt = ADRENO_CONTEXT(context);

	hdr = drawctxt->gmu_context_queue.hostptr;
	queue = (u32 *)(drawctxt->gmu_context_queue.hostptr + sizeof(*hdr));

	for (i = hdr->read_index; i != hdr->write_index;) {
		if (MSG_HDR_GET_ID(queue[i]) != H2F_MSG_ISSUE_SYNCOBJ) {
			i = (i + MSG_HDR_GET_SIZE(queue[i])) % hdr->queue_size;
			continue;
		}

		if (adreno_gmu_context_queue_read(drawctxt, (u32 *) &cmd, i, sizeof(cmd) >> 2))
			break;

		if (cmd.timestamp == ts) {
			log_syncobj(gmu, drawctxt, &cmd,
				(i + (sizeof(cmd) >> 2)) % hdr->queue_size);
			break;
		}
		i = (i + MSG_HDR_GET_SIZE(queue[i])) % hdr->queue_size;
	}

	if (i == hdr->write_index)
		dev_err(GMU_PDEV_DEV(device), "Couldn't find unsignaled syncobj ctx:%d ts:%d\n",
			ctxt_id, ts);

	kgsl_context_put(context);
}

/* Look up a particular key's value for a given type of payload */
static u32 gen8_hwsched_lookup_key_value(struct adreno_device *adreno_dev,
	u32 type, u32 key)
{
	struct hfi_context_bad_cmd *cmd = adreno_dev->hwsched.ctxt_bad;
	u32 i = 0, payload_bytes;
	void *start;

	if (!cmd->hdr)
		return 0;

	payload_bytes = (MSG_HDR_GET_SIZE(cmd->hdr) << 2) -
			offsetof(struct hfi_context_bad_cmd, payload);

	start = &cmd->payload[0];

	while (i < payload_bytes) {
		struct payload_section *payload = start + i;

		if (payload->type == type)
			return adreno_hwsched_parse_payload(payload, key);

		i += struct_size(payload, data, payload->dwords);
	}

	return 0;
}

static u32 get_payload_rb_key(struct adreno_device *adreno_dev,
	u32 rb_id, u32 key)
{
	struct hfi_context_bad_cmd *cmd = adreno_dev->hwsched.ctxt_bad;
	u32 i = 0, payload_bytes;
	void *start;

	if (!cmd->hdr)
		return 0;

	payload_bytes = (MSG_HDR_GET_SIZE(cmd->hdr) << 2) -
			offsetof(struct hfi_context_bad_cmd, payload);

	start = &cmd->payload[0];

	while (i < payload_bytes) {
		struct payload_section *payload = start + i;

		if (payload->type == PAYLOAD_RB) {
			u32 id = adreno_hwsched_parse_payload(payload, KEY_RB_ID);

			if (id == rb_id)
				return adreno_hwsched_parse_payload(payload, key);
		}

		i += struct_size(payload, data, payload->dwords);
	}

	return 0;
}

static bool log_gpu_fault(struct adreno_device *adreno_dev)
{
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	struct hfi_context_bad_cmd *cmd = adreno_dev->hwsched.ctxt_bad;

	/* Return false for non fatal errors */
	if (adreno_hwsched_log_nonfatal_gpu_fault(adreno_dev, gmu_pdev_dev, cmd->error))
		return false;

	switch (cmd->error) {
	case GMU_GPU_HW_HANG:
		dev_crit_ratelimited(gmu_pdev_dev, "MISC: GPU hang detected\n");
		break;
	case GMU_GPU_SW_HANG:
		dev_crit_ratelimited(gmu_pdev_dev, "gpu timeout ctx %d ts %d\n",
			cmd->gc.ctxt_id, cmd->gc.ts);
		break;
	case GMU_CP_OPCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP opcode error interrupt | opcode=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
			KEY_CP_OPCODE_ERROR));
		break;
	case GMU_CP_PROTECTED_ERROR: {
		u32 status = gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_PROTECTED_ERROR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"CP | Protected mode error | %s | addr=0x%5.5x | status=0x%8.8x\n",
			status & (1 << 20) ? "READ" : "WRITE",
			status & 0x3FFFF, status);
		}
		break;
	case GMU_CP_ILLEGAL_INST_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP Illegal instruction error\n");
		break;
	case GMU_CP_UCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP ucode error interrupt\n");
		break;
	case GMU_CP_HW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP | Ringbuffer HW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_HW_FAULT));
		break;
	case GMU_GPU_PREEMPT_TIMEOUT: {
		u32 cur, next, cur_rptr, cur_wptr, next_rptr, next_wptr;

		cur = gen8_hwsched_lookup_key_value(adreno_dev,
			PAYLOAD_PREEMPT_TIMEOUT, KEY_PREEMPT_TIMEOUT_CUR_RB_ID);
		next = gen8_hwsched_lookup_key_value(adreno_dev,
			PAYLOAD_PREEMPT_TIMEOUT,
			KEY_PREEMPT_TIMEOUT_NEXT_RB_ID);
		cur_rptr = get_payload_rb_key(adreno_dev, cur, KEY_RB_RPTR);
		cur_wptr = get_payload_rb_key(adreno_dev, cur, KEY_RB_WPTR);
		next_rptr = get_payload_rb_key(adreno_dev, next, KEY_RB_RPTR);
		next_wptr = get_payload_rb_key(adreno_dev, next, KEY_RB_WPTR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"Preemption Fault: cur=%d R/W=0x%x/0x%x, next=%d R/W=0x%x/0x%x\n",
			cur, cur_rptr, cur_wptr, next, next_rptr, next_wptr);
		}
		break;
	case GMU_CP_GPC_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "RBBM: GPC error\n");
		break;
	case GMU_CP_BV_OPCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP BV opcode error | opcode=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
			KEY_CP_BV_OPCODE_ERROR));
		break;
	case GMU_CP_BV_PROTECTED_ERROR: {
		u32 status = gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_BV_PROTECTED_ERROR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"CP BV | Protected mode error | %s | addr=0x%5.5x | status=0x%8.8x\n",
			status & (1 << 20) ? "READ" : "WRITE",
			status & 0x3FFFF, status);
		}
		break;
	case GMU_CP_BV_HW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP BV | Ringbuffer HW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_BV_HW_FAULT));
		break;
	case GMU_CP_BV_ILLEGAL_INST_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP BV Illegal instruction error\n");
		break;
	case GMU_CP_BV_UCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP BV ucode error interrupt\n");
		break;
	case GMU_CP_LPAC_OPCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP LPAC opcode error | opcode=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
			KEY_CP_LPAC_OPCODE_ERROR));
		break;
	case GMU_CP_LPAC_PROTECTED_ERROR: {
		u32 status = gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_LPAC_PROTECTED_ERROR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"CP LPAC | Protected mode error | %s | addr=0x%5.5x | status=0x%8.8x\n",
			status & (1 << 20) ? "READ" : "WRITE",
			status & 0x3FFFF, status);
		}
		break;
	case GMU_CP_LPAC_HW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP LPAC | Ringbuffer HW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_LPAC_HW_FAULT));
		break;
	case GMU_CP_LPAC_ILLEGAL_INST_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP LPAC Illegal instruction error\n");
		break;
	case GMU_CP_LPAC_UCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP LPAC ucode error interrupt\n");
		break;
	case GMU_GPU_LPAC_SW_HANG:
		dev_crit_ratelimited(gmu_pdev_dev, "LPAC: gpu timeout ctx %d ts %d\n",
			cmd->lpac.ctxt_id, cmd->lpac.ts);
		break;
	case GMU_GPU_SW_FUSE_VIOLATION:
		dev_crit_ratelimited(gmu_pdev_dev, "RBBM: SW Feature Fuse violation status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_SWFUSE_VIOLATION_FAULT));
		break;
	case GMU_GPU_AQE0_OPCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE0 opcode error | opcode=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev,
				PAYLOAD_FAULT_REGS, KEY_AQE0_OPCODE_ERROR));
		break;
	case GMU_GPU_AQE0_UCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE0 ucode error interrupt\n");
		break;
	case GMU_GPU_AQE0_HW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE0 HW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev,
				PAYLOAD_FAULT_REGS, KEY_AQE0_HW_FAULT));
		break;
	case GMU_GPU_AQE0_ILLEGAL_INST_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE0 Illegal instruction error\n");
		break;
	case GMU_GPU_AQE1_OPCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE1 opcode error | opcode=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev,
				PAYLOAD_FAULT_REGS, KEY_AQE1_OPCODE_ERROR));
		break;
	case GMU_GPU_AQE1_UCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE1 ucode error interrupt\n");
		break;
	case GMU_GPU_AQE1_HW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE1 HW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev,
				PAYLOAD_FAULT_REGS, KEY_AQE1_HW_FAULT));
		break;
	case GMU_GPU_AQE1_ILLEGAL_INST_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "AQE1 Illegal instruction error\n");
		break;
	case GMU_SYNCOBJ_TIMEOUT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "syncobj timeout ctx %d ts %u\n",
			cmd->gc.ctxt_id, cmd->gc.ts);
		find_timeout_syncobj(adreno_dev, cmd->gc.ctxt_id, cmd->gc.ts);
		break;
	case GMU_CP_DDEBR_HW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BR | Ringbuffer HW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_DDEBR_HW_FAULT));
		break;
	case GMU_CP_DDEBR_OPCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BR opcode error | opcode=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
			KEY_CP_DDEBR_OPCODE_ERROR));
		break;
	case GMU_CP_DDEBR_UCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP DDE BR ucode error\n");
		break;
	case GMU_CP_DDEBR_PROTECTED_ERROR: {
		u32 status = gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_DDEBR_PROTECTED_ERROR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BR | Protected mode error | %s | addr=0x%5.5x | status=0x%8.8x\n",
			status & (1 << 20) ? "READ" : "WRITE",
			status & 0x3FFFF, status);
		}
		break;
	case GMU_CP_DDEBR_ILLEGAL_INST_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP DDEBR Illegal instruction error\n");
		break;
	case GMU_CP_DDEBV_HW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BV | Ringbuffer HW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_DDEBV_HW_FAULT));
		break;
	case GMU_CP_DDEBV_OPCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BV opcode error | opcode=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
			KEY_CP_DDEBV_OPCODE_ERROR));
		break;
	case GMU_CP_DDEBV_UCODE_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP DDE BV ucode error\n");
		break;
	case GMU_CP_DDEBV_PROTECTED_ERROR: {
		u32 status = gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_DDEBV_PROTECTED_ERROR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BV | Protected mode error | %s | addr=0x%5.5x | status=0x%8.8x\n",
			status & (1 << 20) ? "READ" : "WRITE",
			status & 0x3FFFF, status);
		}
		break;
	case GMU_CP_DDEBV_ILLEGAL_INST_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev, "CP DDE BV Illegal instruction error\n");
		break;
	case GMU_CP_BR_SW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP BR | SW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_BR_SW_FAULT));
		break;
	case GMU_CP_BV_SW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP BV | SW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_BV_SW_FAULT));
		break;
	case GMU_CP_LPAC_SW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP LPAC | SW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_LPAC_SW_FAULT));
		break;
	case GMU_CP_AQE0_SW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP AQE0 | SW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_AQE0_SW_FAULT));
		break;
	case GMU_CP_AQE1_SW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP AQE1 | SW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_AQE1_SW_FAULT));
		break;
	case GMU_CP_AQE0_PROTECTED_ERROR: {
		u32 status = gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_AQE0_PROTECTED_ERROR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"CP AQE0 | Protected mode error | %s | addr=0x%5.5x | status=0x%8.8x\n",
			status & (1 << 20) ? "READ" : "WRITE",
			status & 0x3FFFF, status);
		}
		break;
	case GMU_CP_AQE1_PROTECTED_ERROR: {
		u32 status = gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_AQE1_PROTECTED_ERROR);

		dev_crit_ratelimited(gmu_pdev_dev,
			"CP AQE1 | Protected mode error | %s | addr=0x%5.5x | status=0x%8.8x\n",
			status & (1 << 20) ? "READ" : "WRITE",
			status & 0x3FFFF, status);
		}
		break;
	case GMU_CP_DDEBR_SW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BR | SW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_DDEBR_SW_FAULT));
		break;
	case GMU_CP_DDEBV_SW_FAULT_ERROR:
		dev_crit_ratelimited(gmu_pdev_dev,
			"CP DDE BV | SW fault | status=0x%8.8x\n",
			gen8_hwsched_lookup_key_value(adreno_dev, PAYLOAD_FAULT_REGS,
				KEY_CP_DDEBV_SW_FAULT));
		break;
	case GMU_CP_UNKNOWN_ERROR:
		fallthrough;
	default:
		dev_crit_ratelimited(gmu_pdev_dev, "Unknown GPU fault: %u\n",
			cmd->error);
		break;
	}

	/* Return true for fatal errors to perform recovery sequence */
	return true;
}

static bool is_queue_empty(struct adreno_device *adreno_dev, u32 queue_idx)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct kgsl_memdesc *mem_addr = gmu->hfi.hfi_mem;
	struct hfi_queue_table *tbl = mem_addr->hostptr;
	struct hfi_queue_header *hdr = &tbl->qhdr[queue_idx];

	if (hdr->status == HFI_QUEUE_STATUS_DISABLED)
		return true;

	if (hdr->read_index == hdr->write_index)
		return true;

	/*
	 * This is to ensure that the queue is not read speculatively before the queue empty
	 * condition is evaluated
	 */
	rmb();

	return false;
}

static u32 peek_next_header(struct adreno_device *adreno_dev, struct gen8_gmu_device *gmu,
	u32 queue_idx)
{
	struct kgsl_memdesc *mem_addr = gmu->hfi.hfi_mem;
	struct hfi_queue_table *tbl = mem_addr->hostptr;
	struct hfi_queue_header *hdr = &tbl->qhdr[queue_idx];
	u32 *queue;

	if (is_queue_empty(adreno_dev, queue_idx))
		return 0;

	queue = HOST_QUEUE_START_ADDR(mem_addr, queue_idx);

	return queue[hdr->read_index];
}

static void process_ctx_bad(struct adreno_device *adreno_dev)
{
	/* Non fatal RBBM error interrupts don't go through reset and recovery */
	if (!log_gpu_fault(adreno_dev)) {
		memset(adreno_dev->hwsched.ctxt_bad, 0x0, HFI_MAX_MSG_SIZE);
		return;
	}

	gen8_hwsched_fault(adreno_dev, ADRENO_HARD_FAULT);
}

#define GET_QUERIED_FENCE_INDEX(x) (x / BITS_PER_SYNCOBJ_QUERY)
#define GET_QUERIED_FENCE_BIT(x) (x % BITS_PER_SYNCOBJ_QUERY)

static bool fence_is_queried(struct hfi_syncobj_query_cmd *cmd, u32 fence_index)
{
	u32 index = GET_QUERIED_FENCE_INDEX(fence_index);
	u32 bit = GET_QUERIED_FENCE_BIT(fence_index);

	return (cmd->queries[index].query_bitmask & BIT(bit));
}

static void set_fence_signal_bit(struct adreno_device *adreno_dev,
	struct hfi_syncobj_query_cmd *reply, struct dma_fence *fence, u32 fence_index)
{
	u32 index = GET_QUERIED_FENCE_INDEX(fence_index);
	u32 bit = GET_QUERIED_FENCE_BIT(fence_index);
	u64 flags = ADRENO_HW_FENCE_SW_STATUS_PENDING;
	char name[KGSL_FENCE_NAME_LEN];
	char value[32] = "unknown";

	if (fence->ops->timeline_value_str)
		fence->ops->timeline_value_str(fence, value, sizeof(value));

	if (test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &fence->flags)) {
		dev_err(GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev)),
			"GMU is waiting for signaled fence(ctx:%llu seqno:%llu value:%s)\n",
			fence->context, fence->seqno, value);
		reply->queries[index].query_bitmask |= BIT(bit);
		flags = ADRENO_HW_FENCE_SW_STATUS_SIGNALED;
	}
	kgsl_get_fence_name(fence, name, sizeof(name));

	trace_adreno_hw_fence_query(fence->context, fence->seqno, flags, name, value);
}

static void gen8_syncobj_query_reply(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj, struct hfi_syncobj_query_cmd *cmd)
{
	struct hfi_syncobj_query_cmd reply = {0};
	int i;
	struct kgsl_drawobj_sync *syncobj = SYNCOBJ(drawobj);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	for (i = 0; i < syncobj->num_hw_fence; i++) {
		struct dma_fence *fence = syncobj->hw_fences[i].fence;

		if (!fence_is_queried(cmd, i))
			continue;

		set_fence_signal_bit(adreno_dev, &reply, fence, i);
	}

	reply.hdr = CREATE_MSG_HDR(F2H_MSG_SYNCOBJ_QUERY, HFI_MSG_CMD);
	reply.gmu_ctxt_id = cmd->gmu_ctxt_id;
	reply.sync_obj_ts = cmd->sync_obj_ts;

	trace_adreno_syncobj_query_reply(reply.gmu_ctxt_id, reply.sync_obj_ts,
		gpudev->read_alwayson(adreno_dev));

	gen8_hfi_send_cmd_async(adreno_dev, &reply, sizeof(reply));
}

struct syncobj_query_work {
	/** @cmd: The query command to be processed */
	struct hfi_syncobj_query_cmd cmd;
	/** @context: kgsl context that is waiting for this sync object */
	struct kgsl_context *context;
	/** @work: The work structure to execute syncobj query reply */
	struct kthread_work work;
};

static void gen8_process_syncobj_query_work(struct kthread_work *work)
{
	struct syncobj_query_work *query_work = container_of(work,
						struct syncobj_query_work, work);
	struct hfi_syncobj_query_cmd *cmd = (struct hfi_syncobj_query_cmd *)&query_work->cmd;
	struct kgsl_context *context = query_work->context;
	struct kgsl_device *device = context->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct cmd_list_obj *obj;
	bool missing = true;

	mutex_lock(&hwsched->mutex);
	mutex_lock(&device->mutex);

	list_for_each_entry(obj, &hwsched->cmd_list, node) {
		struct kgsl_drawobj *drawobj = obj->drawobj;

		if ((drawobj->type & SYNCOBJ_TYPE) == 0)
			continue;

		if ((drawobj->context->id == cmd->gmu_ctxt_id) &&
			(drawobj->timestamp == cmd->sync_obj_ts)) {
			gen8_syncobj_query_reply(adreno_dev, drawobj, cmd);
			missing = false;
			break;
		}
	}

	if (missing) {
		struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
		struct gmu_context_queue_header *hdr = drawctxt->gmu_context_queue.hostptr;

		/*
		 * If the sync object is not found, it can only mean that the sync object was
		 * retired by the GMU in the meanwhile. However, if that is not the case, then
		 * we have a problem.
		 */
		if (timestamp_cmp(cmd->sync_obj_ts, hdr->sync_obj_ts) > 0) {
			dev_err(GMU_PDEV_DEV(device),
				"Missing sync object ctx:%d ts:%d retired:%d\n",
				context->id, cmd->sync_obj_ts, hdr->sync_obj_ts);
			gmu_core_fault_snapshot(device, GMU_FAULT_HW_FENCE);
			gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
		}
	}

	mutex_unlock(&device->mutex);
	mutex_unlock(&hwsched->mutex);

	kgsl_context_put(context);
	kfree(query_work);
}

static void gen8_trigger_syncobj_query(struct adreno_device *adreno_dev,
	u32 *rcvd)
{
	struct syncobj_query_work *query_work;
	struct hfi_syncobj_query_cmd *cmd = (struct hfi_syncobj_query_cmd *)rcvd;
	struct kgsl_context *context = NULL;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int ret;

	trace_adreno_syncobj_query(cmd->gmu_ctxt_id, cmd->sync_obj_ts,
		gpudev->read_alwayson(adreno_dev));

	/*
	 * We need the context even if it is detached. Hence, we can't use kgsl_context_get here.
	 * We must make sure that this context id doesn't get destroyed (to avoid re-use) until GMU
	 * has ack'd the query reply.
	 */
	read_lock(&device->context_lock);
	context = idr_find(&device->context_idr, cmd->gmu_ctxt_id);
	ret = _kgsl_context_get(context);
	read_unlock(&device->context_lock);

	if (!ret)
		return;

	query_work = kzalloc(sizeof(*query_work), GFP_KERNEL);
	if (!query_work) {
		kgsl_context_put(context);
		return;
	}

	kthread_init_work(&query_work->work, gen8_process_syncobj_query_work);
	memcpy(&query_work->cmd, cmd, sizeof(*cmd));
	query_work->context = context;

	kthread_queue_work(adreno_dev->scheduler_worker, &query_work->work);
}

/*
 * This defines the maximum unack'd hardware fences that we allow. When this limit is reached, we
 * will put all threads (that want to create a hardware fence) to sleep until the maximum unack'd
 * hardware fence count drops to MIN_HW_FENCE_UNACK_COUNT
 */
#define MAX_HW_FENCE_UNACK_COUNT 20

/*
 * Once the maximum unack'd hardware fences drops to this value, wake up all the threads (that want
 * to create hardware fences)
 */
#define MIN_HW_FENCE_UNACK_COUNT 10

/*
 * This is the maximum duration (in milliseconds) a thread (that wants to create a hardware fence)
 * is put to sleep while we wait for the maximum number of unack'd hardware fences to drop from
 * MAX_HW_FENCE_UNACK_COUNT to MIN_HW_FENCE_UNACK_COUNT. If the count doesn't drop to the desired
 * value, then log an error and trigger snapshot and recovery.
 */
#define HW_FENCE_SLEEP_MS 200

static void _enable_hw_fence_throttle(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);

	set_bit(GEN8_HWSCHED_HW_FENCE_SLEEP_BIT, &hfi->hw_fence.flags);
	set_bit(GEN8_HWSCHED_HW_FENCE_MAX_BIT, &hfi->hw_fence.flags);

	/* Avoid submitting new work to gpu until the unack count drops to a desired threshold */
	adreno_get_gpu_halt(adreno_dev);

	mod_timer(&hfi->hw_fence_timer, jiffies + msecs_to_jiffies(HW_FENCE_SLEEP_MS));
}

static void _increment_hw_fence_unack_count(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);

	if ((++hfi->hw_fence.unack_count) == MAX_HW_FENCE_UNACK_COUNT)
		_enable_hw_fence_throttle(adreno_dev);
}

/**
 * _send_hw_fence_no_ack - Send a hardware fence hfi packet to GMU without waiting for its ack.
 * Increment the unack count on success
 *
 * Return: 0 on success or negative error on failure
 */
static int _send_hw_fence_no_ack(struct adreno_device *adreno_dev,
	struct adreno_hw_fence_entry *entry)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	u32 seqnum;
	int ret;

	seqnum = atomic_inc_return(&hfi->hw_fence.seqnum);
	entry->cmd.hdr = MSG_HDR_SET_SEQNUM_SIZE(entry->cmd.hdr, seqnum, sizeof(entry->cmd) >> 2);

	ret = gen8_hfi_cmdq_write(adreno_dev, (u32 *)&entry->cmd, sizeof(entry->cmd));
	if (!ret)
		_increment_hw_fence_unack_count(adreno_dev);

	return ret;
}

static struct adreno_hw_fence_entry *_get_deferred_hw_fence(struct adreno_context *drawctxt, u32 ts)
{
	struct adreno_hw_fence_entry *entry = NULL, *next, *deferred_hw_fence_entry = NULL;

	spin_lock(&drawctxt->lock);
	list_for_each_entry_safe(entry, next, &drawctxt->hw_fence_list, node) {

		if (timestamp_cmp((u32)entry->cmd.ts, ts) > 0)
			break;

		/* We found a deferred hardware fence */
		deferred_hw_fence_entry = entry;
		break;
	}
	spin_unlock(&drawctxt->lock);

	/*
	 * This path executes in isolation from any paths that may release this entry. So, it is
	 * safe to handle this entry outside of the drawctxt spinlock
	 */
	return deferred_hw_fence_entry;
}

static int _send_deferred_hw_fence(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, struct adreno_hw_fence_entry *entry, u32 ts)
{
	bool retired = kgsl_check_timestamp(KGSL_DEVICE(adreno_dev), &drawctxt->base, ts) ||
				kgsl_context_is_bad(&drawctxt->base);
	int ret = 0;
	u32 flags = 0;

	if (retired)
		flags |= HW_FENCE_FLAG_SKIP_MEMSTORE;

	ret = gen8_send_hw_fence_hfi_wait_ack(adreno_dev, entry, flags);
	if (ret)
		return ret;

	spin_lock(&drawctxt->lock);
	if (!retired)
		list_move_tail(&entry->node, &drawctxt->hw_fence_inflight_list);
	else
		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	spin_unlock(&drawctxt->lock);

	return 0;
}

/**
 * process_hw_fence_deferred_ctxt - This function sends hardware fences to GMU (from the
 * deferred drawctxt) which couldn't be sent earlier
 */
static int process_hw_fence_deferred_ctxt(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, u32 ts)
{
	struct adreno_hw_fence_entry *deferred_hw_fence_entry = NULL;
	int ret = 0;

	do {
		deferred_hw_fence_entry = _get_deferred_hw_fence(drawctxt, ts);

		if (!deferred_hw_fence_entry)
			break;

		ret = _send_deferred_hw_fence(adreno_dev, drawctxt, deferred_hw_fence_entry, ts);
		if (ret)
			break;

	} while (deferred_hw_fence_entry != NULL);

	return ret;
}

static void _disable_hw_fence_throttle(struct adreno_device *adreno_dev, bool clear_abort_bit)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	bool max;

	spin_lock(&hfi->hw_fence.lock);

	hfi->hw_fence.defer_drawctxt = NULL;
	hfi->hw_fence.defer_ts = 0;
	max = test_bit(GEN8_HWSCHED_HW_FENCE_MAX_BIT, &hfi->hw_fence.flags);
	if (max) {
		clear_bit(GEN8_HWSCHED_HW_FENCE_SLEEP_BIT, &hfi->hw_fence.flags);
		clear_bit(GEN8_HWSCHED_HW_FENCE_MAX_BIT, &hfi->hw_fence.flags);
	}

	if (clear_abort_bit)
		clear_bit(GEN8_HWSCHED_HW_FENCE_ABORT_BIT, &hfi->hw_fence.flags);
	spin_unlock(&hfi->hw_fence.lock);

	/* Wake up dispatcher and any sleeping threads that want to create hardware fences */
	if (max) {
		adreno_put_gpu_halt(adreno_dev);
		adreno_scheduler_queue(adreno_dev);
		wake_up_all(&hfi->hw_fence.unack_wq);
	}
}

static void gen8_defer_hw_fence_work(struct kthread_work *work)
{
	struct gen8_hwsched_hfi *hfi = container_of(work,
						struct gen8_hwsched_hfi, defer_hw_fence_work);
	struct gen8_hwsched_device *gen8_hwsched = container_of(hfi, struct gen8_hwsched_device,
							hwsched_hfi);
	struct adreno_context *drawctxt;
	struct adreno_device *adreno_dev = &gen8_hwsched->gen8_dev.adreno_dev;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 ts;
	int ret;

	/*
	 * Grab the dispatcher and device mutex as we don't want to race with concurrent fault
	 * recovery
	 */
	mutex_lock(&adreno_dev->hwsched.mutex);
	mutex_lock(&device->mutex);

	spin_lock(&hfi->hw_fence.lock);
	drawctxt = hfi->hw_fence.defer_drawctxt;
	ts = hfi->hw_fence.defer_ts;
	spin_unlock(&hfi->hw_fence.lock);

	if (!drawctxt)
		goto unlock;

	ret = process_hw_fence_deferred_ctxt(adreno_dev, drawctxt, ts);
	if (ret) {
		/* the deferred drawctxt will be handled post fault recovery */
		gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
		goto unlock;
	}

	/*
	 * Put back the context reference which was incremented when hw_fence.defer_drawctxt was set
	 */
	kgsl_context_put(&drawctxt->base);

	adreno_active_count_put(adreno_dev);

	_disable_hw_fence_throttle(adreno_dev, false);

unlock:
	mutex_unlock(&device->mutex);
	mutex_unlock(&adreno_dev->hwsched.mutex);
}

static void process_hw_fence_ack(struct adreno_device *adreno_dev, u32 received_hdr)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct adreno_context *drawctxt = NULL;

	spin_lock(&hfi->hw_fence.lock);

	/* If this ack is being waited on, we don't need to touch the unack count */
	if (gen8_hw_fence_ack.sent_hdr &&
	    CMP_HFI_ACK_HDR(gen8_hw_fence_ack.sent_hdr, received_hdr)) {
		spin_unlock(&hfi->hw_fence.lock);
		complete(&gen8_hw_fence_ack.complete);
		return;
	}

	hfi->hw_fence.unack_count--;

	/* The unack count should never be greater than MAX_HW_FENCE_UNACK_COUNT */
	if (hfi->hw_fence.unack_count > MAX_HW_FENCE_UNACK_COUNT)
		dev_err(GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev)),
			"unexpected hardware fence unack count:%d\n",
			hfi->hw_fence.unack_count);

	if (!test_bit(GEN8_HWSCHED_HW_FENCE_MAX_BIT, &hfi->hw_fence.flags) ||
		(hfi->hw_fence.unack_count != MIN_HW_FENCE_UNACK_COUNT)) {
		spin_unlock(&hfi->hw_fence.lock);
		return;
	}

	drawctxt = hfi->hw_fence.defer_drawctxt;

	spin_unlock(&hfi->hw_fence.lock);

	del_timer_sync(&hfi->hw_fence_timer);

	/*
	 * We need to handle the deferred context in another thread so that we can unblock the f2h
	 * daemon here as it will need to process the acks for the hardware fences belonging to the
	 * deferred context
	 */
	if (drawctxt) {
		kthread_init_work(&hfi->defer_hw_fence_work, gen8_defer_hw_fence_work);
		kthread_queue_work(adreno_dev->scheduler_worker, &hfi->defer_hw_fence_work);
		return;
	}

	_disable_hw_fence_throttle(adreno_dev, false);
}

void gen8_hwsched_process_msgq(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);
	u32 rcvd[MAX_RCVD_SIZE], next_hdr, type;

	if (!(hw_hfi->irq_mask & HFI_IRQ_MSGQ_MASK))
		return;

	mutex_lock(&hw_hfi->msgq_mutex);

	for (;;) {
		next_hdr = peek_next_header(adreno_dev, gmu, HFI_MSG_ID);

		if (!next_hdr)
			break;

		if (MSG_HDR_GET_TYPE(next_hdr) == HFI_MSG_ACK)
			type = HFI_MSG_ACK;
		else
			type = MSG_HDR_GET_ID(next_hdr);

		if (type != F2H_MSG_CONTEXT_BAD)
			gen8_hfi_queue_read(gmu, HFI_MSG_ID, rcvd, sizeof(rcvd));

		switch (type) {
		case HFI_MSG_ACK:
			/*
			 * We are assuming that there is only one outstanding ack because hfi
			 * sending thread waits for completion while holding the device mutex
			 * (except when we send H2F_MSG_HW_FENCE_INFO packets)
			 */
			if (MSG_HDR_GET_ID(rcvd[1]) == H2F_MSG_HW_FENCE_INFO)
				process_hw_fence_ack(adreno_dev, rcvd[1]);
			else
				gen8_receive_ack_async(adreno_dev, rcvd);
			break;
		case F2H_MSG_CONTEXT_BAD:
			gen8_hfi_queue_read(gmu, HFI_MSG_ID, (u32 *)adreno_dev->hwsched.ctxt_bad,
						HFI_MAX_MSG_SIZE);
			process_ctx_bad(adreno_dev);
			break;
		case F2H_MSG_TS_RETIRE:
			log_profiling_info(adreno_dev, rcvd);
			adreno_scheduler_queue(adreno_dev);
			break;
		case F2H_MSG_SYNCOBJ_QUERY:
			gen8_trigger_syncobj_query(adreno_dev, rcvd);
			break;
		case F2H_MSG_GMU_CNTR_RELEASE: {
			struct hfi_gmu_cntr_release_cmd *cmd =
				(struct hfi_gmu_cntr_release_cmd *) rcvd;

			adreno_perfcounter_put(adreno_dev,
				cmd->group_id, cmd->countable, PERFCOUNTER_FLAG_KERNEL);

			adreno_mark_for_coldboot(adreno_dev);
			}
			break;
		}
	}
	mutex_unlock(&hw_hfi->msgq_mutex);
}

static void process_log_block(struct adreno_device *adreno_dev, void *data)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct hfi_log_block *cmd = data;
	u32 *log_event = gmu->gmu_log->hostptr;
	u32 start, end;

	start = cmd->start_index;
	end = cmd->stop_index;

	log_event += start * 4;
	while (start != end) {
		trace_gmu_event(log_event);
		log_event += 4;
		start++;
	}
}

static void gen8_hwsched_process_dbgq(struct adreno_device *adreno_dev, bool limited)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u32 rcvd[MAX_RCVD_SIZE];
	bool recovery = false;

	while (gen8_hfi_queue_read(gmu, HFI_DBG_ID, rcvd, sizeof(rcvd)) > 0) {

		if (MSG_HDR_GET_ID(rcvd[0]) == F2H_MSG_ERR) {
			adreno_gen8_receive_err_req(gmu, rcvd);
			recovery = true;
			break;
		}

		if (MSG_HDR_GET_ID(rcvd[0]) == F2H_MSG_DEBUG)
			adreno_gen8_receive_debug_req(gmu, rcvd);

		if (MSG_HDR_GET_ID(rcvd[0]) == F2H_MSG_LOG_BLOCK)
			process_log_block(adreno_dev, rcvd);

		/* Process one debug queue message and return to not delay msgq processing */
		if (limited)
			break;
	}

	if (!recovery)
		return;

	gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
}

/* HFI interrupt handler */
static irqreturn_t gen8_hwsched_hfi_handler(int irq, void *data)
{
	struct adreno_device *adreno_dev = data;
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 status = 0;

	/*
	 * GEN8_GMUCX_GMU2HOST_INTR_INFO may have bits set not specified in hfi->irq_mask.
	 * Read and clear only those irq bits that we are processing here.
	 */
	gmu_core_regread(device, GEN8_GMUCX_GMU2HOST_INTR_INFO, &status);
	gmu_core_regwrite(device, GEN8_GMUCX_GMU2HOST_INTR_CLR, status & hfi->irq_mask);

	/*
	 * If interrupts are not enabled on the HFI message queue,
	 * the inline message processing loop will process it,
	 * else, process it here.
	 */
	if (!(hfi->irq_mask & HFI_IRQ_MSGQ_MASK))
		status &= ~HFI_IRQ_MSGQ_MASK;

	if (status & (HFI_IRQ_MSGQ_MASK | HFI_IRQ_DBGQ_MASK)) {
		wake_up_interruptible(&hfi->f2h_wq);
		adreno_scheduler_queue(adreno_dev);
	}
	if (status & HFI_IRQ_CM3_FAULT_MASK) {
		atomic_set(&gmu->cm3_fault, 1);

		/* make sure other CPUs see the update */
		smp_wmb();

		dev_err_ratelimited(GMU_PDEV_DEV(device),
				"GMU CM3 fault interrupt received\n");

		gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
	}

	/* Ignore OOB bits */
	status &= GENMASK(31 - (oob_max - 1), 0);

	if (status & ~hfi->irq_mask)
		dev_err_ratelimited(GMU_PDEV_DEV(device),
			"Unhandled HFI interrupts 0x%x\n",
			status & ~hfi->irq_mask);

	return IRQ_HANDLED;
}

#define HFI_IRQ_MSGQ_MASK BIT(0)

static int check_ack_failure(struct adreno_device *adreno_dev,
	struct pending_cmd *ack)
{
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u64 ticks = gpudev->read_alwayson(adreno_dev);

	if (ack->results[2] != 0xffffffff)
		return 0;

	dev_err(GMU_PDEV_DEV(device),
		"ACK error: sender id %d seqnum %d\n",
		MSG_HDR_GET_ID(ack->sent_hdr),
		MSG_HDR_GET_SEQNUM(ack->sent_hdr));

	KGSL_GMU_CORE_FORCE_PANIC(device->gmu_core.gf_panic,
				GMU_PDEV(device), ticks, GMU_FAULT_HFI_ACK);
	return -EINVAL;
}

int gen8_hfi_send_cmd_async(struct adreno_device *adreno_dev, void *data, u32 size_bytes)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	u32 *cmd = data;
	u32 seqnum;
	int rc;
	struct pending_cmd pending_ack;

	seqnum = atomic_inc_return(&gmu->hfi.seqnum);
	*cmd = MSG_HDR_SET_SEQNUM_SIZE(*cmd, seqnum, size_bytes >> 2);

	add_waiter(hfi, *cmd, &pending_ack);

	rc = gen8_hfi_cmdq_write(adreno_dev, cmd, size_bytes);
	if (rc)
		goto done;

	rc = adreno_hwsched_wait_ack_completion(adreno_dev,
		GMU_PDEV_DEV(device), &pending_ack, gen8_hwsched_process_msgq);
	if (rc)
		goto done;

	rc = check_ack_failure(adreno_dev, &pending_ack);

done:
	del_waiter(hfi, &pending_ack);

	return rc;
}

static void init_queues(struct gen8_hfi *hfi)
{
	u32 gmuaddr = hfi->hfi_mem->gmuaddr;
	struct hfi_queue_table hfi_table = {
		.qtbl_hdr = {
			.version = 0,
			.size = sizeof(struct hfi_queue_table) >> 2,
			.qhdr0_offset =
				sizeof(struct hfi_queue_table_header) >> 2,
			.qhdr_size = sizeof(struct hfi_queue_header) >> 2,
			.num_q = HFI_QUEUE_MAX,
			.num_active_q = HFI_QUEUE_MAX,
		},
		.qhdr = {
			DEFINE_QHDR(gmuaddr, HFI_CMD_ID, 0),
			DEFINE_QHDR(gmuaddr, HFI_MSG_ID, 0),
			DEFINE_QHDR(gmuaddr, HFI_DBG_ID, 0),
		},
	};

	memcpy(hfi->hfi_mem->hostptr, &hfi_table, sizeof(hfi_table));
}

/* Total header sizes + queue sizes + 16 for alignment */
#define HFIMEM_SIZE (sizeof(struct hfi_queue_table) + 16 + \
	(SZ_4K * HFI_QUEUE_MAX))

static int hfi_f2h_main(void *arg);

int gen8_hwsched_hfi_init(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct gen8_hfi *hfi = to_gen8_hfi(adreno_dev);

	if (IS_ERR_OR_NULL(hw_hfi->big_ib)) {
		hw_hfi->big_ib = gen8_reserve_gmu_kernel_block(
				to_gen8_gmu(adreno_dev), 0,
				HWSCHED_MAX_IBS * sizeof(struct hfi_issue_ib),
				GMU_NONCACHED_KERNEL, 0);
		if (IS_ERR(hw_hfi->big_ib))
			return PTR_ERR(hw_hfi->big_ib);
	}

	if (ADRENO_FEATURE(adreno_dev, ADRENO_LSR) &&
			IS_ERR_OR_NULL(hw_hfi->big_ib_recurring)) {
		hw_hfi->big_ib_recurring = gen8_reserve_gmu_kernel_block(
				to_gen8_gmu(adreno_dev), 0,
				HWSCHED_MAX_IBS * sizeof(struct hfi_issue_ib),
				GMU_NONCACHED_KERNEL, 0);
		if (IS_ERR(hw_hfi->big_ib_recurring))
			return PTR_ERR(hw_hfi->big_ib_recurring);
	}

	if (IS_ERR_OR_NULL(hfi->hfi_mem)) {
		hfi->hfi_mem = gen8_reserve_gmu_kernel_block(
				to_gen8_gmu(adreno_dev),
				0, HFIMEM_SIZE, GMU_NONCACHED_KERNEL, 0);
		if (IS_ERR(hfi->hfi_mem))
			return PTR_ERR(hfi->hfi_mem);
		init_queues(hfi);
	}

	if (IS_ERR_OR_NULL(hw_hfi->f2h_task)) {
		hw_hfi->f2h_task = kthread_run(hfi_f2h_main, adreno_dev, "gmu_f2h");
		if (!IS_ERR(hw_hfi->f2h_task))
			sched_set_fifo(hw_hfi->f2h_task);
	}

	return PTR_ERR_OR_ZERO(hw_hfi->f2h_task);
}

static int get_attrs(u32 flags)
{
	int attrs = IOMMU_READ;

	if (flags & HFI_MEMFLAG_GMU_PRIV)
		attrs |= IOMMU_PRIV;

	if (flags & HFI_MEMFLAG_GMU_WRITEABLE)
		attrs |= IOMMU_WRITE;

	return attrs;
}

static int gmu_import_buffer(struct adreno_device *adreno_dev,
	struct hfi_mem_alloc_entry *entry)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct hfi_mem_alloc_desc *desc = &entry->desc;
	u32 vma_id = (desc->flags & HFI_MEMFLAG_GMU_CACHEABLE) ? GMU_CACHE : GMU_NONCACHED_KERNEL;

	return gen8_gmu_import_buffer(gmu, vma_id, entry->md, get_attrs(desc->flags), desc->align);
}

static struct hfi_mem_alloc_entry *lookup_mem_alloc_table(
	struct adreno_device *adreno_dev, struct hfi_mem_alloc_desc *desc)
{
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);
	int i;

	for (i = 0; i < hw_hfi->mem_alloc_entries; i++) {
		struct hfi_mem_alloc_entry *entry = &hw_hfi->mem_alloc_table[i];

		if ((entry->desc.mem_kind == desc->mem_kind) &&
			(entry->desc.gmu_mem_handle == desc->gmu_mem_handle))
			return entry;
	}

	return NULL;
}

static struct hfi_mem_alloc_entry *get_mem_alloc_entry(
	struct adreno_device *adreno_dev, struct hfi_mem_alloc_desc *desc)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(device);
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct hfi_mem_alloc_entry *entry =
		lookup_mem_alloc_table(adreno_dev, desc);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u64 flags = 0;
	u32 priv = 0;
	int ret;
	const char *memkind_string = desc->mem_kind < HFI_MEMKIND_MAX ?
			hfi_memkind_strings[desc->mem_kind] : "UNKNOWN";

	if (entry)
		return entry;

	if (desc->mem_kind >= HFI_MEMKIND_MAX) {
		dev_err(gmu_pdev_dev, "Invalid mem kind: %d\n",
			desc->mem_kind);
		return ERR_PTR(-EINVAL);
	}

	if (hfi->mem_alloc_entries == ARRAY_SIZE(hfi->mem_alloc_table)) {
		dev_err(gmu_pdev_dev,
			"Reached max mem alloc entries\n");
		return ERR_PTR(-ENOMEM);
	}

	entry = &hfi->mem_alloc_table[hfi->mem_alloc_entries];

	memcpy(&entry->desc, desc, sizeof(*desc));

	entry->desc.host_mem_handle = desc->gmu_mem_handle;

	if (desc->flags & HFI_MEMFLAG_GFX_PRIV)
		priv |= KGSL_MEMDESC_PRIVILEGED;

	if (!(desc->flags & HFI_MEMFLAG_GFX_WRITEABLE))
		flags |= KGSL_MEMFLAGS_GPUREADONLY;

	if (desc->flags & HFI_MEMFLAG_GFX_SECURE)
		flags |= KGSL_MEMFLAGS_SECURE;

	if (!(desc->flags & HFI_MEMFLAG_GFX_ACC) &&
		(desc->mem_kind != HFI_MEMKIND_HW_FENCE)) {
		if (desc->mem_kind == HFI_MEMKIND_MMIO_IPC_CORE)
			entry->md = gen8_reserve_gmu_kernel_block_fixed(gmu, 0,
					desc->size,
					(desc->flags & HFI_MEMFLAG_GMU_CACHEABLE) ?
					GMU_CACHE : GMU_NONCACHED_KERNEL,
					"qcom,ipc-core", get_attrs(desc->flags),
					desc->align);
		else
			entry->md = gen8_reserve_gmu_kernel_block(gmu, 0,
					desc->size,
					(desc->flags & HFI_MEMFLAG_GMU_CACHEABLE) ?
					GMU_CACHE : GMU_NONCACHED_KERNEL,
					desc->align);

		if (IS_ERR(entry->md)) {
			int ret = PTR_ERR(entry->md);

			memset(entry, 0, sizeof(*entry));
			return ERR_PTR(ret);
		}
		entry->desc.size = entry->md->size;
		entry->desc.gmu_addr = entry->md->gmuaddr;

		goto done;
	}

	/*
	 * Use pre-allocated memory descriptors to map the HFI_MEMKIND_HW_FENCE and
	 * HFI_MEMKIND_MEMSTORE
	 */
	switch (desc->mem_kind) {
	case HFI_MEMKIND_HW_FENCE:
		entry->md = &adreno_dev->hwsched.hw_fence_md;
		break;
	case HFI_MEMKIND_MEMSTORE:
		entry->md = device->memstore;
		break;
	default:
		entry->md = kgsl_allocate_global(device, desc->size, 0, flags,
			priv, memkind_string);
		break;
	}
	if (IS_ERR(entry->md)) {
		int ret = PTR_ERR(entry->md);

		memset(entry, 0, sizeof(*entry));
		return ERR_PTR(ret);
	}

	entry->desc.size = entry->md->size;
	entry->desc.gpu_addr = entry->md->gpuaddr;

	if (!(desc->flags & HFI_MEMFLAG_GMU_ACC))
		goto done;

	 /*
	  * If gmu mapping fails, then we have to live with
	  * leaking the gpu global buffer allocated above.
	  */
	ret = gmu_import_buffer(adreno_dev, entry);
	if (ret) {
		dev_err(gmu_pdev_dev,
			"gpuaddr: 0x%llx size: %lld bytes lost\n",
			entry->md->gpuaddr, entry->md->size);
		memset(entry, 0, sizeof(*entry));
		return ERR_PTR(ret);
	}

	entry->desc.gmu_addr = entry->md->gmuaddr;
done:
	hfi->mem_alloc_entries++;

	return entry;
}

static int process_mem_alloc(struct adreno_device *adreno_dev,
	struct hfi_mem_alloc_desc *mad)
{
	struct hfi_mem_alloc_entry *entry;

	entry = get_mem_alloc_entry(adreno_dev, mad);
	if (IS_ERR(entry))
		return PTR_ERR(entry);

	if (entry->md) {
		mad->gpu_addr = entry->md->gpuaddr;
		mad->gmu_addr = entry->md->gmuaddr;
	}

	/*
	 * GMU uses the host_mem_handle to check if this memalloc was
	 * successful
	 */
	mad->host_mem_handle = mad->gmu_mem_handle;

	return 0;
}

static int mem_alloc_reply(struct adreno_device *adreno_dev, void *rcvd)
{
	struct hfi_mem_alloc_desc desc = {0};
	struct hfi_mem_alloc_reply_cmd out = {0};
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u32 seqnum;
	int ret;

	hfi_get_mem_alloc_desc(rcvd, &desc);

	ret = process_mem_alloc(adreno_dev, &desc);
	if (ret)
		return ret;

	memcpy(&out.desc, &desc, sizeof(out.desc));

	out.hdr = ACK_MSG_HDR(F2H_MSG_MEM_ALLOC);

	seqnum = atomic_inc_return(&gmu->hfi.seqnum);
	out.hdr = MSG_HDR_SET_SEQNUM_SIZE(out.hdr, seqnum, sizeof(out) >> 2);

	out.req_hdr = *(u32 *)rcvd;

	return gen8_hfi_cmdq_write(adreno_dev, (u32 *)&out, sizeof(out));
}

static int gmu_cntr_register_reply(struct adreno_device *adreno_dev, void *rcvd)
{
	struct hfi_gmu_cntr_register_cmd *in = (struct hfi_gmu_cntr_register_cmd *)rcvd;
	struct hfi_gmu_cntr_register_reply_cmd out = {0};
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u32 lo = 0, hi = 0, seqnum;

	/*
	 * Failure to allocate counter is not fatal. Sending lo = 0, hi = 0
	 * indicates to GMU that counter allocation failed.
	 */
	adreno_perfcounter_get(adreno_dev,
		in->group_id, in->countable, &lo, &hi, PERFCOUNTER_FLAG_KERNEL);

	out.hdr = ACK_MSG_HDR(F2H_MSG_GMU_CNTR_REGISTER);
	seqnum = atomic_inc_return(&gmu->hfi.seqnum);
	out.hdr = MSG_HDR_SET_SEQNUM_SIZE(out.hdr, seqnum, sizeof(out) >> 2);
	out.req_hdr = in->hdr;
	out.group_id = in->group_id;
	out.countable = in->countable;
	/* Fill in byte offset of counter */
	out.cntr_lo = lo << 2;
	out.cntr_hi = hi << 2;

	return gen8_hfi_cmdq_write(adreno_dev, (u32 *)&out, sizeof(out));
}

static int send_warmboot_start_msg(struct adreno_device *adreno_dev)
{
	int ret = 0;
	struct hfi_start_cmd cmd;

	if (!adreno_dev->warmboot_enabled)
		return ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_START);
	if (ret)
		return ret;

	cmd.hdr = RECORD_NOP_MSG_HDR(cmd.hdr);

	return gen8_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
}

static int send_start_msg(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(device);
	int ret, rc = 0;
	struct hfi_start_cmd cmd;
	u32 seqnum, rcvd[MAX_RCVD_SIZE];
	struct pending_cmd pending_ack = {0};

	ret = CMD_MSG_HDR(cmd, H2F_MSG_START);
	if (ret)
		return ret;

	seqnum = atomic_inc_return(&gmu->hfi.seqnum);
	cmd.hdr = MSG_HDR_SET_SEQNUM_SIZE(cmd.hdr, seqnum, sizeof(cmd) >> 2);

	pending_ack.sent_hdr = cmd.hdr;

	rc = gen8_hfi_cmdq_write(adreno_dev, (u32 *)&cmd, sizeof(cmd));
	if (rc)
		return rc;

poll:
	rc = adreno_hwsched_poll_msg_queue_write_index(gmu->hfi.hfi_mem);

	if (rc) {
		dev_err(gmu_pdev_dev,
			"Timed out processing MSG_START seqnum: %d\n",
			seqnum);
		gmu_core_fault_snapshot(device, GMU_FAULT_H2F_MSG_START);
		goto done;
	}

	rc = gen8_hfi_queue_read(gmu, HFI_MSG_ID, rcvd, sizeof(rcvd));
	if (rc <= 0) {
		dev_err(gmu_pdev_dev,
			"MSG_START: payload error: %d\n",
			rc);
		gmu_core_fault_snapshot(device, GMU_FAULT_H2F_MSG_START);
		goto done;
	}

	switch (MSG_HDR_GET_ID(rcvd[0])) {
	case F2H_MSG_MEM_ALLOC:
		rc = mem_alloc_reply(adreno_dev, rcvd);
		break;
	case F2H_MSG_GMU_CNTR_REGISTER:
		rc = gmu_cntr_register_reply(adreno_dev, rcvd);
		break;
	default:
		if (MSG_HDR_GET_TYPE(rcvd[0]) == HFI_MSG_ACK) {
			rc = gen8_receive_ack_cmd(gmu, rcvd, &pending_ack);
			/* Check ack failure if we received an expected ack */
			if (!rc)
				rc = check_ack_failure(adreno_dev, &pending_ack);
			goto done;
		} else {
			dev_err(gmu_pdev_dev,
				"MSG_START: unexpected response id:%d, type:%d\n",
				MSG_HDR_GET_ID(rcvd[0]),
				MSG_HDR_GET_TYPE(rcvd[0]));
			gmu_core_fault_snapshot(device, GMU_FAULT_H2F_MSG_START);
			rc = -EINVAL;
			goto done;
		}
	}

	if (!rc)
		goto poll;

done:
	/* Clear the interrupt */
	gmu_core_regwrite(device, GEN8_GMUCX_GMU2HOST_INTR_CLR, HFI_IRQ_MSGQ_MASK);
	/*
	 * Add a write barrier to post the interrupt clear so that we dont have a
	 * pending interrupt.
	 */
	wmb();
	return rc;
}

static void reset_hfi_mem_records(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct kgsl_memdesc *md = NULL;
	u32 i;

	for (i = 0; i < hw_hfi->mem_alloc_entries; i++) {
		struct hfi_mem_alloc_desc *desc = &hw_hfi->mem_alloc_table[i].desc;

		if (desc->flags & HFI_MEMFLAG_HOST_INIT) {
			md = hw_hfi->mem_alloc_table[i].md;
			memset(md->hostptr, 0x0, md->size);
		}
	}
}

static void reset_hfi_queues(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct hfi_queue_table *tbl = gmu->hfi.hfi_mem->hostptr;
	u32 i;

	/* Flush HFI queues */
	for (i = 0; i < HFI_QUEUE_MAX; i++) {
		struct hfi_queue_header *hdr = &tbl->qhdr[i];

		if (hdr->status == HFI_QUEUE_STATUS_DISABLED)
			continue;

		hdr->read_index = hdr->write_index;
	}
}

void gen8_hwsched_hfi_stop(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);

	hfi->irq_mask &= ~HFI_IRQ_MSGQ_MASK;

	/*
	 * In some corner cases, it is possible that GMU put TS_RETIRE
	 * on the msgq after we have turned off gmu interrupts. Hence,
	 * drain the queue one last time before we reset HFI queues.
	 */
	gen8_hwsched_process_msgq(adreno_dev);

	/* Drain the debug queue before we reset HFI queues */
	gen8_hwsched_process_dbgq(adreno_dev, false);

	kgsl_pwrctrl_axi(KGSL_DEVICE(adreno_dev), false);

	clear_bit(GMU_PRIV_HFI_STARTED, &gmu->flags);

	/*
	 * Reset the hfi host access memory records, As GMU expects hfi memory
	 * records to be clear in bootup.
	 */
	reset_hfi_mem_records(adreno_dev);
}

static void gen8_hwsched_enable_async_hfi(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);

	hfi->irq_mask |= HFI_IRQ_MSGQ_MASK;

	gmu_core_regwrite(KGSL_DEVICE(adreno_dev), GEN8_GMUCX_GMU2HOST_INTR_MASK,
		(u32)~hfi->irq_mask);
}

static int enable_preemption(struct adreno_device *adreno_dev)
{
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 data;
	int ret;

	if (!adreno_is_preemption_enabled(adreno_dev))
		return 0;

	/*
	 * Bits [0:1] contains the preemption level
	 * Bit 2 is to enable/disable gmem save/restore
	 * Bit 3 is to enable/disable skipsaverestore
	 */
	data = FIELD_PREP(GENMASK(1, 0), adreno_dev->preempt.preempt_level) |
			FIELD_PREP(BIT(2), adreno_dev->preempt.usesgmem) |
			FIELD_PREP(BIT(3), adreno_dev->preempt.skipsaverestore);

	ret = gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_PREEMPTION, 1,
			data);
	if (ret)
		return ret;

	if (gen8_core->qos_value) {
		int i;

		for (i = 0; i < KGSL_PRIORITY_MAX_RB_LEVELS; i++) {
			if (!gen8_core->qos_value[i])
				continue;

			gen8_hfi_send_set_value(adreno_dev,
				HFI_VALUE_RB_GPU_QOS, i,
				gen8_core->qos_value[i]);
		}
	}

	if (device->pwrctrl.rt_bus_hint) {
		ret = gen8_hfi_send_set_value(adreno_dev, HFI_VALUE_RB_IB_RULE, 0,
			device->pwrctrl.rt_bus_hint);
		if (ret)
			device->pwrctrl.rt_bus_hint = 0;
	}

	/*
	 * Bits[3:0] contain the preemption timeout enable bit per ringbuffer
	 * Bits[31:4] contain the timeout in ms
	 */
	return gen8_hfi_send_set_value(adreno_dev, HFI_VALUE_BIN_TIME, 1,
		FIELD_PREP(GENMASK(31, 4), ADRENO_PREEMPT_TIMEOUT) |
		FIELD_PREP(GENMASK(3, 0), 0xf));

}

static int enable_gmu_stats(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u32 data;

	if (!gmu->stats_enable)
		return 0;

	/*
	 * Bits [23:0] contains the countables mask
	 * Bits [31:24] is the sampling interval
	 */
	data = FIELD_PREP(GENMASK(23, 0), gmu->stats_mask) |
		FIELD_PREP(GENMASK(31, 24), gmu->stats_interval);

	return gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_GMU_STATS, 1, data);
}

static int gen8_hfi_send_perfcounter_feature_ctrl(struct adreno_device *adreno_dev)
{
	/*
	 * Perfcounter retention is disabled by default in GMU firmware.
	 * In case perfcounter retention behaviour is overwritten by sysfs
	 * setting dynmaically, send this HFI feature with 'enable = 0' to
	 * disable this feature in GMU firmware.
	 */
	if (adreno_dev->perfcounter)
		return gen8_hfi_send_feature_ctrl(adreno_dev,
				HFI_FEATURE_PERF_NORETAIN, 0, 0);

	return 0;
}

u32 gen8_hwsched_hfi_get_value(struct adreno_device *adreno_dev, u32 prop)
{
	struct hfi_get_value_cmd cmd;
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct pending_cmd pending_ack;
	u32 seqnum;
	int rc;

	rc = CMD_MSG_HDR(cmd, H2F_MSG_GET_VALUE);
	if (rc)
		return 0;

	seqnum = atomic_inc_return(&gmu->hfi.seqnum);
	cmd.hdr = MSG_HDR_SET_SEQNUM_SIZE(cmd.hdr, seqnum, sizeof(cmd) >> 2);
	cmd.type = prop;
	cmd.subtype = 0;

	add_waiter(hfi, cmd.hdr, &pending_ack);

	rc = gen8_hfi_cmdq_write(adreno_dev, (u32 *)&cmd, sizeof(cmd));
	if (rc)
		goto done;

	rc = adreno_hwsched_wait_ack_completion(adreno_dev,
		gmu_pdev_dev, &pending_ack, gen8_hwsched_process_msgq);

done:
	del_waiter(hfi, &pending_ack);

	if (rc || (pending_ack.results[2] == UINT_MAX))
		return 0;

	return pending_ack.results[2];
}

static int gen8_hfi_send_hw_fence_feature_ctrl(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	int ret;

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags))
		return 0;

	ret = gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_HW_FENCE, 1, 0);
	if (ret && (ret == -ENOENT)) {
		dev_err(gmu_pdev_dev,
			"GMU doesn't support HW_FENCE feature\n");
		adreno_hwsched_deregister_hw_fence(adreno_dev);
		return 0;
	}

	return ret;
}

static void gen8_spin_idle_debug_lpac(struct adreno_device *adreno_dev,
				const char *str)
{
	struct kgsl_device *device = &adreno_dev->dev;
	u32 rptr, wptr, status, intstatus, global_status;
	bool val = adreno_is_preemption_enabled(adreno_dev);

	dev_err(device->dev, str);

	kgsl_regread(device, GEN8_CP_RB_RPTR_LPAC, &rptr);
	kgsl_regread(device, GEN8_CP_RB_WPTR_LPAC, &wptr);

	kgsl_regread(device, GEN8_RBBM_STATUS, &status);
	kgsl_regread(device, GEN8_RBBM_INT_0_STATUS, &intstatus);
	kgsl_regread(device, GEN8_CP_INTERRUPT_STATUS_GLOBAL, &global_status);

	dev_err(device->dev,
		"LPAC rb=%d pos=%X/%X rbbm_status=%8.8X int_0_status=%8.8X global_status=%8.8X\n",
		val ? KGSL_LPAC_RB_ID : 1, rptr, wptr,
		status, intstatus, global_status);

	kgsl_device_snapshot(device, NULL, NULL, false);
}

static bool gen8_hwsched_warmboot_possible(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);

	if (adreno_dev->warmboot_enabled && test_bit(GMU_PRIV_WARMBOOT_GMU_INIT_DONE, &gmu->flags)
		&& test_bit(GMU_PRIV_WARMBOOT_GPU_BOOT_DONE, &gmu->flags) &&
		!test_bit(ADRENO_DEVICE_FORCE_COLDBOOT, &adreno_dev->priv))
		return true;

	return false;
}

static int gen8_hwsched_hfi_send_warmboot_cmd(struct adreno_device *adreno_dev,
		struct kgsl_memdesc *desc, u32 flag, bool async, struct pending_cmd *ack)
{
	struct hfi_warmboot_scratch_cmd cmd = {0};
	int ret;

	if (!adreno_dev->warmboot_enabled)
		return 0;

	cmd.scratch_addr = desc->gmuaddr;
	cmd.scratch_size =  desc->size;
	cmd.flags = flag;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_WARMBOOT_CMD);
	if (ret)
		return ret;

	if (async)
		return gen8_hfi_send_cmd_async(adreno_dev, &cmd, sizeof(cmd));

	return gen8_hfi_send_generic_req_v5(adreno_dev, &cmd, ack, sizeof(cmd));
}

static int gen8_hwsched_hfi_warmboot_gpu_cmd(struct adreno_device *adreno_dev,
		struct pending_cmd *ret_cmd)
{
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct hfi_warmboot_scratch_cmd cmd = {
		.scratch_addr = gmu->gpu_boot_scratch->gmuaddr,
		.scratch_size = gmu->gpu_boot_scratch->size,
		.flags = HFI_WARMBOOT_EXEC_SCRATCH,
	};
	int ret = 0;
	u32 seqnum;

	if (!adreno_dev->warmboot_enabled)
		return 0;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_WARMBOOT_CMD);
	if (ret)
		return ret;

	seqnum = atomic_inc_return(&gmu->hfi.seqnum);
	cmd.hdr = MSG_HDR_SET_SEQNUM_SIZE(cmd.hdr, seqnum, sizeof(cmd) >> 2);
	add_waiter(hfi, cmd.hdr, ret_cmd);

	ret = gen8_hfi_cmdq_write(adreno_dev, (u32 *)&cmd, sizeof(cmd));
	if (ret)
		goto err;

	ret = adreno_hwsched_wait_ack_completion(adreno_dev,
		gmu_pdev_dev, ret_cmd, gen8_hwsched_process_msgq);
err:
	del_waiter(hfi, ret_cmd);

	return ret;
}

static void print_warmboot_gpu_error(struct device *dev, struct pending_cmd *ret_cmd)
{
	dev_err(dev,
		"HFI ACK: Req=0x%8.8X, Result=0x%8.8X Error:0x%8.8X\n",
		ret_cmd->results[1], ret_cmd->results[2], ret_cmd->results[3]);
}

static int gen8_hwsched_warmboot_gpu(struct adreno_device *adreno_dev)
{
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct pending_cmd ret_cmd = {0};
	int ret = 0;

	ret = gen8_hwsched_hfi_warmboot_gpu_cmd(adreno_dev, &ret_cmd);
	if (ret)
		goto err;

	/* Check if the ack belongs to the warmboot command */
	if (MSG_HDR_GET_ID(ret_cmd.results[0]) != H2F_MSG_WARMBOOT_CMD) {
		ret = -EINVAL;
		goto err;
	}

	switch (ret_cmd.results[3]) {
	/* If ack has no error code then GPU executed raw commands just fine */
	case GMU_SUCCESS:
		return ret_cmd.results[2];
	/* If GPU timedout processing raw commands, check which raw command failed */
	case GMU_ERROR_TIMEOUT: {
		if (MSG_HDR_GET_ID(ret_cmd.results[1]) == H2F_MSG_ISSUE_CMD_RAW) {

			/* Based on sequence number we can differentiate which command failed */
			if (ret_cmd.results[1] == gmu->cp_init_hdr)
				gen8_spin_idle_debug(adreno_dev,
					"CP initialization failed to idle\n");
			else if (ret_cmd.results[1] == gmu->switch_to_unsec_hdr)
				gen8_spin_idle_debug(adreno_dev,
					"Switch to unsecure failed to idle\n");
		} else if (MSG_HDR_GET_ID(ret_cmd.results[1]) == H2F_MSG_ISSUE_LPAC_CMD_RAW) {
			gen8_spin_idle_debug_lpac(adreno_dev,
				"LPAC CP initialization failed to idle\n");
		} else {
			print_warmboot_gpu_error(gmu_pdev_dev, &ret_cmd);
		}
		ret = -EINVAL;
		break;
	}
	default:
		print_warmboot_gpu_error(gmu_pdev_dev, &ret_cmd);
		ret = -EINVAL;
		break;
	}
err:
	/* Clear the bit on error so that in the next slumber exit we coldboot */
	clear_bit(GMU_PRIV_WARMBOOT_GPU_BOOT_DONE, &gmu->flags);
	gen8_disable_gpu_irq(adreno_dev);
	return ret;
}

static int gen8_hwsched_coldboot_gpu(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hfi *hfi = to_gen8_hfi(adreno_dev);
	struct pending_cmd ack = {0};
	int ret = 0;

	ret = gen8_hwsched_hfi_send_warmboot_cmd(adreno_dev, gmu->gpu_boot_scratch,
		 HFI_WARMBOOT_SET_SCRATCH, true, &ack);
	if (ret)
		goto done;

	ret = gen8_hwsched_cp_init(adreno_dev);
	if (ret)
		goto done;

	ret = gen8_hwsched_lpac_cp_init(adreno_dev);
	if (ret)
		goto done;

	ret = gen8_hwsched_hfi_send_warmboot_cmd(adreno_dev, gmu->gpu_boot_scratch,
		HFI_WARMBOOT_QUERY_SCRATCH, true, &ack);
	if (ret)
		goto done;

	if (adreno_dev->warmboot_enabled)
		set_bit(GMU_PRIV_WARMBOOT_GPU_BOOT_DONE, &gmu->flags);

done:
	/* Clear the bitmask so that we don't send record bit with future HFI messages */
	memset(hfi->wb_set_record_bitmask, 0x0, sizeof(hfi->wb_set_record_bitmask));

	if (ret)
		gen8_disable_gpu_irq(adreno_dev);

	return ret;
}

int gen8_hwsched_boot_gpu(struct adreno_device *adreno_dev)
{
	/* If warmboot is possible just send the warmboot command else coldboot */
	if (gen8_hwsched_warmboot_possible(adreno_dev))
		return gen8_hwsched_warmboot_gpu(adreno_dev);
	else
		return gen8_hwsched_coldboot_gpu(adreno_dev);
}

int gen8_hwsched_warmboot_init_gmu(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct pending_cmd ack = {0};
	int ret = 0;

	ret = gen8_hwsched_hfi_send_warmboot_cmd(adreno_dev, gmu->gmu_init_scratch,
		 HFI_WARMBOOT_EXEC_SCRATCH, false, &ack);
	if (ret)
		goto err;

	gen8_hwsched_enable_async_hfi(adreno_dev);

	set_bit(GMU_PRIV_HFI_STARTED, &gmu->flags);

	ret = kgsl_pwrctrl_setup_default_votes(KGSL_DEVICE(adreno_dev));

err:
	if (ret) {
		/* Clear the bit in case of an error so next boot will be coldboot */
		clear_bit(GMU_PRIV_WARMBOOT_GMU_INIT_DONE, &gmu->flags);
		clear_bit(GMU_PRIV_WARMBOOT_GPU_BOOT_DONE, &gmu->flags);
		gen8_hwsched_hfi_stop(adreno_dev);
	}

	return ret;
}

static void warmboot_init_message_record_bitmask(struct adreno_device *adreno_dev)
{
	struct gen8_hfi *hfi = to_gen8_hfi(adreno_dev);

	if (!adreno_dev->warmboot_enabled)
		return;

	/* Set the record bit for all the messages */
	memset(hfi->wb_set_record_bitmask, 0xFF, sizeof(hfi->wb_set_record_bitmask));

	/* These messages should not be recorded */
	clear_bit(H2F_MSG_WARMBOOT_CMD, hfi->wb_set_record_bitmask);
	clear_bit(H2F_MSG_START, hfi->wb_set_record_bitmask);
	clear_bit(H2F_MSG_GET_VALUE, hfi->wb_set_record_bitmask);
	clear_bit(H2F_MSG_GX_BW_PERF_VOTE, hfi->wb_set_record_bitmask);
}

static int gen8_hfi_send_thermal_feature_ctrl(struct adreno_device *adreno_dev)
{
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	const struct hfi_therm_profile_ctrl *therm = gen8_core->therm_profile;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	static struct hfi_thermaltable_cmd cmd = {0};
	int ret;

	if (!test_bit(GMU_THERMAL_MITIGATION, &device->gmu_core.flags) || !therm)
		return 0;

	ret = gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_THERMAL, 1, 0);
	if (ret)
		return ret;

	if (cmd.version == 0) {
		ret = CMD_MSG_HDR(cmd, H2F_MSG_THERM_TBL);
		if (ret)
			return ret;

		cmd.version = 1;
		memcpy(&cmd.ctrl, therm, sizeof(*therm));
	}

	return gen8_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
}

int gen8_hwsched_hfi_start(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct pending_cmd ack = {0};
	int ret;

	reset_hfi_queues(adreno_dev);

	ret = gen8_gmu_hfi_start(adreno_dev);
	if (ret)
		goto err;

	if (gen8_hwsched_warmboot_possible(adreno_dev))
		return gen8_hwsched_warmboot_init_gmu(adreno_dev);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_GMU_WARMBOOT) &&
		(!test_bit(GMU_PRIV_FIRST_BOOT_DONE, &gmu->flags))) {
		if (gen8_hfi_send_get_value(adreno_dev, HFI_VALUE_GMU_WARMBOOT, 0) == 1)
			adreno_dev->warmboot_enabled = true;
	}

	warmboot_init_message_record_bitmask(adreno_dev);

	/* Reset the variable here and set it when we successfully record the scratch */
	clear_bit(GMU_PRIV_WARMBOOT_GMU_INIT_DONE, &gmu->flags);
	clear_bit(GMU_PRIV_WARMBOOT_GPU_BOOT_DONE, &gmu->flags);

	ret = gen8_hwsched_hfi_send_warmboot_cmd(adreno_dev, gmu->gmu_init_scratch,
		HFI_WARMBOOT_SET_SCRATCH, false, &ack);
	if (ret)
		goto err;

	ret = gen8_hfi_send_gpu_perf_table(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_generic_req(adreno_dev, &gmu->hfi.bw_table, sizeof(gmu->hfi.bw_table));
	if (ret)
		goto err;

	ret = gen8_hfi_send_acd_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_bcl_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_clx_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_ifpc_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_thermal_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_HWSCHED, 1, 0);
	if (ret)
		goto err;

	ret = gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_KPROF, 1, 0);
	if (ret)
		goto err;

	if (ADRENO_FEATURE(adreno_dev, ADRENO_LSR)) {
		ret = gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_LSR,
				1, 0);
		if (ret)
			goto err;
	}

	ret = gen8_hfi_send_perfcounter_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	/* Enable the long ib timeout detection */
	if (adreno_long_ib_detect(adreno_dev)) {
		ret = gen8_hfi_send_feature_ctrl(adreno_dev,
			HFI_FEATURE_BAIL_OUT_TIMER, 1, 0);
		if (ret)
			goto err;
	}

	enable_gmu_stats(adreno_dev);

	if (gmu->log_stream_enable)
		gen8_hfi_send_set_value(adreno_dev,
			HFI_VALUE_LOG_STREAM_ENABLE, 0, 1);

	if (gmu->log_group_mask)
		gen8_hfi_send_set_value(adreno_dev,
			HFI_VALUE_LOG_GROUP, 0, gmu->log_group_mask);

	ret = gen8_hfi_send_core_fw_start(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_hw_fence_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	ret = enable_preemption(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_lpac_feature_ctrl(adreno_dev);
	if (ret)
		goto err;

	if (ADRENO_FEATURE(adreno_dev, ADRENO_AQE)) {
		ret = gen8_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_AQE, 1, 0);
		if (ret)
			goto err;
	}

	ret = send_start_msg(adreno_dev);
	if (ret)
		goto err;

	/*
	 * Send this additional start message on cold boot if warmboot is enabled.
	 * This message will be recorded and on a warmboot this will trigger the
	 * sequence to replay memory allocation requests and ECP task setup
	 */
	ret = send_warmboot_start_msg(adreno_dev);
	if (ret)
		goto err;

	gen8_hwsched_enable_async_hfi(adreno_dev);

	set_bit(GMU_PRIV_HFI_STARTED, &gmu->flags);

	/* Send this message only on cold boot */
	ret = gen8_hwsched_hfi_send_warmboot_cmd(adreno_dev, gmu->gmu_init_scratch,
		HFI_WARMBOOT_QUERY_SCRATCH, true, &ack);
	if (ret)
		goto err;

	if (adreno_dev->warmboot_enabled)
		set_bit(GMU_PRIV_WARMBOOT_GMU_INIT_DONE, &gmu->flags);

	ret = kgsl_pwrctrl_setup_default_votes(KGSL_DEVICE(adreno_dev));

err:
	if (ret)
		gen8_hwsched_hfi_stop(adreno_dev);

	return ret;
}

static int submit_raw_cmds(struct adreno_device *adreno_dev, void *cmds, u32 size_bytes,
	const char *str)
{
	int ret;

	ret = gen8_hfi_send_cmd_async(adreno_dev, cmds, size_bytes);
	if (ret)
		return ret;

	ret = gmu_core_timed_poll_check(KGSL_DEVICE(adreno_dev),
			GEN8_GMUAO_GPU_CX_BUSY_STATUS, 0, 200, BIT(23));
	if (ret)
		gen8_spin_idle_debug(adreno_dev, str);

	return ret;
}

static int submit_lpac_raw_cmds(struct adreno_device *adreno_dev, void *cmds, u32 size_bytes,
	const char *str)
{
	int ret;

	ret = gen8_hfi_send_cmd_async(adreno_dev, cmds, size_bytes);
	if (ret)
		return ret;

	ret = gmu_core_timed_poll_check(KGSL_DEVICE(adreno_dev),
			GEN8_GMUAO_LPAC_BUSY_STATUS, 0, 200, BIT(23));
	if (ret)
		gen8_spin_idle_debug_lpac(adreno_dev, str);

	return ret;
}

static int cp_init(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u32 cmds[GEN8_CP_INIT_DWORDS + 1];
	int ret = 0;

	cmds[0] = CREATE_MSG_HDR(H2F_MSG_ISSUE_CMD_RAW, HFI_MSG_CMD);

	gen8_cp_init_cmds(adreno_dev, &cmds[1]);

	ret = submit_raw_cmds(adreno_dev, cmds, sizeof(cmds),
			"CP initialization failed to idle\n");

	/* Save the header incase we need a warmboot debug */
	gmu->cp_init_hdr = cmds[0];

	return ret;
}

static int send_switch_to_unsecure(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u32 cmds[3];
	int ret = 0;

	cmds[0] = CREATE_MSG_HDR(H2F_MSG_ISSUE_CMD_RAW, HFI_MSG_CMD);

	cmds[1] = cp_type7_packet(CP_SET_SECURE_MODE, 1);
	cmds[2] = 0;

	ret = submit_raw_cmds(adreno_dev, cmds, sizeof(cmds),
			"Switch to unsecure failed to idle\n");

	/* Save the header incase we need a warmboot debug */
	gmu->switch_to_unsec_hdr = cmds[0];

	return ret;
}

int gen8_hwsched_cp_init(struct adreno_device *adreno_dev)
{
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	int ret;

	ret = cp_init(adreno_dev);
	if (ret)
		return ret;

	ret = adreno_zap_shader_load(adreno_dev, gen8_core->zap_name);
	if (ret)
		return ret;

	if (!adreno_dev->zap_loaded)
		kgsl_regwrite(KGSL_DEVICE(adreno_dev),
			GEN8_RBBM_SECVID_TRUST_CNTL, 0x0);
	else
		ret = send_switch_to_unsecure(adreno_dev);

	return ret;
}

int gen8_hwsched_lpac_cp_init(struct adreno_device *adreno_dev)
{
	u32 cmds[GEN8_CP_INIT_DWORDS + 1];

	if (!adreno_dev->lpac_enabled)
		return 0;

	cmds[0] = CREATE_MSG_HDR(H2F_MSG_ISSUE_LPAC_CMD_RAW, HFI_MSG_CMD);

	gen8_cp_init_cmds(adreno_dev, &cmds[1]);

	return submit_lpac_raw_cmds(adreno_dev, cmds, sizeof(cmds),
			"LPAC CP initialization failed to idle\n");
}

static int hfi_f2h_main(void *arg)
{
	struct adreno_device *adreno_dev = arg;
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);

	while (!kthread_should_stop()) {
		wait_event_interruptible(hfi->f2h_wq, kthread_should_stop() ||
			/* If msgq irq is enabled and msgq has messages to process */
			(((hfi->irq_mask & HFI_IRQ_MSGQ_MASK) &&
			  !is_queue_empty(adreno_dev, HFI_MSG_ID)) ||
			 /* Trace buffer has messages to process */
			 !gmu_core_is_trace_empty(gmu->trace.md->hostptr) ||
			 /* Dbgq has messages to process */
			 !is_queue_empty(adreno_dev, HFI_DBG_ID)));

		if (kthread_should_stop())
			break;

		gen8_hwsched_process_msgq(adreno_dev);
		gmu_core_process_trace_data(KGSL_DEVICE(adreno_dev),
					gmu_pdev_dev, &gmu->trace);
		gen8_hwsched_process_dbgq(adreno_dev, true);
	}

	return 0;
}

static void gen8_hwsched_hw_fence_timeout(struct work_struct *work)
{
	struct gen8_hwsched_hfi *hfi = container_of(work, struct gen8_hwsched_hfi, hw_fence_ws);
	struct gen8_hwsched_device *gen8_hw_dev = container_of(hfi, struct gen8_hwsched_device,
						hwsched_hfi);
	struct adreno_device *adreno_dev = &gen8_hw_dev->gen8_dev.adreno_dev;
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	u32 unack_count, ts;
	struct adreno_context *drawctxt = NULL;
	bool fault;

	/* Check msgq one last time before recording a fault */
	gen8_hwsched_process_msgq(adreno_dev);

	spin_lock(&hfi->hw_fence.lock);

	unack_count = hfi->hw_fence.unack_count;

	fault = test_bit(GEN8_HWSCHED_HW_FENCE_SLEEP_BIT, &hfi->hw_fence.flags) &&
		test_bit(GEN8_HWSCHED_HW_FENCE_MAX_BIT, &hfi->hw_fence.flags) &&
		(unack_count > MIN_HW_FENCE_UNACK_COUNT);

	drawctxt = hfi->hw_fence.defer_drawctxt;
	ts = hfi->hw_fence.defer_ts;

	spin_unlock(&hfi->hw_fence.lock);

	if (!fault)
		return;

	dev_err(gmu_pdev_dev, "Hardware fence unack(%d) timeout\n", unack_count);

	if (drawctxt) {
		struct kgsl_process_private *proc_priv = drawctxt->base.proc_priv;

		dev_err(gmu_pdev_dev,
			"Hardware fence got deferred for ctx:%d ts:%d pid:%d proc:%s\n",
			drawctxt->base.id, ts, pid_nr(proc_priv->pid), proc_priv->comm);
	}
	gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
}

static void gen8_hwsched_hw_fence_timer(struct timer_list *t)
{
	struct gen8_hwsched_hfi *hfi = from_timer(hfi, t, hw_fence_timer);

	kgsl_schedule_work(&hfi->hw_fence_ws);
}

int gen8_hwsched_hfi_probe(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct platform_device *gmu_pdev = GMU_PDEV(KGSL_DEVICE(adreno_dev));
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);

	gmu->hfi.irq = kgsl_request_irq(gmu_pdev, "hfi",
		gen8_hwsched_hfi_handler, adreno_dev);

	if (gmu->hfi.irq < 0)
		return gmu->hfi.irq;

	hw_hfi->irq_mask = HFI_IRQ_MASK;

	rwlock_init(&hw_hfi->msglock);

	INIT_LIST_HEAD(&hw_hfi->msglist);
	INIT_LIST_HEAD(&hw_hfi->detached_hw_fence_list);

	init_waitqueue_head(&hw_hfi->f2h_wq);
	init_waitqueue_head(&hw_hfi->hw_fence.unack_wq);

	spin_lock_init(&hw_hfi->hw_fence.lock);

	mutex_init(&hw_hfi->msgq_mutex);

	INIT_WORK(&hw_hfi->hw_fence_ws, gen8_hwsched_hw_fence_timeout);

	timer_setup(&hw_hfi->hw_fence_timer, gen8_hwsched_hw_fence_timer, 0);

	return 0;
}

void gen8_hwsched_hfi_remove(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);

	if (hw_hfi->f2h_task)
		kthread_stop(hw_hfi->f2h_task);
}

static void init_gmu_context_queue(struct adreno_context *drawctxt)
{
	struct kgsl_memdesc *md = &drawctxt->gmu_context_queue;
	struct gmu_context_queue_header *hdr = md->hostptr;

	hdr->start_addr = md->gmuaddr + sizeof(*hdr);
	hdr->queue_size = (md->size - sizeof(*hdr)) >> 2;
	hdr->hw_fence_buffer_va = drawctxt->gmu_hw_fence_queue.gmuaddr;
	hdr->hw_fence_buffer_size = drawctxt->gmu_hw_fence_queue.size;
}

static int allocate_context_queues(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	int ret = 0;

	if (test_bit(ADRENO_HWSCHED_HW_FENCE, &adreno_dev->hwsched.flags) &&
		!drawctxt->gmu_hw_fence_queue.gmuaddr) {
		ret = gen8_alloc_gmu_kernel_block(
			to_gen8_gmu(adreno_dev), &drawctxt->gmu_hw_fence_queue,
			HW_FENCE_QUEUE_SIZE, GMU_NONCACHED_KERNEL,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV);
		if (ret) {
			memset(&drawctxt->gmu_hw_fence_queue, 0x0,
				sizeof(drawctxt->gmu_hw_fence_queue));
			return ret;
		}
	}

	if (!drawctxt->gmu_context_queue.gmuaddr) {
		ret = gen8_alloc_gmu_kernel_block(
			to_gen8_gmu(adreno_dev), &drawctxt->gmu_context_queue,
			SZ_4K, GMU_NONCACHED_KERNEL,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV);
		if (ret) {
			memset(&drawctxt->gmu_context_queue, 0x0,
				sizeof(drawctxt->gmu_context_queue));
			return ret;
		}
		init_gmu_context_queue(drawctxt);
	}

	return 0;
}

static int send_context_register(struct adreno_device *adreno_dev,
	struct kgsl_context *context)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct hfi_register_ctxt_cmd cmd;
	struct kgsl_pagetable *pt = context->proc_priv->pagetable;
	int ret, asid = kgsl_mmu_pagetable_get_asid(pt, context);

	if (asid < 0)
		return asid;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_REGISTER_CONTEXT);
	if (ret)
		return ret;

	ret = allocate_context_queues(adreno_dev, drawctxt);
	if (ret)
		return ret;

	cmd.ctxt_id = context->id;
	cmd.flags = HFI_CTXT_FLAG_NOTIFY | context->flags;
	/*
	 * HLOS SMMU driver programs context bank to look up ASID from TTBR0 during a page
	 * table walk. So the TLB entries are tagged with the ASID from TTBR0. TLBIASID
	 * invalidates TLB entries whose ASID matches the value that was written to the
	 * CBn_TLBIASID register. Set ASID along with PT address.
	 */
	cmd.pt_addr = kgsl_mmu_pagetable_get_ttbr0(pt) |
		FIELD_PREP(GENMASK_ULL(63, KGSL_IOMMU_ASID_START_BIT), asid);
	cmd.ctxt_idr = context->id;
	cmd.ctxt_bank = kgsl_mmu_pagetable_get_context_bank(pt, context);

	return gen8_hfi_send_cmd_async(adreno_dev, &cmd, sizeof(cmd));
}

static int send_context_pointers(struct adreno_device *adreno_dev,
	struct kgsl_context *context)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct hfi_context_pointers_cmd cmd = {0};
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	int ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_CONTEXT_POINTERS);
	if (ret)
		return ret;

	cmd.ctxt_id = context->id;
	cmd.sop_addr = MEMSTORE_ID_GPU_ADDR(device, context->id, soptimestamp);
	cmd.eop_addr = MEMSTORE_ID_GPU_ADDR(device, context->id, eoptimestamp);
	if (context->user_ctxt_record)
		cmd.user_ctxt_record_addr =
			context->user_ctxt_record->memdesc.gpuaddr;

	cmd.gmu_context_queue_addr = drawctxt->gmu_context_queue.gmuaddr;

	return gen8_hfi_send_cmd_async(adreno_dev, &cmd, sizeof(cmd));
}

static int hfi_context_register(struct adreno_device *adreno_dev,
	struct kgsl_context *context)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	if (context->gmu_registered)
		return 0;

	ret = send_context_register(adreno_dev, context);
	if (ret) {
		dev_err(GMU_PDEV_DEV(device),
			"Unable to register context %u: %d\n",
			context->id, ret);

		if (device->gmu_fault)
			gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);

		return ret;
	}

	ret = send_context_pointers(adreno_dev, context);
	if (ret) {
		dev_err(GMU_PDEV_DEV(device),
			"Unable to register context %u pointers: %d\n",
			context->id, ret);

		if (device->gmu_fault)
			gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);

		return ret;
	}

	context->gmu_registered = true;
	context->gmu_dispatch_queue = UINT_MAX;

	return 0;
}

static void populate_ibs(struct adreno_device *adreno_dev,
	struct hfi_submit_cmd *cmd, struct kgsl_drawobj_cmd *cmdobj)
{
	struct hfi_issue_ib *issue_ib;
	struct kgsl_memobj_node *ib;

	if (cmdobj->numibs > HWSCHED_MAX_DISPATCH_NUMIBS) {
		struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
		struct kgsl_memdesc *big_ib;

		if (test_bit(CMDOBJ_RECURRING_START, &cmdobj->priv))
			big_ib = hfi->big_ib_recurring;
		else
			big_ib = hfi->big_ib;
		/*
		 * The dispatcher ensures that there is only one big IB inflight
		 */
		cmd->big_ib_gmu_va = big_ib->gmuaddr;
		cmd->flags |= CMDBATCH_INDIRECT;
		issue_ib = big_ib->hostptr;
	} else {
		issue_ib = (struct hfi_issue_ib *)&cmd[1];
	}

	list_for_each_entry(ib, &cmdobj->cmdlist, node) {
		issue_ib->addr = ib->gpuaddr;
		issue_ib->size = ib->size;
		issue_ib++;
	}

	cmd->numibs = cmdobj->numibs;
}

#define HFI_DSP_IRQ_BASE 2

#define DISPQ_IRQ_BIT(_idx) BIT((_idx) + HFI_DSP_IRQ_BASE)
#define DISPQ_SYNC_IRQ_BIT(_idx) ((DISPQ_IRQ_BIT(_idx) << (KGSL_PRIORITY_MAX_RB_LEVELS + 1)))


static u32 get_irq_bit(struct adreno_device *adreno_dev, struct kgsl_drawobj *drawobj)
{
	if (adreno_is_preemption_enabled(adreno_dev))
		return adreno_get_level(drawobj->context);

	if (kgsl_context_is_lpac(drawobj->context))
		return 1;

	return 0;
}

static void populate_kgsl_fence(struct kgsl_drawobj_sync_hw_fence *hw_fence,
	struct hfi_syncobj *obj)
{
	struct kgsl_sync_fence *kfence = (struct kgsl_sync_fence *)hw_fence->fence;
	struct kgsl_sync_timeline *ktimeline = kfence->parent;
	unsigned long flags;

	obj->flags |= BIT(GMU_SYNCOBJ_FLAG_KGSL_FENCE_BIT);

	spin_lock_irqsave(&ktimeline->lock, flags);

	if (dma_fence_is_signaled_locked(&kfence->fence) || !_kgsl_context_get(ktimeline->context))
		obj->flags |= BIT(GMU_SYNCOBJ_FLAG_KGSL_FENCE_BIT);
	else
		hw_fence->context = ktimeline->context;

	spin_unlock_irqrestore(&ktimeline->lock, flags);

	obj->ctxt_id = kfence->context_id;
	obj->seq_no =  kfence->timestamp;
}

static int _submit_hw_fence(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj, void *cmdbuf)
{
	struct adreno_context *drawctxt = ADRENO_CONTEXT(drawobj->context);
	int i;
	u32 cmd_sizebytes, seqnum;
	struct kgsl_drawobj_sync *syncobj = SYNCOBJ(drawobj);
	struct hfi_submit_syncobj *cmd;
	struct hfi_syncobj *obj = NULL;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	/* Add hfi_syncobj struct for sync object */
	cmd_sizebytes = sizeof(*cmd) +
			(sizeof(struct hfi_syncobj) *
			syncobj->num_hw_fence);

	if (WARN_ON(cmd_sizebytes > HFI_MAX_MSG_SIZE))
		return -EMSGSIZE;

	memset(cmdbuf, 0x0, cmd_sizebytes);
	cmd = cmdbuf;
	cmd->num_syncobj = syncobj->num_hw_fence;
	obj = (struct hfi_syncobj *)&cmd[1];

	for (i = 0; i < syncobj->num_hw_fence; i++) {
		struct dma_fence *fence = syncobj->hw_fences[i].fence;

		if (is_kgsl_fence(fence)) {
			populate_kgsl_fence(&syncobj->hw_fences[i], obj);
		} else {
			int ret = kgsl_hw_fence_add_waiter(device, fence,
				&obj->hash_index);

			if (ret) {
				adreno_hwsched_syncobj_kfence_put(syncobj);
				syncobj->flags &= ~KGSL_SYNCOBJ_HW;
				return ret;
			}

			if (kgsl_hw_fence_signaled(fence) ||
				test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &fence->flags))
				obj->flags |= BIT(GMU_SYNCOBJ_FLAG_SIGNALED_BIT);

			obj->ctxt_id = fence->context;
			obj->seq_no =  fence->seqno;
		}
		trace_adreno_input_hw_fence(drawobj->context->id, obj->ctxt_id,
			obj->seq_no, obj->flags, fence->ops->get_timeline_name ?
			fence->ops->get_timeline_name(fence) : "unknown");

		obj->header = FIELD_PREP(GENMASK(15, 0), sizeof(*obj) >> 2);
		obj++;
	}

	/*
	 * Attach a timestamp to this SYNCOBJ to keep track whether GMU has deemed it signaled
	 * or not.
	 */
	drawobj->timestamp = ++drawctxt->syncobj_timestamp;
	cmd->timestamp = drawobj->timestamp;

	cmd->hdr = CREATE_MSG_HDR(H2F_MSG_ISSUE_SYNCOBJ, HFI_MSG_CMD);
	seqnum = atomic_inc_return(&adreno_dev->hwsched.submission_seqnum);
	cmd->hdr = MSG_HDR_SET_SEQNUM_SIZE(cmd->hdr, seqnum, cmd_sizebytes >> 2);

	return adreno_gmu_context_queue_write(adreno_dev, &drawctxt->gmu_context_queue,
			(u32 *)cmd, cmd_sizebytes, drawobj, NULL);
}

int gen8_hwsched_check_context_inflight_hw_fences(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct adreno_hw_fence_entry *entry, *tmp;
	int ret = 0;

	spin_lock(&drawctxt->lock);
	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_inflight_list, node) {
		struct gmu_context_queue_header *hdr =  drawctxt->gmu_context_queue.hostptr;

		if (timestamp_cmp((u32)entry->cmd.ts, hdr->out_fence_ts) > 0) {
			dev_err(GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev)),
				"detached ctx:%d has unsignaled fence ts:%d retired:%d\n",
				drawctxt->base.id, (u32)entry->cmd.ts, hdr->out_fence_ts);
			ret = -EINVAL;
			break;
		}

		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}
	spin_unlock(&drawctxt->lock);

	return ret;
}

/**
 * move_detached_context_hardware_fences - Move all pending hardware fences belonging to this
 * context to the detached hardware fence list so as to send them to TxQueue after fault recovery.
 * This is needed because this context may get destroyed before fault recovery gets executed.
 */
static void move_detached_context_hardware_fences(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct adreno_hw_fence_entry *entry, *tmp;
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);

	/* We don't need the drawctxt lock here because this context has already been detached */
	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_inflight_list, node) {
		struct gmu_context_queue_header *hdr =  drawctxt->gmu_context_queue.hostptr;

		if ((timestamp_cmp((u32)entry->cmd.ts, hdr->out_fence_ts) > 0)) {
			_kgsl_context_get(&drawctxt->base);
			list_move_tail(&entry->node, &hfi->detached_hw_fence_list);
			continue;
		}

		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}

	/* Also grab all the hardware fences which were never sent to GMU */
	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_list, node) {
		_kgsl_context_get(&drawctxt->base);
		list_move_tail(&entry->node, &hfi->detached_hw_fence_list);
	}
}

static int drain_context_hw_fence_gmu(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct adreno_hw_fence_entry *entry, *tmp;
	int ret = 0;

	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_list, node) {

		ret = gen8_send_hw_fence_hfi_wait_ack(adreno_dev, entry,
			HW_FENCE_FLAG_SKIP_MEMSTORE);
		if (ret)
			break;

		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}

	if (ret) {
		move_detached_context_hardware_fences(adreno_dev, drawctxt);
		gmu_core_fault_snapshot(KGSL_DEVICE(adreno_dev), GMU_FAULT_HW_FENCE);
		gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
	}

	return ret;
}

/**
 * check_detached_context_hardware_fences - When this context has been un-registered with the GMU,
 * make sure all the hardware fences(that were sent to GMU) for this context have been sent to
 * TxQueue. Also, send any hardware fences (to GMU) that were not yet dispatched to the GMU. In case
 * of an error, move the pending hardware fences to detached hardware fence list, log the error,
 * take a snapshot and trigger recovery.
 */
static int check_detached_context_hardware_fences(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hw_fence_entry *entry, *tmp;
	int ret = 0;

	/* We don't need the drawctxt lock because this context has been detached */
	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_inflight_list, node) {
		struct gmu_context_queue_header *hdr =  drawctxt->gmu_context_queue.hostptr;

		if ((timestamp_cmp((u32)entry->cmd.ts, hdr->out_fence_ts) > 0)) {
			dev_err(GMU_PDEV_DEV(device),
				"detached ctx:%d has unsignaled fence ts:%d retired:%d\n",
				drawctxt->base.id, (u32)entry->cmd.ts, hdr->out_fence_ts);
			ret = -EINVAL;
			goto fault;
		}
		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}

	/* Send hardware fences (to TxQueue) that were not dispatched to GMU */
	return drain_context_hw_fence_gmu(adreno_dev, drawctxt);

fault:
	move_detached_context_hardware_fences(adreno_dev, drawctxt);
	gmu_core_fault_snapshot(device, GMU_FAULT_HW_FENCE);
	gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);

	return ret;
}

static inline int setup_hw_fence_info_cmd(struct adreno_device *adreno_dev,
	struct adreno_hw_fence_entry *entry)
{
	struct kgsl_sync_fence *kfence = entry->kfence;
	int ret;

	ret = CMD_MSG_HDR(entry->cmd, H2F_MSG_HW_FENCE_INFO);
	if (ret)
		return ret;

	ret = kgsl_hw_fence_create(KGSL_DEVICE(adreno_dev), kfence);
	if (ret)
		return ret;

	entry->cmd.gmu_ctxt_id = entry->drawctxt->base.id;
	entry->cmd.ctxt_id = kfence->fence.context;
	entry->cmd.ts = kfence->fence.seqno;

	entry->cmd.hash_index = kfence->hw_fence_index;

	return 0;
}

/*
 * gen8_send_hw_fence_hfi_wait_ack - This function is used in cases where multiple hardware fences
 * are to be sent to GMU. Hence, we must send them one by one to avoid overwhelming the GMU with
 * mutliple fences in a short span of time.
 */
int gen8_send_hw_fence_hfi_wait_ack(struct adreno_device *adreno_dev,
	struct adreno_hw_fence_entry *entry, u64 flags)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret = 0;
	u32 seqnum;

	/* Device mutex is necessary to ensure only one hardware fence ack is being waited for */
	if (WARN_ON(!mutex_is_locked(&device->mutex)))
		return -EINVAL;

	spin_lock(&hfi->hw_fence.lock);

	init_completion(&gen8_hw_fence_ack.complete);

	entry->cmd.flags |= flags;
	seqnum = atomic_inc_return(&hfi->hw_fence.seqnum);
	entry->cmd.hdr = MSG_HDR_SET_SEQNUM_SIZE(entry->cmd.hdr, seqnum, sizeof(entry->cmd) >> 2);

	gen8_hw_fence_ack.sent_hdr = entry->cmd.hdr;

	/*
	 * We don't need to increment the unack count here as we are waiting for the ack for
	 * this fence before sending another hardware fence.
	 */
	ret = gen8_hfi_cmdq_write(adreno_dev, (u32 *)&entry->cmd, sizeof(entry->cmd));

	spin_unlock(&hfi->hw_fence.lock);

	if (!ret)
		ret = adreno_hwsched_wait_ack_completion(adreno_dev,
			GMU_PDEV_DEV(device), &gen8_hw_fence_ack,
			gen8_hwsched_process_msgq);

	memset(&gen8_hw_fence_ack, 0x0, sizeof(gen8_hw_fence_ack));
	return ret;
}

#define DRAWCTXT_SLOT_AVAILABLE(count)  \
	((count + 1) < (HW_FENCE_QUEUE_SIZE / sizeof(struct hfi_hw_fence_info)))

/**
 * allocate_hw_fence_entry - Allocate an entry to keep track of a hardware fence. This is free'd
 * when we know GMU has sent this fence to the TxQueue
 */
static struct adreno_hw_fence_entry *allocate_hw_fence_entry(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, struct kgsl_sync_fence *kfence)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct adreno_hw_fence_entry *entry;

	if (!DRAWCTXT_SLOT_AVAILABLE(drawctxt->hw_fence_count))
		return NULL;

	if (!_kgsl_context_get(&drawctxt->base))
		return NULL;

	entry = kmem_cache_zalloc(hwsched->hw_fence_cache, GFP_ATOMIC);
	if (!entry) {
		kgsl_context_put_deferred(&drawctxt->base);
		return NULL;
	}

	entry->kfence = kfence;
	entry->drawctxt = drawctxt;

	if (setup_hw_fence_info_cmd(adreno_dev, entry)) {
		kmem_cache_free(hwsched->hw_fence_cache, entry);
		kgsl_context_put_deferred(&drawctxt->base);
		return NULL;
	}

	dma_fence_get(&kfence->fence);

	drawctxt->hw_fence_count++;
	atomic_inc(&hwsched->hw_fence_count);

	INIT_LIST_HEAD(&entry->node);
	INIT_LIST_HEAD(&entry->reset_node);
	return entry;
}

/**
 * drawctxt_queue_hw_fence - Add a hardware fence to draw context's hardware fence list and make
 * sure the list remains sorted (with the fence with the largest timestamp at the end)
 */
static void drawctxt_queue_hw_fence(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, struct kgsl_sync_fence *kfence)
{
	struct adreno_hw_fence_entry *entry = NULL;
	struct adreno_hw_fence_entry *new = allocate_hw_fence_entry(adreno_dev, drawctxt, kfence);
	u32 ts = kfence->timestamp;

	if (!new)
		return;

	/* Walk the list backwards to find the right spot for this fence */
	list_for_each_entry_reverse(entry, &drawctxt->hw_fence_list, node) {
		if (timestamp_cmp(ts, (u32)entry->cmd.ts) > 0)
			break;
	}

	list_add(&new->node, &entry->node);
}

static bool _hw_fence_end_sleep(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	bool ret;

	spin_lock(&hfi->hw_fence.lock);
	ret = !test_bit(GEN8_HWSCHED_HW_FENCE_SLEEP_BIT, &hfi->hw_fence.flags);
	spin_unlock(&hfi->hw_fence.lock);

	return ret;
}

/**
 * _hw_fence_sleep() - Check if the thread needs to sleep until the hardware fence unack count
 * drops to a desired threshold.
 *
 * Return: negative error code if the thread was woken up by a signal, or the context became bad in
 * the meanwhile, or the hardware fence unack count hasn't yet dropped to a desired threshold, or
 * if fault recovery is imminent.
 * Otherwise, return 0.
 */
static int _hw_fence_sleep(struct adreno_device *adreno_dev, struct adreno_context *drawctxt)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	int ret = 0;

	if (!test_bit(GEN8_HWSCHED_HW_FENCE_SLEEP_BIT, &hfi->hw_fence.flags))
		return 0;

	spin_unlock(&hfi->hw_fence.lock);
	spin_unlock(&drawctxt->lock);

	ret = wait_event_interruptible(hfi->hw_fence.unack_wq,
		_hw_fence_end_sleep(adreno_dev));

	spin_lock(&drawctxt->lock);
	spin_lock(&hfi->hw_fence.lock);

	/*
	 * If the thread received a signal, or the context became bad in the meanwhile or the limit
	 * is still not settled, then return error to avoid creating this hardware fence
	 */
	if ((ret == -ERESTARTSYS) || kgsl_context_is_bad(&drawctxt->base) ||
		test_bit(GEN8_HWSCHED_HW_FENCE_MAX_BIT, &hfi->hw_fence.flags))
		return -EINVAL;

	/*
	 * If fault recovery is imminent then return error code to avoid creating new hardware
	 * fences until recovery is complete
	 */
	if (test_bit(GEN8_HWSCHED_HW_FENCE_ABORT_BIT, &hfi->hw_fence.flags))
		return -EBUSY;

	return ret;
}

void gen8_hwsched_create_hw_fence(struct adreno_device *adreno_dev,
	struct kgsl_sync_fence *kfence)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_sync_timeline *ktimeline = kfence->parent;
	struct kgsl_context *context = ktimeline->context;
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct adreno_hw_fence_entry *entry = NULL;
	/* Only allow a single log in a second */
	static DEFINE_RATELIMIT_STATE(_rs, HZ, 1);
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);
	u32 retired = 0;
	int ret = 0;

	spin_lock(&drawctxt->lock);
	spin_lock(&hw_hfi->hw_fence.lock);

	/*
	 * If we create a hardware fence and this context is going away, we may never dispatch
	 * this fence to the GMU. Hence, avoid creating a hardware fence if context is going away.
	 */
	if (kgsl_context_is_bad(context))
		goto done;

	/* If recovery is imminent, then do not create a hardware fence */
	if (test_bit(GEN8_HWSCHED_HW_FENCE_ABORT_BIT, &hw_hfi->hw_fence.flags))
		goto done;

	ret = _hw_fence_sleep(adreno_dev, drawctxt);
	if (ret)
		goto done;

	/*
	 * If this ts hasn't been submitted yet, then store it in the drawctxt hardware fence
	 * list and return. This fence will be sent to GMU when this ts is dispatched to GMU.
	 */
	if (timestamp_cmp(kfence->timestamp, drawctxt->gmu_hw_fence_ready_ts) > 0) {
		drawctxt_queue_hw_fence(adreno_dev, drawctxt, kfence);
		goto done;
	}

	kgsl_readtimestamp(device, context, KGSL_TIMESTAMP_RETIRED, &retired);

	/*
	 * Check if timestamp is retired. If we are in SLUMBER at this point, the timestamp is
	 * guaranteed to be retired. This way, we don't need the device mutex to check the device
	 * state explicitly.
	 */
	if (timestamp_cmp(retired, kfence->timestamp) >= 0) {
		kgsl_sync_timeline_signal(ktimeline, kfence->timestamp);
		goto done;
	}

	entry = allocate_hw_fence_entry(adreno_dev, drawctxt, kfence);
	if (!entry)
		goto done;

	/*
	 * If timestamp is not retired then GMU must already be powered up. This is because SLUMBER
	 * thread has to wait for hardware fence spinlock to make sure the hardware fence unack
	 * count is zero.
	 */
	ret = _send_hw_fence_no_ack(adreno_dev, entry);
	if (ret) {
		if (__ratelimit(&_rs))
			dev_err(GMU_PDEV_DEV(device),
				"hw fence for ctx:%d ts:%d ret:%d may not be destroyed\n",
				kfence->context_id, kfence->timestamp, ret);
		kgsl_hw_fence_destroy(kfence);
		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
		goto done;
	}

	list_add_tail(&entry->node, &drawctxt->hw_fence_inflight_list);

done:
	spin_unlock(&hw_hfi->hw_fence.lock);
	spin_unlock(&drawctxt->lock);
}

/**
 * setup_hw_fence_deferred_ctxt - The hardware fence(s) from this context couldn't be sent to the
 * GMU because the hardware fence unack count reached a threshold. Hence, setup this context such
 * that these hardware fences are sent to the GMU when the unack count drops to a desired threshold.
 */
static void setup_hw_fence_deferred_ctxt(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, u32 ts)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);

	if (!_kgsl_context_get(&drawctxt->base))
		return;

	hfi->hw_fence.defer_drawctxt = drawctxt;
	hfi->hw_fence.defer_ts = ts;
	/*
	 * Increment the active count so that device doesn't get powered off until this fence has
	 * been sent to GMU
	 */
	gen8_hwsched_active_count_get(adreno_dev);
}

/**
 * process_hw_fence_queue - This function walks the draw context's list of hardware fences
 * and sends the ones which have a timestamp less than or equal to the timestamp that just
 * got submitted to the GMU.
 */
static void process_hw_fence_queue(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, u32 ts)
{
	struct adreno_hw_fence_entry *entry = NULL, *next;
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	int ret = 0;

	/* This list is sorted with smallest timestamp at head and highest timestamp at tail */
	list_for_each_entry_safe(entry, next, &drawctxt->hw_fence_list, node) {

		if (timestamp_cmp((u32)entry->cmd.ts, ts) > 0)
			return;

		spin_lock(&hfi->hw_fence.lock);

		if (test_bit(GEN8_HWSCHED_HW_FENCE_MAX_BIT, &hfi->hw_fence.flags)) {
			setup_hw_fence_deferred_ctxt(adreno_dev, drawctxt, ts);
			spin_unlock(&hfi->hw_fence.lock);
			return;
		}

		ret = _send_hw_fence_no_ack(adreno_dev, entry);

		spin_unlock(&hfi->hw_fence.lock);

		if (ret)
			return;

		/*
		 * A fence that is sent to GMU must be added to the drawctxt->hw_fence_inflight_list
		 * so that we can keep track of when GMU sends it to the TxQueue
		 */
		list_move_tail(&entry->node, &drawctxt->hw_fence_inflight_list);
	}
}

int gen8_hwsched_submit_drawobj(struct adreno_device *adreno_dev, struct kgsl_drawobj *drawobj)
{
	int ret = 0;
	u32 cmd_sizebytes, seqnum;
	struct kgsl_drawobj_cmd *cmdobj = NULL;
	struct hfi_submit_cmd *cmd;
	struct adreno_submit_time time = {0};
	struct adreno_context *drawctxt = ADRENO_CONTEXT(drawobj->context);
	static void *cmdbuf;
	struct gmu_context_queue_header *hdr = NULL;

	if (cmdbuf == NULL) {
		struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

		cmdbuf = devm_kzalloc(&device->pdev->dev, HFI_MAX_MSG_SIZE,
				GFP_KERNEL);
		if (!cmdbuf)
			return -ENOMEM;
	}

	ret = hfi_context_register(adreno_dev, drawobj->context);
	if (ret)
		return ret;

	if ((drawobj->type & SYNCOBJ_TYPE) != 0)
		return _submit_hw_fence(adreno_dev, drawobj, cmdbuf);

	cmdobj = CMDOBJ(drawobj);

	/*
	 * If the MARKER object is retired, it doesn't need to be dispatched to GMU. Simply trigger
	 * any pending fences that are less than/equal to this object's timestamp.
	 */
	if (test_bit(CMDOBJ_MARKER_EXPIRED, &cmdobj->priv)) {
		spin_lock(&drawctxt->lock);
		process_hw_fence_queue(adreno_dev, drawctxt, drawobj->timestamp);
		spin_unlock(&drawctxt->lock);
		return 0;
	}

	/* Add a *issue_ib struct for each IB */
	if (cmdobj->numibs > HWSCHED_MAX_DISPATCH_NUMIBS ||
		test_bit(CMDOBJ_SKIP, &cmdobj->priv))
		cmd_sizebytes = sizeof(*cmd);
	else
		cmd_sizebytes = sizeof(*cmd) +
			(sizeof(struct hfi_issue_ib) * cmdobj->numibs);

	if (WARN_ON(cmd_sizebytes > HFI_MAX_MSG_SIZE))
		return -EMSGSIZE;

	memset(cmdbuf, 0x0, cmd_sizebytes);

	cmd = cmdbuf;

	cmd->ctxt_id = drawobj->context->id;
	cmd->flags = HFI_CTXT_FLAG_NOTIFY;
	if (drawobj->flags & KGSL_DRAWOBJ_END_OF_FRAME)
		cmd->flags |= CMDBATCH_EOF;

	cmd->ts = drawobj->timestamp;

	if (test_bit(CMDOBJ_SKIP, &cmdobj->priv))
		goto skipib;

	populate_ibs(adreno_dev, cmd, cmdobj);

	if ((drawobj->flags & KGSL_DRAWOBJ_PROFILING) &&
		cmdobj->profiling_buf_entry) {

		time.drawobj = drawobj;

		cmd->profile_gpuaddr_lo =
			lower_32_bits(cmdobj->profiling_buffer_gpuaddr);
		cmd->profile_gpuaddr_hi =
			upper_32_bits(cmdobj->profiling_buffer_gpuaddr);

		/* Indicate to GMU to do user profiling for this submission */
		cmd->flags |= CMDBATCH_PROFILING;
	}

skipib:
	adreno_drawobj_set_constraint(KGSL_DEVICE(adreno_dev), drawobj);

	cmd->hdr = CREATE_MSG_HDR(H2F_MSG_ISSUE_CMD, HFI_MSG_CMD);
	seqnum = atomic_inc_return(&adreno_dev->hwsched.submission_seqnum);
	cmd->hdr = MSG_HDR_SET_SEQNUM_SIZE(cmd->hdr, seqnum, cmd_sizebytes >> 2);

	ret = adreno_gmu_context_queue_write(adreno_dev, &drawctxt->gmu_context_queue,
		(u32 *)cmd, cmd_sizebytes, drawobj, &time);
	if (ret)
		return ret;

	hdr = drawctxt->gmu_context_queue.hostptr;
	/* The last sync object has been retired by the GMU */
	if (timestamp_cmp(hdr->sync_obj_ts, drawctxt->syncobj_timestamp) >= 0)
		/* Send interrupt to GMU to receive the message */
		gmu_core_regwrite(KGSL_DEVICE(adreno_dev), GEN8_GMUCX_HOST2GMU_INTR_SET,
			DISPQ_IRQ_BIT(get_irq_bit(adreno_dev, drawobj)));
	else
		gmu_core_regwrite(KGSL_DEVICE(adreno_dev), GEN8_GMUCX_HOST2GMU_INTR_SET,
			DISPQ_SYNC_IRQ_BIT(get_irq_bit(adreno_dev, drawobj)));

	drawctxt->internal_timestamp = drawobj->timestamp;

	spin_lock(&drawctxt->lock);
	process_hw_fence_queue(adreno_dev, drawctxt, drawobj->timestamp);
	/*
	 * We need to update the gmu_hw_fence_ready_ts while holding the drawctxt lock since we
	 * have to check it in the hardware fence creation path, where we are not taking the
	 * device mutex.
	 */
	drawctxt->gmu_hw_fence_ready_ts = drawobj->timestamp;
	spin_unlock(&drawctxt->lock);

	return 0;
}

int gen8_hwsched_send_recurring_cmdobj(struct adreno_device *adreno_dev,
	struct kgsl_drawobj_cmd *cmdobj)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_drawobj *drawobj = DRAWOBJ(cmdobj);
	struct hfi_submit_cmd *cmd;
	struct kgsl_memobj_node *ib;
	u32 cmd_sizebytes;
	int ret;
	static bool active;

	if (adreno_gpu_halt(adreno_dev) || adreno_gpu_fault(adreno_dev))
		return -EBUSY;

	if (test_bit(CMDOBJ_RECURRING_STOP, &cmdobj->priv)) {
		cmdobj->numibs = 0;
	} else {
		list_for_each_entry(ib, &cmdobj->cmdlist, node)
			cmdobj->numibs++;
	}

	if (cmdobj->numibs > HWSCHED_MAX_IBS)
		return -EINVAL;

	if (cmdobj->numibs > HWSCHED_MAX_DISPATCH_NUMIBS)
		cmd_sizebytes = sizeof(*cmd);
	else
		cmd_sizebytes = sizeof(*cmd) +
			(sizeof(struct hfi_issue_ib) * cmdobj->numibs);

	if (WARN_ON(cmd_sizebytes > HFI_MAX_MSG_SIZE))
		return -EMSGSIZE;

	cmd = kzalloc(cmd_sizebytes, GFP_KERNEL);
	if (cmd == NULL)
		return -ENOMEM;

	if (test_bit(CMDOBJ_RECURRING_START, &cmdobj->priv)) {
		if (!active) {
			ret = adreno_active_count_get(adreno_dev);
			if (ret) {
				kfree(cmd);
				return ret;
			}
			active = true;
		}
		cmd->flags |= CMDBATCH_RECURRING_START;
		populate_ibs(adreno_dev, cmd, cmdobj);
	} else
		cmd->flags |= CMDBATCH_RECURRING_STOP;

	cmd->ctxt_id = drawobj->context->id;

	ret = hfi_context_register(adreno_dev, drawobj->context);
	if (ret) {
		adreno_active_count_put(adreno_dev);
		active = false;
		kfree(cmd);
		return ret;
	}

	cmd->hdr = CREATE_MSG_HDR(H2F_MSG_ISSUE_RECURRING_CMD, HFI_MSG_CMD);

	ret = gen8_hfi_send_cmd_async(adreno_dev, cmd, sizeof(*cmd));

	kfree(cmd);

	if (ret) {
		adreno_active_count_put(adreno_dev);
		active = false;
		return ret;
	}

	if (test_bit(CMDOBJ_RECURRING_STOP, &cmdobj->priv)) {
		adreno_hwsched_retire_cmdobj(hwsched, hwsched->recurring_cmdobj);
		del_timer_sync(&hwsched->lsr_timer);
		hwsched->recurring_cmdobj = NULL;
		if (active)
			adreno_active_count_put(adreno_dev);
		active = false;
		return ret;
	}

	hwsched->recurring_cmdobj = cmdobj;
	/* Star LSR timer for power stats collection */
	mod_timer(&hwsched->lsr_timer, jiffies + msecs_to_jiffies(10));
	return ret;
}

/* We don't want to unnecessarily wake the GMU to trigger hardware fences */
static void drain_context_hw_fence_cpu(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct adreno_hw_fence_entry *entry, *tmp;

	/*
	 * Triggering these fences from HLOS may send interrupts to soccp. Hence, vote for soccp
	 * here
	 */
	gen8_hwsched_soccp_vote(adreno_dev, true);

	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_list, node) {

		kgsl_hw_fence_trigger_cpu(KGSL_DEVICE(adreno_dev), entry->kfence);

		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}

	gen8_hwsched_soccp_vote(adreno_dev, false);
}

static void drain_context_hw_fences(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);

	if (list_empty(&drawctxt->hw_fence_list))
		return;

	if (test_bit(GMU_PRIV_GPU_STARTED, &gmu->flags))
		drain_context_hw_fence_gmu(adreno_dev, drawctxt);
	else
		drain_context_hw_fence_cpu(adreno_dev, drawctxt);
}

static void trigger_context_unregister_fault(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	gmu_core_fault_snapshot(KGSL_DEVICE(adreno_dev), GMU_FAULT_CTX_UNREGISTER);

	/* Make sure we send all fences from this context to the TxQueue after recovery */
	move_detached_context_hardware_fences(adreno_dev, drawctxt);
	gen8_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
}

static int send_context_unregister_hfi(struct adreno_device *adreno_dev,
	struct kgsl_context *context, u32 ts)
{
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct pending_cmd pending_ack;
	struct hfi_unregister_ctxt_cmd cmd;
	u32 seqnum;
	int ret;

	/* Only send HFI if device is not in SLUMBER */
	if (!context->gmu_registered ||
		!test_bit(GMU_PRIV_GPU_STARTED, &gmu->flags)) {
		drain_context_hw_fences(adreno_dev, drawctxt);
		return 0;
	}

	ret = CMD_MSG_HDR(cmd, H2F_MSG_UNREGISTER_CONTEXT);
	if (ret)
		return ret;

	cmd.ctxt_id = context->id,
	cmd.ts = ts,

	/*
	 * Although we know device is powered on, we can still enter SLUMBER
	 * because the wait for ack below is done without holding the mutex. So
	 * take an active count before releasing the mutex so as to avoid a
	 * concurrent SLUMBER sequence while GMU is un-registering this context.
	 */
	ret = gen8_hwsched_active_count_get(adreno_dev);
	if (ret) {
		trigger_context_unregister_fault(adreno_dev, drawctxt);
		return ret;
	}

	seqnum = atomic_inc_return(&gmu->hfi.seqnum);
	cmd.hdr = MSG_HDR_SET_SEQNUM_SIZE(cmd.hdr, seqnum, sizeof(cmd) >> 2);
	add_waiter(hfi, cmd.hdr, &pending_ack);

	ret = gen8_hfi_cmdq_write(adreno_dev, (u32 *)&cmd, sizeof(cmd));
	if (ret) {
		trigger_context_unregister_fault(adreno_dev, drawctxt);
		goto done;
	}

	ret = adreno_hwsched_ctxt_unregister_wait_completion(adreno_dev,
			gmu_pdev_dev, &pending_ack,
			gen8_hwsched_process_msgq, &cmd);
	if (ret) {
		trigger_context_unregister_fault(adreno_dev, drawctxt);
		goto done;
	}

	ret = check_detached_context_hardware_fences(adreno_dev, drawctxt);
	if (!ret)
		ret = check_ack_failure(adreno_dev, &pending_ack);

done:
	adreno_active_count_put(adreno_dev);
	del_waiter(hfi, &pending_ack);

	return ret;
}

void gen8_hwsched_context_detach(struct adreno_context *drawctxt)
{
	struct kgsl_context *context = &drawctxt->base;
	struct kgsl_device *device = context->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret = 0;

	mutex_lock(&device->mutex);

	ret = send_context_unregister_hfi(adreno_dev, context,
		drawctxt->internal_timestamp);

	if (!ret) {
		kgsl_sharedmem_writel(device->memstore,
			KGSL_MEMSTORE_OFFSET(context->id, soptimestamp),
			drawctxt->timestamp);

		kgsl_sharedmem_writel(device->memstore,
			KGSL_MEMSTORE_OFFSET(context->id, eoptimestamp),
			drawctxt->timestamp);

		adreno_profile_process_results(adreno_dev);
	}

	context->gmu_registered = false;

	mutex_unlock(&device->mutex);
}

u32 gen8_hwsched_preempt_count_get(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret, preempt_count = 0;

	ret = gmu_core_get_vrb_register(gmu->vrb, VRB_PREEMPT_COUNT_TOTAL, &preempt_count);
	if (ret)
		return 0;

	if ((preempt_count != 0) || (device->state != KGSL_STATE_ACTIVE))
		return preempt_count;

	return gen8_hwsched_hfi_get_value(adreno_dev, HFI_VALUE_PREEMPT_COUNT);
}

void gen8_hwsched_context_destroy(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	if (drawctxt->gmu_context_queue.gmuaddr)
		gen8_free_gmu_block(to_gen8_gmu(adreno_dev), &drawctxt->gmu_context_queue);

	if (drawctxt->gmu_hw_fence_queue.gmuaddr)
		gen8_free_gmu_block(to_gen8_gmu(adreno_dev), &drawctxt->gmu_hw_fence_queue);
}

int gen8_hwsched_disable_hw_fence_throttle(struct adreno_device *adreno_dev)
{
	struct gen8_hwsched_hfi *hfi = to_gen8_hwsched_hfi(adreno_dev);
	struct adreno_context *drawctxt = NULL;
	u32 ts = 0;
	int ret = 0;

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &adreno_dev->hwsched.flags))
		return 0;

	spin_lock(&hfi->hw_fence.lock);

	drawctxt = hfi->hw_fence.defer_drawctxt;
	ts = hfi->hw_fence.defer_ts;

	spin_unlock(&hfi->hw_fence.lock);

	if (!drawctxt)
		goto done;

	ret = process_hw_fence_deferred_ctxt(adreno_dev, drawctxt, ts);

	kgsl_context_put(&drawctxt->base);
	adreno_active_count_put(adreno_dev);

done:
	_disable_hw_fence_throttle(adreno_dev, true);

	return ret;
}

void *gen8_hwsched_get_rb_hostptr(struct adreno_device *adreno_dev,
	u64 gpuaddr, u32 size)
{
	struct gen8_hwsched_hfi *hw_hfi = to_gen8_hwsched_hfi(adreno_dev);
	u64 offset;
	u32 i;

	for (i = 0; i < hw_hfi->mem_alloc_entries; i++) {
		struct kgsl_memdesc *md = hw_hfi->mem_alloc_table[i].md;

		if (kgsl_gpuaddr_in_memdesc(md, gpuaddr, size)) {
			offset = gpuaddr - md->gpuaddr;
			return md->hostptr + offset;
		}
	}

	return NULL;
}
