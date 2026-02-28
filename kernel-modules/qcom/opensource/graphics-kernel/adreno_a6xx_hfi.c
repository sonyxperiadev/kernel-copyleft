// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/nvmem-consumer.h>

#include "adreno.h"
#include "adreno_a6xx.h"
#include "adreno_a6xx_hfi.h"
#include "kgsl_device.h"
#include "kgsl_trace.h"

/* Below section is for all structures related to HFI queues */
#define HFI_QUEUE_MAX HFI_QUEUE_DEFAULT_CNT

/* Total header sizes + queue sizes + 16 for alignment */
#define HFIMEM_SIZE (sizeof(struct hfi_queue_table) + 16 + \
		(HFI_QUEUE_SIZE * HFI_QUEUE_MAX))

struct a6xx_hfi *to_a6xx_hfi(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);

	return &gmu->hfi;
}

/* Size in below functions are in unit of dwords */
int a6xx_hfi_queue_read(struct a6xx_gmu_device *gmu, uint32_t queue_idx,
		unsigned int *output, unsigned int max_size)
{
	struct kgsl_device *device = KGSL_DEVICE(a6xx_gmu_to_adreno(gmu));
	struct kgsl_memdesc *mem_addr = gmu->hfi.hfi_mem;
	struct hfi_queue_table *tbl = mem_addr->hostptr;
	struct hfi_queue_header *hdr = &tbl->qhdr[queue_idx];
	uint32_t *queue;
	uint32_t msg_hdr;
	uint32_t i, read;
	uint32_t size;
	int result = 0;

	if (hdr->status == HFI_QUEUE_STATUS_DISABLED)
		return -EINVAL;

	if (hdr->read_index == hdr->write_index)
		return -ENODATA;

	/* Clear the output data before populating */
	memset(output, 0, max_size);

	queue = HOST_QUEUE_START_ADDR(mem_addr, queue_idx);
	msg_hdr = queue[hdr->read_index];
	size = MSG_HDR_GET_SIZE(msg_hdr);

	if (size > (max_size >> 2)) {
		dev_err(GMU_PDEV_DEV(device),
		"HFI message too big: hdr:0x%x rd idx=%d\n",
			msg_hdr, hdr->read_index);
		result = -EMSGSIZE;
		goto done;
	}

	read = hdr->read_index;

	if (read < hdr->queue_size) {
		for (i = 0; i < size && i < (max_size >> 2); i++) {
			output[i] = queue[read];
			read = (read + 1)%hdr->queue_size;
		}
		result = size;
	} else {
		/* In case FW messed up */
		dev_err(GMU_PDEV_DEV(device),
			"Read index %d greater than queue size %d\n",
			hdr->read_index, hdr->queue_size);
		result = -ENODATA;
	}

	if (GMU_VER_MAJOR(gmu->ver.hfi) >= 2)
		read = ALIGN(read, SZ_4) % hdr->queue_size;

	/* For acks, trace the packet for which this ack was sent */
	if (MSG_HDR_GET_TYPE(msg_hdr) == HFI_MSG_ACK)
		trace_kgsl_hfi_receive(MSG_HDR_GET_ID(output[1]),
			MSG_HDR_GET_SIZE(output[1]),
			MSG_HDR_GET_SEQNUM(output[1]));
	else
		trace_kgsl_hfi_receive(MSG_HDR_GET_ID(msg_hdr),
			MSG_HDR_GET_SIZE(msg_hdr), MSG_HDR_GET_SEQNUM(msg_hdr));

	hfi_update_read_idx(hdr, read);

done:
	return result;
}

/* Size in below functions are in unit of dwords */
int a6xx_hfi_queue_write(struct adreno_device *adreno_dev, uint32_t queue_idx,
		uint32_t *msg, u32 size_bytes)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct hfi_queue_table *tbl = gmu->hfi.hfi_mem->hostptr;
	struct hfi_queue_header *hdr = &tbl->qhdr[queue_idx];
	uint32_t *queue;
	uint32_t i, write_idx, read_idx, empty_space;
	uint32_t size_dwords = size_bytes >> 2;
	u32 align_size = ALIGN(size_dwords, SZ_4);
	uint32_t id = MSG_HDR_GET_ID(*msg);

	if (hdr->status == HFI_QUEUE_STATUS_DISABLED || !IS_ALIGNED(size_bytes, sizeof(u32)))
		return -EINVAL;

	queue = HOST_QUEUE_START_ADDR(gmu->hfi.hfi_mem, queue_idx);

	write_idx = hdr->write_index;
	read_idx = hdr->read_index;

	empty_space = (write_idx >= read_idx) ?
			(hdr->queue_size - (write_idx - read_idx))
			: (read_idx - write_idx);

	if (empty_space <= align_size)
		return -ENOSPC;

	for (i = 0; i < size_dwords; i++) {
		queue[write_idx] = msg[i];
		write_idx = (write_idx + 1) % hdr->queue_size;
	}

	/* Cookify any non used data at the end of the write buffer */
	if (GMU_VER_MAJOR(gmu->ver.hfi) >= 2) {
		for (; i < align_size; i++) {
			queue[write_idx] = 0xFAFAFAFA;
			write_idx = (write_idx + 1) % hdr->queue_size;
		}
	}

	trace_kgsl_hfi_send(id, size_dwords, MSG_HDR_GET_SEQNUM(*msg));

	hfi_update_write_idx(&hdr->write_index, write_idx);

	return 0;
}

int a6xx_hfi_cmdq_write(struct adreno_device *adreno_dev, u32 *msg, u32 size_bytes)
{
	int ret;

	ret = a6xx_hfi_queue_write(adreno_dev, HFI_CMD_ID, msg, size_bytes);

	/*
	 * Memory barrier to make sure packet and write index are written before
	 * an interrupt is raised
	 */
	wmb();

	/* Send interrupt to GMU to receive the message */
	if (!ret)
		gmu_core_regwrite(KGSL_DEVICE(adreno_dev),
			A6XX_GMU_HOST2GMU_INTR_SET,
			0x1);

	return ret;
}

/* Sizes of the queue and message are in unit of dwords */
static void init_queues(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct kgsl_memdesc *mem_addr = gmu->hfi.hfi_mem;
	int i;
	struct hfi_queue_table *tbl;
	struct hfi_queue_header *hdr;
	struct {
		unsigned int idx;
		unsigned int pri;
		unsigned int status;
	} queue[HFI_QUEUE_MAX] = {
		{ HFI_CMD_IDX, HFI_CMD_PRI, HFI_QUEUE_STATUS_ENABLED },
		{ HFI_MSG_IDX, HFI_MSG_PRI, HFI_QUEUE_STATUS_ENABLED },
		{ HFI_DBG_IDX, HFI_DBG_PRI, HFI_QUEUE_STATUS_ENABLED },
	};

	/*
	 * Overwrite the queue IDs for A630, A615 and A616 as they use
	 * legacy firmware. Legacy firmware has different queue IDs for
	 * message, debug and dispatch queues (dispatch queues aren't used
	 * on these targets so the queue idx value update is not needed).
	 */
	if (adreno_is_a630(adreno_dev) || adreno_is_a615_family(adreno_dev)) {
		queue[HFI_MSG_ID].idx = HFI_MSG_IDX_LEGACY;
		queue[HFI_DBG_ID].idx = HFI_DBG_IDX_LEGACY;
	}

	/* Fill Table Header */
	tbl = mem_addr->hostptr;
	tbl->qtbl_hdr.version = 0;
	tbl->qtbl_hdr.size = sizeof(struct hfi_queue_table) >> 2;
	tbl->qtbl_hdr.qhdr0_offset = sizeof(struct hfi_queue_table_header) >> 2;
	tbl->qtbl_hdr.qhdr_size = sizeof(struct hfi_queue_header) >> 2;
	tbl->qtbl_hdr.num_q = HFI_QUEUE_MAX;
	tbl->qtbl_hdr.num_active_q = HFI_QUEUE_MAX;

	memset(&tbl->qhdr[0], 0, sizeof(tbl->qhdr));

	/* Fill Individual Queue Headers */
	for (i = 0; i < HFI_QUEUE_MAX; i++) {
		hdr = &tbl->qhdr[i];
		hdr->start_addr = GMU_QUEUE_START_ADDR(mem_addr->gmuaddr, i);
		hdr->type = QUEUE_HDR_TYPE(queue[i].idx, queue[i].pri, 0,  0);
		hdr->status = queue[i].status;
		hdr->queue_size = HFI_QUEUE_SIZE >> 2; /* convert to dwords */
	}
}

int a6xx_hfi_init(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct a6xx_hfi *hfi = &gmu->hfi;

	/* Allocates & maps memory for HFI */
	if (IS_ERR_OR_NULL(hfi->hfi_mem)) {
		hfi->hfi_mem = reserve_gmu_kernel_block(gmu, 0, HFIMEM_SIZE,
			GMU_NONCACHED_KERNEL, 0);
		if (!IS_ERR(hfi->hfi_mem))
			init_queues(adreno_dev);
	}

	return PTR_ERR_OR_ZERO(hfi->hfi_mem);
}

int a6xx_receive_ack_cmd(struct a6xx_gmu_device *gmu, void *rcvd,
	struct pending_cmd *ret_cmd)
{
	struct adreno_device *adreno_dev = a6xx_gmu_to_adreno(gmu);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	uint32_t *ack = rcvd;
	uint32_t hdr = ack[0];
	uint32_t req_hdr = ack[1];

	if (ret_cmd == NULL)
		return -EINVAL;

	if (CMP_HFI_ACK_HDR(ret_cmd->sent_hdr, req_hdr)) {
		memcpy(&ret_cmd->results, ack, MSG_HDR_GET_SIZE(hdr) << 2);
		return 0;
	}

	/* Didn't find the sender, list the waiter */
	dev_err_ratelimited(GMU_PDEV_DEV(device),
		"HFI ACK: Cannot find sender for 0x%8.8x Waiter: 0x%8.8x\n",
		req_hdr, ret_cmd->sent_hdr);

	gmu_core_fault_snapshot(device, GMU_FAULT_HFI_RECIVE_ACK);

	return -ENODEV;
}

static int a6xx_hfi_send_cmd_wait_inline(struct adreno_device *adreno_dev,
	void *data, u32 size_bytes, struct pending_cmd *ret_cmd)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int rc;
	uint32_t *cmd = data;
	struct a6xx_hfi *hfi = &gmu->hfi;
	unsigned int seqnum = atomic_inc_return(&hfi->seqnum);

	*cmd = MSG_HDR_SET_SEQNUM_SIZE(*cmd, seqnum, size_bytes >> 2);
	if (ret_cmd == NULL)
		return a6xx_hfi_cmdq_write(adreno_dev, cmd, size_bytes);

	ret_cmd->sent_hdr = cmd[0];

	rc = a6xx_hfi_cmdq_write(adreno_dev, cmd, size_bytes);
	if (rc)
		return rc;

	rc = gmu_core_timed_poll_check(device, A6XX_GMU_GMU2HOST_INTR_INFO,
			HFI_IRQ_MSGQ_MASK, HFI_RSP_TIMEOUT, HFI_IRQ_MSGQ_MASK);

	if (rc) {
		dev_err(GMU_PDEV_DEV(device),
		"Timed out waiting on ack for 0x%8.8x (id %d, sequence %d)\n",
		cmd[0], MSG_HDR_GET_ID(*cmd), MSG_HDR_GET_SEQNUM(*cmd));
		gmu_core_fault_snapshot(device, GMU_FAULT_SEND_CMD_WAIT_INLINE);
		return rc;
	}

	/* Clear the interrupt */
	gmu_core_regwrite(device, A6XX_GMU_GMU2HOST_INTR_CLR,
		HFI_IRQ_MSGQ_MASK);

	rc = a6xx_hfi_process_queue(gmu, HFI_MSG_ID, ret_cmd);

	return rc;
}

int a6xx_hfi_send_generic_req(struct adreno_device *adreno_dev, void *cmd, u32 size_bytes)
{
	struct pending_cmd ret_cmd;
	int rc;

	memset(&ret_cmd, 0, sizeof(ret_cmd));

	rc = a6xx_hfi_send_cmd_wait_inline(adreno_dev, cmd, size_bytes, &ret_cmd);
	if (rc)
		return rc;

	if (ret_cmd.results[2]) {
		struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

		dev_err(GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev)),
				"HFI ACK failure: Req=0x%8.8X, Result=0x%8.8X\n",
				ret_cmd.results[1],
				ret_cmd.results[2]);
		gmu_core_fault_snapshot(device, GMU_FAULT_HFI_SEND_GENERIC_REQ);
		return -EINVAL;
	}

	return 0;
}

static int a6xx_hfi_send_gmu_init(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct hfi_gmu_init_cmd cmd = {
		.seg_id = 0,
		.dbg_buffer_addr = (unsigned int) gmu->dump_mem->gmuaddr,
		.dbg_buffer_size = (unsigned int) gmu->dump_mem->size,
		.boot_state = 0x1,
	};
	int ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_INIT);
	if (ret)
		return ret;

	return a6xx_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
}

static int a6xx_hfi_get_fw_version(struct adreno_device *adreno_dev,
		uint32_t expected_ver, uint32_t *ver)
{
	struct hfi_fw_version_cmd cmd = {
		.supported_ver = expected_ver,
	};
	int rc;
	struct pending_cmd ret_cmd;

	rc = CMD_MSG_HDR(cmd, H2F_MSG_FW_VER);
	if (rc)
		return rc;

	memset(&ret_cmd, 0, sizeof(ret_cmd));

	rc = a6xx_hfi_send_cmd_wait_inline(adreno_dev, &cmd, sizeof(cmd), &ret_cmd);
	if (rc)
		return rc;

	rc = ret_cmd.results[2];
	if (!rc)
		*ver = ret_cmd.results[3];
	else
		dev_err(GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev)),
			"gmu get fw ver failed with error=%d\n", rc);

	return rc;
}

int a6xx_hfi_send_core_fw_start(struct adreno_device *adreno_dev)
{
	struct hfi_core_fw_start_cmd cmd = {
		.handle = 0x0,
	};
	int ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_CORE_FW_START);
	if (ret)
		return ret;

	return a6xx_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
}

int a6xx_hfi_send_feature_ctrl(struct adreno_device *adreno_dev,
	uint32_t feature, uint32_t enable, uint32_t data)
{
	struct hfi_feature_ctrl_cmd cmd = {
		.feature = feature,
		.enable = enable,
		.data = data,
	};
	int ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_FEATURE_CTRL);
	if (ret)
		return ret;

	ret = a6xx_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
	if (ret)
		dev_err(GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev)), "Unable to %s feature %s (%d)\n",
			enable ? "enable" : "disable", hfi_feature_to_string(feature), feature);
	return ret;
}

int a6xx_hfi_send_set_value(struct adreno_device *adreno_dev,
		u32 type, u32 subtype, u32 data)
{
	struct hfi_set_value_cmd cmd = {
		.type = type,
		.subtype = subtype,
		.data = data,
	};
	int ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_SET_VALUE);
	if (ret)
		return ret;

	ret = a6xx_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
	if (ret)
		dev_err(GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev)),
			"Unable to set HFI Value %d, %d to %d, error = %d\n",
			type, subtype, data, ret);
	return ret;
}

static int a6xx_hfi_send_dcvstbl_v1(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct hfi_dcvstable_cmd *table = &gmu->hfi.dcvs_table;
	struct hfi_dcvstable_v1_cmd cmd = {
		.gpu_level_num = table->gpu_level_num,
		.gmu_level_num = table->gmu_level_num,
	};
	int i, ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_PERF_TBL);
	if (ret)
		return ret;

	for (i = 0; i < table->gpu_level_num; i++) {
		cmd.gx_votes[i].vote = table->gx_votes[i].vote;
		cmd.gx_votes[i].freq = table->gx_votes[i].freq;
	}

	cmd.cx_votes[0].vote = table->cx_votes[0].vote;
	cmd.cx_votes[0].freq = table->cx_votes[0].freq;
	cmd.cx_votes[1].vote = table->cx_votes[1].vote;
	cmd.cx_votes[1].freq = table->cx_votes[1].freq;

	return a6xx_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
}

static int a6xx_hfi_send_test(struct adreno_device *adreno_dev)
{
	struct hfi_test_cmd cmd;
	int ret;

	ret = CMD_MSG_HDR(cmd, H2F_MSG_TEST);
	if (ret)
		return ret;

	cmd.data = 0;

	return a6xx_hfi_send_generic_req(adreno_dev, &cmd, sizeof(cmd));
}

void adreno_a6xx_receive_err_req(struct a6xx_gmu_device *gmu, void *rcvd)
{
	struct kgsl_device *device = KGSL_DEVICE(a6xx_gmu_to_adreno(gmu));
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(ADRENO_DEVICE(device));
	u64 ticks = gpudev->read_alwayson(ADRENO_DEVICE(device));
	struct hfi_err_cmd *cmd = rcvd;

	dev_err(GMU_PDEV_DEV(device), "HFI Error Received: %d %d %.16s\n",
			((cmd->error_code >> 16) & 0xFFFF),
			(cmd->error_code & 0xFFFF),
			(char *) cmd->data);

	KGSL_GMU_CORE_FORCE_PANIC(device->gmu_core.gf_panic,
			GMU_PDEV(device), ticks, GMU_FAULT_F2H_MSG_ERR);
}

void adreno_a6xx_receive_debug_req(struct a6xx_gmu_device *gmu, void *rcvd)
{
	struct kgsl_device *device = KGSL_DEVICE(a6xx_gmu_to_adreno(gmu));
	struct hfi_debug_cmd *cmd = rcvd;

	dev_dbg(GMU_PDEV_DEV(device), "HFI Debug Received: %d %d %d\n",
			cmd->type, cmd->timestamp, cmd->data);
}

static void a6xx_hfi_v1_receiver(struct a6xx_gmu_device *gmu, uint32_t *rcvd,
	struct pending_cmd *ret_cmd)
{
	struct kgsl_device *device = KGSL_DEVICE(a6xx_gmu_to_adreno(gmu));
	/* V1 ACK Handler */
	if (MSG_HDR_GET_TYPE(rcvd[0]) == HFI_V1_MSG_ACK) {
		a6xx_receive_ack_cmd(gmu, rcvd, ret_cmd);
		return;
	}

	/* V1 Request Handler */
	switch (MSG_HDR_GET_ID(rcvd[0])) {
	case F2H_MSG_ERR: /* No Reply */
		adreno_a6xx_receive_err_req(gmu, rcvd);
		break;
	case F2H_MSG_DEBUG: /* No Reply */
		adreno_a6xx_receive_debug_req(gmu, rcvd);
		break;
	default: /* No Reply */
		dev_err(GMU_PDEV_DEV(device),
				"HFI V1 request %d not supported\n",
				MSG_HDR_GET_ID(rcvd[0]));
		break;
	}
}

int a6xx_hfi_process_queue(struct a6xx_gmu_device *gmu,
		uint32_t queue_idx, struct pending_cmd *ret_cmd)
{
	struct kgsl_device *device = KGSL_DEVICE(a6xx_gmu_to_adreno(gmu));
	uint32_t rcvd[MAX_RCVD_SIZE];

	while (a6xx_hfi_queue_read(gmu, queue_idx, rcvd, sizeof(rcvd)) > 0) {
		/* Special case if we're v1 */
		if (GMU_VER_MAJOR(gmu->ver.hfi) < 2) {
			a6xx_hfi_v1_receiver(gmu, rcvd, ret_cmd);
			continue;
		}

		/* V2 ACK Handler */
		if (MSG_HDR_GET_TYPE(rcvd[0]) == HFI_MSG_ACK) {
			int ret = a6xx_receive_ack_cmd(gmu, rcvd, ret_cmd);

			if (ret)
				return ret;
			continue;
		}

		/* V2 Request Handler */
		switch (MSG_HDR_GET_ID(rcvd[0])) {
		case F2H_MSG_ERR: /* No Reply */
			adreno_a6xx_receive_err_req(gmu, rcvd);
			break;
		case F2H_MSG_DEBUG: /* No Reply */
			adreno_a6xx_receive_debug_req(gmu, rcvd);
			break;
		default: /* No Reply */
			dev_err(GMU_PDEV_DEV(device),
				"HFI request %d not supported\n",
				MSG_HDR_GET_ID(rcvd[0]));
			break;
		}
	}

	return 0;
}

static int a6xx_hfi_verify_fw_version(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	const struct adreno_a6xx_core *a6xx_core = to_a6xx_core(adreno_dev);
	struct device *gmu_pdev_dev = GMU_PDEV_DEV(KGSL_DEVICE(adreno_dev));
	int result;
	unsigned int ver, major, minor;

	/* GMU version is already known, so don't waste time finding again */
	if (gmu->ver.core != 0)
		return 0;

	major = a6xx_core->gmu_major;
	minor = a6xx_core->gmu_minor;

	result = a6xx_hfi_get_fw_version(adreno_dev, GMU_VERSION(major, minor, 0),
			&ver);
	if (result) {
		dev_err_once(gmu_pdev_dev,
				"Failed to get FW version via HFI\n");
		return result;
	}

	/* For now, warn once. Could return error later if needed */
	if (major != GMU_VER_MAJOR(ver))
		dev_err_once(gmu_pdev_dev,
				"FW Major Error: Wanted %d, got %d\n",
				major, GMU_VER_MAJOR(ver));

	if (minor > GMU_VER_MINOR(ver))
		dev_err_once(gmu_pdev_dev,
				"FW Minor Error: Wanted < %d, got %d\n",
				GMU_VER_MINOR(ver), minor);

	/* Save the gmu version information */
	gmu->ver.core = ver;

	return 0;
}

int a6xx_hfi_send_bcl_feature_ctrl(struct adreno_device *adreno_dev)
{
	int ret;

	if (!adreno_dev->bcl_enabled)
		return 0;

	ret = a6xx_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_BCL, 1, 0);

	return ret;
}

int a6xx_hfi_send_lm_feature_ctrl(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct hfi_set_value_cmd req;
	u32 slope = 0;
	int ret;

	if (!adreno_dev->lm_enabled)
		return 0;

	memset(&req, 0, sizeof(req));

	nvmem_cell_read_u32(&device->pdev->dev, "isense_slope", &slope);

	ret = CMD_MSG_HDR(req, H2F_MSG_SET_VALUE);
	if (ret)
		return ret;

	req.type = HFI_VALUE_LM_CS0;
	req.subtype = 0;
	req.data = slope;

	ret = a6xx_hfi_send_feature_ctrl(adreno_dev, HFI_FEATURE_LM, 1,
			device->pwrctrl.throttle_mask);

	if (!ret)
		ret = a6xx_hfi_send_generic_req(adreno_dev, &req, sizeof(req));

	return ret;
}

int a6xx_hfi_send_acd_feature_ctrl(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	int ret = 0;

	if (adreno_dev->acd_enabled) {
		ret = a6xx_hfi_send_generic_req(adreno_dev,
			&gmu->hfi.acd_table, sizeof(gmu->hfi.acd_table));
		if (!ret)
			ret = a6xx_hfi_send_feature_ctrl(adreno_dev,
				HFI_FEATURE_ACD, 1, 0);
	}

	return ret;
}

static void reset_hfi_queues(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct kgsl_memdesc *mem_addr = gmu->hfi.hfi_mem;
	struct hfi_queue_table *tbl = mem_addr->hostptr;
	struct hfi_queue_header *hdr;
	unsigned int i;

	/* Flush HFI queues */
	for (i = 0; i < HFI_QUEUE_MAX; i++) {
		hdr = &tbl->qhdr[i];

		if (hdr->status == HFI_QUEUE_STATUS_DISABLED)
			continue;

		hdr->read_index = hdr->write_index;
	}
}

int a6xx_hfi_start(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int result;

	reset_hfi_queues(adreno_dev);

	/* This is legacy HFI message for A630 and A615 family firmware */
	if (adreno_is_a630(adreno_dev) || adreno_is_a615_family(adreno_dev)) {
		result = a6xx_hfi_send_gmu_init(adreno_dev);
		if (result)
			goto err;
	}

	result = a6xx_hfi_verify_fw_version(adreno_dev);
	if (result)
		goto err;

	if (GMU_VER_MAJOR(gmu->ver.hfi) < 2)
		result = a6xx_hfi_send_dcvstbl_v1(adreno_dev);
	else
		result = a6xx_hfi_send_generic_req(adreno_dev,
			&gmu->hfi.dcvs_table, sizeof(gmu->hfi.dcvs_table));
	if (result)
		goto err;

	result = a6xx_hfi_send_generic_req(adreno_dev, &gmu->hfi.bw_table,
			sizeof(gmu->hfi.bw_table));
	if (result)
		goto err;

	/*
	 * If quirk is enabled send H2F_MSG_TEST and tell the GMU
	 * we are sending no more HFIs until the next boot otherwise
	 * send H2F_MSG_CORE_FW_START and features for A640 devices
	 */
	if (GMU_VER_MAJOR(gmu->ver.hfi) >= 2) {
		result = a6xx_hfi_send_acd_feature_ctrl(adreno_dev);
		if (result)
			goto err;

		result = a6xx_hfi_send_lm_feature_ctrl(adreno_dev);
		if (result)
			goto err;

		result = a6xx_hfi_send_bcl_feature_ctrl(adreno_dev);
		if (result)
			goto err;

		result = a6xx_hfi_send_core_fw_start(adreno_dev);
		if (result)
			goto err;
	} else {
		if (ADRENO_QUIRK(adreno_dev, ADRENO_QUIRK_HFI_USE_REG)) {
			result = a6xx_hfi_send_test(adreno_dev);
			if (result)
				goto err;
		}
	}

	set_bit(GMU_PRIV_HFI_STARTED, &gmu->flags);

	result = kgsl_pwrctrl_setup_default_votes(device);

err:
	if (result)
		a6xx_hfi_stop(adreno_dev);

	return result;

}

void a6xx_hfi_stop(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	kgsl_pwrctrl_axi(device, false);

	clear_bit(GMU_PRIV_HFI_STARTED, &gmu->flags);
}

/* HFI interrupt handler */
irqreturn_t a6xx_hfi_irq_handler(int irq, void *data)
{
	struct kgsl_device *device = data;
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(ADRENO_DEVICE(device));
	unsigned int status = 0;

	gmu_core_regread(device, A6XX_GMU_GMU2HOST_INTR_INFO, &status);
	gmu_core_regwrite(device, A6XX_GMU_GMU2HOST_INTR_CLR, HFI_IRQ_MASK);

	if (status & HFI_IRQ_DBGQ_MASK)
		a6xx_hfi_process_queue(gmu, HFI_DBG_ID, NULL);
	if (status & HFI_IRQ_CM3_FAULT_MASK) {
		dev_err_ratelimited(GMU_PDEV_DEV(device),
				"GMU CM3 fault interrupt received\n");
		atomic_set(&gmu->cm3_fault, 1);

		/* make sure other CPUs see the update */
		smp_wmb();
	}
	if (status & ~HFI_IRQ_MASK)
		dev_err_ratelimited(GMU_PDEV_DEV(device),
				"Unhandled HFI interrupts 0x%lx\n",
				status & ~HFI_IRQ_MASK);

	return IRQ_HANDLED;
}
