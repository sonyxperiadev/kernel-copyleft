// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__
#include <linux/sync_file.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-array.h>
#include <linux/file.h>
#include "msm_drv.h"
#include "sde_kms.h"
#include "sde_fence.h"
#include "sde_encoder.h"

#define TIMELINE_VAL_LENGTH		128
#define SPEC_FENCE_FLAG_FENCE_ARRAY	0x10
#define SPEC_FENCE_FLAG_ARRAY_BIND	0x11
#define HW_FENCE_DIR_WRITE_SIZE	0x2
#define HW_FENCE_DIR_WRITE_MASK	0xFFFFFFFF
#define HW_FENCE_HFI_MMAP_DPU_BA	0x200000

/**
 * struct sde_fence - release/retire fence structure
 * @base: base fence structure
 * @ctx: fence context
 * @name: name of each fence- it is fence timeline + commit_count
 * @fence_list: list to associated this fence on timeline/context
 * @fd: fd attached to this fence - debugging purpose.
 * @hwfence_out_ctl: hw ctl for the output fence
 * @hwfence_index: hw fence index for this fence
 * @txq_updated_fence: flag to indicate that a fence has been updated in txq
 */
struct sde_fence {
	struct dma_fence base;
	struct sde_fence_context *ctx;
	char name[SDE_FENCE_NAME_SIZE];
	struct list_head fence_list;
	int fd;
	struct sde_hw_ctl *hwfence_out_ctl;
	u64 hwfence_index;
	bool txq_updated_fence;
};

/**
 * enum sde_hw_fence_clients - sde clients for the hw-fence feature
 *
 * Do not modify the order of this struct and/or add more elements
 * without modify/add fields in the 'hw_fence_data' structs.
 */
enum sde_hw_fence_clients {
	SDE_HW_FENCE_CLIENT_CTL_0,
	SDE_HW_FENCE_CLIENT_CTL_1,
	SDE_HW_FENCE_CLIENT_CTL_2,
	SDE_HW_FENCE_CLIENT_CTL_3,
	SDE_HW_FENCE_CLIENT_CTL_4,
	SDE_HW_FENCE_CLIENT_CTL_5,
	SDE_HW_FENCE_CLIENT_MAX,
};

/**
 * hw_fence_data_dpu_client - this table maps the dpu ipcc input and output signals for each display
 *              clients to communicate with the fence controller.
 * This struct must match the order of the 'sde_hw_fence_clients' enum,
 * the output signal must match with the signals that FenceCTL expects for each display client.
 * This 'hw_fence_data_dpu_client' must be used for HW that does not support dpu-signal.
 */
struct sde_hw_fence_data hw_fence_data_no_dpu[SDE_HW_FENCE_CLIENT_MAX] = {
	{SDE_HW_FENCE_CLIENT_CTL_0, HW_FENCE_CLIENT_ID_CTL0, NULL, {0}, NULL, NULL, 8, 14, {2, 3},
		0, 8, 8, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_1, HW_FENCE_CLIENT_ID_CTL1, NULL, {0}, NULL, NULL, 8, 15, {4, 5},
		0, 8, 8, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_2, HW_FENCE_CLIENT_ID_CTL2, NULL, {0}, NULL, NULL, 8, 16, {6, 7},
		0, 8, 8, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_3, HW_FENCE_CLIENT_ID_CTL3, NULL, {0}, NULL, NULL, 8, 17, {8, 9},
		0, 8, 8, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_4, HW_FENCE_CLIENT_ID_CTL4, NULL, {0}, NULL, NULL, 8, 18, {10, 11},
		0, 8, 8, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_5, HW_FENCE_CLIENT_ID_CTL5, NULL, {0}, NULL, NULL, 8, 19, {12, 13},
		0, 8, 8, 0, 0}
};

/**
 * hw_fence_data_dpu_client - this table maps the dpu ipcc input and output signals for each display
 *              clients to communicate with the fence controller.
 * This struct must match the order of the 'sde_hw_fence_clients' enum,
 * the output signal must match with the signals that FenceCTL expects for each display client.
 * This 'hw_fence_data_dpu_client' must be used for HW that supports dpu-signal
 */
struct sde_hw_fence_data hw_fence_data_dpu_client[SDE_HW_FENCE_CLIENT_MAX] = {
	{SDE_HW_FENCE_CLIENT_CTL_0, HW_FENCE_CLIENT_ID_CTL0, NULL, {0}, NULL, NULL, 8, 0, {0, 6},
		0, 8, 25, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_1, HW_FENCE_CLIENT_ID_CTL1, NULL, {0}, NULL, NULL, 8, 1, {1, 7},
		0, 8, 25, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_2, HW_FENCE_CLIENT_ID_CTL2, NULL, {0}, NULL, NULL, 8, 2, {2, 8},
		0, 8, 25, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_3, HW_FENCE_CLIENT_ID_CTL3, NULL, {0}, NULL, NULL, 8, 3, {3, 9},
		0, 8, 25, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_4, HW_FENCE_CLIENT_ID_CTL4, NULL, {0}, NULL, NULL, 8, 4, {4, 10},
		0, 8, 25, 0, 0},
	{SDE_HW_FENCE_CLIENT_CTL_5, HW_FENCE_CLIENT_ID_CTL5, NULL, {0}, NULL, NULL, 8, 5, {5, 11},
		0, 8, 25, 0, 0}
};

void msm_hw_fence_error_cb(u32 handle, int error, void *cb_data)
{
	struct msm_hw_fence_cb_data *msm_hw_fence_cb_data;
	struct sde_hw_fence_error_cb_data *sde_hw_fence_error_data;

	SDE_EVT32(handle, error, SDE_EVTLOG_FUNC_ENTRY);

	msm_hw_fence_cb_data = (struct msm_hw_fence_cb_data *)cb_data;
	if (!msm_hw_fence_cb_data) {
		SDE_ERROR("msm hw fence cb data is NULL.\n");
		SDE_EVT32(SDE_EVTLOG_FUNC_CASE1, SDE_EVTLOG_ERROR);
		return;
	}

	sde_hw_fence_error_data = (struct sde_hw_fence_error_cb_data *)(msm_hw_fence_cb_data->data);
	if (!sde_hw_fence_error_data) {
		SDE_ERROR("sde hw fence cb data is NULL.\n");
		SDE_EVT32(SDE_EVTLOG_FUNC_CASE2, SDE_EVTLOG_ERROR);
		return;
	}

	sde_encoder_handle_hw_fence_error(sde_hw_fence_error_data->ctl_idx,
		sde_hw_fence_error_data->sde_kms, handle, error);
}

int sde_hw_fence_init(struct sde_hw_ctl *hw_ctl, struct sde_kms *sde_kms, bool use_dpu_ipcc,
		struct msm_mmu *mmu)
{
	struct msm_hw_fence_hfi_queue_header *hfi_queue_header_va, *hfi_queue_header_pa;
	struct msm_hw_fence_hfi_queue_table_header *hfi_table_header;
	struct sde_hw_fence_data *sde_hw_fence_data;
	struct sde_hw_fence_data *hwfence_data;
	struct sde_hw_fence_error_cb_data *sde_hw_fence_error_cb_data;
	phys_addr_t queue_pa;
	void *queue_va;
	u32 qhdr0_offset, ctl_hfi_iova;
	int ctl_id, ret;

	if (!hw_ctl || !hw_ctl->ops.hw_fence_output_fence_dir_write_init)
		return -EINVAL;

	ctl_id = hw_ctl->idx - CTL_0;
	if (ctl_id >= SDE_HW_FENCE_CLIENT_MAX || ctl_id < 0) {
		SDE_ERROR("unexpected ctl_id:%d\n", ctl_id);
		return -EINVAL;
	}

	hwfence_data = &hw_ctl->hwfence_data;
	sde_hw_fence_data = use_dpu_ipcc ? hw_fence_data_dpu_client : hw_fence_data_no_dpu;

	if (sde_hw_fence_data[ctl_id].client_id != ctl_id) {
		SDE_ERROR("Unexpected client_id:%d for ctl_id:%d\n",
			sde_hw_fence_data[ctl_id].client_id, ctl_id);
		return -EINVAL;
	}

	/* init the default fence-data for this client */
	memcpy(hwfence_data, &sde_hw_fence_data[ctl_id], sizeof(struct sde_hw_fence_data));

	SDE_DEBUG("hwfence register ctl:%d client:%d\n", ctl_id, hwfence_data->hw_fence_client_id);
	hwfence_data->hw_fence_handle = msm_hw_fence_register(hwfence_data->hw_fence_client_id,
		&hwfence_data->mem_descriptor);

	hwfence_data->dma_context = dma_fence_context_alloc(1);

	if (IS_ERR_OR_NULL(hwfence_data->hw_fence_handle)) {

		hwfence_data->hw_fence_handle = NULL;

		SDE_DEBUG("error cannot register ctl_id:%d hw-fence client:%d\n", ctl_id,
			hwfence_data->hw_fence_client_id);
		return -EINVAL;
	}

	sde_hw_fence_error_cb_data = &(hwfence_data->sde_hw_fence_error_cb_data);
	sde_hw_fence_error_cb_data->ctl_idx = hw_ctl->idx;
	sde_hw_fence_error_cb_data->sde_kms = sde_kms;

	ret = msm_hw_fence_register_error_cb(hwfence_data->hw_fence_handle,
		msm_hw_fence_error_cb, (void *)sde_hw_fence_error_cb_data);
	if (ret) {
		SDE_EVT32(hw_ctl->idx, SDE_EVTLOG_ERROR);
		SDE_DEBUG("hw fence cb register failed. ret = %d\n", ret);
	}

	/* one-to-one memory map of ctl-path client queues */
	ctl_hfi_iova = HW_FENCE_HFI_MMAP_DPU_BA +
		PAGE_ALIGN(hwfence_data->mem_descriptor.size * ctl_id);
	ret = mmu->funcs->one_to_one_map(mmu, ctl_hfi_iova,
		hwfence_data->mem_descriptor.device_addr,
		hwfence_data->mem_descriptor.size, IOMMU_READ | IOMMU_WRITE);
	if (ret) {
		SDE_ERROR("queue one2one memory smmu map failed, ret:%d ctl_id:%d, client:%d\n",
			ret, ctl_id, hwfence_data->hw_fence_client_id);
		return ret;
	}

	/* get queue header offset */
	queue_va = hwfence_data->mem_descriptor.virtual_addr;
	hfi_table_header = (struct msm_hw_fence_hfi_queue_table_header *)queue_va;
	qhdr0_offset = hfi_table_header->qhdr0_offset;

	/* initialize tx_wm pointer */
	hfi_queue_header_va = (struct msm_hw_fence_hfi_queue_header *)(queue_va + qhdr0_offset);
	hwfence_data->txq_tx_wm_va = &hfi_queue_header_va->tx_wm;

	/* initialize txq wr_ptr addr pointer */
	queue_pa = ctl_hfi_iova;
	hfi_queue_header_pa = (struct msm_hw_fence_hfi_queue_header *)(queue_pa + qhdr0_offset);
	hwfence_data->txq_wr_ptr_pa = &hfi_queue_header_pa->write_index;

	SDE_DEBUG("hwfence registered ctl:%d client:%d handle:0x%pK tx_wm:0x%x wr_idx:0x%x\n",
		ctl_id, hwfence_data->hw_fence_client_id, hwfence_data->hw_fence_handle,
		*hwfence_data->txq_tx_wm_va, *hwfence_data->txq_wr_ptr_pa);

	return 0;
}

void sde_hw_fence_deinit(struct sde_hw_ctl *hw_ctl)
{
	struct sde_hw_fence_data *hwfence_data;

	if (!hw_ctl)
		return;

	hwfence_data = &hw_ctl->hwfence_data;

	/* client was not registered */
	if (IS_ERR_OR_NULL(hwfence_data->hw_fence_handle))
		return;

	SDE_DEBUG("hwfence deregister ctl_id:%d hw_fence_client_id:%d\n",
		hw_ctl->idx - CTL_0, hwfence_data->hw_fence_client_id);

	msm_hw_fence_deregister(hwfence_data->hw_fence_handle);

	hwfence_data->hw_fence_handle = NULL;
}

static int sde_fence_create_hw_fence(struct sde_hw_ctl *hw_ctl, struct sde_fence *sde_fence)
{
	struct sde_hw_fence_data *data;
	struct msm_hw_fence_create_params params;
	int ctl_id;
	u64 hwfence_index;
	int ret;

	if (!hw_ctl)
		return -EINVAL;

	ctl_id = hw_ctl->idx - CTL_0;
	data = &hw_ctl->hwfence_data;

	if (IS_ERR_OR_NULL(data->hw_fence_handle)) {
		SDE_ERROR("unexpected handle for ctl_id:%d\n", ctl_id);
		return -EINVAL;
	}
	params.fence = &sde_fence->base;
	params.handle = &hwfence_index;

	/* Create the HW fence */
	ret = msm_hw_fence_create(data->hw_fence_handle, &params);
	if (ret) {
		SDE_ERROR("failed to create hw_fence for client:%d ctx:%llu seqno:%llu\n", ctl_id,
			sde_fence->base.context, sde_fence->base.seqno);
	} else {
		/* store ctl and index for this fence */
		sde_fence->hwfence_out_ctl = hw_ctl;
		sde_fence->hwfence_index = hwfence_index;

		SDE_DEBUG("create hfence index:%llu ctl:%d ctx:%llu seqno:%llu name:%s\n",
			sde_fence->hwfence_index, ctl_id, sde_fence->base.context,
			sde_fence->base.seqno, sde_fence->name);
	}

	return ret;
}

static inline char *_get_client_id_name(int hw_fence_client_id)
{
	switch (hw_fence_client_id) {
	case HW_FENCE_CLIENT_ID_CTX0:
		return "HW_FENCE_CLIENT_ID_CTX0";
	case HW_FENCE_CLIENT_ID_CTL0:
		return "HW_FENCE_CLIENT_ID_CTL0";
	case HW_FENCE_CLIENT_ID_CTL1:
		return "HW_FENCE_CLIENT_ID_CTL1";
	case HW_FENCE_CLIENT_ID_CTL2:
		return "HW_FENCE_CLIENT_ID_CTL2";
	case HW_FENCE_CLIENT_ID_CTL3:
		return "HW_FENCE_CLIENT_ID_CTL3";
	case HW_FENCE_CLIENT_ID_CTL4:
		return "HW_FENCE_CLIENT_ID_CTL4";
	case HW_FENCE_CLIENT_ID_CTL5:
		return "HW_FENCE_CLIENT_ID_CTL15";
	default:
		return "Unknown";
	}

	return "unknown";
}

static void _cleanup_fences_refcount(struct dma_fence **fences, u32 num_fences)
{
	int i;

	for (i = 0; i < num_fences; i++)
		dma_fence_put(fences[i]);
}

int sde_fence_register_hw_fences_wait(struct sde_hw_ctl *hw_ctl, struct dma_fence **fences,
	u32 num_fences)
{
	struct sde_hw_fence_data *data;
	int i, j, ret;
	int ctl_id;
	struct dma_fence_array *temp_array = NULL;
	struct dma_fence *base_fence;
	struct dma_fence **hw_fences;
	u32 num_hw_fences;
	struct dma_fence **fence_list;
	struct dma_fence_array *array = NULL;
	int array_childs = 0;
	int array_count = 0;
	int fence_list_index = 0;
	u64 seqno;

	if (!hw_ctl) {
		SDE_ERROR("wrong ctl\n");
		return -EINVAL;
	}

	ctl_id = hw_ctl->idx - CTL_0;
	data = &hw_ctl->hwfence_data;
	if (IS_ERR_OR_NULL(data->hw_fence_handle)) {
		SDE_ERROR("unexpected handle for ctl_id:%d\n", ctl_id);
		return -EINVAL;
	}

	SDE_DEBUG("register for wait fences:%d ctl_id:%d hw_fence_client:%s\n",
		num_fences, ctl_id, _get_client_id_name(data->hw_fence_client_id));

	for (i = 0; i < num_fences; i++) {
		/* get a refcount for each of the fences */
		dma_fence_get(fences[i]);

		if (dma_fence_is_array(fences[i])) {
			array_count++;
			array = container_of(fences[i], struct dma_fence_array, base);
			array_childs += array->num_fences;
		}

		SDE_DEBUG("registering fence: ctx:%llu seqno:%llu\n",
			(fences[i])->context, (fences[i])->seqno);
	}

	if (num_fences > 1) {
		/* fence_list memory is freed during fence-array release */
		fence_list = kzalloc(((num_fences - array_count) + array_childs)
				* (sizeof(struct dma_fence *)), GFP_KERNEL);
		if (!fence_list) {
			_cleanup_fences_refcount(fences, num_fences);
			return -EINVAL;
		}

		/* populate fence_list with the fences */
		for (i = 0; i < num_fences; i++) {
			if (dma_fence_is_array(fences[i])) {
				array = container_of(fences[i], struct dma_fence_array, base);
				for (j = 0; j < array->num_fences; j++) {
					/* get a refcount for each of the child fences */
					dma_fence_get(array->fences[j]);
					fence_list[fence_list_index++] = array->fences[j];
				}

				if (array->num_fences) /* print the first fence from array */
					SDE_EVT32(ctl_id, num_fences, array->num_fences, i,
						SDE_EVTLOG_H32(array->fences[0]->context),
						SDE_EVTLOG_L32(array->fences[0]->context),
						SDE_EVTLOG_H32(array->fences[0]->seqno),
						SDE_EVTLOG_L32(array->fences[0]->seqno));
				else
					SDE_EVT32(ctl_id, num_fences, array->num_fences, i,
						SDE_EVTLOG_ERROR);

				/* remove refcount on parent */
				dma_fence_put(fences[i]);
			} else {
				fence_list[fence_list_index++] = fences[i];

				SDE_EVT32(ctl_id, num_fences, i, SDE_EVTLOG_H32(fences[i]->context),
					SDE_EVTLOG_L32(fences[i]->context),
					SDE_EVTLOG_H32(fences[i]->seqno),
					SDE_EVTLOG_L32(fences[i]->seqno));
			}
		}

		seqno = data->hw_fence_array_seqno++;
		temp_array = dma_fence_array_create(fence_list_index, fence_list,
				data->dma_context, seqno, 0);
		if (!temp_array) {
			SDE_ERROR("unable to create fence array, cant register for wait\n");
			_cleanup_fences_refcount(fences, num_fences);
			kfree(fence_list);
			return -EINVAL;
		}
		SDE_EVT32(ctl_id, fence_list_index, SDE_EVTLOG_H32(data->dma_context),
			SDE_EVTLOG_L32(data->dma_context), SDE_EVTLOG_H32(seqno),
			SDE_EVTLOG_L32(seqno));

		base_fence = &temp_array->base;
		hw_fences = &base_fence;
		num_hw_fences = 1;

	} else {
		struct dma_fence_array *tmp_array;

		hw_fences = fences;
		num_hw_fences = num_fences;
		tmp_array = dma_fence_is_array(fences[0]) ?
				container_of(fences[0], struct dma_fence_array, base) :
				NULL;
		SDE_EVT32(ctl_id, num_hw_fences, SDE_EVTLOG_H32(fences[0]->context),
			SDE_EVTLOG_L32(fences[0]->context), SDE_EVTLOG_H32(fences[0]->seqno),
			SDE_EVTLOG_L32(fences[0]->seqno), fences[0]->flags,
			tmp_array ? tmp_array->num_fences : SDE_EVTLOG_FUNC_CASE2);
	}

	/* register for wait */
	ret = msm_hw_fence_wait_update(data->hw_fence_handle, hw_fences, num_hw_fences, true);
	if (ret)
		SDE_ERROR("failed to register wait fences for ctl_id:%d ret:%d\n", ctl_id, ret);

	/* fence-array put will release each individual extra refcount during array release */
	if (temp_array)
		dma_fence_put(&temp_array->base);
	else
		dma_fence_put(fences[0]);

	SDE_EVT32_VERBOSE(ctl_id, num_fences, ret);

	return ret;
}

static int _arm_output_hw_fence(struct sde_hw_ctl *hw_ctl, bool vid_mode, u32 line_count,
		u32 debugfs_hw_fence)
{
	struct sde_hw_fence_data *data;
	u32 ipcc_out_signal;
	int ctl_id;

	if (!hw_ctl || !hw_ctl->ops.hw_fence_trigger_output_fence ||
			!hw_ctl->ops.hw_fence_update_output_fence) {
		SDE_ERROR("missing ctl/trigger or update fence %d\n", !hw_ctl);
		return -EINVAL;
	}

	ctl_id = hw_ctl->idx - CTL_0;
	data = &hw_ctl->hwfence_data;
	if (data->ipcc_out_signal_pp_idx >= MAX_SDE_HFENCE_OUT_SIGNAL_PING_PONG) {
		/* This should not have happened!, review the ping pong calculation */
		SDE_ERROR("Wrong pp_idx:%d, max:%d\n", data->ipcc_out_signal_pp_idx,
			MAX_SDE_HFENCE_OUT_SIGNAL_PING_PONG);
		return -EINVAL;
	}

	ipcc_out_signal = data->ipcc_out_signal_pp[data->ipcc_out_signal_pp_idx];
	data->ipcc_out_signal_pp_idx = (++data->ipcc_out_signal_pp_idx %
		MAX_SDE_HFENCE_OUT_SIGNAL_PING_PONG);

	SDE_DEBUG("out-fence ctl_id:%d out_signal:%d hw_fence_client:%s\n",
		ctl_id, ipcc_out_signal, _get_client_id_name(data->hw_fence_client_id));

	if ((debugfs_hw_fence & SDE_OUTPUT_HW_FENCE_TIMESTAMP) &&
			hw_ctl->ops.hw_fence_output_timestamp_ctrl)
		hw_ctl->ops.hw_fence_output_timestamp_ctrl(hw_ctl, true, false);

	/* update client/signal output fence */
	hw_ctl->ops.hw_fence_update_output_fence(hw_ctl, data->ipcc_out_client, ipcc_out_signal);
	SDE_EVT32_VERBOSE(ctl_id, ipcc_out_signal);

	/* arm dpu to trigger output fence signal once ready */
	if (line_count)
		hw_ctl->ops.hw_fence_trigger_output_fence(hw_ctl,
			HW_FENCE_TRIGGER_SEL_PROG_LINE_COUNT);
	else if (vid_mode && (hw_ctl->caps->features & BIT(SDE_CTL_HW_FENCE_TRIGGER_SEL)))
		hw_ctl->ops.hw_fence_trigger_output_fence(hw_ctl, HW_FENCE_TRIGGER_SEL_VID_MODE);
	else
		hw_ctl->ops.hw_fence_trigger_output_fence(hw_ctl, HW_FENCE_TRIGGER_SEL_CMD_MODE);

	return 0;
}

static int _sde_fence_arm_output_hw_fence(struct sde_fence_context *ctx, bool vid_mode,
		u32 line_count, u32 debugfs_hw_fence)
{
	struct sde_hw_ctl *hw_ctl = NULL;
	struct sde_fence *fc, *next;

	spin_lock(&ctx->list_lock);
	if (list_empty(&ctx->fence_list_head)) {
		spin_unlock(&ctx->list_lock);
		return 0;
	}

	list_for_each_entry_safe(fc, next, &ctx->fence_list_head, fence_list) {
		struct dma_fence *fence = &fc->base;

		/* this is not hw-fence, or already processed */
		if (!test_bit(MSM_HW_FENCE_FLAG_ENABLED_BIT, &fence->flags))
			continue;

		hw_ctl = fc->hwfence_out_ctl;
		if (!hw_ctl) {
			/*
			 * We flaged an output dma-fence as hw-fence but the hw ctl to handle
			 * it is not available, this should not have happened, but if it does,
			 * this can translate to a fence-timeout!
			 */
			SDE_ERROR("invalid hw ctl, this can cause a fence-timeout!\n");
			SDE_EVT32(SDE_EVTLOG_ERROR, SDE_EVTLOG_FUNC_CASE1, fence->flags,
				fence->context, fence->seqno);

			spin_unlock(&ctx->list_lock);
			return -EINVAL;
		}
	}
	spin_unlock(&ctx->list_lock);

	/* arm dpu to trigger output hw-fence ipcc signal upon completion */
	if (hw_ctl)
		_arm_output_hw_fence(hw_ctl, vid_mode, line_count, debugfs_hw_fence);

	return 0;
}

void sde_fence_output_hw_fence_dir_write_init(struct sde_hw_ctl *hw_ctl)
{
	if (hw_ctl && hw_ctl->ops.hw_fence_output_fence_dir_write_init)
		hw_ctl->ops.hw_fence_output_fence_dir_write_init(hw_ctl,
			hw_ctl->hwfence_data.txq_wr_ptr_pa, HW_FENCE_DIR_WRITE_SIZE,
			HW_FENCE_DIR_WRITE_MASK);
}

/* update output hw_fences txq */
int sde_fence_update_hw_fences_txq(struct sde_fence_context *ctx, bool vid_mode, u32 line_count,
		u32 debugfs_hw_fence)
{
	int ret = 0;
	struct sde_hw_fence_data *data;
	struct sde_fence *fc, *next;
	struct sde_hw_ctl *hw_ctl = NULL;
	int ctl_id;
	bool txq_updated = false;

	spin_lock(&ctx->list_lock);
	if (list_empty(&ctx->fence_list_head)) {
		spin_unlock(&ctx->list_lock);
		return 0;
	}

	list_for_each_entry_safe(fc, next, &ctx->fence_list_head, fence_list) {
		struct dma_fence *fence = &fc->base;

		/* this is not hw-fence, or already processed */
		if (!test_bit(MSM_HW_FENCE_FLAG_ENABLED_BIT, &fence->flags) ||
				fc->txq_updated_fence)
			continue;

		hw_ctl = fc->hwfence_out_ctl;
		if (!hw_ctl) {
			/* We flaged an output dma-fence as hw-fence but the hw ctl to handle
			 * it is not available, this should not have happened, but if it does,
			 * this can translate to a fence-timeout!
			 */
			SDE_ERROR("invalid hw ctl, this can cause a fence-timeout!\n");
			SDE_EVT32(SDE_EVTLOG_FUNC_CASE1, fence->flags, fence->context,
				fence->seqno, SDE_EVTLOG_ERROR);
			ret = -EINVAL;
			goto exit;
		}

		ctl_id = hw_ctl->idx - CTL_0;
		data = &hw_ctl->hwfence_data;
		if (IS_ERR_OR_NULL(data->hw_fence_handle)) {
			SDE_ERROR("unexpected handle for ctl_id:%d, this can fence-timeout\n",
				ctl_id);
			SDE_EVT32(SDE_EVTLOG_FUNC_CASE2, fence->flags, fence->context,
				fence->seqno, ctl_id, SDE_EVTLOG_ERROR);
			ret = -EINVAL;
			goto exit;
		}

		/* update hw-fence tx queue */
		SDE_EVT32(ctl_id, SDE_EVTLOG_H32(fc->hwfence_index),
			SDE_EVTLOG_L32(fc->hwfence_index), *data->txq_tx_wm_va);
		ret = msm_hw_fence_update_txq(data->hw_fence_handle, fc->hwfence_index, 0, 0);
		if (ret) {
			SDE_ERROR("fail txq update index:%llu fctx:%llu seqno:%llu client:%d\n",
				fc->hwfence_index, fence->context, fence->seqno,
				data->hw_fence_client_id);
			SDE_EVT32(SDE_EVTLOG_FUNC_CASE3, fence->flags, fence->context,
				fence->seqno, ctl_id, SDE_EVTLOG_ERROR);
			goto exit;
		}

		/* update hw-fence tx queue wr_idx data */
		if (hw_ctl->ops.hw_fence_output_fence_dir_write_data)
			hw_ctl->ops.hw_fence_output_fence_dir_write_data(hw_ctl,
				*data->txq_tx_wm_va);

		/* avoid updating txq more than once and avoid repeating the same fence twice */
		txq_updated = fc->txq_updated_fence = true;

		SDE_DEBUG("update txq fence:0x%pK ctx:%llu seqno:%llu f:0x%llx ctl:%d vid:%d\n",
			fence, fence->context, fence->seqno, fence->flags, ctl_id, vid_mode);

		/* We will update TxQ one time per frame */
		if (txq_updated)
			break;
	}

exit:
	spin_unlock(&ctx->list_lock);

	/* arm dpu to trigger output hw-fence ipcc signal upon completion in vid-mode */
	if ((txq_updated && hw_ctl) || line_count)
		_sde_fence_arm_output_hw_fence(ctx, vid_mode, line_count, debugfs_hw_fence);

	return ret;
}

static void _sde_hw_fence_release(struct sde_fence *f)
{
	struct sde_hw_fence_data *data;
	struct sde_hw_ctl *hw_ctl = f->hwfence_out_ctl;
	int ctl_id;
	int ret;

	if (!hw_ctl) {
		SDE_ERROR("invalid hw_ctl\n");
		return;
	}

	ctl_id = hw_ctl->idx - CTL_0;
	data = &hw_ctl->hwfence_data;
	if (IS_ERR_OR_NULL(data->hw_fence_handle)) {
		SDE_ERROR("unexpected handle for ctl_id:%d\n", ctl_id);
		return;
	}

	SDE_DEBUG("destroy hw fence ctl_id:%d ctx:%llu seqno:%llu name:%s\n",
		ctl_id, f->base.context, f->base.seqno, f->name);

	/* Delete the HW fence */
	ret = msm_hw_fence_destroy(data->hw_fence_handle, &f->base);
	if (ret)
		SDE_ERROR("failed to destroy hw_fence for ctl_id:%d ctx:%llu seqno:%llu\n", ctl_id,
			f->base.context, f->base.seqno);
}

static int _reset_hw_fence_timeline(struct sde_hw_ctl *hw_ctl, u32 flags)
{
	struct sde_hw_fence_data *data;
	int ret = 0;

	data = &hw_ctl->hwfence_data;

	if (!IS_ERR_OR_NULL(data->hw_fence_handle)) {
		SDE_EVT32(data->hw_fence_client_id);
		ret = msm_hw_fence_reset_client(data->hw_fence_handle, flags);
		if (ret) {
			pr_err("failed to reset client %d\n", data->hw_fence_client_id);
			return -EINVAL;
		}
	}

	return ret;
}

int sde_fence_update_input_hw_fence_signal(struct sde_hw_ctl *hw_ctl, u32 debugfs_hw_fence,
		struct sde_hw_mdp *hw_mdp, bool disable)
{
	struct sde_hw_fence_data *data;
	u32 ipcc_signal_id;
	u32 ipcc_client_id;
	int ctl_id;
	u64 qtime;

	/* we must support sw_override as well, so check both functions */
	if (!hw_mdp || !hw_ctl || !hw_ctl->ops.hw_fence_update_input_fence ||
			!hw_ctl->ops.hw_fence_trigger_sw_override) {
		SDE_ERROR("missing ctl/override/update fence %d\n", !hw_ctl);
		return -EINVAL;
	}

	ctl_id = hw_ctl->idx - CTL_0;
	data = &hw_ctl->hwfence_data;

	if (disable) {
		hw_ctl->ops.hw_fence_ctrl(hw_ctl, false, false, 0);
		return -EPERM;
	}

	if ((debugfs_hw_fence & SDE_INPUT_HW_FENCE_TIMESTAMP)
			&& hw_mdp->ops.hw_fence_input_timestamp_ctrl)
		hw_mdp->ops.hw_fence_input_timestamp_ctrl(hw_mdp, true, false);

	ipcc_signal_id = data->ipcc_in_signal;
	ipcc_client_id = data->ipcc_in_client;

	SDE_DEBUG("configure input signal:%d out client:%d ctl_id:%d\n", ipcc_signal_id,
		ipcc_client_id, ctl_id);

	/* configure dpu hw for the client/signal pair signaling input-fence */
	hw_ctl->ops.hw_fence_update_input_fence(hw_ctl, ipcc_client_id, ipcc_signal_id);

	/* Enable hw-fence for this ctrl-path */
	hw_ctl->ops.hw_fence_ctrl(hw_ctl, true, true, 1);

	qtime = arch_timer_read_counter();
	SDE_EVT32(ctl_id, ipcc_signal_id, ipcc_client_id, SDE_EVTLOG_H32(qtime),
		SDE_EVTLOG_L32(qtime));

	return 0;
}

void sde_fence_error_ctx_update(struct sde_fence_context *ctx, int input_fence_status,
	enum sde_fence_error_state sde_fence_error_state)
{
	if (!ctx) {
		SDE_DEBUG("invalid fence\n");
		SDE_EVT32(input_fence_status, sde_fence_error_state, SDE_EVTLOG_ERROR);
		return;
	}

	ctx->sde_fence_error_ctx.fence_error_status = input_fence_status;
	ctx->sde_fence_error_ctx.fence_error_state = sde_fence_error_state;
}

void *sde_sync_get(uint64_t fd)
{
	/* force signed compare, fdget accepts an int argument */
	return (signed int)fd >= 0 ? sync_file_get_fence(fd) : NULL;
}

void sde_sync_put(void *fence)
{
	if (fence)
		dma_fence_put(fence);
}

void sde_fence_dump(struct dma_fence *fence)
{
	char timeline_str[TIMELINE_VAL_LENGTH];

	if (fence->ops->timeline_value_str)
		fence->ops->timeline_value_str(fence, timeline_str, TIMELINE_VAL_LENGTH);

	SDE_ERROR(
		"fence drv name:%s timeline name:%s seqno:0x%llx timeline:%s signaled:0x%x status:%d flags:0x%x\n",
		fence->ops->get_driver_name(fence),
		fence->ops->get_timeline_name(fence),
		fence->seqno, timeline_str,
		fence->ops->signaled ?
		fence->ops->signaled(fence) : 0xffffffff,
		dma_fence_get_status(fence), fence->flags);
}

static void sde_fence_dump_user_fds_info(struct dma_fence *base_fence)
{
	struct dma_fence_array *array;
	struct dma_fence *user_fence;
	int i;

	array = container_of(base_fence, struct dma_fence_array, base);
	if (test_bit(SPEC_FENCE_FLAG_FENCE_ARRAY, &base_fence->flags) &&
		test_bit(SPEC_FENCE_FLAG_ARRAY_BIND, &base_fence->flags)) {
		for (i = 0; i < array->num_fences; i++) {
			user_fence = array->fences[i];
			if (user_fence) {
				dma_fence_get(user_fence);
				sde_fence_dump(user_fence);
				dma_fence_put(user_fence);
			}
		}
	}
}

signed long sde_sync_wait(void *fnc, long timeout_ms, int *error_status)
{
	struct dma_fence *fence = fnc;
	int rc, status = 0;

	if (!fence)
		return -EINVAL;
	else if (dma_fence_is_signaled(fence))
		return timeout_ms ? msecs_to_jiffies(timeout_ms) : 1;

	rc = dma_fence_wait_timeout(fence, true,
				msecs_to_jiffies(timeout_ms));
	if (!rc || (rc == -EINVAL) || fence->error) {
		status = dma_fence_get_status(fence);
		if (test_bit(SPEC_FENCE_FLAG_FENCE_ARRAY, &fence->flags)) {
			if (status == -EINVAL) {
				SDE_INFO("spec fence bind failure status:%d\n", status);
				rc = -EBADF;
			} else if (fence->ops->signaled && fence->ops->signaled(fence)) {
				SDE_INFO("spec fence status:%d\n", status);
			} else {
				sde_fence_dump(fence);
				sde_fence_dump_user_fds_info(fence);
			}
		} else {
			sde_fence_dump(fence);
		}
	}

	if (error_status)
		*error_status = fence->error;
	return rc;
}

uint32_t sde_sync_get_name_prefix(void *fence)
{
	const char *name;
	uint32_t i, prefix;
	struct dma_fence *f = fence;

	if (!fence)
		return 0;

	name = f->ops->get_driver_name(f);
	if (!name)
		return 0;

	prefix = 0x0;
	for (i = 0; i < sizeof(uint32_t) && name[i]; ++i)
		prefix = (prefix << CHAR_BIT) | name[i];

	return prefix;
}

static void sde_fence_destroy(struct kref *kref)
{
	struct sde_fence_context *ctx;

	if (!kref) {
		SDE_ERROR("received invalid kref\n");
		return;
	}

	ctx = container_of(kref, struct sde_fence_context, kref);
	kfree(ctx);
}

static inline struct sde_fence *to_sde_fence(struct dma_fence *fence)
{
	return container_of(fence, struct sde_fence, base);
}

static const char *sde_fence_get_driver_name(struct dma_fence *fence)
{
	struct sde_fence *f = to_sde_fence(fence);

	return f->name;
}

static const char *sde_fence_get_timeline_name(struct dma_fence *fence)
{
	struct sde_fence *f = to_sde_fence(fence);

	return f->ctx->name;
}

static bool sde_fence_enable_signaling(struct dma_fence *fence)
{
	return true;
}

static bool sde_fence_signaled(struct dma_fence *fence)
{
	struct sde_fence *f = to_sde_fence(fence);
	bool status;

	status = ((int)(fence->seqno - f->ctx->done_count) <= 0);
	SDE_DEBUG("status:%d fence seq:%llu and timeline:%u\n",
			status, fence->seqno, f->ctx->done_count);
	return status;
}

static void sde_fence_release(struct dma_fence *fence)
{
	struct sde_fence *f;

	if (fence) {
		f = to_sde_fence(fence);

		/* Delete the HW fence */
		if (test_bit(MSM_HW_FENCE_FLAG_ENABLED_BIT, &fence->flags))
			_sde_hw_fence_release(f);

		kref_put(&f->ctx->kref, sde_fence_destroy);
		kfree(f);
	}
}

static void sde_fence_value_str(struct dma_fence *fence, char *str, int size)
{
	if (!fence || !str)
		return;

	snprintf(str, size, "%llu", fence->seqno);
}

static void sde_fence_timeline_value_str(struct dma_fence *fence, char *str,
		int size)
{
	struct sde_fence *f = to_sde_fence(fence);

	if (!fence || !f->ctx || !str)
		return;

	snprintf(str, size, "%d", f->ctx->done_count);
}

static struct dma_fence_ops sde_fence_ops = {
	.get_driver_name = sde_fence_get_driver_name,
	.get_timeline_name = sde_fence_get_timeline_name,
	.enable_signaling = sde_fence_enable_signaling,
	.signaled = sde_fence_signaled,
	.wait = dma_fence_default_wait,
	.release = sde_fence_release,
	.fence_value_str = sde_fence_value_str,
	.timeline_value_str = sde_fence_timeline_value_str,
};

/**
 * _sde_fence_create_fd - create fence object and return an fd for it
 * This function is NOT thread-safe.
 * @timeline: Timeline to associate with fence
 * @val: Timeline value at which to signal the fence
 * Return: File descriptor on success, or error code on error
 */
static int _sde_fence_create_fd(void *fence_ctx, uint32_t val, struct sde_hw_ctl *hw_ctl)
{
	struct sde_fence *sde_fence;
	struct sync_file *sync_file;
	signed int fd = -EINVAL;
	struct sde_fence_context *ctx = fence_ctx;

	if (!ctx) {
		SDE_ERROR("invalid context\n");
		goto exit;
	}

	sde_fence = kzalloc(sizeof(*sde_fence), GFP_KERNEL);
	if (!sde_fence)
		return -ENOMEM;

	sde_fence->ctx = fence_ctx;
	snprintf(sde_fence->name, SDE_FENCE_NAME_SIZE, "sde_fence:%s:%u",
						sde_fence->ctx->name, val);
	dma_fence_init(&sde_fence->base, &sde_fence_ops, &ctx->lock,
		ctx->context, val);
	kref_get(&ctx->kref);

	ctx->sde_fence_error_ctx.curr_frame_fence_seqno = val;

	/* create fd */
	fd = get_unused_fd_flags(0);
	if (fd < 0) {
		SDE_ERROR("failed to get_unused_fd_flags(), %s\n",
							sde_fence->name);
		dma_fence_put(&sde_fence->base);
		goto exit;
	}

	/* create fence */
	sync_file = sync_file_create(&sde_fence->base);
	if (sync_file == NULL) {
		put_unused_fd(fd);
		fd = -EINVAL;
		SDE_ERROR("couldn't create fence, %s\n", sde_fence->name);
		dma_fence_put(&sde_fence->base);
		goto exit;
	}

	/* If ctl_id is valid, try to create a hw-fence */
	if (hw_ctl)
		sde_fence_create_hw_fence(hw_ctl, sde_fence);

	fd_install(fd, sync_file->file);
	sde_fence->fd = fd;

	spin_lock(&ctx->list_lock);
	list_add_tail(&sde_fence->fence_list, &ctx->fence_list_head);
	spin_unlock(&ctx->list_lock);

exit:
	return fd;
}

struct sde_fence_context *sde_fence_init(const char *name, uint32_t drm_id)
{
	struct sde_fence_context *ctx;

	if (!name) {
		SDE_ERROR("invalid argument(s)\n");
		return ERR_PTR(-EINVAL);
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);

	if (!ctx) {
		SDE_ERROR("failed to alloc fence ctx\n");
		return ERR_PTR(-ENOMEM);
	}

	strlcpy(ctx->name, name, ARRAY_SIZE(ctx->name));
	ctx->drm_id = drm_id;
	kref_init(&ctx->kref);
	ctx->context = dma_fence_context_alloc(1);

	spin_lock_init(&ctx->lock);
	spin_lock_init(&ctx->list_lock);
	INIT_LIST_HEAD(&ctx->fence_list_head);

	return ctx;
}

void sde_fence_deinit(struct sde_fence_context *ctx)
{
	if (!ctx) {
		SDE_ERROR("invalid fence\n");
		return;
	}

	kref_put(&ctx->kref, sde_fence_destroy);
}

void sde_fence_prepare(struct sde_fence_context *ctx)
{
	unsigned long flags;

	if (!ctx) {
		SDE_ERROR("invalid argument(s), fence %pK\n", ctx);
	} else {
		spin_lock_irqsave(&ctx->lock, flags);
		++ctx->commit_count;
		spin_unlock_irqrestore(&ctx->lock, flags);
	}
}

static void _sde_fence_trigger(struct sde_fence_context *ctx, bool error, ktime_t ts)
{
	unsigned long flags;
	struct sde_fence *fc, *next;
	bool is_signaled = false;
	enum sde_fence_error_state fence_error_state = 0;
	struct sde_fence_error_ctx *fence_error_ctx;

	kref_get(&ctx->kref);

	spin_lock(&ctx->list_lock);
	if (list_empty(&ctx->fence_list_head)) {
		SDE_DEBUG("nothing to trigger!\n");
		goto end;
	}

	fence_error_ctx = &ctx->sde_fence_error_ctx;

	list_for_each_entry_safe(fc, next, &ctx->fence_list_head, fence_list) {
		spin_lock_irqsave(&ctx->lock, flags);
		if (error)
			dma_fence_set_error(&fc->base, -EBUSY);

		fence_error_state = fence_error_ctx->fence_error_state;
		if (fence_error_state) {
			if (fence_error_state == HANDLE_OUT_OF_ORDER &&
				fence_error_ctx->last_good_frame_fence_seqno == fc->base.seqno) {
				SDE_EVT32(fence_error_ctx->last_good_frame_fence_seqno,
					fence_error_state, SDE_EVTLOG_FUNC_CASE1);
				spin_unlock_irqrestore(&ctx->lock, flags);
				continue;
			} else if (((fence_error_state == HANDLE_OUT_OF_ORDER) ||
				(fence_error_state == SET_ERROR_ONLY_VID) ||
				(fence_error_state == SET_ERROR_ONLY_CMD_RELEASE))
				&& (fence_error_ctx->fence_error_status < 0)) {
				dma_fence_set_error(&fc->base, fence_error_ctx->fence_error_status);
				dma_fence_signal_timestamp_locked(&fc->base, ts);
				spin_unlock_irqrestore(&ctx->lock, flags);

				SDE_EVT32(fence_error_state, fence_error_ctx->fence_error_status,
					ktime_to_us(ts), fc->base.seqno, SDE_EVTLOG_FUNC_CASE2);
				list_del_init(&fc->fence_list);
				dma_fence_put(&fc->base);
				continue;
			}
		}

		is_signaled = sde_fence_signaled(&fc->base);
		if (is_signaled)
			dma_fence_signal_timestamp_locked(&fc->base, ts);
		spin_unlock_irqrestore(&ctx->lock, flags);

		if (is_signaled) {
			list_del_init(&fc->fence_list);
			dma_fence_put(&fc->base);
		}
	}
end:
	spin_unlock(&ctx->list_lock);
	kref_put(&ctx->kref, sde_fence_destroy);
}

int sde_fence_create(struct sde_fence_context *ctx, uint64_t *val,
				uint32_t offset, struct sde_hw_ctl *hw_ctl)
{
	uint32_t trigger_value;
	int fd, rc = -EINVAL;
	unsigned long flags;

	if (!ctx || !val) {
		SDE_ERROR("invalid argument(s), fence %d, pval %d\n",
				ctx != NULL, val != NULL);
		return rc;
	}

	/*
	 * Allow created fences to have a constant offset with respect
	 * to the timeline. This allows us to delay the fence signalling
	 * w.r.t. the commit completion (e.g., an offset of +1 would
	 * cause fences returned during a particular commit to signal
	 * after an additional delay of one commit, rather than at the
	 * end of the current one.
	 */
	spin_lock_irqsave(&ctx->lock, flags);
	trigger_value = ctx->commit_count + offset;
	spin_unlock_irqrestore(&ctx->lock, flags);

	fd = _sde_fence_create_fd(ctx, trigger_value, hw_ctl);
	*val = fd;
	SDE_DEBUG("fd:%d trigger:%d commit:%d offset:%d\n",
			fd, trigger_value, ctx->commit_count, offset);

	SDE_EVT32(ctx->drm_id, trigger_value, fd, hw_ctl ? hw_ctl->idx : 0);
	rc = (fd >= 0) ? 0 : fd;

	return rc;
}

void sde_fence_signal(struct sde_fence_context *ctx, ktime_t ts,
		enum sde_fence_event fence_event, struct sde_hw_ctl *hw_ctl)
{
	unsigned long flags;

	if (!ctx) {
		SDE_ERROR("invalid ctx, %pK\n", ctx);
		return;
	}

	spin_lock_irqsave(&ctx->lock, flags);
	if (fence_event == SDE_FENCE_RESET_TIMELINE) {
		/* reset hw-fences without error */
		if (hw_ctl)
			_reset_hw_fence_timeline(hw_ctl, MSM_HW_FENCE_RESET_WITHOUT_ERROR |
				MSM_HW_FENCE_RESET_WITHOUT_DESTROY);

		if ((int)(ctx->done_count - ctx->commit_count) < 0) {
			SDE_DEBUG(
			  "timeline reset attempt! ctx:0x%x done count:%d commit:%d\n",
				ctx->drm_id, ctx->done_count, ctx->commit_count);
			ctx->done_count = ctx->commit_count;
			SDE_EVT32(ctx->drm_id, ctx->done_count,
				ctx->commit_count, ktime_to_us(ts),
				fence_event, SDE_EVTLOG_FUNC_CASE1);
		} else {
			spin_unlock_irqrestore(&ctx->lock, flags);
			return;
		}
	} else if ((int)(ctx->done_count - ctx->commit_count) < 0) {
		++ctx->done_count;
		SDE_DEBUG("fence_signal:done count:%d commit count:%d\n",
					ctx->done_count, ctx->commit_count);
	} else {
		SDE_ERROR("extra signal attempt! done count:%d commit:%d\n",
					ctx->done_count, ctx->commit_count);
		SDE_EVT32(ctx->drm_id, ctx->done_count, ctx->commit_count,
			ktime_to_us(ts), fence_event, SDE_EVTLOG_FATAL);
		spin_unlock_irqrestore(&ctx->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	SDE_EVT32(ctx->drm_id, ctx->done_count, ctx->commit_count,
			ktime_to_us(ts));

	_sde_fence_trigger(ctx, (fence_event == SDE_FENCE_SIGNAL_ERROR), ts);
}

void sde_fence_timeline_status(struct sde_fence_context *ctx,
					struct drm_mode_object *drm_obj)
{
	char *obj_name;

	if (!ctx || !drm_obj) {
		SDE_ERROR("invalid input params\n");
		return;
	}

	switch (drm_obj->type) {
	case DRM_MODE_OBJECT_CRTC:
		obj_name = "crtc";
		break;
	case DRM_MODE_OBJECT_CONNECTOR:
		obj_name = "connector";
		break;
	default:
		obj_name = "unknown";
		break;
	}

	SDE_ERROR("drm obj:%s id:%d type:0x%x done_count:%d commit_count:%d\n",
		obj_name, drm_obj->id, drm_obj->type, ctx->done_count,
		ctx->commit_count);
}

void sde_fence_list_dump(struct dma_fence *fence, struct seq_file **s)
{
	char timeline_str[TIMELINE_VAL_LENGTH];

	if (fence->ops->timeline_value_str)
		fence->ops->timeline_value_str(fence,
		timeline_str, TIMELINE_VAL_LENGTH);

	seq_printf(*s, "fence name:%s timeline name:%s seqno:0x%llx timeline:%s signaled:0x%x\n",
		fence->ops->get_driver_name(fence),
		fence->ops->get_timeline_name(fence),
		fence->seqno, timeline_str,
		fence->ops->signaled ?
		fence->ops->signaled(fence) : 0xffffffff);
}

void sde_debugfs_timeline_dump(struct sde_fence_context *ctx,
		struct drm_mode_object *drm_obj, struct seq_file **s)
{
	char *obj_name;
	struct sde_fence *fc, *next;
	struct dma_fence *fence;

	if (!ctx || !drm_obj) {
		SDE_ERROR("invalid input params\n");
		return;
	}

	switch (drm_obj->type) {
	case DRM_MODE_OBJECT_CRTC:
		obj_name = "crtc";
		break;
	case DRM_MODE_OBJECT_CONNECTOR:
		obj_name = "connector";
		break;
	default:
		obj_name = "unknown";
		break;
	}

	seq_printf(*s, "drm obj:%s id:%d type:0x%x done_count:%d commit_count:%d\n",
		obj_name, drm_obj->id, drm_obj->type, ctx->done_count,
		ctx->commit_count);

	spin_lock(&ctx->list_lock);
	list_for_each_entry_safe(fc, next, &ctx->fence_list_head, fence_list) {
		fence = &fc->base;
		sde_fence_list_dump(fence, s);
	}
	spin_unlock(&ctx->list_lock);
}
