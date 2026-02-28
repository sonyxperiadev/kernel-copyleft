// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "adreno.h"
#include "adreno_hwsched_snapshot.h"
#include "adreno_snapshot.h"

size_t adreno_hwsched_snapshot_rb(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv)
{
	struct kgsl_snapshot_rb_v2 *header = (struct kgsl_snapshot_rb_v2 *)buf;
	u32 *data = (u32 *)(buf + sizeof(*header));
	struct kgsl_memdesc *rb = (struct kgsl_memdesc *)priv;

	if (remain < rb->size + sizeof(*header)) {
		SNAPSHOT_ERR_NOMEM(device, "RB");
		return 0;
	}

	header->start = 0;
	header->end = rb->size >> 2;
	header->rptr = 0;
	header->rbsize = rb->size >> 2;
	header->count = rb->size >> 2;
	header->timestamp_queued = 0;
	header->timestamp_retired = 0;
	header->gpuaddr = rb->gpuaddr;
	header->id = 0;

	memcpy(data, rb->hostptr, rb->size);

	return rb->size + sizeof(*header);
}

static u32 copy_gpu_global(void *out, void *in, u32 size)
{
	if (out && in) {
		memcpy(out, in, size);
		return size;
	}

	return 0;
}

static void adreno_hwsched_snapshot_rb_payload(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot, struct payload_section *payload)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_snapshot_section_header *section_header =
		(struct kgsl_snapshot_section_header *)snapshot->ptr;
	u8 *buf = snapshot->ptr + sizeof(*section_header);
	struct kgsl_snapshot_rb_v2 *header = (struct kgsl_snapshot_rb_v2 *)buf;
	u32 *data = (u32 *)(buf + sizeof(*header));
	u32 size = adreno_hwsched_parse_payload(payload, KEY_RB_SIZEDWORDS) << 2;
	const struct adreno_hwsched_ops *hwsched_ops = adreno_dev->hwsched.hwsched_ops;
	u64 lo, hi, gpuaddr;
	void *rb_hostptr = NULL;
	char str[16];

	lo = adreno_hwsched_parse_payload(payload, KEY_RB_GPUADDR_LO);
	hi = adreno_hwsched_parse_payload(payload, KEY_RB_GPUADDR_HI);
	gpuaddr = hi << 32 | lo;

	/* Sanity check to make sure there is enough for the header */
	if (snapshot->remain < sizeof(*section_header))
		goto err;

	if (hwsched_ops->get_rb_hostptr)
		rb_hostptr = hwsched_ops->get_rb_hostptr(adreno_dev, gpuaddr, size);

	if (rb_hostptr == NULL)
		goto err;

	/* If the gpuaddress and size don't match any allocation, then abort */
	if (((snapshot->remain - sizeof(*section_header)) < (size + sizeof(*header))) ||
		!copy_gpu_global(data, rb_hostptr, size))
		goto err;

	if (device->dump_all_ibs) {
		u64 rbaddr, lpac_rbaddr;

		adreno_readreg64(adreno_dev, ADRENO_REG_CP_RB_BASE,
			ADRENO_REG_CP_RB_BASE_HI, &rbaddr);

		adreno_readreg64(adreno_dev, ADRENO_REG_CP_LPAC_RB_BASE,
			ADRENO_REG_CP_LPAC_RB_BASE_HI, &lpac_rbaddr);

		/* Parse all IBs from current RB */
		if ((rbaddr == gpuaddr) || (lpac_rbaddr == gpuaddr))
			adreno_snapshot_dump_all_ibs(device, rb_hostptr, snapshot);
	}

	header->start = 0;
	header->end = size >> 2;
	header->rptr = adreno_hwsched_parse_payload(payload, KEY_RB_RPTR);
	header->wptr = adreno_hwsched_parse_payload(payload, KEY_RB_WPTR);
	header->rbsize = size >> 2;
	header->count = size >> 2;
	header->timestamp_queued = adreno_hwsched_parse_payload(payload,
			KEY_RB_QUEUED_TS);
	header->timestamp_retired = adreno_hwsched_parse_payload(payload,
			KEY_RB_RETIRED_TS);
	header->gpuaddr = gpuaddr;
	header->id = adreno_hwsched_parse_payload(payload, KEY_RB_ID);

	section_header->magic = SNAPSHOT_SECTION_MAGIC;
	section_header->id = KGSL_SNAPSHOT_SECTION_RB_V2;
	section_header->size = size + sizeof(*header) + sizeof(*section_header);

	snapshot->ptr += section_header->size;
	snapshot->remain -= section_header->size;
	snapshot->size += section_header->size;

	return;
err:
	snprintf(str, sizeof(str), "RB addr:0x%llx", gpuaddr);
	SNAPSHOT_ERR_NOMEM(device, str);
}

bool adreno_hwsched_parse_payload_rb_legacy(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot)
{
	struct hfi_context_bad_cmd_legacy *cmd = adreno_dev->hwsched.ctxt_bad;
	u32 i = 0, payload_bytes;
	void *start;
	bool ret = false;

	/* Skip if we didn't receive a context bad HFI */
	if (!cmd->hdr)
		return false;

	payload_bytes = (MSG_HDR_GET_SIZE(cmd->hdr) << 2) -
			offsetof(struct hfi_context_bad_cmd_legacy, payload);

	start = &cmd->payload[0];

	while (i < payload_bytes) {
		struct payload_section *payload = start + i;

		if (payload->type == PAYLOAD_RB) {
			adreno_hwsched_snapshot_rb_payload(adreno_dev, snapshot, payload);
			ret = true;
		}

		i += sizeof(*payload) + (payload->dwords << 2);
	}

	return ret;
}

bool adreno_hwsched_parse_payload_rb(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot)
{
	struct hfi_context_bad_cmd *cmd = adreno_dev->hwsched.ctxt_bad;
	u32 i = 0, payload_bytes;
	void *start;
	bool ret = false;

	/* Skip if we didn't receive a context bad HFI */
	if (!cmd->hdr)
		return false;

	payload_bytes = (MSG_HDR_GET_SIZE(cmd->hdr) << 2) -
			offsetof(struct hfi_context_bad_cmd, payload);

	start = &cmd->payload[0];

	while (i < payload_bytes) {
		struct payload_section *payload = start + i;

		if (payload->type == PAYLOAD_RB) {
			adreno_hwsched_snapshot_rb_payload(adreno_dev,
							snapshot, payload);
			ret = true;
		}

		i += sizeof(*payload) + (payload->dwords << 2);
	}

	return ret;
}

size_t adreno_hwsched_snapshot_aqe_buffer(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv)
{
	struct kgsl_memdesc *memdesc = priv;

	struct kgsl_snapshot_gpu_object_v2 *header =
		(struct kgsl_snapshot_gpu_object_v2 *)buf;

	u8 *ptr = buf + sizeof(*header);

	if (IS_ERR_OR_NULL(memdesc) || memdesc->size == 0)
		return 0;

	if (remain < (memdesc->size + sizeof(*header))) {
		SNAPSHOT_ERR_NOMEM(device, "AQE BUFFER");
		return 0;
	}

	header->size = memdesc->size >> 2;
	header->gpuaddr = memdesc->gpuaddr;
	header->ptbase = MMU_DEFAULT_TTBR0(device);
	header->type = SNAPSHOT_GPU_OBJECT_GLOBAL;

	memcpy(ptr, memdesc->hostptr, memdesc->size);

	return memdesc->size + sizeof(*header);
}

static int snapshot_context_queue(int id, void *ptr, void *data)
{
	struct kgsl_snapshot *snapshot = data;
	struct kgsl_context *context = ptr;
	struct adreno_context *drawctxt = ADRENO_CONTEXT(context);
	struct gmu_mem_type_desc desc;

	if (!context->gmu_registered)
		return 0;

	desc.memdesc = &drawctxt->gmu_context_queue;
	desc.type = SNAPSHOT_GMU_MEM_CONTEXT_QUEUE;
	kgsl_snapshot_add_section(context->device,
		KGSL_SNAPSHOT_SECTION_GMU_MEMORY,
		snapshot, adreno_snapshot_gmu_mem, &desc);

	return 0;
}

void adreno_hwsched_snapshot_context_queue(struct kgsl_device *device,
	struct kgsl_snapshot *snapshot)
{
	if (!adreno_hwsched_context_queue_enabled(ADRENO_DEVICE(device)))
		return;

	read_lock(&device->context_lock);
	idr_for_each(&device->context_idr, snapshot_context_queue, snapshot);
	read_unlock(&device->context_lock);
}

void adreno_hwsched_snapshot_preemption_record(struct kgsl_device *device,
	struct kgsl_snapshot *snapshot, struct kgsl_memdesc *md, u64 offset, u64 size)
{
	struct kgsl_snapshot_section_header *section_header =
		(struct kgsl_snapshot_section_header *)snapshot->ptr;
	u8 *dest = snapshot->ptr + sizeof(*section_header);
	struct kgsl_snapshot_gpu_object_v2 *header =
		(struct kgsl_snapshot_gpu_object_v2 *)dest;
	u64 ctxt_record_size = min_t(u64, device->snapshot_ctxt_record_size, size);
	size_t section_size;

	if (WARN_RATELIMIT((ctxt_record_size > md->size) ||
		(offset > (md->size - ctxt_record_size)),
		"Invalid preemption context record size: md_size: 0x%llx, ctxt_record_size: 0x%llx\n",
		md->size, ctxt_record_size))
		return;

	section_size = sizeof(*section_header) + sizeof(*header) + ctxt_record_size;
	if (snapshot->remain < section_size) {
		SNAPSHOT_ERR_NOMEM(device, "PREEMPTION RECORD");
		return;
	}

	section_header->magic = SNAPSHOT_SECTION_MAGIC;
	section_header->id = KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2;
	section_header->size = section_size;

	header->size = ctxt_record_size >> 2;
	header->gpuaddr = md->gpuaddr + offset;
	header->ptbase =
		kgsl_mmu_pagetable_get_ttbr0(device->mmu.defaultpagetable);
	header->type = SNAPSHOT_GPU_OBJECT_GLOBAL;

	dest += sizeof(*header);

	memcpy(dest, md->hostptr + offset, ctxt_record_size);

	snapshot->ptr += section_header->size;
	snapshot->remain -= section_header->size;
	snapshot->size += section_header->size;
}
