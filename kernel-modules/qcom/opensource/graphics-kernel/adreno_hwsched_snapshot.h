/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _ADRENO_HWSCHED_SNAPSHOT_H_
#define _ADRENO_HWSCHED_SNAPSHOT_H_

bool adreno_hwsched_parse_payload_rb_legacy(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot);

bool adreno_hwsched_parse_payload_rb(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot);

size_t adreno_hwsched_snapshot_rb(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv);

size_t adreno_hwsched_snapshot_aqe_buffer(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv);

void adreno_hwsched_snapshot_context_queue(struct kgsl_device *device,
	struct kgsl_snapshot *snapshot);

void adreno_hwsched_snapshot_preemption_record(struct kgsl_device *device,
	struct kgsl_snapshot *snapshot, struct kgsl_memdesc *md, u64 offset, u64 size);

#endif
