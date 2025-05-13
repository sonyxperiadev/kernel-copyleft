/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __H_MSM_VIDC_FENCE_H__
#define __H_MSM_VIDC_FENCE_H__

#include "msm_vidc_inst.h"
#include "msm_vidc_buffer.h"

int msm_vidc_fence_init(struct msm_vidc_inst *inst);
void msm_vidc_fence_deinit(struct msm_vidc_inst *inst);

#define call_fence_op(c, op, ...)                  \
	(((c) && (c)->fence_ops && (c)->fence_ops->op) ? \
	((c)->fence_ops->op(__VA_ARGS__)) : 0)

struct msm_vidc_fence_ops {
	int (*fence_register)(struct msm_vidc_core *core);
	int (*fence_deregister)(struct msm_vidc_core *core);
	struct msm_vidc_fence *(*fence_create)(struct msm_vidc_inst *inst);
	int (*fence_create_fd)(struct msm_vidc_inst *inst,
			       struct msm_vidc_fence *fence);
	void (*fence_destroy)(struct msm_vidc_inst *inst,
			      u64 fence_id);
	int (*fence_signal)(struct msm_vidc_inst *inst,
			    u64 fence_id);
	void (*fence_recover)(struct msm_vidc_core *core);
};

const struct msm_vidc_fence_ops *get_dma_fence_ops(void);

#endif // __H_MSM_VIDC_FENCE_H__
