/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_WAIPIO_H_
#define _MSM_VIDC_WAIPIO_H_

#include "msm_vidc_core.h"
#include "msm_vidc_iris2.h"

#if defined(CONFIG_MSM_VIDC_WAIPIO)
struct context_bank_info *msm_vidc_context_bank(struct msm_vidc_core *core,
	enum msm_vidc_buffer_region region);
int msm_vidc_init_platform_waipio(struct msm_vidc_core *core);
#else
struct context_bank_info *msm_vidc_context_bank(struct msm_vidc_core *core,
	enum msm_vidc_buffer_region region)
{
	return NULL;
}

int msm_vidc_init_platform_waipio(struct msm_vidc_core *core)
{
	return -EINVAL;
}
#endif

#endif // _MSM_VIDC_WAIPIO_H_
