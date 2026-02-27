/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_PINEAPPLE_H_
#define _MSM_VIDC_PINEAPPLE_H_

#include "msm_vidc_core.h"

#if defined(CONFIG_MSM_VIDC_PINEAPPLE)
int msm_vidc_init_platform_pineapple(struct msm_vidc_core *core);
#else
int msm_vidc_init_platform_pineapple(struct msm_vidc_core *core)
{
	return -EINVAL;
}
#endif

#endif // _MSM_VIDC_PINEAPPLE_H_
