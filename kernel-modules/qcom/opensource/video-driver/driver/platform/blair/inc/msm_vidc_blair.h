/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_BLAIR_H_
#define _MSM_VIDC_BLAIR_H_

#include "msm_vidc_core.h"

#if defined(CONFIG_MSM_VIDC_BLAIR)
int msm_vidc_init_platform_blair(struct msm_vidc_core *core, struct device *dev);
int msm_vidc_deinit_platform_blair(struct msm_vidc_core *core, struct device *dev);
#else
int msm_vidc_init_platform_blair(struct msm_vidc_core *core, struct device *dev)
{
	return -EINVAL;
}
int msm_vidc_deinit_platform_blair(struct msm_vidc_core *core, struct device *dev)
{
	return -EINVAL;
}
#endif

#endif // _MSM_VIDC_BLAIR_H_
