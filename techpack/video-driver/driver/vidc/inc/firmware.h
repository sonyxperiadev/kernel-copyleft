/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_FIRMWARE_H_
#define _MSM_VIDC_FIRMWARE_H_

struct msm_vidc_core;

int fw_load(struct msm_vidc_core *core);
int fw_unload(struct msm_vidc_core *core);
int fw_suspend(struct msm_vidc_core *core);
int fw_resume(struct msm_vidc_core *core);
void fw_coredump(struct msm_vidc_core *core);

#endif
