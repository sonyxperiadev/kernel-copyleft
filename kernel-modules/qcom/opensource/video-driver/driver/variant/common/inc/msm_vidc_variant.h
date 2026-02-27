/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_VARIANT_H_
#define _MSM_VIDC_VARIANT_H_

#include <linux/types.h>

struct msm_vidc_core;

int __write_register_masked(struct msm_vidc_core *core, u32 reg, u32 value,
		u32 mask);
int __write_register(struct msm_vidc_core *core, u32 reg, u32 value);
int __read_register(struct msm_vidc_core *core, u32 reg, u32 *value);
int __read_register_with_poll_timeout(struct msm_vidc_core *core, u32 reg,
		u32 mask, u32 exp_val, u32 sleep_us, u32 timeout_us);
int __set_registers(struct msm_vidc_core *core);

#endif
