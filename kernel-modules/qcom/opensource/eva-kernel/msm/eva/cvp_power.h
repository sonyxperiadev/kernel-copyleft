/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CVP_POWER_H_
#define _CVP_POWER_H_

#include "msm_cvp_internal.h"
#include "msm_cvp_common.h"
#include "msm_cvp_clocks.h"
#include "msm_cvp_debug.h"
#include "msm_cvp_dsp.h"

struct cvp_power_level {
	unsigned long core_sum;
	unsigned long op_core_sum;
	unsigned long bw_sum;
};

int msm_cvp_update_power(struct msm_cvp_inst *inst);
unsigned int msm_cvp_get_hw_aggregate_cycles(enum hfi_hw_thread hwblk);
#endif
