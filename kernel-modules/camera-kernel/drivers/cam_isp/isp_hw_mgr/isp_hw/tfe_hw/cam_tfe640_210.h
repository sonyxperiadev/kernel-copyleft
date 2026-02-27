/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#ifndef _CAM_TFE640_210_H_
#define _CAM_TFE640_210_H_
#include "cam_tfe_core.h"
#include "cam_tfe_bus.h"
#include "cam_tfe640.h"

struct cam_tfe_hw_info cam_tfe640_210 = {
	.top_irq_mask = {
		0x00001834,
		0x00001838,
		0x0000183C,
	},
	.top_irq_clear = {
		0x00001840,
		0x00001844,
		0x00001848,
	},
	.top_irq_status = {
		0x0000184C,
		0x00001850,
		0x00001854,
	},
	.top_irq_cmd                       = 0x00001830,
	.global_clear_bitmask              = 0x00000001,
	.bus_irq_mask = {
		0x00003018,
		0x0000301C,
	},
	.bus_irq_clear = {
		0x00003020,
		0x00003024,
	},
	.bus_irq_status = {
		0x00003028,
		0x0000302C,
	},
	.bus_irq_cmd = 0x00003030,
	.bus_violation_reg = 0x00003064,
	.bus_overflow_reg = 0x00003068,
	.bus_image_size_vilation_reg = 0x3070,
	.bus_overflow_clear_cmd = 0x3060,
	.debug_status_top = 0x30D8,

	.reset_irq_mask = {
		0x00000001,
		0x00000000,
		0x00000000,
	},
	.error_irq_mask = {
		0x000F0F00,
		0x00000000,
		0x0000003F,
	},
	.bus_reg_irq_mask = {
		0x00000002,
		0x00000000,
	},
	.bus_error_irq_mask = {
		0xC0000000,
		0x00000000,
	},

	.num_clc = 39,
	.clc_hw_status_info            = tfe640_clc_hw_info,
	.bus_version                   = CAM_TFE_BUS_1_0,
	.bus_hw_info                   = &tfe640_bus_hw_info,

	.top_version                   = CAM_TFE_TOP_1_0,
	.top_hw_info                   = &tfe640_top_hw_info,
};

#endif /* _CAM_TFE640_210_H_ */
