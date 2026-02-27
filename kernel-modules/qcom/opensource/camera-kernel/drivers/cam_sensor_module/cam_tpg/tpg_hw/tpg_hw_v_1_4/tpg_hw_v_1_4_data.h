/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __TPG_HW_V_1_4_DATA_H__
#define __TPG_HW_V_1_4_DATA_H__

#include "../tpg_hw.h"
#include "../tpg_hw_common.h"
#include "tpg_hw_v_1_4.h"
#include "tpg_hw_v_1_4_0.h"

struct tpg_hw_ops tpg_hw_v_1_4_ops = {
	.start = tpg_hw_v_1_4_start,
	.stop  = tpg_hw_v_1_4_stop,
	.init  = tpg_hw_v_1_4_init,
	.process_cmd = tpg_hw_v_1_4_process_cmd,
	.dump_status = tpg_hw_v_1_4_dump_status,
	.write_settings = tpg_hw_write_settings,
	.handle_irq  = tpg_hw_v_1_4_handle_irq,
};

struct tpg_hw_info tpg_v_1_4_hw_info = {
	.version = TPG_HW_VERSION_1_4,
	.max_vc_channels = 4,
	.max_dt_channels_per_vc = 4,
	.ops = &tpg_hw_v_1_4_ops,
	.hw_data = &cam_tpg104_reg,
	.layer_init = &tpg_1_4_layer_init,
	.xcfa_debug = 0,
	.shdr_overlap = 0,
	.shdr_offset_num_batch = 0,
	.shdr_line_offset0 = 0x6c,
	.shdr_line_offset1 = 0x6c,
};

#endif
