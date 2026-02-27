/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_SFE860_H_
#define _CAM_SFE860_H_
#include "cam_sfe_core.h"
#include "cam_sfe_bus.h"
#include "cam_sfe_bus_rd.h"
#include "cam_sfe_bus_wr.h"
#include "cam_sfe880.h"

static struct cam_sfe_top_common_reg_offset  sfe860_top_commong_reg  = {
	.hw_version                    = 0x00000000,
	.hw_capability                 = 0x00000004,
	.stats_feature                 = 0x00000008,
	.core_cgc_ctrl                 = 0x00000010,
	.ahb_clk_ovd                   = 0x00000014,
	.core_cfg                      = 0x000000CC,
	.ipp_violation_status          = 0x00000030,
	.diag_config                   = 0x00000034,
	.diag_sensor_status_0          = 0x00000038,
	.diag_sensor_status_1          = 0x0000003C,
	.diag_sensor_frame_cnt_status0 = 0x00000040,
	.diag_sensor_frame_cnt_status1 = 0x00000044,
	.stats_ch2_throttle_cfg        = 0x000000B0,
	.stats_ch1_throttle_cfg        = 0x000000B4,
	.stats_ch0_throttle_cfg        = 0x000000B8,
	.hdr_throttle_cfg              = 0x000000C0,
	.sfe_op_throttle_cfg           = 0x000000C4,
	.irc_throttle_cfg              = 0x000000C8,
	.sfe_single_dual_cfg           = 0x000000D0,
	.bus_overflow_status           = 0x00000868,
	.num_perf_counters             = 2,
	.perf_count_reg = {
		{
			.perf_count_cfg        = 0x00000080,
			.perf_pix_count        = 0x00000084,
			.perf_line_count       = 0x00000088,
			.perf_stall_count      = 0x0000008C,
			.perf_always_count     = 0x00000090,
			.perf_count_status     = 0x00000094,
		},
		{
			.perf_count_cfg        = 0x00000098,
			.perf_pix_count        = 0x0000009C,
			.perf_line_count       = 0x000000A0,
			.perf_stall_count      = 0x000000A4,
			.perf_always_count     = 0x000000A8,
			.perf_count_status     = 0x000000AC,
		},
	},
	.top_debug_cfg                  = 0x0000007C,
	.top_cc_test_bus_ctrl           = 0x000001F0,
	.lcr_supported                  = false,
	.ir_supported                   = true,
	.qcfa_only                      = false,
	.num_sfe_mode                   = ARRAY_SIZE(sfe_880_mode),
	.sfe_mode                       = sfe_880_mode,
	.ipp_violation_mask             = 0x4000,
	.top_debug_testbus_reg          = 13,
	.top_debug_nonccif_regstart_idx = 12,
	.top_cc_test_bus_supported      = true,
	.num_debug_registers            = 20,
	.top_debug = {
		0x0000004C,
		0x00000050,
		0x00000054,
		0x00000058,
		0x0000005C,
		0x00000060,
		0x00000064,
		0x00000068,
		0x0000006C,
		0x00000070,
		0x00000074,
		0x00000078,
		0x000000EC,
		0x000000F0,
		0x000000F4,
		0x000000F8,
		0x000000FC,
		0x00000100,
		0x00000104,
		0x00000108,
	},
};

static struct cam_sfe_top_hw_info sfe860_top_hw_info = {
	.common_reg = &sfe860_top_commong_reg,
	.modules_hw_info = &sfe880_modules_common_reg,
	.common_reg_data = &sfe_880_top_common_reg_data,
	.ipp_module_desc =  sfe_880_mod_desc,
	.wr_client_desc  =  sfe_880_wr_client_desc,
	.pix_reg_data    = &sfe_880_pix_reg_data,
	.rdi_reg_data[0] = &sfe_880_rdi0_reg_data,
	.rdi_reg_data[1] = &sfe_880_rdi1_reg_data,
	.rdi_reg_data[2] = &sfe_880_rdi2_reg_data,
	.rdi_reg_data[3] = &sfe_880_rdi3_reg_data,
	.rdi_reg_data[4] = &sfe_880_rdi4_reg_data,
	.num_inputs = 6,
	.input_type = {
		CAM_SFE_PIX_VER_1_0,
		CAM_SFE_RDI_VER_1_0,
		CAM_SFE_RDI_VER_1_0,
		CAM_SFE_RDI_VER_1_0,
		CAM_SFE_RDI_VER_1_0,
		CAM_SFE_RDI_VER_1_0,
	},
	.num_top_errors  = ARRAY_SIZE(sfe_880_top_irq_err_desc),
	.top_err_desc    = sfe_880_top_irq_err_desc,
	.num_clc_module  = 12,
	.clc_dbg_mod_info = &sfe880_clc_dbg_module_info,
	.num_of_testbus = 2,
	.test_bus_info = {
		/* TEST BUS 1 INFO */
		{
			.debugfs_val  = SFE_DEBUG_ENABLE_TESTBUS1,
			.enable       = false,
			.value        = 0x1,
			.size         = ARRAY_SIZE(sfe880_testbus1_info),
			.testbus      = sfe880_testbus1_info,
		},
		/* TEST BUS 2 INFO */
		{
			.debugfs_val  = SFE_DEBUG_ENABLE_TESTBUS2,
			.enable       = false,
			.value        = 0x3,
			.size         = ARRAY_SIZE(sfe880_testbus2_info),
			.testbus      = sfe880_testbus2_info,
		},
	},
};

struct cam_sfe_hw_info cam_sfe860_hw_info = {
	.irq_reg_info                  = &sfe880_top_irq_reg_info,

	.bus_wr_version                = CAM_SFE_BUS_WR_VER_1_0,
	.bus_wr_hw_info                = &sfe880_bus_wr_hw_info,

	.bus_rd_version                = CAM_SFE_BUS_RD_VER_1_0,
	.bus_rd_hw_info                = &sfe880_bus_rd_hw_info,

	.top_version                   = CAM_SFE_TOP_VER_1_0,
	.top_hw_info                   = &sfe860_top_hw_info,
};

#endif /* _CAM_SFE860_H_ */
