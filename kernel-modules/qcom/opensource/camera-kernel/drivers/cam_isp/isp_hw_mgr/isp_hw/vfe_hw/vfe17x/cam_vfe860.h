/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_VFE860_H_
#define _CAM_VFE860_H_
#include "cam_vfe_top_ver4.h"
#include "cam_vfe_core.h"
#include "cam_vfe_bus_ver3.h"
#include "cam_irq_controller.h"
#include "cam_vfe880.h"

static struct cam_vfe_top_ver4_reg_offset_common vfe860_top_common_reg = {
	.hw_version               = 0x00000000,
	.hw_capability            = 0x00000004,
	.lens_feature             = 0x00000008,
	.stats_feature            = 0x0000000C,
	.color_feature            = 0x00000010,
	.zoom_feature             = 0x00000014,
	.core_cfg_0               = 0x00000024,
	.core_cfg_1               = 0x00000028,
	.core_cfg_2               = 0x0000002C,
	.global_reset_cmd         = 0x00000030,
	.diag_config              = 0x00000050,
	.diag_sensor_status_0     = 0x00000054,
	.diag_sensor_status_1     = 0x00000058,
	.diag_frm_cnt_status_0    = 0x0000005C,
	.diag_frm_cnt_status_1    = 0x00000060,
	.ipp_violation_status     = 0x00000064,
	.pdaf_violation_status    = 0x00000404,
	.core_cgc_ovd_0           = 0x00000018,
	.core_cgc_ovd_1           = 0x0000001C,
	.ahb_cgc_ovd              = 0x00000020,
	.dsp_status               = 0x0000006C,
	.stats_throttle_cfg_0     = 0x00000070,
	.stats_throttle_cfg_1     = 0x00000074,
	.stats_throttle_cfg_2     = 0x00000078,
	.core_cfg_4               = 0x00000080,
	.core_cfg_5               = 0x00000084,
	.core_cfg_6               = 0x00000088,
	.period_cfg               = 0x0000008C,
	.irq_sub_pattern_cfg      = 0x00000090,
	.epoch0_pattern_cfg       = 0x00000094,
	.epoch1_pattern_cfg       = 0x00000098,
	.epoch_height_cfg         = 0x0000009C,
	.bus_violation_status     = 0x00000C64,
	.bus_overflow_status      = 0x00000C68,
	.num_perf_counters        = 2,
	.perf_count_reg = {
		{
			.perf_count_cfg    = 0x00000100,
			.perf_pix_count    = 0x00000104,
			.perf_line_count   = 0x00000108,
			.perf_stall_count  = 0x0000010C,
			.perf_always_count = 0x00000110,
			.perf_count_status = 0x00000114,
		},
		{
			.perf_count_cfg    = 0x00000118,
			.perf_pix_count    = 0x0000011C,
			.perf_line_count   = 0x00000120,
			.perf_stall_count  = 0x00000124,
			.perf_always_count = 0x00000128,
			.perf_count_status = 0x0000012C,
		},
	},
	.top_debug_cfg            = 0x000000FC,
	.num_top_debug_reg        = CAM_VFE_880_NUM_DBG_REG,
	.pdaf_input_cfg_0         = 0x00000130,
	.pdaf_input_cfg_1         = 0x00000134,
	.top_debug = vfe880_top_debug_reg,
	.frame_timing_irq_reg_idx = CAM_IFE_IRQ_CAMIF_REG_STATUS1,
};

struct cam_vfe_ver4_path_hw_info
	vfe860_rdi_hw_info_arr[] = {
	{
		.common_reg     = &vfe860_top_common_reg,
		.reg_data       = &vfe880_vfe_full_rdi_reg_data[0],
	},
	{
		.common_reg     = &vfe860_top_common_reg,
		.reg_data       = &vfe880_vfe_full_rdi_reg_data[1],
	},
	{
		.common_reg     = &vfe860_top_common_reg,
		.reg_data       = &vfe880_vfe_full_rdi_reg_data[2],
	},
};

static struct cam_vfe_top_ver4_hw_info vfe860_top_hw_info = {
	.common_reg = &vfe860_top_common_reg,
	.vfe_full_hw_info = {
		.common_reg     = &vfe860_top_common_reg,
		.reg_data       = &vfe880_pp_common_reg_data,
	},
	.pdlib_hw_info = {
		.common_reg     = &vfe860_top_common_reg,
		.reg_data       = &vfe880_pdlib_reg_data,
	},
	.rdi_hw_info            = vfe860_rdi_hw_info_arr,
	.wr_client_desc         = vfe880_wr_client_desc,
	.ipp_module_desc        = vfe880_ipp_mod_desc,
	.num_mux = 5,
	.mux_type = {
		CAM_VFE_CAMIF_VER_4_0,
		CAM_VFE_PDLIB_VER_1_0,
		CAM_VFE_RDI_VER_1_0,
		CAM_VFE_RDI_VER_1_0,
		CAM_VFE_RDI_VER_1_0,
	},
	.num_path_port_map = 3,
	.path_port_map = {
		{CAM_ISP_HW_VFE_IN_PDLIB, CAM_ISP_IFE_OUT_RES_2PD},
		{CAM_ISP_HW_VFE_IN_PDLIB, CAM_ISP_IFE_OUT_RES_PREPROCESS_2PD},
		{CAM_ISP_HW_VFE_IN_PDLIB, CAM_ISP_IFE_OUT_RES_PDAF_PARSED_DATA},
	},
	.num_rdi                         = ARRAY_SIZE(vfe860_rdi_hw_info_arr),
	.num_top_errors                  = ARRAY_SIZE(vfe880_top_irq_err_desc),
	.top_err_desc                    = vfe880_top_irq_err_desc,
	.num_pdaf_violation_errors       = ARRAY_SIZE(vfe880_pdaf_violation_desc),
	.pdaf_violation_desc             = vfe880_pdaf_violation_desc,
	.debug_reg_info                  = &vfe880_dbg_reg_info,
	.pdaf_lcr_res_mask               = vfe880_pdaf_lcr_res_mask,
	.num_pdaf_lcr_res                = ARRAY_SIZE(vfe880_pdaf_lcr_res_mask),
};

static struct cam_vfe_hw_info cam_vfe860_hw_info = {
	.irq_hw_info                   = &vfe880_irq_hw_info,

	.bus_version                   = CAM_VFE_BUS_VER_3_0,
	.bus_hw_info                   = &vfe880_bus_hw_info,

	.top_version                   = CAM_VFE_TOP_VER_4_0,
	.top_hw_info                   = &vfe860_top_hw_info,
};

#endif /* _CAM_VFE860_H_ */
