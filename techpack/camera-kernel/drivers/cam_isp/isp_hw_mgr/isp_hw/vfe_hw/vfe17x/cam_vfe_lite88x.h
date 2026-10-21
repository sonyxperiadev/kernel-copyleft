/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#ifndef _CAM_VFE_LITE88X_H_
#define _CAM_VFE_LITE88X_H_
#include "cam_vfe_camif_ver3.h"
#include "cam_vfe_top_ver4.h"
#include "cam_vfe_core.h"
#include "cam_vfe_bus_ver3.h"
#include "cam_irq_controller.h"
#include "cam_vfe_lite78x.h"

#define CAM_VFE_88X_NUM_DBG_REG 5

static struct cam_irq_register_set vfe_lite88x_top_irq_reg_set[2] = {
	{
		.mask_reg_offset   = 0x00002024,
		.clear_reg_offset  = 0x0000202C,
		.status_reg_offset = 0x0000201C,
		.set_reg_offset    = 0x00002034,
		.test_set_val      = BIT(0),
		.test_sub_val      = BIT(0),
	},
	{
		.mask_reg_offset   = 0x00002028,
		.clear_reg_offset  = 0x00002030,
		.status_reg_offset = 0x00002020,
	},
};

static struct cam_irq_controller_reg_info vfe_lite88x_top_irq_reg_info = {
	.num_registers = 2,
	.irq_reg_set = vfe_lite88x_top_irq_reg_set,
	.global_irq_cmd_offset = 0x00002038,
	.global_clear_bitmask  = 0x00000001,
	.global_set_bitmask    = 0x00000010,
	.clear_all_bitmask     = 0xFFFFFFFF,
};

static struct cam_vfe_top_ver4_reg_offset_common vfe_lite88x_top_common_reg = {
	.hw_version               = 0x00002000,
	.hw_capability            = 0x00002004,
	.core_cgc_ovd_0           = 0x00002014,
	.ahb_cgc_ovd              = 0x00002018,
	.core_cfg_0               = 0x0000203C,
	.diag_config              = 0x00002040,
	.diag_sensor_status_0     = 0x00002044,
	.diag_sensor_status_1     = 0x00002048,
	.ipp_violation_status     = 0x00002054,
	.bus_violation_status     = 0x00002264,
	.bus_overflow_status      = 0x00002268,
	.top_debug_cfg            = 0x00002074,
	.num_top_debug_reg        = CAM_VFE_88X_NUM_DBG_REG,
	.top_debug                = {
		0x0000205C,
		0x00002060,
		0x00002064,
		0x00002068,
		0x0000206C,
	},
};

static struct cam_vfe_ver4_path_reg_data vfe_lite88x_ipp_reg_data = {
	.sof_irq_mask                    = 0x1,
	.eof_irq_mask                    = 0x2,
	.error_irq_mask                  = 0x2,
	.enable_diagnostic_hw            = 0x1,
	.top_debug_cfg_en                = 0x3,
	.ipp_violation_mask              = 0x10,
};

static struct cam_vfe_ver4_path_reg_data vfe_lite88x_rdi_reg_data[4] = {

	{
		.sof_irq_mask                    = 0x4,
		.eof_irq_mask                    = 0x8,
		.error_irq_mask                  = 0x0,
		.enable_diagnostic_hw            = 0x1,
		.top_debug_cfg_en                = 0x3,
	},
	{
		.sof_irq_mask                    = 0x10,
		.eof_irq_mask                    = 0x20,
		.error_irq_mask                  = 0x0,
		.enable_diagnostic_hw            = 0x1,
		.top_debug_cfg_en                = 0x3,
	},
	{
		.sof_irq_mask                    = 0x40,
		.eof_irq_mask                    = 0x80,
		.error_irq_mask                  = 0x0,
		.enable_diagnostic_hw            = 0x1,
		.top_debug_cfg_en                = 0x3,
	},
	{
		.sof_irq_mask                    = 0x100,
		.eof_irq_mask                    = 0x200,
		.error_irq_mask                  = 0x0,
		.enable_diagnostic_hw            = 0x1,
		.top_debug_cfg_en                = 0x3,
	},
};

static struct cam_vfe_ver4_path_hw_info
	vfe_lite88x_rdi_hw_info[CAM_VFE_RDI_VER2_MAX] = {
	{
		.common_reg     = &vfe_lite88x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[0],
	},
	{
		.common_reg     = &vfe_lite88x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[1],
	},
	{
		.common_reg     = &vfe_lite88x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[2],
	},
	{
		.common_reg     = &vfe_lite88x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[3],
	},
};

static struct cam_vfe_top_ver4_hw_info vfe_lite88x_top_hw_info = {
	.common_reg = &vfe_lite88x_top_common_reg,
	.rdi_hw_info[0] = &vfe_lite88x_rdi_hw_info[0],
	.rdi_hw_info[1] = &vfe_lite88x_rdi_hw_info[1],
	.rdi_hw_info[2] = &vfe_lite88x_rdi_hw_info[2],
	.rdi_hw_info[3] = &vfe_lite88x_rdi_hw_info[3],
	.vfe_full_hw_info = {
		.common_reg     = &vfe_lite88x_top_common_reg,
		.reg_data       = &vfe_lite88x_ipp_reg_data,
	},
	.ipp_module_desc        = vfe_lite78x_ipp_mod_desc,
	.wr_client_desc         = vfe_lite78x_wr_client_desc,
	.num_mux = 5,
	.mux_type = {
		CAM_VFE_CAMIF_VER_4_0,
		CAM_VFE_RDI_VER_1_0,
		CAM_VFE_RDI_VER_1_0,
		CAM_VFE_RDI_VER_1_0,
		CAM_VFE_RDI_VER_1_0,
	},
	.debug_reg_info = &vfe78x_dbg_reg_info,
};

static struct cam_irq_register_set vfe_lite88x_bus_irq_reg[1] = {
	{
		.mask_reg_offset   = 0x00002218,
		.clear_reg_offset  = 0x00002220,
		.status_reg_offset = 0x00002228,
	},
};

static struct cam_vfe_bus_ver3_hw_info vfe_lite88x_bus_hw_info = {
	.common_reg = {
		.hw_version                       = 0x00002200,
		.cgc_ovd                          = 0x00002208,
		.if_frameheader_cfg               = {
			0x00002234,
			0x00002238,
			0x0000223C,
			0x00002240,
			0x00002244,
		},
		.pwr_iso_cfg                      = 0x0000225C,
		.overflow_status_clear            = 0x00002260,
		.ccif_violation_status            = 0x00002264,
		.overflow_status                  = 0x00002268,
		.image_size_violation_status      = 0x00002270,
		.debug_status_top_cfg             = 0x000022F0,
		.debug_status_top                 = 0x000022F4,
		.test_bus_ctrl                    = 0x00002394,
		.irq_reg_info = {
			.num_registers            = 1,
			.irq_reg_set              = vfe_lite88x_bus_irq_reg,
			.global_irq_cmd_offset    = 0x00002230,
			.global_clear_bitmask     = 0x00000001,
		},
	},
	.num_client = 6,
	.bus_client_reg = {
		/* BUS Client 0 RDI0 */
		{
			.cfg                      = 0x00002400,
			.image_addr               = 0x00002404,
			.frame_incr               = 0x00002408,
			.image_cfg_0              = 0x0000240C,
			.image_cfg_1              = 0x00002410,
			.image_cfg_2              = 0x00002414,
			.packer_cfg               = 0x00002418,
			.frame_header_addr        = 0x00002420,
			.frame_header_incr        = 0x00002424,
			.frame_header_cfg         = 0x00002428,
			.line_done_cfg            = 0x0000242C,
			.irq_subsample_period     = 0x00002430,
			.irq_subsample_pattern    = 0x00002434,
			.mmu_prefetch_cfg         = 0x00002460,
			.mmu_prefetch_max_offset  = 0x00002464,
			.system_cache_cfg         = 0x00002468,
			.addr_cfg                 = 0x00002470,
			.addr_status_0            = 0x00002478,
			.addr_status_1            = 0x0000247C,
			.addr_status_2            = 0x00002480,
			.addr_status_3            = 0x00002484,
			.debug_status_cfg         = 0x00002488,
			.debug_status_0           = 0x0000248C,
			.debug_status_1           = 0x00002490,
			.comp_group               = CAM_VFE_BUS_VER3_COMP_GRP_1,
			.ubwc_regs                = NULL,
		},
		/* BUS Client 1 RDI1 */
		{
			.cfg                      = 0x00002500,
			.image_addr               = 0x00002504,
			.frame_incr               = 0x00002508,
			.image_cfg_0              = 0x0000250C,
			.image_cfg_1              = 0x00002510,
			.image_cfg_2              = 0x00002514,
			.packer_cfg               = 0x00002518,
			.frame_header_addr        = 0x00002520,
			.frame_header_incr        = 0x00002524,
			.frame_header_cfg         = 0x00002528,
			.line_done_cfg            = 0x0000252C,
			.irq_subsample_period     = 0x00002530,
			.irq_subsample_pattern    = 0x00002534,
			.mmu_prefetch_cfg         = 0x00002560,
			.mmu_prefetch_max_offset  = 0x00002564,
			.system_cache_cfg         = 0x00002568,
			.addr_cfg                 = 0x00002570,
			.addr_status_0            = 0x00002578,
			.addr_status_1            = 0x0000257C,
			.addr_status_2            = 0x00002580,
			.addr_status_3            = 0x00002584,
			.debug_status_cfg         = 0x00002588,
			.debug_status_0           = 0x0000258C,
			.debug_status_1           = 0x00002590,
			.comp_group               = CAM_VFE_BUS_VER3_COMP_GRP_2,
			.ubwc_regs                = NULL,
		},
		/* BUS Client 2 RDI2 */
		{
			.cfg                      = 0x00002600,
			.image_addr               = 0x00002604,
			.frame_incr               = 0x00002608,
			.image_cfg_0              = 0x0000260C,
			.image_cfg_1              = 0x00002610,
			.image_cfg_2              = 0x00002614,
			.packer_cfg               = 0x00002618,
			.frame_header_addr        = 0x00002620,
			.frame_header_incr        = 0x00002624,
			.frame_header_cfg         = 0x00002628,
			.line_done_cfg            = 0x0000262C,
			.irq_subsample_period     = 0x00002630,
			.irq_subsample_pattern    = 0x00002634,
			.mmu_prefetch_cfg         = 0x00002660,
			.mmu_prefetch_max_offset  = 0x00002664,
			.system_cache_cfg         = 0x00002668,
			.addr_cfg                 = 0x00002670,
			.addr_status_0            = 0x00002678,
			.addr_status_1            = 0x0000267C,
			.addr_status_2            = 0x00002680,
			.addr_status_3            = 0x00002684,
			.debug_status_cfg         = 0x00002688,
			.debug_status_0           = 0x0000268C,
			.debug_status_1           = 0x00002690,
			.comp_group               = CAM_VFE_BUS_VER3_COMP_GRP_3,
			.ubwc_regs                = NULL,
		},
		/* BUS Client 3 RDI3 */
		{
			.cfg                      = 0x00002700,
			.image_addr               = 0x00002704,
			.frame_incr               = 0x00002708,
			.image_cfg_0              = 0x0000270C,
			.image_cfg_1              = 0x00002710,
			.image_cfg_2              = 0x00002714,
			.packer_cfg               = 0x00002718,
			.frame_header_addr        = 0x00002720,
			.frame_header_incr        = 0x00002724,
			.frame_header_cfg         = 0x00002728,
			.line_done_cfg            = 0x0000272C,
			.irq_subsample_period     = 0x00002730,
			.irq_subsample_pattern    = 0x00002734,
			.mmu_prefetch_cfg         = 0x00002760,
			.mmu_prefetch_max_offset  = 0x00002764,
			.system_cache_cfg         = 0x00002768,
			.addr_cfg                 = 0x00002770,
			.addr_status_0            = 0x00002778,
			.addr_status_1            = 0x0000277C,
			.addr_status_2            = 0x00002780,
			.addr_status_3            = 0x00002784,
			.debug_status_cfg         = 0x00002788,
			.debug_status_0           = 0x0000278C,
			.debug_status_1           = 0x00002790,
			.comp_group               = CAM_VFE_BUS_VER3_COMP_GRP_4,
			.ubwc_regs                = NULL,
		},
		/* BUS Client 4 Gamma */
		{
			.cfg                      = 0x00002800,
			.image_addr               = 0x00002804,
			.frame_incr               = 0x00002808,
			.image_cfg_0              = 0x0000280C,
			.image_cfg_1              = 0x00002810,
			.image_cfg_2              = 0x00002814,
			.packer_cfg               = 0x00002818,
			.frame_header_addr        = 0x00002820,
			.frame_header_incr        = 0x00002824,
			.frame_header_cfg         = 0x00002828,
			.line_done_cfg            = 0x0000282C,
			.irq_subsample_period     = 0x00002830,
			.irq_subsample_pattern    = 0x00002834,
			.mmu_prefetch_cfg         = 0x00002860,
			.mmu_prefetch_max_offset  = 0x00002864,
			.system_cache_cfg         = 0x00002868,
			.addr_cfg                 = 0x00002870,
			.addr_status_0            = 0x00002878,
			.addr_status_1            = 0x0000287C,
			.addr_status_2            = 0x00002880,
			.addr_status_3            = 0x00002884,
			.debug_status_cfg         = 0x00002888,
			.debug_status_0           = 0x0000288C,
			.debug_status_1           = 0x00002890,
			.comp_group               = CAM_VFE_BUS_VER3_COMP_GRP_0,
			.ubwc_regs                = NULL,
		},
		/* BUS Client 5 Stats BE */
		{
			.cfg                      = 0x00002900,
			.image_addr               = 0x00002904,
			.frame_incr               = 0x00002908,
			.image_cfg_0              = 0x0000290C,
			.image_cfg_1              = 0x00002910,
			.image_cfg_2              = 0x00002914,
			.packer_cfg               = 0x00002918,
			.frame_header_addr        = 0x00002920,
			.frame_header_incr        = 0x00002924,
			.frame_header_cfg         = 0x00002928,
			.line_done_cfg            = 0x0000292C,
			.irq_subsample_period     = 0x00002930,
			.irq_subsample_pattern    = 0x00002934,
			.mmu_prefetch_cfg         = 0x00002960,
			.mmu_prefetch_max_offset  = 0x00002964,
			.system_cache_cfg         = 0x00002968,
			.addr_cfg                 = 0x00002970,
			.addr_status_0            = 0x00002978,
			.addr_status_1            = 0x0000297C,
			.addr_status_2            = 0x00002980,
			.addr_status_3            = 0x00002984,
			.debug_status_cfg         = 0x00002988,
			.debug_status_0           = 0x0000298C,
			.debug_status_1           = 0x00002990,
			.comp_group               = CAM_VFE_BUS_VER3_COMP_GRP_0,
			.ubwc_regs                = NULL,
		},
	},
	.num_out = 6,
	.vfe_out_hw_info = {
		{
			.vfe_out_type  = CAM_VFE_BUS_VER3_VFE_OUT_RDI0,
			.max_width     = -1,
			.max_height    = -1,
			.source_group  = CAM_VFE_BUS_VER3_SRC_GRP_1,
			.num_wm        = 1,
			.line_based    = 1,
			.mid[0]        = 8,
			.wm_idx        = {
				0,
			},
			.name          = {
				"LITE_0",
			},
		},
		{
			.vfe_out_type  = CAM_VFE_BUS_VER3_VFE_OUT_RDI1,
			.max_width     = -1,
			.max_height    = -1,
			.source_group  = CAM_VFE_BUS_VER3_SRC_GRP_2,
			.num_wm        = 1,
			.line_based    = 1,
			.mid[0]        = 9,
			.wm_idx        = {
				1,
			},
			.name          = {
				"LITE_1",
			},
		},
		{
			.vfe_out_type  = CAM_VFE_BUS_VER3_VFE_OUT_RDI2,
			.max_width     = -1,
			.max_height    = -1,
			.source_group  = CAM_VFE_BUS_VER3_SRC_GRP_3,
			.num_wm        = 1,
			.line_based    = 1,
			.mid[0]        = 10,
			.wm_idx        = {
				2,
			},
			.name          = {
				"LITE_2",
			},
		},
		{
			.vfe_out_type  = CAM_VFE_BUS_VER3_VFE_OUT_RDI3,
			.max_width     = -1,
			.max_height    = -1,
			.source_group  = CAM_VFE_BUS_VER3_SRC_GRP_4,
			.num_wm        = 1,
			.line_based    = 1,
			.mid[0]        = 11,
			.wm_idx        = {
				3,
			},
			.name          = {
				"LITE_3",
			},
		},
		{
			.vfe_out_type  =
				CAM_VFE_BUS_VER3_VFE_OUT_PREPROCESS_RAW,
			.max_width     = 1920,
			.max_height    = 1080,
			.source_group  = CAM_VFE_BUS_VER3_SRC_GRP_0,
			.num_wm        = 1,
			.mid[0]        = 12,
			.wm_idx        = {
				4,
			},
			.name          = {
				"PREPROCESS_RAW",
			},
		},
		{
			.vfe_out_type  = CAM_VFE_BUS_VER3_VFE_OUT_STATS_BG,
			.max_width     = -1,
			.max_height    = -1,
			.source_group  = CAM_VFE_BUS_VER3_SRC_GRP_0,
			.num_wm        = 1,
			.mid[0]        = 13,
			.wm_idx        = {
				5,
			},
			.name          = {
				"STATS_BG",
			},
		},
	},
	.num_comp_grp    = 5,
	.support_consumed_addr = true,
	.comp_done_shift = 0,
	.top_irq_shift   = 0,
	.max_out_res = CAM_ISP_IFE_OUT_RES_BASE + 34,
};

static struct cam_vfe_irq_hw_info vfe_lite88x_irq_hw_info = {
	.reset_mask    = 0,
	.supported_irq = CAM_VFE_HW_IRQ_CAP_LITE_EXT_CSID,
	.top_irq_reg   = &vfe_lite88x_top_irq_reg_info,
};

static struct cam_vfe_hw_info cam_vfe_lite88x_hw_info = {
	.irq_hw_info                   = &vfe_lite88x_irq_hw_info,

	.bus_version                   = CAM_VFE_BUS_VER_3_0,
	.bus_hw_info                   = &vfe_lite88x_bus_hw_info,

	.top_version                   = CAM_VFE_TOP_VER_4_0,
	.top_hw_info                   = &vfe_lite88x_top_hw_info,
};

#endif /* _CAM_VFE_LITE88X_H_ */
