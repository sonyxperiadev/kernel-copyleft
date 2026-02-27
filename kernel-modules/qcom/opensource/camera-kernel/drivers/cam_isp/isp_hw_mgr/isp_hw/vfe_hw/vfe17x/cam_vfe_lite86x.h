/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#ifndef _CAM_VFE_LITE86X_H_
#define _CAM_VFE_LITE86X_H_
#include "cam_vfe_camif_ver3.h"
#include "cam_vfe_top_ver4.h"
#include "cam_vfe_core.h"
#include "cam_vfe_bus_ver3.h"
#include "cam_irq_controller.h"
#include "cam_vfe_lite78x.h"
#include "cam_vfe_lite88x.h"

static struct cam_vfe_top_ver4_reg_offset_common vfe_lite86x_top_common_reg = {
	.hw_version               = 0x00001000,
	.hw_capability            = 0x00001004,
	.core_cgc_ovd_0           = 0x00001014,
	.ahb_cgc_ovd              = 0x00001018,
	.core_cfg_0               = 0x0000103C,
	.diag_config              = 0x00001040,
	.diag_sensor_status_0     = 0x00001044,
	.diag_sensor_status_1     = 0x00001048,
	.ipp_violation_status     = 0x00001054,
	.bus_violation_status     = 0x00001264,
	.bus_overflow_status      = 0x00001268,
	.top_debug_cfg            = 0x00001074,
	.num_top_debug_reg        = CAM_VFE_88X_NUM_DBG_REG,
	.top_debug                = vfe_lite88x_top_debug_reg,
	.frame_timing_irq_reg_idx = CAM_IFE_IRQ_CAMIF_REG_STATUS1,
};

static struct cam_vfe_ver4_path_hw_info
	vfe_lite86x_rdi_hw_info[] = {
	{
		.common_reg     = &vfe_lite86x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[0],
	},
	{
		.common_reg     = &vfe_lite86x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[1],
	},
	{
		.common_reg     = &vfe_lite86x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[2],
	},
	{
		.common_reg     = &vfe_lite86x_top_common_reg,
		.reg_data       = &vfe_lite88x_rdi_reg_data[3],
	},
};

static struct cam_vfe_top_ver4_hw_info vfe_lite86x_top_hw_info = {
	.common_reg = &vfe_lite86x_top_common_reg,
	.rdi_hw_info = vfe_lite86x_rdi_hw_info,
	.vfe_full_hw_info = {
		.common_reg     = &vfe_lite86x_top_common_reg,
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
	.num_rdi        = ARRAY_SIZE(vfe_lite88x_rdi_hw_info),
};

static struct cam_vfe_hw_info cam_vfe_lite86x_hw_info = {
	.irq_hw_info                   = &vfe_lite88x_irq_hw_info,

	.bus_version                   = CAM_VFE_BUS_VER_3_0,
	.bus_hw_info                   = &vfe_lite88x_bus_hw_info,

	.top_version                   = CAM_VFE_TOP_VER_4_0,
	.top_hw_info                   = &vfe_lite86x_top_hw_info,
};

#endif /* _CAM_VFE_LITE86X_H_ */
