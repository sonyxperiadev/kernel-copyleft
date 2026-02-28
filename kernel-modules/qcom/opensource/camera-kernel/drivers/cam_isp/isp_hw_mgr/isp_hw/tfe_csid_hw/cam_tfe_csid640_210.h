/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_TFE_CSID_640_210_H_
#define _CAM_TFE_CSID_640_210_H_

#include "cam_tfe_csid_core.h"
#include "cam_tfe_csid640.h"

#define CAM_TFE_CSID_VERSION_V640_210               0x60040000

static struct cam_tfe_csid_common_reg_offset
	cam_tfe_csid_640_210_cmn_reg_offset = {
	.csid_hw_version_addr                         = 0x0,
	.csid_cfg0_addr                               = 0x4,
	.csid_ctrl_addr                               = 0x8,
	.csid_rst_strobes_addr                        = 0x10,

	.csid_test_bus_ctrl_addr                      = 0x14,
	.csid_top_irq_status_addr                     = 0x70,
	.csid_top_irq_mask_addr                       = 0x74,
	.csid_top_irq_clear_addr                      = 0x78,
	.csid_top_irq_set_addr                        = 0x7c,
	.csid_irq_cmd_addr                            = 0x80,

	/*configurations */
	.major_version                                = 5,
	.minor_version                                = 3,
	.version_incr                                 = 0,
	.num_rdis                                     = 3,
	.num_pix                                      = 1,
	.csid_reg_rst_stb                             = 1,
	.csid_rst_stb                                 = 0x1e,
	.csid_rst_stb_sw_all                          = 0x1f,
	.ipp_path_rst_stb_all                         = 0x17,
	.rdi_path_rst_stb_all                         = 0x97,
	.path_rst_done_shift_val                      = 1,
	.path_en_shift_val                            = 31,
	.dt_id_shift_val                              = 27,
	.vc_shift_val                                 = 22,
	.dt_shift_val                                 = 16,
	.vc1_shift_val                                = 2,
	.dt1_shift_val                                = 7,
	.multi_vc_dt_en_shift_val                     = 0,
	.fmt_shift_val                                = 12,
	.plain_fmt_shit_val                           = 10,
	.crop_v_en_shift_val                          = 6,
	.crop_h_en_shift_val                          = 5,
	.crop_shift                                   = 16,
	.ipp_irq_mask_all                             = 0x3FFFF,
	.rdi_irq_mask_all                             = 0x3FFFF,
	.top_tfe2_pix_pipe_fuse_reg                   = 0xFE4,
	.top_tfe2_fuse_reg                            = 0xFE8,
	.format_measure_support                       = true,
	.format_measure_height_shift_val              = 16,
	.format_measure_height_mask_val               = 0xe,
	.format_measure_width_mask_val                = 0x10,
	.sync_clk                                     = true,
	.tfe_pix_fuse_en                              = false,
	.disable_pix_tfe_idx                          = -1,
};

static struct cam_tfe_csid_reg_offset cam_tfe_csid_640_210_reg_offset = {
	.cmn_reg          = &cam_tfe_csid_640_210_cmn_reg_offset,
	.csi2_reg         = &cam_tfe_csid_640_csi2_reg_offset,
	.ipp_reg          = &cam_tfe_csid_640_ipp_reg_offset,
	.rdi_reg = {
		&cam_tfe_csid_640_rdi_0_reg_offset,
		&cam_tfe_csid_640_rdi_1_reg_offset,
		&cam_tfe_csid_640_rdi_2_reg_offset,
		},
};

static struct cam_tfe_csid_hw_info cam_tfe_csid640_210_hw_info = {
	.csid_reg = &cam_tfe_csid_640_210_reg_offset,
	.hw_dts_version = CAM_TFE_CSID_VERSION_V640_210,
};

#endif /*_CAM_TFE_CSID_640_210_H_ */
