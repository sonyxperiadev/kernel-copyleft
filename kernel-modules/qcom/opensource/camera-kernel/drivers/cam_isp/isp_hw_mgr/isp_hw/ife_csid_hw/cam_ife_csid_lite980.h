/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_IFE_CSID_LITE_980_H_
#define _CAM_IFE_CSID_LITE_980_H_

#include "cam_ife_csid_lite880.h"

static struct cam_ife_csid_ver2_reg_info cam_ife_csid_lite_980_reg_info = {
	.top_irq_reg_info      = &cam_ife_csid_lite_880_top_irq_reg_info,
	.rx_irq_reg_info       = &cam_ife_csid_lite_880_rx_irq_reg_info,
	.path_irq_reg_info     = {
		&cam_ife_csid_lite_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_0],
		&cam_ife_csid_lite_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_1],
		&cam_ife_csid_lite_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_2],
		&cam_ife_csid_lite_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_3],
		NULL,
		&cam_ife_csid_lite_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_IPP],
		},
	.buf_done_irq_reg_info = &cam_ife_csid_lite_880_buf_done_irq_reg_info,
	.cmn_reg               = &cam_ife_csid_lite_880_cmn_reg_info,
	.csi2_reg              = &cam_ife_csid_lite_880_csi2_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_IPP]   = &cam_ife_csid_lite_880_ipp_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_PPP]   = NULL,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_0] = &cam_ife_csid_lite_880_rdi_0_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_1] = &cam_ife_csid_lite_880_rdi_1_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_2] = &cam_ife_csid_lite_880_rdi_2_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_3] = &cam_ife_csid_lite_880_rdi_3_reg_info,
	.need_top_cfg = 0,
	.top_irq_desc       = &cam_ife_csid_lite_880_top_irq_desc,
	.rx_irq_desc        = &cam_ife_csid_lite_880_rx_irq_desc,
	.path_irq_desc      = cam_ife_csid_lite_880_path_irq_desc,
	.num_top_err_irqs   = cam_ife_csid_lite_880_num_top_irq_desc,
	.num_rx_err_irqs    = cam_ife_csid_lite_880_num_rx_irq_desc,
	.num_path_err_irqs  = ARRAY_SIZE(cam_ife_csid_lite_880_path_irq_desc),
	.num_top_regs       = 1,
	.num_rx_regs        = 1,
};

#endif /* _CAM_IFE_CSID_LITE_980_H_ */
