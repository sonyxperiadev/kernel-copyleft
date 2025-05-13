/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_IFE_CSID_860_H_
#define _CAM_IFE_CSID_860_H_

#include <linux/module.h>
#include "cam_ife_csid_dev.h"
#include "camera_main.h"
#include "cam_ife_csid_common.h"
#include "cam_ife_csid_hw_ver2.h"
#include "cam_irq_controller.h"
#include "cam_isp_hw_mgr_intf.h"
#include "cam_ife_csid880.h"

#define CAM_CSID_VERSION_V860                 0x80060000

static struct cam_ife_csid_ver2_reg_info cam_ife_csid_860_reg_info = {
	.top_irq_reg_info      = cam_ife_csid_880_top_irq_reg_info,
	.rx_irq_reg_info       = cam_ife_csid_880_rx_irq_reg_info,
	.path_irq_reg_info     = {
		&cam_ife_csid_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_0],
		&cam_ife_csid_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_1],
		&cam_ife_csid_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_2],
		&cam_ife_csid_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_3],
		&cam_ife_csid_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_RDI_4],
		&cam_ife_csid_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_IPP],
		&cam_ife_csid_880_path_irq_reg_info[CAM_IFE_PIX_PATH_RES_PPP],
	},
	.buf_done_irq_reg_info = &cam_ife_csid_880_buf_done_irq_reg_info,
	.cmn_reg                              = &cam_ife_csid_880_cmn_reg_info,
	.csi2_reg                             = &cam_ife_csid_880_csi2_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_IPP]   = &cam_ife_csid_880_ipp_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_PPP]   = &cam_ife_csid_880_ppp_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_0] = &cam_ife_csid_880_rdi_0_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_1] = &cam_ife_csid_880_rdi_1_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_2] = &cam_ife_csid_880_rdi_2_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_3] = &cam_ife_csid_880_rdi_3_reg_info,
	.path_reg[CAM_IFE_PIX_PATH_RES_RDI_4] = &cam_ife_csid_880_rdi_4_reg_info,
	.top_reg                              = &cam_ife_csid_880_top_reg_info,
	.input_core_sel = {
		{
			0x0,
			0x1,
			0x2,
			0x3,
			-1,
			-1,
			-1,
			-1,
		},
		{
			0x0,
			0x1,
			0x2,
			0x3,
			-1,
			-1,
			-1,
			-1,
		},
		{
			0x0,
			0x1,
			0x2,
			0x3,
			-1,
			-1,
			-1,
			-1,
		},
	},
	.need_top_cfg = 0x1,
	.top_irq_desc       = &cam_ife_csid_880_top_irq_desc,
	.rx_irq_desc        = &cam_ife_csid_880_rx_irq_desc,
	.path_irq_desc      = cam_ife_csid_880_path_irq_desc,
	.num_top_err_irqs   = cam_ife_csid_880_num_top_irq_desc,
	.num_rx_err_irqs    = cam_ife_csid_880_num_rx_irq_desc,
	.num_path_err_irqs  = ARRAY_SIZE(cam_ife_csid_880_path_irq_desc),
	.num_top_regs       = 1,
	.num_rx_regs        = 1,
};

#endif /*_CAM_IFE_CSID_860_H_ */
