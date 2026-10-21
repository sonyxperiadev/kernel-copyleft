/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef CAM_ICP_SOC_COMMON_H
#define CAM_ICP_SOC_COMMON_H

#include "cam_soc_util.h"
#include "cam_icp_hw_intf.h"

#define ICP_UBWC_CFG_MAX     2
#define CAM_ICP_V1_VERSION   0x0100
#define CAM_ICP_V2_VERSION   0x0200
#define CAM_ICP_V2_1_VERSION 0x0201

/**
 * struct cam_icp_ubwc_cfg - ICP ubwc cfg
 * @ipe_fetch: UBWC configuration for IPE fetch.
 * @ipe_write: UBWC configuration for IPE write.
 * @bps_fetch: UBWC configuration for BPS fetch.
 * @bps_write: UBWC configuration for BPS write.
 */
struct cam_icp_ubwc_cfg {
	uint32_t ipe_fetch[ICP_UBWC_CFG_MAX];
	uint32_t ipe_write[ICP_UBWC_CFG_MAX];
	uint32_t bps_fetch[ICP_UBWC_CFG_MAX];
	uint32_t bps_write[ICP_UBWC_CFG_MAX];
};

/**
 * struct cam_icp_soc_info - ICP soc info
 * @dev_type: Device type.
 * @qos_val: QOS value.
 * @hw_version: hw version.
 * @uconfig: union of ubwc_cfg_ext and ubwc_cfg. ubwc_cfg may
 *           be used in icp_v1 for older chipsets. icp_v2 only
 *           uses ubwc_cfg_ext.
 * @is_ubwc_cfg: indicate if ubwc_cfg is used.
 */
struct cam_icp_soc_info {
	enum cam_icp_hw_type dev_type;
	uint32_t qos_val;
	uint32_t hw_version;
	union {
		uint32_t ubwc_cfg[ICP_UBWC_CFG_MAX];
		struct cam_icp_ubwc_cfg ubwc_cfg_ext;
	} uconfig;
	bool is_ubwc_cfg;
};

int cam_icp_soc_resources_init(struct cam_hw_soc_info *soc_info,
	irq_handler_t irq_handler, void *irq_data);

int cam_icp_soc_resources_deinit(struct cam_hw_soc_info *soc_info);

int cam_icp_soc_resources_enable(struct cam_hw_soc_info *soc_info);

int cam_icp_soc_resources_disable(struct cam_hw_soc_info *soc_info);

int cam_icp_soc_update_clk_rate(struct cam_hw_soc_info *soc_info,
	int32_t clk_level);

#endif
