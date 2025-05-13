/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_icp_v1_core.h"
#include "cam_icp_v2_core.h"
#include "hfi_intf.h"

#define CAM_ICP_GET_PROC_DEV_INTF(devices)                              \
	(devices[CAM_ICP_HW_ICP_V1] ? devices[CAM_ICP_HW_ICP_V1][0] : \
	devices[CAM_ICP_HW_ICP_V2][0])

/**
 * @brief : Get ICP device type (ICP_V1/ICP_V2/...)
 */
int cam_icp_alloc_processor_devs(struct device_node *np, enum cam_icp_hw_type *icp_hw_type,
	struct cam_hw_intf ***devices, uint32_t *hw_dev_cnt);

/**
 * @brief : Get device operations per device type
 */
int cam_icp_get_hfi_device_ops(uint32_t hw_type, const struct hfi_ops **hif_ops);

/**
 * @brief : Get number of icp_v2 hw instances
 */
uint32_t cam_icp_v2_get_device_num(void);

/**
 * @brief : Get number of icp_v1 hw instances
 */
uint32_t cam_icp_v1_get_device_num(void);

