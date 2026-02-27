/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_ICP_V1_DEV_H_
#define _CAM_ICP_V1_DEV_H_

/**
 * @brief : API to register icp_v1 hw to platform framework.
 * @return struct platform_device pointer on success, or ERR_PTR() on error.
 */
int cam_icp_v1_init_module(void);

/**
 * @brief : API to remove icp_v1 hw from platform framework.
 */
void cam_icp_v1_exit_module(void);

#endif /* _CAM_ICP_V1_DEV_H_ */
