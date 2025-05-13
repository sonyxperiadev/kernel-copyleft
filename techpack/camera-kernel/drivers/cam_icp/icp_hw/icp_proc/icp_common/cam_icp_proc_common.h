/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_ICP_UTILS_H_
#define _CAM_ICP_UTILS_H_

#include <linux/firmware.h>
#include <linux/elf.h>
#include <linux/iopoll.h>

#include "cam_debug_util.h"
#include "cam_cpas_api.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_icp_soc_common.h"

#define ICP_FW_NAME_MAX_SIZE    32
#define PC_POLL_DELAY_US        100
#define PC_POLL_TIMEOUT_US      10000

/**
 * struct cam_icp_proc_ubwc_cfg_cmd
 * @ubwc_cfg: UBWC cfg info parsed from DT
 * @ubwc_cfg_dev_mask: mask to indicate which device
 *                     ubwc cfg to send to fw
 */
struct cam_icp_proc_ubwc_cfg_cmd {
	struct cam_icp_ubwc_cfg *ubwc_cfg;
	uint32_t ubwc_cfg_dev_mask;
};

/**
 * @brief : Validate FW elf image
 */
int32_t cam_icp_validate_fw(const uint8_t *elf, uint32_t machine_id);

/**
 * @brief : Get FW elf size
 */
int32_t cam_icp_get_fw_size(const uint8_t *elf, uint32_t *fw_size);

/**
 * @brief : Program FW memory
 */
int32_t cam_icp_program_fw(const uint8_t *elf,
	uintptr_t fw_kva_addr);

/**
 * @brief : Update ahb and axi votes
 */
int cam_icp_proc_cpas_vote(uint32_t cpas_handle,
	struct cam_icp_cpas_vote *vote);

/**
 * @brief : dump FW memory into mini dump
 */
int cam_icp_proc_mini_dump(struct cam_icp_hw_dump_args *args,
	uintptr_t fw_kva_addr, uint64_t fw_buf_len);

/**
 * @brief : Update UBWC configuration for IPE and BPS
 */
int cam_icp_proc_ubwc_configure(struct cam_icp_proc_ubwc_cfg_cmd *ubwc_cfg_cmd,
	bool force_disable_ubwc, int hfi_handle);

#endif /* _CAM_ICP_UTILS_H_ */
