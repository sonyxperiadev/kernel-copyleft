/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef CAM_ICP_HW_MGR_INTF_H
#define CAM_ICP_HW_MGR_INTF_H

#include <linux/of.h>
#include <media/cam_icp.h>
#include <media/cam_defs.h>
#include "cam_cpas_api.h"


#define CAM_ICP_SUBDEV_NAME_LEN  32
#define CAM_ICP_SUBDEV_NAME      "cam-icp"

#define ICP_CLK_TURBO_HZ         600000000
#define ICP_CLK_SVS_HZ           400000000

#define CAM_ICP_BW_BYTES_VOTE    40000000

#define CAM_ICP_CTX_MAX          54

#define CPAS_IPE1_BIT            0x2000

#define CAM_IPE_DEFAULT_AXI_PATH    CAM_AXI_PATH_DATA_IPE_WR_VID
#define CAM_IPE_DEFAULT_AXI_TRANSAC CAM_AXI_TRANSACTION_WRITE
#define CAM_BPS_DEFAULT_AXI_PATH    CAM_AXI_PATH_DATA_ALL
#define CAM_BPS_DEFAULT_AXI_TRANSAC CAM_AXI_TRANSACTION_WRITE
#define CAM_OFE_DEFAULT_AXI_PATH    CAM_AXI_PATH_DATA_OFE_WR_VID
#define CAM_OFE_DEFAULT_AXI_TRANSAC CAM_AXI_TRANSACTION_WRITE
#define CAM_ICP_DEFAULT_AXI_PATH    CAM_AXI_PATH_DATA_ALL
#define CAM_ICP_DEFAULT_AXI_TRANSAC CAM_AXI_TRANSACTION_READ

#define CAM_ICP_DUMP_TAG_MAX_LEN 128
#define CAM_ICP_DUMP_NUM_WORDS   5

enum cam_icp_subdev_id {
	CAM_ICP0_SUBDEV,
	CAM_ICP1_SUBDEV,
	CAM_ICP_SUBDEV_MAX
};

enum cam_icp_hw_event_type {
	CAM_ICP_EVT_ID_BUF_DONE,
	CAM_ICP_EVT_ID_ERROR,
	CAM_ICP_EVT_ID_INJECT_EVENT,
};

enum cam_icp_hw_error_type {
	CAM_ICP_HW_ERROR_NO_MEM,
	CAM_ICP_HW_ERROR_SYSTEM_FAILURE,
};

typedef void(*cam_icp_mini_dump_cb)(void *priv,
	void *args);

/**
 * struct cam_icp_cpas_vote
 * @ahb_vote: AHB vote info
 * @axi_vote: AXI vote info
 * @ahb_vote_valid: Flag for ahb vote data
 * @axi_vote_valid: flag for axi vote data
 */
struct cam_icp_cpas_vote {
	struct cam_ahb_vote ahb_vote;
	struct cam_axi_vote axi_vote;
	uint32_t ahb_vote_valid;
	uint32_t axi_vote_valid;
};

/**
 * struct cam_icp_hw_dump_args
 * @cpu_addr: kernel vaddr
 * @buf_len:  buffer length
 * @offset:   offset
 */
struct cam_icp_hw_dump_args {
	uintptr_t  cpu_addr;
	size_t     buf_len;
	size_t     offset;
};

/**
 * struct cam_icp_dump_header
 * @tag:        tag of the packet
 * @size:       size of data in packet
 * @word_size:  size of each word in packet
 */
struct cam_icp_dump_header {
	uint8_t    tag[CAM_ICP_DUMP_TAG_MAX_LEN];
	uint64_t   size;
	int32_t    word_size;
};

/**
 * struct cam_icp_hw_buf_done_evt_data
 * @buf_done_data:    buf done payload
 * @evt_id:           event id (success/cancel/error)
 */
struct cam_icp_hw_buf_done_evt_data {
	void      *buf_done_data;
	uint32_t   evt_id;
};

/**
 * struct cam_icp_hw_error_evt_data
 * @req_id:    request id
 * @err_type:      type of fatal error
 */
struct cam_icp_hw_error_evt_data {
	uint64_t                       req_id;
	enum cam_icp_hw_error_type     err_type;
};

int cam_icp_hw_mgr_init(struct device_node *of_node,
	uint64_t *hw_mgr_hdl, int *iommu_hdl,
	cam_icp_mini_dump_cb mini_dump_cb, int device_idx);

void cam_icp_hw_mgr_deinit(int device_idx);

#endif /* CAM_ICP_HW_MGR_INTF_H */
