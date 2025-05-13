/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef SMMU_PROXY_MSGQ_H
#define SMMU_PROXY_MSGQ_H

#include <linux/gunyah/gh_rm_drv.h>


/**
 * enum smmu_proxy_msg_type: Message types used by the SMMU proxy driver for
 * communication.
 * @SMMU_PROXY_MAP: The message is a request to map memory into the VM's
 * SMMU.
 * @SMMU_PROXY_MAP_RESP: The message is a response from a remote VM to a
 * mapping request issued by the receiving VM
 * @SMMU_PROXY_UNMAP: The message is a request to unmap some previously
 * SMMU-mapped memory from the VM
 * @SMMU_PROXY_UNMAP_RESP: The message is a response from a remote VM to an
 * unmapping request issued by the receiving VM
 * @SMMU_PROXY_ERR_RESP: The message is a response from a remote VM to give
 * a generic error response for a prior message sent to the remote VM
 */
enum smmu_proxy_msg_type {
	SMMU_PROXY_MAP,
	SMMU_PROXY_MAP_RESP,
	SMMU_PROXY_UNMAP,
	SMMU_PROXY_UNMAP_RESP,
	SMMU_PROXY_ERR_RESP,
	SMMU_PROXY_MSG_MAX,
};

/**
 * struct smmu_proxy_msg_hdr: The header for SMMU proxy messages
 * @msg_type: The type of message.
 * @msg_size: The size of message.
 */
struct smmu_proxy_msg_hdr {
	u32 msg_type;
	u32 msg_size;
} __packed;

/**
 * struct smmu_proxy_msg_hdr: The header for responses to SMMU proxy messages
 * @msg_type: The type of message.
 * @msg_size: The size of message.
 * @ret: Return code from remote VM
 */
struct smmu_proxy_resp_hdr {
	u32 msg_type;
	u32 msg_size;
	s32 ret;
} __packed;

/**
 * struct smmu_proxy_map_req: The message format for an SMMU mapping request from
 * another VM.
 * @hdr: Message header
 * @hdl: The memparcel handle associated with the memory to be mapped in the SMMU
 * of the relevant VM
 * @cb_id: Context bank ID that we will map the memory associated with @hdl to
 * @acl_desc: A GH ACL descriptor that describes the VMIDs that will be
 * accessing the memory, as well as what permissions each VMID will have.
 */
struct smmu_proxy_map_req {
	struct smmu_proxy_msg_hdr hdr;
	u32 hdl;
	u32 cb_id;
	struct gh_acl_desc acl_desc;
} __packed;

/**
 * struct smmu_proxy_map_resp: The message format for an SMMU mapping
 * request response.
 * @hdr: Response header
 * @iova: IOVA of mapped memory
 * @mapping_len: Lenth of IOMMU IOVA mapping
 */
struct smmu_proxy_map_resp {
	struct smmu_proxy_resp_hdr hdr;
	u64 iova;
	u64 mapping_len;
} __packed;

/**
 * struct smmu_proxy_unmap_req: The message format for an SMMU unmapping request from
 * another VM.
 * @hdr: Message header
 * @hdl: The memparcel handle associated with the memory to be mapped in the SMMU
 * of the relevant VM
 */
struct smmu_proxy_unmap_req {
	struct smmu_proxy_msg_hdr hdr;
	u32 hdl;
} __packed;

/**
 * struct smmu_proxy_unmap_resp: The message format for an SMMU unmapping
 * request response.
 * @hdr: Response header
 */
struct smmu_proxy_unmap_resp {
	struct smmu_proxy_resp_hdr hdr;
} __packed;

#endif /* SMMU_PROXY_MSGQ_H */
