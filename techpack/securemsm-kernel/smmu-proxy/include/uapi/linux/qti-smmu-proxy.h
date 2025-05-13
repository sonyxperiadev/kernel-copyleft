/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __QTI_SMMU_PROXY_UAPI_H_
#define __QTI_SMMU_PROXY_UAPI_H_

#include <linux/types.h>

#define QTI_SMMU_PROXY_CAMERA_CB 0
#define QTI_SMMU_PROXY_DISPLAY_CB 1
#define QTI_SMMU_PROXY_EVA_CB 2

#define QTI_SMMU_PROXY_IOC_BASE 0x55

struct csf_version {
	__u32 arch_ver;
	__u32 max_ver;
	__u32 min_ver;
	__u32 padding;
};

#define QTI_SMMU_PROXY_GET_VERSION_IOCTL	_IOR(QTI_SMMU_PROXY_IOC_BASE, 0, \
						     struct csf_version)

struct smmu_proxy_acl_ctl {
	__u32 dma_buf_fd;
	__u32 padding;
};

#define QTI_SMMU_PROXY_AC_LOCK_BUFFER	_IOW(QTI_SMMU_PROXY_IOC_BASE, 1, \
					     struct smmu_proxy_acl_ctl)
#define QTI_SMMU_PROXY_AC_UNLOCK_BUFFER	_IOW(QTI_SMMU_PROXY_IOC_BASE, 2, \
					     struct smmu_proxy_acl_ctl)

struct smmu_proxy_wipe_buf_ctl {
	__u64 context_bank_id_array;
	__u32 num_cb_ids;
	__u32 padding;
};

#define QTI_SMMU_PROXY_WIPE_BUFFERS	_IOW(QTI_SMMU_PROXY_IOC_BASE, 3, \
					     struct smmu_proxy_wipe_buf_ctl)

struct smmu_proxy_get_dma_buf_ctl {
	/*
	 * memparcel_hdl only needs to be 32-bit for Gunyah, but a 64-bit value
	 * is needed to remain forward compatible with FF-A .
	 */
	__u64 memparcel_hdl;
	__u32 dma_buf_fd;
	__u32 padding;
};

#define QTI_SMMU_PROXY_GET_DMA_BUF	_IOWR(QTI_SMMU_PROXY_IOC_BASE, 4, \
					      struct smmu_proxy_get_dma_buf_ctl)

#endif /* __QTI_SMMU_PROXY_UAPI_H_ */
