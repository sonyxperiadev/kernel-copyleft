/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _DRIVERS_QCEDEV_SMMU_H_
#define _DRIVERS_QCEDEV_SMMU_H_

#include <linux/dma-buf.h>
#include <linux/types.h>
#include "qcedev_fe.h"

struct msm_gpce_smmu_vm_map_cmd {
	int cmd_id;
	u32 export_id;
	u32 buf_size;
	void *mem_handle;
	u32 is_secure;
} __packed;

struct msm_gpce_smmu_vm_map_cmd_rsp {
	int status;
	u64 addr;
} __packed;

struct msm_gpce_smmu_vm_unmap_cmd {
	int cmd_id;
	u32 export_id;
	void *unused; // for aligement with the host
	u32 is_secure;
} __packed;

struct msm_gpce_smmu_vm_unmap_cmd_rsp {
	int status;
} __packed;

enum gpce_cmd_id {
	MSM_GPCE_SMMU_VM_CMD_MAP,
	MSM_GPCE_SMMU_VM_CMD_UNMAP,
};

int qcedev_check_and_map_buffer(void *qce_hndl,
		int fd, unsigned int offset, unsigned int fd_size,
		unsigned long long *vaddr, struct qce_fe_drv_hab_handles *drv_handles);
int qcedev_check_and_unmap_buffer(void *handle, int fd, struct qce_fe_drv_hab_handles *drv_handles);
int qcedev_unmap_all_buffers(void *handle, struct qce_fe_drv_hab_handles *drv_handles);

#endif //_DRIVERS_QCEDEV_SMMU_H_

