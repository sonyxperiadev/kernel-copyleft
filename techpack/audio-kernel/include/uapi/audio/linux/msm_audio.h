/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2012, 2014, 2017, 2020, 2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _UAPI_LINUX_MSM_AUDIO_H
#define _UAPI_LINUX_MSM_AUDIO_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define AUDIO_IOCTL_MAGIC 'a'

#define IOCTL_MAP_PHYS_ADDR _IOW(AUDIO_IOCTL_MAGIC, 97, int)
#define IOCTL_UNMAP_PHYS_ADDR _IOW(AUDIO_IOCTL_MAGIC, 98, int)

#define IOCTL_MAP_HYP_ASSIGN _IOW(AUDIO_IOCTL_MAGIC, 109, int)
#define IOCTL_UNMAP_HYP_ASSIGN _IOW(AUDIO_IOCTL_MAGIC, 110, int)
#define IOCTL_MAP_HYP_ASSIGN_V2 _IOW(AUDIO_IOCTL_MAGIC, 111, struct msm_mdf_data)
#define IOCTL_UNMAP_HYP_ASSIGN_V2 _IOW(AUDIO_IOCTL_MAGIC, 112, struct msm_mdf_data)

#define AUDIO_MAX_COMMON_IOCTL_NUM 113

/* ss_masks is generated from sys_ids
 * for id in sys_ids
 * ss_masks |= 1 << (id - 1)
 */
struct msm_mdf_data {
	__u64 ss_masks;
	__u32 mem_fd;
	__u32 reserved;
};

#endif
