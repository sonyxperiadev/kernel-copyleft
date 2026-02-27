/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __QTI_SMMU_PROXY_COMMON_H_
#define __QTI_SMMU_PROXY_COMMON_H_

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/dma-buf.h>

#include <linux/mem-buf.h>
#include <soc/qcom/secure_buffer.h>
#include <linux/gunyah/gh_msgq.h>
#include "qti-smmu-proxy-msgq.h"
#include "linux/qti-smmu-proxy.h"

union smmu_proxy_ioctl_arg {
	struct csf_version csf_version;
	struct smmu_proxy_acl_ctl acl_ctl;
	struct smmu_proxy_wipe_buf_ctl wipe_buf_ctl;
	struct smmu_proxy_get_dma_buf_ctl get_dma_buf_ctl;
};

int smmu_proxy_create_dev(const struct file_operations *fops);

#endif /* __QTI_SMMU_PROXY_COMMON_H_ */
