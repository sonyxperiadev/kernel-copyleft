/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __QTI_SMMU_PROXY_H_
#define __QTI_SMMU_PROXY_H_

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <linux/align.h>

#include <smmu-proxy/include/uapi/linux/qti-smmu-proxy.h>

#define SMMU_PROXY_MEM_ALIGNMENT (1 << 21)

int smmu_proxy_get_csf_version(struct csf_version *csf_version);

#endif /* __QTI_SMMU_PROXY_H_ */
