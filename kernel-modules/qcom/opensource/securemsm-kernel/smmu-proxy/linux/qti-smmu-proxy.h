/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __QTI_SMMU_PROXY_H_
#define __QTI_SMMU_PROXY_H_

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <linux/align.h>

#include <smmu-proxy/include/uapi/linux/qti-smmu-proxy.h>

#define SMMU_PROXY_MEM_ALIGNMENT (1 << 21)
#define SMMU_PROXY_SWITCH_OP_ACQUIRE_SID 0
#define SMMU_PROXY_SWITCH_OP_RELEASE_SID 1

int smmu_proxy_get_csf_version(struct csf_version *csf_version);

/*
 * Decouple the unmap call from the SID switch, to allow the SID switch
 * to happen more deterministically compared to the lazy unmap call which
 * delays the SID switch.
 */
int smmu_proxy_switch_sid(struct device *client_dev, u32 op);

#endif /* __QTI_SMMU_PROXY_H_ */
