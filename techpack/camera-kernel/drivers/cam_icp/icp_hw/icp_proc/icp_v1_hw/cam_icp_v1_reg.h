/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_ICP_V1_REG_H_
#define _CAM_ICP_V1_REG_H_

#define ICP_V1_CSR_NSEC_RESET           0x4
#define ICP_V1_GEN_PURPOSE_REG_OFFSET   0x40
#define ICP_V1_CSR_FUNC_RESET           (1 << 4)
#define ICP_V1_CSR_DBG_RESET            (1 << 3)
#define ICP_V1_CSR_CPU_RESET            (1 << 2)

#define ICP_V1_CSR_CONTROL              0x8
#define ICP_V1_CSR_DBGSWENABLE          (1 << 22)
#define ICP_V1_CSR_EDBGRQ               (1 << 14)
#define ICP_V1_CSR_EN_CLKGATE_WFI       (1 << 12)
#define ICP_V1_CSR_CPU_EN               (1 << 9)
#define ICP_V1_CSR_WAKE_UP_EN           (1 << 4)

#define ICP_V1_CSR_FULL_DBG_EN          (ICP_V1_CSR_DBGSWENABLE | ICP_V1_CSR_EDBGRQ)
#define ICP_V1_CSR_FULL_CPU_EN          (ICP_V1_CSR_CPU_EN | \
					ICP_V1_CSR_WAKE_UP_EN | \
					ICP_V1_CSR_EN_CLKGATE_WFI)

#define ICP_V1_CSR_A2HOSTINTEN          0x10
#define ICP_V1_WDT_WS1EN                (1 << 2)
#define ICP_V1_WDT_WS0EN                (1 << 1)
#define ICP_V1_A2HOSTINTEN              (1 << 0)

#define ICP_V1_CSR_HOST2ICPINT          0x30
#define ICP_V1_HOSTINT                  (1 << 0)
#define ICP_V1_HOST_INT_CLR             0x18
#define ICP_V1_HOST_INT_STATUS          0x1c
#define ICP_V1_HOST_INT_SET             0x20

#define ICP_V1_CSR_STATUS               0x200
#define ICP_V1_CSR_STANDBYWFI           (1 << 7)

#define ICP_V1_INIT_REQ                 0x48
#define ICP_V1_INIT_RESPONSE            0x4c
#define ICP_V1_SHARED_MEM_PTR           0x50
#define ICP_V1_SHARED_MEM_SIZE          0x54
#define ICP_V1_QTBL_PTR                 0x58
#define ICP_V1_UNCACHED_HEAP_PTR        0x5c
#define ICP_V1_UNCACHED_HEAP_SIZE       0x60

#endif /* _CAM_ICP_V1_REG_H_ */
