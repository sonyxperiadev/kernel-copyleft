/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_ICP_V2_REG_H_
#define _CAM_ICP_V2_REG_H_

struct cam_icp_v2_hw_info {
	uint32_t ob_irq_status;
	uint32_t ob_irq_mask;
	uint32_t ob_irq_clear;
	uint32_t ob_irq_set;
	uint32_t ob_irq_cmd;
	uint32_t host2icpint;
	uint32_t pfault_info;
};

/* ICP CSR info */
#define ICP_V2_GEN_PURPOSE_REG_OFFSET     0x20
#define ICP_V2_CSR_DBG_STATUS_REG_OFFSET  0xC0
#define ICP_V2_CSR_DBG_CTRL_REG_OFFSET    0xC4
#define ICP_V2_CSR_GP_REG_COUNT           0x18

/* ICP_SYS - Protected reg space defined in AC policy */
#define ICP_V2_SYS_RESET      0x0
#define ICP_V2_SYS_CONTROL    0x4
#define ICP_V2_SYS_STATUS     0xC
#define ICP_V2_SYS_ACCESS     0x10

#define ICP_V2_STANDBYWFI     (1 << 7)
#define ICP_V2_EN_CPU         (1 << 9)
#define ICP_V2_FUNC_RESET     (1 << 4)

/* ICP WD reg space */
#define ICP_V2_WD_CTRL        0x8
#define ICP_V2_WD_INTCLR      0xC

/* ICP DOM_MASK reg space */
#define ICP_V2_DOM_0_CFG_OFFSET    0x0
#define ICP_V2_DOM_1_CFG_OFFSET    0x20

/* These bitfields are shared by OB_MASK, OB_CLEAR, OB_STATUS */
#define ICP_V2_WDT_BITE_WS1       (1 << 6)
#define ICP_V2_WDT_BARK_WS1       (1 << 5)
#define ICP_V2_WDT_BITE_WS0       (1 << 4)
#define ICP_V2_WDT_BARK_WS0       (1 << 3)
#define ICP_V2_ICP2HOSTINT        (1 << 2)

#define ICP_V2_IRQ_CLEAR_CMD      (1 << 1)
#define ICP_V2_IRQ_SET_CMD        (1 << 0)

#define ICP_V2_HOST2ICPINT        (1 << 0)

#endif /* _CAM_ICP_V2_REG_H_ */
