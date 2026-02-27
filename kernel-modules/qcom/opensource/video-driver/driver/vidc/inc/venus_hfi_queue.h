/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _VENUS_HFI_QUEUE_H_
#define _VENUS_HFI_QUEUE_H_

#include <linux/types.h>

#include "msm_vidc_internal.h"

#define HFI_MASK_QHDR_TX_TYPE			0xff000000
#define HFI_MASK_QHDR_RX_TYPE			0x00ff0000
#define HFI_MASK_QHDR_PRI_TYPE			0x0000ff00
#define HFI_MASK_QHDR_Q_ID_TYPE			0x000000ff
#define HFI_Q_ID_HOST_TO_CTRL_CMD_Q		0
#define HFI_Q_ID_CTRL_TO_HOST_MSG_Q		1
#define HFI_Q_ID_CTRL_TO_HOST_DEBUG_Q		2
#define HFI_MASK_QHDR_STATUS			0x000000ff

#define VIDC_IFACEQ_NUMQ                3
#define VIDC_IFACEQ_CMDQ_IDX            0
#define VIDC_IFACEQ_MSGQ_IDX            1
#define VIDC_IFACEQ_DBGQ_IDX            2
#define VIDC_IFACEQ_MAX_BUF_COUNT       50
#define VIDC_IFACE_MAX_PARALLEL_CLNTS   16
#define VIDC_IFACEQ_DFLT_QHDR           0x01010000

struct hfi_queue_table_header {
	u32 qtbl_version;
	u32 qtbl_size;
	u32 qtbl_qhdr0_offset;
	u32 qtbl_qhdr_size;
	u32 qtbl_num_q;
	u32 qtbl_num_active_q;
	void *device_addr;
	char name[256];
};

struct hfi_queue_header {
	u32 qhdr_status;
	u32 qhdr_start_addr;
	u32 qhdr_type;
	u32 qhdr_q_size;
	u32 qhdr_pkt_size;
	u32 qhdr_pkt_drop_cnt;
	u32 qhdr_rx_wm;
	u32 qhdr_tx_wm;
	u32 qhdr_rx_req;
	u32 qhdr_tx_req;
	u32 qhdr_rx_irq_status;
	u32 qhdr_tx_irq_status;
	u32 qhdr_read_idx;
	u32 qhdr_write_idx;
};

#define VIDC_IFACEQ_TABLE_SIZE	(sizeof(struct hfi_queue_table_header) + \
			sizeof(struct hfi_queue_header) * VIDC_IFACEQ_NUMQ)

#define VIDC_IFACEQ_QUEUE_SIZE	(VIDC_IFACEQ_MAX_PKT_SIZE *  \
	VIDC_IFACEQ_MAX_BUF_COUNT * VIDC_IFACE_MAX_PARALLEL_CLNTS)

#define VIDC_IFACEQ_GET_QHDR_START_ADDR(ptr, i)     \
	((void *)((ptr + sizeof(struct hfi_queue_table_header)) + \
		(i * sizeof(struct hfi_queue_header))))

#define QDSS_SIZE	4096
#define SFR_SIZE	4096
#define MMAP_BUF_SIZE	4096

#define QUEUE_SIZE	(VIDC_IFACEQ_TABLE_SIZE + \
			(VIDC_IFACEQ_QUEUE_SIZE * VIDC_IFACEQ_NUMQ))

#define ALIGNED_QDSS_SIZE	ALIGN(QDSS_SIZE, SZ_4K)
#define ALIGNED_SFR_SIZE	ALIGN(SFR_SIZE, SZ_4K)
#define ALIGNED_MMAP_BUF_SIZE	ALIGN(MMAP_BUF_SIZE, SZ_4K)
#define ALIGNED_QUEUE_SIZE	ALIGN(QUEUE_SIZE, SZ_4K)
#define SHARED_QSIZE		ALIGN(ALIGNED_SFR_SIZE + ALIGNED_QUEUE_SIZE + \
				      ALIGNED_QDSS_SIZE + ALIGNED_MMAP_BUF_SIZE, SZ_1M)
#define TOTAL_QSIZE	(SHARED_QSIZE - ALIGNED_SFR_SIZE - ALIGNED_QDSS_SIZE - \
					ALIGNED_MMAP_BUF_SIZE)

struct msm_vidc_core;

int venus_hfi_queue_cmd_write(struct msm_vidc_core *core, void *pkt);
int venus_hfi_queue_cmd_write_intr(struct msm_vidc_core *core, void *pkt,
				   bool allow_intr);
int venus_hfi_queue_msg_read(struct msm_vidc_core *core, void *pkt);
int venus_hfi_queue_dbg_read(struct msm_vidc_core *core, void *pkt);
void venus_hfi_queue_deinit(struct msm_vidc_core *core);
int venus_hfi_queue_init(struct msm_vidc_core *core);
int venus_hfi_reset_queue_header(struct msm_vidc_core *core);

#endif
