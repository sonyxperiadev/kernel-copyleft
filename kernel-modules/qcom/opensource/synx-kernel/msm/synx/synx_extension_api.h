/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SYNX_EXTENSION_API_H__
#define __SYNX_EXTENSION_API_H__

/**
 * SYNX_HW_FENCE_FLAG_ENABLED_BIT - synx hw-fence is enabled for the dma_fence.
 *
 * This flag is set when fences are backed up by a synx hw-fence.
 */
#define SYNX_HW_FENCE_FLAG_ENABLED_BIT    31

/**
 * SYNX_HW_FENCE_FLAG_SIGNALED_BIT - synx hw-fence is signaled for the dma_fence.
 *
 * This flag is set when a client adds itself as a waiter for an already signaled
 * synx hw-fence. The client uses this flag to avoid adding itself as a waiter for
 * a fence that is already retired.
 */
#define SYNX_HW_FENCE_FLAG_SIGNALED_BIT    30

/**
 * struct synx_hw_fence_hfi_queue_table_header_v0 - HFI queue table structure version 0.
 * @version: HFI protocol version.
 * @size: Queue table size in dwords.
 * @qhdr0_offset: First queue header offset (dwords) in this table.
 * @qhdr_size: Queue header size.
 * @num_q: Number of queues defined in this table.
 * @num_active_q: Number of active queues.
 */
struct synx_hw_fence_hfi_queue_table_header_v0 {
	u32 version;
	u32 size;
	u32 qhdr0_offset;
	u32 qhdr_size;
	u32 num_q;
	u32 num_active_q;
};

/**
 * struct synx_hw_fence_hfi_queue_header_v0 - HFI queue header structure version 0.
 * @status: Active = 1, Inactive = 0.
 * @start_addr: Starting address of the queue.
 * @type: Queue type (rx/tx).
 * @queue_size: Size of the queue.
 * @pkt_size: Size of the queue packet entries,
 *            0 - means variable size of message in the queue,
 *            non-zero - size of the packet, fixed.
 * @pkt_drop_cnt: Number of packets drop by sender.
 * @rx_wm: Receiver watermark, applicable in event driven mode.
 * @tx_wm: Sender watermark, applicable in event driven mode.
 * @rx_req: Receiver sets this bit if queue is empty.
 * @tx_req: Sender sets this bit if queue is full.
 * @rx_irq_status: Receiver sets this bit and triggers an interrupt to the
 *                 sender after packets are dequeued. Sender clears this bit.
 * @tx_irq_status: Sender sets this bit and triggers an interrupt to the
 *                 receiver after packets are queued. Receiver clears this bit.
 * @read_index: read index of the queue.
 * @write_index: write index of the queue.
 */
struct synx_hw_fence_hfi_queue_header_v0 {
	u32 status;
	u32 start_addr;
	u32 type;
	u32 queue_size;
	u32 pkt_size;
	u32 pkt_drop_cnt;
	u32 rx_wm;
	u32 tx_wm;
	u32 rx_req;
	u32 tx_req;
	u32 rx_irq_status;
	u32 tx_irq_status;
	u32 read_index;
	u32 write_index;
};

/**
 * struct synx_hw_fence_hfi_queue_table_header - HFI queue table structure.
 * @version: HFI protocol version.
 * @size: Queue table size in dwords.
 * @qhdr0_offset: First queue header offset (dwords) in this table.
 * @qhdr_size: Queue header size.
 * @num_q: Number of queues defined in this table.
 * @num_active_q: Number of active queues.
 * @reserved: reserved memory used for 64-byte alignment
 */
struct synx_hw_fence_hfi_queue_table_header {
	u32 version;
	u32 size;
	u32 qhdr0_offset;
	u32 qhdr_size;
	u32 num_q;
	u32 num_active_q;
	u32 reserved[10];
};

/**
 * struct synx_hw_fence_hfi_queue_header - HFI queue header structure.
 * @status: Active = 1, Inactive = 0.
 * @start_addr: Starting address of the queue.
 * @type: Queue type (rx/tx).
 * @queue_size: Size of the queue.
 * @pkt_size: Size of the queue packet entries,
 *            0 - means variable size of message in the queue,
 *            non-zero - size of the packet, fixed.
 * @pkt_drop_cnt: Number of packets drop by sender.
 * @rx_wm: Receiver watermark, applicable in event driven mode.
 * @tx_wm: Sender watermark, applicable in event driven mode.
 * @rx_req: Receiver sets this bit if queue is empty.
 * @tx_req: Sender sets this bit if queue is full.
 * @rx_irq_status: Receiver sets this bit and triggers an interrupt to the
 *                 sender after packets are dequeued. Sender clears this bit.
 * @tx_irq_status: Sender sets this bit and triggers an interrupt to the
 *                 receiver after packets are queued. Receiver clears this bit.
 * @init_reserved: reservation for 64-byte alignment of read and write indexes
 * @read_index: read index of the queue.
 * @read_index_reserved: reservation for 64-byte alignment of read and write indexes
 * @write_index: write index of the queue.
 * @write_index_reserved: reservation for 64-byte alignment of read and write indexes
 */
struct synx_hw_fence_hfi_queue_header {
	u32 status;
	u32 start_addr;
	u32 type;
	u32 queue_size;
	u32 pkt_size;
	u32 pkt_drop_cnt;
	u32 rx_wm;
	u32 tx_wm;
	u32 rx_req;
	u32 tx_req;
	u32 rx_irq_status;
	u32 tx_irq_status;
	u32 init_reserved[4];
	u32 read_index;
	u32 read_index_reserved[15];
	u32 write_index;
	u32 write_index_reserved[15];
};

#endif /* __SYNX_EXTENSION_API_H__ */
