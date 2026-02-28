/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __ADRENO_GEN8_HFI_H
#define __ADRENO_GEN8_HFI_H

#include "adreno_hfi.h"

/**
 * struct gen8_hfi - HFI control structure
 */
struct gen8_hfi {
	/** @irq: HFI interrupt line */
	int irq;
	/** @seqnum: atomic counter that is incremented for each message sent.
	 *   The value of the counter is used as sequence number for HFI message.
	 */
	atomic_t seqnum;
	/** @hfi_mem: Memory descriptor for the hfi memory */
	struct kgsl_memdesc *hfi_mem;
	/** @bw_table: HFI BW table buffer */
	struct hfi_bwtable_cmd bw_table;
	/** @acd_table: HFI table for ACD data */
	struct hfi_acd_table_cmd acd_table;
	/** @cmdq_lock: Spinlock for accessing the cmdq */
	spinlock_t cmdq_lock;
	/**
	 * @wb_set_record_bitmask: Bitmask to enable or disable the recording
	 * of messages in the GMU scratch.
	 */
	unsigned long wb_set_record_bitmask[BITS_TO_LONGS(HFI_MAX_ID)];
};

struct gen8_gmu_device;

/* gen8_hfi_irq_handler - IRQ handler for HFI interripts */
irqreturn_t gen8_hfi_irq_handler(int irq, void *data);

/**
 * gen8_hfi_start - Send the various HFIs during device boot up
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_start(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_start - Send the various HFIs during device boot up
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
void gen8_hfi_stop(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_init - Initialize hfi resources
 * @adreno_dev: Pointer to the adreno device
 *
 * This function allocates and sets up hfi queues
 * when a process creates the very first kgsl instance
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_init(struct adreno_device *adreno_dev);

/* Helper function to get to gen8 hfi struct from adreno device */
struct gen8_hfi *to_gen8_hfi(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_queue_write - Write a command to hfi queue
 * @adreno_dev: Pointer to the adreno device
 * @queue_idx: destination queue id
 * @msg: Data to be written to the queue
 * @size_bytes: Size of the command in bytes
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_queue_write(struct adreno_device *adreno_dev, u32 queue_idx,
		u32 *msg, u32 size_bytes);

/**
 * gen8_hfi_queue_read - Read data from hfi queue
 * @gmu: Pointer to the gen8 gmu device
 * @queue_idx: queue id to read from
 * @output: Pointer to read the data into
 * @max_size: Number of bytes to read from the queue
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_queue_read(struct gen8_gmu_device *gmu, u32 queue_idx,
		u32 *output, u32 max_size);

/**
 * gen8_receive_ack_cmd - Process ack type packets
 * @gmu: Pointer to the gen8 gmu device
 * @rcvd: Pointer to the data read from hfi queue
 * @ret_cmd: Container for the hfi packet for which this ack is received
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_receive_ack_cmd(struct gen8_gmu_device *gmu, void *rcvd,
		struct pending_cmd *ret_cmd);

/**
 * gen8_hfi_send_feature_ctrl - Enable gmu feature via hfi
 * @adreno_dev: Pointer to the adreno device
 * @feature: feature to be enabled or disabled
 * enable: Set 1 to enable or 0 to disable a feature
 * @data: payload for the send feature hfi packet
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_feature_ctrl(struct adreno_device *adreno_dev,
		u32 feature, u32 enable, u32 data);

/**
 * gen8_hfi_send_get_value - Send gmu get_values via hfi
 * @adreno_dev: Pointer to the adreno device
 * @type: GMU get_value type
 * @subtype: GMU get_value subtype
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_get_value(struct adreno_device *adreno_dev, u32 type, u32 subtype);

/**
 * gen8_hfi_send_set_value - Send gmu set_values via hfi
 * @adreno_dev: Pointer to the adreno device
 * @type: GMU set_value type
 * @subtype: GMU set_value subtype
 * @data: Value to set
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_set_value(struct adreno_device *adreno_dev,
		u32 type, u32 subtype, u32 data);

/**
 * gen8_hfi_send_core_fw_start - Send the core fw start hfi
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_core_fw_start(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_send_acd_feature_ctrl - Send the acd table and acd feature
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_acd_feature_ctrl(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_send_generic_req - Send a generic hfi packet
 * @adreno_dev: Pointer to the adreno device
 * @cmd: Pointer to the hfi packet header and data
 * @size_bytes: Size of the packet in bytes
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_generic_req(struct adreno_device *adreno_dev, void *cmd, u32 size_bytes);

/**
 * gen8_hfi_send_generic_req_v5 - Send a generic hfi packet with additional error handling
 * @adreno_dev: Pointer to the adreno device
 * @cmd: Pointer to the hfi packet header and data
 * @ret_cmd: Ack for the command we just sent
 * @size_bytes: Size of the packet in bytes
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_generic_req_v5(struct adreno_device *adreno_dev, void *cmd,
		struct pending_cmd *ret_cmd, u32 size_bytes);

/**
 * gen8_hfi_send_bcl_feature_ctrl - Send the bcl feature hfi packet
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_bcl_feature_ctrl(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_send_clx_feature_ctrl - Send the clx feature hfi packet
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_clx_feature_ctrl(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_send_ifpc_feature_ctrl - Send the ipfc feature hfi packet
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_ifpc_feature_ctrl(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_send_gpu_perf_table - Send the gpu perf table hfi packet
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_gpu_perf_table(struct adreno_device *adreno_dev);

/*
 * gen8_hfi_process_queue - Check hfi queue for messages from gmu
 * @gmu: Pointer to the gen8 gmu device
 * @queue_idx: queue id to be processed
 * @ret_cmd: Container for data needed for waiting for the ack
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_process_queue(struct gen8_gmu_device *gmu,
		u32 queue_idx, struct pending_cmd *ret_cmd);

/**
 * gen8_hfi_cmdq_write - Write a command to command queue
 * @adreno_dev: Pointer to the adreno device
 * @msg: Data to be written to the queue
 * @size_bytes: Size of the command in bytes
 *
 * This function takes the cmdq lock before writing data to the queue

 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_cmdq_write(struct adreno_device *adreno_dev, u32 *msg, u32 size_bytes);
void adreno_gen8_receive_err_req(struct gen8_gmu_device *gmu, void *rcvd);
void adreno_gen8_receive_debug_req(struct gen8_gmu_device *gmu, void *rcvd);
#endif
