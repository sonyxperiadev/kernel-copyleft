/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _ADRENO_HWSCHED_H_
#define _ADRENO_HWSCHED_H_

#include "kgsl_sync.h"

/* This structure represents inflight command object */
struct cmd_list_obj {
	/** @drawobj: Handle to the draw object */
	struct kgsl_drawobj *drawobj;
	/** @node: List node to put it in the list of inflight commands */
	struct list_head node;
};

/**
 * struct adreno_hw_fence_entry - A structure to store hardware fence and the context
 */
struct adreno_hw_fence_entry {
	/** @cmd: H2F_MSG_HW_FENCE_INFO packet for this hardware fence */
	struct hfi_hw_fence_info cmd;
	/** @kfence: Pointer to the kgsl fence */
	struct kgsl_sync_fence *kfence;
	/** @drawctxt: Pointer to the context */
	struct adreno_context *drawctxt;
	/** @node: list node to add it to a list */
	struct list_head node;
	/** @reset_node: list node to add it to post reset list of hardware fences */
	struct list_head reset_node;
};

/**
 * struct adreno_hwsched_ops - Function table to hook hwscheduler things
 * to target specific routines
 */
struct adreno_hwsched_ops {
	/**
	 * @submit_drawobj - Target specific function to submit IBs to hardware
	 */
	int (*submit_drawobj)(struct adreno_device *adreno_dev,
		struct kgsl_drawobj *drawobj);
	/**
	 * @preempt_count - Target specific function to get preemption count
	 */
	u32 (*preempt_count)(struct adreno_device *adreno_dev);
	/**
	 * @create_hw_fence - Target specific function to create a hardware fence
	 */
	void (*create_hw_fence)(struct adreno_device *adreno_dev,
		struct kgsl_sync_fence *kfence);
	/**
	 * @get_rb_hostptr - Target specific function to get ringbuffer host pointer
	 */
	void *(*get_rb_hostptr)(struct adreno_device *adreno_dev, u64 gpuaddr, u32 size);
};

/**
 * struct adreno_hwsched - Container for the hardware scheduler
 */
struct adreno_hwsched {
	 /** @mutex: Mutex needed to run dispatcher function */
	struct mutex mutex;
	/** @flags: Container for the dispatcher internal flags */
	unsigned long flags;
	/** @inflight: Number of active submissions to the dispatch queues */
	u32 inflight;
	/** @jobs - Array of dispatch job lists for each priority level */
	struct llist_head jobs[16];
	/** @requeue - Array of lists for dispatch jobs that got requeued */
	struct llist_head requeue[16];
	/** @cmd_list: List of objects submitted to dispatch queues */
	struct list_head cmd_list;
	/** @hwsched_ops: Container for target specific hwscheduler ops */
	const struct adreno_hwsched_ops *hwsched_ops;
	/** @ctxt_bad: Container for the context bad hfi packet */
	void *ctxt_bad;
	/** @idle_gate: Gate to wait on for hwscheduler to idle */
	struct completion idle_gate;
	/** @big_cmdobj = Points to the big IB that is inflight */
	struct kgsl_drawobj_cmd *big_cmdobj;
	/** @recurring_cmdobj: Recurring commmand object sent to GMU */
	struct kgsl_drawobj_cmd *recurring_cmdobj;
	/** @lsr_timer: Timer struct to schedule lsr work */
	struct timer_list lsr_timer;
	/** @lsr_check_ws: Lsr work to update power stats */
	struct work_struct lsr_check_ws;
	/** @hw_fence: Container for the hw fences instance */
	struct kmem_cache *hw_fence_cache;
	/** @hw_fence_count: Number of hardware fences that haven't yet been sent to Tx Queue */
	atomic_t hw_fence_count;
	/**
	 * @submission_seqnum: Sequence number for sending submissions to GMU context queues or
	 * dispatch queues
	 */
	atomic_t submission_seqnum;
	/** @global_ctxtq: Memory descriptor for global context queue */
	struct kgsl_memdesc global_ctxtq;
	/** @global_ctxt_gmu_registered: Whether global context is registered with gmu */
	bool global_ctxt_gmu_registered;
	/** @hw_fence_md: Kgsl memory descriptor for hardware fences queue */
	struct kgsl_memdesc hw_fence_md;
};

/*
 * This value is based on maximum number of IBs that can fit
 * in the ringbuffer.
 */
#define HWSCHED_MAX_IBS 2000

enum adreno_hwsched_flags {
	ADRENO_HWSCHED_POWER = 0,
	ADRENO_HWSCHED_ACTIVE,
	ADRENO_HWSCHED_CTX_BAD_LEGACY,
	ADRENO_HWSCHED_CONTEXT_QUEUE,
	ADRENO_HWSCHED_HW_FENCE,
};

/**
 * adreno_hwsched_start() - activate the hwsched dispatcher
 * @adreno_dev: pointer to the adreno device
 *
 * Enable dispatcher thread to execute
 */
void adreno_hwsched_start(struct adreno_device *adreno_dev);
/**
 * adreno_hwsched_init() - Initialize the hwsched
 * @adreno_dev: pointer to the adreno device
 * @hwsched_ops: Pointer to target specific hwsched ops
 *
 * Set up the hwsched resources.
 * Return: 0 on success or negative on failure.
 */
int adreno_hwsched_init(struct adreno_device *adreno_dev,
	const struct adreno_hwsched_ops *hwsched_ops);

/**
 * adreno_hwsched_parse_fault_ib - Parse the faulty submission
 * @adreno_dev: pointer to the adreno device
 * @snapshot: Pointer to the snapshot structure
 *
 * Walk the list of active submissions to find the one that faulted and
 * parse it so that relevant command buffers can be added to the snapshot
 */
void adreno_hwsched_parse_fault_cmdobj(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot);

/**
 * adreno_hwsched_unregister_contexts - Reset context gmu_registered bit
 * @adreno_dev: pointer to the adreno device
 *
 * Walk the list of contexts and reset the gmu_registered for all
 * contexts
 */
void adreno_hwsched_unregister_contexts(struct adreno_device *adreno_dev);

/**
 * adreno_hwsched_idle - Wait for dispatcher and hardware to become idle
 * @adreno_dev: A handle to adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int adreno_hwsched_idle(struct adreno_device *adreno_dev);

void adreno_hwsched_retire_cmdobj(struct adreno_hwsched *hwsched,
	struct kgsl_drawobj_cmd *cmdobj);

bool adreno_hwsched_context_queue_enabled(struct adreno_device *adreno_dev);

/**
 * adreno_hwsched_register_hw_fence - Register GPU as a hardware fence client
 * @adreno_dev: pointer to the adreno device
 *
 * Register with the hardware fence driver to be able to trigger and wait
 * for hardware fences. Also, set up the memory descriptor for mapping the
 * client queue to the GMU.
 */
void adreno_hwsched_register_hw_fence(struct adreno_device *adreno_dev);

/**
 * adreno_hwsched_deregister_hw_fence - Deregister GPU as a hardware fence client
 * @adreno_dev: pointer to the adreno device
 *
 * Deregister with the hardware fence driver and free up any resources allocated
 * as part of registering with the hardware fence driver
 */
void adreno_hwsched_deregister_hw_fence(struct adreno_device *adreno_dev);

/**
 * adreno_hwsched_replay - Resubmit inflight cmdbatches after gpu reset
 * @adreno_dev: pointer to the adreno device
 *
 * Resubmit all cmdbatches to GMU after device reset
 */
void adreno_hwsched_replay(struct adreno_device *adreno_dev);

/**
 * adreno_hwsched_parse_payload - Parse payload to look up a key
 * @payload: Pointer to a payload section
 * @key: The key who's value is to be looked up
 *
 * This function parses the payload data which is a sequence
 * of key-value pairs.
 *
 * Return: The value of the key or 0 if key is not found
 */
u32 adreno_hwsched_parse_payload(struct payload_section *payload, u32 key);

/**
 * adreno_hwsched_gpu_fault - Gets hwsched gpu fault info
 * @adreno_dev: pointer to the adreno device
 *
 * Returns zero for hwsched fault else non zero value
 */
u32 adreno_hwsched_gpu_fault(struct adreno_device *adreno_dev);

/**
 * adreno_hwsched_log_destroy_pending_fences - Log and destroy any pending hardware fences if soccp
 * vote failed
 * @adreno_dev: pointer to the adreno device
 * @dev: Pointer to the gmu pdev device
 */
void adreno_hwsched_log_destroy_pending_hw_fences(struct adreno_device *adreno_dev,
	struct device *dev);

/**
 * adreno_hwsched_syncobj_kfence_put - Put back kfence context refcounts for this sync object
 * @syncobj: Pointer to the sync object
 *
 */
void adreno_hwsched_syncobj_kfence_put(struct kgsl_drawobj_sync *syncobj);

/**
 * adreno_hwsched_log_nonfatal_gpu_fault - Logs non fatal GPU error from context bad hfi packet
 * @adreno_dev: pointer to the adreno device
 * @dev: Pointer to the struct device for the GMU platform device
 * @error: Types of error that triggered from context bad HFI
 *
 * This function parses context bad hfi packet and logs error information.
 *
 * Return: True for non fatal error code else false.
 */
bool adreno_hwsched_log_nonfatal_gpu_fault(struct adreno_device *adreno_dev,
		struct device *dev, u32 error);

/**
 * adreno_hwsched_poll_msg_queue_write_index - Poll on write index of HFI message queue
 * @hfi_mem: Memory descriptor for HFI queue table
 *
 * Returns zero if write index advances or ETIMEDOUT if timed out polling
 */
int adreno_hwsched_poll_msg_queue_write_index(struct kgsl_memdesc *hfi_mem);

/**
 * adreno_hwsched_remove_hw_fence_entry - Remove hardware fence entry
 * @adreno_dev: pointer to the adreno device
 * @entry: Pointer to the hardware fence entry
 */
void adreno_hwsched_remove_hw_fence_entry(struct adreno_device *adreno_dev,
	struct adreno_hw_fence_entry *entry);

/**
 * adreno_gmu_context_queue_read - Read data from context queue
 * @drawctxt: Pointer to the adreno draw context
 * @output: Pointer to read the data into
 * @read_idx: Index to read the data from
 * @size: Number of dwords to read from the context queue
 *
 * Return: 0 on success or negative error on failure
 */
int adreno_gmu_context_queue_read(struct adreno_context *drawctxt, u32 *output,
	u32 read_idx, u32 size);

/**
 * adreno_gmu_context_queue_write - Write data to context queue
 *
 * @adreno_dev: Pointer to adreno device structure
 * @gmu_context_queue: Pointer to the memory descriptor for context queue
 * @msg: Pointer to the message data to be written
 * @size_bytes: Size of the message data in bytes
 * @drawobj: Pointer to the draw object
 * @time: Pointer to the submission time information
 *
 * Return: 0 on success or negative error on failure
 */
int adreno_gmu_context_queue_write(struct adreno_device *adreno_dev,
	struct kgsl_memdesc *gmu_context_queue, u32 *msg, u32 size_bytes,
	struct kgsl_drawobj *drawobj, struct adreno_submit_time *time);

/**
 * adreno_hwsched_add_profile_events - Add profiling events
 *
 * @adreno_dev: Pointer to the adreno device structure
 * @cmdobj: Pointer to the command object
 * @time: Pointer to the submission time information
 */
void adreno_hwsched_add_profile_events(struct adreno_device *adreno_dev,
	struct kgsl_drawobj_cmd *cmdobj, struct adreno_submit_time *time);
#endif
