/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _ADRENO_GEN8_HWSCHED_HFI_H_
#define _ADRENO_GEN8_HWSCHED_HFI_H_

/* Maximum number of IBs in a submission */
#define HWSCHED_MAX_NUMIBS \
	((HFI_MAX_MSG_SIZE - offsetof(struct hfi_issue_cmd_cmd, ibs)) \
		/ sizeof(struct hfi_issue_ib))

/*
 * This is used to put userspace threads to sleep when hardware fence unack count reaches a
 * threshold. This bit is cleared in two scenarios:
 * 1. If the hardware fence unack count drops to a desired threshold.
 * 2. If there is a GMU/GPU fault. Because we don't want the threads to keep sleeping through fault
 *    recovery, which can easily take 100s of milliseconds to complete.
 */
#define GEN8_HWSCHED_HW_FENCE_SLEEP_BIT	0x0

/*
 * This is used to avoid creating any more hardware fences until the hardware fence unack count
 * drops to a desired threshold. This bit is required in cases where GEN8_HWSCHED_HW_FENCE_SLEEP_BIT
 * will be cleared, but we still want to avoid creating any more hardware fences. For example, if
 * hardware fence unack count reaches a maximum threshold, both GEN8_HWSCHED_HW_FENCE_SLEEP_BIT and
 * GEN8_HWSCHED_HW_FENCE_MAX_BIT will be set. Say, a GMU/GPU fault happens and
 * GEN8_HWSCHED_HW_FENCE_SLEEP_BIT will be cleared to wake up any sleeping threads. But,
 * GEN8_HWSCHED_HW_FENCE_MAX_BIT will remain set to avoid creating any new hardware fences until
 * recovery is complete and deferred drawctxt (if any) is handled.
 */
#define GEN8_HWSCHED_HW_FENCE_MAX_BIT	0x1

/*
 * This is used to avoid creating any more hardware fences until concurrent reset/recovery completes
 * or when soccp vote fails
 */
#define GEN8_HWSCHED_HW_FENCE_ABORT_BIT 0x2

#define MAX_THROTTLE_LVLS 3
struct hfi_tsens_cfg {
	/** @limit_u: deci-C value for upper trigger point */
	u32 limit_u;
	/** @limit_l: deci-C value for lower trigger point */
	u32 limit_l;
	/** @margin_u: deci-C value for upper trigger intercept margin */
	u32 margin_u;
	/** @margin_l: deci-C value for lower trigger intercept margin */
	u32 margin_l;
} __packed;

struct hfi_tsens_throttle_param {
	/** @throttle_hyst: Microsecond wait between each throttle level */
	u32 throttle_hyst;
	/** @num_throttle_cnt: Number of entries in throttle_levels */
	u32 num_throttle_cnt;
	/** @throttle_lvls: Percent of original clock to throttle per level */
	u32 throttle_lvls[MAX_THROTTLE_LVLS];
} __packed;

struct hfi_therm_profile_ctrl {
	/** @feature_en: Feature enable status */
	u16 feature_en;
	/** @feature_rev: Feature revision */
	u16 feature_rev;
	/** @tsens_en: tsens sensor enable status */
	u32 tsens_en;
	/** @tj_limit: deci-C value for tj limit */
	u32 tj_limit;
	/** @tskin_addr: unused */
	u32 tskin_addr;
	/** @tskin_limit: unused */
	u32 tskin_limit;
	/** @tsens_cfg_cnt: Count of tsens configuration structs */
	u32 tsens_cfg_cnt;
	/** @tsens_cfg: Struct of tsens configurations */
	struct hfi_tsens_cfg tsens_cfg;
	/** @throttle_cfg: Struct of throttle configurations */
	struct hfi_tsens_throttle_param throttle_cfg;
} __packed;

/* H2F */
struct hfi_thermaltable_cmd {
	/** @hdr: HFI header message */
	u32 hdr;
	/** @version: Version identifier for the format used for domains */
	u32 version;
	/** @ctrl: Thermal profile control information */
	struct hfi_therm_profile_ctrl ctrl;
} __packed;

struct gen8_hwsched_hfi {
	struct hfi_mem_alloc_entry mem_alloc_table[32];
	u32 mem_alloc_entries;
	/** @irq_mask: Store the hfi interrupt mask */
	u32 irq_mask;
	/** @msglock: To protect the list of un-ACKed hfi packets */
	rwlock_t msglock;
	/** @msglist: List of un-ACKed hfi packets */
	struct list_head msglist;
	/** @f2h_task: Task for processing gmu fw to host packets */
	struct task_struct *f2h_task;
	/** @f2h_wq: Waitqueue for the f2h_task */
	wait_queue_head_t f2h_wq;
	/** @big_ib: GMU buffer to hold big IBs */
	struct kgsl_memdesc *big_ib;
	/** @big_ib_recurring: GMU buffer to hold big recurring IBs */
	struct kgsl_memdesc *big_ib_recurring;
	/** @msg_mutex: Mutex for accessing the msgq */
	struct mutex msgq_mutex;
	struct {
		/** @lock: Spinlock for managing hardware fences */
		spinlock_t lock;
		/**
		 * @unack_count: Number of hardware fences sent to GMU but haven't yet been ack'd
		 * by GMU
		 */
		u32 unack_count;
		/**
		 * @unack_wq: Waitqueue to wait on till number of unacked hardware fences drops to
		 * a desired threshold
		 */
		wait_queue_head_t unack_wq;
		/**
		 * @defer_drawctxt: Drawctxt to send hardware fences from as soon as unacked
		 * hardware fences drops to a desired threshold
		 */
		struct adreno_context *defer_drawctxt;
		/**
		 * @defer_ts: The timestamp of the hardware fence which got deferred
		 */
		u32 defer_ts;
		/**
		 * @flags: Flags to control the creation of new hardware fences
		 */
		unsigned long flags;
		/** @seqnum: Sequence number for hardware fence packet header */
		atomic_t seqnum;
		/** @soccp_rproc: rproc handle for soccp */
		struct rproc *soccp_rproc;
	} hw_fence;
	/**
	 * @hw_fence_timer: Timer to trigger fault if unack'd hardware fence count does'nt drop
	 * to a desired threshold in given amount of time
	 */
	struct timer_list hw_fence_timer;
	/**
	 * @hw_fence_ws: Work struct that gets scheduled when hw_fence_timer expires
	 */
	struct work_struct hw_fence_ws;
	/** @detached_hw_fences_list: List of hardware fences belonging to detached contexts */
	struct list_head detached_hw_fence_list;
	/** @defer_hw_fence_work: The work structure to send deferred hardware fences to GMU */
	struct kthread_work defer_hw_fence_work;
};

struct kgsl_drawobj_cmd;

/**
 * gen8_hwsched_hfi_probe - Probe hwsched hfi resources
 * @adreno_dev: Pointer to adreno device structure
 *
 * Return: 0 on success and negative error on failure.
 */
int gen8_hwsched_hfi_probe(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_hfi_remove - Release hwsched hfi resources
 * @adreno_dev: Pointer to adreno device structure
 */
void gen8_hwsched_hfi_remove(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_hfi_init - Initialize hfi resources
 * @adreno_dev: Pointer to adreno device structure
 *
 * This function is used to initialize hfi resources
 * once before the very first gmu boot
 *
 * Return: 0 on success and negative error on failure.
 */
int gen8_hwsched_hfi_init(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_hfi_start - Start hfi resources
 * @adreno_dev: Pointer to adreno device structure
 *
 * Send the various hfi packets before booting the gpu
 *
 * Return: 0 on success and negative error on failure.
 */
int gen8_hwsched_hfi_start(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_hfi_stop - Stop the hfi resources
 * @adreno_dev: Pointer to the adreno device
 *
 * This function does the hfi cleanup when powering down the gmu
 */
void gen8_hwsched_hfi_stop(struct adreno_device *adreno_dev);

/**
 * gen8_hwched_cp_init - Send CP_INIT via HFI
 * @adreno_dev: Pointer to adreno device structure
 *
 * This function is used to send CP INIT packet and bring
 * GPU out of secure mode using hfi raw packets.
 *
 * Return: 0 on success and negative error on failure.
 */
int gen8_hwsched_cp_init(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_send_cmd_async - Send an hfi packet
 * @adreno_dev: Pointer to adreno device structure
 * @data: Data to be sent in the hfi packet
 * @size_bytes: Size of the packet in bytes
 *
 * Send data in the form of an HFI packet to gmu and wait for
 * it's ack asynchronously
 *
 * Return: 0 on success and negative error on failure.
 */
int gen8_hfi_send_cmd_async(struct adreno_device *adreno_dev, void *data, u32 size_bytes);

/**
 * gen8_hwsched_submit_drawobj - Dispatch IBs to dispatch queues
 * @adreno_dev: Pointer to adreno device structure
 * @drawobj: The command draw object which needs to be submitted
 *
 * This function is used to register the context if needed and submit
 * IBs to the hfi dispatch queues.

 * Return: 0 on success and negative error on failure
 */
int gen8_hwsched_submit_drawobj(struct adreno_device *adreno_dev,
		struct kgsl_drawobj *drawobj);

/**
 * gen8_hwsched_context_detach - Unregister a context with GMU
 * @drawctxt: Pointer to the adreno context
 *
 * This function sends context unregister HFI and waits for the ack
 * to ensure all submissions from this context have retired
 */
void gen8_hwsched_context_detach(struct adreno_context *drawctxt);

/* Helper function to get to gen8 hwsched hfi device from adreno device */
struct gen8_hwsched_hfi *to_gen8_hwsched_hfi(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_preempt_count_get - Get preemption count from GMU
 * @adreno_dev: Pointer to adreno device
 *
 * This function sends a GET_VALUE HFI packet to get the number of
 * preemptions completed since last SLUMBER exit.
 *
 * Return: Preemption count
 */
u32 gen8_hwsched_preempt_count_get(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_lpac_cp_init - Send CP_INIT to LPAC via HFI
 * @adreno_dev: Pointer to adreno device structure
 *
 * This function is used to send CP INIT packet to LPAC and
 * enable submission to LPAC queue.
 *
 * Return: 0 on success and negative error on failure.
 */
int gen8_hwsched_lpac_cp_init(struct adreno_device *adreno_dev);

/**
 * gen8_hfi_send_lpac_feature_ctrl - Send the lpac feature hfi packet
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_hfi_send_lpac_feature_ctrl(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_context_destroy - Destroy any hwsched related resources during context destruction
 * @adreno_dev: Pointer to adreno device
 * @drawctxt: Pointer to the adreno context
 *
 * This functions destroys any hwsched related resources when this context is destroyed
 */
void gen8_hwsched_context_destroy(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt);

/**
 * gen8_hwsched_hfi_get_value - Send GET_VALUE packet to GMU to get the value of a property
 * @adreno_dev: Pointer to adreno device
 * @prop: property to get from GMU
 *
 * This functions sends GET_VALUE HFI packet to query value of a property
 *
 * Return: On success, return the value in the GMU response. On failure, return 0
 */
u32 gen8_hwsched_hfi_get_value(struct adreno_device *adreno_dev, u32 prop);

/**
 * gen8_send_hw_fence_hfi_wait_ack - Send hardware fence info to GMU
 * @adreno_dev: Pointer to adreno device
 * @entry: Pointer to the adreno hardware fence entry
 * @flags: Flags for this hardware fence
 *
 * Send the hardware fence info to the GMU and wait for the ack
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_send_hw_fence_hfi_wait_ack(struct adreno_device *adreno_dev,
	struct adreno_hw_fence_entry *entry, u64 flags);

/**
 * gen8_hwsched_create_hw_fence - Create a hardware fence
 * @adreno_dev: Pointer to adreno device
 * @kfence: Pointer to the kgsl fence
 *
 * Create a hardware fence, set up hardware fence info and send it to GMU if required
 */
void gen8_hwsched_create_hw_fence(struct adreno_device *adreno_dev,
	struct kgsl_sync_fence *kfence);

/**
 * gen8_hwsched_check_context_inflight_hw_fences - Check whether all hardware fences
 * from a context have been sent to the TxQueue or not
 * @adreno_dev: Pointer to adreno device
 * @drawctxt: Pointer to the adreno context which is to be flushed
 *
 * Check if all hardware fences from this context have been sent to the
 * TxQueue. If not, log an error and return error code.
 *
 * Return: Zero on success or negative error on failure
 */
int gen8_hwsched_check_context_inflight_hw_fences(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt);

/**
 * gen8_hwsched_disable_hw_fence_throttle - Disable hardware fence throttling after reset
 * @adreno_dev: pointer to the adreno device
 *
 * After device reset, clear hardware fence related data structures and send any hardware fences
 * that got deferred (prior to reset) and re-open the gates for hardware fence creation
 *
 * Return: Zero on success or negative error on failure
 */
int gen8_hwsched_disable_hw_fence_throttle(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_process_msgq - Process msgq
 * @adreno_dev: pointer to the adreno device
 *
 * This function grabs the msgq mutex and processes msgq for any outstanding hfi packets
 */
void gen8_hwsched_process_msgq(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_boot_gpu - Send the command to boot GPU
 * @adreno_dev: Pointer to adreno device
 *
 * Send the hfi to boot GPU, and check the ack, incase of a failure
 * get a snapshot and capture registers of interest.
 *
 * Return: Zero on success or negative error on failure
 */
int gen8_hwsched_boot_gpu(struct adreno_device *adreno_dev);

/**
 * gen8_hwsched_get_rb_hostptr - Get rinbuffer host pointer
 * @adreno_dev: pointer to the adreno device
 * @gpuaddr: ringbuffer gpu address
 * @size: size of the ringbuffer
 *
 * Return: Host pointer of the gpu ringbuffer
 */
void *gen8_hwsched_get_rb_hostptr(struct adreno_device *adreno_dev,
	u64 gpuaddr, u32 size);
#endif
