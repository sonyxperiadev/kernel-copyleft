/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __HW_FENCE_DRV_UTILS_H
#define __HW_FENCE_DRV_UTILS_H

/**
 * enum hw_fence_mem_reserve - Types of reservations for the carved-out memory.
 * HW_FENCE_MEM_RESERVE_CTRL_QUEUE: Reserve memory for the ctrl rx/tx queues.
 * HW_FENCE_MEM_RESERVE_LOCKS_REGION: Reserve memory for the per-client locks memory region.
 * HW_FENCE_MEM_RESERVE_TABLE: Reserve memory for the hw-fences global table.
 * HW_FENCE_MEM_RESERVE_CLIENT_QUEUE: Reserve memory per-client for the rx/tx queues.
 */
enum hw_fence_mem_reserve {
	HW_FENCE_MEM_RESERVE_CTRL_QUEUE,
	HW_FENCE_MEM_RESERVE_LOCKS_REGION,
	HW_FENCE_MEM_RESERVE_TABLE,
	HW_FENCE_MEM_RESERVE_CLIENT_QUEUE
};

/**
 * global_atomic_store() - Inter-processor lock
 * @drv_data: hw fence driver data
 * @lock: memory to lock
 * @val: if true, api locks the memory, if false it unlocks the memory
 */
void global_atomic_store(struct hw_fence_driver_data *drv_data, uint64_t *lock, bool val);

/**
 * hw_fence_utils_init_virq() - Initialilze doorbell (i.e. vIRQ) for SVM to HLOS signaling
 * @drv_data: hw fence driver data
 *
 * Returns zero if success, otherwise returns negative error code.
 */
int hw_fence_utils_init_virq(struct hw_fence_driver_data *drv_data);

/**
 * hw_fence_utils_process_doorbell_mask() - Sends doorbell mask to process the signaled clients
 *                                          this API is only exported for simulation purposes.
 * @drv_data: hw fence driver data.
 * @db_flags: doorbell flag
 */
void hw_fence_utils_process_doorbell_mask(struct hw_fence_driver_data *drv_data, u64 db_flags);

/**
 * hw_fence_utils_alloc_mem() - Allocates the carved-out memory pool that will be used for the HW
 *                              Fence global table, locks and queues.
 * @hw_fence_drv_data: hw fence driver data
 *
 * Returns zero if success, otherwise returns negative error code.
 */
int hw_fence_utils_alloc_mem(struct hw_fence_driver_data *hw_fence_drv_data);

/**
 * hw_fence_utils_reserve_mem() - Reserves memory from the carved-out memory pool.
 * @drv_data: hw fence driver data.
 * @type: memory reservation type.
 * @phys: physical address of the carved-out memory pool
 *
 * Returns zero if success, otherwise returns negative error code.
 */
int hw_fence_utils_reserve_mem(struct hw_fence_driver_data *drv_data,
	enum hw_fence_mem_reserve type, phys_addr_t *phys, void **pa, u32 *size, int client_id);

/**
 * hw_fence_utils_parse_dt_props() -  Init dt properties
 * @drv_data: hw fence driver data
 *
 * Returns zero if success, otherwise returns negative error code.
 */
int hw_fence_utils_parse_dt_props(struct hw_fence_driver_data *drv_data);

/**
 * hw_fence_utils_map_ipcc() -  Maps IPCC registers and enable signaling
 * @drv_data: hw fence driver data
 *
 * Returns zero if success, otherwise returns negative error code.
 */
int hw_fence_utils_map_ipcc(struct hw_fence_driver_data *drv_data);

/**
 * hw_fence_utils_map_qtime() -  Maps qtime register
 * @drv_data: hw fence driver data
 *
 * Returns zero if success, otherwise returns negative error code.
 */
int hw_fence_utils_map_qtime(struct hw_fence_driver_data *drv_data);

/**
 * hw_fence_utils_map_ctl_start() -  Maps ctl_start registers from dpu hw
 * @drv_data: hw fence driver data
 *
 * Returns zero if success, otherwise returns negative error code. This API is only used
 * for simulation purposes in platforms where dpu does not support ipc signal.
 */
int hw_fence_utils_map_ctl_start(struct hw_fence_driver_data *drv_data);

/**
 * hw_fence_utils_cleanup_fence() -  Cleanup the hw-fence from a specified client
 * @drv_data: hw fence driver data
 * @hw_fence_client: client, for which the fence must be cleared
 * @hw_fence: hw-fence to cleanup
 * @hash: hash of the hw-fence to cleanup
 * @reset_flags: flags to determine how to handle the reset
 *
 * Returns zero if success, otherwise returns negative error code.
 */
int hw_fence_utils_cleanup_fence(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client, struct msm_hw_fence *hw_fence, u64 hash,
	u32 reset_flags);

#endif /* __HW_FENCE_DRV_UTILS_H */
