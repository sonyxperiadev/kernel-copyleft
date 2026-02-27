/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_MEM_MGR_H_
#define _CAM_MEM_MGR_H_

#include <linux/mutex.h>
#include <linux/dma-buf.h>
#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)
#include <linux/dma-heap.h>
#endif
#include <media/cam_req_mgr.h>
#include "cam_mem_mgr_api.h"

/* Enum for possible mem mgr states */
enum cam_mem_mgr_state {
	CAM_MEM_MGR_UNINITIALIZED,
	CAM_MEM_MGR_INITIALIZED,
};

/*Enum for memory allocation initiator */
enum cam_mem_mgr_allocator {
	CAM_MEMMGR_ALLOC_USER,
	CAM_MEMMGR_ALLOC_KERNEL,
};

/*Enum for possible SMMU operations */
enum cam_smmu_mapping_client {
	CAM_SMMU_MAPPING_USER,
	CAM_SMMU_MAPPING_KERNEL,
};

#ifdef CONFIG_CAM_PRESIL
struct cam_presil_dmabuf_params {
	int32_t fd_for_umd_daemon;
	uint32_t refcount;
};
#endif

/**
 * struct cam_mem_buf_hw_vaddr_info
 *
 * @iommu_hdl:     IOMMU handle for the given bank
 * @vaddr:         IOVA of the buffer
 * @len:           cached length for a given handle
 * @ref_count:     ref count for buffer
 * @addr_updated:  Indicates if entry is updated only for addr caching
 * @valid_mapping: Indicates if entry is indeed a valid mapping for this buf
 *
 */
struct cam_mem_buf_hw_hdl_info {
	int32_t iommu_hdl;
	dma_addr_t vaddr;
	size_t len;
	struct kref *ref_count;

	bool addr_updated;
	bool valid_mapping;
};

/**
 * struct cam_mem_buf_queue
 *
 * @dma_buf:           pointer to the allocated dma_buf in the table
 * @q_lock:            mutex lock for buffer
 * @fd:                file descriptor of buffer
 * @i_ino:             inode number of this dmabuf. Uniquely identifies a buffer
 * @buf_handle:        unique handle for buffer
 * @align:             alignment for allocation
 * @len:               size of buffer
 * @flags:             attributes of buffer
 * @num_hdls:          number of valid handles
 * @vaddr_info:        Array of IOVA addresses mapped for different devices
 *                     using the same indexing as SMMU
 * @kmdvaddr:          Kernel virtual address
 * @active:            state of the buffer
 * @release_deferred:  Buffer is deferred for release.
 * @is_imported:       Flag indicating if buffer is imported from an FD in user space
 * @is_internal:       Flag indicating kernel allocated buffer
 * @timestamp:         Timestamp at which this entry in tbl was made
 * @krefcount:         Reference counter to track whether the buffer is
 *                     mapped and in use
 * @smmu_mapping_client: Client buffer (User or kernel)
 * @buf_name:            Name associated with buffer.
 * @presil_params:       Parameters specific to presil environment
 */
struct cam_mem_buf_queue {
	struct dma_buf *dma_buf;
	struct mutex q_lock;
	int32_t fd;
	unsigned long i_ino;
	int32_t buf_handle;
	int32_t align;
	size_t len;
	uint32_t flags;
	uintptr_t kmdvaddr;
	int32_t num_hdls;
	struct cam_mem_buf_hw_hdl_info *hdls_info;
	bool active;
	bool release_deferred;
	bool is_imported;
	bool is_internal;
	struct timespec64 timestamp;
	struct kref krefcount;
	enum cam_smmu_mapping_client smmu_mapping_client;
	char buf_name[CAM_DMA_BUF_NAME_LEN];

#ifdef CONFIG_CAM_PRESIL
	struct cam_presil_dmabuf_params presil_params;
#endif
};

/**
 * struct cam_mem_table
 *
 * @m_lock: mutex lock for table
 * @bitmap: bitmap of the mem mgr utility
 * @bits: max bits of the utility
 * @bufq: array of buffers
 * @dbg_buf_idx: debug buffer index to get usecases info
 * @max_hdls_supported: Maximum number of SMMU device handles supported
 *                      A buffer can only be mapped for these number of
 *                      device context banks
 * @max_hdls_info_size: Size of the hdls array allocated per buffer,
 *                      computed value to be used in driver
 * @force_cache_allocs: Force all internal buffer allocations with cache
 * @need_shared_buffer_padding: Whether padding is needed for shared buffer
 *                              allocations.
 * @csf_version: Camera security framework version
 * @system_heap: Handle to system heap
 * @system_movable_heap: Handle to system movable heap
 * @system_uncached_heap: Handle to system uncached heap
 * @camera_heap: Handle to camera heap
 * @camera_uncached_heap: Handle to camera uncached heap
 * @secure_display_heap: Handle to secure display heap
 * @ubwc_p_heap: Handle to ubwc-p heap
 * @ubwc_p_movable_heap: Handle to ubwc-p movable heap
 */
struct cam_mem_table {
	struct mutex m_lock;
	void *bitmap;
	size_t bits;
	struct cam_mem_buf_queue bufq[CAM_MEM_BUFQ_MAX];
	size_t dbg_buf_idx;
	int32_t max_hdls_supported;
	size_t max_hdls_info_size;
	bool force_cache_allocs;
	bool need_shared_buffer_padding;
	struct cam_csf_version csf_version;
#if IS_REACHABLE(CONFIG_DMABUF_HEAPS)
	struct dma_heap *system_heap;
	struct dma_heap *system_movable_heap;
	struct dma_heap *system_uncached_heap;
	struct dma_heap *camera_heap;
	struct dma_heap *camera_uncached_heap;
	struct dma_heap *secure_display_heap;
	struct dma_heap *ubwc_p_heap;
	struct dma_heap *ubwc_p_movable_heap;
#endif

};

/**
 * struct cam_mem_table_mini_dump
 *
 * @bufq: array of buffers
 * @dbg_buf_idx: debug buffer index to get usecases info
 * @alloc_profile_enable: Whether to enable alloc profiling
 * @dbg_buf_idx: debug buffer index to get usecases info
 * @force_cache_allocs: Force all internal buffer allocations with cache
 * @need_shared_buffer_padding: Whether padding is needed for shared buffer
 *                              allocations.
 */
struct cam_mem_table_mini_dump {
	struct cam_mem_buf_queue bufq[CAM_MEM_BUFQ_MAX];
	size_t dbg_buf_idx;
	bool   alloc_profile_enable;
	bool   force_cache_allocs;
	bool   need_shared_buffer_padding;
};

/**
 * @brief: Allocates and maps buffer
 *
 * @cmd:   Allocation information
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_alloc_and_map(struct cam_mem_mgr_alloc_cmd_v2 *cmd);

/**
 * @brief: Releases a buffer reference
 *
 * @cmd:   Buffer release information
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_release(struct cam_mem_mgr_release_cmd *cmd);

/**
 * @brief Maps a buffer
 *
 * @cmd: Buffer mapping information
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_map(struct cam_mem_mgr_map_cmd_v2 *cmd);

/**
 * @brief: Perform cache ops on the buffer
 *
 * @cmd:   Cache ops information
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_cache_ops(struct cam_mem_cache_ops_cmd *cmd);

/**
 * @brief: Perform cpu access ops on the buffer
 *
 * @cmd:   CPU access ops information
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_cpu_access_op(struct cam_mem_cpu_access_op *cmd);

/**
 * @brief: Provide all supported heap capabilities
 *
 * @heap_mask: Update mask for all supported heaps
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_check_for_supported_heaps(uint64_t *heap_mask);

/**
 * @brief: Initializes the memory manager
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_init(void);

/**
 * @brief:  Tears down the memory manager
 *
 * @return None
 */
void cam_mem_mgr_deinit(void);

#ifdef CONFIG_CAM_PRESIL
/**
 * @brief: Put dma-buf for input dmabuf
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_put_dmabuf_from_fd(uint64_t input_dmabuf);

/**
 * @brief: Create a fd for dma-buf
 *
 * @return Status of operation. Negative in case of error. Zero or
 *       Positive otherwise.
 */
int cam_mem_mgr_get_fd_from_dmabuf(uint64_t input_dmabuf);
#endif /* ifdef CONFIG_CAM_PRESIL */

/**
 * @brief: Copy buffer content to presil mem for all buffers of
 *       iommu handle
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_send_all_buffers_to_presil(int32_t iommu_hdl);

/**
 * @brief: Copy buffer content of single buffer to presil
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_send_buffer_to_presil(int32_t iommu_hdl, int32_t buf_handle);

/**
 * @brief: Copy back buffer content of single buffer from
 *       presil
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_retrieve_buffer_from_presil(int32_t buf_handle,
	uint32_t buf_size, uint32_t offset, int32_t iommu_hdl);

/**
 * @brief: Dump mem mgr info into user buffer
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_mem_mgr_dump_user(struct cam_dump_req_cmd *dump_req);
#endif /* _CAM_MEM_MGR_H_ */
