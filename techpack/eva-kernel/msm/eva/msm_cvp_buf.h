/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_CVP_BUF_H_
#define _MSM_CVP_BUF_H_

#include <linux/poll.h>
#include <linux/types.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/refcount.h>
#include <media/msm_eva_private.h>
#include "cvp_comm_def.h"

#define MAX_FRAME_BUFFER_NUMS 40
#define MAX_DMABUF_NUMS 64
#define IS_CVP_BUF_VALID(buf, smem) \
	((buf->size <= smem->size) && \
	(buf->size <= smem->size - buf->offset))

struct msm_cvp_inst;
struct msm_cvp_platform_resources;
struct msm_cvp_list;

enum smem_cache_ops {
	SMEM_CACHE_CLEAN,
	SMEM_CACHE_INVALIDATE,
	SMEM_CACHE_CLEAN_INVALIDATE,
};

enum smem_prop {
	SMEM_UNCACHED = 0x1,
	SMEM_CACHED = 0x2,
	SMEM_SECURE = 0x4,
	SMEM_CDSP = 0x8,
	SMEM_NON_PIXEL = 0x10,
	SMEM_PIXEL = 0x20,
	SMEM_CAMERA = 0x40,
	SMEM_PERSIST = 0x100,
};

struct msm_cvp_list {
	struct list_head list;
	struct mutex lock;
};

static inline void INIT_MSM_CVP_LIST(struct msm_cvp_list *mlist)
{
	mutex_init(&mlist->lock);
	INIT_LIST_HEAD(&mlist->list);
}

static inline void DEINIT_MSM_CVP_LIST(struct msm_cvp_list *mlist)
{
	mutex_destroy(&mlist->lock);
}

struct cvp_dma_mapping_info {
	struct device *dev;
	struct iommu_domain *domain;
	struct sg_table *table;
	struct dma_buf_attachment *attach;
	struct dma_buf *buf;
	void *cb_info;
};

struct msm_cvp_smem {
	struct list_head list;
	atomic_t refcount;
	struct dma_buf *dma_buf;
	void *kvaddr;
	u32 device_addr;
	dma_addr_t dma_handle;
	u32 size;
	u32 bitmap_index;
	u32 flags;
	u32 pkt_type;
	u32 buf_idx;
	u32 fd;
	struct cvp_dma_mapping_info mapping_info;
};

struct msm_cvp_wncc_buffer {
	u32 fd;
	u32 iova;
	u32 size;
};

struct cvp_dmamap_cache {
	unsigned long usage_bitmap;
	struct mutex lock;
	struct msm_cvp_smem *entries[MAX_DMABUF_NUMS];
	unsigned int nr;
};

static inline void INIT_DMAMAP_CACHE(struct cvp_dmamap_cache *cache)
{
	mutex_init(&cache->lock);
	cache->usage_bitmap = 0;
	cache->nr = 0;
}

static inline void DEINIT_DMAMAP_CACHE(struct cvp_dmamap_cache *cache)
{
	mutex_destroy(&cache->lock);
	cache->usage_bitmap = 0;
	cache->nr = 0;
}

#define INPUT_FENCE_BITMASK 0x1
#define OUTPUT_FENCE_BITMASK 0x2

/* Track source of dma_buf allocator/owner */
enum buffer_owner {
	DRIVER,		/* Allocated by KMD, for CPU driver */
	CLIENT,		/* Allocated by Client (DSP or CPU) */
	DSP,		/* Allocated by KMD, for DSP driver */
	MAX_OWNER
};

struct cvp_internal_buf {
	struct list_head list;
	s32 fd;
	u32 size;
	u32 offset;
	u32 type;
	u32 index;
	u64 ktid;
	enum buffer_owner ownership;
	struct msm_cvp_smem *smem;
};

struct msm_cvp_frame {
	struct list_head list;
	struct cvp_internal_buf bufs[MAX_FRAME_BUFFER_NUMS];
	u32 nr;
	u64 ktid;
	u32 pkt_type;
};

struct cvp_frame_bufs {
	u64 ktid;
	u32 nr;
	struct msm_cvp_smem smem[MAX_FRAME_BUFFER_NUMS];
};

struct wncc_oob_buf {
	u32 bitmap_idx;
	struct eva_kmd_oob_wncc *buf;
};

#define NUM_WNCC_BUFS 8
struct cvp_oob_pool {
	struct mutex lock;
	bool allocated;
	u32 used_bitmap;
	struct eva_kmd_oob_wncc *bufs[NUM_WNCC_BUFS];
};

extern struct cvp_oob_pool wncc_buf_pool;

void print_cvp_buffer(u32 tag, const char *str,
		struct msm_cvp_inst *inst,
		struct cvp_internal_buf *cbuf);
void print_cvp_buffer(u32 tag, const char *str,
		struct msm_cvp_inst *inst,
		struct cvp_internal_buf *cbuf);
void print_client_buffer(u32 tag, const char *str,
		struct msm_cvp_inst *inst,
		struct eva_kmd_buffer *cbuf);
int print_smem(u32 tag, const char *str,
		struct msm_cvp_inst *inst,
		struct msm_cvp_smem *smem);

/*Kernel DMA buffer and IOMMU mapping functions*/
int msm_cvp_smem_alloc(size_t size, u32 align, int map_kernel,
			void  *res, struct msm_cvp_smem *smem);
int msm_cvp_smem_free(struct msm_cvp_smem *smem);
struct context_bank_info *msm_cvp_smem_get_context_bank(
				struct msm_cvp_platform_resources *res,
				unsigned int flags);
int msm_cvp_map_smem(struct msm_cvp_inst *inst,
			struct msm_cvp_smem *smem,
			const char *str);
int msm_cvp_unmap_smem(struct msm_cvp_inst *inst,
			struct msm_cvp_smem *smem,
			const char *str);
struct dma_buf *msm_cvp_smem_get_dma_buf(int fd);
void msm_cvp_smem_put_dma_buf(void *dma_buf);
int msm_cvp_smem_cache_operations(struct dma_buf *dbuf,
				enum smem_cache_ops cache_op,
				unsigned long offset,
				unsigned long size);
int msm_cvp_map_ipcc_regs(u32 *iova);
int msm_cvp_unmap_ipcc_regs(u32 iova);

/* CVP driver internal buffer management functions*/
struct cvp_internal_buf *cvp_allocate_arp_bufs(struct msm_cvp_inst *inst,
					u32 buffer_size);
int cvp_release_arp_buffers(struct msm_cvp_inst *inst);
int msm_cvp_map_buf_dsp(struct msm_cvp_inst *inst,
			struct eva_kmd_buffer *buf);
int msm_cvp_unmap_buf_dsp(struct msm_cvp_inst *inst,
			struct eva_kmd_buffer *buf);
int msm_cvp_map_buf_dsp_new(struct msm_cvp_inst *inst,
			struct eva_kmd_buffer *buf,
			int32_t pid,
			uint32_t *iova);
int msm_cvp_unmap_buf_dsp_new(struct msm_cvp_inst *inst,
			struct eva_kmd_buffer *buf);
int msm_cvp_map_buf_wncc(struct msm_cvp_inst* inst,
			struct eva_kmd_buffer* buf);
int msm_cvp_unmap_buf_wncc(struct msm_cvp_inst* inst,
			struct eva_kmd_buffer* buf);
int msm_cvp_proc_oob(struct msm_cvp_inst* inst,
			struct eva_kmd_hfi_packet* in_pkt);
void msm_cvp_cache_operations(struct msm_cvp_smem *smem,
			u32 type, u32 offset, u32 size);
int msm_cvp_unmap_user_persist(struct msm_cvp_inst *inst,
			struct eva_kmd_hfi_packet *in_pkt,
			unsigned int offset, unsigned int buf_num);
int msm_cvp_map_user_persist(struct msm_cvp_inst *inst,
			struct eva_kmd_hfi_packet *in_pkt,
			unsigned int offset, unsigned int buf_num);
int msm_cvp_map_frame(struct msm_cvp_inst *inst,
		struct eva_kmd_hfi_packet *in_pkt,
		unsigned int offset, unsigned int buf_num);
void msm_cvp_unmap_frame(struct msm_cvp_inst *inst, u64 ktid);
int msm_cvp_register_buffer(struct msm_cvp_inst *inst,
		struct eva_kmd_buffer *buf);
int msm_cvp_unregister_buffer(struct msm_cvp_inst *inst,
		struct eva_kmd_buffer *buf);
int msm_cvp_session_deinit_buffers(struct msm_cvp_inst *inst);
void msm_cvp_print_inst_bufs(struct msm_cvp_inst *inst, bool log);
int cvp_allocate_dsp_bufs(struct msm_cvp_inst *inst,
			struct cvp_internal_buf *buf,
			u32 buffer_size,
			u32 secure_type);
int cvp_release_dsp_buffers(struct msm_cvp_inst *inst,
			struct cvp_internal_buf *buf);
#endif
