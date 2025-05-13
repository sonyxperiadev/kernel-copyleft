/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_MEMORY_H_
#define _MSM_VIDC_MEMORY_H_

#include "msm_vidc_internal.h"

struct msm_vidc_core;
struct msm_vidc_inst;

#define MSM_MEM_POOL_PACKET_SIZE 1024

struct msm_memory_dmabuf {
	struct list_head       list;
	struct dma_buf        *dmabuf;
	u32                    refcount;
};

enum msm_memory_pool_type {
	MSM_MEM_POOL_BUFFER  = 0,
	MSM_MEM_POOL_ALLOC_MAP,
	MSM_MEM_POOL_TIMESTAMP,
	MSM_MEM_POOL_DMABUF,
	MSM_MEM_POOL_PACKET,
	MSM_MEM_POOL_BUF_TIMER,
	MSM_MEM_POOL_BUF_STATS,
	MSM_MEM_POOL_MAX,
};

struct msm_memory_alloc_header {
	struct list_head       list;
	u32                    type;
	bool                   busy;
	void                  *buf;
};

struct msm_memory_pool {
	u32                    size;
	char                  *name;
	struct list_head       free_pool; /* list of struct msm_memory_alloc_header */
	struct list_head       busy_pool; /* list of struct msm_memory_alloc_header */
};

void *msm_vidc_pool_alloc(struct msm_vidc_inst *inst,
			  enum msm_memory_pool_type type);
void msm_vidc_pool_free(struct msm_vidc_inst *inst, void *vidc_buf);
int msm_vidc_pools_init(struct msm_vidc_inst *inst);
void msm_vidc_pools_deinit(struct msm_vidc_inst *inst);

#define call_mem_op(c, op, ...)                  \
	(((c) && (c)->mem_ops && (c)->mem_ops->op) ? \
	((c)->mem_ops->op(__VA_ARGS__)) : 0)

struct msm_vidc_memory_ops {
	struct dma_buf *(*dma_buf_get)(struct msm_vidc_inst *inst,
				       int fd);
	void (*dma_buf_put)(struct msm_vidc_inst *inst,
			    struct dma_buf *dmabuf);
	void (*dma_buf_put_completely)(struct msm_vidc_inst *inst,
				       struct msm_memory_dmabuf *buf);
	struct dma_buf_attachment *(*dma_buf_attach)(struct msm_vidc_core *core,
						     struct dma_buf *dbuf, struct device *dev);
	int (*dma_buf_detach)(struct msm_vidc_core *core, struct dma_buf *dbuf,
			      struct dma_buf_attachment *attach);
	struct sg_table
		*(*dma_buf_map_attachment)(struct msm_vidc_core *core,
					   struct dma_buf_attachment *attach);
	int (*dma_buf_unmap_attachment)(struct msm_vidc_core *core,
					struct dma_buf_attachment *attach,
					struct sg_table *table);
	int (*memory_alloc_map)(struct msm_vidc_core *core,
				struct msm_vidc_mem *mem);
	int (*memory_unmap_free)(struct msm_vidc_core *core,
				 struct msm_vidc_mem *mem);
	int (*mem_dma_map_page)(struct msm_vidc_core *core,
				struct msm_vidc_mem *mem);
	int (*mem_dma_unmap_page)(struct msm_vidc_core *core,
				  struct msm_vidc_mem *mem);
	u32 (*buffer_region)(struct msm_vidc_inst *inst,
			     enum msm_vidc_buffer_type buffer_type);
	int (*iommu_map)(struct msm_vidc_core *core,
			 struct msm_vidc_mem *mem);
	int (*iommu_unmap)(struct msm_vidc_core *core,
			   struct msm_vidc_mem *mem);
};

const struct msm_vidc_memory_ops *get_mem_ops(void);

#endif // _MSM_VIDC_MEMORY_H_
