/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _LINUX_SI_CORE_XTS_H__
#define _LINUX_SI_CORE_XTS_H__

#include <linux/dma-buf.h>
#include <linux/firmware/qcom/si_object.h>

#define SI_CORE_MEM_OBJ_SHARE     0
#define SI_CORE_MEM_OBJ_LEND      1

/* When UNCACHED flag is used the use-case owner must perform explicit cache
 * maintenance operations, i.e. cache flush and invalidate on the memory
 * shared with QTEE.
 */
#define SI_CORE_MEM_OBJ_UNCACHED  16

struct si_object *init_si_mem_object_user(struct dma_buf *dma_buf,
	void (*release)(void *), void *private);
#if IS_ENABLED(CONFIG_QCOM_SI_CORE_MEM_FFA)
struct si_object *init_si_mem_object_pages(size_t *size, uint64_t tag,
					   uint64_t flags, void (*release)(void *),
					   void *private);
#else
static inline struct si_object *init_si_mem_object_pages(size_t *size, uint64_t tag,
							 uint64_t flags,
							 void (*release)(void *),
							 void *private)
{
	return ERR_PTR(-EOPNOTSUPP);
}
#endif
struct si_object *init_si_mem_object_sg(struct sg_table *sgt, uint64_t tag,
					uint32_t flags, void (*release)(void *),
					void *private);

int dma_map_mem_object(struct si_object *object, unsigned int nents);
void dma_unmap_mem_object(struct si_object *object, unsigned int nents);
/* For 'mem_object_to_dma_buf' and 'is_mem_object' caller should own the 'object',
 * (i.e. someone should have already called '__get_si_object').
 */

int is_mem_object(struct si_object *object);
int early_map_memory_obj(struct si_object *object);
struct dma_buf *mem_object_to_dma_buf(struct si_object *object);
struct sg_table *mem_object_to_sgt(struct si_object *object);

#endif /* _LINUX_SI_CORE_XTS_H__ */
