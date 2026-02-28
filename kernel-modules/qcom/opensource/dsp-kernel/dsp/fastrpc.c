// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2011-2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2018, Linaro Limited
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-resv.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/sort.h>
#include <linux/of_platform.h>
#include <linux/iommu.h>
#include <linux/msm_dma_iommu_mapping.h>
#include <linux/genalloc.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm_qos.h>
#include "../include/uapi/misc/fastrpc.h"
#include "../include/linux/fastrpc.h"
#include <linux/of_reserved_mem.h>
#include <linux/cred.h>
#include <linux/arch_topology.h>
#include <linux/mem-buf.h>
#include <linux/soc/qcom/pdr.h>
#include <soc/qcom/secure_buffer.h>
#include "fastrpc_shared.h"
#include <linux/platform_device.h>

#define CREATE_TRACE_POINTS
#include "fastrpc_trace.h"

/* Struct to hold globally used variables */
struct fastrpc_common {
	/* global lock  to access channel context */
	spinlock_t glock;

	/* global copy of channel contexts */
	struct fastrpc_channel_ctx *gctx[FASTRPC_DEV_MAX];

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
	struct dentry *debugfs_global_file;
#endif
};

/* Global fastrpc driver object */
struct fastrpc_common g_frpc;

static inline int64_t getnstimediff(struct timespec64 *start)
{
	int64_t ns;
	struct timespec64 ts, b;

	ktime_get_real_ts64(&ts);
	b = timespec64_sub(ts, *start);
	ns = timespec64_to_ns(&b);
	return ns;
}


static int fastrpc_device_create(struct fastrpc_user *fl);

/*
* fastrpc_update_gctx() - copy channel context to a global structure.
* @arg1: channel context.
* @arg2: flag to enable or disable copy
*
*/

void fastrpc_update_gctx(struct fastrpc_channel_ctx *cctx, int flag)
{
	struct fastrpc_channel_ctx **ctx = &g_frpc.gctx[cctx->domain_id];

	if (flag == 1)
		*ctx = cctx;
	else
		*ctx = NULL;
}


static void dma_buf_unmap_attachment_wrap(struct fastrpc_map *map)
{
	trace_fastrpc_dma_unmap(map->fl->cctx->domain_id, map->phys,
		map->size, map->fd);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0))
	dma_buf_unmap_attachment_unlocked(map->attach, map->table,
		DMA_BIDIRECTIONAL);
#else
	dma_buf_unmap_attachment(map->attach, map->table,
		DMA_BIDIRECTIONAL);
#endif
}
static int dma_buf_map_attachment_wrap(struct fastrpc_map *map)
{
	int err = 0;
	struct sg_table *table;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0))
	table = dma_buf_map_attachment_unlocked(map->attach,
		DMA_BIDIRECTIONAL);
	if (IS_ERR(table)) {
		err = PTR_ERR(table);
		return err;
	}
#else
	table = dma_buf_map_attachment(map->attach,
		DMA_BIDIRECTIONAL);
	if (IS_ERR(table)) {
		err = PTR_ERR(table);
		return err;
	}
#endif
	map->table = table;

	return 0;
}

static inline void __fastrpc_dma_map_free(struct fastrpc_map *map)
{
	dma_buf_unmap_attachment_wrap(map);
	dma_buf_detach(map->buf, map->attach);
	dma_buf_put(map->buf);
}

static void __fastrpc_free_map(struct fastrpc_map *map)
{
	struct fastrpc_user *fl = NULL;
	struct fastrpc_smmu *smmucb = NULL;

	if (!map)
		return;

	fl = map->fl;
	if (fl) {
		spin_lock(&map->fl->lock);
		list_del(&map->node);
		spin_unlock(&map->fl->lock);
	}

	if (map->table) {
		if (fl && (map->attr & FASTRPC_ATTR_SECUREMAP)) {
			struct qcom_scm_vmperm perm;
			int vmid = fl->cctx->vmperms[0].vmid;
			u64 src_perms = BIT(QCOM_SCM_VMID_HLOS) | BIT(vmid);
			int err = 0;

			perm.vmid = QCOM_SCM_VMID_HLOS;
			perm.perm = QCOM_SCM_PERM_RWX;
			err = qcom_scm_assign_mem(map->phys, map->size,
				&src_perms, &perm, 1);
			if (err) {
				dev_err(fl->cctx->dev,
					"Failed to assign memory phys 0x%llx size 0x%llx err %d",
						map->phys, map->size, err);
				goto free_map;
			}
		}
		/*
		 * FASTRPC_MAP_FD_NOMAP and FASTRPC_ATTR_NOMAP
		 * is not mapped on SMMU CB device
		 */
		if (map->attr & FASTRPC_ATTR_NOMAP ||
			map->flags == FASTRPC_MAP_FD_NOMAP) {
			__fastrpc_dma_map_free(map);
		} else {
			smmucb = map->smmucb;
			mutex_lock(&smmucb->map_mutex);
			if (!smmucb->dev) {
				mutex_unlock(&smmucb->map_mutex);
				goto free_map;
			}
			__fastrpc_dma_map_free(map);
			smmucb->allocatedbytes -= SMMU_ALIGN(map->size);
			mutex_unlock(&smmucb->map_mutex);
		}
	}

free_map:
	kfree(map);
}

static void fastrpc_free_map(struct kref *ref)
{
	struct fastrpc_map *map = NULL;

	map = container_of(ref, struct fastrpc_map, refcount);
	__fastrpc_free_map(map);
}

static void fastrpc_map_put(struct fastrpc_map *map)
{
	if (map)
		kref_put(&map->refcount, fastrpc_free_map);
}

static int fastrpc_map_get(struct fastrpc_map *map)
{
	if (!map)
		return -ENOENT;

	return kref_get_unless_zero(&map->refcount) ? 0 : -ENOENT;
}


static int fastrpc_map_lookup(struct fastrpc_user *fl, int fd,
			    u64 va, u64 len, struct dma_buf *buf, int mflags,
			    struct fastrpc_map **ppmap, bool take_ref)
{
	struct fastrpc_pool_ctx *sess = fl->sctx;
	struct fastrpc_map *map = NULL;
	int ret = -ENOENT;

	if (mflags == ADSP_MMAP_DMA_BUFFER) {
		if (!buf)
			return ret;
	} else {
		/* Fetch DMA buffer from fd */
		buf = dma_buf_get(fd);
		if (IS_ERR(buf))
			return PTR_ERR(buf);
	}

	spin_lock(&fl->lock);
		list_for_each_entry(map, &fl->maps, node) {
			/*
			 * Retrieve the map if the DMA buffer and fd match. For
			 * duplicated fds with the same DMA buffer, create separate
			 * maps for each duplicated fd.
			 */
			if (map->buf == buf && map->fd == fd)
				goto map_found;
		}
	goto error;

map_found:
	if (take_ref) {
		ret = fastrpc_map_get(map);
		if (ret) {
			dev_dbg(sess->smmucb[DEFAULT_SMMU_IDX].dev,
				"%s: Failed to get map fd=%d ret=%d\n",
				__func__, fd, ret);
				goto error;
		}
	}
	*ppmap = map;
	ret = 0;

error:
	spin_unlock(&fl->lock);
	/* Drop the DMA buf ref except for the DMA bus driver */
	if (mflags != ADSP_MMAP_DMA_BUFFER)
		dma_buf_put(buf);
	return ret;
}

static bool fastrpc_get_persistent_buf(struct fastrpc_user *fl,
		size_t size, int buf_type, struct fastrpc_buf **obuf)
{
	u32 i = 0;
	bool found = false;
	struct fastrpc_buf *buf = NULL;

	spin_lock(&fl->lock);
	/*
	 * Persistent header buffer can be used only if
	 * metadata length is less than 1 page size.
	 */
	if (!fl->num_pers_hdrs || buf_type != METADATA_BUF || size > PAGE_SIZE) {
		spin_unlock(&fl->lock);
		return found;
	}

	for (i = 0; i < fl->num_pers_hdrs; i++) {
		buf = &fl->hdr_bufs[i];
		/* If buffer not in use, then assign it for requested alloc */
		if (!buf->in_use) {
			buf->in_use = true;
			*obuf = buf;
			found = true;
			break;
		}
	}
	spin_unlock(&fl->lock);
	return found;
}

static void __fastrpc_dma_buf_free(struct fastrpc_buf *buf)
{
	trace_fastrpc_dma_free(buf->domain_id, buf->phys, buf->size);
	dma_free_coherent(buf->dev, buf->size, buf->virt,
			  FASTRPC_PHYS(buf->phys));
	kfree(buf);
}

static void __fastrpc_buf_free(struct fastrpc_buf *buf)
{
	struct fastrpc_smmu *smmucb = NULL;

	/* REMOTEHEAP_BUF is not mapped on SMMU device */
	if (buf->type == REMOTEHEAP_BUF) {
		__fastrpc_dma_buf_free(buf);
	} else {
		smmucb = buf->smmucb;
		mutex_lock(&smmucb->map_mutex);
		if (smmucb->dev) {
			smmucb->allocatedbytes -= SMMU_ALIGN(buf->size);
			__fastrpc_dma_buf_free(buf);
		}
		mutex_unlock(&smmucb->map_mutex);
	}
}

static void fastrpc_cached_buf_list_add(struct fastrpc_buf *buf)
{
	struct fastrpc_user *fl = buf->fl;

	if (buf->size < FASTRPC_MAX_CACHE_BUF_SIZE) {
		spin_lock(&fl->lock);
		if (fl->num_cached_buf > FASTRPC_MAX_CACHED_BUFS) {
			spin_unlock(&fl->lock);
			goto skip_buf_cache;
		}

		list_add_tail(&buf->node, &fl->cached_bufs);
		fl->num_cached_buf++;
		buf->type = -1;
		spin_unlock(&fl->lock);
		return;
	}

skip_buf_cache:
	__fastrpc_buf_free(buf);
	return;
}

static void fastrpc_buf_free(struct fastrpc_buf *buf, bool cache)
{
	struct fastrpc_user *fl = buf->fl;

	if (buf->in_use) {
		/* Don't free persistent header buf. Just mark as available */
		spin_lock(&fl->lock);
		buf->in_use = false;
		spin_unlock(&fl->lock);
		return;
	}
	if (cache)
		fastrpc_cached_buf_list_add(buf);
	else
		__fastrpc_buf_free(buf);
}

static inline bool fastrpc_get_cached_buf(struct fastrpc_user *fl,
		size_t size, int buf_type, struct fastrpc_buf **obuf)
{
	bool found = false;
	struct fastrpc_buf *buf, *n, *cbuf = NULL;

	if (buf_type == USER_BUF || buf_type == REMOTEHEAP_BUF)
		return found;

	/* find the smallest buffer that fits in the cache */
	spin_lock(&fl->lock);
	list_for_each_entry_safe(buf, n, &fl->cached_bufs, node) {
		if (buf->size >= size && (!cbuf || cbuf->size > buf->size))
			cbuf = buf;
	}
	if (cbuf) {
		list_del_init(&cbuf->node);
		fl->num_cached_buf--;
	}
	spin_unlock(&fl->lock);
	if (cbuf) {
		cbuf->type = buf_type;
		*obuf = cbuf;
		found = true;
	}

	return found;
}

static void fastrpc_buf_list_free(struct fastrpc_user *fl,
	struct list_head *buf_list, bool is_cached_buf)
{
	struct fastrpc_buf *buf = NULL, *n = NULL, *free = NULL;

	do {
		free = NULL;
		spin_lock(&fl->lock);
		list_for_each_entry_safe(buf, n, buf_list, node) {
			list_del(&buf->node);
			if (is_cached_buf)
				fl->num_cached_buf--;
			free = buf;
			break;
		}
		spin_unlock(&fl->lock);
		if (free)
			fastrpc_buf_free(free, false);
	} while (free);
}

/*
 * Free list of buffers donated for rootheap
 * @arg1: channel context.
 * @arg2: rootpd session context
 *
 * Returns void
 */
static void fastrpc_rootheap_buf_list_free(struct fastrpc_channel_ctx *cctx)
{
	struct fastrpc_buf *buf = NULL, *n = NULL, *free = NULL;
	unsigned long flags = 0;

	/* Return if no rootheap buffers were donated */
	if (!cctx->rootheap_bufs.num)
		return;

	do {
		free = NULL;
		spin_lock_irqsave(&cctx->lock, flags);
		list_for_each_entry_safe(buf, n, &cctx->rootheap_bufs.list, node) {
			list_del(&buf->node);
			cctx->rootheap_bufs.num--;
			free = buf;
			break;
		}
		spin_unlock_irqrestore(&cctx->lock, flags);

		if (free)
			__fastrpc_buf_free(free);
	} while (free);
}

static inline void __fastrpc_dma_alloc(struct fastrpc_buf *buf)
{
	buf->virt = dma_alloc_coherent(buf->dev, buf->size,
				(dma_addr_t *)&buf->phys, GFP_KERNEL);
}

static int __fastrpc_buf_alloc(struct fastrpc_user *fl,
		struct fastrpc_smmu *smmucb, u32 domain_id,
		u64 size, struct fastrpc_buf **obuf, u32 buf_type)
{
	struct fastrpc_buf *buf;
	struct timespec64 start_ts, end_ts;

	/* Check if the size is valid (non-zero and within integer range) */
	if (!size || size > INT_MAX)
		return -EFAULT;
	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	INIT_LIST_HEAD(&buf->attachments);
	INIT_LIST_HEAD(&buf->node);
	mutex_init(&buf->lock);

	buf->fl = fl;
	buf->virt = NULL;
	buf->phys = 0;
	buf->size = size;
	buf->raddr = 0;
	buf->type = buf_type;
	buf->domain_id = domain_id;
	ktime_get_boottime_ts64(&start_ts);

	/* REMOTEHEAP_BUF is allocated using cctx device */
	if (buf_type == REMOTEHEAP_BUF) {
		buf->dev = fl->cctx->dev;
		/*
		 * Do not acquire spinlock with IRQ disabled
		 * as "dma_alloc_coherent" locks a mutex
		 */
		if (fl->cctx->dev)
			__fastrpc_dma_alloc(buf);
	} else {
		buf->dev = smmucb->dev;
		buf->smmucb = smmucb;
		mutex_lock(&smmucb->map_mutex);
		if (smmucb->dev)
			__fastrpc_dma_alloc(buf);
		if (buf->virt) {
			smmucb->allocatedbytes += SMMU_ALIGN(buf->size);
			buf->phys += ((u64)smmucb->sid << 32);
		}
		mutex_unlock(&smmucb->map_mutex);
	}

	if (!buf->virt) {
		mutex_destroy(&buf->lock);
		kfree(buf);
		return -ENOMEM;
	}

	*obuf = buf;

	trace_fastrpc_dma_alloc(domain_id, (uint64_t)buf->phys, buf->size,
								(unsigned long)buf->type, 0);
	ktime_get_boottime_ts64(&end_ts);
	buf->alloc_time = timespec64_sub(end_ts, start_ts);
	return 0;
}

static int fastrpc_buf_alloc(struct fastrpc_user *fl,
			struct fastrpc_smmu *smmucb, u64 size,
			u32 buf_type, struct fastrpc_buf **obuf)
{
	int ret;

	if (fastrpc_get_persistent_buf(fl, size, buf_type, obuf))
		return 0;
	if (fastrpc_get_cached_buf(fl, size, buf_type, obuf))
		return 0;
	ret = __fastrpc_buf_alloc(fl, smmucb, fl->cctx->domain_id,
						size, obuf, buf_type);
	if (ret == -ENOMEM) {
		fastrpc_buf_list_free(fl, &fl->cached_bufs, true);
		ret = __fastrpc_buf_alloc(fl, smmucb, fl->cctx->domain_id,
					size, obuf, buf_type);
		if (ret)
			return ret;
	}

	return ret;
}

/**
 * fastrpc_smmu_device_lookup() -
 * Function to get IOMMU device index from the session pool
 * @arg1: Fastrpc pool session
 * @arg2: Allocation/map buffer size
 * @arg3: Current IOMMU pool device index
 *
 * Starting from current IOMMU pool device index, function
 * finds a IOMMU CB where there is enough virtual space available
 * to allocate/map buffer size.
 *
 * Return: Returns IOMMU pool device index,
 *         where virtual space is available
 */
static u32 fastrpc_smmu_device_lookup(struct fastrpc_pool_ctx *sess,
						u64 size, u32 smmuidx)
{
	struct fastrpc_smmu *smmucb = NULL;

	for (; smmuidx < sess->smmucount; smmuidx++) {
		smmucb = &sess->smmucb[smmuidx];
		/*
		 * Use the SMMU index device, if the SMMU pool
		 * alloc ranges are not defined.
		 */
		if (smmucb->maxallocsize == 0)
			break;

		if (size >= smmucb->minallocsize &&
			size < (smmucb->totalbytes - smmucb->allocatedbytes))
			break;
	}

	return smmuidx;
}

/**
 * fastrpc_smmu_buf_alloc() - Allocates memory on IOMMU CB
 * @arg1: Fastrpc user file pointer
 * @arg2: Allocation buffer size
 * @arg3: Allocation buffer type
 * @arg4: Output argument pointer to the fastrpc_buf
 *
 * Return: Returns 0 on success, error code on failure
 */
static int fastrpc_smmu_buf_alloc(struct fastrpc_user *fl, u64 size,
						u32 buf_type, struct fastrpc_buf **obuf)
{
	int err = 0;
	struct fastrpc_pool_ctx *sess = NULL;
	struct fastrpc_smmu *smmucb = NULL;
	u32 smmuidx = DEFAULT_SMMU_IDX;

	sess = fl->sctx;
retry_alloc:
	smmuidx = fastrpc_smmu_device_lookup(sess, size, smmuidx);
	if (smmuidx >= sess->smmucount) {
		dev_err(fl->cctx->dev,
			"%s: No valid smmu context bank found for size 0x%llx\n",
			__func__, size);
		err = -ENOSR;
		return err;
	} else {
		smmucb = &sess->smmucb[smmuidx];
	}

	err = fastrpc_buf_alloc(fl, smmucb, size, buf_type, obuf);
	/*
	 * Retry allocation on next availale IOMMU CB,
	 * if there is no enough virtual space available on current IOMMU CB
	 */
	if (err == -ENOMEM || err == -EINVAL) {
		smmuidx++;
		goto retry_alloc;
	}
	return err;
}

static void fastrpc_channel_ctx_free(struct kref *ref)
{
	struct fastrpc_channel_ctx *cctx;
	int i, j;

	cctx = container_of(ref, struct fastrpc_channel_ctx, refcount);
	mutex_destroy(&cctx->wake_mutex);

	for (i = 0; i < FASTRPC_MAX_SESSIONS; i++)
		for (j = 0; j < cctx->session[i].smmucount; j++)
			mutex_destroy(&cctx->session[i].smmucb[j].map_mutex);
	ida_destroy(&cctx->tgid_frpc_ida);
	kfree(cctx);
}

static void fastrpc_channel_ctx_get(struct fastrpc_channel_ctx *cctx)
{
	kref_get(&cctx->refcount);
}

void fastrpc_channel_ctx_put(struct fastrpc_channel_ctx *cctx)
{
	kref_put(&cctx->refcount, fastrpc_channel_ctx_free);
}

static void fastrpc_context_free(struct kref *ref)
{
	struct fastrpc_invoke_ctx *ctx;
	struct fastrpc_channel_ctx *cctx;
	unsigned long flags;
	int i;

	ctx = container_of(ref, struct fastrpc_invoke_ctx, refcount);
	cctx = ctx->cctx;

	mutex_lock(&ctx->fl->map_mutex);
	for (i = 0; i < ctx->nbufs; i++)
		fastrpc_map_put(ctx->maps[i]);
	mutex_unlock(&ctx->fl->map_mutex);

	if (ctx->buf)
		fastrpc_buf_free(ctx->buf, true);

	if (ctx->fl->profile)
		kfree(ctx->perf);

	spin_lock_irqsave(&cctx->lock, flags);
	idr_remove(&cctx->ctx_idr, FASTRPC_GET_IDR_FROM_CTXID(ctx->ctxid));
	spin_unlock_irqrestore(&cctx->lock, flags);

	trace_fastrpc_context_free((uint64_t)ctx,
		ctx->ctxid, ctx->handle, ctx->sc);

	kfree(ctx->maps);
	kfree(ctx->olaps);
	kfree(ctx->args);
	kfree(ctx->outbufs);
	kfree(ctx);

	fastrpc_channel_ctx_put(cctx);
}

// static void fastrpc_context_get(struct fastrpc_invoke_ctx *ctx)
// {
	// kref_get(&ctx->refcount);
// }

static void fastrpc_context_put(struct fastrpc_invoke_ctx *ctx)
{
	kref_put(&ctx->refcount, fastrpc_context_free);
}

// static void fastrpc_context_put_wq(struct work_struct *work)
// {
	// struct fastrpc_invoke_ctx *ctx =
			// container_of(work, struct fastrpc_invoke_ctx, put_work);

	// fastrpc_context_put(ctx);
// }

#define CMP(aa, bb) ((aa) == (bb) ? 0 : (aa) < (bb) ? -1 : 1)

static u32 sorted_lists_intersection(u32 *listA,
		u32 lenA, u32 *listB, u32 lenB)
{
	u32 i = 0, j = 0;

	while (i < lenA && j < lenB) {
		if (listA[i] < listB[j])
			i++;
		else if (listA[i] > listB[j])
			j++;
		else
			return listA[i];
	}
	return 0;
}

static int uint_cmp_func(const void *p1, const void *p2)
{
	u32 a1 = *((u32 *)p1);
	u32 a2 = *((u32 *)p2);

	return CMP(a1, a2);
}

static int olaps_cmp(const void *a, const void *b)
{
	struct fastrpc_buf_overlap *pa = (struct fastrpc_buf_overlap *)a;
	struct fastrpc_buf_overlap *pb = (struct fastrpc_buf_overlap *)b;
	/* sort with lowest starting buffer first */
	int st = CMP(pa->start, pb->start);
	/* sort with highest ending buffer first */
	int ed = CMP(pb->end, pa->end);

	return st == 0 ? ed : st;
}

static int fastrpc_get_buff_overlaps(struct fastrpc_invoke_ctx *ctx)
{
	u64 max_end = 0;
	int i;
	struct device *dev = ctx->fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;

	for (i = 0; i < ctx->nbufs; ++i) {
		ctx->olaps[i].start = ctx->args[i].ptr;
		/* Check the overflow for user buffer */
		if (ctx->olaps[i].start > (ULLONG_MAX - ctx->args[i].length)) {
			dev_dbg(dev,
				"user passed invalid non ion buffer addr 0x%llx, size %llx\n",
				ctx->args[i].ptr, ctx->args[i].length);
			return -EFAULT;
		}
		ctx->olaps[i].end = ctx->olaps[i].start + ctx->args[i].length;
		ctx->olaps[i].raix = i;
	}

	sort(ctx->olaps, ctx->nbufs, sizeof(*ctx->olaps), olaps_cmp, NULL);

	for (i = 0; i < ctx->nbufs; ++i) {
		/* Falling inside previous range */
		if (ctx->olaps[i].start < max_end) {
			ctx->olaps[i].mstart = max_end;
			ctx->olaps[i].mend = ctx->olaps[i].end;
			ctx->olaps[i].offset = max_end - ctx->olaps[i].start;

			if (ctx->olaps[i].end > max_end) {
				max_end = ctx->olaps[i].end;
			} else {
				ctx->olaps[i].mend = 0;
				ctx->olaps[i].mstart = 0;
			}

		} else  {
			ctx->olaps[i].mend = ctx->olaps[i].end;
			ctx->olaps[i].mstart = ctx->olaps[i].start;
			ctx->olaps[i].offset = 0;
			max_end = ctx->olaps[i].end;
		}
	}
	return 0;
}

static struct fastrpc_invoke_ctx *fastrpc_context_alloc(
			struct fastrpc_user *user, u32 kernel, u32 sc,
			struct fastrpc_enhanced_invoke *invoke)
{
	struct fastrpc_channel_ctx *cctx = user->cctx;
	struct fastrpc_invoke_ctx *ctx = NULL;
	unsigned long flags;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&ctx->node);
	ctx->fl = user;
	ctx->nscalars = REMOTE_SCALARS_LENGTH(sc);
	ctx->nbufs = REMOTE_SCALARS_INBUFS(sc) +
		     REMOTE_SCALARS_OUTBUFS(sc);

	if (ctx->nscalars) {
		ctx->maps = kcalloc(ctx->nscalars,
				    sizeof(*ctx->maps), GFP_KERNEL);
		if (!ctx->maps) {
			ret = -ENOMEM;
			goto err_alloc;
		}
		ctx->olaps = kcalloc(ctx->nscalars,
				    sizeof(*ctx->olaps), GFP_KERNEL);
		if (!ctx->olaps) {
			ret = -ENOMEM;
			goto err_alloc;
		}
		ctx->args = kcalloc(ctx->nscalars,
				    sizeof(*ctx->args), GFP_KERNEL);
		if (!ctx->args) {
			ret = -ENOMEM;
			goto err_alloc;
		}
		if (!kernel) {
			if (copy_from_user((void *)ctx->args,
					(void __user *)(uintptr_t)invoke->inv.args,
					ctx->nscalars * sizeof(*ctx->args))) {
				ret = -EFAULT;
				goto err_alloc;
			}
		} else {
			memcpy((void *)ctx->args,
					(void *)(uintptr_t)invoke->inv.args,
					ctx->nscalars * sizeof(*ctx->args));
		}
		invoke->inv.args = (__u64)ctx->args;
		ret = fastrpc_get_buff_overlaps(ctx);
		if (ret)
			goto err_alloc;
	}

	/* Released in fastrpc_context_put() */
	fastrpc_channel_ctx_get(cctx);

	ctx->crc = (u32 *)(uintptr_t)invoke->crc;
	ctx->perf_dsp = (u64 *)(uintptr_t)invoke->perf_dsp;
	ctx->perf_kernel = (u64 *)(uintptr_t)invoke->perf_kernel;
	if (ctx->fl->profile) {
		ctx->perf = kzalloc(sizeof(*(ctx->perf)), GFP_KERNEL);
		if (!ctx->perf) {
			ret = -ENOMEM;
			goto err_perf_alloc;
		}
		ctx->perf->tid = ctx->fl->tgid;
	}
	ctx->handle = invoke->inv.handle;
	ctx->sc = sc;
	ctx->retval = -1;
	ctx->pid = current->pid;
	ctx->tgid = user->tgid;
	ctx->cctx = cctx;
	ctx->rsp_flags = NORMAL_RESPONSE;
	ctx->is_work_done = false;
	init_completion(&ctx->work);
	// INIT_WORK(&ctx->put_work, fastrpc_context_put_wq);

	spin_lock(&user->lock);
	list_add_tail(&ctx->node, &user->pending);
	spin_unlock(&user->lock);

	spin_lock_irqsave(&cctx->lock, flags);
	ret = idr_alloc_cyclic(&cctx->ctx_idr, ctx, 1,
			       FASTRPC_CTX_MAX, GFP_ATOMIC);
	if (ret < 0) {
		spin_unlock_irqrestore(&cctx->lock, flags);
		goto err_idr;
	}
	cctx->jobid++;
	ctx->ctxid = FASTRPC_PACK_JOBID_IN_CTXID(ctx->ctxid, cctx->jobid);
	ctx->ctxid = FASTRPC_PACK_IDR_IN_CTXID(ctx->ctxid, ret);
	spin_unlock_irqrestore(&cctx->lock, flags);

	trace_fastrpc_context_alloc((uint64_t)ctx,
				ctx->ctxid, ctx->handle, ctx->sc);
	kref_init(&ctx->refcount);

	return ctx;
err_idr:
	spin_lock(&user->lock);
	list_del(&ctx->node);
	spin_unlock(&user->lock);
err_perf_alloc:
	fastrpc_channel_ctx_put(cctx);
err_alloc:
	kfree(ctx->maps);
	kfree(ctx->olaps);
	kfree(ctx->args);
	kfree(ctx);

	return ERR_PTR(ret);
}

static struct fastrpc_invoke_ctx *fastrpc_context_restore_interrupted(
			struct fastrpc_user *fl, struct fastrpc_invoke *inv)
{
	struct fastrpc_invoke_ctx *ctx = NULL, *ictx = NULL, *n;

	spin_lock(&fl->lock);
	list_for_each_entry_safe(ictx, n, &fl->interrupted, node) {
		if (ictx->pid == current->pid) {
			if (inv->sc != ictx->sc || ictx->fl != fl) {
				dev_err(ictx->fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev,
					"interrupted sc (0x%x) or fl (%pK) does not match with invoke sc (0x%x) or fl (%pK)\n",
					ictx->sc, ictx->fl, inv->sc, fl);
				spin_unlock(&fl->lock);
				return ERR_PTR(-EINVAL);
			} else {
				ctx = ictx;
				list_del(&ctx->node);
				list_add_tail(&ctx->node, &fl->pending);
			}
			break;
		}
	}
	spin_unlock(&fl->lock);
	return ctx;
}

static void fastrpc_context_save_interrupted(
			struct fastrpc_invoke_ctx *ctx)
{
	trace_fastrpc_context_interrupt(ctx->cctx->domain_id, (uint64_t)ctx,
					ctx->msg.ctx, ctx->msg.handle, ctx->msg.sc);
	spin_lock(&ctx->fl->lock);
	list_del(&ctx->node);
	list_add_tail(&ctx->node, &ctx->fl->interrupted);
	spin_unlock(&ctx->fl->lock);
}

static struct sg_table *
fastrpc_map_dma_buf(struct dma_buf_attachment *attachment,
		    enum dma_data_direction dir)
{
	struct fastrpc_dma_buf_attachment *a = attachment->priv;
	struct sg_table *table;
	int ret;

	table = &a->sgt;

	ret = dma_map_sgtable(attachment->dev, table, dir, 0);
	if (ret)
		table = ERR_PTR(ret);
	return table;
}

static void fastrpc_unmap_dma_buf(struct dma_buf_attachment *attach,
				  struct sg_table *table,
				  enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, table, dir, 0);
}

static void fastrpc_release(struct dma_buf *dmabuf)
{
	struct fastrpc_buf *buffer = dmabuf->priv;

	fastrpc_buf_free(buffer, false);
}

static int fastrpc_dma_buf_attach(struct dma_buf *dmabuf,
				  struct dma_buf_attachment *attachment)
{
	struct fastrpc_dma_buf_attachment *a;
	struct fastrpc_buf *buffer = dmabuf->priv;
	int ret;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	ret = dma_get_sgtable(buffer->dev, &a->sgt, buffer->virt,
			      FASTRPC_PHYS(buffer->phys), buffer->size);
	if (ret < 0) {
		dev_err(buffer->dev, "failed to get scatterlist from DMA API\n");
		kfree(a);
		return -EINVAL;
	}

	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->node);
	attachment->priv = a;

	mutex_lock(&buffer->lock);
	list_add(&a->node, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return 0;
}

static void fastrpc_dma_buf_detatch(struct dma_buf *dmabuf,
				    struct dma_buf_attachment *attachment)
{
	struct fastrpc_dma_buf_attachment *a = attachment->priv;
	struct fastrpc_buf *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	list_del(&a->node);
	mutex_unlock(&buffer->lock);
	sg_free_table(&a->sgt);
	kfree(a);
}

static int fastrpc_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct fastrpc_buf *buf = dmabuf->priv;

	iosys_map_set_vaddr(map, buf->virt);

	return 0;
}

static int fastrpc_mmap(struct dma_buf *dmabuf,
			struct vm_area_struct *vma)
{
	struct fastrpc_buf *buf = dmabuf->priv;
	size_t size = vma->vm_end - vma->vm_start;

	return dma_mmap_coherent(buf->dev, vma, buf->virt,
				 FASTRPC_PHYS(buf->phys), size);
}

static const struct dma_buf_ops fastrpc_dma_buf_ops = {
	.attach = fastrpc_dma_buf_attach,
	.detach = fastrpc_dma_buf_detatch,
	.map_dma_buf = fastrpc_map_dma_buf,
	.unmap_dma_buf = fastrpc_unmap_dma_buf,
	.mmap = fastrpc_mmap,
	.vmap = fastrpc_vmap,
	.release = fastrpc_release,
};

static struct fastrpc_pool_ctx *fastrpc_session_alloc(
				struct fastrpc_user *fl, bool secure)
{

	struct fastrpc_pool_ctx *session = NULL, *isess = NULL;
	struct fastrpc_channel_ctx *cctx = fl->cctx;
	unsigned long flags;
	bool sharedcb = fl->sharedcb;
	int pd_type = fl->pd_type;
	int i;

	if (!cctx->dev)
		return session;

	/*
	 * If PD type is configured for context banks in device tree,
	 * use CPZ_USERPD, to allocate secure context bank type.
	 */
	if (secure && cctx->pd_type) {
		pd_type = CPZ_USERPD;
		sharedcb = true;
	} else if (secure)
		/* Legacy case, where pd_type is not configured in device tree */
		pd_type = DEFAULT_UNUSED;

	/*
	 * If session allocated already and PD type is configured for non secure,
	 * use same session.
	 */
	if (fl->sctx && !secure)
		return fl->sctx;

	spin_lock_irqsave(&cctx->lock, flags);
	for (i = 0; i < cctx->sesscount; i++) {
		/*
		 * Session is chosen based on following conditions:
		 * 1. If session is SID pooled (smmucount > 1), then any number of applications
		 *    can use session, else only one application (usecount == 0) is allowed to
		 *    use session
		 * AND
		 * 2. SMMU CB should always be valid, should not have been unregistered.
		 * AND
		 * 3. If process is secure usecase (CPZ usecase), then session also
		 *    should have secure parameter set.
		 * AND
		 * 4. If process needs to share CB (sensors usecases share one CB), then
		 *    session also should have sharedcb parameter set.
		 * AND
		 * 5. If pd_type is configured, then process pd_type needs to match with
		 *    session pd_type, else pd_type check is ignored
		 */
		isess = &cctx->session[i];
		if ((isess->usecount == 0 || isess->smmucount > 1) &&
			isess->smmucb[DEFAULT_SMMU_IDX].valid &&
			isess->secure == secure &&
			isess->sharedcb == sharedcb &&
			(pd_type == DEFAULT_UNUSED || isess->pd_type == pd_type || secure)) {
			session = isess;
			/*
			 * Increment number of apps using session.
			 * Will be max 1 for sessions that don't have
			 * pooled context banks or a shared context bank.
			 */
			session->usecount++;
			break;
		} else if((isess->usecount == 0 || isess->smmucount > 1) &&
 			isess->smmucb[DEFAULT_SMMU_IDX].valid &&
 			isess->secure == secure &&
                        (pd_type == USERPD && isess->pd_type == SENSORS_STATICPD && isess->usershare > 0)){
			session = isess;
						/*
						 * Increment number of apps using session.
						 * Will be max 1 for sessions that don't have
						 * pooled context banks or a shared context bank.
						 */
						session->usecount++;
						break;
		}
	}
	spin_unlock_irqrestore(&cctx->lock, flags);

	return session;
}

static void fastrpc_session_free(struct fastrpc_channel_ctx *cctx,
				 struct fastrpc_pool_ctx *session)
{
	unsigned long flags;

	spin_lock_irqsave(&cctx->lock, flags);
	if (session->usecount > 0)
		session->usecount--;
	spin_unlock_irqrestore(&cctx->lock, flags);
}

static void fastrpc_pm_awake(struct fastrpc_user *fl,
					u32 is_secure_channel)
{
	struct fastrpc_channel_ctx *cctx = fl->cctx;
	struct wakeup_source *wake_source = NULL;

	/*
	 * Vote with PM to abort any suspend in progress and
	 * keep system awake for specified timeout
	 */
	if (is_secure_channel)
		wake_source = cctx->wake_source_secure;
	else
		wake_source = cctx->wake_source;

	if (wake_source)
		pm_wakeup_ws_event(wake_source, fl->ws_timeout, true);
}

static void fastrpc_pm_relax(struct fastrpc_user *fl,
					u32 is_secure_channel)
{
	struct fastrpc_channel_ctx *cctx = fl->cctx;
	struct wakeup_source *wake_source = NULL;

	if (!fl->wake_enable)
		return;

	mutex_lock(&cctx->wake_mutex);
	if (is_secure_channel)
		wake_source = cctx->wake_source_secure;
	else
		wake_source = cctx->wake_source;

	if (wake_source)
		__pm_relax(wake_source);
	mutex_unlock(&cctx->wake_mutex);
}

static int get_buffer_attr(struct dma_buf *buf, bool *exclusive_access)
{
	const int *vmids_list = NULL;
	const int  *perms = NULL;
	int err = 0;
	int vmids_list_len = 0;
	*exclusive_access = false;

	err = mem_buf_dma_buf_get_vmperm(buf, &vmids_list, &perms, &vmids_list_len);
	if (err)
		return err;
	/*
	 * If one VM has access to buffer and is the current VM,
	 * then VM has exclusive access to buffer
	 */
	if (vmids_list_len == 1 && vmids_list[0] == mem_buf_current_vmid())
		*exclusive_access = true;

	return err;
}

static int set_buffer_secure_type(struct fastrpc_map *map)
{
	int err = 0;
	bool exclusive_access = false;
	struct device *dev = map->fl->cctx->dev;

	err = get_buffer_attr(map->buf, &exclusive_access);
	if (err) {
		dev_err(dev, "failed to obtain buffer attributes for fd %d ret %d\n", map->fd, err);
		return -EBADFD;
	}
	/*
	 * Secure buffers would always be owned by multiple VMs.
	 * If current VM is the exclusive owner of a buffer, it is considered non-secure.
	 * In PVM:
	 *	- CPZ buffers are secure
	 *	- All other buffers are non-secure
	 * In TVM:
	 *	- Since it is a secure environment by default, there are no explicit "secure" buffers
	 *	- All buffers are marked "non-secure"
	 */
#if IS_ENABLED(CONFIG_QCOM_FASTRPC_TRUSTED)
	map->secure = 0;
#else
	map->secure = (exclusive_access) ? 0 : 1;
#endif

	return err;
}

static int fastrpc_map_create(struct fastrpc_user *fl, int fd,
			      u64 va, struct dma_buf *buf, u64 len,
			      u32 attr, int mflags, struct fastrpc_map **ppmap,
				  bool take_ref)
{
	struct fastrpc_pool_ctx *sess = NULL;
	struct fastrpc_map *map = NULL;
	struct scatterlist *sgl = NULL;
	int err = 0, sgl_index = 0;
	struct device *dev = NULL;
	struct fastrpc_smmu *smmucb = NULL;
	struct fastrpc_pool_ctx *secsctx = NULL;
	u32 smmuidx = DEFAULT_SMMU_IDX;

	if (!fastrpc_map_lookup(fl, fd, va, len, buf, mflags, ppmap, take_ref))
		return 0;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	INIT_LIST_HEAD(&map->node);
	kref_init(&map->refcount);

	map->fl = fl;
	map->fd = fd;
	map->flags = mflags;
	map->len = len;

	if(mflags == ADSP_MMAP_DMA_BUFFER) {
		if (!buf) {
			err = -EFAULT;
			goto get_err;
		}
		map->buf = buf;
		get_dma_buf(map->buf);

	} else {
		map->buf = dma_buf_get(fd);
		if (IS_ERR(map->buf)) {
			err = PTR_ERR(map->buf);
			goto get_err;
		}
	}

	err = set_buffer_secure_type(map);
	if (err)
		goto attach_err;

	if (map->secure && (!(attr & FASTRPC_ATTR_NOMAP || mflags == FASTRPC_MAP_FD_NOMAP))) {
		if (!fl->secsctx) {
			secsctx = fastrpc_session_alloc(fl, true);
			if (!secsctx) {
				dev_err(fl->cctx->dev, "No secure session available\n");
				err = -EBUSY;
				goto attach_err;
			}
			fl->secsctx = secsctx;
		}
		sess = fl->secsctx;
	} else {
		sess = fl->sctx;
	}

map_retry:
	smmuidx = fastrpc_smmu_device_lookup(sess, len, smmuidx);
	if (smmuidx >= sess->smmucount) {
		dev_err(fl->cctx->dev,
			"%s: No valid smmu context bank found for len 0x%llx\n",
			__func__, len);
		err = -ENOSR;
		goto attach_err;
	} else {
		smmucb = &sess->smmucb[smmuidx];
	}

	if (attr & FASTRPC_ATTR_NOMAP || mflags == FASTRPC_MAP_FD_NOMAP) {
		dev = fl->cctx->dev;
	} else {
		dev =  smmucb->dev;
		map->smmucb = smmucb;
	}

	mutex_lock(&smmucb->map_mutex);
	if (!smmucb->dev) {
		err = -ENODEV;
		mutex_unlock(&smmucb->map_mutex);
		goto attach_err;
	}

	map->attach = dma_buf_attach(map->buf, dev);
	if (IS_ERR(map->attach)) {
		dev_err(dev, "Failed to attach dmabuf\n");
		err = PTR_ERR(map->attach);
		mutex_unlock(&smmucb->map_mutex);
		goto attach_err;
	}

	err = dma_buf_map_attachment_wrap(map);
	/*
	 * Retry allocation on next availale IOMMU CB,
	 * if there is no enough virtual space available on current IOMMU CB.
	 * Detach from current IOMMU CB.
	 */
	if (err == -ENOMEM || err == -EINVAL) {
		mutex_unlock(&smmucb->map_mutex);
		dma_buf_detach(map->buf, map->attach);
		smmuidx++;
		goto map_retry;
	} else if (err) {
		mutex_unlock(&smmucb->map_mutex);
		goto map_err;
	}

	if (attr & FASTRPC_ATTR_SECUREMAP) {
		map->phys = sg_phys(map->table->sgl);
		for_each_sg(map->table->sgl, sgl, map->table->nents,
			sgl_index)
			map->size += sg_dma_len(sgl);
		map->va = (void *) (uintptr_t) va;
		smmucb->allocatedbytes += SMMU_ALIGN(map->size);
	} else if (attr & FASTRPC_ATTR_NOMAP || mflags == FASTRPC_MAP_FD_NOMAP){

		map->phys = sg_dma_address(map->table->sgl);
		map->size = sg_dma_len(map->table->sgl);
		map->va = (void *) (uintptr_t) va;
	} else {
		map->phys = sg_dma_address(map->table->sgl);
		map->phys += ((u64)smmucb->sid << 32);
		for_each_sg(map->table->sgl, sgl, map->table->nents,
			sgl_index)
			map->size += sg_dma_len(sgl);
		map->va = (void *) (uintptr_t) va;
		smmucb->allocatedbytes += SMMU_ALIGN(map->size);
	}

	trace_fastrpc_dma_map(map->fl->cctx->domain_id, map->fd, map->phys,
		map->size, map->len, map->attach->dma_map_attrs, map->flags);
	mutex_unlock(&smmucb->map_mutex);

	if (attr & FASTRPC_ATTR_SECUREMAP) {
		/*
		 * If subsystem VMIDs are defined in DTSI, then do
		 * hyp_assign from HLOS to those VM(s)
		 */
		u64 src_perms = BIT(QCOM_SCM_VMID_HLOS);
		struct qcom_scm_vmperm dst_perms[2] = {0};

		dst_perms[0].vmid = QCOM_SCM_VMID_HLOS;
		dst_perms[0].perm = QCOM_SCM_PERM_RW;
		dst_perms[1].vmid = fl->cctx->vmperms[0].vmid;
		dst_perms[1].perm = QCOM_SCM_PERM_RWX;
		err = qcom_scm_assign_mem(map->phys, (u64)map->size, &src_perms, dst_perms, 2);
		if (err) {
			dev_err(smmucb->dev,
			"Failed to assign memory with phys 0x%llx size 0x%llx err %d",
			map->phys, map->size, err);
			goto assign_err;
		}
	}
	map->attr = attr;
	spin_lock(&fl->lock);
	list_add_tail(&map->node, &fl->maps);
	spin_unlock(&fl->lock);
	*ppmap = map;

	return 0;

assign_err:
	dma_buf_unmap_attachment_wrap(map);
map_err:
	dma_buf_detach(map->buf, map->attach);
attach_err:
	dma_buf_put(map->buf);
get_err:
	kfree(map);

	return err;
}

/*
 * Fastrpc payload buffer with metadata looks like:
 *
 * >>>>>>  START of METADATA <<<<<<<<<
 * +---------------------------------+
 * |           Arguments             |
 * | type:(union fastrpc_remote_arg)|
 * |             (0 - N)             |
 * +---------------------------------+
 * |         Invoke Buffer list      |
 * | type:(struct fastrpc_invoke_buf)|
 * |           (0 - N)               |
 * +---------------------------------+
 * |         Page info list          |
 * | type:(struct fastrpc_phy_page)  |
 * |             (0 - N)             |
 * +---------------------------------+
 * |         Optional info           |
 * |(can be specific to SoC/Firmware)|
 * +---------------------------------+
 * >>>>>>>>  END of METADATA <<<<<<<<<
 * +---------------------------------+
 * |         Inline ARGS             |
 * |            (0-N)                |
 * +---------------------------------+
 */

static int fastrpc_get_meta_size(struct fastrpc_invoke_ctx *ctx)
{
	int size = 0;

	size = (sizeof(struct fastrpc_remote_buf) +
		sizeof(struct fastrpc_invoke_buf) +
		sizeof(struct fastrpc_phy_page)) * ctx->nscalars +
		sizeof(u64) * FASTRPC_MAX_FDLIST +
		sizeof(u32) * FASTRPC_MAX_CRCLIST +
		sizeof(u32) + sizeof(u64) * FASTRPC_DSP_PERF_LIST;

	return size;
}

static u64 fastrpc_get_payload_size(struct fastrpc_invoke_ctx *ctx, int metalen)
{
	u64 size = 0, len;
	int oix;

	size = ALIGN(metalen, FASTRPC_ALIGN);
	for (oix = 0; oix < ctx->nbufs; oix++) {
		int i = ctx->olaps[oix].raix;

		if (ctx->args[i].fd == 0 || ctx->args[i].fd == -1) {

			if (ctx->olaps[oix].offset == 0)
				size = ALIGN(size, FASTRPC_ALIGN);

			len = (ctx->olaps[oix].mend - ctx->olaps[oix].mstart);
			/* Check the overflow for payload */
			if (size > (ULLONG_MAX - len))
				return 0;
			size += len;
		}
	}

	return size;
}

static int fastrpc_create_maps(struct fastrpc_invoke_ctx *ctx)
{
	struct device *dev = ctx->fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	struct fastrpc_channel_ctx *cctx = ctx->fl->cctx;
	int i, err;

	for (i = 0; i < ctx->nscalars; ++i) {
		bool take_ref = true;
		int mflags = 0;

		if (ctx->args[i].fd == 0 || ctx->args[i].fd == -1 ||
		   (i >= ctx->nbufs && cctx->dsp_attributes[DMA_HANDLE_REVERSE_RPC_CAP]) ||
                    ctx->args[i].length == 0)
			continue;

		if (i >= ctx->nbufs) {
			take_ref = false;
			/* Set the DMA handle mapping flag for DMA handles */
			mflags = FASTRPC_MAP_LEGACY_DMA_HANDLE;
		}
		mutex_lock(&ctx->fl->map_mutex);
		err = fastrpc_map_create(ctx->fl, ctx->args[i].fd, (u64)ctx->args[i].ptr, NULL,
			 ctx->args[i].length, ctx->args[i].attr, mflags, &ctx->maps[i], take_ref);
		mutex_unlock(&ctx->fl->map_mutex);
		if (err) {
			dev_err(dev, "Error Creating map %d\n", err);
			return -EINVAL;
		}

	}
	return 0;
}

static struct fastrpc_invoke_buf *fastrpc_invoke_buf_start(union fastrpc_remote_arg *pra, int len)
{
	return (struct fastrpc_invoke_buf *)(&pra[len]);
}

static struct fastrpc_phy_page *fastrpc_phy_page_start(struct fastrpc_invoke_buf *buf, int len)
{
	return (struct fastrpc_phy_page *)(&buf[len]);
}

/*
 * Validate the user provided buffer against the map buffer and
 * retrieve the offset if the buffer is valid.
 * @arg1: invoke context
 * @arg2: current index passed during ctx map iteration
 * @arg3: output argument pointer to get the offset
 *
 * Return: returns 0 on success, error code on failure.
 */
static int fastrpc_get_buffer_offset(struct fastrpc_invoke_ctx *ctx, int index,
				u64 *offset)
{
	u64 addr = (u64)ctx->args[index].ptr & PAGE_MASK, vm_start = 0, vm_end = 0;
	struct vm_area_struct *vma;
	struct file *vma_file = NULL;
	int err = 0;

	if (!(ctx->maps[index]->attr & FASTRPC_ATTR_NOVA)) {
		mmap_read_lock(current->mm);
		vma = find_vma(current->mm, ctx->args[index].ptr);
		if (vma) {
			vm_start = vma->vm_start;
			vm_end = vma->vm_end;
			vma_file = vma->vm_file;
		}
		mmap_read_unlock(current->mm);
		/*
		 * Error out if:
		 * 1. The DMA buffer file does not match the VMA file
		 *    retrieved from the user provided buffer va
		 * 2. The user provided buffer’s address does not fall
		 *    within the VMA address range
		 * 3. The length of the user provided buffer does not
		 *    fall within the VMA address range
		 */
		if ((ctx->maps[index]->buf->file != vma_file) ||
			(addr < vm_start || addr + ctx->args[index].length > vm_end ||
			(addr - vm_start) + ctx->args[index].length >
				ctx->maps[index]->size)) {
			err = -EFAULT;
			goto bail;
		}
		*offset = addr - vm_start;
	}
	return 0;

bail:
	dev_err(ctx->fl->cctx->dev,
			"Invalid buffer fd %d addr 0x%llx len 0x%llx vm start 0x%llx vm end 0x%llx IPA 0x%llx size 0x%llx, err %d\n",
			ctx->maps[index]->fd, ctx->args[index].ptr,
			ctx->args[index].length, vm_start, vm_end,
			ctx->maps[index]->phys, ctx->maps[index]->size, err);
	return err;
}

static int fastrpc_get_args(u32 kernel, struct fastrpc_invoke_ctx *ctx)
{
	struct device *dev = ctx->fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	union fastrpc_remote_arg *rpra;
	struct fastrpc_invoke_buf *list;
	struct fastrpc_phy_page *pages;
	int inbufs, outbufs, i, oix, err = 0;
	u64 len, rlen, pkt_size, outbufslen;
	u64 pg_start, pg_end;
	u64 *perf_counter = NULL;
	uintptr_t args;
	int metalen;

	if (ctx->fl->profile)
		perf_counter = (u64 *)ctx->perf + PERF_COUNT;

	inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);
	metalen = fastrpc_get_meta_size(ctx);
	pkt_size = fastrpc_get_payload_size(ctx, metalen);
	if (!pkt_size) {
		dev_err(dev, "invalid payload size for handle 0x%x, sc 0x%x\n",
			ctx->handle, ctx->sc);
		return -EFAULT;
	}

	PERF(ctx->fl->profile, GET_COUNTER(perf_counter, PERF_MAP),
	err = fastrpc_create_maps(ctx);
	if (err)
		return err;
	PERF_END);

	ctx->msg_sz = metalen;

	err = fastrpc_smmu_buf_alloc(ctx->fl, pkt_size, METADATA_BUF, &ctx->buf);
	if (err)
		return err;

	memset(ctx->buf->virt, 0, pkt_size);
	rpra = ctx->buf->virt;
	list = fastrpc_invoke_buf_start(rpra, ctx->nscalars);
	pages = fastrpc_phy_page_start(list, ctx->nscalars);
	args = (uintptr_t)ctx->buf->virt + metalen;
	rlen = pkt_size - metalen;
	ctx->rpra = rpra;

	for (oix = 0; oix < ctx->nbufs; ++oix) {
		u64 mlen;
		u64 offset = 0;

		i = ctx->olaps[oix].raix;
		len = ctx->args[i].length;

		rpra[i].buf.pv = 0;
		rpra[i].buf.len = len;
		list[i].num = len ? 1 : 0;
		list[i].pgidx = i;

		if (!len)
			continue;

		if (ctx->maps[i]) {

			PERF(ctx->fl->profile, GET_COUNTER(perf_counter, PERF_MAP),

			rpra[i].buf.pv = (u64) ctx->args[i].ptr;
			pages[i].addr = ctx->maps[i]->phys;
			/* validate user passed buffer length with map buffer size */
			if (len > ctx->maps[i]->size) {
				err = -EFAULT;
				dev_err(dev,
					"Invalid buffer addr 0x%llx len 0x%llx IPA 0x%llx size 0x%llx fd %d\n",
					ctx->args[i].ptr, len, ctx->maps[i]->phys,
					ctx->maps[i]->size, ctx->maps[i]->fd);
				goto bail;
			}
			err = fastrpc_get_buffer_offset(ctx, i, &offset);
			if (err)
				goto bail;

			pages[i].addr += offset;
			pg_start = (ctx->args[i].ptr & PAGE_MASK) >> PAGE_SHIFT;
			pg_end = ((ctx->args[i].ptr + len - 1) & PAGE_MASK) >>
				  PAGE_SHIFT;
			pages[i].size = (pg_end - pg_start + 1) * PAGE_SIZE;
			PERF_END);
		} else {
			PERF(ctx->fl->profile, GET_COUNTER(perf_counter, PERF_COPY),
			if (ctx->olaps[oix].offset == 0) {
				rlen -= ALIGN(args, FASTRPC_ALIGN) - args;
				args = ALIGN(args, FASTRPC_ALIGN);
			}

			mlen = ctx->olaps[oix].mend - ctx->olaps[oix].mstart;

			if (mlen > COPY_BUF_WARN_LIMIT)
				dev_dbg(dev, "user passed non ion buffer size 0x%llx, mend 0x%llx mstart 0x%llx, sc 0x%x\n",
					mlen, ctx->olaps[oix].mend, ctx->olaps[oix].mstart, ctx->sc);

			if (rlen < mlen)
				goto bail;

			rpra[i].buf.pv = args - ctx->olaps[oix].offset;
			pages[i].addr = ctx->buf->phys -
					ctx->olaps[oix].offset +
					(pkt_size - rlen);
			pages[i].addr = pages[i].addr &	PAGE_MASK;

			pg_start = (rpra[i].buf.pv & PAGE_MASK) >> PAGE_SHIFT;
			pg_end = ((rpra[i].buf.pv + len - 1) & PAGE_MASK) >> PAGE_SHIFT;
			pages[i].size = (pg_end - pg_start + 1) * PAGE_SIZE;
			args = args + mlen;
			rlen -= mlen;
			PERF_END);
		}

		if (i < inbufs && !ctx->maps[i]) {
			void *dst = (void *)(uintptr_t)rpra[i].buf.pv;
			void *src = (void *)(uintptr_t)ctx->args[i].ptr;
			PERF(ctx->fl->profile, GET_COUNTER(perf_counter, PERF_COPY),

			if (!kernel) {
				if (copy_from_user(dst, (void __user *)src, len)) {
					dev_err(dev, "invalid buffer length 0x%llx\n", len);
					err = -EFAULT;
					goto bail;
				}
			} else {
				memcpy(dst, src, len);
			}
			PERF_END);
		}
	}

	for (i = ctx->nbufs; i < ctx->nscalars; ++i) {
		list[i].num = ctx->args[i].length ? 1 : 0;
		list[i].pgidx = i;
		if (ctx->maps[i]) {
			/* It is possible that map is created using mflag
			 * is FASTRPC_MAP_LEGACY_DMA_HANDLE and take_ref
			 * is false. Check if map is still exist or is
			 * being freed as take_ref is false
			 */
			mutex_lock(&ctx->fl->map_mutex);
			if (!fastrpc_map_lookup(ctx->fl, ctx->args[i].fd,
				 0, 0, NULL, 0 , &ctx->maps[i], false)) {
				pages[i].addr = ctx->maps[i]->phys;
				pages[i].size = ctx->maps[i]->size;
			}
			mutex_unlock(&ctx->fl->map_mutex);
		}
		rpra[i].dma.fd = ctx->args[i].fd;
		rpra[i].dma.len = ctx->args[i].length;
		rpra[i].dma.offset = (u64) ctx->args[i].ptr;
	}
	outbufslen = sizeof(struct fastrpc_remote_buf) * outbufs;
	ctx->outbufs = kzalloc(outbufslen, GFP_KERNEL);
	if (!ctx->outbufs) {
		err = -ENOMEM;
		goto bail;
	}
	memcpy(ctx->outbufs, rpra + inbufs, outbufslen);

bail:
	if (err)
		dev_err(dev, "Error: get invoke args failed:%d\n", err);

	return err;
}

static int fastrpc_put_args(struct fastrpc_invoke_ctx *ctx,
			    u32 kernel)
{
	union fastrpc_remote_arg *rpra = ctx->rpra;
	struct fastrpc_user *fl = ctx->fl;
	struct fastrpc_map *mmap = NULL;
	struct fastrpc_invoke_buf *list;
	struct fastrpc_phy_page *pages;
	u64 *fdlist, *perf_dsp_list;
	u32 *crclist, *poll;
	int i, inbufs, outbufs, handles, perferr;

	inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);
	handles = REMOTE_SCALARS_INHANDLES(ctx->sc) + REMOTE_SCALARS_OUTHANDLES(ctx->sc);
	list = fastrpc_invoke_buf_start(rpra, ctx->nscalars);
	pages = fastrpc_phy_page_start(list, ctx->nscalars);
	fdlist = (u64 *)(pages + inbufs + outbufs + handles);
	crclist = (u32 *)(fdlist + FASTRPC_MAX_FDLIST);
	poll = (u32 *)(crclist + FASTRPC_MAX_CRCLIST);
	perf_dsp_list = (u64 *)(poll + 1);

	for (i = inbufs; i < ctx->nbufs; ++i) {
		if (!ctx->maps[i]) {
			int j = i - inbufs;
			void *src = (void *)(uintptr_t)ctx->outbufs[j].buf.pv;
			void *dst = (void *)(uintptr_t)ctx->args[i].ptr;
			u64 len = ctx->outbufs[j].buf.len;

			if (!kernel) {
				if (copy_to_user((void __user *)dst, src, len))
					return -EFAULT;
			} else {
				memcpy(dst, src, len);
			}
		}
	}

	for (i = 0; i < FASTRPC_MAX_FDLIST; i++) {
		if (!fdlist[i])
			break;
		mutex_lock(&fl->map_mutex);
		if (!fastrpc_map_lookup(fl, (int)fdlist[i], 0, 0, NULL, 0, &mmap, false))
			/* Validate the map flags for DMA handles and skip freeing map if invalid */
			if (mmap->flags == FASTRPC_MAP_LEGACY_DMA_HANDLE) {
				/* Allow DMA handle maps to free only once */
				mmap->flags = 0;
				fastrpc_map_put(mmap);
			}
		mutex_unlock(&fl->map_mutex);
	}
	if (ctx->crc && crclist && rpra) {
		if (copy_to_user((void __user *)ctx->crc, crclist, FASTRPC_MAX_CRCLIST * sizeof(u32)))
			return -EFAULT;
	}
	if (ctx->perf_dsp && perf_dsp_list) {
		if (0 != (perferr = copy_to_user((void __user *)ctx->perf_dsp, perf_dsp_list, FASTRPC_DSP_PERF_LIST * sizeof(u64)))) {
			pr_err("failed to copy perf data %d\n", perferr);
		}
	}
	return 0;
}

static s64 get_timestamp_in_ns(void)
{
	s64 ns = 0;
	struct timespec64 ts;

	ktime_get_boottime_ts64(&ts);
	ns = timespec64_to_ns(&ts);
	return ns;
}

static void fastrpc_update_txmsg_buf(struct fastrpc_channel_ctx *chan,
				struct fastrpc_msg *msg, int rpmsg_send_err, s64 ns)
{
	unsigned long flags = 0;
	u32 tx_index = 0;
	struct fastrpc_tx_msg *tx_msg = NULL;

	spin_lock_irqsave(&(chan->gmsg_log.tx_lock), flags);

	tx_index = chan->gmsg_log.tx_index;
	tx_msg = &(chan->gmsg_log.tx_msgs[tx_index]);

	memcpy(&tx_msg->msg, msg, sizeof(struct fastrpc_msg));
	tx_msg->rpmsg_send_err = rpmsg_send_err;
	tx_msg->ns = ns;

	tx_index++;
	chan->gmsg_log.tx_index =
		(tx_index > (GLINK_MSG_HISTORY_LEN - 1)) ? 0 : tx_index;

	spin_unlock_irqrestore(&(chan->gmsg_log.tx_lock), flags);
}

static void fastrpc_update_rxmsg_buf(struct fastrpc_channel_ctx *chan,
							u64 ctx, int retval, u32 rsp_flags,
							u32 early_wake_time, u32 ver, s64 ns)
{
	unsigned long flags = 0;
	u32 rx_index = 0;
	struct fastrpc_rx_msg *rx_msg = NULL;
	struct fastrpc_invoke_rspv2 *rsp = NULL;

	spin_lock_irqsave(&(chan->gmsg_log.rx_lock), flags);

	rx_index = chan->gmsg_log.rx_index;
	rx_msg = &(chan->gmsg_log.rx_msgs[rx_index]);
	rsp = &rx_msg->rsp;

	rsp->ctx = ctx;
	rsp->retval = retval;
	rsp->flags = rsp_flags;
	rsp->early_wake_time = early_wake_time;
	rsp->version = ver;
	rx_msg->ns = ns;

	rx_index++;
	chan->gmsg_log.rx_index =
		(rx_index > (GLINK_MSG_HISTORY_LEN - 1)) ? 0 : rx_index;

	spin_unlock_irqrestore(&(chan->gmsg_log.rx_lock), flags);
}

/*
 * fastrpc_getpd_msgidx()
 * Function returns msg index that is embedded in rpc msg ctx sent to dsp
 */
static inline int fastrpc_getpd_msgidx(u32 pd_type) {
	if (pd_type == ROOT_PD)
		return 0;
	else if (pd_type == SENSORS_STATICPD)
		return 2;
	else
		return 1;
}

static int fastrpc_invoke_send(struct fastrpc_pool_ctx *sctx,
			       struct fastrpc_invoke_ctx *ctx,
			       u32 kernel, uint32_t handle)
{
	struct fastrpc_channel_ctx *cctx;
	struct fastrpc_user *fl = ctx->fl;
	struct fastrpc_msg *msg = &ctx->msg;
	int ret;

	cctx = fl->cctx;
	msg->pid = fl->tgid_frpc;
	msg->tid = current->pid;

	if (kernel == KERNEL_MSG_WITH_ZERO_PID)
		msg->pid = 0;

	/* Last 2 ctx ID bits, to route glink msg to appropriate PD type on DSP */
	msg->ctx = FASTRPC_PACK_PD_IN_CTXID(ctx->ctxid,
				fastrpc_getpd_msgidx(fl->pd_type));
	msg->handle = handle;
	msg->sc = ctx->sc;
	msg->addr = ctx->buf ? ctx->buf->phys : 0;
	msg->size = roundup(ctx->msg_sz, PAGE_SIZE);
	// fastrpc_context_get(ctx);

	ret = fastrpc_transport_send(cctx, (void *)msg, sizeof(*msg));
	trace_fastrpc_transport_send(cctx->domain_id, (uint64_t)ctx, msg->ctx,
			msg->handle, msg->sc, msg->addr, msg->size);

	// if (ret)
		// fastrpc_context_put(ctx);
	fastrpc_update_txmsg_buf(cctx, msg, ret, get_timestamp_in_ns());

	return ret;

}

static int poll_for_remote_response(struct fastrpc_invoke_ctx *ctx, u32 timeout)
{
	int err = -EIO, ii = 0, jj = 0;
	u32 sc = ctx->sc;
	struct fastrpc_invoke_buf *list;
	struct fastrpc_phy_page *pages;
	u64 *fdlist = NULL;
	u32 *crclist = NULL, *poll = NULL;
	unsigned int inbufs, outbufs, handles;

	/* calculate poll memory location */
	inbufs = REMOTE_SCALARS_INBUFS(sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	handles = REMOTE_SCALARS_INHANDLES(sc) + REMOTE_SCALARS_OUTHANDLES(sc);
	list = fastrpc_invoke_buf_start(ctx->rpra, ctx->nscalars);
	pages = fastrpc_phy_page_start(list, ctx->nscalars);
	fdlist = (u64 *)(pages + inbufs + outbufs + handles);
	crclist = (u32 *)(fdlist + FASTRPC_MAX_FDLIST);
	poll = (u32 *)(crclist + FASTRPC_MAX_CRCLIST);

	/* poll on memory for DSP response. Return failure on timeout */
	for (ii = 0, jj = 0; ii < timeout; ii++, jj++) {
		if (*poll == FASTRPC_EARLY_WAKEUP_POLL) {
			/* Remote processor sent early response */
			err = 0;
			break;
		} else if (*poll == FASTRPC_POLL_RESPONSE) {
			err = 0;
			ctx->is_work_done = true;
			ctx->retval = 0;
			fastrpc_update_rxmsg_buf(ctx->fl->cctx, ctx->msg.ctx, 0,
			POLL_MODE, 0, FASTRPC_RSP_VERSION2, get_timestamp_in_ns());
			break;
		}
		if (jj == FASTRPC_POLL_TIME_MEM_UPDATE) {
			/* Wait for DSP to finish updating poll memory */
			rmb();
			jj = 0;
		}
		udelay(1);
	}
	return err;
}

static inline int fastrpc_wait_for_response(struct fastrpc_invoke_ctx *ctx,
						u32 kernel)
{
	int interrupted = 0;

	if (kernel)
		wait_for_completion(&ctx->work);
	else
		interrupted = wait_for_completion_interruptible(&ctx->work);

	return interrupted;
}

static void fastrpc_wait_for_completion(struct fastrpc_invoke_ctx *ctx,
			int *ptr_interrupted, u32 kernel)
{
	int err = 0, jj = 0;
	bool wait_resp = false;
	u32 wTimeout = FASTRPC_USER_EARLY_HINT_TIMEOUT;
	u32 wakeTime = ctx->early_wake_time;

	do {
		switch (ctx->rsp_flags) {
		/* try polling on completion with timeout */
		case USER_EARLY_SIGNAL:
			/* try wait if completion time is less than timeout */
			/* disable preempt to avoid context switch latency */
			preempt_disable();
			jj = 0;
			wait_resp = false;
			for (; wakeTime < wTimeout && jj < wTimeout; jj++) {
				wait_resp = try_wait_for_completion(&ctx->work);
				if (wait_resp)
					break;
				udelay(1);
			}
			preempt_enable();
			if (!wait_resp) {
				*ptr_interrupted = fastrpc_wait_for_response(ctx, kernel);
				if (*ptr_interrupted || ctx->is_work_done)
					return;
			}
			break;
		/* busy poll on memory for actual job done */
		case EARLY_RESPONSE:
			trace_fastrpc_msg("early_response: poll_begin");
			err = poll_for_remote_response(ctx, FASTRPC_POLL_TIME);
			/* Mark job done if poll on memory successful */
			/* Wait for completion if poll on memory timeout */
			if (!err) {
				ctx->is_work_done = true;
				return;
			}
			trace_fastrpc_msg("early_response: poll_timeout");
			if (!ctx->is_work_done) {
				*ptr_interrupted = fastrpc_wait_for_response(ctx, kernel);
				if (*ptr_interrupted || ctx->is_work_done)
					return;
			}
			break;
		case COMPLETE_SIGNAL:
		case NORMAL_RESPONSE:
			*ptr_interrupted = fastrpc_wait_for_response(ctx, kernel);
			if (*ptr_interrupted || ctx->is_work_done)
				return;
			break;
		case POLL_MODE:
			trace_fastrpc_msg("poll_mode: begin");
			err = poll_for_remote_response(ctx, ctx->fl->poll_timeout);

			/* If polling timed out, move to normal response state */
			if (err) {
				trace_fastrpc_msg("poll_mode: timeout");
				ctx->rsp_flags = NORMAL_RESPONSE;
			} else {
				*ptr_interrupted = 0;
			}
			break;
		default:
			*ptr_interrupted = -EBADR;
			pr_err("unsupported response type:0x%x\n", ctx->rsp_flags);
			break;
		}
	} while (!ctx->is_work_done);
}

static void fastrpc_update_invoke_count(u32 handle, u64 *perf_counter,
					struct timespec64 *invoket)
{
	/* update invoke count for dynamic handles */
	u64 *invcount, *count;
	invcount = GET_COUNTER(perf_counter, PERF_INVOKE);
	if (invcount)
		*invcount += getnstimediff(invoket);

	count = GET_COUNTER(perf_counter, PERF_COUNT);
	if (count)
		*count += 1;
}

static int fastrpc_internal_invoke(struct fastrpc_user *fl,  u32 kernel,
				   struct fastrpc_enhanced_invoke *invoke)
{
	struct fastrpc_invoke_ctx *ctx = NULL;
	struct fastrpc_invoke *inv = &invoke->inv;
	u32 handle, sc;
	int err = 0, perferr = 0, interrupted = 0;
	u64 *perf_counter = NULL;
	struct timespec64 invoket = {0};
	struct device *dev = NULL;

	if (atomic_read(&fl->cctx->teardown))
		return -EPIPE;

	if (fl->profile)
		ktime_get_real_ts64(&invoket);

	if (!fl->sctx)
		return -EINVAL;

	dev = fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	if ((!fl->cctx->dev) || (!dev))
		return -EPIPE;

	handle = inv->handle;
	sc = inv->sc;
	if (handle == FASTRPC_INIT_HANDLE && !kernel) {
		dev_warn_ratelimited(dev,
		"user app trying to send a kernel RPC message (%d)\n",  handle);
		return -EPERM;
	}

	/*
	 * After PDR, for Audio & OIS PD, kill call is still needed to clean
	 * the Audio & OIS PD process in root PD. For Sensors PD, no cleanup
	 * is needed in root PD of DSP.
	 */
	if (IS_PDR(fl) && fl->pd_type == SENSORS_STATICPD) {
		err = -EPIPE;
		return err;
	}

	if (!kernel) {
		ctx = fastrpc_context_restore_interrupted(fl, inv);
		if (IS_ERR(ctx))
			return PTR_ERR(ctx);
		if (ctx) {
			trace_fastrpc_context_restore(ctx->cctx->domain_id, (uint64_t)ctx,
					ctx->msg.ctx, ctx->msg.handle, ctx->msg.sc);
			goto wait;
		}
	}

	trace_fastrpc_msg("context_alloc: begin");
	ctx = fastrpc_context_alloc(fl, kernel, sc, invoke);
	trace_fastrpc_msg("context_alloc: end");
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	if (fl->profile)
		perf_counter = (u64 *)ctx->perf + PERF_COUNT;
	PERF(fl->profile, GET_COUNTER(perf_counter, PERF_GETARGS),
	err = fastrpc_get_args(kernel, ctx);
	if (err)
		goto bail;
	PERF_END);
	trace_fastrpc_msg("get_args: end");
	/* make sure that all CPU memory writes are seen by DSP */
	dma_wmb();
	/* Send invoke buffer to remote dsp */
	PERF(fl->profile, GET_COUNTER(perf_counter, PERF_LINK),
	err = fastrpc_invoke_send(fl->sctx, ctx, kernel, handle);
	if (err)
		goto bail;
	PERF_END);
	trace_fastrpc_msg("invoke_send: end");
wait:
	if (fl->poll_mode &&
		handle > FASTRPC_MAX_STATIC_HANDLE &&
		fl->cctx->domain_id == CDSP_DOMAIN_ID &&
		(fl->pd_type == USERPD || fl->pd_type == USER_UNSIGNEDPD_POOL))
		ctx->rsp_flags = POLL_MODE;

	fastrpc_wait_for_completion(ctx, &interrupted, kernel);
	if (interrupted != 0) {
		trace_fastrpc_msg("wait_for_completion: interrupted");
		err = interrupted;
		goto bail;
	}
	trace_fastrpc_msg("wait_for_completion: end");
	if (!ctx->is_work_done) {
		err = -ETIMEDOUT;
		dev_err(dev, "Error: Invalid workdone state for handle 0x%x, sc 0x%x\n",
			handle, sc);
		goto bail;
	}

	/* make sure that all memory writes by DSP are seen by CPU */
	dma_rmb();
	/* populate all the output buffers with results */
	PERF(fl->profile, GET_COUNTER(perf_counter, PERF_PUTARGS),
	err = fastrpc_put_args(ctx, kernel);
	if (err)
		goto bail;
	PERF_END);
	trace_fastrpc_msg("put_args: end");
	/* Check the response from remote dsp */
	err = ctx->retval;
	if (err)
		goto bail;

bail:
	if (ctx && interrupted == -ERESTARTSYS) {
		fastrpc_context_save_interrupted(ctx);
	} else if (ctx) {
		if (fl->profile && !interrupted)
			fastrpc_update_invoke_count(handle, perf_counter, &invoket);
		if (fl->profile && ctx->perf && handle > FASTRPC_RMID_INIT_MAX) {
			trace_fastrpc_perf_counters(handle, ctx->sc,
			ctx->perf->count, ctx->perf->flush, ctx->perf->map,
			ctx->perf->copy, ctx->perf->link, ctx->perf->getargs,
			ctx->perf->putargs, ctx->perf->invargs,
			ctx->perf->invoke, ctx->perf->tid);
			if (fl->profile && ctx->perf && ctx->perf_kernel)
				if (0 != (perferr = copy_to_user((void __user *)ctx->perf_kernel, ctx->perf, FASTRPC_KERNEL_PERF_LIST * sizeof(u64)))) {
					pr_warn("failed to copy perf data err 0x%x\n", perferr);
				}
		}
		spin_lock(&fl->lock);
		list_del(&ctx->node);
		spin_unlock(&fl->lock);
		fastrpc_context_put(ctx);
		trace_fastrpc_msg("context_free: end");
	}

	if (err)
		dev_dbg(dev, "Error: Invoke Failed %d\n", err);

	return err;
}

static int fastrpc_mem_map_to_dsp(struct fastrpc_user *fl, int fd, int offset,
				u32 flags, u64 va, u64 phys,
				size_t size, uintptr_t *raddr)
{
	struct fastrpc_invoke_args args[4] = { [0 ... 3] = { 0 } };
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_mem_map_req_msg req_msg = { 0 };
	struct fastrpc_mmap_rsp_msg rsp_msg = { 0 };
	struct fastrpc_phy_page pages = { 0 };
	struct device *dev = fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	int err = 0;

	if (!fl) {
		err = -EBADF;
		return err;
	}

	req_msg.pgid = fl->tgid_frpc;
	req_msg.fd = fd;
	req_msg.offset = offset;
	req_msg.vaddrin = va;
	req_msg.flags = flags;
	req_msg.num = sizeof(pages);
	req_msg.data_len = 0;

	args[0].ptr = (u64) (uintptr_t) &req_msg;
	args[0].length = sizeof(req_msg);

	pages.addr = phys;
	pages.size = size;

	args[1].ptr = (u64) (uintptr_t) &pages;
	args[1].length = sizeof(pages);

	args[2].ptr = (u64) (uintptr_t) &pages;
	args[2].length = 0;

	args[3].ptr = (u64) (uintptr_t) &rsp_msg;
	args[3].length = sizeof(rsp_msg);

	ioctl.inv.handle = FASTRPC_INIT_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_MEM_MAP, 3, 1);
	ioctl.inv.args = (__u64)args;
	err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
	if (err) {
		dev_err(dev, "mem mmap error, fd %d, vaddr %llx, size %zx, err 0x%x\n",
			fd, va, size, err);
		return err;
	}
	*raddr = rsp_msg.vaddr;

	return 0;
}

static int fastrpc_create_persistent_headers(struct fastrpc_user *fl)
{
	int err = 0;
	int i = 0;
	u64 virtb = 0;
	struct device *dev = fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	struct fastrpc_buf *hdr_bufs, *buf, *pers_hdr_buf = NULL;
	u32 num_pers_hdrs = 0;
	size_t hdr_buf_alloc_len = 0;

	/*
	 * Pre-allocate memory for persistent header buffers based
	 * on concurrency info passed by user. Upper limit enforced.
	 */
	num_pers_hdrs = FASTRPC_MAX_PERSISTENT_HEADERS;
	hdr_buf_alloc_len = num_pers_hdrs * PAGE_SIZE;

	err = fastrpc_smmu_buf_alloc(fl, hdr_buf_alloc_len,
			METADATA_BUF, &pers_hdr_buf);
	if (err)
		return err;

	virtb = (u64) (uintptr_t)(pers_hdr_buf->virt);
	err = fastrpc_mem_map_to_dsp(fl, -1, 0,
				ADSP_MMAP_PERSIST_HDR, 0, (u64) (uintptr_t)(pers_hdr_buf->phys),
				pers_hdr_buf->size, &pers_hdr_buf->raddr);
	if (err)
		goto err_dsp_map;

	hdr_bufs = kcalloc(num_pers_hdrs, sizeof(struct fastrpc_buf),
				GFP_KERNEL);
	if (!hdr_bufs)
		return -ENOMEM;

	spin_lock(&fl->lock);
	fl->pers_hdr_buf = pers_hdr_buf;
	fl->num_pers_hdrs = num_pers_hdrs;
	fl->hdr_bufs = hdr_bufs;
	for (i = 0; i < num_pers_hdrs; i++) {
		buf = &fl->hdr_bufs[i];
		buf->fl = fl;
		buf->virt = (void *)(virtb + (i * PAGE_SIZE));
		buf->phys = pers_hdr_buf->phys + (i * PAGE_SIZE);
		buf->size = PAGE_SIZE;
		buf->type = pers_hdr_buf->type;
		buf->in_use = false;
	}
	spin_unlock(&fl->lock);

	return 0;
err_dsp_map:
	dev_err(dev, "Warning: failed to map len %zu, flags %d, num headers %u with err %d\n",
			hdr_buf_alloc_len, ADSP_MMAP_PERSIST_HDR,
			num_pers_hdrs, err);
	fastrpc_buf_free(pers_hdr_buf, 0);
	return err;
}

static bool is_session_rejected(struct fastrpc_user *fl, bool unsigned_pd_request)
{
	/* Check if the device node is non-secure and channel is secure */
	if (!fl->is_secure_dev && fl->cctx->secure) {
		/*
		 * Allow untrusted applications to offload only to Unsigned PD when
		 * channel is configured as secure and block untrusted apps on channel
		 * that does not support unsigned PD offload
		 */
		if (!fl->cctx->unsigned_support || !unsigned_pd_request)
			goto reject_session;
	}
	/* Check if untrusted process is trying to offload to signed PD */
	if (fl->untrusted_process && !unsigned_pd_request)
		goto reject_session;

	return false;
reject_session:
	dev_err(fl->cctx->dev, "Error: Untrusted application trying to offload to signed PD");
	return true;
}

static int fastrpc_get_process_gids(struct gid_list *gidlist)
{
	struct group_info *group_info = current_cred()->group_info;
	int i, num_gids;
	u32 *gids = NULL;

	if (!group_info)
		return -EFAULT;

	num_gids = group_info->ngroups + 1;
	gids = kcalloc(num_gids, sizeof(u32), GFP_KERNEL);
	if (!gids)
		return -ENOMEM;

	/* Get the real GID */
	gids[0] = __kgid_val(current_gid());

	/* Get the supplemental GIDs */
	for (i = 1; i < num_gids; i++)
		gids[i] = __kgid_val(group_info->gid[i - 1]);

	sort(gids, num_gids, sizeof(*gids), uint_cmp_func, NULL);
	gidlist->gids = gids;
	gidlist->gidcount = num_gids;

	return 0;
}

static void fastrpc_check_privileged_process(struct fastrpc_user *fl,
				struct fastrpc_init_create *init)
{
	struct gid_list gidlist = {0};
	u32 gid;

	/* disregard any privilege bits from userspace */
	init->attrs &= (~FASTRPC_MODE_PRIVILEGED);

	if (fastrpc_get_process_gids(&gidlist)) {
		dev_info(fl->cctx->dev, "%s failed to get gidlist\n",
				__func__);
		return;
	}

	gid = sorted_lists_intersection(gidlist.gids,
			gidlist.gidcount, fl->cctx->gidlist.gids,
			fl->cctx->gidlist.gidcount);


	if (gid) {
		dev_info(fl->cctx->dev, "%s: %s (PID %d, GID %u) is a privileged process\n",
				__func__, current->comm, fl->tgid, gid);
		init->attrs |= FASTRPC_MODE_PRIVILEGED;
	}
	/* Free memory for gid allocated in fastrpc_get_process_gids */
	kfree(gidlist.gids);
}

int fastrpc_mmap_remove_ssr(struct fastrpc_channel_ctx *cctx)
{
	struct fastrpc_buf *buf, *b, *match;
	unsigned long flags;
	int err = 0;

	do {
		match = NULL;
		spin_lock_irqsave(&cctx->lock, flags);
		list_for_each_entry_safe(buf, b, &cctx->gmaps, node) {
			match = buf;
			list_del(&buf->node);
			break;
		}
		spin_unlock_irqrestore(&cctx->lock, flags);
		if (!match)
			return 0;

		if (cctx->vmcount) {
			u64 src_perms = 0;
			struct qcom_scm_vmperm dst_perms;
			u32 i;

			for (i = 0; i < cctx->vmcount; i++)
				src_perms |= BIT(cctx->vmperms[i].vmid);

			dst_perms.vmid = QCOM_SCM_VMID_HLOS;
			dst_perms.perm = QCOM_SCM_PERM_RWX;
			err = qcom_scm_assign_mem(match->phys, (u64)match->size,
							&src_perms, &dst_perms, 1);
			if (err) {
				dev_err(cctx->dev, "%s: Failed to assign memory with phys 0x%llx size 0x%llx err %d",
					__func__, match->phys, match->size, err);
				spin_lock_irqsave(&cctx->lock, flags);
				list_add_tail(&match->node, &cctx->gmaps);
				spin_unlock_irqrestore(&cctx->lock, flags);
				return err;
			}
		}
		__fastrpc_buf_free(match);

	} while (match);

	return 0;
}

/*
 * Function to get static PD for process trying to attach,
 * by comparing service locator
 */
static int fastrpc_get_static_pd_session(struct fastrpc_user *fl, u32 *session)
{
	int i, err = 0;

	if (!fl)
		return -EBADF;

	for (i = 0; i < FASTRPC_MAX_SPD ; i++) {
		if (!fl->cctx->spd[i].servloc_name)
			continue;
		if (!strcmp(fl->servloc_name, fl->cctx->spd[i].servloc_name)) {
			*session = i;
			break;
		}
	}

	if (i >= FASTRPC_MAX_SPD)
		return -EUSERS;

	if (atomic_read(&fl->cctx->spd[i].ispdup) == 0)
		return -ENOTCONN;

	return err;
}

/* Function to check if static PD is up on remote subsystem */
static int fastrpc_check_static_pd_status(struct fastrpc_user *fl, u32 session)
{
	if (atomic_read(&fl->cctx->spd[session].ispdup) == 0)
		return -ENOTCONN;
	return 0;
}

/*
 * Function to get static PD to attach to and check its status.
 * Only one application can attach to Audio & OIS PD.
 */
static int fastrpc_init_static_pd_status(struct fastrpc_user *fl)
{
	int err = 0;
	u32 session = 0;

	if (!fl)
		return -EBADF;

	err = fastrpc_get_static_pd_session(fl, &session);
	if (err)
		return err;

	err = fastrpc_check_static_pd_status(fl, session);
	if (err)
		return err;

	// Allow only one application to connect to audio & OIS PD
	if (atomic_add_unless(&fl->cctx->spd[session].is_attached, 1, 1)) {
		fl->spd = &fl->cctx->spd[session];
	} else {
		dev_err(fl->cctx->dev,"Application already attached to audio PD\n");
		return -ECONNREFUSED;
	}

	return err;
}

/*
 * Function to get static PD to attach to and check its status.
 * Multiple applications can attach to sensors PD
 */
static int fastrpc_init_sensor_static_pd_status(struct fastrpc_user *fl)
{
	int err = 0;
	u32 session = 0;

	if (!fl)
		return -EBADF;

	err = fastrpc_get_static_pd_session(fl, &session);
	if (err)
		return err;

	err = fastrpc_check_static_pd_status(fl, session);
	if (err)
		return err;

	fl->spd = &fl->cctx->spd[session];

	// Update PDR count, to check for any PDR.
	fl->spd->prevpdrcount = fl->spd->pdrcount;

	return err;
}

#ifdef CONFIG_DEBUG_FS
void print_buf_info(struct seq_file *s_file, struct fastrpc_buf *buf)
{
    seq_printf(s_file,"\n %s %2s 0x%p", "virt", ":", buf->virt);
	seq_printf(s_file,"\n %s %2s 0x%llx", "phys", ":", buf->phys);
	seq_printf(s_file,"\n %s %2s 0x%lx", "raddr", ":", buf->raddr);
	seq_printf(s_file,"\n %s %2s 0x%x", "type", ":", buf->type);
	seq_printf(s_file,"\n %s %2s 0x%llx", "size", ":", buf->size);
	seq_printf(s_file,"\n %s %s %d", "in_use", ":", buf->in_use);
}

void print_ictx_info(struct seq_file *s_file, struct fastrpc_invoke_ctx *ictx)
{
	seq_printf(s_file,"\n %s %7s %d", "nscalars", ":", ictx->nscalars);
	seq_printf(s_file,"\n %s %10s %d", "nbufs", ":", ictx->nbufs);
	seq_printf(s_file,"\n %s %10s %d", "retval", ":", ictx->retval);
	seq_printf(s_file,"\n %s %12s %px", "crc", ":", ictx->crc);
	seq_printf(s_file,"\n %s %1s %d", "early_wake_time", ":", ictx->early_wake_time);
	seq_printf(s_file,"\n %s %5s %px", "perf_kernel", ":", ictx->perf_kernel);
	seq_printf(s_file,"\n %s %7s %px", "perf_dsp", ":", ictx->perf_dsp);
	seq_printf(s_file,"\n %s %12s %d", "pid", ":", ictx->pid);
	seq_printf(s_file,"\n %s %11s %d", "tgid", ":", ictx->tgid);
	seq_printf(s_file,"\n %s %13s 0x%x", "sc", ":", ictx->sc);
	seq_printf(s_file,"\n %s %10s %llu", "ctxid", ":", ictx->ctxid);
	seq_printf(s_file,"\n %s %3s %d", "is_work_done", ":", ictx->is_work_done);
	seq_printf(s_file,"\n %s %9s %llu", "msg_sz", ":", ictx->msg_sz);
}

void print_sctx_info(struct seq_file *s_file, struct fastrpc_pool_ctx *sctx)
{
	int i;
	struct fastrpc_smmu *s = NULL;

	seq_printf(s_file,"%s %9s %d\n", "pd_type", ":", sctx->pd_type);
	seq_printf(s_file,"%s %10s %d\n", "secure", ":", sctx->secure);
	seq_printf(s_file,"%s %8s %d\n", "sharedcb", ":", sctx->sharedcb);
	seq_printf(s_file,"%s %7s %d\n", "smmucount", ":", sctx->smmucount);
	seq_printf(s_file,"%s %8s %d\n", "usecount", ":", sctx->usecount);

	for (i = 0; i < sctx->smmucount; i++) {
		s = &sctx->smmucb[i];
		seq_printf(s_file,"\n========== SMMU context bank %d=============\n", i);
		seq_printf(s_file,"%s %13s %d\n", "sid", ":", s->sid);
		seq_printf(s_file,"%s %11s %d\n", "valid", ":", s->valid);
		seq_printf(s_file,"%s %4s %lu\n", "genpool_iova", ":",
								s->genpool_iova);
		seq_printf(s_file,"%s %4s %zu\n", "genpool_size", ":",
								s->genpool_size);
		seq_printf(s_file,"%s %2s %llx\n", "allocatedbytes", ":",
								s->allocatedbytes);
		seq_printf(s_file,"%s %6s %llx\n", "totalbytes", ":", s->totalbytes);
		seq_printf(s_file,"%s %4s %llx\n", "minallocsize", ":",
								s->minallocsize);
		seq_printf(s_file,"%s %4s %llx\n", "maxallocsize", ":",
								s->maxallocsize);
	}
}

void print_ctx_info(struct seq_file *s_file, struct fastrpc_channel_ctx *ctx)
{
	seq_printf(s_file,"%s %8s %d\n", "domain_id", ":", ctx->domain_id);
	seq_printf(s_file,"%s %8s %d\n", "sesscount", ":", ctx->sesscount);
	seq_printf(s_file,"%s %10s %d\n", "vmcount", ":", ctx->vmcount);
	seq_printf(s_file,"%s %12s %llu\n", "perms", ":", ctx->perms);
	seq_printf(s_file,"%s %s %d\n", "valid_attributes", ":", ctx->valid_attributes);
	seq_printf(s_file,"%s %3s %d\n", "cpuinfo_status", ":", ctx->cpuinfo_status);
	seq_printf(s_file,"%s %2s %d\n", "staticpd_status", ":", ctx->staticpd_status);
	seq_printf(s_file,"%s %11s %d\n", "secure", ":", ctx->secure);
	seq_printf(s_file,"%s %s %d\n", "unsigned_support", ":", ctx->unsigned_support);
}

void print_map_info(struct seq_file *s_file, struct fastrpc_map *map)
{
	seq_printf(s_file,"%s %4s %d\n", "fd", ":", map->fd);
	seq_printf(s_file,"%s %s 0x%llx\n", "phys", ":", map->phys);
	seq_printf(s_file,"%s %s 0x%llx\n", "size", ":", map->size);
	seq_printf(s_file,"%s %4s 0x%p\n", "va", ":", map->va);
	seq_printf(s_file,"%s %3s 0x%llx\n", "len", ":", map->len);
	seq_printf(s_file,"%s %2s 0x%llx\n", "raddr", ":", map->raddr);
	seq_printf(s_file,"%s %2s 0x%x\n", "attr", ":", map->attr);
	seq_printf(s_file,"%s %2s 0x%x\n", "flags", ":", map->flags);
}

static int fastrpc_debugfs_show(struct seq_file *s_file, void *data)
{
	struct fastrpc_user *fl = s_file->private;
	struct fastrpc_map *map;
	struct fastrpc_channel_ctx *ctx;
	struct fastrpc_pool_ctx *sctx = NULL;
	struct fastrpc_invoke_ctx *ictx, *m;
	struct fastrpc_buf *buf, *n;
	int i;
	unsigned long irq_flags = 0;

	if (fl != NULL) {
		seq_printf(s_file,"%s %12s %d\n", "tgid", ":", fl->tgid);
		seq_printf(s_file,"%s %7s %d\n", "tgid_frpc", ":", fl->tgid_frpc);
		seq_printf(s_file,"%s %3s %d\n", "is_secure_dev", ":", fl->is_secure_dev);
		seq_printf(s_file,"%s %3s %d\n", "num_pers_hdrs", ":", fl->num_pers_hdrs);
		seq_printf(s_file,"%s %2s %d\n", "num_cached_buf", ":", fl->num_cached_buf);
		seq_printf(s_file,"%s %5s %d\n", "wake_enable", ":", fl->wake_enable);
		seq_printf(s_file,"%s %2s %d\n",  "is_unsigned_pd", ":", fl->is_unsigned_pd);
		seq_printf(s_file,"%s %7s %d\n",  "sessionid", ":", fl->sessionid);
		seq_printf(s_file,"%s %9s %d\n", "pd_type", ":", fl->pd_type);
		seq_printf(s_file,"%s %9s %d\n",  "profile", ":", fl->profile);

                if(!fl->cctx)
                   return 0;

                seq_printf(s_file,"\n=============== Channel Context ===============\n");
		ctx = fl->cctx;
		print_ctx_info(s_file, ctx);
		if(fl->sctx) {
			seq_printf(s_file,"\n=============== Session Context ===============\n");
			sctx = fl->sctx;
			print_sctx_info(s_file, sctx);
		}
		if(fl->secsctx) {
			seq_printf(s_file,"\n=============== Secure Session Context ===============\n");
			sctx = fl->secsctx;
			print_sctx_info(s_file, sctx);
		}

		spin_lock(&fl->lock);
		if (fl->init_mem) {
			seq_printf(s_file,"\n=============== Init Mem ===============\n");
			buf = fl->init_mem;
			print_buf_info(s_file, buf);
		}
		if (fl->pers_hdr_buf) {
			seq_printf(s_file,"\n=============== Persistent Header Buf ===============\n");
			buf = fl->pers_hdr_buf;
			print_buf_info(s_file, buf);
		}
		if (fl->hdr_bufs) {
			seq_printf(s_file,"\n=============== Pre-allocated Header Buf ===============\n");
			buf = fl->hdr_bufs;
			print_buf_info(s_file, buf);
		}
		spin_unlock(&fl->lock);

		seq_printf(s_file,"\n=============== Global Maps ===============\n");
		spin_lock_irqsave(&fl->cctx->lock, irq_flags);
		list_for_each_entry_safe(buf, n, &fl->cctx->gmaps, node) {
			print_buf_info(s_file, buf);
		}
		spin_unlock_irqrestore(&fl->cctx->lock, irq_flags);
		seq_printf(s_file,"\n=============== DSP Signal Status ===============\n");
		spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);
		for (i = 0; i < FASTRPC_DSPSIGNAL_NUM_SIGNALS/FASTRPC_DSPSIGNAL_GROUP_SIZE; i++) {
			if (fl->signal_groups[i] != NULL)
				seq_printf(s_file,"%d : %d ",i, fl->signal_groups[i]->state);
		}
		spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
		seq_printf(s_file,"\n=============== User space maps ===============\n");
		spin_lock(&fl->lock);
		list_for_each_entry(map, &fl->maps, node) {
			if (map)
				print_map_info(s_file, map);
		}
		seq_printf(s_file,"\n=============== Kernel maps ===============\n");
		list_for_each_entry(buf, &fl->mmaps, node) {
			if (buf)
				print_buf_info(s_file, buf);
		}
		seq_printf(s_file,"\n=============== Cached Bufs ===============\n");
		list_for_each_entry_safe(buf, n, &fl->cached_bufs, node) {
			if(buf)
				print_buf_info(s_file, buf);
		}
		seq_printf(s_file,"\n=============== Pending contexts ===============\n");
		list_for_each_entry_safe(ictx, m, &fl->pending, node) {
			if (ictx)
				print_ictx_info(s_file, ictx);
		}
		seq_printf(s_file,"\n=============== Interrupted contexts ===============\n");
		list_for_each_entry_safe(ictx, m, &fl->interrupted, node) {
			if (ictx)
				print_ictx_info(s_file, ictx);
		}
		spin_unlock(&fl->lock);
	}
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(fastrpc_debugfs);

static int fastrpc_create_session_debugfs(struct fastrpc_user *fl)
{
	char cur_comm[TASK_COMM_LEN];
	int domain_id = -1, size = 0;
	struct dentry *debugfs_root = g_frpc.debugfs_root;

	if (atomic_cmpxchg(&fl->debugfs_file_create, 0, 1))
		return 0;
	memcpy(cur_comm, current->comm, TASK_COMM_LEN);
	cur_comm[TASK_COMM_LEN-1] = '\0';
	if (debugfs_root != NULL && fl != NULL) {
		domain_id = fl->cctx->domain_id;
		size = strlen(cur_comm) + strlen("_")
			+ COUNT_OF(current->pid) + strlen("_")
			+ COUNT_OF(fl->tgid_frpc) + strlen("_")
			+ COUNT_OF(FASTRPC_DEV_MAX)
			+ 1;

		fl->debugfs_buf = kzalloc(size, GFP_KERNEL);
		if (fl->debugfs_buf == NULL) {
			return -ENOMEM;
		}
		/*
		 * Use HLOS process name, HLOS PID, unique fastrpc PID
		 * domain_id in debugfs filename to create unique file name
		 */
		snprintf(fl->debugfs_buf, size, "%.10s%s%d%s%d%s%d",
			cur_comm, "_", current->pid, "_",
			fl->tgid_frpc, "_", domain_id);
		fl->debugfs_file = debugfs_create_file(fl->debugfs_buf, 0644,
			debugfs_root, fl, &fastrpc_debugfs_fops);
		if (IS_ERR_OR_NULL(fl->debugfs_file)) {
			pr_warn("Error: %s: %s: failed to create debugfs file %s\n",
					cur_comm, __func__, fl->debugfs_buf);
			fl->debugfs_file = NULL;
		}
		kfree(fl->debugfs_buf);
	}
return 0;
}
#endif

static int fastrpc_init_create_static_process(struct fastrpc_user *fl,
					      char __user *argp)
{
	struct fastrpc_init_create_static init;
	struct fastrpc_invoke_args args[FASTRPC_CREATE_STATIC_PROCESS_NARGS] = {0};
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_phy_page pages[1];
	struct fastrpc_buf *buf = NULL;
	struct fastrpc_smmu *smmucb = NULL;
	struct fastrpc_pool_ctx *sctx = NULL;
	u64 phys = 0, size = 0;
	char *name;
	int err = 0;
	bool scm_done = false;
	bool is_oispd = false, is_audiopd = false;
	unsigned long flags;
	struct {
		int pgid;
		u32 namelen;
		u32 pageslen;
	} inbuf;

	if (!fl->is_secure_dev) {
		dev_err(fl->cctx->dev, "untrusted app trying to attach to privileged DSP PD\n");
		return -EACCES;
	}

	if (copy_from_user(&init, argp, sizeof(init)))
		return -EFAULT;

	if ((init.namelen > INIT_FILE_NAMELEN_MAX) || (!init.namelen))
		return -EINVAL;

	name = memdup_user_nul(u64_to_user_ptr(init.name), init.namelen);
	/* ret -ENOMEM for malloc failure, -EFAULT for copy_from_user failure */
	if (IS_ERR(name))
		return PTR_ERR(name);

	sctx = fastrpc_session_alloc(fl, false);
	if (!sctx) {
		dev_err(fl->cctx->dev, "No session available\n");
		err = -EBUSY;
		goto err_name;
	}
	fl->sctx = sctx;

	smmucb = &fl->sctx->smmucb[DEFAULT_SMMU_IDX];
	is_oispd = !strcmp(name, "oispd");
	is_audiopd = !strcmp(name, "audiopd");

	/*
	 * Update the pd_type, to direct the messages to correct PD, when
	 * fastrpc_getpd_msgidx is queried. Update pd_type only after session
	 * allocation. Session is allocated based on user configured pd_type
	 */
	if (is_audiopd) {
		fl->pd_type = AUDIO_STATICPD;
		fl->servloc_name = AUDIO_PDR_SERVICE_LOCATION_CLIENT_NAME;
	} else if (is_oispd) {
		fl->pd_type = OIS_STATICPD;
		fl->servloc_name = OIS_PDR_ADSP_SERVICE_LOCATION_CLIENT_NAME;
	} else {
		dev_err(smmucb->dev,
		"Create static process is failed for proc_name %s", name);
		err = -EINVAL;
		goto err_name;
	}

	err = fastrpc_init_static_pd_status(fl);
	if (err)
		goto err_name;
	if (is_audiopd && IS_PDR(fl)) {
		/*
		 * Remove any previous mappings in case process is trying
		 * to reconnect after a PD restart on remote subsystem.
		 */
		err = fastrpc_mmap_remove_ssr(fl->cctx);
		if (err) {
			pr_warn("%s: %s: failed to unmap remote heap (err %d)\n",
				current->comm, __func__, err);
			goto err_name;
		}
	}
	// Update PDR count, to check for any PDR.
	fl->spd->prevpdrcount =	fl->spd->pdrcount;

	inbuf.pgid = fl->tgid_frpc;
	inbuf.namelen = init.namelen;
	inbuf.pageslen = 0;

	// Remote heap feature is available only for audio static PD
	if (!fl->cctx->staticpd_status && !is_oispd) {
		inbuf.pageslen = 1;
		err = fastrpc_buf_alloc(fl, NULL, init.memlen, REMOTEHEAP_BUF, &buf);
		if (err)
			goto err_name;

		phys = buf->phys;
		size = buf->size;
		/* Map if we have any heap VMIDs associated with this ADSP Static Process. */
		if (fl->cctx->vmcount) {
			u64 src_perms = BIT(QCOM_SCM_VMID_HLOS);

			err = qcom_scm_assign_mem(phys, (u64)size,
							&src_perms, fl->cctx->vmperms, fl->cctx->vmcount);
			if (err) {
				dev_err(smmucb->dev,
			"%s: Failed to assign memory with phys 0x%llx size 0x%llx err %d",
					__func__, phys, size, err);
				goto err_map;
			}
			scm_done = true;
		}
		fl->cctx->staticpd_status = true;
	}

	args[0].ptr = (u64)(uintptr_t)&inbuf;
	args[0].length = sizeof(inbuf);
	args[0].fd = -1;

	args[1].ptr = (u64)(uintptr_t)name;
	args[1].length = inbuf.namelen;
	args[1].fd = -1;

	pages[0].addr = phys;
	pages[0].size = size;

	args[2].ptr = (u64)(uintptr_t) pages;
	args[2].length = sizeof(*pages);
	args[2].fd = -1;

	ioctl.inv.handle = FASTRPC_INIT_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_CREATE_STATIC, 3, 0);
	ioctl.inv.args = (__u64)args;

	err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
	if (err)
		goto err_invoke;

#ifdef CONFIG_DEBUG_FS
	if (fl != NULL)
		fastrpc_create_session_debugfs(fl);
#endif
	kfree(name);

	if (buf) {
		spin_lock_irqsave(&fl->cctx->lock, flags);
		list_add_tail(&buf->node, &fl->cctx->gmaps);
		spin_unlock_irqrestore(&fl->cctx->lock, flags);
	}
	return 0;
err_invoke:
	if (fl->cctx->vmcount && scm_done) {
		u64 src_perms = 0;
		struct qcom_scm_vmperm dst_perms;
		u32 i;

		for (i = 0; i < fl->cctx->vmcount; i++)
			src_perms |= BIT(fl->cctx->vmperms[i].vmid);

		dst_perms.vmid = QCOM_SCM_VMID_HLOS;
		dst_perms.perm = QCOM_SCM_PERM_RWX;
		err = qcom_scm_assign_mem(phys, (u64)size,
						&src_perms, &dst_perms, 1);
		if (err)
			dev_err(smmucb->dev,
			"%s: Failed to assign memory phys 0x%llx size 0x%llx err %d",
				__func__, phys, size, err);
	}
err_map:
	if (buf) {
		fl->cctx->staticpd_status = false;
		fastrpc_buf_free(buf, false);
	}
err_name:
	kfree(name);
	return err;
}

/*
 * Find context bank / session with root PD type
 * @arg1: channel context.
 * @arg2: session context.
 *
 * The function searches for the session reserved for root pd from
 * the list of available sessions in a channel.
 *
 * Returns 0 if there is a session reserved for root pd.
 */
static int fastrpc_get_root_session(struct fastrpc_channel_ctx *cctx,
	struct fastrpc_pool_ctx **sess)
{
	int i = 0, err = -ENOSR;
	struct fastrpc_pool_ctx *s = NULL;
	unsigned long flags = 0;

	spin_lock_irqsave(&cctx->lock, flags);
	for (i = 0; i < cctx->sesscount; i++) {
		s = &cctx->session[i];
		if (s->pd_type == ROOT_PD && s->smmucb[DEFAULT_SMMU_IDX].valid) {
			*sess = s;
			err = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&cctx->lock, flags);
	return err;
}

/*
 * Allocate buffer for growing rootheap on DSP
 * @arg1: channel context.
 * @arg2: page array to be sent with process spawn msg
 * @arg3: number of pages
 *
 * Returns 0 on success
 */
static int fastrpc_alloc_rootheap_buf(struct fastrpc_channel_ctx *cctx,
	struct fastrpc_phy_page *pages, u32 *pageslen)
{
	struct fastrpc_buf *buf = NULL;
	struct fastrpc_pool_ctx *sess = NULL;
	struct fastrpc_smmu *smmucb = NULL;
	const unsigned int ROOTHEAP_BUF_SIZE = (1024 * 1024),
			NUM_ROOTHEAP_BUFS = 3;
	int err = 0;
	unsigned long flags = 0;

	/* Allocate buffer only if DSP supports growing of rootheap */
	if (!cctx->dsp_attributes[ROOTPD_RPC_HEAP_SUPPORT] ||
		cctx->rootheap_bufs.num >= NUM_ROOTHEAP_BUFS)
		return err;

	/* Get context bank / session reserved for rootPD */
	err = fastrpc_get_root_session(cctx, &sess);
	if (err)
		goto bail;

	smmucb = &sess->smmucb[DEFAULT_SMMU_IDX];
	err = __fastrpc_buf_alloc(NULL, smmucb, cctx->domain_id,
				ROOTHEAP_BUF_SIZE, &buf, ROOTHEAP_BUF);
	if (err)
		goto bail;

	/* Update paramaters of process-spawn with buffer info */
	*pageslen = NUM_PAGES_WITH_ROOTHEAP_BUF;
	pages[NUM_PAGES_WITH_ROOTHEAP_BUF - 1].addr = buf->phys;
	pages[NUM_PAGES_WITH_ROOTHEAP_BUF - 1].size = buf->size;

	/* Add buf to channel's rootheap buf-list and increment count */
	spin_lock_irqsave(&cctx->lock, flags);
	list_add_tail(&buf->node, &cctx->rootheap_bufs.list);
	cctx->rootheap_bufs.num++;
	spin_unlock_irqrestore(&cctx->lock, flags);
bail:
	return err;
}

static int get_unique_hlos_process_id(struct fastrpc_channel_ctx *cctx)
{
	int tgid_frpc = -1;
	int ret = -1;

	/* allocate unique id between 1 and MAX_FRPC_TGID both inclusive */
	ret = ida_alloc_range(&cctx->tgid_frpc_ida, 1,
			       MAX_FRPC_TGID, GFP_ATOMIC);
	if (ret < 0) {
		return -1;
	}
	tgid_frpc = ((cctx->domain_id) * FASTRPC_UNIQUE_ID_CONST) + ret;
	return tgid_frpc;
}

/**
 * fastrpc_pack_root_sharedpage()- Packs shared page for rootPD.
 * @fl: fastrpc user instance.
 * @pages: pages to be packed for DSP.
 * @pageslen: Number of pages.
 *
 * fastrpc_pack_root_sharedpage packs root shared page during
 * creation of a dynamic process.
 *
 * Return: 0 on success.
 */
static int fastrpc_pack_root_sharedpage(struct fastrpc_user *fl,
	struct fastrpc_phy_page *pages, u32 *pageslen)
{
	int err = 0;
	u64 addr = fl->config.root_addr;
	u32 size = fl->config.root_size;
	struct fastrpc_smmu *smmucb = &fl->sctx->smmucb[DEFAULT_SMMU_IDX];

	/* Allocate kernel buffer for rootPD shared page */
	if (addr && size) {
		err = fastrpc_buf_alloc(fl, smmucb, size, USER_BUF,
					&fl->proc_init_sharedbuf);
		if (err) {
			dev_err(smmucb->dev, "failed to allocate buffer\n");
			return err;
		}
		/* Copy contents from userspace buffer containing data for rootPD */
		if (copy_from_user(fl->proc_init_sharedbuf->virt,
				(void __user *)(uintptr_t)addr, size)) {
			err = -EFAULT;
			goto err_sharedbuf_fail;
		}
		/* Update paramaters of process-spawn with buffer info */
		*pageslen = NUM_PAGES_WITH_PROC_INIT_SHAREDBUF;
		pages[NUM_PAGES_WITH_PROC_INIT_SHAREDBUF-1].addr =
			fl->proc_init_sharedbuf->phys;
		pages[NUM_PAGES_WITH_PROC_INIT_SHAREDBUF-1].size =
			fl->proc_init_sharedbuf->size;
	}

	return 0;

err_sharedbuf_fail:
	if (fl->proc_init_sharedbuf) {
		fastrpc_buf_free(fl->proc_init_sharedbuf, false);
		fl->proc_init_sharedbuf = NULL;
	}
	return err;
}

static int fastrpc_init_create_process(struct fastrpc_user *fl,
					char __user *argp)
{
	struct fastrpc_init_create init;
	struct fastrpc_invoke_args args[FASTRPC_CREATE_PROCESS_NARGS] = {0};
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_phy_page pages[NUM_PAGES_WITH_PROC_INIT_SHAREDBUF] = {0};
	struct fastrpc_map *configmap = NULL;
	struct fastrpc_buf *imem = NULL;
	struct fastrpc_pool_ctx *sctx = NULL;
	int memlen;
	int err = 0;
	int user_fd = fl->config.user_fd, user_size = fl->config.user_size;
	void *file = NULL;
	struct {
		int pgid;
		u32 namelen;
		u32 filelen;
		u32 pageslen;
		u32 attrs;
		u32 siglen;
	} inbuf;

	if (copy_from_user(&init, argp, sizeof(init)))
		return -EFAULT;

	if (init.filelen > INIT_FILELEN_MAX)
		return -EINVAL;

	/* Return an error if the create process already started or completed */
	if (atomic_cmpxchg(&fl->state, DEFAULT_PROC_STATE,
				DSP_CREATE_START) != DEFAULT_PROC_STATE)
		return -EALREADY;

	/* Verify shell file passed by user */
	if (init.filefd <= 0) {
		if (!init.filelen || !init.file) {
		/*In this case shell will be loaded by DSP using daemon */
			init.file = 0;
			init.filelen = 0;
		} else {
			file = kzalloc(init.filelen, GFP_KERNEL);
			if (!file) {
				err = -ENOMEM;
				goto err_out;
			}
			if (copy_from_user(file,
				(void *)(uintptr_t)init.file,
				init.filelen)) {
				err = -EFAULT;
				dev_err(fl->cctx->dev, "copy_from_user failed for shell file\n");
				goto err_out;
			}
		}
	}
	/*
	 * Third-party apps don't have permission to open the fastrpc device, so
	 * it is opened on their behalf by DSP HAL. This is detected by
	 * comparing current PID with the one stored during device open.
	 */
	if (current->tgid != fl->tgid)
		fl->untrusted_process = true;

	if (init.attrs & FASTRPC_MODE_UNSIGNED_MODULE)
		fl->is_unsigned_pd = true;

	/* Disregard any system unsigned PD attribute from userspace */
	init.attrs &= (~FASTRPC_MODE_SYSTEM_UNSIGNED_PD);

	if (is_session_rejected(fl, fl->is_unsigned_pd)) {
		err = -EACCES;
		goto err_out;
	}

	/* Trusted apps will be launched as system unsigned PDs */
	if (!fl->untrusted_process && fl->is_unsigned_pd)
		init.attrs |= FASTRPC_MODE_SYSTEM_UNSIGNED_PD;

	/*
	 * Use SMMU pooled session for unsigned PD,
	 * if smmucb_pool is set to true
	 */
	if (fl->is_unsigned_pd && fl->cctx->smmucb_pool)
		fl->pd_type = USER_UNSIGNEDPD_POOL;

	sctx = fastrpc_session_alloc(fl, false);
	if (!sctx) {
		dev_err(fl->cctx->dev, "No session available\n");
		err = -EBUSY;
		goto err_out;
	}
	fl->sctx = sctx;

	/* In case of privileged process update attributes */
	fastrpc_check_privileged_process(fl, &init);

	inbuf.pgid = fl->tgid_frpc;
	inbuf.namelen = strlen(current->comm) + 1;
	inbuf.filelen = init.filelen;
	inbuf.pageslen = 1;
	inbuf.attrs = init.attrs;
	inbuf.siglen = init.siglen;

	/*
	 * Default value at fastrpc_device_open is set as DEFAULT_UNUSED.
	 * If pd_type is not configured by the process in fastrpc_set_session_info,
	 * update the pd_type to USERPD, so that messages are directed to
	 * dynamic process when fastrpc_getpd_msgidx is queried.
	 * Do this only after session allocation
	 */
	if (fl->pd_type == DEFAULT_UNUSED)
		fl->pd_type = USERPD;

	if (user_fd != -1 && user_size > 0) {
		mutex_lock(&fl->map_mutex);
		err = fastrpc_map_create(fl, user_fd, 0, NULL,
				user_size, 0, 0, &configmap, true);
		mutex_unlock(&fl->map_mutex);
		if (err)
			goto err_out;
		inbuf.pageslen = NUM_PAGES_WITH_SHARED_BUF;
		pages[NUM_PAGES_WITH_SHARED_BUF - 1].addr = configmap->phys;
		pages[NUM_PAGES_WITH_SHARED_BUF - 1].size = configmap->size;
	}

	/* Process spawn should not fail if unable to alloc rootheap buffer */
	fastrpc_alloc_rootheap_buf(fl->cctx, pages, &inbuf.pageslen);

	/* Process spawn should not fail if unable to pack root buffer */
	fastrpc_pack_root_sharedpage(fl, pages, &inbuf.pageslen);

	memlen = ALIGN(max(INIT_FILELEN_MAX, (int)init.filelen * 4),
		       1024 * 1024);

	err = fastrpc_smmu_buf_alloc(fl, memlen, INITMEM_BUF, &imem);
	if (err)
		goto err_alloc;

	fl->init_mem = imem;
	args[0].ptr = (u64)(uintptr_t)&inbuf;
	args[0].length = sizeof(inbuf);
	args[0].fd = -1;

	args[1].ptr = (u64)(uintptr_t)current->comm;
	args[1].length = inbuf.namelen;
	args[1].fd = -1;

	args[2].ptr = file ? (u64)(uintptr_t)file : init.file;
	args[2].length = inbuf.filelen;
	args[2].fd = init.filefd;

	pages[0].addr = imem->phys;
	pages[0].size = imem->size;

	args[3].ptr = (u64)(uintptr_t) pages;
	args[3].length = inbuf.pageslen * sizeof(*pages);
	args[3].fd = -1;

	args[4].ptr = (u64)(uintptr_t)&inbuf.attrs;
	args[4].length = sizeof(inbuf.attrs);
	args[4].fd = -1;

	args[5].ptr = (u64)(uintptr_t) &inbuf.siglen;
	args[5].length = sizeof(inbuf.siglen);
	args[5].fd = -1;

	ioctl.inv.handle = FASTRPC_INIT_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_CREATE, 4, 0);
	if (init.attrs)
		ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_CREATE_ATTR, 4, 0);
	ioctl.inv.args = (__u64)args;

	err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
	if (err)
		goto err_invoke;

	if (fl->cctx->domain_id == CDSP_DOMAIN_ID) {
		fastrpc_create_persistent_headers(fl);
	}

#ifdef CONFIG_DEBUG_FS
		fastrpc_create_session_debugfs(fl);
#endif
	/* remove buffer on success as no longer required */
	if (fl->proc_init_sharedbuf) {
		fastrpc_buf_free(fl->proc_init_sharedbuf, false);
		fl->proc_init_sharedbuf = NULL;
	}
	kfree(file);

	return 0;

err_invoke:
	spin_lock(&fl->lock);
	fl->init_mem = NULL;
	spin_unlock(&fl->lock);
	fastrpc_buf_free(imem, false);
err_alloc:
	if (fl->proc_init_sharedbuf) {
		fastrpc_buf_free(fl->proc_init_sharedbuf, false);
		fl->proc_init_sharedbuf = NULL;
	}
	if (configmap) {
		mutex_lock(&fl->map_mutex);
		fastrpc_map_put(configmap);
		mutex_unlock(&fl->map_mutex);
	}
err_out:
	kfree(file);
	/* Reset the process state to its default in case of an error. */
	atomic_set(&fl->state, DEFAULT_PROC_STATE);
	return err;
}

static void fastrpc_context_list_free(struct fastrpc_user *fl)
{
	struct fastrpc_invoke_ctx *ctx, *n;

	list_for_each_entry_safe(ctx, n, &fl->interrupted, node) {
		spin_lock(&fl->lock);
		list_del(&ctx->node);
		spin_unlock(&fl->lock);
		fastrpc_context_put(ctx);
	}

	list_for_each_entry_safe(ctx, n, &fl->pending, node) {
		spin_lock(&fl->lock);
		list_del(&ctx->node);
		spin_unlock(&fl->lock);
		fastrpc_context_put(ctx);
	}
}

static int fastrpc_release_current_dsp_process(struct fastrpc_user *fl)
{
	struct fastrpc_invoke_args args[1];
	struct fastrpc_enhanced_invoke ioctl;
	int tgid = 0;

	tgid = fl->tgid_frpc;
	args[0].ptr = (u64)(uintptr_t) &tgid;
	args[0].length = sizeof(tgid);
	args[0].fd = -1;

	ioctl.inv.handle = FASTRPC_INIT_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_RELEASE, 1, 0);
	ioctl.inv.args = (__u64)args;

	return fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_NONZERO_PID, &ioctl);
}

/*
 * Helper function to increment / decrement invoke count of channel
 * Caller of this function MUST spin-lock 'cctx->lock' first.
 */
static inline void fastrpc_channel_update_invoke_cnt(
		struct fastrpc_channel_ctx *cctx, bool incr)
{
	if (incr) {
		cctx->invoke_cnt++;
	} else {
		cctx->invoke_cnt--;
		/* Wake up any waiting SSR handling thread */
		if (cctx->invoke_cnt == 0)
			wake_up_interruptible(&cctx->ssr_wait_queue);
	}
}

void fastrpc_free_user(struct fastrpc_user *fl)
{
	struct fastrpc_map *map = NULL, *m = NULL;

	fastrpc_context_list_free(fl);

	if (fl->init_mem) {
		fastrpc_buf_free(fl->init_mem, false);
		fl->init_mem = NULL;
	}

	mutex_lock(&fl->remote_map_mutex);
	mutex_lock(&fl->map_mutex);
	// During process tear down free the map, even if refcount is non-zero
	list_for_each_entry_safe(map, m, &fl->maps, node)
		__fastrpc_free_map(map);
	mutex_unlock(&fl->map_mutex);
	mutex_unlock(&fl->remote_map_mutex);

	fastrpc_buf_list_free(fl, &fl->mmaps, false);

	if (fl->pers_hdr_buf) {
		fastrpc_buf_free(fl->pers_hdr_buf, false);
		fl->pers_hdr_buf = NULL;
	}

	if (fl->hdr_bufs) {
		kfree(fl->hdr_bufs);
		fl->hdr_bufs = NULL;
	}

	fastrpc_buf_list_free(fl, &fl->cached_bufs, true);

	return;
}

static int fastrpc_device_release(struct inode *inode, struct file *file)
{
	struct fastrpc_user *fl = (struct fastrpc_user *)file->private_data;
	struct fastrpc_channel_ctx *cctx = fl->cctx;
	struct fastrpc_driver *frpc_drv, *d;
	struct fastrpc_buf *buf, *b;
	int i;
	unsigned long flags, irq_flags;
	bool locked = false, is_driver_registered = false;
	spinlock_t *glock = &g_frpc.glock;
	int err = 0;
	struct fastrpc_notif_rsp *inotif, *n1;

	spin_lock_irqsave(glock, irq_flags);
	spin_lock_irqsave(&cctx->lock, flags);
	if (atomic_read(&cctx->teardown)) {
		spin_unlock_irqrestore(&cctx->lock, flags);
		spin_unlock_irqrestore(glock, irq_flags);
		/*
		 * Wait until SSR cleanup is done to avoid parallel access of
		 * fastrpc_user object from device release thread and
		 * SSR handling thread.
		 */
		wait_for_completion(&cctx->ssr_complete);
		spin_lock_irqsave(glock, irq_flags);
		spin_lock_irqsave(&cctx->lock, flags);
	} else {
		/*
		 * Update invoke count to block the SSR handling thread from cleaning up
		 * the channel resources, while it is still being used by this thread.
		 */
		fastrpc_channel_update_invoke_cnt(cctx, true);
	}
	if (fl->device) {
		fl->device->dev_close = true;
		fl->device->fl = NULL;
	}
	atomic_set(&fl->state, DSP_EXIT_START);
	list_for_each_entry_safe(frpc_drv, d, &fl->fastrpc_drivers, hn){
		/*
		 * Registered driver can free driver object in callback.
		 * So, delete object from list first.
		 */
		list_del(&frpc_drv->hn);
		if(frpc_drv->callback) {
			spin_unlock_irqrestore(&cctx->lock, flags);
			spin_unlock_irqrestore(glock, irq_flags);
			frpc_drv->callback(fl->device, FASTRPC_PROC_DOWN);
			spin_lock_irqsave(glock, irq_flags);
			spin_lock_irqsave(&cctx->lock, flags);
		}
		is_driver_registered = true;
	}
	spin_unlock_irqrestore(&cctx->lock, flags);
	spin_unlock_irqrestore(glock, irq_flags);

	/*
	 * If no driver is registered on the device, free it here.
	 * If any active driver is still registered, device will
	 * be freed when driver is unregistered.
	 */
	if (!is_driver_registered)
		kfree(fl->device);
	if (fl->spd)
		atomic_set(&fl->spd->is_attached, 0);

	err = fastrpc_release_current_dsp_process(fl);
	if (err == -ETIMEDOUT) {
		pr_err("%s failed with err %d for process %s fl->tgid %d fl->tgid_frpc %d\n",
			__func__, err, current->comm, fl->tgid, fl->tgid_frpc);
		BUG_ON(1);
	}
	atomic_set(&fl->state, DSP_EXIT_COMPLETE);

	spin_lock_irqsave(&cctx->lock, flags);
	locked = true;
	if(fl->is_dma_invoke_pend) {
		spin_unlock_irqrestore(&cctx->lock, flags);
		wait_for_completion(&fl->dma_invoke);
		locked = false;
	}
	if(locked)
		spin_unlock_irqrestore(&cctx->lock, flags);

	spin_lock_irqsave(&cctx->lock, flags);
	list_del(&fl->user);
	spin_unlock_irqrestore(&cctx->lock, flags);

	spin_lock_irqsave(&fl->proc_state_notif.nqlock, flags);
	atomic_add(1, &fl->proc_state_notif.notif_queue_count);
	wake_up_interruptible(&fl->proc_state_notif.notif_wait_queue);
	list_for_each_entry_safe(inotif, n1, &fl->notif_queue, notifn) {
		list_del_init(&inotif->notifn);
		atomic_sub(1, &fl->proc_state_notif.notif_queue_count);
		kfree(inotif);
	}
	spin_unlock_irqrestore(&fl->proc_state_notif.nqlock, flags);

	if (fl->tgid_frpc != -1)
		ida_free(&cctx->tgid_frpc_ida, fl->tgid_frpc-(cctx->domain_id*FASTRPC_UNIQUE_ID_CONST));

	fl->is_dma_invoke_pend = false;

	fastrpc_free_user(fl);

	/*
	 * Audio remote-heap buffers won't be freed as part of "fastrpc_user" object
	 * cleanup. Instead, they will be freed after SSR dump collection.
	 * Reset "fl" pointer in the buffer objects if it is the object getting
	 * freed here.
	 */
	spin_lock_irqsave(&cctx->lock, flags);
	list_for_each_entry_safe(buf, b, &cctx->gmaps, node) {
		if (buf->fl == fl)
			buf->fl = NULL;
	}
	spin_unlock_irqrestore(&cctx->lock, flags);

	if (fl->qos_request && fl->dev_pm_qos_req) {
		for (i = 0; i < cctx->lowest_capacity_core_count; i++) {
			if (!dev_pm_qos_request_active(&fl->dev_pm_qos_req[i]))
				continue;
			dev_pm_qos_remove_request(&fl->dev_pm_qos_req[i]);
		}
	}
	kfree(fl->dev_pm_qos_req);
	fastrpc_pm_relax(fl,cctx->secure);
	if (fl->sctx)
		fastrpc_session_free(cctx, fl->sctx);
	if (fl->secsctx)
		fastrpc_session_free(cctx, fl->secsctx);
	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);
	for (i = 0; i < (FASTRPC_DSPSIGNAL_NUM_SIGNALS /FASTRPC_DSPSIGNAL_GROUP_SIZE); i++)
		kfree(fl->signal_groups[i]);
	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(fl->debugfs_file);
#endif
	mutex_destroy(&fl->signal_create_mutex);
	mutex_destroy(&fl->remote_map_mutex);
	mutex_destroy(&fl->map_mutex);
	mutex_destroy(&fl->pm_qos_mutex);
	spin_lock_irqsave(glock, irq_flags);
	kfree(fl);

	spin_lock_irqsave(&cctx->lock, flags);
	fastrpc_channel_update_invoke_cnt(cctx, false);
	spin_unlock_irqrestore(&cctx->lock, flags);
	fastrpc_channel_ctx_put(cctx);
	file->private_data = NULL;
	spin_unlock_irqrestore(glock, irq_flags);
	return 0;
}

static int fastrpc_device_open(struct inode *inode, struct file *filp)
{
	struct fastrpc_channel_ctx *cctx;
	struct fastrpc_device_node *fdevice;
	struct fastrpc_user *fl = NULL;
	unsigned long flags;
	int err;

	fdevice = miscdev_to_fdevice(filp->private_data);
	cctx = fdevice->cctx;

	if (atomic_read(&cctx->teardown))
		return -EPIPE;

	fl = kzalloc(sizeof(*fl), GFP_KERNEL);
	if (!fl)
		return -ENOMEM;

	/* Released in fastrpc_device_release() */
	fastrpc_channel_ctx_get(cctx);

	filp->private_data = fl;
	spin_lock_init(&fl->lock);
	mutex_init(&fl->remote_map_mutex);
	mutex_init(&fl->map_mutex);
	spin_lock_init(&fl->dspsignals_lock);
	mutex_init(&fl->signal_create_mutex);
	mutex_init(&fl->pm_qos_mutex);
	INIT_LIST_HEAD(&fl->pending);
	INIT_LIST_HEAD(&fl->interrupted);
	INIT_LIST_HEAD(&fl->maps);
	INIT_LIST_HEAD(&fl->mmaps);
	INIT_LIST_HEAD(&fl->user);
	INIT_LIST_HEAD(&fl->cached_bufs);
	INIT_LIST_HEAD(&fl->notif_queue);
	INIT_LIST_HEAD(&fl->fastrpc_drivers);
	init_waitqueue_head(&fl->proc_state_notif.notif_wait_queue);
	spin_lock_init(&fl->proc_state_notif.nqlock);
	init_completion(&fl->dma_invoke);

	fl->cctx = cctx;
	fl->tgid = current->tgid;
	fl->tgid_frpc = get_unique_hlos_process_id(cctx);

	if (fl->tgid_frpc == -1) {
		dev_err(cctx->dev, "too many fastrpc clients, max %u allowed\n", MAX_FRPC_TGID);
		err = -EUSERS;
		goto error;
	}
	dev_dbg(cctx->dev, "HLOS pid %d, domain %d is mapped to unique sessions pid %d",
			fl->tgid, fl->cctx->domain_id, fl->tgid_frpc);
	fl->is_secure_dev = fdevice->secure;
	fl->sessionid = 0;
	fl->config.user_fd = -1;
	fl->pd_type = DEFAULT_UNUSED;
	fl->multi_session_support = false;
	fl->set_session_info = false;

	if (cctx->lowest_capacity_core_count) {
		fl->dev_pm_qos_req = kzalloc((cctx->lowest_capacity_core_count) *
				sizeof(struct dev_pm_qos_request), GFP_KERNEL);
		if (!fl->dev_pm_qos_req) {
			err = -ENOMEM;
			goto error;
		}
	}

	spin_lock_irqsave(&cctx->lock, flags);
	list_add_tail(&fl->user, &cctx->users);
	spin_unlock_irqrestore(&cctx->lock, flags);

	return 0;
error:
	mutex_destroy(&fl->remote_map_mutex);
	mutex_destroy(&fl->map_mutex);
	mutex_destroy(&fl->signal_create_mutex);
	kfree(fl);
	fastrpc_channel_ctx_put(cctx);

	return err;
}

static int fastrpc_dmabuf_alloc(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_alloc_dma_buf bp;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct fastrpc_buf *buf = NULL;
	int err;

	if (copy_from_user(&bp, argp, sizeof(bp)))
		return -EFAULT;
	if (!bp.size)
		return -EFAULT;
	if (!fl->sctx)
		return -EINVAL;

	err = fastrpc_smmu_buf_alloc(fl, bp.size, USER_BUF, &buf);
	if (err)
		return err;
	exp_info.ops = &fastrpc_dma_buf_ops;
	exp_info.size = bp.size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buf;
	buf->dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(buf->dmabuf)) {
		err = PTR_ERR(buf->dmabuf);
		fastrpc_buf_free(buf, false);
		return err;
	}

	bp.fd = dma_buf_fd(buf->dmabuf, O_ACCMODE);
	if (bp.fd < 0) {
		dma_buf_put(buf->dmabuf);
		return -EINVAL;
	}

	if (copy_to_user(argp, &bp, sizeof(bp))) {
		/*
		 * The usercopy failed, but we can't do much about it, as
		 * dma_buf_fd() already called fd_install() and made the
		 * file descriptor accessible for the current process. It
		 * might already be closed and dmabuf no longer valid when
		 * we reach this point. Therefore "leak" the fd and rely on
		 * the process exit path to do any required cleanup.
		 */
		return -EFAULT;
	}

	return 0;
}

static int fastrpc_send_cpuinfo_to_dsp(struct fastrpc_user *fl)
{
	int err = 0;
	u64 cpuinfo = 0;
	struct fastrpc_invoke_args args[1];
	struct fastrpc_enhanced_invoke ioctl;

	if (!fl) {
		return -EBADF;
	}

	cpuinfo = fl->cctx->cpuinfo_todsp;
	/* return success if already updated to remote processor */
	if (fl->cctx->cpuinfo_status)
		return 0;

	args[0].ptr = (u64)(uintptr_t)&cpuinfo;
	args[0].length = sizeof(cpuinfo);
	args[0].fd = -1;

	ioctl.inv.handle = FASTRPC_DSP_UTILITIES_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(1, 1, 0);
	ioctl.inv.args = (__u64)args;

	err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
	if (!err)
		fl->cctx->cpuinfo_status = true;

	return err;
}

static int fastrpc_init_attach(struct fastrpc_user *fl, int pd)
{
	struct fastrpc_invoke_args args[1];
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_pool_ctx *sctx = NULL;
	int err, tgid = fl->tgid_frpc;

	if (!fl->is_secure_dev) {
		dev_err(fl->cctx->dev, "untrusted app trying to attach to privileged DSP PD\n");
		return -EACCES;
	}
	sctx = fastrpc_session_alloc(fl, false);
	if (!sctx) {
		dev_err(fl->cctx->dev, "No session available\n");
		return -EBUSY;
	}
	fl->sctx = sctx;

	/*
	 * Default value at fastrpc_device_open is set as DEFAULT_UNUSED.
	 * If pd_type is not configured by the process in fastrpc_set_session_info,
	 * update the pd_type, so that messages are directed to right process,
	 * when fastrpc_getpd_msgidx is queried.
	 * Do this only after session allocation.
	 */
	if (fl->pd_type == DEFAULT_UNUSED)
		fl->pd_type = pd;

	if (pd == SENSORS_STATICPD) {
		if (fl->cctx->domain_id == ADSP_DOMAIN_ID)
			fl->servloc_name = SENSORS_PDR_ADSP_SERVICE_LOCATION_CLIENT_NAME;
		else if (fl->cctx->domain_id == SDSP_DOMAIN_ID)
			fl->servloc_name = SENSORS_PDR_SLPI_SERVICE_LOCATION_CLIENT_NAME;

		err = fastrpc_init_sensor_static_pd_status(fl);
		if (err)
			return err;
	}

	args[0].ptr = (u64)(uintptr_t) &tgid;
	args[0].length = sizeof(tgid);
	args[0].fd = -1;

	ioctl.inv.handle = FASTRPC_INIT_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_ATTACH, 1, 0);
	ioctl.inv.args = (__u64)args;

	err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
	if (err)
		return err;

#ifdef CONFIG_DEBUG_FS
		fastrpc_create_session_debugfs(fl);
#endif
	return 0;
}

static int fastrpc_invoke(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_invoke inv;
	int err;

	if (copy_from_user(&inv, argp, sizeof(inv)))
		return -EFAULT;

	ioctl.inv = inv;

	err = fastrpc_internal_invoke(fl, USER_MSG, &ioctl);

	return err;
}

void fastrpc_queue_pd_status(struct fastrpc_user *fl, int domain, int status, int sessionid)
{
	struct fastrpc_notif_rsp *notif_rsp = NULL;
	unsigned long flags;

	notif_rsp = kzalloc(sizeof(*notif_rsp), GFP_ATOMIC);
	if (!notif_rsp) {
		dev_err(fl->cctx->dev, "Allocation failed for notif\n");
		return;
	}

	notif_rsp->status = status;
	notif_rsp->domain = domain;
	notif_rsp->session = sessionid;

	spin_lock_irqsave(&fl->proc_state_notif.nqlock, flags);
	list_add_tail(&notif_rsp->notifn, &fl->notif_queue);
	atomic_add(1, &fl->proc_state_notif.notif_queue_count);
	wake_up_interruptible(&fl->proc_state_notif.notif_wait_queue);
	spin_unlock_irqrestore(&fl->proc_state_notif.nqlock, flags);
}

static void fastrpc_notif_find_process(int domain, struct fastrpc_channel_ctx *cctx, struct dsp_notif_rsp *notif)
{
	bool is_process_found = false;
	unsigned long irq_flags = 0;
	struct fastrpc_user *user;

	spin_lock_irqsave(&cctx->lock, irq_flags);
	list_for_each_entry(user, &cctx->users, user) {
		if (user->tgid_frpc == notif->pid) {
			is_process_found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&cctx->lock, irq_flags);

	if (!is_process_found)
		return;
	fastrpc_queue_pd_status(user, domain, notif->status, user->sessionid);
}

static int fastrpc_wait_on_notif_queue(
			struct fastrpc_internal_notif_rsp *notif_rsp,
			struct fastrpc_user *fl)
{
	int err = 0;
	unsigned long flags;
	struct fastrpc_notif_rsp *notif = NULL, *inotif, *n;

read_notif_status:
	err = wait_event_interruptible(fl->proc_state_notif.notif_wait_queue,
				atomic_read(&fl->proc_state_notif.notif_queue_count));
	if (err)
		return err;
	if (fl->exit_notif)
		return -EFAULT;

	spin_lock_irqsave(&fl->proc_state_notif.nqlock, flags);
	list_for_each_entry_safe(inotif, n, &fl->notif_queue, notifn) {
		list_del(&inotif->notifn);
		atomic_sub(1, &fl->proc_state_notif.notif_queue_count);
		notif = inotif;
		break;
	}
	spin_unlock_irqrestore(&fl->proc_state_notif.nqlock, flags);

	if (notif) {
		notif_rsp->status = notif->status;
		notif_rsp->domain = notif->domain;
		notif_rsp->session = notif->session;
	} else {// Go back to wait if ctx is invalid
		dev_err(fl->cctx->dev, "Invalid status notification response\n");
		goto read_notif_status;
	}

	kfree(notif);
	return err;
}

static int fastrpc_get_notif_response(
			struct fastrpc_internal_notif_rsp *notif,
			void *param, struct fastrpc_user *fl)
{
	int err = 0;
	err = fastrpc_wait_on_notif_queue(notif, fl);
	if (err)
		return err;

	if (copy_to_user((void __user *)param, notif,
			sizeof(struct fastrpc_internal_notif_rsp)))
		return -EFAULT;

	return 0;
}

static int fastrpc_manage_poll_mode(struct fastrpc_user *fl, u32 enable, u32 timeout)
{
	const unsigned int MAX_POLL_TIMEOUT_US = 10000;

	if ((fl->cctx->domain_id != CDSP_DOMAIN_ID) || (fl->pd_type != USERPD &&
			fl->pd_type != USER_UNSIGNEDPD_POOL)) {
		dev_err(fl->cctx->dev,"poll mode only allowed for dynamic CDSP process\n");
		return -EPERM;
	}
	if (timeout > MAX_POLL_TIMEOUT_US) {
		dev_err(fl->cctx->dev,"poll timeout %u is greater than max allowed value %u\n",
			timeout, MAX_POLL_TIMEOUT_US);
		return -EBADMSG;
	}
	spin_lock(&fl->lock);
	if (enable) {
		fl->poll_mode = true;
		fl->poll_timeout = timeout;
	} else {
		fl->poll_mode = false;
		fl->poll_timeout = 0;
	}
	spin_unlock(&fl->lock);
	dev_info(fl->cctx->dev,"updated poll mode to %d, timeout %u\n", enable, timeout);
	return 0;
}

static int fastrpc_internal_control(struct fastrpc_user *fl,
					struct fastrpc_internal_control *cp)
{
	int err = 0;
	struct fastrpc_channel_ctx *cctx = fl->cctx;
	u32 latency = 0, cpu = 0;
	unsigned long flags = 0;

	if (!fl) {
		return -EBADF;
	}
	if (!cp) {
		return -EINVAL;
	}

	switch (cp->req) {
	case FASTRPC_CONTROL_LATENCY:
		if (cp->lp.enable)
			latency =  cctx->qos_latency;
		else
			latency = PM_QOS_RESUME_LATENCY_DEFAULT_VALUE;
		if (latency == 0)
			return -EINVAL;
		if (!(cctx->lowest_capacity_core_count && fl->dev_pm_qos_req)) {
			dev_err(fl->cctx->dev, "Skipping PM QoS latency voting, core count: %u\n",
						cctx->lowest_capacity_core_count);
			return -EINVAL;
		}
		/*
		 * Add voting request for all possible cores corresponding to cluster
		 * id 0. If DT property 'qcom,single-core-latency-vote' is enabled
		 * then add voting request for only one core of cluster id 0.
		 */
		 mutex_lock(&fl->pm_qos_mutex);
		 for (cpu = 0; cpu < cctx->lowest_capacity_core_count; cpu++) {
			if (!fl->qos_request) {
				err = dev_pm_qos_add_request(
						get_cpu_device(cpu),
						&fl->dev_pm_qos_req[cpu],
						DEV_PM_QOS_RESUME_LATENCY,
						latency);
			} else {
				err = dev_pm_qos_update_request(
						&fl->dev_pm_qos_req[cpu],
						latency);
			}
			if (err < 0) {
				dev_err(fl->cctx->dev, "QoS with lat %u failed for CPU %d, err %d, req %d\n",
					latency, cpu, err, fl->qos_request);
				break;
			}
		}
		if (err >= 0) {
			fl->qos_request = 1;
			err = 0;
		}
		mutex_unlock(&fl->pm_qos_mutex);
		break;
	case FASTRPC_CONTROL_SMMU:
		fl->sharedcb = cp->smmu.sharedcb;
		break;
	case FASTRPC_CONTROL_WAKELOCK:
		if (!fl->is_secure_dev) {
			dev_err(fl->cctx->dev,
				"PM voting not allowed for non-secure device node");
			err = -EPERM;
			return err;
		}
		fl->wake_enable = cp->wp.enable;
		break;
	case FASTRPC_CONTROL_PM:
		if (!fl->wake_enable)
			return -EACCES;
		if (cp->pm.timeout > FASTRPC_MAX_PM_TIMEOUT_MS)
			fl->ws_timeout = FASTRPC_MAX_PM_TIMEOUT_MS;
		else
			fl->ws_timeout = cp->pm.timeout;
		mutex_lock(&cctx->wake_mutex);
		fastrpc_pm_awake(fl, fl->cctx->secure);
		mutex_unlock(&cctx->wake_mutex);
		break;
	case FASTRPC_CONTROL_DSPPROCESS_CLEAN:
		err = fastrpc_release_current_dsp_process(fl);
		if (!err)
			fastrpc_queue_pd_status(fl, fl->cctx->domain_id, FASTRPC_USERPD_FORCE_KILL, fl->sessionid);
		break;
	case FASTRPC_CONTROL_RPC_POLL:
		err = fastrpc_manage_poll_mode(fl, cp->lp.enable, cp->lp.latency);
		break;
	case FASTRPC_CONTROL_NOTIF_WAKE:
		fl->exit_notif = true;
		spin_lock_irqsave(&fl->proc_state_notif.nqlock, flags);
		atomic_add(1, &fl->proc_state_notif.notif_queue_count);
		wake_up_interruptible(&fl->proc_state_notif.notif_wait_queue);
		spin_unlock_irqrestore(&fl->proc_state_notif.nqlock, flags);
		break;
	default:
		err = -EBADRQC;
		break;
	}
	return err;
}

static int fastrpc_set_session_info(
		struct fastrpc_user *fl, struct fastrpc_internal_sessinfo *sessinfo)
{
	spin_lock(&fl->lock);
	if (fl->set_session_info) {
		spin_unlock(&fl->lock);
		dev_err(fl->cctx->dev,"Set session info invoked multiple times\n");
		return -EBADR;
	}
	fl->set_session_info = true;
	spin_unlock(&fl->lock);

	if(sessinfo->pd <= DEFAULT_UNUSED ||
				sessinfo->pd >= MAX_PD_TYPE) {
		dev_err(fl->cctx->dev,"Invalid PD type %d, range is %d - %d\n",
					sessinfo->pd, DEFAULT_UNUSED + 1, MAX_PD_TYPE - 1);
		return -EBADR;
	}

	/*
	 * If PD type is not configured for context banks,
	 * ignore PD type passed by the user, leave pd_type set to DEFAULT_UNUSED(0)
	 */
	if (fl->cctx->pd_type)
		fl->pd_type = sessinfo->pd;
	// Processes attaching to Sensor Static PD, share context bank.
	if (sessinfo->pd == SENSORS_STATICPD)
		fl->sharedcb = 1;
	if (sessinfo->session_id >= fl->cctx->max_sess_per_proc) {
		dev_err(fl->cctx->dev,
		"Session ID %u cannot be beyond %u\n",
				sessinfo->session_id, fl->cctx->max_sess_per_proc);
		return -EBADR;
	}
	fl->sessionid = sessinfo->session_id;
	// Set multi_session_support, to disable old way of setting session_id
	fl->multi_session_support = true;

	return 0;
}

static int fastrpc_dspsignal_signal(struct fastrpc_user *fl,
			     struct fastrpc_internal_dspsignal *fsig)
{
	int err = 0;
	struct fastrpc_channel_ctx *cctx = NULL;
	u64 msg = 0;
	u32 signal_id = fsig->signal_id;

	dev_dbg(fl->cctx->dev, "Send signal PID %u, unique fastrpc pid %u signal %u\n",
					fl->tgid, fl->tgid_frpc, signal_id);
	cctx = fl->cctx;
	if (!(signal_id < FASTRPC_DSPSIGNAL_NUM_SIGNALS)) {
		dev_err(fl->cctx->dev, "Sending bad signal %u for PID %u",
				signal_id, fl->tgid);
		return -EINVAL;
	}

	msg = (((uint64_t)fl->tgid_frpc) << 32) | ((uint64_t)fsig->signal_id);
	err = fastrpc_transport_send(cctx, (void *)&msg, sizeof(msg));
	trace_fastrpc_dspsignal("signal", signal_id, 0, 0);

	return err;
}

int fastrpc_dspsignal_wait(struct fastrpc_user *fl,
			     struct fastrpc_internal_dspsignal *fsig)
{
	int err = 0;
	unsigned long timeout = usecs_to_jiffies(fsig->timeout_usec);
	u32 signal_id = fsig->signal_id;
	struct fastrpc_dspsignal *s = NULL;
	long ret = 0;
	unsigned long irq_flags = 0;

	dev_dbg(fl->cctx->dev, "Wait for signal %u\n", signal_id);
	if (!(signal_id <FASTRPC_DSPSIGNAL_NUM_SIGNALS)) {
		dev_err(fl->cctx->dev, "Waiting on bad signal %u\n", signal_id);
		return -EINVAL;
	}

	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);
	if (fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE] != NULL) {
		struct fastrpc_dspsignal *group =
			fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE];

		s = &group[signal_id %FASTRPC_DSPSIGNAL_GROUP_SIZE];
	}
	if ((s == NULL) || (s->state == DSPSIGNAL_STATE_UNUSED)) {
		spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
		dev_err(fl->cctx->dev, "Unknown signal id %u\n", signal_id);
		return -ENOENT;
	}
	if (s->state != DSPSIGNAL_STATE_PENDING) {
		if ((s->state == DSPSIGNAL_STATE_CANCELED) || (s->state == DSPSIGNAL_STATE_UNUSED))
			err = -EINTR;
		if (s->state == DSPSIGNAL_STATE_SIGNALED) {
			/* Signal already received from DSP. Reset signal state and return */
			s->state = DSPSIGNAL_STATE_PENDING;
			reinit_completion(&s->comp);
		}
		spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
		dev_dbg(fl->cctx->dev, "Signal %u in state %u, complete wait immediately",
				signal_id, s->state);
		return err;
	}
	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
	trace_fastrpc_dspsignal("wait", signal_id, s->state, fsig->timeout_usec);
	if (timeout != 0xffffffff)
		ret = wait_for_completion_interruptible_timeout(&s->comp, timeout);
	else
		ret = wait_for_completion_interruptible(&s->comp);
	trace_fastrpc_dspsignal("wakeup", signal_id, s->state, fsig->timeout_usec);

	if (ret == 0) {
		dev_dbg(fl->cctx->dev, "Wait for signal %u timed out\n", signal_id);
		return -ETIMEDOUT;
	} else if (ret < 0) {
		dev_err(fl->cctx->dev, "Wait for signal %u failed %d\n", signal_id, (int)ret);
		return ret;
	}

	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);
	if (s->state == DSPSIGNAL_STATE_SIGNALED) {
		s->state = DSPSIGNAL_STATE_PENDING;
		dev_dbg(fl->cctx->dev, "Signal %u completed\n", signal_id);
	} else if ((s->state == DSPSIGNAL_STATE_CANCELED) || (s->state == DSPSIGNAL_STATE_UNUSED)) {
		dev_dbg(fl->cctx->dev, "Signal %u cancelled or destroyed\n", signal_id);
		err = -EINTR;
	}
	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);

	return err;
}

static int fastrpc_dspsignal_create(struct fastrpc_user *fl,
			     struct fastrpc_internal_dspsignal *fsig)
{
	int err = 0;
	u32 signal_id = fsig->signal_id;
	struct fastrpc_dspsignal *group, *sig;
	unsigned long irq_flags = 0;

	if (!(signal_id <FASTRPC_DSPSIGNAL_NUM_SIGNALS))
		return -EINVAL;

	mutex_lock(&fl->signal_create_mutex);
	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);

	group = fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE];
	if (group == NULL) {
		int i;
		spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
		group = kzalloc(FASTRPC_DSPSIGNAL_GROUP_SIZE * sizeof(*group),
					     GFP_KERNEL);
		if (group == NULL) {
			dev_err(fl->cctx->dev, "Unable to allocate signal group\n");
			mutex_unlock(&fl->signal_create_mutex);
			return -ENOMEM;
		}

		for (i = 0; i < FASTRPC_DSPSIGNAL_GROUP_SIZE; i++) {
			sig = &group[i];
			init_completion(&sig->comp);
			sig->state = DSPSIGNAL_STATE_UNUSED;
		}
		spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);
		fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE] = group;
	}

	sig = &group[signal_id %FASTRPC_DSPSIGNAL_GROUP_SIZE];
	if (sig->state != DSPSIGNAL_STATE_UNUSED) {
		spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
		mutex_unlock(&fl->signal_create_mutex);
		dev_err(fl->cctx->dev,"Attempting to create signal %u already in use (state %u)\n",
			    signal_id, sig->state);
		return -EBUSY;
	}

	sig->state = DSPSIGNAL_STATE_PENDING;
	reinit_completion(&sig->comp);

	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
	mutex_unlock(&fl->signal_create_mutex);
	dev_dbg(fl->cctx->dev, "Signal %u created\n", signal_id);

	return err;
}

static int fastrpc_dspsignal_destroy(struct fastrpc_user *fl,
			      struct fastrpc_internal_dspsignal *fsig)
{
	u32 signal_id = fsig->signal_id;
	struct fastrpc_dspsignal *s = NULL;
	unsigned long irq_flags = 0;

	dev_dbg(fl->cctx->dev, "Destroy signal %u\n", signal_id);
	if (!(signal_id <FASTRPC_DSPSIGNAL_NUM_SIGNALS))
		return -EINVAL;

	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);

	if (fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE] != NULL) {
		struct fastrpc_dspsignal *group =
			fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE];

		s = &group[signal_id % FASTRPC_DSPSIGNAL_GROUP_SIZE];
	}
	if ((s == NULL) || (s->state == DSPSIGNAL_STATE_UNUSED)) {
		spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
		dev_err(fl->cctx->dev,"Attempting to destroy unused signal %u\n", signal_id);
		return -ENOENT;
	}

	s->state = DSPSIGNAL_STATE_UNUSED;
	complete_all(&s->comp);

	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
	dev_dbg(fl->cctx->dev, "Signal %u destroyed\n", signal_id);

	return 0;
}

static int fastrpc_dspsignal_cancel_wait(struct fastrpc_user *fl,
				  struct fastrpc_internal_dspsignal *fsig)
{
	u32 signal_id = fsig->signal_id;
	struct fastrpc_dspsignal *s = NULL;
	unsigned long irq_flags = 0;

	dev_dbg(fl->cctx->dev, "Cancel wait for signal %u\n", signal_id);
	if (!(signal_id <FASTRPC_DSPSIGNAL_NUM_SIGNALS))
		return -EINVAL;

	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);

	if (fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE] != NULL) {
		struct fastrpc_dspsignal *group =
			fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE];

		s = &group[signal_id %FASTRPC_DSPSIGNAL_GROUP_SIZE];
	}
	if ((s == NULL) || (s->state == DSPSIGNAL_STATE_UNUSED)) {
		spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
		dev_err(fl->cctx->dev,"Attempting to cancel unused signal %u\n", signal_id);
		return -ENOENT;
	}

	if (s->state != DSPSIGNAL_STATE_CANCELED) {
		s->state = DSPSIGNAL_STATE_CANCELED;
		trace_fastrpc_dspsignal("cancel", signal_id, s->state, 0);
		complete_all(&s->comp);
	}

	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
	dev_dbg(fl->cctx->dev, "Signal %u cancelled\n", signal_id);

	return 0;
}

/**
 * fastrpc_ssr_dspsignal_cancel_wait() -
 * Function to cancel waiting signals during SSR
 * @arg1: Fastrpc user file pointer
 *
 * dspsignals will be waiting for DSP response
 * cancel wait for these signals during SSR
 *
 * Return: void
 */
void fastrpc_ssr_dspsignal_cancel_wait(struct fastrpc_user *fl)
{
	unsigned long irq_flags = 0;
	unsigned int i, j;
	struct fastrpc_dspsignal *group, *sig;

	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);
	for (i = 0; i < (FASTRPC_DSPSIGNAL_NUM_SIGNALS /
	         FASTRPC_DSPSIGNAL_GROUP_SIZE); i++) {
		group = fl->signal_groups[i];
		if (group) {
			for (j = 0; j < FASTRPC_DSPSIGNAL_GROUP_SIZE;
			     j++) {
				sig = &group[j];
				if (sig && sig->state ==
				    DSPSIGNAL_STATE_PENDING) {
					complete_all(&sig->comp);
					sig->state =
					    DSPSIGNAL_STATE_CANCELED;
				}
			}
		}
	}
	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
}

static int fastrpc_invoke_dspsignal(struct fastrpc_user *fl, struct fastrpc_internal_dspsignal *fsig)
{
	int err = 0;

	switch(fsig->req) {
	case FASTRPC_DSPSIGNAL_SIGNAL:
		err = fastrpc_dspsignal_signal(fl,fsig);
		break;
	case FASTRPC_DSPSIGNAL_WAIT :
		err = fastrpc_dspsignal_wait(fl,fsig);
		break;
	case FASTRPC_DSPSIGNAL_CREATE :
		err = fastrpc_dspsignal_create(fl,fsig);
		break;
	case FASTRPC_DSPSIGNAL_DESTROY :
		err = fastrpc_dspsignal_destroy(fl,fsig);
		break;
	case FASTRPC_DSPSIGNAL_CANCEL_WAIT :
		err = fastrpc_dspsignal_cancel_wait(fl,fsig);
		break;
	}
	return err;
}

static int fastrpc_multimode_invoke(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_enhanced_invoke inv2 ;
	struct fastrpc_ioctl_multimode_invoke invoke;
	struct fastrpc_internal_control cp = {0};
	struct fastrpc_internal_dspsignal *fsig = NULL;
	struct fastrpc_internal_notif_rsp notif;
	struct fastrpc_internal_config config;
	struct fastrpc_internal_sessinfo sessinfo;
	u32 multisession, size = 0;
	u64 *perf_kernel;
	int err = 0;

	if (copy_from_user(&invoke, argp, sizeof(invoke)))
		return -EFAULT;
	switch (invoke.req) {
	case FASTRPC_INVOKE:
		size = sizeof(struct fastrpc_ioctl_multimode_invoke);
		fallthrough;
	case FASTRPC_INVOKE_ENHANCED:
		/* nscalars is truncated here to max supported value */
		if (!size)
			size = sizeof(struct fastrpc_enhanced_invoke);
		if (copy_from_user(&inv2, (void __user *)(uintptr_t)invoke.invparam,
				   size))
			return -EFAULT;
		perf_kernel = (u64 *)(uintptr_t)inv2.perf_kernel;
		if (perf_kernel)
			fl->profile = true;
		err = fastrpc_internal_invoke(fl, USER_MSG, &inv2);
		break;
	case FASTRPC_INVOKE_CONTROL:
		if (copy_from_user(&cp, (void __user *)(uintptr_t)invoke.invparam, sizeof(cp)))
			return  -EFAULT;

		err = fastrpc_internal_control(fl, &cp);
		break;
	case FASTRPC_INVOKE_DSPSIGNAL:
		if (invoke.size > sizeof(*fsig))
			return -EINVAL;
		fsig = kzalloc(sizeof(*fsig), GFP_KERNEL);
		if (!fsig)
			return -ENOMEM;
		if (copy_from_user(fsig, (void __user *)(uintptr_t)invoke.invparam,
				invoke.size)) {
			kfree(fsig);
			return -EFAULT;
		}
		err = fastrpc_invoke_dspsignal(fl, fsig);
		kfree(fsig);
		break;
	case FASTRPC_INVOKE_NOTIF:
		err = fastrpc_get_notif_response(&notif,
						(void *)invoke.invparam, fl);
		break;
	case FASTRPC_INVOKE_MULTISESSION:
		if (copy_from_user(&multisession, (void __user *)(uintptr_t)invoke.invparam, sizeof(multisession)))
			return  -EFAULT;
		if(!fl->multi_session_support)
			fl->sessionid = 1;
		break;
	case FASTRPC_INVOKE_CONFIG:
		size = sizeof(struct fastrpc_internal_config);
		/* Copy with which ever is miminum size, ensures backward compatibility */
		if (invoke.size < size )
			size = invoke.size; 
		if (copy_from_user(&config, (void __user *)(uintptr_t)invoke.invparam,
			size))
			return -EFAULT;
		fl->config.user_fd = config.user_fd;
		fl->config.user_size = config.user_size;
		fl->config.root_addr = config.root_addr;
		fl->config.root_size = config.root_size;
		break;
	case FASTRPC_INVOKE_SESSIONINFO:
		if(copy_from_user(&sessinfo,(void __user *)(uintptr_t)invoke.invparam,
			sizeof(struct fastrpc_internal_sessinfo)))
			return -EFAULT;
		err = fastrpc_set_session_info(fl, &sessinfo);
		break;
	default:
		err = -ENOTTY;
		break;
	}
	return err;
}

static int fastrpc_get_info_from_dsp(struct fastrpc_user *fl, uint32_t *dsp_attr_buf,
				     uint32_t dsp_attr_buf_len)
{
	struct fastrpc_invoke_args args[2] = { 0 };
	struct fastrpc_enhanced_invoke ioctl;

	/* Capability filled in userspace */
	dsp_attr_buf[0] = 0;
	dsp_attr_buf_len -= 1;

	args[0].ptr = (u64)(uintptr_t)&dsp_attr_buf_len;
	args[0].length = sizeof(dsp_attr_buf_len);
	args[0].fd = -1;
	args[1].ptr = (u64)(uintptr_t)&dsp_attr_buf[1];
	args[1].length = dsp_attr_buf_len * sizeof(uint32_t);
	args[1].fd = -1;

	ioctl.inv.handle = FASTRPC_DSP_UTILITIES_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(0, 1, 1);
	ioctl.inv.args = (__u64)args;

	return fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
}

static int fastrpc_get_info_from_kernel(struct fastrpc_ioctl_capability *cap,
					struct fastrpc_user *fl)
{
	struct fastrpc_channel_ctx *cctx = fl->cctx;
	uint32_t attribute_id = cap->attribute_id;
	uint32_t *dsp_attributes;
	unsigned long flags;
	uint32_t domain = cap->domain;
	int err;

	spin_lock_irqsave(&cctx->lock, flags);
	/* check if we already have queried dsp for attributes */
	if (cctx->valid_attributes) {
		spin_unlock_irqrestore(&cctx->lock, flags);
		goto done;
	}
	spin_unlock_irqrestore(&cctx->lock, flags);

	dsp_attributes = kzalloc(FASTRPC_MAX_DSP_ATTRIBUTES_LEN, GFP_KERNEL);
	if (!dsp_attributes)
		return -ENOMEM;

	err = fastrpc_get_info_from_dsp(fl, dsp_attributes, FASTRPC_MAX_DSP_ATTRIBUTES);
	if (err == DSP_UNSUPPORTED_API) {
		dev_info(cctx->dev,
			 "Warning: DSP capabilities not supported on domain: %d\n", domain);
		kfree(dsp_attributes);
		return -EOPNOTSUPP;
	} else if (err) {
		dev_dbg(cctx->dev, "Failed to get dsp information err: %d\n", err);
		kfree(dsp_attributes);
		return err;
	}

	spin_lock_irqsave(&cctx->lock, flags);
	memcpy(cctx->dsp_attributes, dsp_attributes, FASTRPC_MAX_DSP_ATTRIBUTES_LEN);
	cctx->valid_attributes = true;
	spin_unlock_irqrestore(&cctx->lock, flags);
	kfree(dsp_attributes);
done:
	cap->capability = cctx->dsp_attributes[attribute_id];
	return 0;
}

static int fastrpc_get_dsp_info(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_ioctl_capability cap = {0};
	int err = 0;

	if (copy_from_user(&cap, argp, sizeof(cap)))
		return  -EFAULT;

	cap.capability = 0;
	if (cap.domain >= FASTRPC_DEV_MAX) {
		dev_err(fl->cctx->dev, "Error: Invalid domain id:%d, err:%d\n",
			cap.domain, err);
		return -ECHRNG;
	}

	/* Fastrpc Capablities does not support modem domain */
	if (cap.domain == MDSP_DOMAIN_ID) {
		dev_err(fl->cctx->dev, "Error: modem not supported %d\n", err);
		return -ECHRNG;
	}

	if (cap.attribute_id >= FASTRPC_MAX_DSP_ATTRIBUTES) {
		dev_err(fl->cctx->dev, "Error: invalid attribute: %d, err: %d\n",
			cap.attribute_id, err);
		return -EOVERFLOW;
	}

	err = fastrpc_get_info_from_kernel(&cap, fl);
	if (err)
		return err;

	if (copy_to_user(argp, &cap, sizeof(cap)))
		return -EFAULT;

	return 0;
}

static int fastrpc_req_munmap_dsp(struct fastrpc_user *fl, uintptr_t raddr, u64 size) {

	struct fastrpc_invoke_args args[1] = { [0] = { 0 } };
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_munmap_req_msg req_msg;
	int err = 0;

	req_msg.pgid = fl->tgid_frpc;
	req_msg.size = size;
	req_msg.vaddr = raddr;

	args[0].ptr = (u64) (uintptr_t) &req_msg;
	args[0].length = sizeof(req_msg);

	ioctl.inv.handle = FASTRPC_INIT_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_MUNMAP, 1, 0);
	ioctl.inv.args = (__u64)args;

	err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
	/* error to be printed by caller function */
	return err;

}

static int fastrpc_req_munmap_impl(struct fastrpc_user *fl, struct fastrpc_buf *buf)
{
	struct device *dev = fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	int err;

	err = fastrpc_req_munmap_dsp(fl, buf->raddr, buf->size);
	if (!err) {
		if (buf->type == REMOTEHEAP_BUF) {
			if (fl->cctx->vmcount) {
				u64 src_perms = 0;
				struct qcom_scm_vmperm dst_perms;
				u32 i;

				for (i = 0; i < fl->cctx->vmcount; i++)
					src_perms |= BIT(fl->cctx->vmperms[i].vmid);

				dst_perms.vmid = QCOM_SCM_VMID_HLOS;
				dst_perms.perm = QCOM_SCM_PERM_RWX;
				err = qcom_scm_assign_mem(buf->phys, (u64)buf->size,
								&src_perms, &dst_perms, 1);
				if (err) {
					dev_err(dev,
				"%s: Failed to assign memory phys 0x%llx size 0x%llx err %d",
						__func__, buf->phys, buf->size, err);
					return err;
				}
			}
		}
		dev_dbg(dev, "unmmap\tpt 0x%09lx OK\n", buf->raddr);
	} else {
		dev_err(dev, "unmmap\tpt 0x%09lx ERROR\n", buf->raddr);
	}

	return err;
}

static int fastrpc_req_munmap(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_buf *buf = NULL, *iter, *b;
	struct fastrpc_req_munmap req;
	struct fastrpc_map *map = NULL, *iterm, *m;
	struct device *dev = NULL;
	int err = -EINVAL;
	unsigned long flags;

	if (atomic_read(&fl->state) != DSP_CREATE_COMPLETE) {
		dev_err(fl->cctx->dev,
			" %s: %s: trying to unmap buf before creating remote session\n",
			__func__, current->comm);
		return -EHOSTDOWN;
	}
	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	dev = fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	spin_lock(&fl->lock);
	list_for_each_entry_safe(iter, b, &fl->mmaps, node) {
		if ((iter->raddr == req.vaddrout) && (iter->size == req.size)) {
			buf = iter;
			list_del(&buf->node);
			break;
		}
	}
	spin_unlock(&fl->lock);

	if (buf) {
		err = fastrpc_req_munmap_impl(fl, buf);
		if(!err) {
			fastrpc_buf_free(buf, false);
		} else {
			spin_lock(&fl->lock);
			list_add_tail(&buf->node, &fl->mmaps);
			spin_unlock(&fl->lock);
		}
		return err;
	}

	spin_lock_irqsave(&fl->cctx->lock, flags);
	list_for_each_entry_safe(iter, b, &fl->cctx->gmaps, node) {
		if ((iter->raddr == req.vaddrout) && (iter->size == req.size)) {
			buf = iter;
			list_del(&buf->node);
			break;
		}
	}
	spin_unlock_irqrestore(&fl->cctx->lock, flags);

	if (buf) {
		err = fastrpc_req_munmap_impl(fl, buf);
		if(!err) {
			fastrpc_buf_free(buf, false);
		} else {
			spin_lock_irqsave(&fl->cctx->lock, flags);
			list_add_tail(&buf->node, &fl->cctx->gmaps);
			spin_unlock_irqrestore(&fl->cctx->lock, flags);
		}
		return err;
	}

	spin_lock(&fl->lock);
	list_for_each_entry_safe(iterm, m, &fl->maps, node) {
		if (iterm->raddr == req.vaddrout) {
			/*
			 * Check if DSP mapping is complete, then move the state to
			 * unmap in progress only if there is no other ongoing unmap.
			 */
			if (atomic_cmpxchg(&iterm->state, FD_DSP_MAP_COMPLETE,
				FD_DSP_UNMAP_IN_PROGRESS) != FD_DSP_MAP_COMPLETE)
				err = -EALREADY;
			else
				map = iterm;
			break;
		}
	}
	spin_unlock(&fl->lock);
	if (!map) {
		dev_err(dev, "buffer not in buf or map list\n");
		return err;
	}

	err = fastrpc_req_munmap_dsp(fl, map->raddr, map->size);
	if (err) {
		dev_err(dev, "unmmap\tpt fd = %d, 0x%09llx error\n",  map->fd, map->raddr);
		/* Revert the map state to map complete */
		atomic_set(&map->state, FD_DSP_MAP_COMPLETE);
	} else {
		/* Set the map state to default on successful unmapping */
		atomic_set(&map->state, FD_MAP_DEFAULT);
		mutex_lock(&fl->map_mutex);
		fastrpc_map_put(map);
		mutex_unlock(&fl->map_mutex);
	}

	return err;
}

static int fastrpc_req_mmap(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_invoke_args args[3] = { [0 ... 2] = { 0 } };
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_buf *buf = NULL;
	struct fastrpc_mmap_req_msg req_msg;
	struct fastrpc_mmap_rsp_msg rsp_msg;
	struct fastrpc_phy_page pages;
	struct fastrpc_req_mmap req;
	struct fastrpc_map *map = NULL;
	struct fastrpc_smmu *smmucb = NULL;
	struct device *dev = NULL;
	struct timespec64 start_ts, end_ts;
	int err;
	unsigned long flags;

	if (atomic_read(&fl->state) != DSP_CREATE_COMPLETE) {
		dev_err(fl->cctx->dev,
			"%s: %s: trying to map buf before creating remote session\n",
			__func__, current->comm);
		return -EHOSTDOWN;
	}
	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;
	if (!req.size)
		return -EFAULT;

	smmucb = &fl->sctx->smmucb[DEFAULT_SMMU_IDX];
	dev = smmucb->dev;
	if ((req.flags == ADSP_MMAP_ADD_PAGES ||
		req.flags == ADSP_MMAP_REMOTE_HEAP_ADDR) && !fl->is_unsigned_pd) {
		if (req.vaddrin) {
			dev_err(dev,
			"adding user allocated pages is only supported for unsigned PD\n");
			return -EINVAL;
		}

		if (req.flags == ADSP_MMAP_REMOTE_HEAP_ADDR) {
			err = fastrpc_buf_alloc(fl, NULL, req.size, REMOTEHEAP_BUF, &buf);
		} else {
			err = fastrpc_smmu_buf_alloc(fl, req.size, USER_BUF, &buf);
		}

		if (err) {
			dev_err(dev, "failed to allocate buffer\n");
			return err;
		}

		/*
		 * Update dev with correct SMMU device,
		 * on which the memory is allocated.
		 */
		if (req.flags == ADSP_MMAP_ADD_PAGES)
			dev = buf->smmucb->dev;

		req_msg.pgid = fl->tgid_frpc;
		req_msg.flags = req.flags;
		req_msg.vaddr = req.vaddrin;
		req_msg.num = sizeof(pages);

		args[0].ptr = (u64) (uintptr_t) &req_msg;
		args[0].length = sizeof(req_msg);

		pages.addr = buf->phys;
		pages.size = buf->size;

		args[1].ptr = (u64) (uintptr_t) &pages;
		args[1].length = sizeof(pages);

		args[2].ptr = (u64) (uintptr_t) &rsp_msg;
		args[2].length = sizeof(rsp_msg);

		ioctl.inv.handle = FASTRPC_INIT_HANDLE;
		ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_MMAP, 2, 1);
		ioctl.inv.args = (__u64)args;

		err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
		if (err) {
			dev_err(dev, "mmap error (len 0x%08llx)\n", buf->size);
			goto err_invoke;
		}

		/* update the buffer to be able to deallocate the memory on the DSP */
		buf->raddr = (uintptr_t) rsp_msg.vaddr;

		/* let the client know the address to use */
		req.vaddrout = rsp_msg.vaddr;

		/* Add memory to static PD pool, protection thru hypervisor */
		if (req.flags == ADSP_MMAP_REMOTE_HEAP_ADDR && fl->cctx->vmcount) {
			u64 src_perms = BIT(QCOM_SCM_VMID_HLOS);

			ktime_get_boottime_ts64(&start_ts);
			err = qcom_scm_assign_mem(buf->phys,(u64)buf->size,
				&src_perms, fl->cctx->vmperms, fl->cctx->vmcount);
			ktime_get_boottime_ts64(&end_ts);
			buf->scm_assign_time = timespec64_sub(end_ts, start_ts);
			if (err) {
				dev_err(dev, "Failed to assign memory phys 0x%llx size 0x%llx err %d",
						buf->phys, buf->size, err);
				goto err_assign;
			}
		}
		if (req.flags == ADSP_MMAP_REMOTE_HEAP_ADDR) {
			spin_lock_irqsave(&fl->cctx->lock, flags);
			list_add_tail(&buf->node, &fl->cctx->gmaps);
			spin_unlock_irqrestore(&fl->cctx->lock, flags);
		} else {
			spin_lock(&fl->lock);
			list_add_tail(&buf->node, &fl->mmaps);
			spin_unlock(&fl->lock);
		}
		if (copy_to_user((void __user *)argp, &req, sizeof(req)))
			/*
			 * The usercopy failed, but we can't do much about it, as this
			 * buf is already mapped in the DSP and accessible for the
			 * current process. Therefore "leak" the buf and rely on the
			 * process exit path to do any required cleanup.
			 */
			return -EFAULT;

	} else {
		if ((req.flags == ADSP_MMAP_REMOTE_HEAP_ADDR) && fl->is_unsigned_pd) {
			dev_err(dev, "remote heap is not supported for unsigned PD\n");
			return -EINVAL;
		}
		mutex_lock(&fl->map_mutex);
		err = fastrpc_map_create(fl, req.fd, req.vaddrin, NULL, req.size, 0, 0, &map, true);
		mutex_unlock(&fl->map_mutex);
		if (err) {
			dev_err(dev, "failed to map buffer, fd = %d\n", req.fd);
			return err;
		}
		/*
		 * Update the map state to in progress only if there is no ongoing or
		 * completed DSP mapping.
		 */
		if (atomic_cmpxchg(&map->state, FD_MAP_DEFAULT, FD_DSP_MAP_IN_PROGRESS)
			!= FD_MAP_DEFAULT) {
			err = -EALREADY;
			goto err_invoke;
		}
		req_msg.pgid = fl->tgid_frpc;
		req_msg.flags = req.flags;
		req_msg.vaddr = req.vaddrin;
		req_msg.num = sizeof(pages);

		args[0].ptr = (u64) (uintptr_t) &req_msg;
		args[0].length = sizeof(req_msg);

		pages.addr = map->phys;
		pages.size = map->size;

		args[1].ptr = (u64) (uintptr_t) &pages;
		args[1].length = sizeof(pages);

		args[2].ptr = (u64) (uintptr_t) &rsp_msg;
		args[2].length = sizeof(rsp_msg);

		ioctl.inv.handle = FASTRPC_INIT_HANDLE;
		ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_MMAP, 2, 1);
		ioctl.inv.args = (__u64)args;

		err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
		if (err) {
			dev_err(dev, "mmap error (len 0x%08llx)\n", map->size);
			/* Revert the map state to default */
			atomic_set(&map->state, FD_MAP_DEFAULT);
			goto err_invoke;
		}

		/* update the buffer to be able to deallocate the memory on the DSP */
		map->raddr = (uintptr_t) rsp_msg.vaddr;

		/* let the client know the address to use */
		req.vaddrout = rsp_msg.vaddr;
		/* Set the map state to complete on successful mapping */
		atomic_set(&map->state, FD_DSP_MAP_COMPLETE);
		if (copy_to_user((void __user *)argp, &req, sizeof(req)))
			/*
			 * The usercopy failed, but we can't do much about it, as this
			 * map is already mapped in the DSP and accessible for the
			 * current process. Therefore "leak" the map and rely on the
			 * process exit path to do any required cleanup.
			 */
			return -EFAULT;

	}
	return 0;

err_assign:
	err = fastrpc_req_munmap_dsp(fl, buf->raddr, buf->size);
	if (err) {
		if (req.flags == ADSP_MMAP_REMOTE_HEAP_ADDR) {
			spin_lock_irqsave(&fl->cctx->lock, flags);
			list_add_tail(&buf->node, &fl->cctx->gmaps);
			spin_unlock_irqrestore(&fl->cctx->lock, flags);
		} else {
			spin_lock(&fl->lock);
			list_add_tail(&buf->node, &fl->mmaps);
			spin_unlock(&fl->lock);
		}
		buf = NULL;
	}

err_invoke:
	if (map) {
		mutex_lock(&fl->map_mutex);
		fastrpc_map_put(map);
		mutex_unlock(&fl->map_mutex);
	}
	if (buf)
		fastrpc_buf_free(buf, false);

	return err;
}

static int fastrpc_req_mem_unmap_impl(struct fastrpc_user *fl, struct fastrpc_mem_unmap *req)
{
	struct fastrpc_invoke_args args[1] = { [0] = { 0 } };
	struct fastrpc_enhanced_invoke ioctl;
	struct fastrpc_map *map = NULL, *iter, *m;
	struct fastrpc_mem_unmap_req_msg req_msg = { 0 };
	int err = -EINVAL;
	struct device *dev = fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;

	spin_lock(&fl->lock);
	list_for_each_entry_safe(iter, m, &fl->maps, node) {
		if ((req->fd < 0 || iter->fd == req->fd) && (iter->raddr == req->vaddr)) {
			/*
			 * Check if DSP mapping is complete, then move the state to
			 * unmap in progress only if there is no other ongoing unmap.
			 */
			if (atomic_cmpxchg(&iter->state, FD_DSP_MAP_COMPLETE,
				FD_DSP_UNMAP_IN_PROGRESS) != FD_DSP_MAP_COMPLETE)
				err = -EALREADY;
			else
				map = iter;
			break;
		}
	}

	spin_unlock(&fl->lock);

	if (!map) {
		dev_err(dev, "map not in list\n");
		return err;
	}

	req_msg.pgid = fl->tgid_frpc;
	req_msg.len = map->len;
	req_msg.vaddrin = map->raddr;
	req_msg.fd = map->fd;

	args[0].ptr = (u64) (uintptr_t) &req_msg;
	args[0].length = sizeof(req_msg);

	ioctl.inv.handle = FASTRPC_INIT_HANDLE;
	ioctl.inv.sc = FASTRPC_SCALARS(FASTRPC_RMID_INIT_MEM_UNMAP, 1, 0);
	ioctl.inv.args = (__u64)args;

	err = fastrpc_internal_invoke(fl, KERNEL_MSG_WITH_ZERO_PID, &ioctl);
	if (err) {
		dev_err(dev, "Unmap on DSP failed for fd:%d, addr:0x%09llx\n",  map->fd, map->raddr);
		/* Revert the map state to map complete */
		atomic_set(&map->state, FD_DSP_MAP_COMPLETE);
		return err;
	}
	/* Set the map state to default on successful unmapping */
	atomic_set(&map->state, FD_MAP_DEFAULT);
	mutex_lock(&fl->map_mutex);
	fastrpc_map_put(map);
	mutex_unlock(&fl->map_mutex);
	return 0;
}

static int fastrpc_req_mem_unmap(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_mem_unmap req;

	if (atomic_read(&fl->state) != DSP_CREATE_COMPLETE) {
		dev_err(fl->cctx->dev,
			"%s: %s: trying to unmap buf before creating remote session\n",
			__func__, current->comm);
		return -EHOSTDOWN;
	}
	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;

	return fastrpc_req_mem_unmap_impl(fl, &req);
}

static int fastrpc_req_mem_map(struct fastrpc_user *fl, char __user *argp)
{
	struct fastrpc_mem_map req = {0};
	struct device *dev = NULL;
	struct fastrpc_map *map = NULL;
	int err;

	if (atomic_read(&fl->state) != DSP_CREATE_COMPLETE) {
		dev_err(fl->cctx->dev,
			"%s: %s: trying to map buf before creating remote session\n",
			__func__, current->comm);
		return -EHOSTDOWN;
	}
	if (copy_from_user(&req, argp, sizeof(req)))
		return -EFAULT;
	/*
	 * Prevent mapping backward compatible DMA handles here, as they are
	 * already mapped in the remote call.
	 */
	if (req.flags == FASTRPC_MAP_LEGACY_DMA_HANDLE)
		return -EINVAL;
	dev = fl->sctx->smmucb[DEFAULT_SMMU_IDX].dev;
	/* create SMMU mapping */
	mutex_lock(&fl->map_mutex);
	err = fastrpc_map_create(fl, req.fd, req.vaddrin, NULL, req.length, req.attrs, req.flags, &map, true);
	mutex_unlock(&fl->map_mutex);
	if (err) {
		dev_err(dev, "failed to map buffer, fd = %d\n", req.fd);
		return err;
	}
	/*
	 * Update the map state to in progress only if there is no ongoing or
	 * completed DSP mapping.
	 */
	if (atomic_cmpxchg(&map->state, FD_MAP_DEFAULT, FD_DSP_MAP_IN_PROGRESS)
		!= FD_MAP_DEFAULT) {
		err = -EALREADY;
		goto err_invoke;
	}
	map->va = (void *) (uintptr_t) req.vaddrin;
	/* map to dsp, get virtual adrress for the user*/
	err = fastrpc_mem_map_to_dsp(fl, map->fd, req.offset,
					req.flags, req.vaddrin, map->phys,
					map->size, (uintptr_t *)&req.vaddrout);
	if (err) {
		dev_err(dev, "failed to map buffer on dsp, fd = %d\n", map->fd);
		/* Revert the map state to default */
		atomic_set(&map->state, FD_MAP_DEFAULT);
		goto err_invoke;
	}

	/* update the buffer to be able to deallocate the memory on the DSP */
	map->raddr = req.vaddrout;
	/* Set the map state to complete on successful mapping */
	atomic_set(&map->state, FD_DSP_MAP_COMPLETE);
	if (copy_to_user((void __user *)argp, &req, sizeof(req)))
		/*
		 * The usercopy failed, but we can't do much about it, as this
		 * map is already mapped in the DSP and accessible for the
		 * current process. Therefore "leak" the map and rely on the
		 * process exit path to do any required cleanup.
		 */
		return -EFAULT;

	return 0;
err_invoke:
	mutex_lock(&fl->map_mutex);
	fastrpc_map_put(map);
	mutex_unlock(&fl->map_mutex);

	return err;
}

static long fastrpc_device_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct fastrpc_user *fl = (struct fastrpc_user *)file->private_data;
	struct fastrpc_channel_ctx *cctx = fl->cctx;
	char __user *argp = (char __user *)arg;
	int err;
	int process_init = 0;
	unsigned long flags = 0;

	fastrpc_channel_ctx_get(cctx);
	spin_lock_irqsave(&cctx->lock, flags);
	if (atomic_read(&cctx->teardown)) {
		/* If subsystem already going thru SSR, then fail ioctl immediately */
		spin_unlock_irqrestore(&cctx->lock, flags);
		fastrpc_channel_ctx_put(cctx);
		return -EPIPE;
	}
	/*
	 * Update invoke count to block SSR handling thread from cleaning up
	 * the channel resources, while it is still being used by this thread.
	 */
	fastrpc_channel_update_invoke_cnt(cctx, true);
	spin_unlock_irqrestore(&cctx->lock, flags);

	switch (cmd) {
	case FASTRPC_IOCTL_INVOKE:
		trace_fastrpc_msg("invoke: begin");
		err = fastrpc_invoke(fl, argp);
		trace_fastrpc_msg("invoke: end");
		break;
	case FASTRPC_IOCTL_MULTIMODE_INVOKE:
		err = fastrpc_multimode_invoke(fl, argp);
		break;
	case FASTRPC_IOCTL_INIT_ATTACH:
		err = fastrpc_init_attach(fl, ROOT_PD);
		fastrpc_send_cpuinfo_to_dsp(fl);
		process_init = 1;
		break;
	case FASTRPC_IOCTL_INIT_ATTACH_SNS:
		err = fastrpc_init_attach(fl, SENSORS_STATICPD);
		process_init = 1;
		break;
	case FASTRPC_IOCTL_INIT_CREATE_STATIC:
		err = fastrpc_init_create_static_process(fl, argp);
		process_init = 1;
		break;
	case FASTRPC_IOCTL_INIT_CREATE:
		err = fastrpc_init_create_process(fl, argp);
		process_init = 1;
		break;
	case FASTRPC_IOCTL_ALLOC_DMA_BUFF:
		err = fastrpc_dmabuf_alloc(fl, argp);
		break;
	case FASTRPC_IOCTL_MMAP:
		mutex_lock(&fl->remote_map_mutex);
		err = fastrpc_req_mmap(fl, argp);
		mutex_unlock(&fl->remote_map_mutex);
		break;
	case FASTRPC_IOCTL_MUNMAP:
		mutex_lock(&fl->remote_map_mutex);
		err = fastrpc_req_munmap(fl, argp);
		mutex_unlock(&fl->remote_map_mutex);
		break;
	case FASTRPC_IOCTL_MEM_MAP:
		err = fastrpc_req_mem_map(fl, argp);
		break;
	case FASTRPC_IOCTL_MEM_UNMAP:
		err = fastrpc_req_mem_unmap(fl, argp);
		break;
	case FASTRPC_IOCTL_GET_DSP_INFO:
		err = fastrpc_get_dsp_info(fl, argp);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if (process_init && !err) {
		err = fastrpc_device_create(fl);
		if (err)
			atomic_set(&fl->state, DEFAULT_PROC_STATE);
		else
			atomic_set(&fl->state, DSP_CREATE_COMPLETE);
	}

	spin_lock_irqsave(&cctx->lock, flags);
	fastrpc_channel_update_invoke_cnt(cctx, false);
	spin_unlock_irqrestore(&cctx->lock, flags);
	fastrpc_channel_ctx_put(fl->cctx);
	return err;
}

int fastrpc_init_privileged_gids(struct device *dev, char *prop_name,
						struct gid_list *gidlist)
{
	int err = 0;
	u32 len = 0, i;
	u32 *gids = NULL;

	if (!of_find_property(dev->of_node, prop_name, &len))
		return 0;
	if (len == 0)
		return 0;

	len /= sizeof(u32);
	gids = kcalloc(len, sizeof(u32), GFP_KERNEL);
	if (!gids)
		return -ENOMEM;

	for (i = 0; i < len; i++) {
		err = of_property_read_u32_index(dev->of_node, prop_name,
								i, &gids[i]);
		if (err) {
			dev_err(dev, "%s: failed to read GID %u\n",
					__func__, i);
			goto read_error;
		}
		dev_info(dev, "adsprpc: %s: privileged GID: %u\n", __func__, gids[i]);
	}
	sort(gids, len, sizeof(*gids), uint_cmp_func, NULL);
	gidlist->gids = gids;
	gidlist->gidcount = len;

	return 0;
read_error:
	kfree(gids);
	return err;
}

union fastrpc_dev_param {
	struct fastrpc_dev_map_dma *map;
	struct fastrpc_dev_unmap_dma *unmap;
	struct fastrpc_dev_get_hlos_pid *hpid;
};
   /*
	* fastrpc_dev_map_dma() - Function to map buffers mapped on DSP.
	* @arg1: client instance of fastrpc_device struct
	* @arg2: invoke param
	*
	* fastrpc_dev_map_dma is used to map buffers mapped on DSP
	*
	*
	* Return: 0 on success.
	*
	*/
long fastrpc_dev_map_dma(struct fastrpc_device *dev,
			unsigned long invoke_param)
{
	int err = 0;
	bool is_cnt_updated = false;
	union fastrpc_dev_param p;
	struct fastrpc_user *fl = NULL;
	struct fastrpc_map *map = NULL;
	uintptr_t raddr = 0;
	unsigned long irq_flags = 0;
	struct fastrpc_channel_ctx * cctx = NULL;
	spinlock_t *glock = &g_frpc.glock;

	p.map = (struct fastrpc_dev_map_dma *)invoke_param;

	spin_lock_irqsave(glock, irq_flags);
	if (!dev || dev->dev_close) {
		err = -ESRCH;
		pr_err("%s : bad dev or device is already closed", __func__);
		spin_unlock_irqrestore(glock, irq_flags);
		return err;
	}

	fl = dev->fl;
	if (!fl) {
		err = -EBADF;
		pr_err("%s : bad fl", __func__);
		spin_unlock_irqrestore(glock, irq_flags);
		return err;
	}
	cctx = fl->cctx;
	fastrpc_channel_ctx_get(cctx);
	fl->is_dma_invoke_pend = true;
	spin_unlock_irqrestore(glock, irq_flags);

	spin_lock_irqsave(&cctx->lock, irq_flags);
	if (atomic_read(&cctx->teardown)) {
		spin_unlock_irqrestore(&cctx->lock, irq_flags);
		err = -EPIPE;
		goto error;
	} else {
		/*
		 * Update invoke count to block SSR handling thread
		 * from cleaning up the channel resources, while it
		 * is stillbeing used by this thread.
		 */
		fastrpc_channel_update_invoke_cnt(cctx, true);
		is_cnt_updated = true;
	}
	spin_unlock_irqrestore(&cctx->lock, irq_flags);

	/* Map DMA buffer on SMMU device*/
	mutex_lock(&fl->remote_map_mutex);
	mutex_lock(&fl->map_mutex);
	err = fastrpc_map_create(fl, -1, 0, p.map->buf,
				p.map->size, p.map->attrs,
				ADSP_MMAP_DMA_BUFFER, &map, true);
	mutex_unlock(&fl->map_mutex);
	if (err)
		goto error;
	/*
	 * Update the map state to in progress only if there is no ongoing or
	 * completed DSP mapping.
	 */
	if (atomic_cmpxchg(&map->state, FD_MAP_DEFAULT, FD_DSP_MAP_IN_PROGRESS)
		!= FD_MAP_DEFAULT) {
		err = -EALREADY;
		goto error;
	}
	/* Map DMA buffer on DSP*/

	err = fastrpc_mem_map_to_dsp(fl, -1, 0, map->flags, 0, map->phys, map->size, &raddr);
	if (err) {
		pr_err("%s : failed to map buffer on DSP ", __func__);
		/* Revert the map state to map default */
		atomic_set(&map->state, FD_MAP_DEFAULT);
		goto error;
	}
	map->raddr = raddr;
	p.map->v_dsp_addr = raddr;
	/* Set the map state to complete on successful mapping */
	atomic_set(&map->state, FD_DSP_MAP_COMPLETE);
error:
	if (err && map) {
		mutex_lock(&fl->map_mutex);
		fastrpc_map_put(map);
		mutex_unlock(&fl->map_mutex);
	}

	spin_lock_irqsave(&cctx->lock, irq_flags);
	if (fl) {
		if (atomic_read(&fl->state) >= DSP_EXIT_START && fl->is_dma_invoke_pend) {
			/*
			 * If process exit has already started and is waiting for this invoke
			 * to complete, then unblock it.
			 */
			complete(&fl->dma_invoke);
		}
		fl->is_dma_invoke_pend = false;
	}
	mutex_unlock(&fl->remote_map_mutex);
	if (is_cnt_updated)
		fastrpc_channel_update_invoke_cnt(cctx, false);
	spin_unlock_irqrestore(&cctx->lock, irq_flags);
	fastrpc_channel_ctx_put(cctx);
	return err;
}
   /*
	* fastrpc_dev_unmap_dma() - Function to unmap buffers mapped on DSP.
	* @arg1: client instance of fastrpc_device struct
	* @arg2: invoke param
	*
	* fastrpc_dev_unmap_dma is used to unmap buffers mapped on DSP
	*
	*
	* Return: 0 on success.
	*
	*/
long fastrpc_dev_unmap_dma(struct fastrpc_device *dev,
			unsigned long invoke_param)
{
	int err = 0;
	bool is_cnt_updated = false;
	union fastrpc_dev_param p;
	struct fastrpc_user *fl = NULL;
	struct fastrpc_map *map = NULL;
	unsigned long irq_flags = 0;
	struct fastrpc_channel_ctx * cctx = NULL;
	spinlock_t *glock = &g_frpc.glock;

	p.unmap = (struct fastrpc_dev_unmap_dma *)invoke_param;

	spin_lock_irqsave(glock, irq_flags);
	if (!dev || dev->dev_close) {
		pr_err("%s : bad dev or device is already closed", __func__);
		err = -ESRCH;
		spin_unlock_irqrestore(glock, irq_flags);
		return err;
	}
	fl = dev->fl;
	if (!fl) {
		err = -EBADF;
		pr_err("%s : bad fl ", __func__);
		spin_unlock_irqrestore(glock, irq_flags);
		return err;
	}
	cctx = fl->cctx;
	fastrpc_channel_ctx_get(cctx);
	fl->is_dma_invoke_pend = true;
	spin_unlock_irqrestore(glock, irq_flags);

	spin_lock_irqsave(&cctx->lock, irq_flags);
	if (atomic_read(&cctx->teardown)) {
		spin_unlock_irqrestore(&cctx->lock, irq_flags);
		err = -EPIPE;
		goto error;
	} else {
		/*
		 * Update invoke count to block SSR handling thread
		 * from cleaning up the channel resources, while it
		 * is stillbeing used by this thread.
		 */
		fastrpc_channel_update_invoke_cnt(cctx, true);
		is_cnt_updated = true;
	}
	spin_unlock_irqrestore(&cctx->lock, irq_flags);

	mutex_lock(&fl->remote_map_mutex);
	mutex_lock(&fl->map_mutex);
	err = fastrpc_map_lookup(fl, -1, 0, 0, p.unmap->buf,
				ADSP_MMAP_DMA_BUFFER, &map, false);
	 /*
	  * Check if DSP mapping is complete, then move the state to
	  * unmap in progress only if there is no other ongoing unmap.
	  */
	if (!err && atomic_cmpxchg(&map->state, FD_DSP_MAP_COMPLETE,
		FD_DSP_UNMAP_IN_PROGRESS) != FD_DSP_MAP_COMPLETE)
		err = -EALREADY;
	mutex_unlock(&fl->map_mutex);
	if (err)
		goto error;
	/* Un-map DMA buffer on DSP*/
	err = fastrpc_req_munmap_dsp(fl, map->raddr, map->size);
	if (err) {
		pr_err("Unmap on DSP failed for buf phy:0x%llx, raddr:0x%llx, size:0x%llx\n",
			map->phys, map->raddr, map->size);
		/* Revert the map state to map complete */
		atomic_set(&map->state, FD_DSP_MAP_COMPLETE);
		goto error;
	}
	/* Set the map state to default on successful unmapping */
	atomic_set(&map->state, FD_MAP_DEFAULT);
	mutex_lock(&fl->map_mutex);
	fastrpc_map_put(map);
	mutex_unlock(&fl->map_mutex);

error:
	spin_lock_irqsave(&cctx->lock, irq_flags);
	if (fl) {
		if (atomic_read(&fl->state) >= DSP_EXIT_START && fl->is_dma_invoke_pend) {
			/*
			 * If process exit has already started and is waiting for this invoke
			 * to complete, then unblock it.
			 */
			complete(&fl->dma_invoke);
		}
		fl->is_dma_invoke_pend = false;
	}
	mutex_unlock(&fl->remote_map_mutex);
	if (is_cnt_updated)
		fastrpc_channel_update_invoke_cnt(cctx, false);
	spin_unlock_irqrestore(&cctx->lock, irq_flags);
	fastrpc_channel_ctx_put(cctx);
	return err;
}
   /*
	* fastrpc_dev_get_hlos_pid() - Function to get hlos pid.
	* @arg1: client instance of fastrpc_device struct.
	* @arg2: invoke param.
	*
	* fastrpc_dev_get_hlos_pid is used to get hlos id
	*
	* Return: void.
	*
	*/
long fastrpc_dev_get_hlos_pid(struct fastrpc_device *dev,
			unsigned long invoke_param)
{
	int err = 0;
	union fastrpc_dev_param p;
	struct fastrpc_user *fl = NULL;
	unsigned long irq_flags = 0;
	struct fastrpc_channel_ctx * cctx = NULL;
	spinlock_t *glock = &g_frpc.glock;

	spin_lock_irqsave(glock, irq_flags);
	if (!dev  || dev->dev_close) {
		pr_err("%s : bad dev or device is already closed", __func__);
		err = -ESRCH;
		spin_unlock_irqrestore(glock, irq_flags);
		return err;
	}

	fl = dev->fl;
	if (!fl) {
		err = -EBADF;
		pr_err("%s : bad fl ", __func__);
		spin_unlock_irqrestore(glock, irq_flags);
		return err;
	}
	cctx = fl->cctx;
	fastrpc_channel_ctx_get(cctx);

	p.hpid = (struct fastrpc_dev_get_hlos_pid *)invoke_param;
	p.hpid->hlos_pid = fl->tgid;
	spin_unlock_irqrestore(glock, irq_flags);
	fastrpc_channel_ctx_put(cctx);

	return err;
}
   /*
	* fastrpc_driver_invoke() - Invocation function for client drivers.
	* @arg1: client instance of fastrpc_device struct
	* @arg2: invoke number
	* @arg3: invoke param
	*
	* fastrpc_driver_invoke is exposed to the client drivers to make invoke
	* calls. Clients can map and unmap buffers on dsp using invoke calls.
	* function can be called with an instance of the fastrpc_device instance,
	* invocation number and corresponding invoke params.
	*
	*
	* Return: 0 on success.
	*
	*/
long fastrpc_driver_invoke(struct fastrpc_device *dev, unsigned int invoke_num,
			unsigned long invoke_param)
{
	int err = 0;

	switch (invoke_num) {
	case FASTRPC_DEV_MAP_DMA:
		err = fastrpc_dev_map_dma(dev, invoke_param);
		break;
	case FASTRPC_DEV_UNMAP_DMA:
		err = fastrpc_dev_unmap_dma(dev, invoke_param);
		break;
	case FASTRPC_DEV_GET_HLOS_PID:
		err = fastrpc_dev_get_hlos_pid(dev, invoke_param);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	return err;
}
EXPORT_SYMBOL_GPL(fastrpc_driver_invoke);

   /*
	* fastrpc_device_create() - Create an instance of fastrpc_device.
	* @arg1: fastrpc_user instance corresponding to the process.
	*
	* fastrpc_device_create will create an instance of struct fastrpc_device
	* for each process
	*
	*
	* Return: 0 on success, error code on failure.
	*
	*/
static int fastrpc_device_create(struct fastrpc_user *fl)
{
	int err = 0;
	struct fastrpc_device *frpc_dev = NULL;

	frpc_dev = kzalloc(sizeof(*frpc_dev), GFP_KERNEL);
	if (!frpc_dev) {
		err = -ENOMEM;
		return err;
	}

	frpc_dev->fl = fl;
	frpc_dev->handle = fl->tgid_frpc;
	fl->device = frpc_dev;
	return err;
}

   /*
	* fastrpc_driver_unregister() - Function to unregister client drivers.
	* @arg1: client instance of fastrpc_driver struct
	*
	* fastrpc_driver_unregister is used to unregister the client drivers
	* from fastrpc driver.
	*
	* Context: Acquires channel context spin-lock and glock
	*
	* Return: void.
	*
	*/
void fastrpc_driver_unregister(struct fastrpc_driver *frpc_driver){

	struct fastrpc_device *frpc_dev = NULL;
	unsigned long irq_flags = 0, flags = 0;
	struct fastrpc_channel_ctx * cctx = NULL;
	struct fastrpc_user *fl = NULL;
	spinlock_t *glock = &g_frpc.glock;

	spin_lock_irqsave(glock, irq_flags);
	frpc_dev = (struct fastrpc_device *)frpc_driver->device;
	if (!frpc_dev) {
		spin_unlock_irqrestore(glock, irq_flags);
		pr_err("passed invalid driver, fastrpc device not present");
		return;
	}

	// If device is already closed, free the device
	if (frpc_dev->dev_close) {
		spin_unlock_irqrestore(glock, irq_flags);
		kfree(frpc_dev);
		pr_info("Un-registering fastrpc driver with handle 0x%x\n",
			frpc_driver->handle);
		return;
	}

	fl = frpc_dev->fl;
	if (!fl) {
		spin_unlock_irqrestore(glock, irq_flags);
		pr_err("passed invalid driver, invalid process");
		return;
	}
	cctx = frpc_dev->fl->cctx;
	fastrpc_channel_ctx_get(cctx);

	spin_lock_irqsave(&cctx->lock, flags);
	list_del_init(&frpc_driver->hn);
	spin_unlock_irqrestore(&cctx->lock, flags);
	spin_unlock_irqrestore(glock, irq_flags);

	fastrpc_channel_ctx_put(cctx);

	pr_info("Un-registering fastrpc driver with handle 0x%x\n",
			frpc_driver->handle);
}
EXPORT_SYMBOL_GPL(fastrpc_driver_unregister);

   /*
	* fastrpc_driver_register() - Function to register client drivers.
	* @arg1: client instance of fastrpc_driver struct.
	*
	* fastrpc_driver_register is used to register client drivers with
	* fastrpc driver. Clients will pass instance of fastrpc_driver struct.
	* The instance will contain unique id corresponding to a process. Function
	* will iterate through channel context to find a match. If match is found,
	* probe function provided in the input struct will be called. During probe
	* we will share fastrpc_device instance as a handle which can be used by the
	* client driver while making invoke calls.
	*
	* Context: Acquires channel context spin-lock to iterate through
	*          contexts.
	* Return: 0 on success. Corresponding error value on failure.
	*
	*/

int fastrpc_driver_register(struct fastrpc_driver *frpc_driver)
{
	int err = 0, i = 0;
	unsigned long irq_flags = 0;
	struct fastrpc_user *user = NULL;
	struct fastrpc_channel_ctx *cctx = NULL;

	if(frpc_driver == NULL) {
		pr_err("%s : invalid registraion request", __func__);
		return -EINVAL;
	}

	/* Set to NULL to avoid stale values */
	frpc_driver->device = NULL;

	/*
	 * Iterate through all channel contexts to find the process
	 * requested by the client driver.
	 */
	for (i = 0; i < FASTRPC_DEV_MAX; i++) {
		cctx = g_frpc.gctx[i];
		if (!cctx)
			continue;

		spin_lock_irqsave(&cctx->lock, irq_flags);
		list_for_each_entry(user, &cctx->users, user) {
			if (user->tgid_frpc == frpc_driver->handle) {
				goto process_found;
			}
		}
		spin_unlock_irqrestore(&cctx->lock, irq_flags);
	}
	pr_err("%s: no client found for handle 0x%x",
		__func__, frpc_driver->handle);
	return -ESRCH;

process_found:
	if(atomic_read(&user->state) >= DSP_EXIT_START) {
		spin_unlock_irqrestore(&cctx->lock, irq_flags);
		pr_err("%s : process already exited", __func__);
		return -ESRCH;
	}

	frpc_driver->device = (struct device *)user->device;
	list_add_tail(&frpc_driver->hn, &user->fastrpc_drivers);
	spin_unlock_irqrestore(&cctx->lock, irq_flags);
	/* Execute the probe fn. of the client driver if matching process found */
	frpc_driver->probe(user->device);
	pr_info("fastrpc driver registered with handle 0x%x\n", frpc_driver->handle);

	return err;
}
EXPORT_SYMBOL_GPL(fastrpc_driver_register);
void fastrpc_notify_users(struct fastrpc_user *user)
{
	struct fastrpc_invoke_ctx *ctx;
	struct fastrpc_user *fl;

	spin_lock(&user->lock);
	list_for_each_entry(ctx, &user->pending, node) {
		fl = ctx->fl;
		/*
		 * After audio or ois PDR, skip notifying the pending kill call,
		 * as the DSP guestOS may still be processing and might result
		 * improper access issues. But in case of SSR cleanup pending
                 * kill calls as well.
		 */
		if (atomic_read(&fl->state) >= DSP_EXIT_START &&
                        !IS_SSR(fl) && IS_PDR(fl) &&
			fl->pd_type != SENSORS_STATICPD &&
			ctx->msg.handle == FASTRPC_INIT_HANDLE)
			continue;
		ctx->retval = -EPIPE;
		ctx->is_work_done = true;
		trace_fastrpc_context_complete(ctx->fl->cctx->domain_id, (uint64_t)ctx,
			ctx->retval, ctx->pid, ctx->pid, ctx->sc);
		complete(&ctx->work);
	}
	list_for_each_entry(ctx, &user->interrupted, node) {
		ctx->retval = -EPIPE;
		ctx->is_work_done = true;
		trace_fastrpc_context_complete(ctx->fl->cctx->domain_id, (uint64_t)ctx,
			ctx->retval, ctx->pid, ctx->pid, ctx->sc);
		complete(&ctx->work);
	}
	spin_unlock(&user->lock);
}

static void fastrpc_notify_pdr_drivers(struct fastrpc_channel_ctx *cctx,
		char *servloc_name)
{
	struct fastrpc_user *fl;
	unsigned long flags;

	spin_lock_irqsave(&cctx->lock, flags);
	list_for_each_entry(fl, &cctx->users, user) {
		if (fl->servloc_name && !strcmp(servloc_name, fl->servloc_name))
			fastrpc_notify_users(fl);
	}
	spin_unlock_irqrestore(&cctx->lock, flags);
}

static void fastrpc_pdr_cb(int state, char *service_path, void *priv)
{
	struct fastrpc_static_pd *spd = (struct fastrpc_static_pd *)priv;
	struct fastrpc_channel_ctx *cctx;
	unsigned long flags;

	if (!spd)
		return;

	cctx = spd->cctx;
	switch (state) {
	case SERVREG_SERVICE_STATE_DOWN:
		pr_info("fastrpc: %s: %s (%s) is down for PDR on %s\n",
			__func__, spd->spdname,
			spd->servloc_name,
			domains[cctx->domain_id]);
		spin_lock_irqsave(&cctx->lock, flags);
		spd->pdrcount++;
		atomic_set(&spd->ispdup, 0);
		atomic_set(&spd->is_attached, 0);
		spin_unlock_irqrestore(&cctx->lock, flags);
		if (!strcmp(spd->servloc_name,
				AUDIO_PDR_SERVICE_LOCATION_CLIENT_NAME))
			cctx->staticpd_status = false;

		fastrpc_notify_pdr_drivers(cctx, spd->servloc_name);
		break;
	case SERVREG_SERVICE_STATE_UP:
		pr_info("fastrpc: %s: %s (%s) is up for PDR on %s\n",
			__func__, spd->spdname,
			spd->servloc_name,
			domains[cctx->domain_id]);
		atomic_set(&spd->ispdup, 1);
		break;
	default:
		break;
	}
	return;
}

static const struct file_operations fastrpc_fops = {
	.open = fastrpc_device_open,
	.release = fastrpc_device_release,
	.unlocked_ioctl = fastrpc_device_ioctl,
	.compat_ioctl = fastrpc_device_ioctl,
};

static int fastrpc_cb_probe(struct platform_device *pdev)
{
	struct fastrpc_channel_ctx *cctx;
	struct fastrpc_pool_ctx *sess = NULL;
	struct device *dev = &pdev->dev;
	int i, sessions = 0, usershare = 0;
	unsigned long flags;
	u32 pd_type = DEFAULT_UNUSED, smmuidx = DEFAULT_SMMU_IDX;
	int rc, err = 0;
	struct fastrpc_buf *buf = NULL;
	struct iommu_domain *domain = NULL;
	struct gen_pool *gen_pool = NULL;
	int frpc_gen_addr_pool[2] = {0, 0};
	u32 smmu_alloc_range[2] = {0, 0};
	struct sg_table sgt;
	struct fastrpc_smmu *smmucb = NULL;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root = g_frpc.debugfs_root;
	struct dentry *debugfs_global_file = NULL;
#endif

	cctx = get_current_channel_ctx(dev);

	if (IS_ERR_OR_NULL(cctx))
		return -EINVAL;

	of_property_read_u32(dev->of_node, "qcom,nsessions", &sessions);
	of_property_read_u32(dev->of_node, "qcom,usershare", &usershare);
        cctx->usershare = usershare;

	if (of_get_property(dev->of_node, "pd-type", NULL) != NULL) {
		err = of_property_read_u32(dev->of_node, "pd-type",
				&pd_type);
		if (err)
			goto bail;
		// Set pd_type, if the process type is configured for context banks
		cctx->pd_type = true;
	}

	spin_lock_irqsave(&cctx->lock, flags);
	if (cctx->sesscount >= FASTRPC_MAX_SESSIONS) {
		dev_err(&pdev->dev, "too many sessions\n");
		spin_unlock_irqrestore(&cctx->lock, flags);
		return -ENOSPC;
	}

	/* Find any existing session for pooling CBs with same PD type */
	for (i = 0; i < cctx->sesscount; i++) {
		/* Only USER_UNSIGNEDPD_POOL type is pooled */
		if (pd_type != USER_UNSIGNEDPD_POOL)
			break;

		if (cctx->session[i].pd_type == pd_type) {
			sess = &cctx->session[i];
			/* Set smmucb_pool to true, if SMMU CB pooling is enabled */
			cctx->smmucb_pool = true;
			break;
		}
	}

	/* If no existing session was found, prepare new session */
	if (!sess)
		sess = &cctx->session[cctx->sesscount++];

	/* Update session info during probe of first CB only */
	if (sess->smmucount == 0) {
		sess->usecount = 0;
		sess->pd_type = pd_type;
	}
        sess->usershare = usershare;
	/* Read secure flag for each context bank, even if part of CB pool */
	sess->secure = of_property_read_bool(dev->of_node,
						"qcom,secure-context-bank");

	/* Populate SMMU CB info at next available free SMMU index */
	smmuidx = sess->smmucount++;
	smmucb = &sess->smmucb[smmuidx];
	smmucb->valid = true;
	smmucb->dev = dev;
	smmucb->sess = sess;
	mutex_init(&smmucb->map_mutex);

	if (of_property_read_u32(dev->of_node, "reg", &smmucb->sid))
		dev_info(dev, "FastRPC Session ID not specified in DT\n");

	/* Set SMMU context bank, min and max allocation range */
	if (!of_property_read_u32_array(dev->of_node, "alloc-size-range",
							smmu_alloc_range, sizeof(smmu_alloc_range))) {
		smmucb->minallocsize = smmu_alloc_range[0];
		smmucb->maxallocsize = smmu_alloc_range[1];
	}
	smmucb->totalbytes = SMMU_4GB_ADDRESS_SPACE;

	/* Set SMMU device private data with fastrpc SMMU CB pointer */
	dev_set_drvdata(dev, smmucb);

	/* Context bank can be shared by multiple apps. Create duplicate sessions */
	if (sessions > 0) {
		struct fastrpc_pool_ctx *dup_sess = NULL;

		sess->sharedcb = true;
		for (i = 1; i < sessions; i++) {
			if (cctx->sesscount >= FASTRPC_MAX_SESSIONS)
				break;
			dup_sess = &cctx->session[cctx->sesscount++];
			memcpy(dup_sess, sess, sizeof(*dup_sess));
			mutex_init(&dup_sess->smmucb[DEFAULT_SMMU_IDX].map_mutex);
		}
	}
	spin_unlock_irqrestore(&cctx->lock, flags);
	if (of_get_property(dev->of_node, "qrtr-gen-pool", NULL) != NULL) {

		err = of_property_read_u32_array(dev->of_node, "frpc-gen-addr-pool",
							frpc_gen_addr_pool, 2);
		if (err) {
			dev_err(&pdev->dev, "Error: parsing frpc-gen-addr-pool arguments failed for %s with err %d\n",
					dev_name(dev), err);
			goto bail;
		}
		smmucb->genpool_iova = frpc_gen_addr_pool[0];
		smmucb->genpool_size = frpc_gen_addr_pool[1];

		buf = kzalloc(sizeof(*buf), GFP_KERNEL);
		if (IS_ERR_OR_NULL(buf)) {
			err = -ENOMEM;
			dev_err(&pdev->dev, "allocation failed for size 0x%zx\n", sizeof(*buf));
			goto bail;
		}
		INIT_LIST_HEAD(&buf->attachments);
		INIT_LIST_HEAD(&buf->node);
		mutex_init(&buf->lock);
		buf->virt = NULL;
		buf->phys = 0;
		buf->size = frpc_gen_addr_pool[1];
		buf->dev = smmucb->dev;
		buf->raddr = 0;


		/* Allocate memory for adding to genpool */
		buf->virt = dma_alloc_coherent(buf->dev, buf->size,
					(dma_addr_t *)&buf->phys, GFP_KERNEL);

		if (IS_ERR_OR_NULL(buf->virt)) {
			dev_err(&pdev->dev, "dma_alloc failed for size 0x%llx, returned %pK\n",
				buf->size, buf->virt);
			err = -ENOBUFS;
			goto dma_alloc_bail;
		}

		err = dma_get_sgtable(smmucb->dev, &sgt, buf->virt,
				buf->phys, buf->size);
		if (err) {
			dev_err(&pdev->dev, "dma_get_sgtable_attrs failed with err %d", err);
				goto iommu_map_bail;
		}
		domain = iommu_get_domain_for_dev(smmucb->dev);
		if (!domain) {
			dev_err(&pdev->dev, "iommu_get_domain_for_dev failed ");
			goto iommu_map_bail;
		}

		/* Map the allocated memory with fixed IOVA and is shared to remote subsystem */
#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
		err = iommu_map_sg(domain, frpc_gen_addr_pool[0], sgt.sgl,
				sgt.nents, IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE, GFP_KERNEL);
#else
		err = iommu_map_sg(domain, frpc_gen_addr_pool[0], sgt.sgl,
				sgt.nents, IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE);

#endif
		if (err < 0) {
			dev_err(&pdev->dev, "iommu_map_sg failed with err %d", err);
			goto iommu_map_bail;
		}

		/* Create genpool using SMMU device */
		gen_pool = devm_gen_pool_create(smmucb->dev, 0, NUMA_NO_NODE, NULL);
		if (IS_ERR(gen_pool)) {
			err = PTR_ERR(gen_pool);
			dev_err(&pdev->dev, "devm_gen_pool_create failed with err %d", err);
			goto genpool_create_bail;
		}
		/* Add allocated memory to genpool */
		err = gen_pool_add_virt(gen_pool, (unsigned long)buf->virt,
				buf->phys, buf->size, NUMA_NO_NODE);
		if (err) {
				dev_err(&pdev->dev, "gen_pool_add_virt failed with err %d", err);
			goto genpool_add_bail;
		}
		smmucb->frpc_genpool = gen_pool;
		smmucb->frpc_genpool_buf = buf;
		dev_err(&pdev->dev, "fastrpc_cb_probe qrtr-gen-pool end\n");
	}
	rc = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(dev, "32-bit DMA enable failed\n");
		return rc;
	}
#ifdef CONFIG_DEBUG_FS
	if (debugfs_root && !g_frpc.debugfs_global_file) {
		debugfs_global_file = debugfs_create_file("global", 0644,
			debugfs_root, NULL, &fastrpc_debugfs_fops);
		if (IS_ERR_OR_NULL(debugfs_global_file)) {
			pr_warn("Error: %s: %s: failed to create debugfs global file\n",
				current->comm, __func__);
			debugfs_global_file = NULL;
		}
		g_frpc.debugfs_global_file = debugfs_global_file;
	}
#endif

bail:
	if (!err)
		dev_info(dev, "Successfully added %s", dev->kobj.name);
	return err;
genpool_add_bail:
	gen_pool_destroy(gen_pool);
genpool_create_bail:
	iommu_unmap(domain, smmucb->genpool_iova, smmucb->genpool_size);
iommu_map_bail:
	dma_free_coherent(smmucb->dev, buf->size, buf->virt, FASTRPC_PHYS(buf->phys));
dma_alloc_bail:
	kfree(buf);
	return err;
}

/* Function to free fastrpc genpool buffer */
static void fastrpc_genpool_free(struct fastrpc_smmu *smmucb)
{
	struct fastrpc_buf *buf = NULL;
	struct iommu_domain *domain = NULL;

	if (!smmucb)
		return;
	buf = smmucb->frpc_genpool_buf;
	if (smmucb->frpc_genpool) {
		gen_pool_destroy(smmucb->frpc_genpool);
		smmucb->frpc_genpool = NULL;
	}
	if (buf && smmucb->dev) {
		domain = iommu_get_domain_for_dev(smmucb->dev);
		iommu_unmap(domain, smmucb->genpool_iova,
					smmucb->genpool_size);
		if (buf->phys)
			dma_free_coherent(buf->dev, buf->size, buf->virt,
									FASTRPC_PHYS(buf->phys));
		kfree(buf);
		smmucb->frpc_genpool_buf = NULL;
	}
}

static int fastrpc_cb_remove(struct platform_device *pdev)
{
	struct fastrpc_channel_ctx *cctx = dev_get_drvdata(pdev->dev.parent);
	struct fastrpc_smmu *smmucb = dev_get_drvdata(&pdev->dev),
							*ismmucb = NULL;
	struct fastrpc_pool_ctx *sess = smmucb->sess;
	unsigned long flags;
	int i = 0, j = 0;

	if (sess->pd_type == ROOT_PD)
		fastrpc_rootheap_buf_list_free(cctx);

	spin_lock_irqsave(&cctx->lock, flags);
	for (i = 0; i < FASTRPC_MAX_SESSIONS; i++) {
		for (j = 0; j < cctx->session[i].smmucount; j++) {
			ismmucb = &cctx->session[i].smmucb[j];
			if (ismmucb->sid != smmucb->sid)
				continue;
			spin_unlock_irqrestore(&cctx->lock, flags);
			mutex_lock(&ismmucb->map_mutex);
			if (ismmucb->frpc_genpool)
				fastrpc_genpool_free(ismmucb);
			ismmucb->dev = NULL;
			mutex_unlock(&ismmucb->map_mutex);
			spin_lock_irqsave(&cctx->lock, flags);
			ismmucb->valid = false;
			cctx->sesscount--;
		}
	}
	spin_unlock_irqrestore(&cctx->lock, flags);
	dev_info(&pdev->dev, "Successfully removed %s", pdev->dev.kobj.name);
	return 0;
}

static const struct of_device_id fastrpc_match_table[] = {
	{ .compatible = "qcom,fastrpc-compute-cb", },
	{}
};

static struct platform_driver fastrpc_cb_driver = {
	.probe = fastrpc_cb_probe,
	.remove = fastrpc_cb_remove,
	.driver = {
		.name = "qcom,fastrpc-cb",
		.of_match_table = fastrpc_match_table,
		.suppress_bind_attrs = true,
	},
};

int fastrpc_device_register(struct device *dev, struct fastrpc_channel_ctx *cctx,
				   bool is_secured, const char *domain)
{
	struct fastrpc_device_node *fdev;
	int err;

	fdev = devm_kzalloc(dev, sizeof(*fdev), GFP_KERNEL);
	if (!fdev)
		return -ENOMEM;

	fdev->secure = is_secured;
	fdev->cctx = cctx;
	cctx->dev = dev;
	fdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	fdev->miscdev.fops = &fastrpc_fops;
	fdev->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "fastrpc-%s%s",
					    domain, is_secured ? "-secure" : "");
	if (!fdev->miscdev.name)
		return -ENOMEM;

	err = misc_register(&fdev->miscdev);
	if (!err) {
		if (is_secured)
			cctx->secure_fdevice = fdev;
		else
			cctx->fdevice = fdev;
	}

	return err;
}

void fastrpc_lowest_capacity_corecount(struct device *dev, struct fastrpc_channel_ctx *cctx)
{
	u32 cpu = 0;

	cpu =  cpumask_first(cpu_possible_mask);
	for_each_cpu(cpu, cpu_possible_mask) {
		if (topology_cluster_id(cpu) == 0)
			cctx->lowest_capacity_core_count++;
	}
	dev_info(dev, "Lowest capacity core count: %u\n",
					cctx->lowest_capacity_core_count);
}

int fastrpc_setup_service_locator(struct fastrpc_channel_ctx *cctx, char *client_name,
					char *service_name, char *service_path, int spd_session)
{
	int err = 0;
	struct pdr_handle *handle = NULL;
	struct pdr_service *service = NULL;

	/* Register the service locator's callback function */
	handle = pdr_handle_alloc(fastrpc_pdr_cb, &cctx->spd[spd_session]);
	if (IS_ERR(handle)) {
		err = PTR_ERR(handle);
		goto bail;
	}
	cctx->spd[spd_session].pdrhandle = handle;
	cctx->spd[spd_session].servloc_name = client_name;
	cctx->spd[spd_session].spdname = service_path;
	cctx->spd[spd_session].cctx = cctx;
	service = pdr_add_lookup(handle, service_name, service_path);
	if (IS_ERR(service)) {
		err = PTR_ERR(service);
		goto bail;
	}
	dev_info(cctx->dev, "%s: pdr_add_lookup enabled for %s (%s, %s)\n",
		__func__, service_name, client_name, service_path);

bail:
	if (err)
		dev_err(cctx->dev, "%s: failed for %s (%s, %s)with err %d\n",
				__func__, service_name, client_name, service_path, err);
	return err;
}

void fastrpc_register_wakeup_source(struct device *dev,
	const char *client_name, struct wakeup_source **device_wake_source)
{
	struct wakeup_source *wake_source = NULL;

	wake_source = wakeup_source_register(dev, client_name);
	if (IS_ERR_OR_NULL(wake_source)) {
		dev_err(dev, "wakeup_source_register failed for dev %s, client %s with err %ld\n",
		dev_name(dev), client_name, PTR_ERR(wake_source));
		return;
	}

	*device_wake_source = wake_source;
}

static void fastrpc_notify_user_ctx(struct fastrpc_invoke_ctx *ctx, int retval,
		u32 rsp_flags, u32 early_wake_time)
{
        if(!ctx->cctx)
           return;
	if (!atomic_read(&ctx->cctx->teardown))
		fastrpc_pm_awake(ctx->fl, ctx->cctx->secure);
	ctx->retval = retval;
	ctx->rsp_flags = (enum fastrpc_response_flags)rsp_flags;
	trace_fastrpc_context_complete(ctx->cctx->domain_id, (uint64_t)ctx,
			retval, ctx->ctxid, ctx->pid, ctx->sc);
	switch (rsp_flags) {
	case NORMAL_RESPONSE:
	case COMPLETE_SIGNAL:
		/* normal and complete response with return value */
		ctx->is_work_done = true;
		trace_fastrpc_msg("wakeup_task: begin");
		complete(&ctx->work);
		trace_fastrpc_msg("wakeup_task: end");
		break;
	case USER_EARLY_SIGNAL:
		/* user hint of approximate time of completion */
		ctx->early_wake_time = early_wake_time;
		break;
	case EARLY_RESPONSE:
		/* rpc framework early response with return value */
		trace_fastrpc_msg("wakeup_task: begin");
		complete(&ctx->work);
		trace_fastrpc_msg("wakeup_task: end");
		break;
	default:
		break;
	}
}

static void fastrpc_handle_signal_rpmsg(uint64_t msg, struct fastrpc_channel_ctx *cctx)
{
	u32 pid = msg >> 32;
	u32 signal_id = msg & 0xffffffff;
	struct fastrpc_user *fl ;
	unsigned long irq_flags = 0;
	bool process_found = false;

	if (signal_id >=FASTRPC_DSPSIGNAL_NUM_SIGNALS)
		return;

	spin_lock_irqsave(&cctx->lock, irq_flags);
	list_for_each_entry(fl, &cctx->users, user) {
		if (fl->tgid_frpc == pid && atomic_read(&fl->state) < DSP_EXIT_START) {
			process_found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&cctx->lock, irq_flags);

	if (!process_found) {
		pr_warn("Warning: %s: no active processes found for pid %u, signal id %u",
			__func__, pid, signal_id);
		return;
	}

	spin_lock_irqsave(&fl->dspsignals_lock, irq_flags);
	if (fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE]) {
		struct fastrpc_dspsignal *group =
			fl->signal_groups[signal_id /FASTRPC_DSPSIGNAL_GROUP_SIZE];
		struct fastrpc_dspsignal *sig =
			&group[signal_id %FASTRPC_DSPSIGNAL_GROUP_SIZE];
		if ((sig->state == DSPSIGNAL_STATE_PENDING) ||
			(sig->state == DSPSIGNAL_STATE_SIGNALED)) {
			trace_fastrpc_dspsignal("complete", signal_id, sig->state, 0);
			complete(&sig->comp);
			sig->state = DSPSIGNAL_STATE_SIGNALED;
		} else if (sig->state == DSPSIGNAL_STATE_UNUSED) {
			pr_err("Received unknown signal %u for PID %u\n",
					signal_id, pid);
		}
	} else {
		pr_err("Received unknown signal %u for PID %u\n",
				signal_id, pid);
	}
	spin_unlock_irqrestore(&fl->dspsignals_lock, irq_flags);
}

int fastrpc_handle_rpc_response(struct fastrpc_channel_ctx *cctx, void *data, int len)
{
	struct fastrpc_invoke_rsp *rsp = data;
	struct fastrpc_invoke_rspv2 *rspv2 = NULL;
	struct dsp_notif_rsp *notif = (struct dsp_notif_rsp *)data;
	struct fastrpc_invoke_ctx *ctx;
	unsigned long flags = 0, idr = 0;
	u64 ctxid = 0;
	u32 rsp_flags = 0, early_wake_time = 0, version = 0;

	if (len == sizeof(uint64_t)) {
		trace_fastrpc_transport_response(cctx->domain_id, *((uint64_t *)data), 0, 0, 0);
		fastrpc_handle_signal_rpmsg(*((uint64_t *)data), cctx);
		return 0;
	}

	if (notif->ctx == FASTRPC_NOTIF_CTX_RESERVED) {
		if (notif->type == STATUS_RESPONSE && len >= sizeof(*notif)) {
			fastrpc_notif_find_process(cctx->domain_id, cctx, notif);
			return 0;
		} else {
			return -ENOENT;
		}
	}

	if (len < sizeof(*rsp))
		return -EINVAL;

	if (len >= sizeof(*rspv2)) {
		rspv2 = data;
		if (rspv2) {
			early_wake_time = rspv2->early_wake_time;
			rsp_flags = rspv2->flags;
			version = rspv2->version;
		}
	}

	fastrpc_update_rxmsg_buf(cctx, rsp->ctx, rsp->retval,
		rsp_flags, early_wake_time, version, get_timestamp_in_ns());
	trace_fastrpc_transport_response(cctx->domain_id, rsp->ctx,
			rsp->retval, rsp_flags, early_wake_time);

	idr = FASTRPC_GET_IDR_FROM_CTXID(rsp->ctx);
	ctxid = FASTRPC_GET_CTXID_FROM_RSP_CTX(rsp->ctx);

	spin_lock_irqsave(&cctx->lock, flags);
	ctx = idr_find(&cctx->ctx_idr, idr);

	if (!ctx) {
		spin_unlock_irqrestore(&cctx->lock, flags);
		return 0;
	}

	if (ctx->ctxid != ctxid) {
		spin_unlock_irqrestore(&cctx->lock, flags);
		dev_info(cctx->dev,
			"Warning: rsp ctxid 0x%llx mismatch with local ctxid 0x%llx (full rsp ctx 0x%llx)",
				ctxid, ctx->ctxid, rsp->ctx);
		return 0;
	}

	if (rspv2) {
		if (rspv2->version != FASTRPC_RSP_VERSION2) {
			dev_err(cctx->dev, "Incorrect response version %d\n", rspv2->version);
			spin_unlock_irqrestore(&cctx->lock, flags);
			return -EINVAL;
		}
	}
	fastrpc_notify_user_ctx(ctx, rsp->retval, rsp_flags, early_wake_time);
	spin_unlock_irqrestore(&cctx->lock, flags);
	/*
	 * The DMA buffer associated with the context cannot be freed in
	 * interrupt context so schedule it through a worker thread to
	 * avoid a kernel BUG.
	 */
	// schedule_work(&ctx->put_work);

	return 0;
}

static int fastrpc_init(void)
{
	int ret;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root = NULL;
#endif

	spin_lock_init(&g_frpc.glock);
	ret = platform_driver_register(&fastrpc_cb_driver);
	if (ret < 0) {
		pr_err("fastrpc: failed to register cb driver\n");
		return ret;
	}

	ret = fastrpc_transport_init();
	if (ret < 0) {
		pr_err("fastrpc: failed to register rpmsg driver\n");
		platform_driver_unregister(&fastrpc_cb_driver);
		return ret;
	}
#ifdef CONFIG_DEBUG_FS
	debugfs_root = debugfs_create_dir("fastrpc", NULL);
	if (IS_ERR_OR_NULL(debugfs_root)) {
		pr_warn("Error: %s: %s: failed to create debugfs root dir\n",
			current->comm, __func__);
		debugfs_remove_recursive(debugfs_root);
		debugfs_root = NULL;
	}
	g_frpc.debugfs_root = debugfs_root;
#endif
	return 0;
}
module_init(fastrpc_init);

static void fastrpc_exit(void)
{
	platform_driver_unregister(&fastrpc_cb_driver);
	fastrpc_transport_deinit();
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(g_frpc.debugfs_root);
#endif
}
module_exit(fastrpc_exit);

MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(DMA_BUF);
