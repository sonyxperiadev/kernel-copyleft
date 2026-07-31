// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "si-mo: %s: " fmt, __func__

#include <linux/platform_device.h>
#include <linux/dma-buf.h>
#include <linux/mem-buf.h>
#include <linux/of_platform.h>
#include <linux/qtee_shmbridge.h>
#include <linux/firmware/qcom/si_core_xts.h>

#include "si_core.h"
#include "mem-object.h"

/* Memory object operations. */
/* ... */

/* 'Primordial Object' operations related to memory object. */
#define OBJECT_OP_MAP_REGION_SHM	0
#define OBJECT_OP_MAP_REGION_FFA	3

#define SMCINVOKE_MIN_ASYNC_VERSION 0x00010002U
#define SMCINVOKE_ASYNC_VERSION_SHM 0x00010002U
#define SMCINVOKE_ASYNC_VERSION_FFA 0x00010003U

/* TZ defined values for cacheability */
#define CACHE_NS_CACHED 0x10000000U
#define CACHE_UNCACHED 0x20000000U

static struct platform_device *mem_object_pdev;

static struct si_object primordial_object;

/* **/
/* Memory object reference counting details:
 * There is one reference counter in memory object, i.e. 'object'.
 * 'object' counts number of times this object has been exported to TZ plus
 * total number of mappings plus one (for ownership reference).
 *
 * HOW IT WORKS
 *
 * Client obtains an instance of 'si_object' by calling 'init_si_mem_object_user'
 * with an instance of 'struct dma_buf' to initialize a memory object. It can
 * immediately use this instance of 'si_object' to share memory with TZ.
 * However, by transferring this object to TZ, client will lose it's ownership.
 * To retain the ownership it should call 'get_si_object' and send a second
 * instance of this object to TZ while keeping the initial 'si_object' instance
 * (hence plus one for ownership).
 *
 * Every time TZ request mapping of the memory object, the driver issues
 * 'get_si_object' on 'object'.
 *
 **/

struct mem_object {
	struct si_object object;

	struct dma_buf *dma_buf;

	uint64_t flags;

	struct map {
		struct dma_buf_attachment *buf_attach;
		struct sg_table *sgt;
		bool owned;

		/* 'lock' to protect concurrent request from QTEE. */
		struct mutex lock;
		int early_mapped;
	} map;

	/* Either use an SHMBridge or FFA handle. */
	u64 handle;
	u64 tag;

	/* Either SHM bridge or FFA mapping info. */
	union {
		struct shm_mapping_info {
			phys_addr_t p_addr;
			size_t p_addr_len;
			uint32_t perms;
		} shm_mapping_info;

		struct ffa_mapping_info {
			u64 ffa_tag;
			size_t offset;
			size_t size;
			uint32_t mem_attr;
		} ffa_mapping_info;

		/* XXX information. */
		/* struct { ... } */
	};

	struct list_head node;

	/* Private pointer passed for callbacks. */

	void *private;

	void (*release)(void *private);
};

struct mi_shm {
	u64 p_addr;
	u64 len;
	u32 perms;
};

struct mi_ffa {
	u64 ffa_handle;
	u64 ffa_tag;
	size_t offset;
	size_t size;
	u32 mem_attr;
};

#define to_mem_object(o) container_of((o), struct mem_object, object)

/* List of memory objects. Only used for sysfs. */

static LIST_HEAD(mo_list);
static DEFINE_MUTEX(mo_list_mutex);

/* 'mo_notify' and 'mo_dispatch' are shared by all types of memory objects. */

static void mo_notify(unsigned int context_id, struct si_object *object, int status)
{

}

static int mo_dispatch(unsigned int context_id,
	struct si_object *object, unsigned long op, struct si_arg args[])
{
	return 0;
}

static struct si_object_operations mem_ops = {
	.notify = mo_notify,
	.dispatch = mo_dispatch
};

#if IS_ENABLED(CONFIG_QCOM_SI_CORE_MEM_FFA)
int op_supported(unsigned long op)
{
	switch (op) {
	case OBJECT_OP_MAP_REGION_SHM:
		return 0;
	case OBJECT_OP_MAP_REGION_FFA:
		return 1;
	default:
		return 0;
	}
}

/** Support for FFA based memory sharing, which supports scattered memory **/

static int map_via_ffa_abi(struct mem_object *mo)
{
	int ret;
	unsigned int i;
	size_t total_len = 0;
	struct sg_table *sgt = mo->map.sgt;
	struct scatterlist *sgl = sgt->sgl;
	unsigned int nents = sg_nents(sgl);
	struct scatterlist *sg;

	if (mo->flags & SI_CORE_MEM_OBJ_LEND)
		ret = qtee_ffa_mem_lend(sgt, mo->tag, &mo->handle);
	else /* By default, we always share */
		ret = qtee_ffa_mem_share(sgt, mo->tag, &mo->handle);

	if (ret) {
		mo->handle = 0;
		goto out;
	}

	/* A DMA API such as dma_alloc_coherent() was used to allocate
	 * this memory
	 */
	if (sg_dma_address(sgl)) {
		for_each_sg(sgl, sg, nents, i) {
			total_len += sg_dma_len(sg);
		}
	} else {
		for_each_sg(sgl, sg, nents, i) {
			total_len += sg->length;
		}
	}

	mo->ffa_mapping_info.ffa_tag = mo->tag;
	mo->ffa_mapping_info.offset = 0;
	mo->ffa_mapping_info.size = total_len;
	mo->ffa_mapping_info.mem_attr = QCOM_SCM_PERM_RW;

out:
	return ret;
}

static int register_tz_shm(struct mem_object *mo)
{
	return map_via_ffa_abi(mo);
}

static void deregister_tz_shm(struct mem_object *mo)
{
	if (mo->handle)
		qtee_ffa_mem_reclaim(mo->handle);

	mo->handle = 0;
}

#else /* CONFIG_QCOM_SI_CORE_MEM_FFA */

int op_supported(unsigned long op)
{
	switch (op) {
	case OBJECT_OP_MAP_REGION_SHM:
		return 1;
	case OBJECT_OP_MAP_REGION_FFA:
		return 0;
	default:
		return 0;
	}
}

/** Support for 'SHMBridge'. Which only supports continuous memory **/

/* 'map_via_shm_bridge' only support single continuous memory. */

static int map_via_shm_bridge(struct mem_object *mo)
{
	int ret;
	u32 *vmid_list, *perms_list, nelems;
	u32 hlos_vmid = QCOM_SCM_VMID_HLOS;
	u32 hlos_perms = QCOM_SCM_PERM_RW;
	struct scatterlist *sgl;

	if (mo->map.sgt->nents != 1)
		return -EINVAL;

	sgl = mo->map.sgt->sgl;

	/* If this MO is associated with a dmabuf, get the associated
	 * VM permissions for sharing with QTEE
	 */
	if (mo->dma_buf) {
		ret = mem_buf_dma_buf_copy_vmperm(mo->dma_buf,
						 (int **)(&vmid_list),
						 (int **)(&perms_list),
						 (int *)(&nelems));

		if (ret)
			return ret;

		if (mem_buf_dma_buf_exclusive_owner(mo->dma_buf))
			perms_list[0] = QCOM_SCM_PERM_RW;

		mo->shm_mapping_info.p_addr = sg_dma_address(sgl);
		mo->shm_mapping_info.p_addr_len = sg_dma_len(sgl);
	} else {
		/* TODO: Add and fetch vmid from DTSI */
		vmid_list = &hlos_vmid;
		perms_list = &hlos_perms;
		nelems = 1;

		/* A DMA API such as dma_alloc_coherent() was used to allocate
		 * this memory
		 */
		if (sg_dma_address(sgl)) {
			mo->shm_mapping_info.p_addr = sg_dma_address(sgl);
			mo->shm_mapping_info.p_addr_len = sg_dma_len(sgl);
		} else  {
			mo->shm_mapping_info.p_addr = page_to_phys(sg_page(sgl));
			mo->shm_mapping_info.p_addr_len = sgl->length;
		}
	}

	mo->shm_mapping_info.perms = QCOM_SCM_PERM_RW;

	ret = qtee_shmbridge_register(mo->shm_mapping_info.p_addr,
				      mo->shm_mapping_info.p_addr_len,
				      vmid_list, perms_list, nelems,
				      mo->shm_mapping_info.perms,
				      &mo->handle);

	if (mo->dma_buf) {
		kfree(perms_list);
		kfree(vmid_list);
	}

	if (ret) {
		/* If 'handle' is not zero, then the memory object is already mapped. */

		mo->shm_mapping_info.p_addr = 0;
		mo->shm_mapping_info.p_addr_len = 0;
		// SCM driver touch this value even an failure so set to 0
		mo->handle = 0;
	}

	return ret;
}

static int register_tz_shm(struct mem_object *mo)
{
	return map_via_shm_bridge(mo);
}

static void deregister_tz_shm(struct mem_object *mo)
{
	if (mo->handle)
		qtee_shmbridge_deregister(mo->handle);

	mo->handle = 0;
}

#endif /* CONFIG_QCOM_SI_CORE_MEM_FFA */

static void free_mo_sgt(struct mem_object *mo)
{
	struct sg_table *sgt;
	struct scatterlist *sglist;
	unsigned int nents;
	struct scatterlist *sg;
	int i;
	struct page *page;

	if (mo->map.sgt && mo->map.owned) {
		sgt = mo->map.sgt;
		sglist = sgt->sgl;
		nents = sg_nents(sglist);

		for_each_sg(sglist, sg, nents, i) {
			page = sg_page(sg);
			if (page)
				__free_page(page);
		}

		sg_free_table(sgt);
		kfree(sgt);
	}
}

static void detach_dma_buf(struct mem_object *mo)
{
	if (mo->map.sgt) {
		dma_buf_unmap_attachment_unlocked(mo->map.buf_attach,
			mo->map.sgt, DMA_BIDIRECTIONAL);
	}

	if (mo->map.buf_attach)
		dma_buf_detach(mo->dma_buf, mo->map.buf_attach);

	mo->map.buf_attach = NULL;
	mo->map.sgt = NULL;
}

/* 'init_tz_shared_memory' is called while holding the 'map.lock' mutex. */

static int init_tz_shared_memory(struct mem_object *mo)
{
	int ret;
	struct dma_buf_attachment *buf_attach;
	struct sg_table *sgt;

	mo->map.buf_attach = NULL;
	mo->map.sgt = NULL;

	buf_attach = dma_buf_attach(mo->dma_buf, &mem_object_pdev->dev);
	if (IS_ERR(buf_attach))
		return PTR_ERR(buf_attach);

	mo->map.buf_attach = buf_attach;

	sgt = dma_buf_map_attachment_unlocked(buf_attach, DMA_BIDIRECTIONAL);

	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);

		goto out_failed;
	}

	mo->map.sgt = sgt;

	ret = register_tz_shm(mo);
	if (ret)
		goto out_failed;

	return 0;

out_failed:
	detach_dma_buf(mo);

	return ret;
}

static int map_memory_obj(struct mem_object *mo, int advisory)
{
	int ret;

	if (mo->map.early_mapped)
		pr_debug("%s auto-mapped. Memory optimization unavailable.\n",
			si_object_name(&mo->object));

	mutex_lock(&mo->map.lock);
	if (mo->handle == 0) {
		/* 'mo' has not been mapped before. Do it now. */
		if (mo->dma_buf) {
			/* A dmabuf associated 'mo' which needs mapping */
			ret = init_tz_shared_memory(mo);
		} else {
			/* A plain 'mo', sgt already prepared for sharing */
			ret = register_tz_shm(mo);
		}
	} else {

		/* 'mo' is already mapped. Just return. */

		ret = advisory;
	}

	mutex_unlock(&mo->map.lock);

	return ret;
}

static void release_memory_obj(struct mem_object *mo)
{
	deregister_tz_shm(mo);

	if (mo->dma_buf)
		detach_dma_buf(mo);
	else
		free_mo_sgt(mo);
}

#if IS_ENABLED(CONFIG_QCOM_SI_CORE_MEM_FFA)
static unsigned long mo_prepare_ffa(struct si_object *object, struct si_arg args[])
{
	struct mem_object *mo = to_mem_object(object);
	struct mi_ffa *ffa;

	if (args[0].b.size < sizeof(struct mi_ffa))
		return SI_OBJECT_OP_NO_OP;

	if (!map_memory_obj(mo, 1)) {
		mo->map.early_mapped = 1;

		/* 'object' has been mapped. Share it. */

		get_si_object(object);

		ffa = (struct mi_ffa *)(args[0].b.addr);

		ffa->ffa_handle = mo->handle;
		ffa->ffa_tag = mo->ffa_mapping_info.ffa_tag;
		ffa->offset = mo->ffa_mapping_info.offset;
		ffa->size = mo->ffa_mapping_info.size;
		/* append cacheability info to upper nibble */
		if (mo->flags & SI_CORE_MEM_OBJ_UNCACHED)
			ffa->mem_attr = CACHE_UNCACHED | mo->ffa_mapping_info.mem_attr;
		else
			ffa->mem_attr = CACHE_NS_CACHED | mo->ffa_mapping_info.mem_attr;

		args[0].b.size = sizeof(*ffa);

		args[1].o = object;

		return OBJECT_OP_AUTO_MAP_FFA;
	}

	return SI_OBJECT_OP_NO_OP;
}

static unsigned long mo_prepare_shm(struct si_object *object, struct si_arg args[])
{
	pr_err("mapping of a memory object only supported via ffa.\n");
	return SI_OBJECT_OP_NO_OP;
}

#else /* CONFIG_QCOM_SI_CORE_MEM_FFA */

static unsigned long mo_prepare_ffa(struct si_object *object, struct si_arg args[])
{
	pr_err("mapping of a memory object only supported via shmb.\n");
	return SI_OBJECT_OP_NO_OP;
}

static unsigned long mo_prepare_shm(struct si_object *object, struct si_arg args[])
{
	struct mem_object *mo = to_mem_object(object);
	struct mi_shm *shm;

	if (args[0].b.size < sizeof(struct mi_shm))
		return SI_OBJECT_OP_NO_OP;

	if (!map_memory_obj(mo, 1)) {
		mo->map.early_mapped = 1;

		/* 'object' has been mapped. Share it. */

		get_si_object(object);

		shm = (struct mi_shm *)(args[0].b.addr);

		shm->p_addr = mo->shm_mapping_info.p_addr;
		shm->len = mo->shm_mapping_info.p_addr_len;
		/* append cacheability info to upper nibble */
		if (mo->flags & SI_CORE_MEM_OBJ_UNCACHED)
			shm->perms = CACHE_UNCACHED | mo->shm_mapping_info.perms;
		else
			shm->perms = CACHE_NS_CACHED | mo->shm_mapping_info.perms;

		args[0].b.size = sizeof(*shm);

		args[1].o = object;

		return OBJECT_OP_AUTO_MAP_SHM;
	}

	return SI_OBJECT_OP_NO_OP;
}

#endif /* CONFIG_QCOM_SI_CORE_MEM_FFA */

static unsigned long mo_prepare(struct si_object *object, struct si_arg args[])
{
	unsigned long ret;
	uint32_t async_version = get_async_proto_version();

	if (async_version < SMCINVOKE_MIN_ASYNC_VERSION)
		return SI_OBJECT_OP_NO_OP;

	/* QTEE bumps up the async protocol minor version on introducing
	 * a new handler. FFA based memory object is supported from
	 * SMCINVOKE_ASYNC_VERSION_FFA onwards.
	 */
	if (async_version < SMCINVOKE_ASYNC_VERSION_FFA)
		ret = mo_prepare_shm(object, args);
	else
		ret = mo_prepare_ffa(object, args);

	return ret;
}

static void mo_release(struct si_object *object)
{
	struct mem_object *mo = to_mem_object(object);

	release_memory_obj(mo);

	if (mo->release)
		mo->release(mo->private);

	/* Put a dma-buf copy obtained in 'init_si_mem_object_user'.*/

	if (mo->dma_buf)
		dma_buf_put(mo->dma_buf);

	mutex_lock(&mo_list_mutex);
	list_del(&mo->node);
	mutex_unlock(&mo_list_mutex);

	pr_info("%s unmapped.\n", si_object_name(object));

	kfree(mo);
}

#if IS_ENABLED(CONFIG_QCOM_SI_CORE_MEM_FFA)

static int map_memory_obj_ffa(struct si_arg args[])
{
	int ret;
	struct mi_ffa *ffa;
	struct si_object *object;
	struct mem_object *mo;

	/* Format of response as expected by TZ. */

	if (size_of_arg(args) != 3 ||
	    args[0].type != SI_AT_OB  ||
	    args[1].type != SI_AT_IO  ||
	    args[2].type != SI_AT_OO) {

		pr_err("mapping of a memory object with invalid message format.\n");

		return -EINVAL;
	}

	object = args[1].o;

	if (!is_mem_object(object)) {
		pr_err("mapping of a non-memory object.\n");

		put_si_object(object);
		return -EINVAL;
	}

	mo = to_mem_object(object);

	ret = map_memory_obj(mo, 0);
	if (!ret) {

		/* 'object' has been mapped. Share it. */

		args[2].o = object;

		ffa = (struct mi_ffa *)(args[0].b.addr);

		ffa->ffa_handle = mo->handle;
		ffa->ffa_tag = mo->ffa_mapping_info.ffa_tag;
		ffa->offset = mo->ffa_mapping_info.offset;
		ffa->size = mo->ffa_mapping_info.size;
		/* append cacheability info to upper nibble */
		if (mo->flags & SI_CORE_MEM_OBJ_UNCACHED)
			ffa->mem_attr = CACHE_UNCACHED | mo->ffa_mapping_info.mem_attr;
		else
			ffa->mem_attr = CACHE_NS_CACHED | mo->ffa_mapping_info.mem_attr;

		pr_info("%s ffa-mapped %llx %lx\n",
			si_object_name(object), ffa->ffa_handle, ffa->size);
	} else {
		pr_err("mapping memory object via ffa %s failed.\n",
			si_object_name(object));

		put_si_object(object);
	}

	return ret;
}

static int map_memory_obj_shm(struct si_arg args[])
{
	pr_err("mapping of a memory object only supported via ffa.\n");
	return -EOPNOTSUPP;
}

#else /* CONFIG_QCOM_SI_CORE_MEM_FFA */

static int map_memory_obj_ffa(struct si_arg args[])
{
	pr_err("mapping of a memory object only supported via shmb.\n");
	return -EOPNOTSUPP;
}

static int map_memory_obj_shm(struct si_arg args[])
{
	int ret;
	struct mi_shm *shm;
	struct si_object *object;
	struct mem_object *mo;

	if (size_of_arg(args) != 3 ||
	    args[0].type != SI_AT_OB  ||
	    args[1].type != SI_AT_IO  ||
	    args[2].type != SI_AT_OO) {

		pr_err("mapping of a memory object with invalid message format.\n");

		return -EINVAL;
	}

	object = args[1].o;

	if (!is_mem_object(object)) {
		pr_err("mapping of a non-memory object.\n");

		put_si_object(object);
		return -EINVAL;
	}

	mo = to_mem_object(object);

	ret = map_memory_obj(mo, 0);
	if (!ret) {

		/* 'object' has been mapped. Share it. */

		args[2].o = object;

		shm = (struct mi_shm *)(args[0].b.addr);

		shm->p_addr = mo->shm_mapping_info.p_addr;
		shm->len = mo->shm_mapping_info.p_addr_len;
		/* append cacheability info to upper nibble */
		if (mo->flags & SI_CORE_MEM_OBJ_UNCACHED)
			shm->perms = CACHE_UNCACHED | mo->shm_mapping_info.perms;
		else
			shm->perms = CACHE_NS_CACHED | mo->shm_mapping_info.perms;

		pr_info("%s shm-mapped %llx %llx\n",
			si_object_name(object), shm->p_addr, shm->len);
	} else {
		pr_err("mapping memory object via shmb %s failed.\n",
			si_object_name(object));

		put_si_object(object);
	}

	return ret;
}

#endif /* CONFIG_QCOM_SI_CORE_MEM_FFA */

/* Primordial object for 'SHMBridge'. */

static int shm_bridge__po_dispatch(unsigned int context_id,
	struct si_object *unused, unsigned long op, struct si_arg args[])
{
	int ret;

	switch (op) {
	case OBJECT_OP_MAP_REGION_SHM:

		ret = map_memory_obj_shm(args);

		break;
	case OBJECT_OP_MAP_REGION_FFA:

		ret = map_memory_obj_ffa(args);

		break;
	default:

		/* The operation is not supported! */

		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct si_object_operations shm_bridge__po_ops = {
	.op_supported = op_supported,
	.dispatch = shm_bridge__po_dispatch
};

/* Memory Object Extension. */

struct si_object *init_si_mem_object_user(struct dma_buf *dma_buf,
	void (*release)(void *), void *private)
{
	struct mem_object *mo;

	if (!mem_ops.release) {
		pr_err("memory object type is unknown.\n");

		return NULL_SI_OBJECT;
	}

	mo = kzalloc(sizeof(*mo), GFP_KERNEL);
	if (!mo)
		return NULL_SI_OBJECT;

	mutex_init(&mo->map.lock);

	/* Get a copy of dma-buf. */
	get_dma_buf(dma_buf);

	mo->dma_buf = dma_buf;
	mo->private = private;
	mo->release = release;

	init_si_object_user(&mo->object, SI_OT_CB_OBJECT, &mem_ops, "mem-object-%p", dma_buf);

	mutex_lock(&mo_list_mutex);
	list_add_tail(&mo->node, &mo_list);
	mutex_unlock(&mo_list_mutex);

	return &mo->object;
}
EXPORT_SYMBOL_GPL(init_si_mem_object_user);

#if IS_ENABLED(CONFIG_QCOM_SI_CORE_MEM_FFA)
static const unsigned int order_table[] = { 8, 4, 0 };

static void si_core_free_page_policy(struct list_head *pages)
{
	struct page *page, *tmp_page;

	list_for_each_entry_safe(page, tmp_page, pages, lru) {
		list_del(&page->lru);
		__free_pages(page, compound_order(page));
	}
}

static size_t si_core_alloc_page_policy(struct list_head *pages, size_t size,
					gfp_t gfp_base)
{
	size_t size_remaining = PAGE_ALIGN(size);
	size_t count = 0;

	for (int i = 0; i < ARRAY_SIZE(order_table) && size_remaining; i++) {
		/* Next order to allocate: */
		const unsigned int order = order_table[i];
		const size_t chunk = PAGE_SIZE << order;

		gfp_t gfp = gfp_base;
		/* Some default flag combinations: */
		if (order) {
			gfp |= __GFP_COMP;
			/* Don't thrash if higher order pages are scarce. */
			gfp |= __GFP_NOWARN | __GFP_NORETRY;
		}

		while (size_remaining >= chunk) {
			struct page *page;

			/* TODO use dma_alloc_pages. */
			page = alloc_pages(gfp, order);
			if (!page) /* drop to next smaller order  */
				break;

			list_add_tail(&page->lru, pages);
			size_remaining -= chunk;
			count++;
		}
	}

	if (size_remaining) {
		si_core_free_page_policy(pages);
		return -ENOMEM;
	}

	return count;
}

struct si_object *init_si_mem_object_pages(size_t *size, uint64_t tag,
					   uint64_t flags, void (*release)(void *),
					   void *private)
{
	int rc;
	struct page *page, *tmp_page;
	unsigned int chunk_size;
	struct sg_table *sgt;
	struct scatterlist *sg;
	LIST_HEAD(page_list);
	int count;
	struct mem_object *mo;

	if (!size || *size == 0) {
		pr_err("invalid size input!\n");
		return NULL_SI_OBJECT;
	}

	if (!mem_ops.release) {
		pr_err("memory object type is unknown.\n");
		return NULL_SI_OBJECT;
	}

	/* This API only allocates at page granularity */
	*size = PAGE_ALIGN(*size);

	count = si_core_alloc_page_policy(&page_list, *size, GFP_KERNEL);
	if (!count) {
		pr_err("si_core_alloc_page_policy failed.\n");
		goto err_si_core_alloc_pages;
	}

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		goto err_sgt_alloc;

	rc = sg_alloc_table(sgt, count, GFP_KERNEL);
	if (rc)
		goto err_sg_alloc;

	/* Transfer page_list to sg_table. */
	sg = sgt->sgl;
	list_for_each_entry_safe(page, tmp_page, &page_list, lru) {
		chunk_size = PAGE_SIZE << compound_order(page);
		sg_set_page(sg, page, chunk_size, 0);
		sg = sg_next(sg);
		list_del(&page->lru);
	}

	mo = kzalloc(sizeof(*mo), GFP_KERNEL);
	if (!mo)
		goto err_mo_alloc;

	mutex_init(&mo->map.lock);

	mo->private = private;
	mo->release = release;

	mo->map.sgt = sgt;
	mo->map.owned = true;
	mo->flags = flags;
	mo->tag = tag;

	init_si_object_user(&mo->object, SI_OT_CB_OBJECT, &mem_ops,
		"kernel-mem-object-%p", sgt);

	mutex_lock(&mo_list_mutex);
	list_add_tail(&mo->node, &mo_list);
	mutex_unlock(&mo_list_mutex);

	return &mo->object;

err_mo_alloc:
	sg_free_table(sgt);
err_sg_alloc:
	kfree(sgt);
err_sgt_alloc:
	si_core_free_page_policy(&page_list);
err_si_core_alloc_pages:
	return NULL_SI_OBJECT;
}
EXPORT_SYMBOL_GPL(init_si_mem_object_pages);
#endif

struct si_object *init_si_mem_object_sg(struct sg_table *sgt, uint64_t tag,
					uint32_t flags, void (*release)(void *),
					void *private)
{
	struct mem_object *mo;

	if (!sgt) {
		pr_err("sgt not initialized!\n");
		return NULL_SI_OBJECT;
	}

#if !IS_ENABLED(CONFIG_QCOM_SI_CORE_MEM_FFA)
	/* Better to fail early here to avoid failuring during memory object
	 * mapping
	 */
	if (sgt->nents > 1) {
		pr_err("SHM bridge only maps continuous memory!\n");
		return NULL_SI_OBJECT;
	}
#endif

	if (!mem_ops.release) {
		pr_err("memory object type is unknown.\n");
		return NULL_SI_OBJECT;
	}

	mo = kzalloc(sizeof(*mo), GFP_KERNEL);
	if (!mo)
		return NULL_SI_OBJECT;

	mutex_init(&mo->map.lock);

	mo->private = private;
	mo->release = release;

	/* This sgt is not owned by us, we will not free it */
	mo->map.sgt = sgt;
	mo->map.owned = false;
	mo->flags = flags;
	mo->tag = tag;

	init_si_object_user(&mo->object, SI_OT_CB_OBJECT, &mem_ops,
		"kernel-mem-object-%p", sgt);

	mutex_lock(&mo_list_mutex);
	list_add_tail(&mo->node, &mo_list);
	mutex_unlock(&mo_list_mutex);

	return &mo->object;
}
EXPORT_SYMBOL_GPL(init_si_mem_object_sg);

int dma_map_mem_object(struct si_object *object, unsigned int nents)
{
	struct mem_object *mo;
	struct scatterlist *sglist;
	int count;

	if (!is_mem_object(object)) {
		pr_err("dma-mapping a non-memory object.\n");
		return -EINVAL;
	}

	mo = to_mem_object(object);
	sglist = mo->map.sgt->sgl;

	/* Create a DMA mapping and flush the cache line */
	count = dma_map_sg(&mem_object_pdev->dev, sglist, nents, DMA_TO_DEVICE);
	if (!count) {
		pr_err("dma_map_sg() failed memory object %s\n", si_object_name(object));
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dma_map_mem_object);

void dma_unmap_mem_object(struct si_object *object, unsigned int nents)
{
	struct mem_object *mo;
	struct scatterlist *sglist;

	if (!is_mem_object(object)) {
		pr_err("dma-unmapping a non-memory object.\n");
		return;
	}

	mo = to_mem_object(object);
	sglist = mo->map.sgt->sgl;

	/* Remove the DMA mapping and invalidate the cache line */
	dma_unmap_sg(&mem_object_pdev->dev, sglist, nents, DMA_TO_DEVICE);
}
EXPORT_SYMBOL_GPL(dma_unmap_mem_object);

struct sg_table *mem_object_to_sgt(struct si_object *object)
{
	if (is_mem_object(object))
		return to_mem_object(object)->map.sgt;

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(mem_object_to_sgt);

struct dma_buf *mem_object_to_dma_buf(struct si_object *object)
{
	if (is_mem_object(object))
		return to_mem_object(object)->dma_buf;

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(mem_object_to_dma_buf);

int is_mem_object(struct si_object *object)
{
	/* Check 'typeof_si_object' to make sure 'object''s 'ops' has been
	 * initialized before checking it.
	 */

	return (typeof_si_object(object) == SI_OT_CB_OBJECT) &&
		(object->ops == &mem_ops);
}
EXPORT_SYMBOL_GPL(is_mem_object);

int early_map_memory_obj(struct si_object *object)
{
	int ret;
	struct mem_object *mo;
	int advisory = 1;

	if (!is_mem_object(object)) {
		pr_err("mapping of a non-memory object.\n");
		return -EINVAL;
	}

	mo = to_mem_object(object);

	ret = map_memory_obj(mo, advisory);
	if (ret == advisory) {
		pr_err("failed to early map, memory object already mapped!\n");
		return -EINVAL;
	}

	if (ret) {
		pr_err("failed to early map memory object. ret %d\n", ret);
		return ret;
	}

	mo->map.early_mapped = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(early_map_memory_obj);

ssize_t mem_objects_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	size_t len = 0;
	struct mem_object *mo;

	mutex_lock(&mo_list_mutex);

#if IS_ENABLED(CONFIG_QCOM_SI_CORE_MEM_FFA)
	list_for_each_entry(mo, &mo_list, node) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
				"%s %u (%llx %zx) %d\n",
				si_object_name(&mo->object),
				kref_read(&mo->object.refcount),
				mo->handle,
				mo->ffa_mapping_info.size,
				mo->map.early_mapped);
	}
#else
	list_for_each_entry(mo, &mo_list, node) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
				"%s %u (%llx %zx) %d\n",
				si_object_name(&mo->object),
				kref_read(&mo->object.refcount),
				mo->shm_mapping_info.p_addr,
				mo->shm_mapping_info.p_addr_len,
				mo->map.early_mapped);
	}
#endif
	mutex_unlock(&mo_list_mutex);

	return len;
}

int mem_object_init(struct platform_device *pdev)
{
	/* Select memory object type: default to SHMBridge. */
	mem_ops.release = mo_release;
	mem_ops.prepare = mo_prepare;

	init_si_object_user(&primordial_object,
		SI_OT_ROOT, &shm_bridge__po_ops, "po_in_mem_object");

	mem_object_pdev = pdev;

	return 0;
}

