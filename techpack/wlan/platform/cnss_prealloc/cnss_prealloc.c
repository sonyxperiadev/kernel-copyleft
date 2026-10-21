// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012,2014-2017,2019-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mempool.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/version.h>
#ifdef CONFIG_CNSS_OUT_OF_TREE
#include "cnss_prealloc.h"
#else
#include <net/cnss_prealloc.h>
#endif

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CNSS prealloc driver");

/* cnss preallocation scheme is a memory pool that always tries to keep a
 * list of free memory for use in emergencies. It is implemented on kernel
 * features: memorypool and kmem cache.
 */

struct cnss_pool {
	size_t size;
	int min;
	const char name[50];
	mempool_t *mp;
	struct kmem_cache *cache;
};

/**
 * Memory pool
 * -----------
 *
 * How to update this table:
 *
 *  1. Add a new row with following elements
 *      size  : Size of one allocation unit in bytes.
 *      min   : Minimum units to be reserved. Used only if a regular
 *              allocation fails.
 *      name  : Name of the cache/pool. Will be displayed in /proc/slabinfo
 *              if not merged with another pool.
 *      mp    : A pointer to memory pool. Updated during init.
 *      cache : A pointer to cache. Updated during init.
 * 2. Always keep the table in increasing order
 * 3. Please keep the reserve pool as minimum as possible as it's always
 *    preallocated.
 * 4. Always profile with different use cases after updating this table.
 * 5. A dynamic view of this pool can be viewed at /proc/slabinfo.
 * 6. Each pool has a sys node at /sys/kernel/slab/<name>
 *
 */

/* size, min pool reserve, name, memorypool handler, cache handler*/
static struct cnss_pool cnss_pools[] = {
	{8 * 1024, 16, "cnss-pool-8k", NULL, NULL},
	{16 * 1024, 16, "cnss-pool-16k", NULL, NULL},
	{32 * 1024, 22, "cnss-pool-32k", NULL, NULL},
	{64 * 1024, 38, "cnss-pool-64k", NULL, NULL},
	{128 * 1024, 10, "cnss-pool-128k", NULL, NULL},
};

/**
 * cnss_pool_alloc_threshold() - Allocation threshold
 *
 * Minimum memory size to be part of cnss pool.
 *
 * Return: Size
 *
 */
static inline size_t cnss_pool_alloc_threshold(void)
{
	return cnss_pools[0].size;
}

/**
 * cnss_pool_int() - Initialize memory pools.
 *
 * Create cnss pools as configured by cnss_pools[]. It is the responsibility of
 * the caller to invoke cnss_pool_deinit() routine to clean it up. This
 * function needs to be called at early boot to preallocate minimum buffers in
 * the pool.
 *
 * Return: 0 - success, otherwise error code.
 *
 */
static int cnss_pool_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cnss_pools); i++) {
		/* Create the slab cache */
		cnss_pools[i].cache =
			kmem_cache_create_usercopy(cnss_pools[i].name,
						   cnss_pools[i].size, 0,
						   SLAB_ACCOUNT, 0,
						   cnss_pools[i].size, NULL);
		if (!cnss_pools[i].cache) {
			pr_err("cnss_prealloc: cache %s failed\n",
			       cnss_pools[i].name);
			continue;
		}

		/* Create the pool and associate to slab cache */
		cnss_pools[i].mp =
		    mempool_create(cnss_pools[i].min, mempool_alloc_slab,
				   mempool_free_slab, cnss_pools[i].cache);

		if (!cnss_pools[i].mp) {
			pr_err("cnss_prealloc: mempool %s failed\n",
			       cnss_pools[i].name);
			kmem_cache_destroy(cnss_pools[i].cache);
			cnss_pools[i].cache = NULL;
			continue;
		}

		pr_info("cnss_prealloc: created mempool %s of min size %d * %zu\n",
			cnss_pools[i].name, cnss_pools[i].min,
			cnss_pools[i].size);
	}

	return 0;
}

/**
 * cnss_pool_deinit() - Free memory pools.
 *
 * Free the memory pools and return resources back to the system. It warns
 * if there is any pending element in memory pool or cache.
 *
 */
static void cnss_pool_deinit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cnss_pools); i++) {
		pr_info("cnss_prealloc: destroy mempool %s\n",
			cnss_pools[i].name);
		mempool_destroy(cnss_pools[i].mp);
		kmem_cache_destroy(cnss_pools[i].cache);
	}
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
/* In kernel 5.17, slab_cache is removed from page struct, so
 * store cache in the beginning of memory buffer.
 */
static inline void cnss_pool_put_cache_in_mem(void *mem, struct kmem_cache *cache)
{
	/* put cache at the beginnging of mem */
	(*(struct kmem_cache **)mem) = cache;
}

static inline struct kmem_cache *cnss_pool_get_cache_from_mem(void *mem)
{
	struct kmem_cache *cache;

	/* read cache from the beginnging of mem */
	cache = (struct kmem_cache *)(*(struct kmem_cache **)mem);

	return cache;
}
#else /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)) */
/* for older kernel < 5.17, we use page->slab_cache. In such case
 * we do not reserve headroom in memory buffer to store cache.
 */
static inline void cnss_pool_put_cache_in_mem(void *mem, struct kmem_cache *cache)
{
}

static inline struct kmem_cache *cnss_pool_get_cache_from_mem(void *mem)
{
	struct page *page;

	if (!virt_addr_valid(mem))
		return NULL;

	/* mem -> page -> cache */
	page = virt_to_head_page(mem);
	if (!page)
		return NULL;

	return page->slab_cache;
}
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)) */

/**
 * cnss_pool_get_index() - Get the index of memory pool
 * @mem: Allocated memory
 *
 * Returns the index of the memory pool which fits the reqested memory. The
 * complexity of this check is O(num of memory pools). Returns a negative
 * value with error code in case of failure.
 *
 */
static int cnss_pool_get_index(void *mem)
{
	struct kmem_cache *cache;
	int i;

	cache = cnss_pool_get_cache_from_mem(mem);
	if (!cache)
		return -ENOENT;

	/* Check if memory belongs to a pool */
	for (i = 0; i < ARRAY_SIZE(cnss_pools); i++) {
		if (cnss_pools[i].cache == cache)
			return i;
	}

	return -ENOENT;
}

/**
 * wcnss_prealloc_get() - Get preallocated memory from a pool
 * @size: Size to allocate
 *
 * Memory pool is chosen based on the size. If memory is not available in a
 * given pool it goes to next higher sized pool until it succeeds.
 *
 * Return: A void pointer to allocated memory
 */
void *wcnss_prealloc_get(size_t size)
{

	void *mem = NULL;
	gfp_t gfp_mask = __GFP_ZERO;
	int i;

	if (in_interrupt() || !preemptible() || rcu_preempt_depth())
		gfp_mask |= GFP_ATOMIC;
	else
		gfp_mask |= GFP_KERNEL;

	if (size >= cnss_pool_alloc_threshold()) {

		for (i = 0; i < ARRAY_SIZE(cnss_pools); i++) {
			if (cnss_pools[i].size >= size) {
				mem = mempool_alloc(cnss_pools[i].mp, gfp_mask);
				if (mem) {
					cnss_pool_put_cache_in_mem(mem, cnss_pools[i].cache);
					break;
				}
			}
		}
	}

	if (!mem && size >= cnss_pool_alloc_threshold()) {
		pr_debug("cnss_prealloc: not available for size %zu, flag %x\n",
			 size, gfp_mask);
	}

	return mem;
}
EXPORT_SYMBOL(wcnss_prealloc_get);

/**
 * wcnss_prealloc_put() - Relase allocated memory
 * @mem: Allocated memory
 *
 * Free the memory got by wcnss_prealloc_get() to slab or pool reserve if memory
 * pool doesn't have enough elements.
 *
 * Return: 1 - success
 *         0 - fail
 */
int wcnss_prealloc_put(void *mem)
{
	int i;

	if (!mem)
		return 0;

	i = cnss_pool_get_index(mem);

	if (i >= 0 && i < ARRAY_SIZE(cnss_pools)) {
		mempool_free(mem, cnss_pools[i].mp);
		return 1;
	}

	return 0;
}
EXPORT_SYMBOL(wcnss_prealloc_put);

/* Not implemented. Make use of Linux SLAB features. */
void wcnss_prealloc_check_memory_leak(void) {}
EXPORT_SYMBOL(wcnss_prealloc_check_memory_leak);

/* Not implemented. Make use of Linux SLAB features. */
int wcnss_pre_alloc_reset(void) { return -EOPNOTSUPP; }
EXPORT_SYMBOL(wcnss_pre_alloc_reset);

/**
 * cnss_prealloc_is_valid_dt_node_found - Check if valid device tree node
 *                                        present
 *
 * Valid device tree node means a node with "qcom,wlan" property present
 * and "status" property not disabled.
 *
 * Return: true if valid device tree node found, false if not found
 */
static bool cnss_prealloc_is_valid_dt_node_found(void)
{
	struct device_node *dn = NULL;

	for_each_node_with_property(dn, "qcom,wlan") {
		if (of_device_is_available(dn))
			break;
	}

	if (dn)
		return true;

	return false;
}

static int __init cnss_prealloc_init(void)
{
	if (!cnss_prealloc_is_valid_dt_node_found())
		return -ENODEV;

	return cnss_pool_init();
}

static void __exit cnss_prealloc_exit(void)
{
	cnss_pool_deinit();
}

module_init(cnss_prealloc_init);
module_exit(cnss_prealloc_exit);

