/*
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are Copyright (c) 2017 Sony Mobile Communications Inc,
 * and licensed under the license of the file.
 */
// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/staging/android/ion/ion_mem_pool.c
 *
 * Copyright (C) 2011 Google, Inc.
 */

#include <linux/list.h>
#include <linux/migrate.h>
#include <linux/mount.h>
#include <linux/init.h>
#include <linux/page-flags.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/sched/signal.h>
#include <linux/compaction.h>

#include "ion.h"

#define ION_PAGE_CACHE	1

/* do a simple check to see if we are in any low memory situation */
static bool pool_refill_ok(struct ion_page_pool *pool)
{
	struct zonelist *zonelist;
	struct zoneref *z;
	struct zone *zone;
	int mark;
	enum zone_type classzone_idx = gfp_zone(pool->gfp_mask);
	s64 delta;

	/* check if we are within the refill defer window */
	delta = ktime_ms_delta(ktime_get(), pool->last_low_watermark_ktime);
	if (delta < ION_POOL_REFILL_DEFER_WINDOW_MS)
		return false;

	zonelist = node_zonelist(numa_node_id(), pool->gfp_mask);
	/*
	 * make sure that if we allocate a pool->order page from buddy,
	 * we don't put the zone watermarks go below the high threshold.
	 * This makes sure there's no unwanted repetitive refilling and
	 * reclaiming of buddy pages on the pool.
	 */
	for_each_zone_zonelist(zone, z, zonelist, classzone_idx) {
		mark = high_wmark_pages(zone);
		mark += 1 << pool->order;
		if (!zone_watermark_ok_safe(zone, pool->order, mark,
					    classzone_idx)) {
			pool->last_low_watermark_ktime = ktime_get();
			return false;
		}
	}

	return true;
}

static inline struct page *ion_page_pool_alloc_pages(struct ion_page_pool *pool)
{
	if (fatal_signal_pending(current))
		return NULL;
	return alloc_pages(pool->gfp_mask, pool->order);
}

static void ion_page_pool_free_pages(struct ion_page_pool *pool,
				     struct page *page)
{
	if (pool->inode && pool->order == 0) {
		lock_page(page);
		__ClearPageMovable(page);
		unlock_page(page);
	}
	__free_pages(page, pool->order);
}

static void ion_page_pool_add(struct ion_page_pool *pool, struct page *page)
{
	mutex_lock(&pool->mutex);
	page->private = ION_PAGE_CACHE;
	if (PageHighMem(page)) {
		list_add_tail(&page->lru, &pool->high_items);
		pool->high_count++;
	} else {
		list_add_tail(&page->lru, &pool->low_items);
		pool->low_count++;
	}

	atomic_inc(&pool->count);
	mod_node_page_state(page_pgdat(page), NR_INDIRECTLY_RECLAIMABLE_BYTES,
			    (1 << (PAGE_SHIFT + pool->order)));

	if (pool->inode && pool->order == 0)
		__SetPageMovable(page, pool->inode->i_mapping);
	mutex_unlock(&pool->mutex);
}

void ion_page_pool_refill(struct ion_page_pool *pool)
{
	struct page *page;
	gfp_t gfp_refill = (pool->gfp_mask | __GFP_RECLAIM) & ~__GFP_NORETRY;
	struct device *dev = pool->dev;

	/* skip refilling order 0 pools */
	if (!pool->order)
		return;

	while (!pool_fillmark_reached(pool) && pool_refill_ok(pool)) {
		page = alloc_pages(gfp_refill, pool->order);
		if (!page)
			break;
		if (!pool->cached)
			ion_pages_sync_for_device(dev, page,
						  PAGE_SIZE << pool->order,
						  DMA_BIDIRECTIONAL);
		ion_page_pool_add(pool, page);
	}
}

static struct page *ion_page_pool_remove(struct ion_page_pool *pool, bool high)
{
	struct page *page;

	if (high) {
		BUG_ON(!pool->high_count);
		page = list_first_entry(&pool->high_items, struct page, lru);
		pool->high_count--;
	} else {
		BUG_ON(!pool->low_count);
		page = list_first_entry(&pool->low_items, struct page, lru);
		pool->low_count--;
	}

	atomic_dec(&pool->count);
	mod_node_page_state(page_pgdat(page), NR_INDIRECTLY_RECLAIMABLE_BYTES,
			    -(1 << (PAGE_SHIFT + pool->order)));
	clear_bit(ION_PAGE_CACHE, &page->private);

	list_del_init(&page->lru);
	return page;
}

struct page *ion_page_pool_alloc(struct ion_page_pool *pool, bool *from_pool)
{
	struct page *page = NULL;

	BUG_ON(!pool);

	if (fatal_signal_pending(current))
		return ERR_PTR(-EINTR);

	if (*from_pool && mutex_trylock_spin(&pool->mutex)) {
		if (pool->high_count)
			page = ion_page_pool_remove(pool, true);
		else if (pool->low_count)
			page = ion_page_pool_remove(pool, false);
		mutex_unlock(&pool->mutex);
	}
	if (!page) {
		page = ion_page_pool_alloc_pages(pool);
		*from_pool = false;
	} else {
		lock_page(page);
		__ClearPageMovable(page);
		unlock_page(page);
	}

	if (!page)
		return ERR_PTR(-ENOMEM);
	return page;
}

/*
 * Tries to allocate from only the specified Pool and returns NULL otherwise
 */
struct page *ion_page_pool_alloc_pool_only(struct ion_page_pool *pool)
{
	struct page *page = NULL;

	if (!pool)
		return ERR_PTR(-EINVAL);

	if (mutex_trylock_spin(&pool->mutex)) {
		if (pool->high_count)
			page = ion_page_pool_remove(pool, true);
		else if (pool->low_count)
			page = ion_page_pool_remove(pool, false);
		mutex_unlock(&pool->mutex);
	}

	if (!page)
		return ERR_PTR(-ENOMEM);

	lock_page(page);
	__ClearPageMovable(page);
	unlock_page(page);
	return page;
}

void ion_page_pool_free(struct ion_page_pool *pool, struct page *page)
{
	ion_page_pool_add(pool, page);
}

void ion_page_pool_free_immediate(struct ion_page_pool *pool, struct page *page)
{
	ion_page_pool_free_pages(pool, page);
}

int ion_page_pool_total(struct ion_page_pool *pool, bool high)
{
	int count = pool->low_count;

	if (high)
		count += pool->high_count;

	return count << pool->order;
}

int ion_page_pool_shrink(struct ion_page_pool *pool, gfp_t gfp_mask,
			 int nr_to_scan)
{
	int freed = 0;
	bool high;

	if (current_is_kswapd())
		high = true;
	else
		high = !!(gfp_mask & __GFP_HIGHMEM);

	if (nr_to_scan == 0)
		return ion_page_pool_total(pool, high);

	while (freed < nr_to_scan) {
		struct page *page;

		mutex_lock(&pool->mutex);
		if (pool->low_count) {
			page = ion_page_pool_remove(pool, false);
		} else if (high && pool->high_count) {
			page = ion_page_pool_remove(pool, true);
		} else {
			mutex_unlock(&pool->mutex);
			break;
		}
		mutex_unlock(&pool->mutex);
		ion_page_pool_free_pages(pool, page);
		freed += (1 << pool->order);
	}

	return freed;
}

static bool ion_page_pool_isolate(struct page *page, isolate_mode_t mode)
{
	struct ion_page_pool *pool;
	struct address_space *mapping = page_mapping(page);

	VM_BUG_ON_PAGE(PageIsolated(page), page);

	if (!mapping)
		return false;
	pool = mapping->private_data;

	mutex_lock(&pool->mutex);
	/* could be removed from the cache pool and thus become unmovable */
	if (!__PageMovable(page)) {
		mutex_unlock(&pool->mutex);
		return false;
	}

	if (unlikely(!test_bit(ION_PAGE_CACHE, &page->private))) {
		mutex_unlock(&pool->mutex);
		return false;
	}

	list_del(&page->lru);
	if (PageHighMem(page))
		pool->high_count--;
	else
		pool->low_count--;
	mutex_unlock(&pool->mutex);

	return true;
}

static int ion_page_pool_migrate(struct address_space *mapping,
				 struct page *newpage,
				 struct page *page, enum migrate_mode mode)
{
	struct ion_page_pool *pool = mapping->private_data;

	VM_BUG_ON_PAGE(!PageMovable(page), page);
	VM_BUG_ON_PAGE(!PageIsolated(page), page);

	lock_page(page);
	newpage->private = ION_PAGE_CACHE;
	__SetPageMovable(newpage, page_mapping(page));
	get_page(newpage);
	__ClearPageMovable(page);
	ClearPagePrivate(page);
	unlock_page(page);
	mutex_lock(&pool->mutex);
	if (PageHighMem(newpage)) {
		list_add_tail(&newpage->lru, &pool->high_items);
		pool->high_count++;
	} else {
		list_add_tail(&newpage->lru, &pool->low_items);
		pool->low_count++;
	}
	mutex_unlock(&pool->mutex);
	put_page(page);
	return 0;
}

static void ion_page_pool_putback(struct page *page)
{
	/*
	 * migrate function either succeeds or returns -EAGAIN, which
	 * results in calling it again until it succeeds, sothis callback
	 * is not needed.
	 */
}

static struct dentry *ion_pool_do_mount(struct file_system_type *fs_type,
				int flags, const char *dev_name, void *data)
{
	static const struct dentry_operations ops = {
		.d_dname = simple_dname,
	};

	return mount_pseudo(fs_type, "ion_pool:", NULL, &ops, 0x77);
}

static struct file_system_type ion_pool_fs = {
	.name		= "ion_pool",
	.mount		= ion_pool_do_mount,
	.kill_sb	= kill_anon_super,
};

static int ion_pool_cnt;
static struct vfsmount *ion_pool_mnt;
static int ion_pool_mount(void)
{
	int ret = 0;

	ion_pool_mnt = kern_mount(&ion_pool_fs);
	if (IS_ERR(ion_pool_mnt))
		ret = PTR_ERR(ion_pool_mnt);

	return ret;
}

static const struct address_space_operations ion_pool_aops = {
	.isolate_page = ion_page_pool_isolate,
	.migratepage = ion_page_pool_migrate,
	.putback_page = ion_page_pool_putback,
};

static int ion_pool_register_migration(struct ion_page_pool *pool)
{
	int  ret = simple_pin_fs(&ion_pool_fs, &ion_pool_mnt, &ion_pool_cnt);

	if (ret < 0) {
		pr_err("Cannot mount pseudo fs: %d\n", ret);
		return ret;
	}
	pool->inode = alloc_anon_inode(ion_pool_mnt->mnt_sb);
	if (IS_ERR(pool->inode)) {
		ret = PTR_ERR(pool->inode);
		pool->inode = NULL;
		simple_release_fs(&ion_pool_mnt, &ion_pool_cnt);
		return ret;
	}

	pool->inode->i_mapping->private_data = pool;
	pool->inode->i_mapping->a_ops = &ion_pool_aops;
	return 0;
}

static void ion_pool_unregister_migration(struct ion_page_pool *pool)
{
	if (pool->inode) {
		iput(pool->inode);
		pool->inode = NULL;
		simple_release_fs(&ion_pool_mnt, &ion_pool_cnt);
	}
}

struct ion_page_pool *ion_page_pool_create(gfp_t gfp_mask, unsigned int order,
					   bool cached, bool movable)
{
	struct ion_page_pool *pool = kzalloc(sizeof(*pool), GFP_KERNEL);

	if (!pool)
		return NULL;
	INIT_LIST_HEAD(&pool->low_items);
	INIT_LIST_HEAD(&pool->high_items);
	pool->gfp_mask = gfp_mask;
	pool->order = order;
	mutex_init(&pool->mutex);
	plist_node_init(&pool->list, order);
	if (cached)
		pool->cached = true;

	pool->inode = NULL;
	if (movable)
		ion_pool_register_migration(pool);

	return pool;
}

void ion_page_pool_destroy(struct ion_page_pool *pool)
{
	ion_pool_unregister_migration(pool);
	kfree(pool);
}

static int __init ion_page_pool_init(void)
{
	return ion_pool_mount();
}
device_initcall(ion_page_pool_init);
