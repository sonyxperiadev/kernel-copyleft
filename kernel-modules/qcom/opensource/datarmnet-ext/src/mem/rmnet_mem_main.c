// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/mm.h>
#include "rmnet_mem_nl.h"
#include "rmnet_mem.h"

#include "rmnet_mem_priv.h"

MODULE_LICENSE("GPL v2");

DEFINE_SPINLOCK(rmnet_mem_lock);

int rmnet_mem_id_gaveup[POOL_LEN];
module_param_array(rmnet_mem_id_gaveup, int, NULL, 0444);
MODULE_PARM_DESC(rmnet_mem_id_gaveup, "gaveup per id");

int max_pool_size[POOL_LEN] = { 0, 0, MAX_POOL_O2, MAX_POOL_O3};
module_param_array(max_pool_size, int, NULL, 0644);
MODULE_PARM_DESC(max_pool_size, "Max Pool size per order");

int static_pool_size[POOL_LEN];
module_param_array(static_pool_size, int, NULL, 0444);
MODULE_PARM_DESC(static_pool_size, "Pool size per order");

int pool_unbound_feature[POOL_LEN] = { 0, 0, 1, 1};
module_param_array(pool_unbound_feature, int, NULL, 0644);
MODULE_PARM_DESC(pool_unbound_featue, "Pool bound gate");

int rmnet_mem_order_requests[POOL_LEN];
module_param_array(rmnet_mem_order_requests, int, NULL, 0444);
MODULE_PARM_DESC(rmnet_mem_order_requests, "Request per order");

int rmnet_mem_id_req[POOL_LEN];
module_param_array(rmnet_mem_id_req, int, NULL, 0444);
MODULE_PARM_DESC(rmnet_mem_id_req, "Request per id");

int rmnet_mem_id_recycled[POOL_LEN];
module_param_array(rmnet_mem_id_recycled, int, NULL, 0444);
MODULE_PARM_DESC(rmnet_mem_id_recycled, "Recycled per id");

int rmnet_mem_stats[RMNET_MEM_STAT_MAX];
module_param_array(rmnet_mem_stats, int, NULL, 0444);
MODULE_PARM_DESC(rmnet_mem_stats, "Rmnet mem stats for modules");

int rmnet_mem_err[ERR_MAX];
module_param_array(rmnet_mem_err, int, NULL, 0444);
MODULE_PARM_DESC(rmnet_mem_err, "Error counting");

unsigned int rmnet_mem_pb_ind_max[POOL_LEN];
module_param_array(rmnet_mem_pb_ind_max, uint, NULL, 0644);
MODULE_PARM_DESC(rmnet_mem_pb_ind_max, "Pool size vote that is active on PB ind");

unsigned target_pool_size[POOL_LEN] = { 0, 0, MID_POOL_O2, MID_POOL_O3};
module_param_array(target_pool_size, uint, NULL, 0444);
MODULE_PARM_DESC(target_pool_size, "Pool size wq will adjust to on run");

static char *verinfo[] = {"2003bae3"};
module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

struct workqueue_struct *mem_wq;
struct delayed_work pool_adjust_work;
int pb_ind_pending;
struct  hrtimer pb_timer;

struct list_head rmnet_mem_pool[POOL_LEN];

struct mem_info {
	struct page *addr;
	struct list_head  mem_head;
	u8 order;
};

void rmnet_mem_page_ref_inc_entry(struct page *page, unsigned id)
{
	page_ref_inc(page);
}
EXPORT_SYMBOL_GPL(rmnet_mem_page_ref_inc_entry);

struct rmnet_mem_notif_s {
	struct raw_notifier_head chain;
	spinlock_t lock;
};

struct rmnet_mem_notif_s rmnet_mem_notifier = {
	.chain = RAW_NOTIFIER_INIT(rmnet_mem_notifier.chain),
	.lock  = __SPIN_LOCK_UNLOCKED(rmnet_mem_notifier.lock),
};
EXPORT_SYMBOL_GPL(rmnet_mem_notifier);

int rmnet_mem_get_pool_size(unsigned order)
{
	if (order >= POOL_LEN) {
		rmnet_mem_err[ERR_GET_ORDER_ERR]++;
		return 0;
	}
	/* Return actual size or configured amount if not grown yet.*/
	return (static_pool_size[order]) ? static_pool_size[order]: target_pool_size[order];
}
EXPORT_SYMBOL_GPL(rmnet_mem_get_pool_size);

int rmnet_mem_mode_notify(unsigned pool_size)
{

	unsigned long flags;

	spin_lock_irqsave(&rmnet_mem_notifier.lock, flags);
	raw_notifier_call_chain(&rmnet_mem_notifier.chain, pool_size, NULL);
	spin_unlock_irqrestore(&rmnet_mem_notifier.lock, flags);
	return NOTIFY_OK;
}

int rmnet_mem_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&rmnet_mem_notifier.lock, flags);
	ret = raw_notifier_chain_register(&rmnet_mem_notifier.chain, nb);
	spin_unlock_irqrestore(&rmnet_mem_notifier.lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(rmnet_mem_register_notifier);

int rmnet_mem_unregister_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&rmnet_mem_notifier.lock, flags);
	ret = raw_notifier_chain_unregister(&rmnet_mem_notifier.chain, nb);
	spin_unlock_irqrestore(&rmnet_mem_notifier.lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(rmnet_mem_unregister_notifier);

/* Malloc by client so rem from to pool */
struct mem_info* rmnet_mem_add_page(struct page *page, u8 pageorder)
{
	struct mem_info *mem_slot;

	mem_slot = kzalloc(sizeof(*mem_slot), GFP_ATOMIC);
	if (!mem_slot) {
		rmnet_mem_err[ERR_MALLOC_FAIL1]++;
		return NULL;
	}

	static_pool_size[pageorder]++;

	mem_slot->order = pageorder;
	mem_slot->addr = (void*)page;
	INIT_LIST_HEAD(&mem_slot->mem_head);

	if (pageorder < POOL_LEN) {
		list_add_rcu(&mem_slot->mem_head, &(rmnet_mem_pool[pageorder]));
	}
	return mem_slot;
}

/* Freed by client so added back to pool */
void rmnet_mem_free_all(void)
{
	unsigned long flags;
	struct mem_info *mem_slot;
	struct list_head *ptr = NULL, *next = NULL;
	int i;

	spin_lock_irqsave(&rmnet_mem_lock, flags);
	for (i = 0; i < POOL_LEN; i++) {
		list_for_each_safe(ptr, next, &rmnet_mem_pool[i]) {
			mem_slot = list_entry(ptr, struct mem_info, mem_head);

			list_del(&mem_slot->mem_head);
			put_page(mem_slot->addr);
			static_pool_size[mem_slot->order]--;

			kfree(mem_slot);
		}
	}
	spin_unlock_irqrestore(&rmnet_mem_lock, flags);
}

/* Freed by client so added back to pool */
struct page* rmnet_mem_get_pages_entry(gfp_t gfp_mask, unsigned int order, int *code, int *pageorder, unsigned id)
{
	unsigned long flags;
	struct mem_info *mem_page;
	struct page *page = NULL;
	int i = 0;
	int j = 0;
	int adding = 0;

	spin_lock_irqsave(&rmnet_mem_lock, flags);
	if (order < POOL_LEN) {
		rmnet_mem_id_req[id]++;
		rmnet_mem_order_requests[order]++;
		/* Check high order for rmnet and lower order for IPA if matching order fails */
		for (j = order; j > 0 && j < POOL_LEN; j++) {
			do {
				mem_page = list_first_entry_or_null(&rmnet_mem_pool[j], struct mem_info, mem_head);
				if (!mem_page) {
					break;
				}
				if (page_ref_count(mem_page->addr) == 1) {
					rmnet_mem_id_recycled[j]++;
					page = mem_page->addr;
					page_ref_inc(mem_page->addr);
					list_rotate_left(&rmnet_mem_pool[j]);
					break;
				}
				list_rotate_left(&rmnet_mem_pool[j]);
				i++;
			} while (i <= 5);
			if (page && pageorder) {
				*pageorder = j;
				break;
			}
			i = 0;
		}
	}
	if (static_pool_size[order] < max_pool_size[order] &&
	    pool_unbound_feature[order]) {
		adding = 1;
	}  else
		spin_unlock_irqrestore(&rmnet_mem_lock, flags);

	if (!page) {
		rmnet_mem_id_gaveup[id]++;
		/* IPA doesn't want retry logic but pool will be empty for lower orders and those
		 * will fail too so that is akin to retry. So just hardcode to not retry for o3 page req
		 */
		if (order < 3) {
			page = __dev_alloc_pages((adding)? GFP_ATOMIC : gfp_mask, order);
			if (page) {
				/* If below unbound limit then add page to static pool*/
				if (adding) {
					rmnet_mem_add_page(page, order);
					page_ref_inc(page);
				}

				if (pageorder) {
					*pageorder = order;
				}
			}
		} else {
			/* Only call get page if we will add page to static pool*/
			if (adding) {
				page = __dev_alloc_pages((adding)? GFP_ATOMIC : gfp_mask, order);
				if (page) {

					rmnet_mem_add_page(page, order);
					page_ref_inc(page);
				}

				if (pageorder) {
					*pageorder = order;
				}
			}
		}
	}
	/* If we had potential to add, this won't occur after we fill up to limit */
	if (adding)
		spin_unlock_irqrestore(&rmnet_mem_lock, flags);

	if (pageorder && code && page) {
		if (*pageorder == order)
			*code = RMNET_MEM_SUCCESS;
		else if (*pageorder > order)
			*code = RMNET_MEM_UPGRADE;
		else if (*pageorder < order)
			*code = RMNET_MEM_DOWNGRADE;
	} else if (pageorder && code) {
		*code = RMNET_MEM_FAIL;
		*pageorder = 0;
	}

	return page;
}
EXPORT_SYMBOL_GPL(rmnet_mem_get_pages_entry);

/* Freed by client so added back to pool */
void rmnet_mem_put_page_entry(struct page *page)
{
	put_page(page);
}
EXPORT_SYMBOL_GPL(rmnet_mem_put_page_entry);

static void mem_update_pool_work(struct work_struct *work)
{
	int i;
	int new_size;

	local_bh_disable();
	for (i = 0; i < POOL_LEN; i++) {
	/* If PB ind is active and max pool has been configured
	 * new pool size is max of the two.
	 */
		new_size = (pb_ind_pending && rmnet_mem_pb_ind_max[i]) ?
			    MAX_VOTE(rmnet_mem_pb_ind_max[i],target_pool_size[i]):
			    target_pool_size[i];

		rmnet_mem_adjust(new_size, i);
	}
	local_bh_enable();

}

/* Freed by client so added back to pool */
void rmnet_mem_adjust(unsigned perm_size, u8 pageorder)
{
	struct list_head *entry, *next;
	struct mem_info *mem_slot;
	int i;
	struct page  *newpage = NULL;
	int adjustment;
	unsigned long flags;

	if (pageorder >= POOL_LEN || perm_size > MAX_STATIC_POOL) {
		rmnet_mem_err[ERR_INV_ARGS]++;
		return;
	}

	adjustment = perm_size - static_pool_size[pageorder];

	if (perm_size == static_pool_size[pageorder])
		return;

	spin_lock_irqsave(&rmnet_mem_lock, flags);

	if (perm_size > static_pool_size[pageorder]) {
		for (i = 0; i < (adjustment); i++) {
			newpage = __dev_alloc_pages(GFP_ATOMIC, pageorder);
			if (!newpage) {
				continue;
			}
			rmnet_mem_add_page(newpage, pageorder);
		}
	} else {
	/*TODO what if shrink comes in when we have allocated all pages, can't shrink currently */
	/* Shrink static pool */
		list_for_each_safe(entry, next, &(rmnet_mem_pool[pageorder])) {
			mem_slot = list_entry(entry, struct mem_info, mem_head);
			/* Freeing temp pool memory Remove from ht and kfree*/
			list_del(&mem_slot->mem_head);
			put_page(mem_slot->addr);
			kfree(mem_slot);
			static_pool_size[pageorder]--;

			if (static_pool_size[pageorder] == perm_size)
				break;
		}
	}
	spin_unlock_irqrestore(&rmnet_mem_lock, flags);
	if (pageorder == POOL_NOTIF) {
		rmnet_mem_mode_notify(perm_size);
	}
}

enum hrtimer_restart rmnet_mem_pb_timer_cb(struct hrtimer *t)
{
	unsigned jiffies;

	pb_ind_pending = 0;
	rmnet_mem_stats[RMNET_MEM_PB_TIMEOUT]++;
	jiffies = msecs_to_jiffies(RAMP_DOWN_DELAY);
	/* Ramping down can be done with a delay. Less urgent.*/
	queue_delayed_work(mem_wq, &pool_adjust_work, jiffies);

	return HRTIMER_NORESTART;
}

void rmnet_mem_pb_ind(void)
{
	/* Only listen to pb idn vote if configured*/
	if (!rmnet_mem_pb_ind_max[POOL_NOTIF]) {
		rmnet_mem_stats[RMNET_MEM_PB_IND_CONFIG_FAIL]++;
		return;
	}

	pb_ind_pending = 1;
	/* Trigger update to change pool size */
	if (hrtimer_active(&pb_timer)) {
		hrtimer_cancel(&pb_timer);
	} else {
		cancel_delayed_work(&pool_adjust_work);
		queue_delayed_work(mem_wq, &pool_adjust_work, 0);
	}
	rmnet_mem_stats[RMNET_MEM_PB_IND]++;
	hrtimer_start(&pb_timer, ns_to_ktime(PB_IND_DUR* NS_IN_MS),
					     HRTIMER_MODE_REL| HRTIMER_MODE_PINNED);
}
EXPORT_SYMBOL_GPL(rmnet_mem_pb_ind);

int __init rmnet_mem_module_init(void)
{
	int rc, i = 0;

	pr_info("%s(): Starting rmnet mem module\n", __func__);
	for (i = 0; i < POOL_LEN; i++) {
		INIT_LIST_HEAD(&(rmnet_mem_pool[i]));
	}

	mem_wq = alloc_workqueue("mem_wq", WQ_HIGHPRI, 0);
	if (!mem_wq) {
		pr_err("%s(): Failed to alloc workqueue \n", __func__);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&pool_adjust_work, mem_update_pool_work);

	rc = rmnet_mem_nl_register();
	if (rc) {
		pr_err("%s(): Failed to register generic netlink family\n", __func__);
		destroy_workqueue(mem_wq);
		mem_wq = NULL;
		return -ENOMEM;
	}

	hrtimer_init(&pb_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pb_timer.function = rmnet_mem_pb_timer_cb;

	return 0;
}

void __exit rmnet_mem_module_exit(void)
{
	rmnet_mem_nl_unregister();

	if (mem_wq) {
		cancel_delayed_work_sync(&pool_adjust_work);
		drain_workqueue(mem_wq);
		destroy_workqueue(mem_wq);
		mem_wq = NULL;
	}
	rmnet_mem_free_all();
}
module_init(rmnet_mem_module_init);
module_exit(rmnet_mem_module_exit);
