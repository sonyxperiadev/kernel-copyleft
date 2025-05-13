/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _RMNET_MEM_H_
#define _RMNET_MEM_H_
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/mm.h>
#define IPA_ID (0xd26+209-0xdf6)
#define RMNET_CORE_ID (0xd1f+216-0xdf5)
#define POOL_NOTIF (0xd18+223-0xdf4)
#define RMNET_MEM_SUCCESS (0xd2d+202-0xdf7)
#define RMNET_MEM_FAIL -(0xd26+209-0xdf6)
#define RMNET_MEM_DOWNGRADE -(0xd1f+216-0xdf5)
#define RMNET_MEM_UPGRADE -(0xd18+223-0xdf4)
int rmnet_mem_unregister_notifier(struct notifier_block*nb);int 
rmnet_mem_register_notifier(struct notifier_block*nb);extern struct 
rmnet_mem_notif_s rmnet_mem_notifier;void rmnet_mem_put_page_entry(struct page*
page);void rmnet_mem_page_ref_inc_entry(struct page*page,unsigned id);struct 
page*rmnet_mem_get_pages_entry(gfp_t gfp_mask,unsigned int order,int*code,int*
pageorder,unsigned id);
#endif

