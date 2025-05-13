/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef DATARMNET64d33b9eb9
#define DATARMNET64d33b9eb9
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/mm.h>
#define IPA_ID (0xd26+209-0xdf6)
#define RMNET_CORE_ID (0xd1f+216-0xdf5)
#define POOL_LEN (0xd11+230-0xdf3)
#define MAX_STATIC_POOL (0xc07+1233-0xe1c)
#define MAX_POOL_O3 (0xbb7+1296-0xe24)
#define MAX_POOL_O2 (0xbb7+4453-0x1c3c)
void rmnet_mem_adjust(unsigned perm_size,u8 order);
#define rm_err(DATARMNET6c3cf5865b, ...)  \
	do { if ((0xd2d+202-0xdf7)) pr_err(DATARMNET6c3cf5865b, __VA_ARGS__); } while (\
(0xd2d+202-0xdf7))
extern int max_pool_size[POOL_LEN];extern int static_pool_size[POOL_LEN];extern 
int pool_unbound_feature[POOL_LEN];extern int rmnet_mem_order_requests[POOL_LEN]
;extern int rmnet_mem_id_req[POOL_LEN];extern int rmnet_mem_id_recycled[POOL_LEN
];extern int target_static_pool_size[POOL_LEN];
#endif

