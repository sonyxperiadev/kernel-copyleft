/*
 * Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTE: This file has been modified by Sony Mobile Communications AB.
 * Modifications are licensed under the License.
 */

#ifndef SSR_MONITOR_H
#define SSR_MONITOR_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/wait.h>
#include <mach/msm_smd.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>

#define SSR_MONITOR_IOCTL_WAITSSR 0


struct ssr_monitor_dev {
	int initialized;
	struct workqueue_struct *ssr_monitor_wait_wq;
	struct work_struct ssr_monitor_work;
	wait_queue_head_t ssr_monitor_wait_q;
	int readyet;
	int data_ready;
	int minor;
	int major;
	int num;
	char name[50];
	struct cdev *cdev;
	struct class *ssrmonitor_class;
};

extern int ssr_monitor_notify(char *name);
extern void ssr_monitor_store_crashreason(char *reason);

#endif /*SSR_MONITOR_H */


