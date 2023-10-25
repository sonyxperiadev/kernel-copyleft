/*
 * Copyright 2022 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Sony Corporation.
 *
 * Author: Keita Aihara <keita.aihara@sony.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <trace/events/block.h>
#include <trace/hooks/mmc.h>

#define MMC_LOG_PROBES_TAG "mmc_log_probes"
#define mmc_log_probes_pr_err(fmt, args...)	pr_err(MMC_LOG_PROBES_TAG ": " fmt, ##args)

#define BAD_SECTOR_LOG_INTERVAL	3000

static struct kobject *mmc_log_kobj;

static int mmc_recovery_failed;

struct bad_sector_stat {
	char *disk_name;
	unsigned int read;
	unsigned int write;
	unsigned int others;
	ktime_t start_ktime;
};

static struct bad_sector_stat stat = {
	.disk_name = "mmcblk0",
	.read = 0,
	.write = 0,
	.others = 0,
	.start_ktime = 0,
};

static ssize_t bad_sector_log_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "disk %s, read %u, write %u, others %u\n",
			stat.disk_name, stat.read, stat.write, stat.others);
}

static ssize_t mmc_recovery_failed_show(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", mmc_recovery_failed);
}

static struct kobj_attribute bad_sector_log_attr =
	__ATTR(bad_sector_log, 0440, bad_sector_log_show, NULL);
static struct kobj_attribute mmc_recovery_failed_attr =
	__ATTR(mmc_recovery_failed, 0440, mmc_recovery_failed_show, NULL);

static struct attribute *mmc_log_attrs[] = {
	&bad_sector_log_attr.attr,
	&mmc_recovery_failed_attr.attr,
	NULL,
};

static struct attribute_group mmc_log_attr_group = {
	.attrs = mmc_log_attrs,
};

static void add_bad_sector_stat(char *disk_name, unsigned int op)
{
	if (strncmp(disk_name, stat.disk_name, strlen(disk_name)+1) == 0) {
		if (!stat.start_ktime)
			stat.start_ktime = ktime_get();

		switch (op) {
		case 0x00:
			stat.read++;
			break;
		case 0x01:
			stat.write++;
			break;
		default:
			stat.others++;
		}
	}
}

static void probe_block_rq_complete(void *ignore,
				    struct request *req, int error, unsigned int nr_bytes)
{
	ktime_t curr_ktime;
	s64 elapsed_ms;

	if (!req->bio || !req->rq_disk)
		return;

	if (unlikely(error && !blk_rq_is_passthrough(req) && !(req->rq_flags & RQF_QUIET)))
		add_bad_sector_stat(req->rq_disk->disk_name, req_op(req));

	if (!stat.start_ktime)
		return;

	curr_ktime = ktime_get();
	elapsed_ms = ktime_ms_delta(curr_ktime, stat.start_ktime);
	if (elapsed_ms > BAD_SECTOR_LOG_INTERVAL) {
		sysfs_notify(mmc_log_kobj, NULL, bad_sector_log_attr.attr.name);
		stat.start_ktime = 0;
	}
}

static void probe_android_vh_mmc_blk_mq_rw_recovery(void *ignore, struct mmc_card *card)
{
	mmc_recovery_failed = 1;
	sysfs_notify(mmc_log_kobj, NULL, mmc_recovery_failed_attr.attr.name);
}

static int __init mmc_log_probes_init(void)
{
	int ret;

	mmc_log_kobj = kobject_create_and_add("mmc_log", kernel_kobj);
	if (!mmc_log_kobj) {
		mmc_log_probes_pr_err("Failed to create mmc_log sysfs entry\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(mmc_log_kobj, &mmc_log_attr_group);
	if (ret) {
		mmc_log_probes_pr_err("Failed to create mmc_log sysfs attributes\n");
		kobject_put(mmc_log_kobj);
		return ret;
	}

	ret = register_trace_block_rq_complete(probe_block_rq_complete, NULL);
	if (ret) {
		mmc_log_probes_pr_err("Failed to register block_rq_complete hooks\n");
		kobject_put(mmc_log_kobj);
		return ret;
	}

	ret = register_trace_android_vh_mmc_blk_mq_rw_recovery(
		probe_android_vh_mmc_blk_mq_rw_recovery, NULL);
	if (ret) {
		mmc_log_probes_pr_err("Failed to register mmc_blk_mq_rw_recovery hooks\n");
		unregister_trace_block_rq_complete(probe_block_rq_complete, NULL);
		kobject_put(mmc_log_kobj);
		return ret;
	}

	return 0;
}

static void __exit mmc_log_probes_exit(void)
{
	unregister_trace_block_rq_complete(probe_block_rq_complete, NULL);
	unregister_trace_android_vh_mmc_blk_mq_rw_recovery(probe_android_vh_mmc_blk_mq_rw_recovery,
		NULL);
	kobject_put(mmc_log_kobj);
}

module_init(mmc_log_probes_init);
module_exit(mmc_log_probes_exit);

MODULE_LICENSE("GPL v2");
