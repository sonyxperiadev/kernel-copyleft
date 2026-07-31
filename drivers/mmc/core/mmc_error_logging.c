// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 Sony Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <trace/events/block.h>

#include <uapi/fels/fels_common_types.h>
#include <fels.h>

#define MMC_ERROR_LOGGING_TAG "mmc_error_logging"
#define mmc_error_logging_pr_err(fmt, args...)	pr_err(MMC_ERROR_LOGGING_TAG ": " fmt, ##args)

#define MMC_ERROR_LOGGING_INTERVAL	3000

static struct kobject *mmc_error_logging_kobj;

struct bad_sector_stat {
	char *disk_name;
	unsigned int read;
	unsigned int write;
	unsigned int others;
	ktime_t prev_ktime;
	struct kobject *kobj;
	struct kobj_attribute attr;
};

static struct bad_sector_stat stats[2] = {
	{"mmcblk0", 0, 0, 0, 0, NULL, {{0}}},
	{"mmcblk1", 0, 0, 0, 0, NULL, {{0}}}, };

static ssize_t bad_sector_log_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct bad_sector_stat *stat = container_of(attr, struct bad_sector_stat, attr);
	return sysfs_emit(buf, "disk %s, read %u, write %u, others %u\n",
			stat->disk_name, stat->read, stat->write, stat->others);
}

static void update_stat(struct bad_sector_stat *stat, unsigned int op)
{
	switch (op) {
	case REQ_OP_READ:
		stat->read++;
		break;
	case REQ_OP_WRITE:
		stat->write++;
		break;
	default:
		stat->others++;
		break;
	}
}

static void probe_block_rq_complete(void *ignore,
				    struct request *req, blk_status_t error, unsigned int nr_bytes)
{
	int i;
	ktime_t curr_ktime;
	s64 elapsed_ms;

	if (!req->bio || !req->q->disk)
		return;

	if (likely(!error || blk_rq_is_passthrough(req) ||
		   (req->rq_flags & RQF_QUIET) ||
		   test_bit(GD_DEAD, &req->q->disk->state))) {
		return;
	}

	curr_ktime = ktime_get();
	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		if (strncmp(req->q->disk->disk_name, stats[i].disk_name,
			    strlen(req->q->disk->disk_name) + 1) == 0) {
			update_stat(&stats[i], req_op(req));
			elapsed_ms = ktime_ms_delta(curr_ktime, stats[i].prev_ktime);
			if (elapsed_ms > MMC_ERROR_LOGGING_INTERVAL) {
				fels_log(FELS_CATEGORY_LINUX_OS, FELS_LOG_LEVEL_WARNING,
					 FELS_ERROR_CODE_LINUX_OS_BAD_SECTOR,
					 stats[i].read, stats[i].write, stats[i].others, 0);
				stats[i].prev_ktime = curr_ktime;
				if (stats[i].kobj)
					sysfs_notify(stats[i].kobj, NULL, stats[i].attr.attr.name);
			}
		}
	}
}

static int __init mmc_error_logging_init(void)
{
	int i;
	int ret;
	ktime_t curr_ktime = ktime_get();

	for (i = 0; i < ARRAY_SIZE(stats); i++)
		stats[i].prev_ktime = curr_ktime;

	mmc_error_logging_kobj = kobject_create_and_add("mmc_error_logging", kernel_kobj);
	if (!mmc_error_logging_kobj) {
		mmc_error_logging_pr_err("Failed to create mmc_error_logging sysfs entry\n");
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		stats[i].kobj = kobject_create_and_add(stats[i].disk_name, mmc_error_logging_kobj);
		if (!stats[i].kobj) {
			mmc_error_logging_pr_err("Failed to create %s sysfs entry\n", stats[i].disk_name);
			ret = -ENOMEM;
			goto error_cleanup;
		}

		stats[i].attr.attr.name = "bad_sector_log";
		stats[i].attr.attr.mode = 0440;
		stats[i].attr.show = bad_sector_log_show;
		stats[i].attr.store = NULL;

		ret = sysfs_create_file(stats[i].kobj, &stats[i].attr.attr);
		if (ret) {
			mmc_error_logging_pr_err("Failed to create %s sysfs attribute\n", stats[i].disk_name);
			kobject_put(stats[i].kobj);
			stats[i].kobj = NULL;
			goto error_cleanup;
		}
	}

	ret = register_trace_block_rq_complete(probe_block_rq_complete, NULL);
	if (ret) {
		mmc_error_logging_pr_err("Failed to register block_rq_complete probe\n");
		goto error_cleanup;
	}

	return 0;

error_cleanup:
	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		if (stats[i].kobj) {
			sysfs_remove_file(stats[i].kobj, &stats[i].attr.attr);
			kobject_put(stats[i].kobj);
			stats[i].kobj = NULL;
		}
	}
	kobject_put(mmc_error_logging_kobj);
	return ret;
}

static void __exit mmc_error_logging_exit(void)
{
	int i;

	unregister_trace_block_rq_complete(probe_block_rq_complete, NULL);

	for (i = 0; i < ARRAY_SIZE(stats); i++) {
		if (stats[i].kobj) {
			sysfs_remove_file(stats[i].kobj, &stats[i].attr.attr);
			kobject_put(stats[i].kobj);
		}
	}
	kobject_put(mmc_error_logging_kobj);
}

module_init(mmc_error_logging_init);
module_exit(mmc_error_logging_exit);

MODULE_LICENSE("GPL");
