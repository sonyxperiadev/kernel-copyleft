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
#include <trace/events/block.h>
#include <trace/hooks/mmc.h>

#include <uapi/fels/fels_common_types.h>
#include <fels.h>

#define MMC_ERROR_LOGGING_TAG "mmc_error_logging"
#define mmc_error_logging_pr_err(fmt, args...)	pr_err(MMC_ERROR_LOGGING_TAG ": " fmt, ##args)

#define MMC_ERROR_LOGGING_INTERVAL	3000

struct bad_sector_stat {
	char *disk_name;
	unsigned int read;
	unsigned int write;
	unsigned int others;
	ktime_t prev_ktime;
};

static struct bad_sector_stat stats[2] = {
	{"mmcblk0", 0, 0, 0, 0},
	{"mmcblk1", 0, 0, 0, 0}, };

static unsigned int recovery_failure_count;

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
			}
		}
	}
}

static void probe_android_vh_mmc_blk_mq_rw_recovery(void *ignore, struct mmc_card *card)
{
	fels_log(FELS_CATEGORY_LINUX_OS, FELS_LOG_LEVEL_ERROR,
		 FELS_ERROR_CODE_LINUX_OS_MMC_RECOVERY_FAILURE,
		 ++recovery_failure_count, 0, 0, 0);
}

static int __init mmc_error_logging_init(void)
{
	int i;
	int ret;
	ktime_t curr_ktime = ktime_get();

	for (i = 0; i < ARRAY_SIZE(stats); i++)
		stats[i].prev_ktime = curr_ktime;

	ret = register_trace_block_rq_complete(probe_block_rq_complete, NULL);
	if (ret) {
		mmc_error_logging_pr_err("Failed to register block_rq_complete probe\n");
		return ret;
	}

	ret = register_trace_android_vh_mmc_blk_mq_rw_recovery(
		probe_android_vh_mmc_blk_mq_rw_recovery, NULL);
	if (ret) {
		mmc_error_logging_pr_err(
			"Failed to register android_vh_mmc_blk_mq_rw_recovery probe\n");
		unregister_trace_block_rq_complete(probe_block_rq_complete, NULL);
		return ret;
	}

	return 0;
}

static void __exit mmc_error_logging_exit(void)
{
	unregister_trace_block_rq_complete(probe_block_rq_complete, NULL);
	unregister_trace_android_vh_mmc_blk_mq_rw_recovery(probe_android_vh_mmc_blk_mq_rw_recovery,
							   NULL);
}

module_init(mmc_error_logging_init);
module_exit(mmc_error_logging_exit);

MODULE_LICENSE("GPL");
