/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#ifndef _LAST_LOGS_H
#define _LAST_LOGS_H

#ifdef CONFIG_LAST_LOGS

#ifdef CONFIG_MSM_TZ_LOG
#include <soc/qcom/last_logs_tz.h>
#endif /* CONFIG_MSM_TZ_LOG */

#define MAX_LAST_LOGS_REGIONS 16
#define LAST_LOGS_VERSION 0x00010000
#define LAST_LOGS_MAGIC 0x442474D4
#define MAX_NAME_LENGTH 16

typedef struct __packed {
	char name[MAX_NAME_LENGTH];
	uint32_t offset;
	uint32_t size;
	uint32_t reserved;
} last_logs_region;

typedef struct __packed {
	uint32_t version;
	uint32_t magic;
	uint32_t num_regions;
	uint32_t reserved[3];
	last_logs_region regions[MAX_LAST_LOGS_REGIONS];
} last_logs_header;

struct last_logs_data {
	void *addr;
	uint32_t size;
	struct dentry *debugfs_file;
};

#endif /* CONFIG_LAST_LOGS */
#endif /* _LAST_LOGS_H */
