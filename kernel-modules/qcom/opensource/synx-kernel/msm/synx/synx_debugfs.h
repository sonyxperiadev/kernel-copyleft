/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SYNX_DEBUGFS_H__
#define __SYNX_DEBUGFS_H__

#include <linux/debugfs.h>
#include <linux/delay.h>

#include "synx_private.h"
//#define ENABLE_DEBUGFS
#define STATE_NAME_SPACE (4)

enum synx_debug_level {
	SYNX_ERR  = 0x0001,
	SYNX_WARN = 0x0002,
	SYNX_INFO = 0x0004,
	SYNX_DBG  = 0x0008,
	SYNX_VERB = 0x0010,
	SYNX_IPCL = 0x0020,
	SYNX_GSM  = 0x0040,
	SYNX_MEM  = 0x0080,
	SYNX_ALL  = SYNX_ERR | SYNX_WARN | SYNX_INFO |
				SYNX_DBG | SYNX_IPCL | SYNX_GSM  | SYNX_MEM,
};

enum synx_columns_level {
	NAME_COLUMN         = 0x00000001,
	ID_COLUMN           = 0x00000002,
	BOUND_COLUMN        = 0x00000004,
	STATUS_COLUMN       = 0x00000008,
	FENCE_COLUMN        = 0x00000010,
	COREDATA_COLUMN     = 0x00000020,
	GLOBAL_IDX_COLUMN   = 0x00000040,
	REL_CNT_COLUMN      = 0x00000080,
	MAP_CNT_COLUMN      = 0x00000100,
	REF_CNT_COLUMN      = 0x00000200,
	NUM_CHILD_COLUMN    = 0x00000400,
	SUBSCRIBERS_COLUMN  = 0x00000800,
	WAITERS_COLUMN      = 0x00001000,
	PARENTS_COLUMN      = 0x00002000,
	CLIENT_ID_COLUMN    = 0x00004000,

	LOCAL_HASHTABLE     = 0x00010000,
	GLOBAL_HASHTABLE    = 0x00020000,
	CLIENT_HASHTABLE    = 0x00040000,
	GLOBAL_SHARED_MEM   = 0x00080000,
	DMA_FENCE_MAP       = 0x00100000,
	CSL_FENCE_MAP       = 0x00200000,

	ERROR_CODES         = 0x00008000,
};

#ifndef SYNX_DBG_LABEL
#define SYNX_DBG_LABEL "synx"
#endif

#define SYNX_DBG_TAG SYNX_DBG_LABEL ": %4s: "

extern int synx_debug;
extern u32 lower_handle_id, upper_handle_id;
extern long synx_columns;

static inline char *synx_debug_str(int level)
{
	switch (level) {
	case SYNX_ERR:
		return "err";
	case SYNX_WARN:
		return "warn";
	case SYNX_INFO:
		return "info";
	case SYNX_DBG:
		return "dbg";
	case SYNX_VERB:
		return "verb";
	case SYNX_IPCL:
		return "ipcl";
	case SYNX_GSM:
		return "gmem";
	case SYNX_MEM:
		return "mem";
	default:
		return "???";
	}
}

#define dprintk(__level, __fmt, arg...)                 \
	do {                                                \
		if (synx_debug & __level) {                     \
			pr_info(SYNX_DBG_TAG "%s: %d: "  __fmt,     \
				synx_debug_str(__level), __func__,      \
				__LINE__, ## arg);                      \
		}                                               \
	} while (0)

#define SYNX_CONSOLE_LOG(__cur, __end,                  \
		__fmt_string, arg...)                           \
	do {                                                \
		if ((__end - __cur) * (sizeof(char *))          \
			- strlen(__fmt_string) <= STATE_NAME_SPACE) \
			dprintk(SYNX_DBG, __fmt_string, ## arg);    \
		else                                            \
			__cur += scnprintf(__cur, __end - __cur,    \
			__fmt_string, ## arg);                      \
	} while (0)

#define SYNX_READ_CHAR(__buf, __num,                    \
		__base, __pos)                                  \
	do {                                                \
		if (__buf[__pos]  >= '0' &&                     \
		__buf[__pos] <= '9')                            \
			__num = __num * __base +                    \
			(__buf[__pos] - '0');                       \
		else if (__buf[__pos] >= 'a' &&                 \
		__buf[__pos] <= 'f')                            \
			__num = __num * __base +                    \
			(__buf[__pos] - 'a' + 10);                  \
		else if (__buf[__pos] >= 'A' &&                 \
		__buf[__pos] <= 'F')                            \
			__num = __num * __base +                    \
			(__buf[__pos] - 'A' + 10);                  \
		else                                            \
			invalid_val = true;                         \
	} while (0)

/**
 * synx_init_debugfs_dir - Initializes debugfs
 *
 * @param dev : Pointer to synx device structure
 */
struct dentry *synx_init_debugfs_dir(struct synx_device *dev);

/**
 * synx_remove_debugfs_dir - Removes debugfs
 *
 * @param dev : Pointer to synx device structure
 */
void synx_remove_debugfs_dir(struct synx_device *dev);

#endif /* __SYNX_DEBUGFS_H__ */
