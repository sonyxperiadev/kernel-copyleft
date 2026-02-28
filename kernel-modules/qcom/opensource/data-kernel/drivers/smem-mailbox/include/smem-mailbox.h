/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SMEM_MAILBOX_H__

#include <linux/types.h>

/* Flags for Write API */
#define FLAG_URGENT 1

typedef void (*smem_mailbox_urgent_cb)(u16 pending_bytes);

/* API's Exposed to External Modules */
int smem_mailbox_start(int id, smem_mailbox_urgent_cb urgent_cb);
int smem_mailbox_stop(int id);
int smem_mailbox_read(int id, u8 **data, u16 *data_length, unsigned long long *xo_time);
int smem_mailbox_write(int id, int flags, __u8 *data, u16 data_length);

#endif