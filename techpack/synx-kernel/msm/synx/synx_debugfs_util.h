/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SYNX_DEBUGFS_UTIL_H__
#define __SYNX_DEBUGFS_UTIL_H__

#include "synx_api.h"
#include "synx_private.h"

#define GLOBAL_HANDLE_STARTING_ID (1048577)

/* DMA FENCE print function */
void synx_debugfs_util_print_dma_fence(char **cur, char **end);

/* CSL FENCE print function */
void synx_debugfs_util_print_csl_fence(char **cur, char **end);

/* GLOBAL & LOCAL MAP print function */
void synx_debugfs_util_print_hash_table(char **cur, char **end, bool flag);

/* GLOBAL SHARED MEMORY print function */
void synx_debugfs_util_print_global_shared_memory(char **cur, char **end);

/* CLIENT MAP print function */
void synx_debugfs_util_print_client_table(char **cur, char **end);

/* Function to get SYNX State Name */
char *synx_debugfs_util_get_state_name(u32 status);

/* Function for loading content of the help option for debugfs v2 */
void synx_debugfs_util_load_help_content(char **cur, char **end);

/* Function to check entry of the global shared memory is valid or not */
bool synx_debugfs_util_is_valid_global_shared_memory_entry(struct synx_global_coredata *entry,
u32 idx);

#endif /* __SYNX_DEBUGFS_UTIL_H__ */
