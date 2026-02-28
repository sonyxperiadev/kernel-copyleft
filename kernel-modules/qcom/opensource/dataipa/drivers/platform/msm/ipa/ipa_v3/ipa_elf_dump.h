/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/devcoredump.h>
#include <linux/elf.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/reboot.h>
#include <linux/rwsem.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/thermal.h>
#include <linux/version.h>
#include <linux/kmsg_dump.h>
#include "ipa_i.h"

#define MAX_ELF_REGIONS 150
#define IPA_RAMDUMP_MAGIC 0x495041 /* IPA in ASCII */
#define IPA_RAMDUMP_VERSION 0
#define TIMEOUT_SAVE_DUMP_MS 30000
#define IPC_LOG_MAX_CONTEXT_NAME_LEN 32

#define MAX_NUM_OF_SS           10
#define MAX_REGION_NAME_LENGTH  16
#define SBL_MINIDUMP_SMEM_ID	602
#define MD_REGION_VALID		('V' << 24 | 'A' << 16 | 'L' << 8 | 'I' << 0)
#define MD_SS_ENCR_DONE		('D' << 24 | 'O' << 16 | 'N' << 8 | 'E' << 0)
#define MD_SS_ENABLED		('E' << 24 | 'N' << 16 | 'B' << 8 | 'L' << 0)
#define IPA_ELF_PAGE_SIZE 4096
#define IPA_ELF_CHUNK_SIZE (4 * IPA_ELF_PAGE_SIZE)
#define DMESG_BUF_CHUNKS 8
#define IPC_BUF_CHUNKS 14
#define DMESG_BUF_SIZE (DMESG_BUF_CHUNKS * IPA_ELF_CHUNK_SIZE)
#define IPC_BUF_SIZE (IPC_BUF_CHUNKS * IPA_ELF_CHUNK_SIZE)


/* ELF_SSR structures necessary for constructing the elf */

struct elf_ssr_driver_dump_entry {
	char region_name[100];
	void *buffer_pointer;
	size_t buffer_size;
	uint8_t entry_num;
};

struct ipa_qcom_dump_segment {
	struct list_head node;
	dma_addr_t da;
	void *va;
	size_t size;
};

struct ipa_qcom_ramdump_desc {
	void *data;
	struct completion dump_done;
};

struct ipa_dump_entry {
	int type;
	u32 entry_start;
	u32 entry_num;
};

struct ipa_host_dump_meta_info {
	u32 magic;
	u32 version;
	u32 chipset;
	u32 total_entries;
	struct ipa_dump_entry entry[MAX_ELF_REGIONS];
};


enum ipa_host_dump_type {
	IPA_HOST_DUMP_IPA_CTX = 0,
	IPA_HOST_DUMP_GSI_CTX = 1,
	IPA_HOST_DUMP_IPC_LOGS = 2,
	IPA_HOST_DUMP_DMESG_LOGS = 3,
	IPA_HOST_DUMP_MAX = 4
};

/** Structures pertaining to ipc logs, and to make the SMC call
 * figure out if minidump enabled or not
 */

/* Structs storing ipc logs */
struct ipc_log_page_header {
	uint32_t magic;
	uint32_t nmagic;
	uint32_t page_num;
	uint16_t read_offset;
	uint16_t write_offset;
	uint64_t log_id;
	uint64_t start_time;
	uint64_t end_time;
	int64_t ctx_offset;

	/* add local data structures after this point */
	struct list_head list;
	uint16_t nd_read_offset;
};

/**
 * struct ipc_log_page - Individual log page
 *
 * @hdr: Log page header
 * @data: Log data
 *
 * Each log consists of 1 to N log pages.  Data size is adjusted to always fit
 * the structure into a single kernel page.
 */
struct ipc_log_page {
	struct ipc_log_page_header hdr;
	char data[PAGE_SIZE - sizeof(struct ipc_log_page_header)];
};

/**
 * struct ipc_log_context - main logging context
 *
 * @magic:  Magic number (used for log extraction)
 * @nmagic:  Inverse of magic number (used for log extraction)
 * @version:  IPC Logging version of log format
 * @user_version:  Version number for user-defined messages
 * @header_size:  Size of the log header which is used to determine the offset
 *                of ipc_log_page::data
 * @log_id:  Log ID (assigned when log is created)
 * @name:  Name of the log used to uniquely identify the log during extraction
 *
 * @list:  List of log contexts (struct ipc_log_context)
 * @page_list:  List of log pages (struct ipc_log_page)
 * @first_page:  First page in list of logging pages
 * @last_page:  Last page in list of logging pages
 * @write_page:  Current write page
 * @read_page:  Current read page (for internal reads)
 * @nd_read_page:  Current debugfs extraction page (non-destructive)
 *
 * @write_avail:  Number of bytes available to write in all pages
 * @dent:  Debugfs node for run-time log extraction
 * @dfunc_info_list:  List of deserialization functions
 * @context_lock_lhb1:  Lock for entire structure
 * @read_avail:  Completed when new data is added to the log
 */
struct ipc_log_context {
	uint32_t magic;
	uint32_t nmagic;
	uint32_t version;
	uint16_t user_version;
	uint16_t header_size;
	uint64_t log_id;
	char name[IPC_LOG_MAX_CONTEXT_NAME_LEN];

	/* add local data structures after this point */
	struct list_head list;
	struct list_head page_list;
	struct ipc_log_page *first_page;
	struct ipc_log_page *last_page;
	struct ipc_log_page *write_page;
	struct ipc_log_page *read_page;
	struct ipc_log_page *nd_read_page;

	uint32_t write_avail;
	struct dentry *dent;
	struct list_head dfunc_info_list;
	spinlock_t context_lock_lhb1;
	struct completion read_avail;
	struct kref refcount;
	bool destroyed;
};


/**
 * struct minidump_region - Minidump region
 * @name		: Name of the region to be dumped
 * @seq_num:		: Use to differentiate regions with same name.
 * @valid		: This entry to be dumped (if set to 1)
 * @address		: Physical address of region to be dumped
 * @size		: Size of the region
 */
struct minidump_region {
	char	name[MAX_REGION_NAME_LENGTH];
	__le32	seq_num;
	__le32	valid;
	__le64	address;
	__le64	size;
};

/**
 * struct minidump_subsystem_toc: Subsystem's SMEM Table of content
 * @status : Subsystem toc init status
 * @enabled : if set to 1, this region would be copied during coredump
 * @encryption_status: Encryption status for this subsystem
 * @encryption_required : Decides to encrypt the subsystem regions or not
 * @region_count : Number of regions added in this subsystem toc
 * @regions_baseptr : regions base pointer of the subsystem
 */
struct minidump_subsystem {
	__le32	status;
	__le32	enabled;
	__le32	encryption_status;
	__le32	encryption_required;
	__le32	region_count;
	__le64	regions_baseptr;
};

/**
 * struct minidump_global_toc: Global Table of Content
 * @status : Global Minidump init status
 * @md_revision : Minidump revision
 * @enabled : Minidump enable status
 * @subsystems : Array of subsystems toc
 */
struct minidump_global_toc {
	__le32				status;
	__le32				md_revision;
	__le32				enabled;
	struct minidump_subsystem	subsystems[MAX_NUM_OF_SS];
};



bool ipa_minidump_enabled(void);
struct ipc_log_page *get_next_page(struct ipc_log_context *ilctxt,
		struct ipc_log_page *cur_pg);
int elf_ssr_driver_dump_retrieve_regions(
	struct elf_ssr_driver_dump_entry *input_array,
	size_t *num_entries_retrieved);
int ipa_retrieve_and_dump(void);
int ipa_do_host_ramdump(
	struct elf_ssr_driver_dump_entry *ssr_entry,
	size_t num_entries_loaded);
int ipa_ssr_driver_dump_init(void);
int ipa_ssr_driver_dump_deinit(void);
int get_ipc_and_dmesg_logs(void);
int ipa_dmesg_logs_register_each_page(char *region_name);

int ipa_ipc_logs_register_each_page(char *region_name);

int ipa_ssr_driver_dump_register_region(char *region_name,
void *region_buffer, size_t region_size);

int ipa_ssr_driver_dump_unregister_region(char *region_name);
int ipa_ssr_driver_dump_retrieve_regions(
	struct elf_ssr_driver_dump_entry *input_array,
	size_t *num_entries_loaded);


