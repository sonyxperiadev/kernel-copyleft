/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2023, Qualcomm Innovation Center, Inc. All rights reserved..
 */
#include <linux/hwspinlock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <dt-bindings/soc/qcom,ipcc.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include "ipclite_client.h"

/* version related entries */
#define MAJOR_VERSION		1
#define MINOR_VERSION		0

#define IPCMEM_INIT_COMPLETED	0x1
#define ACTIVE_CHANNEL			0x1

#define IPCMEM_TOC_SIZE			(4*1024)
#define IPCMEM_TOC_VAR_OFFSET	0x100

#define GLOBAL_ATOMIC_SUPPORT_BMSK 0x1UL

/* IPCC signal info */
#define IPCLITE_MSG_SIGNAL		0
#define IPCLITE_MEM_INIT_SIGNAL 1
#define IPCLITE_VERSION_SIGNAL  2
#define IPCLITE_TEST_SIGNAL		3
#define IPCLITE_SSR_SIGNAL		4
#define IPCLITE_DEBUG_SIGNAL	5
#define MAX_CHANNEL_SIGNALS		6

/** Flag definitions for the entries */
#define IPCMEM_FLAGS_ENABLE_READ_PROTECTION   (0x01)
#define IPCMEM_FLAGS_ENABLE_WRITE_PROTECTION  (0x02)
#define IPCMEM_FLAGS_ENABLE_RW_PROTECTION \
		(IPCMEM_FLAGS_ENABLE_READ_PROTECTION | \
		IPCMEM_FLAGS_ENABLE_WRITE_PROTECTION)

#define IPCMEM_FLAGS_IGNORE_PARTITION         (0x00000004)

/*Hardcoded macro to identify local host on each core*/
#define LOCAL_HOST		IPCMEM_APPS

/* Timeout (ms) for the trylock of remote spinlocks */
#define HWSPINLOCK_TIMEOUT	1000

/* queue related entries */
#define FIFO_FULL_RESERVE		8
#define FIFO_ALIGNMENT			8

/* debug related entries */
#define IPCLITE_DEBUG_INFO_SIZE		256
#define IPCLITE_CORE_DBG_LABEL		"APSS:"
#define IPCLITE_LOG_MSG_SIZE		100
#define IPCLITE_LOG_BUF_SIZE		512
#define IPCLITE_DBG_LABEL_SIZE		5
#define IPCLITE_SIGNAL_LABEL_SIZE	10
#define PREV_INDEX					2

#define ADD_OFFSET(x, y)	((void *)((size_t)x + y))

/* IPCLite Logging Mechanism */
#define IPCLITE_OS_LOG(__level, __fmt, arg...) \
	do { \
		if (ipclite_debug_level & __level) { \
			if (ipclite_debug_control & IPCLITE_DMESG_LOG) \
				pr_info(IPCLITE_CORE_DBG_LABEL "%s:"__fmt, \
							ipclite_dbg_label[__level], ## arg); \
			if (ipclite_debug_control & IPCLITE_INMEM_LOG) \
				ipclite_inmem_log(IPCLITE_CORE_DBG_LABEL "%s:"__fmt, \
							ipclite_dbg_label[__level], ## arg); \
		} \
	} while (0)

/* IPCLite Debug enable status */
#define IS_DEBUG_CONFIG(ipclite_debug) (ipclite_debug_control & ipclite_debug)

/* IPCLite Feature enable status */
#define IS_FEATURE_CONFIG(ipclite_feature) (feature_mask & ipclite_feature)

/* Global Atomic status */
#define ATOMIC_HW_MUTEX_ACQUIRE \
(IS_FEATURE_CONFIG(IPCLITE_GLOBAL_ATOMIC) ?: ipclite_hw_mutex_acquire())
#define ATOMIC_HW_MUTEX_RELEASE \
(IS_FEATURE_CONFIG(IPCLITE_GLOBAL_ATOMIC) ?: ipclite_hw_mutex_release())

/* API Structure */
struct ipclite_api_list {
	int (*init)(struct platform_device *pdev);
	int32_t (*register_client)(IPCLite_Client cb_func_ptr, void *priv);
	int32_t (*register_test_client)(IPCLite_Client cb_func_ptr, void *priv);
	int32_t (*msg_send)(int32_t proc_id, uint64_t data);
	int32_t (*test_msg_send)(int32_t proc_id, uint64_t data);
	int32_t (*partition_info)(struct global_region_info *global_ipcmem);
	void (*recover)(enum ipcmem_host_type core_id);
} api_list_t;

/**
 * enum ipclite_channel_status - channel status
 *
 * INACTIVE             : Channel uninitialized or init failed
 * IN_PROGRESS          : Channel init passed, awaiting confirmation from remote host
 * ACTIVE               : Channel init passed in local and remote host, thus active
 */
enum ipclite_channel_status {
	INACTIVE				= 0,
	IN_PROGRESS				= 1,
	ACTIVE					= 2,
};

enum ipclite_feature_mask {
	IPCLITE_GLOBAL_ATOMIC = 0x0001ULL,
	IPCLITE_TEST_SUITE = 0x0002ULL,
};

enum ipclite_debug_level {
	IPCLITE_ERR  = 0x0001,
	IPCLITE_WARN = 0x0002,
	IPCLITE_INFO = 0x0004,
	IPCLITE_DBG  = 0x0008,
};

enum ipclite_debug_control {
	IPCLITE_DMESG_LOG = 0x0001,
	IPCLITE_DBG_STRUCT = 0x0002,
	IPCLITE_INMEM_LOG = 0x0004,
};

enum ipclite_debug_dump {
	IPCLITE_DUMP_DBG_STRUCT = 0x0001,
	IPCLITE_DUMP_INMEM_LOG = 0x0002,
	IPCLITE_DUMP_SSR = 0x0004,
};

static const char ipclite_dbg_label[][IPCLITE_DBG_LABEL_SIZE] = {
	[IPCLITE_ERR] = "err",
	[IPCLITE_WARN] = "warn",
	[IPCLITE_INFO] = "info",
	[IPCLITE_DBG] = "dbg"
};

/**
 * IPCMEM Debug Structure Definitions
 *  - Present in Local Memory
 */

struct ipclite_debug_info_host {
	uint32_t numsig_sent; //no. of signals sent from the core
	uint32_t numsig_recv; //no. of signals received on the core
	uint32_t tx_wr_index; //write index of tx queue
	uint32_t tx_rd_index; //read index of tx queue
	uint32_t rx_wr_index; //write index of rx queue
	uint32_t rx_rd_index; //read index of rx queue
	uint32_t num_intr; //no. of interrupts received on the core
	uint32_t prev_tx_wr_index[PREV_INDEX]; //previous write index of tx queue
	uint32_t prev_tx_rd_index[PREV_INDEX]; //previous read index of tx queue
	uint32_t prev_rx_wr_index[PREV_INDEX]; //previous write index of rx queue
	uint32_t prev_rx_rd_index[PREV_INDEX]; //previous read index of rx queue
};

struct ipclite_debug_info_overall {
	uint32_t total_numsig_sent; //total no. of signals sent
	uint32_t total_numsig_recv; //total no. of signals received
	uint32_t last_sent_host_id; //last signal sent to host
	uint32_t last_recv_host_id; //last signal received from host
	uint32_t last_sigid_sent; //last sent signal id
	uint32_t last_sigid_recv; //last received signal id
};

struct ipclite_debug_info {
	uint32_t debug_version;
	uint32_t debug_level;
	uint32_t debug_control;
	uint32_t debug_dump;
	uint32_t debug_log_index;
};

struct ipclite_debug_inmem_buf {
	char IPCLITELog[IPCLITE_LOG_BUF_SIZE][IPCLITE_LOG_MSG_SIZE];
};

struct ipclite_debug_struct {
	struct ipclite_debug_info_overall dbg_info_overall;
	struct ipclite_debug_info_host dbg_info_host[IPCMEM_NUM_HOSTS];
};

/**
 * IPCMEM TOC Structure Definitions
 *  - Present in toc in shared memory
 */

struct ipcmem_host_info {
	uint32_t hwlock_owner;
	uint32_t configured_host;
};

struct ipcmem_partition_entry {
	uint32_t base_offset;	/*partition offset from IPCMEM base*/
	uint32_t size;			/*partition size*/
	uint32_t flags;			/*partition flags if required*/
	uint32_t host0;			/*subsystem 0 who can access this partition*/
	uint32_t host1;			/*subsystem 1 who can access this partition*/
	uint32_t reserved;		/*legacy partition active status*/
};

struct ipcmem_partition_info {
	uint32_t num_entries;	/* Number of channel partitions */
	uint32_t entry_size;	/* Size of partition_entry structure */
};

struct ipcmem_offsets {
	uint32_t host_info;
	uint32_t global_entry;
	uint32_t partition_info;
	uint32_t partition_entry;
	uint32_t debug;
	uint32_t reserved;		/*Padded for 64-bit alignment*/
};

/**
 * Any change in TOC header size can only be accomodated with
 * major version change, as it is not backward compatible.
 */
struct ipcmem_toc_header {
	uint32_t magic_number;		/*Checksum of TOC*/
	uint32_t init_done;			/*TOC initialization status*/
	uint32_t major_version;
	uint32_t minor_version;
	uint64_t feature_mask;
	uint32_t reserved[6];		/*Padded for future use and 64-bit alignment*/
};

/**
 * struct ipcmem_toc - Table of contents in ipcmem
 *
 * @hdr     : Header to check for toc integrity, version and features
 * @offsets : List of offsetted structures and partition entries
 *            available in the toc data region (ipcmem_toc_data)
 */
struct ipcmem_toc {
	struct ipcmem_toc_header hdr;
	struct ipcmem_offsets offsets;

	/* ---------------------------------------
	 * ipcmem_toc_data @ 256-byte offset
	 * struct ipcmem_host_info host_info;
	 * struct ipcmem_partition_entry global_entry;
	 * struct ipcmem_partition_info partition_info;
	 * struct ipcmem_partition_entry partition_entry[num_entries];
	 * ---------------------------------------
	 */
};

/**
 * IPCMEM Partition Structure Definitions
 *  - Present in partitions in shared memory
 */

struct global_partition_header {
	uint32_t partition_type;
	uint32_t region_offset;
	uint32_t region_size;
};

struct ipcmem_global_partition {
	struct global_partition_header hdr;
};

struct ipcmem_partition_header {
	uint32_t type;			   /*partition type*/
	uint32_t desc_offset;      /*descriptor offset*/
	uint32_t desc_size;        /*descriptor size*/
	uint32_t fifo0_offset;     /*fifo 0 offset*/
	uint32_t fifo0_size;       /*fifo 0 size*/
	uint32_t fifo1_offset;     /*fifo 1 offset*/
	uint32_t fifo1_size;       /*fifo 1 size*/
	uint32_t status;           /*partition status*/
};

struct ipcmem_partition {
	struct ipcmem_partition_header hdr;
};

/**
 * IPCMEM Helper Structure Definitions
 *  - Present in local memory
 *  - Can have pointers to toc and partitions in shared memory
 */

/*Pointers to offsetted structures in TOC*/
struct ipcmem_toc_data {
	struct ipcmem_host_info *host_info;
	struct ipcmem_partition_entry *global_entry;
	struct ipcmem_partition_info *partition_info;
	struct ipcmem_partition_entry *partition_entry;
};

struct ipcmem_region {
	u64 aux_base;
	void __iomem *virt_base;
	uint32_t size;
};

struct ipclite_mem {
	struct ipcmem_toc *toc;
	struct ipcmem_toc_data toc_data;
	struct ipcmem_region mem;
	struct ipcmem_global_partition *global_partition;
	struct ipcmem_partition **partition;
};

/**
 * IPCLite Structure Definitions
 *  - Present in local memory
 *  - Can have pointers to partitions in shared memory
 */

struct ipclite_fifo {
	uint32_t length;

	__le32 *tail;
	__le32 *head;

	void *fifo;

	size_t (*avail)(struct ipclite_fifo *fifo);

	void (*peak)(struct ipclite_fifo *fifo,
					void *data, size_t count);

	void (*advance)(struct ipclite_fifo *fifo,
					size_t count, uint32_t core_id);

	void (*write)(struct ipclite_fifo *fifo,
		const void *data, size_t dlen, uint32_t core_id, uint32_t signal_id);

	void (*reset)(struct ipclite_fifo *fifo);
};

struct ipclite_irq_info {
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	int irq;
	int signal_id;
	char irqname[32];
};

struct ipclite_client {
	IPCLite_Client callback;
	void *priv_data;
	int reg_complete;
};

struct ipclite_channel {
	uint32_t remote_pid;

	struct ipclite_fifo *tx_fifo;
	struct ipclite_fifo *rx_fifo;
	spinlock_t tx_lock;

	struct ipclite_irq_info irq_info[MAX_CHANNEL_SIGNALS];

	struct ipclite_client client;

	uint32_t channel_version;
	uint32_t version_finalised;

	uint32_t *gstatus_ptr;
	uint32_t status;
};

/*Single structure that defines everything about IPCLite*/
struct ipclite_info {
	struct device *dev;
	struct ipclite_channel channel[IPCMEM_NUM_HOSTS];
	struct ipclite_mem ipcmem;
	struct hwspinlock *hwlock;
	unsigned long hw_mutex_flags;
};

/*Default partition parameters*/
#define DEFAULT_PARTITION_TYPE			0x0
#define DEFAULT_PARTITION_STATUS		INACTIVE
#define DEFAULT_PARTITION_HDR_SIZE		1024

#define DEFAULT_DESCRIPTOR_OFFSET		1024
#define DEFAULT_DESCRIPTOR_SIZE			(3*1024)
#define DEFAULT_FIFO0_OFFSET			(4*1024)
#define DEFAULT_FIFO0_SIZE				(8*1024)
#define DEFAULT_FIFO1_OFFSET			(12*1024)
#define DEFAULT_FIFO1_SIZE				(8*1024)

#define DEFAULT_PARTITION_SIZE			(32*1024)
#define DEFAULT_PARTITION_FLAGS			IPCMEM_FLAGS_ENABLE_RW_PROTECTION

/*Loopback partition parameters*/
#define LOOPBACK_PARTITION_TYPE			0x1

/*Global partition parameters*/
#define GLOBAL_PARTITION_TYPE			0xFF
#define GLOBAL_PARTITION_HDR_SIZE		(4*1024)

#define GLOBAL_REGION_OFFSET			(4*1024)
#define GLOBAL_REGION_SIZE				(124*1024)

#define GLOBAL_PARTITION_SIZE			(128*1024)
#define GLOBAL_PARTITION_FLAGS			IPCMEM_FLAGS_ENABLE_RW_PROTECTION

/*Debug partition parameters*/
#define DEBUG_PARTITION_SIZE			(64*1024)

const struct ipcmem_partition_header default_partition_hdr = {
	DEFAULT_PARTITION_TYPE,
	DEFAULT_DESCRIPTOR_OFFSET,
	DEFAULT_DESCRIPTOR_SIZE,
	DEFAULT_FIFO0_OFFSET,
	DEFAULT_FIFO0_SIZE,
	DEFAULT_FIFO1_OFFSET,
	DEFAULT_FIFO1_SIZE,
	DEFAULT_PARTITION_STATUS,
};

/* TX and RX FIFO point to same location for such loopback partition type
 * (FIFO0 offset = FIFO1 offset)
 */
const struct ipcmem_partition_header loopback_partition_hdr = {
	LOOPBACK_PARTITION_TYPE,
	DEFAULT_DESCRIPTOR_OFFSET,
	DEFAULT_DESCRIPTOR_SIZE,
	DEFAULT_FIFO0_OFFSET,
	DEFAULT_FIFO0_SIZE,
	DEFAULT_FIFO0_OFFSET,
	DEFAULT_FIFO0_SIZE,
	DEFAULT_PARTITION_STATUS,
};

const struct global_partition_header global_partition_hdr = {
	GLOBAL_PARTITION_TYPE,
	GLOBAL_REGION_OFFSET,
	GLOBAL_REGION_SIZE,
};
