/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __HW_FENCE_DRV_INTERNAL_H
#define __HW_FENCE_DRV_INTERNAL_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/soc/qcom/msm_hw_fence.h>
#include <linux/dma-fence-array.h>
#include <linux/slab.h>

/* max u64 to indicate invalid fence */
#define HW_FENCE_INVALID_PARENT_FENCE (~0ULL)

/* hash algorithm constants */
#define HW_FENCE_HASH_A_MULT	4969 /* a multiplier for Hash algorithm */
#define HW_FENCE_HASH_C_MULT	907  /* c multiplier for Hash algorithm */

/* number of queues per type (i.e. ctrl or client queues) */
#define HW_FENCE_CTRL_QUEUES	2 /* Rx and Tx Queues */
#define HW_FENCE_CLIENT_QUEUES	2 /* Rx and Tx Queues */

/* hfi headers calculation */
#define HW_FENCE_HFI_TABLE_HEADER_SIZE (sizeof(struct msm_hw_fence_hfi_queue_table_header))
#define HW_FENCE_HFI_QUEUE_HEADER_SIZE (sizeof(struct msm_hw_fence_hfi_queue_header))

#define HW_FENCE_HFI_CTRL_HEADERS_SIZE (HW_FENCE_HFI_TABLE_HEADER_SIZE + \
			(HW_FENCE_HFI_QUEUE_HEADER_SIZE * HW_FENCE_CTRL_QUEUES))

#define HW_FENCE_HFI_CLIENT_HEADERS_SIZE(queues_num) (HW_FENCE_HFI_TABLE_HEADER_SIZE + \
			(HW_FENCE_HFI_QUEUE_HEADER_SIZE * queues_num))

/*
 * Max Payload size is the bigest size of the message that we can have in the CTRL queue
 * in this case the max message is calculated like following, using 32-bits elements:
 * 1 header + 1 msg-type + 1 client_id + 2 hash + 1 error
 */
#define HW_FENCE_CTRL_QUEUE_MAX_PAYLOAD_SIZE ((1 + 1 + 1 + 2 + 1) * sizeof(u32))

#define HW_FENCE_CTRL_QUEUE_PAYLOAD HW_FENCE_CTRL_QUEUE_MAX_PAYLOAD_SIZE
#define HW_FENCE_CLIENT_QUEUE_PAYLOAD (sizeof(struct msm_hw_fence_queue_payload))

/* Locks area for all clients with RxQ */
#define HW_FENCE_MEM_LOCKS_SIZE(rxq_clients_num) (sizeof(u64) * rxq_clients_num)

#define HW_FENCE_TX_QUEUE 1
#define HW_FENCE_RX_QUEUE 2

/* ClientID for the internal join fence, this is used by the framework when creating a join-fence */
#define HW_FENCE_JOIN_FENCE_CLIENT_ID (~(u32)0)

/**
 * msm hw fence flags:
 * MSM_HW_FENCE_FLAG_SIGNAL - Flag set when the hw-fence is signaled
 */
#define MSM_HW_FENCE_FLAG_SIGNAL	BIT(0)

/**
 * MSM_HW_FENCE_MAX_JOIN_PARENTS:
 * Maximum number of parents that a fence can have for a join-fence
 */
#define MSM_HW_FENCE_MAX_JOIN_PARENTS	3

/**
 * HW_FENCE_PAYLOAD_REV:
 * Payload version with major and minor version information
 */
#define HW_FENCE_PAYLOAD_REV(major, minor) (major << 8 | (minor & 0xFF))

/**
 * HW_FENCE_EVENT_MAX_DATA:
 * Maximum data that can be added to the debug event
 */
#define HW_FENCE_EVENT_MAX_DATA 12

enum hw_fence_lookup_ops {
	HW_FENCE_LOOKUP_OP_CREATE = 0x1,
	HW_FENCE_LOOKUP_OP_DESTROY,
	HW_FENCE_LOOKUP_OP_CREATE_JOIN,
	HW_FENCE_LOOKUP_OP_FIND_FENCE
};

/**
 * enum hw_fence_client_data_id - Enum with the clients having client_data, an optional
 *                                parameter passed from the waiting client and returned
 *                                to it upon fence signaling. Only the first HW Fence
 *                                Client for non-VAL clients (e.g. GFX, IPE, VPU) have
 *                                client_data.
 * @HW_FENCE_CLIENT_DATA_ID_CTX0: GFX Client 0.
 * @HW_FENCE_CLIENT_DATA_ID_IPE: IPE Client 0.
 * @HW_FENCE_CLIENT_DATA_ID_VPU: VPU Client 0.
 * @HW_FENCE_CLIENT_DATA_ID_VAL0: Debug validation client 0.
 * @HW_FENCE_CLIENT_DATA_ID_VAL1: Debug validation client 1.
 * @HW_FENCE_MAX_CLIENTS_WITH_DATA: Max number of clients with data, also indicates an
 *                                  invalid hw_fence_client_data_id
 */
enum hw_fence_client_data_id {
	HW_FENCE_CLIENT_DATA_ID_CTX0,
	HW_FENCE_CLIENT_DATA_ID_IPE,
	HW_FENCE_CLIENT_DATA_ID_VPU,
	HW_FENCE_CLIENT_DATA_ID_VAL0,
	HW_FENCE_CLIENT_DATA_ID_VAL1,
	HW_FENCE_MAX_CLIENTS_WITH_DATA,
};

/**
 * struct msm_hw_fence_queue - Structure holding the data of the hw fence queues.
 * @va_queue: pointer to the virtual address of the queue elements
 * @q_size_bytes: size of the queue
 * @va_header: pointer to the hfi header virtual address
 * @pa_queue: physical address of the queue
 * @rd_wr_idx_start: start read and write indexes for client queue (zero by default)
 * @rd_wr_idx_factor: factor to multiply custom index to get index in dwords (one by default)
 * @skip_wr_idx: bool to indicate if update to write_index is skipped within hw fence driver and
 *               hfi_header->tx_wm is updated instead
 */
struct msm_hw_fence_queue {
	void *va_queue;
	u32 q_size_bytes;
	void *va_header;
	phys_addr_t pa_queue;
	u32 rd_wr_idx_start;
	u32 rd_wr_idx_factor;
	bool skip_wr_idx;
};

/**
 * enum payload_type - Enum with the queue payload types.
 * HW_FENCE_PAYLOAD_TYPE_1: client queue payload
 * HW_FENCE_PAYLOAD_TYPE_2: ctrl queue payload for fence error; client_data stores client_id
 */
enum payload_type {
	HW_FENCE_PAYLOAD_TYPE_1 = 1,
	HW_FENCE_PAYLOAD_TYPE_2
};

/**
 * struct msm_hw_fence_client - Structure holding the per-Client allocated resources.
 * @client_id: internal client_id used within HW fence driver; index into the clients struct
 * @client_id_ext: external client_id, equal to client_id except for clients with configurable
 *                 number of sub-clients (e.g. ife clients)
 * @mem_descriptor: hfi header memory descriptor
 * @queues: queues descriptor
 * @queues_num: number of client queues
 * @fence_error_cb: function called for waiting clients that need HLOS notification of fence error
 * @fence_error_cb_userdata: opaque pointer registered with fence error callback and passed to
 *                           client during invocation of callback function
 * @error_cb_lock: lock to synchronize access to fence error cb and fence error cb data
 * @ipc_signal_id: id of the signal to be triggered for this client
 * @ipc_client_vid: virtual id of the ipc client for this hw fence driver client
 * @ipc_client_pid: physical id of the ipc client for this hw fence driver client
 * @update_rxq: bool to indicate if client uses rx-queue
 * @send_ipc: bool to indicate if client requires ipc interrupt for already signaled fences
 * @wait_queue: wait queue for the validation clients
 * @val_signal: doorbell flag to signal the validation clients in the wait queue
 */
struct msm_hw_fence_client {
	enum hw_fence_client_id client_id;
	enum hw_fence_client_id client_id_ext;
	struct msm_hw_fence_mem_addr mem_descriptor;
	struct msm_hw_fence_queue queues[HW_FENCE_CLIENT_QUEUES];
	int queues_num;
	msm_hw_fence_error_cb_t fence_error_cb;
	void *fence_error_cb_userdata;
	struct mutex error_cb_lock;
	int ipc_signal_id;
	int ipc_client_vid;
	int ipc_client_pid;
	bool update_rxq;
	bool send_ipc;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	wait_queue_head_t wait_queue;
	atomic_t val_signal;
#endif /* CONFIG_DEBUG_FS */
};

/**
 * struct msm_hw_fence_mem_data - Structure holding internal memory attributes
 *
 * @attrs: attributes for the memory allocation
 */
struct msm_hw_fence_mem_data {
	unsigned long attrs;
};

/**
 * struct msm_hw_fence_dbg_data - Structure holding debugfs data
 *
 * @root: debugfs root
 * @entry_rd: flag to indicate if debugfs dumps a single line or table
 * @context_rd: debugfs setting to indicate which context id to dump
 * @seqno_rd: debugfs setting to indicate which seqno to dump
 * @hw_fence_sim_release_delay: delay in micro seconds for the debugfs node that simulates the
 *                              hw-fences behavior, to release the hw-fences
 * @create_hw_fences: boolean to continuosly create hw-fences within debugfs
 * @clients_list: list of debug clients registered
 * @clients_list_lock: lock to synchronize access to the clients list
 * @lock_wake_cnt: number of times that driver triggers wake-up ipcc to unlock inter-vm try-lock
 */
struct msm_hw_fence_dbg_data {
	struct dentry *root;

	bool entry_rd;
	u64 context_rd;
	u64 seqno_rd;

	u32 hw_fence_sim_release_delay;
	bool create_hw_fences;

	struct list_head clients_list;
	struct mutex clients_list_lock;

	u64 lock_wake_cnt;
};

/**
 * struct hw_fence_client_type_desc - Structure holding client type properties, including static
 *                                    properties and client queue properties read from device-tree.
 *
 * @name: name of client type, used to parse properties from device-tree
 * @init_id: initial client_id for given client type within the 'hw_fence_client_id' enum, e.g.
 *           HW_FENCE_CLIENT_ID_CTL0 for DPU clients
 * @max_clients_num: maximum number of clients of given client type
 * @clients_num: number of clients of given client type
 * @queues_num: number of queues per client of given client type; either one (for only Tx Queue) or
 *              two (for both Tx and Rx Queues)
 * @queue_entries: number of entries per client queue of given client type
 * @start_padding: size of padding between queue table header and first queue header in bytes
 * @end_padding: size of padding between queue header(s) and first queue payload in bytes
 * @mem_size: size of memory allocated for client queue(s) per client in bytes
 * @txq_idx_start: start read and write indexes for client tx queue (zero by default)
 * @txq_idx_factor: factor to multiply custom TxQ idx to get index in dwords (one by default)
 * @skip_txq_wr_idx: bool to indicate if update to tx queue write_index is skipped within hw fence
 *                   driver and hfi_header->tx_wm is updated instead
 */
struct hw_fence_client_type_desc {
	char *name;
	enum hw_fence_client_id init_id;
	u32 max_clients_num;
	u32 clients_num;
	u32 queues_num;
	u32 queue_entries;
	u32 start_padding;
	u32 end_padding;
	u32 mem_size;
	u32 txq_idx_start;
	u32 txq_idx_factor;
	bool skip_txq_wr_idx;
};

/**
 * struct hw_fence_client_queue_desc - Structure holding client queue properties for a client.
 *
 * @type: pointer to client queue properties of client type
 * @start_offset: start offset of client queue memory region, from beginning of carved-out memory
 *                allocation for hw fence driver
 */
struct hw_fence_client_queue_desc {
	struct hw_fence_client_type_desc *type;
	u32 start_offset;
};

/**
 * struct hw_fence_driver_data - Structure holding internal hw-fence driver data
 *
 * @dev: device driver pointer
 * @resources_ready: value set by driver at end of probe, once all resources are ready
 * @hw_fence_table_entries: total number of hw-fences in the global table
 * @hw_fence_mem_fences_table_size: hw-fences global table total size
 * @hw_fence_queue_entries: total number of entries that can be available in the queue
 * @hw_fence_ctrl_queue_size: size of the ctrl queue for the payload
 * @hw_fence_mem_ctrl_queues_size: total size of ctrl queues, including: header + rxq + txq
 * @hw_fence_client_queue_size: descriptors of client queue properties for each hw fence client
 * @hw_fence_client_types: descriptors of properties for each hw fence client type
 * @rxq_clients_num: number of supported hw fence clients with rxq (configured based on device-tree)
 * @clients_num: number of supported hw fence clients (configured based on device-tree)
 * @hw_fences_tbl: pointer to the hw-fences table
 * @hw_fences_tbl_cnt: number of elements in the hw-fence table
 * @events: start address of hw fence debug events
 * @total_events: total number of hw fence debug events supported
 * @client_lock_tbl: pointer to the per-client locks table
 * @client_lock_tbl_cnt: number of elements in the locks table
 * @hw_fences_mem_desc: memory descriptor for the hw-fence table
 * @clients_locks_mem_desc: memory descriptor for the locks table
 * @ctrl_queue_mem_desc: memory descriptor for the ctrl queues
 * @ctrl_queues: pointer to the ctrl queues
 * @io_mem_base: pointer to the carved-out io memory
 * @res: resources for the carved out memory
 * @size: size of the carved-out memory
 * @label: label for the carved-out memory (this is used by SVM to find the memory)
 * @peer_name: peer name for this carved-out memory
 * @rm_nb: hyp resource manager notifier
 * @memparcel: memparcel for the allocated memory
 * @used_mem_size: total memory size of global table, lock region, and ctrl and client queues
 * @db_label: doorbell label
 * @rx_dbl: handle to the Rx doorbell
 * @debugfs_data: debugfs info
 * @ipcc_reg_base: base for ipcc regs mapping
 * @ipcc_io_mem: base for the ipcc io mem map
 * @ipcc_size: size of the ipcc io mem mapping
 * @protocol_id: ipcc protocol id used by this driver
 * @ipcc_client_vid: ipcc client virtual-id for this driver
 * @ipcc_client_pid: ipcc client physical-id for this driver
 * @ipc_clients_table: table with the ipcc mapping for each client of this driver
 * @qtime_reg_base: qtimer register base address
 * @qtime_io_mem: qtimer io mem map
 * @qtime_size: qtimer io mem map size
 * @client_id_mask: bitmask for tracking registered client_ids
 * @clients_register_lock: lock to synchronize clients registration and deregistration
 * @clients: table with the handles of the registered clients; size is equal to clients_num
 * @vm_ready: flag to indicate if vm has been initialized
 * @ipcc_dpu_initialized: flag to indicate if dpu hw is initialized
 */
struct hw_fence_driver_data {

	struct device *dev;
	bool resources_ready;

	/* Table & Queues info */
	u32 hw_fence_table_entries;
	u32 hw_fence_mem_fences_table_size;
	u32 hw_fence_queue_entries;
	/* ctrl queues */
	u32 hw_fence_ctrl_queue_size;
	u32 hw_fence_mem_ctrl_queues_size;
	/* client queues */
	struct hw_fence_client_queue_desc *hw_fence_client_queue_size;
	struct hw_fence_client_type_desc *hw_fence_client_types;
	u32 rxq_clients_num;
	u32 clients_num;

	/* HW Fences Table VA */
	struct msm_hw_fence *hw_fences_tbl;
	u32 hw_fences_tbl_cnt;

	/* events */
	struct msm_hw_fence_event *events;
	u32 total_events;

	/* Table with a Per-Client Lock */
	u64 *client_lock_tbl;
	u32 client_lock_tbl_cnt;

	/* Memory Descriptors */
	struct msm_hw_fence_mem_addr hw_fences_mem_desc;
	struct msm_hw_fence_mem_addr clients_locks_mem_desc;
	struct msm_hw_fence_mem_addr ctrl_queue_mem_desc;
	struct msm_hw_fence_queue ctrl_queues[HW_FENCE_CTRL_QUEUES];

	/* carved out memory */
	void __iomem *io_mem_base;
	struct resource res;
	size_t size;
	u32 label;
	u32 peer_name;
	struct notifier_block rm_nb;
	u32 memparcel;
	u32 used_mem_size;

	/* doorbell */
	u32 db_label;

	/* VM virq */
	void *rx_dbl;

	/* debugfs */
	struct msm_hw_fence_dbg_data debugfs_data;

	/* ipcc regs */
	phys_addr_t ipcc_reg_base;
	void __iomem *ipcc_io_mem;
	uint32_t ipcc_size;
	u32 protocol_id;
	u32 ipcc_client_vid;
	u32 ipcc_client_pid;

	/* table with mapping of ipc client for each hw-fence client */
	struct hw_fence_client_ipc_map *ipc_clients_table;

	/* qtime reg */
	phys_addr_t qtime_reg_base;
	void __iomem *qtime_io_mem;
	uint32_t qtime_size;

	/* synchronize client_ids registration and deregistration */
	struct mutex clients_register_lock;

	/* table with registered client handles */
	struct msm_hw_fence_client **clients;

	bool vm_ready;
	/* state variables */
	bool ipcc_dpu_initialized;
};

/**
 * struct msm_hw_fence_queue_payload - hardware fence clients queues payload.
 * @size: size of queue payload
 * @type: type of queue payload
 * @version: version of queue payload. High eight bits are for major and lower eight
 *           bits are for minor version
 * @ctxt_id: context id of the dma fence
 * @seqno: sequence number of the dma fence
 * @hash: fence hash
 * @flags: see MSM_HW_FENCE_FLAG_* flags descriptions
 * @client_data: data passed from and returned to waiting client upon fence signaling
 * @error: error code for this fence, fence controller receives this
 *		  error from the signaling client through the tx queue and
 *		  propagates the error to the waiting client through rx queue
 * @timestamp_lo: low 32-bits of qtime of when the payload is written into the queue
 * @timestamp_hi: high 32-bits of qtime of when the payload is written into the queue
 */
struct msm_hw_fence_queue_payload {
	u32 size;
	u16 type;
	u16 version;
	u64 ctxt_id;
	u64 seqno;
	u64 hash;
	u64 flags;
	u64 client_data;
	u32 error;
	u32 timestamp_lo;
	u32 timestamp_hi;
	u32 reserve;
};

/**
 * struct msm_hw_fence_event - hardware fence ctl debug event
 * time: qtime when the event is logged
 * cpu: cpu id where the event is logged
 * data_cnt: count of valid data available in the data field
 * data: debug data logged by the event
 */
struct msm_hw_fence_event {
	u64 time;
	u32 cpu;
	u32 data_cnt;
	u32 data[HW_FENCE_EVENT_MAX_DATA];
};

/**
 * struct msm_hw_fence - structure holding each hw fence data.
 * @valid: field updated when a hw-fence is reserved. True if hw-fence is in use
 * @error: field to hold a hw-fence error
 * @ctx_id: context id
 * @seq_id: sequence id
 * @wait_client_mask: bitmask holding the waiting-clients of the fence
 * @fence_allocator: field to indicate the client_id that reserved the fence
 * @fence_signal-client:
 * @lock: this field is required to share information between the Driver & Driver ||
 *        Driver & FenceCTL. Needs to be 64-bit atomic inter-processor lock.
 * @flags: field to indicate the state of the fence
 * @parent_list: list of indexes with the parents for a child-fence in a join-fence
 * @parent_cnt: total number of parents for a child-fence in a join-fence
 * @pending_child_cnt: children refcount for a parent-fence in a join-fence. Access must be atomic
 *        or locked
 * @fence_create_time: debug info with the create time timestamp
 * @fence_trigger_time: debug info with the trigger time timestamp
 * @fence_wait_time: debug info with the register-for-wait timestamp
 * @debug_refcount: refcount used for debugging
 * @client_data: array of data optionally passed from and returned to clients waiting on the fence
 *               during fence signaling
 */
struct msm_hw_fence {
	u32 valid;
	u32 error;
	u64 ctx_id;
	u64 seq_id;
	u64 wait_client_mask;
	u32 fence_allocator;
	u32 fence_signal_client;
	u64 lock;	/* Datatype must be 64-bit. */
	u64 flags;
	u64 parent_list[MSM_HW_FENCE_MAX_JOIN_PARENTS];
	u32 parents_cnt;
	u32 pending_child_cnt;
	u64 fence_create_time;
	u64 fence_trigger_time;
	u64 fence_wait_time;
	u64 debug_refcount;
	u64 client_data[HW_FENCE_MAX_CLIENTS_WITH_DATA];
};

int hw_fence_init(struct hw_fence_driver_data *drv_data);
int hw_fence_alloc_client_resources(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client,
	struct msm_hw_fence_mem_addr *mem_descriptor);
int hw_fence_init_controller_signal(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client);
int hw_fence_init_controller_resources(struct msm_hw_fence_client *hw_fence_client);
void hw_fence_cleanup_client(struct hw_fence_driver_data *drv_data,
	 struct msm_hw_fence_client *hw_fence_client);
void hw_fence_utils_reset_queues(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client);
int hw_fence_create(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client,
	u64 context, u64 seqno, u64 *hash);
int hw_fence_destroy(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client,
	u64 context, u64 seqno);
int hw_fence_destroy_with_hash(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client, u64 hash);
int hw_fence_process_fence_array(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client,
	struct dma_fence_array *array, u64 *hash_join_fence, u64 client_data);
int hw_fence_process_fence(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client, struct dma_fence *fence, u64 *hash,
	u64 client_data);
int hw_fence_update_queue(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client, u64 ctxt_id, u64 seqno, u64 hash,
	u64 flags, u64 client_data, u32 error, int queue_type);
int hw_fence_update_existing_txq_payload(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client, u64 hash, u32 error);
inline u64 hw_fence_get_qtime(struct hw_fence_driver_data *drv_data);
int hw_fence_read_queue(struct msm_hw_fence_client *hw_fence_client,
	struct msm_hw_fence_queue_payload *payload, int queue_type);
int hw_fence_read_queue_helper(struct msm_hw_fence_queue *queue,
	struct msm_hw_fence_queue_payload *payload);
int hw_fence_register_wait_client(struct hw_fence_driver_data *drv_data,
	struct dma_fence *fence, struct msm_hw_fence_client *hw_fence_client, u64 context,
	u64 seqno, u64 *hash, u64 client_data);
struct msm_hw_fence *msm_hw_fence_find(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client,
	u64 context, u64 seqno, u64 *hash);
enum hw_fence_client_data_id hw_fence_get_client_data_id(enum hw_fence_client_id client_id);

#endif /* __HW_FENCE_DRV_INTERNAL_H */
