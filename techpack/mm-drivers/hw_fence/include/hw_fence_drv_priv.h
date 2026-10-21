/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

/* Add define only for platforms that support IPCC in dpu-hw */
#define HW_DPU_IPCC 1

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

#define HW_FENCE_HFI_CLIENT_HEADERS_SIZE (HW_FENCE_HFI_TABLE_HEADER_SIZE + \
			(HW_FENCE_HFI_QUEUE_HEADER_SIZE * HW_FENCE_CLIENT_QUEUES))

/*
 * Max Payload size is the bigest size of the message that we can have in the CTRL queue
 * in this case the max message is calculated like following, using 32-bits elements:
 * 1 header + 1 msg-type + 1 client_id + 2 hash + 1 error
 */
#define HW_FENCE_CTRL_QUEUE_MAX_PAYLOAD_SIZE ((1 + 1 + 1 + 2 + 1) * sizeof(u32))

#define HW_FENCE_CTRL_QUEUE_PAYLOAD HW_FENCE_CTRL_QUEUE_MAX_PAYLOAD_SIZE
#define HW_FENCE_CLIENT_QUEUE_PAYLOAD (sizeof(struct msm_hw_fence_queue_payload))

/* Locks area for all the clients */
#define HW_FENCE_MEM_LOCKS_SIZE (sizeof(u64) * (HW_FENCE_CLIENT_MAX - 1))

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

enum hw_fence_lookup_ops {
	HW_FENCE_LOOKUP_OP_CREATE = 0x1,
	HW_FENCE_LOOKUP_OP_DESTROY,
	HW_FENCE_LOOKUP_OP_CREATE_JOIN,
	HW_FENCE_LOOKUP_OP_FIND_FENCE
};

/**
 * enum hw_fence_loopback_id - Enum with the clients having a loopback signal (i.e AP to AP signal).
 * HW_FENCE_LOOPBACK_DPU_CTL_0: dpu client 0. Used in platforms with no dpu-ipc.
 * HW_FENCE_LOOPBACK_DPU_CTL_1: dpu client 1. Used in platforms with no dpu-ipc.
 * HW_FENCE_LOOPBACK_DPU_CTL_2: dpu client 2. Used in platforms with no dpu-ipc.
 * HW_FENCE_LOOPBACK_DPU_CTL_3: dpu client 3. Used in platforms with no dpu-ipc.
 * HW_FENCE_LOOPBACK_DPU_CTL_4: dpu client 4. Used in platforms with no dpu-ipc.
 * HW_FENCE_LOOPBACK_DPU_CTL_5: dpu client 5. Used in platforms with no dpu-ipc.
 * HW_FENCE_LOOPBACK_DPU_CTX_0: gfx client 0. Used in platforms with no gmu support.
 * HW_FENCE_LOOPBACK_VAL_0: debug validation client 0.
 * HW_FENCE_LOOPBACK_VAL_1: debug validation client 1.
 * HW_FENCE_LOOPBACK_VAL_2: debug validation client 2.
 * HW_FENCE_LOOPBACK_VAL_3: debug validation client 3.
 * HW_FENCE_LOOPBACK_VAL_4: debug validation client 4.
 * HW_FENCE_LOOPBACK_VAL_5: debug validation client 5.
 * HW_FENCE_LOOPBACK_VAL_6: debug validation client 6.
 */
enum hw_fence_loopback_id {
	HW_FENCE_LOOPBACK_DPU_CTL_0,
	HW_FENCE_LOOPBACK_DPU_CTL_1,
	HW_FENCE_LOOPBACK_DPU_CTL_2,
	HW_FENCE_LOOPBACK_DPU_CTL_3,
	HW_FENCE_LOOPBACK_DPU_CTL_4,
	HW_FENCE_LOOPBACK_DPU_CTL_5,
	HW_FENCE_LOOPBACK_GFX_CTX_0,
#if IS_ENABLED(CONFIG_DEBUG_FS)
	HW_FENCE_LOOPBACK_VAL_0,
	HW_FENCE_LOOPBACK_VAL_1,
	HW_FENCE_LOOPBACK_VAL_2,
	HW_FENCE_LOOPBACK_VAL_3,
	HW_FENCE_LOOPBACK_VAL_4,
	HW_FENCE_LOOPBACK_VAL_5,
	HW_FENCE_LOOPBACK_VAL_6,
#endif /* CONFIG_DEBUG_FS */
	HW_FENCE_LOOPBACK_MAX,
};

#define HW_FENCE_MAX_DPU_LOOPBACK_CLIENTS (HW_FENCE_LOOPBACK_DPU_CTL_5 + 1)

/**
 * enum hw_fence_client_data_id - Enum with the clients having client_data, an optional
 *                                parameter passed from the waiting client and returned
 *                                to it upon fence signaling
 * @HW_FENCE_CLIENT_DATA_ID_CTX0: GFX Client.
 * @HW_FENCE_CLIENT_DATA_ID_IPE: IPE Client.
 * @HW_FENCE_CLIENT_DATA_ID_VPU: VPU Client.
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
 */
struct msm_hw_fence_queue {
	void *va_queue;
	u32 q_size_bytes;
	void *va_header;
	phys_addr_t pa_queue;
};

/**
 * enum payload_type - Enum with the queue payload types.
 */
enum payload_type {
	HW_FENCE_PAYLOAD_TYPE_1 = 1
};

/**
 * struct msm_hw_fence_client - Structure holding the per-Client allocated resources.
 * @client_id: id of the client
 * @mem_descriptor: hfi header memory descriptor
 * @queues: queues descriptor
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
	struct msm_hw_fence_mem_addr mem_descriptor;
	struct msm_hw_fence_queue queues[HW_FENCE_CLIENT_QUEUES];
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
 * struct hw_fence_driver_data - Structure holding internal hw-fence driver data
 *
 * @dev: device driver pointer
 * @resources_ready: value set by driver at end of probe, once all resources are ready
 * @hw_fence_table_entries: total number of hw-fences in the global table
 * @hw_fence_mem_fences_table_size: hw-fences global table total size
 * @hw_fence_queue_entries: total number of entries that can be available in the queue
 * @hw_fence_ctrl_queue_size: size of the ctrl queue for the payload
 * @hw_fence_mem_ctrl_queues_size: total size of ctrl queues, including: header + rxq + txq
 * @hw_fence_client_queue_size: size of the client queue for the payload
 * @hw_fence_mem_clients_queues_size: total size of client queues, including: header + rxq + txq
 * @hw_fences_tbl: pointer to the hw-fences table
 * @hw_fences_tbl_cnt: number of elements in the hw-fence table
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
 * @ctl_start_ptr: pointer to the ctl_start registers of the display hw (platforms with no dpu-ipc)
 * @ctl_start_size: size of the ctl_start registers of the display hw (platforms with no dpu-ipc)
 * @client_id_mask: bitmask for tracking registered client_ids
 * @clients_register_lock: lock to synchronize clients registration and deregistration
 * @msm_hw_fence_client: table with the handles of the registered clients
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
	u32 hw_fence_client_queue_size;
	u32 hw_fence_mem_clients_queues_size;

	/* HW Fences Table VA */
	struct msm_hw_fence *hw_fences_tbl;
	u32 hw_fences_tbl_cnt;

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

	/* base address for dpu ctl start regs */
	void *ctl_start_ptr[HW_FENCE_MAX_DPU_LOOPBACK_CLIENTS];
	uint32_t ctl_start_size[HW_FENCE_MAX_DPU_LOOPBACK_CLIENTS];

	/* synchronize client_ids registration and deregistration */
	struct mutex clients_register_lock;

	/* table with registered client handles */
	struct msm_hw_fence_client *clients[HW_FENCE_CLIENT_MAX];

	bool vm_ready;
#ifdef HW_DPU_IPCC
	/* state variables */
	bool ipcc_dpu_initialized;
#endif /* HW_DPU_IPCC */
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
inline u64 hw_fence_get_qtime(struct hw_fence_driver_data *drv_data);
int hw_fence_read_queue(struct msm_hw_fence_client *hw_fence_client,
	struct msm_hw_fence_queue_payload *payload, int queue_type);
int hw_fence_register_wait_client(struct hw_fence_driver_data *drv_data,
	struct dma_fence *fence, struct msm_hw_fence_client *hw_fence_client, u64 context,
	u64 seqno, u64 *hash, u64 client_data);
struct msm_hw_fence *msm_hw_fence_find(struct hw_fence_driver_data *drv_data,
	struct msm_hw_fence_client *hw_fence_client,
	u64 context, u64 seqno, u64 *hash);
enum hw_fence_client_data_id hw_fence_get_client_data_id(enum hw_fence_client_id client_id);

#endif /* __HW_FENCE_DRV_INTERNAL_H */
