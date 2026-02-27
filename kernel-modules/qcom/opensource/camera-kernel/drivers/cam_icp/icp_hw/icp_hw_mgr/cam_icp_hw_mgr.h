/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef CAM_ICP_HW_MGR_H
#define CAM_ICP_HW_MGR_H

#include <linux/types.h>
#include <linux/completion.h>
#include <linux/semaphore.h>
#include <media/cam_icp.h>
#include "cam_icp_hw_intf.h"
#include "cam_hw_mgr_intf.h"
#include "cam_hw_intf.h"
#include "hfi_session_defs.h"
#include "hfi_intf.h"
#include "cam_req_mgr_workq.h"
#include "cam_mem_mgr.h"
#include "cam_smmu_api.h"
#include "cam_soc_util.h"
#include "cam_req_mgr_timer.h"

#define CAM_ICP_ROLE_PARENT     1
#define CAM_ICP_ROLE_CHILD      2

#define CAM_FRAME_CMD_MAX       40

#define CAM_MAX_OUT_RES         14
#define CAM_MAX_IN_RES          16

#define ICP_WORKQ_NUM_TASK      100
#define ICP_WORKQ_TASK_CMD_TYPE 1
#define ICP_WORKQ_TASK_MSG_TYPE 2

#define ICP_PACKET_SIZE         0
#define ICP_PACKET_TYPE         1
#define ICP_PACKET_OPCODE       2
#define ICP_MAX_OUTPUT_SUPPORTED 6

#define ICP_FRAME_PROCESS_SUCCESS 0
#define ICP_FRAME_PROCESS_FAILURE 1

/* size of buffer to drain from msg/dbq queue */
#define ICP_MSG_BUF_SIZE_IN_WORDS 512
#define ICP_DBG_BUF_SIZE_IN_WORDS 25600

#define ICP_OVER_CLK_THRESHOLD  5
#define ICP_TWO_DEV_BW_SHARE_RATIO 2

#define CPAS_IPE0_BIT           0x1000
#define CPAS_IPE1_BIT           0x2000
#define CPAS_BPS_BIT            0x400
#define CPAS_ICP_BIT            0x1
#define CPAS_ICP1_BIT           0x4
#define CPAS_OFE_BIT            0x10

/* Used for targets >= 480 and its variants */
#define CPAS_TITAN_IPE0_CAP_BIT 0x800

#define CAM_ICP_CTX_STATE_FREE      0x0
#define CAM_ICP_CTX_STATE_IN_USE    0x1
#define CAM_ICP_CTX_STATE_ACQUIRED  0x2
#define CAM_ICP_CTX_STATE_RELEASE   0x3

#define CAM_ICP_CTX_MAX_CMD_BUFFERS 0x2

/* Current appliacble vote paths, based on number of UAPI definitions */
#define CAM_ICP_MAX_PER_PATH_VOTES 12

#define CAM_ICP_HW_MGR_NAME_SIZE  32

#define CAM_ICP_IS_DEV_HW_EXIST(hw_cap_mask, hw_dev_type)  \
({                                                         \
	(hw_cap_mask) & BIT((hw_dev_type));                \
})

#define CAM_ICP_IS_VALID_HW_DEV_TYPE(type)                          \
({                                                                  \
	((type) >= CAM_ICP_HW_ICP_V1) && ((type) < CAM_ICP_HW_MAX); \
})

#define CAM_ICP_MAX_ICP_PROC_PER_DEV 1

struct hfi_mini_dump_info;

/**
 * struct icp_hfi_mem_info
 * @qtbl: Memory info of queue table
 * @cmd_q: Memory info of command queue
 * @msg_q: Memory info of message queue
 * @dbg_q: Memory info of debug queue
 * @sec_heap: Memory info of secondary heap
 * @fw_buf: Memory info of firmware
 * @qdss_buf: Memory info of qdss
 * @sfr_buf: Memory info for sfr buffer
 * @fw_uncached_generic: Memory info for fw uncached region
 * @fw_uncached_global_sync: Memory info for global sync, in fw uncached region
 * @synx_hwmutex: Memory info for synx hwmutex region mapped as device memory
 * @ipc_hwmutex: Memory info for ipc hwmutex region mapped as device memory
 * @global_cntr: Memory info for global cntr region mapped as device memory
 * @shmem: Memory info for shared region
 * @io_mem: Memory info for io region
 * @fw_uncached: Memory info for fw uncached nested region
 * @device: Memory info for the device region
 * @fw_uncached_region: region support for fw uncached
 */
struct icp_hfi_mem_info {
	struct cam_mem_mgr_memory_desc qtbl;
	struct cam_mem_mgr_memory_desc cmd_q;
	struct cam_mem_mgr_memory_desc msg_q;
	struct cam_mem_mgr_memory_desc dbg_q;
	struct cam_mem_mgr_memory_desc sec_heap;
	struct cam_mem_mgr_memory_desc fw_buf;
	struct cam_mem_mgr_memory_desc qdss_buf;
	struct cam_mem_mgr_memory_desc sfr_buf;
	struct cam_mem_mgr_memory_desc fw_uncached_generic;
	struct cam_mem_mgr_memory_desc fw_uncached_global_sync;
	struct cam_mem_mgr_memory_desc synx_hwmutex;
	struct cam_mem_mgr_memory_desc ipc_hwmutex;
	struct cam_mem_mgr_memory_desc global_cntr;
	struct cam_smmu_region_info shmem;
	struct cam_smmu_region_info io_mem;
	struct cam_smmu_region_info fw_uncached;
	struct cam_smmu_region_info device;
	bool fw_uncached_region;
};

/**
 * struct hfi_cmd_work_data
 * @type: Task type
 * @data: Pointer to command data
 * @request_id: Request id
 */
struct hfi_cmd_work_data {
	uint32_t type;
	void *data;
	int32_t request_id;
};

/**
 * struct hfi_msg_work_data
 * @type: Task type
 * @data: Pointer to message data
 * @recover: Device needs recovery
 */
struct hfi_msg_work_data {
	uint32_t type;
	void *data;
	bool recover;
};

/**
 * struct clk_work_data
 * @type: Task type
 * @data: Pointer to clock info
 */
struct clk_work_data {
	uint32_t type;
	void *data;
};

/*
 * struct icp_frame_info
 * @request_id: request id
 * @io_config: the address of io config
 * @hfi_cfg_io_cmd: command struct to be sent to hfi
 * @pkt: pointer to the packet header of current request
 */
struct icp_frame_info {
	uint64_t request_id;
	dma_addr_t io_config;
	struct hfi_cmd_dev_async hfi_cfg_io_cmd;
	struct cam_packet *pkt;
};

/**
 * struct cam_icp_clk_bw_request_v2
 *
 * @budget_ns: Time required to process frame
 * @frame_cycles: Frame cycles needed to process the frame
 * @rt_flag: Flag to indicate real time stream
 * @reserved: Reserved filed.
 * @num_paths: Number of paths for per path bw vote
 * @axi_path: Per path vote info for IPE/BPS
 */
struct cam_icp_clk_bw_req_internal_v2 {
	uint64_t budget_ns;
	uint32_t frame_cycles;
	uint32_t rt_flag;
	uint32_t reserved;
	uint32_t num_paths;
	struct cam_cpas_axi_per_path_bw_vote axi_path[CAM_ICP_MAX_PER_PATH_VOTES];
};

#define HANG_DUMP_REGIONS_MAX 10

/**
 * struct cam_hangdump_mem_regions -
 *        List of multiple memory descriptors of different
 *        regions
 *
 * @num_regions    : Number of regions
 * @map_info_array : Array of all the regions
 */
struct cam_hangdump_mem_regions {
	uint32_t num_mem_regions;
	struct cam_cmd_mem_region_info mem_info_array[HANG_DUMP_REGIONS_MAX];
};

/**
 * struct cam_icp_ctx_perf_stats -
 *        ICP general Perf stats per ctx
 *
 * @total_resp_time: accumulative FW response time
 * @total_requests : accumulative submitted requests
 */
struct cam_icp_ctx_perf_stats {
	uint64_t total_resp_time;
	uint64_t total_requests;
};

/**
 * struct cam_icp_hw_ctx_dev_info -
 *        Info of ICP devices (IPE/BPS/OFE) that can be attached to a context
 *
 * @dev_ctxt_cnt : device context count
 * @dev_clk_state: device clock state
 */
struct cam_icp_hw_ctx_dev_info {
	uint32_t dev_ctxt_cnt;
	bool dev_clk_state;
};

/**
 * struct hfi_frame_process_info
 * @hfi_frame_cmd: Frame process command info
 * @bitmap: Bitmap for hfi_frame_cmd
 * @bits: Used in hfi_frame_cmd bitmap
 * @lock: Lock for hfi_frame_cmd
 * @request_id: Request id list
 * @num_out_resources: Number of out syncs
 * @out_resource: Out sync info
 * @fw_process_flag: Frame process flag
 * @clk_info: Clock information for a request
 * @clk_info_v2: Clock info for AXI bw voting v2
 * @frame_info: information needed to process request
 * @submit_timestamp: Submit timestamp to hw
 * @hangdump_mem_regions: Mem regions for hangdump
 */
struct hfi_frame_process_info {
	struct hfi_cmd_dev_async hfi_frame_cmd[CAM_FRAME_CMD_MAX];
	void *bitmap;
	size_t bits;
	struct mutex lock;
	uint64_t request_id[CAM_FRAME_CMD_MAX];
	uint32_t num_out_resources[CAM_FRAME_CMD_MAX];
	uint32_t out_resource[CAM_FRAME_CMD_MAX][CAM_MAX_OUT_RES];
	uint32_t in_resource[CAM_FRAME_CMD_MAX];
	uint32_t in_free_resource[CAM_FRAME_CMD_MAX];
	bool fw_process_flag[CAM_FRAME_CMD_MAX];
	struct cam_icp_clk_bw_request clk_info[CAM_FRAME_CMD_MAX];
	struct cam_icp_clk_bw_req_internal_v2 clk_info_v2[CAM_FRAME_CMD_MAX];
	struct icp_frame_info frame_info[CAM_FRAME_CMD_MAX];
	ktime_t submit_timestamp[CAM_FRAME_CMD_MAX];
	struct cam_hangdump_mem_regions *hangdump_mem_regions;
};

/**
 * struct cam_ctx_clk_info
 * @curr_fc: Context latest request frame cycles
 * @rt_flag: Flag to indicate real time request
 * @base_clk: Base clock to process the request
 * @reserved: Reserved field
 * #uncompressed_bw: Current bandwidth voting
 * @compressed_bw: Current compressed bandwidth voting
 * @clk_rate: Supported clock rates for the context
 * @num_paths: Number of valid AXI paths
 * @axi_path: ctx based per path bw vote
 * @bw_included: Whether bw of this context is included in overal voting
 */
struct cam_ctx_clk_info {
	uint32_t curr_fc;
	uint32_t rt_flag;
	uint32_t base_clk;
	uint32_t reserved;
	uint64_t uncompressed_bw;
	uint64_t compressed_bw;
	int32_t clk_rate[CAM_MAX_VOTE];
	uint32_t num_paths;
	struct cam_cpas_axi_per_path_bw_vote axi_path[CAM_ICP_MAX_PER_PATH_VOTES];
	bool bw_included;
};

/**
 * struct icp_cmd_generic_blob
 * @ctx: Current context info
 * @frame_info_idx: Index used for frame process info
 * @io_buf_addr: pointer to io buffer address
 */
struct icp_cmd_generic_blob {
	struct cam_icp_hw_ctx_data *ctx;
	uint32_t frame_info_idx;
	dma_addr_t *io_buf_addr;
};

/**
 * struct cam_icp_clk_info
 * @base_clk: Base clock to process request
 * @curr_clk: Current clock of hadrware
 * @prev_clk: Previous clock of hadrware
 * @threshold: Threshold for overclk count
 * @over_clked: Over clock count
 * @uncompressed_bw: Current bandwidth voting
 * @compressed_bw: Current compressed bandwidth voting
 * @num_paths: Number of AXI vote paths
 * @axi_path: Current per path bw vote info
 * @watch_dog: watchdog timer handle
 * @watch_dog_reset_counter: Counter for watch dog reset
 * @timeout_cb_data: private cb data to be used when device timeouts
 */
struct cam_icp_clk_info {
	uint32_t base_clk;
	uint32_t curr_clk;
	uint32_t prev_clk;
	uint32_t threshold;
	uint32_t over_clked;
	uint64_t uncompressed_bw;
	uint64_t compressed_bw;
	uint32_t num_paths;
	struct cam_cpas_axi_per_path_bw_vote axi_path[CAM_ICP_MAX_PER_PATH_VOTES];
	struct cam_req_mgr_timer *watch_dog;
	uint32_t watch_dog_reset_counter;
	void *timeout_cb_data;
};

/**
 * struct cam_icp_hw_device_info
 * @dev_name: name of IPE/BPS/OFE device
 * @hw_dev_type: type of IPE/BPS/OFE device
 * @dev_intf: interface to device hardware layer
 * @clk_info: clock info for the device
 * @dev_ctx_info: context related info for the device
 * @hw_dev_cnt: number of devices for this particular device type
 *              Exa - for IPE device - IPE0 and IPE1
 */
struct cam_icp_hw_device_info {
	char *dev_name;
	enum cam_icp_hw_type hw_dev_type;
	struct cam_hw_intf **dev_intf;
	struct cam_icp_clk_info clk_info;
	struct cam_icp_hw_ctx_dev_info dev_ctx_info;
	uint32_t hw_dev_cnt;
};

/**
 * struct cam_icp_hw_ctx_data
 * @context_priv: Context private data
 * @hw_mgr_priv: HW MGR of the context
 * @device_info: device info associated with this ctx
 * @ctx_mutex: Mutex for context
 * @fw_handle: Firmware handle
 * @scratch_mem_size: Scratch memory size
 * @icp_dev_acquire_info: Acquire device info
 * @ctxt_event_cb: Context callback function
 * @state: context state
 * @role: Role of a context in case of chaining
 * @chain_ctx: Peer context
 * @hfi_frame_process: Frame process command
 * @wait_complete: Completion info
 * @ctx_id: Context Id
 * @bw_config_version: BW config version indicator
 * @clk_info: Current clock info of a context
 * @watch_dog: watchdog timer handle
 * @watch_dog_reset_counter: Counter for watch dog reset
 * @last_flush_req: last flush req for this ctx
 * @ctx_id_string: string helps to identify context (primarily used for logging)
 * @perf_stats: performance statistics info
 * @evt_inject_params: Event injection data for hw_mgr_ctx
 * @abort_timed_out: Indicates if abort timed out
 */
struct cam_icp_hw_ctx_data {
	void *context_priv;
	void *hw_mgr_priv;
	struct cam_icp_hw_device_info *device_info;
	struct mutex ctx_mutex;
	uint32_t fw_handle;
	uint32_t scratch_mem_size;
	struct cam_icp_acquire_dev_info *icp_dev_acquire_info;
	cam_hw_event_cb_func ctxt_event_cb;
	uint32_t state;
	uint32_t role;
	struct cam_icp_hw_ctx_data *chain_ctx;
	struct hfi_frame_process_info hfi_frame_process;
	struct completion wait_complete;
	uint32_t ctx_id;
	uint32_t bw_config_version;
	struct cam_ctx_clk_info clk_info;
	struct cam_req_mgr_timer *watch_dog;
	uint32_t watch_dog_reset_counter;
	uint64_t last_flush_req;
	char ctx_id_string[128];
	struct cam_icp_ctx_perf_stats perf_stats;
	struct cam_hw_inject_evt_param evt_inject_params;
	bool abort_timed_out;
};

/**
 * struct cam_icp_hw_mgr
 * @hw_mgr_mutex: Mutex for ICP hardware manager
 * @hw_mgr_lock: Spinlock for ICP hardware manager
 * @dev_info: array of available device info (IPE/BPS/OFE)
 *            for the hw mgr
 * @num_dev_info: number of device info for available device for the hw mgr
 * @dev_info_idx: map hw dev type to index for device info array indexing
 * @icp_dev_intf: ICP device interface
 * @ctx_data: Context data
 * @mini_dump_cb: Mini dump cb
 * @hw_mgr_name: name of the hw mgr
 * @hw_mgr_id: ID of the hw mgr, equivalent to hw mgr index
 * @iommu_hdl: Non secure IOMMU handle
 * @iommu_sec_hdl: Secure IOMMU handle
 * @hfi_handle: hfi handle for this ICP hw mgr
 * @synx_core_id: Synx core ID if applicable
 * @hfi_mem: Memory for hfi
 * @cmd_work: Work queue for hfi commands
 * @msg_work: Work queue for hfi messages
 * @timer_work: Work queue for timer watchdog
 * @msg_buf: Drain Buffer for message data from firmware
 *           Buffer is an array of type __u32, total size
 *           would be sizeof(_u32) * queue_size
 * @dbg_buf: Drain Buffer for debug data from firmware
 * @icp_complete: Completion info
 * @cmd_work_data: Pointer to command work queue task
 * @msg_work_data: Pointer to message work queue task
 * @timer_work_data: Pointer to timer work queue task
 * @ctxt_cnt: Active context count
 * @dentry: Debugfs entry
 * @icp_debug_clk: Set clock based on debug value
 * @icp_default_clk: Set this clok if user doesn't supply
 * @secure_mode: Flag to enable/disable secure camera
 * @icp_debug_type : entry to enable FW debug message/qdss
 * @icp_dbg_lvl : debug level set to FW.
 * @icp_fw_dump_lvl : level set for dumping the FW data
 * @icp_fw_ramdump_lvl : level set for FW ram dumps
 * @recovery: Flag to validate if in previous session FW
 *            reported a fatal error or wdt. If set FW is
 *            re-downloaded for new camera session.
 * @frame_in_process: Counter for frames in process
 * @frame_in_process_ctx_id: Contxt id processing frame
 * @hw_cap_mask: device capability mask to indicate which devices type
 *               are available in this hw mgr
 * @icp_booted: Processor is booted i.e. firmware loaded
 * @icp_resumed: Processor is powered on
 * @icp_pc_flag: Flag to enable/disable power collapse
 * @dev_pc_flag: Flag to enable/disable
 *                   power collapse for ipe & bps
 * @icp_use_pil: Flag to indicate usage of PIL framework
 * @icp_jtag_debug: entry to enable ICP JTAG debugging
 * @disable_ubwc_comp: Disable UBWC compression
 * @synx_signaling_en: core to core fencing is enabled
 *                     using synx
 */
struct cam_icp_hw_mgr {
	struct mutex hw_mgr_mutex;
	spinlock_t hw_mgr_lock;

	struct cam_icp_hw_device_info *dev_info;
	uint32_t num_dev_info;
	int8_t dev_info_idx[CAM_ICP_HW_MAX];
	struct cam_hw_intf *icp_dev_intf;
	struct cam_icp_hw_ctx_data ctx_data[CAM_ICP_CTX_MAX];
	cam_icp_mini_dump_cb mini_dump_cb;
	char hw_mgr_name[CAM_ICP_HW_MGR_NAME_SIZE];
	uint32_t hw_mgr_id;

	int32_t iommu_hdl;
	int32_t iommu_sec_hdl;
	int32_t hfi_handle;
	enum cam_sync_synx_supported_cores synx_core_id;
	struct icp_hfi_mem_info hfi_mem;
	struct cam_req_mgr_core_workq *cmd_work;
	struct cam_req_mgr_core_workq *msg_work;
	struct cam_req_mgr_core_workq *timer_work;
	uint32_t msg_buf[ICP_MSG_BUF_SIZE_IN_WORDS];
	uint32_t dbg_buf[ICP_DBG_BUF_SIZE_IN_WORDS];
	struct completion icp_complete;
	struct hfi_cmd_work_data *cmd_work_data;
	struct hfi_msg_work_data *msg_work_data;
	struct hfi_msg_work_data *timer_work_data;
	uint32_t ctxt_cnt;
	struct dentry *dentry;
	uint64_t icp_debug_clk;
	uint64_t icp_default_clk;
	uint32_t secure_mode;
	u64 icp_debug_type;
	u64 icp_dbg_lvl;
	u64 icp_fw_dump_lvl;
	u32 icp_fw_ramdump_lvl;
	atomic_t recovery;
	uint64_t icp_svs_clk;
	atomic_t frame_in_process;
	int frame_in_process_ctx_id;
	uint32_t hw_cap_mask;
	bool icp_booted;
	bool icp_resumed;
	bool icp_pc_flag;
	bool dev_pc_flag;
	bool icp_use_pil;
	bool icp_jtag_debug;
	bool disable_ubwc_comp;
	bool synx_signaling_en;
};

/**
 * struct cam_icp_mini_dump_acquire_info - ICP mini dump device info
 *
 * @in_res: resource info used for clock and bandwidth calculation
 * @out_res: output resource
 * @num_out_res: number of output resources
 * @dev_type: device type (IPE_RT/IPE_NON_RT/BPS)
 * @secure_mode: camera mode (secure/non secure)
 */
struct cam_icp_mini_dump_acquire_info {
	struct cam_icp_res_info out_res[ICP_MAX_OUTPUT_SUPPORTED];
	struct cam_icp_res_info in_res;
	uint16_t                num_out_res;
	uint8_t                 dev_type;
	uint8_t                 secure_mode;
};

/**
 * struct hfi_frame_process_info
 * @request_id: Request id list
 * @num_out_res: Number of out syncs
 * @out_res: Out sync info
 * @in_resource: In sync info
 * @submit_timestamp: Submit timestamp to hw
 * @fw_process_flag: Frame process flag
 */
struct  hfi_frame_mini_dump_info {
	uint64_t                              request_id[CAM_FRAME_CMD_MAX];
	uint32_t                              num_out_res[CAM_FRAME_CMD_MAX];
	uint32_t                              out_res[CAM_FRAME_CMD_MAX][CAM_MAX_OUT_RES];
	uint32_t                              in_resource[CAM_FRAME_CMD_MAX];
	ktime_t                               submit_timestamp[CAM_FRAME_CMD_MAX];
	uint8_t                               fw_process_flag[CAM_FRAME_CMD_MAX];
};

/**
 * struct cam_icp_hw_ctx_mini_dump
 * @acquire: Acquire device info
 * @hfi_frame: Frame  command
 * @hw_ctx: Context private data
 * @state: context state
 * @ctx_id_string: Context id string
 * @ctx_id: Context Id
 */
struct cam_icp_hw_ctx_mini_dump {
	struct cam_icp_mini_dump_acquire_info acquire;
	struct hfi_frame_mini_dump_info       hfi_frame;
	void                                 *hw_ctx;
	char                                  ctx_id_string[128];
	uint8_t                               state;
	uint8_t                               ctx_id;
};

/**
 * struct cam_icp_hw_mini_dump_info
 *
 * @hw_mgr_name: name of the hw mgr that is dumped
 * @ctx: Context for minidump
 * @hfi_info: hfi info
 * @hfi_mem_info: hfi mem info
 * @dev_info: array of device info
 * @fw_img: FW image
 * @recovery: To indicate if recovery is on
 * @num_context: number of active context
 * @num_device_info: number of available device info
 * @icp_booted: Indicate if ICP is booted
 * @icp_resumed: Indicate if ICP is resumed
 * @disable_ubwc_comp: Indicate if ubws comp is disabled
 * @icp_pc_flag: Is ICP PC enabled
 * @dev_pc_flag: Is IPE BPS PC enabled
 * @icp_use_pil: Is PIL used
 */
struct cam_icp_hw_mini_dump_info {
	char                               hw_mgr_name[CAM_ICP_HW_MGR_NAME_SIZE];
	struct cam_icp_hw_ctx_mini_dump   *ctx[CAM_ICP_CTX_MAX];
	struct hfi_mini_dump_info          hfi_info;
	struct icp_hfi_mem_info            hfi_mem_info;
	struct cam_icp_hw_device_info      dev_info[CAM_ICP_DEV_NUM];
	void                              *fw_img;
	uint32_t                           recovery;
	uint32_t                           num_context;
	uint32_t                           num_device_info;
	bool                               icp_booted;
	bool                               icp_resumed;
	bool                               disable_ubwc_comp;
	bool                               icp_pc_flag;
	bool                               dev_pc_flag;
	bool                               icp_use_pil;
};

static int cam_icp_mgr_hw_close(void *hw_priv, void *hw_close_args);
static int cam_icp_mgr_hw_open(void *hw_mgr_priv, void *download_fw_args);
static int cam_icp_mgr_icp_resume(struct cam_icp_hw_mgr *hw_mgr);
static int cam_icp_mgr_icp_power_collapse(struct cam_icp_hw_mgr *hw_mgr);
#endif /* CAM_ICP_HW_MGR_H */
