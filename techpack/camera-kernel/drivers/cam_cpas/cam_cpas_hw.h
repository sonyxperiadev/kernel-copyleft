/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_CPAS_HW_H_
#define _CAM_CPAS_HW_H_

#include <dt-bindings/msm-camera.h>

#include "cam_cpas_api.h"
#include "cam_cpas_hw_intf.h"
#include "cam_common_util.h"
#include "cam_soc_bus.h"

#define CAM_CPAS_INFLIGHT_WORKS              5
#define CAM_CPAS_MAX_CLIENTS                 43
#define CAM_CPAS_MAX_AXI_PORTS               6
#define CAM_CPAS_MAX_DRV_PORTS               4
#define CAM_CPAS_MAX_TREE_LEVELS             4
#define CAM_CPAS_MAX_RT_WR_NIU_NODES         10
#define CAM_CPAS_MAX_GRAN_PATHS_PER_CLIENT   32
#define CAM_CPAS_PATH_DATA_MAX               42
#define CAM_CPAS_TRANSACTION_MAX             2
#define CAM_CAMNOC_FILL_LVL_REG_INFO_MAX     6
#define CAM_CPAS_MAX_SLOPE_FACTOR            100
#define CAM_CPAS_MAX_STRESS_INDICATOR        100

/* Number of camera (CAM_SS) instances */
#define CAM_CPAS_CAMERA_INSTANCES            1

#define CAM_CPAS_AXI_MIN_MNOC_AB_BW   (2048 * 1024)
#define CAM_CPAS_AXI_MIN_MNOC_IB_BW   (2048 * 1024)
#define CAM_CPAS_AXI_MIN_CAMNOC_AB_BW (2048 * 1024)
#define CAM_CPAS_AXI_MIN_CAMNOC_IB_BW (3000000000UL)

#define CAM_CPAS_GET_CLIENT_IDX(handle) (handle)
#define CAM_CPAS_GET_CLIENT_HANDLE(indx) (indx)

#define CAM_CPAS_WORKQUEUE_NAME "cam-cpas"

#define CAM_CPAS_CLIENT_VALID(indx) \
	((indx >= 0) && (indx < CAM_CPAS_MAX_CLIENTS))
#define CAM_CPAS_CLIENT_REGISTERED(cpas_core, indx)        \
	((CAM_CPAS_CLIENT_VALID(indx)) && \
	(cpas_core->cpas_client[indx]->registered))
#define CAM_CPAS_CLIENT_STARTED(cpas_core, indx)          \
	((CAM_CPAS_CLIENT_REGISTERED(cpas_core, indx)) && \
	(cpas_core->cpas_client[indx]->started))

/* Array indices to represent corresponding RPMH BCM info */
#define CAM_RPMH_NUMBER_OF_BCMS 0
#define CAM_RPMH_BCM_FE_OFFSET  1
#define CAM_RPMH_BCM_BE_OFFSET  2
#define CAM_RPMH_BCM_DDR_INDEX  3
#define CAM_RPMH_BCM_MNOC_INDEX 4
#define CAM_RPMH_BCM_INFO_MAX   5

#define CAM_CPAS_MONITOR_MAX_ENTRIES   100
#define CAM_CPAS_INC_MONITOR_HEAD(head, ret) \
	div_u64_rem(atomic64_add_return(1, head),\
	CAM_CPAS_MONITOR_MAX_ENTRIES, (ret))
#define CAM_CPAS_MAX_CESTA_VCD_NUM 9

#define CAM_CPAS_DUMP_NUM_WORDS_COMM              20
#define CAM_CPAS_DUMP_NUM_WORDS_VOTE_TYEP_DRV     4
#define CAM_CPAS_DUMP_NUM_WORDS_VOTE_TYEP_HLOS    2
#define CAM_CPAS_DUMP_NUM_WORDS_RT_WR_NIUS        2
#define CAM_CPAS_DUMP_NUM_WORDS_VCD_CURR_LVL      2

/**
 * enum cam_camnoc_domain_type - Enum for different camnoc domains
 * @CAM_CAMNOC_HW_COMBINED: refer to legacy camnoc info that combines RT/NRT HW
 * @CAM_CAMNOC_HW_RT: type for camnoc RT info
 * @CAM_CAMNOC_HW_NRT: type for camnoc NRT info
 * @CAM_CAMNOC_HW_TYPE_MAX: camnoc info maximum type
 */
enum cam_camnoc_hw_type {
	CAM_CAMNOC_HW_COMBINED,
	CAM_CAMNOC_HW_RT,
	CAM_CAMNOC_HW_NRT,
	CAM_CAMNOC_HW_TYPE_MAX,
};

/**
 * enum cam_cpas_access_type - Enum for Register access type
 */
enum cam_cpas_access_type {
	CAM_REG_TYPE_READ,
	CAM_REG_TYPE_WRITE,
	CAM_REG_TYPE_READ_WRITE,
};

/**
 * struct cam_cpas_vdd_ahb_mapping : Voltage to ahb level mapping
 *
 * @vdd_corner : Voltage corner value
 * @ahb_level : AHB vote level corresponds to this vdd_corner
 *
 */
struct cam_cpas_vdd_ahb_mapping {
	unsigned int vdd_corner;
	enum cam_vote_level ahb_level;
};

/**
 * struct cam_cpas_bw_vote : AXI bw vote
 *
 * @ab:     AB bw value
 * @ib:     IB bw value
 * @camnoc: CAMNOC bw value
 *
 */
struct cam_cpas_bw_vote {
	uint64_t ab;
	uint64_t ib;
	uint64_t camnoc;
};

/**
 * struct cam_cpas_drv_vote : DRV bw vote
 *
 * @high: Active bw values
 * @low:  Sleep bw values
 *
 */
struct cam_cpas_drv_vote {
	struct cam_cpas_bw_vote high;
	struct cam_cpas_bw_vote low;
};

/**
 * struct cam_cpas_axi_bw_info : AXI bw info
 *
 * @vote_type:  HLOS or DRV vote type
 * @hlos_vote: HLOS bw values
 * @drv_vote:  DRV bw values
 *
 */
struct cam_cpas_axi_bw_info {
	enum cam_cpas_vote_type vote_type;
	union {
		struct cam_cpas_bw_vote hlos_vote;
		struct cam_cpas_drv_vote drv_vote;
	};
};

/**
 * struct cam_cpas_kobj_map: wrapper structure for base kobject
 *                               and cam cpas private soc info
 * @base_kobj: kernel object for camera sysfs
 * @cpas_hw: pointer to cam_hw_info structure
 */
struct cam_cpas_kobj_map {
	struct kobject base_kobj;
	struct cam_hw_info *cpas_hw;
};

/**
 * struct cam_cpas_internal_ops - CPAS Hardware layer internal ops
 *
 * @get_hw_info: Function pointer for get hw info
 * @init_hw_version: Function pointer for hw init based on version
 * @handle_irq: Function poniter for irq handling
 * @setup_regbase: Function pointer for setup rebase indices
 * @power_on: Function pointer for hw core specific power on settings
 * @power_off: Function pointer for hw core specific power off settings
 * @setup_qos_settings: Function pointer for hw to select a specific qos header
 * @print_poweron_settings: Function pointer for hw to print poweron settings
 * @qchannel_handshake: Function pointer for hw core specific qchannel
 *                      handshake settings
 *
 */
struct cam_cpas_internal_ops {
	int (*get_hw_info)(struct cam_hw_info *cpas_hw,
		struct cam_cpas_hw_caps *hw_caps);
	int (*init_hw_version)(struct cam_hw_info *cpas_hw,
		struct cam_cpas_hw_caps *hw_caps);
	irqreturn_t (*handle_irq)(int irq_num, void *data);
	int (*setup_regbase)(struct cam_hw_soc_info *soc_info,
		int32_t regbase_index[], int32_t num_reg_map);
	int (*power_on)(struct cam_hw_info *cpas_hw);
	int (*power_off)(struct cam_hw_info *cpas_hw);
	int (*setup_qos_settings)(struct cam_hw_info *cpas_hw,
		uint32_t selection_mask);
	int (*print_poweron_settings)(struct cam_hw_info *cpas_hw);
	int (*qchannel_handshake)(struct cam_hw_info *cpas_hw, bool power_on, bool force_on);
};

/**
 * struct cam_cpas_reg : CPAS register info
 *
 * @enable: Whether this reg info need to be enabled
 * @access_type: Register access type
 * @masked_value: Whether this register write/read is based on mask, shift
 * @mask: Mask for this register value
 * @shift: Shift for this register value
 * @value: Register value
 *
 */
struct cam_cpas_reg {
	bool enable;
	enum cam_cpas_access_type access_type;
	bool masked_value;
	uint32_t offset;
	uint32_t mask;
	uint32_t shift;
	uint32_t value;
};

/**
 * struct cam_cpas_client : CPAS Client structure info
 *
 * @data: Client register params
 * @registered: Whether client has registered with cpas
 * @started: Whether client has streamed on
 * @tree_node_valid: Indicates whether tree node has at least one valid node
 * @is_drv_dyn: Indicates whether this client is DRV dynamic voting client
 * @ahb_level: Determined/Applied ahb level for the client
 * @axi_vote: Determined/Applied axi vote for the client
 * @axi_port: Client's parent axi port
 * @tree_node: All granular path voting nodes for the client
 *
 */
struct cam_cpas_client {
	struct cam_cpas_register_params data;
	bool registered;
	bool started;
	bool tree_node_valid;
	bool is_drv_dyn;
	enum cam_vote_level ahb_level;
	struct cam_axi_vote axi_vote;
	struct cam_cpas_axi_port *axi_port;
	struct cam_cpas_tree_node *tree_node[CAM_CPAS_PATH_DATA_MAX]
		[CAM_CPAS_TRANSACTION_MAX];
};

/**
 * struct cam_cpas_bus_client : Bus client information
 *
 * @valid: Whether bus client is valid
 * @name: Name of the bus client
 * @lock: Mutex lock used while voting on this client
 * @curr_vote_level: current voted index
 * @common_data: Common data fields for bus client
 * @soc_bus_client: Bus client private information
 */
struct cam_cpas_bus_client {
	bool valid;
	struct mutex lock;
	unsigned int curr_vote_level;
	struct cam_soc_bus_client_common_data common_data;
	void *soc_bus_client;
};

/**
 * struct cam_cpas_axi_port : AXI port information
 *
 * @axi_port_name: Name of this AXI port
 * @bus_client: bus client info for this port
 * @ib_bw_voting_needed: if this port can update ib bw dynamically
 * @is_rt: if this port represents a real time axi port
 * @axi_port_node: Node representing AXI Port info in device tree
 * @drv_idx: DRV index for axi port node
 * @cam_rsc_dev: Cam RSC device for DRV
 * @is_drv_started: Indicates if DRV started for RSC device corresponding to port
 * @curr_bw: Current voted bw after cpas consolidation
 * @additional_bw: Additional bandwidth to cover non-hw cpas clients
 * @applied_bw: Actual applied bw to port
 */
struct cam_cpas_axi_port {
	const char *axi_port_name;
	struct cam_cpas_bus_client bus_client;
	bool ib_bw_voting_needed;
	bool is_rt;
	struct device_node *axi_port_node;
	uint32_t drv_idx;
	const struct device *cam_rsc_dev;
	bool is_drv_started;
	struct cam_cpas_axi_bw_info curr_bw;
	uint64_t additional_bw;
	struct cam_cpas_axi_bw_info applied_bw;
};

/**
 * struct cam_cpas_axi_port_debug_info : AXI port information
 *
 * @axi_port_name: Name of this AXI port
 * @curr_bw: Current voted bw after cpas consolidation
 * @camnoc_bw: CAMNOC bw value for this port
 * @applied_bw: Actual applied bw to port
 * @is_drv_started: Indicates if DRV started for RSC device corresponding to port
 */
struct cam_cpas_axi_port_debug_info {
	const char *axi_port_name;
	struct cam_cpas_axi_bw_info curr_bw;
	uint64_t camnoc_bw;
	struct cam_cpas_axi_bw_info applied_bw;
	bool is_drv_started;
};

struct cam_cpas_cesta_vcd_curr_lvl_debug_info {
	uint8_t index;
	uint32_t reg_value;
};

/**
 * struct cam_cpas_cesta_vcd_reg_debug_info : to hold all cesta register information
 *
 * @vcd_currol: vcd control reg info
 *
 */
struct cam_cpas_cesta_vcd_reg_debug_info {
	struct cam_cpas_cesta_vcd_curr_lvl_debug_info
		vcd_curr_lvl_debug_info[CAM_CPAS_MAX_CESTA_VCD_NUM];
};


/**
 * struct cam_cpas_monitor : CPAS monitor array
 *
 * @timestamp: Timestamp at which this monitor entry is saved
 * @identifier_string: String passed by caller
 * @identifier_value: Identifier value passed by caller
 * @axi_info: AXI port information
 * @applied_camnoc_clk: Applied camnoc axi clock rate with sw, hw clients
 * @applied_ahb_level: Applied camcc ahb level
 * @fe_ddr: RPMH DDR BCM FE (front-end) status register value.
 *          This indicates requested clock plan
 * @be_ddr: RPMH DDR BCM BE (back-end) status register value.
 *          This indicates actual current clock plan
 * @fe_mnoc: RPMH MNOC BCM FE (front-end) status register value.
 *           This indicates requested clock plan
 * @be_mnoc: RPMH MNOC BCM BE (back-end) status register value.
 *           This indicates actual current clock plan
 * @be_shub: RPMH SHUB BCM BE (back-end) status register value.
 *           This indicates actual current clock plan
 * @num_camnoc_lvl_regs: Number of enabled camnoc fill level
 *           monitoring registers
 * @camnoc_port_name: Camnoc port names
 * @camnoc_fill_level: Camnoc fill level register info
 * @rt_wr_niu_pri_lut_low: priority lut low values of RT Wr NIUs
 * @rt_wr_niu_pri_lut_high: priority lut high values of RT Wr NIUs
 * @vcd_reg_debug_info: vcd reg debug information
 */
struct cam_cpas_monitor {
	struct timespec64   timestamp;
	char                identifier_string[128];
	int32_t             identifier_value;
	struct cam_cpas_axi_port_debug_info axi_info[CAM_CPAS_MAX_AXI_PORTS];
	struct cam_soc_util_clk_rates       applied_camnoc_clk;
	unsigned int        applied_ahb_level;
	uint32_t            fe_ddr;
	uint32_t            be_ddr;
	uint32_t            fe_mnoc;
	uint32_t            be_mnoc;
	uint32_t            be_shub;
	uint32_t            num_camnoc_lvl_regs[CAM_CAMNOC_HW_TYPE_MAX];
	const char          *camnoc_port_name[CAM_CAMNOC_HW_TYPE_MAX]
		[CAM_CAMNOC_FILL_LVL_REG_INFO_MAX];
	uint32_t            camnoc_fill_level[CAM_CAMNOC_HW_TYPE_MAX]
		[CAM_CAMNOC_FILL_LVL_REG_INFO_MAX];
	uint32_t            rt_wr_niu_pri_lut_low[CAM_CPAS_MAX_RT_WR_NIU_NODES];
	uint32_t            rt_wr_niu_pri_lut_high[CAM_CPAS_MAX_RT_WR_NIU_NODES];
	struct cam_cpas_cesta_vcd_reg_debug_info vcd_reg_debug_info;
	struct cam_hw_info  *cpas_hw;
};

/**
 * struct cam_cpas : CPAS core data structure info
 *
 * @hw_caps: CPAS hw capabilities
 * @cpas_client: Array of pointers to CPAS clients info
 * @client_mutex: Mutex for accessing client info
 * @tree_lock: Mutex lock for accessing CPAS node tree
 * @num_clients: Total number of clients that CPAS supports
 * @num_axi_ports: Total number of axi ports found in device tree
 * @num_camnoc_axi_ports: Total number of camnoc axi ports found in device tree
 * @registered_clients: Number of Clients registered currently
 * @streamon_clients: Number of Clients that are in start state currently
 * @slave_err_irq_idx: Index of slave error in irq error data structure,
 *                     avoids iterating the entire structure to find this
 *                     idx in irq th
 * @regbase_index: Register base indices for CPAS register base IDs
 * @ahb_bus_client: AHB Bus client info
 * @axi_port: AXI port info for a specific axi index
 * @camnoc_axi_port: CAMNOC AXI port info for a specific camnoc axi index
 * @cam_subpart_info: camera subparts fuse description
 * @internal_ops: CPAS HW internal ops
 * @work_queue: Work queue handle
 * @soc_access_count: atomic soc_access_count count
 * @soc_access_count_wq: wait variable to ensure CPAS is not stop,
 *						 while accessing hw through CPAS
 * @dentry: debugfs file entry
 * @ahb_bus_scaling_disable: ahb scaling based on src clk corner for bus
 * @applied_camnoc_axi_rate: applied camnoc axi clock rate through sw, hw clients
 * @monitor_head: Monitor array head
 * @monitor_entries: cpas monitor array
 * @camnoc_info: array of camnoc info pointer
 * @cesta_info: Pointer to cesta header info
 * @num_valid_camnoc: number of valid camnoc info
 * @camnoc_rt_idx: index to real time camnoc info array
 * @camnoc_info_idx: map camnoc hw type to index used for camnoc_info array indexing
 * @full_state_dump: Whether to enable full cpas state dump or not
 * @smart_qos_dump: Whether to dump smart qos information on update
 * @slave_err_irq_en: Whether slave error irq is enabled to detect memory
 *                    config issues
 * @smmu_fault_handled: Handled address decode error, on fault at SMMU
 * @force_hlos_drv: Whether to force disable DRV voting
 * @force_cesta_sw_client: Whether to force voting through cesta sw client
 */
struct cam_cpas {
	struct cam_cpas_hw_caps hw_caps;
	struct cam_cpas_client *cpas_client[CAM_CPAS_MAX_CLIENTS];
	struct mutex client_mutex[CAM_CPAS_MAX_CLIENTS];
	struct mutex tree_lock;
	uint32_t num_clients;
	uint32_t num_axi_ports;
	uint32_t num_camnoc_axi_ports;
	uint32_t registered_clients;
	uint32_t streamon_clients;
	uint32_t slave_err_irq_idx[CAM_CAMNOC_HW_TYPE_MAX];
	int32_t regbase_index[CAM_CPAS_REG_MAX];
	struct cam_cpas_bus_client ahb_bus_client;
	struct cam_cpas_axi_port axi_port[CAM_CPAS_MAX_AXI_PORTS];
	struct cam_cpas_axi_port camnoc_axi_port[CAM_CPAS_MAX_AXI_PORTS];
	struct cam_cpas_subpart_info *cam_subpart_info;
	struct cam_cpas_internal_ops internal_ops;
	struct workqueue_struct *work_queue;
	atomic_t soc_access_count;
	wait_queue_head_t soc_access_count_wq;
	struct dentry *dentry;
	bool ahb_bus_scaling_disable;
	struct cam_soc_util_clk_rates applied_camnoc_axi_rate;
	atomic64_t  monitor_head;
	struct cam_cpas_monitor monitor_entries[CAM_CPAS_MONITOR_MAX_ENTRIES];
	void *camnoc_info[CAM_CAMNOC_HW_TYPE_MAX];
	void *cesta_info;
	uint8_t num_valid_camnoc;
	int8_t camnoc_rt_idx;
	int8_t camnoc_info_idx[CAM_CAMNOC_HW_TYPE_MAX];
	bool full_state_dump;
	bool smart_qos_dump;
	bool slave_err_irq_en[CAM_CAMNOC_HW_TYPE_MAX];
	bool smmu_fault_handled;
	bool force_hlos_drv;
	bool force_cesta_sw_client;
};

int cam_camsstop_get_internal_ops(struct cam_cpas_internal_ops *internal_ops);
int cam_cpastop_get_internal_ops(struct cam_cpas_internal_ops *internal_ops);

int cam_cpas_util_reg_update(struct cam_hw_info *cpas_hw,
	enum cam_cpas_reg_base reg_base, struct cam_cpas_reg *reg_info);
int cam_cpas_util_reg_read(struct cam_hw_info *cpas_hw,
	enum cam_cpas_reg_base reg_base, struct cam_cpas_reg *reg_info);

int cam_cpas_util_client_cleanup(struct cam_hw_info *cpas_hw);

int cam_cpas_util_vote_default_ahb_axi(struct cam_hw_info *cpas_hw,
	int enable);

#endif /* _CAM_CPAS_HW_H_ */
