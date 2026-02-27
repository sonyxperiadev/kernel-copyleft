/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_CPAS_SOC_H_
#define _CAM_CPAS_SOC_H_

#include <linux/soc/qcom/llcc-qcom.h>
#include "cam_soc_util.h"
#include "cam_cpas_hw.h"

#define CAM_REGULATOR_LEVEL_MAX 16
#define CAM_CPAS_MAX_TREE_NODES 63
#define CAM_CPAS_MAX_FUSE_FEATURE 10

/**
 * enum cam_cpas_num_subparts_types - Enum for types of number of camera subparts
 */
enum cam_cpas_num_subparts_types {
	CAM_CPAS_AVAILABLE_NUM_SUBPARTS,
	CAM_CPAS_FUNCTIONAL_NUM_SUBPARTS,
	CAM_CPAS_NUM_SUBPARTS_MAX_TYPES,
};

/**
 * struct cpas_tree_node: Generic cpas tree node for BW voting
 *
 * @cell_idx: Index to identify node from device tree and its parent
 * @level_idx: Index to identify at what level the node is present
 * @axi_port_idx_arr: Index to identify which axi port to vote the consolidated bw.
 *                    It can point to multiple indexes in case of camera DRV
 * @drv_voting_idx: Specifies the index to which the child node would finally vote.
 * @camnoc_axi_port_idx: Index to find which axi port to vote consolidated bw
 * @path_data_type: Traffic type info from device tree (ife-vid, ife-disp etc)
 * @path_trans_type: Transaction type info from device tree (rd, wr)
 * @merge_type: Traffic merge type (calculation info) from device tree
 * @bus_width_factor: Factor for accounting bus width in CAMNOC bw calculation
 * @bw_info: AXI BW info for all drv ports
 * @camnoc_max_needed: If node is needed for CAMNOC BW calculation then true
 * @constituent_paths: Constituent paths presence info from device tree
 *     Ex: For CAM_CPAS_PATH_DATA_IFE_UBWC_STATS, index corresponding to
 *     CAM_CPAS_PATH_DATA_IFE_VID, CAM_CPAS_PATH_DATA_IFE_DISP and
 *     CAM_CPAS_PATH_DATA_IFE_STATS
 * @tree_dev_node: Device node from devicetree for current tree node
 * @parent_node: Pointer to node one or more level above the current level
 *     (starting from end node of cpas client)
 * @pri_lut_low_offset: Register offset value for priority lut low.
 *                           Valid only for level1 nodes (representing NIUs)
 * @pri_lut_high_offset: Register offset value for priority lut high.
 *                           Valid only for level1 nodes (representing NIUs)
 * @niu_size: Size of NIU that this node represents. Size in KB
 * @curr_priority_low:     New calculated priority lut low values
 * @curr_priority_high:    New calculated priority lut high values
 * @applied_priority_low:  Currently applied priority lut low values
 * @applied_priority_high: Currently applied priority lut high values
 *
 */
struct cam_cpas_tree_node {
	uint32_t cell_idx;
	int level_idx;
	int *axi_port_idx_arr;
	int drv_voting_idx;
	int camnoc_axi_port_idx;
	const char *node_name;
	uint32_t path_data_type;
	uint32_t path_trans_type;
	uint32_t merge_type;
	uint32_t bus_width_factor;
	struct cam_cpas_axi_bw_info *bw_info;
	bool camnoc_max_needed;
	bool constituent_paths[CAM_CPAS_PATH_DATA_MAX];
	struct device_node *tree_dev_node;
	struct cam_cpas_tree_node *parent_node;
	uint32_t pri_lut_low_offset;
	uint32_t pri_lut_high_offset;
	uint32_t niu_size;
	uint32_t curr_priority_low;
	uint32_t curr_priority_high;
	uint32_t applied_priority_low;
	uint32_t applied_priority_high;
};

/**
 * struct cam_cpas_feature_info : CPAS fuse feature info
 * @feature: Identifier for feature
 * @type: Type of feature
 * @value: Fuse value
 * @enable: Feature enable or disable
 * @hw_map: Each bit position indicates if the hw_id for the feature
 */

struct cam_cpas_feature_info {
	uint32_t feature;
	uint32_t type;
	uint32_t value;
	bool enable;
	uint32_t hw_map;
};

/**
 * struct cam_sys_cache_local_info : camera cache info saving locally
 *
 * @type:      cache type small/large etc.
 * @staling_distance:       staling_distance
 * @mode:      camera llc's stalling mode
 * @op_type:      cache operation type EVICT, FORGET
 */
struct cam_sys_cache_local_info {
	enum cam_sys_cache_config_types  type;
	uint32_t staling_distance;
	enum cam_sys_cache_llcc_staling_mode mode;
	enum cam_sys_cache_llcc_staling_op_type op_type;
};

/**
 * struct cam_sys_cache_info : Last level camera cache info
 *
 * @ref_cnt:   Ref cnt activate/deactivate cache
 * @type:      cache type small/large etc.
 * @uid:       Client user ID
 * @size:      Cache size
 * @scid:      Slice ID
 * @slic_desc: Slice descriptor
 * @staling_distance:       staling_distance
 * @mode:      camera llc's stalling mode
 * @op_type:      cache operation type EVICT, FORGET
 */
struct cam_sys_cache_info {
	uint32_t                         ref_cnt;
	enum cam_sys_cache_config_types  type;
	uint32_t                         uid;
	size_t                           size;
	int32_t                          scid;
	const char                      *name;
	struct llcc_slice_desc          *slic_desc;
	uint32_t staling_distance;
	enum cam_sys_cache_llcc_staling_mode mode;
	enum cam_sys_cache_llcc_staling_op_type op_type;

};


/**
 * struct cam_cpas_smart_qos_info : Smart QOS info
 *
 * @rt_wr_priority_min:      Minimum priority value for rt write nius
 * @rt_wr_priority_max:      Maximum priority value for rt write nius
 * @rt_wr_priority_clamp:    Clamp priority value for rt write nius
 * @rt_wr_slope_factor:      Slope factor value for rt write nius
 * @leaststressed_clamp_th:  Leaststressed clamp threshold value for rt write nius
 * @moststressed_clamp_th:   Moststressed clamp threshold value for rt write nius
 * @highstress_indicator_th: Highstress indicator threshold value for rt write nius
 * @lowstress_indicator_th:  Lowstress indicator threshold value for rt write nius
 * @bw_ratio_scale_factor:   BW ratio scale factor value for rt write nius
 * @num_rt_wr_nius:          Number of rt write nius
 * @rt_wr_niu_node:          List of level1 nodes representing rt write nius
 */
struct cam_cpas_smart_qos_info {
	uint8_t rt_wr_priority_min;
	uint8_t rt_wr_priority_max;
	uint8_t rt_wr_priority_clamp;
	uint8_t rt_wr_slope_factor;
	uint8_t leaststressed_clamp_th;
	uint8_t moststressed_clamp_th;
	uint8_t highstress_indicator_th;
	uint8_t lowstress_indicator_th;
	uint8_t bw_ratio_scale_factor;
	uint8_t num_rt_wr_nius;
	struct cam_cpas_tree_node *rt_wr_niu_node[CAM_CPAS_MAX_RT_WR_NIU_NODES];
};

/**
 * struct cam_cpas_domain_id_mapping : Domain id mapping
 *
 * @domain_type: Domain type, currently defined as two,
 *               secure/non-secure. This will be expanded
 *               later to more types, and correspnding ID
 * @mapping_id: ID of domain type
 */
struct cam_cpas_domain_id_mapping {
	uint32_t domain_type;
	uint32_t mapping_id;
};

/**
 * struct cam_cpas_domain_id_info : Stores all information related
 *                                  to domain-id support
 * @domain_id_entries: Stores mapping between domain types and their IDs
 * @num_domain_ids: Num of domain id types found from dtsi
 * @domain_id_supported: Whether domain id is supported
 */
struct cam_cpas_domain_id_info {
	struct cam_cpas_domain_id_mapping *domain_id_entries;
	uint32_t num_domain_ids;
	bool domain_id_supported;
};

/**
 * struct cam_cpas_domain_id_support_clks : Stores all information
 *                                          related to clocks
 *                                          needed to turn on SWIs
 *                                          for domain id programming
 * @clk_names:   Clock names as declared in DT
 * @clk_idx:     Corresponding clk index as declared in DT
 * @number_clks: Number of clocks declared to turn all CSIDs
 */
struct cam_cpas_domain_id_support_clks {
	const char *clk_names[CAM_SOC_MAX_OPT_CLK];
	int32_t clk_idx[CAM_SOC_MAX_OPT_CLK];
	int number_clks;
};

/**
 * struct cam_cpas_soc_irq_data: irq data to be passed in irq handler from ISR
 *
 * @cpas_hw: cpas hw info
 * @camnoc_type: type of camnoc associated with the irq
 *
 */
struct cam_cpas_soc_irq_data {
	struct cam_hw_info *cpas_hw;
	enum cam_camnoc_hw_type camnoc_type;
};

/**
 * struct cam_cpas_sysfs_info - cpas sysfs info
 *
 * @kobj:          Kobj for camera directory
 * @num_ifes:      Number of available and functional IFEs
 * @num_ife_lites: Number of available and functional IFE-LITEs
 * @num_sfes:      Number of available and functional SFEs
 * @num_custom:    Number of available and functional CUSTOM
 */
struct cam_cpas_sysfs_info {
	struct kobject *kobj;
	uint32_t        num_ifes[CAM_CPAS_NUM_SUBPARTS_MAX_TYPES];
	uint32_t        num_ife_lites[CAM_CPAS_NUM_SUBPARTS_MAX_TYPES];
	uint32_t        num_sfes[CAM_CPAS_NUM_SUBPARTS_MAX_TYPES];
	uint32_t        num_custom[CAM_CPAS_NUM_SUBPARTS_MAX_TYPES];
};

/**
 * struct cam_cpas_private_soc : CPAS private DT info
 *
 * @arch_compat: ARCH compatible string
 * @client_id_based: Whether clients are id based
 * @bus_icc_based: Interconnect based bus interaction
 * @num_clients: Number of clients supported
 * @client_name: Client names
 * @tree_node: Array of pointers to all tree nodes required to calculate
 *      axi bw, arranged with help of cell index in device tree
 * @camera_bus_node: Device tree node from cpas node
 * @level_node: Device tree node for each level in camera_bus_node
 * @num_vdd_ahb_mapping : Number of vdd to ahb level mapping supported
 * @vdd_ahb : AHB level mapping info for the supported vdd levels
 * @control_camnoc_axi_clk : Whether CPAS driver need to set camnoc axi clk freq
 * @camnoc_bus_width : CAMNOC Bus width
 * @camnoc_axi_clk_bw_margin : BW Margin in percentage to add while calculating
 *      camnoc axi clock
 * @camnoc_axi_min_ib_bw: Min camnoc BW which varies based on target
 * @fuse_info: fuse information
 * @sysfs_info: Camera subparts sysfs information
 * @rpmh_info: RPMH BCM info
 * @num_feature_info: number of feature_info entries
 * @feature_info: Structure for storing feature information
 * @num_caches: Number of last level caches
 * @part_info: Camera Hw subpart info
 * @llcc_info: Cache info
 * @enable_smart_qos: Whether to enable Smart QoS mechanism on current chipset
 * @enable_cam_ddr_drv: Whether to enable Camera DDR DRV on current chipset
 * @enable_cam_clk_drv: Whether to enable Camera Clk DRV on current chipset
 * @smart_qos_info: Pointer to smart qos info
 * @icp_clk_index: Index of optional icp clk
 * @domain_id_info: Stores all information related to domain id support
 * @domain_id_clks: All clock related information for domain id support
 * @irq_data: array of data for each irq line to be passed in irq handler
 */
struct cam_cpas_private_soc {
	const char *arch_compat;
	bool client_id_based;
	bool bus_icc_based;
	uint32_t num_clients;
	const char *client_name[CAM_CPAS_MAX_CLIENTS];
	struct cam_cpas_tree_node *tree_node[CAM_CPAS_MAX_TREE_NODES];
	struct device_node *camera_bus_node;
	struct device_node *level_node[CAM_CPAS_MAX_TREE_LEVELS];
	uint32_t num_vdd_ahb_mapping;
	struct cam_cpas_vdd_ahb_mapping vdd_ahb[CAM_REGULATOR_LEVEL_MAX];
	bool control_camnoc_axi_clk;
	uint32_t camnoc_bus_width;
	uint32_t camnoc_axi_clk_bw_margin;
	uint64_t camnoc_axi_min_ib_bw;
	struct cam_cpas_fuse_info fuse_info;
	struct cam_cpas_sysfs_info sysfs_info;
	uint32_t rpmh_info[CAM_RPMH_BCM_INFO_MAX];
	uint32_t num_feature_info;
	struct cam_cpas_feature_info  feature_info[CAM_CPAS_MAX_FUSE_FEATURE];
	uint32_t num_caches;
	uint32_t part_info;
	struct cam_sys_cache_info *llcc_info;
	bool enable_smart_qos;
	bool enable_cam_ddr_drv;
	bool enable_cam_clk_drv;
	struct cam_cpas_smart_qos_info *smart_qos_info;
	int32_t icp_clk_index;
	struct cam_cpas_domain_id_info domain_id_info;
	struct cam_cpas_domain_id_support_clks *domain_id_clks;
	struct cam_cpas_soc_irq_data *irq_data;
};

void cam_cpas_dump_tree_vote_info(struct cam_hw_info *cpas_hw,
	const struct cam_cpas_tree_node *tree_node,
	const char *identifier, int ddr_drv_idx, int cesta_drv_idx);
void cam_cpas_dump_full_tree_state(struct cam_hw_info *cpas_hw, const char *identifier);

void cam_cpas_util_debug_parse_data(struct cam_cpas_private_soc *soc_private);
void cam_cpas_dump_axi_vote_info(
	const struct cam_cpas_client *cpas_client,
	const char *identifier,
	struct cam_axi_vote *axi_vote);
int cam_cpas_node_tree_cleanup(struct cam_cpas *cpas_core,
	struct cam_cpas_private_soc *soc_private);
int cam_cpas_soc_init_resources(struct cam_hw_soc_info *soc_info,
	irq_handler_t vfe_irq_handler, struct cam_hw_info *cpas_hw);
int cam_cpas_soc_deinit_resources(struct cam_hw_soc_info *soc_info);
int cam_cpas_soc_enable_resources(struct cam_hw_soc_info *soc_info,
	enum cam_vote_level default_level);
int cam_cpas_soc_disable_resources(struct cam_hw_soc_info *soc_info,
	bool disable_clocks, bool disable_irq);
int cam_cpas_soc_disable_irq(struct cam_hw_soc_info *soc_info);
#endif /* _CAM_CPAS_SOC_H_ */
