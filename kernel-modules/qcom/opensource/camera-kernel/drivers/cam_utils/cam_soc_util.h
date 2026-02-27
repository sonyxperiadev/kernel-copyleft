/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_SOC_UTIL_H_
#define _CAM_SOC_UTIL_H_

#include <linux/version.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/debugfs.h>
#include <linux/of_fdt.h>

#include "cam_io_util.h"
#include "cam_debug_util.h"
#include <media/cam_defs.h>

#if IS_REACHABLE(CONFIG_MSM_MMRM)
#include <linux/soc/qcom/msm_mmrm.h>
#endif

#define NO_SET_RATE  -1
#define INIT_RATE    -2

/* maximum number of device block */
#define CAM_SOC_MAX_BLOCK           8

/* maximum number of device base */
#define CAM_SOC_MAX_BASE            CAM_SOC_MAX_BLOCK

/* maximum number of device regulator */
#define CAM_SOC_MAX_REGULATOR       10

/* maximum number of device clock */
#define CAM_SOC_MAX_CLK             32

/* maximum number of optional device clock */
#define CAM_SOC_MAX_OPT_CLK    7

/* maximum number of pinctrl mapping */
#define CAM_SOC_MAX_PINCTRL_MAP     2

/* maximum number of irq per device */
#define CAM_SOC_MAX_IRQ_LINES_PER_DEV 2

/* DDR device types */
#define DDR_TYPE_LPDDR4        6
#define DDR_TYPE_LPDDR4X       7
#define DDR_TYPE_LPDDR5        8
#define DDR_TYPE_LPDDR5X       9

/* Maximum length of tag while dumping */
#define CAM_SOC_HW_DUMP_TAG_MAX_LEN 128

/* Client index to be used to vote clk frequency through sw client */
#define CAM_CLK_SW_CLIENT_IDX -1

/**
 * enum cam_vote_level - Enum for voting level
 *
 * @CAM_SUSPEND_VOTE   : Suspend vote
 * @CAM_MINSVS_VOTE    : Min SVS vote
 * @CAM_LOWSVS_D1_VOTE : Low SVS D1 vote
 * @CAM_LOWSVS_VOTE    : Low SVS vote
 * @CAM_SVS_VOTE       : SVS vote
 * @CAM_SVSL1_VOTE     : SVS Plus vote
 * @CAM_NOMINAL_VOTE   : Nominal vote
 * @CAM_NOMINALL1_VOTE : Nominal plus vote
 * @CAM_TURBO_VOTE     : Turbo vote
 * @CAM_MAX_VOTE       : Max voting level, This is invalid level.
 */
enum cam_vote_level {
	CAM_SUSPEND_VOTE,
	CAM_MINSVS_VOTE,
	CAM_LOWSVS_D1_VOTE,
	CAM_LOWSVS_VOTE,
	CAM_SVS_VOTE,
	CAM_SVSL1_VOTE,
	CAM_NOMINAL_VOTE,
	CAM_NOMINALL1_VOTE,
	CAM_TURBO_VOTE,
	CAM_MAX_VOTE,
};

/* pinctrl states */
#define CAM_SOC_PINCTRL_STATE_SLEEP "cam_suspend"
#define CAM_SOC_PINCTRL_STATE_DEFAULT "cam_default"

#define CAM_CESTA_MAX_CLIENTS       3
#define CAM_NUM_PWR_STATES          2

/**
 * struct cam_soc_util_hw_client_clk_rates:   Information about HW client clock vote
 *
 * @high:               HW client clock vote high value
 * @low:                HW client clock vote low value
 **/
struct cam_soc_util_hw_client_clk_rates {
	unsigned long high;
	unsigned long low;
};

/**
 * struct cam_soc_util_clk_rates:   Information about clock vote for SW and HW clients
 *
 * @sw_client:               SW client clock vote
 * @hw_client:               HW client clock vote
 **/
struct cam_soc_util_clk_rates {
	unsigned long sw_client;
	struct cam_soc_util_hw_client_clk_rates hw_client[CAM_CESTA_MAX_CLIENTS];
};

/**
 * struct cam_soc_reg_map:   Information about the mapped register space
 *
 * @mem_base:               Starting location of MAPPED register space
 * @mem_cam_base:           Starting offset of this register space compared
 *                          to ENTIRE Camera register space
 * @size:                   Size of register space
 **/
struct cam_soc_reg_map {
	void __iomem                   *mem_base;
	uint32_t                        mem_cam_base;
	resource_size_t                 size;
};

/**
 * struct cam_soc_pinctrl_state:   Information about pinctrl state
 *
 * @gpio_state_active:     default pinctrl state
 * @gpio_state_suspend:    suspend state of pinctrl
 * @is_active:             to identify if pinctrl is in use.
 **/
struct cam_soc_pinctrl_state {
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	bool is_active;
};

/**
 * struct cam_soc_pinctrl_info:   Information about pinctrl data
 *
 * @pinctrl:               pintrl object
 * @pctrl_state:           pinctrl state montior map
 **/
struct cam_soc_pinctrl_info {
	struct pinctrl *pinctrl;
	struct cam_soc_pinctrl_state pctrl_state[
		CAM_SOC_MAX_PINCTRL_MAP];
};

/**
 * struct cam_soc_gpio_data:   Information about the gpio pins
 *
 * @cam_gpio_common_tbl:       It is list of al the gpios present in gpios node
 * @cam_gpio_common_tbl_size:  It is equal to number of gpios prsent in
 *                             gpios node in DTSI
 * @cam_gpio_req_tbl            It is list of al the requesetd gpios
 * @cam_gpio_req_tbl_size:      It is size of requested gpios
 **/
struct cam_soc_gpio_data {
	struct gpio *cam_gpio_common_tbl;
	uint8_t cam_gpio_common_tbl_size;
	struct gpio *cam_gpio_req_tbl;
	uint8_t cam_gpio_req_tbl_size;
};

/**
 * struct cam_hw_soc_info:  Soc information pertaining to specific instance of
 *                          Camera hardware driver module
 *
 * @pdev:                   Platform device pointer
 * @device:                 Device pointer
 * @hw_version:             Camera device version
 * @index:                  Instance id for the camera device
 * @dev_name:               Device Name
 * @is_nrt_dev:             Whether this is a non-real time device
 * @irq_name:               Array of irq name associated with the device
 * @label_name:             label name
 * @irq_line:               Array of Irq resources
 * @irq_num:                Array of Irq numbers
 * @irq_data:               Array of Irq Private data that are passed when IRQs are requested
 * @irq_count:              The number of IRQ lines associated with the device
 * @compatible:             Compatible string associated with the device
 * @num_mem_block:          Number of entry in the "reg-names"
 * @mem_block_name:         Array of the reg block name
 * @mem_block_cam_base:     Array of offset of this register space compared
 *                          to ENTIRE Camera register space
 * @mem_block:              Associated resource structs
 * @reg_map:                Array of Mapped register info for the "reg-names"
 * @num_reg_map:            Number of mapped register space associated
 *                          with mem_block. num_reg_map = num_mem_block in
 *                          most cases
 * @reserve_mem:            Whether to reserve memory for Mem blocks
 * @num_rgltr:              Number of regulators
 * @rgltr_name:             Array of regulator names
 * @rgltr_ctrl_support:     Whether regulator control is supported
 * @rgltr_min_volt:         Array of minimum regulator voltage
 * @rgltr_max_volt:         Array of maximum regulator voltage
 * @rgltr_op_mode:          Array of regulator operation mode
 * @rgltr_type:             Array of regulator names
 * @rgltr:                  Array of associated regulator resources
 * @rgltr_delay:            Array of regulator delay values
 * @num_clk:                Number of clocks
 * @clk_name:               Array of clock names
 * @clk:                    Array of associated clock resources
 * @clk_rate:               2D array of clock rates representing clock rate
 *                          values at different vote levels
 * @clk_id                  Clock IDs
 * @shared_clk_mask         Mask indicating which of the clocks are shared with
 *                          other devices. Set rate on these clocks needs to go
 *                          through camera clk wrapper for aggregation.
 * @prev_clk_level          Last vote level
 * @src_clk_idx:            Source clock index that is rate-controllable
 * @applied_src_clk_rates:  Applied src clock rates for SW and HW client
 * @clk_level_valid:        Indicates whether corresponding level is valid
 * @lowest_clk_level:       Lowest clock level that has valid freq info
 * @scl_clk_count:          Number of scalable clocks present
 * @scl_clk_idx:            Index of scalable clocks
 * @optional_clk_name:      Array of clock names
 * @optional_clk:           Array of associated clock resources
 * @optional_clk_rate:      Optional clock's clk rate
 * @optional_clk_id         Clock IDs
 * @optional_shared_clk_mask Mask indicating which of the clocks are shared with
 *                           other devices. Set rate on these clocks needs to go
 *                           through camera clk wrapper for aggregation.
 * @gpio_data:              Pointer to gpio info
 * @mmrm_handle:            MMRM Client handle for src clock
 * @is_clk_drv_en:          If clock drv is enabled in hw
 * @pinctrl_info:           Pointer to pinctrl info
 * @dentry:                 Debugfs entry
 * @clk_level_override_high:Clk level set from debugfs. When cesta is enabled, used to override
 *                          high clk value
 * @clk_level_override_high:Low clk level set from debugfs when cesta is enabled, used to override
 *                          low clk value
 * @clk_control:            Enable/disable clk rate control through debugfs
 * @cam_cx_ipeak_enable     cx-ipeak enable/disable flag
 * @cam_cx_ipeak_bit        cx-ipeak mask for driver
 * @soc_private:            Soc private data
 */
struct cam_hw_soc_info {
	struct platform_device         *pdev;
	struct device                  *dev;
	uint32_t                        hw_version;
	uint32_t                        index;
	const char                     *dev_name;
	bool                            is_nrt_dev;
	const char                     *irq_name[CAM_SOC_MAX_IRQ_LINES_PER_DEV];
	const char                     *label_name;
	struct resource                *irq_line[CAM_SOC_MAX_IRQ_LINES_PER_DEV];
	int                             irq_num[CAM_SOC_MAX_IRQ_LINES_PER_DEV];
	void                           *irq_data[CAM_SOC_MAX_IRQ_LINES_PER_DEV];
	uint32_t                        irq_count;
	const char                     *compatible;

	uint32_t                        num_mem_block;
	const char                     *mem_block_name[CAM_SOC_MAX_BLOCK];
	uint32_t                        mem_block_cam_base[CAM_SOC_MAX_BLOCK];
	struct resource                *mem_block[CAM_SOC_MAX_BLOCK];
	struct cam_soc_reg_map          reg_map[CAM_SOC_MAX_BASE];
	uint32_t                        num_reg_map;
	uint32_t                        reserve_mem;

	uint32_t                        num_rgltr;
	const char                     *rgltr_name[CAM_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_ctrl_support;
	uint32_t                        rgltr_min_volt[CAM_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_max_volt[CAM_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_op_mode[CAM_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_type[CAM_SOC_MAX_REGULATOR];
	struct regulator               *rgltr[CAM_SOC_MAX_REGULATOR];
	uint32_t                        rgltr_delay[CAM_SOC_MAX_REGULATOR];

	uint32_t                        use_shared_clk;
	uint32_t                        num_clk;
	const char                     *clk_name[CAM_SOC_MAX_CLK];
	struct clk                     *clk[CAM_SOC_MAX_CLK];
	int32_t                         clk_rate[CAM_MAX_VOTE][CAM_SOC_MAX_CLK];
	uint32_t                        clk_id[CAM_SOC_MAX_CLK];
	uint32_t                        shared_clk_mask;
	int32_t                         prev_clk_level;
	int32_t                         src_clk_idx;
	struct cam_soc_util_clk_rates   applied_src_clk_rates;
	bool                            clk_level_valid[CAM_MAX_VOTE];
	uint32_t                        lowest_clk_level;
	int32_t                         scl_clk_count;
	int32_t                         scl_clk_idx[CAM_SOC_MAX_CLK];
	const char                     *optional_clk_name[CAM_SOC_MAX_OPT_CLK];
	struct clk                     *optional_clk[CAM_SOC_MAX_OPT_CLK];
	int32_t                         optional_clk_rate[CAM_SOC_MAX_OPT_CLK];
	uint32_t                        optional_clk_id[CAM_SOC_MAX_OPT_CLK];
	uint32_t                        optional_shared_clk_mask;

	void                           *mmrm_handle;

	bool                            is_clk_drv_en;

	struct cam_soc_gpio_data       *gpio_data;
	struct cam_soc_pinctrl_info     pinctrl_info;

	struct dentry                  *dentry;
	uint32_t                        clk_level_override_high;
	uint32_t                        clk_level_override_low;
	bool                            clk_control_enable;
	bool                            cam_cx_ipeak_enable;
	int32_t                         cam_cx_ipeak_bit;

	void                           *soc_private;
};

/**
 * struct cam_hw_soc_dump_header - SOC dump header
 *
 * @Brief:        soc hw dump header
 *
 * @tag:          Tag name for the header
 * @word_size:    Size of each word
 * @size:         Total size of dumped data
 */
struct cam_hw_soc_dump_header {
	uint8_t   tag[CAM_SOC_HW_DUMP_TAG_MAX_LEN];
	uint64_t  size;
	uint32_t  word_size;
};

/**
 * struct cam_hw_soc_dump_args:   SOC Dump args
 *
 * @request_id:          Issue request id
 * @offset:              Buffer offset, updated as the informaton is dumped
 * @buf_handle:          Buffer handle of the out buffer
 */
struct cam_hw_soc_dump_args {
	uint64_t             request_id;
	size_t               offset;
	uint32_t             buf_handle;
};

/*
 * CAM_SOC_GET_REG_MAP_START
 *
 * @brief:              This MACRO will get the mapped starting address
 *                      where the register space can be accessed
 *
 * @__soc_info:         Device soc information
 * @__base_index:       Index of register space in the HW block
 *
 * @return:             Returns a pointer to the mapped register memory
 */
#define CAM_SOC_GET_REG_MAP_START(__soc_info, __base_index)          \
	((!__soc_info || __base_index >= __soc_info->num_reg_map) ?  \
		NULL : __soc_info->reg_map[__base_index].mem_base)

/*
 * CAM_SOC_GET_REG_MAP_CAM_BASE
 *
 * @brief:              This MACRO will get the cam_base of the
 *                      register space
 *
 * @__soc_info:         Device soc information
 * @__base_index:       Index of register space in the HW block
 *
 * @return:             Returns an int32_t value.
 *                        Failure: -1
 *                        Success: Starting offset of register space compared
 *                                 to entire Camera Register Map
 */
#define CAM_SOC_GET_REG_MAP_CAM_BASE(__soc_info, __base_index)       \
	((!__soc_info || __base_index >= __soc_info->num_reg_map) ?  \
		-1 : __soc_info->reg_map[__base_index].mem_cam_base)

/*
 * CAM_SOC_GET_REG_MAP_SIZE
 *
 * @brief:              This MACRO will get the size of the mapped
 *                      register space
 *
 * @__soc_info:         Device soc information
 * @__base_index:       Index of register space in the HW block
 *
 * @return:             Returns a uint32_t value.
 *                        Failure: 0
 *                        Success: Non-zero size of mapped register space
 */
#define CAM_SOC_GET_REG_MAP_SIZE(__soc_info, __base_index)           \
	((!__soc_info || __base_index >= __soc_info->num_reg_map) ?  \
		0 : __soc_info->reg_map[__base_index].size)

/**
 * cam_soc_util_get_level_from_string()
 *
 * @brief:              Get the associated vote level for the input string
 *
 * @string:             Input string to compare with.
 * @level:              Vote level corresponds to input string.
 *
 * @return:             Success or failure
 */
int cam_soc_util_get_level_from_string(const char *string,
	enum cam_vote_level *level);

/**
 * cam_soc_util_get_dt_properties()
 *
 * @brief:              Parse the DT and populate the common properties that
 *                      are part of the soc_info structure - register map,
 *                      clocks, regulators, irq, etc.
 *
 * @soc_info:           Device soc struct to be populated
 *
 * @return:             Success or failure
 */
int cam_soc_util_get_dt_properties(struct cam_hw_soc_info *soc_info);

/**
 * cam_soc_util_request_platform_resource()
 *
 * @brief:              Request regulator, irq, and clock resources
 *
 * @soc_info:           Device soc information
 * @handler:            Irq handler function pointer
 * @irq_data:           Irq handler function CB data
 *
 * @return:             Success or failure
 */
int cam_soc_util_request_platform_resource(struct cam_hw_soc_info *soc_info,
	irq_handler_t handler, void **irq_data);

/**
 * cam_soc_util_release_platform_resource()
 *
 * @brief:              Release regulator, irq, and clock resources
 *
 * @soc_info:           Device soc information
 *
 * @return:             Success or failure
 */
int cam_soc_util_release_platform_resource(struct cam_hw_soc_info *soc_info);

/**
 * cam_soc_util_enable_platform_resource()
 *
 * @brief:              Enable regulator, irq resources
 *
 * @soc_info:           Device soc information
 * @cesta_client_idx:   CESTA Client idx for hw client based src clocks
 * @enable_clocks:      Boolean flag:
 *                          TRUE: Enable all clocks in soc_info Now.
 *                          False: Don't enable clocks Now. Driver will
 *                                 enable independently.
 * @clk_level:          Clock level to be applied.
 *                      Applicable only if enable_clocks is true
 *                          Valid range : 0 to (CAM_MAX_VOTE - 1)
 * @irq_enable:         Boolean flag:
 *                          TRUE: Enable IRQ in soc_info Now.
 *                          False: Don't enable IRQ Now. Driver will
 *                                 enable independently.
 *
 * @return:             Success or failure
 */
int cam_soc_util_enable_platform_resource(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx, bool enable_clocks, enum cam_vote_level clk_level,
	bool irq_enable);

/**
 * cam_soc_util_disable_platform_resource()
 *
 * @brief:              Disable regulator, irq resources
 *
 * @soc_info:           Device soc information
 * @cesta_client_idx:   CESTA Client idx for hw client based src clocks
 * @disable_irq:        Boolean flag:
 *                          TRUE: Disable IRQ in soc_info Now.
 *                          False: Don't disable IRQ Now. Driver will
 *                                 disable independently.
 *
 * @return:             Success or failure
 */
int cam_soc_util_disable_platform_resource(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx, bool disable_clocks, bool disable_irq);

/**
 * cam_soc_util_get_clk_round_rate()
 *
 * @brief:              Get the rounded clock rate for the given clock's
 *                      clock rate value
 *
 * @soc_info:           Device soc information
 * @clk_index:          Clock index in soc_info for which round rate is needed
 * @clk_rate:           Input clock rate for which rounded rate is needed
 *
 * @return:             Rounded clock rate
 */
long cam_soc_util_get_clk_round_rate(struct cam_hw_soc_info *soc_info,
	uint32_t clk_index, unsigned long clk_rate);

/**
 * cam_soc_util_set_src_clk_rate()
 *
 * @brief:              Set the rate on the source clock for sw or hw clients. Requires a valid
 *                      CESTA client idx for hw client voting.
 *
 * @soc_info:           Device soc information
 * @cesta_client_idx:   CESTA client idx if src clock belongs to cesta client, otherwise -1
 * @clk_rate_high:      High clock rate associated with the src clk, applies to sw client vote
 *                      if not cesta client
 * @clk_rate_low:       Low clock rate associated with the src clk, only applies to cesta based
 *                      hw client vote
 *
 * @return:             success or failure
 */
int cam_soc_util_set_src_clk_rate(struct cam_hw_soc_info *soc_info, int cesta_client_idx,
	unsigned long clk_rate_high, unsigned long clk_rate_low);


/**
 * cam_soc_util_get_option_clk_by_name()
 *
 * @brief:              Get reference to optional clk using name
 *
 * @soc_info:           Device soc information
 * @clk_name:           Name of clock to find reference for
 * @clk_index:          Clk index in the option clk array to be returned
 *
 * @return:             0: Success
 *                      Negative: Failure
 */
int cam_soc_util_get_option_clk_by_name(struct cam_hw_soc_info *soc_info,
	const char *clk_name, int32_t *clk_index);

/**
 * cam_soc_util_put_optional_clk()
 *
 * @brief:              Put clock corresponding to index specified in params
 *
 * @soc_info:           Device soc information
 * @clk_idx:            Clock index in optional clocks to put
 *
 * @return:             Success or failure
 */
int cam_soc_util_put_optional_clk(struct cam_hw_soc_info *soc_info,
	int32_t clk_idx);

/**
 * cam_soc_util_clk_enable()
 *
 * @brief:              Enable clock specified in params
 *
 * @soc_info:           Device soc information
 * @cesta_client_idx:   CESTA Client idx for hw client based src clocks
 * @optional_clk:       Whether to set optional clk or normal clk with
 *                      the idx given
 * @clk_idx:            Clock index to set
 * @apply_level:        Apply level.
 *                      -1 for 0 rate
 *                      any other value indicate level for normal clocks
 *                      For optional clocks any other value means the rate saved
 *                      in soc_info
 *
 * @return:             Success or failure
 */
int cam_soc_util_clk_enable(struct cam_hw_soc_info *soc_info, int cesta_client_idx,
	bool optional_clk, int32_t clk_idx, int32_t apply_level);

/**
 * cam_soc_util_set_clk_rate_level()
 *
 * @brief:              Apply clock rates for the requested level.
 *                      This applies the new requested level for all
 *                      the clocks listed in DT based on their values.
 *
 * @soc_info:           Device soc information
 * @cesta_client_idx:   CESTA client idx for HW client based src clocks
 * @clk_level_high:     Clock level number to set, high value if crm based src clock
 * @clk_level_low:      Low clock level value if crm based src clock
 * @do_not_set_src_clk: If true, set clock rates except the src clk
 *
 * @return:             Success or failure
 */
int cam_soc_util_set_clk_rate_level(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx, enum cam_vote_level clk_level_high,
	enum cam_vote_level clk_level_low, bool do_not_set_src_clk);

/**
 * cam_soc_util_clk_disable()
 *
 * @brief:              Disable clock specified in params
 *
 * @soc_info:           Device soc information
 * @cesta_client_idx:   CESTA Client idx for hw client based src clocks
 * @optional_clk:       Whether to set optional clk or normal clk with
 *                      the idx given
 * @clk_idx:            Clock index to disable
 *
 * @return:             Success or failure
 */
int cam_soc_util_clk_disable(struct cam_hw_soc_info *soc_info, int cesta_client_idx,
	bool optional_clk, int32_t clk_idx);

/**
 * cam_soc_util_irq_enable()
 *
 * @brief:              Enable IRQ in SOC
 *
 * @soc_info:           Device soc information
 *
 * @return:             Success or failure
 */
int cam_soc_util_irq_enable(struct cam_hw_soc_info *soc_info);

/**
 * cam_soc_util_irq_disable()
 *
 * @brief:              Disable IRQ in SOC
 *
 * @soc_info:           Device soc information
 *
 * @return:             Success or failure
 */
int cam_soc_util_irq_disable(struct cam_hw_soc_info *soc_info);

/**
 * cam_soc_util_regulator_enable()
 *
 * @brief:              Enable single regulator
 *
 * @rgltr               Regulator that needs to be turned ON
 * @rgltr_name          Associated Regulator name
 * @rgltr_min_volt:     Requested minimum volatage
 * @rgltr_max_volt:     Requested maximum volatage
 * @rgltr_op_mode:      Requested Load
 * @rgltr_delay:        Requested delay needed aaftre enabling regulator
 *
 * @return:             Success or failure
 */
int cam_soc_util_regulator_enable(struct regulator *rgltr,
	const char *rgltr_name,
	uint32_t rgltr_min_volt, uint32_t rgltr_max_volt,
	uint32_t rgltr_op_mode, uint32_t rgltr_delay);

/**
 * cam_soc_util_regulator_enable()
 *
 * @brief:              Disable single regulator
 *
 * @rgltr               Regulator that needs to be turned ON
 * @rgltr_name          Associated Regulator name
 * @rgltr_min_volt:     Requested minimum volatage
 * @rgltr_max_volt:     Requested maximum volatage
 * @rgltr_op_mode:      Requested Load
 * @rgltr_delay:        Requested delay needed aaftre enabling regulator
 *
 * @return:             Success or failure
 */
int cam_soc_util_regulator_disable(struct regulator *rgltr,
	const char *rgltr_name,
	uint32_t rgltr_min_volt, uint32_t rgltr_max_volt,
	uint32_t rgltr_op_mode, uint32_t rgltr_delay);

/**
 * cam_soc_util_reg_addr_validation()
 *
 * @brief:              Camera SOC util for validating address to be accessed
 *
 * @soc_info:           Device soc information
 * @base_index:         Index of register space in the HW block
 * @offset:             Register offset
 *
 * @return:             0 or specific error code
 */
static inline int cam_soc_util_reg_addr_validation(
	struct cam_hw_soc_info *soc_info,
	uint32_t base_idx, uint32_t offset)
{
	if (offset > (uint32_t)soc_info->reg_map[base_idx].size) {
		CAM_ERR(CAM_UTIL,
			"Reg offset out of range, offset: 0x%X reg_map size: 0x%X",
			offset,
			(uint32_t)soc_info->reg_map[base_idx].size);
		return -EINVAL;
	}

	if (offset % 4) {
		CAM_ERR(CAM_UTIL, "Offset: 0x%X is not memory aligned",
			offset);
		return -EINVAL;
	}

	return 0;
}

/**
 * cam_soc_util_w()
 *
 * @brief:              Camera SOC util for register write
 *
 * @soc_info:           Device soc information
 * @base_index:         Index of register space in the HW block
 * @offset:             Offset of register to be writen
 * @data:               Value to be written
 *
 * @return:             Success or Failure
 */
static inline int cam_soc_util_w(struct cam_hw_soc_info *soc_info,
	uint32_t base_index, uint32_t offset, uint32_t data)
{
	if (!CAM_SOC_GET_REG_MAP_START(soc_info, base_index)) {
		CAM_ERR(CAM_UTIL, "No valid mapped starting address found");
		return -EINVAL;
	}

	if (cam_soc_util_reg_addr_validation(soc_info, base_index, offset))
		return -EINVAL;

	return cam_io_w(data,
		CAM_SOC_GET_REG_MAP_START(soc_info, base_index) + offset);
}

/**
 * cam_soc_util_w_mb()
 *
 * @brief:              Camera SOC util for register write with memory barrier.
 *                      Memory Barrier is only before the write to ensure the
 *                      order. If need to ensure this write is also flushed
 *                      call wmb() independently in the caller.
 *
 * @soc_info:           Device soc information
 * @base_index:         Index of register space in the HW block
 * @offset:             Offset of register to be writen
 * @data:               Value to be written
 *
 * @return:             Success or Failure
 */
static inline int cam_soc_util_w_mb(struct cam_hw_soc_info *soc_info,
	uint32_t base_index, uint32_t offset, uint32_t data)
{
	if (!CAM_SOC_GET_REG_MAP_START(soc_info, base_index)) {
		CAM_ERR(CAM_UTIL, "No valid mapped starting address found");
		return -EINVAL;
	}

	if (cam_soc_util_reg_addr_validation(soc_info, base_index, offset))
		return -EINVAL;

	return cam_io_w_mb(data,
		CAM_SOC_GET_REG_MAP_START(soc_info, base_index) + offset);
}

/**
 * cam_soc_util_r()
 *
 * @brief:              Camera SOC util for register read
 *
 * @soc_info:           Device soc information
 * @base_index:         Index of register space in the HW block
 * @offset:             Offset of register to be read
 *
 * @return:             Value read from the register address
 */
static inline uint32_t cam_soc_util_r(struct cam_hw_soc_info *soc_info,
	uint32_t base_index, uint32_t offset)
{
	if (!CAM_SOC_GET_REG_MAP_START(soc_info, base_index)) {
		CAM_ERR(CAM_UTIL, "No valid mapped starting address found");
		return 0;
	}

	if (cam_soc_util_reg_addr_validation(soc_info, base_index, offset))
		return 0;

	return cam_io_r(
		CAM_SOC_GET_REG_MAP_START(soc_info, base_index) + offset);
}

/**
 * cam_soc_util_r_mb()
 *
 * @brief:              Camera SOC util for register read with memory barrier.
 *                      Memory Barrier is only before the write to ensure the
 *                      order. If need to ensure this write is also flushed
 *                      call rmb() independently in the caller.
 *
 * @soc_info:           Device soc information
 * @base_index:         Index of register space in the HW block
 * @offset:             Offset of register to be read
 *
 * @return:             Value read from the register address
 */
static inline uint32_t cam_soc_util_r_mb(struct cam_hw_soc_info *soc_info,
	uint32_t base_index, uint32_t offset)
{
	if (!CAM_SOC_GET_REG_MAP_START(soc_info, base_index)) {
		CAM_ERR(CAM_UTIL, "No valid mapped starting address found");
		return 0;
	}

	if (cam_soc_util_reg_addr_validation(soc_info, base_index, offset))
		return 0;

	return cam_io_r_mb(
		CAM_SOC_GET_REG_MAP_START(soc_info, base_index) + offset);
}

/**
 * cam_soc_util_reg_dump()
 *
 * @brief:              Camera SOC util for dumping a range of register
 *
 * @soc_info:           Device soc information
 * @base_index:         Index of register space in the HW block
 * @offset:             Start register offset for the dump
 * @size:               Size specifying the range for dump
 *
 * @return:             Success or Failure
 */
int cam_soc_util_reg_dump(struct cam_hw_soc_info *soc_info,
	uint32_t base_index, uint32_t offset, int size);

void cam_soc_util_clk_disable_default(struct cam_hw_soc_info *soc_info,
	int cesta_client_idx);

int cam_soc_util_clk_enable_default(struct cam_hw_soc_info *soc_info, int cesta_client_idx,
	enum cam_vote_level clk_level);

int cam_soc_util_get_clk_level(struct cam_hw_soc_info *soc_info,
	int64_t clk_rate, int clk_idx, int32_t *clk_lvl);

/* Callback to get reg space data for specific HW */
typedef int (*cam_soc_util_regspace_data_cb)(uint32_t reg_base_type,
	void *ctx, struct cam_hw_soc_info **soc_info_ptr,
	uint32_t *reg_base_idx);

/**
 * cam_soc_util_reg_dump_to_cmd_buf()
 *
 * @brief:                 Camera SOC util for dumping sets of register ranges
 *                         command buffer
 *
 * @ctx:                   Context info from specific hardware manager
 * @cmd_desc:              Command buffer descriptor
 * @req_id:                Last applied req id for which reg dump is required
 * @reg_data_cb:           Callback function to get reg space info based on type
 *                         in command buffer
 * @soc_dump_args:         Dump buffer args to dump the soc information.
 * @user_triggered_dump:   Flag to indicate if the dump request is issued by
 *                         user.
 * @return:                Success or Failure
 */
int cam_soc_util_reg_dump_to_cmd_buf(void *ctx,
	struct cam_cmd_buf_desc *cmd_desc, uint64_t req_id,
	cam_soc_util_regspace_data_cb reg_data_cb,
	struct cam_hw_soc_dump_args *soc_dump_args,
	bool user_triggered_dump);

/**
 * cam_soc_util_print_clk_freq()
 *
 * @brief:              This function gets the clk rates for each clk from clk
 *                      driver and prints in log
 *
 * @soc_info:           Device soc struct to be populated
 *
 * @return:             success or failure
 */
int cam_soc_util_print_clk_freq(struct cam_hw_soc_info *soc_info);

/**
 * cam_soc_util_select_pinctrl_state()
 *
 * @brief:              This function gets the pinctrl handle
 *
 * @soc_info:           Device soc struct to be populated
 * @active:             True for active and false for suspend state
 *
 * @return:             success or failure
 */
int cam_soc_util_select_pinctrl_state(
	struct cam_hw_soc_info *soc_info, int idx, bool active);

/**
 * cam_soc_util_print_clk_freq()
 *
 * @brief:              This function checks whether regulators of this device are enabled at this
 *                      time.
 *
 * @soc_info:           Device soc struct to be populated
 *
 * @return:             Number of regulators enabled
 */
int cam_soc_util_regulators_enabled(struct cam_hw_soc_info *soc_info);

/**
 * cam_soc_util_cesta_populate_crm_device()
 *
 * @brief:              This function populates the camera cesta crm device in soc util
 *
 * @return:             success or failure
 */
inline int cam_soc_util_cesta_populate_crm_device(void);

/**
 * cam_soc_util_cesta_channel_switch()
 *
 * @brief:              This function triggers the application of power states to crm
 *                      and channel switch operation in hw. Also, for camera it applies
 *                      the high vote of the active channel
 * @cesta_client_idx:   CESTA client index through which power states need to be applied
 * @identifier:         Identifying the caller triggerring channel switch
 *
 * @return:             success or failure
 */
int cam_soc_util_cesta_channel_switch(uint32_t cesta_client_idx, const char *identifier);

/**
 * cam_soc_util_get_applied_src_clk()
 *
 * @brief:              Inline function to get applied src clk rate.

 * @soc_info:           Device soc struct to be populated
 * @is_max:             Is max of all hw clients if cesta is enabled
 *
 * @return:             success or failure
 */
inline unsigned long cam_soc_util_get_applied_src_clk(
	struct cam_hw_soc_info *soc_info, bool is_max);

/**
 * cam_soc_util_get_string_from_level()
 *
 * @brief:     Returns the string for a given clk level
 *
 * @level:     Clock level
 *
 * @return:    String corresponding to the clk level
 */
const char *cam_soc_util_get_string_from_level(enum cam_vote_level level);

/**
 * cam_wrapper_clk_get_rate()
 *
 * @brief:     Wrapper for clk get rate
 *
 * @clk:       Clock
 *
 * @return:    Clock rate
 */
inline unsigned long cam_wrapper_clk_get_rate(struct clk *clk);

/**
 * cam_wrapper_regulator_set_load()
 *
 * @brief:     Wrapper for regulator set load
 *
 * @regulator: Regulator
 *
 * @uA_load:   Load current
 *
 * @return:    Success or failure
 */
inline int cam_wrapper_regulator_set_load(
	struct regulator *regulator, int uA_load);

/**
 * cam_wrapper_regulator_set_mode()
 *
 * @brief:     Wrapper for regulator set mode
 *
 * @regulator: Regulator
 *
 * @mode:      Mode
 *
 * @return:    Success or failure
 */
inline int cam_wrapper_regulator_set_mode(
	struct regulator *regulator, unsigned int mode);

/**
 * cam_soc_util_set_bypass_drivers()
 *
 * @brief:          Set bypass drivers
 *
 * @bypass_drivers: Bypass drivers
 *
 * @return:         Void
 */
inline void cam_soc_util_set_bypass_drivers(
	uint32_t bypass_drivers);

#endif /* _CAM_SOC_UTIL_H_ */
