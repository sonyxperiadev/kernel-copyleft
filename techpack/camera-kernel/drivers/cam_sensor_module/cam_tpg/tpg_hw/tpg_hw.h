/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __TPG_HW_H__
#define __TPG_HW_H__

#include <linux/kernel.h>
#include "cam_debug_util.h"
#include "cam_soc_util.h"
#include <cam_cpas_api.h>
#include <media/cam_sensor.h>
#include <linux/spinlock.h>
#include <linux/completion.h>

#define TPG_HW_VERSION_1_0   0x10000000
#define TPG_HW_VERSION_1_1   0x10000001
#define TPG_HW_VERSION_1_2   0x10000002
#define TPG_HW_VERSION_1_3   0x10000003
#define TPG_HW_VERSION_1_3_1 0x10000004
#define TPG_HW_VERSION_1_4   0x10000005

struct tpg_hw;

/**
 * tpg_hw_ops : tpg hw operations
 *
 * @init        : tpg hw layer init
 * @start       : tpg hw start stream
 * @stop        : tpg hw stop  stream
 * @deinit      : tpg hw deinit
 * @read        : tpg hw read register
 * @write       : tpg hw write register
 * @process_cmd : tpg hw process command
 * @dump_status : dump any status registers
 * @write_settings : write register settings
 */
struct tpg_hw_ops {
	int (*init)(struct tpg_hw *hw, void *data);
	int (*start)(struct tpg_hw *hw, void *data);
	int (*stop)(struct tpg_hw *hw, void *data);
	int (*deinit)(struct tpg_hw *hw, void *data);
	int (*read)(struct tpg_hw *hw, uint32_t  *addr, uint32_t *val);
	int (*write)(struct tpg_hw *hw, uint32_t *addr, uint32_t *val);
	int (*process_cmd)(struct tpg_hw *hw, uint32_t cmd, void *arg);
	int (*dump_status)(struct tpg_hw *hw, void *data);
	int (*write_settings)(struct tpg_hw *hw,
		struct tpg_settings_config_t *config, struct tpg_reg_settings *settings);
	irqreturn_t (*handle_irq)(struct tpg_hw *hw);
};

/**
 * tpg_hw_state : tpg hw states
 *
 * TPG_HW_STATE_HW_DISABLED: tpg hw is not enabled yet
 * TPG_HW_STATE_HW_ENABLED : tpg hw is enabled
 * TPG_HW_STATE_READY      : tpg hw is ready
 * TPG_HW_STATE_BUSY       : tpg hw is busy
 * TPG_HW_STATE_MAX        : tp hw is max
 */
enum tpg_hw_state {
	TPG_HW_STATE_HW_DISABLED,
	TPG_HW_STATE_HW_ENABLED,
	TPG_HW_STATE_READY,
	TPG_HW_STATE_BUSY,
	TPG_HW_STATE_MAX
};

#define TPG_HW_REQ_TYPE_NOP     0x00
#define TPG_HW_REQ_TYPE_INIT    0x01
#define TPG_HW_REQ_TYPE_UPDATE  0x02
#define TPG_HW_REQ_TYPE_BYPASS  0x03

/**
 * tpg_hw_request : tpg hw request info
 *
 * @request_id        : request id
 * @request_type      : request type
 * @global_config     : global configuration
 * @vc_slots          : vc slots
 * @vc_count          : vc count
 * @list              : list
 */
struct tpg_hw_request {
	uint64_t                   request_id;
	int                        request_type;
	struct tpg_global_config_t global_config;
	struct tpg_vc_slot_info    *vc_slots;
	uint32_t                   vc_count;
	struct list_head           list;
};

/**
 * @brief tpg_vc_slot_info
 * @slot_id      : slot id of this vc slot
 * @vc           : virtual channel configured
 * @stream_count : number of streams in this slot
 * @head         : head pointing all data types in with this vc
 */
struct tpg_vc_slot_info {
	int    slot_id;
	int    vc;
	int    stream_count;
	struct list_head head;
};

/**
 * tpg_hw_info : tpg hw layer info
 *
 * @version:  version of tpg hw
 * @max_vc_channels: max number of virtual channels supported by tpg
 * @max_dt_channels_per_vc: max dts supported in each vc
 * @ops:   tpg hw operations
 * @hw_data: tpg hw data
 * @hw: hw layer initialization
 * @xcfa_debug: for xcfa debug
 * @shdr_overlap: for shdr overlap
 * @shdr_offset_num_batch: for shdr offset num batch
 * @shdr_line_offset0: for shdr line offset0
 * @shdr_line_offset1: for shdr line offset1
 */
struct tpg_hw_info {
	uint32_t          version;
	uint32_t          max_vc_channels;
	uint32_t          max_dt_channels_per_vc;
	struct tpg_hw_ops *ops;
	void              *hw_data;
	int               (*layer_init)(struct tpg_hw *hw);
	int               xcfa_debug;
	int               shdr_overlap;
	int               shdr_offset_num_batch;
	int               shdr_line_offset0;
	int               shdr_line_offset1;
};


/**
 * tpg_hw_stream : tpg hw stream
 *
 * @stream : tpg stream;
 * @list   : entry to tpg stream list
 */
struct tpg_hw_stream {
	struct tpg_stream_config_t stream;
	struct list_head list;
};

/**
 * tpg_hw_stream_v3 : tpg hw stream version 2
 *
 * @stream : tpg stream version 2
 * @list   : entry to tpg stream list
 */
struct tpg_hw_stream_v3 {
	struct tpg_stream_config_v3_t stream;
	struct list_head list;
};


/**
 * tpg_hw : tpg hw
 *
 * @hw_idx                  : hw id
 * @stream_version          : stream version
 * @state                   : tpg hw state
 * @cpas_handle             : handle to cpas
 * @vc_count                : vc count
 * @tpg_clock               : tpg clock
 * @hw_info                 : tp hw info
 * @soc_info                : soc info
 * @mutex                   : lock
 * @complete_rup            : rup done completion
 * @vc_slots                : vc slots info
 * @settings_config         : settings configuration
 * @register_settings       : register settings info
 * @tpg_hw_irq_handler      : handle hw irq
 * @waiting_request_q       : queue of waiting requests to be applied
 * @acitive_request_q       : queue of active requests being applied,
 * @active_request_q_depth  : active request queue depth
 * @waiting_request_q_depth : waiting request queue depth
 * @settings update         : settings update flag
 */
struct tpg_hw {
	uint32_t                     hw_idx;
	uint32_t                     stream_version;
	uint32_t                     state;
	uint32_t                     cpas_handle;
	uint32_t                     vc_count;
	uint64_t                     tpg_clock;
	struct tpg_hw_info           *hw_info;
	struct cam_hw_soc_info       *soc_info;
	struct mutex                 mutex;
	spinlock_t                   hw_state_lock;
	struct completion            complete_rup;
	struct tpg_vc_slot_info      *vc_slots;
	struct tpg_settings_config_t settings_config;
	struct tpg_reg_settings      *register_settings;
	int                          (*tpg_hw_irq_handler)(struct tpg_hw *hw);
	/* Waiting and active request Queues */
	struct list_head             waiting_request_q;
	struct list_head             active_request_q;
	uint32_t                     active_request_q_depth;
	uint32_t                     waiting_request_q_depth;
	uint32_t                     settings_update;
};

/**
 * tpg_hw_acquire_args : tpg hw acquire arguments
 *
 * @resource_list  : list of resources to acquire
 * @count          : number of resources to acquire
 */
struct tpg_hw_acquire_args {
	/* Integer id of resources */
	uint32_t *resource_list;
	ssize_t  count;
};

enum tpg_hw_cmd_t {
	TPG_HW_CMD_INVALID = 0,
	TPG_HW_CMD_INIT_CONFIG,
	TPG_HW_CMD_MAX,
};

#define TPG_HW_CONFIG_BASE 0x4000
#define TPG_CONFIG_CTRL    (TPG_HW_CONFIG_BASE + 0)
#define TPG_CONFIG_VC      (TPG_HW_CONFIG_BASE + 1)
#define TPG_CONFIG_DT      (TPG_HW_CONFIG_BASE + 2)

/* pixel bit width */
#define PACK_8_BIT    8
#define PACK_10_BIT   10
#define PACK_12_BIT   12
#define PACK_14_BIT   14
#define PACK_16_BIT   16

/**
 * @vc_config_args : arguments for vc config process cmd
 *
 * @vc_slot : slot to configure this vc
 * @num_dts : number of dts in this vc
 * @stream  : output stream
 */
struct vc_config_args {
	uint32_t vc_slot;
	uint32_t num_dts;
	struct tpg_stream_config_t *stream;
};

/**
 * dt_config_args : arguments for dt config process cmd
 *
 * @vc_slot  : vc slot to configure this dt
 * @dt_slot  : dt slot to configure this dt
 * @stream   : stream packet to configure for this dt
 */
struct dt_config_args {
	uint32_t vc_slot;
	uint32_t dt_slot;
	struct tpg_stream_config_t *stream;
};

/**
 * @vc_config_args_v3 : arguments for vc config process cmd version 2
 *
 * @vc_slot : slot to configure this vc
 * @num_dts : number of dts in this vc
 * @stream  : output stream version 2
 */
struct vc_config_args_v3 {
	uint32_t vc_slot;
	uint32_t num_dts;
	struct tpg_stream_config_v3_t *stream;
};

/**
 * dt_config_args_v3 : arguments for dt config process cmd version 2
 *
 * @vc_slot  : vc slot to configure this dt
 * @dt_slot  : dt slot to configure this dt
 * @stream   : stream packet to configure for this dt version 2
 */
struct dt_config_args_v3 {
	uint32_t vc_slot;
	uint32_t dt_slot;
	struct tpg_stream_config_v3_t *stream;
};


/**
 * global_config_args : tpg global config args
 *
 * @num_vcs : number of vcs to be configured
 * @globalconfig: global config cmd
 */
struct global_config_args {
	uint32_t num_vcs;
	struct tpg_global_config_t *globalconfig;
};

/**
 * tpg_hw_initsettings : initial configurations
 *
 * @global_config : global configuration
 * @streamconfigs : array of stream configurations
 * @num_streams   : number of streams in strea config array
 */
struct tpg_hw_initsettings {
	struct tpg_global_config_t globalconfig;
	struct tpg_stream_config_t *streamconfigs;
	uint32_t num_streams;
};

/**
 * tpg_hw_initsettings_v3 : initial configurations version 2
 *
 * @global_config : global configuration
 * @streamconfigs : array of stream configurations version 2
 * @num_streams   : number of streams in strea config array
 */
struct tpg_hw_initsettings_v3 {
	struct tpg_global_config_t globalconfig;
	struct tpg_stream_config_v3_t *streamconfigs;
	uint32_t num_streams;
};


/**
 * @brief dump the tpg memory info
 *
 * @param soc_info: soc info for soc related info
 *
 * @return : 0 on succuss
 */
int32_t cam_tpg_mem_dmp(struct cam_hw_soc_info *soc_info);

/**
 * @brief dump global config command
 *
 * @param idx    : hw index
 * @param global : global config command
 *
 * @return       : 0 on success
 */
int dump_global_configs(int idx,
		struct tpg_global_config_t *global);

/**
 * @brief : dump stream config command
 *
 * @param hw_idx: hw index
 * @param stream_idx: stream index
 * @param stream: stream config command
 *
 * @return : 0 on success
 */
int dump_stream_configs(int hw_idx,
		int stream_idx,
		struct tpg_stream_config_t *stream);

/**
 * @brief : dump stream config command version 2
 *
 * @param hw_idx: hw index
 * @param stream_idx: stream index
 * @param stream: stream config command version 2
 *
 * @return : 0 on success
 */
int dump_stream_configs_v3(int hw_idx, int stream_idx, struct tpg_stream_config_v3_t *stream);


/**
 * @brief : dump any hw status registers
 *
 * @param hw: tpg hw instance
 *
 * @return : 0 on success
 */
int tpg_hw_dump_status(struct tpg_hw *hw);
/**
 * @brief : start tpg hw stream
 *
 * @param hw: tpg hw instance
 *
 * @return : 0 on success
 */
int tpg_hw_start(struct tpg_hw *hw);

/**
 * @brief : stop tpg hw stream
 *
 * @param hw: tpg hw instance
 *
 * @return : 0 on success
 */
int tpg_hw_stop(struct tpg_hw *hw);

/**
 * @brief : tpg hw acquire
 *
 * @param hw: tpg hw instance
 * @param acquire: list of resources to acquire
 *
 * @return : 0 on success
 */
int tpg_hw_acquire(struct tpg_hw *hw,
		struct tpg_hw_acquire_args *acquire);
/**
 * @brief release tpg hw
 *
 * @param hw: tpg hw instance
 *
 * @return : 0 on success
 */
int tpg_hw_release(struct tpg_hw *hw);

/**
 * @brief : configure tpg hw
 *
 * @param hw: tpg hw instance
 * @param cmd: configuration command
 * @param arg: configuration command argument
 *
 * @return : 0 on success
 */
int tpg_hw_config(struct tpg_hw *hw, enum tpg_hw_cmd_t cmd, void *arg);

/**
 * @brief : reset the tpg hw instance
 *
 * @param hw: tpg hw instance
 *
 * @return : 0 on success
 */
int tpg_hw_reset(struct tpg_hw *hw);

/**
 * @brief : tp hw add stream
 *
 * @param hw: tpg hw instance
 * @param req: req instance
 * @param cmd: tpg hw command
 *
 * @return : 0 on success
 */
int tpg_hw_add_stream(
	struct tpg_hw *hw,
	struct tpg_hw_request *req,
	struct tpg_stream_config_t *cmd);

/**
 * @brief : tp hw add stream version 2
 *
 * @param hw: tpg hw instance
 * @param req: req instance
 * @param cmd: tpg hw command version 2
 *
 * @return : 0 on success
 */
int tpg_hw_add_stream_v3(
	struct tpg_hw *hw,
	struct tpg_hw_request *req,
	struct tpg_stream_config_v3_t *cmd);

/**
 * @brief : hw apply
 *
 * @param hw: tpg hw instance
 * @param request_id: request id
 *
 * @return : 0 on success
 */
int tpg_hw_apply(
	struct tpg_hw *hw,
	uint64_t request_id);

/**
 * @brief : create hw request
 *
 * @param hw: tpg hw instance
 * @param request_id: request id
 *
 * @return : req on success
 */
struct tpg_hw_request *tpg_hw_create_request(
	struct tpg_hw *hw,
	uint64_t request_id);

/**
 * @brief : free hw request
 *
 * @param hw: tpg hw instance
 * @param req: request instance
 *
 * @return : 0 on success
 */
int tpg_hw_free_request(struct tpg_hw *hw,
		struct tpg_hw_request *req);

/**
 * @brief : add hw request
 *
 * @param hw: tpg hw instance
 * @param req: request instance
 *
 * @return : 0 on success
 */
int tpg_hw_add_request(struct tpg_hw *hw,
	struct tpg_hw_request *req);

/**
 * @brief : set opcode of request
 *
 * @param hw: tpg hw instance
 * @param opcode: op code
 *
 * @return : 0 on success
 */
int tpg_hw_request_set_opcode(
	struct tpg_hw_request *req,
	uint32_t opcode);

/**
 * @brief : copy settings config command
 *
 * @param hw: tpg hw instance
 * @param settings: settings config command
 *
 * @return : 0 on success
 */
int tpg_hw_copy_settings_config(struct tpg_hw *hw, struct tpg_settings_config_t *settings);

#endif
