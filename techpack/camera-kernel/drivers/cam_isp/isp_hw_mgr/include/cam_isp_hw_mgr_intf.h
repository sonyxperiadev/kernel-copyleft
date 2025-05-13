/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_ISP_HW_MGR_INTF_H_
#define _CAM_ISP_HW_MGR_INTF_H_

#include <linux/of.h>
#include <linux/time.h>
#include <linux/list.h>
#include <media/cam_isp.h>
#include "cam_hw_mgr_intf.h"
#include "cam_packet_util.h"
#include "cam_cpas_api.h"

/* MAX IFE instance */
#define CAM_IFE_HW_NUM_MAX               8
#define CAM_SFE_HW_NUM_MAX               3
#define CAM_IFE_RDI_NUM_MAX              4
#define CAM_SFE_RDI_NUM_MAX              5
#define CAM_SFE_FE_RDI_NUM_MAX           3
#define CAM_ISP_BW_CONFIG_V1             1
#define CAM_ISP_BW_CONFIG_V2             2
#define CAM_ISP_BW_CONFIG_V3             3
#define CAM_TFE_HW_NUM_MAX               3
#define CAM_TFE_RDI_NUM_MAX              3
#define CAM_IFE_SCRATCH_NUM_MAX          2
#define CAM_IFE_BUS_COMP_NUM_MAX         18
#define CAM_SFE_BUS_COMP_NUM_MAX         12
#define CAM_TFE_BW_LIMITER_CONFIG_V1     1
#define CAM_TFE_BUS_COMP_NUM_MAX         18

/* maximum context numbers for TFE */
#define CAM_TFE_CTX_MAX      4

/* maximum context numbers for IFE */
#define CAM_IFE_CTX_MAX      8

/* Appliacble vote paths for dual ife, based on no. of UAPI definitions */
#define CAM_ISP_MAX_PER_PATH_VOTES 40

/* Output params for acquire from hw_mgr to ctx */
#define CAM_IFE_CTX_CUSTOM_EN          BIT(0)
#define CAM_IFE_CTX_FRAME_HEADER_EN    BIT(1)
#define CAM_IFE_CTX_CONSUME_ADDR_EN    BIT(2)
#define CAM_IFE_CTX_APPLY_DEFAULT_CFG  BIT(3)
#define CAM_IFE_CTX_SFE_EN             BIT(4)
#define CAM_IFE_CTX_AEB_EN             BIT(5)
#define CAM_IFE_CTX_DYNAMIC_SWITCH_EN  BIT(6)

/*
 * Maximum configuration entry size  - This is based on the
 * worst case DUAL IFE use case plus some margin.
 */
#define CAM_ISP_CTX_CFG_MAX                     30

/*
 * Maximum configuration entry size including SFE & CSID - This is based on the
 * worst case DUAL IFE/SFE use case plus some margin.
 */
#define CAM_ISP_SFE_CTX_CFG_MAX                 40

/* Maximum number of channels/contexts for FCG modules */
#define CAM_ISP_MAX_FCG_CH_CTXS        3
#define CAM_ISP_IFE_MAX_FCG_CH_CTXS    3
#define CAM_ISP_SFE_MAX_FCG_CHANNELS   2

/* Maximum number of predicitons for FCG config */
#define CAM_ISP_MAX_FCG_PREDICTIONS       3
#define CAM_ISP_IFE_MAX_FCG_PREDICTIONS   CAM_ISP_MAX_FCG_PREDICTIONS
#define CAM_ISP_SFE_MAX_FCG_PREDICTIONS   CAM_ISP_MAX_FCG_PREDICTIONS

/**
 *  enum cam_isp_hw_event_type - Collection of the ISP hardware events
 */
enum cam_isp_hw_event_type {
	CAM_ISP_HW_EVENT_ERROR,
	CAM_ISP_HW_EVENT_SOF,
	CAM_ISP_HW_EVENT_REG_UPDATE,
	CAM_ISP_HW_EVENT_EPOCH,
	CAM_ISP_HW_EVENT_EOF,
	CAM_ISP_HW_EVENT_DONE,
	CAM_ISP_HW_SECONDARY_EVENT,
	CAM_ISP_HW_EVENT_MAX
};

/**
 * cam_isp_hw_evt_type_to_string() - convert cam_isp_hw_event_type to string for printing logs
 */
static inline const char *cam_isp_hw_evt_type_to_string(
	enum cam_isp_hw_event_type evt_type)
{
	switch (evt_type) {
	case CAM_ISP_HW_EVENT_ERROR:
		return "ERROR";
	case CAM_ISP_HW_EVENT_SOF:
		return "SOF";
	case CAM_ISP_HW_EVENT_REG_UPDATE:
		return "REG_UPDATE";
	case CAM_ISP_HW_EVENT_EPOCH:
		return "EPOCH";
	case CAM_ISP_HW_EVENT_EOF:
		return "EOF";
	case CAM_ISP_HW_EVENT_DONE:
		return "BUF_DONE";
	default:
		return "INVALID_EVT";
	}
}

/**
 *  enum cam_isp_hw_secondary-event_type - Collection of the ISP hardware secondary events
 */
enum cam_isp_hw_secondary_event_type {
	CAM_ISP_HW_SEC_EVENT_SOF,
	CAM_ISP_HW_SEC_EVENT_EPOCH,
	CAM_ISP_HW_SEC_EVENT_OUT_OF_SYNC_FRAME_DROP,
	CAM_ISP_HW_SEC_EVENT_EVENT_MAX,
};

/**
 * enum cam_isp_hw_err_type - Collection of the ISP error types for
 *                         ISP hardware event CAM_ISP_HW_EVENT_ERROR
 */
enum cam_isp_hw_err_type {
	CAM_ISP_HW_ERROR_NONE                         = 0x00000001,
	CAM_ISP_HW_ERROR_OVERFLOW                     = 0x00000002,
	CAM_ISP_HW_ERROR_P2I_ERROR                    = 0x00000004,
	CAM_ISP_HW_ERROR_VIOLATION                    = 0x00000008,
	CAM_ISP_HW_ERROR_BUSIF_OVERFLOW               = 0x00000010,
	CAM_ISP_HW_ERROR_CSID_FATAL                   = 0x00000020,
	CAM_ISP_HW_ERROR_CSID_OUTPUT_FIFO_OVERFLOW    = 0x00000040,
	CAM_ISP_HW_ERROR_RECOVERY_OVERFLOW            = 0x00000080,
	CAM_ISP_HW_ERROR_CSID_FRAME_SIZE              = 0x00000100,
	CAM_ISP_HW_ERROR_CSID_LANE_FIFO_OVERFLOW      = 0x00000200,
	CAM_ISP_HW_ERROR_CSID_PKT_HDR_CORRUPTED       = 0x00000400,
	CAM_ISP_HW_ERROR_CSID_MISSING_PKT_HDR_DATA    = 0x00000800,
	CAM_ISP_HW_ERROR_CSID_SENSOR_SWITCH_ERROR     = 0x00001000,
	CAM_ISP_HW_ERROR_CSID_UNBOUNDED_FRAME         = 0x00002000,
	CAM_ISP_HW_ERROR_CSID_SENSOR_FRAME_DROP       = 0x00004000,
	CAM_ISP_HW_ERROR_CSID_MISSING_EOT             = 0x00008000,
	CAM_ISP_HW_ERROR_CSID_PKT_PAYLOAD_CORRUPTED   = 0x00010000,
	CAM_ISP_HW_ERROR_CSID_CAMIF_FRAME_DROP        = 0x00020000,
	CAM_ISP_HW_ERROR_HWPD_VIOLATION               = 0x00040000,
};

/**
 *  enum cam_isp_hw_stop_cmd - Specify the stop command type
 */
enum cam_isp_hw_stop_cmd {
	CAM_ISP_HW_STOP_AT_FRAME_BOUNDARY,
	CAM_ISP_HW_STOP_IMMEDIATELY,
	CAM_ISP_HW_STOP_MAX,
};

/**
 * struct cam_isp_stop_args - hardware stop arguments
 *
 * @hw_stop_cmd:               Hardware stop command type information
 * @is_internal_stop:          Stop triggered internally for reset & recovery
 * @stop_only:                 Send stop only to hw drivers. No Deinit to be
 *                             done.
 *
 */
struct cam_isp_stop_args {
	enum cam_isp_hw_stop_cmd      hw_stop_cmd;
	bool                          is_internal_stop;
	bool                          stop_only;
};

/**
 * struct cam_isp_clock_config_internal - Clock configuration
 *
 * @usage_type:                 Usage type (Single/Dual)
 * @num_rdi:                    Number of RDI votes
 * @left_pix_hz:                Pixel Clock for Left ISP
 * @right_pix_hz:               Pixel Clock for Right ISP, valid only if Dual
 * @rdi_hz:                     RDI Clock. ISP clock will be max of RDI and
 *                              PIX clocks. For a particular context which ISP
 *                              HW the RDI is allocated to is not known to UMD.
 *                              Hence pass the clock and let KMD decide.
 */
struct cam_isp_clock_config_internal {
	uint64_t                       usage_type;
	uint64_t                       num_rdi;
	uint64_t                       left_pix_hz;
	uint64_t                       right_pix_hz;
	uint64_t                       rdi_hz[CAM_IFE_RDI_NUM_MAX];
};

/**
 * struct cam_isp_bw_config_internal_v2 - Bandwidth configuration
 *
 * @usage_type:                 ife hw index
 * @num_paths:                  Number of data paths
 * @axi_path:                   per path vote info
 */
struct cam_isp_bw_config_internal_v2 {
	uint32_t                               usage_type;
	uint32_t                               num_paths;
	struct cam_cpas_axi_per_path_bw_vote   axi_path[CAM_ISP_MAX_PER_PATH_VOTES];
};

/**
 * struct cam_isp_bw_config_internal - Internal Bandwidth configuration
 *
 * @usage_type:                 Usage type (Single/Dual)
 * @num_rdi:                    Number of RDI votes
 * @left_pix_vote:              Bandwidth vote for left ISP
 * @right_pix_vote:             Bandwidth vote for right ISP
 * @rdi_vote:                   RDI bandwidth requirements
 */
struct cam_isp_bw_config_internal {
	uint32_t                       usage_type;
	uint32_t                       num_rdi;
	struct cam_isp_bw_vote         left_pix_vote;
	struct cam_isp_bw_vote         right_pix_vote;
	struct cam_isp_bw_vote         rdi_vote[CAM_IFE_RDI_NUM_MAX];
};

/**
 * struct cam_isp_bw_clk_config_info - Bw/Clk config info
 *
 * @bw_config  :            BW vote info for current request V1
 * @bw_config_v2:           BW vote info for current request V2
 * @bw_config_valid:        Flag indicating if BW vote is valid for current request
 * @ife_clock_config:       Clock config information for ife
 * @ife_clock_config_valid: Flag indicating whether clock config is valid for
 *                          current request for ife
 * @sfe_clock_config:       Clock config information for sfe
 * @sfe_clock_config_valid: Flag indicating whether clock config is valid for
 *                          current request for sfe
 */
struct cam_isp_bw_clk_config_info {
	struct cam_isp_bw_config_internal     bw_config;
	struct cam_isp_bw_config_internal_v2  bw_config_v2;
	bool                                  bw_config_valid;
	struct cam_isp_clock_config_internal  ife_clock_config;
	bool                                  ife_clock_config_valid;
	struct cam_isp_clock_config_internal  sfe_clock_config;
	bool                                  sfe_clock_config_valid;

};

/**
 * struct cam_isp_predict_fcg_config_internal - Internal FCG config in a single prediction
 *
 * @phase_index_g:              Starting index of LUT for G channel in phase
 * @phase_index_r:              Starting index of LUT for R channel in phase
 * @phase_index_b:              Starting index of LUT for B channel in phase
 * @stats_index_g:              Starting index of LUT for G channel in stats
 * @stats_index_r:              Starting index of LUT for R channel in stats
 * @stats_index_b:              Starting index of LUT for B channel in stats
 */
struct cam_isp_predict_fcg_config_internal {
	uint32_t                                phase_index_g;
	uint32_t                                phase_index_r;
	uint32_t                                phase_index_b;
	uint32_t                                stats_index_g;
	uint32_t                                stats_index_r;
	uint32_t                                stats_index_b;
};

/**
 * struct cam_isp_ch_ctx_fcg_config_internal - Internal FCG config in a single channel or context
 *
 * @fcg_ch_ctx_id:              Index of the channel in SFE/IFE or context in TFE
 *                              to be configured that FCG blocks reside on.
 *                              For example, if one wants to config FCG block
 *                              for IFE in ctx 0, this value will be CAM_ISP_FCG_MASK_CH0
 * @fcg_enable_mask:            Indicate which module will be enabled for
 *                              FCG. For example, if one wants to config
 *                              SFE FCG STATS module, CAM_ISP_FCG_ENABLE_STATS
 *                              will be set in mask
 * @predicted_fcg_configs:      FCG config for each prediction of the channel
 *                              in serial order
 */
struct cam_isp_ch_ctx_fcg_config_internal {
	uint32_t                                      fcg_ch_ctx_id;
	uint32_t                                      fcg_enable_mask;
	struct cam_isp_predict_fcg_config_internal    predicted_fcg_configs[
							CAM_ISP_MAX_FCG_PREDICTIONS];
};

/**
 * struct cam_isp_fcg_config_internal - Internal FCG config for a frame
 *
 * @num_ch_ctx:                 Number of channels for FCG config for SFE/IFE or
 *                              number of contexts for FCG config for TFE
 * @num_predictions:            Number of predictions for each channel
 * @num_types:                  Number of types(STATS/PHASE) for FCG config
 * @ch_ctx_fcg_configs:         FCG config for each channel or context
 */
struct cam_isp_fcg_config_internal {
	uint32_t                                      num_ch_ctx;
	uint32_t                                      num_predictions;
	uint32_t                                      num_types;
	struct cam_isp_ch_ctx_fcg_config_internal     ch_ctx_fcg_configs[
							CAM_ISP_MAX_FCG_CH_CTXS];
};

/**
 * struct cam_isp_fcg_config_info - Track FCG config for further usage in config stage
 *
 * @prediction_idx:            Indicate which exact prediction to be used, decided
 *                             during trying to apply the request
 * @sfe_fcg_entry_idx:         Index for SFE FCG config in hw update entries
 * @sfe_fcg_config:            Internal storage of SFE FCG configurations
 * @ife_fcg_entry_idx:         Index for IFE/MC_TFE FCG config in hw update entries
 * @ife_fcg_config:            Internal storage of IFE/MC_TFE FCG configurations
 * @use_current_cfg:           Indicate whether use current configuration or replace
 *                             the value with FCG predicted ones.
 * @sfe_fcg_online:            Indicate whether SFE FCG handling is online or not
 * @ife_fcg_online:            Indicate whether IFE/MC_TFE FCG handling is online or not
 */
struct cam_isp_fcg_config_info {
	uint32_t                                 prediction_idx;
	uint32_t                                 sfe_fcg_entry_idx;
	struct cam_isp_fcg_config_internal       sfe_fcg_config;
	uint32_t                                 ife_fcg_entry_idx;
	struct cam_isp_fcg_config_internal       ife_fcg_config;
	bool                                     use_current_cfg;
	bool                                     sfe_fcg_online;
	bool                                     ife_fcg_online;
};

/**
 * struct cam_isp_prepare_hw_update_data - hw prepare data
 *
 * @isp_mgr_ctx:            ISP HW manager Context for current request
 * @packet_opcode_type:     Packet header opcode in the packet header
 *                          this opcode defines, packet is init packet or
 *                          update packet
 * @frame_header_cpu_addr:  Frame header cpu addr
 * @frame_header_iova:      Frame header iova
 * @frame_header_res_id:    Out port res_id corresponding to frame header
 * @bw_clk_config:          BW and clock config info
 * @isp_drv_config:         DRV config info
 * @bw_config_valid:        Flag indicating if DRV config is valid for current request
 * @isp_irq_comp_cfg:       IRQ comp configuration for MC-based TFEs
 * @irq_comp_cfg_valid:     Flag indicating if IRQ comp cfg is valid for current request
 * @reg_dump_buf_desc:     cmd buffer descriptors for reg dump
 * @num_reg_dump_buf:      Count of descriptors in reg_dump_buf_desc
 * @packet:                CSL packet from user mode driver
 * @mup_val:               MUP value if configured
 * @num_exp:               Num of exposures
 * @mup_en:                Flag if dynamic sensor switch is enabled
 * @fcg_info:              Track FCG config for further usage in config stage
 *
 */
struct cam_isp_prepare_hw_update_data {
	void                                 *isp_mgr_ctx;
	uint32_t                              packet_opcode_type;
	uint32_t                             *frame_header_cpu_addr;
	uint64_t                              frame_header_iova;
	uint32_t                              frame_header_res_id;
	struct cam_isp_bw_clk_config_info     bw_clk_config;
	struct cam_isp_drv_config             isp_drv_config;
	bool                                  drv_config_valid;
	struct cam_isp_irq_comp_cfg           isp_irq_comp_cfg;
	bool                                  irq_comp_cfg_valid;
	struct cam_cmd_buf_desc               reg_dump_buf_desc[
						CAM_REG_DUMP_MAX_BUF_ENTRIES];
	uint32_t                              num_reg_dump_buf;
	struct cam_packet                    *packet;
	struct cam_kmd_buf_info               kmd_cmd_buff_info;
	uint32_t                              mup_val;
	uint32_t                              num_exp;
	bool                                  mup_en;
	struct cam_isp_fcg_config_info        fcg_info;
};


/**
 * struct cam_isp_hw_sof_event_data - Event payload for CAM_HW_EVENT_SOF
 *
 * @timestamp           : Time stamp for the sof event
 * @boot_time           : Boot time stamp for the sof event
 *
 */
struct cam_isp_hw_sof_event_data {
	uint64_t       timestamp;
	uint64_t       boot_time;
};

/**
 * struct cam_isp_hw_reg_update_event_data - Event payload for
 *                         CAM_HW_EVENT_REG_UPDATE
 *
 * @timestamp:     Time stamp for the reg update event
 *
 */
struct cam_isp_hw_reg_update_event_data {
	uint64_t       timestamp;
};

/**
 * struct cam_isp_hw_epoch_event_data - Event payload for CAM_HW_EVENT_EPOCH
 *
 * @timestamp:     Time stamp for the epoch event
 * @frame_id_meta: Frame id value corresponding to this frame
 */
struct cam_isp_hw_epoch_event_data {
	uint64_t       timestamp;
	uint32_t       frame_id_meta;
};

/**
 * struct cam_isp_hw_done_event_data - Event payload for CAM_HW_EVENT_DONE
 *
 * @hw_type:             Hw type sending the event
 * @resource_handle:     Resource handle
 * @comp_group_id:       Bus comp group id
 * @last_consumed_addr:  Last consumed addr
 * @timestamp:           Timestamp for the buf done event
 *
 */
struct cam_isp_hw_done_event_data {
	uint32_t             hw_type;
	uint32_t             resource_handle;
	uint32_t             comp_group_id;
	uint32_t             last_consumed_addr;
	uint64_t             timestamp;
};

/**
 * struct cam_isp_hw_eof_event_data - Event payload for CAM_HW_EVENT_EOF
 *
 * @timestamp:             Timestamp for the eof event
 *
 */
struct cam_isp_hw_eof_event_data {
	uint64_t       timestamp;
};

/**
 * struct cam_isp_hw_error_event_data - Event payload for CAM_HW_EVENT_ERROR
 *
 * @error_type:            HW error type for the error event
 * @timestamp:             Timestamp for the error event
 * @recovery_enabled:      Identifies if the context needs to recover & reapply
 *                         this request
 * @enable_req_dump:       Enable request dump on HW errors
 * @try_internal_recovery: Enable internal recovery on HW errors
 */
struct cam_isp_hw_error_event_data {
	uint32_t             error_type;
	uint64_t             timestamp;
	bool                 recovery_enabled;
	bool                 enable_req_dump;
	bool                 try_internal_recovery;
};

/**
 * struct cam_isp_hw_secondary_event_data - Event payload for secondary events
 *
 * @evt_type     : Event notified as secondary
 *
 */
struct cam_isp_hw_secondary_event_data {
	enum cam_isp_hw_secondary_event_type  evt_type;
};

/* enum cam_isp_hw_mgr_command - Hardware manager command type */
enum cam_isp_hw_mgr_command {
	CAM_ISP_HW_MGR_CMD_IS_RDI_ONLY_CONTEXT,
	CAM_ISP_HW_MGR_CMD_PAUSE_HW,
	CAM_ISP_HW_MGR_CMD_RESUME_HW,
	CAM_ISP_HW_MGR_CMD_SOF_DEBUG,
	CAM_ISP_HW_MGR_CMD_CTX_TYPE,
	CAM_ISP_HW_MGR_GET_PACKET_OPCODE,
	CAM_ISP_HW_MGR_GET_LAST_CDM_DONE,
	CAM_ISP_HW_MGR_CMD_PROG_DEFAULT_CFG,
	CAM_ISP_HW_MGR_GET_SOF_TS,
	CAM_ISP_HW_MGR_DUMP_STREAM_INFO,
	CAM_ISP_HW_MGR_GET_BUS_COMP_GROUP,
	CAM_ISP_HW_MGR_CMD_MAX,
};

enum cam_isp_ctx_type {
	CAM_ISP_CTX_FS2 = 1,
	CAM_ISP_CTX_RDI,
	CAM_ISP_CTX_PIX,
	CAM_ISP_CTX_OFFLINE,
	CAM_ISP_CTX_MAX,
};
/**
 * struct cam_isp_hw_cmd_args - Payload for hw manager command
 *
 * @cmd_type:              HW command type
 * @cmd_data:              Command data
 * @sof_irq_enable:        To debug if SOF irq is enabled
 * @ctx_type:              RDI_ONLY, PIX and RDI, or FS2
 * @packet_op_code:        Packet opcode
 * @last_cdm_done:         Last cdm done request
 * @sof_ts:                SOF timestamps (current, boot and previous)
 * @cdm_done_ts:           CDM callback done timestamp
 */
struct cam_isp_hw_cmd_args {
	uint32_t                          cmd_type;
	void                             *cmd_data;
	union {
		uint32_t                         sof_irq_enable;
		uint32_t                         ctx_type;
		uint32_t                         packet_op_code;
		uint64_t                         last_cdm_done;
		struct {
			uint64_t                      curr;
			uint64_t                      prev;
			uint64_t                      boot;
		} sof_ts;
	} u;
	struct timespec64 cdm_done_ts;
};

/**
 * struct cam_isp_start_args - isp hardware start arguments
 *
 * @config_args:               Hardware configuration commands.
 * @is_internal_start:         Start triggered internally for reset & recovery
 * @start_only                 Send start only to hw drivers. No init to
 *                             be done.
 *
 */
struct cam_isp_start_args {
	struct cam_hw_config_args hw_config;
	bool                      is_internal_start;
	bool                      start_only;
};

/**
 * struct cam_isp_lcr_rdi_cfg_args - isp hardware start arguments
 *
 * @rdi_lcr_cfg:            RDI LCR cfg received from User space.
 * @ife_src_res_id:         IFE SRC res id to be used in sfe context
 * @is_init:                Flag to indicate if init packet.
 *
 */
struct cam_isp_lcr_rdi_cfg_args {
	struct cam_isp_lcr_rdi_config *rdi_lcr_cfg;
	uint32_t                       ife_src_res_id;
	bool                           is_init;
};

/**
 * cam_isp_hw_mgr_init()
 *
 * @brief:              Initialization function for the ISP hardware manager
 *
 * @device_name_str:    Device name string
 * @hw_mgr:             Input/output structure for the ISP hardware manager
 *                          initialization
 * @iommu_hdl:          Iommu handle to be returned
 * @isp_device_type:    ISP device type
 */
int cam_isp_hw_mgr_init(const char    *device_name_str,
	struct cam_hw_mgr_intf *hw_mgr, int *iommu_hdl, uint32_t isp_device_type);

void cam_isp_hw_mgr_deinit(const char *device_name_str);

#endif /* __CAM_ISP_HW_MGR_INTF_H__ */
