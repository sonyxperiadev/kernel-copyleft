/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2020-03-23 File created.
 */

#ifndef __FSM_ALGO_H__
#define __FSM_ALGO_H__

#include "fsm-dev.h"

#define AFE_MODULE_ID_FSADSP_RX (0x10001111)
#define AFE_MODULE_ID_FSADSP_TX (0x10001112)

enum fsm_opcode {
	FSM_OPCODE_GET_PARAMS = 0,
	FSM_OPCODE_SET_PARAMS,
	FSM_OPCODE_MAX
};

enum fsadsp_cfg {
	FSADSP_CFG_RE25 = 1,
	FSADSP_CFG_CLIPPING,
	FSADSP_CFG_APP_MODE,
	FSADSP_CFG_TX_FMT,
	FSADSP_CFG_FADE_OUT,
	FSADSP_CFG_RSTRIM,
	FSADSP_CFG_MAX
};

enum capi_v2_param {
	CAPI_V2_PARAM_FSADSP_BASE = 0x10001FA0,
	CAPI_V2_PARAM_FSADSP_MODULE_ENABLE, // enable rx or tx module
	CAPI_V2_PARAM_FSADSP_MODULE_PARAM,	// send preset to adsp
	CAPI_V2_PARAM_FSADSP_APP_MODE,		// set rx effect mode(scene)
	CAPI_V2_PARAM_FSADSP_CLIPPING_EST,	// 
	CAPI_V2_PARAM_FSADSP_TX_FMT,		// tx format
	CAPI_V2_PARAM_FSADSP_LIVEDATA,		// livedata
	CAPI_V2_PARAM_FSADSP_RE25,			// re25
	CAPI_V2_PARAM_FSADSP_CFG,			// configuration
};

#define FSADSP_CMD_HDR (0xA)
#define FSADSP_PARAM_CMD(TYPE, SIZE, CHN) \
	((FSADSP_CMD_HDR << 24) | ((SIZE) << 16) | ((CHN) << 8) | (TYPE))

enum fsm_port_idx {
	FSM_PORT_ZERO = 0,
	FSM_PORT_PRI_MI2S,
	FSM_PORT_SEC_MI2S,
	FSM_PORT_TERT_MI2S,
	FSM_PORT_QUAT_MI2S,
	FSM_PORT_QUIN_MI2S,
	FSM_PORT_PRI_TDM,
	FSM_PORT_SEC_TDM,
	FSM_PORT_TERT_TDM,
	FSM_PORT_QUAT_TDM,
};

struct afe_fsm_header {
	uint32_t module_id;
	uint32_t param_id;
	uint32_t rx_port;
	uint32_t tx_port;
	uint32_t opcode;
	uint32_t payload_size;
	uint32_t cmd_size;
	void *buf;
	uint32_t buf_size;
};

struct afe_fsm_params {
	uint32_t payload[FSM_PAYLOAD_SIZE];
} __packed;

struct afe_fsm_port_config {
	bool enable;
	bool stereo;
	int rx_port;
	int tx_port;
};

#ifdef CONFIG_FSM_QCOM_NONDSP
#ifdef CONFIG_FSM_Q820_EVB
#include <sound/apr_audio-v2.h>
#define INSTANCE_ID_0 0x0000
struct mem_mapping_hdr {
	u32 data_payload_addr_lsw;
	u32 data_payload_addr_msw;
	u32 mem_map_handle;
} __packed;

struct param_hdr_v1 {
	uint32_t module_id;
	uint32_t param_id;
	uint16_t param_size;
	uint16_t reserved;
} __packed;

struct param_hdr_v3 {
	uint32_t module_id;
	uint16_t instance_id;
	uint16_t reserved;
	uint32_t param_id;
	uint32_t param_size;
} __packed;

struct afe_port_fsm_get_param_v2 {
	struct apr_hdr apr_hdr;
	u16 port_id;
	u16 payload_size;
	struct mem_mapping_hdr mem_hdr;
	u32 module_id;
	u32 param_id;
	struct param_hdr_v1 param_hdr;
} __packed;

struct afe_port_fsm_set_param_v2 {
	struct apr_hdr apr_hdr;
	u16 port_id;
	u16 payload_size;
	struct mem_mapping_hdr mem_hdr;
	u8 param_data[0];
} __packed;
bool q6common_is_instance_id_supported(void);
#else // CONFIG_FSM_Q820_EVB
#include <dsp/apr_audio-v2.h>
struct afe_port_param_data_v2 {
	u32 module_id;
	u32 param_id;
	u16 param_size;
	u16 reserved;
} __packed;
#endif

struct afe_port_cmd_get_fsm_params {
	struct apr_hdr hdr;
	struct afe_port_cmd_get_param_v2 get_param;
	struct afe_port_param_data_v2 pdata; // param_hdr
	struct afe_fsm_params fsm_params;
} __packed;

struct afe_port_cmd_set_fsm_params {
	struct apr_hdr hdr;
	struct afe_port_cmd_set_param_v2 set_param;
	struct afe_port_param_data_v2 pdata; // param_hdr
	struct afe_fsm_params fsm_params;
} __packed;

struct afe_get_fsm_param_resp {
	uint32_t status;
	struct afe_fsm_params fsm_params;
} __packed;

struct afe_get_fsm_param_resp_v2 {
	uint32_t status;
	struct afe_port_param_data_v2 param_hdr;
	struct afe_fsm_params fsm_params;
} __packed;

struct afe_get_fsm_param_resp_v3 {
	uint32_t status;
	struct param_hdr_v3 param_hdr;
	struct afe_fsm_params fsm_params;
} __packed;

int afe_set_fsm_iv_fb_port(struct afe_fsm_port_config *port_cfg);
int afe_send_fsm_apr_v2(struct afe_fsm_header *hdr, fsm_msg_t *data);
bool afe_fsm_algo_callback(uint32_t *payload,
				uint32_t payload_size);
#endif // CONFIG_FSM_QCOM_NONDSP

#endif
