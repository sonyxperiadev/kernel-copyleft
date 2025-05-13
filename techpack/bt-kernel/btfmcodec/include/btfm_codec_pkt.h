// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_BTFM_CODEC_PKT_H
#define __LINUX_BTFM_CODEC_PKT_H

typedef uint32_t btm_opcode;

struct btm_req {
	btm_opcode opcode;
	uint32_t len;
	uint8_t *data;
}__attribute__((packed));

struct btm_rsp {
	btm_opcode opcode;
	uint8_t status;
}__attribute__((packed));

struct btm_ind {
	btm_opcode opcode;
	uint32_t len;
	uint8_t *data;
}__attribute__((packed));

struct btm_ctrl_pkt {
	btm_opcode opcode;
	uint32_t len;
	uint8_t active_transport;
	uint8_t status;
}__attribute__((packed));

#define BTM_BTFMCODEC_PREPARE_AUDIO_BEARER_SWITCH_REQ           0x50000000
#define BTM_BTFMCODEC_PREPARE_AUDIO_BEARER_SWITCH_RSP           0x50000001
#define BTM_BTFMCODEC_MASTER_CONFIG_REQ                         0x50000002
#define BTM_BTFMCODEC_MASTER_CONFIG_RSP                         0x50000003
#define BTM_BTFMCODEC_MASTER_SHUTDOWN_REQ                       0x50000004
#define BTM_BTFMCODEC_CTRL_MASTER_SHUTDOWN_RSP                  0x50000005
#define BTM_BTFMCODEC_CODEC_CONFIG_DMA_REQ                      0x58000006
#define BTM_BTFMCODEC_CODEC_CONFIG_DMA_RSP                      0x58000007

#define BTM_BTFMCODEC_BEARER_SWITCH_IND                         0x58000001
#define BTM_BTFMCODEC_TRANSPORT_SWITCH_FAILED_IND               0x58000002
#define BTM_BTFMCODEC_ADSP_STATE_IND                            0x58000003
#define BTM_BTFMCODEC_CTRL_LOG_LVL_IND                          0x58000004

#define BTM_MASTER_CONFIG_REQ_LEN			13
#define BTM_MASTER_CONFIG_RSP_TIMEOUT			5000
#define BTM_MASTER_DMA_CONFIG_RSP_TIMEOUT		5000
#define BTM_HEADER_LEN					8
#define BTM_PREPARE_AUDIO_BEARER_SWITCH_RSP_LEN		2
#define BTM_MASTER_CONFIG_RSP_LEN			2
#define BTM_CODEC_CONFIG_DMA_RSP_LEN			2
#define BTM_MASTER_SHUTDOWN_REQ_LEN			1
#define BTM_PREPARE_AUDIO_BEARER_SWITCH_REQ_LEN		1
#define BTM_BEARER_SWITCH_IND_LEN			1
#define BTM_LOG_LVL_IND_LEN                             1
#define BTM_ADSP_STATE_IND_LEN				4
#define BTM_CODEC_CONFIG_DMA_REQ_LEN			11

#define BTM_BTFMCODEC_USECASE_START_IND			0x58000008
#define BTM_USECASE_START_IND_LEN                       1

enum rx_status {
	/* Waiting for response */
	BTM_WAITING_RSP,
	/* Response recevied */
	BTM_RSP_RECV,
	/* Response recevied with failure status*/
	BTM_FAIL_RESP_RECV,
	/* Response not recevied, but client killed */
        BTM_RSP_NOT_RECV_CLIENT_KILLED,
};

enum btfm_kp_status {
	/* KP processed message succesfully */
	MSG_SUCCESS = 0,
	/* Error while processing the message */
	MSG_FAILED,
	/* Wrong transport type selected by BTADV audio manager */
	MSG_WRONG_TRANSPORT_TYPE,
	/* Timeout triggered to receive bearer switch indications*/
	MSG_INTERNAL_TIMEOUT,
	MSG_FAILED_TO_CONFIGURE_HWEP,
	MSG_FAILED_TO_SHUTDOWN_HWEP,
	MSG_ERR_WHILE_SHUTING_DOWN_HWEP,
};

struct btm_master_config_req {
	btm_opcode opcode;
	uint32_t len;
	uint8_t stream_id;
	uint32_t device_id;
	uint32_t sample_rate;
	uint8_t bit_width;
	uint8_t num_channels;
	uint8_t channel_num;
	uint8_t codec_id;
}__attribute__((packed));

struct btm_dma_config_req {
	btm_opcode opcode;
	uint32_t len;
	uint8_t stream_id;
	uint32_t sample_rate;
	uint8_t bit_width;
	uint8_t num_channels;
	uint8_t codec_id;
	uint8_t lpaif;     // Low power audio interface
	uint8_t inf_index; // interface index
	uint8_t active_channel_mask;
} __packed;

struct btm_usecase_start_ind {
	btm_opcode opcode;
	uint32_t len;
	uint8_t transport;
} __packed;

struct btm_master_shutdown_req {
	btm_opcode opcode;
	uint32_t len;
	uint8_t stream_id;
}__attribute__((packed));

struct btm_adsp_state_ind {
	btm_opcode opcode;
	uint32_t len;
	uint32_t action;
} __attribute__((packed));

int btfmcodec_dev_enqueue_pkt(struct btfmcodec_char_device *, void *, int);
bool btfmcodec_is_valid_cache_avb(struct btfmcodec_data *);
#endif /* __LINUX_BTFM_CODEC_PKT_H*/
