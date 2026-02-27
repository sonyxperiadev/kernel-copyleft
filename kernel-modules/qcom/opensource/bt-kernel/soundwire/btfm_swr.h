// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#ifndef BTFM_SWR_H
#define BTFM_SWR_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <bindings/audio-codec-port-types.h>
#include "soc/soundwire.h"

#define SWR_SLAVE_COMPATIBLE_STR	"btfmswr_slave"


#define BTFMSWR_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n", __func__, ## arg)
#define BTFMSWR_INFO(fmt, arg...) pr_info("%s: " fmt "\n", __func__, ## arg)
#define BTFMSWR_ERR(fmt, arg...)  pr_err("%s: " fmt "\n", __func__, ## arg)

extern struct btfmswr *pbtfmswr;

// assumption is that we use adjacent channels
#define ONE_CHANNEL_MASK 1
#define TWO_CHANNEL_MASK 3

#define MAX_BT_PORTS 1

/* Codec driver defines */
enum {
	FMAUDIO_TX = 0,
	BTAUDIO_TX,
	BTAUDIO_RX,
	BTAUDIO_A2DP_SINK_TX,
	BTFM_NUM_CODEC_DAIS
};

enum {
	EVROS = 0,
	GANGES = 1,
	MAX_SOC_ID = 0xFF
};


struct btfmswr_dai_port_info {
	int dai_id;
	char *dai_name;
	uint8_t port;
};

struct soc_port_mapping {
	// enumeration address of BT SOC
	u64 ea;
	struct btfmswr_dai_port_info port_info[BTFM_NUM_CODEC_DAIS];
};


struct btfmswr {
	struct device *dev;
	struct swr_device *swr_slave;
	bool initialized;
	uint32_t sample_rate;
	uint32_t bps;
	uint16_t direction;
	uint8_t num_channels;
	int soc_index;
	struct soc_port_mapping *p_dai_port;
};

/**
 * btfm_swr_hw_init: Initialize soundwire slave device
 * Returns:
 * 0: Success
 * else: Fail
 */
int btfm_swr_hw_init(void);

int btfm_get_bt_soc_index(int chipset_ver);

int btfm_swr_enable_port(u8 port_num, u8 ch_count, u32 sample_rate,
									u8 port_type);


int btfm_swr_disable_port(u8 port_num, u8 ch_count, u8 usecase);
#endif /* BTFM_SWR_H */
