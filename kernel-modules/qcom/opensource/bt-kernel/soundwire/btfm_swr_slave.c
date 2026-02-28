// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#include "btfm_swr.h"
#include "btfm_swr_slave.h"

struct soc_port_mapping slave_port[] = {
	// Evros
	{
	.ea = EVROS_EA,
	.port_info[FMAUDIO_TX].dai_id = FMAUDIO_TX,
	.port_info[FMAUDIO_TX].port = 5,

	.port_info[BTAUDIO_TX].dai_id = BTAUDIO_TX,
	.port_info[BTAUDIO_TX].port = 3,

	.port_info[BTAUDIO_RX].dai_id = BTAUDIO_RX,
	.port_info[BTAUDIO_RX].port = 1,

	.port_info[BTAUDIO_RX2].dai_id = BTAUDIO_RX2,
	.port_info[BTAUDIO_RX2].port = 2,

	.port_info[BTAUDIO_TX2].dai_id = BTAUDIO_TX2,
	.port_info[BTAUDIO_TX2].port = 4,
	},

	// Ganges
	{
	.ea = GANGES_EA,
	// FM is not supported on Ganges. populate with invalid port number
	.port_info[FMAUDIO_TX].dai_id = FMAUDIO_TX,
	.port_info[FMAUDIO_TX].port = BTFM_INVALID_PORT,

	.port_info[BTAUDIO_TX].dai_id = BTAUDIO_TX,
	.port_info[BTAUDIO_TX].port = 4,

	.port_info[BTAUDIO_RX].dai_id = BTAUDIO_RX,
	.port_info[BTAUDIO_RX].port = 1,

	.port_info[BTAUDIO_RX2].dai_id = BTAUDIO_RX2,
	.port_info[BTAUDIO_RX2].port = 2,

	.port_info[BTAUDIO_TX2].dai_id = BTAUDIO_TX2,
	.port_info[BTAUDIO_TX2].port = 5,
	},
};

