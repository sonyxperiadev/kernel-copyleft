// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#include "btfm_swr.h"
#include "btfm_swr_slave.h"

struct soc_port_mapping slave_port[] = {
	// Evros
	{
	.ea = EVROS_EA,
	.port_info[0].dai_id = FMAUDIO_TX,
	.port_info[0].port = 5,

	.port_info[1].dai_id = BTAUDIO_TX,
	.port_info[1].port = 3,

	.port_info[2].dai_id = BTAUDIO_RX,
	.port_info[2].port = 1,

	.port_info[3].dai_id = BTAUDIO_A2DP_SINK_TX,
	.port_info[3].port = 4,
	},

	// Ganges
	{
	.ea = GANGES_EA,
	// FM is not supported on Ganges. populate with invalid port number
	.port_info[0].dai_id = FMAUDIO_TX,
	.port_info[0].port = BTFM_INVALID_PORT,

	.port_info[1].dai_id = BTAUDIO_TX,
	.port_info[1].port = 4,

	.port_info[2].dai_id = BTAUDIO_RX,
	.port_info[2].port = 1,

	.port_info[3].dai_id = BTAUDIO_A2DP_SINK_TX,
	.port_info[3].port = 5,
	},
};

