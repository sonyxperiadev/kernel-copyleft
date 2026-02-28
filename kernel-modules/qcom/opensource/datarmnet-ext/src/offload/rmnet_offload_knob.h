/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __RMNET_OFFLOAD_KNOB_H__
#define __RMNET_OFFLOAD_KNOB_H__

#include <linux/types.h>

enum {
	RMNET_OFFLOAD_KNOB_TCP_BYTE_LIMIT,
	RMNET_OFFLOAD_KNOB_UDP_BYTE_LIMIT,
	RMNET_OFFLOAD_KNOB_ENGINE_MODE,
	RMNET_OFFLOAD_KNOB_MAX,
};

u64 rmnet_offload_knob_get(u32 knob);

#endif
