/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __RMNET_OFFLOAD_UDP_H__
#define __RMNET_OFFLOAD_UDP_H__

#include "rmnet_offload_main.h"
#include "rmnet_offload_engine.h"

bool rmnet_offload_engine_udp_ingress(struct rmnet_offload_flow *flow,
				      struct rmnet_offload_info *pkt,
				      bool force_flush,
				      struct list_head *flush_list);

#endif
