/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2014, 2016-2017, 2019-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __RMNET_OFFLOAD_STATE_H__
#define __RMNET_OFFLOAD_STATE_H__

#include "rmnet_offload_engine.h"
#include "rmnet_map.h"
#include "rmnet_descriptor.h"
#include "qmi_rmnet.h"

enum {
	RMNET_OFFLOAD_MGMT_SUCCESS,
	RMNET_OFFLOAD_MGMT_PARTIAL,
	RMNET_OFFLOAD_MGMT_FAILURE,
};

struct rmnet_offload_dl_marker_state {
	struct rmnet_map_dl_ind dl_ind;
	u32 dl_marker_seq;
	u32 dl_marker_pkts;
	bool dl_marker_cb_registered;
	bool dl_marker_start;
};

struct rmnet_offload_state {
	struct rmnet_port *core_port;
	struct rmnet_offload_dl_marker_state dl_marker_state;
	struct qmi_rmnet_ps_ind powersave_ind;
	struct rmnet_offload_engine_state engine_state;
	u8 rmnet_offload_vnd_count;
};

struct rmnet_offload_state *rmnet_offload_state_get(void);

#endif
