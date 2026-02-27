// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_BTFM_CODEC_BTADV_INTERFACE_H
#define __LINUX_BTFM_CODEC_BTADV_INTERFACE_H

enum transport_type {
	BT = 1,
	BTADV,
	NONE,
};

static char *transport_type_text[] = {"BT", "BTADV", "NONE"};

void btfmcodec_set_current_state(struct btfmcodec_state_machine *, btfmcodec_state);
void btfmcodec_wq_prepare_bearer(struct work_struct *);
void btfmcodec_wq_hwep_shutdown(struct work_struct *);
void btfmcodec_initiate_hwep_shutdown(struct btfmcodec_char_device *btfmcodec_dev);
btfmcodec_state btfmcodec_get_current_transport(struct btfmcodec_state_machine *state);
#endif /* __LINUX_BTFM_CODEC_BTADV_INTERFACE_H */
