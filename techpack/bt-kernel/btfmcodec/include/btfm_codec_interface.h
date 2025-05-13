// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_BTFM_CODEC_INTERFACE
#define __LINUX_BTFM_CODEC_INTERFACE

#include "btfm_codec_hw_interface.h"
int btfm_register_codec(struct hwep_data *hwep_info);
void btfm_unregister_codec(void);
#endif /*__LINUX_BTFM_CODEC_INTERFACE */
