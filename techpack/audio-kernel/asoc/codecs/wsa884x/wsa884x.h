/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _WSA884X_H
#define _WSA884X_H

#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/info.h>
#include "wsa884x-reg-masks.h"
#include "wsa884x-reg-shifts.h"


#define WSA884X_MAX_SWR_PORTS   6

#if IS_ENABLED(CONFIG_SND_SOC_WSA884X)
int wsa884x_set_channel_map(struct snd_soc_component *component,
				   u8 *port, u8 num_port, unsigned int *ch_mask,
				   unsigned int *ch_rate, u8 *port_type);


int wsa884x_codec_info_create_codec_entry(
					struct snd_info_entry *codec_root,
					struct snd_soc_component *component);
int wsa884x_codec_get_dev_num(struct snd_soc_component *component);
#else
static int wsa884x_set_channel_map(struct snd_soc_component *component,
				   u8 *port, u8 num_port, unsigned int *ch_mask,
				   unsigned int *ch_rate, u8 *port_type)
{
	return 0;
}

static int wsa884x_codec_info_create_codec_entry(
					struct snd_info_entry *codec_root,
					struct snd_soc_component *component)
{
	return 0;
}

static int wsa884x_codec_get_dev_num(struct snd_soc_component *component)
{
	return 0;
}
#endif

#endif /* _WSA884X_H */
