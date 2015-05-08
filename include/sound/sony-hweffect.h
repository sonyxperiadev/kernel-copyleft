/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * Author: Yoshio Yamamoto yoshio.xa.yamamoto@sonymobile.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SONY_HWEFFECT_H
#define _SONY_HWEFFECT_H

/** SONY HW EFFECT **/

/* CONFIG GET/SET */
#define CONFIG_CACHE        0
#define CONFIG_SET          1
#define CONFIG_GET          2

/* CONFIG HEADER */
/*

	MODULE_ID,
	DEVICE,
	NUM_COMMANDS,
	COMMAND_ID_1,
	CONFIG_CACHE/SET/GET,
	OFFSET_1,
	LENGTH_1,
	VALUES_1,
	...,
	...,
	COMMAND_ID_2,
	CONFIG_CACHE/SET/GET,
	OFFSET_2,
	LENGTH_2,
	VALUES_2,
	...,
	...,
	COMMAND_ID_3,
	...
*/

#define BAND_NUM        6

/* CONFIG PARAM IDs */
#define SONYBUNDLE_MODULE           0x00011000

#define SONYBUNDLE_ENABLE           0x00011001

#define DYNAMIC_NORMALIZER_ENABLE   0x00011011

#define SFORCE_ENABLE               0x00011021
#define SFORCE_PARAM_TYPE           0x00011022

#define VPT20_MODE                  0x00011031

#define CLEARPHASE_HP_MODE          0x00011041

#define CLEARAUDIO_CHSEP            0x00011051
#define CLEARAUDIO_EQ_COEF          0x00011052
#define CLEARAUDIO_VOLUME           0x00011053

#define CLEARPHASE_SP_ENABLE        0x00011061

#define XLOUD_ENABLE                0x00011071

#define SONYBUNDLE_ENABLE_PARAM_LEN         1
#define DYNAMIC_NORMALIZER_ENABLE_PARAM_LEN 1
#define SFORCE_ENABLE_PARAM_LEN             1
#define VPT20_MODE_PARAM_LEN                1
#define CLEARPHASE_HP_MODE_PARAM_LEN        1
#define CLEARAUDIO_CHSEP_PARAM_LEN          1
#define CLEARAUDIO_EQ_COEF_PARAM_LEN        6
#define CLEARAUDIO_VOLUME_PARAM_LEN         1
#define CLEARPHASE_SP_ENABLE_PARAM_LEN      1
#define XLOUD_ENABLE_PARAM_LEN              1

#define COMMAND_PAYLOAD_LEN 3
#define COMMAND_PAYLOAD_SZ  (COMMAND_PAYLOAD_LEN * sizeof(uint32_t))
#define MAX_INBAND_PARAM_SZ 4096

#define SONYBUNDLE_ENABLE_PARAM_SZ  \
			(SONYBUNDLE_ENABLE_PARAM_LEN*sizeof(uint32_t))
#define DYNAMIC_NORMALIZER_ENABLE_PARAM_SZ \
			(DYNAMIC_NORMALIZER_ENABLE_PARAM_LEN*sizeof(uint32_t))
#define SFORCE_ENABLE_PARAM_SZ  \
			(SFORCE_ENABLE_PARAM_LEN*sizeof(uint32_t))
#define VPT20_MODE_PARAM_SZ \
			(VPT20_MODE_PARAM_LEN*sizeof(uint32_t))
#define CLEARPHASE_HP_MODE_PARAM_SZ \
			(CLEARPHASE_HP_MODE_PARAM_LEN*sizeof(uint32_t))
#define CLEARAUDIO_CHSEP_PARAM_SZ   \
			(CLEARAUDIO_CHSEP_PARAM_LEN*sizeof(uint32_t))
#define CLEARAUDIO_EQ_COEF_PARAM_SZ \
			(CLEARAUDIO_EQ_COEF_PARAM_LEN*sizeof(int16_t))
#define CLEARAUDIO_VOLUME_PARAM_SZ  \
			(CLEARAUDIO_VOLUME_PARAM_LEN*sizeof(int32_t))
#define CLEARPHASE_SP_ENABLE_PARAM_SZ   \
			(CLEARPHASE_SP_ENABLE_PARAM_LEN*sizeof(uint32_t))
#define XLOUD_ENABLE_PARAM_SZ   \
			(XLOUD_ENABLE_PARAM_LEN*sizeof(uint32_t))

struct common_params {
	uint16_t enable_flag;
	uint16_t reserved;
};

struct dynamic_normalizer_params {
	uint16_t enable_flag;
	uint16_t reserved;
};

struct sforce_params {
	uint16_t enable_flag;
	uint16_t reserved;
};

struct vpt20_params {
	uint16_t mode;
	uint16_t reserved;
};

struct clearphase_hp_params {
	uint16_t mode;
	uint16_t reserved;
};

struct clearaudio_params {
	int32_t  chsep_coef;
	int16_t  eq_coef[6];
};

struct clearaudio_volume {
	uint16_t gain;
	uint16_t reserved;
};

struct clearphase_sp_params {
	uint16_t mode;
	uint16_t reserved;
};

struct xloud_params {
	uint16_t enable_flag;
	uint16_t reserved;
};

struct sonybundle_params {
	struct common_params common;
	struct dynamic_normalizer_params dynamic_normalizer;
	struct sforce_params sforce;
	bool sforce_tuning_update;
	struct vpt20_params vpt20;
	struct clearphase_hp_params clearphase_hp;
	struct clearaudio_params clearaudio;
	struct clearaudio_volume ca_volume;
	struct clearphase_sp_params clearphase_sp;
	bool clearphase_sp_tuning_update;
	struct xloud_params xloud;
	bool xloud_tuning_update;
};

#endif /*_SONYEFFECT_HW_H */
