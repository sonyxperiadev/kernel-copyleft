/*
 * Author: Yoshio Yamamoto yoshio.xa.yamamoto@sonymobile.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __SONY_HWEFFECT_PARAMS_H
#define __SONY_HWEFFECT_PARAMS_H

#define SONYEFFECT_HW_PARAMS_IOCTL_MAGIC 't'

#define SFORCE_PARAM \
	_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 0, unsigned)

#define CLEARPHASE_HP_PARAM \
	_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 1, unsigned)

#define CLEARPHASE_SP_PARAM \
	_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 2, unsigned)

#define XLOUD_PARAM \
	_IOW(SONYEFFECT_HW_PARAMS_IOCTL_MAGIC, 3, unsigned)

#define SFORCE_PARAM_SIZE 1016
#define CLEARPHASE_HP_PARAM_SIZE 2064
#define CLEARPHASE_SP_PARAM_SIZE 2360
#define XLOUD_PARAM_SIZE 512

enum {
	SFORCE_TYPE_MUSIC = 0,
	SFORCE_TYPE_VIDEO,
	SFORCE_TYPE_MAX
};

struct sforce_param_data {
	unsigned char data[SFORCE_PARAM_SIZE];
};

struct clearphase_hp_param_data {
	unsigned char data[CLEARPHASE_HP_PARAM_SIZE];
};

struct clearphase_sp_param_data {
	unsigned char data[CLEARPHASE_SP_PARAM_SIZE];
};

struct xloud_param_data {
	unsigned int level;
	unsigned char data[XLOUD_PARAM_SIZE];
};

void *sony_hweffect_params_getparam(unsigned int effect_id);

#endif
