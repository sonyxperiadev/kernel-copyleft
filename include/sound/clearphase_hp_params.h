/* Copyright (C) 2014 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __CLEARPHASE_HP_PARAMS_H
#define __CLEARPHASE_HP_PARAMS_H

#define CLEARPHASE_HP_IOCTL_MAGIC 't'

#define CLEARPHASE_HP_PARAM \
	_IOW(CLEARPHASE_HP_IOCTL_MAGIC, 0, unsigned)

#define CLEARPHASE_HP_PARAM_SIZE 2064

struct clearphase_hp_param_data {
	unsigned char data[CLEARPHASE_HP_PARAM_SIZE];
};

#endif
