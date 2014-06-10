/* Copyright (C) 2013 Sony Mobile Communications AB.
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

#ifndef __SFORCE_PARAMS_H
#define __SFORCE_PARAMS_H

#define SFORCE_IOCTL_MAGIC 't'

#define SFORCE_PARAM \
	_IOW(SFORCE_IOCTL_MAGIC, 0, unsigned)

#define SFORCE_PARAM_SIZE 1016

enum {
	SFORCE_TYPE_MUSIC = 0,
	SFORCE_TYPE_VIDEO,
	SFORCE_TYPE_MAX
};

struct sforce_param_data {
	unsigned int sforce_type;
	unsigned char data[SFORCE_PARAM_SIZE];
};

#endif
