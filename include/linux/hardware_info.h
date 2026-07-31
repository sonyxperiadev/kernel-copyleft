/*
 * Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
/*
 * Copyright 2025 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __HARDWARE_INFO_H__
#define __HARDWARE_INFO_H__

#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

typedef struct {
	const char *version;
} HARDWARE_INFO;

#endif
