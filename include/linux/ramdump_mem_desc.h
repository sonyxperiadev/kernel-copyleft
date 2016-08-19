/*
 * Author: Nandhakumar Rangasamy<nandhakumar.x.rangasamy@sonymobile.com>
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

#ifndef __RAMDUMP_MEM_DESC_H_
#define __RAMDUMP_MEM_DESC_H_

#define MEM_DESC_NAME_SIZE 32

void ramdump_add_mem_desc(u64 addr, u64 size, char *);

#endif
