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

enum {
	MEM_DESC_PLATFORM = 0,
	MEM_DESC_CORE
};

struct mem_desc {
	u64 phys_addr;
	u64 size;
	u8  name[MEM_DESC_NAME_SIZE];
	u32 flags;
	u32 reserved;
} __attribute__ ((__packed__));

void ramdump_add_mem_desc(struct mem_desc *);
void ramdump_remove_mem_desc(struct mem_desc *);

#endif
