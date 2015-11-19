/* bcm_wlan_ramdump.h
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
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __BCM_WLAN_RAMDUMP_H
#define __BCM_WLAN_RAMDUMP_H

extern void bcm_wlan_crash_reason(char *msg);
extern void bcm_wlan_ramdump(void *addr, int size);

#define BCM_WLAN_CRASH_REASON_LEN 512

#endif
