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

#ifndef __SONY_SUBSYS_RAMDUMP_H
#define __SONY_SUBSYS_RAMDUMP_H

extern int register_sony_subsys(const char *name);
extern void unregister_sony_subsys(const char *name);
extern void sony_subsys_notify_crash(const char *name, char *msg);

#define SUBSYS_NAME_LEN 32
#define SUBSYS_CRASH_REASON_LEN 512

#endif
