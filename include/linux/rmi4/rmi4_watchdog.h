/*
 * RMI4 bus driver.
 * include/linux/rmi4/rmi4_watchdog.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __RMI4_WATCHDOG__
#define __RMI4_WATCHDOG__

#define RMI4_WATCHDOG_NAME "rmi4_watchdog"

struct rmi4_wd_platform_data {
	int (*reset_func)(struct device *dev);
	int poll_t_ms;
};

#endif
