/*
 * leds-as3665.c - AS3665 LED driver
 *
 * Copyright (C) 2012 ams AG
 *
 * Author: Byron Shi <byron.shi@ams.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef __LINUX_AS3665_H
#define __LINUX_AS3665_H

/* See Documentation/leds/leds-as3665.txt */

struct as3665_led_config {
	const char *name;
	u8		chan_nr;
	u8		led_current; /* mA x10, 0 if led is not connected */
	u8		max_current;
	u8		startup_current;
};

struct as3665_platform_data {
	struct as3665_led_config *led_config;
	u8	num_channels;
	u8	clock_mode;
	int	(*setup_resources)(struct device *dev);
	void	(*release_resources)(struct device *dev);
	int	(*enable)(struct device *dev, bool state);
};

#endif /* __LINUX_AS3665_H */
