/*
 * as3677.h - platform data structure for as3677 led controller
 *
 * Copyright (C) 2010 Ulrich Herrmann <ulrich.herrmann@austriamicrosystems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */

#ifndef __LINUX_AS3677_H
#define __LINUX_AS3677_H

enum as3677_step_up_vmax {
	VMAX_16 = 0,
	VMAX_10 = 1,
	VMAX_25 = 2,
};

struct as3677_platform_led {
	char *name; /* if NULL the default name is used */
	u8    on_charge_pump; /* Is this led connected to the charge pump or
				 the DCDC converter */
	u32 max_current_uA; /* This leds maximum current in uA */
	u32 startup_current_uA; /* On driver load this brightness will be set,
				   useful for early backlight, etc. */
};

struct as3677_platform_data {
	struct as3677_platform_led leds[6]; /* order: curr1, curr2, curr6,
					       rgb1, rgb2, rgb3 */
	enum as3677_step_up_vmax step_up_vmax;
	bool dls_analog; /* Should the  analog dynamic luminance scaling be
			    used (less noise)? */
};

#endif /* __LINUX_as3677_H */

