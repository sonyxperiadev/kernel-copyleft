/* arch/arm/mach-msm/board-sony_rhine-usb.c
 *
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <mach/board-usb.h>

bool msm_is_usb3_available(void)
{
	/* Rhine products can use usb3 basically. */
	pr_debug("usb3.0 is available.\n");
	return true;
}
