/* include/linux/cxd224x.h
 *
 * Copyright (C) 2012 Sony Corporation.
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

#ifndef _CXD224X_H
#define _CXD224X_H

#define CXD224X_DEVICE_NAME "cxd224x-i2c"

struct cxd224x_platform_data {
	unsigned int irq_gpio;
	unsigned int wake_gpio;
};

#endif
