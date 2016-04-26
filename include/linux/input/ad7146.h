/**
 * \file ad7146.h
 * This file is the header file for AD7146 captouch, contains the platform
 data structure and can be found in the include/linux/input/ad7146.h.
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL version 2 or later.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_INPUT_AD714X_H__
#define __LINUX_INPUT_AD714X_H__
#include <linux/sched.h>

/**
Register count of AD7146
*/
#define REGCNT		26

/**
This is the platform data for the AD7146 chip used in registration.
This structure is also sent to the I2C client and will be used in the
AD7146 probe routine, Hardware initialization...etc.,
*/
struct ad7146_platform_data {
	unsigned int regs[REGCNT];
	unsigned long irqflags;
};

#endif
