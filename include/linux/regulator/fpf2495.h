/*
 * fpf2495.h
 *
 * based on fixed.h
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

#ifndef __FPF2495_H
#define __FPF2495_H

struct regulator_init_data;

/**
 * struct fpf2495_config - fpf2495_config structure
 * @supply_name:		Name of the regulator supply
 * @input_supply:		Name of the input regulator supply
 * @flag_pull_supply:		Name of the pull flag regulator supply
 * @microvolts:			Output voltage of regulator
 * @startup_delay:		Start-up time in microseconds
 * @gpio_en.gpio:		GPIO to use for enable control
 *				set to -EINVAL if not used
 * @gpio_en.active_high:	Polarity of enable GPIO
 *				1 = Active high, 0 = Active low
 * @gpio_flag.gpio:		GPIO to use for enable control
 *				set to -EINVAL if not used
 * @gpio_flag.active_high:	Polarity of enable GPIO
 *				1 = Active high, 0 = Active low
 * @init_data:			regulator_init_data
 * @oc_delay:			dead time for inrush current
 *
 * This structure contains fpf2495 regulator configuration
 * information that must be passed by platform code to the fpf2495
 * regulator driver.
 */
struct fpf2495_config {
	const char *supply_name;
	const char *input_supply;
	const char *flag_pull_supply;
	int microvolts;
	struct {
		int gpio;
		unsigned int active_high:1;
	} gpio_en, gpio_flag;
	unsigned startup_delay;
	struct regulator_init_data *init_data;
	unsigned oc_delay;
};

#endif
