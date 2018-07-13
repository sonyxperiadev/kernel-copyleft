/*
 * Author: Nandhakumar Rangasamy <nandhakumar.x.rangasamy@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/of_platform.h>
#include <asm/setup.h>
#include <soc/qcom/security_status.h>

#define SECURITY_ENABLED 0x2

static int security_status = -1;

static int __init security_config_setup(char *p)
{
	unsigned long res;

	if (!p || !*p)
		return -EINVAL;

	if (!kstrtoul(p, 0, &res)) {
		if (res & SECURITY_ENABLED)
			security_status = SECURITY_ON;
		else
			security_status = SECURITY_OFF;
	}

	pr_info("system booted with SECURITY_STATUS : %s\n",
		security_status ? "ON" : "OFF");
	return 0;
}
early_param("oemandroidboot.securityflags", security_config_setup);

int get_security_status(int *status)
{
	if (security_status == -1)
		return -EINVAL;
	else {
		*status = security_status;
		return 0;
	}
}
