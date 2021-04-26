/* drivers/soc/qcom/console_setup.c
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
#include <linux/console.h>
#include <linux/string.h>
#include <linux/of_platform.h>
#include <asm/setup.h>

#define CONSOLE_NAME "ttyMSM"
#define CONSOLE_IX 0
#define CONSOLE_OPTIONS "115200,n8"
#define CONSOLE_ENABLE 0x01
#define SERIAL_NODE_STR "/soc/qcom,qup_uart@a90000"

static int need_serial_console;

static int __init setup_serial_console(char *console_flag)
{
	unsigned long val;

	if (kstrtoul(console_flag, 16, &val))
		return -EINVAL;

	if (val & CONSOLE_ENABLE)
		need_serial_console = 1;

	return 0;
}

/*
* The S1 Boot configuration TA unit can specify that the serial console
* enable flag will be passed as Kernel boot arg with tag babe09A9.
*/
early_param("oemandroidboot.babe09a9", setup_serial_console);

static int __init init_console_setup(void)
{
	if (need_serial_console) {
		pr_info("Adding %s%d as preferred console\n",
			CONSOLE_NAME, CONSOLE_IX);
		add_preferred_console(CONSOLE_NAME,
			CONSOLE_IX,
			CONSOLE_OPTIONS);
	} else {
		struct device_node *np;
		static struct property serial_con_status = {
			.name = "status",
			.value = "disabled",
			.length = sizeof("disabled"),
		};

		np = of_find_node_by_path(SERIAL_NODE_STR);
		if (!np) {
			pr_err("couldn't find %s node\n", SERIAL_NODE_STR);
			return -EINVAL;
		}

		pr_info("disabling %s node", np->full_name);
		of_update_property(np, &serial_con_status);
		of_node_put(np);
	}

	return 0;
}
early_initcall(init_console_setup);

