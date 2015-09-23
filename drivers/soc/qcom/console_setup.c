/* drivers/soc/qcom/console_setup.c
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

#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/string.h>
#include <asm/setup.h>

#define CONSOLE_NAME "ttyHSL"
#define CONSOLE_IX 0
#define CONSOLE_OPTIONS "115200,n8"

static int need_serial_console;

static void dummy_write(struct console *con, const char *s, unsigned n)
{
}

static struct console dummy_console_dev = {
	.name = "dummycon",
	.write = dummy_write,
	.flags = CON_PRINTBUFFER,
	.index = -1,
};

static int __init setup_serial_console(char *console_flag)
{
	if (console_flag &&
		strnlen(console_flag, COMMAND_LINE_SIZE) >= 2 &&
		(console_flag[0] != '0' || console_flag[1] != '0')) {
		need_serial_console = 1;
		return 0;
	}

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
		pr_info("Registering dummy console\n");
		register_console(&dummy_console_dev);
		add_preferred_console(dummy_console_dev.name, 0, NULL);
	}

	return 0;
}
early_initcall(init_console_setup);
