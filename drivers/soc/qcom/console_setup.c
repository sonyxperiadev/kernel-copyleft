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
#include <linux/of_platform.h>
#include <asm/setup.h>

#define CONSOLE_NAME "ttyHSL"
#define CONSOLE_IX 0
#define CONSOLE_OPTIONS "115200,n8"

static int need_serial_console;

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

static void disable_serial_gpio(void)
{
	struct device_node *np_tx, *np_rx, *np_tx_def, *np_rx_def;
	struct property *pp;
	static u32 pin_func_val;
	static struct property pin_func = {
		.name = "qcom,pin-func",
		.value = &pin_func_val,
		.length = sizeof(pin_func_val),
	};
	static struct property output_low = {
		.name = "output-low",
		.value = NULL,
		.length = 0,
	};
	static struct property bias_disable = {
		.name = "bias-disable",
		.value = NULL,
		.length = 0,
	};

	np_tx = of_find_node_by_path(
			"/soc/pinctrl@fd510000/msm_gpio_4");
	if (!np_tx) {
		pr_err("couldn't find msm_gpio_4 node\n");
		return;
	}

	np_rx = of_find_node_by_path(
			"/soc/pinctrl@fd510000/msm_gpio_5");
	if (!np_rx) {
		pr_err("couldn't find msm_gpio_5 node\n");
		goto err0;
	}

	of_update_property(np_tx, &pin_func);
	of_update_property(np_rx, &pin_func);

	np_tx_def = of_find_node_by_name(np_tx, "default");
	if (!np_tx_def) {
		pr_err("couldn't find msm_gpio_4_def node\n");
		goto err1;
	}

	np_rx_def = of_find_node_by_name(np_rx, "default");
	if (!np_rx_def) {
		pr_err("couldn't find msm_gpio_5_def node\n");
		goto err2;
	}

	of_add_property(np_tx_def, &output_low);

	pp = of_find_property(np_rx_def, "bias-pull-up", NULL);
	if (pp) {
		of_remove_property(np_rx_def, pp);
		of_add_property(np_rx_def, &bias_disable);
	}
	of_add_property(np_rx_def, &output_low);

	of_node_put(np_rx_def);
err2:
	of_node_put(np_tx_def);
err1:
	of_node_put(np_rx);
err0:
	of_node_put(np_tx);
	return;
}

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

		np = of_find_node_by_path("/soc/serial@f991e000");
		if (!np) {
			pr_err("couldn't find /soc/serial@f991e000 node\n");
			return -EINVAL;
		}

		pr_info("disabling %s node", np->full_name);
		of_update_property(np, &serial_con_status);
		of_node_put(np);
		disable_serial_gpio();
	}

	return 0;
}
early_initcall(init_console_setup);
