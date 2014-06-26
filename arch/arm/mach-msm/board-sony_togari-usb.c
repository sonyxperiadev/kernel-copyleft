/* arch/arm/mach-msm/board-sony_togari-usb.c
 *
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/board-usb.h>

/*
 * Check whether usb3 is available by checking hw_id.
 */
bool msm_is_usb3_available(void)
{
	struct device_node *node;
	int gpio_count;
	int hw_id;
	int i;

	node = of_find_node_by_name(of_find_node_by_path("/"), "gpio_hw_ids");
	if (!node) {
		pr_err("hw_id: node \"gpio_hw_ids\" does not exist.");
		return false;
	}

	gpio_count = of_gpio_count(node);
	if (!gpio_count) {
		pr_err("hw_id: GPIO is not assigned.");
		return false;
	}

	hw_id = 0;
	for (i = 0; i < gpio_count; i++) {
		int gpio;
		int ret;

		gpio = of_get_gpio(node, i);
		if (!gpio_is_valid(gpio)) {
			pr_err("hw_id: Failed to get gpio hw_id_%d.", i);
			return false;
		}

		ret = gpio_request(gpio, NULL);
		if (ret) {
			pr_err("hw_id: Failed to request gpio hw_id_%d.", i);
			return false;
		}

		ret = gpio_get_value(gpio);
		gpio_free(gpio);
		hw_id |= ret << i;
	}

	pr_info("usb3.0 is %savailable. hw_id=0x%02x",
						hw_id ? "" : "NOT ", hw_id);

	/* If hw_id is 0, usb3 is not available. */
	return hw_id ? true : false;
}
