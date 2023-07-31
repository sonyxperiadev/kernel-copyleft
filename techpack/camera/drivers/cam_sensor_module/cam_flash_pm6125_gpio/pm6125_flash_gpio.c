/*
 * Copyright (C) 2021 HUAQIN Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/rc-core.h>
#include <linux/platform_device.h>
#include "pm6125_flash_gpio.h"



/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct pwm_device *pwm;

/*****************************************************************************
 * Function
 *****************************************************************************/

void pm6125_flash_gpio_select_state(PM6125_FLASH_GPIO_STATE s){
    struct pwm_state pstate;
    int rc = 0;

    pwm_get_state(pwm, &pstate);
    pstate.period = 500000;
    pstate.duty_cycle = 500000;
    switch(s){
    case PM6125_FLASH_GPIO_STATE_ACTIVE:
        pstate.enabled = true;
        rc = pwm_apply_state(pwm, &pstate);
        break;
    case PM6125_FLASH_GPIO_STATE_SUSPEND:
        pstate.enabled = false;
        rc = pwm_apply_state(pwm, &pstate);
        break;
    default:
        PM6125_FLASH_PRINT("[pm6125_flash_gpio]Failed to control PWM use a err state!\n");
    }

    if(rc < 0) {
        PM6125_FLASH_PRINT("[pm6125_flash_gpio]Apply PWM state fail, rc = %d", rc);
    }
}
EXPORT_SYMBOL(pm6125_flash_gpio_select_state);

static int pm6125_flash_pwm_probe(struct platform_device *pdev)
{
	int rc = 0;

	pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pwm))
		return PTR_ERR(pwm);

	return rc;
}

static const struct of_device_id gpio_of_match[] = {
    { .compatible = "qualcomm,pm6125_flash_gpio", },
    {},
};

static struct platform_driver pm6125_flash_gpio_platform_driver = {
    .probe = pm6125_flash_pwm_probe,
    .driver = {
        .name = "PM6125_FLASH_GPIO_DTS",
        .of_match_table = gpio_of_match,
    },
};

int pm6125_flash_gpio_init_module(void)
{
    if (platform_driver_register(&pm6125_flash_gpio_platform_driver)) {
	    PM6125_FLASH_PRINT("[pm6125_flash_gpio]Failed to register pm6125_flash_gpio_platform_driver!\n");
	    return -1;
    }
    return 0;
}

void pm6125_flash_gpio_exit_module(void)
{
    platform_driver_unregister(&pm6125_flash_gpio_platform_driver);
}

MODULE_AUTHOR("chejian <chejian@huaqin.com>");
MODULE_DESCRIPTION("CONTROL PM6125 FLASH GPIO Driver");
MODULE_LICENSE("GPL");
