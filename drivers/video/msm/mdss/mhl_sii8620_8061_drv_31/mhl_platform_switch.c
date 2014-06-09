/* [kernel/drivers/video/msm/mdss/mhl_sii8620_8061_drv/mhl_platform_switch.c]
 *
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * Author: [Hirokuni Kawasaki <hirokuni.kawaaki@sonymobile.com>]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "mhl_common.h"
#include "mhl_platform.h"
static struct usb_ext_notification *mhl_info;
static bool isDiscoveryCalled;
static void (*notify_usb_online)(void *ctx, int on);
static void *context_cb;
static int (*device_discovery_cb)(void *context_cb);
static void *usb_ctx;

static void mhl_pf_switch_gpio_to_usb(void)
{
	/* todo : must use dtsi for gpio */
	gpio_set_value(GPIO_MHL_SWITCH_SEL_1, 0);
	gpio_set_value(GPIO_MHL_SWITCH_SEL_2, 0);
	pr_debug("%s: gpio(%d) : %d", __func__,
			 GPIO_MHL_SWITCH_SEL_1,
			 gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	pr_debug("%s: gpio(%d) : %d", __func__,
			 GPIO_MHL_SWITCH_SEL_2,
			 gpio_get_value(GPIO_MHL_SWITCH_SEL_2));
}

static void mhl_pf_switch_gpio_to_mhl(void)
{
	/* todo : must use dtsi for gpio */
	gpio_set_value(GPIO_MHL_SWITCH_SEL_1, 1);
	gpio_set_value(GPIO_MHL_SWITCH_SEL_2, 1);
	pr_debug("%s: gpio(%d) : %d", __func__,
			 GPIO_MHL_SWITCH_SEL_1,
			 gpio_get_value(GPIO_MHL_SWITCH_SEL_1));
	pr_debug("%s: gpio(%d) : %d", __func__,
			 GPIO_MHL_SWITCH_SEL_2,
			 gpio_get_value(GPIO_MHL_SWITCH_SEL_2));
}

/**
 * mhl_pf_switch_to_usb: switch to usb.
 * This API must be called after the INT by PMIC.
 */
int mhl_pf_switch_to_usb(void)
{
	pr_info("%s:", __func__);
	isDiscoveryCalled = false;
	mhl_pf_switch_gpio_to_usb();
	msleep(20);

	if (!notify_usb_online) {
		pr_warn("%s: no notify_usb_online registration\n", __func__);
		/*
		 * This API may be called after mhl_pf_switch_device_discovery
		 * is called by USB, otherwise the "online" notification can't be sent
		 * to USB via the "notify_usb_online".
		 * Thought the notification doesn't occur, the GPIO will be somehow
		 * switched to USB.
		 * (USB is default. So, the system will try to switch to USB.)
		 */
		/*return MHL_FAIL;*/
	} else {
		notify_usb_online(usb_ctx, 0);
	}

	return MHL_SUCCESS;
}
EXPORT_SYMBOL(mhl_pf_switch_to_usb);

/**
 * mhl_pf_switch_to_mhl: switch to mhl
 */
int mhl_pf_switch_to_mhl(void)
{
	pr_info("%s:", __func__);
	if (!notify_usb_online) {
		pr_warn("%s: no notify_usb_online registration\n", __func__);
		return MHL_FAIL;
	} else {
		notify_usb_online(usb_ctx, 1);
	}
	mhl_pf_switch_gpio_to_mhl();
	return MHL_SUCCESS;
}
EXPORT_SYMBOL(mhl_pf_switch_to_mhl);

/**
 * mhl_pf_switch_register_cb: register
 * a call back for notifying device discovery.
 * The client should start device discovery
 * when call the callback and must notify the
 * result. Only one registration is acceptable.
 * When the register API is called before unregister,
 * the previous call back and context will be
 * replace with new registration.
 *
 * @device_discovery - call back API. Must not be NULL.
 * @context - it can be NULL. It is notified through the call back API.
 *
 */
void mhl_pf_switch_register_cb(int (*device_discovery)(void *context),
								void *context)
{
	pr_info("%s:\n", __func__);
	device_discovery_cb = device_discovery;
	context_cb = context;
	if (isDiscoveryCalled)
		device_discovery_cb(context_cb);
}
EXPORT_SYMBOL(mhl_pf_switch_register_cb);

/**
 * mhl_pf_switch_unregister_cb: unregister
 * the registerred call back and stored context is
 * unregisterred either.
 */
void mhl_pf_switch_unregister_cb(void)
{
	device_discovery_cb = NULL;
	context_cb = NULL;
}
EXPORT_SYMBOL(mhl_pf_switch_unregister_cb);

static void mhl_pf_swtich_resource_free(void)
{
	kfree(mhl_info);
}

static int mhl_pf_switch_device_discovery(void *data,
					int id,
					void (*usb_notify_cb)(void *, int), void *ctx)
{
	int rc = MHL_USB_NON_INUSE;
	pr_info("%s()\n", __func__);

	if (id) {
		/* todo : this logic can be reused in 8620? */
		pr_debug("%s: USB ID pin high. id=%d\n", __func__, id);
		return id;
	}

	if (!usb_notify_cb) {
		pr_warn("%s: cb || ctrl is NULL\n", __func__);
		return -EINVAL;
	}

	isDiscoveryCalled = true;
	usb_ctx = ctx;
	mhl_pf_switch_gpio_to_mhl();
	if (!notify_usb_online)
		notify_usb_online = usb_notify_cb;
	if (device_discovery_cb) {
		rc = device_discovery_cb(context_cb);
	} else {
		/*
		 * Even if there is no registerred call back,
		 * MHL_USB_INUSE must be returned since the
		 * MHL ko object is supposed to be installed
		 * at boot timing and immediately registerring
		 * its call back. So this API must return INUSE
		 * to usb.
		 */
		pr_warn("%s: no registerred cb.\n", __func__);
		rc = MHL_USB_INUSE;
	}

	pr_info("%s: mhl is inuse ? : %d\n", __func__, (rc == MHL_USB_INUSE));

	if (rc != MHL_USB_INUSE)
			mhl_pf_switch_gpio_to_usb();

	return rc;
}

static int __init mhl_pf_switch_init(void)
{
	int rc = 0;

	pr_info("%s:\n", __func__);
	mhl_info = NULL;
	isDiscoveryCalled = false;
	notify_usb_online = NULL;

	device_discovery_cb = NULL;
	context_cb = NULL;

	mhl_info = kzalloc(sizeof(*mhl_info), GFP_KERNEL);
	if (!mhl_info) {
		pr_err("%s: FAILED: cannot alloc platform_switch\n", __func__);
		rc = -ENOMEM;
		goto failed_probe;
	}

	mhl_info->notify = mhl_pf_switch_device_discovery;

	if (msm_register_usb_ext_notification(mhl_info) != 0) {
		pr_err("%s: register for usb notifcn failed\n", __func__);
		rc = -EPROBE_DEFER;
		goto failed_probe;
	}

	return 0;

failed_probe:
	mhl_pf_swtich_resource_free();

	return rc;
}

static void __exit mhl_pf_switch_exit(void)
{
	pr_info("%s:\n", __func__);
	mhl_pf_swtich_resource_free();
}

module_init(mhl_pf_switch_init);
module_exit(mhl_pf_switch_exit);
MODULE_LICENSE("GPL");

