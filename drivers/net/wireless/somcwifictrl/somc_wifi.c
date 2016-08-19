/*
 * drivers/net/wireless/somcwifictrl/somc_wifi.c
 *
 */
/*
 * Copyright (C) 2015 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/gpio.h>
#include <linux/wlan_plat.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/msm_pcie.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <net/somc_wifi.h>

#include "dhd_custom_memprealloc.h"

struct bcmdhd_platform_data {
	struct platform_device *pdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	unsigned int wlan_reg_on;
	unsigned int pci_number;
};

static struct bcmdhd_platform_data *bcmdhd_data;

static struct mmc_host *wlan_mmc_host;

int somc_wifi_init(struct platform_device *pdev)
{
	int ret, ret_sus, gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *lookup_state;

	bcmdhd_data = kzalloc(sizeof(*bcmdhd_data), GFP_KERNEL);
	if (!bcmdhd_data) {
		dev_err(&pdev->dev, "%s: no memory\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_bcmdhd_data;
	}

	bcmdhd_data->pdev = pdev;
	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "%s: pinctrl not defined\n", __func__);
		ret = PTR_ERR(pinctrl);
		goto err_pinctrl;
	}
	bcmdhd_data->pinctrl = pinctrl;

	lookup_state = pinctrl_lookup_state(pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(lookup_state)) {
		dev_err(&pdev->dev, "%s: pinctrl lookup failed for default\n",
			__func__);
		ret = PTR_ERR(lookup_state);
		goto err_pinctrl;
	}
	bcmdhd_data->gpio_state_active = lookup_state;

	lookup_state = pinctrl_lookup_state(pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(lookup_state)) {
		dev_err(&pdev->dev, "%s: pinctrl lookup failed for sleep\n",
			__func__);
		ret = PTR_ERR(lookup_state);
		goto err_pinctrl;
	}
	bcmdhd_data->gpio_state_suspend = lookup_state;

	ret = pinctrl_select_state(bcmdhd_data->pinctrl,
		bcmdhd_data->gpio_state_active);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to select active state\n",
			__func__);
		goto err_pinctrl;
	}

	gpio = of_get_gpio(pdev->dev.of_node, 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "%s: invalid gpio #%s: %d\n",
			__func__, "wlan-reg-on", gpio);
		ret = -ENXIO;
		goto err_gpio;
	}
	bcmdhd_data->wlan_reg_on = gpio;

	ret = gpio_request(bcmdhd_data->wlan_reg_on,
			"wlan-reg-on");
	if (ret) {
		dev_err(&pdev->dev, "%s: request err %s: %d\n",
			__func__, "wlan-reg-on", ret);
		goto err_gpio;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "wlan-pci-number",
		&bcmdhd_data->pci_number);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: failed to find PCI number %d %d \n",
			__func__, ret, bcmdhd_data->pci_number);
		goto err_gpio_request;
	}

	return 0;

err_gpio_request:
	gpio_free(bcmdhd_data->wlan_reg_on);
err_gpio:
	ret_sus = pinctrl_select_state(bcmdhd_data->pinctrl,
			bcmdhd_data->gpio_state_suspend);
	if (ret_sus)
		dev_err(&pdev->dev, "%s: failed to select suspend state\n",
			__func__);
err_pinctrl:
	kzfree(bcmdhd_data);
err_alloc_bcmdhd_data:
	return ret;
}
EXPORT_SYMBOL(somc_wifi_init);

void somc_wifi_deinit(struct platform_device *pdev)
{
	if (bcmdhd_data) {
		if (gpio_is_valid(bcmdhd_data->wlan_reg_on))
			gpio_free(bcmdhd_data->wlan_reg_on);
		if (!IS_ERR_OR_NULL(bcmdhd_data->pinctrl) &&
			!IS_ERR_OR_NULL(bcmdhd_data->gpio_state_suspend)) {
			int ret = pinctrl_select_state(bcmdhd_data->pinctrl,
				bcmdhd_data->gpio_state_suspend);
			if (ret)
				dev_err(&pdev->dev, "%s: failed to select"
					" suspend state\n", __func__);
		}
		kzfree(bcmdhd_data);
	}
}
EXPORT_SYMBOL(somc_wifi_deinit);

void somc_wifi_mmc_host_register(struct mmc_host *host)
{
	wlan_mmc_host = host;
}

int somc_wifi_set_power(int on)
{
	gpio_set_value(bcmdhd_data->wlan_reg_on, on);
	return 0;
}

#ifdef CONFIG_BCMDHD_PCIE
int somc_wifi_set_carddetect(int present)
{
	int ret = 0;
	if (present)
		ret = msm_pcie_enumerate_locked(bcmdhd_data->pci_number);
	return ret;
}
#else /* CONFIG_BCMDHD_PCIE */
int somc_wifi_set_carddetect(int present)
{
	if (wlan_mmc_host)
		mmc_detect_change(wlan_mmc_host, 0);
	return 0;
}
#endif /* CONFIG_BCMDHD_PCIE */

struct wifi_platform_data somc_wifi_control = {
	.set_power	= somc_wifi_set_power,
	.set_carddetect	= somc_wifi_set_carddetect,
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif
};
EXPORT_SYMBOL(somc_wifi_control);

MODULE_LICENSE("GPL v2");
