/*
 * drivers/net/wireless/somcwifictrl/somc_kitakami-wifi.c
 *
 * Copyright (C) 2013 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/msm_pcie.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <net/somc_kitakami-wifi.h>

/* These definitions need to be aligned with bcmdhd */
#define WLAN_STATIC_SCAN_BUF 5
#define WLAN_STATIC_DHD_INFO_BUF 7
#define WLAN_STATIC_DHD_IF_FLOW_LKUP 9
#define WLAN_STATIC_DHD_MEMDUMP_BUF 10
#define WLAN_STATIC_DHD_MEMDUMP_RAM 11
#define WLAN_STATIC_DHD_PKTID_MAP 12

#define ESCAN_BUF_SIZE (64 * 1024) /* for WIPHY_ESCAN0 */
#define PREALLOC_WLAN_SEC_NUM 4
#define PREALLOC_WLAN_BUF_NUM 160
#define PREALLOC_WLAN_SECTION_HEADER 24

#define WLAN_DHD_INFO_BUF_SIZE (24 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE (36 * 1024)
#define WLAN_DHD_MEMDUMP_SIZE (800 * 1024)

#define DHD_PKTIDMAP_FIFO_MAX 4
#define WLAN_MAX_PKTID_ITEMS (8192)
#define WLAN_DHD_PKTID_MAP_HDR_SIZE (20 + 4 * (WLAN_MAX_PKTID_ITEMS + 1))
#define WLAN_DHD_PKTID_MAP_ITEM_SIZE (48)
#define WLAN_DHD_PKTID_MAP_SIZE (WLAN_DHD_PKTID_MAP_HDR_SIZE + \
	(DHD_PKTIDMAP_FIFO_MAX * (WLAN_MAX_PKTID_ITEMS + 1) * \
	WLAN_DHD_PKTID_MAP_ITEM_SIZE))

#define WLAN_SECTION_SIZE_0 (PREALLOC_WLAN_BUF_NUM * 128)  /* for PROT */
#define WLAN_SECTION_SIZE_1 0                              /* for RXBUF */
#define WLAN_SECTION_SIZE_2 0                              /* for DATABUF */
#define WLAN_SECTION_SIZE_3 (PREALLOC_WLAN_BUF_NUM * 1024) /* for OSL_BUF */

/* These definitions are copied from bcmdhd */
#define DHD_SKB_1PAGE_BUFSIZE (PAGE_SIZE * 1)
#define DHD_SKB_2PAGE_BUFSIZE (PAGE_SIZE * 2)
#define DHD_SKB_4PAGE_BUFSIZE (PAGE_SIZE * 4)

#define DHD_SKB_1PAGE_BUF_NUM 0
#define DHD_SKB_2PAGE_BUF_NUM 64
#define DHD_SKB_4PAGE_BUF_NUM 0

#define WLAN_SKB_1_2PAGE_BUF_NUM ((DHD_SKB_1PAGE_BUF_NUM) + \
	(DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM ((WLAN_SKB_1_2PAGE_BUF_NUM) + \
	(DHD_SKB_4PAGE_BUF_NUM))

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *wlan_static_scan_buf;
static void *wlan_static_dhd_info_buf;
static void *wlan_static_if_flow_lkup;
static void *wlan_static_dhd_memdump_buf;
static void *wlan_static_dhd_memdump_ram;
static void *wlan_static_dhd_pktid_map;

struct bcmdhd_platform_data {
	struct platform_device *pdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	unsigned int wlan_reg_on;
	unsigned int pci_number;
};

static struct bcmdhd_platform_data *bcmdhd_data;

static int somc_wifi_init_mem(void)
{
	int i;
	for (i = 0; i < WLAN_SKB_BUF_NUM; i++)
		wlan_static_skb[i] = NULL;

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (i = 0; i < PREALLOC_WLAN_SEC_NUM; i++) {
		if (wlan_mem_array[i].size > 0) {
			wlan_mem_array[i].mem_ptr =
				kzalloc(wlan_mem_array[i].size, GFP_KERNEL);

			if (!wlan_mem_array[i].mem_ptr)
				goto err_mem_alloc;
		}
	}

	wlan_static_scan_buf = kzalloc(ESCAN_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf) {
		pr_err("%s: failed to allocate wlan_static_scan_buf\n",
			__func__);
		goto err_mem_alloc;
	}

	wlan_static_dhd_info_buf = kzalloc(WLAN_DHD_INFO_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_info_buf) {
		pr_err("%s: failed to allocate wlan_static_dhd_info_buf\n",
			__func__);
		goto err_mem_alloc;
	}

	wlan_static_if_flow_lkup = kzalloc(WLAN_DHD_IF_FLOW_LKUP_SIZE,
		GFP_KERNEL);
	if (!wlan_static_if_flow_lkup) {
		pr_err("%s: failed to allocate wlan_static_if_flow_lkup\n",
			__func__);
		goto err_mem_alloc;
	}
#ifdef CONFIG_BCMDHD_DEBUG_PAGEALLOC
	wlan_static_dhd_memdump_buf = kzalloc(WLAN_DHD_MEMDUMP_SIZE,
		GFP_KERNEL);
	if (!wlan_static_dhd_memdump_buf) {
		pr_err("%s: failed to alloc wlan_static_dhd_memdump_buf\n",
			__func__);
		goto err_mem_alloc;
	}

	wlan_static_dhd_memdump_ram = kzalloc(WLAN_DHD_MEMDUMP_SIZE,
		GFP_KERNEL);
	if (!wlan_static_dhd_memdump_ram) {
		pr_err("%s: failed to alloc wlan_static_dhd_memdump_ram\n",
			__func__);
		goto err_mem_alloc;
	}
#endif /* CONFIG_BCMDHD_DEBUG_PAGEALLOC */
	wlan_static_dhd_pktid_map = kzalloc(WLAN_DHD_PKTID_MAP_SIZE,
		GFP_KERNEL);
	if (!wlan_static_dhd_pktid_map) {
		pr_err("%s: failed to allocate wlan_static_dhd_pktid_map\n",
			__func__);
		goto err_mem_alloc;
	}

	pr_err("%s: Wi-Fi static memory allocated\n", __func__);
	return 0;

err_mem_alloc:
	kzfree(wlan_static_dhd_pktid_map);
#ifdef CONFIG_BCMDHD_DEBUG_PAGEALLOC
	kzfree(wlan_static_dhd_memdump_ram);
	kzfree(wlan_static_dhd_memdump_buf);
#endif /* CONFIG_BCMDHD_DEBUG_PAGEALLOC */
	kzfree(wlan_static_if_flow_lkup);
	kzfree(wlan_static_dhd_info_buf);
	kzfree(wlan_static_scan_buf);
	pr_err("%s: failed to allocate mem_alloc\n", __func__);
	for (i--; i >= 0; i--) {
		kzfree(wlan_mem_array[i].mem_ptr);
		wlan_mem_array[i].mem_ptr = NULL;
	}

	i = WLAN_SKB_BUF_NUM;
err_skb_alloc:
	pr_err("%s: failed to allocate skb_alloc\n", __func__);
	for (i--; i >= 0; i--) {
		dev_kfree_skb(wlan_static_skb[i]);
		wlan_static_skb[i] = NULL;
	}

	return -ENOMEM;
}

static void *somc_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF)
		return wlan_static_scan_buf;

	if (section == WLAN_STATIC_DHD_INFO_BUF) {
		if (size > WLAN_DHD_INFO_BUF_SIZE) {
			pr_err("%s: request DHD_INFO size(%lu) is bigger than"
				" static size(%d).\n", __func__, size,
				WLAN_DHD_INFO_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_info_buf;
	}

	if (section == WLAN_STATIC_DHD_IF_FLOW_LKUP)  {
		if (size > WLAN_DHD_IF_FLOW_LKUP_SIZE) {
			pr_err("%s: request DHD_IF_FLOW size(%lu) is bigger"
				" than static size(%d).\n", __func__, size,
				WLAN_DHD_IF_FLOW_LKUP_SIZE);
			return NULL;
		}
		return wlan_static_if_flow_lkup;
	}

	if (section == WLAN_STATIC_DHD_MEMDUMP_BUF) {
		if (size > WLAN_DHD_MEMDUMP_SIZE) {
			pr_err("%s: request DHD_MEMDUMP_BUF size(%lu) is bigger"
				" than static size(%d).\n", __func__, size,
				WLAN_DHD_MEMDUMP_SIZE);
			return NULL;
		}
		return wlan_static_dhd_memdump_buf;
	}

	if (section == WLAN_STATIC_DHD_MEMDUMP_RAM) {
		if (size > WLAN_DHD_MEMDUMP_SIZE) {
			pr_err("%s: request DHD_MEMDUMP_RAM size(%lu) is bigger"
				" than static size(%d).\n", __func__, size,
				WLAN_DHD_MEMDUMP_SIZE);
			return NULL;
		}
		return wlan_static_dhd_memdump_ram;
	}

	if (section == WLAN_STATIC_DHD_PKTID_MAP)  {
		if (size > WLAN_DHD_PKTID_MAP_SIZE) {
			pr_err("%s: request DHD_PKTID size(%lu) is"
				" bigger than static size(%d).\n", __func__,
				size, WLAN_DHD_PKTID_MAP_SIZE);
			return NULL;
		}
		return wlan_static_dhd_pktid_map;
	}

	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;
	if (size > wlan_mem_array[section].size)
		return NULL;
	return wlan_mem_array[section].mem_ptr;
}

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

int somc_wifi_set_power(int on)
{
	gpio_set_value(bcmdhd_data->wlan_reg_on, on);
	return 0;
}

int somc_wifi_set_carddetect(int present)
{
	int ret = 0;
	if (present)
		ret = msm_pcie_enumerate(bcmdhd_data->pci_number);
	return ret;
}

struct wifi_platform_data somc_wifi_control = {
	.set_power	= somc_wifi_set_power,
	.set_carddetect	= somc_wifi_set_carddetect,
	.mem_prealloc	= somc_wifi_mem_prealloc,
};
EXPORT_SYMBOL(somc_wifi_control);

static int __init somc_wifi_init_on_boot(void)
{
	if (somc_wifi_init_mem())
		return -ENOMEM;
	return 0;
}

device_initcall(somc_wifi_init_on_boot);

MODULE_LICENSE("GPL v2");
