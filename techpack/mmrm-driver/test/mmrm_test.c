/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "mmrm_test: " fmt

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <soc/qcom/socinfo.h>

#include "mmrm_test_internal.h"

#define MODULE_NAME "mmrm_test"

enum PLATFORM_SOC_ID {
	SOC_ID_LAHAINA = 415,	/* LAHAINA */
	SOC_ID_WAIPIO = 457,	/* WAIPIO */
	SOC_ID_KAILUA = 519,	/* KAILUA */
};


struct mmrm_test_platform_resources {
	struct platform_device *pdev;
	struct clock_rate *clk_rate_tbl;
	u32 count;
};

struct mmrm_test_driver_data {
	struct mmrm_test_platform_resources clk_res;
};

static struct mmrm_test_driver_data *test_drv_data = (void *) -EPROBE_DEFER;
int mmrm_load_mmrm_test_table(
	struct mmrm_test_platform_resources *dt_res)
{
	int rc = 0, num_clock_names = 0, c = 0;
	struct platform_device *pdev = dt_res->pdev;
	int   entry_offset = 0;
	struct clock_rate *clk_rate;

	num_clock_names = of_property_count_strings(pdev->dev.of_node,
			"clock-names");
	if (num_clock_names <= 0) {
		dt_res->count = 0;
		goto err_load_corner_tbl;
	}
	pr_info("%s: count =%d\n", __func__, num_clock_names);

	dt_res->clk_rate_tbl = devm_kzalloc(&pdev->dev,
		sizeof(*dt_res->clk_rate_tbl) * num_clock_names, GFP_KERNEL);

	if (!dt_res->clk_rate_tbl) {
		rc = -ENOMEM;
		goto err_load_corner_tbl;
	}
	dt_res->count = num_clock_names;

	clk_rate = dt_res->clk_rate_tbl;
	for (c = 0; c < num_clock_names; c++, clk_rate++) {
		of_property_read_string_index(pdev->dev.of_node,
			"clock-names", c, &clk_rate->name);
	}

	clk_rate = dt_res->clk_rate_tbl;
	for (c = 0; c < num_clock_names; c++, entry_offset += 7, clk_rate++) {
		of_property_read_u32_index(pdev->dev.of_node,
			"clock_rates", entry_offset, &clk_rate->domain);

		of_property_read_u32_index(pdev->dev.of_node,
			"clock_rates", entry_offset+1, &clk_rate->id);

		of_property_read_u32_index(pdev->dev.of_node,
			"clock_rates", entry_offset+2, &clk_rate->clk_rates[0]);

		of_property_read_u32_index(pdev->dev.of_node,
			"clock_rates", entry_offset+3, &clk_rate->clk_rates[1]);

		of_property_read_u32_index(pdev->dev.of_node,
			"clock_rates", entry_offset+4, &clk_rate->clk_rates[2]);

		of_property_read_u32_index(pdev->dev.of_node,
			"clock_rates", entry_offset+5, &clk_rate->clk_rates[3]);

		of_property_read_u32_index(pdev->dev.of_node,
			"clock_rates", entry_offset+6, &clk_rate->clk_rates[4]);
	}

	/* print clock rate tables */

	clk_rate = dt_res->clk_rate_tbl;
	for (c = 0; c < num_clock_names; c++, clk_rate++) {
		pr_info("clock name:%s, %d, %d, %d, %d, %d, %d, %d\n", clk_rate->name,
			clk_rate->domain, clk_rate->id,
			clk_rate->clk_rates[0], clk_rate->clk_rates[1],
			clk_rate->clk_rates[2], clk_rate->clk_rates[3],
			clk_rate->clk_rates[4]);
	}

	return 0;

err_load_corner_tbl:
	return rc;
}

struct clock_rate *find_clk_by_name(const char *name)
{
	int  i;
	struct mmrm_test_platform_resources  *res = &test_drv_data->clk_res;
	struct clock_rate *p = res->clk_rate_tbl;

	for (i=0; i < res->count; i++, p++)
	{
		if (strcmp(name, p->name) == 0)
			return p;
	}
	return NULL;
}

struct clock_rate *get_nth_clock(int nth)
{
	struct mmrm_test_platform_resources  *res = &test_drv_data->clk_res;

	if (nth >= res->count)
		return NULL;

	return &(res->clk_rate_tbl[nth]);
}

int mmrm_test_read_platform_resources(struct platform_device *pdev)
{
	int rc = 0;

	pr_info("%s: mmrm_test_read_platform_resources =%p test_drv_data:%p\n",
		__func__, pdev, test_drv_data);

	if (pdev == NULL || pdev->dev.of_node == NULL) {
		rc = -EINVAL;
		goto exit;
	}

	if (test_drv_data == (void *) -EPROBE_DEFER) {
		pr_info("%s: mmrm_test_read_platform_resources\n", __func__);
		goto exit;
	}

	rc = mmrm_load_mmrm_test_table(&test_drv_data->clk_res);
	if (rc) {
		goto exit;
	}
exit:
	return rc;
}

static int mmrm_test_probe(struct platform_device *pdev)
{
	bool is_mmrm_supported = false;
	int soc_id;
	int rc;

	// Check if of_node is found
	if (!of_device_is_compatible(pdev->dev.of_node, "qcom,msm-mmrm-test")) {
		dev_info(&pdev->dev, "No compatible device node\n");
		return 1;
	}

	is_mmrm_supported = mmrm_client_check_scaling_supported(MMRM_CLIENT_CLOCK, 0);
	if (!is_mmrm_supported) {
		pr_info("%s: MMRM not supported on %s\n", __func__, socinfo_get_id_string());
		return 0;
	}

	test_drv_data = kzalloc(sizeof(*test_drv_data), GFP_KERNEL);
	if (!test_drv_data) {
		rc = -ENOMEM;
		goto err_no_mem;
	}
	test_drv_data->clk_res.pdev = pdev;
	dev_set_drvdata(&pdev->dev, test_drv_data);

	rc = mmrm_test_read_platform_resources(pdev);
	if (rc) {
		pr_info("%s: unable to read platform resources for mmrm\n",
			__func__);
		goto err_read;
	}
	dev_info(&pdev->dev, "%s: Validating mmrm on target: %s\n", __func__, socinfo_get_id_string());

	// Get socid to get known mmrm configurations
	soc_id = socinfo_get_id();
	switch (soc_id) {
	case SOC_ID_LAHAINA: /* LAHAINA */
		test_mmrm_client(pdev, MMRM_TEST_LAHAINA, MMRM_TEST_LAHAINA_NUM_CLK_CLIENTS);
//		test_mmrm_concurrent_client_cases(pdev, all_lahaina_testcases);
		break;
	case SOC_ID_WAIPIO: /* WAIPIO */
		test_mmrm_client(pdev, MMRM_TEST_WAIPIO, MMRM_TEST_WAIPIO_NUM_CLK_CLIENTS);
		test_mmrm_concurrent_client_cases(pdev, waipio_testcases, waipio_testcases_count);
		test_mmrm_switch_volt_corner_client_testcases(pdev, waipio_cornercase_testcases, waipio_cornercase_testcases_count);
		break;
	case SOC_ID_KAILUA: /* KAILUA */
		test_mmrm_client(pdev, MMRM_TEST_KAILUA, MMRM_TEST_KAILUA_NUM_CLK_CLIENTS);
		test_mmrm_concurrent_client_cases(pdev, waipio_testcases, waipio_testcases_count);
		test_mmrm_switch_volt_corner_client_testcases(pdev, waipio_cornercase_testcases, waipio_cornercase_testcases_count);
		break;
	default:
		pr_info("%s: Not supported for soc_id %d [Target %s]\n",
			__func__,
			soc_id,
			socinfo_get_id_string());
		return -ENODEV;
	}

err_no_mem:
err_read:
	return 0;
}

int mmrm_test_remove(struct platform_device *pdev) {
	int rc = 0;

	if (!pdev) {
		rc = -EINVAL;
		goto err_exit;
	}

	test_drv_data = dev_get_drvdata(&pdev->dev);
	if (!test_drv_data) {
		rc = -EINVAL;
		goto err_exit;
	}

	if (test_drv_data->clk_res.clk_rate_tbl)
		kfree(test_drv_data->clk_res.clk_rate_tbl);

	dev_set_drvdata(&pdev->dev, NULL);

	kfree(test_drv_data);
	test_drv_data = (void *) -EPROBE_DEFER;

err_exit:
	return rc;
}

static const struct of_device_id mmrm_test_dt_match[] = {
	{.compatible = "qcom,msm-mmrm-test"}, {} // empty
};

static struct platform_driver mmrm_test_driver = {
	.probe = mmrm_test_probe,
	.remove = mmrm_test_remove,
	.driver = {
			.name = MODULE_NAME,
			.owner = THIS_MODULE,
			.of_match_table = mmrm_test_dt_match,
		},
};

static int __init mmrm_test_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&mmrm_test_driver);
	if (rc) {
		pr_info("%s: failed to register platform driver\n", __func__);
	}
	return rc;
}
module_init(mmrm_test_init);

static void __exit mmrm_test_exit(void)
{
	platform_driver_unregister(&mmrm_test_driver);
}
module_exit(mmrm_test_exit);

MODULE_DESCRIPTION("MMRM TEST");
MODULE_LICENSE("GPL v2");
