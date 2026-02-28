/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
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

#ifndef TEST_MMRM_TEST_INTERNAL_H_
#define TEST_MMRM_TEST_INTERNAL_H_

#include <linux/platform_device.h>
#include <linux/soc/qcom/msm_mmrm.h>

#define MMRM_TEST_LAHAINA_NUM_CLK_CLIENTS 23
#define MMRM_TEST_WAIPIO_NUM_CLK_CLIENTS 28
#define MMRM_TEST_NEO_NUM_CLK_CLIENTS 23

struct mmrm_test_desc {
	struct mmrm_test_clk_client  *clk_client;
	u32 clk_rate_id;
};

#define MMRM_SYSFS_ENTRY_MAX_LEN     PAGE_SIZE

enum mmrm_vdd_level {
	MMRM_TEST_VDD_LEVEL_LOW_SVS=0,
	MMRM_TEST_VDD_LEVEL_SVS,
	MMRM_TEST_VDD_LEVEL_SVS_L1,
	MMRM_TEST_VDD_LEVEL_NOM,
	MMRM_TEST_VDD_LEVEL_TURBO,
	MMRM_TEST_VDD_LEVEL_MAX
};

struct clock_rate {
	const char *name;
	u32   domain;
	u32   id;
	u32   clk_rates[MMRM_TEST_VDD_LEVEL_MAX];
};

extern struct  mmrm_test_desc  *waipio_all_testcases[];
extern int waipio_all_testcases_count;

extern struct  mmrm_test_desc  *neo_all_testcases[];
extern int neo_all_testcases_count;

typedef struct test_case_info_s {
	const char name[MMRM_CLK_CLIENT_NAME_SIZE];
	int  vdd_level;
	u32 flags;
	u32 num_hw_blocks;
	u32 client_domain;
	u32 client_id;
	u32 clk_rate[MMRM_TEST_VDD_LEVEL_MAX];
	struct mmrm_client *client;
} test_case_info_t;

extern test_case_info_t  *waipio_testcases[];
extern int waipio_testcases_count;

extern test_case_info_t *waipio_cornercase_testcases [];
extern int waipio_cornercase_testcases_count;

extern test_case_info_t  *neo_testcases[];
extern int neo_testcases_count;

extern test_case_info_t *neo_cornercase_testcases [];
extern int neo_cornercase_testcases_count;

extern struct  mmrm_test  *all_lahaina_testcases[];

void test_mmrm_client(struct platform_device *pdev, int count);
void test_mmrm_single_client_cases(struct platform_device *pdev,
		int index, int count);
void test_mmrm_concurrent_client_cases(struct platform_device *pdev,
		test_case_info_t **testcases, int count);
struct clock_rate *find_clk_by_name(const char *name);
struct clock_rate *get_nth_clock(int nth);
void test_mmrm_switch_volt_corner_client_testcases(struct platform_device *pdev,
		test_case_info_t **testcases, int count);

#endif  // TEST_MMRM_TEST_INTERNAL_H_
