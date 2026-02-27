// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017, 2021 The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include "audio-ext-clk-up.h"

static int __init wcd9xxx_soc_init(void)
{
	int ret = 0;

	ret = audio_ref_clk_platform_init();
	if (ret)
		pr_err("%s: init extclk fail: %d\n", __func__, ret);

	return ret;
}
module_init(wcd9xxx_soc_init);

static void __exit wcd9xxx_soc_exit(void)
{
	audio_ref_clk_platform_exit();
}
module_exit(wcd9xxx_soc_exit);

MODULE_DESCRIPTION("WCD9XXX CODEC soc init driver");
MODULE_LICENSE("GPL v2");
