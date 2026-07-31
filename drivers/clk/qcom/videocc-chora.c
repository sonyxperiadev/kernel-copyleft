// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,chora-videocc.h>

#include "clk-alpha-pll.h"
#include "clk-branch.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-regmap.h"
#include "clk-regmap-divider.h"
#include "clk-regmap-mux.h"
#include "common.h"
#include "gdsc.h"
#include "reset.h"
#include "vdd-level.h"

static DEFINE_VDD_REGULATORS(vdd_cx, VDD_HIGH + 1, 1, vdd_corner);
static DEFINE_VDD_REGULATORS(vdd_mx, VDD_HIGH + 1, 1, vdd_corner);

static struct clk_vdd_class *video_cc_chora_regulators[] = {
	&vdd_cx,
	&vdd_mx,
};

enum {
	P_BI_TCXO,
	P_SLEEP_CLK,
	P_VIDEO_CC_PLL0_OUT_EVEN,
	P_VIDEO_CC_PLL0_OUT_MAIN,
	P_VIDEO_CC_PLL0_OUT_ODD,
};

static const struct pll_vco lucid_ole_vco[] = {
	{ 249600000, 2300000000, 0 },
};

/* 330.0 MHz Configuration */
static const struct alpha_pll_config video_cc_pll0_config = {
	.l = 0x11,
	.cal_l = 0x44,
	.cal_l_ringosc = 0x44,
	.alpha = 0x3000,
	.config_ctl_val = 0x20485699,
	.config_ctl_hi_val = 0x00182261,
	.config_ctl_hi1_val = 0x82aa299c,
	.test_ctl_val = 0x00000000,
	.test_ctl_hi_val = 0x00000003,
	.test_ctl_hi1_val = 0x00009000,
	.test_ctl_hi2_val = 0x00000034,
	.user_ctl_val = 0x00000000,
	.user_ctl_hi_val = 0x00000005,
};

static struct clk_alpha_pll video_cc_pll0 = {
	.offset = 0x0,
	.vco_table = lucid_ole_vco,
	.num_vco = ARRAY_SIZE(lucid_ole_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_LUCID_OLE],
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_pll0",
			.parent_data = &(const struct clk_parent_data) {
				.fw_name = "bi_tcxo",
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_lucid_ole_ops,
		},
		.vdd_data = {
			.vdd_class = &vdd_mx,
			.num_rate_max = VDD_NUM,
			.rate_max = (unsigned long[VDD_NUM]) {
				[VDD_LOWER_D1] = 615000000,
				[VDD_LOW] = 1100000000,
				[VDD_LOW_L1] = 1600000000,
				[VDD_NOMINAL] = 2000000000,
				[VDD_HIGH_L1] = 2300000000},
		},
	},
};

static const struct parent_map video_cc_parent_map_0[] = {
	{ P_BI_TCXO, 0 },
};

static const struct clk_parent_data video_cc_parent_data_0_ao[] = {
	{ .fw_name = "bi_tcxo_ao" },
};

static const struct parent_map video_cc_parent_map_1[] = {
	{ P_BI_TCXO, 0 },
	{ P_VIDEO_CC_PLL0_OUT_MAIN, 1 },
	{ P_VIDEO_CC_PLL0_OUT_EVEN, 2 },
	{ P_VIDEO_CC_PLL0_OUT_ODD, 3 },
};

static const struct clk_parent_data video_cc_parent_data_1[] = {
	{ .fw_name = "bi_tcxo" },
	{ .hw = &video_cc_pll0.clkr.hw },
	{ .hw = &video_cc_pll0.clkr.hw },
	{ .hw = &video_cc_pll0.clkr.hw },
};

static const struct freq_tbl ftbl_video_cc_ahb_clk_src[] = {
	F(19200000, P_BI_TCXO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 video_cc_ahb_clk_src = {
	.cmd_rcgr = 0x8018,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = video_cc_parent_map_0,
	.freq_tbl = ftbl_video_cc_ahb_clk_src,
	.enable_safe_config = true,
	.flags = HW_CLK_CTRL_MODE,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "video_cc_ahb_clk_src",
		.parent_data = video_cc_parent_data_0_ao,
		.num_parents = ARRAY_SIZE(video_cc_parent_data_0_ao),
		.ops = &clk_rcg2_ops,
	},
};

static const struct freq_tbl ftbl_video_cc_mvs0_clk_src[] = {
	F(165000000, P_VIDEO_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(270000000, P_VIDEO_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(366000000, P_VIDEO_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(424000000, P_VIDEO_CC_PLL0_OUT_MAIN, 2, 0, 0),
	F(460000000, P_VIDEO_CC_PLL0_OUT_MAIN, 2, 0, 0),
	{ }
};

static struct clk_rcg2 video_cc_mvs0_clk_src = {
	.cmd_rcgr = 0x8000,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = video_cc_parent_map_1,
	.freq_tbl = ftbl_video_cc_mvs0_clk_src,
	.enable_safe_config = true,
	.flags = HW_CLK_CTRL_MODE,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "video_cc_mvs0_clk_src",
		.parent_data = video_cc_parent_data_1,
		.num_parents = ARRAY_SIZE(video_cc_parent_data_1),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_classes = video_cc_chora_regulators,
		.num_vdd_classes = ARRAY_SIZE(video_cc_chora_regulators),
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER] = 165000000,
			[VDD_LOW] = 270000000,
			[VDD_LOW_L1] = 366000000,
			[VDD_NOMINAL] = 424000000,
			[VDD_HIGH] = 460000000},
	},
};

static struct clk_rcg2 video_cc_xo_clk_src = {
	.cmd_rcgr = 0x80a0,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = video_cc_parent_map_0,
	.freq_tbl = ftbl_video_cc_ahb_clk_src,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "video_cc_xo_clk_src",
		.parent_data = video_cc_parent_data_0_ao,
		.num_parents = ARRAY_SIZE(video_cc_parent_data_0_ao),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_branch video_cc_mvs0_axi_clk = {
	.halt_reg = 0xb014,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0xb014,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0_clk = {
	.halt_reg = 0x8074,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x8074,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x8074,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0_shift_clk = {
	.halt_reg = 0x80bc,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x80bc,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0_shift_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_xo_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0c_clk = {
	.halt_reg = 0x804c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x804c,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0c_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0c_shift_clk = {
	.halt_reg = 0x80c0,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x80c0,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0c_shift_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_xo_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvsc_ctl_axi_clk = {
	.halt_reg = 0xb004,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0xb004,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvsc_ctl_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_trig_clk = {
	.halt_reg = 0xb024,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0xb024,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_trig_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_venus_ahb_clk = {
	.halt_reg = 0xb02c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0xb02c,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_venus_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct gdsc video_cc_mvs0c_gdsc = {
	.gdscr = 0x8034,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0x6,
	.pd = {
		.name = "video_cc_mvs0c_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE,
	.supply = "vdd_cx",
};

static struct gdsc video_cc_mvs0_gdsc = {
	.gdscr = 0x8060,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0x6,
	.pd = {
		.name = "video_cc_mvs0_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.parent = &video_cc_mvs0c_gdsc.pd,
	.flags = HW_CTRL_TRIGGER | POLL_CFG_GDSCR | RETAIN_FF_ENABLE,
	.supply = "vdd_cx",
};

static struct clk_regmap *video_cc_chora_clocks[] = {
	[VIDEO_CC_AHB_CLK_SRC] = &video_cc_ahb_clk_src.clkr,
	[VIDEO_CC_MVS0_AXI_CLK] = &video_cc_mvs0_axi_clk.clkr,
	[VIDEO_CC_MVS0_CLK] = &video_cc_mvs0_clk.clkr,
	[VIDEO_CC_MVS0_CLK_SRC] = &video_cc_mvs0_clk_src.clkr,
	[VIDEO_CC_MVS0_SHIFT_CLK] = &video_cc_mvs0_shift_clk.clkr,
	[VIDEO_CC_MVS0C_CLK] = &video_cc_mvs0c_clk.clkr,
	[VIDEO_CC_MVS0C_SHIFT_CLK] = &video_cc_mvs0c_shift_clk.clkr,
	[VIDEO_CC_MVSC_CTL_AXI_CLK] = &video_cc_mvsc_ctl_axi_clk.clkr,
	[VIDEO_CC_PLL0] = &video_cc_pll0.clkr,
	[VIDEO_CC_TRIG_CLK] = &video_cc_trig_clk.clkr,
	[VIDEO_CC_VENUS_AHB_CLK] = &video_cc_venus_ahb_clk.clkr,
	[VIDEO_CC_XO_CLK_SRC] = &video_cc_xo_clk_src.clkr,
};

static struct gdsc *video_cc_chora_gdscs[] = {
	[VIDEO_CC_MVS0_GDSC] = &video_cc_mvs0_gdsc,
	[VIDEO_CC_MVS0C_GDSC] = &video_cc_mvs0c_gdsc,
};

static const struct qcom_reset_map video_cc_chora_resets[] = {
	[VIDEO_CC_INTERFACE_AHB_BCR] = { 0x8084 },
	[VIDEO_CC_INTERFACE_BCR] = { 0xb000 },
	[VIDEO_CC_MVS0_BCR] = { 0x805c },
	[VIDEO_CC_MVS0C_CLK_ARES] = { 0x804c, 2 },
	[VIDEO_CC_MVS0C_BCR] = { 0x8030 },
};

static const struct regmap_config video_cc_chora_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0xb034,
	.fast_io = true,
};

static struct qcom_cc_desc video_cc_chora_desc = {
	.config = &video_cc_chora_regmap_config,
	.clks = video_cc_chora_clocks,
	.num_clks = ARRAY_SIZE(video_cc_chora_clocks),
	.resets = video_cc_chora_resets,
	.num_resets = ARRAY_SIZE(video_cc_chora_resets),
	.clk_regulators = video_cc_chora_regulators,
	.num_clk_regulators = ARRAY_SIZE(video_cc_chora_regulators),
	.gdscs = video_cc_chora_gdscs,
	.num_gdscs = ARRAY_SIZE(video_cc_chora_gdscs),
};

static const struct of_device_id video_cc_chora_match_table[] = {
	{ .compatible = "qcom,chora-videocc" },
	{ }
};
MODULE_DEVICE_TABLE(of, video_cc_chora_match_table);

static int video_cc_chora_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int ret;

	regmap = qcom_cc_map(pdev, &video_cc_chora_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	clk_lucid_ole_pll_configure(&video_cc_pll0, regmap, &video_cc_pll0_config);

	/*
	 * Keep clocks always enabled:
	 *	video_cc_ahb_clk
	 *	video_cc_sleep_clk
	 *	video_cc_xo_clk
	 */
	regmap_update_bits(regmap, 0x8088, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x80dc, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x80b8, BIT(0), BIT(0));

	ret = qcom_cc_really_probe(&pdev->dev, &video_cc_chora_desc, regmap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register VIDEO CC clocks ret=%d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "Registered VIDEO CC clocks\n");

	return ret;
}

static void video_cc_chora_sync_state(struct device *dev)
{
	qcom_cc_sync_state(dev, &video_cc_chora_desc);
}

static struct platform_driver video_cc_chora_driver = {
	.probe = video_cc_chora_probe,
	.driver = {
		.name = "videocc-chora",
		.of_match_table = video_cc_chora_match_table,
		.sync_state = video_cc_chora_sync_state,
	},
};

module_platform_driver(video_cc_chora_driver);

MODULE_DESCRIPTION("QTI VIDEOCC CHORA Driver");
MODULE_LICENSE("GPL");
