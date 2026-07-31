// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,videocc-canoe.h>

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

#define ACCU_CFG_MASK (0x1f << 21)

static DEFINE_VDD_REGULATORS(vdd_mm, VDD_HIGH_L1 + 1, 1, vdd_corner);
static DEFINE_VDD_REGULATORS(vdd_mxc, VDD_HIGH_L1 + 1, 1, vdd_corner);

static struct clk_vdd_class *video_cc_canoe_regulators[] = {
	&vdd_mm,
	&vdd_mxc,
};

enum {
	P_BI_TCXO,
	P_VIDEO_CC_PLL0_OUT_MAIN,
	P_VIDEO_CC_PLL1_OUT_MAIN,
	P_VIDEO_CC_PLL2_OUT_MAIN,
	P_VIDEO_CC_PLL3_OUT_MAIN,
};

static const struct pll_vco taycan_eko_t_vco[] = {
	{ 249600000, 2500000000, 0 },
};

/* 360.0 MHz Configuration */
static struct alpha_pll_config video_cc_pll0_config = {
	.l = 0x12,
	.cal_l = 0x48,
	.alpha = 0xc000,
	.config_ctl_val = 0x25c400e7,
	.config_ctl_hi_val = 0x0a8062e0,
	.config_ctl_hi1_val = 0xf51dea20,
	.user_ctl_val = 0x00000008,
	.user_ctl_hi_val = 0x00000002,
};

static struct clk_alpha_pll video_cc_pll0 = {
	.offset = 0x0,
	.vco_table = taycan_eko_t_vco,
	.num_vco = ARRAY_SIZE(taycan_eko_t_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_TAYCAN_EKO_T],
	.flags = DISABLE_TO_OFF,
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_pll0",
			.parent_data = &(const struct clk_parent_data) {
				.fw_name = "bi_tcxo",
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_taycan_eko_t_ops,
		},
		.vdd_data = {
			.vdd_class = &vdd_mxc,
			.num_rate_max = VDD_NUM,
			.rate_max = (unsigned long[VDD_NUM]) {
				[VDD_LOWER_D2] = 621000000,
				[VDD_LOWER_D1] = 1600000000,
				[VDD_NOMINAL] = 2000000000,
				[VDD_HIGH] = 2500000000},
		},
	},
};

/* 480.0 MHz Configuration */
static struct alpha_pll_config video_cc_pll1_config = {
	.l = 0x19,
	.cal_l = 0x48,
	.alpha = 0x0,
	.config_ctl_val = 0x25c400e7,
	.config_ctl_hi_val = 0x0a8062e0,
	.config_ctl_hi1_val = 0xf51dea20,
	.user_ctl_val = 0x00000008,
	.user_ctl_hi_val = 0x00000002,
};

static struct clk_alpha_pll video_cc_pll1 = {
	.offset = 0x1000,
	.vco_table = taycan_eko_t_vco,
	.num_vco = ARRAY_SIZE(taycan_eko_t_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_TAYCAN_EKO_T],
	.flags = DISABLE_TO_OFF,
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_pll1",
			.parent_data = &(const struct clk_parent_data) {
				.fw_name = "bi_tcxo",
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_taycan_eko_t_ops,
		},
		.vdd_data = {
			.vdd_class = &vdd_mxc,
			.num_rate_max = VDD_NUM,
			.rate_max = (unsigned long[VDD_NUM]) {
				[VDD_LOWER_D2] = 621000000,
				[VDD_LOWER_D1] = 1600000000,
				[VDD_NOMINAL] = 2000000000,
				[VDD_HIGH] = 2500000000},
		},
	},
};

/* 480.0 MHz Configuration */
static struct alpha_pll_config video_cc_pll2_config = {
	.l = 0x19,
	.cal_l = 0x48,
	.alpha = 0x0,
	.config_ctl_val = 0x25c400e7,
	.config_ctl_hi_val = 0x0a8062e0,
	.config_ctl_hi1_val = 0xf51dea20,
	.user_ctl_val = 0x00000008,
	.user_ctl_hi_val = 0x00000002,
};

static struct clk_alpha_pll video_cc_pll2 = {
	.offset = 0x2000,
	.vco_table = taycan_eko_t_vco,
	.num_vco = ARRAY_SIZE(taycan_eko_t_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_TAYCAN_EKO_T],
	.flags = DISABLE_TO_OFF,
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_pll2",
			.parent_data = &(const struct clk_parent_data) {
				.fw_name = "bi_tcxo",
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_taycan_eko_t_ops,
		},
		.vdd_data = {
			.vdd_class = &vdd_mxc,
			.num_rate_max = VDD_NUM,
			.rate_max = (unsigned long[VDD_NUM]) {
				[VDD_LOWER_D2] = 621000000,
				[VDD_LOWER_D1] = 1600000000,
				[VDD_NOMINAL] = 2000000000,
				[VDD_HIGH] = 2500000000},
		},
	},
};

/* 480.0 MHz Configuration */
static struct alpha_pll_config video_cc_pll3_config = {
	.l = 0x19,
	.cal_l = 0x48,
	.alpha = 0x0,
	.config_ctl_val = 0x25c400e7,
	.config_ctl_hi_val = 0x0a8062e0,
	.config_ctl_hi1_val = 0xf51dea20,
	.user_ctl_val = 0x00000008,
	.user_ctl_hi_val = 0x00000002,
};

static struct clk_alpha_pll video_cc_pll3 = {
	.offset = 0x3000,
	.vco_table = taycan_eko_t_vco,
	.num_vco = ARRAY_SIZE(taycan_eko_t_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_TAYCAN_EKO_T],
	.flags = DISABLE_TO_OFF,
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_pll3",
			.parent_data = &(const struct clk_parent_data) {
				.fw_name = "bi_tcxo",
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_taycan_eko_t_ops,
		},
		.vdd_data = {
			.vdd_class = &vdd_mxc,
			.num_rate_max = VDD_NUM,
			.rate_max = (unsigned long[VDD_NUM]) {
				[VDD_LOWER_D2] = 621000000,
				[VDD_LOWER_D1] = 1600000000,
				[VDD_NOMINAL] = 2000000000,
				[VDD_HIGH] = 2500000000},
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
	{ P_VIDEO_CC_PLL1_OUT_MAIN, 1 },
};

static const struct clk_parent_data video_cc_parent_data_1[] = {
	{ .fw_name = "bi_tcxo" },
	{ .hw = &video_cc_pll1.clkr.hw },
};

static const struct parent_map video_cc_parent_map_2[] = {
	{ P_BI_TCXO, 0 },
	{ P_VIDEO_CC_PLL3_OUT_MAIN, 1 },
};

static const struct clk_parent_data video_cc_parent_data_2[] = {
	{ .fw_name = "bi_tcxo" },
	{ .hw = &video_cc_pll3.clkr.hw },
};

static const struct parent_map video_cc_parent_map_3[] = {
	{ P_BI_TCXO, 0 },
	{ P_VIDEO_CC_PLL2_OUT_MAIN, 1 },
};

static const struct clk_parent_data video_cc_parent_data_3[] = {
	{ .fw_name = "bi_tcxo" },
	{ .hw = &video_cc_pll2.clkr.hw },
};

static const struct parent_map video_cc_parent_map_4[] = {
	{ P_BI_TCXO, 0 },
	{ P_VIDEO_CC_PLL0_OUT_MAIN, 1 },
};

static const struct clk_parent_data video_cc_parent_data_4[] = {
	{ .fw_name = "bi_tcxo" },
	{ .hw = &video_cc_pll0.clkr.hw },
};

static const struct freq_tbl ftbl_video_cc_ahb_clk_src[] = {
	F(19200000, P_BI_TCXO, 1, 0, 0),
	{ }
};

static struct clk_rcg2 video_cc_ahb_clk_src = {
	.cmd_rcgr = 0x8060,
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
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_class = &vdd_mm,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 19200000},
	},
};

static const struct freq_tbl ftbl_video_cc_mvs0_clk_src[] = {
	F(240000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(338000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(420000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(444000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(533000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(630000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(800000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	{ }
};

static const struct freq_tbl ftbl_video_cc_mvs0_clk_src_canoe_v2[] = {
	F(240000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(338000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(420000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(444000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(533000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(630000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(800000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(1000000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	{ }
};

static const struct freq_tbl ftbl_video_cc_mvs0_clk_src_alor[] = {
	F(240000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(338000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(420000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(444000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(560000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(630000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(800000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	F(970000000, P_VIDEO_CC_PLL1_OUT_MAIN, 2, 0, 0),
	{ }
};

static struct clk_rcg2 video_cc_mvs0_clk_src = {
	.cmd_rcgr = 0x8030,
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
		.vdd_classes = video_cc_canoe_regulators,
		.num_vdd_classes = ARRAY_SIZE(video_cc_canoe_regulators),
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 240000000,
			[VDD_LOWER] = 338000000,
			[VDD_LOW] = 420000000,
			[VDD_LOW_L1] = 444000000,
			[VDD_NOMINAL] = 533000000,
			[VDD_HIGH] = 630000000,
			[VDD_HIGH_L1] = 800000000},
	},
};

static const struct freq_tbl ftbl_video_cc_mvs0a_clk_src[] = {
	F(240000000, P_VIDEO_CC_PLL3_OUT_MAIN, 2, 0, 0),
	F(338000000, P_VIDEO_CC_PLL3_OUT_MAIN, 2, 0, 0),
	F(420000000, P_VIDEO_CC_PLL3_OUT_MAIN, 2, 0, 0),
	F(444000000, P_VIDEO_CC_PLL3_OUT_MAIN, 2, 0, 0),
	F(533000000, P_VIDEO_CC_PLL3_OUT_MAIN, 2, 0, 0),
	F(630000000, P_VIDEO_CC_PLL3_OUT_MAIN, 2, 0, 0),
	{ }
};

static struct clk_rcg2 video_cc_mvs0a_clk_src = {
	.cmd_rcgr = 0x8000,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = video_cc_parent_map_2,
	.freq_tbl = ftbl_video_cc_mvs0a_clk_src,
	.enable_safe_config = true,
	.flags = HW_CLK_CTRL_MODE,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "video_cc_mvs0a_clk_src",
		.parent_data = video_cc_parent_data_2,
		.num_parents = ARRAY_SIZE(video_cc_parent_data_2),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_classes = video_cc_canoe_regulators,
		.num_vdd_classes = ARRAY_SIZE(video_cc_canoe_regulators),
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 240000000,
			[VDD_LOWER] = 338000000,
			[VDD_LOW] = 420000000,
			[VDD_LOW_L1] = 444000000,
			[VDD_NOMINAL] = 533000000,
			[VDD_HIGH] = 630000000},
	},
};

static const struct freq_tbl ftbl_video_cc_mvs0b_clk_src[] = {
	F(240000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(338000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(420000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(444000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(533000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(630000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	{ }
};

static const struct freq_tbl ftbl_video_cc_mvs0b_clk_src_canoe_v2[] = {
	F(240000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(338000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(420000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(444000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(533000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(630000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(850000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	{ }
};

static const struct freq_tbl ftbl_video_cc_mvs0b_clk_src_alor[] = {
	F(240000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(338000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(420000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(444000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(533000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(630000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	F(800000000, P_VIDEO_CC_PLL2_OUT_MAIN, 2, 0, 0),
	{ }
};

static struct clk_rcg2 video_cc_mvs0b_clk_src = {
	.cmd_rcgr = 0x8018,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = video_cc_parent_map_3,
	.freq_tbl = ftbl_video_cc_mvs0b_clk_src,
	.enable_safe_config = true,
	.flags = HW_CLK_CTRL_MODE,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "video_cc_mvs0b_clk_src",
		.parent_data = video_cc_parent_data_3,
		.num_parents = ARRAY_SIZE(video_cc_parent_data_3),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_classes = video_cc_canoe_regulators,
		.num_vdd_classes = ARRAY_SIZE(video_cc_canoe_regulators),
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 240000000,
			[VDD_LOWER] = 338000000,
			[VDD_LOW] = 420000000,
			[VDD_LOW_L1] = 444000000,
			[VDD_NOMINAL] = 533000000,
			[VDD_HIGH] = 630000000},
	},
};

static const struct freq_tbl ftbl_video_cc_mvs0c_clk_src[] = {
	F(360000000, P_VIDEO_CC_PLL0_OUT_MAIN, 1, 0, 0),
	F(507000000, P_VIDEO_CC_PLL0_OUT_MAIN, 1, 0, 0),
	F(630000000, P_VIDEO_CC_PLL0_OUT_MAIN, 1, 0, 0),
	F(666000000, P_VIDEO_CC_PLL0_OUT_MAIN, 1, 0, 0),
	F(800000000, P_VIDEO_CC_PLL0_OUT_MAIN, 1, 0, 0),
	F(1104000000, P_VIDEO_CC_PLL0_OUT_MAIN, 1, 0, 0),
	F(1260000000, P_VIDEO_CC_PLL0_OUT_MAIN, 1, 0, 0),
	{ }
};

static struct clk_rcg2 video_cc_mvs0c_clk_src = {
	.cmd_rcgr = 0x8048,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = video_cc_parent_map_4,
	.freq_tbl = ftbl_video_cc_mvs0c_clk_src,
	.enable_safe_config = true,
	.flags = HW_CLK_CTRL_MODE,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "video_cc_mvs0c_clk_src",
		.parent_data = video_cc_parent_data_4,
		.num_parents = ARRAY_SIZE(video_cc_parent_data_4),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_classes = video_cc_canoe_regulators,
		.num_vdd_classes = ARRAY_SIZE(video_cc_canoe_regulators),
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 360000000,
			[VDD_LOWER] = 507000000,
			[VDD_LOW] = 630000000,
			[VDD_LOW_L1] = 666000000,
			[VDD_NOMINAL] = 800000000,
			[VDD_HIGH] = 1104000000,
			[VDD_HIGH_L1] = 1260000000},
	},
};

static struct clk_rcg2 video_cc_xo_clk_src = {
	.cmd_rcgr = 0x8194,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = video_cc_parent_map_0,
	.freq_tbl = ftbl_video_cc_ahb_clk_src,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "video_cc_xo_clk_src",
		.parent_data = video_cc_parent_data_0_ao,
		.num_parents = ARRAY_SIZE(video_cc_parent_data_0_ao),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_class = &vdd_mm,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 19200000},
	},
};

static struct clk_branch video_cc_mvs0_clk = {
	.halt_reg = 0x80d0,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x80d0,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x80d0,
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

static struct clk_mem_branch video_cc_mvs0_freerun_clk = {
	.mem_enable_reg = 0x80E4,
	.mem_ack_reg =  0x80E4,
	.mem_enable_mask = BIT(3),
	.mem_enable_ack_mask = 0xc00,
	.mem_enable_inverted = true,
	.branch = {
		.halt_reg = 0x80e0,
		.halt_check = BRANCH_HALT,
		.clkr = {
			.enable_reg = 0x80e0,
			.enable_mask = BIT(0),
			.hw.init = &(const struct clk_init_data) {
				.name = "video_cc_mvs0_freerun_clk",
				.parent_hws = (const struct clk_hw*[]) {
					&video_cc_mvs0_clk_src.clkr.hw,
				},
				.num_parents = 1,
				.flags = CLK_SET_RATE_PARENT,
				.ops = &clk_branch2_mem_ops,
			},
		},
	}
};

static struct clk_branch video_cc_mvs0_shift_clk = {
	.halt_reg = 0x81b4,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x81b4,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x81b4,
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

static struct clk_branch video_cc_mvs0_vpp0_clk = {
	.halt_reg = 0x8134,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x8134,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x8134,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0_vpp0_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0_vpp0_freerun_clk = {
	.halt_reg = 0x8144,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x8144,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0_vpp0_freerun_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0_vpp1_clk = {
	.halt_reg = 0x8108,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x8108,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x8108,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0_vpp1_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0_vpp1_freerun_clk = {
	.halt_reg = 0x8118,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x8118,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0_vpp1_freerun_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0a_clk = {
	.halt_reg = 0x8090,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x8090,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x8090,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0a_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0a_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0a_freerun_clk = {
	.halt_reg = 0x80a0,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x80a0,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0a_freerun_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0a_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0b_clk = {
	.halt_reg = 0x80bc,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x80bc,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x80bc,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0b_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0b_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0b_freerun_clk = {
	.halt_reg = 0x80cc,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x80cc,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0b_freerun_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0b_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0c_clk = {
	.halt_reg = 0x8164,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x8164,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x8164,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0c_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0c_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0c_freerun_clk = {
	.halt_reg = 0x8174,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x8174,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "video_cc_mvs0c_freerun_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&video_cc_mvs0c_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch video_cc_mvs0c_shift_clk = {
	.halt_reg = 0x81b8,
	.halt_check = BRANCH_HALT_VOTED,
	.hwcg_reg = 0x81b8,
	.hwcg_bit = 1,
	.clkr = {
		.enable_reg = 0x81b8,
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

static struct gdsc video_cc_mvs0_vpp0_gdsc = {
	.gdscr = 0x8120,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0xf,
	.pd = {
		.name = "video_cc_mvs0_vpp0_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE | HW_CTRL_TRIGGER,
	.supply = "vdd_mm_mxc_voter",
};

static struct gdsc video_cc_mvs0_vpp1_gdsc = {
	.gdscr = 0x80f4,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0xf,
	.pd = {
		.name = "video_cc_mvs0_vpp1_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE | HW_CTRL_TRIGGER,
	.supply = "vdd_mm_mxc_voter",
};

static struct gdsc video_cc_mvs0a_gdsc = {
	.gdscr = 0x807c,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0xf,
	.pd = {
		.name = "video_cc_mvs0a_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE | HW_CTRL_TRIGGER,
	.supply = "vdd_mm_mxc_voter",
};

static struct gdsc video_cc_mvs0c_gdsc = {
	.gdscr = 0x814c,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0x6,
	.pd = {
		.name = "video_cc_mvs0c_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE,
	.supply = "vdd_mm_mxc_voter",
};

static struct gdsc video_cc_mvs0_gdsc = {
	.gdscr = 0x80a8,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0x6,
	.pd = {
		.name = "video_cc_mvs0_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.parent = &video_cc_mvs0c_gdsc.pd,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE | HW_CTRL_TRIGGER,
	.supply = "vdd_mm_mxc_voter",
};

static struct clk_regmap *video_cc_canoe_clocks[] = {
	[VIDEO_CC_AHB_CLK_SRC] = &video_cc_ahb_clk_src.clkr,
	[VIDEO_CC_MVS0_CLK] = &video_cc_mvs0_clk.clkr,
	[VIDEO_CC_MVS0_CLK_SRC] = &video_cc_mvs0_clk_src.clkr,
	[VIDEO_CC_MVS0_FREERUN_CLK] = &video_cc_mvs0_freerun_clk.branch.clkr,
	[VIDEO_CC_MVS0_SHIFT_CLK] = &video_cc_mvs0_shift_clk.clkr,
	[VIDEO_CC_MVS0_VPP0_CLK] = &video_cc_mvs0_vpp0_clk.clkr,
	[VIDEO_CC_MVS0_VPP0_FREERUN_CLK] = &video_cc_mvs0_vpp0_freerun_clk.clkr,
	[VIDEO_CC_MVS0_VPP1_CLK] = &video_cc_mvs0_vpp1_clk.clkr,
	[VIDEO_CC_MVS0_VPP1_FREERUN_CLK] = &video_cc_mvs0_vpp1_freerun_clk.clkr,
	[VIDEO_CC_MVS0A_CLK] = &video_cc_mvs0a_clk.clkr,
	[VIDEO_CC_MVS0A_CLK_SRC] = &video_cc_mvs0a_clk_src.clkr,
	[VIDEO_CC_MVS0A_FREERUN_CLK] = &video_cc_mvs0a_freerun_clk.clkr,
	[VIDEO_CC_MVS0B_CLK] = &video_cc_mvs0b_clk.clkr,
	[VIDEO_CC_MVS0B_CLK_SRC] = &video_cc_mvs0b_clk_src.clkr,
	[VIDEO_CC_MVS0B_FREERUN_CLK] = &video_cc_mvs0b_freerun_clk.clkr,
	[VIDEO_CC_MVS0C_CLK] = &video_cc_mvs0c_clk.clkr,
	[VIDEO_CC_MVS0C_CLK_SRC] = &video_cc_mvs0c_clk_src.clkr,
	[VIDEO_CC_MVS0C_FREERUN_CLK] = &video_cc_mvs0c_freerun_clk.clkr,
	[VIDEO_CC_MVS0C_SHIFT_CLK] = &video_cc_mvs0c_shift_clk.clkr,
	[VIDEO_CC_PLL0] = &video_cc_pll0.clkr,
	[VIDEO_CC_PLL1] = &video_cc_pll1.clkr,
	[VIDEO_CC_PLL2] = &video_cc_pll2.clkr,
	[VIDEO_CC_PLL3] = &video_cc_pll3.clkr,
	[VIDEO_CC_XO_CLK_SRC] = &video_cc_xo_clk_src.clkr,
};

static struct gdsc *video_cc_canoe_gdscs[] = {
	[VIDEO_CC_MVS0A_GDSC] = &video_cc_mvs0a_gdsc,
	[VIDEO_CC_MVS0_GDSC] = &video_cc_mvs0_gdsc,
	[VIDEO_CC_MVS0_VPP1_GDSC] = &video_cc_mvs0_vpp1_gdsc,
	[VIDEO_CC_MVS0_VPP0_GDSC] = &video_cc_mvs0_vpp0_gdsc,
	[VIDEO_CC_MVS0C_GDSC] = &video_cc_mvs0c_gdsc,
};

static const struct qcom_reset_map video_cc_canoe_resets[] = {
	[VIDEO_CC_INTERFACE_BCR] = { 0x8178 },
	[VIDEO_CC_MVS0_BCR] = { 0x80a4 },
	[VIDEO_CC_MVS0_VPP0_BCR] = { 0x811c },
	[VIDEO_CC_MVS0_VPP1_BCR] = { 0x80f0 },
	[VIDEO_CC_MVS0A_BCR] = { 0x8078 },
	[VIDEO_CC_MVS0C_CLK_ARES] = { 0x8164, 2 },
	[VIDEO_CC_MVS0C_BCR] = { 0x8148 },
	[VIDEO_CC_MVS0_FREERUN_CLK_ARES] = { 0x80e0, 2 },
	[VIDEO_CC_MVS0C_FREERUN_CLK_ARES] = { 0x8174, 2 },
	[VIDEO_CC_XO_CLK_ARES] = { 0x81ac, 2 },
};

static const struct regmap_config video_cc_canoe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0xa010,
	.fast_io = true,
};

static struct qcom_cc_desc video_cc_canoe_desc = {
	.config = &video_cc_canoe_regmap_config,
	.clks = video_cc_canoe_clocks,
	.num_clks = ARRAY_SIZE(video_cc_canoe_clocks),
	.resets = video_cc_canoe_resets,
	.num_resets = ARRAY_SIZE(video_cc_canoe_resets),
	.clk_regulators = video_cc_canoe_regulators,
	.num_clk_regulators = ARRAY_SIZE(video_cc_canoe_regulators),
	.gdscs = video_cc_canoe_gdscs,
	.num_gdscs = ARRAY_SIZE(video_cc_canoe_gdscs),
};

static const struct of_device_id video_cc_canoe_match_table[] = {
	{ .compatible = "qcom,canoe-videocc" },
	{ .compatible = "qcom,alor-videocc" },
	{ .compatible = "qcom,canoe-videocc-v2" },
	{ .compatible = "qcom,whale-videocc" },
	{ }
};
MODULE_DEVICE_TABLE(of, video_cc_canoe_match_table);

static void video_cc_canoe_fixup_alor(struct regmap *regmap)
{
	video_cc_canoe_clocks[VIDEO_CC_PLL3] = NULL;
	video_cc_canoe_clocks[VIDEO_CC_MVS0A_CLK] = NULL;
	video_cc_canoe_clocks[VIDEO_CC_MVS0A_CLK_SRC] = NULL;
	video_cc_canoe_clocks[VIDEO_CC_MVS0A_FREERUN_CLK] = NULL;
	video_cc_canoe_clocks[VIDEO_CC_MVS0_VPP1_CLK] = NULL;
	video_cc_canoe_clocks[VIDEO_CC_MVS0_VPP1_FREERUN_CLK] = NULL;

	video_cc_canoe_gdscs[VIDEO_CC_MVS0A_GDSC] = NULL;
	video_cc_canoe_gdscs[VIDEO_CC_MVS0_VPP1_GDSC] = NULL;

	video_cc_mvs0_clk_src.freq_tbl = ftbl_video_cc_mvs0_clk_src_alor;
	video_cc_mvs0_clk_src.clkr.vdd_data.rate_max[VDD_NOMINAL] = 560000000;
	video_cc_mvs0_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L0] = 800000000;
	video_cc_mvs0_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L1] = 970000000;
	video_cc_mvs0b_clk_src.freq_tbl = ftbl_video_cc_mvs0b_clk_src_alor;
	video_cc_mvs0b_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L1] = 800000000;
	video_cc_mvs0c_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L0] = 1260000000;

	video_cc_pll0_config.config_ctl_hi_val = 0x0a8060e0;
	video_cc_pll1_config.config_ctl_hi_val = 0x0a8060e0;
	video_cc_pll2_config.config_ctl_hi_val = 0x0a8060e0;

}

static int video_cc_canoe_fixup(struct platform_device *pdev, struct regmap *regmap)
{
	const char *compat = NULL;
	int compatlen = 0;

	compat = of_get_property(pdev->dev.of_node, "compatible", &compatlen);
	if (!compat || compatlen <= 0)
		return -EINVAL;

	/*
	 * Update VIDEO_CC_SPARE1 to have same clk_on for video_cc_mvs0_clk,
	 * video_cc_mvs0_vpp0_clk, video_cc_mvs0_vpp1_clk during core reset by default.
	 */
	regmap_update_bits(regmap, 0x9f24, BIT(0), BIT(0));

	if (!strcmp(compat, "qcom,alor-videocc")) {
		video_cc_canoe_fixup_alor(regmap);
		return 0;
	}

	if (!strcmp(compat, "qcom,canoe-videocc-v2")) {
		video_cc_mvs0_clk_src.freq_tbl = ftbl_video_cc_mvs0_clk_src_canoe_v2;
		video_cc_mvs0b_clk_src.freq_tbl = ftbl_video_cc_mvs0b_clk_src_canoe_v2;
		video_cc_mvs0_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L0] = 800000000;
		video_cc_mvs0_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L1] = 1000000000;
		video_cc_mvs0b_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L1] = 850000000;
		video_cc_mvs0c_clk_src.clkr.vdd_data.rate_max[VDD_HIGH_L0] = 1260000000;
	}

	if (!strcmp(compat, "qcom,whale-videocc")) {
		video_cc_pll0_config.config_ctl_hi_val = 0x0a8060e0;
		video_cc_pll1_config.config_ctl_hi_val = 0x0a8060e0;
		video_cc_pll2_config.config_ctl_hi_val = 0x0a8060e0;
		video_cc_pll3_config.config_ctl_hi_val = 0x0a8060e0;
	}

	clk_taycan_eko_t_pll_configure(&video_cc_pll3, regmap, &video_cc_pll3_config);

	/*
	 * Maximize MVS0A CFG3 ctl data download delay and enable memory redundancy.
	 */
	regmap_update_bits(regmap, 0x8088, ACCU_CFG_MASK, ACCU_CFG_MASK);

	return 0;
}


static int video_cc_canoe_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int ret;

	regmap = qcom_cc_map(pdev, &video_cc_canoe_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = qcom_cc_runtime_init(pdev, &video_cc_canoe_desc);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret)
		return ret;

	ret = video_cc_canoe_fixup(pdev, regmap);
	if (ret)
		return ret;

	clk_taycan_eko_t_pll_configure(&video_cc_pll0, regmap, &video_cc_pll0_config);
	clk_taycan_eko_t_pll_configure(&video_cc_pll1, regmap, &video_cc_pll1_config);
	clk_taycan_eko_t_pll_configure(&video_cc_pll2, regmap, &video_cc_pll2_config);

	/*
	 *	Maximize ctl data download delay and enable memory redundancy:
	 *	MVS0 CFG3
	 *	MVS0 VPP1 CFG3
	 *	MVS0 VPP0 CFG3
	 *	MVS0C CFG3
	 */
	regmap_update_bits(regmap, 0x80b4, ACCU_CFG_MASK, ACCU_CFG_MASK);
	regmap_update_bits(regmap, 0x8100, ACCU_CFG_MASK, ACCU_CFG_MASK);
	regmap_update_bits(regmap, 0x812c, ACCU_CFG_MASK, ACCU_CFG_MASK);
	regmap_update_bits(regmap, 0x8158, ACCU_CFG_MASK, ACCU_CFG_MASK);

	/*
	 * Keep clocks always enabled:
	 *	video_cc_ahb_clk
	 *	video_cc_sleep_clk
	 *	video_cc_ts_xo_clk
	 *	video_cc_xo_clk
	 */
	regmap_update_bits(regmap, 0x817c, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x81bc, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x81b0, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x81ac, BIT(0), BIT(0));

	ret = qcom_cc_really_probe(&pdev->dev, &video_cc_canoe_desc, regmap);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "Failed to register VIDEO CC clocks\n");
		goto err;
	}

	dev_info(&pdev->dev, "Registered VIDEO CC clocks\n");

err:
	pm_runtime_put_sync(&pdev->dev);

	return ret;
}

static void video_cc_canoe_sync_state(struct device *dev)
{
	qcom_cc_sync_state(dev, &video_cc_canoe_desc);
}

static const struct dev_pm_ops video_cc_canoe_pm_ops = {
	SET_RUNTIME_PM_OPS(qcom_cc_runtime_suspend, qcom_cc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver video_cc_canoe_driver = {
	.probe = video_cc_canoe_probe,
	.driver = {
		.name = "videocc-canoe",
		.of_match_table = video_cc_canoe_match_table,
		.sync_state = video_cc_canoe_sync_state,
		.pm = &video_cc_canoe_pm_ops,
	},
};

module_platform_driver(video_cc_canoe_driver);

MODULE_DESCRIPTION("QTI VIDEOCC CANOE Driver");
MODULE_LICENSE("GPL");
