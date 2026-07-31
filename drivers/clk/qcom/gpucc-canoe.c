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

#include <dt-bindings/clock/qcom,gpucc-canoe.h>
#include <dt-bindings/clock/qcom,gxclkctl-canoe.h>

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

static DEFINE_VDD_REGULATORS(vdd_cx, VDD_NOMINAL + 1, 1, vdd_corner);
static DEFINE_VDD_REGULATORS(vdd_mx, VDD_NOMINAL + 1, 1, vdd_corner);

static struct clk_vdd_class *gpu_cc_canoe_regulators[] = {
	&vdd_cx,
	&vdd_mx,
};

enum {
	P_BI_TCXO,
	P_GPLL0_OUT_MAIN,
	P_GPLL0_OUT_MAIN_DIV,
	P_GPU_CC_PLL0_OUT_EVEN,
	P_GPU_CC_PLL0_OUT_MAIN,
	P_GPU_CC_PLL0_OUT_ODD,
};

static const struct pll_vco taycan_eko_t_vco[] = {
	{ 249600000, 2500000000, 0 },
};

/* 950.0 MHz Configuration */
static struct alpha_pll_config gpu_cc_pll0_config = {
	.l = 0x31,
	.cal_l = 0x48,
	.alpha = 0x7aaa,
	.config_ctl_val = 0x25c400e7,
	.config_ctl_hi_val = 0x0a8062e0,
	.config_ctl_hi1_val = 0xf51dea20,
	.user_ctl_val = 0x00000408,
	.user_ctl_hi_val = 0x00000002,
};

static struct clk_alpha_pll gpu_cc_pll0 = {
	.offset = 0x0,
	.vco_table = taycan_eko_t_vco,
	.num_vco = ARRAY_SIZE(taycan_eko_t_vco),
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_TAYCAN_EKO_T],
	.flags = DISABLE_TO_OFF,
	.clkr = {
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_pll0",
			.parent_data = &(const struct clk_parent_data) {
				.fw_name = "bi_tcxo",
			},
			.num_parents = 1,
			.ops = &clk_alpha_pll_taycan_eko_t_ops,
		},
		.vdd_data = {
			.vdd_class = &vdd_mx,
			.num_rate_max = VDD_NUM,
			.rate_max = (unsigned long[VDD_NUM]) {
				[VDD_LOWER_D2] = 621000000,
				[VDD_LOWER_D1] = 1600000000,
				[VDD_NOMINAL] = 2000000000,
				[VDD_HIGH] = 2500000000},
		},
	},
};

static const struct clk_div_table post_div_table_gpu_cc_pll0_out_even[] = {
	{ 0x1, 2 },
	{ }
};

static struct clk_alpha_pll_postdiv gpu_cc_pll0_out_even = {
	.offset = 0x0,
	.post_div_shift = 10,
	.post_div_table = post_div_table_gpu_cc_pll0_out_even,
	.num_post_div = ARRAY_SIZE(post_div_table_gpu_cc_pll0_out_even),
	.width = 4,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_TAYCAN_EKO_T],
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "gpu_cc_pll0_out_even",
		.parent_hws = (const struct clk_hw*[]) {
			&gpu_cc_pll0.clkr.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_alpha_pll_postdiv_taycan_eko_t_ops,
	},
};

static const struct parent_map gpu_cc_parent_map_0[] = {
	{ P_BI_TCXO, 0 },
	{ P_GPLL0_OUT_MAIN, 5 },
	{ P_GPLL0_OUT_MAIN_DIV, 6 },
};

static const struct clk_parent_data gpu_cc_parent_data_0[] = {
	{ .fw_name = "bi_tcxo" },
	{ .fw_name = "gpll0_out_main" },
	{ .fw_name = "gpll0_out_main_div" },
};

static const struct parent_map gpu_cc_parent_map_1[] = {
	{ P_BI_TCXO, 0 },
	{ P_GPU_CC_PLL0_OUT_MAIN, 1 },
	{ P_GPU_CC_PLL0_OUT_EVEN, 2 },
	{ P_GPU_CC_PLL0_OUT_ODD, 3 },
	{ P_GPLL0_OUT_MAIN, 5 },
	{ P_GPLL0_OUT_MAIN_DIV, 6 },
};

static const struct clk_parent_data gpu_cc_parent_data_1[] = {
	{ .fw_name = "bi_tcxo" },
	{ .hw = &gpu_cc_pll0.clkr.hw },
	{ .hw = &gpu_cc_pll0_out_even.clkr.hw },
	{ .hw = &gpu_cc_pll0.clkr.hw },
	{ .fw_name = "gpll0_out_main" },
	{ .fw_name = "gpll0_out_main_div" },
};

static const struct freq_tbl ftbl_gpu_cc_gmu_clk_src[] = {
	F(19200000, P_BI_TCXO, 1, 0, 0),
	F(200000000, P_GPU_CC_PLL0_OUT_EVEN, 1, 0, 0),
	F(300000000, P_GPU_CC_PLL0_OUT_EVEN, 1, 0, 0),
	F(475000000, P_GPU_CC_PLL0_OUT_EVEN, 1, 0, 0),
	F(575000000, P_GPU_CC_PLL0_OUT_EVEN, 1, 0, 0),
	F(700000000, P_GPU_CC_PLL0_OUT_EVEN, 1, 0, 0),
	F(725000000, P_GPU_CC_PLL0_OUT_EVEN, 1, 0, 0),
	F(750000000, P_GPU_CC_PLL0_OUT_EVEN, 1, 0, 0),
	{ }
};

static struct clk_rcg2 gpu_cc_gmu_clk_src = {
	.cmd_rcgr = 0x9318,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = gpu_cc_parent_map_1,
	.freq_tbl = ftbl_gpu_cc_gmu_clk_src,
	.enable_safe_config = true,
	.flags = HW_CLK_CTRL_MODE,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "gpu_cc_gmu_clk_src",
		.parent_data = gpu_cc_parent_data_1,
		.num_parents = ARRAY_SIZE(gpu_cc_parent_data_1),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_classes = gpu_cc_canoe_regulators,
		.num_vdd_classes = ARRAY_SIZE(gpu_cc_canoe_regulators),
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 475000000,
			[VDD_LOWER] = 575000000,
			[VDD_LOW] = 700000000,
			[VDD_LOW_L1] = 725000000,
			[VDD_NOMINAL] = 750000000},
	},
};

static const struct freq_tbl ftbl_gpu_cc_hub_clk_src[] = {
	F(100000000, P_GPLL0_OUT_MAIN_DIV, 3, 0, 0),
	F(120000000, P_GPLL0_OUT_MAIN_DIV, 2.5, 0, 0),
	F(150000000, P_GPLL0_OUT_MAIN_DIV, 2, 0, 0),
	F(200000000, P_GPLL0_OUT_MAIN, 3, 0, 0),
	F(300000000, P_GPLL0_OUT_MAIN, 2, 0, 0),
	F(400000000, P_GPLL0_OUT_MAIN, 1.5, 0, 0),
	{ }
};

static struct clk_rcg2 gpu_cc_hub_clk_src = {
	.cmd_rcgr = 0x93f0,
	.mnd_width = 0,
	.hid_width = 5,
	.parent_map = gpu_cc_parent_map_1,
	.freq_tbl = ftbl_gpu_cc_hub_clk_src,
	.enable_safe_config = true,
	.flags = HW_CLK_CTRL_MODE,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "gpu_cc_hub_clk_src",
		.parent_data = gpu_cc_parent_data_1,
		.num_parents = ARRAY_SIZE(gpu_cc_parent_data_1),
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_rcg2_ops,
	},
	.clkr.vdd_data = {
		.vdd_class = &vdd_cx,
		.num_rate_max = VDD_NUM,
		.rate_max = (unsigned long[VDD_NUM]) {
			[VDD_LOWER_D1] = 150000000,
			[VDD_LOWER] = 200000000,
			[VDD_LOW] = 300000000,
			[VDD_LOW_L1] = 400000000},
	},
};

static struct clk_regmap_div gpu_cc_hub_div_clk_src = {
	.reg = 0x9430,
	.shift = 0,
	.width = 4,
	.clkr.hw.init = &(const struct clk_init_data) {
		.name = "gpu_cc_hub_div_clk_src",
		.parent_hws = (const struct clk_hw*[]) {
			&gpu_cc_hub_clk_src.clkr.hw,
		},
		.num_parents = 1,
		.flags = CLK_SET_RATE_PARENT,
		.ops = &clk_regmap_div_ro_ops,
	},
};

static struct clk_branch gpu_cc_ahb_clk = {
	.halt_reg = 0x90bc,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = 0x90bc,
		.enable_mask = BIT(0),
		.flags = QCOM_CLK_MINIDUMP_ENABLE,
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_ahb_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&gpu_cc_hub_div_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_cx_accu_shift_clk = {
	.halt_reg = 0x9104,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x9104,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_cx_accu_shift_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_cx_gmu_clk = {
	.halt_reg = 0x90d4,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x90d4,
		.enable_mask = BIT(0),
		.flags = QCOM_CLK_MINIDUMP_ENABLE,
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_cx_gmu_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&gpu_cc_gmu_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_DONT_HOLD_STATE | CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_aon_ops,
		},
	},
};

static struct clk_branch gpu_cc_cxo_clk = {
	.halt_reg = 0x90e4,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x90e4,
		.enable_mask = BIT(0),
		.flags = QCOM_CLK_MINIDUMP_ENABLE,
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_cxo_clk",
			.flags = CLK_DONT_HOLD_STATE,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_demet_clk = {
	.halt_reg = 0x9010,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x9010,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_demet_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_dpm_clk = {
	.halt_reg = 0x9108,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x9108,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_dpm_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_freq_measure_clk = {
	.halt_reg = 0x900c,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x900c,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_freq_measure_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_gpu_smmu_vote_clk = {
	.halt_reg = 0x7000,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x7000,
		.enable_mask = BIT(0),
		.flags = QCOM_CLK_MINIDUMP_ENABLE,
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_gpu_smmu_vote_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_gx_accu_shift_clk = {
	.halt_reg = 0x9070,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x9070,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_gx_accu_shift_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_gx_gmu_clk = {
	.halt_reg = 0x9060,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = 0x9060,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_gx_gmu_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&gpu_cc_gmu_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gpu_cc_hub_aon_clk = {
	.halt_reg = 0x93ec,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x93ec,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_hub_aon_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&gpu_cc_hub_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_aon_ops,
		},
	},
};

static struct clk_branch gpu_cc_hub_cx_int_clk = {
	.halt_reg = 0x90e8,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x90e8,
		.enable_mask = BIT(0),
		.flags = QCOM_CLK_MINIDUMP_ENABLE,
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_hub_cx_int_clk",
			.parent_hws = (const struct clk_hw*[]) {
				&gpu_cc_hub_clk_src.clkr.hw,
			},
			.num_parents = 1,
			.flags = CLK_DONT_HOLD_STATE | CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_aon_ops,
		},
	},
};

static struct clk_branch gpu_cc_memnoc_gfx_clk = {
	.halt_reg = 0x90ec,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x90ec,
		.enable_mask = BIT(0),
		.hw.init = &(const struct clk_init_data) {
			.name = "gpu_cc_memnoc_gfx_clk",
			.parent_data = &(const struct clk_parent_data) {
				.fw_name = "gcc_gpu_gemnoc_gfx_clk",
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct gdsc gpu_cc_cx_gdsc = {
	.gdscr = 0x9080,
	.gds_hw_ctrl = 0x9094,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0x8,
	.pd = {
		.name = "gpu_cc_cx_gdsc",
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = RETAIN_FF_ENABLE | VOTABLE,
	.supply = "vdd_cx",
};

static int gdsc_cx_do_nothing(struct generic_pm_domain *domain)
{
	return 0;
}

static struct gdsc gpu_cc_cx_smmu_gdsc = {
	.gdscr = 0x9080,
	.gds_hw_ctrl = 0x9094,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0x8,
	.pd = {
		.name = "gpu_cc_cx_smmu_gdsc",
		.power_on = gdsc_cx_do_nothing,
		.power_off = gdsc_cx_do_nothing,
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = RETAIN_FF_ENABLE | VOTABLE,
	.parent = &gpu_cc_cx_gdsc.pd,
};

static struct gdsc gpu_cc_cx_gmu_gdsc = {
	.gdscr = 0x9080,
	.gds_hw_ctrl = 0x9094,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0x8,
	.pd = {
		.name = "gpu_cc_cx_gmu_gdsc",
		.power_on = gdsc_cx_do_nothing,
		.power_off = gdsc_cx_do_nothing,
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = RETAIN_FF_ENABLE | VOTABLE,
	.parent = &gpu_cc_cx_gdsc.pd,
};

static struct gdsc gx_clkctl_gx_gdsc = {
	.gdscr = 0x0,
	.en_rest_wait_val = 0x2,
	.en_few_wait_val = 0x2,
	.clk_dis_wait_val = 0xf,
	.pd = {
		.name = "gx_clkctl_gx_gdsc",
		.power_on = gdsc_gx_do_nothing_enable,
	},
	.pwrsts = PWRSTS_OFF_ON,
	.flags = POLL_CFG_GDSCR | RETAIN_FF_ENABLE,
	.supply = "vdd_gfx",
};

static struct clk_regmap *gpu_cc_canoe_clocks[] = {
	[GPU_CC_AHB_CLK] = &gpu_cc_ahb_clk.clkr,
	[GPU_CC_CX_ACCU_SHIFT_CLK] = &gpu_cc_cx_accu_shift_clk.clkr,
	[GPU_CC_CX_GMU_CLK] = &gpu_cc_cx_gmu_clk.clkr,
	[GPU_CC_CXO_CLK] = &gpu_cc_cxo_clk.clkr,
	[GPU_CC_DEMET_CLK] = &gpu_cc_demet_clk.clkr,
	[GPU_CC_DPM_CLK] = &gpu_cc_dpm_clk.clkr,
	[GPU_CC_FREQ_MEASURE_CLK] = &gpu_cc_freq_measure_clk.clkr,
	[GPU_CC_GMU_CLK_SRC] = &gpu_cc_gmu_clk_src.clkr,
	[GPU_CC_GPU_SMMU_VOTE_CLK] = &gpu_cc_gpu_smmu_vote_clk.clkr,
	[GPU_CC_GX_ACCU_SHIFT_CLK] = &gpu_cc_gx_accu_shift_clk.clkr,
	[GPU_CC_GX_GMU_CLK] = &gpu_cc_gx_gmu_clk.clkr,
	[GPU_CC_HUB_AON_CLK] = &gpu_cc_hub_aon_clk.clkr,
	[GPU_CC_HUB_CLK_SRC] = &gpu_cc_hub_clk_src.clkr,
	[GPU_CC_HUB_CX_INT_CLK] = &gpu_cc_hub_cx_int_clk.clkr,
	[GPU_CC_HUB_DIV_CLK_SRC] = &gpu_cc_hub_div_clk_src.clkr,
	[GPU_CC_MEMNOC_GFX_CLK] = &gpu_cc_memnoc_gfx_clk.clkr,
	[GPU_CC_PLL0] = &gpu_cc_pll0.clkr,
	[GPU_CC_PLL0_OUT_EVEN] = &gpu_cc_pll0_out_even.clkr,
};

static struct gdsc *gpu_cc_canoe_gdscs[] = {
	[GPU_CC_CX_GDSC] = &gpu_cc_cx_gdsc,
	[GPU_CC_CX_SMMU_GDSC] = &gpu_cc_cx_smmu_gdsc,
	[GPU_CC_CX_GMU_GDSC] = &gpu_cc_cx_gmu_gdsc,
};

static struct gdsc *gx_clkctl_gdscs[] = {
	[GX_CLKCTL_GX_GDSC] = &gx_clkctl_gx_gdsc,
};

static const struct qcom_reset_map gpu_cc_canoe_resets[] = {
	[GPU_CC_CB_BCR] = { 0x93a0 },
	[GPU_CC_CX_BCR] = { 0x907c },
	[GPU_CC_FAST_HUB_BCR] = { 0x93e4 },
	[GPU_CC_FF_BCR] = { 0x9470 },
	[GPU_CC_GMU_BCR] = { 0x9314 },
	[GPU_CC_GX_BCR] = { 0x905c },
	[GPU_CC_XO_BCR] = { 0x9000 },
};

static const struct regmap_config gpu_cc_canoe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x95e8,
	.fast_io = true,
};

static const struct regmap_config gx_clkctl_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x8,
	.fast_io = true,
};

static const struct qcom_cc_desc gpu_cc_canoe_desc = {
	.config = &gpu_cc_canoe_regmap_config,
	.clks = gpu_cc_canoe_clocks,
	.num_clks = ARRAY_SIZE(gpu_cc_canoe_clocks),
	.resets = gpu_cc_canoe_resets,
	.num_resets = ARRAY_SIZE(gpu_cc_canoe_resets),
	.clk_regulators = gpu_cc_canoe_regulators,
	.num_clk_regulators = ARRAY_SIZE(gpu_cc_canoe_regulators),
	.gdscs = gpu_cc_canoe_gdscs,
	.num_gdscs = ARRAY_SIZE(gpu_cc_canoe_gdscs),
};

static const struct qcom_cc_desc gx_clkctl_canoe_desc = {
	.config = &gx_clkctl_regmap_config,
	.gdscs = gx_clkctl_gdscs,
	.num_gdscs = ARRAY_SIZE(gx_clkctl_gdscs),
};

static const struct of_device_id gpu_cc_canoe_match_table[] = {
	{ .compatible = "qcom,canoe-gpucc" },
	{ .compatible = "qcom,alor-gpucc" },
	{ .compatible = "qcom,whale-gpucc" },
	{ }
};
MODULE_DEVICE_TABLE(of, gpu_cc_canoe_match_table);

static void gpu_cc_alor_fixup(struct regmap *regmap)
{
	gpu_cc_pll0_config.config_ctl_hi_val = 0x0a8060e0;

	gpu_cc_cx_accu_shift_clk.halt_reg = 0x9108;
	gpu_cc_cx_accu_shift_clk.clkr.enable_reg = 0x9108;
	gpu_cc_dpm_clk.halt_reg = 0x910c;
	gpu_cc_dpm_clk.clkr.enable_reg = 0x910c;
	gpu_cc_memnoc_gfx_clk.halt_reg = 0x90f0;
	gpu_cc_memnoc_gfx_clk.clkr.enable_reg = 0x90f0;
}

static int gpu_cc_canoe_fixup(struct platform_device *pdev, struct regmap *regmap)
{
	const char *compat = NULL;
	int compatlen = 0;

	compat = of_get_property(pdev->dev.of_node, "compatible", &compatlen);
	if (!compat || compatlen <= 0)
		return -EINVAL;

	if (!strcmp(compat, "qcom,alor-gpucc"))
		gpu_cc_alor_fixup(regmap);

	if (!strcmp(compat, "qcom,whale-gpucc"))
		gpu_cc_pll0_config.config_ctl_hi_val = 0x0a8060e0;

	return 0;
}

static int gpu_cc_canoe_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int ret;

	regmap = qcom_cc_map(pdev, &gpu_cc_canoe_desc);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = gpu_cc_canoe_fixup(pdev, regmap);
	if (ret)
		return ret;

	clk_taycan_eko_t_pll_configure(&gpu_cc_pll0, regmap, &gpu_cc_pll0_config);

	/*
	 * Keep clocks always enabled:
	 *	gpu_cc_cb_clk
	 *	gpu_cc_cxo_aon_clk
	 *	gpu_cc_rscc_hub_aon_clk
	 *	gpu_cc_rscc_xo_aon_clk
	 *	gpu_cc_sleep_clk
	 */
	regmap_update_bits(regmap, 0x93a4, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x9008, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x93e8, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x9004, BIT(0), BIT(0));
	regmap_update_bits(regmap, 0x90cc, BIT(0), BIT(0));

	ret = qcom_cc_really_probe(&pdev->dev, &gpu_cc_canoe_desc, regmap);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register GPU CC clocks\n");

	dev_info(&pdev->dev, "Registered GPU CC clocks\n");

	return ret;
}

static void gpu_cc_canoe_sync_state(struct device *dev)
{
	qcom_cc_sync_state(dev, &gpu_cc_canoe_desc);
}

static struct platform_driver gpu_cc_canoe_driver = {
	.probe = gpu_cc_canoe_probe,
	.driver = {
		.name = "gpucc-canoe",
		.of_match_table = gpu_cc_canoe_match_table,
		.sync_state = gpu_cc_canoe_sync_state,
	},
};

static const struct of_device_id gx_clkctl_canoe_match_table[] = {
	{ .compatible = "qcom,canoe-gx_clkctl" },
	{ }
};
MODULE_DEVICE_TABLE(of, gx_clkctl_canoe_match_table);

static int gx_clkctl_canoe_probe(struct platform_device *pdev)
{

	struct regmap *regmap;
	int ret;

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret)
		return ret;

	regmap = qcom_cc_map(pdev, &gx_clkctl_canoe_desc);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		goto err;
	}

	ret = qcom_cc_really_probe(&pdev->dev, &gx_clkctl_canoe_desc, regmap);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "Failed to register GX CLKCTL\n");
		goto err;
	}

	dev_info(&pdev->dev, "Registered GX CLKCTL clocks\n");

err:
	pm_runtime_put(&pdev->dev);

	return ret;
}

static struct platform_driver gx_clkctl_canoe_driver = {
	.probe = gx_clkctl_canoe_probe,
	.driver = {
		.name = "gx_clkctl-canoe",
		.of_match_table = gx_clkctl_canoe_match_table,
	},
};

static int __init gpu_cc_canoe_init(void)
{
	int ret;

	ret = platform_driver_register(&gpu_cc_canoe_driver);
	if (ret)
		return ret;

	return platform_driver_register(&gx_clkctl_canoe_driver);
}
subsys_initcall(gpu_cc_canoe_init);

static void __exit gpu_cc_canoe_exit(void)
{
	platform_driver_unregister(&gx_clkctl_canoe_driver);
	platform_driver_unregister(&gpu_cc_canoe_driver);
}
module_exit(gpu_cc_canoe_exit);

MODULE_DESCRIPTION("QTI GPUCC CANOE Driver");
MODULE_LICENSE("GPL");
