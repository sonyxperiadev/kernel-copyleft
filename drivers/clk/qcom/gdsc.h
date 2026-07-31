/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015, 2017-2018, 2022, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __QCOM_GDSC_H__
#define __QCOM_GDSC_H__

#include <linux/err.h>
#include <linux/interconnect.h>
#include <linux/pm_domain.h>

struct regmap;
struct regulator;
struct reset_controller_dev;

/**
 * struct gdsc - Globally Distributed Switch Controller
 * @pd: generic power domain
 * @regmap: regmap for MMIO accesses
 * @gdscr: gsdc control register
 * @collapse_ctrl: APCS collapse-vote register
 * @collapse_mask: APCS collapse-vote mask
 * @gds_hw_ctrl: gds_hw_ctrl register
 * @cxcs: offsets of branch registers to toggle mem/periph bits in
 * @cxc_count: number of @cxcs
 * @pwrsts: Possible powerdomain power states
 * @en_rest_wait_val: transition delay value for receiving enr ack signal
 * @en_few_wait_val: transition delay value for receiving enf ack signal
 * @clk_dis_wait_val: transition delay value for halting clock
 * @resets: ids of resets associated with this gdsc
 * @reset_count: number of @resets
 * @rcdev: reset controller
 */
struct gdsc {
	struct generic_pm_domain	pd;
	struct generic_pm_domain	*parent;
	struct regmap			*regmap;
	unsigned int			gdscr;
	unsigned int			collapse_ctrl;
	unsigned int			collapse_mask;
	unsigned int			gds_hw_ctrl;
	unsigned int			clamp_io_ctrl;
	unsigned int			*cxcs;
	unsigned int			cxc_count;
	unsigned int			en_rest_wait_val;
	unsigned int			en_few_wait_val;
	unsigned int			clk_dis_wait_val;
	const u8			pwrsts;
/* Powerdomain allowable state bitfields */
#define PWRSTS_OFF		BIT(0)
/*
 * There is no SW control to transition a GDSC into
 * PWRSTS_RET. This happens in HW when the parent
 * domain goes down to a low power state
 */
#define PWRSTS_RET		BIT(1)
#define PWRSTS_ON		BIT(2)
#define PWRSTS_OFF_ON		(PWRSTS_OFF | PWRSTS_ON)
#define PWRSTS_RET_ON		(PWRSTS_RET | PWRSTS_ON)
	const u16			flags;
#define VOTABLE		BIT(0)
#define CLAMP_IO	BIT(1)
#define HW_CTRL		BIT(2)
#define SW_RESET	BIT(3)
#define AON_RESET	BIT(4)
#define POLL_CFG_GDSCR	BIT(5)
#define ALWAYS_ON	BIT(6)
#define RETAIN_FF_ENABLE	BIT(7)
#define NO_RET_PERIPH	BIT(8)
#define HW_CTRL_TRIGGER	BIT(9)
#define SKIP_DIS	BIT(10)
	struct reset_controller_dev	*rcdev;
	unsigned int			*resets;
	unsigned int			reset_count;

	const char 			*supply;
	struct regulator		*rsupply;

	const char			*path_name;
	struct icc_path			*path;
};

struct gdsc_desc {
	struct device *dev;
	struct gdsc **scs;
	size_t num;
};

#ifdef CONFIG_QCOM_GDSC
int gdsc_register(struct gdsc_desc *desc, struct reset_controller_dev *,
		  struct regmap *);
void gdsc_unregister(struct gdsc_desc *desc);
int gdsc_gx_do_nothing_enable(struct generic_pm_domain *domain);

struct gdsc_register_data {
	char *name;
	u32 offset;
};

/**
 * gdsc_genpd_print_regs - Print GDSC register values for debugging
 * @sc: GDSC to print registers for
 *
 * Prints the values of key GDSC registers to help diagnose issues
 * when status polling timeouts occur.
 */

static inline void gdsc_genpd_print_regs(struct gdsc *sc)
{
	int i;
	u32 val;

	const struct gdsc_register_data data[] = {
		{"GDSCR", 0x0},
		{"CFG_GDSCR", 0x4},
		{"CFG2_GDSCR", 0x8},
	};

	for (i = 0; i < ARRAY_SIZE(data); i++) {
		regmap_read(sc->regmap, sc->gdscr + data[i].offset,
					&val);
		pr_info("%s: 0x%.8x\n", data[i].name, val);
	}

	if (sc->gds_hw_ctrl) {
		regmap_read(sc->regmap, sc->gds_hw_ctrl, &val);
		pr_info("GDS_HW_CTRL: 0x%.8x\n", val);
	}

	if (sc->collapse_ctrl) {
		regmap_read(sc->regmap, sc->collapse_ctrl, &val);
		pr_info("COLLAPSE_CTRL: 0x%.8x\n", val);
	}
}
#else
static inline int gdsc_register(struct gdsc_desc *desc,
				struct reset_controller_dev *rcdev,
				struct regmap *r)
{
	return -ENOSYS;
}

static inline void gdsc_unregister(struct gdsc_desc *desc) {};
static inline void gdsc_genpd_print_regs(struct gdsc *sc) {};
#endif /* CONFIG_QCOM_GDSC */
#endif /* __QCOM_GDSC_H__ */
