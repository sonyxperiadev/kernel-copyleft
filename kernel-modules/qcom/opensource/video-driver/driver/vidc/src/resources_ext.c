// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/clk/qcom.h>

#ifdef CONFIG_MSM_MMRM
#include <linux/soc/qcom/msm_mmrm.h>
#endif

#include "resources.h"
#include "msm_vidc_core.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_power.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_platform.h"

static void __fatal_error(bool fatal)
{
	WARN_ON(fatal);
}

static int __init_regulators(struct msm_vidc_core *core)
{
	const struct regulator_table *regulator_tbl;
	struct regulator_set *regulators;
	struct regulator_info *rinfo = NULL;
	u32 regulator_count = 0, cnt = 0;
	int rc = 0;

	regulators = &core->resource->regulator_set;

	regulator_tbl = core->platform->data.regulator_tbl;
	regulator_count = core->platform->data.regulator_tbl_size;

	/* skip init if regulators not supported */
	if (!regulator_count) {
		d_vpr_h("%s: regulators are not available in database\n", __func__);
		return 0;
	}

	/* sanitize regulator table */
	if (!regulator_tbl) {
		d_vpr_e("%s: invalid regulator tbl\n", __func__);
		return -EINVAL;
	}

	/* allocate regulator_set */
	regulators->regulator_tbl = devm_kzalloc(&core->pdev->dev,
			sizeof(*regulators->regulator_tbl) * regulator_count, GFP_KERNEL);
	if (!regulators->regulator_tbl) {
		d_vpr_e("%s: failed to alloc memory for regulator table\n", __func__);
		return -ENOMEM;
	}
	regulators->count = regulator_count;

	/* populate regulator fields */
	for (cnt = 0; cnt < regulators->count; cnt++) {
		regulators->regulator_tbl[cnt].name = regulator_tbl[cnt].name;
		regulators->regulator_tbl[cnt].hw_power_collapse = regulator_tbl[cnt].hw_trigger;
	}

	/* print regulator fields */
	venus_hfi_for_each_regulator(core, rinfo) {
		d_vpr_h("%s: name %s hw_power_collapse %d\n",
			__func__, rinfo->name, rinfo->hw_power_collapse);
	}

	/* get regulator handle */
	venus_hfi_for_each_regulator(core, rinfo) {
		rinfo->regulator = devm_regulator_get(&core->pdev->dev, rinfo->name);
		if (IS_ERR_OR_NULL(rinfo->regulator)) {
			rc = PTR_ERR(rinfo->regulator) ?
				PTR_ERR(rinfo->regulator) : -EBADHANDLE;
			d_vpr_e("%s: failed to get regulator: %s\n", __func__, rinfo->name);
			rinfo->regulator = NULL;
			return rc;
		}
	}

	return rc;
}

static int __acquire_regulator(struct msm_vidc_core *core,
			       struct regulator_info *rinfo)
{
	int rc = 0;

	rc = call_res_op(core, reset_control_acquire, core, "video_xo_reset");
	if (rc) {
		d_vpr_e("%s: failed to acquire video_xo_reset control\n", __func__);
		goto fail_assert_xo_reset;
	}

	if (rinfo->hw_power_collapse) {
		if (!rinfo->regulator) {
			d_vpr_e("%s: invalid regulator\n", __func__);
			rc = -EINVAL;
			goto exit;
		}

		if (regulator_get_mode(rinfo->regulator) ==
				REGULATOR_MODE_NORMAL) {
			/* clear handoff from core sub_state */
			msm_vidc_change_core_sub_state(core,
				CORE_SUBSTATE_GDSC_HANDOFF, 0, __func__);
			d_vpr_h("Skip acquire regulator %s\n", rinfo->name);
			goto exit;
		}

		rc = regulator_set_mode(rinfo->regulator,
				REGULATOR_MODE_NORMAL);
		if (rc) {
			/*
			 * This is somewhat fatal, but nothing we can do
			 * about it. We can't disable the regulator w/o
			 * getting it back under s/w control
			 */
			d_vpr_e("Failed to acquire regulator control: %s\n",
				rinfo->name);
			goto exit;
		} else {
			/* reset handoff from core sub_state */
			msm_vidc_change_core_sub_state(core,
				CORE_SUBSTATE_GDSC_HANDOFF, 0, __func__);
			d_vpr_h("Acquired regulator control from HW: %s\n",
					rinfo->name);

		}

		if (!regulator_is_enabled(rinfo->regulator)) {
			d_vpr_e("%s: Regulator is not enabled %s\n",
				__func__, rinfo->name);
			__fatal_error(true);
		}
	}

exit:
	rc = call_res_op(core, reset_control_release, core, "video_xo_reset");
	if (rc)
		d_vpr_e("%s: failed to release video_xo_reset reset\n", __func__);
fail_assert_xo_reset:
	return rc;
}

static int __hand_off_regulator(struct msm_vidc_core *core,
	struct regulator_info *rinfo)
{
	int rc = 0;

	rc = call_res_op(core, reset_control_acquire, core, "video_xo_reset");
	if (rc) {
		d_vpr_e("%s: failed to acquire video_xo_reset control\n", __func__);
		goto fail_assert_xo_reset;
	}

	if (rinfo->hw_power_collapse) {
		if (!rinfo->regulator) {
			d_vpr_e("%s: invalid regulator\n", __func__);
			goto exit;
		}

		rc = regulator_set_mode(rinfo->regulator,
				REGULATOR_MODE_FAST);
		if (rc) {
			d_vpr_e("Failed to hand off regulator control: %s\n",
				rinfo->name);
			goto exit;
		} else {
			/* set handoff done in core sub_state */
			msm_vidc_change_core_sub_state(core,
				0, CORE_SUBSTATE_GDSC_HANDOFF, __func__);
			d_vpr_h("Hand off regulator control to HW: %s\n",
					rinfo->name);
		}

		if (!regulator_is_enabled(rinfo->regulator)) {
			d_vpr_e("%s: Regulator is not enabled %s\n",
				__func__, rinfo->name);
			__fatal_error(true);
		}
	}

exit:
	rc = call_res_op(core, reset_control_release, core, "video_xo_reset");
	if (rc)
		d_vpr_e("%s: failed to release video_xo_reset reset\n", __func__);
fail_assert_xo_reset:
	return rc;
}

static int __enable_regulator(struct msm_vidc_core *core, const char *reg_name)
{
	int rc = 0;
	struct regulator_info *rinfo;
	bool found;

	found = false;
	venus_hfi_for_each_regulator(core, rinfo) {
		if (!rinfo->regulator) {
			d_vpr_e("%s: invalid regulator %s\n",
				__func__, rinfo->name);
			return -EINVAL;
		}
		if (strcmp(rinfo->name, reg_name))
			continue;
		found = true;

		rc = call_res_op(core, reset_control_acquire, core, "video_xo_reset");
		if (rc) {
			d_vpr_e("%s: failed to acquire video_xo_reset control\n", __func__);
			goto fail_assert_xo_reset;
		}

		rc = regulator_enable(rinfo->regulator);
		if (rc) {
			d_vpr_e("%s: failed to enable %s, rc = %d\n",
				__func__, rinfo->name, rc);
			goto fail_regulator_enable;
		}
		if (!regulator_is_enabled(rinfo->regulator)) {
			d_vpr_e("%s: regulator %s not enabled\n",
				__func__, rinfo->name);
			regulator_disable(rinfo->regulator);
			rc = -EINVAL;
			goto fail_regulator_enable;
		}

		rc = call_res_op(core, reset_control_release, core, "video_xo_reset");
		if (rc)
			d_vpr_e("%s: failed to release video_xo_reset reset\n", __func__);

		d_vpr_h("%s: enabled regulator %s\n", __func__, rinfo->name);
		break;
	}
	if (!found) {
		d_vpr_e("%s: regulator %s not found\n", __func__, reg_name);
		return -EINVAL;
	}

	return rc;

fail_regulator_enable:
	call_res_op(core, reset_control_release, core, "video_xo_reset");
fail_assert_xo_reset:
	return rc;
}

static int __disable_regulator(struct msm_vidc_core *core, const char *reg_name)
{
	int rc = 0;
	struct regulator_info *rinfo;
	bool found;

	found = false;
	venus_hfi_for_each_regulator(core, rinfo) {
		if (!rinfo->regulator) {
			d_vpr_e("%s: invalid regulator %s\n",
				__func__, rinfo->name);
			return -EINVAL;
		}
		if (strcmp(rinfo->name, reg_name))
			continue;
		found = true;

		rc = __acquire_regulator(core, rinfo);
		if (rc) {
			d_vpr_e("%s: failed to acquire %s, rc = %d\n",
				__func__, rinfo->name, rc);
			/* Bring attention to this issue */
			WARN_ON(true);
			return rc;
		}
		/* reset handoff done from core sub_state */
		msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_GDSC_HANDOFF, 0, __func__);

		rc = call_res_op(core, reset_control_acquire, core, "video_xo_reset");
		if (rc) {
			d_vpr_e("%s: failed to acquire video_xo_reset control\n", __func__);
			goto fail_assert_xo_reset;
		}

		rc = regulator_disable(rinfo->regulator);
		if (rc) {
			d_vpr_e("%s: failed to disable %s, rc = %d\n",
				__func__, rinfo->name, rc);
			goto fail_regulator_disable;
		}

		rc = call_res_op(core, reset_control_release, core, "video_xo_reset");
		if (rc)
			d_vpr_e("%s: failed to release video_xo_reset reset\n", __func__);

		d_vpr_h("%s: disabled regulator %s\n", __func__, rinfo->name);
		break;
	}
	if (!found) {
		d_vpr_e("%s: regulator %s not found\n", __func__, reg_name);
		return -EINVAL;
	}

	return rc;

fail_regulator_disable:
	call_res_op(core, reset_control_release, core, "video_xo_reset");
fail_assert_xo_reset:
	return rc;
}

static int __hand_off_regulators(struct msm_vidc_core *core)
{
	struct regulator_info *rinfo;
	int rc = 0, c = 0;

	venus_hfi_for_each_regulator(core, rinfo) {
		rc = __hand_off_regulator(core, rinfo);
		/*
		 * If one regulator hand off failed, driver should take
		 * the control for other regulators back.
		 */
		if (rc)
			goto err_reg_handoff_failed;
		c++;
	}

	return rc;
err_reg_handoff_failed:
	venus_hfi_for_each_regulator_reverse_continue(core, rinfo, c)
		__acquire_regulator(core, rinfo);

	return rc;
}

static int __acquire_regulators(struct msm_vidc_core *core)
{
	int rc = 0;
	struct regulator_info *rinfo;

	venus_hfi_for_each_regulator(core, rinfo)
		__acquire_regulator(core, rinfo);

	return rc;
}

#ifdef CONFIG_MSM_MMRM
static int __set_clk_rate(struct msm_vidc_core *core, struct clock_info *cl,
			  u64 rate)
{
	int rc = 0;
	struct mmrm_client_data client_data;
	struct mmrm_client *client;
	u64 srate;

	if (is_mmrm_supported(core) && !cl->mmrm_client) {
		d_vpr_e("%s: invalid mmrm client\n", __func__);
		return -EINVAL;
	}

	/* update clock residency stats */
	call_res_op(core, clk_update_residency_stats, core, cl, rate);

	/*
	 * This conversion is necessary since we are scaling clock values based on
	 * the branch clock. However, mmrm driver expects source clock to be registered
	 * and used for scaling.
	 * TODO: Remove this scaling if using source clock instead of branch clock.
	 */
	srate = rate * MSM_VIDC_CLOCK_SOURCE_SCALING_RATIO;

	/* bail early if requested clk rate is not changed */
	if (rate == cl->prev)
		return 0;

	d_vpr_p("Scaling clock %s to %llu, prev %llu\n",
		cl->name, srate, cl->prev * MSM_VIDC_CLOCK_SOURCE_SCALING_RATIO);

	if (is_mmrm_supported(core)) {
		/* set clock rate to mmrm driver */
		client = cl->mmrm_client;
		memset(&client_data, 0, sizeof(client_data));
		client_data.num_hw_blocks = 1;
		rc = mmrm_client_set_value(client, &client_data, srate);
		if (rc) {
			d_vpr_e("%s: Failed to set mmrm clock rate %llu %s: %d\n",
				__func__, srate, cl->name, rc);
			return rc;
		}
	} else {
		/* set clock rate to clock driver */
		rc = clk_set_rate(cl->clk, srate);
		if (rc) {
			d_vpr_e("%s: Failed to set clock rate %llu %s: %d\n",
				__func__, srate, cl->name, rc);
			return rc;
		}
	}
	cl->prev = rate;
	return rc;
}
#else
static int __set_clk_rate(struct msm_vidc_core *core, struct clock_info *cl,
			  u64 rate)
{
	u64 srate;
	int rc = 0;

	/* update clock residency stats */
	call_res_op(core, clk_update_residency_stats, core, cl, rate);

	/*
	 * This conversion is necessary since we are scaling clock values based on
	 * the branch clock. However, mmrm driver expects source clock to be registered
	 * and used for scaling.
	 * TODO: Remove this scaling if using source clock instead of branch clock.
	 */
	srate = rate * MSM_VIDC_CLOCK_SOURCE_SCALING_RATIO;

	/* bail early if requested clk rate is not changed */
	if (rate == cl->prev)
		return 0;

	d_vpr_p("Scaling clock %s to %llu, prev %llu\n",
		cl->name, srate, cl->prev * MSM_VIDC_CLOCK_SOURCE_SCALING_RATIO);

	rc = clk_set_rate(cl->clk, srate);
	if (rc) {
		d_vpr_e("%s: Failed to set clock rate %llu %s: %d\n",
			__func__, srate, cl->name, rc);
		return rc;
	}

	cl->prev = rate;

	return rc;
}
#endif

static int __set_clocks_ext(struct msm_vidc_core *core, u64 freq)
{
	int rc = 0;
	struct clock_info *cl;

	venus_hfi_for_each_clock(core, cl) {
		if (cl->has_scaling) {
			rc = __set_clk_rate(core, cl, freq);
			if (rc)
				return rc;
		}
	}

	return 0;
}

static int qcom_clk_get_branch_flag(enum msm_vidc_branch_mem_flags vidc_flag,
	enum branch_mem_flags *clk_flag)
{
	switch (vidc_flag) {
	case MSM_VIDC_CLKFLAG_RETAIN_PERIPH:
		*clk_flag = CLKFLAG_RETAIN_PERIPH;
		break;
	case MSM_VIDC_CLKFLAG_NORETAIN_PERIPH:
		*clk_flag = CLKFLAG_NORETAIN_PERIPH;
		break;
	case MSM_VIDC_CLKFLAG_RETAIN_MEM:
		*clk_flag = CLKFLAG_RETAIN_MEM;
		break;
	case MSM_VIDC_CLKFLAG_NORETAIN_MEM:
		*clk_flag = CLKFLAG_NORETAIN_MEM;
		break;
	case MSM_VIDC_CLKFLAG_PERIPH_OFF_SET:
		*clk_flag = CLKFLAG_PERIPH_OFF_SET;
		break;
	case MSM_VIDC_CLKFLAG_PERIPH_OFF_CLEAR:
		*clk_flag = CLKFLAG_PERIPH_OFF_CLEAR;
		break;
	default:
		d_vpr_e("%s: invalid clk flag: %d\n", __func__, vidc_flag);
		return -EINVAL;
	}
	return 0;
}

static int __clock_set_flag_ext(struct msm_vidc_core *core,
	const char *name, enum msm_vidc_branch_mem_flags flag)
{
	int rc = 0;
	struct clock_info *cinfo = NULL;
	bool found = false;
	enum branch_mem_flags mem_flag;

	/* get clock handle */
	venus_hfi_for_each_clock(core, cinfo) {
		if (strcmp(cinfo->name, name))
			continue;
		found = true;
		rc = qcom_clk_get_branch_flag(flag, &mem_flag);
		if (rc)
			return rc;

		qcom_clk_set_flags(cinfo->clk, mem_flag);
		d_vpr_h("%s: set flag %d on clock %s\n", __func__, mem_flag, name);
		break;
	}
	if (!found) {
		d_vpr_e("%s: failed to find clock: %s\n", __func__, name);
		return -EINVAL;
	}
	return 0;
}

const struct msm_vidc_resources_ops *get_res_ops_ext(void)
{
	const struct msm_vidc_resources_ops *res_ops = get_resources_ops();
	static struct msm_vidc_resources_ops res_ops_ext;

	memcpy(&res_ops_ext, res_ops, sizeof(struct msm_vidc_resources_ops));
	res_ops_ext.gdsc_init        = __init_regulators;
	res_ops_ext.gdsc_on          = __enable_regulator;
	res_ops_ext.gdsc_off         = __disable_regulator;
	res_ops_ext.gdsc_hw_ctrl     = __hand_off_regulators;
	res_ops_ext.gdsc_sw_ctrl     = __acquire_regulators;
	res_ops_ext.set_clks         = __set_clocks_ext;
	res_ops_ext.clk_set_flag     = __clock_set_flag_ext;

	return &res_ops_ext;
}
