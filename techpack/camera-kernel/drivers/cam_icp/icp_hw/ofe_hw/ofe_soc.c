// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/cam_defs.h>
#include <media/cam_icp.h>
#include "ofe_soc.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"

static int cam_ofe_get_dt_properties(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0)
		CAM_ERR(CAM_ICP, "get ofe dt prop is failed");

	return rc;
}

static int cam_ofe_request_platform_resource(
	struct cam_hw_soc_info *soc_info,
	irq_handler_t ofe_irq_handler, void *data)
{
	int rc = 0, i;
	void *irq_data[CAM_SOC_MAX_IRQ_LINES_PER_DEV] = {0};

	for (i = 0; i < soc_info->irq_count; i++)
		irq_data[i] = data;

	rc = cam_soc_util_request_platform_resource(soc_info,
		ofe_irq_handler, &(irq_data[0]));

	return rc;
}

int cam_ofe_init_soc_resources(struct cam_hw_soc_info *soc_info,
	irq_handler_t ofe_irq_handler, void *irq_data)
{
	int rc = 0;

	rc = cam_ofe_get_dt_properties(soc_info);
	if (rc < 0)
		return rc;

	rc = cam_ofe_request_platform_resource(soc_info,
		ofe_irq_handler, irq_data);
	if (rc < 0)
		return rc;

	return rc;
}

void cam_ofe_deinit_soc_resources(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_release_platform_resource(soc_info);
	if (rc)
		CAM_WARN(CAM_ICP, "release platform resources fail");
}

int cam_ofe_enable_soc_resources(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_enable_platform_resource(soc_info, CAM_CLK_SW_CLIENT_IDX,
		true, CAM_SVS_VOTE, false);
	if (rc)
		CAM_ERR(CAM_ICP, "enable platform failed");

	return rc;
}

int cam_ofe_disable_soc_resources(struct cam_hw_soc_info *soc_info,
	bool disable_clk)
{
	int rc = 0;

	rc = cam_soc_util_disable_platform_resource(soc_info, CAM_CLK_SW_CLIENT_IDX,
		disable_clk, false);
	if (rc)
		CAM_ERR(CAM_ICP, "disable platform failed");

	return rc;
}

static inline int cam_ofe_set_regulators_mode(struct cam_hw_soc_info *soc_info, uint32_t mode)
{
	int i, rc;

	for (i = 0; i < soc_info->num_rgltr; i++) {
		rc = cam_wrapper_regulator_set_mode(soc_info->rgltr[i], mode);
		if (rc) {
			CAM_ERR(CAM_ICP, "Regulator set mode %s failed",
				soc_info->rgltr_name[i]);
			goto rgltr_set_mode_failed;
		}
	}
	return 0;

rgltr_set_mode_failed:
	for (i = i - 1; i >= 0; i--) {
		if (soc_info->rgltr[i])
			cam_wrapper_regulator_set_mode(soc_info->rgltr[i],
				(mode == REGULATOR_MODE_NORMAL ?
				REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL));
	}

	return rc;
}

int cam_ofe_transfer_gdsc_control(struct cam_hw_soc_info *soc_info)
{
	return cam_ofe_set_regulators_mode(soc_info, REGULATOR_MODE_FAST);
}

int cam_ofe_get_gdsc_control(struct cam_hw_soc_info *soc_info)
{
	return cam_ofe_set_regulators_mode(soc_info, REGULATOR_MODE_NORMAL);
}

int cam_ofe_update_clk_rate(struct cam_hw_soc_info *soc_info,
	uint32_t clk_rate)
{
	int32_t src_clk_idx;

	if (!soc_info)
		return -EINVAL;

	src_clk_idx = soc_info->src_clk_idx;

	if ((soc_info->clk_level_valid[CAM_TURBO_VOTE] == true) &&
		(soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx] != 0) &&
		(clk_rate > soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx])) {
		CAM_DBG(CAM_PERF, "clk_rate %d greater than max, reset to %d",
			clk_rate,
			soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx]);
		clk_rate = soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx];
	}

	return cam_soc_util_set_src_clk_rate(soc_info, CAM_CLK_SW_CLIENT_IDX, clk_rate, 0);
}

int cam_ofe_toggle_clk(struct cam_hw_soc_info *soc_info, bool clk_enable)
{
	int rc = 0;

	if (clk_enable)
		rc = cam_soc_util_clk_enable_default(soc_info, CAM_CLK_SW_CLIENT_IDX, CAM_SVS_VOTE);
	else
		cam_soc_util_clk_disable_default(soc_info, CAM_CLK_SW_CLIENT_IDX);

	CAM_DBG(CAM_ICP, "%s OFE clock", clk_enable ? "Enable" : "Disable");

	return rc;
}
