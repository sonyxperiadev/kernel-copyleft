// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/interrupt.h>
#include <linux/of.h>

#include "cam_debug_util.h"
#include "cam_soc_util.h"
#include "hfi_intf.h"
#include "cam_icp_soc_common.h"

static int __ubwc_config_get(struct device_node *np, char *name, uint32_t *cfg, bool *ubwc_listed)
{
	int nconfig, i, rc;

	nconfig = of_property_count_u32_elems(np, name);
	if (nconfig < 0) {
		CAM_DBG(CAM_ICP, "prop: %s is not listed", name);
		*ubwc_listed = false;
		return 0;
	}
	*ubwc_listed = true;

	if (nconfig > ICP_UBWC_CFG_MAX) {
		CAM_ERR(CAM_ICP, "Invalid number of UBWC configs[=%d] MAX UBWC=%d",
			nconfig, ICP_UBWC_CFG_MAX);
		return -EINVAL;
	}

	for (i = 0; i < nconfig; i++) {
		rc = of_property_read_u32_index(np, name, i, &cfg[i]);
		if (rc) {
			CAM_ERR(CAM_ICP,
				"Node %pOF has no valid %s prop at index=%d",
				np, name, i);
			return rc;
		}
	}

	return 0;
}

static int cam_icp_soc_ubwc_config_get(struct device_node *np,
	struct cam_icp_soc_info *icp_soc_info)
{
	struct cam_icp_ubwc_cfg *ubwc_cfg_ext;
	int rc;
	uint32_t dev_type;
	bool ubwc_listed, ubwc_fetch_found, ubwc_write_found;

	dev_type = icp_soc_info->dev_type;

	rc = __ubwc_config_get(np, "ubwc-cfg", icp_soc_info->uconfig.ubwc_cfg,
		&ubwc_listed);
	if (ubwc_listed) {
		if (dev_type == CAM_ICP_HW_ICP_V1)
			icp_soc_info->is_ubwc_cfg = true;
		return rc;
	}

	ubwc_cfg_ext = &icp_soc_info->uconfig.ubwc_cfg_ext;
	rc = __ubwc_config_get(np, "ubwc-ipe-fetch-cfg",
		ubwc_cfg_ext->ipe_fetch, &ubwc_fetch_found);
	if (rc)
		return rc;

	rc = __ubwc_config_get(np, "ubwc-ipe-write-cfg",
		ubwc_cfg_ext->ipe_write, &ubwc_write_found);
	if (rc)
		return rc;

	if (ubwc_fetch_found && ubwc_write_found)
		ubwc_cfg_ext->found_ubwc_cfg_mask |= BIT(CAM_ICP_DEV_IPE);
	else {
		if (ubwc_fetch_found ^ ubwc_write_found) {
			CAM_ERR(CAM_ICP, "Missing %s ipe ubwc config",
				ubwc_fetch_found ? "write" : "fetch");
			return -EINVAL;
		}
	}

	rc = __ubwc_config_get(np, "ubwc-bps-fetch-cfg",
		ubwc_cfg_ext->bps_fetch, &ubwc_fetch_found);
	if (rc)
		return rc;

	rc = __ubwc_config_get(np, "ubwc-bps-write-cfg",
		ubwc_cfg_ext->bps_write, &ubwc_write_found);
	if (rc)
		return rc;

	if (ubwc_fetch_found && ubwc_write_found)
		ubwc_cfg_ext->found_ubwc_cfg_mask |= BIT(CAM_ICP_DEV_BPS);
	else {
		if (ubwc_fetch_found ^ ubwc_write_found) {
			CAM_ERR(CAM_ICP, "Missing %s bps ubwc config",
				ubwc_fetch_found ? "write" : "fetch");
			return -EINVAL;
		}
	}

	rc = __ubwc_config_get(np, "ubwc-ofe-fetch-cfg",
		ubwc_cfg_ext->ofe_fetch, &ubwc_fetch_found);
	if (rc)
		return rc;

	rc = __ubwc_config_get(np, "ubwc-ofe-write-cfg",
		ubwc_cfg_ext->ofe_write, &ubwc_write_found);
	if (rc)
		return rc;

	if (ubwc_fetch_found && ubwc_write_found)
		ubwc_cfg_ext->found_ubwc_cfg_mask |= BIT(CAM_ICP_DEV_OFE);
	else {
		if (ubwc_fetch_found ^ ubwc_write_found) {
			CAM_ERR(CAM_ICP, "Missing %s ofe ubwc config",
				ubwc_fetch_found ? "write" : "fetch");
			return -EINVAL;
		}
	}

	return 0;
}

static inline void cam_icp_soc_qos_get(struct device_node *np,
	struct cam_icp_soc_info *icp_soc_info)
{
	if (of_property_read_u32(np, "qos-val", &icp_soc_info->qos_val)) {
		CAM_WARN(CAM_ICP, "QoS not set for device: %d",
			icp_soc_info->dev_type);
		icp_soc_info->qos_val = 0;
	}
}

static int cam_icp_soc_get_hw_version(struct device_node *np,
	struct cam_icp_soc_info *icp_soc_info)
{
	int rc;
	uint32_t version;

	rc = of_property_read_u32(np, "icp-version", &version);
	if (rc) {
		CAM_ERR(CAM_ICP, "read icp-version failed rc=%d", rc);
		return -ENODEV;
	}

	switch (version) {
	case CAM_ICP_V1_VERSION:
	case CAM_ICP_V2_VERSION:
	case CAM_ICP_V2_1_VERSION:
		icp_soc_info->hw_version = version;
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid ICP version: %u", version);
		rc = -ENODEV;
	}
	return rc;
}

static void cam_icp_soc_get_fw_pas_id(struct device_node *np,
	struct cam_icp_soc_info *icp_soc_info)
{
	if (of_property_read_u32(np, "fw-pas-id", &icp_soc_info->fw_pas_id)) {
		CAM_WARN(CAM_ICP, "PAS_ID is not passed from DTSI and use default value: %d",
			CAM_FW_PAS_ID_DEFAULT);
		icp_soc_info->fw_pas_id = CAM_FW_PAS_ID_DEFAULT;
	}
}

static int cam_icp_soc_dt_properties_get(struct cam_hw_soc_info *soc_info)
{
	struct cam_icp_soc_info *icp_soc_info;
	struct device_node *np;
	int rc;

	if (!soc_info->soc_private) {
		CAM_ERR(CAM_ICP, "soc private is NULL");
		return -EINVAL;
	}

	icp_soc_info = (struct cam_icp_soc_info *)soc_info->soc_private;
	np = soc_info->pdev->dev.of_node;

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to get DT properties rc=%d", rc);
		return rc;
	}

	rc = cam_icp_soc_ubwc_config_get(np, icp_soc_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to get UBWC config props rc=%d", rc);
		return rc;
	}

	cam_icp_soc_qos_get(np, icp_soc_info);

	cam_icp_soc_get_fw_pas_id(np, icp_soc_info);

	rc = cam_icp_soc_get_hw_version(np, icp_soc_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "Get ICP HW version failed");
		return rc;
	}

	return 0;
}

int cam_icp_soc_resources_init(struct cam_hw_soc_info *soc_info,
	irq_handler_t handler, void *data)
{
	int rc, i;
	void *irq_data[CAM_SOC_MAX_IRQ_LINES_PER_DEV] = {0};

	rc = cam_icp_soc_dt_properties_get(soc_info);
	if (rc)
		return rc;

	for (i = 0; i < soc_info->irq_count; i++)
		irq_data[i] = data;

	rc = cam_soc_util_request_platform_resource(soc_info, handler, &(irq_data[0]));
	if (rc) {
		CAM_ERR(CAM_ICP,
			"request for soc platform resource failed rc=%d", rc);
		return rc;
	}

	return 0;
}

int cam_icp_soc_resources_deinit(struct cam_hw_soc_info *soc_info)
{
	int rc;

	rc = cam_soc_util_release_platform_resource(soc_info);
	if (rc)
		CAM_ERR(CAM_ICP,
			"release of soc platform resource failed rc=%d", rc);

	return rc;
}

int cam_icp_soc_resources_enable(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_enable_platform_resource(soc_info, CAM_CLK_SW_CLIENT_IDX, true,
		soc_info->lowest_clk_level, true);
	if (rc)
		CAM_ERR(CAM_ICP, "failed to enable soc resources rc=%d", rc);

	return rc;
}

int cam_icp_soc_resources_disable(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_disable_platform_resource(soc_info, CAM_CLK_SW_CLIENT_IDX, true, true);
	if (rc)
		CAM_ERR(CAM_ICP, "failed to disable soc resources rc=%d", rc);

	return rc;
}

int cam_icp_soc_update_clk_rate(struct cam_hw_soc_info *soc_info,
	int32_t clk_level, int hfi_handle)
{
	int32_t src_clk_idx = 0;
	int32_t clk_rate = 0;
	int rc = 0;

	if (!soc_info) {
		CAM_ERR(CAM_ICP, "Invalid soc_info");
		return -EINVAL;
	}

	if ((clk_level < 0) || (clk_level >= CAM_MAX_VOTE)) {
		CAM_ERR(CAM_ICP, "clock level %d is not valid",
			clk_level);
		return -EINVAL;
	}

	if (!soc_info->clk_level_valid[clk_level]) {
		CAM_ERR(CAM_ICP,
			"Clock level %d not supported",
			clk_level);
		return -EINVAL;
	}

	src_clk_idx = soc_info->src_clk_idx;
	if ((src_clk_idx < 0) || (src_clk_idx >= CAM_SOC_MAX_CLK)) {
		CAM_WARN(CAM_ICP, "src_clk not defined for %s",
			soc_info->dev_name);
		return -EINVAL;
	}

	clk_rate = soc_info->clk_rate[clk_level][src_clk_idx];
	if ((soc_info->clk_level_valid[CAM_TURBO_VOTE]) &&
		(soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx] != 0) &&
		(clk_rate > soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx])) {
		CAM_DBG(CAM_ICP, "clk_rate %d greater than max, reset to %d",
			clk_rate,
			soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx]);
		clk_rate = soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx];
	}

	rc = cam_soc_util_set_src_clk_rate(soc_info, CAM_CLK_SW_CLIENT_IDX, clk_rate, 0);
	if (rc)
		return rc;

	hfi_send_freq_info(hfi_handle, clk_rate);
	return 0;
}
