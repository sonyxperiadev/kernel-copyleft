// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_debug_util.h"
#include "cam_icp_proc.h"

uint32_t icp_request_cnt[CAM_ICP_MAX_ICP_HW_TYPE];

static int cam_icp_get_device_num(enum cam_icp_hw_type dev_type, uint32_t *num_dev)
{
	int rc = 0;

	switch (dev_type) {
	case CAM_ICP_HW_ICP_V1:
		*num_dev = cam_icp_v1_get_device_num();
		break;
	case CAM_ICP_HW_ICP_V2:
		*num_dev = cam_icp_v2_get_device_num();
		break;
	default:
		CAM_ERR(CAM_ICP, "Invalid dev type: %d", dev_type);
		rc = -EINVAL;
	}

	return rc;
}

int cam_icp_alloc_processor_devs(struct device_node *np, enum cam_icp_hw_type *icp_hw_type,
	struct cam_hw_intf ***devices, uint32_t *hw_dev_cnt)
{
	uint32_t num_icp_found = 0, num_icp_listed;
	int rc, i;

	if (!np) {
		CAM_ERR(CAM_ICP, "Invalid device node");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "num-icp", &num_icp_listed);
	if (rc) {
		CAM_ERR(CAM_ICP, "read num-icp failed rc=%d", rc);
		return -ENODEV;
	}

	if (!num_icp_listed) {
		CAM_ERR(CAM_ICP, "No ICP device %d", num_icp_listed);
		return -EINVAL;
	}

	/* Only one version of ICP processor supported per ICP subdevice */
	for (i = 0; i < CAM_ICP_MAX_ICP_HW_TYPE; i++) {
		rc = cam_icp_get_device_num(i, &num_icp_found);
		if (rc)
			return rc;

		if (num_icp_found) {
			*icp_hw_type = i;
			break;
		}
	}

	if (i == CAM_ICP_MAX_ICP_HW_TYPE) {
		CAM_ERR(CAM_ICP, "No ICP device probed");
		return -ENODEV;
	}

	icp_request_cnt[i] += num_icp_listed;

	if (icp_request_cnt[i] > num_icp_found) {
		CAM_ERR(CAM_ICP,
			"number of ICP_V%u total requested: %u exceeds number of icp hw available: %u",
			i+1, icp_request_cnt[i], num_icp_found);
		return -EINVAL;
	}

	*devices = kcalloc(num_icp_listed, sizeof(**devices), GFP_KERNEL);
	if (!(*devices)) {
		CAM_ERR(CAM_ICP,
			"ICP device memory allocation failed. Num devices: %u",
			num_icp_listed);
		return -ENOMEM;
	}

	hw_dev_cnt[i] = num_icp_listed;

	CAM_DBG(CAM_ICP, "allocated device iface for %s",
		i == CAM_ICP_HW_ICP_V1 ? "ICP_V1" : "ICP_V2");

	return rc;
}

int cam_icp_get_hfi_device_ops(uint32_t hw_type, const struct hfi_ops **hfi_proc_ops)
{
	int rc = 0;

	switch (hw_type) {
	case CAM_ICP_HW_ICP_V1:
		cam_icp_v1_populate_hfi_ops(hfi_proc_ops);
		break;
	case CAM_ICP_HW_ICP_V2:
		cam_icp_v2_populate_hfi_ops(hfi_proc_ops);
		break;
	default:
		rc = -EINVAL;
		CAM_ERR(CAM_ICP, "Invalid ICP device type: %u", hw_type);
	}

	return rc;
}
