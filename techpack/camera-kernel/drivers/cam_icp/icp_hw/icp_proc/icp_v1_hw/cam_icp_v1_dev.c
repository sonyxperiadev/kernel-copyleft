// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include "cam_icp_v1_core.h"
#include "cam_io_util.h"
#include "cam_hw.h"
#include "cam_hw_intf.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_cpas_api.h"
#include "cam_debug_util.h"
#include "camera_main.h"
#include "cam_icp_soc_common.h"
#include "cam_icp_v1_dev.h"

static int max_icp_v1_hw_idx = -1;

uint32_t cam_icp_v1_get_device_num(void)
{
	return max_icp_v1_hw_idx + 1;
}

static bool cam_icp_v1_cpas_cb(uint32_t client_handle, void *userdata,
	struct cam_cpas_irq_data *irq_data)
{
	bool error_handled = false;

	if (!irq_data)
		return error_handled;

	switch (irq_data->irq_type) {
	case CAM_CAMNOC_IRQ_IPE_BPS_UBWC_DECODE_ERROR:
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"IPE/BPS UBWC Decode error type=%d status=%x thr_err=%d, fcl_err=%d, len_md_err=%d, format_err=%d",
			irq_data->irq_type,
			irq_data->u.dec_err.decerr_status.value,
			irq_data->u.dec_err.decerr_status.thr_err,
			irq_data->u.dec_err.decerr_status.fcl_err,
			irq_data->u.dec_err.decerr_status.len_md_err,
			irq_data->u.dec_err.decerr_status.format_err);
		error_handled = true;
		break;
	case CAM_CAMNOC_IRQ_IPE_BPS_UBWC_ENCODE_ERROR:
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"IPE/BPS UBWC Encode error type=%d status=%x",
			irq_data->irq_type,
			irq_data->u.enc_err.encerr_status.value);
		error_handled = true;
		break;
	default:
		break;
	}

	return error_handled;
}

int cam_icp_v1_register_cpas(struct cam_hw_soc_info *soc_info,
	struct cam_icp_v1_device_core_info *core_info, uint32_t hw_idx)
{
	struct cam_cpas_register_params cpas_register_params;
	int rc;

	cpas_register_params.dev = &soc_info->pdev->dev;
	memcpy(cpas_register_params.identifier, "icp", sizeof("icp"));
	cpas_register_params.cam_cpas_client_cb = cam_icp_v1_cpas_cb;
	cpas_register_params.cell_index = hw_idx;
	cpas_register_params.userdata = NULL;

	rc = cam_cpas_register_client(&cpas_register_params);
	if (rc < 0) {
		CAM_ERR(CAM_ICP, "failed: %d", rc);
		return rc;
	}

	core_info->cpas_handle = cpas_register_params.client_handle;
	return rc;
}

static inline void cam_icp_v1_soc_info_deinit(struct cam_hw_soc_info *soc_info)
{
	kfree(soc_info->soc_private);
}

static int cam_icp_v1_soc_info_init(struct cam_hw_soc_info *soc_info,
	struct platform_device *pdev)
{
	struct cam_icp_soc_info *icp_soc_info = NULL;

	icp_soc_info = kzalloc(sizeof(*icp_soc_info), GFP_KERNEL);
	if (!icp_soc_info)
		return -ENOMEM;

	soc_info->pdev = pdev;
	soc_info->dev = &pdev->dev;
	soc_info->dev_name = pdev->name;
	soc_info->soc_private = icp_soc_info;

	icp_soc_info->dev_type = CAM_ICP_DEV_ICP_V1;

	return 0;
}

static int cam_icp_v1_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc = 0;
	struct cam_hw_info *icp_v1_dev = NULL;
	struct cam_hw_intf *icp_v1_dev_intf = NULL;
	const struct of_device_id *match_dev = NULL;
	struct cam_icp_v1_device_core_info *core_info = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	icp_v1_dev_intf = kzalloc(sizeof(struct cam_hw_intf), GFP_KERNEL);
	if (!icp_v1_dev_intf)
		return -ENOMEM;

	of_property_read_u32(pdev->dev.of_node,
		"cell-index", &icp_v1_dev_intf->hw_idx);

	icp_v1_dev = kzalloc(sizeof(struct cam_hw_info), GFP_KERNEL);
	if (!icp_v1_dev) {
		rc = -ENOMEM;
		goto icp_v1_dev_alloc_failure;
	}

	icp_v1_dev_intf->hw_priv = icp_v1_dev;
	icp_v1_dev_intf->hw_ops.init = cam_icp_v1_init_hw;
	icp_v1_dev_intf->hw_ops.deinit = cam_icp_v1_deinit_hw;
	icp_v1_dev_intf->hw_ops.process_cmd = cam_icp_v1_process_cmd;
	icp_v1_dev_intf->hw_type = CAM_ICP_DEV_ICP_V1;

	CAM_DBG(CAM_ICP, "type %d index %d",
		icp_v1_dev_intf->hw_type,
		icp_v1_dev_intf->hw_idx);

	platform_set_drvdata(pdev, icp_v1_dev_intf);

	core_info = kzalloc(sizeof(struct cam_icp_v1_device_core_info),
		GFP_KERNEL);
	if (!core_info) {
		rc = -ENOMEM;
		goto core_info_alloc_failure;
	}
	icp_v1_dev->core_info = core_info;

	match_dev = of_match_device(pdev->dev.driver->of_match_table,
		&pdev->dev);
	if (!match_dev) {
		CAM_ERR(CAM_ICP, "No icp_v1 hardware info");
		rc = -EINVAL;
		goto match_err;
	}

	rc = cam_icp_v1_soc_info_init(&icp_v1_dev->soc_info, pdev);
	if (rc)
		goto init_soc_failure;

	rc = cam_icp_soc_resources_init(&icp_v1_dev->soc_info, cam_icp_v1_irq,
		icp_v1_dev);
	if (rc < 0) {
		CAM_ERR(CAM_ICP, "failed to init_soc");
		goto init_soc_failure;
	}

	rc = cam_icp_v1_register_cpas(&icp_v1_dev->soc_info,
		core_info, icp_v1_dev_intf->hw_idx);
	if (rc < 0) {
		CAM_ERR(CAM_ICP, "icp_v1 cpas registration failed");
		goto cpas_reg_failed;
	}
	icp_v1_dev->hw_state = CAM_HW_STATE_POWER_DOWN;
	mutex_init(&icp_v1_dev->hw_mutex);
	spin_lock_init(&icp_v1_dev->hw_lock);
	init_completion(&icp_v1_dev->hw_complete);

	if ((int)(icp_v1_dev_intf->hw_idx) > max_icp_v1_hw_idx)
		max_icp_v1_hw_idx = icp_v1_dev_intf->hw_idx;

	CAM_DBG(CAM_ICP, "ICP_V1:%u component bound successfully",
		icp_v1_dev_intf->hw_idx);

	return 0;

cpas_reg_failed:
	cam_icp_soc_resources_deinit(&icp_v1_dev->soc_info);
init_soc_failure:
	cam_icp_v1_soc_info_deinit(&icp_v1_dev->soc_info);
match_err:
	kfree(core_info);
core_info_alloc_failure:
	kfree(icp_v1_dev);
icp_v1_dev_alloc_failure:
	kfree(icp_v1_dev_intf);

	return rc;
}

static void cam_icp_v1_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_hw_info *icp_v1_dev = NULL;
	struct cam_hw_intf *icp_v1_dev_intf = NULL;
	struct cam_icp_v1_device_core_info *core_info = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	icp_v1_dev_intf = platform_get_drvdata(pdev);
	icp_v1_dev = icp_v1_dev_intf->hw_priv;
	core_info = (struct cam_icp_v1_device_core_info *)icp_v1_dev->core_info;

	cam_cpas_unregister_client(core_info->cpas_handle);
	cam_icp_soc_resources_deinit(&icp_v1_dev->soc_info);
	cam_icp_v1_soc_info_deinit(&icp_v1_dev->soc_info);

	max_icp_v1_hw_idx = -1;

	kfree(icp_v1_dev->core_info);
	kfree(icp_v1_dev);
	kfree(icp_v1_dev_intf);
}

static const struct component_ops cam_icp_v1_component_ops = {
	.bind = cam_icp_v1_component_bind,
	.unbind = cam_icp_v1_component_unbind,
};

int cam_icp_v1_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_ICP, "Adding ICP_V1 component");
	rc = component_add(&pdev->dev, &cam_icp_v1_component_ops);
	if (rc)
		CAM_ERR(CAM_ICP, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_icp_v1_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_icp_v1_component_ops);
	return 0;
}

static const struct of_device_id cam_icp_v1_dt_match[] = {
	{.compatible = "qcom,cam-icp_v1",},
	{},
};
MODULE_DEVICE_TABLE(of, cam_icp_v1_dt_match);

struct platform_driver cam_icp_v1_driver = {
	.probe = cam_icp_v1_probe,
	.remove = cam_icp_v1_remove,
	.driver = {
		.name = "cam-icp_v1",
		.owner = THIS_MODULE,
		.of_match_table = cam_icp_v1_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_icp_v1_init_module(void)
{
	return platform_driver_register(&cam_icp_v1_driver);
}

void cam_icp_v1_exit_module(void)
{
	platform_driver_unregister(&cam_icp_v1_driver);
}

MODULE_DESCRIPTION("CAM ICP_V1 driver");
MODULE_LICENSE("GPL v2");
