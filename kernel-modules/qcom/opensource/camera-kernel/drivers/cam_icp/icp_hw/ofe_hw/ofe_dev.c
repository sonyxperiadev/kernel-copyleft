// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include "ofe_core.h"
#include "ofe_soc.h"
#include "cam_hw.h"
#include "cam_hw_intf.h"
#include "cam_io_util.h"
#include "cam_icp_hw_intf.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_cpas_api.h"
#include "cam_debug_util.h"
#include "camera_main.h"

static struct cam_ofe_device_hw_info cam_ofe_hw_info = {
	.hw_idx         = 0x0,
	.pwr_ctrl       = 0x100,
	.pwr_status     = 0xFC,
	.top_rst_cmd    = 0x5B4,
	.top_irq_status = 0x5C4,
	.top_rst_val    = 0x3,
	.cdm_rst_cmd    = 0x10,
	.cdm_irq_status = 0x44,
	.cdm_rst_val    = 0x7F,
};

static bool cam_ofe_cpas_cb(uint32_t client_handle, void *userdata,
	struct cam_cpas_irq_data *irq_data)
{
	bool error_handled = false;

	if (!irq_data)
		return error_handled;

	switch (irq_data->irq_type) {
	case CAM_CAMNOC_IRQ_OFE_WR_UBWC_ENCODE_ERROR:
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"OFE Write UBWC Decode error type=%d status=%x thr_err=%d, fcl_err=%d, len_md_err=%d, format_err=%d",
			irq_data->irq_type,
			irq_data->u.dec_err.decerr_status.value,
			irq_data->u.dec_err.decerr_status.thr_err,
			irq_data->u.dec_err.decerr_status.fcl_err,
			irq_data->u.dec_err.decerr_status.len_md_err,
			irq_data->u.dec_err.decerr_status.format_err);
		error_handled = true;
		break;
	case CAM_CAMNOC_IRQ_OFE_RD_UBWC_DECODE_ERROR:
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"OFE Read UBWC Decode error type=%d status=%x thr_err=%d, fcl_err=%d, len_md_err=%d, format_err=%d",
			irq_data->irq_type,
			irq_data->u.dec_err.decerr_status.value,
			irq_data->u.dec_err.decerr_status.thr_err,
			irq_data->u.dec_err.decerr_status.fcl_err,
			irq_data->u.dec_err.decerr_status.len_md_err,
			irq_data->u.dec_err.decerr_status.format_err);
		error_handled = true;
		break;
	default:
		break;
	}

	return error_handled;
}

int cam_ofe_register_cpas(struct cam_hw_soc_info *soc_info,
	struct cam_ofe_device_core_info *core_info, uint32_t hw_idx)
{
	struct cam_cpas_register_params cpas_register_params;
	int rc;

	cpas_register_params.dev = &soc_info->pdev->dev;
	memcpy(cpas_register_params.identifier, "ofe", sizeof("ofe"));
	cpas_register_params.cam_cpas_client_cb = cam_ofe_cpas_cb;
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

static int cam_ofe_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_hw_info                *ofe_dev      = NULL;
	struct cam_hw_intf                *ofe_dev_intf = NULL;
	const struct of_device_id         *match_dev    = NULL;
	struct cam_ofe_device_core_info   *core_info    = NULL;
	struct cam_ofe_device_hw_info     *hw_info      = NULL;
	int                                rc           = 0;
	struct platform_device *pdev = to_platform_device(dev);

	ofe_dev_intf = kzalloc(sizeof(struct cam_hw_intf), GFP_KERNEL);
	if (!ofe_dev_intf)
		return -ENOMEM;

	of_property_read_u32(pdev->dev.of_node,
		"cell-index", &ofe_dev_intf->hw_idx);

	ofe_dev = kzalloc(sizeof(struct cam_hw_info), GFP_KERNEL);
	if (!ofe_dev) {
		rc = -ENOMEM;
		goto free_dev_intf;
	}

	ofe_dev->soc_info.pdev = pdev;
	ofe_dev->soc_info.dev = &pdev->dev;
	ofe_dev->soc_info.dev_name = pdev->name;
	ofe_dev_intf->hw_priv = ofe_dev;
	ofe_dev_intf->hw_ops.init = cam_ofe_init_hw;
	ofe_dev_intf->hw_ops.deinit = cam_ofe_deinit_hw;
	ofe_dev_intf->hw_ops.process_cmd = cam_ofe_process_cmd;
	ofe_dev_intf->hw_type = CAM_ICP_DEV_OFE;
	platform_set_drvdata(pdev, ofe_dev_intf);
	ofe_dev->core_info = kzalloc(sizeof(struct cam_ofe_device_core_info), GFP_KERNEL);
	if (!ofe_dev->core_info) {
		rc = -ENOMEM;
		goto free_dev;
	}
	core_info = (struct cam_ofe_device_core_info *)ofe_dev->core_info;

	match_dev = of_match_device(pdev->dev.driver->of_match_table, &pdev->dev);
	if (!match_dev) {
		CAM_ERR(CAM_ICP, "No ofe hardware info");
		goto free_core_info;
	}

	hw_info = (struct cam_ofe_device_hw_info *)match_dev->data;
	core_info->ofe_hw_info = hw_info;

	rc = cam_ofe_init_soc_resources(&ofe_dev->soc_info, cam_ofe_irq, ofe_dev);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to init_soc");
		goto free_core_info;
	}
	CAM_DBG(CAM_ICP, "soc info : %pK", (void *)&ofe_dev->soc_info);

	rc = cam_ofe_register_cpas(&ofe_dev->soc_info,
			core_info, ofe_dev_intf->hw_idx);
	if (rc)
		goto free_soc_resources;

	ofe_dev->hw_state = CAM_HW_STATE_POWER_DOWN;
	mutex_init(&ofe_dev->hw_mutex);
	spin_lock_init(&ofe_dev->hw_lock);
	init_completion(&ofe_dev->hw_complete);
	CAM_DBG(CAM_ICP, "OFE:%d component bound successfully",
		ofe_dev_intf->hw_idx);

	return rc;

free_soc_resources:
	cam_ofe_deinit_soc_resources(&ofe_dev->soc_info);
free_core_info:
	kfree(ofe_dev->core_info);
free_dev:
	kfree(ofe_dev);
free_dev_intf:
	kfree(ofe_dev_intf);

	return rc;
}

static void cam_ofe_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_hw_info            *ofe_dev = NULL;
	struct cam_hw_intf            *ofe_dev_intf = NULL;
	struct cam_ofe_device_core_info   *core_info = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	CAM_DBG(CAM_ICP, "Unbinding component: %s", pdev->name);
	ofe_dev_intf = platform_get_drvdata(pdev);
	ofe_dev = ofe_dev_intf->hw_priv;
	core_info = (struct cam_ofe_device_core_info *)ofe_dev->core_info;
	cam_cpas_unregister_client(core_info->cpas_handle);
	cam_ofe_deinit_soc_resources(&ofe_dev->soc_info);

	kfree(ofe_dev->core_info);
	kfree(ofe_dev);
	kfree(ofe_dev_intf);
}

static const struct component_ops cam_ofe_component_ops = {
	.bind = cam_ofe_component_bind,
	.unbind = cam_ofe_component_unbind,
};

int cam_ofe_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_ICP, "Adding OFE component");
	rc = component_add(&pdev->dev, &cam_ofe_component_ops);
	if (rc)
		CAM_ERR(CAM_ICP, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_ofe_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_ofe_component_ops);
	return 0;
}

static const struct of_device_id cam_ofe_dt_match[] = {
	{
		.compatible = "qcom,cam-ofe",
		.data = &cam_ofe_hw_info,
	},
	{}
};
MODULE_DEVICE_TABLE(of, cam_ofe_dt_match);

struct platform_driver cam_ofe_driver = {
	.probe = cam_ofe_probe,
	.remove = cam_ofe_remove,
	.driver = {
		.name = "cam-ofe",
		.owner = THIS_MODULE,
		.of_match_table = cam_ofe_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_ofe_init_module(void)
{
	return platform_driver_register(&cam_ofe_driver);
}

void cam_ofe_exit_module(void)
{
	platform_driver_unregister(&cam_ofe_driver);
}

MODULE_DESCRIPTION("CAM OFE driver");
MODULE_LICENSE("GPL v2");
