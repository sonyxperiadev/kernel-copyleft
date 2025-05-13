// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/cam_req_mgr.h>
#include <media/cam_defs.h>
#include <media/cam_ope.h>
#include "cam_req_mgr_dev.h"
#include "cam_subdev.h"
#include "cam_node.h"
#include "cam_context.h"
#include "cam_ope_context.h"
#include "cam_ope_hw_mgr_intf.h"
#include "cam_hw_mgr_intf.h"
#include "cam_debug_util.h"
#include "cam_smmu_api.h"
#include "camera_main.h"
#include "cam_context_utils.h"

#define OPE_DEV_NAME        "cam-ope"

struct cam_ope_subdev {
	struct cam_subdev sd;
	struct cam_node *node;
	struct cam_context ctx[OPE_CTX_MAX];
	struct cam_ope_context ctx_ope[OPE_CTX_MAX];
	struct mutex ope_lock;
	int32_t open_cnt;
	int32_t reserved;
};

static struct cam_ope_subdev g_ope_dev;

static void cam_ope_dev_iommu_fault_handler(
	struct cam_smmu_pf_info *pf_smmu_info)
{
	int i, rc;
	struct cam_node *node = NULL;
	struct cam_hw_dump_pf_args pf_args = {0};

	if (!pf_smmu_info || !pf_smmu_info->token) {
		CAM_ERR(CAM_OPE, "invalid token in page handler cb");
		return;
	}

	node = (struct cam_node *)pf_smmu_info->token;
	pf_args.pf_smmu_info = pf_smmu_info;

	for (i = 0; i < node->ctx_size; i++) {
		cam_context_dump_pf_info(&(node->ctx_list[i]), &pf_args);
		if (pf_args.pf_context_info.ctx_found)
			/* found ctx and packet of the faulted address */
			break;
	}

	if (i == node->ctx_size) {
		/* Faulted ctx not found. But report PF to UMD anyway*/
		rc = cam_context_send_pf_evt(NULL, &pf_args);
		if (rc)
			CAM_ERR(CAM_OPE,
				"Failed to notify PF event to userspace rc: %d", rc);
	}
}

static int cam_ope_subdev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_hw_mgr_intf *hw_mgr_intf = NULL;
	struct cam_node *node = v4l2_get_subdevdata(sd);
	int rc = 0;

	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_LOCK);

	mutex_lock(&g_ope_dev.ope_lock);
	if (g_ope_dev.open_cnt >= 1) {
		CAM_ERR(CAM_OPE, "OPE subdev is already opened");
		rc = -EALREADY;
		goto end;
	}

	if (!node) {
		CAM_ERR(CAM_OPE, "Invalid args");
		rc = -EINVAL;
		goto end;
	}

	hw_mgr_intf = &node->hw_mgr_intf;
	rc = hw_mgr_intf->hw_open(hw_mgr_intf->hw_mgr_priv, NULL);
	if (rc < 0) {
		CAM_ERR(CAM_OPE, "OPE HW open failed: %d", rc);
		goto end;
	}
	g_ope_dev.open_cnt++;
	CAM_DBG(CAM_OPE, "OPE HW open success: %d", rc);
end:
	mutex_unlock(&g_ope_dev.ope_lock);
	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_UNLOCK);
	return rc;
}

static int cam_ope_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	struct cam_hw_mgr_intf *hw_mgr_intf = NULL;
	struct cam_node *node = v4l2_get_subdevdata(sd);

	mutex_lock(&g_ope_dev.ope_lock);
	if (g_ope_dev.open_cnt <= 0) {
		CAM_DBG(CAM_OPE, "OPE subdev is already closed");
		rc = -EINVAL;
		goto end;
	}
	g_ope_dev.open_cnt--;
	if (!node) {
		CAM_ERR(CAM_OPE, "Invalid args");
		rc = -EINVAL;
		goto end;
	}

	hw_mgr_intf = &node->hw_mgr_intf;
	if (!hw_mgr_intf) {
		CAM_ERR(CAM_OPE, "hw_mgr_intf is not initialized");
		rc = -EINVAL;
		goto end;
	}

	rc = cam_node_shutdown(node);
	if (rc < 0) {
		CAM_ERR(CAM_OPE, "HW close failed");
		goto end;
	}
	CAM_DBG(CAM_OPE, "OPE HW close success: %d", rc);

end:
	mutex_unlock(&g_ope_dev.ope_lock);
	return rc;
}

static int cam_ope_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open();

	if (crm_active) {
		CAM_DBG(CAM_OPE, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}

	return cam_ope_subdev_close_internal(sd, fh);
}

const struct v4l2_subdev_internal_ops cam_ope_subdev_internal_ops = {
	.open = cam_ope_subdev_open,
	.close = cam_ope_subdev_close,
};

static int cam_ope_subdev_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc = 0, i = 0;
	struct cam_node *node;
	struct cam_hw_mgr_intf *hw_mgr_intf;
	int iommu_hdl = -1;
	struct platform_device *pdev = to_platform_device(dev);

	CAM_DBG(CAM_OPE, "Binding OPE subdev component");
	if (!pdev) {
		CAM_ERR(CAM_OPE, "pdev is NULL");
		return -EINVAL;
	}

	g_ope_dev.sd.pdev = pdev;
	g_ope_dev.sd.internal_ops = &cam_ope_subdev_internal_ops;
	g_ope_dev.sd.close_seq_prior = CAM_SD_CLOSE_MEDIUM_PRIORITY;
	rc = cam_subdev_probe(&g_ope_dev.sd, pdev, OPE_DEV_NAME,
		CAM_OPE_DEVICE_TYPE);
	if (rc) {
		CAM_ERR(CAM_OPE, "OPE cam_subdev_probe failed:%d", rc);
		return rc;
	}

	node = (struct cam_node *) g_ope_dev.sd.token;

	hw_mgr_intf = kzalloc(sizeof(*hw_mgr_intf), GFP_KERNEL);
	if (!hw_mgr_intf) {
		rc = -EINVAL;
		goto hw_alloc_fail;
	}

	rc = cam_ope_hw_mgr_init(pdev->dev.of_node, (uint64_t *)hw_mgr_intf,
		&iommu_hdl);
	if (rc) {
		CAM_ERR(CAM_OPE, "OPE HW manager init failed: %d", rc);
		goto hw_init_fail;
	}

	for (i = 0; i < OPE_CTX_MAX; i++) {
		g_ope_dev.ctx_ope[i].base = &g_ope_dev.ctx[i];
		rc = cam_ope_context_init(&g_ope_dev.ctx_ope[i],
			hw_mgr_intf, i, iommu_hdl);
		if (rc) {
			CAM_ERR(CAM_OPE, "OPE context init failed");
			goto ctx_fail;
		}
	}

	rc = cam_node_init(node, hw_mgr_intf, g_ope_dev.ctx,
		OPE_CTX_MAX, OPE_DEV_NAME);
	if (rc) {
		CAM_ERR(CAM_OPE, "OPE node init failed");
		goto ctx_fail;
	}

	cam_smmu_set_client_page_fault_handler(iommu_hdl,
		cam_ope_dev_iommu_fault_handler, node);

	g_ope_dev.open_cnt = 0;
	mutex_init(&g_ope_dev.ope_lock);

	node->sd_handler = cam_ope_subdev_close_internal;
	CAM_DBG(CAM_OPE, "Subdev component bound successfully");

	return rc;

ctx_fail:
	for (--i; i >= 0; i--)
		cam_ope_context_deinit(&g_ope_dev.ctx_ope[i]);
hw_init_fail:
	kfree(hw_mgr_intf);
hw_alloc_fail:
	cam_subdev_remove(&g_ope_dev.sd);
	return rc;
}

static void cam_ope_subdev_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int i;
	struct v4l2_subdev *sd;
	struct cam_subdev *subdev;
	struct platform_device *pdev = to_platform_device(dev);

	if (!pdev) {
		CAM_ERR(CAM_OPE, "pdev is NULL");
		return;
	}

	sd = platform_get_drvdata(pdev);
	if (!sd) {
		CAM_ERR(CAM_OPE, "V4l2 subdev is NULL");
		return;
	}

	subdev = v4l2_get_subdevdata(sd);
	if (!subdev) {
		CAM_ERR(CAM_OPE, "cam subdev is NULL");
		return;
	}

	for (i = 0; i < OPE_CTX_MAX; i++)
		cam_ope_context_deinit(&g_ope_dev.ctx_ope[i]);

	cam_node_deinit(g_ope_dev.node);
	cam_subdev_remove(&g_ope_dev.sd);
	mutex_destroy(&g_ope_dev.ope_lock);
}

const static struct component_ops cam_ope_subdev_component_ops = {
	.bind = cam_ope_subdev_component_bind,
	.unbind = cam_ope_subdev_component_unbind,
};

static int cam_ope_subdev_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_OPE, "Adding OPE subdev component");
	rc = component_add(&pdev->dev, &cam_ope_subdev_component_ops);
	if (rc)
		CAM_ERR(CAM_OPE, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_ope_subdev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_ope_subdev_component_ops);
	return 0;
}

static const struct of_device_id cam_ope_dt_match[] = {
	{.compatible = "qcom,cam-ope"},
	{}
};


struct platform_driver cam_ope_subdev_driver = {
	.probe = cam_ope_subdev_probe,
	.remove = cam_ope_subdev_remove,
	.driver = {
		.name = "cam_ope",
		.of_match_table = cam_ope_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_ope_subdev_init_module(void)
{
	return platform_driver_register(&cam_ope_subdev_driver);
}

void cam_ope_subdev_exit_module(void)
{
	platform_driver_unregister(&cam_ope_subdev_driver);
}

MODULE_DESCRIPTION("MSM OPE driver");
MODULE_LICENSE("GPL v2");

