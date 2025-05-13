// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
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
#include <media/cam_icp.h>
#include "cam_req_mgr_dev.h"
#include "cam_subdev.h"
#include "cam_node.h"
#include "cam_context.h"
#include "cam_icp_context.h"
#include "cam_hw_mgr_intf.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_debug_util.h"
#include "cam_smmu_api.h"
#include "camera_main.h"
#include "cam_common_util.h"
#include "cam_context_utils.h"


#define CAM_ICP_IS_DEV_IDX_INVALID(dev_idx)                   \
({                                                            \
	((dev_idx) < 0) || ((dev_idx) >= CAM_ICP_SUBDEV_MAX); \
})

struct cam_icp_subdev {
	struct cam_subdev sd;
	struct cam_node *node;
	struct cam_context ctx[CAM_ICP_CTX_MAX];
	struct cam_icp_context ctx_icp[CAM_ICP_CTX_MAX];
	struct mutex icp_lock;
	int32_t open_cnt;
	int32_t reserved;
};

static DEFINE_MUTEX(g_dev_lock);
static struct cam_icp_subdev *g_icp_dev[CAM_ICP_SUBDEV_MAX];

static char cam_icp_subdev_name_arr[CAM_ICP_SUBDEV_MAX][CAM_ICP_SUBDEV_NAME_LEN] = {
	"cam-icp0",
	"cam-icp1",
};

static const struct of_device_id cam_icp_dt_match[] = {
	{.compatible = "qcom,cam-icp"},
	{.compatible = "qcom,cam-icp0"},
	{.compatible = "qcom,cam-icp1"},
	{}
};

static int cam_icp_dev_evt_inject_cb(void *inject_args)
{
	struct cam_common_inject_evt_param *inject_params = inject_args;
	struct cam_icp_subdev *icp_dev;
	int i;

	/* Event Injection currently supported for only a single instance of ICP */
	icp_dev = g_icp_dev[0];

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		if (icp_dev->ctx[i].dev_hdl == inject_params->dev_hdl) {
			cam_context_add_evt_inject(&icp_dev->ctx[i],
				&inject_params->evt_params);
			return 0;
		}
	}

	CAM_ERR(CAM_ICP, "No dev hdl found %d", inject_params->dev_hdl);
	return -ENODEV;
}

static void cam_icp_dev_iommu_fault_handler(struct cam_smmu_pf_info *pf_smmu_info)
{
	int i, rc;
	struct cam_node *node = NULL;
	struct cam_hw_dump_pf_args pf_args = {0};

	if (!pf_smmu_info || !pf_smmu_info->token) {
		CAM_ERR(CAM_ICP, "invalid token in page handler cb");
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
		/* Faulted ctx not found. Report PF to userspace */
		rc = cam_context_send_pf_evt(NULL, &pf_args);
		if (rc)
			CAM_ERR(CAM_ICP,
				"Failed to notify PF event to userspace rc: %d", rc);
	}
}

static void cam_icp_dev_mini_dump_cb(void *priv, void *args)
{
	struct cam_context *ctx = NULL;

	if (!priv || !args) {
		CAM_ERR(CAM_ICP, "Invalid params: priv: %pK args %pK", priv, args);
		return;
	}

	ctx = (struct cam_context *)priv;
	cam_context_mini_dump_from_hw(ctx, args);
}

static int cam_icp_subdev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_hw_mgr_intf *hw_mgr_intf = NULL;
	struct cam_node *node = v4l2_get_subdevdata(sd);
	struct cam_icp_subdev *icp_dev = NULL;
	int rc = 0;

	if (!node) {
		CAM_ERR(CAM_ICP, "Invalid params: Node is NULL");
		return -EINVAL;
	}

	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_LOCK);
	CAM_DBG(CAM_ICP, "Enter device open for %s[%u]",
		sd->name, node->device_idx);

	if (CAM_ICP_IS_DEV_IDX_INVALID(node->device_idx)) {
		CAM_ERR(CAM_ICP, "Invalid device idx: %u for device: %s",
			node->device_idx, sd->name);
		cam_req_mgr_rwsem_read_op(CAM_SUBDEV_UNLOCK);
		return -ENODEV;
	}
	icp_dev = g_icp_dev[node->device_idx];

	mutex_lock(&icp_dev->icp_lock);
	if (icp_dev->open_cnt >= 1) {
		CAM_ERR(CAM_ICP, "device[%s] is already opened, open count: %u",
			sd->name, icp_dev->open_cnt);
		rc = -EALREADY;
		goto end;
	}

	hw_mgr_intf = &node->hw_mgr_intf;
	rc = hw_mgr_intf->hw_open(hw_mgr_intf->hw_mgr_priv, NULL);
	if (rc < 0) {
		CAM_ERR(CAM_ICP, "FW download failed for device [%s]", sd->name);
		goto end;
	}

	icp_dev->open_cnt++;

end:
	mutex_unlock(&icp_dev->icp_lock);
	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_UNLOCK);
	return rc;
}

static int cam_icp_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	struct cam_hw_mgr_intf *hw_mgr_intf = NULL;
	struct cam_node *node = v4l2_get_subdevdata(sd);
	struct cam_icp_subdev *icp_dev = NULL;

	if (!node) {
		CAM_ERR(CAM_ICP, "device[%s] Invalid params: node is NULL",
			sd->name);
		rc = -EINVAL;
		goto end;
	}

	if (CAM_ICP_IS_DEV_IDX_INVALID(node->device_idx)) {
		CAM_ERR(CAM_ICP, "Invalid device idx: %u for device: %s",
			node->device_idx, node->name);
		return -ENODEV;
	}
	icp_dev = g_icp_dev[node->device_idx];

	mutex_lock(&icp_dev->icp_lock);
	if (icp_dev->open_cnt <= 0) {
		CAM_DBG(CAM_ICP, "device[%s] is already closed",
			sd->name);
		goto end;
	}

	icp_dev->open_cnt--;

	hw_mgr_intf = &node->hw_mgr_intf;
	if (!hw_mgr_intf) {
		CAM_ERR(CAM_ICP, "device[%s] hw_mgr_intf is not initialized",
			sd->name);
		rc = -EINVAL;
		goto end;
	}

	rc = cam_node_shutdown(node);
	if (rc < 0) {
		CAM_ERR(CAM_ICP, "device[%s] HW close failed",
			sd->name);
		goto end;
	}

end:
	mutex_unlock(&icp_dev->icp_lock);
	return rc;
}

static int cam_icp_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open();

	if (crm_active) {
		CAM_DBG(CAM_ICP,
			"CRM is ACTIVE, close should be from CRM for device[%s]",
			sd->name);
		return 0;
	}

	return cam_icp_subdev_close_internal(sd, fh);
}

const struct v4l2_subdev_internal_ops cam_icp_subdev_internal_ops = {
	.open = cam_icp_subdev_open,
	.close = cam_icp_subdev_close,
};

static inline int cam_icp_subdev_clean_up(uint32_t device_idx)
{
	struct cam_icp_subdev *icp_subdev;

	icp_subdev = g_icp_dev[device_idx];
	icp_subdev->node = NULL;
	icp_subdev->open_cnt = 0;

	return 0;
}

static int cam_icp_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc = 0, i = 0;
	struct cam_node *node;
	struct cam_hw_mgr_intf hw_mgr_intf;
	int iommu_hdl = -1;
	struct platform_device *pdev = to_platform_device(dev);
	struct cam_icp_subdev *icp_dev;
	char *subdev_name;
	uint32_t device_idx;

	if (!pdev) {
		CAM_ERR(CAM_ICP, "Invalid params: pdev is %s",
			CAM_IS_NULL_TO_STR(pdev));
		return -EINVAL;
	}

	rc = of_property_read_u32(dev->of_node, "cell-index", &device_idx);
	if (rc)
		device_idx = 0;

	if (CAM_ICP_IS_DEV_IDX_INVALID(device_idx)) {
		CAM_ERR(CAM_ICP, "Invalid device idx: %u exceeds subdev max: %u",
			device_idx, CAM_ICP_SUBDEV_MAX);
		return -EINVAL;
	}

	/*
	 * For targets where only one subdevice exists, cell-index property is not listed
	 * in the DT node, so the default name and device index are "cam-icp" and 0 respectively
	 */
	if (rc)
		subdev_name = "cam-icp";
	else
		subdev_name = cam_icp_subdev_name_arr[device_idx];

	icp_dev = kzalloc(sizeof(struct cam_icp_subdev), GFP_KERNEL);
	if (!icp_dev) {
		CAM_ERR(CAM_ICP,
			"Unable to allocate memory for icp device:%s size:%llu",
			pdev->name, sizeof(struct cam_icp_subdev));
		return -ENOMEM;
	}

	mutex_lock(&g_dev_lock);
	if (g_icp_dev[device_idx]) {
		CAM_ERR(CAM_ICP,
			"Invalid device index: %u for pdev: %s, ICP device for this idx is already bound",
			device_idx, pdev->name);
		rc = -EBADSLT;
		mutex_unlock(&g_dev_lock);
		goto probe_fail;
	}
	g_icp_dev[device_idx] = icp_dev;
	mutex_unlock(&g_dev_lock);

	icp_dev->sd.internal_ops = &cam_icp_subdev_internal_ops;
	icp_dev->sd.close_seq_prior = CAM_SD_CLOSE_MEDIUM_PRIORITY;
	rc = cam_subdev_probe(&icp_dev->sd, pdev, subdev_name,
		CAM_ICP_DEVICE_TYPE);
	if (rc) {
		CAM_ERR(CAM_ICP, "device[%s] probe failed", subdev_name);
		goto probe_fail;
	}

	node = (struct cam_node *) icp_dev->sd.token;
	node->sd_handler = cam_icp_subdev_close_internal;
	node->device_idx = device_idx;
	mutex_init(&icp_dev->icp_lock);

	rc = cam_icp_hw_mgr_init(pdev->dev.of_node, (uint64_t *)(&hw_mgr_intf),
		&iommu_hdl, cam_icp_dev_mini_dump_cb, device_idx);
	if (rc) {
		CAM_ERR(CAM_ICP, "device[%s] HW manager init failed: %d", subdev_name, rc);
		goto hw_init_fail;
	}

	for (i = 0; i < CAM_ICP_CTX_MAX; i++) {
		icp_dev->ctx_icp[i].base = &icp_dev->ctx[i];
		rc = cam_icp_context_init(&icp_dev->ctx_icp[i],
			&node->hw_mgr_intf, i, iommu_hdl, subdev_name);
		if (rc) {
			CAM_ERR(CAM_ICP, "device[%s] context init failed", subdev_name);
			goto ctx_fail;
		}
	}

	rc = cam_node_init(node, &hw_mgr_intf, icp_dev->ctx,
		CAM_ICP_CTX_MAX, subdev_name);
	if (rc) {
		CAM_ERR(CAM_ICP, "device[%s] node init failed", subdev_name);
		goto ctx_fail;
	}

	cam_common_register_evt_inject_cb(cam_icp_dev_evt_inject_cb,
		CAM_COMMON_EVT_INJECT_HW_ICP);

	cam_smmu_set_client_page_fault_handler(iommu_hdl,
		cam_icp_dev_iommu_fault_handler, node);

	icp_dev->open_cnt = 0;

	CAM_DBG(CAM_ICP, "device[%s] id: %u component bound successfully",
		subdev_name, device_idx);

	return rc;

ctx_fail:
	for (--i; i >= 0; i--)
		cam_icp_context_deinit(&icp_dev->ctx_icp[i]);
	cam_icp_hw_mgr_deinit(device_idx);
hw_init_fail:
	cam_subdev_remove(&icp_dev->sd);
probe_fail:
	cam_icp_subdev_clean_up(device_idx);
	return rc;
}

static void cam_icp_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int i, rc;
	struct platform_device *pdev = to_platform_device(dev);
	struct v4l2_subdev *sd;
	struct cam_icp_subdev *icp_dev;
	uint32_t device_idx;

	if (!pdev) {
		CAM_ERR(CAM_ICP, "pdev is NULL");
		return;
	}

	sd = platform_get_drvdata(pdev);
	if (!sd) {
		CAM_ERR(CAM_ICP,
			"V4l2 subdev is NULL for pdev: %s", pdev->name);
		return;
	}

	rc = of_property_read_u32(dev->of_node, "cell-index", &device_idx);
	if (rc)
		device_idx = 0;

	if (CAM_ICP_IS_DEV_IDX_INVALID(device_idx)) {
		CAM_ERR(CAM_ICP, "Invalid device idx: %u exceeds subdev max: %u",
			device_idx, CAM_ICP_SUBDEV_MAX);
		return;
	}

	icp_dev = g_icp_dev[device_idx];

	for (i = 0; i < CAM_ICP_CTX_MAX; i++)
		cam_icp_context_deinit(&icp_dev->ctx_icp[i]);

	cam_icp_hw_mgr_deinit(device_idx);
	cam_node_deinit(icp_dev->node);
	cam_subdev_remove(&icp_dev->sd);
	mutex_destroy(&icp_dev->icp_lock);
	cam_icp_subdev_clean_up(device_idx);

	CAM_DBG(CAM_ICP, "device[%s] component unbinded successfully", pdev->name);
}

const static struct component_ops cam_icp_component_ops = {
	.bind = cam_icp_component_bind,
	.unbind = cam_icp_component_unbind,
};

static int cam_icp_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_ICP, "%s Adding ICP component", pdev->name);
	rc = component_add(&pdev->dev, &cam_icp_component_ops);
	if (rc)
		CAM_ERR(CAM_ICP, "%s failed to add component rc: %d", pdev->name, rc);

	return rc;
}

static int cam_icp_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_icp_component_ops);
	return 0;
}

struct platform_driver cam_icp_driver = {
	.probe = cam_icp_probe,
	.remove = cam_icp_remove,
	.driver = {
		.name = "cam_icp",
		.owner = THIS_MODULE,
		.of_match_table = cam_icp_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_icp_init_module(void)
{
	return platform_driver_register(&cam_icp_driver);
}

void cam_icp_exit_module(void)
{
	platform_driver_unregister(&cam_icp_driver);
}

MODULE_DESCRIPTION("MSM ICP driver");
MODULE_LICENSE("GPL v2");
