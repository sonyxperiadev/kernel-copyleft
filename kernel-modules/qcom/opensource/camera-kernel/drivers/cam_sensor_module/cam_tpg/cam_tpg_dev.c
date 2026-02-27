// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_tpg_dev.h"
#include "cam_tpg_core.h"
#include "camera_main.h"
#include "tpg_hw/tpg_hw_v_1_0/tpg_hw_v_1_0_data.h"
#include "tpg_hw/tpg_hw_v_1_2/tpg_hw_v_1_2_data.h"
#include "tpg_hw/tpg_hw_v_1_3/tpg_hw_v_1_3_data.h"
#include "tpg_hw/tpg_hw_v_1_4/tpg_hw_v_1_4_data.h"

static int cam_tpg_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_tpg_device *tpg_dev =
		v4l2_get_subdevdata(sd);

	bool crm_active = cam_req_mgr_is_open();

	if (!tpg_dev) {
		CAM_ERR(CAM_TPG, "tpg_dev ptr is NULL");
		return -EINVAL;
	}

	if (crm_active) {
		CAM_DBG(CAM_TPG, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}

	mutex_lock(&tpg_dev->mutex);
	if (tpg_dev->state == CAM_TPG_STATE_INIT) {
		CAM_DBG(CAM_TPG, "TPG node %d is succesfully closed", tpg_dev->soc_info.index);
		mutex_unlock(&tpg_dev->mutex);
		return 0;
	}
	cam_tpg_shutdown(tpg_dev);
	mutex_unlock(&tpg_dev->mutex);

	return 0;
}

static long cam_tpg_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	struct cam_tpg_device *tpg_dev = v4l2_get_subdevdata(sd);
	int rc = 0;

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_tpg_core_cfg(tpg_dev, arg);
		break;
	case CAM_SD_SHUTDOWN:
		if (!cam_req_mgr_is_shutdown()) {
			CAM_ERR(CAM_CORE, "SD shouldn't come from user space");
			return 0;
		}
		mutex_lock(&tpg_dev->mutex);
		cam_tpg_shutdown(tpg_dev);
		mutex_unlock(&tpg_dev->mutex);
		break;
	default:
		CAM_ERR(CAM_TPG, "Wrong ioctl : %d", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}


#ifdef CONFIG_COMPAT
static long cam_tpg_subdev_compat_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	int32_t rc = 0;
	struct cam_control cmd_data;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_TPG, "Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	/* All the arguments converted to 64 bit here
	 * Passed to the api in core.c
	 */
	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_tpg_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc)
			CAM_ERR(CAM_TPG,
				"Failed in subdev_ioctl: %d", rc);
		break;
	default:
		CAM_ERR(CAM_TPG, "Invalid compat ioctl cmd: %d", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_TPG,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}

	return rc;
}
#endif


static struct v4l2_subdev_core_ops tpg_subdev_core_ops = {
	.ioctl = cam_tpg_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_tpg_subdev_compat_ioctl,
#endif
};


static const struct v4l2_subdev_ops tpg_subdev_ops = {
	.core = &tpg_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops tpg_subdev_intern_ops = {
	.close = cam_tpg_subdev_close,
};

static irqreturn_t cam_tpg_irq_handler(int irq_num, void *data)
{
	struct cam_tpg_device *tpg_dev =
		(struct cam_tpg_device *)data;
	struct tpg_hw *hw = &tpg_dev->tpg_hw;
	irqreturn_t    rc = IRQ_NONE;

	/* handle the registered irq handler */
	if (hw &&
		hw->hw_info &&
		hw->hw_info->ops &&
		hw->hw_info->ops->handle_irq) {
		rc = hw->hw_info->ops->handle_irq(hw);
	}

	return rc;
}

static int tpg_subdev_init(struct cam_tpg_device *tpg_dev,
		struct device *dev)
{
	int32_t rc = 0;
	struct platform_device *pdev = to_platform_device(dev);

	tpg_dev->tpg_subdev.pdev = pdev;
	tpg_dev->tpg_subdev.internal_ops = &tpg_subdev_intern_ops;
	tpg_dev->tpg_subdev.ops = &tpg_subdev_ops;
	tpg_dev->tpg_subdev.name = tpg_dev->device_name;
	tpg_dev->tpg_subdev.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	tpg_dev->tpg_subdev.ent_function = CAM_TPG_DEVICE_TYPE;
	tpg_dev->tpg_subdev.msg_cb = NULL;
	tpg_dev->tpg_subdev.token = tpg_dev;

	rc = cam_register_subdev(&(tpg_dev->tpg_subdev));
	if (rc)
		CAM_ERR(CAM_TPG, "cam_register_subdev Failed rc: %d", rc);
	else
		CAM_DBG(CAM_TPG, "TPG subdev init done");
	return rc;

}

static int tpg_soc_info_init(struct cam_tpg_device *tpg_dev,
		struct device *dev)
{
	int32_t rc = 0, i;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *of_node = NULL;
	void *irq_data[CAM_SOC_MAX_IRQ_LINES_PER_DEV] = {0};

	if (!dev || !tpg_dev)
		return -EINVAL;

	tpg_dev->soc_info.pdev = pdev;
	tpg_dev->soc_info.dev = &pdev->dev;
	tpg_dev->soc_info.dev_name = pdev->name;

	of_node = dev->of_node;

	rc = cam_soc_util_get_dt_properties(&tpg_dev->soc_info);
	if (rc < 0) {
		CAM_ERR(CAM_CSIPHY, "parsing common soc dt(rc %d)", rc);
		return  rc;
	}

	rc = of_property_read_u32(of_node, "phy-id", &(tpg_dev->phy_id));
	if (rc) {
		CAM_ERR(CAM_TPG, "device %s failed to read phy-id",
			tpg_dev->soc_info.dev_name);
		return rc;
	}

	for (i = 0; i < tpg_dev->soc_info.irq_count; i++)
		irq_data[i] = tpg_dev;

	rc = cam_soc_util_request_platform_resource(
		&tpg_dev->soc_info, cam_tpg_irq_handler, &(irq_data[0]));
	if (rc)
		CAM_ERR(CAM_TPG, "unable to request platfrom resources");
	else
		CAM_DBG(CAM_TPG, "TPG dt parse done");

	return rc;
}

static int tpg_register_cpas_client(struct cam_tpg_device *tpg_dev,
		struct device *dev)
{
	int32_t rc = 0;
	struct cam_cpas_register_params cpas_parms;
	struct platform_device *pdev = to_platform_device(dev);

	cpas_parms.cam_cpas_client_cb = NULL;
	cpas_parms.cell_index = tpg_dev->soc_info.index;
	cpas_parms.dev = &pdev->dev;
	cpas_parms.userdata = tpg_dev;

	strlcpy(cpas_parms.identifier, "tpg", CAM_HW_IDENTIFIER_LENGTH);

	rc = cam_cpas_register_client(&cpas_parms);
	if (rc) {
		CAM_ERR(CAM_TPG, "CPAS registration failed rc: %d", rc);
		return rc;
	}

	tpg_dev->cpas_handle = cpas_parms.client_handle;
	CAM_DBG(CAM_TPG, "CPAS registration successful handle=%d",
		cpas_parms.client_handle);

	return rc;
}

static int cam_tpg_hw_layer_init(struct cam_tpg_device *tpg_dev,
		struct device *dev)
{
	/* get top tpg hw information */
	const struct of_device_id      *match_dev = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	match_dev = of_match_device(pdev->dev.driver->of_match_table,
		&pdev->dev);
	if (!match_dev) {
		CAM_ERR(CAM_TPG, "No matching table for the top tpg hw");
		return -EINVAL;
	}

	tpg_dev->tpg_hw.hw_idx   = tpg_dev->soc_info.index;
	tpg_dev->tpg_hw.hw_info  = (struct tpg_hw_info *)match_dev->data;
	tpg_dev->tpg_hw.soc_info = &tpg_dev->soc_info;
	tpg_dev->tpg_hw.cpas_handle = tpg_dev->cpas_handle;
	spin_lock_init(&tpg_dev->tpg_hw.hw_state_lock);
	tpg_dev->tpg_hw.state = TPG_HW_STATE_HW_DISABLED;
	mutex_init(&tpg_dev->tpg_hw.mutex);
	init_completion(&tpg_dev->tpg_hw.complete_rup);
	/*Initialize the waiting queue and active queues*/
	INIT_LIST_HEAD(&(tpg_dev->tpg_hw.waiting_request_q));
	INIT_LIST_HEAD(&(tpg_dev->tpg_hw.active_request_q));
	tpg_dev->tpg_hw.waiting_request_q_depth = 0;
	tpg_dev->tpg_hw.active_request_q_depth  = 0;
	tpg_dev->tpg_hw.settings_update         = 0;
	tpg_dev->tpg_hw.tpg_clock               = 0;
	tpg_dev->tpg_hw.hw_info->layer_init(&tpg_dev->tpg_hw);
	return 0;
}

static int cam_tpg_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc = 0;
	struct cam_tpg_device  *tpg_dev = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	tpg_dev = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_tpg_device), GFP_KERNEL);
	if (!tpg_dev) {
		CAM_ERR(CAM_TPG, "TPG dev allocation failed");
		return -ENOMEM;
	}

	strlcpy(tpg_dev->device_name, CAMX_TPG_DEV_NAME,
		sizeof(tpg_dev->device_name));
	mutex_init(&tpg_dev->mutex);
	tpg_dev->tpg_subdev.pdev = pdev;
	tpg_dev->state = CAM_TPG_STATE_INIT;
	rc = tpg_subdev_init(tpg_dev, dev);
	if (rc < 0) {
		CAM_ERR(CAM_TPG, "subdev init failed");
		goto bind_error_exit;
	}

	rc = tpg_soc_info_init(tpg_dev, dev);
	if (rc < 0) {
		CAM_ERR(CAM_TPG, "soc init failed");
		goto release_subdev;
	}

	rc = tpg_register_cpas_client(tpg_dev, dev);
	if (rc < 0) {
		CAM_ERR(CAM_TPG, "cpas register failed");
		goto release_subdev;
	}
	tpg_crm_intf_init(tpg_dev);
	rc = cam_tpg_hw_layer_init(tpg_dev, dev);
	if (rc < 0) {
		CAM_ERR(CAM_TPG, "Hw layer init failed");
		goto release_subdev;
	}

	platform_set_drvdata(pdev, tpg_dev);

	return rc;

release_subdev:
	cam_unregister_subdev(&(tpg_dev->tpg_subdev));
bind_error_exit:
	mutex_destroy(&tpg_dev->mutex);
	return rc;
}

static void cam_tpg_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cam_tpg_device  *tpg_dev = platform_get_drvdata(pdev);

	CAM_INFO(CAM_TPG, "Unbind TPG component");
	cam_cpas_unregister_client(tpg_dev->cpas_handle);
	cam_soc_util_release_platform_resource(&tpg_dev->soc_info);
	mutex_destroy(&tpg_dev->mutex);
	mutex_destroy(&tpg_dev->tpg_hw.mutex);
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&(tpg_dev->tpg_subdev.sd), NULL);
	cam_unregister_subdev(&(tpg_dev->tpg_subdev));
}

const static struct component_ops cam_tpg_component_ops = {
	.bind = cam_tpg_component_bind,
	.unbind = cam_tpg_component_unbind,
};

static int32_t cam_tpg_platform_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_TPG, "Adding TPG component");
	rc = component_add(&pdev->dev, &cam_tpg_component_ops);
	if (rc)
		CAM_ERR(CAM_TPG, "failed to add component rc: %d", rc);

	return rc;
}


static int32_t cam_tpg_device_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_tpg_component_ops);
	return 0;
}

static const struct of_device_id cam_tpg_dt_match[] = {
	{
		.compatible = "qcom,cam-tpg101",
		.data = &tpg_v_1_0_hw_info,
	},
	{
		.compatible = "qcom,cam-tpg102",
		.data = &tpg_v_1_2_hw_info,
	},
	{
		.compatible = "qcom,cam-tpg103",
		.data = &tpg_v_1_3_hw_info,
	},
	{
		.compatible = "qcom,cam-tpg1031",
		.data = &tpg_v_1_3_1_hw_info,
	},
	{
		.compatible = "qcom,cam-tpg104",
		.data = &tpg_v_1_4_hw_info,
	},
	{}
};

MODULE_DEVICE_TABLE(of, cam_tpg_dt_match);

struct platform_driver cam_tpg_driver = {
	.probe = cam_tpg_platform_probe,
	.remove = cam_tpg_device_remove,
	.driver = {
		.name = CAMX_TPG_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_tpg_dt_match,
		.suppress_bind_attrs = true,
	},
};

int32_t cam_tpg_init_module(void)
{
	CAM_DBG(CAM_TPG, "tpg module init");
	return platform_driver_register(&cam_tpg_driver);
}

void cam_tpg_exit_module(void)
{
	CAM_DBG(CAM_TPG, "tpg exit module");
	platform_driver_unregister(&cam_tpg_driver);
}

MODULE_DESCRIPTION("CAM TPG driver");
MODULE_LICENSE("GPL v2");
