/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are licensed under the License.
 */
#include "msm_sensor.h"
#define IMX134_SENSOR_NAME "imx134"
DEFINE_MSM_MUTEX(imx134_mut);

static struct msm_sensor_ctrl_t imx134_s_ctrl;

static struct msm_sensor_power_setting imx134_power_setting[] = {
#ifdef CONFIG_SONY_CAM_QCAMERA
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
#else
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
#endif
};

static struct v4l2_subdev_info imx134_subdev_info[] = {
	{
	#ifdef CONFIG_SONY_CAM_QCAMERA
		.code = V4L2_MBUS_FMT_SRGGB10_1X10,
	#else
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
	#endif
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx134_i2c_id[] = {
	{IMX134_SENSOR_NAME, (kernel_ulong_t)&imx134_s_ctrl},
	{ }
};

static int32_t msm_imx134_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx134_s_ctrl);
}

static struct i2c_driver imx134_i2c_driver = {
	.id_table = imx134_i2c_id,
	.probe  = msm_imx134_i2c_probe,
	.driver = {
		.name = IMX134_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx134_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx134_dt_match[] = {
#ifdef CONFIG_SONY_CAM_QCAMERA
	{.compatible = "qcom,imx134", .data = &imx134_s_ctrl},
#else
	{.compatible = "sne,imx134", .data = &imx134_s_ctrl},
#endif
	{}
};

MODULE_DEVICE_TABLE(of, imx134_dt_match);

static struct platform_driver imx134_platform_driver = {
	.driver = {
	#ifdef CONFIG_SONY_CAM_QCAMERA
		.name = "qcom,imx134",
	#else
		.name = "sne,imx134",
	#endif
		.owner = THIS_MODULE,
		.of_match_table = imx134_dt_match,
	},
};

static int32_t imx134_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx134_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx134_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&imx134_platform_driver,
		imx134_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx134_i2c_driver);
}

static void __exit imx134_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx134_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx134_s_ctrl);
		platform_driver_unregister(&imx134_platform_driver);
	} else {
		i2c_del_driver(&imx134_i2c_driver);
	}
	return;
}

static struct msm_sensor_ctrl_t imx134_s_ctrl = {
	.sensor_i2c_client = &imx134_sensor_i2c_client,
	.power_setting_array.power_setting = imx134_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx134_power_setting),
	.msm_sensor_mutex = &imx134_mut,
	.sensor_v4l2_subdev_info = imx134_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx134_subdev_info),
};

module_init(imx134_init_module);
module_exit(imx134_exit_module);
MODULE_DESCRIPTION("imx134");
MODULE_LICENSE("GPL v2");
