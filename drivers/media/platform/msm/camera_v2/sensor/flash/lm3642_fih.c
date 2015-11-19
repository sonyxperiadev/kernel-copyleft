/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include "../cci/msm_cci.h"

#define FLASH_NAME "qcom,led-flash"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3642_i2c_driver;

static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{0x0A, 0x30},/* MM-MC-ModifiedFlashSetting-01* */
	{0x08, 0x07},
	{0x09, 0x49},/* MM-MC-ModifiedFlashSetting-00* */
};

static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
	{0x0A, 0x32},/* MM-MC-ModifiedFlashSetting-01* */
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
	{0x0A, 0x33},/* MM-MC-ModifiedFlashSetting-01* */
};

static void __exit msm_flash_lm3642_i2c_remove(void)
{
    CDBG("%s:%d \n", __func__, __LINE__);
	i2c_del_driver(&lm3642_i2c_driver);
	return;
}

static const struct of_device_id lm3642_trigger_dt_match[] = {
	{.compatible = FLASH_NAME, .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, lm3642_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

/* MM-MC-BringUpCameraFlashLM3642-01+{ */
int msm_flash_lm3642_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

#if 0
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		CDBG("%s:%d PC:: flash pins setting to active state\n",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}
#endif
	msleep(20);

    /* The Lm3642 driver IC has no I2C enable pin, but it has HW strobe and torch pin.
            Here to pull low HW strobe and torch pins. We will control Lm3642 by SW I2C. 
            So we will not use HW strobe and torch pins.*/
    /* MM-MC-ModifiedFlashSetting-01+{ */
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
    {   
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],/* GPIO98 = CAM_FLASH_TORCH */
			GPIO_OUT_LOW);//GPIO_OUT_HIGH
	    CDBG("%s:%d Init HW torch pin \n", __func__, __LINE__);
    }
	/* MM-MC-ModifiedFlashSetting-01+} */
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN] == 1)
    {   
        gpio_set_value_cansleep(
            power_info->gpio_conf->gpio_num_info->
            gpio_num[SENSOR_GPIO_FL_EN],/* GPIO97 = CAM_FLASH_STROBE */
            GPIO_OUT_LOW);//GPIO_OUT_HIGH
        CDBG("%s:%d Init HW strobe pin \n", __func__, __LINE__);
    }

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return rc;
}

int msm_flash_lm3642_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
    //int ret = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CDBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
#if 0
	if (fctrl->pinctrl_info.use_pinctrl == true) {
		ret = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (ret)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
#endif
	/* MM-MC-ModifiedFlashSetting-01+{ */

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{	
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],/* GPIO98 = CAM_FLASH_TORCH */
			GPIO_OUT_LOW);//GPIO_OUT_HIGH
		CDBG("%s:%d disable HW torch pin \n", __func__, __LINE__);
	}
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN] == 1)
	{	
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],/* GPIO97 = CAM_FLASH_STROBE */
			GPIO_OUT_LOW);//GPIO_OUT_HIGH
		CDBG("%s:%d disable HW strobe pin \n", __func__, __LINE__);
	}
	/* MM-MC-ModifiedFlashSetting-01+} */

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	return 0;
}

int msm_flash_lm3642_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CDBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	/* MM-MC-ModifiedFlashSetting-01+{ */

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
	{	
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],/* GPIO98 = CAM_FLASH_TORCH */
			GPIO_OUT_LOW);//GPIO_OUT_HIGH
		CDBG("%s:%d disable HW torch pin \n", __func__, __LINE__);
	}

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN] == 1)
	{	
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],/* GPIO97 = CAM_FLASH_STROBE */
			GPIO_OUT_LOW);//GPIO_OUT_HIGH
		CDBG("%s:%d disable HW strobe pin \n", __func__, __LINE__);
	}
	/* MM-MC-ModifiedFlashSetting-01+} */

	return rc;
}

int msm_flash_lm3642_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	/* MM-MC-ModifiedFlashSetting-01+{ */
	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_RESET] == 1)
    {   
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],/* GPIO98 = CAM_FLASH_TORCH */
			GPIO_OUT_HIGH);//GPIO_OUT_HIGH
	    CDBG("%s:%d enable HW torch pin \n", __func__, __LINE__);
    }
	/* MM-MC-ModifiedFlashSetting-01+} */
	return rc;
}

int msm_flash_lm3642_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	/* MM-MC-ModifiedFlashSetting-01+{ */

	if (power_info->gpio_conf->gpio_num_info->valid[SENSOR_GPIO_FL_EN] == 1)
	{	
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],/* GPIO97 = CAM_FLASH_STROBE */
			GPIO_OUT_HIGH);//GPIO_OUT_HIGH
		CDBG("%s:%d enable HW strobe pin \n", __func__, __LINE__);
	}
	/* MM-MC-ModifiedFlashSetting-01+} */

	return rc;
}
/* MM-MC-BringUpCameraFlashLM3642-01+} */

static int msm_flash_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
    CDBG("%s:%d \n", __func__, __LINE__);
	if (!id) {
		pr_err("msm_flash_lm3642_i2c_probe: id is NULL");
		id = lm3642_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm3642_i2c_driver = {
	.id_table = lm3642_i2c_id,
	.probe  = msm_flash_lm3642_i2c_probe,
	.remove = __exit_p(msm_flash_lm3642_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_trigger_dt_match,
	},
};

static int msm_flash_lm3642_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
    CDBG("%s:%d \n", __func__, __LINE__);
	match = of_match_device(lm3642_trigger_dt_match, &pdev->dev);
	if (!match)
    {
        CDBG("%s:%d : of_match_device() failed !\n", __func__, __LINE__);
		return -EFAULT;
    }
    CDBG("%s:%d call msm_flash_probe()\n", __func__, __LINE__);
	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver lm3642_platform_driver = {
	.probe = msm_flash_lm3642_platform_probe,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3642_trigger_dt_match,
	},
};

static int __init msm_flash_lm3642_init_module(void)
{
	int32_t rc = 0;
	rc = platform_driver_register(&lm3642_platform_driver);
    CDBG("%s:%d rc1 %d\n", __func__, __LINE__, rc);
	if (!rc)
		return rc;
	pr_debug("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&lm3642_i2c_driver);
}

static void __exit msm_flash_lm3642_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&lm3642_platform_driver);
	else
		i2c_del_driver(&lm3642_i2c_driver);
}

static struct msm_camera_i2c_client lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.high_setting = &lm3642_high_setting,
	.release_setting = &lm3642_release_setting,
};

/* MM-MC-BringUpCameraFlashLM3642-01*{ */
static struct msm_flash_fn_t lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_lm3642_led_init,
	.flash_led_release = msm_flash_lm3642_led_release,
	.flash_led_off = msm_flash_lm3642_led_off,
	.flash_led_low = msm_flash_lm3642_led_low,
	.flash_led_high = msm_flash_lm3642_led_high,
};
/* MM-MC-BringUpCameraFlashLM3642-01*} */

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3642_i2c_client,
	.reg_setting = &lm3642_regs,
	.func_tbl = &lm3642_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_lm3642_init_module);
module_exit(msm_flash_lm3642_exit_module);
MODULE_DESCRIPTION("lm3642 FLASH");
MODULE_LICENSE("GPL v2");
