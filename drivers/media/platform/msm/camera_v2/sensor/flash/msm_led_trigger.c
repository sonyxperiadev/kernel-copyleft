/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_led_flash.h"
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
#include "msm_camera_io_util.h"
#include "../msm_sensor.h"
#include <mach/gpiomux.h>
#define CAM2_PWDN 36
#endif

#define FLASH_NAME "camera-led-flash"

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

extern int32_t msm_led_torch_create_classdev(
				struct platform_device *pdev, void *data);

static enum flash_type flashtype;
static struct msm_led_flash_ctrl_t fctrl;

static int32_t msm_led_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	CDBG("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
	return 0;
}

static int32_t msm_led_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	uint32_t i;
	uint32_t curr_l, max_curr_l;
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
	struct msm_camera_power_ctrl_t *power_info = NULL;
#endif
	CDBG("called led_state %d\n", cfg->cfgtype);

	if (!fctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
	power_info = &fctrl->flashdata->power_info;
#endif

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
		if (gpio_get_value_cansleep(CAM2_PWDN) == 0) {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_HIGH);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_LOW);
		} else {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_LOW);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_HIGH);
		}
#endif
		for (i = 0; i < fctrl->num_sources; i++)
			if (fctrl->flash_trigger[i])
				led_trigger_event(fctrl->flash_trigger[i], 0);
		if (fctrl->torch_trigger)
			led_trigger_event(fctrl->torch_trigger, 0);
		break;

	case MSM_CAMERA_LED_LOW:
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
		if (gpio_get_value_cansleep(CAM2_PWDN) == 0) {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_HIGH);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_LOW);
			CDBG("back torch mode\n");
			if (fctrl->torch_trigger) {
				max_curr_l = fctrl->torch_max_current;
				if (cfg->torch_current > 0 &&
					cfg->torch_current < max_curr_l) {
					curr_l = cfg->torch_current;
				} else {
					curr_l = fctrl->torch_op_current;
					pr_err("LED current clamped to %d\n",
						curr_l);
				}
				led_trigger_event(fctrl->torch_trigger,
					0);
				msleep(20);
				led_trigger_event(fctrl->torch_trigger,
					curr_l);
			}
		} else {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_LOW);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_HIGH);
			CDBG("front torch mode\n");
			if (fctrl->torch_trigger) {
				max_curr_l = fctrl->front_torch_max_current;
				if (cfg->torch_current > 0 &&
					cfg->torch_current < max_curr_l) {
					curr_l = cfg->torch_current;
				} else {
					curr_l = fctrl->front_torch_op_current;
					pr_err("LED current clamped to %d\n",
						curr_l);
				}
				led_trigger_event(fctrl->torch_trigger,
					0);
				msleep(20);
				led_trigger_event(fctrl->torch_trigger,
					curr_l);
			}
		}
#else
		if (fctrl->torch_trigger) {
			max_curr_l = fctrl->torch_max_current;
			if (cfg->torch_current > 0 &&
					cfg->torch_current < max_curr_l) {
				curr_l = cfg->torch_current;
			} else {
				curr_l = fctrl->torch_op_current;
				pr_err("LED current clamped to %d\n",
					curr_l);
			}
			led_trigger_event(fctrl->torch_trigger,
				curr_l);
		}
#endif
		break;

	case MSM_CAMERA_LED_HIGH:
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
		if (gpio_get_value_cansleep(CAM2_PWDN) == 0) {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_HIGH);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_LOW);
			CDBG("back flash mode\n");
			if (fctrl->torch_trigger)
				led_trigger_event(fctrl->torch_trigger, 0);
			for (i = 0; i < fctrl->num_sources; i++)
				if (fctrl->flash_trigger[i]) {
					max_curr_l =
					fctrl->flash_max_current[i];
					if (cfg->flash_current[i] > 0 &&
						cfg->flash_current[i] <
						max_curr_l) {
						curr_l = cfg->flash_current[i];
					} else {
						curr_l =
						fctrl->flash_op_current[i];
						pr_err("LED current clamped to %d\n",
						curr_l);
					}
					led_trigger_event(
					fctrl->flash_trigger[i], curr_l);
				}
		} else {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_LOW);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_HIGH);
			CDBG("front flash torch mode\n");
			if (fctrl->torch_trigger)
				led_trigger_event(fctrl->torch_trigger, 0);
			for (i = 0; i < fctrl->num_sources; i++)
				if (fctrl->flash_trigger[i]) {
					max_curr_l =
					fctrl->front_flash_max_current[i];
					if (cfg->flash_current[i] > 0 &&
						cfg->flash_current[i] <
						max_curr_l) {
						curr_l = cfg->flash_current[i];
					} else {
						curr_l = fctrl->
						front_flash_op_current[i];
						pr_err("LED current clamped to %d\n",
							curr_l);
					}
					led_trigger_event(
						fctrl->flash_trigger[i],
						curr_l);
				}
		}
#else
		if (fctrl->torch_trigger)
			led_trigger_event(fctrl->torch_trigger, 0);
		for (i = 0; i < fctrl->num_sources; i++)
			if (fctrl->flash_trigger[i]) {
				max_curr_l = fctrl->flash_max_current[i];
				if (cfg->flash_current[i] > 0 &&
						cfg->flash_current[i] < max_curr_l) {
					curr_l = cfg->flash_current[i];
				} else {
					curr_l = fctrl->flash_op_current[i];
					pr_err("LED current clamped to %d\n",
						curr_l);
				}
				led_trigger_event(fctrl->flash_trigger[i],
					curr_l);
			}
#endif
		break;

	case MSM_CAMERA_LED_INIT:
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
		if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
			pr_err("%s:%d mux install\n", __func__, __LINE__);
			msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			power_info->gpio_conf->cam_gpiomux_conf_tbl,
			power_info->gpio_conf->cam_gpiomux_conf_tbl_size);
		}

		rc = msm_camera_request_gpio_table(
			power_info->gpio_conf->cam_gpio_req_tbl,
			power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
			if (rc < 0) {
				pr_err("%s: request gpio failed\n", __func__);
				return rc;
			}
		if (gpio_get_value_cansleep(CAM2_PWDN) == 0) {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_HIGH);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_LOW);
		} else {
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_LOW);
			gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_HIGH);
		}
		for (i = 0; i < fctrl->num_sources; i++)
			if (fctrl->flash_trigger[i])
				led_trigger_event(fctrl->flash_trigger[i], 0);
		if (fctrl->torch_trigger)
			led_trigger_event(fctrl->torch_trigger, 0);
		break;
#endif
	case MSM_CAMERA_LED_RELEASE:
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_BACK],
			GPIO_OUT_LOW);
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->gpio_num
			[FLASH_GPIO_FRONT],
			GPIO_OUT_LOW);
		rc = msm_camera_request_gpio_table(
			power_info->gpio_conf->cam_gpio_req_tbl,
			power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
			if (rc < 0) {
				pr_err("%s: request gpio failed\n", __func__);
				return rc;
			}
#endif
		for (i = 0; i < fctrl->num_sources; i++)
			if (fctrl->flash_trigger[i])
				led_trigger_event(fctrl->flash_trigger[i], 0);
		if (fctrl->torch_trigger)
			led_trigger_event(fctrl->torch_trigger, 0);
		break;

	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
static int32_t msm_flash_init_gpio_pin_tbl(struct device_node *of_node,
	struct msm_camera_gpio_conf *gconf, uint16_t *gpio_array,
	uint16_t gpio_array_size)
{
	int32_t rc = 0;
	int32_t val = 0;

	gconf->gpio_num_info = kzalloc(sizeof(struct msm_camera_gpio_num_info),
		GFP_KERNEL);
	if (!gconf->gpio_num_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		return rc;
	}

	rc = of_property_read_u32(of_node, "qcom,gpio-flash-back", &val);
	if (rc < 0) {
		pr_err("%s:%d read qcom,gpio-flash-back failed rc %d\n",
			__func__, __LINE__, rc);
		goto ERROR;
	} else if (val >= gpio_array_size) {
		pr_err("%s:%d qcom,gpio-flash-back invalid %d\n",
			__func__, __LINE__, val);
		goto ERROR;
	}
	/*index 0 is for qcom,gpio-flash-back */
	gconf->gpio_num_info->gpio_num[FLASH_GPIO_BACK] =
		gpio_array[val];
	CDBG("%s qcom,gpio-flash-back %d\n", __func__,
		gconf->gpio_num_info->gpio_num[FLASH_GPIO_BACK]);

	rc = of_property_read_u32(of_node, "qcom,gpio-flash-front", &val);
	if (rc < 0) {
		pr_err("%s:%d read qcom,gpio-flash-front failed rc %d\n",
			__func__, __LINE__, rc);
		goto ERROR;
	} else if (val >= gpio_array_size) {
		pr_err("%s:%d qcom,gpio-flash-front invalid %d\n",
			__func__, __LINE__, val);
		goto ERROR;
	}
	/*index 1 is for qcom,gpio-flash-front */
	gconf->gpio_num_info->gpio_num[FLASH_GPIO_FRONT] =
		gpio_array[val];
	CDBG("%s qcom,gpio-flash-front %d\n", __func__,
		gconf->gpio_num_info->gpio_num[FLASH_GPIO_FRONT]);

	return rc;

ERROR:
	kfree(gconf->gpio_num_info);
	gconf->gpio_num_info = NULL;
	return rc;
}
#endif

static const struct of_device_id msm_led_trigger_dt_match[] = {
	{.compatible = "qcom,camera-led-flash"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_led_trigger_dt_match);

static struct platform_driver msm_led_trigger_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_led_trigger_dt_match,
	},
};

static int32_t msm_led_trigger_probe(struct platform_device *pdev)
{
	int32_t rc = 0, rc_1 = 0, i = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *flash_src_node = NULL;
	uint32_t count = 0;
	struct led_trigger *temp = NULL;
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size;
	int32_t rc_2 = 0, rc_3 = 0;
#endif

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl.pdev = pdev;
	fctrl.num_sources = 0;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node,
			"qcom,flash-type", &flashtype);
	if (rc < 0) {
		pr_err("flash-type: read failed\n");
		return -EINVAL;
	}

	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
		fctrl.num_sources = count;
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.flash_trigger_name[i]);
			if (rc < 0) {
				pr_err("default-trigger: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				fctrl.flash_trigger_name[i]);

			if (flashtype == GPIO_FLASH) {
				/* use fake current */
				fctrl.flash_op_current[i] = LED_FULL;
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.flash_op_current[i]);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.flash_max_current[i]);
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
				rc_2 = of_property_read_u32(flash_src_node,
					"qcom,front-current",
					&fctrl.front_flash_op_current[i]);
				rc_3 = of_property_read_u32(flash_src_node,
					"qcom,front-max-current",
					&fctrl.front_flash_max_current[i]);
				if ((rc < 0) || (rc_1 < 0) ||
					 (rc_2 < 0) || (rc_3 < 0)) {
					pr_err("current: read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
#else
				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
#endif
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl.flash_op_current[i]);

			led_trigger_register_simple(fctrl.flash_trigger_name[i],
				&fctrl.flash_trigger[i]);

			if (flashtype == GPIO_FLASH)
				if (fctrl.flash_trigger[i])
					temp = fctrl.flash_trigger[i];
		}

		/* Torch source */
		flash_src_node = of_parse_phandle(of_node, "qcom,torch-source",
			0);
		if (flash_src_node) {
			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.torch_trigger_name);
			if (rc < 0) {
				pr_err("default-trigger: read failed\n");
				goto torch_failed;
			}

			CDBG("default trigger %s\n",
				fctrl.torch_trigger_name);

			if (flashtype == GPIO_FLASH) {
				/* use fake current */
				fctrl.torch_op_current = LED_FULL;
				if (temp)
					fctrl.torch_trigger = temp;
				else
					led_trigger_register_simple(
						fctrl.torch_trigger_name,
						&fctrl.torch_trigger);
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.torch_op_current);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.torch_max_current);
#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
				rc_2 = of_property_read_u32(flash_src_node,
					"qcom,front-current",
					&fctrl.front_torch_op_current);
				rc_3 = of_property_read_u32(flash_src_node,
					"qcom,front-max-current",
					&fctrl.front_torch_max_current);

				if ((rc < 0) || (rc_1 < 0) ||
					(rc_2 < 0) || (rc_3 < 0)) {
					pr_err("current: read failed\n");
					goto torch_failed;
				}
#else
				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					goto torch_failed;
				}
#endif

				CDBG("torch max_current %d\n",
					fctrl.torch_op_current);

				led_trigger_register_simple(
					fctrl.torch_trigger_name,
					&fctrl.torch_trigger);
			}
torch_failed:
			of_node_put(flash_src_node);
		}
	}

#if defined(CONFIG_SONY_CAM_QCAMERA) && defined(CONFIG_MACH_SONY_WUKONG)
	fctrl.flashdata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl.flashdata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;
	power_info->gpio_conf =
			 kzalloc(sizeof(struct msm_camera_gpio_conf),
				 GFP_KERNEL);
		if (!power_info->gpio_conf) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			return rc;
		}
		gconf = power_info->gpio_conf;

		gpio_array_size = of_gpio_count(of_node);
		CDBG("%s gpio count %d\n", __func__, gpio_array_size);

		if (gpio_array_size) {
			gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
				GFP_KERNEL);
			if (!gpio_array) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				rc = -ENOMEM;
				goto ERROR1;
			}
			for (i = 0; i < gpio_array_size; i++) {
				gpio_array[i] = of_get_gpio(of_node, i);
				CDBG("%s gpio_array[%d] = %d\n", __func__, i,
					gpio_array[i]);
			}

			rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR2;
			}

			rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR3;
			}

			rc = msm_flash_init_gpio_pin_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR3;
			}
			kfree(gpio_array);
		}

		rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
		if (!rc)
			msm_led_torch_create_classdev(pdev, &fctrl);
		return rc;
ERROR3:
		kfree(gconf->cam_gpio_req_tbl);
ERROR2:
		kfree(gpio_array);
ERROR1:
		kfree(gconf);
	return rc;
#else
	rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
	if (!rc)
		msm_led_torch_create_classdev(pdev, &fctrl);

	return rc;
#endif
}

static int __init msm_led_trigger_add_driver(void)
{
	CDBG("called\n");
	return platform_driver_probe(&msm_led_trigger_driver,
		msm_led_trigger_probe);
}

static struct msm_flash_fn_t msm_led_trigger_func_tbl = {
	.flash_get_subdev_id = msm_led_trigger_get_subdev_id,
	.flash_led_config = msm_led_trigger_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &msm_led_trigger_func_tbl,
};

module_init(msm_led_trigger_add_driver);
MODULE_DESCRIPTION("LED TRIGGER FLASH");
MODULE_LICENSE("GPL v2");
