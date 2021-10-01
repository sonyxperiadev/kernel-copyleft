/*
 * Copyright 2021 Sony Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#define ENABLE_LOGE
/*#define ENABLE_LOGD*/
/*#define ENABLE_LOGI*/

#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/stat.h>
#include <linux/kthread.h>
#include <linux/thermal.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/pm_runtime.h>
#include <linux/sched/types.h>
#include <linux/i2c.h>
#include <cam_cci_dev.h>
#include "sony_camera.h"

#ifdef ENABLE_LOGE
#define LOGE(f, a...)	pr_err("%s: " f, __func__, ##a)
#else
#define LOGE(f, a...)
#endif

#ifdef ENABLE_LOGD
#define LOGD(f, a...)	pr_debug("%s: " f, __func__, ##a)
#else
#define LOGD(f, a...)
#endif

#ifdef ENABLE_LOGI
#define LOGI(f, a...)	pr_info("%s: " f, __func__, ##a)
#else
#define LOGI(f, a...)
#endif

#define SONY_CAMERA_I2C_MAX_DATA_LEN		8192
#define SONY_CAMERA_NAME_LEN			8
#define SONY_CAMERA_MCLK_DEFAULT		8000000
#define SONY_CAMERA_MAX_RETRY_COUNT		5
#define SONY_CAMERA_FRONT_SENSOR_POWER_UP_WAIT	10

#define SONY_CAMERA_I2C_MAX_LEN			(256)
#define SONY_CAMERA_I2C_MAX_SIZE		(sizeof(struct cam_sensor_i2c_reg_array) * SONY_CAMERA_I2C_MAX_LEN)

#define SONY_CAMERA_QUP_I2C_MAX_LEN		(SONY_CAMERA_I2C_MAX_LEN + SONY_CAMERA_I2C_TYPE_MAX)

#define SONY_CAMERA_DEV_NAME			"sony_camera"

#define SONY_CAMERA_PINCTRL_STATE_SLEEP		"cam_suspend"
#define SONY_CAMERA_PINCTRL_STATE_DEFAULT	"cam_default"

#define SONY_CAMERA_GPIO_RESET			"SONY_CAMERA_RESET"
#define SONY_CAMERA_GPIO_IRQ_SOF		"SONY_CAMERA_SOF"
#define SONY_CAMERA_GPIO_VANA			"SONY_CAMERA_VANA"
#define SONY_CAMERA_GPIO_IRQ_EXTERNAL		"SONY_CAMERA_EXTERNAL"

#define SONY_CAMERA_MIPI_SWITCH			"SONY_CAMERA_SWITCH"

enum sony_camera_irq_type {
	SONY_CAMERA_IRQ_SOF_EVENT			=  1,
	SONY_CAMERA_IRQ_EXTERNAL_EVENT			=  2,
	SONY_CAMERA_IRQ_EVENT_MAX			=  3,
};

enum sony_camera_state {
	SONY_CAMERA_STATE_POWER_UP,
	SONY_CAMERA_STATE_POWER_DOWN,
	SONY_CAMERA_STATE_MAX
};

struct sony_camera_seq {
	enum sony_camera_cmd	cmd;
	int			val1;
	int			val2;
	int			wait;
};

struct sony_camera_module {
	const char		*name;
	struct sony_camera_seq	*seq_on;
	struct sony_camera_seq	*seq_off;
	uint32_t		i2c_freq_mode;
};

struct sony_camera_match_id {
	bool		enabled;
	uint16_t	addr;
	uint8_t		len;
	uint16_t	expect_value;
};

struct sony_camera_info {
	uint16_t			i2c_addr;
	uint16_t			slave_addr;
	struct sony_camera_match_id	match_id;
	uint16_t			thremal_enable;
	int				modules_num;
	struct sony_camera_module	*modules;
};

struct sony_camera_event_list_data {
	struct list_head		list;
	struct sony_camera_event_data	event_data;
};

struct sony_camera_event_sof_data32 {
	uint32_t		sof_count;
	struct compat_timeval	mono_timestamp;
};

struct sony_camera_event32 {
	uint32_t	type;
	struct sony_camera_event_sof_data32	sof_data;
};

struct sony_camera_qup_i2c_device {
	uint8_t		enabled;
	struct i2c_client	*client;
	uint8_t		*i2c_data;
	struct mutex	command_lock;
};

struct sony_camera_data {
	// platform device
	uint32_t			id;
	struct platform_device		*p_dev;
	struct sony_camera_module	*module;
	bool				probe_done;
	// CCI
	struct cam_sensor_cci_client	cci_info;
	char				*i2c_data;
	// pin controll
	struct pinctrl			*pinctrl;
	struct pinctrl_state		*gpio_state_active;
	struct pinctrl_state		*gpio_state_suspend;
	struct gpio			*gpio_req_tbl;
	uint8_t				gpio_req_tbl_size;
	bool				gpio_requested;
	bool				has_hw_sof;
	bool				has_hw_ext_irq;
	// power
	struct regulator		*cam_vdig;
	struct regulator		*cam_vio;
	struct regulator		*cam_vana;
	struct regulator		*cam_vaf;
	struct regulator		*cam_vaf2;
	struct regulator		*cam_ois;
	struct regulator		*cam_ois2;
	struct regulator		*cam_clk;
	struct regulator		*cam_vmdr;
	struct regulator		*cam_vdig2;
	struct regulator		*pmic_vin;
	struct regulator		*cam_pmic0;
	struct regulator		*cam_pmic1;
	struct regulator		*cam_gyro;
	struct clk			*clk_handle;
	struct mutex			state_lock;
	enum sony_camera_state		state;
	bool				enable_cam_vdig_by_user;
	bool				enable_cam_vio_by_user;
	bool				enable_cam_vana_by_user;
	bool				enable_cam_vaf_by_user;
	bool				enable_cam_clk_by_user;
	bool				enable_cam_vmdr_by_user;
	bool				enable_cam_vdig2_by_user;
	// thermal
	struct mutex			thermal_lock;
	struct thermal_zone_device	*thermal_zone_dev[SONY_CAMERA_MAX_TEMPERATURE_NUM_PER_CAMERA];
	int32_t				thermal_sensor_temperature[SONY_CAMERA_MAX_TEMPERATURE_NUM_PER_CAMERA];
	int				thermal_ret_val;
	// Sensor SOF count
	spinlock_t			sof_lock;
	uint32_t			sof_count;
	// User event
	spinlock_t			event_list_lock;
	wait_queue_head_t		event_wait_q;
	struct list_head		event_available;
	// Kernel event
	struct mutex			command_lock;
	uint32_t			open_count;
	// camera sub device on qup
	struct sony_camera_qup_i2c_device	qup_I2C_dev;
	// i2c_client for External Camera
	uint16_t			use_qup;
	struct i2c_client		*qup_client;
};

static int dev_id;
static struct cdev cdev;
static struct class *c = NULL;
static uint16_t sensor_num;
static struct platform_device *camera_device;
static struct class *camera_device_class;
struct i2c_client *ext_cam_client = NULL;

static struct sony_camera_info camera_info[] = {
	{
	},
	{
	},
	{
	},
	{
	},
	{
	},
	{
	},
};

static struct sony_camera_data camera_data[] = {
	{
		.id = 0,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
	},
	{
		.id = 1,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
	},
	{
		.id = 2,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
	},
	{
		.id = 3,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
	},
	{
		.id = 4,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
	},
	{
		.id = 5,
		.thermal_sensor_temperature = { 0, 0, 0 },
		.thermal_ret_val = -ENODEV,
		.has_hw_sof = 0,
	},
};


struct sony_camera_driver_combination_info {
	uint32_t id;
	uint32_t thermal_zone;
	char thermal_name[32];
};

// TODO: Add thermal name on dtsi-file.
static struct sony_camera_driver_combination_info driver_comb_tbl[] = {
	{
		.id = 0,
		.thermal_zone = 0,
		.thermal_name = "sony_cam_back0",
	},
	{
		.id = 1,
		.thermal_zone = 0,
		.thermal_name = "sony_cam_back1",
	},
	{
		.id = 1,
		.thermal_zone = 1,
		.thermal_name = "sony_cam_back1_ext",
	},
	{
		.id = 2,
		.thermal_zone = 0,
		.thermal_name = "sony_cam_front0",
	},
	{
		.id = 3,
		.thermal_zone = 0,
		.thermal_name = "sony_cam_back2",
	},
	{
		.id = 4,
		.thermal_zone = 0,
		.thermal_name = "sony_cam_tof",
	},
	{
		.id = 4,
		.thermal_zone = 1,
		.thermal_name = "sony_cam_tof_laser",
	},
};

static int sony_camera_set_power(struct sony_camera_data*, int, bool);

static int sony_camera_get_dt_gpio_req_tbl(struct device_node *of_node,
	struct sony_camera_data *camera_data)
{
	int32_t rc = 0, i = 0;
	uint32_t count = 0;
	uint16_t *gpio_array = NULL;
	int16_t gpio_array_size = 0;
	uint32_t *val_array = NULL;

	gpio_array_size = of_gpio_count(of_node);

	if (gpio_array_size <= 0)
		return 0;

	LOGI("%s %d gpio count %d\n", __func__, __LINE__, gpio_array_size);

	gpio_array = vzalloc(gpio_array_size * sizeof(uint16_t));
	if (!gpio_array)
		goto exit;

	for (i = 0; i < gpio_array_size; i++)
		gpio_array[i] = of_get_gpio(of_node, i);

	if (!of_get_property(of_node, "gpio-req-tbl-num", &count))
		return 0;

	count /= sizeof(uint32_t);
	if (!count) {
		LOGE("%s %d gpio-req-tbl-num 0\n", __func__, __LINE__);
		return 0;
	}

	val_array = vzalloc(count * sizeof(uint32_t));
	if (!val_array) {
		vfree(gpio_array);
		return -ENOMEM;
	}

	camera_data->gpio_req_tbl = vzalloc(count * sizeof(struct gpio));
	if (!camera_data->gpio_req_tbl) {
		rc = -ENOMEM;
		goto free_val_array;
	}
	camera_data->gpio_req_tbl_size = count;

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-num",
		val_array, count);
	if (rc) {
		LOGE("%s %d failed in reading gpio-req-tbl-num, rc = %d\n",
			__func__, __LINE__, rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++) {
		if (val_array[i] >= gpio_array_size) {
			LOGE("%s %d gpio req tbl index %d invalid\n",
				__func__, __LINE__, val_array[i]);
			goto free_gpio_req_tbl;
		}
		camera_data->gpio_req_tbl[i].gpio = gpio_array[val_array[i]];
	}

	rc = of_property_read_u32_array(of_node, "gpio-req-tbl-flags",
		val_array, count);
	if (rc) {
		LOGE("%s %d Failed in gpio-req-tbl-flags, rc %d\n",
			__func__, __LINE__, rc);
		goto free_gpio_req_tbl;
	}

	for (i = 0; i < count; i++)
		camera_data->gpio_req_tbl[i].flags = val_array[i];

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node,
			"gpio-req-tbl-label", i,
			&camera_data->gpio_req_tbl[i].label);
		if (rc) {
			LOGE("%s %d Failed rc %d\n", __func__, __LINE__, rc);
			goto free_gpio_req_tbl;
		}
	}

	vfree(val_array);
	vfree(gpio_array);

	return rc;

free_gpio_req_tbl:
	vfree(camera_data->gpio_req_tbl);
free_val_array:
	vfree(val_array);
	camera_data->gpio_req_tbl_size = 0;
	vfree(gpio_array);
exit:

	return rc;
}

static struct gpio *sony_camera_get_gpio_pin(
	struct sony_camera_data *camera_data, const char *label)
{
	uint32_t i = 0;
	uint32_t size = 0;
	struct gpio *gpio_tbl = NULL;

	if (camera_data == NULL || label == NULL) {
		LOGE("%s:%d invalid args:%p, %p\n", __func__,
			__LINE__, camera_data, label);
		return NULL;
	}
	gpio_tbl = camera_data->gpio_req_tbl;
	size = camera_data->gpio_req_tbl_size;
	for (i = 0; i < size; i++) {
		if (!strcmp(label, gpio_tbl->label))
			return gpio_tbl;
		gpio_tbl++;
	}
	return NULL;
}

static int sony_camera_info_deinit(uint32_t id)
{
	uint16_t i = 0;

	if (camera_info[id].modules) {
		for (i = 0; i < camera_info[id].modules_num; i++) {
			vfree(camera_info[id].modules[i].seq_on);
			vfree(camera_info[id].modules[i].seq_off);
		}
		vfree(camera_info[id].modules);
	}
	memset(&(camera_info[id]), 0, sizeof(struct sony_camera_info));

	return 0;
}

static int sony_camera_info_init(struct platform_device *p_dev,
	uint32_t id)
{
	int rc = 0;
	int count = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	uint32_t val_u32[4] = {0};
	struct device_node *of_node = p_dev->dev.of_node;
	struct device_node *of_node_modules = NULL;
	struct device_node *of_node_modules_power_off = NULL;
	struct device_node *of_node_modules_power_on = NULL;
	const int8_t *power_order_name = NULL;

	rc = of_property_read_u32(of_node, "slave_addr", &val_u32[0]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}

	camera_info[id].slave_addr = val_u32[0];

	rc = of_property_read_u32_array(of_node, "match_id", &val_u32[0], 3);
	if (rc < 0) {
		camera_info[id].match_id.enabled = false;
	} else {
		camera_info[id].match_id.enabled = true;
		camera_info[id].match_id.addr = val_u32[0];
		camera_info[id].match_id.len = val_u32[1];
		camera_info[id].match_id.expect_value = val_u32[2];
	}

	rc = of_property_read_u32(of_node, "cci-device", &val_u32[0]);
	if (rc < 0) {
		camera_data[id].cci_info.cci_device = CCI_DEVICE_0;
	} else {
		camera_data[id].cci_info.cci_device = val_u32[0];
	}

	rc = of_property_read_u32(of_node, "cci-master", &val_u32[0]);
	if (rc < 0) {
		camera_data[id].cci_info.cci_i2c_master = MASTER_0;
	} else {
		camera_data[id].cci_info.cci_i2c_master = val_u32[0];
	}

	rc = of_property_read_u32(of_node, "use_qup", &val_u32[0]);
	if (rc < 0) {
		camera_data[id].use_qup = 0;
	} else {
		camera_data[id].use_qup = val_u32[0];
	}

	rc = of_property_read_u32(of_node, "thremal_enable", &val_u32[0]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}
	camera_info[id].thremal_enable = val_u32[0];

	rc = sony_camera_get_dt_gpio_req_tbl(of_node, &camera_data[id]);
	if (rc < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		goto fail;
	}

	if (sony_camera_get_gpio_pin(&camera_data[id],
		SONY_CAMERA_GPIO_IRQ_SOF) == NULL) {
		camera_data[id].has_hw_sof = 0;
	} else {
		camera_data[id].has_hw_sof = 1;
	}

	if (sony_camera_get_gpio_pin(&camera_data[id],
		SONY_CAMERA_GPIO_IRQ_EXTERNAL) == NULL) {
		camera_data[id].has_hw_ext_irq = 0;
	} else {
		camera_data[id].has_hw_ext_irq = 1;
	}

	count = of_property_count_strings(of_node, "module_names");
	if (count < 0) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		rc = -EFAULT;
		goto fail;
	}
	camera_info[id].modules_num = count;

	camera_info[id].modules = vzalloc(sizeof(struct sony_camera_module) * count);
	if (!camera_info[id].modules) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < camera_info[id].modules_num; i++) {
		rc = of_property_read_string_index(of_node,
			"module_names", i,
			(const char **)(&camera_info[id].modules[i].name));
		LOGD("%s name[%d] = %s\n", __func__, i,
			camera_info[id].modules[i].name);
		if (rc < 0) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			goto fail;
		}
	}

	for (i = 0; i < camera_info[id].modules_num; i++) {
		of_node_modules = of_find_node_by_name(of_node,
			camera_info[id].modules[i].name);
		if (!of_node_modules) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -EFAULT;
			goto fail;
		}

		rc = of_property_read_u32(of_node_modules,
			"i2c_freq_mode",
			&camera_info[id].modules[i].i2c_freq_mode);

		if (rc < 0) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			goto fail;
		}

		of_node_modules_power_off = of_find_node_by_name(
			of_node_modules, "power_off");
		if (!of_node_modules_power_off) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -EFAULT;
			goto fail;
		}

		count = of_property_count_strings(of_node_modules_power_off,
			"commands");
		if (count < 0) {
			LOGE("%s failed power off commands 0\n", __func__);
			rc = -EFAULT;
			goto fail;
		}
		camera_info[id].modules[i].seq_off = vzalloc(sizeof(struct sony_camera_seq) * count);
		if (!camera_info[id].modules[i].seq_off) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto fail;
		}

		for (j = 0; j < count; j++) {
			rc = of_property_read_string_index(
				of_node_modules_power_off,
				"commands", j,
				(const char **)(&power_order_name));
			if (rc < 0) {
				LOGE("%s failed %d\n", __func__, __LINE__);
				goto fail;
			}

			rc = of_property_read_u32_array(
				of_node_modules_power_off, power_order_name,
				&val_u32[0], 4);
			if (rc < 0) {
				LOGE("%s failed %d\n", __func__, __LINE__);
				goto fail;
			}
			camera_info[id].modules[i].seq_off[j].cmd = val_u32[0];
			camera_info[id].modules[i].seq_off[j].val1 = val_u32[1];
			camera_info[id].modules[i].seq_off[j].val2 = val_u32[2];
			camera_info[id].modules[i].seq_off[j].wait = val_u32[3];
		}

		of_node_modules_power_on = of_find_node_by_name(of_node_modules,
			"power_on");
		if (!of_node_modules_power_on) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -EFAULT;
			goto fail;
		}

		count = of_property_count_strings(of_node_modules_power_on,
			"commands");
		if (count < 0) {
			LOGE("%s failed power on commands 0\n", __func__);
			rc = -EFAULT;
			goto fail;
		}

		camera_info[id].modules[i].seq_on = vzalloc(sizeof(struct sony_camera_seq) * count);
		if (!camera_info[id].modules[i].seq_on) {
			LOGE("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto fail;
		}

		for (j = 0; j < count; j++) {
			rc = of_property_read_string_index(
				of_node_modules_power_on,
				"commands", j,
				(const char **)(&power_order_name));
			if (rc < 0) {
				LOGE("%s failed %d j=%d count=%d\n", __func__, __LINE__, j, count);
				goto fail;
			}

			rc = of_property_read_u32_array(
				of_node_modules_power_on, power_order_name,
				&val_u32[0], 4);
			if (rc < 0) {
				LOGE("%s failed %d j=%d count=%d\n", __func__, __LINE__, j, count);
				goto fail;
			}
			camera_info[id].modules[i].seq_on[j].cmd = val_u32[0];
			camera_info[id].modules[i].seq_on[j].val1 = val_u32[1];
			camera_info[id].modules[i].seq_on[j].val2 = val_u32[2];
			camera_info[id].modules[i].seq_on[j].wait = val_u32[3];
		}
	}

	return 0;
fail:
	sony_camera_info_deinit(i);

	return rc;
}

static int sony_camera_gpio_init(struct sony_camera_data *data)
{
	int rc = 0;
	uint16_t i = 0;
	struct gpio *gpio_tbl = data->gpio_req_tbl;
	uint8_t size = data->gpio_req_tbl_size;

	if (!data->gpio_requested) {
		for (i = 0; i < size; i++) {
			rc = gpio_request_one(gpio_tbl[i].gpio,
				gpio_tbl[i].flags, gpio_tbl[i].label);
			if (rc) {
				// TODO: After GPIO request fails, contine to apply new gpios,
				// outout a error message for driver bringup debug
				LOGE("%s:%d gpio %d:%s request fails\n",
					__func__, __LINE__,
					gpio_tbl[i].gpio, gpio_tbl[i].label);
			}
		}
		if (rc == 0)
			data->gpio_requested = true;
	}
	return rc;
}

static int sony_camera_gpio_deinit(struct sony_camera_data *data)
{
	int rc = 0;
	struct gpio *gpio_tbl = data->gpio_req_tbl;
	uint8_t size = data->gpio_req_tbl_size;

	if (data->gpio_requested) {
		gpio_free_array(gpio_tbl, size);
		data->gpio_requested = false;
	}
	return rc;
}

static int sony_camera_gpio_set(struct sony_camera_data *data,
	int gpio_pin, int value)
{
	int rc = 0;

	if (data->gpio_requested)
		gpio_set_value_cansleep(gpio_pin, value);
	else
		rc = -EPERM;

	return rc;
}

static int sony_camera_regist_gpio_irq(
	struct sony_camera_data *camera_data, irq_handler_t handler,
	unsigned int flags, const char *label)
{
	int irq;
	int rc = 0;
	struct gpio *irq_gpio = NULL;

	irq_gpio = sony_camera_get_gpio_pin(camera_data, label);
	if (!irq_gpio) {
		LOGE("%s:%d Sony sensor: can't find gpio irq:%s\n",
			__func__, __LINE__, label);
		return -EINVAL;
	}
	irq = gpio_to_irq(irq_gpio->gpio);
	rc = request_irq(irq, handler, flags, irq_gpio->label, camera_data);
	if (rc) {
		LOGE("%s:%d Sony sensor: can't regist gpio irq:%s\n",
			__func__, __LINE__, label);
		return -EINVAL;
	}
	LOGD("%s regist gpio %d for %s\n", __func__, irq_gpio->gpio, label);
	return 0;
}

static int sony_camera_unregist_gpio_irq(
	struct sony_camera_data *camera_data, const char *label)
{
	int irq;
	struct gpio *irq_gpio = NULL;

	irq_gpio = sony_camera_get_gpio_pin(camera_data, label);
	if (!irq_gpio) {
		LOGE("%s:%d Sony sensor: can't find gpio irq:%s\n",
			__func__, __LINE__, label);
		return -EINVAL;
	}
	irq = gpio_to_irq(irq_gpio->gpio);
	disable_irq(irq);
	free_irq(irq, camera_data);
	LOGD("%s unregist gpio %d for %s\n", __func__, irq_gpio->gpio, label);
	return 0;
}

static void sony_camera_send_event(struct sony_camera_data *data,
	const struct sony_camera_event_data *sensor_event) {
	unsigned long flags;

	struct sony_camera_event_list_data *event = kzalloc(
		sizeof(struct sony_camera_event_list_data), GFP_ATOMIC);
	if (!event) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		return;
	}
	LOGI("sony_camera_send_event type 0x%x\n", sensor_event->type);
	memcpy(&event->event_data, sensor_event,
		sizeof(struct sony_camera_event_data));
	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	list_add_tail(&event->list, &data->event_available);
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	wake_up_interruptible(&data->event_wait_q);
}

static irqreturn_t sony_camera_irq_handler(int irq, void *info)
{
	struct irq_desc *desc = NULL;
	struct sony_camera_data *camera_data = NULL;

	camera_data = (struct sony_camera_data *)info;
	if (!camera_data) {
		LOGE("%s:%d err camera_data is NULL\n", __func__, __LINE__);
		goto exit;
	}
	desc = irq_to_desc(irq);
	if (desc != NULL) {
		if (!strcmp(desc->action->name, SONY_CAMERA_GPIO_IRQ_SOF)) {
			struct sony_camera_event_data camera_event;
			struct timespec64 ts;
			unsigned long sof_lock_flags;

			ktime_get_boottime_ts64(&ts);
			memset(&camera_event, 0, sizeof(camera_event));
			spin_lock_irqsave(&camera_data->sof_lock, sof_lock_flags);
			camera_data->sof_count++;
			if (camera_data->sof_count > 0xFFFFFFF0)
				camera_data->sof_count = 1;
			camera_event.sof_data.sof_count = camera_data->sof_count;
			spin_unlock_irqrestore(&camera_data->sof_lock, sof_lock_flags);
			camera_event.type = SONY_CAMERA_EVT_SOF;
			camera_event.sof_data.mono_timestamp.tv_sec = ts.tv_sec;
			camera_event.sof_data.mono_timestamp.tv_usec = ts.tv_nsec / 1000;
			LOGI("%s:%d sof_count = %d\n",
				__func__, __LINE__, camera_data->sof_count);
			sony_camera_send_event(camera_data, &camera_event);
			return IRQ_HANDLED;
		} else if (!strcmp(desc->action->name, SONY_CAMERA_GPIO_IRQ_EXTERNAL)) {
			struct sony_camera_event_data camera_event;
			memset(&camera_event, 0, sizeof(camera_event));
			camera_event.type = SONY_CAMERA_EVT_EXTERNAL;
			sony_camera_send_event(camera_data, &camera_event);
			return IRQ_HANDLED;
		}
	}

exit:
	return IRQ_HANDLED;
}

static int sony_camera_irq_init(
	struct sony_camera_data *camera_data,
	struct sony_camera_info *camera_info)
{
	int rc = 0;

	if (camera_data->has_hw_sof) {
		rc = sony_camera_regist_gpio_irq(camera_data,
			sony_camera_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			SONY_CAMERA_GPIO_IRQ_SOF);
		if (rc < 0)
			LOGE("regist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_SOF);
	}
	if (camera_data->has_hw_ext_irq) {
		rc = sony_camera_regist_gpio_irq(camera_data,
			sony_camera_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			SONY_CAMERA_GPIO_IRQ_EXTERNAL);
		if (rc < 0)
			LOGE("regist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_EXTERNAL);
	}

	return rc;
}

static int sony_camera_irq_deinit(struct sony_camera_data *camera_data,
	struct sony_camera_info *camera_info)
{
	int rc = 0;

	if (camera_data->has_hw_sof) {
		rc = sony_camera_unregist_gpio_irq(camera_data,
			SONY_CAMERA_GPIO_IRQ_SOF);
		if (rc < 0)
			LOGE("unregist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_SOF);
	}
	if (camera_data->has_hw_ext_irq) {
		rc = sony_camera_unregist_gpio_irq(camera_data,
			SONY_CAMERA_GPIO_IRQ_EXTERNAL);
		if (rc < 0)
			LOGE("unregist gpio irq: %s failed\n",
				SONY_CAMERA_GPIO_IRQ_EXTERNAL);
	}
	return rc;
}

static int sony_camera_vreg_set(struct sony_camera_data *data,
	enum sony_camera_cmd cmd, int level, int op_mode)
{
	int rc = 0;
	struct regulator *vreg = NULL;
	struct device *dev = &data->p_dev->dev;

	if (cmd == SONY_CAM_VDIG) {
		if (data->cam_vdig) {
			vreg = data->cam_vdig;
		} else {
			vreg = regulator_get(dev, "cam_vdig");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vdig, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vdig = vreg;
		}
	} else if (cmd == SONY_CAM_VIO) {
		if (data->cam_vio) {
			vreg = data->cam_vio;
		} else {
			vreg = regulator_get(dev, "cam_vio");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vio, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vio = vreg;
		}
	} else if (cmd == SONY_CAM_VANA) {
		if (data->cam_vana) {
			vreg = data->cam_vana;
		} else {
			vreg = regulator_get(dev, "cam_vana");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vana, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vana = vreg;
		}
	} else if (cmd == SONY_CAM_VAF) {
		if (data->cam_vaf) {
			vreg = data->cam_vaf;
		} else {
			vreg = regulator_get(dev, "cam_vaf");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vaf, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vaf = vreg;
		}
	} else if (cmd == SONY_CAM_VAF2) {
		if (data->cam_vaf2) {
			vreg = data->cam_vaf2;
		} else {
			vreg = regulator_get(dev, "cam_vaf2");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vaf2, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vaf2 = vreg;
		}
	} else if (cmd == SONY_CAM_OIS) {
		if (data->cam_ois) {
			vreg = data->cam_ois;
		} else {
			vreg = regulator_get(dev, "cam_ois");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_ois, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_ois = vreg;
		}
	} else if (cmd == SONY_CAM_OIS2) {
		if (data->cam_ois2) {
			vreg = data->cam_ois2;
		} else {
			vreg = regulator_get(dev, "cam_ois2");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_ois2, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_ois2 = vreg;
		}
	} else if (cmd == SONY_CAM_CLK) {
		if (data->cam_clk) {
			vreg = data->cam_clk;
		} else {
			vreg = regulator_get(dev, "cam_clk");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_clk, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_clk = vreg;
		}
	} else if (cmd == SONY_CAM_VMDR) {
		if (data->cam_vmdr) {
			vreg = data->cam_vmdr;
		} else {
			vreg = regulator_get(dev, "cam_vmdr");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vmdr, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vmdr = vreg;
		}
	} else if (cmd == SONY_CAM_GYRO) {
		if (data->cam_gyro) {
			vreg = data->cam_gyro;
		} else {
			vreg = regulator_get(dev, "cam_gyro");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_gyro, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_gyro = vreg;
		}
	} else if (cmd == SONY_CAM_VDIG2) {
		if (data->cam_vdig2) {
			vreg = data->cam_vdig2;
		} else {
			vreg = regulator_get(dev, "cam_vdig2");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_vdig2, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_vdig2 = vreg;
		}
	} else if (cmd == SONY_CAM_PMIC0) {
		if (data->cam_pmic0) {
			vreg = data->cam_pmic0;
		} else {
			vreg = regulator_get(dev, "cam_pmic0");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_pmic0, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_pmic0 = vreg;
		}
	} else if (cmd == SONY_CAM_PMIC1) {
		if (data->cam_pmic1) {
			vreg = data->cam_pmic1;
		} else {
			vreg = regulator_get(dev, "cam_pmic1");
			if (IS_ERR(vreg)) {
				LOGE("could not get cam_pmic1, vreg = %ld\n",
					PTR_ERR(vreg));
				rc = -ENODEV;
				goto exit;
			}
			data->cam_pmic1 = vreg;
		}
	} else {
		rc = -EINVAL;
		LOGE("invalid resource\n");
		goto exit;
	}

	level *= 1000;
	if (level >= 0) {
		if (level > 0) {
			rc = regulator_set_voltage(vreg, level, level);
			if (rc < 0)
				goto set_voltage_fail;
		}
		if (op_mode > 0) {
			rc = regulator_set_load(vreg, op_mode);
			if (rc < 0)
				goto set_voltage_fail;
		}
		rc = regulator_enable(vreg);
		if (rc < 0)
			goto enable_fail;
	} else {
		if (op_mode == 0)
			(void)regulator_set_load(vreg, 0);
		(void)regulator_disable(vreg);
		regulator_put(vreg);
	}
	goto exit;

enable_fail:
	(void)regulator_set_load(vreg, 0);
set_voltage_fail:
	regulator_put(vreg);
exit:
	if (rc < 0 || level < 0) {
		if (vreg == data->cam_vdig)
			data->cam_vdig = NULL;
		else if (vreg == data->cam_vio)
			data->cam_vio = NULL;
		else if (vreg == data->cam_vana)
			data->cam_vana = NULL;
		else if (vreg == data->cam_vaf)
			data->cam_vaf = NULL;
		else if (vreg == data->cam_vaf2)
			data->cam_vaf2 = NULL;
		else if (vreg == data->cam_ois)
			data->cam_ois = NULL;
		else if (vreg == data->cam_ois2)
			data->cam_ois2 = NULL;
		else if (vreg == data->cam_vdig2)
			data->cam_vdig2 = NULL;
		else if (vreg == data->cam_pmic0)
			data->cam_pmic0 = NULL;
		else if (vreg == data->cam_pmic1)
			data->cam_pmic1 = NULL;
		else if (vreg == data->cam_gyro)
			data->cam_gyro = NULL;
		else if (vreg == data->cam_clk)
			data->cam_clk = NULL;
		else if (vreg == data->cam_vmdr)
			data->cam_vmdr = NULL;
	}

	if (rc < 0)
		LOGE("error happened (%d)\n", rc);
	return rc;
}

static int sony_camera_mclk_set(struct sony_camera_data *data, int value)
{
	int rc = 0;
	int clk_rate = 0;
	struct device *dev = &data->p_dev->dev;
	struct clk *clk_handle = data->clk_handle;

	if (value >= 0) {
		clk_handle = clk_get(dev, "cam_clk");
		if (clk_handle == NULL) {
			LOGE("%s line %d get clk failed\n", __func__, __LINE__);
			rc = -EINVAL;
			goto fail_get;
		}
		clk_rate = clk_round_rate(clk_handle,
			value ? value : SONY_CAMERA_MCLK_DEFAULT);
		if (clk_rate < 0) {
			LOGE("%s line %d %s round failed\n", __func__, __LINE__, "cam_clk");
			goto fail_set;
		}
		rc = clk_set_rate(clk_handle, clk_rate);
		if (rc < 0) {
			LOGE("%s line %d %s set failed\n", __func__, __LINE__, "cam_clk");
			goto fail_set;
		}
		rc = clk_prepare_enable(clk_handle);
		if (rc < 0) {
			LOGE("%s line %d %s enable failed\n", __func__, __LINE__, "cam_clk");
			goto fail_prepare_enable;
		}
		data->clk_handle = clk_handle;
	} else {
		if (clk_handle != NULL) {
			clk_disable_unprepare(clk_handle);
			clk_put(clk_handle);
			data->clk_handle = NULL;
		}
	}

	if (rc < 0)
		LOGE("error happened (%d)\n", rc);
	return rc;

fail_prepare_enable:
	clk_disable_unprepare(clk_handle);
fail_set:
	clk_put(clk_handle);
fail_get:
	return rc;
}

static int sony_camera_cci_init(struct sony_camera_data *data)
{
	int rc = 0;
	struct cam_cci_ctrl cci_ctrl;

	memset(&cci_ctrl, 0, sizeof(cci_ctrl));
	cci_ctrl.cmd = MSM_CCI_INIT;
	cci_ctrl.cci_info = &data->cci_info;
	rc = v4l2_subdev_call(data->cci_info.cci_subdev,
		core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		LOGE("%s line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	rc = cci_ctrl.status;
	return rc;
}

static int sony_camera_cci_deinit(struct sony_camera_data *data)
{
	int rc = 0;
	struct cam_cci_ctrl cci_ctrl;

	memset(&cci_ctrl, 0, sizeof(cci_ctrl));
	cci_ctrl.cmd = MSM_CCI_RELEASE;
	cci_ctrl.cci_info = &data->cci_info;
	rc = v4l2_subdev_call(data->cci_info.cci_subdev,
		core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		LOGE("%s line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	rc = cci_ctrl.status;
	return rc;
}

static int sony_camera_qup_i2c_read(struct sony_camera_data *data,
	uint8_t slave_addr, uint32_t addr, uint8_t addr_type,
	int size, uint8_t *buf)
{
	int rc;
	struct i2c_client *client = data->qup_I2C_dev.client;
	uint8_t w_buf[4];
	struct i2c_msg msg[2];

	mutex_lock(&data->qup_I2C_dev.command_lock);

	if (!data->qup_I2C_dev.enabled) {
		LOGE("%s %d device not enabled\n", __func__, __LINE__);
		rc = -EINVAL;
		goto end;
	}
	if (client->addr != slave_addr) {
		LOGE("%s %d no selected device found\n", __func__, __LINE__);
		rc = -ENODEV;
		goto end;
	}
	if (!buf) {
		LOGE("%s %d no memory\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto end;
	}
	if (addr_type == SONY_CAMERA_I2C_TYPE_BYTE) {
		w_buf[0] = (uint8_t)addr;
	} else if (addr_type == SONY_CAMERA_I2C_TYPE_WORD) {
		w_buf[0] = (uint8_t)(addr >> 8);
		w_buf[1] = (uint8_t)(addr & 0xFF);
	} else {
		w_buf[0] = (uint8_t)(addr >> 24);
		w_buf[1] = (uint8_t)(addr >> 16);
		w_buf[2] = (uint8_t)(addr >> 8);
		w_buf[3] = (uint8_t)(addr & 0xFF);
	}

	msg[0].addr = (client->addr) >> 1;
	msg[0].flags = 0;
	msg[0].buf = w_buf;
	msg[0].len = addr_type;

	msg[1].addr = (client->addr) >> 1;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = size;
	rc = i2c_transfer(client->adapter, msg, 2);
	if (rc != 2) {
		LOGE("%s %d i2c_transfer fail\n", __func__, __LINE__);
		rc = -EFAULT;
	}

end:
	mutex_unlock(&data->qup_I2C_dev.command_lock);
	return rc;
}

static int sony_camera_qup_i2c_write(struct sony_camera_data *data,
	uint8_t slave_addr, uint32_t addr, uint8_t addr_type,
	int size, uint8_t *buf)
{
	int rc;
	struct i2c_client *client = data->qup_I2C_dev.client;
	struct i2c_msg msg[2];
	uint8_t *w_buf = NULL;
	uint8_t need_free = 0;
	uint16_t totol_size = size + addr_type;

	mutex_lock(&data->qup_I2C_dev.command_lock);

	if (!data->qup_I2C_dev.enabled) {
		LOGE("%s %d device not enabled\n", __func__, __LINE__);
		rc = -EINVAL;
		goto end;
	}
	if (client->addr != slave_addr) {
		LOGE("%s %d no selected device found\n", __func__, __LINE__);
		rc = -ENODEV;
		goto end;
	}
	if (!buf) {
		LOGE("%s %d no memory\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto end;
	}
	if (totol_size > SONY_CAMERA_QUP_I2C_MAX_LEN) {
		w_buf = vzalloc(totol_size);
		need_free = true;
	} else {
		memset(data->qup_I2C_dev.i2c_data, 0, SONY_CAMERA_QUP_I2C_MAX_LEN);
		w_buf = data->qup_I2C_dev.i2c_data;
	}

	if (!w_buf) {
		LOGE("%s %d no memory\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto end;
	}
	if (addr_type == SONY_CAMERA_I2C_TYPE_BYTE) {
		w_buf[0] = (uint8_t)addr;
	} else if (addr_type == SONY_CAMERA_I2C_TYPE_WORD) {
		w_buf[0] = (uint8_t)(addr >> 8);
		w_buf[1] = (uint8_t)(addr & 0xFF);
	} else {
		w_buf[0] = (uint8_t)(addr >> 24);
		w_buf[1] = (uint8_t)(addr >> 16);
		w_buf[2] = (uint8_t)(addr >> 8);
		w_buf[3] = (uint8_t)(addr & 0xFF);
	}
	memcpy(&w_buf[addr_type], buf, size);

	msg[0].addr = (client->addr) >> 1;
	msg[0].flags = 0;
	msg[0].buf = w_buf;
	msg[0].len = totol_size;
	rc = i2c_transfer(client->adapter, msg, 1);
	if (rc != 1) {
		LOGE("%s %d i2c_transfer fail\n", __func__, __LINE__);
		rc = -EFAULT;
	}

end:
	if (need_free)
		vfree(w_buf);
	mutex_unlock(&data->qup_I2C_dev.command_lock);
	return rc;
}

static int sony_camera_i2c_read(struct sony_camera_data *data,
			uint8_t slave_addr, uint32_t addr,
			uint8_t type, uint16_t len, uint8_t *buf)
{
	int rc = 0;
	int i = 0;
	int retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;
	uint8_t read_data[SONY_CAMERA_MAX_I2C_DATA];
	struct cam_cci_ctrl cci_ctrl;

	memset(&cci_ctrl, 0, sizeof(cci_ctrl));

	if (type > CAMERA_SENSOR_I2C_TYPE_MAX || len > SONY_CAMERA_MAX_I2C_DATA) {
		rc = -EINVAL;
	} else {
		cci_ctrl.cmd = MSM_CCI_I2C_READ;
		cci_ctrl.cci_info = &data->cci_info;
		cci_ctrl.cci_info->sid = slave_addr >> 1;
		cci_ctrl.cfg.cci_i2c_read_cfg.addr = addr;
		cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = type;
		cci_ctrl.cfg.cci_i2c_read_cfg.data = read_data;
		cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = len;
		while ((rc = v4l2_subdev_call(data->cci_info.cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl)) && retry_cnt) {
			retry_cnt--;
			usleep_range(1000, 3000);
			LOGE("Retry slave0x%04x,addr0x%04x,type0x%02x,len0x%02x\n",
				slave_addr, addr, type, len);
		}
		if (rc < 0) {
			LOGE("slave0x%04x,addr0x%04x,type0x%02x,len0x%02x\n",
				slave_addr, addr, type, len);
			LOGE("i2c read failed(%d)\n", rc);
			rc = -EIO;
		} else {
			rc = cci_ctrl.status;
			for (i = 0; i < len; i++)
				buf[i] = read_data[i];
		}
	}
	return rc;
}

static int sony_camera_i2c_write(struct sony_camera_data *data,
	uint8_t slave_addr, uint32_t addr, uint8_t type,
	uint16_t len, uint8_t *buf)
{
	int32_t rc = -EINVAL;
	int i = 0;
	struct cam_cci_ctrl cci_ctrl;
	struct cam_sensor_i2c_reg_array* reg_settings;
	bool need_free = false;
	if (len > SONY_CAMERA_I2C_MAX_LEN) {
		reg_settings = vzalloc(sizeof(struct cam_sensor_i2c_reg_array) * len);
		need_free = true;
	} else {
		memset(data->i2c_data, 0, SONY_CAMERA_I2C_MAX_SIZE);
		reg_settings = (struct cam_sensor_i2c_reg_array*)data->i2c_data;
	}

	for (i = 0; i < len; i++) {
		reg_settings[i].reg_addr = addr + i;
		reg_settings[i].reg_data = buf[i];
		reg_settings[i].delay = 0;
		reg_settings[i].data_mask = 0;
	}
	memset(&cci_ctrl, 0, sizeof(cci_ctrl));
	cci_ctrl.cmd = MSM_CCI_I2C_WRITE_BURST;
	cci_ctrl.cci_info = &data->cci_info;
	cci_ctrl.cci_info->sid = slave_addr >> 1;
	cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = reg_settings;
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = 1; // TODO: CAMERA_SENSOR_I2C_TYPE_BYTE
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = type;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = len;
	rc = v4l2_subdev_call(data->cci_info.cci_subdev,
		core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0)
		LOGE("i2c write failed(%d) slave0x%04x,addr0x%04x,type0x%02x,len0x%02x\n",
			rc, slave_addr, addr, type, len);
	if (need_free)
		vfree(reg_settings);
	return rc;
}

static int sony_camera_ext_i2c_read(struct sony_camera_data *data,
	uint8_t slave_addr, uint32_t addr,
	uint8_t type, int size, uint8_t *buf)
{
	int rc;
	struct i2c_client *qup_client = data->qup_client;
	uint8_t w_buf[4];
	struct i2c_msg msg[2];
	int i = 0;
	if (!buf) {
		LOGE("no memory\n");
		return -ENOMEM;
	}
	if (type == 1) {
		w_buf[0] = (uint8_t)addr;
	} else if (type == 2) {
		w_buf[0] = (uint8_t)(addr >> 8);
		w_buf[1] = (uint8_t)(addr & 0xFF);
	} else {
		w_buf[0] = (uint8_t)(addr >> 24);
		w_buf[1] = (uint8_t)(addr >> 16);
		w_buf[2] = (uint8_t)(addr >> 8);
		w_buf[3] = (uint8_t)(addr & 0xFF);
	}
	for (i = 0; i < size; i++) {
		w_buf[type + i] = buf[i];
	}

	qup_client->addr = slave_addr;
	msg[0].addr = (qup_client->addr) >> 1;
	msg[0].flags = 0;
	msg[0].buf = w_buf;
	msg[0].len = type;

	msg[1].addr = (qup_client->addr) >> 1;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = size;
	rc = i2c_transfer(qup_client->adapter, msg, 2);
	if (rc != 2) {
		LOGE("i2c_transfer fail\n");
		return -EFAULT;
	}
	return 0;
}

static int sony_camera_ext_i2c_write(struct sony_camera_data *data,
	uint8_t slave_addr, uint32_t addr, uint8_t type,
	int size, uint8_t *buf)
{
	int rc;
	struct i2c_client *qup_client = data->qup_client;
	uint8_t *w_buf;
	struct i2c_msg msg[2];
	int i = 0;
	w_buf = kmalloc(size + type, GFP_KERNEL);
	if (!w_buf) {
		LOGE("no memory\n");
		return -ENOMEM;
	}
	if (type == 1) {
		w_buf[0] = (uint8_t)addr;
	} else if (type == 2) {
		w_buf[0] = (uint8_t)(addr >> 8);
		w_buf[1] = (uint8_t)(addr & 0xFF);
	} else {
		w_buf[0] = (uint8_t)(addr >> 24);
		w_buf[1] = (uint8_t)(addr >> 16);
		w_buf[2] = (uint8_t)(addr >> 8);
		w_buf[3] = (uint8_t)(addr & 0xFF);
	}
	for (i = 0; i < size; i++) {
		w_buf[type + i] = buf[i];
	}

	qup_client->addr = slave_addr;
	msg[0].addr = (qup_client->addr) >> 1;
	msg[0].flags = 0;
	msg[0].buf = w_buf;
	msg[0].len = size + type;
	rc = i2c_transfer(qup_client->adapter, msg, 1);
	kfree(w_buf);
	if (rc != 1) {
		LOGE("i2c_transfer fail\n");
		return -EFAULT;
	}
	return 0;
}

static int sony_camera_power_ctrl(struct sony_camera_data *data,
	uint8_t on)
{
	int rc = 0;
	const struct sony_camera_module *mod = data->probe_done ?
		data->module : camera_info[data->id].modules;
	const struct sony_camera_seq *seq = on ? mod->seq_on : mod->seq_off;
	struct gpio *gpio_reset = NULL;

	while (seq->cmd != EXIT) {
		uint8_t iodt = 0x00;
		LOGD("%s sony_camera_power_ctrl %d cmd %d val1 %d val2 %d wait %d\n",
			__func__, __LINE__, seq->cmd, seq->val1, seq->val2, seq->wait);
		switch (seq->cmd) {
		case SONY_GPIO_RESET:
			gpio_reset = sony_camera_get_gpio_pin(data,
				SONY_CAMERA_GPIO_RESET);
			if (gpio_reset->gpio <= 0) {
				rc = -EPERM;
				break;
			}
			LOGD("reset gpio %d->%d\n", gpio_reset->gpio, seq->val1);
			rc = sony_camera_gpio_set(data, gpio_reset->gpio, seq->val1);
			break;
		case SONY_GPIO_VANA:
		{
			struct gpio *gpio_vana = sony_camera_get_gpio_pin(data,
				SONY_CAMERA_GPIO_VANA);
			if (!gpio_vana) {
				rc = -EPERM;
				break;
			}
			LOGD("vana gpio %d->%d\n", gpio_vana->gpio, seq->val1);
			rc = sony_camera_gpio_set(data, gpio_vana->gpio, seq->val1);
			break;
		}
		case SONY_CAM_VDIG:
		case SONY_CAM_VIO:
		case SONY_CAM_VANA:
		case SONY_CAM_VAF:
		case SONY_CAM_VAF2:
		case SONY_CAM_OIS:
		case SONY_CAM_OIS2:
		case SONY_CAM_VMDR:
		case SONY_CAM_GYRO:
		case SONY_CAM_VDIG2:
		case SONY_CAM_PMIC0:
		case SONY_CAM_PMIC1:
			rc = sony_camera_vreg_set(data,
				seq->cmd, seq->val1, seq->val2);
			break;
		case SONY_CAM_CLK:
			rc = sony_camera_vreg_set(data,
				seq->cmd, on ? 0 : -1, seq->val2);
			if (rc < 0) {
				LOGE("%s mclk vreg set failed %d\n", __func__, __LINE__);
			} else {
				rc = sony_camera_mclk_set(data, seq->val1);
			}
			break;
		case SONY_PMIC_VIN:
			if (!data->pmic_vin) {
				data->pmic_vin = regulator_get(&data->p_dev->dev, "pmic_vin");
				if (IS_ERR(data->pmic_vin)) {
					LOGE("could not get pmic_vin, vreg = %ld\n",
						PTR_ERR(data->pmic_vin));
					rc = -ENODEV;
					goto exit;
				}
			}
			rc = regulator_set_voltage(data->pmic_vin, seq->val1 * 1000, seq->val2 * 1000);
			if (rc < 0) {
				LOGE("%s pmic vin voltage set failed %d\n", __func__, __LINE__);
				goto exit;
			}
			if (on) {
				rc = regulator_enable(data->pmic_vin);
				if (rc < 0) {
					LOGE("%s pmic vin enable failed %d\n", __func__, __LINE__);
					goto exit;
				}
			} else {
				(void)regulator_disable(data->pmic_vin);
			}
			break;
		case SONY_MIPI_SWITCH:
			gpio_reset = sony_camera_get_gpio_pin(data,
				SONY_CAMERA_MIPI_SWITCH);
			if (gpio_reset->gpio <= 0) {
				rc = -EPERM;
				break;
			}
			LOGD("reset gpio %d->%d\n", gpio_reset->gpio, seq->val1);
			rc = sony_camera_gpio_set(data, gpio_reset->gpio, seq->val1);
			break;
		case SONY_I2C_WRITE:
			rc = sony_camera_i2c_write(data,
				camera_info[data->id].i2c_addr, seq->val1, 2, 1, &iodt);
			break;
		case SONY_PROBE_WAIT:
		case SONY_PROBE_WAIT2:
			if (!data->probe_done)
				usleep_range(seq->val1, seq->val1);
			break;
		default:
			goto exit;
		}
		usleep_range(seq->wait, seq->wait);
		if (rc < 0 && on)
			goto exit;
		seq++;
	}
exit:
	return rc;
}

static int sony_camera_thermal_get_temp(
	struct thermal_zone_device *thermal, int *temp)
{
	int rc = 0;
	int id = 0;
	int thermal_id = 0;
	uint8_t i = 0;
	uint8_t max_cmb_tbl_num = 0;

	if (!temp) {
		LOGE("%s failed %d\n", __func__, __LINE__);
		rc = -EPERM;
		goto error;
	}

	max_cmb_tbl_num = sizeof(driver_comb_tbl) / sizeof(driver_comb_tbl[0]);
	for (i = 0; i < max_cmb_tbl_num; i++) {
		if (!strncmp(thermal->type, driver_comb_tbl[i].thermal_name, sizeof(thermal->type))) {
			id = driver_comb_tbl[i].id;
			thermal_id = driver_comb_tbl[i].thermal_zone;
			break;
		}
	}
	if (i == max_cmb_tbl_num) {
		rc = -EPERM;
		goto error;
	}
	mutex_lock(&camera_data[id].thermal_lock);
	if (camera_data[id].thermal_ret_val < 0)
		rc = camera_data[id].thermal_ret_val;
	else
		*temp = (int)camera_data[id].thermal_sensor_temperature[thermal_id];
		LOGD("%s %d, id = %d rc = %d *temp = %d\n",
			__func__, __LINE__, id, rc, *temp);
	mutex_unlock(&camera_data[id].thermal_lock);

error:
	return rc;
}

static struct thermal_zone_device_ops sony_camera_thermal_ops = {
	.get_temp = sony_camera_thermal_get_temp,
};

static int sony_camera_match_sensor(uint32_t id)
{
	int rc = 0;
	uint8_t data[2];
	uint16_t match_value = 0;
	uint8_t match_len = 1;

	if (!camera_info[id].match_id.enabled) {
		LOGI("camera %d match skipped\n", id);
		camera_data[id].module = &camera_info[id].modules[0];
		goto exit;
	}

	match_len = camera_info[id].match_id.len;
	if (camera_data[id].qup_client) {
		rc = sony_camera_ext_i2c_read(&camera_data[id], camera_info[id].slave_addr,
			camera_info[id].match_id.addr, 2, match_len, data);
	} else {
		rc = sony_camera_i2c_read(&camera_data[id], camera_info[id].slave_addr,
			camera_info[id].match_id.addr, 2, match_len, data);
	}
	if (rc < 0) {
		LOGE("%s %d camera %d match failed %d match_addr=0x%x  match_value=0x%x read_value=0x%x 0x%x\n",
			__func__, __LINE__, id, rc,
			camera_info[id].match_id.addr, camera_info[id].match_id.expect_value,
			data[0], data[1]);
		goto exit;
	} else {
		if (match_len == 2) {
			match_value = ((data[0] << 8) & 0xFF00) | (data[1] & 0x00FF);
		} else if (camera_info[id].match_id.expect_value == 0) {
			match_value = 0;
		} else {
			match_value = data[0] & 0x00FF;
		}
	}

	if (match_value == camera_info[id].match_id.expect_value) {
		LOGD("camera %d match success match_addr=0x%x  match_value=0x%x\n",
			id, camera_info[id].match_id.addr, camera_info[id].match_id.expect_value);
		camera_data[id].module = &camera_info[id].modules[0];
	} else {
		LOGE("camera %d match failed match_addr=0x%x  match_value=0x%x read_value=0x%x 0x%x\n",
			id, camera_info[id].match_id.addr, camera_info[id].match_id.expect_value, data[0], data[1]);
		rc = -1;
	}

exit:
	return rc;
}

static int sony_camera_clear(struct sony_camera_data *data)
{
	int rc = 0;
	unsigned long flags;
	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	while (!list_empty(&data->event_available)) {
		struct sony_camera_event_list_data *event = list_entry(
			data->event_available.next,
			struct sony_camera_event_list_data, list);

		list_del(&event->list);
		kfree(event);
	}
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	spin_lock_irqsave(&camera_data->sof_lock, flags);
	data->sof_count = 0;
	spin_unlock_irqrestore(&camera_data->sof_lock, flags);

	return rc;
}

static int sony_camera_power_down(struct sony_camera_data *data)
{
	int rc = 0;

	if (data->state == SONY_CAMERA_STATE_POWER_DOWN) {
	    goto exit;
	}
	mutex_lock(&data->state_lock);
	data->state = SONY_CAMERA_STATE_POWER_DOWN;
	mutex_unlock(&data->state_lock);

	if (data->enable_cam_vio_by_user) {
		rc = sony_camera_vreg_set(data,
			SONY_CAM_VIO, 0xFFFFFFFF, 0);
		data->enable_cam_vio_by_user= false;
		if (rc < 0)
			LOGE("power_down vio fail\n");
	}

	if (data->enable_cam_vana_by_user) {
		rc = sony_camera_vreg_set(data,
			SONY_CAM_VANA, 0xFFFFFFFF, 0);
		data->enable_cam_vana_by_user = false;
		if (rc < 0)
			LOGE("power_down vana fail\n");
	}

	if (data->enable_cam_vdig_by_user) {
		rc = sony_camera_vreg_set(data,
			SONY_CAM_VDIG, 0xFFFFFFFF, 0);
		data->enable_cam_vdig_by_user = false;
		if (rc < 0)
			LOGE("power_down vdig fail\n");
	}

	if (data->enable_cam_vaf_by_user) {
		rc = sony_camera_vreg_set(data,
			SONY_CAM_VAF, 0xFFFFFFFF, 0);
		data->enable_cam_vaf_by_user = false;
		if (rc < 0)
			LOGE("power_down vaf fail\n");
	}

	if (data->enable_cam_clk_by_user) {
		rc = sony_camera_vreg_set(data,
			SONY_CAM_CLK, 0xFFFFFFFF, 0);
		data->enable_cam_clk_by_user = false;
		if (rc < 0)
			LOGE("power_down clk fail\n");
	}

	if (data->enable_cam_vmdr_by_user) {
		rc = sony_camera_vreg_set(data,
			SONY_CAM_VMDR, 0xFFFFFFFF, 0);
		data->enable_cam_vmdr_by_user = false;
		if (rc < 0)
			LOGE("power_down vmdr fail\n");
	}

	if (data->enable_cam_vdig2_by_user) {
		rc = sony_camera_vreg_set(data,
			SONY_CAM_VDIG2, 0xFFFFFFFF, 0);
		data->enable_cam_vdig2_by_user = false;
		if (rc < 0)
			LOGE("power_down vdig2 fail\n");
	}

	rc = sony_camera_power_ctrl(data, false);
	if (rc < 0)
		LOGE("power_down fail\n");

	if (data->has_hw_sof || data->has_hw_ext_irq) {
		rc = sony_camera_irq_deinit(data,
			&camera_info[data->id]);
		if (rc < 0)
			LOGE("%s: irq_deinit failed\n", __func__);
	}

	if (data->i2c_data) {
		kfree(data->i2c_data);
		if (!data->use_qup) {
			rc = sony_camera_cci_deinit(data);
		}
	}
	if (rc < 0)
		LOGE("%s cci_deinit failed\n", __func__);

	rc = pinctrl_select_state(data->pinctrl,
		data->gpio_state_suspend);
	if (rc)
		pr_err("%s:%d cannot set pin to suspend state",
			__func__, __LINE__);

	rc = sony_camera_gpio_deinit(data);
	if (rc < 0)
		LOGE("%s: gpio_deinit failed\n", __func__);
	mutex_lock(&data->thermal_lock);
	data->thermal_ret_val = -ENODEV;
	mutex_unlock(&data->thermal_lock);
	sony_camera_clear(data);

exit:

	return rc;
}

static int sony_camera_power_up(struct sony_camera_data *data)
{
	int rc = 0;
	uint8_t retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;

	if (data->state == SONY_CAMERA_STATE_POWER_UP) {
	    goto exit;
	}

	LOGD("%s: %d\n", __func__, __LINE__);
	rc = sony_camera_gpio_init(data);
	if (rc < 0) {
		LOGE("%s: gpio_init failed\n", __func__);
		sony_camera_gpio_deinit(data);
		goto exit;
	}

	rc = pinctrl_select_state(data->pinctrl,
		data->gpio_state_active);
	if (rc) {
		LOGE("%s:%d cannot set pin to active state\n",
			__func__, __LINE__);
		sony_camera_gpio_deinit(data);
		pinctrl_select_state(data->pinctrl,
			data->gpio_state_suspend);
		goto exit;
	}

	LOGI("%s: id = %d, has_hw_sof %d %p\n", __func__,
		data->id, data->has_hw_sof, dev_name(&data->p_dev->dev));

	if (data->has_hw_sof || data->has_hw_ext_irq) {
		rc = sony_camera_irq_init(data, &camera_info[data->id]);
		if (rc < 0) {
			LOGE("irq_init failed\n");
			goto exit;
		}
	}
	if (!data->use_qup) {
		rc = sony_camera_cci_init(data);
		if (rc < 0) {
			LOGE("%s cci_init failed\n", __func__);
			sony_camera_gpio_deinit(data);
			pinctrl_select_state(data->pinctrl,
				data->gpio_state_suspend);
			sony_camera_cci_deinit(data);
			goto exit;
		}
	}

	retry_cnt = SONY_CAMERA_MAX_RETRY_COUNT;
	while (!(data->i2c_data = kzalloc(SONY_CAMERA_I2C_MAX_SIZE, GFP_KERNEL)) &&
			retry_cnt) {
		retry_cnt--;
		usleep_range(1000, 3000);
		LOGE("Retry kmalloc i2c data\n");
	}
	if (!data->i2c_data) {
		LOGE("%s: i2c data kmalloc failed\n", __func__);
		return -ENOMEM;
	}
	rc = sony_camera_power_ctrl(data, true);
	if (rc < 0) {
		LOGE("power_up fail\n");
		sony_camera_power_down(data);
		goto exit;
	}
	mutex_lock(&data->state_lock);
	data->state = SONY_CAMERA_STATE_POWER_UP;
	mutex_unlock(&data->state_lock);
	mutex_lock(&data->thermal_lock);
	data->thermal_ret_val = -EINVAL;
	mutex_unlock(&data->thermal_lock);

exit:

	return rc;
}

static int sony_camera_open(struct inode* inode, struct file* file)
{
	int rc = 0;
	unsigned int id = iminor(inode);
	file->private_data = &camera_data[id];

	mutex_lock(&camera_data[id].command_lock);

	if (camera_data[id].open_count == 0) {
		device_init_wakeup(&camera_data[id].p_dev->dev, 1);
		pm_stay_awake(&camera_data[id].p_dev->dev);
	}
	camera_data[id].open_count++;

	mutex_unlock(&camera_data[id].command_lock);

	LOGI("sensor opened %d\n", id);
	return rc;
}

static int sony_camera_close(struct inode* inode, struct file* file)
{
	int rc = 0;
	struct sony_camera_data* data = (struct sony_camera_data*)file->private_data;

	mutex_lock(&data->command_lock);

	if (0 < data->open_count) {
		data->open_count--;
	}
	if (data->open_count == 0) {
		sony_camera_power_down(data);
		pm_relax(&data->p_dev->dev);
		device_init_wakeup(&data->p_dev->dev, 0);
	}

	mutex_unlock(&data->command_lock);

	LOGI("sensor closed\n");
	return rc;
}

static unsigned int sony_camera_poll(struct file *file,
	struct poll_table_struct *wait)
{
	unsigned int rc = 0;
	unsigned long flags;
	int empty = 0;
	struct sony_camera_data* data = (struct sony_camera_data *)file->private_data;

	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	empty = list_empty(&data->event_available);
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	if (empty) {
		poll_wait(file, &data->event_wait_q, wait);
	}
	spin_lock_irqsave(&camera_data->event_list_lock, flags);
	empty = list_empty(&data->event_available);
	spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
	if (!empty) {
		rc = POLLPRI;
	}
	return rc;
}

static long sony_camera_ioctl_common(struct file *file,
	unsigned int cmd, unsigned long parm, bool compat)
{
	int rc = 0;
	unsigned long flags;
	struct sony_camera_data *data = (struct sony_camera_data *)file->private_data;
	struct sony_camera_i2c_data setting;
	uint8_t i2c_freq_mode = SONY_CAMERA_I2C_FREQ_MODE_MAX;

	if (!data->probe_done) {
		LOGE("device probe not done\n");
		rc = -ENODEV;
		goto exit;
	}

	if (cmd == SONY_CAMERA_CMD_GET_EVENT) {
		spin_lock_irqsave(&camera_data->event_list_lock, flags);
		if (list_empty(&data->event_available)) {
			rc = -ENODATA;
			spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
		} else {
			struct sony_camera_event_list_data *event = list_entry(
				data->event_available.next,
				struct sony_camera_event_list_data, list);
			list_del(&event->list);
			spin_unlock_irqrestore(&camera_data->event_list_lock, flags);
			if (compat) {
				struct sony_camera_event32 event_data32;
				memset(&event_data32, 0, sizeof(event_data32));
				event_data32.type = event->event_data.type;
				event_data32.sof_data.sof_count = event->event_data.sof_data.sof_count;
				event_data32.sof_data.mono_timestamp.tv_sec = event->event_data.sof_data.mono_timestamp.tv_sec;
				event_data32.sof_data.mono_timestamp.tv_usec = event->event_data.sof_data.mono_timestamp.tv_usec;
				rc = copy_to_user((void __user *) parm,
					&event_data32 , sizeof(event_data32));
			} else {
				rc = copy_to_user((void __user *) parm,
					&event->event_data , sizeof(event->event_data));
			}
			kfree(event);
		}
	} else if (cmd == SONY_CAMERA_CMD_I2C_READ_2 ||
				cmd == SONY_CAMERA_CMD_I2C_WRITE_2) {
		switch (cmd)
		{
		case SONY_CAMERA_CMD_I2C_READ_2:
			LOGD("sensor qup I2C read id=%d\n", data->id);
			rc = copy_from_user(&setting,
				(void __user *) parm, sizeof(setting));
			if (rc < 0) {
				LOGE("%s: copy_from_user failed\n", __func__);
				goto exit;
			}
			rc = sony_camera_qup_i2c_read(data,
				setting.slave_addr, setting.addr, setting.addr_type,
				setting.len, setting.data);
			if (rc < 0) {
				LOGE("%s: sony_camera_i2c_read failed\n", __func__);
				goto exit;
			}
			rc = copy_to_user((void __user *) parm,
				&setting, sizeof(setting));
			if (rc < 0)
				LOGE("%s: copy_to_user failed\n", __func__);
			break;
		case SONY_CAMERA_CMD_I2C_WRITE_2:
			LOGD("sensor qup I2C write id=%d\n", data->id);
			rc = copy_from_user(&setting,
				(void __user *) parm, sizeof(setting));
			if (rc < 0) {
				LOGE("%s: copy_from_user failed\n", __func__);
				goto exit;
			}
			LOGD("sensor I2C write slave_addr=%x addr=%x len=%d\n",
				setting.slave_addr, setting.addr, setting.len);
			rc = sony_camera_qup_i2c_write(data,
				setting.slave_addr, setting.addr, setting.addr_type,
				setting.len, setting.data);
			break;
		default:
			break;
		}

	} else {
		mutex_lock(&data->command_lock);
		switch (cmd) {
		case SONY_CAMERA_CMD_POWER_UP:
			LOGI("sensor power up %d\n", data->id);
			rc = sony_camera_power_up(data);
			break;
		case SONY_CAMERA_CMD_POWER_DOWN:
			LOGI("sensor power down %d\n", data->id);
			rc = sony_camera_power_down(data);
			break;
		case SONY_CAMERA_CMD_I2C_READ:
			LOGD("sensor I2C read id=%d\n", data->id);
			rc = copy_from_user(&setting,
				(void __user *) parm, sizeof(setting));
			if (rc < 0) {
				LOGE("%s: copy_from_user failed\n", __func__);
				goto exit;
			}
			if (data->qup_client) {
				rc = sony_camera_ext_i2c_read(data,
					setting.slave_addr, setting.addr, setting.addr_type,
					setting.len, setting.data);
			} else {
				rc = sony_camera_i2c_read(data,
					setting.slave_addr, setting.addr, setting.addr_type,
					setting.len, setting.data);
			}
			if (rc < 0) {
				LOGE("%s: sony_camera_i2c_read failed\n", __func__);
				goto exit;
			}
			rc = copy_to_user((void __user *) parm,
				&setting, sizeof(setting));
			if (rc < 0)
				LOGE("%s: copy_to_user failed\n", __func__);
			break;
		case SONY_CAMERA_CMD_I2C_WRITE:
			LOGD("sensor I2C write id=%d\n", data->id);
			rc = copy_from_user(&setting,
				(void __user *) parm, sizeof(setting));
			if (rc < 0) {
				LOGE("%s: copy_from_user failed\n", __func__);
				goto exit;
			}
			LOGD("sensor I2C write slave_addr=%x addr=%x len=%d\n",
				setting.slave_addr, setting.addr, setting.len);
			if (data->qup_client) {
				rc = sony_camera_ext_i2c_write(data,
					setting.slave_addr, setting.addr, setting.addr_type,
					setting.len, setting.data);
			} else {
				rc = sony_camera_i2c_write(data,
					setting.slave_addr, setting.addr, setting.addr_type,
					setting.len, setting.data);
			}
			break;
		case SONY_CAMERA_CMD_SET_THERMAL:
			mutex_lock(&data->thermal_lock);
			data->thermal_ret_val = 0;
			rc = copy_from_user(&data->thermal_sensor_temperature,
				(void __user *) parm, sizeof(data->thermal_sensor_temperature));
			mutex_unlock(&data->thermal_lock);
			if (rc < 0) {
				LOGE("%s:%d copy_from_user failed\n", __func__, __LINE__);
				rc = -EINVAL;
			}
			break;
		case SONY_CAMERA_CMD_GET_POWER_STATE:
			rc = data->state;
			break;
		case SONY_CAMERA_CMD_GET_I2C_FREQ_MODE:
			rc = data->cci_info.i2c_freq_mode;
			break;
		case SONY_CAMERA_CMD_SET_I2C_FREQ_MODE:
			rc = copy_from_user(&i2c_freq_mode,
				(void __user *) parm, sizeof(i2c_freq_mode));
			if (rc < 0) {
				LOGE("%s: copy_from_user failed\n", __func__);
				goto exit;
			}
			LOGD("sensor I2C freq mode=%d\n", i2c_freq_mode);
			data->cci_info.i2c_freq_mode = i2c_freq_mode;
			break;
		case SONY_CAMERA_CMD_CLEAR:
			rc = sony_camera_clear(data);
			break;
		case SONY_CAMERA_CMD_BUG_ON:
			BUG_ON(1);
			break;
		case SONY_CAMERA_CMD_SET_POWER:
			{
				struct sony_camera_power_cmd cmd;
				rc = copy_from_user(&cmd,
					(void __user *) parm, sizeof(cmd));
				if (rc < 0) {
					LOGE("%s: copy_from_user failed\n", __func__);
					goto exit;
				}
				rc = sony_camera_set_power(data, cmd.cmd, (cmd.arg != 0));
				if (rc < 0) {
					LOGE("%s: set_power failed\n", __func__);
					goto exit;
				}
			}
			break;
		default:
			rc = -EINVAL;
			break;
		}
exit:
		mutex_unlock(&data->command_lock);
	}
	return rc;
}

static long sony_camera_ioctl(struct file* file,
	unsigned int cmd, unsigned long parm)
{
	unsigned int rc = 0;

	rc = sony_camera_ioctl_common(file, cmd, parm, false);

	return rc;
}

static long sony_camera_ioctl32(struct file* file,
	unsigned int cmd, unsigned long parm)
{
	unsigned int rc = 0;

	rc = sony_camera_ioctl_common(file, cmd, parm, true);

	return rc;
}

static struct file_operations sony_camera_fops =
{
	.owner   = THIS_MODULE,
	.poll    = sony_camera_poll,
	.open    = sony_camera_open,
	.release = sony_camera_close,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sony_camera_ioctl32,
#endif	/* CONFIG_COMPAT */
	.unlocked_ioctl = sony_camera_ioctl,
};

static const struct of_device_id sony_camera_0_dt_match[] = {
	{
		.compatible = "sony_camera_0",
		.data = &camera_data[0]
	},
	{
	},
};

static const struct of_device_id sony_camera_1_dt_match[] = {
	{
		.compatible = "sony_camera_1",
		.data = &camera_data[1]
	},
	{
	},
};

static const struct of_device_id sony_camera_2_dt_match[] = {
	{
		.compatible = "sony_camera_2",
		.data = &camera_data[2]
	},
	{
	},
};

static const struct of_device_id sony_camera_3_dt_match[] = {
	{
		.compatible = "sony_camera_3",
		.data = &camera_data[3]
	},
	{
	},
};

static const struct of_device_id sony_camera_4_dt_match[] = {
	{
		.compatible = "sony_camera_4",
		.data = &camera_data[4]
	},
	{
	},

};

static const struct of_device_id sony_camera_5_dt_match[] = {
	{
		.compatible = "sony_camera_5",
		.data = &camera_data[5]
	},
	{
	},
};

static const struct of_device_id sony_camera_spi_dt_match[] = {
	{
		.compatible = "sony_aube"
	},
	{ },
};

static const struct of_device_id sony_camera_qup_i2c_subdev_dt_match[] = {
	{
		.compatible = "sony,camera_subdev"
	},
	{ },
};

MODULE_DEVICE_TABLE(of,  sony_camera_0_dt_match              );
MODULE_DEVICE_TABLE(of,  sony_camera_1_dt_match              );
MODULE_DEVICE_TABLE(of,  sony_camera_2_dt_match              );
MODULE_DEVICE_TABLE(of,  sony_camera_3_dt_match              );
MODULE_DEVICE_TABLE(of,  sony_camera_4_dt_match              );
MODULE_DEVICE_TABLE(of,  sony_camera_5_dt_match              );
MODULE_DEVICE_TABLE(of,  sony_camera_qup_i2c_subdev_dt_match );

static const struct i2c_device_id sony_camera_qup_i2c_subdev_id[] = {
	{ "camera_subdev", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sony_camera_qup_i2c_subdev_id       );


static int sony_camera_qup_i2c_subdev_probe(struct i2c_client *qup_client,
	const struct i2c_device_id *id)
{	uint32_t val_u32 = 0;
	struct sony_camera_qup_i2c_device *device = NULL;
	uint8_t retry_count = SONY_CAMERA_MAX_RETRY_COUNT;

	int rc = 0;
	if (!i2c_check_functionality(qup_client->adapter, I2C_FUNC_I2C)) {
		LOGE("%s:%d check functionality failed\n",
			__func__, __LINE__);
		rc = -EIO;
		goto fail;
	}

	rc = of_property_read_u32(qup_client->dev.of_node, "camera_id", &val_u32);
	if (rc < 0) {
	LOGE("%s:%d find camera_id failed\n",
			__func__, __LINE__);
		rc = -EINVAL;
		goto fail;
	}

	device = &camera_data[val_u32].qup_I2C_dev;
	device->client = qup_client;
	device->enabled = true;

	while (!(device->i2c_data = kzalloc(SONY_CAMERA_QUP_I2C_MAX_LEN, GFP_KERNEL)) &&
			retry_count) {
		retry_count--;
		usleep_range(1000, 3000);
		LOGE("Retry kmalloc qup i2c data\n");
	}
	if (!device->i2c_data) {
		LOGE("%s: qup i2c data kmalloc failed\n", __func__);
		return -ENOMEM;
	}

	LOGI("QUP I2C driver probe success\n");
	return 0;
fail:
	if (device->i2c_data)
		kfree(device->i2c_data);
	return rc;
}

static int sony_camera_qup_i2c_subdev_remove(struct i2c_client *qup_client)
{
	i2c_set_clientdata(qup_client, NULL);
	return 0;
}

static struct i2c_driver sony_camera_qup_i2c_subdev_driver = {
	.driver = {
		.name = "camera_subdev",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sony_camera_qup_i2c_subdev_dt_match),
	},
	.probe = sony_camera_qup_i2c_subdev_probe,
	.id_table = sony_camera_qup_i2c_subdev_id,
	.remove = sony_camera_qup_i2c_subdev_remove,
};

static struct platform_driver sony_camera_platform_driver[] = {
	{
		.driver = {
			.name = "sony_camera_0",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_0_dt_match,
		},
	},
	{
		.driver = {
			.name = "sony_camera_1",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_1_dt_match,
		},
	},
	{
		.driver = {
			.name = "sony_camera_2",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_2_dt_match,
		},
	},
	{
		.driver = {
			.name = "sony_camera_3",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_3_dt_match,
		},
	},
	{
		.driver = {
			.name = "sony_camera_4",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_4_dt_match,
		},
	},
	{
		.driver = {
			.name = "sony_camera_5",
			.owner = THIS_MODULE,
			.of_match_table = sony_camera_5_dt_match,
		},
	},
};

static int sony_camera_ext_i2c_probe(struct i2c_client *qup_client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(qup_client->adapter, I2C_FUNC_I2C)) {
		LOGE("%s:%d check functionality failed\n",
			__func__, __LINE__);
		rc = -EIO;
		goto fail;
	}
	ext_cam_client = qup_client;
	return 0;
fail:
	return rc;
}

static int sony_camera_ext_i2c_remove(struct i2c_client *qup_client)
{
	i2c_set_clientdata(qup_client, NULL);
	return 0;
}

static const struct i2c_device_id ext_cam_id[] = {
	{ "external_camera", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ext_cam_id);

static const struct of_device_id ext_cam_i2c_dt_match[] = {
	{
		.compatible = "ext_cam_i2c"
	},
	{ },
};

static struct i2c_driver ext_cam_i2c_driver = {
	.driver = {
		.name = "external_camera",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ext_cam_i2c_dt_match),
	},
	.probe = sony_camera_ext_i2c_probe,
	.id_table = ext_cam_id,
	.remove = sony_camera_ext_i2c_remove,
};

static void sony_camera_platform_cleanup(void)
{
	uint16_t i;
	uint16_t j;

	for (i = 0; i < sensor_num; i++) {
		platform_driver_unregister(&sony_camera_platform_driver[i]);
		sony_camera_info_deinit(i);
		for (j = 0; j < SONY_CAMERA_MAX_TEMPERATURE_NUM_PER_CAMERA; j++) {
			if (camera_data[i].thermal_zone_dev[j]) {
				thermal_zone_device_unregister(camera_data[i].thermal_zone_dev[j]);
			}
		}
	}
}

static int sony_camera_platform_probe(struct platform_device *p_dev)
{
	int rc = 0;
	uint32_t id = 0;
	const struct of_device_id *match;
	match = of_match_device(sony_camera_0_dt_match, &p_dev->dev);
	if (!match && sensor_num > 1) {
		match = of_match_device(sony_camera_1_dt_match, &p_dev->dev);
		id = 1;
	}
	if (!match && sensor_num > 2) {
		match = of_match_device(sony_camera_2_dt_match, &p_dev->dev);
		id = 2;
	}
	if (!match && sensor_num > 3) {
		match = of_match_device(sony_camera_3_dt_match, &p_dev->dev);
		id = 3;
	}
	if (!match && sensor_num > 4) {
		match = of_match_device(sony_camera_4_dt_match, &p_dev->dev);
		id = 4;
	}
	if (!match && sensor_num > 5) {
		match = of_match_device(sony_camera_5_dt_match, &p_dev->dev);
		id = 5;
	}
	if (!match) {
		LOGE("of_match_device fail\n");
		rc = -EFAULT;
		goto fail;
	}

	camera_data[id].id = id;
	camera_data[id].p_dev = p_dev;
	camera_data[id].probe_done = false;
	camera_data[id].enable_cam_vio_by_user = false;
	camera_data[id].enable_cam_vana_by_user = false;
	camera_data[id].enable_cam_vdig_by_user = false;
	camera_data[id].enable_cam_vaf_by_user = false;
	camera_data[id].enable_cam_clk_by_user = false;
	camera_data[id].enable_cam_vmdr_by_user = false;
	camera_data[id].enable_cam_vdig2_by_user = false;

	rc = sony_camera_info_init(p_dev, id);
	if (rc < 0) {
		LOGE("%s sony_camera_info_init failed %d\n",
			__func__, __LINE__);
		goto fail;
	}
	if (!camera_data[id].use_qup) {
		// CCI initialize
		camera_data[id].cci_info.cci_subdev = cam_cci_get_subdev(camera_data[id].cci_info.cci_device);
		camera_data[id].cci_info.i2c_freq_mode = camera_info[id].modules[0].i2c_freq_mode;
		camera_data[id].cci_info.sid = 0;
		camera_data[id].cci_info.cid = 0;
		camera_data[id].cci_info.timeout = 0;
		camera_data[id].cci_info.retries = 3;
		camera_data[id].cci_info.id_map = 0;
	} else {
		camera_data[id].qup_client = ext_cam_client;
	}
	camera_data[id].pinctrl = devm_pinctrl_get(&p_dev->dev);
	if (IS_ERR_OR_NULL(camera_data[id].pinctrl)) {
		LOGE("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		rc = -EINVAL;
		goto fail;
	}
	camera_data[id].gpio_state_active =
		pinctrl_lookup_state(camera_data[id].pinctrl,
			SONY_CAMERA_PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(camera_data[id].gpio_state_active)) {
		LOGE("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		rc = -EINVAL;
		goto fail;
	}
	camera_data[id].gpio_state_suspend =
		pinctrl_lookup_state(camera_data[id].pinctrl,
			SONY_CAMERA_PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(camera_data[id].gpio_state_suspend)) {
		LOGE("%s:%d Failed to get the suspend state pinctrl handle\n",
			__func__, __LINE__);
		rc = -EINVAL;
		goto fail;
	}

	rc = sony_camera_power_up(&camera_data[id]);
	if (rc < 0) {
		LOGE("%s sony_camera_power_up failed %d\n",
			__func__, __LINE__);
		goto fail;
	}

	rc = sony_camera_match_sensor(id);
	if (rc < 0) {
		LOGE("%s sony_camera_match_sensor failed %d\n",
			__func__, __LINE__);
		sony_camera_power_down(&camera_data[id]);
		goto fail;
	}

	rc = sony_camera_power_down(&camera_data[id]);
	if (rc < 0) {
		LOGE("%s sony_camera_power_down failed %d\n",
			__func__, __LINE__);
		goto fail;
	}

	if (camera_info[id].thremal_enable) {
		uint8_t n = 0;
		uint8_t max_cmb_tbl_num = sizeof(driver_comb_tbl) / sizeof(driver_comb_tbl[0]);
		for (n = 0; n < max_cmb_tbl_num; n++) {
			if (id == driver_comb_tbl[n].id) {
				camera_data[id].thermal_zone_dev[driver_comb_tbl[n].thermal_zone] =
					thermal_zone_device_register(driver_comb_tbl[n].thermal_name,
					0, 0, 0, &sony_camera_thermal_ops, 0, 0, 0);
				if (IS_ERR(camera_data[id].thermal_zone_dev[driver_comb_tbl[n].thermal_zone])) {
					LOGE("%s thermal_zone_device_register (%u) %d\n",
						__func__, id, __LINE__);
					rc = PTR_ERR(camera_data[id].thermal_zone_dev[driver_comb_tbl[n].thermal_zone]);
					goto fail;
				}
			}
		}
	}

	camera_data[id].probe_done = true;
	LOGI("camera %d probe ok\n", id);

	return 0;

fail:
	return rc;
}

static int __init sony_camera_init_module(void)
{
	int rc = 0;
	uint16_t i;
	uint16_t probe_count = 0;

	sensor_num = ARRAY_SIZE(sony_camera_platform_driver);
	i2c_add_driver(&ext_cam_i2c_driver);

	for (i = 0; i < sensor_num; i++) {
		camera_data[i].state = SONY_CAMERA_STATE_POWER_DOWN;
		camera_data[i].open_count = 0;
		mutex_init(&camera_data[i].command_lock);
		mutex_init(&camera_data[i].qup_I2C_dev.command_lock);
		spin_lock_init(&camera_data[i].event_list_lock);
		mutex_init(&camera_data[i].thermal_lock);
		spin_lock_init(&camera_data[i].sof_lock);
		mutex_init(&camera_data[i].state_lock);
		init_waitqueue_head(&camera_data[i].event_wait_q);
		INIT_LIST_HEAD(&camera_data[i].event_available);
		rc = platform_driver_probe(
			&sony_camera_platform_driver[i],
			sony_camera_platform_probe);
		if (rc < 0) {
			LOGE("%s platform_driver_probe (%u) %d\n",
				__func__, i, __LINE__);
			continue;
		}
		probe_count++;
		msleep(SONY_CAMERA_FRONT_SENSOR_POWER_UP_WAIT);
	}

	if (!probe_count) {
		LOGE("%s platform_driver_probe (%u) %d\n",
			__func__, probe_count, __LINE__);
		goto fail_probe;
	}

	i2c_add_driver(&sony_camera_qup_i2c_subdev_driver);

	/* register the driver */
	dev_id = register_chrdev(0, SONY_CAMERA_DEV_NAME, &sony_camera_fops);
	LOGD("%s register_chrdev() (Major:%d).\n", __func__, dev_id);
	if (dev_id < 0) {
		pr_err("register_chrdev() failed (Major:%d).\n",
			dev_id);
		rc = -EINVAL;
		goto fail_register_chrdev;
	}

	/* memory allocation */
	camera_device = platform_device_alloc(SONY_CAMERA_DEV_NAME, -1);
	if (!camera_device) {
		LOGE("%s platform_device_alloc() failed.\n", __func__);
		rc = -ENOMEM;
		goto fail_probe;
	}

	/* add device */
	rc = platform_device_add(camera_device);
	if (rc) {
		LOGE("platform_device_add() failed.\n");
		goto fail_platform_dev_add;
	}

	/* create the node of device */
	camera_device_class = class_create(THIS_MODULE, SONY_CAMERA_DEV_NAME);
	if (IS_ERR(camera_device_class)) {
		LOGE("%s class_create() failed.\n", __func__);
		rc = PTR_ERR(camera_device_class);
		goto fail_class_create;
	}

	/* create the logical device */
	for (i = 0; i < sensor_num; i++) {
		if (camera_data[i].probe_done) {
			struct device *dev = NULL;
			dev = device_create(camera_device_class, NULL, MKDEV(dev_id, i), NULL, SONY_CAMERA_DEV_NAME"%d", i);
			if (IS_ERR(dev)) {
				LOGE("%s device_create() failed.\n", __func__);
				rc = PTR_ERR(dev);
				goto fail_device_create;
			}
		} else {
			LOGE("%s camera %d probefailed do not create device\n", __func__, i);
		}
	}

	return 0;

fail_register_chrdev:
	for (i = 0; i < sensor_num; i++) {
		device_destroy(camera_device_class, MKDEV(dev_id,i));
	}
fail_device_create:
	class_destroy(camera_device_class);
fail_class_create:
	platform_device_del(camera_device);
fail_platform_dev_add:
	platform_device_put(camera_device);
	sony_camera_platform_cleanup();
fail_probe:
	return rc;
}

static void __exit sony_camera_exit_module(void)
{
	uint16_t i;

	sony_camera_platform_cleanup();
	for (i = 0; i < sensor_num; i++) {
		dev_t dn = MKDEV(MAJOR(dev_id), i);
		device_destroy(c, dn);
		mutex_destroy(&camera_data[i].command_lock);
		mutex_destroy(&camera_data[i].qup_I2C_dev.command_lock);
		mutex_destroy(&camera_data[i].thermal_lock);
		mutex_destroy(&camera_data[i].state_lock);
	}
	class_destroy(c);
	cdev_del(&cdev);
	unregister_chrdev_region(dev_id, sensor_num);
}

static int sony_camera_set_power(struct sony_camera_data *data,
	int cmd, bool enable)
{
	unsigned int rc;

	if (enable) {
		rc = sony_camera_vreg_set(data,
			cmd, 0, 0);
	} else {
		rc = sony_camera_vreg_set(data,
			cmd, 0xFFFFFFFF, 0);
	}
	if (rc >= 0) {
		switch(cmd) {
		case SONY_CAM_VDIG:
			data->enable_cam_vdig_by_user = enable;
			break;
		case SONY_CAM_VIO:
			data->enable_cam_vio_by_user = enable;
			break;
		case SONY_CAM_VANA:
			data->enable_cam_vana_by_user = enable;
			break;
		case SONY_CAM_VAF:
			data->enable_cam_vaf_by_user = enable;
			break;
		case SONY_CAM_CLK:
			data->enable_cam_clk_by_user = enable;
			break;
		case SONY_CAM_VMDR:
			data->enable_cam_vmdr_by_user = enable;
			break;
		case SONY_CAM_VDIG2:
			data->enable_cam_vdig2_by_user = enable;
			break;
		default:
			break;
		}
	}
	return rc;
}

module_init(sony_camera_init_module);
module_exit(sony_camera_exit_module);

MODULE_DESCRIPTION("SONY camera sensor driver");
MODULE_LICENSE("GPL v2");
