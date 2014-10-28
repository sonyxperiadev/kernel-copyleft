/* drivers/misc/felica_snfc/cxd2235agg.c
 *
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Author: Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/qpnp/pin.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <mach/gpiomux.h>
#include <mach/irqs-8974.h>
#include "felica_snfc.h"

#define CEN_RETRY_MAX	5

enum gpio_id {
	LOCK_PIN = 0,
	FF_PIN,
	PON_PIN,
	LDO_EN_PIN,
	INTU_PIN,
	HSEL_PIN,
	RFS_PIN,
	INT_PIN,
	PVDD_PIN,
	KOTO_PWR_PIN,
	TX_PIN,
	RX_PIN,
};

static char const * const gpio_rsrcs[] = {
	"lock-gpio",
	"ff-gpio",
	"pon-gpio",
	"ldo_en-gpio",
	"intu-gpio",
	"hsel-gpio",
	"rfs-gpio",
	"int-gpio",
	"hvdd-gpio",
	"koto_pwr-gpio",
	"tx-gpio",
	"rx-gpio",
};

struct cxd2235agg_data {
	struct platform_device *pdev;
	unsigned int gpios[ARRAY_SIZE(gpio_rsrcs)];
	struct felica_data felica_data;
	bool is_enable;
};

static int felica_cen_init(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return 0;
}

static int felica_cen_release(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return 0;
}

static int felica_cen_read(u8 *buf, void *user)
{
	struct cxd2235agg_data *my_data = user;
	int st = 0;

	if (!buf || !my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	st = gpio_get_value_cansleep(my_data->gpios[LOCK_PIN]);

	dev_dbg(&my_data->pdev->dev, ": FELICA_LOCK = 0x%x\n", st);
	*buf = (st) ? 0x1 : 0x0;

	return 0;
}

static int felica_cen_write(u8 arg, void *user)
{
	struct cxd2235agg_data *my_data = user;
	int i;
	u8 state;
	bool wait_flg = false;
	static bool koto_pwr_init;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s: %x\n", __func__, arg);

	if (arg > 0x1) {
		dev_err(&my_data->pdev->dev,
			"%s: Error. Invalid val @CEN write.\n", __func__);
		return -EINVAL;
	}

	if (arg == 0x1) {
		if (!koto_pwr_init) {
			koto_pwr_init = true;
			wait_flg = true;
			gpio_set_value_cansleep(my_data->gpios[LDO_EN_PIN],
						1);
			msleep_interruptible(1);
			gpio_set_value_cansleep(my_data->gpios[KOTO_PWR_PIN],
						1);
			msleep_interruptible(2);
			gpio_set_value_cansleep(my_data->gpios[LDO_EN_PIN],
						0);
			msleep_interruptible(3);
			gpio_set_value_cansleep(my_data->gpios[KOTO_PWR_PIN],
						0);
			msleep_interruptible(50);
		}
	}

	for (i = 0; i < CEN_RETRY_MAX; i++) {
		felica_cen_read(&state, my_data);
		if (arg == state)
			goto end;
		gpio_set_value_cansleep(my_data->gpios[FF_PIN], 0);
		msleep_interruptible(1);
		gpio_set_value_cansleep(my_data->gpios[FF_PIN], 1);
		msleep_interruptible(1);
		gpio_set_value_cansleep(my_data->gpios[FF_PIN], 0);
	}

	dev_err(&my_data->pdev->dev, "%s: Error. Cannot write CEN.\n",
		__func__);

	return -EIO;

end:
	if (wait_flg)
		msleep_interruptible(1);
	return 0;
}

static struct gpiomux_setting tx_cfg[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir  = GPIOMUX_OUT_LOW,
	},
	{
		.func = GPIOMUX_FUNC_4,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
};

static struct gpiomux_setting rx_cfg[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,
		.drv  = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir  = GPIOMUX_IN,
	},
	{
		.func = GPIOMUX_FUNC_3,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
};

static void cxd2235agg_setup_uart_gpio(struct cxd2235agg_data *my_data, int val)
{
	int id = val ? 1 : 0;

	dev_dbg(&my_data->pdev->dev, ": %s: %d-%d\n", __func__, val, id);

	if (msm_gpiomux_write(my_data->gpios[TX_PIN], GPIOMUX_ACTIVE,
				&tx_cfg[id], NULL))
		dev_err(&my_data->pdev->dev, "%s: msm_gpiomux_write %d failed.",
			__func__, my_data->gpios[TX_PIN]);

	if (msm_gpiomux_write(my_data->gpios[RX_PIN], GPIOMUX_ACTIVE,
			&rx_cfg[id], NULL))
		dev_err(&my_data->pdev->dev, "%s: msm_gpiomux_write %d failed.",
			__func__, my_data->gpios[RX_PIN]);
}

static int felica_pon_init(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return 0;
}

static enum gpio_id second_req_ids[] = {
	INTU_PIN,
	RFS_PIN,
	INT_PIN,
};

static struct qpnp_pin_cfg intu_pin_cfg[] = {
	{
		.mode = QPNP_PIN_MODE_DIG_IN,
		.pull = QPNP_PIN_GPIO_PULL_DN,
		.vin_sel = QPNP_PIN_VIN2,
		.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
		.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT,
		.master_en = QPNP_PIN_MASTER_ENABLE,
	},
	{
		.mode = QPNP_PIN_MODE_DIG_IN,
		.pull = QPNP_PIN_GPIO_PULL_NO,
		.vin_sel = QPNP_PIN_VIN2,
		.out_strength = QPNP_PIN_OUT_STRENGTH_LOW,
		.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT,
		.master_en = QPNP_PIN_MASTER_ENABLE,
	},
};

static void felica_pon_write(int val, void *user)
{
	static bool pvdd_enable_oneshot;
	struct cxd2235agg_data *my_data = user;
	int i, ret;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s: %d\n", __func__, val);

	if (val) {
		if (!pvdd_enable_oneshot && !my_data->is_enable) {
			gpio_set_value_cansleep(my_data->gpios[PVDD_PIN], 1);
			msleep_interruptible(1);
			for (i = 0; i < ARRAY_SIZE(second_req_ids); i++) {
				ret = gpio_request(
					my_data->gpios[second_req_ids[i]],
					gpio_rsrcs[second_req_ids[i]]);
				if (ret) {
					dev_err(&my_data->pdev->dev,
						"%s: request err %s: %d\n",
						__func__,
						gpio_rsrcs[second_req_ids[i]],
						ret);
					goto error_gpio_second_request;
				}
			}
			ret = qpnp_pin_config(my_data->gpios[INTU_PIN],
						&intu_pin_cfg[1]);
			if (ret) {
				dev_err(&my_data->pdev->dev,
					"%s: INTU set config failed: %d\n",
					__func__, ret);
				goto error_intu_pin_config;
			}
			my_data->is_enable = true;
			usleep_range(500, 550);
			ret = felica_snfc_irq_start(&my_data->pdev->dev);
			if (ret) {
				dev_err(&my_data->pdev->dev,
					"%s: irq_start err: %d\n", __func__,
					ret);
				goto error_irq_start;
			}
			cxd2235agg_setup_uart_gpio(my_data, 1);
			pvdd_enable_oneshot = true;
			msleep_interruptible(10);
		}
		gpio_set_value_cansleep(my_data->gpios[PON_PIN], 1);
	} else {
		gpio_set_value_cansleep(my_data->gpios[PON_PIN], 0);
	}

	return;

error_irq_start:
	qpnp_pin_config(my_data->gpios[INTU_PIN], &intu_pin_cfg[0]);
error_intu_pin_config:
error_gpio_second_request:
	for (; i >= 0; --i)
		gpio_free(my_data->gpios[second_req_ids[i]]);
	gpio_set_value_cansleep(my_data->gpios[PVDD_PIN], 0);
	my_data->is_enable = false;
}

static void felica_pon_release(void *user)
{
	struct cxd2235agg_data *my_data = user;
	int i;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	if (my_data->is_enable) {
		qpnp_pin_config(my_data->gpios[INTU_PIN], &intu_pin_cfg[0]);
		for (i = 0; i < ARRAY_SIZE(second_req_ids); i++)
			gpio_free(my_data->gpios[second_req_ids[i]]);
		gpio_set_value_cansleep(my_data->gpios[PON_PIN], 0);
		gpio_set_value_cansleep(my_data->gpios[PVDD_PIN], 0);
		my_data->is_enable = false;
	}
}

static int felica_rfs_init(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return 0;
}

static int felica_rfs_read(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	if (!my_data->is_enable)
		return -EIO;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return gpio_get_value(my_data->gpios[RFS_PIN]);
}

static void felica_rfs_release(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);
}

static int felica_int_init(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return 0;
}

static int felica_int_read(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	if (!my_data->is_enable)
		return -EIO;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return gpio_get_value(my_data->gpios[INT_PIN]);
}

static void felica_int_release(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);
}

static int nfc_intu_init(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);
	return 0;
}

static int nfc_intu_read(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	if (!my_data->is_enable)
		return -EIO;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return gpio_get_value(my_data->gpios[INTU_PIN]);
}

static void nfc_intu_release(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);
}

static int nfc_hsel_init(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return -EINVAL;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	return 0;
}

static void nfc_hsel_write(int val, void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s: %d\n", __func__, val);

	gpio_set_value_cansleep(my_data->gpios[HSEL_PIN], val);
}

static void nfc_hsel_release(void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s\n", __func__);

	cxd2235agg_setup_uart_gpio(my_data, 0);
	gpio_set_value_cansleep(my_data->gpios[HSEL_PIN], 0);
}

static void nfc_shutdown(struct cxd2235agg_data *my_data)
{
	nfc_hsel_release(my_data);
	felica_snfc_irq_shutdown(&my_data->pdev->dev);
	felica_pon_release(my_data);
	msleep_interruptible(100);
}

static void nfc_ldo_write(int val, void *user)
{
	struct cxd2235agg_data *my_data = user;

	if (!my_data)
		return;

	dev_dbg(&my_data->pdev->dev, ": %s: %d\n", __func__, val);

	if (!val)
		nfc_shutdown(my_data);

	gpio_set_value_cansleep(my_data->gpios[LDO_EN_PIN], val);
}

static const struct felica_cen_ops cen_ops = {
	.cen_init = felica_cen_init,
	.cen_read = felica_cen_read,
	.cen_write = felica_cen_write,
	.cen_release = felica_cen_release,
};

static const struct felica_pon_ops pon_ops = {
	.pon_init = felica_pon_init,
	.pon_write = felica_pon_write,
	.pon_release = felica_pon_release,
};

static const struct felica_rfs_ops rfs_ops = {
	.rfs_init = felica_rfs_init,
	.rfs_read = felica_rfs_read,
	.rfs_release = felica_rfs_release,
};

static const struct felica_int_ops int_ops = {
	.int_init = felica_int_init,
	.int_read = felica_int_read,
	.int_release = felica_int_release,
};

static const struct nfc_intu_ops intu_ops = {
	.intu_init = nfc_intu_init,
	.intu_read = nfc_intu_read,
	.intu_release = nfc_intu_release,
};

static const struct nfc_hsel_ops hsel_ops = {
	.hsel_init = nfc_hsel_init,
	.hsel_write = nfc_hsel_write,
	.hsel_release = nfc_hsel_release,
};

static const struct nfc_ldo_ops ldo_ops = {
	.ldo_write = nfc_ldo_write,
};

static enum gpio_id req_ids[] = {
	LOCK_PIN,
	FF_PIN,
	PON_PIN,
	LDO_EN_PIN,
	HSEL_PIN,
	PVDD_PIN,
	KOTO_PWR_PIN,
	TX_PIN,
	RX_PIN,
};

static int cxd2235agg_dev_init(struct platform_device *pdev,
				struct cxd2235agg_data *my_data)
{
	int i, ret, gpio;
	unsigned int flags;
	struct device_node *of_node = pdev->dev.of_node;

	my_data->is_enable = false;

	for (i = 0; i < ARRAY_SIZE(gpio_rsrcs); i++) {
		gpio = of_get_gpio_flags(of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(&pdev->dev, "%s: invalid gpio #%s: %d\n",
				__func__, gpio_rsrcs[i], gpio);
			ret = -EINVAL;
			goto error_gpio;
		}
		my_data->gpios[i] = gpio;
	}

	for (i = 0; i < ARRAY_SIZE(req_ids); i++) {
		ret = gpio_request(my_data->gpios[req_ids[i]],
				gpio_rsrcs[req_ids[i]]);
		if (ret) {
			dev_err(&pdev->dev, "%s: request err %s: %d\n",
				__func__, gpio_rsrcs[req_ids[i]], ret);
			goto error_gpio_request;
		}
	}

	return 0;

error_gpio_request:
	for (; i >= 0; --i)
		gpio_free(my_data->gpios[req_ids[i]]);
error_gpio:
	return ret;
}

static int cxd2235agg_dev_remove(struct platform_device *pdev,
				struct cxd2235agg_data *my_data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(req_ids); i++)
		gpio_free(my_data->gpios[req_ids[i]]);

	return 0;
}

static struct qpnp_pin_cfg lock_pin_enable = {
	.mode = QPNP_PIN_MODE_DIG_IN,
	.vin_sel = QPNP_PIN_VIN0,
	.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT,
	.master_en = QPNP_PIN_MASTER_ENABLE,
};

static int cxd2235agg_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct felica_data *felica_data;
	struct cxd2235agg_data *my_data;
	struct device_node *of_node = pdev->dev.of_node;

	dev_info(&pdev->dev, "cxd2235agg driver being loaded\n");

	if (!of_node) {
		dev_err(&pdev->dev, "%s: of_node is null\n", __func__);
		ret = -EPERM;
		goto error_of_node;
	}

	my_data = kzalloc(sizeof(struct cxd2235agg_data), GFP_KERNEL);
	if (!my_data) {
		dev_err(&pdev->dev, "%s: no memory\n", __func__);
		ret = -ENOMEM;
		goto error_alloc_my_data;
	}
	felica_data = &my_data->felica_data;

	my_data->pdev = pdev;
	ret = cxd2235agg_dev_init(pdev, my_data);
	if (ret) {
		dev_err(&pdev->dev, "%s: dev init failed\n", __func__);
		goto error_dev_init;
	}

	irq = gpio_to_irq(my_data->gpios[INT_PIN]);
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: failed to get int irq: %d\n",
			__func__, irq);
		goto error_get_irq;
	}
	felica_data->irq_int = irq;
	irq = gpio_to_irq(my_data->gpios[RFS_PIN]);
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: failed to get rfs irq: %d\n",
			__func__, irq);
		goto error_get_irq;
	}
	felica_data->irq_rfs = irq;
	irq = gpio_to_irq(my_data->gpios[INTU_PIN]);
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: failed to get intu irq: %d\n",
			__func__, irq);
		goto error_get_irq;
	}
	felica_data->irq_intu = irq;

	felica_data->flcen = &cen_ops;
	felica_data->flpon = &pon_ops;
	felica_data->flrfs = &rfs_ops;
	felica_data->flint = &int_ops;
	felica_data->flintu = &intu_ops;
	felica_data->flhsel = &hsel_ops;
	felica_data->flldo = &ldo_ops;
	felica_data->type = FELICA_SNFC;
	felica_data->user = my_data;

	pdev->dev.platform_data = my_data;

	ret = felica_snfc_register(&pdev->dev, felica_data);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to felica_snfc_register\n",
			__func__);
		goto error_felica_snfc_register;
	}
	ret = qpnp_pin_config(my_data->gpios[LOCK_PIN], &lock_pin_enable);
	if (ret) {
		dev_err(&pdev->dev, "%s: LOCK set config failed: %d\n",
			__func__, ret);
		goto error_lock_pin_config;
	}

	return 0;

error_lock_pin_config:
error_felica_snfc_register:
error_get_irq:
	cxd2235agg_dev_remove(pdev, my_data);
error_dev_init:
	kfree(my_data);
error_alloc_my_data:
error_of_node:
	return ret;
}

static int cxd2235agg_remove(struct platform_device *pdev)
{
	struct cxd2235agg_data *my_data = pdev->dev.platform_data;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	felica_snfc_unregister(&pdev->dev);
	cxd2235agg_dev_remove(pdev, my_data);
	kfree(my_data);

	return 0;
}

static struct of_device_id cxd2235agg_match_table[] = {
	{	.compatible = "sony,cxd2235agg",
	},
	{}
};

static struct platform_driver cxd2235agg_driver = {
	.probe = cxd2235agg_probe,
	.remove = cxd2235agg_remove,
	.driver = {
		.name = "cxd2235agg",
		.owner = THIS_MODULE,
		.of_match_table = cxd2235agg_match_table,
	},
};

static int __init cxd2235agg_init(void)
{
	return platform_driver_register(&cxd2235agg_driver);
}

static void __exit cxd2235agg_exit(void)
{
	platform_driver_unregister(&cxd2235agg_driver);
}

module_init(cxd2235agg_init);
module_exit(cxd2235agg_exit);

MODULE_AUTHOR("Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>");
MODULE_DESCRIPTION("Sony CXD2235AGG FeliCa and NFC driver");
MODULE_LICENSE("GPL");
