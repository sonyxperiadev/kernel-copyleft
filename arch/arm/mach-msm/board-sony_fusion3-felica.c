/* arch/arm/mach-msm/board-sony_fusion3-felica.c
 *
 * Copyright (C) 2010-2011 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/felica.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/platform_device.h>
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
#include <mach/gpiomux.h>
#endif

#include "board-8064.h"

#define PM_GPIO_FELICA_LOCK	31
#define PM_GPIO_FELICA_FF	32
#define PM_GPIO_FELICA_PON	33

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
#define PM_GPIO_NFC_EXT_LDO_EN	30
#define MSM_GPIO_S_COMBO_INTU	32
#define MSM_GPIO_S_COMBO_HSEL	33
#define GSBI7_UART_TX_GPIO	82
#define GSBI7_UART_RX_GPIO	83
#endif

#define PM_GPIO_FELICA_RFS	21
#define MSM_GPIO_FELICA_INT	29

#define CEN_RETRY_MAX	5

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
static struct regulator *hvdd_reg;
static struct gpiomux_setting gsbi7_func1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting gsbi7_func2_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting unused_gpio = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir  = GPIOMUX_IN,
};
static struct pm_gpio pm_gpio_disable = {
	.direction        = PM_GPIO_DIR_IN,
	.output_buffer    = 0,
	.output_value     = 0,
	.pull             = PM_GPIO_PULL_DN,
	.vin_sel          = PM_GPIO_VIN_S4,
	.out_strength     = PM_GPIO_STRENGTH_NO,
	.function         = 0,
	.inv_int_pol      = 0,
	.disable_pin      = 0,
};
static struct pm_gpio pm_gpio_input = {
	.direction        = PM_GPIO_DIR_IN,
	.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
	.output_value     = 0,
	.pull             = PM_GPIO_PULL_NO,
	.vin_sel          = PM_GPIO_VIN_S4,
	.out_strength     = PM_GPIO_STRENGTH_NO,
	.function         = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol      = 0,
	.disable_pin      = 0,
};

static void uart_gpio_enable(struct felica_dev *dev)
{
	if (msm_gpiomux_write(GSBI7_UART_TX_GPIO, GPIOMUX_ACTIVE,
			&gsbi7_func2_cfg, NULL)) {
		dev_dbg(dev->dev, "%s: msm_gpiomux_write %d failed.",
			__func__, GSBI7_UART_TX_GPIO);
	}
	if (msm_gpiomux_write(GSBI7_UART_RX_GPIO, GPIOMUX_ACTIVE,
			&gsbi7_func1_cfg, NULL)) {
		dev_dbg(dev->dev, "%s: msm_gpiomux_write %d failed.",
			__func__, GSBI7_UART_RX_GPIO);
	}
}

static void uart_gpio_disable(struct felica_dev *dev)
{
	if (msm_gpiomux_write(GSBI7_UART_TX_GPIO, GPIOMUX_ACTIVE,
			&unused_gpio, NULL)) {
		dev_dbg(dev->dev, "%s: msm_gpiomux_write %d failed.",
			__func__, GSBI7_UART_TX_GPIO);
	}
	if (msm_gpiomux_write(GSBI7_UART_RX_GPIO, GPIOMUX_ACTIVE,
			&unused_gpio, NULL)) {
		dev_dbg(dev->dev, "%s: msm_gpiomux_write %d failed.",
			__func__, GSBI7_UART_RX_GPIO);
	}
}
#endif

static int felica_device_init(struct felica_dev *dev)
{
	int ret;
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	struct regulator *vreg;
#endif

	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);

	ret = gpio_request(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_LOCK), "felica_lock");
	if (ret) {
		dev_err(dev->dev, "%s: request err LOCK%d\n",
				__func__, ret);
		goto err_lock;
	}
	ret = gpio_request(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_FF), "felica_ff");
	if (ret) {
		dev_err(dev->dev, "%s: request err FF%d\n",
				__func__, ret);
		goto err_ff;
	}
	ret = gpio_request(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_PON), "felica_pon");
	if (ret) {
		dev_err(dev->dev, "%s: request err PON%d\n",
				__func__, ret);
		goto err_pon;
	}
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	ret = gpio_request(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_NFC_EXT_LDO_EN),
		"nfc_ext_ldo_en");
	if (ret) {
		dev_err(dev->dev, "%s: request err NFC_EXT_LDO_EN%d\n",
			__func__, ret);
		goto err_nfc_ext_ldo_en;
	}
	ret = gpio_request(MSM_GPIO_S_COMBO_INTU, "snfc_intu");
	if (ret) {
		dev_err(dev->dev, "%s: request err INTU%d\n",
			__func__, ret);
		goto err_intu;
	}
	ret = gpio_request(MSM_GPIO_S_COMBO_HSEL, "snfc_hsel");
	if (ret) {
		dev_err(dev->dev, "%s: request err HSEL%d\n",
			__func__, ret);
		goto err_hsel;
	}
#endif
	ret = gpio_request(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_RFS),
			"felica_rfs");
	if (ret) {
		dev_err(dev->dev, "%s: request err RFS%d\n",
				__func__, ret);
		goto err_rfs;
	}
	ret = gpio_request(MSM_GPIO_FELICA_INT, "felica_int");
	if (ret) {
		dev_err(dev->dev, "%s: request err INT%d\n",
				__func__, ret);
		goto err_int;
	}

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	vreg = regulator_get(dev->dev, "8921_l23");
	if (IS_ERR(vreg)) {
		dev_err(dev->dev, "%s: Get HVDD failed\n",
				__func__);
		goto err_hvdd;
	}
	ret = regulator_set_voltage(vreg, 1800000, 1800000);
	if (ret) {
		dev_err(dev->dev, "%s: VREG_L23 set voltage failed\n",
				__func__);
		regulator_put(vreg);
		goto err_hvdd;
	}

	hvdd_reg = vreg;
#endif

	return 0;

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
err_hvdd:
	gpio_free(MSM_GPIO_FELICA_INT);
#endif
err_int:
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_RFS));
err_rfs:
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	gpio_free(MSM_GPIO_S_COMBO_HSEL);
err_hsel:
	gpio_free(MSM_GPIO_S_COMBO_INTU);
err_intu:
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_NFC_EXT_LDO_EN));
err_nfc_ext_ldo_en:
#endif
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_PON));
err_pon:
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_FF));
err_ff:
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_LOCK));
err_lock:
	return -ENODEV;
}

static int felica_device_release(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	gpio_free(MSM_GPIO_S_COMBO_INTU);
	gpio_free(MSM_GPIO_S_COMBO_HSEL);
#endif
	gpio_free(MSM_GPIO_FELICA_INT);
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_RFS));
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_PON));
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_FF));
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_LOCK));
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	gpio_free(PM8921_GPIO_PM_TO_SYS(PM_GPIO_NFC_EXT_LDO_EN));
#endif

	return 0;
}

static int felica_cen_init(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->device_cen.this_device, ": %s\n", __func__);

	return 0;
}

static int felica_cen_read(u8 *buf, struct felica_dev *dev)
{
	int st = 0;

	if (!buf || !dev)
		return -EINVAL;
	dev_dbg(dev->device_cen.this_device, ": %s\n", __func__);

	st = gpio_get_value_cansleep(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_LOCK));

	dev_dbg(dev->device_cen.this_device,
			": FELICA_LOCK = 0x%x\n", st);
	*buf = (st) ? 0x1 : 0x0;

	return 0;
}

static int felica_cen_write(u8 arg, struct felica_dev *dev)
{
	int i;
	u8 state;

	if (!dev)
		return -EINVAL;
	dev_dbg(dev->device_cen.this_device, ": %s\n", __func__);

	if (arg > 0x1) {
		dev_err(dev->device_cen.this_device,
			"%s: Error. Invalid val @CEN write.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < CEN_RETRY_MAX; i++) {
		felica_cen_read(&state, dev);
		if (arg == state)
			goto end;
		gpio_set_value_cansleep(
			PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_FF), 0);
		msleep_interruptible(1);
		gpio_set_value_cansleep(
			PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_FF), 1);
		msleep_interruptible(1);
		gpio_set_value_cansleep(
			PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_FF), 0);
	}
	dev_err(dev->device_cen.this_device,
			"%s: Error. Cannot write CEN.\n", __func__);
	return -EIO;

end:
	return 0;
}

static int felica_pon_init(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->device_pon.this_device, ": %s\n", __func__);

	return 0;
}

static void felica_pon_write(int val, struct felica_dev *dev)
{
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	int ret;
#endif

	if (!dev)
		return;
	dev_dbg(dev->device_pon.this_device, ": %s\n", __func__);

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	if (val && !regulator_is_enabled(hvdd_reg)) {
		ret = regulator_enable(hvdd_reg);
		if (ret) {
			dev_err(dev->device_pon.this_device,
				"%s: vreg L23 enable failed :%d\n",
				 __func__, ret);
			goto exit;
		}
		ret = regulator_set_optimum_mode(hvdd_reg, 5000);
		if (ret < 0) {
			regulator_disable(hvdd_reg);
			dev_err(dev->device_pon.this_device,
				"%s: Error. Cannot set LPM:%d\n",
				 __func__, ret);
			goto exit;
		}
		msleep_interruptible(1);
		pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_RFS),
			&pm_gpio_input);
		uart_gpio_enable(dev);
	}
exit:
#endif

	gpio_set_value_cansleep(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_PON), val);
}

static void felica_pon_release(struct felica_dev *dev)
{
	if (!dev)
		return;
	dev_dbg(dev->device_pon.this_device, ": %s\n", __func__);
}

static int felica_tvdd_on(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);

	return 0;
}

static void felica_tvdd_off(struct felica_dev *dev)
{
	if (!dev)
		return;
	dev_dbg(dev->dev, ": %s\n", __func__);
}

static int felica_rfs_init(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->device_rfs.this_device, ": %s\n", __func__);

	return 0;
}

static int felica_rfs_read(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->device_rfs.this_device, ": %s\n", __func__);

	return gpio_get_value(PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_RFS));
}

static void felica_rfs_release(struct felica_dev *dev)
{
	if (!dev)
		return;
	dev_dbg(dev->device_rfs.this_device, ": %s\n", __func__);
}

static int felica_int_init(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);

	return 0;
}

static int felica_int_read(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);

	return gpio_get_value(MSM_GPIO_FELICA_INT);
}

static void felica_int_release(struct felica_dev *dev)
{
	if (!dev)
		return;
	dev_dbg(dev->dev, ": %s\n", __func__);
}

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
static int snfc_intu_init(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);
	return 0;
}

static int snfc_intu_read(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);
	return gpio_get_value(MSM_GPIO_S_COMBO_INTU);
}

static void snfc_intu_release(struct felica_dev *dev)
{
	if (!dev)
		return;
	dev_dbg(dev->dev, ": %s\n", __func__);
}

static int snfc_hsel_init(struct felica_dev *dev)
{
	if (!dev)
		return -EINVAL;
	dev_dbg(dev->dev, ": %s\n", __func__);
	return 0;
}

static void snfc_hsel_write(int val, struct felica_dev *dev)
{
	if (!dev)
		return;
	dev_dbg(dev->dev, ": %s\n", __func__);
	gpio_set_value_cansleep(MSM_GPIO_S_COMBO_HSEL, val);
}

static void snfc_hsel_release(struct felica_dev *dev)
{
	if (!dev)
		return;
	dev_dbg(dev->dev, ": %s\n", __func__);
}

static void snfc_ldo_write(int val, struct felica_dev *dev)
{
	int ret;

	if (!dev)
		return;
	dev_dbg(dev->dev, ": %s\n", __func__);

	if (!val && regulator_is_enabled(hvdd_reg)) {
		dev_dbg(dev->dev, ": %s HVDD off\n", __func__);
		uart_gpio_disable(dev);
		gpio_free(MSM_GPIO_S_COMBO_HSEL);
		gpio_free(MSM_GPIO_S_COMBO_INTU);
		gpio_free(MSM_GPIO_FELICA_INT);
		ret = pm8xxx_gpio_config(
			PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_RFS),
			&pm_gpio_disable);
		if (ret)
			dev_err(dev->dev, "%s: Error pm8xxx_gpio_config RFS:%d\n",
				__func__, ret);
		ret = pm8xxx_gpio_config(
			PM8921_GPIO_PM_TO_SYS(PM_GPIO_FELICA_PON),
			&pm_gpio_disable);
		if (ret)
			dev_err(dev->dev, "%s: Error pm8xxx_gpio_config PON:%d\n",
				__func__, ret);
		regulator_disable(hvdd_reg);
		msleep_interruptible(100);
	}

	gpio_set_value_cansleep(
		PM8921_GPIO_PM_TO_SYS(PM_GPIO_NFC_EXT_LDO_EN), val);
}
#endif

static struct felica_platform_data felica_pfdata = {
	.cen_pfdata = {
		.cen_init = felica_cen_init,
		.cen_read = felica_cen_read,
		.cen_write = felica_cen_write,
	},
	.pon_pfdata = {
		.pon_init = felica_pon_init,
		.pon_write = felica_pon_write,
		.pon_release = felica_pon_release,
		.tvdd_on = felica_tvdd_on,
		.tvdd_off = felica_tvdd_off,
	},
	.rfs_pfdata = {
		.rfs_init = felica_rfs_init,
		.rfs_read = felica_rfs_read,
		.rfs_release = felica_rfs_release,
		.irq_rfs = PM8921_GPIO_IRQ(PM8921_IRQ_BASE, PM_GPIO_FELICA_RFS),
	},
	.int_pfdata = {
		.int_init = felica_int_init,
		.int_read = felica_int_read,
		.int_release = felica_int_release,
		.irq_int = MSM_GPIO_TO_INT(MSM_GPIO_FELICA_INT),
	},
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	.intu_pfdata = {
		.intu_init = snfc_intu_init,
		.intu_read = snfc_intu_read,
		.intu_release = snfc_intu_release,
		.irq_intu = MSM_GPIO_TO_INT(MSM_GPIO_S_COMBO_INTU),
	},
	.hsel_pfdata = {
		.hsel_init = snfc_hsel_init,
		.hsel_write = snfc_hsel_write,
		.hsel_release = snfc_hsel_release,
	},
	.ldo_pfdata = {
		.ldo_write = snfc_ldo_write,
	},
#endif
	.gpio_init = felica_device_init,
	.reg_release = felica_device_release,
};

struct platform_device sony_felica_device = {
	.name = FELICA_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &felica_pfdata,
	},
};
