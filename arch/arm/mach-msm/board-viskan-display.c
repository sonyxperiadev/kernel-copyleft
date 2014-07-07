/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Sony Mobile Communications AB.
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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/msm_ion.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/ion.h>
#include <mach/socinfo.h>

#include "devices.h"
#include "board-8960.h"
#include "board-viskan-display.h"

#ifdef CONFIG_FB_MSM_MHL_SII8334
#include <linux/mhl_sii8334.h>
#endif

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_PRIM_BUF_SIZE \
		(roundup((roundup(1920, 32) * roundup(1200, 32) * 4), 4096) * 3)
			/* 4 bpp x 3 pages */
#else
#define MSM_FB_PRIM_BUF_SIZE \
		(roundup((roundup(1920, 32) * roundup(1200, 32) * 4), 4096) * 2)
			/* 4 bpp x 2 pages */
#endif

/* Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE, 4096)

#ifdef CONFIG_FB_MSM_OVERLAY0_WRITEBACK
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE \
		roundup((roundup(1920, 32) * roundup(1200, 32) * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY0_WRITEBACK */

#ifdef CONFIG_FB_MSM_OVERLAY1_WRITEBACK
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE \
		roundup((roundup(1920, 32) * roundup(1080, 32) * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY1_WRITEBACK */

#define MDP_VSYNC_GPIO 0

#define HDMI_PANEL_NAME	"hdmi_msm"
#define TVOUT_PANEL_NAME	"tvout_msm"

#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
static unsigned char hdmi_is_primary = 1;
#else
static unsigned char hdmi_is_primary;
#endif

unsigned char msm8960_hdmi_as_primary_selected(void)
{
	return hdmi_is_primary;
}

static struct resource msm_fb_resources[] = {
	{
		.flags = IORESOURCE_DMA,
	}
};

static void set_mdp_clocks_for_wuxga(void);

static int msm_fb_detect_panel(const char *name)
{
	if (!strncmp(name, HDMI_PANEL_NAME,
			strnlen(HDMI_PANEL_NAME,
				PANEL_NAME_MAX_LEN))) {
		if (hdmi_is_primary)
			set_mdp_clocks_for_wuxga();
			return 0;
	}

	if (!strncmp(name, TVOUT_PANEL_NAME,
			strnlen(TVOUT_PANEL_NAME,
				PANEL_NAME_MAX_LEN)))
		return 0;
	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

#define MLCD_RESET_N 43
#define LCD_PWR_EN 36
#define LCD_VREG_ON_WAIT_MS 10
#define LCD_RESET_WAIT_MS 10
#define LCD_POWER_WAIT_MS 50

static struct pm_gpio gpio43_param = {
	.direction = PM_GPIO_DIR_OUT,
	.output_buffer = PM_GPIO_OUT_BUF_CMOS,
	.output_value = 0,
	.pull = PM_GPIO_PULL_NO,
	.vin_sel = PM_GPIO_VIN_S4,
	.out_strength = PM_GPIO_STRENGTH_LOW,
	.function = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol = 0,
	.disable_pin = 0,
};

static int gpio_lcd_reset;
static struct regulator *vreg_lcd_vci;
static struct regulator *vreg_lcd_vddio;

static int lcd_gpio_setup(int request)
{
	int rc = 0;

	if (!gpio_lcd_reset)
		gpio_lcd_reset = PM8921_GPIO_PM_TO_SYS(MLCD_RESET_N);

	if (request) {
		/* LCD_PWR_EN */
		rc = gpio_request(LCD_PWR_EN, "lcd power gpio");
		if (rc) {
			pr_err("%s: GPIO %d: request failed. rc=%d\n",
				__func__, LCD_PWR_EN, rc);
			return rc;
		}
		rc = gpio_direction_output(LCD_PWR_EN, 0);
		if (rc) {
			pr_err("%s: GPIO %d: direction out failed. rc=%d\n",
				__func__, LCD_PWR_EN, rc);
			goto out_pwr;
		}

		rc = pm8xxx_gpio_config(gpio_lcd_reset, &gpio43_param);
		if (rc) {
			pr_err("gpio_config 43 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		/* gpio_lcd_reset */
		rc = gpio_request(gpio_lcd_reset, "lcd reset");
		if (rc) {
			pr_err("%s: GPIO %d: request failed. rc=%d\n",
				__func__, gpio_lcd_reset, rc);
			goto out_pwr;
		}
	} else {
		gpio_free(LCD_PWR_EN);
		gpio_free(gpio_lcd_reset);
		gpio_lcd_reset = 0;
	}

	return rc;
out_pwr:
	gpio_free(LCD_PWR_EN);
	return rc;
}

static int lcd_power(int on)
{
	if (on)
		gpio_set_value(LCD_PWR_EN, 1);
	else
		gpio_set_value(LCD_PWR_EN, 0);
	msleep(LCD_POWER_WAIT_MS);

	return 0;
}

static int lcd_reset(int on)
{
	int rc;

	if (!gpio_lcd_reset) {
		rc = lcd_gpio_setup(1);
		if (rc) {
			pr_err("gpio setup failed , rc=%d\n", rc);
			return -EINVAL;
		}
	}

	if (on) {
		gpio_set_value_cansleep(gpio_lcd_reset, 0);
		gpio_set_value_cansleep(gpio_lcd_reset, 1);
	} else {
		gpio_set_value_cansleep(gpio_lcd_reset, 1);
		gpio_set_value_cansleep(gpio_lcd_reset, 0);
	}
	msleep(LCD_RESET_WAIT_MS);

	return 0;
}

/*
 * MIPI_DSI only use 8058_LDO2 which need always on
 * therefore it need to be put at low power mode if
 * it was not used instead of turn it off.
 */
static int mipi_dsi_power(int on)
{
	static int curr_power;
	static struct regulator *ldo2;
	int rc;

	if (curr_power == on)
		return 0;

	if (ldo2 == NULL) {	/* init */
		ldo2 = regulator_get(&msm_mipi_dsi1_device.dev,
			"dsi_vdda");
		if (IS_ERR(ldo2)) {
			pr_err("%s: dsi_vdda failed\n", __func__);
			rc = PTR_ERR(ldo2);
			return rc;
		}
	}

	if (on) {
		rc = regulator_set_voltage(ldo2, 1200000, 1200000);
		if (rc) {
			pr_err("%s: Unable to set voltage level for dsi_vdda\n",
				__func__);
			goto out_put;
		}

		rc = regulator_enable(ldo2);
		if (rc) {
			pr_err("%s: Unable to enable dsi_vdda\n", __func__);
			goto out_put;
		}
	} else {
		rc = regulator_disable(ldo2);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "dsi_vdda", rc);
	}

	curr_power = on;

	return 0;
out_put:
	regulator_put(ldo2);
	ldo2 = NULL;
	return rc;
}

#ifdef CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET

#define LCD_VENDOR_DET 66
#define LCD_VENDOR_SAMSUNG 1
#define LCD_VENDOR_RENESAS 0
#define LCD_VENDOR_DET_WAIT_MS 50
#define LCD_VENDOR_DISABLE_VREG_WAIT_MS 10

static int sony_viskan_is_dric_det(void)
{
	int rc = 0;
	static int val = -ENODEV;

	if (val != -ENODEV)
		return val;

	if (!vreg_lcd_vci) {
		vreg_lcd_vci = regulator_get(NULL, "8921_l8");
		if (IS_ERR(vreg_lcd_vci)) {
			pr_err("%s: Unable to get 8921_l8\n", __func__);
			vreg_lcd_vci = NULL;
			return -ENODEV;
		}
	}

	if (!vreg_lcd_vddio) {
		vreg_lcd_vddio = regulator_get(NULL, "8921_l29");
		if (IS_ERR(vreg_lcd_vddio)) {
			pr_err("%s: Unable to get 8921_l29\n", __func__);
			vreg_lcd_vddio = NULL;
			goto out_put_l8;
		}
	}

	rc = regulator_set_voltage(vreg_lcd_vci, 2850000, 2850000);
	if (rc) {
		pr_err("%s:%d unable to set L8 voltage to 2.85V\n",
			__func__, rc);
		goto out_put_l29;
	}

	rc = regulator_set_voltage(vreg_lcd_vddio, 1800000, 1800000);
	if (rc) {
		pr_err("%s:%d unable to set L29 voltage to 1.8V\n",
			__func__, rc);
		goto out_put_l29;
	}

	rc = regulator_enable(vreg_lcd_vci);
	if (rc) {
		pr_err("%s: Enable regulator 8921_l8 failed\n",
			__func__);
		goto out_put_l29;
	}

	rc = regulator_enable(vreg_lcd_vddio);
	if (rc) {
		pr_err("%s: Enable regulator 8921_l29 failed\n",
			__func__);
		goto out_put_l29;
	}

	lcd_gpio_setup(true);

	msleep(LCD_VENDOR_DET_WAIT_MS);

	/* LCD_VENDOR_DET */
	rc = gpio_request(LCD_VENDOR_DET, "lcd vendor detect");
	if (rc) {
		pr_err("%s: GPIO %d: request failed. rc=%d\n",
			__func__, LCD_VENDOR_DET, rc);
		goto out_put_l29;
	}
	rc = gpio_direction_input(LCD_VENDOR_DET);
	if (rc) {
		pr_err("%s: GPIO %d: direction in failed. rc=%d\n",
			__func__, LCD_VENDOR_DET, rc);
		goto out_free;
	}

	val = gpio_get_value(LCD_VENDOR_DET);
	pr_info("%s: GPIO:%d\n", __func__, val);

	rc = regulator_disable(vreg_lcd_vddio);
	if (rc)
		pr_err("%s: Enable regulator 8921_l29 failed\n",
			__func__);
	rc = regulator_disable(vreg_lcd_vci);
	if (rc)
		pr_err("%s: Enable regulator 8921_l8 failed\n",
			__func__);

	lcd_gpio_setup(false);

out_free:
	gpio_free(LCD_VENDOR_DET);
out_put_l29:
	regulator_put(vreg_lcd_vddio);
	vreg_lcd_vddio = NULL;
out_put_l8:
	regulator_put(vreg_lcd_vci);
	vreg_lcd_vci = NULL;

	msleep(LCD_VENDOR_DISABLE_VREG_WAIT_MS);

	return val;
}

#endif /* CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET */

#ifdef CONFIG_FB_MSM_MIPI_DSI_RENESAS_R63306

#define MIPI_DSI_RENESAS_NAME "mipi_renesas_r63306"

static int r63306_vreg_power(int on)
{
	int rc = 0;

	if (!vreg_lcd_vci) {
		vreg_lcd_vci = regulator_get(&msm_mipi_dsi1_device.dev,
			"dsi_vci");
		if (IS_ERR(vreg_lcd_vci)) {
			pr_err("could not get dsi_vci, rc = %ld\n",
				PTR_ERR(vreg_lcd_vci));
			return -ENODEV;
		}
	}

	if (!vreg_lcd_vddio) {
		vreg_lcd_vddio = regulator_get(&msm_mipi_dsi1_device.dev,
			"dsi_vddio");
		if (IS_ERR(vreg_lcd_vddio)) {
			pr_err("%s: Unable to get dsi_vddio\n", __func__);
			vreg_lcd_vddio = NULL;
			goto out_put;
		}
	}

	if (on) {
		rc = regulator_set_voltage(vreg_lcd_vci, 2850000, 2850000);
		if (rc) {
			pr_err("%s:%d unable to set dsi_vci voltage to 2.8V\n",
				__func__, rc);
			goto out_put_all;
		}

		rc = regulator_enable(vreg_lcd_vci);
		if (rc) {
			pr_err("%s: Enable regulator dsi_vci failed\n",
				__func__);
			goto out_put_all;
		}

		rc = regulator_set_voltage(vreg_lcd_vddio, 1800000, 1800000);
		if (rc) {
			pr_err("%s:%d unable to set dsi_vddio voltage to 1.8V\n",
				__func__, rc);
			goto out_disable;
		}

		rc = regulator_enable(vreg_lcd_vddio);
		if (rc) {
			pr_err("%s: Enable regulator dsi_vddio failed\n",
				__func__);
			goto out_disable;
		}

		rc = lcd_gpio_setup(on);
		if (rc) {
			pr_err("gpio setup failed , rc=%d\n", rc);
			goto out_disable_all;
		}

		msleep(LCD_VREG_ON_WAIT_MS);

		lcd_reset(on);
	} else {
		msleep(LCD_RESET_WAIT_MS);
		lcd_reset(on);

		rc = lcd_gpio_setup(on);
		if (rc) {
			pr_err("gpio setup failed , rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_disable(vreg_lcd_vddio);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "dsi_vddio", rc);
		rc = regulator_disable(vreg_lcd_vci);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "dsi_vci", rc);
	}

	return 0;
out_disable_all:
	regulator_disable(vreg_lcd_vddio);
out_disable:
	regulator_disable(vreg_lcd_vci);
out_put_all:
	regulator_put(vreg_lcd_vddio);
	vreg_lcd_vddio = NULL;
out_put:
	regulator_put(vreg_lcd_vci);
	vreg_lcd_vci = NULL;
	return rc;
}

static int r63306_lcd_power(int on)
{
	static int curr_power;
	int rc;

	if (curr_power == on)
		return 0;

	if (on) {
		rc = r63306_vreg_power(on);
		if (!rc)
			rc = mipi_dsi_power(on);
	} else {
		rc = mipi_dsi_power(on);
		if (!rc)
			rc = r63306_vreg_power(on);
	}

	if (!rc)
		curr_power = on;

	return rc;
}

static const struct panel_id *default_panel_ids_r63306[] = {
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01
	&sharp_ls046k3sy01_panel_default,
#endif /*CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01*/
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01
	&sharp_ls043k3sx01_panel_default_old,
	&sharp_ls043k3sx01_panel_default,
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04
	&sharp_ls043k3sx04_panel_default,
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04 */
	NULL,
};

static const struct panel_id *panel_ids_r63306[] = {
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01
	&sharp_ls046k3sy01_panel_id_1a,
	&sharp_ls046k3sy01_panel_id,
#endif /*CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS046K3SY01*/
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01
	&sharp_ls043k3sx01_panel_id_old,
	&sharp_ls043k3sx01_panel_id_1a,
	&sharp_ls043k3sx01_panel_id,
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX01 */
#ifdef CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04
	&sharp_ls043k3sx04_panel_id_1a,
	&sharp_ls043k3sx04_panel_id,
#endif /* CONFIG_FB_MSM_MIPI_R63306_PANEL_SHARP_LS043K3SX04 */
	NULL,
};

#endif /* CONFIG_FB_MSM_MIPI_DSI_RENESAS_R63306 */


#ifdef CONFIG_FB_MSM_MIPI_DSI_SAMSUNG_S6D6AA0

#define MIPI_DSI_SAMSUNG_NAME "mipi_samsung_s6d6aa0"

static int s6d6aa0_vreg_power(int on)
{
	int rc = 0;

	if (!vreg_lcd_vddio) {
		vreg_lcd_vddio = regulator_get(&msm_mipi_dsi1_device.dev,
			"dsi_vddio");
		if (IS_ERR(vreg_lcd_vddio)) {
			pr_err("%s: Unable to get 8921_l29\n", __func__);
			vreg_lcd_vddio = NULL;
			return -ENODEV;
		}
	}

	if (on) {
		rc = regulator_set_voltage(vreg_lcd_vddio, 1800000, 1800000);
		if (rc) {
			pr_err("%s:%d unable to set L29 voltage to 1.8V\n",
				__func__, rc);
			goto out_put;
		}

		rc = regulator_enable(vreg_lcd_vddio);
		if (rc) {
			pr_err("%s: Enable regulator 8921_l29 failed\n",
				__func__);
			goto out_put;
		}

		rc = lcd_gpio_setup(on);
		if (rc) {
			pr_err("gpio setup failed , rc=%d\n", rc);
			goto out_desable;
		}
		gpio_set_value(LCD_PWR_EN, 1);

		msleep(LCD_POWER_WAIT_MS);

		lcd_reset(on);
	} else {
		lcd_reset(on);

		gpio_set_value(LCD_PWR_EN, 0);
		rc = lcd_gpio_setup(on);
		if (rc) {
			pr_err("gpio setup failed , rc=%d\n", rc);
			return -EINVAL;
		}

		rc = regulator_disable(vreg_lcd_vddio);
		if (rc)
			pr_warning("%s: '%s' regulator disable failed, rc=%d\n",
				__func__, "8921_l29", rc);
		msleep(LCD_POWER_WAIT_MS);
	}

	return 0;
out_desable:
	regulator_disable(vreg_lcd_vddio);
out_put:
	regulator_put(vreg_lcd_vddio);
	vreg_lcd_vddio = NULL;
	return rc;
}

static int s6d6aa0_lcd_power(int on)
{
	static int curr_power;
	int rc;

	if (curr_power == on)
		return 0;

	if (on) {
		rc = s6d6aa0_vreg_power(on);
		if (!rc)
			rc = mipi_dsi_power(on);
	} else {
		rc = mipi_dsi_power(on);
		if (!rc)
			rc = s6d6aa0_vreg_power(on);
	}

	if (!rc)
		curr_power = on;

	return rc;
}

static const struct panel_id *default_panel_ids_s6d6aa0[] = {
#ifdef CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01
	&auo_h455tvn01_panel_default,
#endif /* CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01 */
	NULL,
};

static const struct panel_id *panel_ids_s6d6aa0[] = {
#ifdef CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01
	&auo_h455tvn01_panel_id_1a,
	&auo_h455tvn01_panel_id,
#endif /* CONFIG_FB_MSM_MIPI_S6D6AA0_PANEL_AUO_H455TVN01 */
	NULL,
};

#endif /* CONFIG_FB_MSM_MIPI_DSI_SAMSUNG_S6D6AA0 */


#ifdef CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET

const struct panel_id **default_panel_ids;
const struct panel_id **panel_ids;

static struct mipi_dsi_platform_data mipi_dsi_pdata;

static struct platform_device sony_viskan_lcd_device = {
	.id = 0,
};
#else /* CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET */

#ifdef CONFIG_FB_MSM_MIPI_DSI_RENESAS_R63306

const struct panel_id **default_panel_ids = default_panel_ids_r63306;
const struct panel_id **panel_ids = panel_ids_r63306;

static struct platform_device sony_viskan_lcd_device = {
	.name = MIPI_DSI_RENESAS_NAME,
	.id = 0,
};

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.dsi_power_save   = r63306_lcd_power,
};
#endif /* CONFIG_FB_MSM_MIPI_DSI_RENESAS_R63306 */

#ifdef CONFIG_FB_MSM_MIPI_DSI_SAMSUNG_S6D6AA0

const struct panel_id **default_panel_ids = default_panel_ids_s6d6aa0;
const struct panel_id **panel_ids = panel_ids_s6d6aa0;

static struct platform_device sony_viskan_lcd_device = {
	.name = MIPI_DSI_SAMSUNG_NAME,
	.id = 0,
};

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.dsi_power_save   = s6d6aa0_lcd_power,
};
#endif /* CONFIG_FB_MSM_MIPI_DSI_SAMSUNG_S6D6AA0 */

#endif /* CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET */

void __init sony_viskan_add_lcd_device(void)
{
	int rc;
	struct lcd_panel_platform_data *pdata;

	pdata = kmalloc(sizeof(struct lcd_panel_platform_data), GFP_KERNEL);

	pdata->default_panels = default_panel_ids;
	pdata->panels = panel_ids;
	pdata->lcd_power = lcd_power;
	pdata->lcd_reset = lcd_reset;

#ifdef CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET
	if (sony_viskan_is_dric_det() == LCD_VENDOR_SAMSUNG) {
		sony_viskan_lcd_device.name = MIPI_DSI_SAMSUNG_NAME;
		pdata->default_panels = default_panel_ids_s6d6aa0;
		pdata->panels = panel_ids_s6d6aa0;
	} else {
		sony_viskan_lcd_device.name = MIPI_DSI_RENESAS_NAME;
		pdata->default_panels = default_panel_ids_r63306;
		pdata->panels = panel_ids_r63306;
	}
#endif /* CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET */

	sony_viskan_lcd_device.dev.platform_data = pdata;
	rc = platform_device_register(&sony_viskan_lcd_device);
	if (rc)
		dev_err(&sony_viskan_lcd_device.dev,
			"%s: platform_device_register() failed = %d\n",
			__func__, rc);

}

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_ui_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 304128000,
		.ib = 304128000 * 1.25 * 2,
	},
};

static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 248832000,
		.ib = 248832000 * 1.5 * 2,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 304128000,
		.ib = 304128000 * 1.5 * 2,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 407808000,
		.ib = 407808000 * 1.5 * 2,
	},
};

static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};

static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

#endif

static int mdp_core_clk_rate_table[] = {
	85330000,
	128000000,
	200000000,
	200000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = MDP_VSYNC_GPIO,
	.mdp_max_clk = 200000000,
	.mdp_max_bw = 2000000000,
	.mdp_bw_ab_factor = 115,
	.mdp_bw_ib_factor = 150,
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
	.mdp_rev = MDP_REV_42,
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	.mem_hid = BIT(ION_CP_MM_HEAP_ID),
#else
	.mem_hid = MEMTYPE_EBI1,
#endif

	.mdp_iommu_split_domain = 0,
};

void __init msm8960_mdp_writeback(struct memtype_reserve* reserve_table)
{
	mdp_pdata.ov0_wb_size = MSM_FB_OVERLAY0_WRITEBACK_SIZE;
	mdp_pdata.ov1_wb_size = MSM_FB_OVERLAY1_WRITEBACK_SIZE;
#if defined(CONFIG_ANDROID_PMEM) && !defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov0_wb_size;
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov1_wb_size;

	pr_info("mem_map: mdp reserved with size 0x%lx in pool\n",
			mdp_pdata.ov0_wb_size + mdp_pdata.ov1_wb_size);
#endif
}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct resource hdmi_msm_resources[] = {
	{
		.name  = "hdmi_msm_qfprom_addr",
		.start = 0x00700000,
		.end   = 0x007060FF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_hdmi_addr",
		.start = 0x04A00000,
		.end   = 0x04A00FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_irq",
		.start = HDMI_IRQ,
		.end   = HDMI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on, int show);
static int hdmi_cec_power(int on);
static int hdmi_gpio_config(int on);
static int hdmi_panel_power(int on);

static struct msm_hdmi_platform_data hdmi_msm_data = {
	.irq = HDMI_IRQ,
#ifdef CONFIG_FB_MSM_MHL_SII8334
	.coupled_mhl_device = SII_DEV_NAME,
#endif
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
	.cec_power = hdmi_cec_power,
	.panel_power = hdmi_panel_power,
	.gpio_config = hdmi_gpio_config,
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
	.dev.platform_data = &hdmi_msm_data,
};
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
static struct platform_device wfd_device = {
	.name          = "msm_wfd",
	.id            = -1,
};
#endif

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 566092800 * 2,
		.ib = 707616000 * 2,
	},
};

static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
	.lcdc_power_save = hdmi_panel_power,
};

static int hdmi_panel_power(int on)
{
	int rc;

	pr_debug("%s: HDMI Core: %s\n", __func__, (on ? "ON" : "OFF"));
	rc = hdmi_core_power(on, 1);
	if (rc)
		rc = hdmi_cec_power(on);

	pr_debug("%s: HDMI Core: %s Success\n", __func__, (on ? "ON" : "OFF"));
	return rc;
}
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static int hdmi_enable_5v(int on)
{
	return 0;
}

static int hdmi_core_power(int on, int show)
{
	static struct regulator *reg_8921_l23, *reg_8921_s4;
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	/* TBD: PM8921 regulator instead of 8901 */
	if (!reg_8921_l23) {
		reg_8921_l23 = regulator_get(&hdmi_msm_device.dev, "hdmi_avdd");
		if (IS_ERR(reg_8921_l23)) {
			pr_err("could not get reg_8921_l23, rc = %ld\n",
				PTR_ERR(reg_8921_l23));
			return -ENODEV;
		}
		rc = regulator_set_voltage(reg_8921_l23, 1800000, 1800000);
		if (rc) {
			pr_err("set_voltage failed for 8921_l23, rc=%d\n", rc);
			return -EINVAL;
		}
	}
	if (!reg_8921_s4) {
		reg_8921_s4 = regulator_get(&hdmi_msm_device.dev, "hdmi_vcc");
		if (IS_ERR(reg_8921_s4)) {
			pr_err("could not get reg_8921_s4, rc = %ld\n",
				PTR_ERR(reg_8921_s4));
			return -ENODEV;
		}
		rc = regulator_set_voltage(reg_8921_s4, 1800000, 1800000);
		if (rc) {
			pr_err("set_voltage failed for 8921_s4, rc=%d\n", rc);
			return -EINVAL;
		}
	}

	if (on) {
		rc = regulator_set_optimum_mode(reg_8921_l23, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_enable(reg_8921_l23);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"hdmi_avdd", rc);
			return rc;
		}
		rc = regulator_enable(reg_8921_s4);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"hdmi_vcc", rc);
			return rc;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8921_l23);
		if (rc) {
			pr_err("disable reg_8921_l23 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_8921_s4);
		if (rc) {
			pr_err("disable reg_8921_s4 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_set_optimum_mode(reg_8921_l23, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_gpio_config(int on)
{
	int rc = 0;
	static int prev_on;

	if (on == prev_on)
		return 0;

	if (on) {
		rc = gpio_request(100, "HDMI_DDC_CLK");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_CLK", 100, rc);
			return rc;
		}
		rc = gpio_request(101, "HDMI_DDC_DATA");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_DATA", 101, rc);
			goto error1;
		}
		rc = gpio_request(102, "HDMI_HPD");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_HPD", 102, rc);
			goto error2;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		gpio_free(100);
		gpio_free(101);
		gpio_free(102);
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;
	return 0;

error2:
	gpio_free(101);
error1:
	gpio_free(100);
	return rc;
}

static int hdmi_cec_power(int on)
{
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (on) {
		rc = gpio_request(99, "HDMI_CEC_VAR");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_CEC_VAR", 99, rc);
			goto error;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		gpio_free(99);
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
error:
	return rc;
}
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

void __init msm8960_init_fb(void)
{
	uint32_t soc_platform_version = socinfo_get_version();


	if (SOCINFO_VERSION_MAJOR(soc_platform_version) >= 3)
		mdp_pdata.mdp_rev = MDP_REV_43;

	if (cpu_is_msm8960ab())
		mdp_pdata.mdp_rev = MDP_REV_44;

	sony_viskan_add_lcd_device();
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	platform_device_register(&hdmi_msm_device);
#endif
	platform_device_register(&msm_fb_device);
#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
	platform_device_register(&wfd_device);
#endif
	msm_fb_register_device("mdp", &mdp_pdata);
#ifdef CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET
	if (sony_viskan_is_dric_det() == LCD_VENDOR_SAMSUNG)
		mipi_dsi_pdata.dsi_power_save = s6d6aa0_lcd_power;
	else
		mipi_dsi_pdata.dsi_power_save = r63306_lcd_power;
#endif /* CONFIG_FB_MSM_MIPI_DSI_VENDOR_DET */
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
}

void __init msm8960_allocate_fb_region(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
}

/**
 * Set MDP clocks to high frequency to avoid DSI underflow
 * when using high resolution 1200x1920 WUXGA panels
 */
static void set_mdp_clocks_for_wuxga(void)
{
	int i;

	mdp_ui_vectors[0].ab = 2000000000;
	mdp_ui_vectors[0].ib = 2000000000;
	mdp_vga_vectors[0].ab = 2000000000;
	mdp_vga_vectors[0].ib = 2000000000;
	mdp_720p_vectors[0].ab = 2000000000;
	mdp_720p_vectors[0].ib = 2000000000;
	mdp_1080p_vectors[0].ab = 2000000000;
	mdp_1080p_vectors[0].ib = 2000000000;


	for (i = 0; i < ARRAY_SIZE(mdp_core_clk_rate_table); i++)
		mdp_core_clk_rate_table[i] = 200000000;

	if (hdmi_is_primary) {
		dtv_bus_def_vectors[0].ab = 2000000000;
		dtv_bus_def_vectors[0].ib = 2000000000;
	}
}

void __init msm8960_set_display_params(char *prim_panel, char *ext_panel)
{
	int disable_splash = 0;
	if (strnlen(prim_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.prim_panel_name, prim_panel,
			PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.prim_panel_name %s\n",
			msm_fb_pdata.prim_panel_name);

		if (!strncmp((char *)msm_fb_pdata.prim_panel_name,
			HDMI_PANEL_NAME, strnlen(HDMI_PANEL_NAME,
				PANEL_NAME_MAX_LEN))) {
			pr_debug("HDMI is the primary display by"
				" boot parameter\n");
			hdmi_is_primary = 1;
			set_mdp_clocks_for_wuxga();
		}

	}
	if (strnlen(ext_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.ext_panel_name, ext_panel,
			PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.ext_panel_name %s\n",
			msm_fb_pdata.ext_panel_name);
	}

	if (disable_splash)
		mdp_pdata.cont_splash_enabled = 0;
}
