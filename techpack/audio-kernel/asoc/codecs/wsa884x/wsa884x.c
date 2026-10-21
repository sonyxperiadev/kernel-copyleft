// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <soc/soundwire.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <asoc/msm-cdc-pinctrl.h>
#include <asoc/msm-cdc-supply.h>
#include "wsa884x-registers.h"
#include "wsa884x.h"
#include "internal.h"
#include "asoc/bolero-slave-internal.h"
#include <linux/qti-regmap-debugfs.h>

#define T1_TEMP -10
#define T2_TEMP 150
#define LOW_TEMP_THRESHOLD 5
#define HIGH_TEMP_THRESHOLD 45
#define TEMP_INVALID	0xFFFF
#define WSA884X_TEMP_RETRY 3
#define WSA884X_IRQ_RETRY 2
#define PBR_MAX_VOLTAGE 20
#define PBR_MAX_CODE 255
#define WSA884X_IDLE_DETECT_NG_BLOCK_MASK	0x38
#define MAX_NAME_LEN	40
#define WSA884X_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
			SNDRV_PCM_RATE_384000)
/* Fractional Rates */
#define WSA884X_FRAC_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800)

#define WSA884X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
		SNDRV_PCM_FMTBIT_S24_LE |\
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

#define REG_FIELD_VALUE(register_name, field_name, value) \
WSA884X_##register_name, FIELD_MASK(register_name, field_name), \
value << FIELD_SHIFT(register_name, field_name)

enum {
	IDLE_DETECT,
	NG1,
	NG2,
	NG3,
};

struct wsa_temp_register {
	u8 d1_msb;
	u8 d1_lsb;
	u8 d2_msb;
	u8 d2_lsb;
	u8 dmeas_msb;
	u8 dmeas_lsb;
};

enum {
	COMP_OFFSET0,
	COMP_OFFSET1,
	COMP_OFFSET2,
	COMP_OFFSET3,
	COMP_OFFSET4,
};

#define WSA884X_VTH_TO_REG(vth) \
	((vth) != 0 ? (((vth) - 150) * PBR_MAX_CODE / (PBR_MAX_VOLTAGE * 100) + 1) : 0)

struct wsa_reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

static const struct wsa_reg_mask_val reg_init[] = {
	{REG_FIELD_VALUE(CKWD_CTL_1, VPP_SW_CTL, 0x00)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A2_0, COEF_A2, 0x0A)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A2_1, COEF_A2, 0x08)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A3_0, COEF_A3, 0xF3)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A3_1, COEF_A3, 0x07)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A4_0, COEF_A4, 0x79)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A5_0, COEF_A5, 0x0B)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A6_0, COEF_A6, 0x8A)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_A7_0, COEF_A7, 0x9B)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_C_0, COEF_C3, 0x06)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_C_0, COEF_C2, 0x08)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_C_2, COEF_C7, 0x0F)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_C_3, COEF_C7, 0x20)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_R1, SAT_LIMIT_R1, 0x83)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_R2, SAT_LIMIT_R2, 0x7F)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_R3, SAT_LIMIT_R3, 0x9D)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_R4, SAT_LIMIT_R4, 0x82)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_R5, SAT_LIMIT_R5, 0x8B)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_R6, SAT_LIMIT_R6, 0x9B)},
	{REG_FIELD_VALUE(CDC_SPK_DSM_R7, SAT_LIMIT_R7, 0x3F)},
	{REG_FIELD_VALUE(BOP_DEGLITCH_CTL, BOP_DEGLITCH_SETTING, 0x08)},
	{REG_FIELD_VALUE(VBAT_THRM_FLT_CTL, VBAT_COEF_SEL, 0x04)},
	{REG_FIELD_VALUE(CLSH_CTL_0, DLY_CODE, 0x06)},
	{REG_FIELD_VALUE(CLSH_SOFT_MAX, SOFT_MAX, 0xFF)},
	{REG_FIELD_VALUE(OTP_REG_38, BOOST_ILIM_TUNE, 0x00)},
	{REG_FIELD_VALUE(OTP_REG_40, ISENSE_RESCAL, 0x08)},
	{REG_FIELD_VALUE(STB_CTRL1, SLOPE_COMP_CURRENT, 0x0D)},
	{REG_FIELD_VALUE(ILIM_CTRL1, ILIM_OFFSET_PB, 0x03)},
	{REG_FIELD_VALUE(CURRENT_LIMIT, CURRENT_LIMIT, 0x09)},
	{REG_FIELD_VALUE(CKWD_CTL_1, CKWD_VCOMP_VREF_SEL, 0x13)},
	{REG_FIELD_VALUE(BOP2_PROG, BOP2_VTH, 0x06)},
	{REG_FIELD_VALUE(BOP2_PROG, BOP2_HYST, 0x06)},
	{REG_FIELD_VALUE(VBAT_CAL_CTL, RESERVE, 0x02)},
	{REG_FIELD_VALUE(REF_CTRL, BG_RDY_SEL, 0x01)},
	{REG_FIELD_VALUE(ZX_CTRL1, ZX_DET_SW_SEL, 0x03)},
};

static int wsa884x_handle_post_irq(void *data);
static int wsa884x_get_temperature(struct snd_soc_component *component,
				   int *temp);
enum {
	WSA8840 = 0,
	WSA8845 = 5,
	WSA8845H = 0xC,
};

enum {
	SPKR_STATUS = 0,
	WSA_SUPPLIES_LPM_MODE,
	SPKR_ADIE_LB,
};

enum {
	WSA884X_IRQ_INT_SAF2WAR = 0,
	WSA884X_IRQ_INT_WAR2SAF,
	WSA884X_IRQ_INT_DISABLE,
	WSA884X_IRQ_INT_OCP,
	WSA884X_IRQ_INT_CLIP,
	WSA884X_IRQ_INT_PDM_WD,
	WSA884X_IRQ_INT_CLK_WD,
	WSA884X_IRQ_INT_INTR_PIN,
	WSA884X_IRQ_INT_UVLO,
	WSA884X_IRQ_INT_PA_ON_ERR,
	WSA884X_NUM_IRQS,
};

static const struct regmap_irq wsa884x_irqs[WSA884X_NUM_IRQS] = {
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_SAF2WAR, 0, 0x01),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_WAR2SAF, 0, 0x02),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_DISABLE, 0, 0x04),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_OCP, 0, 0x08),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_CLIP, 0, 0x10),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_PDM_WD, 0, 0x20),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_CLK_WD, 0, 0x40),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_INTR_PIN, 0, 0x80),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_UVLO, 1, 0x01),
	REGMAP_IRQ_REG(WSA884X_IRQ_INT_PA_ON_ERR, 1, 0x02),
};

static struct regmap_irq_chip wsa884x_regmap_irq_chip = {
	.name = "wsa884x",
	.irqs = wsa884x_irqs,
	.num_irqs = ARRAY_SIZE(wsa884x_irqs),
	.num_regs = 2,
	.status_base = WSA884X_INTR_STATUS0,
	.mask_base = WSA884X_INTR_MASK0,
	.type_base = WSA884X_INTR_LEVEL0,
	.ack_base = WSA884X_INTR_CLEAR0,
	.use_ack = 1,
	.runtime_pm = false,
	.handle_post_irq = wsa884x_handle_post_irq,
	.irq_drv_data = NULL,
};

static int wsa884x_handle_post_irq(void *data)
{
	struct wsa884x_priv *wsa884x = data;
	u32 sts1 = 0, sts2 = 0;
	int retry = WSA884X_IRQ_RETRY;

	struct snd_soc_component *component = NULL;

	if (!wsa884x)
		return IRQ_NONE;

	component = wsa884x->component;
	if (!wsa884x->pa_mute) {
		do {
			wsa884x->pa_mute = 0;
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(PA_FSM_EN, GLOBAL_PA_EN, 0x01));
			usleep_range(1000, 1100);

			regmap_read(wsa884x->regmap, WSA884X_INTR_STATUS0, &sts1);
			regmap_read(wsa884x->regmap, WSA884X_INTR_STATUS1, &sts2);

			wsa884x->swr_slave->slave_irq_pending =
					((sts1 || sts2) ? true : false);
			pr_debug("%s: IRQs Sts0: %x, Sts1: %x\n", __func__,
				 sts1, sts2);
			if (wsa884x->swr_slave->slave_irq_pending) {
				pr_debug("%s: IRQ retries left: %0d\n",
					__func__, retry);
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PA_FSM_EN, GLOBAL_PA_EN, 0x00));
				wsa884x->pa_mute = 1;
				if (retry--)
					usleep_range(1000, 1100);
			} else {
				break;
			}
		} while (retry);
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_DEBUG_FS
static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, u32 *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");
	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (kstrtou32(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static bool is_swr_slave_reg_readable(int reg)
{
	int ret = true;

	if (((reg > 0x46) && (reg < 0x4A)) ||
	    ((reg > 0x4A) && (reg < 0x50)) ||
	    ((reg > 0x55) && (reg < 0x60)) ||
	    ((reg > 0x60) && (reg < 0x70)) ||
	    ((reg > 0x70) && (reg < 0xC0)) ||
	    ((reg > 0xC1) && (reg < 0xC8)) ||
	    ((reg > 0xC8) && (reg < 0xD0)) ||
	    ((reg > 0xD0) && (reg < 0xE0)) ||
	    ((reg > 0xE0) && (reg < 0xF0)) ||
	    ((reg > 0xF0) && (reg < 0x100)) ||
	    ((reg > 0x105) && (reg < 0x120)) ||
	    ((reg > 0x205) && (reg < 0x220)) ||
	    ((reg > 0x305) && (reg < 0x320)) ||
	    ((reg > 0x405) && (reg < 0x420)) ||
	    ((reg > 0x505) && (reg < 0x520)) ||
	    ((reg > 0x605) && (reg < 0x620)) ||
	    ((reg > 0x127) && (reg < 0x130)) ||
	    ((reg > 0x227) && (reg < 0x230)) ||
	    ((reg > 0x327) && (reg < 0x330)) ||
	    ((reg > 0x427) && (reg < 0x430)) ||
	    ((reg > 0x527) && (reg < 0x530)) ||
	    ((reg > 0x627) && (reg < 0x630)) ||
	    ((reg > 0x137) && (reg < 0x200)) ||
	    ((reg > 0x237) && (reg < 0x300)) ||
	    ((reg > 0x337) && (reg < 0x400)) ||
	    ((reg > 0x437) && (reg < 0x500)) ||
	    ((reg > 0x537) && (reg < 0x600)) ||
	    ((reg > 0x637) && (reg < 0xF00)) ||
	    ((reg > 0xF05) && (reg < 0xF20)) ||
	    ((reg > 0xF25) && (reg < 0xF30)) ||
	    ((reg > 0xF35) && (reg < 0x2000)))
		ret = false;

	return ret;
}

static ssize_t swr_slave_reg_show(struct swr_device *pdev, char __user *ubuf,
					size_t count, loff_t *ppos)
{
	int i, reg_val, len;
	ssize_t total = 0;
	char tmp_buf[SWR_SLV_MAX_BUF_LEN];

	if (!ubuf || !ppos)
		return 0;

	for (i = (((int) *ppos/BYTES_PER_LINE) + SWR_SLV_START_REG_ADDR);
		i <= SWR_SLV_MAX_REG_ADDR; i++) {
		if (!is_swr_slave_reg_readable(i))
			continue;
		swr_read(pdev, pdev->dev_num, i, &reg_val, 1);
		len = snprintf(tmp_buf, sizeof(tmp_buf), "0x%.3x: 0x%.2x\n", i,
			       (reg_val & 0xFF));
		if (len < 0) {
			pr_err_ratelimited("%s: fail to fill the buffer\n", __func__);
			total = -EFAULT;
			goto copy_err;
		}
		if ((total + len) >= count - 1)
			break;
		if (copy_to_user((ubuf + total), tmp_buf, len)) {
			pr_err_ratelimited("%s: fail to copy reg dump\n", __func__);
			total = -EFAULT;
			goto copy_err;
		}
		total += len;
		*ppos += len;
	}

copy_err:
	*ppos = SWR_SLV_MAX_REG_ADDR * BYTES_PER_LINE;
	return total;
}

static ssize_t codec_debug_dump(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct swr_device *pdev;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	return swr_slave_reg_show(pdev, ubuf, count, ppos);
}

static ssize_t codec_debug_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char lbuf[SWR_SLV_RD_BUF_LEN];
	struct swr_device *pdev = NULL;
	struct wsa884x_priv *wsa884x = NULL;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	wsa884x = swr_get_dev_data(pdev);
	if (!wsa884x)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	snprintf(lbuf, sizeof(lbuf), "0x%x\n",
			(wsa884x->read_data & 0xFF));

	return simple_read_from_buffer(ubuf, count, ppos, lbuf,
					       strnlen(lbuf, 7));
}

static ssize_t codec_debug_peek_write(struct file *file,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[SWR_SLV_WR_BUF_LEN];
	int rc = 0;
	u32 param[5];
	struct swr_device *pdev = NULL;
	struct wsa884x_priv *wsa884x = NULL;

	if (!cnt || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	wsa884x = swr_get_dev_data(pdev);
	if (!wsa884x)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	rc = get_parameters(lbuf, param, 1);
	if (!((param[0] <= SWR_SLV_MAX_REG_ADDR) && (rc == 0)))
		return -EINVAL;
	swr_read(pdev, pdev->dev_num, param[0], &wsa884x->read_data, 1);
	if (rc == 0)
		rc = cnt;
	else
		pr_err_ratelimited("%s: rc = %d\n", __func__, rc);

	return rc;
}

static ssize_t codec_debug_write(struct file *file,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[SWR_SLV_WR_BUF_LEN];
	int rc = 0;
	u32 param[5];
	struct swr_device *pdev;

	if (!file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	rc = get_parameters(lbuf, param, 2);
	if (!((param[0] <= SWR_SLV_MAX_REG_ADDR) &&
		(param[1] <= 0xFF) && (rc == 0)))
		return -EINVAL;
	swr_write(pdev, pdev->dev_num, param[0], &param[1]);
	if (rc == 0)
		rc = cnt;
	else
		pr_err_ratelimited("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations codec_debug_write_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
};

static const struct file_operations codec_debug_read_ops = {
	.open = codec_debug_open,
	.read = codec_debug_read,
	.write = codec_debug_peek_write,
};

static const struct file_operations codec_debug_dump_ops = {
	.open = codec_debug_open,
	.read = codec_debug_dump,
};
#endif

static void wsa884x_regcache_sync(struct wsa884x_priv *wsa884x)
{
	mutex_lock(&wsa884x->res_lock);
	regcache_mark_dirty(wsa884x->regmap);
	regcache_sync(wsa884x->regmap);
	mutex_unlock(&wsa884x->res_lock);
}

static irqreturn_t wsa884x_saf2war_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_war2saf_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_otp_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_ocp_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_clip_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_pdm_wd_handle_irq(int irq, void *data)
{
	struct wsa884x_priv *wsa884x = data;
	struct snd_soc_component *component = NULL;

	if (!wsa884x)
		return IRQ_NONE;
	component = wsa884x->component;
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PDM_WD_CTL, PDM_WD_EN, 0x00));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PDM_WD_CTL, PDM_WD_EN, 0x01));

	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_clk_wd_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_ext_int_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_uvlo_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t wsa884x_pa_on_err_handle_irq(int irq, void *data)
{
	u8 pa_fsm_sta = 0, pa_fsm_err = 0;
	struct wsa884x_priv *wsa884x = data;
	struct snd_soc_component *component = NULL;

	if (!wsa884x)
		return IRQ_NONE;

	component = wsa884x->component;
	if (!component)
		return IRQ_NONE;

	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PA_FSM_EN, GLOBAL_PA_EN, 0x00));
	pa_fsm_sta = (snd_soc_component_read(component, WSA884X_PA_FSM_STA1)
			& 0x1F);
	if (pa_fsm_sta)
		pa_fsm_err = snd_soc_component_read(component,
				WSA884X_PA_FSM_ERR_COND0);

	pr_err_ratelimited("%s: interrupt for irq =%d triggered\n",
			   __func__, irq);

	snd_soc_component_update_bits(component, WSA884X_PA_FSM_CTL0,
				0x10, 0x00);
	snd_soc_component_update_bits(component, WSA884X_PA_FSM_CTL0,
				0x10, 0x10);
	snd_soc_component_update_bits(component, WSA884X_PA_FSM_CTL0,
				0x10, 0x00);

	return IRQ_HANDLED;
}

static int wsa884x_set_gain_parameters(struct snd_soc_component *component)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	u8 igain;
	u8 vgain;

	switch (wsa884x->bat_cfg) {
	case CONFIG_1S:
	case EXT_1S:
		switch (wsa884x->system_gain) {
		case G_21_DB:
			wsa884x->comp_offset = COMP_OFFSET0;
			wsa884x->min_gain = G_0_DB;
			wsa884x->pa_aux_gain = PA_AUX_0_DB;
			break;
		case G_19P5_DB:
			wsa884x->comp_offset = COMP_OFFSET1;
			wsa884x->min_gain = G_M1P5_DB;
			wsa884x->pa_aux_gain =  PA_AUX_M1P5_DB;
			break;
		case G_18_DB:
			wsa884x->comp_offset = COMP_OFFSET2;
			wsa884x->min_gain = G_M3_DB;
			wsa884x->pa_aux_gain =  PA_AUX_M3_DB;
			break;
		case G_16P5_DB:
			wsa884x->comp_offset = COMP_OFFSET3;
			wsa884x->min_gain = G_M4P5_DB;
			wsa884x->pa_aux_gain =  PA_AUX_M4P5_DB;
			break;
		default:
			wsa884x->comp_offset = COMP_OFFSET4;
			wsa884x->min_gain = G_M6_DB;
			wsa884x->pa_aux_gain =  PA_AUX_M6_DB;
			break;
		}
		break;
	case CONFIG_3S:
	case EXT_3S:
		wsa884x->comp_offset = COMP_OFFSET0;
		wsa884x->min_gain = G_7P5_DB;
		wsa884x->pa_aux_gain =  PA_AUX_7P5_DB;
		break;
	case EXT_ABOVE_3S:
		wsa884x->comp_offset = COMP_OFFSET0;
		wsa884x->min_gain = G_12_DB;
		wsa884x->pa_aux_gain =  PA_AUX_12_DB;
		break;
	default:
		wsa884x->comp_offset = COMP_OFFSET0;
		wsa884x->min_gain = G_0_DB;
		wsa884x->pa_aux_gain = PA_AUX_0_DB;
		break;
	}

	igain = isense_gain_data[wsa884x->system_gain][wsa884x->rload];
	vgain = vsense_gain_data[wsa884x->system_gain];
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(ISENSE2, ISENSE_GAIN_CTL, igain));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(VSENSE1, GAIN_VSENSE_FE, vgain));

	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(GAIN_RAMPING_MIN, MIN_GAIN, wsa884x->min_gain));

	if (wsa884x->comp_enable) {
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(DRE_CTL_0, OFFSET,
					wsa884x->comp_offset));
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(DRE_CTL_1, CSR_GAIN_EN, 0x00));
	} else {
		wsa884x->pa_aux_gain = pa_aux_no_comp[wsa884x->pa_gain];
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(DRE_CTL_1, CSR_GAIN_EN, 0x01));
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(DRE_CTL_1, CSR_GAIN, wsa884x->pa_gain));

	}
	return 0;
}

static int wsa884x_set_pbr_parameters(struct snd_soc_component *component)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int vth1_reg_val;
	int vth2_reg_val;
	int vth3_reg_val;
	int vth4_reg_val;
	int vth5_reg_val;
	int vth6_reg_val;
	int vth7_reg_val;
	int vth8_reg_val;
	int vth9_reg_val;
	int vth10_reg_val;
	int vth11_reg_val;
	int vth12_reg_val;
	int vth13_reg_val;
	int vth14_reg_val;
	int vth15_reg_val;


	int vth1_val = pbr_vth1_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth2_val = pbr_vth2_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth3_val = pbr_vth3_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth4_val = pbr_vth4_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth5_val = pbr_vth5_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth6_val = pbr_vth6_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth7_val = pbr_vth7_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth8_val = pbr_vth8_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth9_val = pbr_vth9_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth10_val = pbr_vth10_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth11_val = pbr_vth11_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth12_val = pbr_vth12_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth13_val = pbr_vth13_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth14_val = pbr_vth14_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];
	int vth15_val = pbr_vth15_data[wsa884x->system_gain][wsa884x->bat_cfg][wsa884x->rload];

	vth1_reg_val = WSA884X_VTH_TO_REG(vth1_val);
	vth2_reg_val = WSA884X_VTH_TO_REG(vth2_val);
	vth3_reg_val = WSA884X_VTH_TO_REG(vth3_val);
	vth4_reg_val = WSA884X_VTH_TO_REG(vth4_val);
	vth5_reg_val = WSA884X_VTH_TO_REG(vth5_val);
	vth6_reg_val = WSA884X_VTH_TO_REG(vth6_val);
	vth7_reg_val = WSA884X_VTH_TO_REG(vth7_val);
	vth8_reg_val = WSA884X_VTH_TO_REG(vth8_val);
	vth9_reg_val = WSA884X_VTH_TO_REG(vth9_val);
	vth10_reg_val = WSA884X_VTH_TO_REG(vth10_val);
	vth11_reg_val = WSA884X_VTH_TO_REG(vth11_val);
	vth12_reg_val = WSA884X_VTH_TO_REG(vth12_val);
	vth13_reg_val = WSA884X_VTH_TO_REG(vth13_val);
	vth14_reg_val = WSA884X_VTH_TO_REG(vth14_val);
	vth15_reg_val = WSA884X_VTH_TO_REG(vth15_val);

	snd_soc_component_write(component, WSA884X_CLSH_VTH1, vth1_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH2, vth2_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH3, vth3_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH4, vth4_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH5, vth5_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH6, vth6_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH7, vth7_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH8, vth8_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH9, vth9_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH10, vth10_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH11, vth11_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH12, vth12_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH13, vth13_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH14, vth14_reg_val);
	snd_soc_component_write(component, WSA884X_CLSH_VTH15, vth15_reg_val);

	return 0;
}

static void wsa_noise_gate_write(struct snd_soc_component *component,
			int imode)
{
	switch (imode) {
	case NG1:
		snd_soc_component_update_bits(component, WSA884X_PA_FSM_CTL1,
			WSA884X_IDLE_DETECT_NG_BLOCK_MASK, 0x30);
		break;
	case NG2:
		snd_soc_component_update_bits(component, WSA884X_PA_FSM_CTL1,
			WSA884X_IDLE_DETECT_NG_BLOCK_MASK, 0x20);
		break;
	case NG3:
		snd_soc_component_update_bits(component, WSA884X_PA_FSM_CTL1,
			WSA884X_IDLE_DETECT_NG_BLOCK_MASK, 0x10);
		break;
	default:
		snd_soc_component_update_bits(component, WSA884X_PA_FSM_CTL1,
			WSA884X_IDLE_DETECT_NG_BLOCK_MASK, 0x8);
		break;
	}
}

static int wsa_dev_mode_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa884x->dev_mode;

	dev_dbg(component->dev, "%s: mode = 0x%x\n", __func__,
			wsa884x->dev_mode);

	return 0;
}

static int wsa_dev_mode_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int dev_mode;
	int wsa_dev_index;

	if ((ucontrol->value.integer.value[0] >= SPEAKER) &&
			(ucontrol->value.integer.value[0] < MAX_DEV_MODE))
		dev_mode = ucontrol->value.integer.value[0];
	else
		return -EINVAL;

	dev_dbg(component->dev, "%s: Dev Mode current: %d, new: %d\n",
		__func__, wsa884x->dev_mode, dev_mode);

	/* Check if input parameter is in range */
	wsa_dev_index = (wsa884x->dev_index - 1) % 2;
	if ((dev_mode + wsa_dev_index * 2) < (MAX_DEV_MODE * 2)) {
		wsa884x->dev_mode =  dev_mode;
		wsa884x->system_gain = wsa884x->sys_gains[dev_mode + wsa_dev_index * 2];
	} else {
		return -EINVAL;
	}

	return 0;
}

static const char * const wsa_pa_gain_text[] = {
	"G_21_DB", "G_19P5_DB" "G_18_DB", "G_16P5_DB", "G_15_DB", "G_13P5_DB",
	"G_12_DB", "G_10P5_DB", "G_9_DB", "G_7P5_DB", "G_6_DB", "G_4P5_DB",
	"G_3_DB", "G_1P5_DB", "G_0_DB", "G_M1P5_DB", "G_M3_DB", "G_M4P5_DB"
	"G_M6_DB", "G_M7P5_DB", "G_M9_DB"
};

static const struct soc_enum wsa_pa_gain_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wsa_pa_gain_text), wsa_pa_gain_text);

static int wsa_pa_gain_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa884x->pa_gain;

	dev_dbg(component->dev, "%s: PA gain = 0x%x\n", __func__,
			wsa884x->pa_gain);

	return 0;
}

static int wsa_pa_gain_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	wsa884x->pa_gain =  ucontrol->value.integer.value[0];

	return 0;
}

static int wsa_get_temp(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int temp = 0;

	if (test_bit(SPKR_STATUS, &wsa884x->status_mask))
		temp = wsa884x->curr_temp;
	else
		wsa884x_get_temperature(component, &temp);

	ucontrol->value.integer.value[0] = temp;

	return 0;
}

static ssize_t wsa884x_codec_version_read(struct snd_info_entry *entry,
			       void *file_private_data, struct file *file,
			       char __user *buf, size_t count, loff_t pos)
{
	struct wsa884x_priv *wsa884x;
	char buffer[WSA884X_VERSION_ENTRY_SIZE];
	int len = 0;

	wsa884x = (struct wsa884x_priv *) entry->private_data;
	if (!wsa884x) {
		pr_err_ratelimited("%s: wsa884x priv is null\n", __func__);
		return -EINVAL;
	}

	switch (wsa884x->version) {
	case WSA884X_VERSION_1_0:
		len = snprintf(buffer, sizeof(buffer), "WSA884X_1_0\n");
		break;
	default:
		len = snprintf(buffer, sizeof(buffer), "VER_UNDEFINED\n");
		break;
	}

	return simple_read_from_buffer(buf, count, &pos, buffer, len);
}

static struct snd_info_entry_ops wsa884x_codec_info_ops = {
	.read = wsa884x_codec_version_read,
};

static ssize_t wsa884x_variant_read(struct snd_info_entry *entry,
				    void *file_private_data,
				    struct file *file,
				    char __user *buf, size_t count,
				    loff_t pos)
{
	struct wsa884x_priv *wsa884x;
	char buffer[WSA884X_VARIANT_ENTRY_SIZE];
	int len = 0;

	wsa884x = (struct wsa884x_priv *) entry->private_data;
	if (!wsa884x) {
		pr_err_ratelimited("%s: wsa884x priv is null\n", __func__);
		return -EINVAL;
	}

	switch (wsa884x->variant) {
	case WSA8840:
		len = snprintf(buffer, sizeof(buffer), "WSA8840\n");
		break;
	case WSA8845:
		len = snprintf(buffer, sizeof(buffer), "WSA8845\n");
		break;
	case WSA8845H:
		len = snprintf(buffer, sizeof(buffer), "WSA8845H\n");
		break;
	default:
		len = snprintf(buffer, sizeof(buffer), "UNDEFINED\n");
		break;
	}

	return simple_read_from_buffer(buf, count, &pos, buffer, len);
}

static struct snd_info_entry_ops wsa884x_variant_ops = {
	.read = wsa884x_variant_read,
};

/*
 * wsa884x_codec_info_create_codec_entry - creates wsa884x module
 * @codec_root: The parent directory
 * @component: Codec instance
 *
 * Creates wsa884x module and version entry under the given
 * parent directory.
 *
 * Return: 0 on success or negative error code on failure.
 */
int wsa884x_codec_info_create_codec_entry(struct snd_info_entry *codec_root,
					  struct snd_soc_component *component)
{
	struct snd_info_entry *version_entry;
	struct snd_info_entry *variant_entry;
	struct wsa884x_priv *wsa884x;
	struct snd_soc_card *card;
	char name[80];

	if (!codec_root || !component)
		return -EINVAL;

	wsa884x = snd_soc_component_get_drvdata(component);
	if (wsa884x->entry) {
		dev_dbg(wsa884x->dev,
			"%s:wsa884x module already created\n", __func__);
		return 0;
	}
	card = component->card;

	snprintf(name, sizeof(name), "%s.%llx", "wsa884x",
		 wsa884x->swr_slave->addr);

	wsa884x->entry = snd_info_create_module_entry(codec_root->module,
						(const char *)name,
						codec_root);
	if (!wsa884x->entry) {
		dev_dbg(component->dev, "%s: failed to create wsa884x entry\n",
			__func__);
		return -ENOMEM;
	}
	wsa884x->entry->mode = S_IFDIR | 0555;
	if (snd_info_register(wsa884x->entry) < 0) {
		snd_info_free_entry(wsa884x->entry);
		return -ENOMEM;
	}

	version_entry = snd_info_create_card_entry(card->snd_card,
						   "version",
						   wsa884x->entry);
	if (!version_entry) {
		dev_dbg(component->dev, "%s: failed to create wsa884x version entry\n",
			__func__);
		snd_info_free_entry(wsa884x->entry);
		return -ENOMEM;
	}

	version_entry->private_data = wsa884x;
	version_entry->size = WSA884X_VERSION_ENTRY_SIZE;
	version_entry->content = SNDRV_INFO_CONTENT_DATA;
	version_entry->c.ops = &wsa884x_codec_info_ops;

	if (snd_info_register(version_entry) < 0) {
		snd_info_free_entry(version_entry);
		snd_info_free_entry(wsa884x->entry);
		return -ENOMEM;
	}
	wsa884x->version_entry = version_entry;

	variant_entry = snd_info_create_card_entry(card->snd_card,
						   "variant",
						   wsa884x->entry);
	if (!variant_entry) {
		dev_dbg(component->dev,
			"%s: failed to create wsa884x variant entry\n",
			__func__);
		snd_info_free_entry(version_entry);
		snd_info_free_entry(wsa884x->entry);
		return -ENOMEM;
	}

	variant_entry->private_data = wsa884x;
	variant_entry->size = WSA884X_VARIANT_ENTRY_SIZE;
	variant_entry->content = SNDRV_INFO_CONTENT_DATA;
	variant_entry->c.ops = &wsa884x_variant_ops;

	if (snd_info_register(variant_entry) < 0) {
		snd_info_free_entry(variant_entry);
		snd_info_free_entry(version_entry);
		snd_info_free_entry(wsa884x->entry);
		return -ENOMEM;
	}
	wsa884x->variant_entry = variant_entry;

	return 0;
}
EXPORT_SYMBOL(wsa884x_codec_info_create_codec_entry);

/*
 * wsa884x_codec_get_dev_num - returns swr device number
 * @component: Codec instance
 *
 * Return: swr device number on success or negative error
 * code on failure.
 */
int wsa884x_codec_get_dev_num(struct snd_soc_component *component)
{
	struct wsa884x_priv *wsa884x;

	if (!component)
		return -EINVAL;

	wsa884x = snd_soc_component_get_drvdata(component);
	if (!wsa884x) {
		pr_err_ratelimited("%s: wsa884x component is NULL\n", __func__);
		return -EINVAL;
	}

	return wsa884x->swr_slave->dev_num;
}
EXPORT_SYMBOL(wsa884x_codec_get_dev_num);

static int wsa884x_get_dev_num(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x;

	if (!component)
		return -EINVAL;

	wsa884x = snd_soc_component_get_drvdata(component);
	if (!wsa884x) {
		pr_err_ratelimited("%s: wsa884x component is NULL\n", __func__);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = wsa884x->swr_slave->dev_num;
	return 0;
}

static int wsa884x_get_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa884x->comp_enable;
	return 0;
}

/*
 * wsa884x_validate_dt_configuration_params - returns 1 or 0
 * Return: 0 Valid configuration, 1 Invalid configuration
 */
static bool wsa884x_validate_dt_configuration_params(struct snd_soc_component *component,
					u8 irload, u8 ibat_cfg_dts, u8 isystem_gain)
{
	u8 bat_cfg_reg;
	bool is_invalid_flag = true;

	bat_cfg_reg = snd_soc_component_read(component, WSA884X_VPHX_SYS_EN_STATUS);
	if ((ibat_cfg_dts == EXT_1S) || (ibat_cfg_dts == EXT_2S) || (ibat_cfg_dts == EXT_3S))
		ibat_cfg_dts = EXT_ABOVE_3S;
	if ((WSA_4_OHMS <= irload && irload < WSA_MAX_OHMS) &&
		(G_21_DB <= isystem_gain && isystem_gain < G_MAX_DB) &&
		(EXT_ABOVE_3S <= ibat_cfg_dts && ibat_cfg_dts < CONFIG_MAX) &&
		(ibat_cfg_dts == bat_cfg_reg))
			is_invalid_flag = false;

	return is_invalid_flag;
}

static int wsa884x_set_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: Compander enable current %d, new %d\n",
		 __func__, wsa884x->comp_enable, value);
	wsa884x->comp_enable = value;
	return 0;
}

static int wsa884x_get_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa884x->visense_enable;
	return 0;
}

static int wsa884x_set_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: VIsense enable current %d, new %d\n",
		 __func__, wsa884x->visense_enable, value);
	wsa884x->visense_enable = value;
	return 0;
}

static int wsa884x_get_pbr(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa884x->pbr_enable;
	return 0;
}

static int wsa884x_set_pbr(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: PBR enable current %d, new %d\n",
		 __func__, wsa884x->pbr_enable, value);
	wsa884x->pbr_enable = value;
	return 0;
}

static int wsa884x_get_cps(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa884x->cps_enable;
	return 0;
}

static int wsa884x_set_cps(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: CPS enable current %d, new %d\n",
		 __func__, wsa884x->cps_enable, value);
	wsa884x->cps_enable = value;
	return 0;
}

static const struct snd_kcontrol_new wsa884x_snd_controls[] = {
	SOC_ENUM_EXT("WSA PA Gain", wsa_pa_gain_enum,
			wsa_pa_gain_get, wsa_pa_gain_put),

	SOC_SINGLE_EXT("WSA Temp", SND_SOC_NOPM, 0, UINT_MAX, 0,
			wsa_get_temp, NULL),

	SOC_SINGLE_EXT("WSA Get DevNum", SND_SOC_NOPM, 0, UINT_MAX, 0,
			wsa884x_get_dev_num, NULL),

	SOC_SINGLE_EXT("WSA MODE", SND_SOC_NOPM, 0, 1, 0,
			wsa_dev_mode_get, wsa_dev_mode_put),

	SOC_SINGLE_EXT("COMP Switch", SND_SOC_NOPM, 0, 1, 0,
			wsa884x_get_compander, wsa884x_set_compander),

	SOC_SINGLE_EXT("VISENSE Switch", SND_SOC_NOPM, 0, 1, 0,
			wsa884x_get_visense, wsa884x_set_visense),

	SOC_SINGLE_EXT("PBR Switch", SND_SOC_NOPM, 0, 1, 0,
		wsa884x_get_pbr, wsa884x_set_pbr),

	SOC_SINGLE_EXT("CPS Switch", SND_SOC_NOPM, 0, 1, 0,
		wsa884x_get_cps, wsa884x_set_cps),

};

static const struct snd_kcontrol_new swr_dac_port[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static int wsa884x_set_port(struct snd_soc_component *component, int port_idx,
			u8 *port_id, u8 *num_ch, u8 *ch_mask, u32 *ch_rate,
			u8 *port_type)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	*port_id = wsa884x->port[port_idx].port_id;
	*num_ch = wsa884x->port[port_idx].num_ch;
	*ch_mask = wsa884x->port[port_idx].ch_mask;
	*ch_rate = wsa884x->port[port_idx].ch_rate;
	*port_type = wsa884x->port[port_idx].port_type;
	return 0;
}

static int wsa884x_enable_swr_dac_port(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	u8 port_id[WSA884X_MAX_SWR_PORTS];
	u8 num_ch[WSA884X_MAX_SWR_PORTS];
	u8 ch_mask[WSA884X_MAX_SWR_PORTS];
	u32 ch_rate[WSA884X_MAX_SWR_PORTS];
	u8 port_type[WSA884X_MAX_SWR_PORTS];
	u8 num_port = 0;

	dev_dbg(component->dev, "%s: event %d name %s\n", __func__,
		event, w->name);
	if (wsa884x == NULL)
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wsa884x_set_port(component, SWR_DAC_PORT,
				&port_id[num_port], &num_ch[num_port],
				&ch_mask[num_port], &ch_rate[num_port],
				&port_type[num_port]);
		if (wsa884x->dev_mode == RECEIVER)
			ch_rate[num_port] = SWR_CLK_RATE_4P8MHZ;
		++num_port;

		if (wsa884x->comp_enable) {
			wsa884x_set_port(component, SWR_COMP_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa884x->pbr_enable) {
			wsa884x_set_port(component, SWR_PBR_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa884x->visense_enable) {
			wsa884x_set_port(component, SWR_VISENSE_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa884x->cps_enable) {
			wsa884x_set_port(component, SWR_CPS_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		swr_connect_port(wsa884x->swr_slave, &port_id[0], num_port,
				&ch_mask[0], &ch_rate[0], &num_ch[0],
					&port_type[0]);
		break;
	case SND_SOC_DAPM_POST_PMU:
		set_bit(SPKR_STATUS, &wsa884x->status_mask);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		wsa884x_set_port(component, SWR_DAC_PORT,
				&port_id[num_port], &num_ch[num_port],
				&ch_mask[num_port], &ch_rate[num_port],
				&port_type[num_port]);
		++num_port;

		if (wsa884x->comp_enable) {
			wsa884x_set_port(component, SWR_COMP_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa884x->pbr_enable) {
			wsa884x_set_port(component, SWR_PBR_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa884x->visense_enable) {
			wsa884x_set_port(component, SWR_VISENSE_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa884x->cps_enable) {
			wsa884x_set_port(component, SWR_CPS_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		swr_disconnect_port(wsa884x->swr_slave, &port_id[0], num_port,
				&ch_mask[0], &port_type[0]);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (swr_set_device_group(wsa884x->swr_slave, SWR_GROUP_NONE))
			dev_err_ratelimited(component->dev,
				"%s: set num ch failed\n", __func__);

		swr_slvdev_datapath_control(wsa884x->swr_slave,
					    wsa884x->swr_slave->dev_num,
					    false);
		break;
	default:
		break;
	}
	return 0;
}

static int wsa884x_spkr_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: %s %d\n", __func__, w->name, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		swr_slvdev_datapath_control(wsa884x->swr_slave,
					    wsa884x->swr_slave->dev_num,
					    true);
		wsa884x_set_gain_parameters(component);
		if (wsa884x->dev_mode == SPEAKER) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(DRE_CTL_0, PROG_DELAY, 0x0F));
		} else {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(DRE_CTL_0, PROG_DELAY, 0x03));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_PATH_MODE, RXD_MODE, 0x01));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(PWM_CLK_CTL,
				PWM_CLK_FREQ_SEL, 0x01));
		}
		if (wsa884x->pbr_enable) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CURRENT_LIMIT,
				CURRENT_LIMIT_OVRD_EN, 0x00));
			switch (wsa884x->bat_cfg) {
			case CONFIG_1S:
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CURRENT_LIMIT,
					CURRENT_LIMIT, 0x15));
				break;
			case CONFIG_2S:
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CURRENT_LIMIT,
					CURRENT_LIMIT, 0x11));
				break;
			case CONFIG_3S:
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CURRENT_LIMIT,
					CURRENT_LIMIT, 0x0D));
				break;
			}
		} else {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CURRENT_LIMIT,
				CURRENT_LIMIT_OVRD_EN, 0x01));
			if (wsa884x->system_gain >= G_12_DB)
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CURRENT_LIMIT,
					CURRENT_LIMIT, 0x15));
			else
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CURRENT_LIMIT,
					CURRENT_LIMIT, 0x09));
		}
		/* Force remove group */
		swr_remove_from_group(wsa884x->swr_slave,
				      wsa884x->swr_slave->dev_num);
		if (test_bit(SPKR_ADIE_LB, &wsa884x->status_mask) &&
		    !wsa884x->pa_mute)
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(PA_FSM_EN, GLOBAL_PA_EN, 0x01));
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(PA_FSM_EN, GLOBAL_PA_EN, 0x00));
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(PDM_WD_CTL, PDM_WD_EN, 0x00));
		clear_bit(SPKR_STATUS, &wsa884x->status_mask);
		clear_bit(SPKR_ADIE_LB, &wsa884x->status_mask);
		wsa884x->pa_mute = 0;
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget wsa884x_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_MIXER_E("SWR DAC_Port", SND_SOC_NOPM, 0, 0, swr_dac_port,
		ARRAY_SIZE(swr_dac_port), wsa884x_enable_swr_dac_port,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SPK("SPKR", wsa884x_spkr_event),
};

static const struct snd_soc_dapm_route wsa884x_audio_map[] = {
	{"SWR DAC_Port", "Switch", "IN"},
	{"SPKR", NULL, "SWR DAC_Port"},
};

int wsa884x_set_channel_map(struct snd_soc_component *component, u8 *port,
			    u8 num_port, unsigned int *ch_mask,
			    unsigned int *ch_rate, u8 *port_type)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int i;

	if (!port || !ch_mask || !ch_rate ||
		(num_port > WSA884X_MAX_SWR_PORTS)) {
		dev_err_ratelimited(component->dev,
			"%s: Invalid port=%pK, ch_mask=%pK, ch_rate=%pK\n",
			__func__, port, ch_mask, ch_rate);
		return -EINVAL;
	}
	for (i = 0; i < num_port; i++) {
		wsa884x->port[i].port_id = port[i];
		wsa884x->port[i].ch_mask = ch_mask[i];
		wsa884x->port[i].ch_rate = ch_rate[i];
		wsa884x->port[i].num_ch = __sw_hweight8(ch_mask[i]);
		if (port_type)
			wsa884x->port[i].port_type = port_type[i];
	}

	return 0;
}
EXPORT_SYMBOL(wsa884x_set_channel_map);

static void wsa884x_codec_init(struct snd_soc_component *component)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	int i;

	if (!wsa884x)
		return;

	for (i = 0; i < ARRAY_SIZE(reg_init); i++)
		snd_soc_component_update_bits(component, reg_init[i].reg,
					reg_init[i].mask, reg_init[i].val);

	wsa_noise_gate_write(component, wsa884x->noise_gate_mode);

}

static int32_t wsa884x_temp_reg_read(struct snd_soc_component *component,
				     struct wsa_temp_register *wsa_temp_reg)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	if (!wsa884x) {
		dev_err_ratelimited(component->dev, "%s: wsa884x is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&wsa884x->res_lock);

	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PA_FSM_BYP0, DC_CAL_EN, 0x01));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PA_FSM_BYP0, BG_EN, 0x01));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PA_FSM_BYP0, CLK_WD_EN, 0x01));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PA_FSM_BYP0, TSADC_EN, 0x01));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PA_FSM_BYP0, D_UNMUTE, 0x01));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(PA_FSM_BYP0, SPKR_PROT_EN, 0x01));

	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(TADC_VALUE_CTL, TEMP_VALUE_RD_EN, 0x00));
	wsa_temp_reg->dmeas_msb = snd_soc_component_read(component,
							WSA884X_TEMP_DIN_MSB);
	wsa_temp_reg->dmeas_lsb = snd_soc_component_read(component,
							WSA884X_TEMP_DIN_LSB);
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(TADC_VALUE_CTL, TEMP_VALUE_RD_EN, 0x01));
	wsa_temp_reg->d1_msb = snd_soc_component_read(component,
						     WSA884X_OTP_REG_1);
	wsa_temp_reg->d1_lsb = snd_soc_component_read(component,
						     WSA884X_OTP_REG_2);
	wsa_temp_reg->d2_msb = snd_soc_component_read(component,
						     WSA884X_OTP_REG_3);
	wsa_temp_reg->d2_lsb = snd_soc_component_read(component,
						     WSA884X_OTP_REG_4);

	snd_soc_component_update_bits(component,
				     WSA884X_PA_FSM_BYP0, 0xE7, 0x00);
	mutex_unlock(&wsa884x->res_lock);

	return 0;
}

static int wsa884x_get_temperature(struct snd_soc_component *component,
				   int *temp)
{
	struct wsa_temp_register reg;
	int dmeas, d1, d2;
	int ret = 0;
	int temp_val = 0;
	int t1 = T1_TEMP;
	int t2 = T2_TEMP;
	u8 retry = WSA884X_TEMP_RETRY;
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	if (!wsa884x)
		return -EINVAL;

	do {
		ret = wsa884x_temp_reg_read(component, &reg);
		if (ret) {
			pr_err_ratelimited("%s: temp read failed: %d, current temp: %d\n",
				__func__, ret, wsa884x->curr_temp);
			if (temp)
				*temp = wsa884x->curr_temp;
			return 0;
		}
		/*
		 * Temperature register values are expected to be in the
		 * following range.
		 * d1_msb  = 68 - 92 and d1_lsb  = 0, 64, 128, 192
		 * d2_msb  = 185 -218 and  d2_lsb  = 0, 64, 128, 192
		 */
		if ((reg.d1_msb < 68 || reg.d1_msb > 92) ||
		    (!(reg.d1_lsb == 0 || reg.d1_lsb == 64 || reg.d1_lsb == 128 ||
			reg.d1_lsb == 192)) ||
		    (reg.d2_msb < 185 || reg.d2_msb > 218) ||
		    (!(reg.d2_lsb == 0 || reg.d2_lsb == 64 || reg.d2_lsb == 128 ||
			reg.d2_lsb == 192))) {
			printk_ratelimited("%s: Temperature registers[%d %d %d %d] are out of range\n",
					   __func__, reg.d1_msb, reg.d1_lsb, reg.d2_msb,
					   reg.d2_lsb);
		}
		dmeas = ((reg.dmeas_msb << 0x8) | reg.dmeas_lsb) >> 0x6;
		d1 = ((reg.d1_msb << 0x8) | reg.d1_lsb) >> 0x6;
		d2 = ((reg.d2_msb << 0x8) | reg.d2_lsb) >> 0x6;

		if (d1 == d2)
			temp_val = TEMP_INVALID;
		else
			temp_val = t1 + (((dmeas - d1) * (t2 - t1))/(d2 - d1));

		if (temp_val <= LOW_TEMP_THRESHOLD ||
			temp_val >= HIGH_TEMP_THRESHOLD) {
			pr_debug("%s: T0: %d is out of range[%d, %d]\n", __func__,
				 temp_val, LOW_TEMP_THRESHOLD, HIGH_TEMP_THRESHOLD);
			if (retry--)
				msleep(10);
		} else {
			break;
		}
	} while (retry);

	wsa884x->curr_temp = temp_val;
	if (temp)
		*temp = temp_val;
	pr_debug("%s: t0 measured: %d dmeas = %d, d1 = %d, d2 = %d\n",
		  __func__, temp_val, dmeas, d1, d2);

	return ret;
}

static int wsa884x_codec_probe(struct snd_soc_component *component)
{
	char w_name[MAX_NAME_LEN];
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);
	struct swr_device *dev;
	int variant = 0, version = 0;
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);

	if (!wsa884x)
		return -EINVAL;

	if (!component->name_prefix)
		return -EINVAL;

	snd_soc_component_init_regmap(component, wsa884x->regmap);

	dev = wsa884x->swr_slave;
	wsa884x->component = component;

	variant = (snd_soc_component_read(component, WSA884X_OTP_REG_0)
					 & FIELD_MASK(OTP_REG_0, WSA884X_ID));
	wsa884x->variant = variant;

	version = (snd_soc_component_read(component, WSA884X_CHIP_ID0)
					& FIELD_MASK(CHIP_ID0, BYTE_0));

	wsa884x->version = version;

	wsa884x->comp_offset = COMP_OFFSET2;
	wsa884x_codec_init(component);
	wsa884x->global_pa_cnt = 0;

	memset(w_name, 0, sizeof(w_name));
	strlcpy(w_name, wsa884x->dai_driver->playback.stream_name,
				sizeof(w_name));
	snd_soc_dapm_ignore_suspend(dapm, w_name);

	memset(w_name, 0, sizeof(w_name));
	strlcpy(w_name, "IN", sizeof(w_name));
	snd_soc_dapm_ignore_suspend(dapm, w_name);

	memset(w_name, 0, sizeof(w_name));
	strlcpy(w_name, "SWR DAC_Port", sizeof(w_name));
	snd_soc_dapm_ignore_suspend(dapm, w_name);

	memset(w_name, 0, sizeof(w_name));
	strlcpy(w_name, "SPKR", sizeof(w_name));
	snd_soc_dapm_ignore_suspend(dapm, w_name);

	snd_soc_dapm_sync(dapm);

	return 0;
}

static void wsa884x_codec_remove(struct snd_soc_component *component)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	if (!wsa884x)
		return;

	snd_soc_component_exit_regmap(component);

	return;
}

static int wsa884x_soc_codec_suspend(struct snd_soc_component *component)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	if (!wsa884x)
		return 0;

	wsa884x->dapm_bias_off = true;
	return 0;
}

static int wsa884x_soc_codec_resume(struct snd_soc_component *component)
{
	struct wsa884x_priv *wsa884x = snd_soc_component_get_drvdata(component);

	if (!wsa884x)
		return 0;

	wsa884x->dapm_bias_off = false;
	return 0;
}

static const struct snd_soc_component_driver soc_codec_dev_wsa884x_wsa = {
	.name = "",
	.probe = wsa884x_codec_probe,
	.remove = wsa884x_codec_remove,
	.controls = wsa884x_snd_controls,
	.num_controls = ARRAY_SIZE(wsa884x_snd_controls),
	.dapm_widgets = wsa884x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wsa884x_dapm_widgets),
	.dapm_routes = wsa884x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wsa884x_audio_map),
	.suspend =  wsa884x_soc_codec_suspend,
	.resume = wsa884x_soc_codec_resume,
};

static int wsa884x_gpio_ctrl(struct wsa884x_priv *wsa884x, bool enable)
{
	int ret = 0;

	if (enable)
		ret = msm_cdc_pinctrl_select_active_state(
						wsa884x->wsa_rst_np);
	else
		ret = msm_cdc_pinctrl_select_sleep_state(
						wsa884x->wsa_rst_np);
	if (ret != 0)
		dev_err_ratelimited(wsa884x->dev,
			"%s: Failed to turn state %d; ret=%d\n",
			__func__, enable, ret);

	return ret;
}

static int wsa884x_swr_up(struct wsa884x_priv *wsa884x)
{
	int ret;

	ret = wsa884x_gpio_ctrl(wsa884x, true);
	if (ret)
		dev_err_ratelimited(wsa884x->dev, "%s: Failed to enable gpio\n", __func__);

	return ret;
}

static int wsa884x_swr_down(struct wsa884x_priv *wsa884x)
{
	int ret;

	ret = wsa884x_gpio_ctrl(wsa884x, false);
	if (ret)
		dev_err_ratelimited(wsa884x->dev, "%s: Failed to disable gpio\n", __func__);

	return ret;
}

static int wsa884x_swr_reset(struct wsa884x_priv *wsa884x)
{
	u8 retry = WSA884X_NUM_RETRY;
	u8 devnum = 0;
	struct swr_device *pdev;

	pdev = wsa884x->swr_slave;
	while (swr_get_logical_dev_num(pdev, pdev->addr, &devnum) && retry--) {
		/* Retry after 1 msec delay */
		usleep_range(1000, 1100);
	}
	pdev->dev_num = devnum;
	wsa884x_regcache_sync(wsa884x);

	return 0;
}

static int wsa884x_event_notify(struct notifier_block *nb,
				unsigned long val, void *ptr)
{
	u16 event = (val & 0xffff);
	struct wsa884x_priv *wsa884x = container_of(nb, struct wsa884x_priv,
						    parent_nblock);

	if (!wsa884x)
		return -EINVAL;

	switch (event) {
	case BOLERO_SLV_EVT_PA_OFF_PRE_SSR:
		if (test_bit(SPKR_STATUS, &wsa884x->status_mask))
			snd_soc_component_update_bits(wsa884x->component,
				REG_FIELD_VALUE(PA_FSM_EN, GLOBAL_PA_EN, 0x00));
		wsa884x_swr_down(wsa884x);
		break;

	case BOLERO_SLV_EVT_SSR_UP:
		wsa884x_swr_up(wsa884x);
		/* Add delay to allow enumerate */
		usleep_range(20000, 20010);
		wsa884x_swr_reset(wsa884x);
		dev_err(wsa884x->dev, "%s: BOLERO_SLV_EVT_SSR_UP Called", __func__);
		swr_init_port_params(wsa884x->swr_slave, WSA884X_MAX_SWR_PORTS,
			wsa884x->swr_wsa_port_params);
		break;

	case BOLERO_SLV_EVT_PA_ON_POST_FSCLK:
		if (test_bit(SPKR_STATUS, &wsa884x->status_mask)) {
			snd_soc_component_update_bits(wsa884x->component,
				REG_FIELD_VALUE(PDM_WD_CTL, PDM_WD_EN, 0x01));
			snd_soc_component_update_bits(wsa884x->component,
				REG_FIELD_VALUE(PA_FSM_EN, GLOBAL_PA_EN, 0x01));
		}
		break;
	case BOLERO_SLV_EVT_PA_ON_POST_FSCLK_ADIE_LB:
		if (test_bit(SPKR_STATUS, &wsa884x->status_mask))
			set_bit(SPKR_ADIE_LB, &wsa884x->status_mask);
		break;
	default:
		dev_dbg(wsa884x->dev, "%s: unknown event %d\n",
			__func__, event);
		break;
	}

	return 0;
}

static int wsa884x_parse_port_params(struct device *dev, char *prop)
{
	u32 *dt_array, map_size, max_uc;
	int ret = 0;
	u32 cnt = 0;
	u32 i, j;
	struct swr_port_params (*map)[SWR_UC_MAX][WSA884X_MAX_SWR_PORTS];
	struct swr_dev_frame_config (*map_uc)[SWR_UC_MAX];
	struct wsa884x_priv *wsa884x = dev_get_drvdata(dev);

	map = &wsa884x->wsa_port_params;
	map_uc = &wsa884x->swr_wsa_port_params;

	if (!of_find_property(dev->of_node, prop,
				&map_size)) {
		dev_err(dev, "missing port mapping prop %s\n", prop);
		ret = -EINVAL;
		goto err_port_map;
	}

	max_uc = map_size / (WSA884X_MAX_SWR_PORTS * SWR_PORT_PARAMS * sizeof(u32));

	if (max_uc != SWR_UC_MAX) {
		dev_err(dev, "%s: port params not provided for all usecases\n",
			__func__);
		ret = -EINVAL;
		goto err_port_map;
	}
	dt_array = kzalloc(map_size, GFP_KERNEL);

	if (!dt_array) {
		ret = -ENOMEM;
		goto err_port_map;
	}
	ret = of_property_read_u32_array(dev->of_node, prop, dt_array,
				WSA884X_MAX_SWR_PORTS * SWR_PORT_PARAMS * max_uc);
	if (ret) {
		dev_err(dev, "%s: Failed to read port mapping from prop %s\n",
					__func__, prop);
		goto err_pdata_fail;
	}

	for (i = 0; i < max_uc; i++) {
		for (j = 0; j < WSA884X_MAX_SWR_PORTS; j++) {
			cnt = (i * WSA884X_MAX_SWR_PORTS + j) * SWR_PORT_PARAMS;
			(*map)[i][j].offset1 = dt_array[cnt];
			(*map)[i][j].lane_ctrl = dt_array[cnt + 1];
		}
		(*map_uc)[i].pp = &(*map)[i][0];
	}
	kfree(dt_array);
	return 0;

err_pdata_fail:
	kfree(dt_array);
err_port_map:
	return ret;
}

static int wsa884x_enable_supplies(struct device *dev,
				   struct wsa884x_priv *priv)
{
	int ret = 0;

	/* Parse power supplies */
	msm_cdc_get_power_supplies(dev, &priv->regulator,
				   &priv->num_supplies);
	if (!priv->regulator || (priv->num_supplies <= 0)) {
		dev_err(dev, "%s: no power supplies defined\n", __func__);
		return -EINVAL;
	}

	ret = msm_cdc_init_supplies(dev, &priv->supplies,
				    priv->regulator, priv->num_supplies);
	if (!priv->supplies) {
		dev_err(dev, "%s: Cannot init wsa supplies\n",
			__func__);
		return ret;
	}

	ret = msm_cdc_enable_static_supplies(dev, priv->supplies,
					     priv->regulator,
					     priv->num_supplies);
	if (ret)
		dev_err(dev, "%s: wsa static supply enable failed!\n",
			__func__);

	return ret;
}

static struct snd_soc_dai_driver wsa_dai[] = {
	{
		.name = "",
		.playback = {
			.stream_name = "",
			.rates = WSA884X_RATES | WSA884X_FRAC_RATES,
			.formats = WSA884X_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
	},
};

static int wsa884x_swr_probe(struct swr_device *pdev)
{
	int ret = 0;
	struct wsa884x_priv *wsa884x;
	u8 devnum = 0;
	bool pin_state_current = false;
	struct wsa_ctrl_platform_data *plat_data = NULL;
	struct snd_soc_component *component;
	u32 noise_gate_mode;
	char buffer[MAX_NAME_LEN];
	int dev_index = 0;
	struct regmap_irq_chip *wsa884x_sub_regmap_irq_chip = NULL;
	u8 wo0_val;
	int sys_gain_size, sys_gain_length;
	int wsa_dev_index;


	wsa884x = devm_kzalloc(&pdev->dev, sizeof(struct wsa884x_priv),
			    GFP_KERNEL);
	if (!wsa884x)
		return -ENOMEM;

	wsa884x_sub_regmap_irq_chip = devm_kzalloc(&pdev->dev, sizeof(struct regmap_irq_chip),
				 GFP_KERNEL);
	if (!wsa884x_sub_regmap_irq_chip)
		return -ENOMEM;
	memcpy(wsa884x_sub_regmap_irq_chip, &wsa884x_regmap_irq_chip,
			sizeof(struct regmap_irq_chip));

	ret = wsa884x_enable_supplies(&pdev->dev, wsa884x);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err;
	}

	wsa884x->wsa_rst_np = of_parse_phandle(pdev->dev.of_node,
					     "qcom,spkr-sd-n-node", 0);
	if (!wsa884x->wsa_rst_np) {
		dev_dbg(&pdev->dev, "%s: pinctrl not defined\n", __func__);
		goto err_supply;
	}
	swr_set_dev_data(pdev, wsa884x);
	wsa884x->swr_slave = pdev;
	wsa884x->dev = &pdev->dev;
	pin_state_current = msm_cdc_pinctrl_get_state(wsa884x->wsa_rst_np);
	wsa884x_gpio_ctrl(wsa884x, true);
	/*
	 * Add 5msec delay to provide sufficient time for
	 * soundwire auto enumeration of slave devices as
	 * per HW requirement.
	 */
	usleep_range(5000, 5010);
	ret = swr_get_logical_dev_num(pdev, pdev->addr, &devnum);
	if (ret) {
		dev_dbg(&pdev->dev,
			"%s get devnum %d for dev addr %lx failed\n",
			__func__, devnum, pdev->addr);
		ret = -EPROBE_DEFER;
		goto err_supply;
	}
	pdev->dev_num = devnum;

	wsa884x->regmap = devm_regmap_init_swr(pdev,
					       &wsa884x_regmap_config);
	if (IS_ERR(wsa884x->regmap)) {
		ret = PTR_ERR(wsa884x->regmap);
		dev_err(&pdev->dev, "%s: regmap_init failed %d\n",
			__func__, ret);
		goto dev_err;
	}

	devm_regmap_qti_debugfs_register(&pdev->dev, wsa884x->regmap);

	wsa884x_sub_regmap_irq_chip->irq_drv_data = wsa884x;
	wsa884x->irq_info.wcd_regmap_irq_chip = wsa884x_sub_regmap_irq_chip;
	wsa884x->irq_info.codec_name = "WSA884X";
	wsa884x->irq_info.regmap = wsa884x->regmap;
	wsa884x->irq_info.dev = &pdev->dev;
	ret = wcd_irq_init(&wsa884x->irq_info, &wsa884x->virq);

	if (ret) {
		dev_err(wsa884x->dev, "%s: IRQ init failed: %d\n",
			__func__, ret);
		goto dev_err;
	}

	wsa884x->swr_slave->slave_irq = wsa884x->virq;

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_SAF2WAR,
			"WSA SAF2WAR", wsa884x_saf2war_handle_irq, wsa884x);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_WAR2SAF,
			"WSA WAR2SAF", wsa884x_war2saf_handle_irq, wsa884x);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_DISABLE,
			"WSA OTP", wsa884x_otp_handle_irq, wsa884x);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_OCP,
			"WSA OCP", wsa884x_ocp_handle_irq, wsa884x);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_CLIP,
			"WSA CLIP", wsa884x_clip_handle_irq, wsa884x);

	wcd_disable_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_CLIP);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_PDM_WD,
			"WSA PDM WD", wsa884x_pdm_wd_handle_irq, wsa884x);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_CLK_WD,
			"WSA CLK WD", wsa884x_clk_wd_handle_irq, wsa884x);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_INTR_PIN,
			"WSA EXT INT", wsa884x_ext_int_handle_irq, wsa884x);

	wcd_disable_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_INTR_PIN);

	/* Under Voltage Lock out (UVLO) interrupt handle */
	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_UVLO,
			"WSA UVLO", wsa884x_uvlo_handle_irq, wsa884x);

	wcd_request_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_PA_ON_ERR,
			"WSA PA ERR", wsa884x_pa_on_err_handle_irq, wsa884x);

	wsa884x->driver = devm_kzalloc(&pdev->dev,
			sizeof(struct snd_soc_component_driver), GFP_KERNEL);
	if (!wsa884x->driver) {
		ret = -ENOMEM;
		goto err_irq;
	}

	memcpy(wsa884x->driver, &soc_codec_dev_wsa884x_wsa,
			sizeof(struct snd_soc_component_driver));

	wsa884x->dai_driver = devm_kzalloc(&pdev->dev,
				sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!wsa884x->dai_driver) {
		ret = -ENOMEM;
		goto err_mem;
	}

	memcpy(wsa884x->dai_driver, wsa_dai, sizeof(struct snd_soc_dai_driver));

	/* Get last digit from HEX format */
	dev_index = (int)((char)(pdev->addr & 0xF));
	dev_index += 1;

	if (of_device_is_compatible(pdev->dev.of_node, "qcom,wsa884x_2"))
		dev_index += 2;

	snprintf(buffer, sizeof(buffer), "wsa-codec.%d", dev_index);
	wsa884x->driver->name = kstrndup(buffer, strlen(buffer), GFP_KERNEL);

	snprintf(buffer, sizeof(buffer), "wsa_rx%d", dev_index);
	wsa884x->dai_driver->name =
				kstrndup(buffer, strlen(buffer), GFP_KERNEL);

	snprintf(buffer, sizeof(buffer), "WSA884X_AIF%d Playback", dev_index);
	wsa884x->dai_driver->playback.stream_name =
				kstrndup(buffer, strlen(buffer), GFP_KERNEL);

	/* Number of DAI's used is 1 */
	ret = snd_soc_register_component(&pdev->dev,
				wsa884x->driver, wsa884x->dai_driver, 1);

	component = snd_soc_lookup_component(&pdev->dev, wsa884x->driver->name);
	if (!component) {
		dev_err(&pdev->dev, "%s: component is NULL\n", __func__);
		ret = -EINVAL;
		goto err_mem;
	}

	wsa884x->parent_np = of_parse_phandle(pdev->dev.of_node,
					      "qcom,bolero-handle", 0);
	if (!wsa884x->parent_np)
		wsa884x->parent_np = of_parse_phandle(pdev->dev.of_node,
					      "qcom,lpass-cdc-handle", 0);
	if (wsa884x->parent_np) {
		wsa884x->parent_dev =
				of_find_device_by_node(wsa884x->parent_np);
		if (wsa884x->parent_dev) {
			plat_data = dev_get_platdata(&wsa884x->parent_dev->dev);
			if (plat_data) {
				wsa884x->parent_nblock.notifier_call =
							wsa884x_event_notify;
				if (plat_data->register_notifier)
					plat_data->register_notifier(
						plat_data->handle,
						&wsa884x->parent_nblock,
						true);
				wsa884x->register_notifier =
						plat_data->register_notifier;
				wsa884x->handle = plat_data->handle;
			} else {
				dev_err(&pdev->dev, "%s: plat data not found\n",
					__func__);
			}
		} else {
			dev_err(&pdev->dev, "%s: parent dev not found\n",
				__func__);
		}
	} else {
		dev_info(&pdev->dev, "%s: parent node not found\n", __func__);
	}

	/* Start in speaker mode by default */
	wsa884x->dev_mode = SPEAKER;
	wsa884x->dev_index = dev_index;
	/* wsa_dev_index is macro_agnostic index */
	wsa_dev_index = (wsa884x->dev_index - 1) % 2;
	wsa884x->macro_np = of_parse_phandle(pdev->dev.of_node,
				"qcom,wsa-macro-handle", 0);
	if (wsa884x->macro_np) {
		wsa884x->macro_dev =
				of_find_device_by_node(wsa884x->macro_np);
		if (wsa884x->macro_dev) {
			ret = of_property_read_u32_index(
				wsa884x->macro_dev->dev.of_node,
				"qcom,wsa-rloads",
				wsa_dev_index,
				&wsa884x->rload);
			if (ret) {
				dev_err(&pdev->dev,
					"%s: Failed to read wsa rloads\n",
							__func__);
				goto err_mem;
			}

			ret = of_property_read_u32_index(
				wsa884x->macro_dev->dev.of_node,
				"qcom,wsa-bat-cfgs",
				wsa_dev_index,
				&wsa884x->bat_cfg);
			if (ret) {
				dev_err(&pdev->dev,
					"%s: Failed to read wsa bat cfgs\n",
							__func__);
				goto err_mem;
			}

			ret = of_property_read_u32(wsa884x->macro_dev->dev.of_node,
				"qcom,noise-gate-mode", &noise_gate_mode);
			if (ret) {
				dev_info(&pdev->dev,
					"%s: Failed to read wsa noise gate mode\n",
						__func__);
				wsa884x->noise_gate_mode = IDLE_DETECT;
			} else {
				if (IDLE_DETECT <= noise_gate_mode && noise_gate_mode <= NG3)
					wsa884x->noise_gate_mode = noise_gate_mode;
				else
					wsa884x->noise_gate_mode = IDLE_DETECT;
			}

			if (!of_find_property(wsa884x->macro_dev->dev.of_node,
				"qcom,wsa-system-gains", &sys_gain_size)) {
				dev_err(&pdev->dev,
					"%s: missing wsa-system-gains\n",
					__func__);
				goto err_mem;
			}

			sys_gain_length = sys_gain_size / (2 * sizeof(u32));
			ret = of_property_read_u32_array(
				wsa884x->macro_dev->dev.of_node,
				"qcom,wsa-system-gains", wsa884x->sys_gains,
				sys_gain_length);

			if (ret) {
				dev_err(&pdev->dev,
					"%s: Failed to read wsa system gains\n",
						__func__);
				goto err_mem;
			}
			wsa884x->system_gain = wsa884x->sys_gains[
				wsa884x->dev_mode + wsa_dev_index * 2];
		} else {
			dev_err(&pdev->dev, "%s: parent dev not found\n",
				__func__);
			goto err_mem;
		}
	} else {
		dev_err(&pdev->dev, "%s: parent node not found\n", __func__);
		goto err_mem;
	}

	dev_dbg(component->dev,
		"%s: Bat_cfg: 0x%x rload: 0x%x, sys_gain: 0x%x\n", __func__,
		wsa884x->bat_cfg, wsa884x->rload, wsa884x->system_gain);
	ret = wsa884x_validate_dt_configuration_params(component, wsa884x->rload,
		wsa884x->bat_cfg, wsa884x->system_gain);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: invalid dt parameter: Bat_cfg: 0x%x rload: 0x%x, sys_gain: 0x%x\n",
			__func__, wsa884x->bat_cfg, wsa884x->rload, wsa884x->system_gain);
		ret = -EINVAL;
		goto err_mem;
	}
	/* Assume that compander is enabled by default unless it is haptics sku */
	if (wsa884x->variant == WSA8845H)
		wsa884x->comp_enable = false;
	else
		wsa884x->comp_enable = true;
	wsa884x_set_gain_parameters(component);
	wsa884x_set_pbr_parameters(component);
	/* Must write WO registers in a single write */
	wo0_val = (0xC | (wsa884x->pa_aux_gain << 0x02) | !wsa884x->dev_mode);
	snd_soc_component_write(component, WSA884X_ANA_WO_CTL_0, wo0_val);
	snd_soc_component_write(component, WSA884X_ANA_WO_CTL_1, 0x0);
	if (wsa884x->rload == WSA_4_OHMS || wsa884x->rload == WSA_6_OHMS)
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(OCP_CTL, OCP_CURR_LIMIT, 0x07));

	if (wsa884x->dev_mode == SPEAKER) {
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(DRE_CTL_0, PROG_DELAY, 0x0F));
	} else {
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(DRE_CTL_0, PROG_DELAY, 0x03));
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(CDC_PATH_MODE, RXD_MODE, 0x01));
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(PWM_CLK_CTL,
			PWM_CLK_FREQ_SEL, 0x01));
	}
	if (wsa884x->bat_cfg != CONFIG_1S && wsa884x->bat_cfg != EXT_1S)
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(TOP_CTRL1,
			OCP_LOWVBAT_ITH_SEL_EN, 0x00));
	ret = wsa884x_parse_port_params(&pdev->dev, "qcom,swr-wsa-port-params");
	if (ret) {
		dev_err(&pdev->dev, "Failed to read port params\n");
		goto err;
	}
	swr_init_port_params(wsa884x->swr_slave, WSA884X_MAX_SWR_PORTS,
		wsa884x->swr_wsa_port_params);
	mutex_init(&wsa884x->res_lock);

#ifdef CONFIG_DEBUG_FS
	if (!wsa884x->debugfs_dent) {
		wsa884x->debugfs_dent = debugfs_create_dir(
					dev_name(&pdev->dev), 0);
		if (!IS_ERR(wsa884x->debugfs_dent)) {
			wsa884x->debugfs_peek =
				debugfs_create_file("swrslave_peek",
				S_IFREG | 0444,
				wsa884x->debugfs_dent,
				(void *) pdev,
				&codec_debug_read_ops);

		wsa884x->debugfs_poke =
				debugfs_create_file("swrslave_poke",
				S_IFREG | 0444,
				wsa884x->debugfs_dent,
				(void *) pdev,
				&codec_debug_write_ops);

		wsa884x->debugfs_reg_dump =
				debugfs_create_file(
				"swrslave_reg_dump",
				S_IFREG | 0444,
				wsa884x->debugfs_dent,
				(void *) pdev,
				&codec_debug_dump_ops);
	}
}
#endif

	return 0;

err_mem:
	if (wsa884x->dai_driver) {
		kfree(wsa884x->dai_driver->name);
		kfree(wsa884x->dai_driver->playback.stream_name);
		kfree(wsa884x->dai_driver);
	}
	if (wsa884x->driver) {
		kfree(wsa884x->driver->name);
		kfree(wsa884x->driver);
	}
err_irq:
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_SAF2WAR, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_WAR2SAF, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_DISABLE, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_OCP, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_CLIP, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_PDM_WD, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_CLK_WD, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_INTR_PIN, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_UVLO, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_PA_ON_ERR, NULL);
	wcd_irq_exit(&wsa884x->irq_info, wsa884x->virq);
dev_err:
	if (pin_state_current == false)
		wsa884x_gpio_ctrl(wsa884x, false);
	swr_remove_device(pdev);
err_supply:
	msm_cdc_release_supplies(&pdev->dev, wsa884x->supplies,
				 wsa884x->regulator,
				 wsa884x->num_supplies);
err:
	swr_set_dev_data(pdev, NULL);
	return ret;
}

static int wsa884x_swr_remove(struct swr_device *pdev)
{
	struct wsa884x_priv *wsa884x;

	wsa884x = swr_get_dev_data(pdev);
	if (!wsa884x) {
		dev_err(&pdev->dev, "%s: wsa884x is NULL\n", __func__);
		return -EINVAL;
	}

	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_SAF2WAR, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_WAR2SAF, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_DISABLE, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_OCP, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_CLIP, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_PDM_WD, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_CLK_WD, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_INTR_PIN, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_UVLO, NULL);
	wcd_free_irq(&wsa884x->irq_info, WSA884X_IRQ_INT_PA_ON_ERR, NULL);

	if (wsa884x->register_notifier)
		wsa884x->register_notifier(wsa884x->handle,
				&wsa884x->parent_nblock, false);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(wsa884x->debugfs_dent);
	wsa884x->debugfs_dent = NULL;
#endif
	mutex_destroy(&wsa884x->res_lock);
	snd_soc_unregister_component(&pdev->dev);
	if (wsa884x->dai_driver) {
		kfree(wsa884x->dai_driver->name);
		kfree(wsa884x->dai_driver->playback.stream_name);
		kfree(wsa884x->dai_driver);
	}
	if (wsa884x->driver) {
		kfree(wsa884x->driver->name);
		kfree(wsa884x->driver);
	}
	msm_cdc_release_supplies(&pdev->dev, wsa884x->supplies,
				 wsa884x->regulator,
				 wsa884x->num_supplies);
	swr_set_dev_data(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wsa884x_swr_suspend(struct device *dev)
{
	struct wsa884x_priv *wsa884x = swr_get_dev_data(to_swr_device(dev));

	if (!wsa884x) {
		dev_err_ratelimited(dev, "%s: wsa884x private data is NULL\n", __func__);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: system suspend\n", __func__);
	if (wsa884x->dapm_bias_off ||
		(snd_soc_component_get_bias_level(wsa884x->component) ==
		 SND_SOC_BIAS_OFF)) {
		msm_cdc_set_supplies_lpm_mode(dev, wsa884x->supplies,
					wsa884x->regulator,
					wsa884x->num_supplies,
					true);
		set_bit(WSA_SUPPLIES_LPM_MODE, &wsa884x->status_mask);
	}
	return 0;
}

static int wsa884x_swr_resume(struct device *dev)
{
	struct wsa884x_priv *wsa884x = swr_get_dev_data(to_swr_device(dev));

	if (!wsa884x) {
		dev_err(dev, "%s: wsa884x private data is NULL\n", __func__);
		return -EINVAL;
	}
	if (test_bit(WSA_SUPPLIES_LPM_MODE, &wsa884x->status_mask)) {
		msm_cdc_set_supplies_lpm_mode(dev, wsa884x->supplies,
					wsa884x->regulator,
					wsa884x->num_supplies,
					false);
		clear_bit(WSA_SUPPLIES_LPM_MODE, &wsa884x->status_mask);
	}
	dev_dbg(dev, "%s: system resume\n", __func__);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops wsa884x_swr_pm_ops = {
	.suspend_late = wsa884x_swr_suspend,
	.resume_early = wsa884x_swr_resume,
};

static const struct swr_device_id wsa884x_swr_id[] = {
	{"wsa884x", 0},
	{"wsa884x_2", 0},
	{}
};

static const struct of_device_id wsa884x_swr_dt_match[] = {
	{
		.compatible = "qcom,wsa884x",
	},
	{
		.compatible = "qcom,wsa884x_2",
	},
	{}
};

static struct swr_driver wsa884x_swr_driver = {
	.driver = {
		.name = "wsa884x",
		.owner = THIS_MODULE,
		.pm = &wsa884x_swr_pm_ops,
		.of_match_table = wsa884x_swr_dt_match,
	},
	.probe = wsa884x_swr_probe,
	.remove = wsa884x_swr_remove,
	.id_table = wsa884x_swr_id,
};

static int __init wsa884x_swr_init(void)
{
	return swr_driver_register(&wsa884x_swr_driver);
}

static void __exit wsa884x_swr_exit(void)
{
	swr_driver_unregister(&wsa884x_swr_driver);
}

module_init(wsa884x_swr_init);
module_exit(wsa884x_swr_exit);

MODULE_DESCRIPTION("WSA884x codec driver");
MODULE_LICENSE("GPL v2");
