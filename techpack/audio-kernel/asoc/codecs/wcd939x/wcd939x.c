// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/component.h>
#include <linux/stringify.h>
#include <linux/regulator/consumer.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <soc/soundwire.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asoc/wcdcal-hwdep.h>
#include <asoc/msm-cdc-pinctrl.h>
#include <asoc/msm-cdc-supply.h>
#include <asoc/wcd-mbhc-v2-api.h>
#include <bindings/audio-codec-port-types.h>
#include <linux/qti-regmap-debugfs.h>
#include "wcd939x-registers.h"
#include "wcd939x.h"
#include "internal.h"
#include "asoc/bolero-slave-internal.h"
#include "wcd939x-reg-masks.h"
#include "wcd939x-reg-shifts.h"
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
#include <linux/soc/qcom/wcd939x-i2c.h>
#endif


#define NUM_SWRS_DT_PARAMS 5
#define WCD939X_VARIANT_ENTRY_SIZE 32

#define WCD939X_VERSION_ENTRY_SIZE 32

#define ADC_MODE_VAL_HIFI     0x01
#define ADC_MODE_VAL_LO_HIF   0x02
#define ADC_MODE_VAL_NORMAL   0x03
#define ADC_MODE_VAL_LP       0x05
#define ADC_MODE_VAL_ULP1     0x09
#define ADC_MODE_VAL_ULP2     0x0B

#define HPH_IMPEDANCE_2VPK_MODE_OHMS 260
#define XTALK_L_CH_NUM 0
#define XTALK_R_CH_NUM 1
#define GND_EXT_FET_MARGIN_MOHMS 200

#define NUM_ATTEMPTS 5
#define COMP_MAX_COEFF 25
#define HPH_MODE_MAX 4

#define WCD_USBSS_WRITE true
#define WCD_USBSS_READ false
#define WCD_USBSS_DP_EN 0x1E
#define WCD_USBSS_DN_EN 0x21
#define P_THRESH_SEL_MASK 0x0E
#define P_THRESH_SEL_SHIFT 0x01
#define VTH_4P0 0x04
#define VTH_4P2 0x06

#define DAPM_MICBIAS1_STANDALONE "MIC BIAS1 Standalone"
#define DAPM_MICBIAS2_STANDALONE "MIC BIAS2 Standalone"
#define DAPM_MICBIAS3_STANDALONE "MIC BIAS3 Standalone"
#define DAPM_MICBIAS4_STANDALONE "MIC BIAS4 Standalone"

#define WCD939X_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
			SNDRV_PCM_RATE_384000)
/* Fractional Rates */
#define WCD939X_FRAC_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800)

#define WCD939X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
		SNDRV_PCM_FMTBIT_S24_LE |\
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

#define REG_FIELD_VALUE(register_name, field_name, value) \
WCD939X_##register_name, FIELD_MASK(register_name, field_name), \
value << FIELD_SHIFT(register_name, field_name)

#define WCD939X_COMP_OFFSET \
		(WCD939X_R_BASE - WCD939X_COMPANDER_HPHL_BASE)

#define WCD939X_XTALK_OFFSET \
		(WCD939X_HPHR_RX_PATH_SEC0 - WCD939X_HPHL_RX_PATH_SEC0)

enum {
	CODEC_TX = 0,
	CODEC_RX,
};

enum {
	WCD_ADC1 = 0,
	WCD_ADC2,
	WCD_ADC3,
	WCD_ADC4,
	ALLOW_BUCK_DISABLE,
	HPH_COMP_DELAY,
	HPH_PA_DELAY,
	AMIC2_BCS_ENABLE,
	WCD_SUPPLIES_LPM_MODE,
	WCD_ADC1_MODE,
	WCD_ADC2_MODE,
	WCD_ADC3_MODE,
	WCD_ADC4_MODE,
};

enum {
	ADC_MODE_INVALID = 0,
	ADC_MODE_HIFI,
	ADC_MODE_LO_HIF,
	ADC_MODE_NORMAL,
	ADC_MODE_LP,
	ADC_MODE_ULP1,
	ADC_MODE_ULP2,
};

enum {
	SUPPLY_LEVEL_2VPK,
	REGULATOR_MODE_2VPK,
	SET_HPH_GAIN_2VPK,
};

static u8 tx_mode_bit[] = {
	[ADC_MODE_INVALID] = 0x00,
	[ADC_MODE_HIFI] = 0x01,
	[ADC_MODE_LO_HIF] = 0x02,
	[ADC_MODE_NORMAL] = 0x04,
	[ADC_MODE_LP] = 0x08,
	[ADC_MODE_ULP1] = 0x10,
	[ADC_MODE_ULP2] = 0x20,
};

extern const u8 wcd939x_reg_access[WCD939X_NUM_REGISTERS];
static const SNDRV_CTL_TLVD_DECLARE_DB_MINMAX(hph_analog_gain, 600, -3000);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);

/* Will be set by reading the registers during bind()*/
static int wcd939x_version = WCD939X_VERSION_2_0;

static int wcd939x_handle_post_irq(void *data);
static int wcd939x_reset(struct device *dev);
static int wcd939x_reset_low(struct device *dev);
static int wcd939x_get_adc_mode(int val);
static void wcd939x_config_2Vpk_mode(struct snd_soc_component *component,
			struct wcd939x_priv *wcd939x, int mode_2vpk);

static const struct regmap_irq wcd939x_irqs[WCD939X_NUM_IRQS] = {
	REGMAP_IRQ_REG(WCD939X_IRQ_MBHC_BUTTON_PRESS_DET, 0, 0x01),
	REGMAP_IRQ_REG(WCD939X_IRQ_MBHC_BUTTON_RELEASE_DET, 0, 0x02),
	REGMAP_IRQ_REG(WCD939X_IRQ_MBHC_ELECT_INS_REM_DET, 0, 0x04),
	REGMAP_IRQ_REG(WCD939X_IRQ_MBHC_ELECT_INS_REM_LEG_DET, 0, 0x08),
	REGMAP_IRQ_REG(WCD939X_IRQ_MBHC_SW_DET, 0, 0x10),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHR_OCP_INT, 0, 0x20),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHR_CNP_INT, 0, 0x40),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHL_OCP_INT, 0, 0x80),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHL_CNP_INT, 1, 0x01),
	REGMAP_IRQ_REG(WCD939X_IRQ_EAR_CNP_INT, 1, 0x02),
	REGMAP_IRQ_REG(WCD939X_IRQ_EAR_SCD_INT, 1, 0x04),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHL_PDM_WD_INT, 1, 0x20),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHR_PDM_WD_INT, 1, 0x40),
	REGMAP_IRQ_REG(WCD939X_IRQ_EAR_PDM_WD_INT, 1, 0x80),
	REGMAP_IRQ_REG(WCD939X_IRQ_MBHC_MOISTURE_INT, 2, 0x02),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHL_SURGE_DET_INT, 2, 0x04),
	REGMAP_IRQ_REG(WCD939X_IRQ_HPHR_SURGE_DET_INT, 2, 0x08),
};

static struct regmap_irq_chip wcd939x_regmap_irq_chip = {
	.name = "wcd939x",
	.irqs = wcd939x_irqs,
	.num_irqs = ARRAY_SIZE(wcd939x_irqs),
	.num_regs = 3,
	.status_base = WCD939X_INTR_STATUS_0,
	.mask_base = WCD939X_INTR_MASK_0,
	.type_base = WCD939X_INTR_LEVEL_0,
	.ack_base = WCD939X_INTR_CLEAR_0,
	.use_ack = 1,
	.runtime_pm = false,
	.handle_post_irq = wcd939x_handle_post_irq,
	.irq_drv_data = NULL,
};

static bool wcd939x_readable_register(struct device *dev, unsigned int reg)
{
	if (reg <= WCD939X_BASE + 1)
		return 0;

	if (reg >= WCD939X_FLYBACK_NEW_CTRL_2 && reg <= WCD939X_FLYBACK_NEW_CTRL_4) {
		if (wcd939x_version == WCD939X_VERSION_1_0)
			return 0;
	}
	return wcd939x_reg_access[WCD939X_REG(reg)] & RD_REG;
}

static int wcd939x_handle_post_irq(void *data)
{
	struct wcd939x_priv *wcd939x = data;
	u32 sts1 = 0, sts2 = 0, sts3 = 0;

	regmap_read(wcd939x->regmap, WCD939X_INTR_STATUS_0, &sts1);
	regmap_read(wcd939x->regmap, WCD939X_INTR_STATUS_1, &sts2);
	regmap_read(wcd939x->regmap, WCD939X_INTR_STATUS_2, &sts3);

	wcd939x->tx_swr_dev->slave_irq_pending =
			((sts1 || sts2 || sts3) ? true : false);

	return IRQ_HANDLED;
}

static int wcd939x_hph_compander_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	int compander = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;

	ucontrol->value.integer.value[0] = wcd939x->compander_enabled[compander];
	return 0;
}


static int wcd939x_hph_compander_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	int compander = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;

	int value = ucontrol->value.integer.value[0];

	if (value < WCD939X_HPH_MAX && value >= 0)
		wcd939x->compander_enabled[compander] = value;
	else {
		dev_err(component->dev, "%s: Invalid comp value = %d\n", __func__, value);
		return -EINVAL;
	}

	dev_dbg(component->dev, "%s: Compander %d  value %d\n",
			__func__, wcd939x->compander_enabled[compander], value);
	return 0;
}

static int wcd939x_hph_xtalk_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	int xtalk = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;

	int value = ucontrol->value.integer.value[0];

	if (value < WCD939X_HPH_MAX && value >= 0)
		wcd939x->xtalk_enabled[xtalk] = value;
	else {
		dev_err(component->dev, "%s: Invalid xtalk value = %d\n", __func__, value);
		return -EINVAL;
	}

	 dev_dbg(component->dev, "%s: xtalk %d  value %d\n",
			 __func__, wcd939x->xtalk_enabled[xtalk], value);

	return 0;

}

static int wcd939x_hph_xtalk_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	int xtalk = ((struct soc_multi_mixer_control *)
			kcontrol->private_value)->shift;

	ucontrol->value.integer.value[0] = wcd939x->xtalk_enabled[xtalk];

	return 0;
}

static int wcd939x_hph_pcm_enable_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	wcd939x->hph_pcm_enabled = ucontrol->value.integer.value[0];
	dev_dbg(component->dev, "%s: pcm enabled %d \n",
			 __func__, wcd939x->hph_pcm_enabled);
	return 0;

}

static int wcd939x_hph_pcm_enable_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
					snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wcd939x->hph_pcm_enabled;

	return 0;
}

static int wcd939x_swr_slv_get_current_bank(struct swr_device *dev, u8 devnum)
{
	int ret = 0;
	int bank = 0;

	ret = swr_read(dev, devnum, SWR_SCP_CONTROL, &bank, 1);
	if (ret)
		return -EINVAL;

	return ((bank & 0x40) ? 1: 0);
}

static int wcd939x_get_clk_rate(int mode)
{
	int rate;

	switch (mode) {
	case ADC_MODE_ULP2:
		rate = SWR_CLK_RATE_0P6MHZ;
		break;
	case ADC_MODE_ULP1:
		rate = SWR_CLK_RATE_1P2MHZ;
		break;
	case ADC_MODE_LP:
		rate = SWR_CLK_RATE_4P8MHZ;
		break;
	case ADC_MODE_NORMAL:
	case ADC_MODE_LO_HIF:
	case ADC_MODE_HIFI:
	case ADC_MODE_INVALID:
	default:
		rate = SWR_CLK_RATE_9P6MHZ;
		break;
	}

	return rate;
}

static int wcd939x_set_swr_clk_rate(struct snd_soc_component *component,
					int rate, int bank)
{
	u8 mask = (bank ? 0xF0 : 0x0F);
	u8 val = 0;

	switch (rate) {
	case SWR_CLK_RATE_0P6MHZ:
		val = (bank ? 0x60 : 0x06);
		break;
	case SWR_CLK_RATE_1P2MHZ:
		val = (bank ? 0x50 : 0x05);
		break;
	case SWR_CLK_RATE_2P4MHZ:
		val = (bank ? 0x30 : 0x03);
		break;
	case SWR_CLK_RATE_4P8MHZ:
		val = (bank ? 0x10 : 0x01);
		break;
	case SWR_CLK_RATE_9P6MHZ:
	default:
		val = 0x00;
		break;
	}
	snd_soc_component_update_bits(component,
				      WCD939X_SWR_TX_CLK_RATE,
				      mask, val);

	return 0;
}

static int wcd939x_init_reg(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(BIAS, ANALOG_BIAS_EN, 0x01));
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(BIAS, PRECHRG_EN, 0x01));

	/* 10 msec delay as per HW requirement */
	usleep_range(10000, 10010);
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(BIAS, PRECHRG_EN, 0x00));

	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(RDAC_HD2_CTL_L, HD2_RES_DIV_CTL_L, 0x15));
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(RDAC_HD2_CTL_R, HD2_RES_DIV_CTL_R, 0x15));
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_DMIC_CTL, CLK_SCALE_EN, 0x01));

	snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(TXFE_ICTRL_STG2CASC_ULP, ICTRL_SCBIAS_ULP0P6M, 0x1));
	snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(TXFE_ICTRL_STG2CASC_ULP, ICTRL_STG2CASC_ULP, 0x4));

	snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(TXFE_ICTRL_STG2MAIN_ULP, ICTRL_STG2MAIN_ULP, 0x08));


	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(TEST_CTL_1, NOISE_FILT_RES_VAL, 0x07));
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(MICB2_TEST_CTL_1, NOISE_FILT_RES_VAL, 0x07));
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(MICB3_TEST_CTL_1, NOISE_FILT_RES_VAL, 0x07));
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(MICB4_TEST_CTL_1, NOISE_FILT_RES_VAL, 0x07));
	snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(TEST_BLK_EN2, TXFE2_MBHC_CLKRST_EN, 0x00));

	if (of_find_property(component->card->dev->of_node, "qcom,wcd-disable-legacy-surge", NULL)) {
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(HPHLR_SURGE_EN, EN_SURGE_PROTECTION_HPHL, 0x00));
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(HPHLR_SURGE_EN, EN_SURGE_PROTECTION_HPHR, 0x00));
	}
	else {
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(HPHLR_SURGE_EN, EN_SURGE_PROTECTION_HPHL, 0x01));
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(HPHLR_SURGE_EN, EN_SURGE_PROTECTION_HPHR, 0x01));
	}

	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(HPH_OCP_CTL, OCP_FSM_EN, 0x01));
	snd_soc_component_update_bits(component,
		REG_FIELD_VALUE(HPH_OCP_CTL, SCD_OP_EN, 0x01));

	if (wcd939x->version != WCD939X_VERSION_2_0)
		snd_soc_component_write(component, WCD939X_CFG0, 0x05);

	/*
	 * Disable 1M pull-up by default during boot by writing 0b1 to bit[7].
	 * This gets re-enabled when headset is inserted.
	 */
	snd_soc_component_update_bits(component, WCD939X_ZDET_BIAS_CTL, 0x80, 0x80);
	return 0;
}

static int wcd939x_set_port_params(struct snd_soc_component *component,
			u8 slv_prt_type, u8 *port_id, u8 *num_ch,
			u8 *ch_mask, u32 *ch_rate,
			u8 *port_type, u8 path)
{
	int i, j;
	u8 num_ports = 0;
	struct codec_port_info (*map)[MAX_PORT][MAX_CH_PER_PORT];
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	switch (path) {
	case CODEC_RX:
		map = &wcd939x->rx_port_mapping;
		num_ports = wcd939x->num_rx_ports;
		break;
	case CODEC_TX:
		map = &wcd939x->tx_port_mapping;
		num_ports = wcd939x->num_tx_ports;
		break;
	default:
		dev_err_ratelimited(component->dev, "%s Invalid path selected %u\n",
					__func__, path);
		return -EINVAL;
	}

	for (i = 0; i <= num_ports; i++) {
		for (j = 0; j < MAX_CH_PER_PORT; j++) {
			if ((*map)[i][j].slave_port_type == slv_prt_type)
				goto found;
		}
	}
found:
	if (i > num_ports || j == MAX_CH_PER_PORT) {
		dev_err_ratelimited(component->dev, "%s Failed to find slave port for type %u\n",
						__func__, slv_prt_type);
		return -EINVAL;
	}
	*port_id = i;
	*num_ch = (*map)[i][j].num_ch;
	*ch_mask = (*map)[i][j].ch_mask;
	*ch_rate = (*map)[i][j].ch_rate;
	*port_type = (*map)[i][j].master_port_type;

	return 0;
}


/* qcom,swr-tx-port-params = <OFFSET1_VAL0 LANE1>, <OFFSET1_VAL5 LANE0>, <OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>,*UC0*
			<OFFSET1_VAL0 LANE1>, <OFFSET1_VAL2 LANE0>, <OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>, *UC1*
			<OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>; *UC2*
			<OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>, <OFFSET1_VAL1 LANE0>; *UC3 */
static int wcd939x_parse_port_params(struct device *dev,
			char *prop, u8 path)
{
	u32 *dt_array, map_size, max_uc;
	int ret = 0;
	u32 cnt = 0;
	u32 i, j;
	struct swr_port_params (*map)[SWR_UC_MAX][SWR_NUM_PORTS];
	struct swr_dev_frame_config (*map_uc)[SWR_UC_MAX];
	struct wcd939x_priv *wcd939x = dev_get_drvdata(dev);

	switch (path) {
	case CODEC_TX:
		map = &wcd939x->tx_port_params;
		map_uc = &wcd939x->swr_tx_port_params;
		break;
	default:
		ret = -EINVAL;
		goto err_port_map;
	}

	if (!of_find_property(dev->of_node, prop,
				&map_size)) {
		dev_err(dev, "missing port mapping prop %s\n", prop);
		ret = -EINVAL;
		goto err_port_map;
	}

	max_uc = map_size / (SWR_NUM_PORTS * SWR_PORT_PARAMS * sizeof(u32));

	if (max_uc != SWR_UC_MAX) {
		dev_err(dev, "%s: port params not provided for all usecases\n",
			__func__);
		ret = -EINVAL;
		goto err_port_map;
	}
	dt_array = kzalloc(map_size, GFP_KERNEL);

	if (!dt_array) {
		ret = -ENOMEM;
		goto err_alloc;
	}
	ret = of_property_read_u32_array(dev->of_node, prop, dt_array,
				SWR_NUM_PORTS * SWR_PORT_PARAMS * max_uc);
	if (ret) {
		dev_err(dev, "%s: Failed to read  port mapping from prop %s\n",
					__func__, prop);
		goto err_pdata_fail;
	}

	for (i = 0; i < max_uc; i++) {
		for (j = 0; j < SWR_NUM_PORTS; j++) {
			cnt = (i * SWR_NUM_PORTS + j) * SWR_PORT_PARAMS;
			(*map)[i][j].offset1 = dt_array[cnt];
			(*map)[i][j].lane_ctrl = dt_array[cnt + 1];
		}
		(*map_uc)[i].pp = &(*map)[i][0];
	}
	kfree(dt_array);
	return 0;

err_pdata_fail:
	kfree(dt_array);
err_alloc:
err_port_map:
	return ret;
}

static int wcd939x_parse_port_mapping(struct device *dev,
			char *prop, u8 path)
{
	u32 *dt_array, map_size, map_length;
	u32 port_num = 0, ch_mask, ch_rate, old_port_num = 0;
	u32 slave_port_type, master_port_type;
	u32 i, ch_iter = 0;
	int ret = 0;
	u8 *num_ports = NULL;
	struct codec_port_info (*map)[MAX_PORT][MAX_CH_PER_PORT];
	struct wcd939x_priv *wcd939x = dev_get_drvdata(dev);

	switch (path) {
	case CODEC_RX:
		map = &wcd939x->rx_port_mapping;
		num_ports = &wcd939x->num_rx_ports;
		break;
	case CODEC_TX:
		map = &wcd939x->tx_port_mapping;
		num_ports = &wcd939x->num_tx_ports;
		break;
	default:
		dev_err(dev, "%s Invalid path selected %u\n",
			      __func__, path);
		return -EINVAL;
	}

	if (!of_find_property(dev->of_node, prop,
				&map_size)) {
		dev_err(dev, "missing port mapping prop %s\n", prop);
		ret = -EINVAL;
		goto err_port_map;
	}

	map_length = map_size / (NUM_SWRS_DT_PARAMS * sizeof(u32));

	dt_array = kzalloc(map_size, GFP_KERNEL);

	if (!dt_array) {
		ret = -ENOMEM;
		goto err_alloc;
	}
	ret = of_property_read_u32_array(dev->of_node, prop, dt_array,
				NUM_SWRS_DT_PARAMS * map_length);
	if (ret) {
		dev_err(dev, "%s: Failed to read  port mapping from prop %s\n",
					__func__, prop);
		goto err_pdata_fail;
	}

	for (i = 0; i < map_length; i++) {
		port_num = dt_array[NUM_SWRS_DT_PARAMS * i];
		slave_port_type = dt_array[NUM_SWRS_DT_PARAMS * i + 1];
		ch_mask = dt_array[NUM_SWRS_DT_PARAMS * i + 2];
		ch_rate = dt_array[NUM_SWRS_DT_PARAMS * i + 3];
		master_port_type = dt_array[NUM_SWRS_DT_PARAMS * i + 4];

		if (port_num != old_port_num)
			ch_iter = 0;

		(*map)[port_num][ch_iter].slave_port_type = slave_port_type;
		(*map)[port_num][ch_iter].ch_mask = ch_mask;
		(*map)[port_num][ch_iter].master_port_type = master_port_type;
		(*map)[port_num][ch_iter].num_ch = __sw_hweight8(ch_mask);
		(*map)[port_num][ch_iter++].ch_rate = ch_rate;
		old_port_num = port_num;
	}
	*num_ports = port_num;
	kfree(dt_array);
	return 0;

err_pdata_fail:
	kfree(dt_array);
err_alloc:
err_port_map:
	return ret;
}

static int wcd939x_tx_connect_port(struct snd_soc_component *component,
					u8 slv_port_type, int clk_rate,
					u8 enable)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	u8 port_id, num_ch, ch_mask;
	u8 ch_type = 0;
	u32 ch_rate;
	int slave_ch_idx;
	u8 num_port = 1;
	int ret = 0;

	ret = wcd939x_set_port_params(component, slv_port_type, &port_id,
				&num_ch, &ch_mask, &ch_rate,
				&ch_type, CODEC_TX);
	if (ret)
		return ret;

	if (clk_rate)
		ch_rate = clk_rate;

	slave_ch_idx = wcd939x_slave_get_slave_ch_val(slv_port_type);
	if (slave_ch_idx != -EINVAL)
		ch_type = wcd939x->tx_master_ch_map[slave_ch_idx];

	dev_dbg(component->dev, "%s slv_ch_idx: %d, mstr_ch_type: %d\n",
		__func__, slave_ch_idx, ch_type);

	if (enable)
		ret = swr_connect_port(wcd939x->tx_swr_dev, &port_id,
					num_port, &ch_mask, &ch_rate,
					 &num_ch, &ch_type);
	else
		ret = swr_disconnect_port(wcd939x->tx_swr_dev, &port_id,
					num_port, &ch_mask, &ch_type);
	return ret;

}
static int wcd939x_rx_connect_port(struct snd_soc_component *component,
					u8 slv_port_type, u8 enable)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	u8 port_id, num_ch, ch_mask, port_type;
	u32 ch_rate;
	u8 num_port = 1;
	int ret = 0;

	ret = wcd939x_set_port_params(component, slv_port_type, &port_id,
				&num_ch, &ch_mask, &ch_rate,
				&port_type, CODEC_RX);

	if (ret)
		return ret;

	if (enable)
		ret = swr_connect_port(wcd939x->rx_swr_dev, &port_id,
					num_port, &ch_mask, &ch_rate,
					&num_ch, &port_type);
	else
		ret = swr_disconnect_port(wcd939x->rx_swr_dev, &port_id,
					num_port, &ch_mask, &port_type);
	return ret;
}

static int wcd939x_rx_clk_enable(struct snd_soc_component *component, int rx_num)
{

	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s rx_clk_cnt: %d\n", __func__, wcd939x->rx_clk_cnt);

	if (wcd939x->rx_clk_cnt == 0) {
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RX_SUPPLIES, RX_BIAS_ENABLE, 0x01));

		/*Analog path clock controls*/
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_RX_CLK_EN, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_RX_DIV2_CLK_EN, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_RX_DIV4_CLK_EN, 0x01));

		/*Digital path clock controls*/
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, RXD0_CLK_EN, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, RXD1_CLK_EN, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, RXD2_CLK_EN, 0x01));

	}
	wcd939x->rx_clk_cnt++;

	return 0;
}

static int wcd939x_rx_clk_disable(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s rx_clk_cnt: %d\n", __func__, wcd939x->rx_clk_cnt);

	if (wcd939x->rx_clk_cnt == 0)
		return 0;

	wcd939x->rx_clk_cnt--;
	if (wcd939x->rx_clk_cnt == 0) {

		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RX_SUPPLIES, VNEG_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RX_SUPPLIES, VPOS_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, RXD2_CLK_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, RXD1_CLK_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, RXD0_CLK_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_RX_DIV4_CLK_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_RX_DIV2_CLK_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_RX_CLK_EN, 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RX_SUPPLIES, RX_BIAS_ENABLE, 0x00));

	}
	return 0;
}

/*
 * wcd939x_soc_get_mbhc: get wcd939x_mbhc handle of corresponding component
 * @component: handle to snd_soc_component *
 *
 * return wcd939x_mbhc handle or error code in case of failure
 */
struct wcd939x_mbhc *wcd939x_soc_get_mbhc(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x;

	if (!component) {
		pr_err_ratelimited("%s: Invalid params, NULL component\n", __func__);
		return NULL;
	}
	wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x) {
		pr_err_ratelimited("%s: wcd939x is NULL\n", __func__);
		return NULL;
	}

	return wcd939x->mbhc;
}
EXPORT_SYMBOL(wcd939x_soc_get_mbhc);

static int  wcd939x_config_power_mode(struct snd_soc_component *component,
				int event, int index, int mode)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (mode == CLS_H_ULP) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(REFBUFF_UHQA_CTL, REFBUFP_IOUT_CTL, 0x1));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(REFBUFF_UHQA_CTL, REFBUFN_IOUT_CTL, 0x1));

			if (wcd939x->compander_enabled[index]) {
				if (index == WCD939X_HPHL) {
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL12, ZONE3_RMS, 0x21));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL13, ZONE4_RMS, 0x30));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL14, ZONE5_RMS, 0x3F));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL15, ZONE6_RMS, 0x48));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL17, PATH_GAIN, 0x0C));
				} else if (index == WCD939X_HPHR) {
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL12, ZONE3_RMS, 0x21));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL13, ZONE4_RMS, 0x30));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL14, ZONE5_RMS, 0x3F));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL15, ZONE6_RMS, 0x48));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL17, PATH_GAIN, 0x0C));
				}
			}
		} else {
			if (wcd939x->compander_enabled[index]) {
				if (index == WCD939X_HPHL) {
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL12, ZONE3_RMS, 0x1E));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL13, ZONE4_RMS, 0x2A));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL14, ZONE5_RMS, 0x36));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL15, ZONE6_RMS, 0x3C));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(CTL17, PATH_GAIN, 0x00));
				} else if (index == WCD939X_HPHR) {
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL12, ZONE3_RMS, 0x1E));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL13, ZONE4_RMS, 0x2A));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL14, ZONE5_RMS, 0x36));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL15, ZONE6_RMS, 0x3C));
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_CTL17, PATH_GAIN, 0x00));
				}
			}
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (mode == CLS_H_ULP) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(REFBUFF_UHQA_CTL, REFBUFN_IOUT_CTL, 0x0));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(REFBUFF_UHQA_CTL, REFBUFP_IOUT_CTL, 0x0));
		}
		break;
	}
	return 0;
}

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
static int wcd939x_get_usbss_hph_power_mode(int hph_mode)
{
	switch (hph_mode) {
	case CLS_H_HIFI:
	case CLS_H_LOHIFI:
		return 0x4;
	default:
		/* set default mode to ULP */
		return 0x2;
	}
}
#endif

static int wcd939x_enable_hph_pcm_index(struct snd_soc_component *component,
				int event, int hph)
{
	struct wcd939x_priv *wcd939x = NULL;

	if (!component) {
		pr_err_ratelimited("%s: Invalid params, NULL component\n", __func__);
		return -EINVAL;
	}

	wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x->hph_pcm_enabled)
		return 0;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (hph == WCD939X_HPHL) {
			if (wcd939x->rx_clk_config == RX_CLK_11P2896MHZ)
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPHL_RX_PATH_CFG1,
					RX_DC_DROOP_COEFF_SEL, 0x2));
			else if (wcd939x->rx_clk_config == RX_CLK_9P6MHZ)
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPHL_RX_PATH_CFG1,
					RX_DC_DROOP_COEFF_SEL, 0x3));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(HPHL_RX_PATH_CFG0,
				DLY_ZN_EN, 0x1));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(HPHL_RX_PATH_CFG0,
				INT_EN, 0x3));
		} else if (hph == WCD939X_HPHR) {
			if (wcd939x->rx_clk_config == RX_CLK_11P2896MHZ)
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPHR_RX_PATH_CFG1,
					RX_DC_DROOP_COEFF_SEL, 0x2));
			else if (wcd939x->rx_clk_config == RX_CLK_9P6MHZ)
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPHR_RX_PATH_CFG1,
					RX_DC_DROOP_COEFF_SEL, 0x3));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(HPHR_RX_PATH_CFG0,
				DLY_ZN_EN, 0x1));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(HPHR_RX_PATH_CFG0,
				INT_EN, 0x3));
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		break;
	}

	return 0;
}

static int wcd939x_config_compander(struct snd_soc_component *component,
				int event, int compander_indx)
{
	u16 comp_ctl7_reg = 0, comp_ctl0_reg = 0;
	u16 comp_en_mask_val = 0, gain_source_sel = 0;
	struct wcd939x_priv *wcd939x;

	if (compander_indx >= WCD939X_HPH_MAX || compander_indx < 0) {
		pr_err_ratelimited("%s: Invalid compander value: %d\n",
				__func__, compander_indx);
		return -EINVAL;
	}

	if (!component) {
		pr_err_ratelimited("%s: Invalid params, NULL component\n", __func__);
		return -EINVAL;
	}
	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x->hph_pcm_enabled)
		return 0;

	dev_dbg(component->dev, "%s compander_index = %d\n", __func__, compander_indx);

	if (!wcd939x->compander_enabled[compander_indx]) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			gain_source_sel = 0x01;
		else
			gain_source_sel = 0x00;

		if (compander_indx == WCD939X_HPHL) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(L_EN, GAIN_SOURCE_SEL, gain_source_sel));
		} else if (compander_indx == WCD939X_HPHR) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(R_EN, GAIN_SOURCE_SEL, gain_source_sel));
		}
		wcd939x_config_2Vpk_mode(component, wcd939x, SET_HPH_GAIN_2VPK);
		return 0;
	}

	if (compander_indx == WCD939X_HPHL)
		comp_en_mask_val = 1 << 1;
	else if (compander_indx == WCD939X_HPHR)
		comp_en_mask_val = 1 << 0;
	else
		return 0;

	comp_ctl0_reg  = WCD939X_CTL0 + (compander_indx * WCD939X_COMP_OFFSET);
	comp_ctl7_reg = WCD939X_CTL7 + (compander_indx * WCD939X_COMP_OFFSET);

	if (SND_SOC_DAPM_EVENT_ON(event)) {

		snd_soc_component_update_bits(component,
			comp_ctl7_reg, 0x1E, 0x00);
		/* Enable compander clock*/
		snd_soc_component_update_bits(component,
			comp_ctl0_reg , 0x01, 0x01);

		/* 250us sleep required as per HW Sequence */
		usleep_range(250, 260);
		snd_soc_component_update_bits(component,
			comp_ctl0_reg, 0x02, 0x02);
		snd_soc_component_update_bits(component,
			comp_ctl0_reg, 0x02, 0x00);

		/* Enable compander*/
		snd_soc_component_update_bits(component,
				WCD939X_CDC_COMP_CTL_0, comp_en_mask_val, comp_en_mask_val);

	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
			snd_soc_component_update_bits(component,
				WCD939X_CDC_COMP_CTL_0, comp_en_mask_val, 0x00);
			snd_soc_component_update_bits(component,
				comp_ctl0_reg , 0x01, 0x00);
			if (compander_indx == WCD939X_HPHL)
				snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(L_EN, GAIN_SOURCE_SEL, 0x0));
			if (compander_indx == WCD939X_HPHR)
				snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(R_EN, GAIN_SOURCE_SEL, 0x0));
	}

	return 0;
}

static int wcd939x_config_xtalk(struct snd_soc_component *component,
					int event, int xtalk_indx)
{
	u16 xtalk_sec0 = 0, xtalk_sec1 = 0, xtalk_sec2 = 0, xtalk_sec3 = 0;
	struct wcd939x_priv *wcd939x = NULL;
	struct wcd939x_pdata *pdata = NULL;
	if (!component) {
		pr_err_ratelimited("%s: Invalid params, NULL component\n", __func__);
		return -EINVAL;
	}

	wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x->xtalk_enabled[xtalk_indx])
		return 0;

	pdata = dev_get_platdata(wcd939x->dev);

	dev_dbg(component->dev, "%s xtalk_indx = %d event = %d\n",
					__func__, xtalk_indx, event);

	switch(event) {

	case SND_SOC_DAPM_PRE_PMU:
		xtalk_sec0 = WCD939X_HPHL_RX_PATH_SEC0 + (xtalk_indx * WCD939X_XTALK_OFFSET);
		xtalk_sec1 = WCD939X_HPHL_RX_PATH_SEC1 + (xtalk_indx * WCD939X_XTALK_OFFSET);
		xtalk_sec2 = WCD939X_HPHL_RX_PATH_SEC2 + (xtalk_indx * WCD939X_XTALK_OFFSET);
		xtalk_sec3 = WCD939X_HPHL_RX_PATH_SEC3 + (xtalk_indx * WCD939X_XTALK_OFFSET);

		/* Write scale and alpha based on channel */
		if (xtalk_indx == XTALK_L_CH_NUM) {
			snd_soc_component_update_bits(component, xtalk_sec1, 0xFF,
						      pdata->usbcss_hs.xtalk.alpha_l);
			snd_soc_component_update_bits(component, xtalk_sec0, 0x1F,
						      pdata->usbcss_hs.xtalk.scale_l);
		} else if (xtalk_indx == XTALK_R_CH_NUM) {
			snd_soc_component_update_bits(component, xtalk_sec1, 0xFF,
						      pdata->usbcss_hs.xtalk.alpha_r);
			snd_soc_component_update_bits(component, xtalk_sec0, 0x1F,
						      pdata->usbcss_hs.xtalk.scale_r);
		} else {
			snd_soc_component_update_bits(component, xtalk_sec1, 0xFF, MIN_XTALK_ALPHA);
			snd_soc_component_update_bits(component, xtalk_sec0, 0x1F, MAX_XTALK_SCALE);
		}

		dev_dbg(component->dev, "%s Scale = 0x%x, Alpha = 0x%x\n", __func__,
			snd_soc_component_read(component, xtalk_sec0),
			snd_soc_component_read(component, xtalk_sec1));

		snd_soc_component_update_bits(component, xtalk_sec3, 0xFF, 0x4F);
		snd_soc_component_update_bits(component, xtalk_sec2, 0x1F, 0x11);

		break;
	case SND_SOC_DAPM_POST_PMU:
		/* enable xtalk for L and R channels*/
		snd_soc_component_update_bits(component, WCD939X_RX_PATH_CFG2,
					0x0F, 0x0F);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Disable Xtalk for L and R channels*/
		snd_soc_component_update_bits(component, WCD939X_RX_PATH_CFG2,
				0x00, 0x00);
		break;
	}
	return 0;
}

static int wcd939x_rx3_mux(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	dev_dbg(component->dev, "%s event: %d wshift: %d wname: %s\n",
					__func__, event, w->shift, w->name);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x_rx_clk_enable(component, w->shift);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_rx_clk_disable(component);
		break;
	}

	return 0;
}

static int wcd939x_rx_mux(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol,
			int event)
{

	int hph_mode = 0;
	struct wcd939x_priv *wcd939x = NULL;
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	wcd939x = snd_soc_component_get_drvdata(component);
	hph_mode = wcd939x->hph_mode;

	dev_dbg(component->dev, "%s event: %d wshift: %d wname: %s\n",
					__func__, event, w->shift, w->name);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x_rx_clk_enable(component, w->shift);
		if (wcd939x->hph_pcm_enabled)
			wcd939x_config_power_mode(component, event, w->shift, hph_mode);
		wcd939x_config_compander(component, event, w->shift);
		wcd939x_config_xtalk(component, event, w->shift);
		break;
	case SND_SOC_DAPM_POST_PMU:
		wcd939x_config_xtalk(component, event, w->shift);
		/*TBD: need to revisit , for both L & R we are updating, but in QCRG only once*/
		if (wcd939x->hph_pcm_enabled) {
			if (hph_mode == CLS_H_HIFI || hph_mode == CLS_AB_HIFI)
				snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(TOP_CFG0, HPH_DAC_RATE_SEL, 0x1));
			else
				snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(TOP_CFG0, HPH_DAC_RATE_SEL, 0x0));
		}
		wcd939x_enable_hph_pcm_index(component, event, w->shift);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_config_xtalk(component, event, w->shift);
		wcd939x_config_compander(component, event, w->shift);
		if (wcd939x->hph_pcm_enabled)
			wcd939x_config_power_mode(component, event, w->shift, hph_mode);
		wcd939x_rx_clk_disable(component);
		break;

	}

	return 0;
}

static void wcd939x_config_2Vpk_mode(struct snd_soc_component *component,
			struct wcd939x_priv *wcd939x, int mode_2vpk)
{
	uint32_t zl = 0, zr = 0;
	int rc;

	if (!wcd939x->in_2Vpk_mode)
		return;

	rc = wcd_mbhc_get_impedance(&wcd939x->mbhc->wcd_mbhc, &zl, &zr);
	if (rc) {
		dev_err_ratelimited(component->dev, "%s: Unable to get impedance for 2Vpk mode", __func__);
		return;
	}

	switch (mode_2vpk) {
	case SUPPLY_LEVEL_2VPK:
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(PA_GAIN_CTL_L, RX_SUPPLY_LEVEL, 0x01));

		if (zl < HPH_IMPEDANCE_2VPK_MODE_OHMS)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PA_GAIN_CTL_L, EN_HPHPA_2VPK, 0x00));
		else
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PA_GAIN_CTL_L, EN_HPHPA_2VPK, 0x01));
		break;
	case REGULATOR_MODE_2VPK:
		if (zl >= HPH_IMPEDANCE_2VPK_MODE_OHMS) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RX_SUPPLIES, REGULATOR_MODE, 0x01));
			snd_soc_component_update_bits(component, WCD939X_FLYBACK_TEST_CTL,
					0x0F, 0x02);
		} else {
			snd_soc_component_update_bits(component, WCD939X_FLYBACK_TEST_CTL,
					0x0F, 0x0D);
		}
		break;
	case SET_HPH_GAIN_2VPK:
		if (zl >= HPH_IMPEDANCE_2VPK_MODE_OHMS) {
			snd_soc_component_update_bits(component, WCD939X_PA_GAIN_CTL_L, 0x1F, 0x02);
			snd_soc_component_update_bits(component, WCD939X_PA_GAIN_CTL_R, 0x1F, 0x02);
		}

		break;
	}
}

static int wcd939x_codec_hphl_dac_event(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (!wcd939x->hph_pcm_enabled)
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RDAC_CLK_CTL1, OPAMP_CHOP_CLK_EN, 0x00));
		wcd939x_config_2Vpk_mode(component, wcd939x, SUPPLY_LEVEL_2VPK);

		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_HPH_GAIN_CTL, HPHL_RX_EN, 0x01));
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(RDAC_HD2_CTL_L, HD2_RES_DIV_CTL_L, 0x1D));
		if (!wcd939x->hph_pcm_enabled) {
			if (wcd939x->comp1_enable) {
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_COMP_CTL_0, HPHL_COMP_EN, 0x01));
				 /* 5msec compander delay as per HW requirement */
				if (!wcd939x->comp2_enable ||
					(snd_soc_component_read(component,
						WCD939X_CDC_COMP_CTL_0) & 0x01))
				usleep_range(5000, 5010);
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH_TIMER1, AUTOCHOP_TIMER_CTL_EN, 0x00));
			} else {
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_COMP_CTL_0, HPHL_COMP_EN, 0x00));
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(L_EN, GAIN_SOURCE_SEL, 0x01));
			}
		}
		if (wcd939x->hph_pcm_enabled) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH_TIMER1, AUTOCHOP_TIMER_CTL_EN, 0x00));
			snd_soc_component_write(component, WCD939X_VNEG_CTRL_1, 0xEB);
			if (wcd939x->hph_mode == CLS_H_LOHIFI)
				snd_soc_component_write(component,
					WCD939X_HPH_RDAC_BIAS_LOHIFI, 0x52);
			else
				snd_soc_component_write(component,
					WCD939X_HPH_RDAC_BIAS_LOHIFI, 0x64);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RDAC_HD2_CTL_L, HD2_RES_DIV_CTL_L, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_HPH_GAIN_CTL, HPHL_RX_EN, 0x00));
		break;
	}

	return 0;
}

static int wcd939x_codec_hphr_dac_event(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (!wcd939x->hph_pcm_enabled)
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RDAC_CLK_CTL1, OPAMP_CHOP_CLK_EN, 0x00));
		wcd939x_config_2Vpk_mode(component, wcd939x, SUPPLY_LEVEL_2VPK);

		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_HPH_GAIN_CTL, HPHR_RX_EN, 0x01));
		break;
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RDAC_HD2_CTL_R, HD2_RES_DIV_CTL_R, 0x1D));
		if (!wcd939x->hph_pcm_enabled) {
			if (wcd939x->comp1_enable) {
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_COMP_CTL_0, HPHR_COMP_EN, 0x01));
				 /* 5msec compander delay as per HW requirement */
				if (!wcd939x->comp2_enable ||
					(snd_soc_component_read(component,
						WCD939X_CDC_COMP_CTL_0) & 0x02))
				usleep_range(5000, 5010);
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH_TIMER1, AUTOCHOP_TIMER_CTL_EN, 0x00));
			} else {
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_COMP_CTL_0, HPHR_COMP_EN, 0x00));
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(R_EN, GAIN_SOURCE_SEL, 0x01));
			}
		}
		if (wcd939x->hph_pcm_enabled) {
			snd_soc_component_write(component, WCD939X_VNEG_CTRL_1, 0xEB);
			if (wcd939x->hph_mode == CLS_H_LOHIFI)
				snd_soc_component_write(component,
					WCD939X_HPH_RDAC_BIAS_LOHIFI, 0x52);
			else
				snd_soc_component_write(component,
					WCD939X_HPH_RDAC_BIAS_LOHIFI, 0x64);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(RDAC_HD2_CTL_R, HD2_RES_DIV_CTL_R, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_HPH_GAIN_CTL, HPHR_RX_EN, 0x00));
		break;
	}

	return 0;
}

static int wcd939x_codec_ear_dac_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_EAR_GAIN_CTL, EAR_EN, 0x01));

		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(EAR_DAC_CON, DAC_SAMPLE_EDGE_SEL, 0x00));
		/* 5 msec delay as per HW requirement */
		usleep_range(5000, 5010);
		wcd_cls_h_fsm(component, &wcd939x->clsh_info,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_EAR,
			     CLS_AB_HIFI);
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(VNEG_CTRL_4, ILIM_SEL, 0xD));
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(EAR_DAC_CON, DAC_SAMPLE_EDGE_SEL, 0x01));
		break;
	};
	return 0;

}

static int wcd939x_codec_enable_hphr_pa(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int ret = 0;
	int hph_mode = wcd939x->hph_mode;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (wcd939x->ldoh)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(MODE, LDOH_EN, 0x01));
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX2 << 0x10 | 0x1));
		ret = swr_slvdev_datapath_control(wcd939x->rx_swr_dev,
				    wcd939x->rx_swr_dev->dev_num,
				    true);
		wcd_cls_h_fsm(component, &wcd939x->clsh_info,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_HPHR,
			     hph_mode);
		wcd939x_config_2Vpk_mode(component, wcd939x, REGULATOR_MODE_2VPK);
		if (hph_mode == CLS_H_LP || hph_mode == CLS_H_LOHIFI ||
		    hph_mode == CLS_H_ULP) {
			if (!wcd939x->hph_pcm_enabled)
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(REFBUFF_LP_CTL, PREREF_FILT_BYPASS, 0x01));
		}
		/* update Mode for LOHIFI */
		if (hph_mode == CLS_H_LOHIFI) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, PWR_LEVEL, 0x00));
		}
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		/* update USBSS power mode for AATC */
		if (wcd939x->mbhc->wcd_mbhc.mbhc_cfg->enable_usbc_analog)
			wcd_usbss_audio_config(NULL, WCD_USBSS_CONFIG_TYPE_POWER_MODE,
				wcd939x_get_usbss_hph_power_mode(hph_mode));
#endif
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(VNEG_CTRL_4, ILIM_SEL, 0xD));
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHR_REF_ENABLE, 0x01));
		if ((snd_soc_component_read(component, WCD939X_HPH) & 0x30) == 0x30)
			usleep_range(2500, 2600); /* 2.5msec delay as per HW requirement */
		set_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		if (!wcd939x->hph_pcm_enabled)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PDM_WD_CTL1, PDM_WD_EN, 0x03));
		break;
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * 7ms sleep is required if compander is enabled as per
		 * HW requirement. If compander is disabled, then
		 * 20ms delay is required.
		 */
		if (test_bit(HPH_PA_DELAY, &wcd939x->status_mask)) {
			if (!wcd939x->comp2_enable)
				usleep_range(20000, 20100);
			else
				usleep_range(7000, 7100);
			if (hph_mode == CLS_H_LP ||
			    hph_mode == CLS_H_LOHIFI ||
			    hph_mode == CLS_H_ULP)
				if (!wcd939x->hph_pcm_enabled)
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(REFBUFF_LP_CTL, PREREF_FILT_BYPASS, 0x00));
			clear_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		}
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(HPH_TIMER1, AUTOCHOP_TIMER_CTL_EN, 0x01));
		if (hph_mode == CLS_AB || hph_mode == CLS_AB_HIFI ||
			hph_mode == CLS_AB_LP || hph_mode == CLS_AB_LOHIFI)
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RX_SUPPLIES, REGULATOR_MODE, 0x01));
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX2 << 0x10));
		/*Enable PDM INT for PDM data path only*/
		if (!wcd939x->hph_pcm_enabled)
			wcd_enable_irq(&wcd939x->irq_info,
					WCD939X_IRQ_HPHR_PDM_WD_INT);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX2 << 0x10 | 0x1));
		wcd_disable_irq(&wcd939x->irq_info,
					WCD939X_IRQ_HPHR_PDM_WD_INT);
		if (wcd939x->update_wcd_event && wcd939x->comp2_enable)
			wcd939x->update_wcd_event(wcd939x->handle,
					SLV_BOLERO_EVT_RX_COMPANDER_SOFT_RST,
					(WCD_RX2 << 0x10));
		/*
		 * 7ms sleep is required if compander is enabled as per
		 * HW requirement. If compander is disabled, then
		 * 20ms delay is required.
		 */
		if (!wcd939x->comp2_enable)
			usleep_range(20000, 20100);
		else
			usleep_range(7000, 7100);
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHR_ENABLE, 0x00));

		blocking_notifier_call_chain(&wcd939x->mbhc->notifier,
					     WCD_EVENT_PRE_HPHR_PA_OFF,
					     &wcd939x->mbhc->wcd_mbhc);
		set_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/*
		 * 7ms sleep is required if compander is enabled as per
		 * HW requirement. If compander is disabled, then
		 * 20ms delay is required.
		 */
		if (test_bit(HPH_PA_DELAY, &wcd939x->status_mask)) {
			if (!wcd939x->comp2_enable)
				usleep_range(20000, 20100);
			else
				usleep_range(7000, 7100);
			clear_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		}
		blocking_notifier_call_chain(&wcd939x->mbhc->notifier,
					     WCD_EVENT_POST_HPHR_PA_OFF,
					     &wcd939x->mbhc->wcd_mbhc);

		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHR_REF_ENABLE, 0x00));
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PDM_WD_CTL1, PDM_WD_EN, 0x00));
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		if (wcd939x->mbhc->wcd_mbhc.mbhc_cfg->enable_usbc_analog &&
			!(snd_soc_component_read(component, WCD939X_HPH) & 0XC0))
			wcd_usbss_audio_config(NULL, WCD_USBSS_CONFIG_TYPE_POWER_MODE, 1);
#endif
		wcd_cls_h_fsm(component, &wcd939x->clsh_info,
			     WCD_CLSH_EVENT_POST_PA,
			     WCD_CLSH_STATE_HPHR,
			     hph_mode);
		if (wcd939x->ldoh)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(MODE, LDOH_EN, 0x00));
		break;
	};
	return ret;
}

static int wcd939x_codec_enable_hphl_pa(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int ret = 0;
	int hph_mode = wcd939x->hph_mode;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (wcd939x->ldoh)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(MODE, LDOH_EN, 0x01));
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX1 << 0x10 | 0x01));
		ret = swr_slvdev_datapath_control(wcd939x->rx_swr_dev,
				    wcd939x->rx_swr_dev->dev_num,
				    true);
		wcd_cls_h_fsm(component, &wcd939x->clsh_info,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_HPHL,
			     hph_mode);
		wcd939x_config_2Vpk_mode(component, wcd939x, REGULATOR_MODE_2VPK);
		if (hph_mode == CLS_H_LP || hph_mode == CLS_H_LOHIFI ||
		    hph_mode == CLS_H_ULP) {
			if (!wcd939x->hph_pcm_enabled)
				snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(REFBUFF_LP_CTL, PREREF_FILT_BYPASS, 0x01));
		}
		/* update Mode for LOHIFI */
		if (hph_mode == CLS_H_LOHIFI) {
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(HPH, PWR_LEVEL, 0x00));
		}
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		/* update USBSS power mode for AATC */
		if (wcd939x->mbhc->wcd_mbhc.mbhc_cfg->enable_usbc_analog)
			wcd_usbss_audio_config(NULL, WCD_USBSS_CONFIG_TYPE_POWER_MODE,
				wcd939x_get_usbss_hph_power_mode(hph_mode));
#endif
		snd_soc_component_update_bits(component,
			REG_FIELD_VALUE(VNEG_CTRL_4, ILIM_SEL, 0xD));
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHL_REF_ENABLE, 0x01));
		if ((snd_soc_component_read(component, WCD939X_HPH) & 0x30) == 0x30)
			usleep_range(2500, 2600); /* 2.5msec delay as per HW requirement */
		set_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		if (!wcd939x->hph_pcm_enabled)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PDM_WD_CTL0, PDM_WD_EN, 0x03));
		break;
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * 7ms sleep is required if compander is enabled as per
		 * HW requirement. If compander is disabled, then
		 * 20ms delay is required.
		 */
		if (test_bit(HPH_PA_DELAY, &wcd939x->status_mask)) {
			if (!wcd939x->comp1_enable)
				usleep_range(20000, 20100);
			else
				usleep_range(7000, 7100);
			if (hph_mode == CLS_H_LP ||
			    hph_mode == CLS_H_LOHIFI ||
			    hph_mode == CLS_H_ULP)
				if (!wcd939x->hph_pcm_enabled)
					snd_soc_component_update_bits(component,
						REG_FIELD_VALUE(REFBUFF_LP_CTL, PREREF_FILT_BYPASS, 0x00));
			clear_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		}
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH_TIMER1, AUTOCHOP_TIMER_CTL_EN, 0x01));
		if (hph_mode == CLS_AB || hph_mode == CLS_AB_HIFI ||
			hph_mode == CLS_AB_LP || hph_mode == CLS_AB_LOHIFI)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(RX_SUPPLIES, REGULATOR_MODE, 0x01));
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX1 << 0x10));
		/*Enable PDM INT for PDM data path only*/
		if (!wcd939x->hph_pcm_enabled)
			wcd_enable_irq(&wcd939x->irq_info,
					WCD939X_IRQ_HPHL_PDM_WD_INT);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_RX_MUTE,
						(WCD_RX1 << 0x10 | 0x1));
		wcd_disable_irq(&wcd939x->irq_info,
					WCD939X_IRQ_HPHL_PDM_WD_INT);
		if (wcd939x->update_wcd_event && wcd939x->comp1_enable)
			wcd939x->update_wcd_event(wcd939x->handle,
					SLV_BOLERO_EVT_RX_COMPANDER_SOFT_RST,
					(WCD_RX1 << 0x10));
		/*
		 * 7ms sleep is required if compander is enabled as per
		 * HW requirement. If compander is disabled, then
		 * 20ms delay is required.
		 */
		if (!wcd939x->comp1_enable)
			usleep_range(20000, 20100);
		else
			usleep_range(7000, 7100);
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHL_ENABLE, 0x00));
		blocking_notifier_call_chain(&wcd939x->mbhc->notifier,
					     WCD_EVENT_PRE_HPHL_PA_OFF,
					     &wcd939x->mbhc->wcd_mbhc);
		set_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/*
		 * 7ms sleep is required if compander is enabled as per
		 * HW requirement. If compander is disabled, then
		 * 20ms delay is required.
		 */
		if (test_bit(HPH_PA_DELAY, &wcd939x->status_mask)) {
			if (!wcd939x->comp1_enable)
				usleep_range(21000, 21100);
			else
				usleep_range(7000, 7100);
			clear_bit(HPH_PA_DELAY, &wcd939x->status_mask);
		}
		blocking_notifier_call_chain(&wcd939x->mbhc->notifier,
					     WCD_EVENT_POST_HPHL_PA_OFF,
					     &wcd939x->mbhc->wcd_mbhc);
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHL_REF_ENABLE, 0x00));
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PDM_WD_CTL0, PDM_WD_EN, 0x00));
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
		if (wcd939x->mbhc->wcd_mbhc.mbhc_cfg->enable_usbc_analog &&
			!(snd_soc_component_read(component, WCD939X_HPH) & 0XC0))
			wcd_usbss_audio_config(NULL, WCD_USBSS_CONFIG_TYPE_POWER_MODE, 1);
#endif
		wcd_cls_h_fsm(component, &wcd939x->clsh_info,
			     WCD_CLSH_EVENT_POST_PA,
			     WCD_CLSH_STATE_HPHL,
			     hph_mode);
		if (wcd939x->ldoh)
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(MODE, LDOH_EN, 0x00));

		break;
	};
	return ret;
}

static int wcd939x_codec_enable_ear_pa(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_component *component =
					snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = swr_slvdev_datapath_control(wcd939x->rx_swr_dev,
			    wcd939x->rx_swr_dev->dev_num,
			    true);
		/*
		 * Enable watchdog interrupt for HPHL
		 */
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PDM_WD_CTL0, PDM_WD_EN, 0x03));
		/* For EAR, use CLASS_AB regulator mode */
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(RX_SUPPLIES, REGULATOR_MODE, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(EAR_COMPANDER_CTL, GAIN_OVRD_REG, 0x01));
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* 6 msec delay as per HW requirement */
		usleep_range(6000, 6010);
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
					SLV_BOLERO_EVT_RX_MUTE,
					(WCD_RX3 << 0x10));
		wcd_enable_irq(&wcd939x->irq_info, WCD939X_IRQ_EAR_PDM_WD_INT);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		wcd_disable_irq(&wcd939x->irq_info,
				WCD939X_IRQ_EAR_PDM_WD_INT);
		if (wcd939x->update_wcd_event)
			wcd939x->update_wcd_event(wcd939x->handle,
					SLV_BOLERO_EVT_RX_MUTE,
					(WCD_RX3 << 0x10 | 0x1));
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(EAR_COMPANDER_CTL, GAIN_OVRD_REG, 0x00));
		/* 7 msec delay as per HW requirement */
		usleep_range(7000, 7010);
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(PDM_WD_CTL0, PDM_WD_EN, 0x00));
		wcd_cls_h_fsm(component, &wcd939x->clsh_info,
			     WCD_CLSH_EVENT_POST_PA,
			     WCD_CLSH_STATE_EAR,
			     CLS_AB_HIFI);
		break;
	};
	return ret;
}

static int wcd939x_clsh_dummy(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol,
			       int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	if (SND_SOC_DAPM_EVENT_OFF(event))
		ret = swr_slvdev_datapath_control(
				wcd939x->rx_swr_dev,
				wcd939x->rx_swr_dev->dev_num,
				false);
	return ret;
}

static int wcd939x_enable_clsh(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol,
			       int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int mode = wcd939x->hph_mode;
	int ret = 0;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	if (mode == CLS_H_LOHIFI || mode == CLS_H_ULP ||
		mode == CLS_H_HIFI || mode == CLS_H_LP) {
		wcd939x_rx_connect_port(component, CLSH,
				SND_SOC_DAPM_EVENT_ON(event));
	}
	if (SND_SOC_DAPM_EVENT_OFF(event))
		ret = swr_slvdev_datapath_control(
				wcd939x->rx_swr_dev,
				wcd939x->rx_swr_dev->dev_num,
				false);
	return ret;
}

static int wcd939x_enable_rx1(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol,
			      int event)
{
	struct snd_soc_component *component =
					snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (wcd939x->hph_pcm_enabled)
			wcd939x_rx_connect_port(component, HIFI_PCM_L, true);
		else {
			wcd939x_rx_connect_port(component, HPH_L, true);
			if (wcd939x->comp1_enable)
				wcd939x_rx_connect_port(component, COMP_L, true);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (wcd939x->hph_pcm_enabled)
			wcd939x_rx_connect_port(component, HIFI_PCM_L, false);
		else {
			wcd939x_rx_connect_port(component, HPH_L, false);
			if (wcd939x->comp1_enable)
				wcd939x_rx_connect_port(component, COMP_L, false);
		}
		break;
	};

	return 0;
}

static int wcd939x_enable_rx2(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (wcd939x->hph_pcm_enabled)
			wcd939x_rx_connect_port(component, HIFI_PCM_R, true);
		else {
			wcd939x_rx_connect_port(component, HPH_R, true);
			if (wcd939x->comp2_enable)
				wcd939x_rx_connect_port(component, COMP_R, true);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (wcd939x->hph_pcm_enabled)
			wcd939x_rx_connect_port(component, HIFI_PCM_R, false);
		else {
			wcd939x_rx_connect_port(component, HPH_R, false);
			if (wcd939x->comp2_enable)
				wcd939x_rx_connect_port(component, COMP_R, false);
		}
		break;
	};

	return 0;
}

static int wcd939x_enable_rx3(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol,
			      int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x_rx_connect_port(component, LO, true);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_rx_connect_port(component, LO, false);
		/* 6 msec delay as per HW requirement */
		usleep_range(6000, 6010);
		break;
	}

	return 0;
}

static int wcd939x_codec_enable_dmic(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct snd_soc_component *component =
				snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	u16 dmic_clk_reg, dmic_clk_en_reg;
	s32 *dmic_clk_cnt;
	u8 dmic_ctl_shift = 0;
	u8 dmic_clk_shift = 0;
	u8 dmic_clk_mask = 0;
	u16 dmic2_left_en = 0;
	int ret = 0;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (w->shift) {
	case 0:
	case 1:
		dmic_clk_cnt = &(wcd939x->dmic_0_1_clk_cnt);
		dmic_clk_reg = WCD939X_CDC_DMIC_RATE_1_2;
		dmic_clk_en_reg = WCD939X_CDC_DMIC1_CTL;
		dmic_clk_mask = 0x0F;
		dmic_clk_shift = 0x00;
		dmic_ctl_shift = 0x00;
		break;
	case 2:
		dmic2_left_en = WCD939X_CDC_DMIC2_CTL;
		fallthrough;
	case 3:
		dmic_clk_cnt = &(wcd939x->dmic_2_3_clk_cnt);
		dmic_clk_reg = WCD939X_CDC_DMIC_RATE_1_2;
		dmic_clk_en_reg = WCD939X_CDC_DMIC2_CTL;
		dmic_clk_mask = 0xF0;
		dmic_clk_shift = 0x04;
		dmic_ctl_shift = 0x01;
		break;
	case 4:
	case 5:
		dmic_clk_cnt = &(wcd939x->dmic_4_5_clk_cnt);
		dmic_clk_reg = WCD939X_CDC_DMIC_RATE_3_4;
		dmic_clk_en_reg = WCD939X_CDC_DMIC3_CTL;
		dmic_clk_mask = 0x0F;
		dmic_clk_shift = 0x00;
		dmic_ctl_shift = 0x02;
		break;
	case 6:
	case 7:
		dmic_clk_cnt = &(wcd939x->dmic_6_7_clk_cnt);
		dmic_clk_reg = WCD939X_CDC_DMIC_RATE_3_4;
		dmic_clk_en_reg = WCD939X_CDC_DMIC4_CTL;
		dmic_clk_mask = 0xF0;
		dmic_clk_shift = 0x04;
		dmic_ctl_shift = 0x03;
		break;
	default:
		dev_err_ratelimited(component->dev, "%s: Invalid DMIC Selection\n",
			__func__);
		return -EINVAL;
	};
	dev_dbg(component->dev, "%s: event %d DMIC%d dmic_clk_cnt %d\n",
			__func__, event,  (w->shift +1), *dmic_clk_cnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component,
				WCD939X_CDC_AMIC_CTL,
				(0x01 << dmic_ctl_shift), 0x00);
		/* 250us sleep as per HW requirement */
		usleep_range(250, 260);
		if (dmic2_left_en)
			snd_soc_component_update_bits(component,
				dmic2_left_en, 0x80, 0x80);
		/* Setting DMIC clock rate to 2.4MHz */
		snd_soc_component_update_bits(component,
					      dmic_clk_reg, dmic_clk_mask,
					      (0x03 << dmic_clk_shift));
		snd_soc_component_update_bits(component,
					      dmic_clk_en_reg, 0x08, 0x08);
		/* enable clock scaling */
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DMIC_CTL, CLK_SCALE_EN, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DMIC_CTL, DMIC_DIV_BAK_EN, 0x01));
		ret = swr_slvdev_datapath_control(wcd939x->tx_swr_dev,
				wcd939x->tx_swr_dev->dev_num,
				true);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_tx_connect_port(component, DMIC0 + (w->shift), 0,
				false);
		snd_soc_component_update_bits(component,
				WCD939X_CDC_AMIC_CTL,
				(0x01 << dmic_ctl_shift),
				(0x01 << dmic_ctl_shift));
		if (dmic2_left_en)
			snd_soc_component_update_bits(component,
				dmic2_left_en, 0x80, 0x00);
		snd_soc_component_update_bits(component,
					      dmic_clk_en_reg, 0x08, 0x00);
		break;
	};
	return ret;
}

/*
 * wcd939x_get_micb_vout_ctl_val: converts micbias from volts to register value
 * @micb_mv: micbias in mv
 *
 * return register value converted
 */
int wcd939x_get_micb_vout_ctl_val(u32 micb_mv)
{
	/* min micbias voltage is 1V and maximum is 2.85V */
	if (micb_mv < 1000 || micb_mv > 2850) {
		pr_err_ratelimited("%s: unsupported micbias voltage\n", __func__);
		return -EINVAL;
	}

	return (micb_mv - 1000) / 50;
}
EXPORT_SYMBOL(wcd939x_get_micb_vout_ctl_val);

/*
 * wcd939x_mbhc_micb_adjust_voltage: adjust specific micbias voltage
 * @component: handle to snd_soc_component *
 * @req_volt: micbias voltage to be set
 * @micb_num: micbias to be set, e.g. micbias1 or micbias2
 *
 * return 0 if adjustment is success or error code in case of failure
 */
int wcd939x_mbhc_micb_adjust_voltage(struct snd_soc_component *component,
				   int req_volt, int micb_num)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int cur_vout_ctl, req_vout_ctl;
	int micb_reg, micb_val, micb_en;
	int ret = 0;

	switch (micb_num) {
	case MIC_BIAS_1:
		micb_reg = WCD939X_MICB1;
		break;
	case MIC_BIAS_2:
		micb_reg = WCD939X_MICB2;
		break;
	case MIC_BIAS_3:
		micb_reg = WCD939X_MICB3;
		break;
	case MIC_BIAS_4:
		micb_reg = WCD939X_MICB4;
		break;
	default:
		return -EINVAL;
	}
	mutex_lock(&wcd939x->micb_lock);

	/*
	 * If requested micbias voltage is same as current micbias
	 * voltage, then just return. Otherwise, adjust voltage as
	 * per requested value. If micbias is already enabled, then
	 * to avoid slow micbias ramp-up or down enable pull-up
	 * momentarily, change the micbias value and then re-enable
	 * micbias.
	 */
	micb_val = snd_soc_component_read(component, micb_reg);
	micb_en = (micb_val & 0xC0) >> 6;
	cur_vout_ctl = micb_val & 0x3F;

	req_vout_ctl = wcd939x_get_micb_vout_ctl_val(req_volt);
	if (req_vout_ctl < 0) {
		ret = -EINVAL;
		goto exit;
	}
	if (cur_vout_ctl == req_vout_ctl) {
		ret = 0;
		goto exit;
	}

	dev_dbg(component->dev, "%s: micb_num: %d, cur_mv: %d, req_mv: %d, micb_en: %d\n",
		 __func__, micb_num, WCD_VOUT_CTL_TO_MICB(cur_vout_ctl),
		 req_volt, micb_en);

	if (micb_en == 0x1)
		snd_soc_component_update_bits(component, micb_reg, 0xC0, 0x80);

	snd_soc_component_update_bits(component, micb_reg, 0x3F, req_vout_ctl);

	if (micb_en == 0x1) {
		snd_soc_component_update_bits(component, micb_reg, 0xC0, 0x40);
		/*
		 * Add 2ms delay as per HW requirement after enabling
		 * micbias
		 */
		usleep_range(2000, 2100);
	}
exit:
	mutex_unlock(&wcd939x->micb_lock);
	return ret;
}
EXPORT_SYMBOL(wcd939x_mbhc_micb_adjust_voltage);

static int wcd939x_tx_swr_ctrl(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol,
				    int event)
{
	struct snd_soc_component *component =
					snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int ret = 0;
	int bank = 0;
	u8 mode = 0;
	int i = 0;
	int rate = 0;

	bank = (wcd939x_swr_slv_get_current_bank(wcd939x->tx_swr_dev,
					wcd939x->tx_swr_dev->dev_num) ? 0 : 1);

	/* power mode is applicable only to analog mics */
	if (strnstr(w->name, "ADC", sizeof("ADC"))) {
		/* Get channel rate */
		rate = wcd939x_get_clk_rate(wcd939x->tx_mode[w->shift - ADC1]);
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Check AMIC2 is connected to ADC2 to take an action on BCS */
		if (w->shift == ADC2 &&
			(((snd_soc_component_read(component, WCD939X_TX_CH12_MUX) &
				   0x38) >> 3)  == 0x2)) {
			if (!wcd939x->bcs_dis) {
				wcd939x_tx_connect_port(component, MBHC,
					SWR_CLK_RATE_4P8MHZ, true);
				set_bit(AMIC2_BCS_ENABLE, &wcd939x->status_mask);
			}
		}
		if (strnstr(w->name, "ADC", sizeof("ADC"))) {
			set_bit(w->shift - ADC1, &wcd939x->status_mask);
			wcd939x_tx_connect_port(component, w->shift, rate,
					true);
		} else {
			wcd939x_tx_connect_port(component, w->shift,
					SWR_CLK_RATE_2P4MHZ, true);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (strnstr(w->name, "ADC", sizeof("ADC"))) {
			if (strnstr(w->name, "ADC1", sizeof("ADC1"))) {
				clear_bit(WCD_ADC1, &wcd939x->status_mask);
				clear_bit(WCD_ADC1_MODE, &wcd939x->status_mask);
			} else if (strnstr(w->name, "ADC2", sizeof("ADC2"))) {
				clear_bit(WCD_ADC2, &wcd939x->status_mask);
				clear_bit(WCD_ADC2_MODE, &wcd939x->status_mask);
			} else if (strnstr(w->name, "ADC3", sizeof("ADC3"))) {
				clear_bit(WCD_ADC3, &wcd939x->status_mask);
				clear_bit(WCD_ADC3_MODE, &wcd939x->status_mask);
			} else if (strnstr(w->name, "ADC4", sizeof("ADC4"))) {
				clear_bit(WCD_ADC4, &wcd939x->status_mask);
				clear_bit(WCD_ADC4_MODE, &wcd939x->status_mask);
			}
		}
		if (strnstr(w->name, "ADC", sizeof("ADC"))) {
			if (test_bit(WCD_ADC1, &wcd939x->status_mask) ||
				test_bit(WCD_ADC1_MODE, &wcd939x->status_mask))
				mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC1]];
			if (test_bit(WCD_ADC2, &wcd939x->status_mask) ||
				test_bit(WCD_ADC2_MODE, &wcd939x->status_mask))
				mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC2]];
			if (test_bit(WCD_ADC3, &wcd939x->status_mask) ||
				test_bit(WCD_ADC3_MODE, &wcd939x->status_mask))
				mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC3]];
			if (test_bit(WCD_ADC4, &wcd939x->status_mask) ||
				test_bit(WCD_ADC4_MODE, &wcd939x->status_mask))
				mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC4]];

			if (mode != 0) {
				for (i = 0; i < ADC_MODE_ULP2; i++) {
					if (mode & (1 << i)) {
						i++;
						break;
					}
				}
			}
			rate = wcd939x_get_clk_rate(i);
			if (wcd939x->adc_count) {
				rate =  (wcd939x->adc_count * rate);
				if (rate > SWR_CLK_RATE_9P6MHZ)
					rate = SWR_CLK_RATE_9P6MHZ;
			}
			wcd939x_set_swr_clk_rate(component, rate, bank);
		}
		ret = swr_slvdev_datapath_control(wcd939x->tx_swr_dev,
				wcd939x->tx_swr_dev->dev_num,
				false);

		if (strnstr(w->name, "ADC", sizeof("ADC")))
			wcd939x_set_swr_clk_rate(component, rate, !bank);
		break;
	};

	return ret;
}

static int wcd939x_get_adc_mode(int val)
{
	int ret = 0;

	switch (val) {
	case ADC_MODE_INVALID:
		ret = ADC_MODE_VAL_NORMAL;
		break;
	case ADC_MODE_HIFI:
		ret = ADC_MODE_VAL_HIFI;
		break;
	case ADC_MODE_LO_HIF:
		ret = ADC_MODE_VAL_LO_HIF;
		break;
	case ADC_MODE_NORMAL:
		ret = ADC_MODE_VAL_NORMAL;
		break;
	case ADC_MODE_LP:
		ret = ADC_MODE_VAL_LP;
		break;
	case ADC_MODE_ULP1:
		ret = ADC_MODE_VAL_ULP1;
		break;
	case ADC_MODE_ULP2:
		ret = ADC_MODE_VAL_ULP2;
		break;
	default:
		ret = -EINVAL;
		pr_err_ratelimited("%s: invalid ADC mode value %d\n", __func__, val);
		break;
	}
	return ret;
}

int wcd939x_tx_channel_config(struct snd_soc_component *component,
			      int channel, int mode)
{
	int reg = WCD939X_TX_CH2, mask = 0, val = 0;
	int ret = 0;

	switch (channel) {
	case 0:
		reg = WCD939X_TX_CH2;
		mask = 0x40;
		break;
	case 1:
		reg = WCD939X_TX_CH2;
		mask = 0x20;
		break;
	case 2:
		reg = WCD939X_TX_CH4;
		mask = 0x40;
		break;
	case 3:
		reg = WCD939X_TX_CH4;
		mask = 0x20;
		break;
	default:
		pr_err_ratelimited("%s: Invalid channel num %d\n", __func__, channel);
		ret = -EINVAL;
		break;
	}

	if (!mode)
		val = 0x00;
	else
		val = mask;

	if (!ret)
		snd_soc_component_update_bits(component, reg, mask, val);

	return ret;
}

static int wcd939x_codec_enable_adc(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol,
				    int event){
	struct snd_soc_component *component =
					snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int clk_rate = 0, ret = 0;
	int mode = 0, i = 0, bank = 0;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);
	bank = (wcd939x_swr_slv_get_current_bank(wcd939x->tx_swr_dev,
		wcd939x->tx_swr_dev->dev_num) ? 0 : 1);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x->adc_count++;
		if (test_bit(WCD_ADC1, &wcd939x->status_mask) ||
			test_bit(WCD_ADC1_MODE, &wcd939x->status_mask))
			mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC1]];
		if (test_bit(WCD_ADC2, &wcd939x->status_mask) ||
			test_bit(WCD_ADC2_MODE, &wcd939x->status_mask))
			mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC2]];
		if (test_bit(WCD_ADC3, &wcd939x->status_mask) ||
			test_bit(WCD_ADC3_MODE, &wcd939x->status_mask))
			mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC3]];
		if (test_bit(WCD_ADC4, &wcd939x->status_mask) ||
			test_bit(WCD_ADC4_MODE, &wcd939x->status_mask))
			mode |= tx_mode_bit[wcd939x->tx_mode[WCD_ADC4]];

		if (mode != 0) {
			for (i = 0; i < ADC_MODE_ULP2; i++) {
				if (mode & (1 << i)) {
					i++;
					break;
				}
			}
		}
		clk_rate = wcd939x_get_clk_rate(i);

		/* clk_rate depends on number of paths getting enabled */
		clk_rate =  (wcd939x->adc_count * clk_rate);
		if (clk_rate > SWR_CLK_RATE_9P6MHZ)
			clk_rate = SWR_CLK_RATE_9P6MHZ;
		wcd939x_set_swr_clk_rate(component, clk_rate, bank);
		ret = swr_slvdev_datapath_control(wcd939x->tx_swr_dev,
				 wcd939x->tx_swr_dev->dev_num,
				 true);
		wcd939x_set_swr_clk_rate(component, clk_rate, !bank);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x->adc_count--;
		if (wcd939x->adc_count < 0)
			wcd939x->adc_count = 0;

		wcd939x_tx_connect_port(component, ADC1 + w->shift, 0, false);
		if (w->shift + ADC1 == ADC2 &&
			test_bit(AMIC2_BCS_ENABLE, &wcd939x->status_mask)) {
			wcd939x_tx_connect_port(component, MBHC, 0,
					false);
			clear_bit(AMIC2_BCS_ENABLE, &wcd939x->status_mask);
		}
		break;
	};

	return ret;
}

void wcd939x_disable_bcs_before_slow_insert(struct snd_soc_component *component,
					    bool bcs_disable)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	if (wcd939x->update_wcd_event) {
		if (bcs_disable)
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_BCS_CLK_OFF, 0);
		else
			wcd939x->update_wcd_event(wcd939x->handle,
						SLV_BOLERO_EVT_BCS_CLK_OFF, 1);
	}
}

static int wcd939x_enable_req(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
					snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x =
					snd_soc_component_get_drvdata(component);
	int ret = 0;
	u8 mode = 0;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_TX_CLK_EN, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_TX_DIV2_CLK_EN, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_REQ_CTL, FS_RATE_4P8, 0x01));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_REQ_CTL, NO_NOTCH, 0x00));

		ret = wcd939x_tx_channel_config(component, w->shift, 1);
		mode = wcd939x_get_adc_mode(wcd939x->tx_mode[w->shift]);
		if (mode < 0) {
			dev_info_ratelimited(component->dev,
				 "%s: invalid mode, setting to normal mode\n",
				 __func__);
			mode = ADC_MODE_VAL_NORMAL;
		}
		switch (w->shift) {
		case 0:
			snd_soc_component_update_bits(component,
				WCD939X_CDC_TX_ANA_MODE_0_1, 0x0F,
				mode);
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD0_CLK_EN, 0x01));
			break;
		case 1:
			snd_soc_component_update_bits(component,
				WCD939X_CDC_TX_ANA_MODE_0_1, 0xF0,
				mode << 4);
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD1_CLK_EN, 0x01));
			break;
		case 2:
			snd_soc_component_update_bits(component,
				WCD939X_CDC_TX_ANA_MODE_2_3, 0x0F,
				mode);
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD2_CLK_EN, 0x01));
			break;
		case 3:
			snd_soc_component_update_bits(component,
				WCD939X_CDC_TX_ANA_MODE_2_3, 0xF0,
				mode << 4);
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD3_CLK_EN, 0x01));
			break;
		default:
			break;
		}
		ret |= wcd939x_tx_channel_config(component, w->shift, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		switch (w->shift) {
		case 0:
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_TX_ANA_MODE_0_1, TXD0_MODE, 0x00));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD0_CLK_EN, 0x00));
			break;
		case 1:
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_TX_ANA_MODE_0_1, TXD1_MODE, 0x00));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD1_CLK_EN, 0x00));
			break;
		case 2:
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_TX_ANA_MODE_2_3, TXD2_MODE, 0x00));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD2_CLK_EN, 0x00));
			break;
		case 3:
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_TX_ANA_MODE_2_3, TXD3_MODE, 0x00));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(CDC_DIG_CLK_CTL, TXD3_CLK_EN, 0x00));
			break;
		default:
			break;
		}
		if (wcd939x->adc_count == 0) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_TX_DIV2_CLK_EN, 0x00));
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_TX_CLK_EN, 0x00));
		}
		break;
	};
	return ret;
}

int wcd939x_micbias_control(struct snd_soc_component *component,
				int micb_num, int req, bool is_dapm)
{

	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int micb_index = micb_num - 1;
	u16 micb_reg;
	int pre_off_event = 0, post_off_event = 0;
	int post_on_event = 0, post_dapm_off = 0;
	int post_dapm_on = 0;
	int ret = 0;

	if ((micb_index < 0) || (micb_index > WCD939X_MAX_MICBIAS - 1)) {
		dev_err_ratelimited(component->dev,
			"%s: Invalid micbias index, micb_ind:%d\n",
			__func__, micb_index);
		return -EINVAL;
	}

	if (NULL == wcd939x) {
		dev_err_ratelimited(component->dev,
			"%s: wcd939x private data is NULL\n", __func__);
		return -EINVAL;
	}

	switch (micb_num) {
	case MIC_BIAS_1:
		micb_reg = WCD939X_MICB1;
		break;
	case MIC_BIAS_2:
		micb_reg = WCD939X_MICB2;
		pre_off_event = WCD_EVENT_PRE_MICBIAS_2_OFF;
		post_off_event = WCD_EVENT_POST_MICBIAS_2_OFF;
		post_on_event = WCD_EVENT_POST_MICBIAS_2_ON;
		post_dapm_on = WCD_EVENT_POST_DAPM_MICBIAS_2_ON;
		post_dapm_off = WCD_EVENT_POST_DAPM_MICBIAS_2_OFF;
		break;
	case MIC_BIAS_3:
		micb_reg = WCD939X_MICB3;
		break;
	case MIC_BIAS_4:
		micb_reg = WCD939X_MICB4;
		break;
	default:
		dev_err_ratelimited(component->dev, "%s: Invalid micbias number: %d\n",
			__func__, micb_num);
		return -EINVAL;
	};
	mutex_lock(&wcd939x->micb_lock);

	switch (req) {
	case MICB_PULLUP_ENABLE:
		if (!wcd939x->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
				__func__, req);
			ret = -ENODEV;
			goto done;
		}
		wcd939x->pullup_ref[micb_index]++;
		if ((wcd939x->pullup_ref[micb_index] == 1) &&
		    (wcd939x->micb_ref[micb_index] == 0))
			snd_soc_component_update_bits(component, micb_reg,
							0xC0, 0x80);
		break;
	case MICB_PULLUP_DISABLE:
		if (wcd939x->pullup_ref[micb_index] > 0)
			wcd939x->pullup_ref[micb_index]--;
		if (!wcd939x->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
				__func__, req);
			ret = -ENODEV;
			goto done;
		}
		if ((wcd939x->pullup_ref[micb_index] == 0) &&
		    (wcd939x->micb_ref[micb_index] == 0))
			snd_soc_component_update_bits(component, micb_reg,
							0xC0, 0x00);
		break;
	case MICB_ENABLE:
		if (!wcd939x->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
				__func__, req);
			ret = -ENODEV;
			goto done;
		}
		wcd939x->micb_ref[micb_index]++;
		if (wcd939x->micb_ref[micb_index] == 1) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_DIG_CLK_CTL,TXD3_CLK_EN, 0x01));
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_DIG_CLK_CTL,TXD2_CLK_EN, 0x01));
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_DIG_CLK_CTL,TXD1_CLK_EN, 0x01));
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_DIG_CLK_CTL,TXD0_CLK_EN, 0x01));
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_ANA_CLK_CTL, ANA_TX_DIV2_CLK_EN, 0x01));
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(CDC_ANA_TX_CLK_CTL, ANA_TXSCBIAS_CLK_EN, 0x01));
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(TEST_CTL_2, IBIAS_LDO_DRIVER, 0x01));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(MICB2_TEST_CTL_2, IBIAS_LDO_DRIVER, 0x01));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(MICB3_TEST_CTL_2, IBIAS_LDO_DRIVER, 0x01));
			snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(MICB4_TEST_CTL_2, IBIAS_LDO_DRIVER, 0x01));
			snd_soc_component_update_bits(component,
				micb_reg, 0xC0, 0x40);
			if (post_on_event)
				blocking_notifier_call_chain(
						&wcd939x->mbhc->notifier,
						post_on_event,
						&wcd939x->mbhc->wcd_mbhc);
		}
		if (is_dapm && post_dapm_on && wcd939x->mbhc)
			blocking_notifier_call_chain(&wcd939x->mbhc->notifier,
						     post_dapm_on,
						     &wcd939x->mbhc->wcd_mbhc);
		break;
	case MICB_DISABLE:
		if (wcd939x->micb_ref[micb_index] > 0)
			wcd939x->micb_ref[micb_index]--;
		if (!wcd939x->dev_up) {
			dev_dbg(component->dev, "%s: enable req %d wcd device down\n",
				__func__, req);
			ret = -ENODEV;
			goto done;
		}
		if ((wcd939x->micb_ref[micb_index] == 0) &&
		    (wcd939x->pullup_ref[micb_index] > 0))
			snd_soc_component_update_bits(component, micb_reg,
							0xC0, 0x80);
		else if ((wcd939x->micb_ref[micb_index] == 0) &&
			 (wcd939x->pullup_ref[micb_index] == 0)) {
			if (pre_off_event && wcd939x->mbhc)
				blocking_notifier_call_chain(
						&wcd939x->mbhc->notifier,
						pre_off_event,
						&wcd939x->mbhc->wcd_mbhc);
			snd_soc_component_update_bits(component, micb_reg,
							0xC0, 0x00);
			if (post_off_event && wcd939x->mbhc)
				blocking_notifier_call_chain(
						&wcd939x->mbhc->notifier,
						post_off_event,
						&wcd939x->mbhc->wcd_mbhc);
		}
		if (is_dapm && post_dapm_off && wcd939x->mbhc)
			blocking_notifier_call_chain(&wcd939x->mbhc->notifier,
						post_dapm_off,
						&wcd939x->mbhc->wcd_mbhc);
		break;
	};

	dev_dbg(component->dev,
		"%s: micb_num:%d, micb_ref: %d, pullup_ref: %d\n",
		__func__, micb_num, wcd939x->micb_ref[micb_index],
		wcd939x->pullup_ref[micb_index]);

done:
	mutex_unlock(&wcd939x->micb_lock);
	return ret;
}
EXPORT_SYMBOL(wcd939x_micbias_control);

static int wcd939x_get_logical_addr(struct swr_device *swr_dev)
{
	int ret = 0;
	uint8_t devnum = 0;
	int num_retry = NUM_ATTEMPTS;

	do {
		/* retry after 1ms */
		usleep_range(1000, 1010);
		ret = swr_get_logical_dev_num(swr_dev, swr_dev->addr, &devnum);
	} while (ret && --num_retry);

	if (ret)
		dev_err_ratelimited(&swr_dev->dev,
			"%s get devnum %d for dev addr %llx failed\n",
			__func__, devnum, swr_dev->addr);

	swr_dev->dev_num = devnum;
	return 0;
}

static bool get_usbc_hs_status(struct snd_soc_component *component,
			struct wcd_mbhc_config *mbhc_cfg)
{
	if (mbhc_cfg->enable_usbc_analog) {
		if (!(snd_soc_component_read(component, WCD939X_MBHC_MECH)
			& 0x20))
			return true;
	}
	return false;
}

int wcd939x_swr_dmic_register_notifier(struct snd_soc_component *component,
					struct notifier_block *nblock,
					bool enable)
{
	struct wcd939x_priv *wcd939x_priv;
	if(NULL == component) {
		pr_err_ratelimited("%s: wcd939x component is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x_priv = snd_soc_component_get_drvdata(component);
	wcd939x_priv->notify_swr_dmic = enable;
	if (enable)
		return blocking_notifier_chain_register(&wcd939x_priv->notifier,
							nblock);
	else
		return blocking_notifier_chain_unregister(
				&wcd939x_priv->notifier, nblock);
}
EXPORT_SYMBOL(wcd939x_swr_dmic_register_notifier);

static int wcd939x_event_notify(struct notifier_block *block,
				unsigned long val,
				void *data)
{
	u16 event = (val & 0xffff);
	int ret = 0;
	int rx_clk_type;
	struct wcd939x_priv *wcd939x = dev_get_drvdata((struct device *)data);
	struct snd_soc_component *component = wcd939x->component;
	struct wcd_mbhc *mbhc;

	switch (event) {
	case BOLERO_SLV_EVT_TX_CH_HOLD_CLEAR:
		if (test_bit(WCD_ADC1, &wcd939x->status_mask)) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(TX_CH2, HPF1_INIT, 0x00));
			set_bit(WCD_ADC1_MODE, &wcd939x->status_mask);
			clear_bit(WCD_ADC1, &wcd939x->status_mask);
		}
		if (test_bit(WCD_ADC2, &wcd939x->status_mask)) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(TX_CH2, HPF2_INIT, 0x00));
			set_bit(WCD_ADC2_MODE, &wcd939x->status_mask);
			clear_bit(WCD_ADC2, &wcd939x->status_mask);
		}
		if (test_bit(WCD_ADC3, &wcd939x->status_mask)) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(TX_CH4, HPF3_INIT, 0x00));
			set_bit(WCD_ADC3_MODE, &wcd939x->status_mask);
			clear_bit(WCD_ADC3, &wcd939x->status_mask);
		}
		if (test_bit(WCD_ADC4, &wcd939x->status_mask)) {
			snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(TX_CH4, HPF4_INIT, 0x00));
			set_bit(WCD_ADC4_MODE, &wcd939x->status_mask);
			clear_bit(WCD_ADC4, &wcd939x->status_mask);
		}
		break;
	case BOLERO_SLV_EVT_PA_OFF_PRE_SSR:
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHL_ENABLE, 0x00));
		snd_soc_component_update_bits(component,
					REG_FIELD_VALUE(HPH, HPHR_ENABLE , 0x00));
		snd_soc_component_update_bits(component,
				REG_FIELD_VALUE(EAR, ENABLE, 0x00));
		break;
	case BOLERO_SLV_EVT_SSR_DOWN:
		wcd939x->dev_up = false;
		if(wcd939x->notify_swr_dmic)
			blocking_notifier_call_chain(&wcd939x->notifier,
						     WCD939X_EVT_SSR_DOWN,
						     NULL);
		wcd939x->mbhc->wcd_mbhc.deinit_in_progress = true;
		mbhc = &wcd939x->mbhc->wcd_mbhc;
		wcd939x->usbc_hs_status = get_usbc_hs_status(component,
						mbhc->mbhc_cfg);
		wcd939x_mbhc_ssr_down(wcd939x->mbhc, component);
		wcd939x_reset_low(wcd939x->dev);
		break;
	case BOLERO_SLV_EVT_SSR_UP:
		wcd939x_reset(wcd939x->dev);
		/* allow reset to take effect */
		usleep_range(10000, 10010);

		wcd939x_get_logical_addr(wcd939x->tx_swr_dev);
		wcd939x_get_logical_addr(wcd939x->rx_swr_dev);

		wcd939x_init_reg(component);
		regcache_mark_dirty(wcd939x->regmap);
		regcache_sync(wcd939x->regmap);
		/* Initialize MBHC module */
		mbhc = &wcd939x->mbhc->wcd_mbhc;
		ret = wcd939x_mbhc_post_ssr_init(wcd939x->mbhc, component);
		if (ret) {
			dev_err_ratelimited(component->dev, "%s: mbhc initialization failed\n",
				__func__);
		} else {
			wcd939x_mbhc_hs_detect(component, mbhc->mbhc_cfg);
		}
		wcd939x->mbhc->wcd_mbhc.deinit_in_progress = false;
		wcd939x->dev_up = true;
		if(wcd939x->notify_swr_dmic)
			blocking_notifier_call_chain(&wcd939x->notifier,
						     WCD939X_EVT_SSR_UP,
						     NULL);
		if (wcd939x->usbc_hs_status)
			mdelay(500);
		break;
	case BOLERO_SLV_EVT_CLK_NOTIFY:
		snd_soc_component_update_bits(component,
				WCD939X_TOP_CLK_CFG, 0x06,
				((val >> 0x10) << 0x01));

		rx_clk_type = (val >> 0x10);

		switch(rx_clk_type) {
		case RX_CLK_12P288MHZ:
			wcd939x->rx_clk_config = RX_CLK_12P288MHZ;
			break;
		case RX_CLK_11P2896MHZ:
			wcd939x->rx_clk_config = RX_CLK_11P2896MHZ;
			break;
		default:
			wcd939x->rx_clk_config = RX_CLK_9P6MHZ;
			break;
		}
		dev_dbg(component->dev, "%s: rx clk config %d\n", __func__, wcd939x->rx_clk_config);
		break;
	default:
		dev_dbg(component->dev, "%s: invalid event %d\n", __func__, event);
		break;
	}
	return 0;
}

static int __wcd939x_codec_enable_micbias(struct snd_soc_dapm_widget *w,
					  int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	int micb_num;

	dev_dbg(component->dev, "%s: wname: %s, event: %d\n",
		__func__, w->name, event);

	if (strnstr(w->name, "MIC BIAS1", sizeof("MIC BIAS1")))
		micb_num = MIC_BIAS_1;
	else if (strnstr(w->name, "MIC BIAS2", sizeof("MIC BIAS2")))
		micb_num = MIC_BIAS_2;
	else if (strnstr(w->name, "MIC BIAS3", sizeof("MIC BIAS3")))
		micb_num = MIC_BIAS_3;
	else if (strnstr(w->name, "MIC BIAS4", sizeof("MIC BIAS4")))
		micb_num = MIC_BIAS_4;
	else
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x_micbias_control(component, micb_num,
						MICB_ENABLE, true);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* 1 msec delay as per HW requirement */
		usleep_range(1000, 1100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_micbias_control(component, micb_num,
						MICB_DISABLE, true);
		break;
	};

	return 0;

}

static int wcd939x_codec_enable_micbias(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	return __wcd939x_codec_enable_micbias(w, event);
}

static int __wcd939x_codec_enable_micbias_pullup(struct snd_soc_dapm_widget *w,
						 int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	int micb_num;

	dev_dbg(component->dev, "%s: wname: %s, event: %d\n",
		__func__, w->name, event);

	if (strnstr(w->name, "VA MIC BIAS1", sizeof("VA MIC BIAS1")))
		micb_num = MIC_BIAS_1;
	else if (strnstr(w->name, "VA MIC BIAS2", sizeof("VA MIC BIAS2")))
		micb_num = MIC_BIAS_2;
	else if (strnstr(w->name, "VA MIC BIAS3", sizeof("VA MIC BIAS3")))
		micb_num = MIC_BIAS_3;
	else if (strnstr(w->name, "VA MIC BIAS4", sizeof("VA MIC BIAS4")))
		micb_num = MIC_BIAS_4;
	else
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x_micbias_control(component, micb_num,
					MICB_PULLUP_ENABLE, true);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* 1 msec delay as per HW requirement */
		usleep_range(1000, 1100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_micbias_control(component, micb_num,
					MICB_PULLUP_DISABLE, true);
		break;
	};

	return 0;

}

static int wcd939x_codec_enable_micbias_pullup(struct snd_soc_dapm_widget *w,
					       struct snd_kcontrol *kcontrol,
					       int event)
{
	return __wcd939x_codec_enable_micbias_pullup(w, event);
}

static int wcd939x_wakeup(void *handle, bool enable)
{
	struct wcd939x_priv *priv;
	int ret = 0;

	if (!handle) {
		pr_err_ratelimited("%s: NULL handle\n", __func__);
		return -EINVAL;
	}
	priv = (struct wcd939x_priv *)handle;
	if (!priv->tx_swr_dev) {
		pr_err_ratelimited("%s: tx swr dev is NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&priv->wakeup_lock);
	if (enable)
		ret = swr_device_wakeup_vote(priv->tx_swr_dev);
	else
		ret = swr_device_wakeup_unvote(priv->tx_swr_dev);
	mutex_unlock(&priv->wakeup_lock);

	return ret;
}

static int wcd939x_codec_force_enable_micbias(struct snd_soc_dapm_widget *w,
					    struct snd_kcontrol *kcontrol,
					    int event)
{
	int ret = 0;
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x_wakeup(wcd939x, true);
		ret = __wcd939x_codec_enable_micbias(w, SND_SOC_DAPM_PRE_PMU);
		wcd939x_wakeup(wcd939x, false);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_wakeup(wcd939x, true);
		ret = __wcd939x_codec_enable_micbias(w, SND_SOC_DAPM_POST_PMD);
		wcd939x_wakeup(wcd939x, false);
		break;
	}

	return ret;
}

static int wcd939x_enable_micbias(struct wcd939x_priv *wcd939x,
					int micb_num, int req)
{
	int micb_index = micb_num - 1;
	u16 micb_reg;

	if (NULL == wcd939x) {
		pr_err_ratelimited("%s: wcd939x private data is NULL\n", __func__);
		return -EINVAL;
	}

	switch (micb_num) {
	case MIC_BIAS_1:
		micb_reg = WCD939X_MICB1;
		break;
	case MIC_BIAS_2:
		micb_reg = WCD939X_MICB2;
		break;
	case MIC_BIAS_3:
		micb_reg = WCD939X_MICB3;
		break;
	case MIC_BIAS_4:
		micb_reg = WCD939X_MICB4;
		break;
	default:
		pr_err_ratelimited("%s: Invalid micbias number: %d\n", __func__, micb_num);
		return -EINVAL;
	};

	pr_debug("%s: req: %d micb_num: %d  micb_ref: %d pullup_ref: %d\n",
		__func__, req, micb_num, wcd939x->micb_ref[micb_index],
		wcd939x->pullup_ref[micb_index]);
	mutex_lock(&wcd939x->micb_lock);

	switch (req) {
	case MICB_ENABLE:
		wcd939x->micb_ref[micb_index]++;
		if (wcd939x->micb_ref[micb_index] == 1) {
			regmap_update_bits(wcd939x->regmap,
				WCD939X_CDC_DIG_CLK_CTL, 0xE0, 0xE0);
			regmap_update_bits(wcd939x->regmap,
				WCD939X_CDC_ANA_CLK_CTL, 0x10, 0x10);
			regmap_update_bits(wcd939x->regmap,
			       WCD939X_CDC_ANA_TX_CLK_CTL, 0x01, 0x01);
			regmap_update_bits(wcd939x->regmap,
				WCD939X_TEST_CTL_2, 0x01, 0x01);
			regmap_update_bits(wcd939x->regmap,
				WCD939X_MICB2_TEST_CTL_2, 0x01, 0x01);
			regmap_update_bits(wcd939x->regmap,
				WCD939X_MICB3_TEST_CTL_2, 0x01, 0x01);
			regmap_update_bits(wcd939x->regmap,
				WCD939X_MICB4_TEST_CTL_2, 0x01, 0x01);
			regmap_update_bits(wcd939x->regmap,
				micb_reg, 0xC0, 0x40);
			regmap_update_bits(wcd939x->regmap, micb_reg, 0x3F, 0x10);
		}
		break;
	case MICB_PULLUP_ENABLE:
		wcd939x->pullup_ref[micb_index]++;
		if ((wcd939x->pullup_ref[micb_index] == 1) &&
		    (wcd939x->micb_ref[micb_index] == 0))
			regmap_update_bits(wcd939x->regmap, micb_reg,
							0xC0, 0x80);
		break;
	case MICB_PULLUP_DISABLE:
		if (wcd939x->pullup_ref[micb_index] > 0)
			wcd939x->pullup_ref[micb_index]--;

		if ((wcd939x->pullup_ref[micb_index] == 0) &&
		    (wcd939x->micb_ref[micb_index] == 0))
			regmap_update_bits(wcd939x->regmap, micb_reg,
							0xC0, 0x00);
		break;
	case MICB_DISABLE:
		if (wcd939x->micb_ref[micb_index] > 0)
			wcd939x->micb_ref[micb_index]--;

		if ((wcd939x->micb_ref[micb_index] == 0) &&
		    (wcd939x->pullup_ref[micb_index] > 0))
			regmap_update_bits(wcd939x->regmap, micb_reg,
							0xC0, 0x80);
		else if ((wcd939x->micb_ref[micb_index] == 0) &&
			 (wcd939x->pullup_ref[micb_index] == 0))
			regmap_update_bits(wcd939x->regmap, micb_reg,
							0xC0, 0x00);
		break;
	};

	mutex_unlock(&wcd939x->micb_lock);
	return 0;
}

int wcd939x_codec_force_enable_micbias_v2(struct snd_soc_component *component,
					int event, int micb_num)
{
	struct wcd939x_priv *wcd939x_priv = NULL;
	int ret = 0;
	int micb_index = micb_num - 1;

	if(NULL == component) {
		pr_err_ratelimited("%s: wcd939x component is NULL\n", __func__);
		return -EINVAL;
	}
	if(event != SND_SOC_DAPM_PRE_PMU && event != SND_SOC_DAPM_POST_PMD) {
		pr_err_ratelimited("%s: invalid event: %d\n", __func__, event);
		return -EINVAL;
	}
	if(micb_num < MIC_BIAS_1 || micb_num > MIC_BIAS_4) {
		pr_err_ratelimited("%s: invalid mic bias num: %d\n", __func__, micb_num);
		return -EINVAL;
	}

	wcd939x_priv = snd_soc_component_get_drvdata(component);

	if (!wcd939x_priv->dev_up) {
		if ((wcd939x_priv->pullup_ref[micb_index] > 0) &&
			(event == SND_SOC_DAPM_POST_PMD)) {
			wcd939x_priv->pullup_ref[micb_index]--;
			ret = -ENODEV;
			goto done;
		}
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd939x_wakeup(wcd939x_priv, true);
		wcd939x_enable_micbias(wcd939x_priv, micb_num, MICB_PULLUP_ENABLE);
		wcd939x_wakeup(wcd939x_priv, false);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd939x_wakeup(wcd939x_priv, true);
		wcd939x_enable_micbias(wcd939x_priv, micb_num, MICB_PULLUP_DISABLE);
		wcd939x_wakeup(wcd939x_priv, false);
		break;
	}

done:
	return ret;
}
EXPORT_SYMBOL(wcd939x_codec_force_enable_micbias_v2);

static inline int wcd939x_tx_path_get(const char *wname,
				      unsigned int *path_num)
{
	int ret = 0;
	char *widget_name = NULL;
	char *w_name = NULL;
	char *path_num_char = NULL;
	char *path_name = NULL;

	widget_name = kstrndup(wname, 9, GFP_KERNEL);
	if (!widget_name)
		return -EINVAL;

	w_name = widget_name;

	path_name = strsep(&widget_name, " ");
	if (!path_name) {
		pr_err_ratelimited("%s: Invalid widget name = %s\n",
			__func__, widget_name);
		ret = -EINVAL;
		goto err;
	}
	path_num_char = strpbrk(path_name, "0123");
	if (!path_num_char) {
		pr_err_ratelimited("%s: tx path index not found\n",
			__func__);
		ret = -EINVAL;
		goto err;
	}
	ret = kstrtouint(path_num_char, 10, path_num);
	if (ret < 0)
		pr_err_ratelimited("%s: Invalid tx path = %s\n",
			__func__, w_name);

err:
	kfree(w_name);
	return ret;
}

static int wcd939x_tx_mode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = NULL;
	int ret = 0;
	unsigned int path = 0;

	if (!component)
		return -EINVAL;

	wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x)
		return -EINVAL;

	ret = wcd939x_tx_path_get(kcontrol->id.name, &path);
	if (ret < 0)
		return ret;

	ucontrol->value.integer.value[0] = wcd939x->tx_mode[path];

	return 0;
}

static int wcd939x_tx_mode_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = NULL;
	u32 mode_val;
	unsigned int path = 0;
	int ret = 0;

	if (!component)
		return -EINVAL;

	wcd939x  = snd_soc_component_get_drvdata(component);

	if (!wcd939x)
		return -EINVAL;

	ret = wcd939x_tx_path_get(kcontrol->id.name, &path);
	if (ret)
		return ret;

	mode_val = ucontrol->value.enumerated.item[0];

	dev_dbg(component->dev, "%s: mode: %d\n", __func__, mode_val);

	wcd939x->tx_mode[path] = mode_val;

	return 0;
}

static int wcd939x_rx_hph_mode_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wcd939x->hph_mode;
	return 0;
}

static int wcd939x_rx_hph_mode_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	u32 mode_val;

	mode_val = ucontrol->value.enumerated.item[0];

	dev_dbg(component->dev, "%s: mode: %d\n", __func__, mode_val);

	if (wcd939x->variant == WCD9390) {
		if (mode_val == CLS_H_HIFI || mode_val == CLS_AB_HIFI) {
			dev_info_ratelimited(component->dev,
				"%s:Invalid HPH Mode, default to CLS_H_ULP\n",
				__func__);
			mode_val = CLS_H_ULP;
		}
	}
	if (mode_val == CLS_H_NORMAL) {
		dev_info_ratelimited(component->dev,
			"%s:Invalid HPH Mode, default to class_AB\n",
			__func__);
		mode_val = CLS_H_ULP;
	}
	wcd939x->hph_mode = mode_val;

	return 0;
}

static int wcd939x_ear_pa_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain = 0;
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);

	ear_pa_gain = snd_soc_component_read(component,
				WCD939X_EAR_COMPANDER_CTL);

	ear_pa_gain = (ear_pa_gain & 0x7C) >> 2;

	ucontrol->value.integer.value[0] = ear_pa_gain;

	dev_dbg(component->dev, "%s: ear_pa_gain = 0x%x\n", __func__,
		ear_pa_gain);

	return 0;
}

static int wcd939x_ear_pa_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain = 0;
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);

	dev_dbg(component->dev, "%s: ucontrol->value.integer.value[0]  = %ld\n",
			__func__, ucontrol->value.integer.value[0]);

	ear_pa_gain =  ucontrol->value.integer.value[0] << 2;

	snd_soc_component_update_bits(component,
			WCD939X_EAR_COMPANDER_CTL,
			0x7C, ear_pa_gain);

	return 0;
}

/* wcd939x_codec_get_dev_num - returns swr device number
 * @component: Codec instance
 *
 * Return: swr device number on success or negative error
 * code on failure.
 */
int wcd939x_codec_get_dev_num(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x;

	if (!component)
		return -EINVAL;

	wcd939x = snd_soc_component_get_drvdata(component);
	if (!wcd939x || !wcd939x->rx_swr_dev) {
		pr_err_ratelimited("%s: wcd939x component is NULL\n", __func__);
		return -EINVAL;
	}

	return wcd939x->rx_swr_dev->dev_num;
}
EXPORT_SYMBOL(wcd939x_codec_get_dev_num);

static int wcd939x_get_compander(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	bool hphr;
	struct soc_multi_mixer_control *mc;

	mc = (struct soc_multi_mixer_control *)(kcontrol->private_value);
	hphr = mc->shift;

	ucontrol->value.integer.value[0] = hphr ? wcd939x->comp2_enable :
						wcd939x->comp1_enable;
	return 0;
}

static int wcd939x_set_compander(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];
	bool hphr;
	struct soc_multi_mixer_control *mc;

	mc = (struct soc_multi_mixer_control *)(kcontrol->private_value);
	hphr = mc->shift;
	if (hphr)
		wcd939x->comp2_enable = value;
	else
		wcd939x->comp1_enable = value;

	return 0;
}

static int wcd939x_codec_enable_vdd_buck(struct snd_soc_dapm_widget *w,
					 struct snd_kcontrol *kcontrol,
					 int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	struct wcd939x_pdata *pdata = NULL;
	int ret = 0;

	pdata = dev_get_platdata(wcd939x->dev);

	if (!pdata) {
		dev_err_ratelimited(component->dev, "%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	if (!msm_cdc_is_ondemand_supply(wcd939x->dev,
					wcd939x->supplies,
					pdata->regulator,
					pdata->num_supplies,
					"cdc-vdd-buck"))
		return 0;

	dev_dbg(component->dev, "%s wname: %s event: %d\n", __func__,
		w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (test_bit(ALLOW_BUCK_DISABLE, &wcd939x->status_mask)) {
			dev_dbg(component->dev,
				"%s: buck already in enabled state\n",
				__func__);
			clear_bit(ALLOW_BUCK_DISABLE, &wcd939x->status_mask);
			return 0;
		}
		ret = msm_cdc_enable_ondemand_supply(wcd939x->dev,
						wcd939x->supplies,
						pdata->regulator,
						pdata->num_supplies,
						"cdc-vdd-buck");
		if (ret == -EINVAL) {
			dev_err_ratelimited(component->dev, "%s: vdd buck is not enabled\n",
				__func__);
			return ret;
		}
		clear_bit(ALLOW_BUCK_DISABLE, &wcd939x->status_mask);
		/*
		 * 200us sleep is required after LDO is enabled as per
		 * HW requirement
		 */
		usleep_range(200, 250);
		break;
	case SND_SOC_DAPM_POST_PMD:
		set_bit(ALLOW_BUCK_DISABLE, &wcd939x->status_mask);
		break;
	}
	return 0;
}


static int wcd939x_ldoh_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wcd939x->ldoh;

	return 0;
}

static int wcd939x_ldoh_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	wcd939x->ldoh = ucontrol->value.integer.value[0];

	return 0;
}

const char * const tx_master_ch_text[] = {
	"ZERO", "SWRM_PCM_OUT", "SWRM_TX1_CH1", "SWRM_TX1_CH2", "SWRM_TX1_CH3",
	"SWRM_TX1_CH4", "SWRM_TX2_CH1", "SWRM_TX2_CH2", "SWRM_TX2_CH3",
	"SWRM_TX2_CH4", "SWRM_TX3_CH1", "SWRM_TX3_CH2", "SWRM_TX3_CH3",
	"SWRM_TX3_CH4", "SWRM_PCM_IN",
};

const struct soc_enum tx_master_ch_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tx_master_ch_text),
					tx_master_ch_text);

static void wcd939x_tx_get_slave_ch_type_idx(const char *wname, int *ch_idx)
{
	u8 ch_type = 0;

	if (strnstr(wname, "ADC1", sizeof("ADC1")))
		ch_type = ADC1;
	else if (strnstr(wname, "ADC2", sizeof("ADC2")))
		ch_type = ADC2;
	else if (strnstr(wname, "ADC3", sizeof("ADC3")))
		ch_type = ADC3;
	else if (strnstr(wname, "ADC4", sizeof("ADC4")))
		ch_type = ADC4;
	else if (strnstr(wname, "DMIC0", sizeof("DMIC0")))
		ch_type = DMIC0;
	else if (strnstr(wname, "DMIC1", sizeof("DMIC1")))
		ch_type = DMIC1;
	else if (strnstr(wname, "MBHC", sizeof("MBHC")))
		ch_type = MBHC;
	else if (strnstr(wname, "DMIC2", sizeof("DMIC2")))
		ch_type = DMIC2;
	else if (strnstr(wname, "DMIC3", sizeof("DMIC3")))
		ch_type = DMIC3;
	else if (strnstr(wname, "DMIC4", sizeof("DMIC4")))
		ch_type = DMIC4;
	else if (strnstr(wname, "DMIC5", sizeof("DMIC5")))
		ch_type = DMIC5;
	else if (strnstr(wname, "DMIC6", sizeof("DMIC6")))
		ch_type = DMIC6;
	else if (strnstr(wname, "DMIC7", sizeof("DMIC7")))
		ch_type = DMIC7;
	else
		pr_err_ratelimited("%s: port name: %s is not listed\n", __func__, wname);

	if (ch_type)
		*ch_idx = wcd939x_slave_get_slave_ch_val(ch_type);
	else
		*ch_idx = -EINVAL;
}

static int wcd939x_tx_master_ch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = NULL;
	int slave_ch_idx = -EINVAL;

	if (component == NULL)
		return -EINVAL;

	wcd939x = snd_soc_component_get_drvdata(component);
	if (wcd939x == NULL)
		return -EINVAL;

	wcd939x_tx_get_slave_ch_type_idx(kcontrol->id.name, &slave_ch_idx);
	if (slave_ch_idx < 0 || slave_ch_idx >= WCD939X_MAX_SLAVE_CH_TYPES)
		return -EINVAL;

	ucontrol->value.integer.value[0] = wcd939x_slave_get_master_ch_val(
			wcd939x->tx_master_ch_map[slave_ch_idx]);

	return 0;
}

static int wcd939x_tx_master_ch_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = NULL;
	int slave_ch_idx = -EINVAL, idx = 0;

	if (component == NULL)
		return -EINVAL;

	wcd939x = snd_soc_component_get_drvdata(component);
	if (wcd939x == NULL)
		return -EINVAL;

	wcd939x_tx_get_slave_ch_type_idx(kcontrol->id.name, &slave_ch_idx);

	if (slave_ch_idx < 0 || slave_ch_idx >= WCD939X_MAX_SLAVE_CH_TYPES)
		return -EINVAL;

	dev_dbg(component->dev, "%s: slave_ch_idx: %d", __func__, slave_ch_idx);
	dev_dbg(component->dev, "%s: ucontrol->value.enumerated.item[0] = %ld\n",
			__func__, ucontrol->value.enumerated.item[0]);

	idx = ucontrol->value.enumerated.item[0];
	if (idx < 0 || idx >= ARRAY_SIZE(swr_master_ch_map))
		return -EINVAL;

	wcd939x->tx_master_ch_map[slave_ch_idx] = wcd939x_slave_get_master_ch(idx);
	return 0;
}

static int wcd939x_bcs_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wcd939x->bcs_dis;

	return 0;
}

static int wcd939x_bcs_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	wcd939x->bcs_dis = ucontrol->value.integer.value[0];

	return 0;
}

static const char * const tx_mode_mux_text_wcd9390[] = {
	"ADC_INVALID", "ADC_HIFI", "ADC_LO_HIF", "ADC_NORMAL", "ADC_LP",
};

static const struct soc_enum tx_mode_mux_enum_wcd9390 =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tx_mode_mux_text_wcd9390),
			    tx_mode_mux_text_wcd9390);

static const char * const tx_mode_mux_text[] = {
	"ADC_INVALID", "ADC_HIFI", "ADC_LO_HIF", "ADC_NORMAL", "ADC_LP",
	"ADC_ULP1", "ADC_ULP2",
};

static const struct soc_enum tx_mode_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tx_mode_mux_text),
			    tx_mode_mux_text);

static const char * const rx_hph_mode_mux_text_wcd9390[] = {
	"CLS_H_INVALID", "CLS_H_INVALID_1", "CLS_H_LP", "CLS_AB",
	"CLS_H_LOHIFI", "CLS_H_ULP", "CLS_H_INVALID_2", "CLS_AB_LP",
	"CLS_AB_LOHIFI",
};

static const char * const wcd939x_ear_pa_gain_text[] = {
	"G_6_DB", "G_4P5_DB", "G_3_DB", "G_1P5_DB", "G_0_DB",
	"G_M1P5_DB", "G_M3_DB", "G_M4P5_DB",
	"G_M6_DB", "G_7P5_DB", "G_M9_DB",
	"G_M10P5_DB", "G_M12_DB", "G_M13P5_DB",
	"G_M15_DB", "G_M16P5_DB", "G_M18_DB",
};

static const struct soc_enum rx_hph_mode_mux_enum_wcd9390 =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rx_hph_mode_mux_text_wcd9390),
			    rx_hph_mode_mux_text_wcd9390);

static SOC_ENUM_SINGLE_EXT_DECL(wcd939x_ear_pa_gain_enum,
				wcd939x_ear_pa_gain_text);

static const char * const rx_hph_mode_mux_text[] = {
	"CLS_H_INVALID", "CLS_H_HIFI", "CLS_H_LP", "CLS_AB", "CLS_H_LOHIFI",
	"CLS_H_ULP", "CLS_AB_HIFI", "CLS_AB_LP", "CLS_AB_LOHIFI",
};

static const struct soc_enum rx_hph_mode_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rx_hph_mode_mux_text),
			    rx_hph_mode_mux_text);

static const struct snd_kcontrol_new wcd9390_snd_controls[] = {
	SOC_ENUM_EXT("EAR PA GAIN", wcd939x_ear_pa_gain_enum,
		wcd939x_ear_pa_gain_get, wcd939x_ear_pa_gain_put),

	SOC_ENUM_EXT("RX HPH Mode", rx_hph_mode_mux_enum_wcd9390,
		wcd939x_rx_hph_mode_get, wcd939x_rx_hph_mode_put),

	SOC_ENUM_EXT("TX0 MODE", tx_mode_mux_enum_wcd9390,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
	SOC_ENUM_EXT("TX1 MODE", tx_mode_mux_enum_wcd9390,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
	SOC_ENUM_EXT("TX2 MODE", tx_mode_mux_enum_wcd9390,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
	SOC_ENUM_EXT("TX3 MODE", tx_mode_mux_enum_wcd9390,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
};

static const struct snd_kcontrol_new wcd9395_snd_controls[] = {
	SOC_ENUM_EXT("RX HPH Mode", rx_hph_mode_mux_enum,
		wcd939x_rx_hph_mode_get, wcd939x_rx_hph_mode_put),

	SOC_ENUM_EXT("TX0 MODE", tx_mode_mux_enum,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
	SOC_ENUM_EXT("TX1 MODE", tx_mode_mux_enum,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
	SOC_ENUM_EXT("TX2 MODE", tx_mode_mux_enum,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
	SOC_ENUM_EXT("TX3 MODE", tx_mode_mux_enum,
			wcd939x_tx_mode_get, wcd939x_tx_mode_put),
};

static const struct snd_kcontrol_new wcd939x_snd_controls[] = {
	SOC_SINGLE_EXT("HPHL_COMP Switch", SND_SOC_NOPM, 0, 1, 0,
		wcd939x_get_compander, wcd939x_set_compander),
	SOC_SINGLE_EXT("HPHR_COMP Switch", SND_SOC_NOPM, 1, 1, 0,
		wcd939x_get_compander, wcd939x_set_compander),
	SOC_SINGLE_EXT("LDOH Enable", SND_SOC_NOPM, 0, 1, 0,
		wcd939x_ldoh_get, wcd939x_ldoh_put),

	SOC_SINGLE_EXT("ADC2_BCS Disable", SND_SOC_NOPM, 0, 1, 0,
		wcd939x_bcs_get, wcd939x_bcs_put),

	SOC_SINGLE_TLV("HPHL Volume", WCD939X_PA_GAIN_CTL_L, 0, 0x18, 0, hph_analog_gain),
	SOC_SINGLE_TLV("HPHR Volume", WCD939X_PA_GAIN_CTL_R, 0, 0x18, 0, hph_analog_gain),
	SOC_SINGLE_TLV("ADC1 Volume", WCD939X_TX_CH1, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", WCD939X_TX_CH2, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", WCD939X_TX_CH3, 0, 20, 0,
			analog_gain),
	SOC_SINGLE_TLV("ADC4 Volume", WCD939X_TX_CH4, 0, 20, 0,
			analog_gain),

	SOC_SINGLE_EXT("HPHL Compander", SND_SOC_NOPM, WCD939X_HPHL, 1, 0,
		wcd939x_hph_compander_get, wcd939x_hph_compander_put),
	SOC_SINGLE_EXT("HPHR Compander", SND_SOC_NOPM, WCD939X_HPHR, 1, 0,
		wcd939x_hph_compander_get, wcd939x_hph_compander_put),

	SOC_SINGLE_EXT("HPHL XTALK", SND_SOC_NOPM, WCD939X_HPHL, 1, 0,
		wcd939x_hph_xtalk_get, wcd939x_hph_xtalk_put),
	SOC_SINGLE_EXT("HPHR XTALK", SND_SOC_NOPM, WCD939X_HPHR, 1, 0,
		wcd939x_hph_xtalk_get, wcd939x_hph_xtalk_put),

	SOC_SINGLE_EXT("HPH PCM Enable", SND_SOC_NOPM, 0, 1, 0,
			wcd939x_hph_pcm_enable_get, wcd939x_hph_pcm_enable_put),

	SOC_ENUM_EXT("ADC1 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("ADC2 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("ADC3 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("ADC4 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC0 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC1 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("MBHC ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC2 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC3 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC4 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC5 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC6 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
	SOC_ENUM_EXT("DMIC7 ChMap", tx_master_ch_enum,
			wcd939x_tx_master_ch_get, wcd939x_tx_master_ch_put),
};

static const struct snd_kcontrol_new adc1_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new adc2_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new adc3_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new adc4_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new amic1_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new amic2_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new amic3_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new amic4_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new amic5_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new va_amic1_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new va_amic2_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new va_amic3_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new va_amic4_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new va_amic5_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic1_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic2_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic3_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic4_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic5_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic6_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic7_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic8_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new ear_rdac_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new hphl_rdac_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new hphr_rdac_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const char * const adc1_mux_text[] = {
	"CH1_AMIC_DISABLE", "CH1_AMIC1", "CH1_AMIC2", "CH1_AMIC3", "CH1_AMIC4", "CH1_AMIC5"
};

static const struct soc_enum adc1_enum =
	SOC_ENUM_SINGLE(WCD939X_TX_CH12_MUX, WCD939X_TX_CH12_MUX_CH1_SEL_SHIFT,
		ARRAY_SIZE(adc1_mux_text), adc1_mux_text);

static const struct snd_kcontrol_new tx_adc1_mux =
	SOC_DAPM_ENUM("ADC1 MUX Mux", adc1_enum);

static const char * const adc2_mux_text[] = {
	"CH2_AMIC_DISABLE", "CH2_AMIC1", "CH2_AMIC2", "CH2_AMIC3", "CH2_AMIC4", "CH2_AMIC5"
};

static const struct soc_enum adc2_enum =
	SOC_ENUM_SINGLE(WCD939X_TX_CH12_MUX, WCD939X_TX_CH12_MUX_CH2_SEL_SHIFT,
		ARRAY_SIZE(adc2_mux_text), adc2_mux_text);

static const struct snd_kcontrol_new tx_adc2_mux =
	SOC_DAPM_ENUM("ADC2 MUX Mux", adc2_enum);

static const char * const adc3_mux_text[] = {
	"CH3_AMIC_DISABLE", "CH3_AMIC1", "CH3_AMIC3", "CH3_AMIC4", "CH3_AMIC5"
};

static const struct soc_enum adc3_enum =
	SOC_ENUM_SINGLE(WCD939X_TX_CH34_MUX, WCD939X_TX_CH34_MUX_CH3_SEL_SHIFT,
		ARRAY_SIZE(adc3_mux_text), adc3_mux_text);

static const struct snd_kcontrol_new tx_adc3_mux =
	SOC_DAPM_ENUM("ADC3 MUX Mux", adc3_enum);

static const char * const adc4_mux_text[] = {
	"CH4_AMIC_DISABLE", "CH4_AMIC1", "CH4_AMIC3", "CH4_AMIC4", "CH4_AMIC5"
};

static const struct soc_enum adc4_enum =
	SOC_ENUM_SINGLE(WCD939X_TX_CH34_MUX, WCD939X_TX_CH34_MUX_CH4_SEL_SHIFT,
		ARRAY_SIZE(adc4_mux_text), adc4_mux_text);

static const struct snd_kcontrol_new tx_adc4_mux =
	SOC_DAPM_ENUM("ADC4 MUX Mux", adc4_enum);

static const char * const rdac3_mux_text[] = {
	"RX3", "RX1"
};


static const struct soc_enum rdac3_enum =
	SOC_ENUM_SINGLE(WCD939X_CDC_EAR_PATH_CTL, 0,
		ARRAY_SIZE(rdac3_mux_text), rdac3_mux_text);

static const struct snd_kcontrol_new rx_rdac3_mux =
	SOC_DAPM_ENUM("RDAC3_MUX Mux", rdac3_enum);


static const char * const rx1_mux_text[] = {
	"ZERO", "RX1 MUX"
};
static const struct soc_enum rx1_enum =
		SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 0, rx1_mux_text);
static const struct snd_kcontrol_new rx1_mux =
	SOC_DAPM_ENUM("RX1 MUX Mux", rx1_enum);


static const char * const rx2_mux_text[] = {
	"ZERO", "RX2 MUX"
};
static const struct soc_enum rx2_enum =
		SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 0, rx2_mux_text);
static const struct snd_kcontrol_new rx2_mux =
	SOC_DAPM_ENUM("RX2 MUX Mux", rx2_enum);

static const char * const rx3_mux_text[] = {
	"ZERO", "RX3 MUX"
};
static const struct soc_enum rx3_enum =
		SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, 0, rx3_mux_text);
static const struct snd_kcontrol_new rx3_mux =
	SOC_DAPM_ENUM("RX3 MUX Mux", rx3_enum);

static const struct snd_soc_dapm_widget wcd939x_dapm_widgets[] = {

	/*input widgets*/
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_INPUT("AMIC3"),
	SND_SOC_DAPM_INPUT("AMIC4"),
	SND_SOC_DAPM_INPUT("AMIC5"),
	SND_SOC_DAPM_INPUT("VA AMIC1"),
	SND_SOC_DAPM_INPUT("VA AMIC2"),
	SND_SOC_DAPM_INPUT("VA AMIC3"),
	SND_SOC_DAPM_INPUT("VA AMIC4"),
	SND_SOC_DAPM_INPUT("VA AMIC5"),

	SND_SOC_DAPM_INPUT("IN1_HPHL"),
	SND_SOC_DAPM_INPUT("IN2_HPHR"),
	SND_SOC_DAPM_INPUT("IN3_EAR"),
	/*
	 * These dummy widgets are null connected to WCD939x dapm input and
	 * output widgets which are not actual path endpoints. This ensures
	 * dapm doesnt set these dapm input and output widgets as endpoints.
	 */
	SND_SOC_DAPM_INPUT("WCD_TX_DUMMY"),
	SND_SOC_DAPM_OUTPUT("WCD_RX_DUMMY"),

	/*tx widgets*/
	SND_SOC_DAPM_ADC_E("ADC1", NULL, SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_adc,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2", NULL, SND_SOC_NOPM, 1, 0,
				wcd939x_codec_enable_adc,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC3", NULL, SND_SOC_NOPM, 2, 0,
				wcd939x_codec_enable_adc,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC4", NULL, SND_SOC_NOPM, 3, 0,
				wcd939x_codec_enable_adc,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 1, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC3", NULL, SND_SOC_NOPM, 2, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC4", NULL, SND_SOC_NOPM, 3, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC5", NULL, SND_SOC_NOPM, 4, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC6", NULL, SND_SOC_NOPM, 5, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC7", NULL, SND_SOC_NOPM, 6, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC8", NULL, SND_SOC_NOPM, 7, 0,
				wcd939x_codec_enable_dmic,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("ADC1 REQ", SND_SOC_NOPM, 0, 0,
				NULL, 0, wcd939x_enable_req,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC2 REQ", SND_SOC_NOPM, 1, 0,
				NULL, 0, wcd939x_enable_req,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC3 REQ", SND_SOC_NOPM, 2, 0,
				NULL, 0, wcd939x_enable_req,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC4 REQ", SND_SOC_NOPM, 3, 0,
				NULL, 0, wcd939x_enable_req,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("AMIC1_MIXER", SND_SOC_NOPM, 0, 0,
				amic1_switch, ARRAY_SIZE(amic1_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("AMIC2_MIXER", SND_SOC_NOPM, 0, 0,
				amic2_switch, ARRAY_SIZE(amic2_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("AMIC3_MIXER", SND_SOC_NOPM, 0, 0,
				amic3_switch, ARRAY_SIZE(amic3_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("AMIC4_MIXER", SND_SOC_NOPM, 0, 0,
				amic4_switch, ARRAY_SIZE(amic4_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("AMIC5_MIXER", SND_SOC_NOPM, 0, 0,
				amic5_switch, ARRAY_SIZE(amic5_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("VA_AMIC1_MIXER", SND_SOC_NOPM, 0, 0,
				va_amic1_switch, ARRAY_SIZE(va_amic1_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("VA_AMIC2_MIXER", SND_SOC_NOPM, 0, 0,
				va_amic2_switch, ARRAY_SIZE(va_amic2_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("VA_AMIC3_MIXER", SND_SOC_NOPM, 0, 0,
				va_amic3_switch, ARRAY_SIZE(va_amic3_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("VA_AMIC4_MIXER", SND_SOC_NOPM, 0, 0,
				va_amic4_switch, ARRAY_SIZE(va_amic4_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("VA_AMIC5_MIXER", SND_SOC_NOPM, 0, 0,
				va_amic5_switch, ARRAY_SIZE(va_amic5_switch), NULL,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("ADC1 MUX", SND_SOC_NOPM, 0, 0,
				&tx_adc1_mux),
	SND_SOC_DAPM_MUX("ADC2 MUX", SND_SOC_NOPM, 0, 0,
				&tx_adc2_mux),
	SND_SOC_DAPM_MUX("ADC3 MUX", SND_SOC_NOPM, 0, 0,
				&tx_adc3_mux),
	SND_SOC_DAPM_MUX("ADC4 MUX", SND_SOC_NOPM, 0, 0,
				&tx_adc4_mux),
	/*tx mixers*/
	SND_SOC_DAPM_MIXER_E("ADC1_MIXER", SND_SOC_NOPM, ADC1, 0,
				adc1_switch, ARRAY_SIZE(adc1_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC2_MIXER", SND_SOC_NOPM, ADC2, 0,
				adc2_switch, ARRAY_SIZE(adc2_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC3_MIXER", SND_SOC_NOPM, ADC3, 0, adc3_switch,
				ARRAY_SIZE(adc3_switch), wcd939x_tx_swr_ctrl,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC4_MIXER", SND_SOC_NOPM, ADC4, 0, adc4_switch,
				ARRAY_SIZE(adc4_switch), wcd939x_tx_swr_ctrl,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC1_MIXER", SND_SOC_NOPM, DMIC1,
				0, dmic1_switch, ARRAY_SIZE(dmic1_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC2_MIXER", SND_SOC_NOPM, DMIC2,
				0, dmic2_switch, ARRAY_SIZE(dmic2_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC3_MIXER", SND_SOC_NOPM, DMIC3,
				0, dmic3_switch, ARRAY_SIZE(dmic3_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC4_MIXER", SND_SOC_NOPM, DMIC4,
				0, dmic4_switch, ARRAY_SIZE(dmic4_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC5_MIXER", SND_SOC_NOPM, DMIC5,
				0, dmic5_switch, ARRAY_SIZE(dmic5_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC6_MIXER", SND_SOC_NOPM, DMIC6,
				0, dmic6_switch, ARRAY_SIZE(dmic6_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC7_MIXER", SND_SOC_NOPM, DMIC7,
				0, dmic7_switch, ARRAY_SIZE(dmic7_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC8_MIXER", SND_SOC_NOPM, DMIC8,
				0, dmic8_switch, ARRAY_SIZE(dmic8_switch),
				wcd939x_tx_swr_ctrl, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	/* micbias widgets*/
	SND_SOC_DAPM_SUPPLY("MIC BIAS1", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS3", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS4", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY(DAPM_MICBIAS1_STANDALONE, SND_SOC_NOPM, 0, 0,
		wcd939x_codec_force_enable_micbias,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY(DAPM_MICBIAS2_STANDALONE, SND_SOC_NOPM, 0, 0,
		wcd939x_codec_force_enable_micbias,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY(DAPM_MICBIAS3_STANDALONE, SND_SOC_NOPM, 0, 0,
		wcd939x_codec_force_enable_micbias,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY(DAPM_MICBIAS4_STANDALONE, SND_SOC_NOPM, 0, 0,
		wcd939x_codec_force_enable_micbias,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("VDD_BUCK", SND_SOC_NOPM, 0, 0,
			     wcd939x_codec_enable_vdd_buck,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("CLS_H_PORT", 1, SND_SOC_NOPM, 0, 0,
			     wcd939x_enable_clsh,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("CLS_H_DUMMY", 1, SND_SOC_NOPM, 0, 0,
			     wcd939x_clsh_dummy, SND_SOC_DAPM_POST_PMD),

	/*rx widgets*/
	SND_SOC_DAPM_PGA_E("EAR PGA", WCD939X_EAR, 7, 0, NULL, 0,
				wcd939x_codec_enable_ear_pa,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHL PGA", WCD939X_HPH, 7, 0, NULL, 0,
				wcd939x_codec_enable_hphl_pa,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHR PGA", WCD939X_HPH, 6, 0, NULL, 0,
				wcd939x_codec_enable_hphr_pa,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("RDAC1", NULL, SND_SOC_NOPM, 0, 0,
				wcd939x_codec_hphl_dac_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RDAC2", NULL, SND_SOC_NOPM, 0, 0,
				wcd939x_codec_hphr_dac_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RDAC3", NULL, SND_SOC_NOPM, 0, 0,
				wcd939x_codec_ear_dac_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("RDAC3_MUX", SND_SOC_NOPM, 0, 0, &rx_rdac3_mux),

	SND_SOC_DAPM_MUX_E("RX1 MUX", SND_SOC_NOPM,  WCD_RX1, 0, &rx1_mux,
		wcd939x_rx_mux, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU
				| SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX2 MUX", SND_SOC_NOPM, WCD_RX2, 0, &rx2_mux,
			wcd939x_rx_mux, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU
				| SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX_E("RX3 MUX", SND_SOC_NOPM, WCD_RX3, 0, &rx3_mux,
			wcd939x_rx3_mux, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("RX1", SND_SOC_NOPM, 0, 0, NULL, 0,
				wcd939x_enable_rx1, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX2", SND_SOC_NOPM, 0, 0, NULL, 0,
				wcd939x_enable_rx2, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX3", SND_SOC_NOPM, 0, 0, NULL, 0,
				wcd939x_enable_rx3, SND_SOC_DAPM_PRE_PMU |
				SND_SOC_DAPM_POST_PMD),

	/* rx mixer widgets*/

	SND_SOC_DAPM_MIXER("EAR_RDAC", SND_SOC_NOPM, 0, 0,
			   ear_rdac_switch, ARRAY_SIZE(ear_rdac_switch)),
	SND_SOC_DAPM_MIXER("HPHL_RDAC", SND_SOC_NOPM, 0, 0,
			   hphl_rdac_switch, ARRAY_SIZE(hphl_rdac_switch)),
	SND_SOC_DAPM_MIXER("HPHR_RDAC", SND_SOC_NOPM, 0, 0,
			   hphr_rdac_switch, ARRAY_SIZE(hphr_rdac_switch)),

	/*output widgets tx*/
	SND_SOC_DAPM_OUTPUT("WCD_TX_OUTPUT"),

	/*output widgets rx*/
	SND_SOC_DAPM_OUTPUT("EAR"),
	SND_SOC_DAPM_OUTPUT("HPHL"),
	SND_SOC_DAPM_OUTPUT("HPHR"),

	/* micbias pull up widgets*/
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS1", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS2", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS3", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS4", SND_SOC_NOPM, 0, 0,
				wcd939x_codec_enable_micbias_pullup,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route wcd939x_audio_map[] = {

/*ADC-1 (channel-1)*/
	{"WCD_TX_DUMMY", NULL, "WCD_TX_OUTPUT"},
	{"WCD_TX_OUTPUT", NULL, "ADC1_MIXER"},
	{"ADC1_MIXER", "Switch", "ADC1 REQ"},
	{"ADC1 REQ", NULL, "ADC1"},
	{"ADC1", NULL, "ADC1 MUX"},
	{"ADC1 MUX", "CH1_AMIC1", "AMIC1_MIXER"},
	{"ADC1 MUX", "CH1_AMIC2", "AMIC2_MIXER"},
	{"ADC1 MUX", "CH1_AMIC3", "AMIC3_MIXER"},
	{"ADC1 MUX", "CH1_AMIC4", "AMIC4_MIXER"},
	{"ADC1 MUX", "CH1_AMIC5", "AMIC5_MIXER"},

	{"AMIC1_MIXER", "Switch", "AMIC1"},
	{"AMIC1_MIXER", NULL, "VA_AMIC1_MIXER"},
	{"VA_AMIC1_MIXER", "Switch", "VA AMIC1"},

	{"AMIC2_MIXER", "Switch", "AMIC2"},
	{"AMIC2_MIXER", NULL, "VA_AMIC2_MIXER"},
	{"VA_AMIC2_MIXER", "Switch", "VA AMIC2"},

	{"AMIC3_MIXER", "Switch", "AMIC3"},
	{"AMIC3_MIXER", NULL, "VA_AMIC3_MIXER"},
	{"VA_AMIC3_MIXER", "Switch", "VA AMIC3"},

	{"AMIC4_MIXER", "Switch", "AMIC4"},
	{"AMIC4_MIXER", NULL, "VA_AMIC4_MIXER"},
	{"VA_AMIC4_MIXER", "Switch", "VA AMIC4"},

	{"AMIC5_MIXER", "Switch", "AMIC5"},
	{"AMIC5_MIXER", NULL, "VA_AMIC5_MIXER"},
	{"VA_AMIC5_MIXER", "Switch", "VA AMIC5"},

/*ADC-2 (channel-2)*/
	{"WCD_TX_DUMMY", NULL, "WCD_TX_OUTPUT"},
	{"WCD_TX_OUTPUT", NULL, "ADC2_MIXER"},
	{"ADC2_MIXER", "Switch", "ADC2 REQ"},
	{"ADC2 REQ", NULL, "ADC2"},
	{"ADC2", NULL, "ADC2 MUX"},
	{"ADC2 MUX", "CH2_AMIC1", "AMIC1_MIXER"},
	{"ADC2 MUX", "CH2_AMIC2", "AMIC2_MIXER"},
	{"ADC2 MUX", "CH2_AMIC3", "AMIC3_MIXER"},
	{"ADC2 MUX", "CH2_AMIC4", "AMIC4_MIXER"},
	{"ADC2 MUX", "CH2_AMIC5", "AMIC5_MIXER"},

/*ADC-3 (channel-3)*/
	{"WCD_TX_DUMMY", NULL, "WCD_TX_OUTPUT"},
	{"WCD_TX_OUTPUT", NULL, "ADC3_MIXER"},
	{"ADC3_MIXER", "Switch", "ADC3 REQ"},
	{"ADC3 REQ", NULL, "ADC3"},
	{"ADC3", NULL, "ADC3 MUX"},
	{"ADC3 MUX", "CH3_AMIC1", "AMIC1_MIXER"},
	{"ADC3 MUX", "CH3_AMIC3", "AMIC3_MIXER"},
	{"ADC3 MUX", "CH3_AMIC4", "AMIC4_MIXER"},
	{"ADC3 MUX", "CH3_AMIC5", "AMIC5_MIXER"},

/*ADC-4 (channel-4)*/
	{"WCD_TX_DUMMY", NULL, "WCD_TX_OUTPUT"},
	{"WCD_TX_OUTPUT", NULL, "ADC4_MIXER"},
	{"ADC4_MIXER", "Switch", "ADC4 REQ"},
	{"ADC4 REQ", NULL, "ADC4"},
	{"ADC4", NULL, "ADC4 MUX"},
	{"ADC4 MUX", "CH4_AMIC1", "AMIC1_MIXER"},
	{"ADC4 MUX", "CH4_AMIC3", "AMIC3_MIXER"},
	{"ADC4 MUX", "CH4_AMIC4", "AMIC4_MIXER"},
	{"ADC4 MUX", "CH4_AMIC5", "AMIC5_MIXER"},

	{"WCD_TX_OUTPUT", NULL, "DMIC1_MIXER"},
	{"DMIC1_MIXER", "Switch", "DMIC1"},

	{"WCD_TX_OUTPUT", NULL, "DMIC2_MIXER"},
	{"DMIC2_MIXER", "Switch", "DMIC2"},

	{"WCD_TX_OUTPUT", NULL, "DMIC3_MIXER"},
	{"DMIC3_MIXER", "Switch", "DMIC3"},

	{"WCD_TX_OUTPUT", NULL, "DMIC4_MIXER"},
	{"DMIC4_MIXER", "Switch", "DMIC4"},

	{"WCD_TX_OUTPUT", NULL, "DMIC5_MIXER"},
	{"DMIC5_MIXER", "Switch", "DMIC5"},

	{"WCD_TX_OUTPUT", NULL, "DMIC6_MIXER"},
	{"DMIC6_MIXER", "Switch", "DMIC6"},

	{"WCD_TX_OUTPUT", NULL, "DMIC7_MIXER"},
	{"DMIC7_MIXER", "Switch", "DMIC7"},

	{"WCD_TX_OUTPUT", NULL, "DMIC8_MIXER"},
	{"DMIC8_MIXER", "Switch", "DMIC8"},

	{"IN1_HPHL", NULL, "WCD_RX_DUMMY"},
	{"IN1_HPHL", NULL, "VDD_BUCK"},
	{"IN1_HPHL", NULL, "CLS_H_PORT"},
	{"RX1 MUX", NULL, "IN1_HPHL"},
	{"RX1", NULL, "RX1 MUX"},
	{"RDAC1", NULL, "RX1"},
	{"HPHL_RDAC", "Switch", "RDAC1"},
	{"HPHL PGA", NULL, "HPHL_RDAC"},
	{"HPHL", NULL, "HPHL PGA"},

	{"IN2_HPHR", NULL, "WCD_RX_DUMMY"},
	{"IN2_HPHR", NULL, "VDD_BUCK"},
	{"IN2_HPHR", NULL, "CLS_H_PORT"},
	{"RX2 MUX", NULL, "IN2_HPHR"},
	{"RX2", NULL, "RX2 MUX"},
	{"RDAC2", NULL, "RX2"},
	{"HPHR_RDAC", "Switch", "RDAC2"},
	{"HPHR PGA", NULL, "HPHR_RDAC"},
	{"HPHR", NULL, "HPHR PGA"},

	{"IN3_EAR", NULL, "WCD_RX_DUMMY"},
	{"IN3_EAR", NULL, "VDD_BUCK"},
	{"IN3_EAR", NULL, "CLS_H_DUMMY"},
	{"RX3 MUX", NULL, "IN3_EAR"},
	{"RX3", NULL, "RX3 MUX"},
	{"RDAC3_MUX", "RX3", "RX3"},
	{"RDAC3_MUX", "RX1", "RX1"},
	{"RDAC3", NULL, "RDAC3_MUX"},
	{"EAR_RDAC", "Switch", "RDAC3"},
	{"EAR PGA", NULL, "EAR_RDAC"},
	{"EAR", NULL, "EAR PGA"},
};

static ssize_t wcd939x_version_read(struct snd_info_entry *entry,
				   void *file_private_data,
				   struct file *file,
				   char __user *buf, size_t count,
				   loff_t pos)
{
	struct wcd939x_priv *priv;
	char buffer[WCD939X_VERSION_ENTRY_SIZE];
	int len = 0;

	priv = (struct wcd939x_priv *) entry->private_data;
	if (!priv) {
		pr_err_ratelimited("%s: wcd939x priv is null\n", __func__);
		return -EINVAL;
	}

	switch (priv->version) {
	case WCD939X_VERSION_1_0:
	case WCD939X_VERSION_1_1:
		len = snprintf(buffer, sizeof(buffer), "WCD939X_1_0\n");
		break;
	case WCD939X_VERSION_2_0:
		len = snprintf(buffer, sizeof(buffer), "WCD939X_2_0\n");
		break;
	default:
		len = snprintf(buffer, sizeof(buffer), "VER_UNDEFINED\n");
	}

	return simple_read_from_buffer(buf, count, &pos, buffer, len);
}

static struct snd_info_entry_ops wcd939x_info_ops = {
	.read = wcd939x_version_read,
};

static ssize_t wcd939x_variant_read(struct snd_info_entry *entry,
				    void *file_private_data,
				    struct file *file,
				    char __user *buf, size_t count,
				    loff_t pos)
{
	struct wcd939x_priv *priv;
	char buffer[WCD939X_VARIANT_ENTRY_SIZE];
	int len = 0;

	priv = (struct wcd939x_priv *) entry->private_data;
	if (!priv) {
		pr_err_ratelimited("%s: wcd939x priv is null\n", __func__);
		return -EINVAL;
	}

	switch (priv->variant) {
	case WCD9390:
		len = snprintf(buffer, sizeof(buffer), "WCD9390\n");
		break;
	case WCD9395:
		len = snprintf(buffer, sizeof(buffer), "WCD9395\n");
		break;
	default:
		len = snprintf(buffer, sizeof(buffer), "VER_UNDEFINED\n");
	}

	return simple_read_from_buffer(buf, count, &pos, buffer, len);
}

static struct snd_info_entry_ops wcd939x_variant_ops = {
	.read = wcd939x_variant_read,
};

/*
 * wcd939x_get_codec_variant
 * @component: component instance
 *
 * Return: codec variant or -EINVAL in error.
 */
int wcd939x_get_codec_variant(struct snd_soc_component *component)
{
	struct wcd939x_priv *priv = NULL;

	if (!component)
		return -EINVAL;

	priv = snd_soc_component_get_drvdata(component);
	if (!priv) {
		dev_err(component->dev,
			"%s:wcd939x not probed\n", __func__);
		return 0;
	}

	return priv->variant;
}
EXPORT_SYMBOL(wcd939x_get_codec_variant);

/*
 * wcd939x_info_create_codec_entry - creates wcd939x module
 * @codec_root: The parent directory
 * @component: component instance
 *
 * Creates wcd939x module, variant and version entry under the given
 * parent directory.
 *
 * Return: 0 on success or negative error code on failure.
 */
int wcd939x_info_create_codec_entry(struct snd_info_entry *codec_root,
				   struct snd_soc_component *component)
{
	struct snd_info_entry *version_entry;
	struct snd_info_entry *variant_entry;
	struct wcd939x_priv *priv;
	struct snd_soc_card *card;

	if (!codec_root || !component)
		return -EINVAL;

	priv = snd_soc_component_get_drvdata(component);
	if (priv->entry) {
		dev_dbg(priv->dev,
			"%s:wcd939x module already created\n", __func__);
		return 0;
	}
	card = component->card;

	priv->entry = snd_info_create_module_entry(codec_root->module,
					     "wcd939x", codec_root);
	if (!priv->entry) {
		dev_dbg(component->dev, "%s: failed to create wcd939x entry\n",
			__func__);
		return -ENOMEM;
	}
	priv->entry->mode = S_IFDIR | 0555;
	if (snd_info_register(priv->entry) < 0) {
		snd_info_free_entry(priv->entry);
		return -ENOMEM;
	}
	version_entry = snd_info_create_card_entry(card->snd_card,
						   "version",
						   priv->entry);
	if (!version_entry) {
		dev_dbg(component->dev, "%s: failed to create wcd939x version entry\n",
			__func__);
		snd_info_free_entry(priv->entry);
		return -ENOMEM;
	}

	version_entry->private_data = priv;
	version_entry->size = WCD939X_VERSION_ENTRY_SIZE;
	version_entry->content = SNDRV_INFO_CONTENT_DATA;
	version_entry->c.ops = &wcd939x_info_ops;

	if (snd_info_register(version_entry) < 0) {
		snd_info_free_entry(version_entry);
		snd_info_free_entry(priv->entry);
		return -ENOMEM;
	}
	priv->version_entry = version_entry;

	variant_entry = snd_info_create_card_entry(card->snd_card,
						   "variant",
						   priv->entry);
	if (!variant_entry) {
		dev_dbg(component->dev, "%s: failed to create wcd939x variant entry\n",
			__func__);
		snd_info_free_entry(version_entry);
		snd_info_free_entry(priv->entry);
		return -ENOMEM;
	}

	variant_entry->private_data = priv;
	variant_entry->size = WCD939X_VARIANT_ENTRY_SIZE;
	variant_entry->content = SNDRV_INFO_CONTENT_DATA;
	variant_entry->c.ops = &wcd939x_variant_ops;

	if (snd_info_register(variant_entry) < 0) {
		snd_info_free_entry(variant_entry);
		snd_info_free_entry(version_entry);
		snd_info_free_entry(priv->entry);
		return -ENOMEM;
	}
	priv->variant_entry = variant_entry;

	return 0;
}
EXPORT_SYMBOL(wcd939x_info_create_codec_entry);

static int wcd939x_set_micbias_data(struct wcd939x_priv *wcd939x,
			      struct wcd939x_pdata *pdata)
{
	int vout_ctl_1 = 0, vout_ctl_2 = 0, vout_ctl_3 = 0, vout_ctl_4 = 0;
	int rc = 0;

	if (!pdata) {
		dev_err(wcd939x->dev, "%s: NULL pdata\n", __func__);
		return -ENODEV;
	}

	/* set micbias voltage */
	vout_ctl_1 = wcd939x_get_micb_vout_ctl_val(pdata->micbias.micb1_mv);
	vout_ctl_2 = wcd939x_get_micb_vout_ctl_val(pdata->micbias.micb2_mv);
	vout_ctl_3 = wcd939x_get_micb_vout_ctl_val(pdata->micbias.micb3_mv);
	vout_ctl_4 = wcd939x_get_micb_vout_ctl_val(pdata->micbias.micb4_mv);
	if (vout_ctl_1 < 0 || vout_ctl_2 < 0 || vout_ctl_3 < 0 ||
	    vout_ctl_4 < 0) {
		rc = -EINVAL;
		goto done;
	}
	regmap_update_bits(wcd939x->regmap, WCD939X_MICB1, 0x3F,
			   vout_ctl_1);
	regmap_update_bits(wcd939x->regmap, WCD939X_MICB2, 0x3F,
			   vout_ctl_2);
	regmap_update_bits(wcd939x->regmap, WCD939X_MICB3, 0x3F,
			   vout_ctl_3);
	regmap_update_bits(wcd939x->regmap, WCD939X_MICB4, 0x3F,
			   vout_ctl_4);

done:
	return rc;
}

static int wcd939x_soc_codec_probe(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_get_dapm(component);
	int ret = -EINVAL;

	dev_info(component->dev, "%s()\n", __func__);
	wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x)
		return -EINVAL;

	wcd939x->component = component;
	snd_soc_component_init_regmap(component, wcd939x->regmap);

	devm_regmap_qti_debugfs_register(&wcd939x->tx_swr_dev->dev, wcd939x->regmap);

	/*Harmonium contains only one variant i.e wcd9395*/
	wcd939x->variant = WCD9395;

	/* Check device tree to see if 2Vpk flag is enabled, this value should not be changed */
	wcd939x->in_2Vpk_mode = of_find_property(wcd939x->dev->of_node,
					"qcom,hph-2p15v-mode", NULL) != NULL;

	wcd939x->fw_data = devm_kzalloc(component->dev,
					sizeof(*(wcd939x->fw_data)),
					GFP_KERNEL);
	if (!wcd939x->fw_data) {
		dev_err(component->dev, "Failed to allocate fw_data\n");
		ret = -ENOMEM;
		goto err;
	}

	set_bit(WCD9XXX_MBHC_CAL, wcd939x->fw_data->cal_bit);
	ret = wcd_cal_create_hwdep(wcd939x->fw_data,
				   WCD9XXX_CODEC_HWDEP_NODE, component);

	if (ret < 0) {
		dev_err(component->dev, "%s hwdep failed %d\n", __func__, ret);
		goto err_hwdep;
	}

	ret = wcd939x_mbhc_init(&wcd939x->mbhc, component, wcd939x->fw_data);
	if (ret) {
		pr_err("%s: mbhc initialization failed\n", __func__);
		goto err_hwdep;
	}

	snd_soc_dapm_ignore_suspend(dapm, "WCD939X_AIF Playback");
	snd_soc_dapm_ignore_suspend(dapm, "WCD939X_AIF Capture");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC4");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "VA AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "VA AMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "VA AMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "VA AMIC4");
	snd_soc_dapm_ignore_suspend(dapm, "VA AMIC5");
	snd_soc_dapm_ignore_suspend(dapm, "WCD_TX_OUTPUT");
	snd_soc_dapm_ignore_suspend(dapm, "IN1_HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "IN2_HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "IN3_EAR");
	snd_soc_dapm_ignore_suspend(dapm, "EAR");
	snd_soc_dapm_ignore_suspend(dapm, "HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "WCD_TX_DUMMY");
	snd_soc_dapm_ignore_suspend(dapm, "WCD_RX_DUMMY");
	snd_soc_dapm_sync(dapm);

	wcd_cls_h_init(&wcd939x->clsh_info);
	wcd939x_init_reg(component);

	if (wcd939x->variant == WCD9390) {
		ret = snd_soc_add_component_controls(component, wcd9390_snd_controls,
					ARRAY_SIZE(wcd9390_snd_controls));
		if (ret < 0) {
			dev_err(component->dev,
				"%s: Failed to add snd ctrls for variant: %d\n",
				__func__, wcd939x->variant);
			goto err_hwdep;
		}
	}
	if (wcd939x->variant == WCD9395) {
		ret = snd_soc_add_component_controls(component, wcd9395_snd_controls,
					ARRAY_SIZE(wcd9395_snd_controls));
		if (ret < 0) {
			dev_err(component->dev,
				"%s: Failed to add snd ctrls for variant: %d\n",
				__func__, wcd939x->variant);
			goto err_hwdep;
		}
	}
       /* Register event notifier */
	wcd939x->nblock.notifier_call = wcd939x_event_notify;
	if (wcd939x->register_notifier) {
		ret = wcd939x->register_notifier(wcd939x->handle,
						&wcd939x->nblock,
						true);
		if (ret) {
			dev_err(component->dev,
				"%s: Failed to register notifier %d\n",
				__func__, ret);
			return ret;
		}
	}
	return ret;

err_hwdep:
	wcd939x->fw_data = NULL;

err:
	return ret;
}

static void wcd939x_soc_codec_remove(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x) {
		dev_err(component->dev, "%s: wcd939x is already NULL\n",
			__func__);
		return;
	}
	if (wcd939x->register_notifier)
		wcd939x->register_notifier(wcd939x->handle,
						&wcd939x->nblock,
						false);
}

static int wcd939x_soc_codec_suspend(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x)
		return 0;
	wcd939x->dapm_bias_off = true;
	return 0;
}

static int wcd939x_soc_codec_resume(struct snd_soc_component *component)
{
	struct wcd939x_priv *wcd939x = snd_soc_component_get_drvdata(component);

	if (!wcd939x)
		return 0;
	wcd939x->dapm_bias_off = false;
	return 0;
}

static struct snd_soc_component_driver soc_codec_dev_wcd939x = {
	.name = WCD939X_DRV_NAME,
	.probe = wcd939x_soc_codec_probe,
	.remove = wcd939x_soc_codec_remove,
	.controls = wcd939x_snd_controls,
	.num_controls = ARRAY_SIZE(wcd939x_snd_controls),
	.dapm_widgets = wcd939x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wcd939x_dapm_widgets),
	.dapm_routes = wcd939x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wcd939x_audio_map),
	.suspend =  wcd939x_soc_codec_suspend,
	.resume = wcd939x_soc_codec_resume,
};

static void wcd_usbss_set_ovp_threshold(u32 threshold)
{
	uint32_t ovp_regs[2][2] = {{WCD_USBSS_DP_EN, 0x00}, {WCD_USBSS_DN_EN, 0x00}};

	/* Get current register values */
	wcd_usbss_register_update(ovp_regs, WCD_USBSS_READ, ARRAY_SIZE(ovp_regs));
	/* Overwrite OVP tresholds */
	ovp_regs[0][1] &= (~P_THRESH_SEL_MASK);
	ovp_regs[0][1] |= (threshold << P_THRESH_SEL_SHIFT);
	ovp_regs[1][1] &= (~P_THRESH_SEL_MASK);
	ovp_regs[1][1] |= (threshold << P_THRESH_SEL_SHIFT);
	/* Write updated register values */
	wcd_usbss_register_update(ovp_regs, WCD_USBSS_WRITE, ARRAY_SIZE(ovp_regs));
}

static int wcd939x_reset(struct device *dev)
{
	struct wcd939x_priv *wcd939x = NULL;
	int rc = 0;
	int value = 0;

	if (!dev)
		return -ENODEV;

	wcd939x = dev_get_drvdata(dev);
	if (!wcd939x)
		return -EINVAL;

	if (!wcd939x->rst_np) {
		dev_err_ratelimited(dev, "%s: reset gpio device node not specified\n",
				__func__);
		return -EINVAL;
	}

	value = msm_cdc_pinctrl_get_state(wcd939x->rst_np);
	if (value > 0)
		return 0;

	/* Set OVP threshold to 4.0V before reset */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_set_ovp_threshold(VTH_4P0);
#endif

	rc = msm_cdc_pinctrl_select_sleep_state(wcd939x->rst_np);
	if (rc) {
		dev_err_ratelimited(dev, "%s: wcd sleep state request fail!\n",
				__func__);
		return rc;
	}
	/* 20us sleep required after pulling the reset gpio to LOW */
	usleep_range(20, 30);

	rc = msm_cdc_pinctrl_select_active_state(wcd939x->rst_np);
	if (rc) {
		dev_err_ratelimited(dev, "%s: wcd active state request fail!\n",
				__func__);
		return rc;
	}
	/* 20us sleep required after pulling the reset gpio to HIGH */
	usleep_range(20, 30);

	/* Set OVP threshold to 4.2V after reset */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_set_ovp_threshold(VTH_4P2);
#endif

	return rc;
}

static int wcd939x_read_of_property_u32(struct device *dev, const char *name,
					u32 *val)
{
	int rc = 0;

	rc = of_property_read_u32(dev->of_node, name, val);
	if (rc)
		dev_err(dev, "%s: Looking up %s property in node %s failed\n",
			__func__, name, dev->of_node->full_name);

	return rc;
}

static int wcd939x_read_of_property_s32(struct device *dev, const char *name,
					s32 *val)
{
	int rc = 0;

	rc = of_property_read_s32(dev->of_node, name, val);
	if (rc)
		dev_err(dev, "%s: Looking up %s property in node %s failed\n",
			__func__, name, dev->of_node->full_name);

	return rc;
}

static void wcd939x_dt_parse_micbias_info(struct device *dev,
					  struct wcd939x_micbias_setting *mb)
{
	u32 prop_val = 0;
	int rc = 0;

	/* MB1 */
	if (of_find_property(dev->of_node, "qcom,cdc-micbias1-mv",
				    NULL)) {
		rc = wcd939x_read_of_property_u32(dev,
						  "qcom,cdc-micbias1-mv",
						  &prop_val);
		if (!rc)
			mb->micb1_mv = prop_val;
	} else {
		dev_info(dev, "%s: Micbias1 DT property not found\n",
			__func__);
	}

	/* MB2 */
	if (of_find_property(dev->of_node, "qcom,cdc-micbias2-mv",
				    NULL)) {
		rc = wcd939x_read_of_property_u32(dev,
						  "qcom,cdc-micbias2-mv",
						  &prop_val);
		if (!rc)
			mb->micb2_mv = prop_val;
	} else {
		dev_info(dev, "%s: Micbias2 DT property not found\n",
			__func__);
	}

	/* MB3 */
	if (of_find_property(dev->of_node, "qcom,cdc-micbias3-mv",
				    NULL)) {
		rc = wcd939x_read_of_property_u32(dev,
						  "qcom,cdc-micbias3-mv",
						  &prop_val);
		if (!rc)
			mb->micb3_mv = prop_val;
	} else {
		dev_info(dev, "%s: Micbias3 DT property not found\n",
			__func__);
	}

	/* MB4 */
	if (of_find_property(dev->of_node, "qcom,cdc-micbias4-mv",
				    NULL)) {
		rc = wcd939x_read_of_property_u32(dev,
						  "qcom,cdc-micbias4-mv",
						  &prop_val);
		if (!rc)
			mb->micb4_mv = prop_val;
	} else {
		dev_info(dev, "%s: Micbias4 DT property not found\n",
			__func__);
	}
}

static void fill_r_common_gnd_buffer(struct wcd939x_usbcss_hs_params *usbcss_hs, u32 val)
{
	size_t i;

	for (i = 0; i < R_COMMON_GND_BUFFER_SIZE; i++)
		usbcss_hs->gnd.r_cm_gnd_buffer.data[i] = val;
}

static void init_usbcss_hs_params(struct wcd939x_usbcss_hs_params *usbcss_hs)
{
	fill_r_common_gnd_buffer(usbcss_hs, 0);
	usbcss_hs->gnd.sbu1.r_gnd_int_fet_mohms = 183;
	usbcss_hs->gnd.sbu2.r_gnd_int_fet_mohms = 127;
	usbcss_hs->gnd.r_cm_gnd_buffer.write_index = 0;
	usbcss_hs->gnd.rdson_mohms = 500;
	usbcss_hs->gnd.rdson_3p6v_mohms = 545;
	usbcss_hs->gnd.r_gnd_ext_fet_mohms = 0;  /* to be computed during MBHC zdet */
	usbcss_hs->gnd.r_common_gnd_mohms = 0;
	usbcss_hs->gnd.r_common_gnd_offset = 0;
	usbcss_hs->gnd.r_common_gnd_margin = 500;
	usbcss_hs->gnd.gnd_ext_fet_delta_mohms = 45;
	usbcss_hs->gnd.gnd_ext_fet_min_mohms = 0;
	usbcss_hs->gnd.sbu1.r_gnd_par_route1_mohms = 5;
	usbcss_hs->gnd.sbu2.r_gnd_par_route1_mohms = 5;
	usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms = 330;
	usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms = 330;
	usbcss_hs->gnd.sbu1.r_gnd_par_tot_mohms = 0;
	usbcss_hs->gnd.sbu2.r_gnd_par_tot_mohms = 0;
	usbcss_hs->gnd.sbu1.r_gnd_res_tot_mohms = 0;
	usbcss_hs->gnd.sbu2.r_gnd_res_tot_mohms = 0;
	usbcss_hs->aud.l.r_aud_int_fet_mohms = 290;
	usbcss_hs->aud.r.r_aud_int_fet_mohms = 290;
	usbcss_hs->aud.l.r_aud_ext_fet_mohms = 0; /* to be computed during MBHC zdet */
	usbcss_hs->aud.r.r_aud_ext_fet_mohms = 0; /* to be computed during MBHC zdet */
	usbcss_hs->aud.l.r_aud_res_tot_mohms = 0;
	usbcss_hs->aud.r.r_aud_res_tot_mohms = 0;
	usbcss_hs->aud.r_surge_mohms = 229;
	usbcss_hs->aud.l.r_load_eff_mohms = 0; /* to be computed during MBHC zdet */
	usbcss_hs->aud.r.r_load_eff_mohms = 0; /* to be computed during MBHC zdet */
	usbcss_hs->aud.l.zval = 0; /* to be computed during MBHC zdet */
	usbcss_hs->aud.r.zval = 0; /* to be computed during MBHC zdet */
	usbcss_hs->zdiffval = 0; /* to be computed during MBHC zdet */
	usbcss_hs->diff_slope_factor_times_1000 = 9898;
	usbcss_hs->se_slope_factor_times_1000 = 9906;
	usbcss_hs->aud.l.r1 = 310;
	usbcss_hs->aud.r.r1 = 310;
	usbcss_hs->aud.l.r3 = 1;
	usbcss_hs->aud.r.r3 = 1;
	usbcss_hs->gnd.sbu1.r4 = 530;
	usbcss_hs->gnd.sbu2.r4 = 530;
	usbcss_hs->gnd.sbu1.r5 = 5;
	usbcss_hs->gnd.sbu2.r5 = 5;
	usbcss_hs->gnd.sbu1.r6 = 1;
	usbcss_hs->gnd.sbu2.r6 = 1;
	usbcss_hs->gnd.sbu1.r7 = 5;
	usbcss_hs->gnd.sbu2.r7 = 5;
	usbcss_hs->aud.k_aud_times_100 = 13;
	usbcss_hs->aud.aud_tap_offset = 0;
	usbcss_hs->xtalk.scale_l = MAX_XTALK_SCALE;
	usbcss_hs->xtalk.alpha_l = MIN_XTALK_ALPHA;
	usbcss_hs->xtalk.scale_r = MAX_XTALK_SCALE;
	usbcss_hs->xtalk.alpha_r = MIN_XTALK_ALPHA;
	usbcss_hs->xtalk.xtalk_config = XTALK_NONE;
}

static void parse_xtalk_param(struct device *dev, u32 default_val, u32 *prop_val_p,
			      char *prop)
{
	int rc = 0;

	if (of_find_property(dev->of_node, prop, NULL)) {
		rc = wcd939x_read_of_property_u32(dev, prop, prop_val_p);
		if ((!rc) && (*prop_val_p <= MAX_USBCSS_HS_IMPEDANCE_MOHMS) && (*prop_val_p > 0))
			return;
		*prop_val_p = default_val;
		dev_dbg(dev, "%s: %s OOB. Default value of %d will be used.\n", __func__, prop,
			default_val);
	} else {
		*prop_val_p = default_val;
		dev_dbg(dev, "%s: %s property not found. Default value of %d will be used.\n",
			__func__, prop, default_val);
	}
}

static void wcd939x_dt_parse_usbcss_hs_info(struct device *dev,
					    struct wcd939x_usbcss_hs_params *usbcss_hs)
{
	u32 prop_val = 0, r_common_gnd_mohms = 0;
	s32 prop_val_signed = 0;
	int rc = 0;

	/* Default values for parameters */
	init_usbcss_hs_params(usbcss_hs);

	/* xtalk_config: Determine type of crosstalk: none (0), digital (1), or analog (2) */
	if (of_find_property(dev->of_node, "qcom,usbcss-hs-xtalk-config", NULL)) {
		rc = wcd939x_read_of_property_u32(dev, "qcom,usbcss-hs-xtalk-config", &prop_val);
		if ((!rc) && (prop_val == XTALK_NONE || prop_val == XTALK_DIGITAL
			      || prop_val == XTALK_ANALOG)) {
			usbcss_hs->xtalk.xtalk_config = (enum xtalk_mode) prop_val;
		} else
			dev_dbg(dev, "%s: %s OOB. Default value of %s used.\n",
				__func__, "qcom,usbcss-hs-xtalk-config", "XTALK_NONE");
	} else
		dev_dbg(dev, "%s: %s property not found. Default value of %s used.\n",
			__func__, "qcom,usbcss-hs-xtalk-config", "XTALK_NONE");

	/* k values for linearizer */
	if (of_find_property(dev->of_node, "qcom,usbcss-hs-lin-k-aud", NULL)) {
		rc = wcd939x_read_of_property_s32(dev, "qcom,usbcss-hs-lin-k-aud",
						  &prop_val_signed);
		if ((!rc) && (prop_val_signed <= MAX_K_TIMES_100) &&
		    (prop_val_signed >= MIN_K_TIMES_100))
			usbcss_hs->aud.k_aud_times_100 = prop_val_signed;
		else
			dev_dbg(dev, "%s: %s OOB. Default value of %d will be used.\n",
				__func__, "qcom,usbcss-hs-lin-k-aud",
				usbcss_hs->aud.k_aud_times_100);
	} else {
		dev_dbg(dev, "%s: %s property not found. Default value of %d will be used.\n",
			__func__, "qcom,usbcss-hs-lin-k-aud",
			usbcss_hs->aud.k_aud_times_100);
	}

	/* Differential slope factor */
	if (of_find_property(dev->of_node, "qcom,usbcss-hs-diff-slope", NULL)) {
		rc = wcd939x_read_of_property_u32(dev, "qcom,usbcss-hs-diff-slope", &prop_val);
		if ((!rc) && (prop_val <= MAX_DIFF_SLOPE_FACTOR) &&
		    (prop_val >= MIN_DIFF_SLOPE_FACTOR))
			usbcss_hs->diff_slope_factor_times_1000 = prop_val;
		else
			dev_dbg(dev, "%s: %s OOB. Default value of %d will be used.\n",
				__func__, "qcom,usbcss-hs-diff-slope",
				usbcss_hs->diff_slope_factor_times_1000);
	} else {
		dev_dbg(dev, "%s: %s property not found. Default value of %d will be used.\n",
			__func__, "qcom,usbcss-hs-diff-slope",
			usbcss_hs->diff_slope_factor_times_1000);
	}

	/* R_ds(on) */
	parse_xtalk_param(dev, usbcss_hs->gnd.rdson_mohms, &prop_val, "qcom,usbcss-hs-rdson-6v");
	usbcss_hs->gnd.rdson_mohms = prop_val;
	/* R_ds(on) Vgs=3.6V */
	parse_xtalk_param(dev, usbcss_hs->gnd.rdson_3p6v_mohms, &prop_val,
			  "qcom,usbcss-hs-rdson-3p6v");
	usbcss_hs->gnd.rdson_3p6v_mohms = prop_val;
	usbcss_hs->gnd.gnd_ext_fet_delta_mohms = (s32) (usbcss_hs->gnd.rdson_3p6v_mohms -
							usbcss_hs->gnd.rdson_mohms);
	/* r_common_gnd_margin */
	parse_xtalk_param(dev, usbcss_hs->gnd.r_common_gnd_margin, &prop_val,
			  "qcom,usbcss-hs-rcom-margin");
	usbcss_hs->gnd.r_common_gnd_margin = prop_val;
	/* r1 */
	parse_xtalk_param(dev, usbcss_hs->aud.l.r1, &prop_val,
			  "qcom,usbcss-hs-r1-l");
	usbcss_hs->aud.l.r1 = prop_val;
	parse_xtalk_param(dev, usbcss_hs->aud.r.r1, &prop_val,
			  "qcom,usbcss-hs-r1-r");
	usbcss_hs->aud.r.r1 = prop_val;
	/* r3 */
	parse_xtalk_param(dev, usbcss_hs->aud.l.r3, &prop_val,
			  "qcom,usbcss-hs-r3-l");
	usbcss_hs->aud.l.r3 = prop_val;
	parse_xtalk_param(dev, usbcss_hs->aud.r.r3, &prop_val,
			  "qcom,usbcss-hs-r3-r");
	usbcss_hs->aud.r.r3 = prop_val;
	/* r4 */
	parse_xtalk_param(dev, usbcss_hs->gnd.sbu1.r4, &prop_val,
			  "qcom,usbcss-hs-r4-sbu1");
	usbcss_hs->gnd.sbu1.r4 = prop_val;
	parse_xtalk_param(dev, usbcss_hs->gnd.sbu2.r4, &prop_val,
			  "qcom,usbcss-hs-r4-sbu2");
	usbcss_hs->gnd.sbu2.r4 = prop_val;
	/* r_gnd_par_route1_mohms and r_gnd_par_route2_mohms */
	if (usbcss_hs->xtalk.xtalk_config == XTALK_ANALOG) {
		parse_xtalk_param(dev, usbcss_hs->gnd.sbu1.r5, &prop_val,
				  "qcom,usbcss-hs-r5-sbu1");
		usbcss_hs->gnd.sbu1.r5 = prop_val;
		parse_xtalk_param(dev, usbcss_hs->gnd.sbu2.r5, &prop_val,
				  "qcom,usbcss-hs-r5-sbu2");
		usbcss_hs->gnd.sbu2.r5 = prop_val;
		usbcss_hs->gnd.sbu1.r_gnd_par_route1_mohms = usbcss_hs->gnd.sbu1.r5 +
							     usbcss_hs->gnd.sbu1.r4;
		usbcss_hs->gnd.sbu2.r_gnd_par_route1_mohms = usbcss_hs->gnd.sbu2.r5 +
							     usbcss_hs->gnd.sbu2.r4;
		usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms = 125;
		usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms = 125;
	} else if (usbcss_hs->xtalk.xtalk_config == XTALK_DIGITAL) {
		parse_xtalk_param(dev, usbcss_hs->gnd.sbu1.r6, &prop_val,
				  "qcom,usbcss-hs-r6-sbu1");
		usbcss_hs->gnd.sbu1.r6 = prop_val;
		parse_xtalk_param(dev, usbcss_hs->gnd.sbu2.r6, &prop_val,
				  "qcom,usbcss-hs-r6-sbu2");
		usbcss_hs->gnd.sbu2.r6 = prop_val;
		usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms = usbcss_hs->gnd.sbu1.r6 +
							     usbcss_hs->gnd.sbu1.r4;
		usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms = usbcss_hs->gnd.sbu2.r6 +
							     usbcss_hs->gnd.sbu2.r4;
		parse_xtalk_param(dev, usbcss_hs->gnd.sbu1.r7, &prop_val,
				  "qcom,usbcss-hs-r7-sbu1");
		usbcss_hs->gnd.sbu1.r7 = prop_val;
		usbcss_hs->gnd.sbu1.r_gnd_par_route1_mohms = prop_val;
		parse_xtalk_param(dev, usbcss_hs->gnd.sbu2.r7, &prop_val,
				  "qcom,usbcss-hs-r7-sbu2");
		usbcss_hs->gnd.sbu2.r7 = prop_val;
		usbcss_hs->gnd.sbu2.r_gnd_par_route1_mohms = prop_val;
	}

	/* Compute total resistances */
	usbcss_hs->gnd.sbu1.r_gnd_par_tot_mohms = usbcss_hs->gnd.sbu1.r_gnd_par_route1_mohms +
						  usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms;
	usbcss_hs->gnd.sbu2.r_gnd_par_tot_mohms = usbcss_hs->gnd.sbu2.r_gnd_par_route1_mohms +
						  usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms;
	usbcss_hs->gnd.sbu1.r_gnd_res_tot_mohms = get_r_gnd_res_tot_mohms(
							usbcss_hs->gnd.sbu1.r_gnd_int_fet_mohms,
							usbcss_hs->gnd.r_gnd_ext_fet_mohms,
							usbcss_hs->gnd.sbu1.r_gnd_par_tot_mohms);
	usbcss_hs->gnd.sbu2.r_gnd_res_tot_mohms = get_r_gnd_res_tot_mohms(
							usbcss_hs->gnd.sbu2.r_gnd_int_fet_mohms,
							usbcss_hs->gnd.r_gnd_ext_fet_mohms,
							usbcss_hs->gnd.sbu2.r_gnd_par_tot_mohms);
	usbcss_hs->aud.l.r_aud_res_tot_mohms = get_r_aud_res_tot_mohms(
							usbcss_hs->aud.l.r_aud_int_fet_mohms,
							usbcss_hs->aud.l.r_aud_ext_fet_mohms,
							usbcss_hs->aud.l.r3);
	usbcss_hs->aud.r.r_aud_res_tot_mohms = get_r_aud_res_tot_mohms(
							usbcss_hs->aud.r.r_aud_int_fet_mohms,
							usbcss_hs->aud.r.r_aud_ext_fet_mohms,
							usbcss_hs->aud.r.r3);

	/* Fill r_common_gnd buffer */
	r_common_gnd_mohms = usbcss_hs->gnd.rdson_mohms +
			     (usbcss_hs->gnd.sbu1.r_gnd_par_route2_mohms +
			      usbcss_hs->gnd.sbu2.r_gnd_par_route2_mohms) / 2;
	fill_r_common_gnd_buffer(usbcss_hs, r_common_gnd_mohms);

	/* Determine min val used for linearizer audio tap calculations */
	if (usbcss_hs->gnd.rdson_3p6v_mohms < GND_EXT_FET_MARGIN_MOHMS)
		usbcss_hs->gnd.gnd_ext_fet_min_mohms = 0;
	else if ((usbcss_hs->gnd.rdson_3p6v_mohms - GND_EXT_FET_MARGIN_MOHMS)
		 < GND_EXT_FET_MARGIN_MOHMS)
		usbcss_hs->gnd.gnd_ext_fet_min_mohms = usbcss_hs->gnd.rdson_3p6v_mohms -
						       GND_EXT_FET_MARGIN_MOHMS;
	else
		usbcss_hs->gnd.gnd_ext_fet_min_mohms = GND_EXT_FET_MARGIN_MOHMS;

#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	/* Set linearizer calibration codes to be sourced from SW */
	wcd_usbss_linearizer_rdac_cal_code_select(LINEARIZER_SOURCE_SW);
#endif
}

static int wcd939x_reset_low(struct device *dev)
{
	struct wcd939x_priv *wcd939x = NULL;
	int rc = 0;

	if (!dev)
		return -ENODEV;

	wcd939x = dev_get_drvdata(dev);
	if (!wcd939x)
		return -EINVAL;

	if (!wcd939x->rst_np) {
		dev_err_ratelimited(dev, "%s: reset gpio device node not specified\n",
				__func__);
		return -EINVAL;
	}

	/* Set OVP threshold to 4.0V before reset */
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	wcd_usbss_set_ovp_threshold(VTH_4P0);
#endif

	rc = msm_cdc_pinctrl_select_sleep_state(wcd939x->rst_np);
	if (rc) {
		dev_err_ratelimited(dev, "%s: wcd sleep state request fail!\n",
				__func__);
		return rc;
	}
	/* 20us sleep required after pulling the reset gpio to LOW */
	usleep_range(20, 30);

	return rc;
}

struct wcd939x_pdata *wcd939x_populate_dt_data(struct device *dev)
{
	struct wcd939x_pdata *pdata = NULL;

	pdata = devm_kzalloc(dev, sizeof(struct wcd939x_pdata),
				GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->rst_np = of_parse_phandle(dev->of_node,
			"qcom,wcd-rst-gpio-node", 0);

	if (!pdata->rst_np) {
		dev_err_ratelimited(dev, "%s: Looking up %s property in node %s failed\n",
				__func__, "qcom,wcd-rst-gpio-node",
				dev->of_node->full_name);
		return NULL;
	}

	/* Parse power supplies */
	msm_cdc_get_power_supplies(dev, &pdata->regulator,
				   &pdata->num_supplies);
	if (!pdata->regulator || (pdata->num_supplies <= 0)) {
		dev_err_ratelimited(dev, "%s: no power supplies defined for codec\n",
			__func__);
		return NULL;
	}

	pdata->rx_slave = of_parse_phandle(dev->of_node, "qcom,rx-slave", 0);
	pdata->tx_slave = of_parse_phandle(dev->of_node, "qcom,tx-slave", 0);

	wcd939x_dt_parse_micbias_info(dev, &pdata->micbias);
	wcd939x_dt_parse_usbcss_hs_info(dev, &pdata->usbcss_hs);

	return pdata;
}

static irqreturn_t wcd939x_wd_handle_irq(int irq, void *data)
{
	pr_err_ratelimited("%s: Watchdog interrupt for irq =%d triggered\n",
			   __func__, irq);
	return IRQ_HANDLED;
}

static struct snd_soc_dai_driver wcd939x_dai[] = {
	{
		.name = "wcd939x_cdc",
		.playback = {
			.stream_name = "WCD939X_AIF Playback",
                        .rates = WCD939X_RATES | WCD939X_FRAC_RATES,
			.formats = WCD939X_FORMATS,
			.rate_max = 384000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.capture = {
			.stream_name = "WCD939X_AIF Capture",
                        .rates = WCD939X_RATES | WCD939X_FRAC_RATES,
			.formats = WCD939X_FORMATS,
			.rate_max = 384000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
	},
};


static const struct reg_default reg_def_1_1[] = {
	{WCD939X_VBG_FINE_ADJ, 0xA5},
	{WCD939X_FLYBACK_NEW_CTRL_2, 0x0},
	{WCD939X_FLYBACK_NEW_CTRL_3, 0x0},
	{WCD939X_FLYBACK_NEW_CTRL_4, 0x44},
	{WCD939X_PA_GAIN_CTL_R, 0x80},
};

static const struct reg_default reg_def_2_0[] = {
	{WCD939X_INTR_MASK_2, 0x3E},
};

static const char *version_to_str(u32 version)
{
	switch (version) {
	case WCD939X_VERSION_1_0:
		return __stringify(WCD939X_1_0);
	case WCD939X_VERSION_1_1:
		return __stringify(WCD939X_1_1);
	case WCD939X_VERSION_2_0:
		return __stringify(WCD939X_2_0);
	}
	return NULL;
}

static void wcd939x_update_regmap_cache(struct wcd939x_priv *wcd939x)
{
	if (wcd939x->version == WCD939X_VERSION_1_0)
		return;

	if (wcd939x->version >= WCD939X_VERSION_1_1) {
		for (int i = 0; i < ARRAY_SIZE(reg_def_1_1); ++i)
			regmap_write(wcd939x->regmap, reg_def_1_1[i].reg, reg_def_1_1[i].def);
	}

	if (wcd939x->version == WCD939X_VERSION_2_0) {
		for (int i = 0; i < ARRAY_SIZE(reg_def_2_0); ++i)
			regmap_write(wcd939x->regmap, reg_def_2_0[i].reg, reg_def_2_0[i].def);
	}
}

static int wcd939x_bind(struct device *dev)
{
	int ret = 0, i = 0;
	struct wcd939x_pdata *pdata = dev_get_platdata(dev);
	struct wcd939x_priv *wcd939x = dev_get_drvdata(dev);
	u8 id1 = 0, status1 = 0;
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	int val = 0;
#endif
	/*
	 * Add 5msec delay to provide sufficient time for
	 * soundwire auto enumeration of slave devices as
	 * as per HW requirement.
	 */
	usleep_range(5000, 5010);

	ret = component_bind_all(dev, wcd939x);
	if (ret) {
		dev_err_ratelimited(dev, "%s: Slave bind failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	wcd939x->rx_swr_dev = get_matching_swr_slave_device(pdata->rx_slave);
	if (!wcd939x->rx_swr_dev) {
		dev_err_ratelimited(dev, "%s: Could not find RX swr slave device\n",
			 __func__);
		ret = -ENODEV;
		goto err;
	}

	wcd939x->tx_swr_dev = get_matching_swr_slave_device(pdata->tx_slave);
	if (!wcd939x->tx_swr_dev) {
		dev_err_ratelimited(dev, "%s: Could not find TX swr slave device\n",
			__func__);
		ret = -ENODEV;
		goto err;
	}
	swr_init_port_params(wcd939x->tx_swr_dev, SWR_NUM_PORTS,
			     wcd939x->swr_tx_port_params);

	/* Check WCD9395 version */
	swr_read(wcd939x->tx_swr_dev, wcd939x->tx_swr_dev->dev_num,
			WCD939X_CHIP_ID1, &id1, 1);
	swr_read(wcd939x->tx_swr_dev, wcd939x->tx_swr_dev->dev_num,
			WCD939X_STATUS_REG_1, &status1, 1);
	if (id1 == 0)
		wcd939x->version = ((status1 & 0x3) ? WCD939X_VERSION_1_1 : WCD939X_VERSION_1_0);
	else if (id1 == 1)
		wcd939x->version = WCD939X_VERSION_2_0;
	wcd939x_version = wcd939x->version;
	dev_info(dev, "%s: wcd9395 version: %s\n", __func__,
			version_to_str(wcd939x->version));
	wcd939x_regmap_config.readable_reg = wcd939x_readable_register;
	wcd939x->regmap = devm_regmap_init_swr(wcd939x->tx_swr_dev,
					       &wcd939x_regmap_config);
	if (!wcd939x->regmap) {
		dev_err_ratelimited(dev, "%s: Regmap init failed\n",
				__func__);
		goto err;
	}
#if IS_ENABLED(CONFIG_QCOM_WCD_USBSS_I2C)
	regmap_read(wcd939x->regmap, WCD939X_EFUSE_REG_17, &val);
	if (wcd939x_version == WCD939X_VERSION_2_0 && val < 3)
		wcd_usbss_update_default_trim();
#endif
	wcd939x_update_regmap_cache(wcd939x);

	/* Set all interupts as edge triggered */
	for (i = 0; i < wcd939x_regmap_irq_chip.num_regs; i++)
		regmap_write(wcd939x->regmap,
			     (WCD939X_INTR_LEVEL_0 + i), 0);

	wcd939x_regmap_irq_chip.irq_drv_data = wcd939x;
	wcd939x->irq_info.wcd_regmap_irq_chip = &wcd939x_regmap_irq_chip;
	wcd939x->irq_info.codec_name = "WCD939X";
	wcd939x->irq_info.regmap = wcd939x->regmap;
	wcd939x->irq_info.dev = dev;
	ret = wcd_irq_init(&wcd939x->irq_info, &wcd939x->virq);

	if (ret) {
		dev_err_ratelimited(wcd939x->dev, "%s: IRQ init failed: %d\n",
			__func__, ret);
		goto err;
	}
	wcd939x->tx_swr_dev->slave_irq = wcd939x->virq;

	ret = wcd939x_set_micbias_data(wcd939x, pdata);
	if (ret < 0) {
		dev_err_ratelimited(dev, "%s: bad micbias pdata\n", __func__);
		goto err_irq;
	}

	/* Request for watchdog interrupt */
	wcd_request_irq(&wcd939x->irq_info, WCD939X_IRQ_HPHR_PDM_WD_INT,
			"HPHR PDM WD INT", wcd939x_wd_handle_irq, NULL);
	wcd_request_irq(&wcd939x->irq_info, WCD939X_IRQ_HPHL_PDM_WD_INT,
			"HPHL PDM WD INT", wcd939x_wd_handle_irq, NULL);
	wcd_request_irq(&wcd939x->irq_info, WCD939X_IRQ_EAR_PDM_WD_INT,
			"EAR PDM WD INT", wcd939x_wd_handle_irq, NULL);
	/* Disable watchdog interrupt for HPH and EAR */
	wcd_disable_irq(&wcd939x->irq_info, WCD939X_IRQ_HPHR_PDM_WD_INT);
	wcd_disable_irq(&wcd939x->irq_info, WCD939X_IRQ_HPHL_PDM_WD_INT);
	wcd_disable_irq(&wcd939x->irq_info, WCD939X_IRQ_EAR_PDM_WD_INT);

	ret = snd_soc_register_component(dev, &soc_codec_dev_wcd939x,
					wcd939x_dai, ARRAY_SIZE(wcd939x_dai));
	if (ret) {
		dev_err_ratelimited(dev, "%s: Codec registration failed\n",
				__func__);
		goto err_irq;
	}
	wcd939x->dev_up = true;

	return ret;
err_irq:
	wcd_irq_exit(&wcd939x->irq_info, wcd939x->virq);
err:
	component_unbind_all(dev, wcd939x);
	return ret;
}

static void wcd939x_unbind(struct device *dev)
{
	struct wcd939x_priv *wcd939x = dev_get_drvdata(dev);

	wcd_free_irq(&wcd939x->irq_info, WCD939X_IRQ_HPHR_PDM_WD_INT, NULL);
	wcd_free_irq(&wcd939x->irq_info, WCD939X_IRQ_HPHL_PDM_WD_INT, NULL);
	wcd_free_irq(&wcd939x->irq_info, WCD939X_IRQ_EAR_PDM_WD_INT, NULL);
	wcd_irq_exit(&wcd939x->irq_info, wcd939x->virq);
	snd_soc_unregister_component(dev);
	component_unbind_all(dev, wcd939x);
}

static const struct of_device_id wcd939x_dt_match[] = {
	{ .compatible = "qcom,wcd939x-codec", .data = "wcd939x"},
	{}
};

static const struct component_master_ops wcd939x_comp_ops = {
	.bind   = wcd939x_bind,
	.unbind = wcd939x_unbind,
};

static int wcd939x_compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static void wcd939x_release_of(struct device *dev, void *data)
{
	of_node_put(data);
}

static int wcd939x_add_slave_components(struct device *dev,
				struct component_match **matchptr)
{
	struct device_node *np, *rx_node, *tx_node;

	np = dev->of_node;

	rx_node = of_parse_phandle(np, "qcom,rx-slave", 0);
	if (!rx_node) {
		dev_err_ratelimited(dev, "%s: Rx-slave node not defined\n", __func__);
		return -ENODEV;
	}
	of_node_get(rx_node);
	component_match_add_release(dev, matchptr,
			wcd939x_release_of,
			wcd939x_compare_of,
			rx_node);

	tx_node = of_parse_phandle(np, "qcom,tx-slave", 0);
	if (!tx_node) {
		dev_err_ratelimited(dev, "%s: Tx-slave node not defined\n", __func__);
			return -ENODEV;
	}
	of_node_get(tx_node);
	component_match_add_release(dev, matchptr,
			wcd939x_release_of,
			wcd939x_compare_of,
			tx_node);
	return 0;
}

static int wcd939x_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	struct wcd939x_priv *wcd939x = NULL;
	struct wcd939x_pdata *pdata = NULL;
	struct wcd_ctrl_platform_data *plat_data = NULL;
	struct device *dev = &pdev->dev;
	int ret;

	wcd939x = devm_kzalloc(dev, sizeof(struct wcd939x_priv),
				GFP_KERNEL);
	if (!wcd939x)
		return -ENOMEM;

	dev_set_drvdata(dev, wcd939x);
	wcd939x->dev = dev;

	pdata = wcd939x_populate_dt_data(dev);
	if (!pdata) {
		dev_err(dev, "%s: Fail to obtain platform data\n", __func__);
		return -EINVAL;
	}
	dev->platform_data = pdata;

	wcd939x->rst_np = pdata->rst_np;
	ret = msm_cdc_init_supplies(dev, &wcd939x->supplies,
				    pdata->regulator, pdata->num_supplies);
	if (!wcd939x->supplies) {
		dev_err(dev, "%s: Cannot init wcd supplies\n",
			__func__);
		return ret;
	}

	plat_data = dev_get_platdata(dev->parent);
	if (!plat_data) {
		dev_err(dev, "%s: platform data from parent is NULL\n",
			__func__);
		return -EINVAL;
	}
	wcd939x->handle = (void *)plat_data->handle;
	if (!wcd939x->handle) {
		dev_err(dev, "%s: handle is NULL\n", __func__);
		return -EINVAL;
	}

	wcd939x->update_wcd_event = plat_data->update_wcd_event;
	if (!wcd939x->update_wcd_event) {
		dev_err(dev, "%s: update_wcd_event api is null!\n",
			__func__);
		return -EINVAL;
	}
	wcd939x->register_notifier = plat_data->register_notifier;
	if (!wcd939x->register_notifier) {
		dev_err(dev, "%s: register_notifier api is null!\n",
			__func__);
		return -EINVAL;
	}

	ret = msm_cdc_enable_static_supplies(&pdev->dev, wcd939x->supplies,
					     pdata->regulator,
					     pdata->num_supplies);
	if (ret) {
		dev_err(dev, "%s: wcd static supply enable failed!\n",
			__func__);
		return ret;
	}

	if (msm_cdc_is_ondemand_supply(wcd939x->dev, wcd939x->supplies,
			pdata->regulator, pdata->num_supplies, "cdc-vdd-px")) {
		ret = msm_cdc_enable_ondemand_supply(wcd939x->dev,
				wcd939x->supplies, pdata->regulator,
				pdata->num_supplies, "cdc-vdd-px");
		if (ret) {
			dev_err(dev, "%s: vdd px supply enable failed!\n",
				__func__);
			return ret;
		}
	}

	ret = wcd939x_parse_port_mapping(dev, "qcom,rx_swr_ch_map",
					CODEC_RX);
	ret |= wcd939x_parse_port_mapping(dev, "qcom,tx_swr_ch_map",
					CODEC_TX);

	if (ret) {
		dev_err(dev, "Failed to read port mapping\n");
		goto err;
	}
	ret = wcd939x_parse_port_params(dev, "qcom,swr-tx-port-params",
					CODEC_TX);
	if (ret) {
		dev_err(dev, "Failed to read port params\n");
		goto err;
	}

	mutex_init(&wcd939x->wakeup_lock);
	mutex_init(&wcd939x->micb_lock);
	ret = wcd939x_add_slave_components(dev, &match);
	if (ret)
		goto err_lock_init;

	wcd939x_reset(dev);

	wcd939x->wakeup = wcd939x_wakeup;

	return component_master_add_with_match(dev,
					&wcd939x_comp_ops, match);

err_lock_init:
	mutex_destroy(&wcd939x->micb_lock);
	mutex_destroy(&wcd939x->wakeup_lock);
err:
	return ret;
}

static int wcd939x_remove(struct platform_device *pdev)
{
	struct wcd939x_priv *wcd939x = NULL;

	wcd939x = platform_get_drvdata(pdev);
	component_master_del(&pdev->dev, &wcd939x_comp_ops);
	mutex_destroy(&wcd939x->micb_lock);
	mutex_destroy(&wcd939x->wakeup_lock);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wcd939x_suspend(struct device *dev)
{
	struct wcd939x_priv *wcd939x = NULL;
	int ret = 0;
	struct wcd939x_pdata *pdata = NULL;

	if (!dev)
		return -ENODEV;

	wcd939x = dev_get_drvdata(dev);
	if (!wcd939x)
		return -EINVAL;

	pdata = dev_get_platdata(wcd939x->dev);

	if (!pdata) {
		dev_err_ratelimited(dev, "%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	if (test_bit(ALLOW_BUCK_DISABLE, &wcd939x->status_mask)) {
		ret = msm_cdc_disable_ondemand_supply(wcd939x->dev,
						wcd939x->supplies,
						pdata->regulator,
						pdata->num_supplies,
						"cdc-vdd-buck");
		if (ret == -EINVAL) {
			dev_err_ratelimited(dev, "%s: vdd buck is not disabled\n",
				__func__);
			return 0;
		}
		clear_bit(ALLOW_BUCK_DISABLE, &wcd939x->status_mask);
	}
	if (wcd939x->dapm_bias_off ||
		(wcd939x->component &&
		(snd_soc_component_get_bias_level(wcd939x->component) ==
			SND_SOC_BIAS_OFF))) {
		msm_cdc_set_supplies_lpm_mode(wcd939x->dev,
					      wcd939x->supplies,
					      pdata->regulator,
					      pdata->num_supplies,
					      true);
		set_bit(WCD_SUPPLIES_LPM_MODE, &wcd939x->status_mask);
		if (msm_cdc_is_ondemand_supply(wcd939x->dev, wcd939x->supplies, pdata->regulator,
				pdata->num_supplies, "cdc-vdd-px")) {

			if (msm_cdc_supply_supports_retention_mode(wcd939x->dev, wcd939x->supplies,
					pdata->regulator, pdata->num_supplies, "cdc-vdd-px") &&
					msm_cdc_check_supply_vote(wcd939x->dev, wcd939x->supplies,
					pdata->regulator, pdata->num_supplies, "cdc-vdd-px")) {
				ret = msm_cdc_disable_ondemand_supply(wcd939x->dev,
					wcd939x->supplies, pdata->regulator,
					pdata->num_supplies, "cdc-vdd-px");
				if (ret) {
					dev_dbg(dev, "%s: vdd px supply suspend failed!\n",
						__func__);
				}
			}
		}
	}

	return 0;
}

static int wcd939x_resume(struct device *dev)
{
	int ret = 0;
	struct wcd939x_priv *wcd939x = NULL;
	struct wcd939x_pdata *pdata = NULL;

	if (!dev)
		return -ENODEV;

	wcd939x = dev_get_drvdata(dev);
	if (!wcd939x)
		return -EINVAL;

	pdata = dev_get_platdata(wcd939x->dev);

	if (!pdata) {
		dev_err_ratelimited(dev, "%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	if (msm_cdc_is_ondemand_supply(wcd939x->dev, wcd939x->supplies, pdata->regulator,
			pdata->num_supplies, "cdc-vdd-px")) {
		if (msm_cdc_supply_supports_retention_mode(wcd939x->dev, wcd939x->supplies,
				pdata->regulator, pdata->num_supplies, "cdc-vdd-px") &&
			!msm_cdc_check_supply_vote(wcd939x->dev, wcd939x->supplies,
					pdata->regulator, pdata->num_supplies, "cdc-vdd-px")) {
			ret = msm_cdc_enable_ondemand_supply(wcd939x->dev, wcd939x->supplies,
					pdata->regulator, pdata->num_supplies, "cdc-vdd-px");
			if (ret) {
				dev_dbg(dev, "%s: vdd px supply resume failed!\n",
					__func__);
			}
		}
	}

	if (test_bit(WCD_SUPPLIES_LPM_MODE, &wcd939x->status_mask)) {
		msm_cdc_set_supplies_lpm_mode(wcd939x->dev,
					      wcd939x->supplies,
					      pdata->regulator,
					      pdata->num_supplies,
					      false);
		clear_bit(WCD_SUPPLIES_LPM_MODE, &wcd939x->status_mask);
	}

	return 0;
}

static const struct dev_pm_ops wcd939x_dev_pm_ops = {
	.suspend_late = wcd939x_suspend,
	.resume_early = wcd939x_resume,
};
#endif

static struct platform_driver wcd939x_codec_driver = {
	.probe = wcd939x_probe,
	.remove = wcd939x_remove,
	.driver = {
		.name = "wcd939x_codec",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(wcd939x_dt_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &wcd939x_dev_pm_ops,
#endif
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(wcd939x_codec_driver);
MODULE_DESCRIPTION("WCD939X Codec driver");
MODULE_LICENSE("GPL v2");
