// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022. Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/of_device.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <linux/pm_qos.h>
#include <linux/nvmem-consumer.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <soc/snd_event.h>
#include <dsp/audio_prm.h>
#include <soc/swr-common.h>
#include <soc/soundwire.h>
#include "device_event.h"
#include "asoc/msm-cdc-pinctrl.h"
#include "asoc/wcd-mbhc-v2.h"
#include "codecs/wcd937x/wcd937x-mbhc.h"
#include "codecs/rouleur/rouleur-mbhc.h"
#include "codecs/wsa881x-analog.h"
#include "codecs/wcd937x/wcd937x.h"
#include "codecs/rouleur/rouleur.h"
#include "codecs/bolero/bolero-cdc.h"
#include <bindings/audio-codec-port-types.h>
#include "bengal-port-config.h"
#include "msm-audio-defs.h"
#include "msm_common.h"
#include "msm_bengal_dailink.h"

#define DRV_NAME "bengal-asoc-snd"
#define __CHIPSET__ "BENGAL "
#define MSM_DAILINK_NAME(name) (__CHIPSET__#name)

#define WCD9XXX_MBHC_DEF_RLOADS     5
#define WCD9XXX_MBHC_DEF_BUTTONS    8
#define DEV_NAME_STR_LEN            32
#define WCD_MBHC_HS_V_MAX           1600

#define WCN_CDC_SLIM_RX_CH_MAX 2
#define WCN_CDC_SLIM_TX_CH_MAX 3

enum {
	PRIM_MI2S = 0,
	SEC_MI2S,
	TERT_MI2S,
	QUAT_MI2S,
	MI2S_MAX,
};

struct msm_asoc_mach_data {
	struct snd_info_entry *codec_root;
	struct msm_common_pdata *common_pdata;
	int usbc_en2_gpio; /* used by gpio driver API */
	struct device_node *dmic01_gpio_p; /* used by pinctrl API */
	struct device_node *dmic23_gpio_p; /* used by pinctrl API */
	struct device_node *mi2s_gpio_p[MI2S_MAX]; /* used by pinctrl API */
	atomic_t mi2s_gpio_ref_count[MI2S_MAX]; /* used by pinctrl API */
	struct device_node *us_euro_gpio_p; /* used by pinctrl API */
	struct pinctrl *usbc_en2_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en1_gpio_p; /* used by pinctrl API */
	struct device_node *hph_en0_gpio_p; /* used by pinctrl API */
	struct device_node *fsa_handle;
	u32 wsa_max_devs;
};

static bool va_disable;
static int msm_rx_tx_codec_init(struct snd_soc_pcm_runtime *rtd);
static int msm_int_wsa_init(struct snd_soc_pcm_runtime *rtd);

static bool is_initial_boot;
static bool codec_reg_done;
static struct snd_soc_card snd_soc_card_bengal_msm;
static int dmic_0_1_gpio_cnt;
static int dmic_2_3_gpio_cnt;

static void *def_wcd_mbhc_cal(void);

/*
 * Need to report LINEIN
 * if R/L channel impedance is larger than 5K ohm
 */
static struct wcd_mbhc_config wcd_mbhc_cfg = {
	.read_fw_bin = false,
	.calibration = NULL,
	.detect_extn_cable = true,
	.mono_stero_detection = false,
	.swap_gnd_mic = NULL,
	.hs_ext_micbias = true,
	.key_code[0] = KEY_MEDIA,
	.key_code[1] = KEY_VOICECOMMAND,
	.key_code[2] = KEY_VOLUMEUP,
	.key_code[3] = KEY_VOLUMEDOWN,
	.key_code[4] = 0,
	.key_code[5] = 0,
	.key_code[6] = 0,
	.key_code[7] = 0,
	.linein_th = 5000,
	.moisture_en = false,
	.mbhc_micbias = MIC_BIAS_2,
	.anc_micbias = MIC_BIAS_2,
	.enable_anc_mic_detect = false,
	.moisture_duty_cycle_en = true,
};

static bool msm_usbc_swap_gnd_mic(struct snd_soc_component *component,
				  bool active)
{
	struct snd_soc_card *card = component->card;
	struct msm_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(card);

	if (!pdata->fsa_handle)
		return false;

	return fsa4480_switch_event(pdata->fsa_handle, FSA_MIC_GND_SWAP);
}

static bool msm_swap_gnd_mic(struct snd_soc_component *component, bool active)
{
	int value = 0;
	bool ret = false;
	struct snd_soc_card *card;
	struct msm_asoc_mach_data *pdata;

	if (!component) {
		pr_err("%s component is NULL\n", __func__);
		return false;
	}
	card = component->card;
	pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return false;

	if (wcd_mbhc_cfg.enable_usbc_analog)
		return msm_usbc_swap_gnd_mic(component, active);

	/* if usbc is not defined, swap using us_euro_gpio_p */
	if (pdata->us_euro_gpio_p) {
		value = msm_cdc_pinctrl_get_state(
				pdata->us_euro_gpio_p);
		if (value)
			msm_cdc_pinctrl_select_sleep_state(
					pdata->us_euro_gpio_p);
		else
			msm_cdc_pinctrl_select_active_state(
					pdata->us_euro_gpio_p);
		dev_dbg(component->dev, "%s: swap select switch %d to %d\n",
			__func__, value, !value);
		ret = true;
	}

	return ret;
}

static int msm_dmic_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct msm_asoc_mach_data *pdata = NULL;
	struct snd_soc_component *component =
					snd_soc_dapm_to_component(w->dapm);
	int ret = 0;
	u32 dmic_idx;
	int *dmic_gpio_cnt;
	struct device_node *dmic_gpio;
	char  *wname;

	wname = strpbrk(w->name, "0123");
	if (!wname) {
		dev_err(component->dev, "%s: widget not found\n", __func__);
		return -EINVAL;
	}

	ret = kstrtouint(wname, 10, &dmic_idx);
	if (ret < 0) {
		dev_err(component->dev, "%s: Invalid DMIC line on the codec\n",
			__func__);
		return -EINVAL;
	}

	pdata = snd_soc_card_get_drvdata(component->card);

	switch (dmic_idx) {
	case 0:
	case 1:
		dmic_gpio_cnt = &dmic_0_1_gpio_cnt;
		dmic_gpio = pdata->dmic01_gpio_p;
		break;
	case 2:
	case 3:
		dmic_gpio_cnt = &dmic_2_3_gpio_cnt;
		dmic_gpio = pdata->dmic23_gpio_p;
		break;
	default:
		dev_err(component->dev, "%s: Invalid DMIC Selection\n",
			__func__);
		return -EINVAL;
	}

	dev_dbg(component->dev, "%s: event %d DMIC%d dmic_gpio_cnt %d\n",
			__func__, event, dmic_idx, *dmic_gpio_cnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		(*dmic_gpio_cnt)++;
		if (*dmic_gpio_cnt == 1) {
			ret = msm_cdc_pinctrl_select_active_state(
						dmic_gpio);
			if (ret < 0) {
				pr_err("%s: gpio set cannot be activated %sd",
					__func__, "dmic_gpio");
				return ret;
			}
		}

		break;
	case SND_SOC_DAPM_POST_PMD:
		(*dmic_gpio_cnt)--;
		if (*dmic_gpio_cnt == 0) {
			ret = msm_cdc_pinctrl_select_sleep_state(
					dmic_gpio);
			if (ret < 0) {
				pr_err("%s: gpio set cannot be de-activated %sd",
					__func__, "dmic_gpio");
				return ret;
			}
		}
		break;
	default:
		pr_err("%s: invalid DAPM event %d\n", __func__, event);
		return -EINVAL;
	}
	return 0;
}

static const struct snd_soc_dapm_widget msm_int_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Analog Mic1", NULL),
	SND_SOC_DAPM_MIC("Analog Mic2", NULL),
	SND_SOC_DAPM_MIC("Analog Mic3", NULL),
	SND_SOC_DAPM_MIC("Analog Mic4", NULL),
	SND_SOC_DAPM_MIC("Digital Mic0", msm_dmic_event),
	SND_SOC_DAPM_MIC("Digital Mic1", msm_dmic_event),
	SND_SOC_DAPM_MIC("Digital Mic2", msm_dmic_event),
	SND_SOC_DAPM_MIC("Digital Mic3", msm_dmic_event),
};

static int msm_wcn_init(struct snd_soc_pcm_runtime *rtd)
{
	unsigned int rx_ch[WCN_CDC_SLIM_RX_CH_MAX] = {157, 158};
	unsigned int tx_ch[WCN_CDC_SLIM_TX_CH_MAX]  = {159, 160, 161};
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	int ret = 0;

	ret = snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
					   tx_ch, ARRAY_SIZE(rx_ch), rx_ch);
	if (ret)
                return ret;

        msm_common_dai_link_init(rtd);
	return ret;
}

static struct snd_info_entry *msm_snd_info_create_subdir(struct module *mod,
                                const char *name,
                                struct snd_info_entry *parent)
{
        struct snd_info_entry *entry;

        entry = snd_info_create_module_entry(mod, name, parent);
        if (!entry)
                return NULL;
        entry->mode = S_IFDIR | 0555;
        if (snd_info_register(entry) < 0) {
                snd_info_free_entry(entry);
                return NULL;
        }
        return entry;
}

static void *def_wcd_mbhc_cal(void)
{
	void *wcd_mbhc_cal;
	struct wcd_mbhc_btn_detect_cfg *btn_cfg;
	u16 *btn_high;

	wcd_mbhc_cal = kzalloc(WCD_MBHC_CAL_SIZE(WCD_MBHC_DEF_BUTTONS,
				WCD9XXX_MBHC_DEF_RLOADS), GFP_KERNEL);
	if (!wcd_mbhc_cal)
		return NULL;

	WCD_MBHC_CAL_PLUG_TYPE_PTR(wcd_mbhc_cal)->v_hs_max = WCD_MBHC_HS_V_MAX;
	WCD_MBHC_CAL_BTN_DET_PTR(wcd_mbhc_cal)->num_btn = WCD_MBHC_DEF_BUTTONS;
	btn_cfg = WCD_MBHC_CAL_BTN_DET_PTR(wcd_mbhc_cal);
	btn_high = ((void *)&btn_cfg->_v_btn_low) +
		(sizeof(btn_cfg->_v_btn_low[0]) * btn_cfg->num_btn);

	btn_high[0] = 75;
	btn_high[1] = 150;
	btn_high[2] = 237;
	btn_high[3] = 500;
	btn_high[4] = 500;
	btn_high[5] = 500;
	btn_high[6] = 500;
	btn_high[7] = 500;

	return wcd_mbhc_cal;
}

static struct snd_soc_ops msm_common_be_ops = {
        .hw_params = msm_common_snd_hw_params,
        .startup = msm_common_snd_startup,
        .shutdown = msm_common_snd_shutdown,
};

static struct snd_soc_dai_link msm_common_be_dai_links[] = {
	/* Backend AFE DAI Links */
#if 0
	/* Incall Record Uplink BACK END DAI Link */
	{
		.name = LPASS_BE_INCALL_RECORD_TX,
		.stream_name = "Voice Uplink Capture",
		.cpu_dai_name = "msm-dai-q6-dev.32772",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_INCALL_RECORD_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	/* Incall Record Downlink BACK END DAI Link */
	{
		.name = LPASS_BE_INCALL_RECORD_RX,
		.stream_name = "Voice Downlink Capture",
		.cpu_dai_name = "msm-dai-q6-dev.32771",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.dpcm_capture = 1,
		.id = MSM_BACKEND_DAI_INCALL_RECORD_RX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	/* Incall Music BACK END DAI Link */
	{
		.name = LPASS_BE_VOICE_PLAYBACK_TX,
		.stream_name = "Voice Farend Playback",
		.cpu_dai_name = "msm-dai-q6-dev.32773",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
	},
	/* Incall Music 2 BACK END DAI Link */
	{
		.name = LPASS_BE_VOICE2_PLAYBACK_TX,
		.stream_name = "Voice2 Farend Playback",
		.cpu_dai_name = "msm-dai-q6-dev.32770",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.dpcm_playback = 1,
		.id = MSM_BACKEND_DAI_VOICE2_PLAYBACK_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
	},
#endif
	/* Proxy Tx BACK END DAI Link */
	{
		.name = LPASS_BE_RT_PROXY_PCM_TX,
		.stream_name = LPASS_BE_RT_PROXY_PCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(proxy_tx),
	},
	/* Proxy Rx BACK END DAI Link */
	{
		.name = LPASS_BE_RT_PROXY_PCM_RX,
		.stream_name = LPASS_BE_RT_PROXY_PCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(proxy_rx),
	},
	{
		.name = LPASS_BE_USB_AUDIO_RX,
		.stream_name = LPASS_BE_USB_AUDIO_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(usb_audio_rx),
	},
	{
		.name = LPASS_BE_USB_AUDIO_TX,
		.stream_name = LPASS_BE_USB_AUDIO_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(usb_audio_tx),
	},
};

static struct snd_soc_dai_link msm_tdm_be_dai_links[] = {
	{
		.name = LPASS_BE_PRI_TDM_RX_0,
		.stream_name = LPASS_BE_PRI_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_tdm_rx_0),
	},
	{
		.name = LPASS_BE_PRI_TDM_TX_0,
		.stream_name = LPASS_BE_PRI_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_tdm_tx_0),
	},
	{
		.name = LPASS_BE_SEC_TDM_RX_0,
		.stream_name = LPASS_BE_SEC_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(sec_tdm_rx_0),
	},
	{
		.name = LPASS_BE_SEC_TDM_TX_0,
		.stream_name = LPASS_BE_SEC_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_tdm_tx_0),
	},
	{
		.name = LPASS_BE_TERT_TDM_RX_0,
		.stream_name = LPASS_BE_TERT_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(tert_tdm_tx_0),
	},
	{
		.name = LPASS_BE_TERT_TDM_TX_0,
		.stream_name = LPASS_BE_TERT_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tert_tdm_tx_0),
	},
	{
		.name = LPASS_BE_QUAT_TDM_RX_0,
		.stream_name = LPASS_BE_QUAT_TDM_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(quat_tdm_rx_0),
	},
	{
		.name = LPASS_BE_QUAT_TDM_TX_0,
		.stream_name = LPASS_BE_QUAT_TDM_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(quat_tdm_tx_0),
	},
};

static struct snd_soc_dai_link msm_wcn_btfm_be_dai_links[] = {
	{
		.name = LPASS_BE_SLIMBUS_7_RX,
		.stream_name = LPASS_BE_SLIMBUS_7_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.init = &msm_wcn_init,
		.ops = &msm_common_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(slimbus_7_rx),
	},
	{
		.name = LPASS_BE_SLIMBUS_7_TX,
		.stream_name = LPASS_BE_SLIMBUS_7_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(slimbus_7_tx),
	},
	{
		.name = LPASS_BE_SLIMBUS_8_TX,
		.stream_name = LPASS_BE_SLIMBUS_8_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(slimbus_8_tx),
	},
};

static struct snd_soc_dai_link msm_mi2s_be_dai_links[] = {
	{
		.name = LPASS_BE_PRI_MI2S_RX,
		.stream_name = LPASS_BE_PRI_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(pri_mi2s_rx),
	},
	{
		.name = LPASS_BE_PRI_MI2S_TX,
		.stream_name = LPASS_BE_PRI_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pri_mi2s_tx),
	},
	{
		.name = LPASS_BE_SEC_MI2S_RX,
		.stream_name = LPASS_BE_SEC_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(sec_mi2s_rx),
	},
	{
		.name = LPASS_BE_SEC_MI2S_TX,
		.stream_name = LPASS_BE_SEC_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_mi2s_tx),
	},
	{
		.name = LPASS_BE_TERT_MI2S_RX,
		.stream_name = LPASS_BE_TERT_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(tert_mi2s_rx),
	},
	{
		.name = LPASS_BE_TERT_MI2S_TX,
		.stream_name = LPASS_BE_TERT_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tert_mi2s_tx),
	},
	{
		.name = LPASS_BE_QUAT_MI2S_RX,
		.stream_name = LPASS_BE_QUAT_MI2S_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(quat_mi2s_rx),
	},
	{
		.name = LPASS_BE_QUAT_MI2S_TX,
		.stream_name = LPASS_BE_QUAT_MI2S_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(quat_mi2s_tx),
	},
};

static struct snd_soc_dai_link msm_auxpcm_be_dai_links[] = {
	/* Primary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_PRI_AUXPCM_RX,
		.stream_name = LPASS_BE_PRI_AUXPCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,// TODO for AUX
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_PRI_AUXPCM_TX,
		.stream_name = LPASS_BE_PRI_AUXPCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
	/* Secondary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_SEC_AUXPCM_RX,
		.stream_name = LPASS_BE_SEC_AUXPCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,// TODO for AUX
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_auxpcm_rx),
	},
	{
		.name = LPASS_BE_SEC_AUXPCM_TX,
		.stream_name = LPASS_BE_SEC_AUXPCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(sec_auxpcm_tx),
	},
	/* Tertiary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_TERT_AUXPCM_RX,
		.stream_name = LPASS_BE_TERT_AUXPCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,// TODO for AUX
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tert_auxpcm_rx),
	},
	{
		.name = LPASS_BE_TERT_AUXPCM_TX,
		.stream_name = LPASS_BE_TERT_AUXPCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(tert_auxpcm_tx),
	},
	/* Quaternary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_QUAT_AUXPCM_RX,
		.stream_name = LPASS_BE_QUAT_AUXPCM_RX,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,// TODO for AUX
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(quat_auxpcm_rx),
	},
	{
		.name = LPASS_BE_QUAT_AUXPCM_TX,
		.stream_name = LPASS_BE_QUAT_AUXPCM_TX,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ops = &msm_common_be_ops,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(quat_auxpcm_tx),
	},
};

static struct snd_soc_dai_link msm_rx_tx_cdc_dma_be_dai_links[] = {
	/* RX CDC DMA Backend DAI Links */
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_0,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_0,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(rx_dma_rx0),
		.init = &msm_rx_tx_codec_init,
	},
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_1,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_1,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(rx_dma_rx1),
		.init = &msm_int_wsa_init,
	},
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_2,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_2,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(rx_dma_rx2),
	},
	{
		.name = LPASS_BE_RX_CDC_DMA_RX_3,
		.stream_name = LPASS_BE_RX_CDC_DMA_RX_3,
		.playback_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(rx_dma_rx3),
	},
	/* TX CDC DMA Backend DAI Links */
	{
		.name = LPASS_BE_TX_CDC_DMA_TX_3,
		.stream_name = LPASS_BE_TX_CDC_DMA_TX_3,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(tx_dma_tx3),
	},
	{
		.name = LPASS_BE_TX_CDC_DMA_TX_4,
		.stream_name = LPASS_BE_TX_CDC_DMA_TX_4,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(tx_dma_tx4),
	},
};

static struct snd_soc_dai_link msm_va_cdc_dma_be_dai_links[] = {
	{
		.name = LPASS_BE_VA_CDC_DMA_TX_0,
		.stream_name = LPASS_BE_VA_CDC_DMA_TX_0,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(va_dma_tx0),
	},
	{
		.name = LPASS_BE_VA_CDC_DMA_TX_1,
		.stream_name = LPASS_BE_VA_CDC_DMA_TX_1,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(va_dma_tx1),
	},
	{
		.name = LPASS_BE_VA_CDC_DMA_TX_2,
		.stream_name = LPASS_BE_VA_CDC_DMA_TX_2,
		.capture_only = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.ops = &msm_common_be_ops,
		SND_SOC_DAILINK_REG(va_dma_tx2),
	},
};

static struct snd_soc_dai_link msm_bengal_dai_links[
			ARRAY_SIZE(msm_common_be_dai_links) +
			ARRAY_SIZE(msm_mi2s_be_dai_links) +
			ARRAY_SIZE(msm_auxpcm_be_dai_links) +
			ARRAY_SIZE(msm_rx_tx_cdc_dma_be_dai_links) +
			ARRAY_SIZE(msm_va_cdc_dma_be_dai_links) +
			ARRAY_SIZE(msm_wcn_btfm_be_dai_links) +
			ARRAY_SIZE(msm_tdm_be_dai_links)];

static int msm_populate_dai_link_component_of_node(
					struct snd_soc_card *card)
{
	int i, j, index, ret = 0;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np = NULL;
	int codecs_enabled = 0;
	struct snd_soc_dai_link_component *codecs_comp = NULL;

	if (!cdev) {
		dev_err(cdev, "%s: Sound card device memory NULL\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < card->num_links; i++) {
		if (dai_link[i].init == NULL)
			dai_link[i].init = &msm_common_dai_link_init;

		/* populate codec_of_node for snd card dai links */
		if (dai_link[i].num_codecs > 0) {
			for (j = 0; j < dai_link[i].num_codecs; j++) {
				if (dai_link[i].codecs[j].of_node ||
					!dai_link[i].codecs[j].name)
					continue;
				index = of_property_match_string(cdev->of_node,
						 "asoc-codec-names",
						 dai_link[i].codecs[j].name);
				if (index < 0)
					continue;
				np = of_parse_phandle(cdev->of_node, "asoc-codec",
					      index);
				if (!np) {
					dev_err(cdev,
					"%s: retrieving phandle for codec %s failed\n",
					__func__, dai_link[i].codecs[j].name);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].codecs[j].of_node = np;
				dai_link[i].codecs[j].name = NULL;
			}
		}
	}

	/* In multi-codec scenario, check if codecs are enabled for this platform */
	for (i = 0; i < card->num_links; i++) {
		codecs_enabled = 0;
		if (dai_link[i].num_codecs > 1) {
			for (j = 0; j < dai_link[i].num_codecs; j++) {
				if (!dai_link[i].codecs[j].of_node)
					continue;
				np = dai_link[i].codecs[j].of_node;
				if (!of_device_is_available(np)) {
					dev_dbg(cdev, "%s: codec is disabled: %s\n",
						__func__,np->full_name);
					dai_link[i].codecs[j].of_node = NULL;
					continue;
				}
				codecs_enabled++;
			}
			if (codecs_enabled > 0 &&
				codecs_enabled < dai_link[i].num_codecs) {
					codecs_comp = devm_kzalloc(cdev,
						sizeof(struct snd_soc_dai_link_component)
						* codecs_enabled, GFP_KERNEL);
					if (!codecs_comp) {
						dev_err(cdev, "%s: %s dailink codec component alloc failed\n",
							 __func__, dai_link[i].name);
						ret = -ENOMEM;
						goto err;
					}
					index = 0;
					for (j = 0; j < dai_link[i].num_codecs; j++) {
						if(dai_link[i].codecs[j].of_node) {
							codecs_comp[index].of_node =
								dai_link[i].codecs[j].of_node;
							codecs_comp[index].dai_name =
								dai_link[i].codecs[j].dai_name;
							codecs_comp[index].name = NULL;
							index++;
						}
					}
					dai_link[i].codecs = codecs_comp;
					dai_link[i].num_codecs = codecs_enabled;
				}
			}
		}
err:
	return ret;
}

struct snd_soc_card snd_soc_card_stub_msm = {
	.name		= "bengal-stub-snd-card",
};

static int msm_audrx_stub_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int msm_snd_stub_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	return 0;
}

static struct snd_soc_ops msm_stub_be_ops = {
	.hw_params = msm_snd_stub_hw_params,
};

static struct snd_soc_dai_link msm_stub_be_dai_links[] = {
	/* Backend DAI Links */
	{
		.name = LPASS_BE_PRI_AUXPCM_RX,
		.stream_name = LPASS_BE_PRI_AUXPCM_RX,
		.playback_only = 1,
		.init = &msm_audrx_stub_init,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_rx),
	},
	{
		.name = LPASS_BE_PRI_AUXPCM_TX,
		.stream_name = LPASS_BE_PRI_AUXPCM_TX,
		.capture_only = 1,
		.ignore_suspend = 1,
		.ops = &msm_stub_be_ops,
		SND_SOC_DAILINK_REG(auxpcm_tx),
	},
};

static struct snd_soc_dai_link msm_stub_dai_links[
			 ARRAY_SIZE(msm_stub_be_dai_links)];

static struct snd_soc_card snd_soc_card_bengal_msm;
static const struct of_device_id bengal_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,bengal-asoc-snd",
	  .data = "codec"},
	{ .compatible = "qcom,bengal-asoc-snd-stub",
	  .data = "stub_codec"},
	{},
};

static struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *dailink = NULL;
	int total_links = 0;
	int len_1 = 0;
	int rc = 0;
	u32 mi2s_audio_intf = 0;
	u32 auxpcm_audio_intf = 0;
	u32 rxtx_bolero_codec = 0;
	u32 va_bolero_codec = 0;
	u32 val = 0;
	u32 wcn_btfm_intf = 0;
	const struct of_device_id *match;

	match = of_match_node(bengal_asoc_machine_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "%s: No DT match found for sound card\n",
			__func__);
		return NULL;
	}

	if (!strcmp(match->data, "codec")) {
		card = &snd_soc_card_bengal_msm;

		memcpy(msm_bengal_dai_links + total_links,
		       msm_common_be_dai_links,
		       sizeof(msm_common_be_dai_links));
		total_links += ARRAY_SIZE(msm_common_be_dai_links);

		rc = of_property_read_u32(dev->of_node,
					  "qcom,rxtx-bolero-codec",
					  &rxtx_bolero_codec);
		if (rc) {
			dev_dbg(dev, "%s: No DT match RXTX Macro codec\n",
				__func__);
		} else {
			if (rxtx_bolero_codec) {
				memcpy(msm_bengal_dai_links + total_links,
				       msm_rx_tx_cdc_dma_be_dai_links,
				       sizeof(msm_rx_tx_cdc_dma_be_dai_links));
				total_links +=
					ARRAY_SIZE(
						msm_rx_tx_cdc_dma_be_dai_links);
			}
		}

		rc = of_property_read_u32(dev->of_node, "qcom,va-bolero-codec",
					  &va_bolero_codec);
		if (rc) {
			dev_dbg(dev, "%s: No DT match VA Macro codec\n",
				__func__);
		} else {
			if (va_bolero_codec) {
				memcpy(msm_bengal_dai_links + total_links,
				       msm_va_cdc_dma_be_dai_links,
				       sizeof(msm_va_cdc_dma_be_dai_links));
				total_links +=
					ARRAY_SIZE(msm_va_cdc_dma_be_dai_links);
			}
		}

		rc = of_property_read_u32(dev->of_node, "qcom,mi2s-audio-intf",
					  &mi2s_audio_intf);
		if (rc) {
			dev_dbg(dev, "%s: No DT match MI2S audio interface\n",
				__func__);
		} else {
			if (mi2s_audio_intf) {
				memcpy(msm_bengal_dai_links + total_links,
					msm_mi2s_be_dai_links,
					sizeof(msm_mi2s_be_dai_links));
				total_links +=
					ARRAY_SIZE(msm_mi2s_be_dai_links);
			}
		}

		rc = of_property_read_u32(dev->of_node,
					  "qcom,auxpcm-audio-intf",
					  &auxpcm_audio_intf);
		if (rc) {
			dev_dbg(dev, "%s: No DT match Aux PCM interface\n",
				__func__);
		} else {
			if (auxpcm_audio_intf) {
				memcpy(msm_bengal_dai_links + total_links,
					msm_auxpcm_be_dai_links,
					sizeof(msm_auxpcm_be_dai_links));
				total_links +=
					ARRAY_SIZE(msm_auxpcm_be_dai_links);
			}
		}

		rc = of_property_read_u32(dev->of_node, "qcom,tdm-audio-intf",
				&val);
		if (!rc && val) {
			memcpy(msm_bengal_dai_links + total_links,
				msm_tdm_be_dai_links,
				sizeof(msm_tdm_be_dai_links));
			total_links +=
				ARRAY_SIZE(msm_tdm_be_dai_links);
		}

		rc = of_property_read_u32(dev->of_node, "qcom,wcn-btfm",
					  &wcn_btfm_intf);
		if (rc) {
			dev_dbg(dev, "%s: No DT match wcn btfm interface\n",
				__func__);
		} else {
			if (wcn_btfm_intf) {
				memcpy(msm_bengal_dai_links + total_links,
					msm_wcn_btfm_be_dai_links,
					sizeof(msm_wcn_btfm_be_dai_links));
				total_links +=
					ARRAY_SIZE(msm_wcn_btfm_be_dai_links);
			}
		}
		dailink = msm_bengal_dai_links;
	} 

	else if (!strcmp(match->data, "stub_codec")) {
		card = &snd_soc_card_stub_msm;
		len_1 = ARRAY_SIZE(msm_stub_be_dai_links);

		memcpy(msm_stub_dai_links + len_1,
		       msm_stub_be_dai_links,
		       sizeof(msm_stub_be_dai_links));

		dailink = msm_stub_dai_links;
		total_links = len_1;
	}

	if (card) {
		card->dai_link = dailink;
		card->num_links = total_links;
	}

	return card;
}

static int msm_rx_tx_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *bolero_component = NULL;
	struct snd_soc_component *component = NULL;
	struct snd_soc_dapm_context *dapm = NULL;
	int ret = 0;
	void *mbhc_calibration;
	struct snd_info_entry *entry;
	struct snd_card *card = NULL;
	struct msm_asoc_mach_data *pdata =
			snd_soc_card_get_drvdata(rtd->card);

	pr_err("%s:: enter \n", __func__);

	bolero_component = snd_soc_rtdcom_lookup(rtd, "bolero_codec");
	if (!bolero_component) {
		pr_err("%s: could not find component for bolero_codec\n",
			__func__);
		return -EINVAL;
	}
	pr_err("%s:: bolero comp lookup done\n", __func__);

	dapm = snd_soc_component_get_dapm(bolero_component);
	snd_soc_dapm_new_controls(dapm, msm_int_dapm_widgets,
		ARRAY_SIZE(msm_int_dapm_widgets));
	pr_err("%s:: dapm new controls msm_int_dapm_widgets \n", __func__);

	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic0");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic1");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic2");
	snd_soc_dapm_ignore_suspend(dapm, "Digital Mic3");

	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic1");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic2");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic3");
	snd_soc_dapm_ignore_suspend(dapm, "Analog Mic4");

	snd_soc_dapm_sync(dapm);
	card = rtd->card->snd_card;

	if (!pdata->codec_root) {
		entry = msm_snd_info_create_subdir(card->module, "codecs",
				card->proc_root);
		if (!entry) {
			pr_debug("%s: Cannot create codecs module entry\n",
				__func__);
			ret = 0;
			goto err;
		}
		pdata->codec_root = entry;
	}

	bolero_info_create_codec_entry(pdata->codec_root, bolero_component);
	bolero_register_wake_irq(bolero_component, false);

	component = snd_soc_rtdcom_lookup(rtd, WCD937X_DRV_NAME);
	if (!component)
		component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);

	if (!component) {
		pr_err("%s component is NULL\n", __func__);
		return -EINVAL;
	}

	dapm = snd_soc_component_get_dapm(component);
	card = component->card->snd_card;

	pdata = snd_soc_card_get_drvdata(component->card);
	if (!pdata)
		return -EINVAL;

	if (!pdata->codec_root) {
		entry = msm_snd_info_create_subdir(card->module, "codecs",
						 card->proc_root);
		if (!entry) {
			dev_dbg(component->dev, "%s: Cannot create codecs module entry\n",
				 __func__);
			ret = 0;
			goto mbhc_cfg_cal;
		}
		pdata->codec_root = entry;
	}

	if (!strncmp(component->driver->name, WCD937X_DRV_NAME, 13)) {
		wcd937x_info_create_codec_entry(pdata->codec_root, component);
		bolero_set_port_map(bolero_component,
			ARRAY_SIZE(sm_port_map),
			sm_port_map);
	}

	snd_soc_dapm_ignore_suspend(dapm, "EAR");
	snd_soc_dapm_ignore_suspend(dapm, "AUX");
	snd_soc_dapm_ignore_suspend(dapm, "LO");
	snd_soc_dapm_ignore_suspend(dapm, "HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC4");
	snd_soc_dapm_sync(dapm);

        codec_reg_done = true;
        msm_common_dai_link_init(rtd);
mbhc_cfg_cal:
	if (!strncmp(component->driver->name, WCD937X_DRV_NAME, 13)) {
		mbhc_calibration = def_wcd_mbhc_cal();
		if (!mbhc_calibration)
			return -ENOMEM;
		wcd_mbhc_cfg.calibration = mbhc_calibration;
		ret = wcd937x_mbhc_hs_detect(component, &wcd_mbhc_cfg);
	}

	if (ret) {
		dev_err(component->dev, "%s: mbhc hs detect failed, err:%d\n",
			__func__, ret);
		goto err_hs_detect;
	}
	return 0;

err_hs_detect:
	kfree(mbhc_calibration);
err:
	pr_err("%s:: return %d \n", __func__, ret);
	return ret;
}

static int msm_int_wsa_init(struct snd_soc_pcm_runtime *rtd)
{
	struct msm_asoc_mach_data *pdata =
		snd_soc_card_get_drvdata(rtd->card);

	int ret = 0;

	if (pdata->wsa_max_devs == 0)
		pr_info("%s: WSA is not enabled\n", __func__);

	msm_common_dai_link_init(rtd);

	return ret;
}

static int bengal_ssr_enable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (!card) {
		dev_err_ratelimited(dev, "%s: card is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	if (!strcmp(card->name, "bengal-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO\n", __func__);
	}

	snd_card_notify_user(SND_CARD_STATUS_ONLINE);
	dev_dbg(dev, "%s: setting snd_card to ONLINE\n", __func__);

err:
	return ret;
}

static void bengal_ssr_disable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		return;
	}

	dev_dbg(dev, "%s: setting snd_card to OFFLINE\n", __func__);
	snd_card_notify_user(SND_CARD_STATUS_OFFLINE);

	if (!strcmp(card->name, "bengal-stub-snd-card")) {
		/* TODO */
		dev_dbg(dev, "%s: TODO\n", __func__);
	}
}

static const struct snd_event_ops bengal_ssr_ops = {
	.enable = bengal_ssr_enable,
	.disable = bengal_ssr_disable,
};

static int msm_audio_ssr_compare(struct device *dev, void *data)
{
	struct device_node *node = data;

	dev_dbg(dev, "%s: dev->of_node = 0x%p, node = 0x%p\n",
		__func__, dev->of_node, node);
	return (dev->of_node && dev->of_node == node);
}

static int msm_audio_ssr_register(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct snd_event_clients *ssr_clients = NULL;
	struct device_node *node = NULL;
	int ret = 0;
	int i = 0;

	for (i = 0; ; i++) {
		node = of_parse_phandle(np, "qcom,msm_audio_ssr_devs", i);
		if (!node)
			break;
		snd_event_mstr_add_client(&ssr_clients,
					msm_audio_ssr_compare, node);
	}

	ret = snd_event_master_register(dev, &bengal_ssr_ops,
					ssr_clients, NULL);
	if (!ret)
		snd_event_notify(dev, SND_EVENT_UP);

	return ret;
}

struct msm_common_pdata *msm_common_get_pdata(struct snd_soc_card *card)
{
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return NULL;

	return pdata->common_pdata;
}

void msm_common_set_pdata(struct snd_soc_card *card,
                          struct msm_common_pdata *common_pdata)
{
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (!pdata)
		return;

	pdata->common_pdata = common_pdata;
}

static int msm_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = NULL;
	struct msm_asoc_mach_data *pdata = NULL;
	const char *mbhc_audio_jack_type = NULL;
	int ret = 0;
	uint index = 0;
	struct nvmem_cell *cell;
	size_t len;
	u32 *buf;
	u32 adsp_var_idx = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev,
			"%s: No platform supplied from device tree\n",
			__func__);
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_asoc_mach_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	card = populate_snd_card_dailinks(&pdev->dev);
	if (!card) {
		dev_err(&pdev->dev, "%s: Card uninitialized\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, pdata);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(&pdev->dev, "%s: parse card name failed, err:%d\n",
			__func__, ret);
		goto err;
	}

	ret = snd_soc_of_parse_audio_routing(card, "qcom,audio-routing");
	if (ret) {
		dev_err(&pdev->dev, "%s: parse audio routing failed, err:%d\n",
			__func__, ret);
		goto err;
	}

	ret = msm_populate_dai_link_component_of_node(card);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err;
	}

        /* Get maximum WSA device count for this platform */
	ret = of_property_read_u32(pdev->dev.of_node,
		"qcom,wsa-max-devs", &pdata->wsa_max_devs);
	if (ret) {
		dev_info(&pdev->dev,
		"%s: wsa-max-devs property missing in DT %s, ret = %d\n",
		__func__, pdev->dev.of_node->full_name, ret);
		pdata->wsa_max_devs = 0;
	}

	/* Make sure prefix string passed for each WSA device */
	ret = of_property_count_strings(pdev->dev.of_node,
					"qcom,wsa-aux-dev-prefix");
	if (!ret) {
		dev_err(&pdev->dev,
			"property %s not defined in DT\n",
			__func__, "qcom,wsa-aux-dev-prefix");
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret == -EPROBE_DEFER) {
		if (codec_reg_done)
			ret = -EINVAL;
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "%s: snd_soc_register_card failed (%d)\n",
			__func__, ret);
		goto err;
	}
	dev_info(&pdev->dev, "%s: Sound card %s registered\n",
		 __func__, card->name);

	pdata->hph_en1_gpio_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,hph-en1-gpio", 0);
	if (!pdata->hph_en1_gpio_p) {
		dev_dbg(&pdev->dev, "%s: property %s not detected in node %s\n",
			__func__, "qcom,hph-en1-gpio",
			pdev->dev.of_node->full_name);
	}

	pdata->hph_en0_gpio_p = of_parse_phandle(pdev->dev.of_node,
						"qcom,hph-en0-gpio", 0);
	if (!pdata->hph_en0_gpio_p) {
		dev_dbg(&pdev->dev, "%s: property %s not detected in node %s\n",
			__func__, "qcom,hph-en0-gpio",
			pdev->dev.of_node->full_name);
	}

	ret = of_property_read_string(pdev->dev.of_node,
		"qcom,mbhc-audio-jack-type", &mbhc_audio_jack_type);
	if (ret) {
		dev_dbg(&pdev->dev, "%s: Looking up %s property in node %s failed\n",
			__func__, "qcom,mbhc-audio-jack-type",
			pdev->dev.of_node->full_name);
		dev_dbg(&pdev->dev, "Jack type properties set to default\n");
	} else {
		if (!strcmp(mbhc_audio_jack_type, "4-pole-jack")) {
			wcd_mbhc_cfg.enable_anc_mic_detect = false;
			dev_dbg(&pdev->dev, "This hardware has 4 pole jack");
		} else if (!strcmp(mbhc_audio_jack_type, "5-pole-jack")) {
			wcd_mbhc_cfg.enable_anc_mic_detect = true;
			dev_dbg(&pdev->dev, "This hardware has 5 pole jack");
		} else if (!strcmp(mbhc_audio_jack_type, "6-pole-jack")) {
			wcd_mbhc_cfg.enable_anc_mic_detect = true;
			dev_dbg(&pdev->dev, "This hardware has 6 pole jack");
		} else {
			wcd_mbhc_cfg.enable_anc_mic_detect = false;
			dev_dbg(&pdev->dev, "Unknown value, set to default\n");
		}
	}
	/*
	 * Parse US-Euro gpio info from DT. Report no error if us-euro
	 * entry is not found in DT file as some targets do not support
	 * US-Euro detection
	 */
	pdata->us_euro_gpio_p = of_parse_phandle(pdev->dev.of_node,
					"qcom,us-euro-gpios", 0);
	if (!pdata->us_euro_gpio_p) {
		dev_dbg(&pdev->dev, "property %s not detected in node %s",
			"qcom,us-euro-gpios", pdev->dev.of_node->full_name);
	} else {
		dev_dbg(&pdev->dev, "%s detected\n",
			"qcom,us-euro-gpios");
		wcd_mbhc_cfg.swap_gnd_mic = msm_swap_gnd_mic;
	}

	if (wcd_mbhc_cfg.enable_usbc_analog)
		wcd_mbhc_cfg.swap_gnd_mic = msm_usbc_swap_gnd_mic;

	pdata->fsa_handle = of_parse_phandle(pdev->dev.of_node,
					"fsa4480-i2c-handle", 0);
	if (!pdata->fsa_handle)
		dev_dbg(&pdev->dev, "property %s not detected in node %s\n",
			"fsa4480-i2c-handle", pdev->dev.of_node->full_name);

	pdata->dmic01_gpio_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,cdc-dmic01-gpios",
					       0);
	pdata->dmic23_gpio_p = of_parse_phandle(pdev->dev.of_node,
					      "qcom,cdc-dmic23-gpios",
					       0);

	pdata->mi2s_gpio_p[PRIM_MI2S] = of_parse_phandle(pdev->dev.of_node,
					"qcom,pri-mi2s-gpios", 0);
	pdata->mi2s_gpio_p[SEC_MI2S] = of_parse_phandle(pdev->dev.of_node,
					"qcom,sec-mi2s-gpios", 0);
	pdata->mi2s_gpio_p[TERT_MI2S] = of_parse_phandle(pdev->dev.of_node,
					"qcom,tert-mi2s-gpios", 0);
	pdata->mi2s_gpio_p[QUAT_MI2S] = of_parse_phandle(pdev->dev.of_node,
					"qcom,quat-mi2s-gpios", 0);
	for (index = PRIM_MI2S; index < MI2S_MAX; index++)
		atomic_set(&(pdata->mi2s_gpio_ref_count[index]), 0);

	msm_common_snd_init(pdev, card);

	ret = msm_audio_ssr_register(&pdev->dev);
	if (ret)
		pr_err("%s: Registration with SND event FWK failed ret = %d\n",
			__func__, ret);

	is_initial_boot = true;
	/* get adsp variant idx */
	cell = nvmem_cell_get(&pdev->dev, "adsp_variant");
	if (IS_ERR_OR_NULL(cell)) {
		dev_dbg(&pdev->dev, "%s: FAILED to get nvmem cell \n", __func__);
		goto ret;
	}
	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR_OR_NULL(buf)) {
		dev_dbg(&pdev->dev, "%s: FAILED to read nvmem cell \n", __func__);
		goto ret;
	}
	if (len <= 0 || len > sizeof(u32)) {
		dev_dbg(&pdev->dev, "%s: nvmem cell length out of range: %d\n",
			__func__, len);
		kfree(buf);
		goto ret;
	}
	memcpy(&adsp_var_idx, buf, len);
	kfree(buf);
	va_disable = adsp_var_idx;

ret:
	return 0;
err:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int msm_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = NULL;
	struct msm_common_pdata *common_pdata = NULL;

	if (card)
		pdata = snd_soc_card_get_drvdata(card);

	if (pdata)
		common_pdata = pdata->common_pdata;

	msm_common_snd_deinit(common_pdata);
	snd_event_master_deregister(&pdev->dev);
	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver bengal_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = bengal_asoc_machine_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_asoc_machine_probe,
	.remove = msm_asoc_machine_remove,
};

static int __init msm_asoc_machine_init(void)
{
        snd_card_sysfs_init();
        return platform_driver_register(&bengal_asoc_machine_driver);
}
module_init(msm_asoc_machine_init);

static void __exit msm_asoc_machine_exit(void)
{
        platform_driver_unregister(&bengal_asoc_machine_driver);
}
module_exit(msm_asoc_machine_exit);

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, bengal_asoc_machine_of_match);
