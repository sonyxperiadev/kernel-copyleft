// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <dsp/audio_notifier.h>
#include <dsp/audio_prm.h>
#include <soc/snd_event.h>
#include "msm_dailink.h"
#include "msm_common.h"


#define DRV_NAME "spf-asoc-snd"
#define DRV_PINCTRL_NAME "audio-pcm-pinctrl"

#define __CHIPSET__ "SA8xx5 "
#define MSM_DAILINK_NAME(name) (__CHIPSET__#name)

#ifdef CONFIG_MSM_COUPLED_SSR
enum subsys_state {
	SUBSYS_DOWN = 0,
	SUBSYS_UP = 1
};

enum subsys_doamin {
	SUBSYS_DOMAIN_ADSP = 0,
	SUBSYS_DOMAIN_MDSP,
	SUBSYS_DOMAIN_MAX
};

static struct dsps_state_t {
	struct mutex lock;
	enum subsys_state states[SUBSYS_DOMAIN_MAX];
} dsps_state;
#endif


enum pinctrl_pin_state {
	STATE_SLEEP = 0, /* All pins are in sleep state */
	STATE_ACTIVE,  /* TDM = active */
};

struct msm_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *sleep;
	struct pinctrl_state *active;
	enum pinctrl_pin_state curr_state;
};

static const char *const pin_states[] = {"sleep", "active"};

static const char *const tdm_gpio_phandle[] = {"qcom,pri-tdm-gpios",
						"qcom,sec-tdm-gpios",
						"qcom,tert-tdm-gpios",
						"qcom,quat-tdm-gpios",
						"qcom,quin-tdm-gpios",
						"qcom,sen-tdm-gpios",
						"qcom,sep-tdm-gpios",
						"qcom,oct-tdm-gpios",
						"qcom,hsif0-tdm-gpios",
						"qcom,hsif1-tdm-gpios",
						"qcom,hsif2-tdm-gpios",
						};

static const char *const mclk_gpio_phandle[] = { "qcom,internal-mclk1-gpios" };

enum {
	TDM_PRI = 0,
	TDM_SEC,
	TDM_TERT,
	TDM_QUAT,
	TDM_QUIN,
	TDM_SEN,
	TDM_SEP,
	TDM_OCT,
	TDM_HSIF0,
	TDM_HSIF1,
	TDM_HSIF2,
	TDM_INTERFACE_MAX,
};

enum {
	IDX_PRIMARY_TDM_RX_0,
	IDX_PRIMARY_TDM_TX_0,
	IDX_SECONDARY_TDM_RX_0,
	IDX_SECONDARY_TDM_TX_0,
	IDX_TERTIARY_TDM_RX_0,
	IDX_TERTIARY_TDM_TX_0,
	IDX_QUATERNARY_TDM_RX_0,
	IDX_QUATERNARY_TDM_TX_0,
	IDX_QUINARY_TDM_RX_0,
	IDX_QUINARY_TDM_TX_0,
	IDX_SENARY_TDM_RX_0,
	IDX_SENARY_TDM_TX_0,
	IDX_SEPTENARY_TDM_RX_0,
	IDX_SEPTENARY_TDM_TX_0,
	IDX_OCTONARY_TDM_RX_0,
	IDX_OCTONARY_TDM_TX_0,
	IDX_HSIF0_TDM_RX_0,
	IDX_HSIF0_TDM_TX_0,
	IDX_HSIF1_TDM_RX_0,
	IDX_HSIF1_TDM_TX_0,
	IDX_HSIF2_TDM_RX_0,
	IDX_HSIF2_TDM_TX_0,
	IDX_HSIF3_TDM_RX_0,
	IDX_HSIF3_TDM_TX_0,
	IDX_HSIF4_TDM_RX_0,
	IDX_HSIF4_TDM_TX_0,
	IDX_QUATERNARY_TDM_RX_DUMMY_0,
	IDX_QUATERNARY_TDM_TX_DUMMY_0,
	IDX_QUINARY_TDM_RX_DUMMY_0,
	IDX_QUINARY_TDM_TX_DUMMY_0,
	IDX_GROUP_TDM_MAX,
};

enum msm_mclk_index {
	MCLK_NONE = -1,
	MCLK1 = 0,
	MCLK_MAX,
};

enum msm_mclk_status {
	MCLK_DISABLED = 0,
	MCLK_ENABLED,
};

struct msm_mclk_conf {
	struct mutex lock;
	enum msm_mclk_status mclk_status;
};

struct tdm_conf {
	struct mutex lock;
	u32 ref_cnt;
};

struct msm_asoc_mach_data {
	struct msm_common_pdata *common_pdata;
	struct tdm_conf tdm_intf_conf[TDM_INTERFACE_MAX];
	struct msm_pinctrl_info pinctrl_info[TDM_INTERFACE_MAX];
	bool mclk_used;
	struct msm_pinctrl_info mclk_pinctrl_info[MCLK_MAX];
	struct msm_mclk_conf mclk_conf[MCLK_MAX];
	struct snd_pcm_hardware hw_params;
};

static struct platform_device *spdev;

static struct clk_cfg internal_mclk[MCLK_MAX] = {
	{
		CLOCK_ID_MCLK_1,
		IBIT_CLOCK_12_P288_MHZ,
		CLOCK_ATTRIBUTE_COUPLE_NO,
		CLOCK_ROOT_DEFAULT,
	}
};

struct snd_soc_card sa8155_snd_soc_card_auto_msm = {
	.name = "sa8155-adp-star-snd-card",
};

struct snd_soc_card sa8295_snd_soc_card_auto_msm = {
	.name = "sa8295-adp-star-snd-card",
};

struct snd_soc_card sa8255_snd_soc_card_auto_msm = {
	.name = "sa8255-adp-star-snd-card",
};

struct snd_soc_card sa7255_snd_soc_card_auto_msm = {
	.name = "sa7255-adp-star-snd-card",
};

/* FIXME: it may various on different platform,
 * better to move to dt configuration in future
 */
static enum msm_mclk_index msm_get_mclk_index(int intf_idx)
{
	switch (intf_idx) {
	/* for sa8255 */
	case TDM_HSIF2:
		return MCLK1;

	default: return MCLK_NONE;
	}
}

static int msm_tdm_get_intf_idx(u16 id)
{
	switch (id) {
	case IDX_PRIMARY_TDM_RX_0:
	case IDX_PRIMARY_TDM_TX_0:
		return TDM_PRI;
	case IDX_SECONDARY_TDM_RX_0:
	case IDX_SECONDARY_TDM_TX_0:
		return TDM_SEC;
	case IDX_TERTIARY_TDM_RX_0:
	case IDX_TERTIARY_TDM_TX_0:
		return TDM_TERT;
	case IDX_QUATERNARY_TDM_RX_0:
	case IDX_QUATERNARY_TDM_TX_0:
	case IDX_QUATERNARY_TDM_RX_DUMMY_0:
	case IDX_QUATERNARY_TDM_TX_DUMMY_0:
		return TDM_QUAT;
	case IDX_QUINARY_TDM_RX_0:
	case IDX_QUINARY_TDM_TX_0:
	case IDX_QUINARY_TDM_RX_DUMMY_0:
	case IDX_QUINARY_TDM_TX_DUMMY_0:
		return TDM_QUIN;
	case IDX_SENARY_TDM_RX_0:
	case IDX_SENARY_TDM_TX_0:
		return TDM_SEN;
	case IDX_SEPTENARY_TDM_RX_0:
	case IDX_SEPTENARY_TDM_TX_0:
		return TDM_SEP;
	case IDX_OCTONARY_TDM_RX_0:
	case IDX_OCTONARY_TDM_TX_0:
		return TDM_OCT;
	case IDX_HSIF0_TDM_RX_0:
	case IDX_HSIF0_TDM_TX_0:
		return TDM_HSIF0;
	case IDX_HSIF1_TDM_RX_0:
	case IDX_HSIF1_TDM_TX_0:
		return TDM_HSIF1;
	case IDX_HSIF2_TDM_RX_0:
	case IDX_HSIF2_TDM_TX_0:
		return TDM_HSIF2;
	case IDX_HSIF3_TDM_RX_0:
	case IDX_HSIF3_TDM_TX_0:
		return TDM_SEC; //muxed
	case IDX_HSIF4_TDM_RX_0:
	case IDX_HSIF4_TDM_TX_0:
		return TDM_TERT; //muxed

	default: return -EINVAL;
	}
}

#ifdef CONFIG_MSM_COUPLED_SSR
static void set_subsys_state_l(enum subsys_doamin subsys,
				enum subsys_state state)
{
	dsps_state.states[subsys] = state;
}

static void set_combined_subsystem_state_l(enum subsys_state state)
{
	int i;

	for (i = 0; i < SUBSYS_DOMAIN_MAX; i++)
		dsps_state.states[i] = state;
}

static enum subsys_state get_combined_dsps_state_l(void)
{
	int i;

	for (i = 0; i < SUBSYS_DOMAIN_MAX; i++) {
		if (!dsps_state.states[i])
			return SUBSYS_DOWN;
	}
	return SUBSYS_UP;
}

static int modem_notifier_service_cb(struct notifier_block *this,
			   unsigned long opcode, void *data)
{
	pr_info("%s: modem pdr service opcode 0x%lx\n", __func__, opcode);
	switch (opcode) {
	case AUDIO_NOTIFIER_SERVICE_DOWN:
		mutex_lock(&dsps_state.lock);
		if (get_combined_dsps_state_l() == SUBSYS_UP) {
			set_combined_subsystem_state_l(SUBSYS_DOWN);
			pr_info("%s: setting snd_card to ONLINE\n", __func__);
			snd_card_notify_user(SND_CARD_STATUS_OFFLINE);
		}
		mutex_unlock(&dsps_state.lock);
		break;
	case AUDIO_NOTIFIER_SERVICE_UP:
		mutex_lock(&dsps_state.lock);
		set_subsys_state_l(SUBSYS_DOMAIN_MDSP, SUBSYS_UP);
		if (get_combined_dsps_state_l() == SUBSYS_UP) {
			pr_info("%s: setting snd_card to ONLINE\n", __func__);
			snd_card_notify_user(SND_CARD_STATUS_ONLINE);
		}
		mutex_unlock(&dsps_state.lock);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block modem_service_nb = {
	.notifier_call  = modem_notifier_service_cb,
	.priority = 0,
};
#endif

static int auto_adsp_ssr_enable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

#ifdef CONFIG_MSM_COUPLED_SSR
	mutex_lock(&dsps_state.lock);
	set_subsys_state_l(SUBSYS_DOMAIN_ADSP, SUBSYS_UP);
	if (get_combined_dsps_state_l() == SUBSYS_UP) {
		dev_dbg(dev, "%s: setting snd_card to ONLINE\n", __func__);
		snd_card_notify_user(SND_CARD_STATUS_ONLINE);
	}
	mutex_unlock(&dsps_state.lock);
#else
	dev_dbg(dev, "%s: setting snd_card to ONLINE\n", __func__);
	snd_card_notify_user(SND_CARD_STATUS_ONLINE);
#endif

err:
	return ret;
}

static void auto_adsp_ssr_disable(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if (!card) {
		dev_err(dev, "%s: card is NULL\n", __func__);
		return;
	}

#ifdef CONFIG_MSM_COUPLED_SSR
	mutex_lock(&dsps_state.lock);
	if (get_combined_dsps_state_l() == SUBSYS_UP) {
		set_combined_subsystem_state_l(SUBSYS_DOWN);
		dev_dbg(dev, "%s: setting snd_card to OFFLINE\n", __func__);
		snd_card_notify_user(SND_CARD_STATUS_OFFLINE);
	}
	mutex_unlock(&dsps_state.lock);
#else
	dev_dbg(dev, "%s: setting snd_card to OFFLINE\n", __func__);
	snd_card_notify_user(SND_CARD_STATUS_OFFLINE);
#endif
}

static const struct snd_event_ops auto_adsp_ssr_ops = {
	.enable = auto_adsp_ssr_enable,
	.disable = auto_adsp_ssr_disable,
};


static int msm_audio_ssr_compare(struct device *dev, void *data)
{
	struct device_node *node = data;

	dev_dbg(dev, "%s: dev->of_node = 0x%p, node = 0x%p\n",
		__func__, dev->of_node, node);
	return (dev->of_node && dev->of_node == node);
}

static int msm_audio_adsp_ssr_register(struct device *dev)
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

	ret = snd_event_master_register(dev, &auto_adsp_ssr_ops,
					ssr_clients, NULL);
	if (!ret)
		snd_event_notify(dev, SND_EVENT_UP);

	return ret;
}

static int msm_set_pinctrl(struct msm_pinctrl_info *pinctrl_info,
				enum pinctrl_pin_state new_state)
{
	int ret = 0;
	int curr_state = 0;

	if (pinctrl_info == NULL) {
		pr_err("%s: pinctrl info is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	if (pinctrl_info->pinctrl == NULL) {
		pr_err("%s: pinctrl handle is NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	curr_state = pinctrl_info->curr_state;
	pinctrl_info->curr_state = new_state;
	pr_debug("%s: curr_state = %s new_state = %s\n", __func__,
		pin_states[curr_state], pin_states[pinctrl_info->curr_state]);

	if (curr_state == pinctrl_info->curr_state) {
		pr_err("%s: pin already in same state\n", __func__);
		goto err;
	}

	if (curr_state != STATE_SLEEP &&
		pinctrl_info->curr_state != STATE_SLEEP) {
		pr_err("%s: pin state is already active, cannot switch\n",
			__func__);
		ret = -EIO;
		goto err;
	}
	switch (pinctrl_info->curr_state) {
	case STATE_ACTIVE:
		ret = pinctrl_select_state(pinctrl_info->pinctrl,
					pinctrl_info->active);
		if (ret) {
			pr_err("%s: state select to active failed with %d\n",
				__func__, ret);
			ret = -EIO;
			goto err;
		}
		break;
	case STATE_SLEEP:
		ret = pinctrl_select_state(pinctrl_info->pinctrl,
					pinctrl_info->sleep);
		if (ret) {
			pr_err("%s: state select to sleep failed with %d\n",
				__func__, ret);
			ret = -EIO;
			goto err;
		}
		break;
	default:
		pr_err("%s: pin state is invalid\n", __func__);
		return -EINVAL;
	}

err:
	return ret;
}


static int msm_mclk_disable(struct snd_soc_card *card,
		enum msm_mclk_index index)
{
	struct msm_asoc_mach_data *pdata = NULL;
	struct msm_pinctrl_info *pinctrl_info = NULL;
	int ret = 0;

	if (!card) {
		pr_err("%s: failed to get snd card!\n", __func__);
		return -EINVAL;
	}

	pdata = snd_soc_card_get_drvdata(card);
	if (!pdata) {
		pr_err("%s: no pdata\n", __func__);
		return -EINVAL;
	}
	if (!pdata->mclk_used) {
		pr_info("%s: mclk is not used\n", __func__);
		return 0;
	}

	mutex_lock(&pdata->mclk_conf[index].lock);
	pinctrl_info = &pdata->mclk_pinctrl_info[index];
	if (pinctrl_info && pinctrl_info->pinctrl) {
		ret = msm_set_pinctrl(pinctrl_info, STATE_SLEEP);
		if (ret != 0) {
			pr_err("%s: set pin state to sleep for mclk[%d], failed with %d\n",
				__func__, index, ret);
		}
		pinctrl_info->curr_state = STATE_SLEEP;
	}

	if (pdata->mclk_conf[index].mclk_status == MCLK_ENABLED) {
		ret = audio_prm_set_lpass_clk_cfg(&internal_mclk[index], 0);
		if (ret < 0) {
			pr_err("%s: audio_prm_set_lpass_clk_cfg failed to disable mclk[%d], err:%d\n",
				__func__, index, ret);
		}
		pdata->mclk_conf[index].mclk_status = MCLK_DISABLED;
	} else {
		pr_info("%s: mclk[%d] already disabled\n", __func__, index);
	}
	mutex_unlock(&pdata->mclk_conf[index].lock);
	return ret;
}

static int msm_mclk_enable(struct snd_soc_card *card,
		enum msm_mclk_index index)
{
	struct msm_asoc_mach_data *pdata = NULL;
	struct msm_pinctrl_info *pinctrl_info = NULL;
	int ret = 0;

	if (!card) {
		pr_err("%s: failed to get snd card!\n", __func__);
		return -EINVAL;
	}

	pdata = snd_soc_card_get_drvdata(card);
	if (!pdata) {
		pr_err("%s: no pdata\n", __func__);
		return -EINVAL;
	}
	if (!pdata->mclk_used) {
		pr_info("%s: mclk is not used\n", __func__);
		return 0;
	}

	mutex_lock(&pdata->mclk_conf[index].lock);
	if (pdata->mclk_conf[index].mclk_status == MCLK_DISABLED) {
		ret = audio_prm_set_lpass_clk_cfg(&internal_mclk[index], 1);
		if (ret < 0) {
			pr_err("%s: audio_prm_set_lpass_clk_cfg failed to enable mclk[%d], err:%d\n",
				__func__, index, ret);
		} else {
			pdata->mclk_conf[index].mclk_status = MCLK_ENABLED;
		}
	} else {
		pr_info("%s: mclk[%d] already enabled\n", __func__, index);
	}

	pinctrl_info = &pdata->mclk_pinctrl_info[index];
	if (pinctrl_info && pinctrl_info->pinctrl) {
		ret = msm_set_pinctrl(pinctrl_info, STATE_ACTIVE);
		if (ret != 0) {
			pr_err("%s: set pin state to active for mclk[%d], failed with %d\n",
				__func__, index, ret);
		}
		pinctrl_info->curr_state = STATE_ACTIVE;
	}
	mutex_unlock(&pdata->mclk_conf[index].lock);
	return ret;
}

static int tdm_snd_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct tdm_conf *intf_conf = NULL;
	struct msm_pinctrl_info *pinctrl_info = NULL;
	int ret_pinctrl = 0;
	int index, mclk_index;

	if (!dai_link->no_pcm)
		snd_soc_set_runtime_hwparams(substream, &pdata->hw_params);

	index = msm_tdm_get_intf_idx(dai_link->id);
	if (index < 0) {
		ret = -EINVAL;
		pr_err("%s: DAI link id (%d) out of range\n",
			__func__, dai_link->id);
		goto err;
	}

	if (pdata->mclk_used) {
		mclk_index = msm_get_mclk_index(index);
		if (mclk_index != MCLK_NONE)
			msm_mclk_enable(card, mclk_index);
	}
	/*
	 * Mutex protection in case the same TDM
	 * interface using for both TX and RX so
	 * that the same clock won't be enable twice.
	 */
	intf_conf = &pdata->tdm_intf_conf[index];
	mutex_lock(&intf_conf->lock);
	if (++intf_conf->ref_cnt == 1) {
		pinctrl_info = &pdata->pinctrl_info[index];
		if (pinctrl_info->pinctrl) {
			ret_pinctrl = msm_set_pinctrl(pinctrl_info,
						STATE_ACTIVE);
			if (ret_pinctrl)
				pr_err("%s: TDM TLMM pinctrl set failed with %d\n",
					__func__, ret_pinctrl);
		}
	}
	mutex_unlock(&intf_conf->lock);

err:
	return ret;
}

static void tdm_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_pinctrl_info *pinctrl_info = NULL;
	struct tdm_conf *intf_conf = NULL;
	int ret_pinctrl = 0;
	int index, mclk_index;

	pr_debug("%s: substream = %s, stream = %d\n", __func__,
		substream->name, substream->stream);

	index = msm_tdm_get_intf_idx(dai_link->id);
	if (index < 0) {
		pr_err("%s: DAI link id (%d) out of range\n",
			__func__, dai_link->id);
		return;
	}

	if (pdata->mclk_used) {
		mclk_index = msm_get_mclk_index(index);
		if (mclk_index != MCLK_NONE)
			msm_mclk_disable(card, mclk_index);
	}

	intf_conf = &pdata->tdm_intf_conf[index];
	mutex_lock(&intf_conf->lock);
	if (--intf_conf->ref_cnt == 0) {
		pinctrl_info = &pdata->pinctrl_info[index];
		if (pinctrl_info->pinctrl) {
			ret_pinctrl = msm_set_pinctrl(pinctrl_info,
							STATE_SLEEP);
			if (ret_pinctrl)
				pr_err("%s: TDM TLMM pinctrl set failed with %d\n",
					__func__, ret_pinctrl);
		}
	}
	mutex_unlock(&intf_conf->lock);
}

static const struct snd_soc_ops tdm_be_ops = {
	.startup = tdm_snd_startup,
	.shutdown = tdm_snd_shutdown
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm_common_dai_links[] = {
/* BackEnd DAI Links */
{
	.name = "LPASS_BE_AUXPCM_RX_DUMMY",
	.stream_name = "AUXPCM-LPAIF-RX-PRIMARY",
	.dpcm_playback = 1,
	.ops = &tdm_be_ops,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_PRIMARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(lpass_be_auxpcm_rx_dummy),
},
{
	.name = "LPASS_BE_AUXPCM_TX_DUMMY",
	.stream_name = "AUXPCM-LPAIF-TX-PRIMARY",
	.dpcm_capture = 1,
	.ops = &tdm_be_ops,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_PRIMARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(lpass_be_auxpcm_tx_dummy),
},
{
	.name = "SEC_TDM_RX_0",
	.stream_name = "TDM-LPAIF-RX-SECONDARY",
	.dpcm_playback = 1,
	.ops = &tdm_be_ops,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_SECONDARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(sec_tdm_rx_0),
},
{
	.name = "SEC_TDM_TX_0",
	.stream_name = "TDM-LPAIF-TX-SECONDARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_SECONDARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(sec_tdm_tx_0),
},
{
	.name = "TERT_TDM_RX_0",
	.stream_name = "TDM-LPAIF-RX-TERTIARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_TERTIARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(tert_tdm_rx_0),
},
{
	.name = "TERT_TDM_TX_0",
	.stream_name = "TDM-LPAIF-TX-TERTIARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_TERTIARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(tert_tdm_tx_0),
},
{
	.name = "QUAT_TDM_RX_0",
	.stream_name = "TDM-LPAIF_RXTX-RX-PRIMARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUATERNARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(quat_tdm_rx_0),
},
{
	.name = "QUAT_TDM_TX_0",
	.stream_name = "TDM-LPAIF_RXTX-TX-PRIMARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUATERNARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(quat_tdm_tx_0),
},
{
	.name = "QUIN_TDM_RX_0",
	.stream_name = "TDM-LPAIF_VA-RX-PRIMARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUINARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(quin_tdm_rx_0),
},
{
	.name = "QUIN_TDM_TX_0",
	.stream_name = "TDM-LPAIF_VA-TX-PRIMARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUINARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(quin_tdm_tx_0),
},
{
	.name = "SEN_TDM_RX_0",
	.stream_name = "TDM-LPAIF_WSA-RX-PRIMARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_SENARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(sen_tdm_rx_0),
},
{
	.name = "SEN_TDM_TX_0",
	.stream_name = "TDM-LPAIF_WSA-TX-PRIMARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_SENARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(sen_tdm_tx_0),
},
{
	.name = "SEP_TDM_RX_0",
	.stream_name = "TDM-LPAIF_AUD-RX-PRIMARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_SEPTENARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(sep_tdm_rx_0),
},
{
	.name = "SEP_TDM_TX_0",
	.stream_name = "TDM-LPAIF_AUD-TX-PRIMARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_SEPTENARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(sep_tdm_tx_0),
},
{
	.name = "OCT_TDM_RX_0",
	.stream_name = "TDM-LPAIF_WSA2-RX-PRIMARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_OCTONARY_TDM_RX_0,
	SND_SOC_DAILINK_REG(oct_tdm_rx_0),
},
{
	.name = "OCT_TDM_TX_0",
	.stream_name = "TDM-LPAIF_WSA2-TX-PRIMARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_OCTONARY_TDM_TX_0,
	SND_SOC_DAILINK_REG(oct_tdm_tx_0),
},
{
	.name = "HS_IF0_TDM_RX_0",
	.stream_name = "TDM-LPAIF_SDR-RX-PRIMARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
		.id = IDX_HSIF0_TDM_RX_0,
	SND_SOC_DAILINK_REG(hs_if0_tdm_rx_0),
},
{
	.name = "HS_IF0_TDM_TX_0",
	.stream_name = "TDM-LPAIF_SDR-TX-PRIMARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF0_TDM_TX_0,
	SND_SOC_DAILINK_REG(hs_if0_tdm_tx_0),
},
{
	.name = "HS_IF1_TDM_RX_0",
	.stream_name = "TDM-LPAIF_SDR-RX-SECONDARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF1_TDM_RX_0,
	SND_SOC_DAILINK_REG(hs_if1_tdm_rx_0),
},
{
	.name = "HS_IF1_TDM_TX_0",
	.stream_name = "TDM-LPAIF_SDR-TX-SECONDARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF1_TDM_TX_0,
	SND_SOC_DAILINK_REG(hs_if1_tdm_tx_0),
},
{
	.name = "HS_IF2_TDM_RX_0",
	.stream_name = "TDM-LPAIF_SDR-RX-TERTIARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF2_TDM_RX_0,
	SND_SOC_DAILINK_REG(hs_if2_tdm_rx_0),
},
{
	.name = "HS_IF2_TDM_TX_0",
	.stream_name = "TDM-LPAIF_SDR-TX-TERTIARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF2_TDM_TX_0,
	SND_SOC_DAILINK_REG(hs_if2_tdm_tx_0),
},
{
	.name = "HS_IF3_TDM_RX_0",
	.stream_name = "TDM-LPAIF_SDR-RX-QUATERNARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF3_TDM_RX_0,
	SND_SOC_DAILINK_REG(hs_if3_tdm_rx_0),
},
{
	.name = "HS_IF3_TDM_TX_0",
	.stream_name = "TDM-LPAIF_SDR-TX-QUATERNARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF3_TDM_TX_0,
	SND_SOC_DAILINK_REG(hs_if3_tdm_tx_0),
},
{
	.name = "HS_IF4_TDM_RX_0",
	.stream_name = "TDM-LPAIF_SDR-RX-QUINARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF4_TDM_RX_0,
	SND_SOC_DAILINK_REG(hs_if4_tdm_rx_0),
},
{
	.name = "HS_IF4_TDM_TX_0",
	.stream_name = "TDM-LPAIF_SDR-TX-QUINARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_HSIF4_TDM_TX_0,
	SND_SOC_DAILINK_REG(hs_if4_tdm_tx_0),
},
{
	.name = "QUAT_TDM_RX_0_DUMMY",
	.stream_name = "TDM-LPAIF-RX-QUATERNARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUATERNARY_TDM_RX_DUMMY_0,
	SND_SOC_DAILINK_REG(quat_tdm_rx_0_dummy),
},
{
	.name = "QUAT_TDM_TX_0_DUMMY",
	.stream_name = "TDM-LPAIF-TX-QUATERNARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUATERNARY_TDM_TX_DUMMY_0,
	SND_SOC_DAILINK_REG(quat_tdm_tx_0_dummy),
},
{
	.name = "QUIN_TDM_RX_0_DUMMY",
	.stream_name = "TDM-LPAIF-RX-QUINARY",
	.dpcm_playback = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUINARY_TDM_RX_DUMMY_0,
	SND_SOC_DAILINK_REG(quin_tdm_rx_0_dummy),
},
{
	.name = "QUIN_TDM_TX_0_DUMMY",
	.stream_name = "TDM-LPAIF-TX-QUINARY",
	.dpcm_capture = 1,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
	.ops = &tdm_be_ops,
	.ignore_suspend = 1,
	.ignore_pmdown_time = 1,
	.id = IDX_QUINARY_TDM_TX_DUMMY_0,
	SND_SOC_DAILINK_REG(quin_tdm_tx_0_dummy),
},
};


static int msm_populate_dai_link_component_of_node(
					struct snd_soc_card *card)
{
	int i, j, index, ret = 0;
	struct device *cdev = card->dev;
	struct snd_soc_dai_link *dai_link = card->dai_link;
	struct device_node *np;

	if (!cdev) {
		pr_err("%s: Sound card device memory NULL\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < card->num_links; i++) {
		if (dai_link[i].platforms->of_node && dai_link[i].cpus->of_node)
			continue;

	/* populate platform_of_node for snd card dai links
	 * Skip this part for dummy snd card
	 */
	if (0) {
		if (dai_link[i].platforms->name &&
			!dai_link[i].platforms->of_node) {
			index = of_property_match_string(cdev->of_node,
						"asoc-platform-names",
						dai_link[i].platforms->name);
			if (index < 0) {
				pr_err("%s: No match found for platform name: %s\n index: %i cdev_of_node: %s\n",
					__func__,
					dai_link[i].platforms->name,
					index,
					cdev->of_node);
				ret = index;
				goto err;
			}
			np = of_parse_phandle(cdev->of_node, "asoc-platform",
						index);
			if (!np) {
				pr_err("%s: retrieving phandle for platform %s, index %d failed\n",
					__func__, dai_link[i].platforms->name,
					index);
				ret = -ENODEV;
				goto err;
			}
			dai_link[i].platforms->of_node = np;
			dai_link[i].platforms->name = NULL;
		}
	}


		/* populate cpu_of_node for snd card dai links */
		if (dai_link[i].cpus->dai_name && !dai_link[i].cpus->of_node) {
			index = of_property_match_string(cdev->of_node,
						 "asoc-cpu-names",
						 dai_link[i].cpus->dai_name);
			pr_err("%s: retrieving cpu_of_node for %s\n",
						__func__,
						dai_link[i].cpus->dai_name);
			if (index >= 0) {
				np = of_parse_phandle(cdev->of_node, "asoc-cpu",
						index);
				if (!np) {
					pr_err("%s: retrieving phandle for cpu dai %s failed\n",
						__func__,
						dai_link[i].cpus->dai_name);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].cpus->of_node = np;
				dai_link[i].cpus->dai_name = NULL;
			}
		}

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
				np = of_parse_phandle(cdev->of_node,
						      "asoc-codec",
						      index);
				if (!np) {
					pr_err("%s: retrieving phandle for codec %s failed\n",
						__func__,
						dai_link[i].codecs[j].name);
					ret = -ENODEV;
					goto err;
				}
				dai_link[i].codecs[j].of_node = np;
				dai_link[i].codecs[j].name = NULL;
			}
		}
	}

err:
	return ret;
}

static const struct of_device_id asoc_machine_of_match[]  = {
	{ .compatible = "qcom,sa8295-asoc-snd-adp-star",
	  .data = "adp_star_codec"},
	{ .compatible = "qcom,sa8155-asoc-snd-adp-star",
	  .data = "adp_star_codec"},
	{ .compatible = "qcom,sa8255-asoc-snd-adp-star",
	  .data = "adp_star_codec"},
	{ .compatible = "qcom,sa7255-asoc-snd-adp-star",
	  .data = "adp_star_codec"},
	{},
};

static const struct of_device_id audio_pinctrl_dummy_match[] = {
	{ .compatible = "qcom,msm-pcm-pinctrl" },
	{ },
};

static struct snd_soc_dai_link msm_auto_dai_links[
			 ARRAY_SIZE(msm_common_dai_links)];

static struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *dailink;
	int total_links;
	const struct of_device_id *match;

	match = of_match_node(asoc_machine_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "%s: No DT match found for sound card\n",
			__func__);
		return NULL;
	}

	if (!strcmp(match->compatible, "qcom,sa8155-asoc-snd-adp-star"))
		card = &sa8155_snd_soc_card_auto_msm;
	else if (!strcmp(match->compatible, "qcom,sa8295-asoc-snd-adp-star"))
		card = &sa8295_snd_soc_card_auto_msm;
	else if (!strcmp(match->compatible, "qcom,sa8255-asoc-snd-adp-star"))
		card = &sa8255_snd_soc_card_auto_msm;
	else if (!strcmp(match->compatible, "qcom,sa7255-asoc-snd-adp-star"))
		card = &sa7255_snd_soc_card_auto_msm;

	total_links = ARRAY_SIZE(msm_common_dai_links);
	memcpy(msm_auto_dai_links,
		msm_common_dai_links,
		sizeof(msm_common_dai_links));
	dailink = msm_auto_dai_links;

	if (card) {
		card->dai_link = dailink;
		card->num_links = total_links;
	}

	return card;
}

static int msm_get_hwparams(struct platform_device *pdev)
{
	struct snd_soc_card *card = NULL;
	struct msm_asoc_mach_data *pdata = NULL;
	u32 pcm_info = 0;
	u32 buffer_bytes_max = 0;
	u32 periods_bytes[2] = {0};
	u32 periods_count[2] = {0};
	int ret = 0;

	card = platform_get_drvdata(pdev);
	if (!card) {
		pr_err("%s: card is NULL\n",
			__func__);
		return -EINVAL;
	}
	pdata = snd_soc_card_get_drvdata(card);
	if (!pdata) {
		pr_err("%s: pdata is NULL\n",
			__func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			"qcom,hw_pcm_info",
			&pcm_info);
	if (ret) {
		pr_err("%s: read pcm info failed\n",
			__func__);
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			"qcom,hw_buffer_size_max",
			&buffer_bytes_max);
	if (ret) {
		pr_err("%s: read buffer size max failed\n",
			__func__);
		return ret;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,hw_period_byte_size",
			periods_bytes, 2);
	if (ret) {
		pr_err("%s: read period byte size failed\n",
			__func__);
		return ret;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,hw_period_count_size",
			periods_count, 2);
	if (ret) {
		pr_err("%s: read period count size failed\n",
			__func__);
		return ret;
	}

	pdata->hw_params.info = pcm_info;
	pdata->hw_params.buffer_bytes_max = buffer_bytes_max;
	pdata->hw_params.period_bytes_min = periods_bytes[0];
	pdata->hw_params.period_bytes_max = periods_bytes[1];
	pdata->hw_params.periods_min = periods_count[0];
	pdata->hw_params.periods_max = periods_count[1];

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

/*****************************************************************************
 * pinctrl
 *****************************************************************************/
static void msm_release_pinctrl(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_pinctrl_info *pinctrl_info = NULL;
	int i;

	for (i = TDM_PRI; i < TDM_INTERFACE_MAX; i++) {
		pinctrl_info = &pdata->pinctrl_info[i];
		if (pinctrl_info == NULL)
			continue;
		if (pinctrl_info->pinctrl) {
			devm_pinctrl_put(pinctrl_info->pinctrl);
			pinctrl_info->pinctrl = NULL;
		}
	}
}

static void msm_release_mclk_pinctrl(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_pinctrl_info *pinctrl_info = NULL;
	int i;

	for (i = 0; i < MCLK_MAX; i++) {
		pinctrl_info = &pdata->mclk_pinctrl_info[i];
		if (pinctrl_info == NULL)
			continue;
		if (pinctrl_info->pinctrl) {
			devm_pinctrl_put(pinctrl_info->pinctrl);
			pinctrl_info->pinctrl = NULL;
		}
	}
}

static int msm_get_pinctrl(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_pinctrl_info *pinctrl_info = NULL;
	struct pinctrl *pinctrl = NULL;
	int i, j;
	struct device_node *np = NULL;
	struct platform_device *pdev_np = NULL;
	int ret = 0;

	for (i = TDM_PRI; i < TDM_INTERFACE_MAX; i++) {
		np = of_parse_phandle(pdev->dev.of_node,
					tdm_gpio_phandle[i], 0);
		if (!np) {
			pr_debug("%s: device node %s is null\n",
				__func__, tdm_gpio_phandle[i]);
			continue;
		}

		pdev_np = of_find_device_by_node(np);
		if (!pdev_np) {
			pr_err("%s: platform device not found\n",
				__func__);
			continue;
		}

		pinctrl_info = &pdata->pinctrl_info[i];
		if (pinctrl_info == NULL) {
			pr_err("%s: pinctrl info is null\n",
				__func__);
			continue;
		}

		pinctrl = devm_pinctrl_get(&pdev_np->dev);
		if (IS_ERR_OR_NULL(pinctrl)) {
			pr_err("%s: fail to get pinctrl handle\n",
				__func__);
			goto err;
		}
		pinctrl_info->pinctrl = pinctrl;

		/* get all the states handles from Device Tree */
		pinctrl_info->sleep = pinctrl_lookup_state(pinctrl,
							"default");
		if (IS_ERR(pinctrl_info->sleep)) {
			pr_err("%s: could not get sleep pin state\n",
				__func__);
			goto err;
		}
		pinctrl_info->active = pinctrl_lookup_state(pinctrl,
							"active");
		if (IS_ERR(pinctrl_info->active)) {
			pr_err("%s: could not get active pin state\n",
				__func__);
			goto err;
		}

		/* Reset the TLMM pins to a sleep state */
		ret = pinctrl_select_state(pinctrl_info->pinctrl,
						pinctrl_info->sleep);
		if (ret != 0) {
			pr_err("%s: set pin state to sleep failed with %d\n",
				__func__, ret);
			ret = -EIO;
			goto err;
		}
		pinctrl_info->curr_state = STATE_SLEEP;
	}
	return 0;

err:
	for (j = i; j >= 0; j--) {
		pinctrl_info = &pdata->pinctrl_info[j];
		if (pinctrl_info == NULL)
			continue;
		if (pinctrl_info->pinctrl) {
			devm_pinctrl_put(pinctrl_info->pinctrl);
			pinctrl_info->pinctrl = NULL;
		}
	}
	return -EINVAL;
}

static int msm_get_mclk_pinctrl(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_pinctrl_info *pinctrl_info = NULL;
	struct pinctrl *pinctrl = NULL;
	int i, j;
	struct device_node *np = NULL;
	struct platform_device *pdev_np = NULL;
	int ret = 0;

	for (i = 0; i < MCLK_MAX; i++) {

		np = of_parse_phandle(pdev->dev.of_node,
			mclk_gpio_phandle[i],
			0);
		if (!np) {
			pr_err("%s: device node %s is null\n",
				__func__, mclk_gpio_phandle[i]);
			continue;
		}
		pdev_np = of_find_device_by_node(np);
		if (!pdev_np) {
			pr_err("%s: platform device not found\n",
				__func__);
			continue;
		}

		pinctrl_info = &pdata->mclk_pinctrl_info[i];
		if (pinctrl_info == NULL) {
			pr_err("%s: pinctrl info is null\n",
				__func__);
			continue;
		}

		pinctrl = devm_pinctrl_get(&pdev_np->dev);
		if (IS_ERR_OR_NULL(pinctrl)) {
			pr_err("%s: fail to get pinctrl handle\n",
				__func__);
			goto err;
		}
		pinctrl_info->pinctrl = pinctrl;
		/* get all the states handles from Device Tree */
		pinctrl_info->sleep = pinctrl_lookup_state(pinctrl,
			"default");
		if (IS_ERR(pinctrl_info->sleep)) {
			pr_err("%s: could not get sleep pin state\n",
				__func__);
			goto err;
		}
		pinctrl_info->active = pinctrl_lookup_state(pinctrl,
			"active");
		if (IS_ERR(pinctrl_info->active)) {
			pr_err("%s: could not get active pin state\n",
				__func__);
			goto err;
		}

		/* Reset the mclk pins to a sleep state */
		ret = pinctrl_select_state(pinctrl_info->pinctrl,
						pinctrl_info->sleep);
		if (ret != 0) {
			pr_err("%s: set pin state to sleep failed with %d\n",
				__func__, ret);
			ret = -EIO;
			goto err;
		}
		pinctrl_info->curr_state = STATE_SLEEP;
	}
	return 0;

err:
	for (j = i; j >= 0; j--) {
		pinctrl_info = &pdata->mclk_pinctrl_info[i];
		if (pinctrl_info == NULL)
			continue;
		if (pinctrl_info->pinctrl) {
			devm_pinctrl_put(pinctrl_info->pinctrl);
			pinctrl_info->pinctrl = NULL;
		}
	}
	return -EINVAL;
}

static int msm_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = NULL;
	struct msm_asoc_mach_data *pdata = NULL;
	int ret = 0;
	const struct of_device_id *match = NULL;

	dev_err(&pdev->dev, "%s: audio_reach\n",
		__func__);

	match = of_match_node(asoc_machine_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "%s: No DT match found for sound card\n",
			__func__);
		return -EINVAL;
	}

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform supplied from device tree\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_asoc_mach_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	card = populate_snd_card_dailinks(&pdev->dev);
	if (!card) {
		dev_err(&pdev->dev, "%s: Card uninitialized\n",
			__func__);
		ret = -EINVAL;
		goto err;
	}
	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, pdata);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(&pdev->dev, "parse card name failed, err:%d\n",
			ret);
		pr_err("%s: parse card name failed with err:%d\n",
			__func__, ret);
		goto err;
	}

	ret = msm_populate_dai_link_component_of_node(card);
	if (ret) {
		ret = -EPROBE_DEFER;
		goto err;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);

	if (ret == -EPROBE_DEFER) {
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		pr_err("snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	msm_common_snd_init(pdev, card);

	/* Parse pinctrl info from devicetree */
	ret = msm_get_pinctrl(pdev);
	if (!ret) {
		pr_err("%s: pinctrl parsing successful\n",
			__func__);
	} else {
		dev_dbg(&pdev->dev,
			"%s: pinctrl parsing failed with %d\n",
			__func__, ret);
		ret = 0;
	}

	pdata->mclk_used = false;
	if (strnstr(match->compatible, "sa8295",
			sizeof(match->compatible)) ||
		strnstr(match->compatible, "sa8255",
			sizeof(match->compatible)) ||
		strnstr(match->compatible, "sa7255",
			sizeof(match->compatible))) {
		/* get mclk pinctrl info from devicetree */
		ret = msm_get_mclk_pinctrl(pdev);
		if (!ret) {
			pr_debug("%s: pinctrl mclk parsing successful\n",
				__func__);
			pdata->mclk_used = true;
		} else {
			dev_err(&pdev->dev,
				"%s: pinctrl mclk parsing failed with %d\n",
				__func__, ret);
			ret = 0;
		}
	}

	ret = msm_get_hwparams(pdev);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: hwparams get failed with %d\n",
			__func__, ret);
		goto err;
	}

	dev_info(&pdev->dev, "Sound card %s registered\n",
		card->name);
	pr_err("Sound card %s registered\n",
		card->name);
	spdev = pdev;

#ifdef CONFIG_MSM_COUPLED_SSR
	mutex_init(&dsps_state.lock);
	ret = audio_notifier_register("auto_modem",
					AUDIO_NOTIFIER_MODEM_ROOT_DOMAIN,
					&modem_service_nb);
	if (ret < 0)
		pr_err("%s: Registration with modem PDR failed ret = %d\n",
			__func__, ret);
#endif

	ret = msm_audio_adsp_ssr_register(&pdev->dev);
	if (ret)
		pr_err("%s: Registration with SND event FWK failed ret = %d\n",
			__func__, ret);

	snd_card_notify_user(SND_CARD_STATUS_ONLINE);
	return 0;
err:
	msm_release_mclk_pinctrl(pdev);
	msm_release_pinctrl(pdev);
	return ret;
}

static int msm_asoc_machine_remove(struct platform_device *pdev)
{
	msm_release_mclk_pinctrl(pdev);
	msm_release_pinctrl(pdev);
	snd_event_master_deregister(&pdev->dev);
#ifdef CONFIG_MSM_COUPLED_SSR
	audio_notifier_deregister("auto_modem");
	mutex_destroy(&dsps_state.lock);
#endif
	return 0;
}

static int audio_pinctrl_dummy_probe(struct platform_device *pdev)
{
	return 0;
}

static int audio_pinctrl_dummy_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver audio_pinctrl_dummy_driver = {
	.driver = {
		.name = DRV_PINCTRL_NAME,
		.of_match_table = audio_pinctrl_dummy_match,
		.suppress_bind_attrs = true,
	},
	.probe = audio_pinctrl_dummy_probe,
	.remove = audio_pinctrl_dummy_remove,
};

static struct platform_driver asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &snd_soc_pm_ops,
		.of_match_table = asoc_machine_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_asoc_machine_probe,
	.remove = msm_asoc_machine_remove,
};

int __init auto_spf_init(void)
{
	snd_card_sysfs_init();
	platform_driver_register(&audio_pinctrl_dummy_driver);
	return platform_driver_register(&asoc_machine_driver);
}

void auto_spf_exit(void)
{
	platform_driver_unregister(&audio_pinctrl_dummy_driver);
	platform_driver_unregister(&asoc_machine_driver);
}

module_init(auto_spf_init);
module_exit(auto_spf_exit);

MODULE_DESCRIPTION("ALSA SoC Machine Driver for SPF");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_ALIAS("platform:" DRV_PINCTRL_NAME);
MODULE_DEVICE_TABLE(of, asoc_machine_of_match);
MODULE_DEVICE_TABLE(of, audio_pinctrl_dummy_match);
