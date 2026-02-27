// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "btfm_swr.h"
#include "btfm_swr_hw_interface.h"
#include "btfm_codec_hw_interface.h"

#define LPAIF_AUD     0x05

static int bt_soc_enable_status;
int btfm_feedback_ch_setting;
static uint8_t usecase_codec;

static int btfm_swr_hwep_write(struct snd_soc_component *codec,
			unsigned int reg, unsigned int value)
{
	BTFMSWR_DBG("");
	return 0;
}

static unsigned int btfm_swr_hwep_read(struct snd_soc_component *codec,
				unsigned int reg)
{
	BTFMSWR_DBG("");
	return 0;
}

static int btfm_soc_status_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	BTFMSWR_DBG("");
	ucontrol->value.integer.value[0] = bt_soc_enable_status;
	return 1;
}

static int btfm_soc_status_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	BTFMSWR_DBG("");
	return 1;
}

static int btfm_get_feedback_ch_setting(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	BTFMSWR_DBG("");
	ucontrol->value.integer.value[0] = btfm_feedback_ch_setting;
	return 1;
}

static int btfm_put_feedback_ch_setting(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	BTFMSWR_DBG("");
	btfm_feedback_ch_setting = ucontrol->value.integer.value[0];
	return 1;
}

static int btfm_get_codec_type(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	BTFMSWR_DBG("current codec type:%s", codec_text[usecase_codec]);
	ucontrol->value.integer.value[0] = usecase_codec;
	return 1;
}

static int btfm_put_codec_type(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	usecase_codec = ucontrol->value.integer.value[0];
	BTFMSWR_DBG("codec type set to:%s", codec_text[usecase_codec]);
	return 1;
}

static struct snd_kcontrol_new status_controls[] = {
	SOC_SINGLE_EXT("BT SOC status", 0, 0, 1, 0,
	btfm_soc_status_get, btfm_soc_status_put),
	SOC_SINGLE_EXT("BT set feedback channel", 0, 0, 1, 0,
	btfm_get_feedback_ch_setting,
	btfm_put_feedback_ch_setting),
	SOC_ENUM_EXT("BT codec type", codec_display,
	btfm_get_codec_type, btfm_put_codec_type),
};


static int btfm_swr_hwep_probe(struct snd_soc_component *codec)
{
	BTFMSWR_DBG("");
	return 0;
}

static void btfm_swr_hwep_remove(struct snd_soc_component *codec)
{
	BTFMSWR_DBG("");
}

static int btfm_swr_dai_startup(void *dai)
{
	//struct hwep_data *hwep_info = (struct hwep_data *)dai;
	int ret = -1;

	BTFMSWR_DBG("");

	ret = btfm_swr_hw_init();
	return ret;
}

static void btfm_swr_dai_shutdown(void *dai, int id)
{
	struct hwep_data *hwep_info = (struct hwep_data *)dai;
	struct btfmswr *btfmswr = dev_get_drvdata(hwep_info->dev);
	int ret = 0;
	u8 port_type;

	BTFMSWR_INFO("");

	if (btfmswr == NULL)
		BTFMSWR_INFO("btfmswr is NULL\n");

	switch (id) {
	case FMAUDIO_TX:
		port_type = FM_AUDIO_TX1;
		break;
	case BTAUDIO_TX:
		port_type = BT_AUDIO_TX1;
		break;
	case BTAUDIO_RX:
		port_type = BT_AUDIO_RX1;
		break;
	case BTAUDIO_A2DP_SINK_TX:
		port_type = BT_AUDIO_TX2;
		break;
	case BTFM_NUM_CODEC_DAIS:
	default:
		BTFMSWR_ERR("dai->id is invalid:%d", id);
		return;
	}

	ret = btfm_swr_disable_port(btfmswr->p_dai_port->port_info[id].port,
				    btfmswr->num_channels, port_type);
}

static int btfm_swr_dai_hw_params(void *dai, uint32_t bps,
				   uint32_t direction, uint8_t num_channels)
{
	struct hwep_data *hwep_info = (struct hwep_data *)dai;
	struct btfmswr *btfmswr = dev_get_drvdata(hwep_info->dev);

	BTFMSWR_DBG("");
	btfmswr->bps = bps;
	btfmswr->direction = direction;
	btfmswr->num_channels = num_channels;

	return 0;
}

void btfm_get_sampling_rate(uint32_t *sampling_rate)
{
	uint8_t codec_types_avb = ARRAY_SIZE(codec_text);

	if (usecase_codec > (codec_types_avb - 1)) {
		BTFMSWR_ERR("falling back to use default sampling_rate: %u",
				*sampling_rate);
		return;
	}

	if (*sampling_rate == 44100 || *sampling_rate == 48000) {
		if (usecase_codec == LDAC ||
		    usecase_codec == APTX_AD)
			*sampling_rate = (*sampling_rate) * 2;
	}

	if (usecase_codec == LC3_VOICE ||
	    usecase_codec == APTX_AD_SPEECH ||
	    usecase_codec == LC3 || usecase_codec == APTX_AD_QLEA ||
	    usecase_codec == APTX_AD_R4) {
		*sampling_rate = 96000;
	}

	BTFMSWR_INFO("current usecase codec type %s and sampling rate:%u khz",
			codec_text[usecase_codec], *sampling_rate);
}

static int btfm_swr_dai_prepare(void *dai, uint32_t sampling_rate, uint32_t direction, int id)
{
	struct hwep_data *hwep_info = (struct hwep_data *)dai;
	struct btfmswr *btfmswr = dev_get_drvdata(hwep_info->dev);
	int ret = -EINVAL;
	u8 port_type;

	bt_soc_enable_status = 0;
	BTFMSWR_INFO("dai->id: %d, dai->rate: %d direction: %d", id, sampling_rate, direction);

	btfm_get_sampling_rate(&sampling_rate);
	btfmswr->sample_rate = sampling_rate;

	switch (id) {
	case FMAUDIO_TX:
		port_type = FM_AUDIO_TX1;
		break;
	case BTAUDIO_TX:
		port_type = BT_AUDIO_TX1;
		break;
	case BTAUDIO_RX:
		port_type = BT_AUDIO_RX1;
		break;
	case BTAUDIO_A2DP_SINK_TX:
		port_type = BT_AUDIO_TX2;
		break;
	case BTFM_NUM_CODEC_DAIS:
	default:
		BTFMSWR_ERR("dai->id is invalid:%d", id);
		return -EINVAL;
	}

	ret = btfm_swr_enable_port(btfmswr->p_dai_port->port_info[id].port,
				   btfmswr->num_channels, sampling_rate, port_type);

	/* save the enable channel status */
	if (ret == 0)
		bt_soc_enable_status = 1;

	if (ret == -EISCONN) {
		BTFMSWR_ERR("channel opened without closing, returning success");
		ret = 0;
	}

	return ret;
}

/* This function will be called once during boot up */
static int btfm_swr_dai_set_channel_map(void *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	BTFMSWR_DBG("");
	return 0;
}

static int btfm_swr_dai_get_channel_map(void *dai,
				 unsigned int *tx_num, unsigned int *tx_slot,
				 unsigned int *rx_num, unsigned int *rx_slot, int id)
{

	struct hwep_data *hwep_info = (struct hwep_data *)dai;
	struct btfmswr *btfmswr = dev_get_drvdata(hwep_info->dev);

	*rx_slot = 0;
	*tx_slot = 0;
	*rx_num = 0;
	*tx_num = 0;

	switch (id) {
	case FMAUDIO_TX:
	case BTAUDIO_TX:
	case BTAUDIO_A2DP_SINK_TX:
		*tx_num = btfmswr->num_channels;
		*tx_slot = btfmswr->num_channels == 2 ? TWO_CHANNEL_MASK : ONE_CHANNEL_MASK;
		break;
	case BTAUDIO_RX:
		*rx_num = btfmswr->num_channels;
		*rx_slot = btfmswr->num_channels == 2 ? TWO_CHANNEL_MASK : ONE_CHANNEL_MASK;
		break;

	default:
		BTFMSWR_ERR("Unsupported DAI %d", id);
		return -EINVAL;
	}

	return 0;
}

int btfm_swr_dai_get_configs(void *dai, void *config, uint8_t id)
{
	struct hwep_data *hwep_info = (struct hwep_data *)dai;
	struct btfmswr *btfmswr = dev_get_drvdata(hwep_info->dev);
	struct hwep_dma_configurations *hwep_config;

	BTFMSWR_DBG("");
	hwep_config = (struct hwep_dma_configurations *)config;

	hwep_config->stream_id = id;
	hwep_config->sample_rate = btfmswr->sample_rate;
	hwep_config->bit_width = (uint8_t)btfmswr->bps;
	hwep_config->codectype = usecase_codec;

	hwep_config->num_channels = btfmswr->num_channels;
	hwep_config->active_channel_mask = (btfmswr->num_channels == 2 ?
					   TWO_CHANNEL_MASK : ONE_CHANNEL_MASK);
	hwep_config->lpaif = LPAIF_AUD;
	hwep_config->inf_index = 1;
	return 1;
}

static struct hwep_dai_ops  btfmswr_hw_dai_ops = {
	.hwep_startup = btfm_swr_dai_startup,
	.hwep_shutdown = btfm_swr_dai_shutdown,
	.hwep_hw_params = btfm_swr_dai_hw_params,
	.hwep_prepare = btfm_swr_dai_prepare,
	.hwep_set_channel_map = btfm_swr_dai_set_channel_map,
	.hwep_get_channel_map = btfm_swr_dai_get_channel_map,
	.hwep_get_configs = btfm_swr_dai_get_configs,
	.hwep_codectype = &usecase_codec,
};

static struct hwep_dai_driver btfmswr_dai_driver[] = {
	{	/* FM Audio data multiple channel  : FM -> lpass */
		.dai_name = "btaudio_fm_tx",
		.id = FMAUDIO_TX,
		.capture = {
			.stream_name = "FM SWR TX Capture",
			.rates = SNDRV_PCM_RATE_48000, /* 48 KHz */
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 48000,
			.rate_min = 48000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.dai_ops = &btfmswr_hw_dai_ops,
	},
	{	/* Bluetooth SCO voice uplink: bt -> lpass */
		.dai_name = "btaudio_tx",
		.id = BTAUDIO_TX,
		.capture = {
			.stream_name = "BT Audio SWR Tx Capture",
			/* 8 KHz or 16 KHz */
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000
				| SNDRV_PCM_RATE_8000_192000
				| SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000
				| SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000
				| SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.dai_ops = &btfmswr_hw_dai_ops,
	},
	{	/* Bluetooth SCO voice downlink: lpass -> bt or A2DP Playback */
		.dai_name = "btaudio_rx",
		.id = BTAUDIO_RX,
		.playback = {
			.stream_name = "BT Audio SWR Rx Playback",
			/* 8/16/44.1/48/88.2/96 Khz */
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000
				| SNDRV_PCM_RATE_8000_192000
				| SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000
				| SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000
				| SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.dai_ops = &btfmswr_hw_dai_ops,
	},
	{	/* Bluetooth A2DP sink: bt -> lpass */
		.dai_name = "btfm_a2dp_sink_swr_tx",
		.id = BTAUDIO_A2DP_SINK_TX,
		.capture = {
			.stream_name = "A2DP sink TX Capture",
			/* 8/16/44.1/48/88.2/96/192 Khz */
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000
				| SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000
				| SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000
				| SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.dai_ops = &btfmswr_hw_dai_ops,
	}
};

static struct hwep_comp_drv btfmswr_hw_driver = {
	.hwep_probe	= btfm_swr_hwep_probe,
	.hwep_remove	= btfm_swr_hwep_remove,
	.hwep_read	= btfm_swr_hwep_read,
	.hwep_write	= btfm_swr_hwep_write,
};

int btfm_swr_register_hw_ep(struct btfmswr *btfm_swr)
{
	struct device *dev = btfm_swr->dev;
	struct hwep_data *hwep_info;
	int ret = 0;

	BTFMSWR_INFO("Registering with BTFMCODEC HWEP interface\n");
	hwep_info = kzalloc(sizeof(struct hwep_data), GFP_KERNEL);

	if (!hwep_info) {
		BTFMSWR_ERR("%s: failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto end;
	}

	/* Copy EP device parameters as intercations will be on the same device */
	hwep_info->dev = dev;
	strscpy(hwep_info->driver_name, SWR_SLAVE_COMPATIBLE_STR, DEVICE_NAME_MAX_LEN);
	hwep_info->drv = &btfmswr_hw_driver;
	hwep_info->dai_drv = btfmswr_dai_driver;
	hwep_info->num_dai = ARRAY_SIZE(btfmswr_dai_driver);
	hwep_info->num_dai = 4;
	hwep_info->num_mixer_ctrl = ARRAY_SIZE(status_controls);
	hwep_info->mixer_ctrl = status_controls;
	/* Register to hardware endpoint */
	ret = btfmcodec_register_hw_ep(hwep_info);
	if (ret) {
		BTFMSWR_ERR("failed to register with btfmcodec driver hw interface (%d)", ret);
		goto end;
	}

	BTFMSWR_INFO("Registered succesfull with BTFMCODEC HWEP interface\n");
	return ret;
end:
	return ret;
}

void btfm_swr_unregister_hwep(void)
{
	BTFMSWR_INFO("Unregistered with BTFMCODEC HWEP	interface");
	/* Unregister with BTFMCODEC HWEP	driver */
	btfmcodec_unregister_hw_ep(SWR_SLAVE_COMPATIBLE_STR);

}

MODULE_DESCRIPTION("BTFM SoundWire Codec driver");
MODULE_LICENSE("GPL v2");
