/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/types.h>

#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/q6adm-v2.h>
#include <sound/q6asm-v2.h>
#include <sound/q6afe-v2.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "msm-ahc-config.h"

enum {
	AHC_IIR_TF_REG_ENABLE = 0,
	AHC_IIR_TF_REG_ENABLE_CONFIG,
	AHC_IIR_TF_REG_PRE_GAIN,
	AHC_IIR_TF_REG_CONFIG_PARAMS,
	AHC_IIR_TF_REG_CONFIG_ALL_PARAMS,
	AHC_MBDRC_REG_ENABLE,
	AHC_MBDRC_REG_ENABLE_CONFIG,
};

enum {
	AHC_IIR_TF_SHIFT_BAND1 = 0,
	AHC_IIR_TF_SHIFT_BAND2,
	AHC_IIR_TF_SHIFT_BAND3,
	AHC_IIR_TF_SHIFT_BAND4,
	AHC_IIR_TF_SHIFT_BAND5,
};

#define AHC_MAX_NBR_OF_BANDS		20
#define AHC_BANDS_USED			5

#define IIR_COEFF_CHANNEL_LEFT		0
#define IIR_COEFF_CHANNEL_CENTER	1
#define IIR_COEFF_CHANNEL_RIGHT		2

/* 1073741824 is Q30 format for 1.0 */
#define DEFAULT_COEFF {1073741824, 0, 0, 0, 0}
#define START_COEFF_STRUCT {.coeff = DEFAULT_COEFF, .num_shift_factor = 2, \
			.pan_setting = IIR_COEFF_CHANNEL_LEFT}

/* Multiplier to get around a bug in Qualcomm's DSP adm_get_params command.
 * We need to multiply param size with 4 since Qualcomm's code expects
 * param size when response is received to be in number of words, but
 * DSP returns number of bytes. When data is interpreted by us it is
 * done correctly so there is no need to change this.
 */
#define QCOM_GET_BUG_MULTIPLIER		4

struct audproc_common_enable {
	uint32_t enable;
} __packed;

struct iir_tf_5_config {
	struct asm_iir_filter_config_params bands;
	int32_t coeffs[5][5];
	int16_t num_shift_factor[5];
	uint16_t pan_setting[5];
} __packed;

struct iir_tf_20_config {
	struct asm_iir_filter_config_params bands;
	int32_t coeffs[20][5];
	int16_t num_shift_factor[20];
	uint16_t pan_setting[20];
} __packed;

struct iir_tf_coeff {
	int32_t coeff[5];
	int16_t num_shift_factor;
	uint16_t pan_setting;
};

static struct iir_tf_coeff iir_tf_coeffs[AHC_BANDS_USED] = {
	START_COEFF_STRUCT, START_COEFF_STRUCT, START_COEFF_STRUCT,
	START_COEFF_STRUCT, START_COEFF_STRUCT
};

static int32_t default_coeff[5] = DEFAULT_COEFF;

static int ahc_copp_idx = -1;

static int iir_tf_send_params(int port_id, uint32_t module_id,
		uint32_t param_id, uint8_t *param, uint32_t param_len)
{
	uint8_t *params_value;
	struct adm_param_data_v5 *p_hdr;
	uint32_t pkt_len = sizeof(*p_hdr) + param_len;
	int rc;

	pr_debug("%s: module_id 0x%X param_id 0x%X param_len %d\n",
			__func__, module_id, param_id, param_len);

	params_value = kmalloc(pkt_len, GFP_KERNEL);
	if (!params_value) {
		pr_err("%s, params memory alloc failed\n", __func__);
		return -ENOMEM;
	}

	p_hdr = (struct adm_param_data_v5 *)params_value;
	p_hdr->module_id = module_id;
	p_hdr->param_id = param_id;
	p_hdr->param_size = param_len;
	p_hdr->reserved = 0;
	memcpy(params_value + sizeof(*p_hdr), param, param_len);

	rc = adm_ahc_send_params(port_id, ahc_copp_idx, params_value,
			pkt_len);
	if (rc)
		pr_err("%s: send ahc params failed %d\n", __func__, rc);

	kfree(params_value);
	return rc;
}

static int send_iir_config(void)
{
	struct iir_tf_5_config *params;
	int rc = 0, i, j;

	params = kmalloc(sizeof(*params), GFP_KERNEL);
	if (!params) {
		pr_err("%s: Could not allocate params\n", __func__);
		return -ENOMEM;
	}

	params->bands.num_biquad_stages = AHC_BANDS_USED;
	params->bands.reserved = 0;

	for (i = 0; i < AHC_BANDS_USED; i++) {
		for (j = 0; j < 5; j++)
			params->coeffs[i][j] = iir_tf_coeffs[i].coeff[j];
		params->num_shift_factor[i] =
				iir_tf_coeffs[i].num_shift_factor;
		params->pan_setting[i] = iir_tf_coeffs[i].pan_setting;
	}

	rc = iir_tf_send_params(AHC_PORT_ID,
			ASM_MODULE_ID_IIRUNING_FILTER,
			ASM_PARAM_ID_IIRUNING_FILTER_CONFIG_PARAMS,
			(uint8_t *)params, sizeof(*params));

	kfree(params);
	return rc;
}

static int get_params_for_get_cmd(int cmd, int band, int *param_len,
		uint32_t *param_id, uint32_t *module_id)
{
	switch (cmd) {
	case AHC_IIR_TF_REG_ENABLE:
		pr_debug("%s: AHC_IIR_TF_REG_ENABLE\n", __func__);
		*param_len = sizeof(struct audproc_common_enable);
		*param_id = AUDPROC_PARAM_ID_ENABLE;
		*module_id = ASM_MODULE_ID_IIRUNING_FILTER;
		break;

	case AHC_IIR_TF_REG_ENABLE_CONFIG:
		pr_debug("%s: AHC_IIR_TF_REG_ENABLE_CONFIG\n", __func__);
		*param_len = sizeof(struct asm_iiruning_filter_enable);
		*param_id = ASM_PARAM_ID_IIRUNING_FILTER_ENABLE_CONFIG;
		*module_id = ASM_MODULE_ID_IIRUNING_FILTER;
		break;

	case AHC_IIR_TF_REG_PRE_GAIN:
		pr_debug("%s: AHC_IIR_TF_REG_PRE_GAIN\n", __func__);
		*param_len = sizeof(struct asm_iiruning_filter_pregain);
		*param_id = ASM_PARAM_ID_IIRUNING_FILTER_PRE_GAIN;
		*module_id = ASM_MODULE_ID_IIRUNING_FILTER;
		break;

	case AHC_IIR_TF_REG_CONFIG_PARAMS:
		pr_debug("%s: AHC_IIR_TF_REG_CONFIG_PARAMS for band %d\n",
				__func__, band);
		*param_len = sizeof(struct iir_tf_20_config);
		*param_id = ASM_PARAM_ID_IIRUNING_FILTER_CONFIG_PARAMS;
		*module_id = ASM_MODULE_ID_IIRUNING_FILTER;
		break;

	case AHC_IIR_TF_REG_CONFIG_ALL_PARAMS:
		pr_debug("%s: AHC_IIR_TF_REG_CONFIG_ALL_PARAMS\n", __func__);
		*param_len = sizeof(struct iir_tf_20_config);
		*param_id = ASM_PARAM_ID_IIRUNING_FILTER_CONFIG_PARAMS;
		*module_id = ASM_MODULE_ID_IIRUNING_FILTER;
		break;

	case AHC_MBDRC_REG_ENABLE:
		pr_debug("%s: AHC_MBDRC_REG_ENABLE\n", __func__);
		*param_len = sizeof(struct audproc_common_enable);
		*param_id = AUDPROC_PARAM_ID_ENABLE;
		*module_id = ASM_MODULE_ID_MBDRCV3;
		break;

	case AHC_MBDRC_REG_ENABLE_CONFIG:
		pr_debug("%s: AHC_MBDRC_REG_ENABLE_CONFIG\n", __func__);
		*param_len = sizeof(struct asm_mbdrc_enable);
		*param_id = ASM_PARAM_ID_MBDRC_ENABLE;
		*module_id = ASM_MODULE_ID_MBDRCV3;
		break;

	default:
		pr_err("%s: Received unknown command %d\n", __func__, cmd);
		*param_len = 0;
		*param_id = 0;
		*module_id = 0;
		return -EINVAL;
	}
	return 0;
}

static int set_values_for_get_cmd(int cmd, int band,
		struct snd_ctl_elem_value *ucontrol, uint8_t *params_value)
{
	switch (cmd) {
	case AHC_IIR_TF_REG_ENABLE: /* FALLTHROUGH */
	case AHC_MBDRC_REG_ENABLE: {
		struct audproc_common_enable *param =
				(struct audproc_common_enable *)params_value;
		ucontrol->value.integer.value[0] = param->enable;
		pr_debug("%s: Val %ld returned\n", __func__,
				ucontrol->value.integer.value[0]);
		break;
	}

	case AHC_IIR_TF_REG_ENABLE_CONFIG: {
		struct asm_iiruning_filter_enable *param =
			(struct asm_iiruning_filter_enable *)params_value;
		ucontrol->value.integer.value[0] = param->enable_flag;
		pr_debug("%s: Val %ld returned\n", __func__,
				ucontrol->value.integer.value[0]);
		break;
	}

	case AHC_IIR_TF_REG_PRE_GAIN: {
		struct asm_iiruning_filter_pregain *param =
			(struct asm_iiruning_filter_pregain *)params_value;
		ucontrol->value.integer.value[0] = param->pregain;
		pr_debug("%s: Val %ld returned\n", __func__,
				ucontrol->value.integer.value[0]);
		break;
	}

	case AHC_IIR_TF_REG_CONFIG_PARAMS: {
		/*
		 * We can always assume 20 band struct since we only care
		 * the coeff values.
		 */
		struct iir_tf_20_config *param =
				(struct iir_tf_20_config *)params_value;
		int32_t *p_coeff;
		int i;

		if (band < param->bands.num_biquad_stages)
			p_coeff = &param->coeffs[band][0];
		else
			p_coeff = &default_coeff[0];

		for (i = 0; i < 5; i++) {
			ucontrol->value.integer.value[i] = p_coeff[i];
			pr_debug("%s: Val %ld returned\n", __func__,
				ucontrol->value.integer.value[i]);
		}
		break;
	}

	case AHC_IIR_TF_REG_CONFIG_ALL_PARAMS: {
		/*
		 * We can always assume 20 band struct since we only care
		 * the coeff values.
		 */
		struct iir_tf_20_config *param =
				(struct iir_tf_20_config *)params_value;
		int32_t *p_coeff;
		int i;
		int j;
		int idx = 0;

		for (i = 0; i < AHC_BANDS_USED; i++) {
			if (i < param->bands.num_biquad_stages)
				p_coeff = &param->coeffs[i][0];
			else
				p_coeff = &default_coeff[0];

			for (j = 0; j < 5; j++) {
				ucontrol->value.integer.value[idx] = p_coeff[j];
				pr_debug("%s: Val %ld returned\n", __func__,
					ucontrol->value.integer.value[i]);
				idx++;
			}
		}
		break;
	}

	case AHC_MBDRC_REG_ENABLE_CONFIG: {
		struct asm_mbdrc_enable *param =
				(struct asm_mbdrc_enable *)params_value;
		ucontrol->value.integer.value[0] = param->enable_flag;
		pr_debug("%s: Val %ld returned\n", __func__,
				ucontrol->value.integer.value[0]);
		break;
	}

	default:
		/* Will never happen. Checked in get_params_for_get_cmd */
		break;
	}
	return 0;
}

static int handle_get_cmd(int cmd, int band,
		struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	int param_len = 0;
	uint32_t param_id = 0;
	uint32_t module_id = 0;
	uint8_t *params_value;

	if ((ahc_copp_idx < 0) || (ahc_copp_idx > MAX_COPPS_PER_PORT)) {
		pr_err("%s: ahc_copp_idx not set\n", __func__);
		return -EINVAL;
	}

	rc = get_params_for_get_cmd(cmd, band, &param_len, &param_id,
			&module_id);
	if (rc)
		return rc;

	/* Fix to handle Qualcomm bug */
	param_len *= QCOM_GET_BUG_MULTIPLIER;

	params_value = kzalloc(param_len, GFP_KERNEL);
	if (!params_value) {
		pr_err("%s: params memory alloc failed\n", __func__);
		return -ENOMEM;
	}

	pr_debug("%s: Retrieving parameters len %d\n", __func__, param_len);
	/*
	 * Retrieve the parameters.
	 * param_len must be increased with size of struct adm_param_data_v5
	 * because adm_get_params() does not add this size to the return
	 * buffer allocation even though that header is always included
	 * before the actual parameter payload.
	 */
	rc = adm_get_params(AHC_PORT_ID,
			ahc_copp_idx,
			module_id,
			param_id,
			param_len + sizeof(struct adm_param_data_v5),
			params_value);
	if (rc) {
		pr_err("%s: get parameters failed %d\n", __func__, rc);
		goto exit;
	}

	rc = set_values_for_get_cmd(cmd, band, ucontrol, params_value);

exit:
	kfree(params_value);
	return rc;
}

static int handle_put_cmd(int cmd, int band,
		struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;

	if ((ahc_copp_idx < 0) || (ahc_copp_idx > MAX_COPPS_PER_PORT)) {
		pr_err("%s: ahc_copp_idx not set\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case AHC_IIR_TF_REG_ENABLE: {
		struct audproc_common_enable param;

		param.enable = ucontrol->value.integer.value[0];
		pr_debug("%s: AHC_IIR_TF_REG_ENABLE val: %u\n",
				__func__, param.enable);
		rc = iir_tf_send_params(AHC_PORT_ID,
				ASM_MODULE_ID_IIRUNING_FILTER,
				AUDPROC_PARAM_ID_ENABLE,
				(uint8_t *)&param, sizeof(param));
		break;
	}

	case AHC_IIR_TF_REG_ENABLE_CONFIG: {
		struct asm_iiruning_filter_enable param;

		param.enable_flag = ucontrol->value.integer.value[0];
		pr_debug("%s: AHC_IIR_TF_REG_ENABLE_CONFIG val: %u\n",
				__func__, param.enable_flag);
		rc = iir_tf_send_params(AHC_PORT_ID,
				ASM_MODULE_ID_IIRUNING_FILTER,
				ASM_PARAM_ID_IIRUNING_FILTER_ENABLE_CONFIG,
				(uint8_t *)&param, sizeof(param));
		break;
	}

	case AHC_IIR_TF_REG_PRE_GAIN: {
		struct asm_iiruning_filter_pregain param;

		pr_debug("%s: AHC_IIR_TF_REG_PRE_GAIN val: %ld\n",
				__func__, ucontrol->value.integer.value[0]);
		if (ucontrol->value.integer.value[0] < 0 ||
				ucontrol->value.integer.value[0] > 0xFFFF) {
			pr_err("%s: value %ld is not uint16_t", __func__,
					ucontrol->value.integer.value[0]);
			return -EINVAL;
		}
		param.pregain = ucontrol->value.integer.value[0];
		param.reserved = 0;
		rc = iir_tf_send_params(AHC_PORT_ID,
				ASM_MODULE_ID_IIRUNING_FILTER,
				ASM_PARAM_ID_IIRUNING_FILTER_PRE_GAIN,
				(uint8_t *)&param, sizeof(param));
		break;
	}

	case AHC_IIR_TF_REG_CONFIG_PARAMS: {
		long *temp = ucontrol->value.integer.value;
		int i;

		/* Two prints to avoid checkpatch warning */
		pr_debug("%s: AHC_IIR_TF_REG_CONFIG_PARAMS band %d:",
				__func__, band);
		pr_debug(" %ld %ld %ld %ld %ld\n",
				temp[0], temp[1], temp[2], temp[3], temp[4]);

		for (i = 0; i < 5; i++)
			iir_tf_coeffs[band].coeff[i] = temp[i];

		rc = send_iir_config();
		break;
	}

	case AHC_IIR_TF_REG_CONFIG_ALL_PARAMS: {
		long *temp = ucontrol->value.integer.value;
		int i;
		int j;

		pr_debug("%s: AHC_IIR_TF_REG_CONFIG_ALL_PARAMS", __func__);

		for (i = 0; i < AHC_BANDS_USED; i++)
			for (j = 0; j < 5; j++)
				iir_tf_coeffs[i].coeff[j] =
						temp[(i * 5) + j];

		rc = send_iir_config();
		break;
	}

	case AHC_MBDRC_REG_ENABLE: {
		struct audproc_common_enable param;

		param.enable = ucontrol->value.integer.value[0];
		pr_debug("%s: AHC_MBDRC_REG_ENABLE recvd val: %u\n",
				__func__, param.enable);
		rc = iir_tf_send_params(AHC_PORT_ID, ASM_MODULE_ID_MBDRCV3,
				AUDPROC_PARAM_ID_ENABLE,
				(uint8_t *)&param, sizeof(param));
		break;
	}

	case AHC_MBDRC_REG_ENABLE_CONFIG: {
		struct asm_mbdrc_enable param;

		param.enable_flag = ucontrol->value.integer.value[0];
		pr_debug("%s: AHC_MBDRC_REG_ENABLE_CONFIG recvd val: %u\n",
				__func__, param.enable_flag);
		rc = iir_tf_send_params(AHC_PORT_ID, ASM_MODULE_ID_MBDRCV3,
				ASM_PARAM_ID_MBDRC_ENABLE,
				(uint8_t *)&param, sizeof(param));
		break;
	}

	default:
		pr_err("%s: Received unknown command %d\n", __func__, cmd);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int get_ahc_multi_param(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	struct soc_multi_mixer_control *mc =
			(struct soc_multi_mixer_control *)
			kcontrol->private_value;

	return handle_get_cmd(mc->reg, mc->shift, ucontrol);
}

static int put_ahc_multi_param(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	struct soc_multi_mixer_control *mc =
			(struct soc_multi_mixer_control *)
			kcontrol->private_value;

	return handle_put_cmd(mc->reg, mc->shift, ucontrol);
}

static const struct snd_kcontrol_new ahc_multi_controls[] = {

	/* IIR Tuning Filter */
	SOC_SINGLE_MULTI_EXT("AHC IIR Enable", AHC_IIR_TF_REG_ENABLE,
			0, 1, 0, 1,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR EnableConfig",
			AHC_IIR_TF_REG_ENABLE_CONFIG,
			0, 1, 0, 1,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR PreGain", AHC_IIR_TF_REG_PRE_GAIN,
			0, 0x0000FFFF, 0, 1,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR Band1", AHC_IIR_TF_REG_CONFIG_PARAMS,
			AHC_IIR_TF_SHIFT_BAND1, 0xFFFFFFFF, 0, 5,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR Band2", AHC_IIR_TF_REG_CONFIG_PARAMS,
			AHC_IIR_TF_SHIFT_BAND2, 0xFFFFFFFF, 0, 5,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR Band3", AHC_IIR_TF_REG_CONFIG_PARAMS,
			AHC_IIR_TF_SHIFT_BAND3, 0xFFFFFFFF, 0, 5,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR Band4", AHC_IIR_TF_REG_CONFIG_PARAMS,
			AHC_IIR_TF_SHIFT_BAND4, 0xFFFFFFFF, 0, 5,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR Band5", AHC_IIR_TF_REG_CONFIG_PARAMS,
			AHC_IIR_TF_SHIFT_BAND5, 0xFFFFFFFF, 0, 5,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC IIR Bands", AHC_IIR_TF_REG_CONFIG_ALL_PARAMS,
			0, 0xFFFFFFFF, 0, 25,
			get_ahc_multi_param, put_ahc_multi_param),

	/* MBDRC */
	SOC_SINGLE_MULTI_EXT("AHC MBDRC Enable", AHC_MBDRC_REG_ENABLE,
			0, 1, 0, 1,
			get_ahc_multi_param, put_ahc_multi_param),

	SOC_SINGLE_MULTI_EXT("AHC MBDRC EnableConfig",
			AHC_MBDRC_REG_ENABLE_CONFIG,
			0, 1, 0, 1,
			get_ahc_multi_param, put_ahc_multi_param),
};

void msm_routing_ahc_set_copp_idx(int copp_idx)
{
	pr_debug("%s: copp_idx %d\n", __func__, copp_idx);
	ahc_copp_idx = copp_idx;
}

void msm_routing_ahc_add_controls(struct snd_soc_platform *platform)
{
	int rc;

	rc = snd_soc_add_platform_controls(platform, ahc_multi_controls,
			ARRAY_SIZE(ahc_multi_controls));
	if (rc)
		pr_err("%s: Failed to add multi controls %d\n", __func__, rc);
}
