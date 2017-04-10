/*
 * Author: Yoshio Yamamoto yoshio.xa.yamamoto@sonymobile.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <sound/apr_audio-v2.h>
#include <sound/q6asm-v2.h>
#include <sound/compress_params.h>
#include "msm-sony-hweffect.h"
#include "sound/sony-hweffect-params.h"

/*
 * Value array (max size:128)
 * +---------------+---------------------------+
 * |  array index  | value                     |
 * +---------------+---------------------------+
 * |       0       | output device (unused)    |
 * +---------------+---------------------------+
 * |       1       | num of commands           |
 * +---------------+---------------------------+
 * |       2       | 1st command               |
 * +---------------+---------------------------+
 * |       3       | config state              |
 * +---------------+---------------------------+
 * |       4       | 1st param offset          |
 * +---------------+---------------------------+
 * |       5       | 1st param length          |
 * +---------------+---------------------------+
 * |     6 to n    | 1st param effect params   |
 * +---------------+---------------------------+
 * |     n + 1     | 2nd command               |
 * +---------------+---------------------------+
 * |     n + 2     | config state              |
 * +---------------+---------------------------+
 * |     n + 3     | 2nd param offset          |
 * +---------------+---------------------------+
 * |     n + 4     | 2nd param length          |
 * +---------------+---------------------------+
 * | (n + 5) to m  | 2nd param effect params   |
 * +---------------+---------------------------+
 *                       .
 *                       .
 *                       .
*/

int msm_sony_hweffect_sonybundle_handler(struct audio_client *ac,
				struct sonybundle_params *sb,
				long *values)
{
	int devices = *values++;
	int num_commands = *values++;
	char *params, *effect;
	int i, j;
	uint32_t module_id, param_id;
	int rc = 0;

	pr_debug("%s\n", __func__);
	if (!ac) {
		pr_err("%s: cannot set sonyeffect hw\n", __func__);
		return -EINVAL;
	}
	params = kzalloc(MAX_INBAND_PARAM_SZ, GFP_KERNEL);
	if (!params) {
		pr_err("%s, params memory alloc failed\n", __func__);
		return -ENOMEM;
	}
	pr_debug("%s: device: %d, num_commands %d\n",
				__func__, devices, num_commands);

	effect = params + sizeof(struct asm_stream_param_data_v2);
	for (i = 0; i < num_commands; i++) {
		uint32_t p_length = 0;
		uint32_t command_id = *values++;
		uint32_t command_config_state = *values++;
		uint32_t index_offset = *values++;
		uint32_t length = *values++;
		switch (command_id) {
		case SONYBUNDLE_ENABLE:
			if (length != 1 || index_offset != 0) {
				pr_err("SONYBUNDLE_ENABLE:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->common.enable_flag = *values++;
			pr_debug("%s:SONYBUNDLE_ENABLE state:%d\n", __func__,
			sb->common.enable_flag);
			if (command_config_state == CONFIG_SET) {
				struct common_params *cmn_params =
					(struct common_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_COMMON_USER_PARAM;
				cmn_params->enable_flag =
					sb->common.enable_flag;
				p_length += sizeof(struct common_params);
			}
			break;

		case DYNAMIC_NORMALIZER_ENABLE:
			if (length != 1 || index_offset != 0) {
				pr_err("DYNAMIC_NORMALIZER_ENABLE:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->dynamic_normalizer.enable_flag = *values++;
			pr_debug("%s:DYNAMIC_NORMALIZER_ENABLE state:%d\n",
				__func__, sb->dynamic_normalizer.enable_flag);
			if (command_config_state == CONFIG_SET) {
				struct dynamic_normalizer_params *dn_params =
				(struct dynamic_normalizer_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id =
				PARAM_ID_SB_DYNAMICNORMALIZER_USER_PARAM;
				dn_params->enable_flag =
					sb->dynamic_normalizer.enable_flag;
				p_length +=
				sizeof(struct dynamic_normalizer_params);
			}
			break;

		case SFORCE_ENABLE:
			if (length != 1 || index_offset != 0) {
				pr_err("SFORCE_ENABLE:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->sforce.enable_flag = *values++;
			pr_debug("%s:SFORCE_ENABLE state:%d\n", __func__,
				sb->sforce.enable_flag);
			if (command_config_state == CONFIG_SET) {
				struct sforce_params *sf_params =
					(struct sforce_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_SFORCE_USER_PARAM;
				sf_params->enable_flag =
					sb->sforce.enable_flag;
				p_length += sizeof(struct sforce_params);

				if (!sb->sforce_tuning_update) {
					int ret;
					ret = sony_hweffect_send_tuning_params(
							SFORCE_PARAM,
							(void *)ac);
					pr_debug("%s:sony_hweffect_send_tuning_params(S-Force) ret=%d\n",
						__func__, ret);
					if (ret < 0) {
						pr_err("SFORCE_ENABLE: send tuning param error\n");
						rc = -EINVAL;
						goto invalid_config;
					}
					sb->sforce_tuning_update = true;
				}
			}
			break;

		case VPT20_MODE:
			if (length != 1 || index_offset != 0) {
				pr_err("VPT20_MODE:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->vpt20.mode = *values++;
			pr_debug("%s:VPT20_MODE: %d\n", __func__,
				sb->vpt20.mode);
			if (command_config_state == CONFIG_SET) {
				struct vpt20_params *vpt_params =
				(struct vpt20_params *)effect;
				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_VPT20_USER_PARAM;
				vpt_params->mode = sb->vpt20.mode;
				p_length += sizeof(struct vpt20_params);
			}
			break;

		case CLEARPHASE_HP_MODE:
			if (length != 1 || index_offset != 0) {
				pr_err("CLEARPHASE_HP_MODE:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->clearphase_hp.mode = *values++;
			pr_debug("%s:CLEARPHASE_HP_MODE: %d\n", __func__,
				sb->clearphase_hp.mode);
			if (command_config_state == CONFIG_SET) {
				struct clearphase_hp_params *cp_hp_params =
				(struct clearphase_hp_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_CLEARPHASE_HP_USER_PARAM;
				cp_hp_params->mode = sb->clearphase_hp.mode;
				p_length += sizeof(struct clearphase_hp_params);

				{
					int ret;
					ret = sony_hweffect_send_tuning_params(
							CLEARPHASE_HP_PARAM,
							(void *)ac);
					pr_debug("%s:sony_hweffect_send_tuning_params(CP_HP) ret=%d\n",
						__func__, ret);
					if (ret < 0) {
						pr_err("CLEARPHASE_HP_MODE: send tuning param error\n");
						rc = -EINVAL;
						goto invalid_config;
					}
				}
			}
			break;

		case CLEARAUDIO_CHSEP:
			if (length != 1 || index_offset != 0) {
				pr_err("CLEARAUDIO_CHSEP:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->clearaudio.chsep_coef = *values++;
			pr_debug("%s:CLEARAUDIO_CHSEP: %d\n", __func__,
				sb->clearaudio.chsep_coef);
			if (command_config_state == CONFIG_SET) {
				struct clearaudio_params *ca_params =
				(struct clearaudio_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_CLEARAUDIO_USER_PARAM;
				ca_params->chsep_coef =
					sb->clearaudio.chsep_coef;
				for (j = 0; j < BAND_NUM; j++)
					ca_params->eq_coef[j] =
						sb->clearaudio.eq_coef[j];
				p_length +=
					sizeof(struct clearaudio_params);
			}
			break;

		case CLEARAUDIO_EQ_COEF:
			if (length != 6 || index_offset != 0) {
				pr_err("CLEARAUDIO_EQ_COEF:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			for (j = 0; j < BAND_NUM; j++)
				sb->clearaudio.eq_coef[j] =
					(int16_t)*values++;
				pr_debug("%s:CLEARAUDIO_EQ_COEF: %d %d %d %d %d %d\n",
						__func__,
						sb->clearaudio.eq_coef[0],
						sb->clearaudio.eq_coef[1],
						sb->clearaudio.eq_coef[2],
						sb->clearaudio.eq_coef[3],
						sb->clearaudio.eq_coef[4],
						sb->clearaudio.eq_coef[5]);
			if (command_config_state == CONFIG_SET) {
				struct clearaudio_params *ca_params =
				(struct clearaudio_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_CLEARAUDIO_USER_PARAM;
				ca_params->chsep_coef =
					sb->clearaudio.chsep_coef;
				for (j = 0; j < BAND_NUM; j++)
					ca_params->eq_coef[j] =
						sb->clearaudio.eq_coef[j];
				p_length +=
					sizeof(struct clearaudio_params);
			}
			break;

		case CLEARAUDIO_VOLUME:
			if (length != 1 || index_offset != 0) {
				pr_err("CLEARAUDIO_VOLUME:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->ca_volume.gain = *values++;
			pr_debug("%s:CLEARAUDIO_VOLUME: %d\n", __func__,
				sb->ca_volume.gain);
			if (command_config_state == CONFIG_SET) {
				struct clearaudio_volume *ca_volume =
				(struct clearaudio_volume *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_CLEARAUDIO_VOLUME_PARAM;
				ca_volume->gain = sb->ca_volume.gain;
				p_length +=
					sizeof(struct clearaudio_volume);
			}
			break;

		case CLEARPHASE_SP_ENABLE:
			if (length != 1 || index_offset != 0) {
				pr_err("CLEARPHASE_SP_ENABLE:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->clearphase_sp.mode = *values++;
			pr_debug("%s:CLEARPHASE_SP_ENABLE mode:%d\n", __func__,
				sb->clearphase_sp.mode);
			if (command_config_state == CONFIG_SET) {
				struct clearphase_sp_params *cp_sp_params =
				(struct clearphase_sp_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_CLEARPHASE_SP_USER_PARAM;
				cp_sp_params->mode = sb->clearphase_sp.mode;
				p_length +=
					sizeof(struct clearphase_sp_params);

				if (!sb->clearphase_sp_tuning_update) {
					int ret;
					ret = sony_hweffect_send_tuning_params(
							CLEARPHASE_SP_PARAM,
							(void *)ac);
					pr_debug("%s:sony_hweffect_send_tuning_params(CP_SP) ret=%d\n",
						__func__, ret);
					if (ret < 0) {
						pr_err("CLEARPHASE_SP_ENABLE: send tuning param error\n");
						rc = -EINVAL;
						goto invalid_config;
					}
					sb->clearphase_sp_tuning_update = true;
				}
			}
			break;

		case XLOUD_ENABLE:
			if (length != 1 || index_offset != 0) {
				pr_err("XLOUD_ENABLE:invalid params\n");
				rc = -EINVAL;
				goto invalid_config;
			}
			sb->xloud.enable_flag = *values++;
			pr_debug("%s:XLOUD_ENABLE state:%d\n", __func__,
				sb->xloud.enable_flag);
			if (command_config_state == CONFIG_SET) {
				struct xloud_params *xl_params =
				(struct xloud_params *)effect;

				module_id = ASM_MODULE_ID_SONYBUNDLE;
				param_id = PARAM_ID_SB_XLOUD_USER_PARAM;
				xl_params->enable_flag = sb->xloud.enable_flag;
				p_length += sizeof(struct xloud_params);

				if (!sb->xloud_tuning_update) {
					int ret;
					ret = sony_hweffect_send_tuning_params(
							XLOUD_PARAM,
							(void *)ac);
					pr_debug("%s:sony_hweffect_send_tuning_params(XLOUD) ret=%d\n",
						__func__, ret);
					if (ret < 0) {
						pr_err("XLOUD_ENABLE: send tuning param error\n");
						rc = -EINVAL;
						goto invalid_config;
					}
					sb->xloud_tuning_update = true;
				}
			}
			break;

		default:
			pr_err("%s: Invalid command to set config\n", __func__);
			break;
		}

		if (p_length) {
			struct asm_stream_param_data_v2 *param_data;
			int ret;

			param_data =
			(struct asm_stream_param_data_v2 *)params;
			param_data->module_id = module_id;
			param_data->param_id = param_id;
			param_data->param_size = (uint16_t)p_length;
			param_data->reserved = 0;
			p_length +=
				sizeof(struct asm_stream_param_data_v2);

			ret = q6asm_send_audio_effects_params(ac,
						params, p_length);
			if (ret < 0) {
				pr_err("q6asm_send_audio_effects_params "
						"failed %d\n", ret);
				rc = -EINVAL;
				goto invalid_config;
			}
		}
	}

invalid_config:
	kfree(params);
	return rc;
}

void init_sonybundle_params(struct sonybundle_params *sb)
{
	/* Initialization of effect having initial value except for 0 */
	sb->clearphase_hp.mode = CP_MODE_OFF;
	sb->clearphase_sp.mode = CP_MODE_OFF;
	sb->vpt20.mode = VPT_MODE_OFF;

	return;
}
