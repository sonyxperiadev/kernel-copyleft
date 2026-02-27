// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/io.h>
#include <linux/of.h>

#include "msm_vidc_platform.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_v4l2.h"
#include "msm_vidc_vb2.h"
#include "msm_vidc_core.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_memory.h"
#include "msm_vidc_control.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_fence.h"
#include "hfi_packet.h"
#include "hfi_property.h"
#include "venus_hfi.h"

#if defined(CONFIG_MSM_VIDC_PINEAPPLE)
#include "msm_vidc_pineapple.h"
#include "msm_vidc_cliffs.h"
#include "msm_vidc_iris33.h"
#endif
#if defined(CONFIG_MSM_VIDC_KALAMA)
#include "msm_vidc_kalama.h"
#include "msm_vidc_iris3.h"
#endif
#if defined(CONFIG_MSM_VIDC_WAIPIO)
#include "msm_vidc_waipio.h"
#include "msm_vidc_iris2.h"
#endif

#define CAP_TO_8BIT_QP(a) {          \
	if ((a) < MIN_QP_8BIT)                 \
		(a) = MIN_QP_8BIT;             \
}

/*
 * Below calculation for number of reference frames
 * is picked up from hfi macro HFI_IRIS3_ENC_RECON_BUF_COUNT
 */
#define SLIDING_WINDOW_REF_FRAMES(codec, total_hp_layers, ltr_count, num_ref) {   \
	if (codec == MSM_VIDC_HEVC) {                                          \
		num_ref = (total_hp_layers + 1) >> 1;                          \
	} else if (codec == MSM_VIDC_H264) {                                   \
		if (total_hp_layers < 4)                                       \
			num_ref = (total_hp_layers - 1);                       \
		else                                                           \
			num_ref = total_hp_layers;                             \
	}                                                                      \
	if (ltr_count)                                                         \
		num_ref = num_ref + ltr_count;                                 \
}

/*
 * Custom conversion coefficients for resolution: 176x144 negative
 * coeffs are converted to s4.9 format
 * (e.g. -22 converted to ((1 << 13) - 22)
 * 3x3 transformation matrix coefficients in s4.9 fixed point format
 */
u32 vpe_csc_custom_matrix_coeff[MAX_MATRIX_COEFFS] = {
	440, 8140, 8098, 0, 460, 52, 0, 34, 463
};

/* offset coefficients in s9 fixed point format */
u32 vpe_csc_custom_bias_coeff[MAX_BIAS_COEFFS] = {
	53, 0, 4
};

/* clamping value for Y/U/V([min,max] for Y/U/V) */
u32 vpe_csc_custom_limit_coeff[MAX_LIMIT_COEFFS] = {
	16, 235, 16, 240, 16, 240
};

static struct v4l2_file_operations msm_v4l2_file_operations = {
	.owner                          = THIS_MODULE,
	.open                           = msm_v4l2_open,
	.release                        = msm_v4l2_close,
	.unlocked_ioctl                 = video_ioctl2,
	.poll                           = msm_v4l2_poll,
};

static struct v4l2_ioctl_ops msm_v4l2_ioctl_ops_enc = {
	.vidioc_querycap                = msm_v4l2_querycap,
	.vidioc_enum_fmt_vid_cap        = msm_v4l2_enum_fmt,
	.vidioc_enum_fmt_vid_out        = msm_v4l2_enum_fmt,
	.vidioc_enum_fmt_meta_cap       = msm_v4l2_enum_fmt,
	.vidioc_enum_fmt_meta_out       = msm_v4l2_enum_fmt,
	.vidioc_enum_framesizes         = msm_v4l2_enum_framesizes,
	.vidioc_enum_frameintervals     = msm_v4l2_enum_frameintervals,
	.vidioc_try_fmt_vid_cap_mplane  = msm_v4l2_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = msm_v4l2_try_fmt,
	.vidioc_try_fmt_meta_cap        = msm_v4l2_try_fmt,
	.vidioc_try_fmt_meta_out        = msm_v4l2_try_fmt,
	.vidioc_s_fmt_vid_cap           = msm_v4l2_s_fmt,
	.vidioc_s_fmt_vid_out           = msm_v4l2_s_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = msm_v4l2_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = msm_v4l2_s_fmt,
	.vidioc_s_fmt_meta_out          = msm_v4l2_s_fmt,
	.vidioc_s_fmt_meta_cap          = msm_v4l2_s_fmt,
	.vidioc_g_fmt_vid_cap           = msm_v4l2_g_fmt,
	.vidioc_g_fmt_vid_out           = msm_v4l2_g_fmt,
	.vidioc_g_fmt_vid_cap_mplane    = msm_v4l2_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = msm_v4l2_g_fmt,
	.vidioc_g_fmt_meta_out          = msm_v4l2_g_fmt,
	.vidioc_g_fmt_meta_cap          = msm_v4l2_g_fmt,
	.vidioc_g_selection             = msm_v4l2_g_selection,
	.vidioc_s_selection             = msm_v4l2_s_selection,
	.vidioc_s_parm                  = msm_v4l2_s_parm,
	.vidioc_g_parm                  = msm_v4l2_g_parm,
	.vidioc_reqbufs                 = msm_v4l2_reqbufs,
	.vidioc_querybuf                = msm_v4l2_querybuf,
	.vidioc_create_bufs             = msm_v4l2_create_bufs,
	.vidioc_prepare_buf             = msm_v4l2_prepare_buf,
	.vidioc_qbuf                    = msm_v4l2_qbuf,
	.vidioc_dqbuf                   = msm_v4l2_dqbuf,
	.vidioc_streamon                = msm_v4l2_streamon,
	.vidioc_streamoff               = msm_v4l2_streamoff,
	.vidioc_queryctrl               = msm_v4l2_queryctrl,
	.vidioc_querymenu               = msm_v4l2_querymenu,
	.vidioc_subscribe_event         = msm_v4l2_subscribe_event,
	.vidioc_unsubscribe_event       = msm_v4l2_unsubscribe_event,
	.vidioc_try_encoder_cmd         = msm_v4l2_try_encoder_cmd,
	.vidioc_encoder_cmd             = msm_v4l2_encoder_cmd,
};

static struct v4l2_ioctl_ops msm_v4l2_ioctl_ops_dec = {
	.vidioc_querycap                = msm_v4l2_querycap,
	.vidioc_enum_fmt_vid_cap        = msm_v4l2_enum_fmt,
	.vidioc_enum_fmt_vid_out        = msm_v4l2_enum_fmt,
	.vidioc_enum_fmt_meta_cap       = msm_v4l2_enum_fmt,
	.vidioc_enum_fmt_meta_out       = msm_v4l2_enum_fmt,
	.vidioc_enum_framesizes         = msm_v4l2_enum_framesizes,
	.vidioc_enum_frameintervals     = msm_v4l2_enum_frameintervals,
	.vidioc_try_fmt_vid_cap_mplane  = msm_v4l2_try_fmt,
	.vidioc_try_fmt_vid_out_mplane  = msm_v4l2_try_fmt,
	.vidioc_try_fmt_meta_cap        = msm_v4l2_try_fmt,
	.vidioc_try_fmt_meta_out        = msm_v4l2_try_fmt,
	.vidioc_s_fmt_vid_cap           = msm_v4l2_s_fmt,
	.vidioc_s_fmt_vid_out           = msm_v4l2_s_fmt,
	.vidioc_s_fmt_vid_cap_mplane    = msm_v4l2_s_fmt,
	.vidioc_s_fmt_vid_out_mplane    = msm_v4l2_s_fmt,
	.vidioc_s_fmt_meta_out          = msm_v4l2_s_fmt,
	.vidioc_s_fmt_meta_cap          = msm_v4l2_s_fmt,
	.vidioc_g_fmt_vid_cap           = msm_v4l2_g_fmt,
	.vidioc_g_fmt_vid_out           = msm_v4l2_g_fmt,
	.vidioc_g_fmt_vid_cap_mplane    = msm_v4l2_g_fmt,
	.vidioc_g_fmt_vid_out_mplane    = msm_v4l2_g_fmt,
	.vidioc_g_fmt_meta_out          = msm_v4l2_g_fmt,
	.vidioc_g_fmt_meta_cap          = msm_v4l2_g_fmt,
	.vidioc_g_selection             = msm_v4l2_g_selection,
	.vidioc_s_selection             = msm_v4l2_s_selection,
	.vidioc_reqbufs                 = msm_v4l2_reqbufs,
	.vidioc_querybuf                = msm_v4l2_querybuf,
	.vidioc_create_bufs             = msm_v4l2_create_bufs,
	.vidioc_prepare_buf             = msm_v4l2_prepare_buf,
	.vidioc_qbuf                    = msm_v4l2_qbuf,
	.vidioc_dqbuf                   = msm_v4l2_dqbuf,
	.vidioc_streamon                = msm_v4l2_streamon,
	.vidioc_streamoff               = msm_v4l2_streamoff,
	.vidioc_queryctrl               = msm_v4l2_queryctrl,
	.vidioc_querymenu               = msm_v4l2_querymenu,
	.vidioc_subscribe_event         = msm_v4l2_subscribe_event,
	.vidioc_unsubscribe_event       = msm_v4l2_unsubscribe_event,
	.vidioc_try_decoder_cmd         = msm_v4l2_try_decoder_cmd,
	.vidioc_decoder_cmd             = msm_v4l2_decoder_cmd,
};

static struct v4l2_ctrl_ops msm_v4l2_ctrl_ops = {
	.s_ctrl                         = msm_v4l2_op_s_ctrl,
	.g_volatile_ctrl                = msm_v4l2_op_g_volatile_ctrl,
};

static struct vb2_ops msm_vb2_ops = {
	.queue_setup                    = msm_vb2_queue_setup,
	.start_streaming                = msm_vb2_start_streaming,
	.buf_queue                      = msm_vb2_buf_queue,
	.stop_streaming                 = msm_vb2_stop_streaming,
	.buf_out_validate               = msm_vb2_buf_out_validate,
	.buf_request_complete           = msm_vb2_request_complete,
};

static struct vb2_mem_ops msm_vb2_mem_ops = {
	.alloc                          = msm_vb2_alloc,
	.put                            = msm_vb2_put,
	.mmap                           = msm_vb2_mmap,
	.attach_dmabuf                  = msm_vb2_attach_dmabuf,
	.detach_dmabuf                  = msm_vb2_detach_dmabuf,
	.map_dmabuf                     = msm_vb2_map_dmabuf,
	.unmap_dmabuf                   = msm_vb2_unmap_dmabuf,
};

static struct media_device_ops msm_v4l2_media_ops = {
	.req_validate                   = msm_v4l2_request_validate,
	.req_queue                      = msm_v4l2_request_queue,
};

static struct v4l2_m2m_ops msm_v4l2_m2m_ops = {
	.device_run                     = msm_v4l2_m2m_device_run,
	.job_abort                      = msm_v4l2_m2m_job_abort,
};

static const struct msm_vidc_compat_handle compat_handle[] = {
#if defined(CONFIG_MSM_VIDC_PINEAPPLE)
	{
		.compat                     = "qcom,sm8650-vidc",
		.init_platform              = msm_vidc_init_platform_pineapple,
		.init_iris                  = msm_vidc_init_iris33,
	},
	{
		.compat                     = "qcom,sm8650-vidc-v2",
		.init_platform              = msm_vidc_init_platform_pineapple,
		.init_iris                  = msm_vidc_init_iris33,
	},
	{
		.compat                     = "qcom,cliffs-vidc",
		.init_platform              = msm_vidc_init_platform_cliffs,
		.init_iris                  = msm_vidc_init_iris33,
	},
#endif
#if defined(CONFIG_MSM_VIDC_KALAMA)
	{
		.compat                     = "qcom,sm8550-vidc",
		.init_platform              = msm_vidc_init_platform_kalama,
		.init_iris                  = msm_vidc_init_iris3,
	},
	{
		.compat                     = "qcom,sm8550-vidc-v2",
		.init_platform              = msm_vidc_init_platform_kalama,
		.init_iris                  = msm_vidc_init_iris3,
	},
#endif
#if defined(CONFIG_MSM_VIDC_WAIPIO)
	{
		.compat                     = "qcom,sm8450-vidc",
		.init_platform              = msm_vidc_init_platform_waipio,
		.init_iris                  = msm_vidc_init_iris2,
	},
#endif
};

static int msm_vidc_init_ops(struct msm_vidc_core *core)
{
	d_vpr_h("%s: initialize ops\n", __func__);
	core->v4l2_file_ops = &msm_v4l2_file_operations;
	core->v4l2_ioctl_ops_enc = &msm_v4l2_ioctl_ops_enc;
	core->v4l2_ioctl_ops_dec = &msm_v4l2_ioctl_ops_dec;
	core->v4l2_ctrl_ops = &msm_v4l2_ctrl_ops;
	core->vb2_ops = &msm_vb2_ops;
	core->vb2_mem_ops = &msm_vb2_mem_ops;
	core->media_device_ops = &msm_v4l2_media_ops;
	core->v4l2_m2m_ops = &msm_v4l2_m2m_ops;
	core->mem_ops = get_mem_ops();
	if (!core->mem_ops) {
		d_vpr_e("%s: invalid memory ops\n", __func__);
		return -EINVAL;
	}
	core->res_ops = get_resources_ops();
	if (!core->res_ops) {
		d_vpr_e("%s: invalid resource ops\n", __func__);
		return -EINVAL;
	}
	core->fence_ops = get_dma_fence_ops();
	if (!core->fence_ops) {
		d_vpr_e("%s: invalid dma fence ops\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int msm_vidc_init_platform_variant(struct msm_vidc_core *core)
{
	struct device *dev = NULL;
	int i, rc = 0;

	dev = &core->pdev->dev;

	d_vpr_h("%s()\n", __func__);

	/* select platform based on compatible match */
	for (i = 0; i < ARRAY_SIZE(compat_handle); i++) {
		if (of_device_is_compatible(dev->of_node, compat_handle[i].compat)) {
			rc = compat_handle[i].init_platform(core);
			if (rc) {
				d_vpr_e("%s: (%s) init failed with %d\n",
					__func__, compat_handle[i].compat, rc);
				return rc;
			}
			break;
		}
	}

	/* handle unknown compat type */
	if (i == ARRAY_SIZE(compat_handle)) {
		d_vpr_e("%s: Unsupported device: (%s)\n", __func__, dev_name(dev));
		return -EINVAL;
	}

	return rc;
}

static int msm_vidc_init_vpu(struct msm_vidc_core *core)
{
	struct device *dev = NULL;
	int i, rc = 0;

	dev = &core->pdev->dev;

	/* select platform based on compatible match */
	for (i = 0; i < ARRAY_SIZE(compat_handle); i++) {
		if (of_device_is_compatible(dev->of_node, compat_handle[i].compat)) {
			rc = compat_handle[i].init_iris(core);
			if (rc) {
				d_vpr_e("%s: (%s) init failed with %d\n",
					__func__, compat_handle[i].compat, rc);
				return rc;
			}
			break;
		}
	}

	/* handle unknown compat type */
	if (i == ARRAY_SIZE(compat_handle)) {
		d_vpr_e("%s: Unsupported device: (%s)\n", __func__, dev_name(dev));
		return -EINVAL;
	}

	return rc;
}

int msm_vidc_init_platform(struct msm_vidc_core *core)
{
	int rc = 0;
	struct msm_vidc_platform *platform = NULL;

	d_vpr_h("%s()\n", __func__);

	platform = devm_kzalloc(&core->pdev->dev,
				sizeof(struct msm_vidc_platform), GFP_KERNEL);
	if (!platform) {
		d_vpr_e("%s: failed to alloc memory for platform\n", __func__);
		return -ENOMEM;
	}

	core->platform = platform;

	/* selected ops can be re-assigned in platform specific file */
	rc = msm_vidc_init_ops(core);
	if (rc)
		return rc;

	rc = msm_vidc_init_platform_variant(core);
	if (rc)
		return rc;

	rc = msm_vidc_init_vpu(core);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_read_efuse(struct msm_vidc_core *core)
{
	int rc = 0;
	void __iomem *base;
	u32 i = 0, efuse = 0, efuse_data_count = 0;
	struct msm_vidc_efuse_data *efuse_data = NULL;
	struct msm_vidc_platform_data *platform_data;

	platform_data = &core->platform->data;
	efuse_data = platform_data->efuse_data;
	efuse_data_count = platform_data->efuse_data_size;

	if (!efuse_data)
		return 0;

	for (i = 0; i < efuse_data_count; i++) {
		switch (efuse_data[i].purpose) {
		case SKU_VERSION:
			base = devm_ioremap(&core->pdev->dev, efuse_data[i].start_address,
					efuse_data[i].size);
			if (!base) {
				d_vpr_e("failed efuse: start %#x, size %d\n",
					efuse_data[i].start_address,
					efuse_data[i].size);
				return -EINVAL;
			}
			efuse = readl_relaxed(base);
			platform_data->sku_version =
					(efuse & efuse_data[i].mask) >>
					efuse_data[i].shift;
			break;
		default:
			break;
		}
		if (platform_data->sku_version) {
			d_vpr_h("efuse 0x%x, platform version 0x%x\n",
				efuse, platform_data->sku_version);
			break;
		}
	}
	return rc;
}

/****************** control framework utility functions **********************/

enum msm_vidc_inst_capability_type msm_vidc_get_cap_id(struct msm_vidc_inst *inst, u32 id)
{
	enum msm_vidc_inst_capability_type i = INST_CAP_NONE + 1;

	enum msm_vidc_inst_capability_type cap_id = INST_CAP_NONE;

	do {
		if (inst->capabilities[i].v4l2_id == id) {
			cap_id = inst->capabilities[i].cap_id;
			break;
		}
		i++;
	} while (i < INST_CAP_MAX);

	return cap_id;
}

int msm_vidc_update_cap_value(struct msm_vidc_inst *inst, u32 cap_id,
			      s32 adjusted_val, const char *func)
{
	int prev_value = 0;

	prev_value = inst->capabilities[cap_id].value;

	if (is_meta_cap(inst, cap_id)) {
		if (adjusted_val & MSM_VIDC_META_ENABLE &&
		    adjusted_val & MSM_VIDC_META_DYN_ENABLE) {
			i_vpr_e(inst,
				"%s: %s cannot be enabled both statically and dynamically",
				__func__, cap_name(cap_id));
			return -EINVAL;
		}
		/*
		 * cumulative control value if client set same metadata
		 * control multiple times.
		 */
		if (adjusted_val & MSM_VIDC_META_ENABLE ||
		    adjusted_val & MSM_VIDC_META_DYN_ENABLE) {
			/* enable metadata */
			inst->capabilities[cap_id].value |= adjusted_val;
		} else {
			/* disable metadata */
			inst->capabilities[cap_id].value &= ~adjusted_val;
		}
	} else {
		inst->capabilities[cap_id].value = adjusted_val;
	}

	if (prev_value != inst->capabilities[cap_id].value) {
		i_vpr_h(inst,
			"%s: updated database: name: %s, value: %#x -> %#x\n",
			func, cap_name(cap_id),
			prev_value, inst->capabilities[cap_id].value);
	}

	return 0;
}

bool is_parent_available(struct msm_vidc_inst *inst,
			 u32 cap_id, u32 check_parent, const char *func)
{
	int i = 0;
	u32 cap_child;

	if (!is_valid_cap_id(cap_id) || !is_valid_cap_id(check_parent))
		return false;

	while (i < MAX_CAP_CHILDREN &&
	       inst->capabilities[check_parent].children[i]) {
		cap_child = inst->capabilities[check_parent].children[i];
		if (cap_child == cap_id)
			return true;
		i++;
	}

	i_vpr_e(inst,
		"%s: missing parent %s for %s\n",
		func, cap_name(check_parent), cap_name(cap_id));
	return false;
}

int msm_vidc_get_parent_value(struct msm_vidc_inst *inst,
			      u32 cap_id, u32 parent, s32 *value, const char *func)
{
	int rc = 0;

	if (is_parent_available(inst, cap_id, parent, func)) {
		switch (parent) {
		case BITRATE_MODE:
			*value = inst->hfi_rc_type;
			break;
		case LAYER_TYPE:
			*value = inst->hfi_layer_type;
			break;
		default:
			*value = inst->capabilities[parent].value;
			break;
		}
	} else {
		rc = -EINVAL;
	}

	return rc;
}

u32 msm_vidc_get_port_info(struct msm_vidc_inst *inst,
			   enum msm_vidc_inst_capability_type cap_id)
{
	if (inst->capabilities[cap_id].flags & CAP_FLAG_INPUT_PORT &&
	    inst->capabilities[cap_id].flags & CAP_FLAG_OUTPUT_PORT) {
		if (inst->bufq[OUTPUT_PORT].vb2q->streaming)
			return get_hfi_port(inst, INPUT_PORT);
		else
			return get_hfi_port(inst, OUTPUT_PORT);
	}

	if (inst->capabilities[cap_id].flags & CAP_FLAG_INPUT_PORT)
		return get_hfi_port(inst, INPUT_PORT);
	else if (inst->capabilities[cap_id].flags & CAP_FLAG_OUTPUT_PORT)
		return get_hfi_port(inst, OUTPUT_PORT);
	else
		return HFI_PORT_NONE;
}

int msm_vidc_v4l2_menu_to_hfi(struct msm_vidc_inst *inst,
			      enum msm_vidc_inst_capability_type cap_id, u32 *value)
{
	switch (cap_id) {
	case ENTROPY_MODE:
		switch (inst->capabilities[cap_id].value) {
		case V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC:
			*value = 1;
			break;
		case V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC:
			*value = 0;
			break;
		default:
			*value = 1;
			goto set_default;
		}
		return 0;
	default:
		i_vpr_e(inst,
			"%s: mapping not specified for ctrl_id: %#x\n",
			__func__, inst->capabilities[cap_id].v4l2_id);
		return -EINVAL;
	}

set_default:
	i_vpr_e(inst,
		"%s: invalid value %d for ctrl id: %#x. Set default: %u\n",
		__func__, inst->capabilities[cap_id].value,
		inst->capabilities[cap_id].v4l2_id, *value);
	return 0;
}

int msm_vidc_v4l2_to_hfi_enum(struct msm_vidc_inst *inst,
			      enum msm_vidc_inst_capability_type cap_id, u32 *value)
{
	switch (cap_id) {
	case BITRATE_MODE:
		*value = inst->hfi_rc_type;
		return 0;
	case PROFILE:
	case LEVEL:
	case HEVC_TIER:
	case AV1_TIER:
	case BLUR_TYPES:
		*value = inst->capabilities[cap_id].value;
		return 0;
	case LAYER_TYPE:
		if (inst->codec == MSM_VIDC_HEVC) {
			switch (inst->capabilities[cap_id].value) {
			case V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_B:
				*value = HFI_HIER_B;
				break;
			case V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_P:
				//TODO (AS): check if this is right mapping
				*value = HFI_HIER_P_SLIDING_WINDOW;
				break;
			default:
				*value = HFI_HIER_P_SLIDING_WINDOW;
				goto set_default;
			}
		}
		return 0;
	case ROTATION:
		switch (inst->capabilities[cap_id].value) {
		case 0:
			*value = HFI_ROTATION_NONE;
			break;
		case 90:
			*value = HFI_ROTATION_90;
			break;
		case 180:
			*value = HFI_ROTATION_180;
			break;
		case 270:
			*value = HFI_ROTATION_270;
			break;
		default:
			*value = HFI_ROTATION_NONE;
			goto set_default;
		}
		return 0;
	case LF_MODE:
		if (inst->codec == MSM_VIDC_HEVC) {
			switch (inst->capabilities[cap_id].value) {
			case V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_ENABLED:
				*value = HFI_DEBLOCK_ALL_BOUNDARY;
				break;
			case V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED:
				*value = HFI_DEBLOCK_DISABLE;
				break;
			case DB_HEVC_DISABLE_SLICE_BOUNDARY:
				*value = HFI_DEBLOCK_DISABLE_AT_SLICE_BOUNDARY;
				break;
			default:
				*value = HFI_DEBLOCK_ALL_BOUNDARY;
				goto set_default;
			}
		} else if (inst->codec == MSM_VIDC_H264) {
			switch (inst->capabilities[cap_id].value) {
			case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED:
				*value = HFI_DEBLOCK_ALL_BOUNDARY;
				break;
			case V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED:
				*value = HFI_DEBLOCK_DISABLE;
				break;
			case DB_H264_DISABLE_SLICE_BOUNDARY:
				*value = HFI_DEBLOCK_DISABLE_AT_SLICE_BOUNDARY;
				break;
			default:
				*value = HFI_DEBLOCK_ALL_BOUNDARY;
				goto set_default;
			}
		}
		return 0;
	case NAL_LENGTH_FIELD:
		switch (inst->capabilities[cap_id].value) {
		case V4L2_MPEG_VIDEO_HEVC_SIZE_4:
			*value = HFI_NAL_LENGTH_SIZE_4;
			break;
		default:
			*value = HFI_NAL_LENGTH_STARTCODES;
			goto set_default;
		}
		return 0;
	default:
		i_vpr_e(inst,
			"%s: mapping not specified for ctrl_id: %#x\n",
			__func__, inst->capabilities[cap_id].v4l2_id);
		return -EINVAL;
	}

set_default:
	i_vpr_e(inst,
		"%s: invalid value %d for ctrl id: %#x. Set default: %u\n",
		__func__, inst->capabilities[cap_id].value,
		inst->capabilities[cap_id].v4l2_id, *value);
	return 0;
}

int msm_vidc_packetize_control(struct msm_vidc_inst *inst,
			       enum msm_vidc_inst_capability_type cap_id, u32 payload_type,
			       void *hfi_val, u32 payload_size, const char *func)
{
	int rc = 0;
	u64 payload = 0;

	if (payload_size > sizeof(u32)) {
		i_vpr_e(inst, "%s: payload size is more than u32 for cap[%d] %s\n",
			func, cap_id, cap_name(cap_id));
		return -EINVAL;
	}

	if (payload_size == sizeof(u32))
		payload = *(u32 *)hfi_val;
	else if (payload_size == sizeof(u8))
		payload = *(u8 *)hfi_val;
	else if (payload_size == sizeof(u16))
		payload = *(u16 *)hfi_val;

	i_vpr_h(inst, FMT_STRING_SET_CAP,
		cap_name(cap_id), inst->capabilities[cap_id].value, payload);

	rc = venus_hfi_session_property(inst,
					inst->capabilities[cap_id].hfi_id,
					HFI_HOST_FLAGS_NONE,
					msm_vidc_get_port_info(inst, cap_id),
					payload_type,
					hfi_val,
					payload_size);
	if (rc) {
		i_vpr_e(inst, "%s: failed to set cap[%d] %s to fw\n",
			func, cap_id, cap_name(cap_id));
		return rc;
	}

	return 0;
}

/*************** End of control framework utility functions ******************/

/*********************** Control Adjust functions ****************************/

int msm_vidc_adjust_entropy_mode(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 profile = -1;

	/* ctrl is always NULL in streamon case */
	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[ENTROPY_MODE].value;

	if (inst->codec != MSM_VIDC_H264) {
		i_vpr_e(inst,
			"%s: incorrect entry in database. fix the database\n",
			__func__);
		return 0;
	}

	if (msm_vidc_get_parent_value(inst, ENTROPY_MODE, PROFILE, &profile, __func__))
		return -EINVAL;

	if (profile == V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE ||
	    profile == V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE)
		adjusted_value = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC;

	msm_vidc_update_cap_value(inst, ENTROPY_MODE, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_bitrate_mode(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	int lossless, frame_rc, bitrate_mode, frame_skip;
	u32 hfi_value = 0;

	bitrate_mode = inst->capabilities[BITRATE_MODE].value;
	lossless = inst->capabilities[LOSSLESS].value;
	frame_rc = inst->capabilities[FRAME_RC_ENABLE].value;
	frame_skip = inst->capabilities[FRAME_SKIP_MODE].value;

	if (lossless ||
	    (msm_vidc_lossless_encode && inst->codec == MSM_VIDC_HEVC)) {
		hfi_value = HFI_RC_LOSSLESS;
		goto update;
	}

	if (!frame_rc && !is_image_session(inst)) {
		hfi_value = HFI_RC_OFF;
		goto update;
	}

	if (bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
		hfi_value = HFI_RC_VBR_CFR;
	} else if (bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR) {
		if (frame_skip)
			hfi_value = HFI_RC_CBR_VFR;
		else
			hfi_value = HFI_RC_CBR_CFR;
	} else if (bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CQ) {
		hfi_value = HFI_RC_CQ;
	}

update:
	inst->hfi_rc_type = hfi_value;
	i_vpr_h(inst, "%s: hfi rc type: %#x\n",
		__func__, inst->hfi_rc_type);

	return 0;
}

int msm_vidc_adjust_profile(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 pix_fmt = -1;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[PROFILE].value;

	/* PIX_FMTS dependency is common across all chipsets.
	 * Hence, PIX_FMTS must be specified as Parent for HEVC profile.
	 * Otherwise it would be a database error that should be fixed.
	 */
	if (msm_vidc_get_parent_value(inst, PROFILE, PIX_FMTS,
				      &pix_fmt, __func__))
		return -EINVAL;

	/* 10 bit profile for 10 bit color format */
	if (pix_fmt == MSM_VIDC_FMT_TP10C || pix_fmt == MSM_VIDC_FMT_P010) {
		if (is_image_session(inst))
			adjusted_value = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10_STILL_PICTURE;
		else
			adjusted_value = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10;
	} else {
		/* 8 bit profile for 8 bit color format */
		if (is_image_session(inst))
			adjusted_value = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE;
		else
			adjusted_value = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
	}

	msm_vidc_update_cap_value(inst, PROFILE, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_ltr_count(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1, all_intra = 0, pix_fmts = MSM_VIDC_FMT_NONE;
	s32 layer_type = -1, enh_layer_count = -1;
	u32 num_ref_frames = 0, max_exceeding_ref_frames = 0;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[LTR_COUNT].value;

	if (msm_vidc_get_parent_value(inst, LTR_COUNT, BITRATE_MODE,
				      &rc_type, __func__))
		return -EINVAL;

	if ((rc_type != HFI_RC_OFF &&
	     rc_type != HFI_RC_CBR_CFR &&
	     rc_type != HFI_RC_CBR_VFR)) {
		adjusted_value = 0;
		i_vpr_h(inst,
			"%s: ltr count unsupported, rc_type: %#x\n",
			__func__, rc_type);
		goto exit;
	}

	if (is_valid_cap(inst, ALL_INTRA)) {
		if (msm_vidc_get_parent_value(inst, LTR_COUNT,
					      ALL_INTRA, &all_intra, __func__))
			return -EINVAL;
		if (all_intra) {
			adjusted_value = 0;
			goto exit;
		}
	}

	if (!msm_vidc_get_parent_value(inst, LTR_COUNT, PIX_FMTS,
				       &pix_fmts, __func__)) {
		if (is_10bit_colorformat(pix_fmts))
			adjusted_value = 0;
	}

	if (!msm_vidc_get_parent_value(inst, LTR_COUNT, ENH_LAYER_COUNT,
				       &enh_layer_count, __func__) &&
	    !msm_vidc_get_parent_value(inst, LTR_COUNT, LAYER_TYPE,
				       &layer_type, __func__)) {
		if (layer_type == HFI_HIER_P_SLIDING_WINDOW) {
			SLIDING_WINDOW_REF_FRAMES(inst->codec,
						  inst->capabilities[ENH_LAYER_COUNT].value + 1,
						  adjusted_value, num_ref_frames);
			if (num_ref_frames > MAX_ENCODING_REFERNCE_FRAMES) {
				/*
				 * reduce ltr count to avoid num ref
				 * frames going beyond limit
				 */
				max_exceeding_ref_frames = num_ref_frames -
					MAX_ENCODING_REFERNCE_FRAMES;
				if (adjusted_value >= max_exceeding_ref_frames)
					adjusted_value -= max_exceeding_ref_frames;
				else
					adjusted_value = 0;
			}
		}
		i_vpr_h(inst,
			"%s: ltr count %d enh_layers %d layer_type %d\n",
			__func__, adjusted_value,
			inst->capabilities[ENH_LAYER_COUNT].value,
			layer_type);
	}

exit:
	msm_vidc_update_cap_value(inst, LTR_COUNT, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_use_ltr(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value, ltr_count;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[USE_LTR].value;

	/*
	 * Since USE_LTR is only set dynamically, and LTR_COUNT is static
	 * control, no need to make LTR_COUNT as parent for USE_LTR as
	 * LTR_COUNT value will always be updated when dynamically USE_LTR
	 * is set
	 */
	ltr_count = inst->capabilities[LTR_COUNT].value;
	if (!ltr_count)
		return 0;

	if (adjusted_value <= 0 ||
	    adjusted_value > ((1 << ltr_count) - 1)) {
		/*
		 * USE_LTR is bitmask value, hence should be
		 * > 0 and <= (2 ^ LTR_COUNT) - 1
		 */
		i_vpr_e(inst, "%s: invalid value %d\n",
			__func__, adjusted_value);
		return 0;
	}

	/* USE_LTR value is a bitmask value */
	msm_vidc_update_cap_value(inst, USE_LTR, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_mark_ltr(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value, ltr_count;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[MARK_LTR].value;

	/*
	 * Since MARK_LTR is only set dynamically, and LTR_COUNT is static
	 * control, no need to make LTR_COUNT as parent for MARK_LTR as
	 * LTR_COUNT value will always be updated when dynamically MARK_LTR
	 * is set
	 */
	ltr_count = inst->capabilities[LTR_COUNT].value;
	if (!ltr_count)
		return 0;

	if (adjusted_value < 0 ||
	    adjusted_value > (ltr_count - 1)) {
		/* MARK_LTR value should be >= 0 and <= (LTR_COUNT - 1) */
		i_vpr_e(inst, "%s: invalid value %d\n",
			__func__, adjusted_value);
		return 0;
	}

	msm_vidc_update_cap_value(inst, MARK_LTR, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_delta_based_rc(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[TIME_DELTA_BASED_RC].value;

	if (msm_vidc_get_parent_value(inst, TIME_DELTA_BASED_RC, BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	if (rc_type == HFI_RC_OFF || rc_type == HFI_RC_CQ)
		adjusted_value = 0;

	msm_vidc_update_cap_value(inst, TIME_DELTA_BASED_RC, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_output_order(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	s32 tn_mode = -1, display_delay = -1, display_delay_enable = -1;
	u32 adjusted_value;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[OUTPUT_ORDER].value;

	if (msm_vidc_get_parent_value(inst, OUTPUT_ORDER, DISPLAY_DELAY,
				      &display_delay, __func__) ||
		msm_vidc_get_parent_value(inst, OUTPUT_ORDER, DISPLAY_DELAY_ENABLE,
					  &display_delay_enable, __func__))
		return -EINVAL;

	if (display_delay_enable && !display_delay)
		adjusted_value = 1;

	if (is_valid_cap(inst, THUMBNAIL_MODE)) {
		if (msm_vidc_get_parent_value(inst, OUTPUT_ORDER, THUMBNAIL_MODE,
					      &tn_mode, __func__))
			return -EINVAL;

		if (tn_mode == 1)
			adjusted_value = 1;
	}

	msm_vidc_update_cap_value(inst, OUTPUT_ORDER, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_input_buf_host_max_count(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	u32 adjusted_value;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[INPUT_BUF_HOST_MAX_COUNT].value;

	if (msm_vidc_is_super_buffer(inst) || is_image_session(inst))
		adjusted_value = DEFAULT_MAX_HOST_BURST_BUF_COUNT;

	msm_vidc_update_cap_value(inst, INPUT_BUF_HOST_MAX_COUNT, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_output_buf_host_max_count(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	u32 adjusted_value;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[OUTPUT_BUF_HOST_MAX_COUNT].value;

	if (msm_vidc_is_super_buffer(inst) || is_image_session(inst) ||
	    is_enc_slice_delivery_mode(inst))
		adjusted_value = DEFAULT_MAX_HOST_BURST_BUF_COUNT;

	msm_vidc_update_cap_value(inst, OUTPUT_BUF_HOST_MAX_COUNT, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_transform_8x8(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 profile = -1;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[TRANSFORM_8X8].value;

	if (inst->codec != MSM_VIDC_H264) {
		i_vpr_e(inst,
			"%s: incorrect entry in database. fix the database\n",
			__func__);
		return 0;
	}

	if (msm_vidc_get_parent_value(inst, TRANSFORM_8X8,
				      PROFILE, &profile, __func__))
		return -EINVAL;

	if (profile != V4L2_MPEG_VIDEO_H264_PROFILE_HIGH &&
	    profile != V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH)
		adjusted_value = 0;

	msm_vidc_update_cap_value(inst, TRANSFORM_8X8, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_chroma_qp_index_offset(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[CHROMA_QP_INDEX_OFFSET].value;

	if (adjusted_value != MIN_CHROMA_QP_OFFSET)
		adjusted_value = MAX_CHROMA_QP_OFFSET;

	msm_vidc_update_cap_value(inst, CHROMA_QP_INDEX_OFFSET, adjusted_value, __func__);

	return 0;
}

static bool msm_vidc_check_all_layer_bitrate_set(struct msm_vidc_inst *inst)
{
	bool layer_bitrate_set = true;
	u32 cap_id = 0, i, enh_layer_count;
	u32 layer_br_caps[6] = {L0_BR, L1_BR, L2_BR, L3_BR, L4_BR, L5_BR};

	enh_layer_count = inst->capabilities[ENH_LAYER_COUNT].value;

	for (i = 0; i <= enh_layer_count; i++) {
		if (i >= ARRAY_SIZE(layer_br_caps))
			break;
		cap_id = layer_br_caps[i];
		if (!(inst->capabilities[cap_id].flags & CAP_FLAG_CLIENT_SET)) {
			layer_bitrate_set = false;
			break;
		}
	}

	return layer_bitrate_set;
}

static u32 msm_vidc_get_cumulative_bitrate(struct msm_vidc_inst *inst)
{
	int i;
	u32 cap_id = 0;
	u32 cumulative_br = 0;
	s32 enh_layer_count;
	u32 layer_br_caps[6] = {L0_BR, L1_BR, L2_BR, L3_BR, L4_BR, L5_BR};

	enh_layer_count = inst->capabilities[ENH_LAYER_COUNT].value;

	for (i = 0; i <= enh_layer_count; i++) {
		if (i >= ARRAY_SIZE(layer_br_caps))
			break;
		cap_id = layer_br_caps[i];
		cumulative_br += inst->capabilities[cap_id].value;
	}

	return cumulative_br;
}

int msm_vidc_adjust_slice_count(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	struct v4l2_format *output_fmt;
	s32 adjusted_value, rc_type = -1, slice_mode, all_intra = 0,
		enh_layer_count = 0;
	u32 slice_val, mbpf = 0, mbps = 0, max_mbpf = 0, max_mbps = 0, bitrate = 0;
	u32 update_cap, max_avg_slicesize, output_width, output_height;
	u32 min_width, min_height, max_width, max_height, fps;

	slice_mode = ctrl ? ctrl->val :
		inst->capabilities[SLICE_MODE].value;

	if (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE)
		return 0;

	bitrate = inst->capabilities[BIT_RATE].value;

	if (msm_vidc_get_parent_value(inst, SLICE_MODE,
				      BITRATE_MODE, &rc_type, __func__) ||
	    msm_vidc_get_parent_value(inst, SLICE_MODE,
				      ENH_LAYER_COUNT, &enh_layer_count, __func__))
		return -EINVAL;

	if (enh_layer_count && msm_vidc_check_all_layer_bitrate_set(inst))
		bitrate = msm_vidc_get_cumulative_bitrate(inst);

	fps = inst->capabilities[FRAME_RATE].value >> 16;
	if (fps > MAX_SLICES_FRAME_RATE ||
	    (rc_type != HFI_RC_OFF && rc_type != HFI_RC_CBR_CFR &&
	    rc_type != HFI_RC_CBR_VFR && rc_type != HFI_RC_VBR_CFR)) {
		adjusted_value = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE;
		update_cap = SLICE_MODE;
		i_vpr_h(inst,
			"%s: slice unsupported, fps: %u, rc_type: %#x\n",
			__func__, fps, rc_type);
		goto exit;
	}

	if (is_valid_cap(inst, ALL_INTRA)) {
		if (msm_vidc_get_parent_value(inst, SLICE_MODE,
					      ALL_INTRA, &all_intra, __func__))
			return -EINVAL;

		if (all_intra == 1) {
			adjusted_value = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE;
			update_cap = SLICE_MODE;
			i_vpr_h(inst, "%s: slice unsupported, all_intra %d\n", __func__, all_intra);
			goto exit;
		}
	}

	output_fmt = &inst->fmts[OUTPUT_PORT];
	output_width = output_fmt->fmt.pix_mp.width;
	output_height = output_fmt->fmt.pix_mp.height;

	max_width = (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB) ?
		MAX_MB_SLICE_WIDTH : MAX_BYTES_SLICE_WIDTH;
	max_height = (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB) ?
		MAX_MB_SLICE_HEIGHT : MAX_BYTES_SLICE_HEIGHT;
	min_width = (inst->codec == MSM_VIDC_HEVC) ?
		MIN_HEVC_SLICE_WIDTH : MIN_AVC_SLICE_WIDTH;
	min_height = MIN_SLICE_HEIGHT;

	/*
	 * For V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB:
	 * - width >= 384 and height >= 128
	 * - width and height <= 4096
	 * For V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_BYTES:
	 * - width >= 192 and height >= 128
	 * - width and height <= 1920
	 */
	if (output_width < min_width || output_height < min_height ||
	    output_width > max_width || output_height > max_width) {
		adjusted_value = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE;
		update_cap = SLICE_MODE;
		i_vpr_h(inst,
			"%s: slice unsupported, codec: %#x wxh: [%dx%d]\n",
			__func__, inst->codec, output_width, output_height);
		goto exit;
	}

	mbpf = NUM_MBS_PER_FRAME(output_height, output_width);
	mbps = NUM_MBS_PER_SEC(output_height, output_width, fps);
	max_mbpf = NUM_MBS_PER_FRAME(max_height, max_width);
	max_mbps = NUM_MBS_PER_SEC(max_height, max_width, MAX_SLICES_FRAME_RATE);

	if (mbpf > max_mbpf || mbps > max_mbps) {
		adjusted_value = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE;
		update_cap = SLICE_MODE;
		i_vpr_h(inst,
			"%s: Unsupported, mbpf[%u] > max[%u], mbps[%u] > max[%u]\n",
			__func__, mbpf, max_mbpf, mbps, max_mbps);
		goto exit;
	}

	if (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB) {
		update_cap = SLICE_MAX_MB;
		slice_val = inst->capabilities[SLICE_MAX_MB].value;
		slice_val = max(slice_val, mbpf / MAX_SLICES_PER_FRAME);
	} else {
		slice_val = inst->capabilities[SLICE_MAX_BYTES].value;
		update_cap = SLICE_MAX_BYTES;
		if (rc_type != HFI_RC_OFF) {
			max_avg_slicesize = ((bitrate / fps) / 8) /
				MAX_SLICES_PER_FRAME;
			slice_val = max(slice_val, max_avg_slicesize);
		}
	}
	adjusted_value = slice_val;

exit:
	msm_vidc_update_cap_value(inst, update_cap, adjusted_value, __func__);

	return 0;
}

static int msm_vidc_adjust_static_layer_count_and_type(struct msm_vidc_inst *inst, s32 layer_count)
{
	bool hb_requested = false;

	if (!layer_count) {
		i_vpr_h(inst, "client not enabled layer encoding\n");
		goto exit;
	}

	if (inst->hfi_rc_type == HFI_RC_CQ) {
		i_vpr_h(inst, "rc type is CQ, disabling layer encoding\n");
		layer_count = 0;
		goto exit;
	}

	if (inst->codec == MSM_VIDC_H264) {
		if (!inst->capabilities[LAYER_ENABLE].value) {
			layer_count = 0;
			goto exit;
		}

		hb_requested = (inst->capabilities[LAYER_TYPE].value ==
				V4L2_MPEG_VIDEO_H264_HIERARCHICAL_CODING_B) ?
				true : false;
	} else if (inst->codec == MSM_VIDC_HEVC) {
		hb_requested = (inst->capabilities[LAYER_TYPE].value ==
				V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_B) ?
				true : false;
	}

	if (hb_requested && inst->hfi_rc_type != HFI_RC_VBR_CFR) {
		i_vpr_h(inst,
			"%s: HB layer encoding is supported for VBR rc only\n",
			__func__);
		layer_count = 0;
		goto exit;
	}

	/* decide hfi layer type */
	if (hb_requested) {
		inst->hfi_layer_type = HFI_HIER_B;
	} else {
		/* HP requested */
		inst->hfi_layer_type = HFI_HIER_P_SLIDING_WINDOW;
		if (inst->codec == MSM_VIDC_H264 &&
		    inst->hfi_rc_type == HFI_RC_VBR_CFR)
			inst->hfi_layer_type = HFI_HIER_P_HYBRID_LTR;
	}

	/* sanitize layer count based on layer type and codec, and rc type */
	if (inst->hfi_layer_type == HFI_HIER_B) {
		if (layer_count > MAX_ENH_LAYER_HB)
			layer_count = MAX_ENH_LAYER_HB;
	} else if (inst->hfi_layer_type == HFI_HIER_P_HYBRID_LTR) {
		if (layer_count > MAX_AVC_ENH_LAYER_HYBRID_HP)
			layer_count = MAX_AVC_ENH_LAYER_HYBRID_HP;
	} else if (inst->hfi_layer_type == HFI_HIER_P_SLIDING_WINDOW) {
		if (inst->codec == MSM_VIDC_H264) {
			if (layer_count > MAX_AVC_ENH_LAYER_SLIDING_WINDOW)
				layer_count = MAX_AVC_ENH_LAYER_SLIDING_WINDOW;
		} else if (inst->codec == MSM_VIDC_HEVC) {
			if (inst->hfi_rc_type == HFI_RC_VBR_CFR) {
				if (layer_count > MAX_HEVC_VBR_ENH_LAYER_SLIDING_WINDOW)
					layer_count = MAX_HEVC_VBR_ENH_LAYER_SLIDING_WINDOW;
			} else {
				if (layer_count > MAX_HEVC_NON_VBR_ENH_LAYER_SLIDING_WINDOW)
					layer_count = MAX_HEVC_NON_VBR_ENH_LAYER_SLIDING_WINDOW;
			}
		}
	}

exit:
	msm_vidc_update_cap_value(inst, ENH_LAYER_COUNT, layer_count, __func__);
	inst->capabilities[ENH_LAYER_COUNT].max = layer_count;
	return 0;
}

int msm_vidc_adjust_layer_count(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	s32 client_layer_count;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	client_layer_count = ctrl ? ctrl->val :
		inst->capabilities[ENH_LAYER_COUNT].value;

	if (!is_parent_available(inst, ENH_LAYER_COUNT,
				 BITRATE_MODE, __func__))
		return -EINVAL;

	if (!inst->bufq[OUTPUT_PORT].vb2q->streaming) {
		rc = msm_vidc_adjust_static_layer_count_and_type(inst, client_layer_count);
		if (rc)
			goto exit;
	} else {
		if (inst->hfi_rc_type == HFI_RC_CBR_CFR ||
		    inst->hfi_rc_type == HFI_RC_CBR_VFR) {
			i_vpr_h(inst,
				"%s: ignoring dynamic layer count change for CBR mode\n",
				__func__);
			goto exit;
		}

		if (inst->hfi_layer_type == HFI_HIER_P_HYBRID_LTR ||
		    inst->hfi_layer_type == HFI_HIER_P_SLIDING_WINDOW) {
			/* dynamic layer count change is only supported for HP */
			if (client_layer_count >
				inst->capabilities[ENH_LAYER_COUNT].max)
				client_layer_count =
					inst->capabilities[ENH_LAYER_COUNT].max;

			msm_vidc_update_cap_value(inst, ENH_LAYER_COUNT,
						  client_layer_count, __func__);
		}
	}

exit:
	return rc;
}

int msm_vidc_adjust_gop_size(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 adjusted_value, enh_layer_count = -1;
	u32 min_gop_size, num_subgops;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[GOP_SIZE].value;

	if (msm_vidc_get_parent_value(inst, GOP_SIZE,
				      ENH_LAYER_COUNT, &enh_layer_count, __func__))
		return -EINVAL;

	if (!enh_layer_count)
		goto exit;

	/*
	 * Layer encoding needs GOP size to be multiple of subgop size
	 * And subgop size is 2 ^ number of enhancement layers.
	 */

	/* v4l2 layer count is the number of enhancement layers */
	min_gop_size = 1 << enh_layer_count;
	num_subgops = (adjusted_value + (min_gop_size >> 1)) /
			min_gop_size;
	if (num_subgops)
		adjusted_value = num_subgops * min_gop_size;
	else
		adjusted_value = min_gop_size;

exit:
	msm_vidc_update_cap_value(inst, GOP_SIZE, adjusted_value, __func__);
	return 0;
}

int msm_vidc_adjust_b_frame(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 adjusted_value, enh_layer_count = -1;
	const u32 max_bframe_size = 7;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[B_FRAME].value;

	if (msm_vidc_get_parent_value(inst, B_FRAME,
				      ENH_LAYER_COUNT, &enh_layer_count, __func__))
		return -EINVAL;

	if (!enh_layer_count || inst->hfi_layer_type != HFI_HIER_B) {
		adjusted_value = 0;
		goto exit;
	}

	adjusted_value = (1 << enh_layer_count) - 1;
	/* Allowed Bframe values are 0, 1, 3, 7 */
	if (adjusted_value > max_bframe_size)
		adjusted_value = max_bframe_size;

exit:
	msm_vidc_update_cap_value(inst, B_FRAME, adjusted_value, __func__);
	return 0;
}

int msm_vidc_adjust_bitrate(void *instance, struct v4l2_ctrl *ctrl)
{
	int i, rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	s32 adjusted_value, enh_layer_count;
	u32 cumulative_bitrate = 0, cap_id = 0, cap_value = 0;
	u32 layer_br_caps[6] = {L0_BR, L1_BR, L2_BR, L3_BR, L4_BR, L5_BR};
	u32 max_bitrate = 0;

	/* ignore layer bitrate when total bitrate is set */
	if (inst->capabilities[BIT_RATE].flags & CAP_FLAG_CLIENT_SET) {
		/*
		 * For static case, ctrl is null.
		 * For dynamic case, only BIT_RATE cap uses this adjust function.
		 * Hence, no need to check for ctrl id to be BIT_RATE control, and not
		 * any of layer bitrate controls.
		 */
		adjusted_value = ctrl ? ctrl->val : inst->capabilities[BIT_RATE].value;
		msm_vidc_update_cap_value(inst, BIT_RATE, adjusted_value, __func__);

		return 0;
	}

	if (inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

	if (msm_vidc_get_parent_value(inst, BIT_RATE,
				      ENH_LAYER_COUNT, &enh_layer_count, __func__))
		return -EINVAL;

	/* get max bit rate for current session config*/
	max_bitrate = msm_vidc_get_max_bitrate(inst);
	if (inst->capabilities[BIT_RATE].value > max_bitrate)
		msm_vidc_update_cap_value(inst, BIT_RATE, max_bitrate, __func__);

	/*
	 * ENH_LAYER_COUNT cap max is positive only if
	 * layer encoding is enabled during streamon.
	 */
	if (inst->capabilities[ENH_LAYER_COUNT].max) {
		if (!msm_vidc_check_all_layer_bitrate_set(inst)) {
			i_vpr_h(inst,
				"%s: client did not set all layer bitrates\n",
				__func__);
			return 0;
		}

		cumulative_bitrate = msm_vidc_get_cumulative_bitrate(inst);

		/* cap layer bitrates to max supported bitrate */
		if (cumulative_bitrate > max_bitrate) {
			u32 decrement_in_value = 0;
			u32 decrement_in_percent = ((cumulative_bitrate - max_bitrate) * 100) /
				max_bitrate;

			cumulative_bitrate = 0;
			for (i = 0; i <= enh_layer_count; i++) {
				if (i >= ARRAY_SIZE(layer_br_caps))
					break;
				cap_id = layer_br_caps[i];
				cap_value = inst->capabilities[cap_id].value;

				decrement_in_value = (cap_value *
					decrement_in_percent) / 100;
				cumulative_bitrate += (cap_value - decrement_in_value);

				/*
				 * cap value for the L*_BR is changed. Hence, update cap,
				 * and add to FW_LIST to set new values to firmware.
				 */
				msm_vidc_update_cap_value(inst, cap_id,
							  (cap_value - decrement_in_value),
							  __func__);
			}
		}

		i_vpr_h(inst,
			"%s: update BIT_RATE with cumulative bitrate\n",
			__func__);
		msm_vidc_update_cap_value(inst, BIT_RATE, cumulative_bitrate, __func__);
	}

	return rc;
}

int msm_vidc_adjust_layer_bitrate(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	u32 cumulative_bitrate = 0;
	u32 client_set_cap_id = INST_CAP_NONE;
	u32 old_br = 0, new_br = 0, exceeded_br = 0;
	s32 max_bitrate;

	if (!ctrl)
		return 0;

	/* ignore layer bitrate when total bitrate is set */
	if (inst->capabilities[BIT_RATE].flags & CAP_FLAG_CLIENT_SET)
		return 0;

	/*
	 * This is no-op function because layer bitrates were already adjusted
	 * in msm_vidc_adjust_bitrate function
	 */
	if (!inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

	/*
	 * ENH_LAYER_COUNT cap max is positive only if
	 * layer encoding is enabled during streamon.
	 */
	if (!inst->capabilities[ENH_LAYER_COUNT].max) {
		i_vpr_e(inst, "%s: layers not enabled\n", __func__);
		return -EINVAL;
	}

	if (!msm_vidc_check_all_layer_bitrate_set(inst)) {
		i_vpr_h(inst,
			"%s: client did not set all layer bitrates\n",
			__func__);
		return 0;
	}

	client_set_cap_id = msm_vidc_get_cap_id(inst, ctrl->id);
	if (client_set_cap_id == INST_CAP_NONE) {
		i_vpr_e(inst, "%s: could not find cap_id for ctrl %s\n",
			__func__, ctrl->name);
		return -EINVAL;
	}

	cumulative_bitrate = msm_vidc_get_cumulative_bitrate(inst);
	max_bitrate = inst->capabilities[BIT_RATE].max;
	old_br = inst->capabilities[client_set_cap_id].value;
	new_br = ctrl->val;

	/*
	 * new bitrate is not supposed to cause cumulative bitrate to
	 * exceed max supported bitrate
	 */

	if ((cumulative_bitrate - old_br + new_br) > max_bitrate) {
		/* adjust new bitrate */
		exceeded_br = (cumulative_bitrate - old_br + new_br) - max_bitrate;
		new_br = ctrl->val - exceeded_br;
	}
	msm_vidc_update_cap_value(inst, client_set_cap_id, new_br, __func__);

	/* adjust totol bitrate cap */
	i_vpr_h(inst,
		"%s: update BIT_RATE with cumulative bitrate\n",
		__func__);
	msm_vidc_update_cap_value(inst, BIT_RATE,
				  msm_vidc_get_cumulative_bitrate(inst), __func__);

	return rc;
}

int msm_vidc_adjust_peak_bitrate(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1, bitrate = -1;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[PEAK_BITRATE].value;

	if (msm_vidc_get_parent_value(inst, PEAK_BITRATE,
				      BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	if (rc_type != HFI_RC_CBR_CFR &&
	    rc_type != HFI_RC_CBR_VFR)
		return 0;

	if (msm_vidc_get_parent_value(inst, PEAK_BITRATE,
				      BIT_RATE, &bitrate, __func__))
		return -EINVAL;

	/* Peak Bitrate should be larger than or equal to avg bitrate */
	if (inst->capabilities[PEAK_BITRATE].flags & CAP_FLAG_CLIENT_SET) {
		if (adjusted_value < bitrate)
			adjusted_value = bitrate;
	} else {
		adjusted_value = inst->capabilities[BIT_RATE].value;
	}

	msm_vidc_update_cap_value(inst, PEAK_BITRATE,
				  adjusted_value, __func__);

	return 0;
}

static int msm_vidc_adjust_hevc_qp(struct msm_vidc_inst *inst,
				   enum msm_vidc_inst_capability_type cap_id)
{
	s32 pix_fmt = -1;

	if (!(inst->codec == MSM_VIDC_HEVC || inst->codec == MSM_VIDC_HEIC)) {
		i_vpr_e(inst,
			"%s: incorrect cap[%d] %s entry in database, fix database\n",
			__func__, cap_id, cap_name(cap_id));
		return -EINVAL;
	}

	if (msm_vidc_get_parent_value(inst, cap_id,
				      PIX_FMTS, &pix_fmt, __func__))
		return -EINVAL;

	if (pix_fmt == MSM_VIDC_FMT_P010 || pix_fmt == MSM_VIDC_FMT_TP10C)
		goto exit;

	CAP_TO_8BIT_QP(inst->capabilities[cap_id].value);
	if (cap_id == MIN_FRAME_QP) {
		CAP_TO_8BIT_QP(inst->capabilities[I_FRAME_MIN_QP].value);
		CAP_TO_8BIT_QP(inst->capabilities[P_FRAME_MIN_QP].value);
		CAP_TO_8BIT_QP(inst->capabilities[B_FRAME_MIN_QP].value);
	} else if (cap_id == MAX_FRAME_QP) {
		CAP_TO_8BIT_QP(inst->capabilities[I_FRAME_MAX_QP].value);
		CAP_TO_8BIT_QP(inst->capabilities[P_FRAME_MAX_QP].value);
		CAP_TO_8BIT_QP(inst->capabilities[B_FRAME_MAX_QP].value);
	}

exit:
	return 0;
}

int msm_vidc_adjust_hevc_min_qp(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	if (ctrl)
		msm_vidc_update_cap_value(inst, MIN_FRAME_QP,
					  ctrl->val, __func__);

	rc = msm_vidc_adjust_hevc_qp(inst, MIN_FRAME_QP);

	return rc;
}

int msm_vidc_adjust_hevc_max_qp(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	if (ctrl)
		msm_vidc_update_cap_value(inst, MAX_FRAME_QP, ctrl->val, __func__);

	rc = msm_vidc_adjust_hevc_qp(inst, MAX_FRAME_QP);

	return rc;
}

int msm_vidc_adjust_hevc_i_frame_qp(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	if (ctrl)
		msm_vidc_update_cap_value(inst, I_FRAME_QP, ctrl->val, __func__);

	rc = msm_vidc_adjust_hevc_qp(inst, I_FRAME_QP);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_adjust_hevc_p_frame_qp(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	if (ctrl)
		msm_vidc_update_cap_value(inst, P_FRAME_QP, ctrl->val, __func__);

	rc = msm_vidc_adjust_hevc_qp(inst, P_FRAME_QP);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_adjust_hevc_b_frame_qp(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	if (ctrl)
		msm_vidc_update_cap_value(inst, B_FRAME_QP, ctrl->val, __func__);

	rc = msm_vidc_adjust_hevc_qp(inst, B_FRAME_QP);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_adjust_blur_type(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1, roi_enable = -1;
	s32 pix_fmts = -1, min_quality = -1;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[BLUR_TYPES].value;

	if (adjusted_value == MSM_VIDC_BLUR_NONE)
		return 0;

	if (msm_vidc_get_parent_value(inst, BLUR_TYPES, BITRATE_MODE,
				      &rc_type, __func__) ||
	    msm_vidc_get_parent_value(inst, BLUR_TYPES, MIN_QUALITY,
				      &min_quality, __func__))
		return -EINVAL;

	if (adjusted_value == MSM_VIDC_BLUR_EXTERNAL) {
		if (is_scaling_enabled(inst) || min_quality)
			adjusted_value = MSM_VIDC_BLUR_NONE;
	} else if (adjusted_value == MSM_VIDC_BLUR_ADAPTIVE) {
		if (is_valid_cap(inst, META_ROI_INFO)) {
			if (msm_vidc_get_parent_value(inst, BLUR_TYPES,
						      META_ROI_INFO, &roi_enable, __func__))
				return -EINVAL;
			if (is_meta_tx_inp_enabled(inst, META_ROI_INFO)) {
				adjusted_value = MSM_VIDC_BLUR_NONE;
				goto exit;
			}
		}

		if (is_scaling_enabled(inst) || min_quality ||
		    (rc_type != HFI_RC_VBR_CFR &&
		    rc_type != HFI_RC_CBR_CFR &&
		    rc_type != HFI_RC_CBR_VFR)) {
			adjusted_value = MSM_VIDC_BLUR_NONE;
			goto exit;
		}

		if (inst->codec == MSM_VIDC_HEVC) {
			if (msm_vidc_get_parent_value(inst, BLUR_TYPES, PIX_FMTS,
						      &pix_fmts, __func__))
				return -EINVAL;
			if (is_10bit_colorformat(pix_fmts))
				adjusted_value = MSM_VIDC_BLUR_NONE;
		}
	}

exit:
	msm_vidc_update_cap_value(inst, BLUR_TYPES, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_all_intra(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_core *core;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 gop_size = -1, bframe = -1;
	u32 width, height, fps, mbps, max_mbps;

	adjusted_value = inst->capabilities[ALL_INTRA].value;

	if (msm_vidc_get_parent_value(inst, ALL_INTRA, GOP_SIZE,
				      &gop_size, __func__) ||
	    msm_vidc_get_parent_value(inst, ALL_INTRA, B_FRAME,
				      &bframe, __func__))
		return -EINVAL;

	width = inst->crop.width;
	height = inst->crop.height;
	fps =  msm_vidc_get_fps(inst);
	mbps = NUM_MBS_PER_SEC(height, width, fps);
	core = inst->core;
	max_mbps = core->capabilities[MAX_MBPS_ALL_INTRA].value;

	if (mbps > max_mbps) {
		adjusted_value = 0;
		i_vpr_h(inst, "%s: mbps %d exceeds max supported mbps %d\n",
			__func__, mbps, max_mbps);
		goto exit;
	}

	if (!gop_size && !bframe)
		adjusted_value = 1;

exit:
	msm_vidc_update_cap_value(inst, ALL_INTRA, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_blur_resolution(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 blur_type = -1;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[BLUR_RESOLUTION].value;

	if (msm_vidc_get_parent_value(inst, BLUR_RESOLUTION, BLUR_TYPES,
				      &blur_type, __func__))
		return -EINVAL;

	if (blur_type != MSM_VIDC_BLUR_EXTERNAL)
		return 0;

	msm_vidc_update_cap_value(inst, BLUR_RESOLUTION,
				  adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_brs(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1, layer_enabled = -1, layer_type = -1;
	bool hp_requested = false;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[CONTENT_ADAPTIVE_CODING].value;

	if (inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

	if (msm_vidc_get_parent_value(inst, CONTENT_ADAPTIVE_CODING,
				      BITRATE_MODE, &rc_type, __func__) ||
	    msm_vidc_get_parent_value(inst, CONTENT_ADAPTIVE_CODING,
				      LAYER_ENABLE, &layer_enabled, __func__) ||
	    msm_vidc_get_parent_value(inst, CONTENT_ADAPTIVE_CODING,
				      LAYER_TYPE, &layer_type, __func__))
		return -EINVAL;

	/*
	 * -BRS is supported only for VBR rc type.
	 *  Hence, do not adjust or set to firmware for non VBR rc's
	 * -If HP is enabled then BRS is not allowed.
	 */
	if (rc_type != HFI_RC_VBR_CFR) {
		adjusted_value = 0;
		goto adjust;
	}

	if (inst->codec == MSM_VIDC_H264)
		layer_type = V4L2_MPEG_VIDEO_H264_HIERARCHICAL_CODING_P;
	else if (inst->codec == MSM_VIDC_HEVC)
		layer_type = V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_P;

	hp_requested = (inst->capabilities[LAYER_TYPE].value == layer_type);

	/*
	 * Disable BRS in case of HP encoding
	 * Hence set adjust value to 0.
	 */
	if (layer_enabled == 1 && hp_requested) {
		adjusted_value = 0;
		goto adjust;
	}

adjust:
	msm_vidc_update_cap_value(inst, CONTENT_ADAPTIVE_CODING,
				  adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_bitrate_boost(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 min_quality = -1, rc_type = -1;
	u32 max_bitrate = 0, bitrate = 0;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[BITRATE_BOOST].value;

	if (inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

	if (msm_vidc_get_parent_value(inst, BITRATE_BOOST,
				      MIN_QUALITY, &min_quality, __func__) ||
	    msm_vidc_get_parent_value(inst, BITRATE_BOOST,
				      BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	/*
	 * Bitrate Boost are supported only for VBR rc type.
	 * Hence, do not adjust or set to firmware for non VBR rc's
	 */
	if (rc_type != HFI_RC_VBR_CFR) {
		adjusted_value = 0;
		goto adjust;
	}

	if (min_quality) {
		adjusted_value = MAX_BITRATE_BOOST;
		goto adjust;
	}

	max_bitrate = msm_vidc_get_max_bitrate(inst);
	bitrate = inst->capabilities[BIT_RATE].value;
	if (adjusted_value) {
		if ((bitrate + bitrate / (100 / adjusted_value)) > max_bitrate) {
			i_vpr_h(inst,
				"%s: bitrate %d is beyond max bitrate %d, remove bitrate boost\n",
				__func__, max_bitrate, bitrate);
			adjusted_value = 0;
		}
	}

adjust:
	msm_vidc_update_cap_value(inst, BITRATE_BOOST, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_min_quality(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 roi_enable = -1, rc_type = -1, enh_layer_count = -1, pix_fmts = -1;
	u32 width, height, frame_rate;
	struct v4l2_format *f;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[MIN_QUALITY].value;

	/*
	 * Although MIN_QUALITY is static, one of its parents,
	 * ENH_LAYER_COUNT is dynamic cap. Hence, dynamic call
	 * may be made for MIN_QUALITY via ENH_LAYER_COUNT.
	 * Therefore, below streaming check is required to avoid
	 * runtime modification of MIN_QUALITY.
	 */
	if (inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

	if (msm_vidc_get_parent_value(inst, MIN_QUALITY,
				      BITRATE_MODE, &rc_type, __func__) ||
	    msm_vidc_get_parent_value(inst, MIN_QUALITY,
				      ENH_LAYER_COUNT, &enh_layer_count, __func__))
		return -EINVAL;

	/*
	 * Min Quality is supported only for VBR rc type.
	 * Hence, do not adjust or set to firmware for non VBR rc's
	 */
	if (rc_type != HFI_RC_VBR_CFR) {
		adjusted_value = 0;
		goto update_and_exit;
	}

	frame_rate = inst->capabilities[FRAME_RATE].value >> 16;
	f = &inst->fmts[OUTPUT_PORT];
	width = f->fmt.pix_mp.width;
	height = f->fmt.pix_mp.height;

	/*
	 * VBR Min Quality not supported for:
	 * - HEVC 10bit
	 * - ROI support
	 * - HP encoding
	 * - External Blur
	 * - Resolution beyond 1080P
	 * (It will fall back to CQCAC 25% or 0% (CAC) or CQCAC-OFF)
	 */
	if (inst->codec == MSM_VIDC_HEVC) {
		if (msm_vidc_get_parent_value(inst, MIN_QUALITY,
					      PIX_FMTS, &pix_fmts, __func__))
			return -EINVAL;

		if (is_10bit_colorformat(pix_fmts)) {
			i_vpr_h(inst,
				"%s: min quality is supported only for 8 bit\n",
				__func__);
			adjusted_value = 0;
			goto update_and_exit;
		}
	}

	if (res_is_greater_than(width, height, 1920, 1080)) {
		i_vpr_h(inst, "%s: unsupported res, wxh %ux%u\n",
			__func__, width, height);
		adjusted_value = 0;
		goto update_and_exit;
	}

	if (frame_rate > 60) {
		i_vpr_h(inst, "%s: unsupported fps %u\n",
			__func__, frame_rate);
		adjusted_value = 0;
		goto update_and_exit;
	}

	if (is_valid_cap(inst, META_ROI_INFO)) {
		if (msm_vidc_get_parent_value(inst, MIN_QUALITY,
					      META_ROI_INFO, &roi_enable, __func__))
			return -EINVAL;
		if (is_meta_tx_inp_enabled(inst, META_ROI_INFO)) {
			i_vpr_h(inst,
				"%s: min quality not supported with roi metadata\n",
				__func__);
			adjusted_value = 0;
			goto update_and_exit;
		}
	}

	if (enh_layer_count > 0 && inst->hfi_layer_type != HFI_HIER_B) {
		i_vpr_h(inst,
			"%s: min quality not supported for HP encoding\n",
			__func__);
		adjusted_value = 0;
		goto update_and_exit;
	}

	/* Above conditions are met. Hence enable min quality */
	adjusted_value = MAX_SUPPORTED_MIN_QUALITY;

update_and_exit:
	msm_vidc_update_cap_value(inst, MIN_QUALITY, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_preprocess(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 brs = 0;
	u32 width, height, frame_rate, operating_rate, max_fps;
	struct v4l2_format *f;

	adjusted_value = inst->capabilities[REQUEST_PREPROCESS].value;

	width = inst->crop.width;
	height = inst->crop.height;
	frame_rate = msm_vidc_get_frame_rate(inst);
	operating_rate = msm_vidc_get_operating_rate(inst);

	max_fps = max(frame_rate, operating_rate);
	f = &inst->fmts[OUTPUT_PORT];

	/*
	 * enable preprocess if
	 * BRS enabled and upto 4k @ 60 fps
	 */
	if (is_valid_cap(inst, CONTENT_ADAPTIVE_CODING)) {
		if (msm_vidc_get_parent_value(inst,
					      REQUEST_PREPROCESS,
					      CONTENT_ADAPTIVE_CODING,
					      &brs, __func__))
			return -EINVAL;
		if (brs == 0) {
			/* preprocess not required as BRS not enabled */
			adjusted_value = 0;
			goto update_preprocess;
		}
	} else {
		/* preprocess not required as BRS not available */
		adjusted_value = 0;
		goto update_preprocess;
	}

	if (res_is_less_than_or_equal_to(width, height, 3840, 2160) &&
	    max_fps <= 60)
		adjusted_value = 1;
	else
		adjusted_value = 0;

update_preprocess:
	msm_vidc_update_cap_value(inst, REQUEST_PREPROCESS, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_enc_lowlatency_mode(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[LOWLATENCY_MODE].value;

	if (msm_vidc_get_parent_value(inst, LOWLATENCY_MODE, BITRATE_MODE,
				      &rc_type, __func__))
		return -EINVAL;

	if (rc_type == HFI_RC_CBR_CFR ||
	    rc_type == HFI_RC_CBR_VFR ||
	    is_enc_slice_delivery_mode(inst))
		adjusted_value = 1;

	msm_vidc_update_cap_value(inst, LOWLATENCY_MODE, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_dec_lowlatency_mode(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 outbuf_fence = MSM_VIDC_META_DISABLE;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[LOWLATENCY_MODE].value;

	if (is_valid_cap(inst, META_OUTBUF_FENCE)) {
		if (msm_vidc_get_parent_value(inst, LOWLATENCY_MODE, META_OUTBUF_FENCE,
					      &outbuf_fence, __func__))
			return -EINVAL;
		/* enable lowlatency if outbuf fence is enabled */
		if (outbuf_fence & MSM_VIDC_META_ENABLE &&
		    outbuf_fence & MSM_VIDC_META_RX_INPUT)
			adjusted_value = 1;
	}

	msm_vidc_update_cap_value(inst, LOWLATENCY_MODE, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_session_priority(void *instance, struct v4l2_ctrl *ctrl)
{
	int adjusted_value;

	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	/*
	 * Priority handling
	 * Client will set 0 (realtime), (1 to 4) (non-realtime)
	 * When driver setting priority to firmware, driver adds
	 * FIRMWARE_PRIORITY_OFFSET (1) for all sessions except
	 * non-critical sessions. So finally firmware priority values ranges
	 * from 0 (Critical session), 1 (realtime session),
	 * (2 to 5) (non-realtime session).
	 * Driver may move decode realtime sessions to non-realtime by
	 * increasing priority by 1 to RT sessions in HW overloaded cases.
	 * Following will be priority based on requirement -
	 *  ___________________________________________________________
	 *  |  Description    |    HAL Value     |Driver value|FW Value|
	 *  |_________________|__________________|____________|________|
	 *  |Critical Priority| Vendor Extension | Via control|   0    |
	 *  |      RT         |        0         |      0     |   1    |
	 *  |      NRT        |     -1 to -4     |     1-4    |  2-5   |
	 *  |_________________|__________________|____________|________|
	 */

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[PRIORITY].value;

	msm_vidc_update_cap_value(inst, PRIORITY, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_roi_info(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1, pix_fmt = -1;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[META_ROI_INFO].value;

	if (msm_vidc_get_parent_value(inst, META_ROI_INFO, BITRATE_MODE,
				      &rc_type, __func__))
		return -EINVAL;

	if (msm_vidc_get_parent_value(inst, META_ROI_INFO, PIX_FMTS,
				      &pix_fmt, __func__))
		return -EINVAL;

	if ((rc_type != HFI_RC_VBR_CFR && rc_type != HFI_RC_CBR_CFR &&
	     rc_type != HFI_RC_CBR_VFR) || !is_8bit_colorformat(pix_fmt) ||
	     is_scaling_enabled(inst) || is_rotation_90_or_270(inst))
		adjusted_value = 0;

	msm_vidc_update_cap_value(inst, META_ROI_INFO, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_dec_outbuf_fence_type(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst_cap *capability;
	s32 adjusted_value, meta_outbuf_fence = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	struct msm_vidc_core *core;

	capability = inst->capabilities;
	core = inst->core;

	adjusted_value = ctrl ? ctrl->val :
		capability[OUTBUF_FENCE_TYPE].value;

	if (msm_vidc_get_parent_value(inst, OUTBUF_FENCE_TYPE,
				      META_OUTBUF_FENCE, &meta_outbuf_fence, __func__))
		return -EINVAL;

	if (is_meta_rx_inp_enabled(inst, META_OUTBUF_FENCE)) {
		if (core->capabilities[SUPPORTS_SYNX_FENCE].value)
			adjusted_value = MSM_VIDC_SYNX_V2_FENCE;
		else
			adjusted_value = MSM_VIDC_SW_FENCE;
	} else {
		adjusted_value = MSM_VIDC_FENCE_NONE;
	}

	msm_vidc_update_cap_value(inst, OUTBUF_FENCE_TYPE, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_dec_outbuf_fence_direction(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst_cap *capability;
	s32 adjusted_value, meta_outbuf_fence = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	struct msm_vidc_core *core;

	capability = inst->capabilities;
	core = inst->core;

	adjusted_value = ctrl ? ctrl->val :
		capability[OUTBUF_FENCE_DIRECTION].value;

	if (msm_vidc_get_parent_value(inst, OUTBUF_FENCE_DIRECTION,
				      META_OUTBUF_FENCE, &meta_outbuf_fence, __func__))
		return -EINVAL;

	if (is_meta_rx_inp_enabled(inst, META_OUTBUF_FENCE))
		adjusted_value = MSM_VIDC_FENCE_DIR_TX;
	else
		adjusted_value = MSM_VIDC_FENCE_DIR_NONE;

	msm_vidc_update_cap_value(inst, OUTBUF_FENCE_DIRECTION, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_dec_slice_mode(void *instance, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 adjusted_value = 0;
	s32 low_latency = -1;
	s32 picture_order = -1;
	s32 outbuf_fence = 0;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[SLICE_DECODE].value;

	if (msm_vidc_get_parent_value(inst, SLICE_DECODE, LOWLATENCY_MODE,
				      &low_latency, __func__) ||
	    msm_vidc_get_parent_value(inst, SLICE_DECODE, OUTPUT_ORDER,
				      &picture_order, __func__) ||
	    msm_vidc_get_parent_value(inst, SLICE_DECODE, META_OUTBUF_FENCE,
				      &outbuf_fence, __func__))
		return -EINVAL;

	if (!low_latency || !picture_order ||
	    !is_meta_rx_inp_enabled(inst, META_OUTBUF_FENCE))
		adjusted_value = 0;

	msm_vidc_update_cap_value(inst, SLICE_DECODE, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_eva_stats(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[META_EVA_STATS].value;

	if (msm_vidc_get_parent_value(inst, META_EVA_STATS, BITRATE_MODE,
				      &rc_type, __func__))
		return -EINVAL;

	/* disable Eva stats metadata for CQ rate control */
	if (rc_type == HFI_RC_CQ) {
		i_vpr_h(inst, "%s: unsupported for CQ rate control\n", __func__);
		adjusted_value = 0;
	}

	msm_vidc_update_cap_value(inst, META_EVA_STATS, adjusted_value, __func__);

	return 0;
}

int msm_vidc_adjust_sei_mastering_disp(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 profile = -1;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[META_SEI_MASTERING_DISP].value;

	if (msm_vidc_get_parent_value(inst, META_SEI_MASTERING_DISP, PROFILE,
				      &profile, __func__))
		return -EINVAL;

	if (inst->codec != MSM_VIDC_HEVC && inst->codec != MSM_VIDC_HEIC) {
		adjusted_value = 0;
		goto adjust;
	}

	if ((inst->codec == MSM_VIDC_HEVC &&
	     profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10) ||
	     (inst->codec == MSM_VIDC_HEIC &&
	     profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10_STILL_PICTURE)) {
		adjusted_value = 0;
		goto adjust;
	}

adjust:
	msm_vidc_update_cap_value(inst, META_SEI_MASTERING_DISP,
				  adjusted_value, __func__);
	return 0;
}

int msm_vidc_adjust_sei_cll(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 profile = -1;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[META_SEI_CLL].value;

	if (msm_vidc_get_parent_value(inst, META_SEI_CLL, PROFILE,
				      &profile, __func__))
		return -EINVAL;

	if (inst->codec != MSM_VIDC_HEVC && inst->codec != MSM_VIDC_HEIC) {
		adjusted_value = 0;
		goto adjust;
	}

	if ((inst->codec == MSM_VIDC_HEVC &&
	     profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10) ||
	    (inst->codec == MSM_VIDC_HEIC &&
	     profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10_STILL_PICTURE)) {
		adjusted_value = 0;
		goto adjust;
	}

adjust:
	msm_vidc_update_cap_value(inst, META_SEI_CLL, adjusted_value, __func__);
	return 0;
}

int msm_vidc_adjust_hdr10plus(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 profile = -1;

	adjusted_value = ctrl ? ctrl->val : inst->capabilities[META_HDR10PLUS].value;

	if (msm_vidc_get_parent_value(inst, META_HDR10PLUS, PROFILE,
				      &profile, __func__))
		return -EINVAL;

	if (inst->codec != MSM_VIDC_HEVC && inst->codec != MSM_VIDC_HEIC) {
		adjusted_value = 0;
		goto adjust;
	}

	if ((inst->codec == MSM_VIDC_HEVC &&
	     profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10) ||
	    (inst->codec == MSM_VIDC_HEIC &&
	     profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10_STILL_PICTURE)) {
		adjusted_value = 0;
		goto adjust;
	}

adjust:
	msm_vidc_update_cap_value(inst, META_HDR10PLUS, adjusted_value, __func__);
	return 0;
}

int msm_vidc_adjust_transcoding_stats(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1;
	u32 width, height, fps;
	struct v4l2_format *f;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[META_TRANSCODING_STAT_INFO].value;

	if (msm_vidc_get_parent_value(inst, META_TRANSCODING_STAT_INFO,
				      BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	/*
	 * transcoding stats metadata is supported for:
	 * - VBR bitrate mode
	 * - fps <= 60
	 * - Resolution <= 4K
	 */
	if (rc_type != HFI_RC_VBR_CFR) {
		i_vpr_h(inst, "%s: unsupported rc_type: %#x\n",
			__func__, rc_type);
		adjusted_value = 0;
		goto exit;
	}

	fps = inst->capabilities[FRAME_RATE].value >> 16;
	if (fps > MAX_TRANSCODING_STATS_FRAME_RATE) {
		i_vpr_h(inst, "%s: unsupported fps %u\n", __func__, fps);
		adjusted_value = 0;
		goto exit;
	}

	f = &inst->fmts[OUTPUT_PORT];
	width = f->fmt.pix_mp.width;
	height = f->fmt.pix_mp.height;
	if (res_is_greater_than(width, height,
				MAX_TRANSCODING_STATS_WIDTH, MAX_TRANSCODING_STATS_HEIGHT)) {
		i_vpr_h(inst, "%s: unsupported res, wxh %ux%u\n",
			__func__, width, height);
		adjusted_value = 0;
		goto exit;
	}

exit:
	msm_vidc_update_cap_value(inst, META_TRANSCODING_STAT_INFO,
				  adjusted_value, __func__);

	return 0;
}

/******************* End of Control Adjust functions *************************/

/************************* Control Set functions *****************************/

int msm_vidc_set_header_mode(void *instance,
			     enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	int header_mode, prepend_sps_pps;
	u32 hfi_value = 0;

	header_mode = inst->capabilities[cap_id].value;
	prepend_sps_pps = inst->capabilities[PREPEND_SPSPPS_TO_IDR].value;

	/* prioritize PREPEND_SPSPPS_TO_IDR mode over other header modes */
	if (prepend_sps_pps)
		hfi_value = HFI_SEQ_HEADER_PREFIX_WITH_SYNC_FRAME;
	else if (header_mode == V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME)
		hfi_value = HFI_SEQ_HEADER_JOINED_WITH_1ST_FRAME;
	else
		hfi_value = HFI_SEQ_HEADER_SEPERATE_FRAME;

	if (is_meta_rx_out_enabled(inst, META_SEQ_HDR_NAL))
		hfi_value |= HFI_SEQ_HEADER_METADATA;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_deblock_mode(void *instance,
			      enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 alpha = 0, beta = 0;
	u32 lf_mode, hfi_value = 0, lf_offset = 6;

	rc = msm_vidc_v4l2_to_hfi_enum(inst, LF_MODE, &lf_mode);
	if (rc)
		return -EINVAL;

	beta = inst->capabilities[LF_BETA].value + lf_offset;
	alpha = inst->capabilities[LF_ALPHA].value + lf_offset;
	hfi_value = (alpha << 16) | (beta << 8) | lf_mode;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_32_PACKED,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_constant_quality(void *instance,
				  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;
	s32 rc_type = -1;

	if (msm_vidc_get_parent_value(inst, cap_id,
				      BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	if (rc_type != HFI_RC_CQ)
		return 0;

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_vbr_related_properties(void *instance,
					enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;
	s32 rc_type = -1;

	if (msm_vidc_get_parent_value(inst, cap_id,
				      BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	if (rc_type != HFI_RC_VBR_CFR)
		return 0;

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_cbr_related_properties(void *instance,
					enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;
	s32 rc_type = -1;

	if (msm_vidc_get_parent_value(inst, cap_id,
				      BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	if (rc_type != HFI_RC_CBR_VFR &&
	    rc_type != HFI_RC_CBR_CFR)
		return 0;

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_use_and_mark_ltr(void *instance,
				  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;

	if (!inst->capabilities[LTR_COUNT].value ||
	    inst->capabilities[cap_id].value ==
	    INVALID_DEFAULT_MARK_OR_USE_LTR) {
		i_vpr_h(inst,
			"%s: LTR_COUNT: %d %s: %d, cap %s is not set\n",
			__func__, inst->capabilities[LTR_COUNT].value,
			cap_name(cap_id),
			inst->capabilities[cap_id].value,
			cap_name(cap_id));
		return 0;
	}

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_min_qp(void *instance,
			enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	s32 i_frame_qp = 0, p_frame_qp = 0, b_frame_qp = 0, min_qp_enable = 0;
	u32 i_qp_enable = 0, p_qp_enable = 0, b_qp_enable = 0;
	u32 client_qp_enable = 0, hfi_value = 0, offset = 0;

	if (inst->capabilities[MIN_FRAME_QP].flags & CAP_FLAG_CLIENT_SET)
		min_qp_enable = 1;

	if (min_qp_enable ||
	    (inst->capabilities[I_FRAME_MIN_QP].flags & CAP_FLAG_CLIENT_SET))
		i_qp_enable = 1;
	if (min_qp_enable ||
	    (inst->capabilities[P_FRAME_MIN_QP].flags & CAP_FLAG_CLIENT_SET))
		p_qp_enable = 1;
	if (min_qp_enable ||
	    (inst->capabilities[B_FRAME_MIN_QP].flags & CAP_FLAG_CLIENT_SET))
		b_qp_enable = 1;

	client_qp_enable = i_qp_enable | p_qp_enable << 1 | b_qp_enable << 2;
	if (!client_qp_enable) {
		i_vpr_h(inst,
			"%s: client did not set min qp, cap %s is not set\n",
			__func__, cap_name(cap_id));
		return 0;
	}

	if (is_10bit_colorformat(inst->capabilities[PIX_FMTS].value))
		offset = 12;

	/*
	 * I_FRAME_MIN_QP, P_FRAME_MIN_QP, B_FRAME_MIN_QP,
	 * MIN_FRAME_QP caps have default value as MIN_QP_10BIT values.
	 * Hence, if client sets either one among MIN_FRAME_QP
	 * and (I_FRAME_MIN_QP or P_FRAME_MIN_QP or B_FRAME_MIN_QP),
	 * max of both caps will result into client set value.
	 */
	i_frame_qp = max(inst->capabilities[I_FRAME_MIN_QP].value,
			 inst->capabilities[MIN_FRAME_QP].value) + offset;
	p_frame_qp = max(inst->capabilities[P_FRAME_MIN_QP].value,
			 inst->capabilities[MIN_FRAME_QP].value) + offset;
	b_frame_qp = max(inst->capabilities[B_FRAME_MIN_QP].value,
			 inst->capabilities[MIN_FRAME_QP].value) + offset;

	hfi_value = i_frame_qp | p_frame_qp << 8 | b_frame_qp << 16 |
		client_qp_enable << 24;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_32_PACKED,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_max_qp(void *instance,
			enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	s32 i_frame_qp = 0, p_frame_qp = 0, b_frame_qp = 0, max_qp_enable = 0;
	u32 i_qp_enable = 0, p_qp_enable = 0, b_qp_enable = 0;
	u32 client_qp_enable = 0, hfi_value = 0, offset = 0;

	if (inst->capabilities[MAX_FRAME_QP].flags & CAP_FLAG_CLIENT_SET)
		max_qp_enable = 1;

	if (max_qp_enable ||
	    (inst->capabilities[I_FRAME_MAX_QP].flags & CAP_FLAG_CLIENT_SET))
		i_qp_enable = 1;
	if (max_qp_enable ||
	    (inst->capabilities[P_FRAME_MAX_QP].flags & CAP_FLAG_CLIENT_SET))
		p_qp_enable = 1;
	if (max_qp_enable ||
	    (inst->capabilities[B_FRAME_MAX_QP].flags & CAP_FLAG_CLIENT_SET))
		b_qp_enable = 1;

	client_qp_enable = i_qp_enable | p_qp_enable << 1 | b_qp_enable << 2;
	if (!client_qp_enable) {
		i_vpr_h(inst,
			"%s: client did not set max qp, cap %s is not set\n",
			__func__, cap_name(cap_id));
		return 0;
	}

	if (is_10bit_colorformat(inst->capabilities[PIX_FMTS].value))
		offset = 12;

	/*
	 * I_FRAME_MAX_QP, P_FRAME_MAX_QP, B_FRAME_MAX_QP,
	 * MAX_FRAME_QP caps have default value as MAX_QP values.
	 * Hence, if client sets either one among MAX_FRAME_QP
	 * and (I_FRAME_MAX_QP or P_FRAME_MAX_QP or B_FRAME_MAX_QP),
	 * min of both caps will result into client set value.
	 */
	i_frame_qp = min(inst->capabilities[I_FRAME_MAX_QP].value,
			 inst->capabilities[MAX_FRAME_QP].value) + offset;
	p_frame_qp = min(inst->capabilities[P_FRAME_MAX_QP].value,
			 inst->capabilities[MAX_FRAME_QP].value) + offset;
	b_frame_qp = min(inst->capabilities[B_FRAME_MAX_QP].value,
			 inst->capabilities[MAX_FRAME_QP].value) + offset;

	hfi_value = i_frame_qp | p_frame_qp << 8 | b_frame_qp << 16 |
		client_qp_enable << 24;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_32_PACKED,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_frame_qp(void *instance,
			  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	struct msm_vidc_inst_cap *capab;
	s32 i_frame_qp = 0, p_frame_qp = 0, b_frame_qp = 0;
	u32 i_qp_enable = 0, p_qp_enable = 0, b_qp_enable = 0;
	u32 client_qp_enable = 0, hfi_value = 0, offset = 0;
	s32 rc_type = -1;

	capab = inst->capabilities;

	if (msm_vidc_get_parent_value(inst, cap_id,
				      BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	if (inst->bufq[OUTPUT_PORT].vb2q->streaming) {
		if (rc_type != HFI_RC_OFF) {
			i_vpr_h(inst,
				"%s: dynamic qp not allowed for rc type %d\n",
				__func__, rc_type);
			return 0;
		}
	}

	if (rc_type == HFI_RC_OFF) {
		/* Mandatorily set for rc off case */
		i_qp_enable = 1;
		p_qp_enable = 1;
		b_qp_enable = 1;
	} else {
		/* Set only if client has set for NON rc off case */
		if (capab[I_FRAME_QP].flags & CAP_FLAG_CLIENT_SET)
			i_qp_enable = 1;
		if (capab[P_FRAME_QP].flags & CAP_FLAG_CLIENT_SET)
			p_qp_enable = 1;
		if (capab[B_FRAME_QP].flags & CAP_FLAG_CLIENT_SET)
			b_qp_enable = 1;
	}

	client_qp_enable = i_qp_enable | p_qp_enable << 1 | b_qp_enable << 2;
	if (!client_qp_enable) {
		i_vpr_h(inst,
			"%s: client did not set frame qp, cap %s is not set\n",
			__func__, cap_name(cap_id));
		return 0;
	}

	if (is_10bit_colorformat(capab[PIX_FMTS].value))
		offset = 12;

	i_frame_qp = capab[I_FRAME_QP].value + offset;
	p_frame_qp = capab[P_FRAME_QP].value + offset;
	b_frame_qp = capab[B_FRAME_QP].value + offset;

	hfi_value = i_frame_qp | p_frame_qp << 8 | b_frame_qp << 16 |
		client_qp_enable << 24;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_32_PACKED,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_req_sync_frame(void *instance,
				enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 prepend_spspps;
	u32 hfi_value = 0;

	prepend_spspps = inst->capabilities[PREPEND_SPSPPS_TO_IDR].value;
	if (prepend_spspps)
		hfi_value = HFI_SYNC_FRAME_REQUEST_WITH_PREFIX_SEQ_HDR;
	else
		hfi_value = HFI_SYNC_FRAME_REQUEST_WITHOUT_SEQ_HDR;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_chroma_qp_index_offset(void *instance,
					enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0, chroma_qp_offset_mode = 0, chroma_qp = 0;
	u32 offset = 12;

	if (inst->capabilities[cap_id].flags & CAP_FLAG_CLIENT_SET)
		chroma_qp_offset_mode = HFI_FIXED_CHROMAQP_OFFSET;
	else
		chroma_qp_offset_mode = HFI_ADAPTIVE_CHROMAQP_OFFSET;

	chroma_qp = inst->capabilities[cap_id].value + offset;
	hfi_value = chroma_qp_offset_mode | chroma_qp << 8 | chroma_qp << 16;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_32_PACKED,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_slice_count(void *instance,
			     enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 slice_mode = -1;
	u32 hfi_value = 0, set_cap_id = 0;

	slice_mode = inst->capabilities[SLICE_MODE].value;

	if (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE) {
		i_vpr_h(inst, "%s: slice mode is: %u, ignore setting to fw\n",
			__func__, slice_mode);
		return 0;
	}
	if (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB) {
		hfi_value = (inst->codec == MSM_VIDC_HEVC) ?
			((inst->capabilities[SLICE_MAX_MB].value + 3) / 4) :
			inst->capabilities[SLICE_MAX_MB].value;
		set_cap_id = SLICE_MAX_MB;
	} else if (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_BYTES) {
		hfi_value = inst->capabilities[SLICE_MAX_BYTES].value;
		set_cap_id = SLICE_MAX_BYTES;
	}

	rc = msm_vidc_packetize_control(inst, set_cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_nal_length(void *instance,
			    enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = HFI_NAL_LENGTH_STARTCODES;

	if (!inst->capabilities[WITHOUT_STARTCODE].value) {
		hfi_value = HFI_NAL_LENGTH_STARTCODES;
	} else {
		rc = msm_vidc_v4l2_to_hfi_enum(inst, NAL_LENGTH_FIELD, &hfi_value);
		if (rc)
			return -EINVAL;
	}

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_layer_count_and_type(void *instance,
				      enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_layer_count, hfi_layer_type = 0;

	if (!inst->bufq[OUTPUT_PORT].vb2q->streaming) {
		/* set layer type */
		hfi_layer_type = inst->hfi_layer_type;
		cap_id = LAYER_TYPE;

		rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
						&hfi_layer_type, sizeof(u32), __func__);
		if (rc)
			goto exit;
	} else {
		if (inst->hfi_layer_type == HFI_HIER_B) {
			i_vpr_l(inst,
				"%s: HB dyn layers change is not supported\n",
				__func__);
			return 0;
		}
	}

	/* set layer count */
	cap_id = ENH_LAYER_COUNT;
	/* hfi baselayer starts from 1 */
	hfi_layer_count = inst->capabilities[ENH_LAYER_COUNT].value + 1;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_layer_count, sizeof(u32), __func__);
	if (rc)
		goto exit;

exit:
	return rc;
}

int msm_vidc_set_gop_size(void *instance,
			  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	if (inst->bufq[OUTPUT_PORT].vb2q->streaming) {
		if (inst->hfi_layer_type == HFI_HIER_B) {
			i_vpr_l(inst,
				"%s: HB dyn GOP setting is not supported\n",
				__func__);
			return 0;
		}
	}

	hfi_value = inst->capabilities[GOP_SIZE].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_bitrate(void *instance,
			 enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;

	/* set Total Bitrate */
	if (inst->capabilities[BIT_RATE].flags & CAP_FLAG_CLIENT_SET)
		goto set_total_bitrate;

	/*
	 * During runtime, if BIT_RATE cap CLIENT_SET flag is not set,
	 * then this function will be called due to change in ENH_LAYER_COUNT.
	 * In this case, client did not change bitrate, hence, no need to set
	 * to fw.
	 */
	if (inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

set_total_bitrate:
	hfi_value = inst->capabilities[BIT_RATE].value;
	rc = msm_vidc_packetize_control(inst, BIT_RATE, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_layer_bitrate(void *instance,
			       enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;

	if (!inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

	/* set Total Bitrate */
	if (inst->capabilities[BIT_RATE].flags & CAP_FLAG_CLIENT_SET) {
		i_vpr_h(inst,
			"%s: Total bitrate is set, ignore layer bitrate\n",
			__func__);
		return 0;
	}

	/*
	 * ENH_LAYER_COUNT cap max is positive only if
	 *    layer encoding is enabled during streamon.
	 */
	if (!inst->capabilities[ENH_LAYER_COUNT].max ||
	    !msm_vidc_check_all_layer_bitrate_set(inst)) {
		i_vpr_h(inst,
			"%s: invalid layer bitrate, ignore setting to fw\n",
			__func__);
		return 0;
	}

	/*
	 * Accept layerwise bitrate but set total bitrate which was already
	 * adjusted based on layer bitrate
	 */
	hfi_value = inst->capabilities[BIT_RATE].value;
	rc = msm_vidc_packetize_control(inst, BIT_RATE, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_session_priority(void *instance,
				  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	u32 hfi_value = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	hfi_value = inst->capabilities[cap_id].value;
	if (!is_critical_priority_session(inst))
		hfi_value = inst->capabilities[cap_id].value +
			inst->capabilities[FIRMWARE_PRIORITY_OFFSET].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_flip(void *instance,
		      enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	u32 hflip, vflip, hfi_value = HFI_DISABLE_FLIP;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	hflip = inst->capabilities[HFLIP].value;
	vflip = inst->capabilities[VFLIP].value;

	if (hflip)
		hfi_value |= HFI_HORIZONTAL_FLIP;

	if (vflip)
		hfi_value |= HFI_VERTICAL_FLIP;

	if (inst->bufq[OUTPUT_PORT].vb2q->streaming) {
		if (hfi_value != HFI_DISABLE_FLIP) {
			rc = msm_vidc_set_req_sync_frame(inst,
							 REQUEST_I_FRAME);
			if (rc)
				return rc;
		}
	}

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_preprocess(void *instance,
			    enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_rotation(void *instance,
			  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	rc = msm_vidc_v4l2_to_hfi_enum(inst, cap_id, &hfi_value);
	if (rc)
		return -EINVAL;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_blur_resolution(void *instance,
				 enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 blur_type = -1;
	u32 hfi_value, blur_width, blur_height;

	if (msm_vidc_get_parent_value(inst, cap_id,
				      BLUR_TYPES, &blur_type, __func__))
		return -EINVAL;

	if (blur_type != MSM_VIDC_BLUR_EXTERNAL)
		return 0;

	hfi_value = inst->capabilities[cap_id].value;

	blur_width = (hfi_value & 0xFFFF0000) >> 16;
	blur_height = hfi_value & 0xFFFF;

	if (blur_width > inst->crop.width ||
	    blur_height > inst->crop.height) {
		i_vpr_e(inst,
			"%s: blur wxh: %dx%d exceeds crop wxh: %dx%d\n",
			__func__, blur_width, blur_height,
			inst->crop.width, inst->crop.height);
		hfi_value = 0;
	}

	if (blur_width == inst->crop.width &&
	    blur_height == inst->crop.height) {
		i_vpr_e(inst,
			"%s: blur wxh: %dx%d is equal to crop wxh: %dx%d\n",
			__func__, blur_width, blur_height,
			inst->crop.width, inst->crop.height);
		hfi_value = 0;
	}

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_32_PACKED,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

static int msm_venc_set_csc_coeff(struct msm_vidc_inst *inst,
				  const char *prop_name, u32 hfi_id, void *payload,
	u32 payload_size, u32 row_count, u32 column_count)
{
	int rc = 0;

	i_vpr_h(inst,
		"set cap: name: %24s, hard coded %dx%d matrix array\n",
		prop_name, row_count, column_count);
	rc = venus_hfi_session_property(inst,
					hfi_id,
					HFI_HOST_FLAGS_NONE,
					HFI_PORT_BITSTREAM,
					HFI_PAYLOAD_S32_ARRAY,
					payload,
					payload_size);
	if (rc) {
		i_vpr_e(inst,
			"%s: failed to set %s to fw\n",
			__func__, prop_name);
	}

	return rc;
}

int msm_vidc_set_csc_custom_matrix(void *instance,
				   enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	int i;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	struct msm_vidc_core *core;
	struct msm_vidc_csc_coeff *csc_coeff;
	s32 matrix_payload[MAX_MATRIX_COEFFS + 2];
	s32 csc_bias_payload[MAX_BIAS_COEFFS + 2];
	s32 csc_limit_payload[MAX_LIMIT_COEFFS + 2];

	core = inst->core;
	csc_coeff = &core->platform->data.csc_data;

	if (!inst->capabilities[cap_id].value ||
	    !inst->capabilities[CSC].value) {
		i_vpr_h(inst,
			"%s: ignored as custom martix %u, csc %u\n",
			__func__, inst->capabilities[cap_id].value,
			inst->capabilities[CSC].value);
		return 0;
	}

	/*
	 * first 2 u32's of payload in each case are for
	 * row and column count, next remaining u32's are
	 * for the actual payload values.
	 */

	/* set custom matrix */
	matrix_payload[0] = 3;
	matrix_payload[1] = 3;

	for (i = 0; i < MAX_MATRIX_COEFFS; i++) {
		if ((i + 2) >= ARRAY_SIZE(matrix_payload))
			break;
		matrix_payload[i + 2] =
			csc_coeff->vpe_csc_custom_matrix_coeff[i];
	}

	rc = msm_venc_set_csc_coeff(inst, "CSC_CUSTOM_MATRIX",
				    HFI_PROP_CSC_MATRIX, &matrix_payload[0],
				    ARRAY_SIZE(matrix_payload) * sizeof(s32),
				    matrix_payload[0], matrix_payload[1]);
	if (rc)
		return rc;

	/* set csc bias */
	csc_bias_payload[0] = 1;
	csc_bias_payload[1] = 3;

	for (i = 0; i < MAX_BIAS_COEFFS; i++) {
		if ((i + 2) >= ARRAY_SIZE(csc_bias_payload))
			break;
		csc_bias_payload[i + 2] =
			csc_coeff->vpe_csc_custom_bias_coeff[i];
	}

	rc = msm_venc_set_csc_coeff(inst, "CSC_BIAS",
				    HFI_PROP_CSC_BIAS, &csc_bias_payload[0],
				    ARRAY_SIZE(csc_bias_payload) * sizeof(s32),
				    csc_bias_payload[0], csc_bias_payload[1]);
	if (rc)
		return rc;

	/* set csc limit */
	csc_limit_payload[0] = 1;
	csc_limit_payload[1] = 6;

	for (i = 0; i < MAX_LIMIT_COEFFS; i++) {
		if ((i + 2) >= ARRAY_SIZE(csc_limit_payload))
			break;
		csc_limit_payload[i + 2] =
			csc_coeff->vpe_csc_custom_limit_coeff[i];
	}

	rc = msm_venc_set_csc_coeff(inst, "CSC_LIMIT",
				    HFI_PROP_CSC_LIMIT, &csc_limit_payload[0],
				    ARRAY_SIZE(csc_limit_payload) * sizeof(s32),
				    csc_limit_payload[0], csc_limit_payload[1]);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_reserve_duration(void *instance,
				  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	u32 hfi_value = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	/* reserve hardware only when input port is streaming*/
	if (!inst->bufq[INPUT_PORT].vb2q->streaming)
		return 0;

	if (!(inst->capabilities[cap_id].flags & CAP_FLAG_CLIENT_SET))
		return 0;

	inst->capabilities[cap_id].flags &= (~CAP_FLAG_CLIENT_SET);

	if (!is_critical_priority_session(inst)) {
		i_vpr_h(inst, "%s: reserve duration allowed only for critical session\n", __func__);
		return 0;
	}

	hfi_value = inst->capabilities[cap_id].value;

	rc = venus_hfi_reserve_hardware(inst, hfi_value);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_level(void *instance,
		       enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;

	hfi_value = inst->capabilities[cap_id].value;
	if (!(inst->capabilities[cap_id].flags & CAP_FLAG_CLIENT_SET))
		hfi_value = HFI_LEVEL_NONE;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_q16(void *instance,
		     enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_Q16,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_u32(void *instance,
		     enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	if (inst->capabilities[cap_id].flags & CAP_FLAG_MENU) {
		rc = msm_vidc_v4l2_menu_to_hfi(inst, cap_id, &hfi_value);
		if (rc)
			return -EINVAL;
	} else {
		hfi_value = inst->capabilities[cap_id].value;
	}

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_u32_packed(void *instance,
			    enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	if (inst->capabilities[cap_id].flags & CAP_FLAG_MENU) {
		rc = msm_vidc_v4l2_menu_to_hfi(inst, cap_id, &hfi_value);
		if (rc)
			return -EINVAL;
	} else {
		hfi_value = inst->capabilities[cap_id].value;
	}

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_32_PACKED,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_u32_enum(void *instance,
			  enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	rc = msm_vidc_v4l2_to_hfi_enum(inst, cap_id, &hfi_value);
	if (rc)
		return -EINVAL;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_s32(void *instance,
		     enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 hfi_value = 0;

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_S32,
					&hfi_value, sizeof(s32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_stage(void *instance,
		       enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	u32 stage = 0;
	struct msm_vidc_core *core;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	core = inst->core;

	rc = call_session_op(core, decide_work_mode, inst);
	if (rc) {
		i_vpr_e(inst, "%s: decide_work_mode failed\n", __func__);
		return -EINVAL;
	}

	stage = inst->capabilities[STAGE].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&stage, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_pipe(void *instance,
		      enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	u32 pipe;
	struct msm_vidc_core *core;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	core = inst->core;

	rc = call_session_op(core, decide_work_route, inst);
	if (rc) {
		i_vpr_e(inst, "%s: decide_work_route failed\n",
			__func__);
		return -EINVAL;
	}

	pipe = inst->capabilities[PIPE].value;
	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&pipe, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_vui_timing_info(void *instance,
				 enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	/*
	 * hfi is HFI_PROP_DISABLE_VUI_TIMING_INFO and v4l2 cap is
	 * V4L2_CID_MPEG_VIDC_VUI_TIMING_INFO and hence reverse
	 * the hfi_value from cap_id value.
	 */
	if (inst->capabilities[cap_id].value == 1)
		hfi_value = 0;
	else
		hfi_value = 1;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_outbuf_fence_type(void *instance,
				   enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	if (inst->capabilities[OUTBUF_FENCE_TYPE].value ==
	    MSM_VIDC_FENCE_NONE)
		return 0;

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_set_outbuf_fence_direction(void *instance,
					enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	if (inst->capabilities[OUTBUF_FENCE_DIRECTION].value ==
	    MSM_VIDC_FENCE_DIR_NONE)
		return 0;

	hfi_value = inst->capabilities[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
					&hfi_value, sizeof(u32), __func__);
	if (rc)
		return rc;

	return rc;
}

/********************* End of Control Set functions **************************/
