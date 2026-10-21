// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__
#include <linux/debugfs.h>
#include <drm/sde_drm.h>

#include "sde_encoder_phys.h"
#include "sde_formats.h"
#include "sde_hw_top.h"
#include "sde_hw_interrupts.h"
#include "sde_core_irq.h"
#include "sde_wb.h"
#include "sde_vbif.h"
#include "sde_crtc.h"
#include "sde_hw_dnsc_blur.h"
#include "sde_trace.h"

#define to_sde_encoder_phys_wb(x) \
	container_of(x, struct sde_encoder_phys_wb, base)

#define WBID(wb_enc) \
	((wb_enc && wb_enc->wb_dev) ? wb_enc->wb_dev->wb_idx - WB_0 : -1)

#define TO_S15D16(_x_)	((_x_) << 7)

#define SDE_WB_MAX_LINEWIDTH(fmt, wb_cfg) \
	((SDE_FORMAT_IS_UBWC(fmt) || SDE_FORMAT_IS_YUV(fmt)) ? wb_cfg->sblk->maxlinewidth : \
	wb_cfg->sblk->maxlinewidth_linear)

static const u32 cwb_irq_tbl[PINGPONG_MAX] = {SDE_NONE, INTR_IDX_PP1_OVFL,
	INTR_IDX_PP2_OVFL, INTR_IDX_PP3_OVFL, INTR_IDX_PP4_OVFL,
	INTR_IDX_PP5_OVFL, SDE_NONE, SDE_NONE};

static const u32 dcwb_irq_tbl[PINGPONG_MAX] = {SDE_NONE, SDE_NONE,
	SDE_NONE, SDE_NONE, SDE_NONE, SDE_NONE,
	INTR_IDX_PP_CWB_OVFL, SDE_NONE};

/**
 * sde_rgb2yuv_601l - rgb to yuv color space conversion matrix
 *
 */
static struct sde_csc_cfg sde_encoder_phys_wb_rgb2yuv_601l = {
	{
		TO_S15D16(0x0083), TO_S15D16(0x0102), TO_S15D16(0x0032),
		TO_S15D16(0x1fb5), TO_S15D16(0x1f6c), TO_S15D16(0x00e1),
		TO_S15D16(0x00e1), TO_S15D16(0x1f45), TO_S15D16(0x1fdc)
	},
	{ 0x00, 0x00, 0x00 },
	{ 0x0040, 0x0200, 0x0200 },
	{ 0x000, 0x3ff, 0x000, 0x3ff, 0x000, 0x3ff },
	{ 0x040, 0x3ac, 0x040, 0x3c0, 0x040, 0x3c0 },
};

/**
 * sde_encoder_phys_wb_is_master - report wb always as master encoder
 */
static bool sde_encoder_phys_wb_is_master(struct sde_encoder_phys *phys_enc)
{
	return true;
}

/**
 * sde_encoder_phys_wb_get_intr_type - get interrupt type based on block mode
 * @hw_wb:	Pointer to h/w writeback driver
 */
static enum sde_intr_type sde_encoder_phys_wb_get_intr_type(
		struct sde_hw_wb *hw_wb)
{
	return (hw_wb->caps->features & BIT(SDE_WB_BLOCK_MODE)) ?
			SDE_IRQ_TYPE_WB_ROT_COMP : SDE_IRQ_TYPE_WB_WFD_COMP;
}

/**
 * sde_encoder_phys_wb_set_ot_limit - set OT limit for writeback interface
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_set_ot_limit(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	struct drm_connector_state *conn_state;
	struct sde_vbif_set_ot_params ot_params;
	enum sde_wb_usage_type usage_type;

	conn_state = phys_enc->connector->state;
	usage_type = sde_connector_get_property(conn_state, CONNECTOR_PROP_WB_USAGE_TYPE);

	memset(&ot_params, 0, sizeof(ot_params));
	ot_params.xin_id = hw_wb->caps->xin_id;
	ot_params.num = hw_wb->idx - WB_0;
	ot_params.width = wb_enc->wb_roi.w;
	ot_params.height = wb_enc->wb_roi.h;
	ot_params.is_wfd = ((phys_enc->in_clone_mode) || (usage_type == WB_USAGE_OFFLINE_WB)) ?
					false : true;
	ot_params.frame_rate = drm_mode_vrefresh(&phys_enc->cached_mode);
	ot_params.vbif_idx = hw_wb->caps->vbif_idx;
	ot_params.clk_ctrl = hw_wb->caps->clk_ctrl;
	ot_params.rd = false;

	sde_vbif_set_ot_limit(phys_enc->sde_kms, &ot_params);
}

/**
 * sde_encoder_phys_wb_set_qos_remap - set QoS remapper for writeback
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_set_qos_remap(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc;
	struct sde_hw_wb *hw_wb;
	struct drm_crtc *crtc;
	struct drm_connector_state *conn_state;
	struct sde_vbif_set_qos_params qos_params;
	enum sde_wb_usage_type usage_type;

	if (!phys_enc || !phys_enc->parent || !phys_enc->parent->crtc) {
		SDE_ERROR("invalid arguments\n");
		return;
	}

	wb_enc = to_sde_encoder_phys_wb(phys_enc);
	if (!wb_enc->crtc) {
		SDE_ERROR("[enc:%d, wb:%d] invalid crtc\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	crtc = wb_enc->crtc;
	conn_state = phys_enc->connector->state;
	usage_type = sde_connector_get_property(conn_state, CONNECTOR_PROP_WB_USAGE_TYPE);

	if (!wb_enc->hw_wb || !wb_enc->hw_wb->caps) {
		SDE_ERROR("[enc:%d wb:%d] invalid WB HW\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	hw_wb = wb_enc->hw_wb;

	memset(&qos_params, 0, sizeof(qos_params));
	qos_params.vbif_idx = hw_wb->caps->vbif_idx;
	qos_params.xin_id = hw_wb->caps->xin_id;
	qos_params.clk_ctrl = hw_wb->caps->clk_ctrl;
	qos_params.num = hw_wb->idx - WB_0;
	if (phys_enc->in_clone_mode)
		qos_params.client_type = VBIF_CWB_CLIENT;
	else if (usage_type == WB_USAGE_OFFLINE_WB)
		qos_params.client_type = VBIF_OFFLINE_WB_CLIENT;
	else
		qos_params.client_type = VBIF_NRT_CLIENT;

	SDE_DEBUG("[enc:%d wb:%d] qos_remap - wb:%d vbif:%d xin:%d clone:%d\n",
		DRMID(phys_enc->parent), WBID(wb_enc), qos_params.num,
		qos_params.vbif_idx, qos_params.xin_id, qos_params.client_type);

	sde_vbif_set_qos_remap(phys_enc->sde_kms, &qos_params);
}

/**
 * sde_encoder_phys_wb_set_qos - set QoS/danger/safe LUTs for writeback
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_set_qos(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc;
	struct sde_hw_wb *hw_wb;
	struct drm_connector_state *conn_state;
	struct sde_hw_wb_qos_cfg qos_cfg = {0};
	struct sde_perf_cfg *perf;
	u32 fps_index = 0, lut_index, creq_index, ds_index, frame_rate, qos_count;
	enum sde_wb_usage_type usage_type;

	if (!phys_enc || !phys_enc->sde_kms || !phys_enc->sde_kms->catalog) {
		SDE_ERROR("invalid parameter(s)\n");
		return;
	}

	wb_enc = to_sde_encoder_phys_wb(phys_enc);
	if (!wb_enc->hw_wb) {
		SDE_ERROR("[enc:%d wb:%d] invalid WB HW\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	conn_state = phys_enc->connector->state;
	usage_type = sde_connector_get_property(conn_state, CONNECTOR_PROP_WB_USAGE_TYPE);

	perf = &phys_enc->sde_kms->catalog->perf;
	frame_rate = drm_mode_vrefresh(&phys_enc->cached_mode);

	hw_wb = wb_enc->hw_wb;
	qos_count = perf->qos_refresh_count;
	while ((fps_index < qos_count) && perf->qos_refresh_rate) {
		if ((frame_rate <= perf->qos_refresh_rate[fps_index]) ||
				(fps_index == qos_count - 1))
			break;
		fps_index++;
	}

	qos_cfg.danger_safe_en = true;

	if (phys_enc->in_clone_mode)
		lut_index = (SDE_FORMAT_IS_TILE(wb_enc->wb_fmt)
				|| SDE_FORMAT_IS_UBWC(wb_enc->wb_fmt)) ?
					SDE_QOS_LUT_USAGE_CWB_TILE : SDE_QOS_LUT_USAGE_CWB;
	else
		lut_index = (usage_type == WB_USAGE_OFFLINE_WB) ?
					SDE_QOS_LUT_USAGE_OFFLINE_WB : SDE_QOS_LUT_USAGE_NRT;

	creq_index = lut_index * SDE_CREQ_LUT_TYPE_MAX;
	creq_index += (fps_index * SDE_QOS_LUT_USAGE_MAX * SDE_CREQ_LUT_TYPE_MAX);
	qos_cfg.creq_lut = perf->creq_lut[creq_index];

	ds_index = lut_index * SDE_DANGER_SAFE_LUT_TYPE_MAX;
	ds_index += (fps_index * SDE_QOS_LUT_USAGE_MAX * SDE_DANGER_SAFE_LUT_TYPE_MAX);
	qos_cfg.danger_lut = perf->danger_lut[ds_index];
	qos_cfg.safe_lut = (u32) perf->safe_lut[ds_index];

	SDE_DEBUG("[enc:%d wb:%d] fps:%d mode:%d type:%d luts[0x%x,0x%x 0x%llx]\n",
		DRMID(phys_enc->parent), WBID(wb_enc), frame_rate, phys_enc->in_clone_mode,
		usage_type, qos_cfg.danger_lut, qos_cfg.safe_lut, qos_cfg.creq_lut);

	if (hw_wb->ops.setup_qos_lut)
		hw_wb->ops.setup_qos_lut(hw_wb, &qos_cfg);
}

/**
 * sde_encoder_phys_setup_cdm - setup chroma down block
 * @phys_enc:	Pointer to physical encoder
 * @fb:		Pointer to output framebuffer
 * @format:	Output format
 */
void sde_encoder_phys_setup_cdm(struct sde_encoder_phys *phys_enc, struct drm_framebuffer *fb,
		const struct sde_format *format, struct sde_rect *wb_roi)
{
	struct sde_hw_cdm *hw_cdm;
	struct sde_hw_cdm_cfg *cdm_cfg;
	struct sde_hw_pingpong *hw_pp;
	struct sde_encoder_phys_wb *wb_enc;
	int ret;

	if (!phys_enc || !format)
		return;

	wb_enc = to_sde_encoder_phys_wb(phys_enc);
	cdm_cfg = &phys_enc->cdm_cfg;
	hw_pp = phys_enc->hw_pp;
	hw_cdm = phys_enc->hw_cdm;
	if (!hw_cdm)
		return;

	if (!SDE_FORMAT_IS_YUV(format)) {
		SDE_DEBUG("[enc:%d wb:%d] cdm_disable fmt:%x\n", DRMID(phys_enc->parent),
				WBID(wb_enc), format->base.pixel_format);
		if (hw_cdm && hw_cdm->ops.disable)
			hw_cdm->ops.disable(hw_cdm);

		return;
	}

	memset(cdm_cfg, 0, sizeof(struct sde_hw_cdm_cfg));

	if (!wb_roi)
		return;

	cdm_cfg->output_width = wb_roi->w;
	cdm_cfg->output_height = wb_roi->h;
	cdm_cfg->output_fmt = format;
	cdm_cfg->output_type = CDM_CDWN_OUTPUT_WB;
	cdm_cfg->output_bit_depth = SDE_FORMAT_IS_DX(format) ?
		CDM_CDWN_OUTPUT_10BIT : CDM_CDWN_OUTPUT_8BIT;

	/* enable 10 bit logic */
	switch (cdm_cfg->output_fmt->chroma_sample) {
	case SDE_CHROMA_RGB:
		cdm_cfg->h_cdwn_type = CDM_CDWN_DISABLE;
		cdm_cfg->v_cdwn_type = CDM_CDWN_DISABLE;
		break;
	case SDE_CHROMA_H2V1:
		cdm_cfg->h_cdwn_type = CDM_CDWN_COSITE;
		cdm_cfg->v_cdwn_type = CDM_CDWN_DISABLE;
		break;
	case SDE_CHROMA_420:
		cdm_cfg->h_cdwn_type = CDM_CDWN_COSITE;
		cdm_cfg->v_cdwn_type = CDM_CDWN_OFFSITE;
		break;
	case SDE_CHROMA_H1V2:
	default:
		SDE_ERROR("[enc:%d wb:%d] unsupported chroma sampling type\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		cdm_cfg->h_cdwn_type = CDM_CDWN_DISABLE;
		cdm_cfg->v_cdwn_type = CDM_CDWN_DISABLE;
		break;
	}

	SDE_DEBUG("[enc:%d wb:%d] cdm_enable:%d,%d,%X,%d,%d,%d,%d]\n",
		DRMID(phys_enc->parent), WBID(wb_enc), cdm_cfg->output_width,
		cdm_cfg->output_height, cdm_cfg->output_fmt->base.pixel_format,
		cdm_cfg->output_type, cdm_cfg->output_bit_depth,
		cdm_cfg->h_cdwn_type, cdm_cfg->v_cdwn_type);

	if (hw_cdm && hw_cdm->ops.setup_csc_data) {
		ret = hw_cdm->ops.setup_csc_data(hw_cdm, &sde_encoder_phys_wb_rgb2yuv_601l);
		if (ret < 0) {
			SDE_ERROR("[enc:%d wb:%d] failed to setup CSC; ret:%d\n",
					DRMID(phys_enc->parent), WBID(wb_enc), ret);
			return;
		}
	}

	if (hw_cdm && hw_cdm->ops.setup_cdwn) {
		ret = hw_cdm->ops.setup_cdwn(hw_cdm, cdm_cfg);
		if (ret < 0) {
			SDE_ERROR("[enc:%d wb:%d] failed to setup CDWN; ret:%d\n",
					DRMID(phys_enc->parent), WBID(wb_enc), ret);
			return;
		}
	}

	if (hw_cdm && hw_pp && hw_cdm->ops.enable) {
		cdm_cfg->pp_id = hw_pp->idx;
		ret = hw_cdm->ops.enable(hw_cdm, cdm_cfg);
		if (ret < 0) {
			SDE_ERROR("[enc:%d wb:%d] failed to enable CDM; ret:%d\n",
					DRMID(phys_enc->parent), WBID(wb_enc), ret);
			return;
		}
	}
}

static void _sde_enc_phys_wb_get_out_resolution(struct drm_crtc_state *crtc_state,
			struct drm_connector_state *conn_state, u32 *out_width, u32 *out_height)
{
	struct sde_crtc_state *cstate = to_sde_crtc_state(crtc_state);
	const struct drm_display_mode *mode = &crtc_state->mode;
	struct sde_io_res ds_res = {0, }, dnsc_blur_res = {0, };
	u32 ds_tap_pt = sde_crtc_get_property(cstate, CRTC_PROP_CAPTURE_OUTPUT);

	sde_crtc_get_ds_io_res(crtc_state, &ds_res);
	sde_connector_get_dnsc_blur_io_res(conn_state, &dnsc_blur_res);

	if (dnsc_blur_res.enabled) {
		*out_width = dnsc_blur_res.dst_w;
		*out_height = dnsc_blur_res.dst_h;
	} else if (ds_res.enabled) {
		if (ds_tap_pt == CAPTURE_DSPP_OUT) {
			*out_width = ds_res.dst_w;
			*out_height = ds_res.dst_h;
		} else if (ds_tap_pt == CAPTURE_MIXER_OUT) {
			*out_width = ds_res.src_w;
			*out_height = ds_res.src_h;
		} else {
			*out_width = mode->hdisplay;
			*out_height = mode->vdisplay;
		}
	}  else {
		*out_width = mode->hdisplay;
		*out_height = mode->vdisplay;
	}
}

static void _sde_encoder_phys_wb_setup_cdp(struct sde_encoder_phys *phys_enc,
		struct sde_hw_wb_cfg *wb_cfg)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	struct sde_hw_wb_cdp_cfg *cdp_cfg = &wb_enc->cdp_cfg;
	u32 cdp_index;

	if (!hw_wb->ops.setup_cdp)
		return;

	memset(cdp_cfg, 0, sizeof(struct sde_hw_wb_cdp_cfg));

	cdp_index = phys_enc->in_clone_mode ? SDE_PERF_CDP_USAGE_RT : SDE_PERF_CDP_USAGE_NRT;
	cdp_cfg->enable = phys_enc->sde_kms->catalog->perf.cdp_cfg[cdp_index].wr_enable;
	cdp_cfg->ubwc_meta_enable = SDE_FORMAT_IS_UBWC(wb_cfg->dest.format);
	cdp_cfg->tile_amortize_enable = SDE_FORMAT_IS_UBWC(wb_cfg->dest.format) ||
						SDE_FORMAT_IS_TILE(wb_cfg->dest.format);
	cdp_cfg->preload_ahead = SDE_WB_CDP_PRELOAD_AHEAD_64;

	hw_wb->ops.setup_cdp(hw_wb, cdp_cfg);
}

static void _sde_encoder_phys_wb_setup_roi(struct sde_encoder_phys *phys_enc,
		struct sde_hw_wb_cfg *wb_cfg, u32 out_width, u32 out_height)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	struct drm_crtc_state *crtc_state = wb_enc->crtc->state;
	struct sde_crtc_state *cstate = to_sde_crtc_state(crtc_state);
	struct sde_rect pu_roi = {0,};

	if (!hw_wb->ops.setup_roi)
		return;

	if (hw_wb->ops.setup_crop && phys_enc->in_clone_mode) {
		wb_cfg->crop.x = wb_cfg->roi.x;
		wb_cfg->crop.y = wb_cfg->roi.y;

		if (cstate->user_roi_list.num_rects) {
			sde_kms_rect_merge_rectangles(&cstate->user_roi_list, &pu_roi);

			if ((wb_cfg->roi.w != pu_roi.w) || (wb_cfg->roi.h != pu_roi.h)) {
				/* offset cropping region to PU region */
				wb_cfg->crop.x = wb_cfg->crop.x - pu_roi.x;
				wb_cfg->crop.y = wb_cfg->crop.y - pu_roi.y;
				hw_wb->ops.setup_crop(hw_wb, wb_cfg, true);
			}
		} else if ((wb_cfg->roi.w != out_width) || (wb_cfg->roi.h != out_height)) {
			hw_wb->ops.setup_crop(hw_wb, wb_cfg, true);
		} else {
			hw_wb->ops.setup_crop(hw_wb, wb_cfg, false);
		}

		/* If output buffer is less than source size, align roi at top left corner */
		if (wb_cfg->dest.width < out_width || wb_cfg->dest.height < out_height) {
			wb_cfg->roi.x = 0;
			wb_cfg->roi.y = 0;
		}

		SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), wb_cfg->crop.x, wb_cfg->crop.y,
				pu_roi.x, pu_roi.y, pu_roi.w, pu_roi.h);
	}

	hw_wb->ops.setup_roi(hw_wb, wb_cfg);
}

static void _sde_encoder_phys_wb_setup_out_cfg(struct sde_encoder_phys *phys_enc,
		struct sde_hw_wb_cfg *wb_cfg)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;

	SDE_DEBUG("[enc:%d wb:%d] [fb_offset:%8.8x,%8.8x,%8.8x,%8.8x], fb_sec:%d\n",
		DRMID(phys_enc->parent), WBID(wb_enc), wb_cfg->dest.plane_addr[0],
		wb_cfg->dest.plane_addr[1], wb_cfg->dest.plane_addr[2],
		wb_cfg->dest.plane_addr[3], wb_cfg->is_secure);
	SDE_DEBUG("[fb_stride:%8.8x,%8.8x,%8.8x,%8.8x]\n", wb_cfg->dest.plane_pitch[0],
		wb_cfg->dest.plane_pitch[1], wb_cfg->dest.plane_pitch[2],
		wb_cfg->dest.plane_pitch[3]);

	if (hw_wb->ops.setup_outformat)
		hw_wb->ops.setup_outformat(hw_wb, wb_cfg);

	if (hw_wb->ops.setup_outaddress) {
		SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc),
			wb_cfg->dest.width, wb_cfg->dest.height,
			wb_cfg->dest.plane_addr[0], wb_cfg->dest.plane_size[0],
			wb_cfg->dest.plane_addr[1], wb_cfg->dest.plane_size[1],
			wb_cfg->dest.plane_addr[2], wb_cfg->dest.plane_size[2],
			wb_cfg->dest.plane_addr[3], wb_cfg->dest.plane_size[3],
			wb_cfg->roi.x, wb_cfg->roi.y, wb_cfg->roi.w, wb_cfg->roi.h);
		hw_wb->ops.setup_outaddress(hw_wb, wb_cfg);
	}
}

/**
 * sde_encoder_phys_wb_setup_fb - setup output framebuffer
 * @phys_enc:	Pointer to physical encoder
 * @fb:		Pointer to output framebuffer
 * @wb_roi:	Pointer to output region of interest
 */
static void sde_encoder_phys_wb_setup_fb(struct sde_encoder_phys *phys_enc,
		struct drm_framebuffer *fb, struct sde_rect *wb_roi, u32 out_width, u32 out_height)
{
	struct sde_encoder_phys_wb *wb_enc;
	struct sde_hw_wb *hw_wb;
	struct sde_hw_wb_cfg *wb_cfg;
	const struct msm_format *format;
	int ret;
	struct msm_gem_address_space *aspace;
	u32 fb_mode;

	if (!phys_enc || !phys_enc->sde_kms || !phys_enc->sde_kms->catalog ||
			!phys_enc->connector) {
		SDE_ERROR("invalid encoder\n");
		return;
	}

	wb_enc = to_sde_encoder_phys_wb(phys_enc);
	hw_wb = wb_enc->hw_wb;
	wb_cfg = &wb_enc->wb_cfg;
	memset(wb_cfg, 0, sizeof(struct sde_hw_wb_cfg));

	wb_cfg->intf_mode = phys_enc->intf_mode;

	fb_mode = sde_connector_get_property(phys_enc->connector->state,
			CONNECTOR_PROP_FB_TRANSLATION_MODE);
	if (phys_enc->enable_state == SDE_ENC_DISABLING)
		wb_cfg->is_secure = false;
	else
		wb_cfg->is_secure = (fb_mode == SDE_DRM_FB_SEC) ? true : false;

	aspace = (wb_cfg->is_secure) ? wb_enc->aspace[SDE_IOMMU_DOMAIN_SECURE] :
			wb_enc->aspace[SDE_IOMMU_DOMAIN_UNSECURE];

	ret = msm_framebuffer_prepare(fb, aspace);
	if (ret) {
		SDE_ERROR("[enc:%d wb:%d] prep fb failed; fb_sec:%d, ret:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), wb_cfg->is_secure, ret);
		return;
	}

	/* cache framebuffer for cleanup in writeback done */
	wb_enc->wb_fb = fb;
	wb_enc->wb_aspace = aspace;
	drm_framebuffer_get(fb);

	format = msm_framebuffer_format(fb);
	if (!format) {
		SDE_DEBUG("[enc:%d wb:%d] invalid fb fmt\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	wb_cfg->dest.format = sde_get_sde_format_ext(format->pixel_format, fb->modifier);
	if (!wb_cfg->dest.format) {
		/* this error should be detected during atomic_check */
		SDE_ERROR("[enc:%d wb:%d] failed to get format:%x\n",
				DRMID(phys_enc->parent), WBID(wb_enc), format->pixel_format);
		return;
	}
	wb_cfg->roi = *wb_roi;

	ret = sde_format_populate_layout(aspace, fb, &wb_cfg->dest);
	if (ret) {
		SDE_DEBUG("[enc:%d wb:%d] failed to populate layout; ret:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), ret);
		return;
	}
	wb_cfg->dest.width = fb->width;
	wb_cfg->dest.height = fb->height;
	wb_cfg->dest.num_planes = wb_cfg->dest.format->num_planes;

	if ((wb_cfg->dest.format->fetch_planes == SDE_PLANE_PLANAR) &&
			(wb_cfg->dest.format->element[0] == C1_B_Cb))
		swap(wb_cfg->dest.plane_addr[1], wb_cfg->dest.plane_addr[2]);

	_sde_encoder_phys_wb_setup_roi(phys_enc, wb_cfg, out_width, out_height);

	_sde_encoder_phys_wb_setup_cdp(phys_enc, wb_cfg);

	_sde_encoder_phys_wb_setup_out_cfg(phys_enc, wb_cfg);

}

static void _sde_encoder_phys_wb_setup_cwb(struct sde_encoder_phys *phys_enc, bool enable)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	struct sde_hw_wb_cfg *wb_cfg = &wb_enc->wb_cfg;
	struct sde_hw_ctl *hw_ctl = phys_enc->hw_ctl;
	struct sde_crtc *crtc = to_sde_crtc(wb_enc->crtc);
	struct sde_hw_pingpong *hw_pp = phys_enc->hw_pp;
	struct sde_hw_dnsc_blur *hw_dnsc_blur = phys_enc->hw_dnsc_blur;
	bool need_merge = (crtc->num_mixers > 1);
	enum sde_dcwb;
	int i = 0;
	const int num_wb = 1;

	if (!phys_enc->in_clone_mode) {
		SDE_DEBUG("[enc:%d wb:%d] not in CWB mode. early return\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	if (!hw_pp || !hw_ctl || !hw_wb || hw_pp->idx >= PINGPONG_MAX) {
		SDE_ERROR("[enc:%d wb:%d] invalid hw resources - return\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	hw_ctl = crtc->mixers[0].hw_ctl;
	if (hw_ctl && hw_ctl->ops.setup_intf_cfg_v1 &&
			(test_bit(SDE_WB_CWB_CTRL, &hw_wb->caps->features) ||
			test_bit(SDE_WB_DCWB_CTRL, &hw_wb->caps->features))) {
		struct sde_hw_intf_cfg_v1 intf_cfg = { 0, };

		intf_cfg.wb_count = num_wb;
		intf_cfg.wb[0] = hw_wb->idx;

		for (i = 0; i < crtc->num_mixers; i++) {
			if (test_bit(SDE_WB_DCWB_CTRL, &hw_wb->caps->features))
				intf_cfg.cwb[intf_cfg.cwb_count++] =
						(enum sde_cwb)(hw_pp->dcwb_idx + i);
			else
				intf_cfg.cwb[intf_cfg.cwb_count++] = (enum sde_cwb)(hw_pp->idx + i);
		}

		if (hw_pp->merge_3d && (intf_cfg.merge_3d_count <
				MAX_MERGE_3D_PER_CTL_V1) && need_merge)
			intf_cfg.merge_3d[intf_cfg.merge_3d_count++] = hw_pp->merge_3d->idx;

		if (hw_dnsc_blur)
			intf_cfg.dnsc_blur[intf_cfg.dnsc_blur_count++] = hw_dnsc_blur->idx;

		if (hw_pp->ops.setup_3d_mode)
			hw_pp->ops.setup_3d_mode(hw_pp, (enable && need_merge) ?
					BLEND_3D_H_ROW_INT : 0);

		if ((hw_wb->ops.bind_pingpong_blk) &&
				test_bit(SDE_WB_CWB_CTRL, &hw_wb->caps->features))
			hw_wb->ops.bind_pingpong_blk(hw_wb, enable, hw_pp->idx);

		if ((hw_wb->ops.bind_dcwb_pp_blk) &&
				test_bit(SDE_WB_DCWB_CTRL, &hw_wb->caps->features))
			hw_wb->ops.bind_dcwb_pp_blk(hw_wb, enable, hw_pp->idx);

		if (hw_wb->ops.setup_crop && !enable)
			hw_wb->ops.setup_crop(hw_wb, wb_cfg, false);

		if (hw_ctl->ops.update_intf_cfg) {
			hw_ctl->ops.update_intf_cfg(hw_ctl, &intf_cfg, enable);
			SDE_DEBUG("[enc:%d wb:%d] in CWB/DCWB mode on CTL_%d PP-%d merge3d:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc),
				hw_ctl->idx - CTL_0, hw_pp->idx - PINGPONG_0,
				hw_pp->merge_3d ? hw_pp->merge_3d->idx - MERGE_3D_0 : -1);
		}
	} else {
		struct sde_hw_intf_cfg *intf_cfg = &phys_enc->intf_cfg;

		memset(intf_cfg, 0, sizeof(struct sde_hw_intf_cfg));
		intf_cfg->intf = SDE_NONE;
		intf_cfg->wb = hw_wb->idx;

		if (hw_ctl && hw_ctl->ops.update_wb_cfg) {
			hw_ctl->ops.update_wb_cfg(hw_ctl, intf_cfg, enable);
			SDE_DEBUG("[enc:%d wb:%d] in CWB/DCWB mode adding WB for CTL_%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), hw_ctl->idx - CTL_0);
		}
	}
}

static void _sde_encoder_phys_wb_setup_ctl(struct sde_encoder_phys *phys_enc,
		const struct sde_format *format)
{
	struct sde_encoder_phys_wb *wb_enc;
	struct sde_hw_wb *hw_wb;
	struct sde_hw_cdm *hw_cdm;
	struct sde_hw_dnsc_blur *hw_dnsc_blur;
	struct sde_hw_ctl *ctl;
	const int num_wb = 1;

	if (!phys_enc) {
		SDE_ERROR("invalid encoder\n");
		return;
	}
	wb_enc = to_sde_encoder_phys_wb(phys_enc);

	if (phys_enc->in_clone_mode) {
		SDE_DEBUG("[enc:%d wb:%d] in CWB mode. early return\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	hw_wb = wb_enc->hw_wb;
	hw_cdm = phys_enc->hw_cdm;
	hw_dnsc_blur = phys_enc->hw_dnsc_blur;
	ctl = phys_enc->hw_ctl;

	if (test_bit(SDE_CTL_ACTIVE_CFG, &ctl->caps->features) &&
			(phys_enc->hw_ctl && phys_enc->hw_ctl->ops.setup_intf_cfg_v1)) {
		struct sde_hw_intf_cfg_v1 *intf_cfg_v1 = &phys_enc->intf_cfg_v1;
		struct sde_hw_pingpong *hw_pp = phys_enc->hw_pp;
		enum sde_3d_blend_mode mode_3d;

		memset(intf_cfg_v1, 0, sizeof(struct sde_hw_intf_cfg_v1));

		mode_3d = sde_encoder_helper_get_3d_blend_mode(phys_enc);
		intf_cfg_v1->intf_count = SDE_NONE;
		intf_cfg_v1->wb_count = num_wb;
		intf_cfg_v1->wb[0] = hw_wb->idx;
		if (SDE_FORMAT_IS_YUV(format)) {
			intf_cfg_v1->cdm_count = num_wb;
			intf_cfg_v1->cdm[0] = hw_cdm->idx;
		}

		if (hw_dnsc_blur) {
			intf_cfg_v1->dnsc_blur_count = num_wb;
			intf_cfg_v1->dnsc_blur[0] = hw_dnsc_blur->idx;
		}

		if (mode_3d && hw_pp && hw_pp->merge_3d &&
			intf_cfg_v1->merge_3d_count < MAX_MERGE_3D_PER_CTL_V1)
			intf_cfg_v1->merge_3d[intf_cfg_v1->merge_3d_count++] = hw_pp->merge_3d->idx;

		if (hw_pp && hw_pp->ops.setup_3d_mode)
			hw_pp->ops.setup_3d_mode(hw_pp, mode_3d);

		/* setup which pp blk will connect to this wb */
		if (hw_pp && hw_wb->ops.bind_pingpong_blk)
			hw_wb->ops.bind_pingpong_blk(hw_wb, true, hw_pp->idx);

		phys_enc->hw_ctl->ops.setup_intf_cfg_v1(phys_enc->hw_ctl, intf_cfg_v1);
	} else if (phys_enc->hw_ctl && phys_enc->hw_ctl->ops.setup_intf_cfg) {
		struct sde_hw_intf_cfg *intf_cfg = &phys_enc->intf_cfg;

		memset(intf_cfg, 0, sizeof(struct sde_hw_intf_cfg));

		intf_cfg->intf = SDE_NONE;
		intf_cfg->wb = hw_wb->idx;
		intf_cfg->mode_3d = sde_encoder_helper_get_3d_blend_mode(phys_enc);
		phys_enc->hw_ctl->ops.setup_intf_cfg(phys_enc->hw_ctl, intf_cfg);
	}

}

static void _sde_enc_phys_wb_detect_cwb(struct sde_encoder_phys *phys_enc,
		struct drm_crtc_state *crtc_state)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_crtc_state *cstate = to_sde_crtc_state(crtc_state);
	const struct sde_wb_cfg *wb_cfg = wb_enc->hw_wb->caps;
	u32 encoder_mask = 0;

	/* Check if WB has CWB support */
	if ((wb_cfg->features & BIT(SDE_WB_HAS_CWB)) || (wb_cfg->features & BIT(SDE_WB_HAS_DCWB))) {
		encoder_mask = crtc_state->encoder_mask;
		encoder_mask &= ~drm_encoder_mask(phys_enc->parent);
	}

	cstate->cwb_enc_mask = encoder_mask ? drm_encoder_mask(phys_enc->parent) : 0;

	SDE_DEBUG("[enc:%d wb:%d] detect CWB - status:%d, phys state:%d in_clone_mode:%d\n",
		 DRMID(phys_enc->parent), WBID(wb_enc), cstate->cwb_enc_mask,
		 phys_enc->enable_state, phys_enc->in_clone_mode);
}

static int _sde_enc_phys_wb_validate_dnsc_blur_filter(
		struct sde_dnsc_blur_filter_info *filter_info, u32 src, u32 dst)
{
	u32 dnsc_ratio;

	if (!src || !dst || (src < dst)) {
		SDE_ERROR("invalid dnsc_blur src:%u, dst:%u\n", src, dst);
		return -EINVAL;
	}

	dnsc_ratio = DIV_ROUND_UP(src, dst);

	if ((src < filter_info->src_min) || (src > filter_info->src_max)
			|| (dst < filter_info->dst_min) || (dst > filter_info->dst_max)) {
		SDE_ERROR(
		  "invalid dnsc_blur size, fil:%d, src/dst:%u/%u, [min/max-src:%u/%u, dst:%u/%u]\n",
				filter_info->filter, src, dst, filter_info->src_min,
				filter_info->src_max, filter_info->dst_min, filter_info->dst_max);
		return -EINVAL;
	}  else if ((dnsc_ratio < filter_info->min_ratio)
			|| (dnsc_ratio > filter_info->max_ratio)) {
		SDE_ERROR(
		  "invalid dnsc_blur ratio, fil:%d, src/dst:%u/%u, ratio:%u, ratio-min/max:%u/%u\n",
				filter_info->filter, src, dst, dnsc_ratio,
				filter_info->min_ratio, filter_info->max_ratio);
		return -EINVAL;
	}

	return 0;
}

static int _sde_enc_phys_wb_validate_dnsc_blur_filters(struct drm_crtc_state *crtc_state,
		struct drm_connector_state *conn_state)
{
	struct sde_connector_state *sde_conn_state = to_sde_connector_state(conn_state);
	struct sde_dnsc_blur_filter_info *filter_info;
	struct sde_drm_dnsc_blur_cfg *cfg;
	struct sde_kms *sde_kms;
	int ret = 0, i, j;

	sde_kms = sde_connector_get_kms(conn_state->connector);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return -EINVAL;
	}

	for (i = 0; i < sde_conn_state->dnsc_blur_count; i++) {
		cfg = &sde_conn_state->dnsc_blur_cfg[i];

		for (j = 0; j < sde_kms->catalog->dnsc_blur_filter_count; j++) {
			filter_info = &sde_kms->catalog->dnsc_blur_filters[i];
			if (cfg->flags_h == filter_info->filter) {
				ret = _sde_enc_phys_wb_validate_dnsc_blur_filter(filter_info,
						cfg->src_width, cfg->dst_width);
				if (ret)
					break;
			}
			if (cfg->flags_v == filter_info->filter) {
				ret = _sde_enc_phys_wb_validate_dnsc_blur_filter(filter_info,
						cfg->src_height, cfg->dst_height);
				if (ret)
					break;
			}
		}
	}

	return ret;
}

static int _sde_enc_phys_wb_validate_dnsc_blur_ds(struct drm_crtc_state *crtc_state,
			struct drm_connector_state *conn_state, const struct sde_format *fmt,
			struct sde_rect *wb_roi)
{
	struct sde_crtc_state *cstate = to_sde_crtc_state(crtc_state);
	const struct drm_display_mode *mode = &crtc_state->mode;
	struct sde_io_res ds_res = {0, }, dnsc_blur_res = {0, };
	u32 ds_tap_pt = sde_crtc_get_property(cstate, CRTC_PROP_CAPTURE_OUTPUT);

	sde_crtc_get_ds_io_res(crtc_state, &ds_res);
	sde_connector_get_dnsc_blur_io_res(conn_state, &dnsc_blur_res);

	/* wb_roi should match with mode w/h if none of these features are enabled */
	if ((!ds_res.enabled && !dnsc_blur_res.enabled && !cstate->cwb_enc_mask)
			&& ((wb_roi->w && (wb_roi->w != mode->hdisplay))
				|| (wb_roi->h && (wb_roi->h != mode->vdisplay)))) {
		SDE_ERROR("invalid wb-roi {%u,%u,%u,%u} mode:%ux%u\n",
				wb_roi->x, wb_roi->y, wb_roi->w, wb_roi->h,
				mode->hdisplay, mode->vdisplay);
		return -EINVAL;
	}

	if (!dnsc_blur_res.enabled)
		return 0;

	if (!dnsc_blur_res.src_w || !dnsc_blur_res.src_h
			|| !dnsc_blur_res.dst_w || !dnsc_blur_res.dst_h
			|| (dnsc_blur_res.src_w < dnsc_blur_res.dst_w)
			|| (dnsc_blur_res.src_h < dnsc_blur_res.dst_h)) {
		SDE_ERROR("invalid dnsc_blur cfg src:%ux%u dst:%ux%u\n",
				dnsc_blur_res.src_w, dnsc_blur_res.src_h,
				dnsc_blur_res.dst_w, dnsc_blur_res.dst_h);
		return -EINVAL;
	} else if (ds_res.enabled && (ds_tap_pt == CAPTURE_DSPP_OUT)
			&& ((ds_res.dst_w  != dnsc_blur_res.src_w)
				|| (ds_res.dst_h != dnsc_blur_res.src_h))) {
		SDE_ERROR("invalid DSPP OUT cfg: ds dst:%ux%u dnsc_blur src:%ux%u\n",
				ds_res.dst_w, ds_res.dst_h,
				dnsc_blur_res.src_w, dnsc_blur_res.src_h);
		return -EINVAL;
	} else if (ds_res.enabled && (ds_tap_pt == CAPTURE_MIXER_OUT)
			&& ((ds_res.src_w  != dnsc_blur_res.src_w)
				|| (ds_res.src_h != dnsc_blur_res.src_h))) {
		SDE_ERROR("invalid MIXER OUT cfg: ds src:%ux%u dnsc_blur src:%ux%u\n",
				ds_res.dst_w, ds_res.dst_h,
				dnsc_blur_res.src_w, dnsc_blur_res.src_h);
		return -EINVAL;
	} else if (cstate->user_roi_list.num_rects) {
		SDE_ERROR("PU with dnsc_blur not supported\n");
		return -EINVAL;
	} else if (SDE_FORMAT_IS_YUV(fmt)) {
		SDE_ERROR("YUV output not supported with dnsc_blur\n");
		return -EINVAL;
	} else if ((wb_roi->w && (wb_roi->w != dnsc_blur_res.dst_w)) ||
			(wb_roi->h && (wb_roi->h != dnsc_blur_res.dst_h))) {
		SDE_ERROR("invalid WB ROI with dnsc_blur, roi:{%d,%d,%d,%d}, dnsc_blur dst:%ux%u\n",
				wb_roi->x, wb_roi->y, wb_roi->w, wb_roi->h,
				dnsc_blur_res.dst_w, dnsc_blur_res.dst_h);
		return -EINVAL;
	}

	return _sde_enc_phys_wb_validate_dnsc_blur_filters(crtc_state, conn_state);
}

static int _sde_enc_phys_wb_validate_cwb(struct sde_encoder_phys *phys_enc,
			struct drm_crtc_state *crtc_state,
			struct drm_connector_state *conn_state)
{
	struct drm_framebuffer *fb;
	struct sde_crtc_state *cstate = to_sde_crtc_state(crtc_state);
	struct sde_rect wb_roi = {0,}, pu_roi = {0,};
	u32  out_width = 0, out_height = 0;
	const struct sde_format *fmt;
	int prog_line, ret = 0;

	fb = sde_wb_connector_state_get_output_fb(conn_state);
	if (!fb) {
		SDE_DEBUG("no output framebuffer\n");
		return 0;
	}

	fmt = sde_get_sde_format_ext(fb->format->format, fb->modifier);
	if (!fmt) {
		SDE_ERROR("unsupported output pixel format:%x\n", fb->format->format);
		return -EINVAL;
	}

	ret = sde_wb_connector_state_get_output_roi(conn_state, &wb_roi);
	if (ret) {
		SDE_ERROR("failed to get roi %d\n", ret);
		return ret;
	}

	if (!wb_roi.w || !wb_roi.h) {
		SDE_ERROR("cwb roi is not set wxh:%dx%d\n", wb_roi.w, wb_roi.h);
		return -EINVAL;
	}

	prog_line = sde_connector_get_property(conn_state, CONNECTOR_PROP_EARLY_FENCE_LINE);
	if (prog_line) {
		SDE_ERROR("early fence not supported with CWB, prog_line:%d\n", prog_line);
		return -EINVAL;
	}

	/*
	 * 1) No DS case: same restrictions for LM & DSSPP tap point
	 *	a) wb-roi should be inside FB
	 *	b) mode resolution & wb-roi should be same
	 * 2) With DS case: restrictions would change based on tap point
	 *	2.1) LM Tap Point:
	 *		a) wb-roi should be inside FB
	 *		b) wb-roi should be same as crtc-LM bounds
	 *	2.2) DSPP Tap point: same as No DS case
	 *		a) wb-roi should be inside FB
	 *		b) mode resolution & wb-roi should be same
	 * 3) With DNSC_BLUR case:
	 *      a) wb-roi should be inside FB
	 *      b) mode resolution and wb-roi should be same
	 * 4) Partial Update case: additional stride check
	 *      a) cwb roi should be inside PU region or FB
	 *      b) cropping is only allowed for fully sampled data
	 *      c) add check for stride and QOS setting by 256B
	 */
	_sde_enc_phys_wb_get_out_resolution(crtc_state, conn_state, &out_width, &out_height);

	if (SDE_FORMAT_IS_YUV(fmt) && ((wb_roi.w != out_width) || (wb_roi.h != out_height))) {
		SDE_ERROR("invalid wb roi[%dx%d] out[%dx%d] fmt:%x\n",
				wb_roi.w, wb_roi.h, out_width, out_height, fmt->base.pixel_format);
		return -EINVAL;
	}

	if ((wb_roi.w > out_width) || (wb_roi.h > out_height)) {
		SDE_ERROR("invalid wb roi[%dx%d] out[%dx%d]\n",
				wb_roi.w, wb_roi.h, out_width, out_height);
		return -EINVAL;
	}

	/*
	 * If output size is equal to input size ensure wb_roi with x and y offset
	 * will be within buffer. If output size is smaller, only width and height are taken
	 * into consideration as output region will begin at top left corner
	 */
	if ((fb->width == out_width && fb->height == out_height) &&
			(((wb_roi.x + wb_roi.w) > fb->width)
				|| ((wb_roi.y + wb_roi.h) > fb->height))) {
		SDE_ERROR("invalid wb roi[%d,%d,%d,%d] fb[%dx%d] out[%dx%d]\n",
				wb_roi.x, wb_roi.y, wb_roi.w, wb_roi.h, fb->width, fb->height,
				out_width, out_height);
		return -EINVAL;
	} else if ((fb->width < out_width || fb->height < out_height) &&
			((wb_roi.w > fb->width || wb_roi.h > fb->height))) {
		SDE_ERROR("invalid wb roi[%d,%d,%d,%d] fb[%dx%d] out[%dx%d]\n",
				wb_roi.x, wb_roi.y, wb_roi.w, wb_roi.h, fb->width, fb->height,
				out_width, out_height);
		return -EINVAL;
	}

	/* validate wb roi against pu rect */
	if (cstate->user_roi_list.num_rects) {
		sde_kms_rect_merge_rectangles(&cstate->user_roi_list, &pu_roi);
		if (wb_roi.w > pu_roi.w || wb_roi.h > pu_roi.h) {
			SDE_ERROR("invalid wb roi with pu [%dx%d vs %dx%d]\n",
					wb_roi.w, wb_roi.h, pu_roi.w, pu_roi.h);
			return -EINVAL;
		}
	}

	return ret;
}

/**
 * sde_encoder_phys_wb_atomic_check - verify and fixup given atomic states
 * @phys_enc:	Pointer to physical encoder
 * @crtc_state:	Pointer to CRTC atomic state
 * @conn_state:	Pointer to connector atomic state
 */
static int sde_encoder_phys_wb_atomic_check(struct sde_encoder_phys *phys_enc,
		struct drm_crtc_state *crtc_state, struct drm_connector_state *conn_state)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_crtc_state *cstate = to_sde_crtc_state(crtc_state);
	struct sde_connector_state *sde_conn_state;
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	const struct sde_wb_cfg *wb_cfg = hw_wb->caps;
	struct drm_framebuffer *fb;
	const struct sde_format *fmt;
	struct sde_rect wb_roi;
	u32 out_width = 0, out_height = 0;
	const struct drm_display_mode *mode = &crtc_state->mode;
	int rc;
	bool clone_mode_curr = false;

	SDE_DEBUG("[enc:%d wb:%d] atomic_check:\"%s\",%d,%d]\n", DRMID(phys_enc->parent),
			WBID(wb_enc), mode->name, mode->hdisplay, mode->vdisplay);

	if (!conn_state || !conn_state->connector) {
		SDE_ERROR("[enc:%d wb:%d] invalid connector state\n",
			DRMID(phys_enc->parent), WBID(wb_enc));
		return -EINVAL;
	} else if (conn_state->connector->status != connector_status_connected) {
		SDE_ERROR("[enc:%d wb:%d] connector not connected; ret:%d\n",
			DRMID(phys_enc->parent), WBID(wb_enc), conn_state->connector->status);
		return -EINVAL;
	}

	sde_conn_state = to_sde_connector_state(conn_state);
	clone_mode_curr = phys_enc->in_clone_mode;

	_sde_enc_phys_wb_detect_cwb(phys_enc, crtc_state);

	if (clone_mode_curr && !cstate->cwb_enc_mask) {
		SDE_ERROR("[enc:%d wb:%d] WB commit before CWB disable\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return -EINVAL;
	}

	memset(&wb_roi, 0, sizeof(struct sde_rect));

	rc = sde_wb_connector_state_get_output_roi(conn_state, &wb_roi);
	if (rc) {
		SDE_ERROR("[enc:%d wb:%d] failed to get roi; ret:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), rc);
		return rc;
	}

	/* bypass check if commit with no framebuffer */
	fb = sde_wb_connector_state_get_output_fb(conn_state);
	if (!fb) {
		SDE_ERROR("[enc:%d wb:%d] no out fb\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return -EINVAL;
	}

	fmt = sde_get_sde_format_ext(fb->format->format, fb->modifier);
	if (!fmt) {
		SDE_ERROR("[enc:%d wb:%d] unsupported output pixel format:%x\n",
				DRMID(phys_enc->parent), WBID(wb_enc), fb->format->format);
		return -EINVAL;
	}

	SDE_DEBUG("[enc:%d enc:%d] fb_id:%u, wxh:%ux%u, fb_fmt:%x,%llx, roi:{%d,%d,%d,%d}\n",
		DRMID(phys_enc->parent), WBID(wb_enc), fb->base.id, fb->width, fb->height,
		fb->format->format, fb->modifier, wb_roi.x, wb_roi.y, wb_roi.w, wb_roi.h);

	if (fmt->chroma_sample == SDE_CHROMA_H2V1 ||
		fmt->chroma_sample == SDE_CHROMA_H1V2) {
		SDE_ERROR("[enc:%d wb:%d] invalid chroma sample type in output format:%x\n",
			DRMID(phys_enc->parent), WBID(wb_enc), fmt->base.pixel_format);
		return -EINVAL;
	}

	if (SDE_FORMAT_IS_UBWC(fmt) && !(wb_cfg->features & BIT(SDE_WB_UBWC))) {
		SDE_ERROR("[enc:%d wb:%d] invalid output format:%x\n",
				DRMID(phys_enc->parent), WBID(wb_enc), fmt->base.pixel_format);
		return -EINVAL;
	}

	if (SDE_FORMAT_IS_YUV(fmt) != !!phys_enc->hw_cdm)
		crtc_state->mode_changed = true;

	rc = _sde_enc_phys_wb_validate_dnsc_blur_ds(crtc_state, conn_state, fmt, &wb_roi);
	if (rc) {
		SDE_ERROR("[enc:%d wb:%d] failed dnsc_blur/ds validation; ret:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), rc);
		return rc;
	}

	/* if in clone mode, return after cwb validation */
	if (cstate->cwb_enc_mask) {
		rc = _sde_enc_phys_wb_validate_cwb(phys_enc, crtc_state, conn_state);
		if (rc)
			SDE_ERROR("[enc:%d wb:%d] failed in cwb validation %d\n",
					DRMID(phys_enc->parent), WBID(wb_enc), rc);

		return rc;
	}

	_sde_enc_phys_wb_get_out_resolution(crtc_state, conn_state, &out_width, &out_height);
	if (!wb_roi.w || !wb_roi.h) {
		wb_roi.x = 0;
		wb_roi.y = 0;
		wb_roi.w = out_width;
		wb_roi.h = out_height;
	}

	if ((wb_roi.x + wb_roi.w > fb->width) || (wb_roi.x + wb_roi.w > out_width)) {
		SDE_ERROR("[enc:%d wb:%d] invalid roi x:%d, w:%d, fb_w:%d, mode_w:%d, out_w:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), wb_roi.x, wb_roi.w,
				fb->width, mode->hdisplay, out_width);
		return -EINVAL;
	} else if ((wb_roi.y + wb_roi.h > fb->height) || (wb_roi.y + wb_roi.h > out_height)) {
		SDE_ERROR("[enc:%d wb:%d] invalid roi y:%d, h:%d, fb_h:%d, mode_h%d, out_h:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), wb_roi.y, wb_roi.h,
				fb->height, mode->vdisplay, out_height);
		return -EINVAL;
	} else if ((out_width > mode->hdisplay) || (out_height > mode->vdisplay)) {
		SDE_ERROR("[enc:%d wb:%d] invalid o w/h o_w:%d, mode_w:%d, o_h:%d, mode_h:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), out_width, mode->hdisplay,
				out_height, mode->vdisplay);
		return -EINVAL;
	} else if (wb_roi.w > SDE_WB_MAX_LINEWIDTH(fmt, wb_cfg)) {
		SDE_ERROR("[enc:%d wb:%d] invalid roi ubwc:%d, w:%d, maxlinewidth:%u\n",
				DRMID(phys_enc->parent), WBID(wb_enc), SDE_FORMAT_IS_UBWC(fmt),
				wb_roi.w, SDE_WB_MAX_LINEWIDTH(fmt, wb_cfg));
		return -EINVAL;
	}

	return rc;
}

static void _sde_encoder_phys_wb_setup_sys_cache(struct sde_encoder_phys *phys_enc,
		struct drm_framebuffer *fb)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_wb_device *wb_dev = wb_enc->wb_dev;
	struct drm_connector_state *state = wb_dev->connector->state;
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	struct sde_crtc *sde_crtc = to_sde_crtc(wb_enc->crtc);
	struct sde_sc_cfg *sc_cfg;
	struct sde_hw_wb_sc_cfg *cfg  = &wb_enc->sc_cfg;
	u32 cache_enable, cache_flag, cache_rd_type, cache_wr_type;
	int i;

	if (!fb) {
		SDE_ERROR("invalid fb on wb %d\n", WBID(wb_enc));
		return;
	}

	if (!hw_wb || !hw_wb->ops.setup_sys_cache) {
		SDE_DEBUG("unsupported ops: setup_sys_cache WB %d\n", WBID(wb_enc));
		return;
	}

	/*
	 * - use LLCC_DISP/LLCC_DISP_1 for cwb static display
	 * - use LLCC_DISP_WB for 2-pass composition using offline-wb
	 */
	if (phys_enc->in_clone_mode) {
		/* toggle system cache SCID between consecutive CWB writes */
		if (test_bit(SDE_SYS_CACHE_DISP_1, hw_wb->catalog->sde_sys_cache_type_map)
				&& cfg->type == SDE_SYS_CACHE_DISP) {
			cache_wr_type = SDE_SYS_CACHE_DISP_1;
			cache_rd_type = SDE_SYS_CACHE_DISP_1;
		} else {
			cache_wr_type = SDE_SYS_CACHE_DISP;
			cache_rd_type = SDE_SYS_CACHE_DISP;
		}
	} else {
		cache_rd_type = SDE_SYS_CACHE_DISP_WB;
		cache_wr_type = SDE_SYS_CACHE_DISP_WB;
	}

	sc_cfg = &hw_wb->catalog->sc_cfg[cache_wr_type];
	if (!test_bit(cache_wr_type, hw_wb->catalog->sde_sys_cache_type_map)) {
		SDE_DEBUG("sys cache type %d not enabled\n", cache_wr_type);
		return;
	}

	cache_enable = sde_connector_get_property(state, CONNECTOR_PROP_CACHE_STATE);

	if (!cfg->wr_en && !cache_enable)
		return;

	cfg->wr_en = cache_enable;
	cfg->flags = SYS_CACHE_EN_FLAG | SYS_CACHE_SCID;

	if (cache_enable) {
		cfg->wr_scid = sc_cfg->llcc_scid;
		cfg->type = cache_wr_type;
		cache_flag = MSM_FB_CACHE_WRITE_EN;
	} else {
		cfg->wr_scid = 0x0;
		cfg->type = SDE_SYS_CACHE_NONE;
		cache_flag = MSM_FB_CACHE_NONE;
		cache_rd_type = SDE_SYS_CACHE_NONE;
		cache_wr_type = SDE_SYS_CACHE_NONE;
	}
	msm_framebuffer_set_cache_hint(fb, cache_flag, cache_rd_type, cache_wr_type);

	/*
	 * avoid llcc_active reset for crtc while in clone mode as it will reset it for
	 * primary display as well
	 */
	if (cache_enable) {
		sde_crtc->new_perf.llcc_active[cache_wr_type] = true;
		sde_crtc->new_perf.llcc_active[cache_rd_type] = true;
		sde_core_perf_crtc_update_llcc(wb_enc->crtc);
	} else if (!phys_enc->in_clone_mode) {
		for (i = 0; i < SDE_SYS_CACHE_MAX; i++)
			sde_crtc->new_perf.llcc_active[i] = false;
		sde_core_perf_crtc_update_llcc(wb_enc->crtc);
	}

	hw_wb->ops.setup_sys_cache(hw_wb, cfg);
	SDE_EVT32(WBID(wb_enc), cfg->wr_scid, cfg->flags, cfg->type, cache_enable,
			phys_enc->in_clone_mode, cache_flag, cache_rd_type,
			cache_wr_type, fb->base.id);
}

static void _sde_encoder_phys_wb_update_cwb_flush_helper(
		struct sde_encoder_phys *phys_enc, bool enable)
{
	struct sde_connector *c_conn = NULL;
	struct sde_connector_state *c_state = NULL;
	struct sde_hw_wb *hw_wb;
	struct sde_hw_ctl *hw_ctl;
	struct sde_hw_pingpong *hw_pp;
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_crtc_state *crtc_state;
	struct sde_crtc *crtc;
	int i = 0;
	int cwb_capture_mode = 0;
	bool need_merge = false;
	bool dspp_out = false;
	enum sde_cwb cwb_idx = 0;
	enum sde_cwb src_pp_idx = 0;
	enum sde_dcwb dcwb_idx = 0;
	size_t dither_sz = 0;
	void *dither_cfg = NULL;

	/* In CWB mode, program actual source master sde_hw_ctl from crtc */
	crtc = to_sde_crtc(wb_enc->crtc);
	hw_ctl = crtc->mixers[0].hw_ctl;
	hw_pp = phys_enc->hw_pp;
	hw_wb = wb_enc->hw_wb;
	if (!hw_ctl || !hw_wb || !hw_pp) {
		SDE_ERROR("[enc:%d wb:%d] HW resource not available for CWB\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	crtc_state = to_sde_crtc_state(wb_enc->crtc->state);
	cwb_capture_mode = sde_crtc_get_property(crtc_state, CRTC_PROP_CAPTURE_OUTPUT);
	need_merge = (crtc->num_mixers > 1) ? true : false;
	dspp_out = (cwb_capture_mode == CAPTURE_DSPP_OUT);
	cwb_idx = (enum sde_cwb)hw_pp->idx;
	src_pp_idx = (enum sde_cwb)crtc->mixers[0].hw_lm->idx;

	if (test_bit(SDE_WB_CWB_DITHER_CTRL, &hw_wb->caps->features)) {
		if (cwb_capture_mode) {
			c_conn = to_sde_connector(phys_enc->connector);
			c_state = to_sde_connector_state(phys_enc->connector->state);
			dither_cfg = msm_property_get_blob(&c_conn->property_info,
					&c_state->property_state, &dither_sz,
					CONNECTOR_PROP_PP_CWB_DITHER);
			SDE_DEBUG("Read cwb dither setting from blob %pK\n", dither_cfg);
		} else {
			/* disable case: tap is lm */
			dither_cfg = NULL;
		}
	}

	for (i = 0; i < crtc->num_mixers; i++) {
		src_pp_idx = (enum sde_cwb) (src_pp_idx + i);

		if (test_bit(SDE_WB_DCWB_CTRL, &hw_wb->caps->features)) {
			dcwb_idx = (enum sde_dcwb) ((hw_pp->idx % 2) + i);
			if ((test_bit(SDE_WB_CWB_DITHER_CTRL, &hw_wb->caps->features)) &&
				hw_wb->ops.program_cwb_dither_ctrl){
				hw_wb->ops.program_cwb_dither_ctrl(hw_wb,
					dcwb_idx, dither_cfg, dither_sz, enable);
			}
			if (hw_wb->ops.program_dcwb_ctrl)
				hw_wb->ops.program_dcwb_ctrl(hw_wb, dcwb_idx,
					src_pp_idx, cwb_capture_mode, enable);
			if (hw_ctl->ops.update_bitmask)
				hw_ctl->ops.update_bitmask(hw_ctl,
					SDE_HW_FLUSH_CWB, dcwb_idx, 1);

		} else if (test_bit(SDE_WB_CWB_CTRL, &hw_wb->caps->features)) {
			cwb_idx = (enum sde_cwb) (hw_pp->idx + i);
			if (hw_wb->ops.program_cwb_ctrl)
				hw_wb->ops.program_cwb_ctrl(hw_wb, cwb_idx,
					src_pp_idx, dspp_out, enable);
			if (hw_ctl->ops.update_bitmask)
				hw_ctl->ops.update_bitmask(hw_ctl,
					SDE_HW_FLUSH_CWB, cwb_idx, 1);
		}
	}

	if (need_merge && hw_ctl->ops.update_bitmask && hw_pp && hw_pp->merge_3d)
		hw_ctl->ops.update_bitmask(hw_ctl, SDE_HW_FLUSH_MERGE_3D,
				hw_pp->merge_3d->idx, 1);
}

static void _sde_encoder_phys_wb_update_cwb_flush(struct sde_encoder_phys *phys_enc, bool enable)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb;
	struct sde_hw_ctl *hw_ctl;
	struct sde_hw_cdm *hw_cdm;
	struct sde_hw_pingpong *hw_pp;
	struct sde_hw_dnsc_blur *hw_dnsc_blur;
	struct sde_crtc *crtc;
	struct sde_crtc_state *crtc_state;
	int cwb_capture_mode = 0;
	enum sde_cwb cwb_idx = 0;
	enum sde_dcwb dcwb_idx = 0;
	enum sde_cwb src_pp_idx = 0;
	bool dspp_out = false, need_merge = false;

	if (!phys_enc->in_clone_mode) {
		SDE_DEBUG("enc:%d, wb:%d - not in CWB mode. early return\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	crtc = to_sde_crtc(wb_enc->crtc);
	crtc_state = to_sde_crtc_state(wb_enc->crtc->state);
	cwb_capture_mode = sde_crtc_get_property(crtc_state,
			CRTC_PROP_CAPTURE_OUTPUT);

	hw_pp = phys_enc->hw_pp;
	hw_wb = wb_enc->hw_wb;
	hw_cdm = phys_enc->hw_cdm;
	hw_dnsc_blur = phys_enc->hw_dnsc_blur;

	/* In CWB mode, program actual source master sde_hw_ctl from crtc */
	hw_ctl = crtc->mixers[0].hw_ctl;
	if (!hw_ctl || !hw_wb || !hw_pp) {
		SDE_ERROR("[enc:%d wb:%d] HW resource not available for CWB\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	/* treating LM idx of primary display ctl path as source ping-pong idx*/
	src_pp_idx = (enum sde_cwb)crtc->mixers[0].hw_lm->idx;
	cwb_idx = (enum sde_cwb)hw_pp->idx;
	dspp_out = (cwb_capture_mode == CAPTURE_DSPP_OUT);
	need_merge = (crtc->num_mixers > 1) ? true : false;

	if (test_bit(SDE_WB_DCWB_CTRL, &hw_wb->caps->features)) {
		dcwb_idx = hw_pp->dcwb_idx;
		if ((dcwb_idx + crtc->num_mixers) > DCWB_MAX) {
			SDE_ERROR("[enc:%d, wb:%d] invalid DCWB config; dcwb=%d, num_lm=%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), dcwb_idx, crtc->num_mixers);
			return;
		}
	} else {
		if (src_pp_idx > CWB_0 ||  ((cwb_idx + crtc->num_mixers) > CWB_MAX)) {
			SDE_ERROR("[enc:%d wb:%d] invalid CWB onfig; pp_idx:%d, cwb:%d, num_lm%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), src_pp_idx,
				dcwb_idx, crtc->num_mixers);
			return;
		}
	}

	if (hw_ctl->ops.update_bitmask)
		hw_ctl->ops.update_bitmask(hw_ctl, SDE_HW_FLUSH_WB, hw_wb->idx, 1);

	if (hw_ctl->ops.update_bitmask && hw_cdm)
		hw_ctl->ops.update_bitmask(hw_ctl, SDE_HW_FLUSH_CDM, hw_cdm->idx, 1);

	if (hw_ctl->ops.update_dnsc_blur_bitmask && hw_dnsc_blur)
		hw_ctl->ops.update_dnsc_blur_bitmask(hw_ctl, hw_dnsc_blur->idx, 1);

	if (test_bit(SDE_WB_CWB_CTRL, &hw_wb->caps->features) ||
			test_bit(SDE_WB_DCWB_CTRL, &hw_wb->caps->features)) {
		_sde_encoder_phys_wb_update_cwb_flush_helper(phys_enc, enable);
	} else {
		phys_enc->hw_mdptop->ops.set_cwb_ppb_cntl(phys_enc->hw_mdptop,
				need_merge, dspp_out);
	}
}

/**
 * _sde_encoder_phys_wb_update_flush - flush hardware update
 * @phys_enc:	Pointer to physical encoder
 */
static void _sde_encoder_phys_wb_update_flush(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc;
	struct sde_hw_wb *hw_wb;
	struct sde_hw_ctl *hw_ctl;
	struct sde_hw_cdm *hw_cdm;
	struct sde_hw_pingpong *hw_pp;
	struct sde_hw_dnsc_blur *hw_dnsc_blur;
	struct sde_ctl_flush_cfg pending_flush = {0,};

	if (!phys_enc)
		return;

	wb_enc = to_sde_encoder_phys_wb(phys_enc);
	hw_wb = wb_enc->hw_wb;
	hw_cdm = phys_enc->hw_cdm;
	hw_pp = phys_enc->hw_pp;
	hw_ctl = phys_enc->hw_ctl;
	hw_dnsc_blur = phys_enc->hw_dnsc_blur;

	SDE_DEBUG("[enc:%d wb:%d]\n", DRMID(phys_enc->parent), WBID(wb_enc));

	if (phys_enc->in_clone_mode) {
		SDE_DEBUG("[enc:%d wb:%d] in CWB mode. early return\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	if (!hw_ctl) {
		SDE_DEBUG("[enc:%d wb:%d] invalid ctl\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	if (hw_ctl->ops.update_bitmask)
		hw_ctl->ops.update_bitmask(hw_ctl, SDE_HW_FLUSH_WB, hw_wb->idx, 1);

	if (hw_ctl->ops.update_bitmask && hw_cdm)
		hw_ctl->ops.update_bitmask(hw_ctl, SDE_HW_FLUSH_CDM, hw_cdm->idx, 1);

	if (hw_ctl->ops.update_bitmask && hw_pp && hw_pp->merge_3d)
		hw_ctl->ops.update_bitmask(hw_ctl, SDE_HW_FLUSH_MERGE_3D, hw_pp->merge_3d->idx, 1);

	if (hw_ctl->ops.update_dnsc_blur_bitmask && hw_dnsc_blur)
		hw_ctl->ops.update_dnsc_blur_bitmask(hw_ctl, hw_dnsc_blur->idx, 1);

	if (hw_ctl->ops.get_pending_flush)
		hw_ctl->ops.get_pending_flush(hw_ctl, &pending_flush);

	SDE_DEBUG("[enc:%d wb:%d] Pending flush mask for CTL_%d is 0x%x\n",
			DRMID(phys_enc->parent), WBID(wb_enc),
			hw_ctl->idx - CTL_0, pending_flush.pending_flush_mask);
}

static void _sde_encoder_phys_wb_setup_dnsc_blur(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_wb_device *wb_dev = wb_enc->wb_dev;
	struct sde_kms *sde_kms = phys_enc->sde_kms;
	struct sde_hw_dnsc_blur *hw_dnsc_blur = phys_enc->hw_dnsc_blur;
	struct sde_hw_pingpong *hw_pp = phys_enc->hw_pp;
	struct sde_connector *sde_conn;
	struct sde_connector_state *sde_conn_state;
	struct sde_drm_dnsc_blur_cfg *cfg;
	int i;
	bool enable;

	if (!sde_kms->catalog->dnsc_blur_count || !hw_pp)
		return;

	sde_conn = to_sde_connector(wb_dev->connector);
	sde_conn_state = to_sde_connector_state(wb_dev->connector->state);

	if (sde_conn_state->dnsc_blur_count
			&& (!hw_dnsc_blur || !hw_dnsc_blur->ops.setup_dnsc_blur)) {
		SDE_ERROR("[enc:%d wb:%d] invalid config - dnsc_blur block not reserved\n",
			DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	/* swap between 0 & 1 lut idx on each config change for gaussian lut */
	sde_conn_state->dnsc_blur_lut = 1 - sde_conn_state->dnsc_blur_lut;

	/*
	 * disable dnsc_blur case - safe to update the opmode as dynamic switching of
	 * dnsc_blur hw block between WBs are not supported currently.
	 */
	if (hw_dnsc_blur && !sde_conn_state->dnsc_blur_count) {
		hw_dnsc_blur->ops.setup_dnsc_blur(hw_dnsc_blur, NULL, 0);
		SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), SDE_EVTLOG_FUNC_CASE1);
		return;
	}

	for (i = 0; i < sde_conn_state->dnsc_blur_count; i++) {
		cfg = &sde_conn_state->dnsc_blur_cfg[i];

		enable = (cfg->flags & DNSC_BLUR_EN);
		hw_dnsc_blur->ops.setup_dnsc_blur(hw_dnsc_blur, cfg, sde_conn_state->dnsc_blur_lut);

		if (hw_dnsc_blur->ops.setup_dither)
			hw_dnsc_blur->ops.setup_dither(hw_dnsc_blur, cfg);

		if (hw_dnsc_blur->ops.bind_pingpong_blk)
			hw_dnsc_blur->ops.bind_pingpong_blk(hw_dnsc_blur, enable, hw_pp->idx,
					phys_enc->in_clone_mode);

		SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), sde_conn_state->dnsc_blur_count,
				cfg->flags, cfg->flags_h, cfg->flags_v, cfg->src_width,
				cfg->src_height, cfg->dst_width, cfg->dst_height,
				sde_conn_state->dnsc_blur_lut);
	}
}

static void _sde_encoder_phys_wb_setup_prog_line(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_wb_device *wb_dev = wb_enc->wb_dev;
	struct drm_connector_state *state = wb_dev->connector->state;
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	u32 prog_line;

	if (phys_enc->in_clone_mode || !hw_wb->ops.set_prog_line_count)
		return;

	prog_line = sde_connector_get_property(state, CONNECTOR_PROP_EARLY_FENCE_LINE);
	if (wb_enc->prog_line != prog_line) {
		SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), wb_enc->prog_line, prog_line);
		wb_enc->prog_line = prog_line;
		hw_wb->ops.set_prog_line_count(hw_wb, prog_line);
	}
}

/**
 * sde_encoder_phys_wb_setup - setup writeback encoder
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_setup(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct drm_display_mode mode = phys_enc->cached_mode;
	struct drm_connector_state *conn_state = phys_enc->connector->state;
	struct drm_crtc_state *crtc_state = wb_enc->crtc->state;
	struct drm_framebuffer *fb;
	struct sde_rect *wb_roi = &wb_enc->wb_roi;
	u32 out_width = 0, out_height = 0;

	SDE_DEBUG("[enc:%d wb:%d] mode_set:\"%s\",%d,%d]\n", DRMID(phys_enc->parent),
			WBID(wb_enc), mode.name, mode.hdisplay, mode.vdisplay);

	memset(wb_roi, 0, sizeof(struct sde_rect));

	/* clear writeback framebuffer - will be updated in setup_fb */
	wb_enc->wb_fb = NULL;
	wb_enc->wb_aspace = NULL;

	if (phys_enc->enable_state == SDE_ENC_DISABLING) {
		fb = wb_enc->fb_disable;
		wb_roi->w = 0;
		wb_roi->h = 0;
	} else {
		fb = sde_wb_get_output_fb(wb_enc->wb_dev);
		sde_wb_get_output_roi(wb_enc->wb_dev, wb_roi);
	}

	if (!fb) {
		SDE_DEBUG("[enc:%d wb:%d] no out fb\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	SDE_DEBUG("[fb_id:%u][fb:%u,%u]\n", fb->base.id, fb->width, fb->height);

	_sde_enc_phys_wb_get_out_resolution(crtc_state, conn_state, &out_width, &out_height);
	if (wb_roi->w == 0 || wb_roi->h == 0) {
		wb_roi->x = 0;
		wb_roi->y = 0;
		wb_roi->w = out_width;
		wb_roi->h = out_height;
	}

	wb_enc->wb_fmt = sde_get_sde_format_ext(fb->format->format,
							fb->modifier);
	if (!wb_enc->wb_fmt) {
		SDE_ERROR("[enc:%d wb:%d] unsupported output pixel format:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), fb->format->format);
		return;
	}

	SDE_DEBUG("[enc:%d enc:%d] fb_id:%u, wxh:%ux%u, fb_fmt:%x,%llx, roi:{%d,%d,%d,%d}\n",
		DRMID(phys_enc->parent), WBID(wb_enc), fb->base.id, fb->width, fb->height,
		fb->format->format, fb->modifier, wb_roi->x, wb_roi->y, wb_roi->w, wb_roi->h);

	SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), wb_roi->x, wb_roi->y, wb_roi->w, wb_roi->h,
			out_width, out_height, fb->width, fb->height, mode.hdisplay, mode.vdisplay);

	sde_encoder_phys_wb_set_ot_limit(phys_enc);

	sde_encoder_phys_wb_set_qos_remap(phys_enc);

	sde_encoder_phys_wb_set_qos(phys_enc);

	sde_encoder_phys_setup_cdm(phys_enc, fb, wb_enc->wb_fmt, wb_roi);

	sde_encoder_phys_wb_setup_fb(phys_enc, fb, wb_roi, out_width, out_height);

	_sde_encoder_phys_wb_setup_ctl(phys_enc, wb_enc->wb_fmt);

	_sde_encoder_phys_wb_setup_sys_cache(phys_enc, fb);

	_sde_encoder_phys_wb_setup_cwb(phys_enc, true);

	_sde_encoder_phys_wb_setup_prog_line(phys_enc);

	_sde_encoder_phys_wb_setup_dnsc_blur(phys_enc);
}

static void sde_encoder_phys_wb_ctl_start_irq(void *arg, int irq_idx)
{
	struct sde_encoder_phys_wb *wb_enc = arg;
	struct sde_encoder_phys *phys_enc;
	struct sde_hw_wb *hw_wb;
	u32 line_cnt = 0;

	if (!wb_enc)
		return;

	SDE_ATRACE_BEGIN("ctl_start_irq");
	phys_enc = &wb_enc->base;
	if (atomic_add_unless(&phys_enc->pending_ctl_start_cnt, -1, 0))
		wake_up_all(&phys_enc->pending_kickoff_wq);

	hw_wb = wb_enc->hw_wb;
	if (hw_wb->ops.get_line_count)
		line_cnt = hw_wb->ops.get_line_count(hw_wb);

	SDE_ATRACE_END("ctl_start_irq");
	SDE_EVT32_IRQ(DRMID(phys_enc->parent), WBID(wb_enc), line_cnt);
}

static void _sde_encoder_phys_wb_frame_done_helper(void *arg, bool frame_error)
{
	struct sde_encoder_phys_wb *wb_enc = arg;
	struct sde_encoder_phys *phys_enc = &wb_enc->base;
	u32 event = frame_error ? SDE_ENCODER_FRAME_EVENT_ERROR : 0;
	u32 ubwc_error = 0;

	/* don't notify upper layer for internal commit */
	if (phys_enc->enable_state == SDE_ENC_DISABLING && !phys_enc->in_clone_mode)
		goto end;

	if (phys_enc->parent_ops.handle_frame_done &&
			atomic_add_unless(&phys_enc->pending_kickoff_cnt, -1, 0)) {
		event |= SDE_ENCODER_FRAME_EVENT_DONE;

		/*
		 * signal retire-fence during wb-done
		 * - when prog_line is not configured
		 * - when prog_line is configured and line-ptr-irq is missed
		 */
		if (!wb_enc->prog_line || (wb_enc->prog_line &&
				(atomic_read(&phys_enc->pending_kickoff_cnt) <
					atomic_read(&phys_enc->pending_retire_fence_cnt)))) {
			atomic_add_unless(&phys_enc->pending_retire_fence_cnt, -1, 0);
			event |= SDE_ENCODER_FRAME_EVENT_SIGNAL_RETIRE_FENCE;
		}

		if (phys_enc->in_clone_mode)
			event |= SDE_ENCODER_FRAME_EVENT_CWB_DONE
					| SDE_ENCODER_FRAME_EVENT_SIGNAL_RETIRE_FENCE;
		else
			event |= SDE_ENCODER_FRAME_EVENT_SIGNAL_RELEASE_FENCE;

		phys_enc->parent_ops.handle_frame_done(phys_enc->parent, phys_enc, event);
	}

	if (!phys_enc->in_clone_mode && phys_enc->parent_ops.handle_vblank_virt)
		phys_enc->parent_ops.handle_vblank_virt(phys_enc->parent, phys_enc);

end:
	if (frame_error && wb_enc->hw_wb->ops.get_ubwc_error
			&& wb_enc->hw_wb->ops.clear_ubwc_error) {
		wb_enc->hw_wb->ops.get_ubwc_error(wb_enc->hw_wb);
		wb_enc->hw_wb->ops.clear_ubwc_error(wb_enc->hw_wb);
	}
	SDE_EVT32_IRQ(DRMID(phys_enc->parent), WBID(wb_enc), phys_enc->in_clone_mode,
			phys_enc->enable_state, event, atomic_read(&phys_enc->pending_kickoff_cnt),
			atomic_read(&phys_enc->pending_retire_fence_cnt),
			ubwc_error, frame_error);

	wake_up_all(&phys_enc->pending_kickoff_wq);
}

/**
 * sde_encoder_phys_wb_done_irq - Pingpong overflow interrupt handler for CWB
 * @arg:	Pointer to writeback encoder
 * @irq_idx:	interrupt index
 */
static void sde_encoder_phys_cwb_ovflow(void *arg, int irq_idx)
{
	_sde_encoder_phys_wb_frame_done_helper(arg, true);
}

/**
 * sde_encoder_phys_wb_done_irq - writeback interrupt handler
 * @arg:	Pointer to writeback encoder
 * @irq_idx:	interrupt index
 */
static void sde_encoder_phys_wb_done_irq(void *arg, int irq_idx)
{
	SDE_ATRACE_BEGIN("wb_done_irq");
	_sde_encoder_phys_wb_frame_done_helper(arg, false);
	SDE_ATRACE_END("wb_done_irq");
}

static void sde_encoder_phys_wb_lineptr_irq(void *arg, int irq_idx)
{
	struct sde_encoder_phys_wb *wb_enc = arg;
	struct sde_encoder_phys *phys_enc;
	struct sde_hw_wb *hw_wb;
	u32 event = 0, line_cnt = 0;

	if (!wb_enc || !wb_enc->prog_line)
		return;

	SDE_ATRACE_BEGIN("wb_lineptr_irq");
	phys_enc = &wb_enc->base;
	if (phys_enc->parent_ops.handle_frame_done &&
			atomic_add_unless(&phys_enc->pending_retire_fence_cnt, -1, 0)) {
		event = SDE_ENCODER_FRAME_EVENT_SIGNAL_RETIRE_FENCE;
		phys_enc->parent_ops.handle_frame_done(phys_enc->parent, phys_enc, event);
	}

	hw_wb = wb_enc->hw_wb;
	if (hw_wb->ops.get_line_count)
		line_cnt = hw_wb->ops.get_line_count(hw_wb);

	SDE_ATRACE_END("wb_lineptr_irq");
	SDE_EVT32_IRQ(DRMID(phys_enc->parent), WBID(wb_enc), event, wb_enc->prog_line, line_cnt);
}

/**
 * sde_encoder_phys_wb_irq_ctrl - irq control of WB
 * @phys:	Pointer to physical encoder
 * @enable:	indicates enable or disable interrupts
 */
static void sde_encoder_phys_wb_irq_ctrl(struct sde_encoder_phys *phys, bool enable)
{

	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys);
	const struct sde_wb_cfg *wb_cfg;
	int index = 0, pp = 0;
	u32 max_num_of_irqs = 0;
	const u32 *irq_table = NULL;

	if (!wb_enc)
		return;

	pp = phys->hw_pp->idx - PINGPONG_0;
	if ((pp + CRTC_DUAL_MIXERS_ONLY) >= PINGPONG_MAX) {
		SDE_ERROR("[enc:%d wb:%d] invalid pp:%d\n", DRMID(phys->parent), WBID(wb_enc), pp);
		return;
	}

	/*
	 * For Dedicated CWB, only one overflow IRQ is used for
	 * both the PP_CWB blks. Make sure only one IRQ is registered
	 * when D-CWB is enabled.
	 */
	wb_cfg = wb_enc->hw_wb->caps;
	if (wb_cfg->features & BIT(SDE_WB_HAS_DCWB)) {
		max_num_of_irqs = 1;
		irq_table = dcwb_irq_tbl;
	} else {
		max_num_of_irqs = CRTC_DUAL_MIXERS_ONLY;
		irq_table = cwb_irq_tbl;
	}

	if (enable && atomic_inc_return(&phys->wbirq_refcount) == 1) {
		sde_encoder_helper_register_irq(phys, INTR_IDX_WB_DONE);
		sde_encoder_helper_register_irq(phys, INTR_IDX_CTL_START);

		if (test_bit(SDE_WB_PROG_LINE, &wb_cfg->features))
			sde_encoder_helper_register_irq(phys, INTR_IDX_WB_LINEPTR);

		for (index = 0; index < max_num_of_irqs; index++)
			if (irq_table[index + pp] != SDE_NONE)
				sde_encoder_helper_register_irq(phys, irq_table[index + pp]);
	} else if (!enable && atomic_dec_return(&phys->wbirq_refcount) == 0) {
		sde_encoder_helper_unregister_irq(phys, INTR_IDX_WB_DONE);
		sde_encoder_helper_unregister_irq(phys, INTR_IDX_CTL_START);

		if (test_bit(SDE_WB_PROG_LINE, &wb_cfg->features))
			sde_encoder_helper_unregister_irq(phys, INTR_IDX_WB_LINEPTR);

		for (index = 0; index < max_num_of_irqs; index++)
			if (irq_table[index + pp] != SDE_NONE)
				sde_encoder_helper_unregister_irq(phys, irq_table[index + pp]);
	}
}

/**
 * sde_encoder_phys_wb_mode_set - set display mode
 * @phys_enc:	Pointer to physical encoder
 * @mode:	Pointer to requested display mode
 * @adj_mode:	Pointer to adjusted display mode
 */
static void sde_encoder_phys_wb_mode_set(
		struct sde_encoder_phys *phys_enc,
		struct drm_display_mode *mode,
		struct drm_display_mode *adj_mode, bool *reinit_mixers)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_rm *rm = &phys_enc->sde_kms->rm;
	struct sde_rm_hw_iter iter;
	int i, instance;
	struct sde_encoder_irq *irq;

	phys_enc->cached_mode = *adj_mode;
	instance = phys_enc->split_role == ENC_ROLE_SLAVE ? 1 : 0;

	SDE_DEBUG("[enc:%d wb:%d] mode_set_cache:\"%s\",%d,%d\n", DRMID(phys_enc->parent),
			WBID(wb_enc), mode->name, mode->hdisplay, mode->vdisplay);

	phys_enc->hw_ctl = NULL;
	phys_enc->hw_cdm = NULL;
	phys_enc->hw_dnsc_blur = NULL;

	/* Retrieve previously allocated HW Resources. CTL shouldn't fail */
	sde_rm_init_hw_iter(&iter, phys_enc->parent->base.id, SDE_HW_BLK_CTL);
	for (i = 0; i <= instance; i++) {
		sde_rm_get_hw(rm, &iter);
		if (i == instance) {
			if (phys_enc->hw_ctl && phys_enc->hw_ctl != to_sde_hw_ctl(iter.hw)) {
				*reinit_mixers =  true;
				SDE_EVT32(phys_enc->hw_ctl->idx, to_sde_hw_ctl(iter.hw)->idx);
			}
			phys_enc->hw_ctl = to_sde_hw_ctl(iter.hw);
		}
	}

	if (IS_ERR_OR_NULL(phys_enc->hw_ctl)) {
		SDE_ERROR("[enc:%d, wb:%d] failed init ctl: %ld\n", DRMID(phys_enc->parent),
			WBID(wb_enc), (!phys_enc->hw_ctl) ? -EINVAL : PTR_ERR(phys_enc->hw_ctl));
		phys_enc->hw_ctl = NULL;
		return;
	}

	/* CDM is optional */
	sde_rm_init_hw_iter(&iter, phys_enc->parent->base.id, SDE_HW_BLK_CDM);
	for (i = 0; i <= instance; i++) {
		sde_rm_get_hw(rm, &iter);
		if (i == instance)
			phys_enc->hw_cdm = to_sde_hw_cdm(iter.hw);
	}

	if (IS_ERR(phys_enc->hw_cdm)) {
		SDE_ERROR("[enc:%d wb:%d] CDM required but not allocated:%ld\n",
			DRMID(phys_enc->parent), WBID(wb_enc), PTR_ERR(phys_enc->hw_cdm));
		phys_enc->hw_cdm = NULL;
	}

	/* Downscale Blur is optional */
	sde_rm_init_hw_iter(&iter, phys_enc->parent->base.id, SDE_HW_BLK_DNSC_BLUR);
	for (i = 0; i <= instance; i++) {
		sde_rm_get_hw(rm, &iter);
		if (i == instance)
			phys_enc->hw_dnsc_blur =  to_sde_hw_dnsc_blur(iter.hw);
	}

	if (IS_ERR(phys_enc->hw_dnsc_blur)) {
		SDE_ERROR("[enc:%d wb:%d] Downscale Blur required but not allocated:%ld\n",
			DRMID(phys_enc->parent), WBID(wb_enc), PTR_ERR(phys_enc->hw_dnsc_blur));
		phys_enc->hw_dnsc_blur = NULL;
	}

	phys_enc->kickoff_timeout_ms =
		sde_encoder_helper_get_kickoff_timeout_ms(phys_enc->parent);

	/* set ctl idx for ctl-start-irq */
	irq = &phys_enc->irq[INTR_IDX_CTL_START];
	irq->hw_idx = phys_enc->hw_ctl->idx;
}

static bool _sde_encoder_phys_wb_is_idle(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	struct sde_vbif_get_xin_status_params xin_status = {0};

	xin_status.vbif_idx = hw_wb->caps->vbif_idx;
	xin_status.xin_id = hw_wb->caps->xin_id;
	xin_status.clk_ctrl = hw_wb->caps->clk_ctrl;

	return sde_vbif_get_xin_status(phys_enc->sde_kms, &xin_status);
}

static void _sde_encoder_phys_wb_reset_state(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);

	phys_enc->enable_state = SDE_ENC_DISABLED;

	/* cleanup any pending buffer */
	if (wb_enc->wb_fb && wb_enc->wb_aspace) {
		msm_framebuffer_cleanup(wb_enc->wb_fb, wb_enc->wb_aspace);
		drm_framebuffer_put(wb_enc->wb_fb);
		wb_enc->wb_fb = NULL;
		wb_enc->wb_aspace = NULL;
	}

	wb_enc->crtc = NULL;
	phys_enc->hw_cdm = NULL;
	phys_enc->hw_ctl = NULL;
	phys_enc->in_clone_mode = false;
	atomic_set(&phys_enc->pending_kickoff_cnt, 0);
	atomic_set(&phys_enc->pending_retire_fence_cnt, 0);
	atomic_set(&phys_enc->pending_ctl_start_cnt, 0);
}

static int _sde_encoder_phys_wb_wait_for_idle(struct sde_encoder_phys *phys_enc, bool force_wait)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_encoder_wait_info wait_info = {0};
	int rc = 0;
	bool is_idle;

	/* Return EWOULDBLOCK since we know the wait isn't necessary */
	if (phys_enc->enable_state == SDE_ENC_DISABLED) {
		SDE_ERROR("enc:%d, wb:%d - encoder already disabled\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return -EWOULDBLOCK;
	}

	SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), phys_enc->in_clone_mode,
			atomic_read(&phys_enc->pending_kickoff_cnt), force_wait);

	if (!force_wait && phys_enc->in_clone_mode
			&& (atomic_read(&phys_enc->pending_kickoff_cnt) <= 1))
		return 0;

	/*
	 * signal completion if commit with no framebuffer
	 * handle frame-done when WB HW is idle
	 */
	is_idle = _sde_encoder_phys_wb_is_idle(phys_enc);
	if (!wb_enc->wb_fb || is_idle) {
		SDE_EVT32((phys_enc->parent), WBID(wb_enc), !wb_enc->wb_fb, is_idle);
		goto frame_done;
	}

	if (atomic_read(&phys_enc->pending_kickoff_cnt) > 1)
		wait_info.count_check = 1;

	wait_info.wq = &phys_enc->pending_kickoff_wq;
	wait_info.atomic_cnt = &phys_enc->pending_kickoff_cnt;
	wait_info.timeout_ms = max_t(u32, wb_enc->wbdone_timeout, phys_enc->kickoff_timeout_ms);

	rc = sde_encoder_helper_wait_for_irq(phys_enc, INTR_IDX_WB_DONE, &wait_info);
	if (rc == -ETIMEDOUT) {
		/* handle frame-done when WB HW is idle */
		if (_sde_encoder_phys_wb_is_idle(phys_enc))
			rc = 0;

		SDE_ERROR("caller:%pS [enc:%d, wb:%d] clone_mode:%d kickoff timed out\n",
			__builtin_return_address(0), DRMID(phys_enc->parent), WBID(wb_enc),
			phys_enc->in_clone_mode);
		SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc),
			atomic_read(&phys_enc->pending_kickoff_cnt), SDE_EVTLOG_ERROR);
		goto frame_done;
	}

	return 0;

frame_done:
	_sde_encoder_phys_wb_frame_done_helper(wb_enc, rc ? true : false);
	return rc;
}

static int _sde_encoder_phys_wb_wait_for_ctl_start(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_encoder_wait_info wait_info = {0};
	int rc = 0;

	if (!atomic_read(&phys_enc->pending_ctl_start_cnt))
		return 0;

	SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), phys_enc->in_clone_mode,
			atomic_read(&phys_enc->pending_kickoff_cnt),
			atomic_read(&phys_enc->pending_retire_fence_cnt),
			atomic_read(&phys_enc->pending_ctl_start_cnt));

	wait_info.wq = &phys_enc->pending_kickoff_wq;
	wait_info.atomic_cnt = &phys_enc->pending_ctl_start_cnt;
	wait_info.timeout_ms = max_t(u32, wb_enc->wbdone_timeout, phys_enc->kickoff_timeout_ms);

	rc = sde_encoder_helper_wait_for_irq(phys_enc, INTR_IDX_CTL_START, &wait_info);
	if (rc == -ETIMEDOUT) {
		atomic_add_unless(&phys_enc->pending_ctl_start_cnt, -1, 0);
		SDE_ERROR("[enc:%d wb:%d] ctl_start timed out\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), SDE_EVTLOG_ERROR);
	}

	return rc;
}

/**
 * sde_encoder_phys_wb_wait_for_commit_done - wait until request is committed
 * @phys_enc:	Pointer to physical encoder
 */
static int sde_encoder_phys_wb_wait_for_commit_done(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	int rc, pending_cnt, i;
	bool is_idle;

	/* CWB - wait for previous frame completion */
	if (phys_enc->in_clone_mode) {
		rc = _sde_encoder_phys_wb_wait_for_idle(phys_enc, false);
		goto end;
	}

	/*
	 * WB - wait for ctl-start-irq by default and additionally for
	 * wb-done-irq during timeout or serialize frame-trigger
	 */
	rc = _sde_encoder_phys_wb_wait_for_ctl_start(phys_enc);

	pending_cnt = atomic_read(&phys_enc->pending_kickoff_cnt);
	is_idle = _sde_encoder_phys_wb_is_idle(phys_enc);

	if (rc || (pending_cnt > 1) || (pending_cnt && is_idle)
			|| (!rc && (phys_enc->frame_trigger_mode == FRAME_DONE_WAIT_SERIALIZE))) {
		for (i = 0; i < pending_cnt; i++)
			rc |= _sde_encoder_phys_wb_wait_for_idle(phys_enc, true);

		if (rc) {
			SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc),
					phys_enc->frame_trigger_mode,
					atomic_read(&phys_enc->pending_kickoff_cnt), is_idle, rc);
			SDE_ERROR("[enc:%d, wb:%d] failed wait_for_idle; ret:%d\n",
					DRMID(phys_enc->parent), WBID(wb_enc), rc);
		}
	}

end:
	/* cleanup any pending previous buffer */
	if (wb_enc->old_fb && wb_enc->old_aspace) {
		msm_framebuffer_cleanup(wb_enc->old_fb, wb_enc->old_aspace);
		drm_framebuffer_put(wb_enc->old_fb);
		wb_enc->old_fb = NULL;
		wb_enc->old_aspace = NULL;
	}

	return rc;
}

static int sde_encoder_phys_wb_wait_for_tx_complete(struct sde_encoder_phys *phys_enc)
{
	int rc = 0;

	if (atomic_read(&phys_enc->pending_kickoff_cnt))
		rc = _sde_encoder_phys_wb_wait_for_idle(phys_enc, true);

	if ((phys_enc->enable_state == SDE_ENC_DISABLING) && phys_enc->in_clone_mode) {
		_sde_encoder_phys_wb_reset_state(phys_enc);
		sde_encoder_phys_wb_irq_ctrl(phys_enc, false);
	}

	return rc;
}

/**
 * sde_encoder_phys_wb_prepare_for_kickoff - pre-kickoff processing
 * @phys_enc:	Pointer to physical encoder
 * @params:	kickoff parameters
 * Returns:	Zero on success
 */
static int sde_encoder_phys_wb_prepare_for_kickoff(struct sde_encoder_phys *phys_enc,
		struct sde_encoder_kickoff_params *params)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	int ret = 0;

	phys_enc->frame_trigger_mode = params ?
		params->frame_trigger_mode : FRAME_DONE_WAIT_DEFAULT;
	if (!phys_enc->in_clone_mode && (phys_enc->frame_trigger_mode == FRAME_DONE_WAIT_DEFAULT)
			&& (atomic_read(&phys_enc->pending_kickoff_cnt))) {
		ret = _sde_encoder_phys_wb_wait_for_idle(phys_enc, true);
		if (ret)
			atomic_set(&phys_enc->pending_kickoff_cnt, 0);
	}

	/* cache the framebuffer/aspace for cleanup later */
	wb_enc->old_fb = wb_enc->wb_fb;
	wb_enc->old_aspace = wb_enc->wb_aspace;

	/* set OT limit & enable traffic shaper */
	sde_encoder_phys_wb_setup(phys_enc);

	_sde_encoder_phys_wb_update_flush(phys_enc);

	_sde_encoder_phys_wb_update_cwb_flush(phys_enc, true);

	SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), phys_enc->in_clone_mode,
			phys_enc->frame_trigger_mode, ret);
	return ret;
}

/**
 * sde_encoder_phys_wb_trigger_flush - trigger flush processing
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_trigger_flush(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);

	if (!phys_enc || !wb_enc->hw_wb) {
		SDE_ERROR("invalid encoder\n");
		return;
	}

	/*
	 * Bail out iff in CWB mode. In case of CWB, primary control-path
	 * which is actually driving would trigger the flush
	 */
	if (phys_enc->in_clone_mode) {
		SDE_DEBUG("[enc:%d wb:%d] in CWB mode. early return\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	SDE_DEBUG("[enc:%d wb:%d]\n", DRMID(phys_enc->parent), WBID(wb_enc));

	/* clear pending flush if commit with no framebuffer */
	if (!wb_enc->wb_fb) {
		SDE_DEBUG("[enc:%d wb:%d] no out FB\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	sde_encoder_helper_trigger_flush(phys_enc);
}

/**
 * _sde_encoder_phys_wb_init_internal_fb - create fb for internal commit
 * @wb_enc:		Pointer to writeback encoder
 * @pixel_format:	DRM pixel format
 * @width:		Desired fb width
 * @height:		Desired fb height
 * @pitch:		Desired fb pitch
 */
static int _sde_encoder_phys_wb_init_internal_fb(struct sde_encoder_phys_wb *wb_enc,
		uint32_t pixel_format, uint32_t width, uint32_t height, uint32_t pitch)
{
	struct drm_device *dev;
	struct drm_framebuffer *fb;
	struct drm_mode_fb_cmd2 mode_cmd;
	uint32_t size;
	int nplanes, i, ret;
	struct msm_gem_address_space *aspace;
	const struct drm_format_info *info;
	struct sde_encoder_phys *phys_enc;

	if (!wb_enc || !wb_enc->base.parent || !wb_enc->base.sde_kms) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}
	phys_enc = &wb_enc->base;

	aspace = wb_enc->base.sde_kms->aspace[SDE_IOMMU_DOMAIN_UNSECURE];
	if (!aspace) {
		SDE_ERROR("[enc:%d wb:%d] invalid aspace\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return -EINVAL;
	}

	dev = wb_enc->base.sde_kms->dev;
	if (!dev) {
		SDE_ERROR("[enc:%d wb:%d] invalid dev\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return -EINVAL;
	}

	memset(&mode_cmd, 0, sizeof(mode_cmd));
	mode_cmd.pixel_format = pixel_format;
	mode_cmd.width = width;
	mode_cmd.height = height;
	mode_cmd.pitches[0] = pitch;

	size = sde_format_get_framebuffer_size(pixel_format, mode_cmd.width, mode_cmd.height,
			mode_cmd.pitches, 0);
	if (!size) {
		SDE_DEBUG("[enc:%d wb:%d] invalid fbsize\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return -EINVAL;
	}

	/* allocate gem tracking object */
	info = drm_get_format_info(dev, &mode_cmd);
	nplanes = info->num_planes;
	if (nplanes >= SDE_MAX_PLANES) {
		SDE_ERROR("[enc:%d wb:%d] requested format has too many planes:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), nplanes);
		return -EINVAL;
	}

	wb_enc->bo_disable[0] = msm_gem_new(dev, size, MSM_BO_SCANOUT | MSM_BO_WC);
	if (IS_ERR_OR_NULL(wb_enc->bo_disable[0])) {
		ret = PTR_ERR(wb_enc->bo_disable[0]);
		wb_enc->bo_disable[0] = NULL;

		SDE_ERROR("[enc:%d wb:%d] failed to create bo; ret:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), ret);
		return ret;
	}

	for (i = 0; i < nplanes; ++i) {
		wb_enc->bo_disable[i] = wb_enc->bo_disable[0];
		mode_cmd.pitches[i] = width * info->cpp[i];
	}

	fb = msm_framebuffer_init(dev, &mode_cmd, wb_enc->bo_disable);
	if (IS_ERR_OR_NULL(fb)) {
		ret = PTR_ERR(fb);
		drm_gem_object_put(wb_enc->bo_disable[0]);
		wb_enc->bo_disable[0] = NULL;

		SDE_ERROR("[enc:%d wb:%d] failed to init fb; ret:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), ret);
		return ret;
	}

	/* prepare the backing buffer now so that it's available later */
	ret = msm_framebuffer_prepare(fb, aspace);
	if (!ret)
		wb_enc->fb_disable = fb;
	return ret;
}

/**
 * _sde_encoder_phys_wb_destroy_internal_fb - deconstruct internal fb
 * @wb_enc:		Pointer to writeback encoder
 */
static void _sde_encoder_phys_wb_destroy_internal_fb(
		struct sde_encoder_phys_wb *wb_enc)
{
	if (!wb_enc)
		return;

	if (wb_enc->fb_disable) {
		drm_framebuffer_unregister_private(wb_enc->fb_disable);
		drm_framebuffer_remove(wb_enc->fb_disable);
		wb_enc->fb_disable = NULL;
	}

	if (wb_enc->bo_disable[0]) {
		drm_gem_object_put(wb_enc->bo_disable[0]);
		wb_enc->bo_disable[0] = NULL;
	}
}

/**
 * sde_encoder_phys_wb_enable - enable writeback encoder
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_enable(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct drm_device *dev;
	struct drm_connector *connector;

	SDE_DEBUG("[enc:%d wb:%d]\n", DRMID(phys_enc->parent), WBID(wb_enc));

	if (!wb_enc->base.parent || !wb_enc->base.parent->dev) {
		SDE_ERROR("[enc:%d, wb:%d] invalid dev\n", DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}
	dev = wb_enc->base.parent->dev;

	/* find associated writeback connector */
	connector = phys_enc->connector;

	if (!connector || connector->encoder != phys_enc->parent) {
		SDE_ERROR("[enc:%d, wb:%d] failed to find writeback connector\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}
	wb_enc->wb_dev = sde_wb_connector_get_wb(connector);

	phys_enc->enable_state = SDE_ENC_ENABLED;

	/*
	 * cache the crtc in wb_enc on enable for duration of use case
	 * for correctly servicing asynchronous irq events and timers
	 */
	wb_enc->crtc = phys_enc->parent->crtc;
}

/**
 * sde_encoder_phys_wb_disable - disable writeback encoder
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_disable(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);
	struct sde_hw_wb *hw_wb = wb_enc->hw_wb;
	struct sde_crtc *sde_crtc = to_sde_crtc(wb_enc->crtc);
	struct sde_hw_wb_sc_cfg cfg = { 0 };
	int i;

	if (phys_enc->enable_state == SDE_ENC_DISABLED) {
		SDE_ERROR("[enc:%d wb:%d] encoder is already disabled\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		return;
	}

	SDE_DEBUG("[enc:%d, wb:%d] clone_mode:%d, kickoff_cnt:%u\n",
			DRMID(phys_enc->parent), WBID(wb_enc), phys_enc->in_clone_mode,
			atomic_read(&phys_enc->pending_kickoff_cnt));

	if (!phys_enc->hw_ctl || !phys_enc->parent ||
			!phys_enc->sde_kms || !wb_enc->fb_disable) {
		SDE_DEBUG("[enc:%d wb:%d] invalid hw; skipping extra commit\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		goto exit;
	}

	/* reset system cache properties */
	if (wb_enc->sc_cfg.wr_en) {
		if (hw_wb->ops.setup_sys_cache)
			hw_wb->ops.setup_sys_cache(hw_wb, &cfg);

		/*
		 * avoid llcc_active reset for crtc while in clone mode as it will reset it for
		 * primary display as well
		 */
		if (!phys_enc->in_clone_mode) {
			for (i = 0; i < SDE_SYS_CACHE_MAX; i++)
				sde_crtc->new_perf.llcc_active[i] = 0;
			sde_core_perf_crtc_update_llcc(wb_enc->crtc);
		}
	}

	if (phys_enc->in_clone_mode) {
		_sde_encoder_phys_wb_setup_cwb(phys_enc, false);
		_sde_encoder_phys_wb_update_cwb_flush(phys_enc, false);
		phys_enc->enable_state = SDE_ENC_DISABLING;

		if (wb_enc->crtc->state->active) {
			sde_encoder_phys_wb_irq_ctrl(phys_enc, true);
			return;
		}

		if (phys_enc->connector)
			sde_connector_commit_reset(phys_enc->connector, ktime_get());
		goto exit;
	}

	/* reset h/w before final flush */
	if (phys_enc->hw_ctl->ops.clear_pending_flush)
		phys_enc->hw_ctl->ops.clear_pending_flush(phys_enc->hw_ctl);

	/*
	 * New CTL reset sequence from 5.0 MDP onwards.
	 * If has_3d_merge_reset is not set, legacy reset
	 * sequence is executed.
	 */
	if (test_bit(SDE_FEATURE_3D_MERGE_RESET, hw_wb->catalog->features)) {
		sde_encoder_helper_phys_disable(phys_enc, wb_enc);
		goto exit;
	}

	if (sde_encoder_helper_reset_mixers(phys_enc, NULL))
		goto exit;

	phys_enc->enable_state = SDE_ENC_DISABLING;

	sde_encoder_phys_wb_prepare_for_kickoff(phys_enc, NULL);
	sde_encoder_phys_wb_irq_ctrl(phys_enc, true);
	if (phys_enc->hw_ctl->ops.trigger_flush)
		phys_enc->hw_ctl->ops.trigger_flush(phys_enc->hw_ctl);

	sde_encoder_helper_trigger_start(phys_enc);
	_sde_encoder_phys_wb_wait_for_idle(phys_enc, true);
	sde_encoder_phys_wb_irq_ctrl(phys_enc, false);

exit:
	SDE_EVT32(DRMID(phys_enc->parent), WBID(wb_enc), phys_enc->in_clone_mode);
	_sde_encoder_phys_wb_reset_state(phys_enc);
}

/**
 * sde_encoder_phys_wb_get_hw_resources - get hardware resources
 * @phys_enc:	Pointer to physical encoder
 * @hw_res:	Pointer to encoder resources
 */
static void sde_encoder_phys_wb_get_hw_resources(struct sde_encoder_phys *phys_enc,
		struct sde_encoder_hw_resources *hw_res, struct drm_connector_state *conn_state)
{
	struct sde_encoder_phys_wb *wb_enc;
	struct sde_hw_wb *hw_wb;
	struct drm_framebuffer *fb;
	const struct sde_format *fmt = NULL;

	if (!phys_enc) {
		SDE_ERROR("invalid encoder\n");
		return;
	}
	wb_enc = to_sde_encoder_phys_wb(phys_enc);

	fb = sde_wb_connector_state_get_output_fb(conn_state);
	if (fb) {
		fmt = sde_get_sde_format_ext(fb->format->format, fb->modifier);
		if (!fmt) {
			SDE_ERROR("[enc:%d wb:%d] unsupported output pixel format:%d\n",
				DRMID(phys_enc->parent), WBID(wb_enc), fb->format->format);
			return;
		}
	}

	hw_wb = wb_enc->hw_wb;
	hw_res->wbs[hw_wb->idx - WB_0] = phys_enc->intf_mode;
	hw_res->needs_cdm = fmt ? SDE_FORMAT_IS_YUV(fmt) : false;
	SDE_DEBUG("[enc:%d wb:%d] intf_mode:%d needs_cdm:%d\n", DRMID(phys_enc->parent),
		WBID(wb_enc), hw_res->wbs[hw_wb->idx - WB_0], hw_res->needs_cdm);
}

#if IS_ENABLED(CONFIG_DEBUG_FS)
/**
 * sde_encoder_phys_wb_init_debugfs - initialize writeback encoder debugfs
 * @phys_enc:		Pointer to physical encoder
 * @debugfs_root:	Pointer to virtual encoder's debugfs_root dir
 */
static int sde_encoder_phys_wb_init_debugfs(
		struct sde_encoder_phys *phys_enc, struct dentry *debugfs_root)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);

	if (!phys_enc || !wb_enc->hw_wb || !debugfs_root)
		return -EINVAL;

	debugfs_create_u32("wbdone_timeout", 0600, debugfs_root, &wb_enc->wbdone_timeout);

	return 0;
}
#else
static int sde_encoder_phys_wb_init_debugfs(
		struct sde_encoder_phys *phys_enc, struct dentry *debugfs_root)
{
	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static int sde_encoder_phys_wb_late_register(struct sde_encoder_phys *phys_enc,
		struct dentry *debugfs_root)
{
	return sde_encoder_phys_wb_init_debugfs(phys_enc, debugfs_root);
}

/**
 * sde_encoder_phys_wb_destroy - destroy writeback encoder
 * @phys_enc:	Pointer to physical encoder
 */
static void sde_encoder_phys_wb_destroy(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc;

	if (!phys_enc)
		return;
	wb_enc = to_sde_encoder_phys_wb(phys_enc);

	SDE_DEBUG("[enc:%d wb:%d]\n", DRMID(phys_enc->parent), WBID(wb_enc));

	_sde_encoder_phys_wb_destroy_internal_fb(wb_enc);

	kfree(wb_enc);
}

void sde_encoder_phys_wb_add_enc_to_minidump(struct sde_encoder_phys *phys_enc)
{
	struct sde_encoder_phys_wb *wb_enc = to_sde_encoder_phys_wb(phys_enc);

	sde_mini_dump_add_va_region("sde_enc_phys_wb", sizeof(*wb_enc), wb_enc);
}

/**
 * sde_encoder_phys_wb_init_ops - initialize writeback operations
 * @ops:	Pointer to encoder operation table
 */
static void sde_encoder_phys_wb_init_ops(struct sde_encoder_phys_ops *ops)
{
	ops->late_register = sde_encoder_phys_wb_late_register;
	ops->is_master = sde_encoder_phys_wb_is_master;
	ops->mode_set = sde_encoder_phys_wb_mode_set;
	ops->enable = sde_encoder_phys_wb_enable;
	ops->disable = sde_encoder_phys_wb_disable;
	ops->destroy = sde_encoder_phys_wb_destroy;
	ops->atomic_check = sde_encoder_phys_wb_atomic_check;
	ops->get_hw_resources = sde_encoder_phys_wb_get_hw_resources;
	ops->wait_for_commit_done = sde_encoder_phys_wb_wait_for_commit_done;
	ops->wait_for_tx_complete = sde_encoder_phys_wb_wait_for_tx_complete;
	ops->prepare_for_kickoff = sde_encoder_phys_wb_prepare_for_kickoff;
	ops->trigger_flush = sde_encoder_phys_wb_trigger_flush;
	ops->trigger_start = sde_encoder_helper_trigger_start;
	ops->hw_reset = sde_encoder_helper_hw_reset;
	ops->irq_control = sde_encoder_phys_wb_irq_ctrl;
	ops->add_to_minidump = sde_encoder_phys_wb_add_enc_to_minidump;
}

/**
 * sde_encoder_phys_wb_init - initialize writeback encoder
 * @init:	Pointer to init info structure with initialization params
 */
struct sde_encoder_phys *sde_encoder_phys_wb_init(struct sde_enc_phys_init_params *p)
{
	struct sde_encoder_phys *phys_enc;
	struct sde_encoder_phys_wb *wb_enc;
	const struct sde_wb_cfg *wb_cfg;
	struct sde_hw_mdp *hw_mdp;
	struct sde_encoder_irq *irq;
	int ret = 0, i;

	SDE_DEBUG("\n");

	if (!p || !p->parent) {
		SDE_ERROR("invalid params\n");
		ret = -EINVAL;
		goto fail_alloc;
	}

	wb_enc = kzalloc(sizeof(*wb_enc), GFP_KERNEL);
	if (!wb_enc) {
		SDE_ERROR("failed to allocate wb enc\n");
		ret = -ENOMEM;
		goto fail_alloc;
	}

	phys_enc = &wb_enc->base;
	phys_enc->kickoff_timeout_ms = DEFAULT_KICKOFF_TIMEOUT_MS;

	if (p->sde_kms->vbif[VBIF_NRT]) {
		wb_enc->aspace[SDE_IOMMU_DOMAIN_UNSECURE] =
			p->sde_kms->aspace[MSM_SMMU_DOMAIN_NRT_UNSECURE];
		wb_enc->aspace[SDE_IOMMU_DOMAIN_SECURE] =
			p->sde_kms->aspace[MSM_SMMU_DOMAIN_NRT_SECURE];
	} else {
		wb_enc->aspace[SDE_IOMMU_DOMAIN_UNSECURE] =
			p->sde_kms->aspace[MSM_SMMU_DOMAIN_UNSECURE];
		wb_enc->aspace[SDE_IOMMU_DOMAIN_SECURE] =
			p->sde_kms->aspace[MSM_SMMU_DOMAIN_SECURE];
	}

	hw_mdp = sde_rm_get_mdp(&p->sde_kms->rm);
	if (IS_ERR_OR_NULL(hw_mdp)) {
		ret = PTR_ERR(hw_mdp);
		SDE_ERROR("failed to init hw_top: %d\n", ret);
		goto fail_mdp_init;
	}
	phys_enc->hw_mdptop = hw_mdp;

	/**
	 * hw_wb resource permanently assigned to this encoder
	 * Other resources allocated at atomic commit time by use case
	 */
	if (p->wb_idx != SDE_NONE) {
		struct sde_rm_hw_iter iter;

		sde_rm_init_hw_iter(&iter, 0, SDE_HW_BLK_WB);
		while (sde_rm_get_hw(&p->sde_kms->rm, &iter)) {
			struct sde_hw_wb *hw_wb = to_sde_hw_wb(iter.hw);

			if (hw_wb->idx == p->wb_idx) {
				wb_enc->hw_wb = hw_wb;
				break;
			}
		}

		if (!wb_enc->hw_wb) {
			ret = -EINVAL;
			SDE_ERROR("failed to init hw_wb%d\n", p->wb_idx - WB_0);
			goto fail_wb_init;
		}
	} else {
		ret = -EINVAL;
		SDE_ERROR("invalid wb_idx\n");
		goto fail_wb_check;
	}

	sde_encoder_phys_wb_init_ops(&phys_enc->ops);
	phys_enc->parent = p->parent;
	phys_enc->parent_ops = p->parent_ops;
	phys_enc->sde_kms = p->sde_kms;
	phys_enc->split_role = p->split_role;
	phys_enc->intf_mode = INTF_MODE_WB_LINE;
	phys_enc->intf_idx = p->intf_idx;
	phys_enc->enc_spinlock = p->enc_spinlock;
	atomic_set(&phys_enc->pending_retire_fence_cnt, 0);
	atomic_set(&phys_enc->pending_kickoff_cnt, 0);
	atomic_set(&phys_enc->pending_ctl_start_cnt, 0);
	init_waitqueue_head(&phys_enc->pending_kickoff_wq);
	wb_cfg = wb_enc->hw_wb->caps;

	for (i = 0; i < INTR_IDX_MAX; i++) {
		irq = &phys_enc->irq[i];
		INIT_LIST_HEAD(&irq->cb.list);
		irq->irq_idx = -EINVAL;
		irq->hw_idx = -EINVAL;
		irq->cb.arg = wb_enc;
	}

	irq = &phys_enc->irq[INTR_IDX_WB_DONE];
	irq->name = "wb_done";
	irq->hw_idx =  wb_enc->hw_wb->idx;
	irq->intr_type = sde_encoder_phys_wb_get_intr_type(wb_enc->hw_wb);
	irq->intr_idx = INTR_IDX_WB_DONE;
	irq->cb.func = sde_encoder_phys_wb_done_irq;

	irq = &phys_enc->irq[INTR_IDX_CTL_START];
	irq->name = "ctl_start";
	irq->intr_type = SDE_IRQ_TYPE_CTL_START;
	irq->intr_idx = INTR_IDX_CTL_START;
	irq->cb.func = sde_encoder_phys_wb_ctl_start_irq;

	irq = &phys_enc->irq[INTR_IDX_WB_LINEPTR];
	irq->name = "lineptr_irq";
	irq->hw_idx =  wb_enc->hw_wb->idx;
	irq->intr_type = SDE_IRQ_TYPE_WB_PROG_LINE;
	irq->intr_idx = INTR_IDX_WB_LINEPTR;
	irq->cb.func = sde_encoder_phys_wb_lineptr_irq;

	if (wb_cfg && (wb_cfg->features & BIT(SDE_WB_HAS_DCWB))) {
		irq = &phys_enc->irq[INTR_IDX_PP_CWB_OVFL];
		irq->name = "pp_cwb0_overflow";
		irq->hw_idx = PINGPONG_CWB_0;
		irq->intr_type = SDE_IRQ_TYPE_CWB_OVERFLOW;
		irq->intr_idx = INTR_IDX_PP_CWB_OVFL;
		irq->cb.func = sde_encoder_phys_cwb_ovflow;

	} else {
		irq = &phys_enc->irq[INTR_IDX_PP1_OVFL];
		irq->name = "pp1_overflow";
		irq->hw_idx = CWB_1;
		irq->intr_type = SDE_IRQ_TYPE_CWB_OVERFLOW;
		irq->intr_idx = INTR_IDX_PP1_OVFL;
		irq->cb.func = sde_encoder_phys_cwb_ovflow;

		irq = &phys_enc->irq[INTR_IDX_PP2_OVFL];
		irq->name = "pp2_overflow";
		irq->hw_idx = CWB_2;
		irq->intr_type = SDE_IRQ_TYPE_CWB_OVERFLOW;
		irq->intr_idx = INTR_IDX_PP2_OVFL;
		irq->cb.func = sde_encoder_phys_cwb_ovflow;

		irq = &phys_enc->irq[INTR_IDX_PP3_OVFL];
		irq->name = "pp3_overflow";
		irq->hw_idx = CWB_3;
		irq->intr_type = SDE_IRQ_TYPE_CWB_OVERFLOW;
		irq->intr_idx = INTR_IDX_PP3_OVFL;
		irq->cb.func = sde_encoder_phys_cwb_ovflow;

		irq = &phys_enc->irq[INTR_IDX_PP4_OVFL];
		irq->name = "pp4_overflow";
		irq->hw_idx = CWB_4;
		irq->intr_type = SDE_IRQ_TYPE_CWB_OVERFLOW;
		irq->intr_idx = INTR_IDX_PP4_OVFL;
		irq->cb.func = sde_encoder_phys_cwb_ovflow;

		irq = &phys_enc->irq[INTR_IDX_PP5_OVFL];
		irq->name = "pp5_overflow";
		irq->hw_idx = CWB_5;
		irq->intr_type = SDE_IRQ_TYPE_CWB_OVERFLOW;
		irq->intr_idx = INTR_IDX_PP5_OVFL;
		irq->cb.func = sde_encoder_phys_cwb_ovflow;
	}

	/* create internal buffer for disable logic */
	if (_sde_encoder_phys_wb_init_internal_fb(wb_enc, DRM_FORMAT_RGB888, 2, 1, 6)) {
		SDE_ERROR("[enc:%d, wb:%d] failed to init internal fb\n",
				DRMID(phys_enc->parent), WBID(wb_enc));
		goto fail_wb_init;
	}

	SDE_DEBUG("[enc:%d wb:%d] Created wb_phys\n", DRMID(phys_enc->parent), WBID(wb_enc));

	return phys_enc;

fail_wb_init:
fail_wb_check:
fail_mdp_init:
	kfree(wb_enc);
fail_alloc:
	return ERR_PTR(ret);
}
