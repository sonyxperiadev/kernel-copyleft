// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <drm/sde_drm.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>

#include "msm_kms.h"
#include "sde_kms.h"
#include "sde_wb.h"
#include "sde_formats.h"

/* maximum display mode resolution if not available from catalog */
#define SDE_WB_MODE_MAX_WIDTH	5120
#define SDE_WB_MODE_MAX_HEIGHT	5120

static const struct drm_display_mode sde_custom_wb_modes[] = {
/* 5120x2160@60Hz */
{ DRM_MODE("5120x2160", DRM_MODE_TYPE_DRIVER, 693264, 5120, 5128,
		5160, 5200, 0, 2160, 2208, 2216, 2222, 0,
		DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },

{ DRM_MODE("2160x5120", DRM_MODE_TYPE_DRIVER, 693264, 2160, 2208,
		2216, 2222, 0, 5120, 5128, 5160, 5200, 0,
		DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },

{ DRM_MODE("5120x2560", DRM_MODE_TYPE_DRIVER, 818064, 5120, 5128,
		5160, 5200, 0, 2560, 2608, 2616, 2622, 0,
		DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
};

/* Serialization lock for sde_wb_list */
static DEFINE_MUTEX(sde_wb_list_lock);

/* List of all writeback devices installed */
static LIST_HEAD(sde_wb_list);

/**
 * sde_wb_is_format_valid - check if given format/modifier is supported
 * @wb_dev:	Pointer to writeback device
 * @pixel_format:	Fourcc pixel format
 * @format_modifier:	Format modifier
 * Returns:		true if valid; false otherwise
 */
static int sde_wb_is_format_valid(struct sde_wb_device *wb_dev,
		u32 pixel_format, u64 format_modifier)
{
	const struct sde_format_extended *fmts = wb_dev->wb_cfg->format_list;
	int i;

	if (!fmts)
		return false;

	for (i = 0; fmts[i].fourcc_format; i++)
		if ((fmts[i].modifier == format_modifier) &&
				(fmts[i].fourcc_format == pixel_format))
			return true;

	return false;
}

enum drm_connector_status
sde_wb_connector_detect(struct drm_connector *connector,
		bool force,
		void *display)
{
	enum drm_connector_status rc = connector_status_unknown;

	SDE_DEBUG("\n");

	if (display)
		rc = ((struct sde_wb_device *)display)->detect_status;

	return rc;
}

static int sde_wb_connector_add_custom_modes(struct drm_connector *connector,
		u32 hdisplay, u32 vdisplay)
{
	int i, num_modes = 0;
	struct drm_display_mode *mode;
	struct drm_device *dev = connector->dev;

	if (!hdisplay || !vdisplay)
		return 0;

	if (hdisplay > SDE_WB_MODE_MAX_WIDTH)
		hdisplay = SDE_WB_MODE_MAX_WIDTH;
	if (vdisplay > SDE_WB_MODE_MAX_HEIGHT)
		vdisplay = SDE_WB_MODE_MAX_HEIGHT;

	for (i = 0; i < ARRAY_SIZE(sde_custom_wb_modes); i++) {
		const struct drm_display_mode *ptr = &sde_custom_wb_modes[i];

		if (ptr->hdisplay > hdisplay || ptr->vdisplay > vdisplay)
			continue;

		mode = drm_mode_duplicate(dev, ptr);
		if (mode) {
			drm_mode_probed_add(connector, mode);
			num_modes++;
		}
	}

	return num_modes;
}

int sde_wb_connector_get_modes(struct drm_connector *connector, void *display,
		const struct msm_resource_caps_info *avail_res)
{
	struct sde_wb_device *wb_dev;
	int num_modes = 0;

	if (!connector || !display)
		return 0;

	wb_dev = display;

	SDE_DEBUG("\n");

	mutex_lock(&wb_dev->wb_lock);
	if (wb_dev->count_modes && wb_dev->modes) {
		struct drm_display_mode *mode;
		int i, ret;

		for (i = 0; i < wb_dev->count_modes; i++) {
			mode = drm_mode_create(connector->dev);
			if (!mode) {
				SDE_ERROR("failed to create mode\n");
				break;
			}
			ret = drm_mode_convert_umode(wb_dev->drm_dev, mode,
					&wb_dev->modes[i]);
			if (ret) {
				SDE_ERROR("failed to convert mode %d\n", ret);
				break;
			}

			drm_mode_probed_add(connector, mode);
			num_modes++;
		}
	} else {
		u32 max_width = SDE_WB_MODE_MAX_WIDTH;

		if (wb_dev->wb_cfg && wb_dev->wb_cfg->sblk)
			max_width = max(wb_dev->wb_cfg->sblk->maxlinewidth,
				wb_dev->wb_cfg->sblk->maxlinewidth_linear);

		num_modes = drm_add_modes_noedid(connector, max_width,
				SDE_WB_MODE_MAX_HEIGHT);

		num_modes += sde_wb_connector_add_custom_modes(connector, max_width,
				SDE_WB_MODE_MAX_HEIGHT);
	}
	mutex_unlock(&wb_dev->wb_lock);
	return num_modes;
}

struct drm_framebuffer *
sde_wb_connector_state_get_output_fb(struct drm_connector_state *state)
{
	if (!state || !state->connector ||
		(state->connector->connector_type !=
				DRM_MODE_CONNECTOR_VIRTUAL)) {
		SDE_ERROR("invalid params\n");
		return NULL;
	}

	SDE_DEBUG("\n");

	return sde_connector_get_out_fb(state);
}

int sde_wb_connector_state_get_output_roi(struct drm_connector_state *state,
		struct sde_rect *roi)
{
	if (!state || !roi || !state->connector ||
		(state->connector->connector_type !=
				DRM_MODE_CONNECTOR_VIRTUAL)) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	roi->x = sde_connector_get_property(state, CONNECTOR_PROP_DST_X);
	roi->y = sde_connector_get_property(state, CONNECTOR_PROP_DST_Y);
	roi->w = sde_connector_get_property(state, CONNECTOR_PROP_DST_W);
	roi->h = sde_connector_get_property(state, CONNECTOR_PROP_DST_H);

	return 0;
}

/**
 * sde_wb_connector_set_modes - set writeback modes and connection status
 * @wb_dev:	Pointer to write back device
 * @count_modes:	Count of modes
 * @modes:	Pointer to writeback mode requested
 * @connected:	Connection status requested
 * Returns:	0 if success; error code otherwise
 */
static
int sde_wb_connector_set_modes(struct sde_wb_device *wb_dev,
		u32 count_modes, struct drm_mode_modeinfo __user *modes,
		bool connected)
{
	struct drm_mode_modeinfo *modeinfo = NULL;
	int ret = 0;
	int i;

	if (!wb_dev || !wb_dev->connector ||
			(wb_dev->connector->connector_type !=
			 DRM_MODE_CONNECTOR_VIRTUAL)) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	if (connected) {
		SDE_DEBUG("connect\n");

		if (!count_modes || !modes) {
			SDE_ERROR("invalid count_modes :%u and modes :%d\n",
				count_modes, !modes);
			return -EINVAL;
		}

		modeinfo = kcalloc(count_modes,
				sizeof(struct drm_mode_modeinfo),
				GFP_KERNEL);
		if (!modeinfo) {
			SDE_ERROR("invalid params\n");
			ret = -ENOMEM;
			goto error;
		}

		if (copy_from_user(modeinfo, modes,
				count_modes *
				sizeof(struct drm_mode_modeinfo))) {
			SDE_ERROR("failed to copy modes\n");
			kfree(modeinfo);
			ret = -EFAULT;
			goto error;
		}

		for (i = 0; i < count_modes; i++) {
			struct drm_display_mode dispmode;

			memset(&dispmode, 0, sizeof(dispmode));
			ret = drm_mode_convert_umode(wb_dev->drm_dev,
					&dispmode, &modeinfo[i]);
			if (ret) {
				SDE_ERROR(
					"failed to convert mode %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x status:%d rc:%d\n",
					i,
					modeinfo[i].name,
					modeinfo[i].vrefresh,
					modeinfo[i].clock,
					modeinfo[i].hdisplay,
					modeinfo[i].hsync_start,
					modeinfo[i].hsync_end,
					modeinfo[i].htotal,
					modeinfo[i].vdisplay,
					modeinfo[i].vsync_start,
					modeinfo[i].vsync_end,
					modeinfo[i].vtotal,
					modeinfo[i].type,
					modeinfo[i].flags,
					dispmode.status,
					ret);
				kfree(modeinfo);
				goto error;
			}
		}

		if (wb_dev->modes) {
			wb_dev->count_modes = 0;

			kfree(wb_dev->modes);
			wb_dev->modes = NULL;
		}

		wb_dev->count_modes = count_modes;
		wb_dev->modes = modeinfo;
		wb_dev->detect_status = connector_status_connected;
	} else {
		SDE_DEBUG("disconnect\n");

		if (wb_dev->modes) {
			wb_dev->count_modes = 0;

			kfree(wb_dev->modes);
			wb_dev->modes = NULL;
		}

		wb_dev->detect_status = connector_status_disconnected;
	}

error:
	return ret;
}

static void _sde_wb_connector_clear_dnsc_blur(struct drm_connector_state *state)
{
	struct sde_connector_state *cstate = to_sde_connector_state(state);
	int i;

	for (i = 0; i < cstate->dnsc_blur_count; i++)
		memset(&cstate->dnsc_blur_cfg[i], 0, sizeof(struct sde_drm_dnsc_blur_cfg));
	cstate->dnsc_blur_count = 0;
}

static int _sde_wb_connector_set_dnsc_blur(struct sde_wb_device *wb_dev,
		struct drm_connector_state *state, void __user *usr_ptr)
{
	struct sde_connector_state *cstate = to_sde_connector_state(state);
	struct sde_kms *sde_kms = sde_connector_get_kms(wb_dev->connector);
	struct sde_drm_dnsc_blur_cfg *dnsc_blur_cfg = &cstate->dnsc_blur_cfg[0];
	u32 copy_count;
	int ret = 0, i;

	if (!sde_kms || !sde_kms->catalog)
		return -EINVAL;

	if (!usr_ptr)
		goto disable;

	/* copy only the first block */
	if (copy_from_user(dnsc_blur_cfg, usr_ptr, sizeof(struct sde_drm_dnsc_blur_cfg))) {
		SDE_ERROR("failed to copy dnsc_blur block 0 data\n");
		ret = -EINVAL;
		goto disable;
	}

	if (dnsc_blur_cfg->num_blocks > sde_kms->catalog->dnsc_blur_count) {
		SDE_ERROR("invalid number of dnsc_blur blocks:%d\n", dnsc_blur_cfg->num_blocks);
		ret = -EINVAL;
		goto disable;
	}

	/* no further data required */
	if (dnsc_blur_cfg->num_blocks <= 1)
		goto end;

	dnsc_blur_cfg++;
	usr_ptr += sizeof(struct sde_drm_dnsc_blur_cfg);
	copy_count = dnsc_blur_cfg->num_blocks - 1;

	/* copy rest of the blocks */
	if ((dnsc_blur_cfg->flags & DNSC_BLUR_INDEPENDENT_BLK_CFG)) {
		if (copy_from_user(dnsc_blur_cfg, usr_ptr,
				copy_count * sizeof(struct sde_drm_dnsc_blur_cfg))) {
			SDE_ERROR("failed to copy dnsc_blur data\n");
			ret = -EINVAL;
			goto disable;
		}

	/* duplicate rest of the blocks */
	} else if (dnsc_blur_cfg->flags & DNSC_BLUR_MIRROR_BLK_CFG) {
		for (i = 0; i < copy_count; i++) {
			memcpy(dnsc_blur_cfg, &cstate->dnsc_blur_cfg[0],
					sizeof(struct sde_drm_dnsc_blur_cfg));
			dnsc_blur_cfg++;
		}
	}

end:
	cstate->dnsc_blur_count = dnsc_blur_cfg->num_blocks;
	return 0;

disable:
	_sde_wb_connector_clear_dnsc_blur(state);
	return ret;
}

static int _sde_wb_connector_set_out_fb(struct sde_wb_device *wb_dev,
		struct drm_connector_state *state)
{
	struct drm_framebuffer *out_fb;
	const struct sde_format *sde_format;
	int rc = 0;

	out_fb = sde_connector_get_out_fb(state);
	if (!out_fb)
		goto end;

	sde_format = sde_get_sde_format_ext(out_fb->format->format, out_fb->modifier);
	if (!sde_format) {
		SDE_ERROR("failed to get sde format\n");
		rc = -EINVAL;
		goto end;
	}

	if (!sde_wb_is_format_valid(wb_dev, out_fb->format->format, out_fb->modifier)) {
		SDE_ERROR("unsupported writeback format 0x%x/0x%llx\n",
				out_fb->format->format, out_fb->modifier);
		rc = -EINVAL;
		goto end;
	}

end:
	return rc;
}

int sde_wb_connector_set_property(struct drm_connector *connector,
		struct drm_connector_state *state, int idx, uint64_t value, void *display)
{
	struct sde_wb_device *wb_dev = display;
	int rc = 0;

	if (!connector || !state || !wb_dev) {
		SDE_ERROR("invalid argument(s)\n");
		return -EINVAL;
	}

	switch (idx) {
	case CONNECTOR_PROP_OUT_FB:
		rc = _sde_wb_connector_set_out_fb(wb_dev, state);
		break;
	case CONNECTOR_PROP_DNSC_BLUR:
		rc = _sde_wb_connector_set_dnsc_blur(wb_dev, state,
				(void __user *)(uintptr_t)value);
		break;
	default:
		/* nothing to do */
		break;
	}

	return rc;
}

int sde_wb_get_info(struct drm_connector *connector,
		struct msm_display_info *info, void *display)
{
	struct sde_wb_device *wb_dev = display;
	u32 max_width = SDE_WB_MODE_MAX_WIDTH;

	if (!info || !wb_dev) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	if (wb_dev->wb_cfg && wb_dev->wb_cfg->sblk)
		max_width = max(wb_dev->wb_cfg->sblk->maxlinewidth,
				wb_dev->wb_cfg->sblk->maxlinewidth_linear);

	memset(info, 0, sizeof(struct msm_display_info));
	info->intf_type = DRM_MODE_CONNECTOR_VIRTUAL;
	info->num_of_h_tiles = 1;
	info->h_tile_instance[0] = sde_wb_get_index(display);
	info->is_connected = true;
	info->capabilities = MSM_DISPLAY_CAP_HOT_PLUG | MSM_DISPLAY_CAP_EDID;
	info->max_width = max_width;
	info->max_height = SDE_WB_MODE_MAX_HEIGHT;
	return 0;
}

int sde_wb_get_mode_info(struct drm_connector *connector,
		const struct drm_display_mode *drm_mode,
		struct msm_sub_mode *sub_mode,
		struct msm_mode_info *mode_info,
		void *display, const struct msm_resource_caps_info *avail_res)
{
	const u32 dual_lm = 2;
	const u32 single_lm = 1;
	const u32 single_intf = 1;
	const u32 no_enc = 0;
	struct msm_display_topology *topology;

	if (!drm_mode || !mode_info || !avail_res ||
			!avail_res->max_mixer_width || !display) {
		pr_err("invalid params\n");
		return -EINVAL;
	}

	topology = &mode_info->topology;
	topology->num_lm = (avail_res->max_mixer_width <= drm_mode->hdisplay) ?
			dual_lm : single_lm;
	topology->num_enc = no_enc;
	topology->num_intf = single_intf;

	if (topology->num_lm == dual_lm && (drm_mode->hdisplay % 4)) {
		SDE_ERROR("invalid mode settings for 3d-merge, hdisplay:%d\n", drm_mode->hdisplay);
		return -EINVAL;
	}

	mode_info->comp_info.comp_type = MSM_DISPLAY_COMPRESSION_NONE;
	mode_info->wide_bus_en = false;
	mode_info->comp_info.comp_ratio = MSM_DISPLAY_COMPRESSION_RATIO_NONE;

	return 0;
}

int sde_wb_connector_set_info_blob(struct drm_connector *connector,
		void *info, void *display, struct msm_mode_info *mode_info)
{
	struct sde_wb_device *wb_dev = display;
	const struct sde_format_extended *format_list;
	struct sde_kms *sde_kms;
	struct sde_mdss_cfg *catalog;
	int i;

	if (!connector || !info || !display || !wb_dev->wb_cfg) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	sde_kms = sde_connector_get_kms(connector);
	if (!sde_kms)
		return -EINVAL;
	catalog = sde_kms->catalog;

	format_list = wb_dev->wb_cfg->format_list;

	/* Populate info buffer */
	if (format_list) {
		sde_kms_info_start(info, "pixel_formats");
		while (format_list->fourcc_format) {
			sde_kms_info_append_format(info, format_list->fourcc_format,
					format_list->modifier);
			++format_list;
		}
		sde_kms_info_stop(info);
	}

	/* Populate info buffer with WB rotation output formats */
	format_list = wb_dev->wb_cfg->rot_format_list;
	if (format_list) {
		sde_kms_info_start(info, "rot_output_formats");
		while (format_list->fourcc_format) {
			sde_kms_info_append_format(info, format_list->fourcc_format,
					format_list->modifier);
			++format_list;
		}
		sde_kms_info_stop(info);
	}

	sde_kms_info_add_keyint(info, "wb_intf_index", wb_dev->wb_idx - WB_0);
	sde_kms_info_add_keyint(info, "maxlinewidth", wb_dev->wb_cfg->sblk->maxlinewidth);
	sde_kms_info_add_keyint(info, "maxlinewidth_linear",
			wb_dev->wb_cfg->sblk->maxlinewidth_linear);

	sde_kms_info_start(info, "features");
	if (wb_dev->wb_cfg && (wb_dev->wb_cfg->features & BIT(SDE_WB_UBWC)))
		sde_kms_info_append(info, "wb_ubwc");
	sde_kms_info_stop(info);

	sde_kms_info_add_keyint(info, "has_cwb_dither", test_bit(SDE_FEATURE_CWB_DITHER,
				catalog->features));

	if (catalog->cdm_count)
		sde_kms_info_add_keyint(info, "cdm_count", catalog->cdm_count);

	if (catalog->dnsc_blur_count && catalog->dnsc_blur_filters) {
		sde_kms_info_add_keyint(info, "dnsc_blur_count", catalog->dnsc_blur_count);

		sde_kms_info_start(info, "dnsc_blur_info");
		for (i = 0; i < catalog->dnsc_blur_filter_count; i++)
			sde_kms_info_append_dnsc_blur_filter_info(info,
						&catalog->dnsc_blur_filters[i]);
		sde_kms_info_stop(info);
	}

	return 0;
}

static void _sde_wb_connector_install_dither_property(struct sde_wb_device *wb_dev)
{
	struct sde_connector *c_conn = to_sde_connector(wb_dev->connector);
	struct sde_kms *sde_kms = sde_connector_get_kms(wb_dev->connector);
	struct sde_mdss_cfg *catalog;
	char prop_name[DRM_PROP_NAME_LEN];
	u32 version = 0;

	if (!sde_kms || !sde_kms->catalog)
		return;
	catalog = sde_kms->catalog;

	if (!test_bit(SDE_FEATURE_CWB_DITHER, catalog->features))
		return;

	version = SDE_COLOR_PROCESS_MAJOR(catalog->pingpong[0].sblk->dither.version);
	snprintf(prop_name, ARRAY_SIZE(prop_name), "%s%d", "SDE_PP_CWB_DITHER_V", version);
	switch (version) {
	case 2:
		msm_property_install_blob(&c_conn->property_info, prop_name,
			DRM_MODE_PROP_BLOB, CONNECTOR_PROP_PP_CWB_DITHER);
		break;
	default:
		SDE_ERROR("unsupported cwb dither version %d\n", version);
		return;
	}
}

int sde_wb_connector_post_init(struct drm_connector *connector, void *display)
{
	struct sde_connector *c_conn;
	struct sde_wb_device *wb_dev = display;
	struct msm_drm_private *priv;
	struct sde_kms *sde_kms;
	struct sde_mdss_cfg *catalog;
	static const struct drm_prop_enum_list e_fb_translation_mode[] = {
		{SDE_DRM_FB_NON_SEC, "non_sec"},
		{SDE_DRM_FB_SEC, "sec"},
	};
	static const struct drm_prop_enum_list e_cache_state[] = {
		{CACHE_STATE_DISABLED, "cache_state_disabled"},
		{CACHE_STATE_ENABLED, "cache_state_enabled"},
	};

	static const struct drm_prop_enum_list e_wb_usage_type[] = {
		{WB_USAGE_WFD, "wb_usage_wfd"},
		{WB_USAGE_CWB, "wb_usage_cwb"},
		{WB_USAGE_OFFLINE_WB, "wb_usage_offline_wb"},
		{WB_USAGE_ROT, "wb_usage_rot"},
	};

	static const struct drm_prop_enum_list e_wb_rotate_type[] = {
		{WB_ROT_NONE, "wb_rot_none"},
		{WB_ROT_SINGLE, "wb_rot_single"},
		{WB_ROT_JOB1, "wb_rot_job1"},
		{WB_ROT_JOB2, "wb_rot_job2"},
	};

	if (!connector || !display || !wb_dev->wb_cfg || !wb_dev->drm_dev->dev_private) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	priv = wb_dev->drm_dev->dev_private;
	sde_kms = to_sde_kms(priv->kms);
	if (!sde_kms || !sde_kms->catalog) {
		SDE_ERROR("invalid sde_kms\n");
		return -EINVAL;
	}

	catalog = sde_kms->catalog;
	c_conn = to_sde_connector(connector);
	wb_dev->connector = connector;
	wb_dev->detect_status = connector_status_connected;

	if (test_bit(SDE_SYS_CACHE_DISP, catalog->sde_sys_cache_type_map)
			|| test_bit(SDE_SYS_CACHE_DISP_WB, catalog->sde_sys_cache_type_map))
		msm_property_install_enum(&c_conn->property_info, "cache_state",
			0x0, 0, e_cache_state, ARRAY_SIZE(e_cache_state),
			0, CONNECTOR_PROP_CACHE_STATE);

	/*
	 * Add extra connector properties
	 */
	msm_property_install_range(&c_conn->property_info, "FB_ID",
			0x0, 0, ~0, 0, CONNECTOR_PROP_OUT_FB);
	msm_property_install_range(&c_conn->property_info, "DST_X",
			0x0, 0, UINT_MAX, 0, CONNECTOR_PROP_DST_X);
	msm_property_install_range(&c_conn->property_info, "DST_Y",
			0x0, 0, UINT_MAX, 0, CONNECTOR_PROP_DST_Y);
	msm_property_install_range(&c_conn->property_info, "DST_W",
			0x0, 0, UINT_MAX, 0, CONNECTOR_PROP_DST_W);
	msm_property_install_range(&c_conn->property_info, "DST_H",
			0x0, 0, UINT_MAX, 0, CONNECTOR_PROP_DST_H);
	msm_property_install_enum(&c_conn->property_info,
			"fb_translation_mode",
			0x0,
			0, e_fb_translation_mode,
			ARRAY_SIZE(e_fb_translation_mode), 0,
			CONNECTOR_PROP_FB_TRANSLATION_MODE);

	if (wb_dev->wb_cfg->features & BIT(SDE_WB_PROG_LINE))
		msm_property_install_range(&c_conn->property_info, "early_fence_line",
			0x0, 0, UINT_MAX, 0, CONNECTOR_PROP_EARLY_FENCE_LINE);

	if (catalog->dnsc_blur_count && catalog->dnsc_blur_filters)
		msm_property_install_range(&c_conn->property_info, "dnsc_blur",
			0x0, 0, ~0, 0, CONNECTOR_PROP_DNSC_BLUR);

	if (wb_dev->wb_cfg->features & BIT(SDE_WB_LINEAR_ROTATION)) {
		msm_property_install_enum(&c_conn->property_info, "wb_rotate_type",
			0x0, 0, e_wb_rotate_type, ARRAY_SIZE(e_wb_rotate_type),
			0, CONNECTOR_PROP_WB_ROT_TYPE);

		msm_property_install_range(&c_conn->property_info, "wb_rot_bytes_per_clk",
			0x0, 0, UINT_MAX, 0, CONNECTOR_PROP_WB_ROT_BYTES_PER_CLK);
	}

	msm_property_install_enum(&c_conn->property_info, "wb_usage_type",
			0x0, 0, e_wb_usage_type, ARRAY_SIZE(e_wb_usage_type),
			0, CONNECTOR_PROP_WB_USAGE_TYPE);

	_sde_wb_connector_install_dither_property(wb_dev);

	return 0;
}

struct drm_framebuffer *sde_wb_get_output_fb(struct sde_wb_device *wb_dev)
{
	struct drm_framebuffer *fb;

	if (!wb_dev || !wb_dev->connector) {
		SDE_ERROR("invalid params\n");
		return NULL;
	}

	SDE_DEBUG("\n");

	mutex_lock(&wb_dev->wb_lock);
	fb = sde_wb_connector_state_get_output_fb(wb_dev->connector->state);
	mutex_unlock(&wb_dev->wb_lock);

	return fb;
}

int sde_wb_get_output_roi(struct sde_wb_device *wb_dev, struct sde_rect *roi)
{
	int rc;

	if (!wb_dev || !wb_dev->connector || !roi) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	mutex_lock(&wb_dev->wb_lock);
	rc = sde_wb_connector_state_get_output_roi(
			wb_dev->connector->state, roi);
	mutex_unlock(&wb_dev->wb_lock);

	return rc;
}

u32 sde_wb_get_num_of_displays(void)
{
	u32 count = 0;
	struct sde_wb_device *wb_dev;

	SDE_DEBUG("\n");

	mutex_lock(&sde_wb_list_lock);
	list_for_each_entry(wb_dev, &sde_wb_list, wb_list) {
		count++;
	}
	mutex_unlock(&sde_wb_list_lock);

	return count;
}

int wb_display_get_displays(void **display_array, u32 max_display_count)
{
	struct sde_wb_device *curr;
	int i = 0;

	SDE_DEBUG("\n");

	if (!display_array || !max_display_count) {
		if (!display_array)
			SDE_ERROR("invalid param\n");
		return 0;
	}

	mutex_lock(&sde_wb_list_lock);
	list_for_each_entry(curr, &sde_wb_list, wb_list) {
		if (i >= max_display_count)
			break;
		display_array[i++] = curr;
	}
	mutex_unlock(&sde_wb_list_lock);

	return i;
}

int sde_wb_config(struct drm_device *drm_dev, void *data,
				struct drm_file *file_priv)
{
	struct sde_drm_wb_cfg *config = data;
	struct msm_drm_private *priv;
	struct sde_wb_device *wb_dev = NULL;
	struct sde_wb_device *curr;
	struct drm_connector *connector;
	uint32_t flags;
	uint32_t connector_id;
	uint32_t count_modes;
	uint64_t modes;
	int rc;

	if (!drm_dev || !data) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	flags = config->flags;
	connector_id = config->connector_id;
	count_modes = config->count_modes;
	modes = config->modes;

	priv = drm_dev->dev_private;

	connector = drm_connector_lookup(drm_dev, file_priv, connector_id);
	if (!connector) {
		SDE_ERROR("failed to find connector\n");
		rc = -ENOENT;
		goto fail;
	}

	mutex_lock(&sde_wb_list_lock);
	list_for_each_entry(curr, &sde_wb_list, wb_list) {
		if (curr->connector == connector) {
			wb_dev = curr;
			break;
		}
	}
	mutex_unlock(&sde_wb_list_lock);

	if (!wb_dev) {
		SDE_ERROR("failed to find wb device\n");
		rc = -ENOENT;
		goto fail;
	}

	mutex_lock(&wb_dev->wb_lock);

	rc = sde_wb_connector_set_modes(wb_dev, count_modes,
		(struct drm_mode_modeinfo __user *) (uintptr_t) modes,
		(flags & SDE_DRM_WB_CFG_FLAGS_CONNECTED) ? true : false);

	mutex_unlock(&wb_dev->wb_lock);
	drm_helper_hpd_irq_event(drm_dev);
fail:
	return rc;
}

/**
 * _sde_wb_dev_init - perform device initialization
 * @wb_dev:	Pointer to writeback device
 */
static int _sde_wb_dev_init(struct sde_wb_device *wb_dev)
{
	int rc = 0;

	if (!wb_dev) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	return rc;
}

/**
 * _sde_wb_dev_deinit - perform device de-initialization
 * @wb_dev:	Pointer to writeback device
 */
static int _sde_wb_dev_deinit(struct sde_wb_device *wb_dev)
{
	int rc = 0;

	if (!wb_dev) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	return rc;
}

/**
 * sde_wb_bind - bind writeback device with controlling device
 * @dev:        Pointer to base of platform device
 * @master:     Pointer to container of drm device
 * @data:       Pointer to private data
 * Returns:     Zero on success
 */
static int sde_wb_bind(struct device *dev, struct device *master, void *data)
{
	struct sde_wb_device *wb_dev;

	if (!dev || !master) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	wb_dev = platform_get_drvdata(to_platform_device(dev));
	if (!wb_dev) {
		SDE_ERROR("invalid wb device\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	mutex_lock(&wb_dev->wb_lock);
	wb_dev->drm_dev = dev_get_drvdata(master);
	mutex_unlock(&wb_dev->wb_lock);

	return 0;
}

/**
 * sde_wb_unbind - unbind writeback from controlling device
 * @dev:        Pointer to base of platform device
 * @master:     Pointer to container of drm device
 * @data:       Pointer to private data
 */
static void sde_wb_unbind(struct device *dev,
		struct device *master, void *data)
{
	struct sde_wb_device *wb_dev;

	if (!dev) {
		SDE_ERROR("invalid params\n");
		return;
	}

	wb_dev = platform_get_drvdata(to_platform_device(dev));
	if (!wb_dev) {
		SDE_ERROR("invalid wb device\n");
		return;
	}

	SDE_DEBUG("\n");

	mutex_lock(&wb_dev->wb_lock);
	wb_dev->drm_dev = NULL;
	mutex_unlock(&wb_dev->wb_lock);
}

static const struct component_ops sde_wb_comp_ops = {
	.bind = sde_wb_bind,
	.unbind = sde_wb_unbind,
};

/**
 * sde_wb_drm_init - perform DRM initialization
 * @wb_dev:	Pointer to writeback device
 * @encoder:	Pointer to associated encoder
 */
int sde_wb_drm_init(struct sde_wb_device *wb_dev, struct drm_encoder *encoder)
{
	int rc = 0;

	if (!wb_dev || !wb_dev->drm_dev || !encoder) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	mutex_lock(&wb_dev->wb_lock);

	if (wb_dev->drm_dev->dev_private) {
		struct msm_drm_private *priv = wb_dev->drm_dev->dev_private;
		struct sde_kms *sde_kms = to_sde_kms(priv->kms);

		if (wb_dev->index < sde_kms->catalog->wb_count) {
			wb_dev->wb_idx = sde_kms->catalog->wb[wb_dev->index].id;
			wb_dev->wb_cfg = &sde_kms->catalog->wb[wb_dev->index];
		}
	}

	wb_dev->drm_dev = encoder->dev;
	wb_dev->encoder = encoder;
	mutex_unlock(&wb_dev->wb_lock);
	return rc;
}

int sde_wb_drm_deinit(struct sde_wb_device *wb_dev)
{
	int rc = 0;

	if (!wb_dev) {
		SDE_ERROR("invalid params\n");
		return -EINVAL;
	}

	SDE_DEBUG("\n");

	return rc;
}

/**
 * sde_wb_probe - load writeback module
 * @pdev:	Pointer to platform device
 */
static int sde_wb_probe(struct platform_device *pdev)
{
	struct sde_wb_device *wb_dev;
	int ret;

	wb_dev = devm_kzalloc(&pdev->dev, sizeof(*wb_dev), GFP_KERNEL);
	if (!wb_dev)
		return -ENOMEM;

	SDE_DEBUG("\n");

	ret = of_property_read_u32(pdev->dev.of_node, "cell-index",
			&wb_dev->index);
	if (ret) {
		SDE_DEBUG("cell index not set, default to 0\n");
		wb_dev->index = 0;
	}

	wb_dev->name = of_get_property(pdev->dev.of_node, "label", NULL);
	if (!wb_dev->name) {
		SDE_DEBUG("label not set, default to unknown\n");
		wb_dev->name = "unknown";
	}

	wb_dev->wb_idx = SDE_NONE;

	mutex_init(&wb_dev->wb_lock);
	platform_set_drvdata(pdev, wb_dev);

	mutex_lock(&sde_wb_list_lock);
	list_add_tail(&wb_dev->wb_list, &sde_wb_list);
	mutex_unlock(&sde_wb_list_lock);

	if (!_sde_wb_dev_init(wb_dev)) {
		ret = component_add(&pdev->dev, &sde_wb_comp_ops);
		if (ret)
			pr_err("component add failed\n");
	}

	return ret;
}

/**
 * sde_wb_remove - unload writeback module
 * @pdev:	Pointer to platform device
 */
static int sde_wb_remove(struct platform_device *pdev)
{
	struct sde_wb_device *wb_dev;
	struct sde_wb_device *curr, *next;

	wb_dev = platform_get_drvdata(pdev);
	if (!wb_dev)
		return 0;

	SDE_DEBUG("\n");

	(void)_sde_wb_dev_deinit(wb_dev);

	mutex_lock(&sde_wb_list_lock);
	list_for_each_entry_safe(curr, next, &sde_wb_list, wb_list) {
		if (curr == wb_dev) {
			list_del(&wb_dev->wb_list);
			break;
		}
	}
	mutex_unlock(&sde_wb_list_lock);

	kfree(wb_dev->modes);
	mutex_destroy(&wb_dev->wb_lock);

	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, wb_dev);

	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "qcom,wb-display"},
	{}
};

static struct platform_driver sde_wb_driver = {
	.probe = sde_wb_probe,
	.remove = sde_wb_remove,
	.driver = {
		.name = "sde_wb",
		.of_match_table = dt_match,
		.suppress_bind_attrs = true,
	},
};

void __init sde_wb_register(void)
{
	platform_driver_register(&sde_wb_driver);
}

void __exit sde_wb_unregister(void)
{
	platform_driver_unregister(&sde_wb_driver);
}
