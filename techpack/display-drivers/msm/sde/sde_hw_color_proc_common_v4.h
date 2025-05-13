/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2017-2019, 2021 The Linux Foundation. All rights reserved.
 */
#ifndef _SDE_HW_COLOR_PROC_COMMON_V4_H_
#define _SDE_HW_COLOR_PROC_COMMON_V4_H_

#include "sde_hw_mdss.h"
#include "sde_dbg.h"

/*
 * DEMURA fetch planes
 * @DEM_FETCH_DMA1_RECT0      Demura data fetched from DMA plane 1 rectangle 0
 * @DEM_FETCH_DMA1_RECT1      Demura data fetched from DMA plane 1 rectangle 1
 * @DEM_FETCH_DMA3_RECT0      Demura data fetched from DMA plane 3 rectangle 0
 * @DEM_FETCH_DMA3_RECT1      Demura data fetched from DMA plane 3 rectangle 1
 * @DEM_FETCH_DMA_INVALID     Invalid DMA plane for fetching Demmura data
 */
enum demura_fetch_planes {
	DEM_FETCH_DMA1_RECT0 = 0,
	DEM_FETCH_DMA1_RECT1,
	DEM_FETCH_DMA3_RECT0,
	DEM_FETCH_DMA3_RECT1,
	DEM_FETCH_DMA_INVALID,
};

/**
 * struct sde_demura_fetch_planes - drm prop enun struct containing bit
 *     mask enum properties and values
 */
static const struct drm_prop_enum_list sde_demura_fetch_planes[] = {
	{DEM_FETCH_DMA1_RECT0, "demura_dma1_rect0"},
	{DEM_FETCH_DMA1_RECT1, "demura_dma1_rect1"},
	{DEM_FETCH_DMA3_RECT0, "demura_dma3_rect0"},
	{DEM_FETCH_DMA3_RECT1, "demura_dma3_rect1"},
};

#define GAMUT_TABLE_SEL_OFF 0x4
#define GAMUT_UPPER_COLOR_OFF 0x8
#define GAMUT_LOWER_COLOR_OFF 0xc
#define GAMUT_SCALEA_OFFSET_OFF 0x10
#define GAMUT_SCALEB_OFFSET_OFF 0xe0
#define GAMUT_TABLE0_SEL BIT(12)
#define GAMUT_MAP_EN BIT(1)
#define GAMUT_EN BIT(0)
#define GAMUT_MODE_13B_OFF 640
#define GAMUT_MODE_5_OFF 1248

enum {
	gamut_mode_17 = 0,
	gamut_mode_5,
	gamut_mode_13a,
	gamut_mode_13b,
	gamut_mode_17b,
};

#define GC_C0_OFF 0x4
#define GC_C0_INDEX_OFF 0x8
#define GC_8B_ROUND_EN BIT(1)
#define GC_EN BIT(0)
#define GC_TBL_NUM 3
#define GC_LUT_SWAP_OFF 0x1c

#define IGC_TBL_NUM 3
#define IGC_DITHER_OFF 0x7e0
#define IGC_OPMODE_OFF 0x0
#define IGC_C0_OFF 0x0
#define IGC_DATA_MASK (BIT(12) - 1)
#define IGC_DSPP_SEL_MASK_MAX (BIT(4) - 1)
#define IGC_DSPP_SEL_MASK(n) \
	((IGC_DSPP_SEL_MASK_MAX & ~(1 << (n))) << 28)
#define IGC_INDEX_UPDATE BIT(25)
#define IGC_EN BIT(0)
#define IGC_DIS 0
#define IGC_DITHER_DATA_MASK (BIT(4) - 1)

#define PCC_NUM_PLANES 3
#define PCC_NUM_COEFF 11
#define PCC_EN BIT(0)
#define PCC_DIS 0
#define PCC_C_OFF 0x4
#define PCC_R_OFF 0x10
#define PCC_G_OFF 0x1c
#define PCC_B_OFF 0x28
#define PCC_RG_OFF 0x34
#define PCC_RB_OFF 0x40
#define PCC_GB_OFF 0x4c
#define PCC_RGB_OFF 0x58
#define PCC_RR_OFF 0x64
#define PCC_GG_OFF 0x70
#define PCC_BB_OFF 0x7c

#define PA_EN BIT(20)
#define PA_HUE_EN BIT(25)
#define PA_SAT_EN BIT(26)
#define PA_VAL_EN BIT(27)
#define PA_CONT_EN BIT(28)

#define PA_SIXZONE_HUE_EN BIT(29)
#define PA_SIXZONE_SAT_EN BIT(30)
#define PA_SIXZONE_VAL_EN BIT(31)
#define PA_SIXZONE_SV_EN BIT(0)

#define PA_HIST_EN BIT(16)

#define PA_SKIN_EN BIT(5)
#define PA_FOL_EN BIT(6)
#define PA_SKY_EN BIT(7)

#define PA_HUE_MASK (BIT(12) - 1)
#define PA_SAT_MASK (BIT(16) - 1)
#define PA_VAL_MASK (BIT(8) - 1)
#define PA_CONT_MASK (BIT(8) - 1)

#define PA_HUE_OFF 0x1c
#define PA_SAT_OFF 0x20
#define PA_VAL_OFF 0x24
#define PA_CONT_OFF 0x28
#define PA_PWL_HOLD_OFF 0x40

#define PA_DISABLE_REQUIRED(x) \
	!((x) & (PA_SKIN_EN | PA_SKY_EN | \
	PA_FOL_EN | PA_HUE_EN | \
	PA_SAT_EN | PA_VAL_EN | \
	PA_CONT_EN | PA_HIST_EN | \
	PA_SIXZONE_HUE_EN | PA_SIXZONE_SAT_EN | \
	PA_SIXZONE_VAL_EN))

#define SIXZONE_ADJ_CURVE_P1_OFF 0x4
#define SIXZONE_THRESHOLDS_OFF 0x8
#define SIXZONE_ADJ_PWL0_OFF 0xC
#define SIXZONE_ADJ_PWL1_OFF 0x10
#define SIXZONE_SAT_PWL0_OFF 0x14
#define SIXZONE_SAT_PWL1_OFF 0x18
#define SIXZONE_SV_CTL_OFF 0x20

#define MEMCOL_SIZE0 20
#define MEMCOL_SIZE1 8
#define MEMCOL_PWL0_OFF 0x0
#define MEMCOL_PWL2_OFF 0x3C
#define MEMCOL_HOLD_SIZE 0x4

#define MEMCOL_PROT_VAL_EN BIT(24)
#define MEMCOL_PROT_SAT_EN BIT(23)
#define MEMCOL_PROT_HUE_EN BIT(22)
#define MEMCOL_PROT_CONT_EN BIT(18)
#define MEMCOL_PROT_SIXZONE_EN BIT(17)
#define MEMCOL_PROT_BLEND_EN BIT(3)

#define MEMCOL_PROT_MASK \
	(MEMCOL_PROT_VAL_EN | MEMCOL_PROT_SAT_EN | \
	MEMCOL_PROT_HUE_EN | MEMCOL_PROT_CONT_EN | \
	MEMCOL_PROT_SIXZONE_EN | MEMCOL_PROT_BLEND_EN)

#define SSPP 0
#define DSPP 1

#define LTM_CONFIG_MERGE_MODE_ONLY (BIT(16) | BIT(17))

/**
 * Hardware register set
 */
#define SDE_HW_RC_REG0 0x00
#define SDE_HW_RC_REG1 0x04
#define SDE_HW_RC_REG2 0x08
#define SDE_HW_RC_REG3 0x0C
#define SDE_HW_RC_REG4 0x10
#define SDE_HW_RC_REG5 0x14
#define SDE_HW_RC_REG6 0x18
#define SDE_HW_RC_REG7 0x1C
#define SDE_HW_RC_REG8 0x20
#define SDE_HW_RC_REG9 0x24
#define SDE_HW_RC_REG10 0x28
#define SDE_HW_RC_REG11 0x2C
#define SDE_HW_RC_REG12 0x30
#define SDE_HW_RC_REG13 0x34

#define SDE_HW_RC_DATA_REG_SIZE  18
#define SDE_HW_RC_SKIP_DATA_PROG 0x1

#define SDE_HW_RC_DISABLE_R1 0x01E
#define SDE_HW_RC_DISABLE_R2 0x1E0

#define SDE_HW_RC_PU_SKIP_OP 0x1

#define RC_IDX(hw_dspp) hw_dspp->cap->sblk->rc.idx

enum rc_param_r {
	RC_PARAM_R0     = 0x0,
	RC_PARAM_R1     = 0x1,
	RC_PARAM_R2     = 0x2,
	RC_PARAM_R1R2   = (RC_PARAM_R1 | RC_PARAM_R2),
};

enum rc_param_a {
	RC_PARAM_A0     = 0x2,
	RC_PARAM_A1     = 0x4,
};

enum rc_param_b {
	RC_PARAM_B0     = 0x0,
	RC_PARAM_B1     = 0x1,
	RC_PARAM_B2     = 0x2,
	RC_PARAM_B1B2   = (RC_PARAM_B1 | RC_PARAM_B2),
};

enum rc_param_c {
	RC_PARAM_C0     = (BIT(8)),
	RC_PARAM_C1     = (BIT(10)),
	RC_PARAM_C2     = (BIT(10) | BIT(11)),
	RC_PARAM_C3     = (BIT(8) | BIT(10)),
	RC_PARAM_C4     = (BIT(8) | BIT(9)),
	RC_PARAM_C5     = (BIT(8) | BIT(9) | BIT(10) | BIT(11)),
};

enum rc_merge_mode {
	RC_MERGE_SINGLE_PIPE = 0x0,
	RC_MERGE_DUAL_PIPE   = 0x1
};

struct rc_config_table {
	enum rc_param_a param_a;
	enum rc_param_b param_b;
	enum rc_param_c param_c;
	enum rc_merge_mode merge_mode;
	enum rc_merge_mode merge_mode_en;
};

struct sde_ltm_phase_info {
	u32 init_h[LTM_MAX];
	u32 init_v;
	u32 inc_h;
	u32 inc_v;
	bool portrait_en;
	bool merge_en;
};

static inline void sde_ltm_get_phase_info(struct sde_hw_cp_cfg *hw_cfg,
		struct sde_ltm_phase_info *info)
{
	u32 count_v, count_h, num_mixers;

	if (hw_cfg->displayh < hw_cfg->displayv) {
		count_h = 4;
		count_v = 8;
		info->portrait_en = true;
	} else {
		count_h = 8;
		count_v = 4;
		info->portrait_en = false;
	}

	num_mixers = hw_cfg->num_of_mixers;
	if (num_mixers == 1)
		info->merge_en = false;
	else
		info->merge_en = true;

	info->init_h[LTM_0] = (1 << 23);
	info->init_h[LTM_1] = (1 << 23);
	info->init_h[LTM_2] = (1 << 23);
	info->init_h[LTM_3] = (1 << 23);
	info->init_v = (1 << 23);
	info->inc_h = ((count_h - 1) << 24) / (hw_cfg->displayh - 1);
	info->inc_v = ((count_v - 1) << 24) / (hw_cfg->displayv - 1);
	if (info->merge_en) {
		info->init_h[LTM_1] = info->init_h[LTM_0] +
			info->inc_h * (hw_cfg->displayh / 2);
		info->init_h[LTM_3] = info->init_h[LTM_2] +
			info->inc_h * (hw_cfg->displayh / 2);
	}
}

static struct rc_config_table config_table[] =  {
	/* RC_PARAM_A0 configurations */
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B0,
		.param_c = RC_PARAM_C5,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B1B2,
		.param_c = RC_PARAM_C3,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B1,
		.param_c = RC_PARAM_C0,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B2,
		.param_c = RC_PARAM_C1,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B0,
		.param_c = RC_PARAM_C5,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_DUAL_PIPE,

	},
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B1B2,
		.param_c = RC_PARAM_C3,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_DUAL_PIPE,

	},
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B1,
		.param_c = RC_PARAM_C0,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A0,
		.param_b = RC_PARAM_B2,
		.param_c = RC_PARAM_C1,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,
	},

	/* RC_PARAM_A1 configurations */
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B0,
		.param_c = RC_PARAM_C5,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B1B2,
		.param_c = RC_PARAM_C5,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B1,
		.param_c = RC_PARAM_C4,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B2,
		.param_c = RC_PARAM_C2,
		.merge_mode = RC_MERGE_SINGLE_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B0,
		.param_c = RC_PARAM_C5,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_DUAL_PIPE,

	},
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B1B2,
		.param_c = RC_PARAM_C5,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_DUAL_PIPE,

	},
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B1,
		.param_c = RC_PARAM_C4,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
	{
		.param_a = RC_PARAM_A1,
		.param_b = RC_PARAM_B2,
		.param_c = RC_PARAM_C2,
		.merge_mode = RC_MERGE_DUAL_PIPE,
		.merge_mode_en = RC_MERGE_SINGLE_PIPE,

	},
};

static inline int _sde_hw_rc_get_enable_bits(
		enum rc_param_a param_a,
		enum rc_param_b param_b,
		enum rc_param_c *param_c,
		u32 merge_mode,
		u32 *merge_mode_en)
{
	int i = 0;

	if (!param_c || !merge_mode_en) {
		DRM_ERROR("invalid arguments\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(config_table); i++) {
		if (merge_mode == config_table[i].merge_mode &&
				param_a == config_table[i].param_a &&
				param_b == config_table[i].param_b) {
			*param_c = config_table[i].param_c;
			*merge_mode_en = config_table[i].merge_mode_en;
			DRM_DEBUG("found param_c:0x%08X, merge_mode_en:%d\n",
					*param_c, *merge_mode_en);
			return 0;
		}
	}
	DRM_ERROR("configuration not supported");

	return -EINVAL;
}

static inline int _sde_hw_rc_get_merge_mode(
		const struct sde_hw_cp_cfg *hw_cfg,
		u32 *merge_mode)
{
	int rc = 0;

	if (!hw_cfg || !merge_mode) {
		DRM_ERROR("invalid arguments\n");
		return -EINVAL;
	}

	if (hw_cfg->num_of_mixers == 1)
		*merge_mode = RC_MERGE_SINGLE_PIPE;
	else if (hw_cfg->num_of_mixers == 2)
		*merge_mode = RC_MERGE_DUAL_PIPE;
	else {
		DRM_ERROR("invalid number of mixers:%d\n",
				hw_cfg->num_of_mixers);
		return -EINVAL;
	}

	DRM_DEBUG("number mixers:%u, merge mode:%u\n",
			hw_cfg->num_of_mixers, *merge_mode);

	return rc;
}

static inline int _sde_hw_rc_get_ajusted_roi(
		const struct sde_hw_cp_cfg *hw_cfg,
		const struct sde_rect *pu_roi,
		struct sde_rect *rc_roi)
{
	int rc = 0;

	if (!hw_cfg || !pu_roi || !rc_roi) {
		DRM_ERROR("invalid arguments\n");
		return -EINVAL;
	}

	/*when partial update is disabled, use full screen ROI*/
	if (pu_roi->w == 0 && pu_roi->h == 0) {
		rc_roi->x = pu_roi->x;
		rc_roi->y = pu_roi->y;
		rc_roi->w = hw_cfg->panel_width;
		rc_roi->h = hw_cfg->panel_height;
	} else {
		memcpy(rc_roi, pu_roi, sizeof(struct sde_rect));
	}

	SDE_EVT32(hw_cfg->displayh, hw_cfg->displayv, hw_cfg->panel_width, hw_cfg->panel_height);
	DRM_DEBUG("displayh:%u, displayv:%u, panel_w:%u, panel_h:%u\n", hw_cfg->displayh,
			hw_cfg->displayv, hw_cfg->panel_width, hw_cfg->panel_height);
	DRM_DEBUG("pu_roi x:%u, y:%u, w:%u, h:%u\n", pu_roi->x, pu_roi->y,
			pu_roi->w, pu_roi->h);
	DRM_DEBUG("rc_roi x:%u, y:%u, w:%u, h:%u\n", rc_roi->x, rc_roi->y,
			rc_roi->w, rc_roi->h);

	return rc;
}

static inline int _sde_hw_rc_get_param_rb(
		const struct drm_msm_rc_mask_cfg *rc_mask_cfg,
		const struct sde_rect *rc_roi,
		enum rc_param_r *param_r,
		enum rc_param_b *param_b)
{
	int rc = 0;
	int half_panel_x = 0, half_panel_w = 0;
	int cfg_param_01 = 0, cfg_param_02 = 0;
	int x1 = 0, x2 = 0, y1 = 0, y2 = 0;

	if (!rc_mask_cfg || !rc_roi || !param_r || !param_b) {
		DRM_ERROR("invalid arguments\n");
		return -EINVAL;
	}

	if (rc_mask_cfg->cfg_param_03 == RC_PARAM_A1)
		half_panel_w = rc_mask_cfg->cfg_param_04[0] +
				rc_mask_cfg->cfg_param_04[1];
	else if (rc_mask_cfg->cfg_param_03 == RC_PARAM_A0)
		half_panel_w = rc_mask_cfg->cfg_param_04[0];
	else {
		DRM_ERROR("invalid cfg_param_03:%u\n",
				rc_mask_cfg->cfg_param_03);
		return -EINVAL;
	}

	cfg_param_01 = rc_mask_cfg->cfg_param_01;
	cfg_param_02 = rc_mask_cfg->cfg_param_02;
	x1 = rc_roi->x;
	x2 = rc_roi->x + rc_roi->w - 1;
	y1 = rc_roi->y;
	y2 = rc_roi->y + rc_roi->h - 1;
	half_panel_x = half_panel_w - 1;

	DRM_DEBUG("x1:%u y1:%u x2:%u y2:%u\n", x1, y1, x2, y2);
	DRM_DEBUG("cfg_param_01:%u cfg_param_02:%u half_panel_x:%u",
			cfg_param_01, cfg_param_02, half_panel_x);

	if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || half_panel_x < 0 ||
			x1 >= x2 || y1 >= y2) {
		DRM_ERROR("invalid coordinates\n");
		return -EINVAL;
	}

	if (y1 <= cfg_param_01) {
		*param_r |= RC_PARAM_R1;
		if (x1 <= half_panel_x && x2 <= half_panel_x)
			*param_b |= RC_PARAM_B1;
		else if (x1 > half_panel_x && x2 > half_panel_x)
			*param_b |= RC_PARAM_B2;
		else
			*param_b |= RC_PARAM_B1B2;
	}

	if (y2 >= cfg_param_02) {
		*param_r |= RC_PARAM_R2;
		if (x1 <= half_panel_x && x2 <= half_panel_x)
			*param_b |= RC_PARAM_B1;
		else if (x1 > half_panel_x && x2 > half_panel_x)
			*param_b |= RC_PARAM_B2;
		else
			*param_b |= RC_PARAM_B1B2;
	}

	DRM_DEBUG("param_r:0x%08X param_b:0x%08X\n", *param_r, *param_b);
	SDE_EVT32(rc_roi->x, rc_roi->y, rc_roi->w, rc_roi->h);
	SDE_EVT32(x1, y1, x2, y2, cfg_param_01, cfg_param_02, half_panel_x);

	return rc;
}

#endif /* _SDE_HW_COLOR_PROC_COMMON_V4_H_ */
