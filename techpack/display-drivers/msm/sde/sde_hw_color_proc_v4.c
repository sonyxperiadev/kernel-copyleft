// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */
#include <drm/msm_drm_pp.h>
#include "sde_hw_color_proc_common_v4.h"
#include "sde_hw_color_proc_v4.h"
#include "sde_dbg.h"

#define DEMURAV1_CFG0_PARAM4_MASK 5
#define DEMURAV2_CFG0_PARAM4_MASK 7

static int sde_write_3d_gamut(struct sde_hw_blk_reg_map *hw,
		struct drm_msm_3d_gamut *payload, u32 base,
		u32 *opcode, u32 pipe, u32 scale_tbl_a_len,
		u32 scale_tbl_b_len)
{
	u32 reg, tbl_len, tbl_off, scale_off, i, j;
	u32 scale_tbl_len, scale_tbl_off;
	u32 *scale_data;

	if (!payload || !opcode || !hw) {
		DRM_ERROR("invalid payload %pK opcode %pK hw %pK\n",
			payload, opcode, hw);
		return -EINVAL;
	}

	switch (payload->mode) {
	case GAMUT_3D_MODE_17:
		tbl_len = GAMUT_3D_MODE17_TBL_SZ;
		tbl_off = 0;
		if (pipe == DSPP) {
			scale_off = GAMUT_SCALEA_OFFSET_OFF;
			*opcode = gamut_mode_17;
		} else {
			*opcode = (*opcode & (BIT(5) - 1)) >> 2;
			if (*opcode == gamut_mode_17b)
				*opcode = gamut_mode_17;
			else
				*opcode = gamut_mode_17b;
			scale_off = (*opcode == gamut_mode_17) ?
				GAMUT_SCALEA_OFFSET_OFF :
				GAMUT_SCALEB_OFFSET_OFF;
		}
		break;
	case GAMUT_3D_MODE_13:
		*opcode = (*opcode & (BIT(4) - 1)) >> 2;
		if (*opcode == gamut_mode_13a)
			*opcode = gamut_mode_13b;
		else
			*opcode = gamut_mode_13a;
		tbl_len = GAMUT_3D_MODE13_TBL_SZ;
		tbl_off = (*opcode == gamut_mode_13a) ? 0 :
			GAMUT_MODE_13B_OFF;
		scale_off = (*opcode == gamut_mode_13a) ?
			GAMUT_SCALEA_OFFSET_OFF : GAMUT_SCALEB_OFFSET_OFF;
		*opcode <<= 2;
		break;
	case GAMUT_3D_MODE_5:
		*opcode = gamut_mode_5 << 2;
		tbl_len = GAMUT_3D_MODE5_TBL_SZ;
		tbl_off = GAMUT_MODE_5_OFF;
		scale_off = GAMUT_SCALEB_OFFSET_OFF;
		break;
	default:
		DRM_ERROR("invalid mode %d\n", payload->mode);
		return -EINVAL;
	}

	if (payload->flags & GAMUT_3D_MAP_EN)
		*opcode |= GAMUT_MAP_EN;
	*opcode |= GAMUT_EN;

	for (i = 0; i < GAMUT_3D_TBL_NUM; i++) {
		reg = GAMUT_TABLE0_SEL << i;
		reg |= ((tbl_off) & (BIT(11) - 1));
		SDE_REG_WRITE(hw, base + GAMUT_TABLE_SEL_OFF, reg);
		for (j = 0; j < tbl_len; j++) {
			SDE_REG_WRITE(hw, base + GAMUT_LOWER_COLOR_OFF,
					payload->col[i][j].c2_c1);
			SDE_REG_WRITE(hw, base + GAMUT_UPPER_COLOR_OFF,
					payload->col[i][j].c0);
		}
	}

	if ((*opcode & GAMUT_MAP_EN)) {
		if (scale_off == GAMUT_SCALEA_OFFSET_OFF)
			scale_tbl_len = scale_tbl_a_len;
		else
			scale_tbl_len = scale_tbl_b_len;
		for (i = 0; i < GAMUT_3D_SCALE_OFF_TBL_NUM; i++) {
			scale_tbl_off = base + scale_off +
					i * scale_tbl_len * sizeof(u32);
			scale_data = &payload->scale_off[i][0];
			for (j = 0; j < scale_tbl_len; j++)
				SDE_REG_WRITE(hw,
					scale_tbl_off + (j * sizeof(u32)),
					scale_data[j]);
		}
	}
	SDE_REG_WRITE(hw, base, *opcode);
	return 0;
}

void sde_setup_dspp_3d_gamutv4(struct sde_hw_dspp *ctx, void *cfg)
{
	struct drm_msm_3d_gamut *payload;
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	u32 op_mode;

	if (!ctx || !cfg) {
		DRM_ERROR("invalid param ctx %pK cfg %pK\n", ctx, cfg);
		return;
	}

	op_mode = SDE_REG_READ(&ctx->hw, ctx->cap->sblk->gamut.base);
	if (!hw_cfg->payload) {
		DRM_DEBUG_DRIVER("disable gamut feature\n");
		SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->gamut.base, 0);
		return;
	}

	payload = hw_cfg->payload;
	sde_write_3d_gamut(&ctx->hw, payload, ctx->cap->sblk->gamut.base,
		&op_mode, DSPP, GAMUT_3D_SCALE_OFF_SZ, GAMUT_3D_SCALEB_OFF_SZ);

}

void sde_setup_dspp_3d_gamutv41(struct sde_hw_dspp *ctx, void *cfg)
{
	struct drm_msm_3d_gamut *payload;
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	u32 op_mode;

	if (!ctx || !cfg) {
		DRM_ERROR("invalid param ctx %pK cfg %pK\n", ctx, cfg);
		return;
	}

	op_mode = SDE_REG_READ(&ctx->hw, ctx->cap->sblk->gamut.base);
	if (!hw_cfg->payload) {
		DRM_DEBUG_DRIVER("disable gamut feature\n");
		SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->gamut.base, 0);
		return;
	}

	payload = hw_cfg->payload;
	sde_write_3d_gamut(&ctx->hw, payload, ctx->cap->sblk->gamut.base,
		&op_mode, DSPP, GAMUT_3D_SCALE_OFF_SZ, GAMUT_3D_SCALE_OFF_SZ);
}

void sde_setup_dspp_igcv3(struct sde_hw_dspp *ctx, void *cfg)
{
	struct drm_msm_igc_lut *lut_cfg;
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	int i = 0, j = 0;
	u32 *addr[IGC_TBL_NUM];
	u32 offset = 0;

	if (!ctx || !cfg) {
		DRM_ERROR("invalid param ctx %pK cfg %pK\n", ctx, cfg);
		return;
	}

	if (!hw_cfg->payload) {
		DRM_DEBUG_DRIVER("disable igc feature\n");
		SDE_REG_WRITE(&ctx->hw, IGC_OPMODE_OFF, 0);
		return;
	}

	if (hw_cfg->len != sizeof(struct drm_msm_igc_lut)) {
		DRM_ERROR("invalid size of payload len %d exp %zd\n",
				hw_cfg->len, sizeof(struct drm_msm_igc_lut));
		return;
	}

	lut_cfg = hw_cfg->payload;

	addr[0] = lut_cfg->c0;
	addr[1] = lut_cfg->c1;
	addr[2] = lut_cfg->c2;

	for (i = 0; i < IGC_TBL_NUM; i++) {
		offset = IGC_C0_OFF + (i * sizeof(u32));

		for (j = 0; j < IGC_TBL_LEN; j++) {
			addr[i][j] &= IGC_DATA_MASK;
			addr[i][j] |= IGC_DSPP_SEL_MASK(ctx->idx - 1);
			if (j == 0)
				addr[i][j] |= IGC_INDEX_UPDATE;
			/* IGC lut registers are part of DSPP Top HW block */
			SDE_REG_WRITE(&ctx->hw_top, offset, addr[i][j]);
		}
	}

	if (lut_cfg->flags & IGC_DITHER_ENABLE) {
		SDE_REG_WRITE(&ctx->hw, IGC_DITHER_OFF,
			lut_cfg->strength & IGC_DITHER_DATA_MASK);
	}

	SDE_REG_WRITE(&ctx->hw, IGC_OPMODE_OFF, IGC_EN);
}

void sde_setup_dspp_pccv4(struct sde_hw_dspp *ctx, void *cfg)
{
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	struct drm_msm_pcc *pcc_cfg;
	struct drm_msm_pcc_coeff *coeffs = NULL;
	int i = 0;
	u32 base = 0;

	if (!ctx || !cfg) {
		DRM_ERROR("invalid param ctx %pK cfg %pK\n", ctx, cfg);
		return;
	}

	if (!hw_cfg->payload) {
		DRM_DEBUG_DRIVER("disable pcc feature\n");
		SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->pcc.base, 0);
		return;
	}

	if (hw_cfg->len != sizeof(struct drm_msm_pcc)) {
		DRM_ERROR("invalid size of payload len %d exp %zd\n",
				hw_cfg->len, sizeof(struct drm_msm_pcc));
		return;
	}

	pcc_cfg = hw_cfg->payload;

	for (i = 0; i < PCC_NUM_PLANES; i++) {
		base = ctx->cap->sblk->pcc.base + (i * sizeof(u32));
		switch (i) {
		case 0:
			coeffs = &pcc_cfg->r;
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_RR_OFF, pcc_cfg->r_rr);
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_GG_OFF, pcc_cfg->r_gg);
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_BB_OFF, pcc_cfg->r_bb);
			break;
		case 1:
			coeffs = &pcc_cfg->g;
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_RR_OFF, pcc_cfg->g_rr);
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_GG_OFF, pcc_cfg->g_gg);
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_BB_OFF, pcc_cfg->g_bb);
			break;
		case 2:
			coeffs = &pcc_cfg->b;
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_RR_OFF, pcc_cfg->b_rr);
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_GG_OFF, pcc_cfg->b_gg);
			SDE_REG_WRITE(&ctx->hw,
				base + PCC_BB_OFF, pcc_cfg->b_bb);
			break;
		default:
			DRM_ERROR("invalid pcc plane: %d\n", i);
			return;
		}

		SDE_REG_WRITE(&ctx->hw, base + PCC_C_OFF, coeffs->c);
		SDE_REG_WRITE(&ctx->hw, base + PCC_R_OFF, coeffs->r);
		SDE_REG_WRITE(&ctx->hw, base + PCC_G_OFF, coeffs->g);
		SDE_REG_WRITE(&ctx->hw, base + PCC_B_OFF, coeffs->b);
		SDE_REG_WRITE(&ctx->hw, base + PCC_RG_OFF, coeffs->rg);
		SDE_REG_WRITE(&ctx->hw, base + PCC_RB_OFF, coeffs->rb);
		SDE_REG_WRITE(&ctx->hw, base + PCC_GB_OFF, coeffs->gb);
		SDE_REG_WRITE(&ctx->hw, base + PCC_RGB_OFF, coeffs->rgb);
	}

	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->pcc.base, PCC_EN);
}

void sde_setup_dspp_ltm_threshv1(struct sde_hw_dspp *ctx, void *cfg)
{
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	u64 thresh = 0;

	if (!ctx || !cfg) {
		DRM_ERROR("invalid parameters ctx %pK cfg %pK\n", ctx, cfg);
		return;
	}

	if (!hw_cfg->payload) {
		DRM_DEBUG_DRIVER("Disable LTM noise thresh feature\n");
		SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x60, 0);
		return;
	}

	thresh = *((u64 *)hw_cfg->payload);

	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x60,
			(thresh & 0x3FF));
}

void sde_setup_dspp_ltm_hist_bufferv1(struct sde_hw_dspp *ctx, u64 addr)
{
	struct drm_msm_ltm_stats_data *hist = NULL;
	u64 lh_addr, hs_addr;

	if (!ctx || !addr) {
		DRM_ERROR("invalid parameter ctx %pK addr 0x%llx\n", ctx, addr);
		return;
	}

	hist = (struct drm_msm_ltm_stats_data *)addr;
	lh_addr = (u64)(&hist->stats_02[0]);
	hs_addr = (u64)(&hist->stats_03[0]);
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x70,
			(addr & 0xFFFFFF00));
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x74,
			(lh_addr & 0xFFFFFF00));
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x78,
			(hs_addr & 0xFFFFFF00));
}

static void sde_setup_dspp_ltm_hist_ctrl_common(struct sde_hw_dspp *ctx,
					u64 addr, u32 op_mode,
					struct sde_ltm_phase_info *phase)
{
	u32 offset;

	if (ctx->idx >= DSPP_MAX) {
		DRM_ERROR("Invalid idx %d\n", ctx->idx);
		return;
	}

	if (phase->portrait_en)
		op_mode |= BIT(2);
	else
		op_mode &= ~BIT(2);

	offset = ctx->cap->sblk->ltm.base + 0x8;
	SDE_REG_WRITE(&ctx->hw, offset, (phase->init_h[ctx->idx] & 0x7FFFFFF));
	offset += 4;
	SDE_REG_WRITE(&ctx->hw, offset, (phase->init_v & 0xFFFFFF));
	offset += 4;
	SDE_REG_WRITE(&ctx->hw, offset, (phase->inc_h & 0xFFFFFF));
	offset += 4;
	SDE_REG_WRITE(&ctx->hw, offset, (phase->inc_v & 0xFFFFFF));

	op_mode |= BIT(0);
	sde_setup_dspp_ltm_hist_bufferv1(ctx, addr);

	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x4,
			(op_mode & 0x1FFFFFF));
}

void sde_setup_dspp_ltm_hist_ctrlv1(struct sde_hw_dspp *ctx, void *cfg,
					bool enable, u64 addr)
{
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	struct sde_ltm_phase_info phase;
	u32 op_mode, offset;

	if (!ctx) {
		DRM_ERROR("invalid parameters ctx %pK\n", ctx);
		return;
	}

	if (enable && (!addr || !cfg)) {
		DRM_ERROR("invalid addr 0x%llx cfg %pK\n", addr, cfg);
		return;
	}

	offset = ctx->cap->sblk->ltm.base + 0x4;
	op_mode = SDE_REG_READ(&ctx->hw, offset);

	if (!enable) {
		if (op_mode & BIT(1))
			op_mode &= ~BIT(0);
		else
			op_mode &= LTM_CONFIG_MERGE_MODE_ONLY;

		SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x4,
			(op_mode & 0x1FFFFFF));
		return;
	}

	memset(&phase, 0, sizeof(phase));
	sde_ltm_get_phase_info(hw_cfg, &phase);
	if (phase.merge_en)
		op_mode |= BIT(16);
	else
		op_mode &= ~LTM_CONFIG_MERGE_MODE_ONLY;

	sde_setup_dspp_ltm_hist_ctrl_common(ctx, addr, op_mode, &phase);
}

void sde_setup_dspp_ltm_hist_ctrlv1_2(struct sde_hw_dspp *ctx, void *cfg,
					bool enable, u64 addr)
{
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	struct sde_ltm_phase_info phase;
	u32 op_mode, offset;
	u32 merge_mode = 0;

	if (!ctx) {
		DRM_ERROR("invalid parameters ctx %pK\n", ctx);
		return;
	}

	if (enable && (!addr || !cfg)) {
		DRM_ERROR("invalid addr 0x%llx cfg %pK\n", addr, cfg);
		return;
	}

	offset = ctx->cap->sblk->ltm.base + 0x4;
	op_mode = SDE_REG_READ(&ctx->hw, offset);
	if (!enable) {
		if (op_mode & BIT(1))
			op_mode &= ~BIT(0);
		else
			op_mode = 0x0;

		SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x4,
			(op_mode & 0x1FFFFFF));
		return;
	}

	memset(&phase, 0, sizeof(phase));
	sde_ltm_get_phase_info(hw_cfg, &phase);

	offset = ctx->cap->sblk->ltm.base + 0x18;
	if (phase.merge_en)
		merge_mode = BIT(0);
	else
		merge_mode = 0x0;
	SDE_REG_WRITE(&ctx->hw, offset, (merge_mode & 0x3));

	sde_setup_dspp_ltm_hist_ctrl_common(ctx, addr, op_mode, &phase);
}

void sde_ltm_read_intr_status(struct sde_hw_dspp *ctx, u32 *status)
{
	u32 clear;

	if (!ctx || !status) {
		DRM_ERROR("invalid parameters ctx %pK status %pK\n", ctx,
				status);
		return;
	}

	*status = SDE_REG_READ(&ctx->hw, ctx->cap->sblk->ltm.base + 0x54);
	pr_debug("%s(): LTM interrupt status 0x%x\n", __func__, *status);
	/* clear the hist_sat and hist_merge_sat bits */
	clear = SDE_REG_READ(&ctx->hw, ctx->cap->sblk->ltm.base + 0x58);
	clear |= BIT(1) | BIT(2);
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x58, clear);
}

void sde_ltm_clear_merge_modev1_2(struct sde_hw_dspp *ctx)
{
	if (!ctx) {
		DRM_ERROR("invalid parameters ctx %pK\n", ctx);
		return;
	}

	/* clear the merge_mode bit */
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x18, 0x0);
}

void sde_ltm_clear_merge_mode(struct sde_hw_dspp *ctx)
{
	u32 clear;

	if (!ctx) {
		DRM_ERROR("invalid parameters ctx %pK\n", ctx);
		return;
	}

	/* clear the merge_mode bit */
	clear = SDE_REG_READ(&ctx->hw, ctx->cap->sblk->ltm.base + 0x04);
	clear &= ~LTM_CONFIG_MERGE_MODE_ONLY;
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->ltm.base + 0x04, clear);
}

void sde_demura_backlight_cfg(struct sde_hw_dspp *ctx, u64 val, struct sde_hw_cp_cfg *hw_cfg)
{
	u32 demura_base;
	u32 backlight, gain[2];
	struct drm_msm_dem_cfg *dcfg = NULL;
	uint32_t mask_bits = DEMURAV1_CFG0_PARAM4_MASK;

	if (!ctx) {
		DRM_ERROR("invalid parameter ctx %pK", ctx);
		return;
	}

	if (!hw_cfg) {
		DRM_ERROR("invalid parameter hw_cfg");
		return;
	}

	if (!hw_cfg->payload) {
		DRM_DEBUG_DRIVER("disable demura feature\n");
		return;
	}

	dcfg = (struct drm_msm_dem_cfg *)hw_cfg->payload;
	demura_base = ctx->cap->sblk->demura.base;
	backlight = (val & REG_MASK(32));
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->demura.base + 0x8, backlight);

	backlight = backlight & REG_MASK(16);
	if (backlight <= 41) {
		/* Gain0.....Gain7
		* {0, 0, 0, 3, 2, 2, 1, 1}
		*/
		gain[0] = (0x0 & 0x3f) | ((0x0 & 0x3f) << 8) |
			  ((0x0 & 0x3f) << 16) | ((0x3 & 0x3f) << 24);
		gain[1] = (0x2 & 0x3f) | ((0x2 & 0x3f) << 8) |
			  ((0x1 & 0x3f) << 16) | ((0x1 & 0x3f) << 24);
	} else if (backlight <= 164) {
		/* Gain0.....Gain7
		* {0, 0, 0, 2, 1, 1, 2, 3}
		*/
		gain[0] = (0x0 & 0x3f) | ((0x0 & 0x3f) << 8) |
			  ((0x0 & 0x3f) << 16) | ((0x2 & 0x3f) << 24);
		gain[1] = (0x1 & 0x3f) | ((0x1 & 0x3f) << 8) |
			  ((0x2 & 0x3f) << 16) | ((0x3 & 0x3f) << 24);
	} else {
		/* check for demura v2 and assign mask bit accordingly */
		if (ctx->cap->sblk->demura.version == SDE_COLOR_PROCESS_VER(0x2, 0x0))
			mask_bits = DEMURAV2_CFG0_PARAM4_MASK;

		gain[0] = (dcfg->cfg0_param4[0] & REG_MASK(mask_bits)) |
			((dcfg->cfg0_param4[1] & REG_MASK(mask_bits)) << 8) |
			  ((dcfg->cfg0_param4[2] & REG_MASK(mask_bits)) << 16) |
			  ((dcfg->cfg0_param4[3] & REG_MASK(mask_bits)) << 24);
		gain[1] = (dcfg->cfg0_param4[4] & REG_MASK(mask_bits)) |
			((dcfg->cfg0_param4[5] & REG_MASK(mask_bits)) << 8) |
			  ((dcfg->cfg0_param4[6] & REG_MASK(mask_bits)) << 16) |
			  ((dcfg->cfg0_param4[7] & REG_MASK(mask_bits)) << 24);
	}
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->demura.base + 0x4c, gain[0]);
	SDE_REG_WRITE(&ctx->hw, ctx->cap->sblk->demura.base + 0x50, gain[1]);
}

void sde_setup_fp16_cscv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	struct drm_msm_fp16_csc *fp16_csc;
	u32 csc_base, csc, i, offset = 0;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		csc_base = ctx->cap->sblk->fp16_csc_blk[0].base;
	else
		csc_base = ctx->cap->sblk->fp16_csc_blk[1].base;

	if (!csc_base) {
		DRM_ERROR("invalid offset for FP16 CSC CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	fp16_csc = (struct drm_msm_fp16_csc *)(hw_cfg->payload);
	if (!fp16_csc)
		goto write_base;

	if (hw_cfg->len != sizeof(struct drm_msm_fp16_csc) ||
			!hw_cfg->payload) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d\tpayload: %pK\n",
				ctx->idx, index, hw_cfg->len, hw_cfg->payload);
		return;
	}

	if (fp16_csc->cfg_param_0_len != FP16_CSC_CFG0_PARAM_LEN) {
		DRM_ERROR("invalid param 0 length! Got: %d\tExpected: %d\tpipe: %d\tindex: %d\n",
				fp16_csc->cfg_param_0_len, FP16_CSC_CFG0_PARAM_LEN,
				ctx->idx, index);
		return;
	} else if (fp16_csc->cfg_param_1_len !=  FP16_CSC_CFG1_PARAM_LEN) {
		DRM_ERROR("invalid param 1 length! Got: %d\tExpected: %d\tpipe: %d\tindex: %d\n",
				fp16_csc->cfg_param_1_len, FP16_CSC_CFG1_PARAM_LEN,
				ctx->idx, index);
		return;
	}

	for (i = 0; i < (fp16_csc->cfg_param_0_len / 2); i++) {
		offset += 0x4;
		csc = fp16_csc->cfg_param_0[2 * i] & 0xFFFF;
		csc |= (fp16_csc->cfg_param_0[2 * i + 1] & 0xFFFF) << 16;
		SDE_REG_WRITE(&ctx->hw, csc_base + offset, csc);
	}
	for (i = 0; i < (fp16_csc->cfg_param_1_len / 2); i++) {
		offset += 0x4;
		csc = fp16_csc->cfg_param_1[2 * i] & 0xFFFF;
		csc |= (fp16_csc->cfg_param_1[2 * i + 1] & 0xFFFF) << 16;
		SDE_REG_WRITE(&ctx->hw, csc_base + offset, csc);
	}

write_base:
	csc = SDE_REG_READ(&ctx->hw, csc_base);
	if (fp16_csc)
		csc |= BIT(2);
	else
		csc &= ~BIT(2);

	SDE_REG_WRITE(&ctx->hw, csc_base, csc);
}

void sde_setup_fp16_gcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	struct drm_msm_fp16_gc *fp16_gc;
	u32 gc_base, gc;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	}

	fp16_gc = (struct drm_msm_fp16_gc *)(hw_cfg->payload);
	if (fp16_gc && (hw_cfg->len != sizeof(struct drm_msm_fp16_gc) ||
			fp16_gc->mode == FP16_GC_MODE_INVALID)) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d\tmode: %d",
				ctx->idx, index, hw_cfg->len, fp16_gc->mode);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		gc_base = ctx->cap->sblk->fp16_gc_blk[0].base;
	else
		gc_base = ctx->cap->sblk->fp16_gc_blk[1].base;

	if (!gc_base) {
		DRM_ERROR("invalid offset for FP16 GC CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	gc = SDE_REG_READ(&ctx->hw, gc_base);
	gc &= ~0xF8;

	if (fp16_gc) {
		gc |= BIT(4);
		if (fp16_gc->flags & FP16_GC_FLAG_ALPHA_EN)
			gc |= BIT(3);

		if (fp16_gc->mode == FP16_GC_MODE_PQ)
			gc |= BIT(5);
	}

	SDE_REG_WRITE(&ctx->hw, gc_base, gc);
}

void sde_setup_fp16_igcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	bool *fp16_igc;
	u32 igc_base, igc;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	} else if (hw_cfg->len != sizeof(bool)) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d",
				ctx->idx, index, hw_cfg->len);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		igc_base = ctx->cap->sblk->fp16_igc_blk[0].base;
	else
		igc_base = ctx->cap->sblk->fp16_igc_blk[1].base;

	if (!igc_base) {
		DRM_ERROR("invalid offset for FP16 IGC CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	igc = SDE_REG_READ(&ctx->hw, igc_base);
	fp16_igc = (bool *)(hw_cfg->payload);

	if (fp16_igc && *fp16_igc)
		igc |= BIT(1);
	else
		igc &= ~BIT(1);

	SDE_REG_WRITE(&ctx->hw, igc_base, igc);
}

void sde_setup_fp16_unmultv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	bool *fp16_unmult;
	u32 unmult_base, unmult;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	} else if (hw_cfg->len != sizeof(bool)) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d",
				ctx->idx, index, hw_cfg->len);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		unmult_base = ctx->cap->sblk->fp16_unmult_blk[0].base;
	else
		unmult_base = ctx->cap->sblk->fp16_unmult_blk[1].base;

	if (!unmult_base) {
		DRM_ERROR("invalid offset for FP16 UNMULT CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	unmult = SDE_REG_READ(&ctx->hw, unmult_base);
	fp16_unmult = (bool *)(hw_cfg->payload);

	if (fp16_unmult && *fp16_unmult)
		unmult |= BIT(0);
	else
		unmult &= ~BIT(0);

	SDE_REG_WRITE(&ctx->hw, unmult_base, unmult);
}

void sde_demura_read_plane_status(struct sde_hw_dspp *ctx, u32 *status)
{
	u32 demura_base;
	u32 value;

	if (!ctx) {
		DRM_ERROR("invalid parameter ctx %pK", ctx);
		return;
	}

	demura_base = ctx->cap->sblk->demura.base;
	value = SDE_REG_READ(&ctx->hw, demura_base + 0x4);
	if (!(value & 0x4)) {
		*status = DEM_FETCH_DMA_INVALID;
	} else if (ctx->idx == DSPP_0 || ctx->idx == DSPP_2) {
		if (value & 0x80000000)
			*status = DEM_FETCH_DMA1_RECT0;
		else
			*status = DEM_FETCH_DMA3_RECT0;
	} else if (ctx->idx == DSPP_1 || ctx->idx == DSPP_3) {
		if (value & 0x80000000)
			*status = DEM_FETCH_DMA1_RECT1;
		else
			*status = DEM_FETCH_DMA3_RECT1;
	}
}

void sde_demura_pu_cfg(struct sde_hw_dspp *dspp, void *cfg)
{
	u32 demura_base;
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	struct msm_roi_list *roi_list = NULL;
	u32 temp;

	if (!dspp) {
		DRM_ERROR("invalid parameter ctx %pK", dspp);
		return;
	}
	demura_base = dspp->cap->sblk->demura.base;
	if (!cfg || !hw_cfg->payload) {
		temp = 0;
	} else {
		roi_list = hw_cfg->payload;
		if (hw_cfg->panel_width < hw_cfg->panel_height)
			temp = (16 * (1 << 21)) / hw_cfg->panel_height;
		else
			temp = (8 * (1 << 21)) / hw_cfg->panel_height;
		temp = temp * (roi_list->roi[0].y1);
	}
	SDE_REG_WRITE(&dspp->hw, dspp->cap->sblk->demura.base + 0x60,
			temp);

	SDE_EVT32(0x60, temp, dspp->idx, ((roi_list) ? roi_list->roi[0].y1 : -1),
			((roi_list) ? roi_list->roi[0].y2 : -1),
			((hw_cfg) ? hw_cfg->panel_height : -1));
}

int sde_spr_check_init_cfg(struct sde_hw_dspp *ctx, void *cfg)
{
	struct drm_msm_spr_init_cfg *spr_payload;
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	int rc = 0;

	if (!ctx || !cfg)
		return -EINVAL;

	if (!hw_cfg->payload)
		return 0;

	spr_payload = hw_cfg->payload;

	if (spr_payload->cfg18_en) {
		bool invalid = spr_payload->cfg11[0] != 1 || spr_payload->cfg11[1] != 0 ||
			       spr_payload->cfg11[2] != 1;

		if (spr_payload->cfg4 || invalid) {
			DRM_ERROR("CFG18 can't be enabled with this config. CFG4: %u CFG11: %u\n",
				spr_payload->cfg4, invalid);
			return -EINVAL;
		}
	}

	return rc;
}

int sde_spr_check_udc_cfg(struct sde_hw_dspp *ctx, void *cfg)
{
	uint32_t j = 0, i = 0;
	struct drm_msm_spr_udc_cfg *spr_payload;
	struct sde_hw_cp_cfg *hw_cfg = cfg;
	bool invalid = false;

	if (!ctx || !cfg)
		return -EINVAL;

	if (!hw_cfg->payload)
		return 0;

	spr_payload = hw_cfg->payload;
	invalid = !(spr_payload->init_cfg11[0] || spr_payload->init_cfg11[1] ||
			spr_payload->init_cfg11[2]);

	if (spr_payload->init_cfg4 || invalid) {
		DRM_ERROR("SPR UDC can not be used with this config. CFG4: %u CFG11: %u",
			spr_payload->init_cfg4, invalid);
		return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		uint32_t max_h, max_v, limit;
		uint32_t cfg_1_start = (SPR_UDC_PARAM_SIZE_2 / 3) * i;
		uint32_t x1 =
			spr_payload->cfg1[(SPR_UDC_PARAM_SIZE_1 / 3) * i] & 0xffff;
		uint32_t y1 =
			spr_payload->cfg1[(SPR_UDC_PARAM_SIZE_1 / 3) * i + 1] & 0xffff;
		uint32_t w = spr_payload->cfg1[(SPR_UDC_PARAM_SIZE_1 / 3) * i + 2];
		uint32_t lines = spr_payload->cfg1[(SPR_UDC_PARAM_SIZE_1 / 3) * i + 3];
		uint32_t x2 = x1 + w;
		uint32_t y2 = y1 + lines;

		switch (spr_payload->init_cfg11[i]) {
		case 0:
			max_h = hw_cfg->panel_width;
			max_v = hw_cfg->panel_height;
			break;
		case 1:
			max_h = (hw_cfg->panel_width + 1) >> 1;
			max_v = hw_cfg->panel_height;
			break;
		case 2:
			max_h = (hw_cfg->panel_width * 2 + 2) / 3;
			max_v = hw_cfg->panel_height;
			break;
		default:
			DRM_ERROR("Can't calculate decimation");
			return -EINVAL;
		}

		if (x2 >= max_h) {
			DRM_ERROR("Invalid CFG1 - C%u x2 %u", i, x2);
			return -EINVAL;
		} else if (y2 >= max_v) {
			DRM_ERROR("Invalid CFG1 - C%u y2 %u", i, y2);
			return -EINVAL;
		} else if (lines < 2 || lines > 256) {
			DRM_ERROR("Invalid CFG1 - C%u Lines %u", i, lines);
			return -EINVAL;
		} else if (w < 2 || w > 512) {
			DRM_ERROR("Invalid CFG1 - C%u W %u", i, w);
			return -EINVAL;
		}

		j = 0;
		limit = (w >> 1) + 1;
		for (j = 0; j < lines; j++) {
			uint32_t o1 = spr_payload->cfg2[cfg_1_start + (j*2)];
			uint32_t o2 = spr_payload->cfg2[cfg_1_start + (j*2) + 1];


			if (o1 > limit) {
				DRM_ERROR("Invalid CFG2 - C%u L%u, o1 exceeds limits",
						i, j);
				return -EINVAL;
			} else if (o2 > limit) {
				DRM_ERROR("Invalid CFG2 - C%u L%u, o2 exceeds limits",
						i, j);
				return -EINVAL;
			}
		}
	}

	return 0;
}

int sde_spr_read_opr_value(struct sde_hw_dspp *ctx, uint32_t *opr_value)
{
	uint32_t reg_off;

	if (!ctx || !opr_value)
		return -EINVAL;

	reg_off = ctx->cap->sblk->spr.base + 0x78;

	*opr_value = SDE_REG_READ(&ctx->hw, reg_off);

	return 0;
}

void sde_setup_ucsc_cscv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	drm_msm_ucsc_csc *ucsc_csc;
	u32 csc_base, csc, i, offset = 0;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		csc_base = ctx->cap->sblk->ucsc_csc_blk[0].base;
	else
		csc_base = ctx->cap->sblk->ucsc_csc_blk[1].base;

	if (!csc_base) {
		DRM_ERROR("invalid offset for UCSC CSC CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	ucsc_csc = (drm_msm_ucsc_csc *)(hw_cfg->payload);
	if (!ucsc_csc)
		goto write_base;

	if (hw_cfg->len != sizeof(drm_msm_ucsc_csc) ||
			!hw_cfg->payload) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d\tpayload: %pK\n",
				ctx->idx, index, hw_cfg->len, hw_cfg->payload);
		return;
	}

	if (ucsc_csc->cfg_param_0_len != UCSC_CSC_CFG0_PARAM_LEN) {
		DRM_ERROR("invalid param 0 length! Got: %d\tExpected: %d\tpipe: %d\tindex: %d\n",
				ucsc_csc->cfg_param_0_len, UCSC_CSC_CFG0_PARAM_LEN,
				ctx->idx, index);
		return;
	} else if (ucsc_csc->cfg_param_1_len !=  UCSC_CSC_CFG1_PARAM_LEN) {
		DRM_ERROR("invalid param 1 length! Got: %d\tExpected: %d\tpipe: %d\tindex: %d\n",
				ucsc_csc->cfg_param_1_len, UCSC_CSC_CFG1_PARAM_LEN,
				ctx->idx, index);
		return;
	}

	for (i = 0; i < (ucsc_csc->cfg_param_0_len / 2); i++) {
		offset += 0x4;
		csc = ucsc_csc->cfg_param_0[2 * i] & 0xFFFF;
		csc |= (ucsc_csc->cfg_param_0[2 * i + 1] & 0xFFFF) << 16;
		SDE_REG_WRITE(&ctx->hw, csc_base + offset, csc);
	}
	for (i = 0; i < (ucsc_csc->cfg_param_1_len / 2); i++) {
		offset += 0x4;
		csc = ucsc_csc->cfg_param_1[2 * i] & 0xFFFF;
		csc |= (ucsc_csc->cfg_param_1[2 * i + 1] & 0xFFFF) << 16;
		SDE_REG_WRITE(&ctx->hw, csc_base + offset, csc);
	}

write_base:
	csc = SDE_REG_READ(&ctx->hw, csc_base);
	if (ucsc_csc)
		csc |= BIT(2);
	else
		csc &= ~BIT(2);

	SDE_REG_WRITE(&ctx->hw, csc_base, csc);
}

void sde_setup_ucsc_gcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	int *ucsc_gc;
	u32 gc_base, gc;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		gc_base = ctx->cap->sblk->ucsc_gc_blk[0].base;
	else
		gc_base = ctx->cap->sblk->ucsc_gc_blk[1].base;

	if (!gc_base) {
		DRM_ERROR("invalid offset for UCSC GC CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	ucsc_gc = (int *)(hw_cfg->payload);

	if (!ucsc_gc || (hw_cfg->len != sizeof(int))) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d\tpayload: %pK\n",
				ctx->idx, index, hw_cfg->len, ucsc_gc);
		return;
	}

	gc = SDE_REG_READ(&ctx->hw, gc_base);
	gc &= 0x60707;

	if (*ucsc_gc == UCSC_GC_MODE_DISABLE)
		goto reset_gc;

	if (ucsc_gc && *ucsc_gc) {
		gc |= BIT(4);

		switch (*ucsc_gc)
		{
		case UCSC_GC_MODE_SRGB:
			break;
		case UCSC_GC_MODE_PQ:
			gc |= BIT(5);
			break;
		case UCSC_GC_MODE_GAMMA2_2:
			gc |= BIT(6);
			break;
		case UCSC_GC_MODE_HLG:
			gc |= BIT(5)|BIT(6);
			break;
		default:
			DRM_ERROR("Invalid UCSC GC mode \tmode: %d\n", *ucsc_gc);
			return;
		}
	}

reset_gc:
	SDE_REG_WRITE(&ctx->hw, gc_base, gc);
}

void sde_setup_ucsc_igcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	int *ucsc_igc;
	u32 igc_base, igc;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		igc_base = ctx->cap->sblk->ucsc_igc_blk[0].base;
	else
		igc_base = ctx->cap->sblk->ucsc_igc_blk[1].base;

	if (!igc_base) {
		DRM_ERROR("invalid offset for UCSC GC CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	ucsc_igc = (int *)(hw_cfg->payload);

	if (!ucsc_igc || (hw_cfg->len != sizeof(int))) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d\tpayload: %pK\n",
				ctx->idx, index, hw_cfg->len, ucsc_igc);
		return;
	}

	igc = SDE_REG_READ(&ctx->hw, igc_base);
	igc &= 0x600FD;

	if (*ucsc_igc == UCSC_IGC_MODE_DISABLE)
		goto reset_igc;

	if (ucsc_igc && *ucsc_igc) {
		igc |= BIT(1);

		switch (*ucsc_igc)
		{
		case UCSC_IGC_MODE_SRGB:
			break;
		case UCSC_IGC_MODE_REC709:
			igc |= BIT(8);
			break;
		case UCSC_IGC_MODE_GAMMA2_2:
			igc |= BIT(9);
			break;
		case UCSC_IGC_MODE_HLG:
			igc |= BIT(8)|BIT(9);
			break;
		case UCSC_IGC_MODE_PQ:
			igc |= BIT(10);
			break;
		default:
		    DRM_ERROR("Invalid UCSC IGC mode \tmode: %d\n", *ucsc_igc);
			return;
		}
	}

reset_igc:
	SDE_REG_WRITE(&ctx->hw, igc_base, igc);
}

void sde_setup_ucsc_unmultv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	bool *ucsc_unmult;
	u32 unmult_base, unmult;

	if (!ctx || !data || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tdata: %pK\tindex: %d\n",
				ctx, data, index);
		return;
	} else if (hw_cfg->len != sizeof(bool) || !hw_cfg->payload) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d\tpayload: %pK\n",
				  ctx->idx, index, hw_cfg->len, hw_cfg->payload);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		unmult_base = ctx->cap->sblk->ucsc_unmult_blk[0].base;
	else
		unmult_base = ctx->cap->sblk->ucsc_unmult_blk[1].base;

	if (!unmult_base) {
		DRM_ERROR("invalid offset for UCSC UNMULT CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	unmult = SDE_REG_READ(&ctx->hw, unmult_base);
	ucsc_unmult = (bool *)(hw_cfg->payload);

	if (ucsc_unmult && *ucsc_unmult)
		unmult |= BIT(0)|BIT(18);
	else
		unmult &= ~(BIT(0)|BIT(18));

	SDE_REG_WRITE(&ctx->hw, unmult_base, unmult);
}

void sde_setup_ucsc_alpha_ditherv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data)
{
	struct sde_hw_cp_cfg *hw_cfg = data;
	bool *ucsc_alpha_dither;
	u32 alpha_dither_base, alpha_dither;

	if (!ctx || index == SDE_SSPP_RECT_MAX) {
		DRM_ERROR("invalid parameter\tctx: %pK\tindex: %d\n",
				ctx, index);
		return;
	} else if (hw_cfg->len != sizeof(bool) || !hw_cfg->payload) {
		DRM_ERROR("invalid hw_cfg payload\tpipe: %d\tindex: %d\tlen: %d\tpayload: %pK\n",
				  ctx->idx, index, hw_cfg->len, hw_cfg->payload);
		return;
	}

	if (index == SDE_SSPP_RECT_SOLO || index == SDE_SSPP_RECT_0)
		alpha_dither_base = ctx->cap->sblk->ucsc_alpha_dither_blk[0].base;
	else
		alpha_dither_base = ctx->cap->sblk->ucsc_alpha_dither_blk[1].base;

	if (!alpha_dither_base) {
		DRM_ERROR("invalid offset for UCSC ALPHA DITHER CP block\tpipe: %d\tindex: %d\n",
				ctx->idx, index);
		return;
	}

	alpha_dither = SDE_REG_READ(&ctx->hw, alpha_dither_base);
	ucsc_alpha_dither = (bool *)(hw_cfg->payload);

	if (ucsc_alpha_dither && *ucsc_alpha_dither)
		alpha_dither |= BIT(17);
	else
		alpha_dither &= ~BIT(17);

	SDE_REG_WRITE(&ctx->hw, alpha_dither_base, alpha_dither);
}
