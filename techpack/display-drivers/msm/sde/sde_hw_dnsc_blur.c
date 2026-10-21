// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#include "sde_hw_mdss.h"
#include "sde_hwio.h"
#include "sde_hw_catalog.h"
#include "sde_hw_dnsc_blur.h"
#include "sde_dbg.h"
#include "sde_kms.h"

#define DNSC_BLUR_OP_MODE		0x0
#define DNSC_BLUR_BLUR_RATIO_H		0x4
#define DNSC_BLUR_BLUR_RATIO_V		0x8
#define DNSC_BLUR_PCMN_PHASE_INIT_H	0xC
#define DNSC_BLUR_PCMN_PHASE_STEP_H	0x10
#define DNSC_BLUR_PCMN_PHASE_INIT_V	0x14
#define DNSC_BLUR_PCMN_PHASE_STEP_V	0x18
#define DNSC_BLUR_OUT_IMG_SIZE		0x1C
#define DNSC_BLUR_GAUS_COEF_LUT_SEL	0x20
#define DNSC_BLUR_MUX			0x24
#define DNSC_BLUR_SRC_IMG_SIZE		0x28

#define DNSC_BLUR_GAUS_COEF_LUT_H0	0x0
#define DNSC_BLUR_GAUS_COEF_LUT_V0	0x100
#define DNSC_BLUR_GAUS_COEF_LUT_H1	0x200
#define DNSC_BLUR_GAUS_COEF_LUT_V1	0x300

#define DNSC_BLUR_DITHER_OP_MODE	0x0
#define DNSC_BLUR_DITHER_BITDEPTH	0x4
#define DNSC_BLUR_DITHER_MATRIX_ROW0	0x8

/* DNSC_BLUR_OP_MODE bits */
#define DNSC_BLUR_OPMODE_ENABLE		BIT(0)
#define DNSC_BLUR_OPMODE_DWNS_H_EN	BIT(1)
#define DNSC_BLUR_OPMODE_DWNS_V_EN	BIT(2)
#define DNSC_BLUR_OPMODE_PCMN_H		BIT(8)
#define DNSC_BLUR_OPMODE_PCMN_V		BIT(12)
#define DNSC_BLUR_OPMODE_OUT_RND_8B_EN	BIT(16)

static struct sde_dnsc_blur_cfg *_dnsc_blur_offset(enum sde_dnsc_blur idx,
		struct sde_mdss_cfg *m, void __iomem *addr, struct sde_hw_blk_reg_map *b)
{
	int i;

	for (i = 0; i < m->dnsc_blur_count; i++) {
		if (idx == m->dnsc_blur[i].id) {
			b->base_off = addr;
			b->blk_off = m->dnsc_blur[i].base;
			b->length = m->dnsc_blur[i].len;
			b->hw_rev = m->hw_rev;
			b->log_mask = SDE_DBG_MASK_DNSC_BLUR;
			return &m->dnsc_blur[i];
		}
	}

	return ERR_PTR(-EINVAL);
}

static inline int _dnsc_blur_subblk_offset(struct sde_hw_dnsc_blur *hw_dnsc_blur,
		int s_id, u32 *base)
{
	const struct sde_dnsc_blur_sub_blks *sblk;

	sblk = hw_dnsc_blur->caps->sblk;

	switch (s_id) {
	case SDE_DNSC_BLUR_GAUS_LUT:
		*base = sblk->gaus_lut.base;
		break;
	case SDE_DNSC_BLUR_DITHER:
		*base = sblk->dither.base;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void _sde_hw_dnsc_blur_gaus_lut_setup(struct sde_hw_dnsc_blur *hw_dnsc_blur,
		struct sde_drm_dnsc_blur_cfg *cfg, u32 lut_sel)
{
	struct sde_hw_blk_reg_map *hw = &hw_dnsc_blur->hw;
	u32 lut_off, base;
	int i;

	if (_dnsc_blur_subblk_offset(hw_dnsc_blur, SDE_DNSC_BLUR_GAUS_LUT, &base))
		return;

	SDE_REG_WRITE(hw, DNSC_BLUR_GAUS_COEF_LUT_SEL, lut_sel);

	if (cfg->flags_h & DNSC_BLUR_GAUS_FILTER) {
		lut_off = lut_sel ? DNSC_BLUR_GAUS_COEF_LUT_H1 : DNSC_BLUR_GAUS_COEF_LUT_H0;
		for (i = 0; i < DNSC_BLUR_COEF_NUM; i++)
			SDE_REG_WRITE(hw, lut_off + (i * 0x4) + base, cfg->coef_hori[i]);
	}

	if (cfg->flags_v & DNSC_BLUR_GAUS_FILTER) {
		lut_off = lut_sel ? DNSC_BLUR_GAUS_COEF_LUT_V1 : DNSC_BLUR_GAUS_COEF_LUT_V0;
		for (i = 0; i < DNSC_BLUR_COEF_NUM; i++)
			SDE_REG_WRITE(hw, lut_off + (i * 0x4) + base, cfg->coef_vert[i]);
	}
}

static void _sde_hw_dnsc_blur_filter_setup(struct sde_hw_dnsc_blur *hw_dnsc_blur,
		struct sde_drm_dnsc_blur_cfg *cfg, u32 lut_sel)
{
	struct sde_hw_blk_reg_map *hw = &hw_dnsc_blur->hw;
	u32 val;

	/* PCMN */
	if (cfg->flags_h & DNSC_BLUR_PCMN_FILTER) {
		SDE_REG_WRITE(hw, DNSC_BLUR_PCMN_PHASE_INIT_H, cfg->phase_init_h);
		SDE_REG_WRITE(hw, DNSC_BLUR_PCMN_PHASE_STEP_H, cfg->phase_step_h);
	}

	if (cfg->flags_v & DNSC_BLUR_PCMN_FILTER) {
		SDE_REG_WRITE(hw, DNSC_BLUR_PCMN_PHASE_INIT_V, cfg->phase_init_v);
		SDE_REG_WRITE(hw, DNSC_BLUR_PCMN_PHASE_STEP_V, cfg->phase_step_v);
	}

	/* Gaussian */
	if (cfg->flags_h & DNSC_BLUR_GAUS_FILTER) {
		val = (cfg->norm_h << 16) | cfg->ratio_h;
		SDE_REG_WRITE(hw, DNSC_BLUR_BLUR_RATIO_H, val);
	}

	if (cfg->flags_v & DNSC_BLUR_GAUS_FILTER) {
		val = (cfg->norm_v << 16) | cfg->ratio_v;
		SDE_REG_WRITE(hw, DNSC_BLUR_BLUR_RATIO_V, val);
	}

	if ((cfg->flags_v | cfg->flags_h) & DNSC_BLUR_GAUS_FILTER)
		_sde_hw_dnsc_blur_gaus_lut_setup(hw_dnsc_blur, cfg, lut_sel);
}

static void _sde_hw_dnsc_blur_setup(struct sde_hw_dnsc_blur *hw_dnsc_blur,
		struct sde_drm_dnsc_blur_cfg *cfg, u32 lut_sel)
{
	struct sde_hw_blk_reg_map *hw = &hw_dnsc_blur->hw;
	u32 opmode = 0;

	/* disable, when no scaling involved */
	if (!cfg || !(cfg->flags & DNSC_BLUR_EN)) {
		SDE_REG_WRITE(hw, DNSC_BLUR_OP_MODE, 0x0);
		return;
	}

	opmode = DNSC_BLUR_OPMODE_ENABLE;
	opmode |= (cfg->flags & DNSC_BLUR_RND_8B_EN) ? DNSC_BLUR_OPMODE_OUT_RND_8B_EN : 0;

	if (cfg->flags_h) {
		opmode |= DNSC_BLUR_OPMODE_DWNS_H_EN;
		opmode |= (cfg->flags_h & DNSC_BLUR_PCMN_FILTER) ? DNSC_BLUR_OPMODE_PCMN_H : 0;
	}

	if (cfg->flags_v) {
		opmode |= DNSC_BLUR_OPMODE_DWNS_V_EN;
		opmode |= (cfg->flags_v & DNSC_BLUR_PCMN_FILTER) ? DNSC_BLUR_OPMODE_PCMN_V : 0;
	}

	_sde_hw_dnsc_blur_filter_setup(hw_dnsc_blur, cfg, lut_sel);

	SDE_REG_WRITE(hw, DNSC_BLUR_OP_MODE, opmode);
	SDE_REG_WRITE(hw, DNSC_BLUR_OUT_IMG_SIZE, (cfg->dst_height << 16) | cfg->dst_width);
	SDE_REG_WRITE(hw, DNSC_BLUR_SRC_IMG_SIZE, (cfg->src_height << 16) | cfg->src_width);
}

static void _sde_hw_dnsc_blur_dither_setup(struct sde_hw_dnsc_blur *hw_dnsc_blur,
		struct sde_drm_dnsc_blur_cfg *cfg)
{
	struct sde_hw_blk_reg_map *hw = &hw_dnsc_blur->hw;
	int i;
	u32 base, data, offset;

	if (_dnsc_blur_subblk_offset(hw_dnsc_blur, SDE_DNSC_BLUR_DITHER, &base))
		return;

	/* disable case */
	if (!cfg || !(cfg->flags & DNSC_BLUR_DITHER_EN)) {
		SDE_REG_WRITE(hw, DNSC_BLUR_DITHER_OP_MODE + base, 0x0);
		return;
	}

	data = (dither_depth_map[cfg->c0_bitdepth] & REG_MASK(2)) |
			((dither_depth_map[cfg->c1_bitdepth] & REG_MASK(2)) << 2) |
			((dither_depth_map[cfg->c2_bitdepth] & REG_MASK(2)) << 4) |
			((dither_depth_map[cfg->c3_bitdepth] & REG_MASK(2)) << 6) |
			((cfg->temporal_en) ? (1 << 8) : 0);
	SDE_REG_WRITE(hw, DNSC_BLUR_DITHER_BITDEPTH + base, data);

	offset = DNSC_BLUR_DITHER_MATRIX_ROW0;
	for (i = 0; i < DNSC_BLUR_DITHER_MATRIX_SZ - 3; i += 4) {
		data = (cfg->dither_matrix[i] & REG_MASK(4)) |
				((cfg->dither_matrix[i + 1] & REG_MASK(4)) << 4) |
				((cfg->dither_matrix[i + 2] & REG_MASK(4)) << 8) |
				((cfg->dither_matrix[i + 3] & REG_MASK(4)) << 12);
		SDE_REG_WRITE(hw, base + offset, data);
		offset += 4;
	}

	data = BIT(0);
	data |= (cfg->dither_flags & DNSC_BLUR_DITHER_LUMA_MODE) ? BIT(4) : 0;
	SDE_REG_WRITE(hw, DNSC_BLUR_DITHER_OP_MODE + base, data);
}

static void _sde_hw_dnsc_blur_bind_pingpong_blk(struct sde_hw_dnsc_blur *hw_dnsc_blur,
		bool enable, const enum sde_pingpong pp, bool cwb)
{
	struct sde_hw_blk_reg_map *hw = &hw_dnsc_blur->hw;
	int mux_cfg;

	if (enable && (pp < PINGPONG_0 || pp >= PINGPONG_MAX))
		return;

	if (enable)
		mux_cfg = cwb ? 0xD : (pp - PINGPONG_0) & 0x7;
	else
		mux_cfg = 0xF;
	SDE_REG_WRITE(hw, DNSC_BLUR_MUX, mux_cfg);
}

static void _setup_dnsc_blur_ops(struct sde_hw_dnsc_blur_ops *ops, unsigned long features)
{
	ops->setup_dnsc_blur = _sde_hw_dnsc_blur_setup;
	ops->setup_dither = _sde_hw_dnsc_blur_dither_setup;
	ops->bind_pingpong_blk = _sde_hw_dnsc_blur_bind_pingpong_blk;
}

struct sde_hw_blk_reg_map *sde_hw_dnsc_blur_init(enum sde_dnsc_blur idx,
		void __iomem *addr, struct sde_mdss_cfg *m)
{
	struct sde_hw_dnsc_blur *c;
	struct sde_dnsc_blur_cfg *cfg;

	if (!addr || !m)
		return ERR_PTR(-EINVAL);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	cfg = _dnsc_blur_offset(idx, m, addr, &c->hw);
	if (IS_ERR_OR_NULL(cfg)) {
		kfree(c);
		return ERR_PTR(-EINVAL);
	}

	c->idx = idx;
	c->caps = cfg;
	_setup_dnsc_blur_ops(&c->ops, c->caps->features);

	sde_dbg_reg_register_dump_range(SDE_DBG_NAME, cfg->name, c->hw.blk_off,
			c->hw.blk_off + c->hw.length, c->hw.xin_id);

	if (cfg->sblk->gaus_lut.base && cfg->sblk->gaus_lut.len)
		sde_dbg_reg_register_dump_range(SDE_DBG_NAME, cfg->sblk->gaus_lut.name,
				c->hw.blk_off + cfg->sblk->gaus_lut.base,
				c->hw.blk_off + cfg->sblk->gaus_lut.base +
					cfg->sblk->gaus_lut.len, c->hw.xin_id);

	if (cfg->sblk->dither.base && cfg->sblk->dither.len)
		sde_dbg_reg_register_dump_range(SDE_DBG_NAME, cfg->sblk->dither.name,
				c->hw.blk_off + cfg->sblk->dither.base,
				c->hw.blk_off + cfg->sblk->dither.base +
					cfg->sblk->dither.len, c->hw.xin_id);

	return &c->hw;
}

void sde_hw_dnsc_blur_destroy(struct sde_hw_blk_reg_map *hw)
{
	if (hw)
		kfree(to_sde_hw_dnsc_blur(hw));
}

