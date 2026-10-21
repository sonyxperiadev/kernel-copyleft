// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__
#include "sde_hw_ds.h"
#include "sde_formats.h"
#include "sde_dbg.h"
#include "sde_kms.h"

/* Destination scaler TOP registers */
#define DEST_SCALER_OP_MODE     0x00
#define DEST_SCALER_HW_VERSION  0x10
#define DEST_SCALER_MERGE_CTRL  0x0C

#define DEST_SCALER_DUAL_PIPE   1
#define DEST_SCALER_QUAD_PIPE   3

static void sde_hw_ds_setup_opmode(struct sde_hw_ds *hw_ds, u32 op_mode)
{
	struct sde_hw_blk_reg_map *hw = &hw_ds->hw;
	u32 op_mode_val;

	op_mode_val = SDE_REG_READ(hw, DEST_SCALER_OP_MODE);

	if (op_mode)
		op_mode_val |= op_mode;
	else if (!op_mode && (op_mode_val & SDE_DS_OP_MODE_DUAL))
		op_mode_val = 0;
	else
		op_mode_val &= ~BIT(hw_ds->idx - DS_0);

	SDE_REG_WRITE(hw, DEST_SCALER_OP_MODE, op_mode_val);
}

static void sde_hw_ds_setup_opmode_v1(struct sde_hw_ds *hw_ds, u32 op_mode)
{
	struct sde_hw_blk_reg_map *hw = &hw_ds->hw;

	if (op_mode & SDE_DS_OP_MODE_DUAL) {
		op_mode = DEST_SCALER_DUAL_PIPE;
		SDE_REG_WRITE(hw, DEST_SCALER_MERGE_CTRL + hw_ds->scl->base, op_mode);
	}
}

static void sde_hw_ds_setup_scaler3(struct sde_hw_ds *hw_ds,
			void *scaler_cfg, void *scaler_lut_cfg)
{
	struct sde_hw_scaler3_cfg *scl3_cfg = scaler_cfg;
	struct sde_hw_scaler3_lut_cfg *scl3_lut_cfg = scaler_lut_cfg;

	bool de_lpf_en = false;

	if (!hw_ds || !hw_ds->scl || !scl3_cfg || !scl3_lut_cfg)
		return;

	/*
	 * copy LUT values to scaler structure
	 */
	if (scl3_lut_cfg->is_configured) {
		scl3_cfg->dir_lut = scl3_lut_cfg->dir_lut;
		scl3_cfg->dir_len = scl3_lut_cfg->dir_len;
		scl3_cfg->cir_lut = scl3_lut_cfg->cir_lut;
		scl3_cfg->cir_len = scl3_lut_cfg->cir_len;
		scl3_cfg->sep_lut = scl3_lut_cfg->sep_lut;
		scl3_cfg->sep_len = scl3_lut_cfg->sep_len;
	}


	if (test_bit(SDE_DS_DE_LPF_BLEND, &hw_ds->scl->features))
		de_lpf_en = true;
	sde_hw_setup_scaler3(&hw_ds->hw, scl3_cfg, hw_ds->scl->version,
			 hw_ds->scl->base,
			 sde_get_sde_format(DRM_FORMAT_XBGR2101010), de_lpf_en);
}

static void _setup_ds_ops(struct sde_hw_ds_ops *ops, unsigned long features)
{

	if (test_bit(SDE_DS_MERGE_CTRL, &features))
		ops->setup_opmode = sde_hw_ds_setup_opmode_v1;
	else
		ops->setup_opmode = sde_hw_ds_setup_opmode;

	if (test_bit(SDE_SSPP_SCALER_QSEED3, &features) ||
			test_bit(SDE_SSPP_SCALER_QSEED3LITE, &features))
		ops->setup_scaler = sde_hw_ds_setup_scaler3;
}

static struct sde_ds_cfg *_ds_offset(enum sde_ds ds,
		struct sde_mdss_cfg *m,
		void __iomem *addr,
		struct sde_hw_blk_reg_map *b)
{
	int i;

	if (!m || !addr || !b)
		return ERR_PTR(-EINVAL);

	for (i = 0; i < m->ds_count; i++) {
		if ((ds == m->ds[i].id) &&
			 (m->ds[i].top)) {
			b->base_off = addr;
			b->blk_off = m->ds[i].top->base;
			b->length = m->ds[i].top->len;
			b->hw_rev = m->hw_rev;
			b->log_mask = SDE_DBG_MASK_DS;
			return &m->ds[i];
		}
	}

	return ERR_PTR(-EINVAL);
}

struct sde_hw_blk_reg_map *sde_hw_ds_init(enum sde_ds idx,
			void __iomem *addr,
			struct sde_mdss_cfg *m)
{
	struct sde_hw_ds *hw_ds;
	struct sde_ds_cfg *cfg;

	if (!addr || !m)
		return ERR_PTR(-EINVAL);

	hw_ds = kzalloc(sizeof(*hw_ds), GFP_KERNEL);
	if (!hw_ds)
		return ERR_PTR(-ENOMEM);

	cfg = _ds_offset(idx, m, addr, &hw_ds->hw);
	if (IS_ERR_OR_NULL(cfg)) {
		SDE_ERROR("failed to get ds cfg\n");
		kfree(hw_ds);
		return ERR_PTR(-EINVAL);
	}

	/* Assign ops */
	hw_ds->idx = idx;
	hw_ds->scl = cfg;
	_setup_ds_ops(&hw_ds->ops, hw_ds->scl->features);

	if (m->qseed_hw_rev)
		hw_ds->scl->version = m->qseed_hw_rev;

	if (cfg->len) {
		sde_dbg_reg_register_dump_range(SDE_DBG_NAME, cfg->name,
				hw_ds->hw.blk_off + cfg->base,
				hw_ds->hw.blk_off + cfg->base + cfg->len,
				hw_ds->hw.xin_id);
	}

	return &hw_ds->hw;
}

void sde_hw_ds_destroy(struct sde_hw_blk_reg_map *hw)
{
	if (hw)
		kfree(to_sde_hw_ds(hw));
}
