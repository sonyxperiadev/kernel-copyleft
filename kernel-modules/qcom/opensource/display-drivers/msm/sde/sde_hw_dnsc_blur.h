/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_HW_DNSC_BLUR_H
#define _SDE_HW_DNSC_BLUR_H

#include <drm/sde_drm.h>
#include <drm/msm_drm_pp.h>

#include "sde_hw_mdss.h"

struct sde_hw_dnsc_blur;

/**
 * sde_hw_dnsc_blur_ops - interface to the DNSC_BLUR HW driver functions
 * Caller must call the init function to the dnsc_blur hw context for dnsc_blur
 * Assumption is these functions will be called after clocks are enabled
 */
struct sde_hw_dnsc_blur_ops {
	/**
	 * setup_dnsc_blur - downscale blur block setup
	 * @hw_dnsc_blur: Pointer to dnsc_blur context
	 * @cfg : Pointer to dnsc_blur configs
	 * @lut_sel: LUT index for Gausian filter
	 */
	void (*setup_dnsc_blur)(struct sde_hw_dnsc_blur *hw_dnsc_blur,
			struct sde_drm_dnsc_blur_cfg *cfg, u32 lut_sel);

	/**
	 * setup_dither - Dither programming
	 * @hw_dnsc_blur: Pointer to dnsc_blur context
	 * @cfg : Pointer to dnsc_blur configs
	 */
	void (*setup_dither)(struct sde_hw_dnsc_blur *hw_dnsc_blur,
			struct sde_drm_dnsc_blur_cfg *cfg);

	/**
	 * bind_pingpong_blk - connection with pingpong block which feeds pixels
	 * to downscale blur block
	 * @hw_dnsc_blur: Pointer to dnsc_blur context
	 * @enable: Boolean to indicate enable/disable of the binding
	 * @pp: Pingpong block idx for binding
	 * @cwb: Flag to indicate concurrent writeback
	 */
	void (*bind_pingpong_blk)(struct sde_hw_dnsc_blur *hw_dnsc_blur,
			bool enable, const enum sde_pingpong pp, bool cwb);
};

/**
 * struct sde_hw_dnsc_blur - downscale blur description
 * @hw: Block hardware details
 * @caps: Pointer to block capabilities
 * @idx: Downscale Blur index
 * @ops: Pointer to operations for this block
 */
struct sde_hw_dnsc_blur {
	struct sde_hw_blk_reg_map hw;
	const struct sde_dnsc_blur_cfg *caps;
	enum sde_dnsc_blur idx;
	struct sde_hw_dnsc_blur_ops ops;
};

/**
 * to_sde_hw_dnsc_blur - convert base hw object to sde_hw_dnsc_blur to container
 * @hw: Pointer to base hardware block register map object
 * return: Pointer to hardware block container
 */
static inline struct sde_hw_dnsc_blur *to_sde_hw_dnsc_blur(struct sde_hw_blk_reg_map *hw)
{
	return container_of(hw, struct sde_hw_dnsc_blur, hw);
}

/**
 * sde_hw_dnsc_blur_init - initializes the dnsc_blur hw driver object
 * @idx: dnsc_blur index for which driver object is required
 * @addr: mapped register io address of MDP
 * @m: pointer to mdss catalog data
 */
struct sde_hw_blk_reg_map *sde_hw_dnsc_blur_init(enum sde_dnsc_blur idx,
		void __iomem *addr, struct sde_mdss_cfg *m);

/**
 * sde_hw_dnsc_blur_destroy - destroys dnsc_blur driver context
 * @hw: Pointer to hardware block register map object
 */
void sde_hw_dnsc_blur_destroy(struct sde_hw_blk_reg_map *hw);

#endif /*_SDE_HW_DNSC_BLUR_H */
