/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2017-2019, 2021 The Linux Foundation. All rights reserved.
 */
#ifndef _SDE_HW_COLOR_PROC_V4_H_
#define _SDE_HW_COLOR_PROC_V4_H_

#include "sde_hw_util.h"
#include "sde_hw_catalog.h"
#include "sde_hw_dspp.h"
#include "sde_hw_sspp.h"

/**
 * sde_setup_dspp_3d_gamutv4 - Function for 3d gamut v4 version feature
 *                             programming.
 * @ctx: dspp ctx pointer
 * @cfg: pointer to sde_hw_cp_cfg
 */
void sde_setup_dspp_3d_gamutv4(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_setup_dspp_3d_gamutv41 - Function for 3d gamut v4_1 version feature
 *                             programming.
 * @ctx: dspp ctx pointer
 * @cfg: pointer to sde_hw_cp_cfg
 */
void sde_setup_dspp_3d_gamutv41(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_setup_dspp_igcv3 - Function for igc v3 version feature
 *                             programming.
 * @ctx: dspp ctx pointer
 * @cfg: pointer to sde_hw_cp_cfg
 */
void sde_setup_dspp_igcv3(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_setup_dspp_pccv4 - Function for pcc v4 version feature
 *                             programming.
 * @ctx: dspp ctx pointer
 * @cfg: pointer to sde_hw_cp_cfg
 */
void sde_setup_dspp_pccv4(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_setup_dspp_ltm_threshv1 - Function for ltm thresh v1 programming.
 * @ctx: dspp ctx pointer
 * @cfg: pointer to sde_hw_cp_cfg
 */
void sde_setup_dspp_ltm_threshv1(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_setup_dspp_ltm_hist_ctrlv1 - Function for ltm hist_ctrl v1 programming.
 * @ctx: dspp ctx pointer
 * @cfg: pointer to sde_hw_cp_cfg
 * @enable: feature enable/disable value
 * @addr: aligned iova address
 */
void sde_setup_dspp_ltm_hist_ctrlv1(struct sde_hw_dspp *ctx, void *cfg,
				    bool enable, u64 addr);

/**
 * sde_setup_dspp_ltm_hist_ctrlv1_2 - Same as v1 except ltm merge mode setting.
 * @ctx: dspp ctx pointer
 * @cfg: pointer to sde_hw_cp_cfg
 * @enable: feature enable/disable value
 * @addr: aligned iova address
 */
void sde_setup_dspp_ltm_hist_ctrlv1_2(struct sde_hw_dspp *ctx, void *cfg,
				    bool enable, u64 addr);

/**
 * sde_setup_dspp_ltm_hist_bufferv1 - Function for setting ltm hist buffer v1.
 * @ctx: dspp ctx pointer
 * @addr: aligned iova address
 */
void sde_setup_dspp_ltm_hist_bufferv1(struct sde_hw_dspp *ctx, u64 addr);

/**
 * sde_ltm_read_intr_status - api to get ltm interrupt status
 * @dspp: pointer to dspp object
 * @status: Pointer to u32 where ltm status value is dumped.
 */
void sde_ltm_read_intr_status(struct sde_hw_dspp *dspp, u32 *status);

/**
 * sde_ltm_clear_merge_mode - api to clear ltm merge_mode
 * @dspp: pointer to dspp object
 */
void sde_ltm_clear_merge_mode(struct sde_hw_dspp *dspp);

/**
 * sde_ltm_clear_merge_modev1_2 - api to clear ltm merge_mode
 * @dspp: pointer to dspp object
 */
void sde_ltm_clear_merge_modev1_2(struct sde_hw_dspp *dspp);


/**
 * sde_demura_backlight_cfg - api to set backlight for demura
 * @ctx: pointer to dspp object
 * @val: value of backlight
 * @cfg: pointer to sde_hw_cp_cfg
 */
void sde_demura_backlight_cfg(struct sde_hw_dspp *ctx, u64 val, struct sde_hw_cp_cfg *cfg);

/**
 * sde_demura_read_plane_status - api to read demura plane fetch setup.
 * @ctx: pointer to dspp object.
 * @status: Currently present plane. Reported as a demura_fetch_planes value.
 */
void sde_demura_read_plane_status(struct sde_hw_dspp *ctx, u32 *status);

/**
 * sde_setup_fp16_cscv1 - api to set FP16 CSC cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing drm_msm_fp16_csc data
 */
void sde_setup_fp16_cscv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_setup_fp16_gcv1 - api to set FP16 GC cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing drm_msm_fp16_gc data
 */
void sde_setup_fp16_gcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_setup_fp16_igcv1 - api to set FP16 IGC cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing bool data
 */
void sde_setup_fp16_igcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_setup_fp16_unmultv1 - api to set FP16 UNMULT cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing bool data
 */
void sde_setup_fp16_unmultv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_demura_pu_cfg - api to set the partial update information for demura
 * @ctx: pointer to dspp object.
 * @cfg: partial update configuraton for the frame.
*/
void sde_demura_pu_cfg(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_spr_check_init_cfg - api to validate the SPR configuration for the frame
 * @ctx: pointer to dspp object.
 * @cfg: spr configuration  for the frame.
 */
int sde_spr_check_init_cfg(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_spr_check_udc_cfg - api to validate the SPR UDC configuration for the frame
 * @ctx: pointer to dspp object.
 * @cfg: spr UDC configuration  for the frame.
 */
int sde_spr_check_udc_cfg(struct sde_hw_dspp *ctx, void *cfg);

/**
 * sde_spr_read_opr_value - api to read spr opr value
 * @ctx: pointer to dspp object.
 * @opr_value: Pointer to opr value.
 */
int sde_spr_read_opr_value(struct sde_hw_dspp *ctx, uint32_t *opr_value);

/**
 * sde_setup_ucsc_cscv1 - api to set UCSC CSC cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing drm_msm_ucsc_csc data
 */
void sde_setup_ucsc_cscv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_setup_ucsc_gcv1 - api to set UCSC GC cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing gc mode data
 */
void sde_setup_ucsc_gcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_setup_ucsc_igcv1 - api to set UCSC IGC cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing igc mode data
 */
void sde_setup_ucsc_igcv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_setup_ucsc_unmultv1 - api to set UCSC UNMULT cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing bool data
 */
void sde_setup_ucsc_unmultv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

/**
 * sde_setup_ucsc_alpha_ditherv1 - api to set UCSC ALPHA DITHER cp block
 * @ctx: pointer to pipe object
 * @index: pipe rectangle to operate on
 * @data: pointer to sde_hw_cp_cfg object containing bool data
 */
void sde_setup_ucsc_alpha_ditherv1(struct sde_hw_pipe *ctx,
		enum sde_sspp_multirect_index index, void *data);

#endif /* _SDE_HW_COLOR_PROC_V4_H_ */
