// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "tpg_hw_v_1_3.h"

enum tpg_hw_v_1_3_encode_fomat_t {
	RAW_8_BIT = 1,
	RAW_10_BIT,
	RAW_12_BIT,
	RAW_14_BIT,
	RAW_16_BIT
};


#define  FRAME_INTERLEAVE           0x0
#define  LINE_INTERLEAVE            0x1
#define  SHDR_INTERLEAVE            0x2
#define  SPARSE_PD_INTERLEAVE       0x3
#define  CFA_PATTERN_ROW_WIDTH      8
#define  CFA_PATTERN_BITS_PER_INDEX 2
#define  Invalid 0x0
#define  Red     0x0
#define  Green   0x1
#define  Blue    0x2
#define  IR      0x3
#define  Mono    0x3

static int get_tpg_vc_dt_pattern_id(
		enum tpg_interleaving_format_t vc_dt_pattern)
{
	switch (vc_dt_pattern) {
	case TPG_INTERLEAVING_FORMAT_INVALID:
	case TPG_INTERLEAVING_FORMAT_MAX:
	case TPG_INTERLEAVING_FORMAT_FRAME:
		return FRAME_INTERLEAVE;
	case TPG_INTERLEAVING_FORMAT_LINE:
		return LINE_INTERLEAVE;
	case TPG_INTERLEAVING_FORMAT_SHDR:
		return SHDR_INTERLEAVE;
	case TPG_INTERLEAVING_FORMAT_SPARSE_PD:
		return SPARSE_PD_INTERLEAVE;

	}
	return FRAME_INTERLEAVE;
}

static int configure_global_configs(
	struct tpg_hw *hw,
	int num_vcs,
	struct tpg_global_config_t *configs)
{
	uint32_t val, phy_type = 0;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}

	tpg_reg  = hw->hw_info->hw_data;
	soc_info = hw->soc_info;

	if (configs->phy_type == TPG_PHY_TYPE_CPHY)
		phy_type = 1;

	if (num_vcs <= 0) {
		CAM_ERR(CAM_TPG, "Invalid vc count");
		return -EINVAL;
	}

	val = configs->skip_pattern;
	cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_throttle);
	CAM_DBG(CAM_TPG, "tpg[%d] throttle=0x%x", hw->hw_idx, val);

	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_top_irq_mask);


	val = ((num_vcs - 1) <<
			(tpg_reg->tpg_num_active_vcs_shift) |
			(configs->lane_count - 1) <<
			tpg_reg->tpg_num_active_lanes_shift) |
		get_tpg_vc_dt_pattern_id(configs->interleaving_format) <<
		(tpg_reg->tpg_vc_dt_pattern_id_shift) |
		(phy_type << tpg_reg->tpg_cphy_dphy_sel_shift_val) |
		(1 << tpg_reg->tpg_en_shift_val);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base + tpg_reg->tpg_ctrl);
	CAM_DBG(CAM_TPG, "tpg[%d] tpg_ctrl=0x%x", hw->hw_idx, val);
	return 0;
}

static int get_tpg_encode_format(int sw_encode_format)
{
	switch (sw_encode_format) {
	case PACK_8_BIT:
		return RAW_8_BIT;
	case PACK_10_BIT:
		return RAW_10_BIT;
	case PACK_12_BIT:
		return RAW_12_BIT;
	case PACK_14_BIT:
		return RAW_14_BIT;
	case PACK_16_BIT:
		return RAW_16_BIT;
	}
	return RAW_8_BIT;
}

#define  INCREMENTING       0x0
#define  ALTERNATING_55_AA  0x1
#define  RANDOM             0x4
#define  USER_SPECIFIED     0x5
#define  COLOR_BARS         0x8

static int get_tpg_payload_mode(enum tpg_pattern_t pattern)
{
	switch (pattern) {
	case TPG_PATTERN_INVALID:
	case TPG_PATTERN_REAL_IMAGE:
	case TPG_PATTERN_COLOR_BAR:
		return COLOR_BARS;
	case TPG_PATTERN_RANDOM_PIXL:
	case TPG_PATTERN_RANDOM_INCREMENTING_PIXEL:
		return RANDOM;
	case TPG_PATTERN_ALTERNATING_55_AA:
		return ALTERNATING_55_AA;
	case TPG_PATTERN_ALTERNATING_USER_DEFINED:
		return USER_SPECIFIED;
	default:
		return COLOR_BARS;
	}
	return COLOR_BARS;
}

static int get_pixel_coordinate(
	int cfa_pattern_start_index,
	int cfa_pattern_end_index,
	uint32_t *val,
	struct tpg_stream_config_v3_t *configs)
{
	uint32_t shift = 0;
	int idx = 0;
	int i = 0;
	int j = 0;
	*val = 0;
	for (i = cfa_pattern_start_index; i < cfa_pattern_end_index; i++) {
		for (j = 0; j < configs->cfa_info.pattern_width; j++) {
			shift = ((i * CFA_PATTERN_ROW_WIDTH) + j) *
				CFA_PATTERN_BITS_PER_INDEX;
			idx = i * configs->cfa_info.pattern_height + j;
			*val |= (configs->cfa_info.pixel_coordinate[idx].pixel_type) << shift;
		}
	}
	return 0;
}

static int configure_xcfa_array_v3(
	struct tpg_hw *hw,
	struct tpg_stream_config_v3_t *configs)
{
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;
	uint32_t val = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}
	tpg_reg  = hw->hw_info->hw_data;

	soc_info = hw->soc_info;

	switch (configs->xcfa_type) {
	case XCFA_TYPE_RGBIR:
		get_pixel_coordinate(0, 2, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		get_pixel_coordinate(2, configs->cfa_info.pattern_height, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color1);
		break;
	case XCFA_TYPE_QUADCFA:
		get_pixel_coordinate(0, 2, &val, configs);
		CAM_DBG(CAM_TPG, "val = 0x%x", val);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		get_pixel_coordinate(2, configs->cfa_info.pattern_height, &val, configs);
		CAM_DBG(CAM_TPG, "val = 0x%x", val);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color1);
		break;
	case XCFA_TYPE_THREEXTHREECFA:
		get_pixel_coordinate(0, 2, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		get_pixel_coordinate(2, 4, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color1);
		get_pixel_coordinate(4, configs->cfa_info.pattern_height, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color2);
		break;
	case XCFA_TYPE_FOURXFOURCFA:
		get_pixel_coordinate(0, 2, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		get_pixel_coordinate(2, 4, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color1);
		get_pixel_coordinate(4, 6, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color2);
		get_pixel_coordinate(6, configs->cfa_info.pattern_height, &val, configs);
		cam_io_w_mb(val,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color3);
		break;
	default:
		break;
	}
	return 0;
}

static int configure_dt_v3(
	struct tpg_hw *hw,
	uint32_t       vc_slot,
	uint32_t       dt_slot,
	struct tpg_stream_config_v3_t *stream)
{
	uint32_t val;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}

	tpg_reg  = hw->hw_info->hw_data;

	soc_info = hw->soc_info;

	CAM_DBG(CAM_TPG, "TPG[%d] slot(%d,%d) <= dt:%d",
			hw->hw_idx,
			vc_slot,
			dt_slot,
			stream->dt);

	val = (((stream->stream_dimension.width & 0xFFFF) << 16) |
			(stream->stream_dimension.height & 0xFFFF));
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_dt_0_cfg_0 +
			(0x60 * vc_slot) + (dt_slot * 0x0c));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_dt%d_cfg_0=0x%x",
			hw->hw_idx,
			vc_slot, dt_slot, val);

	cam_io_w_mb(stream->dt,
			soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_dt_0_cfg_1 +
			(0x60 * vc_slot) + (dt_slot * 0x0c));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_dt%d_cfg_1=0x%x",
			hw->hw_idx,
			vc_slot, dt_slot, stream->dt);

	val = ((get_tpg_encode_format(stream->pixel_depth) & 0xF) <<
			tpg_reg->tpg_dt_encode_format_shift) |
			get_tpg_payload_mode(stream->pattern_type);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_dt_0_cfg_2 +
			(0x60 * vc_slot) + (dt_slot * 0x0c));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_dt%d_cfg_2=0x%x",
			hw->hw_idx,
			vc_slot, dt_slot, val);

	return 0;
}

static int configure_dt(
	struct tpg_hw *hw,
	uint32_t       vc_slot,
	uint32_t       dt_slot,
	struct tpg_stream_config_t *stream)
{
	uint32_t val;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}

	tpg_reg  = hw->hw_info->hw_data;

	soc_info = hw->soc_info;

	CAM_DBG(CAM_TPG, "TPG[%d] slot(%d,%d) <= dt:%d",
			hw->hw_idx,
			vc_slot,
			dt_slot,
			stream->dt);

	val = (((stream->stream_dimension.width & 0xFFFF) << 16) |
			(stream->stream_dimension.height & 0xFFFF));
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_dt_0_cfg_0 +
			(0x60 * vc_slot) + (dt_slot * 0x0c));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_dt%d_cfg_0=0x%x",
			hw->hw_idx,
			vc_slot, dt_slot, val);

	cam_io_w_mb(stream->dt,
			soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_dt_0_cfg_1 +
			(0x60 * vc_slot) + (dt_slot * 0x0c));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_dt%d_cfg_1=0x%x",
			hw->hw_idx,
			vc_slot, dt_slot, stream->dt);

	val = ((get_tpg_encode_format(stream->pixel_depth) & 0xF) <<
			tpg_reg->tpg_dt_encode_format_shift) |
		get_tpg_payload_mode(stream->pattern_type);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_dt_0_cfg_2 +
			(0x60 * vc_slot) + (dt_slot * 0x0c));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_dt%d_cfg_2=0x%x",
			hw->hw_idx,
			vc_slot, dt_slot, val);

	return 0;
}



#define VC1_GAIN   0x100

static int configure_vc_v3(
	struct tpg_hw *hw,
	uint32_t       vc_slot,
	int            num_dts,
	struct tpg_stream_config_v3_t *stream)
{
	uint32_t val = 0;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}
	tpg_reg  = hw->hw_info->hw_data;

	soc_info = hw->soc_info;
	/* Use CFA pattern here */
	if (stream->output_format == TPG_IMAGE_FORMAT_QCFA)
		val |= (1 << tpg_reg->tpg_color_bar_qcfa_en_shift);

	if (stream->cb_mode == TPG_COLOR_BAR_MODE_SPLIT)
		val |= (1 << tpg_reg->tpg_split_en_shift);

	if (stream->cfa_info_exist != 0) {
		val |= ((stream->cfa_info.pattern_height - 1) << tpg_reg->tpg_size_y_shift);
		val |= ((stream->cfa_info.pattern_width - 1) << tpg_reg->tpg_size_x_shift);
		val |= (1 << tpg_reg->tpg_xcfa_en_shift);
		configure_xcfa_array_v3(hw, stream);
	}

	CAM_DBG(CAM_TPG, "TPG[%d] period: %d", hw->hw_idx, stream->rotate_period);
	val |= ((stream->rotate_period & 0x3F) <<
			tpg_reg->tpg_color_bar_qcfa_rotate_period_shift);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_color_bar_cfg + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_color_bar_cfg=0x%x",
			hw->hw_idx,
			vc_slot, val);

	val = stream->hbi;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_hbi_cfg + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_hbi_cfg=0x%x",
			hw->hw_idx,
			vc_slot, val);

	val = stream->vbi;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_vbi_cfg + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_vbi_cgf=0x%x",
			hw->hw_idx,
			vc_slot, val);

	cam_io_w_mb(0x12345678,
		soc_info->reg_map[0].mem_base +
		tpg_reg->tpg_vc0_lfsr_seed + (0x60 * vc_slot));

	val = ((0 << tpg_reg->tpg_num_frames_shift_val) |
		((num_dts-1) <<	 tpg_reg->tpg_num_dts_shift_val) |
		stream->vc);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_cfg0 + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_cfg0=0x%x",
			hw->hw_idx,
			vc_slot, val);
	if (hw->hw_info->shdr_overlap == 1) {
		cam_io_w_mb(hw->hw_info->shdr_overlap << tpg_reg->tpg_overlap_shdr_en_shift,
			soc_info->reg_map[0].mem_base + tpg_reg->tpg_ctrl);
	}
	if (hw->hw_info->shdr_offset_num_batch >= 0 && vc_slot > 0)	{
		val =  (VC1_GAIN << tpg_reg->tpg_gain_shift);
		val |= (hw->hw_info->shdr_offset_num_batch <<
				tpg_reg->tpg_shdr_offset_num_batch_shift);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
					tpg_reg->tpg_vc1_gain_cfg + (0x60 * (vc_slot-1)));
		val =  ((stream->shdr_line_offset0 * vc_slot)
			<< tpg_reg->tpg_shdr_line_offset0_shift);
		val |= ((stream->shdr_line_offset1 * vc_slot)
			<< tpg_reg->tpg_shdr_line_offset1_shift);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc1_shdr_cfg + (0x60 * (vc_slot-1)));
		CAM_DBG(CAM_TPG, "TPG[%d] vc%d_cfg0=0x%x shdr",
			hw->hw_idx,
			vc_slot, val);
	}

	return 0;
}

#define RGGB_IR_0  0x00770091
#define RGGB_IR_1  0x00770019
#define RGGB_2x2   0x05055A5A
#define RGGB_3x3_0 0x05400540
#define RGGB_3x3_1 0x0a950540
#define RGGB_3x3_2 0x0a950a95
#define RGGB_4x4_0 0x55005500
#define RGGB_4x4_1 0x55005500
#define RGGB_4x4_2 0xaa55aa55
#define RGGB_4x4_3 0xaa55aa55
#define VC1_GAIN   0x100

static int configure_xcfa_array(struct tpg_hw *hw, int config)
{
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}
	tpg_reg  = hw->hw_info->hw_data;

	soc_info = hw->soc_info;

	switch (config) {
	case 1:
		cam_io_w_mb(RGGB_IR_0,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		cam_io_w_mb(RGGB_IR_1,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color1);
		break;
	case 2:
		cam_io_w_mb(RGGB_2x2,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		break;
	case 3:
		cam_io_w_mb(RGGB_3x3_0,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		cam_io_w_mb(RGGB_3x3_1,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color1);
		cam_io_w_mb(RGGB_3x3_2,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color2);
		break;
	case 4:
		cam_io_w_mb(RGGB_4x4_0,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color0);
		cam_io_w_mb(RGGB_4x4_1,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color1);
		cam_io_w_mb(RGGB_4x4_2,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color2);
		cam_io_w_mb(RGGB_4x4_3,
		soc_info->reg_map[0].mem_base + tpg_reg->tpg_vc0_color_bar_cfa_color3);
		break;
	break;

	}
	return 0;
}

static int configure_vc(
	struct tpg_hw *hw,
	uint32_t       vc_slot,
	int            num_dts,
	struct tpg_stream_config_t *stream)
{
	uint32_t val = 0;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}
	tpg_reg = hw->hw_info->hw_data;

	soc_info = hw->soc_info;
	/* Use CFA pattern here */
	if (stream->output_format == TPG_IMAGE_FORMAT_QCFA)
		val |= (1 << tpg_reg->tpg_color_bar_qcfa_en_shift);

	if (stream->cb_mode == TPG_COLOR_BAR_MODE_SPLIT)
		val |= (1 << tpg_reg->tpg_split_en_shift);

	if (stream->xcfa_debug > 0) {
		if (stream->xcfa_debug == 1) {
			val |= (3 << tpg_reg->tpg_size_y_shift);
			val |= (3 << tpg_reg->tpg_size_x_shift);
		} else {
			val |= ((stream->xcfa_debug * 2 - 1) << tpg_reg->tpg_size_y_shift);
			val |= ((stream->xcfa_debug * 2 - 1) << tpg_reg->tpg_size_x_shift);
		}
		val |= (1 << tpg_reg->tpg_xcfa_en_shift);
		configure_xcfa_array(hw, stream->xcfa_debug);
		CAM_DBG(CAM_TPG, "xcfa_debug = %d", stream->xcfa_debug);
	}

	CAM_DBG(CAM_TPG, "TPG[%d] period: %d", hw->hw_idx, stream->rotate_period);
	val |= ((stream->rotate_period & 0x3F) <<
			tpg_reg->tpg_color_bar_qcfa_rotate_period_shift);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_color_bar_cfg + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_color_bar_cfg=0x%x",
			hw->hw_idx,
			vc_slot, val);

	val = stream->hbi;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_hbi_cfg + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_hbi_cfg=0x%x",
			hw->hw_idx,
			vc_slot, val);

	val = stream->vbi;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_vbi_cfg + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_vbi_cgf=0x%x",
			hw->hw_idx,
			vc_slot, val);

	cam_io_w_mb(0x12345678,
		soc_info->reg_map[0].mem_base +
		tpg_reg->tpg_vc0_lfsr_seed + (0x60 * vc_slot));

	val = ((0 << tpg_reg->tpg_num_frames_shift_val) |
		((num_dts-1) << tpg_reg->tpg_num_dts_shift_val) |
		stream->vc);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc0_cfg0 + (0x60 * vc_slot));
	CAM_DBG(CAM_TPG, "TPG[%d] vc%d_cfg0=0x%x",
			hw->hw_idx,
			vc_slot, val);
	if (hw->hw_info->shdr_overlap == 1) {
		cam_io_w_mb(hw->hw_info->shdr_overlap << tpg_reg->tpg_overlap_shdr_en_shift,
			soc_info->reg_map[0].mem_base + tpg_reg->tpg_ctrl);
	}
	if (hw->hw_info->shdr_offset_num_batch >= 0 && vc_slot > 0) {
		val =  (VC1_GAIN << tpg_reg->tpg_gain_shift);
		val |= (hw->hw_info->shdr_offset_num_batch <<
				tpg_reg->tpg_shdr_offset_num_batch_shift);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
					tpg_reg->tpg_vc1_gain_cfg + (0x60 * (vc_slot-1)));
		val =  ((stream->shdr_line_offset0 * vc_slot)
			<< tpg_reg->tpg_shdr_line_offset0_shift);
		val |= ((stream->shdr_line_offset1 * vc_slot)
			<< tpg_reg->tpg_shdr_line_offset1_shift);
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_vc1_shdr_cfg + (0x60 * (vc_slot-1)));
		CAM_DBG(CAM_TPG, "TPG[%d] vc%d_cfg0=0x%x shdr",
			hw->hw_idx,
			vc_slot, val);
	}
	return 0;
}

static int tpg_hw_v_1_3_reset(
	struct tpg_hw *hw, void *data)
{
	struct cam_hw_soc_info *soc_info = NULL;
	uint32_t val;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}
	tpg_reg  = hw->hw_info->hw_data;

	soc_info = hw->soc_info;

	/* Clear out tpg_ctrl and irqs before reset */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base + tpg_reg->tpg_ctrl);

	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_top_irq_mask);

	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_top_irq_clear);

	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_top_irq_cmd);

	cam_io_w_mb(1, soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_top_clear);

	/* Read the version */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_hw_version);
	CAM_INFO(CAM_TPG, "TPG[%d] TPG HW version: 0x%x started",
			hw->hw_idx, val);
	return 0;
}

int tpg_hw_v_1_3_process_cmd(
	struct tpg_hw *hw,
	uint32_t       cmd,
	void          *arg)
{
	int rc = 0;
	if (hw == NULL) {
		CAM_ERR(CAM_TPG, "invalid argument");
		return -EINVAL;
	}
	switch(cmd) {
	case TPG_CONFIG_VC:
	{
		if (hw->stream_version == 1) {
			struct vc_config_args *vc_config =
				(struct vc_config_args *)arg;

			if (vc_config == NULL) {
				CAM_ERR(CAM_TPG, "invalid argument");
				return -EINVAL;
			}
			rc = configure_vc(hw,
				vc_config->vc_slot,
				vc_config->num_dts,
				vc_config->stream);
		} else if (hw->stream_version == 3) {
			struct vc_config_args_v3 *vc_config_v3 =
				(struct vc_config_args_v3 *)arg;

			if (vc_config_v3 == NULL) {
				CAM_ERR(CAM_TPG, "invalid argument");
				return -EINVAL;
			}
			rc = configure_vc_v3(hw,
				vc_config_v3->vc_slot,
				vc_config_v3->num_dts,
				vc_config_v3->stream);
		}
	}
	break;
	case TPG_CONFIG_DT:
	{
		if (hw->stream_version == 1) {
			struct dt_config_args *dt_config =
				(struct dt_config_args *)arg;

			if (dt_config == NULL) {
				CAM_ERR(CAM_TPG, "invalid argument");
				return -EINVAL;
			}
			rc = configure_dt(hw,
				dt_config->vc_slot,
				dt_config->dt_slot,
				dt_config->stream);
		} else if (hw->stream_version == 3) {
			struct dt_config_args_v3 *dt_config_v3 =
				(struct dt_config_args_v3 *)arg;

			if (dt_config_v3 == NULL) {
				CAM_ERR(CAM_TPG, "invalid argument");
				return -EINVAL;
			}
			rc = configure_dt_v3(hw,
				dt_config_v3->vc_slot,
				dt_config_v3->dt_slot,
				dt_config_v3->stream);
		}
	}
	break;
	case TPG_CONFIG_CTRL:
	{
		struct global_config_args *global_args =
			(struct global_config_args *)arg;
		rc = configure_global_configs(hw,
				global_args->num_vcs,
				global_args->globalconfig);
	}
	break;
	default:
		CAM_ERR(CAM_TPG, "invalid argument");
		break;
	}
	return rc;
}

int tpg_hw_v_1_3_start(struct tpg_hw *hw, void *data)
{
	CAM_DBG(CAM_TPG, "TPG V1.3 HWL start");
	return 0;
}

int tpg_hw_v_1_3_stop(struct tpg_hw *hw, void *data)
{
	CAM_DBG(CAM_TPG, "TPG V1.3 HWL stop");
	tpg_hw_v_1_3_reset(hw, data);
	return 0;
}

int tpg_hw_v_1_3_dump_status(struct tpg_hw *hw, void *data)
{
	struct cam_hw_soc_info *soc_info = NULL;
	uint32_t val;
	struct cam_tpg_ver_1_3_reg_offset *tpg_reg = NULL;

	if (!hw || !hw->hw_info || !hw->hw_info->hw_data) {
		CAM_ERR(CAM_TPG, "invalid params");
		return -EINVAL;
	}

	tpg_reg  = hw->hw_info->hw_data;

	soc_info = hw->soc_info;
	CAM_DBG(CAM_TPG, "TPG V1.3 HWL status dump");
	/* Read the version */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_hw_status);
	CAM_INFO(CAM_TPG, "TPG[%d] TPG HW status: 0x%x started",
			hw->hw_idx, val);
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			tpg_reg->tpg_top_irq_status);
	CAM_INFO(CAM_TPG, "TPG[%d] TPG HW irq status: 0x%x started",
			hw->hw_idx, val);

	return 0;
}

int tpg_hw_v_1_3_init(struct tpg_hw *hw, void *data)
{
	CAM_DBG(CAM_TPG, "TPG V1.3 HWL init");
	tpg_hw_v_1_3_reset(hw, data);
	return 0;
}

static int tpg_1_3_get_xcfa_test(void *data, u64 *val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "get xcfa test %d", hw->hw_info->xcfa_debug);
	*val = hw->hw_info->xcfa_debug;
	return 0;
}
static int tpg_1_3_get_shdr_overlap_test(void *data, u64 *val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "get shdr test : %d", hw->hw_info->shdr_overlap);
	*val = hw->hw_info->shdr_overlap;
	return 0;
}
static int tpg_1_3_get_shdr_offset_num_batch(void *data, u64 *val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "get shdr_num_batch : %d", hw->hw_info->shdr_offset_num_batch);
	*val = hw->hw_info->shdr_offset_num_batch;
	return 0;
}
static int tpg_1_3_get_shdr_line_offset0(void *data, u64 *val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "get shdr_offset0 : %d", hw->hw_info->shdr_line_offset0);
	*val = hw->hw_info->shdr_line_offset0;
	return 0;
}
static int tpg_1_3_get_shdr_line_offset1(void *data, u64 *val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "get shdr_offset1 : %d", hw->hw_info->shdr_line_offset1);
	*val = hw->hw_info->shdr_line_offset1;
	return 0;
}


static int tpg_1_3_set_xcfa_test(void *data, u64 val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "set xcfa test prev : %d", hw->hw_info->xcfa_debug);
	hw->hw_info->xcfa_debug = val;
	return 0;
}
static int tpg_1_3_set_shdr_overlap_test(void *data, u64 val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "set shdr test prev : %d", hw->hw_info->shdr_overlap);
	hw->hw_info->shdr_overlap = val;
	return 0;
}
static int tpg_1_3_set_shdr_offset_num_batch(void *data, u64 val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "set shdr_num_batch : %d", hw->hw_info->shdr_offset_num_batch);
	hw->hw_info->shdr_offset_num_batch = val;
	return 0;
}
static int tpg_1_3_set_shdr_line_offset0(void *data, u64 val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "set shdr_offset0 : %d", hw->hw_info->shdr_line_offset0);
	hw->hw_info->shdr_line_offset0 = val;
	return 0;
}
static int tpg_1_3_set_shdr_line_offset1(void *data, u64 val)
{
	struct tpg_hw *hw = (struct tpg_hw *)data;

	CAM_INFO(CAM_TPG, "set shdr_offset1 : %d", hw->hw_info->shdr_line_offset1);
	hw->hw_info->shdr_line_offset1 = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tpg_1_3_xcfa_test,
		&tpg_1_3_get_xcfa_test,
		&tpg_1_3_set_xcfa_test,
		"%16d");

DEFINE_SIMPLE_ATTRIBUTE(tpg_1_3_shdr_overlap_test,
		&tpg_1_3_get_shdr_overlap_test,
		&tpg_1_3_set_shdr_overlap_test,
		"%16d");
DEFINE_SIMPLE_ATTRIBUTE(tpg_1_3_shdr_offset_num_batch,
		&tpg_1_3_get_shdr_offset_num_batch,
		&tpg_1_3_set_shdr_offset_num_batch,
		"%16d");
DEFINE_SIMPLE_ATTRIBUTE(tpg_1_3_shdr_line_offset0,
		&tpg_1_3_get_shdr_line_offset0,
		&tpg_1_3_set_shdr_line_offset0,
		"%16d");
DEFINE_SIMPLE_ATTRIBUTE(tpg_1_3_shdr_line_offset1,
		&tpg_1_3_get_shdr_line_offset1,
		&tpg_1_3_set_shdr_line_offset1,
		"%16d");


int tpg_1_3_layer_init(struct tpg_hw *hw)
{
	int rc = 0;
	struct dentry *dbgfileptr_parent = NULL;
	char dir_name[160];

	snprintf(dir_name, sizeof(dir_name), "tpg%d",
		hw->hw_idx);

	dbgfileptr_parent = debugfs_create_dir(dir_name, NULL);
	if (!dbgfileptr_parent) {
		CAM_ERR(CAM_TPG, "Debug fs could not create directory");
		rc = -ENOENT;
	}
	debugfs_create_file("tpg_xcfa_test", 0644,
		dbgfileptr_parent, hw, &tpg_1_3_xcfa_test);
	debugfs_create_file("tpg_shdr_overlap_test", 0644,
		dbgfileptr_parent, hw, &tpg_1_3_shdr_overlap_test);
	debugfs_create_file("tpg_shdr_offset_num_batch", 0644,
		dbgfileptr_parent, hw, &tpg_1_3_shdr_offset_num_batch);
	debugfs_create_file("tpg_shdr_line_offset0", 0644,
		dbgfileptr_parent, hw, &tpg_1_3_shdr_line_offset0);
	debugfs_create_file("tpg_shdr_line_offset1", 0644,
		dbgfileptr_parent, hw, &tpg_1_3_shdr_line_offset1);
	CAM_INFO(CAM_TPG, "Layer init called");
	return rc;
}
