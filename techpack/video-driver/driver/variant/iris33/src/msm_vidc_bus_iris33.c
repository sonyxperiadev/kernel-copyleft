// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "perf_static_model.h"
#include "msm_vidc_debug.h"

/* 100x */
static u32 dpbopb_ubwc30_cr_table_cratio_iris33[7][12] = {
	{237, 399, 272, 137, 225, 158, 185, 259, 203, 138, 167, 152},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{237, 399, 272, 137, 225, 158, 185, 259, 203, 138, 167, 152},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
};

/* 100x */
static u32 rpb_ubwc30_cr_table_cratio_iris33[7][12] = {
	{193, 294, 218, 135, 214, 155, 175, 241, 191, 139, 162, 149},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{193, 294, 218, 135, 214, 155, 175, 241, 191, 139, 162, 149},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
};

/* 100x */
static u32 ipblossy_ubwc30_cr_table_cratio_iris33[7][12] = {
	{215, 215, 215, 174, 174, 174, 266, 266, 266, 231, 231, 231},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{215, 215, 215, 174, 174, 174, 266, 266, 266, 231, 231, 231},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
};

/* 100x */
static u32 ipblossless_ubwc30_cr_table_cratio_iris33[7][12] = {
	{185, 215, 194, 147, 178, 159, 162, 181, 169, 138, 161, 146},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{185, 215, 194, 147, 178, 159, 162, 181, 169, 138, 161, 146},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
};

/* 100x */
static u32 en_original_compression_factor_rgba_pwd_iris33 = 243;
/* 100x */
static u32 en_original_compression_factor_rgba_avg_iris33 = 454;

static u32 av1_num_tiles_iris33[7][3] = {
	{2, 1, 1},
	{4, 2, 2},
	{4, 2, 2},
	{4, 2, 2},
	{1, 1, 1},
	{2, 1, 1},
	{16, 4, 4},
};

/*                                H   I   J         K   L   M      N   O   P
 *         TotalW   Total R       Frequency         Write         Read
 * Name                           B   b   P         B   b   P      B   b   P
 * I3B4b1P    0.5    1.875        3   4   1         1   0   1      2   2   1
 * I1B2b1P    0.5    1.75         1   2   1         1   0   1      2   2   1
 * IbP        0.5    1.5          0   1   1         1   0   1      2   2   1
 * IPP        1      1            0   0   1         1   0   1      2   2   1
 * P          1      1            0   0   1         1   0   1      2   2   1
 * smallB     0      2            0   1   0         1   0   1      2   2   1
 * bigB       1      2            1   0   0         1   0   1      2   2   1
 *
 * Total W = SUMPRODUCT(H16:J16, K16 : M16) / SUM(H16:J16)
 * Total R = SUMPRODUCT(H16:J16, N16 : P16) / SUM(H16:J16)
 */

/* 1000x */
static u32 pineapple_en_readfactor[8] = {1000, 1500, 1750, 1875, 1000, 2000, 2000, 1000};
/* 1000x */
static u32 pineapple_en_writefactor[8] = {1000, 500, 500, 500, 1000, 0, 1000, 1000};
static u32 pineapple_en_frame_num_parallel = 1;

u32 calculate_number_lcus_iris33(u32 width, u32 height, u32 lcu_size)
{
	u32 mbs_width = (width % lcu_size) ?
		(width / lcu_size + 1) : (width / lcu_size);
	u32 mbs_height = (height % lcu_size) ?
		(height / lcu_size + 1) : (height / lcu_size);

	return mbs_width * mbs_height;
}

u32 calculate_number_ubwctiles_iris33(
		u32 width, u32 height, u32 tile_w, u32 tile_h)
{
	u32 tiles_width = (width % tile_w) ?
		(width / tile_w + 1) : (width / tile_w);
	u32 tiles_height = (height % tile_h) ?
		(height / tile_h + 1) : (height / tile_h);

	return tiles_width * tiles_height;
}

struct compression_factors {
	u32 dpb_cf_y;
	u32 dpb_cf_cbcr;
	u32 opb_cf_ycbcr;
	u32 dpb_cr_y;
	u32 ipb_cr_y;
	u32 ipb_cr;
} compression_factor;

u32 get_compression_factors(struct compression_factors *compression_factor,
		struct api_calculation_input codec_input)
{
	u8 cr_index_entry, cr_index_y, cr_index_c, cr_index_uni;
	u32 frame_width;
	u32 frame_height;

	frame_width = codec_input.frame_width;
	frame_height = codec_input.frame_height;
	if (frame_width * frame_height <= 1920 * 1080)
		cr_index_entry = 0;
	else
		cr_index_entry = 1;

	if (codec_input.bitdepth == CODEC_BITDEPTH_8) {
		/* NOT PWC or average and power case */
		if (codec_input.complexity_setting != 0) {
			cr_index_y = 0;
			cr_index_c = 1;
			cr_index_uni = 2;
		} else {
			cr_index_y = 3;
			cr_index_c = 4;
			cr_index_uni = 5;
		}
	} else {
		/* NOT PWC or average and power case */
		if (codec_input.complexity_setting != 0) {
			cr_index_y = 6;
			cr_index_c = 7;
			cr_index_uni = 8;
		} else {
			cr_index_y = 9;
			cr_index_c = 10;
			cr_index_uni = 11;
		}
	}

	if (codec_input.decoder_or_encoder == CODEC_DECODER) {
		compression_factor->dpb_cf_y =
			dpbopb_ubwc30_cr_table_cratio_iris33[cr_index_entry][cr_index_y];
		compression_factor->dpb_cf_cbcr =
			dpbopb_ubwc30_cr_table_cratio_iris33[cr_index_entry][cr_index_c];
		compression_factor->opb_cf_ycbcr =
			dpbopb_ubwc30_cr_table_cratio_iris33[cr_index_entry][cr_index_uni];

		if ((codec_input.regression_mode == 3) &&
			/* input cr numbers from interface */
			((codec_input.cr_dpb != 0) || (codec_input.cr_opb != 0))) {
			compression_factor->dpb_cf_y = codec_input.cr_dpb;
			compression_factor->dpb_cf_cbcr = codec_input.cr_dpb;
			compression_factor->opb_cf_ycbcr = codec_input.cr_opb;
		}
	} else { /* encoder */
		/*
		 * IPB CR Table Choice; static sheet (if framewidth<3840, use lossless table)
		 * (else, use lossy table)
		 * stick to this choice for SW purpose (no change for SW)
		 */
		if (frame_width < 3840) {
			compression_factor->ipb_cr =
				ipblossless_ubwc30_cr_table_cratio_iris33[cr_index_entry]
					[cr_index_uni];
			compression_factor->ipb_cr_y =
				ipblossless_ubwc30_cr_table_cratio_iris33[cr_index_entry]
					[cr_index_y];
		} else {
			compression_factor->ipb_cr =
				ipblossy_ubwc30_cr_table_cratio_iris33[cr_index_entry]
					[cr_index_uni];
			compression_factor->ipb_cr_y =
				ipblossy_ubwc30_cr_table_cratio_iris33[cr_index_entry]
					[cr_index_y];
		}

		compression_factor->dpb_cf_y =
			rpb_ubwc30_cr_table_cratio_iris33[cr_index_entry][cr_index_y];

		compression_factor->dpb_cf_cbcr =
			rpb_ubwc30_cr_table_cratio_iris33[cr_index_entry][cr_index_c];

		if ((codec_input.regression_mode == 3) &&
			/* input cr from interface */
			((codec_input.cr_ipb != 0) || (codec_input.cr_rpb != 0))) {
			compression_factor->dpb_cf_y = codec_input.cr_rpb;
			compression_factor->dpb_cf_cbcr = codec_input.cr_rpb;
			compression_factor->ipb_cr_y = codec_input.cr_ipb;
		}
	}

	return 0;
}

static int calculate_bandwidth_decoder_iris33(
		struct api_calculation_input codec_input,
		struct api_calculation_bw_output *codec_output)
{
	/* common control parameters */
	u32 frame_width;
	u32 frame_height;
	u32 frame_lcu_size = 16; /* initialized to h264 */
	u32 lcu_per_frame;
	u32 target_bitrate;
	u32 collocated_bytes_per_lcu = 16; /* initialized to h264 */
	u32 av1d_segment_read_per_lcu;
	u32 av1d_fe_leftlinebuffer_perlcu_tileboudary;

	u32 frame420_y_bw_linear_8bpp;
	u32 frame420_y_bw_no_ubwc_tile_10bpp;
	u32 frame420_y_bw_linear_10bpp;

	u16 ubwc_tile_w;
	u16 ubwc_tile_h;

	u32 dpb_compression_factor_y;
	u32 dpb_compression_factor_cbcr;

	u32 reconstructed_write_bw_factor_rd;
	u32 reference_y_read_bw_factor;
	u32 reference_cbcr_read_bw_factor;

	/* decoder control parameters */
	u32 decoder_vsp_read_factor = 6;
	u32 bins_to_bits_factor = 4;

	u32 dpb_to_opb_ratios_ds = 1;

	u8 llc_enabled_ref_y_rd;
	u8 llc_enable_ref_crcb_rd;
	u8 llc_enabled_bse_tlb;

	/* this is for 2pipe and 1pipe LLC */
	u8 llc_enable_probtable_av1d_21pipe = 0;

	u32 opb_compression_factor_ycbcr;
	u32 dpb_ubwc_tile_width_pixels;
	u32 dpb_ubwc_tile_height_pixels;
	u32 decoder_frame_complexity_factor;
	u32 llc_saving = 130; /* Initialized to H264 */

	u16 av1_tile_numbers;
	u32 av1_collated_seg_buffer_rd_wr;
	/* need divide by 1M at later step; */
	u32 av1_probability_table_rdwr_bytesperframe = 22784;
	u32 av1_fe_left_line_buffer_rdwr;

	u32 bse_tlb_byte_per_lcu = 0;

	u32 large_bw_calculation_fp = 0;

	if (codec_input.pipe_num == 4) {
		llc_enabled_ref_y_rd = 1;
		llc_enable_ref_crcb_rd = 1;
		llc_enabled_bse_tlb = 1;
	} else if (codec_input.pipe_num == 2) {
		llc_enabled_ref_y_rd = 0; // llc 128kb for palawan
		llc_enable_ref_crcb_rd = 0;
		llc_enabled_bse_tlb = (codec_input.status_llc_onoff == 1) ? 1 : 0;
	}

	llc_enabled_ref_y_rd = (codec_input.status_llc_onoff) ? 1 : 0;
	llc_enable_ref_crcb_rd = (codec_input.status_llc_onoff) ? 1 : 0;
	/* H265D BSE tlb in LLC will be pored in Kailua */
	llc_enabled_bse_tlb = (codec_input.status_llc_onoff) ? 1 : 0;

	frame_width = codec_input.frame_width;
	frame_height = codec_input.frame_height;
	if ((codec_input.codec == CODEC_H264) ||
		(codec_input.codec == CODEC_H264_CAVLC)) {
		frame_lcu_size = 16;
		collocated_bytes_per_lcu = 16;
		llc_saving = 130;
	} else if (codec_input.codec == CODEC_HEVC) {
		if (codec_input.lcu_size == 32) {
			frame_lcu_size = 32;
			collocated_bytes_per_lcu = 64;
			llc_saving = 114;
		} else if (codec_input.lcu_size == 64) {
			frame_lcu_size = 64;
			collocated_bytes_per_lcu = 256;
			llc_saving = 107;
		}
	} else if (codec_input.codec == CODEC_VP9) {
		if (codec_input.lcu_size == 32) {
			frame_lcu_size = 32;
			collocated_bytes_per_lcu = 64;
			llc_saving = 114;
		} else if (codec_input.lcu_size == 64) {
			frame_lcu_size = 64;
			collocated_bytes_per_lcu = 256;
			llc_saving = 107;
		}
	} else if (codec_input.codec == CODEC_AV1) {
		u32 av1d_leftline_cdef = (2944 + 896 + 896);
		u32 av1d_leftline_scaling = (2176 + 1408 + 1408);
		u32 av1d_leftline_fg = (1280);
		u32 av1d_leftline_lr = (1536 + 1024 + 1024);

		av1d_fe_leftlinebuffer_perlcu_tileboudary =
			av1d_leftline_cdef + av1d_leftline_scaling +
			av1d_leftline_fg + av1d_leftline_lr;

		if (codec_input.lcu_size == 128) {
			frame_lcu_size = 128;
			collocated_bytes_per_lcu = 4 * 512;
			av1d_segment_read_per_lcu = 512;
			llc_saving = 104;
		} else if (codec_input.lcu_size == 32) {
			frame_lcu_size = 32;
			collocated_bytes_per_lcu = 4 * 512 / (128 * 128 / 32 / 32);
			av1d_segment_read_per_lcu = 512 / (128 * 128 / 32 / 32);
			av1d_fe_leftlinebuffer_perlcu_tileboudary =
				av1d_fe_leftlinebuffer_perlcu_tileboudary / (128 * 128 / 32 / 32);
			llc_saving = 114;
		} else if (codec_input.lcu_size == 64) {
			frame_lcu_size = 64;
			collocated_bytes_per_lcu = 4 * 512 / (128 * 128 / 64 / 64);
			av1d_segment_read_per_lcu = 512 / (128 * 128 / 64 / 64);
			av1d_fe_leftlinebuffer_perlcu_tileboudary =
				av1d_fe_leftlinebuffer_perlcu_tileboudary / (128 * 128 / 64 / 64);
			llc_saving = 107;
		}
	}

	lcu_per_frame =
		calculate_number_lcus_iris33(frame_width, frame_height, frame_lcu_size);

	target_bitrate = (u32)(codec_input.bitrate_mbps); /* Mbps */

	ubwc_tile_w = (codec_input.bitdepth == CODEC_BITDEPTH_8) ? 32 : 48;
	ubwc_tile_h = (codec_input.bitdepth == CODEC_BITDEPTH_8) ? 8 : 4;

	frame420_y_bw_linear_8bpp =
		((calculate_number_ubwctiles_iris33(frame_width, frame_height, 32, 8) *
		256 * codec_input.frame_rate + 999) / 1000 + 999) / 1000;

	frame420_y_bw_no_ubwc_tile_10bpp =
		((calculate_number_ubwctiles_iris33(frame_width, frame_height, 48, 4) *
		256 * codec_input.frame_rate + 999) / 1000 + 999) / 1000;
	frame420_y_bw_linear_10bpp = ((frame_width * frame_height *
		codec_input.frame_rate * 2 + 999) / 1000 + 999) / 1000;

	/* TODO Integrate Compression Ratio returned by FW */
	get_compression_factors(&compression_factor, codec_input);
	dpb_compression_factor_y = compression_factor.dpb_cf_y;
	dpb_compression_factor_cbcr = compression_factor.dpb_cf_cbcr;
	opb_compression_factor_ycbcr = compression_factor.opb_cf_ycbcr;

	dpb_ubwc_tile_width_pixels = ubwc_tile_w;

	dpb_ubwc_tile_height_pixels = ubwc_tile_h;

	decoder_frame_complexity_factor =
		(codec_input.complexity_setting == 0) ?
		400 : ((codec_input.complexity_setting == 1) ? 266 : 100);

	reconstructed_write_bw_factor_rd = (codec_input.complexity_setting == 0) ?
		105 : 100;

	reference_y_read_bw_factor = llc_saving;

	reference_cbcr_read_bw_factor = llc_saving;

	if (codec_input.codec == CODEC_AV1) {
		u8 av1tile_index_entry, av1tile_complexity;

		if (frame_width * frame_height <= 1280 * 720)
			av1tile_index_entry = 4;
		else if (frame_width * frame_height <= 1920 * 1080)
			av1tile_index_entry = 0;
		else if (frame_width * frame_height <= 2560 * 1440)
			av1tile_index_entry = 5;
		else if (frame_width * frame_height <= 4096 * 2304)
			av1tile_index_entry = 1;
		else
			av1tile_index_entry = 6;

		/* NOT PWC or average and power case */
		if (codec_input.complexity_setting != 0)
			av1tile_complexity = 1;
		else
			av1tile_complexity = 0;

		av1_tile_numbers = av1_num_tiles_iris33[av1tile_index_entry][av1tile_complexity];

		/* these bw can be ignored */
		av1_collated_seg_buffer_rd_wr =
			((av1d_segment_read_per_lcu * lcu_per_frame *
			codec_input.frame_rate + 999) / 1000 + 999) / 1000;

		av1_fe_left_line_buffer_rdwr =
			(((av1d_fe_leftlinebuffer_perlcu_tileboudary *
			frame_height * (av1_tile_numbers > 1 ? av1_tile_numbers / 2 : 0)
			+ 999) / 1000 + 999) / 1000 + (frame_lcu_size - 1)) / frame_lcu_size;
	}

	if (codec_input.codec == CODEC_HEVC) {
		if (codec_input.lcu_size == 32)
			bse_tlb_byte_per_lcu = 64;
		else if (codec_input.lcu_size == 16)
			bse_tlb_byte_per_lcu = 32;
		else
			bse_tlb_byte_per_lcu = 128;
	} else if ((codec_input.codec == CODEC_H264) ||
		(codec_input.codec == CODEC_H264_CAVLC)) {
		bse_tlb_byte_per_lcu = 64;
	} else if (codec_input.codec == CODEC_VP9) {
		bse_tlb_byte_per_lcu = 304;
	} else if (codec_input.codec == CODEC_AV1) {
		if (codec_input.lcu_size == 128)
			bse_tlb_byte_per_lcu = 2064;
		else if (codec_input.lcu_size == 64)
			bse_tlb_byte_per_lcu = 1056;
		else if (codec_input.lcu_size == 32)
			bse_tlb_byte_per_lcu = 2064 / (128 * 128 / 32 / 32);
	}

	codec_output->noc_bw_rd = 0;
	codec_output->noc_bw_wr = 0;
	codec_output->ddr_bw_rd = 0;
	codec_output->ddr_bw_wr = 0;

	large_bw_calculation_fp = 0;
	large_bw_calculation_fp = ((target_bitrate *
		decoder_vsp_read_factor + 7) / 8);

	codec_output->vsp_read_noc = large_bw_calculation_fp;

	codec_output->vsp_read_ddr = codec_output->vsp_read_noc;

	large_bw_calculation_fp = ((target_bitrate *
		bins_to_bits_factor + 7) / 8);

	codec_output->vsp_write_noc = large_bw_calculation_fp;
	codec_output->vsp_write_ddr = codec_output->vsp_write_noc;

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->vsp_read_noc;
	codec_output->ddr_bw_rd += codec_output->vsp_read_ddr;
	codec_output->noc_bw_wr += codec_output->vsp_write_noc;
	codec_output->ddr_bw_wr += codec_output->vsp_write_ddr;

	large_bw_calculation_fp = 0;
	large_bw_calculation_fp = ((collocated_bytes_per_lcu *
		lcu_per_frame * codec_input.frame_rate + 999) / 1000 + 999) / 1000;
	codec_output->collocated_rd_noc = large_bw_calculation_fp;
	codec_output->collocated_wr_noc = codec_output->collocated_rd_noc;
	codec_output->collocated_rd_ddr = codec_output->collocated_rd_noc;
	codec_output->collocated_wr_ddr = codec_output->collocated_wr_noc;

	codec_output->collocated_rd_wr_total_noc =
		(u32)(codec_output->collocated_rd_noc + codec_output->collocated_wr_noc);

	codec_output->collocated_rd_wr_total_ddr =
		codec_output->collocated_rd_wr_total_noc;

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->collocated_rd_noc;
	codec_output->noc_bw_wr += codec_output->collocated_wr_noc;
	codec_output->ddr_bw_rd += codec_output->collocated_rd_ddr;
	codec_output->ddr_bw_wr += codec_output->collocated_wr_ddr;

	large_bw_calculation_fp = 0;
	large_bw_calculation_fp = ((codec_input.bitdepth == CODEC_BITDEPTH_8) ?
		frame420_y_bw_linear_8bpp :
		frame420_y_bw_no_ubwc_tile_10bpp) * decoder_frame_complexity_factor;

	large_bw_calculation_fp =
		(large_bw_calculation_fp + dpb_compression_factor_y - 1) /
		dpb_compression_factor_y;

	codec_output->dpb_rd_y_noc = large_bw_calculation_fp;

	large_bw_calculation_fp = ((codec_input.bitdepth == CODEC_BITDEPTH_8) ?
		frame420_y_bw_linear_8bpp : frame420_y_bw_no_ubwc_tile_10bpp) *
		decoder_frame_complexity_factor;

	large_bw_calculation_fp =
		(large_bw_calculation_fp + dpb_compression_factor_cbcr - 1) /
		dpb_compression_factor_cbcr / 2;

	codec_output->dpb_rd_crcb_noc = large_bw_calculation_fp;
	codec_output->dpb_rdwr_duetooverlap_noc = 0;

	large_bw_calculation_fp = ((codec_input.bitdepth == CODEC_BITDEPTH_8) ?
		frame420_y_bw_linear_8bpp : frame420_y_bw_no_ubwc_tile_10bpp) *
		reconstructed_write_bw_factor_rd;

	large_bw_calculation_fp = ((codec_input.bitdepth == CODEC_BITDEPTH_8) ?
		frame420_y_bw_linear_8bpp : frame420_y_bw_no_ubwc_tile_10bpp) *
		reconstructed_write_bw_factor_rd;

	large_bw_calculation_fp = large_bw_calculation_fp *
		(dpb_compression_factor_y / 2 + dpb_compression_factor_cbcr);

	large_bw_calculation_fp = (large_bw_calculation_fp + dpb_compression_factor_y - 1) /
		dpb_compression_factor_y;

	large_bw_calculation_fp =
		(large_bw_calculation_fp + dpb_compression_factor_cbcr - 1) /
		dpb_compression_factor_cbcr;

	codec_output->dpb_wr_noc = large_bw_calculation_fp;

	codec_output->dpb_rd_y_ddr = (llc_enabled_ref_y_rd) ?
		((codec_output->dpb_rd_y_noc * 100 + reference_y_read_bw_factor - 1) /
		reference_y_read_bw_factor) : codec_output->dpb_rd_y_noc;

	codec_output->dpb_rd_crcb_ddr = (llc_enable_ref_crcb_rd) ?
		((codec_output->dpb_rd_crcb_noc * 100 +
		reference_cbcr_read_bw_factor - 1) /
		reference_cbcr_read_bw_factor) : codec_output->dpb_rd_crcb_noc;

	codec_output->dpb_rdwr_duetooverlap_ddr = 0;
	codec_output->dpb_wr_ddr = codec_output->dpb_wr_noc;

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->dpb_rd_y_noc;
	codec_output->noc_bw_rd += codec_output->dpb_rd_crcb_noc;
	codec_output->noc_bw_rd += codec_output->dpb_rdwr_duetooverlap_noc;
	codec_output->noc_bw_wr += codec_output->dpb_wr_noc;
	codec_output->ddr_bw_rd += codec_output->dpb_rd_y_ddr;
	codec_output->ddr_bw_rd += codec_output->dpb_rd_crcb_ddr;
	codec_output->ddr_bw_rd += codec_output->dpb_rdwr_duetooverlap_ddr;
	codec_output->ddr_bw_wr += codec_output->dpb_wr_ddr;

	if (codec_input.linear_opb || codec_input.split_opb) {
		if (codec_input.linear_opb) {
			if (codec_input.bitdepth == CODEC_BITDEPTH_8) {
				large_bw_calculation_fp = ((frame420_y_bw_linear_8bpp) *
				3 / 2 / dpb_to_opb_ratios_ds);

				codec_output->opb_write_total_noc = large_bw_calculation_fp;
			} else {
				large_bw_calculation_fp = ((frame420_y_bw_linear_10bpp) *
				3 / 2 / dpb_to_opb_ratios_ds);

				codec_output->opb_write_total_noc = large_bw_calculation_fp;
			}
		} else { /* (CODEC_INPUT.split_opb) */
			if (codec_input.bitdepth == CODEC_BITDEPTH_8) {
				large_bw_calculation_fp =
					(frame420_y_bw_linear_8bpp * 3 / 2 / dpb_to_opb_ratios_ds *
					100 + opb_compression_factor_ycbcr - 1) /
					opb_compression_factor_ycbcr;

				codec_output->opb_write_total_noc = large_bw_calculation_fp;
			} else {
				large_bw_calculation_fp =
					(frame420_y_bw_no_ubwc_tile_10bpp * 3 / 2 /
					dpb_to_opb_ratios_ds * 100 +
					opb_compression_factor_ycbcr - 1) /
					opb_compression_factor_ycbcr;

				codec_output->opb_write_total_noc = large_bw_calculation_fp;
			}
		}
	} else {
		codec_output->opb_write_total_noc = 0;
	}

	codec_output->opb_write_total_ddr = codec_output->opb_write_total_noc;

	/* accumulation */
	codec_output->noc_bw_wr += codec_output->opb_write_total_noc;
	codec_output->ddr_bw_wr += codec_output->opb_write_total_ddr;

	large_bw_calculation_fp = ((bse_tlb_byte_per_lcu * lcu_per_frame *
		codec_input.frame_rate + 999) / 1000 + 999) / 1000;

	codec_output->bse_tlb_rd_noc = large_bw_calculation_fp;

	if (llc_enabled_bse_tlb)
		codec_output->bse_tlb_rd_ddr = 0;
	else
		codec_output->bse_tlb_rd_ddr = codec_output->bse_tlb_rd_noc;

	codec_output->bse_tlb_wr_noc = codec_output->bse_tlb_rd_noc;

	if (llc_enabled_bse_tlb)
		codec_output->bse_tlb_wr_ddr = 0;
	else
		codec_output->bse_tlb_wr_ddr = codec_output->bse_tlb_wr_noc;

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->bse_tlb_rd_noc;
	codec_output->ddr_bw_rd += codec_output->bse_tlb_rd_ddr;
	codec_output->noc_bw_wr += codec_output->bse_tlb_wr_noc;
	codec_output->ddr_bw_wr += codec_output->bse_tlb_wr_ddr;

	if (codec_input.codec == CODEC_AV1) {
		codec_output->statistics_rd_noc = (av1_collated_seg_buffer_rd_wr +
			av1_probability_table_rdwr_bytesperframe * av1_tile_numbers /
			1000 / 1000 + av1_fe_left_line_buffer_rdwr);

		codec_output->statistics_wr_noc = (av1_collated_seg_buffer_rd_wr +
			av1_probability_table_rdwr_bytesperframe * av1_tile_numbers /
			1000 / 1000 + av1_fe_left_line_buffer_rdwr);

		if (llc_enable_probtable_av1d_21pipe) {
			/* assert(CODEC_INPUT.pipe_num != 4); */
			codec_output->statistics_rd_ddr = codec_output->statistics_rd_noc -
				av1_probability_table_rdwr_bytesperframe *
				av1_tile_numbers / 1000 / 1000;

			codec_output->statistics_wr_ddr = codec_output->statistics_wr_noc -
				av1_probability_table_rdwr_bytesperframe *
				av1_tile_numbers / 1000 / 1000;
		} else {
			codec_output->statistics_rd_ddr = codec_output->statistics_rd_noc;
			codec_output->statistics_wr_ddr = codec_output->statistics_wr_noc;
		}

		/* accumulation */
		codec_output->noc_bw_rd += codec_output->statistics_rd_noc;
		codec_output->ddr_bw_rd += codec_output->statistics_rd_ddr;
		codec_output->noc_bw_wr += codec_output->statistics_wr_noc;
		codec_output->ddr_bw_wr += codec_output->statistics_wr_ddr;
	}


	codec_output->mmu_rd_ddr = 0;
	codec_output->mmu_rd_noc = 0;
	/* accumulation */
	codec_output->noc_bw_rd += codec_output->mmu_rd_noc;
	codec_output->ddr_bw_rd += codec_output->mmu_rd_ddr;

	return 0;
}

static int calculate_bandwidth_encoder_iris33(
		struct api_calculation_input codec_input,
		struct api_calculation_bw_output *codec_output)
{
	/* common control parameters */
	u32 frame_width;
	u32 frame_height;
	u32 frame_lcu_size;
	u32 lcu_per_frame;
	u32 target_bitrate;
	u32 collocated_bytes_per_lcu;

	u32 frame420_y_bw_linear_8bpp;
	u32 frame420_y_bw_no_ubwc_tile_10bpp;
	u32 frame420_y_bw_linear_10bpp;

	u16 ubwc_tile_w;
	u16 ubwc_tile_h;

	u32 dpb_compression_factor_y;
	u32 dpb_compression_factor_cbcr;

	u32 reconstructed_write_bw_factor_rd;
	u32 reference_y_read_bw_factor;
	u32 reference_crcb_read_bw_factor;

	u32 en_vertical_tiles_width = 960;
	u32 en_search_windows_size_horizontal = 96;

	u8 en_rotation_90_270 = 0;
	/* TODO Can we use (codec_input.status_llc_onoff) for enc_llc_*? */
	u8 en_llc_enable_ref_rd_crcb = 0;
	u8 en_llc_enable_rec_wr_uncompleted = 0;
	u8 en_llc_enable_ref_rd_y_overlap = 0;

	u32 en_bins_to_bits_factor = 4;

	u32 en_tile_number;
	u32 ipb_compression_factor_y;
	u32 ipb_compression_factor;

	u32 large_bw_calculation_fp = 0;

	/* TODO Are these really needed in Encoder? */
	u32 bse_tlb_byte_per_lcu = 0;
	u8 llc_enabled_bse_tlb = 1;

	/*H265D BSE tlb in LLC will be pored in Kailua */
	llc_enabled_bse_tlb = (codec_input.status_llc_onoff) ? 1 : 0;

	/* encoder control parameters
	 * iris3 2pipe uses 768 vertical tile width for IPP GoP
	 */
	if (codec_input.pipe_num == 2) {
		en_vertical_tiles_width =
		(codec_input.hierachical_layer == CODEC_GOP_IPP) ? 768 : 576;
		// search ranges for iris3 2pipe
		en_search_windows_size_horizontal =
		(codec_input.hierachical_layer == CODEC_GOP_IPP) ? 192 : 96;
	}

	frame_width = codec_input.frame_width;
	frame_height = codec_input.frame_height;
	if ((codec_input.codec == CODEC_H264) ||
		(codec_input.codec == CODEC_H264_CAVLC)) {
		frame_lcu_size = 16;
		collocated_bytes_per_lcu = 16;
	} else if (codec_input.codec == CODEC_HEVC) {
		frame_lcu_size = 32;
		collocated_bytes_per_lcu = 64;
	} else {
		/* TODO What is the value for VP9, AV1? */
		frame_lcu_size = 16;
		collocated_bytes_per_lcu = 16; /* TODO Fixes Uninitialized compilation error. */
	}

	lcu_per_frame =
		calculate_number_lcus_iris33(frame_width, frame_height, frame_lcu_size);

	bse_tlb_byte_per_lcu = 16; /* TODO Should be in common declaration */

	target_bitrate = (u32)(codec_input.bitrate_mbps); /* Mbps */

	ubwc_tile_w = (codec_input.bitdepth == CODEC_BITDEPTH_8) ? 32 : 48;
	ubwc_tile_h = (codec_input.bitdepth == CODEC_BITDEPTH_8) ? 8 : 4;

	/* yuv */
	if (codec_input.ipb_yuvrgb == 0) {
		frame420_y_bw_linear_8bpp =
			((calculate_number_ubwctiles_iris33(frame_width, frame_height,
			32, 8) * 256 * codec_input.frame_rate + 999) / 1000 + 999) / 1000;
	} else { /* RGBA */
		frame420_y_bw_linear_8bpp =
			((calculate_number_ubwctiles_iris33(frame_width, frame_height,
			16, 4) * 256 * codec_input.frame_rate + 999) / 1000 + 999) / 1000;
	}

	frame420_y_bw_no_ubwc_tile_10bpp =
		((calculate_number_ubwctiles_iris33(frame_width, frame_height, 48, 4) *
		256 * codec_input.frame_rate + 999) / 1000 + 999) / 1000;

	frame420_y_bw_linear_10bpp = ((frame_width * frame_height *
		codec_input.frame_rate * 2 + 999) / 1000 + 999) / 1000;

	/* TODO Integrate Compression Ratio returned by FW */
	get_compression_factors(&compression_factor, codec_input);
	dpb_compression_factor_y = compression_factor.dpb_cf_y;
	dpb_compression_factor_cbcr = compression_factor.dpb_cf_cbcr;
	ipb_compression_factor_y = compression_factor.ipb_cr_y;
	ipb_compression_factor = compression_factor.ipb_cr;

	en_tile_number = (frame_width % en_vertical_tiles_width) ?
		((frame_width / en_vertical_tiles_width) + 1) :
		(frame_width / en_vertical_tiles_width);

	en_tile_number = en_tile_number * 100;

	/* ceil is same as excel roundup (float, 0); */
	reconstructed_write_bw_factor_rd = ((en_tile_number - 100) * 2 *
		((codec_input.lcu_size + ubwc_tile_w - 1) / ubwc_tile_w) *
		ubwc_tile_w + (frame_width - 1)) / (frame_width)+100;

	reference_y_read_bw_factor = ((en_tile_number - 100) * 2 *
		((en_search_windows_size_horizontal + ubwc_tile_w - 1) / ubwc_tile_w) *
		ubwc_tile_w + (frame_width - 1)) / frame_width + 100;

	reference_crcb_read_bw_factor = 150;

	codec_output->noc_bw_rd = 0;
	codec_output->noc_bw_wr = 0;
	codec_output->ddr_bw_rd = 0;
	codec_output->ddr_bw_wr = 0;

	large_bw_calculation_fp = (target_bitrate * en_bins_to_bits_factor + 7) / 8;
	codec_output->vsp_read_noc = large_bw_calculation_fp;
	codec_output->vsp_read_ddr = codec_output->vsp_read_noc;
	large_bw_calculation_fp = (target_bitrate + 7) / 8;

	codec_output->vsp_write_noc = codec_output->vsp_read_noc +
		large_bw_calculation_fp;

	codec_output->vsp_write_ddr = codec_output->vsp_write_noc;

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->vsp_read_noc;
	codec_output->ddr_bw_rd += codec_output->vsp_read_ddr;
	codec_output->noc_bw_wr += codec_output->vsp_write_noc;
	codec_output->ddr_bw_wr += codec_output->vsp_write_ddr;

	large_bw_calculation_fp = ((collocated_bytes_per_lcu * lcu_per_frame *
		codec_input.frame_rate + 999) / 1000 + 999) / 1000;

	codec_output->collocated_rd_noc = large_bw_calculation_fp;
	codec_output->collocated_wr_noc = codec_output->collocated_rd_noc;
	codec_output->collocated_rd_ddr = codec_output->collocated_rd_noc;
	codec_output->collocated_wr_ddr = codec_output->collocated_wr_noc;

	codec_output->collocated_rd_wr_total_noc =
		(u32)(codec_output->collocated_rd_noc + codec_output->collocated_wr_noc);
	codec_output->collocated_rd_wr_total_ddr =
		codec_output->collocated_rd_wr_total_noc;

	/* I frame only */
	if (codec_input.hierachical_layer == CODEC_GOP_IONLY) {
		codec_output->collocated_rd_noc = 0;
		codec_output->collocated_wr_noc = 0;
		codec_output->collocated_rd_ddr = 0;
		codec_output->collocated_wr_ddr = 0;
		codec_output->collocated_rd_wr_total_noc = 0;
		codec_output->collocated_rd_wr_total_ddr = 0;
	}

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->collocated_rd_noc;
	codec_output->noc_bw_wr += codec_output->collocated_wr_noc;
	codec_output->ddr_bw_rd += codec_output->collocated_rd_ddr;
	codec_output->ddr_bw_wr += codec_output->collocated_wr_ddr;

	large_bw_calculation_fp = 0;

	large_bw_calculation_fp = ((codec_input.bitdepth == CODEC_BITDEPTH_8) ?
		frame420_y_bw_linear_8bpp :
		frame420_y_bw_no_ubwc_tile_10bpp) * reference_y_read_bw_factor;

	large_bw_calculation_fp = (large_bw_calculation_fp *
		pineapple_en_readfactor[codec_input.hierachical_layer]);

	large_bw_calculation_fp = (large_bw_calculation_fp +
		dpb_compression_factor_y - 1) / dpb_compression_factor_y;

	large_bw_calculation_fp = (large_bw_calculation_fp + 999) / 1000;

	codec_output->dpb_rd_y_noc = large_bw_calculation_fp;

	large_bw_calculation_fp = 0;

	large_bw_calculation_fp = ((codec_input.bitdepth == CODEC_BITDEPTH_8) ?
		frame420_y_bw_linear_8bpp :
		frame420_y_bw_no_ubwc_tile_10bpp) * reference_crcb_read_bw_factor / 2;

	large_bw_calculation_fp = large_bw_calculation_fp *
		pineapple_en_readfactor[codec_input.hierachical_layer];

	large_bw_calculation_fp = (large_bw_calculation_fp +
		dpb_compression_factor_cbcr - 1) / dpb_compression_factor_cbcr;

	large_bw_calculation_fp = (large_bw_calculation_fp + 999) / 1000;
	codec_output->dpb_rd_crcb_noc = large_bw_calculation_fp;

	large_bw_calculation_fp = 0;

	large_bw_calculation_fp = ((codec_input.bitdepth == CODEC_BITDEPTH_8) ?
		frame420_y_bw_linear_8bpp : frame420_y_bw_no_ubwc_tile_10bpp) *
		reconstructed_write_bw_factor_rd *
		pineapple_en_writefactor[codec_input.hierachical_layer] /
		pineapple_en_frame_num_parallel;

	large_bw_calculation_fp = (large_bw_calculation_fp + 999) / 1000;

	large_bw_calculation_fp = large_bw_calculation_fp *
		(dpb_compression_factor_cbcr + dpb_compression_factor_y / 2);

	large_bw_calculation_fp = (large_bw_calculation_fp +
		dpb_compression_factor_y - 1) / dpb_compression_factor_y;

	large_bw_calculation_fp = (large_bw_calculation_fp +
		dpb_compression_factor_cbcr - 1) / dpb_compression_factor_cbcr;

	codec_output->dpb_wr_noc = large_bw_calculation_fp;

	/*
	 * Summary:
	 * by default (for both HFR and HSR cases) :
	 *      -Any resolution and fps >= 120, enable layering.
	 * (120 -> 3, 240 -> 4, 480 -> 5)
	 *      - (once we enable layering) : 50 per cent frames are Non - reference
	 *  frames.recon write is disable by Venus firmware
	 *      - Customer has ability to enable / disable layering.
	 *  Hence, recon write savings would not be there if
	 *  customer explicitly disables layer encoding.
	 */

	/*HFR Cases use alternating rec write if not PWC*/
	if ((codec_input.frame_rate >= 120) && (codec_input.complexity_setting != 0))
		codec_output->dpb_wr_noc = codec_output->dpb_wr_noc / 2;

	/* for power cases with [B1] adaptive non-ref b frame */
	/* power caes IbP non reference b */
	if ((codec_input.hierachical_layer >= 1) &&
		(codec_input.hierachical_layer <= 3) &&
		(codec_input.complexity_setting != 0))
		codec_output->dpb_wr_noc = codec_output->dpb_wr_noc / 2;

	large_bw_calculation_fp = 0;
	large_bw_calculation_fp = codec_output->dpb_wr_noc *
		(reconstructed_write_bw_factor_rd - 100);

	large_bw_calculation_fp = (large_bw_calculation_fp +
		reconstructed_write_bw_factor_rd - 1) / reconstructed_write_bw_factor_rd;

	codec_output->dpb_rdwr_duetooverlap_noc = large_bw_calculation_fp;

	codec_output->dpb_rd_y_ddr = (en_llc_enable_ref_rd_y_overlap) ?
		(codec_output->dpb_rd_y_noc * 100 + reference_y_read_bw_factor - 1) /
		reference_y_read_bw_factor : codec_output->dpb_rd_y_noc;

	codec_output->dpb_rd_crcb_ddr = (en_llc_enable_ref_rd_crcb) ?
		(codec_output->dpb_rd_crcb_noc * 100 + reference_crcb_read_bw_factor - 1) /
		reference_crcb_read_bw_factor : codec_output->dpb_rd_crcb_noc;

	codec_output->dpb_rdwr_duetooverlap_ddr = (en_llc_enable_rec_wr_uncompleted) ?
		0 : codec_output->dpb_rdwr_duetooverlap_noc;

	codec_output->dpb_wr_ddr = (en_llc_enable_rec_wr_uncompleted) ?
		0 : codec_output->dpb_wr_noc;

	/* I frame only */
	if (codec_input.hierachical_layer == CODEC_GOP_IONLY) {
		codec_output->dpb_rd_y_noc = 0;
		codec_output->dpb_rd_crcb_noc = 0;
		codec_output->dpb_rdwr_duetooverlap_noc = 0;
		codec_output->dpb_wr_noc = 0;
		codec_output->dpb_rd_y_ddr = 0;
		codec_output->dpb_rd_crcb_ddr = 0;
		codec_output->dpb_rdwr_duetooverlap_ddr = 0;
		codec_output->dpb_wr_ddr = 0;
	}

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->dpb_rd_y_noc;
	codec_output->noc_bw_rd += codec_output->dpb_rd_crcb_noc;
	codec_output->noc_bw_rd += codec_output->dpb_rdwr_duetooverlap_noc;
	codec_output->noc_bw_wr += codec_output->dpb_wr_noc;
	codec_output->ddr_bw_rd += codec_output->dpb_rd_y_ddr;
	codec_output->ddr_bw_rd += codec_output->dpb_rd_crcb_ddr;
	codec_output->ddr_bw_rd += codec_output->dpb_rdwr_duetooverlap_ddr;
	codec_output->ddr_bw_wr += codec_output->dpb_wr_ddr;

	if (codec_input.bitdepth == CODEC_BITDEPTH_8) {
		if (codec_input.ipb_yuvrgb == 0) { /* yuv */
			large_bw_calculation_fp = ((frame420_y_bw_linear_8bpp) * 3 / 2);
			codec_output->ipb_rd_total_noc = large_bw_calculation_fp;
			if (codec_input.linear_ipb == 0) {
				codec_output->ipb_rd_total_noc =
					(large_bw_calculation_fp * 100 +
					 ipb_compression_factor - 1) /
					ipb_compression_factor;
			}
		} else { /* rgb */
			large_bw_calculation_fp = frame420_y_bw_linear_8bpp;
			codec_output->ipb_rd_total_noc = large_bw_calculation_fp;
			if (codec_input.linear_ipb == 0) {
				if (codec_input.complexity_setting == 0) /* pwc */
					codec_output->ipb_rd_total_noc =
						(large_bw_calculation_fp * 100 +
						 en_original_compression_factor_rgba_pwd_iris33
						 - 1) /
						en_original_compression_factor_rgba_pwd_iris33;
				else
					codec_output->ipb_rd_total_noc =
					(large_bw_calculation_fp * 100 +
					en_original_compression_factor_rgba_avg_iris33 - 1) /
					en_original_compression_factor_rgba_avg_iris33;
			}
		}
	} else {
		if (codec_input.linear_ipb == 1) {
			large_bw_calculation_fp = (frame420_y_bw_linear_10bpp) * 3 / 2;
			codec_output->ipb_rd_total_noc = large_bw_calculation_fp;
		} else {
			large_bw_calculation_fp = (frame420_y_bw_no_ubwc_tile_10bpp *
				300 / 2 + ipb_compression_factor - 1) / ipb_compression_factor;
			codec_output->ipb_rd_total_noc = large_bw_calculation_fp;
		}
	}

	if (en_rotation_90_270) {
		if (codec_input.codec == CODEC_HEVC) {
			if ((codec_input.bitdepth == CODEC_BITDEPTH_8) &&
					(codec_input.ipb_yuvrgb == 0))
				codec_output->ipb_rd_total_noc = codec_output->ipb_rd_total_noc
					* 1;
			else
				codec_output->ipb_rd_total_noc = codec_output->ipb_rd_total_noc
					* 3;
		} else {
			codec_output->ipb_rd_total_noc = codec_output->ipb_rd_total_noc * 2;
		}
	}

	codec_output->ipb_rd_total_ddr = codec_output->ipb_rd_total_noc;

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->ipb_rd_total_noc;
	codec_output->ddr_bw_rd += codec_output->ipb_rd_total_ddr;

	codec_output->bse_tlb_rd_noc =
		((bse_tlb_byte_per_lcu * lcu_per_frame * codec_input.frame_rate + 999)
		/ 1000 + 999) / 1000;

	if (llc_enabled_bse_tlb) /* TODO should be common declaration */
		codec_output->bse_tlb_rd_ddr = 0;
	else
		codec_output->bse_tlb_rd_ddr = codec_output->bse_tlb_rd_noc;

	codec_output->bse_tlb_wr_noc = codec_output->bse_tlb_rd_noc;

	if (llc_enabled_bse_tlb)
		codec_output->bse_tlb_wr_ddr = 0;
	else
		codec_output->bse_tlb_wr_ddr = codec_output->bse_tlb_wr_noc;

	/* accumulation */
	codec_output->noc_bw_rd += codec_output->bse_tlb_rd_noc;
	codec_output->ddr_bw_rd += codec_output->bse_tlb_rd_ddr;
	codec_output->noc_bw_wr += codec_output->bse_tlb_wr_noc;
	codec_output->ddr_bw_wr += codec_output->bse_tlb_wr_ddr;

	codec_output->mmu_rd_ddr = 0;
	codec_output->mmu_rd_noc = 0;
	/* accumulation */
	codec_output->noc_bw_rd += codec_output->mmu_rd_noc;
	codec_output->ddr_bw_rd += codec_output->mmu_rd_ddr;

	return 0;
}

int msm_vidc_calculate_bandwidth(struct api_calculation_input codec_input,
		struct api_calculation_bw_output *codec_output)
{
	int rc = 0;

	if (codec_input.decoder_or_encoder == CODEC_DECODER) {
		rc = calculate_bandwidth_decoder_iris33(codec_input, codec_output);
	} else if (codec_input.decoder_or_encoder == CODEC_ENCODER) {
		rc = calculate_bandwidth_encoder_iris33(codec_input, codec_output);
	} else {
		d_vpr_e("%s: invalid codec\n", codec_input.decoder_or_encoder);
		return -EINVAL;
	}

	return rc;
}
