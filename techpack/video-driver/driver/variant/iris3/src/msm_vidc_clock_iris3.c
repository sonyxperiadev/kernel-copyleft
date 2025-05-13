// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "kalama_technology.h"
#include "msm_vidc_debug.h"

static u32 calculate_number_mbs_kalama(u32 width, u32 height, u32 lcu_size)
{
	u32 mbs_width = (width % lcu_size) ?
		(width / lcu_size + 1) : (width / lcu_size);

	u32 mbs_height = (height % lcu_size) ?
		(height / lcu_size + 1) : (height / lcu_size);

	return mbs_width * mbs_height * (lcu_size / 16) * (lcu_size / 16);
}

static int initialize_encoder_complexity_table(void)
{
	/* Beging Calculate Encoder GOP Complexity Table and HW Floor numbers */
	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_Bb_ENTRY] = 70000;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_P_ENTRY] = 10000;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_Bb_ENTRY] * 150 +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_P_ENTRY] * 100);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] +
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_P_ENTRY] - 1);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] /
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I3B4b1P][CODEC_ENCODER_GOP_P_ENTRY]);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_Bb_ENTRY] = 30000;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_P_ENTRY] = 10000;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_Bb_ENTRY] * 150 +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_P_ENTRY] * 100);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] +
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_P_ENTRY] - 1);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_FACTORY_ENTRY] /
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_I1B2b1P][CODEC_ENCODER_GOP_P_ENTRY]);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_Bb_ENTRY] = 10000;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_P_ENTRY] = 10000;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_Bb_ENTRY] * 150 +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_P_ENTRY] * 100);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_FACTORY_ENTRY] +
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_P_ENTRY] - 1);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_FACTORY_ENTRY] /
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IbP][CODEC_ENCODER_GOP_P_ENTRY]);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_Bb_ENTRY] = 0;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_P_ENTRY] = 1;

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_Bb_ENTRY] * 150 +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_P_ENTRY] * 100);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_FACTORY_ENTRY] +
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_P_ENTRY] - 1);

	codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_FACTORY_ENTRY] =
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_FACTORY_ENTRY] /
		(codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_Bb_ENTRY] +
		codec_encoder_gop_complexity_table_fp
		[CODEC_GOP_IPP][CODEC_ENCODER_GOP_P_ENTRY]);

	return 0;
}

u32 get_bitrate_entry(u32 pixle_count)
{
	u32 bitrate_entry = 0;

	if (pixle_count >= fp_pixel_count_bar1)
		bitrate_entry = 1;
	else if (pixle_count >= fp_pixel_count_bar2)
		bitrate_entry = 2;
	else if (pixle_count >= fp_pixel_count_bar3)
		bitrate_entry = 3;
	else if (pixle_count >= fp_pixel_count_bar4)
		bitrate_entry = 4;
	else if (pixle_count >= fp_pixel_count_bar5)
		bitrate_entry = 5;
	else if (pixle_count >= fp_pixel_count_bar6)
		bitrate_entry = 6;
	else if (pixle_count >= fp_pixel_count_bar7)
		bitrate_entry = 7;
	else if (pixle_count >= fp_pixel_count_bar8)
		bitrate_entry = 8;
	else if (pixle_count >= fp_pixel_count_bar9)
		bitrate_entry = 9;
	else
		bitrate_entry = 9;

	return bitrate_entry;
}

static int calculate_vsp_min_freq(struct api_calculation_input codec_input,
		struct api_calculation_freq_output *codec_output)
{
	/*
	 * VSP calculation
	 * different methodology from Lahaina
	 */
	u32 vsp_hw_min_frequency = 0;
	/* UInt32 decoder_vsp_fw_overhead = 100 + 5; // amplified by 100x */
	u32 fw_sw_vsp_offset = 1000 + 55;      /* amplified by 1000x */

	/*
	 * Ignore fw_sw_vsp_offset, as this is baked into the reference bitrate tables.
	 *  As a consequence remove x1000 multipler as well.
	 */
	u32 codec = codec_input.codec;
	/* UInt32 *bitratetable; */
	u32 pixle_count = codec_input.frame_width *
		codec_input.frame_height * codec_input.frame_rate;

	u8 bitrate_entry = get_bitrate_entry(pixle_count); /* TODO EXTRACT */

	input_bitrate_fp = ((u32)(codec_input.bitrate_mbps * 100 + 99)) / 100;
	vsp_hw_min_frequency = frequency_table_kalama[0][1] * input_bitrate_fp * 1000;

	/* 8KUHD60fps with B frame */
	if ((pixle_count >= fp_pixel_count_bar0) &&
		(codec_input.hierachical_layer != CODEC_GOP_IPP)) {
		/*
		 *  FORMULA: VSPfreq = NOMINAL * (InputBitrate / ReferenceBitrate);
		 *  ReferenceBitrate = 0 for,
		 *     - 1Stage TURBO, all Codecs.
		 *     - 2Stage TURBO, H264 & H265.
		 *
		 *  8KUHD60fps with B frame
		 *     - bitrate_entry = 0
		 *     - Clock=NOMINAL for H264 & 2Stage H265. Because bitrate
		 *       table entry for TURBO is 0.
		 *
		 *  TODO : Reduce these conditions by removing the zero entries from Bitrate table.
		 */
		vsp_hw_min_frequency = frequency_table_kalama[0][1] *
			input_bitrate_fp * 1000;

		if (codec_input.codec == CODEC_AV1)
			vsp_hw_min_frequency = frequency_table_kalama[0][0] *
				input_bitrate_fp * 1000;

		if ((codec_input.codec == CODEC_H264) ||
			(codec_input.codec == CODEC_H264_CAVLC) ||
			((codec_input.codec == CODEC_HEVC) &&
			(codec_input.vsp_vpp_mode == CODEC_VSPVPP_MODE_1S))) {
			vsp_hw_min_frequency =
				DIV_ROUND_UP(frequency_table_kalama[0][1], fw_sw_vsp_offset);
		} else if (((codec_input.codec == CODEC_HEVC) &&
			(codec_input.vsp_vpp_mode == CODEC_VSPVPP_MODE_2S))
			|| (codec_input.codec == CODEC_VP9)
			|| (codec_input.codec == CODEC_AV1)) {
			if (codec_input.vsp_vpp_mode == CODEC_VSPVPP_MODE_2S) {
				vsp_hw_min_frequency = DIV_ROUND_UP(vsp_hw_min_frequency,
					(bitrate_table_kalama_2stage_fp[codec][0] *
					fw_sw_vsp_offset));
			} else {
				vsp_hw_min_frequency = DIV_ROUND_UP(vsp_hw_min_frequency,
					(bitrate_table_kalama_1stage_fp[codec][0] *
					fw_sw_vsp_offset));
			}
		}
	} else {
		vsp_hw_min_frequency = frequency_table_kalama[0][1] *
			input_bitrate_fp * 1000;

		if (codec_input.codec == CODEC_AV1 && bitrate_entry == 1)
			vsp_hw_min_frequency = frequency_table_kalama[0][0] *
				input_bitrate_fp * 1000;

		if ((codec_input.codec == CODEC_H264_CAVLC) &&
			(codec_input.entropy_coding_mode == CODEC_ENTROPY_CODING_CAVLC))
			codec = CODEC_H264_CAVLC;
		else if ((codec_input.codec == CODEC_H264) &&
			(codec_input.entropy_coding_mode == CODEC_ENTROPY_CODING_CABAC))
			codec = CODEC_H264;

		if (codec_input.vsp_vpp_mode == CODEC_VSPVPP_MODE_2S)
			vsp_hw_min_frequency = DIV_ROUND_UP(vsp_hw_min_frequency,
				(bitrate_table_kalama_2stage_fp[codec][bitrate_entry]) *
					fw_sw_vsp_offset);
		else
			vsp_hw_min_frequency = DIV_ROUND_UP(vsp_hw_min_frequency,
				(bitrate_table_kalama_1stage_fp[codec][bitrate_entry]) *
					fw_sw_vsp_offset);
	}

	codec_output->vsp_min_freq = vsp_hw_min_frequency;
	return 0;
}

static u32 calculate_pipe_penalty(struct api_calculation_input codec_input)
{
	u32 pipe_penalty_codec = 0;
	u8 avid_commercial_content = 0;
	u32 pixel_count = 0;

	/* decoder */
	if (codec_input.decoder_or_encoder == CODEC_DECODER) {
		pipe_penalty_codec = pipe_penalty_kalama[0][0];
		avid_commercial_content = codec_input.av1d_commer_tile_enable;
		if (codec_input.codec == CODEC_AV1) {
			pixel_count = codec_input.frame_width * codec_input.frame_height;
			if (pixel_count <= 1920 * 1080)
				pipe_penalty_codec =
					pipe_penalty_kalama[avid_commercial_content + 1][0];
			else if (pixel_count < 3840 * 2160)
				pipe_penalty_codec =
					(pipe_penalty_kalama[avid_commercial_content + 1][0] +
					pipe_penalty_kalama[avid_commercial_content + 1][1]) / 2;
			else if ((pixel_count == 3840 * 2160) ||
				(pixel_count == 4096 * 2160) || (pixel_count == 4096 * 2304))
				pipe_penalty_codec =
					pipe_penalty_kalama[avid_commercial_content + 1][1];
			else if (pixel_count < 7680 * 4320)
				pipe_penalty_codec =
					(pipe_penalty_kalama[avid_commercial_content + 1][1] +
					pipe_penalty_kalama[avid_commercial_content + 1][2]) / 2;
			else
				pipe_penalty_codec =
					pipe_penalty_kalama[avid_commercial_content + 1][2];
		}
	} else {
		pipe_penalty_codec = 101;
	}

	return pipe_penalty_codec;
}

static int calculate_vpp_min_freq(struct api_calculation_input codec_input,
		struct api_calculation_freq_output *codec_output)
{
	u32 vpp_hw_min_frequency = 0;
	u32 fmin = 0;
	u32 tensilica_min_frequency = 0;
	u32 decoder_vsp_fw_overhead = 100 + 5; /* amplified by 100x */
	/* UInt32 fw_sw_vsp_offset = 1000 + 55;       amplified by 1000x */
	/* TODO from calculate_sw_vsp_min_freq */
	u32 vsp_hw_min_frequency = codec_output->vsp_min_freq;
	u32 pipe_penalty_codec = 0;
	u32 fmin_fwoverhead105 = 0;
	u32 fmin_measured_fwoverhead = 0;
	u32 lpmode_uhd_cycle_permb = 0;
	u32 hqmode1080p_cycle_permb = 0;
	u32 encoder_vpp_target_clk_per_mb = 0;

	codec_mbspersession_kalama =
		calculate_number_mbs_kalama(codec_input.frame_width,
		codec_input.frame_height, codec_input.lcu_size) *
		codec_input.frame_rate;

	/* Section 2. 0  VPP/VSP calculation */
	if (codec_input.decoder_or_encoder == CODEC_DECODER) { /* decoder */
		vpp_hw_min_frequency = ((decoder_vpp_target_clk_per_mb_kalama) *
			(codec_mbspersession_kalama) + codec_input.pipe_num - 1) /
			(codec_input.pipe_num);

		vpp_hw_min_frequency = (vpp_hw_min_frequency + 99999) / 1000000;

		if (codec_input.pipe_num > 1) {
			pipe_penalty_codec = calculate_pipe_penalty(codec_input);
			vpp_hw_min_frequency = (vpp_hw_min_frequency *
				pipe_penalty_codec + 999) / 1000;
		}

		if (codec_input.vsp_vpp_mode == CODEC_VSPVPP_MODE_2S) {
			/* FW overhead, convert FW cycles to impact to one pipe */
			u64 decoder_vpp_fw_overhead = 0;

			decoder_vpp_fw_overhead =
				DIV_ROUND_UP((DECODER_VPP_FW_OVERHEAD_KALAMA * 10 *
				codec_input.frame_rate), 15);

			decoder_vpp_fw_overhead =
				DIV_ROUND_UP((decoder_vpp_fw_overhead * 1000),
				(codec_mbspersession_kalama *
				decoder_vpp_target_clk_per_mb_kalama / codec_input.pipe_num));

			decoder_vpp_fw_overhead += 1000;
			decoder_vpp_fw_overhead = (decoder_vpp_fw_overhead < 1050) ?
				1050 : decoder_vpp_fw_overhead;

			/* VPP HW + FW */
			if (codec_input.linear_opb == 1 &&
			    codec_input.bitdepth == CODEC_BITDEPTH_10)
				/* multiply by 1.20 for 10b case */
				decoder_vpp_fw_overhead = 1200 + decoder_vpp_fw_overhead - 1000;

			vpp_hw_min_frequency = (vpp_hw_min_frequency *
				decoder_vpp_fw_overhead + 999) / 1000;

			/* VSP HW+FW */
			vsp_hw_min_frequency =
				(vsp_hw_min_frequency * decoder_vsp_fw_overhead + 99) / 100;

			fmin = (vpp_hw_min_frequency > vsp_hw_min_frequency) ?
				vpp_hw_min_frequency : vsp_hw_min_frequency;
		} else {
			/* 1-stage need SW cycles + FW cycles + HW time */
			if (codec_input.linear_opb == 1 &&
			    codec_input.bitdepth == CODEC_BITDEPTH_10)
				/* multiply by 1.20 for 10b linear case */
				vpp_hw_min_frequency =
					(vpp_hw_min_frequency * 1200 + 999) / 1000;

			/*
			 * HW time
			 * comment: 02/23/2021 SY: the bitrate is measured bitrate,
			 * the overlapping effect is already considered into bitrate.
			 * no need to add extra anymore
			 */
			fmin = (vpp_hw_min_frequency > vsp_hw_min_frequency) ?
				vpp_hw_min_frequency : vsp_hw_min_frequency;

			/* FW time */
			fmin_fwoverhead105 = (fmin * 105 + 99) / 100;
			fmin_measured_fwoverhead = fmin +
				(((DECODER_VPPVSP1STAGE_FW_OVERHEAD_KALAMA *
				codec_input.frame_rate * 10 + 14) / 15 + 999) / 1000 + 999) /
				1000;

			fmin = (fmin_fwoverhead105 > fmin_measured_fwoverhead) ?
				fmin_fwoverhead105 : fmin_measured_fwoverhead;
		}

		tensilica_min_frequency = (DECODER_SW_OVERHEAD_KALAMA * 10 + 14) / 15;
		tensilica_min_frequency = (tensilica_min_frequency + 999) / 1000;
		tensilica_min_frequency = tensilica_min_frequency * codec_input.frame_rate;
		tensilica_min_frequency = (tensilica_min_frequency + 999) / 1000;
		fmin = (tensilica_min_frequency > fmin) ? tensilica_min_frequency : fmin;
	} else { /* encoder */
		/* Decide LP/HQ */
		u8 hq_mode = 0;

		if (codec_input.pipe_num > 1)
			if (codec_input.frame_width * codec_input.frame_height <=
				1920 * 1080)
				if (codec_input.frame_width * codec_input.frame_height *
					codec_input.frame_rate <= 1920 * 1080 * 60)
					hq_mode = 1;

		codec_output->enc_hqmode = hq_mode;

		/* Section 1. 0 */
		/* TODO ONETIME call, should be in another place. */
		initialize_encoder_complexity_table();

		/* End Calculate Encoder GOP Complexity Table */

		/* VPP base cycle */
		lpmode_uhd_cycle_permb = (320 *
			codec_encoder_gop_complexity_table_fp
			[codec_input.hierachical_layer][CODEC_ENCODER_GOP_FACTORY_ENTRY]
			+ 99) / 100;

		if ((codec_input.frame_width == 1920) &&
			((codec_input.frame_height == 1080) ||
			(codec_input.frame_height == 1088)) &&
			(codec_input.frame_rate >= 480))
			lpmode_uhd_cycle_permb = (90 * 4 *
				codec_encoder_gop_complexity_table_fp
				[codec_input.hierachical_layer][CODEC_ENCODER_GOP_FACTORY_ENTRY]
				+ 99) / 100;

		if ((codec_input.frame_width == 1280) &&
			((codec_input.frame_height == 720) ||
			(codec_input.frame_height == 768)) &&
			(codec_input.frame_rate >= 960))
			lpmode_uhd_cycle_permb = (99 * 4 *
				codec_encoder_gop_complexity_table_fp
				[codec_input.hierachical_layer][CODEC_ENCODER_GOP_FACTORY_ENTRY]
				+ 99) / 100;

		hqmode1080p_cycle_permb = (675 *
			codec_encoder_gop_complexity_table_fp
			[codec_input.hierachical_layer][CODEC_ENCODER_GOP_FACTORY_ENTRY]
			+ 99) / 100;

		encoder_vpp_target_clk_per_mb = (hq_mode) ?
			hqmode1080p_cycle_permb : lpmode_uhd_cycle_permb;

		vpp_hw_min_frequency = ((encoder_vpp_target_clk_per_mb) *
			(codec_mbspersession_kalama) + codec_input.pipe_num - 1) /
			(codec_input.pipe_num);

		vpp_hw_min_frequency = (vpp_hw_min_frequency + 99999) / 1000000;

		if (codec_input.pipe_num > 1) {
			u32 pipe_penalty_codec = 101;

			vpp_hw_min_frequency = (vpp_hw_min_frequency *
				pipe_penalty_codec + 99) / 100;
		}

		if (codec_input.vsp_vpp_mode == CODEC_VSPVPP_MODE_2S) {
			/* FW overhead, convert FW cycles to impact to one pipe */
			u64 encoder_vpp_fw_overhead = 0;

			encoder_vpp_fw_overhead =
				DIV_ROUND_UP((ENCODER_VPP_FW_OVERHEAD_KALAMA * 10 *
				codec_input.frame_rate), 15);

			encoder_vpp_fw_overhead =
				DIV_ROUND_UP((encoder_vpp_fw_overhead * 1000),
				(codec_mbspersession_kalama * encoder_vpp_target_clk_per_mb /
				codec_input.pipe_num));

			encoder_vpp_fw_overhead += 1000;

			encoder_vpp_fw_overhead = (encoder_vpp_fw_overhead < 1050) ?
				1050 : encoder_vpp_fw_overhead;

			/* VPP HW + FW */
			vpp_hw_min_frequency = (vpp_hw_min_frequency *
				encoder_vpp_fw_overhead + 999) / 1000;

			/* TODO : decoder_vsp_fw_overhead? */
			vsp_hw_min_frequency = (vsp_hw_min_frequency *
				decoder_vsp_fw_overhead + 99) / 100;

			fmin = (vpp_hw_min_frequency > vsp_hw_min_frequency) ?
				vpp_hw_min_frequency : vsp_hw_min_frequency;
		} else {
			/* HW time */
			fmin = (vpp_hw_min_frequency > vsp_hw_min_frequency) ?
				vpp_hw_min_frequency : vsp_hw_min_frequency;

			/* FW time */
			fmin_fwoverhead105 = (fmin * 105 + 99) / 100;
			fmin_measured_fwoverhead = fmin +
				(((DECODER_VPPVSP1STAGE_FW_OVERHEAD_KALAMA *
				codec_input.frame_rate * 10 + 14) / 15 + 999) /
				1000 + 999) / 1000;

			fmin = (fmin_fwoverhead105 > fmin_measured_fwoverhead) ?
				fmin_fwoverhead105 : fmin_measured_fwoverhead;
			/* SW time */
		}

		tensilica_min_frequency = (ENCODER_SW_OVERHEAD_KALAMA * 10 + 14) / 15;
		tensilica_min_frequency = (tensilica_min_frequency + 999) / 1000;

		tensilica_min_frequency = tensilica_min_frequency *
			codec_input.frame_rate;

		tensilica_min_frequency = (tensilica_min_frequency + 999) / 1000;

		fmin = (tensilica_min_frequency > fmin) ?
			tensilica_min_frequency : fmin;
	}

	codec_output->vpp_min_freq = vpp_hw_min_frequency;
	codec_output->vsp_min_freq = vsp_hw_min_frequency;
	codec_output->tensilica_min_freq = tensilica_min_frequency;
	codec_output->hw_min_freq = fmin;

	return 0;
}

int msm_vidc_calculate_frequency(struct api_calculation_input codec_input,
		struct api_calculation_freq_output *codec_output)
{
	int rc = 0;

	rc = calculate_vsp_min_freq(codec_input, codec_output);
	if (rc)
		return rc;

	rc = calculate_vpp_min_freq(codec_input, codec_output);
	if (rc)
		return rc;

	return rc;
}
