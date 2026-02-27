// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "msm_vidc_power_iris3.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_inst.h"
#include "msm_vidc_core.h"
#include "msm_vidc_debug.h"
#include "perf_static_model.h"
#include "msm_vidc_power.h"

static u64 __calculate_decoder(struct vidc_bus_vote_data *d);
static u64 __calculate_encoder(struct vidc_bus_vote_data *d);
static u64 __calculate(struct msm_vidc_inst *inst, struct vidc_bus_vote_data *d);
static u64 msm_vidc_calc_freq_iris3_legacy(struct msm_vidc_inst *inst, u32 data_size);

static int msm_vidc_init_codec_input_freq(struct msm_vidc_inst *inst, u32 data_size,
		struct api_calculation_input *codec_input)
{
	enum msm_vidc_port_type port;
	u32 color_fmt;

	if (is_encode_session(inst)) {
		codec_input->decoder_or_encoder = CODEC_ENCODER;
	} else if (is_decode_session(inst)) {
		codec_input->decoder_or_encoder = CODEC_DECODER;
	} else {
		d_vpr_e("%s: invalid domain %d\n", __func__, inst->domain);
		return -EINVAL;
	}

	codec_input->chipset_gen = MSM_KALAMA;

	if (inst->codec == MSM_VIDC_H264) {
		codec_input->codec    = CODEC_H264;
		codec_input->lcu_size = 16;
		if (inst->capabilities[ENTROPY_MODE].value ==
				V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC)
			codec_input->entropy_coding_mode = CODEC_ENTROPY_CODING_CABAC;
		else
			codec_input->entropy_coding_mode = CODEC_ENTROPY_CODING_CAVLC;
	} else if (inst->codec == MSM_VIDC_HEVC) {
		codec_input->codec    = CODEC_HEVC;
		codec_input->lcu_size = 32;
	} else if (inst->codec == MSM_VIDC_VP9) {
		codec_input->codec    = CODEC_VP9;
		codec_input->lcu_size = 16;
	} else if (inst->codec == MSM_VIDC_AV1) {
		codec_input->codec    = CODEC_AV1;
		codec_input->lcu_size = 32;
	} else {
		d_vpr_e("%s: invalid codec %d\n", __func__, inst->codec);
		return -EINVAL;
	}

	codec_input->pipe_num = inst->capabilities[PIPE].value;
	codec_input->frame_rate = inst->max_rate;

	port = is_decode_session(inst) ? INPUT_PORT : OUTPUT_PORT;
	codec_input->frame_width = inst->fmts[port].fmt.pix_mp.width;
	codec_input->frame_height = inst->fmts[port].fmt.pix_mp.height;

	if (inst->capabilities[STAGE].value == MSM_VIDC_STAGE_1) {
		codec_input->vsp_vpp_mode = CODEC_VSPVPP_MODE_1S;
	} else if (inst->capabilities[STAGE].value == MSM_VIDC_STAGE_2) {
		codec_input->vsp_vpp_mode = CODEC_VSPVPP_MODE_2S;
	} else {
		d_vpr_e("%s: invalid stage %d\n", __func__,
				inst->capabilities[STAGE].value);
		return -EINVAL;
	}

	if (inst->capabilities[BIT_DEPTH].value == BIT_DEPTH_8)
		codec_input->bitdepth = CODEC_BITDEPTH_8;
	else
		codec_input->bitdepth = CODEC_BITDEPTH_10;

	/*
	 * Used for calculating Encoder GOP Complexity
	 * hierachical_layer= 0..7 used as Array Index
	 * inst->capabilities[B_FRAME].value=[ 0 1 2]
	 * TODO how to map?
	 */

	/* set as IPP */
	codec_input->hierachical_layer = 0;

	if (is_decode_session(inst))
		color_fmt = v4l2_colorformat_to_driver(inst,
			inst->fmts[OUTPUT_PORT].fmt.pix_mp.pixelformat, __func__);
	else
		color_fmt = v4l2_colorformat_to_driver(inst,
			inst->fmts[INPUT_PORT].fmt.pix_mp.pixelformat, __func__);

	codec_input->linear_opb = is_linear_colorformat(color_fmt);
	codec_input->bitrate_mbps =
		(codec_input->frame_rate * data_size * 8) / 1000000;

	/* disable av1d commercial tile */
	codec_input->av1d_commer_tile_enable = 0;
	/* set as sanity mode */
	codec_input->regression_mode = 1;

	return 0;
}

static int msm_vidc_init_codec_input_bus(struct msm_vidc_inst *inst, struct vidc_bus_vote_data *d,
		struct api_calculation_input *codec_input)
{
	u32 complexity_factor_int = 0, complexity_factor_frac = 0;
	bool opb_compression_enabled = false;

	if (!d)
		return -EINVAL;

	if (d->domain == MSM_VIDC_ENCODER) {
		codec_input->decoder_or_encoder = CODEC_ENCODER;
	} else if (d->domain == MSM_VIDC_DECODER) {
		codec_input->decoder_or_encoder = CODEC_DECODER;
	} else {
		d_vpr_e("%s: invalid domain %d\n", __func__, d->domain);
		return -EINVAL;
	}

	codec_input->chipset_gen = MSM_KALAMA;

	if (d->codec == MSM_VIDC_H264) {
		codec_input->codec = CODEC_H264;
	} else if (d->codec == MSM_VIDC_HEVC) {
		codec_input->codec = CODEC_HEVC;
	} else if (d->codec == MSM_VIDC_VP9) {
		codec_input->codec = CODEC_VP9;
	} else if (d->codec == MSM_VIDC_AV1) {
		codec_input->codec = CODEC_AV1;
	} else {
		d_vpr_e("%s: invalid codec %d\n", __func__, d->codec);
		return -EINVAL;
	}

	codec_input->lcu_size = d->lcu_size;
	codec_input->pipe_num = d->num_vpp_pipes;
	codec_input->frame_rate = d->fps;
	codec_input->frame_width = d->input_width;
	codec_input->frame_height = d->input_height;

	if (d->work_mode == MSM_VIDC_STAGE_1) {
		codec_input->vsp_vpp_mode = CODEC_VSPVPP_MODE_1S;
	} else if (d->work_mode == MSM_VIDC_STAGE_2) {
		codec_input->vsp_vpp_mode = CODEC_VSPVPP_MODE_2S;
	} else {
		d_vpr_e("%s: invalid stage %d\n", __func__, d->work_mode);
		return -EINVAL;
	}

	if (inst->capabilities[ENTROPY_MODE].value ==
			V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC) {
		codec_input->entropy_coding_mode = CODEC_ENTROPY_CODING_CABAC;
	} else if (inst->capabilities[ENTROPY_MODE].value ==
			V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC) {
		codec_input->entropy_coding_mode = CODEC_ENTROPY_CODING_CAVLC;
	} else {
		d_vpr_e("%s: invalid entropy %d\n", __func__,
				inst->capabilities[ENTROPY_MODE].value);
		return -EINVAL;
	}

	/*
	 * Used for calculating Encoder GOP Complexity
	 * hierachical_layer= 0..7 used as Array Index
	 * TODO how to map?
	 */
	codec_input->hierachical_layer = 0; /* set as IPP */

	/*
	 * If the calculated motion_vector_complexity is > 2 then set the
	 * complexity_setting and refframe_complexity to be pwc(performance worst case)
	 * values. If the motion_vector_complexity is < 2 then set the complexity_setting
	 * and refframe_complexity to be average case values.
	 */

	complexity_factor_int = Q16_INT(d->complexity_factor);
	complexity_factor_frac = Q16_FRAC(d->complexity_factor);

	if (complexity_factor_int < COMPLEXITY_THRESHOLD ||
		(complexity_factor_int == COMPLEXITY_THRESHOLD &&
		complexity_factor_frac == 0)) {
		/* set as average case values */
		codec_input->complexity_setting = COMPLEXITY_SETTING_AVG;
		codec_input->refframe_complexity = REFFRAME_COMPLEXITY_AVG;
	} else {
		/* set as pwc */
		codec_input->complexity_setting = COMPLEXITY_SETTING_PWC;
		codec_input->refframe_complexity = REFFRAME_COMPLEXITY_PWC;
	}

	codec_input->status_llc_onoff = d->use_sys_cache;

	if (__bpp(d->color_formats[0]) == 8)
		codec_input->bitdepth = CODEC_BITDEPTH_8;
	else
		codec_input->bitdepth = CODEC_BITDEPTH_10;

	if (d->num_formats == 1) {
		codec_input->split_opb = 0;
		codec_input->linear_opb = !__ubwc(d->color_formats[0]);
	} else if (d->num_formats == 2) {
		codec_input->split_opb = 1;
		codec_input->linear_opb = !__ubwc(d->color_formats[1]);
	} else {
		d_vpr_e("%s: invalid num_formats %d\n",
			__func__, d->num_formats);
		return -EINVAL;
	}

	codec_input->linear_ipb = 0;   /* set as ubwc ipb */

	/* TODO Confirm if we always LOSSLESS mode ie lossy_ipb = 0*/
	codec_input->lossy_ipb = 0;   /* set as lossless ipb */

	/* TODO Confirm if no multiref */
	codec_input->encoder_multiref = 0;	/* set as no multiref */
	codec_input->bitrate_mbps = (d->bitrate / 1000000); /* bps 10;	set as 10mbps */

	opb_compression_enabled = d->num_formats >= 2 && __ubwc(d->color_formats[1]);

	/* ANDROID CR is in Q16 format, StaticModel CR in x100 format */
	codec_input->cr_dpb = ((Q16_INT(d->compression_ratio)*100) +
		Q16_FRAC(d->compression_ratio));

	codec_input->cr_opb = opb_compression_enabled ?
		codec_input->cr_dpb : FP_ONE;

	codec_input->cr_ipb = ((Q16_INT(d->input_cr)*100) + Q16_FRAC(d->input_cr));
	codec_input->cr_rpb = codec_input->cr_dpb;  /* cr_rpb ony for encoder */

	/* disable by default, only enable for aurora depth map session */
	codec_input->lumaonly_decode = 0;
	/* TODO: disable av1d commercial tile */
	codec_input->av1d_commer_tile_enable = 0;
	/* set as custom regression mode, as are using cr,cf values from FW */
	codec_input->regression_mode = REGRESSION_MODE_CUSTOM;


	/* Dump all the variables for easier debugging */
	if (msm_vidc_debug & VIDC_BUS) {
		struct dump dump[] = {
		{"complexity_factor_int", "%d", complexity_factor_int},
		{"complexity_factor_frac", "%d", complexity_factor_frac},
		{"refframe_complexity", "%d", codec_input->refframe_complexity},
		{"complexity_setting", "%d", codec_input->complexity_setting},
		{"cr_dpb", "%d", codec_input->cr_dpb},
		{"cr_opb", "%d", codec_input->cr_opb},
		{"cr_ipb", "%d", codec_input->cr_ipb},
		{"cr_rpb", "%d", codec_input->cr_rpb},
		{"lcu size", "%d", codec_input->lcu_size},
		{"pipe number", "%d", codec_input->pipe_num},
		{"frame_rate", "%d", codec_input->frame_rate},
		{"frame_width", "%d", codec_input->frame_width},
		{"frame_height", "%d", codec_input->frame_height},
		{"work_mode", "%d", d->work_mode},
		{"encoder_or_decode", "%d", inst->domain},
		{"chipset_gen", "%d", codec_input->chipset_gen},
		{"codec_input", "%d", codec_input->codec},
		{"entropy_coding_mode", "%d", codec_input->entropy_coding_mode},
		{"hierachical_layer", "%d", codec_input->hierachical_layer},
		{"status_llc_onoff", "%d", codec_input->status_llc_onoff},
		{"bit_depth", "%d", codec_input->bitdepth},
		{"split_opb", "%d", codec_input->split_opb},
		{"linear_opb", "%d", codec_input->linear_opb},
		{"linear_ipb", "%d", codec_input->linear_ipb},
		{"lossy_ipb", "%d", codec_input->lossy_ipb},
		{"encoder_multiref", "%d", codec_input->encoder_multiref},
		{"bitrate_mbps", "%d", codec_input->bitrate_mbps},
		{"lumaonly_decode", "%d", codec_input->lumaonly_decode},
		{"av1d_commer_tile_enable", "%d", codec_input->av1d_commer_tile_enable},
		{"regression_mode", "%d", codec_input->regression_mode},
		};
		__dump(dump, ARRAY_SIZE(dump));
	}

	return 0;
}

static u64 msm_vidc_calc_freq_iris3_new(struct msm_vidc_inst *inst, u32 data_size)
{
	u64 freq = 0;
	struct msm_vidc_core *core;
	int ret = 0;
	struct api_calculation_input codec_input;
	struct api_calculation_freq_output codec_output;
	u32 fps, mbpf;

	core = inst->core;

	mbpf = msm_vidc_get_mbs_per_frame(inst);
	fps = inst->max_rate;

	memset(&codec_input, 0, sizeof(struct api_calculation_input));
	memset(&codec_output, 0, sizeof(struct api_calculation_freq_output));
	ret = msm_vidc_init_codec_input_freq(inst, data_size, &codec_input);
	if (ret)
		return freq;
	ret = msm_vidc_calculate_frequency(codec_input, &codec_output);
	if (ret)
		return freq;
	freq = codec_output.hw_min_freq * 1000000; /* Convert to Hz */

	i_vpr_p(inst, "%s: filled len %d, required freq %llu, fps %u, mbpf %u\n",
		__func__, data_size, freq, fps, mbpf);

	if (inst->codec == MSM_VIDC_AV1 ||
		(inst->iframe && is_hevc_10bit_decode_session(inst))) {
		/*
		 * for AV1 or HEVC 10bit and iframe case only allow TURBO and
		 * limit to NOM for all other cases
		 */
	} else {
		/* limit to NOM, index 0 is TURBO, index 1 is NOM clock rate */
		if (core->resource->freq_set.count >= 2 &&
			freq > core->resource->freq_set.freq_tbl[1].freq)
			freq = core->resource->freq_set.freq_tbl[1].freq;
	}

	return freq;
}

static int msm_vidc_calc_bw_iris3_new(struct msm_vidc_inst *inst,
		struct vidc_bus_vote_data *vidc_data)
{
	u32 ret = 0;
	struct api_calculation_input codec_input;
	struct api_calculation_bw_output codec_output;

	memset(&codec_input, 0, sizeof(struct api_calculation_input));
	memset(&codec_output, 0, sizeof(struct api_calculation_bw_output));

	ret = msm_vidc_init_codec_input_bus(inst, vidc_data, &codec_input);
	if (ret)
		return ret;
	ret = msm_vidc_calculate_bandwidth(codec_input, &codec_output);
	if (ret)
		return ret;

	vidc_data->calc_bw_ddr = kbps(codec_output.ddr_bw_rd + codec_output.ddr_bw_wr);
	vidc_data->calc_bw_llcc = kbps(codec_output.noc_bw_rd + codec_output.noc_bw_wr);

	i_vpr_l(inst, "%s: calc_bw_ddr %lu calc_bw_llcc %lu",
		__func__, vidc_data->calc_bw_ddr, vidc_data->calc_bw_llcc);

	return ret;
}

u64 msm_vidc_calc_freq_iris3(struct msm_vidc_inst *inst, u32 data_size)
{
	u64 freq = 0;

	if (ENABLE_LEGACY_POWER_CALCULATIONS)
		freq = msm_vidc_calc_freq_iris3_legacy(inst, data_size);
	else
		freq = msm_vidc_calc_freq_iris3_new(inst, data_size);

	return freq;
}

static u64 msm_vidc_calc_freq_iris3_legacy(struct msm_vidc_inst *inst, u32 data_size)
{
	u64 freq = 0;
	struct msm_vidc_core *core;
	u64 vsp_cycles = 0, vpp_cycles = 0, fw_cycles = 0;
	u64 fw_vpp_cycles = 0, bitrate = 0;
	u32 vpp_cycles_per_mb;
	u32 mbs_per_second;
	u32 operating_rate, vsp_factor_num = 1, vsp_factor_den = 1;
	u32 base_cycles = 0;
	u32 fps, mbpf;

	core = inst->core;

	if (!core->resource || !core->resource->freq_set.freq_tbl ||
		!core->resource->freq_set.count) {
		d_vpr_e("%s: invalid params\n", __func__);
		return freq;
	}

	mbpf = msm_vidc_get_mbs_per_frame(inst);
	fps = inst->max_rate;
	mbs_per_second = mbpf * fps;

	/*
	 * Calculate vpp, vsp, fw cycles separately for encoder and decoder.
	 * Even though, most part is common now, in future it may change
	 * between them.
	 */
	fw_cycles = fps * inst->capabilities[MB_CYCLES_FW].value;
	fw_vpp_cycles = fps * inst->capabilities[MB_CYCLES_FW_VPP].value;

	if (is_encode_session(inst)) {
		vpp_cycles_per_mb = is_low_power_session(inst) ?
			inst->capabilities[MB_CYCLES_LP].value :
			inst->capabilities[MB_CYCLES_VPP].value;

		vpp_cycles = mbs_per_second * vpp_cycles_per_mb /
			inst->capabilities[PIPE].value;

		/* Factor 1.25 for IbP and 1.375 for I1B2b1P GOP structure */
		if (inst->capabilities[B_FRAME].value > 1)
			vpp_cycles += (vpp_cycles / 4) + (vpp_cycles / 8);
		else if (inst->capabilities[B_FRAME].value)
			vpp_cycles += vpp_cycles / 4;
		/* 21 / 20 is minimum overhead factor */
		vpp_cycles += max(div_u64(vpp_cycles, 20), fw_vpp_cycles);
		/* 1.01 is multi-pipe overhead */
		if (inst->capabilities[PIPE].value > 1)
			vpp_cycles += div_u64(vpp_cycles, 100);
		/*
		 * 1080p@480fps usecase needs exactly 338MHz
		 * without any margin left. Hence, adding 2 percent
		 * extra to bump it to next level (366MHz).
		 */
		if (fps == 480)
			vpp_cycles += div_u64(vpp_cycles * 2, 100);

		/*
		 * Add 5 percent extra for 720p@960fps use case
		 * to bump it to next level (366MHz).
		 */
		if (fps == 960)
			vpp_cycles += div_u64(vpp_cycles * 5, 100);

		/* increase vpp_cycles by 50% for preprocessing */
		if (inst->capabilities[REQUEST_PREPROCESS].value)
			vpp_cycles = vpp_cycles + vpp_cycles / 2;

		/* VSP */
		/* bitrate is based on fps, scale it using operating rate */
		operating_rate = inst->capabilities[OPERATING_RATE].value >> 16;
		if (operating_rate >
			(inst->capabilities[FRAME_RATE].value >> 16) &&
			(inst->capabilities[FRAME_RATE].value >> 16)) {
			vsp_factor_num = operating_rate;
			vsp_factor_den = inst->capabilities[FRAME_RATE].value >> 16;
		}
		vsp_cycles = div_u64(((u64)inst->capabilities[BIT_RATE].value *
					vsp_factor_num), vsp_factor_den);

		base_cycles = inst->capabilities[MB_CYCLES_VSP].value;
		if (inst->codec == MSM_VIDC_VP9) {
			vsp_cycles = div_u64(vsp_cycles * 170, 100);
		} else if (inst->capabilities[ENTROPY_MODE].value ==
			V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC) {
			vsp_cycles = div_u64(vsp_cycles * 135, 100);
		} else {
			base_cycles = 0;
			vsp_cycles = div_u64(vsp_cycles, 2);
		}
		/* VSP FW Overhead 1.05 */
		vsp_cycles = div_u64(vsp_cycles * 21, 20);

		if (inst->capabilities[STAGE].value == MSM_VIDC_STAGE_1)
			vsp_cycles = vsp_cycles * 3;

		vsp_cycles += mbs_per_second * base_cycles;

	} else if (is_decode_session(inst)) {
		/* VPP */
		vpp_cycles = mbs_per_second * inst->capabilities[MB_CYCLES_VPP].value /
			inst->capabilities[PIPE].value;
		/* 21 / 20 is minimum overhead factor */
		vpp_cycles += max(vpp_cycles / 20, fw_vpp_cycles);
		if (inst->capabilities[PIPE].value > 1) {
			if (inst->codec == MSM_VIDC_AV1) {
				/*
				 * Additional vpp_cycles are required for bitstreams with
				 * 128x128 superblock and non-recommended tile settings.
				 * recommended tiles: 1080P_V2XH1, UHD_V2X2, 8KUHD_V8X2
				 * non-recommended tiles: 1080P_V4XH2_V4X1, UHD_V8X4_V8X1,
				 * 8KUHD_V8X8_V8X1
				 */
				if (inst->capabilities[SUPER_BLOCK].value)
					vpp_cycles += div_u64(vpp_cycles * 1464, 1000);
				else
					vpp_cycles += div_u64(vpp_cycles * 410, 1000);
			} else {
				/* 1.059 is multi-pipe overhead */
				vpp_cycles += div_u64(vpp_cycles * 59, 1000);
			}
		}

		/* VSP */
		if (inst->codec == MSM_VIDC_AV1) {
			/*
			 * For AV1: Use VSP calculations from Kalama perf model.
			 * For legacy codecs, use vsp_cycles based on legacy MB_CYCLES_VSP.
			 */
			u32 decoder_vsp_fw_overhead = 105;
			u32 fw_sw_vsp_offset = 1055;
			u64 vsp_hw_min_frequency = 0;
			u32 input_bitrate_mbps = 0;
			u32 bitrate_2stage[2] = {130, 120};
			u32 bitrate_1stage = 100;
			u32 width, height;
			u32 bitrate_entry, freq_entry, freq_tbl_value;
			struct frequency_table *freq_tbl;
			struct v4l2_format *out_f = &inst->fmts[OUTPUT_PORT];

			width = out_f->fmt.pix_mp.width;
			height = out_f->fmt.pix_mp.height;

			bitrate_entry = 1;
			/* 8KUHD60, UHD240, 1080p960 */
			if (width * height * fps >= 3840 * 2160 * 240)
				bitrate_entry = 0;

			freq_entry = bitrate_entry;

			freq_tbl = core->resource->freq_set.freq_tbl;
			freq_tbl_value = freq_tbl[freq_entry].freq / 1000000;

			input_bitrate_mbps = fps * data_size * 8 / (1024 * 1024);
			vsp_hw_min_frequency = freq_tbl_value * 1000 * input_bitrate_mbps;

			if (inst->capabilities[STAGE].value == MSM_VIDC_STAGE_2) {
				vsp_hw_min_frequency +=
					(bitrate_2stage[bitrate_entry] * fw_sw_vsp_offset - 1);
				vsp_hw_min_frequency = div_u64(vsp_hw_min_frequency,
					(bitrate_2stage[bitrate_entry] * fw_sw_vsp_offset));
				/* VSP fw overhead 1.05 */
				vsp_hw_min_frequency = div_u64(vsp_hw_min_frequency *
					decoder_vsp_fw_overhead + 99, 100);
			} else {
				vsp_hw_min_frequency += (bitrate_1stage * fw_sw_vsp_offset - 1);
				vsp_hw_min_frequency = div_u64(vsp_hw_min_frequency,
					(bitrate_1stage * fw_sw_vsp_offset));
			}

			vsp_cycles = vsp_hw_min_frequency * 1000000;
		} else {
			base_cycles = inst->has_bframe ?
					80 : inst->capabilities[MB_CYCLES_VSP].value;
			bitrate = fps * data_size * 8;
			vsp_cycles = bitrate;

			if (inst->codec == MSM_VIDC_VP9) {
				vsp_cycles = div_u64(vsp_cycles * 170, 100);
			} else if (inst->capabilities[ENTROPY_MODE].value ==
				V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC) {
				vsp_cycles = div_u64(vsp_cycles * 135, 100);
			} else {
				base_cycles = 0;
				vsp_cycles = div_u64(vsp_cycles, 2);
			}
			/* VSP FW overhead 1.05 */
			vsp_cycles = div_u64(vsp_cycles * 21, 20);

			if (inst->capabilities[STAGE].value == MSM_VIDC_STAGE_1)
				vsp_cycles = vsp_cycles * 3;

			vsp_cycles += mbs_per_second * base_cycles;

			/* Add 25 percent extra for 960fps use case */
			if (fps >= 960)
				vsp_cycles += div_u64(vpp_cycles * 25, 100);

			/* Add 25 percent extra for HEVC 10bit all intra use case */
			if (inst->iframe && is_hevc_10bit_decode_session(inst))
				vsp_cycles += div_u64(vsp_cycles * 25, 100);

			if (inst->codec == MSM_VIDC_VP9 &&
					inst->capabilities[STAGE].value ==
						MSM_VIDC_STAGE_2 &&
					inst->capabilities[PIPE].value == 4 &&
					bitrate > 90000000)
				vsp_cycles = msm_vidc_max_freq(inst);
		}
	} else {
		i_vpr_e(inst, "%s: Unknown session type\n", __func__);
		return msm_vidc_max_freq(inst);
	}

	freq = max(vpp_cycles, vsp_cycles);
	freq = max(freq, fw_cycles);

	if (inst->codec == MSM_VIDC_AV1 ||
		(inst->iframe && is_hevc_10bit_decode_session(inst))) {
		/*
		 * for AV1 or HEVC 10bit and iframe case only allow TURBO and
		 * limit to NOM for all other cases
		 */
	} else {
		/* limit to NOM, index 0 is TURBO, index 1 is NOM clock rate */
		if (core->resource->freq_set.count >= 2 &&
		    freq > core->resource->freq_set.freq_tbl[1].freq)
			freq = core->resource->freq_set.freq_tbl[1].freq;
	}

	i_vpr_p(inst, "%s: filled len %d, required freq %llu, fps %u, mbpf %u\n",
		__func__, data_size, freq, fps, mbpf);

	return freq;
}

static u64 __calculate_decoder(struct vidc_bus_vote_data *d)
{
	/*
	 * XXX: Don't fool around with any of the hardcoded numbers unless you
	 * know /exactly/ what you're doing.  Many of these numbers are
	 * measured heuristics and hardcoded numbers taken from the firmware.
	 */
	/* Decoder parameters */
	int width, height, lcu_size, fps, dpb_bpp;
	bool unified_dpb_opb, dpb_compression_enabled = true,
		opb_compression_enabled = false,
		llc_ref_read_l2_cache_enabled = false,
		llc_top_line_buf_enabled = false;
	fp_t dpb_read_compression_factor, dpb_opb_scaling_ratio,
		dpb_write_compression_factor, opb_write_compression_factor,
		qsmmu_bw_overhead_factor;
	bool is_h264_category = (d->codec == MSM_VIDC_H264) ? true : false;

	/* Derived parameters */
	int lcu_per_frame, collocated_bytes_per_lcu, tnbr_per_lcu;
	unsigned long bitrate;

	fp_t bins_to_bit_factor, vsp_read_factor, vsp_write_factor,
		dpb_factor, dpb_write_factor, y_bw_no_ubwc_8bpp;
	fp_t y_bw_no_ubwc_10bpp = 0, y_bw_10bpp_p010 = 0,
	     motion_vector_complexity = 0;
	fp_t	dpb_total = 0;

	/* Output parameters */
	struct {
		fp_t vsp_read, vsp_write, collocated_read, collocated_write,
			dpb_read, dpb_write, opb_read, opb_write,
			line_buffer_read, line_buffer_write,
			total;
	} ddr = {0};

	struct {
		fp_t dpb_read, line_buffer_read, line_buffer_write, total;
	} llc = {0};

	unsigned long ret = 0;
	unsigned int integer_part, frac_part;

	width = max(d->input_width, BASELINE_DIMENSIONS.width);
	height = max(d->input_height, BASELINE_DIMENSIONS.height);

	fps = d->fps;

	lcu_size = d->lcu_size;

	dpb_bpp = __bpp(d->color_formats[0]);

	unified_dpb_opb = d->num_formats == 1;

	dpb_opb_scaling_ratio = fp_div(FP_INT(d->input_width * d->input_height),
		FP_INT(d->output_width * d->output_height));

	opb_compression_enabled = d->num_formats >= 2 &&
		__ubwc(d->color_formats[1]);

	integer_part = Q16_INT(d->compression_ratio);
	frac_part = Q16_FRAC(d->compression_ratio);
	dpb_read_compression_factor = FP(integer_part, frac_part, 100);

	integer_part = Q16_INT(d->complexity_factor);
	frac_part = Q16_FRAC(d->complexity_factor);
	motion_vector_complexity = FP(integer_part, frac_part, 100);

	dpb_write_compression_factor = dpb_read_compression_factor;
	opb_write_compression_factor = opb_compression_enabled ?
		dpb_write_compression_factor : FP_ONE;

	if (d->use_sys_cache) {
		llc_ref_read_l2_cache_enabled = true;
		if (is_h264_category)
			llc_top_line_buf_enabled = true;
	}

	/* Derived parameters setup */
	lcu_per_frame = DIV_ROUND_UP(width, lcu_size) *
		DIV_ROUND_UP(height, lcu_size);

	bitrate = DIV_ROUND_UP(d->bitrate, 1000000);

	bins_to_bit_factor = FP_INT(4);

	vsp_write_factor = bins_to_bit_factor;
	vsp_read_factor = bins_to_bit_factor + FP_INT(2);

	collocated_bytes_per_lcu = lcu_size == 16 ? 16 :
				lcu_size == 32 ? 64 : 256;

	if (d->codec == MSM_VIDC_AV1) {
		collocated_bytes_per_lcu = 4 * 512; /* lcu_size = 128 */
		if (lcu_size == 32)
			collocated_bytes_per_lcu = 4 * 512 / (128 * 128 / 32 / 32);
		else if (lcu_size == 64)
			collocated_bytes_per_lcu = 4 * 512 / (128 * 128 / 64 / 64);
	}

	dpb_factor = FP(1, 50, 100);
	dpb_write_factor = FP(1, 5, 100);

	tnbr_per_lcu = lcu_size == 16 ? 128 :
		lcu_size == 32 ? 64 : 128;

	/* .... For DDR & LLC  ...... */
	ddr.vsp_read = fp_div(fp_mult(FP_INT(bitrate),
				vsp_read_factor), FP_INT(8));
	ddr.vsp_write = fp_div(fp_mult(FP_INT(bitrate),
				vsp_write_factor), FP_INT(8));

	ddr.collocated_read = fp_div(FP_INT(lcu_per_frame *
			collocated_bytes_per_lcu * fps), FP_INT(bps(1)));
	ddr.collocated_write = ddr.collocated_read;

	y_bw_no_ubwc_8bpp = fp_div(FP_INT(width * height * fps),
		FP_INT(1000 * 1000));

	if (dpb_bpp != 8) {
		y_bw_no_ubwc_10bpp =
			fp_div(fp_mult(y_bw_no_ubwc_8bpp, FP_INT(256)),
				FP_INT(192));
		y_bw_10bpp_p010 = y_bw_no_ubwc_8bpp * 2;
	}

	ddr.dpb_read = dpb_bpp == 8 ? y_bw_no_ubwc_8bpp : y_bw_no_ubwc_10bpp;
	ddr.dpb_read = fp_div(fp_mult(ddr.dpb_read,
			fp_mult(dpb_factor, motion_vector_complexity)),
			dpb_read_compression_factor);

	ddr.dpb_write = dpb_bpp == 8 ? y_bw_no_ubwc_8bpp : y_bw_no_ubwc_10bpp;
	ddr.dpb_write = fp_div(fp_mult(ddr.dpb_write,
			fp_mult(dpb_factor, dpb_write_factor)),
			dpb_write_compression_factor);

	dpb_total = ddr.dpb_read + ddr.dpb_write;

	if (llc_ref_read_l2_cache_enabled) {
		ddr.dpb_read = fp_div(ddr.dpb_read, is_h264_category ?
					FP(1, 30, 100) : FP(1, 14, 100));
		llc.dpb_read = dpb_total - ddr.dpb_write - ddr.dpb_read;
	}

	ddr.opb_read = FP_ZERO;
	ddr.opb_write = unified_dpb_opb ? FP_ZERO : (dpb_bpp == 8 ?
		y_bw_no_ubwc_8bpp : (opb_compression_enabled ?
		y_bw_no_ubwc_10bpp : y_bw_10bpp_p010));
	ddr.opb_write = fp_div(fp_mult(dpb_factor, ddr.opb_write),
		fp_mult(dpb_opb_scaling_ratio, opb_write_compression_factor));

	ddr.line_buffer_read =
		fp_div(FP_INT(tnbr_per_lcu * lcu_per_frame * fps),
			FP_INT(bps(1)));

	if (is_h264_category)
		ddr.line_buffer_write = fp_div(ddr.line_buffer_read, FP_INT(2));
	else
		ddr.line_buffer_write = ddr.line_buffer_read;
	if (llc_top_line_buf_enabled) {
		llc.line_buffer_read = ddr.line_buffer_read;
		llc.line_buffer_write = ddr.line_buffer_write;
		ddr.line_buffer_write = ddr.line_buffer_read = FP_ZERO;
	}

	ddr.total = ddr.vsp_read + ddr.vsp_write +
		ddr.collocated_read + ddr.collocated_write +
		ddr.dpb_read + ddr.dpb_write +
		ddr.opb_read + ddr.opb_write +
		ddr.line_buffer_read + ddr.line_buffer_write;

	qsmmu_bw_overhead_factor = FP(1, 3, 100);

	ddr.total = fp_mult(ddr.total, qsmmu_bw_overhead_factor);
	llc.total = llc.dpb_read + llc.line_buffer_read +
			llc.line_buffer_write + ddr.total;

	/* Add 25 percent extra for 960fps use case */
	if (fps >= 960) {
		ddr.total += div_u64(ddr.total * 25, 100);
		llc.total += div_u64(llc.total * 25, 100);
	}

	/* Dump all the variables for easier debugging */
	if (msm_vidc_debug & VIDC_BUS) {
		struct dump dump[] = {
		{"DECODER PARAMETERS", "", DUMP_HEADER_MAGIC},
		{"lcu size", "%d", lcu_size},
		{"dpb bitdepth", "%d", dpb_bpp},
		{"frame rate", "%d", fps},
		{"dpb/opb unified", "%d", unified_dpb_opb},
		{"dpb/opb downscaling ratio", DUMP_FP_FMT,
			dpb_opb_scaling_ratio},
		{"dpb compression", "%d", dpb_compression_enabled},
		{"opb compression", "%d", opb_compression_enabled},
		{"dpb read compression factor", DUMP_FP_FMT,
			dpb_read_compression_factor},
		{"dpb write compression factor", DUMP_FP_FMT,
			dpb_write_compression_factor},
		{"frame width", "%d", width},
		{"frame height", "%d", height},
		{"llc ref read l2 cache enabled", "%d",
			llc_ref_read_l2_cache_enabled},
		{"llc top line buf enabled", "%d",
			llc_top_line_buf_enabled},

		{"DERIVED PARAMETERS (1)", "", DUMP_HEADER_MAGIC},
		{"lcus/frame", "%d", lcu_per_frame},
		{"bitrate (Mbit/sec)", "%d", bitrate},
		{"bins to bit factor", DUMP_FP_FMT, bins_to_bit_factor},
		{"dpb write factor", DUMP_FP_FMT, dpb_write_factor},
		{"vsp read factor", DUMP_FP_FMT, vsp_read_factor},
		{"vsp write factor", DUMP_FP_FMT, vsp_write_factor},
		{"tnbr/lcu", "%d", tnbr_per_lcu},
		{"collocated bytes/LCU", "%d", collocated_bytes_per_lcu},
		{"bw for NV12 8bpc)", DUMP_FP_FMT, y_bw_no_ubwc_8bpp},
		{"bw for NV12 10bpc)", DUMP_FP_FMT, y_bw_no_ubwc_10bpp},

		{"DERIVED PARAMETERS (2)", "", DUMP_HEADER_MAGIC},
		{"mv complexity", DUMP_FP_FMT, motion_vector_complexity},
		{"qsmmu_bw_overhead_factor", DUMP_FP_FMT,
			qsmmu_bw_overhead_factor},

		{"INTERMEDIATE DDR B/W", "", DUMP_HEADER_MAGIC},
		{"vsp read", DUMP_FP_FMT, ddr.vsp_read},
		{"vsp write", DUMP_FP_FMT, ddr.vsp_write},
		{"collocated read", DUMP_FP_FMT, ddr.collocated_read},
		{"collocated write", DUMP_FP_FMT, ddr.collocated_write},
		{"line buffer read", DUMP_FP_FMT, ddr.line_buffer_read},
		{"line buffer write", DUMP_FP_FMT, ddr.line_buffer_write},
		{"opb read", DUMP_FP_FMT, ddr.opb_read},
		{"opb write", DUMP_FP_FMT, ddr.opb_write},
		{"dpb read", DUMP_FP_FMT, ddr.dpb_read},
		{"dpb write", DUMP_FP_FMT, ddr.dpb_write},
		{"dpb total", DUMP_FP_FMT, dpb_total},
		{"INTERMEDIATE LLC B/W", "", DUMP_HEADER_MAGIC},
		{"llc dpb read", DUMP_FP_FMT, llc.dpb_read},
		{"llc line buffer read", DUMP_FP_FMT, llc.line_buffer_read},
		{"llc line buffer write", DUMP_FP_FMT, llc.line_buffer_write},

		};
		__dump(dump, ARRAY_SIZE(dump));
	}

	d->calc_bw_ddr = kbps(fp_round(ddr.total));
	d->calc_bw_llcc = kbps(fp_round(llc.total));

	return ret;
}

static u64 __calculate_encoder(struct vidc_bus_vote_data *d)
{
	/*
	 * XXX: Don't fool around with any of the hardcoded numbers unless you
	 * know /exactly/ what you're doing.  Many of these numbers are
	 * measured heuristics and hardcoded numbers taken from the firmware.
	 */
	/* Encoder Parameters */
	int width, height, fps, lcu_size, bitrate, lcu_per_frame,
		collocated_bytes_per_lcu, tnbr_per_lcu, dpb_bpp,
		original_color_format, vertical_tile_width, rotation;
	bool work_mode_1, original_compression_enabled,
		low_power, cropping_or_scaling,
		b_frames_enabled = false,
		llc_ref_chroma_cache_enabled = false,
		llc_top_line_buf_enabled = false,
		llc_vpss_rot_line_buf_enabled = false,
		vpss_preprocessing_enabled = false;

	unsigned int bins_to_bit_factor;
	fp_t dpb_compression_factor,
		original_compression_factor,
		original_compression_factor_y,
		y_bw_no_ubwc_8bpp, y_bw_no_ubwc_10bpp = 0, y_bw_10bpp_p010 = 0,
		input_compression_factor,
		downscaling_ratio,
		ref_y_read_bw_factor, ref_cbcr_read_bw_factor,
		recon_write_bw_factor,
		total_ref_read_crcb,
		qsmmu_bw_overhead_factor;
	fp_t integer_part, frac_part;
	unsigned long ret = 0;

	/* Output parameters */
	struct {
		fp_t vsp_read, vsp_write, collocated_read, collocated_write,
			ref_read_y, ref_read_crcb, ref_write,
			ref_write_overlap, orig_read,
			line_buffer_read, line_buffer_write,
			total;
	} ddr = {0};

	struct {
		fp_t ref_read_crcb, line_buffer, total;
	} llc = {0};

	/* Encoder Parameters setup */
	rotation = d->rotation;
	cropping_or_scaling = false;
	vertical_tile_width = 960;
	/*
	 * recon_write_bw_factor varies according to resolution and bit-depth,
	 * here use 1.08(1.075) for worst case.
	 * Similar for ref_y_read_bw_factor, it can reach 1.375 for worst case,
	 * here use 1.3 for average case, and can somewhat balance the
	 * worst case assumption for UBWC CR factors.
	 */
	recon_write_bw_factor = FP(1, 8, 100);
	ref_y_read_bw_factor = FP(1, 30, 100);
	ref_cbcr_read_bw_factor = FP(1, 50, 100);


	/* Derived Parameters */
	fps = d->fps;
	width = max(d->output_width, BASELINE_DIMENSIONS.width);
	height = max(d->output_height, BASELINE_DIMENSIONS.height);
	downscaling_ratio = fp_div(FP_INT(d->input_width * d->input_height),
		FP_INT(d->output_width * d->output_height));
	downscaling_ratio = max(downscaling_ratio, FP_ONE);
	bitrate = d->bitrate > 0 ? DIV_ROUND_UP(d->bitrate, 1000000) :
		__lut(width, height, fps)->bitrate;
	lcu_size = d->lcu_size;
	lcu_per_frame = DIV_ROUND_UP(width, lcu_size) *
		DIV_ROUND_UP(height, lcu_size);
	tnbr_per_lcu = 16;

	dpb_bpp = __bpp(d->color_formats[0]);

	y_bw_no_ubwc_8bpp = fp_div(FP_INT(width * height * fps),
		FP_INT(1000 * 1000));

	if (dpb_bpp != 8) {
		y_bw_no_ubwc_10bpp = fp_div(fp_mult(y_bw_no_ubwc_8bpp,
			FP_INT(256)), FP_INT(192));
		y_bw_10bpp_p010 = y_bw_no_ubwc_8bpp * 2;
	}

	b_frames_enabled = d->b_frames_enabled;
	original_color_format = d->num_formats >= 1 ?
		d->color_formats[0] : MSM_VIDC_FMT_NV12C;

	original_compression_enabled = __ubwc(original_color_format);

	work_mode_1 = d->work_mode == MSM_VIDC_STAGE_1;
	low_power = d->power_mode == VIDC_POWER_LOW;
	bins_to_bit_factor = 4;
	vpss_preprocessing_enabled = d->vpss_preprocessing_enabled;

	if (d->use_sys_cache) {
		llc_ref_chroma_cache_enabled = true;
		llc_top_line_buf_enabled = true,
		llc_vpss_rot_line_buf_enabled = true;
	}

	integer_part = Q16_INT(d->compression_ratio);
	frac_part = Q16_FRAC(d->compression_ratio);
	dpb_compression_factor = FP(integer_part, frac_part, 100);

	integer_part = Q16_INT(d->input_cr);
	frac_part = Q16_FRAC(d->input_cr);
	input_compression_factor = FP(integer_part, frac_part, 100);

	original_compression_factor = original_compression_factor_y =
		!original_compression_enabled ? FP_ONE :
		__compression_ratio(__lut(width, height, fps), dpb_bpp);
	/* use input cr if it is valid (not 1), otherwise use lut */
	if (original_compression_enabled &&
		input_compression_factor != FP_ONE) {
		original_compression_factor = input_compression_factor;
		/* Luma usually has lower compression factor than Chroma,
		 * input cf is overall cf, add 1.08 factor for Luma cf
		 */
		original_compression_factor_y =
			input_compression_factor > FP(1, 8, 100) ?
			fp_div(input_compression_factor, FP(1, 8, 100)) :
			input_compression_factor;
	}

	ddr.vsp_read = fp_div(FP_INT(bitrate * bins_to_bit_factor), FP_INT(8));
	ddr.vsp_write = ddr.vsp_read + fp_div(FP_INT(bitrate), FP_INT(8));

	collocated_bytes_per_lcu = lcu_size == 16 ? 16 :
				lcu_size == 32 ? 64 : 256;

	ddr.collocated_read = fp_div(FP_INT(lcu_per_frame *
			collocated_bytes_per_lcu * fps), FP_INT(bps(1)));

	ddr.collocated_write = ddr.collocated_read;

	ddr.ref_read_y = dpb_bpp == 8 ?
		y_bw_no_ubwc_8bpp : y_bw_no_ubwc_10bpp;
	if (b_frames_enabled)
		ddr.ref_read_y = ddr.ref_read_y * 2;
	ddr.ref_read_y = fp_div(ddr.ref_read_y, dpb_compression_factor);

	ddr.ref_read_crcb = fp_mult((ddr.ref_read_y / 2),
		ref_cbcr_read_bw_factor);

	if (width > vertical_tile_width) {
		ddr.ref_read_y = fp_mult(ddr.ref_read_y,
			ref_y_read_bw_factor);
	}

	if (llc_ref_chroma_cache_enabled) {
		total_ref_read_crcb = ddr.ref_read_crcb;
		ddr.ref_read_crcb = fp_div(ddr.ref_read_crcb,
					   ref_cbcr_read_bw_factor);
		llc.ref_read_crcb = total_ref_read_crcb - ddr.ref_read_crcb;
	}

	ddr.ref_write = dpb_bpp == 8 ? y_bw_no_ubwc_8bpp : y_bw_no_ubwc_10bpp;
	ddr.ref_write = fp_div(fp_mult(ddr.ref_write, FP(1, 50, 100)),
			dpb_compression_factor);

	if (width > vertical_tile_width) {
		ddr.ref_write_overlap = fp_mult(ddr.ref_write,
			(recon_write_bw_factor - FP_ONE));
		ddr.ref_write = fp_mult(ddr.ref_write, recon_write_bw_factor);
	}

	/* double ref_write */
	if (vpss_preprocessing_enabled)
		ddr.ref_write = ddr.ref_write * 2;

	ddr.orig_read = dpb_bpp == 8 ? y_bw_no_ubwc_8bpp :
		(original_compression_enabled ? y_bw_no_ubwc_10bpp :
		y_bw_10bpp_p010);
	ddr.orig_read = fp_div(fp_mult(fp_mult(ddr.orig_read, FP(1, 50, 100)),
		downscaling_ratio), original_compression_factor);
	if (rotation == 90 || rotation == 270)
		ddr.orig_read *= lcu_size == 32 ? (dpb_bpp == 8 ? 1 : 3) : 2;

	/* double orig_read */
	if (vpss_preprocessing_enabled)
		ddr.orig_read = ddr.orig_read * 2;

	ddr.line_buffer_read =
		fp_div(FP_INT(tnbr_per_lcu * lcu_per_frame * fps),
			FP_INT(bps(1)));

	ddr.line_buffer_write = ddr.line_buffer_read;
	if (llc_top_line_buf_enabled) {
		llc.line_buffer = ddr.line_buffer_read + ddr.line_buffer_write;
		ddr.line_buffer_read = ddr.line_buffer_write = FP_ZERO;
	}

	ddr.total = ddr.vsp_read + ddr.vsp_write +
		ddr.collocated_read + ddr.collocated_write +
		ddr.ref_read_y + ddr.ref_read_crcb +
		ddr.ref_write + ddr.ref_write_overlap +
		ddr.orig_read +
		ddr.line_buffer_read + ddr.line_buffer_write;

	qsmmu_bw_overhead_factor = FP(1, 3, 100);
	ddr.total = fp_mult(ddr.total, qsmmu_bw_overhead_factor);
	llc.total = llc.ref_read_crcb + llc.line_buffer + ddr.total;

	if (msm_vidc_debug & VIDC_BUS) {
		struct dump dump[] = {
		{"ENCODER PARAMETERS", "", DUMP_HEADER_MAGIC},
		{"width", "%d", width},
		{"height", "%d", height},
		{"fps", "%d", fps},
		{"dpb bitdepth", "%d", dpb_bpp},
		{"input downscaling ratio", DUMP_FP_FMT, downscaling_ratio},
		{"rotation", "%d", rotation},
		{"cropping or scaling", "%d", cropping_or_scaling},
		{"low power mode", "%d", low_power},
		{"work Mode", "%d", work_mode_1},
		{"B frame enabled", "%d", b_frames_enabled},
		{"original frame format", "%#x", original_color_format},
		{"VPSS preprocessing", "%d", vpss_preprocessing_enabled},
		{"original compression enabled", "%d",
			original_compression_enabled},
		{"dpb compression factor", DUMP_FP_FMT,
			dpb_compression_factor},
		{"input compression factor", DUMP_FP_FMT,
			input_compression_factor},
		{"llc ref chroma cache enabled", DUMP_FP_FMT,
		llc_ref_chroma_cache_enabled},
		{"llc top line buf enabled", DUMP_FP_FMT,
			llc_top_line_buf_enabled},
		{"llc vpss rot line buf enabled ", DUMP_FP_FMT,
			llc_vpss_rot_line_buf_enabled},

		{"DERIVED PARAMETERS", "", DUMP_HEADER_MAGIC},
		{"lcu size", "%d", lcu_size},
		{"bitrate (Mbit/sec)", "%lu", bitrate},
		{"bins to bit factor", "%u", bins_to_bit_factor},
		{"original compression factor", DUMP_FP_FMT,
			original_compression_factor},
		{"original compression factor y", DUMP_FP_FMT,
			original_compression_factor_y},
		{"qsmmu_bw_overhead_factor",
			 DUMP_FP_FMT, qsmmu_bw_overhead_factor},
		{"bw for NV12 8bpc)", DUMP_FP_FMT, y_bw_no_ubwc_8bpp},
		{"bw for NV12 10bpc)", DUMP_FP_FMT, y_bw_no_ubwc_10bpp},

		{"INTERMEDIATE B/W DDR", "", DUMP_HEADER_MAGIC},
		{"vsp read", DUMP_FP_FMT, ddr.vsp_read},
		{"vsp write", DUMP_FP_FMT, ddr.vsp_write},
		{"collocated read", DUMP_FP_FMT, ddr.collocated_read},
		{"collocated write", DUMP_FP_FMT, ddr.collocated_write},
		{"ref read y", DUMP_FP_FMT, ddr.ref_read_y},
		{"ref read crcb", DUMP_FP_FMT, ddr.ref_read_crcb},
		{"ref write", DUMP_FP_FMT, ddr.ref_write},
		{"ref write overlap", DUMP_FP_FMT, ddr.ref_write_overlap},
		{"original read", DUMP_FP_FMT, ddr.orig_read},
		{"line buffer read", DUMP_FP_FMT, ddr.line_buffer_read},
		{"line buffer write", DUMP_FP_FMT, ddr.line_buffer_write},
		{"INTERMEDIATE LLC B/W", "", DUMP_HEADER_MAGIC},
		{"llc ref read crcb", DUMP_FP_FMT, llc.ref_read_crcb},
		{"llc line buffer", DUMP_FP_FMT, llc.line_buffer},
		};
		__dump(dump, ARRAY_SIZE(dump));
	}

	d->calc_bw_ddr = kbps(fp_round(ddr.total));
	d->calc_bw_llcc = kbps(fp_round(llc.total));

	return ret;
}

static u64 __calculate(struct msm_vidc_inst *inst, struct vidc_bus_vote_data *d)
{
	u64 value = 0;

	switch (d->domain) {
	case MSM_VIDC_ENCODER:
		value = __calculate_encoder(d);
		break;
	case MSM_VIDC_DECODER:
		value = __calculate_decoder(d);
		break;
	default:
		i_vpr_e(inst, "%s: Unknown Domain %#x", __func__, d->domain);
	}

	return value;
}

int msm_vidc_calc_bw_iris3(struct msm_vidc_inst *inst,
		struct vidc_bus_vote_data *vidc_data)
{
	int value = 0;

	if (!vidc_data)
		return value;

	if (ENABLE_LEGACY_POWER_CALCULATIONS)
		value = __calculate(inst, vidc_data);
	else
		value = msm_vidc_calc_bw_iris3_new(inst, vidc_data);

	return value;
}