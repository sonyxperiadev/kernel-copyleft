/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _PERF_STATIC_MODEL_H_
#define _PERF_STATIC_MODEL_H_

#include <linux/types.h>

/* Reordered CODECS to match Bitrate Table rows */
#define CODEC_H264_CAVLC                        0
#define CODEC_H264                              1
#define CODEC_HEVC                              2
#define CODEC_VP9                               3
#define CODEC_AV1                               4

#define CODEC_BSE_FrameFactor                   0
#define CODEC_BSE_MBFactor                      1
#define CODEC_BSE_LUC_SIZE                      2

#define CODEC_GOP_IPP                           0
#define CODEC_GOP_IbP                           1
#define CODEC_GOP_I1B2b1P                       2
#define CODEC_GOP_I3B4b1P                       3
#define CODEC_GOP_PONLY                         4
#define CODEC_GOP_bONLY                         5
#define CODEC_GOP_BONLY                         6
#define CODEC_GOP_IONLY                         7

#define CODEC_ENCODER_GOP_Bb_ENTRY              0
#define CODEC_ENCODER_GOP_P_ENTRY               1
#define CODEC_ENCODER_GOP_FACTORY_ENTRY         2

#define CODEC_ENTROPY_CODING_CAVLC              0
#define CODEC_ENTROPY_CODING_CABAC              1

#define CODEC_VSPVPP_MODE_1S                    1
#define CODEC_VSPVPP_MODE_2S                    2

#define COMP_SETTING_PWC                        0
#define COMP_SETTING_AVG                        1
#define COMP_SETTING_POWER                      2

#define CODEC_BITDEPTH_8                        8
#define CODEC_BITDEPTH_10                       10

#define ENCODE_YUV                              0
#define ENCODE_RGB                              1

#define COMPLEXITY_PWC                          0
#define COMPLEXITY_AVG                          1
#define COMPLEXITY_POWER                        2

#define MAX_LINE                                2048
#ifndef VENUS_MAX_FILENAME_LENGTH
#define VENUS_MAX_FILENAME_LENGTH               1024
#endif

#define CODEC_ENCODER                           1
#define CODEC_DECODER                           2

#define COMPLEXITY_THRESHOLD                    2

enum chipset_generation {
	MSM_KONA = 0,
	MSM_LAHAINA,
	MSM_WAIPIO,
	MSM_MAKENA,
	MSM_KALAMA,
	MSM_QOGNITION,
	MSM_PINEAPPLE,
	MSM_MAX,
};

enum regression_mode {
	/* ignores client set cr and bitrate settings */
	REGRESSION_MODE_SANITY = 1,
	/* cr and bitrate default mode */
	REGRESSION_MODE_DEFAULT,
	/* custom mode where client will set cr and bitrate values */
	REGRESSION_MODE_CUSTOM,
};

/*
 * If firmware provided motion_vector_complexity is >= 2 then set the
 * complexity_setting as PWC (performance worst case)
 * If the motion_vector_complexity is < 2 then set the complexity_setting
 * as AVG (average case value)
 */
enum complexity_setting {
	COMPLEXITY_SETTING_PWC = 0,
	COMPLEXITY_SETTING_AVG = 1,
	COMPLEXITY_SETTING_PWR = 2,
};

/*
 * If firmware provided motion_vector_complexity is >= 2 then set the
 * refframe_complexity as PWC (performance worst case)
 * If the motion_vector_complexity is < 2 then set the refframe_complexity
 * as AVG (average case value)
 */
enum refframe_complexity {
	REFFRAME_COMPLEXITY_PWC = 4,
	REFFRAME_COMPLEXITY_AVG = 2,
	REFFRAME_COMPLEXITY_PWR = 1,
};

struct api_calculation_input {
	/*2: decoder; 1: encoder */
	u32 decoder_or_encoder;

	/* enum chipset_generation */
	u32 chipset_gen;

	u32 codec;
	u32 lcu_size;
	u32 pipe_num;
	u32 frame_rate;
	u32 frame_width;
	u32 frame_height;
	u32 vsp_vpp_mode;
	u32 entropy_coding_mode;
	u32 hierachical_layer;

	/* PWC, AVG/POWER */
	u32 complexity_setting;

	u32 status_llc_onoff;
	u32 bitdepth;
	u32 linear_opb;

	/* AV1D FG */
	u32 split_opb;

	u32 linear_ipb;
	u32 lossy_ipb;
	u32 ipb_yuvrgb;
	u32 encoder_multiref;
	u32 bitrate_mbps;
	u32 refframe_complexity;
	u32 cr_ipb;
	u32 cr_rpb;
	u32 cr_dpb;
	u32 cr_opb;
	u32  av1d_commer_tile_enable;
	u32 regression_mode;

	/* used in aurora for depth map decode */
	u32 lumaonly_decode;
};

struct corner_voting {
	u32 percent_lowbound;
	u32 percent_highbound;
};

struct api_calculation_freq_output {
	u32 vpp_min_freq;
	u32 vsp_min_freq;
	u32 tensilica_min_freq;
	u32 hw_min_freq;
	u32 enc_hqmode;
	struct corner_voting usecase_corner;
};

struct api_calculation_bw_output {
	u32 vsp_read_noc;
	u32 vsp_write_noc;
	u32 vsp_read_ddr;
	u32 vsp_write_ddr;
	u32 vsp_rd_wr_total_noc;
	u32 vsp_rd_wr_total_ddr;

	u32 collocated_rd_noc;
	u32 collocated_wr_noc;
	u32 collocated_rd_ddr;
	u32 collocated_wr_ddr;
	u32 collocated_rd_wr_total_noc;
	u32 collocated_rd_wr_total_ddr;

	u32 dpb_rd_y_noc;
	u32 dpb_rd_crcb_noc;
	u32 dpb_rdwr_duetooverlap_noc;
	u32 dpb_wr_noc;
	u32 dpb_rd_y_ddr;
	u32 dpb_rd_crcb_ddr;
	u32 dpb_rdwr_duetooverlap_ddr;
	u32 dpb_wr_ddr;
	u32 dpb_rd_wr_total_noc;
	u32 dpb_rd_wr_total_ddr;

	u32 opb_write_total_noc;
	u32 opb_write_total_ddr;

	u32 ipb_rd_total_noc;
	u32 ipb_rd_total_ddr;

	u32 bse_tlb_rd_noc;
	u32 bse_tlb_wr_noc;
	u32 bse_tlb_rd_ddr;
	u32 bse_tlb_wr_ddr;
	u32 bse_rd_wr_total_noc;
	u32 bse_rd_wr_total_ddr;

	u32 statistics_rd_noc;
	u32 statistics_wr_noc;
	u32 statistics_rd_ddr;
	u32 statistics_wr_ddr;

	u32 mmu_rd_noc;
	u32 mmu_rd_ddr;

	u32 noc_bw_rd;
	u32 noc_bw_wr;
	u32 ddr_bw_rd;
	u32 ddr_bw_wr;

	/* llc BW components for aurora */
	u32 dpb_rd_y_llc;
	u32 dpb_rd_crcb_llc;
	u32 dpb_wr_llc;
	u32 bse_tlb_rd_llc;
	u32 bse_tlb_wr_llc;
	u32 vsp_read_llc;
	u32 vsp_write_llc;

	u32 llc_bw_rd;
	u32 llc_bw_wr;
};

int msm_vidc_calculate_frequency(struct api_calculation_input codec_input,
				 struct api_calculation_freq_output *codec_output);
int msm_vidc_calculate_bandwidth(struct api_calculation_input codec_input,
				 struct api_calculation_bw_output *codec_output);

#endif /*_PERF_STATIC_MODEL_H_ */
