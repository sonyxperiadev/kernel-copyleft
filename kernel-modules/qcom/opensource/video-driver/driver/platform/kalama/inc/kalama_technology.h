/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "perf_static_model.h"

#define ENABLE_FINEBITRATE_SUBUHD60 0

/*
 * Chipset Generation Technology: SW/FW overhead profiling
 * need update with new numbers
 */
static u32 frequency_table_kalama[2][6] = {
	/* //make lowsvs_D1 as invalid; */
	{533, 444, 366, 338, 240, 0},
	{800, 666, 549, 507, 360, 0},
};

 /*
  * TODO Move to kalama.c
  * TODO Replace hardcoded values with
  * ENCODER_VPP_TARGET_CLK_PER_MB_KALAMA in CPP file.
  */

 /* Tensilica cycles */
#define DECODER_VPP_FW_OVERHEAD_KALAMA                                                  66234

/* Tensilica cycles; this is measured in Lahaina 1stage with FW profiling */
#define DECODER_VPPVSP1STAGE_FW_OVERHEAD_KALAMA                                         93000

#define DECODER_VSP_FW_OVERHEAD_KALAMA \
	(DECODER_VPPVSP1STAGE_FW_OVERHEAD_KALAMA - DECODER_VPP_FW_OVERHEAD_KALAMA)

/* Tensilica cycles; encoder has ARP register */
#define ENCODER_VPP_FW_OVERHEAD_KALAMA                                                  48405

#define ENCODER_VPPVSP1STAGE_FW_OVERHEAD_KALAMA \
	(ENCODER_VPP_FW_OVERHEAD_KALAMA + DECODER_VSP_FW_OVERHEAD_KALAMA)

#define DECODER_SW_OVERHEAD_KALAMA                                                      489583
#define ENCODER_SW_OVERHEAD_KALAMA                                                      489583

/* Video IP Core Technology: pipefloor and pipe penlaty */
static u32 encoder_vpp_target_clk_per_mb_kalama[2] = {320, 675};
static u32 decoder_vpp_target_clk_per_mb_kalama = 200;

/*
 * These pipe penalty numbers only applies to 4 pipe
 * For 2pipe and 1pipe, these numbers need recalibrate
 */
static u32 pipe_penalty_kalama[3][3] = {
	/* NON AV1 */
	{1059, 1059, 1059},
	/* AV1 RECOMMENDED TILE 1080P_V2XH1, UHD_V2X2, 8KUHD_V8X2 */
	{1410, 1248, 1226},
	/* AV1 YOUTUBE/NETFLIX TILE 1080P_V4XH2_V4X1, UHD_V8X4_V8X1, 8KUHD_V8X8_V8X1 */
	{2039, 2464, 1191},
};

/*
 * Video IP Core Technology: bitrate constraint
 * HW limit bitrate table (these values are measured end to end fw/sw impacts are also considered)
 * TODO Can we convert to Cycles/MB? This will remove DIVISION.
 */
static u32 bitrate_table_kalama_2stage_fp[5][10] = {
	/* h264 cavlc */
	{0, 220, 220, 220, 220, 220, 220, 220, 220, 220},
	/* h264 cabac */
	{0, 140, 150, 160, 175, 190, 190, 190, 190, 190},
	/* h265 */
	{90, 140, 160, 180, 190, 200, 200, 200, 200, 200},
	/* vp9 */
	{90, 90, 90, 90, 90, 90, 90, 90, 90, 90},
	/* av1 */
	{130, 130, 120, 120, 120, 120, 120, 120, 120, 120},
};

/*
 * HW limit bitrate table (these values are measured
 * end to end fw/sw impacts are also considered)
 */
static u32 bitrate_table_kalama_1stage_fp[5][10] = { /* 1-stage assume IPPP */
	/* h264 cavlc */
	{0, 220, 220, 220, 220, 220, 220, 220, 220, 220},
	/* h264 cabac */
	{0, 110, 150, 150, 150, 150, 150, 150, 150, 150},
	/* h265 */
	{0, 140, 150, 150, 150, 150, 150, 150, 150, 150},
	/* vp9 */
	{0, 70, 70, 70, 70, 70, 70, 70, 70, 70},
	/* av1 */
	{0, 100, 100, 100, 100, 100, 100, 100, 100, 100},
};

/* rec pwc and power bitrate table */
static u32 bitrate_table_kalama_rec_fp[5][10] = {
	/* rec. worst bitrate based on bitrate table */
#if ENABLE_FINEBITRATE_SUBUHD60
	/* h264 cavlc */
	{0, 168, 150, 120, 100, 90, 50, 32, 20, 14},
	/* h264 cabac 8bit */
	{0, 134, 109, 84, 67, 56, 35, 23, 14, 10},
	/* h265 10bit assumption */
	{70, 140, 116, 92, 74, 62, 39, 25, 16, 11},
	/* vp9 (profiled content from youtube and nflx) */
	{70, 70, 65, 55, 45, 35, 20, 8, 6, 5},
	/* av1 (profiled content from youtube and nflx) */
	{100, 100, 85, 70, 55, 30, 15, 5, 5, 5},
#else
	/* h264 cavlc */
	{0, 168, 150, 120, 100, 90, 90, 90, 90, 90},
	/* h264 cabac 8bit */
	{0, 134, 109, 84, 67, 56, 56, 56, 56, 56},
	/* h265 10bit assumption */
	{70, 140, 116, 92, 74, 62, 62, 62, 62, 62},
	/* vp9 */
	{70, 70, 65, 55, 45, 35, 35, 35, 35, 35},
	/* av1 */
	{100, 100, 85, 70, 55, 50, 50, 50, 50, 50},
#endif
};

static u32 input_bitrate_fp;

/* 8KUHD60; UHD240; 1080p960  with B */
static u32 fp_pixel_count_bar0 = 3840 * 2160 * 240;
/* 8KUHD60; UHD240; 1080p960  without B */
static u32 fp_pixel_count_bar1 = 3840 * 2160 * 240;
/* 1080p720 */
static u32 fp_pixel_count_bar2 = 3840 * 2160 * 180;
/* UHD120 */
static u32 fp_pixel_count_bar3 = 3840 * 2160 * 120;
/* UHD90 */
static u32 fp_pixel_count_bar4 = 3840 * 2160 * 90;
/* UHD60 */
static u32 fp_pixel_count_bar5 = 3840 * 2160 * 60;
/* UHD30; FHD120; HD240 */
static u32 fp_pixel_count_bar6 = 3840 * 2160 * 30;
/* FHD60 */
static u32 fp_pixel_count_bar7 = 1920 * 1080 * 60;
/* FHD30 */
static u32 fp_pixel_count_bar8 = 1920 * 1080 * 30;
/* HD30 */
static u32 fp_pixel_count_bar9 = 1280 * 720 * 30;

static u32 codec_encoder_gop_complexity_table_fp[8][3];
static u32 codec_mbspersession_kalama;

static u32 cr_table_basic_kalama[7][4] = {
	{1920, 1080, 20, 40},
	{3840, 2160, 42, 84},
	{4096, 2160, 44, 88},
	{4096, 2304, 48, 96},
	{1280, 720, 7, 14},
	{2560, 1440, 32, 64},
	{7680, 4320, 84, 168},
};

/* 100x */
static u32 dpbopb_ubwc30_cr_table_cratio_kalama[7][12] = {
	{237, 399, 272, 137, 225, 158, 185, 259, 203, 138, 167, 152},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{237, 399, 272, 137, 225, 158, 185, 259, 203, 138, 167, 152},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
	{269, 404, 302, 202, 367, 238, 210, 299, 232, 134, 181, 149},
};

/* 100x */
static u32 rpb_ubwc30_cr_table_cratio_kalama[7][12] = {
	{193, 294, 218, 135, 214, 155, 175, 241, 191, 139, 162, 149},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{193, 294, 218, 135, 214, 155, 175, 241, 191, 139, 162, 149},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
	{285, 406, 316, 207, 373, 243, 201, 280, 221, 139, 177, 152},
};

/* 100x */
static u32 ipblossy_ubwc30_cr_table_cratio_kalama[7][12] = {
	{215, 215, 215, 174, 174, 174, 266, 266, 266, 231, 231, 231},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{215, 215, 215, 174, 174, 174, 266, 266, 266, 231, 231, 231},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
	{254, 254, 254, 219, 219, 219, 292, 292, 292, 249, 249, 249},
};

/* 100x */
static u32 ipblossless_ubwc30_cr_table_cratio_kalama[7][12] = {
	{185, 215, 194, 147, 178, 159, 162, 181, 169, 138, 161, 146},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{185, 215, 194, 147, 178, 159, 162, 181, 169, 138, 161, 146},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
	{186, 217, 195, 151, 183, 161, 164, 182, 170, 140, 168, 148},
};

/* 100x */
static u32 en_original_compression_factor_rgba_pwd_kalama = 243;
/* 100x */
static u32 en_original_compression_factor_rgba_avg_kalama = 454;

static u32 av1_num_tiles_kalama[7][3] = {
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
static u32 kalama_en_readfactor[7] = {1000, 1500, 1750, 1875, 1000, 2000, 2000};
/* 1000x */
static u32 kalama_en_writefactor[7] = {1000, 500, 500, 500, 1000, 0, 1000};
static u32 kalama_en_frame_num_parallel = 1;
