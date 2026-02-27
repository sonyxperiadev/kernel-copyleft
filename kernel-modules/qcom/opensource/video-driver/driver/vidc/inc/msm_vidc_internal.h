/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MSM_VIDC_INTERNAL_H_
#define _MSM_VIDC_INTERNAL_H_

#include <linux/version.h>
#include <linux/bits.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/sync_file.h>
#include <linux/dma-fence.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>

struct msm_vidc_inst;

/* start of vidc specific colorspace definitions */
/*
 * V4L2_COLORSPACE_VIDC_START, V4L2_XFER_FUNC_VIDC_START
 * and V4L2_YCBCR_VIDC_START are introduced because
 * V4L2_COLORSPACE_LAST, V4L2_XFER_FUNC_LAST, and
 * V4L2_YCBCR_ENC_LAST respectively are not accessible
 * in userspace. These values are needed in userspace
 * to check if the colorspace info is private.
 */
#define V4L2_COLORSPACE_VIDC_START           100
#define V4L2_COLORSPACE_VIDC_GENERIC_FILM    101
#define V4L2_COLORSPACE_VIDC_EG431           102
#define V4L2_COLORSPACE_VIDC_EBU_TECH        103

#define V4L2_XFER_FUNC_VIDC_START            200
#define V4L2_XFER_FUNC_VIDC_BT470_SYSTEM_M   201
#define V4L2_XFER_FUNC_VIDC_BT470_SYSTEM_BG  202
#define V4L2_XFER_FUNC_VIDC_BT601_525_OR_625 203
#define V4L2_XFER_FUNC_VIDC_LINEAR           204
#define V4L2_XFER_FUNC_VIDC_XVYCC            205
#define V4L2_XFER_FUNC_VIDC_BT1361           206
#define V4L2_XFER_FUNC_VIDC_BT2020           207
#define V4L2_XFER_FUNC_VIDC_ST428            208
#define V4L2_XFER_FUNC_VIDC_HLG              209

/* should be 255 or below due to u8 limitation */
#define V4L2_YCBCR_VIDC_START                240
#define V4L2_YCBCR_VIDC_SRGB_OR_SMPTE_ST428  241
#define V4L2_YCBCR_VIDC_FCC47_73_682         242
/* end of vidc specific colorspace definitions */

/* TODO : remove once available in mainline kernel */
#ifndef V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10_STILL_PICTURE
#define V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10_STILL_PICTURE    (3)
#endif

enum msm_vidc_blur_types {
	MSM_VIDC_BLUR_NONE               = 0x0,
	MSM_VIDC_BLUR_EXTERNAL           = 0x1,
	MSM_VIDC_BLUR_ADAPTIVE           = 0x2,
};

/* various Metadata - encoder & decoder */
enum msm_vidc_metadata_bits {
	MSM_VIDC_META_DISABLE          = 0x0,
	MSM_VIDC_META_ENABLE           = 0x1,
	MSM_VIDC_META_TX_INPUT         = 0x2,
	MSM_VIDC_META_TX_OUTPUT        = 0x4,
	MSM_VIDC_META_RX_INPUT         = 0x8,
	MSM_VIDC_META_RX_OUTPUT        = 0x10,
	MSM_VIDC_META_DYN_ENABLE       = 0x20,
	MSM_VIDC_META_MAX              = 0x40,
};

#define MSM_VIDC_METADATA_SIZE             (4 * 4096) /* 16 KB */
#define ENCODE_INPUT_METADATA_SIZE         (512 * 4096) /* 2 MB */
#define DECODE_INPUT_METADATA_SIZE         MSM_VIDC_METADATA_SIZE
#define MSM_VIDC_METADATA_DOLBY_RPU_SIZE   (41 * 1024) /* 41 KB */

#define MAX_NAME_LENGTH   128
#define VENUS_VERSION_LENGTH 128
#define MAX_MATRIX_COEFFS 9
#define MAX_BIAS_COEFFS   3
#define MAX_LIMIT_COEFFS  6
#define MAX_DEBUGFS_NAME  50
#define DEFAULT_HEIGHT    240
#define DEFAULT_WIDTH     320
#define DEFAULT_FPS       30
#define MAXIMUM_VP9_FPS   60
#define NRT_PRIORITY_OFFSET        2
#define RT_DEC_DOWN_PRORITY_OFFSET 1
#define MAX_SUPPORTED_INSTANCES  16
#define DEFAULT_BSE_VPP_DELAY    2
#define MAX_CAP_PARENTS          20
#define MAX_CAP_CHILDREN         20
#define DEFAULT_MAX_HOST_BUF_COUNT  64
#define DEFAULT_MAX_HOST_BURST_BUF_COUNT 256
#define BIT_DEPTH_8 (8 << 16 | 8)
#define BIT_DEPTH_10 (10 << 16 | 10)
#define CODED_FRAMES_PROGRESSIVE 0x0
#define CODED_FRAMES_INTERLACE 0x1
#define MAX_VP9D_INST_COUNT     6
/* TODO: move below macros to waipio.c */
#define MAX_ENH_LAYER_HB        3
#define MAX_HEVC_VBR_ENH_LAYER_SLIDING_WINDOW         5
#define MAX_HEVC_NON_VBR_ENH_LAYER_SLIDING_WINDOW     3
#define MAX_AVC_ENH_LAYER_SLIDING_WINDOW      3
#define MAX_AVC_ENH_LAYER_HYBRID_HP           5
#define INVALID_DEFAULT_MARK_OR_USE_LTR      -1
#define MAX_SLICES_PER_FRAME                 10
#define MAX_SLICES_FRAME_RATE                60
#define MAX_MB_SLICE_WIDTH                 4096
#define MAX_MB_SLICE_HEIGHT                2160
#define MAX_BYTES_SLICE_WIDTH              1920
#define MAX_BYTES_SLICE_HEIGHT             1088
#define MIN_HEVC_SLICE_WIDTH                384
#define MIN_AVC_SLICE_WIDTH                 192
#define MIN_SLICE_HEIGHT                    128
#define MAX_BITRATE_BOOST                    25
#define MAX_SUPPORTED_MIN_QUALITY            70
#define MIN_CHROMA_QP_OFFSET                -12
#define MAX_CHROMA_QP_OFFSET                  0
#define MIN_QP_10BIT                        -11
#define MIN_QP_8BIT                           1
#define INVALID_FD                           -1
#define INVALID_CLIENT_ID                    -1
#define MAX_ENCODING_REFERNCE_FRAMES          7
#define MAX_LTR_FRAME_COUNT_5                 5
#define MAX_LTR_FRAME_COUNT_2                 2
#define MAX_ENC_RING_BUF_COUNT                5 /* to be tuned */
#define MAX_TRANSCODING_STATS_FRAME_RATE     60
#define MAX_TRANSCODING_STATS_WIDTH        4096
#define MAX_TRANSCODING_STATS_HEIGHT       2304
#define HEIC_GRID_WIDTH                     512

#define DCVS_WINDOW 16
#define ENC_FPS_WINDOW 3
#define DEC_FPS_WINDOW 10
#define INPUT_TIMER_LIST_SIZE 30

#define DEFAULT_COMPLEXITY 50

#define INPUT_MPLANE V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
#define OUTPUT_MPLANE V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
#define INPUT_META_PLANE V4L2_BUF_TYPE_META_OUTPUT
#define OUTPUT_META_PLANE V4L2_BUF_TYPE_META_CAPTURE

#define VIDC_IFACEQ_MAX_PKT_SIZE                1024
#define VIDC_IFACEQ_MED_PKT_SIZE                768
#define VIDC_IFACEQ_MIN_PKT_SIZE                8
#define VIDC_IFACEQ_VAR_SMALL_PKT_SIZE          100
#define VIDC_IFACEQ_VAR_LARGE_PKT_SIZE          512
#define VIDC_IFACEQ_VAR_HUGE_PKT_SIZE          (1024 * 4)

#define NUM_MBS_PER_SEC(__height, __width, __fps) \
	(NUM_MBS_PER_FRAME(__height, __width) * __fps)

#define NUM_MBS_PER_FRAME(__height, __width) \
	((ALIGN(__height, 16) / 16) * (ALIGN(__width, 16) / 16))

#ifdef V4L2_CTRL_CLASS_CODEC
#define IS_PRIV_CTRL(idx) ( \
	(V4L2_CTRL_ID2WHICH(idx) == V4L2_CTRL_CLASS_CODEC) && \
	V4L2_CTRL_DRIVER_PRIV(idx))
#else
#define IS_PRIV_CTRL(idx) ( \
	(V4L2_CTRL_ID2WHICH(idx) == V4L2_CTRL_CLASS_MPEG) && \
	V4L2_CTRL_DRIVER_PRIV(idx))
#endif

#define BUFFER_ALIGNMENT_SIZE(x) x
#define NUM_MBS_360P (((480 + 15) >> 4) * ((360 + 15) >> 4))
#define NUM_MBS_720P (((1280 + 15) >> 4) * ((720 + 15) >> 4))
#define NUM_MBS_4k (((4096 + 15) >> 4) * ((2304 + 15) >> 4))
#define MB_SIZE_IN_PIXEL (16 * 16)

#define DB_H264_DISABLE_SLICE_BOUNDARY \
		V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY

#define DB_HEVC_DISABLE_SLICE_BOUNDARY \
		V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY

/*
 * Convert Q16 number into Integer and Fractional part upto 2 places.
 * Ex : 105752 / 65536 = 1.61; 1.61 in Q16 = 105752;
 * Integer part =  105752 / 65536 = 1;
 * Reminder = 105752 * 0xFFFF = 40216; Last 16 bits.
 * Fractional part = 40216 * 100 / 65536 = 61;
 * Now convert to FP(1, 61, 100).
 */
#define Q16_INT(q) ((q) >> 16)
#define Q16_FRAC(q) ((((q) & 0xFFFF) * 100) >> 16)

/* define timeout values */
#define HW_RESPONSE_TIMEOUT_VALUE     (1000)
#define SW_PC_DELAY_VALUE             (HW_RESPONSE_TIMEOUT_VALUE + 500)
#define FW_UNLOAD_DELAY_VALUE         (SW_PC_DELAY_VALUE + 1500)

#define MAX_DPB_COUNT 32
 /*
  * max dpb count in firmware = 16
  * each dpb: 4 words - <base_address, addr_offset, data_offset>
  * dpb list array size = 16 * 4
  * dpb payload size = 16 * 4 * 4
  */
#define MAX_DPB_LIST_ARRAY_SIZE (16 * 4)
#define MAX_DPB_LIST_PAYLOAD_SIZE (16 * 4 * 4)

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) (#STRING),

/* append MSM_VIDC_ to prepare enum */
#define GENERATE_MSM_VIDC_ENUM(ENUM) MSM_VIDC_##ENUM,

/* append MSM_VIDC_BUF_ to prepare enum */
#define GENERATE_MSM_VIDC_BUF_ENUM(ENUM) MSM_VIDC_BUF_##ENUM,

/**
 * msm_vidc_prepare_dependency_list() api will prepare caps_list by looping over
 * enums(msm_vidc_inst_capability_type) from 0 to INST_CAP_MAX and arranges the
 * node in such a way that parents willbe at the front and dependent children
 * in the back.
 *
 * caps_list preparation may become CPU intensive task, so to save CPU cycles,
 * organize enum in proper order(leaf caps at the beginning and dependent parent caps
 * at back), so that during caps_list preparation num CPU cycles spent will reduce.
 *
 * Note: It will work, if enum kept at different places, but not efficient.
 *
 * - place all metadata cap(META_*) af the front.
 * - place all leaf(no child) enums before PROFILE cap.
 * - place all intermittent(having both parent and child) enums before FRAME_WIDTH cap.
 * - place all root(no parent) enums before INST_CAP_MAX cap.
 */
#define FOREACH_CAP(CAP) {                        \
	CAP(INST_CAP_NONE)                        \
	CAP(META_SEQ_HDR_NAL)                     \
	CAP(META_BITSTREAM_RESOLUTION)            \
	CAP(META_CROP_OFFSETS)                    \
	CAP(META_DPB_MISR)                        \
	CAP(META_OPB_MISR)                        \
	CAP(META_INTERLACE)                       \
	CAP(META_OUTBUF_FENCE)                    \
	CAP(META_LTR_MARK_USE)                    \
	CAP(META_TIMESTAMP)                       \
	CAP(META_CONCEALED_MB_CNT)                \
	CAP(META_HIST_INFO)                       \
	CAP(META_PICTURE_TYPE)                    \
	CAP(META_SEI_MASTERING_DISP)              \
	CAP(META_SEI_CLL)                         \
	CAP(META_HDR10PLUS)                       \
	CAP(META_BUF_TAG)                         \
	CAP(META_DPB_TAG_LIST)                    \
	CAP(META_SUBFRAME_OUTPUT)                 \
	CAP(META_ENC_QP_METADATA)                 \
	CAP(META_DEC_QP_METADATA)                 \
	CAP(META_MAX_NUM_REORDER_FRAMES)          \
	CAP(META_EVA_STATS)                       \
	CAP(META_ROI_INFO)                        \
	CAP(META_SALIENCY_INFO)                   \
	CAP(META_TRANSCODING_STAT_INFO)           \
	CAP(META_DOLBY_RPU)                       \
	CAP(DRV_VERSION)                          \
	CAP(MIN_FRAME_QP)                         \
	CAP(MAX_FRAME_QP)                         \
	CAP(I_FRAME_QP)                           \
	CAP(P_FRAME_QP)                           \
	CAP(B_FRAME_QP)                           \
	CAP(TIME_DELTA_BASED_RC)                  \
	CAP(CONSTANT_QUALITY)                     \
	CAP(VBV_DELAY)                            \
	CAP(PEAK_BITRATE)                         \
	CAP(ENTROPY_MODE)                         \
	CAP(TRANSFORM_8X8)                        \
	CAP(STAGE)                                \
	CAP(LTR_COUNT)                            \
	CAP(IR_PERIOD)                            \
	CAP(BITRATE_BOOST)                        \
	CAP(BLUR_RESOLUTION)                      \
	CAP(OUTPUT_ORDER)                         \
	CAP(INPUT_BUF_HOST_MAX_COUNT)             \
	CAP(OUTPUT_BUF_HOST_MAX_COUNT)            \
	CAP(DELIVERY_MODE)                        \
	CAP(VUI_TIMING_INFO)                      \
	CAP(SLICE_DECODE)                         \
	CAP(INBUF_FENCE_TYPE)                     \
	CAP(OUTBUF_FENCE_TYPE)                    \
	CAP(INBUF_FENCE_DIRECTION)                \
	CAP(OUTBUF_FENCE_DIRECTION)               \
	CAP(PROFILE)                              \
	CAP(ENH_LAYER_COUNT)                      \
	CAP(BIT_RATE)                             \
	CAP(LOWLATENCY_MODE)                      \
	CAP(GOP_SIZE)                             \
	CAP(B_FRAME)                              \
	CAP(ALL_INTRA)                            \
	CAP(MIN_QUALITY)                          \
	CAP(CONTENT_ADAPTIVE_CODING)              \
	CAP(BLUR_TYPES)                           \
	CAP(REQUEST_PREPROCESS)                   \
	CAP(SLICE_MODE)                           \
	CAP(FRAME_WIDTH)                          \
	CAP(LOSSLESS_FRAME_WIDTH)                 \
	CAP(SECURE_FRAME_WIDTH)                   \
	CAP(FRAME_HEIGHT)                         \
	CAP(LOSSLESS_FRAME_HEIGHT)                \
	CAP(SECURE_FRAME_HEIGHT)                  \
	CAP(PIX_FMTS)                             \
	CAP(MIN_BUFFERS_INPUT)                    \
	CAP(MIN_BUFFERS_OUTPUT)                   \
	CAP(MBPF)                                 \
	CAP(BATCH_MBPF)                           \
	CAP(BATCH_FPS)                            \
	CAP(LOSSLESS_MBPF)                        \
	CAP(SECURE_MBPF)                          \
	CAP(FRAME_RATE)                           \
	CAP(OPERATING_RATE)                       \
	CAP(INPUT_RATE)                           \
	CAP(TIMESTAMP_RATE)                       \
	CAP(SCALE_FACTOR)                         \
	CAP(MB_CYCLES_VSP)                        \
	CAP(MB_CYCLES_VPP)                        \
	CAP(MB_CYCLES_LP)                         \
	CAP(MB_CYCLES_FW)                         \
	CAP(MB_CYCLES_FW_VPP)                     \
	CAP(ENC_RING_BUFFER_COUNT)                \
	CAP(CLIENT_ID)                            \
	CAP(SECURE_MODE)                          \
	CAP(FENCE_ID)                             \
	CAP(FENCE_FD)                             \
	CAP(FENCE_ERROR_DATA_CORRUPT)             \
	CAP(TS_REORDER)                           \
	CAP(HFLIP)                                \
	CAP(VFLIP)                                \
	CAP(ROTATION)                             \
	CAP(SUPER_FRAME)                          \
	CAP(HEADER_MODE)                          \
	CAP(PREPEND_SPSPPS_TO_IDR)                \
	CAP(WITHOUT_STARTCODE)                    \
	CAP(NAL_LENGTH_FIELD)                     \
	CAP(REQUEST_I_FRAME)                      \
	CAP(BITRATE_MODE)                         \
	CAP(LOSSLESS)                             \
	CAP(FRAME_SKIP_MODE)                      \
	CAP(FRAME_RC_ENABLE)                      \
	CAP(GOP_CLOSURE)                          \
	CAP(CSC)                                  \
	CAP(CSC_CUSTOM_MATRIX)                    \
	CAP(USE_LTR)                              \
	CAP(MARK_LTR)                             \
	CAP(BASELAYER_PRIORITY)                   \
	CAP(IR_TYPE)                              \
	CAP(AU_DELIMITER)                         \
	CAP(GRID_ENABLE)                          \
	CAP(GRID_SIZE)                            \
	CAP(I_FRAME_MIN_QP)                       \
	CAP(P_FRAME_MIN_QP)                       \
	CAP(B_FRAME_MIN_QP)                       \
	CAP(I_FRAME_MAX_QP)                       \
	CAP(P_FRAME_MAX_QP)                       \
	CAP(B_FRAME_MAX_QP)                       \
	CAP(LAYER_TYPE)                           \
	CAP(LAYER_ENABLE)                         \
	CAP(L0_BR)                                \
	CAP(L1_BR)                                \
	CAP(L2_BR)                                \
	CAP(L3_BR)                                \
	CAP(L4_BR)                                \
	CAP(L5_BR)                                \
	CAP(LEVEL)                                \
	CAP(HEVC_TIER)                            \
	CAP(AV1_TIER)                             \
	CAP(DISPLAY_DELAY_ENABLE)                 \
	CAP(DISPLAY_DELAY)                        \
	CAP(CONCEAL_COLOR_8BIT)                   \
	CAP(CONCEAL_COLOR_10BIT)                  \
	CAP(LF_MODE)                              \
	CAP(LF_ALPHA)                             \
	CAP(LF_BETA)                              \
	CAP(SLICE_MAX_BYTES)                      \
	CAP(SLICE_MAX_MB)                         \
	CAP(MB_RC)                                \
	CAP(CHROMA_QP_INDEX_OFFSET)               \
	CAP(PIPE)                                 \
	CAP(POC)                                  \
	CAP(MAX_NUM_REORDER_FRAMES)               \
	CAP(CODED_FRAMES)                         \
	CAP(BIT_DEPTH)                            \
	CAP(CODEC_CONFIG)                         \
	CAP(BITSTREAM_SIZE_OVERWRITE)             \
	CAP(THUMBNAIL_MODE)                       \
	CAP(DEFAULT_HEADER)                       \
	CAP(RAP_FRAME)                            \
	CAP(SEQ_CHANGE_AT_SYNC_FRAME)             \
	CAP(QUALITY_MODE)                         \
	CAP(PRIORITY)                             \
	CAP(FIRMWARE_PRIORITY_OFFSET)             \
	CAP(CRITICAL_PRIORITY)                    \
	CAP(RESERVE_DURATION)                     \
	CAP(FILM_GRAIN)                           \
	CAP(SUPER_BLOCK)                          \
	CAP(DRAP)                                 \
	CAP(ENC_IP_CR)                            \
	CAP(COMPLEXITY)                           \
	CAP(CABAC_MAX_BITRATE)                    \
	CAP(CAVLC_MAX_BITRATE)                    \
	CAP(ALLINTRA_MAX_BITRATE)                 \
	CAP(LOWLATENCY_MAX_BITRATE)               \
	CAP(LAST_FLAG_EVENT_ENABLE)               \
	CAP(NUM_COMV)                             \
	CAP(SIGNAL_COLOR_INFO)                    \
	CAP(INST_CAP_MAX)                         \
}

#define FOREACH_BUF_TYPE(BUF_TYPE) {              \
	BUF_TYPE(NONE)                            \
	BUF_TYPE(INPUT)                           \
	BUF_TYPE(OUTPUT)                          \
	BUF_TYPE(INPUT_META)                      \
	BUF_TYPE(OUTPUT_META)                     \
	BUF_TYPE(READ_ONLY)                       \
	BUF_TYPE(INTERFACE_QUEUE)                 \
	BUF_TYPE(BIN)                             \
	BUF_TYPE(ARP)                             \
	BUF_TYPE(COMV)                            \
	BUF_TYPE(NON_COMV)                        \
	BUF_TYPE(LINE)                            \
	BUF_TYPE(DPB)                             \
	BUF_TYPE(PERSIST)                         \
	BUF_TYPE(VPSS)                            \
	BUF_TYPE(PARTIAL_DATA)                    \
}

#define FOREACH_ALLOW(ALLOW) {                    \
	ALLOW(MSM_VIDC_DISALLOW)                  \
	ALLOW(MSM_VIDC_ALLOW)                     \
	ALLOW(MSM_VIDC_DEFER)                     \
	ALLOW(MSM_VIDC_DISCARD)                   \
	ALLOW(MSM_VIDC_IGNORE)                    \
}

enum msm_vidc_domain_type {
	MSM_VIDC_ENCODER           = BIT(0),
	MSM_VIDC_DECODER           = BIT(1),
};

enum msm_vidc_codec_type {
	MSM_VIDC_H264              = BIT(0),
	MSM_VIDC_HEVC              = BIT(1),
	MSM_VIDC_VP9               = BIT(2),
	MSM_VIDC_HEIC              = BIT(3),
	MSM_VIDC_AV1               = BIT(4),
};

enum msm_vidc_colorformat_type {
	MSM_VIDC_FMT_NONE          = 0,
	MSM_VIDC_FMT_NV12C         = BIT(0),
	MSM_VIDC_FMT_NV12          = BIT(1),
	MSM_VIDC_FMT_NV21          = BIT(2),
	MSM_VIDC_FMT_TP10C         = BIT(3),
	MSM_VIDC_FMT_P010          = BIT(4),
	MSM_VIDC_FMT_RGBA8888C     = BIT(5),
	MSM_VIDC_FMT_RGBA8888      = BIT(6),
	MSM_VIDC_FMT_META          = BIT(31),
};

enum msm_vidc_buffer_type FOREACH_BUF_TYPE(GENERATE_MSM_VIDC_BUF_ENUM);

/* always match with v4l2 flags V4L2_BUF_FLAG_* */
enum msm_vidc_buffer_flags {
	MSM_VIDC_BUF_FLAG_KEYFRAME         = 0x00000008,
	MSM_VIDC_BUF_FLAG_PFRAME           = 0x00000010,
	MSM_VIDC_BUF_FLAG_BFRAME           = 0x00000020,
	MSM_VIDC_BUF_FLAG_ERROR            = 0x00000040,
	MSM_VIDC_BUF_FLAG_LAST             = 0x00100000,
	/* codec config is a vendor specific flag */
	MSM_VIDC_BUF_FLAG_CODECCONFIG      = 0x01000000,
	/* sub frame is a vendor specific flag */
	MSM_VIDC_BUF_FLAG_SUBFRAME         = 0x02000000,
};

enum msm_vidc_buffer_attributes {
	MSM_VIDC_ATTR_DEFERRED                  = BIT(0),
	MSM_VIDC_ATTR_READ_ONLY                 = BIT(1),
	MSM_VIDC_ATTR_PENDING_RELEASE           = BIT(2),
	MSM_VIDC_ATTR_QUEUED                    = BIT(3),
	MSM_VIDC_ATTR_DEQUEUED                  = BIT(4),
	MSM_VIDC_ATTR_BUFFER_DONE               = BIT(5),
	MSM_VIDC_ATTR_RELEASE_ELIGIBLE          = BIT(6),
};

enum msm_vidc_buffer_region {
	MSM_VIDC_REGION_NONE = 0,
	MSM_VIDC_NON_SECURE,
	MSM_VIDC_NON_SECURE_PIXEL,
	MSM_VIDC_SECURE_PIXEL,
	MSM_VIDC_SECURE_NONPIXEL,
	MSM_VIDC_SECURE_BITSTREAM,
	MSM_VIDC_REGION_MAX,
};

enum msm_vidc_device_region {
	MSM_VIDC_DEVICE_REGION_NONE = 0,
	MSM_VIDC_AON,
	MSM_VIDC_PROTOCOL_FENCE_CLIENT_VPU,
	MSM_VIDC_QTIMER,
	MSM_VIDC_DEVICE_REGION_MAX,
};

enum msm_vidc_port_type {
	INPUT_PORT = 0,
	OUTPUT_PORT,
	INPUT_META_PORT,
	OUTPUT_META_PORT,
	PORT_NONE,
	MAX_PORT,
};

enum msm_vidc_stage_type {
	MSM_VIDC_STAGE_NONE = 0,
	MSM_VIDC_STAGE_1 = 1,
	MSM_VIDC_STAGE_2 = 2,
};

enum msm_vidc_pipe_type {
	MSM_VIDC_PIPE_NONE = 0,
	MSM_VIDC_PIPE_1 = 1,
	MSM_VIDC_PIPE_2 = 2,
	MSM_VIDC_PIPE_4 = 4,
};

enum msm_vidc_quality_mode {
	MSM_VIDC_MAX_QUALITY_MODE = 0x1,
	MSM_VIDC_POWER_SAVE_MODE = 0x2,
};

enum msm_vidc_color_primaries {
	MSM_VIDC_PRIMARIES_RESERVED                         = 0,
	MSM_VIDC_PRIMARIES_BT709                            = 1,
	MSM_VIDC_PRIMARIES_UNSPECIFIED                      = 2,
	MSM_VIDC_PRIMARIES_BT470_SYSTEM_M                   = 4,
	MSM_VIDC_PRIMARIES_BT470_SYSTEM_BG                  = 5,
	MSM_VIDC_PRIMARIES_BT601_525                        = 6,
	MSM_VIDC_PRIMARIES_SMPTE_ST240M                     = 7,
	MSM_VIDC_PRIMARIES_GENERIC_FILM                     = 8,
	MSM_VIDC_PRIMARIES_BT2020                           = 9,
	MSM_VIDC_PRIMARIES_SMPTE_ST428_1                    = 10,
	MSM_VIDC_PRIMARIES_SMPTE_RP431_2                    = 11,
	MSM_VIDC_PRIMARIES_SMPTE_EG431_1                    = 12,
	MSM_VIDC_PRIMARIES_SMPTE_EBU_TECH                   = 22,
};

enum msm_vidc_transfer_characteristics {
	MSM_VIDC_TRANSFER_RESERVED                          = 0,
	MSM_VIDC_TRANSFER_BT709                             = 1,
	MSM_VIDC_TRANSFER_UNSPECIFIED                       = 2,
	MSM_VIDC_TRANSFER_BT470_SYSTEM_M                    = 4,
	MSM_VIDC_TRANSFER_BT470_SYSTEM_BG                   = 5,
	MSM_VIDC_TRANSFER_BT601_525_OR_625                  = 6,
	MSM_VIDC_TRANSFER_SMPTE_ST240M                      = 7,
	MSM_VIDC_TRANSFER_LINEAR                            = 8,
	MSM_VIDC_TRANSFER_LOG_100_1                         = 9,
	MSM_VIDC_TRANSFER_LOG_SQRT                          = 10,
	MSM_VIDC_TRANSFER_XVYCC                             = 11,
	MSM_VIDC_TRANSFER_BT1361_0                          = 12,
	MSM_VIDC_TRANSFER_SRGB_SYCC                         = 13,
	MSM_VIDC_TRANSFER_BT2020_14                         = 14,
	MSM_VIDC_TRANSFER_BT2020_15                         = 15,
	MSM_VIDC_TRANSFER_SMPTE_ST2084_PQ                   = 16,
	MSM_VIDC_TRANSFER_SMPTE_ST428_1                     = 17,
	MSM_VIDC_TRANSFER_BT2100_2_HLG                      = 18,
};

enum msm_vidc_matrix_coefficients {
	MSM_VIDC_MATRIX_COEFF_SRGB_SMPTE_ST428_1             = 0,
	MSM_VIDC_MATRIX_COEFF_BT709                          = 1,
	MSM_VIDC_MATRIX_COEFF_UNSPECIFIED                    = 2,
	MSM_VIDC_MATRIX_COEFF_RESERVED                       = 3,
	MSM_VIDC_MATRIX_COEFF_FCC_TITLE_47                   = 4,
	MSM_VIDC_MATRIX_COEFF_BT470_SYS_BG_OR_BT601_625      = 5,
	MSM_VIDC_MATRIX_COEFF_BT601_525_BT1358_525_OR_625    = 6,
	MSM_VIDC_MATRIX_COEFF_SMPTE_ST240                    = 7,
	MSM_VIDC_MATRIX_COEFF_YCGCO                          = 8,
	MSM_VIDC_MATRIX_COEFF_BT2020_NON_CONSTANT            = 9,
	MSM_VIDC_MATRIX_COEFF_BT2020_CONSTANT                = 10,
	MSM_VIDC_MATRIX_COEFF_SMPTE_ST2085                   = 11,
	MSM_VIDC_MATRIX_COEFF_SMPTE_CHROM_DERV_NON_CONSTANT  = 12,
	MSM_VIDC_MATRIX_COEFF_SMPTE_CHROM_DERV_CONSTANT      = 13,
	MSM_VIDC_MATRIX_COEFF_BT2100                         = 14,
};

enum msm_vidc_preprocess_type {
	MSM_VIDC_PREPROCESS_NONE = BIT(0),
	MSM_VIDC_PREPROCESS_TYPE0 = BIT(1),
};

enum msm_vidc_core_capability_type {
	CORE_CAP_NONE = 0,
	ENC_CODECS,
	DEC_CODECS,
	MAX_SESSION_COUNT,
	MAX_NUM_720P_SESSIONS,
	MAX_NUM_1080P_SESSIONS,
	MAX_NUM_4K_SESSIONS,
	MAX_NUM_8K_SESSIONS,
	MAX_SECURE_SESSION_COUNT,
	MAX_LOAD,
	MAX_RT_MBPF,
	MAX_MBPF,
	MAX_MBPS,
	MAX_IMAGE_MBPF,
	MAX_MBPF_HQ,
	MAX_MBPS_HQ,
	MAX_MBPF_B_FRAME,
	MAX_MBPS_B_FRAME,
	MAX_MBPS_ALL_INTRA,
	MAX_ENH_LAYER_COUNT,
	NUM_VPP_PIPE,
	SW_PC,
	SW_PC_DELAY,
	FW_UNLOAD,
	FW_UNLOAD_DELAY,
	HW_RESPONSE_TIMEOUT,
	PREFIX_BUF_COUNT_PIX,
	PREFIX_BUF_SIZE_PIX,
	PREFIX_BUF_COUNT_NON_PIX,
	PREFIX_BUF_SIZE_NON_PIX,
	PAGEFAULT_NON_FATAL,
	PAGETABLE_CACHING,
	DCVS,
	DECODE_BATCH,
	DECODE_BATCH_TIMEOUT,
	STATS_TIMEOUT_MS,
	AV_SYNC_WINDOW_SIZE,
	CLK_FREQ_THRESHOLD,
	NON_FATAL_FAULTS,
	ENC_AUTO_FRAMERATE,
	DEVICE_CAPS,
	SUPPORTS_REQUESTS,
	SUPPORTS_SYNX_FENCE,
	CORE_CAP_MAX,
};

enum msm_vidc_inst_capability_type FOREACH_CAP(GENERATE_ENUM);

enum msm_vidc_inst_capability_flags {
	CAP_FLAG_NONE                    = 0,
	CAP_FLAG_DYNAMIC_ALLOWED         = BIT(0),
	CAP_FLAG_MENU                    = BIT(1),
	CAP_FLAG_INPUT_PORT              = BIT(2),
	CAP_FLAG_OUTPUT_PORT             = BIT(3),
	CAP_FLAG_CLIENT_SET              = BIT(4),
	CAP_FLAG_BITMASK                 = BIT(5),
	CAP_FLAG_VOLATILE                = BIT(6),
	CAP_FLAG_META                    = BIT(7),
};

struct msm_vidc_inst_cap {
	enum msm_vidc_inst_capability_type cap_id;
	s32 min;
	s32 max;
	u32 step_or_mask;
	s32 value;
	u32 v4l2_id;
	u32 hfi_id;
	enum msm_vidc_inst_capability_flags flags;
	enum msm_vidc_inst_capability_type children[MAX_CAP_CHILDREN];
	int (*adjust)(void *inst,
		      struct v4l2_ctrl *ctrl);
	int (*set)(void *inst,
		   enum msm_vidc_inst_capability_type cap_id);
};

struct msm_vidc_inst_capability {
	enum msm_vidc_domain_type domain;
	enum msm_vidc_codec_type codec;
	struct msm_vidc_inst_cap cap[INST_CAP_MAX + 1];
};

struct msm_vidc_core_capability {
	enum msm_vidc_core_capability_type type;
	u32 value;
};

struct msm_vidc_inst_cap_entry {
	/* list of struct msm_vidc_inst_cap_entry */
	struct list_head list;
	enum msm_vidc_inst_capability_type cap_id;
};

struct msm_vidc_event_data {
	union {
		bool                         bval;
		u32                          uval;
		u64                          uval64;
		s32                          val;
		s64                          val64;
		void                        *ptr;
	} edata;
};

struct debug_buf_count {
	u64 etb;
	u64 ftb;
	u64 fbd;
	u64 ebd;
};

struct msm_vidc_statistics {
	struct debug_buf_count             count;
	u64                                data_size;
	u64                                time_ms;
	u32                                avg_bw_llcc;
	u32                                avg_bw_ddr;
};

enum efuse_purpose {
	SKU_VERSION = 0,
};

enum sku_version {
	SKU_VERSION_0 = 0,
	SKU_VERSION_1,
	SKU_VERSION_2,
};

enum msm_vidc_ssr_trigger_type {
	SSR_ERR_FATAL = 1,
	SSR_SW_DIV_BY_ZERO,
	SSR_HW_WDOG_IRQ,
};

enum msm_vidc_stability_trigger_type {
	STABILITY_VCODEC_HUNG = 1,
	STABILITY_ENC_BUFFER_FULL,
};

enum msm_vidc_cache_op {
	MSM_VIDC_CACHE_CLEAN,
	MSM_VIDC_CACHE_INVALIDATE,
	MSM_VIDC_CACHE_CLEAN_INVALIDATE,
};

enum msm_vidc_dcvs_flags {
	MSM_VIDC_DCVS_INCR               = BIT(0),
	MSM_VIDC_DCVS_DECR               = BIT(1),
};

enum msm_vidc_clock_properties {
	CLOCK_PROP_HAS_SCALING           = BIT(0),
	CLOCK_PROP_HAS_MEM_RETENTION     = BIT(1),
};

enum profiling_points {
	FRAME_PROCESSING       = 0,
	MAX_PROFILING_POINTS,
};

enum signal_session_response {
	SIGNAL_CMD_STOP_INPUT = 0,
	SIGNAL_CMD_STOP_OUTPUT,
	SIGNAL_CMD_CLOSE,
	MAX_SIGNAL,
};

struct profile_data {
	u64                    start;
	u64                    stop;
	u64                    cumulative;
	char                   name[64];
	u32                    sampling;
	u64                    average;
};

struct msm_vidc_debug {
	struct profile_data    pdata[MAX_PROFILING_POINTS];
	u32                    profile;
	u32                    samples;
};

struct msm_vidc_input_cr_data {
	struct list_head       list;
	u32                    index;
	u32                    input_cr;
};

struct msm_vidc_session_idle {
	bool                   idle;
	u64                    last_activity_time_ns;
};

struct msm_vidc_color_info {
	u32 colorspace;
	u32 ycbcr_enc;
	u32 xfer_func;
	u32 quantization;
};

struct msm_vidc_rectangle {
	u32 left;
	u32 top;
	u32 width;
	u32 height;
};

struct msm_vidc_subscription_params {
	u32                    bitstream_resolution;
	u32                    crop_offsets[2];
	u32                    bit_depth;
	u32                    coded_frames;
	u32                    fw_min_count;
	u32                    pic_order_cnt;
	u32                    color_info;
	u32                    profile;
	u32                    level;
	u32                    tier;
	u32                    av1_film_grain_present;
	u32                    av1_super_block_enabled;
	u32                    max_num_reorder_frames;
};

struct msm_vidc_hfi_frame_info {
	u32                    picture_type;
	u32                    no_output;
	u32                    subframe_input;
	u32                    cr;
	u32                    cf;
	u32                    data_corrupt;
	u32                    overflow;
	u32                    fence_id;
	u32                    fence_error;
	u32                    av1_tile_rows_columns;
	bool                   av1_non_uniform_tile_spacing;
};

struct msm_vidc_decode_vpp_delay {
	bool                   enable;
	u32                    size;
};

struct msm_vidc_decode_batch {
	bool                   enable;
	u32                    size;
	struct delayed_work    work;
};

enum msm_vidc_power_mode {
	VIDC_POWER_NORMAL = 0,
	VIDC_POWER_LOW,
	VIDC_POWER_TURBO,
};

struct vidc_bus_vote_data {
	enum msm_vidc_domain_type domain;
	enum msm_vidc_codec_type codec;
	enum msm_vidc_power_mode power_mode;
	u32 color_formats[2];
	int num_formats; /* 1 = DPB-OPB unified; 2 = split */
	int input_height, input_width, bitrate;
	int output_height, output_width;
	int rotation;
	int compression_ratio;
	int complexity_factor;
	int input_cr;
	u32 lcu_size;
	u32 fps;
	u32 work_mode;
	bool use_sys_cache;
	bool b_frames_enabled;
	u64 calc_bw_ddr;
	u64 calc_bw_llcc;
	u32 num_vpp_pipes;
	bool vpss_preprocessing_enabled;
};

struct msm_vidc_power {
	enum msm_vidc_power_mode power_mode;
	u32                    buffer_counter;
	u32                    min_threshold;
	u32                    nom_threshold;
	u32                    max_threshold;
	bool                   dcvs_mode;
	u32                    dcvs_window;
	u64                    min_freq;
	u64                    curr_freq;
	u32                    ddr_bw;
	u32                    sys_cache_bw;
	u32                    dcvs_flags;
	u32                    fw_cr;
	u32                    fw_cf;
	u32                    fw_av1_tile_rows;
	u32                    fw_av1_tile_columns;
};

enum msm_vidc_fence_type {
	MSM_VIDC_FENCE_NONE         = 0,
	MSM_VIDC_SW_FENCE           = 1,
	MSM_VIDC_SYNX_V2_FENCE      = 2,
};

enum msm_vidc_fence_direction {
	MSM_VIDC_FENCE_DIR_NONE    = 0,
	MSM_VIDC_FENCE_DIR_TX      = 1,
	MSM_VIDC_FENCE_DIR_RX      = 2,
};

struct msm_vidc_fence_context {
	char                      name[MAX_NAME_LENGTH];
	u64                       ctx_num;
	u64                       seq_num;
};

struct msm_vidc_fence {
	struct list_head            list;
	struct dma_fence            dma_fence;
	char                        name[MAX_NAME_LENGTH];
	spinlock_t                  lock;
	struct sync_file            *sync_file;
	int                         fd;
	u64                         fence_id;
	void                        *session;
};

struct msm_vidc_mem {
	struct list_head            list;
	enum msm_vidc_buffer_type   type;
	enum msm_vidc_buffer_region region;
	u32                         size;
	u8                          secure:1;
	u8                          map_kernel:1;
	struct dma_buf             *dmabuf;
	/*
	 * Kalama uses Kernel Version 5.15.x,
	 * Pineapple uses Kernel version 5.18.x
	 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	struct iosys_map            dmabuf_map;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct dma_buf_map          dmabuf_map;
#endif
	void                       *kvaddr;
	dma_addr_t                  device_addr;
	unsigned long               attrs;
	u32                         refcount;
	struct sg_table            *table;
	struct dma_buf_attachment  *attach;
	phys_addr_t                 phys_addr;
	enum dma_data_direction     direction;
};

struct msm_vidc_mem_list {
	struct list_head            list; // list of "struct msm_vidc_mem"
};

struct msm_vidc_buffer {
	struct list_head                   list;
	struct msm_vidc_inst              *inst;
	enum msm_vidc_buffer_type          type;
	enum msm_vidc_buffer_region        region;
	u32                                index;
	int                                fd;
	u32                                buffer_size;
	u32                                data_offset;
	u32                                data_size;
	u64                                device_addr;
	u32                                flags;
	u64                                timestamp;
	enum msm_vidc_buffer_attributes    attr;
	void                              *dmabuf;
	struct sg_table                   *sg_table;
	struct dma_buf_attachment         *attach;
	u32                                dbuf_get:1;
	u64                                fence_id;
	u32                                start_time_ms;
	u32                                end_time_ms;
};

struct msm_vidc_buffers {
	struct list_head       list; // list of "struct msm_vidc_buffer"
	u32                    min_count;
	u32                    extra_count;
	u32                    actual_count;
	u32                    size;
	bool                   reuse;
};

struct msm_vidc_buffer_stats {
	struct list_head                   list;
	u32                                frame_num;
	u64                                timestamp;
	u32                                etb_time_ms;
	u32                                ebd_time_ms;
	u32                                ftb_time_ms;
	u32                                fbd_time_ms;
	u32                                data_size;
	u32                                flags;
	u32                                ts_offset;
};

enum msm_vidc_buffer_stats_flag {
	MSM_VIDC_STATS_FLAG_CORRUPT        = BIT(0),
	MSM_VIDC_STATS_FLAG_OVERFLOW       = BIT(1),
	MSM_VIDC_STATS_FLAG_NO_OUTPUT      = BIT(2),
	MSM_VIDC_STATS_FLAG_SUBFRAME_INPUT = BIT(3),
};

struct msm_vidc_sort {
	struct list_head       list;
	s64                    val;
};

struct msm_vidc_timestamp {
	struct msm_vidc_sort   sort;
	u64                    rank;
};

struct msm_vidc_timestamps {
	struct list_head       list;
	u32                    count;
	u64                    rank;
};

struct msm_vidc_input_timer {
	struct list_head       list;
	u64                    time_us;
};

enum msm_vidc_allow FOREACH_ALLOW(GENERATE_ENUM);

struct msm_vidc_ssr {
	enum msm_vidc_ssr_trigger_type     ssr_type;
	u32                                sub_client_id;
	u32                                test_addr;
};

struct msm_vidc_stability {
	enum msm_vidc_stability_trigger_type     stability_type;
	u32                                      sub_client_id;
	u32                                      value;
};

struct msm_vidc_sfr {
	u32 buf_size;
	u8 rg_data[1];
};

struct msm_vidc_ctrl_data {
	bool skip_s_ctrl;
};

#endif // _MSM_VIDC_INTERNAL_H_
