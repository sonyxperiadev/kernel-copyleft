/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __ADRENO_GEN8_0_0_SNAPSHOT_H
#define __ADRENO_GEN8_0_0_SNAPSHOT_H

#include "adreno_gen8_snapshot.h"

static struct gen8_shader_block gen8_0_0_shader_blocks[] = {
	{ TP0_TMO_DATA, 0x0200, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ TP0_SMO_DATA, 0x0080, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ TP0_MIPMAP_BASE_DATA, 0x0080, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_3, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_1, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_0_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_1_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_2_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_3_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_4_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_5_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_6_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_7_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_CB_RAM, 0x0390, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_13_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_14_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_TAG, 0x0100, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_2, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_TMO_TAG, 0x0080, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_SMO_TAG, 0x0080, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_STATE_DATA, 0x0040, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_HWAVE_RAM, 0x0200, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_L0_INST_BUF, 0x0080, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_8_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_9_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_10_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_11_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_12_DATA, 0x0800, 2, 1, PIPE_BR, USPTP, SLICE, 1},
	{ HLSQ_DATAPATH_DSTR_META, 0x0600, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_DATAPATH_DSTR_META, 0x0020, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_DESC_REMAP_META, 0x00A0, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_DESC_REMAP_META, 0x000C, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_DESC_REMAP_META, 0x0008, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_SLICE_TOP_META, 0x0048, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_SLICE_TOP_META, 0x0048, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_SLICE_TOP_META, 0x0020, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_L2STC_TAG_RAM, 0x0200, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_L2STC_INFO_CMD, 0x0474, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CVS_BE_CTXT_BUF_RAM_TAG, 0x0100, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CVS_BE_CTXT_BUF_RAM_TAG, 0x0100, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CPS_BE_CTXT_BUF_RAM_TAG, 0x0100, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CVS_BE_CTXT_BUF_RAM, 0x0400, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CVS_BE_CTXT_BUF_RAM, 0x0400, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CPS_BE_CTXT_BUF_RAM, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CVS_RAM, 0x01C0, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CVS_RAM, 0x01C0, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CPS_RAM, 0x0300, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CPS_RAM, 0x0180, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CVS_RAM_TAG, 0x0040, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CVS_RAM_TAG, 0x0040, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CPS_RAM_TAG, 0x0040, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CPS_RAM_TAG, 0x0040, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_ICB_CVS_CB_BASE_TAG, 0x0010, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_ICB_CVS_CB_BASE_TAG, 0x0010, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_ICB_CPS_CB_BASE_TAG, 0x0010, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_ICB_CPS_CB_BASE_TAG, 0x0010, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CVS_MISC_RAM, 0x0540, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CVS_MISC_RAM, 0x0540, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CPS_MISC_RAM, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CPS_MISC_RAM, 0x00B0, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CPS_MISC_RAM_1, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM, 0x0800, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM, 0x0200, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CVS_CONST_RAM, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CVS_CONST_RAM, 0x0800, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CPS_CONST_RAM, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CPS_CONST_RAM, 0x0800, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CVS_MISC_RAM_TAG, 0x0060, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CVS_MISC_RAM_TAG, 0x0060, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CPS_MISC_RAM_TAG, 0x0600, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CPS_MISC_RAM_TAG, 0x0012, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM_TAG, 0x0040, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM_TAG, 0x0010, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM_TAG, 0x0004, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CVS_CONST_RAM_TAG, 0x0040, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CVS_CONST_RAM_TAG, 0x0040, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CPS_CONST_RAM_TAG, 0x0060, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_CPS_CONST_RAM_TAG, 0x0020, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_LOCAL_MISC_RAM, 0x03C0, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_LOCAL_MISC_RAM, 0x0280, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_LOCAL_MISC_RAM, 0x0050, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_LOCAL_MISC_RAM_TAG, 0x0014, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_GFX_LOCAL_MISC_RAM_TAG, 0x000C, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM_1, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_STPROC_META, 0x0400, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_SLICE_BACKEND_META, 0x0188, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_SLICE_BACKEND_META, 0x0188, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_SLICE_BACKEND_META, 0x00C0, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INST_RAM_2, 0x0800, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_DATAPATH_META, 0x0020, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_FRONTEND_META, 0x0080, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_FRONTEND_META, 0x0080, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_FRONTEND_META, 0x0080, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_INDIRECT_META, 0x0010, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_BACKEND_META, 0x0034, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_BACKEND_META, 0x0034, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_BACKEND_META, 0x0020, 1, 1, PIPE_LPAC, HLSQ_STATE, UNSLICE, 1},
};

/*
 * Block   : ['AHB_PRECD']
 * pairs   : 1 (Regs:2)
 */
static const u32 gen8_0_0_ahb_precd_gpu_registers[] = {
	 0x00012, 0x00013,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_ahb_precd_gpu_registers), 8));

/*
 * Block   : ['AHB_PRECD']
 * REGION  : SLICE
 * pairs   : 1 (Regs:3)
 */
static const u32 gen8_0_0_ahb_precd_gpu_slice_slice_registers[] = {
	 0x00580, 0x00582,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_ahb_precd_gpu_slice_slice_registers), 8));

/*
 * Block   : ['AHB_SECURE']
 * pairs   : 3 (Regs:7)
 */
static const u32 gen8_0_0_ahb_secure_gpu_registers[] = {
	 0x0f400, 0x0f400, 0x0f800, 0x0f803, 0x0fc00, 0x0fc01,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_ahb_secure_gpu_registers), 8));

/*
 * Block   : ['GBIF']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 8 (Regs:59)
 */
static const u32 gen8_0_0_gbif_registers[] = {
	 0x03c00, 0x03c0b, 0x03c40, 0x03c42, 0x03c45, 0x03c47, 0x03c49, 0x03c4e,
	 0x03c50, 0x03c57, 0x03cc0, 0x03cc4, 0x03cc6, 0x03cd5, 0x03ce0, 0x03ce5,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gbif_registers), 8));

/*
 * Block   : ['BROADCAST', 'GRAS', 'PC']
 * Block   : ['RBBM', 'RDVM', 'UCHE']
 * Block   : ['VFD', 'VPC', 'VSC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 120 (Regs:1161)
 */
static const u32 gen8_0_0_gpu_registers[] = {
	 0x00000, 0x00000, 0x00002, 0x00002, 0x00008, 0x0000d, 0x00010, 0x00013,
	 0x00015, 0x00016, 0x00018, 0x00018, 0x0001a, 0x0001a, 0x0001c, 0x0001c,
	 0x00028, 0x0002b, 0x0002d, 0x00039, 0x00040, 0x00053, 0x00062, 0x00066,
	 0x00069, 0x0006e, 0x00071, 0x00072, 0x00074, 0x00074, 0x00076, 0x0007c,
	 0x0007f, 0x0009a, 0x0009d, 0x000af, 0x000b2, 0x000d4, 0x000d7, 0x000e2,
	 0x000e5, 0x000e6, 0x000e9, 0x000f1, 0x000f4, 0x000f6, 0x000f9, 0x00108,
	 0x0010b, 0x0010e, 0x00111, 0x00111, 0x00114, 0x0011c, 0x0011f, 0x00121,
	 0x00125, 0x00125, 0x00127, 0x00127, 0x00129, 0x00129, 0x0012b, 0x00131,
	 0x00134, 0x00138, 0x0013a, 0x0013a, 0x0013c, 0x0013f, 0x00142, 0x00150,
	 0x00153, 0x00155, 0x00158, 0x00159, 0x0015c, 0x0015c, 0x00166, 0x00179,
	 0x0019e, 0x001a3, 0x001b0, 0x002c9, 0x002e2, 0x0036b, 0x00380, 0x0039b,
	 0x003a4, 0x003ab, 0x003b4, 0x003c5, 0x003ce, 0x003cf, 0x003e0, 0x003e0,
	 0x003f0, 0x003f0, 0x00440, 0x00444, 0x00460, 0x00460, 0x00c02, 0x00c04,
	 0x00c06, 0x00c06, 0x00c10, 0x00cd9, 0x00ce0, 0x00d0c, 0x00df0, 0x00df4,
	 0x00e01, 0x00e04, 0x00e06, 0x00e09, 0x00e0e, 0x00e13, 0x00e15, 0x00e16,
	 0x00e20, 0x00e37, 0x0ec00, 0x0ec01, 0x0ec05, 0x0ec05, 0x0ec07, 0x0ec07,
	 0x0ec0a, 0x0ec0a, 0x0ec12, 0x0ec12, 0x0ec26, 0x0ec28, 0x0ec2b, 0x0ec2d,
	 0x0ec2f, 0x0ec2f, 0x0ec40, 0x0ec41, 0x0ec45, 0x0ec45, 0x0ec47, 0x0ec47,
	 0x0ec4a, 0x0ec4a, 0x0ec52, 0x0ec52, 0x0ec66, 0x0ec68, 0x0ec6b, 0x0ec6d,
	 0x0ec6f, 0x0ec6f, 0x0ec80, 0x0ec81, 0x0ec85, 0x0ec85, 0x0ec87, 0x0ec87,
	 0x0ec8a, 0x0ec8a, 0x0ec92, 0x0ec92, 0x0eca6, 0x0eca8, 0x0ecab, 0x0ecad,
	 0x0ecaf, 0x0ecaf, 0x0ecc0, 0x0ecc1, 0x0ecc5, 0x0ecc5, 0x0ecc7, 0x0ecc7,
	 0x0ecca, 0x0ecca, 0x0ecd2, 0x0ecd2, 0x0ece6, 0x0ece8, 0x0eceb, 0x0eced,
	 0x0ecef, 0x0ecef, 0x0ed00, 0x0ed01, 0x0ed05, 0x0ed05, 0x0ed07, 0x0ed07,
	 0x0ed0a, 0x0ed0a, 0x0ed12, 0x0ed12, 0x0ed26, 0x0ed28, 0x0ed2b, 0x0ed2d,
	 0x0ed2f, 0x0ed2f, 0x0ed40, 0x0ed41, 0x0ed45, 0x0ed45, 0x0ed47, 0x0ed47,
	 0x0ed4a, 0x0ed4a, 0x0ed52, 0x0ed52, 0x0ed66, 0x0ed68, 0x0ed6b, 0x0ed6d,
	 0x0ed6f, 0x0ed6f, 0x0ed80, 0x0ed81, 0x0ed85, 0x0ed85, 0x0ed87, 0x0ed87,
	 0x0ed8a, 0x0ed8a, 0x0ed92, 0x0ed92, 0x0eda6, 0x0eda8, 0x0edab, 0x0edad,
	 0x0edaf, 0x0edaf, UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gpu_registers), 8));

/*
 * Block   : ['BROADCAST', 'GRAS', 'PC']
 * Block   : ['RBBM', 'RDVM', 'UCHE']
 * Block   : ['VFD', 'VPC', 'VSC']
 * REGION  : SLICE
 * Pipeline: PIPE_NONE
 * pairs   : 12 (Regs:89)
 */
static const u32 gen8_0_0_gpu_slice_registers[] = {
	 0x00500, 0x00500, 0x00580, 0x00584, 0x00586, 0x0058b, 0x0058f, 0x00599,
	 0x005a0, 0x005b3, 0x005c0, 0x005c0, 0x005c2, 0x005c6, 0x005e0, 0x005e3,
	 0x005ec, 0x005ec, 0x00f01, 0x00f02, 0x00f04, 0x00f0c, 0x00f20, 0x00f37,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gpu_slice_registers), 8));

/*
 * Block   : ['GMUAO', 'GMUCX', 'GMUCX_RAM']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 161 (Regs:614)
 */
static const u32 gen8_0_0_gmu_registers[] = {
	0x10001, 0x10001, 0x10003, 0x10003, 0x10401, 0x10401, 0x10403, 0x10403,
	0x10801, 0x10801, 0x10803, 0x10803, 0x10c01, 0x10c01, 0x10c03, 0x10c03,
	0x11001, 0x11001, 0x11003, 0x11003, 0x11401, 0x11401, 0x11403, 0x11403,
	0x11801, 0x11801, 0x11803, 0x11803, 0x11c01, 0x11c01, 0x11c03, 0x11c03,
	0x1f400, 0x1f40b, 0x1f40f, 0x1f411, 0x1f500, 0x1f500, 0x1f507, 0x1f507,
	0x1f509, 0x1f50b, 0x1f700, 0x1f701, 0x1f704, 0x1f706, 0x1f708, 0x1f709,
	0x1f70c, 0x1f70d, 0x1f710, 0x1f711, 0x1f713, 0x1f716, 0x1f718, 0x1f71d,
	0x1f720, 0x1f725, 0x1f729, 0x1f729, 0x1f730, 0x1f747, 0x1f750, 0x1f75a,
	0x1f75c, 0x1f75c, 0x1f780, 0x1f781, 0x1f784, 0x1f78b, 0x1f790, 0x1f797,
	0x1f7a0, 0x1f7a7, 0x1f7b0, 0x1f7b7, 0x1f7e0, 0x1f7e1, 0x1f7e4, 0x1f7e5,
	0x1f7e8, 0x1f7e9, 0x1f7ec, 0x1f7ed, 0x1f800, 0x1f804, 0x1f807, 0x1f808,
	0x1f80b, 0x1f80c, 0x1f80f, 0x1f80f, 0x1f811, 0x1f811, 0x1f813, 0x1f817,
	0x1f819, 0x1f81c, 0x1f824, 0x1f830, 0x1f840, 0x1f842, 0x1f848, 0x1f848,
	0x1f84c, 0x1f84c, 0x1f850, 0x1f850, 0x1f858, 0x1f859, 0x1f868, 0x1f869,
	0x1f878, 0x1f883, 0x1f930, 0x1f931, 0x1f934, 0x1f935, 0x1f938, 0x1f939,
	0x1f93c, 0x1f93d, 0x1f940, 0x1f941, 0x1f943, 0x1f943, 0x1f948, 0x1f94a,
	0x1f94f, 0x1f951, 0x1f954, 0x1f955, 0x1f95d, 0x1f95d, 0x1f962, 0x1f96b,
	0x1f970, 0x1f970, 0x1f97c, 0x1f97e, 0x1f980, 0x1f981, 0x1f984, 0x1f986,
	0x1f992, 0x1f993, 0x1f996, 0x1f99e, 0x1f9c0, 0x1f9cf, 0x1f9f0, 0x1f9f1,
	0x1f9f8, 0x1f9fa, 0x1f9fc, 0x1f9fc, 0x1fa00, 0x1fa03, 0x1fc00, 0x1fc01,
	0x1fc04, 0x1fc07, 0x1fc10, 0x1fc10, 0x1fc14, 0x1fc14, 0x1fc18, 0x1fc19,
	0x1fc20, 0x1fc20, 0x1fc24, 0x1fc26, 0x1fc30, 0x1fc33, 0x1fc38, 0x1fc3b,
	0x1fc40, 0x1fc49, 0x1fc50, 0x1fc59, 0x1fc60, 0x1fc7f, 0x1fca0, 0x1fcef,
	0x20000, 0x20007, 0x20010, 0x20015, 0x20018, 0x2001a, 0x2001c, 0x2001d,
	0x20020, 0x20021, 0x20024, 0x20025, 0x2002a, 0x2002c, 0x20030, 0x20031,
	0x20034, 0x20036, 0x20080, 0x20087, 0x20300, 0x20301, 0x20304, 0x20305,
	0x20308, 0x2030c, 0x20310, 0x20314, 0x20318, 0x2031a, 0x20320, 0x20322,
	0x20324, 0x20326, 0x20328, 0x2032a, 0x20330, 0x20333, 0x20338, 0x20338,
	0x20340, 0x20345, 0x20348, 0x20350, 0x20354, 0x2035b, 0x20360, 0x20367,
	0x20370, 0x20377, 0x23801, 0x23801, 0x23803, 0x23803, 0x23805, 0x23805,
	0x23807, 0x23807, 0x23809, 0x23809, 0x2380b, 0x2380b, 0x2380d, 0x2380d,
	0x2380f, 0x2380f, 0x23811, 0x23811, 0x23813, 0x23813, 0x23815, 0x23815,
	0x23817, 0x23817, 0x23819, 0x23819, 0x2381b, 0x2381b, 0x2381d, 0x2381d,
	0x2381f, 0x23820, 0x23822, 0x23822, 0x23824, 0x23824, 0x23826, 0x23826,
	0x23828, 0x23828, 0x2382a, 0x2382a, 0x2382c, 0x2382c, 0x2382e, 0x2382e,
	0x23830, 0x23830, 0x23832, 0x23832, 0x23834, 0x23834, 0x23836, 0x23836,
	0x23838, 0x23838, 0x2383a, 0x2383a, 0x2383c, 0x2383c, 0x2383e, 0x2383e,
	0x23840, 0x23847, 0x23b00, 0x23b01, 0x23b03, 0x23b03, 0x23b05, 0x23b0e,
	0x23b10, 0x23b13, 0x23b15, 0x23b16, 0x23b28, 0x23b28, 0x23b30, 0x23b30,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gmu_registers), 8));

/*
 * Block   : ['GMUGX']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 15 (Regs:174)
 */
static const u32 gen8_0_0_gmugx_registers[] = {
	0x0dc00, 0x0dc0d, 0x0dc10, 0x0dc11, 0x0dc13, 0x0dc15, 0x0dc18, 0x0dc1a,
	0x0dc1c, 0x0dc2f, 0x0dc40, 0x0dc42, 0x0dc60, 0x0dc7f, 0x0dc88, 0x0dc90,
	0x0dc98, 0x0dc99, 0x0dca0, 0x0dcbf, 0x0dcc8, 0x0dcd0, 0x0dcd8, 0x0dcd9,
	0x0dce0, 0x0dcff, 0x0dd08, 0x0dd10, 0x0dd18, 0x0dd19,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gmugx_registers), 8));

/*
 * Block   : ['GMUGX']
 * REGION  : SLICE
 * Pipeline: PIPE_NONE
 * pairs   : 23 (Regs:312)
 */
static const u32 gen8_0_0_gmugx_slice_registers[] = {
	0x0e400, 0x0e401, 0x0e404, 0x0e40a, 0x0e40e, 0x0e42f, 0x0e438, 0x0e440,
	0x0e448, 0x0e449, 0x0e450, 0x0e46f, 0x0e478, 0x0e480, 0x0e488, 0x0e489,
	0x0e490, 0x0e4af, 0x0e4b8, 0x0e4c0, 0x0e4c8, 0x0e4c9, 0x0e4d0, 0x0e4ef,
	0x0e4f8, 0x0e500, 0x0e508, 0x0e509, 0x0e510, 0x0e52f, 0x0e538, 0x0e540,
	0x0e548, 0x0e549, 0x0e550, 0x0e56f, 0x0e578, 0x0e580, 0x0e588, 0x0e589,
	0x0e590, 0x0e5af, 0x0e5b8, 0x0e5c0, 0x0e5c8, 0x0e5c9,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gmugx_slice_registers), 8));

/*
 * Block   : ['CX_MISC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 7 (Regs:58)
 */
static const u32 gen8_0_0_cx_misc_registers[] = {
	 0x27800, 0x27800, 0x27810, 0x27814, 0x27820, 0x27824, 0x27828, 0x2782a,
	 0x27832, 0x27857, 0x27880, 0x27883, 0x27c00, 0x27c01,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cx_misc_registers), 8));

/*
 * Block   : ['DBGC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 21 (Regs:152)
 */
static const u32 gen8_0_0_dbgc_registers[] = {
	 0x00600, 0x0061c, 0x0061e, 0x0063d, 0x00640, 0x00644, 0x00650, 0x00655,
	 0x00660, 0x00660, 0x00662, 0x00668, 0x0066a, 0x0066a, 0x00680, 0x00685,
	 0x00700, 0x00704, 0x00707, 0x0070a, 0x0070f, 0x00716, 0x00720, 0x00724,
	 0x00730, 0x00732, 0x00740, 0x00740, 0x00742, 0x0074a, 0x00750, 0x00755,
	 0x00759, 0x0075c, 0x00760, 0x00763, 0x00770, 0x00770, 0x00780, 0x0078d,
	 0x00790, 0x00790,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_dbgc_registers), 8));

/*
 * Block   : ['DBGC']
 * REGION  : SLICE
 * Pipeline: PIPE_NONE
 * pairs   : 2 (Regs:61)
 */
static const u32 gen8_0_0_dbgc_slice_registers[] = {
	 0x007a0, 0x007d5, 0x007e0, 0x007e6,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_dbgc_slice_registers), 8));

/*
 * Block   : ['CX_DBGC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 6 (Regs:75)
 */
static const u32 gen8_0_0_cx_dbgc_registers[] = {
	 0x18400, 0x1841c, 0x1841e, 0x1843d, 0x18440, 0x18444, 0x18450, 0x18455,
	 0x1846a, 0x1846a, 0x18580, 0x18581,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cx_dbgc_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * Cluster : CLUSTER_NONE
 * pairs   : 15 (Regs:310)
 */
static const u32 gen8_0_0_cp_cp_pipe_none_registers[] = {
	 0x00800, 0x0081e, 0x00820, 0x0082d, 0x00838, 0x0083e, 0x00840, 0x00847,
	 0x0084b, 0x0084c, 0x00850, 0x0088f, 0x008b5, 0x008b6, 0x008c0, 0x008cb,
	 0x008d0, 0x008e4, 0x008e7, 0x008ee, 0x008fa, 0x008fd, 0x00928, 0x00929,
	 0x00958, 0x0095b, 0x00980, 0x009ff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_none_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 8 (Regs:96)
 */
static const u32 gen8_0_0_cp_cp_pipe_br_registers[] = {
	 0x00830, 0x00837, 0x0084d, 0x0084f, 0x008a0, 0x008b4, 0x008b7, 0x008bb,
	 0x008f0, 0x008f9, 0x00900, 0x0091e, 0x00920, 0x00926, 0x00930, 0x0093a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_br_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 4 (Regs:23)
 */
static const u32 gen8_0_0_cp_slice_cp_pipe_br_registers[] = {
	 0x00b00, 0x00b0c, 0x00b10, 0x00b10, 0x00b80, 0x00b84, 0x00b90, 0x00b93,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_slice_cp_pipe_br_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_NONE
 * pairs   : 9 (Regs:72)
 */
static const u32 gen8_0_0_cp_cp_pipe_bv_registers[] = {
	 0x00830, 0x00835, 0x0084d, 0x0084f, 0x008b0, 0x008b4, 0x008b7, 0x008bb,
	 0x008f0, 0x008f9, 0x00900, 0x00913, 0x00918, 0x0091d, 0x00920, 0x00925,
	 0x00930, 0x0093a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_bv_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_NONE
 * pairs   : 4 (Regs:23)
 */
static const u32 gen8_0_0_cp_slice_cp_pipe_bv_registers[] = {
	 0x00b00, 0x00b0c, 0x00b10, 0x00b10, 0x00b80, 0x00b84, 0x00b90, 0x00b93,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_slice_cp_pipe_bv_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_NONE
 * pairs   : 9 (Regs:77)
 */
static const u32 gen8_0_0_cp_cp_pipe_lpac_registers[] = {
	 0x00830, 0x00837, 0x0084d, 0x0084f, 0x008a0, 0x008b4, 0x008b7, 0x008bb,
	 0x008f0, 0x008f5, 0x008f8, 0x008f9, 0x00900, 0x00913, 0x00918, 0x0091d,
	 0x00920, 0x00925,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_lpac_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_AQE0
 * Cluster : CLUSTER_NONE
 * pairs   : 10 (Regs:22)
 */
static const u32 gen8_0_0_cp_cp_pipe_aqe0_registers[] = {
	 0x0084d, 0x0084d, 0x008b0, 0x008b4, 0x008b7, 0x008b9, 0x008bb, 0x008bb,
	 0x008f0, 0x008f1, 0x008f4, 0x008f5, 0x008f8, 0x008f9, 0x00910, 0x00913,
	 0x0091d, 0x0091d, 0x00925, 0x00925,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_aqe0_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_AQE1
 * Cluster : CLUSTER_NONE
 * pairs   : 10 (Regs:22)
 */
static const u32 gen8_0_0_cp_cp_pipe_aqe1_registers[] = {
	 0x0084d, 0x0084d, 0x008b0, 0x008b4, 0x008b7, 0x008b9, 0x008bb, 0x008bb,
	 0x008f0, 0x008f1, 0x008f4, 0x008f5, 0x008f8, 0x008f9, 0x00910, 0x00913,
	 0x0091d, 0x0091d, 0x00925, 0x00925,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_aqe1_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_DDE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 13 (Regs:30)
 */
static const u32 gen8_0_0_cp_cp_pipe_dde_br_registers[] = {
	 0x0084d, 0x0084d, 0x008b0, 0x008b4, 0x008b7, 0x008b9, 0x008bb, 0x008bb,
	 0x008f0, 0x008f1, 0x008f4, 0x008f5, 0x008f8, 0x008f9, 0x008fe, 0x008ff,
	 0x00910, 0x00913, 0x00918, 0x00918, 0x0091c, 0x0091e, 0x00920, 0x00921,
	 0x00924, 0x00925,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_dde_br_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_DDE_BV
 * Cluster : CLUSTER_NONE
 * pairs   : 13 (Regs:29)
 */
static const u32 gen8_0_0_cp_cp_pipe_dde_bv_registers[] = {
	 0x0084d, 0x0084d, 0x008b0, 0x008b4, 0x008b7, 0x008b9, 0x008bb, 0x008bb,
	 0x008f0, 0x008f1, 0x008f4, 0x008f5, 0x008f8, 0x008f9, 0x008fe, 0x008ff,
	 0x00910, 0x00913, 0x00918, 0x00918, 0x0091c, 0x0091d, 0x00920, 0x00921,
	 0x00924, 0x00925,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cp_cp_pipe_dde_bv_registers), 8));

/*
 * Block   : ['BROADCAST', 'CX_DBGC', 'CX_MISC', 'DBGC', 'GBIF', 'GMUAO']
 * Block   : ['GMUCX', 'GMUGX', 'GRAS', 'PC', 'RBBM']
 * Block   : ['RDVM', 'UCHE', 'VFD', 'VPC', 'VSC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 11 (Regs:1124)
 */
static const u32 gen8_0_0_non_context_pipe_br_registers[] = {
	 0x09600, 0x09605, 0x09610, 0x09617, 0x09620, 0x09627, 0x09670, 0x0967b,
	 0x09e00, 0x09e04, 0x09e06, 0x09e15, 0x09e17, 0x09e23, 0x09e30, 0x09e3f,
	 0x09e50, 0x09e59, 0x09e60, 0x09e65, 0x0d200, 0x0d5ff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_pipe_br_registers), 8));

/*
 * Block   : ['BROADCAST', 'CX_DBGC', 'CX_MISC', 'DBGC', 'GBIF', 'GMUAO']
 * Block   : ['GMUCX', 'GMUGX', 'GRAS', 'PC', 'RBBM']
 * Block   : ['RDVM', 'UCHE', 'VFD', 'VPC', 'VSC']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 21 (Regs:300)
 */
static const u32 gen8_0_0_non_context_slice_pipe_br_registers[] = {
	 0x08600, 0x08602, 0x08610, 0x08613, 0x08700, 0x08703, 0x08710, 0x08713,
	 0x08720, 0x08723, 0x08730, 0x08733, 0x08740, 0x08744, 0x09680, 0x09681,
	 0x09690, 0x0969b, 0x09740, 0x09745, 0x09750, 0x0975b, 0x09770, 0x097ef,
	 0x09f00, 0x09f0f, 0x09f20, 0x09f23, 0x09f30, 0x09f31, 0x0a600, 0x0a600,
	 0x0a603, 0x0a603, 0x0a610, 0x0a61f, 0x0a630, 0x0a632, 0x0a638, 0x0a63c,
	 0x0a640, 0x0a67f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_slice_pipe_br_registers), 8));

/*
 * Block   : ['BROADCAST', 'CX_DBGC', 'CX_MISC', 'DBGC', 'GBIF', 'GMUAO']
 * Block   : ['GMUCX', 'GMUGX', 'GRAS', 'PC', 'RBBM']
 * Block   : ['RDVM', 'UCHE', 'VFD', 'VPC', 'VSC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_NONE
 * pairs   : 11 (Regs:1124)
 */
static const u32 gen8_0_0_non_context_pipe_bv_registers[] = {
	 0x09600, 0x09605, 0x09610, 0x09617, 0x09620, 0x09627, 0x09670, 0x0967b,
	 0x09e00, 0x09e04, 0x09e06, 0x09e15, 0x09e17, 0x09e23, 0x09e30, 0x09e3f,
	 0x09e50, 0x09e59, 0x09e60, 0x09e65, 0x0d200, 0x0d5ff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_pipe_bv_registers), 8));

/*
 * Block   : ['BROADCAST', 'CX_DBGC', 'CX_MISC', 'DBGC', 'GBIF', 'GMUAO']
 * Block   : ['GMUCX', 'GMUGX', 'GRAS', 'PC', 'RBBM']
 * Block   : ['RDVM', 'UCHE', 'VFD', 'VPC', 'VSC']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_NONE
 * pairs   : 21 (Regs:300)
 */
static const u32 gen8_0_0_non_context_slice_pipe_bv_registers[] = {
	 0x08600, 0x08602, 0x08610, 0x08613, 0x08700, 0x08703, 0x08710, 0x08713,
	 0x08720, 0x08723, 0x08730, 0x08733, 0x08740, 0x08744, 0x09680, 0x09681,
	 0x09690, 0x0969b, 0x09740, 0x09745, 0x09750, 0x0975b, 0x09770, 0x097ef,
	 0x09f00, 0x09f0f, 0x09f20, 0x09f23, 0x09f30, 0x09f31, 0x0a600, 0x0a600,
	 0x0a603, 0x0a603, 0x0a610, 0x0a61f, 0x0a630, 0x0a632, 0x0a638, 0x0a63c,
	 0x0a640, 0x0a67f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_slice_pipe_bv_registers), 8));

/*
 * Block   : ['BROADCAST', 'CX_DBGC', 'CX_MISC', 'DBGC', 'GBIF', 'GMUAO']
 * Block   : ['GMUCX', 'GMUGX', 'GRAS', 'PC', 'RBBM']
 * Block   : ['RDVM', 'UCHE', 'VFD', 'VPC', 'VSC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_NONE
 * pairs   : 1 (Regs:1)
 */
static const u32 gen8_0_0_non_context_pipe_lpac_registers[] = {
	 0x00e14, 0x00e14,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_pipe_lpac_registers), 8));

/*
 * Block   : ['RB']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 4 (Regs:26)
 */
static const u32 gen8_0_0_non_context_rb_pipe_br_rbp_registers[] = {
	 0x08f00, 0x08f07, 0x08f10, 0x08f15, 0x08f20, 0x08f29, 0x08f30, 0x08f31,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_rb_pipe_br_rbp_registers), 8));

/*
 * Block   : ['RB']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 5 (Regs:32)
 */
static const u32 gen8_0_0_non_context_rb_slice_pipe_br_rac_registers[] = {
	 0x08e09, 0x08e0b, 0x08e10, 0x08e17, 0x08e51, 0x08e5a, 0x08e69, 0x08e6f,
	 0x08ea0, 0x08ea3,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_rb_slice_pipe_br_rac_registers), 8));

/*
 * Block   : ['RB']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 9 (Regs:28)
 */
static const u32 gen8_0_0_non_context_rb_slice_pipe_br_rbp_registers[] = {
	 0x08e01, 0x08e01, 0x08e04, 0x08e04, 0x08e06, 0x08e08, 0x08e0c, 0x08e0c,
	 0x08e18, 0x08e1c, 0x08e3b, 0x08e40, 0x08e50, 0x08e50, 0x08e5d, 0x08e5d,
	 0x08e77, 0x08e7f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_rb_slice_pipe_br_rbp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * Cluster : CLUSTER_NONE
 * Location: HLSQ_STATE
 * pairs   : 8 (Regs:34)
 */
static const u32 gen8_0_0_non_context_sp_pipe_br_hlsq_state_registers[] = {
	 0x0ae05, 0x0ae05, 0x0ae10, 0x0ae13, 0x0ae15, 0x0ae16, 0x0ae52, 0x0ae52,
	 0x0ae60, 0x0ae67, 0x0ae69, 0x0ae6e, 0x0ae70, 0x0ae75, 0x0aec0, 0x0aec5,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_sp_pipe_br_hlsq_state_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * Cluster : CLUSTER_NONE
 * Location: SP_TOP
 * pairs   : 6 (Regs:60)
 */
static const u32 gen8_0_0_non_context_sp_pipe_br_sp_top_registers[] = {
	 0x0ae00, 0x0ae0c, 0x0ae0f, 0x0ae0f, 0x0ae35, 0x0ae35, 0x0ae3a, 0x0ae3f,
	 0x0ae50, 0x0ae52, 0x0ae80, 0x0aea3,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_sp_pipe_br_sp_top_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * Cluster : CLUSTER_NONE
 * Location: USPTP
 * pairs   : 9 (Regs:64)
 */
static const u32 gen8_0_0_non_context_sp_pipe_br_usptp_registers[] = {
	 0x0ae00, 0x0ae0c, 0x0ae0f, 0x0ae0f, 0x0ae17, 0x0ae19, 0x0ae30, 0x0ae32,
	 0x0ae35, 0x0ae35, 0x0ae3a, 0x0ae3b, 0x0ae3e, 0x0ae3f, 0x0ae50, 0x0ae52,
	 0x0ae80, 0x0aea3,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_sp_pipe_br_usptp_registers), 8));

/*
 * Block   : ['TPL1']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * Cluster : CLUSTER_NONE
 * Location: USPTP
 * pairs   : 5 (Regs:48)
 */
static const u32 gen8_0_0_non_context_tpl1_pipe_br_usptp_registers[] = {
	 0x0b600, 0x0b600, 0x0b602, 0x0b602, 0x0b604, 0x0b604, 0x0b606, 0x0b61e,
	 0x0b620, 0x0b633,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_non_context_tpl1_pipe_br_usptp_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_VPC_VS
 * pairs   : 8 (Regs:232)
 */
static const u32 gen8_0_0_gras_slice_pipe_br_cluster_vpc_vs_registers[] = {
	 0x08200, 0x08213, 0x08220, 0x08225, 0x08230, 0x0823b, 0x08240, 0x0825f,
	 0x08270, 0x0828f, 0x0829f, 0x082b7, 0x082d0, 0x0832f, 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gras_slice_pipe_br_cluster_vpc_vs_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_GRAS
 * pairs   : 13 (Regs:326)
 */
static const u32 gen8_0_0_gras_slice_pipe_br_cluster_gras_registers[] = {
	 0x08080, 0x08080, 0x08086, 0x08092, 0x080c0, 0x080df, 0x08101, 0x08110,
	 0x08130, 0x0814f, 0x08200, 0x08213, 0x08220, 0x08225, 0x08230, 0x0823b,
	 0x08240, 0x0825f, 0x08270, 0x0828f, 0x0829f, 0x082b7, 0x082d0, 0x0832f,
	 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gras_slice_pipe_br_cluster_gras_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_VPC_VS
 * pairs   : 8 (Regs:232)
 */
static const u32 gen8_0_0_gras_slice_pipe_bv_cluster_vpc_vs_registers[] = {
	 0x08200, 0x08213, 0x08220, 0x08225, 0x08230, 0x0823b, 0x08240, 0x0825f,
	 0x08270, 0x0828f, 0x0829f, 0x082b7, 0x082d0, 0x0832f, 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gras_slice_pipe_bv_cluster_vpc_vs_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_GRAS
 * pairs   : 13 (Regs:326)
 */
static const u32 gen8_0_0_gras_slice_pipe_bv_cluster_gras_registers[] = {
	 0x08080, 0x08080, 0x08086, 0x08092, 0x080c0, 0x080df, 0x08101, 0x08110,
	 0x08130, 0x0814f, 0x08200, 0x08213, 0x08220, 0x08225, 0x08230, 0x0823b,
	 0x08240, 0x0825f, 0x08270, 0x0828f, 0x0829f, 0x082b7, 0x082d0, 0x0832f,
	 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gras_slice_pipe_bv_cluster_gras_registers), 8));

/*
 * Block   : ['PC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_FE_US
 * pairs   : 6 (Regs:35)
 */
static const u32 gen8_0_0_pc_pipe_br_cluster_fe_us_registers[] = {
	 0x09805, 0x09807, 0x0980b, 0x0980b, 0x09812, 0x09817, 0x0981a, 0x0981b,
	 0x09b00, 0x09b0d, 0x09b10, 0x09b18,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_pc_pipe_br_cluster_fe_us_registers), 8));

/*
 * Block   : ['PC']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_FE_S
 * pairs   : 2 (Regs:23)
 */
static const u32 gen8_0_0_pc_slice_pipe_br_cluster_fe_s_registers[] = {
	 0x09b00, 0x09b0d, 0x09b10, 0x09b18,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_pc_slice_pipe_br_cluster_fe_s_registers), 8));

/*
 * Block   : ['PC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_FE_US
 * pairs   : 6 (Regs:35)
 */
static const u32 gen8_0_0_pc_pipe_bv_cluster_fe_us_registers[] = {
	 0x09805, 0x09807, 0x0980b, 0x0980b, 0x09812, 0x09817, 0x0981a, 0x0981b,
	 0x09b00, 0x09b0d, 0x09b10, 0x09b18,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_pc_pipe_bv_cluster_fe_us_registers), 8));

/*
 * Block   : ['PC']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_FE_S
 * pairs   : 2 (Regs:23)
 */
static const u32 gen8_0_0_pc_slice_pipe_bv_cluster_fe_s_registers[] = {
	 0x09b00, 0x09b0d, 0x09b10, 0x09b18,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_pc_slice_pipe_bv_cluster_fe_s_registers), 8));

/*
 * Block   : ['VFD']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_FE_S
 * pairs   : 2 (Regs:236)
 */
static const u32 gen8_0_0_vfd_slice_pipe_br_cluster_fe_s_registers[] = {
	 0x0a000, 0x0a009, 0x0a00e, 0x0a0ef,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vfd_slice_pipe_br_cluster_fe_s_registers), 8));

/*
 * Block   : ['VFD']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_FE_S
 * pairs   : 2 (Regs:236)
 */
static const u32 gen8_0_0_vfd_slice_pipe_bv_cluster_fe_s_registers[] = {
	 0x0a000, 0x0a009, 0x0a00e, 0x0a0ef,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vfd_slice_pipe_bv_cluster_fe_s_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_FE_S
 * pairs   : 1 (Regs:27)
 */
static const u32 gen8_0_0_vpc_slice_pipe_br_cluster_fe_s_registers[] = {
	 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_slice_pipe_br_cluster_fe_s_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_VPC_VS
 * pairs   : 2 (Regs:29)
 */
static const u32 gen8_0_0_vpc_slice_pipe_br_cluster_vpc_vs_registers[] = {
	 0x090c0, 0x090c1, 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_slice_pipe_br_cluster_vpc_vs_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_VPC_US
 * pairs   : 3 (Regs:58)
 */
static const u32 gen8_0_0_vpc_pipe_br_cluster_vpc_us_registers[] = {
	 0x09180, 0x09180, 0x09182, 0x0919f, 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_pipe_br_cluster_vpc_us_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_VPC_PS
 * pairs   : 4 (Regs:52)
 */
static const u32 gen8_0_0_vpc_slice_pipe_br_cluster_vpc_ps_registers[] = {
	 0x09240, 0x0924f, 0x09252, 0x09255, 0x09278, 0x0927c, 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_slice_pipe_br_cluster_vpc_ps_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_FE_S
 * pairs   : 1 (Regs:27)
 */
static const u32 gen8_0_0_vpc_slice_pipe_bv_cluster_fe_s_registers[] = {
	 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_slice_pipe_bv_cluster_fe_s_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_VPC_VS
 * pairs   : 2 (Regs:29)
 */
static const u32 gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_vs_registers[] = {
	 0x090c0, 0x090c1, 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_vs_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_VPC_US
 * pairs   : 3 (Regs:58)
 */
static const u32 gen8_0_0_vpc_pipe_bv_cluster_vpc_us_registers[] = {
	 0x09180, 0x09180, 0x09182, 0x0919f, 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_pipe_bv_cluster_vpc_us_registers), 8));

/*
 * Block   : ['VPC']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_VPC_PS
 * pairs   : 4 (Regs:52)
 */
static const u32 gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_ps_registers[] = {
	 0x09240, 0x0924f, 0x09252, 0x09255, 0x09278, 0x0927c, 0x09300, 0x0931a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_ps_registers), 8));

/*
 * Block   : ['RB']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_PS
 * pairs   : 36 (Regs:151)
 */
static const u32 gen8_0_0_rb_slice_pipe_br_cluster_ps_rac_registers[] = {
	 0x08802, 0x08802, 0x08804, 0x0880a, 0x0880e, 0x08811, 0x08813, 0x08814,
	 0x08818, 0x0881e, 0x08821, 0x08821, 0x08823, 0x08826, 0x08829, 0x08829,
	 0x0882b, 0x0882e, 0x08831, 0x08831, 0x08833, 0x08836, 0x08839, 0x08839,
	 0x0883b, 0x0883e, 0x08841, 0x08841, 0x08843, 0x08846, 0x08849, 0x08849,
	 0x0884b, 0x0884e, 0x08851, 0x08851, 0x08853, 0x08856, 0x08859, 0x08859,
	 0x0885b, 0x0885e, 0x08860, 0x08864, 0x08870, 0x08870, 0x08873, 0x08876,
	 0x08878, 0x08879, 0x08882, 0x08885, 0x08887, 0x08889, 0x08891, 0x08891,
	 0x08898, 0x08899, 0x088b0, 0x088cf, 0x088e5, 0x088e5, 0x088f4, 0x088f5,
	 0x08930, 0x08937, 0x08c00, 0x08c01, 0x08c18, 0x08c1f, 0x08c26, 0x08c34,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_rb_slice_pipe_br_cluster_ps_rac_registers), 8));

/*
 * Block   : ['RB']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_PS
 * pairs   : 36 (Regs:104)
 */
static const u32 gen8_0_0_rb_slice_pipe_br_cluster_ps_rbp_registers[] = {
	 0x08800, 0x08801, 0x08803, 0x08803, 0x0880b, 0x0880d, 0x08812, 0x08812,
	 0x08815, 0x08816, 0x08820, 0x08820, 0x08822, 0x08822, 0x08827, 0x08828,
	 0x0882a, 0x0882a, 0x0882f, 0x08830, 0x08832, 0x08832, 0x08837, 0x08838,
	 0x0883a, 0x0883a, 0x0883f, 0x08840, 0x08842, 0x08842, 0x08847, 0x08848,
	 0x0884a, 0x0884a, 0x0884f, 0x08850, 0x08852, 0x08852, 0x08857, 0x08858,
	 0x0885a, 0x0885a, 0x0885f, 0x0885f, 0x08865, 0x08865, 0x08871, 0x08872,
	 0x08877, 0x08877, 0x08880, 0x08881, 0x08886, 0x08886, 0x08890, 0x08890,
	 0x088d0, 0x088e4, 0x088e6, 0x088e6, 0x088e8, 0x088ea, 0x088f0, 0x088f1,
	 0x08900, 0x0891a, 0x08927, 0x08928, 0x08c17, 0x08c17, 0x08c20, 0x08c25,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_rb_slice_pipe_br_cluster_ps_rbp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: HLSQ_STATE
 * pairs   : 29 (Regs:114)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_registers[] = {
	 0x0a800, 0x0a801, 0x0a81b, 0x0a81d, 0x0a822, 0x0a822, 0x0a824, 0x0a824,
	 0x0a827, 0x0a82a, 0x0a830, 0x0a830, 0x0a832, 0x0a835, 0x0a83a, 0x0a83a,
	 0x0a83c, 0x0a83c, 0x0a83f, 0x0a841, 0x0a85b, 0x0a85d, 0x0a862, 0x0a862,
	 0x0a864, 0x0a864, 0x0a867, 0x0a867, 0x0a870, 0x0a870, 0x0a872, 0x0a872,
	 0x0a88c, 0x0a88e, 0x0a893, 0x0a893, 0x0a895, 0x0a895, 0x0a898, 0x0a898,
	 0x0a89a, 0x0a89d, 0x0a8b0, 0x0a8bb, 0x0a8c0, 0x0a8c3, 0x0a974, 0x0a977,
	 0x0ab00, 0x0ab03, 0x0ab05, 0x0ab05, 0x0ab09, 0x0ab09, 0x0ab23, 0x0ab23,
	 0x0abd0, 0x0abff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: HLSQ_STATE
 * pairs   : 2 (Regs:34)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_cctx_registers[] = {
	 0x0a8a0, 0x0a8af, 0x0ab0a, 0x0ab1b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: HLSQ_STATE
 * pairs   : 1 (Regs:160)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_shared_const_registers[] = {
	 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_shared_const_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: SP_TOP
 * pairs   : 23 (Regs:39)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_registers[] = {
	 0x0a800, 0x0a800, 0x0a81c, 0x0a81d, 0x0a822, 0x0a824, 0x0a826, 0x0a826,
	 0x0a82d, 0x0a82d, 0x0a82f, 0x0a831, 0x0a834, 0x0a835, 0x0a83a, 0x0a83c,
	 0x0a83e, 0x0a83e, 0x0a840, 0x0a840, 0x0a85c, 0x0a85d, 0x0a862, 0x0a864,
	 0x0a866, 0x0a866, 0x0a868, 0x0a868, 0x0a870, 0x0a871, 0x0a88d, 0x0a88e,
	 0x0a893, 0x0a895, 0x0a897, 0x0a897, 0x0a899, 0x0a899, 0x0ab00, 0x0ab00,
	 0x0ab02, 0x0ab02, 0x0ab04, 0x0ab05, 0x0ab09, 0x0ab09,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: SP_TOP
 * pairs   : 2 (Regs:34)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_cctx_registers[] = {
	 0x0a8a0, 0x0a8af, 0x0ab0a, 0x0ab1b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: USPTP
 * pairs   : 15 (Regs:145)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_registers[] = {
	 0x0a800, 0x0a81b, 0x0a81e, 0x0a821, 0x0a823, 0x0a827, 0x0a82d, 0x0a82d,
	 0x0a82f, 0x0a833, 0x0a836, 0x0a839, 0x0a83b, 0x0a85b, 0x0a85e, 0x0a861,
	 0x0a863, 0x0a868, 0x0a870, 0x0a88c, 0x0a88f, 0x0a892, 0x0a894, 0x0a899,
	 0x0a8c0, 0x0a8c3, 0x0a974, 0x0a977, 0x0ab00, 0x0ab07,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: USPTP
 * pairs   : 1 (Regs:160)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_shared_const_registers[] = {
	 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_shared_const_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_STATE
 * pairs   : 21 (Regs:88)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_registers[] = {
	 0x0a980, 0x0a984, 0x0a99e, 0x0a99e, 0x0a9a7, 0x0a9a7, 0x0a9aa, 0x0a9aa,
	 0x0a9af, 0x0a9b0, 0x0a9b2, 0x0a9b5, 0x0a9ba, 0x0a9ba, 0x0a9bc, 0x0a9bc,
	 0x0a9c4, 0x0a9c4, 0x0a9c6, 0x0a9c6, 0x0a9cd, 0x0a9cd, 0x0a9fa, 0x0a9fc,
	 0x0aa00, 0x0aa00, 0x0aa0d, 0x0aa12, 0x0aa30, 0x0aa31, 0x0aaf2, 0x0aaf3,
	 0x0ab00, 0x0ab03, 0x0ab05, 0x0ab05, 0x0ab09, 0x0ab09, 0x0ab23, 0x0ab23,
	 0x0abd0, 0x0abff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_STATE
 * pairs   : 2 (Regs:44)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_cctx_registers[] = {
	 0x0a9e0, 0x0a9f9, 0x0ab0a, 0x0ab1b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_STATE
 * pairs   : 2 (Regs:320)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_shared_const_registers[] = {
	 0x0aa40, 0x0aadf, 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_shared_const_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_DP
 * pairs   : 2 (Regs:13)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers[] = {
	 0x0a9b1, 0x0a9b1, 0x0a9d4, 0x0a9df,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: SP_TOP
 * pairs   : 17 (Regs:34)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_registers[] = {
	 0x0a980, 0x0a980, 0x0a982, 0x0a984, 0x0a99e, 0x0a9a2, 0x0a9a7, 0x0a9a8,
	 0x0a9aa, 0x0a9ab, 0x0a9ae, 0x0a9ae, 0x0a9b0, 0x0a9b1, 0x0a9b3, 0x0a9b5,
	 0x0a9ba, 0x0a9bc, 0x0a9be, 0x0a9be, 0x0a9c5, 0x0a9c5, 0x0a9ce, 0x0a9ce,
	 0x0aa00, 0x0aa03, 0x0ab00, 0x0ab00, 0x0ab02, 0x0ab02, 0x0ab04, 0x0ab05,
	 0x0ab09, 0x0ab09,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: SP_TOP
 * pairs   : 2 (Regs:44)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers[] = {
	 0x0a9e0, 0x0a9f9, 0x0ab0a, 0x0ab1b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: USPTP
 * pairs   : 15 (Regs:89)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_registers[] = {
	 0x0a980, 0x0a982, 0x0a985, 0x0a9a6, 0x0a9a8, 0x0a9a9, 0x0a9ab, 0x0a9ae,
	 0x0a9b0, 0x0a9b3, 0x0a9b6, 0x0a9b9, 0x0a9bb, 0x0a9bf, 0x0a9c2, 0x0a9c3,
	 0x0a9c5, 0x0a9c5, 0x0a9cd, 0x0a9ce, 0x0a9d0, 0x0a9d3, 0x0aa01, 0x0aa0c,
	 0x0aa30, 0x0aa31, 0x0aaf2, 0x0aaf3, 0x0ab00, 0x0ab07,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: USPTP
 * pairs   : 2 (Regs:320)
 */
static const u32 gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_shared_const_registers[] = {
	 0x0aa40, 0x0aadf, 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_shared_const_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: HLSQ_STATE
 * pairs   : 28 (Regs:112)
 */
static const u32 gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_registers[] = {
	 0x0a800, 0x0a801, 0x0a81b, 0x0a81d, 0x0a822, 0x0a822, 0x0a824, 0x0a824,
	 0x0a827, 0x0a82a, 0x0a830, 0x0a830, 0x0a832, 0x0a835, 0x0a83a, 0x0a83a,
	 0x0a83c, 0x0a83c, 0x0a83f, 0x0a841, 0x0a85b, 0x0a85d, 0x0a862, 0x0a862,
	 0x0a864, 0x0a864, 0x0a867, 0x0a867, 0x0a870, 0x0a870, 0x0a872, 0x0a872,
	 0x0a88c, 0x0a88e, 0x0a893, 0x0a893, 0x0a895, 0x0a895, 0x0a898, 0x0a898,
	 0x0a89a, 0x0a89d, 0x0a8b0, 0x0a8bb, 0x0a8c0, 0x0a8c3, 0x0a974, 0x0a977,
	 0x0ab00, 0x0ab02, 0x0ab09, 0x0ab09, 0x0ab23, 0x0ab23, 0x0abd0, 0x0abff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: HLSQ_STATE
 * pairs   : 2 (Regs:34)
 */
static const u32 gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_cctx_registers[] = {
	 0x0a8a0, 0x0a8af, 0x0ab0a, 0x0ab1b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: HLSQ_STATE
 * pairs   : 1 (Regs:160)
 */
static const u32 gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_shared_const_registers[] = {
	 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_shared_const_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: SP_TOP
 * pairs   : 22 (Regs:37)
 */
static const u32 gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_registers[] = {
	 0x0a800, 0x0a800, 0x0a81c, 0x0a81d, 0x0a822, 0x0a824, 0x0a826, 0x0a826,
	 0x0a82d, 0x0a82d, 0x0a82f, 0x0a831, 0x0a834, 0x0a835, 0x0a83a, 0x0a83c,
	 0x0a83e, 0x0a83e, 0x0a840, 0x0a840, 0x0a85c, 0x0a85d, 0x0a862, 0x0a864,
	 0x0a866, 0x0a866, 0x0a868, 0x0a868, 0x0a870, 0x0a871, 0x0a88d, 0x0a88e,
	 0x0a893, 0x0a895, 0x0a897, 0x0a897, 0x0a899, 0x0a899, 0x0ab00, 0x0ab00,
	 0x0ab02, 0x0ab02, 0x0ab09, 0x0ab09,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: SP_TOP
 * pairs   : 2 (Regs:34)
 */
static const u32 gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_cctx_registers[] = {
	 0x0a8a0, 0x0a8af, 0x0ab0a, 0x0ab1b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: USPTP
 * pairs   : 16 (Regs:142)
 */
static const u32 gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_registers[] = {
	 0x0a800, 0x0a81b, 0x0a81e, 0x0a821, 0x0a823, 0x0a827, 0x0a82d, 0x0a82d,
	 0x0a82f, 0x0a833, 0x0a836, 0x0a839, 0x0a83b, 0x0a85b, 0x0a85e, 0x0a861,
	 0x0a863, 0x0a868, 0x0a870, 0x0a88c, 0x0a88f, 0x0a892, 0x0a894, 0x0a899,
	 0x0a8c0, 0x0a8c3, 0x0a974, 0x0a977, 0x0ab00, 0x0ab02, 0x0ab06, 0x0ab07,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: USPTP
 * pairs   : 1 (Regs:160)
 */
static const u32 gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_shared_const_registers[] = {
	 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_shared_const_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_STATE
 * pairs   : 14 (Regs:73)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_registers[] = {
	 0x0a9b0, 0x0a9b0, 0x0a9b2, 0x0a9b5, 0x0a9ba, 0x0a9ba, 0x0a9bc, 0x0a9bc,
	 0x0a9c4, 0x0a9c4, 0x0a9cd, 0x0a9cd, 0x0a9fa, 0x0a9fc, 0x0aa00, 0x0aa00,
	 0x0aa10, 0x0aa12, 0x0aa31, 0x0aa35, 0x0aaf3, 0x0aaf3, 0x0ab00, 0x0ab01,
	 0x0ab23, 0x0ab23, 0x0abd0, 0x0abff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_STATE
 * pairs   : 2 (Regs:22)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_cctx_registers[] = {
	 0x0a9e2, 0x0a9e3, 0x0a9e6, 0x0a9f9,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_STATE
 * pairs   : 2 (Regs:320)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_shared_const_registers[] = {
	 0x0aa40, 0x0aadf, 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_shared_const_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: HLSQ_DP
 * pairs   : 2 (Regs:13)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_dp_registers[] = {
	 0x0a9b1, 0x0a9b1, 0x0a9d4, 0x0a9df,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_dp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: SP_TOP
 * pairs   : 8 (Regs:13)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_registers[] = {
	 0x0a9b0, 0x0a9b1, 0x0a9b3, 0x0a9b5, 0x0a9ba, 0x0a9bc, 0x0a9be, 0x0a9be,
	 0x0a9c5, 0x0a9c5, 0x0a9ce, 0x0a9ce, 0x0aa00, 0x0aa00, 0x0ab00, 0x0ab00,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: SP_TOP
 * pairs   : 2 (Regs:22)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_cctx_registers[] = {
	 0x0a9e2, 0x0a9e3, 0x0a9e6, 0x0a9f9,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_cctx_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: USPTP
 * pairs   : 11 (Regs:26)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_registers[] = {
	 0x0a9b0, 0x0a9b3, 0x0a9b6, 0x0a9b9, 0x0a9bb, 0x0a9be, 0x0a9c2, 0x0a9c3,
	 0x0a9c5, 0x0a9c5, 0x0a9cd, 0x0a9ce, 0x0a9d0, 0x0a9d3, 0x0aa31, 0x0aa31,
	 0x0aaf3, 0x0aaf3, 0x0ab00, 0x0ab01, 0x0ab06, 0x0ab06,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: USPTP
 * pairs   : 2 (Regs:320)
 */
static const u32 gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_shared_const_registers[] = {
	 0x0aa40, 0x0aadf, 0x0ab30, 0x0abcf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(
	sizeof(gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_shared_const_registers), 8));

/*
 * Block   : ['TPL1']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_VS
 * Location: USPTP
 * pairs   : 3 (Regs:7)
 */
static const u32 gen8_0_0_tpl1_slice_pipe_br_cluster_sp_vs_usptp_registers[] = {
	 0x0b300, 0x0b304, 0x0b307, 0x0b307, 0x0b309, 0x0b309,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_tpl1_slice_pipe_br_cluster_sp_vs_usptp_registers), 8));

/*
 * Block   : ['TPL1']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_SP_PS
 * Location: USPTP
 * pairs   : 6 (Regs:41)
 */
static const u32 gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers[] = {
	 0x0b180, 0x0b183, 0x0b190, 0x0b195, 0x0b2c0, 0x0b2d7, 0x0b300, 0x0b304,
	 0x0b307, 0x0b307, 0x0b309, 0x0b309,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers), 8));

/*
 * Block   : ['TPL1']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_SP_VS
 * Location: USPTP
 * pairs   : 3 (Regs:7)
 */
static const u32 gen8_0_0_tpl1_slice_pipe_bv_cluster_sp_vs_usptp_registers[] = {
	 0x0b300, 0x0b304, 0x0b307, 0x0b307, 0x0b309, 0x0b309,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_tpl1_slice_pipe_bv_cluster_sp_vs_usptp_registers), 8));

/*
 * Block   : ['TPL1']
 * REGION  : SLICE
 * Pipeline: PIPE_LPAC
 * Cluster : CLUSTER_SP_PS
 * Location: USPTP
 * pairs   : 5 (Regs:7)
 */
static const u32 gen8_0_0_tpl1_slice_pipe_lpac_cluster_sp_ps_usptp_registers[] = {
	 0x0b180, 0x0b181, 0x0b300, 0x0b301, 0x0b304, 0x0b304, 0x0b307, 0x0b307,
	 0x0b309, 0x0b309,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_tpl1_slice_pipe_lpac_cluster_sp_ps_usptp_registers), 8));

static const struct sel_reg gen8_0_0_rb_rac_sel = {
	.host_reg = GEN8_RB_SUB_BLOCK_SEL_CNTL_HOST,
	.cd_reg = GEN8_RB_SUB_BLOCK_SEL_CNTL_CD,
	.val = 0,
};

static const struct sel_reg gen8_0_0_rb_rbp_sel = {
	.host_reg = GEN8_RB_SUB_BLOCK_SEL_CNTL_HOST,
	.cd_reg = GEN8_RB_SUB_BLOCK_SEL_CNTL_CD,
	.val = 0x9,
};

static struct gen8_cluster_registers gen8_0_0_cp_clusters[] = {
	{ CLUSTER_NONE, UNSLICE, PIPE_NONE, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_none_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_br_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_cp_slice_cp_pipe_br_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_bv_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_0_0_cp_slice_cp_pipe_bv_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_LPAC, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_lpac_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_AQE0, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_aqe0_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_AQE1, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_aqe1_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_DDE_BR, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_dde_br_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_DDE_BV, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_dde_bv_registers,  },
};

static struct gen8_cluster_registers gen8_0_0_mvc_clusters[] = {
	{ CLUSTER_NONE, UNSLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_non_context_pipe_br_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_non_context_slice_pipe_br_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_0_0_non_context_pipe_bv_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_0_0_non_context_slice_pipe_bv_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_LPAC, STATE_NON_CONTEXT,
		gen8_0_0_non_context_pipe_lpac_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_non_context_rb_pipe_br_rbp_registers, &gen8_0_0_rb_rbp_sel, },
	{ CLUSTER_NONE, SLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_non_context_rb_slice_pipe_br_rac_registers, &gen8_0_0_rb_rac_sel, },
	{ CLUSTER_NONE, SLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_non_context_rb_slice_pipe_br_rbp_registers, &gen8_0_0_rb_rbp_sel, },
	{ CLUSTER_PS, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_rb_slice_pipe_br_cluster_ps_rac_registers, &gen8_0_0_rb_rac_sel, },
	{ CLUSTER_PS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_rb_slice_pipe_br_cluster_ps_rac_registers, &gen8_0_0_rb_rac_sel, },
	{ CLUSTER_PS, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_rb_slice_pipe_br_cluster_ps_rbp_registers, &gen8_0_0_rb_rbp_sel, },
	{ CLUSTER_PS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_rb_slice_pipe_br_cluster_ps_rbp_registers, &gen8_0_0_rb_rbp_sel, },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_gras_slice_pipe_br_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_gras_slice_pipe_br_cluster_vpc_vs_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_gras_slice_pipe_br_cluster_gras_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_gras_slice_pipe_br_cluster_gras_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_gras_slice_pipe_bv_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_gras_slice_pipe_bv_cluster_vpc_vs_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_gras_slice_pipe_bv_cluster_gras_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_gras_slice_pipe_bv_cluster_gras_registers,  },
	{ CLUSTER_FE_US, UNSLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_pc_pipe_br_cluster_fe_us_registers,  },
	{ CLUSTER_FE_US, UNSLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_pc_pipe_br_cluster_fe_us_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_pc_slice_pipe_br_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_pc_slice_pipe_br_cluster_fe_s_registers,  },
	{ CLUSTER_FE_US, UNSLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_pc_pipe_bv_cluster_fe_us_registers,  },
	{ CLUSTER_FE_US, UNSLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_pc_pipe_bv_cluster_fe_us_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_pc_slice_pipe_bv_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_pc_slice_pipe_bv_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_vfd_slice_pipe_br_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_vfd_slice_pipe_br_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_vfd_slice_pipe_bv_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_vfd_slice_pipe_bv_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_slice_pipe_br_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_slice_pipe_br_cluster_fe_s_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_slice_pipe_br_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_slice_pipe_br_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_US, UNSLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_pipe_br_cluster_vpc_us_registers,  },
	{ CLUSTER_VPC_US, UNSLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_pipe_br_cluster_vpc_us_registers,  },
	{ CLUSTER_VPC_PS, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_slice_pipe_br_cluster_vpc_ps_registers,  },
	{ CLUSTER_VPC_PS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_slice_pipe_br_cluster_vpc_ps_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_slice_pipe_bv_cluster_fe_s_registers,  },
	{ CLUSTER_FE_S, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_slice_pipe_bv_cluster_fe_s_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_US, UNSLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_pipe_bv_cluster_vpc_us_registers,  },
	{ CLUSTER_VPC_US, UNSLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_pipe_bv_cluster_vpc_us_registers,  },
	{ CLUSTER_VPC_PS, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_ps_registers,  },
	{ CLUSTER_VPC_PS, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_0_0_vpc_slice_pipe_bv_cluster_vpc_ps_registers,  },
};

static struct gen8_sptp_cluster_registers gen8_0_0_sptp_clusters[] = {
	{ CLUSTER_NONE, UNSLICE, 2, 1, SP_NCTX_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_non_context_sp_pipe_br_hlsq_state_registers, 0xae00},
	{ CLUSTER_NONE, UNSLICE, 2, 1, SP_NCTX_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_non_context_sp_pipe_br_sp_top_registers, 0xae00},
	{ CLUSTER_NONE, UNSLICE, 2, 1, SP_NCTX_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_non_context_sp_pipe_br_usptp_registers, 0xae00},
	{ CLUSTER_NONE, UNSLICE, 2, 1, TP0_NCTX_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_non_context_tpl1_pipe_br_usptp_registers, 0xb600},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, USPTP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, USPTP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, USPTP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_shared_const_registers,
		0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, USPTP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, USPTP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX2_3D_CPS_REG, PIPE_BR, 2, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX2_3D_CPS_REG, PIPE_BR, 2, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX3_3D_CPS_REG, PIPE_BR, 3, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 2, 1, SP_CTX3_3D_CPS_REG, PIPE_BR, 3, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 2, 1, TP0_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_VS, SLICE, 2, 1, TP0_CTX0_3D_CVS_REG, PIPE_BV, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_VS, SLICE, 2, 1, TP0_CTX1_3D_CVS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_VS, SLICE, 2, 1, TP0_CTX1_3D_CVS_REG, PIPE_BV, 1, USPTP,
		gen8_0_0_tpl1_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 2, 1, TP0_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 2, 1, TP0_CTX0_3D_CPS_REG, PIPE_LPAC, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_lpac_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 2, 1, TP0_CTX1_3D_CPS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 2, 1, TP0_CTX2_3D_CPS_REG, PIPE_BR, 2, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 2, 1, TP0_CTX3_3D_CPS_REG, PIPE_BR, 3, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
};

/*
 * Before dumping the CP MVC
 * Program CP_APERTURE_CNTL_* with pipeID={CP_PIPE}
 * Then dump corresponding {Register_PIPE}
 */
static struct gen8_cp_indexed_reg gen8_0_0_cp_indexed_reg_list[] = {
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_BR, 0x00040},
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_BV, 0x00040},
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_LPAC, 0x00040},
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_AQE0, 0x00040},
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_AQE1, 0x00040},
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_DDE_BR, 0x00040},
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_DDE_BV, 0x00040},
	{ GEN8_CP_DRAW_STATE_ADDR_PIPE, GEN8_CP_DRAW_STATE_DATA_PIPE, UNSLICE, PIPE_BR, 0x00200},
	{ GEN8_CP_DRAW_STATE_ADDR_PIPE, GEN8_CP_DRAW_STATE_DATA_PIPE, UNSLICE, PIPE_BV, 0x00200},
	{ GEN8_CP_DRAW_STATE_ADDR_PIPE, GEN8_CP_DRAW_STATE_DATA_PIPE, UNSLICE, PIPE_LPAC, 0x00200},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_BR, 0x00800},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_BV, 0x00800},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_LPAC, 0x00200},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_AQE0, 0x00100},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_AQE1, 0x00100},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_DDE_BR, 0x00100},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_DDE_BV, 0x00100},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
			UNSLICE, PIPE_BR, 0x08000},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
			UNSLICE, PIPE_BV, 0x08000},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
			UNSLICE, PIPE_LPAC, 0x08000},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
			UNSLICE, PIPE_AQE0, 0x08000},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
			UNSLICE, PIPE_AQE1, 0x08000},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
			UNSLICE, PIPE_DDE_BR, 0x08000},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
			UNSLICE, PIPE_DDE_BV, 0x08000},
	{ GEN8_CP_RESOURCE_TABLE_DBG_ADDR_BV, GEN8_CP_RESOURCE_TABLE_DBG_DATA_BV,
			UNSLICE, PIPE_NONE, 0x04100},
	{ GEN8_CP_FIFO_DBG_ADDR_LPAC, GEN8_CP_FIFO_DBG_DATA_LPAC, UNSLICE, PIPE_NONE, 0x00040},
	{ GEN8_CP_FIFO_DBG_ADDR_DDE_PIPE, GEN8_CP_FIFO_DBG_DATA_DDE_PIPE,
			UNSLICE, PIPE_DDE_BR, 0x01100},
	{ GEN8_CP_FIFO_DBG_ADDR_DDE_PIPE, GEN8_CP_FIFO_DBG_DATA_DDE_PIPE,
			UNSLICE, PIPE_DDE_BV, 0x01100},
};


/*
 * Before dumping the CP Mempool over the CP_*_MEM_POOL_DBG_ADDR/DATA
 * indexed register pair it must be stabilized.
 * for p in [PIPE_BR, PIPE_BV]:
 *   Program CP_APERTURE_CNTL_* with pipeID={p} sliceID={MAX_UINT}
 *   Program CP_CHICKEN_DBG_PIPE[crashStabilizeMVC] bit = 1.
 *   Dump CP_MEM_POOL_DBG_ADDR_PIPE for pipe=p
 *   Program CP_CHICKEN_DBG_PIPE[crashStabilizeMVC] bit = 0.
 *
 * same thing for CP_SLICE_MEM_POOL_DBG_ADDR_PIPE
 * for p in [PIPE_BR, PIPE_BV]:
 *   for s in [0,1,2]:
 *     Program CP_APERTURE_CNTL_* with pipeID={p} sliceID={s}
 *     Program CP_CHICKEN_DBG_PIPE[crashStabilizeMVC] bit = 1.
 *     Program CP_SLICE_CHICKEN_DBG[crashStabilizeMVC] bit = 1.
 *     Dump CP_SLICE_MEM_POOL_DBG_ADDR_PIPE for pipe=p, sliceID=s
 *     Program CP_CHICKEN_DBG_PIPE[crashStabilizeMVC] bit = 0.
 *     Program CP_SLICE_CHICKEN_DBG[crashStabilizeMVC] bit = 0.
 */

static struct gen8_cp_indexed_reg gen8_0_0_cp_mempool_reg_list[] = {
	{ GEN8_CP_MEM_POOL_DBG_ADDR_PIPE, GEN8_CP_MEM_POOL_DBG_DATA_PIPE,
			UNSLICE, PIPE_BR, 0x02400},
	{ GEN8_CP_MEM_POOL_DBG_ADDR_PIPE, GEN8_CP_MEM_POOL_DBG_DATA_PIPE,
			UNSLICE, PIPE_BV, 0x02400},
	{ GEN8_CP_SLICE_MEM_POOL_DBG_ADDR_PIPE, GEN8_CP_SLICE_MEM_POOL_DBG_DATA_PIPE,
			SLICE, PIPE_BR, 0x02400},
	{ GEN8_CP_SLICE_MEM_POOL_DBG_ADDR_PIPE, GEN8_CP_SLICE_MEM_POOL_DBG_DATA_PIPE,
			SLICE, PIPE_BV, 0x02400},
};

/*
 * this is just a temp table to give hint of reg pointers
 * which are not included so far in MVC and SPTP structs
 */
static struct gen8_reg_list gen8_0_0_reg_list[] = {
	{ UNSLICE, gen8_0_0_gpu_registers },
	{ SLICE, gen8_0_0_gpu_slice_registers },
	{ UNSLICE, gen8_0_0_dbgc_registers },
	{ SLICE, gen8_0_0_dbgc_slice_registers },
	{ UNSLICE, gen8_0_0_cx_dbgc_registers },
	{ UNSLICE, NULL},
};

static struct gen8_reg_list gen8_0_0_ahb_registers[] = {
	{ UNSLICE, gen8_0_0_gbif_registers },
	{ UNSLICE, gen8_0_0_ahb_precd_gpu_registers },
	{ SLICE, gen8_0_0_ahb_precd_gpu_slice_slice_registers },
	{ UNSLICE, gen8_0_0_ahb_secure_gpu_registers },
};

/*
 * Block   : ['GDPM_LKG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 9 (Regs:108)
 */
static const u32 gen8_0_0_gdpm_lkg_registers[] = {
	 0x21c00, 0x21c00, 0x21c08, 0x21c09, 0x21c0e, 0x21c0f, 0x21c4f, 0x21c50,
	 0x21c52, 0x21c52, 0x21c54, 0x21c56, 0x21c58, 0x21c5a, 0x21c5c, 0x21c60,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gdpm_lkg_registers), 8));

/*
 * Block   : ['GPU_CC_AHB2PHY_BROADCAST_SWMAN']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:256)
 */
static const u32 gen8_0_0_gpu_cc_ahb2phy_broadcast_swman_registers[] = {
	 0x24c00, 0x24cff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gpu_cc_ahb2phy_broadcast_swman_registers), 8));

/*
 * Block   : ['GPU_CC_AHB2PHY_SWMAN']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:6)
 */
static const u32 gen8_0_0_gpu_cc_ahb2phy_swman_registers[] = {
	 0x24800, 0x24805,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gpu_cc_ahb2phy_swman_registers), 8));

/*
 * Block   : ['GPU_CC_GPU_CC_REG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 26 (Regs:133)
 */
static const u32 gen8_0_0_gpu_cc_gpu_cc_reg_registers[] = {
	 0x25400, 0x25404, 0x25800, 0x25804, 0x25c00, 0x25c04, 0x26000, 0x26004,
	 0x26400, 0x26406, 0x26415, 0x2641d, 0x2641f, 0x26440, 0x26443, 0x26444,
	 0x26478, 0x2647a, 0x26489, 0x2648a, 0x2649c, 0x2649e, 0x264a0, 0x264a1,
	 0x264c5, 0x264c7, 0x264e8, 0x264ea, 0x264f9, 0x264fc, 0x2650b, 0x2650b,
	 0x2651c, 0x2651e, 0x26540, 0x2654b, 0x26554, 0x26556, 0x26558, 0x2655c,
	 0x2655e, 0x2655f, 0x26563, 0x26563, 0x2656d, 0x26573, 0x26576, 0x26576,
	 0x26578, 0x2657a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gpu_cc_gpu_cc_reg_registers), 8));

/*
 * Block   : ['GPU_CC_PLL0_CM_PLL_TAYCAN_COMMON']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:14)
 */
static const u32 gen8_0_0_gpu_cc_pll0_cm_pll_taycan_common_registers[] = {
	 0x24000, 0x2400d,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gpu_cc_pll0_cm_pll_taycan_common_registers), 8));

/*
 * Block   : ['ACD_ACD_MND']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 11 (Regs:68)
 */
static const u32 gen8_0_0_acd_acd_mnd_registers[] = {
	 0x1a400, 0x1a416, 0x1a420, 0x1a42d, 0x1a430, 0x1a431, 0x1a435, 0x1a435,
	 0x1a437, 0x1a437, 0x1a43a, 0x1a43a, 0x1a442, 0x1a44b, 0x1a44e, 0x1a453,
	 0x1a456, 0x1a458, 0x1a45b, 0x1a45d, 0x1a45f, 0x1a462,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_acd_acd_mnd_registers), 8));

/*
 * Block   : ['GX_CLKCTL_AHB2PHY_BROADCAST_SWMAN']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:256)
 */
static const u32 gen8_0_0_gx_clkctl_ahb2phy_broadcast_swman_registers[] = {
	 0x19c00, 0x19cff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gx_clkctl_ahb2phy_broadcast_swman_registers), 8));

/*
 * Block   : ['GX_CLKCTL_AHB2PHY_SWMAN']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:6)
 */
static const u32 gen8_0_0_gx_clkctl_ahb2phy_swman_registers[] = {
	 0x19800, 0x19805,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gx_clkctl_ahb2phy_swman_registers), 8));

/*
 * Block   : ['GX_CLKCTL_PLL0_CM_PLL_TAYCAN_COMMON']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:14)
 */
static const u32 gen8_0_0_gx_clkctl_pll0_cm_pll_taycan_common_registers[] = {
	 0x19000, 0x1900d,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gx_clkctl_pll0_cm_pll_taycan_common_registers), 8));

/*
 * Block   : ['GX_CLKCTL_GX_CLKCTL_REG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 11 (Regs:83)
 */
static const u32 gen8_0_0_gx_clkctl_gx_clkctl_reg_registers[] = {
	 0x1a000, 0x1a004, 0x1a008, 0x1a014, 0x1a017, 0x1a017, 0x1a019, 0x1a019,
	 0x1a022, 0x1a022, 0x1a024, 0x1a029, 0x1a03f, 0x1a05d, 0x1a060, 0x1a063,
	 0x1a065, 0x1a066, 0x1a068, 0x1a076, 0x1a078, 0x1a07b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_gx_clkctl_gx_clkctl_reg_registers), 8));

/*
 * Block   : ['RSCC_RSC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 101 (Regs:606)
 */
static const u32 gen8_0_0_rscc_rsc_registers[] = {
	 0x14000, 0x14034, 0x14036, 0x14036, 0x14040, 0x14042, 0x14044, 0x14045,
	 0x14047, 0x14047, 0x14080, 0x14084, 0x14089, 0x1408c, 0x14091, 0x14094,
	 0x14099, 0x1409c, 0x140a1, 0x140a4, 0x140a9, 0x140ac, 0x140b1, 0x140b4,
	 0x140b9, 0x140bc, 0x14100, 0x14104, 0x14114, 0x14119, 0x14124, 0x14132,
	 0x14154, 0x1416b, 0x14340, 0x14341, 0x14344, 0x14344, 0x14346, 0x1437c,
	 0x143f0, 0x143f8, 0x143fa, 0x143fe, 0x14400, 0x14404, 0x14406, 0x1440a,
	 0x1440c, 0x14410, 0x14412, 0x14416, 0x14418, 0x1441c, 0x1441e, 0x14422,
	 0x14424, 0x14424, 0x14498, 0x144a0, 0x144a2, 0x144a6, 0x144a8, 0x144ac,
	 0x144ae, 0x144b2, 0x144b4, 0x144b8, 0x144ba, 0x144be, 0x144c0, 0x144c4,
	 0x144c6, 0x144ca, 0x144cc, 0x144cc, 0x14540, 0x14548, 0x1454a, 0x1454e,
	 0x14550, 0x14554, 0x14556, 0x1455a, 0x1455c, 0x14560, 0x14562, 0x14566,
	 0x14568, 0x1456c, 0x1456e, 0x14572, 0x14574, 0x14574, 0x145e8, 0x145f0,
	 0x145f2, 0x145f6, 0x145f8, 0x145fc, 0x145fe, 0x14602, 0x14604, 0x14608,
	 0x1460a, 0x1460e, 0x14610, 0x14614, 0x14616, 0x1461a, 0x1461c, 0x1461c,
	 0x14690, 0x14698, 0x1469a, 0x1469e, 0x146a0, 0x146a4, 0x146a6, 0x146aa,
	 0x146ac, 0x146b0, 0x146b2, 0x146b6, 0x146b8, 0x146bc, 0x146be, 0x146c2,
	 0x146c4, 0x146c4, 0x14738, 0x14740, 0x14742, 0x14746, 0x14748, 0x1474c,
	 0x1474e, 0x14752, 0x14754, 0x14758, 0x1475a, 0x1475e, 0x14760, 0x14764,
	 0x14766, 0x1476a, 0x1476c, 0x1476c, 0x147e0, 0x147e8, 0x147ea, 0x147ee,
	 0x147f0, 0x147f4, 0x147f6, 0x147fa, 0x147fc, 0x14800, 0x14802, 0x14806,
	 0x14808, 0x1480c, 0x1480e, 0x14812, 0x14814, 0x14814, 0x14888, 0x14890,
	 0x14892, 0x14896, 0x14898, 0x1489c, 0x1489e, 0x148a2, 0x148a4, 0x148a8,
	 0x148aa, 0x148ae, 0x148b0, 0x148b4, 0x148b6, 0x148ba, 0x148bc, 0x148bc,
	 0x14930, 0x14938, 0x1493a, 0x1493e, 0x14940, 0x14944, 0x14946, 0x1494a,
	 0x1494c, 0x14950, 0x14952, 0x14956, 0x14958, 0x1495c, 0x1495e, 0x14962,
	 0x14964, 0x14964,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_rscc_rsc_registers), 8));

/*
 * Block   : ['CPR']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 20 (Regs:479)
 */
static const u32 gen8_0_0_cpr_registers[] = {
	0x26800, 0x26805, 0x26808, 0x2680c, 0x26814, 0x26814, 0x2681c, 0x2681c,
	0x26820, 0x26838, 0x26840, 0x26840, 0x26848, 0x26848, 0x26850, 0x26850,
	0x26880, 0x2689f, 0x26980, 0x269b0, 0x269c0, 0x269c8, 0x269e0, 0x269ee,
	0x269fb, 0x269ff, 0x26a02, 0x26a07, 0x26a09, 0x26a0b, 0x26a10, 0x26b0f,
	0x27440, 0x27441, 0x27444, 0x27444, 0x27480, 0x274a2, 0x274ac, 0x274c4,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_0_0_cpr_registers), 8));

static const u32 *gen8_0_0_external_core_regs[] = {
	gen8_0_0_gdpm_lkg_registers,
	gen8_0_0_gpu_cc_ahb2phy_broadcast_swman_registers,
	gen8_0_0_gpu_cc_ahb2phy_swman_registers,
	gen8_0_0_gpu_cc_gpu_cc_reg_registers,
	gen8_0_0_gpu_cc_pll0_cm_pll_taycan_common_registers,
	gen8_0_0_cpr_registers,
};

static struct gen8_reg_list gen8_gmu_gx_registers[] = {
	{ UNSLICE, gen8_0_0_gmugx_registers },
	{ UNSLICE, gen8_0_0_gx_clkctl_ahb2phy_broadcast_swman_registers },
	{ UNSLICE, gen8_0_0_gx_clkctl_ahb2phy_swman_registers },
	{ UNSLICE, gen8_0_0_gx_clkctl_pll0_cm_pll_taycan_common_registers },
	{ UNSLICE, gen8_0_0_gx_clkctl_gx_clkctl_reg_registers },
	{ UNSLICE, gen8_0_0_acd_acd_mnd_registers },
	{ SLICE, gen8_0_0_gmugx_slice_registers },
};

#endif /*_ADRENO_GEN8_0_0_SNAPSHOT_H */
