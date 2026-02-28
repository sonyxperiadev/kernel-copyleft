/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __ADRENO_GEN8_6_0_SNAPSHOT_H
#define __ADRENO_GEN8_6_0_SNAPSHOT_H

#include "adreno_gen8_snapshot.h"
#include "adreno_gen8_0_0_snapshot.h"

static const u32 gen8_6_0_debugbus_blocks[] = {
	DEBUGBUS_GMU_GX_GC_US_I_0,
	DEBUGBUS_DBGC_GC_US_I_0,
	DEBUGBUS_RBBM_GC_US_I_0,
	DEBUGBUS_LARC_GC_US_I_0,
	DEBUGBUS_COM_GC_US_I_0,
	DEBUGBUS_HLSQ_GC_US_I_0,
	DEBUGBUS_CGC_GC_US_I_0,
	DEBUGBUS_VSC_GC_US_I_0_0,
	DEBUGBUS_VSC_GC_US_I_0_1,
	DEBUGBUS_UFC_GC_US_I_0,
	DEBUGBUS_UFC_GC_US_I_1,
	DEBUGBUS_CP_GC_US_I_0_0,
	DEBUGBUS_CP_GC_US_I_0_1,
	DEBUGBUS_CP_GC_US_I_0_2,
	DEBUGBUS_PC_BR_US_I_0,
	DEBUGBUS_PC_BV_US_I_0,
	DEBUGBUS_GPC_BR_US_I_0,
	DEBUGBUS_GPC_BV_US_I_0,
	DEBUGBUS_VPC_BR_US_I_0,
	DEBUGBUS_VPC_BV_US_I_0,
	DEBUGBUS_UCHE_WRAPPER_GC_US_I_0,
	DEBUGBUS_UCHE_GC_US_I_0,
	DEBUGBUS_UCHE_GC_US_I_1,
	DEBUGBUS_CP_GC_S_0_I_0,
	DEBUGBUS_PC_BR_S_0_I_0,
	DEBUGBUS_PC_BV_S_0_I_0,
	DEBUGBUS_TESS_GC_S_0_I_0,
	DEBUGBUS_TSEFE_GC_S_0_I_0,
	DEBUGBUS_TSEBE_GC_S_0_I_0,
	DEBUGBUS_RAS_GC_S_0_I_0,
	DEBUGBUS_LRZ_BR_S_0_I_0,
	DEBUGBUS_LRZ_BV_S_0_I_0,
	DEBUGBUS_VFDP_GC_S_0_I_0,
	DEBUGBUS_GPC_BR_S_0_I_0,
	DEBUGBUS_GPC_BV_S_0_I_0,
	DEBUGBUS_VPCFE_BR_S_0_I_0,
	DEBUGBUS_VPCFE_BV_S_0_I_0,
	DEBUGBUS_VPCBE_BR_S_0_I_0,
	DEBUGBUS_VPCBE_BV_S_0_I_0,
	DEBUGBUS_CCHE_GC_S_0_I_0,
	DEBUGBUS_DBGC_GC_S_0_I_0,
	DEBUGBUS_LARC_GC_S_0_I_0,
	DEBUGBUS_RBBM_GC_S_0_I_0,
	DEBUGBUS_CCRE_GC_S_0_I_0,
	DEBUGBUS_CGC_GC_S_0_I_0,
	DEBUGBUS_GMU_GC_S_0_I_0,
	DEBUGBUS_SLICE_GC_S_0_I_0,
	DEBUGBUS_HLSQ_SPTP_STAR_GC_S_0_I_0,
	DEBUGBUS_USP_GC_S_0_I_0,
	DEBUGBUS_USP_GC_S_0_I_1,
	DEBUGBUS_USPTP_GC_S_0_I_0,
	DEBUGBUS_USPTP_GC_S_0_I_1,
	DEBUGBUS_USPTP_GC_S_0_I_2,
	DEBUGBUS_USPTP_GC_S_0_I_3,
	DEBUGBUS_TP_GC_S_0_I_0,
	DEBUGBUS_TP_GC_S_0_I_1,
	DEBUGBUS_TP_GC_S_0_I_2,
	DEBUGBUS_TP_GC_S_0_I_3,
	DEBUGBUS_RB_GC_S_0_I_0,
	DEBUGBUS_RB_GC_S_0_I_1,
	DEBUGBUS_CCU_GC_S_0_I_0,
	DEBUGBUS_CCU_GC_S_0_I_1,
	DEBUGBUS_HLSQ_GC_S_0_I_0,
	DEBUGBUS_HLSQ_GC_S_0_I_1,
	DEBUGBUS_VFD_GC_S_0_I_0,
	DEBUGBUS_VFD_GC_S_0_I_1,
	DEBUGBUS_CP_GC_S_1_I_0,
	DEBUGBUS_PC_BR_S_1_I_0,
	DEBUGBUS_PC_BV_S_1_I_0,
	DEBUGBUS_TESS_GC_S_1_I_0,
	DEBUGBUS_TSEFE_GC_S_1_I_0,
	DEBUGBUS_TSEBE_GC_S_1_I_0,
	DEBUGBUS_RAS_GC_S_1_I_0,
	DEBUGBUS_LRZ_BR_S_1_I_0,
	DEBUGBUS_LRZ_BV_S_1_I_0,
	DEBUGBUS_VFDP_GC_S_1_I_0,
	DEBUGBUS_GPC_BR_S_1_I_0,
	DEBUGBUS_GPC_BV_S_1_I_0,
	DEBUGBUS_VPCFE_BR_S_1_I_0,
	DEBUGBUS_VPCFE_BV_S_1_I_0,
	DEBUGBUS_VPCBE_BR_S_1_I_0,
	DEBUGBUS_VPCBE_BV_S_1_I_0,
	DEBUGBUS_CCHE_GC_S_1_I_0,
	DEBUGBUS_DBGC_GC_S_1_I_0,
	DEBUGBUS_LARC_GC_S_1_I_0,
	DEBUGBUS_RBBM_GC_S_1_I_0,
	DEBUGBUS_CCRE_GC_S_1_I_0,
	DEBUGBUS_CGC_GC_S_1_I_0,
	DEBUGBUS_GMU_GC_S_1_I_0,
	DEBUGBUS_SLICE_GC_S_1_I_0,
	DEBUGBUS_HLSQ_SPTP_STAR_GC_S_1_I_0,
	DEBUGBUS_USP_GC_S_1_I_0,
	DEBUGBUS_USP_GC_S_1_I_1,
	DEBUGBUS_USPTP_GC_S_1_I_0,
	DEBUGBUS_USPTP_GC_S_1_I_1,
	DEBUGBUS_USPTP_GC_S_1_I_2,
	DEBUGBUS_USPTP_GC_S_1_I_3,
	DEBUGBUS_TP_GC_S_1_I_0,
	DEBUGBUS_TP_GC_S_1_I_1,
	DEBUGBUS_TP_GC_S_1_I_2,
	DEBUGBUS_TP_GC_S_1_I_3,
	DEBUGBUS_RB_GC_S_1_I_0,
	DEBUGBUS_RB_GC_S_1_I_1,
	DEBUGBUS_CCU_GC_S_1_I_0,
	DEBUGBUS_CCU_GC_S_1_I_1,
	DEBUGBUS_HLSQ_GC_S_1_I_0,
	DEBUGBUS_HLSQ_GC_S_1_I_1,
	DEBUGBUS_VFD_GC_S_1_I_0,
	DEBUGBUS_VFD_GC_S_1_I_1,
};

static struct gen8_shader_block gen8_6_0_shader_blocks[] = {
	{ TP0_TMO_DATA, 0x0200, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ TP0_SMO_DATA, 0x0080, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ TP0_MIPMAP_BASE_DATA, 0x0080, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_3, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_1, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_0_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_1_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_2_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_3_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_4_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_5_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_6_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_7_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_CB_RAM, 0x0390, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_13_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_14_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_TAG, 0x0100, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_2, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_TMO_TAG, 0x0080, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_SMO_TAG, 0x0080, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_STATE_DATA, 0x0040, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_HWAVE_RAM, 0x0200, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_L0_INST_BUF, 0x0080, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_8_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_9_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_10_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_11_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_12_DATA, 0x0800, 2, 2, PIPE_BR, USPTP, SLICE, 1},
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
 * Block   : ['BROADCAST', 'GRAS', 'PC']
 * Block   : ['RBBM', 'RDVM', 'UCHE']
 * Block   : ['VFD', 'VPC', 'VSC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 118 (Regs:1153)
 */
static const u32 gen8_6_0_gpu_registers[] = {
	 0x00000, 0x00000, 0x00002, 0x00002, 0x00008, 0x0000d, 0x00010, 0x00011,
	 0x00015, 0x00016, 0x00018, 0x00018, 0x0001a, 0x0001a, 0x0001c, 0x0001c,
	 0x0001e, 0x0001e, 0x00028, 0x0002b, 0x0002d, 0x00039, 0x00040, 0x00053,
	 0x00062, 0x00066, 0x00069, 0x0006e, 0x00071, 0x00072, 0x00074, 0x00074,
	 0x00076, 0x0007c, 0x0007f, 0x0009a, 0x0009d, 0x000af, 0x000b2, 0x000d4,
	 0x000d7, 0x000e2, 0x000e5, 0x000e6, 0x000e9, 0x000f1, 0x000f4, 0x000f6,
	 0x000f9, 0x00108, 0x0010b, 0x0010e, 0x00111, 0x00111, 0x00114, 0x0011c,
	 0x0011f, 0x00121, 0x00125, 0x00125, 0x00127, 0x00127, 0x00129, 0x00129,
	 0x0012b, 0x00131, 0x00134, 0x00138, 0x0013a, 0x0013a, 0x0013c, 0x0013f,
	 0x00142, 0x00150, 0x00153, 0x00155, 0x00158, 0x00159, 0x0015c, 0x0015c,
	 0x00166, 0x00179, 0x0019e, 0x001a3, 0x001b0, 0x002c9, 0x002e2, 0x0036b,
	 0x00380, 0x0039b, 0x003a4, 0x003ab, 0x003b4, 0x003c5, 0x003ce, 0x003cf,
	 0x003e0, 0x003e0, 0x003f0, 0x003f0, 0x00440, 0x00444, 0x00460, 0x00460,
	 0x00c02, 0x00c04, 0x00c06, 0x00c06, 0x00c10, 0x00cd9, 0x00ce0, 0x00d0c,
	 0x00df0, 0x00df4, 0x00e01, 0x00e04, 0x00e06, 0x00e09, 0x00e0e, 0x00e13,
	 0x00e15, 0x00e16, 0x00e20, 0x00e37, 0x0ec00, 0x0ec01, 0x0ec05, 0x0ec05,
	 0x0ec07, 0x0ec07, 0x0ec0a, 0x0ec0a, 0x0ec12, 0x0ec12, 0x0ec26, 0x0ec28,
	 0x0ec2b, 0x0ec2d, 0x0ec2f, 0x0ec2f, 0x0ec40, 0x0ec41, 0x0ec45, 0x0ec45,
	 0x0ec47, 0x0ec47, 0x0ec4a, 0x0ec4a, 0x0ec52, 0x0ec52, 0x0ec66, 0x0ec68,
	 0x0ec6b, 0x0ec6d, 0x0ec6f, 0x0ec6f, 0x0ec80, 0x0ec81, 0x0ec85, 0x0ec85,
	 0x0ec87, 0x0ec87, 0x0ec8a, 0x0ec8a, 0x0ec92, 0x0ec92, 0x0eca6, 0x0eca8,
	 0x0ecab, 0x0ecad, 0x0ecaf, 0x0ecaf, 0x0ecc0, 0x0ecc1, 0x0ecc5, 0x0ecc5,
	 0x0ecc7, 0x0ecc7, 0x0ecca, 0x0ecca, 0x0ecd2, 0x0ecd2, 0x0ece6, 0x0ece8,
	 0x0eceb, 0x0eced, 0x0ecef, 0x0ecef, 0x0ed00, 0x0ed01, 0x0ed05, 0x0ed05,
	 0x0ed07, 0x0ed07, 0x0ed0a, 0x0ed0a, 0x0ed12, 0x0ed12, 0x0ed26, 0x0ed28,
	 0x0ed2b, 0x0ed2d, 0x0ed2f, 0x0ed2f, 0x0ed40, 0x0ed41, 0x0ed45, 0x0ed45,
	 0x0ed47, 0x0ed47, 0x0ed4a, 0x0ed4a, 0x0ed52, 0x0ed52, 0x0ed66, 0x0ed68,
	 0x0ed6b, 0x0ed6d, 0x0ed6f, 0x0ed6f, 0x0ed80, 0x0ed81, 0x0ed85, 0x0ed85,
	 0x0ed87, 0x0ed87, 0x0ed8a, 0x0ed8a, 0x0ed92, 0x0ed92, 0x0eda6, 0x0eda8,
	 0x0edab, 0x0edad, 0x0edaf, 0x0edaf,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_gpu_registers), 8));

/*
 * Block   : ['BROADCAST', 'GRAS', 'PC']
 * Block   : ['RBBM', 'RDVM', 'UCHE']
 * Block   : ['VFD', 'VPC', 'VSC']
 * REGION  : SLICE
 * Pipeline: PIPE_NONE
 * pairs   : 12 (Regs:86)
 */
static const u32 gen8_6_0_gpu_slice_registers[] = {
	 0x00500, 0x00500, 0x00583, 0x00584, 0x00586, 0x0058b, 0x0058f, 0x00599,
	 0x005a0, 0x005b3, 0x005c0, 0x005c0, 0x005c2, 0x005c6, 0x005e0, 0x005e3,
	 0x005ec, 0x005ec, 0x00f01, 0x00f02, 0x00f04, 0x00f0c, 0x00f20, 0x00f37,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_gpu_slice_registers), 8));

/*
 * Block   : ['GMUGX']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 160 (Regs:616)
 */
static const u32 gen8_6_0_gmu_registers[] = {
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
	 0x20340, 0x20350, 0x20354, 0x2035b, 0x20360, 0x20367, 0x20370, 0x20377,
	 0x23801, 0x23801, 0x23803, 0x23803, 0x23805, 0x23805, 0x23807, 0x23807,
	 0x23809, 0x23809, 0x2380b, 0x2380b, 0x2380d, 0x2380d, 0x2380f, 0x2380f,
	 0x23811, 0x23811, 0x23813, 0x23813, 0x23815, 0x23815, 0x23817, 0x23817,
	 0x23819, 0x23819, 0x2381b, 0x2381b, 0x2381d, 0x2381d, 0x2381f, 0x23820,
	 0x23822, 0x23822, 0x23824, 0x23824, 0x23826, 0x23826, 0x23828, 0x23828,
	 0x2382a, 0x2382a, 0x2382c, 0x2382c, 0x2382e, 0x2382e, 0x23830, 0x23830,
	 0x23832, 0x23832, 0x23834, 0x23834, 0x23836, 0x23836, 0x23838, 0x23838,
	 0x2383a, 0x2383a, 0x2383c, 0x2383c, 0x2383e, 0x2383e, 0x23840, 0x23847,
	 0x23b00, 0x23b01, 0x23b03, 0x23b03, 0x23b05, 0x23b0e, 0x23b10, 0x23b13,
	 0x23b15, 0x23b16, 0x23b28, 0x23b28, 0x23b30, 0x23b30,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_gmu_registers), 8));

/*
 * Block   : ['BROADCAST', 'CX_DBGC', 'CX_MISC', 'DBGC', 'GBIF', 'GMUAO']
 * Block   : ['GMUCX', 'GMUGX', 'GRAS', 'PC', 'RBBM']
 * Block   : ['RDVM', 'UCHE', 'VFD', 'VPC', 'VSC']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 21 (Regs:301)
 */
static const u32 gen8_6_0_non_context_slice_pipe_br_registers[] = {
	 0x08600, 0x08602, 0x08610, 0x08613, 0x08700, 0x08704, 0x08710, 0x08713,
	 0x08720, 0x08723, 0x08730, 0x08733, 0x08740, 0x08744, 0x09680, 0x09681,
	 0x09690, 0x0969b, 0x09740, 0x09745, 0x09750, 0x0975b, 0x09770, 0x097ef,
	 0x09f00, 0x09f0f, 0x09f20, 0x09f23, 0x09f30, 0x09f31, 0x0a600, 0x0a600,
	 0x0a603, 0x0a603, 0x0a610, 0x0a61f, 0x0a630, 0x0a632, 0x0a638, 0x0a63c,
	 0x0a640, 0x0a67f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_non_context_slice_pipe_br_registers), 8));

/*
 * Block   : ['BROADCAST', 'CX_DBGC', 'CX_MISC', 'DBGC', 'GBIF', 'GMUAO']
 * Block   : ['GMUCX', 'GMUGX', 'GRAS', 'PC', 'RBBM']
 * Block   : ['RDVM', 'UCHE', 'VFD', 'VPC', 'VSC']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_NONE
 * pairs   : 21 (Regs:301)
 */
static const u32 gen8_6_0_non_context_slice_pipe_bv_registers[] = {
	 0x08600, 0x08602, 0x08610, 0x08613, 0x08700, 0x08704, 0x08710, 0x08713,
	 0x08720, 0x08723, 0x08730, 0x08733, 0x08740, 0x08744, 0x09680, 0x09681,
	 0x09690, 0x0969b, 0x09740, 0x09745, 0x09750, 0x0975b, 0x09770, 0x097ef,
	 0x09f00, 0x09f0f, 0x09f20, 0x09f23, 0x09f30, 0x09f31, 0x0a600, 0x0a600,
	 0x0a603, 0x0a603, 0x0a610, 0x0a61f, 0x0a630, 0x0a632, 0x0a638, 0x0a63c,
	 0x0a640, 0x0a67f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_non_context_slice_pipe_bv_registers), 8));

static struct gen8_cluster_registers gen8_6_0_mvc_clusters[] = {
	{ CLUSTER_NONE, UNSLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_non_context_pipe_br_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_6_0_non_context_slice_pipe_br_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_0_0_non_context_pipe_bv_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_6_0_non_context_slice_pipe_bv_registers,  },
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

static struct gen8_reg_list gen8_6_0_reg_list[] = {
	{ UNSLICE, gen8_6_0_gpu_registers },
	{ SLICE, gen8_6_0_gpu_slice_registers },
	{ UNSLICE, gen8_0_0_dbgc_registers },
	{ SLICE, gen8_0_0_dbgc_slice_registers },
	{ UNSLICE, gen8_0_0_cx_dbgc_registers },
	{ UNSLICE, NULL},
};

/*
 * Block   : ['GPU_CC_GPU_CC_REG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 30 (Regs:128)
 */
static const u32 gen8_6_0_gpu_cc_gpu_cc_reg_registers[] = {
	 0x25400, 0x25404, 0x25800, 0x25804, 0x25c00, 0x25c04, 0x26000, 0x26004,
	 0x26400, 0x26406, 0x26415, 0x26418, 0x2641c, 0x2641d, 0x2641f, 0x26437,
	 0x26439, 0x2643a, 0x2643c, 0x2643f, 0x26443, 0x26444, 0x26478, 0x2647a,
	 0x26489, 0x2648a, 0x2649c, 0x2649e, 0x264a0, 0x264a1, 0x264c5, 0x264c7,
	 0x264e8, 0x264ea, 0x264f9, 0x264fc, 0x2650b, 0x2650b, 0x2651c, 0x2651e,
	 0x26540, 0x2654b, 0x26554, 0x26556, 0x26558, 0x2655c, 0x2655e, 0x2655f,
	 0x26563, 0x26563, 0x2656d, 0x26573, 0x26576, 0x26576, 0x26578, 0x2657a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_gpu_cc_gpu_cc_reg_registers), 8));

/*
 * Block   : ['GPU_CC_PLL0_CM_PLL_LUCID_OLE']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:16)
 */
static const u32 gen8_6_0_gpu_cc_pll0_cm_pll_lucid_ole_registers[] = {
	 0x24000, 0x2400f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_gpu_cc_pll0_cm_pll_lucid_ole_registers), 8));

/*
 * Block   : ['ACD_ACD']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 10 (Regs:53)
 */
static const u32 gen8_6_0_acd_acd_registers[] = {
	 0x1a400, 0x1a416, 0x1a420, 0x1a42d, 0x1a430, 0x1a431, 0x1a435, 0x1a435,
	 0x1a437, 0x1a437, 0x1a43a, 0x1a43a, 0x1a442, 0x1a442, 0x1a456, 0x1a458,
	 0x1a45b, 0x1a45d, 0x1a45f, 0x1a462,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_acd_acd_registers), 8));

/*
 * Block   : ['GX_CLKCTL_GX_CLKCTL_REG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 12 (Regs:85)
 */
static const u32 gen8_6_0_gx_clkctl_gx_clkctl_reg_registers[] = {
	 0x1a000, 0x1a004, 0x1a008, 0x1a012, 0x1a014, 0x1a014, 0x1a017, 0x1a017,
	 0x1a019, 0x1a01c, 0x1a022, 0x1a022, 0x1a024, 0x1a029, 0x1a03f, 0x1a05d,
	 0x1a060, 0x1a063, 0x1a065, 0x1a066, 0x1a068, 0x1a076, 0x1a078, 0x1a07b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_gx_clkctl_gx_clkctl_reg_registers), 8));

/*
 * Block   : ['GX_CLKCTL_PLL0_CM_PLL_LUCID_OLE']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:16)
 */
static const u32 gen8_6_0_gx_clkctl_pll0_cm_pll_lucid_ole_registers[] = {
	 0x19000, 0x1900f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_gx_clkctl_pll0_cm_pll_lucid_ole_registers), 8));

/*
 * Block   : ['CPR']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 22 (Regs:504)
 */
static const u32 gen8_6_0_cpr_registers[] = {
	 0x26800, 0x26805, 0x26808, 0x2680d, 0x26814, 0x26815, 0x2681c, 0x2681c,
	 0x26820, 0x26839, 0x26840, 0x26841, 0x26848, 0x26849, 0x26850, 0x26851,
	 0x26880, 0x268a2, 0x26980, 0x269b0, 0x269c0, 0x269c2, 0x269c6, 0x269c8,
	 0x269e0, 0x269ee, 0x269fb, 0x269ff, 0x26a02, 0x26a07, 0x26a09, 0x26a0b,
	 0x26a10, 0x26b0f, 0x27440, 0x27441, 0x27444, 0x27444, 0x27480, 0x274a2,
	 0x274ac, 0x274c4, 0x274c8, 0x274da,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_6_0_cpr_registers), 8));

static const u32 *gen8_6_0_external_core_regs[] = {
	gen8_0_0_gdpm_lkg_registers,
	gen8_0_0_gpu_cc_ahb2phy_broadcast_swman_registers,
	gen8_0_0_gpu_cc_ahb2phy_swman_registers,
	gen8_6_0_gpu_cc_gpu_cc_reg_registers,
	gen8_6_0_gpu_cc_pll0_cm_pll_lucid_ole_registers,
	gen8_6_0_cpr_registers,
};

static struct gen8_reg_list gen8_6_0_gmu_gx_registers[] = {
	{ UNSLICE, gen8_0_0_gmugx_registers },
	{ UNSLICE, gen8_0_0_gx_clkctl_ahb2phy_broadcast_swman_registers },
	{ UNSLICE, gen8_0_0_gx_clkctl_ahb2phy_swman_registers },
	{ UNSLICE, gen8_6_0_gx_clkctl_pll0_cm_pll_lucid_ole_registers },
	{ UNSLICE, gen8_6_0_gx_clkctl_gx_clkctl_reg_registers },
	{ UNSLICE, gen8_6_0_acd_acd_registers },
	{ SLICE, gen8_0_0_gmugx_slice_registers },
};
#endif /*_ADRENO_GEN8_6_0_SNAPSHOT_H */
