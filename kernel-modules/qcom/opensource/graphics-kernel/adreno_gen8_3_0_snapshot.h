/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __ADRENO_GEN8_3_0_SNAPSHOT_H
#define __ADRENO_GEN8_3_0_SNAPSHOT_H

#include "adreno_gen8_snapshot.h"
#include "adreno_gen8_0_0_snapshot.h"

static const u32 gen8_3_0_debugbus_blocks[] = {
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
	DEBUGBUS_CP_GC_S_0_I_0,
	DEBUGBUS_PC_BR_S_0_I_0,
	DEBUGBUS_PC_BV_S_0_I_0,
	DEBUGBUS_TESS_GC_S_0_I_0,
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
	DEBUGBUS_DBGC_GC_S_0_I_0,
	DEBUGBUS_LARC_GC_S_0_I_0,
	DEBUGBUS_RBBM_GC_S_0_I_0,
	DEBUGBUS_CCRE_GC_S_0_I_0,
	DEBUGBUS_CGC_GC_S_0_I_0,
	DEBUGBUS_GMU_GC_S_0_I_0,
	DEBUGBUS_SLICE_GC_S_0_I_0,
	DEBUGBUS_HLSQ_SPTP_STAR_GC_S_0_I_0,
	DEBUGBUS_USP_GC_S_0_I_0,
	DEBUGBUS_USPTP_GC_S_0_I_0,
	DEBUGBUS_USPTP_GC_S_0_I_1,
	DEBUGBUS_TP_GC_S_0_I_0,
	DEBUGBUS_TP_GC_S_0_I_1,
	DEBUGBUS_RB_GC_S_0_I_0,
	DEBUGBUS_CCU_GC_S_0_I_0,
	DEBUGBUS_HLSQ_GC_S_0_I_0,
	DEBUGBUS_VFD_GC_S_0_I_0,
};

static struct gen8_shader_block gen8_3_0_shader_blocks[] = {
	{ TP0_TMO_DATA, 0x0200, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ TP0_SMO_DATA, 0x0080, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ TP0_MIPMAP_BASE_DATA, 0x0080, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_3, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_1, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_0_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_1_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_2_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_3_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_4_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_5_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_6_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_7_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_CB_RAM, 0x0390, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_13_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_14_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_TAG, 0x0100, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_INST_DATA_2, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_TMO_TAG, 0x0080, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_SMO_TAG, 0x0080, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_STATE_DATA, 0x0040, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_HWAVE_RAM, 0x0200, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_L0_INST_BUF, 0x0080, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_8_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_9_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_10_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_11_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
	{ SP_LB_12_DATA, 0x0800, 1, 1, PIPE_BR, USPTP, SLICE, 1},
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
	{ HLSQ_CHUNK_CVS_RAM_TAG, 0x0040, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CVS_RAM_TAG, 0x0040, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_CHUNK_CPS_RAM_TAG, 0x0040, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_ICB_CVS_CB_BASE_TAG, 0x0010, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_ICB_CVS_CB_BASE_TAG, 0x0010, 1, 1, PIPE_BV, HLSQ_STATE, UNSLICE, 1},
	{ HLSQ_ICB_CPS_CB_BASE_TAG, 0x0010, 1, 1, PIPE_BR, HLSQ_STATE, UNSLICE, 1},
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
 * Block   : ['GMUAO', 'GMUCX', 'GMUCX_RAM']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 160 (Regs:616)
 */
static const u32 gen8_3_0_gmu_registers[] = {
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
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gmu_registers), 8));

/*
 * Block   : ['GMUGX']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 17 (Regs:170)
 */
static const u32 gen8_3_0_gmugx_registers[] = {
	 0x0dc00, 0x0dc0d, 0x0dc10, 0x0dc11, 0x0dc13, 0x0dc15, 0x0dc18, 0x0dc1a,
	 0x0dc1c, 0x0dc23, 0x0dc26, 0x0dc2b, 0x0dc2e, 0x0dc2f, 0x0dc40, 0x0dc42,
	 0x0dc60, 0x0dc7f, 0x0dc88, 0x0dc90, 0x0dc98, 0x0dc99, 0x0dca0, 0x0dcbf,
	 0x0dcc8, 0x0dcd0, 0x0dcd8, 0x0dcd9, 0x0dce0, 0x0dcff, 0x0dd08, 0x0dd10,
	 0x0dd18, 0x0dd19,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gmugx_registers), 8));

/*
 * Block   : ['GMUGX']
 * REGION  : SLICE
 * Pipeline: PIPE_NONE
 * pairs   : 19 (Regs:224)
 */
static const u32 gen8_3_0_gmugx_slice_registers[] = {
	 0x0e400, 0x0e401, 0x0e404, 0x0e404, 0x0e406, 0x0e408, 0x0e40a, 0x0e40a,
	 0x0e40e, 0x0e42f, 0x0e438, 0x0e440, 0x0e448, 0x0e449, 0x0e490, 0x0e4af,
	 0x0e4b8, 0x0e4c0, 0x0e4c8, 0x0e4c9, 0x0e4d0, 0x0e4ef, 0x0e4f8, 0x0e500,
	 0x0e508, 0x0e509, 0x0e510, 0x0e52f, 0x0e538, 0x0e540, 0x0e548, 0x0e549,
	 0x0e590, 0x0e5af, 0x0e5b8, 0x0e5c0, 0x0e5c8, 0x0e5c9,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gmugx_slice_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: CP_PIPE_NONE
 * Cluster : CLUSTER_NONE
 * pairs   : 16 (Regs:300)
 */
static const u32 gen8_3_0_cp_cp_pipe_none_registers[] = {
	 0x00800, 0x0080a, 0x00813, 0x0081e, 0x00820, 0x0082d, 0x00838, 0x0083e,
	 0x00840, 0x00847, 0x0084b, 0x0084c, 0x00850, 0x0088f, 0x008b5, 0x008b6,
	 0x008c0, 0x008cb, 0x008d0, 0x008e4, 0x008e7, 0x008ee, 0x008fa, 0x008fb,
	 0x00928, 0x00929, 0x00958, 0x0095b, 0x00980, 0x009ff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_cp_cp_pipe_none_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: CP_PIPE_LPAC
 * Cluster : CLUSTER_NONE
 * pairs   : 3 (Regs:13)
 */
static const u32 gen8_3_0_cp_cp_pipe_lpac_registers[] = {
	 0x00830, 0x00837, 0x008b3, 0x008b4, 0x008b7, 0x008b9,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_cp_cp_pipe_lpac_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: CP_PIPE_AQE0
 * Cluster : CLUSTER_NONE
 * pairs   : 2 (Regs:5)
 */
static const u32 gen8_3_0_cp_cp_pipe_aqe0_registers[] = {
	 0x008b3, 0x008b4, 0x008b7, 0x008b9,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_cp_cp_pipe_aqe0_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: CP_PIPE_AQE1
 * Cluster : CLUSTER_NONE
 * pairs   : 2 (Regs:5)
 */
static const u32 gen8_3_0_cp_cp_pipe_aqe1_registers[] = {
	 0x008b3, 0x008b4, 0x008b7, 0x008b9,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_cp_cp_pipe_aqe1_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: CP_PIPE_DDE_BR
 * Cluster : CLUSTER_NONE
 * pairs   : 3 (Regs:7)
 */
static const u32 gen8_3_0_cp_cp_pipe_dde_br_registers[] = {
	 0x008b3, 0x008b4, 0x008b7, 0x008b9, 0x008fe, 0x008ff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_cp_cp_pipe_dde_br_registers), 8));

/*
 * Block   : ['CP']
 * REGION  : UNSLICE
 * Pipeline: CP_PIPE_DDE_BV
 * Cluster : CLUSTER_NONE
 * pairs   : 3 (Regs:7)
 */
static const u32 gen8_3_0_cp_cp_pipe_dde_bv_registers[] = {
	 0x008b3, 0x008b4, 0x008b7, 0x008b9, 0x008fe, 0x008ff,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_cp_cp_pipe_dde_bv_registers), 8));

/*
 * Block   : ['SP']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * Cluster : CLUSTER_NONE
 * Location: HLSQ_DP_STR
 * pairs   : 5 (Regs:18)
 */
static const u32 gen8_3_0_non_context_sp_pipe_none_hlsq_dp_str_registers[] = {
	 0x0ae05, 0x0ae05, 0x0ae60, 0x0ae65, 0x0ae6b, 0x0ae6c, 0x0ae73, 0x0ae75,
	 0x0aec0, 0x0aec5,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_non_context_sp_pipe_none_hlsq_dp_str_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_VPC_VS
 * pairs   : 9 (Regs:238)
 */
static const u32 gen8_3_0_gras_slice_pipe_br_cluster_vpc_vs_registers[] = {
	 0x08200, 0x08213, 0x08220, 0x08225, 0x08228, 0x0822d, 0x08230, 0x0823b,
	 0x08240, 0x0825f, 0x08270, 0x0828f, 0x0829f, 0x082b7, 0x082d0, 0x0832f,
	 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gras_slice_pipe_br_cluster_vpc_vs_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BR
 * Cluster : CLUSTER_GRAS
 * pairs   : 14 (Regs:332)
 */
static const u32 gen8_3_0_gras_slice_pipe_br_cluster_gras_registers[] = {
	 0x08080, 0x08080, 0x08086, 0x08092, 0x080c0, 0x080df, 0x08101, 0x08110,
	 0x08130, 0x0814f, 0x08200, 0x08213, 0x08220, 0x08225, 0x08228, 0x0822d,
	 0x08230, 0x0823b, 0x08240, 0x0825f, 0x08270, 0x0828f, 0x0829f, 0x082b7,
	 0x082d0, 0x0832f, 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gras_slice_pipe_br_cluster_gras_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_VPC_VS
 * pairs   : 9 (Regs:238)
 */
static const u32 gen8_3_0_gras_slice_pipe_bv_cluster_vpc_vs_registers[] = {
	 0x08200, 0x08213, 0x08220, 0x08225, 0x08228, 0x0822d, 0x08230, 0x0823b,
	 0x08240, 0x0825f, 0x08270, 0x0828f, 0x0829f, 0x082b7, 0x082d0, 0x0832f,
	 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gras_slice_pipe_bv_cluster_vpc_vs_registers), 8));

/*
 * Block   : ['GRAS']
 * REGION  : SLICE
 * Pipeline: PIPE_BV
 * Cluster : CLUSTER_GRAS
 * pairs   : 14 (Regs:332)
 */
static const u32 gen8_3_0_gras_slice_pipe_bv_cluster_gras_registers[] = {
	 0x08080, 0x08080, 0x08086, 0x08092, 0x080c0, 0x080df, 0x08101, 0x08110,
	 0x08130, 0x0814f, 0x08200, 0x08213, 0x08220, 0x08225, 0x08228, 0x0822d,
	 0x08230, 0x0823b, 0x08240, 0x0825f, 0x08270, 0x0828f, 0x0829f, 0x082b7,
	 0x082d0, 0x0832f, 0x08500, 0x08508,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gras_slice_pipe_bv_cluster_gras_registers), 8));

static struct gen8_cluster_registers gen8_3_0_cp_clusters[] = {
	{ CLUSTER_NONE, UNSLICE, PIPE_NONE, STATE_NON_CONTEXT,
		gen8_3_0_cp_cp_pipe_none_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_br_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BR, STATE_NON_CONTEXT,
		gen8_0_0_cp_slice_cp_pipe_br_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_0_0_cp_cp_pipe_bv_registers,  },
	{ CLUSTER_NONE, SLICE, PIPE_BV, STATE_NON_CONTEXT,
		gen8_0_0_cp_slice_cp_pipe_bv_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_LPAC, STATE_NON_CONTEXT,
		gen8_3_0_cp_cp_pipe_lpac_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_AQE0, STATE_NON_CONTEXT,
		gen8_3_0_cp_cp_pipe_aqe0_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_AQE1, STATE_NON_CONTEXT,
		gen8_3_0_cp_cp_pipe_aqe1_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_DDE_BR, STATE_NON_CONTEXT,
		gen8_3_0_cp_cp_pipe_dde_br_registers,  },
	{ CLUSTER_NONE, UNSLICE, PIPE_DDE_BV, STATE_NON_CONTEXT,
		gen8_3_0_cp_cp_pipe_dde_bv_registers,  },
};
static struct gen8_cluster_registers gen8_3_0_mvc_clusters[] = {
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
		gen8_3_0_gras_slice_pipe_br_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_3_0_gras_slice_pipe_br_cluster_vpc_vs_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen8_3_0_gras_slice_pipe_br_cluster_gras_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen8_3_0_gras_slice_pipe_br_cluster_gras_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_3_0_gras_slice_pipe_bv_cluster_vpc_vs_registers,  },
	{ CLUSTER_VPC_VS, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_3_0_gras_slice_pipe_bv_cluster_vpc_vs_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen8_3_0_gras_slice_pipe_bv_cluster_gras_registers,  },
	{ CLUSTER_GRAS, SLICE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen8_3_0_gras_slice_pipe_bv_cluster_gras_registers,  },
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
static struct gen8_sptp_cluster_registers gen8_3_0_sptp_clusters[] = {
	{ CLUSTER_NONE, UNSLICE, 1, 2, SP_NCTX_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_non_context_sp_pipe_br_hlsq_state_registers, 0xae00},
	{ CLUSTER_NONE, UNSLICE, 1, 2, SP_NCTX_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_non_context_sp_pipe_br_sp_top_registers, 0xae00},
	{ CLUSTER_NONE, UNSLICE, 1, 2, SP_NCTX_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_non_context_sp_pipe_br_sp_top_registers, 0xae00},
	{ CLUSTER_NONE, UNSLICE, 1, 2, SP_NCTX_REG, PIPE_BR, 0, HLSQ_DP_STR,
		gen8_3_0_non_context_sp_pipe_none_hlsq_dp_str_registers, 0xae00},
	{ CLUSTER_NONE, UNSLICE, 1, 2, TP0_NCTX_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_non_context_tpl1_pipe_br_usptp_registers, 0xb600},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, USPTP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX0_3D_CVS_REG, PIPE_BV, 0, USPTP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, SP_CTX1_3D_CVS_REG, PIPE_BV, 1, USPTP,
		gen8_0_0_sp_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_state_shared_const_registers,
		0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, SP_TOP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, USPTP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, USPTP,
		gen8_0_0_sp_slice_pipe_lpac_cluster_sp_ps_usptp_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_state_shared_const_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX2_3D_CPS_REG, PIPE_BR, 2, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX2_3D_CPS_REG, PIPE_BR, 2, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX3_3D_CPS_REG, PIPE_BR, 3, HLSQ_DP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_hlsq_dp_registers, 0xa800},
	{ CLUSTER_SP_PS, SLICE, 1, 2, SP_CTX3_3D_CPS_REG, PIPE_BR, 3, SP_TOP,
		gen8_0_0_sp_slice_pipe_br_cluster_sp_ps_sp_top_cctx_registers, 0xa800},
	{ CLUSTER_SP_VS, SLICE, 1, 2, TP0_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_VS, SLICE, 1, 2, TP0_CTX0_3D_CVS_REG, PIPE_BV, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_VS, SLICE, 1, 2, TP0_CTX1_3D_CVS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_VS, SLICE, 1, 2, TP0_CTX1_3D_CVS_REG, PIPE_BV, 1, USPTP,
		gen8_0_0_tpl1_slice_pipe_bv_cluster_sp_vs_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 1, 2, TP0_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 1, 2, TP0_CTX0_3D_CPS_REG, PIPE_LPAC, 0, USPTP,
		gen8_0_0_tpl1_slice_pipe_lpac_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 1, 2, TP0_CTX1_3D_CPS_REG, PIPE_BR, 1, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 1, 2, TP0_CTX2_3D_CPS_REG, PIPE_BR, 2, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
	{ CLUSTER_SP_PS, SLICE, 1, 2, TP0_CTX3_3D_CPS_REG, PIPE_BR, 3, USPTP,
		gen8_0_0_tpl1_slice_pipe_br_cluster_sp_ps_usptp_registers, 0xb000},
};
/*
 * Before dumping the CP MVC
 * Program CP_APERTURE_CNTL_* with pipeID={CP_PIPE}
 * Then dump corresponding {Register_PIPE}
 */
static struct gen8_cp_indexed_reg gen8_3_0_cp_indexed_reg_list[] = {
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_BR, 0x00040},
	{ GEN8_CP_SQE_STAT_ADDR_PIPE, GEN8_CP_SQE_STAT_DATA_PIPE, UNSLICE, PIPE_BV, 0x00040},
	{ GEN8_CP_DRAW_STATE_ADDR_PIPE, GEN8_CP_DRAW_STATE_DATA_PIPE, UNSLICE, PIPE_BR, 0x00200},
	{ GEN8_CP_DRAW_STATE_ADDR_PIPE, GEN8_CP_DRAW_STATE_DATA_PIPE, UNSLICE, PIPE_BV, 0x00200},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_BR, 0x00800},
	{ GEN8_CP_ROQ_DBG_ADDR_PIPE, GEN8_CP_ROQ_DBG_DATA_PIPE, UNSLICE, PIPE_BV, 0x00800},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
	UNSLICE, PIPE_BR, 0x08000},
	{ GEN8_CP_SQE_UCODE_DBG_ADDR_PIPE, GEN8_CP_SQE_UCODE_DBG_DATA_PIPE,
	UNSLICE, PIPE_BV, 0x08000},
	{ GEN8_CP_RESOURCE_TABLE_DBG_ADDR_BV, GEN8_CP_RESOURCE_TABLE_DBG_DATA_BV,
	UNSLICE, PIPE_NONE, 0x04100},
	{ GEN8_CP_FIFO_DBG_ADDR_DDE_PIPE, GEN8_CP_FIFO_DBG_DATA_DDE_PIPE,
	UNSLICE, PIPE_DDE_BR, 0x01100},
	{ GEN8_CP_FIFO_DBG_ADDR_DDE_PIPE, GEN8_CP_FIFO_DBG_DATA_DDE_PIPE,
	UNSLICE, PIPE_DDE_BV, 0x01100},
};

static struct gen8_reg_list gen8_3_0_gmu_gx_regs[] = {
	{ UNSLICE, gen8_3_0_gmugx_registers },
	{ SLICE, gen8_3_0_gmugx_slice_registers },
};

/*
 * Block   : ['GDPM_LKG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 9 (Regs:26)
 */
static const u32 gen8_3_0_gdpm_lkg_registers[] = {
	 0x21c00, 0x21c00, 0x21c08, 0x21c09, 0x21c0e, 0x21c0f, 0x21c4f, 0x21c50,
	 0x21c52, 0x21c52, 0x21c54, 0x21c56, 0x21c58, 0x21c5a, 0x21c5c, 0x21c60,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gdpm_lkg_registers), 8));

/*
 * Block   : ['GPU_CC_GPU_CC_REG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 28 (Regs:130)
 */
static const u32 gen8_3_0_gpu_cc_gpu_cc_reg_registers[] = {
	 0x25400, 0x25404, 0x25800, 0x25804, 0x25c00, 0x25c04, 0x26000, 0x26004,
	 0x26400, 0x26406, 0x26415, 0x2641d, 0x2641f, 0x26437, 0x26439, 0x2643b,
	 0x2643d, 0x2643f, 0x26443, 0x26444, 0x26478, 0x2647a, 0x26489, 0x2648a,
	 0x2649c, 0x2649e, 0x264a0, 0x264a1, 0x264c5, 0x264c7, 0x264e8, 0x264ea,
	 0x264f9, 0x264fc, 0x2650b, 0x2650b, 0x2651c, 0x2651e, 0x26540, 0x2654b,
	 0x26554, 0x26556, 0x26558, 0x2655c, 0x2655e, 0x2655f, 0x26563, 0x26563,
	 0x2656d, 0x26573, 0x26576, 0x26576, 0x26578, 0x2657a,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gpu_cc_gpu_cc_reg_registers), 8));

/*
 * Block   : ['GPU_CC_PLL0_CM_PLL_LUCID_OLE']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:16)
 */
static const u32 gen8_3_0_gpu_cc_pll0_cm_pll_lucid_ole_registers[] = {
	 0x24000, 0x2400f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gpu_cc_pll0_cm_pll_lucid_ole_registers), 8));

/*
 * Block   : ['ACD_ACD']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 10 (Regs:53)
 */
static const u32 gen8_3_0_acd_acd_mnd_registers[] = {
	 0x1a400, 0x1a416, 0x1a420, 0x1a42d, 0x1a430, 0x1a431, 0x1a435, 0x1a435,
	 0x1a437, 0x1a437, 0x1a43a, 0x1a43a, 0x1a442, 0x1a442, 0x1a456, 0x1a458,
	 0x1a45b, 0x1a45d, 0x1a45f, 0x1a462,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_acd_acd_mnd_registers), 8));

/*
 * Block   : ['GX_CLKCTL_GX_CLKCTL_REG']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 12 (Regs:82)
 */
static const u32 gen8_3_0_gx_clkctl_gx_clkctl_reg_registers[] = {
	 0x1a000, 0x1a004, 0x1a008, 0x1a012, 0x1a014, 0x1a014, 0x1a017, 0x1a017,
	 0x1a019, 0x1a019, 0x1a022, 0x1a022, 0x1a024, 0x1a029, 0x1a03f, 0x1a05d,
	 0x1a060, 0x1a063, 0x1a065, 0x1a066, 0x1a068, 0x1a076, 0x1a078, 0x1a07b,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gx_clkctl_gx_clkctl_reg_registers), 8));

/*
 * Block   : ['GX_CLKCTL_PLL0_CM_PLL_LUCID_OLE']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 1 (Regs:16)
 */
static const u32 gen8_3_0_gx_clkctl_pll0_cm_pll_lucid_ole_registers[] = {
	 0x19000, 0x1900f,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_gx_clkctl_pll0_cm_pll_lucid_ole_registers), 8));

/*
 * Block   : ['RSCC_RSC']
 * REGION  : UNSLICE
 * Pipeline: PIPE_NONE
 * pairs   : 99 (Regs:598)
 */
static const u32 gen8_3_0_rscc_rsc_registers[] = {
	 0x14000, 0x14034, 0x14036, 0x14036, 0x14040, 0x14042, 0x14044, 0x14045,
	 0x14047, 0x14047, 0x14080, 0x14084, 0x14089, 0x1408c, 0x14091, 0x14094,
	 0x14099, 0x1409c, 0x140a1, 0x140a4, 0x140a9, 0x140ac, 0x14100, 0x14104,
	 0x14114, 0x14119, 0x14124, 0x14132, 0x14154, 0x1416b, 0x14340, 0x14341,
	 0x14344, 0x14344, 0x14346, 0x1437c, 0x143f0, 0x143f8, 0x143fa, 0x143fe,
	 0x14400, 0x14404, 0x14406, 0x1440a, 0x1440c, 0x14410, 0x14412, 0x14416,
	 0x14418, 0x1441c, 0x1441e, 0x14422, 0x14424, 0x14424, 0x14498, 0x144a0,
	 0x144a2, 0x144a6, 0x144a8, 0x144ac, 0x144ae, 0x144b2, 0x144b4, 0x144b8,
	 0x144ba, 0x144be, 0x144c0, 0x144c4, 0x144c6, 0x144ca, 0x144cc, 0x144cc,
	 0x14540, 0x14548, 0x1454a, 0x1454e, 0x14550, 0x14554, 0x14556, 0x1455a,
	 0x1455c, 0x14560, 0x14562, 0x14566, 0x14568, 0x1456c, 0x1456e, 0x14572,
	 0x14574, 0x14574, 0x145e8, 0x145f0, 0x145f2, 0x145f6, 0x145f8, 0x145fc,
	 0x145fe, 0x14602, 0x14604, 0x14608, 0x1460a, 0x1460e, 0x14610, 0x14614,
	 0x14616, 0x1461a, 0x1461c, 0x1461c, 0x14690, 0x14698, 0x1469a, 0x1469e,
	 0x146a0, 0x146a4, 0x146a6, 0x146aa, 0x146ac, 0x146b0, 0x146b2, 0x146b6,
	 0x146b8, 0x146bc, 0x146be, 0x146c2, 0x146c4, 0x146c4, 0x14738, 0x14740,
	 0x14742, 0x14746, 0x14748, 0x1474c, 0x1474e, 0x14752, 0x14754, 0x14758,
	 0x1475a, 0x1475e, 0x14760, 0x14764, 0x14766, 0x1476a, 0x1476c, 0x1476c,
	 0x147e0, 0x147e8, 0x147ea, 0x147ee, 0x147f0, 0x147f4, 0x147f6, 0x147fa,
	 0x147fc, 0x14800, 0x14802, 0x14806, 0x14808, 0x1480c, 0x1480e, 0x14812,
	 0x14814, 0x14814, 0x14888, 0x14890, 0x14892, 0x14896, 0x14898, 0x1489c,
	 0x1489e, 0x148a2, 0x148a4, 0x148a8, 0x148aa, 0x148ae, 0x148b0, 0x148b4,
	 0x148b6, 0x148ba, 0x148bc, 0x148bc, 0x14930, 0x14938, 0x1493a, 0x1493e,
	 0x14940, 0x14944, 0x14946, 0x1494a, 0x1494c, 0x14950, 0x14952, 0x14956,
	 0x14958, 0x1495c, 0x1495e, 0x14962, 0x14964, 0x14964,
	 UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen8_3_0_rscc_rsc_registers), 8));

static const u32 *gen8_3_0_external_core_regs[] = {
	gen8_3_0_gdpm_lkg_registers,
	gen8_0_0_gpu_cc_ahb2phy_broadcast_swman_registers,
	gen8_0_0_gpu_cc_ahb2phy_swman_registers,
	gen8_3_0_gpu_cc_gpu_cc_reg_registers,
	gen8_3_0_gpu_cc_pll0_cm_pll_lucid_ole_registers,
	gen8_3_0_acd_acd_mnd_registers,
	gen8_0_0_gx_clkctl_ahb2phy_broadcast_swman_registers,
	gen8_0_0_gx_clkctl_ahb2phy_swman_registers,
	gen8_3_0_gx_clkctl_pll0_cm_pll_lucid_ole_registers,
	gen8_3_0_gx_clkctl_gx_clkctl_reg_registers,
};
#endif /*_ADRENO_GEN8_3_0_SNAPSHOT_H */
