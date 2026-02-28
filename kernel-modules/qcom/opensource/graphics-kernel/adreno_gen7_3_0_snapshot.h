/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __ADRENO_GEN7_3_0_SNAPSHOT_H
#define __ADRENO_GEN7_3_0_SNAPSHOT_H

#include "adreno_gen7_snapshot.h"
#include "adreno_gen7_0_0_snapshot.h"

static const u32 gen7_3_0_debugbus_blocks[] = {
	DEBUGBUS_CP_0_0,
	DEBUGBUS_CP_0_1,
	DEBUGBUS_RBBM,
	DEBUGBUS_HLSQ,
	DEBUGBUS_UCHE_0,
	DEBUGBUS_TESS_BR,
	DEBUGBUS_PC_BR,
	DEBUGBUS_VFDP_BR,
	DEBUGBUS_VPC_BR,
	DEBUGBUS_TSE_BR,
	DEBUGBUS_RAS_BR,
	DEBUGBUS_VSC,
	DEBUGBUS_COM_0,
	DEBUGBUS_LRZ_BR,
	DEBUGBUS_UFC_0,
	DEBUGBUS_UFC_1,
	DEBUGBUS_GMU_GX,
	DEBUGBUS_DBGC,
	DEBUGBUS_GPC_BR,
	DEBUGBUS_LARC,
	DEBUGBUS_HLSQ_SPTP,
	DEBUGBUS_RB_0,
	DEBUGBUS_UCHE_WRAPPER,
	DEBUGBUS_CCU_0,
	DEBUGBUS_VFD_BR_0,
	DEBUGBUS_VFD_BR_1,
	DEBUGBUS_USP_0,
	DEBUGBUS_TP_0,
	DEBUGBUS_TP_1,
	DEBUGBUS_USPTP_0,
	DEBUGBUS_USPTP_1,
};

static struct gen7_shader_block gen7_3_0_shader_blocks[] = {
	{HLSQ_CPS_MISC_RAM_1,        0x200, 1, 1, PIPE_BR, HLSQ_STATE},
	{SP_LB_0_DATA,               0x800, 1, 2, PIPE_BR, USPTP},
	{SP_LB_1_DATA,               0x800, 1, 2, PIPE_BR, USPTP},
	{SP_LB_2_DATA,               0x800, 1, 2, PIPE_BR, USPTP},
	{SP_LB_3_DATA,               0x800, 1, 2, PIPE_BR, USPTP},
	{SP_LB_4_DATA,               0x800, 1, 2, PIPE_BR, USPTP},
	{SP_LB_5_DATA,               0x800, 1, 2, PIPE_BR, USPTP},
	{TP0_TMO_DATA,               0x200, 1, 2, PIPE_BR, USPTP},
	{TP0_SMO_DATA,               0x80, 1, 2, PIPE_BR, USPTP},
	{TP0_MIPMAP_BASE_DATA,       0x3c0, 1, 2, PIPE_BR, USPTP},
	{SP_INST_DATA,               0x800, 1, 2, PIPE_BR, USPTP},
	{SP_INST_DATA_1,             0x800, 1, 2, PIPE_BR, USPTP},
	{SP_CB_RAM,                  0x390, 1, 2, PIPE_BR, USPTP,},
	{SP_INST_TAG,                0x90, 1, 2, PIPE_BR, USPTP},
	{SP_TMO_TAG,                 0x80, 1, 2, PIPE_BR, USPTP},
	{SP_SMO_TAG,                 0x80, 1, 2, PIPE_BR, USPTP},
	{SP_STATE_DATA,              0x40, 1, 2, PIPE_BR, USPTP},
	{SP_HWAVE_RAM,               0x100, 1, 2, PIPE_BR, USPTP},
	{SP_L0_INST_BUF,             0x50, 1, 2, PIPE_BR, USPTP},
	{HLSQ_CVS_BE_CTXT_BUF_RAM_TAG,    0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CPS_BE_CTXT_BUF_RAM_TAG,    0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_GFX_CVS_BE_CTXT_BUF_RAM,        0x300, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_GFX_CPS_BE_CTXT_BUF_RAM,        0x300, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CHUNK_CVS_RAM,         0x1c0, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CHUNK_CPS_RAM,         0x300, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CHUNK_CVS_RAM_TAG,     0x40, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CHUNK_CPS_RAM_TAG,     0x40, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_ICB_CVS_CB_BASE_TAG,   0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_ICB_CPS_CB_BASE_TAG,   0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CVS_MISC_RAM,          0x280, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CPS_MISC_RAM,          0x800, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_INST_RAM,              0x800, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_GFX_CVS_CONST_RAM,     0x800, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_GFX_CPS_CONST_RAM,     0x800, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CVS_MISC_RAM_TAG,      0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_CPS_MISC_RAM_TAG,      0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_INST_RAM_TAG,          0x80, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_GFX_CVS_CONST_RAM_TAG, 0x64, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_GFX_CPS_CONST_RAM_TAG, 0x64, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_INST_RAM_1,            0x800, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_STPROC_META,           0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_BV_BE_META,            0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_DATAPATH_META,         0x20, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_FRONTEND_META,         0x40, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_INDIRECT_META,         0x10, 1, 1, PIPE_BR, HLSQ_STATE},
	{HLSQ_BACKEND_META,          0x40, 1, 1, PIPE_BR, HLSQ_STATE},
};

static const u32 gen7_3_0_gpu_registers[] = {
	0x00000, 0x00000, 0x00002, 0x00002, 0x00011, 0x00012, 0x00016, 0x0001b,
	0x0001f, 0x00032, 0x00038, 0x0003c, 0x00042, 0x00042, 0x00044, 0x00044,
	0x00047, 0x00047, 0x00049, 0x0004a, 0x0004c, 0x0004c, 0x00050, 0x00050,
	0x00056, 0x00056, 0x00073, 0x00075, 0x000ad, 0x000ae, 0x000b0, 0x000b0,
	0x000b4, 0x000b4, 0x000b8, 0x000b8, 0x000bc, 0x000bc, 0x000c0, 0x000c0,
	0x000c4, 0x000c4, 0x000c8, 0x000c8, 0x000cc, 0x000cc, 0x000d0, 0x000d0,
	0x000d4, 0x000d4, 0x000d8, 0x000d8, 0x000dc, 0x000dc, 0x000e0, 0x000e0,
	0x000e4, 0x000e4, 0x000e8, 0x000e8, 0x000ec, 0x000ec, 0x000f0, 0x000f0,
	0x000f4, 0x000f4, 0x000f8, 0x000f8, 0x00100, 0x00100, 0x00104, 0x0010b,
	0x0010f, 0x0011d, 0x0012f, 0x0012f, 0x00200, 0x0020d, 0x00211, 0x00211,
	0x00215, 0x00243, 0x00260, 0x00268, 0x00272, 0x00274, 0x00286, 0x00286,
	0x0028a, 0x0028a, 0x0028c, 0x0028c, 0x00300, 0x00401, 0x00500, 0x00500,
	0x00507, 0x0050b, 0x0050f, 0x0050f, 0x00511, 0x00511, 0x00533, 0x00534,
	0x00536, 0x00536, 0x00540, 0x00555, 0x00564, 0x00567, 0x00800, 0x00808,
	0x00810, 0x00813, 0x00820, 0x00821, 0x00823, 0x00827, 0x00830, 0x00834,
	0x00840, 0x00841, 0x00843, 0x00847, 0x0084f, 0x00886, 0x008a0, 0x008ab,
	0x008c0, 0x008c0, 0x008c4, 0x008c5, 0x008d0, 0x008dd, 0x008f0, 0x008f3,
	0x00900, 0x00903, 0x00908, 0x00911, 0x00928, 0x0093e, 0x00942, 0x0094d,
	0x00980, 0x00984, 0x0098d, 0x0098f, 0x009b0, 0x009b4, 0x009c2, 0x009c9,
	0x009ce, 0x009d7, 0x00a00, 0x00a00, 0x00a02, 0x00a03, 0x00a10, 0x00a4f,
	0x00a67, 0x00a6c, 0x00a9c, 0x00a9f, 0x00c00, 0x00c00, 0x00c02, 0x00c04,
	0x00c06, 0x00c06, 0x00c10, 0x00cd9, 0x00ce0, 0x00d0c, 0x00df0, 0x00df4,
	0x00e01, 0x00e02, 0x00e07, 0x00e0e, 0x00e10, 0x00e12, 0x00e17, 0x00e17,
	0x00e19, 0x00e19, 0x00e1b, 0x00e2b, 0x00e30, 0x00e32, 0x00e38, 0x00e3c,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_gpu_registers), 8));

static const u32 gen7_3_0_gmu_registers[] = {
	0x10001, 0x10001, 0x10003, 0x10003, 0x10401, 0x10401, 0x10403, 0x10403,
	0x10801, 0x10801, 0x10803, 0x10803, 0x10c01, 0x10c01, 0x10c03, 0x10c03,
	0x11001, 0x11001, 0x11003, 0x11003, 0x11401, 0x11401, 0x11403, 0x11403,
	0x11801, 0x11801, 0x11803, 0x11803, 0x11c01, 0x11c01, 0x11c03, 0x11c03,
	0x1f400, 0x1f40b, 0x1f40f, 0x1f411, 0x1f500, 0x1f500, 0x1f507, 0x1f507,
	0x1f509, 0x1f50b, 0x1f800, 0x1f804, 0x1f807, 0x1f808, 0x1f80b, 0x1f80c,
	0x1f80f, 0x1f80f, 0x1f811, 0x1f811, 0x1f813, 0x1f817, 0x1f819, 0x1f81c,
	0x1f824, 0x1f82a, 0x1f82d, 0x1f830, 0x1f840, 0x1f853, 0x1f860, 0x1f860,
	0x1f870, 0x1f879, 0x1f87f, 0x1f87f, 0x1f888, 0x1f889, 0x1f8a0, 0x1f8a2,
	0x1f8a4, 0x1f8af, 0x1f8c0, 0x1f8c1, 0x1f8c3, 0x1f8c4, 0x1f8d0, 0x1f8d0,
	0x1f8ec, 0x1f8ec, 0x1f8f0, 0x1f8f1, 0x1f910, 0x1f911, 0x1f920, 0x1f921,
	0x1f924, 0x1f925, 0x1f928, 0x1f929, 0x1f92c, 0x1f92d, 0x1f940, 0x1f940,
	0x1f942, 0x1f944, 0x1f948, 0x1f94a, 0x1f980, 0x1f981, 0x1f984, 0x1f986,
	0x1f992, 0x1f993, 0x1f996, 0x1f99e, 0x1f9c0, 0x1f9c0, 0x1f9c5, 0x1f9d4,
	0x1f9f1, 0x1f9f1, 0x1f9f8, 0x1f9fa, 0x1fa00, 0x1fa03, 0x20000, 0x20005,
	0x20008, 0x20009, 0x20010, 0x20012, 0x20018, 0x20018, 0x20020, 0x20023,
	0x20030, 0x20031, 0x23801, 0x23801, 0x23803, 0x23803, 0x23805, 0x23805,
	0x23807, 0x23807, 0x23809, 0x23809, 0x2380b, 0x2380b, 0x2380d, 0x2380d,
	0x2380f, 0x2380f, 0x23811, 0x23811, 0x23813, 0x23813, 0x23815, 0x23815,
	0x23817, 0x23817, 0x23819, 0x23819, 0x2381b, 0x2381b, 0x2381d, 0x2381d,
	0x2381f, 0x23820, 0x23822, 0x23822, 0x23824, 0x23824, 0x23826, 0x23826,
	0x23828, 0x23828, 0x2382a, 0x2382a, 0x2382c, 0x2382c, 0x2382e, 0x2382e,
	0x23830, 0x23830, 0x23832, 0x23832, 0x23834, 0x23834, 0x23836, 0x23836,
	0x23838, 0x23838, 0x2383a, 0x2383a, 0x2383c, 0x2383c, 0x2383e, 0x2383e,
	0x23840, 0x23847, 0x23b00, 0x23b01, 0x23b03, 0x23b03, 0x23b05, 0x23b0e,
	0x23b10, 0x23b13, 0x23b15, 0x23b16, 0x23b20, 0x23b20, 0x23b28, 0x23b28,
	0x23b30, 0x23b30,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_gmu_registers), 8));

static const u32 gen7_3_0_gmu_gx_registers[] = {
	0x1a802, 0x1a802, 0x1a883, 0x1a884, 0x1a900, 0x1a92b, 0x1a940, 0x1a940,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_gmu_gx_registers), 8));

static const u32 gen7_3_0_noncontext_pipe_bv_registers[] = {
	0x00887, 0x0088c, 0x08600, 0x08600, 0x08602, 0x08602, 0x08610, 0x0861b,
	0x08620, 0x08620, 0x08630, 0x08630, 0x08637, 0x08639, 0x08640, 0x08640,
	0x09600, 0x09600, 0x09602, 0x09603, 0x0960a, 0x09616, 0x09624, 0x0963a,
	0x09640, 0x09640, 0x0a600, 0x0a600, 0x0a603, 0x0a603, 0x0a610, 0x0a61f,
	0x0a630, 0x0a631, 0x0a638, 0x0a638,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_noncontext_pipe_bv_registers), 8));

static const u32 gen7_3_0_noncontext_rb_rac_pipe_br_registers[] = {
	0x08e10, 0x08e1c, 0x08e20, 0x08e25, 0x08e51, 0x08e54,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_noncontext_rb_rac_pipe_br_registers), 8));

/* Block: SP Cluster: CLUSTER_SP_PS Pipeline: PIPE_LPAC Location: HLSQ_STATE */
static const u32 gen7_3_0_sp_cluster_sp_ps_pipe_lpac_hlsq_state_registers[] = {
	0x0aa40, 0x0aabf,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_sp_cluster_sp_ps_pipe_lpac_hlsq_state_registers), 8));

/* Block: SP Cluster: CLUSTER_SP_PS Pipeline: PIPE_LPAC Location: uSPTP */
static const u32 gen7_3_0_sp_cluster_sp_ps_pipe_lpac_usptp_registers[] = {
	0x0aa40, 0x0aabf,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_sp_cluster_sp_ps_pipe_lpac_usptp_registers), 8));

static const struct gen7_sel_reg  gen7_3_0_rb_rac_sel = {
	.host_reg = GEN7_RB_SUB_BLOCK_SEL_CNTL_HOST,
	.cd_reg = GEN7_RB_SUB_BLOCK_SEL_CNTL_CD,
	.val = 0x0,
};

static const struct gen7_sel_reg gen7_3_0_rb_rbp_sel = {
	.host_reg = GEN7_RB_SUB_BLOCK_SEL_CNTL_HOST,
	.cd_reg = GEN7_RB_SUB_BLOCK_SEL_CNTL_CD,
	.val = 0x9,
};

static struct gen7_cluster_registers gen7_3_0_clusters[] = {
	{ CLUSTER_NONE, PIPE_BR, STATE_NON_CONTEXT,
		gen7_0_0_noncontext_pipe_br_registers, },
	{ CLUSTER_NONE, PIPE_BV, STATE_NON_CONTEXT,
		gen7_3_0_noncontext_pipe_bv_registers, },
	{ CLUSTER_NONE, PIPE_LPAC, STATE_NON_CONTEXT,
		gen7_0_0_noncontext_pipe_lpac_registers, },
	{ CLUSTER_NONE, PIPE_BR, STATE_NON_CONTEXT,
		gen7_3_0_noncontext_rb_rac_pipe_br_registers, &gen7_3_0_rb_rac_sel, },
	{ CLUSTER_NONE, PIPE_BR, STATE_NON_CONTEXT,
		gen7_0_0_noncontext_rb_rbp_pipe_br_registers, &gen7_3_0_rb_rbp_sel, },
	{ CLUSTER_GRAS, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_gras_cluster_gras_pipe_br_registers, },
	{ CLUSTER_GRAS, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_gras_cluster_gras_pipe_br_registers, },
	{ CLUSTER_FE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_pc_cluster_fe_pipe_br_registers, },
	{ CLUSTER_FE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_pc_cluster_fe_pipe_br_registers, },
	{ CLUSTER_PS, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_rb_rac_cluster_ps_pipe_br_registers, &gen7_3_0_rb_rac_sel, },
	{ CLUSTER_PS, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_rb_rac_cluster_ps_pipe_br_registers, &gen7_3_0_rb_rac_sel, },
	{ CLUSTER_PS, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_rb_rbp_cluster_ps_pipe_br_registers, &gen7_3_0_rb_rbp_sel, },
	{ CLUSTER_PS, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_rb_rbp_cluster_ps_pipe_br_registers, &gen7_3_0_rb_rbp_sel, },
	{ CLUSTER_FE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_vfd_cluster_fe_pipe_br_registers, },
	{ CLUSTER_FE, PIPE_BV, STATE_FORCE_CTXT_0,
		gen7_0_0_vfd_cluster_fe_pipe_bv_registers, },
	{ CLUSTER_FE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_vfd_cluster_fe_pipe_br_registers, },
	{ CLUSTER_FE, PIPE_BV, STATE_FORCE_CTXT_1,
		gen7_0_0_vfd_cluster_fe_pipe_bv_registers, },
	{ CLUSTER_FE, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_vpc_cluster_fe_pipe_br_registers, },
	{ CLUSTER_FE, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_vpc_cluster_fe_pipe_br_registers, },
	{ CLUSTER_PC_VS, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_vpc_cluster_pc_vs_pipe_br_registers, },
	{ CLUSTER_PC_VS, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_vpc_cluster_pc_vs_pipe_br_registers, },
	{ CLUSTER_VPC_PS, PIPE_BR, STATE_FORCE_CTXT_0,
		gen7_0_0_vpc_cluster_vpc_ps_pipe_br_registers, },
	{ CLUSTER_VPC_PS, PIPE_BR, STATE_FORCE_CTXT_1,
		gen7_0_0_vpc_cluster_vpc_ps_pipe_br_registers, },
};

static struct gen7_sptp_cluster_registers gen7_3_0_sptp_clusters[] = {
	{ CLUSTER_NONE, SP_NCTX_REG, PIPE_BR, 0, HLSQ_STATE,
		gen7_0_0_sp_noncontext_pipe_br_hlsq_state_registers, 0xae00 },
	{ CLUSTER_NONE, SP_NCTX_REG, PIPE_BR, 0, SP_TOP,
		gen7_0_0_sp_noncontext_pipe_br_sp_top_registers, 0xae00 },
	{ CLUSTER_NONE, SP_NCTX_REG, PIPE_BR, 0, USPTP,
		gen7_0_0_sp_noncontext_pipe_br_usptp_registers, 0xae00 },
	{ CLUSTER_NONE, TP0_NCTX_REG, PIPE_BR, 0, USPTP,
		gen7_0_0_tpl1_noncontext_pipe_br_registers, 0xb600 },
	{ CLUSTER_NONE, TP0_NCTX_REG, PIPE_LPAC, 0, USPTP,
		gen7_0_0_tpl1_noncontext_pipe_lpac_registers, 0xb780 },
	{ CLUSTER_SP_PS, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_hlsq_state_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, HLSQ_DP,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_hlsq_dp_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, SP_TOP,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_sp_top_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_usptp_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_hlsq_state_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, HLSQ_DP,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_hlsq_dp_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, SP_TOP,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_sp_top_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX1_3D_CPS_REG, PIPE_BR, 1, USPTP,
		gen7_0_0_sp_cluster_sp_ps_pipe_br_usptp_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, HLSQ_STATE,
		gen7_3_0_sp_cluster_sp_ps_pipe_lpac_hlsq_state_registers, 0xa980 },
	{ CLUSTER_SP_PS, SP_CTX0_3D_CPS_REG, PIPE_LPAC, 0, USPTP,
		gen7_3_0_sp_cluster_sp_ps_pipe_lpac_usptp_registers, 0xa980 },
	{ CLUSTER_SP_VS, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, HLSQ_STATE,
		gen7_0_0_sp_cluster_sp_vs_pipe_br_hlsq_state_registers, 0xa800 },
	{ CLUSTER_SP_VS, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, SP_TOP,
		gen7_0_0_sp_cluster_sp_vs_pipe_br_sp_top_registers, 0xa800 },
	{ CLUSTER_SP_VS, SP_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen7_0_0_sp_cluster_sp_vs_pipe_br_usptp_registers, 0xa800 },
	{ CLUSTER_SP_VS, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, HLSQ_STATE,
		gen7_0_0_sp_cluster_sp_vs_pipe_br_hlsq_state_registers, 0xa800 },
	{ CLUSTER_SP_VS, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, SP_TOP,
		gen7_0_0_sp_cluster_sp_vs_pipe_br_sp_top_registers, 0xa800 },
	{ CLUSTER_SP_VS, SP_CTX1_3D_CVS_REG, PIPE_BR, 1, USPTP,
		gen7_0_0_sp_cluster_sp_vs_pipe_br_usptp_registers, 0xa800 },
	{ CLUSTER_SP_PS, TP0_CTX0_3D_CPS_REG, PIPE_BR, 0, USPTP,
		gen7_0_0_tpl1_cluster_sp_ps_pipe_br_registers, 0xb180 },
	{ CLUSTER_SP_PS, TP0_CTX1_3D_CPS_REG, PIPE_BR, 1, USPTP,
		gen7_0_0_tpl1_cluster_sp_ps_pipe_br_registers, 0xb180 },
	{ CLUSTER_SP_PS, TP0_CTX2_3D_CPS_REG, PIPE_BR, 2, USPTP,
		gen7_0_0_tpl1_cluster_sp_ps_pipe_br_registers, 0xb180 },
	{ CLUSTER_SP_PS, TP0_CTX3_3D_CPS_REG, PIPE_BR, 3, USPTP,
		gen7_0_0_tpl1_cluster_sp_ps_pipe_br_registers, 0xb180 },
	{ CLUSTER_SP_VS, TP0_CTX0_3D_CVS_REG, PIPE_BR, 0, USPTP,
		gen7_0_0_tpl1_cluster_sp_vs_pipe_br_registers, 0xb000 },
	{ CLUSTER_SP_VS, TP0_CTX0_3D_CVS_REG, PIPE_BV, 0, USPTP,
		gen7_0_0_tpl1_cluster_sp_vs_pipe_bv_registers, 0xb000 },
	{ CLUSTER_SP_VS, TP0_CTX1_3D_CVS_REG, PIPE_BR, 1, USPTP,
		gen7_0_0_tpl1_cluster_sp_vs_pipe_br_registers, 0xb000 },
	{ CLUSTER_SP_VS, TP0_CTX1_3D_CVS_REG, PIPE_BV, 1, USPTP,
		gen7_0_0_tpl1_cluster_sp_vs_pipe_bv_registers, 0xb000 },
};

static const u32 gen7_3_0_cpr_registers[] = {
	0x26800, 0x26805, 0x26808, 0x2680c, 0x26814, 0x26814, 0x2681c, 0x2681c,
	0x26820, 0x26838, 0x26840, 0x26840, 0x26848, 0x26848, 0x26850, 0x26850,
	0x26880, 0x26889, 0x26980, 0x269b0, 0x269c0, 0x269c8, 0x269e0, 0x269ee,
	0x269fb, 0x269ff, 0x26a02, 0x26a07, 0x26a09, 0x26a0b, 0x26a10, 0x26b0f,
	0x27440, 0x27441, 0x27444, 0x27444, 0x27480, 0x274a2, 0x274ac, 0x274ac,
	UINT_MAX, UINT_MAX,
};
static_assert(IS_ALIGNED(sizeof(gen7_3_0_cpr_registers), 8));

static struct gen7_reg_list gen7_3_0_reg_list[] = {
	{ gen7_3_0_gpu_registers, NULL },
	{ gen7_0_0_cx_misc_registers, NULL },
	{ gen7_0_0_dpm_registers, NULL },
	{ NULL, NULL },
};

static const u32 *gen7_3_0_external_core_regs[] = {
	gen7_0_0_gpucc_registers,
	gen7_3_0_cpr_registers,
};

static struct gen7_cp_indexed_reg gen7_3_0_cp_indexed_reg_list[] = {
	{ GEN7_CP_SQE_STAT_ADDR, GEN7_CP_SQE_STAT_DATA, 0x33},
	{ GEN7_CP_DRAW_STATE_ADDR, GEN7_CP_DRAW_STATE_DATA, 0x100},
	{ GEN7_CP_SQE_UCODE_DBG_ADDR, GEN7_CP_SQE_UCODE_DBG_DATA, 0x8000},
};

#endif /*_ADRENO_GEN7_3_0_SNAPSHOT_H */
