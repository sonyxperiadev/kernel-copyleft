/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __ADRENO_GEN7_SNAPSHOT_H
#define __ADRENO_GEN7_SNAPSHOT_H

#include "adreno.h"
#include "adreno_gen7.h"
#include "kgsl_regmap.h"

#define CLUSTER_NONE 0
#define CLUSTER_FE 1
#define CLUSTER_SP_VS 2
#define CLUSTER_PC_VS 3
#define CLUSTER_GRAS 4
#define CLUSTER_SP_PS 5
#define CLUSTER_VPC_PS 6
#define CLUSTER_PS 7

#define HLSQ_STATE 0
#define HLSQ_DP 1
#define SP_TOP 2
#define USPTP 3
#define HLSQ_DP_STR 4

#define STATE_NON_CONTEXT 0
#define STATE_TOGGLE_CTXT 1
#define STATE_FORCE_CTXT_0 2
#define STATE_FORCE_CTXT_1 3

#define GEN7_DEBUGBUS_BLOCK_SIZE 0x100

/* Number of dword to dump in snapshot for CP SQE */
#define GEN7_SQE_FW_SNAPSHOT_DWORDS 5

struct gen7_sel_reg {
	unsigned int host_reg;
	unsigned int cd_reg;
	unsigned int val;
};

struct gen7_sptp_cluster_registers {
	/* cluster_id: Cluster identifier */
	int cluster_id;
	/* statetype: SP block state type for the cluster */
	int statetype;
	/* pipe_id: Pipe identifier */
	int pipe_id;
	/* context_id: Context identifier */
	int context_id;
	/* location_id: Location identifier */
	int location_id;
	/* regs: Pointer to the list of register pairs to read */
	const u32 *regs;
	/* regbase: Dword offset of the register block in the GPu register space */
	unsigned int regbase;
	/* offset: Internal variable used to track the crashdump state */
	unsigned int offset;
};

struct gen7_shader_block {
	/* statetype: Type identifer for the block */
	u32 statetype;
	/* size: Size of the block (in dwords) */
	u32 size;
	/* num_sps: The SP id to dump */
	u32 num_sps;
	/* num_usptps: The number of USPTPs to dump */;
	u32 num_usptps;
	/* pipe_id: Pipe identifier for the block data  */
	u32 pipeid;
	/* location: Location identifer for the block data */
	u32 location;
	/* offset: The offset in the snasphot dump */
	u64 offset;
};

struct gen7_shader_block_info {
	struct gen7_shader_block *block;
	unsigned int sp_id;
	unsigned int usptp;
	u32 bank;
	u64 offset;
};

struct gen7_reg_list {
	const u32 *regs;
	const struct gen7_sel_reg *sel;
	u64 offset;
};

struct gen7_cp_indexed_reg {
	u32 addr;
	u32 data;
	u32 size;
};

struct gen7_cluster_registers {
	/* cluster_id: Cluster identifier */
	int cluster_id;
	/* pipe_id: Pipe Identifier */
	int pipe_id;
	/* context_id: one of STATE_ that identifies the context to dump */
	int context_id;
	/* regs: Pointer to an array of register pairs */
	const u32 *regs;
	/* sel: Pointer to a selector register to write before reading */
	const struct gen7_sel_reg *sel;
	/* offset: Internal variable to track the state of the crashdump */
	unsigned int offset;
};

struct gen7_snapshot_block_list {
	/* pre_crashdumper_regs : Registers which need to be dumped before CD runs */
	const u32 *pre_crashdumper_regs;
	/* debugbus_blocks : List of debugbus blocks */
	const u32 *debugbus_blocks;
	/* debugbus_blocks_len : Length of the debugbus list */
	size_t debugbus_blocks_len;
	/* gbif_debugbus_blocks : List of GBIF debugbus blocks */
	const u32 *gbif_debugbus_blocks;
	/* gbif_debugbus_blocks_len : Length of GBIF debugbus list */
	size_t gbif_debugbus_blocks_len;
	/* cx_debugbus_blocks : List of CX debugbus blocks */
	const u32 *cx_debugbus_blocks;
	/* cx_debugbus_blocks_len : Length of the CX debugbus list */
	size_t cx_debugbus_blocks_len;
	/* external_core_regs : List of external core registers */
	const u32 **external_core_regs;
	/* num_external_core_regs : length of external core registers list */
	size_t num_external_core_regs;
	/* gmu_regs : List of GMU registers */
	const u32 *gmu_regs;
	/* gmu_gx_regs : List of GMU GX registers */
	const u32 *gmu_gx_regs;
	/* rscc_regs : List of RSCC registers */
	const u32 *rscc_regs;
	/* reg_list : List of GPU internal registers */
	struct gen7_reg_list *reg_list;
	/* reg_list : List of cx_misc registers */
	const u32 *cx_misc_regs;
	/* shader_blocks : List of GPU shader memory */
	struct gen7_shader_block *shader_blocks;
	/* num_shader_blocks : Length of the shader memory list */
	size_t num_shader_blocks;
	/* cluster_registers : List of GPU cluster registers */
	struct gen7_cluster_registers *clusters;
	/* num_clusters : Length of GPU cluster registers list */
	size_t num_clusters;
	/* spstp_cluster_registers : List of GPU SPTP cluster registers */
	struct gen7_sptp_cluster_registers *sptp_clusters;
	/* num_sptp_clusters : Length of GPU SPTP cluster registers list */
	size_t num_sptp_clusters;
	/* post_crashdumper_regs : Registers which need to be dumped after CD runs */
	const u32 *post_crashdumper_regs;
	/* index_registers : List of index_registers */
	struct gen7_cp_indexed_reg *index_registers;
	/* index_registers_len : Length of the index registers */
	size_t index_registers_len;
};

struct gen7_trace_buffer_info {
	u16 dbgc_ctrl;
	u16 segment;
	u16 granularity;
	u16 ping_blk[TRACE_BUF_NUM_SIG];
	u16 ping_idx[TRACE_BUF_NUM_SIG];
};

enum gen7_debugbus_ids {
	DEBUGBUS_CP_0_0           = 1,
	DEBUGBUS_CP_0_1           = 2,
	DEBUGBUS_RBBM             = 3,
	DEBUGBUS_GBIF_GX          = 5,
	DEBUGBUS_GBIF_CX          = 6,
	DEBUGBUS_HLSQ             = 7,
	DEBUGBUS_UCHE_0           = 9,
	DEBUGBUS_UCHE_1           = 10,
	DEBUGBUS_TESS_BR          = 13,
	DEBUGBUS_TESS_BV          = 14,
	DEBUGBUS_PC_BR            = 17,
	DEBUGBUS_PC_BV            = 18,
	DEBUGBUS_VFDP_BR          = 21,
	DEBUGBUS_VFDP_BV          = 22,
	DEBUGBUS_VPC_BR           = 25,
	DEBUGBUS_VPC_BV           = 26,
	DEBUGBUS_TSE_BR           = 29,
	DEBUGBUS_TSE_BV           = 30,
	DEBUGBUS_RAS_BR           = 33,
	DEBUGBUS_RAS_BV           = 34,
	DEBUGBUS_VSC              = 37,
	DEBUGBUS_COM_0            = 39,
	DEBUGBUS_LRZ_BR           = 43,
	DEBUGBUS_LRZ_BV           = 44,
	DEBUGBUS_UFC_0            = 47,
	DEBUGBUS_UFC_1            = 48,
	DEBUGBUS_GMU_GX           = 55,
	DEBUGBUS_DBGC             = 59,
	DEBUGBUS_CX               = 60,
	DEBUGBUS_GMU_CX           = 61,
	DEBUGBUS_GPC_BR           = 62,
	DEBUGBUS_GPC_BV           = 63,
	DEBUGBUS_LARC             = 66,
	DEBUGBUS_HLSQ_SPTP        = 68,
	DEBUGBUS_RB_0             = 70,
	DEBUGBUS_RB_1             = 71,
	DEBUGBUS_RB_2             = 72,
	DEBUGBUS_RB_3             = 73,
	DEBUGBUS_RB_4             = 74,
	DEBUGBUS_RB_5             = 75,
	DEBUGBUS_UCHE_WRAPPER     = 102,
	DEBUGBUS_CCU_0            = 106,
	DEBUGBUS_CCU_1            = 107,
	DEBUGBUS_CCU_2            = 108,
	DEBUGBUS_CCU_3            = 109,
	DEBUGBUS_CCU_4            = 110,
	DEBUGBUS_CCU_5            = 111,
	DEBUGBUS_VFD_BR_0         = 138,
	DEBUGBUS_VFD_BR_1         = 139,
	DEBUGBUS_VFD_BR_2         = 140,
	DEBUGBUS_VFD_BR_3         = 141,
	DEBUGBUS_VFD_BR_4         = 142,
	DEBUGBUS_VFD_BR_5         = 143,
	DEBUGBUS_VFD_BR_6         = 144,
	DEBUGBUS_VFD_BR_7         = 145,
	DEBUGBUS_VFD_BV_0         = 202,
	DEBUGBUS_VFD_BV_1         = 203,
	DEBUGBUS_VFD_BV_2         = 204,
	DEBUGBUS_VFD_BV_3         = 205,
	DEBUGBUS_USP_0            = 234,
	DEBUGBUS_USP_1            = 235,
	DEBUGBUS_USP_2            = 236,
	DEBUGBUS_USP_3            = 237,
	DEBUGBUS_USP_4            = 238,
	DEBUGBUS_USP_5            = 239,
	DEBUGBUS_TP_0             = 266,
	DEBUGBUS_TP_1             = 267,
	DEBUGBUS_TP_2             = 268,
	DEBUGBUS_TP_3             = 269,
	DEBUGBUS_TP_4             = 270,
	DEBUGBUS_TP_5             = 271,
	DEBUGBUS_TP_6             = 272,
	DEBUGBUS_TP_7             = 273,
	DEBUGBUS_TP_8             = 274,
	DEBUGBUS_TP_9             = 275,
	DEBUGBUS_TP_10            = 276,
	DEBUGBUS_TP_11            = 277,
	DEBUGBUS_USPTP_0          = 330,
	DEBUGBUS_USPTP_1          = 331,
	DEBUGBUS_USPTP_2          = 332,
	DEBUGBUS_USPTP_3          = 333,
	DEBUGBUS_USPTP_4          = 334,
	DEBUGBUS_USPTP_5          = 335,
	DEBUGBUS_USPTP_6          = 336,
	DEBUGBUS_USPTP_7          = 337,
	DEBUGBUS_USPTP_8          = 338,
	DEBUGBUS_USPTP_9          = 339,
	DEBUGBUS_USPTP_10         = 340,
	DEBUGBUS_USPTP_11         = 341,
	DEBUGBUS_CCHE_0           = 396,
	DEBUGBUS_CCHE_1           = 397,
	DEBUGBUS_CCHE_2           = 398,
	DEBUGBUS_VPC_DSTR_0       = 408,
	DEBUGBUS_VPC_DSTR_1       = 409,
	DEBUGBUS_VPC_DSTR_2       = 410,
	DEBUGBUS_HLSQ_DP_STR_0    = 411,
	DEBUGBUS_HLSQ_DP_STR_1    = 412,
	DEBUGBUS_HLSQ_DP_STR_2    = 413,
	DEBUGBUS_HLSQ_DP_STR_3    = 414,
	DEBUGBUS_HLSQ_DP_STR_4    = 415,
	DEBUGBUS_HLSQ_DP_STR_5    = 416,
	DEBUGBUS_UFC_DSTR_0       = 443,
	DEBUGBUS_UFC_DSTR_1       = 444,
	DEBUGBUS_UFC_DSTR_2       = 445,
	DEBUGBUS_CGC_SUBCORE      = 446,
	DEBUGBUS_CGC_CORE         = 447,
};

static const u32 gen7_gbif_debugbus_blocks[] = {
	DEBUGBUS_GBIF_CX,
	DEBUGBUS_GBIF_GX,
};

static const u32 gen7_cx_dbgc_debugbus_blocks[] = {
	DEBUGBUS_GMU_CX,
	DEBUGBUS_CX,
	DEBUGBUS_GBIF_CX,
};

enum gen7_statetype_ids {
	TP0_NCTX_REG                   = 0,
	TP0_CTX0_3D_CVS_REG            = 1,
	TP0_CTX0_3D_CPS_REG            = 2,
	TP0_CTX1_3D_CVS_REG            = 3,
	TP0_CTX1_3D_CPS_REG            = 4,
	TP0_CTX2_3D_CPS_REG            = 5,
	TP0_CTX3_3D_CPS_REG            = 6,
	TP0_TMO_DATA                   = 9,
	TP0_SMO_DATA                   = 10,
	TP0_MIPMAP_BASE_DATA           = 11,
	SP_NCTX_REG                    = 32,
	SP_CTX0_3D_CVS_REG             = 33,
	SP_CTX0_3D_CPS_REG             = 34,
	SP_CTX1_3D_CVS_REG             = 35,
	SP_CTX1_3D_CPS_REG             = 36,
	SP_CTX2_3D_CPS_REG             = 37,
	SP_CTX3_3D_CPS_REG             = 38,
	SP_INST_DATA                   = 39,
	SP_INST_DATA_1                 = 40,
	SP_LB_0_DATA                   = 41,
	SP_LB_1_DATA                   = 42,
	SP_LB_2_DATA                   = 43,
	SP_LB_3_DATA                   = 44,
	SP_LB_4_DATA                   = 45,
	SP_LB_5_DATA                   = 46,
	SP_LB_6_DATA                   = 47,
	SP_LB_7_DATA                   = 48,
	SP_CB_RAM                      = 49,
	SP_LB_13_DATA                  = 50,
	SP_LB_14_DATA                  = 51,
	SP_INST_TAG                    = 52,
	SP_INST_DATA_2                 = 53,
	SP_TMO_TAG                     = 54,
	SP_SMO_TAG                     = 55,
	SP_STATE_DATA                  = 56,
	SP_HWAVE_RAM                   = 57,
	SP_L0_INST_BUF                 = 58,
	SP_LB_8_DATA                   = 59,
	SP_LB_9_DATA                   = 60,
	SP_LB_10_DATA                  = 61,
	SP_LB_11_DATA                  = 62,
	SP_LB_12_DATA                  = 63,
	HLSQ_DATAPATH_DSTR_META        = 64,
	HLSQ_L2STC_TAG_RAM             = 67,
	HLSQ_L2STC_INFO_CMD            = 68,
	HLSQ_CVS_BE_CTXT_BUF_RAM_TAG   = 69,
	HLSQ_CPS_BE_CTXT_BUF_RAM_TAG   = 70,
	HLSQ_GFX_CVS_BE_CTXT_BUF_RAM   = 71,
	HLSQ_GFX_CPS_BE_CTXT_BUF_RAM   = 72,
	HLSQ_CHUNK_CVS_RAM             = 73,
	HLSQ_CHUNK_CPS_RAM             = 74,
	HLSQ_CHUNK_CVS_RAM_TAG         = 75,
	HLSQ_CHUNK_CPS_RAM_TAG         = 76,
	HLSQ_ICB_CVS_CB_BASE_TAG       = 77,
	HLSQ_ICB_CPS_CB_BASE_TAG       = 78,
	HLSQ_CVS_MISC_RAM              = 79,
	HLSQ_CPS_MISC_RAM              = 80,
	HLSQ_CPS_MISC_RAM_1            = 81,
	HLSQ_INST_RAM                  = 82,
	HLSQ_GFX_CVS_CONST_RAM         = 83,
	HLSQ_GFX_CPS_CONST_RAM         = 84,
	HLSQ_CVS_MISC_RAM_TAG          = 85,
	HLSQ_CPS_MISC_RAM_TAG          = 86,
	HLSQ_INST_RAM_TAG              = 87,
	HLSQ_GFX_CVS_CONST_RAM_TAG     = 88,
	HLSQ_GFX_CPS_CONST_RAM_TAG     = 89,
	HLSQ_GFX_LOCAL_MISC_RAM        = 90,
	HLSQ_GFX_LOCAL_MISC_RAM_TAG    = 91,
	HLSQ_INST_RAM_1                = 92,
	HLSQ_STPROC_META               = 93,
	HLSQ_BV_BE_META                = 94,
	HLSQ_INST_RAM_2                = 95,
	HLSQ_DATAPATH_META             = 96,
	HLSQ_FRONTEND_META             = 97,
	HLSQ_INDIRECT_META             = 98,
	HLSQ_BACKEND_META              = 99,
};

static struct gen7_cp_indexed_reg gen7_cp_indexed_reg_list[] = {
	{ GEN7_CP_SQE_STAT_ADDR, GEN7_CP_SQE_STAT_DATA, 0x40},
	{ GEN7_CP_DRAW_STATE_ADDR, GEN7_CP_DRAW_STATE_DATA, 0x100},
	{ GEN7_CP_SQE_UCODE_DBG_ADDR, GEN7_CP_SQE_UCODE_DBG_DATA, 0x8000},
	{ GEN7_CP_BV_SQE_STAT_ADDR, GEN7_CP_BV_SQE_STAT_DATA, 0x40},
	{ GEN7_CP_BV_DRAW_STATE_ADDR, GEN7_CP_BV_DRAW_STATE_DATA, 0x100},
	{ GEN7_CP_BV_SQE_UCODE_DBG_ADDR, GEN7_CP_BV_SQE_UCODE_DBG_DATA, 0x8000},
	{ GEN7_CP_SQE_AC_STAT_ADDR, GEN7_CP_SQE_AC_STAT_DATA, 0x40},
	{ GEN7_CP_LPAC_DRAW_STATE_ADDR, GEN7_CP_LPAC_DRAW_STATE_DATA, 0x100},
	{ GEN7_CP_SQE_AC_UCODE_DBG_ADDR, GEN7_CP_SQE_AC_UCODE_DBG_DATA, 0x8000},
	{ GEN7_CP_LPAC_FIFO_DBG_ADDR, GEN7_CP_LPAC_FIFO_DBG_DATA, 0x40},
};
#endif /*_ADRENO_GEN7_SNAPSHOT_H */
