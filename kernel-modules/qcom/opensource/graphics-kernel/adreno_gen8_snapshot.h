/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __ADRENO_GEN8_SNAPSHOT_H
#define __ADRENO_GEN8_SNAPSHOT_H

#include "adreno.h"
#include "adreno_gen8.h"
#include "kgsl_regmap.h"
#include "kgsl_snapshot.h"

enum cluster_id {
	CLUSTER_NONE   = 0,
	CLUSTER_FE_US  = 1,
	CLUSTER_FE_S   = 2,
	CLUSTER_SP_VS  = 3,
	CLUSTER_VPC_VS = 4,
	CLUSTER_VPC_US = 5,
	CLUSTER_GRAS   = 6,
	CLUSTER_SP_PS  = 7,
	CLUSTER_VPC_PS = 8,
	CLUSTER_PS     = 9,
};

enum location_id {
	HLSQ_STATE  = 0,
	HLSQ_DP     = 1,
	SP_TOP      = 2,
	USPTP       = 3,
	HLSQ_DP_STR = 4,
};

#define STATE_NON_CONTEXT     0
#define STATE_TOGGLE_CTXT     1
#define STATE_FORCE_CTXT_0    2
#define STATE_FORCE_CTXT_1    3

#define UNSLICE                 0
#define SLICE                   1

#define NUMBER_OF_SLICES(region, adreno_dev) \
	((region == SLICE) ? gen8_get_num_slices(adreno_dev) : 1)
#define SLICE_ID(region, j) ((region == SLICE) ? j : 0)

#define GEN8_DEBUGBUS_BLOCK_SIZE 0x100

struct sel_reg {
	u32 host_reg;
	u32 cd_reg;
	u32 val;
};

struct gen8_shader_block_info {
	struct gen8_shader_block *block;
	u32 sp_id;
	u32 usptp;
	u32 slice_id;
	u32 location_id;
	u32 context_id;
	u32 bank;
	u64 offset;
};

struct gen8_shader_block {
	/* statetype: Type identifier for the block */
	u32 statetype;
	/* size: Size of the block (in dwords) */
	u32 size;
	/* num_sps: The number of SPs to dump */
	u32 num_sps;
	/* num_usptps: The number of USPTPs to dump */
	u32 num_usptps;
	/* pipeid: Pipe identifier for the block data  */
	u32 pipeid;
	/* location: Location identifier for the block data */
	u32 location;
	/* slice_region: slice region, if SLICE then loop over all slices */
	u32 slice_region;
	/* num_ctx: repeat id to loop */
	u32 num_ctx;
	/* offset: The offset in the snasphot dump */
	u64 offset;
};

struct gen8_cluster_registers_info {
	struct gen8_cluster_registers *cluster;
	u32 cluster_id;
	u32 slice_id;
	u32 pipe_id;
	u32 context_id;
	u64 offset;
};

struct gen8_cluster_registers {
	/* cluster_id: Cluster identifier */
	u32 cluster_id;
	/* slice_region: is it slice or unslice */
	u32 slice_region;
	/* pipe_id: Pipe Identifier */
	u32 pipe_id;
	/* context_id: one of STATE_ that identifies the context to dump */
	u32 context_id;
	/* regs: Pointer to an array of register pairs */
	const u32 *regs;
	/* sel: Pointer to a selector register to write before reading */
	const struct sel_reg *sel;
	/* offset: Internal variable to track the state of the crashdump */
	u32 offset;
};

struct gen8_reg_list_info {
	struct gen8_reg_list *regs;
	u32 cluster_id;
	u32 slice_id;
	u32 pipe_id;
	u32 sp_id;
	u32 usptp_id;
	u32 context_id;
	u64 offset;
};

struct gen8_sptp_cluster_registers_info {
	struct gen8_sptp_cluster_registers *cluster;
	u32 cluster_id;
	u32 slice_id;
	u32 pipe_id;
	u32 sp_id;
	u32 usptp_id;
	u32 location_id;
	u32 context_id;
	u32 statetype_id;
	u64 offset;
};

struct gen8_sptp_cluster_registers {
	/* cluster_id: Cluster identifier */
	u32 cluster_id;
	/* slice_region: is it slice or unslice */
	u32 slice_region;
	/* num_sps: The number of SPs to dump */
	u32 num_sps;
	/* num_usptps: The number of USPs to dump */
	u32 num_usptps;
	/* statetype: SP block state type for the cluster */
	u32 statetype;
	/* pipe_id: Pipe identifier */
	u32 pipe_id;
	/* context_id: Context identifier */
	u32 context_id;
	/* location_id: Location identifier */
	u32 location_id;
	/* regs: Pointer to the list of register pairs to read */
	const u32 *regs;
	/* regbase: Dword offset of the register block in the GPu register space */
	u32 regbase;
	/* offset: Internal variable used to track the crashdump state */
	u32 offset;
};

struct gen8_cp_indexed_reg {
	u32 addr;
	u32 data;
	u32 slice_region;
	u32 pipe_id;
	u32 size;
};

struct gen8_reg_list {
	u32 slice_region;
	const u32 *regs;
	const struct sel_reg *sel;
	u64 offset;
};

struct gen8_trace_buffer_info {
	u16 dbgc_ctrl;
	u16 segment;
	u16 granularity;
	u16 ping_blk[TRACE_BUF_NUM_SIG];
	u16 ping_idx[TRACE_BUF_NUM_SIG];
};

enum gen8_debugbus_ids {
	DEBUGBUS_GBIF_CX_GC_US_I_0          = 1,
	DEBUGBUS_GMU_CX_GC_US_I_0           = 2,
	DEBUGBUS_CX_GC_US_I_0               = 3,
	DEBUGBUS_GBIF_GX_GC_US_I_0          = 8,
	DEBUGBUS_GMU_GX_GC_US_I_0           = 9,
	DEBUGBUS_DBGC_GC_US_I_0             = 10,
	DEBUGBUS_RBBM_GC_US_I_0             = 11,
	DEBUGBUS_LARC_GC_US_I_0             = 12,
	DEBUGBUS_COM_GC_US_I_0              = 13,
	DEBUGBUS_HLSQ_GC_US_I_0             = 14,
	DEBUGBUS_CGC_GC_US_I_0              = 15,
	DEBUGBUS_VSC_GC_US_I_0_0            = 20,
	DEBUGBUS_VSC_GC_US_I_0_1            = 21,
	DEBUGBUS_UFC_GC_US_I_0              = 24,
	DEBUGBUS_UFC_GC_US_I_1              = 25,
	DEBUGBUS_CP_GC_US_I_0_0             = 40,
	DEBUGBUS_CP_GC_US_I_0_1             = 41,
	DEBUGBUS_CP_GC_US_I_0_2             = 42,
	DEBUGBUS_PC_BR_US_I_0               = 56,
	DEBUGBUS_PC_BV_US_I_0               = 57,
	DEBUGBUS_GPC_BR_US_I_0              = 58,
	DEBUGBUS_GPC_BV_US_I_0              = 59,
	DEBUGBUS_VPC_BR_US_I_0              = 60,
	DEBUGBUS_VPC_BV_US_I_0              = 61,
	DEBUGBUS_UCHE_WRAPPER_GC_US_I_0     = 80,
	DEBUGBUS_UCHE_GC_US_I_0             = 81,
	DEBUGBUS_UCHE_GC_US_I_1             = 82,
	DEBUGBUS_CP_GC_S_0_I_0              = 128,
	DEBUGBUS_PC_BR_S_0_I_0              = 129,
	DEBUGBUS_PC_BV_S_0_I_0              = 130,
	DEBUGBUS_TESS_GC_S_0_I_0            = 131,
	DEBUGBUS_TSEFE_GC_S_0_I_0           = 132,
	DEBUGBUS_TSEBE_GC_S_0_I_0           = 133,
	DEBUGBUS_RAS_GC_S_0_I_0             = 134,
	DEBUGBUS_LRZ_BR_S_0_I_0             = 135,
	DEBUGBUS_LRZ_BV_S_0_I_0             = 136,
	DEBUGBUS_VFDP_GC_S_0_I_0            = 137,
	DEBUGBUS_GPC_BR_S_0_I_0             = 138,
	DEBUGBUS_GPC_BV_S_0_I_0             = 139,
	DEBUGBUS_VPCFE_BR_S_0_I_0           = 140,
	DEBUGBUS_VPCFE_BV_S_0_I_0           = 141,
	DEBUGBUS_VPCBE_BR_S_0_I_0           = 142,
	DEBUGBUS_VPCBE_BV_S_0_I_0           = 143,
	DEBUGBUS_CCHE_GC_S_0_I_0            = 144,
	DEBUGBUS_DBGC_GC_S_0_I_0            = 145,
	DEBUGBUS_LARC_GC_S_0_I_0            = 146,
	DEBUGBUS_RBBM_GC_S_0_I_0            = 147,
	DEBUGBUS_CCRE_GC_S_0_I_0            = 148,
	DEBUGBUS_CGC_GC_S_0_I_0             = 149,
	DEBUGBUS_GMU_GC_S_0_I_0             = 150,
	DEBUGBUS_SLICE_GC_S_0_I_0           = 151,
	DEBUGBUS_HLSQ_SPTP_STAR_GC_S_0_I_0  = 152,
	DEBUGBUS_USP_GC_S_0_I_0             = 160,
	DEBUGBUS_USP_GC_S_0_I_1             = 161,
	DEBUGBUS_USPTP_GC_S_0_I_0           = 166,
	DEBUGBUS_USPTP_GC_S_0_I_1           = 167,
	DEBUGBUS_USPTP_GC_S_0_I_2           = 168,
	DEBUGBUS_USPTP_GC_S_0_I_3           = 169,
	DEBUGBUS_TP_GC_S_0_I_0              = 178,
	DEBUGBUS_TP_GC_S_0_I_1              = 179,
	DEBUGBUS_TP_GC_S_0_I_2              = 180,
	DEBUGBUS_TP_GC_S_0_I_3              = 181,
	DEBUGBUS_RB_GC_S_0_I_0              = 190,
	DEBUGBUS_RB_GC_S_0_I_1              = 191,
	DEBUGBUS_CCU_GC_S_0_I_0             = 196,
	DEBUGBUS_CCU_GC_S_0_I_1             = 197,
	DEBUGBUS_HLSQ_GC_S_0_I_0            = 202,
	DEBUGBUS_HLSQ_GC_S_0_I_1            = 203,
	DEBUGBUS_VFD_GC_S_0_I_0             = 208,
	DEBUGBUS_VFD_GC_S_0_I_1             = 209,
	DEBUGBUS_CP_GC_S_1_I_0              = 256,
	DEBUGBUS_PC_BR_S_1_I_0              = 257,
	DEBUGBUS_PC_BV_S_1_I_0              = 258,
	DEBUGBUS_TESS_GC_S_1_I_0            = 259,
	DEBUGBUS_TSEFE_GC_S_1_I_0           = 260,
	DEBUGBUS_TSEBE_GC_S_1_I_0           = 261,
	DEBUGBUS_RAS_GC_S_1_I_0             = 262,
	DEBUGBUS_LRZ_BR_S_1_I_0             = 263,
	DEBUGBUS_LRZ_BV_S_1_I_0             = 264,
	DEBUGBUS_VFDP_GC_S_1_I_0            = 265,
	DEBUGBUS_GPC_BR_S_1_I_0             = 266,
	DEBUGBUS_GPC_BV_S_1_I_0             = 267,
	DEBUGBUS_VPCFE_BR_S_1_I_0           = 268,
	DEBUGBUS_VPCFE_BV_S_1_I_0           = 269,
	DEBUGBUS_VPCBE_BR_S_1_I_0           = 270,
	DEBUGBUS_VPCBE_BV_S_1_I_0           = 271,
	DEBUGBUS_CCHE_GC_S_1_I_0            = 272,
	DEBUGBUS_DBGC_GC_S_1_I_0            = 273,
	DEBUGBUS_LARC_GC_S_1_I_0            = 274,
	DEBUGBUS_RBBM_GC_S_1_I_0            = 275,
	DEBUGBUS_CCRE_GC_S_1_I_0            = 276,
	DEBUGBUS_CGC_GC_S_1_I_0             = 277,
	DEBUGBUS_GMU_GC_S_1_I_0             = 278,
	DEBUGBUS_SLICE_GC_S_1_I_0           = 279,
	DEBUGBUS_HLSQ_SPTP_STAR_GC_S_1_I_0  = 280,
	DEBUGBUS_USP_GC_S_1_I_0             = 288,
	DEBUGBUS_USP_GC_S_1_I_1             = 289,
	DEBUGBUS_USPTP_GC_S_1_I_0           = 294,
	DEBUGBUS_USPTP_GC_S_1_I_1           = 295,
	DEBUGBUS_USPTP_GC_S_1_I_2           = 296,
	DEBUGBUS_USPTP_GC_S_1_I_3           = 297,
	DEBUGBUS_TP_GC_S_1_I_0              = 306,
	DEBUGBUS_TP_GC_S_1_I_1              = 307,
	DEBUGBUS_TP_GC_S_1_I_2              = 308,
	DEBUGBUS_TP_GC_S_1_I_3              = 309,
	DEBUGBUS_RB_GC_S_1_I_0              = 318,
	DEBUGBUS_RB_GC_S_1_I_1              = 319,
	DEBUGBUS_CCU_GC_S_1_I_0             = 324,
	DEBUGBUS_CCU_GC_S_1_I_1             = 325,
	DEBUGBUS_HLSQ_GC_S_1_I_0            = 330,
	DEBUGBUS_HLSQ_GC_S_1_I_1            = 331,
	DEBUGBUS_VFD_GC_S_1_I_0             = 336,
	DEBUGBUS_VFD_GC_S_1_I_1             = 337,
	DEBUGBUS_CP_GC_S_2_I_0              = 384,
	DEBUGBUS_PC_BR_S_2_I_0              = 385,
	DEBUGBUS_PC_BV_S_2_I_0              = 386,
	DEBUGBUS_TESS_GC_S_2_I_0            = 387,
	DEBUGBUS_TSEFE_GC_S_2_I_0           = 388,
	DEBUGBUS_TSEBE_GC_S_2_I_0           = 389,
	DEBUGBUS_RAS_GC_S_2_I_0             = 390,
	DEBUGBUS_LRZ_BR_S_2_I_0             = 391,
	DEBUGBUS_LRZ_BV_S_2_I_0             = 392,
	DEBUGBUS_VFDP_GC_S_2_I_0            = 393,
	DEBUGBUS_GPC_BR_S_2_I_0             = 394,
	DEBUGBUS_GPC_BV_S_2_I_0             = 395,
	DEBUGBUS_VPCFE_BR_S_2_I_0           = 396,
	DEBUGBUS_VPCFE_BV_S_2_I_0           = 397,
	DEBUGBUS_VPCBE_BR_S_2_I_0           = 398,
	DEBUGBUS_VPCBE_BV_S_2_I_0           = 399,
	DEBUGBUS_CCHE_GC_S_2_I_0            = 400,
	DEBUGBUS_DBGC_GC_S_2_I_0            = 401,
	DEBUGBUS_LARC_GC_S_2_I_0            = 402,
	DEBUGBUS_RBBM_GC_S_2_I_0            = 403,
	DEBUGBUS_CCRE_GC_S_2_I_0            = 404,
	DEBUGBUS_CGC_GC_S_2_I_0             = 405,
	DEBUGBUS_GMU_GC_S_2_I_0             = 406,
	DEBUGBUS_SLICE_GC_S_2_I_0           = 407,
	DEBUGBUS_HLSQ_SPTP_STAR_GC_S_2_I_0  = 408,
	DEBUGBUS_USP_GC_S_2_I_0             = 416,
	DEBUGBUS_USP_GC_S_2_I_1             = 417,
	DEBUGBUS_USPTP_GC_S_2_I_0           = 422,
	DEBUGBUS_USPTP_GC_S_2_I_1           = 423,
	DEBUGBUS_USPTP_GC_S_2_I_2           = 424,
	DEBUGBUS_USPTP_GC_S_2_I_3           = 425,
	DEBUGBUS_TP_GC_S_2_I_0              = 434,
	DEBUGBUS_TP_GC_S_2_I_1              = 435,
	DEBUGBUS_TP_GC_S_2_I_2              = 436,
	DEBUGBUS_TP_GC_S_2_I_3              = 437,
	DEBUGBUS_RB_GC_S_2_I_0              = 446,
	DEBUGBUS_RB_GC_S_2_I_1              = 447,
	DEBUGBUS_CCU_GC_S_2_I_0             = 452,
	DEBUGBUS_CCU_GC_S_2_I_1             = 453,
	DEBUGBUS_HLSQ_GC_S_2_I_0            = 458,
	DEBUGBUS_HLSQ_GC_S_2_I_1            = 459,
	DEBUGBUS_VFD_GC_S_2_I_0             = 464,
	DEBUGBUS_VFD_GC_S_2_I_1             = 465,
};

static const u32 gen8_debugbus_blocks[] = {
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
	DEBUGBUS_CP_GC_S_2_I_0,
	DEBUGBUS_PC_BR_S_2_I_0,
	DEBUGBUS_PC_BV_S_2_I_0,
	DEBUGBUS_TESS_GC_S_2_I_0,
	DEBUGBUS_TSEFE_GC_S_2_I_0,
	DEBUGBUS_TSEBE_GC_S_2_I_0,
	DEBUGBUS_RAS_GC_S_2_I_0,
	DEBUGBUS_LRZ_BR_S_2_I_0,
	DEBUGBUS_LRZ_BV_S_2_I_0,
	DEBUGBUS_VFDP_GC_S_2_I_0,
	DEBUGBUS_GPC_BR_S_2_I_0,
	DEBUGBUS_GPC_BV_S_2_I_0,
	DEBUGBUS_VPCFE_BR_S_2_I_0,
	DEBUGBUS_VPCFE_BV_S_2_I_0,
	DEBUGBUS_VPCBE_BR_S_2_I_0,
	DEBUGBUS_VPCBE_BV_S_2_I_0,
	DEBUGBUS_CCHE_GC_S_2_I_0,
	DEBUGBUS_DBGC_GC_S_2_I_0,
	DEBUGBUS_LARC_GC_S_2_I_0,
	DEBUGBUS_RBBM_GC_S_2_I_0,
	DEBUGBUS_CCRE_GC_S_2_I_0,
	DEBUGBUS_CGC_GC_S_2_I_0,
	DEBUGBUS_GMU_GC_S_2_I_0,
	DEBUGBUS_SLICE_GC_S_2_I_0,
	DEBUGBUS_HLSQ_SPTP_STAR_GC_S_2_I_0,
	DEBUGBUS_USP_GC_S_2_I_0,
	DEBUGBUS_USP_GC_S_2_I_1,
	DEBUGBUS_USPTP_GC_S_2_I_0,
	DEBUGBUS_USPTP_GC_S_2_I_1,
	DEBUGBUS_USPTP_GC_S_2_I_2,
	DEBUGBUS_USPTP_GC_S_2_I_3,
	DEBUGBUS_TP_GC_S_2_I_0,
	DEBUGBUS_TP_GC_S_2_I_1,
	DEBUGBUS_TP_GC_S_2_I_2,
	DEBUGBUS_TP_GC_S_2_I_3,
	DEBUGBUS_RB_GC_S_2_I_0,
	DEBUGBUS_RB_GC_S_2_I_1,
	DEBUGBUS_CCU_GC_S_2_I_0,
	DEBUGBUS_CCU_GC_S_2_I_1,
	DEBUGBUS_HLSQ_GC_S_2_I_0,
	DEBUGBUS_HLSQ_GC_S_2_I_1,
	DEBUGBUS_VFD_GC_S_2_I_0,
	DEBUGBUS_VFD_GC_S_2_I_1,
};

static const u32 gen8_gbif_debugbus_blocks[] = {
	DEBUGBUS_GBIF_GX_GC_US_I_0,
};

static const u32 gen8_cx_debugbus_blocks[] = {
	DEBUGBUS_GBIF_CX_GC_US_I_0,
	DEBUGBUS_GMU_CX_GC_US_I_0,
	DEBUGBUS_CX_GC_US_I_0,
};

enum gen8_statetype_ids {
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
	SP_INST_DATA_3                 = 31,
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
	HLSQ_DESC_REMAP_META           = 65,
	HLSQ_SLICE_TOP_META            = 66,
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
	HLSQ_SLICE_BACKEND_META        = 94,
	HLSQ_INST_RAM_2                = 95,
	HLSQ_DATAPATH_META             = 96,
	HLSQ_FRONTEND_META             = 97,
	HLSQ_INDIRECT_META             = 98,
	HLSQ_BACKEND_META              = 99,
};

struct gen8_snapshot_block_list {
	/* pre_crashdumper_regs : Registers which need to be dumped before CD runs */
	struct gen8_reg_list *pre_crashdumper_regs;
	/* pre_crashdumper_regs_size : Size of registers which need to be dumped before CD runs */
	size_t num_pre_crashdumper_regs;
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
	/* gmu_cx_unsliced_regs : List of GMU CX unsliced registers */
	const u32 *gmu_cx_unsliced_regs;
	/* gmu_gx_registers : List of GMU registers */
	struct gen8_reg_list *gmu_gx_regs;
	/* num_gmu_gx_regs : Length of GMU registers list */
	size_t num_gmu_gx_regs;
	/* rscc_regs : List of RSCC registers */
	const u32 *rscc_regs;
	/* reg_list : List of GPU internal registers */
	struct gen8_reg_list *reg_list;
	/* reg_list : List of cx_misc registers */
	const u32 *cx_misc_regs;
	/* shader_blocks : List of GPU shader memory */
	struct gen8_shader_block *shader_blocks;
	/* num_shader_blocks : Length of the shader memory list */
	size_t num_shader_blocks;
	/* cp_cluster_registers : List of GPU CP cluster registers */
	struct gen8_cluster_registers *cp_clusters;
	/* num_cp_clusters : Length of GPU CP cluster registers list */
	size_t num_cp_clusters;
	/* cluster_registers : List of GPU cluster registers */
	struct gen8_cluster_registers *clusters;
	/* num_clusters : Length of GPU cluster registers list */
	size_t num_clusters;
	/* spstp_cluster_registers : List of GPU SPTP cluster registers */
	struct gen8_sptp_cluster_registers *sptp_clusters;
	/* num_sptp_clusters : Length of GPU SPTP cluster registers list */
	size_t num_sptp_clusters;
	/* post_crashdumper_regs : Registers which need to be dumped after CD runs */
	const u32 *post_crashdumper_regs;
	/* index_registers : List of index_registers */
	struct gen8_cp_indexed_reg *index_registers;
	/* index_registers_len : Length of the index registers */
	size_t index_registers_len;
	/* mempool_index_registers : List of CP mempool_index_registers */
	struct gen8_cp_indexed_reg *mempool_index_registers;
	/* mempool_index_registers_len : Length of the mempool index registers */
	size_t mempool_index_registers_len;
};

#endif /*__ADRENO_GEN8_SNAPSHOT_H */
