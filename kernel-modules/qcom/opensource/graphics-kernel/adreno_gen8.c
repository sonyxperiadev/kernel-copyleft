// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/qcom/llcc-qcom.h>
#include <soc/qcom/of_common.h>

#include "adreno.h"
#include "adreno_gen8.h"
#include "adreno_gen8_hwsched.h"
#include "adreno_pm4types.h"
#include "adreno_trace.h"
#include "kgsl_pwrscale.h"
#include "kgsl_trace.h"
#include "kgsl_util.h"

/* CP Interrupt bits */
#define GEN8_CP_GLOBAL_INT_HWFAULTBR 0
#define GEN8_CP_GLOBAL_INT_HWFAULTBV 1
#define GEN8_CP_GLOBAL_INT_HWFAULTLPAC 2
#define GEN8_CP_GLOBAL_INT_HWFAULTAQE0 3
#define GEN8_CP_GLOBAL_INT_HWFAULTAQE1 4
#define GEN8_CP_GLOBAL_INT_HWFAULTDDEBR 5
#define GEN8_CP_GLOBAL_INT_HWFAULTDDEBV 6
#define GEN8_CP_GLOBAL_INT_SWFAULTBR 16
#define GEN8_CP_GLOBAL_INT_SWFAULTBV 17
#define GEN8_CP_GLOBAL_INT_SWFAULTLPAC 18
#define GEN8_CP_GLOBAL_INT_SWFAULTAQE0 19
#define GEN8_CP_GLOBAL_INT_SWFAULTAQE1 20
#define GEN8_CP_GLOBAL_INT_SWFAULTDDEBR 21
#define GEN8_CP_GLOBAL_INT_SWFAULTDDEBV 22

#define CP_INTERRUPT_STATUS_MASK_GLOBAL		\
	(BIT(GEN8_CP_GLOBAL_INT_HWFAULTBR) |	\
	 BIT(GEN8_CP_GLOBAL_INT_HWFAULTBV) |	\
	 BIT(GEN8_CP_GLOBAL_INT_HWFAULTLPAC) |	\
	 BIT(GEN8_CP_GLOBAL_INT_HWFAULTAQE0) |	\
	 BIT(GEN8_CP_GLOBAL_INT_HWFAULTAQE1) |	\
	 BIT(GEN8_CP_GLOBAL_INT_HWFAULTDDEBR) |	\
	 BIT(GEN8_CP_GLOBAL_INT_HWFAULTDDEBV) |	\
	 BIT(GEN8_CP_GLOBAL_INT_SWFAULTBR) |	\
	 BIT(GEN8_CP_GLOBAL_INT_SWFAULTBV) |	\
	 BIT(GEN8_CP_GLOBAL_INT_SWFAULTLPAC) |	\
	 BIT(GEN8_CP_GLOBAL_INT_SWFAULTAQE0) |	\
	 BIT(GEN8_CP_GLOBAL_INT_SWFAULTAQE1) |	\
	 BIT(GEN8_CP_GLOBAL_INT_SWFAULTDDEBR) |	\
	 BIT(GEN8_CP_GLOBAL_INT_SWFAULTDDEBV))

/* CP HW Fault status bits */
#define CP_HW_RBFAULT 0
#define CP_HW_IB1FAULT 1
#define CP_HW_IB2FAULT 2
#define CP_HW_IB3FAULT 3
#define CP_HW_SDSFAULT 4
#define CP_HW_MRBFAULT 5
#define CP_HW_VSDFAULT 6
#define CP_HW_SQEREADBRUSTOVF 8
#define CP_HW_EVENTENGINEOVF 9
#define CP_HW_UCODEERROR 10

#define CP_HW_FAULT_STATUS_MASK_PIPE	\
	(BIT(CP_HW_RBFAULT) |		\
	 BIT(CP_HW_IB1FAULT) |		\
	 BIT(CP_HW_IB2FAULT) |		\
	 BIT(CP_HW_IB3FAULT) |		\
	 BIT(CP_HW_SDSFAULT) |		\
	 BIT(CP_HW_MRBFAULT) |		\
	 BIT(CP_HW_VSDFAULT) |		\
	 BIT(CP_HW_SQEREADBRUSTOVF) |	\
	 BIT(CP_HW_EVENTENGINEOVF) |	\
	 BIT(CP_HW_UCODEERROR))

/* CP SW Fault status bits */
#define CP_SW_CSFRBWRAP 0
#define CP_SW_CSFIB1WRAP 1
#define CP_SW_CSFIB2WRAP 2
#define CP_SW_CSFIB3WRAP 3
#define CP_SW_SDSWRAP 4
#define CP_SW_MRBWRAP 5
#define CP_SW_VSDWRAP 6
#define CP_SW_OPCODEERROR 8
#define CP_SW_VSDPARITYERROR 9
#define CP_SW_REGISTERPROTECTIONERROR 10
#define CP_SW_ILLEGALINSTRUCTION 11
#define CP_SW_SMMUFAULT 12
#define CP_SW_VBIFRESPCLIENT 13
#define CP_SW_VBIFRESPTYPE 19
#define CP_SW_VBIFRESPREAD 21
#define CP_SW_VBIFRESP 22
#define CP_SW_RTWROVF 23
#define CP_SW_LRZRTWROVF 24
#define CP_SW_LRZRTREFCNTOVF 25
#define CP_SW_LRZRTCLRRESMISS 26

#define CP_SW_FAULT_STATUS_MASK_PIPE		\
	(BIT(CP_SW_CSFRBWRAP) |			\
	 BIT(CP_SW_CSFIB1WRAP) |		\
	 BIT(CP_SW_CSFIB2WRAP) |		\
	 BIT(CP_SW_CSFIB3WRAP) |		\
	 BIT(CP_SW_SDSWRAP) |			\
	 BIT(CP_SW_MRBWRAP) |			\
	 BIT(CP_SW_VSDWRAP) |			\
	 BIT(CP_SW_OPCODEERROR) |		\
	 BIT(CP_SW_VSDPARITYERROR) |		\
	 BIT(CP_SW_REGISTERPROTECTIONERROR) |	\
	 BIT(CP_SW_ILLEGALINSTRUCTION) |	\
	 BIT(CP_SW_SMMUFAULT) |			\
	 BIT(CP_SW_VBIFRESPCLIENT) |		\
	 BIT(CP_SW_VBIFRESPTYPE) |		\
	 BIT(CP_SW_VBIFRESPREAD) |		\
	 BIT(CP_SW_VBIFRESP) |			\
	 BIT(CP_SW_RTWROVF) |			\
	 BIT(CP_SW_LRZRTWROVF) |		\
	 BIT(CP_SW_LRZRTREFCNTOVF) |		\
	 BIT(CP_SW_LRZRTCLRRESMISS))

/* IFPC & Preemption static powerup restore list */
static const u32 gen8_pwrup_reglist[] = {
	GEN8_UCHE_MODE_CNTL,
	GEN8_UCHE_VARB_IDLE_TIMEOUT,
	GEN8_UCHE_GBIF_GX_CONFIG,
	GEN8_UCHE_CACHE_WAYS,
	GEN8_UCHE_CCHE_MODE_CNTL,
	GEN8_UCHE_CCHE_CACHE_WAYS,
	GEN8_UCHE_CCHE_GC_GMEM_RANGE_MIN_LO,
	GEN8_UCHE_CCHE_GC_GMEM_RANGE_MIN_HI,
	GEN8_UCHE_CCHE_LPAC_GMEM_RANGE_MIN_LO,
	GEN8_UCHE_CCHE_LPAC_GMEM_RANGE_MIN_HI,
	GEN8_UCHE_HW_DBG_CNTL,
	GEN8_UCHE_WRITE_THRU_BASE_LO,
	GEN8_UCHE_WRITE_THRU_BASE_HI,
	GEN8_UCHE_TRAP_BASE_LO,
	GEN8_UCHE_TRAP_BASE_HI,
	GEN8_UCHE_CLIENT_PF,
	GEN8_VSC_KMD_DBG_ECO_CNTL,
	GEN8_RB_CMP_NC_MODE_CNTL,
	GEN8_SP_HLSQ_TIMEOUT_THRESHOLD_DP,
	GEN8_SP_HLSQ_GC_GMEM_RANGE_MIN_LO,
	GEN8_SP_HLSQ_GC_GMEM_RANGE_MIN_HI,
	GEN8_SP_READ_SEL,
};

/* IFPC & Preemption static powerup restore list for gen8_3_0 */
static const u32 gen8_3_0_pwrup_reglist[] = {
	GEN8_UCHE_MODE_CNTL,
	GEN8_UCHE_VARB_IDLE_TIMEOUT,
	GEN8_UCHE_GBIF_GX_CONFIG,
	GEN8_UCHE_CACHE_WAYS,
	GEN8_UCHE_CCHE_MODE_CNTL,
	GEN8_UCHE_CCHE_CACHE_WAYS,
	GEN8_UCHE_CCHE_GC_GMEM_RANGE_MIN_LO,
	GEN8_UCHE_CCHE_GC_GMEM_RANGE_MIN_HI,
	GEN8_UCHE_HW_DBG_CNTL,
	GEN8_UCHE_WRITE_THRU_BASE_LO,
	GEN8_UCHE_WRITE_THRU_BASE_HI,
	GEN8_UCHE_TRAP_BASE_LO,
	GEN8_UCHE_TRAP_BASE_HI,
	GEN8_UCHE_CLIENT_PF,
	GEN8_RB_CMP_NC_MODE_CNTL,
	GEN8_SP_HLSQ_TIMEOUT_THRESHOLD_DP,
	GEN8_SP_HLSQ_GC_GMEM_RANGE_MIN_LO,
	GEN8_SP_HLSQ_GC_GMEM_RANGE_MIN_HI,
	GEN8_SP_READ_SEL,
};

/* IFPC only static powerup restore list */
static const u32 gen8_ifpc_pwrup_reglist[] = {
	GEN8_RBBM_NC_MODE_CNTL,
	GEN8_RBBM_SLICE_INTERFACE_HANG_INT_CNTL,
	GEN8_RBBM_SLICE_NC_MODE_CNTL,
	GEN8_SP_NC_MODE_CNTL,
	GEN8_SP_HLSQ_LPAC_GMEM_RANGE_MIN_LO,
	GEN8_SP_HLSQ_LPAC_GMEM_RANGE_MIN_HI,
	GEN8_SP_CHICKEN_BITS_1,
	GEN8_SP_CHICKEN_BITS_2,
	GEN8_SP_CHICKEN_BITS_3,
	GEN8_SP_PERFCTR_SHADER_MASK,
	GEN8_TPL1_NC_MODE_CNTL,
	GEN8_TPL1_DBG_ECO_CNTL,
	GEN8_TPL1_DBG_ECO_CNTL1,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_1,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_2,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_3,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_4,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_5,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_6,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_7,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_8,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_9,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_10,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_11,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_12,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_13,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_14,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_15,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_16,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_17,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_18,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_19,
	GEN8_CP_PROTECT_REG_GLOBAL,
	GEN8_CP_PROTECT_REG_GLOBAL + 1,
	GEN8_CP_PROTECT_REG_GLOBAL + 2,
	GEN8_CP_PROTECT_REG_GLOBAL + 3,
	GEN8_CP_PROTECT_REG_GLOBAL + 4,
	GEN8_CP_PROTECT_REG_GLOBAL + 5,
	GEN8_CP_PROTECT_REG_GLOBAL + 6,
	GEN8_CP_PROTECT_REG_GLOBAL + 7,
	GEN8_CP_PROTECT_REG_GLOBAL + 8,
	GEN8_CP_PROTECT_REG_GLOBAL + 9,
	GEN8_CP_PROTECT_REG_GLOBAL + 10,
	GEN8_CP_PROTECT_REG_GLOBAL + 11,
	GEN8_CP_PROTECT_REG_GLOBAL + 12,
	GEN8_CP_PROTECT_REG_GLOBAL + 13,
	GEN8_CP_PROTECT_REG_GLOBAL + 14,
	GEN8_CP_PROTECT_REG_GLOBAL + 15,
	GEN8_CP_PROTECT_REG_GLOBAL + 16,
	GEN8_CP_PROTECT_REG_GLOBAL + 17,
	GEN8_CP_PROTECT_REG_GLOBAL + 18,
	GEN8_CP_PROTECT_REG_GLOBAL + 19,
	GEN8_CP_PROTECT_REG_GLOBAL + 20,
	GEN8_CP_PROTECT_REG_GLOBAL + 21,
	GEN8_CP_PROTECT_REG_GLOBAL + 22,
	GEN8_CP_PROTECT_REG_GLOBAL + 23,
	GEN8_CP_PROTECT_REG_GLOBAL + 24,
	GEN8_CP_PROTECT_REG_GLOBAL + 25,
	GEN8_CP_PROTECT_REG_GLOBAL + 26,
	GEN8_CP_PROTECT_REG_GLOBAL + 27,
	GEN8_CP_PROTECT_REG_GLOBAL + 28,
	GEN8_CP_PROTECT_REG_GLOBAL + 29,
	GEN8_CP_PROTECT_REG_GLOBAL + 30,
	GEN8_CP_PROTECT_REG_GLOBAL + 31,
	GEN8_CP_PROTECT_REG_GLOBAL + 32,
	GEN8_CP_PROTECT_REG_GLOBAL + 33,
	GEN8_CP_PROTECT_REG_GLOBAL + 34,
	GEN8_CP_PROTECT_REG_GLOBAL + 35,
	GEN8_CP_PROTECT_REG_GLOBAL + 36,
	GEN8_CP_PROTECT_REG_GLOBAL + 37,
	GEN8_CP_PROTECT_REG_GLOBAL + 38,
	GEN8_CP_PROTECT_REG_GLOBAL + 39,
	GEN8_CP_PROTECT_REG_GLOBAL + 40,
	GEN8_CP_PROTECT_REG_GLOBAL + 41,
	GEN8_CP_PROTECT_REG_GLOBAL + 42,
	GEN8_CP_PROTECT_REG_GLOBAL + 43,
	GEN8_CP_PROTECT_REG_GLOBAL + 44,
	GEN8_CP_PROTECT_REG_GLOBAL + 45,
	GEN8_CP_PROTECT_REG_GLOBAL + 46,
	GEN8_CP_PROTECT_REG_GLOBAL + 63,
	GEN8_CP_INTERRUPT_STATUS_MASK_GLOBAL,
};

/* IFPC only static powerup restore list for gen8_3_0*/
static const u32 gen8_3_0_ifpc_pwrup_reglist[] = {
	GEN8_RBBM_NC_MODE_CNTL,
	GEN8_RBBM_SLICE_INTERFACE_HANG_INT_CNTL,
	GEN8_RBBM_SLICE_NC_MODE_CNTL,
	GEN8_SP_NC_MODE_CNTL,
	GEN8_SP_CHICKEN_BITS_2,
	GEN8_SP_CHICKEN_BITS_3,
	GEN8_SP_PERFCTR_SHADER_MASK,
	GEN8_TPL1_NC_MODE_CNTL,
	GEN8_TPL1_DBG_ECO_CNTL,
	GEN8_TPL1_DBG_ECO_CNTL1,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_1,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_2,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_3,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_4,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_5,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_6,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_7,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_8,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_9,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_10,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_11,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_12,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_13,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_14,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_15,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_16,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_17,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_18,
	GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_19,
	GEN8_CP_PROTECT_REG_GLOBAL,
	GEN8_CP_PROTECT_REG_GLOBAL + 1,
	GEN8_CP_PROTECT_REG_GLOBAL + 2,
	GEN8_CP_PROTECT_REG_GLOBAL + 3,
	GEN8_CP_PROTECT_REG_GLOBAL + 4,
	GEN8_CP_PROTECT_REG_GLOBAL + 5,
	GEN8_CP_PROTECT_REG_GLOBAL + 6,
	GEN8_CP_PROTECT_REG_GLOBAL + 7,
	GEN8_CP_PROTECT_REG_GLOBAL + 8,
	GEN8_CP_PROTECT_REG_GLOBAL + 9,
	GEN8_CP_PROTECT_REG_GLOBAL + 10,
	GEN8_CP_PROTECT_REG_GLOBAL + 11,
	GEN8_CP_PROTECT_REG_GLOBAL + 12,
	GEN8_CP_PROTECT_REG_GLOBAL + 13,
	GEN8_CP_PROTECT_REG_GLOBAL + 14,
	GEN8_CP_PROTECT_REG_GLOBAL + 15,
	GEN8_CP_PROTECT_REG_GLOBAL + 16,
	GEN8_CP_PROTECT_REG_GLOBAL + 17,
	GEN8_CP_PROTECT_REG_GLOBAL + 18,
	GEN8_CP_PROTECT_REG_GLOBAL + 19,
	GEN8_CP_PROTECT_REG_GLOBAL + 20,
	GEN8_CP_PROTECT_REG_GLOBAL + 21,
	GEN8_CP_PROTECT_REG_GLOBAL + 22,
	GEN8_CP_PROTECT_REG_GLOBAL + 23,
	GEN8_CP_PROTECT_REG_GLOBAL + 24,
	GEN8_CP_PROTECT_REG_GLOBAL + 25,
	GEN8_CP_PROTECT_REG_GLOBAL + 26,
	GEN8_CP_PROTECT_REG_GLOBAL + 27,
	GEN8_CP_PROTECT_REG_GLOBAL + 28,
	GEN8_CP_PROTECT_REG_GLOBAL + 29,
	GEN8_CP_PROTECT_REG_GLOBAL + 30,
	GEN8_CP_PROTECT_REG_GLOBAL + 31,
	GEN8_CP_PROTECT_REG_GLOBAL + 32,
	GEN8_CP_PROTECT_REG_GLOBAL + 33,
	GEN8_CP_PROTECT_REG_GLOBAL + 34,
	GEN8_CP_PROTECT_REG_GLOBAL + 35,
	GEN8_CP_PROTECT_REG_GLOBAL + 36,
	GEN8_CP_PROTECT_REG_GLOBAL + 37,
	GEN8_CP_PROTECT_REG_GLOBAL + 38,
	GEN8_CP_PROTECT_REG_GLOBAL + 39,
	GEN8_CP_PROTECT_REG_GLOBAL + 40,
	GEN8_CP_PROTECT_REG_GLOBAL + 41,
	GEN8_CP_PROTECT_REG_GLOBAL + 42,
	GEN8_CP_PROTECT_REG_GLOBAL + 43,
	GEN8_CP_PROTECT_REG_GLOBAL + 44,
	GEN8_CP_PROTECT_REG_GLOBAL + 45,
	GEN8_CP_PROTECT_REG_GLOBAL + 46,
	GEN8_CP_PROTECT_REG_GLOBAL + 63,
};

static const struct gen8_pwrup_extlist gen8_0_0_pwrup_extlist[] = {
	{ GEN8_CP_HW_FAULT_STATUS_MASK_PIPE, BIT(PIPE_BR) | BIT(PIPE_BV) | BIT(PIPE_LPAC)
		| BIT(PIPE_AQE0) | BIT(PIPE_AQE1) | BIT(PIPE_DDE_BR) | BIT(PIPE_DDE_BV) },
	{ GEN8_CP_INTERRUPT_STATUS_MASK_PIPE, BIT(PIPE_BR) | BIT(PIPE_BV) | BIT(PIPE_LPAC)
		| BIT(PIPE_AQE0) | BIT(PIPE_AQE1) | BIT(PIPE_DDE_BR) | BIT(PIPE_DDE_BV) },
	{ GEN8_CP_PROTECT_CNTL_PIPE, BIT(PIPE_BR) | BIT(PIPE_BV) | BIT(PIPE_LPAC)},
	{ GEN8_CP_PROTECT_REG_PIPE + 15, BIT(PIPE_BR) | BIT(PIPE_BV) | BIT(PIPE_LPAC)},
	{ GEN8_GRAS_TSEFE_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_GRAS_NC_MODE_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_GRAS_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_RB_CCU_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_CCU_NC_MODE_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_CMP_NC_MODE_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_RESOLVE_PREFETCH_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_CMP_DBG_ECO_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_GC_GMEM_PROTECT, BIT(PIPE_BR)},
	{ GEN8_RB_LPAC_GMEM_PROTECT, BIT(PIPE_BR)},
	{ GEN8_RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE, BIT(PIPE_BR)},
	{ GEN8_VPC_FLATSHADE_MODE_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_1, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_2, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_3, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_4, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_AUTO_VERTEX_STRIDE, BIT(PIPE_BR) | BIT(PIPE_BV)},
	{ GEN8_PC_VIS_STREAM_CNTL, BIT(PIPE_BR) | BIT(PIPE_BV)},
	{ GEN8_PC_CONTEXT_SWITCH_STABILIZE_CNTL_1, BIT(PIPE_BR) | BIT(PIPE_BV)},
	{ GEN8_VFD_CB_BV_THRESHOLD, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_CB_BR_THRESHOLD, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_CB_BUSY_REQ_CNT, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_CB_LP_REQ_CNT, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_DBG_ECO_CNTL, BIT(PIPE_BR) | BIT(PIPE_BV)},
};

static const struct gen8_pwrup_extlist gen8_3_0_pwrup_extlist[] = {
	{ GEN8_CP_PROTECT_CNTL_PIPE, BIT(PIPE_BR) | BIT(PIPE_BV) },
	{ GEN8_CP_PROTECT_REG_PIPE + 15, BIT(PIPE_BR) | BIT(PIPE_BV) },
	{ GEN8_GRAS_TSEFE_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_GRAS_NC_MODE_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_GRAS_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_RB_CCU_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_CCU_NC_MODE_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_CMP_NC_MODE_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_RESOLVE_PREFETCH_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_CMP_DBG_ECO_CNTL, BIT(PIPE_BR)},
	{ GEN8_RB_GC_GMEM_PROTECT, BIT(PIPE_BR)},
	{ GEN8_RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE, BIT(PIPE_BR)},
	{ GEN8_VPC_FLATSHADE_MODE_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_1, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_2, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_3, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_CHICKEN_BITS_4, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_PC_AUTO_VERTEX_STRIDE, BIT(PIPE_BR) | BIT(PIPE_BV)},
	{ GEN8_PC_VIS_STREAM_CNTL, BIT(PIPE_BR) | BIT(PIPE_BV)},
	{ GEN8_PC_CONTEXT_SWITCH_STABILIZE_CNTL_1, BIT(PIPE_BR) | BIT(PIPE_BV)},
	{ GEN8_VFD_CB_BV_THRESHOLD, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_CB_BR_THRESHOLD, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_CB_BUSY_REQ_CNT, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_CB_LP_REQ_CNT, BIT(PIPE_BV) | BIT(PIPE_BR)},
	{ GEN8_VFD_DBG_ECO_CNTL, BIT(PIPE_BR) | BIT(PIPE_BV)},
};

struct gen8_nonctxt_overrides gen8_nc_overrides[] = {
	{ GEN8_UCHE_MODE_CNTL, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_UCHE_CACHE_WAYS, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_UCHE_CLIENT_PF, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_UCHE_DBG_ECO_CNTL_0, BIT(PIPE_NONE), 0, 0, 2, },
	{ GEN8_UCHE_HW_DBG_CNTL, BIT(PIPE_NONE), 0, 0, 2, },
	{ GEN8_UCHE_CCHE_HW_DBG_CNTL, BIT(PIPE_NONE), 0, 0, 2, },
	{ GEN8_GRAS_NC_MODE_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_GRAS_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_RB_DBG_ECO_CNTL, BIT(PIPE_BR), 0, 0, 3, },
	{ GEN8_RB_CCU_DBG_ECO_CNTL, BIT(PIPE_BR), 0, 0, 3, },
	{ GEN8_RB_CCU_CNTL, BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_RB_CCU_NC_MODE_CNTL, BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_RB_SLICE_UFC_PREFETCH_CNTL, BIT(PIPE_BR), 0, 0, 3, },
	{ GEN8_RB_SLICE_UFC_DBG_CNTL, BIT(PIPE_BR), 0, 0, 3, },
	{ GEN8_RB_CMP_NC_MODE_CNTL, BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_RB_RESOLVE_PREFETCH_CNTL, BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_RB_CMP_DBG_ECO_CNTL, BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_RB_UFC_DBG_CNTL, BIT(PIPE_BR), 0, 0, 3, },
	{ GEN8_PC_CHICKEN_BITS_1, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_PC_CHICKEN_BITS_2, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_PC_CHICKEN_BITS_3, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_PC_CHICKEN_BITS_4, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_PC_CHICKEN_BITS_5, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 2, },
	{ GEN8_PC_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 3, },
	{ GEN8_VFD_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_VFD_CB_BV_THRESHOLD, BIT(PIPE_BV) | BIT(PIPE_BR),  0, 0, 0, },
	{ GEN8_VFD_CB_BR_THRESHOLD, BIT(PIPE_BV) | BIT(PIPE_BR),  0, 0, 0, },
	{ GEN8_VFD_CB_LP_REQ_CNT, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_VFD_CB_BUSY_REQ_CNT, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_VPC_DBG_ECO_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 2, },
	{ GEN8_VPC_DBG_ECO_CNTL_1, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 2, },
	{ GEN8_VPC_DBG_ECO_CNTL_2, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 1, },
	{ GEN8_VPC_DBG_ECO_CNTL_3, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 2, },
	{ GEN8_VPC_FLATSHADE_MODE_CNTL, BIT(PIPE_BV) | BIT(PIPE_BR), 0, 0, 0, },
	{ GEN8_SP_DBG_ECO_CNTL, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_SP_NC_MODE_CNTL, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_SP_CHICKEN_BITS, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_SP_NC_MODE_CNTL_2, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_SP_CHICKEN_BITS_1, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_SP_CHICKEN_BITS_2, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_SP_CHICKEN_BITS_3, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_SP_CHICKEN_BITS_4, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_SP_DISPATCH_CNTL, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_SP_HLSQ_DBG_ECO_CNTL, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_SP_DBG_CNTL, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_TPL1_NC_MODE_CNTL, BIT(PIPE_NONE), 0, 0, 1, },
	{ GEN8_TPL1_DBG_ECO_CNTL, BIT(PIPE_NONE), 0, 0, 0, },
	{ GEN8_TPL1_DBG_ECO_CNTL1, BIT(PIPE_NONE), 0, 0, 0, },
	{ 0 }
};

static int acd_calibrate_set(void *data, u64 val)
{
	struct kgsl_device *device = data;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	u32 debug_val = (u32) val;
	int ret;

	mutex_lock(&device->mutex);
	ret = adreno_active_count_get(adreno_dev);
	if (ret)
		goto err;

	ret = gen8_hfi_send_set_value(adreno_dev, HFI_VALUE_DBG,
				F_PWR_ACD_CALIBRATE, debug_val);
	if (!ret)
		gmu->acd_debug_val = debug_val;

	adreno_active_count_put(adreno_dev);
err:
	mutex_unlock(&device->mutex);
	return ret;
}

static int acd_calibrate_get(void *data, u64 *val)
{
	struct kgsl_device *device = data;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);

	*val = (u64) gmu->acd_debug_val;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(acd_cal_fops, acd_calibrate_get, acd_calibrate_set, "%llu\n");

static ssize_t nc_override_get(struct file *filep,
		char __user *user_buf, size_t len, loff_t *off)
{
	struct kgsl_device *device = (struct kgsl_device *) filep->private_data;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct gen8_device *gen8_dev = container_of(adreno_dev,
					struct gen8_device, adreno_dev);
	struct gen8_nonctxt_overrides *nc_overrides = gen8_dev->nc_overrides;
	u32 i, max_size = PAGE_SIZE;
	char *buf, *pos;
	ssize_t size = 0;

	if (!gen8_dev->nc_overrides_enabled || !nc_overrides)
		return 0;

	buf = kzalloc(max_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pos = buf;

	mutex_lock(&gen8_dev->nc_mutex);
	/* Copy all assignments from list to str */
	for (i = 0; nc_overrides[i].offset; i++) {
		if (nc_overrides[i].set) {
			len = scnprintf(pos, max_size, "0x%x:0x%8.8x\n",
					nc_overrides[i].offset, nc_overrides[i].val);
			/* If we run out of space len will be zero */
			if (len == 0)
				break;
			max_size -= len;
			pos += len;
		}
	}
	mutex_unlock(&gen8_dev->nc_mutex);

	size = simple_read_from_buffer(user_buf, len, off, buf, pos - buf);

	kfree(buf);
	return size;
}

static void nc_override_cb(struct adreno_device *adreno_dev, void *priv)
{
	struct gen8_device *gen8_dev = container_of(adreno_dev, struct gen8_device, adreno_dev);

	gen8_dev->nc_overrides_enabled = true;
	/* Force to update and make new patched reglist */
	adreno_dev->patch_reglist = false;
}

static ssize_t nc_override_set(struct file *filep,
		const char __user *user_buf, size_t len, loff_t *off)
{
	struct kgsl_device *device = (struct kgsl_device *) filep->private_data;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct gen8_device *gen8_dev = container_of(adreno_dev, struct gen8_device, adreno_dev);
	struct gen8_nonctxt_overrides *nc_overrides = gen8_dev->nc_overrides;
	u32 i, offset, val;
	int ret = -EINVAL;
	ssize_t size = 0;
	char *buf;

	if (!nc_overrides)
		return 0;

	if ((len >= PAGE_SIZE) || (len == 0))
		return -EINVAL;

	buf = kzalloc(len + 1, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	if (copy_from_user(buf, user_buf, len)) {
		ret = -EFAULT;
		goto err;
	}

	/* For sanity and parsing, ensure it is null terminated */
	buf[len] = '\0';

	size = sscanf(buf, "0x%x:0x%x", &offset, &val);
	if (size == 0)
		goto err;

	size = 0;

	mutex_lock(&gen8_dev->nc_mutex);
	for (i = 0; nc_overrides[i].offset; i++) {
		if (nc_overrides[i].offset == offset) {
			nc_overrides[i].val = val;
			nc_overrides[i].set = true;
			size = len;
			break;
		}
	}
	mutex_unlock(&gen8_dev->nc_mutex);

	if (size > 0) {
		ret = adreno_power_cycle(ADRENO_DEVICE(device), nc_override_cb, NULL);
		if (!ret)
			ret = size;
	}

err:
	kfree(buf);
	return ret;
}

static const struct file_operations nc_override_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = nc_override_get,
	.write = nc_override_set,
	.llseek = noop_llseek,
};

void gen8_cp_init_cmds(struct adreno_device *adreno_dev, u32 *cmds)
{
	u32 i = 0, mask = 0;

	/* Disable concurrent binning before sending CP init */
	cmds[i++] = cp_type7_packet(CP_THREAD_CONTROL, 1);
	cmds[i++] = BIT(27);

	/* Use multiple HW contexts */
	mask |= BIT(0);

	/* Enable error detection */
	mask |= BIT(1);

	/* Set default reset state */
	mask |= BIT(3);

	/* Disable save/restore of performance counters across preemption */
	mask |= BIT(6);

	/* Enable the register init list with the spinlock */
	mask |= BIT(8);

	cmds[i++] = cp_type7_packet(CP_ME_INIT, 7);

	/* Enabled ordinal mask */
	cmds[i++] = mask;
	cmds[i++] = 0x00000003; /* Set number of HW contexts */
	cmds[i++] = 0x20000000; /* Enable error detection */
	cmds[i++] = 0x00000002; /* Operation mode mask */

	/* Register initialization list with spinlock */
	cmds[i++] = lower_32_bits(adreno_dev->pwrup_reglist->gpuaddr);
	cmds[i++] = upper_32_bits(adreno_dev->pwrup_reglist->gpuaddr);
	/*
	 * Gen8 targets with concurrent binning are expected to have a dynamic
	 * power up list with triplets which contains the pipe id in it.
	 * Bit 31 of POWER_UP_REGISTER_LIST_LENGTH is reused here to let CP
	 * know if the power up contains the triplets. If
	 * REGISTER_INIT_LIST_WITH_SPINLOCK is set and bit 31 below is set,
	 * CP expects a dynamic list with triplets.
	 */
	cmds[i++] = BIT(31);
}

int gen8_fenced_write(struct adreno_device *adreno_dev, u32 offset,
		u32 value, u32 mask)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	u32 status, i;
	u64 ts1, ts2;

	kgsl_regwrite(device, offset, value);
	ts1 = gpudev->read_alwayson(adreno_dev);
	for (i = 0; i < GMU_CORE_LONG_WAKEUP_RETRY_LIMIT; i++) {
		/*
		 * Make sure the previous register write is posted before
		 * checking the fence status
		 */
		mb();

		gmu_core_regread(device, GEN8_GMUAO_AHB_FENCE_STATUS, &status);

		/*
		 * If !writedropped0/1, then the write to fenced register
		 * was successful
		 */
		if (!(status & mask))
			break;

		/* Wait a small amount of time before trying again */
		udelay(GMU_CORE_WAKEUP_DELAY_US);

		/* Try to write the fenced register again */
		kgsl_regwrite(device, offset, value);
	}

	if (i < GMU_CORE_SHORT_WAKEUP_RETRY_LIMIT)
		return 0;

	if (i == GMU_CORE_LONG_WAKEUP_RETRY_LIMIT) {
		ts2 = gpudev->read_alwayson(adreno_dev);
		dev_err(device->dev,
				"Timed out waiting %d usecs to write fenced register 0x%x, timestamps: %llx %llx\n",
				i * GMU_CORE_WAKEUP_DELAY_US, offset, ts1, ts2);
		return -ETIMEDOUT;
	}

	dev_info(device->dev,
		"Waited %d usecs to write fenced register 0x%x\n",
		i * GMU_CORE_WAKEUP_DELAY_US, offset);

	return 0;
}

int gen8_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen8_device *gen8_dev = container_of(adreno_dev,
					struct gen8_device, adreno_dev);
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	u64 freq = gen8_core->gmu_hub_clk_freq;

	adreno_dev->highest_bank_bit = gen8_core->highest_bank_bit;
	adreno_dev->gmu_hub_clk_freq = freq ? freq : 150000000;
	adreno_dev->ahb_timeout_val = adreno_get_ahb_timeout_val(adreno_dev,
			gen8_core->noc_timeout_us);
	adreno_dev->bcl_data = gen8_core->bcl_data;

	adreno_dev->cooperative_reset = ADRENO_FEATURE(adreno_dev,
			ADRENO_COOP_RESET);

	/* If the memory type is DDR 4, override the existing configuration */
	if (of_fdt_get_ddrtype() == 0x7)
		adreno_dev->highest_bank_bit = 14;

	gen8_crashdump_init(adreno_dev);

	gen8_dev->nc_overrides = gen8_nc_overrides;
	mutex_init(&gen8_dev->nc_mutex);

	/* Debugfs node for noncontext registers override */
	debugfs_create_file("nc_override", 0644, device->d_debugfs, device, &nc_override_fops);

	return adreno_allocate_global(device, &adreno_dev->pwrup_reglist,
		PAGE_SIZE, 0, 0, KGSL_MEMDESC_PRIVILEGED,
		"powerup_register_list");
}

#define CX_TIMER_INIT_SAMPLES 16
void gen8_cx_timer_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u64 seed_val, tmr, skew = 0;
	int i;
	unsigned long flags;

	/* Set it up during first boot or after suspend resume */
	if (test_bit(ADRENO_DEVICE_CX_TIMER_INITIALIZED, &adreno_dev->priv))
		return;

	/* Disable irqs to get accurate timings */
	local_irq_save(flags);

	/* Calculate the overhead of timer reads and register writes */
	for (i = 0; i < CX_TIMER_INIT_SAMPLES; i++) {
		u64 tmr1, tmr2, tmr3;

		/* Measure time for two reads of the CPU timer */
		tmr1 = arch_timer_read_counter();
		tmr2 = arch_timer_read_counter();

		/* Write to the register and time it */
		kgsl_regwrite(device, GEN8_GPU_CX_MISC_AO_COUNTER_LO, lower_32_bits(tmr2));
		kgsl_regwrite(device, GEN8_GPU_CX_MISC_AO_COUNTER_HI, upper_32_bits(tmr2));

		/* Barrier to make sure the write completes before timing it */
		mb();
		tmr3 = arch_timer_read_counter();

		/* Calculate difference between register write and CPU timer */
		skew += (tmr3 - tmr2) - (tmr2 - tmr1);
	}

	local_irq_restore(flags);

	/* Get the average over all our readings, to the closest integer */
	skew = (skew + CX_TIMER_INIT_SAMPLES / 2) / CX_TIMER_INIT_SAMPLES;

	local_irq_save(flags);
	tmr = arch_timer_read_counter();

	seed_val = tmr + skew;

	/* Seed the GPU CX counter with the adjusted timer */
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_AO_COUNTER_LO, lower_32_bits(seed_val));
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_AO_COUNTER_HI, upper_32_bits(seed_val));

	local_irq_restore(flags);

	set_bit(ADRENO_DEVICE_CX_TIMER_INITIALIZED, &adreno_dev->priv);
}

void gen8_get_gpu_feature_info(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 feature_fuse = 0;

	/* Get HW feature soft fuse value */
	kgsl_regread(device, GEN8_GPU_CX_MISC_SW_FUSE_VALUE, &feature_fuse);

	adreno_dev->fastblend_enabled = feature_fuse & BIT(GEN8_FASTBLEND_SW_FUSE);
	adreno_dev->raytracing_enabled = feature_fuse & BIT(GEN8_RAYTRACING_SW_FUSE);

	/* If software enables LPAC without HW support, disable it */
	if (ADRENO_FEATURE(adreno_dev, ADRENO_LPAC))
		adreno_dev->lpac_enabled = feature_fuse & BIT(GEN8_LPAC_SW_FUSE);

	adreno_dev->feature_fuse = feature_fuse;
}

void gen8_host_aperture_set(struct adreno_device *adreno_dev, u32 pipe_id,
		u32 slice_id, u32 use_slice_id)
{
	struct gen8_device *gen8_dev = container_of(adreno_dev,
					struct gen8_device, adreno_dev);
	u32 aperture_val = (FIELD_PREP(GENMASK(15, 12), pipe_id) |
			    FIELD_PREP(GENMASK(18, 16), slice_id) |
			    FIELD_PREP(GENMASK(23, 23), use_slice_id));

	/* Check if we already set the aperture */
	if (gen8_dev->aperture == aperture_val)
		return;

	kgsl_regwrite(KGSL_DEVICE(adreno_dev), GEN8_CP_APERTURE_CNTL_HOST, aperture_val);

	/* Make sure the aperture write goes through before reading the registers */
	mb();

	gen8_dev->aperture = aperture_val;
}

void gen8_regread64_aperture(struct kgsl_device *device,
	u32 offsetwords_lo, u32 offsetwords_hi, u64 *value, u32 pipe,
	u32 slice_id, u32 use_slice_id)
{
	u32 val_lo = 0, val_hi = 0;

	gen8_host_aperture_set(ADRENO_DEVICE(device), pipe, slice_id, use_slice_id);

	val_lo = kgsl_regmap_read(&device->regmap, offsetwords_lo);
	val_hi = kgsl_regmap_read(&device->regmap, offsetwords_hi);

	*value = (((u64)val_hi << 32) | val_lo);
}

void gen8_regread_aperture(struct kgsl_device *device,
	u32 offsetwords, u32 *value, u32 pipe, u32 slice_id, u32 use_slice_id)
{
	gen8_host_aperture_set(ADRENO_DEVICE(device), pipe, slice_id, use_slice_id);

	*value = kgsl_regmap_read(&device->regmap, offsetwords);
}

static inline void gen8_regwrite_aperture(struct kgsl_device *device,
	u32 offsetwords, u32 value, u32 pipe, u32 slice_id, u32 use_slice_id)
{
	gen8_host_aperture_set(ADRENO_DEVICE(device), pipe, slice_id, use_slice_id);

	kgsl_regmap_write(&device->regmap, value, offsetwords);
}

#define GEN8_CP_PROTECT_DEFAULT (FIELD_PREP(GENMASK(31, 16), 0xffff) | BIT(0) | BIT(1) | BIT(3))
static void gen8_protect_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	const struct gen8_protected_regs *regs = gen8_core->protected_regs;
	u32 count = 0;
	int i;

	/*
	 * Enable access protection to privileged registers, fault on an access
	 * protect violation and select the last span to protect from the start
	 * address all the way to the end of the register address space
	 */
	gen8_regwrite_aperture(device, GEN8_CP_PROTECT_CNTL_PIPE,
			       GEN8_CP_PROTECT_DEFAULT, PIPE_BR, 0, 0);
	gen8_regwrite_aperture(device, GEN8_CP_PROTECT_CNTL_PIPE,
			       GEN8_CP_PROTECT_DEFAULT, PIPE_BV, 0, 0);
	if (adreno_dev->lpac_enabled)
		gen8_regwrite_aperture(device, GEN8_CP_PROTECT_CNTL_PIPE,
			       GEN8_CP_PROTECT_DEFAULT, PIPE_LPAC, 0, 0);

	/* Clear aperture register */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	/* Program each register defined by the core definition */
	for (i = 0; regs[i].reg; i++) {
		/*
		 * This is the offset of the end register as counted from the
		 * start, i.e. # of registers in the range - 1
		 */
		count = regs[i].end - regs[i].start;

		kgsl_regwrite(device, regs[i].reg,
				FIELD_PREP(GENMASK(17, 0), regs[i].start) |
				FIELD_PREP(GENMASK(30, 18), count) |
				FIELD_PREP(BIT(31), regs[i].noaccess));
	}

	/*
	 * Last span setting is only being applied to the last pipe specific
	 * register. Hence duplicate the last span from protect reg into the
	 * BR, BV and LPAC protect reg pipe 15.
	 */
	i--;
	gen8_regwrite_aperture(device, GEN8_CP_PROTECT_REG_PIPE + 15,
			       FIELD_PREP(GENMASK(17, 0), regs[i].start) |
			       FIELD_PREP(GENMASK(30, 18), count) |
			       FIELD_PREP(BIT(31), regs[i].noaccess),
			       PIPE_BR, 0, 0);

	gen8_regwrite_aperture(device, GEN8_CP_PROTECT_REG_PIPE + 15,
			       FIELD_PREP(GENMASK(17, 0), regs[i].start) |
			       FIELD_PREP(GENMASK(30, 18), count) |
			       FIELD_PREP(BIT(31), regs[i].noaccess),
			       PIPE_BV, 0, 0);

	if (adreno_dev->lpac_enabled)
		gen8_regwrite_aperture(device, GEN8_CP_PROTECT_REG_PIPE + 15,
				       FIELD_PREP(GENMASK(17, 0), regs[i].start) |
				       FIELD_PREP(GENMASK(30, 18), count) |
				       FIELD_PREP(BIT(31), regs[i].noaccess),
				       PIPE_LPAC, 0, 0);

	/* Clear aperture register */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);
}

static void gen8_nonctxt_regconfig(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	const struct gen8_nonctxt_regs *regs = gen8_core->nonctxt_regs;
	struct gen8_device *gen8_dev = container_of(adreno_dev,
					struct gen8_device, adreno_dev);
	u32 i, pipe_id;
	unsigned long pipe;

	/* Program non context registers for all pipes */
	for (pipe_id = PIPE_NONE; pipe_id <= PIPE_AQE1; pipe_id++) {

		if ((pipe_id == PIPE_LPAC) && !ADRENO_FEATURE(adreno_dev, ADRENO_LPAC))
			continue;
		else if (((pipe_id == PIPE_AQE0) || (pipe_id == PIPE_AQE1)) &&
			 !ADRENO_FEATURE(adreno_dev, ADRENO_AQE))
			continue;

		for (i = 0; regs[i].offset; i++) {
			pipe = (unsigned long)regs[i].pipelines;
			if (test_bit(pipe_id, &pipe))
				gen8_regwrite_aperture(device, regs[i].offset,
					regs[i].val, pipe_id, 0, 0);
		}
	}

	/* Program non context registers overrides for all pipes */
	if (gen8_dev->nc_overrides_enabled) {
		struct gen8_nonctxt_overrides *nc_overrides = gen8_dev->nc_overrides;

		mutex_lock(&gen8_dev->nc_mutex);
		for (pipe_id = PIPE_NONE; pipe_id <= PIPE_AQE1; pipe_id++) {

			if ((pipe_id == PIPE_LPAC) && !ADRENO_FEATURE(adreno_dev, ADRENO_LPAC))
				continue;
			else if (((pipe_id == PIPE_AQE0) || (pipe_id == PIPE_AQE1)) &&
				 !ADRENO_FEATURE(adreno_dev, ADRENO_AQE))
				continue;

			for (i = 0; nc_overrides[i].offset; i++) {
				if (!nc_overrides[i].set)
					continue;

				pipe = (unsigned long)nc_overrides[i].pipelines;
				if (test_bit(pipe_id, &pipe))
					gen8_regwrite_aperture(device, nc_overrides[i].offset,
							nc_overrides[i].val, pipe_id, 0, 0);
			}
		}
		mutex_unlock(&gen8_dev->nc_mutex);
	}

	/* Clear aperture register */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);
}

#define RBBM_CLOCK_CNTL_ON 0x8aa8aa82

static void gen8_hwcg_set(struct adreno_device *adreno_dev, bool on)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	u32 value;
	int i;

	/*
	 * Increase clock keep-on hysteresis from 5 cycles to 8 cycles
	 * for adreno_is_gen8_0_x_family.
	 */
	if ((adreno_is_gen8_0_x_family(adreno_dev)) && on)
		kgsl_regwrite(device, GEN8_RBBM_CGC_0_PC, 0x00000702);

	if (!adreno_dev->hwcg_enabled)
		on = false;

	for (i = 0; i < gen8_core->ao_hwcg_count; i++)
		gmu_core_regwrite(device, gen8_core->ao_hwcg[i].offset,
			on ? gen8_core->ao_hwcg[i].val : 0);

	kgsl_regwrite(device, GEN8_RBBM_CLOCK_CNTL_GLOBAL, 1);
	kgsl_regwrite(device, GEN8_RBBM_CGC_GLOBAL_LOAD_CMD, on ? 1 : 0);

	if (on) {
		u32 retry = 3;

		kgsl_regwrite(device, GEN8_RBBM_CGC_P2S_TRIG_CMD, 1);
		/* Poll for the TXDONE:BIT(0) status */
		do {
			/* Wait for small amount of time for TXDONE status*/
			udelay(1);
			kgsl_regread(device, GEN8_RBBM_CGC_P2S_STATUS, &value);
		} while (!(value & BIT(0)) && --retry);

		if (!(value & BIT(0))) {
			dev_err(device->dev, "RBBM_CGC_P2S_STATUS:TXDONE Poll failed\n");
			kgsl_device_snapshot(device, NULL, NULL, false);
			return;
		}
		kgsl_regwrite(device, GEN8_RBBM_CLOCK_CNTL_GLOBAL, 0);
	}
}

static void gen8_patch_pwrup_reglist(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen8_device *gen8_dev = container_of(adreno_dev,
					struct gen8_device, adreno_dev);
	struct adreno_reglist_list reglist[3];
	void *ptr = adreno_dev->pwrup_reglist->hostptr;
	struct cpu_gpu_lock *lock = ptr;
	u32 items = 0, i, j, pipe_id;
	u32 *dest = ptr + sizeof(*lock);
	struct gen8_nonctxt_overrides *nc_overrides = gen8_dev->nc_overrides;

	/* Static IFPC restore only registers */
	if (adreno_is_gen8_3_0(adreno_dev)) {
		reglist[items].regs = gen8_3_0_ifpc_pwrup_reglist;
		reglist[items].count = ARRAY_SIZE(gen8_3_0_ifpc_pwrup_reglist);
	} else {
		reglist[items].regs = gen8_ifpc_pwrup_reglist;
		reglist[items].count = ARRAY_SIZE(gen8_ifpc_pwrup_reglist);
	}
	lock->ifpc_list_len = reglist[items].count;
	items++;

	/* Static IFPC + preemption registers */
	if (adreno_is_gen8_3_0(adreno_dev)) {
		reglist[items].regs = gen8_3_0_pwrup_reglist;
		reglist[items].count = ARRAY_SIZE(gen8_3_0_pwrup_reglist);
	} else {
		reglist[items].regs = gen8_pwrup_reglist;
		reglist[items].count = ARRAY_SIZE(gen8_pwrup_reglist);
	}
	lock->preemption_list_len = reglist[items].count;
	items++;

	/*
	 * For each entry in each of the lists, write the offset and the current
	 * register value into the GPU buffer
	 */
	for (i = 0; i < items; i++) {
		const u32 *r = reglist[i].regs;

		for (j = 0; j < reglist[i].count; j++) {
			*dest++ = r[j];
			kgsl_regread(device, r[j], dest++);
		}

		if ((r == gen8_ifpc_pwrup_reglist) || (r == gen8_3_0_ifpc_pwrup_reglist)) {
			u32 cs_len = adreno_coresight_patch_pwrup_reglist(adreno_dev, dest);

			lock->ifpc_list_len += cs_len;
			dest += (cs_len * 2);
		}

		mutex_lock(&gen8_dev->nc_mutex);
		for (j = 0; j < nc_overrides[j].offset; j++) {
			unsigned long pipe = (unsigned long)nc_overrides[j].pipelines;

			if (!(test_bit(PIPE_NONE, &pipe) && nc_overrides[j].set &&
				nc_overrides[j].list_type))
				continue;

			if ((reglist[i].regs == gen8_ifpc_pwrup_reglist ||
				reglist[i].regs == gen8_3_0_ifpc_pwrup_reglist) &&
				(nc_overrides[j].list_type == 1)) {
				*dest++ = nc_overrides[j].offset;
				kgsl_regread(device, nc_overrides[j].offset, dest++);
				lock->ifpc_list_len++;
			} else if ((reglist[i].regs == gen8_pwrup_reglist ||
				reglist[i].regs == gen8_3_0_pwrup_reglist) &&
				(nc_overrides[j].list_type == 2)) {
				*dest++ = nc_overrides[j].offset;
				kgsl_regread(device, nc_overrides[j].offset, dest++);
				lock->preemption_list_len++;
			}
		}
		mutex_unlock(&gen8_dev->nc_mutex);

	}

	/*
	 * The overall register list is composed of
	 * 1. Static IFPC-only registers
	 * 2. Static IFPC + preemption registers
	 * 3. Dynamic IFPC + preemption registers (ex: perfcounter selects)
	 *
	 * The first two lists are static. Size of these lists are stored as
	 * number of pairs in ifpc_list_len and preemption_list_len
	 * respectively. With concurrent binning, Some of the perfcounter
	 * registers being virtualized, CP needs to know the pipe id to program
	 * the aperture inorder to restore the same. Thus, third list is a
	 * dynamic list with triplets as
	 * (<aperture, shifted 12 bits> <address> <data>), and the length is
	 * stored as number for triplets in dynamic_list_len.
	 *
	 * Starting with Gen8, some of the registers that are initialized statically
	 * by the kernel are pipe-specific. Because only the dynamic list is able to
	 * support specifying a pipe ID, these registers are bundled along with any
	 * dynamic entries such as perf counter selects into a single dynamic list.
	 */

	gen8_dev->ext_pwrup_list_len = 0;

	/*
	 * Write external pipe specific regs (<aperture> <address> <value> - triplets)
	 * offset and the current value into GPU buffer
	 */
	for (pipe_id = PIPE_BR; pipe_id <= PIPE_DDE_BV; pipe_id++) {
		for (i = 0; i < ARRAY_SIZE(gen8_0_0_pwrup_extlist); i++) {
			unsigned long pipe = (unsigned long)gen8_0_0_pwrup_extlist[i].pipelines;

			if (!test_bit(pipe_id, &pipe))
				continue;
			if ((pipe_id == PIPE_LPAC) && !ADRENO_FEATURE(adreno_dev, ADRENO_LPAC))
				continue;
			if (((pipe_id == PIPE_AQE0) || (pipe_id == PIPE_AQE1)) &&
				!ADRENO_FEATURE(adreno_dev, ADRENO_AQE))
				continue;

			*dest++ = FIELD_PREP(GENMASK(15, 12), pipe_id);
			*dest++ = gen8_0_0_pwrup_extlist[i].offset;
			gen8_regread_aperture(device, gen8_0_0_pwrup_extlist[i].offset,
					dest++, pipe_id, 0, 0);
			gen8_dev->ext_pwrup_list_len++;
		}
	}

	/*
	 * Write noncontext override pipe specific regs (<aperture> <address> <value> - triplets)
	 * offset and the current value into GPU buffer
	 */
	mutex_lock(&gen8_dev->nc_mutex);
	for (pipe_id = PIPE_BR; pipe_id <= PIPE_BV; pipe_id++) {
		for (i = 0; i < nc_overrides[i].offset; i++) {
			unsigned long pipe = (unsigned long)nc_overrides[i].pipelines;

			if (!(test_bit(pipe_id, &pipe) && nc_overrides[i].set &&
				nc_overrides[i].list_type))
				continue;

			*dest++ = FIELD_PREP(GENMASK(15, 12), pipe_id);
			*dest++ = nc_overrides[i].offset;
			gen8_regread_aperture(device, nc_overrides[i].offset,
					dest++, pipe_id, 0, 0);
			gen8_dev->ext_pwrup_list_len++;
		}
	}
	mutex_unlock(&gen8_dev->nc_mutex);

	/* Clear aperture register */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	lock->dynamic_list_len = gen8_dev->ext_pwrup_list_len;
}

/* _llc_configure_gpu_scid() - Program the sub-cache ID for all GPU blocks */
static void _llc_configure_gpu_scid(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 gpu_scid;

	if (IS_ERR_OR_NULL(adreno_dev->gpu_llc_slice) ||
		!adreno_dev->gpu_llc_slice_enable)
		return;

	if (llcc_slice_activate(adreno_dev->gpu_llc_slice))
		return;

	gpu_scid = llcc_get_slice_id(adreno_dev->gpu_llc_slice);

	/* 5 blocks at 6 bits per block */
	kgsl_regwrite(device, GEN8_GBIF_SCACHE_CNTL1,
			FIELD_PREP(GENMASK(29, 24), gpu_scid) |
			FIELD_PREP(GENMASK(23, 18), gpu_scid) |
			FIELD_PREP(GENMASK(17, 12), gpu_scid) |
			FIELD_PREP(GENMASK(11, 6), gpu_scid) |
			FIELD_PREP(GENMASK(5, 0), gpu_scid));

	kgsl_regwrite(device, GEN8_GBIF_SCACHE_CNTL0,
			FIELD_PREP(GENMASK(15, 10), gpu_scid) |
			FIELD_PREP(GENMASK(21, 16), gpu_scid) |
			FIELD_PREP(GENMASK(27, 22), gpu_scid) | BIT(8));
}

static void _llc_gpuhtw_slice_activate(struct adreno_device *adreno_dev)
{
	if (IS_ERR_OR_NULL(adreno_dev->gpuhtw_llc_slice) ||
		!adreno_dev->gpuhtw_llc_slice_enable)
		return;

	llcc_slice_activate(adreno_dev->gpuhtw_llc_slice);
}

static void _set_secvid(struct kgsl_device *device)
{
	kgsl_regwrite(device, GEN8_RBBM_SECVID_TSB_CNTL, 0x0);
	kgsl_regwrite(device, GEN8_RBBM_SECVID_TSB_TRUSTED_BASE_LO,
		lower_32_bits(KGSL_IOMMU_SECURE_BASE32));
	kgsl_regwrite(device, GEN8_RBBM_SECVID_TSB_TRUSTED_BASE_HI,
		upper_32_bits(KGSL_IOMMU_SECURE_BASE32));
	kgsl_regwrite(device, GEN8_RBBM_SECVID_TSB_TRUSTED_SIZE,
		FIELD_PREP(GENMASK(31, 12),
		(KGSL_IOMMU_SECURE_SIZE(&device->mmu) / SZ_4K)));
}

/* Set UCHE_TRAP_BASE to a page below the top of the memory space */
#define GEN8_UCHE_TRAP_BASE 0x1FFFFFFFFF000ULL

static u64 gen8_get_uche_trap_base(void)
{
	return GEN8_UCHE_TRAP_BASE;
}

/*
 * All Gen8 targets support marking certain transactions as always privileged
 * which allows us to mark more memory as privileged without having to
 * explicitly set the APRIV bit. Choose the following transactions to be
 * privileged by default:
 * CDWRITE     [6:6] - Crashdumper writes
 * CDREAD      [5:5] - Crashdumper reads
 * RBRPWB      [3:3] - RPTR shadow writes
 * RBPRIVLEVEL [2:2] - Memory accesses from PM4 packets in the ringbuffer
 * RBFETCH     [1:1] - Ringbuffer reads
 * ICACHE      [0:0] - Instruction cache fetches
 */

#define GEN8_APRIV_DEFAULT (BIT(3) | BIT(2) | BIT(1) | BIT(0))
/* Add crashdumper permissions for the BR APRIV */
#define GEN8_BR_APRIV_DEFAULT (GEN8_APRIV_DEFAULT | BIT(6) | BIT(5))

static const struct kgsl_regmap_list gen8_0_0_bicubic_regs[] = {
	/*GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_0 default and recomended values are same */
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_1,  0x3fe05ff4 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_2,  0x3fa0ebee },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_3,  0x3f5193ed },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_4,  0x3f0243f0 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_5,  0x00000000 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_6,  0x3fd093e8 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_7,  0x3f4133dc },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_8,  0x3ea1dfdb },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_9,  0x3e0283e0 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_10, 0x0000ac2b },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_11, 0x0000f01d },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_12, 0x00114412 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_13, 0x0021980a },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_14, 0x0051ec05 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_15, 0x0000380e },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_16, 0x3ff09001 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_17, 0x3fc10bfa },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_18, 0x3f9193f7 },
	{ GEN8_TPL1_BICUBIC_WEIGHTS_TABLE_19, 0x3f7227f7 },
};

void gen8_enable_ahb_timeout_detection(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 val;

	if (!adreno_dev->ahb_timeout_val)
		return;

	val = (ADRENO_AHB_CNTL_DEFAULT | FIELD_PREP(GENMASK(4, 0),
			adreno_dev->ahb_timeout_val));
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_CX_AHB_AON_CNTL, val);
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_CX_AHB_GMU_CNTL, val);
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_CX_AHB_CP_CNTL, val);
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_CX_AHB_VBIF_SMMU_CNTL, val);
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_CX_AHB_HOST_CNTL, val);
}

#define MIN_HBB 13
int gen8_start(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	u32 mal, pipe_id, mode = 0, mode2 = 0, rgb565_predicator = 0, amsbc = 0;
	struct gen8_device *gen8_dev = container_of(adreno_dev,
					struct gen8_device, adreno_dev);
	/*
	 * HBB values 13 to 16 can represented LSB of HBB from 0 to 3.
	 * Any HBB value beyond 16 needs programming MSB of HBB.
	 * By default highest bank bit is 14, Hence set default HBB LSB
	 * to "1" and MSB to "0".
	 */
	u32 hbb_lo = 1, hbb_hi = 0, hbb = 1;
	struct cpu_gpu_lock *pwrup_lock = adreno_dev->pwrup_reglist->hostptr;
	u64 uche_trap_base = gen8_get_uche_trap_base();
	u32 rgba8888_lossless = 0, fp16compoptdis = 0;
	int is_current_rt = rt_task(current);
	int nice = task_nice(current);

	/* Reset aperture fields to go through first aperture write check */
	gen8_dev->aperture = UINT_MAX;

	/*
	 * Elevating the thread’s priority to FIFO to ensure sequential register access
	 * on the same CPU, avoiding context switches to a different CPU or thread.
	 */
	if (!is_current_rt)
		sched_set_fifo(current);

	device->regmap.use_relaxed = false;

	/* Make all blocks contribute to the GPU BUSY perf counter */
	kgsl_regwrite(device, GEN8_RBBM_PERFCTR_GPU_BUSY_MASKED, 0xffffffff);

	kgsl_regwrite(device, GEN8_UCHE_CCHE_GC_GMEM_RANGE_MIN_LO,
			lower_32_bits(adreno_dev->uche_gmem_base));
	kgsl_regwrite(device, GEN8_UCHE_CCHE_GC_GMEM_RANGE_MIN_HI,
			upper_32_bits(adreno_dev->uche_gmem_base));
	kgsl_regwrite(device, GEN8_SP_HLSQ_GC_GMEM_RANGE_MIN_LO,
			lower_32_bits(adreno_dev->uche_gmem_base));
	kgsl_regwrite(device, GEN8_SP_HLSQ_GC_GMEM_RANGE_MIN_HI,
			upper_32_bits(adreno_dev->uche_gmem_base));

	if (adreno_dev->lpac_enabled) {
		gen8_regwrite_aperture(device, GEN8_RB_LPAC_GMEM_PROTECT,
			0x0c000000, PIPE_BR, 0, 0);

		/* Clear aperture register  */
		gen8_host_aperture_set(adreno_dev, 0, 0, 0);

		kgsl_regwrite(device, GEN8_UCHE_CCHE_LPAC_GMEM_RANGE_MIN_LO,
				lower_32_bits(adreno_dev->uche_gmem_base));
		kgsl_regwrite(device, GEN8_UCHE_CCHE_LPAC_GMEM_RANGE_MIN_HI,
				upper_32_bits(adreno_dev->uche_gmem_base));
		kgsl_regwrite(device, GEN8_SP_HLSQ_LPAC_GMEM_RANGE_MIN_LO,
				lower_32_bits(adreno_dev->uche_gmem_base));
		kgsl_regwrite(device, GEN8_SP_HLSQ_LPAC_GMEM_RANGE_MIN_HI,
				upper_32_bits(adreno_dev->uche_gmem_base));
	}

	/*
	 * Set UCHE_WRITE_THRU_BASE to the UCHE_TRAP_BASE effectively
	 * disabling L2 bypass
	 */
	kgsl_regwrite(device, GEN8_UCHE_TRAP_BASE_LO, lower_32_bits(uche_trap_base));
	kgsl_regwrite(device, GEN8_UCHE_TRAP_BASE_HI, upper_32_bits(uche_trap_base));
	kgsl_regwrite(device, GEN8_UCHE_WRITE_THRU_BASE_LO, lower_32_bits(uche_trap_base));
	kgsl_regwrite(device, GEN8_UCHE_WRITE_THRU_BASE_HI, upper_32_bits(uche_trap_base));

	/*
	 * CP takes care of the restore during IFPC exit. We need to restore at slumber
	 * boundary as well
	 */
	if (pwrup_lock->dynamic_list_len - gen8_dev->ext_pwrup_list_len > 0) {
		kgsl_regwrite(device, GEN8_RBBM_PERFCTR_CNTL, 0x1);
		kgsl_regwrite(device, GEN8_RBBM_SLICE_PERFCTR_CNTL, 0x1);
	}

	/* Turn on the IFPC counter (countable 4 on XOCLK4) */
	kgsl_regwrite(device, GEN8_GMUCX_POWER_COUNTER_SELECT_XOCLK_1,
			FIELD_PREP(GENMASK(7, 0), 0x4));

	/* Turn on counter to count total time spent in BCL throttle */
	if (adreno_dev->bcl_enabled)
		kgsl_regrmw(device, GEN8_GMUCX_POWER_COUNTER_SELECT_XOCLK_1, GENMASK(15, 8),
				FIELD_PREP(GENMASK(15, 8), 0x26));

	if (of_property_read_u32(device->pdev->dev.of_node, "qcom,min-access-length", &mal))
		mal = 32;

	of_property_read_u32(device->pdev->dev.of_node, "qcom,ubwc-mode", &mode);

	switch (mode) {
	case KGSL_UBWC_5_0:
		amsbc = 1;
		rgb565_predicator = 1;
		mode2 = 4;
		break;
	case KGSL_UBWC_4_0:
		amsbc = 1;
		rgb565_predicator = 1;
		fp16compoptdis = 1;
		rgba8888_lossless = 1;
		mode2 = 2;
		break;
	case KGSL_UBWC_3_0:
		amsbc = 1;
		mode2 = 1;
		break;
	default:
		break;
	}

	if (!WARN_ON(!adreno_dev->highest_bank_bit)) {
		hbb = adreno_dev->highest_bank_bit - MIN_HBB;
		hbb_lo = hbb & 3;
		hbb_hi = (hbb >> 2) & 1;
	}

	mal = (mal == 64) ? 1 : 0;

	gen8_regwrite_aperture(device, GEN8_GRAS_NC_MODE_CNTL,
			       FIELD_PREP(GENMASK(8, 5), hbb), PIPE_BV, 0, 0);
	gen8_regwrite_aperture(device, GEN8_GRAS_NC_MODE_CNTL,
			       FIELD_PREP(GENMASK(8, 5), hbb), PIPE_BR, 0, 0);
	gen8_regwrite_aperture(device, GEN8_RB_CCU_NC_MODE_CNTL,
			       FIELD_PREP(GENMASK(3, 3), hbb_hi) |
			       FIELD_PREP(GENMASK(2, 1), hbb_lo),
			       PIPE_BR, 0, 0);
	gen8_regwrite_aperture(device, GEN8_RB_CMP_NC_MODE_CNTL,
			       FIELD_PREP(GENMASK(17, 15), mode2) |
			       FIELD_PREP(GENMASK(4, 4), rgba8888_lossless) |
			       FIELD_PREP(GENMASK(3, 3), fp16compoptdis) |
			       FIELD_PREP(GENMASK(2, 2), rgb565_predicator) |
			       FIELD_PREP(GENMASK(1, 1), amsbc) |
			       FIELD_PREP(GENMASK(0, 0), mal),
			       PIPE_BR, 0, 0);

	/* Clear aperture register  */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	kgsl_regwrite(device, GEN8_SP_NC_MODE_CNTL,
		      FIELD_PREP(GENMASK(11, 10), hbb_hi) |
		      FIELD_PREP(GENMASK(5, 4), 2) |
		      FIELD_PREP(GENMASK(3, 3), mal) |
		      FIELD_PREP(GENMASK(2, 1), hbb_lo));

	kgsl_regwrite(device, GEN8_TPL1_NC_MODE_CNTL,
		      FIELD_PREP(GENMASK(4, 4), hbb_hi) |
		      FIELD_PREP(GENMASK(3, 3), mal) |
		      FIELD_PREP(GENMASK(2, 1), hbb_lo));

	/* Configure TP bicubic registers */
	kgsl_regmap_multi_write(&device->regmap, gen8_0_0_bicubic_regs,
				ARRAY_SIZE(gen8_0_0_bicubic_regs));

	kgsl_regwrite(device, GEN8_UCHE_CLIENT_PF, BIT(7) |
			FIELD_PREP(GENMASK(6, 0), adreno_dev->uche_client_pf));

	/* Program noncontext registers */
	gen8_nonctxt_regconfig(adreno_dev);

	/* Enable hardware hang detection */
	kgsl_regwrite(device, GEN8_RBBM_INTERFACE_HANG_INT_CNTL, BIT(30) |
			FIELD_PREP(GENMASK(27, 0), gen8_core->hang_detect_cycles));
	kgsl_regwrite(device, GEN8_RBBM_SLICE_INTERFACE_HANG_INT_CNTL, BIT(30));

	/* Enable the GMEM save/restore feature for preemption */
	if (adreno_is_preemption_enabled(adreno_dev)) {
		gen8_regwrite_aperture(device,
				GEN8_RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE,
				0x1, PIPE_BR, 0, 0);
		/* Clear aperture register  */
		gen8_host_aperture_set(adreno_dev, 0, 0, 0);
	}

	/* Enable GMU power counter 0 to count GPU busy */
	kgsl_regwrite(device, GEN8_GMUAO_GPU_CX_BUSY_MASK, 0xff000000);
	kgsl_regrmw(device, GEN8_GMUCX_POWER_COUNTER_SELECT_XOCLK_0, 0xFF, 0x20);
	kgsl_regwrite(device, GEN8_GMUCX_POWER_COUNTER_ENABLE, 0x1);

	gen8_protect_init(adreno_dev);

	/* Configure LLCC */
	_llc_configure_gpu_scid(adreno_dev);
	_llc_gpuhtw_slice_activate(adreno_dev);

	for (pipe_id = PIPE_BR; pipe_id <= PIPE_DDE_BV; pipe_id++) {
		if ((pipe_id == PIPE_LPAC) && !ADRENO_FEATURE(adreno_dev, ADRENO_LPAC))
			continue;
		if (((pipe_id == PIPE_AQE0) || (pipe_id == PIPE_AQE1)) &&
			!ADRENO_FEATURE(adreno_dev, ADRENO_AQE))
			continue;

		gen8_regwrite_aperture(device, GEN8_CP_APRIV_CNTL_PIPE,
			(pipe_id == PIPE_BR ? GEN8_BR_APRIV_DEFAULT : GEN8_APRIV_DEFAULT),
			pipe_id, 0, 0);
		gen8_regwrite_aperture(device, GEN8_CP_INTERRUPT_STATUS_MASK_PIPE,
			CP_SW_FAULT_STATUS_MASK_PIPE, pipe_id, 0, 0);
		gen8_regwrite_aperture(device, GEN8_CP_HW_FAULT_STATUS_MASK_PIPE,
			CP_HW_FAULT_STATUS_MASK_PIPE, pipe_id, 0, 0);
	}

	/* Clear aperture register  */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	/* Program CP interrupt status mask to enable HW and SW error interrupts */
	kgsl_regwrite(device, GEN8_CP_INTERRUPT_STATUS_MASK_GLOBAL,
			CP_INTERRUPT_STATUS_MASK_GLOBAL);

	_set_secvid(device);

	/*
	 * Enable hardware clock gating here to prevent any register access
	 * issue due to internal clock gating.
	 */
	gen8_hwcg_set(adreno_dev, true);

	/*
	 * All registers must be written before this point so that we don't
	 * miss any register programming when we patch the power up register
	 * list.
	 */
	if (!adreno_dev->patch_reglist &&
		(adreno_dev->pwrup_reglist->gpuaddr != 0)) {
		gen8_patch_pwrup_reglist(adreno_dev);
		adreno_dev->patch_reglist = true;
	}

	/* Ensure very last register write is finished before we return from this function */
	mb();
	device->regmap.use_relaxed = true;

	if (!is_current_rt)
		sched_set_normal(current, nice);

	return 0;
}

/* Offsets into the MX/CX mapped register regions */
#define GEN8_RDPM_MX_OFFSET 0xf00
#define GEN8_RDPM_CX_OFFSET 0xf14

void gen8_rdpm_mx_freq_update(struct gen8_gmu_device *gmu, u32 freq)
{
	if (gmu->rdpm_mx_virt) {
		writel_relaxed(freq/1000, (gmu->rdpm_mx_virt + GEN8_RDPM_MX_OFFSET));

		/*
		 * ensure previous writes post before this one,
		 * i.e. act like normal writel()
		 */
		wmb();
	}
}

void gen8_rdpm_cx_freq_update(struct gen8_gmu_device *gmu, u32 freq)
{
	if (gmu->rdpm_cx_virt) {
		writel_relaxed(freq/1000, (gmu->rdpm_cx_virt + GEN8_RDPM_CX_OFFSET));

		/*
		 * ensure previous writes post before this one,
		 * i.e. act like normal writel()
		 */
		wmb();
	}
}

int gen8_scm_gpu_init_cx_regs(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 gpu_req = GPU_ALWAYS_EN_REQ;
	int ret;

	if (ADRENO_FEATURE(adreno_dev, ADRENO_BCL))
		gpu_req |= GPU_BCL_EN_REQ;

	if (ADRENO_FEATURE(adreno_dev, ADRENO_CLX))
		gpu_req |= GPU_CLX_EN_REQ;

	gpu_req |= GPU_TSENSE_EN_REQ;

	ret = kgsl_scm_gpu_init_regs(&device->pdev->dev, gpu_req);

	/*
	 * For targets that support this scm call to program BCL id , enable BCL.
	 * For other targets, BCL is enabled after first GMU boot.
	 */
	if (!ret && ADRENO_FEATURE(adreno_dev, ADRENO_BCL))
		adreno_dev->bcl_enabled = true;

	/* If programming TZ CLX was successful, then program KMD owned CLX regs */
	if (!ret && ADRENO_FEATURE(adreno_dev, ADRENO_CLX))
		adreno_dev->clx_enabled = true;

	/*
	 * If scm call returned EOPNOTSUPP, either we are on a kernel version
	 * lesser than 6.1 where scm call is not supported or we are sending an
	 * empty request. Ignore the error in such cases.
	 */
	return (ret == -EOPNOTSUPP) ? 0 : ret;
}

void gen8_spin_idle_debug(struct adreno_device *adreno_dev,
				const char *str)
{
	struct kgsl_device *device = &adreno_dev->dev;
	u32 rptr, wptr, status, intstatus, global_status;

	dev_err(device->dev, str);

	kgsl_regread(device, GEN8_CP_RB_RPTR_BR, &rptr);
	kgsl_regread(device, GEN8_CP_RB_WPTR_GC, &wptr);

	kgsl_regread(device, GEN8_RBBM_STATUS, &status);
	kgsl_regread(device, GEN8_RBBM_INT_0_STATUS, &intstatus);
	kgsl_regread(device, GEN8_CP_INTERRUPT_STATUS_GLOBAL, &global_status);

	dev_err(device->dev,
		"rb=%d pos=%X/%X rbbm_status=%8.8X int_0_status=%8.8X global_status=%8.8X\n",
		adreno_dev->cur_rb ? adreno_dev->cur_rb->id : -1, rptr, wptr,
		status, intstatus, global_status);

	kgsl_device_snapshot(device, NULL, NULL, false);
}

/*
 * gen8_send_cp_init() - Initialize ringbuffer
 * @adreno_dev: Pointer to adreno device
 * @rb: Pointer to the ringbuffer of device
 *
 * Submit commands for ME initialization,
 */
static int gen8_send_cp_init(struct adreno_device *adreno_dev,
			 struct adreno_ringbuffer *rb)
{
	u32 *cmds;
	int ret;

	cmds = adreno_ringbuffer_allocspace(rb, GEN8_CP_INIT_DWORDS);
	if (IS_ERR(cmds))
		return PTR_ERR(cmds);

	gen8_cp_init_cmds(adreno_dev, cmds);

	ret = gen8_ringbuffer_submit(rb, NULL);
	if (ret)
		return ret;

	ret = adreno_spin_idle(adreno_dev, 2000);
	if (ret) {
		gen8_spin_idle_debug(adreno_dev,
				     "CP initialization failed to idle\n");
		rb->wptr = 0;
		rb->_wptr = 0;
	}

	return ret;
}

static int gen8_post_start(struct adreno_device *adreno_dev)
{
	int ret;
	u32 *cmds;
	struct adreno_ringbuffer *rb = adreno_dev->cur_rb;
	struct adreno_preemption *preempt = &adreno_dev->preempt;
	u64 kmd_postamble_addr;

	if (!adreno_is_preemption_enabled(adreno_dev))
		return 0;

	kmd_postamble_addr = SCRATCH_POSTAMBLE_ADDR(KGSL_DEVICE(adreno_dev));
	gen8_preemption_prepare_postamble(adreno_dev);

	cmds = adreno_ringbuffer_allocspace(rb,
			(preempt->postamble_bootup_len ? 16 : 12));
	if (IS_ERR(cmds))
		return PTR_ERR(cmds);

	*cmds++ = cp_type7_packet(CP_SET_PSEUDO_REGISTER, 6);
	*cmds++ = SET_PSEUDO_PRIV_NON_SECURE_SAVE_ADDR;
	*cmds++ = lower_32_bits(rb->preemption_desc->gpuaddr);
	*cmds++ = upper_32_bits(rb->preemption_desc->gpuaddr);

	*cmds++ = SET_PSEUDO_PRIV_SECURE_SAVE_ADDR;
	*cmds++ = lower_32_bits(rb->secure_preemption_desc->gpuaddr);
	*cmds++ = upper_32_bits(rb->secure_preemption_desc->gpuaddr);

	if (preempt->postamble_bootup_len) {
		*cmds++ = cp_type7_packet(CP_SET_AMBLE, 3);
		*cmds++ = lower_32_bits(kmd_postamble_addr);
		*cmds++ = upper_32_bits(kmd_postamble_addr);
		*cmds++ = FIELD_PREP(GENMASK(22, 20), CP_KMD_AMBLE_TYPE)
			| (FIELD_PREP(GENMASK(19, 0),
				adreno_dev->preempt.postamble_bootup_len));
	}

	*cmds++ = cp_type7_packet(CP_CONTEXT_SWITCH_YIELD, 4);
	*cmds++ = 0;
	*cmds++ = 0;
	*cmds++ = 0;
	/* generate interrupt on preemption completion */
	*cmds++ = 0;

	ret = gen8_ringbuffer_submit(rb, NULL);
	if (!ret) {
		ret = adreno_spin_idle(adreno_dev, 2000);
		if (ret)
			gen8_spin_idle_debug(adreno_dev,
				"hw preemption initialization failed to idle\n");
	}

	return ret;
}

int gen8_rb_start(struct adreno_device *adreno_dev)
{
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	struct adreno_firmware *fw = ADRENO_FW(adreno_dev, ADRENO_FW_SQE);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_ringbuffer *rb;
	u64 addr;
	int ret, i;
	u32 *cmds;

	/* Clear all the ringbuffers */
	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		memset(rb->buffer_desc->hostptr, 0xaa, KGSL_RB_SIZE);
		kgsl_sharedmem_writel(device->scratch,
			SCRATCH_RB_OFFSET(rb->id, rptr), 0);
		kgsl_sharedmem_writel(device->scratch,
			SCRATCH_RB_OFFSET(rb->id, bv_rptr), 0);

		rb->wptr = 0;
		rb->_wptr = 0;
		rb->wptr_preempt_end = UINT_MAX;
	}

	gen8_preemption_start(adreno_dev);

	/* Set up the current ringbuffer */
	rb = ADRENO_CURRENT_RINGBUFFER(adreno_dev);

	addr = SCRATCH_RB_GPU_ADDR(device, rb->id, rptr);
	kgsl_regwrite(device, GEN8_CP_RB_RPTR_ADDR_LO_BR, lower_32_bits(addr));
	kgsl_regwrite(device, GEN8_CP_RB_RPTR_ADDR_HI_BR, upper_32_bits(addr));

	addr = SCRATCH_RB_GPU_ADDR(device, rb->id, bv_rptr);
	kgsl_regwrite(device, GEN8_CP_RB_RPTR_ADDR_LO_BV, lower_32_bits(addr));
	kgsl_regwrite(device, GEN8_CP_RB_RPTR_ADDR_HI_BV, upper_32_bits(addr));

	kgsl_regwrite(device, GEN8_CP_RB_CNTL_GC, GEN8_CP_RB_CNTL_DEFAULT);

	kgsl_regwrite(device, GEN8_CP_RB_BASE_LO_GC,
		lower_32_bits(rb->buffer_desc->gpuaddr));
	kgsl_regwrite(device, GEN8_CP_RB_BASE_HI_GC,
		upper_32_bits(rb->buffer_desc->gpuaddr));

	/* Program the ucode base for CP */
	kgsl_regwrite(device, GEN8_CP_SQE_INSTR_BASE_LO,
		lower_32_bits(fw->memdesc->gpuaddr));
	kgsl_regwrite(device, GEN8_CP_SQE_INSTR_BASE_HI,
		upper_32_bits(fw->memdesc->gpuaddr));

	/* Clear the SQE_HALT to start the CP engine */
	kgsl_regwrite(device, GEN8_CP_SQE_CNTL, 1);

	ret = gen8_send_cp_init(adreno_dev, rb);
	if (ret)
		return ret;

	ret = adreno_zap_shader_load(adreno_dev, gen8_core->zap_name);
	if (ret)
		return ret;

	/*
	 * Take the GPU out of secure mode. Try the zap shader if it is loaded,
	 * otherwise just try to write directly to the secure control register
	 */
	if (!adreno_dev->zap_loaded)
		kgsl_regwrite(device, GEN8_RBBM_SECVID_TRUST_CNTL, 0);
	else {
		cmds = adreno_ringbuffer_allocspace(rb, 2);
		if (IS_ERR(cmds))
			return PTR_ERR(cmds);

		*cmds++ = cp_type7_packet(CP_SET_SECURE_MODE, 1);
		*cmds++ = 0;

		ret = gen8_ringbuffer_submit(rb, NULL);
		if (!ret) {
			ret = adreno_spin_idle(adreno_dev, 2000);
			if (ret) {
				gen8_spin_idle_debug(adreno_dev,
					"Switch to unsecure failed to idle\n");
				return ret;
			}
		}
	}

	return gen8_post_start(adreno_dev);
}

/*
 * gen8_gpu_keepalive() - GMU reg write to request GPU stays on
 * @adreno_dev: Pointer to the adreno device that has the GMU
 * @state: State to set: true is ON, false is OFF
 */
static void gen8_gpu_keepalive(struct adreno_device *adreno_dev,
		bool state)
{
	gmu_core_regwrite(KGSL_DEVICE(adreno_dev),
			GEN8_GMUCX_PWR_COL_KEEPALIVE, state);
}

bool gen8_hw_isidle(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 reg;

	gmu_core_regread(device, GEN8_GMUAO_GPU_CX_BUSY_STATUS, &reg);

	/* Bit 23 is GPUBUSYIGNAHB */
	return (reg & BIT(23)) ? false : true;
}

int gen8_microcode_read(struct adreno_device *adreno_dev)
{
	struct adreno_firmware *sqe_fw = ADRENO_FW(adreno_dev, ADRENO_FW_SQE);
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);

	return adreno_get_firmware(adreno_dev, gen8_core->sqefw_name, sqe_fw);
}

static void gen8_get_cp_hwfault_status(struct adreno_device *adreno_dev, u32 status)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 hw_status;
	u32 pipe_id = PIPE_NONE;
	const char * const table[] = {
			[CP_HW_RBFAULT] = "RBFAULT",
			[CP_HW_IB1FAULT] = "IB1FAULT",
			[CP_HW_IB2FAULT] = "IB2FAULT",
			[CP_HW_SDSFAULT] = "SDSFAULT",
			[CP_HW_MRBFAULT] = "MRGFAULT",
			[CP_HW_VSDFAULT] = "VSDFAULT",
			[CP_HW_SQEREADBRUSTOVF] = "SQEREADBRUSTOVF",
			[CP_HW_EVENTENGINEOVF] = "EVENTENGINEOVF",
			[CP_HW_UCODEERROR] = "UCODEERROR",
	};

	switch (status) {
	case BIT(GEN8_CP_GLOBAL_INT_HWFAULTBR):
		pipe_id = PIPE_BR;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_HWFAULTBV):
		pipe_id = PIPE_BV;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_HWFAULTLPAC):
		pipe_id = PIPE_LPAC;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_HWFAULTAQE0):
		pipe_id = PIPE_AQE0;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_HWFAULTAQE1):
		pipe_id = PIPE_AQE1;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_HWFAULTDDEBR):
		pipe_id = PIPE_DDE_BR;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_HWFAULTDDEBV):
		pipe_id = PIPE_DDE_BV;
		break;
	}

	gen8_regread_aperture(device, GEN8_CP_HW_FAULT_STATUS_PIPE, &hw_status,
		pipe_id, 0, 0);
	/* Clear aperture register */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	dev_crit_ratelimited(device->dev, "CP HW Fault pipe_id:%u %s\n", pipe_id,
			hw_status < ARRAY_SIZE(table) ? table[hw_status] : "UNKNOWN");
}

static void gen8_get_cp_swfault_status(struct adreno_device *adreno_dev, u32 status)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 sw_status, status1;
	u32 opcode, pipe_id = PIPE_NONE;
	const char * const table[] = {
		[CP_SW_CSFRBWRAP] = "CSFRBWRAP",
		[CP_SW_CSFIB1WRAP] = "CSFIB1WRAP",
		[CP_SW_CSFIB2WRAP] = "CSFIB2WRAP",
		[CP_SW_CSFIB3WRAP] = "CSFIB3WRAP",
		[CP_SW_SDSWRAP] = "SDSWRAP",
		[CP_SW_MRBWRAP] = "MRBWRAP",
		[CP_SW_VSDWRAP] = "VSDWRAP",
		[CP_SW_OPCODEERROR] = "OPCODEERROR",
		[CP_SW_VSDPARITYERROR] = "VSDPARITYERROR",
		[CP_SW_REGISTERPROTECTIONERROR] = "REGISTERPROTECTIONERROR",
		[CP_SW_ILLEGALINSTRUCTION] = "ILLEGALINSTRUCTION",
		[CP_SW_SMMUFAULT] = "SMMUFAULT",
		[CP_SW_VBIFRESPCLIENT] = "VBIFRESPCLIENT",
		[CP_SW_VBIFRESPTYPE] = "VBIFRESPTYPE",
		[CP_SW_VBIFRESPREAD] = "VBIFRESPREAD",
		[CP_SW_VBIFRESP] = "VBIFRESP",
		[CP_SW_RTWROVF] = "RTWROVF",
		[CP_SW_LRZRTWROVF] = "LRZRTWROVF",
		[CP_SW_LRZRTREFCNTOVF] = "LRZRTREFCNTOVF",
		[CP_SW_LRZRTCLRRESMISS] = "LRZRTCLRRESMISS",
	};

	switch (status) {
	case BIT(GEN8_CP_GLOBAL_INT_SWFAULTBR):
		pipe_id = PIPE_BR;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_SWFAULTBV):
		pipe_id = PIPE_BV;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_SWFAULTLPAC):
		pipe_id = PIPE_LPAC;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_SWFAULTAQE0):
		pipe_id = PIPE_AQE0;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_SWFAULTAQE1):
		pipe_id = PIPE_AQE1;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_SWFAULTDDEBR):
		pipe_id = PIPE_DDE_BR;
		break;
	case BIT(GEN8_CP_GLOBAL_INT_SWFAULTDDEBV):
		pipe_id = PIPE_DDE_BV;
		break;
	}

	gen8_regread_aperture(device, GEN8_CP_INTERRUPT_STATUS_PIPE, &sw_status,
			      pipe_id, 0, 0);

	dev_crit_ratelimited(device->dev, "CP SW Fault pipe_id: %u %s\n", pipe_id,
			sw_status < ARRAY_SIZE(table) ? table[sw_status] : "UNKNOWN");

	if (sw_status & BIT(CP_SW_OPCODEERROR)) {
		gen8_regwrite_aperture(device, GEN8_CP_SQE_STAT_ADDR_PIPE, 1,
				pipe_id, 0, 0);
		gen8_regread_aperture(device, GEN8_CP_SQE_STAT_DATA_PIPE, &opcode,
				pipe_id, 0, 0);
		dev_crit_ratelimited(device->dev,
			"CP opcode error interrupt | opcode=0x%8.8x\n", opcode);
	}

	if (sw_status & BIT(CP_SW_REGISTERPROTECTIONERROR)) {
		gen8_regread_aperture(device, GEN8_CP_PROTECT_STATUS_PIPE, &status1,
			pipe_id, 0, 0);
		dev_crit_ratelimited(device->dev,
			"CP | Protected mode error | %s | addr=%lx | status=%x\n",
			FIELD_GET(GENMASK(20, 20), status1) ? "READ" : "WRITE",
			FIELD_GET(GENMASK(17, 0), status1), status1);
	}

	/* Clear aperture register */
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);
}

static void gen8_cp_hw_err_callback(struct adreno_device *adreno_dev, int bit)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 global_status;
	u32 hw_fault, sw_fault;

	kgsl_regread(device, GEN8_CP_INTERRUPT_STATUS_GLOBAL, &global_status);

	dev_crit_ratelimited(device->dev, "CP fault int_status_global=0x%x\n", global_status);

	hw_fault = FIELD_GET(GENMASK(6, 0), global_status);
	sw_fault = FIELD_GET(GENMASK(22, 16), global_status);

	if (hw_fault)
		gen8_get_cp_hwfault_status(adreno_dev, hw_fault);
	else if (sw_fault)
		gen8_get_cp_swfault_status(adreno_dev, sw_fault);
}

static void gen8_err_callback(struct adreno_device *adreno_dev, int bit)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	switch (bit) {
	case GEN8_INT_AHBERROR:
		{
		u32 err_details_0, err_details_1;

		kgsl_regread(device, GEN8_CP_RL_ERROR_DETAILS_0, &err_details_0);
		kgsl_regread(device, GEN8_CP_RL_ERROR_DETAILS_1, &err_details_1);
		dev_crit_ratelimited(device->dev,
			"CP: AHB bus error, CP_RL_ERROR_DETAILS_0:0x%x CP_RL_ERROR_DETAILS_1:0x%x\n",
			err_details_0, err_details_1);
		break;
		}
	case GEN8_INT_ATBASYNCFIFOOVERFLOW:
		dev_crit_ratelimited(device->dev, "RBBM: ATB ASYNC overflow\n");
		break;
	case GEN8_INT_ATBBUSOVERFLOW:
		dev_crit_ratelimited(device->dev, "RBBM: ATB bus overflow\n");
		break;
	case GEN8_INT_OUTOFBOUNDACCESS:
		dev_crit_ratelimited(device->dev, "UCHE: Out of bounds access\n");
		break;
	case GEN8_INT_UCHETRAPINTERRUPT:
		dev_crit_ratelimited(device->dev, "UCHE: Trap interrupt\n");
		break;
	case GEN8_INT_TSBWRITEERROR:
		{
		u32 lo, hi;

		kgsl_regread(device, GEN8_RBBM_SECVID_TSB_STATUS_LO, &lo);
		kgsl_regread(device, GEN8_RBBM_SECVID_TSB_STATUS_HI, &hi);

		dev_crit_ratelimited(device->dev, "TSB: Write error interrupt: Address: 0x%lx MID: %lu\n",
			FIELD_GET(GENMASK(16, 0), hi) << 32 | lo,
			FIELD_GET(GENMASK(31, 23), hi));
		break;
		}
	default:
		dev_crit_ratelimited(device->dev, "Unknown interrupt %d\n", bit);
	}
}

static const char *const uche_client[] = {
	"BR_VFD", "BR_SP", "BR_VSC", "BR_VPC",
	"BR_HLSQ", "BR_PC", "BR_LRZ", "BR_TP",
	"BV_VFD", "BV_SP", "BV_VSC", "BV_VPC",
	"BV_HLSQ", "BV_PC", "BV_LRZ", "BV_TP",
	"STCHE",
};

static const char *const uche_lpac_client[] = {
	"-", "SP_LPAC", "-", "-", "HLSQ_LPAC", "-", "-", "TP_LPAC"
};

#define SCOOBYDOO 0x5c00bd00

static const char *gen8_fault_block_uche(struct kgsl_device *device,
		char *str, int size, bool lpac)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	u32 uche_client_id = adreno_dev->uche_client_pf;
	const char *uche_client_str, *fault_block;

	/*
	 * Smmu driver takes a vote on CX gdsc before calling the kgsl
	 * pagefault handler. If there is contention for device mutex in this
	 * path and the dispatcher fault handler is holding this lock, trying
	 * to turn off CX gdsc will fail during the reset. So to avoid blocking
	 * here, try to lock device mutex and return if it fails.
	 */
	if (!mutex_trylock(&device->mutex))
		goto regread_fail;

	if (!kgsl_state_is_awake(device)) {
		mutex_unlock(&device->mutex);
		goto regread_fail;
	}

	kgsl_regread(device, GEN8_UCHE_CLIENT_PF, &uche_client_id);
	mutex_unlock(&device->mutex);

	/* Ignore the value if the gpu is in IFPC */
	if (uche_client_id == SCOOBYDOO) {
		uche_client_id = adreno_dev->uche_client_pf;
		goto regread_fail;
	}

	/* UCHE client id mask is bits [6:0] */
	uche_client_id &= GENMASK(6, 0);

regread_fail:
	if (lpac) {
		fault_block = "UCHE_LPAC";
		if (uche_client_id >= ARRAY_SIZE(uche_lpac_client))
			goto fail;
		uche_client_str = uche_lpac_client[uche_client_id];
	} else {
		fault_block = "UCHE";
		if (uche_client_id >= ARRAY_SIZE(uche_client))
			goto fail;
		uche_client_str = uche_client[uche_client_id];
	}

	snprintf(str, size, "%s: %s", fault_block, uche_client_str);
	return str;

fail:
	snprintf(str, size, "%s: Unknown (client_id: %u)",
			fault_block, uche_client_id);
	return str;
}

static const char *gen8_iommu_fault_block(struct kgsl_device *device,
		u32 fsynr1)
{
	u32 mid = fsynr1 & 0xff;
	static char str[36];

	switch (mid) {
	case 0x0:
		return "CP";
	case 0x1:
		return "UCHE: Unknown";
	case 0x2:
		return "UCHE_LPAC: Unknown";
	case 0x3:
		return gen8_fault_block_uche(device, str, sizeof(str), false);
	case 0x4:
		return "CCU";
	case 0x5:
		return "Flag cache";
	case 0x6:
		return "PREFETCH";
	case 0x7:
		return "GMU";
	case 0x8:
		return gen8_fault_block_uche(device, str, sizeof(str), true);
	case 0x9:
		return "UCHE_HPAC";
	}

	snprintf(str, sizeof(str), "Unknown (mid: %u)", mid);
	return str;
}

static void gen8_cp_callback(struct adreno_device *adreno_dev, int bit)
{
	if (adreno_is_preemption_enabled(adreno_dev))
		gen8_preemption_trigger(adreno_dev, true);

	adreno_scheduler_queue(adreno_dev);
}

/*
 * gen8_gpc_err_int_callback() - Isr for GPC error interrupts
 * @adreno_dev: Pointer to device
 * @bit: Interrupt bit
 */
static void gen8_gpc_err_int_callback(struct adreno_device *adreno_dev, int bit)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	/*
	 * GPC error is typically the result of mistake SW programming.
	 * Force GPU fault for this interrupt so that we can debug it
	 * with help of register dump.
	 */

	dev_crit(device->dev, "RBBM: GPC error\n");
	adreno_irqctrl(adreno_dev, 0);

	/* Trigger a fault in the dispatcher - this will effect a restart */
	adreno_scheduler_fault(adreno_dev, ADRENO_SOFT_FAULT);
}

/*
 * gen8_swfuse_violation_callback() - ISR for software fuse violation interrupt
 * @adreno_dev: Pointer to device
 * @bit: Interrupt bit
 */
static void gen8_swfuse_violation_callback(struct adreno_device *adreno_dev, int bit)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 status;

	/*
	 * SWFUSEVIOLATION error is typically the result of enabling software
	 * feature which is not supported by the hardware. Following are the
	 * Feature violation will be reported
	 * 1) FASTBLEND (BIT:0): NO Fault, RB will send the workload to legacy
	 * blender HW pipeline.
	 * 2) LPAC (BIT:1): Fault
	 * 3) RAYTRACING (BIT:2): Fault
	 */
	kgsl_regread(device, GEN8_RBBM_SW_FUSE_INT_STATUS, &status);

	/*
	 * RBBM_INT_CLEAR_CMD will not clear SWFUSEVIOLATION interrupt. Hence
	 * do explicit swfuse irq clear.
	 */
	kgsl_regwrite(device, GEN8_RBBM_SW_FUSE_INT_MASK, 0);

	dev_crit_ratelimited(device->dev,
		"RBBM: SW Feature Fuse violation status=0x%8.8x\n", status);

	/* Trigger a fault in the dispatcher for LPAC and RAYTRACING violation */
	if (status & GENMASK(GEN8_RAYTRACING_SW_FUSE, GEN8_LPAC_SW_FUSE)) {
		adreno_irqctrl(adreno_dev, 0);
		adreno_scheduler_fault(adreno_dev, ADRENO_HARD_FAULT);
	}
}

static const struct adreno_irq_funcs gen8_irq_funcs[32] = {
	ADRENO_IRQ_CALLBACK(NULL), /* 0 - RBBM_GPU_IDLE */
	ADRENO_IRQ_CALLBACK(gen8_err_callback), /* 1 - RBBM_AHB_ERROR */
	ADRENO_IRQ_CALLBACK(NULL), /* 2 - UNUSED */
	ADRENO_IRQ_CALLBACK(NULL), /* 3 - UNUSED */
	ADRENO_IRQ_CALLBACK(NULL), /* 4 - CPIPCINT0 */
	ADRENO_IRQ_CALLBACK(NULL), /* 5 - CPIPCINT1 */
	ADRENO_IRQ_CALLBACK(gen8_err_callback), /* 6 - ATBASYNCOVERFLOW */
	ADRENO_IRQ_CALLBACK(gen8_gpc_err_int_callback), /* 7 - GPC_ERR */
	ADRENO_IRQ_CALLBACK(gen8_preemption_callback),/* 8 - CP_SW */
	ADRENO_IRQ_CALLBACK(gen8_cp_hw_err_callback), /* 9 - CP_HW_ERROR */
	ADRENO_IRQ_CALLBACK(NULL), /* 10 - CP_CCU_FLUSH_DEPTH_TS */
	ADRENO_IRQ_CALLBACK(NULL), /* 11 - CP_CCU_FLUSH_COLOR_TS */
	ADRENO_IRQ_CALLBACK(NULL), /* 12 - CP_CCU_RESOLVE_TS */
	ADRENO_IRQ_CALLBACK(NULL), /* 13 - UNUSED */
	ADRENO_IRQ_CALLBACK(NULL), /* 14 - UNUSED */
	ADRENO_IRQ_CALLBACK(adreno_cp_callback), /* 15 - CP_RB_INT */
	ADRENO_IRQ_CALLBACK(NULL), /* 16 - CP_RB_INT_LPAC*/
	ADRENO_IRQ_CALLBACK(NULL), /* 17 - CP_RB_DONE_TS */
	ADRENO_IRQ_CALLBACK(NULL), /* 18 - UNUSED */
	ADRENO_IRQ_CALLBACK(NULL), /* 19 - UNUSED */
	ADRENO_IRQ_CALLBACK(gen8_cp_callback), /* 20 - CP_CACHE_FLUSH_TS */
	ADRENO_IRQ_CALLBACK(NULL), /* 21 - CP_CACHE_TS_LPAC */
	ADRENO_IRQ_CALLBACK(gen8_err_callback), /* 22 - RBBM_ATB_BUS_OVERFLOW */
	ADRENO_IRQ_CALLBACK(adreno_hang_int_callback), /* 23 - MISHANGDETECT */
	ADRENO_IRQ_CALLBACK(gen8_err_callback), /* 24 - UCHE_OOB_ACCESS */
	ADRENO_IRQ_CALLBACK(gen8_err_callback), /* 25 - UCHE_TRAP_INTR */
	ADRENO_IRQ_CALLBACK(NULL), /* 26 - DEBBUS_INTR_0 */
	ADRENO_IRQ_CALLBACK(NULL), /* 27 - DEBBUS_INTR_1 */
	ADRENO_IRQ_CALLBACK(gen8_err_callback), /* 28 - TSBWRITEERROR */
	ADRENO_IRQ_CALLBACK(gen8_swfuse_violation_callback), /* 29 - SWFUSEVIOLATION */
	ADRENO_IRQ_CALLBACK(NULL), /* 30 - ISDB_CPU_IRQ */
	ADRENO_IRQ_CALLBACK(NULL), /* 31 - ISDB_UNDER_DEBUG */
};

/*
 * If the AHB fence is not in ALLOW mode when we receive an RBBM
 * interrupt, something went wrong. This means that we cannot proceed
 * since the IRQ status and clear registers are not accessible.
 * This is usually harmless because the GMU will abort power collapse
 * and change the fence back to ALLOW. Poll so that this can happen.
 */
static int gen8_irq_poll_fence(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	u32 status, fence, fence_retries = 0;
	u64 a, b, c;

	a = gpudev->read_alwayson(adreno_dev);

	kgsl_regread(device, GEN8_GMUAO_AHB_FENCE_CTRL, &fence);

	while (fence != 0) {
		b = gpudev->read_alwayson(adreno_dev);

		/* Wait for small time before trying again */
		udelay(1);
		kgsl_regread(device, GEN8_GMUAO_AHB_FENCE_CTRL, &fence);

		if (fence_retries == 100 && fence != 0) {
			c = gpudev->read_alwayson(adreno_dev);

			kgsl_regread(device, GEN8_GMUAO_RBBM_INT_UNMASKED_STATUS_SHADOW,
				&status);

			dev_crit_ratelimited(device->dev,
				"status=0x%x Unmasked status=0x%x Mask=0x%x timestamps: %llx %llx %llx\n",
					status & adreno_dev->irq_mask, status,
					adreno_dev->irq_mask, a, b, c);
				return -ETIMEDOUT;
		}

		fence_retries++;
	}

	return 0;
}

static irqreturn_t gen8_hwsched_irq_handler(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	irqreturn_t ret = IRQ_NONE;
	u32 status;

	/*
	 * GPU can power down once the INT_0_STATUS is read below.
	 * But there still might be some register reads required so
	 * force the GMU/GPU into KEEPALIVE mode until done with the ISR.
	 */
	gen8_gpu_keepalive(adreno_dev, true);

	kgsl_regread(device, GEN8_RBBM_INT_0_STATUS, &status);

	kgsl_regwrite(device, GEN8_RBBM_INT_CLEAR_CMD, status);

	ret = adreno_irq_callbacks(adreno_dev, gen8_irq_funcs, status);

	trace_kgsl_gen8_irq_status(adreno_dev, status);

	/* If hard fault, then let snapshot turn off the keepalive */
	if (!(adreno_gpu_fault(adreno_dev) & ADRENO_HARD_FAULT))
		gen8_gpu_keepalive(adreno_dev, false);

	return ret;
}

static irqreturn_t gen8_irq_handler(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	irqreturn_t ret = IRQ_NONE;
	u32 status;

	/*
	 * GPU can power down once the INT_0_STATUS is read below.
	 * But there still might be some register reads required so
	 * force the GMU/GPU into KEEPALIVE mode until done with the ISR.
	 */
	gen8_gpu_keepalive(adreno_dev, true);

	if (gen8_irq_poll_fence(adreno_dev)) {
		adreno_scheduler_fault(adreno_dev, ADRENO_GMU_FAULT);
		goto done;
	}

	kgsl_regread(device, GEN8_RBBM_INT_0_STATUS, &status);

	kgsl_regwrite(device, GEN8_RBBM_INT_CLEAR_CMD, status);

	ret = adreno_irq_callbacks(adreno_dev, gen8_irq_funcs, status);

	trace_kgsl_gen8_irq_status(adreno_dev, status);

done:
	/* If hard fault, then let snapshot turn off the keepalive */
	if (!(adreno_gpu_fault(adreno_dev) & ADRENO_HARD_FAULT))
		gen8_gpu_keepalive(adreno_dev, false);

	return ret;
}

static irqreturn_t gen8_cx_host_irq_handler(int irq, void *data)
{
	struct kgsl_device *device = data;
	u32 status;

	kgsl_regread(device, GEN8_GPU_CX_MISC_INT_0_STATUS, &status);
	kgsl_regwrite(device, GEN8_GPU_CX_MISC_INT_CLEAR_CMD, status);

	if (status & BIT(GEN8_CX_MISC_GPU_CC_IRQ))
		KGSL_PWRCTRL_LOG_FREQLIM(device);

	if (status & ~GEN8_CX_MISC_INT_MASK)
		dev_err_ratelimited(device->dev, "Unhandled CX MISC interrupts 0x%lx\n",
			status & ~GEN8_CX_MISC_INT_MASK);

	return IRQ_HANDLED;
}

int gen8_probe_common(struct platform_device *pdev,
	struct adreno_device *adreno_dev, u32 chipid,
	const struct adreno_gpu_core *gpucore)
{
	const struct adreno_gpudev *gpudev = gpucore->gpudev;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gen8_core *gen8_core = container_of(gpucore,
			struct adreno_gen8_core, base);
	int ret;

	adreno_dev->gpucore = gpucore;
	adreno_dev->chipid = chipid;

	adreno_reg_offset_init(gpudev->reg_offsets);

	adreno_dev->hwcg_enabled = true;
	adreno_dev->uche_client_pf = 1;

	kgsl_pwrscale_fast_bus_hint(gen8_core->fast_bus_hint);
	device->pwrctrl.cx_cfg_gdsc_offset = GEN8_GPU_CC_CX_CFG_GDSCR;

	device->pwrctrl.rt_bus_hint = gen8_core->rt_bus_hint;

	device->cx_host_irq_num = kgsl_request_irq_optional(pdev,
		"cx_host_irq", gen8_cx_host_irq_handler, device);

	ret = adreno_device_probe(pdev, adreno_dev);
	if (ret)
		return ret;

	if (adreno_preemption_feature_set(adreno_dev)) {
		adreno_dev->preempt.preempt_level = gen8_core->preempt_level;
		adreno_dev->preempt.skipsaverestore = true;
		adreno_dev->preempt.usesgmem = true;
		set_bit(ADRENO_DEVICE_PREEMPTION, &adreno_dev->priv);
	}

	/* debugfs node for ACD calibration */
	debugfs_create_file("acd_calibrate", 0644, device->d_debugfs, device, &acd_cal_fops);

	gen8_coresight_init(adreno_dev);

	/* Dump additional AQE 16KB data on top of default 128KB(64(BR)+64(BV)) */
	device->snapshot_ctxt_record_size = ADRENO_FEATURE(adreno_dev, ADRENO_AQE) ?
			(GEN8_SNAPSHOT_CTXRECORD_SIZE_IN_BYTES + SZ_16K) :
			GEN8_SNAPSHOT_CTXRECORD_SIZE_IN_BYTES;

	return 0;
}

/* Register offset defines for Gen8, in order of enum adreno_regs */
static u32 gen8_register_offsets[ADRENO_REG_REGISTER_MAX] = {
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_BASE, GEN8_CP_RB_BASE_LO_GC),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_BASE_HI, GEN8_CP_RB_BASE_HI_GC),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_LPAC_RB_BASE, GEN8_CP_RB_BASE_LO_LPAC),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_LPAC_RB_BASE_HI, GEN8_CP_RB_BASE_HI_LPAC),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_RPTR, GEN8_CP_RB_RPTR_BR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_WPTR, GEN8_CP_RB_WPTR_GC),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_ME_CNTL, GEN8_CP_SQE_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BASE, GEN8_CP_IB1_BASE_LO_PIPE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BASE_HI, GEN8_CP_IB1_BASE_HI_PIPE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BUFSZ, GEN8_CP_IB1_REM_SIZE_PIPE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BASE, GEN8_CP_IB2_BASE_LO_PIPE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BASE_HI, GEN8_CP_IB2_BASE_HI_PIPE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BUFSZ, GEN8_CP_IB2_REM_SIZE_PIPE),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_STATUS, GEN8_RBBM_STATUS),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_INT_0_MASK, GEN8_RBBM_INT_0_MASK),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_SW_RESET_CMD, GEN8_RBBM_SW_RESET_CMD),
	ADRENO_REG_DEFINE(ADRENO_REG_GMU_AO_HOST_INTERRUPT_MASK,
			GEN8_GMUAO_AO_HOST_INTERRUPT_MASK),
	ADRENO_REG_DEFINE(ADRENO_REG_GMU_GMU2HOST_INTR_MASK,
			GEN8_GMUCX_GMU2HOST_INTR_MASK),
};

static u32 _get_pipeid(u32 groupid)
{
	switch (groupid) {
	case KGSL_PERFCOUNTER_GROUP_BV_PC:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_BV_VFD:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_BV_VPC:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_BV_TSE:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_BV_RAS:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_BV_LRZ:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_BV_HLSQ:
		return PIPE_BV;
	case KGSL_PERFCOUNTER_GROUP_PC:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_VFD:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_HLSQ:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_VPC:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_CCU:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_CMP:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_TSE:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_RAS:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_LRZ:
		fallthrough;
	case KGSL_PERFCOUNTER_GROUP_RB:
		return PIPE_BR;
	default:
		return PIPE_NONE;
	}
}

static bool gen8_acquire_cp_semaphore(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 sem, i;

	for (i = 0; i < 10; i++) {
		kgsl_regwrite(device, GEN8_CP_SEMAPHORE_REG_0, BIT(8));

		/*
		 * Make sure the previous register write is posted before
		 * checking the CP sempahore status
		 */
		mb();

		kgsl_regread(device, GEN8_CP_SEMAPHORE_REG_0, &sem);
		if (sem)
			return true;

		udelay(10);
	}

	/* Check CP semaphore status one last time */
	kgsl_regread(device, GEN8_CP_SEMAPHORE_REG_0, &sem);

	if (!sem)
		return false;

	return true;
}

static void gen8_release_cp_semaphore(struct adreno_device *adreno_dev)
{
	kgsl_regwrite(KGSL_DEVICE(adreno_dev), GEN8_CP_SEMAPHORE_REG_0, 0);
}

int gen8_perfcounter_remove(struct adreno_device *adreno_dev,
			    struct adreno_perfcount_register *reg, u32 groupid)
{
	const struct adreno_perfcounters *counters = ADRENO_PERFCOUNTERS(adreno_dev);
	struct gen8_device *gen8_dev = container_of(adreno_dev, struct gen8_device, adreno_dev);
	const struct adreno_perfcount_group *group;
	void *ptr = adreno_dev->pwrup_reglist->hostptr;
	struct cpu_gpu_lock *lock = ptr;
	u32 offset = ((lock->ifpc_list_len + lock->preemption_list_len) * 2) +
			(gen8_dev->ext_pwrup_list_len * 3);
	int i, last_offset, num_removed, start_offset = -1;
	u32 *data = ptr + sizeof(*lock), pipe = FIELD_PREP(GENMASK(13, 12), _get_pipeid(groupid));
	u16 perfcntr_list_len = lock->dynamic_list_len - gen8_dev->ext_pwrup_list_len;

	if (!perfcntr_list_len)
		return -EINVAL;

	group = &(counters->groups[groupid]);

	if (!(group->flags & ADRENO_PERFCOUNTER_GROUP_RESTORE)) {
		if (perfcntr_list_len != 2)
			return 0;

		if (kgsl_hwlock(lock)) {
			kgsl_hwunlock(lock);
			return -EBUSY;
		}
		goto disable_perfcounter;
	}

	last_offset = offset + (perfcntr_list_len * 3);

	/* Look for the perfcounter to remove in the list */
	for (i = 0; i < perfcntr_list_len - 2; i++) {
		if ((data[offset + 1] == reg->select) && (data[offset] == pipe)) {
			start_offset = offset;
			break;
		}
		offset += 3;
	}

	if (start_offset == -1)
		return -ENOENT;

	for (i = 0; i < PERFCOUNTER_REG_DEPENDENCY_LEN && reg->reg_dependency[i]; i++)
		offset += 3;

	if (kgsl_hwlock(lock)) {
		kgsl_hwunlock(lock);
		return -EBUSY;
	}

	/* Let offset point to the first entry that is going to be retained */
	offset += 3;

	memcpy(&data[start_offset], &data[offset], (last_offset - offset) * sizeof(u32));

	memset(&data[start_offset + (last_offset - offset)], 0,
			(offset - start_offset) * sizeof(u32));

	num_removed = offset - start_offset;
	do_div(num_removed, 3);
	lock->dynamic_list_len -= num_removed;

disable_perfcounter:
	/*
	 * If dynamic list length is 2 and no_restore_count is 0, then we can remove
	 * the perfcounter controls from the list.
	 */
	if (perfcntr_list_len == 2 && !adreno_dev->no_restore_count) {
		memset(&data[offset], 0, 6 * sizeof(u32));
		lock->dynamic_list_len = gen8_dev->ext_pwrup_list_len;
	}

	kgsl_hwunlock(lock);
	return 0;
}

int gen8_perfcounter_update(struct adreno_device *adreno_dev,
	struct adreno_perfcount_register *reg, bool update_reg, u32 pipe, unsigned long flags)
{
	struct gen8_device *gen8_dev = container_of(adreno_dev, struct gen8_device, adreno_dev);
	void *ptr = adreno_dev->pwrup_reglist->hostptr;
	struct cpu_gpu_lock *lock = ptr;
	u32 offset = ((lock->ifpc_list_len + lock->preemption_list_len) * 2) +
			(gen8_dev->ext_pwrup_list_len * 3);
	u32 *data = ptr + sizeof(*lock);
	int i, start_offset = -1;
	u16 perfcntr_list_len = lock->dynamic_list_len - gen8_dev->ext_pwrup_list_len;
	unsigned long irq_flags;
	int ret = 0;

	if (!ADRENO_ACQUIRE_CP_SEMAPHORE(adreno_dev, irq_flags))
		return -EBUSY;

	if (flags & ADRENO_PERFCOUNTER_GROUP_RESTORE) {
		for (i = 0; i < perfcntr_list_len - 2; i++) {
			if ((data[offset + 1] == reg->select) && (data[offset] == pipe)) {
				start_offset = offset;
				break;
			}

			offset += 3;
		}
	} else if (perfcntr_list_len) {
		goto update;
	}

	if (kgsl_hwlock(lock)) {
		kgsl_hwunlock(lock);
		ret = -EBUSY;
		goto err;
	}

	/*
	 * If the perfcounter select register is already present in reglist
	 * update it, otherwise append the <aperture, select register, value>
	 * triplet to the end of the list.
	 */
	if (start_offset != -1) {
		data[offset + 2] = reg->countable;
		for (i = 0; i < PERFCOUNTER_REG_DEPENDENCY_LEN && reg->reg_dependency[i]; i++) {
			offset += 3;
			data[offset + 2] = reg->countable;
		}
		kgsl_hwunlock(lock);
		goto update;
	}

	/* Initialize the lock->dynamic_list_len to account for perfcounter controls */
	if (!perfcntr_list_len)
		lock->dynamic_list_len = gen8_dev->ext_pwrup_list_len + 2;

	/*
	 * For all targets GEN8_SLICE_RBBM_PERFCTR_CNTL needs to be the last entry,
	 * so overwrite the existing GEN8_SLICE_RBBM_PERFCNTL_CNTL and add it back to
	 * the end.
	 */
	if (flags & ADRENO_PERFCOUNTER_GROUP_RESTORE) {
		data[offset++] = pipe;
		data[offset++] = reg->select;
		data[offset++] = reg->countable;
		lock->dynamic_list_len++;

		for (i = 0; i < PERFCOUNTER_REG_DEPENDENCY_LEN && reg->reg_dependency[i]; i++) {
			data[offset++] = pipe;
			data[offset++] = reg->reg_dependency[i];
			data[offset++] = reg->countable;
			lock->dynamic_list_len++;
		}
	}

	data[offset++] = FIELD_PREP(GENMASK(15, 12), PIPE_NONE);
	data[offset++] = GEN8_RBBM_PERFCTR_CNTL;
	data[offset++] = 1;

	data[offset++] = FIELD_PREP(GENMASK(15, 12), PIPE_NONE);
	data[offset++] = GEN8_RBBM_SLICE_PERFCTR_CNTL;
	data[offset++] = 1;

	kgsl_hwunlock(lock);

update:
	if (update_reg) {
		struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

		kgsl_regwrite(device, reg->select, reg->countable);

		for (i = 0; i < PERFCOUNTER_REG_DEPENDENCY_LEN && reg->reg_dependency[i]; i++)
			kgsl_regwrite(device, reg->reg_dependency[i], reg->countable);
	}

err:
	ADRENO_RELEASE_CP_SEMAPHORE(adreno_dev, irq_flags);
	return ret;
}

static u64 gen8_read_alwayson(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 lo = 0, hi = 0, tmp = 0;

	/* Always use the GMU AO counter when doing a AHB read */
	gmu_core_regread(device, GEN8_GMUCX_AO_COUNTER_HI, &hi);
	gmu_core_regread(device, GEN8_GMUCX_AO_COUNTER_LO, &lo);

	/* Check for overflow */
	gmu_core_regread(device, GEN8_GMUCX_AO_COUNTER_HI, &tmp);

	if (hi != tmp) {
		gmu_core_regread(device, GEN8_GMUCX_AO_COUNTER_LO,
				&lo);
		hi = tmp;
	}

	return (((u64) hi) << 32) | lo;
}

static int gen8_lpac_store(struct adreno_device *adreno_dev, bool enable)
{
	if (!ADRENO_FEATURE(adreno_dev, ADRENO_LPAC))
		return -EINVAL;

	if (!(adreno_dev->feature_fuse & BIT(GEN8_LPAC_SW_FUSE)) ||
		(adreno_dev->lpac_enabled == enable))
		return 0;

	/* Power down the GPU before changing the lpac setting */
	return adreno_power_cycle_bool(adreno_dev, &adreno_dev->lpac_enabled, enable);
}

static void gen8_remove(struct adreno_device *adreno_dev)
{
	if (adreno_preemption_feature_set(adreno_dev))
		del_timer(&adreno_dev->preempt.timer);
}

static void gen8_read_bus_stats(struct kgsl_device *device,
		struct kgsl_power_stats *stats,
		struct adreno_busy_data *busy)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	u64 ram_cycles, starved_ram;

	ram_cycles = counter_delta(device, adreno_dev->ram_cycles_lo,
		&busy->bif_ram_cycles);

	starved_ram = counter_delta(device, adreno_dev->starved_ram_lo,
		&busy->bif_starved_ram);

	ram_cycles += counter_delta(device,
		adreno_dev->ram_cycles_lo_ch1_read,
		&busy->bif_ram_cycles_read_ch1);

	ram_cycles += counter_delta(device,
		adreno_dev->ram_cycles_lo_ch0_write,
		&busy->bif_ram_cycles_write_ch0);

	ram_cycles += counter_delta(device,
		adreno_dev->ram_cycles_lo_ch1_write,
		&busy->bif_ram_cycles_write_ch1);

	starved_ram += counter_delta(device,
		adreno_dev->starved_ram_lo_ch1,
		&busy->bif_starved_ram_ch1);

	stats->ram_time = ram_cycles;
	stats->ram_wait = starved_ram;
}

static void gen8_power_stats(struct adreno_device *adreno_dev,
		struct kgsl_power_stats *stats)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_busy_data *busy = &adreno_dev->busy_data;
	u64 gpu_busy;

	/* Set the GPU busy counter for frequency scaling */
	gpu_busy = counter_delta(device, GEN8_GMUCX_POWER_COUNTER_XOCLK_L_0,
		&busy->gpu_busy);

	stats->busy_time = gpu_busy * 10;
	do_div(stats->busy_time, 192);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_IFPC)) {
		u32 ifpc = counter_delta(device,
			GEN8_GMUCX_POWER_COUNTER_XOCLK_L_4,
			&busy->num_ifpc);

		adreno_dev->ifpc_count += ifpc;
		if (ifpc > 0)
			trace_adreno_ifpc_count(adreno_dev->ifpc_count);
	}

	if (device->pwrctrl.bus_control)
		gen8_read_bus_stats(device, stats, busy);

	if (adreno_dev->bcl_enabled) {
		u32 a, b, c, bcl_throttle;

		a = counter_delta(device, GEN8_GMUCX_POWER_COUNTER_XOCLK_L_1,
			&busy->throttle_cycles[0]);

		b = counter_delta(device, GEN8_GMUCX_POWER_COUNTER_XOCLK_L_2,
			&busy->throttle_cycles[1]);

		c = counter_delta(device, GEN8_GMUCX_POWER_COUNTER_XOCLK_L_3,
			&busy->throttle_cycles[2]);

		if (a || b || c)
			trace_kgsl_bcl_clock_throttling(a, b, c);

		bcl_throttle = counter_delta(device,
					GEN8_GMUCX_POWER_COUNTER_XOCLK_L_5, &busy->bcl_throttle);
		/*
		 * This counts number of cycles throttled in XO cycles. Convert it to
		 * micro seconds by dividing by XO freq which is 19.2MHz.
		 */
		adreno_dev->bcl_throttle_time_us += ((bcl_throttle * 10) / 192);
	}
}

static void gen8_set_isdb_breakpoint_registers(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct clk *clk;
	int ret;

	if (!device->set_isdb_breakpoint || device->ftbl->is_hwcg_on(device)
			|| device->qdss_gfx_virt == NULL || !device->force_panic)
		return;

	clk = clk_get(&device->pdev->dev, "apb_pclk");

	if (IS_ERR(clk)) {
		dev_err(device->dev, "Unable to get QDSS clock\n");
		goto err;
	}

	ret = clk_prepare_enable(clk);

	if (ret) {
		dev_err(device->dev, "QDSS Clock enable error: %d\n", ret);
		clk_put(clk);
		goto err;
	}

	/* Issue break command for SPs */
	isdb_write(device->qdss_gfx_virt, 0x0000);
	isdb_write(device->qdss_gfx_virt, 0x1000);
	isdb_write(device->qdss_gfx_virt, 0x2000);
	isdb_write(device->qdss_gfx_virt, 0x3000);
	isdb_write(device->qdss_gfx_virt, 0x4000);
	isdb_write(device->qdss_gfx_virt, 0x5000);
	isdb_write(device->qdss_gfx_virt, 0x6000);
	isdb_write(device->qdss_gfx_virt, 0x7000);
	isdb_write(device->qdss_gfx_virt, 0x8000);
	isdb_write(device->qdss_gfx_virt, 0x9000);
	isdb_write(device->qdss_gfx_virt, 0xa000);
	isdb_write(device->qdss_gfx_virt, 0xb000);

	clk_disable_unprepare(clk);
	clk_put(clk);

	return;

err:
	/* Do not force kernel panic if isdb writes did not go through */
	device->force_panic = false;
}

static void gen8_swfuse_irqctrl(struct adreno_device *adreno_dev, bool state)
{
		kgsl_regwrite(KGSL_DEVICE(adreno_dev), GEN8_RBBM_SW_FUSE_INT_MASK,
			state ? GEN8_SW_FUSE_INT_MASK : 0);
}

static void gen8_lpac_fault_header(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_context *drawctxt;
	u32 status = 0, rptr = 0, wptr = 0, ib1sz = 0, ib2sz = 0, ib3sz = 0;
	u64 ib1base = 0, ib2base = 0, ib3base = 0;
	bool gx_on = adreno_gx_is_on(adreno_dev);

	drawctxt = ADRENO_CONTEXT(drawobj->context);
	drawobj->context->last_faulted_cmd_ts = drawobj->timestamp;
	drawobj->context->total_fault_count++;

	pr_context(device, drawobj->context,
		   "LPAC ctx %u ctx_type %s ts %u policy %lX dispatch_queue=%d\n",
		   drawobj->context->id, kgsl_context_type(drawctxt->type),
		   drawobj->timestamp, CMDOBJ(drawobj)->fault_recovery,
		   drawobj->context->gmu_dispatch_queue);

	pr_context(device, drawobj->context, "lpac cmdline: %s\n",
		   drawctxt->base.proc_priv->cmdline);

	if (!gen8_gmu_rpmh_pwr_state_is_active(device) || !gx_on)
		goto done;

	kgsl_regread(device, GEN8_RBBM_LPAC_STATUS, &status);
	kgsl_regread(device, GEN8_CP_RB_RPTR_LPAC, &rptr);
	kgsl_regread(device, GEN8_CP_RB_WPTR_LPAC, &wptr);
	gen8_regread64_aperture(device, GEN8_CP_IB1_BASE_LO_PIPE,
			GEN8_CP_IB1_BASE_HI_PIPE, &ib1base, PIPE_LPAC, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB1_REM_SIZE_PIPE, &ib1sz, PIPE_LPAC, 0, 0);
	gen8_regread64_aperture(device, GEN8_CP_IB2_BASE_LO_PIPE,
			GEN8_CP_IB2_BASE_HI_PIPE, &ib2base, PIPE_LPAC, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB2_REM_SIZE_PIPE, &ib2sz, PIPE_LPAC, 0, 0);
	gen8_regread64_aperture(device, GEN8_CP_IB3_BASE_LO_PIPE,
			GEN8_CP_IB3_BASE_HI_PIPE, &ib3base, PIPE_LPAC, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB3_REM_SIZE_PIPE, &ib3sz, PIPE_LPAC, 0, 0);
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	pr_context(device, drawobj->context,
		   "LPAC: status %8.8X rb %4.4x/%4.4x ib1 %16.16llX/%4.4x ib2 %16.16llX/%4.4x ib3 %16.16llX/%4.4x\n",
		   status, rptr, wptr, ib1base, ib1sz, ib2base, ib2sz, ib3base, ib3sz);

done:
	trace_adreno_gpu_fault(drawobj->context->id, drawobj->timestamp, status,
		rptr, wptr, ib1base, ib1sz, ib2base, ib2sz,
		adreno_get_level(drawobj->context));

}

static void gen8_fault_header(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_context *drawctxt;
	u32 status = 0, rptr = 0, wptr = 0, ib1sz = 0, ib2sz = 0, ib3sz, rptr_bv = 0;
	u32 ib1sz_bv = 0, ib2sz_bv = 0, ib3sz_bv, gfx_status, gfx_br_status, gfx_bv_status;
	u64 ib1base = 0, ib2base = 0, ib3base, ib1base_bv = 0, ib2base_bv, ib3base_bv;
	u32 ctxt_id = 0, ts = 0;
	int rb_id = -1;
	bool gx_on = adreno_gx_is_on(adreno_dev);

	if (drawobj) {
		drawctxt = ADRENO_CONTEXT(drawobj->context);
		drawobj->context->last_faulted_cmd_ts = drawobj->timestamp;
		drawobj->context->total_fault_count++;
		ctxt_id = drawobj->context->id;
		ts = drawobj->timestamp;
		rb_id = adreno_get_level(drawobj->context);

		pr_context(device, drawobj->context, "ctx %u ctx_type %s ts %u policy %lX\n",
			   drawobj->context->id, kgsl_context_type(drawctxt->type),
			   drawobj->timestamp, CMDOBJ(drawobj)->fault_recovery);

		pr_context(device, drawobj->context, "cmdline: %s\n",
			   drawctxt->base.proc_priv->cmdline);
	}

	if (!gen8_gmu_rpmh_pwr_state_is_active(device) || !gx_on)
		goto done;

	kgsl_regread(device, GEN8_RBBM_STATUS, &status);
	kgsl_regread(device, GEN8_RBBM_GFX_STATUS, &gfx_status);
	kgsl_regread(device, GEN8_RBBM_GFX_BV_STATUS, &gfx_bv_status);
	kgsl_regread(device, GEN8_RBBM_GFX_BR_STATUS, &gfx_br_status);
	kgsl_regread(device, GEN8_CP_RB_RPTR_BR, &rptr);
	kgsl_regread(device, GEN8_CP_RB_WPTR_GC, &wptr);
	kgsl_regread(device, GEN8_CP_RB_RPTR_BV, &rptr_bv);
	gen8_regread64_aperture(device, GEN8_CP_IB1_BASE_LO_PIPE,
			GEN8_CP_IB1_BASE_HI_PIPE, &ib1base, PIPE_BR, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB1_REM_SIZE_PIPE, &ib1sz, PIPE_BR, 0, 0);
	gen8_regread64_aperture(device, GEN8_CP_IB2_BASE_LO_PIPE,
			GEN8_CP_IB2_BASE_HI_PIPE, &ib2base, PIPE_BR, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB2_REM_SIZE_PIPE, &ib2sz, PIPE_BR, 0, 0);
	gen8_regread64_aperture(device, GEN8_CP_IB3_BASE_LO_PIPE,
			GEN8_CP_IB3_BASE_HI_PIPE, &ib3base, PIPE_BR, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB3_REM_SIZE_PIPE, &ib3sz, PIPE_BR, 0, 0);
	gen8_regread64_aperture(device, GEN8_CP_IB1_BASE_LO_PIPE,
			GEN8_CP_IB1_BASE_HI_PIPE, &ib1base_bv, PIPE_BV, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB1_REM_SIZE_PIPE, &ib1sz_bv, PIPE_BV, 0, 0);
	gen8_regread64_aperture(device, GEN8_CP_IB2_BASE_LO_PIPE,
			GEN8_CP_IB2_BASE_HI_PIPE, &ib2base_bv, PIPE_BV, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB2_REM_SIZE_PIPE, &ib2sz_bv, PIPE_BV, 0, 0);
	gen8_regread64_aperture(device, GEN8_CP_IB3_BASE_LO_PIPE,
			GEN8_CP_IB3_BASE_HI_PIPE, &ib3base_bv, PIPE_BV, 0, 0);
	gen8_regread_aperture(device, GEN8_CP_IB3_REM_SIZE_PIPE, &ib3sz_bv, PIPE_BV, 0, 0);
	gen8_host_aperture_set(adreno_dev, 0, 0, 0);

	dev_err(device->dev,
		"status %8.8X gfx_status %8.8X gfx_br_status %8.8X gfx_bv_status %8.8X\n",
		status, gfx_status, gfx_br_status, gfx_bv_status);

	dev_err(device->dev,
		"BR: rb %4.4x/%4.4x ib1 %16.16llX/%4.4x ib2 %16.16llX/%4.4x ib3 %16.16llX/%4.4x\n",
		rptr, wptr, ib1base, ib1sz, ib2base, ib2sz, ib3base, ib3sz);

	dev_err(device->dev,
		"BV: rb %4.4x/%4.4x ib1 %16.16llX/%4.4x ib2 %16.16llX/%4.4x ib3 %16.16llX/%4.4x\n",
		rptr_bv, wptr, ib1base_bv, ib1sz_bv, ib2base_bv, ib2sz_bv, ib3base_bv, ib3sz_bv);

done:
	trace_adreno_gpu_fault(ctxt_id, ts, status,
		rptr, wptr, ib1base, ib1sz, ib2base, ib2sz, rb_id);
}

const struct gen8_gpudev adreno_gen8_hwsched_gpudev = {
	.base = {
		.reg_offsets = gen8_register_offsets,
		.probe = gen8_hwsched_probe,
		.snapshot = gen8_hwsched_snapshot,
		.irq_handler = gen8_hwsched_irq_handler,
		.iommu_fault_block = gen8_iommu_fault_block,
		.context_detach = gen8_hwsched_context_detach,
		.read_alwayson = gen8_read_alwayson,
		.reset = gen8_hwsched_reset_replay,
		.power_ops = &gen8_hwsched_power_ops,
		.power_stats = gen8_power_stats,
		.hw_isidle = gen8_hw_isidle,
		.add_to_va_minidump = gen8_hwsched_add_to_minidump,
		.gx_is_on = gen8_gmu_gx_is_on,
		.send_recurring_cmdobj = gen8_hwsched_send_recurring_cmdobj,
		.perfcounter_remove = gen8_perfcounter_remove,
		.set_isdb_breakpoint_registers = gen8_set_isdb_breakpoint_registers,
		.context_destroy = gen8_hwsched_context_destroy,
		.lpac_store = gen8_lpac_store,
		.get_uche_trap_base = gen8_get_uche_trap_base,
		.fault_header = gen8_fault_header,
		.lpac_fault_header = gen8_lpac_fault_header,
		.acquire_cp_semaphore = gen8_acquire_cp_semaphore,
		.release_cp_semaphore = gen8_release_cp_semaphore,
	},
	.hfi_probe = gen8_hwsched_hfi_probe,
	.hfi_remove = gen8_hwsched_hfi_remove,
	.handle_watchdog = gen8_hwsched_handle_watchdog,
};

const struct gen8_gpudev adreno_gen8_gmu_gpudev = {
	.base = {
		.reg_offsets = gen8_register_offsets,
		.probe = gen8_gmu_device_probe,
		.snapshot = gen8_gmu_snapshot,
		.irq_handler = gen8_irq_handler,
		.rb_start = gen8_rb_start,
		.gpu_keepalive = gen8_gpu_keepalive,
		.hw_isidle = gen8_hw_isidle,
		.iommu_fault_block = gen8_iommu_fault_block,
		.reset = gen8_gmu_reset,
		.preemption_schedule = gen8_preemption_schedule,
		.read_alwayson = gen8_read_alwayson,
		.power_ops = &gen8_gmu_power_ops,
		.remove = gen8_remove,
		.ringbuffer_submitcmd = gen8_ringbuffer_submitcmd,
		.power_stats = gen8_power_stats,
		.add_to_va_minidump = gen8_gmu_add_to_minidump,
		.gx_is_on = gen8_gmu_gx_is_on,
		.perfcounter_remove = gen8_perfcounter_remove,
		.set_isdb_breakpoint_registers = gen8_set_isdb_breakpoint_registers,
		.swfuse_irqctrl = gen8_swfuse_irqctrl,
		.get_uche_trap_base = gen8_get_uche_trap_base,
		.fault_header = gen8_fault_header,
		.acquire_cp_semaphore = gen8_acquire_cp_semaphore,
		.release_cp_semaphore = gen8_release_cp_semaphore,
	},
	.hfi_probe = gen8_gmu_hfi_probe,
	.handle_watchdog = gen8_gmu_handle_watchdog,
};
