// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*/

#include "adreno.h"
#include "adreno_a6xx.h"
#include "adreno_coresight.h"

static struct adreno_coresight_register a6xx_coresight_regs[] = {
	{ A6XX_DBGC_CFG_DBGBUS_SEL_A },
	{ A6XX_DBGC_CFG_DBGBUS_SEL_B },
	{ A6XX_DBGC_CFG_DBGBUS_SEL_C },
	{ A6XX_DBGC_CFG_DBGBUS_SEL_D },
	{ A6XX_DBGC_CFG_DBGBUS_CNTLT },
	{ A6XX_DBGC_CFG_DBGBUS_CNTLM },
	{ A6XX_DBGC_CFG_DBGBUS_OPL },
	{ A6XX_DBGC_CFG_DBGBUS_OPE },
	{ A6XX_DBGC_CFG_DBGBUS_IVTL_0 },
	{ A6XX_DBGC_CFG_DBGBUS_IVTL_1 },
	{ A6XX_DBGC_CFG_DBGBUS_IVTL_2 },
	{ A6XX_DBGC_CFG_DBGBUS_IVTL_3 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKL_0 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKL_1 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKL_2 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKL_3 },
	{ A6XX_DBGC_CFG_DBGBUS_BYTEL_0 },
	{ A6XX_DBGC_CFG_DBGBUS_BYTEL_1 },
	{ A6XX_DBGC_CFG_DBGBUS_IVTE_0 },
	{ A6XX_DBGC_CFG_DBGBUS_IVTE_1 },
	{ A6XX_DBGC_CFG_DBGBUS_IVTE_2 },
	{ A6XX_DBGC_CFG_DBGBUS_IVTE_3 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKE_0 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKE_1 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKE_2 },
	{ A6XX_DBGC_CFG_DBGBUS_MASKE_3 },
	{ A6XX_DBGC_CFG_DBGBUS_NIBBLEE },
	{ A6XX_DBGC_CFG_DBGBUS_PTRC0 },
	{ A6XX_DBGC_CFG_DBGBUS_PTRC1 },
	{ A6XX_DBGC_CFG_DBGBUS_LOADREG },
	{ A6XX_DBGC_CFG_DBGBUS_IDX },
	{ A6XX_DBGC_CFG_DBGBUS_CLRC },
	{ A6XX_DBGC_CFG_DBGBUS_LOADIVT },
	{ A6XX_DBGC_VBIF_DBG_CNTL },
	{ A6XX_DBGC_DBG_LO_HI_GPIO },
	{ A6XX_DBGC_EXT_TRACE_BUS_CNTL },
	{ A6XX_DBGC_READ_AHB_THROUGH_DBG },
	{ A6XX_DBGC_CFG_DBGBUS_TRACE_BUF1 },
	{ A6XX_DBGC_CFG_DBGBUS_TRACE_BUF2 },
	{ A6XX_DBGC_EVT_CFG },
	{ A6XX_DBGC_EVT_INTF_SEL_0 },
	{ A6XX_DBGC_EVT_INTF_SEL_1 },
	{ A6XX_DBGC_PERF_ATB_CFG },
	{ A6XX_DBGC_PERF_ATB_COUNTER_SEL_0 },
	{ A6XX_DBGC_PERF_ATB_COUNTER_SEL_1 },
	{ A6XX_DBGC_PERF_ATB_COUNTER_SEL_2 },
	{ A6XX_DBGC_PERF_ATB_COUNTER_SEL_3 },
	{ A6XX_DBGC_PERF_ATB_TRIG_INTF_SEL_0 },
	{ A6XX_DBGC_PERF_ATB_TRIG_INTF_SEL_1 },
	{ A6XX_DBGC_PERF_ATB_DRAIN_CMD },
	{ A6XX_DBGC_ECO_CNTL },
	{ A6XX_DBGC_AHB_DBG_CNTL },
};

static struct adreno_coresight_register a6xx_coresight_regs_cx[] = {
	{ A6XX_CX_DBGC_CFG_DBGBUS_SEL_A },
	{ A6XX_CX_DBGC_CFG_DBGBUS_SEL_B },
	{ A6XX_CX_DBGC_CFG_DBGBUS_SEL_C },
	{ A6XX_CX_DBGC_CFG_DBGBUS_SEL_D },
	{ A6XX_CX_DBGC_CFG_DBGBUS_CNTLT },
	{ A6XX_CX_DBGC_CFG_DBGBUS_CNTLM },
	{ A6XX_CX_DBGC_CFG_DBGBUS_OPL },
	{ A6XX_CX_DBGC_CFG_DBGBUS_OPE },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTL_0 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTL_1 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTL_2 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTL_3 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKL_0 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKL_1 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKL_2 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKL_3 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_BYTEL_0 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_BYTEL_1 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTE_0 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTE_1 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTE_2 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IVTE_3 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKE_0 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKE_1 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKE_2 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_MASKE_3 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_NIBBLEE },
	{ A6XX_CX_DBGC_CFG_DBGBUS_PTRC0 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_PTRC1 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_LOADREG },
	{ A6XX_CX_DBGC_CFG_DBGBUS_IDX },
	{ A6XX_CX_DBGC_CFG_DBGBUS_CLRC },
	{ A6XX_CX_DBGC_CFG_DBGBUS_LOADIVT },
	{ A6XX_CX_DBGC_VBIF_DBG_CNTL },
	{ A6XX_CX_DBGC_DBG_LO_HI_GPIO },
	{ A6XX_CX_DBGC_EXT_TRACE_BUS_CNTL },
	{ A6XX_CX_DBGC_READ_AHB_THROUGH_DBG },
	{ A6XX_CX_DBGC_CFG_DBGBUS_TRACE_BUF1 },
	{ A6XX_CX_DBGC_CFG_DBGBUS_TRACE_BUF2 },
	{ A6XX_CX_DBGC_EVT_CFG },
	{ A6XX_CX_DBGC_EVT_INTF_SEL_0 },
	{ A6XX_CX_DBGC_EVT_INTF_SEL_1 },
	{ A6XX_CX_DBGC_PERF_ATB_CFG },
	{ A6XX_CX_DBGC_PERF_ATB_COUNTER_SEL_0 },
	{ A6XX_CX_DBGC_PERF_ATB_COUNTER_SEL_1 },
	{ A6XX_CX_DBGC_PERF_ATB_COUNTER_SEL_2 },
	{ A6XX_CX_DBGC_PERF_ATB_COUNTER_SEL_3 },
	{ A6XX_CX_DBGC_PERF_ATB_TRIG_INTF_SEL_0 },
	{ A6XX_CX_DBGC_PERF_ATB_TRIG_INTF_SEL_1 },
	{ A6XX_CX_DBGC_PERF_ATB_DRAIN_CMD },
	{ A6XX_CX_DBGC_ECO_CNTL },
	{ A6XX_CX_DBGC_AHB_DBG_CNTL },
};

static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_a, &a6xx_coresight_regs[0]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_b, &a6xx_coresight_regs[1]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_c, &a6xx_coresight_regs[2]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_d, &a6xx_coresight_regs[3]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_cntlt, &a6xx_coresight_regs[4]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_cntlm, &a6xx_coresight_regs[5]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_opl, &a6xx_coresight_regs[6]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ope, &a6xx_coresight_regs[7]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_0, &a6xx_coresight_regs[8]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_1, &a6xx_coresight_regs[9]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_2, &a6xx_coresight_regs[10]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_3, &a6xx_coresight_regs[11]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_0, &a6xx_coresight_regs[12]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_1, &a6xx_coresight_regs[13]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_2, &a6xx_coresight_regs[14]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_3, &a6xx_coresight_regs[15]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_bytel_0, &a6xx_coresight_regs[16]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_bytel_1, &a6xx_coresight_regs[17]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_0, &a6xx_coresight_regs[18]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_1, &a6xx_coresight_regs[19]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_2, &a6xx_coresight_regs[20]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_3, &a6xx_coresight_regs[21]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_0, &a6xx_coresight_regs[22]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_1, &a6xx_coresight_regs[23]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_2, &a6xx_coresight_regs[24]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_3, &a6xx_coresight_regs[25]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_nibblee, &a6xx_coresight_regs[26]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ptrc0, &a6xx_coresight_regs[27]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ptrc1, &a6xx_coresight_regs[28]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_loadreg, &a6xx_coresight_regs[29]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_idx, &a6xx_coresight_regs[30]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_clrc, &a6xx_coresight_regs[31]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_loadivt, &a6xx_coresight_regs[32]);
static ADRENO_CORESIGHT_ATTR(vbif_dbg_cntl, &a6xx_coresight_regs[33]);
static ADRENO_CORESIGHT_ATTR(dbg_lo_hi_gpio, &a6xx_coresight_regs[34]);
static ADRENO_CORESIGHT_ATTR(ext_trace_bus_cntl, &a6xx_coresight_regs[35]);
static ADRENO_CORESIGHT_ATTR(read_ahb_through_dbg, &a6xx_coresight_regs[36]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_buf1, &a6xx_coresight_regs[37]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_buf2, &a6xx_coresight_regs[38]);
static ADRENO_CORESIGHT_ATTR(evt_cfg, &a6xx_coresight_regs[39]);
static ADRENO_CORESIGHT_ATTR(evt_intf_sel_0, &a6xx_coresight_regs[40]);
static ADRENO_CORESIGHT_ATTR(evt_intf_sel_1, &a6xx_coresight_regs[41]);
static ADRENO_CORESIGHT_ATTR(perf_atb_cfg, &a6xx_coresight_regs[42]);
static ADRENO_CORESIGHT_ATTR(perf_atb_counter_sel_0, &a6xx_coresight_regs[43]);
static ADRENO_CORESIGHT_ATTR(perf_atb_counter_sel_1, &a6xx_coresight_regs[44]);
static ADRENO_CORESIGHT_ATTR(perf_atb_counter_sel_2, &a6xx_coresight_regs[45]);
static ADRENO_CORESIGHT_ATTR(perf_atb_counter_sel_3, &a6xx_coresight_regs[46]);
static ADRENO_CORESIGHT_ATTR(perf_atb_trig_intf_sel_0,
				&a6xx_coresight_regs[47]);
static ADRENO_CORESIGHT_ATTR(perf_atb_trig_intf_sel_1,
				&a6xx_coresight_regs[48]);
static ADRENO_CORESIGHT_ATTR(perf_atb_drain_cmd, &a6xx_coresight_regs[49]);
static ADRENO_CORESIGHT_ATTR(eco_cntl, &a6xx_coresight_regs[50]);
static ADRENO_CORESIGHT_ATTR(ahb_dbg_cntl, &a6xx_coresight_regs[51]);

/*CX debug registers*/
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_sel_a,
				&a6xx_coresight_regs_cx[0]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_sel_b,
				&a6xx_coresight_regs_cx[1]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_sel_c,
				&a6xx_coresight_regs_cx[2]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_sel_d,
				&a6xx_coresight_regs_cx[3]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_cntlt,
				&a6xx_coresight_regs_cx[4]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_cntlm,
				&a6xx_coresight_regs_cx[5]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_opl,
				&a6xx_coresight_regs_cx[6]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ope,
				&a6xx_coresight_regs_cx[7]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivtl_0,
				&a6xx_coresight_regs_cx[8]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivtl_1,
				&a6xx_coresight_regs_cx[9]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivtl_2,
				&a6xx_coresight_regs_cx[10]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivtl_3,
				&a6xx_coresight_regs_cx[11]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maskl_0,
				&a6xx_coresight_regs_cx[12]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maskl_1,
				&a6xx_coresight_regs_cx[13]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maskl_2,
				&a6xx_coresight_regs_cx[14]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maskl_3,
				&a6xx_coresight_regs_cx[15]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_bytel_0,
				&a6xx_coresight_regs_cx[16]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_bytel_1,
				&a6xx_coresight_regs_cx[17]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivte_0,
				&a6xx_coresight_regs_cx[18]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivte_1,
				&a6xx_coresight_regs_cx[19]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivte_2,
				&a6xx_coresight_regs_cx[20]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ivte_3,
				&a6xx_coresight_regs_cx[21]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maske_0,
				&a6xx_coresight_regs_cx[22]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maske_1,
				&a6xx_coresight_regs_cx[23]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maske_2,
				&a6xx_coresight_regs_cx[24]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_maske_3,
				&a6xx_coresight_regs_cx[25]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_nibblee,
				&a6xx_coresight_regs_cx[26]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ptrc0,
				&a6xx_coresight_regs_cx[27]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_ptrc1,
				&a6xx_coresight_regs_cx[28]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_loadreg,
				&a6xx_coresight_regs_cx[29]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_idx,
				&a6xx_coresight_regs_cx[30]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_clrc,
				&a6xx_coresight_regs_cx[31]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_loadivt,
				&a6xx_coresight_regs_cx[32]);
static ADRENO_CORESIGHT_ATTR(cx_vbif_dbg_cntl,
				&a6xx_coresight_regs_cx[33]);
static ADRENO_CORESIGHT_ATTR(cx_dbg_lo_hi_gpio,
				&a6xx_coresight_regs_cx[34]);
static ADRENO_CORESIGHT_ATTR(cx_ext_trace_bus_cntl,
				&a6xx_coresight_regs_cx[35]);
static ADRENO_CORESIGHT_ATTR(cx_read_ahb_through_dbg,
				&a6xx_coresight_regs_cx[36]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_trace_buf1,
				&a6xx_coresight_regs_cx[37]);
static ADRENO_CORESIGHT_ATTR(cx_cfg_dbgbus_trace_buf2,
				&a6xx_coresight_regs_cx[38]);
static ADRENO_CORESIGHT_ATTR(cx_evt_cfg,
				&a6xx_coresight_regs_cx[39]);
static ADRENO_CORESIGHT_ATTR(cx_evt_intf_sel_0,
				&a6xx_coresight_regs_cx[40]);
static ADRENO_CORESIGHT_ATTR(cx_evt_intf_sel_1,
				&a6xx_coresight_regs_cx[41]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_cfg,
				&a6xx_coresight_regs_cx[42]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_counter_sel_0,
				&a6xx_coresight_regs_cx[43]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_counter_sel_1,
				&a6xx_coresight_regs_cx[44]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_counter_sel_2,
				&a6xx_coresight_regs_cx[45]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_counter_sel_3,
				&a6xx_coresight_regs_cx[46]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_trig_intf_sel_0,
				&a6xx_coresight_regs_cx[47]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_trig_intf_sel_1,
				&a6xx_coresight_regs_cx[48]);
static ADRENO_CORESIGHT_ATTR(cx_perf_atb_drain_cmd,
				&a6xx_coresight_regs_cx[49]);
static ADRENO_CORESIGHT_ATTR(cx_eco_cntl,
				&a6xx_coresight_regs_cx[50]);
static ADRENO_CORESIGHT_ATTR(cx_ahb_dbg_cntl,
				&a6xx_coresight_regs_cx[51]);

static struct attribute *a6xx_coresight_attrs[] = {
	&coresight_attr_cfg_dbgbus_sel_a.attr.attr,
	&coresight_attr_cfg_dbgbus_sel_b.attr.attr,
	&coresight_attr_cfg_dbgbus_sel_c.attr.attr,
	&coresight_attr_cfg_dbgbus_sel_d.attr.attr,
	&coresight_attr_cfg_dbgbus_cntlt.attr.attr,
	&coresight_attr_cfg_dbgbus_cntlm.attr.attr,
	&coresight_attr_cfg_dbgbus_opl.attr.attr,
	&coresight_attr_cfg_dbgbus_ope.attr.attr,
	&coresight_attr_cfg_dbgbus_ivtl_0.attr.attr,
	&coresight_attr_cfg_dbgbus_ivtl_1.attr.attr,
	&coresight_attr_cfg_dbgbus_ivtl_2.attr.attr,
	&coresight_attr_cfg_dbgbus_ivtl_3.attr.attr,
	&coresight_attr_cfg_dbgbus_maskl_0.attr.attr,
	&coresight_attr_cfg_dbgbus_maskl_1.attr.attr,
	&coresight_attr_cfg_dbgbus_maskl_2.attr.attr,
	&coresight_attr_cfg_dbgbus_maskl_3.attr.attr,
	&coresight_attr_cfg_dbgbus_bytel_0.attr.attr,
	&coresight_attr_cfg_dbgbus_bytel_1.attr.attr,
	&coresight_attr_cfg_dbgbus_ivte_0.attr.attr,
	&coresight_attr_cfg_dbgbus_ivte_1.attr.attr,
	&coresight_attr_cfg_dbgbus_ivte_2.attr.attr,
	&coresight_attr_cfg_dbgbus_ivte_3.attr.attr,
	&coresight_attr_cfg_dbgbus_maske_0.attr.attr,
	&coresight_attr_cfg_dbgbus_maske_1.attr.attr,
	&coresight_attr_cfg_dbgbus_maske_2.attr.attr,
	&coresight_attr_cfg_dbgbus_maske_3.attr.attr,
	&coresight_attr_cfg_dbgbus_nibblee.attr.attr,
	&coresight_attr_cfg_dbgbus_ptrc0.attr.attr,
	&coresight_attr_cfg_dbgbus_ptrc1.attr.attr,
	&coresight_attr_cfg_dbgbus_loadreg.attr.attr,
	&coresight_attr_cfg_dbgbus_idx.attr.attr,
	&coresight_attr_cfg_dbgbus_clrc.attr.attr,
	&coresight_attr_cfg_dbgbus_loadivt.attr.attr,
	&coresight_attr_vbif_dbg_cntl.attr.attr,
	&coresight_attr_dbg_lo_hi_gpio.attr.attr,
	&coresight_attr_ext_trace_bus_cntl.attr.attr,
	&coresight_attr_read_ahb_through_dbg.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_buf1.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_buf2.attr.attr,
	&coresight_attr_evt_cfg.attr.attr,
	&coresight_attr_evt_intf_sel_0.attr.attr,
	&coresight_attr_evt_intf_sel_1.attr.attr,
	&coresight_attr_perf_atb_cfg.attr.attr,
	&coresight_attr_perf_atb_counter_sel_0.attr.attr,
	&coresight_attr_perf_atb_counter_sel_1.attr.attr,
	&coresight_attr_perf_atb_counter_sel_2.attr.attr,
	&coresight_attr_perf_atb_counter_sel_3.attr.attr,
	&coresight_attr_perf_atb_trig_intf_sel_0.attr.attr,
	&coresight_attr_perf_atb_trig_intf_sel_1.attr.attr,
	&coresight_attr_perf_atb_drain_cmd.attr.attr,
	&coresight_attr_eco_cntl.attr.attr,
	&coresight_attr_ahb_dbg_cntl.attr.attr,
	NULL,
};

/*cx*/
static struct attribute *a6xx_coresight_attrs_cx[] = {
	&coresight_attr_cx_cfg_dbgbus_sel_a.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_sel_b.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_sel_c.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_sel_d.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_cntlt.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_cntlm.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_opl.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ope.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivtl_0.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivtl_1.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivtl_2.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivtl_3.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maskl_0.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maskl_1.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maskl_2.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maskl_3.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_bytel_0.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_bytel_1.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivte_0.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivte_1.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivte_2.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ivte_3.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maske_0.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maske_1.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maske_2.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_maske_3.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_nibblee.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ptrc0.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_ptrc1.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_loadreg.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_idx.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_clrc.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_loadivt.attr.attr,
	&coresight_attr_cx_vbif_dbg_cntl.attr.attr,
	&coresight_attr_cx_dbg_lo_hi_gpio.attr.attr,
	&coresight_attr_cx_ext_trace_bus_cntl.attr.attr,
	&coresight_attr_cx_read_ahb_through_dbg.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_trace_buf1.attr.attr,
	&coresight_attr_cx_cfg_dbgbus_trace_buf2.attr.attr,
	&coresight_attr_cx_evt_cfg.attr.attr,
	&coresight_attr_cx_evt_intf_sel_0.attr.attr,
	&coresight_attr_cx_evt_intf_sel_1.attr.attr,
	&coresight_attr_cx_perf_atb_cfg.attr.attr,
	&coresight_attr_cx_perf_atb_counter_sel_0.attr.attr,
	&coresight_attr_cx_perf_atb_counter_sel_1.attr.attr,
	&coresight_attr_cx_perf_atb_counter_sel_2.attr.attr,
	&coresight_attr_cx_perf_atb_counter_sel_3.attr.attr,
	&coresight_attr_cx_perf_atb_trig_intf_sel_0.attr.attr,
	&coresight_attr_cx_perf_atb_trig_intf_sel_1.attr.attr,
	&coresight_attr_cx_perf_atb_drain_cmd.attr.attr,
	&coresight_attr_cx_eco_cntl.attr.attr,
	&coresight_attr_cx_ahb_dbg_cntl.attr.attr,
	NULL,
};

static const struct attribute_group a6xx_coresight_group = {
	.attrs = a6xx_coresight_attrs,
};

static const struct attribute_group *a6xx_coresight_groups[] = {
	&a6xx_coresight_group,
	NULL,
};

static const struct attribute_group a6xx_coresight_group_cx = {
	.attrs = a6xx_coresight_attrs_cx,
};

static const struct attribute_group *a6xx_coresight_groups_cx[] = {
	&a6xx_coresight_group_cx,
	NULL,
};

static const struct adreno_coresight a6xx_coresight = {
	.registers = a6xx_coresight_regs,
	.count = ARRAY_SIZE(a6xx_coresight_regs),
	.groups = a6xx_coresight_groups,
};

static const struct adreno_coresight a6xx_coresight_cx = {
	.registers = a6xx_coresight_regs_cx,
	.count = ARRAY_SIZE(a6xx_coresight_regs_cx),
	.groups = a6xx_coresight_groups_cx,
};

void a6xx_coresight_init(struct adreno_device *adreno_dev)
{
	adreno_coresight_add_device(adreno_dev, "coresight-gfx",
		&a6xx_coresight, &adreno_dev->gx_coresight);

	adreno_coresight_add_device(adreno_dev, "coresight-gfx-cx",
		&a6xx_coresight_cx, &adreno_dev->cx_coresight);
}
