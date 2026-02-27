// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "adreno.h"
#include "adreno_a5xx.h"
#include "adreno_coresight.h"

static struct adreno_coresight_register a5xx_coresight_registers[] = {
	{ A5XX_RBBM_CFG_DBGBUS_SEL_A },
	{ A5XX_RBBM_CFG_DBGBUS_SEL_B },
	{ A5XX_RBBM_CFG_DBGBUS_SEL_C },
	{ A5XX_RBBM_CFG_DBGBUS_SEL_D },
	{ A5XX_RBBM_CFG_DBGBUS_CNTLT },
	{ A5XX_RBBM_CFG_DBGBUS_CNTLM },
	{ A5XX_RBBM_CFG_DBGBUS_OPL },
	{ A5XX_RBBM_CFG_DBGBUS_OPE },
	{ A5XX_RBBM_CFG_DBGBUS_IVTL_0 },
	{ A5XX_RBBM_CFG_DBGBUS_IVTL_1 },
	{ A5XX_RBBM_CFG_DBGBUS_IVTL_2 },
	{ A5XX_RBBM_CFG_DBGBUS_IVTL_3 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKL_0 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKL_1 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKL_2 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKL_3 },
	{ A5XX_RBBM_CFG_DBGBUS_BYTEL_0 },
	{ A5XX_RBBM_CFG_DBGBUS_BYTEL_1 },
	{ A5XX_RBBM_CFG_DBGBUS_IVTE_0 },
	{ A5XX_RBBM_CFG_DBGBUS_IVTE_1 },
	{ A5XX_RBBM_CFG_DBGBUS_IVTE_2 },
	{ A5XX_RBBM_CFG_DBGBUS_IVTE_3 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKE_0 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKE_1 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKE_2 },
	{ A5XX_RBBM_CFG_DBGBUS_MASKE_3 },
	{ A5XX_RBBM_CFG_DBGBUS_NIBBLEE },
	{ A5XX_RBBM_CFG_DBGBUS_PTRC0 },
	{ A5XX_RBBM_CFG_DBGBUS_PTRC1 },
	{ A5XX_RBBM_CFG_DBGBUS_LOADREG },
	{ A5XX_RBBM_CFG_DBGBUS_IDX },
	{ A5XX_RBBM_CFG_DBGBUS_CLRC },
	{ A5XX_RBBM_CFG_DBGBUS_LOADIVT },
	{ A5XX_RBBM_CFG_DBGBUS_EVENT_LOGIC },
	{ A5XX_RBBM_CFG_DBGBUS_OVER },
	{ A5XX_RBBM_CFG_DBGBUS_COUNT0 },
	{ A5XX_RBBM_CFG_DBGBUS_COUNT1 },
	{ A5XX_RBBM_CFG_DBGBUS_COUNT2 },
	{ A5XX_RBBM_CFG_DBGBUS_COUNT3 },
	{ A5XX_RBBM_CFG_DBGBUS_COUNT4 },
	{ A5XX_RBBM_CFG_DBGBUS_COUNT5 },
	{ A5XX_RBBM_CFG_DBGBUS_TRACE_ADDR },
	{ A5XX_RBBM_CFG_DBGBUS_TRACE_BUF0 },
	{ A5XX_RBBM_CFG_DBGBUS_TRACE_BUF1 },
	{ A5XX_RBBM_CFG_DBGBUS_TRACE_BUF2 },
	{ A5XX_RBBM_CFG_DBGBUS_TRACE_BUF3 },
	{ A5XX_RBBM_CFG_DBGBUS_TRACE_BUF4 },
	{ A5XX_RBBM_CFG_DBGBUS_MISR0 },
	{ A5XX_RBBM_CFG_DBGBUS_MISR1 },
	{ A5XX_RBBM_AHB_DBG_CNTL },
	{ A5XX_RBBM_READ_AHB_THROUGH_DBG },
	{ A5XX_RBBM_DBG_LO_HI_GPIO },
	{ A5XX_RBBM_EXT_TRACE_BUS_CNTL },
	{ A5XX_RBBM_EXT_VBIF_DBG_CNTL },
};

static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_a, &a5xx_coresight_registers[0]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_b, &a5xx_coresight_registers[1]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_c, &a5xx_coresight_registers[2]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_sel_d, &a5xx_coresight_registers[3]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_cntlt, &a5xx_coresight_registers[4]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_cntlm, &a5xx_coresight_registers[5]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_opl, &a5xx_coresight_registers[6]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ope, &a5xx_coresight_registers[7]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_0, &a5xx_coresight_registers[8]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_1, &a5xx_coresight_registers[9]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_2, &a5xx_coresight_registers[10]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivtl_3, &a5xx_coresight_registers[11]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_0, &a5xx_coresight_registers[12]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_1, &a5xx_coresight_registers[13]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_2, &a5xx_coresight_registers[14]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maskl_3, &a5xx_coresight_registers[15]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_bytel_0, &a5xx_coresight_registers[16]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_bytel_1, &a5xx_coresight_registers[17]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_0, &a5xx_coresight_registers[18]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_1, &a5xx_coresight_registers[19]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_2, &a5xx_coresight_registers[20]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ivte_3, &a5xx_coresight_registers[21]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_0, &a5xx_coresight_registers[22]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_1, &a5xx_coresight_registers[23]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_2, &a5xx_coresight_registers[24]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_maske_3, &a5xx_coresight_registers[25]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_nibblee, &a5xx_coresight_registers[26]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ptrc0, &a5xx_coresight_registers[27]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_ptrc1, &a5xx_coresight_registers[28]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_loadreg, &a5xx_coresight_registers[29]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_idx, &a5xx_coresight_registers[30]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_clrc, &a5xx_coresight_registers[31]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_loadivt, &a5xx_coresight_registers[32]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_event_logic,
				&a5xx_coresight_registers[33]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_over, &a5xx_coresight_registers[34]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_count0, &a5xx_coresight_registers[35]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_count1, &a5xx_coresight_registers[36]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_count2, &a5xx_coresight_registers[37]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_count3, &a5xx_coresight_registers[38]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_count4, &a5xx_coresight_registers[39]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_count5, &a5xx_coresight_registers[40]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_addr,
				&a5xx_coresight_registers[41]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_buf0,
				&a5xx_coresight_registers[42]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_buf1,
				&a5xx_coresight_registers[43]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_buf2,
				&a5xx_coresight_registers[44]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_buf3,
				&a5xx_coresight_registers[45]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_trace_buf4,
				&a5xx_coresight_registers[46]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_misr0, &a5xx_coresight_registers[47]);
static ADRENO_CORESIGHT_ATTR(cfg_dbgbus_misr1, &a5xx_coresight_registers[48]);
static ADRENO_CORESIGHT_ATTR(ahb_dbg_cntl, &a5xx_coresight_registers[49]);
static ADRENO_CORESIGHT_ATTR(read_ahb_through_dbg,
				&a5xx_coresight_registers[50]);
static ADRENO_CORESIGHT_ATTR(dbg_lo_hi_gpio, &a5xx_coresight_registers[51]);
static ADRENO_CORESIGHT_ATTR(ext_trace_bus_cntl, &a5xx_coresight_registers[52]);
static ADRENO_CORESIGHT_ATTR(ext_vbif_dbg_cntl, &a5xx_coresight_registers[53]);

static struct attribute *a5xx_coresight_attrs[] = {
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
	&coresight_attr_cfg_dbgbus_event_logic.attr.attr,
	&coresight_attr_cfg_dbgbus_over.attr.attr,
	&coresight_attr_cfg_dbgbus_count0.attr.attr,
	&coresight_attr_cfg_dbgbus_count1.attr.attr,
	&coresight_attr_cfg_dbgbus_count2.attr.attr,
	&coresight_attr_cfg_dbgbus_count3.attr.attr,
	&coresight_attr_cfg_dbgbus_count4.attr.attr,
	&coresight_attr_cfg_dbgbus_count5.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_addr.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_buf0.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_buf1.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_buf2.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_buf3.attr.attr,
	&coresight_attr_cfg_dbgbus_trace_buf4.attr.attr,
	&coresight_attr_cfg_dbgbus_misr0.attr.attr,
	&coresight_attr_cfg_dbgbus_misr1.attr.attr,
	&coresight_attr_ahb_dbg_cntl.attr.attr,
	&coresight_attr_read_ahb_through_dbg.attr.attr,
	&coresight_attr_dbg_lo_hi_gpio.attr.attr,
	&coresight_attr_ext_trace_bus_cntl.attr.attr,
	&coresight_attr_ext_vbif_dbg_cntl.attr.attr,
	NULL,
};

static const struct attribute_group a5xx_coresight_group = {
	.attrs = a5xx_coresight_attrs,
};

static const struct attribute_group *a5xx_coresight_groups[] = {
	&a5xx_coresight_group,
	NULL,
};

static const struct adreno_coresight a5xx_coresight = {
	.registers = a5xx_coresight_registers,
	.count = ARRAY_SIZE(a5xx_coresight_registers),
	.groups = a5xx_coresight_groups,
};

void a5xx_coresight_init(struct adreno_device *adreno_dev)
{
	adreno_coresight_add_device(adreno_dev, "coresight-gfx",
		&a5xx_coresight, &adreno_dev->gx_coresight);
}
