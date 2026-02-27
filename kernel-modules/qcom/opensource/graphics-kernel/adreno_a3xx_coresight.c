// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*/

#include "adreno.h"
#include "adreno_a3xx.h"
#include "adreno_coresight.h"

static struct adreno_coresight_register a3xx_coresight_registers[] = {
	{ A3XX_RBBM_DEBUG_BUS_CTL, 0x0001093F },
	{ A3XX_RBBM_EXT_TRACE_STOP_CNT, 0x00017fff },
	{ A3XX_RBBM_EXT_TRACE_START_CNT, 0x0001000f },
	{ A3XX_RBBM_EXT_TRACE_PERIOD_CNT, 0x0001ffff },
	{ A3XX_RBBM_EXT_TRACE_CMD, 0x00000001 },
	{ A3XX_RBBM_EXT_TRACE_BUS_CTL, 0x89100010 },
	{ A3XX_RBBM_DEBUG_BUS_STB_CTL0, 0x00000000 },
	{ A3XX_RBBM_DEBUG_BUS_STB_CTL1, 0xFFFFFFFE },
	{ A3XX_RBBM_INT_TRACE_BUS_CTL, 0x00201111 },
};

static ADRENO_CORESIGHT_ATTR(config_debug_bus,
	&a3xx_coresight_registers[0]);
static ADRENO_CORESIGHT_ATTR(config_trace_stop_cnt,
	&a3xx_coresight_registers[1]);
static ADRENO_CORESIGHT_ATTR(config_trace_start_cnt,
	&a3xx_coresight_registers[2]);
static ADRENO_CORESIGHT_ATTR(config_trace_period_cnt,
	&a3xx_coresight_registers[3]);
static ADRENO_CORESIGHT_ATTR(config_trace_cmd,
	&a3xx_coresight_registers[4]);
static ADRENO_CORESIGHT_ATTR(config_trace_bus_ctl,
	&a3xx_coresight_registers[5]);

static struct attribute *a3xx_coresight_attrs[] = {
	&coresight_attr_config_debug_bus.attr.attr,
	&coresight_attr_config_trace_start_cnt.attr.attr,
	&coresight_attr_config_trace_stop_cnt.attr.attr,
	&coresight_attr_config_trace_period_cnt.attr.attr,
	&coresight_attr_config_trace_cmd.attr.attr,
	&coresight_attr_config_trace_bus_ctl.attr.attr,
	NULL,
};

static const struct attribute_group a3xx_coresight_group = {
	.attrs = a3xx_coresight_attrs,
};

static const struct attribute_group *a3xx_coresight_groups[] = {
	&a3xx_coresight_group,
	NULL,
};

static const struct adreno_coresight a3xx_coresight = {
	.registers = a3xx_coresight_registers,
	.count = ARRAY_SIZE(a3xx_coresight_registers),
	.groups = a3xx_coresight_groups,
};

void a3xx_coresight_init(struct adreno_device *adreno_dev)
{
	adreno_coresight_add_device(adreno_dev, "coresight-gfx",
		&a3xx_coresight, &adreno_dev->gx_coresight);
}
