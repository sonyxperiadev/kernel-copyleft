/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CPASTOP_V980_100_H_
#define _CPASTOP_V980_100_H_

static struct cam_camnoc_irq_sbm cam_cpas_v980_100_irq_sbm_rt = {
	.sbm_enable = {
		.access_type = CAM_REG_TYPE_READ_WRITE,
		.enable = true,
		.offset = 0x240,  /* CAM_NOC_RT_SBM_FAULTINEN0_LOW */
		.value = 0x01 |   /* RT_SBM_FAULTINEN0_LOW_PORT0_MASK - Slave error IRQ */
			0x02,    /* RT_SBM_FAULTINEN0_LOW_PORT1_MASK - TFE UBWC Encoder Error IRQ */
	},
	.sbm_status = {
		.access_type = CAM_REG_TYPE_READ,
		.enable = true,
		.offset = 0x248, /* CAM_NOC_RT_SBM_FAULTINSTATUS0_LOW */
	},
	.sbm_clear = {
		.access_type = CAM_REG_TYPE_WRITE,
		.enable = true,
		.offset = 0x280, /* CAM_NOC_RT_SBM_FLAGOUTCLR0_LOW */
		.value = 0x7,
	}
};

static struct cam_camnoc_irq_sbm cam_cpas_v980_100_irq_sbm_nrt = {
	.sbm_enable = {
		.access_type = CAM_REG_TYPE_READ_WRITE,
		.enable = true,
		.offset = 0x240,  /* CAM_NOC_NRT_SBM_FAULTINEN0_LOW */
		.value = 0x01 |   /* NRT_SBM_FAULTINEN0_LOW_PORT0_MASK - Slave Error */
			0x02 |    /* NRT_SBM_FAULTINEN0_LOW_PORT1_MASK - IPE WR UBWC En */
			0x04 |    /* NRT_SBM_FAULTINEN0_LOW_PORT2_MASK - OFE WR UBWC En */
			0x08 |    /* NRT_SBM_FAULTINEN0_LOW_PORT3_MASK - OFE RD UBWC De */
			0x10 |    /* NRT_SBM_FAULTINEN0_LOW_PORT4_MASK - IPE RD UBWC En */
			0x20,     /* NRT_SBM_FAULTINEN0_LOW_PORT5_MASK - IPE RD UBWC En */
	},
	.sbm_status = {
		.access_type = CAM_REG_TYPE_READ,
		.enable = true,
		.offset = 0x248, /* CAM_NOC_NRT_SBM_FAULTINSTATUS0_LOW */
	},
	.sbm_clear = {
		.access_type = CAM_REG_TYPE_WRITE,
		.enable = true,
		.offset = 0x280, /* CAM_NOC_NRT_SBM_FLAGOUTCLR0_LOW */
		.value = 0x7,
	}
};

static struct cam_camnoc_irq_err
	cam_cpas_v980_100_irq_err_rt[] = {
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_SLAVE_ERROR,
		.enable = true,
		.sbm_port = 0x1, /* RT_SBM_FAULTINSTATUS0_LOW_PORT0_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x8, /* CAM_NOC_RT_ERL_MAINCTL_LOW */
			.value = 0x1,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x10, /* CAM_NOC_RT_ERL_ERRVLD_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x18, /* CAM_NOC_RT_ERL_ERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_TFE_UBWC_ENCODE_ERROR,
		.enable = true,
		.sbm_port = 0x2, /* RT_SBM_FAULTINSTATUS0_LOW_PORT1_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x47A0, /* TFE_UBWC : RT_0_NIU_ENCERREN_LOW */
			.value = 0xF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x4790, /* IFE_UBWC : RT_0_NIU_ENCERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x4798, /* IFE_UBWC : RT_0_NIU_ENCERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_RESERVED1,
		.enable = false,
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_RESERVED2,
		.enable = false,
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_CAMNOC_TEST,
		.enable = false,
		.sbm_port = 0x20, /* RT_SBM_FAULTINSTATUS0_LOW_PORT5_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x288, /* RT_CAM_NOC_RT_SBM_FLAGOUTSET0_LOW */
			.value = 0x1,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x290, /* CAM_NOC_RT_SBM_FLAGOUTSTATUS0_LOW */
		},
		.err_clear = {
			.enable = false, /* CAM_NOC_RT_SBM_FLAGOUTCLR0_LOW */
		},
	},
};

static struct cam_camnoc_irq_err
	cam_cpas_v980_100_irq_err_nrt[] = {
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_SLAVE_ERROR,
		.enable = true,
		.sbm_port = 0x1, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT0_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x8, /* CAM_NOC_NRT_ERL_MAINCTL_LOW */
			.value = 0x1,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x10, /* CAM_NOC_NRT_ERL_ERRVLD_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x18, /* CAM_NOC_NRT_ERL_ERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_OFE_UBWC_WRITE_ENCODE_ERROR,
		.enable = true,
		.sbm_port = 0x4, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT2_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x63A0, /* OFE_WR : NRT_3_NIU_ENCERREN_LOW */
			.value = 0xF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x6390, /* OFE_WR : NRT_3_NIU_ENCERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x6398, /* OFE_WR : NRT_3_NIU_ENCERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_OFE_UBWC_READ_DECODE_ERROR,
		.enable = true,
		.sbm_port = 0x8, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT3_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x6520, /* OFE_RD : NRT_4_NIU_DECERREN_LOW */
			.value = 0xFF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x6510, /* OFE_RD : NRT_4_NIU_DECERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x6518, /* OFE_RD : NRT_4_NIU_DECERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_IPE_UBWC_ENCODE_ERROR,
		.enable = true,
		.sbm_port = 0x2, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT1_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x59A0, /* IPE_WR : NRT_1_NIU_ENCERREN_LOW */
			.value = 0xF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x5990, /* IPE_WR : NRT_1_NIU_ENCERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x5998, /* IPE_WR : NRT_1_NIU_ENCERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_IPE0_UBWC_DECODE_ERROR,
		.enable = true,
		.sbm_port = 0x20, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT5_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x6D20, /* IPE_0_RD : NRT_8_NIU_DECERREN_LOW */
			.value = 0xFF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x6D10, /* IPE_0_RD : NRT_8_NIU_DECERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x6D18, /* IPE_0_RD : NRT_8_NIU_DECERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_IPE1_UBWC_DECODE_ERROR,
		.enable = true,
		.sbm_port = 0x10, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT4_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x6B20, /* IPE_1_RD : NRT_7_NIU_DECERREN_LOW */
			.value = 0xFF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x6B10, /* IPE_1_RD : NRT_7_NIU_DECERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x6B18, /* IPE_1_RD : NRT_7_NIU_DECERRCLR_LOW */
			.value = 0xFF,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_AHB_TIMEOUT,
		.enable = false,
		.sbm_port = 0x40, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT6_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x288, /* CAM_NOC_NRT_SBM_FLAGOUTSET0_LOW */
			.value = 0xE,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x290, /* CAM_NOC_NRT_SBM_FLAGOUTSTATUS0_LOW */
		},
		.err_clear = {
			.enable = false, /* CAM_NOC_NRT_SBM_FLAGOUTCLR0_LOW */
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_RESERVED1,
		.enable = false,
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_RESERVED2,
		.enable = false,
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_CAMNOC_TEST,
		.enable = false,
		.sbm_port = 0x400, /* NRT_SBM_FAULTINSTATUS0_LOW_PORT10_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x288, /* CAM_NOC_NRT_SBM_FLAGOUTSET0_LOW */
			.value = 0x2,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x290, /* CAM_NOC_NRT_SBM_FLAGOUTSTATUS0_LOW */
		},
		.err_clear = {
			.enable = false, /* CAM_NOC_NRT_SBM_FLAGOUTCLR0_LOW */
		},
	},
};

static struct cam_camnoc_specific
	cam_cpas_v980_100_camnoc_specific_rt[] = {
	/* RT ports */
	{
		.port_name = "RT0-TFE_LINEAR_RDI",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			/*
			 * Do not explicitly set ubwc config register.
			 * Power on default values are taking care of required
			 * register settings.
			 */
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_0_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT1-TFE_FD_PDAF_IFE_LITE",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			/*
			 * Do not explicitly set ubwc config register.
			 * Power on default values are taking care of required
			 * register settings.
			 */
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* TFE_FD_PDAF_IFE_LITE : NOC_RT_1_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT2-TFE_STATS",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			/*
			 * Do not explicitly set ubwc config register.
			 * Power on default values are taking care of required
			 * register settings.
			 */
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* TFE_STATS : NOC_RT_2_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT3-TFE_IFE_LITE_CDM",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			/*
			 * Do not explicitly set ubwc config register.
			 * Power on default values are taking care of required
			 * register settings.
			 */
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* TFE_IFE_LITE_CDM : NOC_RT_3_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT4-TFE_LINEAR_RDI",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			/*
			 * Do not explicitly set ubwc config register.
			 * Power on default values are taking care of required
			 * register settings.
			 */
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* TFE_LINEAR_RDI : NOC_RT_4_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
};

static struct cam_camnoc_specific
	cam_cpas_v980_100_camnoc_specific_nrt[] = {
	/* NRT ports */
	{
		.port_name = "NRT0-IPE_WR_1",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_0_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT1-IPE_WR_0",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_0 : NOC_NRT_1_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT2-OFE_WR_1-CRE_WR",
		.enable = false,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_BPS_WR_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_1-CRE_WR : NOC_NRT_2_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT3-OFE_WR_0",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_NIU_URGENCY_LOW */
			.value = 0x3,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_QOSGEN_MAINCTL */
			.value = 0x2,
		},
		.qosgen_shaping_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_WR_0 : NOC_NRT_3_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT4-OFE_RD",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_NIU_DECCTL_LOW */
			.value = 1,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* OFE_RD : NOC_NRT_4_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT5-CRE_RD",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CRE_RD : NOC_NRT_5_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT6-JPEG_RD_WR",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* JPEG_RD_WR : NOC_NRT_6_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT7-IPE_WR_1",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* IPE_WR_1 : NOC_NRT_7_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT8-IPE_RD_0",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* IPE_RD_0 : NOC_NRT_8_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT9-CDM_IPE_OFE",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_NIU_URGENCY_LOW */
			.value = 0x0,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_NIU_SAFELUT_LOW */
			.value = 0x0,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x0, /* CDM_IPE_OFE : NOC_NRT_9_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "ICP_RD_WR",
		.enable = false,
		.dynattr_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* ICP_RD_WR : NOC_XM_ICP_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* ICP_RD_WR : NOC_XM_ICP_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* ICP_RD_WR : NOC_XM_ICP_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x0, /* ICP_RD_WR : NOC_XM_ICP_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
};

static struct cam_camnoc_err_logger_info cam980_cpas100_err_logger_offsets = {
	.mainctrl     =  0x08, /* NOC_ERL_MAINCTL_LOW */
	.errvld       =  0x10, /* NOC_ERL_ERRVLD_LOW */
	.errlog0_low  =  0x20, /* NOC_ERL_ERRLOG0_LOW */
	.errlog0_high =  0x24, /* NOC_ERL_ERRLOG0_HIGH */
	.errlog1_low  =  0x28, /* NOC_ERL_ERRLOG1_LOW */
	.errlog1_high =  0x2C, /* NOC_ERL_ERRLOG1_HIGH */
	.errlog2_low  =  0x30, /* NOC_ERL_ERRLOG2_LOW */
	.errlog2_high =  0x34, /* NOC_ERL_ERRLOG2_HIGH */
	.errlog3_low  =  0x38, /* NOC_ERL_ERRLOG3_LOW */
	.errlog3_high =  0x3C, /* NOC_ERL_ERRLOG3_HIGH */
};

static struct cam_cpas_hw_errata_wa_list cam980_cpas100_errata_wa_list = {
	.camnoc_flush_slave_pending_trans = {
		.enable = false,
		.data.reg_info = {
			.access_type = CAM_REG_TYPE_READ,
			.offset = 0x300, /* sbm_SenseIn0_Low */
			.mask = 0xE0000, /* Bits 17, 18, 19 */
			.value = 0, /* expected to be 0 */
		},
	},
	.enable_icp_clk_for_qchannel = {
		.enable = false,
	},
};

static struct cam_cpas_cesta_vcd_reg_info cam_cpas_v980_100_cesta_reg_info = {
	.vcd_currol = {
		.reg_offset = 0x266C,
		.vcd_base_inc = 0x210,
		.num_vcds = 8,
	},
};

static struct cam_cpas_vcd_info cam_v980_100_vcd_info[] = {
	{
		.index = 0, .type = CAM_CESTA_CRMC, .clk = "cam_cc_tfe_0_clk_src",
	},
	{
		.index = 1, .type = CAM_CESTA_CRMC, .clk = "cam_cc_tfe_1_clk_src",
	},
	{
		.index = 2, .type = CAM_CESTA_CRMC, .clk = "cam_cc_tfe_2_clk_src",
	},
	{
		.index = 6, .type = CAM_CESTA_CRMC, .clk = "cam_cc_csid_clk_src",
	},
	{
		.index = 7, .type = CAM_CESTA_CRMC, .clk = "cam_cc_cphy_rx_clk_src",
	},
	{
		.index = 8, .type = CAM_CESTA_CRMB, .clk = "cam_cc_camnoc_axi_rt_clk_src",
	},
};

static struct cam_cpas_cesta_info cam_v980_cesta_info = {
	 .vcd_info = &cam_v980_100_vcd_info[0],
	 .num_vcds = ARRAY_SIZE(cam_v980_100_vcd_info),
	 .cesta_reg_info = &cam_cpas_v980_100_cesta_reg_info,
};

static struct cam_camnoc_info cam980_cpas100_camnoc_info_rt = {
	.specific = &cam_cpas_v980_100_camnoc_specific_rt[0],
	.specific_size = ARRAY_SIZE(cam_cpas_v980_100_camnoc_specific_rt),
	.irq_sbm = &cam_cpas_v980_100_irq_sbm_rt,
	.irq_err = &cam_cpas_v980_100_irq_err_rt[0],
	.irq_err_size = ARRAY_SIZE(cam_cpas_v980_100_irq_err_rt),
	.err_logger = &cam980_cpas100_err_logger_offsets,
	.errata_wa_list = &cam980_cpas100_errata_wa_list,
	.test_irq_info = {
		.sbm_enable_mask = 0x20,
		.sbm_clear_mask = 0x4,
	},
};

static struct cam_camnoc_info cam980_cpas100_camnoc_info_nrt = {
	.specific = &cam_cpas_v980_100_camnoc_specific_nrt[0],
	.specific_size = ARRAY_SIZE(cam_cpas_v980_100_camnoc_specific_nrt),
	.irq_sbm = &cam_cpas_v980_100_irq_sbm_nrt,
	.irq_err = &cam_cpas_v980_100_irq_err_nrt[0],
	.irq_err_size = ARRAY_SIZE(cam_cpas_v980_100_irq_err_nrt),
	.err_logger = &cam980_cpas100_err_logger_offsets,
	.errata_wa_list = &cam980_cpas100_errata_wa_list,
	.test_irq_info = {
		.sbm_enable_mask = 0x400,
		.sbm_clear_mask = 0x1,
	},
};

static struct cam_cpas_camnoc_qchannel cam980_cpas100_qchannel_info_rt = {
	.qchannel_ctrl   = 0xEC,
	.qchannel_status = 0xF0,
};

static struct cam_cpas_camnoc_qchannel cam980_cpas100_qchannel_info_nrt = {
	.qchannel_ctrl   = 0xF4,
	.qchannel_status = 0xF8,
};

static struct cam_cpas_info cam980_cpas100_cpas_info = {
	.hw_caps_info = {
		.num_caps_registers = 2,
		.hw_caps_offsets = {0x8, 0xDC},
	},
	.qchannel_info = {&cam980_cpas100_qchannel_info_rt,
		&cam980_cpas100_qchannel_info_nrt},
	.num_qchannel = 2,
};

#endif /* _CPASTOP_V980_100_H_ */
