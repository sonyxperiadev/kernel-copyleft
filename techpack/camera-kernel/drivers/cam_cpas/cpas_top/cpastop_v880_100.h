/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CPASTOP_V880_100_H_
#define _CPASTOP_V880_100_H_

static struct cam_camnoc_irq_sbm cam_cpas_v880_100_irq_sbm = {
	.sbm_enable = {
		.access_type = CAM_REG_TYPE_READ_WRITE,
		.enable = true,
		.offset = 0x240,  /* CAM_NOC_SBM_FAULTINEN0_LOW */
		.value = 0x01 |/* SBM_FAULTINEN0_LOW_PORT0_MASK */
			0x02 | /* SBM_FAULTINEN0_LOW_PORT1_MASK */
			0x04 |    /* SBM_FAULTINEN0_LOW_PORT2_MASK */
			0x08 |    /* SBM_FAULTINEN0_LOW_PORT3_MASK */
			0x10 |    /* SBM_FAULTINEN0_LOW_PORT4_MASK */
			0x20,     /* SBM_FAULTINEN0_LOW_PORT5_MASK */
	},
	.sbm_status = {
		.access_type = CAM_REG_TYPE_READ,
		.enable = true,
		.offset = 0x248, /* CAM_NOC_SBM_FAULTINSTATUS0_LOW */
	},
	.sbm_clear = {
		.access_type = CAM_REG_TYPE_WRITE,
		.enable = true,
		.offset = 0x280, /* CAM_NOC_SBM_FLAGOUTCLR0_LOW */
		.value = 0xE,
	}
};

static struct cam_camnoc_irq_err
	cam_cpas_v880_100_irq_err[] = {
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_SLAVE_ERROR,
		.enable = true,
		.sbm_port = 0x1, /* SBM_FAULTINSTATUS0_LOW_PORT0_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x8, /* CAM_NOC_ERL_MAINCTL_LOW */
			.value = 1,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x10, /* CAM_NOC_ERL_ERRVLD_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x18, /* CAM_NOC_ERL_ERRCLR_LOW */
			.value = 1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_IFE_UBWC_ENCODE_ERROR,
		.enable = true,
		.sbm_port = 0x20, /* SBM_FAULTINSTATUS0_LOW_PORT5_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x93A0, /* IFE_UBWC : RT_1_NIU_ENCERREN_LOW */
			.value = 0xF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x9390, /* IFE_UBWC : RT_1_NIU_ENCERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x9398, /* IFE_UBWC : RT_1_NIU_ENCERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_BPS_UBWC_ENCODE_ERROR,
		.enable = true,
		.sbm_port = 0x2, /* SBM_FAULTINSTATUS0_LOW_PORT1_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x87A0, /* BPS_WR : NRT_2_NIU_ENCERREN_LOW */
			.value = 0XF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x8790, /* BPS_WR : NRT_2_NIU_ENCERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x8798, /* BPS_WR : NRT_2_NIU_ENCERRCLR_LOW */
			.value = 0X1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_IPE0_UBWC_DECODE_ERROR,
		.enable = true,
		.sbm_port = 0x4, /* SBM_FAULTINSTATUS0_LOW_PORT2_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x8B20, /* IPE_0_RD : NRT_4_NIU_DECERREN_LOW */
			.value = 0xFF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x8B10, /* IPE_0_RD : NRT_4_NIU_DECERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x8B18, /* IPE_0_RD : NRT_4_NIU_DECERRCLR_LOW */
			.value = 0X1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_IPE1_UBWC_DECODE_ERROR,
		.enable = true,
		.sbm_port = 0x8, /* SBM_FAULTINSTATUS0_LOW_PORT3_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x8D20, /* IPE_1_RD : NRT_5_NIU_DECERREN_LOW */
			.value = 0XFF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x8D10, /* IPE_1_RD : NRT_5_NIU_DECERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x8D18, /* IPE_1_RD : NRT_5_NIU_DECERRCLR_LOW */
			.value = 0X1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_IPE_UBWC_ENCODE_ERROR,
		.enable = true,
		.sbm_port = 0x10, /* SBM_FAULTINSTATUS0_LOW_PORT4_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x8FA0, /* IPE_WR : NRT_6_NIU_ENCERREN_LOW */
			.value = 0XF,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x8F90, /* IPE_WR : NRT_6_NIU_ENCERRSTATUS_LOW */
		},
		.err_clear = {
			.access_type = CAM_REG_TYPE_WRITE,
			.enable = true,
			.offset = 0x8F98, /* IPE_WR : NRT_6_NIU_ENCERRCLR_LOW */
			.value = 0x1,
		},
	},
	{
		.irq_type = CAM_CAMNOC_HW_IRQ_AHB_TIMEOUT,
		.enable = false,
		.sbm_port = 0x40, /* SBM_FAULTINSTATUS0_LOW_PORT6_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x288, /* CAM_NOC_SBM_FLAGOUTSET0_LOW */
			.value = 0x1,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x290, /* CAM_NOC_SBM_FLAGOUTSTATUS0_LOW */
		},
		.err_clear = {
			.enable = false, /* CAM_NOC_SBM_FLAGOUTCLR0_LOW */
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
		.sbm_port = 0x80, /* SBM_FAULTINSTATUS0_LOW_PORT7_MASK */
		.err_enable = {
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.enable = true,
			.offset = 0x288, /* CAM_NOC_SBM_FLAGOUTSET0_LOW */
			.value = 0x3,
		},
		.err_status = {
			.access_type = CAM_REG_TYPE_READ,
			.enable = true,
			.offset = 0x290, /* CAM_NOC_SBM_FLAGOUTSTATUS0_LOW */
		},
		.err_clear = {
			.enable = false, /* CAM_NOC_SBM_FLAGOUTCLR0_LOW */
		},
	},
};

static struct cam_camnoc_specific
	cam_cpas_v880_100_camnoc_specific[] = {
	/* RT ports */
	{
		.port_name = "RT0-SFE_RD",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9030, /* SFE_RD : NOC_RT_0_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9034, /* SFE_RD : NOC_RT_0_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9038, /* SFE_RD : NOC_RT_0_NIU_URGENCY_LOW */
			.value = 0x4,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9040, /* SFE_RD : NOC_RT_0_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9048, /* SFE_RD : NOC_RT_0_NIU_SAFELUT_LOW */
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
			.offset = 0x9D88, /* SFE_RD : NOC_RT_0_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7388, /* SFE_RD : NOC_RT_0_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x73A0, /* SFE_RD : NOC_RT_0_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x73A4, /* SFE_RD : NOC_RT_0_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT1-IFE_UBWC",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9230, /* IFE_UBWC_LINEAR : NOC_RT_1_NIU_PRIORITYLUT_LOW */
			.value = 0x65555544,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9234, /* IFE_UBWC_LINEAR : NOC_RT_1_NIU_PRIORITYLUT_HIGH */
			.value = 0x66666666,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9238, /* IFE_UBWC_LINEAR : NOC_RT_1_NIU_URGENCY_LOW */
			.value = 0x1E40,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9240, /* IFE_UBWC_LINEAR : NOC_RT_1_NIU_DANGERLUT_LOW */
			.value = 0xffffff00,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9248, /* IFE_UBWC_LINEAR : NOC_RT_1_NIU_SAFELUT_LOW */
			.value = 0x000f,
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
			.offset = 0x9E08, /* IFE_UBWC_LINEAR : NOC_RT_1_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7408, /* IFE_UBWC_LINEAR : NOC_RT_1_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7420, /* IFE_UBWC_LINEAR : NOC_RT_1_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7424, /* IFE_UBWC_LINEAR : NOC_RT_1_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x9220, /* IFE_UBWC_LINEAR : NOC_RT_1_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT2-IFE_STATS",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9430, /* IFE_STATS : NOC_RT_2_NIU_PRIORITYLUT_LOW */
			.value = 0x65555544,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9434, /* IFE_STATS : NOC_RT_2_NIU_PRIORITYLUT_HIGH */
			.value = 0x66666666,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9438, /* IFE_STATS : NOC_RT_2_NIU_URGENCY_LOW */
			.value = 0x1C40,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9440, /* IFE_STATS : NOC_RT_2_NIU_DANGERLUT_LOW */
			.value = 0xffffff00,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9448, /* IFE_STATS : NOC_RT_2_NIU_SAFELUT_LOW */
			.value = 0x000f,
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
			.offset = 0x9E88, /* IFE_STATS : NOC_RT_2_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7488, /* IFE_STATS : NOC_RT_2_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x74A0, /* IFE_STATS : NOC_RT_2_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x74A4, /* IFE_STATS : NOC_RT_2_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x9420, /* IFE_STATS : NOC_RT_2_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT3-IFE_PDAF_LINEAR_IFELITE",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9630, /* IFE_PDAF_IFELITE : NOC_RT_3_NIU_PRIORITYLUT_LOW */
			.value = 0x65555544,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9634, /* IFE_PDAF_IFELITE : NOC_RT_3_NIU_PRIORITYLUT_HIGH */
			.value = 0x66666666,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9638, /* IFE_PDAF_IFELITE : NOC_RT_3_NIU_URGENCY_LOW */
			.value = 0x1C40,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9640, /* IFE_PDAF_IFELITE : NOC_RT_3_NIU_DANGERLUT_LOW */
			.value = 0xffffff00,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9648, /* IFE_PDAF_IFELITE : NOC_RT_3_NIU_SAFELUT_LOW */
			.value = 0x000f,
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
			.offset = 0x9F08, /* IFE_PDAF_IFELITE : NOC_RT_3_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7508, /* IFE_PDAF_IFELITE : NOC_RT_3_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7520, /* IFE_PDAF_IFELITE : NOC_RT_3_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7524, /* IFE_PDAF_IFELITE : NOC_RT_3_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x9620, /* IFE_PDAF_IFELITE : NOC_RT_3_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "RT4-IFE_RDI_SFE",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9830, /* IFE_RDI_SFE : NOC_RT_4_NIU_PRIORITYLUT_LOW */
			.value = 0x65555544,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9834, /* IFE_RDI_SFE : NOC_RT_4_NIU_PRIORITYLUT_HIGH */
			.value = 0x66666666,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9838, /* IFE_RDI_SFE : NOC_RT_4_NIU_URGENCY_LOW */
			.value = 0x1E40,
		},
		.danger_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9840, /* IFE_RDI_SFE : NOC_RT_4_NIU_DANGERLUT_LOW */
			.value = 0xffffff00,
		},
		.safe_lut = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9848, /* IFE_RDI_SFE : NOC_RT_4_NIU_SAFELUT_LOW */
			.value = 0x000f,
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
			.offset = 0x9F88, /* IFE_RDI_SFE : NOC_RT_4_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7588, /* IFE_RDI_SFE : NOC_RT_4_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x75A0, /* IFE_RDI_SFE : NOC_RT_4_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x75A4, /* IFE_RDI_SFE : NOC_RT_4_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x9820, /* IFE_RDI_SFE : NOC_RT_4_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	/* NRT ports */
	{
		.port_name = "NRT0-CDM",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8230, /* CDM : NOC_NRT_0_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8234, /* CDM : NOC_NRT_0_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8238, /* CDM : NOC_NRT_0_NIU_URGENCY_LOW */
			.value = 0x3,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8240, /* CDM : NOC_NRT_0_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8248, /* CDM : NOC_NRT_0_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9A08, /* CDM : NOC_NRT_0_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7008, /* CDM : NOC_NRT_0_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7020, /* CDM : NOC_NRT_0_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7024, /* CDM : NOC_NRT_0_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT1-JPEG_RD_WR",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8430, /* JPEG : NOC_NRT_1_NIU_PRIORITYLUT_LOW */
			.value = 0x22222222,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8434, /* JPEG : NOC_NRT_1_NIU_PRIORITYLUT_HIGH */
			.value = 0x22222222,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8438, /* JPEG : NOC_NRT_1_NIU_URGENCY_LOW */
			.value = 0x22,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8440, /* JPEG : NOC_NRT_1_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8448, /* JPEG : NOC_NRT_1_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9A88, /* JPEG : NOC_NRT_1_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7088, /* JPEG : NOC_NRT_1_QOSGEN_MAINCTL */
			.value = 0x2,
		},
		.qosgen_shaping_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x70A0, /* JPEG : NOC_NRT_1_QOSGEN_SHAPING_LOW */
			.value = 0x10101010,
		},
		.qosgen_shaping_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x70A4, /* JPEG : NOC_NRT_1_QOSGEN_SHAPING_HIGH */
			.value = 0x10101010,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x8420, /* JPEG : NOC_NRT_1_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT2-BPS_CRE_WR",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8630, /* BPS_CRE_WR : NOC_NRT_2_NIU_PRIORITYLUT_LOW */
			.value = 0x33333333,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8634, /* BPS_CRE_WR : NOC_NRT_2_NIU_PRIORITYLUT_HIGH */
			.value = 0x33333333,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8638, /* BPS_CRE_WR : NOC_NRT_2_NIU_URGENCY_LOW */
			.value = 0x30,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8640, /* BPS_CRE_WR : NOC_NRT_2_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8648, /* BPS_CRE_WR : NOC_NRT_2_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9B08, /* BPS_CRE_WR : NOC_NRT_2_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7108, /* BPS_CRE_WR : NOC_NRT_2_QOSGEN_MAINCTL */
			.value = 0x2,
		},
		.qosgen_shaping_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7120, /* BPS_CRE_WR : NOC_NRT_2_QOSGEN_SHAPING_LOW */
			.value = 0x14141414,
		},
		.qosgen_shaping_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7124, /* BPS_CRE_WR : NOC_NRT_2_BPS_WR_QOSGEN_SHAPING_HIGH */
			.value = 0x14141414,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x8620, /* BPS_CRE_WR : NOC_NRT_2_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT3-BPS_CRE_RD",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8830, /* BPS_CRE_RD : NOC_NRT_3_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8834, /* BPS_CRE_RD : NOC_NRT_3_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8838, /* BPS_CRE_RD : NOC_NRT_3_NIU_URGENCY_LOW */
			.value = 0x3,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8840, /* BPS_CRE_RD : NOC_NRT_3_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8848, /* BPS_CRE_RD : NOC_NRT_3_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9B88, /* BPS_CRE_RD : NOC_NRT_3_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7188, /* BPS_CRE_RD : NOC_NRT_3_QOSGEN_MAINCTL */
			.value = 0x2,
		},
		.qosgen_shaping_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x71A0, /* BPS_CRE_RD : NOC_NRT_3_QOSGEN_SHAPING_LOW */
			.value = 0x14141414,
		},
		.qosgen_shaping_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x71A4, /* BPS_CRE_RD : NOC_NRT_3_QOSGEN_SHAPING_HIGH */
			.value = 0x14141414,
		},
	},
	{
		.port_name = "NRT4-IPE_0_RD",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8A30, /* IPE_0_RD : NOC_NRT_4_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8A34, /* IPE_0_RD : NOC_NRT_4_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8A38, /* IPE_0_RD : NOC_NRT_4_NIU_URGENCY_LOW */
			.value = 0x3,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8A40, /* IPE_0_RD : NOC_NRT_4_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8A48, /* IPE_0_RD : NOC_NRT_4_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8B08, /* IPE_0_RD : NOC_NRT_4_NIU_DECCTL_LOW */
			.value = 1,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9C08, /* IPE_0_RD : NOC_NRT_4_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7208, /* IPE_0_RD : NOC_NRT_4_QOSGEN_MAINCTL */
			.value = 0x2,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7220, /* IPE_0_RD : NOC_NRT_4_QOSGEN_SHAPING_LOW */
			.value = 0x2E2E2E2E,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7224, /* IPE_0_RD : NOC_NRT_4_QOSGEN_SHAPING_HIGH */
			.value = 0x2E2E2E2E,
		},
	},
	{
		.port_name = "NRT5-IPE_1_RD",
		.enable = true,
		.priority_lut_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8C30, /* IPE_1_RD : NOC_NRT_5_NIU_PRIORITYLUT_LOW */
			.value = 0x0,
		},
		.priority_lut_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8C34, /* IPE_1_RD : NOC_NRT_5_NIU_PRIORITYLUT_HIGH */
			.value = 0x0,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8C38, /* IPE_1_RD : NOC_NRT_5_NIU_URGENCY_LOW */
			.value = 0x3,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8C40, /* IPE_1_RD : NOC_NRT_5_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8C48, /* IPE_1_RD : NOC_NRT_5_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9C88, /* IPE_1_RD : NOC_NRT_5_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7288, /* IPE_1_RD : NOC_NRT_5_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x72A0, /* IPE_1_RD : NOC_NRT_5_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x72A4, /* IPE_1_RD : NOC_NRT_5_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
	{
		.port_name = "NRT6-IPE_WR_0",
		.enable = true,
		.priority_lut_low = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8E30, /* IPE_WR_0 : NOC_NRT_6_NIU_PRIORITYLUT_LOW */
			.value = 0x33333333,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8E34, /* IPE_WR_0 : NOC_NRT_6_NIU_PRIORITYLUT_HIGH */
			.value = 0x33333333,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8E38, /* IPE_WR_0 : NOC_NRT_6_NIU_URGENCY_LOW */
			.value = 0x30,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8E40, /* IPE_WR_0 : NOC_NRT_6_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x8E48, /* IPE_WR_0 : NOC_NRT_6_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x9D08, /* IPE_WR_0 : NOC_NRT_6_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7308, /* IPE_WR_0 : NOC_NRT_6_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7320, /* IPE_WR_0 : NOC_NRT_6_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7324, /* IPE_WR_0 : NOC_NRT_6_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0x8E20, /* IPE_WR_0 : NOC_NRT_6_NIU_MAXWR_LOW */
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
			.offset = 0xA430, /* IPE_WR_1 : NOC_NRT_7_NIU_PRIORITYLUT_LOW */
			.value = 0x33333333,
		},
		.priority_lut_high = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA434, /* IPE_WR_1 : NOC_NRT_7_NIU_PRIORITYLUT_HIGH */
			.value = 0x33333333,
		},
		.urgency = {
			.enable = true,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA438, /* IPE_WR_1 : NOC_NRT_7_NIU_URGENCY_LOW */
			.value = 0x30,
		},
		.danger_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA440, /* IPE_WR_1 : NOC_NRT_7_NIU_DANGERLUT_LOW */
			.value = 0x0,
		},
		.safe_lut = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA448, /* IPE_WR_1 : NOC_NRT_7_NIU_SAFELUT_LOW */
			.value = 0xffff,
		},
		.ubwc_ctl = {
			.enable = false,
		},
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA608, /* IPE_WR_1 : NOC_NRT_7_DYNATTR_MAINCTL */
			.value = 0x0,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA688, /* IPE_WR_1 : NOC_NRT_7_QOSGEN_MAINCTL */
			.value = 0x0,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA6A0, /* IPE_WR_1 : NOC_NRT_7_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA6A4, /* IPE_WR_1 : NOC_NRT_7_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
		.maxwr_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ,
			.masked_value = 0,
			.offset = 0xA420, /* IPE_WR_1 : NOC_NRT_7_NIU_MAXWR_LOW */
			.value = 0x0,
		},
	},
	{
		.port_name = "ICP_RD_WR",
		.enable = false,
		.dynattr_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0xA008, /* ICP_RD_WR : NOC_XM_ICP_DYNATTR_MAINCTL */
			.value = 0x10,
		},
		.qosgen_mainctl = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7608, /* ICP_RD_WR : NOC_XM_ICP_QOSGEN_MAINCTL */
			.value = 0x50,
		},
		.qosgen_shaping_low = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7620, /* ICP_RD_WR : NOC_XM_ICP_QOSGEN_SHAPING_LOW */
			.value = 0x0,
		},
		.qosgen_shaping_high = {
			.enable = false,
			.access_type = CAM_REG_TYPE_READ_WRITE,
			.masked_value = 0,
			.offset = 0x7624, /* ICP_RD_WR : NOC_XM_ICP_QOSGEN_SHAPING_HIGH */
			.value = 0x0,
		},
	},
};

static struct cam_camnoc_err_logger_info cam880_cpas100_err_logger_offsets = {
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

static struct cam_cpas_hw_errata_wa_list cam880_cpas100_errata_wa_list = {
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
		.enable = true,
	},
};

static struct cam_cpas_cesta_vcd_reg_info cam_cpas_v880_100_cesta_reg_info = {
	.vcd_currol = {
		.reg_offset = 0x300c,
		.vcd_base_inc = 0x200,
		.num_vcds = 8,
	},

};

static struct cam_cpas_vcd_info cam_v880_100_vcd_info[] = {
	{
		.index = 0, .type = CAM_CESTA_CRMC, .clk = "cam_cc_ife_0_clk_src",
	},
	{
		.index = 1, .type = CAM_CESTA_CRMC, .clk = "cam_cc_ife_1_clk_src",
	},
	{
		.index = 2, .type = CAM_CESTA_CRMC, .clk = "cam_cc_ife_2_clk_src",
	},
	{
		.index = 3, .type = CAM_CESTA_CRMC, .clk = "cam_cc_sfe_0_clk_src",
	},
	{
		.index = 4, .type = CAM_CESTA_CRMC, .clk = "cam_cc_sfe_1_clk_src",
	},
	{
		.index = 5, .type = CAM_CESTA_CRMC, .clk = "cam_cc_sfe_2_clk_src",
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

static struct cam_cpas_cesta_info cam_v880_cesta_info = {
	 .vcd_info = &cam_v880_100_vcd_info[0],
	 .num_vcds = ARRAY_SIZE(cam_v880_100_vcd_info),
	 .cesta_reg_info = &cam_cpas_v880_100_cesta_reg_info,
};

static struct cam_camnoc_info cam880_cpas100_camnoc_info = {
	.specific = &cam_cpas_v880_100_camnoc_specific[0],
	.specific_size = ARRAY_SIZE(cam_cpas_v880_100_camnoc_specific),
	.irq_sbm = &cam_cpas_v880_100_irq_sbm,
	.irq_err = &cam_cpas_v880_100_irq_err[0],
	.irq_err_size = ARRAY_SIZE(cam_cpas_v880_100_irq_err),
	.err_logger = &cam880_cpas100_err_logger_offsets,
	.errata_wa_list = &cam880_cpas100_errata_wa_list,
	.test_irq_info = {
		.sbm_enable_mask = 0x80,
		.sbm_clear_mask = 0x4,
	},
};

static struct cam_cpas_camnoc_qchannel cam880_cpas100_qchannel_info = {
	.qchannel_ctrl   = 0x5C,
	.qchannel_status = 0x60,
};

static struct cam_cpas_info cam880_cpas100_cpas_info = {
	.hw_caps_info = {
		.num_caps_registers = 2,
		.hw_caps_offsets = {0x8, 0xDC},
	},
	.qchannel_info = {&cam880_cpas100_qchannel_info},
	.num_qchannel = 1,
};

#endif /* _CPASTOP_V880_100_H_ */
