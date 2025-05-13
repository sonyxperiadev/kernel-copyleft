// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/io.h>
#include <soc/qcom/of_common.h>
#include "msm_cvp_internal.h"
#include "msm_cvp_debug.h"
#include "cvp_hfi_api.h"
#include "cvp_hfi.h"

#define UBWC_CONFIG(mco, mlo, hbo, bslo, bso, rs, mc, ml, hbb, bsl, bsp) \
{	\
	.override_bit_info.max_channel_override = mco,	\
	.override_bit_info.mal_length_override = mlo,	\
	.override_bit_info.hb_override = hbo,	\
	.override_bit_info.bank_swzl_level_override = bslo,	\
	.override_bit_info.bank_spreading_override = bso,	\
	.override_bit_info.reserved = rs,	\
	.max_channels = mc,	\
	.mal_length = ml,	\
	.highest_bank_bit = hbb,	\
	.bank_swzl_level = bsl,	\
	.bank_spreading = bsp,	\
}

static struct msm_cvp_common_data default_common_data[] = {
	{
		.key = "qcom,auto-pil",
		.value = 1,
	},
};

static struct msm_cvp_common_data sm8450_common_data[] = {
	{
		.key = "qcom,pm-qos-latency-us",
		.value = 50,
	},
	{
		.key = "qcom,sw-power-collapse",
		.value = 1,
	},
	{
		.key = "qcom,domain-attr-non-fatal-faults",
		.value = 1,
	},
	{
		.key = "qcom,max-secure-instances",
		.value = 2,             /*
					 * As per design driver allows 3rd
					 * instance as well since the secure
					 * flags were updated later for the
					 * current instance. Hence total
					 * secure sessions would be
					 * max-secure-instances + 1.
					 */
	},
	{
		.key = "qcom,max-ssr-allowed",
		.value = 1,		/*
					 * Maxinum number of SSR before BUG_ON
					 */
	},
	{
		.key = "qcom,power-collapse-delay",
		.value = 3000,
	},
	{
		.key = "qcom,hw-resp-timeout",
		.value = 2000,
	},
	{
		.key = "qcom,dsp-resp-timeout",
		.value = 1000,
	},
	{
		.key = "qcom,debug-timeout",
		.value = 0,
	},
	{
		.key = "qcom,dsp-enabled",
		.value = 1,
	}
};

static struct msm_cvp_common_data sm8550_common_data[] = {
	{
		.key = "qcom,pm-qos-latency-us",
		.value = 50,
	},
	{
		.key = "qcom,sw-power-collapse",
		.value = 1,
	},
	{
		.key = "qcom,domain-attr-non-fatal-faults",
		.value = 0,
	},
	{
		.key = "qcom,max-secure-instances",
		.value = 2,             /*
					 * As per design driver allows 3rd
					 * instance as well since the secure
					 * flags were updated later for the
					 * current instance. Hence total
					 * secure sessions would be
					 * max-secure-instances + 1.
					 */
	},
	{
		.key = "qcom,max-ssr-allowed",
		.value = 1,		/*
					 * Maxinum number of SSR before BUG_ON
					 */
	},
	{
		.key = "qcom,power-collapse-delay",
		.value = 3000,
	},
	{
		.key = "qcom,hw-resp-timeout",
		.value = 2000,
	},
	{
		.key = "qcom,dsp-resp-timeout",
		.value = 1000,
	},
	{
		.key = "qcom,debug-timeout",
		.value = 0,
	},
	{
		.key = "qcom,dsp-enabled",
		.value = 1,
	}
};

static struct msm_cvp_common_data sm8550_tvm_common_data[] = {
	{
		.key = "qcom,pm-qos-latency-us",
		.value = 50,
	},
	{
		.key = "qcom,sw-power-collapse",
		.value = 0,
	},
	{
		.key = "qcom,domain-attr-non-fatal-faults",
		.value = 0,
	},
	{
		.key = "qcom,max-secure-instances",
		.value = 2,             /*
					 * As per design driver allows 3rd
					 * instance as well since the secure
					 * flags were updated later for the
					 * current instance. Hence total
					 * secure sessions would be
					 * max-secure-instances + 1.
					 */
	},
	{
		.key = "qcom,max-ssr-allowed",
		.value = 1,		/*
					 * Maxinum number of SSR before BUG_ON
					 */
	},
	{
		.key = "qcom,power-collapse-delay",
		.value = 3000,
	},
	{
		.key = "qcom,hw-resp-timeout",
		.value = 2000,
	},
	{
		.key = "qcom,dsp-resp-timeout",
		.value = 1000,
	},
	{
		.key = "qcom,debug-timeout",
		.value = 0,
	},
	{
		.key = "qcom,dsp-enabled",
		.value = 0,
	}
};

static struct msm_cvp_common_data sm8650_common_data[] = {
	{
		.key = "qcom,pm-qos-latency-us",
		.value = 50,
	},
	{
		.key = "qcom,sw-power-collapse",
		.value = 1,
	},
	{
		.key = "qcom,domain-attr-non-fatal-faults",
		.value = 0,
	},
	{
		.key = "qcom,max-secure-instances",
		.value = 2, /*
					* As per design driver allows 3rd
					* instance as well since the secure
					* flags were updated later for the
					* current instance. Hence total
					* secure sessions would be
					* max-secure-instances + 1.
					*/
	},
	{
		.key = "qcom,max-ssr-allowed",
		.value = 1,	/*
					* Maxinum number of SSR before BUG_ON
					*/
	},
	{
		.key = "qcom,power-collapse-delay",
		.value = 3000,
	},
	{
		.key = "qcom,hw-resp-timeout",
		.value = 2000,
	},
	{
		.key = "qcom,dsp-resp-timeout",
		.value = 1000,
	},
	{
		.key = "qcom,debug-timeout",
		.value = 0,
	},
	{
		.key = "qcom,dsp-enabled",
		.value = 1,
	},
	{
		.key = "qcom,qos_noc_rge_niu_offset",
		.value = 0x0,
	},
	{
		.key = "qcom,qos_noc_gce_vadl_tof_niu_offset",
		.value = 0x0,
	},
	{
		.key = "qcom,qos_noc_cdm_niu_offset",
		.value = 0x0,
	},
	{
		.key = "qcom,noc_core_err_offset",
		.value = 0xA000,
	},
	{
		.key = "qcom,noc_main_sidebandmanager_offset",
		.value = 0x6E00,
	}
};

static struct msm_cvp_common_data sm7650_common_data[] = {
	{
		.key = "qcom,pm-qos-latency-us",
		.value = 50,
	},
	{
		.key = "qcom,sw-power-collapse",
		.value = 1,
	},
	{
		.key = "qcom,domain-attr-non-fatal-faults",
		.value = 0,
	},
	{
		.key = "qcom,max-secure-instances",
		.value = 2,
	},
	{
		.key = "qcom,max-ssr-allowed",
		.value = 1,
	},
	{
		.key = "qcom,power-collapse-delay",
		.value = 3000,
	},
	{
		.key = "qcom,hw-resp-timeout",
		.value = 2000,
	},
	{
		.key = "qcom,dsp-resp-timeout",
		.value = 1000,
	},
	{
		.key = "qcom,debug-timeout",
		.value = 0,
	},
	{
		.key = "qcom,dsp-enabled",
		.value = 1,
	},
	{
		.key = "qcom,qos_noc_rge_niu_offset",
		.value = 0x200,
	},
	{
		.key = "qcom,qos_noc_gce_vadl_tof_niu_offset",
		.value = 0xE00,
	},
	{
		.key = "qcom,qos_noc_cdm_niu_offset",
		.value = 0x1A00,
	},
	{
		.key = "qcom,noc_core_err_offset",
		.value = 0x0,
	},
	{
		.key = "qcom,noc_main_sidebandmanager_offset",
		.value = 0x0,
	}
};

/* Default UBWC config for LPDDR5 */
static struct msm_cvp_ubwc_config_data kona_ubwc_data[] = {
	UBWC_CONFIG(1, 1, 1, 0, 0, 0, 8, 32, 16, 0, 0),
};

static struct msm_cvp_qos_setting waipio_noc_qos = {
	.axi_qos = 0x99,
	.prioritylut_low = 0x22222222,
	.prioritylut_high = 0x33333333,
	.urgency_low = 0x1022,
	.dangerlut_low = 0x0,
	.safelut_low = 0xffff,
};

static struct msm_cvp_qos_setting lanai_noc_qos = {
	.axi_qos = 0x99,
	.prioritylut_low = 0x33333333,
	.prioritylut_high = 0x33333333,
	.urgency_low = 0x1033,
	.urgency_low_ro = 0x1003,
	.dangerlut_low = 0x0,
	.safelut_low = 0xffff,
};

static struct msm_cvp_qos_setting palawan_noc_qos = {
	.axi_qos = 0x99,
	.prioritylut_low = 0x33333333,
	.prioritylut_high = 0x33333333,
	.urgency_low = 0x1003,
	.urgency_low_ro = 0x1003,
	.dangerlut_low = 0x0,
	.safelut_low = 0xffff,
};

static struct msm_cvp_platform_data default_data = {
	.common_data = default_common_data,
	.common_data_length =  ARRAY_SIZE(default_common_data),
	.sku_version = 0,
	.vpu_ver = VPU_VERSION_5,
	.ubwc_config = 0x0,
	.noc_qos = 0x0,
	.vm_id = 1,
};

static struct msm_cvp_platform_data sm8450_data = {
	.common_data = sm8450_common_data,
	.common_data_length =  ARRAY_SIZE(sm8450_common_data),
	.sku_version = 0,
	.vpu_ver = VPU_VERSION_5,
	.ubwc_config = kona_ubwc_data,
	.noc_qos = &waipio_noc_qos,
	.vm_id = 1,
};

static struct msm_cvp_platform_data sm8550_data = {
	.common_data = sm8550_common_data,
	.common_data_length =  ARRAY_SIZE(sm8550_common_data),
	.sku_version = 0,
	.vpu_ver = VPU_VERSION_5,
	.ubwc_config = kona_ubwc_data,	/*Reuse Kona setting*/
	.noc_qos = &waipio_noc_qos,	/*Reuse Waipio setting*/
	.vm_id = 1,
};

static struct msm_cvp_platform_data sm8550_tvm_data = {
	.common_data = sm8550_tvm_common_data,
	.common_data_length =  ARRAY_SIZE(sm8550_tvm_common_data),
	.sku_version = 0,
	.vpu_ver = VPU_VERSION_5,
	.ubwc_config = kona_ubwc_data,	/*Reuse Kona setting*/
	.noc_qos = &waipio_noc_qos,	/*Reuse Waipio setting*/
	.vm_id = 2,
};

static struct msm_cvp_platform_data sm8650_data = {
	.common_data = sm8650_common_data,
	.common_data_length = ARRAY_SIZE(sm8650_common_data),
	.sku_version = 0,
	.vpu_ver = VPU_VERSION_5,
	.ubwc_config = kona_ubwc_data,	/*Reuse Kona setting*/
	.noc_qos = &lanai_noc_qos,
	.vm_id = 1,
};

static struct msm_cvp_platform_data sm7650_data = {
	.common_data = sm7650_common_data,
	.common_data_length = ARRAY_SIZE(sm7650_common_data),
	.sku_version = 0,
	.vpu_ver = VPU_VERSION_5,
	.ubwc_config = kona_ubwc_data,	/*Reuse Kona setting*/
	.noc_qos = &palawan_noc_qos,
	.vm_id = 1,
};

static const struct of_device_id msm_cvp_dt_match[] = {
	{
		.compatible = "qcom,waipio-cvp",
		.data = &sm8450_data,
	},
	{
		.compatible = "qcom,kalama-cvp",
		.data = &sm8550_data,
	},
	{
		.compatible = "qcom,kalama-cvp-tvm",
		.data = &sm8550_tvm_data,
	},
	{
		.compatible = "qcom,pineapple-cvp",
		.data = &sm8650_data,
	},
	{
		.compatible = "qcom,cliffs-cvp",
		.data = &sm7650_data,
	},
	{},
};

/*
 * WARN: name field CAN NOT hold more than 23 chars
 *	 excluding the ending '\0'
 *
 * NOTE: the def entry index for the command packet is
 *	 "the packet type - HFI_CMD_SESSION_CVP_START"
 */
const struct msm_cvp_hfi_defs cvp_hfi_defs[MAX_PKT_IDX] = {
	[HFI_CMD_SESSION_CVP_DFS_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DFS_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DFS_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DFS",
		},
	[HFI_CMD_SESSION_CVP_DFS_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DFS_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DFS_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DFS_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_SGM_OF_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_SGM_OF_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "SGM_OF",
		},
	[HFI_CMD_SESSION_CVP_SGM_OF_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_SGM_OF_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "SGM_OF_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_WARP_NCC_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_WARP_NCC_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "WARP_NCC",
		},
	[HFI_CMD_SESSION_CVP_WARP_NCC_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_WARP_NCC_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "WARP_NCC_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_WARP_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_WARP_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "WARP",
		},
	[HFI_CMD_SESSION_CVP_WARP_DS_PARAMS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_WARP_DS_PARAMS,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "WARP_DS_PARAMS",
		},
	[HFI_CMD_SESSION_CVP_WARP_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_WARP_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "WARP_FRAME",
			.force_kernel_fence = true,
		},
	[HFI_CMD_SESSION_CVP_DMM_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DMM_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DMM_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DMM",
		},
	[HFI_CMD_SESSION_CVP_DMM_PARAMS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_DMM_PARAMS,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DMM_PARAMS",
		},
	[HFI_CMD_SESSION_CVP_DMM_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DMM_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DMM_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DMM_FRAME",
			.force_kernel_fence = true,
		},
	[HFI_CMD_SESSION_CVP_SET_PERSIST_BUFFERS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_PERSIST_CMD_SIZE,
			.type =HFI_CMD_SESSION_CVP_SET_PERSIST_BUFFERS,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "SET_PERSIST",
		},
	[HFI_CMD_SESSION_CVP_RELEASE_PERSIST_BUFFERS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xffffffff,
			.type =HFI_CMD_SESSION_CVP_RELEASE_PERSIST_BUFFERS,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "REL_PERSIST",
		},
	[HFI_CMD_SESSION_CVP_DS_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DS_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DS_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DS_CONFIG",
		},
	[HFI_CMD_SESSION_CVP_DS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DS_CMD_SIZE,
			.type =HFI_CMD_SESSION_CVP_DS,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DS",
		},
	[HFI_CMD_SESSION_CVP_CV_TME_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_OF_CONFIG_CMD_SIZE,
			.type =HFI_CMD_SESSION_CVP_CV_TME_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "TME",
		},
	[HFI_CMD_SESSION_CVP_CV_TME_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_OF_FRAME_CMD_SIZE,
			.type =HFI_CMD_SESSION_CVP_CV_TME_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "TME_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_CV_ODT_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_ODT_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_CV_ODT_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "ODT",
		},
	[HFI_CMD_SESSION_CVP_CV_ODT_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_ODT_FRAME_CMD_SIZE,
			.type =HFI_CMD_SESSION_CVP_CV_ODT_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "ODT_FRAME",
		},
	[HFI_CMD_SESSION_CVP_CV_OD_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_OD_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_CV_OD_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "OD",
		},
	[HFI_CMD_SESSION_CVP_CV_OD_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_OD_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_CV_OD_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "OD_FRAME",
		},
	[HFI_CMD_SESSION_CVP_NCC_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_NCC_CONFIG_CMD_SIZE,
			.type =HFI_CMD_SESSION_CVP_NCC_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "NCC",
		},
	[HFI_CMD_SESSION_CVP_NCC_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_NCC_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_NCC_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "NCC_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_ICA_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_ICA_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_ICA_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "ICA",
		},
	[HFI_CMD_SESSION_CVP_ICA_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_ICA_FRAME_CMD_SIZE,
			.type =HFI_CMD_SESSION_CVP_ICA_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "ICA_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_HCD_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_HCD_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_HCD_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "HCD",
		},
	[HFI_CMD_SESSION_CVP_HCD_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_HCD_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_HCD_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "HCD_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_DC_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DCM_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DC_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DC",
		},
	[HFI_CMD_SESSION_CVP_DC_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DCM_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DC_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DC_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_DCM_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DCM_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DCM_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DCM",
		},
	[HFI_CMD_SESSION_CVP_DCM_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_DCM_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_DCM_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DCM_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_PYS_HCD_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_PYS_HCD_CONFIG_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_PYS_HCD_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "PYS_HCD",
		},
	[HFI_CMD_SESSION_CVP_PYS_HCD_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = HFI_PYS_HCD_FRAME_CMD_SIZE,
			.type = HFI_CMD_SESSION_CVP_PYS_HCD_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "PYS_HCD_FRAME",
			.force_kernel_fence = true,
		},
	[HFI_CMD_SESSION_CVP_SET_MODEL_BUFFERS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_SET_MODEL_BUFFERS,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "SET_MODEL",
		},
	[HFI_CMD_SESSION_CVP_SET_SNAPSHOT_BUFFERS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_SET_SNAPSHOT_BUFFERS,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "SET_SNAPSHOT",
		},
	[HFI_CMD_SESSION_CVP_RELEASE_SNAPSHOT_BUFFERS - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_RELEASE_SNAPSHOT_BUFFERS,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "REL_SNAPSHOT",
		},
	[HFI_CMD_SESSION_CVP_SET_SNAPSHOT_MODE - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_SET_SNAPSHOT_MODE,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "SNAPSHOT_MODE",
		},
	[HFI_CMD_SESSION_CVP_SNAPSHOT_WRITE_DONE - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_SNAPSHOT_WRITE_DONE,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "SNAPSHOT_DONE",
		},
	[HFI_CMD_SESSION_CVP_FD_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_FD_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "FD",
		},
	[HFI_CMD_SESSION_CVP_FD_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_FD_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "FD_FRAME",
		},
	[HFI_CMD_SESSION_CVP_XRA_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "XRA_FRAME",
		},
	[HFI_CMD_SESSION_CVP_XRA_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "XRA_CONFIG",
		},
	[HFI_CMD_SESSION_CVP_XRA_BLOB_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_BLOB_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "XRA_BLOB_FRAME",
		},
	[HFI_CMD_SESSION_CVP_XRA_BLOB_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_BLOB_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "XRA_BLOB_CONFIG",
		},
	[HFI_CMD_SESSION_CVP_XRA_PATCH_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_PATCH_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "XRA_PATCH_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_XRA_PATCH_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_PATCH_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "XRA_PATCH_CONFIG",
		},
	[HFI_CMD_SESSION_CVP_XRA_MATCH_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_MATCH_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "XRA_MATCH_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_CVP_XRA_MATCH_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_XRA_MATCH_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "XRA_MATCH_CONFIG",
		},
	[HFI_CMD_SESSION_CVP_RGE_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_RGE_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "RGE_FRAME",
			.force_kernel_fence = true,
		},
	[HFI_CMD_SESSION_CVP_RGE_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_RGE_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "RGE_CONFIG",
		},
	[HFI_CMD_SESSION_EVA_ITOF_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_ITOF_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "ITOF_FRAME",
			.force_kernel_fence = true,
		},
	[HFI_CMD_SESSION_EVA_ITOF_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_ITOF_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "ITOF_CONFIG",
		},
	[HFI_CMD_SESSION_EVA_DLFD_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_DLFD_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DLFD_FRAME",
		},
	[HFI_CMD_SESSION_EVA_DLFD_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_DLFD_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DLFD_CONFIG",
		},
	[HFI_CMD_SESSION_EVA_DLFL_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_DLFL_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DLFL_FRAME",
			.force_kernel_fence = false,
		},
	[HFI_CMD_SESSION_EVA_DLFL_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_DLFL_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DLFL_CONFIG",
		},
	[HFI_CMD_SESSION_CVP_SYNX - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_CVP_SYNX,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "SYNX_TEST",
		},
	[HFI_CMD_SESSION_EVA_DME_ONLY_CONFIG - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_DME_ONLY_CONFIG,
			.is_config_pkt = true,
			.resp = HAL_NO_RESP,
			.name = "DME_CONFIG",
		},
	[HFI_CMD_SESSION_EVA_DME_ONLY_FRAME - HFI_CMD_SESSION_CVP_START] =
		{
			.size = 0xFFFFFFFF,
			.type = HFI_CMD_SESSION_EVA_DME_ONLY_FRAME,
			.is_config_pkt = false,
			.resp = HAL_NO_RESP,
			.name = "DME_FRAME",
			.force_kernel_fence = true,
		},

};

int get_pkt_index(struct cvp_hal_session_cmd_pkt *hdr)
{
	if (!hdr || (hdr->packet_type < HFI_CMD_SESSION_CVP_START)
		|| hdr->packet_type >= (HFI_CMD_SESSION_CVP_START + MAX_PKT_IDX))
		return -EINVAL;

	if (cvp_hfi_defs[hdr->packet_type - HFI_CMD_SESSION_CVP_START].size)
		return (hdr->packet_type - HFI_CMD_SESSION_CVP_START);

	return -EINVAL;
}

int get_pkt_fenceoverride(struct cvp_hal_session_cmd_pkt* hdr)
{
	return cvp_hfi_defs[hdr->packet_type - HFI_CMD_SESSION_CVP_START].force_kernel_fence;
}



int get_pkt_index_from_type(u32 pkt_type)
{
	if ((pkt_type < HFI_CMD_SESSION_CVP_START) ||
		pkt_type >= (HFI_CMD_SESSION_CVP_START + MAX_PKT_IDX))
		return -EINVAL;

	if (cvp_hfi_defs[pkt_type - HFI_CMD_SESSION_CVP_START].size)
		return (pkt_type - HFI_CMD_SESSION_CVP_START);

	return -EINVAL;
}
MODULE_DEVICE_TABLE(of, msm_cvp_dt_match);

int cvp_of_fdt_get_ddrtype(void)
{
#ifdef FIXED_DDR_TYPE
	/* of_fdt_get_ddrtype() is usually unavailable during pre-sil */
	return DDR_TYPE_LPDDR5;
#else
	return of_fdt_get_ddrtype();
#endif
}

void *cvp_get_drv_data(struct device *dev)
{
	struct msm_cvp_platform_data *driver_data;
	const struct of_device_id *match;
	uint32_t ddr_type = DDR_TYPE_LPDDR5;

	driver_data = &default_data;

	if (!IS_ENABLED(CONFIG_OF) || !dev->of_node)
		goto exit;

	match = of_match_node(msm_cvp_dt_match, dev->of_node);

	if (!match)
		return NULL;

	driver_data = (struct msm_cvp_platform_data *)match->data;

	if (!strcmp(match->compatible, "qcom,waipio-cvp")) {
		ddr_type = cvp_of_fdt_get_ddrtype();
		if (ddr_type == -ENOENT) {
			dprintk(CVP_ERR,
				"Failed to get ddr type, use LPDDR5\n");
		}

		if (driver_data->ubwc_config &&
			(ddr_type == DDR_TYPE_LPDDR4 ||
			ddr_type == DDR_TYPE_LPDDR4X))
			driver_data->ubwc_config->highest_bank_bit = 15;
		dprintk(CVP_CORE, "DDR Type 0x%x hbb 0x%x\n",
			ddr_type, driver_data->ubwc_config ?
			driver_data->ubwc_config->highest_bank_bit : -1);
	}
exit:
	return driver_data;
}
