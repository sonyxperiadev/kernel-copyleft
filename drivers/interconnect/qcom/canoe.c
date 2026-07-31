// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 */

#include <asm/div64.h>
#include <dt-bindings/interconnect/qcom,canoe.h>
#include <linux/device.h>
#include <linux/interconnect.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sort.h>

#include "icc-rpmh.h"
#include "qnoc-qos.h"

enum {
	VOTER_IDX_HLOS,
	VOTER_IDX_CAM_IFE_0,
	VOTER_IDX_CAM_IFE_1,
	VOTER_IDX_CAM_IFE_2,
	VOTER_IDX_PCIE_CRM_HW_0,
	VOTER_IDX_DISP_CRM_SW_0,
	VOTER_IDX_DISP_CRM_HW_0,
	VOTER_IDX_DISP_CRM_HW_1,
	VOTER_IDX_DISP_CRM_HW_2,
	VOTER_IDX_DISP_CRM_HW_3,
	VOTER_IDX_DISP_CRM_HW_4,
	VOTER_IDX_DISP_CRM_HW_5,
	VOTER_IDX_DISP_CRM_HW_6,
	VOTER_IDX_DISP_CRM_HW_7,
	VOTER_IDX_DISP_CRM_HW_8,
};

static const struct regmap_config icc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
};

static struct qcom_icc_qosbox qhm_qspi_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0xc000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qhm_qspi = {
	.name = "qhm_qspi",
	.id = MASTER_QSPI_0,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qhm_qspi_qos,
	.num_links = 1,
	.links = { SLAVE_A1NOC_SNOC },
};

static struct qcom_icc_qosbox qxm_crypto_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x36000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qxm_crypto = {
	.name = "qxm_crypto",
	.id = MASTER_CRYPTO,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qxm_crypto_qos,
	.num_links = 1,
	.links = { SLAVE_A1NOC_SNOC },
};

static struct qcom_icc_qosbox qxm_qup1_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x11000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qxm_qup1 = {
	.name = "qxm_qup1",
	.id = MASTER_QUP_1,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qxm_qup1_qos,
	.num_links = 1,
	.links = { SLAVE_A1NOC_SNOC },
};

static struct qcom_icc_qosbox xm_sdc4_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0xe000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_sdc4 = {
	.name = "xm_sdc4",
	.id = MASTER_SDCC_4,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_sdc4_qos,
	.num_links = 1,
	.links = { SLAVE_A1NOC_SNOC },
};

static struct qcom_icc_qosbox xm_ufs_mem_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0xf000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_ufs_mem = {
	.name = "xm_ufs_mem",
	.id = MASTER_UFS_MEM,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_ufs_mem_qos,
	.num_links = 1,
	.links = { SLAVE_A1NOC_SNOC },
};

static struct qcom_icc_qosbox xm_usb3_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x10000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_usb3 = {
	.name = "xm_usb3",
	.id = MASTER_USB3,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_usb3_qos,
	.num_links = 1,
	.links = { SLAVE_A1NOC_SNOC },
};

static struct qcom_icc_qosbox qhm_qup2_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x35000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qhm_qup2 = {
	.name = "qhm_qup2",
	.id = MASTER_QUP_2,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qhm_qup2_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_qosbox qhm_qup3_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x3c000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qhm_qup3 = {
	.name = "qhm_qup3",
	.id = MASTER_QUP_3,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qhm_qup3_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_qosbox qhm_qup4_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x3d000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qhm_qup4 = {
	.name = "qhm_qup4",
	.id = MASTER_QUP_4,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qhm_qup4_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_qosbox qxm_ipa_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x37000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qxm_ipa = {
	.name = "qxm_ipa",
	.id = MASTER_IPA,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qxm_ipa_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_qosbox qxm_soccp_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x3b000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qxm_soccp = {
	.name = "qxm_soccp",
	.id = MASTER_SOCCP_AGGR_NOC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qxm_soccp_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_node qxm_sp = {
	.name = "qxm_sp",
	.id = MASTER_SP,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_qosbox xm_qdss_etr_0_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x38000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_qdss_etr_0 = {
	.name = "xm_qdss_etr_0",
	.id = MASTER_QDSS_ETR,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_qdss_etr_0_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_qosbox xm_qdss_etr_1_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x39000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_qdss_etr_1 = {
	.name = "xm_qdss_etr_1",
	.id = MASTER_QDSS_ETR_1,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_qdss_etr_1_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_qosbox xm_sdc2_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x3a000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_sdc2 = {
	.name = "xm_sdc2",
	.id = MASTER_SDCC_2,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_sdc2_qos,
	.num_links = 1,
	.links = { SLAVE_A2NOC_SNOC },
};

static struct qcom_icc_node qup0_core_master = {
	.name = "qup0_core_master",
	.id = MASTER_QUP_CORE_0,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_QUP_CORE_0 },
};

static struct qcom_icc_node qup1_core_master = {
	.name = "qup1_core_master",
	.id = MASTER_QUP_CORE_1,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_QUP_CORE_1 },
};

static struct qcom_icc_node qup2_core_master = {
	.name = "qup2_core_master",
	.id = MASTER_QUP_CORE_2,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_QUP_CORE_2 },
};

static struct qcom_icc_node qup3_core_master = {
	.name = "qup3_core_master",
	.id = MASTER_QUP_CORE_3,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_QUP_CORE_3 },
};

static struct qcom_icc_node qup4_core_master = {
	.name = "qup4_core_master",
	.id = MASTER_QUP_CORE_4,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_QUP_CORE_4 },
};

static struct qcom_icc_node qsm_cfg = {
	.name = "qsm_cfg",
	.id = MASTER_CNOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 35,
	.links = { SLAVE_AHB2PHY_SOUTH, SLAVE_AHB2PHY_NORTH,
		   SLAVE_CAMERA_CFG, SLAVE_CLK_CTL,
		   SLAVE_CRYPTO_0_CFG, SLAVE_DISPLAY_CFG,
		   SLAVE_EVA_CFG, SLAVE_GFX3D_CFG,
		   SLAVE_I2C, SLAVE_I3C_IBI0_CFG,
		   SLAVE_I3C_IBI1_CFG, SLAVE_IMEM_CFG,
		   SLAVE_IPC_ROUTER_CFG, SLAVE_CNOC_MSS,
		   SLAVE_PCIE_CFG, SLAVE_PRNG,
		   SLAVE_QDSS_CFG, SLAVE_QSPI_0,
		   SLAVE_QUP_1, SLAVE_QUP_2,
		   SLAVE_QUP_3, SLAVE_QUP_4,
		   SLAVE_SDCC_2, SLAVE_SDCC_4,
		   SLAVE_SPSS_CFG, SLAVE_TCSR,
		   SLAVE_TLMM, SLAVE_UFS_MEM_CFG,
		   SLAVE_USB3, SLAVE_VENUS_CFG,
		   SLAVE_VSENSE_CTRL_CFG, SLAVE_CNOC_MNOC_CFG,
		   SLAVE_PCIE_ANOC_CFG, SLAVE_QDSS_STM,
		   SLAVE_TCU },
};

static struct qcom_icc_node qnm_gemnoc_cnoc = {
	.name = "qnm_gemnoc_cnoc",
	.id = MASTER_GEM_NOC_CNOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 10,
	.links = { SLAVE_AOSS, SLAVE_IPA_CFG,
		   SLAVE_IPC_ROUTER_FENCE, SLAVE_SOCCP,
		   SLAVE_TME_CFG, SLAVE_APPSS,
		   SLAVE_CNOC_CFG, SLAVE_DDRSS_CFG,
		   SLAVE_BOOT_IMEM, SLAVE_IMEM },
};

static struct qcom_icc_node qnm_gemnoc_pcie = {
	.name = "qnm_gemnoc_pcie",
	.id = MASTER_GEM_NOC_PCIE_SNOC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_PCIE_0 },
};

static struct qcom_icc_qosbox alm_gpu_tcu_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x13d000 },
	.config = &(struct qos_config) {
		.prio = 1,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node alm_gpu_tcu = {
	.name = "alm_gpu_tcu",
	.id = MASTER_GPU_TCU,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &alm_gpu_tcu_qos,
	.num_links = 2,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC },
};

static struct qcom_icc_qosbox alm_sys_tcu_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x13f000 },
	.config = &(struct qos_config) {
		.prio = 6,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node alm_sys_tcu = {
	.name = "alm_sys_tcu",
	.id = MASTER_SYS_TCU,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &alm_sys_tcu_qos,
	.num_links = 2,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC },
};

static struct qcom_icc_node chm_apps = {
	.name = "chm_apps",
	.id = MASTER_APPSS_PROC,
	.channels = 4,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_qosbox qnm_gpu_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x31000, 0xb1000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_gpu = {
	.name = "qnm_gpu",
	.id = MASTER_GFX3D,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_gpu_qos,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_qosbox qnm_lpass_gemnoc_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x141000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_lpass_gemnoc = {
	.name = "qnm_lpass_gemnoc",
	.id = MASTER_LPASS_GEM_NOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_lpass_gemnoc_qos,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_node qnm_mdsp = {
	.name = "qnm_mdsp",
	.id = MASTER_MSS_PROC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_qosbox qnm_mnoc_hf_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x33000, 0xb3000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_mnoc_hf = {
	.name = "qnm_mnoc_hf",
	.id = MASTER_MNOC_HF_MEM_NOC,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_mnoc_hf_qos,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_qosbox qnm_mnoc_sf_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x35000, 0xb5000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_mnoc_sf = {
	.name = "qnm_mnoc_sf",
	.id = MASTER_MNOC_SF_MEM_NOC,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_mnoc_sf_qos,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_qosbox qnm_nsp_gemnoc_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x37000, 0xb7000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_nsp_gemnoc = {
	.name = "qnm_nsp_gemnoc",
	.id = MASTER_COMPUTE_NOC,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_nsp_gemnoc_qos,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_qosbox qnm_pcie_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x143000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_pcie = {
	.name = "qnm_pcie",
	.id = MASTER_ANOC_PCIE_GEM_NOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_pcie_qos,
	.num_links = 2,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC },
};

static struct qcom_icc_qosbox qnm_qpace_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x14e000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_qpace = {
	.name = "qnm_qpace",
	.id = MASTER_QPACE,
	.channels = 1,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_qpace_qos,
	.num_links = 1,
	.links = { SLAVE_LLCC },
};

static struct qcom_icc_qosbox qnm_snoc_sf_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x147000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_snoc_sf = {
	.name = "qnm_snoc_sf",
	.id = MASTER_SNOC_SF_MEM_NOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_snoc_sf_qos,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_node qnm_wlan_q6 = {
	.name = "qnm_wlan_q6",
	.id = MASTER_WLAN_Q6,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 3,
	.links = { SLAVE_GEM_NOC_CNOC, SLAVE_LLCC,
		   SLAVE_MEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_qosbox xm_gic_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x145000 },
	.config = &(struct qos_config) {
		.prio = 4,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_gic = {
	.name = "xm_gic",
	.id = MASTER_GIC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_gic_qos,
	.num_links = 1,
	.links = { SLAVE_LLCC },
};

static struct qcom_icc_node qnm_lpiaon_noc = {
	.name = "qnm_lpiaon_noc",
	.id = MASTER_LPIAON_NOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LPASS_GEM_NOC },
};

static struct qcom_icc_node qnm_lpass_lpinoc = {
	.name = "qnm_lpass_lpinoc",
	.id = MASTER_LPASS_LPINOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LPIAON_NOC_LPASS_AG_NOC },
};

static struct qcom_icc_node qnm_lpinoc_dsp_qns4m = {
	.name = "qnm_lpinoc_dsp_qns4m",
	.id = MASTER_LPASS_PROC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LPICX_NOC_LPIAON_NOC },
};

static struct qcom_icc_node llcc_mc = {
	.name = "llcc_mc",
	.id = MASTER_LLCC,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1 },
};

static struct qcom_icc_qosbox qnm_camnoc_hf_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x2a000, 0x2b000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_camnoc_hf = {
	.name = "qnm_camnoc_hf",
	.id = MASTER_CAMNOC_HF,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_camnoc_hf_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_camnoc_nrt_icp_sf_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x2c000 },
	.config = &(struct qos_config) {
		.prio = 4,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_camnoc_nrt_icp_sf = {
	.name = "qnm_camnoc_nrt_icp_sf",
	.id = MASTER_CAMNOC_NRT_ICP_SF,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_camnoc_nrt_icp_sf_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_camnoc_rt_cdm_sf_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x38000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_camnoc_rt_cdm_sf = {
	.name = "qnm_camnoc_rt_cdm_sf",
	.id = MASTER_CAMNOC_RT_CDM_SF,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_camnoc_rt_cdm_sf_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_camnoc_sf_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x2d000, 0x2e000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_camnoc_sf = {
	.name = "qnm_camnoc_sf",
	.id = MASTER_CAMNOC_SF,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_camnoc_sf_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_mdp_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x2f000, 0x30000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_mdp = {
	.name = "qnm_mdp",
	.id = MASTER_MDP,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_mdp_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_mdss_dcp_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x39000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_mdss_dcp = {
	.name = "qnm_mdss_dcp",
	.id = MASTER_MDSS_DCP,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_mdss_dcp_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_node qnm_vapss_hcp = {
	.name = "qnm_vapss_hcp",
	.id = MASTER_CDSP_HCP,
	.channels = 1,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_video_cv_cpu_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x34000 },
	.config = &(struct qos_config) {
		.prio = 4,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_video_cv_cpu = {
	.name = "qnm_video_cv_cpu",
	.id = MASTER_VIDEO_CV_PROC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_video_cv_cpu_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_video_eva_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x35000, 0x36000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_video_eva = {
	.name = "qnm_video_eva",
	.id = MASTER_VIDEO_EVA,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_video_eva_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_video_mvp_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 2,
	.offsets = { 0x32000, 0x33000 },
	.config = &(struct qos_config) {
		.prio = 0,
		.urg_fwd = 1,
		.prio_fwd_disable = 0,
	},
};

static struct qcom_icc_node qnm_video_mvp = {
	.name = "qnm_video_mvp",
	.id = MASTER_VIDEO_MVP,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_video_mvp_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_qosbox qnm_video_v_cpu_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x37000 },
	.config = &(struct qos_config) {
		.prio = 4,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_video_v_cpu = {
	.name = "qnm_video_v_cpu",
	.id = MASTER_VIDEO_V_PROC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_video_v_cpu_qos,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_node qsm_mnoc_cfg = {
	.name = "qsm_mnoc_cfg",
	.id = MASTER_CNOC_MNOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_SERVICE_MNOC },
};

static struct qcom_icc_node qnm_nsp = {
	.name = "qnm_nsp",
	.id = MASTER_CDSP_PROC,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_CDSP_MEM_NOC },
};

static struct qcom_icc_node qsm_pcie_anoc_cfg = {
	.name = "qsm_pcie_anoc_cfg",
	.id = MASTER_PCIE_ANOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_SERVICE_PCIE_ANOC },
};

static struct qcom_icc_qosbox xm_pcie_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0xb000 },
	.config = &(struct qos_config) {
		.prio = 3,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node xm_pcie = {
	.name = "xm_pcie",
	.id = MASTER_PCIE_0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &xm_pcie_qos,
	.num_links = 1,
	.links = { SLAVE_ANOC_PCIE_GEM_NOC },
};

static struct qcom_icc_node qnm_aggre1_noc = {
	.name = "qnm_aggre1_noc",
	.id = MASTER_A1NOC_SNOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_SNOC_GEM_NOC_SF },
};

static struct qcom_icc_node qnm_aggre2_noc = {
	.name = "qnm_aggre2_noc",
	.id = MASTER_A2NOC_SNOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_SNOC_GEM_NOC_SF },
};

static struct qcom_icc_qosbox qnm_apss_noc_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x1e000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 1,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_apss_noc = {
	.name = "qnm_apss_noc",
	.id = MASTER_APSS_NOC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_apss_noc_qos,
	.num_links = 1,
	.links = { SLAVE_SNOC_GEM_NOC_SF },
};

static struct qcom_icc_qosbox qnm_cnoc_data_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x1f000 },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
		.prio_fwd_disable = 1,
	},
};

static struct qcom_icc_node qnm_cnoc_data = {
	.name = "qnm_cnoc_data",
	.id = MASTER_CNOC_SNOC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_cnoc_data_qos,
	.num_links = 1,
	.links = { SLAVE_SNOC_GEM_NOC_SF },
};

static struct qcom_icc_node qnm_mnoc_hf_cam_ife_0 = {
	.name = "qnm_mnoc_hf_cam_ife_0",
	.id = MASTER_MNOC_HF_MEM_NOC_CAM_IFE_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_CAM_IFE_0 },
};

static struct qcom_icc_node qnm_mnoc_sf_cam_ife_0 = {
	.name = "qnm_mnoc_sf_cam_ife_0",
	.id = MASTER_MNOC_SF_MEM_NOC_CAM_IFE_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_CAM_IFE_0 },
};

static struct qcom_icc_node llcc_mc_cam_ife_0 = {
	.name = "llcc_mc_cam_ife_0",
	.id = MASTER_LLCC_CAM_IFE_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_CAM_IFE_0 },
};

static struct qcom_icc_node qnm_camnoc_hf_cam_ife_0 = {
	.name = "qnm_camnoc_hf_cam_ife_0",
	.id = MASTER_CAMNOC_HF_CAM_IFE_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_0 },
};

static struct qcom_icc_node qnm_camnoc_nrt_icp_sf_cam_ife_0 = {
	.name = "qnm_camnoc_nrt_icp_sf_cam_ife_0",
	.id = MASTER_CAMNOC_NRT_ICP_SF_CAM_IFE_0,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_0 },
};

static struct qcom_icc_node qnm_camnoc_rt_cdm_sf_cam_ife_0 = {
	.name = "qnm_camnoc_rt_cdm_sf_cam_ife_0",
	.id = MASTER_CAMNOC_RT_CDM_SF_CAM_IFE_0,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_0 },
};

static struct qcom_icc_node qnm_camnoc_sf_cam_ife_0 = {
	.name = "qnm_camnoc_sf_cam_ife_0",
	.id = MASTER_CAMNOC_SF_CAM_IFE_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_0 },
};

static struct qcom_icc_node qnm_mnoc_hf_cam_ife_1 = {
	.name = "qnm_mnoc_hf_cam_ife_1",
	.id = MASTER_MNOC_HF_MEM_NOC_CAM_IFE_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_CAM_IFE_1 },
};

static struct qcom_icc_node qnm_mnoc_sf_cam_ife_1 = {
	.name = "qnm_mnoc_sf_cam_ife_1",
	.id = MASTER_MNOC_SF_MEM_NOC_CAM_IFE_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_CAM_IFE_1 },
};

static struct qcom_icc_node llcc_mc_cam_ife_1 = {
	.name = "llcc_mc_cam_ife_1",
	.id = MASTER_LLCC_CAM_IFE_1,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_CAM_IFE_1 },
};

static struct qcom_icc_node qnm_camnoc_hf_cam_ife_1 = {
	.name = "qnm_camnoc_hf_cam_ife_1",
	.id = MASTER_CAMNOC_HF_CAM_IFE_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_1 },
};

static struct qcom_icc_node qnm_camnoc_nrt_icp_sf_cam_ife_1 = {
	.name = "qnm_camnoc_nrt_icp_sf_cam_ife_1",
	.id = MASTER_CAMNOC_NRT_ICP_SF_CAM_IFE_1,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_1 },
};

static struct qcom_icc_node qnm_camnoc_rt_cdm_sf_cam_ife_1 = {
	.name = "qnm_camnoc_rt_cdm_sf_cam_ife_1",
	.id = MASTER_CAMNOC_RT_CDM_SF_CAM_IFE_1,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_1 },
};

static struct qcom_icc_node qnm_camnoc_sf_cam_ife_1 = {
	.name = "qnm_camnoc_sf_cam_ife_1",
	.id = MASTER_CAMNOC_SF_CAM_IFE_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_1 },
};

static struct qcom_icc_node qnm_mnoc_hf_cam_ife_2 = {
	.name = "qnm_mnoc_hf_cam_ife_2",
	.id = MASTER_MNOC_HF_MEM_NOC_CAM_IFE_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_CAM_IFE_2 },
};

static struct qcom_icc_node qnm_mnoc_sf_cam_ife_2 = {
	.name = "qnm_mnoc_sf_cam_ife_2",
	.id = MASTER_MNOC_SF_MEM_NOC_CAM_IFE_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_CAM_IFE_2 },
};

static struct qcom_icc_node llcc_mc_cam_ife_2 = {
	.name = "llcc_mc_cam_ife_2",
	.id = MASTER_LLCC_CAM_IFE_2,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_CAM_IFE_2 },
};

static struct qcom_icc_node qnm_camnoc_hf_cam_ife_2 = {
	.name = "qnm_camnoc_hf_cam_ife_2",
	.id = MASTER_CAMNOC_HF_CAM_IFE_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_2 },
};

static struct qcom_icc_node qnm_camnoc_nrt_icp_sf_cam_ife_2 = {
	.name = "qnm_camnoc_nrt_icp_sf_cam_ife_2",
	.id = MASTER_CAMNOC_NRT_ICP_SF_CAM_IFE_2,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_2 },
};

static struct qcom_icc_node qnm_camnoc_rt_cdm_sf_cam_ife_2 = {
	.name = "qnm_camnoc_rt_cdm_sf_cam_ife_2",
	.id = MASTER_CAMNOC_RT_CDM_SF_CAM_IFE_2,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_2 },
};

static struct qcom_icc_node qnm_camnoc_sf_cam_ife_2 = {
	.name = "qnm_camnoc_sf_cam_ife_2",
	.id = MASTER_CAMNOC_SF_CAM_IFE_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_2 },
};

static struct qcom_icc_node ipa_core_master_pcie_crm_hw_0 = {
	.name = "ipa_core_master_pcie_crm_hw_0",
	.id = MASTER_IPA_CORE_PCIE_CRM_HW_0,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_IPA_CORE_PCIE_CRM_HW_0 },
};

static struct qcom_icc_node qnm_pcie_pcie_crm_hw_0 = {
	.name = "qnm_pcie_pcie_crm_hw_0",
	.id = MASTER_ANOC_PCIE_GEM_NOC_PCIE_CRM_HW_0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_PCIE_CRM_HW_0 },
};

static struct qcom_icc_node llcc_mc_pcie_crm_hw_0 = {
	.name = "llcc_mc_pcie_crm_hw_0",
	.id = MASTER_LLCC_PCIE_CRM_HW_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_PCIE_CRM_HW_0 },
};

static struct qcom_icc_node xm_pcie_pcie_crm_hw_0 = {
	.name = "xm_pcie_pcie_crm_hw_0",
	.id = MASTER_PCIE_0_PCIE_CRM_HW_0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_ANOC_PCIE_GEM_NOC_PCIE_CRM_HW_0 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_sw_0 = {
	.name = "qnm_mnoc_hf_disp_crm_sw_0",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_SW_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_SW_0 },
};

static struct qcom_icc_node llcc_mc_disp_crm_sw_0 = {
	.name = "llcc_mc_disp_crm_sw_0",
	.id = MASTER_LLCC_DISP_CRM_SW_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_SW_0 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_sw_0 = {
	.name = "qnm_mdp_disp_crm_sw_0",
	.id = MASTER_MDP_DISP_CRM_SW_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_SW_0 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_0 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_0",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_0 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_0 = {
	.name = "llcc_mc_disp_crm_hw_0",
	.id = MASTER_LLCC_DISP_CRM_HW_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_0 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_0 = {
	.name = "qnm_mdp_disp_crm_hw_0",
	.id = MASTER_MDP_DISP_CRM_HW_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_0 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_1 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_1",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_1 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_1 = {
	.name = "llcc_mc_disp_crm_hw_1",
	.id = MASTER_LLCC_DISP_CRM_HW_1,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_1 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_1 = {
	.name = "qnm_mdp_disp_crm_hw_1",
	.id = MASTER_MDP_DISP_CRM_HW_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_1 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_2 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_2",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_2 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_2 = {
	.name = "llcc_mc_disp_crm_hw_2",
	.id = MASTER_LLCC_DISP_CRM_HW_2,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_2 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_2 = {
	.name = "qnm_mdp_disp_crm_hw_2",
	.id = MASTER_MDP_DISP_CRM_HW_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_2 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_3 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_3",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_3,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_3 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_3 = {
	.name = "llcc_mc_disp_crm_hw_3",
	.id = MASTER_LLCC_DISP_CRM_HW_3,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_3 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_3 = {
	.name = "qnm_mdp_disp_crm_hw_3",
	.id = MASTER_MDP_DISP_CRM_HW_3,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_3 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_4 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_4",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_4,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_4 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_4 = {
	.name = "llcc_mc_disp_crm_hw_4",
	.id = MASTER_LLCC_DISP_CRM_HW_4,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_4 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_4 = {
	.name = "qnm_mdp_disp_crm_hw_4",
	.id = MASTER_MDP_DISP_CRM_HW_4,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_4 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_5 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_5",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_5,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_5 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_5 = {
	.name = "llcc_mc_disp_crm_hw_5",
	.id = MASTER_LLCC_DISP_CRM_HW_5,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_5 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_5 = {
	.name = "qnm_mdp_disp_crm_hw_5",
	.id = MASTER_MDP_DISP_CRM_HW_5,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_5 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_6 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_6",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_6,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_6 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_6 = {
	.name = "llcc_mc_disp_crm_hw_6",
	.id = MASTER_LLCC_DISP_CRM_HW_6,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_6 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_6 = {
	.name = "qnm_mdp_disp_crm_hw_6",
	.id = MASTER_MDP_DISP_CRM_HW_6,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_6 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_7 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_7",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_7,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_7 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_7 = {
	.name = "llcc_mc_disp_crm_hw_7",
	.id = MASTER_LLCC_DISP_CRM_HW_7,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_7 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_7 = {
	.name = "qnm_mdp_disp_crm_hw_7",
	.id = MASTER_MDP_DISP_CRM_HW_7,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_7 },
};

static struct qcom_icc_node qnm_mnoc_hf_disp_crm_hw_8 = {
	.name = "qnm_mnoc_hf_disp_crm_hw_8",
	.id = MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_8,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_LLCC_DISP_CRM_HW_8 },
};

static struct qcom_icc_node llcc_mc_disp_crm_hw_8 = {
	.name = "llcc_mc_disp_crm_hw_8",
	.id = MASTER_LLCC_DISP_CRM_HW_8,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_EBI1_DISP_CRM_HW_8 },
};

static struct qcom_icc_node qnm_mdp_disp_crm_hw_8 = {
	.name = "qnm_mdp_disp_crm_hw_8",
	.id = MASTER_MDP_DISP_CRM_HW_8,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_8 },
};


static struct qcom_icc_node qns_a1noc_snoc = {
	.name = "qns_a1noc_snoc",
	.id = SLAVE_A1NOC_SNOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_A1NOC_SNOC },
};

static struct qcom_icc_node qns_a2noc_snoc = {
	.name = "qns_a2noc_snoc",
	.id = SLAVE_A2NOC_SNOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_A2NOC_SNOC },
};

static struct qcom_icc_node qup0_core_slave = {
	.name = "qup0_core_slave",
	.id = SLAVE_QUP_CORE_0,
	.channels = 1,
	.buswidth = 4,
	.init_peak = INT_MAX,
	.init_avg  = INT_MAX,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qup1_core_slave = {
	.name = "qup1_core_slave",
	.id = SLAVE_QUP_CORE_1,
	.channels = 1,
	.buswidth = 4,
	.init_peak = INT_MAX,
	.init_avg  = INT_MAX,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qup2_core_slave = {
	.name = "qup2_core_slave",
	.id = SLAVE_QUP_CORE_2,
	.channels = 1,
	.buswidth = 4,
	.init_peak = INT_MAX,
	.init_avg  = INT_MAX,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qup3_core_slave = {
	.name = "qup3_core_slave",
	.id = SLAVE_QUP_CORE_3,
	.channels = 1,
	.buswidth = 4,
	.init_peak = INT_MAX,
	.init_avg  = INT_MAX,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qup4_core_slave = {
	.name = "qup4_core_slave",
	.id = SLAVE_QUP_CORE_4,
	.channels = 1,
	.buswidth = 4,
	.init_peak = INT_MAX,
	.init_avg  = INT_MAX,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_ahb2phy0 = {
	.name = "qhs_ahb2phy0",
	.id = SLAVE_AHB2PHY_SOUTH,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_ahb2phy1 = {
	.name = "qhs_ahb2phy1",
	.id = SLAVE_AHB2PHY_NORTH,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_camera_cfg = {
	.name = "qhs_camera_cfg",
	.id = SLAVE_CAMERA_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_clk_ctl = {
	.name = "qhs_clk_ctl",
	.id = SLAVE_CLK_CTL,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_crypto0_cfg = {
	.name = "qhs_crypto0_cfg",
	.id = SLAVE_CRYPTO_0_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_display_cfg = {
	.name = "qhs_display_cfg",
	.id = SLAVE_DISPLAY_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_eva_cfg = {
	.name = "qhs_eva_cfg",
	.id = SLAVE_EVA_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_gpuss_cfg = {
	.name = "qhs_gpuss_cfg",
	.id = SLAVE_GFX3D_CFG,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_i2c = {
	.name = "qhs_i2c",
	.id = SLAVE_I2C,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_i3c_ibi0_cfg = {
	.name = "qhs_i3c_ibi0_cfg",
	.id = SLAVE_I3C_IBI0_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_i3c_ibi1_cfg = {
	.name = "qhs_i3c_ibi1_cfg",
	.id = SLAVE_I3C_IBI1_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_imem_cfg = {
	.name = "qhs_imem_cfg",
	.id = SLAVE_IMEM_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_ipc_router = {
	.name = "qhs_ipc_router",
	.id = SLAVE_IPC_ROUTER_CFG,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_mss_cfg = {
	.name = "qhs_mss_cfg",
	.id = SLAVE_CNOC_MSS,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_pcie_cfg = {
	.name = "qhs_pcie_cfg",
	.id = SLAVE_PCIE_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_prng = {
	.name = "qhs_prng",
	.id = SLAVE_PRNG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qdss_cfg = {
	.name = "qhs_qdss_cfg",
	.id = SLAVE_QDSS_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qspi = {
	.name = "qhs_qspi",
	.id = SLAVE_QSPI_0,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qup1 = {
	.name = "qhs_qup1",
	.id = SLAVE_QUP_1,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qup2 = {
	.name = "qhs_qup2",
	.id = SLAVE_QUP_2,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qup3 = {
	.name = "qhs_qup3",
	.id = SLAVE_QUP_3,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qup4 = {
	.name = "qhs_qup4",
	.id = SLAVE_QUP_4,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_sdc2 = {
	.name = "qhs_sdc2",
	.id = SLAVE_SDCC_2,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_sdc4 = {
	.name = "qhs_sdc4",
	.id = SLAVE_SDCC_4,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_spss_cfg = {
	.name = "qhs_spss_cfg",
	.id = SLAVE_SPSS_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_tcsr = {
	.name = "qhs_tcsr",
	.id = SLAVE_TCSR,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_tlmm = {
	.name = "qhs_tlmm",
	.id = SLAVE_TLMM,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_ufs_mem_cfg = {
	.name = "qhs_ufs_mem_cfg",
	.id = SLAVE_UFS_MEM_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_usb3 = {
	.name = "qhs_usb3",
	.id = SLAVE_USB3,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_venus_cfg = {
	.name = "qhs_venus_cfg",
	.id = SLAVE_VENUS_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_vsense_ctrl_cfg = {
	.name = "qhs_vsense_ctrl_cfg",
	.id = SLAVE_VSENSE_CTRL_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qss_mnoc_cfg = {
	.name = "qss_mnoc_cfg",
	.id = SLAVE_CNOC_MNOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_CNOC_MNOC_CFG },
};

static struct qcom_icc_node qss_pcie_anoc_cfg = {
	.name = "qss_pcie_anoc_cfg",
	.id = SLAVE_PCIE_ANOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_PCIE_ANOC_CFG },
};

static struct qcom_icc_node xs_qdss_stm = {
	.name = "xs_qdss_stm",
	.id = SLAVE_QDSS_STM,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node xs_sys_tcu_cfg = {
	.name = "xs_sys_tcu_cfg",
	.id = SLAVE_TCU,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_aoss = {
	.name = "qhs_aoss",
	.id = SLAVE_AOSS,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_ipa = {
	.name = "qhs_ipa",
	.id = SLAVE_IPA_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_ipc_router_fence = {
	.name = "qhs_ipc_router_fence",
	.id = SLAVE_IPC_ROUTER_FENCE,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_soccp = {
	.name = "qhs_soccp",
	.id = SLAVE_SOCCP,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qhs_tme_cfg = {
	.name = "qhs_tme_cfg",
	.id = SLAVE_TME_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_apss = {
	.name = "qns_apss",
	.id = SLAVE_APPSS,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qss_cfg = {
	.name = "qss_cfg",
	.id = SLAVE_CNOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_CNOC_CFG },
};

static struct qcom_icc_node qss_ddrss_cfg = {
	.name = "qss_ddrss_cfg",
	.id = SLAVE_DDRSS_CFG,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qxs_boot_imem = {
	.name = "qxs_boot_imem",
	.id = SLAVE_BOOT_IMEM,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qxs_imem = {
	.name = "qxs_imem",
	.id = SLAVE_IMEM,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node xs_pcie = {
	.name = "xs_pcie",
	.id = SLAVE_PCIE_0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_gem_noc_cnoc = {
	.name = "qns_gem_noc_cnoc",
	.id = SLAVE_GEM_NOC_CNOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_GEM_NOC_CNOC },
};

static struct qcom_icc_node qns_llcc = {
	.name = "qns_llcc",
	.id = SLAVE_LLCC,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC },
};

static struct qcom_icc_node qns_pcie = {
	.name = "qns_pcie",
	.id = SLAVE_MEM_NOC_PCIE_SNOC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_GEM_NOC_PCIE_SNOC },
};

static struct qcom_icc_node qns_lpass_ag_noc_gemnoc = {
	.name = "qns_lpass_ag_noc_gemnoc",
	.id = SLAVE_LPASS_GEM_NOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LPASS_GEM_NOC },
};

static struct qcom_icc_node qns_lpass_aggnoc = {
	.name = "qns_lpass_aggnoc",
	.id = SLAVE_LPIAON_NOC_LPASS_AG_NOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LPIAON_NOC },
};

static struct qcom_icc_node qns_lpi_aon_noc = {
	.name = "qns_lpi_aon_noc",
	.id = SLAVE_LPICX_NOC_LPIAON_NOC,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LPASS_LPINOC },
};

static struct qcom_icc_node ebi = {
	.name = "ebi",
	.id = SLAVE_EBI1,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf = {
	.name = "qns_mem_noc_hf",
	.id = SLAVE_MNOC_HF_MEM_NOC,
	.channels = 2,
	.buswidth = 32,
	.init_peak = INT_MAX,
	.init_avg  = INT_MAX,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC },
};

static struct qcom_icc_node qns_mem_noc_sf = {
	.name = "qns_mem_noc_sf",
	.id = SLAVE_MNOC_SF_MEM_NOC,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_SF_MEM_NOC },
};

static struct qcom_icc_node srvc_mnoc = {
	.name = "srvc_mnoc",
	.id = SLAVE_SERVICE_MNOC,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_nsp_gemnoc = {
	.name = "qns_nsp_gemnoc",
	.id = SLAVE_CDSP_MEM_NOC,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_COMPUTE_NOC },
};

static struct qcom_icc_node qns_pcie_gemnoc = {
	.name = "qns_pcie_gemnoc",
	.id = SLAVE_ANOC_PCIE_GEM_NOC,
	.channels = 1,
	.buswidth = 16,
	.init_peak = INT_MAX,
	.init_avg = INT_MAX,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_ANOC_PCIE_GEM_NOC },
};

static struct qcom_icc_node srvc_pcie_aggre_noc = {
	.name = "srvc_pcie_aggre_noc",
	.id = SLAVE_SERVICE_PCIE_ANOC,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_gemnoc_sf = {
	.name = "qns_gemnoc_sf",
	.id = SLAVE_SNOC_GEM_NOC_SF,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_SNOC_SF_MEM_NOC },
};

static struct qcom_icc_node qns_llcc_cam_ife_0 = {
	.name = "qns_llcc_cam_ife_0",
	.id = SLAVE_LLCC_CAM_IFE_0,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_CAM_IFE_0 },
};

static struct qcom_icc_node ebi_cam_ife_0 = {
	.name = "ebi_cam_ife_0",
	.id = SLAVE_EBI1_CAM_IFE_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_cam_ife_0 = {
	.name = "qns_mem_noc_hf_cam_ife_0",
	.id = SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_CAM_IFE_0 },
};

static struct qcom_icc_node qns_mem_noc_sf_cam_ife_0 = {
	.name = "qns_mem_noc_sf_cam_ife_0",
	.id = SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_SF_MEM_NOC_CAM_IFE_0 },
};

static struct qcom_icc_node qns_llcc_cam_ife_1 = {
	.name = "qns_llcc_cam_ife_1",
	.id = SLAVE_LLCC_CAM_IFE_1,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_CAM_IFE_1 },
};

static struct qcom_icc_node ebi_cam_ife_1 = {
	.name = "ebi_cam_ife_1",
	.id = SLAVE_EBI1_CAM_IFE_1,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_cam_ife_1 = {
	.name = "qns_mem_noc_hf_cam_ife_1",
	.id = SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_CAM_IFE_1 },
};

static struct qcom_icc_node qns_mem_noc_sf_cam_ife_1 = {
	.name = "qns_mem_noc_sf_cam_ife_1",
	.id = SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_SF_MEM_NOC_CAM_IFE_1 },
};

static struct qcom_icc_node qns_llcc_cam_ife_2 = {
	.name = "qns_llcc_cam_ife_2",
	.id = SLAVE_LLCC_CAM_IFE_2,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_CAM_IFE_2 },
};

static struct qcom_icc_node ebi_cam_ife_2 = {
	.name = "ebi_cam_ife_2",
	.id = SLAVE_EBI1_CAM_IFE_2,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_cam_ife_2 = {
	.name = "qns_mem_noc_hf_cam_ife_2",
	.id = SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_CAM_IFE_2 },
};

static struct qcom_icc_node qns_mem_noc_sf_cam_ife_2 = {
	.name = "qns_mem_noc_sf_cam_ife_2",
	.id = SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_SF_MEM_NOC_CAM_IFE_2 },
};

static struct qcom_icc_node ipa_core_slave_pcie_crm_hw_0 = {
	.name = "ipa_core_slave_pcie_crm_hw_0",
	.id = SLAVE_IPA_CORE_PCIE_CRM_HW_0,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_llcc_pcie_crm_hw_0 = {
	.name = "qns_llcc_pcie_crm_hw_0",
	.id = SLAVE_LLCC_PCIE_CRM_HW_0,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_PCIE_CRM_HW_0 },
};

static struct qcom_icc_node ebi_pcie_crm_hw_0 = {
	.name = "ebi_pcie_crm_hw_0",
	.id = SLAVE_EBI1_PCIE_CRM_HW_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_pcie_gemnoc_pcie_crm_hw_0 = {
	.name = "qns_pcie_gemnoc_pcie_crm_hw_0",
	.id = SLAVE_ANOC_PCIE_GEM_NOC_PCIE_CRM_HW_0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_ANOC_PCIE_GEM_NOC_PCIE_CRM_HW_0 },
};

static struct qcom_icc_node qns_llcc_disp_crm_sw_0 = {
	.name = "qns_llcc_disp_crm_sw_0",
	.id = SLAVE_LLCC_DISP_CRM_SW_0,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_SW_0 },
};

static struct qcom_icc_node ebi_disp_crm_sw_0 = {
	.name = "ebi_disp_crm_sw_0",
	.id = SLAVE_EBI1_DISP_CRM_SW_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_sw_0 = {
	.name = "qns_mem_noc_hf_disp_crm_sw_0",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_SW_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_SW_0 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_0 = {
	.name = "qns_llcc_disp_crm_hw_0",
	.id = SLAVE_LLCC_DISP_CRM_HW_0,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_0 },
};

static struct qcom_icc_node ebi_disp_crm_hw_0 = {
	.name = "ebi_disp_crm_hw_0",
	.id = SLAVE_EBI1_DISP_CRM_HW_0,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_0 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_0",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_0,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_0 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_1 = {
	.name = "qns_llcc_disp_crm_hw_1",
	.id = SLAVE_LLCC_DISP_CRM_HW_1,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_1 },
};

static struct qcom_icc_node ebi_disp_crm_hw_1 = {
	.name = "ebi_disp_crm_hw_1",
	.id = SLAVE_EBI1_DISP_CRM_HW_1,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_1 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_1",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_1,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_1 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_2 = {
	.name = "qns_llcc_disp_crm_hw_2",
	.id = SLAVE_LLCC_DISP_CRM_HW_2,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_2 },
};

static struct qcom_icc_node ebi_disp_crm_hw_2 = {
	.name = "ebi_disp_crm_hw_2",
	.id = SLAVE_EBI1_DISP_CRM_HW_2,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_2 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_2",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_2,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_2 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_3 = {
	.name = "qns_llcc_disp_crm_hw_3",
	.id = SLAVE_LLCC_DISP_CRM_HW_3,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_3 },
};

static struct qcom_icc_node ebi_disp_crm_hw_3 = {
	.name = "ebi_disp_crm_hw_3",
	.id = SLAVE_EBI1_DISP_CRM_HW_3,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_3 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_3",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_3,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_3 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_4 = {
	.name = "qns_llcc_disp_crm_hw_4",
	.id = SLAVE_LLCC_DISP_CRM_HW_4,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_4 },
};

static struct qcom_icc_node ebi_disp_crm_hw_4 = {
	.name = "ebi_disp_crm_hw_4",
	.id = SLAVE_EBI1_DISP_CRM_HW_4,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_4 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_4",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_4,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_4 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_5 = {
	.name = "qns_llcc_disp_crm_hw_5",
	.id = SLAVE_LLCC_DISP_CRM_HW_5,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_5 },
};

static struct qcom_icc_node ebi_disp_crm_hw_5 = {
	.name = "ebi_disp_crm_hw_5",
	.id = SLAVE_EBI1_DISP_CRM_HW_5,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_5 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_5",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_5,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_5 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_6 = {
	.name = "qns_llcc_disp_crm_hw_6",
	.id = SLAVE_LLCC_DISP_CRM_HW_6,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_6 },
};

static struct qcom_icc_node ebi_disp_crm_hw_6 = {
	.name = "ebi_disp_crm_hw_6",
	.id = SLAVE_EBI1_DISP_CRM_HW_6,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_6 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_6",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_6,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_6 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_7 = {
	.name = "qns_llcc_disp_crm_hw_7",
	.id = SLAVE_LLCC_DISP_CRM_HW_7,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_7 },
};

static struct qcom_icc_node ebi_disp_crm_hw_7 = {
	.name = "ebi_disp_crm_hw_7",
	.id = SLAVE_EBI1_DISP_CRM_HW_7,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_7 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_7",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_7,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_7 },
};

static struct qcom_icc_node qns_llcc_disp_crm_hw_8 = {
	.name = "qns_llcc_disp_crm_hw_8",
	.id = SLAVE_LLCC_DISP_CRM_HW_8,
	.channels = 4,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_LLCC_DISP_CRM_HW_8 },
};

static struct qcom_icc_node ebi_disp_crm_hw_8 = {
	.name = "ebi_disp_crm_hw_8",
	.id = SLAVE_EBI1_DISP_CRM_HW_8,
	.channels = 4,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 0,
};

static struct qcom_icc_node qns_mem_noc_hf_disp_crm_hw_8 = {
	.name = "qns_mem_noc_hf_disp_crm_hw_8",
	.id = SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_8,
	.channels = 2,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.num_links = 1,
	.links = { MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_8 },
};


static struct qcom_icc_bcm bcm_acv = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_HLOS,
	.perf_mode_mask = 0x2,
	.num_nodes = 1,
	.nodes = { &ebi },
};

static struct qcom_icc_bcm bcm_ce0 = {
	.name = "CE0",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 1,
	.nodes = { &qxm_crypto },
};

static struct qcom_icc_bcm bcm_cn0 = {
	.name = "CN0",
	.voter_idx = VOTER_IDX_HLOS,
	.enable_mask = 0x1,
	.keepalive = true,
	.num_nodes = 43,
	.nodes = { &qsm_cfg, &qhs_ahb2phy0,
		   &qhs_ahb2phy1, &qhs_camera_cfg,
		   &qhs_clk_ctl, &qhs_crypto0_cfg,
		   &qhs_eva_cfg, &qhs_gpuss_cfg,
		   &qhs_i3c_ibi0_cfg, &qhs_i3c_ibi1_cfg,
		   &qhs_imem_cfg, &qhs_ipc_router,
		   &qhs_mss_cfg, &qhs_pcie_cfg,
		   &qhs_prng, &qhs_qdss_cfg,
		   &qhs_qspi, &qhs_sdc2,
		   &qhs_sdc4, &qhs_spss_cfg,
		   &qhs_tcsr, &qhs_tlmm,
		   &qhs_ufs_mem_cfg, &qhs_usb3,
		   &qhs_venus_cfg, &qhs_vsense_ctrl_cfg,
		   &qss_mnoc_cfg, &qss_pcie_anoc_cfg,
		   &xs_qdss_stm, &xs_sys_tcu_cfg,
		   &qnm_gemnoc_cnoc, &qnm_gemnoc_pcie,
		   &qhs_aoss, &qhs_ipa,
		   &qhs_ipc_router_fence, &qhs_soccp,
		   &qhs_tme_cfg, &qns_apss,
		   &qss_cfg, &qss_ddrss_cfg,
		   &qxs_boot_imem, &qxs_imem,
		   &xs_pcie },
};

static struct qcom_icc_bcm bcm_cn1 = {
	.name = "CN1",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 6,
	.nodes = { &qhs_display_cfg, &qhs_i2c,
		   &qhs_qup1, &qhs_qup2,
		   &qhs_qup3, &qhs_qup4 },
};

static struct qcom_icc_bcm bcm_co0 = {
	.name = "CO0",
	.voter_idx = VOTER_IDX_HLOS,
	.enable_mask = 0x1,
	.num_nodes = 2,
	.nodes = { &qnm_nsp, &qns_nsp_gemnoc },
};

static struct qcom_icc_bcm bcm_lp0 = {
	.name = "LP0",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 2,
	.nodes = { &qnm_lpass_lpinoc, &qns_lpass_aggnoc },
};

static struct qcom_icc_bcm bcm_mc0 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_HLOS,
	.keepalive = true,
	.num_nodes = 1,
	.nodes = { &ebi },
};

static struct qcom_icc_bcm bcm_mm0 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf },
};

static struct qcom_icc_bcm bcm_mm1 = {
	.name = "MM1",
	.voter_idx = VOTER_IDX_HLOS,
	.enable_mask = 0x1,
	.num_nodes = 9,
	.nodes = { &qnm_camnoc_hf, &qnm_camnoc_nrt_icp_sf,
		   &qnm_camnoc_rt_cdm_sf, &qnm_camnoc_sf,
		   &qnm_vapss_hcp, &qnm_video_cv_cpu,
		   &qnm_video_mvp, &qnm_video_v_cpu,
		   &qns_mem_noc_sf },
};

static struct qcom_icc_bcm bcm_qpc0 = {
	.name = "QPC0",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 1,
	.nodes = { &qnm_qpace },
};

static struct qcom_icc_bcm bcm_qup0 = {
	.name = "QUP0",
	.voter_idx = VOTER_IDX_HLOS,
	.vote_scale = 1,
	.num_nodes = 1,
	.nodes = { &qup0_core_slave },
};

static struct qcom_icc_bcm bcm_qup1 = {
	.name = "QUP1",
	.voter_idx = VOTER_IDX_HLOS,
	.vote_scale = 1,
	.num_nodes = 1,
	.nodes = { &qup1_core_slave },
};

static struct qcom_icc_bcm bcm_qup2 = {
	.name = "QUP2",
	.voter_idx = VOTER_IDX_HLOS,
	.vote_scale = 1,
	.num_nodes = 1,
	.nodes = { &qup2_core_slave },
};

static struct qcom_icc_bcm bcm_qup3 = {
	.name = "QUP3",
	.voter_idx = VOTER_IDX_HLOS,
	.vote_scale = 1,
	.num_nodes = 1,
	.nodes = { &qup3_core_slave },
};

static struct qcom_icc_bcm bcm_qup4 = {
	.name = "QUP4",
	.voter_idx = VOTER_IDX_HLOS,
	.vote_scale = 1,
	.num_nodes = 1,
	.nodes = { &qup4_core_slave },
};

static struct qcom_icc_bcm bcm_sh0 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_HLOS,
	.keepalive = true,
	.num_nodes = 1,
	.nodes = { &qns_llcc },
};

static struct qcom_icc_bcm bcm_sh1 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_HLOS,
	.enable_mask = 0x1,
	.num_nodes = 14,
	.nodes = { &alm_gpu_tcu, &alm_sys_tcu,
		   &chm_apps, &qnm_gpu,
		   &qnm_mdsp, &qnm_mnoc_hf,
		   &qnm_mnoc_sf, &qnm_nsp_gemnoc,
		   &qnm_pcie, &qnm_snoc_sf,
		   &qnm_wlan_q6, &xm_gic,
		   &qns_gem_noc_cnoc, &qns_pcie },
};

static struct qcom_icc_bcm bcm_sn0 = {
	.name = "SN0",
	.voter_idx = VOTER_IDX_HLOS,
	.keepalive = true,
	.num_nodes = 1,
	.nodes = { &qns_gemnoc_sf },
};

static struct qcom_icc_bcm bcm_sn2 = {
	.name = "SN2",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 1,
	.nodes = { &qnm_aggre1_noc },
};

static struct qcom_icc_bcm bcm_sn3 = {
	.name = "SN3",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 1,
	.nodes = { &qnm_aggre2_noc },
};

static struct qcom_icc_bcm bcm_sn4 = {
	.name = "SN4",
	.voter_idx = VOTER_IDX_HLOS,
	.num_nodes = 1,
	.nodes = { &qns_pcie_gemnoc },
};

static struct qcom_icc_bcm bcm_acv_cam_ife_0 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_CAM_IFE_0,
	.num_nodes = 1,
	.nodes = { &ebi_cam_ife_0 },
};

static struct qcom_icc_bcm bcm_mc0_cam_ife_0 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_CAM_IFE_0,
	.num_nodes = 1,
	.nodes = { &ebi_cam_ife_0 },
};

static struct qcom_icc_bcm bcm_mm0_cam_ife_0 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_CAM_IFE_0,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_cam_ife_0 },
};

static struct qcom_icc_bcm bcm_mm1_cam_ife_0 = {
	.name = "MM1",
	.voter_idx = VOTER_IDX_CAM_IFE_0,
	.enable_mask = 0x1,
	.num_nodes = 5,
	.nodes = { &qnm_camnoc_hf_cam_ife_0, &qnm_camnoc_nrt_icp_sf_cam_ife_0,
		   &qnm_camnoc_rt_cdm_sf_cam_ife_0, &qnm_camnoc_sf_cam_ife_0,
		   &qns_mem_noc_sf_cam_ife_0 },
};

static struct qcom_icc_bcm bcm_sh0_cam_ife_0 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_CAM_IFE_0,
	.num_nodes = 1,
	.nodes = { &qns_llcc_cam_ife_0 },
};

static struct qcom_icc_bcm bcm_sh1_cam_ife_0 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_CAM_IFE_0,
	.enable_mask = 0x1,
	.num_nodes = 2,
	.nodes = { &qnm_mnoc_hf_cam_ife_0, &qnm_mnoc_sf_cam_ife_0 },
};

static struct qcom_icc_bcm bcm_acv_cam_ife_1 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_CAM_IFE_1,
	.num_nodes = 1,
	.nodes = { &ebi_cam_ife_1 },
};

static struct qcom_icc_bcm bcm_mc0_cam_ife_1 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_CAM_IFE_1,
	.num_nodes = 1,
	.nodes = { &ebi_cam_ife_1 },
};

static struct qcom_icc_bcm bcm_mm0_cam_ife_1 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_CAM_IFE_1,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_cam_ife_1 },
};

static struct qcom_icc_bcm bcm_mm1_cam_ife_1 = {
	.name = "MM1",
	.voter_idx = VOTER_IDX_CAM_IFE_1,
	.enable_mask = 0x1,
	.num_nodes = 5,
	.nodes = { &qnm_camnoc_hf_cam_ife_1, &qnm_camnoc_nrt_icp_sf_cam_ife_1,
		   &qnm_camnoc_rt_cdm_sf_cam_ife_1, &qnm_camnoc_sf_cam_ife_1,
		   &qns_mem_noc_sf_cam_ife_1 },
};

static struct qcom_icc_bcm bcm_sh0_cam_ife_1 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_CAM_IFE_1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_cam_ife_1 },
};

static struct qcom_icc_bcm bcm_sh1_cam_ife_1 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_CAM_IFE_1,
	.enable_mask = 0x1,
	.num_nodes = 2,
	.nodes = { &qnm_mnoc_hf_cam_ife_1, &qnm_mnoc_sf_cam_ife_1 },
};

static struct qcom_icc_bcm bcm_acv_cam_ife_2 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_CAM_IFE_2,
	.num_nodes = 1,
	.nodes = { &ebi_cam_ife_2 },
};

static struct qcom_icc_bcm bcm_mc0_cam_ife_2 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_CAM_IFE_2,
	.num_nodes = 1,
	.nodes = { &ebi_cam_ife_2 },
};

static struct qcom_icc_bcm bcm_mm0_cam_ife_2 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_CAM_IFE_2,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_cam_ife_2 },
};

static struct qcom_icc_bcm bcm_mm1_cam_ife_2 = {
	.name = "MM1",
	.voter_idx = VOTER_IDX_CAM_IFE_2,
	.enable_mask = 0x1,
	.num_nodes = 5,
	.nodes = { &qnm_camnoc_hf_cam_ife_2, &qnm_camnoc_nrt_icp_sf_cam_ife_2,
		   &qnm_camnoc_rt_cdm_sf_cam_ife_2, &qnm_camnoc_sf_cam_ife_2,
		   &qns_mem_noc_sf_cam_ife_2 },
};

static struct qcom_icc_bcm bcm_sh0_cam_ife_2 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_CAM_IFE_2,
	.num_nodes = 1,
	.nodes = { &qns_llcc_cam_ife_2 },
};

static struct qcom_icc_bcm bcm_sh1_cam_ife_2 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_CAM_IFE_2,
	.enable_mask = 0x1,
	.num_nodes = 2,
	.nodes = { &qnm_mnoc_hf_cam_ife_2, &qnm_mnoc_sf_cam_ife_2 },
};

static struct qcom_icc_bcm bcm_acv_pcie_crm_hw_0 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_PCIE_CRM_HW_0,
	.crm_node = 5,
	.num_nodes = 1,
	.nodes = { &ebi_pcie_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_ip0_pcie_crm_hw_0 = {
	.name = "IP0",
	.voter_idx = VOTER_IDX_PCIE_CRM_HW_0,
	.crm_node = 4,
	.vote_scale = 1,
	.num_nodes = 1,
	.nodes = { &ipa_core_slave_pcie_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_mc0_pcie_crm_hw_0 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_PCIE_CRM_HW_0,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_pcie_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_sh0_pcie_crm_hw_0 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_PCIE_CRM_HW_0,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_pcie_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_sh1_pcie_crm_hw_0 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_PCIE_CRM_HW_0,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_pcie_pcie_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_sn4_pcie_crm_hw_0 = {
	.name = "SN4",
	.voter_idx = VOTER_IDX_PCIE_CRM_HW_0,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_pcie_gemnoc_pcie_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_sw_0 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_SW_0,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_sw_0 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_sw_0 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_SW_0,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_sw_0 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_sw_0 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_SW_0,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_sw_0 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_sw_0 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_SW_0,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_sw_0 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_sw_0 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_SW_0,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_sw_0 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_0 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_0,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_0 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_0,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_0 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_0,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_0 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_0,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_0 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_0,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_0 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_1 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_1,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_1 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_1 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_1,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_1 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_1 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_1,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_1 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_1 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_1,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_1 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_1 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_1,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_1 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_2 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_2,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_2 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_2 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_2,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_2 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_2 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_2,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_2 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_2 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_2,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_2 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_2 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_2,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_2 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_3 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_3,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_3 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_3 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_3,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_3 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_3 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_3,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_3 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_3 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_3,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_3 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_3 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_3,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_3 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_4 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_4,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_4 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_4 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_4,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_4 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_4 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_4,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_4 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_4 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_4,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_4 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_4 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_4,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_4 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_5 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_5,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_5 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_5 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_5,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_5 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_5 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_5,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_5 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_5 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_5,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_5 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_5 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_5,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_5 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_6 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_6,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_6 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_6 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_6,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_6 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_6 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_6,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_6 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_6 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_6,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_6 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_6 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_6,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_6 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_7 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_7,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_7 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_7 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_7,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_7 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_7 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_7,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_7 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_7 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_7,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_7 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_7 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_7,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_7 },
};

static struct qcom_icc_bcm bcm_acv_disp_crm_hw_8 = {
	.name = "ACV",
	.type = QCOM_ICC_BCM_TYPE_MASK,
	.voter_idx = VOTER_IDX_DISP_CRM_HW_8,
	.crm_node = 4,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_8 },
};

static struct qcom_icc_bcm bcm_mc0_disp_crm_hw_8 = {
	.name = "MC0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_8,
	.crm_node = 0,
	.num_nodes = 1,
	.nodes = { &ebi_disp_crm_hw_8 },
};

static struct qcom_icc_bcm bcm_mm0_disp_crm_hw_8 = {
	.name = "MM0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_8,
	.crm_node = 3,
	.num_nodes = 1,
	.nodes = { &qns_mem_noc_hf_disp_crm_hw_8 },
};

static struct qcom_icc_bcm bcm_sh0_disp_crm_hw_8 = {
	.name = "SH0",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_8,
	.crm_node = 1,
	.num_nodes = 1,
	.nodes = { &qns_llcc_disp_crm_hw_8 },
};

static struct qcom_icc_bcm bcm_sh1_disp_crm_hw_8 = {
	.name = "SH1",
	.voter_idx = VOTER_IDX_DISP_CRM_HW_8,
	.crm_node = 2,
	.enable_mask = 0x1,
	.num_nodes = 1,
	.nodes = { &qnm_mnoc_hf_disp_crm_hw_8 },
};

static struct qcom_icc_bcm *aggre_noc_bcms[] = {
	&bcm_ce0,
};

static struct qcom_icc_node *aggre_noc_nodes[] = {
	[MASTER_QSPI_0] = &qhm_qspi,
	[MASTER_CRYPTO] = &qxm_crypto,
	[MASTER_QUP_1] = &qxm_qup1,
	[MASTER_SDCC_4] = &xm_sdc4,
	[MASTER_UFS_MEM] = &xm_ufs_mem,
	[MASTER_USB3] = &xm_usb3,
	[SLAVE_A1NOC_SNOC] = &qns_a1noc_snoc,
	[MASTER_QUP_2] = &qhm_qup2,
	[MASTER_QUP_3] = &qhm_qup3,
	[MASTER_QUP_4] = &qhm_qup4,
	[MASTER_IPA] = &qxm_ipa,
	[MASTER_SOCCP_AGGR_NOC] = &qxm_soccp,
	[MASTER_SP] = &qxm_sp,
	[MASTER_QDSS_ETR] = &xm_qdss_etr_0,
	[MASTER_QDSS_ETR_1] = &xm_qdss_etr_1,
	[MASTER_SDCC_2] = &xm_sdc2,
	[SLAVE_A2NOC_SNOC] = &qns_a2noc_snoc,
};

static char *aggre_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_aggre_noc = {
	.config = &icc_regmap_config,
	.nodes = aggre_noc_nodes,
	.num_nodes = ARRAY_SIZE(aggre_noc_nodes),
	.bcms = aggre_noc_bcms,
	.num_bcms = ARRAY_SIZE(aggre_noc_bcms),
	.voters = aggre_noc_voters,
	.num_voters = ARRAY_SIZE(aggre_noc_voters),
};


static struct qcom_icc_bcm *clk_virt_bcms[] = {
	&bcm_qup0,
	&bcm_qup1,
	&bcm_qup2,
	&bcm_qup3,
	&bcm_qup4,
	&bcm_ip0_pcie_crm_hw_0,
};

static struct qcom_icc_node *clk_virt_nodes[] = {
	[MASTER_QUP_CORE_0] = &qup0_core_master,
	[MASTER_QUP_CORE_1] = &qup1_core_master,
	[MASTER_QUP_CORE_2] = &qup2_core_master,
	[MASTER_QUP_CORE_3] = &qup3_core_master,
	[MASTER_QUP_CORE_4] = &qup4_core_master,
	[SLAVE_QUP_CORE_0] = &qup0_core_slave,
	[SLAVE_QUP_CORE_1] = &qup1_core_slave,
	[SLAVE_QUP_CORE_2] = &qup2_core_slave,
	[SLAVE_QUP_CORE_3] = &qup3_core_slave,
	[SLAVE_QUP_CORE_4] = &qup4_core_slave,
	[MASTER_IPA_CORE_PCIE_CRM_HW_0] = &ipa_core_master_pcie_crm_hw_0,
	[SLAVE_IPA_CORE_PCIE_CRM_HW_0] = &ipa_core_slave_pcie_crm_hw_0,
};

static char *clk_virt_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
	[VOTER_IDX_PCIE_CRM_HW_0] = "pcie_crm_hw_0",
};

static struct qcom_icc_desc canoe_clk_virt = {
	.config = &icc_regmap_config,
	.nodes = clk_virt_nodes,
	.num_nodes = ARRAY_SIZE(clk_virt_nodes),
	.bcms = clk_virt_bcms,
	.num_bcms = ARRAY_SIZE(clk_virt_bcms),
	.voters = clk_virt_voters,
	.num_voters = ARRAY_SIZE(clk_virt_voters),
};

static struct qcom_icc_bcm *cnoc_cfg_bcms[] = {
	&bcm_cn0,
	&bcm_cn1,
};

static struct qcom_icc_node *cnoc_cfg_nodes[] = {
	[MASTER_CNOC_CFG] = &qsm_cfg,
	[SLAVE_AHB2PHY_SOUTH] = &qhs_ahb2phy0,
	[SLAVE_AHB2PHY_NORTH] = &qhs_ahb2phy1,
	[SLAVE_CAMERA_CFG] = &qhs_camera_cfg,
	[SLAVE_CLK_CTL] = &qhs_clk_ctl,
	[SLAVE_CRYPTO_0_CFG] = &qhs_crypto0_cfg,
	[SLAVE_DISPLAY_CFG] = &qhs_display_cfg,
	[SLAVE_EVA_CFG] = &qhs_eva_cfg,
	[SLAVE_GFX3D_CFG] = &qhs_gpuss_cfg,
	[SLAVE_I2C] = &qhs_i2c,
	[SLAVE_I3C_IBI0_CFG] = &qhs_i3c_ibi0_cfg,
	[SLAVE_I3C_IBI1_CFG] = &qhs_i3c_ibi1_cfg,
	[SLAVE_IMEM_CFG] = &qhs_imem_cfg,
	[SLAVE_IPC_ROUTER_CFG] = &qhs_ipc_router,
	[SLAVE_CNOC_MSS] = &qhs_mss_cfg,
	[SLAVE_PCIE_CFG] = &qhs_pcie_cfg,
	[SLAVE_PRNG] = &qhs_prng,
	[SLAVE_QDSS_CFG] = &qhs_qdss_cfg,
	[SLAVE_QSPI_0] = &qhs_qspi,
	[SLAVE_QUP_1] = &qhs_qup1,
	[SLAVE_QUP_2] = &qhs_qup2,
	[SLAVE_QUP_3] = &qhs_qup3,
	[SLAVE_QUP_4] = &qhs_qup4,
	[SLAVE_SDCC_2] = &qhs_sdc2,
	[SLAVE_SDCC_4] = &qhs_sdc4,
	[SLAVE_SPSS_CFG] = &qhs_spss_cfg,
	[SLAVE_TCSR] = &qhs_tcsr,
	[SLAVE_TLMM] = &qhs_tlmm,
	[SLAVE_UFS_MEM_CFG] = &qhs_ufs_mem_cfg,
	[SLAVE_USB3] = &qhs_usb3,
	[SLAVE_VENUS_CFG] = &qhs_venus_cfg,
	[SLAVE_VSENSE_CTRL_CFG] = &qhs_vsense_ctrl_cfg,
	[SLAVE_CNOC_MNOC_CFG] = &qss_mnoc_cfg,
	[SLAVE_PCIE_ANOC_CFG] = &qss_pcie_anoc_cfg,
	[SLAVE_QDSS_STM] = &xs_qdss_stm,
	[SLAVE_TCU] = &xs_sys_tcu_cfg,
};

static char *cnoc_cfg_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_cnoc_cfg = {
	.config = &icc_regmap_config,
	.nodes = cnoc_cfg_nodes,
	.num_nodes = ARRAY_SIZE(cnoc_cfg_nodes),
	.bcms = cnoc_cfg_bcms,
	.num_bcms = ARRAY_SIZE(cnoc_cfg_bcms),
	.voters = cnoc_cfg_voters,
	.num_voters = ARRAY_SIZE(cnoc_cfg_voters),
};

static struct qcom_icc_bcm *cnoc_main_bcms[] = {
	&bcm_cn0,
};

static struct qcom_icc_node *cnoc_main_nodes[] = {
	[MASTER_GEM_NOC_CNOC] = &qnm_gemnoc_cnoc,
	[MASTER_GEM_NOC_PCIE_SNOC] = &qnm_gemnoc_pcie,
	[SLAVE_AOSS] = &qhs_aoss,
	[SLAVE_IPA_CFG] = &qhs_ipa,
	[SLAVE_IPC_ROUTER_FENCE] = &qhs_ipc_router_fence,
	[SLAVE_SOCCP] = &qhs_soccp,
	[SLAVE_TME_CFG] = &qhs_tme_cfg,
	[SLAVE_APPSS] = &qns_apss,
	[SLAVE_CNOC_CFG] = &qss_cfg,
	[SLAVE_DDRSS_CFG] = &qss_ddrss_cfg,
	[SLAVE_BOOT_IMEM] = &qxs_boot_imem,
	[SLAVE_IMEM] = &qxs_imem,
	[SLAVE_PCIE_0] = &xs_pcie,
};

static char *cnoc_main_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_cnoc_main = {
	.config = &icc_regmap_config,
	.nodes = cnoc_main_nodes,
	.num_nodes = ARRAY_SIZE(cnoc_main_nodes),
	.bcms = cnoc_main_bcms,
	.num_bcms = ARRAY_SIZE(cnoc_main_bcms),
	.voters = cnoc_main_voters,
	.num_voters = ARRAY_SIZE(cnoc_main_voters),
};

static struct qcom_icc_bcm *gem_noc_bcms[] = {
	&bcm_qpc0,
	&bcm_sh0,
	&bcm_sh1,
	&bcm_sh0_cam_ife_0,
	&bcm_sh1_cam_ife_0,
	&bcm_sh0_cam_ife_1,
	&bcm_sh1_cam_ife_1,
	&bcm_sh0_cam_ife_2,
	&bcm_sh1_cam_ife_2,
	&bcm_sh0_pcie_crm_hw_0,
	&bcm_sh1_pcie_crm_hw_0,
	&bcm_sh0_disp_crm_sw_0,
	&bcm_sh1_disp_crm_sw_0,
	&bcm_sh0_disp_crm_hw_0,
	&bcm_sh1_disp_crm_hw_0,
	&bcm_sh0_disp_crm_hw_1,
	&bcm_sh1_disp_crm_hw_1,
	&bcm_sh0_disp_crm_hw_2,
	&bcm_sh1_disp_crm_hw_2,
	&bcm_sh0_disp_crm_hw_3,
	&bcm_sh1_disp_crm_hw_3,
	&bcm_sh0_disp_crm_hw_4,
	&bcm_sh1_disp_crm_hw_4,
	&bcm_sh0_disp_crm_hw_5,
	&bcm_sh1_disp_crm_hw_5,
	&bcm_sh0_disp_crm_hw_6,
	&bcm_sh1_disp_crm_hw_6,
	&bcm_sh0_disp_crm_hw_7,
	&bcm_sh1_disp_crm_hw_7,
	&bcm_sh0_disp_crm_hw_8,
	&bcm_sh1_disp_crm_hw_8,
};

static struct qcom_icc_node *gem_noc_nodes[] = {
	[MASTER_GPU_TCU] = &alm_gpu_tcu,
	[MASTER_SYS_TCU] = &alm_sys_tcu,
	[MASTER_APPSS_PROC] = &chm_apps,
	[MASTER_GFX3D] = &qnm_gpu,
	[MASTER_LPASS_GEM_NOC] = &qnm_lpass_gemnoc,
	[MASTER_MSS_PROC] = &qnm_mdsp,
	[MASTER_MNOC_HF_MEM_NOC] = &qnm_mnoc_hf,
	[MASTER_MNOC_SF_MEM_NOC] = &qnm_mnoc_sf,
	[MASTER_COMPUTE_NOC] = &qnm_nsp_gemnoc,
	[MASTER_ANOC_PCIE_GEM_NOC] = &qnm_pcie,
	[MASTER_QPACE] = &qnm_qpace,
	[MASTER_SNOC_SF_MEM_NOC] = &qnm_snoc_sf,
	[MASTER_WLAN_Q6] = &qnm_wlan_q6,
	[MASTER_GIC] = &xm_gic,
	[SLAVE_GEM_NOC_CNOC] = &qns_gem_noc_cnoc,
	[SLAVE_LLCC] = &qns_llcc,
	[SLAVE_MEM_NOC_PCIE_SNOC] = &qns_pcie,
	[MASTER_MNOC_HF_MEM_NOC_CAM_IFE_0] = &qnm_mnoc_hf_cam_ife_0,
	[MASTER_MNOC_SF_MEM_NOC_CAM_IFE_0] = &qnm_mnoc_sf_cam_ife_0,
	[SLAVE_LLCC_CAM_IFE_0] = &qns_llcc_cam_ife_0,
	[MASTER_MNOC_HF_MEM_NOC_CAM_IFE_1] = &qnm_mnoc_hf_cam_ife_1,
	[MASTER_MNOC_SF_MEM_NOC_CAM_IFE_1] = &qnm_mnoc_sf_cam_ife_1,
	[SLAVE_LLCC_CAM_IFE_1] = &qns_llcc_cam_ife_1,
	[MASTER_MNOC_HF_MEM_NOC_CAM_IFE_2] = &qnm_mnoc_hf_cam_ife_2,
	[MASTER_MNOC_SF_MEM_NOC_CAM_IFE_2] = &qnm_mnoc_sf_cam_ife_2,
	[SLAVE_LLCC_CAM_IFE_2] = &qns_llcc_cam_ife_2,
	[MASTER_ANOC_PCIE_GEM_NOC_PCIE_CRM_HW_0] = &qnm_pcie_pcie_crm_hw_0,
	[SLAVE_LLCC_PCIE_CRM_HW_0] = &qns_llcc_pcie_crm_hw_0,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_SW_0] = &qnm_mnoc_hf_disp_crm_sw_0,
	[SLAVE_LLCC_DISP_CRM_SW_0] = &qns_llcc_disp_crm_sw_0,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_0] = &qnm_mnoc_hf_disp_crm_hw_0,
	[SLAVE_LLCC_DISP_CRM_HW_0] = &qns_llcc_disp_crm_hw_0,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_1] = &qnm_mnoc_hf_disp_crm_hw_1,
	[SLAVE_LLCC_DISP_CRM_HW_1] = &qns_llcc_disp_crm_hw_1,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_2] = &qnm_mnoc_hf_disp_crm_hw_2,
	[SLAVE_LLCC_DISP_CRM_HW_2] = &qns_llcc_disp_crm_hw_2,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_3] = &qnm_mnoc_hf_disp_crm_hw_3,
	[SLAVE_LLCC_DISP_CRM_HW_3] = &qns_llcc_disp_crm_hw_3,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_4] = &qnm_mnoc_hf_disp_crm_hw_4,
	[SLAVE_LLCC_DISP_CRM_HW_4] = &qns_llcc_disp_crm_hw_4,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_5] = &qnm_mnoc_hf_disp_crm_hw_5,
	[SLAVE_LLCC_DISP_CRM_HW_5] = &qns_llcc_disp_crm_hw_5,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_6] = &qnm_mnoc_hf_disp_crm_hw_6,
	[SLAVE_LLCC_DISP_CRM_HW_6] = &qns_llcc_disp_crm_hw_6,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_7] = &qnm_mnoc_hf_disp_crm_hw_7,
	[SLAVE_LLCC_DISP_CRM_HW_7] = &qns_llcc_disp_crm_hw_7,
	[MASTER_MNOC_HF_MEM_NOC_DISP_CRM_HW_8] = &qnm_mnoc_hf_disp_crm_hw_8,
	[SLAVE_LLCC_DISP_CRM_HW_8] = &qns_llcc_disp_crm_hw_8,
};

static char *gem_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
	[VOTER_IDX_CAM_IFE_0] = "cam_ife_0",
	[VOTER_IDX_CAM_IFE_1] = "cam_ife_1",
	[VOTER_IDX_CAM_IFE_2] = "cam_ife_2",
	[VOTER_IDX_PCIE_CRM_HW_0] = "pcie_crm_hw_0",
	[VOTER_IDX_DISP_CRM_SW_0] = "disp_crm_sw_0",
	[VOTER_IDX_DISP_CRM_HW_0] = "disp_crm_hw_0",
	[VOTER_IDX_DISP_CRM_HW_1] = "disp_crm_hw_1",
	[VOTER_IDX_DISP_CRM_HW_2] = "disp_crm_hw_2",
	[VOTER_IDX_DISP_CRM_HW_3] = "disp_crm_hw_3",
	[VOTER_IDX_DISP_CRM_HW_4] = "disp_crm_hw_4",
	[VOTER_IDX_DISP_CRM_HW_5] = "disp_crm_hw_5",
	[VOTER_IDX_DISP_CRM_HW_6] = "disp_crm_hw_6",
	[VOTER_IDX_DISP_CRM_HW_7] = "disp_crm_hw_7",
	[VOTER_IDX_DISP_CRM_HW_8] = "disp_crm_hw_8",
};

static struct qcom_icc_desc canoe_gem_noc = {
	.config = &icc_regmap_config,
	.nodes = gem_noc_nodes,
	.num_nodes = ARRAY_SIZE(gem_noc_nodes),
	.bcms = gem_noc_bcms,
	.num_bcms = ARRAY_SIZE(gem_noc_bcms),
	.voters = gem_noc_voters,
	.num_voters = ARRAY_SIZE(gem_noc_voters),
};

static struct qcom_icc_bcm *lpass_ag_noc_bcms[] = {
};

static struct qcom_icc_node *lpass_ag_noc_nodes[] = {
	[MASTER_LPIAON_NOC] = &qnm_lpiaon_noc,
	[SLAVE_LPASS_GEM_NOC] = &qns_lpass_ag_noc_gemnoc,
};

static char *lpass_ag_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_lpass_ag_noc = {
	.config = &icc_regmap_config,
	.nodes = lpass_ag_noc_nodes,
	.num_nodes = ARRAY_SIZE(lpass_ag_noc_nodes),
	.bcms = lpass_ag_noc_bcms,
	.num_bcms = ARRAY_SIZE(lpass_ag_noc_bcms),
	.voters = lpass_ag_noc_voters,
	.num_voters = ARRAY_SIZE(lpass_ag_noc_voters),
};

static struct qcom_icc_bcm *lpass_lpiaon_noc_bcms[] = {
	&bcm_lp0,
};

static struct qcom_icc_node *lpass_lpiaon_noc_nodes[] = {
	[MASTER_LPASS_LPINOC] = &qnm_lpass_lpinoc,
	[SLAVE_LPIAON_NOC_LPASS_AG_NOC] = &qns_lpass_aggnoc,
};

static char *lpass_lpiaon_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_lpass_lpiaon_noc = {
	.config = &icc_regmap_config,
	.nodes = lpass_lpiaon_noc_nodes,
	.num_nodes = ARRAY_SIZE(lpass_lpiaon_noc_nodes),
	.bcms = lpass_lpiaon_noc_bcms,
	.num_bcms = ARRAY_SIZE(lpass_lpiaon_noc_bcms),
	.voters = lpass_lpiaon_noc_voters,
	.num_voters = ARRAY_SIZE(lpass_lpiaon_noc_voters),
};

static struct qcom_icc_bcm *lpass_lpicx_noc_bcms[] = {
};

static struct qcom_icc_node *lpass_lpicx_noc_nodes[] = {
	[MASTER_LPASS_PROC] = &qnm_lpinoc_dsp_qns4m,
	[SLAVE_LPICX_NOC_LPIAON_NOC] = &qns_lpi_aon_noc,
};

static char *lpass_lpicx_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_lpass_lpicx_noc = {
	.config = &icc_regmap_config,
	.nodes = lpass_lpicx_noc_nodes,
	.num_nodes = ARRAY_SIZE(lpass_lpicx_noc_nodes),
	.bcms = lpass_lpicx_noc_bcms,
	.num_bcms = ARRAY_SIZE(lpass_lpicx_noc_bcms),
	.voters = lpass_lpicx_noc_voters,
	.num_voters = ARRAY_SIZE(lpass_lpicx_noc_voters),
};

static struct qcom_icc_bcm *mc_virt_bcms[] = {
	&bcm_acv,
	&bcm_mc0,
	&bcm_acv_cam_ife_0,
	&bcm_mc0_cam_ife_0,
	&bcm_acv_cam_ife_1,
	&bcm_mc0_cam_ife_1,
	&bcm_acv_cam_ife_2,
	&bcm_mc0_cam_ife_2,
	&bcm_acv_pcie_crm_hw_0,
	&bcm_mc0_pcie_crm_hw_0,
	&bcm_acv_disp_crm_sw_0,
	&bcm_mc0_disp_crm_sw_0,
	&bcm_acv_disp_crm_hw_0,
	&bcm_mc0_disp_crm_hw_0,
	&bcm_acv_disp_crm_hw_1,
	&bcm_mc0_disp_crm_hw_1,
	&bcm_acv_disp_crm_hw_2,
	&bcm_mc0_disp_crm_hw_2,
	&bcm_acv_disp_crm_hw_3,
	&bcm_mc0_disp_crm_hw_3,
	&bcm_acv_disp_crm_hw_4,
	&bcm_mc0_disp_crm_hw_4,
	&bcm_acv_disp_crm_hw_5,
	&bcm_mc0_disp_crm_hw_5,
	&bcm_acv_disp_crm_hw_6,
	&bcm_mc0_disp_crm_hw_6,
	&bcm_acv_disp_crm_hw_7,
	&bcm_mc0_disp_crm_hw_7,
	&bcm_acv_disp_crm_hw_8,
	&bcm_mc0_disp_crm_hw_8,
};

static struct qcom_icc_node *mc_virt_nodes[] = {
	[MASTER_LLCC] = &llcc_mc,
	[SLAVE_EBI1] = &ebi,
	[MASTER_LLCC_CAM_IFE_0] = &llcc_mc_cam_ife_0,
	[SLAVE_EBI1_CAM_IFE_0] = &ebi_cam_ife_0,
	[MASTER_LLCC_CAM_IFE_1] = &llcc_mc_cam_ife_1,
	[SLAVE_EBI1_CAM_IFE_1] = &ebi_cam_ife_1,
	[MASTER_LLCC_CAM_IFE_2] = &llcc_mc_cam_ife_2,
	[SLAVE_EBI1_CAM_IFE_2] = &ebi_cam_ife_2,
	[MASTER_LLCC_PCIE_CRM_HW_0] = &llcc_mc_pcie_crm_hw_0,
	[SLAVE_EBI1_PCIE_CRM_HW_0] = &ebi_pcie_crm_hw_0,
	[MASTER_LLCC_DISP_CRM_SW_0] = &llcc_mc_disp_crm_sw_0,
	[SLAVE_EBI1_DISP_CRM_SW_0] = &ebi_disp_crm_sw_0,
	[MASTER_LLCC_DISP_CRM_HW_0] = &llcc_mc_disp_crm_hw_0,
	[SLAVE_EBI1_DISP_CRM_HW_0] = &ebi_disp_crm_hw_0,
	[MASTER_LLCC_DISP_CRM_HW_1] = &llcc_mc_disp_crm_hw_1,
	[SLAVE_EBI1_DISP_CRM_HW_1] = &ebi_disp_crm_hw_1,
	[MASTER_LLCC_DISP_CRM_HW_2] = &llcc_mc_disp_crm_hw_2,
	[SLAVE_EBI1_DISP_CRM_HW_2] = &ebi_disp_crm_hw_2,
	[MASTER_LLCC_DISP_CRM_HW_3] = &llcc_mc_disp_crm_hw_3,
	[SLAVE_EBI1_DISP_CRM_HW_3] = &ebi_disp_crm_hw_3,
	[MASTER_LLCC_DISP_CRM_HW_4] = &llcc_mc_disp_crm_hw_4,
	[SLAVE_EBI1_DISP_CRM_HW_4] = &ebi_disp_crm_hw_4,
	[MASTER_LLCC_DISP_CRM_HW_5] = &llcc_mc_disp_crm_hw_5,
	[SLAVE_EBI1_DISP_CRM_HW_5] = &ebi_disp_crm_hw_5,
	[MASTER_LLCC_DISP_CRM_HW_6] = &llcc_mc_disp_crm_hw_6,
	[SLAVE_EBI1_DISP_CRM_HW_6] = &ebi_disp_crm_hw_6,
	[MASTER_LLCC_DISP_CRM_HW_7] = &llcc_mc_disp_crm_hw_7,
	[SLAVE_EBI1_DISP_CRM_HW_7] = &ebi_disp_crm_hw_7,
	[MASTER_LLCC_DISP_CRM_HW_8] = &llcc_mc_disp_crm_hw_8,
	[SLAVE_EBI1_DISP_CRM_HW_8] = &ebi_disp_crm_hw_8,
};

static char *mc_virt_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
	[VOTER_IDX_CAM_IFE_0] = "cam_ife_0",
	[VOTER_IDX_CAM_IFE_1] = "cam_ife_1",
	[VOTER_IDX_CAM_IFE_2] = "cam_ife_2",
	[VOTER_IDX_PCIE_CRM_HW_0] = "pcie_crm_hw_0",
	[VOTER_IDX_DISP_CRM_SW_0] = "disp_crm_sw_0",
	[VOTER_IDX_DISP_CRM_HW_0] = "disp_crm_hw_0",
	[VOTER_IDX_DISP_CRM_HW_1] = "disp_crm_hw_1",
	[VOTER_IDX_DISP_CRM_HW_2] = "disp_crm_hw_2",
	[VOTER_IDX_DISP_CRM_HW_3] = "disp_crm_hw_3",
	[VOTER_IDX_DISP_CRM_HW_4] = "disp_crm_hw_4",
	[VOTER_IDX_DISP_CRM_HW_5] = "disp_crm_hw_5",
	[VOTER_IDX_DISP_CRM_HW_6] = "disp_crm_hw_6",
	[VOTER_IDX_DISP_CRM_HW_7] = "disp_crm_hw_7",
	[VOTER_IDX_DISP_CRM_HW_8] = "disp_crm_hw_8",
};

static struct qcom_icc_desc canoe_mc_virt = {
	.config = &icc_regmap_config,
	.nodes = mc_virt_nodes,
	.num_nodes = ARRAY_SIZE(mc_virt_nodes),
	.bcms = mc_virt_bcms,
	.num_bcms = ARRAY_SIZE(mc_virt_bcms),
	.voters = mc_virt_voters,
	.num_voters = ARRAY_SIZE(mc_virt_voters),
};

static struct qcom_icc_bcm *mmss_noc_bcms[] = {
	&bcm_mm0,
	&bcm_mm1,
	&bcm_mm0_cam_ife_0,
	&bcm_mm1_cam_ife_0,
	&bcm_mm0_cam_ife_1,
	&bcm_mm1_cam_ife_1,
	&bcm_mm0_cam_ife_2,
	&bcm_mm1_cam_ife_2,
	&bcm_mm0_disp_crm_sw_0,
	&bcm_mm0_disp_crm_hw_0,
	&bcm_mm0_disp_crm_hw_1,
	&bcm_mm0_disp_crm_hw_2,
	&bcm_mm0_disp_crm_hw_3,
	&bcm_mm0_disp_crm_hw_4,
	&bcm_mm0_disp_crm_hw_5,
	&bcm_mm0_disp_crm_hw_6,
	&bcm_mm0_disp_crm_hw_7,
	&bcm_mm0_disp_crm_hw_8,
};

static struct qcom_icc_node *mmss_noc_nodes[] = {
	[MASTER_CAMNOC_HF] = &qnm_camnoc_hf,
	[MASTER_CAMNOC_NRT_ICP_SF] = &qnm_camnoc_nrt_icp_sf,
	[MASTER_CAMNOC_RT_CDM_SF] = &qnm_camnoc_rt_cdm_sf,
	[MASTER_CAMNOC_SF] = &qnm_camnoc_sf,
	[MASTER_MDP] = &qnm_mdp,
	[MASTER_MDSS_DCP] = &qnm_mdss_dcp,
	[MASTER_CDSP_HCP] = &qnm_vapss_hcp,
	[MASTER_VIDEO_CV_PROC] = &qnm_video_cv_cpu,
	[MASTER_VIDEO_EVA] = &qnm_video_eva,
	[MASTER_VIDEO_MVP] = &qnm_video_mvp,
	[MASTER_VIDEO_V_PROC] = &qnm_video_v_cpu,
	[MASTER_CNOC_MNOC_CFG] = &qsm_mnoc_cfg,
	[SLAVE_MNOC_HF_MEM_NOC] = &qns_mem_noc_hf,
	[SLAVE_MNOC_SF_MEM_NOC] = &qns_mem_noc_sf,
	[SLAVE_SERVICE_MNOC] = &srvc_mnoc,
	[MASTER_CAMNOC_HF_CAM_IFE_0] = &qnm_camnoc_hf_cam_ife_0,
	[MASTER_CAMNOC_NRT_ICP_SF_CAM_IFE_0] = &qnm_camnoc_nrt_icp_sf_cam_ife_0,
	[MASTER_CAMNOC_RT_CDM_SF_CAM_IFE_0] = &qnm_camnoc_rt_cdm_sf_cam_ife_0,
	[MASTER_CAMNOC_SF_CAM_IFE_0] = &qnm_camnoc_sf_cam_ife_0,
	[SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_0] = &qns_mem_noc_hf_cam_ife_0,
	[SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_0] = &qns_mem_noc_sf_cam_ife_0,
	[MASTER_CAMNOC_HF_CAM_IFE_1] = &qnm_camnoc_hf_cam_ife_1,
	[MASTER_CAMNOC_NRT_ICP_SF_CAM_IFE_1] = &qnm_camnoc_nrt_icp_sf_cam_ife_1,
	[MASTER_CAMNOC_RT_CDM_SF_CAM_IFE_1] = &qnm_camnoc_rt_cdm_sf_cam_ife_1,
	[MASTER_CAMNOC_SF_CAM_IFE_1] = &qnm_camnoc_sf_cam_ife_1,
	[SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_1] = &qns_mem_noc_hf_cam_ife_1,
	[SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_1] = &qns_mem_noc_sf_cam_ife_1,
	[MASTER_CAMNOC_HF_CAM_IFE_2] = &qnm_camnoc_hf_cam_ife_2,
	[MASTER_CAMNOC_NRT_ICP_SF_CAM_IFE_2] = &qnm_camnoc_nrt_icp_sf_cam_ife_2,
	[MASTER_CAMNOC_RT_CDM_SF_CAM_IFE_2] = &qnm_camnoc_rt_cdm_sf_cam_ife_2,
	[MASTER_CAMNOC_SF_CAM_IFE_2] = &qnm_camnoc_sf_cam_ife_2,
	[SLAVE_MNOC_HF_MEM_NOC_CAM_IFE_2] = &qns_mem_noc_hf_cam_ife_2,
	[SLAVE_MNOC_SF_MEM_NOC_CAM_IFE_2] = &qns_mem_noc_sf_cam_ife_2,
	[MASTER_MDP_DISP_CRM_SW_0] = &qnm_mdp_disp_crm_sw_0,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_SW_0] = &qns_mem_noc_hf_disp_crm_sw_0,
	[MASTER_MDP_DISP_CRM_HW_0] = &qnm_mdp_disp_crm_hw_0,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_0] = &qns_mem_noc_hf_disp_crm_hw_0,
	[MASTER_MDP_DISP_CRM_HW_1] = &qnm_mdp_disp_crm_hw_1,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_1] = &qns_mem_noc_hf_disp_crm_hw_1,
	[MASTER_MDP_DISP_CRM_HW_2] = &qnm_mdp_disp_crm_hw_2,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_2] = &qns_mem_noc_hf_disp_crm_hw_2,
	[MASTER_MDP_DISP_CRM_HW_3] = &qnm_mdp_disp_crm_hw_3,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_3] = &qns_mem_noc_hf_disp_crm_hw_3,
	[MASTER_MDP_DISP_CRM_HW_4] = &qnm_mdp_disp_crm_hw_4,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_4] = &qns_mem_noc_hf_disp_crm_hw_4,
	[MASTER_MDP_DISP_CRM_HW_5] = &qnm_mdp_disp_crm_hw_5,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_5] = &qns_mem_noc_hf_disp_crm_hw_5,
	[MASTER_MDP_DISP_CRM_HW_6] = &qnm_mdp_disp_crm_hw_6,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_6] = &qns_mem_noc_hf_disp_crm_hw_6,
	[MASTER_MDP_DISP_CRM_HW_7] = &qnm_mdp_disp_crm_hw_7,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_7] = &qns_mem_noc_hf_disp_crm_hw_7,
	[MASTER_MDP_DISP_CRM_HW_8] = &qnm_mdp_disp_crm_hw_8,
	[SLAVE_MNOC_HF_MEM_NOC_DISP_CRM_HW_8] = &qns_mem_noc_hf_disp_crm_hw_8,
};

static char *mmss_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
	[VOTER_IDX_CAM_IFE_0] = "cam_ife_0",
	[VOTER_IDX_CAM_IFE_1] = "cam_ife_1",
	[VOTER_IDX_CAM_IFE_2] = "cam_ife_2",
	[VOTER_IDX_DISP_CRM_SW_0] = "disp_crm_sw_0",
	[VOTER_IDX_DISP_CRM_HW_0] = "disp_crm_hw_0",
	[VOTER_IDX_DISP_CRM_HW_1] = "disp_crm_hw_1",
	[VOTER_IDX_DISP_CRM_HW_2] = "disp_crm_hw_2",
	[VOTER_IDX_DISP_CRM_HW_3] = "disp_crm_hw_3",
	[VOTER_IDX_DISP_CRM_HW_4] = "disp_crm_hw_4",
	[VOTER_IDX_DISP_CRM_HW_5] = "disp_crm_hw_5",
	[VOTER_IDX_DISP_CRM_HW_6] = "disp_crm_hw_6",
	[VOTER_IDX_DISP_CRM_HW_7] = "disp_crm_hw_7",
	[VOTER_IDX_DISP_CRM_HW_8] = "disp_crm_hw_8",
};

static struct qcom_icc_desc canoe_mmss_noc = {
	.config = &icc_regmap_config,
	.nodes = mmss_noc_nodes,
	.num_nodes = ARRAY_SIZE(mmss_noc_nodes),
	.bcms = mmss_noc_bcms,
	.num_bcms = ARRAY_SIZE(mmss_noc_bcms),
	.voters = mmss_noc_voters,
	.num_voters = ARRAY_SIZE(mmss_noc_voters),
};

static struct qcom_icc_bcm *nsp_noc_bcms[] = {
	&bcm_co0,
};

static struct qcom_icc_node *nsp_noc_nodes[] = {
	[MASTER_CDSP_PROC] = &qnm_nsp,
	[SLAVE_CDSP_MEM_NOC] = &qns_nsp_gemnoc,
};

static char *nsp_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_nsp_noc = {
	.config = &icc_regmap_config,
	.nodes = nsp_noc_nodes,
	.num_nodes = ARRAY_SIZE(nsp_noc_nodes),
	.bcms = nsp_noc_bcms,
	.num_bcms = ARRAY_SIZE(nsp_noc_bcms),
	.voters = nsp_noc_voters,
	.num_voters = ARRAY_SIZE(nsp_noc_voters),
};

static struct qcom_icc_bcm *pcie_anoc_bcms[] = {
	&bcm_sn4,
	&bcm_sn4_pcie_crm_hw_0,
};

static struct qcom_icc_node *pcie_anoc_nodes[] = {
	[MASTER_PCIE_ANOC_CFG] = &qsm_pcie_anoc_cfg,
	[MASTER_PCIE_0] = &xm_pcie,
	[SLAVE_ANOC_PCIE_GEM_NOC] = &qns_pcie_gemnoc,
	[SLAVE_SERVICE_PCIE_ANOC] = &srvc_pcie_aggre_noc,
	[MASTER_PCIE_0_PCIE_CRM_HW_0] = &xm_pcie_pcie_crm_hw_0,
	[SLAVE_ANOC_PCIE_GEM_NOC_PCIE_CRM_HW_0] = &qns_pcie_gemnoc_pcie_crm_hw_0,
};

static char *pcie_anoc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
	[VOTER_IDX_PCIE_CRM_HW_0] = "pcie_crm_hw_0",
};

static struct qcom_icc_desc canoe_pcie_anoc = {
	.config = &icc_regmap_config,
	.nodes = pcie_anoc_nodes,
	.num_nodes = ARRAY_SIZE(pcie_anoc_nodes),
	.bcms = pcie_anoc_bcms,
	.num_bcms = ARRAY_SIZE(pcie_anoc_bcms),
	.voters = pcie_anoc_voters,
	.num_voters = ARRAY_SIZE(pcie_anoc_voters),
};

static struct qcom_icc_bcm *system_noc_bcms[] = {
	&bcm_sn0,
	&bcm_sn2,
	&bcm_sn3,
};

static struct qcom_icc_node *system_noc_nodes[] = {
	[MASTER_A1NOC_SNOC] = &qnm_aggre1_noc,
	[MASTER_A2NOC_SNOC] = &qnm_aggre2_noc,
	[MASTER_APSS_NOC] = &qnm_apss_noc,
	[MASTER_CNOC_SNOC] = &qnm_cnoc_data,
	[SLAVE_SNOC_GEM_NOC_SF] = &qns_gemnoc_sf,
};

static char *system_noc_voters[] = {
	[VOTER_IDX_HLOS] = "hlos",
};

static struct qcom_icc_desc canoe_system_noc = {
	.config = &icc_regmap_config,
	.nodes = system_noc_nodes,
	.num_nodes = ARRAY_SIZE(system_noc_nodes),
	.bcms = system_noc_bcms,
	.num_bcms = ARRAY_SIZE(system_noc_bcms),
	.voters = system_noc_voters,
	.num_voters = ARRAY_SIZE(system_noc_voters),
};

static int qnoc_probe(struct platform_device *pdev)
{
	int ret;

	ret = qcom_icc_rpmh_probe(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to register ICC provider\n");

	dev_info(&pdev->dev, "Registered ICC provider\n");

	return ret;
}

static const struct of_device_id qnoc_of_match[] = {
	{ .compatible = "qcom,canoe-aggre_noc",
	  .data = &canoe_aggre_noc},
	{ .compatible = "qcom,canoe-clk_virt",
	  .data = &canoe_clk_virt},
	{ .compatible = "qcom,canoe-cnoc_cfg",
	  .data = &canoe_cnoc_cfg},
	{ .compatible = "qcom,canoe-cnoc_main",
	  .data = &canoe_cnoc_main},
	{ .compatible = "qcom,canoe-gem_noc",
	  .data = &canoe_gem_noc},
	{ .compatible = "qcom,canoe-lpass_ag_noc",
	  .data = &canoe_lpass_ag_noc},
	{ .compatible = "qcom,canoe-lpass_lpiaon_noc",
	  .data = &canoe_lpass_lpiaon_noc},
	{ .compatible = "qcom,canoe-lpass_lpicx_noc",
	  .data = &canoe_lpass_lpicx_noc},
	{ .compatible = "qcom,canoe-mc_virt",
	  .data = &canoe_mc_virt},
	{ .compatible = "qcom,canoe-mmss_noc",
	  .data = &canoe_mmss_noc},
	{ .compatible = "qcom,canoe-nsp_noc",
	  .data = &canoe_nsp_noc},
	{ .compatible = "qcom,canoe-pcie_anoc",
	  .data = &canoe_pcie_anoc},
	{ .compatible = "qcom,canoe-system_noc",
	  .data = &canoe_system_noc},
	{ }
};
MODULE_DEVICE_TABLE(of, qnoc_of_match);

static struct platform_driver qnoc_driver = {
	.probe = qnoc_probe,
	.remove_new = qcom_icc_rpmh_remove,
	.driver = {
		.name = "qnoc-canoe",
		.of_match_table = qnoc_of_match,
		.sync_state = icc_sync_state,
	},
};

static int __init qnoc_driver_init(void)
{
	return platform_driver_register(&qnoc_driver);
}
core_initcall(qnoc_driver_init);

MODULE_DESCRIPTION("Canoe NoC driver");
MODULE_LICENSE("GPL");
