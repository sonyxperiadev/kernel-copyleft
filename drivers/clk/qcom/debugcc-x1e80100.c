// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "clk: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-debug.h"
#include "common.h"

static struct measure_clk_data debug_mux_priv = {
	.ctl_reg = 0x62048,
	.status_reg = 0x6204C,
	.xo_div4_cbcr = 0x62008,
};

static const char *const av1e_cc_debug_mux_parent_names[] = {
	"av1e_cc_av1e_core_axi_clk",
	"av1e_cc_av1e_core_clk",
	"av1e_cc_av1e_gdsc_noc_ahb_clk",
	"av1e_cc_av1e_noc_core_axi_clk",
	"av1e_cc_av1e_noc_xo_clk",
	"measure_only_av1e_cc_ahb_clk",
	"measure_only_av1e_cc_av1e_cc_xo_clk",
	"measure_only_av1e_cc_av1e_noc_ahb_clk",
	"measure_only_av1e_cc_sleep_clk",
};

static int av1e_cc_debug_mux_sels[] = {
	0x3,		/* av1e_cc_av1e_core_axi_clk */
	0x1,		/* av1e_cc_av1e_core_clk */
	0x7,		/* av1e_cc_av1e_gdsc_noc_ahb_clk */
	0x5,		/* av1e_cc_av1e_noc_core_axi_clk */
	0xB,		/* av1e_cc_av1e_noc_xo_clk */
	0x9,		/* measure_only_av1e_cc_ahb_clk */
	0xA,		/* measure_only_av1e_cc_av1e_cc_xo_clk */
	0x8,		/* measure_only_av1e_cc_av1e_noc_ahb_clk */
	0xC,		/* measure_only_av1e_cc_sleep_clk */
};

static struct clk_debug_mux av1e_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x6004,
	.post_div_offset = 0x6008,
	.cbcr_offset = 0x600C,
	.src_sel_mask = 0x3F,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 3,
	.mux_sels = av1e_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(av1e_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "av1e_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = av1e_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(av1e_cc_debug_mux_parent_names),
	},
};

static const char *const cam_cc_debug_mux_parent_names[] = {
	"cam_cc_bps_ahb_clk",
	"cam_cc_bps_clk",
	"cam_cc_bps_fast_ahb_clk",
	"cam_cc_camnoc_axi_nrt_clk",
	"cam_cc_camnoc_axi_rt_clk",
	"cam_cc_camnoc_dcd_xo_clk",
	"cam_cc_camnoc_xo_clk",
	"cam_cc_cci_0_clk",
	"cam_cc_cci_1_clk",
	"cam_cc_core_ahb_clk",
	"cam_cc_cpas_ahb_clk",
	"cam_cc_cpas_bps_clk",
	"cam_cc_cpas_fast_ahb_clk",
	"cam_cc_cpas_ife_0_clk",
	"cam_cc_cpas_ife_1_clk",
	"cam_cc_cpas_ife_lite_clk",
	"cam_cc_cpas_ipe_nps_clk",
	"cam_cc_cpas_sfe_0_clk",
	"cam_cc_csi0phytimer_clk",
	"cam_cc_csi1phytimer_clk",
	"cam_cc_csi2phytimer_clk",
	"cam_cc_csi3phytimer_clk",
	"cam_cc_csi4phytimer_clk",
	"cam_cc_csi5phytimer_clk",
	"cam_cc_csid_clk",
	"cam_cc_csid_csiphy_rx_clk",
	"cam_cc_csiphy0_clk",
	"cam_cc_csiphy1_clk",
	"cam_cc_csiphy2_clk",
	"cam_cc_csiphy3_clk",
	"cam_cc_csiphy4_clk",
	"cam_cc_csiphy5_clk",
	"cam_cc_icp_ahb_clk",
	"cam_cc_icp_clk",
	"cam_cc_ife_0_clk",
	"cam_cc_ife_0_dsp_clk",
	"cam_cc_ife_0_fast_ahb_clk",
	"cam_cc_ife_1_clk",
	"cam_cc_ife_1_dsp_clk",
	"cam_cc_ife_1_fast_ahb_clk",
	"cam_cc_ife_lite_ahb_clk",
	"cam_cc_ife_lite_clk",
	"cam_cc_ife_lite_cphy_rx_clk",
	"cam_cc_ife_lite_csid_clk",
	"cam_cc_ipe_nps_ahb_clk",
	"cam_cc_ipe_nps_clk",
	"cam_cc_ipe_nps_fast_ahb_clk",
	"cam_cc_ipe_pps_clk",
	"cam_cc_ipe_pps_fast_ahb_clk",
	"cam_cc_jpeg_clk",
	"cam_cc_mclk0_clk",
	"cam_cc_mclk1_clk",
	"cam_cc_mclk2_clk",
	"cam_cc_mclk3_clk",
	"cam_cc_mclk4_clk",
	"cam_cc_mclk5_clk",
	"cam_cc_mclk6_clk",
	"cam_cc_mclk7_clk",
	"cam_cc_qdss_debug_clk",
	"cam_cc_qdss_debug_xo_clk",
	"cam_cc_sfe_0_clk",
	"cam_cc_sfe_0_fast_ahb_clk",
	"measure_only_cam_cc_gdsc_clk",
	"measure_only_cam_cc_sleep_clk",
};

static int cam_cc_debug_mux_sels[] = {
	0x17,		/* cam_cc_bps_ahb_clk */
	0x18,		/* cam_cc_bps_clk */
	0x16,		/* cam_cc_bps_fast_ahb_clk */
	0x57,		/* cam_cc_camnoc_axi_nrt_clk */
	0x49,		/* cam_cc_camnoc_axi_rt_clk */
	0x4A,		/* cam_cc_camnoc_dcd_xo_clk */
	0x60,		/* cam_cc_camnoc_xo_clk */
	0x44,		/* cam_cc_cci_0_clk */
	0x45,		/* cam_cc_cci_1_clk */
	0x4D,		/* cam_cc_core_ahb_clk */
	0x46,		/* cam_cc_cpas_ahb_clk */
	0x19,		/* cam_cc_cpas_bps_clk */
	0x47,		/* cam_cc_cpas_fast_ahb_clk */
	0x25,		/* cam_cc_cpas_ife_0_clk */
	0x2A,		/* cam_cc_cpas_ife_1_clk */
	0x34,		/* cam_cc_cpas_ife_lite_clk */
	0x1B,		/* cam_cc_cpas_ipe_nps_clk */
	0x39,		/* cam_cc_cpas_sfe_0_clk */
	0x9,		/* cam_cc_csi0phytimer_clk */
	0xC,		/* cam_cc_csi1phytimer_clk */
	0xE,		/* cam_cc_csi2phytimer_clk */
	0x10,		/* cam_cc_csi3phytimer_clk */
	0x12,		/* cam_cc_csi4phytimer_clk */
	0x14,		/* cam_cc_csi5phytimer_clk */
	0x48,		/* cam_cc_csid_clk */
	0xB,		/* cam_cc_csid_csiphy_rx_clk */
	0xA,		/* cam_cc_csiphy0_clk */
	0xD,		/* cam_cc_csiphy1_clk */
	0xF,		/* cam_cc_csiphy2_clk */
	0x11,		/* cam_cc_csiphy3_clk */
	0x13,		/* cam_cc_csiphy4_clk */
	0x15,		/* cam_cc_csiphy5_clk */
	0x43,		/* cam_cc_icp_ahb_clk */
	0x42,		/* cam_cc_icp_clk */
	0x24,		/* cam_cc_ife_0_clk */
	0x26,		/* cam_cc_ife_0_dsp_clk */
	0x28,		/* cam_cc_ife_0_fast_ahb_clk */
	0x29,		/* cam_cc_ife_1_clk */
	0x2B,		/* cam_cc_ife_1_dsp_clk */
	0x2D,		/* cam_cc_ife_1_fast_ahb_clk */
	0x37,		/* cam_cc_ife_lite_ahb_clk */
	0x33,		/* cam_cc_ife_lite_clk */
	0x36,		/* cam_cc_ife_lite_cphy_rx_clk */
	0x35,		/* cam_cc_ife_lite_csid_clk */
	0x1E,		/* cam_cc_ipe_nps_ahb_clk */
	0x1A,		/* cam_cc_ipe_nps_clk */
	0x1F,		/* cam_cc_ipe_nps_fast_ahb_clk */
	0x1C,		/* cam_cc_ipe_pps_clk */
	0x20,		/* cam_cc_ipe_pps_fast_ahb_clk */
	0x40,		/* cam_cc_jpeg_clk */
	0x1,		/* cam_cc_mclk0_clk */
	0x2,		/* cam_cc_mclk1_clk */
	0x3,		/* cam_cc_mclk2_clk */
	0x4,		/* cam_cc_mclk3_clk */
	0x5,		/* cam_cc_mclk4_clk */
	0x6,		/* cam_cc_mclk5_clk */
	0x7,		/* cam_cc_mclk6_clk */
	0x8,		/* cam_cc_mclk7_clk */
	0x4B,		/* cam_cc_qdss_debug_clk */
	0x4C,		/* cam_cc_qdss_debug_xo_clk */
	0x38,		/* cam_cc_sfe_0_clk */
	0x3B,		/* cam_cc_sfe_0_fast_ahb_clk */
	0x4E,		/* measure_only_cam_cc_gdsc_clk */
	0x4F,		/* measure_only_cam_cc_sleep_clk */
};

static struct clk_debug_mux cam_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x16000,
	.post_div_offset = 0x14004,
	.cbcr_offset = 0x14008,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 4,
	.mux_sels = cam_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(cam_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "cam_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = cam_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(cam_cc_debug_mux_parent_names),
	},
};

static const char *const disp_cc_debug_mux_parent_names[] = {
	"disp_cc_mdss_accu_clk",
	"disp_cc_mdss_ahb1_clk",
	"disp_cc_mdss_ahb_clk",
	"disp_cc_mdss_byte0_clk",
	"disp_cc_mdss_byte0_intf_clk",
	"disp_cc_mdss_byte1_clk",
	"disp_cc_mdss_byte1_intf_clk",
	"disp_cc_mdss_dptx0_aux_clk",
	"disp_cc_mdss_dptx0_link_clk",
	"disp_cc_mdss_dptx0_link_intf_clk",
	"disp_cc_mdss_dptx0_pixel0_clk",
	"disp_cc_mdss_dptx0_pixel1_clk",
	"disp_cc_mdss_dptx0_usb_router_link_intf_clk",
	"disp_cc_mdss_dptx1_aux_clk",
	"disp_cc_mdss_dptx1_link_clk",
	"disp_cc_mdss_dptx1_link_intf_clk",
	"disp_cc_mdss_dptx1_pixel0_clk",
	"disp_cc_mdss_dptx1_pixel1_clk",
	"disp_cc_mdss_dptx1_usb_router_link_intf_clk",
	"disp_cc_mdss_dptx2_aux_clk",
	"disp_cc_mdss_dptx2_link_clk",
	"disp_cc_mdss_dptx2_link_intf_clk",
	"disp_cc_mdss_dptx2_pixel0_clk",
	"disp_cc_mdss_dptx2_pixel1_clk",
	"disp_cc_mdss_dptx2_usb_router_link_intf_clk",
	"disp_cc_mdss_dptx3_aux_clk",
	"disp_cc_mdss_dptx3_link_clk",
	"disp_cc_mdss_dptx3_link_intf_clk",
	"disp_cc_mdss_dptx3_pixel0_clk",
	"disp_cc_mdss_esc0_clk",
	"disp_cc_mdss_esc1_clk",
	"disp_cc_mdss_mdp1_clk",
	"disp_cc_mdss_mdp_clk",
	"disp_cc_mdss_mdp_lut1_clk",
	"disp_cc_mdss_mdp_lut_clk",
	"disp_cc_mdss_non_gdsc_ahb_clk",
	"disp_cc_mdss_pclk0_clk",
	"disp_cc_mdss_pclk1_clk",
	"disp_cc_mdss_vsync1_clk",
	"disp_cc_mdss_vsync_clk",
	"measure_only_disp_cc_mdss_rscc_ahb_clk",
	"measure_only_disp_cc_mdss_rscc_vsync_clk",
	"measure_only_disp_cc_sleep_clk",
	"measure_only_disp_cc_xo_clk",
};

static int disp_cc_debug_mux_sels[] = {
	0x47,		/* disp_cc_mdss_accu_clk */
	0x38,		/* disp_cc_mdss_ahb1_clk */
	0x34,		/* disp_cc_mdss_ahb_clk */
	0x14,		/* disp_cc_mdss_byte0_clk */
	0x15,		/* disp_cc_mdss_byte0_intf_clk */
	0x16,		/* disp_cc_mdss_byte1_clk */
	0x17,		/* disp_cc_mdss_byte1_intf_clk */
	0x20,		/* disp_cc_mdss_dptx0_aux_clk */
	0x1A,		/* disp_cc_mdss_dptx0_link_clk */
	0x1C,		/* disp_cc_mdss_dptx0_link_intf_clk */
	0x1E,		/* disp_cc_mdss_dptx0_pixel0_clk */
	0x1F,		/* disp_cc_mdss_dptx0_pixel1_clk */
	0x1B,		/* disp_cc_mdss_dptx0_usb_router_link_intf_clk */
	0x27,		/* disp_cc_mdss_dptx1_aux_clk */
	0x23,		/* disp_cc_mdss_dptx1_link_clk */
	0x25,		/* disp_cc_mdss_dptx1_link_intf_clk */
	0x21,		/* disp_cc_mdss_dptx1_pixel0_clk */
	0x22,		/* disp_cc_mdss_dptx1_pixel1_clk */
	0x24,		/* disp_cc_mdss_dptx1_usb_router_link_intf_clk */
	0x2E,		/* disp_cc_mdss_dptx2_aux_clk */
	0x2A,		/* disp_cc_mdss_dptx2_link_clk */
	0x2B,		/* disp_cc_mdss_dptx2_link_intf_clk */
	0x28,		/* disp_cc_mdss_dptx2_pixel0_clk */
	0x29,		/* disp_cc_mdss_dptx2_pixel1_clk */
	0x2C,		/* disp_cc_mdss_dptx2_usb_router_link_intf_clk */
	0x32,		/* disp_cc_mdss_dptx3_aux_clk */
	0x30,		/* disp_cc_mdss_dptx3_link_clk */
	0x31,		/* disp_cc_mdss_dptx3_link_intf_clk */
	0x2F,		/* disp_cc_mdss_dptx3_pixel0_clk */
	0x18,		/* disp_cc_mdss_esc0_clk */
	0x19,		/* disp_cc_mdss_esc1_clk */
	0x35,		/* disp_cc_mdss_mdp1_clk */
	0x11,		/* disp_cc_mdss_mdp_clk */
	0x36,		/* disp_cc_mdss_mdp_lut1_clk */
	0x12,		/* disp_cc_mdss_mdp_lut_clk */
	0x39,		/* disp_cc_mdss_non_gdsc_ahb_clk */
	0xF,		/* disp_cc_mdss_pclk0_clk */
	0x10,		/* disp_cc_mdss_pclk1_clk */
	0x37,		/* disp_cc_mdss_vsync1_clk */
	0x13,		/* disp_cc_mdss_vsync_clk */
	0x3B,		/* measure_only_disp_cc_mdss_rscc_ahb_clk */
	0x3A,		/* measure_only_disp_cc_mdss_rscc_vsync_clk */
	0x48,		/* measure_only_disp_cc_sleep_clk */
	0x46,		/* measure_only_disp_cc_xo_clk */
};

static struct clk_debug_mux disp_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x11000,
	.post_div_offset = 0xD000,
	.cbcr_offset = 0xD004,
	.src_sel_mask = 0x1FF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 4,
	.mux_sels = disp_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(disp_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "disp_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = disp_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(disp_cc_debug_mux_parent_names),
	},
};

static const char *const gcc_debug_mux_parent_names[] = {
	"av1e_cc_debug_mux",
	"cam_cc_debug_mux",
	"disp_cc_debug_mux",
	"gcc_aggre_noc_usb_north_axi_clk",
	"gcc_aggre_noc_usb_south_axi_clk",
	"gcc_aggre_ufs_phy_axi_clk",
	"gcc_aggre_usb2_prim_axi_clk",
	"gcc_aggre_usb3_mp_axi_clk",
	"gcc_aggre_usb3_prim_axi_clk",
	"gcc_aggre_usb3_sec_axi_clk",
	"gcc_aggre_usb3_tert_axi_clk",
	"gcc_aggre_usb4_0_axi_clk",
	"gcc_aggre_usb4_1_axi_clk",
	"gcc_aggre_usb4_2_axi_clk",
	"gcc_aggre_usb_noc_axi_clk",
	"gcc_av1e_axi_clk",
	"gcc_boot_rom_ahb_clk",
	"gcc_camera_hf_axi_clk",
	"gcc_camera_sf_axi_clk",
	"gcc_cfg_noc_pcie_anoc_ahb_clk",
	"gcc_cfg_noc_pcie_anoc_north_ahb_clk",
	"gcc_cfg_noc_pcie_anoc_south_ahb_clk",
	"gcc_cfg_noc_usb2_prim_axi_clk",
	"gcc_cfg_noc_usb3_mp_axi_clk",
	"gcc_cfg_noc_usb3_prim_axi_clk",
	"gcc_cfg_noc_usb3_sec_axi_clk",
	"gcc_cfg_noc_usb3_tert_axi_clk",
	"gcc_cfg_noc_usb_anoc_ahb_clk",
	"gcc_cfg_noc_usb_anoc_north_ahb_clk",
	"gcc_cfg_noc_usb_anoc_south_ahb_clk",
	"gcc_cnoc_pcie1_tunnel_clk",
	"gcc_cnoc_pcie2_tunnel_clk",
	"gcc_cnoc_pcie_north_sf_axi_clk",
	"gcc_cnoc_pcie_south_sf_axi_clk",
	"gcc_cnoc_pcie_tunnel_clk",
	"gcc_ddrss_gpu_axi_clk",
	"gcc_disp_hf_axi_clk",
	"gcc_gp1_clk",
	"gcc_gp2_clk",
	"gcc_gp3_clk",
	"gcc_gpu_gpll0_cph_clk_src",
	"gcc_gpu_gpll0_div_cph_clk_src",
	"gcc_gpu_memnoc_gfx_clk",
	"gcc_gpu_snoc_dvm_gfx_clk",
	"gcc_pcie0_phy_rchng_clk",
	"gcc_pcie1_phy_rchng_clk",
	"gcc_pcie2_phy_rchng_clk",
	"gcc_pcie_0_aux_clk",
	"gcc_pcie_0_cfg_ahb_clk",
	"gcc_pcie_0_mstr_axi_clk",
	"gcc_pcie_0_pipe_clk",
	"gcc_pcie_0_slv_axi_clk",
	"gcc_pcie_0_slv_q2a_axi_clk",
	"gcc_pcie_1_aux_clk",
	"gcc_pcie_1_cfg_ahb_clk",
	"gcc_pcie_1_mstr_axi_clk",
	"gcc_pcie_1_pipe_clk",
	"gcc_pcie_1_slv_axi_clk",
	"gcc_pcie_1_slv_q2a_axi_clk",
	"gcc_pcie_2_aux_clk",
	"gcc_pcie_2_cfg_ahb_clk",
	"gcc_pcie_2_mstr_axi_clk",
	"gcc_pcie_2_pipe_clk",
	"gcc_pcie_2_slv_axi_clk",
	"gcc_pcie_2_slv_q2a_axi_clk",
	"gcc_pcie_3_aux_clk",
	"gcc_pcie_3_cfg_ahb_clk",
	"gcc_pcie_3_mstr_axi_clk",
	"gcc_pcie_3_phy_aux_clk",
	"gcc_pcie_3_phy_rchng_clk",
	"gcc_pcie_3_pipe_clk",
	"gcc_pcie_3_pipediv2_clk",
	"gcc_pcie_3_slv_axi_clk",
	"gcc_pcie_3_slv_q2a_axi_clk",
	"gcc_pcie_4_aux_clk",
	"gcc_pcie_4_cfg_ahb_clk",
	"gcc_pcie_4_mstr_axi_clk",
	"gcc_pcie_4_phy_rchng_clk",
	"gcc_pcie_4_pipe_clk",
	"gcc_pcie_4_pipediv2_clk",
	"gcc_pcie_4_slv_axi_clk",
	"gcc_pcie_4_slv_q2a_axi_clk",
	"gcc_pcie_5_aux_clk",
	"gcc_pcie_5_cfg_ahb_clk",
	"gcc_pcie_5_mstr_axi_clk",
	"gcc_pcie_5_phy_rchng_clk",
	"gcc_pcie_5_pipe_clk",
	"gcc_pcie_5_pipediv2_clk",
	"gcc_pcie_5_slv_axi_clk",
	"gcc_pcie_5_slv_q2a_axi_clk",
	"gcc_pcie_6a_aux_clk",
	"gcc_pcie_6a_cfg_ahb_clk",
	"gcc_pcie_6a_mstr_axi_clk",
	"gcc_pcie_6a_phy_aux_clk",
	"gcc_pcie_6a_phy_rchng_clk",
	"gcc_pcie_6a_pipe_clk",
	"gcc_pcie_6a_pipediv2_clk",
	"gcc_pcie_6a_slv_axi_clk",
	"gcc_pcie_6a_slv_q2a_axi_clk",
	"gcc_pcie_6b_aux_clk",
	"gcc_pcie_6b_cfg_ahb_clk",
	"gcc_pcie_6b_mstr_axi_clk",
	"gcc_pcie_6b_phy_aux_clk",
	"gcc_pcie_6b_phy_rchng_clk",
	"gcc_pcie_6b_pipe_clk",
	"gcc_pcie_6b_pipediv2_clk",
	"gcc_pcie_6b_slv_axi_clk",
	"gcc_pcie_6b_slv_q2a_axi_clk",
	"gcc_pcie_rscc_ahb_clk",
	"gcc_pcie_rscc_xo_clk",
	"gcc_pdm2_clk",
	"gcc_pdm_ahb_clk",
	"gcc_pdm_xo4_clk",
	"gcc_qmip_av1e_ahb_clk",
	"gcc_qmip_camera_nrt_ahb_clk",
	"gcc_qmip_camera_rt_ahb_clk",
	"gcc_qmip_disp_ahb_clk",
	"gcc_qmip_gpu_ahb_clk",
	"gcc_qmip_video_cv_cpu_ahb_clk",
	"gcc_qmip_video_cvp_ahb_clk",
	"gcc_qmip_video_v_cpu_ahb_clk",
	"gcc_qmip_video_vcodec_ahb_clk",
	"gcc_qupv3_wrap0_core_2x_clk",
	"gcc_qupv3_wrap0_core_clk",
	"gcc_qupv3_wrap0_qspi_s2_clk",
	"gcc_qupv3_wrap0_qspi_s3_clk",
	"gcc_qupv3_wrap0_s0_clk",
	"gcc_qupv3_wrap0_s1_clk",
	"gcc_qupv3_wrap0_s2_clk",
	"gcc_qupv3_wrap0_s3_clk",
	"gcc_qupv3_wrap0_s4_clk",
	"gcc_qupv3_wrap0_s5_clk",
	"gcc_qupv3_wrap0_s6_clk",
	"gcc_qupv3_wrap0_s7_clk",
	"gcc_qupv3_wrap1_core_2x_clk",
	"gcc_qupv3_wrap1_core_clk",
	"gcc_qupv3_wrap1_qspi_s2_clk",
	"gcc_qupv3_wrap1_qspi_s3_clk",
	"gcc_qupv3_wrap1_s0_clk",
	"gcc_qupv3_wrap1_s1_clk",
	"gcc_qupv3_wrap1_s2_clk",
	"gcc_qupv3_wrap1_s3_clk",
	"gcc_qupv3_wrap1_s4_clk",
	"gcc_qupv3_wrap1_s5_clk",
	"gcc_qupv3_wrap1_s6_clk",
	"gcc_qupv3_wrap1_s7_clk",
	"gcc_qupv3_wrap2_core_2x_clk",
	"gcc_qupv3_wrap2_core_clk",
	"gcc_qupv3_wrap2_qspi_s2_clk",
	"gcc_qupv3_wrap2_qspi_s3_clk",
	"gcc_qupv3_wrap2_s0_clk",
	"gcc_qupv3_wrap2_s1_clk",
	"gcc_qupv3_wrap2_s2_clk",
	"gcc_qupv3_wrap2_s3_clk",
	"gcc_qupv3_wrap2_s4_clk",
	"gcc_qupv3_wrap2_s5_clk",
	"gcc_qupv3_wrap2_s6_clk",
	"gcc_qupv3_wrap2_s7_clk",
	"gcc_qupv3_wrap_0_m_ahb_clk",
	"gcc_qupv3_wrap_0_s_ahb_clk",
	"gcc_qupv3_wrap_1_m_ahb_clk",
	"gcc_qupv3_wrap_1_s_ahb_clk",
	"gcc_qupv3_wrap_2_m_ahb_clk",
	"gcc_qupv3_wrap_2_s_ahb_clk",
	"gcc_sdcc2_ahb_clk",
	"gcc_sdcc2_apps_clk",
	"gcc_sdcc4_ahb_clk",
	"gcc_sdcc4_apps_clk",
	"gcc_sys_noc_usb_axi_clk",
	"gcc_ufs_phy_ahb_clk",
	"gcc_ufs_phy_axi_clk",
	"gcc_ufs_phy_ice_core_clk",
	"gcc_ufs_phy_phy_aux_clk",
	"gcc_ufs_phy_rx_symbol_0_clk",
	"gcc_ufs_phy_rx_symbol_1_clk",
	"gcc_ufs_phy_tx_symbol_0_clk",
	"gcc_ufs_phy_unipro_core_clk",
	"gcc_usb20_master_clk",
	"gcc_usb20_mock_utmi_clk",
	"gcc_usb20_sleep_clk",
	"gcc_usb30_mp_master_clk",
	"gcc_usb30_mp_mock_utmi_clk",
	"gcc_usb30_mp_sleep_clk",
	"gcc_usb30_prim_master_clk",
	"gcc_usb30_prim_mock_utmi_clk",
	"gcc_usb30_prim_sleep_clk",
	"gcc_usb30_sec_master_clk",
	"gcc_usb30_sec_mock_utmi_clk",
	"gcc_usb30_sec_sleep_clk",
	"gcc_usb30_tert_master_clk",
	"gcc_usb30_tert_mock_utmi_clk",
	"gcc_usb30_tert_sleep_clk",
	"gcc_usb3_mp_phy_aux_clk",
	"gcc_usb3_mp_phy_com_aux_clk",
	"gcc_usb3_mp_phy_pipe_0_clk",
	"gcc_usb3_mp_phy_pipe_1_clk",
	"gcc_usb3_prim_phy_aux_clk",
	"gcc_usb3_prim_phy_com_aux_clk",
	"gcc_usb3_prim_phy_pipe_clk",
	"gcc_usb3_sec_phy_aux_clk",
	"gcc_usb3_sec_phy_com_aux_clk",
	"gcc_usb3_sec_phy_pipe_clk",
	"gcc_usb3_tert_phy_aux_clk",
	"gcc_usb3_tert_phy_com_aux_clk",
	"gcc_usb3_tert_phy_pipe_clk",
	"gcc_usb4_0_cfg_ahb_clk",
	"gcc_usb4_0_dp0_clk",
	"gcc_usb4_0_dp1_clk",
	"gcc_usb4_0_master_clk",
	"gcc_usb4_0_phy_p2rr2p_pipe_clk",
	"gcc_usb4_0_phy_pcie_pipe_clk",
	"gcc_usb4_0_phy_rx0_clk",
	"gcc_usb4_0_phy_rx1_clk",
	"gcc_usb4_0_phy_usb_pipe_clk",
	"gcc_usb4_0_sb_if_clk",
	"gcc_usb4_0_sys_clk",
	"gcc_usb4_0_tmu_clk",
	"gcc_usb4_1_cfg_ahb_clk",
	"gcc_usb4_1_dp0_clk",
	"gcc_usb4_1_dp1_clk",
	"gcc_usb4_1_master_clk",
	"gcc_usb4_1_phy_p2rr2p_pipe_clk",
	"gcc_usb4_1_phy_pcie_pipe_clk",
	"gcc_usb4_1_phy_rx0_clk",
	"gcc_usb4_1_phy_rx1_clk",
	"gcc_usb4_1_phy_usb_pipe_clk",
	"gcc_usb4_1_sb_if_clk",
	"gcc_usb4_1_sys_clk",
	"gcc_usb4_1_tmu_clk",
	"gcc_usb4_2_cfg_ahb_clk",
	"gcc_usb4_2_dp0_clk",
	"gcc_usb4_2_dp1_clk",
	"gcc_usb4_2_master_clk",
	"gcc_usb4_2_phy_p2rr2p_pipe_clk",
	"gcc_usb4_2_phy_pcie_pipe_clk",
	"gcc_usb4_2_phy_rx0_clk",
	"gcc_usb4_2_phy_rx1_clk",
	"gcc_usb4_2_phy_usb_pipe_clk",
	"gcc_usb4_2_sb_if_clk",
	"gcc_usb4_2_sys_clk",
	"gcc_usb4_2_tmu_clk",
	"gcc_video_axi0_clk",
	"gcc_video_axi1_clk",
	"gpu_cc_debug_mux",
	"mc_cc_debug_mux",
	"measure_only_cnoc_clk",
	"measure_only_memnoc_clk",
	"measure_only_snoc_clk",
	"video_cc_debug_mux",
};

static int gcc_debug_mux_sels[] = {
	0xB4,		/* av1e_cc_debug_mux */
	0xA2,		/* cam_cc_debug_mux */
	0xA7,		/* disp_cc_debug_mux */
	0x6B,		/* gcc_aggre_noc_usb_north_axi_clk */
	0x6A,		/* gcc_aggre_noc_usb_south_axi_clk */
	0x63,		/* gcc_aggre_ufs_phy_axi_clk */
	0x62,		/* gcc_aggre_usb2_prim_axi_clk */
	0x61,		/* gcc_aggre_usb3_mp_axi_clk */
	0x5B,		/* gcc_aggre_usb3_prim_axi_clk */
	0x5D,		/* gcc_aggre_usb3_sec_axi_clk */
	0x5F,		/* gcc_aggre_usb3_tert_axi_clk */
	0x5C,		/* gcc_aggre_usb4_0_axi_clk */
	0x5E,		/* gcc_aggre_usb4_1_axi_clk */
	0x60,		/* gcc_aggre_usb4_2_axi_clk */
	0x69,		/* gcc_aggre_usb_noc_axi_clk */
	0xB2,		/* gcc_av1e_axi_clk */
	0x19B,		/* gcc_boot_rom_ahb_clk */
	0x9D,		/* gcc_camera_hf_axi_clk */
	0x9F,		/* gcc_camera_sf_axi_clk */
	0x3D,		/* gcc_cfg_noc_pcie_anoc_ahb_clk */
	0x3F,		/* gcc_cfg_noc_pcie_anoc_north_ahb_clk */
	0x41,		/* gcc_cfg_noc_pcie_anoc_south_ahb_clk */
	0x25,		/* gcc_cfg_noc_usb2_prim_axi_clk */
	0x24,		/* gcc_cfg_noc_usb3_mp_axi_clk */
	0x21,		/* gcc_cfg_noc_usb3_prim_axi_clk */
	0x22,		/* gcc_cfg_noc_usb3_sec_axi_clk */
	0x23,		/* gcc_cfg_noc_usb3_tert_axi_clk */
	0x3E,		/* gcc_cfg_noc_usb_anoc_ahb_clk */
	0x40,		/* gcc_cfg_noc_usb_anoc_north_ahb_clk */
	0x42,		/* gcc_cfg_noc_usb_anoc_south_ahb_clk */
	0x39,		/* gcc_cnoc_pcie1_tunnel_clk */
	0x3A,		/* gcc_cnoc_pcie2_tunnel_clk */
	0x19,		/* gcc_cnoc_pcie_north_sf_axi_clk */
	0x1A,		/* gcc_cnoc_pcie_south_sf_axi_clk */
	0x38,		/* gcc_cnoc_pcie_tunnel_clk */
	0x1B3,		/* gcc_ddrss_gpu_axi_clk */
	0xA5,		/* gcc_disp_hf_axi_clk */
	0x1FE,		/* gcc_gp1_clk */
	0x1FF,		/* gcc_gp2_clk */
	0x200,		/* gcc_gp3_clk */
	0x25F,		/* gcc_gpu_gpll0_cph_clk_src */
	0x260,		/* gcc_gpu_gpll0_div_cph_clk_src */
	0x25C,		/* gcc_gpu_memnoc_gfx_clk */
	0x25E,		/* gcc_gpu_snoc_dvm_gfx_clk */
	0xFA,		/* gcc_pcie0_phy_rchng_clk */
	0x11F,		/* gcc_pcie1_phy_rchng_clk */
	0x144,		/* gcc_pcie2_phy_rchng_clk */
	0xF8,		/* gcc_pcie_0_aux_clk */
	0xF7,		/* gcc_pcie_0_cfg_ahb_clk */
	0xF6,		/* gcc_pcie_0_mstr_axi_clk */
	0xF9,		/* gcc_pcie_0_pipe_clk */
	0xF5,		/* gcc_pcie_0_slv_axi_clk */
	0xF4,		/* gcc_pcie_0_slv_q2a_axi_clk */
	0x11D,		/* gcc_pcie_1_aux_clk */
	0x11C,		/* gcc_pcie_1_cfg_ahb_clk */
	0x11B,		/* gcc_pcie_1_mstr_axi_clk */
	0x11E,		/* gcc_pcie_1_pipe_clk */
	0x11A,		/* gcc_pcie_1_slv_axi_clk */
	0x119,		/* gcc_pcie_1_slv_q2a_axi_clk */
	0x142,		/* gcc_pcie_2_aux_clk */
	0x141,		/* gcc_pcie_2_cfg_ahb_clk */
	0x140,		/* gcc_pcie_2_mstr_axi_clk */
	0x143,		/* gcc_pcie_2_pipe_clk */
	0x13F,		/* gcc_pcie_2_slv_axi_clk */
	0x13E,		/* gcc_pcie_2_slv_q2a_axi_clk */
	0x205,		/* gcc_pcie_3_aux_clk */
	0x204,		/* gcc_pcie_3_cfg_ahb_clk */
	0x203,		/* gcc_pcie_3_mstr_axi_clk */
	0x206,		/* gcc_pcie_3_phy_aux_clk */
	0x208,		/* gcc_pcie_3_phy_rchng_clk */
	0x207,		/* gcc_pcie_3_pipe_clk */
	0x209,		/* gcc_pcie_3_pipediv2_clk */
	0x202,		/* gcc_pcie_3_slv_axi_clk */
	0x201,		/* gcc_pcie_3_slv_q2a_axi_clk */
	0x211,		/* gcc_pcie_4_aux_clk */
	0x210,		/* gcc_pcie_4_cfg_ahb_clk */
	0x20F,		/* gcc_pcie_4_mstr_axi_clk */
	0x213,		/* gcc_pcie_4_phy_rchng_clk */
	0x212,		/* gcc_pcie_4_pipe_clk */
	0x214,		/* gcc_pcie_4_pipediv2_clk */
	0x20E,		/* gcc_pcie_4_slv_axi_clk */
	0x20D,		/* gcc_pcie_4_slv_q2a_axi_clk */
	0x21B,		/* gcc_pcie_5_aux_clk */
	0x21A,		/* gcc_pcie_5_cfg_ahb_clk */
	0x219,		/* gcc_pcie_5_mstr_axi_clk */
	0x21D,		/* gcc_pcie_5_phy_rchng_clk */
	0x21C,		/* gcc_pcie_5_pipe_clk */
	0x21E,		/* gcc_pcie_5_pipediv2_clk */
	0x218,		/* gcc_pcie_5_slv_axi_clk */
	0x217,		/* gcc_pcie_5_slv_q2a_axi_clk */
	0x231,		/* gcc_pcie_6a_aux_clk */
	0x230,		/* gcc_pcie_6a_cfg_ahb_clk */
	0x22F,		/* gcc_pcie_6a_mstr_axi_clk */
	0x232,		/* gcc_pcie_6a_phy_aux_clk */
	0x234,		/* gcc_pcie_6a_phy_rchng_clk */
	0x233,		/* gcc_pcie_6a_pipe_clk */
	0x235,		/* gcc_pcie_6a_pipediv2_clk */
	0x22E,		/* gcc_pcie_6a_slv_axi_clk */
	0x22D,		/* gcc_pcie_6a_slv_q2a_axi_clk */
	0x225,		/* gcc_pcie_6b_aux_clk */
	0x224,		/* gcc_pcie_6b_cfg_ahb_clk */
	0x223,		/* gcc_pcie_6b_mstr_axi_clk */
	0x226,		/* gcc_pcie_6b_phy_aux_clk */
	0x228,		/* gcc_pcie_6b_phy_rchng_clk */
	0x227,		/* gcc_pcie_6b_pipe_clk */
	0x229,		/* gcc_pcie_6b_pipediv2_clk */
	0x222,		/* gcc_pcie_6b_slv_axi_clk */
	0x221,		/* gcc_pcie_6b_slv_q2a_axi_clk */
	0x84,		/* gcc_pcie_rscc_ahb_clk */
	0x83,		/* gcc_pcie_rscc_xo_clk */
	0x18C,		/* gcc_pdm2_clk */
	0x18A,		/* gcc_pdm_ahb_clk */
	0x18B,		/* gcc_pdm_xo4_clk */
	0xB5,		/* gcc_qmip_av1e_ahb_clk */
	0x9B,		/* gcc_qmip_camera_nrt_ahb_clk */
	0x9C,		/* gcc_qmip_camera_rt_ahb_clk */
	0xA4,		/* gcc_qmip_disp_ahb_clk */
	0x259,		/* gcc_qmip_gpu_ahb_clk */
	0xAC,		/* gcc_qmip_video_cv_cpu_ahb_clk */
	0xA9,		/* gcc_qmip_video_cvp_ahb_clk */
	0xAB,		/* gcc_qmip_video_v_cpu_ahb_clk */
	0xAA,		/* gcc_qmip_video_vcodec_ahb_clk */
	0x163,		/* gcc_qupv3_wrap0_core_2x_clk */
	0x162,		/* gcc_qupv3_wrap0_core_clk */
	0x167,		/* gcc_qupv3_wrap0_qspi_s2_clk */
	0x169,		/* gcc_qupv3_wrap0_qspi_s3_clk */
	0x164,		/* gcc_qupv3_wrap0_s0_clk */
	0x165,		/* gcc_qupv3_wrap0_s1_clk */
	0x166,		/* gcc_qupv3_wrap0_s2_clk */
	0x168,		/* gcc_qupv3_wrap0_s3_clk */
	0x16A,		/* gcc_qupv3_wrap0_s4_clk */
	0x16B,		/* gcc_qupv3_wrap0_s5_clk */
	0x16C,		/* gcc_qupv3_wrap0_s6_clk */
	0x16D,		/* gcc_qupv3_wrap0_s7_clk */
	0x171,		/* gcc_qupv3_wrap1_core_2x_clk */
	0x170,		/* gcc_qupv3_wrap1_core_clk */
	0x175,		/* gcc_qupv3_wrap1_qspi_s2_clk */
	0x177,		/* gcc_qupv3_wrap1_qspi_s3_clk */
	0x172,		/* gcc_qupv3_wrap1_s0_clk */
	0x173,		/* gcc_qupv3_wrap1_s1_clk */
	0x174,		/* gcc_qupv3_wrap1_s2_clk */
	0x176,		/* gcc_qupv3_wrap1_s3_clk */
	0x178,		/* gcc_qupv3_wrap1_s4_clk */
	0x179,		/* gcc_qupv3_wrap1_s5_clk */
	0x17A,		/* gcc_qupv3_wrap1_s6_clk */
	0x17B,		/* gcc_qupv3_wrap1_s7_clk */
	0x17F,		/* gcc_qupv3_wrap2_core_2x_clk */
	0x17E,		/* gcc_qupv3_wrap2_core_clk */
	0x183,		/* gcc_qupv3_wrap2_qspi_s2_clk */
	0x185,		/* gcc_qupv3_wrap2_qspi_s3_clk */
	0x180,		/* gcc_qupv3_wrap2_s0_clk */
	0x181,		/* gcc_qupv3_wrap2_s1_clk */
	0x182,		/* gcc_qupv3_wrap2_s2_clk */
	0x184,		/* gcc_qupv3_wrap2_s3_clk */
	0x186,		/* gcc_qupv3_wrap2_s4_clk */
	0x187,		/* gcc_qupv3_wrap2_s5_clk */
	0x188,		/* gcc_qupv3_wrap2_s6_clk */
	0x189,		/* gcc_qupv3_wrap2_s7_clk */
	0x160,		/* gcc_qupv3_wrap_0_m_ahb_clk */
	0x161,		/* gcc_qupv3_wrap_0_s_ahb_clk */
	0x16E,		/* gcc_qupv3_wrap_1_m_ahb_clk */
	0x16F,		/* gcc_qupv3_wrap_1_s_ahb_clk */
	0x17C,		/* gcc_qupv3_wrap_2_m_ahb_clk */
	0x17D,		/* gcc_qupv3_wrap_2_s_ahb_clk */
	0x15B,		/* gcc_sdcc2_ahb_clk */
	0x15A,		/* gcc_sdcc2_apps_clk */
	0x15E,		/* gcc_sdcc4_ahb_clk */
	0x15D,		/* gcc_sdcc4_apps_clk */
	0xF,		/* gcc_sys_noc_usb_axi_clk */
	0x23A,		/* gcc_ufs_phy_ahb_clk */
	0x239,		/* gcc_ufs_phy_axi_clk */
	0x240,		/* gcc_ufs_phy_ice_core_clk */
	0x241,		/* gcc_ufs_phy_phy_aux_clk */
	0x23C,		/* gcc_ufs_phy_rx_symbol_0_clk */
	0x242,		/* gcc_ufs_phy_rx_symbol_1_clk */
	0x23B,		/* gcc_ufs_phy_tx_symbol_0_clk */
	0x23F,		/* gcc_ufs_phy_unipro_core_clk */
	0x155,		/* gcc_usb20_master_clk */
	0x157,		/* gcc_usb20_mock_utmi_clk */
	0x156,		/* gcc_usb20_sleep_clk */
	0x14A,		/* gcc_usb30_mp_master_clk */
	0x14C,		/* gcc_usb30_mp_mock_utmi_clk */
	0x14B,		/* gcc_usb30_mp_sleep_clk */
	0xD5,		/* gcc_usb30_prim_master_clk */
	0xD7,		/* gcc_usb30_prim_mock_utmi_clk */
	0xD6,		/* gcc_usb30_prim_sleep_clk */
	0xFD,		/* gcc_usb30_sec_master_clk */
	0xFF,		/* gcc_usb30_sec_mock_utmi_clk */
	0xFE,		/* gcc_usb30_sec_sleep_clk */
	0x122,		/* gcc_usb30_tert_master_clk */
	0x124,		/* gcc_usb30_tert_mock_utmi_clk */
	0x123,		/* gcc_usb30_tert_sleep_clk */
	0x14D,		/* gcc_usb3_mp_phy_aux_clk */
	0x14E,		/* gcc_usb3_mp_phy_com_aux_clk */
	0x14F,		/* gcc_usb3_mp_phy_pipe_0_clk */
	0x150,		/* gcc_usb3_mp_phy_pipe_1_clk */
	0xD8,		/* gcc_usb3_prim_phy_aux_clk */
	0xD9,		/* gcc_usb3_prim_phy_com_aux_clk */
	0xDA,		/* gcc_usb3_prim_phy_pipe_clk */
	0x100,		/* gcc_usb3_sec_phy_aux_clk */
	0x101,		/* gcc_usb3_sec_phy_com_aux_clk */
	0x102,		/* gcc_usb3_sec_phy_pipe_clk */
	0x125,		/* gcc_usb3_tert_phy_aux_clk */
	0x126,		/* gcc_usb3_tert_phy_com_aux_clk */
	0x127,		/* gcc_usb3_tert_phy_pipe_clk */
	0xED,		/* gcc_usb4_0_cfg_ahb_clk */
	0xEA,		/* gcc_usb4_0_dp0_clk */
	0xF3,		/* gcc_usb4_0_dp1_clk */
	0xE6,		/* gcc_usb4_0_master_clk */
	0xF2,		/* gcc_usb4_0_phy_p2rr2p_pipe_clk */
	0xE8,		/* gcc_usb4_0_phy_pcie_pipe_clk */
	0xEE,		/* gcc_usb4_0_phy_rx0_clk */
	0xEF,		/* gcc_usb4_0_phy_rx1_clk */
	0xEC,		/* gcc_usb4_0_phy_usb_pipe_clk */
	0xE7,		/* gcc_usb4_0_sb_if_clk */
	0xE9,		/* gcc_usb4_0_sys_clk */
	0xEB,		/* gcc_usb4_0_tmu_clk */
	0x112,		/* gcc_usb4_1_cfg_ahb_clk */
	0x10F,		/* gcc_usb4_1_dp0_clk */
	0x118,		/* gcc_usb4_1_dp1_clk */
	0x10B,		/* gcc_usb4_1_master_clk */
	0x117,		/* gcc_usb4_1_phy_p2rr2p_pipe_clk */
	0x10D,		/* gcc_usb4_1_phy_pcie_pipe_clk */
	0x113,		/* gcc_usb4_1_phy_rx0_clk */
	0x114,		/* gcc_usb4_1_phy_rx1_clk */
	0x111,		/* gcc_usb4_1_phy_usb_pipe_clk */
	0x10C,		/* gcc_usb4_1_sb_if_clk */
	0x10E,		/* gcc_usb4_1_sys_clk */
	0x110,		/* gcc_usb4_1_tmu_clk */
	0x137,		/* gcc_usb4_2_cfg_ahb_clk */
	0x134,		/* gcc_usb4_2_dp0_clk */
	0x13D,		/* gcc_usb4_2_dp1_clk */
	0x130,		/* gcc_usb4_2_master_clk */
	0x13C,		/* gcc_usb4_2_phy_p2rr2p_pipe_clk */
	0x132,		/* gcc_usb4_2_phy_pcie_pipe_clk */
	0x138,		/* gcc_usb4_2_phy_rx0_clk */
	0x139,		/* gcc_usb4_2_phy_rx1_clk */
	0x136,		/* gcc_usb4_2_phy_usb_pipe_clk */
	0x131,		/* gcc_usb4_2_sb_if_clk */
	0x133,		/* gcc_usb4_2_sys_clk */
	0x135,		/* gcc_usb4_2_tmu_clk */
	0xAD,		/* gcc_video_axi0_clk */
	0xAE,		/* gcc_video_axi1_clk */
	0x25B,		/* gpu_cc_debug_mux */
	0x1C0,		/* mc_cc_debug_mux */
	0x16,		/* measure_only_cnoc_clk */
	0x1B8,		/* measure_only_memnoc_clk */
	0xA,		/* measure_only_snoc_clk */
	0xB0,		/* video_cc_debug_mux */
};

static struct clk_debug_mux gcc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x62024,
	.post_div_offset = 0x62000,
	.cbcr_offset = 0x62004,
	.src_sel_mask = 0x3FF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 2,
	.mux_sels = gcc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(gcc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "gcc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gcc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gcc_debug_mux_parent_names),
	},
};

static const char *const gpu_cc_debug_mux_parent_names[] = {
	"gpu_cc_ahb_clk",
	"gpu_cc_crc_ahb_clk",
	"gpu_cc_cx_ff_clk",
	"gpu_cc_cx_gmu_clk",
	"gpu_cc_cxo_aon_clk",
	"gpu_cc_cxo_clk",
	"gpu_cc_demet_clk",
	"gpu_cc_freq_measure_clk",
	"gpu_cc_gx_gmu_clk",
	"gpu_cc_gx_vsense_clk",
	"gpu_cc_hub_aon_clk",
	"gpu_cc_hub_cx_int_clk",
	"gpu_cc_memnoc_gfx_clk",
	"gpu_cc_mnd1x_0_gfx3d_clk",
	"gpu_cc_mnd1x_1_gfx3d_clk",
	"measure_only_gpu_cc_cb_clk",
	"measure_only_gpu_cc_cx_gfx3d_clk",
	"measure_only_gpu_cc_cx_gfx3d_slv_clk",
	"measure_only_gpu_cc_gx_gfx3d_clk",
	"measure_only_gpu_cc_gx_gfx3d_rdvm_clk",
	"measure_only_gpu_cc_sleep_clk",
};

static int gpu_cc_debug_mux_sels[] = {
	0x16,		/* gpu_cc_ahb_clk */
	0x17,		/* gpu_cc_crc_ahb_clk */
	0x20,		/* gpu_cc_cx_ff_clk */
	0x1D,		/* gpu_cc_cx_gmu_clk */
	0xB,		/* gpu_cc_cxo_aon_clk */
	0x1E,		/* gpu_cc_cxo_clk */
	0xD,		/* gpu_cc_demet_clk */
	0xC,		/* gpu_cc_freq_measure_clk */
	0x12,		/* gpu_cc_gx_gmu_clk */
	0xF,		/* gpu_cc_gx_vsense_clk */
	0x2D,		/* gpu_cc_hub_aon_clk */
	0x1F,		/* gpu_cc_hub_cx_int_clk */
	0x21,		/* gpu_cc_memnoc_gfx_clk */
	0x28,		/* gpu_cc_mnd1x_0_gfx3d_clk */
	0x29,		/* gpu_cc_mnd1x_1_gfx3d_clk */
	0x2C,		/* measure_only_gpu_cc_cb_clk */
	0x24,		/* measure_only_gpu_cc_cx_gfx3d_clk */
	0x25,		/* measure_only_gpu_cc_cx_gfx3d_slv_clk */
	0xE,		/* measure_only_gpu_cc_gx_gfx3d_clk */
	0x15,		/* measure_only_gpu_cc_gx_gfx3d_rdvm_clk */
	0x1B,		/* measure_only_gpu_cc_sleep_clk */
};

static struct clk_debug_mux gpu_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x9564,
	.post_div_offset = 0x9270,
	.cbcr_offset = 0x9274,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 2,
	.mux_sels = gpu_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(gpu_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "gpu_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gpu_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gpu_cc_debug_mux_parent_names),
	},
};

static const char *const video_cc_debug_mux_parent_names[] = {
	"measure_only_video_cc_ahb_clk",
	"measure_only_video_cc_sleep_clk",
	"measure_only_video_cc_xo_clk",
	"video_cc_mvs0_clk",
	"video_cc_mvs0c_clk",
	"video_cc_mvs1_clk",
	"video_cc_mvs1c_clk",
};

static int video_cc_debug_mux_sels[] = {
	0x7,		/* measure_only_video_cc_ahb_clk */
	0xC,		/* measure_only_video_cc_sleep_clk */
	0xB,		/* measure_only_video_cc_xo_clk */
	0x3,		/* video_cc_mvs0_clk */
	0x1,		/* video_cc_mvs0c_clk */
	0x5,		/* video_cc_mvs1_clk */
	0x9,		/* video_cc_mvs1c_clk */
};

static struct clk_debug_mux video_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x9A4C,
	.post_div_offset = 0x80F8,
	.cbcr_offset = 0x80FC,
	.src_sel_mask = 0x3F,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 3,
	.mux_sels = video_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(video_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "video_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = video_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(video_cc_debug_mux_parent_names),
	},
};

static const char *const mc_cc_debug_mux_parent_names[] = {
	"measure_only_mccc_clk",
};

static struct clk_debug_mux mc_cc_debug_mux = {
	.period_offset = 0x50,
	.hw.init = &(struct clk_init_data){
		.name = "mc_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = mc_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(mc_cc_debug_mux_parent_names),
	},
};

static struct mux_regmap_names mux_list[] = {
	{ .mux = &mc_cc_debug_mux, .regmap_name = "qcom,mccc" },
	{ .mux = &video_cc_debug_mux, .regmap_name = "qcom,videocc" },
	{ .mux = &gpu_cc_debug_mux, .regmap_name = "qcom,gpucc" },
	{ .mux = &disp_cc_debug_mux, .regmap_name = "qcom,dispcc" },
	{ .mux = &cam_cc_debug_mux, .regmap_name = "qcom,camcc" },
	{ .mux = &av1e_cc_debug_mux, .regmap_name = "qcom,av1ecc" },
	{ .mux = &gcc_debug_mux, .regmap_name = "qcom,gcc" },
};

static struct clk_dummy measure_only_av1e_cc_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_av1e_cc_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_av1e_cc_av1e_cc_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_av1e_cc_av1e_cc_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_av1e_cc_av1e_noc_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_av1e_cc_av1e_noc_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_av1e_cc_sleep_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_av1e_cc_sleep_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_cam_cc_gdsc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_cam_cc_gdsc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_cam_cc_sleep_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_cam_cc_sleep_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_cnoc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_cnoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_disp_cc_mdss_rscc_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_disp_cc_mdss_rscc_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_disp_cc_mdss_rscc_vsync_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_disp_cc_mdss_rscc_vsync_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_disp_cc_sleep_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_disp_cc_sleep_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_disp_cc_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_disp_cc_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_av1e_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_av1e_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_av1e_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_av1e_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_camera_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_camera_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_camera_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_camera_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_disp_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_disp_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_disp_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_disp_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_gpu_cfg_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_gpu_cfg_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_video_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_video_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_video_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_video_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_cb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gpu_cc_cb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_cx_gfx3d_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gpu_cc_cx_gfx3d_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_cx_gfx3d_slv_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gpu_cc_cx_gfx3d_slv_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_gx_gfx3d_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gpu_cc_gx_gfx3d_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_gx_gfx3d_rdvm_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gpu_cc_gx_gfx3d_rdvm_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_sleep_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gpu_cc_sleep_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_mccc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_mccc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_memnoc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_memnoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ncc0_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_ncc0_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ncc1_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_ncc1_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ncc2_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_ncc2_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_3_phy_aux_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_3_phy_aux_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_3_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_3_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_4_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_4_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_5_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_5_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_6a_phy_aux_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_6a_phy_aux_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_6a_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_6a_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_6b_phy_aux_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_6b_phy_aux_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pcie_6b_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pcie_6b_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_qusb4phy_0_gcc_usb4_rx0_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_qusb4phy_0_gcc_usb4_rx0_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_qusb4phy_0_gcc_usb4_rx1_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_qusb4phy_0_gcc_usb4_rx1_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_qusb4phy_1_gcc_usb4_rx0_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_qusb4phy_1_gcc_usb4_rx0_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_qusb4phy_1_gcc_usb4_rx1_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_qusb4phy_1_gcc_usb4_rx1_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_qusb4phy_2_gcc_usb4_rx0_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_qusb4phy_2_gcc_usb4_rx0_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_qusb4phy_2_gcc_usb4_rx1_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_qusb4phy_2_gcc_usb4_rx1_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_snoc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_snoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ufs_phy_rx_symbol_0_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_ufs_phy_rx_symbol_0_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ufs_phy_rx_symbol_1_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_ufs_phy_rx_symbol_1_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ufs_phy_tx_symbol_0_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_ufs_phy_tx_symbol_0_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb3_phy_0_wrapper_gcc_usb30_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb3_phy_0_wrapper_gcc_usb30_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb3_phy_1_wrapper_gcc_usb30_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb3_phy_1_wrapper_gcc_usb30_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb3_phy_2_wrapper_gcc_usb30_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb3_phy_2_wrapper_gcc_usb30_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb3_uni_phy_mp_gcc_usb30_pipe_0_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb3_uni_phy_mp_gcc_usb30_pipe_0_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb3_uni_phy_mp_gcc_usb30_pipe_1_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb3_uni_phy_mp_gcc_usb30_pipe_1_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb4_0_phy_gcc_usb4_pcie_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb4_0_phy_gcc_usb4_pcie_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb4_0_phy_gcc_usb4rtr_max_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb4_0_phy_gcc_usb4rtr_max_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb4_1_phy_gcc_usb4_pcie_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb4_1_phy_gcc_usb4_pcie_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb4_1_phy_gcc_usb4rtr_max_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb4_1_phy_gcc_usb4rtr_max_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb4_2_phy_gcc_usb4_pcie_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb4_2_phy_gcc_usb4_pcie_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb4_2_phy_gcc_usb4rtr_max_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb4_2_phy_gcc_usb4rtr_max_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_video_cc_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_video_cc_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_video_cc_sleep_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_video_cc_sleep_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_video_cc_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_video_cc_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_hw *debugcc_x1e80100_hws[] = {
	&measure_only_av1e_cc_ahb_clk.hw,
	&measure_only_av1e_cc_av1e_cc_xo_clk.hw,
	&measure_only_av1e_cc_av1e_noc_ahb_clk.hw,
	&measure_only_av1e_cc_sleep_clk.hw,
	&measure_only_cam_cc_gdsc_clk.hw,
	&measure_only_cam_cc_sleep_clk.hw,
	&measure_only_cnoc_clk.hw,
	&measure_only_disp_cc_mdss_rscc_ahb_clk.hw,
	&measure_only_disp_cc_mdss_rscc_vsync_clk.hw,
	&measure_only_disp_cc_sleep_clk.hw,
	&measure_only_disp_cc_xo_clk.hw,
	&measure_only_gcc_av1e_ahb_clk.hw,
	&measure_only_gcc_av1e_xo_clk.hw,
	&measure_only_gcc_camera_ahb_clk.hw,
	&measure_only_gcc_camera_xo_clk.hw,
	&measure_only_gcc_disp_ahb_clk.hw,
	&measure_only_gcc_disp_xo_clk.hw,
	&measure_only_gcc_gpu_cfg_ahb_clk.hw,
	&measure_only_gcc_video_ahb_clk.hw,
	&measure_only_gcc_video_xo_clk.hw,
	&measure_only_gpu_cc_cb_clk.hw,
	&measure_only_gpu_cc_cx_gfx3d_clk.hw,
	&measure_only_gpu_cc_cx_gfx3d_slv_clk.hw,
	&measure_only_gpu_cc_gx_gfx3d_clk.hw,
	&measure_only_gpu_cc_gx_gfx3d_rdvm_clk.hw,
	&measure_only_gpu_cc_sleep_clk.hw,
	&measure_only_mccc_clk.hw,
	&measure_only_memnoc_clk.hw,
	&measure_only_ncc0_clk.hw,
	&measure_only_ncc1_clk.hw,
	&measure_only_ncc2_clk.hw,
	&measure_only_pcie_3_phy_aux_clk.hw,
	&measure_only_pcie_3_pipe_clk.hw,
	&measure_only_pcie_4_pipe_clk.hw,
	&measure_only_pcie_5_pipe_clk.hw,
	&measure_only_pcie_6a_phy_aux_clk.hw,
	&measure_only_pcie_6a_pipe_clk.hw,
	&measure_only_pcie_6b_phy_aux_clk.hw,
	&measure_only_pcie_6b_pipe_clk.hw,
	&measure_only_qusb4phy_0_gcc_usb4_rx0_clk.hw,
	&measure_only_qusb4phy_0_gcc_usb4_rx1_clk.hw,
	&measure_only_qusb4phy_1_gcc_usb4_rx0_clk.hw,
	&measure_only_qusb4phy_1_gcc_usb4_rx1_clk.hw,
	&measure_only_qusb4phy_2_gcc_usb4_rx0_clk.hw,
	&measure_only_qusb4phy_2_gcc_usb4_rx1_clk.hw,
	&measure_only_snoc_clk.hw,
	&measure_only_ufs_phy_rx_symbol_0_clk.hw,
	&measure_only_ufs_phy_rx_symbol_1_clk.hw,
	&measure_only_ufs_phy_tx_symbol_0_clk.hw,
	&measure_only_usb3_phy_0_wrapper_gcc_usb30_pipe_clk.hw,
	&measure_only_usb3_phy_1_wrapper_gcc_usb30_pipe_clk.hw,
	&measure_only_usb3_phy_2_wrapper_gcc_usb30_pipe_clk.hw,
	&measure_only_usb3_uni_phy_mp_gcc_usb30_pipe_0_clk.hw,
	&measure_only_usb3_uni_phy_mp_gcc_usb30_pipe_1_clk.hw,
	&measure_only_usb4_0_phy_gcc_usb4_pcie_pipe_clk.hw,
	&measure_only_usb4_0_phy_gcc_usb4rtr_max_pipe_clk.hw,
	&measure_only_usb4_1_phy_gcc_usb4_pcie_pipe_clk.hw,
	&measure_only_usb4_1_phy_gcc_usb4rtr_max_pipe_clk.hw,
	&measure_only_usb4_2_phy_gcc_usb4_pcie_pipe_clk.hw,
	&measure_only_usb4_2_phy_gcc_usb4rtr_max_pipe_clk.hw,
	&measure_only_video_cc_ahb_clk.hw,
	&measure_only_video_cc_sleep_clk.hw,
	&measure_only_video_cc_xo_clk.hw,
};

static const struct of_device_id clk_debug_match_table[] = {
	{ .compatible = "qcom,x1e80100-debugcc" },
	{ }
};

static int clk_debug_x1e80100_probe(struct platform_device *pdev)
{
	struct clk *clk;
	int ret = 0, i;

	BUILD_BUG_ON(ARRAY_SIZE(av1e_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(av1e_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(cam_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(cam_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(disp_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(disp_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(gcc_debug_mux_parent_names) != ARRAY_SIZE(gcc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(gpu_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(gpu_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(video_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(video_cc_debug_mux_sels));

	clk = devm_clk_get(&pdev->dev, "xo_clk_src");
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Unable to get xo clock\n");
		return PTR_ERR(clk);
	}

	debug_mux_priv.cxo = clk;

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		if (IS_ERR_OR_NULL(mux_list[i].mux->regmap)) {
			ret = map_debug_bases(pdev, mux_list[i].regmap_name,
					      mux_list[i].mux);
			if (ret == -EBADR)
				continue;
			else if (ret)
				return ret;
		}
	}

	for (i = 0; i < ARRAY_SIZE(debugcc_x1e80100_hws); i++) {
		clk = devm_clk_register(&pdev->dev, debugcc_x1e80100_hws[i]);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register %s, err:(%ld)\n",
				qcom_clk_hw_get_name(debugcc_x1e80100_hws[i]),
				PTR_ERR(clk));
			return PTR_ERR(clk);
		}
	}

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		ret = devm_clk_register_debug_mux(&pdev->dev, mux_list[i].mux);
		if (ret) {
			dev_err(&pdev->dev, "Unable to register mux clk %s, err:(%d)\n",
				qcom_clk_hw_get_name(&mux_list[i].mux->hw),
				ret);
			return ret;
		}
	}

	ret = clk_debug_measure_register(&gcc_debug_mux.hw);
	if (ret) {
		dev_err(&pdev->dev, "Could not register Measure clocks\n");
		return ret;
	}

	dev_info(&pdev->dev, "Registered debug measure clocks\n");

	return ret;
}

static struct platform_driver clk_debug_driver = {
	.probe = clk_debug_x1e80100_probe,
	.driver = {
		.name = "x1e80100-debugcc",
		.of_match_table = clk_debug_match_table,
	},
};

static int __init clk_debug_x1e80100_init(void)
{
	return platform_driver_register(&clk_debug_driver);
}
fs_initcall(clk_debug_x1e80100_init);

MODULE_DESCRIPTION("QTI DEBUG CC X1E80100 Driver");
MODULE_LICENSE("GPL");
