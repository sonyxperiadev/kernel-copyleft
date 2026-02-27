/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __TPG_HW_V_1_4_H__
#define __TPG_HW_V_1_4_H__

#include "../tpg_hw.h"

struct cam_tpg_ver_1_4_reg_offset {
	/* Register offsets below */
	int hw_version;
	int hw_status;
	int dmi_cfg;
	int dmi_lut_cfg;
	int dmi_data;
	int dmi_data_1;
	int dmi_data_2;
	int dmi_data_3;
	int dmi_data_4;
	int dmi_data_5;
	int dmi_data_6;
	int dmi_data_7;
	int dmi_data_8;
	int dmi_data_9;
	int dmi_data_10;
	int dmi_data_11;
	int dmi_data_12;
	int dmi_data_13;
	int dmi_data_14;
	int dmi_data_15;
	int dmi_cmd;
	int dmi_status;
	int dmi_lut_bank_cfg;
	int module_lut_bank_cfg;
	int tpg_vc0_gain_cfg;
	int tpg_ctrl;
	int tpg_vc0_cfg0;
	int tpg_vc0_lfsr_seed;
	int tpg_vc0_hbi_cfg;
	int tpg_vc0_vbi_cfg;
	int tpg_vc0_color_bars_cfg;
	int tpg_vc0_dt_0_cfg_0;
	int tpg_vc0_dt_0_cfg_1;
	int tpg_vc0_dt_0_cfg_2;
	int tpg_vc0_dt_1_cfg_0;
	int tpg_vc0_dt_1_cfg_1;
	int tpg_vc0_dt_1_cfg_2;
	int tpg_vc0_dt_2_cfg_0;
	int tpg_vc0_dt_2_cfg_1;
	int tpg_vc0_dt_2_cfg_2;
	int tpg_vc0_dt_3_cfg_0;
	int tpg_vc0_dt_3_cfg_1;
	int tpg_vc0_dt_3_cfg_2;
	int tpg_vc0_throttle;
	int tpg_vc0_color_bar_cfa_color0;
	int tpg_vc0_color_bar_cfa_color1;
	int tpg_vc0_color_bar_cfa_color2;
	int tpg_vc0_color_bar_cfa_color3;
	int tpg_vc1_gain_cfg;
	int tpg_vc1_shdr_cfg;
	int tpg_vc1_cfg0;
	int tpg_vc1_lfsr_seed;
	int tpg_vc1_hbi_cfg;
	int tpg_vc1_vbi_cfg;
	int tpg_vc1_color_bars_cfg;
	int tpg_vc1_dt_0_cfg_0;
	int tpg_vc1_dt_0_cfg_1;
	int tpg_vc1_dt_0_cfg_2;
	int tpg_vc1_dt_1_cfg_0;
	int tpg_vc1_dt_1_cfg_1;
	int tpg_vc1_dt_1_cfg_2;
	int tpg_vc1_dt_2_cfg_0;
	int tpg_vc1_dt_2_cfg_1;
	int tpg_vc1_dt_2_cfg_2;
	int tpg_vc1_dt_3_cfg_0;
	int tpg_vc1_dt_3_cfg_1;
	int tpg_vc1_dt_3_cfg_2;
	int tpg_vc1_throttle;
	int tpg_vc1_color_bar_cfa_color0;
	int tpg_vc1_color_bar_cfa_color1;
	int tpg_vc1_color_bar_cfa_color2;
	int tpg_vc1_color_bar_cfa_color3;
	int tpg_vc2_gain_cfg;
	int tpg_vc2_shdr_cfg;
	int tpg_vc2_cfg0;
	int tpg_vc2_lfsr_seed;
	int tpg_vc2_hbi_cfg;
	int tpg_vc2_vbi_cfg;
	int tpg_vc2_color_bars_cfg;
	int tpg_vc2_dt_0_cfg_0;
	int tpg_vc2_dt_0_cfg_1;
	int tpg_vc2_dt_0_cfg_2;
	int tpg_vc2_dt_1_cfg_0;
	int tpg_vc2_dt_1_cfg_1;
	int tpg_vc2_dt_1_cfg_2;
	int tpg_vc2_dt_2_cfg_0;
	int tpg_vc2_dt_2_cfg_1;
	int tpg_vc2_dt_2_cfg_2;
	int tpg_vc2_dt_3_cfg_0;
	int tpg_vc2_dt_3_cfg_1;
	int tpg_vc2_dt_3_cfg_2;
	int tpg_vc2_throttle;
	int tpg_vc2_color_bar_cfa_color0;
	int tpg_vc2_color_bar_cfa_color1;
	int tpg_vc2_color_bar_cfa_color2;
	int tpg_vc2_color_bar_cfa_color3;
	int tpg_vc3_gain_cfg;
	int tpg_vc3_shdr_cfg;
	int tpg_vc3_cfg0;
	int tpg_vc3_lfsr_seed;
	int tpg_vc3_hbi_cfg;
	int tpg_vc3_vbi_cfg;
	int tpg_vc3_color_bars_cfg;
	int tpg_vc3_dt_0_cfg_0;
	int tpg_vc3_dt_0_cfg_1;
	int tpg_vc3_dt_0_cfg_2;
	int tpg_vc3_dt_1_cfg_0;
	int tpg_vc3_dt_1_cfg_1;
	int tpg_vc3_dt_1_cfg_2;
	int tpg_vc3_dt_2_cfg_0;
	int tpg_vc3_dt_2_cfg_1;
	int tpg_vc3_dt_2_cfg_2;
	int tpg_vc3_dt_3_cfg_0;
	int tpg_vc3_dt_3_cfg_1;
	int tpg_vc3_dt_3_cfg_2;
	int tpg_vc3_throttle;
	int tpg_vc3_color_bar_cfa_color0;
	int tpg_vc3_color_bar_cfa_color1;
	int tpg_vc3_color_bar_cfa_color2;
	int tpg_vc3_color_bar_cfa_color3;
	int top_irq_status;
	int top_irq_mask;
	int top_irq_clear;
	int top_irq_set;
	int irq_cmd;
	int tpg_ctrl_cmd;
	int test_bus_ctrl;
	int spare;
	/* Register fields below */

	int gen_shift;
	int rev_shift;
	int step_shift;
	int violation_shift;
	int auto_load_pattern_shift;
	int auto_load_en_shift;
	int addr_shift;
	int lut_sel_shift;
	int data_shift;
	int auto_load_status_clr_shift;
	int auto_load_cmd_shift;
	int auto_load_done_shift;
	int bank_sel_shift;
	int gain_shift;
	int num_active_vc_shift;
	int overlap_shdr_en_shift;
	int vc_dt_pattern_id_shift;
	int num_active_lanes_shift;
	int phy_sel_shift;
	int num_frames_shift;
	int num_batch_shift;
	int num_active_dt_shift;
	int fe_dis_shift;
	int fs_dis_shift;
	int vc_num_shift;
	int seed_shift;
	int hbi_clk_cnt_shift;
	int vbi_line_cnt_shift;
	int size_y_shift;
	int size_x_shift;
	int xcfa_en_shift;
	int rotate_period_shift;
	int pix_intl_hdr_mode_shift;
	int noise_en_shift;
	int split_en_shift;
	int qcfa_en_shift;
	int pix_pattern_shift;
	int frame_width_shift;
	int frame_height_shift;
	int crc_xor_mask_shift;
	int ecc_xor_mask_shift;
	int nfi_ssm_mode_en_shift;
	int data_type_shift;
	int encode_format_shift;
	int user_specified_payload_shift;
	int payload_mode_shift;
	int pattern_shift;
	int array15_shift;
	int array14_shift;
	int array13_shift;
	int array12_shift;
	int array11_shift;
	int array10_shift;
	int array9_shift;
	int array8_shift;
	int array7_shift;
	int array6_shift;
	int array5_shift;
	int array4_shift;
	int array3_shift;
	int array2_shift;
	int array1_shift;
	int array0_shift;
	int array31_shift;
	int array30_shift;
	int array29_shift;
	int array28_shift;
	int array27_shift;
	int array26_shift;
	int array25_shift;
	int array24_shift;
	int array23_shift;
	int array22_shift;
	int array21_shift;
	int array20_shift;
	int array19_shift;
	int array18_shift;
	int array17_shift;
	int array16_shift;
	int array47_shift;
	int array46_shift;
	int array45_shift;
	int array44_shift;
	int array43_shift;
	int array42_shift;
	int array41_shift;
	int array40_shift;
	int array39_shift;
	int array38_shift;
	int array37_shift;
	int array36_shift;
	int array35_shift;
	int array34_shift;
	int array33_shift;
	int array32_shift;
	int array63_shift;
	int array62_shift;
	int array61_shift;
	int array60_shift;
	int array59_shift;
	int array58_shift;
	int array57_shift;
	int array56_shift;
	int array55_shift;
	int array54_shift;
	int array53_shift;
	int array52_shift;
	int array51_shift;
	int array50_shift;
	int array49_shift;
	int array48_shift;
	int shdr_offset_num_batch_shift;
	int shdr_line_offset1_shift;
	int shdr_line_offset0_shift;
	int tpg_done_status_shift;
	int rup_done_status_shift;
	int status_vec_shift;
	int rup_done_mask_vec_shift;
	int tpg_done_mask_vec_shift;
	int rup_done_clear_vec_shift;
	int tpg_done_clear_vec_shift;
	int set_vec_shift;
	int set_shift;
	int clear_shift;
	int test_en_cmd_shift;
	int hw_reset_shift;
	int test_bus_en_shift;
	int test_bus_sel_shift;
	int spare_shift;
	/* Custome Variables below */
};

/**
 * @brief initialize the tpg hw instance
 *
 * @param hw   : tpg hw instance
 * @param data : argument for initialize
 *
 * @return     : 0 on success
 */
int tpg_hw_v_1_4_init(struct tpg_hw *hw, void *data);

/**
 * @brief start tpg hw
 *
 * @param hw    : tpg hw instance
 * @param data  : tpg hw instance data
 *
 * @return      : 0 on success
 */
int tpg_hw_v_1_4_start(struct tpg_hw *hw, void *data);

/**
 * @brief stop tpg hw
 *
 * @param hw   : tpg hw instance
 * @param data : argument for tpg hw stop
 *
 * @return     : 0 on success
 */
int tpg_hw_v_1_4_stop(struct tpg_hw *hw, void *data);

/**
 * @brief process a command send from hw layer
 *
 * @param hw  : tpg hw instance
 * @param cmd : command to process
 * @param arg : argument corresponding to command
 *
 * @return    : 0 on success
 */
int tpg_hw_v_1_4_process_cmd(struct tpg_hw *hw,
		uint32_t cmd, void *arg);

/**
 * @brief  dump hw status registers
 *
 * @param hw   : tpg hw instance
 * @param data : argument for status dump
 *
 * @return     : 0 on success
 */
int tpg_hw_v_1_4_dump_status(struct tpg_hw *hw, void *data);

/**
 * @brief  hw layer initialization
 *
 * @param hw   : tpg hw instance
 *
 * @return     : 0 on success
 */
int tpg_1_4_layer_init(struct tpg_hw *hw);

/**
 * @brief handle tpg irq
 *
 * @param hw   : tpg hw instance
 *
 * @return     : IRQ_HANDLED on success
 */
irqreturn_t tpg_hw_v_1_4_handle_irq(struct tpg_hw *hw);

#endif /* __TPG_HW_V_1_4_H__ */
