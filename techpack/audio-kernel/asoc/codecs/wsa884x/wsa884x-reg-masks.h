/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#ifndef WSA884X_REG_MASKS_H
#define WSA884X_REG_MASKS_H
#include <linux/regmap.h>
#include <linux/device.h>
#include "wsa884x-registers.h"

/*
 * Use in conjunction with wsa884x-reg-shifts.c for field values.
 * field_value = (register_value & field_mask) >> field_shift
 */

#define FIELD_MASK(register_name, field_name) \
	WSA884X_##register_name##_##field_name##_MASK

/* WSA884X_BOP2_PROG Fields: */
#define WSA884X_BOP2_PROG_BOP2_VTH_MASK                                  0xf0
#define WSA884X_BOP2_PROG_BOP2_HYST_MASK                                 0x0f
/* WSA884X_VSENSE1 Fields: */
#define WSA884X_VSENSE1_GAIN_VSENSE_FE_MASK                              0xe0
#define WSA884X_VSENSE1_VSENSE_AMP_IQ_CTL_1_MASK                         0x10
#define WSA884X_VSENSE1_IDLE_MODE_CTL_MASK                               0x0c
#define WSA884X_VSENSE1_VOCM_AMP_CTL_MASK                                0x03
/* WSA884X_ISENSE2 Fields: */
#define WSA884X_ISENSE2_ISENSE_GAIN_CTL_MASK                             0xe0
#define WSA884X_ISENSE2_SUMAMP_IQ_CTL_MASK                               0x10
#define WSA884X_ISENSE2_SPARE_BITS_3_0_MASK                              0x0f
/* WSA884X_ADC_2 Fields: */
#define WSA884X_ADC_2_ATEST_SEL_CAL_REF_MASK                             0x80
#define WSA884X_ADC_2_ISNS_LOAD_STORED_MASK                              0x40
#define WSA884X_ADC_2_EN_DET_MASK                                        0x20
#define WSA884X_ADC_2_EN_ATEST_REF_MASK                                  0x10
#define WSA884X_ADC_2_EN_ATEST_INT_MASK                                  0x0e
#define WSA884X_ADC_2_D_ADC_REG_EN_MASK                                  0x01
/* WSA884X_ADC_7 Fields: */
#define WSA884X_ADC_7_CLAMPON_MASK                                       0x80
#define WSA884X_ADC_7_CAL_LOOP_TRIM_MASK                                 0x70
#define WSA884X_ADC_7_REG_TRIM_EN_MASK                                   0x08
#define WSA884X_ADC_7_EN_AZ_REG_MASK                                     0x04
#define WSA884X_ADC_7_EN_SAR_REG_MASK                                    0x02
#define WSA884X_ADC_7_EN_SW_CURRENT_REG_MASK                             0x01
/* WSA884X_TOP_CTRL1 Fields: */
#define WSA884X_TOP_CTRL1_IDLE_PWRSAV_OVERRIDE_MASK                      0x80
#define WSA884X_TOP_CTRL1_DAC_LDO_PROG_MASK                              0x60
#define WSA884X_TOP_CTRL1_DATA_INV_MASK                                  0x10
#define WSA884X_TOP_CTRL1_DATA_RESET_MASK                                0x08
#define WSA884X_TOP_CTRL1_CLK_DIV2_MASK                                  0x04
#define WSA884X_TOP_CTRL1_CLK_INV_MASK                                   0x02
#define WSA884X_TOP_CTRL1_OCP_LOWVBAT_ITH_SEL_EN_MASK                    0x01
/* WSA884X_BOP_DEGLITCH_CTL Fields: */
#define WSA884X_BOP_DEGLITCH_CTL_BOP_DEGLITCH_SETTING_MASK               0x1e
#define WSA884X_BOP_DEGLITCH_CTL_BOP_DEGLITCH_EN_MASK                    0x01
/* WSA884X_CDC_SPK_DSM_A2_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A2_0_COEF_A2_MASK                            0xff
/* WSA884X_CDC_SPK_DSM_A2_1 Fields: */
#define WSA884X_CDC_SPK_DSM_A2_1_COEF_A2_MASK                            0x0f
/* WSA884X_CDC_SPK_DSM_A3_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A3_0_COEF_A3_MASK                            0xff
/* WSA884X_CDC_SPK_DSM_A3_1 Fields: */
#define WSA884X_CDC_SPK_DSM_A3_1_COEF_A3_MASK                            0x07
/* WSA884X_CDC_SPK_DSM_A4_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A4_0_COEF_A4_MASK                            0xff
/* WSA884X_CDC_SPK_DSM_A5_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A5_0_COEF_A5_MASK                            0xff
/* WSA884X_CDC_SPK_DSM_A6_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A6_0_COEF_A6_MASK                            0xff
/* WSA884X_CDC_SPK_DSM_A7_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A7_0_COEF_A7_MASK                            0xff
/* WSA884X_CDC_SPK_DSM_C_0 Fields: */
#define WSA884X_CDC_SPK_DSM_C_0_COEF_C3_MASK                             0xf0
#define WSA884X_CDC_SPK_DSM_C_0_COEF_C2_MASK                             0x0f
/* WSA884X_CDC_SPK_DSM_C_2 Fields: */
#define WSA884X_CDC_SPK_DSM_C_2_COEF_C7_MASK                             0xf0
#define WSA884X_CDC_SPK_DSM_C_2_COEF_C6_MASK                             0x0f
/* WSA884X_CDC_SPK_DSM_C_3 Fields: */
#define WSA884X_CDC_SPK_DSM_C_3_COEF_C7_MASK                             0x3f
/* WSA884X_CDC_SPK_DSM_R1 Fields: */
#define WSA884X_CDC_SPK_DSM_R1_SAT_LIMIT_R1_MASK                         0xff
/* WSA884X_CDC_SPK_DSM_R2 Fields: */
#define WSA884X_CDC_SPK_DSM_R2_SAT_LIMIT_R2_MASK                         0xff
/* WSA884X_CDC_SPK_DSM_R3 Fields: */
#define WSA884X_CDC_SPK_DSM_R3_SAT_LIMIT_R3_MASK                         0xff
/* WSA884X_CDC_SPK_DSM_R4 Fields: */
#define WSA884X_CDC_SPK_DSM_R4_SAT_LIMIT_R4_MASK                         0xff
/* WSA884X_CDC_SPK_DSM_R5 Fields: */
#define WSA884X_CDC_SPK_DSM_R5_SAT_LIMIT_R5_MASK                         0xff
/* WSA884X_CDC_SPK_DSM_R6 Fields: */
#define WSA884X_CDC_SPK_DSM_R6_SAT_LIMIT_R6_MASK                         0xff
/* WSA884X_CDC_SPK_DSM_R7 Fields: */
#define WSA884X_CDC_SPK_DSM_R7_SAT_LIMIT_R7_MASK                         0xff
/* WSA884X_DRE_CTL_0 Fields: */
#define WSA884X_DRE_CTL_0_PROG_DELAY_MASK                                0xf0
#define WSA884X_DRE_CTL_0_OFFSET_MASK                                    0x07
/* WSA884X_GAIN_RAMPING_MIN Fields: */
#define WSA884X_GAIN_RAMPING_MIN_MIN_GAIN_MASK                           0x1f
/* WSA884X_CLSH_SOFT_MAX Fields: */
#define WSA884X_CLSH_SOFT_MAX_SOFT_MAX_MASK                              0xff
/* WSA884X_CLSH_VTH1 Fields: */
#define WSA884X_CLSH_VTH1_CLSH_VTH1_MASK                                 0xff
/* WSA884X_CLSH_VTH10 Fields: */
#define WSA884X_CLSH_VTH10_CLSH_VTH10_MASK                               0xff
/* WSA884X_CLSH_VTH11 Fields: */
#define WSA884X_CLSH_VTH11_CLSH_VTH11_MASK                               0xff
/* WSA884X_CLSH_VTH12 Fields: */
#define WSA884X_CLSH_VTH12_CLSH_VTH12_MASK                               0xff
/* WSA884X_CLSH_VTH13 Fields: */
#define WSA884X_CLSH_VTH13_CLSH_VTH13_MASK                               0xff
/* WSA884X_CLSH_VTH14 Fields: */
#define WSA884X_CLSH_VTH14_CLSH_VTH14_MASK                               0xff
/* WSA884X_CLSH_VTH15 Fields: */
#define WSA884X_CLSH_VTH15_CLSH_VTH15_MASK                               0xff
/* WSA884X_ANA_WO_CTL_0 Fields: */
#define WSA884X_ANA_WO_CTL_0_VPHX_SYS_EN_MASK                            0xc0
#define WSA884X_ANA_WO_CTL_0_PA_AUX_GAIN_MASK                            0x3c
#define WSA884X_ANA_WO_CTL_0_PA_MIN_GAIN_BYP_MASK                        0x02
#define WSA884X_ANA_WO_CTL_0_DAC_CM_CLAMP_EN_MASK                        0x01
/* WSA884X_ANA_WO_CTL_1 Fields: */
#define WSA884X_ANA_WO_CTL_1_BOOST_SHARE_EN_MASK                         0x08
#define WSA884X_ANA_WO_CTL_1_EXT_VDDSPK_EN_MASK                          0x07
/* WSA884X_DRE_CTL_1 Fields: */
#define WSA884X_DRE_CTL_1_CSR_GAIN_MASK                                  0x3e
#define WSA884X_DRE_CTL_1_CSR_GAIN_EN_MASK                               0x01
/* WSA884X_VBAT_THRM_FLT_CTL Fields: */
#define WSA884X_VBAT_THRM_FLT_CTL_THRM_COEF_SEL_MASK                     0xe0
#define WSA884X_VBAT_THRM_FLT_CTL_THRM_FLT_EN_MASK                       0x10
#define WSA884X_VBAT_THRM_FLT_CTL_VBAT_COEF_SEL_MASK                     0x0e
#define WSA884X_VBAT_THRM_FLT_CTL_VBAT_FLT_EN_MASK                       0x01
/* WSA884X_PDM_WD_CTL Fields: */
#define WSA884X_PDM_WD_CTL_HOLD_OFF_MASK                                 0x04
#define WSA884X_PDM_WD_CTL_TIME_OUT_SEL_MASK                             0x02
#define WSA884X_PDM_WD_CTL_PDM_WD_EN_MASK                                0x01
/* WSA884X_PA_FSM_BYP_CTL Fields: */
#define WSA884X_PA_FSM_BYP_CTL_PA_FSM_BYP_MASK                           0x01
/* WSA884X_TADC_VALUE_CTL Fields: */
#define WSA884X_TADC_VALUE_CTL_VBAT_VALUE_RD_EN_MASK                     0x02
#define WSA884X_TADC_VALUE_CTL_TEMP_VALUE_RD_EN_MASK                     0x01
/* WSA884X_CDC_PATH_MODE Fields: */
#define WSA884X_CDC_PATH_MODE_RXD_MODE_MASK                              0x02
#define WSA884X_CDC_PATH_MODE_TXD_MODE_MASK                              0x01
/* WSA884X_PA_FSM_BYP0 Fields: */
#define WSA884X_PA_FSM_BYP0_TSADC_EN_MASK                                0x80
#define WSA884X_PA_FSM_BYP0_SPKR_PROT_EN_MASK                            0x40
#define WSA884X_PA_FSM_BYP0_D_UNMUTE_MASK                                0x20
#define WSA884X_PA_FSM_BYP0_PA_EN_MASK                                   0x10
#define WSA884X_PA_FSM_BYP0_BOOST_EN_MASK                                0x08
#define WSA884X_PA_FSM_BYP0_BG_EN_MASK                                   0x04
#define WSA884X_PA_FSM_BYP0_CLK_WD_EN_MASK                               0x02
#define WSA884X_PA_FSM_BYP0_DC_CAL_EN_MASK                               0x01
/* WSA884X_PA_FSM_BYP1 Fields: */
#define WSA884X_PA_FSM_BYP1_NG_MODE_MASK                                 0xc0
#define WSA884X_PA_FSM_BYP1_PWRSAV_CTL_MASK                              0x20
#define WSA884X_PA_FSM_BYP1_RAMP_DOWN_MASK                               0x10
#define WSA884X_PA_FSM_BYP1_RAMP_UP_MASK                                 0x08
#define WSA884X_PA_FSM_BYP1_BLEEDER_EN_MASK                              0x04
#define WSA884X_PA_FSM_BYP1_PA_MAIN_EN_MASK                              0x02
#define WSA884X_PA_FSM_BYP1_PA_AUX_EN_MASK                               0x01
/* WSA884X_PA_FSM_EN Fields: */
#define WSA884X_PA_FSM_EN_GLOBAL_PA_EN_MASK                              0x01
/* WSA884X_OTP_REG_0 Fields: */
#define WSA884X_OTP_REG_0_WSA884X_ID_MASK                                0x0f
/* WSA884X_CHIP_ID0 Fields: */
#define WSA884X_CHIP_ID0_BYTE_0_MASK                                     0xff
/* WSA884X_CHIP_ID1 Fields: */
#define WSA884X_CHIP_ID1_BYTE_1_MASK                                     0xff
/* WSA884X_CHIP_ID2 Fields: */
#define WSA884X_CHIP_ID2_BYTE_2_MASK                                     0xff
/* WSA884X_CHIP_ID3 Fields: */
#define WSA884X_CHIP_ID3_BYTE_3_MASK                                     0xff
/* WSA884X_OCP_CTL Fields: */
#define WSA884X_OCP_CTL_OCP_EN_MASK                                      0x80
#define WSA884X_OCP_CTL_OCP_CURR_LIMIT_MASK                              0x70
#define WSA884X_OCP_CTL_GLITCH_FILTER_MASK                               0x0c
#define WSA884X_OCP_CTL_OCP_P_HS_DLY_CTL_MASK                            0x03
/* WSA884X_ILIM_CTRL1 Fields: */
#define WSA884X_ILIM_CTRL1_EN_AUTO_MAXD_SEL_MASK                         0x80
#define WSA884X_ILIM_CTRL1_EN_ILIM_SW_CLH_MASK                           0x40
#define WSA884X_ILIM_CTRL1_ILIM_OFFSET_CLH_MASK                          0x38
#define WSA884X_ILIM_CTRL1_ILIM_OFFSET_PB_MASK                           0x07
/* WSA884X_CLSH_CTL_0 Fields: */
#define WSA884X_CLSH_CTL_0_CSR_GAIN_EN_MASK                              0x80
#define WSA884X_CLSH_CTL_0_DLY_CODE_MASK                                 0x70
#define WSA884X_CLSH_CTL_0_DLY_RST_MASK                                  0x08
#define WSA884X_CLSH_CTL_0_DLY_EN_MASK                                   0x04
#define WSA884X_CLSH_CTL_0_INPUT_EN_MASK                                 0x02
#define WSA884X_CLSH_CTL_0_CLSH_EN_MASK                                  0x01
/* WSA884X_CLSH_CTL_1 Fields: */
#define WSA884X_CLSH_CTL_1_SLR_MAX_MASK                                  0xf0
#define WSA884X_CLSH_CTL_1_VERF_OVRD_EN_MASK                             0x08
#define WSA884X_CLSH_CTL_1_DECAY_RATE_MASK                               0x07
/* WSA884X_CLSH_V_HD_PA Fields: */
#define WSA884X_CLSH_V_HD_PA_V_HD_PA_MASK                                0x1f
/* WSA884X_UVLO_PROG Fields: */
#define WSA884X_UVLO_PROG_UVLO1_HYST_MASK                                0xf0
#define WSA884X_UVLO_PROG_UVLO1_VTH_MASK                                 0x0f
/* WSA884X_DAC_VCM_CTRL_REG2 Fields: */
#define WSA884X_DAC_VCM_CTRL_REG2_DAC_VCM_SHIFT_MASK                     0xff
/* WSA884X_DAC_VCM_CTRL_REG3 Fields: */
#define WSA884X_DAC_VCM_CTRL_REG3_DAC_VCM_SHIFT_MASK                     0xff
/* WSA884X_DAC_VCM_CTRL_REG4 Fields: */
#define WSA884X_DAC_VCM_CTRL_REG4_DAC_VCM_SHIFT_MASK                     0xff
/* WSA884X_DAC_VCM_CTRL_REG5 Fields: */
#define WSA884X_DAC_VCM_CTRL_REG5_DAC_VCM_SHIFT_MASK                     0xff
/* WSA884X_DAC_VCM_CTRL_REG6 Fields: */
#define WSA884X_DAC_VCM_CTRL_REG6_DAC_VCM_SHIFT_MASK                     0xff
/* WSA884X_DAC_VCM_CTRL_REG7 Fields: */
#define WSA884X_DAC_VCM_CTRL_REG7_SPARE_BITS_7_2_MASK                    0xfc
#define WSA884X_DAC_VCM_CTRL_REG7_DAC_VCM_SHIFT_FINAL_OVERRIDE_MASK      0x02
#define WSA884X_DAC_VCM_CTRL_REG7_DAC_VCM_SHIFT_ZONE_OVERRIDE_MASK       0x01
/* WSA884X_STB_CTRL1 Fields: */
#define WSA884X_STB_CTRL1_SLOPE_COMP_CURRENT_MASK                        0xf8
#define WSA884X_STB_CTRL1_VOUT_FS_MASK                                   0x07
/* WSA884X_OTP_REG_38 Fields: */
#define WSA884X_OTP_REG_38_RESERVER_MASK                                 0xf0
#define WSA884X_OTP_REG_38_BST_CFG_SEL_MASK                              0x08
#define WSA884X_OTP_REG_38_BOOST_ILIM_TUNE_MASK                          0x07
/* WSA884X_OTP_REG_40 Fields: */
#define WSA884X_OTP_REG_40_SPARE_TYPE2_MASK                              0xc0
#define WSA884X_OTP_REG_40_ISENSE_RESCAL_MASK                            0x3c
#define WSA884X_OTP_REG_40_ATE_BOOST_RDSON_TEST_MASK                     0x02
#define WSA884X_OTP_REG_40_ATE_CLASSD_RDSON_TEST_MASK                    0x01
/* WSA884X_CURRENT_LIMIT Fields: */
#define WSA884X_CURRENT_LIMIT_CURRENT_LIMIT_OVRD_EN_MASK                 0x80
#define WSA884X_CURRENT_LIMIT_CURRENT_LIMIT_MASK                         0x7c
#define WSA884X_CURRENT_LIMIT_CLK_PHASE_MASK                             0x03
/* WSA884X_PWM_CLK_CTL Fields: */
#define WSA884X_PWM_CLK_CTL_VCMO_INT1_IDLE_MODE_OVRT_MASK                0x80
#define WSA884X_PWM_CLK_CTL_REG_MCLK_DIV_RATIO_MASK                      0x40
#define WSA884X_PWM_CLK_CTL_PWM_DEGLITCH_CLK_DELAY_CTRL_MASK             0x30
#define WSA884X_PWM_CLK_CTL_PWM_CLK_FREQ_SEL_MASK                        0x08
#define WSA884X_PWM_CLK_CTL_PWM_CLK_DIV_RATIO_MASK                       0x06
#define WSA884X_PWM_CLK_CTL_PWM_CLK_DIV_BYPASS_MASK                      0x01
/* WSA884X_CKWD_CTL_1 Fields: */
#define WSA884X_CKWD_CTL_1_SPARE_BITS_7_6_MASK                           0xc0
#define WSA884X_CKWD_CTL_1_VPP_SW_CTL_MASK                               0x20
#define WSA884X_CKWD_CTL_1_CKWD_VCOMP_VREF_SEL_MASK                      0x1f
/* WSA884X_VBAT_CAL_CTL Fields: */
#define WSA884X_VBAT_CAL_CTL_RESERVE_MASK                                0x0e
#define WSA884X_VBAT_CAL_CTL_VBAT_CAL_EN_MASK                            0x01
/* WSA884X_REF_CTRL Fields: */
#define WSA884X_REF_CTRL_DC_STARTUP_EN_MASK                              0x80
#define WSA884X_REF_CTRL_DC_STARTUP_HOLD_MASK                            0x40
#define WSA884X_REF_CTRL_TRAN_STARTUP_EN_CORE_MASK                       0x20
#define WSA884X_REF_CTRL_TRAN_STARTUP_EN_PTAT_MASK                       0x10
#define WSA884X_REF_CTRL_BG_EN_MASK                                      0x08
#define WSA884X_REF_CTRL_BG_READY_FORCE_MASK                             0x04
#define WSA884X_REF_CTRL_BG_RDY_SEL_MASK                                 0x03
/* WSA884X_ZX_CTRL1 Fields: */
#define WSA884X_ZX_CTRL1_ZX_DET_EN_MASK                                  0x80
#define WSA884X_ZX_CTRL1_ZX_DET_SW_EN_MASK                               0x40
#define WSA884X_ZX_CTRL1_ZX_DET_STAGE_DEFAULT_MASK                       0x20
#define WSA884X_ZX_CTRL1_ZX_DET_SW_SEL_MASK                              0x18
#define WSA884X_ZX_CTRL1_ZX_BYP_MASK_IGNORE_MASK                         0x04
#define WSA884X_ZX_CTRL1_ZX_BYP_MASK_DEL_MASK                            0x02
#define WSA884X_ZX_CTRL1_BOOTCAP_REFRESH_DIS_MASK                        0x01
#endif /* WSA884X_REG_MASKS_H */
