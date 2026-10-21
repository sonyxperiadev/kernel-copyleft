/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#ifndef WSA884X_REG_SHIFTS_H
#define WSA884X_REG_SHIFTS_H
#include <linux/regmap.h>
#include <linux/device.h>
#include "wsa884x-registers.h"

/*
 * Use in conjunction with wsa884x-reg-masks.c for field values.
 * field_value = (register_value & field_mask) >> field_shift
 */

#define FIELD_SHIFT(register_name, field_name) \
	WSA884X_##register_name##_##field_name##_SHIFT

/* WSA884X_BOP2_PROG Fields: */
#define WSA884X_BOP2_PROG_BOP2_VTH_SHIFT                                  0x04
#define WSA884X_BOP2_PROG_BOP2_HYST_SHIFT                                 0x00
/* WSA884X_VSENSE1 Fields: */
#define WSA884X_VSENSE1_GAIN_VSENSE_FE_SHIFT                              0x05
#define WSA884X_VSENSE1_VSENSE_AMP_IQ_CTL_1_SHIFT                         0x04
#define WSA884X_VSENSE1_IDLE_MODE_CTL_SHIFT                               0x02
#define WSA884X_VSENSE1_VOCM_AMP_CTL_SHIFT                                0x00
/* WSA884X_ISENSE2 Fields: */
#define WSA884X_ISENSE2_ISENSE_GAIN_CTL_SHIFT                             0x05
#define WSA884X_ISENSE2_SUMAMP_IQ_CTL_SHIFT                               0x04
#define WSA884X_ISENSE2_SPARE_BITS_3_0_SHIFT                              0x00
/* WSA884X_ADC_2 Fields: */
#define WSA884X_ADC_2_ATEST_SEL_CAL_REF_SHIFT                             0x07
#define WSA884X_ADC_2_ISNS_LOAD_STORED_SHIFT                              0x06
#define WSA884X_ADC_2_EN_DET_SHIFT                                        0x05
#define WSA884X_ADC_2_EN_ATEST_REF_SHIFT                                  0x04
#define WSA884X_ADC_2_EN_ATEST_INT_SHIFT                                  0x01
#define WSA884X_ADC_2_D_ADC_REG_EN_SHIFT                                  0x00
/* WSA884X_ADC_7 Fields: */
#define WSA884X_ADC_7_CLAMPON_SHIFT                                       0x07
#define WSA884X_ADC_7_CAL_LOOP_TRIM_SHIFT                                 0x04
#define WSA884X_ADC_7_REG_TRIM_EN_SHIFT                                   0x03
#define WSA884X_ADC_7_EN_AZ_REG_SHIFT                                     0x02
#define WSA884X_ADC_7_EN_SAR_REG_SHIFT                                    0x01
#define WSA884X_ADC_7_EN_SW_CURRENT_REG_SHIFT                             0x00
/* WSA884X_TOP_CTRL1 Fields: */
#define WSA884X_TOP_CTRL1_IDLE_PWRSAV_OVERRIDE_SHIFT                      0x07
#define WSA884X_TOP_CTRL1_DAC_LDO_PROG_SHIFT                              0x05
#define WSA884X_TOP_CTRL1_DATA_INV_SHIFT                                  0x04
#define WSA884X_TOP_CTRL1_DATA_RESET_SHIFT                                0x03
#define WSA884X_TOP_CTRL1_CLK_DIV2_SHIFT                                  0x02
#define WSA884X_TOP_CTRL1_CLK_INV_SHIFT                                   0x01
#define WSA884X_TOP_CTRL1_OCP_LOWVBAT_ITH_SEL_EN_SHIFT                    0x00
/* WSA884X_BOP_DEGLITCH_CTL Fields: */
#define WSA884X_BOP_DEGLITCH_CTL_BOP_DEGLITCH_SETTING_SHIFT               0x01
#define WSA884X_BOP_DEGLITCH_CTL_BOP_DEGLITCH_EN_SHIFT                    0x00
/* WSA884X_CDC_SPK_DSM_A2_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A2_0_COEF_A2_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_A2_1 Fields: */
#define WSA884X_CDC_SPK_DSM_A2_1_COEF_A2_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_A3_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A3_0_COEF_A3_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_A3_1 Fields: */
#define WSA884X_CDC_SPK_DSM_A3_1_COEF_A3_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_A4_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A4_0_COEF_A4_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_A5_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A5_0_COEF_A5_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_A6_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A6_0_COEF_A6_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_A7_0 Fields: */
#define WSA884X_CDC_SPK_DSM_A7_0_COEF_A7_SHIFT                            0x00
/* WSA884X_CDC_SPK_DSM_C_0 Fields: */
#define WSA884X_CDC_SPK_DSM_C_0_COEF_C3_SHIFT                             0x04
#define WSA884X_CDC_SPK_DSM_C_0_COEF_C2_SHIFT                             0x00
/* WSA884X_CDC_SPK_DSM_C_2 Fields: */
#define WSA884X_CDC_SPK_DSM_C_2_COEF_C7_SHIFT                             0x04
#define WSA884X_CDC_SPK_DSM_C_2_COEF_C6_SHIFT                             0x00
/* WSA884X_CDC_SPK_DSM_C_3 Fields: */
#define WSA884X_CDC_SPK_DSM_C_3_COEF_C7_SHIFT                             0x00
/* WSA884X_CDC_SPK_DSM_R1 Fields: */
#define WSA884X_CDC_SPK_DSM_R1_SAT_LIMIT_R1_SHIFT                         0x00
/* WSA884X_CDC_SPK_DSM_R2 Fields: */
#define WSA884X_CDC_SPK_DSM_R2_SAT_LIMIT_R2_SHIFT                         0x00
/* WSA884X_CDC_SPK_DSM_R3 Fields: */
#define WSA884X_CDC_SPK_DSM_R3_SAT_LIMIT_R3_SHIFT                         0x00
/* WSA884X_CDC_SPK_DSM_R4 Fields: */
#define WSA884X_CDC_SPK_DSM_R4_SAT_LIMIT_R4_SHIFT                         0x00
/* WSA884X_CDC_SPK_DSM_R5 Fields: */
#define WSA884X_CDC_SPK_DSM_R5_SAT_LIMIT_R5_SHIFT                         0x00
/* WSA884X_CDC_SPK_DSM_R6 Fields: */
#define WSA884X_CDC_SPK_DSM_R6_SAT_LIMIT_R6_SHIFT                         0x00
/* WSA884X_CDC_SPK_DSM_R7 Fields: */
#define WSA884X_CDC_SPK_DSM_R7_SAT_LIMIT_R7_SHIFT                         0x00
/* WSA884X_DRE_CTL_0 Fields: */
#define WSA884X_DRE_CTL_0_PROG_DELAY_SHIFT                                0x04
#define WSA884X_DRE_CTL_0_OFFSET_SHIFT                                    0x00
/* WSA884X_GAIN_RAMPING_MIN Fields: */
#define WSA884X_GAIN_RAMPING_MIN_MIN_GAIN_SHIFT                           0x00
/* WSA884X_CLSH_SOFT_MAX Fields: */
#define WSA884X_CLSH_SOFT_MAX_SOFT_MAX_SHIFT                              0x00
/* WSA884X_CLSH_VTH1 Fields: */
#define WSA884X_CLSH_VTH1_CLSH_VTH1_SHIFT                                 0x00
/* WSA884X_CLSH_VTH10 Fields: */
#define WSA884X_CLSH_VTH10_CLSH_VTH10_SHIFT                               0x00
/* WSA884X_CLSH_VTH11 Fields: */
#define WSA884X_CLSH_VTH11_CLSH_VTH11_SHIFT                               0x00
/* WSA884X_CLSH_VTH12 Fields: */
#define WSA884X_CLSH_VTH12_CLSH_VTH12_SHIFT                               0x00
/* WSA884X_CLSH_VTH13 Fields: */
#define WSA884X_CLSH_VTH13_CLSH_VTH13_SHIFT                               0x00
/* WSA884X_CLSH_VTH14 Fields: */
#define WSA884X_CLSH_VTH14_CLSH_VTH14_SHIFT                               0x00
/* WSA884X_CLSH_VTH15 Fields: */
#define WSA884X_CLSH_VTH15_CLSH_VTH15_SHIFT                               0x00
/* WSA884X_ANA_WO_CTL_0 Fields: */
#define WSA884X_ANA_WO_CTL_0_VPHX_SYS_EN_SHIFT                            0x06
#define WSA884X_ANA_WO_CTL_0_PA_AUX_GAIN_SHIFT                            0x02
#define WSA884X_ANA_WO_CTL_0_PA_MIN_GAIN_BYP_SHIFT                        0x01
#define WSA884X_ANA_WO_CTL_0_DAC_CM_CLAMP_EN_SHIFT                        0x00
/* WSA884X_ANA_WO_CTL_1 Fields: */
#define WSA884X_ANA_WO_CTL_1_BOOST_SHARE_EN_SHIFT                         0x03
#define WSA884X_ANA_WO_CTL_1_EXT_VDDSPK_EN_SHIFT                          0x00
/* WSA884X_DRE_CTL_1 Fields: */
#define WSA884X_DRE_CTL_1_CSR_GAIN_SHIFT                                  0x01
#define WSA884X_DRE_CTL_1_CSR_GAIN_EN_SHIFT                               0x00
/* WSA884X_VBAT_THRM_FLT_CTL Fields: */
#define WSA884X_VBAT_THRM_FLT_CTL_THRM_COEF_SEL_SHIFT                     0x05
#define WSA884X_VBAT_THRM_FLT_CTL_THRM_FLT_EN_SHIFT                       0x04
#define WSA884X_VBAT_THRM_FLT_CTL_VBAT_COEF_SEL_SHIFT                     0x01
#define WSA884X_VBAT_THRM_FLT_CTL_VBAT_FLT_EN_SHIFT                       0x00
/* WSA884X_PDM_WD_CTL Fields: */
#define WSA884X_PDM_WD_CTL_HOLD_OFF_SHIFT                                 0x02
#define WSA884X_PDM_WD_CTL_TIME_OUT_SEL_SHIFT                             0x01
#define WSA884X_PDM_WD_CTL_PDM_WD_EN_SHIFT                                0x00
/* WSA884X_PA_FSM_BYP_CTL Fields: */
#define WSA884X_PA_FSM_BYP_CTL_PA_FSM_BYP_SHIFT                           0x00
/* WSA884X_TADC_VALUE_CTL Fields: */
#define WSA884X_TADC_VALUE_CTL_VBAT_VALUE_RD_EN_SHIFT                     0x01
#define WSA884X_TADC_VALUE_CTL_TEMP_VALUE_RD_EN_SHIFT                     0x00
/* WSA884X_CDC_PATH_MODE Fields: */
#define WSA884X_CDC_PATH_MODE_RXD_MODE_SHIFT                              0x01
#define WSA884X_CDC_PATH_MODE_TXD_MODE_SHIFT                              0x00
/* WSA884X_PA_FSM_BYP0 Fields: */
#define WSA884X_PA_FSM_BYP0_TSADC_EN_SHIFT                                0x07
#define WSA884X_PA_FSM_BYP0_SPKR_PROT_EN_SHIFT                            0x06
#define WSA884X_PA_FSM_BYP0_D_UNMUTE_SHIFT                                0x05
#define WSA884X_PA_FSM_BYP0_PA_EN_SHIFT                                   0x04
#define WSA884X_PA_FSM_BYP0_BOOST_EN_SHIFT                                0x03
#define WSA884X_PA_FSM_BYP0_BG_EN_SHIFT                                   0x02
#define WSA884X_PA_FSM_BYP0_CLK_WD_EN_SHIFT                               0x01
#define WSA884X_PA_FSM_BYP0_DC_CAL_EN_SHIFT                               0x00
/* WSA884X_PA_FSM_BYP1 Fields: */
#define WSA884X_PA_FSM_BYP1_NG_MODE_SHIFT                                 0x06
#define WSA884X_PA_FSM_BYP1_PWRSAV_CTL_SHIFT                              0x05
#define WSA884X_PA_FSM_BYP1_RAMP_DOWN_SHIFT                               0x04
#define WSA884X_PA_FSM_BYP1_RAMP_UP_SHIFT                                 0x03
#define WSA884X_PA_FSM_BYP1_BLEEDER_EN_SHIFT                              0x02
#define WSA884X_PA_FSM_BYP1_PA_MAIN_EN_SHIFT                              0x01
#define WSA884X_PA_FSM_BYP1_PA_AUX_EN_SHIFT                               0x00
/* WSA884X_PA_FSM_EN Fields: */
#define WSA884X_PA_FSM_EN_GLOBAL_PA_EN_SHIFT                              0x00
/* WSA884X_OCP_CTL Fields: */
#define WSA884X_OCP_CTL_OCP_EN_SHIFT                                      0x07
#define WSA884X_OCP_CTL_OCP_CURR_LIMIT_SHIFT                              0x04
#define WSA884X_OCP_CTL_GLITCH_FILTER_SHIFT                               0x02
#define WSA884X_OCP_CTL_OCP_P_HS_DLY_CTL_SHIFT                            0x00
/* WSA884X_ILIM_CTRL1 Fields: */
#define WSA884X_ILIM_CTRL1_EN_AUTO_MAXD_SEL_SHIFT                         0x07
#define WSA884X_ILIM_CTRL1_EN_ILIM_SW_CLH_SHIFT                           0x06
#define WSA884X_ILIM_CTRL1_ILIM_OFFSET_CLH_SHIFT                          0x03
#define WSA884X_ILIM_CTRL1_ILIM_OFFSET_PB_SHIFT                           0x00
/* WSA884X_CLSH_CTL_0 Fields: */
#define WSA884X_CLSH_CTL_0_CSR_GAIN_EN_SHIFT                              0x07
#define WSA884X_CLSH_CTL_0_DLY_CODE_SHIFT                                 0x04
#define WSA884X_CLSH_CTL_0_DLY_RST_SHIFT                                  0x03
#define WSA884X_CLSH_CTL_0_DLY_EN_SHIFT                                   0x02
#define WSA884X_CLSH_CTL_0_INPUT_EN_SHIFT                                 0x01
#define WSA884X_CLSH_CTL_0_CLSH_EN_SHIFT                                  0x00
/* WSA884X_STB_CTRL1 Fields: */
#define WSA884X_STB_CTRL1_SLOPE_COMP_CURRENT_SHIFT                        0x03
#define WSA884X_STB_CTRL1_VOUT_FS_SHIFT                                   0x00
/* WSA884X_OTP_REG_38 Fields: */
#define WSA884X_OTP_REG_38_RESERVER_SHIFT                                 0x04
#define WSA884X_OTP_REG_38_BST_CFG_SEL_SHIFT                              0x03
#define WSA884X_OTP_REG_38_BOOST_ILIM_TUNE_SHIFT                          0x00
/* WSA884X_OTP_REG_40 Fields: */
#define WSA884X_OTP_REG_40_SPARE_TYPE2_SHIFT                              0x06
#define WSA884X_OTP_REG_40_ISENSE_RESCAL_SHIFT                            0x02
#define WSA884X_OTP_REG_40_ATE_BOOST_RDSON_TEST_SHIFT                     0x01
#define WSA884X_OTP_REG_40_ATE_CLASSD_RDSON_TEST_SHIFT                    0x00
/* WSA884X_CURRENT_LIMIT Fields: */
#define WSA884X_CURRENT_LIMIT_CURRENT_LIMIT_OVRD_EN_SHIFT                 0x07
#define WSA884X_CURRENT_LIMIT_CURRENT_LIMIT_SHIFT                         0x02
#define WSA884X_CURRENT_LIMIT_CLK_PHASE_SHIFT                             0x00
/* WSA884X_PWM_CLK_CTL Fields: */
#define WSA884X_PWM_CLK_CTL_VCMO_INT1_IDLE_MODE_OVRT_SHIFT                0x07
#define WSA884X_PWM_CLK_CTL_REG_MCLK_DIV_RATIO_SHIFT                      0x06
#define WSA884X_PWM_CLK_CTL_PWM_DEGLITCH_CLK_DELAY_CTRL_SHIFT             0x04
#define WSA884X_PWM_CLK_CTL_PWM_CLK_FREQ_SEL_SHIFT                        0x03
#define WSA884X_PWM_CLK_CTL_PWM_CLK_DIV_RATIO_SHIFT                       0x01
#define WSA884X_PWM_CLK_CTL_PWM_CLK_DIV_BYPASS_SHIFT                      0x00
/* WSA884X_CKWD_CTL_1 Fields: */
#define WSA884X_CKWD_CTL_1_SPARE_BITS_7_6_SHIFT                           0x06
#define WSA884X_CKWD_CTL_1_VPP_SW_CTL_SHIFT                               0x05
#define WSA884X_CKWD_CTL_1_CKWD_VCOMP_VREF_SEL_SHIFT                      0x00
/* WSA884X_VBAT_CAL_CTL Fields: */
#define WSA884X_VBAT_CAL_CTL_RESERVE_SHIFT                                0x01
#define WSA884X_VBAT_CAL_CTL_VBAT_CAL_EN_SHIFT                            0x00
/* WSA884X_REF_CTRL Fields: */
#define WSA884X_REF_CTRL_DC_STARTUP_EN_SHIFT                              0x07
#define WSA884X_REF_CTRL_DC_STARTUP_HOLD_SHIFT                            0x06
#define WSA884X_REF_CTRL_TRAN_STARTUP_EN_CORE_SHIFT                       0x05
#define WSA884X_REF_CTRL_TRAN_STARTUP_EN_PTAT_SHIFT                       0x04
#define WSA884X_REF_CTRL_BG_EN_SHIFT                                      0x03
#define WSA884X_REF_CTRL_BG_READY_FORCE_SHIFT                             0x02
#define WSA884X_REF_CTRL_BG_RDY_SEL_SHIFT                                 0x00
/* WSA884X_ZX_CTRL1 Fields: */
#define WSA884X_ZX_CTRL1_ZX_DET_EN_SHIFT                                  0x07
#define WSA884X_ZX_CTRL1_ZX_DET_SW_EN_SHIFT                               0x06
#define WSA884X_ZX_CTRL1_ZX_DET_STAGE_DEFAULT_SHIFT                       0x05
#define WSA884X_ZX_CTRL1_ZX_DET_SW_SEL_SHIFT                              0x03
#define WSA884X_ZX_CTRL1_ZX_BYP_MASK_IGNORE_SHIFT                         0x02
#define WSA884X_ZX_CTRL1_ZX_BYP_MASK_DEL_SHIFT                            0x01
#define WSA884X_ZX_CTRL1_BOOTCAP_REFRESH_DIS_SHIFT                        0x00
#endif /* WSA884X_REG_SHIFTS_H */
