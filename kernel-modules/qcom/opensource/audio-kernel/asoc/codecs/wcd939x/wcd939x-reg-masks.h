// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef WCD939X_REG_MASKS_H
#define WCD939X_REG_MASKS_H
#include <linux/regmap.h>
#include <linux/device.h>
#include "wcd939x-registers.h"

/* Use in conjunction with wcd939x-reg-shifts.c for field values. */
/* field_value = (register_value & field_mask) >> field_shift */

#define FIELD_MASK(register_name, field_name) \
WCD939X_##register_name##_##field_name##_MASK

/* WCD939X_ANA_PAGE Fields: */
#define WCD939X_ANA_PAGE_VALUE_MASK                                      0xff

/* WCD939X_BIAS Fields: */
#define WCD939X_BIAS_ANALOG_BIAS_EN_MASK                                 0x80
#define WCD939X_BIAS_PRECHRG_EN_MASK                                     0x40
#define WCD939X_BIAS_PRECHRG_CTL_MODE_MASK                               0x20

/* WCD939X_RX_SUPPLIES Fields: */
#define WCD939X_RX_SUPPLIES_VPOS_EN_MASK                                 0x80
#define WCD939X_RX_SUPPLIES_VNEG_EN_MASK                                 0x40
#define WCD939X_RX_SUPPLIES_VPOS_PWR_LVL_MASK                            0x08
#define WCD939X_RX_SUPPLIES_VNEG_PWR_LVL_MASK                            0x04
#define WCD939X_RX_SUPPLIES_REGULATOR_MODE_MASK                          0x02
#define WCD939X_RX_SUPPLIES_RX_BIAS_ENABLE_MASK                          0x01

/* WCD939X_HPH Fields: */
#define WCD939X_HPH_HPHL_ENABLE_MASK                                     0x80
#define WCD939X_HPH_HPHR_ENABLE_MASK                                     0x40
#define WCD939X_HPH_HPHL_REF_ENABLE_MASK                                 0x20
#define WCD939X_HPH_HPHR_REF_ENABLE_MASK                                 0x10
#define WCD939X_HPH_PWR_LEVEL_MASK                                       0x0c

/* WCD939X_EAR Fields: */
#define WCD939X_EAR_ENABLE_MASK                                          0x80
#define WCD939X_EAR_SHORT_PROT_EN_MASK                                   0x40
#define WCD939X_EAR_OUT_IMPEDANCE_MASK                                   0x20

/* WCD939X_EAR_COMPANDER_CTL Fields: */
#define WCD939X_EAR_COMPANDER_CTL_GAIN_OVRD_REG_MASK                     0x80
#define WCD939X_EAR_COMPANDER_CTL_EAR_GAIN_MASK                          0x7c
#define WCD939X_EAR_COMPANDER_CTL_COMP_DFF_BYP_MASK                      0x02
#define WCD939X_EAR_COMPANDER_CTL_COMP_DFF_CLK_EDGE_MASK                 0x01

/* WCD939X_TX_CH1 Fields: */
#define WCD939X_TX_CH1_ENABLE_MASK                                       0x80
#define WCD939X_TX_CH1_PWR_LEVEL_MASK                                    0x60
#define WCD939X_TX_CH1_GAIN_MASK                                         0x1f

/* WCD939X_TX_CH2 Fields: */
#define WCD939X_TX_CH2_ENABLE_MASK                                       0x80
#define WCD939X_TX_CH2_HPF1_INIT_MASK                                    0x40
#define WCD939X_TX_CH2_HPF2_INIT_MASK                                    0x20
#define WCD939X_TX_CH2_GAIN_MASK                                         0x1f

/* WCD939X_TX_CH3 Fields: */
#define WCD939X_TX_CH3_ENABLE_MASK                                       0x80
#define WCD939X_TX_CH3_PWR_LEVEL_MASK                                    0x60
#define WCD939X_TX_CH3_GAIN_MASK                                         0x1f

/* WCD939X_TX_CH4 Fields: */
#define WCD939X_TX_CH4_ENABLE_MASK                                       0x80
#define WCD939X_TX_CH4_HPF3_INIT_MASK                                    0x40
#define WCD939X_TX_CH4_HPF4_INIT_MASK                                    0x20
#define WCD939X_TX_CH4_GAIN_MASK                                         0x1f

/* WCD939X_MICB1_MICB2_DSP_EN_LOGIC Fields: */
#define WCD939X_MICB1_MICB2_DSP_EN_LOGIC_MICB1_DSP_OVERRIDE_MASK         0x80
#define WCD939X_MICB1_MICB2_DSP_EN_LOGIC_MICB1_DSP_CTRL_MASK             0x60
#define WCD939X_MICB1_MICB2_DSP_EN_LOGIC_MICB2_DSP_OVERRIDE_MASK         0x10
#define WCD939X_MICB1_MICB2_DSP_EN_LOGIC_MICB2_DSP_CTRL_MASK             0x0c

/* WCD939X_MICB3_DSP_EN_LOGIC Fields: */
#define WCD939X_MICB3_DSP_EN_LOGIC_MICB3_DSP_OVERRIDE_MASK               0x80
#define WCD939X_MICB3_DSP_EN_LOGIC_MICB3_DSP_CTRL_MASK                   0x60

/* WCD939X_MBHC_MECH Fields: */
#define WCD939X_MBHC_MECH_L_DET_EN_MASK                                  0x80
#define WCD939X_MBHC_MECH_GND_DET_EN_MASK                                0x40
#define WCD939X_MBHC_MECH_MECH_DETECT_TYPE_MASK                          0x20
#define WCD939X_MBHC_MECH_HPHL_PLUG_TYPE_MASK                            0x10
#define WCD939X_MBHC_MECH_GND_PLUG_TYPE_MASK                             0x08
#define WCD939X_MBHC_MECH_MECH_HS_L_PULLUP_COMP_EN_MASK                  0x04
#define WCD939X_MBHC_MECH_MECH_HS_G_PULLUP_COMP_EN_MASK                  0x02
#define WCD939X_MBHC_MECH_SW_HPH_L_P_100K_TO_GND_MASK                    0x01

/* WCD939X_MBHC_ELECT Fields: */
#define WCD939X_MBHC_ELECT_FSM_EN_MASK                                   0x80
#define WCD939X_MBHC_ELECT_BTNDET_ISRC_CTL_MASK                          0x70
#define WCD939X_MBHC_ELECT_ELECT_DET_TYPE_MASK                           0x08
#define WCD939X_MBHC_ELECT_ELECT_SCHMT_ISRC_CTL_MASK                     0x06
#define WCD939X_MBHC_ELECT_BIAS_EN_MASK                                  0x01

/* WCD939X_MBHC_ZDET Fields: */
#define WCD939X_MBHC_ZDET_ZDET_L_MEAS_EN_MASK                            0x80
#define WCD939X_MBHC_ZDET_ZDET_R_MEAS_EN_MASK                            0x40
#define WCD939X_MBHC_ZDET_ZDET_CHG_EN_MASK                               0x20
#define WCD939X_MBHC_ZDET_ZDET_ILEAK_COMP_EN_MASK                        0x10
#define WCD939X_MBHC_ZDET_ELECT_ISRC_EN_MASK                             0x02

/* WCD939X_MBHC_RESULT_1 Fields: */
#define WCD939X_MBHC_RESULT_1_Z_RESULT_LSB_MASK                          0xff

/* WCD939X_MBHC_RESULT_2 Fields: */
#define WCD939X_MBHC_RESULT_2_Z_RESULT_MSB_MASK                          0xff

/* WCD939X_MBHC_RESULT_3 Fields: */
#define WCD939X_MBHC_RESULT_3_MIC_SCHMT_RESULT_MASK                      0x20
#define WCD939X_MBHC_RESULT_3_IN2P_CLAMP_STATE_MASK                      0x10
#define WCD939X_MBHC_RESULT_3_BTN_RESULT_MASK                            0x07

/* WCD939X_MBHC_BTN0 Fields: */
#define WCD939X_MBHC_BTN0_VTH_MASK                                       0xfc

/* WCD939X_MBHC_BTN1 Fields: */
#define WCD939X_MBHC_BTN1_VTH_MASK                                       0xfc

/* WCD939X_MBHC_BTN2 Fields: */
#define WCD939X_MBHC_BTN2_VTH_MASK                                       0xfc

/* WCD939X_MBHC_BTN3 Fields: */
#define WCD939X_MBHC_BTN3_VTH_MASK                                       0xfc

/* WCD939X_MBHC_BTN4 Fields: */
#define WCD939X_MBHC_BTN4_VTH_MASK                                       0xfc

/* WCD939X_MBHC_BTN5 Fields: */
#define WCD939X_MBHC_BTN5_VTH_MASK                                       0xfc

/* WCD939X_MBHC_BTN6 Fields: */
#define WCD939X_MBHC_BTN6_VTH_MASK                                       0xfc

/* WCD939X_MBHC_BTN7 Fields: */
#define WCD939X_MBHC_BTN7_VTH_MASK                                       0xfc

/* WCD939X_MICB1 Fields: */
#define WCD939X_MICB1_ENABLE_MASK                                        0xc0
#define WCD939X_MICB1_VOUT_CTL_MASK                                      0x3f

/* WCD939X_MICB2 Fields: */
#define WCD939X_MICB2_ENABLE_MASK                                        0xc0
#define WCD939X_MICB2_VOUT_CTL_MASK                                      0x3f

/* WCD939X_MICB2_RAMP Fields: */
#define WCD939X_MICB2_RAMP_RAMP_ENABLE_MASK                              0x80
#define WCD939X_MICB2_RAMP_MB2_IN2P_SHORT_ENABLE_MASK                    0x40
#define WCD939X_MICB2_RAMP_ALLSW_OVRD_ENABLE_MASK                        0x20
#define WCD939X_MICB2_RAMP_SHIFT_CTL_MASK                                0x1c
#define WCD939X_MICB2_RAMP_USB_MGDET_MICB2_RAMP_MASK                     0x03

/* WCD939X_MICB3 Fields: */
#define WCD939X_MICB3_ENABLE_MASK                                        0xc0

/* WCD939X_MICB4 Fields: */
#define WCD939X_MICB4_ENABLE_MASK                                        0xc0


/* WCD939X_CTL Fields: */
#define WCD939X_CTL_BG_FAST_MODE_EN_MASK                                 0x80
#define WCD939X_CTL_TX_SCBIAS_REF_SEL_MASK                               0x40
#define WCD939X_CTL_DC_START_UP_EN_MASK                                  0x20
#define WCD939X_CTL_TRAN_START_UP_EN_MASK                                0x10
#define WCD939X_CTL_OTA_BIAS_CTL_MASK                                    0x08
#define WCD939X_CTL_ATEST_CTL_MASK                                       0x04
#define WCD939X_CTL_EFUSE_EN_MASK                                        0x02

/* WCD939X_VBG_FINE_ADJ Fields: */
#define WCD939X_VBG_FINE_ADJ_VBG_FINE_ADJ_MASK                           0xf0
#define WCD939X_VBG_FINE_ADJ_EN_DTEST_BG_STATUS_MASK                     0x08
#define WCD939X_VBG_FINE_ADJ_PRECHARGE_TIMER_COUNT_MASK                  0x07


/* WCD939X_VDDCX_ADJUST Fields: */
#define WCD939X_VDDCX_ADJUST_RC_ZERO_FREQ_TUNE_MASK                      0x0c
#define WCD939X_VDDCX_ADJUST_VDDCX_ADJUST_MASK                           0x03

/* WCD939X_DISABLE_LDOL Fields: */
#define WCD939X_DISABLE_LDOL_DISABLE_LDOL_MASK                           0x01


/* WCD939X_CTL_CLK Fields: */
#define WCD939X_CTL_CLK_CLK_SEL_MASK                                     0x40
#define WCD939X_CTL_CLK_COMP_CLK_CTL_MASK                                0x30
#define WCD939X_CTL_CLK_COMP_AZ_CTL_MASK                                 0x0c
#define WCD939X_CTL_CLK_TEST_CLK_EN_MASK                                 0x02
#define WCD939X_CTL_CLK_COMP_AVG_BYP_EN_MASK                             0x01

/* WCD939X_CTL_ANA Fields: */
#define WCD939X_CTL_ANA_BIAS_SEL_MASK                                    0x80

/* WCD939X_ZDET_VNEG_CTL Fields: */
#define WCD939X_ZDET_VNEG_CTL_SPARE_BITS_7_6_MASK                        0xc0
#define WCD939X_ZDET_VNEG_CTL_VPOS_EN_MASK                               0x20
#define WCD939X_ZDET_VNEG_CTL_VNEGDAC_LDO_EN_MASK                        0x10
#define WCD939X_ZDET_VNEG_CTL_RXBIAS_EN_MASK                             0x08
#define WCD939X_ZDET_VNEG_CTL_VNEG_MODE_MASK                             0x04
#define WCD939X_ZDET_VNEG_CTL_VNEG_EN_MASK                               0x02
#define WCD939X_ZDET_VNEG_CTL_HPH_DISABLE_MASK                           0x01

/* WCD939X_ZDET_BIAS_CTL Fields: */
#define WCD939X_ZDET_BIAS_CTL_ZDET_ILEAK_EN_OVR_MASK                     0x80
#define WCD939X_ZDET_BIAS_CTL_ZDET_ILEAK_COMP_CTL_MASK                   0x70
#define WCD939X_ZDET_BIAS_CTL_ZDET_LDO_IREF_MASK                         0x0c
#define WCD939X_ZDET_BIAS_CTL_ZDET_COMP_IREF_MASK                        0x03

/* WCD939X_CTL_BCS Fields: */
#define WCD939X_CTL_BCS_FAST_INT_OVRD_EN_MASK                            0x80
#define WCD939X_CTL_BCS_ELECT_REM_FAST_REG_OVRD_MASK                     0x40
#define WCD939X_CTL_BCS_BTN_RELEASE_FAST_REG_OVRD_MASK                   0x20
#define WCD939X_CTL_BCS_BTN_PRESS_FAST_REG_OVRD_MASK                     0x10
#define WCD939X_CTL_BCS_ANC_DET_EN_MASK                                  0x02
#define WCD939X_CTL_BCS_DEBUG_1_MASK                                     0x01

/* WCD939X_MOISTURE_DET_FSM_STATUS Fields: */
#define WCD939X_MOISTURE_DET_FSM_STATUS_ELECT_IN2P_COMP_MASK             0x80
#define WCD939X_MOISTURE_DET_FSM_STATUS_MECH_HS_G_COMP_MASK              0x40
#define WCD939X_MOISTURE_DET_FSM_STATUS_MECH_HS_M_COMP_MASK              0x20
#define WCD939X_MOISTURE_DET_FSM_STATUS_MECH_HS_L_COMP_MASK              0x10
#define WCD939X_MOISTURE_DET_FSM_STATUS_MOISTURE_INTR_MASK               0x08
#define WCD939X_MOISTURE_DET_FSM_STATUS_MOISTURE_GTPOLLING_STATUS_MASK   0x04
#define WCD939X_MOISTURE_DET_FSM_STATUS_MOISTURE_DET_STATUS_MASK         0x02
#define WCD939X_MOISTURE_DET_FSM_STATUS_ZDET_TIMER_MASK                  0x01

/* WCD939X_TEST_CTL Fields: */
#define WCD939X_TEST_CTL_FAST_DBNC_TIMER_MASK                            0x30
#define WCD939X_TEST_CTL_ATEST_MASK                                      0x0f


/* WCD939X_MODE Fields: */
#define WCD939X_MODE_LDOH_EN_MASK                                        0x80
#define WCD939X_MODE_PWRDN_STATE_MASK                                    0x40
#define WCD939X_MODE_SLOWRAMP_EN_MASK                                    0x20
#define WCD939X_MODE_VOUT_ADJUST_MASK                                    0x18
#define WCD939X_MODE_VOUT_COARSE_ADJ_MASK                                0x07

/* WCD939X_LDOH_BIAS Fields: */
#define WCD939X_LDOH_BIAS_IBIAS_REF_MASK                                 0xe0
#define WCD939X_LDOH_BIAS_IBIAS_ERR_AMP_MASK                             0x18
#define WCD939X_LDOH_BIAS_IBIAS_NATIVE_DEVICE_MASK                       0x04
#define WCD939X_LDOH_BIAS_IBIAS_BUFFER_BLEED_MASK                        0x02
#define WCD939X_LDOH_BIAS_INRUSH_CURRENT_FIX_DIS_MASK                    0x01

/* WCD939X_STB_LOADS Fields: */
#define WCD939X_STB_LOADS_STB_LOADS_1_UA_MASK                            0xf0
#define WCD939X_STB_LOADS_STB_LOAD_10_UA_MASK                            0x08
#define WCD939X_STB_LOADS_FORCE_EN_60K_MASK                              0x04
#define WCD939X_STB_LOADS_CLK_GATE_MASK                                  0x02

/* WCD939X_SLOWRAMP Fields: */
#define WCD939X_SLOWRAMP_SLOWRAMP_IBIAS_MASK                             0xc0
#define WCD939X_SLOWRAMP_SLOWRAMP_RESET_TIME_MASK                        0x30


/* WCD939X_TEST_CTL_1 Fields: */
#define WCD939X_TEST_CTL_1_NOISE_FILT_RES_VAL_MASK                       0xe0
#define WCD939X_TEST_CTL_1_EN_VREFGEN_MASK                               0x10
#define WCD939X_TEST_CTL_1_EN_LDO_MASK                                   0x08
#define WCD939X_TEST_CTL_1_LDO_BLEEDER_I_CTRL_MASK                       0x07

/* WCD939X_TEST_CTL_2 Fields: */
#define WCD939X_TEST_CTL_2_IBIAS_VREFGEN_MASK                            0xc0
#define WCD939X_TEST_CTL_2_INRUSH_CURRENT_FIX_DIS_MASK                   0x20
#define WCD939X_TEST_CTL_2_SPAREBIT_MASK                                 0x18
#define WCD939X_TEST_CTL_2_IBIAS_LDO_DRIVER_MASK                         0x07

/* WCD939X_TEST_CTL_3 Fields: */
#define WCD939X_TEST_CTL_3_CFILT_REF_EN_MASK                             0x80
#define WCD939X_TEST_CTL_3_RZ_LDO_VAL_MASK                               0x70
#define WCD939X_TEST_CTL_3_IBIAS_LDO_STG3_MASK                           0x0c
#define WCD939X_TEST_CTL_3_ATEST_CTRL_MASK                               0x03


/* WCD939X_MICB2_TEST_CTL_1 Fields: */
#define WCD939X_MICB2_TEST_CTL_1_NOISE_FILT_RES_VAL_MASK                 0xe0
#define WCD939X_MICB2_TEST_CTL_1_EN_VREFGEN_MASK                         0x10
#define WCD939X_MICB2_TEST_CTL_1_EN_LDO_MASK                             0x08
#define WCD939X_MICB2_TEST_CTL_1_LDO_BLEEDER_I_CTRL_MASK                 0x07

/* WCD939X_MICB2_TEST_CTL_2 Fields: */
#define WCD939X_MICB2_TEST_CTL_2_IBIAS_VREFGEN_MASK                      0xc0
#define WCD939X_MICB2_TEST_CTL_2_INRUSH_CURRENT_FIX_DIS_MASK             0x20
#define WCD939X_MICB2_TEST_CTL_2_SPAREBIT_MASK                           0x18
#define WCD939X_MICB2_TEST_CTL_2_IBIAS_LDO_DRIVER_MASK                   0x07

/* WCD939X_MICB2_TEST_CTL_3 Fields: */
#define WCD939X_MICB2_TEST_CTL_3_CFILT_REF_EN_MASK                       0x80
#define WCD939X_MICB2_TEST_CTL_3_RZ_LDO_VAL_MASK                         0x70
#define WCD939X_MICB2_TEST_CTL_3_IBIAS_LDO_STG3_MASK                     0x0c
#define WCD939X_MICB2_TEST_CTL_3_ATEST_CTRL_MASK                         0x03


/* WCD939X_MICB3_TEST_CTL_1 Fields: */
#define WCD939X_MICB3_TEST_CTL_1_NOISE_FILT_RES_VAL_MASK                 0xe0
#define WCD939X_MICB3_TEST_CTL_1_EN_VREFGEN_MASK                         0x10
#define WCD939X_MICB3_TEST_CTL_1_EN_LDO_MASK                             0x08
#define WCD939X_MICB3_TEST_CTL_1_LDO_BLEEDER_I_CTRL_MASK                 0x07

/* WCD939X_MICB3_TEST_CTL_2 Fields: */
#define WCD939X_MICB3_TEST_CTL_2_IBIAS_VREFGEN_MASK                      0xc0
#define WCD939X_MICB3_TEST_CTL_2_INRUSH_CURRENT_FIX_DIS_MASK             0x20
#define WCD939X_MICB3_TEST_CTL_2_SPAREBIT_MASK                           0x18
#define WCD939X_MICB3_TEST_CTL_2_IBIAS_LDO_DRIVER_MASK                   0x07

/* WCD939X_MICB3_TEST_CTL_3 Fields: */
#define WCD939X_MICB3_TEST_CTL_3_CFILT_REF_EN_MASK                       0x80
#define WCD939X_MICB3_TEST_CTL_3_RZ_LDO_VAL_MASK                         0x70
#define WCD939X_MICB3_TEST_CTL_3_IBIAS_LDO_STG3_MASK                     0x0c
#define WCD939X_MICB3_TEST_CTL_3_ATEST_CTRL_MASK                         0x03


/* WCD939X_MICB4_TEST_CTL_1 Fields: */
#define WCD939X_MICB4_TEST_CTL_1_NOISE_FILT_RES_VAL_MASK                 0xe0
#define WCD939X_MICB4_TEST_CTL_1_EN_VREFGEN_MASK                         0x10
#define WCD939X_MICB4_TEST_CTL_1_EN_LDO_MASK                             0x08
#define WCD939X_MICB4_TEST_CTL_1_LDO_BLEEDER_I_CTRL_MASK                 0x07

/* WCD939X_MICB4_TEST_CTL_2 Fields: */
#define WCD939X_MICB4_TEST_CTL_2_IBIAS_VREFGEN_MASK                      0xc0
#define WCD939X_MICB4_TEST_CTL_2_INRUSH_CURRENT_FIX_DIS_MASK             0x20
#define WCD939X_MICB4_TEST_CTL_2_SPAREBIT_MASK                           0x18
#define WCD939X_MICB4_TEST_CTL_2_IBIAS_LDO_DRIVER_MASK                   0x07

/* WCD939X_MICB4_TEST_CTL_3 Fields: */
#define WCD939X_MICB4_TEST_CTL_3_CFILT_REF_EN_MASK                       0x80
#define WCD939X_MICB4_TEST_CTL_3_RZ_LDO_VAL_MASK                         0x70
#define WCD939X_MICB4_TEST_CTL_3_IBIAS_LDO_STG3_MASK                     0x0c
#define WCD939X_MICB4_TEST_CTL_3_ATEST_CTRL_MASK                         0x03


/* WCD939X_ADC_VCM Fields: */
#define WCD939X_ADC_VCM_FLL_ATEST_EN_MASK                                0x40
#define WCD939X_ADC_VCM_VCM_L2_12P288_MASK                               0x30
#define WCD939X_ADC_VCM_VCM_L2_9P6_MASK                                  0x0c
#define WCD939X_ADC_VCM_VCM_DEFAULT_MASK                                 0x03

/* WCD939X_BIAS_ATEST Fields: */
#define WCD939X_BIAS_ATEST_TX_CURR_EN_MASK                               0x80
#define WCD939X_BIAS_ATEST_SC_BIAS_EN_MASK                               0x40
#define WCD939X_BIAS_ATEST_SC_BIAS_VREF_SEL_MASK                         0x20
#define WCD939X_BIAS_ATEST_ATEST4_EN_MASK                                0x08
#define WCD939X_BIAS_ATEST_ATEST3_EN_MASK                                0x04
#define WCD939X_BIAS_ATEST_ATEST2_EN_MASK                                0x02
#define WCD939X_BIAS_ATEST_ATEST1_EN_MASK                                0x01

/* WCD939X_SPARE1 Fields: */
#define WCD939X_SPARE1_SPARE_BITS_7_0_MASK                               0xff

/* WCD939X_SPARE2 Fields: */
#define WCD939X_SPARE2_SPARE_BITS_7_0_MASK                               0xff

/* WCD939X_TXFE_DIV_CTL Fields: */
#define WCD939X_TXFE_DIV_CTL_FB_SW_DRIVE_MASK                            0x20
#define WCD939X_TXFE_DIV_CTL_EN_CKGEN_INIT_MASK                          0x10
#define WCD939X_TXFE_DIV_CTL_N_PAUSE_MASK                                0x03

/* WCD939X_TXFE_DIV_START Fields: */
#define WCD939X_TXFE_DIV_START_DIV_MASK                                  0xff

/* WCD939X_SPARE3 Fields: */
#define WCD939X_SPARE3_SPARE_BITS_7_0_MASK                               0xff

/* WCD939X_SPARE4 Fields: */
#define WCD939X_SPARE4_SPARE_BITS_7_0_MASK                               0xff


/* WCD939X_TEST_EN Fields: */
#define WCD939X_TEST_EN_TXFE1_EN_MASK                                    0x80
#define WCD939X_TEST_EN_ADC1_EN_MASK                                     0x40
#define WCD939X_TEST_EN_TXFE1_BYPASS_MASK                                0x20
#define WCD939X_TEST_EN_TXFE1_CLK_MODE_MASK                              0x10
#define WCD939X_TEST_EN_TXFE2_EN_MASK                                    0x08
#define WCD939X_TEST_EN_ADC2_EN_MASK                                     0x04
#define WCD939X_TEST_EN_TXFE2_BYPASS_MASK                                0x02
#define WCD939X_TEST_EN_TXFE2_CLK_MODE_MASK                              0x01

/* WCD939X_ADC_IB Fields: */
#define WCD939X_ADC_IB_ADC2_DEM_MODE_MASK                                0xc0
#define WCD939X_ADC_IB_ADC2_DEM_OPERATION_MASK                           0x30
#define WCD939X_ADC_IB_L2_DAC_DLY_MASK                                   0x0c
#define WCD939X_ADC_IB_DEFAULT_DAC_DLY_MASK                              0x03

/* WCD939X_ATEST_REFCTL Fields: */
#define WCD939X_ATEST_REFCTL_ATEST_CTL_MASK                              0xf0
#define WCD939X_ATEST_REFCTL_TXFE_INCM_REF_MASK                          0x0c
#define WCD939X_ATEST_REFCTL_TXFE_HP_GAIN_MODE_MASK                      0x02
#define WCD939X_ATEST_REFCTL_ADCREF_ULPRES_EN_MASK                       0x01

/* WCD939X_TX_1_2_TEST_CTL Fields: */
#define WCD939X_TX_1_2_TEST_CTL_TXFE_HP_GAIN_MASK                        0x80
#define WCD939X_TX_1_2_TEST_CTL_REF_CAP_MASK                             0x40
#define WCD939X_TX_1_2_TEST_CTL_ADC1_DEM_MODE_MASK                       0x30
#define WCD939X_TX_1_2_TEST_CTL_ADC1_DEM_OPERATION_MASK                  0x0c
#define WCD939X_TX_1_2_TEST_CTL_SAR_ERR_DET_EN_MASK                      0x02
#define WCD939X_TX_1_2_TEST_CTL_SAR_EXT_DELAY_EN_MASK                    0x01

/* WCD939X_TEST_BLK_EN1 Fields: */
#define WCD939X_TEST_BLK_EN1_ADC1_INT1_EN_MASK                           0x80
#define WCD939X_TEST_BLK_EN1_ADC1_INT2_EN_MASK                           0x40
#define WCD939X_TEST_BLK_EN1_ADC1_SAR_EN_MASK                            0x20
#define WCD939X_TEST_BLK_EN1_ADC1_CMGEN_EN_MASK                          0x10
#define WCD939X_TEST_BLK_EN1_ADC1_CLKGEN_EN_MASK                         0x08
#define WCD939X_TEST_BLK_EN1_REF_EN_MASK                                 0x04
#define WCD939X_TEST_BLK_EN1_TXFE1_CLKDIV_EN_MASK                        0x02
#define WCD939X_TEST_BLK_EN1_TXFE2_CLKDIV_EN_MASK                        0x01

/* WCD939X_TXFE1_CLKDIV Fields: */
#define WCD939X_TXFE1_CLKDIV_DIV_MASK                                    0xff

/* WCD939X_SAR2_ERR Fields: */
#define WCD939X_SAR2_ERR_SAR_ERR_COUNT_MASK                              0xff

/* WCD939X_SAR1_ERR Fields: */
#define WCD939X_SAR1_ERR_SAR_ERR_COUNT_MASK                              0xff


/* WCD939X_TX_3_4_TEST_EN Fields: */
#define WCD939X_TX_3_4_TEST_EN_TXFE3_EN_MASK                             0x80
#define WCD939X_TX_3_4_TEST_EN_ADC3_EN_MASK                              0x40
#define WCD939X_TX_3_4_TEST_EN_TXFE3_BYPASS_MASK                         0x20
#define WCD939X_TX_3_4_TEST_EN_TXFE3_CLK_MODE_MASK                       0x10
#define WCD939X_TX_3_4_TEST_EN_TXFE4_EN_MASK                             0x08
#define WCD939X_TX_3_4_TEST_EN_ADC4_EN_MASK                              0x04
#define WCD939X_TX_3_4_TEST_EN_TXFE4_BYPASS_MASK                         0x02
#define WCD939X_TX_3_4_TEST_EN_TXFE4_CLK_MODE_MASK                       0x01

/* WCD939X_TX_3_4_ADC_IB Fields: */
#define WCD939X_TX_3_4_ADC_IB_ADC4_DEM_MODE_MASK                         0xc0
#define WCD939X_TX_3_4_ADC_IB_ADC4_DEM_OPERATION_MASK                    0x30
#define WCD939X_TX_3_4_ADC_IB_L2_DAC_DLY_MASK                            0x0c
#define WCD939X_TX_3_4_ADC_IB_DEFAULT_DAC_DLY_MASK                       0x03

/* WCD939X_TX_3_4_ATEST_REFCTL Fields: */
#define WCD939X_TX_3_4_ATEST_REFCTL_ATEST_CTL_MASK                       0xf0
#define WCD939X_TX_3_4_ATEST_REFCTL_TXFE_INCM_REF_MASK                   0x0c
#define WCD939X_TX_3_4_ATEST_REFCTL_TXFE_HP_GAIN_MODE_MASK               0x02
#define WCD939X_TX_3_4_ATEST_REFCTL_ADCREF_ULPRES_EN_MASK                0x01

/* WCD939X_TX_3_4_TEST_CTL Fields: */
#define WCD939X_TX_3_4_TEST_CTL_TXFE_HP_GAIN_MASK                        0x80
#define WCD939X_TX_3_4_TEST_CTL_REF_CAP_MASK                             0x40
#define WCD939X_TX_3_4_TEST_CTL_ADC3_DEM_MODE_MASK                       0x30
#define WCD939X_TX_3_4_TEST_CTL_ADC3_DEM_OPERATION_MASK                  0x0c
#define WCD939X_TX_3_4_TEST_CTL_SAR_ERR_DET_EN_MASK                      0x02
#define WCD939X_TX_3_4_TEST_CTL_SAR_EXT_DELAY_EN_MASK                    0x01

/* WCD939X_TEST_BLK_EN3 Fields: */
#define WCD939X_TEST_BLK_EN3_ADC3_INT1_EN_MASK                           0x80
#define WCD939X_TEST_BLK_EN3_ADC3_INT2_EN_MASK                           0x40
#define WCD939X_TEST_BLK_EN3_ADC3_SAR_EN_MASK                            0x20
#define WCD939X_TEST_BLK_EN3_ADC3_CMGEN_EN_MASK                          0x10
#define WCD939X_TEST_BLK_EN3_ADC3_CLKGEN_EN_MASK                         0x08
#define WCD939X_TEST_BLK_EN3_REF_EN_MASK                                 0x04
#define WCD939X_TEST_BLK_EN3_TXFE3_CLKDIV_EN_MASK                        0x02
#define WCD939X_TEST_BLK_EN3_TXFE4_CLKDIV_EN_MASK                        0x01

/* WCD939X_TXFE3_CLKDIV Fields: */
#define WCD939X_TXFE3_CLKDIV_DIV_MASK                                    0xff

/* WCD939X_SAR4_ERR Fields: */
#define WCD939X_SAR4_ERR_SAR_ERR_COUNT_MASK                              0xff

/* WCD939X_SAR3_ERR Fields: */
#define WCD939X_SAR3_ERR_SAR_ERR_COUNT_MASK                              0xff

/* WCD939X_TEST_BLK_EN2 Fields: */
#define WCD939X_TEST_BLK_EN2_ADC2_INT1_EN_MASK                           0x80
#define WCD939X_TEST_BLK_EN2_ADC2_INT2_EN_MASK                           0x40
#define WCD939X_TEST_BLK_EN2_ADC2_SAR_EN_MASK                            0x20
#define WCD939X_TEST_BLK_EN2_ADC2_CMGEN_EN_MASK                          0x10
#define WCD939X_TEST_BLK_EN2_ADC2_CLKGEN_EN_MASK                         0x08
#define WCD939X_TEST_BLK_EN2_ADC12_VREF_NONL2_MASK                       0x06
#define WCD939X_TEST_BLK_EN2_TXFE2_MBHC_CLKRST_EN_MASK                   0x01

/* WCD939X_TXFE2_CLKDIV Fields: */
#define WCD939X_TXFE2_CLKDIV_DIV_MASK                                    0xff

/* WCD939X_TX_3_4_SPARE1 Fields: */
#define WCD939X_TX_3_4_SPARE1_SPARE_BITS_7_0_MASK                        0xff

/* WCD939X_TEST_BLK_EN4 Fields: */
#define WCD939X_TEST_BLK_EN4_ADC4_INT1_EN_MASK                           0x80
#define WCD939X_TEST_BLK_EN4_ADC4_INT2_EN_MASK                           0x40
#define WCD939X_TEST_BLK_EN4_ADC4_SAR_EN_MASK                            0x20
#define WCD939X_TEST_BLK_EN4_ADC4_CMGEN_EN_MASK                          0x10
#define WCD939X_TEST_BLK_EN4_ADC4_CLKGEN_EN_MASK                         0x08
#define WCD939X_TEST_BLK_EN4_ADC34_VREF_NONL2_MASK                       0x06
#define WCD939X_TEST_BLK_EN4_SPARE_BITS_0_0_MASK                         0x01

/* WCD939X_TXFE4_CLKDIV Fields: */
#define WCD939X_TXFE4_CLKDIV_DIV_MASK                                    0xff

/* WCD939X_TX_3_4_SPARE2 Fields: */
#define WCD939X_TX_3_4_SPARE2_SPARE_BITS_7_0_MASK                        0xff


/* WCD939X_MODE_1 Fields: */
#define WCD939X_MODE_1_BUCK_EN_DELAY_SEL_MASK                            0x60
#define WCD939X_MODE_1_BUCK_EN_RESET_BY_EXT_MASK                         0x10

/* WCD939X_MODE_2 Fields: */
#define WCD939X_MODE_2_VREF_I2C_MASK                                     0xff

/* WCD939X_MODE_3 Fields: */
#define WCD939X_MODE_3_DELTA_IPEAK_2VPK_MASK                             0xf0
#define WCD939X_MODE_3_DELTA_IPEAK_OVERRIDE_MASK                         0x04
#define WCD939X_MODE_3_CTRL_VREF_BY_MASK                                 0x02
#define WCD939X_MODE_3_MANUAL_PWR_OPT_HPH_MASK                           0x01

/* WCD939X_CTRL_VCL_1 Fields: */
#define WCD939X_CTRL_VCL_1_DELTA_V_SEL_MASK                              0xf0
#define WCD939X_CTRL_VCL_1_VDD_BUCK_FILT_2VPK_MASK                       0x0c
#define WCD939X_CTRL_VCL_1_VREF_DELTA_GEN_GAIN_SEL_MASK                  0x03

/* WCD939X_CTRL_VCL_2 Fields: */
#define WCD939X_CTRL_VCL_2_VDD_BUCK_FILT_MASK                            0xc0
#define WCD939X_CTRL_VCL_2_VREF_FILT_1_MASK                              0x30
#define WCD939X_CTRL_VCL_2_VREF_FILT_2_MASK                              0x0e

/* WCD939X_CTRL_CCL_1 Fields: */
#define WCD939X_CTRL_CCL_1_DELTA_IPEAK_MASK                              0xf0
#define WCD939X_CTRL_CCL_1_DELTA_IVALLEY_MASK                            0x0f

/* WCD939X_CTRL_CCL_2 Fields: */
#define WCD939X_CTRL_CCL_2_CHOOSE_I_LIM_MASK                             0xfc
#define WCD939X_CTRL_CCL_2_BUCK_BYPASS_OVERRIDE_MASK                     0x02
#define WCD939X_CTRL_CCL_2_BUCK_BYPASS_EN_MASK                           0x01

/* WCD939X_CTRL_CCL_3 Fields: */
#define WCD939X_CTRL_CCL_3_MIN_PON_MASK                                  0xc0
#define WCD939X_CTRL_CCL_3_MIN_NON_MASK                                  0x30

/* WCD939X_CTRL_CCL_4 Fields: */
#define WCD939X_CTRL_CCL_4_P_BLNK_INV1_LOAD_MASK                         0x80
#define WCD939X_CTRL_CCL_4_P_BLNK_INV2_LOAD_MASK                         0x40
#define WCD939X_CTRL_CCL_4_N_BLNK_INV1_LOAD_MASK                         0x20
#define WCD939X_CTRL_CCL_4_N_BLNK_INV2_LOAD_MASK                         0x10
#define WCD939X_CTRL_CCL_4_RST_PW_INV_LOAD_MASK                          0x02
#define WCD939X_CTRL_CCL_4_INZ_RST_SW_CTRL_MASK                          0x01

/* WCD939X_CTRL_CCL_5 Fields: */
#define WCD939X_CTRL_CCL_5_IPK_FRC_RST_MASK                              0xe0

/* WCD939X_BUCK_TMUX_A_D Fields: */
#define WCD939X_BUCK_TMUX_A_D_ATEST_SEL_MASK                             0x80
#define WCD939X_BUCK_TMUX_A_D_DTEST_MUX_EN_MASK                          0x08
#define WCD939X_BUCK_TMUX_A_D_DTEST_BRK_4_BRK_3_BRK_2_BRK_1_MASK         0x07

/* WCD939X_BUCK_SW_DRV_CNTL Fields: */
#define WCD939X_BUCK_SW_DRV_CNTL_PSW_DRV_CNTL_MASK                       0xf0
#define WCD939X_BUCK_SW_DRV_CNTL_NSW_DRV_CNTL_MASK                       0x0f

/* WCD939X_SPARE Fields: */
#define WCD939X_SPARE_CHOOSE_I_LIM_2VPK_MASK                             0xfc


/* WCD939X_EN Fields: */
#define WCD939X_EN_FLYBACK_EN_DELAY_SEL_MASK                             0x60
#define WCD939X_EN_FLYBACK_EN_RESET_BY_EXT_MASK                          0x10
#define WCD939X_EN_EN_PWSV_MASK                                          0x08
#define WCD939X_EN_EN_CUR_DET_MASK                                       0x04
#define WCD939X_EN_EN_BLEEDER_MASK                                       0x02
#define WCD939X_EN_VREF_PWR_DAC_SEL_OVERRIDE_MASK                        0x01

/* WCD939X_VNEG_CTRL_1 Fields: */
#define WCD939X_VNEG_CTRL_1_VREF_DELTA_GEN_LP_MASK                       0xe0
#define WCD939X_VNEG_CTRL_1_VREF_DELTA_GEN_UHQA_MASK                     0x1c
#define WCD939X_VNEG_CTRL_1_DRV_PSW_LC_MASK                              0x02
#define WCD939X_VNEG_CTRL_1_DRV_PSW_HC_MASK                              0x01

/* WCD939X_VNEG_CTRL_2 Fields: */
#define WCD939X_VNEG_CTRL_2_MIN_PON_MASK                                 0xc0
#define WCD939X_VNEG_CTRL_2_MIN_NON_MASK                                 0x20
#define WCD939X_VNEG_CTRL_2_RST_PW_MASK                                  0x10
#define WCD939X_VNEG_CTRL_2_P_BLNK_MASK                                  0x0c
#define WCD939X_VNEG_CTRL_2_N_BLNK_MASK                                  0x03

/* WCD939X_VNEG_CTRL_3 Fields: */
#define WCD939X_VNEG_CTRL_3_EN_IVLY_FRC_RST_MASK                         0x10
#define WCD939X_VNEG_CTRL_3_IVLY_FRC_RST_MASK                            0x0c
#define WCD939X_VNEG_CTRL_3_INZ_RDY_CTL_MASK                             0x02
#define WCD939X_VNEG_CTRL_3_INIT_MINPON_CTL_MASK                         0x01

/* WCD939X_VNEG_CTRL_4 Fields: */
#define WCD939X_VNEG_CTRL_4_ILIM_SEL_MASK                                0xf0
#define WCD939X_VNEG_CTRL_4_PW_BUF_POS_MASK                              0x0c
#define WCD939X_VNEG_CTRL_4_PW_BUF_NEG_MASK                              0x03

/* WCD939X_VNEG_CTRL_5 Fields: */
#define WCD939X_VNEG_CTRL_5_IPK_DELTA_VNEG_LP_MASK                       0xf0
#define WCD939X_VNEG_CTRL_5_IPK_DELTA_VNEG_UHQA_MASK                     0x0f

/* WCD939X_VNEG_CTRL_6 Fields: */
#define WCD939X_VNEG_CTRL_6_VREF_THIGH_POS_MASK                          0xf0
#define WCD939X_VNEG_CTRL_6_VREF_TLOW_POS_MASK                           0x0f

/* WCD939X_VNEG_CTRL_7 Fields: */
#define WCD939X_VNEG_CTRL_7_VREF_THIGH_NEG_MASK                          0xf0
#define WCD939X_VNEG_CTRL_7_VREF_TLOW_NEG_MASK                           0x0f

/* WCD939X_VNEG_CTRL_8 Fields: */
#define WCD939X_VNEG_CTRL_8_SW_POS_EN_DLY_MASK                           0xc0
#define WCD939X_VNEG_CTRL_8_SW_NEG_EN_DLY_MASK                           0x30
#define WCD939X_VNEG_CTRL_8_VNEG_EN_DLY_MASK                             0x0e
#define WCD939X_VNEG_CTRL_8_EN_IVLYCMP_STATIC_MASK                       0x01

/* WCD939X_VNEG_CTRL_9 Fields: */
#define WCD939X_VNEG_CTRL_9_CUR_DET_TH_MASK                              0xc0
#define WCD939X_VNEG_CTRL_9_MAXPON_SEL_MASK                              0x38
#define WCD939X_VNEG_CTRL_9_EN_MAXPON_FRC_MASK                           0x04
#define WCD939X_VNEG_CTRL_9_VREF_PWR_DAC_SEL_MASK                        0x02

/* WCD939X_VNEGDAC_CTRL_1 Fields: */
#define WCD939X_VNEGDAC_CTRL_1_VREF_DAC_DELTA_GEN_LP_MASK                0xe0
#define WCD939X_VNEGDAC_CTRL_1_VREF_DAC_DELTA_GEN_UHQA_MASK              0x1c
#define WCD939X_VNEGDAC_CTRL_1_N_BLNK_DAC_MASK                           0x03

/* WCD939X_VNEGDAC_CTRL_2 Fields: */
#define WCD939X_VNEGDAC_CTRL_2_VREF_DAC_SEL_MASK                         0xe0
#define WCD939X_VNEGDAC_CTRL_2_VNEGDAC_1P8REF_EN_DLY_MASK                0x18
#define WCD939X_VNEGDAC_CTRL_2_VREF_BLEEDER_MASK                         0x06
#define WCD939X_VNEGDAC_CTRL_2_N_ICHRG_BLNK_DAC_MASK                     0x01

/* WCD939X_VNEGDAC_CTRL_3 Fields: */
#define WCD939X_VNEGDAC_CTRL_3_IPK_DELTA_VNEGDAC_LP_MASK                 0xf0
#define WCD939X_VNEGDAC_CTRL_3_IPK_DELTA_VNEGDAC_UHQA_MASK               0x0f

/* WCD939X_CTRL_1 Fields: */
#define WCD939X_CTRL_1_ICHRG_VREF_MASK                                   0xc0
#define WCD939X_CTRL_1_EN_INZCMP_CTL_1_MASK                              0x20
#define WCD939X_CTRL_1_EN_INZCMP_CTL_2_MASK                              0x10
#define WCD939X_CTRL_1_DELTAV_STEP_CTL_MASK                              0x08
#define WCD939X_CTRL_1_EN_MAXNON_FRC_MASK                                0x04
#define WCD939X_CTRL_1_MAXNON_SEL_MASK                                   0x03

/* WCD939X_FLYBACK_TEST_CTL Fields: */
#define WCD939X_FLYBACK_TEST_CTL_DTEST_MUX_SEL_MASK                      0x80
#define WCD939X_FLYBACK_TEST_CTL_ILIM_SEL_2VPK_MASK                      0x0f


/* WCD939X_AUX_SW_CTL Fields: */
#define WCD939X_AUX_SW_CTL_AUXL_SW_EN_MASK                               0x80
#define WCD939X_AUX_SW_CTL_AUXR_SW_EN_MASK                               0x40
#define WCD939X_AUX_SW_CTL_AUXL2R_SW_EN_MASK                             0x20

/* WCD939X_PA_AUX_IN_CONN Fields: */
#define WCD939X_PA_AUX_IN_CONN_HPHL_AUX_IN_MASK                          0x80
#define WCD939X_PA_AUX_IN_CONN_HPHR_AUX_IN_MASK                          0x40
#define WCD939X_PA_AUX_IN_CONN_EAR_AUX_IN_MASK                           0x20
#define WCD939X_PA_AUX_IN_CONN_SPARE_BITS0_MASK                          0x10
#define WCD939X_PA_AUX_IN_CONN_SPARE_BITS1_MASK                          0x0e
#define WCD939X_PA_AUX_IN_CONN_RX_CLK_PHASE_INV_MASK                     0x01

/* WCD939X_TIMER_DIV Fields: */
#define WCD939X_TIMER_DIV_RX_CLK_DIVIDER_OVWT_MASK                       0x80
#define WCD939X_TIMER_DIV_RX_CLK_DIVIDER_MASK                            0x7f

/* WCD939X_OCP_CTL Fields: */
#define WCD939X_OCP_CTL_SPARE_BITS_MASK                                  0xf0
#define WCD939X_OCP_CTL_N_CONNECTION_ATTEMPTS_MASK                       0x0f

/* WCD939X_OCP_COUNT Fields: */
#define WCD939X_OCP_COUNT_RUN_N_CYCLES_MASK                              0xf0
#define WCD939X_OCP_COUNT_WAIT_N_CYCLES_MASK                             0x0f

/* WCD939X_BIAS_EAR_DAC Fields: */
#define WCD939X_BIAS_EAR_DAC_EAR_DAC_5_UA_MASK                           0xf0
#define WCD939X_BIAS_EAR_DAC_ATEST_RX_BIAS_MASK                          0x0f

/* WCD939X_BIAS_EAR_AMP Fields: */
#define WCD939X_BIAS_EAR_AMP_EAR_AMP_10_UA_MASK                          0xf0
#define WCD939X_BIAS_EAR_AMP_EAR_AMP_5_UA_MASK                           0x0f

/* WCD939X_BIAS_HPH_LDO Fields: */
#define WCD939X_BIAS_HPH_LDO_HPH_NVLDO2_5_UA_MASK                        0xf0
#define WCD939X_BIAS_HPH_LDO_HPH_NVLDO1_4P5_UA_MASK                      0x0f

/* WCD939X_BIAS_HPH_PA Fields: */
#define WCD939X_BIAS_HPH_PA_HPH_CONSTOP_5_UA_MASK                        0xf0
#define WCD939X_BIAS_HPH_PA_HPH_AMP_5_UA_MASK                            0x0f

/* WCD939X_BIAS_HPH_RDACBUFF_CNP2 Fields: */
#define WCD939X_BIAS_HPH_RDACBUFF_CNP2_RDAC_BUF_3_UA_MASK                0xf0
#define WCD939X_BIAS_HPH_RDACBUFF_CNP2_HPH_CNP_10_UA_MASK                0x0f

/* WCD939X_BIAS_HPH_RDAC_LDO Fields: */
#define WCD939X_BIAS_HPH_RDAC_LDO_RDAC_LDO_1P65_4_UA_MASK                0xf0
#define WCD939X_BIAS_HPH_RDAC_LDO_RDAC_LDO_N1P65_4_UA_MASK               0x0f

/* WCD939X_BIAS_HPH_CNP1 Fields: */
#define WCD939X_BIAS_HPH_CNP1_HPH_CNP_4_UA_MASK                          0xf0
#define WCD939X_BIAS_HPH_CNP1_HPH_CNP_3_UA_MASK                          0x0f

/* WCD939X_BIAS_HPH_LOWPOWER Fields: */
#define WCD939X_BIAS_HPH_LOWPOWER_HPH_AMP_LP_1P5_UA_MASK                 0xf0
#define WCD939X_BIAS_HPH_LOWPOWER_RDAC_BUF_LP_0P25_UA_MASK               0x0f

/* WCD939X_BIAS_AUX_DAC Fields: */
#define WCD939X_BIAS_AUX_DAC_SPARE_BITS0_MASK                            0xf0
#define WCD939X_BIAS_AUX_DAC_SPARE_BITS1_MASK                            0x0f

/* WCD939X_BIAS_AUX_AMP Fields: */
#define WCD939X_BIAS_AUX_AMP_SPARE_BITS0_MASK                            0xf0
#define WCD939X_BIAS_AUX_AMP_SPARE_BITS1_MASK                            0x0f

/* WCD939X_BIAS_VNEGDAC_BLEEDER Fields: */
#define WCD939X_BIAS_VNEGDAC_BLEEDER_BLEEDER_CTRL_MASK                   0xf0

/* WCD939X_BIAS_MISC Fields: */
#define WCD939X_BIAS_MISC_SPARE_BITS_MASK                                0xff

/* WCD939X_BIAS_BUCK_RST Fields: */
#define WCD939X_BIAS_BUCK_RST_BUCK_RST_2_UA_MASK                         0x0f

/* WCD939X_BIAS_BUCK_VREF_ERRAMP Fields: */
#define WCD939X_BIAS_BUCK_VREF_ERRAMP_BUCK_VREF_1_UA_MASK                0xf0
#define WCD939X_BIAS_BUCK_VREF_ERRAMP_BUCK_ERRAMP_1_UA_MASK              0x0f

/* WCD939X_BIAS_FLYB_ERRAMP Fields: */
#define WCD939X_BIAS_FLYB_ERRAMP_FLYB_ERRAMP_1_UA_MASK                   0xf0

/* WCD939X_BIAS_FLYB_BUFF Fields: */
#define WCD939X_BIAS_FLYB_BUFF_FLYB_VNEG_5_UA_MASK                       0xf0
#define WCD939X_BIAS_FLYB_BUFF_FLYB_VPOS_5_UA_MASK                       0x0f

/* WCD939X_BIAS_FLYB_MID_RST Fields: */
#define WCD939X_BIAS_FLYB_MID_RST_FLYB_MID_1_UA_MASK                     0xf0
#define WCD939X_BIAS_FLYB_MID_RST_FLYB_RST_1_UA_MASK                     0x0f


/* WCD939X_L_STATUS Fields: */
#define WCD939X_L_STATUS_CMPDR_GAIN_MASK                                 0xf8
#define WCD939X_L_STATUS_OCP_COMP_DETECT_MASK                            0x04
#define WCD939X_L_STATUS_OCP_LIMIT_MASK                                  0x02
#define WCD939X_L_STATUS_PA_READY_MASK                                   0x01

/* WCD939X_R_STATUS Fields: */
#define WCD939X_R_STATUS_CMPDR_GAIN_MASK                                 0xf8
#define WCD939X_R_STATUS_OCP_COMP_DETECT_MASK                            0x04
#define WCD939X_R_STATUS_OCP_LIMIT_MASK                                  0x02
#define WCD939X_R_STATUS_PA_READY_MASK                                   0x01

/* WCD939X_CNP_EN Fields: */
#define WCD939X_CNP_EN_FSM_CLK_EN_MASK                                   0x80
#define WCD939X_CNP_EN_FSM_RESET_MASK                                    0x40
#define WCD939X_CNP_EN_CNP_IREF_SEL_MASK                                 0x20
#define WCD939X_CNP_EN_FSM_OVERRIDE_EN_MASK                              0x08
#define WCD939X_CNP_EN_WG_LR_SEL_MASK                                    0x04
#define WCD939X_CNP_EN_DBG_CURR_DIRECTION_R_MASK                         0x02
#define WCD939X_CNP_EN_DBG_VREF_EN_MASK                                  0x01

/* WCD939X_CNP_WG_CTL Fields: */
#define WCD939X_CNP_WG_CTL_GM3_BOOST_EN_MASK                             0x80
#define WCD939X_CNP_WG_CTL_NO_PD_SEQU_MASK                               0x40
#define WCD939X_CNP_WG_CTL_VREF_TIMER_MASK                               0x38
#define WCD939X_CNP_WG_CTL_CURR_LDIV_CTL_MASK                            0x07

/* WCD939X_CNP_WG_TIME Fields: */
#define WCD939X_CNP_WG_TIME_WG_FINE_TIMER_MASK                           0xff

/* WCD939X_HPH_OCP_CTL Fields: */
#define WCD939X_HPH_OCP_CTL_OCP_CURR_LIMIT_MASK                          0xe0
#define WCD939X_HPH_OCP_CTL_OCP_FSM_EN_MASK                              0x10
#define WCD939X_HPH_OCP_CTL_SPARE_BITS_MASK                              0x08
#define WCD939X_HPH_OCP_CTL_SCD_OP_EN_MASK                               0x02

/* WCD939X_AUTO_CHOP Fields: */
#define WCD939X_AUTO_CHOP_GM3_CASCODE_CTL_2VPK_MASK                      0xc0
#define WCD939X_AUTO_CHOP_AUTO_CHOPPER_MODE_MASK                         0x20
#define WCD939X_AUTO_CHOP_GAIN_THRESHOLD_MASK                            0x1f

/* WCD939X_CHOP_CTL Fields: */
#define WCD939X_CHOP_CTL_CHOPPER_EN_MASK                                 0x80
#define WCD939X_CHOP_CTL_CLK_INV_MASK                                    0x40
#define WCD939X_CHOP_CTL_SPARE_BITS_MASK                                 0x38
#define WCD939X_CHOP_CTL_DIV2_DIV_BY_2_MASK                              0x04
#define WCD939X_CHOP_CTL_DIV2_DIV_BY_2_4_6_8_MASK                        0x03

/* WCD939X_PA_CTL1 Fields: */
#define WCD939X_PA_CTL1_GM3_IBIAS_CTL_MASK                               0xf0
#define WCD939X_PA_CTL1_GM3_IB_SCALE_MASK                                0x0e
#define WCD939X_PA_CTL1_SPARE_BITS_MASK                                  0x01

/* WCD939X_PA_CTL2 Fields: */
#define WCD939X_PA_CTL2_SPARE_BITS0_MASK                                 0x80
#define WCD939X_PA_CTL2_HPHPA_GND_R_MASK                                 0x40
#define WCD939X_PA_CTL2_SPARE_BITS1_MASK                                 0x20
#define WCD939X_PA_CTL2_HPHPA_GND_L_MASK                                 0x10
#define WCD939X_PA_CTL2_SPARE_BITS2_MASK                                 0x0c
#define WCD939X_PA_CTL2_GM3_CASCODE_CTL_NORMAL_MASK                      0x03

/* WCD939X_L_EN Fields: */
#define WCD939X_L_EN_CONST_SEL_L_MASK                                    0xc0
#define WCD939X_L_EN_GAIN_SOURCE_SEL_MASK                                0x20
#define WCD939X_L_EN_SPARE_BITS_MASK                                     0x1f

/* WCD939X_L_TEST Fields: */
#define WCD939X_L_TEST_PDN_EN_MASK                                       0x80
#define WCD939X_L_TEST_PDN_AMP2_EN_MASK                                  0x40
#define WCD939X_L_TEST_PDN_AMP_EN_MASK                                   0x20
#define WCD939X_L_TEST_PA_CNP_SW_CONN_MASK                               0x10
#define WCD939X_L_TEST_PA_CNP_SW_OFF_MASK                                0x08
#define WCD939X_L_TEST_PA_CNP_SW_ON_MASK                                 0x04
#define WCD939X_L_TEST_SPARE_BITS_MASK                                   0x02
#define WCD939X_L_TEST_OCP_DET_EN_MASK                                   0x01

/* WCD939X_L_ATEST Fields: */
#define WCD939X_L_ATEST_DACL_REF_ATEST1_CONN_MASK                        0x80
#define WCD939X_L_ATEST_LDO1_L_ATEST2_CONN_MASK                          0x40
#define WCD939X_L_ATEST_LDO_L_ATEST2_CAL_MASK                            0x20
#define WCD939X_L_ATEST_LDO2_L_ATEST2_CONN_MASK                          0x10
#define WCD939X_L_ATEST_HPHPA_GND_OVR_MASK                               0x08
#define WCD939X_L_ATEST_SPARE_BITS_MASK                                  0x04
#define WCD939X_L_ATEST_CNP_EXD2_MASK                                    0x02
#define WCD939X_L_ATEST_CNP_EXD1_MASK                                    0x01

/* WCD939X_R_EN Fields: */
#define WCD939X_R_EN_CONST_SEL_R_MASK                                    0xc0
#define WCD939X_R_EN_GAIN_SOURCE_SEL_MASK                                0x20
#define WCD939X_R_EN_SPARE_BITS_MASK                                     0x1f

/* WCD939X_R_TEST Fields: */
#define WCD939X_R_TEST_PDN_EN_MASK                                       0x80
#define WCD939X_R_TEST_PDN_AMP2_EN_MASK                                  0x40
#define WCD939X_R_TEST_PDN_AMP_EN_MASK                                   0x20
#define WCD939X_R_TEST_PA_CNP_SW_CONN_MASK                               0x10
#define WCD939X_R_TEST_PA_CNP_SW_OFF_MASK                                0x08
#define WCD939X_R_TEST_PA_CNP_SW_ON_MASK                                 0x04
#define WCD939X_R_TEST_SPARE_BITS_MASK                                   0x02
#define WCD939X_R_TEST_OCP_DET_EN_MASK                                   0x01

/* WCD939X_R_ATEST Fields: */
#define WCD939X_R_ATEST_DACR_REF_ATEST1_CONN_MASK                        0x80
#define WCD939X_R_ATEST_LDO1_R_ATEST2_CONN_MASK                          0x40
#define WCD939X_R_ATEST_LDO_R_ATEST2_CAL_MASK                            0x20
#define WCD939X_R_ATEST_LDO2_R_ATEST2_CONN_MASK                          0x10
#define WCD939X_R_ATEST_LDO_1P65V_ATEST1_CONN_MASK                       0x08
#define WCD939X_R_ATEST_SPARE_BITS0_MASK                                 0x04
#define WCD939X_R_ATEST_HPH_GND_OVR_MASK                                 0x02
#define WCD939X_R_ATEST_SPARE_BITS1_MASK                                 0x01

/* WCD939X_RDAC_CLK_CTL1 Fields: */
#define WCD939X_RDAC_CLK_CTL1_OPAMP_CHOP_CLK_EN_MASK                     0x80
#define WCD939X_RDAC_CLK_CTL1_OPAMP_CHOP_CLK_DIV_CTRL_MASK               0x70
#define WCD939X_RDAC_CLK_CTL1_SPARE_BITS_MASK                            0x0f

/* WCD939X_RDAC_CLK_CTL2 Fields: */
#define WCD939X_RDAC_CLK_CTL2_SPARE_BITS_MASK                            0xf0
#define WCD939X_RDAC_CLK_CTL2_PREREF_SC_CLK_EN_MASK                      0x08
#define WCD939X_RDAC_CLK_CTL2_PREREF_SC_CLK_DIVIDER_CTRL_MASK            0x07

/* WCD939X_RDAC_LDO_CTL Fields: */
#define WCD939X_RDAC_LDO_CTL_LDO_1P65_BYPASS_MASK                        0x80
#define WCD939X_RDAC_LDO_CTL_LDO_1P65_OUTCTL_MASK                        0x70
#define WCD939X_RDAC_LDO_CTL_N1P65V_LDO_BYPASS_MASK                      0x08
#define WCD939X_RDAC_LDO_CTL_N1P65_LDO_OUTCTL_MASK                       0x07

/* WCD939X_RDAC_CHOP_CLK_LP_CTL Fields: */
#define WCD939X_RDAC_CHOP_CLK_LP_CTL_OPAMP_CHOP_CLK_EN_LP_MASK           0x80
#define WCD939X_RDAC_CHOP_CLK_LP_CTL_SPARE_BITS_MASK                     0x7f

/* WCD939X_REFBUFF_UHQA_CTL Fields: */
#define WCD939X_REFBUFF_UHQA_CTL_SPARE_BITS_MASK                         0xc0
#define WCD939X_REFBUFF_UHQA_CTL_HPH_VNEGREG2_COMP_CTL_OV_MASK           0x20
#define WCD939X_REFBUFF_UHQA_CTL_REFBUFN_RBIAS_ADJUST_MASK               0x10
#define WCD939X_REFBUFF_UHQA_CTL_REFBUFP_IOUT_CTL_MASK                   0x0c
#define WCD939X_REFBUFF_UHQA_CTL_REFBUFN_IOUT_CTL_MASK                   0x03

/* WCD939X_REFBUFF_LP_CTL Fields: */
#define WCD939X_REFBUFF_LP_CTL_HPH_VNEGREG2_CURR_COMP_MASK               0xc0
#define WCD939X_REFBUFF_LP_CTL_SPARE_BITS_MASK                           0x30
#define WCD939X_REFBUFF_LP_CTL_EN_PREREF_FILT_STARTUP_CLKDIV_MASK        0x08
#define WCD939X_REFBUFF_LP_CTL_PREREF_FILT_STARTUP_CLKDIV_CTL_MASK       0x06
#define WCD939X_REFBUFF_LP_CTL_PREREF_FILT_BYPASS_MASK                   0x01

/* WCD939X_L_DAC_CTL Fields: */
#define WCD939X_L_DAC_CTL_SPARE_BITS_MASK                                0x80
#define WCD939X_L_DAC_CTL_DAC_REF_EN_MASK                                0x40
#define WCD939X_L_DAC_CTL_DAC_SAMPLE_EDGE_SELECT_MASK                    0x20
#define WCD939X_L_DAC_CTL_DATA_RESET_MASK                                0x10
#define WCD939X_L_DAC_CTL_INV_DATA_MASK                                  0x08
#define WCD939X_L_DAC_CTL_DAC_L_EN_OV_MASK                               0x04
#define WCD939X_L_DAC_CTL_DAC_LDO_UHQA_OV_MASK                           0x02
#define WCD939X_L_DAC_CTL_DAC_LDO_POWERMODE_MASK                         0x01

/* WCD939X_R_DAC_CTL Fields: */
#define WCD939X_R_DAC_CTL_SPARE_BITS_MASK                                0x80
#define WCD939X_R_DAC_CTL_DAC_REF_EN_MASK                                0x40
#define WCD939X_R_DAC_CTL_DAC_SAMPLE_EDGE_SELECT_MASK                    0x20
#define WCD939X_R_DAC_CTL_DATA_RESET_MASK                                0x10
#define WCD939X_R_DAC_CTL_INV_DATA_MASK                                  0x08
#define WCD939X_R_DAC_CTL_DAC_R_EN_OV_MASK                               0x04
#define WCD939X_R_DAC_CTL_DAC_PREREF_UHQA_OV_MASK                        0x02
#define WCD939X_R_DAC_CTL_DAC_PREREF_POWERMODE_MASK                      0x01


/* WCD939X_HPHLR_SURGE_COMP_SEL Fields: */
#define WCD939X_HPHLR_SURGE_COMP_SEL_COMP_REF_SEL_HPHL_PSURGE_MASK       0xc0
#define WCD939X_HPHLR_SURGE_COMP_SEL_COMP_REF_SEL_HPHL_NSURGE_MASK       0x30
#define WCD939X_HPHLR_SURGE_COMP_SEL_COMP_REF_SEL_HPHR_PSURGE_MASK       0x0c
#define WCD939X_HPHLR_SURGE_COMP_SEL_COMP_REF_SEL_HPHR_NSURGE_MASK       0x03

/* WCD939X_HPHLR_SURGE_EN Fields: */
#define WCD939X_HPHLR_SURGE_EN_EN_SURGE_PROTECTION_HPHL_MASK             0x80
#define WCD939X_HPHLR_SURGE_EN_EN_SURGE_PROTECTION_HPHR_MASK             0x40
#define WCD939X_HPHLR_SURGE_EN_SEL_SURGE_COMP_IQ_MASK                    0x30
#define WCD939X_HPHLR_SURGE_EN_SURGE_VOLT_MODE_SHUTOFF_EN_MASK           0x08
#define WCD939X_HPHLR_SURGE_EN_LATCH_INTR_OP_STG_HIZ_EN_MASK             0x04
#define WCD939X_HPHLR_SURGE_EN_SURGE_LATCH_REG_RESET_MASK                0x02
#define WCD939X_HPHLR_SURGE_EN_SWTICH_VN_VNDAC_NSURGE_EN_MASK            0x01

/* WCD939X_HPHLR_SURGE_MISC1 Fields: */
#define WCD939X_HPHLR_SURGE_MISC1_EN_VNEG_PULLDN_MASK                    0x80
#define WCD939X_HPHLR_SURGE_MISC1_EN_OFFSET_36MV_NSURGE_RESLADDER_MASK   0x40
#define WCD939X_HPHLR_SURGE_MISC1_EN_NMOS_LAMP_MASK                      0x20
#define WCD939X_HPHLR_SURGE_MISC1_EN_NCLAMP_REG_HPHL_MASK                0x10
#define WCD939X_HPHLR_SURGE_MISC1_EN_NCLAMP_REG_HPHR_MASK                0x08
#define WCD939X_HPHLR_SURGE_MISC1_SPARE_BITS_MASK                        0x07

/* WCD939X_HPHLR_SURGE_STATUS Fields: */
#define WCD939X_HPHLR_SURGE_STATUS_HPHL_CLAMP_SW_STATUS_MASK             0x80
#define WCD939X_HPHLR_SURGE_STATUS_HPHR_CLAMP_SW_STATUS_MASK             0x40
#define WCD939X_HPHLR_SURGE_STATUS_HPHL_PSURGE_COMP_STATUS_MASK          0x20
#define WCD939X_HPHLR_SURGE_STATUS_HPHL_NSURGE_COMP_STATUS_MASK          0x10
#define WCD939X_HPHLR_SURGE_STATUS_HPHR_PSURGE_COMP_STATUS_MASK          0x08
#define WCD939X_HPHLR_SURGE_STATUS_HPHR_NSURGE_COMP_STATUS_MASK          0x04
#define WCD939X_HPHLR_SURGE_STATUS_HPHL_SURGE_DET_INTR_EN_MASK           0x02
#define WCD939X_HPHLR_SURGE_STATUS_HPHR_SURGE_DET_INTR_EN_MASK           0x01


/* WCD939X_EAR_EN_REG Fields: */
#define WCD939X_EAR_EN_REG_EAR_DAC_DATA_RESET_MASK                       0x80
#define WCD939X_EAR_EN_REG_EAR_DAC_DATA_EN_MASK                          0x40
#define WCD939X_EAR_EN_REG_EAR_DAC_REF_EN_MASK                           0x20
#define WCD939X_EAR_EN_REG_EAR_VCM_EN_MASK                               0x10
#define WCD939X_EAR_EN_REG_EAR_AMP_EN_MASK                               0x08
#define WCD939X_EAR_EN_REG_EAR_BIAS_EN_MASK                              0x04
#define WCD939X_EAR_EN_REG_EAR_CNP_FSM_EN_MASK                           0x02
#define WCD939X_EAR_EN_REG_EAR_OUTPUT_SHORT_MASK                         0x01

/* WCD939X_EAR_PA_CON Fields: */
#define WCD939X_EAR_PA_CON_EAR_ANA_AUX_EN_MASK                           0x80
#define WCD939X_EAR_PA_CON_EAR_CMFB_SF_BYPASS_MASK                       0x40
#define WCD939X_EAR_PA_CON_EAR_SF_CURR_MASK                              0x20
#define WCD939X_EAR_PA_CON_EAR_BTI_CTL_MASK                              0x10
#define WCD939X_EAR_PA_CON_EAR_GM3_IBIAS_CTL_MASK                        0x0f

/* WCD939X_EAR_SP_CON Fields: */
#define WCD939X_EAR_SP_CON_EAR_SP_INT_EN_MASK                            0x80
#define WCD939X_EAR_SP_CON_EAR_SP_AUTO_SHT_DWN_MASK                      0x40
#define WCD939X_EAR_SP_CON_SP_LIMIT_CURR_NMOS_MASK                       0x38
#define WCD939X_EAR_SP_CON_SP_LIMIT_CURR_PMOS_MASK                       0x07

/* WCD939X_EAR_DAC_CON Fields: */
#define WCD939X_EAR_DAC_CON_DAC_SAMPLE_EDGE_SEL_MASK                     0x80
#define WCD939X_EAR_DAC_CON_REF_DBG_EN_MASK                              0x40
#define WCD939X_EAR_DAC_CON_REF_DBG_GAIN_MASK                            0x38
#define WCD939X_EAR_DAC_CON_GAIN_DAC_MASK                                0x06
#define WCD939X_EAR_DAC_CON_INV_DATA_MASK                                0x01

/* WCD939X_EAR_CNP_FSM_CON Fields: */
#define WCD939X_EAR_CNP_FSM_CON_CNP_FSM_CLK_DIV1_MASK                    0xf0
#define WCD939X_EAR_CNP_FSM_CON_CNP_FSM_CLK_DIV2_MASK                    0x0c
#define WCD939X_EAR_CNP_FSM_CON_SCD_FSM_DEGLITCH_SEL_MASK                0x03

/* WCD939X_EAR_TEST_CTL Fields: */
#define WCD939X_EAR_TEST_CTL_DTEST_EN_MASK                               0x80
#define WCD939X_EAR_TEST_CTL_DTEST_SEL_2_MASK                            0x40
#define WCD939X_EAR_TEST_CTL_EAR_RDAC_ATEST_EN_MASK                      0x20
#define WCD939X_EAR_TEST_CTL_EAR_PA_ATEST_SEL_MASK                       0x1f

/* WCD939X_STATUS_REG_1 Fields: */
#define WCD939X_STATUS_REG_1_SP_INT_MASK                                 0x80
#define WCD939X_STATUS_REG_1_SP_ALL_OUT_MASK                             0x40
#define WCD939X_STATUS_REG_1_SP_NMOS_OUT_MASK                            0x20
#define WCD939X_STATUS_REG_1_SP_PMOS_OUT_MASK                            0x10
#define WCD939X_STATUS_REG_1_PA_READY_MASK                               0x08
#define WCD939X_STATUS_REG_1_CNP_FSM_STATUS_MASK                         0x04

/* WCD939X_STATUS_REG_2 Fields: */
#define WCD939X_STATUS_REG_2_PA_EN_MASK                                  0x80
#define WCD939X_STATUS_REG_2_BIAS_EN_MASK                                0x40
#define WCD939X_STATUS_REG_2_DAC_EN_MASK                                 0x20
#define WCD939X_STATUS_REG_2_VCM_EN_MASK                                 0x10
#define WCD939X_STATUS_REG_2_CLK_EN_MASK                                 0x08
#define WCD939X_STATUS_REG_2_SCD_EN_MASK                                 0x04
#define WCD939X_STATUS_REG_2_SHORT_EN_MASK                               0x02
#define WCD939X_STATUS_REG_2_DAC_RESET_MASK                              0x01


/* WCD939X_ANA_NEW_PAGE Fields: */
#define WCD939X_ANA_NEW_PAGE_VALUE_MASK                                  0xff


/* WCD939X_ANA_HPH2 Fields: */
#define WCD939X_ANA_HPH2_HIFI_2VPK_PA_GAIN_CTL_MASK                      0x80
#define WCD939X_ANA_HPH2_ULP_VREF_CTL_MASK                               0x40
#define WCD939X_ANA_HPH2_SPARE_BITS_MASK                                 0x3f

/* WCD939X_ANA_HPH3 Fields: */
#define WCD939X_ANA_HPH3_SPARE_BITS_MASK                                 0xff


/* WCD939X_SLEEP_CTL Fields: */
#define WCD939X_SLEEP_CTL_SPARE_BITS_MASK                                0x80
#define WCD939X_SLEEP_CTL_LDOL_BG_SEL_MASK                               0x10
#define WCD939X_SLEEP_CTL_BG_CTL_MASK                                    0x0e
#define WCD939X_SLEEP_CTL_DTEST_EN_MASK                                  0x01

/* WCD939X_WATCHDOG_CTL Fields: */
#define WCD939X_WATCHDOG_CTL_EN_WATCHDOG_MASK                            0x80
#define WCD939X_WATCHDOG_CTL_EN_WATCHDOG_VREFGEN_MASK                    0x40
#define WCD939X_WATCHDOG_CTL_BYPASS_WATCHDOG_MASK                        0x20
#define WCD939X_WATCHDOG_CTL_ATEST_CTL_MASK                              0x1c


/* WCD939X_ELECT_REM_CLAMP_CTL Fields: */
#define WCD939X_ELECT_REM_CLAMP_CTL_FSM_ELECT_CLAMP_EN_MASK              0x80
#define WCD939X_ELECT_REM_CLAMP_CTL_SLNQ_ELECT_CLAMP_EN_MASK             0x40
#define WCD939X_ELECT_REM_CLAMP_CTL_SLNQ_FAIL_CLAMP_EN_MASK              0x20
#define WCD939X_ELECT_REM_CLAMP_CTL_SLNQ_ELECT_REM_RST_MASK              0x10

/* WCD939X_CTL_1 Fields: */
#define WCD939X_CTL_1_RCO_EN_MASK                                        0x80
#define WCD939X_CTL_1_ADC_MODE_MASK                                      0x10
#define WCD939X_CTL_1_ADC_ENABLE_MASK                                    0x08
#define WCD939X_CTL_1_DETECTION_DONE_MASK                                0x04
#define WCD939X_CTL_1_BTN_DBNC_CTL_MASK                                  0x03

/* WCD939X_CTL_2 Fields: */
#define WCD939X_CTL_2_MUX_CTL_MASK                                       0x70
#define WCD939X_CTL_2_M_RTH_CTL_MASK                                     0x0c
#define WCD939X_CTL_2_HS_VREF_CTL_MASK                                   0x03

/* WCD939X_PLUG_DETECT_CTL Fields: */
#define WCD939X_PLUG_DETECT_CTL_SPARE_BITS_7_6_MASK                      0xc0
#define WCD939X_PLUG_DETECT_CTL_MIC_CLAMP_CTL_MASK                       0x30
#define WCD939X_PLUG_DETECT_CTL_INSREM_DBNC_CTL_MASK                     0x0f

/* WCD939X_ZDET_ANA_CTL Fields: */
#define WCD939X_ZDET_ANA_CTL_AVERAGING_EN_MASK                           0x80
#define WCD939X_ZDET_ANA_CTL_ZDET_MAXV_CTL_MASK                          0x70
#define WCD939X_ZDET_ANA_CTL_ZDET_RANGE_CTL_MASK                         0x0f

/* WCD939X_ZDET_RAMP_CTL Fields: */
#define WCD939X_ZDET_RAMP_CTL_ZDET_ACC1_MIN_CTL_MASK                     0x70
#define WCD939X_ZDET_RAMP_CTL_ZDET_RAMP_TIME_CTL_MASK                    0x0f

/* WCD939X_FSM_STATUS Fields: */
#define WCD939X_FSM_STATUS_ADC_TIMEOUT_MASK                              0x80
#define WCD939X_FSM_STATUS_ADC_COMPLETE_MASK                             0x40
#define WCD939X_FSM_STATUS_HS_M_COMP_STATUS_MASK                         0x20
#define WCD939X_FSM_STATUS_FAST_PRESS_FLAG_STATUS_MASK                   0x10
#define WCD939X_FSM_STATUS_FAST_REMOVAL_FLAG_STATUS_MASK                 0x08
#define WCD939X_FSM_STATUS_REMOVAL_FLAG_STATUS_MASK                      0x04
#define WCD939X_FSM_STATUS_ELECT_REM_RT_STATUS_MASK                      0x02
#define WCD939X_FSM_STATUS_BTN_STATUS_MASK                               0x01

/* WCD939X_ADC_RESULT Fields: */
#define WCD939X_ADC_RESULT_ADC_RESULT_MASK                               0xff


/* WCD939X_TX_CH12_MUX Fields: */
#define WCD939X_TX_CH12_MUX_SPARE_BITS_MASK                              0xc0
#define WCD939X_TX_CH12_MUX_CH2_SEL_MASK                                 0x38
#define WCD939X_TX_CH12_MUX_CH1_SEL_MASK                                 0x07

/* WCD939X_TX_CH34_MUX Fields: */
#define WCD939X_TX_CH34_MUX_SPARE_BITS_MASK                              0xc0
#define WCD939X_TX_CH34_MUX_CH4_SEL_MASK                                 0x38
#define WCD939X_TX_CH34_MUX_CH3_SEL_MASK                                 0x07


/* WCD939X_DIE_CRK_DET_EN Fields: */
#define WCD939X_DIE_CRK_DET_EN_DIE_CRK_DET_EN_MASK                       0x80
#define WCD939X_DIE_CRK_DET_EN_SEL_CURR_INJCT_PT_MRING_MASK              0x40

/* WCD939X_DIE_CRK_DET_OUT Fields: */
#define WCD939X_DIE_CRK_DET_OUT_DIE_CRK_DET_OUT_MASK                     0x80


/* WCD939X_RDAC_GAIN_CTL Fields: */
#define WCD939X_RDAC_GAIN_CTL_SPARE_BITS_MASK                            0xff

/* WCD939X_PA_GAIN_CTL_L Fields: */
#define WCD939X_PA_GAIN_CTL_L_EN_HPHPA_2VPK_MASK                         0x80
#define WCD939X_PA_GAIN_CTL_L_RX_SUPPLY_LEVEL_MASK                       0x40
#define WCD939X_PA_GAIN_CTL_L_DAC_DR_BOOST_MASK                          0x20
#define WCD939X_PA_GAIN_CTL_L_PA_GAIN_L_MASK                             0x1f

/* WCD939X_RDAC_VREF_CTL Fields: */
#define WCD939X_RDAC_VREF_CTL_DAC_REF_EFUSE_TUNE_EN_MASK                 0x80
#define WCD939X_RDAC_VREF_CTL_DAC_VREFN_TUNE_MASK                        0x70
#define WCD939X_RDAC_VREF_CTL_REFCURRENT_2UA_MASK                        0x08
#define WCD939X_RDAC_VREF_CTL_DAC_VREFP_TUNE_MASK                        0x07

/* WCD939X_RDAC_OVERRIDE_CTL Fields: */
#define WCD939X_RDAC_OVERRIDE_CTL_VDDRX_LDO_LIFT_BYPASS_MASK             0x80
#define WCD939X_RDAC_OVERRIDE_CTL_REFBUF_IREF_OVRIDE_MASK                0x40
#define WCD939X_RDAC_OVERRIDE_CTL_SPARE_BITS1_MASK                       0x30
#define WCD939X_RDAC_OVERRIDE_CTL_RDAC_IDLE_DETECT_OVERRIDE_MASK         0x08
#define WCD939X_RDAC_OVERRIDE_CTL_SPARE_BITS2_MASK                       0x07

/* WCD939X_PA_GAIN_CTL_R Fields: */
#define WCD939X_PA_GAIN_CTL_R_D_RCO_CLK_EN_MASK                          0x80
#define WCD939X_PA_GAIN_CTL_R_SPARE_BITS_MASK                            0x60
#define WCD939X_PA_GAIN_CTL_R_PA_GAIN_R_MASK                             0x1f

/* WCD939X_PA_MISC1 Fields: */
#define WCD939X_PA_MISC1_EN_AUTO_CMPDR_DETECTION_MASK                    0x80
#define WCD939X_PA_MISC1_EN_PA_IDLE_DETECT_OVERRIDE_MASK                 0x40
#define WCD939X_PA_MISC1_D_PZ_INF_EN_MASK                                0x20
#define WCD939X_PA_MISC1_HPHPA_BW_PROG_MASK                              0x18
#define WCD939X_PA_MISC1_PA_CHOP_EN_OVERRIDE_MASK                        0x04
#define WCD939X_PA_MISC1_OCP_FSM_LOCK_EN_MASK                            0x02
#define WCD939X_PA_MISC1_AUTOCHOP_PDN_SEQ_OVERRIDE_MASK                  0x01

/* WCD939X_PA_MISC2 Fields: */
#define WCD939X_PA_MISC2_HPHPA_HI_Z_MASK                                 0x80
#define WCD939X_PA_MISC2_HPH_PSRR_ENH_MASK                               0x40
#define WCD939X_PA_MISC2_FORCE_IQCTRL_MASK                               0x20
#define WCD939X_PA_MISC2_FORCE_PSRREH_MASK                               0x10
#define WCD939X_PA_MISC2_CHOP_CLKLAP_SEL_MASK                            0x08
#define WCD939X_PA_MISC2_SPARE_BITS_MASK                                 0x04
#define WCD939X_PA_MISC2_IDLE_DETECT_L_DTEST_ENABLE_MASK                 0x02
#define WCD939X_PA_MISC2_IDLE_DETECT_R_DTEST_ENABLE_MASK                 0x01

/* WCD939X_PA_RDAC_MISC Fields: */
#define WCD939X_PA_RDAC_MISC_CNP_WG_FINE_TIME_LSB_CTL_MASK               0xf0
#define WCD939X_PA_RDAC_MISC_RDAC_NSW_REG_CTL_MASK                       0x08
#define WCD939X_PA_RDAC_MISC_RDAC_PSW_NSW_CTL_OVERRIDE_MASK              0x04
#define WCD939X_PA_RDAC_MISC_RDAC_PSW_NSW_REG_CTL_MASK                   0x03

/* WCD939X_HPH_TIMER1 Fields: */
#define WCD939X_HPH_TIMER1_CURR_IDIV_CTL_CMPDR_OFF_MASK                  0xe0
#define WCD939X_HPH_TIMER1_CURR_IDIV_CTL_AUTOCHOP_MASK                   0x1c
#define WCD939X_HPH_TIMER1_AUTOCHOP_TIMER_CTL_EN_MASK                    0x02
#define WCD939X_HPH_TIMER1_SPARE_BITS_MASK                               0x01

/* WCD939X_HPH_TIMER2 Fields: */
#define WCD939X_HPH_TIMER2_VREF_TIMER_IDLESTATE_MASK                     0xe0
#define WCD939X_HPH_TIMER2_CNP_WG_FINE_TIME_LSB_CTL_IDLE_MASK            0x1e
#define WCD939X_HPH_TIMER2_SPARE_BITS_MASK                               0x01

/* WCD939X_HPH_TIMER3 Fields: */
#define WCD939X_HPH_TIMER3_WG_FINE_TIMER_CMPDR_OFF_MASK                  0xff

/* WCD939X_HPH_TIMER4 Fields: */
#define WCD939X_HPH_TIMER4_WG_FINE_TIMER_AUTOCHOP_MASK                   0xff

/* WCD939X_PA_RDAC_MISC2 Fields: */
#define WCD939X_PA_RDAC_MISC2_SPARE_BITS_MASK                            0xe0
#define WCD939X_PA_RDAC_MISC2_RDAC_DNW_RES_FORCE_BYPASS_MASK             0x10
#define WCD939X_PA_RDAC_MISC2_SCLPF_BYPASS_TIMER_STG1_MASK               0x0c
#define WCD939X_PA_RDAC_MISC2_SCLPF_BYPASS_TIMER_STG2_MASK               0x03

/* WCD939X_PA_RDAC_MISC3 Fields: */
#define WCD939X_PA_RDAC_MISC3_SPARE_BITS_MASK                            0xff

/* WCD939X_RDAC_HD2_CTL_L Fields: */
#define WCD939X_RDAC_HD2_CTL_L_EN_HD2_RES_DIV_L_MASK                     0x80
#define WCD939X_RDAC_HD2_CTL_L_HD2_RES_DIV_PULLGND_L_MASK                0x40
#define WCD939X_RDAC_HD2_CTL_L_HD2_RES_DIV_CTL_L_MASK                    0x3f

/* WCD939X_RDAC_HD2_CTL_R Fields: */
#define WCD939X_RDAC_HD2_CTL_R_EN_HD2_RES_DIV_R_MASK                     0x80
#define WCD939X_RDAC_HD2_CTL_R_HD2_RES_DIV_PULLGND_L_MASK                0x40
#define WCD939X_RDAC_HD2_CTL_R_HD2_RES_DIV_CTL_R_MASK                    0x3f


/* WCD939X_HPH_RDAC_BIAS_LOHIFI Fields: */
#define WCD939X_HPH_RDAC_BIAS_LOHIFI_HPHPA_BIAS_LOHIFI_MASK              0xf0
#define WCD939X_HPH_RDAC_BIAS_LOHIFI_HPHRDAC_BIAS_LOHIFI_MASK            0x0f

/* WCD939X_HPH_RDAC_BIAS_ULP Fields: */
#define WCD939X_HPH_RDAC_BIAS_ULP_SLEEPBG_PWR_SEL_MASK                   0x80
#define WCD939X_HPH_RDAC_BIAS_ULP_SLEEPBG_PWR_SEL_OVERRIDE_MASK          0x40
#define WCD939X_HPH_RDAC_BIAS_ULP_CDC_3P5MM_LEGACY_IN_MASK               0x20
#define WCD939X_HPH_RDAC_BIAS_ULP_SPARE_BITS1_MASK                       0x10
#define WCD939X_HPH_RDAC_BIAS_ULP_HPHRDAC_BIAS_ULP_MASK                  0x0f

/* WCD939X_HPH_RDAC_LDO_LP Fields: */
#define WCD939X_HPH_RDAC_LDO_LP_HPHRDAC_1P6VLDO_BIAS_LP_MASK             0xf0
#define WCD939X_HPH_RDAC_LDO_LP_HPHRDAC_N1P6VLDO_BIAS_LP_MASK            0x0f


/* WCD939X_MOISTURE_DET_DC_CTRL Fields: */
#define WCD939X_MOISTURE_DET_DC_CTRL_ONCOUNT_MASK                        0x60
#define WCD939X_MOISTURE_DET_DC_CTRL_OFFCOUNT_MASK                       0x1f

/* WCD939X_MOISTURE_DET_POLLING_CTRL Fields: */
#define WCD939X_MOISTURE_DET_POLLING_CTRL_HPHL_PA_EN_MASK                0x40
#define WCD939X_MOISTURE_DET_POLLING_CTRL_DTEST_EN_MASK                  0x30
#define WCD939X_MOISTURE_DET_POLLING_CTRL_MOISTURE_OVRD_POLLING_MASK     0x08
#define WCD939X_MOISTURE_DET_POLLING_CTRL_MOISTURE_EN_POLLING_MASK       0x04
#define WCD939X_MOISTURE_DET_POLLING_CTRL_MOISTURE_DBNC_TIME_MASK        0x03

/* WCD939X_MECH_DET_CURRENT Fields: */
#define WCD939X_MECH_DET_CURRENT_HSDET_PULLUP_CTL_MASK                   0x1f

/* WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW Fields: */
#define WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW_SPARE_BITS_7_MASK          0x80
#define WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW_ZDET_CLK_SEL_MASK          0x40
#define WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW_ZDET_SUBSEL_OV_MASK        0x20
#define WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW_ZDET_CLK_EN_CTL_MASK       0x10
#define WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW_MOIS_CURRENT_CTL_SEL_MASK  0x08
#define WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW_MOIS_CURRENT_ADD_MASK      0x04
#define WCD939X_ZDET_CLK_AND_MOISTURE_CTL_NEW_MECH_REF_SEL_MASK          0x03


/* WCD939X_EAR_CHOPPER_CON Fields: */
#define WCD939X_EAR_CHOPPER_CON_EAR_CHOPPER_EN_MASK                      0x80
#define WCD939X_EAR_CHOPPER_CON_EAR_CHOPPER_CLK_DIV_MASK                 0x78
#define WCD939X_EAR_CHOPPER_CON_EAR_CHOPPER_CLK_INV_MASK                 0x04
#define WCD939X_EAR_CHOPPER_CON_EAR_CHOPPER_CLK_OVERLAP_MASK             0x02
#define WCD939X_EAR_CHOPPER_CON_SCD_SHTDWN_FAST_PATH_DIS_MASK            0x01

/* WCD939X_CNP_VCM_CON1 Fields: */
#define WCD939X_CNP_VCM_CON1_SCD_EN_TIME_SEL_MASK                        0x80
#define WCD939X_CNP_VCM_CON1_NO_DYN_BIAS_DURING_STARTUP_MASK             0x40
#define WCD939X_CNP_VCM_CON1_CNP_VCM_GEN_START_MASK                      0x3f

/* WCD939X_CNP_VCM_CON2 Fields: */
#define WCD939X_CNP_VCM_CON2_DTEST_SEL_MASK                              0xc0
#define WCD939X_CNP_VCM_CON2_CNP_VCM_GEN_STOP_MASK                       0x3f

/* WCD939X_EAR_DYNAMIC_BIAS Fields: */
#define WCD939X_EAR_DYNAMIC_BIAS_EAR_DYN_BIAS_SEL_MASK                   0xe0
#define WCD939X_EAR_DYNAMIC_BIAS_EAR_BIAS_CURR_MASK                      0x1f


/* WCD939X_WATCHDOG_CTL_1 Fields: */
#define WCD939X_WATCHDOG_CTL_1_VREF_HI_CTL_MASK                          0x1f

/* WCD939X_WATCHDOG_CTL_2 Fields: */
#define WCD939X_WATCHDOG_CTL_2_VREF_LO_CTL_MASK                          0x1f


/* WCD939X_DIE_CRK_DET_INT1 Fields: */
#define WCD939X_DIE_CRK_DET_INT1_SEL_EDGE_DET_MASK                       0xc0
#define WCD939X_DIE_CRK_DET_INT1_EN_RINGM_ATEST_MASK                     0x20
#define WCD939X_DIE_CRK_DET_INT1_EN_RINGP_ATEST_MASK                     0x10
#define WCD939X_DIE_CRK_DET_INT1_RING_CURR_SEL_MASK                      0x0e
#define WCD939X_DIE_CRK_DET_INT1_EN_VREF_ATEST_MASK                      0x01

/* WCD939X_DIE_CRK_DET_INT2 Fields: */
#define WCD939X_DIE_CRK_DET_INT2_REF_CURR_SEL_MASK                       0xe0
#define WCD939X_DIE_CRK_DET_INT2_COMP_STG1_IBIAS_MASK                    0x18
#define WCD939X_DIE_CRK_DET_INT2_COMP_STG2_IBIAS_MASK                    0x06
#define WCD939X_DIE_CRK_DET_INT2_EN_ATEST_MASK                           0x01


/* WCD939X_TXFE_DIVSTOP_L2 Fields: */
#define WCD939X_TXFE_DIVSTOP_L2_DIV_L2_MASK                              0xff

/* WCD939X_TXFE_DIVSTOP_L1 Fields: */
#define WCD939X_TXFE_DIVSTOP_L1_DIV_L1_MASK                              0xff

/* WCD939X_TXFE_DIVSTOP_L0 Fields: */
#define WCD939X_TXFE_DIVSTOP_L0_DIV_L0_MASK                              0xff

/* WCD939X_TXFE_DIVSTOP_ULP1P2M Fields: */
#define WCD939X_TXFE_DIVSTOP_ULP1P2M_DIV_ULP1P2M_MASK                    0xff

/* WCD939X_TXFE_DIVSTOP_ULP0P6M Fields: */
#define WCD939X_TXFE_DIVSTOP_ULP0P6M_DIV_ULP0P6M_MASK                    0xff

/* WCD939X_TXFE_ICTRL_STG1_L2L1 Fields: */
#define WCD939X_TXFE_ICTRL_STG1_L2L1_NINIT_L2_MASK                       0xc0
#define WCD939X_TXFE_ICTRL_STG1_L2L1_ICTRL_STG1_L2L1_MASK                0x1f

/* WCD939X_TXFE_ICTRL_STG1_L0 Fields: */
#define WCD939X_TXFE_ICTRL_STG1_L0_NINIT_L1_MASK                         0xc0
#define WCD939X_TXFE_ICTRL_STG1_L0_ICTRL_STG1_L0_MASK                    0x1f

/* WCD939X_TXFE_ICTRL_STG1_ULP Fields: */
#define WCD939X_TXFE_ICTRL_STG1_ULP_NINIT_L0_MASK                        0xc0
#define WCD939X_TXFE_ICTRL_STG1_ULP_ICTRL_STG1_ULP_MASK                  0x1f

/* WCD939X_TXFE_ICTRL_STG2MAIN_L2L1 Fields: */
#define WCD939X_TXFE_ICTRL_STG2MAIN_L2L1_NINIT_ULP1P2M_MASK              0xc0
#define WCD939X_TXFE_ICTRL_STG2MAIN_L2L1_ICTRL_STG2MAIN_L2L1_MASK        0x1f

/* WCD939X_TXFE_ICTRL_STG2MAIN_L0 Fields: */
#define WCD939X_TXFE_ICTRL_STG2MAIN_L0_NINIT_ULP0P6M_MASK                0xc0
#define WCD939X_TXFE_ICTRL_STG2MAIN_L0_ADCREF_ULPIBIAS_EN_MASK           0x20
#define WCD939X_TXFE_ICTRL_STG2MAIN_L0_ICTRL_STG2MAIN_L0_MASK            0x1f

/* WCD939X_TXFE_ICTRL_STG2MAIN_ULP Fields: */
#define WCD939X_TXFE_ICTRL_STG2MAIN_ULP_ICTRL_STG2MAIN_ULP_MASK          0x1f

/* WCD939X_TXFE_ICTRL_STG2CASC_L2L1L0 Fields: */
#define WCD939X_TXFE_ICTRL_STG2CASC_L2L1L0_ICTRL_STG2CASC_L2L1_MASK      0xf0
#define WCD939X_TXFE_ICTRL_STG2CASC_L2L1L0_ICTRL_STG2CASC_L0_MASK        0x0f

/* WCD939X_TXFE_ICTRL_STG2CASC_ULP Fields: */
#define WCD939X_TXFE_ICTRL_STG2CASC_ULP_ICTRL_SCBIAS_ULP0P6M_MASK        0xf0
#define WCD939X_TXFE_ICTRL_STG2CASC_ULP_ICTRL_STG2CASC_ULP_MASK          0x0f

/* WCD939X_TXADC_SCBIAS_L2L1 Fields: */
#define WCD939X_TXADC_SCBIAS_L2L1_ICTRL_SCBIAS_L2_MASK                   0xf0
#define WCD939X_TXADC_SCBIAS_L2L1_ICTRL_SCBIAS_L1_MASK                   0x0f

/* WCD939X_TXADC_SCBIAS_L0ULP Fields: */
#define WCD939X_TXADC_SCBIAS_L0ULP_ICTRL_SCBIAS_L0_MASK                  0xf0
#define WCD939X_TXADC_SCBIAS_L0ULP_ICTRL_SCBIAS_ULP1P2M_MASK             0x0f

/* WCD939X_TXADC_INT_L2 Fields: */
#define WCD939X_TXADC_INT_L2_INT1_L2_MASK                                0xf0
#define WCD939X_TXADC_INT_L2_INT2_L2_MASK                                0x0f

/* WCD939X_TXADC_INT_L1 Fields: */
#define WCD939X_TXADC_INT_L1_INT1_L1_MASK                                0xf0
#define WCD939X_TXADC_INT_L1_INT2_L1_MASK                                0x0f

/* WCD939X_TXADC_INT_L0 Fields: */
#define WCD939X_TXADC_INT_L0_INT1_L0_MASK                                0xf0
#define WCD939X_TXADC_INT_L0_INT2_L0_MASK                                0x0f

/* WCD939X_TXADC_INT_ULP Fields: */
#define WCD939X_TXADC_INT_ULP_INT1_ULP_MASK                              0xf0
#define WCD939X_TXADC_INT_ULP_INT2_ULP_MASK                              0x0f


/* WCD939X_DIGITAL_PAGE Fields: */
#define WCD939X_DIGITAL_PAGE_PAG_REG_MASK                                0xff

/* WCD939X_CHIP_ID0 Fields: */
#define WCD939X_CHIP_ID0_BYTE_0_MASK                                     0xff

/* WCD939X_CHIP_ID1 Fields: */
#define WCD939X_CHIP_ID1_BYTE_1_MASK                                     0xff

/* WCD939X_CHIP_ID2 Fields: */
#define WCD939X_CHIP_ID2_BYTE_2_MASK                                     0xff

/* WCD939X_CHIP_ID3 Fields: */
#define WCD939X_CHIP_ID3_BYTE_3_MASK                                     0xff

/* WCD939X_SWR_TX_CLK_RATE Fields: */
#define WCD939X_SWR_TX_CLK_RATE_CLK_RATE_BK_1_MASK                       0xf0
#define WCD939X_SWR_TX_CLK_RATE_CLK_RATE_BK_0_MASK                       0x0f

/* WCD939X_CDC_RST_CTL Fields: */
#define WCD939X_CDC_RST_CTL_ANA_SW_RST_N_MASK                            0x02
#define WCD939X_CDC_RST_CTL_DIG_SW_RST_N_MASK                            0x01

/* WCD939X_TOP_CLK_CFG Fields: */
#define WCD939X_TOP_CLK_CFG_RX_CLK_CFG_MASK                              0x06
#define WCD939X_TOP_CLK_CFG_TX_CLK_CFG_MASK                              0x01

/* WCD939X_CDC_ANA_CLK_CTL Fields: */
#define WCD939X_CDC_ANA_CLK_CTL_ANA_TX_DIV4_CLK_EN_MASK                  0x20
#define WCD939X_CDC_ANA_CLK_CTL_ANA_TX_DIV2_CLK_EN_MASK                  0x10
#define WCD939X_CDC_ANA_CLK_CTL_ANA_TX_CLK_EN_MASK                       0x08
#define WCD939X_CDC_ANA_CLK_CTL_ANA_RX_DIV4_CLK_EN_MASK                  0x04
#define WCD939X_CDC_ANA_CLK_CTL_ANA_RX_DIV2_CLK_EN_MASK                  0x02
#define WCD939X_CDC_ANA_CLK_CTL_ANA_RX_CLK_EN_MASK                       0x01

/* WCD939X_CDC_DIG_CLK_CTL Fields: */
#define WCD939X_CDC_DIG_CLK_CTL_TXD3_CLK_EN_MASK                         0x80
#define WCD939X_CDC_DIG_CLK_CTL_TXD2_CLK_EN_MASK                         0x40
#define WCD939X_CDC_DIG_CLK_CTL_TXD1_CLK_EN_MASK                         0x20
#define WCD939X_CDC_DIG_CLK_CTL_TXD0_CLK_EN_MASK                         0x10
#define WCD939X_CDC_DIG_CLK_CTL_RXD2_CLK_EN_MASK                         0x04
#define WCD939X_CDC_DIG_CLK_CTL_RXD1_CLK_EN_MASK                         0x02
#define WCD939X_CDC_DIG_CLK_CTL_RXD0_CLK_EN_MASK                         0x01

/* WCD939X_SWR_RST_EN Fields: */
#define WCD939X_SWR_RST_EN_RX_RESET_SYNC_LOST_EN_MASK                    0x20
#define WCD939X_SWR_RST_EN_RX_RESET_SWR_BUS_EN_MASK                      0x10
#define WCD939X_SWR_RST_EN_RX_RESET_SWR_REG_EN_MASK                      0x08
#define WCD939X_SWR_RST_EN_TX_RESET_SYNC_LOST_EN_MASK                    0x04
#define WCD939X_SWR_RST_EN_TX_RESET_SWR_BUS_EN_MASK                      0x02
#define WCD939X_SWR_RST_EN_TX_RESET_SWR_REG_EN_MASK                      0x01

/* WCD939X_CDC_PATH_MODE Fields: */
#define WCD939X_CDC_PATH_MODE_EAR_MODE_MASK                              0x40
#define WCD939X_CDC_PATH_MODE_TXD2_MODE_MASK                             0x30
#define WCD939X_CDC_PATH_MODE_TXD1_MODE_MASK                             0x0c
#define WCD939X_CDC_PATH_MODE_TXD0_MODE_MASK                             0x03

/* WCD939X_CDC_RX_RST Fields: */
#define WCD939X_CDC_RX_RST_RX2_SOFT_RST_MASK                             0x04
#define WCD939X_CDC_RX_RST_RX1_SOFT_RST_MASK                             0x02
#define WCD939X_CDC_RX_RST_RX0_SOFT_RST_MASK                             0x01

/* WCD939X_CDC_RX0_CTL Fields: */
#define WCD939X_CDC_RX0_CTL_DSM_DITHER_ENABLE_MASK                       0x80
#define WCD939X_CDC_RX0_CTL_DEM_DITHER_ENABLE_MASK                       0x40
#define WCD939X_CDC_RX0_CTL_DEM_MID_ENABLE_MASK                          0x20
#define WCD939X_CDC_RX0_CTL_DEM_MOD_SWITCHING_BLOCK_ENABLE_MASK          0x10
#define WCD939X_CDC_RX0_CTL_DEM_SWITCHING_BLOCK_ENABLE_MASK              0x08
#define WCD939X_CDC_RX0_CTL_DEM_SEGMENTING_BLOCK_ENABLE_MASK             0x04
#define WCD939X_CDC_RX0_CTL_DEM_BYPASS_MASK                              0x02

/* WCD939X_CDC_RX1_CTL Fields: */
#define WCD939X_CDC_RX1_CTL_DSM_DITHER_ENABLE_MASK                       0x80
#define WCD939X_CDC_RX1_CTL_DEM_DITHER_ENABLE_MASK                       0x40
#define WCD939X_CDC_RX1_CTL_DEM_MID_ENABLE_MASK                          0x20
#define WCD939X_CDC_RX1_CTL_DEM_MOD_SWITCHING_BLOCK_ENABLE_MASK          0x10
#define WCD939X_CDC_RX1_CTL_DEM_SWITCHING_BLOCK_ENABLE_MASK              0x08
#define WCD939X_CDC_RX1_CTL_DEM_SEGMENTING_BLOCK_ENABLE_MASK             0x04
#define WCD939X_CDC_RX1_CTL_DEM_BYPASS_MASK                              0x02

/* WCD939X_CDC_RX2_CTL Fields: */
#define WCD939X_CDC_RX2_CTL_DSM_DITHER_ENABLE_MASK                       0x80
#define WCD939X_CDC_RX2_CTL_DEM_DITHER_ENABLE_MASK                       0x40
#define WCD939X_CDC_RX2_CTL_DEM_MID_ENABLE_MASK                          0x20
#define WCD939X_CDC_RX2_CTL_DEM_MOD_SWITCHING_BLOCK_ENABLE_MASK          0x10
#define WCD939X_CDC_RX2_CTL_DEM_SWITCHING_BLOCK_ENABLE_MASK              0x08
#define WCD939X_CDC_RX2_CTL_DEM_SEGMENTING_BLOCK_ENABLE_MASK             0x04
#define WCD939X_CDC_RX2_CTL_DEM_BYPASS_MASK                              0x02

/* WCD939X_CDC_TX_ANA_MODE_0_1 Fields: */
#define WCD939X_CDC_TX_ANA_MODE_0_1_TXD1_MODE_MASK                       0xf0
#define WCD939X_CDC_TX_ANA_MODE_0_1_TXD0_MODE_MASK                       0x0f

/* WCD939X_CDC_TX_ANA_MODE_2_3 Fields: */
#define WCD939X_CDC_TX_ANA_MODE_2_3_TXD3_MODE_MASK                       0xf0
#define WCD939X_CDC_TX_ANA_MODE_2_3_TXD2_MODE_MASK                       0x0f

/* WCD939X_CDC_COMP_CTL_0 Fields: */
#define WCD939X_CDC_COMP_CTL_0_HPHL_COMP_EN_MASK                         0x02
#define WCD939X_CDC_COMP_CTL_0_HPHR_COMP_EN_MASK                         0x01

/* WCD939X_CDC_ANA_TX_CLK_CTL Fields: */
#define WCD939X_CDC_ANA_TX_CLK_CTL_ANA_MBHC_1P2M_CLK_EN_MASK             0x20
#define WCD939X_CDC_ANA_TX_CLK_CTL_ANA_TX3_ADC_CLK_EN_MASK               0x10
#define WCD939X_CDC_ANA_TX_CLK_CTL_ANA_TX2_ADC_CLK_EN_MASK               0x08
#define WCD939X_CDC_ANA_TX_CLK_CTL_ANA_TX1_ADC_CLK_EN_MASK               0x04
#define WCD939X_CDC_ANA_TX_CLK_CTL_ANA_TX0_ADC_CLK_EN_MASK               0x02
#define WCD939X_CDC_ANA_TX_CLK_CTL_ANA_TXSCBIAS_CLK_EN_MASK              0x01

/* WCD939X_CDC_HPH_DSM_A1_0 Fields: */
#define WCD939X_CDC_HPH_DSM_A1_0_COEF_A1_MASK                            0xff

/* WCD939X_CDC_HPH_DSM_A1_1 Fields: */
#define WCD939X_CDC_HPH_DSM_A1_1_COEF_A1_MASK                            0x01

/* WCD939X_CDC_HPH_DSM_A2_0 Fields: */
#define WCD939X_CDC_HPH_DSM_A2_0_COEF_A2_MASK                            0xff

/* WCD939X_CDC_HPH_DSM_A2_1 Fields: */
#define WCD939X_CDC_HPH_DSM_A2_1_COEF_A2_MASK                            0x0f

/* WCD939X_CDC_HPH_DSM_A3_0 Fields: */
#define WCD939X_CDC_HPH_DSM_A3_0_COEF_A3_MASK                            0xff

/* WCD939X_CDC_HPH_DSM_A3_1 Fields: */
#define WCD939X_CDC_HPH_DSM_A3_1_COEF_A3_MASK                            0x07

/* WCD939X_CDC_HPH_DSM_A4_0 Fields: */
#define WCD939X_CDC_HPH_DSM_A4_0_COEF_A4_MASK                            0xff

/* WCD939X_CDC_HPH_DSM_A4_1 Fields: */
#define WCD939X_CDC_HPH_DSM_A4_1_COEF_A4_MASK                            0x03

/* WCD939X_CDC_HPH_DSM_A5_0 Fields: */
#define WCD939X_CDC_HPH_DSM_A5_0_COEF_A5_MASK                            0xff

/* WCD939X_CDC_HPH_DSM_A5_1 Fields: */
#define WCD939X_CDC_HPH_DSM_A5_1_COEF_A5_MASK                            0x03

/* WCD939X_CDC_HPH_DSM_A6_0 Fields: */
#define WCD939X_CDC_HPH_DSM_A6_0_COEF_A6_MASK                            0xff

/* WCD939X_CDC_HPH_DSM_A7_0 Fields: */
#define WCD939X_CDC_HPH_DSM_A7_0_COEF_A7_MASK                            0xff

/* WCD939X_CDC_HPH_DSM_C_0 Fields: */
#define WCD939X_CDC_HPH_DSM_C_0_COEF_C3_MASK                             0xf0
#define WCD939X_CDC_HPH_DSM_C_0_COEF_C2_MASK                             0x0f

/* WCD939X_CDC_HPH_DSM_C_1 Fields: */
#define WCD939X_CDC_HPH_DSM_C_1_COEF_C5_MASK                             0xf0
#define WCD939X_CDC_HPH_DSM_C_1_COEF_C4_MASK                             0x0f

/* WCD939X_CDC_HPH_DSM_C_2 Fields: */
#define WCD939X_CDC_HPH_DSM_C_2_COEF_C7_MASK                             0xf0
#define WCD939X_CDC_HPH_DSM_C_2_COEF_C6_MASK                             0x0f

/* WCD939X_CDC_HPH_DSM_C_3 Fields: */
#define WCD939X_CDC_HPH_DSM_C_3_COEF_C7_MASK                             0x3f

/* WCD939X_CDC_HPH_DSM_R1 Fields: */
#define WCD939X_CDC_HPH_DSM_R1_SAT_LIMIT_R1_MASK                         0xff

/* WCD939X_CDC_HPH_DSM_R2 Fields: */
#define WCD939X_CDC_HPH_DSM_R2_SAT_LIMIT_R2_MASK                         0xff

/* WCD939X_CDC_HPH_DSM_R3 Fields: */
#define WCD939X_CDC_HPH_DSM_R3_SAT_LIMIT_R3_MASK                         0xff

/* WCD939X_CDC_HPH_DSM_R4 Fields: */
#define WCD939X_CDC_HPH_DSM_R4_SAT_LIMIT_R4_MASK                         0xff

/* WCD939X_CDC_HPH_DSM_R5 Fields: */
#define WCD939X_CDC_HPH_DSM_R5_SAT_LIMIT_R5_MASK                         0xff

/* WCD939X_CDC_HPH_DSM_R6 Fields: */
#define WCD939X_CDC_HPH_DSM_R6_SAT_LIMIT_R6_MASK                         0xff

/* WCD939X_CDC_HPH_DSM_R7 Fields: */
#define WCD939X_CDC_HPH_DSM_R7_SAT_LIMIT_R7_MASK                         0xff

/* WCD939X_CDC_EAR_DSM_A1_0 Fields: */
#define WCD939X_CDC_EAR_DSM_A1_0_COEF_A1_MASK                            0xff

/* WCD939X_CDC_EAR_DSM_A1_1 Fields: */
#define WCD939X_CDC_EAR_DSM_A1_1_COEF_A1_MASK                            0x01

/* WCD939X_CDC_EAR_DSM_A2_0 Fields: */
#define WCD939X_CDC_EAR_DSM_A2_0_COEF_A2_MASK                            0xff

/* WCD939X_CDC_EAR_DSM_A2_1 Fields: */
#define WCD939X_CDC_EAR_DSM_A2_1_COEF_A2_MASK                            0x0f

/* WCD939X_CDC_EAR_DSM_A3_0 Fields: */
#define WCD939X_CDC_EAR_DSM_A3_0_COEF_A3_MASK                            0xff

/* WCD939X_CDC_EAR_DSM_A3_1 Fields: */
#define WCD939X_CDC_EAR_DSM_A3_1_COEF_A3_MASK                            0x07

/* WCD939X_CDC_EAR_DSM_A4_0 Fields: */
#define WCD939X_CDC_EAR_DSM_A4_0_COEF_A4_MASK                            0xff

/* WCD939X_CDC_EAR_DSM_A4_1 Fields: */
#define WCD939X_CDC_EAR_DSM_A4_1_COEF_A4_MASK                            0x03

/* WCD939X_CDC_EAR_DSM_A5_0 Fields: */
#define WCD939X_CDC_EAR_DSM_A5_0_COEF_A5_MASK                            0xff

/* WCD939X_CDC_EAR_DSM_A5_1 Fields: */
#define WCD939X_CDC_EAR_DSM_A5_1_COEF_A5_MASK                            0x03

/* WCD939X_CDC_EAR_DSM_A6_0 Fields: */
#define WCD939X_CDC_EAR_DSM_A6_0_COEF_A6_MASK                            0xff

/* WCD939X_CDC_EAR_DSM_A7_0 Fields: */
#define WCD939X_CDC_EAR_DSM_A7_0_COEF_A7_MASK                            0xff

/* WCD939X_CDC_EAR_DSM_C_0 Fields: */
#define WCD939X_CDC_EAR_DSM_C_0_COEF_C3_MASK                             0xf0
#define WCD939X_CDC_EAR_DSM_C_0_COEF_C2_MASK                             0x0f

/* WCD939X_CDC_EAR_DSM_C_1 Fields: */
#define WCD939X_CDC_EAR_DSM_C_1_COEF_C5_MASK                             0xf0
#define WCD939X_CDC_EAR_DSM_C_1_COEF_C4_MASK                             0x0f

/* WCD939X_CDC_EAR_DSM_C_2 Fields: */
#define WCD939X_CDC_EAR_DSM_C_2_COEF_C7_MASK                             0xf0
#define WCD939X_CDC_EAR_DSM_C_2_COEF_C6_MASK                             0x0f

/* WCD939X_CDC_EAR_DSM_C_3 Fields: */
#define WCD939X_CDC_EAR_DSM_C_3_COEF_C7_MASK                             0x3f

/* WCD939X_CDC_EAR_DSM_R1 Fields: */
#define WCD939X_CDC_EAR_DSM_R1_SAT_LIMIT_R1_MASK                         0xff

/* WCD939X_CDC_EAR_DSM_R2 Fields: */
#define WCD939X_CDC_EAR_DSM_R2_SAT_LIMIT_R2_MASK                         0xff

/* WCD939X_CDC_EAR_DSM_R3 Fields: */
#define WCD939X_CDC_EAR_DSM_R3_SAT_LIMIT_R3_MASK                         0xff

/* WCD939X_CDC_EAR_DSM_R4 Fields: */
#define WCD939X_CDC_EAR_DSM_R4_SAT_LIMIT_R4_MASK                         0xff

/* WCD939X_CDC_EAR_DSM_R5 Fields: */
#define WCD939X_CDC_EAR_DSM_R5_SAT_LIMIT_R5_MASK                         0xff

/* WCD939X_CDC_EAR_DSM_R6 Fields: */
#define WCD939X_CDC_EAR_DSM_R6_SAT_LIMIT_R6_MASK                         0xff

/* WCD939X_CDC_EAR_DSM_R7 Fields: */
#define WCD939X_CDC_EAR_DSM_R7_SAT_LIMIT_R7_MASK                         0xff

/* WCD939X_CDC_HPH_GAIN_RX_0 Fields: */
#define WCD939X_CDC_HPH_GAIN_RX_0_GAIN_RX_MASK                           0xff

/* WCD939X_CDC_HPH_GAIN_RX_1 Fields: */
#define WCD939X_CDC_HPH_GAIN_RX_1_GAIN_RX_MASK                           0xff

/* WCD939X_CDC_HPH_GAIN_DSD_0 Fields: */
#define WCD939X_CDC_HPH_GAIN_DSD_0_GAIN_DSD_MASK                         0xff

/* WCD939X_CDC_HPH_GAIN_DSD_1 Fields: */
#define WCD939X_CDC_HPH_GAIN_DSD_1_GAIN_DSD_MASK                         0xff

/* WCD939X_CDC_HPH_GAIN_DSD_2 Fields: */
#define WCD939X_CDC_HPH_GAIN_DSD_2_GAIN_LATCH_MASK                       0x02
#define WCD939X_CDC_HPH_GAIN_DSD_2_GAIN_DSD_MASK                         0x01

/* WCD939X_CDC_EAR_GAIN_DSD_0 Fields: */
#define WCD939X_CDC_EAR_GAIN_DSD_0_GAIN_DSD_MASK                         0xff

/* WCD939X_CDC_EAR_GAIN_DSD_1 Fields: */
#define WCD939X_CDC_EAR_GAIN_DSD_1_GAIN_DSD_MASK                         0xff

/* WCD939X_CDC_EAR_GAIN_DSD_2 Fields: */
#define WCD939X_CDC_EAR_GAIN_DSD_2_GAIN_LATCH_MASK                       0x02
#define WCD939X_CDC_EAR_GAIN_DSD_2_GAIN_DSD_MASK                         0x01

/* WCD939X_CDC_HPH_GAIN_CTL Fields: */
#define WCD939X_CDC_HPH_GAIN_CTL_HPH_STEREO_EN_MASK                      0x10
#define WCD939X_CDC_HPH_GAIN_CTL_HPHR_RX_EN_MASK                         0x08
#define WCD939X_CDC_HPH_GAIN_CTL_HPHL_RX_EN_MASK                         0x04
#define WCD939X_CDC_HPH_GAIN_CTL_HPHR_DSD_EN_MASK                        0x02
#define WCD939X_CDC_HPH_GAIN_CTL_HPHL_DSD_EN_MASK                        0x01

/* WCD939X_CDC_EAR_GAIN_CTL Fields: */
#define WCD939X_CDC_EAR_GAIN_CTL_EAR_EN_MASK                             0x01

/* WCD939X_CDC_EAR_PATH_CTL Fields: */
#define WCD939X_CDC_EAR_PATH_CTL_EAR_GAIN_CTL_MASK                       0x3e
#define WCD939X_CDC_EAR_PATH_CTL_EAR_MUX_SEL_MASK                        0x01

/* WCD939X_CDC_SWR_CLH Fields: */
#define WCD939X_CDC_SWR_CLH_CLH_CTL_MASK                                 0xff

/* WCD939X_SWR_CLH_BYP Fields: */
#define WCD939X_SWR_CLH_BYP_SWR_CLH_BYP_MASK                             0x01

/* WCD939X_CDC_TX0_CTL Fields: */
#define WCD939X_CDC_TX0_CTL_REQ_FB_SEL_MASK                              0x40
#define WCD939X_CDC_TX0_CTL_TX_DITHER_EN_MASK                            0x20
#define WCD939X_CDC_TX0_CTL_RANDOM_REGION_MASK                           0x1f

/* WCD939X_CDC_TX1_CTL Fields: */
#define WCD939X_CDC_TX1_CTL_REQ_FB_SEL_MASK                              0x40
#define WCD939X_CDC_TX1_CTL_TX_DITHER_EN_MASK                            0x20
#define WCD939X_CDC_TX1_CTL_RANDOM_REGION_MASK                           0x1f

/* WCD939X_CDC_TX2_CTL Fields: */
#define WCD939X_CDC_TX2_CTL_REQ_FB_SEL_MASK                              0x40
#define WCD939X_CDC_TX2_CTL_TX_DITHER_EN_MASK                            0x20
#define WCD939X_CDC_TX2_CTL_RANDOM_REGION_MASK                           0x1f

/* WCD939X_CDC_TX_RST Fields: */
#define WCD939X_CDC_TX_RST_TX3_SOFT_RST_MASK                             0x08
#define WCD939X_CDC_TX_RST_TX2_SOFT_RST_MASK                             0x04
#define WCD939X_CDC_TX_RST_TX1_SOFT_RST_MASK                             0x02
#define WCD939X_CDC_TX_RST_TX0_SOFT_RST_MASK                             0x01

/* WCD939X_CDC_REQ_CTL Fields: */
#define WCD939X_CDC_REQ_CTL_TX3_WIDE_BAND_MASK                           0x20
#define WCD939X_CDC_REQ_CTL_TX2_WIDE_BAND_MASK                           0x10
#define WCD939X_CDC_REQ_CTL_TX1_WIDE_BAND_MASK                           0x08
#define WCD939X_CDC_REQ_CTL_TX0_WIDE_BAND_MASK                           0x04
#define WCD939X_CDC_REQ_CTL_FS_RATE_4P8_MASK                             0x02
#define WCD939X_CDC_REQ_CTL_NO_NOTCH_MASK                                0x01

/* WCD939X_CDC_RST Fields: */
#define WCD939X_CDC_RST_TX_SOFT_RST_MASK                                 0x02
#define WCD939X_CDC_RST_RX_SOFT_RST_MASK                                 0x01

/* WCD939X_CDC_AMIC_CTL Fields: */
#define WCD939X_CDC_AMIC_CTL_AMIC5_IN_SEL_MASK                           0x08
#define WCD939X_CDC_AMIC_CTL_AMIC4_IN_SEL_MASK                           0x04
#define WCD939X_CDC_AMIC_CTL_AMIC3_IN_SEL_MASK                           0x02
#define WCD939X_CDC_AMIC_CTL_AMIC1_IN_SEL_MASK                           0x01

/* WCD939X_CDC_DMIC_CTL Fields: */
#define WCD939X_CDC_DMIC_CTL_DMIC_LEGACY_SW_MODE_MASK                    0x08
#define WCD939X_CDC_DMIC_CTL_DMIC_DIV_BAK_EN_MASK                        0x04
#define WCD939X_CDC_DMIC_CTL_CLK_SCALE_EN_MASK                           0x02
#define WCD939X_CDC_DMIC_CTL_SOFT_RESET_MASK                             0x01

/* WCD939X_CDC_DMIC1_CTL Fields: */
#define WCD939X_CDC_DMIC1_CTL_DMIC_CLK_SCALE_SEL_MASK                    0x70
#define WCD939X_CDC_DMIC1_CTL_DMIC_CLK_EN_MASK                           0x08
#define WCD939X_CDC_DMIC1_CTL_DMIC_CLK_SEL_MASK                          0x07

/* WCD939X_CDC_DMIC2_CTL Fields: */
#define WCD939X_CDC_DMIC2_CTL_DMIC_LEFT_EN_MASK                          0x80
#define WCD939X_CDC_DMIC2_CTL_DMIC_CLK_SCALE_SEL_MASK                    0x70
#define WCD939X_CDC_DMIC2_CTL_DMIC_CLK_EN_MASK                           0x08
#define WCD939X_CDC_DMIC2_CTL_DMIC_CLK_SEL_MASK                          0x07

/* WCD939X_CDC_DMIC3_CTL Fields: */
#define WCD939X_CDC_DMIC3_CTL_DMIC_CLK_SCALE_SEL_MASK                    0x70
#define WCD939X_CDC_DMIC3_CTL_DMIC_CLK_EN_MASK                           0x08
#define WCD939X_CDC_DMIC3_CTL_DMIC_CLK_SEL_MASK                          0x07

/* WCD939X_CDC_DMIC4_CTL Fields: */
#define WCD939X_CDC_DMIC4_CTL_DMIC_CLK_SCALE_SEL_MASK                    0x70
#define WCD939X_CDC_DMIC4_CTL_DMIC_CLK_EN_MASK                           0x08
#define WCD939X_CDC_DMIC4_CTL_DMIC_CLK_SEL_MASK                          0x07

/* WCD939X_EFUSE_PRG_CTL Fields: */
#define WCD939X_EFUSE_PRG_CTL_PRG_ADDR_MASK                              0xff

/* WCD939X_EFUSE_CTL Fields: */
#define WCD939X_EFUSE_CTL_EFUSE_ST_CNT_MASK                              0x3c
#define WCD939X_EFUSE_CTL_EFUSE_SOFT_RST_N_MASK                          0x02
#define WCD939X_EFUSE_CTL_EFUSE_EN_MASK                                  0x01

/* WCD939X_CDC_DMIC_RATE_1_2 Fields: */
#define WCD939X_CDC_DMIC_RATE_1_2_DMIC2_RATE_MASK                        0xf0
#define WCD939X_CDC_DMIC_RATE_1_2_DMIC1_RATE_MASK                        0x0f

/* WCD939X_CDC_DMIC_RATE_3_4 Fields: */
#define WCD939X_CDC_DMIC_RATE_3_4_DMIC4_RATE_MASK                        0xf0
#define WCD939X_CDC_DMIC_RATE_3_4_DMIC3_RATE_MASK                        0x0f

/* WCD939X_PDM_WD_CTL0 Fields: */
#define WCD939X_PDM_WD_CTL0_HOLD_OFF_MASK                                0x10
#define WCD939X_PDM_WD_CTL0_TIME_OUT_SEL_MASK                            0x08
#define WCD939X_PDM_WD_CTL0_PDM_WD_EN_MASK                               0x07

/* WCD939X_PDM_WD_CTL1 Fields: */
#define WCD939X_PDM_WD_CTL1_HOLD_OFF_MASK                                0x10
#define WCD939X_PDM_WD_CTL1_TIME_OUT_SEL_MASK                            0x08
#define WCD939X_PDM_WD_CTL1_PDM_WD_EN_MASK                               0x07

/* WCD939X_PDM_WD_CTL2 Fields: */
#define WCD939X_PDM_WD_CTL2_HOLD_OFF_MASK                                0x04
#define WCD939X_PDM_WD_CTL2_TIME_OUT_SEL_MASK                            0x02
#define WCD939X_PDM_WD_CTL2_PDM_WD_EN_MASK                               0x01

/* WCD939X_INTR_MODE Fields: */
#define WCD939X_INTR_MODE_SWR_PULSE_CLR_MASK                             0x20
#define WCD939X_INTR_MODE_SWR_RX_INT_OUT_EN_MASK                         0x10
#define WCD939X_INTR_MODE_SWR_INTR_LEVEL_MASK                            0x02
#define WCD939X_INTR_MODE_INT_POLARITY_MASK                              0x01

/* WCD939X_INTR_MASK_0 Fields: */
#define WCD939X_INTR_MASK_0_HPHL_OCP_INT_MASK                            0x80
#define WCD939X_INTR_MASK_0_HPHR_CNP_INT_MASK                            0x40
#define WCD939X_INTR_MASK_0_HPHR_OCP_INT_MASK                            0x20
#define WCD939X_INTR_MASK_0_MBHC_SW_INT_MASK                             0x10
#define WCD939X_INTR_MASK_0_MBHC_ELECT_INS_REM_LEG_INT_MASK              0x08
#define WCD939X_INTR_MASK_0_MBHC_ELECT_INS_REM_INT_MASK                  0x04
#define WCD939X_INTR_MASK_0_MBHC_BTN_RELEASE_INT_MASK                    0x02
#define WCD939X_INTR_MASK_0_MBHC_BTN_PRESS_INT_MASK                      0x01

/* WCD939X_INTR_MASK_1 Fields: */
#define WCD939X_INTR_MASK_1_EAR_PDM_WD_INT_MASK                          0x80
#define WCD939X_INTR_MASK_1_HPHR_PDM_WD_INT_MASK                         0x40
#define WCD939X_INTR_MASK_1_HPHL_PDM_WD_INT_MASK                         0x20
#define WCD939X_INTR_MASK_1_EAR_SCD_INT_MASK                             0x04
#define WCD939X_INTR_MASK_1_EAR_CNP_INT_MASK                             0x02
#define WCD939X_INTR_MASK_1_HPHL_CNP_INT_MASK                            0x01

/* WCD939X_INTR_MASK_2 Fields: */
#define WCD939X_INTR_MASK_2_HPHL_SURGE_DET_INT_MASK                      0x08
#define WCD939X_INTR_MASK_2_HPHR_SURGE_DET_INT_MASK                      0x04
#define WCD939X_INTR_MASK_2_MBHC_MOISTRUE_INT_MASK                       0x02

/* WCD939X_INTR_STATUS_0 Fields: */
#define WCD939X_INTR_STATUS_0_HPHL_OCP_INT_MASK                          0x80
#define WCD939X_INTR_STATUS_0_HPHR_CNP_INT_MASK                          0x40
#define WCD939X_INTR_STATUS_0_HPHR_OCP_INT_MASK                          0x20
#define WCD939X_INTR_STATUS_0_MBHC_SW_INT_MASK                           0x10
#define WCD939X_INTR_STATUS_0_MBHC_ELECT_INS_REM_LEG_INT_MASK            0x08
#define WCD939X_INTR_STATUS_0_MBHC_ELECT_INS_REM_INT_MASK                0x04
#define WCD939X_INTR_STATUS_0_MBHC_BTN_RELEASE_INT_MASK                  0x02
#define WCD939X_INTR_STATUS_0_MBHC_BTN_PRESS_INT_MASK                    0x01

/* WCD939X_INTR_STATUS_1 Fields: */
#define WCD939X_INTR_STATUS_1_EAR_PDM_WD_INT_MASK                        0x80
#define WCD939X_INTR_STATUS_1_HPHR_PDM_WD_INT_MASK                       0x40
#define WCD939X_INTR_STATUS_1_HPHL_PDM_WD_INT_MASK                       0x20
#define WCD939X_INTR_STATUS_1_EAR_SCD_INT_MASK                           0x04
#define WCD939X_INTR_STATUS_1_EAR_CNP_INT_MASK                           0x02
#define WCD939X_INTR_STATUS_1_HPHL_CNP_INT_MASK                          0x01

/* WCD939X_INTR_STATUS_2 Fields: */
#define WCD939X_INTR_STATUS_2_HPHL_SURGE_DET_INT_MASK                    0x08
#define WCD939X_INTR_STATUS_2_HPHR_SURGE_DET_INT_MASK                    0x04
#define WCD939X_INTR_STATUS_2_MBHC_MOISTRUE_INT_MASK                     0x02

/* WCD939X_INTR_CLEAR_0 Fields: */
#define WCD939X_INTR_CLEAR_0_HPHL_OCP_INT_MASK                           0x80
#define WCD939X_INTR_CLEAR_0_HPHR_CNP_INT_MASK                           0x40
#define WCD939X_INTR_CLEAR_0_HPHR_OCP_INT_MASK                           0x20
#define WCD939X_INTR_CLEAR_0_MBHC_SW_INT_MASK                            0x10
#define WCD939X_INTR_CLEAR_0_MBHC_ELECT_INS_REM_LEG_INT_MASK             0x08
#define WCD939X_INTR_CLEAR_0_MBHC_ELECT_INS_REM_INT_MASK                 0x04
#define WCD939X_INTR_CLEAR_0_MBHC_BTN_RELEASE_INT_MASK                   0x02
#define WCD939X_INTR_CLEAR_0_MBHC_BTN_PRESS_INT_MASK                     0x01

/* WCD939X_INTR_CLEAR_1 Fields: */
#define WCD939X_INTR_CLEAR_1_EAR_PDM_WD_INT_MASK                         0x80
#define WCD939X_INTR_CLEAR_1_HPHR_PDM_WD_INT_MASK                        0x40
#define WCD939X_INTR_CLEAR_1_HPHL_PDM_WD_INT_MASK                        0x20
#define WCD939X_INTR_CLEAR_1_EAR_SCD_INT_MASK                            0x04
#define WCD939X_INTR_CLEAR_1_EAR_CNP_INT_MASK                            0x02
#define WCD939X_INTR_CLEAR_1_HPHL_CNP_INT_MASK                           0x01

/* WCD939X_INTR_CLEAR_2 Fields: */
#define WCD939X_INTR_CLEAR_2_HPHL_SURGE_DET_INT_MASK                     0x08
#define WCD939X_INTR_CLEAR_2_HPHR_SURGE_DET_INT_MASK                     0x04
#define WCD939X_INTR_CLEAR_2_MBHC_MOISTRUE_INT_MASK                      0x02

/* WCD939X_INTR_LEVEL_0 Fields: */
#define WCD939X_INTR_LEVEL_0_HPHL_OCP_INT_MASK                           0x80
#define WCD939X_INTR_LEVEL_0_HPHR_CNP_INT_MASK                           0x40
#define WCD939X_INTR_LEVEL_0_HPHR_OCP_INT_MASK                           0x20
#define WCD939X_INTR_LEVEL_0_MBHC_SW_INT_MASK                            0x10
#define WCD939X_INTR_LEVEL_0_MBHC_ELECT_INS_REM_LEG_INT_MASK             0x08
#define WCD939X_INTR_LEVEL_0_MBHC_ELECT_INS_REM_INT_MASK                 0x04
#define WCD939X_INTR_LEVEL_0_MBHC_BTN_RELEASE_INT_MASK                   0x02
#define WCD939X_INTR_LEVEL_0_MBHC_BTN_PRESS_INT_MASK                     0x01

/* WCD939X_INTR_LEVEL_1 Fields: */
#define WCD939X_INTR_LEVEL_1_EAR_PDM_WD_INT_MASK                         0x80
#define WCD939X_INTR_LEVEL_1_HPHR_PDM_WD_INT_MASK                        0x40
#define WCD939X_INTR_LEVEL_1_HPHL_PDM_WD_INT_MASK                        0x20
#define WCD939X_INTR_LEVEL_1_EAR_SCD_INT_MASK                            0x04
#define WCD939X_INTR_LEVEL_1_EAR_CNP_INT_MASK                            0x02
#define WCD939X_INTR_LEVEL_1_HPHL_CNP_INT_MASK                           0x01

/* WCD939X_INTR_LEVEL_2 Fields: */
#define WCD939X_INTR_LEVEL_2_HPHL_SURGE_DET_INT_MASK                     0x08
#define WCD939X_INTR_LEVEL_2_HPHR_SURGE_DET_INT_MASK                     0x04
#define WCD939X_INTR_LEVEL_2_MBHC_MOISTRUE_INT_MASK                      0x02

/* WCD939X_INTR_SET_0 Fields: */
#define WCD939X_INTR_SET_0_HPHL_OCP_INT_MASK                             0x80
#define WCD939X_INTR_SET_0_HPHR_CNP_INT_MASK                             0x40
#define WCD939X_INTR_SET_0_HPHR_OCP_INT_MASK                             0x20
#define WCD939X_INTR_SET_0_MBHC_SW_INT_MASK                              0x10
#define WCD939X_INTR_SET_0_MBHC_ELECT_INS_REM_LEG_INT_MASK               0x08
#define WCD939X_INTR_SET_0_MBHC_ELECT_INS_REM_INT_MASK                   0x04
#define WCD939X_INTR_SET_0_MBHC_BTN_RELEASE_INT_MASK                     0x02
#define WCD939X_INTR_SET_0_MBHC_BTN_PRESS_INT_MASK                       0x01

/* WCD939X_INTR_SET_1 Fields: */
#define WCD939X_INTR_SET_1_EAR_PDM_WD_INT_MASK                           0x80
#define WCD939X_INTR_SET_1_HPHR_PDM_WD_INT_MASK                          0x40
#define WCD939X_INTR_SET_1_HPHL_PDM_WD_INT_MASK                          0x20
#define WCD939X_INTR_SET_1_EAR_SCD_INT_MASK                              0x04
#define WCD939X_INTR_SET_1_EAR_CNP_INT_MASK                              0x02
#define WCD939X_INTR_SET_1_HPHL_CNP_INT_MASK                             0x01

/* WCD939X_INTR_SET_2 Fields: */
#define WCD939X_INTR_SET_2_HPHL_SURGE_DET_INT_MASK                       0x08
#define WCD939X_INTR_SET_2_HPHR_SURGE_DET_INT_MASK                       0x04
#define WCD939X_INTR_SET_2_MBHC_MOISTRUE_INT_MASK                        0x02

/* WCD939X_INTR_TEST_0 Fields: */
#define WCD939X_INTR_TEST_0_HPHL_OCP_INT_MASK                            0x80
#define WCD939X_INTR_TEST_0_HPHR_CNP_INT_MASK                            0x40
#define WCD939X_INTR_TEST_0_HPHR_OCP_INT_MASK                            0x20
#define WCD939X_INTR_TEST_0_MBHC_SW_INT_MASK                             0x10
#define WCD939X_INTR_TEST_0_MBHC_ELECT_INS_REM_LEG_INT_MASK              0x08
#define WCD939X_INTR_TEST_0_MBHC_ELECT_INS_REM_INT_MASK                  0x04
#define WCD939X_INTR_TEST_0_MBHC_BTN_RELEASE_INT_MASK                    0x02
#define WCD939X_INTR_TEST_0_MBHC_BTN_PRESS_INT_MASK                      0x01

/* WCD939X_INTR_TEST_1 Fields: */
#define WCD939X_INTR_TEST_1_EAR_PDM_WD_INT_MASK                          0x80
#define WCD939X_INTR_TEST_1_HPHR_PDM_WD_INT_MASK                         0x40
#define WCD939X_INTR_TEST_1_HPHL_PDM_WD_INT_MASK                         0x20
#define WCD939X_INTR_TEST_1_EAR_SCD_INT_MASK                             0x04
#define WCD939X_INTR_TEST_1_EAR_CNP_INT_MASK                             0x02
#define WCD939X_INTR_TEST_1_HPHL_CNP_INT_MASK                            0x01

/* WCD939X_INTR_TEST_2 Fields: */
#define WCD939X_INTR_TEST_2_HPHL_SURGE_DET_INT_MASK                      0x08
#define WCD939X_INTR_TEST_2_HPHR_SURGE_DET_INT_MASK                      0x04
#define WCD939X_INTR_TEST_2_MBHC_MOISTRUE_INT_MASK                       0x02

/* WCD939X_TX_MODE_DBG_EN Fields: */
#define WCD939X_TX_MODE_DBG_EN_TXD3_MODE_DBG_EN_MASK                     0x08
#define WCD939X_TX_MODE_DBG_EN_TXD2_MODE_DBG_EN_MASK                     0x04
#define WCD939X_TX_MODE_DBG_EN_TXD1_MODE_DBG_EN_MASK                     0x02
#define WCD939X_TX_MODE_DBG_EN_TXD0_MODE_DBG_EN_MASK                     0x01

/* WCD939X_TX_MODE_DBG_0_1 Fields: */
#define WCD939X_TX_MODE_DBG_0_1_TXD1_MODE_DBG_MASK                       0xf0
#define WCD939X_TX_MODE_DBG_0_1_TXD0_MODE_DBG_MASK                       0x0f

/* WCD939X_TX_MODE_DBG_2_3 Fields: */
#define WCD939X_TX_MODE_DBG_2_3_TXD3_MODE_DBG_MASK                       0xf0
#define WCD939X_TX_MODE_DBG_2_3_TXD2_MODE_DBG_MASK                       0x0f

/* WCD939X_LB_IN_SEL_CTL Fields: */
#define WCD939X_LB_IN_SEL_CTL_EAR_LB_IN_SEL_MASK                         0x0c
#define WCD939X_LB_IN_SEL_CTL_HPH_LB_IN_SEL_MASK                         0x03

/* WCD939X_LOOP_BACK_MODE Fields: */
#define WCD939X_LOOP_BACK_MODE_TX_DATA_EDGE_MASK                         0x10
#define WCD939X_LOOP_BACK_MODE_RX_DATA_EDGE_MASK                         0x08
#define WCD939X_LOOP_BACK_MODE_LOOPBACK_MODE_MASK                        0x07

/* WCD939X_SWR_DAC_TEST Fields: */
#define WCD939X_SWR_DAC_TEST_SWR_DAC_TEST_MASK                           0x01

/* WCD939X_SWR_HM_TEST_RX_0 Fields: */
#define WCD939X_SWR_HM_TEST_RX_0_ALT_MODE_MASK                           0x80
#define WCD939X_SWR_HM_TEST_RX_0_IO_MODE_MASK                            0x40
#define WCD939X_SWR_HM_TEST_RX_0_LN2_T_DATA_OE_MASK                      0x20
#define WCD939X_SWR_HM_TEST_RX_0_LN2_T_DATA_OUT_MASK                     0x10
#define WCD939X_SWR_HM_TEST_RX_0_LN2_T_KEEPER_EN_MASK                    0x08
#define WCD939X_SWR_HM_TEST_RX_0_LN1_T_DATA_OE_MASK                      0x04
#define WCD939X_SWR_HM_TEST_RX_0_LN1_T_DATA_OUT_MASK                     0x02
#define WCD939X_SWR_HM_TEST_RX_0_LN1_T_KEEPER_EN_MASK                    0x01

/* WCD939X_SWR_HM_TEST_TX_0 Fields: */
#define WCD939X_SWR_HM_TEST_TX_0_ALT_MODE_MASK                           0x80
#define WCD939X_SWR_HM_TEST_TX_0_IO_MODE_MASK                            0x40
#define WCD939X_SWR_HM_TEST_TX_0_LN2_T_DATA_OE_MASK                      0x20
#define WCD939X_SWR_HM_TEST_TX_0_LN2_T_DATA_OUT_MASK                     0x10
#define WCD939X_SWR_HM_TEST_TX_0_LN2_T_KEEPER_EN_MASK                    0x08
#define WCD939X_SWR_HM_TEST_TX_0_LN1_T_DATA_OE_MASK                      0x04
#define WCD939X_SWR_HM_TEST_TX_0_LN1_T_DATA_OUT_MASK                     0x02
#define WCD939X_SWR_HM_TEST_TX_0_LN1_T_KEEPER_EN_MASK                    0x01

/* WCD939X_SWR_HM_TEST_RX_1 Fields: */
#define WCD939X_SWR_HM_TEST_RX_1_DTEST_SEL_MASK                          0x1c
#define WCD939X_SWR_HM_TEST_RX_1_LN2_DLY_CELL_TEST_EN_MASK               0x02
#define WCD939X_SWR_HM_TEST_RX_1_LN1_DLY_CELL_TEST_EN_MASK               0x01

/* WCD939X_SWR_HM_TEST_TX_1 Fields: */
#define WCD939X_SWR_HM_TEST_TX_1_DTEST_SEL_MASK                          0x78
#define WCD939X_SWR_HM_TEST_TX_1_LN3_DLY_CELL_TEST_EN_MASK               0x04
#define WCD939X_SWR_HM_TEST_TX_1_LN2_DLY_CELL_TEST_EN_MASK               0x02
#define WCD939X_SWR_HM_TEST_TX_1_LN1_DLY_CELL_TEST_EN_MASK               0x01

/* WCD939X_SWR_HM_TEST_TX_2 Fields: */
#define WCD939X_SWR_HM_TEST_TX_2_LN3_T_DATA_OE_MASK                      0x04
#define WCD939X_SWR_HM_TEST_TX_2_LN3_T_DATA_OUT_MASK                     0x02
#define WCD939X_SWR_HM_TEST_TX_2_LN3_T_KEEPER_EN_MASK                    0x01

/* WCD939X_SWR_HM_TEST_0 Fields: */
#define WCD939X_SWR_HM_TEST_0_TX_LN2_T_DATA_IN_MASK                      0x80
#define WCD939X_SWR_HM_TEST_0_TX_LN2_T_CLK_IN_MASK                       0x40
#define WCD939X_SWR_HM_TEST_0_TX_LN1_T_DATA_IN_MASK                      0x20
#define WCD939X_SWR_HM_TEST_0_TX_LN1_T_CLK_IN_MASK                       0x10
#define WCD939X_SWR_HM_TEST_0_RX_LN2_T_DATA_IN_MASK                      0x08
#define WCD939X_SWR_HM_TEST_0_RX_LN2_T_CLK_IN_MASK                       0x04
#define WCD939X_SWR_HM_TEST_0_RX_LN1_T_DATA_IN_MASK                      0x02
#define WCD939X_SWR_HM_TEST_0_RX_LN1_T_CLK_IN_MASK                       0x01

/* WCD939X_SWR_HM_TEST_1 Fields: */
#define WCD939X_SWR_HM_TEST_1_TX_LN3_T_DATA_IN_MASK                      0x02
#define WCD939X_SWR_HM_TEST_1_TX_LN3_T_CLK_IN_MASK                       0x01

/* WCD939X_PAD_CTL_SWR_0 Fields: */
#define WCD939X_PAD_CTL_SWR_0_SWR_SLEW_PRG_MASK                          0xf0
#define WCD939X_PAD_CTL_SWR_0_SWR_DRIVE_PRG_MASK                         0x0f

/* WCD939X_PAD_CTL_SWR_1 Fields: */
#define WCD939X_PAD_CTL_SWR_1_SWR_TDZ_PRG_MASK                           0x0f

/* WCD939X_I2C_CTL Fields: */
#define WCD939X_I2C_CTL_ACTIVE_MODE_MASK                                 0x01

/* WCD939X_CDC_TX_TANGGU_SW_MODE Fields: */
#define WCD939X_CDC_TX_TANGGU_SW_MODE_LEGACY_SW_MODE_MASK                0x01

/* WCD939X_EFUSE_TEST_CTL_0 Fields: */
#define WCD939X_EFUSE_TEST_CTL_0_EFUSE_TEST_CTL_LSB_MASK                 0xff

/* WCD939X_EFUSE_TEST_CTL_1 Fields: */
#define WCD939X_EFUSE_TEST_CTL_1_EFUSE_TEST_CTL_MSB_MASK                 0xff

/* WCD939X_EFUSE_T_DATA_0 Fields: */
#define WCD939X_EFUSE_T_DATA_0_EFUSE_DATA_MASK                           0xff

/* WCD939X_EFUSE_T_DATA_1 Fields: */
#define WCD939X_EFUSE_T_DATA_1_EFUSE_DATA_MASK                           0xff

/* WCD939X_PAD_CTL_PDM_RX0 Fields: */
#define WCD939X_PAD_CTL_PDM_RX0_PDM_SLEW_PRG_MASK                        0xf0
#define WCD939X_PAD_CTL_PDM_RX0_PDM_DRIVE_PRG_MASK                       0x0f

/* WCD939X_PAD_CTL_PDM_RX1 Fields: */
#define WCD939X_PAD_CTL_PDM_RX1_PDM_SLEW_PRG_MASK                        0xf0
#define WCD939X_PAD_CTL_PDM_RX1_PDM_DRIVE_PRG_MASK                       0x0f

/* WCD939X_PAD_CTL_PDM_TX0 Fields: */
#define WCD939X_PAD_CTL_PDM_TX0_PDM_SLEW_PRG_MASK                        0xf0
#define WCD939X_PAD_CTL_PDM_TX0_PDM_DRIVE_PRG_MASK                       0x0f

/* WCD939X_PAD_CTL_PDM_TX1 Fields: */
#define WCD939X_PAD_CTL_PDM_TX1_PDM_SLEW_PRG_MASK                        0xf0
#define WCD939X_PAD_CTL_PDM_TX1_PDM_DRIVE_PRG_MASK                       0x0f

/* WCD939X_PAD_CTL_PDM_TX2 Fields: */
#define WCD939X_PAD_CTL_PDM_TX2_PDM_SLEW_PRG_MASK                        0xf0
#define WCD939X_PAD_CTL_PDM_TX2_PDM_DRIVE_PRG_MASK                       0x0f

/* WCD939X_PAD_INP_DIS_0 Fields: */
#define WCD939X_PAD_INP_DIS_0_DMIC3_CLK_MASK                             0x20
#define WCD939X_PAD_INP_DIS_0_DMIC3_DATA_MASK                            0x10
#define WCD939X_PAD_INP_DIS_0_DMIC2_CLK_MASK                             0x08
#define WCD939X_PAD_INP_DIS_0_DMIC2_DATA_MASK                            0x04
#define WCD939X_PAD_INP_DIS_0_DMIC1_CLK_MASK                             0x02
#define WCD939X_PAD_INP_DIS_0_DMIC1_DATA_MASK                            0x01

/* WCD939X_PAD_INP_DIS_1 Fields: */
#define WCD939X_PAD_INP_DIS_1_DMIC4_CLK_MASK                             0x10
#define WCD939X_PAD_INP_DIS_1_DMIC4_DATA_MASK                            0x08

/* WCD939X_DRIVE_STRENGTH_0 Fields: */
#define WCD939X_DRIVE_STRENGTH_0_DS_DMIC2_CLK_MASK                       0xc0
#define WCD939X_DRIVE_STRENGTH_0_DS_DMIC2_DATA_MASK                      0x30
#define WCD939X_DRIVE_STRENGTH_0_DS_DMIC1_CLK_MASK                       0x0c
#define WCD939X_DRIVE_STRENGTH_0_DS_DMIC1_DATA_MASK                      0x03

/* WCD939X_DRIVE_STRENGTH_1 Fields: */
#define WCD939X_DRIVE_STRENGTH_1_DS_DMIC3_CLK_MASK                       0x0c
#define WCD939X_DRIVE_STRENGTH_1_DS_DMIC3_DATA_MASK                      0x03

/* WCD939X_DRIVE_STRENGTH_2 Fields: */
#define WCD939X_DRIVE_STRENGTH_2_DS_DMIC4_CLK_MASK                       0xc0
#define WCD939X_DRIVE_STRENGTH_2_DS_DMIC4_DATA_MASK                      0x30

/* WCD939X_RX_DATA_EDGE_CTL Fields: */
#define WCD939X_RX_DATA_EDGE_CTL_HPH_CLH_EDGE_MASK                       0x20
#define WCD939X_RX_DATA_EDGE_CTL_EAR_DOUT_EDGE_MASK                      0x10
#define WCD939X_RX_DATA_EDGE_CTL_HPHR_DOUT_EDGE_MASK                     0x08
#define WCD939X_RX_DATA_EDGE_CTL_HPHL_DOUT_EDGE_MASK                     0x04
#define WCD939X_RX_DATA_EDGE_CTL_HPHR_GAIN_EDGE_MASK                     0x02
#define WCD939X_RX_DATA_EDGE_CTL_HPHL_GAIN_EDGE_MASK                     0x01

/* WCD939X_TX_DATA_EDGE_CTL Fields: */
#define WCD939X_TX_DATA_EDGE_CTL_TX_WE_DLY_MASK                          0xc0
#define WCD939X_TX_DATA_EDGE_CTL_TX3_DIN_EDGE_MASK                       0x08
#define WCD939X_TX_DATA_EDGE_CTL_TX2_DIN_EDGE_MASK                       0x04
#define WCD939X_TX_DATA_EDGE_CTL_TX1_DIN_EDGE_MASK                       0x02
#define WCD939X_TX_DATA_EDGE_CTL_TX0_DIN_EDGE_MASK                       0x01

/* WCD939X_GPIO_MODE Fields: */
#define WCD939X_GPIO_MODE_GPIO_3_EN_MASK                                 0x04
#define WCD939X_GPIO_MODE_GPIO_2_EN_MASK                                 0x02
#define WCD939X_GPIO_MODE_TEST_MODE_MASK                                 0x01

/* WCD939X_PIN_CTL_OE Fields: */
#define WCD939X_PIN_CTL_OE_TEST_PIN_CTL_OE_MASK                          0x10
#define WCD939X_PIN_CTL_OE_GPIO_3_PIN_CTL_OE_MASK                        0x08
#define WCD939X_PIN_CTL_OE_GPIO_2_PIN_CTL_OE_MASK                        0x04

/* WCD939X_PIN_CTL_DATA_0 Fields: */
#define WCD939X_PIN_CTL_DATA_0_PAD_DMIC3_CLK_MASK                        0x20
#define WCD939X_PIN_CTL_DATA_0_PAD_DMIC3_DATA_MASK                       0x10
#define WCD939X_PIN_CTL_DATA_0_PAD_DMIC2_CLK_MASK                        0x08
#define WCD939X_PIN_CTL_DATA_0_PAD_DMIC2_DATA_MASK                       0x04
#define WCD939X_PIN_CTL_DATA_0_PAD_DMIC1_CLK_MASK                        0x02
#define WCD939X_PIN_CTL_DATA_0_PAD_DMIC1_DATA_MASK                       0x01

/* WCD939X_PIN_CTL_DATA_1 Fields: */
#define WCD939X_PIN_CTL_DATA_1_PAD_DMIC4_CLK_MASK                        0x08
#define WCD939X_PIN_CTL_DATA_1_PAD_DMIC4_DATA_MASK                       0x04

/* WCD939X_PIN_STATUS_0 Fields: */
#define WCD939X_PIN_STATUS_0_PAD_DMIC3_CLK_MASK                          0x20
#define WCD939X_PIN_STATUS_0_PAD_DMIC3_DATA_MASK                         0x10
#define WCD939X_PIN_STATUS_0_PAD_DMIC2_CLK_MASK                          0x08
#define WCD939X_PIN_STATUS_0_PAD_DMIC2_DATA_MASK                         0x04
#define WCD939X_PIN_STATUS_0_PAD_DMIC1_CLK_MASK                          0x02
#define WCD939X_PIN_STATUS_0_PAD_DMIC1_DATA_MASK                         0x01

/* WCD939X_PIN_STATUS_1 Fields: */
#define WCD939X_PIN_STATUS_1_PAD_DMIC4_CLK_MASK                          0x08
#define WCD939X_PIN_STATUS_1_PAD_DMIC4_DATA_MASK                         0x04

/* WCD939X_DIG_DEBUG_CTL Fields: */
#define WCD939X_DIG_DEBUG_CTL_DIG_DEBUG_CTL_MASK                         0x7f

/* WCD939X_DIG_DEBUG_EN Fields: */
#define WCD939X_DIG_DEBUG_EN_TX_DBG_MODE_MASK                            0x04
#define WCD939X_DIG_DEBUG_EN_RX_DBG_MODE_1_MASK                          0x02
#define WCD939X_DIG_DEBUG_EN_RX_DBG_MODE_0_MASK                          0x01

/* WCD939X_ANA_CSR_DBG_ADD Fields: */
#define WCD939X_ANA_CSR_DBG_ADD_ADD_MASK                                 0xff

/* WCD939X_ANA_CSR_DBG_CTL Fields: */
#define WCD939X_ANA_CSR_DBG_CTL_WR_VALUE_MASK                            0xc0
#define WCD939X_ANA_CSR_DBG_CTL_RD_VALUE_MASK                            0x38
#define WCD939X_ANA_CSR_DBG_CTL_DBG_PAGE_SEL_MASK                        0x06
#define WCD939X_ANA_CSR_DBG_CTL_DBG_EN_MASK                              0x01

/* WCD939X_SSP_DBG Fields: */
#define WCD939X_SSP_DBG_RX_SSP_DBG_MASK                                  0x02
#define WCD939X_SSP_DBG_TX_SSP_DBG_MASK                                  0x01

/* WCD939X_MODE_STATUS_0 Fields: */
#define WCD939X_MODE_STATUS_0_ATE_7_MASK                                 0x80
#define WCD939X_MODE_STATUS_0_ATE_6_MASK                                 0x40
#define WCD939X_MODE_STATUS_0_ATE_5_MASK                                 0x20
#define WCD939X_MODE_STATUS_0_ATE_4_MASK                                 0x10
#define WCD939X_MODE_STATUS_0_ATE_3_MASK                                 0x08
#define WCD939X_MODE_STATUS_0_ATE_2_MASK                                 0x04
#define WCD939X_MODE_STATUS_0_ATE_1_MASK                                 0x02
#define WCD939X_MODE_STATUS_0_SWR_TEST_MASK                              0x01

/* WCD939X_MODE_STATUS_1 Fields: */
#define WCD939X_MODE_STATUS_1_SWR_PAD_TEST_MASK                          0x02
#define WCD939X_MODE_STATUS_1_EFUSE_MODE_MASK                            0x01

/* WCD939X_SPARE_0 Fields: */
#define WCD939X_SPARE_0_SPARE_REG_0_MASK                                 0xff

/* WCD939X_SPARE_1 Fields: */
#define WCD939X_SPARE_1_SPARE_REG_1_MASK                                 0xff

/* WCD939X_SPARE_2 Fields: */
#define WCD939X_SPARE_2_SPARE_REG_2_MASK                                 0xff

/* WCD939X_EFUSE_REG_0 Fields: */
#define WCD939X_EFUSE_REG_0_SPARE_BITS_MASK                              0xe0
#define WCD939X_EFUSE_REG_0_WCD939X_ID_MASK                              0x1e
#define WCD939X_EFUSE_REG_0_EFUSE_BLOWN_MASK                             0x01

/* WCD939X_EFUSE_REG_1 Fields: */
#define WCD939X_EFUSE_REG_1_LOT_ID_0_MASK                                0xff

/* WCD939X_EFUSE_REG_2 Fields: */
#define WCD939X_EFUSE_REG_2_LOT_ID_1_MASK                                0xff

/* WCD939X_EFUSE_REG_3 Fields: */
#define WCD939X_EFUSE_REG_3_LOT_ID_2_MASK                                0xff

/* WCD939X_EFUSE_REG_4 Fields: */
#define WCD939X_EFUSE_REG_4_LOT_ID_3_MASK                                0xff

/* WCD939X_EFUSE_REG_5 Fields: */
#define WCD939X_EFUSE_REG_5_LOT_ID_4_MASK                                0xff

/* WCD939X_EFUSE_REG_6 Fields: */
#define WCD939X_EFUSE_REG_6_LOT_ID_5_MASK                                0xff

/* WCD939X_EFUSE_REG_7 Fields: */
#define WCD939X_EFUSE_REG_7_LOT_ID_6_MASK                                0xff

/* WCD939X_EFUSE_REG_8 Fields: */
#define WCD939X_EFUSE_REG_8_LOT_ID_7_MASK                                0xff

/* WCD939X_EFUSE_REG_9 Fields: */
#define WCD939X_EFUSE_REG_9_LOT_ID_8_MASK                                0xff

/* WCD939X_EFUSE_REG_10 Fields: */
#define WCD939X_EFUSE_REG_10_LOT_ID_9_MASK                               0xff

/* WCD939X_EFUSE_REG_11 Fields: */
#define WCD939X_EFUSE_REG_11_LOT_ID_10_MASK                              0xff

/* WCD939X_EFUSE_REG_12 Fields: */
#define WCD939X_EFUSE_REG_12_LOT_ID_11_MASK                              0xff

/* WCD939X_EFUSE_REG_13 Fields: */
#define WCD939X_EFUSE_REG_13_WAFER_ID_MASK                               0xff

/* WCD939X_EFUSE_REG_14 Fields: */
#define WCD939X_EFUSE_REG_14_X_DIE_LOCATION_MASK                         0xff

/* WCD939X_EFUSE_REG_15 Fields: */
#define WCD939X_EFUSE_REG_15_Y_DIE_LOCATION_MASK                         0xff

/* WCD939X_EFUSE_REG_16 Fields: */
#define WCD939X_EFUSE_REG_16_FAB_ID_MASK                                 0xff

/* WCD939X_EFUSE_REG_17 Fields: */
#define WCD939X_EFUSE_REG_17_TEST_PROGRAM_REV_MASK                       0xff

/* WCD939X_EFUSE_REG_18 Fields: */
#define WCD939X_EFUSE_REG_18_DIE_REVISION_MASK                           0xff

/* WCD939X_EFUSE_REG_19 Fields: */
#define WCD939X_EFUSE_REG_19_MFG_ID_SPARE_MASK                           0xff

/* WCD939X_EFUSE_REG_20 Fields: */
#define WCD939X_EFUSE_REG_20_I2C_SLV_ID_BLOWN_MASK                       0x80
#define WCD939X_EFUSE_REG_20_I2C_SLAVE_ID_MASK                           0x7f

/* WCD939X_EFUSE_REG_21 Fields: */
#define WCD939X_EFUSE_REG_21_MBHC_IMP_DET_0_MASK                         0xff

/* WCD939X_EFUSE_REG_22 Fields: */
#define WCD939X_EFUSE_REG_22_MBHC_IMP_DET_1_MASK                         0xff

/* WCD939X_EFUSE_REG_23 Fields: */
#define WCD939X_EFUSE_REG_23_SWR_PAD_DRIVE_PRG_1P2V_MASK                 0xf0
#define WCD939X_EFUSE_REG_23_SWR_SLEW_PRG_1P2V_MASK                      0x0f

/* WCD939X_EFUSE_REG_24 Fields: */
#define WCD939X_EFUSE_REG_24_SPARE_BITS_MASK                             0xe0
#define WCD939X_EFUSE_REG_24_SWR_PAD_BLOWN_MASK                          0x10
#define WCD939X_EFUSE_REG_24_SWR_TDZ_DELAY_PRG_1P2V_MASK                 0x0f

/* WCD939X_EFUSE_REG_25 Fields: */
#define WCD939X_EFUSE_REG_25_MBHC_IMP_DET_2_MASK                         0xff

/* WCD939X_EFUSE_REG_26 Fields: */
#define WCD939X_EFUSE_REG_26_MBHC_IMP_DET_3_MASK                         0xff

/* WCD939X_EFUSE_REG_27 Fields: */
#define WCD939X_EFUSE_REG_27_HPH_DSD_DIS_MASK                            0x80
#define WCD939X_EFUSE_REG_27_BG_TUNE_BLOWN_MASK                          0x40
#define WCD939X_EFUSE_REG_27_BG_TUNE_MASK                                0x30
#define WCD939X_EFUSE_REG_27_EFUSE_HPH_MASK                              0x0f

/* WCD939X_EFUSE_REG_28 Fields: */
#define WCD939X_EFUSE_REG_28_HPH_CLH_DIS_MASK                            0x80
#define WCD939X_EFUSE_REG_28_HPH_LOHIFI_DIS_MASK                         0x40
#define WCD939X_EFUSE_REG_28_HPH_HIFI_DIS_MASK                           0x20
#define WCD939X_EFUSE_REG_28_EAR_CLH_DIS_MASK                            0x10
#define WCD939X_EFUSE_REG_28_DMIC_DIS_MASK                               0x08
#define WCD939X_EFUSE_REG_28_TX_LP_DIS_MASK                              0x04
#define WCD939X_EFUSE_REG_28_TX_HP_DIS_MASK                              0x02
#define WCD939X_EFUSE_REG_28_SPARE_BITS_MASK                             0x01

/* WCD939X_EFUSE_REG_29 Fields: */
#define WCD939X_EFUSE_REG_29_TX_ULP1_DIS_MASK                            0x80
#define WCD939X_EFUSE_REG_29_TX_ULP2_DIS_MASK                            0x40
#define WCD939X_EFUSE_REG_29_SPARE_BITS_MASK                             0x30
#define WCD939X_EFUSE_REG_29_SWR_PAD_DRIVE_PRG_1P8V_MASK                 0x0f

/* WCD939X_EFUSE_REG_30 Fields: */
#define WCD939X_EFUSE_REG_30_SWR_SLEW_PRG_1P8V_MASK                      0xf0
#define WCD939X_EFUSE_REG_30_SWR_TDZ_DELAY_PRG_1P8V_MASK                 0x0f

/* WCD939X_EFUSE_REG_31 Fields: */
#define WCD939X_EFUSE_REG_31_SPARE_EFUSE_ANA_MASK                        0xff

/* WCD939X_TX_REQ_FB_CTL_0 Fields: */
#define WCD939X_TX_REQ_FB_CTL_0_ULP2_FB_T2_MASK                          0xf0
#define WCD939X_TX_REQ_FB_CTL_0_ULP2_FB_T1_MASK                          0x0f

/* WCD939X_TX_REQ_FB_CTL_1 Fields: */
#define WCD939X_TX_REQ_FB_CTL_1_ULP1_FB_T2_MASK                          0xf0
#define WCD939X_TX_REQ_FB_CTL_1_ULP1_FB_T1_MASK                          0x0f

/* WCD939X_TX_REQ_FB_CTL_2 Fields: */
#define WCD939X_TX_REQ_FB_CTL_2_L0_FB_T2_MASK                            0xf0
#define WCD939X_TX_REQ_FB_CTL_2_L0_FB_T1_MASK                            0x0f

/* WCD939X_TX_REQ_FB_CTL_3 Fields: */
#define WCD939X_TX_REQ_FB_CTL_3_L1_FB_T2_MASK                            0xf0
#define WCD939X_TX_REQ_FB_CTL_3_L1_FB_T1_MASK                            0x0f

/* WCD939X_TX_REQ_FB_CTL_4 Fields: */
#define WCD939X_TX_REQ_FB_CTL_4_L2_FB_T2_MASK                            0xf0
#define WCD939X_TX_REQ_FB_CTL_4_L2_FB_T1_MASK                            0x0f

/* WCD939X_DEM_BYPASS_DATA0 Fields: */
#define WCD939X_DEM_BYPASS_DATA0_DEM_BYPASS_DATA0_MASK                   0xff

/* WCD939X_DEM_BYPASS_DATA1 Fields: */
#define WCD939X_DEM_BYPASS_DATA1_DEM_BYPASS_DATA1_MASK                   0xff

/* WCD939X_DEM_BYPASS_DATA2 Fields: */
#define WCD939X_DEM_BYPASS_DATA2_DEM_BYPASS_DATA2_MASK                   0xff

/* WCD939X_DEM_BYPASS_DATA3 Fields: */
#define WCD939X_DEM_BYPASS_DATA3_DEM_BYPASS_DATA3_MASK                   0x0f

/* WCD939X_DEM_SECOND_ORDER Fields: */
#define WCD939X_DEM_SECOND_ORDER_DEM_1_2ND_ORDER_EN_MASK                 0x02
#define WCD939X_DEM_SECOND_ORDER_DEM_0_2ND_ORDER_EN_MASK                 0x01

/* WCD939X_DSM_CTRL Fields: */
#define WCD939X_DSM_CTRL_DSM_1_STATIC_EN_MASK                            0x02
#define WCD939X_DSM_CTRL_DSM_0_STATIC_EN_MASK                            0x01

/* WCD939X_DSM_0_STATIC_DATA_0 Fields: */
#define WCD939X_DSM_0_STATIC_DATA_0_DSM_0_STATIC_DATA0_MASK              0xff

/* WCD939X_DSM_0_STATIC_DATA_1 Fields: */
#define WCD939X_DSM_0_STATIC_DATA_1_DSM_0_STATIC_DATA1_MASK              0xff

/* WCD939X_DSM_0_STATIC_DATA_2 Fields: */
#define WCD939X_DSM_0_STATIC_DATA_2_DSM_0_STATIC_DATA2_MASK              0xff

/* WCD939X_DSM_0_STATIC_DATA_3 Fields: */
#define WCD939X_DSM_0_STATIC_DATA_3_DSM_0_STATIC_DATA3_MASK              0x07

/* WCD939X_DSM_1_STATIC_DATA_0 Fields: */
#define WCD939X_DSM_1_STATIC_DATA_0_DSM_1_STATIC_DATA0_MASK              0xff

/* WCD939X_DSM_1_STATIC_DATA_1 Fields: */
#define WCD939X_DSM_1_STATIC_DATA_1_DSM_1_STATIC_DATA1_MASK              0xff

/* WCD939X_DSM_1_STATIC_DATA_2 Fields: */
#define WCD939X_DSM_1_STATIC_DATA_2_DSM_1_STATIC_DATA2_MASK              0xff

/* WCD939X_DSM_1_STATIC_DATA_3 Fields: */
#define WCD939X_DSM_1_STATIC_DATA_3_DSM_1_STATIC_DATA3_MASK              0x07


/* WCD939X_RX_PAGE Fields: */
#define WCD939X_RX_PAGE_PAG_REG_MASK                                     0xff

/* WCD939X_TOP_CFG0 Fields: */
#define WCD939X_TOP_CFG0_HPH_DAC_RATE_SEL_MASK                           0x02
#define WCD939X_TOP_CFG0_PGA_UPDATE_MASK                                 0x01

/* WCD939X_HPHL_COMP_WR_LSB Fields: */
#define WCD939X_HPHL_COMP_WR_LSB_COEFF_MASK                              0xff

/* WCD939X_HPHL_COMP_WR_MSB Fields: */
#define WCD939X_HPHL_COMP_WR_MSB_COEFF_MASK                              0x1f

/* WCD939X_HPHL_COMP_LUT Fields: */
#define WCD939X_HPHL_COMP_LUT_BYPASS_MASK                                0x80
#define WCD939X_HPHL_COMP_LUT_MANUAL_RD_MASK                             0x40
#define WCD939X_HPHL_COMP_LUT_MANUAL_WR_MASK                             0x20
#define WCD939X_HPHL_COMP_LUT_ADDR_MASK                                  0x1f

/* WCD939X_HPHL_COMP_RD_LSB Fields: */
#define WCD939X_HPHL_COMP_RD_LSB_COEFF_MASK                              0xff

/* WCD939X_HPHL_COMP_RD_MSB Fields: */
#define WCD939X_HPHL_COMP_RD_MSB_COEFF_MASK                              0x1f

/* WCD939X_HPHR_COMP_WR_LSB Fields: */
#define WCD939X_HPHR_COMP_WR_LSB_COEFF_MASK                              0xff

/* WCD939X_HPHR_COMP_WR_MSB Fields: */
#define WCD939X_HPHR_COMP_WR_MSB_COEFF_MASK                              0x1f

/* WCD939X_HPHR_COMP_LUT Fields: */
#define WCD939X_HPHR_COMP_LUT_BYPASS_MASK                                0x80
#define WCD939X_HPHR_COMP_LUT_MANUAL_RD_MASK                             0x40
#define WCD939X_HPHR_COMP_LUT_MANUAL_WR_MASK                             0x20
#define WCD939X_HPHR_COMP_LUT_ADDR_MASK                                  0x1f

/* WCD939X_HPHR_COMP_RD_LSB Fields: */
#define WCD939X_HPHR_COMP_RD_LSB_COEFF_MASK                              0xff

/* WCD939X_HPHR_COMP_RD_MSB Fields: */
#define WCD939X_HPHR_COMP_RD_MSB_COEFF_MASK                              0x1f

/* WCD939X_DSD0_DEBUG_CFG1 Fields: */
#define WCD939X_DSD0_DEBUG_CFG1_DSD_UNPACKING_ORDER_MASK                 0x08
#define WCD939X_DSD0_DEBUG_CFG1_DSD_DC_DET_EN_MASK                       0x04
#define WCD939X_DSD0_DEBUG_CFG1_DSD_MUTE_DET_EN_MASK                     0x01

/* WCD939X_DSD0_DEBUG_CFG2 Fields: */
#define WCD939X_DSD0_DEBUG_CFG2_MUTE_INI_VAL_MASK                        0x10
#define WCD939X_DSD0_DEBUG_CFG2_DC_INTR_THRESHOLD_MASK                   0x0c
#define WCD939X_DSD0_DEBUG_CFG2_DC_DET_THRESHOLD_MASK                    0x03

/* WCD939X_DSD0_DEBUG_CFG3 Fields: */
#define WCD939X_DSD0_DEBUG_CFG3_DSD_POST_GAIN_MASK                       0x38
#define WCD939X_DSD0_DEBUG_CFG3_DSD_GAIN_ADJ_MASK                        0x07

/* WCD939X_DSD0_DEBUG_CFG4 Fields: */
#define WCD939X_DSD0_DEBUG_CFG4_DSD_INPUT_ZOH_MASK                       0x03

/* WCD939X_DSD0_DEBUG_CFG5 Fields: */
#define WCD939X_DSD0_DEBUG_CFG5_DSD_DC_DET_MASK                          0x80
#define WCD939X_DSD0_DEBUG_CFG5_DSD_PGA_GAIN_UPD_STATUS_MASK             0x40
#define WCD939X_DSD0_DEBUG_CFG5_DSD_DC_SAMPLE_NUM_MSB_MASK               0x03

/* WCD939X_DSD0_DEBUG_CFG6 Fields: */
#define WCD939X_DSD0_DEBUG_CFG6_DSD_DC_SAMPLE_NUM_LSB_MASK               0xff

/* WCD939X_DSD1_DEBUG_CFG1 Fields: */
#define WCD939X_DSD1_DEBUG_CFG1_DSD_UNPACKING_ORDER_MASK                 0x04
#define WCD939X_DSD1_DEBUG_CFG1_DSD_DC_DET_EN_MASK                       0x02
#define WCD939X_DSD1_DEBUG_CFG1_DSD_MUTE_DET_EN_MASK                     0x01

/* WCD939X_DSD1_DEBUG_CFG2 Fields: */
#define WCD939X_DSD1_DEBUG_CFG2_MUTE_INI_VAL_MASK                        0x10
#define WCD939X_DSD1_DEBUG_CFG2_DC_INTR_THRESHOLD_MASK                   0x0c
#define WCD939X_DSD1_DEBUG_CFG2_DC_DET_THRESHOLD_MASK                    0x03

/* WCD939X_DSD1_DEBUG_CFG3 Fields: */
#define WCD939X_DSD1_DEBUG_CFG3_DSD_POST_GAIN_MASK                       0x38
#define WCD939X_DSD1_DEBUG_CFG3_DSD_GAIN_ADJ_MASK                        0x07

/* WCD939X_DSD1_DEBUG_CFG4 Fields: */
#define WCD939X_DSD1_DEBUG_CFG4_DSD_INPUT_ZOH_MASK                       0x03

/* WCD939X_DSD1_DEBUG_CFG5 Fields: */
#define WCD939X_DSD1_DEBUG_CFG5_DSD_DC_DET_MASK                          0x80
#define WCD939X_DSD1_DEBUG_CFG5_DSD_PGA_GAIN_UPD_STATUS_MASK             0x40
#define WCD939X_DSD1_DEBUG_CFG5_DSD_DC_SAMPLE_NUM_MSB_MASK               0x03

/* WCD939X_DSD1_DEBUG_CFG6 Fields: */
#define WCD939X_DSD1_DEBUG_CFG6_DSD_DC_SAMPLE_NUM_LSB_MASK               0xff

/* WCD939X_HPHL_RX_PATH_CFG0 Fields: */
#define WCD939X_HPHL_RX_PATH_CFG0_INT_EN_MASK                            0x02
#define WCD939X_HPHL_RX_PATH_CFG0_DLY_ZN_EN_MASK                         0x01

/* WCD939X_HPHL_RX_PATH_CFG1 Fields: */
#define WCD939X_HPHL_RX_PATH_CFG1_DSM_SOFT_RST_MASK                      0x20
#define WCD939X_HPHL_RX_PATH_CFG1_INT_SOFT_RST_MASK                      0x10
#define WCD939X_HPHL_RX_PATH_CFG1_FMT_CONV_MASK                          0x08
#define WCD939X_HPHL_RX_PATH_CFG1_IDLE_OVRD_EN_MASK                      0x04
#define WCD939X_HPHL_RX_PATH_CFG1_RX_DC_DROOP_COEFF_SEL_MASK             0x03

/* WCD939X_HPHR_RX_PATH_CFG0 Fields: */
#define WCD939X_HPHR_RX_PATH_CFG0_INT_EN_MASK                            0x04
#define WCD939X_HPHR_RX_PATH_CFG0_DLY_ZN_EN_MASK                         0x02

/* WCD939X_HPHR_RX_PATH_CFG1 Fields: */
#define WCD939X_HPHR_RX_PATH_CFG1_DSM_SOFT_RST_MASK                      0x20
#define WCD939X_HPHR_RX_PATH_CFG1_INT_SOFT_RST_MASK                      0x10
#define WCD939X_HPHR_RX_PATH_CFG1_FMT_CONV_MASK                          0x08
#define WCD939X_HPHR_RX_PATH_CFG1_IDLE_OVRD_EN_MASK                      0x04
#define WCD939X_HPHR_RX_PATH_CFG1_RX_DC_DROOP_COEFF_SEL_MASK             0x03

/* WCD939X_RX_PATH_CFG2 Fields: */
#define WCD939X_RX_PATH_CFG2_COMP_XTALK_EN_MASK                          0x08
#define WCD939X_RX_PATH_CFG2_XTALK_NLIN_EN_MASK                          0x04
#define WCD939X_RX_PATH_CFG2_XTALK_LIN_EN_MASK                           0x02
#define WCD939X_RX_PATH_CFG2_XTALK_EN_MASK                               0x01

/* WCD939X_HPHL_RX_PATH_SEC0 Fields: */
#define WCD939X_HPHL_RX_PATH_SEC0_LIN_XTALK_POLARITY_MASK                0x20
#define WCD939X_HPHL_RX_PATH_SEC0_LIN_XTALK_SCALE_MASK                   0x1f

/* WCD939X_HPHL_RX_PATH_SEC1 Fields: */
#define WCD939X_HPHL_RX_PATH_SEC1_LIN_XTALK_ALPHA_MASK                   0xff

/* WCD939X_HPHL_RX_PATH_SEC2 Fields: */
#define WCD939X_HPHL_RX_PATH_SEC2_NLIN_XTALK_POLARITY_MASK               0x40
#define WCD939X_HPHL_RX_PATH_SEC2_NLIN_XTALK_BYPASS_MASK                 0x20
#define WCD939X_HPHL_RX_PATH_SEC2_NLIN_XTALK_SCALE_MASK                  0x1f

/* WCD939X_HPHL_RX_PATH_SEC3 Fields: */
#define WCD939X_HPHL_RX_PATH_SEC3_NLIN_XTALK_ALPHA_MASK                  0xff

/* WCD939X_HPHR_RX_PATH_SEC0 Fields: */
#define WCD939X_HPHR_RX_PATH_SEC0_LIN_XTALK_POLARITY_MASK                0x20
#define WCD939X_HPHR_RX_PATH_SEC0_LIN_XTALK_SCALE_MASK                   0x1f

/* WCD939X_HPHR_RX_PATH_SEC1 Fields: */
#define WCD939X_HPHR_RX_PATH_SEC1_LIN_XTALK_ALPHA_MASK                   0xff

/* WCD939X_HPHR_RX_PATH_SEC2 Fields: */
#define WCD939X_HPHR_RX_PATH_SEC2_NLIN_XTALK_POLARITY_MASK               0x40
#define WCD939X_HPHR_RX_PATH_SEC2_NLIN_XTALK_BYPASS_MASK                 0x20
#define WCD939X_HPHR_RX_PATH_SEC2_NLIN_XTALK_SCALE_MASK                  0x1f

/* WCD939X_HPHR_RX_PATH_SEC3 Fields: */
#define WCD939X_HPHR_RX_PATH_SEC3_NLIN_XTALK_ALPHA_MASK                  0xff

/* WCD939X_RX_PATH_SEC4 Fields: */
#define WCD939X_RX_PATH_SEC4_NLIN_CMB_POLARITY_MASK                      0x20
#define WCD939X_RX_PATH_SEC4_NLIN_CMB_SCALE_MASK                         0x1f

/* WCD939X_RX_PATH_SEC5 Fields: */
#define WCD939X_RX_PATH_SEC5_NLIN_CMB_ALPHA_MASK                         0xff


/* WCD939X_CTL0 Fields: */
#define WCD939X_CTL0_SHUTDWN_TOUT_MASK                                   0x70
#define WCD939X_CTL0_DROPOUT_EN_MASK                                     0x08
#define WCD939X_CTL0_COMP_HALT_MASK                                      0x04
#define WCD939X_CTL0_SOFT_RST_MASK                                       0x02
#define WCD939X_CTL0_CLK_EN_MASK                                         0x01

/* WCD939X_CTL1 Fields: */
#define WCD939X_CTL1_LEVEL_METER_DIV_FACTOR_MASK                         0xf0
#define WCD939X_CTL1_PEAK_METER_TOUT_MASK                                0x0f

/* WCD939X_CTL2 Fields: */
#define WCD939X_CTL2_LEVEL_METER_RESAMPLE_RATE_MASK                      0xff

/* WCD939X_CTL3 Fields: */
#define WCD939X_CTL3_STATIC_GAIN_OFFSET_MASK                             0x80
#define WCD939X_CTL3_ZONE_SELECT_SHIFT_MASK                              0x70
#define WCD939X_CTL3_ZONE_SELECT_ENTRY_MASK                              0x0f

/* WCD939X_CTL4 Fields: */
#define WCD939X_CTL4_DET_WINDOW_MASK                                     0xff

/* WCD939X_CTL5 Fields: */
#define WCD939X_CTL5_GAIN_MAX_THOLD_MASK                                 0x18
#define WCD939X_CTL5_DET_WINDOW_MASK                                     0x07

/* WCD939X_CTL6 Fields: */
#define WCD939X_CTL6_STATUS_MASK                                         0x01

/* WCD939X_CTL7 Fields: */
#define WCD939X_CTL7_DIS_SCD_MASK                                        0x40
#define WCD939X_CTL7_AGAIN_DELAY_MASK                                    0x1e

/* WCD939X_CTL8 Fields: */
#define WCD939X_CTL8_PEAK_TO_FLAG_DIS_MASK                               0x02
#define WCD939X_CTL8_GAIN_STEP_SELECT_MASK                               0x01

/* WCD939X_CTL9 Fields: */
#define WCD939X_CTL9_ZONE0_RMS_MASK                                      0x7f

/* WCD939X_CTL10 Fields: */
#define WCD939X_CTL10_ZONE1_RMS_MASK                                     0x7f

/* WCD939X_CTL11 Fields: */
#define WCD939X_CTL11_ZONE2_RMS_MASK                                     0x7f

/* WCD939X_CTL12 Fields: */
#define WCD939X_CTL12_ZONE3_RMS_MASK                                     0x7f

/* WCD939X_CTL13 Fields: */
#define WCD939X_CTL13_ZONE4_RMS_MASK                                     0x7f

/* WCD939X_CTL14 Fields: */
#define WCD939X_CTL14_ZONE5_RMS_MASK                                     0x7f

/* WCD939X_CTL15 Fields: */
#define WCD939X_CTL15_ZONE6_RMS_MASK                                     0x7f

/* WCD939X_CTL16 Fields: */
#define WCD939X_CTL16_MAX_ATTN_MASK                                      0xff

/* WCD939X_CTL17 Fields: */
#define WCD939X_CTL17_PATH_GAIN_MASK                                     0x3f

/* WCD939X_CTL18 Fields: */
#define WCD939X_CTL18_ANA_ADDR_MAP_MASK                                  0x3f

/* WCD939X_CTL19 Fields: */
#define WCD939X_CTL19_RMS_TOUT_MASK                                      0x3e
#define WCD939X_CTL19_RMS_TOUT_OVERRIDE_MASK                             0x01


/* WCD939X_R_CTL0 Fields: */
#define WCD939X_R_CTL0_SHUTDWN_TOUT_MASK                                 0x70
#define WCD939X_R_CTL0_DROPOUT_EN_MASK                                   0x08
#define WCD939X_R_CTL0_COMP_HALT_MASK                                    0x04
#define WCD939X_R_CTL0_SOFT_RST_MASK                                     0x02
#define WCD939X_R_CTL0_CLK_EN_MASK                                       0x01

/* WCD939X_R_CTL1 Fields: */
#define WCD939X_R_CTL1_LEVEL_METER_DIV_FACTOR_MASK                       0xf0
#define WCD939X_R_CTL1_PEAK_METER_TOUT_MASK                              0x0f

/* WCD939X_R_CTL2 Fields: */
#define WCD939X_R_CTL2_LEVEL_METER_RESAMPLE_RATE_MASK                    0xff

/* WCD939X_R_CTL3 Fields: */
#define WCD939X_R_CTL3_STATIC_GAIN_OFFSET_MASK                           0x80
#define WCD939X_R_CTL3_ZONE_SELECT_SHIFT_MASK                            0x70
#define WCD939X_R_CTL3_ZONE_SELECT_ENTRY_MASK                            0x0f

/* WCD939X_R_CTL4 Fields: */
#define WCD939X_R_CTL4_DET_WINDOW_MASK                                   0xff

/* WCD939X_R_CTL5 Fields: */
#define WCD939X_R_CTL5_GAIN_MAX_THOLD_MASK                               0x18
#define WCD939X_R_CTL5_DET_WINDOW_MASK                                   0x07

/* WCD939X_R_CTL6 Fields: */
#define WCD939X_R_CTL6_STATUS_MASK                                       0x01

/* WCD939X_R_CTL7 Fields: */
#define WCD939X_R_CTL7_DIS_SCD_MASK                                      0x40
#define WCD939X_R_CTL7_AGAIN_DELAY_MASK                                  0x1e

/* WCD939X_R_CTL8 Fields: */
#define WCD939X_R_CTL8_PEAK_TO_FLAG_DIS_MASK                             0x02
#define WCD939X_R_CTL8_GAIN_STEP_SELECT_MASK                             0x01

/* WCD939X_R_CTL9 Fields: */
#define WCD939X_R_CTL9_ZONE0_RMS_MASK                                    0x7f

/* WCD939X_R_CTL10 Fields: */
#define WCD939X_R_CTL10_ZONE1_RMS_MASK                                   0x7f

/* WCD939X_R_CTL11 Fields: */
#define WCD939X_R_CTL11_ZONE2_RMS_MASK                                   0x7f

/* WCD939X_R_CTL12 Fields: */
#define WCD939X_R_CTL12_ZONE3_RMS_MASK                                   0x7f

/* WCD939X_R_CTL13 Fields: */
#define WCD939X_R_CTL13_ZONE4_RMS_MASK                                   0x7f

/* WCD939X_R_CTL14 Fields: */
#define WCD939X_R_CTL14_ZONE5_RMS_MASK                                   0x7f

/* WCD939X_R_CTL15 Fields: */
#define WCD939X_R_CTL15_ZONE6_RMS_MASK                                   0x7f

/* WCD939X_R_CTL16 Fields: */
#define WCD939X_R_CTL16_MAX_ATTN_MASK                                    0xff

/* WCD939X_R_CTL17 Fields: */
#define WCD939X_R_CTL17_PATH_GAIN_MASK                                   0x3f

/* WCD939X_R_CTL18 Fields: */
#define WCD939X_R_CTL18_ANA_ADDR_MAP_MASK                                0x3f

/* WCD939X_R_CTL19 Fields: */
#define WCD939X_R_CTL19_RMS_TOUT_MASK                                    0x3e
#define WCD939X_R_CTL19_RMS_TOUT_OVERRIDE_MASK                           0x01


/* WCD939X_PATH_CTL Fields: */
#define WCD939X_PATH_CTL_RESET_RIGHT_MASK                                0x08
#define WCD939X_PATH_CTL_RESET_LEFT_MASK                                 0x04
#define WCD939X_PATH_CTL_CLK_EN_RIGHT_MASK                               0x02
#define WCD939X_PATH_CTL_CLK_EN_LEFT_MASK                                0x01

/* WCD939X_CFG0 Fields: */
#define WCD939X_CFG0_AUTO_DISABLE_ANC_MASK                               0x04
#define WCD939X_CFG0_AUTO_DISABLE_DSD_MASK                               0x02
#define WCD939X_CFG0_IDLE_STEREO_MASK                                    0x01

/* WCD939X_CFG1 Fields: */
#define WCD939X_CFG1_IDLE_N_HOLDOFF_LSB_MASK                             0xff

/* WCD939X_CFG2 Fields: */
#define WCD939X_CFG2_IDLE_N_HOLDOFF_MSB_MASK                             0x0f

/* WCD939X_CFG3 Fields: */
#define WCD939X_CFG3_IDLE_THRESHOLD_MASK                                 0xff


/* WCD939X_DSD_HPHL_PATH_CTL Fields: */
#define WCD939X_DSD_HPHL_PATH_CTL_RESET_MASK                             0x02
#define WCD939X_DSD_HPHL_PATH_CTL_CLK_EN_MASK                            0x01

/* WCD939X_DSD_HPHL_CFG0 Fields: */
#define WCD939X_DSD_HPHL_CFG0_INP_SEL_MASK                               0x01

/* WCD939X_DSD_HPHL_CFG1 Fields: */
#define WCD939X_DSD_HPHL_CFG1_PGA_GAIN_MASK                              0xff

/* WCD939X_DSD_HPHL_CFG2 Fields: */
#define WCD939X_DSD_HPHL_CFG2_PGA_TIMER_MSB_EXT_MASK                     0x78
#define WCD939X_DSD_HPHL_CFG2_PGA_MUTE_EN_MASK                           0x04
#define WCD939X_DSD_HPHL_CFG2_PGA_MODE_MASK                              0x02
#define WCD939X_DSD_HPHL_CFG2_PGA_HALF_DB_MASK                           0x01

/* WCD939X_DSD_HPHL_CFG3 Fields: */
#define WCD939X_DSD_HPHL_CFG3_PGA_TIMER_MASK                             0xff

/* WCD939X_CFG4 Fields: */
#define WCD939X_CFG4_TOGGLE_THRESHOLD_MASK                               0x18
#define WCD939X_CFG4_MUTE_THRESHOLD_MASK                                 0x07

/* WCD939X_CFG5 Fields: */
#define WCD939X_CFG5_DATA_BIT_POLARITY_MASK                              0x02
#define WCD939X_CFG5_INP_BIT_POLARITY_MASK                               0x01


/* WCD939X_DSD_HPHR_PATH_CTL Fields: */
#define WCD939X_DSD_HPHR_PATH_CTL_RESET_MASK                             0x02
#define WCD939X_DSD_HPHR_PATH_CTL_CLK_EN_MASK                            0x01

/* WCD939X_DSD_HPHR_CFG0 Fields: */
#define WCD939X_DSD_HPHR_CFG0_INP_SEL_MASK                               0x01

/* WCD939X_DSD_HPHR_CFG1 Fields: */
#define WCD939X_DSD_HPHR_CFG1_PGA_GAIN_MASK                              0xff

/* WCD939X_DSD_HPHR_CFG2 Fields: */
#define WCD939X_DSD_HPHR_CFG2_PGA_TIMER_MSB_EXT_MASK                     0x78
#define WCD939X_DSD_HPHR_CFG2_PGA_MUTE_EN_MASK                           0x04
#define WCD939X_DSD_HPHR_CFG2_PGA_MODE_MASK                              0x02
#define WCD939X_DSD_HPHR_CFG2_PGA_HALF_DB_MASK                           0x01

/* WCD939X_DSD_HPHR_CFG3 Fields: */
#define WCD939X_DSD_HPHR_CFG3_PGA_TIMER_MASK                             0xff

/* WCD939X_DSD_HPHR_CFG4 Fields: */
#define WCD939X_DSD_HPHR_CFG4_TOGGLE_THRESHOLD_MASK                      0x18
#define WCD939X_DSD_HPHR_CFG4_MUTE_THRESHOLD_MASK                        0x07

/* WCD939X_DSD_HPHR_CFG5 Fields: */
#define WCD939X_DSD_HPHR_CFG5_DATA_BIT_POLARITY_MASK                     0x02
#define WCD939X_DSD_HPHR_CFG5_INP_BIT_POLARITY_MASK                      0x01


#endif /* WCD939X_REG_MASKS_H */
