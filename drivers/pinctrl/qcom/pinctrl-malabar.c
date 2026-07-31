// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-msm.h"
#define None 0

#define REG_BASE 0x100000
#define REG_SIZE 0x1000
#define PINGROUP(id, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, wake_off, bit)	\
	{						\
		.grp = PINCTRL_PINGROUP("gpio" #id,	\
			gpio##id##_pins,			\
			ARRAY_SIZE(gpio##id##_pins)),	\
		.ctl_reg = REG_BASE + REG_SIZE * id,		\
		.io_reg = REG_BASE + 0x4 + REG_SIZE * id,		\
		.intr_cfg_reg = REG_BASE + 0x8 + REG_SIZE * id,	\
		.intr_status_reg = REG_BASE + 0xc + REG_SIZE * id,	\
		.intr_target_reg = REG_BASE + 0x8 + REG_SIZE * id,	\
		.mux_bit = 2,			\
		.pull_bit = 0,			\
		.drv_bit = 6,			\
		.egpio_enable = 12,		\
		.egpio_present = 11,	\
		.oe_bit = 9,			\
		.in_bit = 0,			\
		.out_bit = 1,			\
		.intr_enable_bit = 0,		\
		.intr_status_bit = 0,		\
		.intr_target_bit = 5,		\
		.intr_target_kpss_val = 3,	\
		.intr_raw_status_bit = 4,	\
		.intr_polarity_bit = 1,		\
		.intr_detection_bit = 2,	\
		.intr_detection_width = 2,	\
		.wake_reg = REG_BASE + wake_off,	\
		.wake_bit = bit,		\
		.funcs = (int[]){			\
			msm_mux_gpio, /* gpio mode */	\
			msm_mux_##f1,			\
			msm_mux_##f2,			\
			msm_mux_##f3,			\
			msm_mux_##f4,			\
			msm_mux_##f5,			\
			msm_mux_##f6,			\
			msm_mux_##f7,			\
			msm_mux_##f8,			\
			msm_mux_##f9,			\
			msm_mux_##f10,			\
			msm_mux_##f11 /* egpio mode */	\
		},					        \
		.nfuncs = 12,				\
	}

#define SDC_QDSD_PINGROUP(pg_name, ctl, pull, drv)	\
	{					        \
		.grp = PINCTRL_PINGROUP(#pg_name,	\
			pg_name##_pins,		        \
			ARRAY_SIZE(pg_name##_pins)),    \
		.ctl_reg = ctl,				\
		.io_reg = 0,				\
		.intr_cfg_reg = 0,			\
		.intr_status_reg = 0,			\
		.intr_target_reg = 0,			\
		.mux_bit = -1,				\
		.pull_bit = pull,			\
		.drv_bit = drv,				\
		.oe_bit = -1,				\
		.in_bit = -1,				\
		.out_bit = -1,				\
		.intr_enable_bit = -1,			\
		.intr_status_bit = -1,			\
		.intr_target_bit = -1,			\
		.intr_raw_status_bit = -1,		\
		.intr_polarity_bit = -1,		\
		.intr_detection_bit = -1,		\
		.intr_detection_width = -1,		\
	}

#define UFS_RESET(pg_name, offset)				\
	{						\
		.grp = PINCTRL_PINGROUP(#pg_name,	\
			pg_name##_pins,			\
			ARRAY_SIZE(pg_name##_pins)),	\
		.ctl_reg = offset,			\
		.io_reg = offset + 0x4,			\
		.intr_cfg_reg = 0,			\
		.intr_status_reg = 0,			\
		.intr_target_reg = 0,			\
		.mux_bit = -1,				\
		.pull_bit = 3,				\
		.drv_bit = 0,				\
		.oe_bit = -1,				\
		.in_bit = -1,				\
		.out_bit = 0,				\
		.intr_enable_bit = -1,			\
		.intr_status_bit = -1,			\
		.intr_target_bit = -1,			\
		.intr_raw_status_bit = -1,		\
		.intr_polarity_bit = -1,		\
		.intr_detection_bit = -1,		\
		.intr_detection_width = -1,		\
	}

#define QUP_I3C(qup_mode, qup_offset)			\
	{						\
		.mode = qup_mode,			\
		.offset = REG_BASE + qup_offset,			\
	}


static const struct pinctrl_pin_desc malabar_pins[] = {
	PINCTRL_PIN(0, "GPIO_0"),
	PINCTRL_PIN(1, "GPIO_1"),
	PINCTRL_PIN(2, "GPIO_2"),
	PINCTRL_PIN(3, "GPIO_3"),
	PINCTRL_PIN(4, "GPIO_4"),
	PINCTRL_PIN(5, "GPIO_5"),
	PINCTRL_PIN(6, "GPIO_6"),
	PINCTRL_PIN(7, "GPIO_7"),
	PINCTRL_PIN(8, "GPIO_8"),
	PINCTRL_PIN(9, "GPIO_9"),
	PINCTRL_PIN(10, "GPIO_10"),
	PINCTRL_PIN(11, "GPIO_11"),
	PINCTRL_PIN(12, "GPIO_12"),
	PINCTRL_PIN(13, "GPIO_13"),
	PINCTRL_PIN(14, "GPIO_14"),
	PINCTRL_PIN(15, "GPIO_15"),
	PINCTRL_PIN(16, "GPIO_16"),
	PINCTRL_PIN(17, "GPIO_17"),
	PINCTRL_PIN(18, "GPIO_18"),
	PINCTRL_PIN(19, "GPIO_19"),
	PINCTRL_PIN(20, "GPIO_20"),
	PINCTRL_PIN(21, "GPIO_21"),
	PINCTRL_PIN(22, "GPIO_22"),
	PINCTRL_PIN(23, "GPIO_23"),
	PINCTRL_PIN(24, "GPIO_24"),
	PINCTRL_PIN(25, "GPIO_25"),
	PINCTRL_PIN(26, "GPIO_26"),
	PINCTRL_PIN(27, "GPIO_27"),
	PINCTRL_PIN(28, "GPIO_28"),
	PINCTRL_PIN(29, "GPIO_29"),
	PINCTRL_PIN(30, "GPIO_30"),
	PINCTRL_PIN(31, "GPIO_31"),
	PINCTRL_PIN(32, "GPIO_32"),
	PINCTRL_PIN(33, "GPIO_33"),
	PINCTRL_PIN(34, "GPIO_34"),
	PINCTRL_PIN(35, "GPIO_35"),
	PINCTRL_PIN(36, "GPIO_36"),
	PINCTRL_PIN(37, "GPIO_37"),
	PINCTRL_PIN(38, "GPIO_38"),
	PINCTRL_PIN(39, "GPIO_39"),
	PINCTRL_PIN(40, "GPIO_40"),
	PINCTRL_PIN(41, "GPIO_41"),
	PINCTRL_PIN(42, "GPIO_42"),
	PINCTRL_PIN(43, "GPIO_43"),
	PINCTRL_PIN(44, "GPIO_44"),
	PINCTRL_PIN(45, "GPIO_45"),
	PINCTRL_PIN(46, "GPIO_46"),
	PINCTRL_PIN(47, "GPIO_47"),
	PINCTRL_PIN(48, "GPIO_48"),
	PINCTRL_PIN(49, "GPIO_49"),
	PINCTRL_PIN(50, "GPIO_50"),
	PINCTRL_PIN(51, "GPIO_51"),
	PINCTRL_PIN(52, "GPIO_52"),
	PINCTRL_PIN(53, "GPIO_53"),
	PINCTRL_PIN(54, "GPIO_54"),
	PINCTRL_PIN(55, "GPIO_55"),
	PINCTRL_PIN(56, "GPIO_56"),
	PINCTRL_PIN(57, "GPIO_57"),
	PINCTRL_PIN(58, "GPIO_58"),
	PINCTRL_PIN(59, "GPIO_59"),
	PINCTRL_PIN(60, "GPIO_60"),
	PINCTRL_PIN(61, "GPIO_61"),
	PINCTRL_PIN(62, "GPIO_62"),
	PINCTRL_PIN(63, "GPIO_63"),
	PINCTRL_PIN(64, "GPIO_64"),
	PINCTRL_PIN(65, "GPIO_65"),
	PINCTRL_PIN(66, "GPIO_66"),
	PINCTRL_PIN(67, "GPIO_67"),
	PINCTRL_PIN(68, "GPIO_68"),
	PINCTRL_PIN(69, "GPIO_69"),
	PINCTRL_PIN(70, "GPIO_70"),
	PINCTRL_PIN(71, "GPIO_71"),
	PINCTRL_PIN(72, "GPIO_72"),
	PINCTRL_PIN(73, "GPIO_73"),
	PINCTRL_PIN(74, "GPIO_74"),
	PINCTRL_PIN(75, "GPIO_75"),
	PINCTRL_PIN(76, "GPIO_76"),
	PINCTRL_PIN(77, "GPIO_77"),
	PINCTRL_PIN(78, "GPIO_78"),
	PINCTRL_PIN(79, "GPIO_79"),
	PINCTRL_PIN(80, "GPIO_80"),
	PINCTRL_PIN(81, "GPIO_81"),
	PINCTRL_PIN(82, "GPIO_82"),
	PINCTRL_PIN(83, "GPIO_83"),
	PINCTRL_PIN(84, "GPIO_84"),
	PINCTRL_PIN(85, "GPIO_85"),
	PINCTRL_PIN(86, "GPIO_86"),
	PINCTRL_PIN(87, "GPIO_87"),
	PINCTRL_PIN(88, "GPIO_88"),
	PINCTRL_PIN(89, "GPIO_89"),
	PINCTRL_PIN(90, "GPIO_90"),
	PINCTRL_PIN(91, "GPIO_91"),
	PINCTRL_PIN(92, "GPIO_92"),
	PINCTRL_PIN(93, "GPIO_93"),
	PINCTRL_PIN(94, "GPIO_94"),
	PINCTRL_PIN(95, "GPIO_95"),
	PINCTRL_PIN(96, "GPIO_96"),
	PINCTRL_PIN(97, "GPIO_97"),
	PINCTRL_PIN(98, "GPIO_98"),
	PINCTRL_PIN(99, "GPIO_99"),
	PINCTRL_PIN(100, "GPIO_100"),
	PINCTRL_PIN(101, "GPIO_101"),
	PINCTRL_PIN(102, "GPIO_102"),
	PINCTRL_PIN(103, "GPIO_103"),
	PINCTRL_PIN(104, "GPIO_104"),
	PINCTRL_PIN(105, "GPIO_105"),
	PINCTRL_PIN(106, "GPIO_106"),
	PINCTRL_PIN(107, "GPIO_107"),
	PINCTRL_PIN(108, "GPIO_108"),
	PINCTRL_PIN(109, "GPIO_109"),
	PINCTRL_PIN(110, "GPIO_110"),
	PINCTRL_PIN(111, "GPIO_111"),
	PINCTRL_PIN(112, "GPIO_112"),
	PINCTRL_PIN(113, "GPIO_113"),
	PINCTRL_PIN(114, "UFS_RESET"),
	PINCTRL_PIN(115, "SDC2_CLK"),
	PINCTRL_PIN(116, "SDC2_CMD"),
	PINCTRL_PIN(117, "SDC2_DATA"),
};

#define DECLARE_MSM_GPIO_PINS(pin) \
	static const unsigned int gpio##pin##_pins[] = { pin }
DECLARE_MSM_GPIO_PINS(0);
DECLARE_MSM_GPIO_PINS(1);
DECLARE_MSM_GPIO_PINS(2);
DECLARE_MSM_GPIO_PINS(3);
DECLARE_MSM_GPIO_PINS(4);
DECLARE_MSM_GPIO_PINS(5);
DECLARE_MSM_GPIO_PINS(6);
DECLARE_MSM_GPIO_PINS(7);
DECLARE_MSM_GPIO_PINS(8);
DECLARE_MSM_GPIO_PINS(9);
DECLARE_MSM_GPIO_PINS(10);
DECLARE_MSM_GPIO_PINS(11);
DECLARE_MSM_GPIO_PINS(12);
DECLARE_MSM_GPIO_PINS(13);
DECLARE_MSM_GPIO_PINS(14);
DECLARE_MSM_GPIO_PINS(15);
DECLARE_MSM_GPIO_PINS(16);
DECLARE_MSM_GPIO_PINS(17);
DECLARE_MSM_GPIO_PINS(18);
DECLARE_MSM_GPIO_PINS(19);
DECLARE_MSM_GPIO_PINS(20);
DECLARE_MSM_GPIO_PINS(21);
DECLARE_MSM_GPIO_PINS(22);
DECLARE_MSM_GPIO_PINS(23);
DECLARE_MSM_GPIO_PINS(24);
DECLARE_MSM_GPIO_PINS(25);
DECLARE_MSM_GPIO_PINS(26);
DECLARE_MSM_GPIO_PINS(27);
DECLARE_MSM_GPIO_PINS(28);
DECLARE_MSM_GPIO_PINS(29);
DECLARE_MSM_GPIO_PINS(30);
DECLARE_MSM_GPIO_PINS(31);
DECLARE_MSM_GPIO_PINS(32);
DECLARE_MSM_GPIO_PINS(33);
DECLARE_MSM_GPIO_PINS(34);
DECLARE_MSM_GPIO_PINS(35);
DECLARE_MSM_GPIO_PINS(36);
DECLARE_MSM_GPIO_PINS(37);
DECLARE_MSM_GPIO_PINS(38);
DECLARE_MSM_GPIO_PINS(39);
DECLARE_MSM_GPIO_PINS(40);
DECLARE_MSM_GPIO_PINS(41);
DECLARE_MSM_GPIO_PINS(42);
DECLARE_MSM_GPIO_PINS(43);
DECLARE_MSM_GPIO_PINS(44);
DECLARE_MSM_GPIO_PINS(45);
DECLARE_MSM_GPIO_PINS(46);
DECLARE_MSM_GPIO_PINS(47);
DECLARE_MSM_GPIO_PINS(48);
DECLARE_MSM_GPIO_PINS(49);
DECLARE_MSM_GPIO_PINS(50);
DECLARE_MSM_GPIO_PINS(51);
DECLARE_MSM_GPIO_PINS(52);
DECLARE_MSM_GPIO_PINS(53);
DECLARE_MSM_GPIO_PINS(54);
DECLARE_MSM_GPIO_PINS(55);
DECLARE_MSM_GPIO_PINS(56);
DECLARE_MSM_GPIO_PINS(57);
DECLARE_MSM_GPIO_PINS(58);
DECLARE_MSM_GPIO_PINS(59);
DECLARE_MSM_GPIO_PINS(60);
DECLARE_MSM_GPIO_PINS(61);
DECLARE_MSM_GPIO_PINS(62);
DECLARE_MSM_GPIO_PINS(63);
DECLARE_MSM_GPIO_PINS(64);
DECLARE_MSM_GPIO_PINS(65);
DECLARE_MSM_GPIO_PINS(66);
DECLARE_MSM_GPIO_PINS(67);
DECLARE_MSM_GPIO_PINS(68);
DECLARE_MSM_GPIO_PINS(69);
DECLARE_MSM_GPIO_PINS(70);
DECLARE_MSM_GPIO_PINS(71);
DECLARE_MSM_GPIO_PINS(72);
DECLARE_MSM_GPIO_PINS(73);
DECLARE_MSM_GPIO_PINS(74);
DECLARE_MSM_GPIO_PINS(75);
DECLARE_MSM_GPIO_PINS(76);
DECLARE_MSM_GPIO_PINS(77);
DECLARE_MSM_GPIO_PINS(78);
DECLARE_MSM_GPIO_PINS(79);
DECLARE_MSM_GPIO_PINS(80);
DECLARE_MSM_GPIO_PINS(81);
DECLARE_MSM_GPIO_PINS(82);
DECLARE_MSM_GPIO_PINS(83);
DECLARE_MSM_GPIO_PINS(84);
DECLARE_MSM_GPIO_PINS(85);
DECLARE_MSM_GPIO_PINS(86);
DECLARE_MSM_GPIO_PINS(87);
DECLARE_MSM_GPIO_PINS(88);
DECLARE_MSM_GPIO_PINS(89);
DECLARE_MSM_GPIO_PINS(90);
DECLARE_MSM_GPIO_PINS(91);
DECLARE_MSM_GPIO_PINS(92);
DECLARE_MSM_GPIO_PINS(93);
DECLARE_MSM_GPIO_PINS(94);
DECLARE_MSM_GPIO_PINS(95);
DECLARE_MSM_GPIO_PINS(96);
DECLARE_MSM_GPIO_PINS(97);
DECLARE_MSM_GPIO_PINS(98);
DECLARE_MSM_GPIO_PINS(99);
DECLARE_MSM_GPIO_PINS(100);
DECLARE_MSM_GPIO_PINS(101);
DECLARE_MSM_GPIO_PINS(102);
DECLARE_MSM_GPIO_PINS(103);
DECLARE_MSM_GPIO_PINS(104);
DECLARE_MSM_GPIO_PINS(105);
DECLARE_MSM_GPIO_PINS(106);
DECLARE_MSM_GPIO_PINS(107);
DECLARE_MSM_GPIO_PINS(108);
DECLARE_MSM_GPIO_PINS(109);
DECLARE_MSM_GPIO_PINS(110);
DECLARE_MSM_GPIO_PINS(111);
DECLARE_MSM_GPIO_PINS(112);
DECLARE_MSM_GPIO_PINS(113);

static const unsigned int ufs_reset_pins[] = { 114 };
static const unsigned int sdc2_clk_pins[] = { 115 };
static const unsigned int sdc2_cmd_pins[] = { 116 };
static const unsigned int sdc2_data_pins[] = { 117 };

enum malabar_functions {
	msm_mux_gpio,
	msm_mux_adsp_ext,
	msm_mux_atest_char0,
	msm_mux_atest_char1,
	msm_mux_atest_char2,
	msm_mux_atest_char3,
	msm_mux_atest_char_start,
	msm_mux_atest_usb0,
	msm_mux_atest_usb00,
	msm_mux_atest_usb01,
	msm_mux_atest_usb02,
	msm_mux_atest_usb03,
	msm_mux_audio_ref_clk,
	msm_mux_cam_mclk,
	msm_mux_cci_async_in0,
	msm_mux_cci_i2c_scl0,
	msm_mux_cci_i2c_scl1,
	msm_mux_cci_i2c_scl2,
	msm_mux_cci_i2c_sda0,
	msm_mux_cci_i2c_sda1,
	msm_mux_cci_i2c_sda2,
	msm_mux_cci_timer0,
	msm_mux_cci_timer1,
	msm_mux_cci_timer2,
	msm_mux_cci_timer3,
	msm_mux_cmu_rng0,
	msm_mux_cmu_rng1,
	msm_mux_cmu_rng2,
	msm_mux_cmu_rng3,
	msm_mux_coex_uart1_rx,
	msm_mux_coex_uart1_tx,
	msm_mux_cri_trng,
	msm_mux_cri_trng0,
	msm_mux_cri_trng1,
	msm_mux_dbg_out_clk,
	msm_mux_ddr_bist_complete,
	msm_mux_ddr_bist_fail,
	msm_mux_ddr_bist_start,
	msm_mux_ddr_bist_stop,
	msm_mux_ddr_pxi0,
	msm_mux_ddr_pxi1,
	msm_mux_egpio,
	msm_mux_gcc_gp1,
	msm_mux_gcc_gp2,
	msm_mux_gcc_gp3,
	msm_mux_jitter_bist,
	msm_mux_mdp_vsync,
	msm_mux_mdp_vsync0_out,
	msm_mux_mdp_vsync1_out,
	msm_mux_mdp_vsync2_out,
	msm_mux_mdp_vsync3_out,
	msm_mux_mpm_pwr,
	msm_mux_nav_gpio0,
	msm_mux_nav_gpio1,
	msm_mux_nav_gpio2,
	msm_mux_pa_indicator,
	msm_mux_phase_flag0,
	msm_mux_phase_flag1,
	msm_mux_phase_flag10,
	msm_mux_phase_flag11,
	msm_mux_phase_flag12,
	msm_mux_phase_flag13,
	msm_mux_phase_flag14,
	msm_mux_phase_flag15,
	msm_mux_phase_flag16,
	msm_mux_phase_flag17,
	msm_mux_phase_flag18,
	msm_mux_phase_flag19,
	msm_mux_phase_flag2,
	msm_mux_phase_flag20,
	msm_mux_phase_flag21,
	msm_mux_phase_flag22,
	msm_mux_phase_flag23,
	msm_mux_phase_flag24,
	msm_mux_phase_flag25,
	msm_mux_phase_flag26,
	msm_mux_phase_flag27,
	msm_mux_phase_flag28,
	msm_mux_phase_flag29,
	msm_mux_phase_flag3,
	msm_mux_phase_flag30,
	msm_mux_phase_flag31,
	msm_mux_phase_flag4,
	msm_mux_phase_flag5,
	msm_mux_phase_flag6,
	msm_mux_phase_flag7,
	msm_mux_phase_flag8,
	msm_mux_phase_flag9,
	msm_mux_pll_bist_sync,
	msm_mux_pll_bypassnl,
	msm_mux_pll_clk_aux,
	msm_mux_pll_reset_n,
	msm_mux_prng_rosc0,
	msm_mux_prng_rosc1,
	msm_mux_prng_rosc2,
	msm_mux_prng_rosc3,
	msm_mux_qdss_cti,
	msm_mux_qdss_gpio,
	msm_mux_qdss_gpio0,
	msm_mux_qdss_gpio1,
	msm_mux_qdss_gpio10,
	msm_mux_qdss_gpio11,
	msm_mux_qdss_gpio12,
	msm_mux_qdss_gpio13,
	msm_mux_qdss_gpio14,
	msm_mux_qdss_gpio15,
	msm_mux_qdss_gpio2,
	msm_mux_qdss_gpio3,
	msm_mux_qdss_gpio4,
	msm_mux_qdss_gpio5,
	msm_mux_qdss_gpio6,
	msm_mux_qdss_gpio7,
	msm_mux_qdss_gpio8,
	msm_mux_qdss_gpio9,
	msm_mux_qlink0_enable,
	msm_mux_qlink0_request,
	msm_mux_qlink0_wmss,
	msm_mux_qup0_se0_l0,
	msm_mux_qup0_se0_l1,
	msm_mux_qup0_se0_l2,
	msm_mux_qup0_se0_l3,
	msm_mux_qup0_se1_l0,
	msm_mux_qup0_se1_l1,
	msm_mux_qup0_se1_l2,
	msm_mux_qup0_se1_l3,
	msm_mux_qup0_se2_l0,
	msm_mux_qup0_se2_l1,
	msm_mux_qup0_se2_l2,
	msm_mux_qup0_se2_l3,
	msm_mux_qup0_se3_l0,
	msm_mux_qup0_se3_l1,
	msm_mux_qup0_se3_l2,
	msm_mux_qup0_se3_l3,
	msm_mux_qup0_se4_l0,
	msm_mux_qup0_se4_l1,
	msm_mux_qup0_se4_l2,
	msm_mux_qup0_se4_l3,
	msm_mux_qup1_se0_l0,
	msm_mux_qup1_se0_l1,
	msm_mux_qup1_se0_l2,
	msm_mux_qup1_se0_l3,
	msm_mux_qup1_se1_l0,
	msm_mux_qup1_se1_l1,
	msm_mux_qup1_se1_l2,
	msm_mux_qup1_se1_l3,
	msm_mux_qup1_se2_l0,
	msm_mux_qup1_se2_l1,
	msm_mux_qup1_se2_l2,
	msm_mux_qup1_se2_l3,
	msm_mux_qup1_se3_l0,
	msm_mux_qup1_se3_l1,
	msm_mux_qup1_se3_l2,
	msm_mux_qup1_se3_l3,
	msm_mux_qup1_se4_l0,
	msm_mux_qup1_se4_l1,
	msm_mux_qup1_se4_l2,
	msm_mux_qup1_se4_l3,
	msm_mux_tb_trig_sdc1,
	msm_mux_tb_trig_sdc2,
	msm_mux_tgu_ch0_trigout,
	msm_mux_tgu_ch1_trigout,
	msm_mux_tgu_ch2_trigout,
	msm_mux_tgu_ch3_trigout,
	msm_mux_tmess_prng0,
	msm_mux_tmess_prng1,
	msm_mux_tmess_prng2,
	msm_mux_tmess_prng3,
	msm_mux_tsense_pwm1,
	msm_mux_tsense_pwm2,
	msm_mux_uim0_clk,
	msm_mux_uim0_data,
	msm_mux_uim0_present,
	msm_mux_uim0_reset,
	msm_mux_uim1_clk_mira,
	msm_mux_uim1_clk_mirb,
	msm_mux_uim1_data_mira,
	msm_mux_uim1_data_mirb,
	msm_mux_uim1_present_mira,
	msm_mux_uim1_present_mirb,
	msm_mux_uim1_reset_mira,
	msm_mux_uim1_reset_mirb,
	msm_mux_usb0_hs,
	msm_mux_vfr_0_mira,
	msm_mux_vfr_0_mirb,
	msm_mux_vfr_1,
	msm_mux_vsense_trigger_mirnat,
	msm_mux_wlan1_adc0,
	msm_mux_wlan1_adc1,
	msm_mux_NA,
};

static const char *const gpio_groups[] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio4", "gpio5",
	"gpio6", "gpio7", "gpio8", "gpio9", "gpio10", "gpio11",
	"gpio12", "gpio13", "gpio14", "gpio15", "gpio16", "gpio17",
	"gpio18", "gpio19", "gpio20", "gpio21", "gpio22", "gpio23",
	"gpio24", "gpio25", "gpio26", "gpio27", "gpio28", "gpio29",
	"gpio30", "gpio31", "gpio32", "gpio33", "gpio34", "gpio35",
	"gpio36", "gpio37", "gpio38", "gpio39", "gpio40", "gpio41",
	"gpio42", "gpio43", "gpio44", "gpio45", "gpio46", "gpio47",
	"gpio48", "gpio49", "gpio50", "gpio51", "gpio52", "gpio53",
	"gpio54", "gpio55", "gpio56", "gpio57", "gpio58", "gpio59",
	"gpio60", "gpio61", "gpio62", "gpio63", "gpio64", "gpio65",
	"gpio66", "gpio67", "gpio68", "gpio69", "gpio70", "gpio71",
	"gpio72", "gpio73", "gpio74", "gpio75", "gpio76", "gpio77",
	"gpio78", "gpio79", "gpio80", "gpio81", "gpio82", "gpio83",
	"gpio84", "gpio85", "gpio86", "gpio87", "gpio88", "gpio89",
	"gpio90", "gpio91", "gpio92", "gpio93", "gpio94", "gpio95",
	"gpio96", "gpio97", "gpio98", "gpio99", "gpio100", "gpio101",
	"gpio102", "gpio103", "gpio104", "gpio105", "gpio106", "gpio107",
	"gpio108", "gpio109", "gpio110", "gpio111", "gpio112", "gpio113",
};

static const char *const adsp_ext_groups[] = {
	"gpio74",
};

static const char *const atest_char0_groups[] = {
	"gpio85",
};

static const char *const atest_char1_groups[] = {
	"gpio86",
};

static const char *const atest_char2_groups[] = {
	"gpio63",
};

static const char *const atest_char3_groups[] = {
	"gpio62",
};

static const char *const atest_char_start_groups[] = {
	"gpio29",
};

static const char *const atest_usb0_groups[] = {
	"gpio30",
};

static const char *const atest_usb00_groups[] = {
	"gpio84",
};

static const char *const atest_usb01_groups[] = {
	"gpio82",
};

static const char *const atest_usb02_groups[] = {
	"gpio32",
};

static const char *const atest_usb03_groups[] = {
	"gpio26",
};

static const char *const audio_ref_clk_groups[] = {
	"gpio26",
};

static const char *const cam_mclk_groups[] = {
	"gpio24", "gpio25", "gpio26",
};

static const char *const cci_async_in0_groups[] = {
	"gpio27",
};

static const char *const cci_i2c_scl0_groups[] = {
	"gpio31",
};

static const char *const cci_i2c_scl1_groups[] = {
	"gpio33",
};

static const char *const cci_i2c_scl2_groups[] = {
	"gpio35",
};

static const char *const cci_i2c_sda0_groups[] = {
	"gpio30",
};

static const char *const cci_i2c_sda1_groups[] = {
	"gpio32",
};

static const char *const cci_i2c_sda2_groups[] = {
	"gpio34",
};

static const char *const cci_timer0_groups[] = {
	"gpio27",
};

static const char *const cci_timer1_groups[] = {
	"gpio28",
};

static const char *const cci_timer2_groups[] = {
	"gpio29",
};

static const char *const cci_timer3_groups[] = {
	"gpio13",
};

static const char *const cmu_rng0_groups[] = {
	"gpio81",
};

static const char *const cmu_rng1_groups[] = {
	"gpio35",
};

static const char *const cmu_rng2_groups[] = {
	"gpio80",
};

static const char *const cmu_rng3_groups[] = {
	"gpio32",
};

static const char *const coex_uart1_rx_groups[] = {
	"gpio21",
};

static const char *const coex_uart1_tx_groups[] = {
	"gpio22",
};

static const char *const cri_trng_groups[] = {
	"gpio12",
};

static const char *const cri_trng0_groups[] = {
	"gpio40",
};

static const char *const cri_trng1_groups[] = {
	"gpio34",
};

static const char *const dbg_out_clk_groups[] = {
	"gpio80",
};

static const char *const ddr_bist_complete_groups[] = {
	"gpio72",
};

static const char *const ddr_bist_fail_groups[] = {
	"gpio73",
};

static const char *const ddr_bist_start_groups[] = {
	"gpio30",
};

static const char *const ddr_bist_stop_groups[] = {
	"gpio31",
};

static const char *const ddr_pxi0_groups[] = {
	"gpio33", "gpio57",
};

static const char *const ddr_pxi1_groups[] = {
	"gpio97", "gpio98",
};

static const char *const egpio_groups[] = {
	"gpio87", "gpio88", "gpio89", "gpio90", "gpio91", "gpio92",
	"gpio93", "gpio94", "gpio95", "gpio96", "gpio97", "gpio98",
	"gpio99", "gpio100", "gpio101", "gpio102", "gpio103", "gpio104",
	"gpio105", "gpio106", "gpio107", "gpio108", "gpio109", "gpio110",
	"gpio111", "gpio112", "gpio113",
};

static const char *const gcc_gp1_groups[] = {
	"gpio51", "gpio91",
};

static const char *const gcc_gp2_groups[] = {
	"gpio30", "gpio49",
};

static const char *const gcc_gp3_groups[] = {
	"gpio10", "gpio50",
};

static const char *const jitter_bist_groups[] = {
	"gpio33",
};

static const char *const mdp_vsync_groups[] = {
	"gpio14", "gpio28", "gpio70", "gpio71", "gpio80", "gpio86",
};

static const char *const mdp_vsync0_out_groups[] = {
	"gpio14",
};

static const char *const mdp_vsync1_out_groups[] = {
	"gpio14",
};

static const char *const mdp_vsync2_out_groups[] = {
	"gpio28",
};

static const char *const mdp_vsync3_out_groups[] = {
	"gpio28",
};

static const char *const mpm_pwr_groups[] = {
	"gpio37",
};

static const char *const nav_gpio0_groups[] = {
	"gpio64",
};

static const char *const nav_gpio1_groups[] = {
	"gpio65",
};

static const char *const nav_gpio2_groups[] = {
	"gpio66",
};

static const char *const pa_indicator_groups[] = {
	"gpio66",
};

static const char *const phase_flag0_groups[] = {
	"gpio3",
};

static const char *const phase_flag1_groups[] = {
	"gpio78",
};

static const char *const phase_flag10_groups[] = {
	"gpio18",
};

static const char *const phase_flag11_groups[] = {
	"gpio20",
};

static const char *const phase_flag12_groups[] = {
	"gpio16",
};

static const char *const phase_flag13_groups[] = {
	"gpio63",
};

static const char *const phase_flag14_groups[] = {
	"gpio29",
};

static const char *const phase_flag15_groups[] = {
	"gpio56",
};

static const char *const phase_flag16_groups[] = {
	"gpio70",
};

static const char *const phase_flag17_groups[] = {
	"gpio62",
};

static const char *const phase_flag18_groups[] = {
	"gpio61",
};

static const char *const phase_flag19_groups[] = {
	"gpio55",
};

static const char *const phase_flag2_groups[] = {
	"gpio7",
};

static const char *const phase_flag20_groups[] = {
	"gpio99",
};

static const char *const phase_flag21_groups[] = {
	"gpio68",
};

static const char *const phase_flag22_groups[] = {
	"gpio28",
};

static const char *const phase_flag23_groups[] = {
	"gpio52",
};

static const char *const phase_flag24_groups[] = {
	"gpio85",
};

static const char *const phase_flag25_groups[] = {
	"gpio86",
};

static const char *const phase_flag26_groups[] = {
	"gpio53",
};

static const char *const phase_flag27_groups[] = {
	"gpio54",
};

static const char *const phase_flag28_groups[] = {
	"gpio58",
};

static const char *const phase_flag29_groups[] = {
	"gpio59",
};

static const char *const phase_flag3_groups[] = {
	"gpio8",
};

static const char *const phase_flag30_groups[] = {
	"gpio60",
};

static const char *const phase_flag31_groups[] = {
	"gpio67",
};

static const char *const phase_flag4_groups[] = {
	"gpio9",
};

static const char *const phase_flag5_groups[] = {
	"gpio11",
};

static const char *const phase_flag6_groups[] = {
	"gpio4",
};

static const char *const phase_flag7_groups[] = {
	"gpio5",
};

static const char *const phase_flag8_groups[] = {
	"gpio6",
};

static const char *const phase_flag9_groups[] = {
	"gpio17",
};

static const char *const pll_bist_sync_groups[] = {
	"gpio46",
};

static const char *const pll_bypassnl_groups[] = {
	"gpio48",
};

static const char *const pll_clk_aux_groups[] = {
	"gpio103",
};

static const char *const pll_reset_n_groups[] = {
	"gpio25",
};

static const char *const prng_rosc0_groups[] = {
	"gpio37",
};

static const char *const prng_rosc1_groups[] = {
	"gpio38",
};

static const char *const prng_rosc2_groups[] = {
	"gpio39",
};

static const char *const prng_rosc3_groups[] = {
	"gpio36",
};

static const char *const qdss_cti_groups[] = {
	"gpio8", "gpio12", "gpio13", "gpio28", "gpio43", "gpio44",
	"gpio51", "gpio90",
};

static const char *const qdss_gpio_groups[] = {
	"gpio4", "gpio5", "gpio105", "gpio108",
};

static const char *const qdss_gpio0_groups[] = {
	"gpio77", "gpio111",
};

static const char *const qdss_gpio1_groups[] = {
	"gpio78", "gpio107",
};

static const char *const qdss_gpio10_groups[] = {
	"gpio6", "gpio93",
};

static const char *const qdss_gpio11_groups[] = {
	"gpio7", "gpio27",
};

static const char *const qdss_gpio12_groups[] = {
	"gpio20", "gpio94",
};

static const char *const qdss_gpio13_groups[] = {
	"gpio19", "gpio95",
};

static const char *const qdss_gpio14_groups[] = {
	"gpio15", "gpio113",
};

static const char *const qdss_gpio15_groups[] = {
	"gpio17", "gpio104",
};

static const char *const qdss_gpio2_groups[] = {
	"gpio23", "gpio106",
};

static const char *const qdss_gpio3_groups[] = {
	"gpio34", "gpio38",
};

static const char *const qdss_gpio4_groups[] = {
	"gpio36", "gpio96",
};

static const char *const qdss_gpio5_groups[] = {
	"gpio37", "gpio112",
};

static const char *const qdss_gpio6_groups[] = {
	"gpio81", "gpio110",
};

static const char *const qdss_gpio7_groups[] = {
	"gpio39", "gpio109",
};

static const char *const qdss_gpio8_groups[] = {
	"gpio16", "gpio31",
};

static const char *const qdss_gpio9_groups[] = {
	"gpio18", "gpio100",
};

static const char *const qlink0_enable_groups[] = {
	"gpio83",
};

static const char *const qlink0_request_groups[] = {
	"gpio82",
};

static const char *const qlink0_wmss_groups[] = {
	"gpio84",
};

static const char *const qup0_se0_l0_groups[] = {
	"gpio0",
};

static const char *const qup0_se0_l1_groups[] = {
	"gpio1",
};

static const char *const qup0_se0_l2_groups[] = {
	"gpio2",
};

static const char *const qup0_se0_l3_groups[] = {
	"gpio3",
};

static const char *const qup0_se1_l0_groups[] = {
	"gpio4",
};

static const char *const qup0_se1_l1_groups[] = {
	"gpio5",
};

static const char *const qup0_se1_l2_groups[] = {
	"gpio23",
};

static const char *const qup0_se1_l3_groups[] = {
	"gpio77",
};

static const char *const qup0_se2_l0_groups[] = {
	"gpio17",
};

static const char *const qup0_se2_l1_groups[] = {
	"gpio18",
};

static const char *const qup0_se2_l2_groups[] = {
	"gpio19",
};

static const char *const qup0_se2_l3_groups[] = {
	"gpio20",
};

static const char *const qup0_se3_l0_groups[] = {
	"gpio8",
};

static const char *const qup0_se3_l1_groups[] = {
	"gpio9",
};

static const char *const qup0_se3_l2_groups[] = {
	"gpio10",
};

static const char *const qup0_se3_l3_groups[] = {
	"gpio11",
};

static const char *const qup0_se4_l0_groups[] = {
	"gpio6",
};

static const char *const qup0_se4_l1_groups[] = {
	"gpio7",
};

static const char *const qup0_se4_l2_groups[] = {
	"gpio12",
};

static const char *const qup0_se4_l3_groups[] = {
	"gpio13",
};

static const char *const qup1_se0_l0_groups[] = {
	"gpio36",
};

static const char *const qup1_se0_l1_groups[] = {
	"gpio37",
};

static const char *const qup1_se0_l2_groups[] = {
	"gpio38",
};

static const char *const qup1_se0_l3_groups[] = {
	"gpio39",
};

static const char *const qup1_se1_l0_groups[] = {
	"gpio78",
};

static const char *const qup1_se1_l1_groups[] = {
	"gpio79",
};

static const char *const qup1_se1_l2_groups[] = {
	"gpio69",
};

static const char *const qup1_se1_l3_groups[] = {
	"gpio70",
};

static const char *const qup1_se2_l0_groups[] = {
	"gpio40",
};

static const char *const qup1_se2_l1_groups[] = {
	"gpio41",
};

static const char *const qup1_se2_l2_groups[] = {
	"gpio42",
};

static const char *const qup1_se2_l3_groups[] = {
	"gpio43",
};

static const char *const qup1_se3_l0_groups[] = {
	"gpio80",
};

static const char *const qup1_se3_l1_groups[] = {
	"gpio81",
};

static const char *const qup1_se3_l2_groups[] = {
	"gpio15",
};

static const char *const qup1_se3_l3_groups[] = {
	"gpio16",
};

static const char *const qup1_se4_l0_groups[] = {
	"gpio71",
};

static const char *const qup1_se4_l1_groups[] = {
	"gpio72",
};

static const char *const qup1_se4_l2_groups[] = {
	"gpio73",
};

static const char *const qup1_se4_l3_groups[] = {
	"gpio74",
};

static const char *const tb_trig_sdc1_groups[] = {
	"gpio102",
};

static const char *const tb_trig_sdc2_groups[] = {
	"gpio19",
};

static const char *const tgu_ch0_trigout_groups[] = {
	"gpio36",
};

static const char *const tgu_ch1_trigout_groups[] = {
	"gpio37",
};

static const char *const tgu_ch2_trigout_groups[] = {
	"gpio38",
};

static const char *const tgu_ch3_trigout_groups[] = {
	"gpio39",
};

static const char *const tmess_prng0_groups[] = {
	"gpio57",
};

static const char *const tmess_prng1_groups[] = {
	"gpio58",
};

static const char *const tmess_prng2_groups[] = {
	"gpio59",
};

static const char *const tmess_prng3_groups[] = {
	"gpio60",
};

static const char *const tsense_pwm1_groups[] = {
	"gpio71",
};

static const char *const tsense_pwm2_groups[] = {
	"gpio71",
};

static const char *const uim0_clk_groups[] = {
	"gpio45",
};

static const char *const uim0_data_groups[] = {
	"gpio44",
};

static const char *const uim0_present_groups[] = {
	"gpio47",
};

static const char *const uim0_reset_groups[] = {
	"gpio46",
};

static const char *const uim1_clk_mira_groups[] = {
	"gpio49",
};

static const char *const uim1_clk_mirb_groups[] = {
	"gpio41",
};

static const char *const uim1_data_mira_groups[] = {
	"gpio48",
};

static const char *const uim1_data_mirb_groups[] = {
	"gpio40",
};

static const char *const uim1_present_mira_groups[] = {
	"gpio51",
};

static const char *const uim1_present_mirb_groups[] = {
	"gpio43",
};

static const char *const uim1_reset_mira_groups[] = {
	"gpio50",
};

static const char *const uim1_reset_mirb_groups[] = {
	"gpio42",
};

static const char *const usb0_hs_groups[] = {
	"gpio23",
};

static const char *const vfr_0_mira_groups[] = {
	"gpio1",
};

static const char *const vfr_0_mirb_groups[] = {
	"gpio69",
};

static const char *const vfr_1_groups[] = {
	"gpio65",
};

static const char *const vsense_trigger_mirnat_groups[] = {
	"gpio30",
};

static const char *const wlan1_adc0_groups[] = {
	"gpio32",
};

static const char *const wlan1_adc1_groups[] = {
	"gpio26",
};


static const struct pinfunction malabar_functions[] = {
	MSM_PIN_FUNCTION(gpio),
	MSM_PIN_FUNCTION(adsp_ext),
	MSM_PIN_FUNCTION(atest_char0),
	MSM_PIN_FUNCTION(atest_char1),
	MSM_PIN_FUNCTION(atest_char2),
	MSM_PIN_FUNCTION(atest_char3),
	MSM_PIN_FUNCTION(atest_char_start),
	MSM_PIN_FUNCTION(atest_usb0),
	MSM_PIN_FUNCTION(atest_usb00),
	MSM_PIN_FUNCTION(atest_usb01),
	MSM_PIN_FUNCTION(atest_usb02),
	MSM_PIN_FUNCTION(atest_usb03),
	MSM_PIN_FUNCTION(audio_ref_clk),
	MSM_PIN_FUNCTION(cam_mclk),
	MSM_PIN_FUNCTION(cci_async_in0),
	MSM_PIN_FUNCTION(cci_i2c_scl0),
	MSM_PIN_FUNCTION(cci_i2c_scl1),
	MSM_PIN_FUNCTION(cci_i2c_scl2),
	MSM_PIN_FUNCTION(cci_i2c_sda0),
	MSM_PIN_FUNCTION(cci_i2c_sda1),
	MSM_PIN_FUNCTION(cci_i2c_sda2),
	MSM_PIN_FUNCTION(cci_timer0),
	MSM_PIN_FUNCTION(cci_timer1),
	MSM_PIN_FUNCTION(cci_timer2),
	MSM_PIN_FUNCTION(cci_timer3),
	MSM_PIN_FUNCTION(cmu_rng0),
	MSM_PIN_FUNCTION(cmu_rng1),
	MSM_PIN_FUNCTION(cmu_rng2),
	MSM_PIN_FUNCTION(cmu_rng3),
	MSM_PIN_FUNCTION(coex_uart1_rx),
	MSM_PIN_FUNCTION(coex_uart1_tx),
	MSM_PIN_FUNCTION(cri_trng),
	MSM_PIN_FUNCTION(cri_trng0),
	MSM_PIN_FUNCTION(cri_trng1),
	MSM_PIN_FUNCTION(dbg_out_clk),
	MSM_PIN_FUNCTION(ddr_bist_complete),
	MSM_PIN_FUNCTION(ddr_bist_fail),
	MSM_PIN_FUNCTION(ddr_bist_start),
	MSM_PIN_FUNCTION(ddr_bist_stop),
	MSM_PIN_FUNCTION(ddr_pxi0),
	MSM_PIN_FUNCTION(ddr_pxi1),
	MSM_PIN_FUNCTION(egpio),
	MSM_PIN_FUNCTION(gcc_gp1),
	MSM_PIN_FUNCTION(gcc_gp2),
	MSM_PIN_FUNCTION(gcc_gp3),
	MSM_PIN_FUNCTION(jitter_bist),
	MSM_PIN_FUNCTION(mdp_vsync),
	MSM_PIN_FUNCTION(mdp_vsync0_out),
	MSM_PIN_FUNCTION(mdp_vsync1_out),
	MSM_PIN_FUNCTION(mdp_vsync2_out),
	MSM_PIN_FUNCTION(mdp_vsync3_out),
	MSM_PIN_FUNCTION(mpm_pwr),
	MSM_PIN_FUNCTION(nav_gpio0),
	MSM_PIN_FUNCTION(nav_gpio1),
	MSM_PIN_FUNCTION(nav_gpio2),
	MSM_PIN_FUNCTION(pa_indicator),
	MSM_PIN_FUNCTION(phase_flag0),
	MSM_PIN_FUNCTION(phase_flag1),
	MSM_PIN_FUNCTION(phase_flag10),
	MSM_PIN_FUNCTION(phase_flag11),
	MSM_PIN_FUNCTION(phase_flag12),
	MSM_PIN_FUNCTION(phase_flag13),
	MSM_PIN_FUNCTION(phase_flag14),
	MSM_PIN_FUNCTION(phase_flag15),
	MSM_PIN_FUNCTION(phase_flag16),
	MSM_PIN_FUNCTION(phase_flag17),
	MSM_PIN_FUNCTION(phase_flag18),
	MSM_PIN_FUNCTION(phase_flag19),
	MSM_PIN_FUNCTION(phase_flag2),
	MSM_PIN_FUNCTION(phase_flag20),
	MSM_PIN_FUNCTION(phase_flag21),
	MSM_PIN_FUNCTION(phase_flag22),
	MSM_PIN_FUNCTION(phase_flag23),
	MSM_PIN_FUNCTION(phase_flag24),
	MSM_PIN_FUNCTION(phase_flag25),
	MSM_PIN_FUNCTION(phase_flag26),
	MSM_PIN_FUNCTION(phase_flag27),
	MSM_PIN_FUNCTION(phase_flag28),
	MSM_PIN_FUNCTION(phase_flag29),
	MSM_PIN_FUNCTION(phase_flag3),
	MSM_PIN_FUNCTION(phase_flag30),
	MSM_PIN_FUNCTION(phase_flag31),
	MSM_PIN_FUNCTION(phase_flag4),
	MSM_PIN_FUNCTION(phase_flag5),
	MSM_PIN_FUNCTION(phase_flag6),
	MSM_PIN_FUNCTION(phase_flag7),
	MSM_PIN_FUNCTION(phase_flag8),
	MSM_PIN_FUNCTION(phase_flag9),
	MSM_PIN_FUNCTION(pll_bist_sync),
	MSM_PIN_FUNCTION(pll_bypassnl),
	MSM_PIN_FUNCTION(pll_clk_aux),
	MSM_PIN_FUNCTION(pll_reset_n),
	MSM_PIN_FUNCTION(prng_rosc0),
	MSM_PIN_FUNCTION(prng_rosc1),
	MSM_PIN_FUNCTION(prng_rosc2),
	MSM_PIN_FUNCTION(prng_rosc3),
	MSM_PIN_FUNCTION(qdss_cti),
	MSM_PIN_FUNCTION(qdss_gpio),
	MSM_PIN_FUNCTION(qdss_gpio0),
	MSM_PIN_FUNCTION(qdss_gpio1),
	MSM_PIN_FUNCTION(qdss_gpio10),
	MSM_PIN_FUNCTION(qdss_gpio11),
	MSM_PIN_FUNCTION(qdss_gpio12),
	MSM_PIN_FUNCTION(qdss_gpio13),
	MSM_PIN_FUNCTION(qdss_gpio14),
	MSM_PIN_FUNCTION(qdss_gpio15),
	MSM_PIN_FUNCTION(qdss_gpio2),
	MSM_PIN_FUNCTION(qdss_gpio3),
	MSM_PIN_FUNCTION(qdss_gpio4),
	MSM_PIN_FUNCTION(qdss_gpio5),
	MSM_PIN_FUNCTION(qdss_gpio6),
	MSM_PIN_FUNCTION(qdss_gpio7),
	MSM_PIN_FUNCTION(qdss_gpio8),
	MSM_PIN_FUNCTION(qdss_gpio9),
	MSM_PIN_FUNCTION(qlink0_enable),
	MSM_PIN_FUNCTION(qlink0_request),
	MSM_PIN_FUNCTION(qlink0_wmss),
	MSM_PIN_FUNCTION(qup0_se0_l0),
	MSM_PIN_FUNCTION(qup0_se0_l1),
	MSM_PIN_FUNCTION(qup0_se0_l2),
	MSM_PIN_FUNCTION(qup0_se0_l3),
	MSM_PIN_FUNCTION(qup0_se1_l0),
	MSM_PIN_FUNCTION(qup0_se1_l1),
	MSM_PIN_FUNCTION(qup0_se1_l2),
	MSM_PIN_FUNCTION(qup0_se1_l3),
	MSM_PIN_FUNCTION(qup0_se2_l0),
	MSM_PIN_FUNCTION(qup0_se2_l1),
	MSM_PIN_FUNCTION(qup0_se2_l2),
	MSM_PIN_FUNCTION(qup0_se2_l3),
	MSM_PIN_FUNCTION(qup0_se3_l0),
	MSM_PIN_FUNCTION(qup0_se3_l1),
	MSM_PIN_FUNCTION(qup0_se3_l2),
	MSM_PIN_FUNCTION(qup0_se3_l3),
	MSM_PIN_FUNCTION(qup0_se4_l0),
	MSM_PIN_FUNCTION(qup0_se4_l1),
	MSM_PIN_FUNCTION(qup0_se4_l2),
	MSM_PIN_FUNCTION(qup0_se4_l3),
	MSM_PIN_FUNCTION(qup1_se0_l0),
	MSM_PIN_FUNCTION(qup1_se0_l1),
	MSM_PIN_FUNCTION(qup1_se0_l2),
	MSM_PIN_FUNCTION(qup1_se0_l3),
	MSM_PIN_FUNCTION(qup1_se1_l0),
	MSM_PIN_FUNCTION(qup1_se1_l1),
	MSM_PIN_FUNCTION(qup1_se1_l2),
	MSM_PIN_FUNCTION(qup1_se1_l3),
	MSM_PIN_FUNCTION(qup1_se2_l0),
	MSM_PIN_FUNCTION(qup1_se2_l1),
	MSM_PIN_FUNCTION(qup1_se2_l2),
	MSM_PIN_FUNCTION(qup1_se2_l3),
	MSM_PIN_FUNCTION(qup1_se3_l0),
	MSM_PIN_FUNCTION(qup1_se3_l1),
	MSM_PIN_FUNCTION(qup1_se3_l2),
	MSM_PIN_FUNCTION(qup1_se3_l3),
	MSM_PIN_FUNCTION(qup1_se4_l0),
	MSM_PIN_FUNCTION(qup1_se4_l1),
	MSM_PIN_FUNCTION(qup1_se4_l2),
	MSM_PIN_FUNCTION(qup1_se4_l3),
	MSM_PIN_FUNCTION(tb_trig_sdc1),
	MSM_PIN_FUNCTION(tb_trig_sdc2),
	MSM_PIN_FUNCTION(tgu_ch0_trigout),
	MSM_PIN_FUNCTION(tgu_ch1_trigout),
	MSM_PIN_FUNCTION(tgu_ch2_trigout),
	MSM_PIN_FUNCTION(tgu_ch3_trigout),
	MSM_PIN_FUNCTION(tmess_prng0),
	MSM_PIN_FUNCTION(tmess_prng1),
	MSM_PIN_FUNCTION(tmess_prng2),
	MSM_PIN_FUNCTION(tmess_prng3),
	MSM_PIN_FUNCTION(tsense_pwm1),
	MSM_PIN_FUNCTION(tsense_pwm2),
	MSM_PIN_FUNCTION(uim0_clk),
	MSM_PIN_FUNCTION(uim0_data),
	MSM_PIN_FUNCTION(uim0_present),
	MSM_PIN_FUNCTION(uim0_reset),
	MSM_PIN_FUNCTION(uim1_clk_mira),
	MSM_PIN_FUNCTION(uim1_clk_mirb),
	MSM_PIN_FUNCTION(uim1_data_mira),
	MSM_PIN_FUNCTION(uim1_data_mirb),
	MSM_PIN_FUNCTION(uim1_present_mira),
	MSM_PIN_FUNCTION(uim1_present_mirb),
	MSM_PIN_FUNCTION(uim1_reset_mira),
	MSM_PIN_FUNCTION(uim1_reset_mirb),
	MSM_PIN_FUNCTION(usb0_hs),
	MSM_PIN_FUNCTION(vfr_0_mira),
	MSM_PIN_FUNCTION(vfr_0_mirb),
	MSM_PIN_FUNCTION(vfr_1),
	MSM_PIN_FUNCTION(vsense_trigger_mirnat),
	MSM_PIN_FUNCTION(wlan1_adc0),
	MSM_PIN_FUNCTION(wlan1_adc1),
};

/* Every pin is maintained as a single group, and missing or non-existing pin
 * would be maintained as dummy group to synchronize pin group index with
 * pin descriptor registered with pinctrl core.
 * Clients would not be able to request these dummy pin groups.
 */
static const struct msm_pingroup malabar_groups[] = {
	[0] = PINGROUP(0, qup0_se0_l0, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
		       0x72000, 3),
	[1] = PINGROUP(1, qup0_se0_l1, vfr_0_mira, NA, NA, NA, NA, NA, NA, NA,
		       NA, NA, 0, -1),
	[2] = PINGROUP(2, qup0_se0_l2, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
		       0, -1),
	[3] = PINGROUP(3, qup0_se0_l3, NA, phase_flag0, NA, NA, NA, NA, NA, NA,
		       NA, NA, 0x72000, 4),
	[4] = PINGROUP(4, qup0_se1_l0, NA, phase_flag6, NA, qdss_gpio, NA, NA,
		       NA, NA, NA, NA, 0x72000, 5),
	[5] = PINGROUP(5, qup0_se1_l1, NA, phase_flag7, qdss_gpio, NA, NA, NA,
		       NA, NA, NA, NA, 0, -1),
	[6] = PINGROUP(6, qup0_se4_l0, NA, phase_flag8, NA, qdss_gpio10, NA, NA,
		       NA, NA, NA, NA, 0x72000, 6),
	[7] = PINGROUP(7, qup0_se4_l1, NA, phase_flag2, NA, qdss_gpio11, NA, NA,
		       NA, NA, NA, NA, 0, -1),
	[8] = PINGROUP(8, qup0_se3_l0, NA, phase_flag3, qdss_cti, NA, NA, NA,
		       NA, NA, NA, NA, 0x72000, 7),
	[9] = PINGROUP(9, qup0_se3_l1, NA, phase_flag4, NA, NA, NA, NA, NA, NA,
		       NA, NA, 0x72000, 8),
	[10] = PINGROUP(10, qup0_se3_l2, gcc_gp3, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0x72000, 9),
	[11] = PINGROUP(11, qup0_se3_l3, NA, phase_flag5, NA, NA, NA, NA, NA,
			NA, NA, NA, 0x72000, 10),
	[12] = PINGROUP(12, qup0_se4_l2, cri_trng, qdss_cti, NA, NA, NA, NA, NA,
			NA, NA, NA, 0x72000, 11),
	[13] = PINGROUP(13, cci_timer3, qup0_se4_l3, qdss_cti, NA, NA, NA, NA,
			NA, NA, NA, NA, 0x72000, 12),
	[14] = PINGROUP(14, mdp_vsync, mdp_vsync0_out, mdp_vsync1_out, NA, NA,
			NA, NA, NA, NA, NA, NA, 0x72000, 13),
	[15] = PINGROUP(15, qup1_se3_l2, qdss_gpio14, NA, NA, NA, NA, NA, NA,
			NA, NA, NA, 0x72000, 14),
	[16] = PINGROUP(16, qup1_se3_l3, NA, phase_flag12, qdss_gpio8, NA, NA,
			NA, NA, NA, NA, NA, 0x72000, 15),
	[17] = PINGROUP(17, qup0_se2_l0, NA, phase_flag9, NA, qdss_gpio15, NA,
			NA, NA, NA, NA, NA, 0x72004, 0),
	[18] = PINGROUP(18, qup0_se2_l1, NA, phase_flag10, qdss_gpio9, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[19] = PINGROUP(19, qup0_se2_l2, tb_trig_sdc2, qdss_gpio13, NA, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[20] = PINGROUP(20, qup0_se2_l3, NA, phase_flag11, qdss_gpio12, NA, NA,
			NA, NA, NA, NA, NA, 0x72004, 1),
	[21] = PINGROUP(21, coex_uart1_rx, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0x72014, 0),
	[22] = PINGROUP(22, coex_uart1_tx, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[23] = PINGROUP(23, qup0_se1_l2, usb0_hs, qdss_gpio2, NA, NA, NA, NA,
			NA, NA, NA, NA, 0x72004, 2),
	[24] = PINGROUP(24, cam_mclk, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[25] = PINGROUP(25, cam_mclk, NA, pll_reset_n, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[26] = PINGROUP(26, cam_mclk, NA, audio_ref_clk, wlan1_adc1,
			atest_usb03, NA, NA, NA, NA, NA, NA, 0x7200C, 3),
	[27] = PINGROUP(27, cci_timer0, cci_async_in0, qdss_gpio11, NA, NA, NA,
			NA, NA, NA, NA, NA, 0x7200C, 4),
	[28] = PINGROUP(28, cci_timer1, mdp_vsync, mdp_vsync2_out,
			mdp_vsync3_out, NA, phase_flag22, qdss_cti, NA, NA, NA,
			NA, 0x7200C, 5),
	[29] = PINGROUP(29, cci_timer2, NA, phase_flag14, atest_char_start, NA,
			NA, NA, NA, NA, NA, NA, 0, -1),
	[30] = PINGROUP(30, cci_i2c_sda0, NA, gcc_gp2, ddr_bist_start,
			vsense_trigger_mirnat, atest_usb0, NA, NA, NA, NA, NA,
			0x7200C, 6),
	[31] = PINGROUP(31, cci_i2c_scl0, ddr_bist_stop, qdss_gpio8, NA, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[32] = PINGROUP(32, cci_i2c_sda1, NA, NA, cmu_rng3, wlan1_adc0,
			atest_usb02, NA, NA, NA, NA, NA, 0, -1),
	[33] = PINGROUP(33, cci_i2c_scl1, jitter_bist, ddr_pxi0, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[34] = PINGROUP(34, cci_i2c_sda2, cri_trng1, qdss_gpio3, NA, NA, NA, NA,
			NA, NA, NA, NA, 0x7200C, 7),
	[35] = PINGROUP(35, cci_i2c_scl2, NA, cmu_rng1, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[36] = PINGROUP(36, qup1_se0_l0, tgu_ch0_trigout, prng_rosc3,
			qdss_gpio4, NA, NA, NA, NA, NA, NA, NA, 0x72004, 3),
	[37] = PINGROUP(37, qup1_se0_l1, mpm_pwr, tgu_ch1_trigout, prng_rosc0,
			qdss_gpio5, NA, NA, NA, NA, NA, NA, 0, -1),
	[38] = PINGROUP(38, qup1_se0_l2, tgu_ch2_trigout, prng_rosc1,
			qdss_gpio3, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[39] = PINGROUP(39, qup1_se0_l3, tgu_ch3_trigout, prng_rosc2,
			qdss_gpio7, NA, NA, NA, NA, NA, NA, NA, 0x72004, 4),
	[40] = PINGROUP(40, qup1_se2_l0, uim1_data_mirb, NA, cri_trng0, NA, NA,
			NA, NA, NA, NA, NA, 0x72004, 5),
	[41] = PINGROUP(41, qup1_se2_l1, uim1_clk_mirb, NA, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[42] = PINGROUP(42, qup1_se2_l2, uim1_reset_mirb, NA, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[43] = PINGROUP(43, qup1_se2_l3, uim1_present_mirb, qdss_cti, NA, NA,
			NA, NA, NA, NA, NA, NA, 0x72004, 6),
	[44] = PINGROUP(44, uim0_data, NA, qdss_cti, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[45] = PINGROUP(45, uim0_clk, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[46] = PINGROUP(46, uim0_reset, pll_bist_sync, NA, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[47] = PINGROUP(47, uim0_present, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0x72014, 1),
	[48] = PINGROUP(48, uim1_data_mira, pll_bypassnl, NA, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[49] = PINGROUP(49, uim1_clk_mira, gcc_gp2, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[50] = PINGROUP(50, uim1_reset_mira, NA, gcc_gp3, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[51] = PINGROUP(51, uim1_present_mira, NA, gcc_gp1, qdss_cti, NA, NA,
			NA, NA, NA, NA, NA, 0x72014, 2),
	[52] = PINGROUP(52, NA, NA, NA, phase_flag23, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[53] = PINGROUP(53, NA, NA, NA, phase_flag26, NA, NA, NA, NA, NA, NA,
			NA, 0x72004, 7),
	[54] = PINGROUP(54, NA, NA, NA, phase_flag27, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[55] = PINGROUP(55, NA, NA, NA, phase_flag19, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[56] = PINGROUP(56, NA, NA, NA, phase_flag15, NA, NA, NA, NA, NA, NA,
			NA, 0x72004, 8),
	[57] = PINGROUP(57, NA, NA, tmess_prng0, ddr_pxi0, NA, NA, NA, NA, NA,
			NA, NA, 0x72004, 9),
	[58] = PINGROUP(58, NA, NA, NA, phase_flag28, tmess_prng1, NA, NA, NA,
			NA, NA, NA, 0x72004, 10),
	[59] = PINGROUP(59, NA, NA, NA, phase_flag29, tmess_prng2, NA, NA, NA,
			NA, NA, NA, 0x72004, 11),
	[60] = PINGROUP(60, NA, NA, NA, phase_flag30, tmess_prng3, NA, NA, NA,
			NA, NA, NA, 0x72004, 12),
	[61] = PINGROUP(61, NA, NA, NA, phase_flag18, NA, NA, NA, NA, NA, NA,
			NA, 0x72004, 13),
	[62] = PINGROUP(62, NA, NA, phase_flag17, atest_char3, NA, NA, NA, NA,
			NA, NA, NA, 0x72004, 14),
	[63] = PINGROUP(63, NA, NA, phase_flag13, atest_char2, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[64] = PINGROUP(64, nav_gpio0, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0, -1),
	[65] = PINGROUP(65, nav_gpio1, vfr_1, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0x72004, 15),
	[66] = PINGROUP(66, nav_gpio2, pa_indicator, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0x72014, 3),
	[67] = PINGROUP(67, NA, NA, phase_flag31, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[68] = PINGROUP(68, NA, NA, phase_flag21, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[69] = PINGROUP(69, qup1_se1_l2, vfr_0_mirb, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0x72008, 0),
	[70] = PINGROUP(70, qup1_se1_l3, mdp_vsync, NA, phase_flag16, NA, NA,
			NA, NA, NA, NA, NA, 0x72008, 1),
	[71] = PINGROUP(71, qup1_se4_l0, mdp_vsync, tsense_pwm1, tsense_pwm2,
			NA, NA, NA, NA, NA, NA, NA, 0x72008, 2),
	[72] = PINGROUP(72, qup1_se4_l1, NA, ddr_bist_complete, NA, NA, NA, NA,
			NA, NA, NA, NA, 0x72008, 3),
	[73] = PINGROUP(73, qup1_se4_l2, NA, ddr_bist_fail, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[74] = PINGROUP(74, qup1_se4_l3, adsp_ext, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0x72008, 4),
	[75] = PINGROUP(75, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0x72014,
			4),
	[76] = PINGROUP(76, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[77] = PINGROUP(77, qup0_se1_l3, qdss_gpio0, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0x72008, 5),
	[78] = PINGROUP(78, qup1_se1_l0, NA, phase_flag1, qdss_gpio1, NA, NA,
			NA, NA, NA, NA, NA, 0x72008, 6),
	[79] = PINGROUP(79, qup1_se1_l1, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0x72008, 7),
	[80] = PINGROUP(80, qup1_se3_l0, mdp_vsync, dbg_out_clk, cmu_rng2, NA,
			NA, NA, NA, NA, NA, NA, 0x72008, 8),
	[81] = PINGROUP(81, qup1_se3_l1, NA, cmu_rng0, qdss_gpio6, NA, NA, NA,
			NA, NA, NA, NA, 0x72008, 9),
	[82] = PINGROUP(82, qlink0_request, NA, atest_usb01, NA, NA, NA, NA, NA,
			NA, NA, NA, 0x72014, 5),
	[83] = PINGROUP(83, qlink0_enable, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[84] = PINGROUP(84, qlink0_wmss, NA, atest_usb00, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[85] = PINGROUP(85, NA, phase_flag24, atest_char0, NA, NA, NA, NA, NA,
			NA, NA, NA, 0x72008, 10),
	[86] = PINGROUP(86, mdp_vsync, NA, phase_flag25, atest_char1, NA, NA,
			NA, NA, NA, NA, NA, 0x72008, 11),
	[87] = PINGROUP(87, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, egpio, 0,
			-1),
	[88] = PINGROUP(88, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, egpio,
			0x7200C, 8),
	[89] = PINGROUP(89, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, egpio, 0,
			-1),
	[90] = PINGROUP(90, NA, qdss_cti, NA, NA, NA, NA, NA, NA, NA, NA, egpio,
			0, -1),
	[91] = PINGROUP(91, NA, gcc_gp1, NA, NA, NA, NA, NA, NA, NA, NA, egpio,
			0x7200C, 9),
	[92] = PINGROUP(92, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, egpio,
			0x7200C, 10),
	[93] = PINGROUP(93, qdss_gpio10, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			egpio, 0x7200C, 11),
	[94] = PINGROUP(94, qdss_gpio12, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			egpio, 0x7200C, 12),
	[95] = PINGROUP(95, qdss_gpio13, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			egpio, 0, -1),
	[96] = PINGROUP(96, qdss_gpio4, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			egpio, 0x7200C, 13),
	[97] = PINGROUP(97, NA, NA, ddr_pxi1, NA, NA, NA, NA, NA, NA, NA, egpio,
			0x7200C, 14),
	[98] = PINGROUP(98, NA, NA, ddr_pxi1, NA, NA, NA, NA, NA, NA, NA, egpio,
			0x7200C, 15),
	[99] = PINGROUP(99, NA, phase_flag20, NA, NA, NA, NA, NA, NA, NA, NA,
			egpio, 0x72010, 0),
	[100] = PINGROUP(100, qdss_gpio9, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 1),
	[101] = PINGROUP(101, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, egpio, 0,
			 -1),
	[102] = PINGROUP(102, tb_trig_sdc1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 2),
	[103] = PINGROUP(103, pll_clk_aux, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 3),
	[104] = PINGROUP(104, NA, qdss_gpio15, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 4),
	[105] = PINGROUP(105, qdss_gpio, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0, -1),
	[106] = PINGROUP(106, qdss_gpio2, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 5),
	[107] = PINGROUP(107, qdss_gpio1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 6),
	[108] = PINGROUP(108, qdss_gpio, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 7),
	[109] = PINGROUP(109, qdss_gpio7, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0, -1),
	[110] = PINGROUP(110, qdss_gpio6, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 8),
	[111] = PINGROUP(111, qdss_gpio0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 9),
	[112] = PINGROUP(112, qdss_gpio5, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0, -1),
	[113] = PINGROUP(113, qdss_gpio14, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 egpio, 0x72010, 10),
	[114] = UFS_RESET(ufs_reset, 0x181000),
	[115] = SDC_QDSD_PINGROUP(sdc2_clk, 0x179000, 14, 6),
	[116] = SDC_QDSD_PINGROUP(sdc2_cmd, 0x179000, 11, 3),
	[117] = SDC_QDSD_PINGROUP(sdc2_data, 0x179000, 9, 0),
};

static struct pinctrl_qup malabar_qup_regs[] = {
};

static const struct msm_gpio_wakeirq_map malabar_mpm_map[] = {
	{ 0, 84 }, { 3, 32 }, { 4, 39 }, { 6, 7 }, { 8, 11 }, { 9, 14 }, { 10, 55 },
	{ 11, 9 }, { 12, 47 }, { 13, 36 }, { 14, 24 }, { 15, 17 }, { 16, 18 },
	{ 17, 27 }, { 20, 25 }, { 21, 29 }, { 23, 30 }, { 26, 31 }, { 27, 75 },
	{ 28, 68 }, { 30, 43 }, { 34, 33 }, { 36, 85 }, { 39, 79 }, { 40, 40 },
	{ 43, 8 }, { 47, 69 }, { 51, 45 }, { 53, 48 }, { 56, 62 }, { 57, 46 },
	{ 58, 19 }, { 59, 51 }, { 60, 52 }, { 61, 70 }, { 62, 53 }, { 65, 35 },
	{ 66, 49 }, { 69, 54 }, { 70, 41 }, { 71, 50 }, { 72, 28 }, { 74, 56 },
	{ 75, 57 }, { 77, 42 }, { 78, 61 }, { 79, 76 }, { 80, 64 }, { 81, 78 },
	{ 82, 44 }, { 85, 60 }, { 86, 66 }, { 88, 77 }, { 91, 58 }, { 92, 20 },
	{ 93, 59 }, { 94, 12 }, { 96, 38 }, { 97, 21 }, { 98, 63 }, { 99, 71 },
	{ 100, 34 }, { 102, 65 }, { 103, 23 }, { 104, 37 }, { 106, 72 }, { 107, 67 },
	{ 108, 73 }, { 110, 16 }, { 111, 74 }, { 113, 26 },
};

static const struct msm_pinctrl_soc_data malabar_tlmm = {
	.pins = malabar_pins,
	.npins = ARRAY_SIZE(malabar_pins),
	.functions = malabar_functions,
	.nfunctions = ARRAY_SIZE(malabar_functions),
	.groups = malabar_groups,
	.ngroups = ARRAY_SIZE(malabar_groups),
	.ngpios = 115,
	.qup_regs = malabar_qup_regs,
	.nqup_regs = ARRAY_SIZE(malabar_qup_regs),
	.wakeirq_map = malabar_mpm_map,
	.nwakeirq_map = ARRAY_SIZE(malabar_mpm_map),
	.egpio_func = 11,
};

static const struct of_device_id malabar_tlmm_of_match[] = {
	{ .compatible = "qcom,malabar-pinctrl", .data = &malabar_tlmm },
	{},
};

static int malabar_tlmm_probe(struct platform_device *pdev)
{
	const struct msm_pinctrl_soc_data *pinctrl_data;
	struct device *dev = &pdev->dev;

	pinctrl_data = of_device_get_match_data(dev);
	if (!pinctrl_data)
		return -EINVAL;

	return msm_pinctrl_probe(pdev, pinctrl_data);
}

static struct platform_driver malabar_tlmm_driver = {
	.driver = {
		.name = "malabar-pinctrl",
		.of_match_table = malabar_tlmm_of_match,
	},
	.probe = malabar_tlmm_probe,
	.remove = msm_pinctrl_remove,
};

static int __init malabar_tlmm_init(void)
{
	return platform_driver_register(&malabar_tlmm_driver);
}
arch_initcall(malabar_tlmm_init);

static void __exit malabar_tlmm_exit(void)
{
	platform_driver_unregister(&malabar_tlmm_driver);
}
module_exit(malabar_tlmm_exit);

MODULE_DESCRIPTION("QTI malabar TLMM driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, malabar_tlmm_of_match);
