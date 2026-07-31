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
#define FUNCTION(fname)			                \
	[msm_mux_##fname] = {		                \
		.name = #fname,				\
		.groups = fname##_groups,               \
		.ngroups = ARRAY_SIZE(fname##_groups),	\
	}

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
		.intr_target_bit = 8,		\
		.intr_wakeup_enable_bit = 7,	\
		.intr_wakeup_present_bit = 6,	\
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
		.name = #pg_name,			\
		.pins = pg_name##_pins,			\
		.npins = (unsigned int)ARRAY_SIZE(pg_name##_pins),	\
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

#define QUP_I3C(qup_mode, qup_offset)			\
	{						\
		.mode = qup_mode,			\
		.offset = REG_BASE + qup_offset,			\
	}


static const struct pinctrl_pin_desc shikra_pins[] = {
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
	PINCTRL_PIN(114, "GPIO_114"),
	PINCTRL_PIN(115, "GPIO_115"),
	PINCTRL_PIN(116, "GPIO_116"),
	PINCTRL_PIN(117, "GPIO_117"),
	PINCTRL_PIN(118, "GPIO_118"),
	PINCTRL_PIN(119, "GPIO_119"),
	PINCTRL_PIN(120, "GPIO_120"),
	PINCTRL_PIN(121, "GPIO_121"),
	PINCTRL_PIN(122, "GPIO_122"),
	PINCTRL_PIN(123, "GPIO_123"),
	PINCTRL_PIN(124, "GPIO_124"),
	PINCTRL_PIN(125, "GPIO_125"),
	PINCTRL_PIN(126, "GPIO_126"),
	PINCTRL_PIN(127, "GPIO_127"),
	PINCTRL_PIN(128, "GPIO_128"),
	PINCTRL_PIN(129, "GPIO_129"),
	PINCTRL_PIN(130, "GPIO_130"),
	PINCTRL_PIN(131, "GPIO_131"),
	PINCTRL_PIN(132, "GPIO_132"),
	PINCTRL_PIN(133, "GPIO_133"),
	PINCTRL_PIN(134, "GPIO_134"),
	PINCTRL_PIN(135, "GPIO_135"),
	PINCTRL_PIN(136, "GPIO_136"),
	PINCTRL_PIN(137, "GPIO_137"),
	PINCTRL_PIN(138, "GPIO_138"),
	PINCTRL_PIN(139, "GPIO_139"),
	PINCTRL_PIN(140, "GPIO_140"),
	PINCTRL_PIN(141, "GPIO_141"),
	PINCTRL_PIN(142, "GPIO_142"),
	PINCTRL_PIN(143, "GPIO_143"),
	PINCTRL_PIN(144, "GPIO_144"),
	PINCTRL_PIN(145, "GPIO_145"),
	PINCTRL_PIN(146, "GPIO_146"),
	PINCTRL_PIN(147, "GPIO_147"),
	PINCTRL_PIN(148, "GPIO_148"),
	PINCTRL_PIN(149, "GPIO_149"),
	PINCTRL_PIN(150, "GPIO_150"),
	PINCTRL_PIN(151, "GPIO_151"),
	PINCTRL_PIN(152, "GPIO_152"),
	PINCTRL_PIN(153, "GPIO_153"),
	PINCTRL_PIN(154, "GPIO_154"),
	PINCTRL_PIN(155, "GPIO_155"),
	PINCTRL_PIN(156, "GPIO_156"),
	PINCTRL_PIN(157, "GPIO_157"),
	PINCTRL_PIN(158, "GPIO_158"),
	PINCTRL_PIN(159, "GPIO_159"),
	PINCTRL_PIN(160, "GPIO_160"),
	PINCTRL_PIN(161, "GPIO_161"),
	PINCTRL_PIN(162, "SDC1_DATA"),
	PINCTRL_PIN(163, "SDC1_RCLK"),
	PINCTRL_PIN(164, "SDC1_CMD"),
	PINCTRL_PIN(165, "SDC1_CLK"),
	PINCTRL_PIN(166, "SDC2_CLK"),
	PINCTRL_PIN(167, "SDC2_CMD"),
	PINCTRL_PIN(168, "SDC2_DATA"),
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
DECLARE_MSM_GPIO_PINS(114);
DECLARE_MSM_GPIO_PINS(115);
DECLARE_MSM_GPIO_PINS(116);
DECLARE_MSM_GPIO_PINS(117);
DECLARE_MSM_GPIO_PINS(118);
DECLARE_MSM_GPIO_PINS(119);
DECLARE_MSM_GPIO_PINS(120);
DECLARE_MSM_GPIO_PINS(121);
DECLARE_MSM_GPIO_PINS(122);
DECLARE_MSM_GPIO_PINS(123);
DECLARE_MSM_GPIO_PINS(124);
DECLARE_MSM_GPIO_PINS(125);
DECLARE_MSM_GPIO_PINS(126);
DECLARE_MSM_GPIO_PINS(127);
DECLARE_MSM_GPIO_PINS(128);
DECLARE_MSM_GPIO_PINS(129);
DECLARE_MSM_GPIO_PINS(130);
DECLARE_MSM_GPIO_PINS(131);
DECLARE_MSM_GPIO_PINS(132);
DECLARE_MSM_GPIO_PINS(133);
DECLARE_MSM_GPIO_PINS(134);
DECLARE_MSM_GPIO_PINS(135);
DECLARE_MSM_GPIO_PINS(136);
DECLARE_MSM_GPIO_PINS(137);
DECLARE_MSM_GPIO_PINS(138);
DECLARE_MSM_GPIO_PINS(139);
DECLARE_MSM_GPIO_PINS(140);
DECLARE_MSM_GPIO_PINS(141);
DECLARE_MSM_GPIO_PINS(142);
DECLARE_MSM_GPIO_PINS(143);
DECLARE_MSM_GPIO_PINS(144);
DECLARE_MSM_GPIO_PINS(145);
DECLARE_MSM_GPIO_PINS(146);
DECLARE_MSM_GPIO_PINS(147);
DECLARE_MSM_GPIO_PINS(148);
DECLARE_MSM_GPIO_PINS(149);
DECLARE_MSM_GPIO_PINS(150);
DECLARE_MSM_GPIO_PINS(151);
DECLARE_MSM_GPIO_PINS(152);
DECLARE_MSM_GPIO_PINS(153);
DECLARE_MSM_GPIO_PINS(154);
DECLARE_MSM_GPIO_PINS(155);
DECLARE_MSM_GPIO_PINS(156);
DECLARE_MSM_GPIO_PINS(157);
DECLARE_MSM_GPIO_PINS(158);
DECLARE_MSM_GPIO_PINS(159);
DECLARE_MSM_GPIO_PINS(160);
DECLARE_MSM_GPIO_PINS(161);
DECLARE_MSM_GPIO_PINS(162);
DECLARE_MSM_GPIO_PINS(163);
DECLARE_MSM_GPIO_PINS(164);
DECLARE_MSM_GPIO_PINS(165);

static const unsigned int sdc1_data_pins[] = { 162 };
static const unsigned int sdc1_rclk_pins[] = { 163 };
static const unsigned int sdc1_cmd_pins[] = { 164 };
static const unsigned int sdc1_clk_pins[] = { 165 };
static const unsigned int sdc2_clk_pins[] = { 166 };
static const unsigned int sdc2_cmd_pins[] = { 167 };
static const unsigned int sdc2_data_pins[] = { 168 };

enum shikra_functions {
	msm_mux_gpio,
	msm_mux_agera_pll,
	msm_mux_atest_bbrx0,
	msm_mux_atest_bbrx1,
	msm_mux_atest_char0,
	msm_mux_atest_char1,
	msm_mux_atest_char2,
	msm_mux_atest_char3,
	msm_mux_atest_char_start,
	msm_mux_atest_gpsadc,
	msm_mux_atest_tsens,
	msm_mux_atest_tsens2,
	msm_mux_atest_usb1,
	msm_mux_atest_usb10,
	msm_mux_atest_usb11,
	msm_mux_atest_usb12,
	msm_mux_atest_usb13,
	msm_mux_atest_usb2,
	msm_mux_atest_usb20,
	msm_mux_atest_usb21,
	msm_mux_atest_usb22,
	msm_mux_atest_usb23,
	msm_mux_cam_mclk,
	msm_mux_cci_async_in0,
	msm_mux_cci_i2c_scl0,
	msm_mux_cci_i2c_scl1,
	msm_mux_cci_i2c_sda0,
	msm_mux_cci_i2c_sda1,
	msm_mux_cci_timer0,
	msm_mux_cci_timer1,
	msm_mux_cci_timer2,
	msm_mux_cci_timer3,
	msm_mux_char_exec_pending,
	msm_mux_char_exec_release,
	msm_mux_cri_trng,
	msm_mux_cri_trng0,
	msm_mux_cri_trng1,
	msm_mux_dac_calib0,
	msm_mux_dac_calib1,
	msm_mux_dac_calib10,
	msm_mux_dac_calib11,
	msm_mux_dac_calib12,
	msm_mux_dac_calib13,
	msm_mux_dac_calib14,
	msm_mux_dac_calib15,
	msm_mux_dac_calib16,
	msm_mux_dac_calib17,
	msm_mux_dac_calib18,
	msm_mux_dac_calib19,
	msm_mux_dac_calib2,
	msm_mux_dac_calib20,
	msm_mux_dac_calib21,
	msm_mux_dac_calib22,
	msm_mux_dac_calib23,
	msm_mux_dac_calib24,
	msm_mux_dac_calib25,
	msm_mux_dac_calib3,
	msm_mux_dac_calib4,
	msm_mux_dac_calib5,
	msm_mux_dac_calib6,
	msm_mux_dac_calib7,
	msm_mux_dac_calib8,
	msm_mux_dac_calib9,
	msm_mux_dbg_out_clk,
	msm_mux_ddr_bist_complete,
	msm_mux_ddr_bist_fail,
	msm_mux_ddr_bist_start,
	msm_mux_ddr_bist_stop,
	msm_mux_ddr_pxi0,
	msm_mux_ddr_pxi1,
	msm_mux_dmic0_clk,
	msm_mux_dmic0_data,
	msm_mux_dmic1_clk,
	msm_mux_dmic1_data,
	msm_mux_emac0_dll,
	msm_mux_emac0_mcg0,
	msm_mux_emac0_mcg1,
	msm_mux_emac0_mcg2,
	msm_mux_emac0_mcg3,
	msm_mux_emac0_phy,
	msm_mux_emac0_ptp,
	msm_mux_emac1_dll,
	msm_mux_emac1_mcg0,
	msm_mux_emac1_mcg1,
	msm_mux_emac1_mcg2,
	msm_mux_emac1_mcg3,
	msm_mux_emac1_phy,
	msm_mux_emac1_ptp,
	msm_mux_ext_mclk,
	msm_mux_gcc_gp1,
	msm_mux_gcc_gp2,
	msm_mux_gcc_gp3,
	msm_mux_gsm0_tx,
	msm_mux_i2s0_clk,
	msm_mux_i2s0_data0,
	msm_mux_i2s0_data1,
	msm_mux_i2s0_data2,
	msm_mux_i2s0_data3,
	msm_mux_i2s0_ws,
	msm_mux_i2s1_clk,
	msm_mux_i2s1_data0,
	msm_mux_i2s1_data1,
	msm_mux_i2s1_ws,
	msm_mux_i2s2_clk,
	msm_mux_i2s2_data0,
	msm_mux_i2s2_data1,
	msm_mux_i2s2_ws,
	msm_mux_i2s3_clk,
	msm_mux_i2s3_data0,
	msm_mux_i2s3_data1,
	msm_mux_i2s3_ws,
	msm_mux_jitter_bist,
	msm_mux_m_voc,
	msm_mux_mdp_vsync,
	msm_mux_mdp_vsync_e,
	msm_mux_mdp_vsync_p,
	msm_mux_mdp_vsync_s,
	msm_mux_mpm_pwr,
	msm_mux_mss_lte,
	msm_mux_nav_gpio,
	msm_mux_pa_indicator_or,
	msm_mux_pbs_in0,
	msm_mux_pbs_in1,
	msm_mux_pbs_in10,
	msm_mux_pbs_in11,
	msm_mux_pbs_in12,
	msm_mux_pbs_in13,
	msm_mux_pbs_in14,
	msm_mux_pbs_in15,
	msm_mux_pbs_in2,
	msm_mux_pbs_in3,
	msm_mux_pbs_in4,
	msm_mux_pbs_in5,
	msm_mux_pbs_in6,
	msm_mux_pbs_in7,
	msm_mux_pbs_in8,
	msm_mux_pbs_in9,
	msm_mux_pbs_out_0,
	msm_mux_pbs_out_1,
	msm_mux_pbs_out_2,
	msm_mux_pcie0_clk_req_n,
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
	msm_mux_pll_bist,
	msm_mux_pll_bypassnl,
	msm_mux_pll_clk,
	msm_mux_pll_reset_n,
	msm_mux_prng_rosc,
	msm_mux_pwm_0_mira,
	msm_mux_pwm_0_mirb,
	msm_mux_pwm_1_mira,
	msm_mux_pwm_1_mirb,
	msm_mux_pwm_2_mira,
	msm_mux_pwm_2_mirb,
	msm_mux_pwm_3,
	msm_mux_pwm_4,
	msm_mux_pwm_5,
	msm_mux_pwm_6_mira,
	msm_mux_pwm_6_mirb,
	msm_mux_pwm_7_mira,
	msm_mux_pwm_7_mirb,
	msm_mux_pwm_8_mira,
	msm_mux_pwm_8_mirb,
	msm_mux_pwm_9_mira,
	msm_mux_pwm_9_mirb,
	msm_mux_qdss_cti,
	msm_mux_qup0_se0_l0,
	msm_mux_qup0_se0_l1,
	msm_mux_qup0_se0_l2,
	msm_mux_qup0_se0_l3,
	msm_mux_qup0_se1_l0,
	msm_mux_qup0_se1_l1,
	msm_mux_qup0_se1_l2_mira,
	msm_mux_qup0_se1_l2_mirb,
	msm_mux_qup0_se1_l3_mira,
	msm_mux_qup0_se1_l3_mirb,
	msm_mux_qup0_se2_l0,
	msm_mux_qup0_se2_l1,
	msm_mux_qup0_se2_l2,
	msm_mux_qup0_se2_l3,
	msm_mux_qup0_se2_l4,
	msm_mux_qup0_se2_l5,
	msm_mux_qup0_se3_l0,
	msm_mux_qup0_se3_l1,
	msm_mux_qup0_se3_l2,
	msm_mux_qup0_se3_l3,
	msm_mux_qup0_se4_l0,
	msm_mux_qup0_se4_l1,
	msm_mux_qup0_se4_l2,
	msm_mux_qup0_se4_l3,
	msm_mux_qup0_se5_l0,
	msm_mux_qup0_se5_l1,
	msm_mux_qup0_se5_l2,
	msm_mux_qup0_se5_l3,
	msm_mux_qup0_se6_l0,
	msm_mux_qup0_se6_l1,
	msm_mux_qup0_se6_l2,
	msm_mux_qup0_se6_l3,
	msm_mux_qup0_se6_l4,
	msm_mux_qup0_se6_l5,
	msm_mux_qup0_se7_l0,
	msm_mux_qup0_se7_l1,
	msm_mux_qup0_se7_l2,
	msm_mux_qup0_se7_l3,
	msm_mux_qup0_se8_l0,
	msm_mux_qup0_se8_l1,
	msm_mux_qup0_se8_l2,
	msm_mux_qup0_se8_l3,
	msm_mux_qup0_se9_l0_mira,
	msm_mux_qup0_se9_l0_mirb,
	msm_mux_qup0_se9_l1_mira,
	msm_mux_qup0_se9_l1_mirb,
	msm_mux_qup0_se9_l2_mira,
	msm_mux_qup0_se9_l2_mirb,
	msm_mux_qup0_se9_l3_mira,
	msm_mux_qup0_se9_l3_mirb,
	msm_mux_rgmii0_mdc,
	msm_mux_rgmii0_mdio,
	msm_mux_rgmii0_rx_ctl,
	msm_mux_rgmii0_rxc,
	msm_mux_rgmii0_rxd0,
	msm_mux_rgmii0_rxd1,
	msm_mux_rgmii0_rxd2,
	msm_mux_rgmii0_rxd3,
	msm_mux_rgmii0_tx_ctl,
	msm_mux_rgmii0_txc,
	msm_mux_rgmii0_txd0,
	msm_mux_rgmii0_txd1,
	msm_mux_rgmii0_txd2,
	msm_mux_rgmii0_txd3,
	msm_mux_rgmii1_mdc,
	msm_mux_rgmii1_mdio,
	msm_mux_rgmii1_rx_ctl,
	msm_mux_rgmii1_rxc,
	msm_mux_rgmii1_rxd0,
	msm_mux_rgmii1_rxd1,
	msm_mux_rgmii1_rxd2,
	msm_mux_rgmii1_rxd3,
	msm_mux_rgmii1_tx_ctl,
	msm_mux_rgmii1_txc,
	msm_mux_rgmii1_txd0,
	msm_mux_rgmii1_txd1,
	msm_mux_rgmii1_txd2,
	msm_mux_rgmii1_txd3,
	msm_mux_sd_write_protect,
	msm_mux_sdc1_tb_trig,
	msm_mux_sdc2_tb_trig,
	msm_mux_ssbi_wtr0,
	msm_mux_ssbi_wtr1,
	msm_mux_ssbi_wtr2,
	msm_mux_ssbi_wtr3,
	msm_mux_swr0_rx_clk,
	msm_mux_swr0_rx_data0,
	msm_mux_swr0_rx_data1,
	msm_mux_swr0_tx_clk,
	msm_mux_swr0_tx_data0,
	msm_mux_tgu_ch0_trigout,
	msm_mux_tgu_ch1_trigout,
	msm_mux_tgu_ch2_trigout,
	msm_mux_tgu_ch3_trigout,
	msm_mux_tsc_async,
	msm_mux_tsense_pwm,
	msm_mux_uim1_clk,
	msm_mux_uim1_data,
	msm_mux_uim1_present,
	msm_mux_uim1_reset,
	msm_mux_uim2_clk,
	msm_mux_uim2_data,
	msm_mux_uim2_present,
	msm_mux_uim2_reset,
	msm_mux_unused_adsp,
	msm_mux_unused_gsm1,
	msm_mux_usb0_phy_ps,
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
	"gpio114", "gpio115", "gpio116", "gpio117", "gpio118", "gpio119",
	"gpio120", "gpio121", "gpio122", "gpio123", "gpio124", "gpio125",
	"gpio126", "gpio127", "gpio128", "gpio129", "gpio130", "gpio131",
	"gpio132", "gpio133", "gpio134", "gpio135", "gpio136", "gpio137",
	"gpio138", "gpio139", "gpio140", "gpio141", "gpio142", "gpio143",
	"gpio144", "gpio145", "gpio146", "gpio147", "gpio148", "gpio149",
	"gpio150", "gpio151", "gpio152", "gpio153", "gpio154", "gpio155",
	"gpio156", "gpio157", "gpio158", "gpio159", "gpio160", "gpio161",
	"gpio162", "gpio163", "gpio164", "gpio165",
};

static const char *const agera_pll_groups[] = {
	"gpio22", "gpio23",
};

static const char *const atest_bbrx0_groups[] = {
	"gpio58",
};

static const char *const atest_bbrx1_groups[] = {
	"gpio59",
};

static const char *const atest_char0_groups[] = {
	"gpio56",
};

static const char *const atest_char1_groups[] = {
	"gpio57",
};

static const char *const atest_char2_groups[] = {
	"gpio54",
};

static const char *const atest_char3_groups[] = {
	"gpio55",
};

static const char *const atest_char_start_groups[] = {
	"gpio62",
};

static const char *const atest_gpsadc_groups[] = {
	"gpio60", "gpio96",
};

static const char *const atest_tsens_groups[] = {
	"gpio1",
};

static const char *const atest_tsens2_groups[] = {
	"gpio2",
};

static const char *const atest_usb1_groups[] = {
	"gpio96",
};

static const char *const atest_usb10_groups[] = {
	"gpio98",
};

static const char *const atest_usb11_groups[] = {
	"gpio99",
};

static const char *const atest_usb12_groups[] = {
	"gpio100",
};

static const char *const atest_usb13_groups[] = {
	"gpio101",
};

static const char *const atest_usb2_groups[] = {
	"gpio60",
};

static const char *const atest_usb20_groups[] = {
	"gpio61",
};

static const char *const atest_usb21_groups[] = {
	"gpio53",
};

static const char *const atest_usb22_groups[] = {
	"gpio58",
};

static const char *const atest_usb23_groups[] = {
	"gpio59",
};

static const char *const cam_mclk_groups[] = {
	"gpio34", "gpio35", "gpio96", "gpio98",
};

static const char *const cci_async_in0_groups[] = {
	"gpio39",
};

static const char *const cci_i2c_scl0_groups[] = {
	"gpio37",
};

static const char *const cci_i2c_scl1_groups[] = {
	"gpio42",
};

static const char *const cci_i2c_sda0_groups[] = {
	"gpio36",
};

static const char *const cci_i2c_sda1_groups[] = {
	"gpio41",
};

static const char *const cci_timer0_groups[] = {
	"gpio38",
};

static const char *const cci_timer1_groups[] = {
	"gpio47",
};

static const char *const cci_timer2_groups[] = {
	"gpio40",
};

static const char *const cci_timer3_groups[] = {
	"gpio43",
};

static const char *const char_exec_pending_groups[] = {
	"gpio13",
};

static const char *const char_exec_release_groups[] = {
	"gpio12",
};

static const char *const cri_trng_groups[] = {
	"gpio20",
};

static const char *const cri_trng0_groups[] = {
	"gpio6",
};

static const char *const cri_trng1_groups[] = {
	"gpio7",
};

static const char *const dac_calib0_groups[] = {
	"gpio3",
};

static const char *const dac_calib1_groups[] = {
	"gpio4",
};

static const char *const dac_calib10_groups[] = {
	"gpio17",
};

static const char *const dac_calib11_groups[] = {
	"gpio18",
};

static const char *const dac_calib12_groups[] = {
	"gpio19",
};

static const char *const dac_calib13_groups[] = {
	"gpio64",
};

static const char *const dac_calib14_groups[] = {
	"gpio66",
};

static const char *const dac_calib15_groups[] = {
	"gpio63",
};

static const char *const dac_calib16_groups[] = {
	"gpio68",
};

static const char *const dac_calib17_groups[] = {
	"gpio69",
};

static const char *const dac_calib18_groups[] = {
	"gpio70",
};

static const char *const dac_calib19_groups[] = {
	"gpio89",
};

static const char *const dac_calib2_groups[] = {
	"gpio5",
};

static const char *const dac_calib20_groups[] = {
	"gpio90",
};

static const char *const dac_calib21_groups[] = {
	"gpio88",
};

static const char *const dac_calib22_groups[] = {
	"gpio117",
};

static const char *const dac_calib23_groups[] = {
	"gpio97",
};

static const char *const dac_calib24_groups[] = {
	"gpio118",
};

static const char *const dac_calib25_groups[] = {
	"gpio116",
};

static const char *const dac_calib3_groups[] = {
	"gpio6",
};

static const char *const dac_calib4_groups[] = {
	"gpio7",
};

static const char *const dac_calib5_groups[] = {
	"gpio8",
};

static const char *const dac_calib6_groups[] = {
	"gpio9",
};

static const char *const dac_calib7_groups[] = {
	"gpio14",
};

static const char *const dac_calib8_groups[] = {
	"gpio15",
};

static const char *const dac_calib9_groups[] = {
	"gpio16",
};

static const char *const dbg_out_clk_groups[] = {
	"gpio61",
};

static const char *const ddr_bist_complete_groups[] = {
	"gpio4",
};

static const char *const ddr_bist_fail_groups[] = {
	"gpio1",
};

static const char *const ddr_bist_start_groups[] = {
	"gpio2",
};

static const char *const ddr_bist_stop_groups[] = {
	"gpio3",
};

static const char *const ddr_pxi0_groups[] = {
	"gpio98", "gpio99",
};

static const char *const ddr_pxi1_groups[] = {
	"gpio100", "gpio101",
};

static const char *const dmic0_clk_groups[] = {
	"gpio96",
};

static const char *const dmic0_data_groups[] = {
	"gpio97",
};

static const char *const dmic1_clk_groups[] = {
	"gpio98",
};

static const char *const dmic1_data_groups[] = {
	"gpio99",
};

static const char *const emac0_dll_groups[] = {
	"gpio60", "gpio61",
};

static const char *const emac0_mcg0_groups[] = {
	"gpio46",
};

static const char *const emac0_mcg1_groups[] = {
	"gpio45",
};

static const char *const emac0_mcg2_groups[] = {
	"gpio44",
};

static const char *const emac0_mcg3_groups[] = {
	"gpio43",
};

static const char *const emac0_phy_groups[] = {
	"gpio120",
};

static const char *const emac0_ptp_groups[] = {
	"gpio60", "gpio63", "gpio69", "gpio85",
};

static const char *const emac1_dll_groups[] = {
	"gpio58", "gpio59",
};

static const char *const emac1_mcg0_groups[] = {
	"gpio29",
};

static const char *const emac1_mcg1_groups[] = {
	"gpio28",
};

static const char *const emac1_mcg2_groups[] = {
	"gpio40",
};

static const char *const emac1_mcg3_groups[] = {
	"gpio47",
};

static const char *const emac1_phy_groups[] = {
	"gpio136",
};

static const char *const emac1_ptp_groups[] = {
	"gpio31", "gpio33", "gpio60", "gpio68",
};

static const char *const ext_mclk_groups[] = {
	"gpio103", "gpio104", "gpio110", "gpio114",
};

static const char *const gcc_gp1_groups[] = {
	"gpio45", "gpio110",
};

static const char *const gcc_gp2_groups[] = {
	"gpio61", "gpio88",
};

static const char *const gcc_gp3_groups[] = {
	"gpio53", "gpio89",
};

static const char *const gsm0_tx_groups[] = {
	"gpio75",
};

static const char *const i2s0_clk_groups[] = {
	"gpio105",
};

static const char *const i2s0_data0_groups[] = {
	"gpio107",
};

static const char *const i2s0_data1_groups[] = {
	"gpio108",
};

static const char *const i2s0_data2_groups[] = {
	"gpio109",
};

static const char *const i2s0_data3_groups[] = {
	"gpio110",
};

static const char *const i2s0_ws_groups[] = {
	"gpio106",
};

static const char *const i2s1_clk_groups[] = {
	"gpio96",
};

static const char *const i2s1_data0_groups[] = {
	"gpio98",
};

static const char *const i2s1_data1_groups[] = {
	"gpio99",
};

static const char *const i2s1_ws_groups[] = {
	"gpio97",
};

static const char *const i2s2_clk_groups[] = {
	"gpio100",
};

static const char *const i2s2_data0_groups[] = {
	"gpio102",
};

static const char *const i2s2_data1_groups[] = {
	"gpio103",
};

static const char *const i2s2_ws_groups[] = {
	"gpio101",
};

static const char *const i2s3_clk_groups[] = {
	"gpio111",
};

static const char *const i2s3_data0_groups[] = {
	"gpio113",
};

static const char *const i2s3_data1_groups[] = {
	"gpio114",
};

static const char *const i2s3_ws_groups[] = {
	"gpio112",
};

static const char *const jitter_bist_groups[] = {
	"gpio96", "gpio99",
};

static const char *const m_voc_groups[] = {
	"gpio0",
};

static const char *const mdp_vsync_groups[] = {
	"gpio86",
};

static const char *const mdp_vsync_e_groups[] = {
	"gpio94",
};

static const char *const mdp_vsync_p_groups[] = {
	"gpio86",
};

static const char *const mdp_vsync_s_groups[] = {
	"gpio95",
};

static const char *const mpm_pwr_groups[] = {
	"gpio1",
};

static const char *const mss_lte_groups[] = {
	"gpio115", "gpio116",
};

static const char *const nav_gpio_groups[] = {
	"gpio53", "gpio58", "gpio63", "gpio71", "gpio91", "gpio92",
	"gpio95", "gpio100", "gpio101", "gpio104",
};

static const char *const pa_indicator_or_groups[] = {
	"gpio61",
};

static const char *const pbs_in0_groups[] = {
	"gpio49",
};

static const char *const pbs_in1_groups[] = {
	"gpio50",
};

static const char *const pbs_in10_groups[] = {
	"gpio59",
};

static const char *const pbs_in11_groups[] = {
	"gpio60",
};

static const char *const pbs_in12_groups[] = {
	"gpio61",
};

static const char *const pbs_in13_groups[] = {
	"gpio62",
};

static const char *const pbs_in14_groups[] = {
	"gpio63",
};

static const char *const pbs_in15_groups[] = {
	"gpio51",
};

static const char *const pbs_in2_groups[] = {
	"gpio48",
};

static const char *const pbs_in3_groups[] = {
	"gpio74",
};

static const char *const pbs_in4_groups[] = {
	"gpio53",
};

static const char *const pbs_in5_groups[] = {
	"gpio54",
};

static const char *const pbs_in6_groups[] = {
	"gpio55",
};

static const char *const pbs_in7_groups[] = {
	"gpio56",
};

static const char *const pbs_in8_groups[] = {
	"gpio57",
};

static const char *const pbs_in9_groups[] = {
	"gpio58",
};

static const char *const pbs_out_0_groups[] = {
	"gpio24",
};

static const char *const pbs_out_1_groups[] = {
	"gpio22",
};

static const char *const pbs_out_2_groups[] = {
	"gpio23",
};

static const char *const pcie0_clk_req_n_groups[] = {
	"gpio117",
};

static const char *const phase_flag0_groups[] = {
	"gpio1",
};

static const char *const phase_flag1_groups[] = {
	"gpio2",
};

static const char *const phase_flag10_groups[] = {
	"gpio56",
};

static const char *const phase_flag11_groups[] = {
	"gpio11",
};

static const char *const phase_flag12_groups[] = {
	"gpio62",
};

static const char *const phase_flag13_groups[] = {
	"gpio63",
};

static const char *const phase_flag14_groups[] = {
	"gpio28",
};

static const char *const phase_flag15_groups[] = {
	"gpio69",
};

static const char *const phase_flag16_groups[] = {
	"gpio70",
};

static const char *const phase_flag17_groups[] = {
	"gpio71",
};

static const char *const phase_flag18_groups[] = {
	"gpio72",
};

static const char *const phase_flag19_groups[] = {
	"gpio74",
};

static const char *const phase_flag2_groups[] = {
	"gpio3",
};

static const char *const phase_flag20_groups[] = {
	"gpio57",
};

static const char *const phase_flag21_groups[] = {
	"gpio31",
};

static const char *const phase_flag22_groups[] = {
	"gpio4",
};

static const char *const phase_flag23_groups[] = {
	"gpio5",
};

static const char *const phase_flag24_groups[] = {
	"gpio6",
};

static const char *const phase_flag25_groups[] = {
	"gpio7",
};

static const char *const phase_flag26_groups[] = {
	"gpio8",
};

static const char *const phase_flag27_groups[] = {
	"gpio9",
};

static const char *const phase_flag28_groups[] = {
	"gpio29",
};

static const char *const phase_flag29_groups[] = {
	"gpio30",
};

static const char *const phase_flag3_groups[] = {
	"gpio54",
};

static const char *const phase_flag30_groups[] = {
	"gpio16",
};

static const char *const phase_flag31_groups[] = {
	"gpio17",
};

static const char *const phase_flag4_groups[] = {
	"gpio102",
};

static const char *const phase_flag5_groups[] = {
	"gpio55",
};

static const char *const phase_flag6_groups[] = {
	"gpio0",
};

static const char *const phase_flag7_groups[] = {
	"gpio48",
};

static const char *const phase_flag8_groups[] = {
	"gpio49",
};

static const char *const phase_flag9_groups[] = {
	"gpio50",
};

static const char *const pll_bist_groups[] = {
	"gpio43", "gpio44",
};

static const char *const pll_bypassnl_groups[] = {
	"gpio76",
};

static const char *const pll_clk_groups[] = {
	"gpio14", "gpio22",
};

static const char *const pll_reset_n_groups[] = {
	"gpio74",
};

static const char *const prng_rosc_groups[] = {
	"gpio27", "gpio28",
};

static const char *const pwm_0_mira_groups[] = {
	"gpio32",
};

static const char *const pwm_0_mirb_groups[] = {
	"gpio55",
};

static const char *const pwm_1_mira_groups[] = {
	"gpio40",
};

static const char *const pwm_1_mirb_groups[] = {
	"gpio54",
};

static const char *const pwm_2_mira_groups[] = {
	"gpio62",
};

static const char *const pwm_2_mirb_groups[] = {
	"gpio57",
};

static const char *const pwm_3_groups[] = {
	"gpio77",
};

static const char *const pwm_4_groups[] = {
	"gpio79",
};

static const char *const pwm_5_groups[] = {
	"gpio80",
};

static const char *const pwm_6_mira_groups[] = {
	"gpio87",
};

static const char *const pwm_6_mirb_groups[] = {
	"gpio56",
};

static const char *const pwm_7_mira_groups[] = {
	"gpio45",
};

static const char *const pwm_7_mirb_groups[] = {
	"gpio53",
};

static const char *const pwm_8_mira_groups[] = {
	"gpio102",
};

static const char *const pwm_8_mirb_groups[] = {
	"gpio58",
};

static const char *const pwm_9_mira_groups[] = {
	"gpio61",
};

static const char *const pwm_9_mirb_groups[] = {
	"gpio68",
};

static const char *const qdss_cti_groups[] = {
	"gpio28", "gpio29", "gpio30", "gpio31", "gpio94", "gpio95",
};

static const char *const qup0_se0_l0_groups[] = {
	"gpio2",
};

static const char *const qup0_se0_l1_groups[] = {
	"gpio3",
};

static const char *const qup0_se0_l2_groups[] = {
	"gpio0",
};

static const char *const qup0_se0_l3_groups[] = {
	"gpio1",
};

static const char *const qup0_se1_l0_groups[] = {
	"gpio5",
};

static const char *const qup0_se1_l1_groups[] = {
	"gpio4",
};

static const char *const qup0_se1_l2_mira_groups[] = {
	"gpio4",
};

static const char *const qup0_se1_l2_mirb_groups[] = {
	"gpio28",
};

static const char *const qup0_se1_l3_mira_groups[] = {
	"gpio5",
};

static const char *const qup0_se1_l3_mirb_groups[] = {
	"gpio29",
};

static const char *const qup0_se2_l0_groups[] = {
	"gpio6",
};

static const char *const qup0_se2_l1_groups[] = {
	"gpio7",
};

static const char *const qup0_se2_l2_groups[] = {
	"gpio8",
};

static const char *const qup0_se2_l3_groups[] = {
	"gpio9",
};

static const char *const qup0_se2_l4_groups[] = {
	"gpio30",
};

static const char *const qup0_se2_l5_groups[] = {
	"gpio31",
};

static const char *const qup0_se3_l0_groups[] = {
	"gpio10",
};

static const char *const qup0_se3_l1_groups[] = {
	"gpio11",
};

static const char *const qup0_se3_l2_groups[] = {
	"gpio11",
};

static const char *const qup0_se3_l3_groups[] = {
	"gpio10",
};

static const char *const qup0_se4_l0_groups[] = {
	"gpio12",
};

static const char *const qup0_se4_l1_groups[] = {
	"gpio13",
};

static const char *const qup0_se4_l2_groups[] = {
	"gpio13",
};

static const char *const qup0_se4_l3_groups[] = {
	"gpio12",
};

static const char *const qup0_se5_l0_groups[] = {
	"gpio14",
};

static const char *const qup0_se5_l1_groups[] = {
	"gpio15",
};

static const char *const qup0_se5_l2_groups[] = {
	"gpio16",
};

static const char *const qup0_se5_l3_groups[] = {
	"gpio17",
};

static const char *const qup0_se6_l0_groups[] = {
	"gpio18",
};

static const char *const qup0_se6_l1_groups[] = {
	"gpio19",
};

static const char *const qup0_se6_l2_groups[] = {
	"gpio28",
};

static const char *const qup0_se6_l3_groups[] = {
	"gpio29",
};

static const char *const qup0_se6_l4_groups[] = {
	"gpio30",
};

static const char *const qup0_se6_l5_groups[] = {
	"gpio31",
};

static const char *const qup0_se7_l0_groups[] = {
	"gpio20",
};

static const char *const qup0_se7_l1_groups[] = {
	"gpio21",
};

static const char *const qup0_se7_l2_groups[] = {
	"gpio21",
};

static const char *const qup0_se7_l3_groups[] = {
	"gpio20",
};

static const char *const qup0_se8_l0_groups[] = {
	"gpio22",
};

static const char *const qup0_se8_l1_groups[] = {
	"gpio23",
};

static const char *const qup0_se8_l2_groups[] = {
	"gpio24",
};

static const char *const qup0_se8_l3_groups[] = {
	"gpio25",
};

static const char *const qup0_se9_l0_mira_groups[] = {
	"gpio27",
};

static const char *const qup0_se9_l0_mirb_groups[] = {
	"gpio48",
};

static const char *const qup0_se9_l1_mira_groups[] = {
	"gpio26",
};

static const char *const qup0_se9_l1_mirb_groups[] = {
	"gpio49",
};

static const char *const qup0_se9_l2_mira_groups[] = {
	"gpio26",
};

static const char *const qup0_se9_l2_mirb_groups[] = {
	"gpio50",
};

static const char *const qup0_se9_l3_mira_groups[] = {
	"gpio27",
};

static const char *const qup0_se9_l3_mirb_groups[] = {
	"gpio51",
};

static const char *const rgmii0_mdc_groups[] = {
	"gpio134",
};

static const char *const rgmii0_mdio_groups[] = {
	"gpio133",
};

static const char *const rgmii0_rx_ctl_groups[] = {
	"gpio122",
};

static const char *const rgmii0_rxc_groups[] = {
	"gpio121",
};

static const char *const rgmii0_rxd0_groups[] = {
	"gpio123",
};

static const char *const rgmii0_rxd1_groups[] = {
	"gpio124",
};

static const char *const rgmii0_rxd2_groups[] = {
	"gpio125",
};

static const char *const rgmii0_rxd3_groups[] = {
	"gpio126",
};

static const char *const rgmii0_tx_ctl_groups[] = {
	"gpio128",
};

static const char *const rgmii0_txc_groups[] = {
	"gpio127",
};

static const char *const rgmii0_txd0_groups[] = {
	"gpio129",
};

static const char *const rgmii0_txd1_groups[] = {
	"gpio130",
};

static const char *const rgmii0_txd2_groups[] = {
	"gpio131",
};

static const char *const rgmii0_txd3_groups[] = {
	"gpio132",
};

static const char *const rgmii1_mdc_groups[] = {
	"gpio150",
};

static const char *const rgmii1_mdio_groups[] = {
	"gpio149",
};

static const char *const rgmii1_rx_ctl_groups[] = {
	"gpio138",
};

static const char *const rgmii1_rxc_groups[] = {
	"gpio137",
};

static const char *const rgmii1_rxd0_groups[] = {
	"gpio139",
};

static const char *const rgmii1_rxd1_groups[] = {
	"gpio140",
};

static const char *const rgmii1_rxd2_groups[] = {
	"gpio141",
};

static const char *const rgmii1_rxd3_groups[] = {
	"gpio142",
};

static const char *const rgmii1_tx_ctl_groups[] = {
	"gpio144",
};

static const char *const rgmii1_txc_groups[] = {
	"gpio143",
};

static const char *const rgmii1_txd0_groups[] = {
	"gpio145",
};

static const char *const rgmii1_txd1_groups[] = {
	"gpio146",
};

static const char *const rgmii1_txd2_groups[] = {
	"gpio147",
};

static const char *const rgmii1_txd3_groups[] = {
	"gpio148",
};

static const char *const sd_write_protect_groups[] = {
	"gpio109",
};

static const char *const sdc1_tb_trig_groups[] = {
	"gpio33",
};

static const char *const sdc2_tb_trig_groups[] = {
	"gpio32",
};

static const char *const ssbi_wtr0_groups[] = {
	"gpio70",
};

static const char *const ssbi_wtr1_groups[] = {
	"gpio71",
};

static const char *const ssbi_wtr2_groups[] = {
	"gpio68",
};

static const char *const ssbi_wtr3_groups[] = {
	"gpio69",
};

static const char *const swr0_rx_clk_groups[] = {
	"gpio107",
};

static const char *const swr0_rx_data0_groups[] = {
	"gpio108",
};

static const char *const swr0_rx_data1_groups[] = {
	"gpio109",
};

static const char *const swr0_tx_clk_groups[] = {
	"gpio105",
};

static const char *const swr0_tx_data0_groups[] = {
	"gpio106",
};

static const char *const tgu_ch0_trigout_groups[] = {
	"gpio14",
};

static const char *const tgu_ch1_trigout_groups[] = {
	"gpio15",
};

static const char *const tgu_ch2_trigout_groups[] = {
	"gpio16",
};

static const char *const tgu_ch3_trigout_groups[] = {
	"gpio17",
};

static const char *const tsc_async_groups[] = {
	"gpio45", "gpio46",
};

static const char *const tsense_pwm_groups[] = {
	"gpio21",
};

static const char *const uim1_clk_groups[] = {
	"gpio82",
};

static const char *const uim1_data_groups[] = {
	"gpio81",
};

static const char *const uim1_present_groups[] = {
	"gpio84",
};

static const char *const uim1_reset_groups[] = {
	"gpio83",
};

static const char *const uim2_clk_groups[] = {
	"gpio78",
};

static const char *const uim2_data_groups[] = {
	"gpio77",
};

static const char *const uim2_present_groups[] = {
	"gpio80",
};

static const char *const uim2_reset_groups[] = {
	"gpio79",
};

static const char *const unused_adsp_groups[] = {
	"gpio35",
};

static const char *const unused_gsm1_groups[] = {
	"gpio64",
};

static const char *const usb0_phy_ps_groups[] = {
	"gpio90",
};

static const char *const vfr_1_groups[] = {
	"gpio59",
};

static const char *const vsense_trigger_mirnat_groups[] = {
	"gpio58",
};

static const char *const wlan1_adc0_groups[] = {
	"gpio14",
};

static const char *const wlan1_adc1_groups[] = {
	"gpio15",
};


static const struct pinfunction shikra_functions[] = {
	MSM_PIN_FUNCTION(gpio),
	FUNCTION(agera_pll),
	FUNCTION(atest_bbrx0),
	FUNCTION(atest_bbrx1),
	FUNCTION(atest_char0),
	FUNCTION(atest_char1),
	FUNCTION(atest_char2),
	FUNCTION(atest_char3),
	FUNCTION(atest_char_start),
	FUNCTION(atest_gpsadc),
	FUNCTION(atest_tsens),
	FUNCTION(atest_tsens2),
	FUNCTION(atest_usb1),
	FUNCTION(atest_usb10),
	FUNCTION(atest_usb11),
	FUNCTION(atest_usb12),
	FUNCTION(atest_usb13),
	FUNCTION(atest_usb2),
	FUNCTION(atest_usb20),
	FUNCTION(atest_usb21),
	FUNCTION(atest_usb22),
	FUNCTION(atest_usb23),
	FUNCTION(cam_mclk),
	FUNCTION(cci_async_in0),
	FUNCTION(cci_i2c_scl0),
	FUNCTION(cci_i2c_scl1),
	FUNCTION(cci_i2c_sda0),
	FUNCTION(cci_i2c_sda1),
	FUNCTION(cci_timer0),
	FUNCTION(cci_timer1),
	FUNCTION(cci_timer2),
	FUNCTION(cci_timer3),
	FUNCTION(char_exec_pending),
	FUNCTION(char_exec_release),
	FUNCTION(cri_trng),
	FUNCTION(cri_trng0),
	FUNCTION(cri_trng1),
	FUNCTION(dac_calib0),
	FUNCTION(dac_calib1),
	FUNCTION(dac_calib10),
	FUNCTION(dac_calib11),
	FUNCTION(dac_calib12),
	FUNCTION(dac_calib13),
	FUNCTION(dac_calib14),
	FUNCTION(dac_calib15),
	FUNCTION(dac_calib16),
	FUNCTION(dac_calib17),
	FUNCTION(dac_calib18),
	FUNCTION(dac_calib19),
	FUNCTION(dac_calib2),
	FUNCTION(dac_calib20),
	FUNCTION(dac_calib21),
	FUNCTION(dac_calib22),
	FUNCTION(dac_calib23),
	FUNCTION(dac_calib24),
	FUNCTION(dac_calib25),
	FUNCTION(dac_calib3),
	FUNCTION(dac_calib4),
	FUNCTION(dac_calib5),
	FUNCTION(dac_calib6),
	FUNCTION(dac_calib7),
	FUNCTION(dac_calib8),
	FUNCTION(dac_calib9),
	FUNCTION(dbg_out_clk),
	FUNCTION(ddr_bist_complete),
	FUNCTION(ddr_bist_fail),
	FUNCTION(ddr_bist_start),
	FUNCTION(ddr_bist_stop),
	FUNCTION(ddr_pxi0),
	FUNCTION(ddr_pxi1),
	FUNCTION(dmic0_clk),
	FUNCTION(dmic0_data),
	FUNCTION(dmic1_clk),
	FUNCTION(dmic1_data),
	FUNCTION(emac0_dll),
	FUNCTION(emac0_mcg0),
	FUNCTION(emac0_mcg1),
	FUNCTION(emac0_mcg2),
	FUNCTION(emac0_mcg3),
	FUNCTION(emac0_phy),
	FUNCTION(emac0_ptp),
	FUNCTION(emac1_dll),
	FUNCTION(emac1_mcg0),
	FUNCTION(emac1_mcg1),
	FUNCTION(emac1_mcg2),
	FUNCTION(emac1_mcg3),
	FUNCTION(emac1_phy),
	FUNCTION(emac1_ptp),
	FUNCTION(ext_mclk),
	FUNCTION(gcc_gp1),
	FUNCTION(gcc_gp2),
	FUNCTION(gcc_gp3),
	FUNCTION(gsm0_tx),
	FUNCTION(i2s0_clk),
	FUNCTION(i2s0_data0),
	FUNCTION(i2s0_data1),
	FUNCTION(i2s0_data2),
	FUNCTION(i2s0_data3),
	FUNCTION(i2s0_ws),
	FUNCTION(i2s1_clk),
	FUNCTION(i2s1_data0),
	FUNCTION(i2s1_data1),
	FUNCTION(i2s1_ws),
	FUNCTION(i2s2_clk),
	FUNCTION(i2s2_data0),
	FUNCTION(i2s2_data1),
	FUNCTION(i2s2_ws),
	FUNCTION(i2s3_clk),
	FUNCTION(i2s3_data0),
	FUNCTION(i2s3_data1),
	FUNCTION(i2s3_ws),
	FUNCTION(jitter_bist),
	FUNCTION(m_voc),
	FUNCTION(mdp_vsync),
	FUNCTION(mdp_vsync_e),
	FUNCTION(mdp_vsync_p),
	FUNCTION(mdp_vsync_s),
	FUNCTION(mpm_pwr),
	FUNCTION(mss_lte),
	FUNCTION(nav_gpio),
	FUNCTION(pa_indicator_or),
	FUNCTION(pbs_in0),
	FUNCTION(pbs_in1),
	FUNCTION(pbs_in10),
	FUNCTION(pbs_in11),
	FUNCTION(pbs_in12),
	FUNCTION(pbs_in13),
	FUNCTION(pbs_in14),
	FUNCTION(pbs_in15),
	FUNCTION(pbs_in2),
	FUNCTION(pbs_in3),
	FUNCTION(pbs_in4),
	FUNCTION(pbs_in5),
	FUNCTION(pbs_in6),
	FUNCTION(pbs_in7),
	FUNCTION(pbs_in8),
	FUNCTION(pbs_in9),
	FUNCTION(pbs_out_0),
	FUNCTION(pbs_out_1),
	FUNCTION(pbs_out_2),
	FUNCTION(pcie0_clk_req_n),
	FUNCTION(phase_flag0),
	FUNCTION(phase_flag1),
	FUNCTION(phase_flag10),
	FUNCTION(phase_flag11),
	FUNCTION(phase_flag12),
	FUNCTION(phase_flag13),
	FUNCTION(phase_flag14),
	FUNCTION(phase_flag15),
	FUNCTION(phase_flag16),
	FUNCTION(phase_flag17),
	FUNCTION(phase_flag18),
	FUNCTION(phase_flag19),
	FUNCTION(phase_flag2),
	FUNCTION(phase_flag20),
	FUNCTION(phase_flag21),
	FUNCTION(phase_flag22),
	FUNCTION(phase_flag23),
	FUNCTION(phase_flag24),
	FUNCTION(phase_flag25),
	FUNCTION(phase_flag26),
	FUNCTION(phase_flag27),
	FUNCTION(phase_flag28),
	FUNCTION(phase_flag29),
	FUNCTION(phase_flag3),
	FUNCTION(phase_flag30),
	FUNCTION(phase_flag31),
	FUNCTION(phase_flag4),
	FUNCTION(phase_flag5),
	FUNCTION(phase_flag6),
	FUNCTION(phase_flag7),
	FUNCTION(phase_flag8),
	FUNCTION(phase_flag9),
	FUNCTION(pll_bist),
	FUNCTION(pll_bypassnl),
	FUNCTION(pll_clk),
	FUNCTION(pll_reset_n),
	FUNCTION(prng_rosc),
	FUNCTION(pwm_0_mira),
	FUNCTION(pwm_0_mirb),
	FUNCTION(pwm_1_mira),
	FUNCTION(pwm_1_mirb),
	FUNCTION(pwm_2_mira),
	FUNCTION(pwm_2_mirb),
	FUNCTION(pwm_3),
	FUNCTION(pwm_4),
	FUNCTION(pwm_5),
	FUNCTION(pwm_6_mira),
	FUNCTION(pwm_6_mirb),
	FUNCTION(pwm_7_mira),
	FUNCTION(pwm_7_mirb),
	FUNCTION(pwm_8_mira),
	FUNCTION(pwm_8_mirb),
	FUNCTION(pwm_9_mira),
	FUNCTION(pwm_9_mirb),
	FUNCTION(qdss_cti),
	FUNCTION(qup0_se0_l0),
	FUNCTION(qup0_se0_l1),
	FUNCTION(qup0_se0_l2),
	FUNCTION(qup0_se0_l3),
	FUNCTION(qup0_se1_l0),
	FUNCTION(qup0_se1_l1),
	FUNCTION(qup0_se1_l2_mira),
	FUNCTION(qup0_se1_l2_mirb),
	FUNCTION(qup0_se1_l3_mira),
	FUNCTION(qup0_se1_l3_mirb),
	FUNCTION(qup0_se2_l0),
	FUNCTION(qup0_se2_l1),
	FUNCTION(qup0_se2_l2),
	FUNCTION(qup0_se2_l3),
	FUNCTION(qup0_se2_l4),
	FUNCTION(qup0_se2_l5),
	FUNCTION(qup0_se3_l0),
	FUNCTION(qup0_se3_l1),
	FUNCTION(qup0_se3_l2),
	FUNCTION(qup0_se3_l3),
	FUNCTION(qup0_se4_l0),
	FUNCTION(qup0_se4_l1),
	FUNCTION(qup0_se4_l2),
	FUNCTION(qup0_se4_l3),
	FUNCTION(qup0_se5_l0),
	FUNCTION(qup0_se5_l1),
	FUNCTION(qup0_se5_l2),
	FUNCTION(qup0_se5_l3),
	FUNCTION(qup0_se6_l0),
	FUNCTION(qup0_se6_l1),
	FUNCTION(qup0_se6_l2),
	FUNCTION(qup0_se6_l3),
	FUNCTION(qup0_se6_l4),
	FUNCTION(qup0_se6_l5),
	FUNCTION(qup0_se7_l0),
	FUNCTION(qup0_se7_l1),
	FUNCTION(qup0_se7_l2),
	FUNCTION(qup0_se7_l3),
	FUNCTION(qup0_se8_l0),
	FUNCTION(qup0_se8_l1),
	FUNCTION(qup0_se8_l2),
	FUNCTION(qup0_se8_l3),
	FUNCTION(qup0_se9_l0_mira),
	FUNCTION(qup0_se9_l0_mirb),
	FUNCTION(qup0_se9_l1_mira),
	FUNCTION(qup0_se9_l1_mirb),
	FUNCTION(qup0_se9_l2_mira),
	FUNCTION(qup0_se9_l2_mirb),
	FUNCTION(qup0_se9_l3_mira),
	FUNCTION(qup0_se9_l3_mirb),
	FUNCTION(rgmii0_mdc),
	FUNCTION(rgmii0_mdio),
	FUNCTION(rgmii0_rx_ctl),
	FUNCTION(rgmii0_rxc),
	FUNCTION(rgmii0_rxd0),
	FUNCTION(rgmii0_rxd1),
	FUNCTION(rgmii0_rxd2),
	FUNCTION(rgmii0_rxd3),
	FUNCTION(rgmii0_tx_ctl),
	FUNCTION(rgmii0_txc),
	FUNCTION(rgmii0_txd0),
	FUNCTION(rgmii0_txd1),
	FUNCTION(rgmii0_txd2),
	FUNCTION(rgmii0_txd3),
	FUNCTION(rgmii1_mdc),
	FUNCTION(rgmii1_mdio),
	FUNCTION(rgmii1_rx_ctl),
	FUNCTION(rgmii1_rxc),
	FUNCTION(rgmii1_rxd0),
	FUNCTION(rgmii1_rxd1),
	FUNCTION(rgmii1_rxd2),
	FUNCTION(rgmii1_rxd3),
	FUNCTION(rgmii1_tx_ctl),
	FUNCTION(rgmii1_txc),
	FUNCTION(rgmii1_txd0),
	FUNCTION(rgmii1_txd1),
	FUNCTION(rgmii1_txd2),
	FUNCTION(rgmii1_txd3),
	FUNCTION(sd_write_protect),
	FUNCTION(sdc1_tb_trig),
	FUNCTION(sdc2_tb_trig),
	FUNCTION(ssbi_wtr0),
	FUNCTION(ssbi_wtr1),
	FUNCTION(ssbi_wtr2),
	FUNCTION(ssbi_wtr3),
	FUNCTION(swr0_rx_clk),
	FUNCTION(swr0_rx_data0),
	FUNCTION(swr0_rx_data1),
	FUNCTION(swr0_tx_clk),
	FUNCTION(swr0_tx_data0),
	FUNCTION(tgu_ch0_trigout),
	FUNCTION(tgu_ch1_trigout),
	FUNCTION(tgu_ch2_trigout),
	FUNCTION(tgu_ch3_trigout),
	FUNCTION(tsc_async),
	FUNCTION(tsense_pwm),
	FUNCTION(uim1_clk),
	FUNCTION(uim1_data),
	FUNCTION(uim1_present),
	FUNCTION(uim1_reset),
	FUNCTION(uim2_clk),
	FUNCTION(uim2_data),
	FUNCTION(uim2_present),
	FUNCTION(uim2_reset),
	FUNCTION(unused_adsp),
	FUNCTION(unused_gsm1),
	FUNCTION(usb0_phy_ps),
	FUNCTION(vfr_1),
	FUNCTION(vsense_trigger_mirnat),
	FUNCTION(wlan1_adc0),
	FUNCTION(wlan1_adc1),
};

/* Every pin is maintained as a single group, and missing or non-existing pin
 * would be maintained as dummy group to synchronize pin group index with
 * pin descriptor registered with pinctrl core.
 * Clients would not be able to request these dummy pin groups.
 */
static const struct msm_pingroup shikra_groups[] = {
	[0] = PINGROUP(0, qup0_se0_l2, m_voc, NA, phase_flag6, NA, NA, NA, NA,
		       NA, NA, NA, 0, -1),
	[1] = PINGROUP(1, qup0_se0_l3, mpm_pwr, ddr_bist_fail, NA, phase_flag0,
		       atest_tsens, NA, NA, NA, NA, NA, 0, -1),
	[2] = PINGROUP(2, qup0_se0_l0, ddr_bist_start, NA, phase_flag1,
		       atest_tsens2, NA, NA, NA, NA, NA, NA, 0, -1),
	[3] = PINGROUP(3, qup0_se0_l1, ddr_bist_stop, NA, phase_flag2,
		       dac_calib0, NA, NA, NA, NA, NA, NA, 0, -1),
	[4] = PINGROUP(4, qup0_se1_l2_mira, qup0_se1_l1, ddr_bist_complete, NA,
		       phase_flag22, dac_calib1, NA, NA, NA, NA, NA, 0, -1),
	[5] = PINGROUP(5, qup0_se1_l3_mira, qup0_se1_l0, NA, phase_flag23,
		       dac_calib2, NA, NA, NA, NA, NA, NA, 0, -1),
	[6] = PINGROUP(6, qup0_se2_l0, cri_trng0, NA, phase_flag24, dac_calib3,
		       NA, NA, NA, NA, NA, NA, 0, -1),
	[7] = PINGROUP(7, qup0_se2_l1, cri_trng1, NA, phase_flag25, dac_calib4,
		       NA, NA, NA, NA, NA, NA, 0, -1),
	[8] = PINGROUP(8, qup0_se2_l2, NA, phase_flag26, dac_calib5, NA, NA, NA,
		       NA, NA, NA, NA, 0, -1),
	[9] = PINGROUP(9, qup0_se2_l3, NA, phase_flag27, dac_calib6, NA, NA, NA,
		       NA, NA, NA, NA, 0, -1),
	[10] = PINGROUP(10, qup0_se3_l0, qup0_se3_l3, NA, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[11] = PINGROUP(11, qup0_se3_l1, qup0_se3_l2, NA, phase_flag11, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[12] = PINGROUP(12, qup0_se4_l0, qup0_se4_l3, char_exec_release, NA, NA,
			NA, NA, NA, NA, NA, NA, 0, -1),
	[13] = PINGROUP(13, qup0_se4_l1, qup0_se4_l2, char_exec_pending, NA, NA,
			NA, NA, NA, NA, NA, NA, 0, -1),
	[14] = PINGROUP(14, qup0_se5_l0, pll_clk, tgu_ch0_trigout, dac_calib7,
			wlan1_adc0, NA, NA, NA, NA, NA, NA, 0, -1),
	[15] = PINGROUP(15, qup0_se5_l1, tgu_ch1_trigout, NA, dac_calib8,
			wlan1_adc1, NA, NA, NA, NA, NA, NA, 0, -1),
	[16] = PINGROUP(16, qup0_se5_l2, tgu_ch2_trigout, NA, phase_flag30,
			dac_calib9, NA, NA, NA, NA, NA, NA, 0, -1),
	[17] = PINGROUP(17, qup0_se5_l3, tgu_ch3_trigout, NA, phase_flag31,
			dac_calib10, NA, NA, NA, NA, NA, NA, 0, -1),
	[18] = PINGROUP(18, qup0_se6_l0, dac_calib11, NA, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[19] = PINGROUP(19, qup0_se6_l1, dac_calib12, NA, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[20] = PINGROUP(20, qup0_se7_l0, qup0_se7_l3, cri_trng, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[21] = PINGROUP(21, qup0_se7_l1, qup0_se7_l2, tsense_pwm, NA, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[22] = PINGROUP(22, qup0_se8_l0, pll_clk, agera_pll, pbs_out_1, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[23] = PINGROUP(23, qup0_se8_l1, agera_pll, pbs_out_2, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[24] = PINGROUP(24, qup0_se8_l2, pbs_out_0, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[25] = PINGROUP(25, qup0_se8_l3, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0, -1),
	[26] = PINGROUP(26, qup0_se9_l2_mira, qup0_se9_l1_mira, NA, NA, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[27] = PINGROUP(27, qup0_se9_l3_mira, qup0_se9_l0_mira, prng_rosc, NA,
			NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[28] = PINGROUP(28, qup0_se1_l2_mirb, qup0_se6_l2, emac1_mcg1,
			prng_rosc, NA, phase_flag14, qdss_cti, NA, NA, NA, NA,
			0, -1),
	[29] = PINGROUP(29, qup0_se1_l3_mirb, qup0_se6_l3, emac1_mcg0, NA,
			phase_flag28, qdss_cti, NA, NA, NA, NA, NA, 0, -1),
	[30] = PINGROUP(30, qup0_se2_l4, qup0_se6_l4, NA, phase_flag29,
			qdss_cti, NA, NA, NA, NA, NA, NA, 0, -1),
	[31] = PINGROUP(31, qup0_se2_l5, qup0_se6_l5, emac1_ptp, emac1_ptp, NA,
			phase_flag21, qdss_cti, NA, NA, NA, NA, 0, -1),
	[32] = PINGROUP(32, pwm_0_mira, sdc2_tb_trig, NA, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[33] = PINGROUP(33, emac1_ptp, emac1_ptp, sdc1_tb_trig, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[34] = PINGROUP(34, cam_mclk, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[35] = PINGROUP(35, cam_mclk, unused_adsp, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[36] = PINGROUP(36, cci_i2c_sda0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[37] = PINGROUP(37, cci_i2c_scl0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[38] = PINGROUP(38, cci_timer0, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0, -1),
	[39] = PINGROUP(39, cci_async_in0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[40] = PINGROUP(40, cci_timer2, emac1_mcg2, pwm_1_mira, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[41] = PINGROUP(41, cci_i2c_sda1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[42] = PINGROUP(42, cci_i2c_scl1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[43] = PINGROUP(43, cci_timer3, emac0_mcg3, pll_bist, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[44] = PINGROUP(44, emac0_mcg2, pll_bist, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[45] = PINGROUP(45, tsc_async, emac0_mcg1, pwm_7_mira, gcc_gp1, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[46] = PINGROUP(46, tsc_async, emac0_mcg0, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[47] = PINGROUP(47, cci_timer1, emac1_mcg3, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[48] = PINGROUP(48, NA, qup0_se9_l0_mirb, NA, NA, pbs_in2, phase_flag7,
			NA, NA, NA, NA, NA, 0, -1),
	[49] = PINGROUP(49, NA, qup0_se9_l1_mirb, NA, NA, pbs_in0, phase_flag8,
			NA, NA, NA, NA, NA, 0, -1),
	[50] = PINGROUP(50, NA, qup0_se9_l2_mirb, NA, NA, pbs_in1, phase_flag9,
			NA, NA, NA, NA, NA, 0, -1),
	[51] = PINGROUP(51, NA, qup0_se9_l3_mirb, pbs_in15, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[52] = PINGROUP(52, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[53] = PINGROUP(53, NA, nav_gpio, gcc_gp3, pwm_7_mirb, NA, pbs_in4,
			atest_usb21, NA, NA, NA, NA, 0, -1),
	[54] = PINGROUP(54, NA, pwm_1_mirb, NA, pbs_in5, phase_flag3,
			atest_char2, NA, NA, NA, NA, NA, 0, -1),
	[55] = PINGROUP(55, NA, pwm_0_mirb, NA, pbs_in6, phase_flag5,
			atest_char3, NA, NA, NA, NA, NA, 0, -1),
	[56] = PINGROUP(56, NA, pwm_6_mirb, NA, pbs_in7, phase_flag10,
			atest_char0, NA, NA, NA, NA, NA, 0, -1),
	[57] = PINGROUP(57, NA, pwm_2_mirb, NA, pbs_in8, phase_flag20,
			atest_char1, NA, NA, NA, NA, NA, 0, -1),
	[58] = PINGROUP(58, NA, nav_gpio, pwm_8_mirb, NA, pbs_in9, atest_bbrx0,
			atest_usb22, vsense_trigger_mirnat, emac1_dll, NA, NA,
			0, -1),
	[59] = PINGROUP(59, NA, vfr_1, NA, pbs_in10, atest_bbrx1, atest_usb23,
			emac1_dll, NA, NA, NA, NA, 0, -1),
	[60] = PINGROUP(60, NA, emac1_ptp, emac1_ptp, emac0_ptp, emac0_ptp, NA,
			pbs_in11, atest_gpsadc, atest_usb2, emac0_dll, NA, 0,
			-1),
	[61] = PINGROUP(61, NA, pwm_9_mira, gcc_gp2, pa_indicator_or,
			dbg_out_clk, pbs_in12, atest_usb20, emac0_dll, NA, NA,
			NA, 0, -1),
	[62] = PINGROUP(62, NA, pwm_2_mira, NA, pbs_in13, phase_flag12,
			atest_char_start, NA, NA, NA, NA, NA, 0, -1),
	[63] = PINGROUP(63, NA, nav_gpio, emac0_ptp, emac0_ptp, NA, pbs_in14,
			phase_flag13, dac_calib15, NA, NA, NA, 0, -1),
	[64] = PINGROUP(64, NA, unused_gsm1, dac_calib13, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[65] = PINGROUP(65, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[66] = PINGROUP(66, NA, dac_calib14, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0, -1),
	[67] = PINGROUP(67, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[68] = PINGROUP(68, NA, ssbi_wtr2, emac1_ptp, emac1_ptp, pwm_9_mirb,
			dac_calib16, NA, NA, NA, NA, NA, 0, -1),
	[69] = PINGROUP(69, NA, ssbi_wtr3, emac0_ptp, emac0_ptp, NA,
			phase_flag15, dac_calib17, NA, NA, NA, NA, 0, -1),
	[70] = PINGROUP(70, NA, ssbi_wtr0, NA, phase_flag16, dac_calib18, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[71] = PINGROUP(71, NA, ssbi_wtr1, nav_gpio, NA, phase_flag17, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[72] = PINGROUP(72, NA, NA, phase_flag18, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[73] = PINGROUP(73, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[74] = PINGROUP(74, pll_reset_n, NA, pbs_in3, phase_flag19, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[75] = PINGROUP(75, gsm0_tx, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[76] = PINGROUP(76, pll_bypassnl, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[77] = PINGROUP(77, uim2_data, pwm_3, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[78] = PINGROUP(78, uim2_clk, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[79] = PINGROUP(79, uim2_reset, pwm_4, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[80] = PINGROUP(80, uim2_present, pwm_5, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[81] = PINGROUP(81, uim1_data, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0, -1),
	[82] = PINGROUP(82, uim1_clk, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[83] = PINGROUP(83, uim1_reset, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0, -1),
	[84] = PINGROUP(84, uim1_present, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[85] = PINGROUP(85, emac0_ptp, emac0_ptp, NA, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[86] = PINGROUP(86, mdp_vsync_p, mdp_vsync, mdp_vsync, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[87] = PINGROUP(87, NA, pwm_6_mira, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			0, -1),
	[88] = PINGROUP(88, gcc_gp2, NA, dac_calib21, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[89] = PINGROUP(89, gcc_gp3, NA, dac_calib19, NA, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[90] = PINGROUP(90, usb0_phy_ps, NA, dac_calib20, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[91] = PINGROUP(91, nav_gpio, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[92] = PINGROUP(92, nav_gpio, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			-1),
	[93] = PINGROUP(93, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[94] = PINGROUP(94, mdp_vsync_e, qdss_cti, qdss_cti, NA, NA, NA, NA, NA,
			NA, NA, NA, 0, -1),
	[95] = PINGROUP(95, nav_gpio, mdp_vsync_s, qdss_cti, qdss_cti, NA, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[96] = PINGROUP(96, dmic0_clk, cam_mclk, i2s1_clk, jitter_bist,
			atest_gpsadc, atest_usb1, NA, NA, NA, NA, NA, 0, -1),
	[97] = PINGROUP(97, dmic0_data, i2s1_ws, dac_calib23, NA, NA, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[98] = PINGROUP(98, dmic1_clk, cam_mclk, i2s1_data0, NA, NA,
			atest_usb10, ddr_pxi0, NA, NA, NA, NA, 0, -1),
	[99] = PINGROUP(99, dmic1_data, i2s1_data1, jitter_bist, NA,
			atest_usb11, ddr_pxi0, NA, NA, NA, NA, NA, 0, -1),
	[100] = PINGROUP(100, i2s2_clk, nav_gpio, NA, NA, atest_usb12, ddr_pxi1,
			 NA, NA, NA, NA, NA, 0, -1),
	[101] = PINGROUP(101, i2s2_ws, nav_gpio, NA, NA, atest_usb13, ddr_pxi1,
			 NA, NA, NA, NA, NA, 0, -1),
	[102] = PINGROUP(102, i2s2_data0, pwm_8_mira, NA, phase_flag4, NA, NA,
			 NA, NA, NA, NA, NA, 0, -1),
	[103] = PINGROUP(103, ext_mclk, i2s2_data1, NA, NA, NA, NA, NA, NA, NA,
			 NA, NA, 0, -1),
	[104] = PINGROUP(104, ext_mclk, nav_gpio, NA, NA, NA, NA, NA, NA, NA,
			 NA, NA, 0, -1),
	[105] = PINGROUP(105, swr0_tx_clk, i2s0_clk, NA, NA, NA, NA, NA, NA, NA,
			 NA, NA, 0, -1),
	[106] = PINGROUP(106, swr0_tx_data0, i2s0_ws, NA, NA, NA, NA, NA, NA,
			 NA, NA, NA, 0, -1),
	[107] = PINGROUP(107, swr0_rx_clk, i2s0_data0, NA, NA, NA, NA, NA, NA,
			 NA, NA, NA, 0, -1),
	[108] = PINGROUP(108, swr0_rx_data0, i2s0_data1, NA, NA, NA, NA, NA, NA,
			 NA, NA, NA, 0, -1),
	[109] = PINGROUP(109, swr0_rx_data1, i2s0_data2, sd_write_protect, NA,
			 NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[110] = PINGROUP(110, ext_mclk, i2s0_data3, NA, gcc_gp1, NA, NA, NA, NA,
			 NA, NA, NA, 0, -1),
	[111] = PINGROUP(111, i2s3_clk, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 0, -1),
	[112] = PINGROUP(112, i2s3_ws, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 0, -1),
	[113] = PINGROUP(113, i2s3_data0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[114] = PINGROUP(114, ext_mclk, i2s3_data1, NA, NA, NA, NA, NA, NA, NA,
			 NA, NA, 0, -1),
	[115] = PINGROUP(115, mss_lte, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 0, -1),
	[116] = PINGROUP(116, mss_lte, NA, dac_calib25, NA, NA, NA, NA, NA, NA,
			 NA, NA, 0, -1),
	[117] = PINGROUP(117, pcie0_clk_req_n, NA, dac_calib22, NA, NA, NA, NA,
			 NA, NA, NA, NA, 0, -1),
	[118] = PINGROUP(118, NA, dac_calib24, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[119] = PINGROUP(119, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[120] = PINGROUP(120, emac0_phy, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 0, -1),
	[121] = PINGROUP(121, rgmii0_rxc, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[122] = PINGROUP(122, rgmii0_rx_ctl, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[123] = PINGROUP(123, rgmii0_rxd0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[124] = PINGROUP(124, rgmii0_rxd1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[125] = PINGROUP(125, rgmii0_rxd2, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[126] = PINGROUP(126, rgmii0_rxd3, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[127] = PINGROUP(127, rgmii0_txc, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[128] = PINGROUP(128, rgmii0_tx_ctl, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[129] = PINGROUP(129, rgmii0_txd0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[130] = PINGROUP(130, rgmii0_txd1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[131] = PINGROUP(131, rgmii0_txd2, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[132] = PINGROUP(132, rgmii0_txd3, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[133] = PINGROUP(133, rgmii0_mdio, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[134] = PINGROUP(134, rgmii0_mdc, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[135] = PINGROUP(135, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[136] = PINGROUP(136, emac1_phy, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 0, -1),
	[137] = PINGROUP(137, rgmii1_rxc, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[138] = PINGROUP(138, rgmii1_rx_ctl, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[139] = PINGROUP(139, rgmii1_rxd0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[140] = PINGROUP(140, rgmii1_rxd1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[141] = PINGROUP(141, rgmii1_rxd2, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[142] = PINGROUP(142, rgmii1_rxd3, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[143] = PINGROUP(143, rgmii1_txc, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[144] = PINGROUP(144, rgmii1_tx_ctl, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[145] = PINGROUP(145, rgmii1_txd0, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[146] = PINGROUP(146, rgmii1_txd1, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[147] = PINGROUP(147, rgmii1_txd2, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[148] = PINGROUP(148, rgmii1_txd3, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[149] = PINGROUP(149, rgmii1_mdio, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[150] = PINGROUP(150, rgmii1_mdc, NA, NA, NA, NA, NA, NA, NA, NA, NA,
			 NA, 0, -1),
	[151] = PINGROUP(151, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[152] = PINGROUP(152, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[153] = PINGROUP(153, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[154] = PINGROUP(154, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[155] = PINGROUP(155, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[156] = PINGROUP(156, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[157] = PINGROUP(157, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[158] = PINGROUP(158, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[159] = PINGROUP(159, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[160] = PINGROUP(160, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[161] = PINGROUP(161, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0,
			 -1),
	[162] = SDC_QDSD_PINGROUP(sdc1_data, 0x1AC000, 9, 0),
	[163] = SDC_QDSD_PINGROUP(sdc1_rclk, 0x1AC004, 0, 0),
	[164] = SDC_QDSD_PINGROUP(sdc1_cmd, 0x1AC000, 11, 3),
	[165] = SDC_QDSD_PINGROUP(sdc1_clk, 0x1AC000, 13, 6),
	[166] = SDC_QDSD_PINGROUP(sdc2_clk, 0x1AA000, 14, 6),
	[167] = SDC_QDSD_PINGROUP(sdc2_cmd, 0x1AA000, 11, 3),
	[168] = SDC_QDSD_PINGROUP(sdc2_data, 0x1AA000, 9, 0),
};

static struct pinctrl_qup shikra_qup_regs[] = {
};

static const struct msm_gpio_wakeirq_map shikra_pdc_map[] = {

};

static const struct msm_pinctrl_soc_data shikra_tlmm = {
	.pins = shikra_pins,
	.npins = ARRAY_SIZE(shikra_pins),
	.functions = shikra_functions,
	.nfunctions = ARRAY_SIZE(shikra_functions),
	.groups = shikra_groups,
	.ngroups = ARRAY_SIZE(shikra_groups),
	.ngpios = 166,
	.qup_regs = shikra_qup_regs,
	.nqup_regs = ARRAY_SIZE(shikra_qup_regs),
	.wakeirq_map = shikra_pdc_map,
	.nwakeirq_map = ARRAY_SIZE(shikra_pdc_map),
	.egpio_func = 11,
};

static const struct of_device_id shikra_tlmm_of_match[] = {
	{ .compatible = "qcom,shikra-tlmm", .data = &shikra_tlmm },
	{},
};

static int shikra_tlmm_probe(struct platform_device *pdev)
{
	const struct msm_pinctrl_soc_data *pinctrl_data;
	struct device *dev = &pdev->dev;

	pinctrl_data = of_device_get_match_data(dev);
	if (!pinctrl_data)
		return -EINVAL;

	return msm_pinctrl_probe(pdev, pinctrl_data);
}

static struct platform_driver shikra_tlmm_driver = {
	.driver = {
		.name = "shikra-tlmm",
		.of_match_table = shikra_tlmm_of_match,
	},
	.probe = shikra_tlmm_probe,
	.remove = msm_pinctrl_remove,
};

static int __init shikra_tlmm_init(void)
{
	return platform_driver_register(&shikra_tlmm_driver);
}
arch_initcall(shikra_tlmm_init);

static void __exit shikra_tlmm_exit(void)
{
	platform_driver_unregister(&shikra_tlmm_driver);
}
module_exit(shikra_tlmm_exit);

MODULE_DESCRIPTION("QTI shikra TLMM driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, shikra_tlmm_of_match);
