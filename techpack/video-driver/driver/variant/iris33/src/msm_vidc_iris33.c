// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/reset.h>
#include "msm_vidc_iris33.h"
#include "msm_vidc_buffer_iris33.h"
#include "msm_vidc_power_iris33.h"
#include "msm_vidc_inst.h"
#include "msm_vidc_core.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_platform.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_buffer.h"
#include "msm_vidc_state.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_variant.h"
#include "venus_hfi.h"

#define VIDEO_ARCH_LX 1

#define VCODEC_BASE_OFFS_IRIS33                 0x00000000
#define VCODEC_CPU_CS_IRIS33                    0x000A0000
#define AON_BASE_OFFS                           0x000E0000

#define VCODEC_VPU_CPU_CS_VCICMDARG0_IRIS33                 (VCODEC_CPU_CS_IRIS33 + 0x24)
#define VCODEC_VPU_CPU_CS_VCICMDARG1_IRIS33                 (VCODEC_CPU_CS_IRIS33 + 0x28)
#define VCODEC_VPU_CPU_CS_SCIACMD_IRIS33                    (VCODEC_CPU_CS_IRIS33 + 0x48)
#define VCODEC_VPU_CPU_CS_SCIACMDARG0_IRIS33                (VCODEC_CPU_CS_IRIS33 + 0x4C)
#define VCODEC_VPU_CPU_CS_SCIACMDARG1_IRIS33                (VCODEC_CPU_CS_IRIS33 + 0x50)
#define VCODEC_VPU_CPU_CS_SCIACMDARG2_IRIS33                (VCODEC_CPU_CS_IRIS33 + 0x54)
#define VCODEC_VPU_CPU_CS_SCIBCMD_IRIS33                    (VCODEC_CPU_CS_IRIS33 + 0x5C)
#define VCODEC_VPU_CPU_CS_SCIBCMDARG0_IRIS33                (VCODEC_CPU_CS_IRIS33 + 0x60)
#define VCODEC_VPU_CPU_CS_SCIBARG1_IRIS33                   (VCODEC_CPU_CS_IRIS33 + 0x64)
#define VCODEC_VPU_CPU_CS_SCIBARG2_IRIS33                   (VCODEC_CPU_CS_IRIS33 + 0x68)

#define HFI_CTRL_INIT_IRIS33                          VCODEC_VPU_CPU_CS_SCIACMD_IRIS33
#define HFI_CTRL_STATUS_IRIS33                        VCODEC_VPU_CPU_CS_SCIACMDARG0_IRIS33
typedef enum {
    HFI_CTRL_NOT_INIT                   = 0x0,
    HFI_CTRL_READY                      = 0x1,
    HFI_CTRL_ERROR_FATAL                = 0x2,
    HFI_CTRL_ERROR_UC_REGION_NOT_SET    = 0x4,
    HFI_CTRL_ERROR_HW_FENCE_QUEUE       = 0x8,
    HFI_CTRL_PC_READY                   = 0x100,
    HFI_CTRL_VCODEC_IDLE                = 0x40000000
} hfi_ctrl_status_type;

#define HFI_QTBL_INFO_IRIS33                          VCODEC_VPU_CPU_CS_SCIACMDARG1_IRIS33
typedef enum {
    HFI_QTBL_DISABLED    = 0x00,
    HFI_QTBL_ENABLED     = 0x01,
} hfi_qtbl_status_type;

#define HFI_QTBL_ADDR_IRIS33                          VCODEC_VPU_CPU_CS_SCIACMDARG2_IRIS33
#define HFI_MMAP_ADDR_IRIS33                          VCODEC_VPU_CPU_CS_SCIBCMDARG0_IRIS33
#define HFI_UC_REGION_ADDR_IRIS33                     VCODEC_VPU_CPU_CS_SCIBARG1_IRIS33
#define HFI_UC_REGION_SIZE_IRIS33                     VCODEC_VPU_CPU_CS_SCIBARG2_IRIS33
#define HFI_DEVICE_REGION_ADDR_IRIS33                 VCODEC_VPU_CPU_CS_VCICMDARG0_IRIS33
#define HFI_DEVICE_REGION_SIZE_IRIS33                 VCODEC_VPU_CPU_CS_VCICMDARG1_IRIS33
#define HFI_SFR_ADDR_IRIS33                           VCODEC_VPU_CPU_CS_SCIBCMD_IRIS33

#define CPU_CS_A2HSOFTINTCLR_IRIS33             (VCODEC_CPU_CS_IRIS33 + 0x1C)
#define CPU_CS_H2XSOFTINTEN_IRIS33	(VCODEC_CPU_CS_IRIS33 + 0x148)

#define CPU_CS_AHB_BRIDGE_SYNC_RESET            (VCODEC_CPU_CS_IRIS33 + 0x160)

/* FAL10 Feature Control */
#define CPU_CS_X2RPMh_IRIS33		(VCODEC_CPU_CS_IRIS33 + 0x168)

#define CPU_IC_SOFTINT_IRIS33		(VCODEC_CPU_CS_IRIS33 + 0x150)
#define CPU_IC_SOFTINT_H2A_SHFT_IRIS33	0x0

/*
 * --------------------------------------------------------------------------
 * MODULE: wrapper
 * --------------------------------------------------------------------------
 */
#define WRAPPER_BASE_OFFS_IRIS33		0x000B0000
#define WRAPPER_INTR_STATUS_IRIS33	(WRAPPER_BASE_OFFS_IRIS33 + 0x0C)
#define WRAPPER_INTR_STATUS_A2HWD_BMSK_IRIS33	0x8
#define WRAPPER_INTR_STATUS_A2H_BMSK_IRIS33	0x4

#define WRAPPER_INTR_MASK_IRIS33		(WRAPPER_BASE_OFFS_IRIS33 + 0x10)
#define WRAPPER_INTR_MASK_A2HWD_BMSK_IRIS33	0x8
#define WRAPPER_INTR_MASK_A2HCPU_BMSK_IRIS33	0x4

#define WRAPPER_DEBUG_BRIDGE_LPI_CONTROL_IRIS33	(WRAPPER_BASE_OFFS_IRIS33 + 0x54)
#define WRAPPER_DEBUG_BRIDGE_LPI_STATUS_IRIS33	(WRAPPER_BASE_OFFS_IRIS33 + 0x58)
#define WRAPPER_IRIS_CPU_NOC_LPI_CONTROL	(WRAPPER_BASE_OFFS_IRIS33 + 0x5C)
#define WRAPPER_IRIS_CPU_NOC_LPI_STATUS		(WRAPPER_BASE_OFFS_IRIS33 + 0x60)
#define WRAPPER_CORE_POWER_STATUS		(WRAPPER_BASE_OFFS_IRIS33 + 0x80)
#define WRAPPER_CORE_CLOCK_CONFIG_IRIS33		(WRAPPER_BASE_OFFS_IRIS33 + 0x88)

/*
 * --------------------------------------------------------------------------
 * MODULE: tz_wrapper
 * --------------------------------------------------------------------------
 */
#define WRAPPER_TZ_BASE_OFFS	0x000C0000
#define WRAPPER_TZ_CPU_STATUS	(WRAPPER_TZ_BASE_OFFS + 0x10)
#define WRAPPER_TZ_CTL_AXI_CLOCK_CONFIG	(WRAPPER_TZ_BASE_OFFS + 0x14)
#define WRAPPER_TZ_QNS4PDXFIFO_RESET	(WRAPPER_TZ_BASE_OFFS + 0x18)

#define AON_WRAPPER_MVP_NOC_LPI_CONTROL	(AON_BASE_OFFS)
#define AON_WRAPPER_MVP_NOC_LPI_STATUS	(AON_BASE_OFFS + 0x4)
#define AON_WRAPPER_MVP_NOC_CORE_SW_RESET (AON_BASE_OFFS + 0x18)
#define AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL (AON_BASE_OFFS + 0x20)
#define AON_WRAPPER_SPARE (AON_BASE_OFFS + 0x28)
/*
 * --------------------------------------------------------------------------
 * MODULE: VCODEC_SS registers
 * --------------------------------------------------------------------------
 */
#define VCODEC_SS_IDLE_STATUSn           (VCODEC_BASE_OFFS_IRIS33 + 0x70)

/*
 * --------------------------------------------------------------------------
 * MODULE: VCODEC_NOC
 * --------------------------------------------------------------------------
 */
#define NOC_BASE_OFFS   0x00010000

#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_MAINCTL_LOW_IRIS33   (NOC_BASE_OFFS + 0xA008)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRCLR_LOW_IRIS33    (NOC_BASE_OFFS + 0xA018)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_LOW_IRIS33   (NOC_BASE_OFFS + 0xA020)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_HIGH_IRIS33  (NOC_BASE_OFFS + 0xA024)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_LOW_IRIS33   (NOC_BASE_OFFS + 0xA028)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_HIGH_IRIS33  (NOC_BASE_OFFS + 0xA02C)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_LOW_IRIS33   (NOC_BASE_OFFS + 0xA030)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_HIGH_IRIS33  (NOC_BASE_OFFS + 0xA034)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_LOW_IRIS33   (NOC_BASE_OFFS + 0xA038)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_HIGH_IRIS33  (NOC_BASE_OFFS + 0xA03C)
#define NOC_SIDEBANDMANAGER_MAIN_SIDEBANDMANAGER_FAULTINEN0_LOW_IRIS33 (NOC_BASE_OFFS + 0x7040)

#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_MAINCTL_LOW_IRIS33_2P   (NOC_BASE_OFFS + 0x3508)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRCLR_LOW_IRIS33_2P    (NOC_BASE_OFFS + 0x3518)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_LOW_IRIS33_2P   (NOC_BASE_OFFS + 0x3520)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_HIGH_IRIS33_2P  (NOC_BASE_OFFS + 0x3524)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_LOW_IRIS33_2P   (NOC_BASE_OFFS + 0x3528)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_HIGH_IRIS33_2P  (NOC_BASE_OFFS + 0x352C)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_LOW_IRIS33_2P   (NOC_BASE_OFFS + 0x3530)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_HIGH_IRIS33_2P  (NOC_BASE_OFFS + 0x3534)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_LOW_IRIS33_2P   (NOC_BASE_OFFS + 0x3538)
#define NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_HIGH_IRIS33_2P  (NOC_BASE_OFFS + 0x353C)
#define NOC_SIDEBANDMANAGER_MAIN_SIDEBANDMANAGER_FAULTINEN0_LOW_IRIS33_2P (NOC_BASE_OFFS + 0x3240)

static int __interrupt_init_iris33(struct msm_vidc_core *core)
{
	u32 mask_val = 0;
	int rc = 0;

	/* All interrupts should be disabled initially 0x1F6 : Reset value */
	rc = __read_register(core, WRAPPER_INTR_MASK_IRIS33, &mask_val);
	if (rc)
		return rc;

	/* Write 0 to unmask CPU and WD interrupts */
	mask_val &= ~(WRAPPER_INTR_MASK_A2HWD_BMSK_IRIS33 |
			WRAPPER_INTR_MASK_A2HCPU_BMSK_IRIS33);
	rc = __write_register(core, WRAPPER_INTR_MASK_IRIS33, mask_val);
	if (rc)
		return rc;

	return 0;
}

static int __get_device_region_info(struct msm_vidc_core *core,
	u32 *min_dev_addr, u32 *dev_reg_size)
{
	struct device_region_set *dev_set;
	u32 min_addr, max_addr, count = 0;
	int rc = 0;

	dev_set = &core->resource->device_region_set;

	if (!dev_set->count) {
		d_vpr_h("%s: device region not available\n", __func__);
		return 0;
	}

	min_addr = 0xFFFFFFFF;
	max_addr = 0x0;
	for (count = 0; count < dev_set->count; count++) {
		if (dev_set->device_region_tbl[count].dev_addr > max_addr)
			max_addr = dev_set->device_region_tbl[count].dev_addr +
				dev_set->device_region_tbl[count].size;
		if (dev_set->device_region_tbl[count].dev_addr < min_addr)
			min_addr = dev_set->device_region_tbl[count].dev_addr;
	}
	if (min_addr == 0xFFFFFFFF || max_addr == 0x0) {
		d_vpr_e("%s: invalid device region\n", __func__);
		return -EINVAL;
	}

	*min_dev_addr = min_addr;
	*dev_reg_size = max_addr - min_addr;

	return rc;
}

static int __program_bootup_registers_iris33(struct msm_vidc_core *core)
{
	u32 min_dev_reg_addr = 0, dev_reg_size = 0;
	u32 value;
	int rc = 0;

	value = (u32)core->iface_q_table.align_device_addr;
	rc = __write_register(core, HFI_UC_REGION_ADDR_IRIS33, value);
	if (rc)
		return rc;

	value = SHARED_QSIZE;
	rc = __write_register(core, HFI_UC_REGION_SIZE_IRIS33, value);
	if (rc)
		return rc;

	value = (u32)core->iface_q_table.align_device_addr;
	rc = __write_register(core, HFI_QTBL_ADDR_IRIS33, value);
	if (rc)
		return rc;

	rc = __write_register(core, HFI_QTBL_INFO_IRIS33, HFI_QTBL_ENABLED);
	if (rc)
		return rc;

	if (core->mmap_buf.align_device_addr) {
		value = (u32)core->mmap_buf.align_device_addr;
		rc = __write_register(core, HFI_MMAP_ADDR_IRIS33, value);
		if (rc)
			return rc;
	} else {
		d_vpr_e("%s: skip mmap buffer programming\n", __func__);
		/* ignore the error for now for backward compatibility */
		/* return -EINVAL; */
	}

	rc = __get_device_region_info(core, &min_dev_reg_addr, &dev_reg_size);
	if (rc)
		return rc;

	if (min_dev_reg_addr && dev_reg_size) {
		rc = __write_register(core, HFI_DEVICE_REGION_ADDR_IRIS33, min_dev_reg_addr);
		if (rc)
			return rc;

		rc = __write_register(core, HFI_DEVICE_REGION_SIZE_IRIS33, dev_reg_size);
		if (rc)
			return rc;
	} else {
		d_vpr_e("%s: skip device region programming\n", __func__);
		/* ignore the error for now for backward compatibility */
		/* return -EINVAL; */
	}

	if (core->sfr.align_device_addr) {
		value = (u32)core->sfr.align_device_addr + VIDEO_ARCH_LX;
		rc = __write_register(core, HFI_SFR_ADDR_IRIS33, value);
		if (rc)
			return rc;
	}

	return 0;
}

static bool is_iris33_hw_power_collapsed(struct msm_vidc_core *core)
{
	int rc = 0;
	u32 value = 0, pwr_status = 0;

	rc = __read_register(core, WRAPPER_CORE_POWER_STATUS, &value);
	if (rc)
		return false;

	/* if BIT(1) is 1 then video hw power is on else off */
	pwr_status = value & BIT(1);
	return pwr_status ? false : true;
}

static int __power_off_iris33_hardware(struct msm_vidc_core *core)
{
	int rc = 0, i;
	u32 value = 0;
	bool pwr_collapsed = false;

	/*
	 * Incase hw power control is enabled, for any error case
	 * CPU WD, video hw unresponsive cases, NOC error case etc,
	 * execute NOC reset sequence before disabling power. If there
	 * is no CPU WD and hw power control is enabled, fw is expected
	 * to power collapse video hw always.
	 */
	if (is_core_sub_state(core, CORE_SUBSTATE_FW_PWR_CTRL)) {
		pwr_collapsed = is_iris33_hw_power_collapsed(core);
		if (pwr_collapsed) {
			d_vpr_h("%s: video hw power collapsed %s\n",
				__func__, core->sub_state_name);
			goto disable_power;
		} else {
			d_vpr_e("%s: video hw is power ON, try power collpase hw %s\n",
				__func__, core->sub_state_name);
		}
	}

	/*
	 * check to make sure core clock branch enabled else
	 * we cannot read vcodec top idle register
	 */
	rc = __read_register(core, WRAPPER_CORE_CLOCK_CONFIG_IRIS33, &value);
	if (rc)
		return rc;

	if (value) {
		d_vpr_e("%s: core clock config not enabled, enabling it to read vcodec registers\n",
			__func__);
		rc = __write_register(core, WRAPPER_CORE_CLOCK_CONFIG_IRIS33, 0);
		if (rc)
			return rc;
	}

	/*
	 * add MNoC idle check before collapsing MVS0 per HPG update
	 * poll for NoC DMA idle -> HPG 6.1.1
	 */
	for (i = 0; i < core->capabilities[NUM_VPP_PIPE].value; i++) {
		rc = __read_register_with_poll_timeout(core, VCODEC_SS_IDLE_STATUSn + 4*i,
				0x400000, 0x400000, 2000, 20000);
		if (rc)
			d_vpr_e("%s: VCODEC_SS_IDLE_STATUSn (%d) is not idle (%#x)\n",
				__func__, i, value);
	}

	/* set MNoC to low power, set PD_NOC_QREQ (bit 0) */
	rc = __write_register_masked(core, AON_WRAPPER_MVP_NOC_LPI_CONTROL,
					0x1, BIT(0));
	if (rc)
		return rc;

	rc = __read_register_with_poll_timeout(core, AON_WRAPPER_MVP_NOC_LPI_STATUS,
					0x1, 0x1, 200, 2000);
	if (rc)
		d_vpr_e("%s: AON_WRAPPER_MVP_NOC_LPI_CONTROL failed\n", __func__);

	rc = __write_register_masked(core, AON_WRAPPER_MVP_NOC_LPI_CONTROL,
					0x0, BIT(0));
	if (rc)
		return rc;

	/*
	 * Reset both sides of 2 ahb2ahb_bridges (TZ and non-TZ)
	 * do we need to check status register here?
	 */
	rc = __write_register(core, CPU_CS_AHB_BRIDGE_SYNC_RESET, 0x3);
	if (rc)
		return rc;
	rc = __write_register(core, CPU_CS_AHB_BRIDGE_SYNC_RESET, 0x2);
	if (rc)
		return rc;
	rc = __write_register(core, CPU_CS_AHB_BRIDGE_SYNC_RESET, 0x0);
	if (rc)
		return rc;

disable_power:
	/* power down process */
	rc = call_res_op(core, gdsc_off, core, "vcodec");
	if (rc) {
		d_vpr_e("%s: disable regulator vcodec failed\n", __func__);
		rc = 0;
	}

	rc = call_res_op(core, clk_disable, core, "video_cc_mvs0_clk");
	if (rc) {
		d_vpr_e("%s: disable unprepare video_cc_mvs0_clk failed\n", __func__);
		rc = 0;
	}

	return rc;
}

static int __power_off_iris33_controller(struct msm_vidc_core *core)
{
	int rc = 0;
	int value = 0;

	/*
	 * mask fal10_veto QLPAC error since fal10_veto can go 1
	 * when pwwait == 0 and clamped to 0 -> HPG 6.1.2
	 */
	rc = __write_register(core, CPU_CS_X2RPMh_IRIS33, 0x3);
	if (rc)
		return rc;

	/* Set Iris CPU NoC to Low power */
	rc = __write_register_masked(core, WRAPPER_IRIS_CPU_NOC_LPI_CONTROL,
			0x1, BIT(0));
	if (rc)
		return rc;

	rc = __read_register_with_poll_timeout(core, WRAPPER_IRIS_CPU_NOC_LPI_STATUS,
			0x1, 0x1, 200, 2000);
	if (rc)
		d_vpr_e("%s: WRAPPER_IRIS_CPU_NOC_LPI_CONTROL failed\n", __func__);

	/* Debug bridge LPI release */
	rc = __write_register(core, WRAPPER_DEBUG_BRIDGE_LPI_CONTROL_IRIS33, 0x0);
	if (rc)
		return rc;

	rc = __read_register_with_poll_timeout(core, WRAPPER_DEBUG_BRIDGE_LPI_STATUS_IRIS33,
			0xffffffff, 0x0, 200, 2000);
	if (rc)
		d_vpr_e("%s: debug bridge release failed\n", __func__);

	/* Reset MVP QNS4PDXFIFO */
	rc = __write_register(core, WRAPPER_TZ_CTL_AXI_CLOCK_CONFIG, 0x3);
	if (rc)
		return rc;

	rc = __write_register(core, WRAPPER_TZ_QNS4PDXFIFO_RESET, 0x1);
	if (rc)
		return rc;

	rc = __write_register(core, WRAPPER_TZ_QNS4PDXFIFO_RESET, 0x0);
	if (rc)
		return rc;

	rc = __write_register(core, WRAPPER_TZ_CTL_AXI_CLOCK_CONFIG, 0x0);
	if (rc)
		return rc;

	/* assert and deassert axi and mvs0c resets */
	rc = call_res_op(core, reset_control_assert, core, "video_axi_reset");
	if (rc)
		d_vpr_e("%s: assert video_axi_reset failed\n", __func__);
	/* set retain mem and peripheral before asset mvs0c reset */
	rc = call_res_op(core, clk_set_flag, core,
		"video_cc_mvs0c_clk", MSM_VIDC_CLKFLAG_RETAIN_MEM);
	if (rc)
		d_vpr_e("%s: set retain mem failed\n", __func__);
	rc = call_res_op(core, clk_set_flag, core,
		"video_cc_mvs0c_clk", MSM_VIDC_CLKFLAG_RETAIN_PERIPH);
	if (rc)
		d_vpr_e("%s: set retain peripheral failed\n", __func__);
	rc = call_res_op(core, reset_control_assert, core, "video_mvs0c_reset");
	if (rc)
		d_vpr_e("%s: assert video_mvs0c_reset failed\n", __func__);
	usleep_range(400, 500);
	rc = call_res_op(core, reset_control_deassert, core, "video_axi_reset");
	if (rc)
		d_vpr_e("%s: de-assert video_axi_reset failed\n", __func__);
	rc = call_res_op(core, reset_control_deassert, core, "video_mvs0c_reset");
	if (rc)
		d_vpr_e("%s: de-assert video_mvs0c_reset failed\n", __func__);

	/* Disable MVP NoC clock */
	rc = __write_register_masked(core, AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL,
			0x1, BIT(0));
	if (rc)
		return rc;

	/* enable MVP NoC reset */
	rc = __write_register_masked(core, AON_WRAPPER_MVP_NOC_CORE_SW_RESET,
			0x1, BIT(0));
	if (rc)
		return rc;

	/*
	 * need to acquire "video_xo_reset" before assert and release
	 * after de-assert "video_xo_reset" reset clock to avoid other
	 * drivers (eva driver) operating on this shared reset clock
	 * and AON_WRAPPER_SPARE register in parallel.
	 */
	rc = call_res_op(core, reset_control_acquire, core, "video_xo_reset");
	if (rc) {
		d_vpr_e("%s: failed to acquire video_xo_reset control\n", __func__);
		goto skip_video_xo_reset;
	}

	/* poll AON spare register bit0 to become zero with 50ms timeout */
	rc = __read_register_with_poll_timeout(core, AON_WRAPPER_SPARE,
			0x1, 0x0, 1000, 50 * 1000);
	if (rc)
		d_vpr_e("%s: AON spare register is not zero\n", __func__);

	/* enable bit(1) to avoid cvp noc xo reset */
	rc = __write_register(core, AON_WRAPPER_SPARE, value | 0x2);
	if (rc)
		goto exit;

	/* assert video_cc XO reset */
	rc = call_res_op(core, reset_control_assert, core, "video_xo_reset");
	if (rc)
		d_vpr_e("%s: assert video_xo_reset failed\n", __func__);

	/* De-assert MVP NoC reset */
	rc = __write_register_masked(core, AON_WRAPPER_MVP_NOC_CORE_SW_RESET,
			0x0, BIT(0));
	if (rc)
		d_vpr_e("%s: MVP_NOC_CORE_SW_RESET failed\n", __func__);

	/* De-assert video_cc XO reset */
	usleep_range(200, 300);
	rc = call_res_op(core, reset_control_deassert, core, "video_xo_reset");
	if (rc)
		d_vpr_e("%s: deassert video_xo_reset failed\n", __func__);

	/* reset AON spare register */
	rc = __write_register(core, AON_WRAPPER_SPARE, 0x0);
	if (rc)
		goto exit;

	/* release reset control for other consumers */
	rc = call_res_op(core, reset_control_release, core, "video_xo_reset");
	if (rc)
		d_vpr_e("%s: failed to release video_xo_reset reset\n", __func__);

skip_video_xo_reset:
	/* Enable MVP NoC clock */
	rc = __write_register_masked(core, AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL,
			0x0, BIT(0));
	if (rc)
		return rc;

	/* remove retain mem and retain peripheral */
	rc = call_res_op(core, clk_set_flag, core,
		"video_cc_mvs0c_clk", MSM_VIDC_CLKFLAG_NORETAIN_PERIPH);
	if (rc)
		d_vpr_e("%s: set noretain peripheral failed\n", __func__);

	rc = call_res_op(core, clk_set_flag, core,
		"video_cc_mvs0c_clk", MSM_VIDC_CLKFLAG_NORETAIN_MEM);
	if (rc)
		d_vpr_e("%s: set noretain mem failed\n", __func__);

	/* Turn off MVP MVS0C core clock */
	rc = call_res_op(core, clk_disable, core, "video_cc_mvs0c_clk");
	if (rc) {
		d_vpr_e("%s: disable unprepare video_cc_mvs0c_clk failed\n", __func__);
		rc = 0;
	}

	/* power down process */
	rc = call_res_op(core, gdsc_off, core, "iris-ctl");
	if (rc) {
		d_vpr_e("%s: disable regulator iris-ctl failed\n", __func__);
		rc = 0;
	}

	/* Turn off GCC AXI clock */
	rc = call_res_op(core, clk_disable, core, "gcc_video_axi0_clk");
	if (rc) {
		d_vpr_e("%s: disable unprepare gcc_video_axi0_clk failed\n", __func__);
		rc = 0;
	}

	return rc;

exit:
	call_res_op(core, reset_control_release, core, "video_xo_reset");
	return rc;
}

static int __power_off_iris33(struct msm_vidc_core *core)
{
	int rc = 0;

	if (!is_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE))
		return 0;

	/**
	 * Reset video_cc_mvs0_clk_src value to resolve MMRM high video
	 * clock projection issue.
	 */
	rc = call_res_op(core, set_clks, core, 0);
	if (rc)
		d_vpr_e("%s: resetting clocks failed\n", __func__);

	if (__power_off_iris33_hardware(core))
		d_vpr_e("%s: failed to power off hardware\n", __func__);

	if (__power_off_iris33_controller(core))
		d_vpr_e("%s: failed to power off controller\n", __func__);

	rc = call_res_op(core, set_bw, core, 0, 0);
	if (rc)
		d_vpr_e("%s: failed to unvote buses\n", __func__);

	if (!call_venus_op(core, watchdog, core, core->intr_status))
		disable_irq_nosync(core->resource->irq);

	msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE, 0, __func__);

	return rc;
}

static int __power_on_iris33_controller(struct msm_vidc_core *core)
{
	int rc = 0;

	rc = call_res_op(core, gdsc_on, core, "iris-ctl");
	if (rc)
		goto fail_regulator;

	rc = call_res_op(core, reset_control_assert, core, "video_axi_reset");
	if (rc)
		goto fail_reset_assert_axi;
	rc = call_res_op(core, reset_control_assert, core, "video_mvs0c_reset");
	if (rc)
		goto fail_reset_assert_mvs0c;
	/* add usleep between assert and deassert */
	usleep_range(1000, 1100);
	rc = call_res_op(core, reset_control_deassert, core, "video_axi_reset");
	if (rc)
		goto fail_reset_deassert_axi;
	rc = call_res_op(core, reset_control_deassert, core, "video_mvs0c_reset");
	if (rc)
		goto fail_reset_deassert_mvs0c;

	rc = call_res_op(core, clk_enable, core, "gcc_video_axi0_clk");
	if (rc)
		goto fail_clk_axi;

	rc = call_res_op(core, clk_enable, core, "video_cc_mvs0c_clk");
	if (rc)
		goto fail_clk_controller;

	return 0;

fail_clk_controller:
	call_res_op(core, clk_disable, core, "gcc_video_axi0_clk");
fail_clk_axi:
fail_reset_deassert_mvs0c:
fail_reset_deassert_axi:
	call_res_op(core, reset_control_deassert, core, "video_mvs0c_reset");
fail_reset_assert_mvs0c:
	call_res_op(core, reset_control_deassert, core, "video_axi_reset");
fail_reset_assert_axi:
	call_res_op(core, gdsc_off, core, "iris-ctl");
fail_regulator:
	return rc;
}

static int __power_on_iris33_hardware(struct msm_vidc_core *core)
{
	int rc = 0;

	rc = call_res_op(core, gdsc_on, core, "vcodec");
	if (rc)
		goto fail_regulator;

	rc = call_res_op(core, clk_enable, core, "video_cc_mvs0_clk");
	if (rc)
		goto fail_clk_controller;

	return 0;

fail_clk_controller:
	call_res_op(core, gdsc_off, core, "vcodec");
fail_regulator:
	return rc;
}

static int __power_on_iris33(struct msm_vidc_core *core)
{
	struct frequency_table *freq_tbl;
	u32 freq = 0;
	int rc = 0;

	if (is_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE))
		return 0;

	if (!core_in_valid_state(core)) {
		d_vpr_e("%s: invalid core state %s\n",
			__func__, core_state_name(core->state));
		return -EINVAL;
	}

	/* Vote for all hardware resources */
	rc = call_res_op(core, set_bw, core, INT_MAX, INT_MAX);
	if (rc) {
		d_vpr_e("%s: failed to vote buses, rc %d\n", __func__, rc);
		goto fail_vote_buses;
	}

	rc = __power_on_iris33_controller(core);
	if (rc) {
		d_vpr_e("%s: failed to power on iris33 controller\n", __func__);
		goto fail_power_on_controller;
	}

	rc = __power_on_iris33_hardware(core);
	if (rc) {
		d_vpr_e("%s: failed to power on iris33 hardware\n", __func__);
		goto fail_power_on_hardware;
	}
	/* video controller and hardware powered on successfully */
	rc = msm_vidc_change_core_sub_state(core, 0, CORE_SUBSTATE_POWER_ENABLE, __func__);
	if (rc)
		goto fail_power_on_substate;

	freq_tbl = core->resource->freq_set.freq_tbl;
	freq = core->power.clk_freq ? core->power.clk_freq :
				      freq_tbl[0].freq;

	rc = call_res_op(core, set_clks, core, freq);
	if (rc) {
		d_vpr_e("%s: failed to scale clocks\n", __func__);
		rc = 0;
	}
	/*
	 * Re-program all of the registers that get reset as a result of
	 * regulator_disable() and _enable()
	 * When video module writing to QOS registers EVA module is not
	 * supposed to do video_xo_reset operations else we will see register
	 * access failure, so acquire video_xo_reset to ensure EVA module is
	 * not doing assert or de-assert on video_xo_reset.
	 */
	rc = call_res_op(core, reset_control_acquire, core, "video_xo_reset");
	if (rc) {
		d_vpr_e("%s: failed to acquire video_xo_reset control\n", __func__);
		goto fail_assert_xo_reset;
	}

	__set_registers(core);

	/*
	 * Programm NOC error registers before releasing xo reset
	 * Clear error logger registers and then enable StallEn
	 */
	if (core->platform->data.vpu_ver == VPU_VERSION_IRIS33) {
		rc = __write_register(core,
				NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRCLR_LOW_IRIS33, 0x1);
		if (rc) {
			d_vpr_e(
				"%s: error clearing NOC_MAIN_ERRORLOGGER_ERRCLR_LOW\n",
				__func__);
			goto fail_program_noc_regs;
		}

		rc = __write_register(core,
				NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_MAINCTL_LOW_IRIS33, 0x3);
		if (rc) {
			d_vpr_e(
				"%s: failed to set NOC_ERL_MAIN_ERRORLOGGER_MAINCTL_LOW\n",
				__func__);
			goto fail_program_noc_regs;
		}
		rc = __write_register(core,
				NOC_SIDEBANDMANAGER_MAIN_SIDEBANDMANAGER_FAULTINEN0_LOW_IRIS33,
				0x1);
		if (rc) {
			d_vpr_e(
				"%s: failed to set NOC_SIDEBANDMANAGER_FAULTINEN0_LOW\n",
				__func__);
			goto fail_program_noc_regs;
		}
	} else if (core->platform->data.vpu_ver == VPU_VERSION_IRIS33_2P) {
		rc = __write_register(core,
				NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRCLR_LOW_IRIS33_2P, 0x1);
		if (rc) {
			d_vpr_e(
				"%s: error clearing NOC_MAIN_ERRORLOGGER_ERRCLR_LOW\n",
				__func__);
			goto fail_program_noc_regs;
		}

		rc = __write_register(core,
				NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_MAINCTL_LOW_IRIS33_2P, 0x3);
		if (rc) {
			d_vpr_e(
				"%s: failed to set NOC_ERL_MAIN_ERRORLOGGER_MAINCTL_LOW\n",
				__func__);
			goto fail_program_noc_regs;
		}
		rc = __write_register(core,
				NOC_SIDEBANDMANAGER_MAIN_SIDEBANDMANAGER_FAULTINEN0_LOW_IRIS33_2P,
				0x1);
		if (rc) {
			d_vpr_e(
				"%s: failed to set NOC_SIDEBANDMANAGER_FAULTINEN0_LOW\n",
				__func__);
			goto fail_program_noc_regs;
		}
	}

	/* release reset control for other consumers */
	rc = call_res_op(core, reset_control_release, core, "video_xo_reset");
	if (rc) {
		d_vpr_e("%s: failed to release video_xo_reset reset\n", __func__);
		goto fail_deassert_xo_reset;
	}

	__interrupt_init_iris33(core);
	core->intr_status = 0;
	enable_irq(core->resource->irq);

	return rc;

fail_program_noc_regs:
	call_res_op(core, reset_control_release, core, "video_xo_reset");
fail_deassert_xo_reset:
fail_assert_xo_reset:
fail_power_on_substate:
	__power_off_iris33_hardware(core);
fail_power_on_hardware:
	__power_off_iris33_controller(core);
fail_power_on_controller:
	call_res_op(core, set_bw, core, 0, 0);
fail_vote_buses:
	msm_vidc_change_core_sub_state(core, CORE_SUBSTATE_POWER_ENABLE, 0, __func__);

	return rc;
}

static int __prepare_pc_iris33(struct msm_vidc_core *core)
{
	int rc = 0;
	u32 wfi_status = 0, idle_status = 0, pc_ready = 0;
	u32 ctrl_status = 0;

	rc = __read_register(core, HFI_CTRL_STATUS_IRIS33, &ctrl_status);
	if (rc)
		return rc;

	pc_ready = ctrl_status & HFI_CTRL_PC_READY;
	idle_status = ctrl_status & BIT(30);

	if (pc_ready) {
		d_vpr_h("Already in pc_ready state\n");
		return 0;
	}
	rc = __read_register(core, WRAPPER_TZ_CPU_STATUS, &wfi_status);
	if (rc)
		return rc;

	wfi_status &= BIT(0);
	if (!wfi_status || !idle_status) {
		d_vpr_e("Skipping PC, wfi status not set\n");
		goto skip_power_off;
	}

	rc = __prepare_pc(core);
	if (rc) {
		d_vpr_e("Failed __prepare_pc %d\n", rc);
		goto skip_power_off;
	}

	rc = __read_register_with_poll_timeout(core, HFI_CTRL_STATUS_IRIS33,
			HFI_CTRL_PC_READY, HFI_CTRL_PC_READY, 250, 2500);
	if (rc) {
		d_vpr_e("%s: Skip PC. Ctrl status not set\n", __func__);
		goto skip_power_off;
	}

	rc = __read_register_with_poll_timeout(core, WRAPPER_TZ_CPU_STATUS,
			BIT(0), 0x1, 250, 2500);
	if (rc) {
		d_vpr_e("%s: Skip PC. Wfi status not set\n", __func__);
		goto skip_power_off;
	}
	return rc;

skip_power_off:
	rc = __read_register(core, HFI_CTRL_STATUS_IRIS33, &ctrl_status);
	if (rc)
		return rc;
	rc = __read_register(core, WRAPPER_TZ_CPU_STATUS, &wfi_status);
	if (rc)
		return rc;
	wfi_status &= BIT(0);
	d_vpr_e("Skip PC, wfi=%#x, idle=%#x, pcr=%#x, ctrl=%#x)\n",
		wfi_status, idle_status, pc_ready, ctrl_status);
	return -EAGAIN;
}

static int __raise_interrupt_iris33(struct msm_vidc_core *core)
{
	int rc = 0;

	rc = __write_register(core, CPU_IC_SOFTINT_IRIS33, 1 << CPU_IC_SOFTINT_H2A_SHFT_IRIS33);
	if (rc)
		return rc;

	return 0;
}

static int __watchdog_iris33(struct msm_vidc_core *core, u32 intr_status)
{
	int rc = 0;

	if (intr_status & WRAPPER_INTR_STATUS_A2HWD_BMSK_IRIS33) {
		d_vpr_e("%s: received watchdog interrupt\n", __func__);
		rc = 1;
	}

	return rc;
}

static int __read_noc_err_register_iris33(struct msm_vidc_core *core)
{
	int rc = 0;
	u32 value;

	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_LOW_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_HIGH_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_HIGH:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_LOW_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_HIGH_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_HIGH:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_LOW_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_HIGH_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_HIGH:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_LOW_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_HIGH_IRIS33, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_HIGH:  %#x\n",
			__func__, value);

	return rc;
}

static int __read_noc_err_register_iris33_2p(struct msm_vidc_core *core)
{
	int rc = 0;
	u32 value;

	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_LOW_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_HIGH_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG0_HIGH:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_LOW_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_HIGH_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG1_HIGH:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_LOW_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_HIGH_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG2_HIGH:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_LOW_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_LOW:  %#x\n",
			__func__, value);
	rc = __read_register(core,
			NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_HIGH_IRIS33_2P, &value);
	if (!rc)
		d_vpr_e("%s: NOC_ERL_ERRORLOGGER_MAIN_ERRORLOGGER_ERRLOG3_HIGH:  %#x\n",
			__func__, value);

	return rc;
}

static int __noc_error_info_iris33(struct msm_vidc_core *core)
{
	int rc = 0;

	/*
	 * we are not supposed to access vcodec subsystem registers
	 * unless vcodec core clock WRAPPER_CORE_CLOCK_CONFIG_IRIS33 is enabled.
	 * core clock might have been disabled by video firmware as part of
	 * inter frame power collapse (power plane control feature).
	 */

	/*
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_SWID_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_SWID_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_SWID_HIGH);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_SWID_HIGH:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_MAINCTL_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_MAINCTL_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRVLD_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRVLD_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRCLR_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRCLR_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG0_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG0_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG0_HIGH);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG0_HIGH:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG1_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG1_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG1_HIGH);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG1_HIGH:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG2_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG2_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG2_HIGH);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG2_HIGH:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG3_LOW);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG3_LOW:     %#x\n", val);
	val = __read_register(core, VCODEC_NOC_ERL_MAIN_ERRLOG3_HIGH);
	d_vpr_e("VCODEC_NOC_ERL_MAIN_ERRLOG3_HIGH:     %#x\n", val);
	 */

	if (is_iris33_hw_power_collapsed(core)) {
		d_vpr_e("%s: video hardware already power collapsed\n", __func__);
		return rc;
	}

	/*
	 * Acquire video_xo_reset to ensure EVA module is
	 * not doing assert or de-assert on video_xo_reset
	 * while reading noc registers
	 */
	d_vpr_e("%s: read NOC ERR LOG registers\n", __func__);
	rc = call_res_op(core, reset_control_acquire, core, "video_xo_reset");
	if (rc) {
		d_vpr_e("%s: failed to acquire video_xo_reset control\n", __func__);
		goto fail_assert_xo_reset;
	}

	if (core->platform->data.vpu_ver == VPU_VERSION_IRIS33)
		rc = __read_noc_err_register_iris33(core);
	else if (core->platform->data.vpu_ver == VPU_VERSION_IRIS33_2P)
		rc = __read_noc_err_register_iris33_2p(core);

	/* release reset control for other consumers */
	rc = call_res_op(core, reset_control_release, core, "video_xo_reset");
	if (rc) {
		d_vpr_e("%s: failed to release video_xo_reset reset\n", __func__);
		goto fail_deassert_xo_reset;
	}

fail_deassert_xo_reset:
fail_assert_xo_reset:
	MSM_VIDC_FATAL(true);
	return rc;
}

static int __clear_interrupt_iris33(struct msm_vidc_core *core)
{
	u32 intr_status = 0, mask = 0;
	int rc = 0;

	rc = __read_register(core, WRAPPER_INTR_STATUS_IRIS33, &intr_status);
	if (rc)
		return rc;

	mask = (WRAPPER_INTR_STATUS_A2H_BMSK_IRIS33|
		WRAPPER_INTR_STATUS_A2HWD_BMSK_IRIS33|
		HFI_CTRL_VCODEC_IDLE);

	if (intr_status & mask) {
		core->intr_status |= intr_status;
		core->reg_count++;
		d_vpr_l("INTERRUPT: times: %d interrupt_status: %d\n",
			core->reg_count, intr_status);
	} else {
		core->spur_count++;
	}

	rc = __write_register(core, CPU_CS_A2HSOFTINTCLR_IRIS33, 1);
	if (rc)
		return rc;

	return 0;
}

static int __boot_firmware_iris33(struct msm_vidc_core *core)
{
	int rc = 0;
	u32 ctrl_init_val = 0, ctrl_status = 0, count = 0, max_tries = 1000;

	rc = __program_bootup_registers_iris33(core);
	if (rc)
		return rc;

	ctrl_init_val = BIT(0);

	rc = __write_register(core, HFI_CTRL_INIT_IRIS33, ctrl_init_val);
	if (rc)
		return rc;

	while (count < max_tries) {
		rc = __read_register(core, HFI_CTRL_STATUS_IRIS33, &ctrl_status);
		if (rc)
			return rc;

		if ((ctrl_status & HFI_CTRL_ERROR_FATAL) ||
			(ctrl_status & HFI_CTRL_ERROR_UC_REGION_NOT_SET) ||
			(ctrl_status & HFI_CTRL_ERROR_HW_FENCE_QUEUE)) {
			d_vpr_e("%s: boot firmware failed, ctrl status %#x\n",
				__func__, ctrl_status);
			return -EINVAL;
		} else if (ctrl_status & HFI_CTRL_READY) {
			d_vpr_h("%s: boot firmware is successful, ctrl status %#x\n",
				__func__, ctrl_status);
			break;
		}

		usleep_range(50, 100);
		count++;
	}

	if (count >= max_tries) {
		d_vpr_e("Error booting up vidc firmware, ctrl status %#x\n", ctrl_status);
		return -ETIME;
	}

	/* Enable interrupt before sending commands to venus */
	rc = __write_register(core, CPU_CS_H2XSOFTINTEN_IRIS33, 0x1);
	if (rc)
		return rc;

	rc = __write_register(core, CPU_CS_X2RPMh_IRIS33, 0x0);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_decide_work_mode_iris33(struct msm_vidc_inst *inst)
{
	u32 work_mode;
	struct v4l2_format *inp_f;
	u32 width, height;
	bool res_ok = false;

	work_mode = MSM_VIDC_STAGE_2;
	inp_f = &inst->fmts[INPUT_PORT];

	if (is_image_decode_session(inst))
		work_mode = MSM_VIDC_STAGE_1;

	if (is_image_session(inst))
		goto exit;

	if (is_decode_session(inst)) {
		height = inp_f->fmt.pix_mp.height;
		width = inp_f->fmt.pix_mp.width;
		res_ok = res_is_less_than(width, height, 1280, 720);
		if (inst->capabilities[CODED_FRAMES].value ==
				CODED_FRAMES_INTERLACE ||
			inst->capabilities[LOWLATENCY_MODE].value ||
			res_ok) {
			work_mode = MSM_VIDC_STAGE_1;
		}
	} else if (is_encode_session(inst)) {
		height = inst->crop.height;
		width = inst->crop.width;
		res_ok = !res_is_greater_than(width, height, 4096, 2160);
		if (res_ok &&
			(inst->capabilities[LOWLATENCY_MODE].value)) {
			work_mode = MSM_VIDC_STAGE_1;
		}
		if (inst->capabilities[SLICE_MODE].value ==
			V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_BYTES) {
			work_mode = MSM_VIDC_STAGE_1;
		}
		if (inst->capabilities[LOSSLESS].value)
			work_mode = MSM_VIDC_STAGE_2;

		if (!inst->capabilities[GOP_SIZE].value)
			work_mode = MSM_VIDC_STAGE_2;
	} else {
		i_vpr_e(inst, "%s: invalid session type\n", __func__);
		return -EINVAL;
	}

exit:
	i_vpr_h(inst, "Configuring work mode = %u low latency = %u, gop size = %u\n",
		work_mode, inst->capabilities[LOWLATENCY_MODE].value,
		inst->capabilities[GOP_SIZE].value);
	msm_vidc_update_cap_value(inst, STAGE, work_mode, __func__);

	return 0;
}

int msm_vidc_decide_work_route_iris33(struct msm_vidc_inst *inst)
{
	u32 work_route;
	struct msm_vidc_core *core;

	core = inst->core;
	work_route = core->capabilities[NUM_VPP_PIPE].value;

	if (is_image_session(inst))
		goto exit;

	if (is_decode_session(inst)) {
		if (inst->capabilities[CODED_FRAMES].value ==
				CODED_FRAMES_INTERLACE)
			work_route = MSM_VIDC_PIPE_1;
	} else if (is_encode_session(inst)) {
		u32 slice_mode;

		slice_mode = inst->capabilities[SLICE_MODE].value;

		/*TODO Pipe=1 for legacy CBR*/
		if (slice_mode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_BYTES)
			work_route = MSM_VIDC_PIPE_1;

	} else {
		i_vpr_e(inst, "%s: invalid session type\n", __func__);
		return -EINVAL;
	}

exit:
	i_vpr_h(inst, "Configuring work route = %u", work_route);
	msm_vidc_update_cap_value(inst, PIPE, work_route, __func__);

	return 0;
}

int msm_vidc_decide_quality_mode_iris33(struct msm_vidc_inst *inst)
{
	struct msm_vidc_core *core;
	u32 mbpf, mbps, max_hq_mbpf, max_hq_mbps;
	u32 mode = MSM_VIDC_POWER_SAVE_MODE;

	if (!is_encode_session(inst))
		return 0;

	/* image or lossless or all intra runs at quality mode */
	if (is_image_session(inst) || inst->capabilities[LOSSLESS].value ||
		inst->capabilities[ALL_INTRA].value) {
		mode = MSM_VIDC_MAX_QUALITY_MODE;
		goto decision_done;
	}

	/* for lesser complexity, make LP for all resolution */
	if (inst->capabilities[COMPLEXITY].value < DEFAULT_COMPLEXITY) {
		mode = MSM_VIDC_POWER_SAVE_MODE;
		goto decision_done;
	}

	mbpf = msm_vidc_get_mbs_per_frame(inst);
	mbps = mbpf * msm_vidc_get_fps(inst);
	core = inst->core;
	max_hq_mbpf = core->capabilities[MAX_MBPF_HQ].value;;
	max_hq_mbps = core->capabilities[MAX_MBPS_HQ].value;;

	if (!is_realtime_session(inst)) {
		if (((inst->capabilities[COMPLEXITY].flags & CAP_FLAG_CLIENT_SET) &&
			(inst->capabilities[COMPLEXITY].value >= DEFAULT_COMPLEXITY)) ||
			mbpf <= max_hq_mbpf) {
			mode = MSM_VIDC_MAX_QUALITY_MODE;
			goto decision_done;
		}
	}

	if (mbpf <= max_hq_mbpf && mbps <= max_hq_mbps)
		mode = MSM_VIDC_MAX_QUALITY_MODE;

decision_done:
	msm_vidc_update_cap_value(inst, QUALITY_MODE, mode, __func__);

	return 0;
}

int msm_vidc_adjust_bitrate_boost_iris33(void *instance, struct v4l2_ctrl *ctrl)
{
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 rc_type = -1;
	u32 width, height, frame_rate;
	struct v4l2_format *f;
	u32 max_bitrate = 0, bitrate = 0;

	adjusted_value = ctrl ? ctrl->val :
		inst->capabilities[BITRATE_BOOST].value;

	if (inst->bufq[OUTPUT_PORT].vb2q->streaming)
		return 0;

	if (msm_vidc_get_parent_value(inst, BITRATE_BOOST,
		BITRATE_MODE, &rc_type, __func__))
		return -EINVAL;

	/*
	 * Bitrate Boost are supported only for VBR rc type.
	 * Hence, do not adjust or set to firmware for non VBR rc's
	 */
	if (rc_type != HFI_RC_VBR_CFR) {
		adjusted_value = 0;
		goto adjust;
	}

	frame_rate = inst->capabilities[FRAME_RATE].value >> 16;
	f = &inst->fmts[OUTPUT_PORT];
	width = f->fmt.pix_mp.width;
	height = f->fmt.pix_mp.height;

	/*
	 * honor client set bitrate boost
	 * if client did not set, keep max bitrate boost upto 4k@60fps
	 * and remove bitrate boost after 4k@60fps
	 */
	if (inst->capabilities[BITRATE_BOOST].flags & CAP_FLAG_CLIENT_SET) {
		/* accept client set bitrate boost value as is */
	} else {
		if (res_is_less_than_or_equal_to(width, height, 4096, 2176) &&
			frame_rate <= 60)
			adjusted_value = MAX_BITRATE_BOOST;
		else
			adjusted_value = 0;
	}

	max_bitrate = msm_vidc_get_max_bitrate(inst);
	bitrate = inst->capabilities[BIT_RATE].value;
	if (adjusted_value) {
		if ((bitrate + bitrate / (100 / adjusted_value)) > max_bitrate) {
			i_vpr_h(inst,
				"%s: bitrate %d is beyond max bitrate %d, remove bitrate boost\n",
				__func__, max_bitrate, bitrate);
			adjusted_value = 0;
		}
	}
adjust:
	msm_vidc_update_cap_value(inst, BITRATE_BOOST, adjusted_value, __func__);

	return 0;
}



static struct msm_vidc_venus_ops iris33_ops = {
	.boot_firmware = __boot_firmware_iris33,
	.raise_interrupt = __raise_interrupt_iris33,
	.clear_interrupt = __clear_interrupt_iris33,
	.power_on = __power_on_iris33,
	.power_off = __power_off_iris33,
	.prepare_pc = __prepare_pc_iris33,
	.watchdog = __watchdog_iris33,
	.noc_error_info = __noc_error_info_iris33,
};

static struct msm_vidc_session_ops msm_session_ops = {
	.buffer_size = msm_buffer_size_iris33,
	.min_count = msm_buffer_min_count_iris33,
	.extra_count = msm_buffer_extra_count_iris33,
	.ring_buf_count = msm_vidc_ring_buf_count_iris33,
	.calc_freq = msm_vidc_calc_freq_iris33,
	.calc_bw = msm_vidc_calc_bw_iris33,
	.decide_work_route = msm_vidc_decide_work_route_iris33,
	.decide_work_mode = msm_vidc_decide_work_mode_iris33,
	.decide_quality_mode = msm_vidc_decide_quality_mode_iris33,
};

int msm_vidc_init_iris33(struct msm_vidc_core *core)
{
	d_vpr_h("%s()\n", __func__);
	core->venus_ops = &iris33_ops;
	core->session_ops = &msm_session_ops;

	return 0;
}
