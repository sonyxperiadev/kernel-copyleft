/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef CAM_OFE_CORE_H
#define CAM_OFE_CORE_H

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/* OFE CDM/TOP status register */
#define OFE_RST_DONE_IRQ_STATUS_BIT  0x1

#define OFE_COLLAPSE_MASK 0x1
#define OFE_PWR_ON_MASK   0x2

struct cam_ofe_device_hw_info {
	uint32_t hw_idx;
	uint32_t pwr_ctrl;
	uint32_t pwr_status;

	uint32_t top_rst_cmd;
	uint32_t top_irq_status;
	uint32_t top_rst_val;
	uint32_t cdm_rst_cmd;
	uint32_t cdm_irq_status;

	uint32_t cdm_rst_val;
};

struct cam_ofe_device_core_info {
	struct cam_ofe_device_hw_info *ofe_hw_info;
	uint32_t cpas_handle;
	bool cpas_start;
	bool clk_enable;
};

int cam_ofe_init_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size);
int cam_ofe_deinit_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size);
int cam_ofe_process_cmd(void *device_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size);

irqreturn_t cam_ofe_irq(int irq_num, void *data);

/**
 * @brief : API to register OFE hw to platform framework.
 * @return struct platform_device pointer on success, or ERR_PTR() on error.
 */
int cam_ofe_init_module(void);

/**
 * @brief : API to remove OFE Hw from platform framework.
 */
void cam_ofe_exit_module(void);
#endif /* CAM_OFE_CORE_H */
