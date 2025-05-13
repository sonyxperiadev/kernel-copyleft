/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_ICP_V2_CORE_H_
#define _CAM_ICP_V2_CORE_H_

#include "cam_hw_intf.h"
#include "cam_icp_hw_intf.h"
#include "hfi_intf.h"
#include "cam_icp_v2_reg.h"

/* Domain ID masks */
#define ICP_V2_DOMAIN_MASK_CFG_0 0x00FF00FF
#define ICP_V2_DOMAIN_MASK_CFG_1 0xFF00FF00

enum cam_icp_v2_reg_base {
	ICP_V2_CSR_BASE,
	ICP_V2_CIRQ_BASE,
	ICP_V2_WD0_BASE,
	ICP_V2_SYS_BASE,
	ICP_V2_DOM_MASK_BASE,
	ICP_V2_BASE_MAX,
};

struct cam_icp_v2_core_info {
	struct cam_icp_irq_cb irq_cb;
	struct cam_icp_v2_hw_info *hw_info;
	int32_t reg_base_idx[ICP_V2_BASE_MAX];
	uint32_t cpas_handle;
	int hfi_handle;
	enum cam_icp_v2_reg_base irq_regbase_idx;
	struct {
		const struct firmware *fw_elf;
		void *fw;
		uint32_t fw_buf;
		uintptr_t fw_kva_addr;
		uint64_t fw_buf_len;
	} fw_params;
	bool cpas_start;
	bool use_sec_pil;
	bool is_irq_test;
};

int cam_icp_v2_hw_init(void *priv, void *args, uint32_t arg_size);
int cam_icp_v2_hw_deinit(void *priv, void *args, uint32_t arg_size);
int cam_icp_v2_process_cmd(void *priv, uint32_t cmd_type,
	void *args, uint32_t arg_size);
int cam_icp_v2_test_irq_line(void *priv);

int cam_icp_v2_cpas_register(struct cam_hw_intf *icp_v2_intf);
int cam_icp_v2_cpas_unregister(struct cam_hw_intf *icp_v2_intf);

irqreturn_t cam_icp_v2_handle_irq(int irq_num, void *data);

void cam_icp_v2_irq_raise(void *priv);
void cam_icp_v2_irq_enable(void *priv);
void __iomem *cam_icp_v2_iface_addr(void *priv);
void cam_icp_v2_populate_hfi_ops(const struct hfi_ops **hfi_proc_ops);

int cam_icp_v2_core_init(struct cam_hw_soc_info *soc_info,
	struct cam_icp_v2_core_info *core_info);

#endif /* _CAM_ICP_V2_CORE_H_ */
