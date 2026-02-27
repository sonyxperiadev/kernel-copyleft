// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <media/cam_icp.h>

#include "cam_io_util.h"
#include "cam_hw.h"
#include "cam_hw_intf.h"
#include "cam_icp_v1_core.h"
#include "cam_icp_v1_reg.h"
#include "cam_soc_util.h"
#include "cam_io_util.h"
#include "hfi_sys_defs.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_cpas_api.h"
#include "cam_debug_util.h"
#include "cam_icp_proc_common.h"
#include "cam_common_util.h"
#include "cam_compat.h"
#include "cam_icp_soc_common.h"

static const struct hfi_ops hfi_icp_v1_ops = {
	.irq_raise = cam_icp_v1_irq_raise,
	.irq_enable = cam_icp_v1_irq_enable,
	.iface_addr = cam_icp_v1_iface_addr,
};

static int cam_icp_v1_cpas_vote(struct cam_icp_v1_device_core_info *core_info,
	struct cam_icp_cpas_vote *cpas_vote)
{
	if (!core_info)
		return -EINVAL;

	return cam_icp_proc_cpas_vote(core_info->cpas_handle, cpas_vote);
}

static int32_t cam_icp_v1_download_fw(void *device_priv)
{
	int32_t rc = 0;
	uint32_t fw_size;
	const uint8_t *fw_start = NULL;
	struct cam_hw_info *icp_v1_dev = device_priv;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_icp_v1_device_core_info *core_info = NULL;
	struct platform_device         *pdev = NULL;
	const char                     *firmware_name = NULL;

	if (!device_priv) {
		CAM_ERR(CAM_ICP, "Invalid cam_dev_info");
		return -EINVAL;
	}

	soc_info = &icp_v1_dev->soc_info;
	core_info = (struct cam_icp_v1_device_core_info *)icp_v1_dev->core_info;
	pdev = soc_info->pdev;

	rc = of_property_read_string(pdev->dev.of_node, "fw_name", &firmware_name);
	if (rc) {
		CAM_INFO(CAM_ICP,
			"FW image name not found. Use default name: CAMERA_ICP.elf");
		rc = firmware_request_nowarn(&core_info->fw_elf,
			"CAMERA_ICP.elf", &pdev->dev);
		if (rc) {
			CAM_ERR(CAM_ICP, "Failed to locate fw: %d", rc);
			return rc;
		}
	} else {
		CAM_INFO(CAM_ICP, "Downloading firmware %s",
			firmware_name);
		rc = firmware_request_nowarn(&core_info->fw_elf,
			firmware_name, &pdev->dev);
		if (rc) {
			CAM_ERR(CAM_ICP, "Failed to locate fw: %d", rc);
			return rc;
		}
	}

	if (!core_info->fw_elf) {
		CAM_ERR(CAM_ICP, "Invalid elf size");
		rc = -EINVAL;
		goto fw_download_failed;
	}

	fw_start = core_info->fw_elf->data;
	rc = cam_icp_validate_fw(fw_start, EM_ARM);
	if (rc) {
		CAM_ERR(CAM_ICP, "fw elf validation failed");
		goto fw_download_failed;
	}

	rc = cam_icp_get_fw_size(fw_start, &fw_size);
	if (rc) {
		CAM_ERR(CAM_ICP, "unable to get fw size");
		goto fw_download_failed;
	}

	if (core_info->fw_buf_len < fw_size) {
		CAM_ERR(CAM_ICP, "mismatch in fw size: %u %llu",
			fw_size, core_info->fw_buf_len);
		rc = -EINVAL;
		goto fw_download_failed;
	}

	rc = cam_icp_program_fw(fw_start, core_info->fw_kva_addr);
	if (rc) {
		CAM_ERR(CAM_ICP, "fw program is failed");
		goto fw_download_failed;
	}

fw_download_failed:
	release_firmware(core_info->fw_elf);
	return rc;
}

static int cam_icp_v1_fw_dump(
	struct cam_icp_hw_dump_args    *dump_args,
	struct cam_icp_v1_device_core_info *core_info)
{
	u8                         *dest;
	u8                         *src;
	uint64_t                    size_required;
	struct cam_icp_dump_header *hdr;

	if (!core_info || !dump_args) {
		CAM_ERR(CAM_ICP, "invalid params %pK %pK",
			core_info, dump_args);
		return -EINVAL;
	}
	if (!core_info->fw_kva_addr || !dump_args->cpu_addr) {
		CAM_ERR(CAM_ICP, "invalid params %pK, 0x%zx",
			core_info->fw_kva_addr, dump_args->cpu_addr);
		return -EINVAL;
	}

	size_required = core_info->fw_buf_len +
		sizeof(struct cam_icp_dump_header);

	if (dump_args->buf_len <= dump_args->offset) {
		CAM_WARN(CAM_ICP, "Dump offset overshoot len %zu offset %zu",
			dump_args->buf_len, dump_args->offset);
		return -ENOSPC;
	}

	if ((dump_args->buf_len - dump_args->offset) < size_required) {
		CAM_WARN(CAM_ICP, "Dump buffer exhaust required %llu len %llu",
			size_required, core_info->fw_buf_len);
		return -ENOSPC;
	}

	dest = (u8 *)dump_args->cpu_addr + dump_args->offset;
	hdr = (struct cam_icp_dump_header *)dest;
	scnprintf(hdr->tag, CAM_ICP_DUMP_TAG_MAX_LEN, "ICP_FW:");
	hdr->word_size = sizeof(u8);
	hdr->size = core_info->fw_buf_len;
	src = (u8 *)core_info->fw_kva_addr;
	dest = (u8 *)dest + sizeof(struct cam_icp_dump_header);
	memcpy_fromio(dest, src, core_info->fw_buf_len);
	dump_args->offset += hdr->size + sizeof(struct cam_icp_dump_header);
	return 0;
}

static int cam_icp_v1_fw_mini_dump(struct cam_icp_hw_dump_args *dump_args,
	struct cam_icp_v1_device_core_info *core_info)
{
	if (!core_info) {
		CAM_ERR(CAM_ICP, "Invalid param %pK", core_info);
		return -EINVAL;
	}

	return cam_icp_proc_mini_dump(dump_args, core_info->fw_kva_addr,
		core_info->fw_buf_len);
}

int cam_icp_v1_init_hw(void *device_priv, void *args,
	uint32_t arg_size)
{
	struct cam_hw_info *icp_v1_dev = device_priv;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_icp_v1_device_core_info *core_info = NULL;
	struct cam_icp_soc_info *icp_v1_soc_info;
	struct cam_icp_cpas_vote cpas_vote;
	unsigned long flags;
	int rc = 0;
	bool send_freq_info;

	if (!device_priv) {
		CAM_ERR(CAM_ICP, "Invalid cam_dev_info");
		return -EINVAL;
	}

	soc_info = &icp_v1_dev->soc_info;
	core_info = (struct cam_icp_v1_device_core_info *)icp_v1_dev->core_info;

	if ((!soc_info) || (!core_info) || (!args)) {
		CAM_ERR(CAM_ICP, "soc_info: %pK core_info: %pK args: %pK",
			soc_info, core_info, args);
		return -EINVAL;
	}

	send_freq_info = *((bool *)args);

	spin_lock_irqsave(&icp_v1_dev->hw_lock, flags);
	if (icp_v1_dev->hw_state == CAM_HW_STATE_POWER_UP) {
		spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);

	icp_v1_soc_info = soc_info->soc_private;

	cpas_vote.ahb_vote.type = CAM_VOTE_ABSOLUTE;
	cpas_vote.ahb_vote.vote.level = CAM_LOWSVS_D1_VOTE;
	cpas_vote.axi_vote.num_paths = 1;
	cpas_vote.axi_vote.axi_path[0].path_data_type =
		CAM_ICP_DEFAULT_AXI_PATH;
	cpas_vote.axi_vote.axi_path[0].transac_type =
		CAM_ICP_DEFAULT_AXI_TRANSAC;
	cpas_vote.axi_vote.axi_path[0].camnoc_bw =
		CAM_ICP_BW_BYTES_VOTE;
	cpas_vote.axi_vote.axi_path[0].mnoc_ab_bw =
		CAM_ICP_BW_BYTES_VOTE;
	cpas_vote.axi_vote.axi_path[0].mnoc_ib_bw =
		CAM_ICP_BW_BYTES_VOTE;

	rc = cam_cpas_start(core_info->cpas_handle,
		&cpas_vote.ahb_vote, &cpas_vote.axi_vote);
	if (rc) {
		CAM_ERR(CAM_ICP, "cpas start failed: %d", rc);
		goto error;
	}
	core_info->cpas_start = true;

	rc = cam_icp_soc_resources_enable(soc_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "soc enable is failed: %d", rc);
		if (cam_cpas_stop(core_info->cpas_handle))
			CAM_ERR(CAM_ICP, "cpas stop is failed");
		else
			core_info->cpas_start = false;
	} else {
		CAM_DBG(CAM_ICP, "icp_v1_qos %d", icp_v1_soc_info->qos_val);
		if (icp_v1_soc_info->qos_val)
			cam_io_w_mb(icp_v1_soc_info->qos_val,
				soc_info->reg_map[ICP_V1_BASE].mem_base +
				ICP_V1_CSR_ACCESS);
		if (send_freq_info) {
			int32_t clk_rate = 0;

			clk_rate = cam_wrapper_clk_get_rate(soc_info->clk[soc_info->src_clk_idx]);
			hfi_send_freq_info(core_info->hfi_handle, clk_rate);
		}
	}

	spin_lock_irqsave(&icp_v1_dev->hw_lock, flags);
	icp_v1_dev->hw_state = CAM_HW_STATE_POWER_UP;
	spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);

error:
	return rc;
}

int cam_icp_v1_deinit_hw(void *device_priv,
	void *args, uint32_t arg_size)
{
	struct cam_hw_info *icp_v1_dev = device_priv;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_icp_v1_device_core_info *core_info = NULL;
	unsigned long flags;
	int rc = 0;
	bool send_freq_info;

	if (!device_priv || !args) {
		CAM_ERR(CAM_ICP, "Invalid icp hw info is %s args is %s",
			CAM_IS_NULL_TO_STR(icp_v1_dev), CAM_IS_NULL_TO_STR(args));
		return -EINVAL;
	}

	send_freq_info = *((bool *)args);
	soc_info = &icp_v1_dev->soc_info;
	core_info = (struct cam_icp_v1_device_core_info *)icp_v1_dev->core_info;
	if ((!soc_info) || (!core_info)) {
		CAM_ERR(CAM_ICP, "soc_info = %pK core_info = %pK",
			soc_info, core_info);
		return -EINVAL;
	}

	spin_lock_irqsave(&icp_v1_dev->hw_lock, flags);
	if (icp_v1_dev->hw_state == CAM_HW_STATE_POWER_DOWN) {
		spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);

	if (send_freq_info)
		hfi_send_freq_info(core_info->hfi_handle, 0);

	rc = cam_icp_soc_resources_disable(soc_info);

	if (rc)
		CAM_ERR(CAM_ICP, "soc disable is failed: %d", rc);

	if (core_info->cpas_start) {
		if (cam_cpas_stop(core_info->cpas_handle))
			CAM_ERR(CAM_ICP, "cpas stop is failed");
		else
			core_info->cpas_start = false;
	}

	spin_lock_irqsave(&icp_v1_dev->hw_lock, flags);
	icp_v1_dev->hw_state = CAM_HW_STATE_POWER_DOWN;
	spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);

	return rc;
}

static int cam_icp_v1_power_resume(struct cam_hw_info *icp_v1_info, bool debug_enabled)
{
	uint32_t val = ICP_V1_CSR_FULL_CPU_EN;
	void __iomem *base;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_icp_soc_info *icp_v1_soc_info;

	if (!icp_v1_info) {
		CAM_ERR(CAM_ICP, "Invalid ICP device info");
		return -EINVAL;
	}

	base = icp_v1_info->soc_info.reg_map[ICP_V1_BASE].mem_base;
	soc_info = &icp_v1_info->soc_info;
	icp_v1_soc_info = soc_info->soc_private;

	cam_io_w_mb(ICP_V1_CSR_CPU_EN, base + ICP_V1_CSR_CONTROL);
	cam_io_w_mb(ICP_V1_CSR_FUNC_RESET, base + ICP_V1_CSR_NSEC_RESET);

	if (debug_enabled)
		val |= ICP_V1_CSR_FULL_DBG_EN;

	cam_io_w_mb(val, base + ICP_V1_CSR_CONTROL);
	cam_io_w_mb(icp_v1_soc_info->qos_val,
		base + ICP_V1_CSR_ACCESS);

	CAM_DBG(CAM_ICP, "icp_v1 qos-val : 0x%x",
		cam_io_r_mb(base + ICP_V1_CSR_ACCESS));

	return 0;
}

static int cam_icp_v1_power_collapse(struct cam_hw_info *icp_v1_info)
{
	uint32_t val, status = 0;
	void __iomem *base;

	if (!icp_v1_info) {
		CAM_ERR(CAM_ICP, "invalid ICP device info");
		return -EINVAL;
	}

	base = icp_v1_info->soc_info.reg_map[ICP_V1_BASE].mem_base;

	/**
	 * Need to poll here to confirm that FW has triggered WFI
	 * and Host can then proceed. No interrupt is expected
	 * from FW at this time.
	 */

	if (cam_common_read_poll_timeout(base +
			ICP_V1_CSR_STATUS,
			PC_POLL_DELAY_US, PC_POLL_TIMEOUT_US,
			ICP_V1_CSR_STANDBYWFI,
			ICP_V1_CSR_STANDBYWFI, &status)) {
		CAM_ERR(CAM_ICP, "WFI poll timed out: status=0x%08x", status);
		return -ETIMEDOUT;
	}

	val = cam_io_r(base + ICP_V1_CSR_CONTROL);
	val &= ~(ICP_V1_CSR_CPU_EN | ICP_V1_CSR_WAKE_UP_EN);
	cam_io_w_mb(val, base + ICP_V1_CSR_CONTROL);

	return 0;
}

static void prepare_boot(struct cam_hw_info *icp_v1_dev,
	struct cam_icp_boot_args *args)
{
	struct cam_icp_v1_device_core_info *core_info = icp_v1_dev->core_info;
	unsigned long flags;

	core_info->fw_buf = args->firmware.iova;
	core_info->fw_kva_addr = args->firmware.kva;
	core_info->fw_buf_len = args->firmware.len;

	spin_lock_irqsave(&icp_v1_dev->hw_lock, flags);
	core_info->irq_cb.data = args->irq_cb.data;
	core_info->irq_cb.cb = args->irq_cb.cb;
	spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);
}

static void prepare_shutdown(struct cam_hw_info *icp_v1_dev)
{
	struct cam_icp_v1_device_core_info *core_info = icp_v1_dev->core_info;
	unsigned long flags;

	core_info->fw_buf = 0;
	core_info->fw_kva_addr = 0;
	core_info->fw_buf_len = 0;

	spin_lock_irqsave(&icp_v1_dev->hw_lock, flags);
	core_info->irq_cb.data = NULL;
	core_info->irq_cb.cb = NULL;
	spin_unlock_irqrestore(&icp_v1_dev->hw_lock, flags);
}

static int cam_icp_v1_boot(struct cam_hw_info *icp_v1_dev,
	struct cam_icp_boot_args *args, size_t arg_size)
{
	int rc;

	if (!icp_v1_dev || !args) {
		CAM_ERR(CAM_ICP,
			"Invalid args: icp_v1_dev=%pK args=%pK",
			icp_v1_dev, args);
		return -EINVAL;
	}

	if (arg_size != sizeof(struct cam_icp_boot_args)) {
		CAM_ERR(CAM_ICP, "Invalid boot args size");
		return -EINVAL;
	}

	prepare_boot(icp_v1_dev, args);

	rc = cam_icp_v1_download_fw(icp_v1_dev);
	if (rc) {
		CAM_ERR(CAM_ICP, "firmware download failed rc=%d", rc);
		goto err;
	}

	rc = cam_icp_v1_power_resume(icp_v1_dev, args->debug_enabled);
	if (rc) {
		CAM_ERR(CAM_ICP, "ICP boot failed rc=%d", rc);
		goto err;
	}

	return 0;
err:
	prepare_shutdown(icp_v1_dev);
	return rc;
}

static int cam_icp_v1_shutdown(struct cam_hw_info *icp_v1_dev)
{
	if (!icp_v1_dev) {
		CAM_ERR(CAM_ICP, "invalid ICP device info");
		return -EINVAL;
	}

	prepare_shutdown(icp_v1_dev);
	cam_icp_v1_power_collapse(icp_v1_dev);
	return 0;
}

irqreturn_t cam_icp_v1_irq(int irq_num, void *data)
{
	struct cam_hw_info *icp_v1_dev = data;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_icp_v1_device_core_info *core_info = NULL;
	uint32_t irq_status = 0;
	bool recover = false;

	if (!data) {
		CAM_ERR(CAM_ICP, "Invalid cam_dev_info or query_cap args");
		return IRQ_HANDLED;
	}

	spin_lock(&icp_v1_dev->hw_lock);
	if (icp_v1_dev->hw_state == CAM_HW_STATE_POWER_DOWN) {
		CAM_WARN(CAM_ICP, "ICP HW powered off");
		spin_unlock(&icp_v1_dev->hw_lock);
		return IRQ_HANDLED;
	}
	spin_unlock(&icp_v1_dev->hw_lock);

	soc_info = &icp_v1_dev->soc_info;
	core_info = (struct cam_icp_v1_device_core_info *)icp_v1_dev->core_info;

	irq_status = cam_io_r_mb(soc_info->reg_map[ICP_V1_BASE].mem_base +
		ICP_V1_HOST_INT_STATUS);

	cam_io_w_mb(irq_status,
		soc_info->reg_map[ICP_V1_BASE].mem_base + ICP_V1_HOST_INT_CLR);

	if ((irq_status & ICP_V1_WDT_0) ||
		(irq_status & ICP_V1_WDT_1)) {
		CAM_ERR_RATE_LIMIT(CAM_ICP, "watch dog interrupt from ICP");
		recover = true;
	}

	spin_lock(&icp_v1_dev->hw_lock);
	if (core_info->irq_cb.cb)
		core_info->irq_cb.cb(core_info->irq_cb.data, recover);
	spin_unlock(&icp_v1_dev->hw_lock);

	return IRQ_HANDLED;
}

void cam_icp_v1_irq_raise(void *priv)
{
	struct cam_hw_info *icp_v1_info = priv;

	if (!icp_v1_info) {
		CAM_ERR(CAM_ICP, "invalid ICP device info");
		return;
	}

	cam_io_w_mb(ICP_V1_HOSTINT,
		icp_v1_info->soc_info.reg_map[ICP_V1_BASE].mem_base +
		ICP_V1_CSR_HOST2ICPINT);
}

void cam_icp_v1_irq_enable(void *priv)
{
	struct cam_hw_info *icp_v1_info = priv;

	if (!icp_v1_info) {
		CAM_ERR(CAM_ICP, "invalid ICP device info");
		return;
	}

	cam_io_w_mb(ICP_V1_WDT_WS0EN | ICP_V1_A2HOSTINTEN,
		icp_v1_info->soc_info.reg_map[ICP_V1_BASE].mem_base +
		ICP_V1_CSR_A2HOSTINTEN);
}

void __iomem *cam_icp_v1_iface_addr(void *priv)
{
	struct cam_hw_info *icp_v1_info = priv;
	void __iomem *base;

	if (!icp_v1_info) {
		CAM_ERR(CAM_ICP, "invalid ICP device info");
		return ERR_PTR(-EINVAL);
	}

	base = icp_v1_info->soc_info.reg_map[ICP_V1_BASE].mem_base;

	return base + ICP_V1_GEN_PURPOSE_REG_OFFSET;
}

void cam_icp_v1_populate_hfi_ops(const struct hfi_ops **hfi_proc_ops)
{
	if (!hfi_proc_ops) {
		CAM_ERR(CAM_ICP, "Invalid hfi ops argument");
		return;
	}

	*hfi_proc_ops = &hfi_icp_v1_ops;
}

static int cam_icp_v1_send_fw_init(struct cam_icp_v1_device_core_info *core_info)
{
	int rc;

	if (!core_info) {
		CAM_ERR(CAM_ICP, "Invalid core info is NULL");
		return -EINVAL;
	}

	rc = hfi_send_system_cmd(core_info->hfi_handle, HFI_CMD_SYS_INIT, 0, 0);
	if (rc) {
		CAM_ERR(CAM_ICP, "Fail to send sys init command for hfi handle: %d",
			core_info->hfi_handle);
		return rc;
	}

	return 0;
}

static int cam_icp_v1_pc_prep(struct cam_icp_v1_device_core_info *core_info)
{
	int rc;

	if (!core_info) {
		CAM_ERR(CAM_ICP, "Invalid ICP core info is NULL");
		return -EINVAL;
	}

	rc = hfi_send_system_cmd(core_info->hfi_handle, HFI_CMD_SYS_PC_PREP, 0, 0);
	if (rc) {
		CAM_ERR(CAM_ICP, "Fail to send PC collapse command for hfi handle: %d",
			core_info->hfi_handle);
		return rc;
	}

	return 0;
}

int cam_icp_v1_process_cmd(void *device_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_hw_info *icp_v1_dev = device_priv;
	struct cam_hw_soc_info *soc_info = NULL;
	struct cam_icp_v1_device_core_info *core_info = NULL;
	struct cam_icp_soc_info *icp_soc_info = NULL;
	int rc = 0;

	if (!device_priv) {
		CAM_ERR(CAM_ICP, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd_type >= CAM_ICP_CMD_MAX) {
		CAM_ERR(CAM_ICP, "Invalid command : %x", cmd_type);
		return -EINVAL;
	}

	soc_info = &icp_v1_dev->soc_info;
	core_info = (struct cam_icp_v1_device_core_info *)icp_v1_dev->core_info;

	switch (cmd_type) {
	case CAM_ICP_CMD_PROC_SHUTDOWN:
		rc = cam_icp_v1_shutdown(device_priv);
		break;
	case CAM_ICP_CMD_PROC_BOOT:
		rc = cam_icp_v1_boot(device_priv, cmd_args, arg_size);
		break;
	case CAM_ICP_CMD_POWER_COLLAPSE:
		rc = cam_icp_v1_power_collapse(icp_v1_dev);
		break;
	case CAM_ICP_CMD_POWER_RESUME:
		rc = cam_icp_v1_power_resume(icp_v1_dev, *((bool *)cmd_args));
		break;
	case CAM_ICP_SEND_INIT:
		rc = cam_icp_v1_send_fw_init(core_info);
		break;
	case CAM_ICP_CMD_PC_PREP:
		rc = cam_icp_v1_pc_prep(core_info);
		break;
	case CAM_ICP_CMD_VOTE_CPAS: {
		struct cam_icp_cpas_vote *cpas_vote = cmd_args;

		if (!cmd_args) {
			CAM_ERR(CAM_ICP, "cmd args NULL");
			return -EINVAL;
		}

		cam_icp_v1_cpas_vote(core_info, cpas_vote);
		break;
	}

	case CAM_ICP_CMD_SET_HFI_HANDLE: {
		if (!core_info || !cmd_args) {
			CAM_ERR(CAM_ICP, "Core info is %s and args is %s",
				CAM_IS_NULL_TO_STR(core_info), CAM_IS_NULL_TO_STR(cmd_args));
			return -EINVAL;
		}

		if (arg_size != sizeof(int)) {
			CAM_ERR(CAM_ICP, "Invalid set hfi handle command arg size:%u",
				arg_size);
			return -EINVAL;
		}

		core_info->hfi_handle = *((int *)cmd_args);
		break;
	}

	case CAM_ICP_CMD_CPAS_START: {
		struct cam_icp_cpas_vote *cpas_vote = cmd_args;

		if (!cmd_args) {
			CAM_ERR(CAM_ICP, "cmd args NULL");
			return -EINVAL;
		}

		if (!core_info->cpas_start) {
			rc = cam_cpas_start(core_info->cpas_handle,
				&cpas_vote->ahb_vote,
				&cpas_vote->axi_vote);
			core_info->cpas_start = true;
		}
		break;
	}

	case CAM_ICP_CMD_CPAS_STOP:
		if (core_info->cpas_start) {
			cam_cpas_stop(core_info->cpas_handle);
			core_info->cpas_start = false;
		}
		break;
	case CAM_ICP_CMD_UBWC_CFG: {
		struct cam_icp_ubwc_cfg_cmd *ubwc_cmd = cmd_args;
		struct cam_icp_proc_ubwc_cfg_cmd ubwc_proc_cmd;

		icp_soc_info = soc_info->soc_private;
		if (!icp_soc_info) {
			CAM_ERR(CAM_ICP, "ICP private soc info is NULL");
			return -EINVAL;
		}

		if (!cmd_args) {
			CAM_ERR(CAM_ICP, "Invalid ubwc cmd args is NULL");
			return -EINVAL;
		}

		if (arg_size != sizeof(struct cam_icp_ubwc_cfg_cmd)) {
			CAM_ERR(CAM_ICP, "Invalid ubwc cmd size:%u", arg_size);
			return -EINVAL;
		}

		if (icp_soc_info->is_ubwc_cfg)
			rc = hfi_cmd_ubwc_config(core_info->hfi_handle,
				icp_soc_info->uconfig.ubwc_cfg);
		else {
			ubwc_proc_cmd.ubwc_cfg = &icp_soc_info->uconfig.ubwc_cfg_ext;
			rc = cam_icp_proc_ubwc_configure(&ubwc_proc_cmd,
				ubwc_cmd->disable_ubwc_comp, core_info->hfi_handle);
		}
		break;
	}
	case CAM_ICP_CMD_CLK_UPDATE: {
		int32_t clk_level = 0;
		struct cam_ahb_vote ahb_vote;

		if (!cmd_args || !core_info) {
			CAM_ERR(CAM_ICP, "Invalid args: core info is %s cmd args is %s",
				CAM_IS_NULL_TO_STR(core_info), CAM_IS_NULL_TO_STR(cmd_args));
			return -EINVAL;
		}

		if (arg_size != sizeof(int32_t)) {
			CAM_ERR(CAM_ICP, "Invalid icp clock update command");
			return -EINVAL;
		}

		clk_level = *((int32_t *)cmd_args);
		CAM_DBG(CAM_ICP,
			"Update ICP clock to level [%d]", clk_level);
		rc = cam_icp_soc_update_clk_rate(soc_info, clk_level, core_info->hfi_handle);
		if (rc)
			CAM_ERR(CAM_ICP,
				"Failed to update clk to level: %d rc: %d",
				clk_level, rc);

		ahb_vote.type = CAM_VOTE_ABSOLUTE;
		ahb_vote.vote.level = clk_level;
		cam_cpas_update_ahb_vote(
			core_info->cpas_handle, &ahb_vote);
		break;
	}
	case CAM_ICP_CMD_HW_DUMP: {
		struct cam_icp_hw_dump_args *dump_args = cmd_args;

		rc = cam_icp_v1_fw_dump(dump_args, core_info);
		break;
	}

	case CAM_ICP_CMD_HW_MINI_DUMP: {
		struct cam_icp_hw_dump_args *dump_args = cmd_args;

		rc = cam_icp_v1_fw_mini_dump(dump_args, core_info);
		break;
	}
	case CAM_ICP_CMD_HW_REG_DUMP: {
		/* reg dump not supported */
		break;
	}
	default:
		break;
	}

	return rc;
}
