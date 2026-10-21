// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of_address.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "cam_cpas_api.h"
#include "cam_debug_util.h"
#include "cam_hw.h"
#include "cam_hw_intf.h"
#include "cam_icp_hw_mgr_intf.h"
#include "cam_icp_hw_intf.h"
#include "hfi_intf.h"
#include "hfi_sys_defs.h"
#include "cam_icp_proc_common.h"
#include "cam_icp_v2_core.h"
#include "cam_common_util.h"
#include "cam_compat.h"
#include "cam_presil_hw_access.h"
#include "cam_icp_soc_common.h"

#define TZ_STATE_SUSPEND 0
#define TZ_STATE_RESUME  1

#define ICP_FW_NAME_MAX_SIZE    32

#define ICP_V2_IRQ_TEST_TIMEOUT 1000

static const struct hfi_ops hfi_icp_v2_ops = {
	.irq_raise = cam_icp_v2_irq_raise,
	.irq_enable = cam_icp_v2_irq_enable,
	.iface_addr = cam_icp_v2_iface_addr,
};

static int cam_icp_v2_ubwc_configure(struct cam_hw_soc_info *soc_info)
{
	struct cam_icp_soc_info *soc_priv;

	if (!soc_info)
		return -EINVAL;

	soc_priv = soc_info->soc_private;

	return cam_icp_proc_ubwc_configure(soc_priv->uconfig.ubwc_cfg_ext, 0);
}

static int cam_icp_v2_cpas_vote(struct cam_icp_v2_core_info *core_info,
	struct cam_icp_cpas_vote *vote)
{
	if (!core_info)
		return -EINVAL;

	return cam_icp_proc_cpas_vote(core_info->cpas_handle, vote);
}

static bool cam_icp_v2_cpas_cb(uint32_t handle, void *user_data,
	struct cam_cpas_irq_data *irq_data)
{
	bool ret = false;
	(void)user_data;

	if (!irq_data)
		return false;

	switch (irq_data->irq_type) {
	case CAM_CAMNOC_IRQ_IPE_BPS_UBWC_DECODE_ERROR:
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"IPE/BPS UBWC decode error status=0x%08x",
			irq_data->u.dec_err.decerr_status.value);
		ret = true;
		break;
	case CAM_CAMNOC_IRQ_IPE_BPS_UBWC_ENCODE_ERROR:
		CAM_ERR_RATE_LIMIT(CAM_ICP,
			"IPE/BPS UBWC encode error status=0x%08x",
			irq_data->u.enc_err.encerr_status.value);
		ret = true;
		break;
	default:
		break;
	}

	return ret;
}

int cam_icp_v2_cpas_register(struct cam_hw_intf *icp_v2_intf)
{
	struct cam_cpas_register_params params;
	struct cam_hw_info *icp_v2_info;
	struct cam_icp_v2_core_info *core_info;
	int rc;

	if (!icp_v2_intf)
		return -EINVAL;

	icp_v2_info = icp_v2_intf->hw_priv;

	params.dev = icp_v2_info->soc_info.dev;
	params.cell_index = icp_v2_intf->hw_idx;
	params.cam_cpas_client_cb = cam_icp_v2_cpas_cb;
	params.userdata = NULL;

	strlcpy(params.identifier, "icp", CAM_HW_IDENTIFIER_LENGTH);

	rc = cam_cpas_register_client(&params);
	if (rc)
		return rc;

	core_info = icp_v2_info->core_info;
	core_info->cpas_handle = params.client_handle;

	return rc;
}

int cam_icp_v2_cpas_unregister(struct cam_hw_intf *icp_v2_intf)
{
	struct cam_hw_info *icp_v2_info;
	struct cam_icp_v2_core_info *core_info;

	if (!icp_v2_intf)
		return -EINVAL;

	icp_v2_info = icp_v2_intf->hw_priv;
	core_info = icp_v2_info->core_info;

	return cam_cpas_unregister_client(core_info->cpas_handle);
}

static int __icp_v2_cpas_start(struct cam_icp_v2_core_info *core_info,
	struct cam_icp_cpas_vote *vote)
{
	int rc;

	if (!core_info || core_info->cpas_start)
		return -EINVAL;

	rc = cam_cpas_start(core_info->cpas_handle,
		&vote->ahb_vote, &vote->axi_vote);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to start cpas rc=%d", rc);
		return rc;
	}

	core_info->cpas_start = true;

	return 0;
}

static int cam_icp_v2_cpas_start(struct cam_icp_v2_core_info *core_info)
{
	struct cam_icp_cpas_vote vote;

	vote.ahb_vote.type = CAM_VOTE_ABSOLUTE;
	vote.ahb_vote.vote.level = CAM_LOWSVS_VOTE;
	vote.axi_vote.num_paths = 1;

	vote.axi_vote.axi_path[0].path_data_type = CAM_ICP_DEFAULT_AXI_PATH;
	vote.axi_vote.axi_path[0].transac_type = CAM_ICP_DEFAULT_AXI_TRANSAC;
	vote.axi_vote.axi_path[0].camnoc_bw = CAM_ICP_BW_BYTES_VOTE;
	vote.axi_vote.axi_path[0].mnoc_ab_bw = CAM_ICP_BW_BYTES_VOTE;
	vote.axi_vote.axi_path[0].mnoc_ib_bw = CAM_ICP_BW_BYTES_VOTE;

	return __icp_v2_cpas_start(core_info, &vote);
}

static int cam_icp_v2_cpas_stop(struct cam_icp_v2_core_info *core_info)
{
	int rc;

	if (!core_info || !core_info->cpas_start)
		return -EINVAL;

	rc = cam_cpas_stop(core_info->cpas_handle);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to stop cpas rc=%d", rc);
		return rc;
	}

	core_info->cpas_start = false;

	return 0;
}

int cam_icp_v2_hw_init(void *priv, void *args, uint32_t arg_size)
{
	struct cam_hw_info *icp_v2 = priv;
	unsigned long flags;
	int rc;
	bool send_freq_info = (args == NULL) ? false : *((bool *)args);

	if (!icp_v2) {
		CAM_ERR(CAM_ICP, "ICP device info cannot be NULL");
		return -EINVAL;
	}

	spin_lock_irqsave(&icp_v2->hw_lock, flags);
	if (icp_v2->hw_state == CAM_HW_STATE_POWER_UP) {
		spin_unlock_irqrestore(&icp_v2->hw_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&icp_v2->hw_lock, flags);

	rc = cam_icp_v2_cpas_start(icp_v2->core_info);
	if (rc)
		return rc;

	rc = cam_icp_soc_resources_enable(&icp_v2->soc_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to enable soc resources rc=%d", rc);
		goto soc_fail;
	} else {
		if (send_freq_info) {
			int32_t clk_rate = 0;

			clk_rate = clk_get_rate(icp_v2->soc_info.clk[icp_v2->soc_info.src_clk_idx]);
			hfi_send_freq_info(clk_rate);
		}
	}

	spin_lock_irqsave(&icp_v2->hw_lock, flags);
	icp_v2->hw_state = CAM_HW_STATE_POWER_UP;
	spin_unlock_irqrestore(&icp_v2->hw_lock, flags);

	return 0;

soc_fail:
	cam_icp_v2_cpas_stop(icp_v2->core_info);
	return rc;
}

int cam_icp_v2_hw_deinit(void *priv, void *args, uint32_t arg_size)
{
	struct cam_hw_info *icp_v2_info = priv;
	unsigned long flags;
	int rc;
	bool send_freq_info = (args == NULL) ? false : *((bool *)args);

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "ICP device info cannot be NULL");
		return -EINVAL;
	}

	spin_lock_irqsave(&icp_v2_info->hw_lock, flags);
	if (icp_v2_info->hw_state == CAM_HW_STATE_POWER_DOWN) {
		spin_unlock_irqrestore(&icp_v2_info->hw_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&icp_v2_info->hw_lock, flags);

	if (send_freq_info)
		hfi_send_freq_info(0);

	rc = cam_icp_soc_resources_disable(&icp_v2_info->soc_info);
	if (rc)
		CAM_WARN(CAM_ICP,
			"failed to disable soc resources rc=%d", rc);

	rc = cam_icp_v2_cpas_stop(icp_v2_info->core_info);
	if (rc)
		CAM_WARN(CAM_ICP, "cpas stop failed rc=%d", rc);

	spin_lock_irqsave(&icp_v2_info->hw_lock, flags);
	icp_v2_info->hw_state = CAM_HW_STATE_POWER_DOWN;
	spin_unlock_irqrestore(&icp_v2_info->hw_lock, flags);

	return rc;
}

static void prepare_boot(struct cam_hw_info *icp_v2_info,
	struct cam_icp_boot_args *args)
{
	struct cam_icp_v2_core_info *core_info = icp_v2_info->core_info;
	unsigned long flags;

	if (!args->use_sec_pil) {
		core_info->fw_params.fw_buf = args->firmware.iova;
		core_info->fw_params.fw_kva_addr = args->firmware.kva;
		core_info->fw_params.fw_buf_len = args->firmware.len;
	}

	spin_lock_irqsave(&icp_v2_info->hw_lock, flags);
	core_info->irq_cb.data = args->irq_cb.data;
	core_info->irq_cb.cb = args->irq_cb.cb;
	spin_unlock_irqrestore(&icp_v2_info->hw_lock, flags);
}

static void prepare_shutdown(struct cam_hw_info *icp_v2_info)
{
	struct cam_icp_v2_core_info *core_info = icp_v2_info->core_info;
	unsigned long flags;

	core_info->fw_params.fw_buf = 0x0;
	core_info->fw_params.fw_kva_addr = 0x0;
	core_info->fw_params.fw_buf_len = 0x0;

	spin_lock_irqsave(&icp_v2_info->hw_lock, flags);
	core_info->irq_cb.data = NULL;
	core_info->irq_cb.cb = NULL;
	spin_unlock_irqrestore(&icp_v2_info->hw_lock, flags);
}

/* Used if ICP_SYS is not protected */
static int __cam_icp_v2_power_collapse(struct cam_hw_info *icp_v2_info)
{
	int32_t sys_base_idx;
	uint32_t status = 0;
	void __iomem *base;
	struct cam_icp_v2_core_info *core_info = NULL;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "invalid ICP dev info");
		return -EINVAL;
	}

	core_info = icp_v2_info->core_info;
	sys_base_idx = core_info->reg_base_idx[ICP_V2_SYS_BASE];

	if (sys_base_idx < 0) {
		CAM_ERR(CAM_ICP, "No reg base idx found for ICP_SYS: %d",
			sys_base_idx);
		return -EINVAL;
	}

	base = icp_v2_info->soc_info.reg_map[sys_base_idx].mem_base;

	/**
	 * Need to poll here to confirm that FW has triggered WFI
	 * and Host can then proceed. No interrupt is expected
	 * from FW at this time.
	 */
	if (cam_common_read_poll_timeout(base + ICP_V2_SYS_STATUS,
		PC_POLL_DELAY_US, PC_POLL_TIMEOUT_US,
		ICP_V2_STANDBYWFI, ICP_V2_STANDBYWFI,
		&status)) {
		CAM_ERR(CAM_ICP, "WFI poll timed out: status=0x%08x", status);
		return -ETIMEDOUT;
	}

	cam_io_w_mb(0x0, base + ICP_V2_SYS_CONTROL);
	return 0;
}

/* Used if ICP_SYS and ICP_DOM_MASK are not protected */
static int __cam_icp_v2_power_resume(struct cam_hw_info *icp_v2_info)
{
	int32_t sys_base_idx, dom_mask_base_idx;
	void __iomem *sys_base, *dom_mask_base;
	struct cam_icp_soc_info     *soc_priv;
	struct cam_hw_soc_info      *soc_info;
	struct cam_icp_v2_core_info *core_info = NULL;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "invalid ICP dev info");
		return -EINVAL;
	}

	soc_info = &icp_v2_info->soc_info;
	core_info = icp_v2_info->core_info;
	soc_priv = soc_info->soc_private;
	sys_base_idx = core_info->reg_base_idx[ICP_V2_SYS_BASE];
	dom_mask_base_idx = core_info->reg_base_idx[ICP_V2_DOM_MASK_BASE];

	if (sys_base_idx < 0) {
		CAM_ERR(CAM_ICP, "No reg base idx found for ICP_SYS: %d",
			sys_base_idx);
		return -EINVAL;
	}

	sys_base = icp_v2_info->soc_info.reg_map[sys_base_idx].mem_base;

	cam_io_w_mb(ICP_V2_FUNC_RESET,
		sys_base + ICP_V2_SYS_RESET);

	if (soc_priv->qos_val)
		cam_io_w_mb(soc_priv->qos_val, sys_base + ICP_V2_SYS_ACCESS);

	/* Program domain ID reg mask values if reg base is available */
	if (dom_mask_base_idx >= 0) {
		dom_mask_base = icp_v2_info->soc_info.reg_map[dom_mask_base_idx].mem_base;

		CAM_DBG(CAM_ICP, "domain_cfg0, offset: 0x%x: 0x%x, domain_cfg1, offset: 0x%x: 0x%x",
			ICP_V2_DOM_0_CFG_OFFSET, ICP_V2_DOMAIN_MASK_CFG_0,
			ICP_V2_DOM_1_CFG_OFFSET, ICP_V2_DOMAIN_MASK_CFG_1);

		cam_io_w_mb(ICP_V2_DOMAIN_MASK_CFG_0,
			dom_mask_base + ICP_V2_DOM_0_CFG_OFFSET);
		cam_io_w_mb(ICP_V2_DOMAIN_MASK_CFG_1,
			dom_mask_base + ICP_V2_DOM_1_CFG_OFFSET);
	}

	cam_io_w_mb(ICP_V2_EN_CPU,
		sys_base + ICP_V2_SYS_CONTROL);

	return 0;
}

#ifndef CONFIG_CAM_PRESIL
/* Used for non secure FW load */
static int32_t __cam_non_sec_load_fw(void *device_priv)
{
	int32_t rc = 0;
	uint32_t fw_size;
	char firmware_name[ICP_FW_NAME_MAX_SIZE] = {0};
	const char               *fw_name;
	const uint8_t            *fw_start = NULL;
	struct cam_hw_info       *icp_v2_dev = device_priv;
	struct cam_hw_soc_info   *soc_info = NULL;
	struct cam_icp_v2_core_info *core_info = NULL;
	struct platform_device   *pdev = NULL;

	if (!device_priv) {
		CAM_ERR(CAM_ICP, "Invalid cam_dev_info");
		return -EINVAL;
	}

	soc_info = &icp_v2_dev->soc_info;
	core_info = (struct cam_icp_v2_core_info *)icp_v2_dev->core_info;
	pdev = soc_info->pdev;

	/**
	 * Use paddr to map 0xE0400000 and 0xE0420000 as these
	 * addresses are routed internally by the core. These segments
	 * are used by the firmware to make use of the rom packing feature.
	 */

	rc = of_property_read_string(pdev->dev.of_node, "fw_name",
		&fw_name);
	if (rc) {
		CAM_ERR(CAM_ICP, "FW image name not found");
		return rc;
	}

	/* Account for ".elf" size [4 characters] */
	if (strlen(fw_name) >= (ICP_FW_NAME_MAX_SIZE - 4)) {
		CAM_ERR(CAM_ICP, "Invalid fw name %s", fw_name);
		return -EINVAL;
	}

	scnprintf(firmware_name, ARRAY_SIZE(firmware_name),
		"%s.elf", fw_name);

	rc = firmware_request_nowarn(&core_info->fw_params.fw_elf,
		firmware_name, &pdev->dev);
	if (rc) {
		CAM_ERR(CAM_ICP, "Failed to locate %s fw: %d",
			firmware_name, rc);
		return rc;
	}

	if (!core_info->fw_params.fw_elf) {
		CAM_ERR(CAM_ICP, "Invalid elf size");
		rc = -EINVAL;
		goto fw_download_failed;
	}

	fw_start = core_info->fw_params.fw_elf->data;

	rc = cam_icp_validate_fw(fw_start, EM_XTENSA);
	if (rc) {
		CAM_ERR(CAM_ICP, "fw elf validation failed");
		goto fw_download_failed;
	}

	rc = cam_icp_get_fw_size(fw_start, &fw_size);
	if (rc) {
		CAM_ERR(CAM_ICP, "unable to get fw size");
		goto fw_download_failed;
	}

	if (core_info->fw_params.fw_buf_len < fw_size) {
		CAM_ERR(CAM_ICP, "mismatch in fw size: %u %llu",
			fw_size, core_info->fw_params.fw_buf_len);
		rc = -EINVAL;
		goto fw_download_failed;
	}

	rc = cam_icp_program_fw(fw_start,
		core_info->fw_params.fw_kva_addr);
	if (rc) {
		CAM_ERR(CAM_ICP, "fw program is failed");
		goto fw_download_failed;
	}

fw_download_failed:
	release_firmware(core_info->fw_params.fw_elf);
	return rc;
}
#else /* #ifndef CONFIG_CAM_PRESIL */
static int __cam_non_sec_load_fw(struct cam_hw_info *icp_v2_info)
{
	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "Invalid ICP dev info");
		return -EINVAL;
	}

	cam_presil_send_event(CAM_PRESIL_EVENT_HFI_REG_ON_FIRST_REG_START_FW_DOWNLOAD, 0xFF);

	return 0;
}
#endif /* #ifndef CONFIG_CAM_PRESIL */

/* Used for non secure FW load */
static int cam_icp_v2_non_sec_boot(
	struct cam_hw_info *icp_v2_info,
	struct cam_icp_boot_args *args,
	uint32_t arg_size)
{
	int rc;
	struct cam_icp_soc_info *soc_priv;

	if (!icp_v2_info || !args) {
		CAM_ERR(CAM_ICP,
			"invalid args: icp_v2_dev=%pK args=%pK",
			icp_v2_info, args);
		return -EINVAL;
	}

	if (arg_size != sizeof(struct cam_icp_boot_args)) {
		CAM_ERR(CAM_ICP, "invalid boot args size");
		return -EINVAL;
	}

	soc_priv = (struct cam_icp_soc_info *)icp_v2_info->soc_info.soc_private;
	if (icp_v2_info->soc_info.num_mem_block > ICP_V2_BASE_MAX) {
		CAM_ERR(CAM_ICP, "check reg config in DT v 0x%x n %d",
			soc_priv->hw_version, icp_v2_info->soc_info.num_mem_block);
		return -EINVAL;
	}

	prepare_boot(icp_v2_info, args);

	rc = __cam_non_sec_load_fw(icp_v2_info);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"firmware download failed rc=%d", rc);
		goto err;
	}

	rc = __cam_icp_v2_power_resume(icp_v2_info);
	if (rc) {
		CAM_ERR(CAM_ICP, "ICP boot failed rc=%d", rc);
		goto err;
	}

	return 0;

err:
	prepare_shutdown(icp_v2_info);
	return rc;
}

#if IS_REACHABLE(CONFIG_QCOM_MDT_LOADER)
static int __load_firmware(struct platform_device *pdev)
{
	const char *fw_name;
	const struct firmware *firmware = NULL;
	char firmware_name[ICP_FW_NAME_MAX_SIZE] = {0};
	void *vaddr = NULL;
	struct device_node *node;
	struct resource res;
	phys_addr_t res_start;
	size_t res_size;
	ssize_t fw_size;
	int rc;

	if (!pdev) {
		CAM_ERR(CAM_ICP, "invalid args");
		return -EINVAL;
	}

	rc = of_property_read_string(pdev->dev.of_node, "fw_name",
		&fw_name);
	if (rc) {
		CAM_ERR(CAM_ICP, "FW image name not found");
		return rc;
	}

	/* Account for ".mdt" size [4 characters] */
	if (strlen(fw_name) >= (ICP_FW_NAME_MAX_SIZE - 4)) {
		CAM_ERR(CAM_ICP, "Invalid fw name %s", fw_name);
		return -EINVAL;
	}

	scnprintf(firmware_name, ARRAY_SIZE(firmware_name),
		"%s.mbn", fw_name);

	node = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!node) {
		CAM_ERR(CAM_ICP, "firmware memory region not found");
		return -ENODEV;
	}

	rc = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (rc) {
		CAM_ERR(CAM_ICP, "missing firmware resource address rc=%d", rc);
		return rc;
	}

	res_start = res.start;
	res_size = (size_t)resource_size(&res);

	rc = firmware_request_nowarn(&firmware, firmware_name, &pdev->dev);
	if (rc) {
		CAM_ERR(CAM_ICP,
			"error requesting %s firmware rc=%d",
			firmware_name, rc);
		return rc;
	}

	/* Make sure carveout and binary sizes are compatible */
	fw_size = qcom_mdt_get_size(firmware);
	if (fw_size < 0 || res_size < (size_t)fw_size) {
		CAM_ERR(CAM_ICP,
			"carveout[sz=%zu] not big enough for firmware[sz=%zd]",
			res_size, fw_size);
		rc = -EINVAL;
		goto out;
	}

	vaddr = ioremap_wc(res_start, res_size);
	if (!vaddr) {
		CAM_ERR(CAM_ICP, "unable to map firmware carveout");
		rc = -ENOMEM;
		goto out;
	}

	rc = qcom_mdt_load(&pdev->dev, firmware, firmware_name, CAM_FW_PAS_ID,
			vaddr, res_start, res_size, NULL);
	if (rc) {
		CAM_ERR(CAM_ICP, "failed to load firmware rc=%d", rc);
		goto out;
	}

out:
	if (vaddr)
		iounmap(vaddr);

	release_firmware(firmware);
	return rc;
}
#endif

static int cam_icp_v2_boot(struct cam_hw_info *icp_v2_info,
	struct cam_icp_boot_args *args, uint32_t arg_size)
{
	int rc;
	struct cam_icp_v2_core_info *core_info = NULL;

	if (!IS_REACHABLE(CONFIG_QCOM_MDT_LOADER))
		return -EOPNOTSUPP;

	if (!icp_v2_info || !args) {
		CAM_ERR(CAM_ICP,
			"invalid args: icp_v2_info=%pK args=%pK",
			icp_v2_info, args);
		return -EINVAL;
	}

	if (arg_size != sizeof(struct cam_icp_boot_args)) {
		CAM_ERR(CAM_ICP, "invalid boot args size");
		return -EINVAL;
	}

	core_info = (struct cam_icp_v2_core_info *)icp_v2_info->core_info;
	prepare_boot(icp_v2_info, args);

#if IS_REACHABLE(CONFIG_QCOM_MDT_LOADER)
	rc = __load_firmware(icp_v2_info->soc_info.pdev);
	if (rc) {
		CAM_ERR(CAM_ICP, "firmware loading failed rc=%d", rc);
		goto err;
	}
#endif

	rc = qcom_scm_pas_auth_and_reset(CAM_FW_PAS_ID);
	if (rc) {
		CAM_ERR(CAM_ICP, "auth and reset failed rc=%d", rc);
		goto err;
	}

	core_info->use_sec_pil = true;
	return 0;
err:
	prepare_shutdown(icp_v2_info);
	return rc;
}

static int cam_icp_v2_shutdown(struct cam_hw_info *icp_v2_info)
{
	int rc = 0;
	struct cam_icp_v2_core_info *core_info =
		(struct cam_icp_v2_core_info *)icp_v2_info->core_info;

	prepare_shutdown(icp_v2_info);

	if (core_info->use_sec_pil)
		rc = qcom_scm_pas_shutdown(CAM_FW_PAS_ID);
	else {
		int32_t sys_base_idx = core_info->reg_base_idx[ICP_V2_SYS_BASE];
		void __iomem *base;

		if (sys_base_idx < 0) {
			CAM_ERR(CAM_ICP, "No reg base idx found for ICP_SYS: %d",
				sys_base_idx);
			return -EINVAL;
		}

		base = icp_v2_info->soc_info.reg_map[sys_base_idx].mem_base;
		cam_io_w_mb(0x0, base + ICP_V2_SYS_CONTROL);
	}

	core_info->use_sec_pil = false;
	return rc;
}

static void __cam_icp_v2_core_reg_dump(
	struct cam_hw_info *icp_v2_info, uint32_t dump_type)
{
	int i;
	int32_t csr_base_idx;
	size_t len = 0;
	char log_info[512];
	struct cam_icp_v2_core_info *core_info = icp_v2_info->core_info;
	void __iomem *irq_base, *csr_base, *csr_gp_base;

	csr_base_idx = core_info->reg_base_idx[ICP_V2_CSR_BASE];
	csr_base =  icp_v2_info->soc_info.reg_map[csr_base_idx].mem_base;
	csr_gp_base = csr_base + ICP_V2_GEN_PURPOSE_REG_OFFSET;
	irq_base = icp_v2_info->soc_info.reg_map[core_info->irq_regbase_idx].mem_base;

	if (dump_type & CAM_ICP_DUMP_STATUS_REGISTERS)
		CAM_INFO(CAM_ICP, "ICP PFault status:0x%x",
			cam_io_r_mb(irq_base + core_info->hw_info->pfault_info));

	if (dump_type & CAM_ICP_DUMP_CSR_REGISTERS) {
		for (i = 0; i < ICP_V2_CSR_GP_REG_COUNT;) {
			CAM_INFO_BUF(CAM_ICP, log_info, 512, &len,
				"GP_%d: 0x%x GP_%d: 0x%x GP_%d: 0x%x GP_%d: 0x%x",
				i, cam_io_r_mb(csr_gp_base + (i << 2)),
				(i + 1), cam_io_r_mb(csr_gp_base + ((i + 1) << 2)),
				(i + 2), cam_io_r_mb(csr_gp_base + ((i + 2) << 2)),
				(i + 3), cam_io_r_mb(csr_gp_base + ((i + 3) << 2)));
			i += 4;
		}

		CAM_INFO(CAM_ICP, "ICP CSR GP registers - %s", log_info);
	}
}

/* API controls collapse/resume of ICP */
static int cam_icp_v2_core_control(struct cam_hw_info *icp_v2_info,
	uint32_t state)
{
	int rc = 0;
	struct cam_icp_v2_core_info *core_info =
		(struct cam_icp_v2_core_info *)icp_v2_info->core_info;

	if (core_info->use_sec_pil) {
		rc = qcom_scm_set_remote_state(state, CAM_FW_PAS_ID);
		if (rc) {
			CAM_ERR(CAM_ICP,
				"remote state set to %s failed rc=%d",
				(state == TZ_STATE_RESUME ? "resume" : "suspend"), rc);
			__cam_icp_v2_core_reg_dump(icp_v2_info, CAM_ICP_DUMP_STATUS_REGISTERS);
		}
	} else {
		if (state == TZ_STATE_RESUME) {
			rc = __cam_icp_v2_power_resume(icp_v2_info);
			if (rc)
				CAM_ERR(CAM_ICP, "ICP resume failed rc=%d", rc);
		} else {
			rc = __cam_icp_v2_power_collapse(icp_v2_info);
			if (rc)
				CAM_ERR(CAM_ICP, "ICP collapse failed rc=%d", rc);
		}
	}

	return rc;
}

static inline int cam_icp_v2_download_fw(struct cam_hw_info *icp_v2_info,
	struct cam_icp_boot_args *args, uint32_t arg_size)
{
	int rc;

	CAM_INFO(CAM_ICP, "Loading Secure PIL : %s", CAM_BOOL_TO_YESNO(args->use_sec_pil));

	if (args->use_sec_pil)
		rc = cam_icp_v2_boot(
			icp_v2_info, args, arg_size);
	else
		rc = cam_icp_v2_non_sec_boot(
			icp_v2_info, args, arg_size);

	return rc;
}

static int __cam_icp_v2_update_clk_rate(struct cam_hw_info *icp_v2_info,
	int32_t *clk_lvl)
{
	int32_t clk_level = 0, rc;
	struct cam_ahb_vote       ahb_vote;
	struct cam_icp_v2_core_info *core_info = NULL;
	struct cam_hw_soc_info   *soc_info = NULL;

	if (!clk_lvl) {
		CAM_ERR(CAM_ICP, "Invalid args");
		return -EINVAL;
	}

	soc_info = &icp_v2_info->soc_info;
	core_info = icp_v2_info->core_info;
	if (!core_info || !soc_info) {
		CAM_ERR(CAM_ICP, "Invalid args");
		return -EINVAL;
	}

	clk_level = *((int32_t *)clk_lvl);
	CAM_DBG(CAM_ICP,
		"Update ICP clock to level [%d]", clk_level);
	rc = cam_icp_soc_update_clk_rate(soc_info, clk_level);
	if (rc)
		CAM_WARN(CAM_ICP,
			"Failed to update clk to level: %d rc: %d",
			clk_level, rc);

	ahb_vote.type = CAM_VOTE_ABSOLUTE;
	ahb_vote.vote.level = clk_level;
	rc = cam_cpas_update_ahb_vote(
		core_info->cpas_handle, &ahb_vote);
	if (rc)
		CAM_WARN(CAM_ICP,
			"Failed to update ahb vote rc: %d", rc);

	return rc;
}

static int __cam_icp_v2_fw_mini_dump(struct cam_icp_v2_core_info *core_info,
	struct cam_icp_hw_dump_args *args)
{
	if (!core_info) {
		CAM_ERR(CAM_ICP, "Invalid param %pK", core_info);
		return -EINVAL;
	}

	return cam_icp_proc_mini_dump(args, core_info->fw_params.fw_kva_addr,
		core_info->fw_params.fw_buf_len);
}

int cam_icp_v2_process_cmd(void *priv, uint32_t cmd_type,
	void *args, uint32_t arg_size)
{
	struct cam_hw_info *icp_v2_info = priv;
	int rc = -EINVAL;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "ICP device info cannot be NULL");
		return -EINVAL;
	}

	switch (cmd_type) {
	case CAM_ICP_CMD_PROC_SHUTDOWN:
		rc = cam_icp_v2_shutdown(icp_v2_info);
		break;
	case CAM_ICP_CMD_PROC_BOOT:
		rc = cam_icp_v2_download_fw(icp_v2_info, args, arg_size);
		break;
	case CAM_ICP_CMD_POWER_COLLAPSE:
		rc = cam_icp_v2_core_control(icp_v2_info, TZ_STATE_SUSPEND);
		break;
	case CAM_ICP_CMD_POWER_RESUME:
		rc = cam_icp_v2_core_control(icp_v2_info, TZ_STATE_RESUME);
		break;
	case CAM_ICP_CMD_VOTE_CPAS:
		rc = cam_icp_v2_cpas_vote(icp_v2_info->core_info, args);
		break;
	case CAM_ICP_CMD_CPAS_START:
		rc = __icp_v2_cpas_start(icp_v2_info->core_info, args);
		break;
	case CAM_ICP_CMD_CPAS_STOP:
		rc = cam_icp_v2_cpas_stop(icp_v2_info->core_info);
		break;
	case CAM_ICP_CMD_UBWC_CFG:
		rc = cam_icp_v2_ubwc_configure(&icp_v2_info->soc_info);
		break;
	case CAM_ICP_SEND_INIT:
		hfi_send_system_cmd(HFI_CMD_SYS_INIT, 0, 0);
		rc = 0;
		break;
	case CAM_ICP_CMD_PC_PREP:
		hfi_send_system_cmd(HFI_CMD_SYS_PC_PREP, 0, 0);
		rc = 0;
		break;
	case CAM_ICP_CMD_CLK_UPDATE: {
		rc = __cam_icp_v2_update_clk_rate(icp_v2_info, args);
		break;
	}
	case CAM_ICP_CMD_HW_DUMP:
		/* Not supported for ICP_V2 */
		rc = 0;
		break;
	case CAM_ICP_CMD_HW_MINI_DUMP: {
		rc = __cam_icp_v2_fw_mini_dump(icp_v2_info->core_info, args);
		break;
	}
	case CAM_ICP_CMD_HW_REG_DUMP: {
		uint32_t dump_type;

		if (!args) {
			CAM_ERR(CAM_ICP, "Invalid args");
			break;
		}

		dump_type = *(uint32_t *) args;
		__cam_icp_v2_core_reg_dump(icp_v2_info, dump_type);
		rc = 0;
		break;
	}
	default:
		CAM_ERR(CAM_ICP, "invalid command type=%u", cmd_type);
		break;
	}

	return rc;
}

irqreturn_t cam_icp_v2_handle_irq(int irq_num, void *data)
{
	struct cam_hw_info *icp_v2_info = data;
	struct cam_icp_v2_core_info *core_info = NULL;
	bool recover = false;
	uint32_t status = 0;
	int32_t wd0_base_idx;
	void __iomem *irq_base;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "invalid ICP device info");
		return IRQ_NONE;
	}

	core_info = icp_v2_info->core_info;
	irq_base = icp_v2_info->soc_info.reg_map[core_info->irq_regbase_idx].mem_base;

	status = cam_io_r_mb(irq_base + core_info->hw_info->ob_irq_status);

	cam_io_w_mb(status, irq_base + core_info->hw_info->ob_irq_clear);
	cam_io_w_mb(ICP_V2_IRQ_CLEAR_CMD, irq_base + core_info->hw_info->ob_irq_cmd);

	if (core_info->is_irq_test) {
		CAM_INFO(CAM_ICP, "ICP_V2 IRQ verified (status=0x%x)", status);
		core_info->is_irq_test = false;
		complete(&icp_v2_info->hw_complete);
		return IRQ_HANDLED;
	}

	/* WD clear sequence - SW listens only to WD0 */
	if (status & ICP_V2_WDT_BITE_WS0) {
		wd0_base_idx = core_info->reg_base_idx[ICP_V2_WD0_BASE];

		cam_io_w_mb(0x0,
			icp_v2_info->soc_info.reg_map[wd0_base_idx].mem_base +
			ICP_V2_WD_CTRL);
		cam_io_w_mb(0x1,
			icp_v2_info->soc_info.reg_map[wd0_base_idx].mem_base +
			ICP_V2_WD_INTCLR);
		CAM_ERR_RATE_LIMIT(CAM_ICP, "Fatal: Watchdog Bite from ICP");
		recover = true;
	}

	spin_lock(&icp_v2_info->hw_lock);
	if (core_info->irq_cb.cb)
		core_info->irq_cb.cb(core_info->irq_cb.data,
			recover);
	spin_unlock(&icp_v2_info->hw_lock);

	return IRQ_HANDLED;
}

void cam_icp_v2_irq_raise(void *priv)
{
	struct cam_hw_info *icp_v2_info = priv;
	struct cam_icp_v2_core_info *core_info = NULL;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "Invalid ICP device info");
		return;
	}

	core_info = icp_v2_info->core_info;
	cam_io_w_mb(ICP_V2_HOST2ICPINT,
		icp_v2_info->soc_info.reg_map[core_info->irq_regbase_idx].mem_base +
		core_info->hw_info->host2icpint);
}

void cam_icp_v2_irq_enable(void *priv)
{
	struct cam_hw_info *icp_v2_info = priv;
	struct cam_icp_v2_core_info *core_info = NULL;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "Invalid ICP device info");
		return;
	}

	core_info = icp_v2_info->core_info;
	cam_io_w_mb(ICP_V2_WDT_BITE_WS0 | ICP_V2_ICP2HOSTINT,
		icp_v2_info->soc_info.reg_map[core_info->irq_regbase_idx].mem_base +
		core_info->hw_info->ob_irq_mask);
}

int cam_icp_v2_test_irq_line(void *priv)
{
	struct cam_hw_info *icp_v2_info = priv;
	struct cam_icp_v2_core_info *core_info = NULL;
	void __iomem *irq_membase;
	unsigned long rem_jiffies;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "invalid ICP device info");
		return -EINVAL;
	}

	core_info = icp_v2_info->core_info;
	irq_membase = icp_v2_info->soc_info.reg_map[core_info->irq_regbase_idx].mem_base;

	reinit_completion(&icp_v2_info->hw_complete);
	core_info->is_irq_test = true;

	cam_icp_v2_hw_init(priv, NULL, 0);

	cam_io_w_mb(ICP_V2_WDT_BARK_WS0, irq_membase + core_info->hw_info->ob_irq_mask);
	cam_io_w_mb(ICP_V2_WDT_BARK_WS0, irq_membase + core_info->hw_info->ob_irq_set);
	cam_io_w_mb(ICP_V2_IRQ_SET_CMD, irq_membase + core_info->hw_info->ob_irq_cmd);

	rem_jiffies = cam_common_wait_for_completion_timeout(&icp_v2_info->hw_complete,
		msecs_to_jiffies(ICP_V2_IRQ_TEST_TIMEOUT));
	if (!rem_jiffies)
		CAM_ERR(CAM_ICP, "ICP IRQ verification timed out");

	cam_io_w_mb(0, irq_membase + core_info->hw_info->ob_irq_mask);
	cam_icp_v2_hw_deinit(priv, NULL, 0);

	core_info->is_irq_test = false;

	return 0;
}

void __iomem *cam_icp_v2_iface_addr(void *priv)
{
	int32_t csr_base_idx;
	struct cam_hw_info *icp_v2_info = priv;
	struct cam_icp_v2_core_info *core_info = NULL;
	void __iomem *base;

	if (!icp_v2_info) {
		CAM_ERR(CAM_ICP, "invalid icp device info");
		return ERR_PTR(-EINVAL);
	}

	core_info = icp_v2_info->core_info;
	csr_base_idx = core_info->reg_base_idx[ICP_V2_CSR_BASE];
	base = icp_v2_info->soc_info.reg_map[csr_base_idx].mem_base;

	return base + ICP_V2_GEN_PURPOSE_REG_OFFSET;
}

void cam_icp_v2_populate_hfi_ops(const struct hfi_ops **hfi_proc_ops)
{
	if (!hfi_proc_ops) {
		CAM_ERR(CAM_ICP, "Invalid hfi ops argument");
		return;
	}

	*hfi_proc_ops = &hfi_icp_v2_ops;
}

static int cam_icp_v2_setup_register_base_indexes(
	struct cam_hw_soc_info *soc_info, uint32_t hw_version,
	int32_t regbase_index[], int32_t num_reg_map)
{
	int rc;
	uint32_t index;

	if (num_reg_map > ICP_V2_BASE_MAX) {
		CAM_ERR(CAM_ICP, "Number of reg maps: %d exceeds max: %d",
			num_reg_map, ICP_V2_BASE_MAX);
		return -EINVAL;
	}

	if (soc_info->num_mem_block > CAM_SOC_MAX_BLOCK) {
		CAM_ERR(CAM_ICP, "Invalid number of mem blocks: %d",
			soc_info->num_mem_block);
		return -EINVAL;
	}

	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "icp_csr", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[ICP_V2_CSR_BASE] = index;
	} else {
		CAM_ERR(CAM_ICP,
			"Failed to get index for icp_csr, rc: %d index: %u num_reg_map: %u",
			rc, index, num_reg_map);
		return -EINVAL;
	}

	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "icp_wd0", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[ICP_V2_WD0_BASE] = index;
	} else {
		CAM_ERR(CAM_ICP,
			"Failed to get index for icp_wd0, rc: %d index: %u num_reg_map: %u",
			rc, index, num_reg_map);
		return -EINVAL;
	}

	if (hw_version == CAM_ICP_V2_VERSION) {
		rc = cam_common_util_get_string_index(soc_info->mem_block_name,
			soc_info->num_mem_block, "icp_cirq", &index);
		if ((rc == 0) && (index < num_reg_map)) {
			regbase_index[ICP_V2_CIRQ_BASE] = index;
		} else {
			CAM_ERR(CAM_ICP,
				"Failed to get index for icp_cirq, rc: %d index: %u num_reg_map: %u",
				rc, index, num_reg_map);
			return -EINVAL;
		}
	} else {
		/* Optional for other versions */
		regbase_index[ICP_V2_CIRQ_BASE] = -1;
	}

	/* optional - ICP SYS map */
	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "icp_sys", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[ICP_V2_SYS_BASE] = index;
	}  else {
		CAM_DBG(CAM_ICP,
			"Failed to get index for icp_sys, rc: %d index: %u num_reg_map: %u",
			rc, index, num_reg_map);
		regbase_index[ICP_V2_SYS_BASE] = -1;
	}

	/* optional - for non secure FW loading */
	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "icp_dom_mask", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[ICP_V2_DOM_MASK_BASE] = index;
	} else {
		CAM_DBG(CAM_ICP,
			"Failed to get index for icp_dom_mask, rc: %d index: %u num_reg_map: %u",
			rc, index, num_reg_map);
		regbase_index[ICP_V2_DOM_MASK_BASE] = -1;
	}

	return 0;
}

int cam_icp_v2_core_init(
	struct cam_hw_soc_info *soc_info,
	struct cam_icp_v2_core_info *core_info)
{
	int rc = 0;
	struct cam_icp_soc_info *soc_priv;

	soc_priv = (struct cam_icp_soc_info *)soc_info->soc_private;

	rc = cam_icp_v2_setup_register_base_indexes(soc_info,
		soc_priv->hw_version, core_info->reg_base_idx, ICP_V2_BASE_MAX);
	if (rc)
		return rc;

	/* Associate OB/HOST2ICP IRQ reg base */
	switch (soc_priv->hw_version) {
	case CAM_ICP_V2_1_VERSION:
		core_info->irq_regbase_idx =
			core_info->reg_base_idx[ICP_V2_CSR_BASE];
		break;
	case CAM_ICP_V2_VERSION:
		core_info->irq_regbase_idx =
			core_info->reg_base_idx[ICP_V2_CIRQ_BASE];
		break;
	default:
		CAM_ERR(CAM_ICP, "Unsupported ICP HW version: %u",
			soc_priv->hw_version);
		rc = -EINVAL;
	}

	return rc;
}
