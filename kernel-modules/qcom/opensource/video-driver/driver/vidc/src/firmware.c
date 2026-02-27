// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/types.h>
#include <linux/list.h>
#include <linux/of_address.h>
#include <linux/devcoredump.h>
#include <linux/firmware.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/soc/qcom/smem.h>

#include "msm_vidc_core.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_events.h"
#include "msm_vidc_platform.h"
#include "firmware.h"

#define MAX_FIRMWARE_NAME_SIZE	128

struct tzbsp_memprot {
	u32 cp_start;
	u32 cp_size;
	u32 cp_nonpixel_start;
	u32 cp_nonpixel_size;
};

enum tzbsp_video_state {
	TZBSP_VIDEO_STATE_SUSPEND = 0,
	TZBSP_VIDEO_STATE_RESUME = 1,
	TZBSP_VIDEO_STATE_RESTORE_THRESHOLD = 2,
};

static int protect_cp_mem(struct msm_vidc_core *core)
{
	struct tzbsp_memprot memprot;
	int rc = 0;
	struct context_bank_info *cb;

	memprot.cp_start = 0x0;
	memprot.cp_size = 0x0;
	memprot.cp_nonpixel_start = 0x0;
	memprot.cp_nonpixel_size = 0x0;

	venus_hfi_for_each_context_bank(core, cb) {
		if (cb->region == MSM_VIDC_NON_SECURE) {
			memprot.cp_size = cb->addr_range.start;

			d_vpr_h("%s: memprot.cp_size: %#x\n",
				__func__, memprot.cp_size);
		}

		if (cb->region == MSM_VIDC_SECURE_NONPIXEL) {
			memprot.cp_nonpixel_start = cb->addr_range.start;
			memprot.cp_nonpixel_size = cb->addr_range.size;

			d_vpr_h("%s: cp_nonpixel_start: %#x size: %#x\n",
				__func__, memprot.cp_nonpixel_start,
				memprot.cp_nonpixel_size);
		}
	}

	rc = qcom_scm_mem_protect_video_var(memprot.cp_start, memprot.cp_size,
			memprot.cp_nonpixel_start, memprot.cp_nonpixel_size);
	if (rc)
		d_vpr_e("Failed to protect memory(%d)\n", rc);

	trace_venus_hfi_var_done(memprot.cp_start, memprot.cp_size,
				 memprot.cp_nonpixel_start, memprot.cp_nonpixel_size);

	return rc;
}

static int __load_fw_to_memory(struct platform_device *pdev,
			       const char *fw_name)
{
	int rc = 0;
	const struct firmware *firmware = NULL;
	struct msm_vidc_core *core;
	char firmware_name[MAX_FIRMWARE_NAME_SIZE] = { 0 };
	struct device_node *node = NULL;
	struct resource res = { 0 };
	phys_addr_t phys = 0;
	size_t res_size = 0;
	ssize_t fw_size = 0;
	void *virt = NULL;
	int pas_id = 0;

	if (!fw_name || !(*fw_name) || !pdev) {
		d_vpr_e("%s: Invalid inputs\n", __func__);
		return -EINVAL;
	}
	if (strlen(fw_name) >= MAX_FIRMWARE_NAME_SIZE - 4) {
		d_vpr_e("%s: Invalid fw name\n", __func__);
		return -EINVAL;
	}

	core = dev_get_drvdata(&pdev->dev);
	if (!core) {
		d_vpr_e("%s: core not found in device %s",
			__func__, dev_name(&pdev->dev));
		return -EINVAL;
	}
	scnprintf(firmware_name, ARRAY_SIZE(firmware_name), "%s.mbn", fw_name);

	pas_id = core->platform->data.pas_id;

	node = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!node) {
		d_vpr_e("%s: failed to read \"memory-region\"\n",
			__func__);
		return -EINVAL;
	}

	rc = of_address_to_resource(node, 0, &res);
	if (rc) {
		d_vpr_e("%s: failed to read \"memory-region\", error %d\n",
			__func__, rc);
		goto exit;
	}
	phys = res.start;
	res_size = (size_t)resource_size(&res);

	rc = request_firmware(&firmware, firmware_name, &pdev->dev);
	if (rc) {
		d_vpr_e("%s: failed to request fw \"%s\", error %d\n",
			__func__, firmware_name, rc);
		goto exit;
	}

	fw_size = qcom_mdt_get_size(firmware);
	if (fw_size < 0 || res_size < (size_t)fw_size) {
		rc = -EINVAL;
		d_vpr_e("%s: out of bound fw image fw size: %ld, res_size: %lu",
			__func__, fw_size, res_size);
		goto exit;
	}

	virt = memremap(phys, res_size, MEMREMAP_WC);
	if (!virt) {
		d_vpr_e("%s: failed to remap fw memory phys %pa[p]\n",
			__func__, &phys);
		return -ENOMEM;
	}

	/* prevent system suspend during fw_load */
	pm_stay_awake(pdev->dev.parent);
	rc = qcom_mdt_load(&pdev->dev, firmware, firmware_name,
			   pas_id, virt, phys, res_size, NULL);
	pm_relax(pdev->dev.parent);
	if (rc) {
		d_vpr_e("%s: error %d loading fw \"%s\"\n",
			__func__, rc, firmware_name);
		goto exit;
	}
	rc = qcom_scm_pas_auth_and_reset(pas_id);
	if (rc) {
		d_vpr_e("%s: error %d authenticating fw \"%s\"\n",
			__func__, rc, firmware_name);
		goto exit;
	}

	memunmap(virt);
	release_firmware(firmware);
	d_vpr_h("%s: firmware \"%s\" loaded successfully\n",
		__func__, firmware_name);

	return pas_id;

exit:
	if (virt)
		memunmap(virt);
	if (firmware)
		release_firmware(firmware);

	return rc;
}

int fw_load(struct msm_vidc_core *core)
{
	int rc;

	if (!core->resource->fw_cookie) {
		core->resource->fw_cookie = __load_fw_to_memory(core->pdev,
								core->platform->data.fwname);
		if (core->resource->fw_cookie <= 0) {
			d_vpr_e("%s: firmware download failed %d\n",
				__func__, core->resource->fw_cookie);
			core->resource->fw_cookie = 0;
			return -ENOMEM;
		}
	}

	rc = protect_cp_mem(core);
	if (rc) {
		d_vpr_e("%s: protect memory failed\n", __func__);
		goto fail_protect_mem;
	}

	return rc;

fail_protect_mem:
	if (core->resource->fw_cookie)
		qcom_scm_pas_shutdown(core->resource->fw_cookie);
	core->resource->fw_cookie = 0;
	return rc;
}

int fw_unload(struct msm_vidc_core *core)
{
	int ret;

	if (!core->resource->fw_cookie)
		return -EINVAL;

	ret = qcom_scm_pas_shutdown(core->resource->fw_cookie);
	if (ret)
		d_vpr_e("Firmware unload failed rc=%d\n", ret);

	core->resource->fw_cookie = 0;

	return ret;
}

int fw_suspend(struct msm_vidc_core *core)
{
	return qcom_scm_set_remote_state(TZBSP_VIDEO_STATE_SUSPEND, 0);
}

int fw_resume(struct msm_vidc_core *core)
{
	return qcom_scm_set_remote_state(TZBSP_VIDEO_STATE_RESUME, 0);
}

void fw_coredump(struct msm_vidc_core *core)
{
	int rc = 0;
	struct platform_device *pdev;
	struct device_node *node = NULL;
	struct resource res = {0};
	phys_addr_t mem_phys = 0;
	size_t res_size = 0;
	void *mem_va = NULL;
	char *data = NULL, *dump = NULL;
	u64 total_size;

	pdev = core->pdev;

	node = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!node) {
		d_vpr_e("%s: DT error getting \"memory-region\" property\n",
			__func__);
		return;
	}

	rc = of_address_to_resource(node, 0, &res);
	if (rc) {
		d_vpr_e("%s: error %d while getting \"memory-region\" resource\n",
			__func__, rc);
		return;
	}

	mem_phys = res.start;
	res_size = (size_t)resource_size(&res);

	mem_va = memremap(mem_phys, res_size, MEMREMAP_WC);
	if (!mem_va) {
		d_vpr_e("%s: unable to remap firmware memory\n", __func__);
		return;
	}
	total_size = res_size + TOTAL_QSIZE + ALIGNED_SFR_SIZE;

	data = vmalloc(total_size);
	if (!data) {
		memunmap(mem_va);
		return;
	}
	dump = data;

	/* copy firmware dump */
	memcpy(data, mem_va, res_size);
	memunmap(mem_va);

	/* copy queues(cmd, msg, dbg) dump(along with headers) */
	data += res_size;
	memcpy(data, (char *)core->iface_q_table.align_virtual_addr, TOTAL_QSIZE);

	/* copy sfr dump */
	data += TOTAL_QSIZE;
	memcpy(data, (char *)core->sfr.align_virtual_addr, ALIGNED_SFR_SIZE);

	dev_coredumpv(&pdev->dev, dump, total_size, GFP_KERNEL);
}
