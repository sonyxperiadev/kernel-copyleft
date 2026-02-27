// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "cam_icp_proc_common.h"
#include "cam_compat.h"
#include "cam_icp_hw_intf.h"
#include "hfi_intf.h"

int32_t cam_icp_validate_fw(const uint8_t *elf,
	uint32_t machine_id)
{
	struct elf32_hdr *elf_hdr;

	if (!elf) {
		CAM_ERR(CAM_ICP, "Invalid params");
		return -EINVAL;
	}

	elf_hdr = (struct elf32_hdr *)elf;

	if (memcmp(elf_hdr->e_ident, ELFMAG, SELFMAG)) {
		CAM_ERR(CAM_ICP, "ICP elf identifier is failed");
		return -EINVAL;
	}

	/* check architecture */
	if (elf_hdr->e_machine != machine_id) {
		CAM_ERR(CAM_ICP, "unsupported arch: 0x%x", elf_hdr->e_machine);
		return -EINVAL;
	}

	/* check elf bit format */
	if (elf_hdr->e_ident[EI_CLASS] != ELFCLASS32) {
		CAM_ERR(CAM_ICP, "elf doesn't support 32 bit format");
		return -EINVAL;
	}

	return 0;
}

int32_t cam_icp_get_fw_size(
	const uint8_t *elf, uint32_t *fw_size)
{
	int32_t rc = 0;
	int32_t i = 0;
	uint32_t num_prg_hdrs;
	unsigned char *icp_prg_hdr_tbl;
	uint32_t seg_mem_size = 0;
	struct elf32_hdr *elf_hdr;
	struct elf32_phdr *prg_hdr;

	if (!elf || !fw_size) {
		CAM_ERR(CAM_ICP, "invalid args");
		return -EINVAL;
	}

	*fw_size = 0;

	elf_hdr = (struct elf32_hdr *)elf;
	num_prg_hdrs = elf_hdr->e_phnum;
	icp_prg_hdr_tbl = (unsigned char *)elf + elf_hdr->e_phoff;
	prg_hdr = (struct elf32_phdr *)&icp_prg_hdr_tbl[0];

	if (!prg_hdr) {
		CAM_ERR(CAM_ICP, "failed to get elf program header attr");
		return -EINVAL;
	}

	CAM_DBG(CAM_ICP, "num_prg_hdrs = %d", num_prg_hdrs);
	for (i = 0; i < num_prg_hdrs; i++, prg_hdr++) {
		if (prg_hdr->p_flags == 0)
			continue;

		seg_mem_size = (prg_hdr->p_memsz + prg_hdr->p_align - 1) &
			~(prg_hdr->p_align - 1);
		seg_mem_size += prg_hdr->p_paddr;
		CAM_DBG(CAM_ICP, "memsz:%x align:%x addr:%x seg_mem_size:%x",
			(int)prg_hdr->p_memsz, (int)prg_hdr->p_align,
			(int)prg_hdr->p_paddr, (int)seg_mem_size);
		if (*fw_size < seg_mem_size)
			*fw_size = seg_mem_size;

	}

	if (*fw_size == 0) {
		CAM_ERR(CAM_ICP, "invalid elf fw file");
		return -EINVAL;
	}

	return rc;
}

int32_t cam_icp_program_fw(const uint8_t *elf,
	uintptr_t fw_kva_addr)
{
	int32_t rc = 0;
	uint32_t num_prg_hdrs;
	unsigned char *icp_prg_hdr_tbl;
	int32_t i = 0;
	u8 *dest;
	u8 *src;
	struct elf32_hdr *elf_hdr;
	struct elf32_phdr *prg_hdr;

	elf_hdr = (struct elf32_hdr *)elf;
	num_prg_hdrs = elf_hdr->e_phnum;
	icp_prg_hdr_tbl = (unsigned char *)elf + elf_hdr->e_phoff;
	prg_hdr = (struct elf32_phdr *)&icp_prg_hdr_tbl[0];

	if (!prg_hdr) {
		CAM_ERR(CAM_ICP, "failed to get elf program header attr");
		return -EINVAL;
	}

	for (i = 0; i < num_prg_hdrs; i++, prg_hdr++) {
		if (prg_hdr->p_flags == 0)
			continue;

		CAM_DBG(CAM_ICP, "Loading FW header size: %u paddr: %pK",
			prg_hdr->p_filesz, prg_hdr->p_paddr);
		if (prg_hdr->p_filesz != 0) {
			src = (u8 *)((u8 *)elf + prg_hdr->p_offset);
			dest = (u8 *)(((u8 *)fw_kva_addr) +
				prg_hdr->p_paddr);

			memcpy_toio(dest, src, prg_hdr->p_filesz);
		}
	}

	return rc;
}

int cam_icp_proc_cpas_vote(uint32_t cpas_handle,
	struct cam_icp_cpas_vote *vote)
{
	int rc;

	if (!vote)
		return -EINVAL;

	if (vote->ahb_vote_valid) {
		rc = cam_cpas_update_ahb_vote(cpas_handle, &vote->ahb_vote);
		if (rc) {
			CAM_ERR(CAM_ICP, "AHB vote update failed rc=%d", rc);
			return rc;
		}
	}

	if (vote->axi_vote_valid) {
		rc = cam_cpas_update_axi_vote(cpas_handle, &vote->axi_vote);
		if (rc) {
			CAM_ERR(CAM_ICP, "AXI vote update failed rc=%d", rc);
			return rc;
		}
	}

	return 0;
}

int cam_icp_proc_mini_dump(struct cam_icp_hw_dump_args *args,
	uintptr_t fw_kva_addr, uint64_t fw_buf_len)
{
	u8                          *dest;
	u8                          *src;
	struct cam_icp_hw_dump_args *dump_args = args;

	if (!dump_args) {
		CAM_ERR(CAM_ICP, "Invalid param %pK", dump_args);
		return -EINVAL;
	}

	if (!fw_kva_addr || !dump_args->cpu_addr) {
		CAM_ERR(CAM_ICP, "invalid params %pK, 0x%zx",
			fw_kva_addr, dump_args->cpu_addr);
		return -EINVAL;
	}

	if (dump_args->buf_len < fw_buf_len) {
		CAM_WARN(CAM_ICP, "Insufficient Len %lu fw_len %llu",
			dump_args->buf_len, fw_buf_len);
		return -ENOSPC;
	}

	dest = (u8 *)dump_args->cpu_addr;
	src = (u8 *)fw_kva_addr;
	memcpy_fromio(dest, src, fw_buf_len);
	dump_args->offset = fw_buf_len;

	return 0;
}

static int cam_icp_proc_validate_ubwc_cfg(struct cam_icp_ubwc_cfg *ubwc_cfg,
	uint32_t ubwc_cfg_dev_mask)
{
	uint32_t found_ubwc_cfg_mask = ubwc_cfg->found_ubwc_cfg_mask;

	if ((ubwc_cfg_dev_mask & BIT(CAM_ICP_DEV_IPE)) &&
		!(found_ubwc_cfg_mask & BIT(CAM_ICP_DEV_IPE))) {
		CAM_ERR(CAM_ICP, "IPE does not have UBWC cfg value");
		return -ENODATA;
	}

	if ((ubwc_cfg_dev_mask & BIT(CAM_ICP_DEV_BPS)) &&
		!(found_ubwc_cfg_mask & BIT(CAM_ICP_DEV_BPS))) {
		CAM_ERR(CAM_ICP, "BPS does not have UBWC cfg value");
		return -ENODATA;
	}

	if ((ubwc_cfg_dev_mask & BIT(CAM_ICP_DEV_OFE)) &&
		!(found_ubwc_cfg_mask & BIT(CAM_ICP_DEV_OFE))) {
		CAM_ERR(CAM_ICP, "OFE does not have UBWC cfg value");
		return -ENODATA;
	}

	return 0;
}

int cam_icp_proc_ubwc_configure(struct cam_icp_proc_ubwc_cfg_cmd *ubwc_cfg_cmd,
	bool force_disable_ubwc, int hfi_handle)
{
	struct cam_icp_ubwc_cfg *ubwc_cfg;
	int i = 0, ddr_type, rc;
	uint32_t ipe_ubwc_cfg[ICP_UBWC_CFG_MAX] = {0};
	uint32_t bps_ubwc_cfg[ICP_UBWC_CFG_MAX] = {0};
	uint32_t ofe_ubwc_cfg[ICP_UBWC_CFG_MAX] = {0};

	if (!ubwc_cfg_cmd) {
		CAM_ERR(CAM_ICP, "ubwc config command is NULL");
		return -EINVAL;
	}

	ubwc_cfg = ubwc_cfg_cmd->ubwc_cfg;
	rc = cam_icp_proc_validate_ubwc_cfg(ubwc_cfg, ubwc_cfg_cmd->ubwc_cfg_dev_mask);
	if (rc) {
		CAM_ERR(CAM_ICP, "UBWC config failed validation rc:%d", rc);
		return rc;
	}

	ddr_type = cam_get_ddr_type();

	if (ddr_type == DDR_TYPE_LPDDR5 || ddr_type == DDR_TYPE_LPDDR5X)
		i = 1;

	ipe_ubwc_cfg[0] = ubwc_cfg->ipe_fetch[i];
	ipe_ubwc_cfg[1] = ubwc_cfg->ipe_write[i];

	bps_ubwc_cfg[0] = ubwc_cfg->bps_fetch[i];
	bps_ubwc_cfg[1] = ubwc_cfg->bps_write[i];

	ofe_ubwc_cfg[0] = ubwc_cfg->ofe_fetch[i];
	ofe_ubwc_cfg[1] = ubwc_cfg->ofe_write[i];

	if (force_disable_ubwc) {
		ipe_ubwc_cfg[1] &= ~CAM_ICP_UBWC_COMP_EN;
		bps_ubwc_cfg[1] &= ~CAM_ICP_UBWC_COMP_EN;
		ofe_ubwc_cfg[1] &= ~CAM_ICP_UBWC_COMP_EN;
		CAM_DBG(CAM_ICP,
			"Force disable UBWC compression, ipe_ubwc_cfg: 0x%x, bps_ubwc_cfg: 0x%x ofe_ubwc_cfg: 0x%x",
			ipe_ubwc_cfg[1], bps_ubwc_cfg[1], ofe_ubwc_cfg[1]);
	}

	rc = hfi_cmd_ubwc_config_ext(hfi_handle, ipe_ubwc_cfg, bps_ubwc_cfg, ofe_ubwc_cfg);
	if (rc) {
		CAM_ERR(CAM_ICP, "Failed to write UBWC configure rc=%d", rc);
		return rc;
	}

	return 0;
}
