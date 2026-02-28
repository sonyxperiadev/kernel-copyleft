// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "adreno.h"
#include "adreno_gen8.h"
#include "adreno_gen8_gmu.h"
#include "adreno_gen8_0_0_snapshot.h"
#include "adreno_snapshot.h"
#include "gen8_reg.h"
#include "kgsl_device.h"

static size_t gen8_gmu_snapshot_dtcm(struct kgsl_device *device,
		u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_gmu_mem *mem_hdr =
		(struct kgsl_snapshot_gmu_mem *)buf;
	struct gen8_gmu_device *gmu = (struct gen8_gmu_device *)priv;
	u32 *data = (u32 *)(buf + sizeof(*mem_hdr));
	u32 i;

	if (remain < gmu->vma[GMU_DTCM].size + sizeof(*mem_hdr)) {
		SNAPSHOT_ERR_NOMEM(device, "GMU DTCM Memory");
		return 0;
	}

	mem_hdr->type = SNAPSHOT_GMU_MEM_BIN_BLOCK;
	mem_hdr->hostaddr = 0;
	mem_hdr->gmuaddr = gmu->vma[GMU_DTCM].start;
	mem_hdr->gpuaddr = 0;

	/*
	 * Read of GMU TCMs over side-band debug controller interface is
	 * supported on gen8 family
	 * region [20]: Dump ITCM/DTCM. Select 1 for DTCM.
	 * autoInc [31]: Autoincrement the address field after each
	 * access to TCM_DBG_DATA
	 */
	kgsl_regwrite(device, GEN8_CX_DBGC_TCM_DBG_ADDR, BIT(20) | BIT(31));

	for (i = 0; i < (gmu->vma[GMU_DTCM].size >> 2); i++)
		kgsl_regread(device, GEN8_CX_DBGC_TCM_DBG_DATA, data++);

	return gmu->vma[GMU_DTCM].size + sizeof(*mem_hdr);
}

static size_t gen8_gmu_snapshot_itcm(struct kgsl_device *device,
	u8 *buf, size_t remain, void *priv)
{
	struct kgsl_snapshot_gmu_mem *mem_hdr =
			(struct kgsl_snapshot_gmu_mem *)buf;
	void *dest = buf + sizeof(*mem_hdr);
	struct gen8_gmu_device *gmu = (struct gen8_gmu_device *)priv;

	if (!gmu->itcm_shadow) {
		dev_err(GMU_PDEV_DEV(device),
				"No memory allocated for ITCM shadow capture\n");
		return 0;
	}

	if (remain < gmu->vma[GMU_ITCM].size + sizeof(*mem_hdr)) {
		SNAPSHOT_ERR_NOMEM(device, "GMU ITCM Memory");
		return 0;
	}

	mem_hdr->type = SNAPSHOT_GMU_MEM_BIN_BLOCK;
	mem_hdr->hostaddr = 0;
	mem_hdr->gmuaddr = gmu->vma[GMU_ITCM].start;
	mem_hdr->gpuaddr = 0;

	memcpy(dest, gmu->itcm_shadow, gmu->vma[GMU_ITCM].size);

	return gmu->vma[GMU_ITCM].size + sizeof(*mem_hdr);
}

static void gen8_gmu_snapshot_memories(struct kgsl_device *device,
	struct gen8_gmu_device *gmu, struct kgsl_snapshot *snapshot)
{
	struct gmu_mem_type_desc desc;
	struct kgsl_memdesc *md;
	int i;

	for (i = 0; i < ARRAY_SIZE(gmu->gmu_globals); i++) {

		md = &gmu->gmu_globals[i];
		if (!md->size)
			continue;

		desc.memdesc = md;
		if (md == gmu->hfi.hfi_mem)
			desc.type = SNAPSHOT_GMU_MEM_HFI;
		else if (md == gmu->gmu_log)
			desc.type = SNAPSHOT_GMU_MEM_LOG;
		else if (md == gmu->dump_mem)
			desc.type = SNAPSHOT_GMU_MEM_DEBUG;
		else if ((md == gmu->gmu_init_scratch) || (md == gmu->gpu_boot_scratch))
			desc.type = SNAPSHOT_GMU_MEM_WARMBOOT;
		else if (md == gmu->vrb)
			desc.type = SNAPSHOT_GMU_MEM_VRB;
		else if (md == gmu->trace.md)
			desc.type = SNAPSHOT_GMU_MEM_TRACE;
		else
			desc.type = SNAPSHOT_GMU_MEM_BIN_BLOCK;

		kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_GMU_MEMORY,
			snapshot, adreno_snapshot_gmu_mem, &desc);
	}
}

static void gen8_gmu_snapshot_versions(struct kgsl_device *device,
		struct gen8_gmu_device *gmu,
		struct kgsl_snapshot *snapshot)
{
	int i;

	struct kgsl_snapshot_gmu_version gmu_vers[] = {
		{ .type = SNAPSHOT_DEBUG_GMU_CORE_VERSION,
			.value = gmu->ver.core, },
		{ .type = SNAPSHOT_DEBUG_GMU_CORE_DEV_VERSION,
			.value = gmu->ver.core_dev, },
		{ .type = SNAPSHOT_DEBUG_GMU_PWR_VERSION,
			.value = gmu->ver.pwr, },
		{ .type = SNAPSHOT_DEBUG_GMU_PWR_DEV_VERSION,
			.value = gmu->ver.pwr_dev, },
		{ .type = SNAPSHOT_DEBUG_GMU_HFI_VERSION,
			.value = gmu->ver.hfi, },
	};

	for (i = 0; i < ARRAY_SIZE(gmu_vers); i++)
		kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_DEBUG,
				snapshot, adreno_snapshot_gmu_version,
				&gmu_vers[i]);
}

#define RSCC_OFFSET_DWORDS 0x14000

static size_t gen8_snapshot_rscc_registers(struct kgsl_device *device, u8 *buf,
	size_t remain, void *priv)
{
	const u32 *regs = priv;
	u32 *data = (u32 *)buf;
	int count = 0, k;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);

	/* Figure out how many registers we are going to dump */
	count = adreno_snapshot_regs_count(regs);

	if (remain < (count * 4)) {
		SNAPSHOT_ERR_NOMEM(device, "RSCC REGISTERS");
		return 0;
	}

	for (regs = priv; regs[0] != UINT_MAX; regs += 2) {
		u32 cnt = REG_COUNT(regs);

		if (cnt == 1) {
			*data++ = BIT(31) |  regs[0];
			*data++ =  __raw_readl(gmu->rscc_virt +
				((regs[0] - RSCC_OFFSET_DWORDS) << 2));
			continue;
		}
		*data++ = regs[0];
		*data++ = cnt;
		for (k = regs[0]; k <= regs[1]; k++)
			*data++ = __raw_readl(gmu->rscc_virt +
				((k - RSCC_OFFSET_DWORDS) << 2));
	}

	/* Return the size of the section */
	return (count * 4);
}

/*
 * gen8_gmu_device_snapshot() - GEN8 GMU snapshot function
 * @device: Device being snapshotted
 * @snapshot: Pointer to the snapshot instance
 *
 * This is where all of the GEN8 GMU specific bits and pieces are grabbed
 * into the snapshot memory
 */
static void gen8_gmu_device_snapshot(struct kgsl_device *device,
	struct kgsl_snapshot *snapshot)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	const struct adreno_gen8_core *gpucore = to_gen8_core(adreno_dev);
	const struct gen8_snapshot_block_list *gen8_snapshot_block_list =
						gpucore->gen8_snapshot_block_list;
	const u32 *regs_ptr = (const u32 *)gen8_snapshot_block_list->cx_misc_regs;
	u32 i, slice, j;
	struct gen8_reg_list_info info = {0};

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_GMU_MEMORY,
		snapshot, gen8_gmu_snapshot_itcm, gmu);

	gen8_gmu_snapshot_versions(device, gmu, snapshot);

	gen8_gmu_snapshot_memories(device, gmu, snapshot);

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS_V2, snapshot,
		gen8_snapshot_rscc_registers, (void *) gen8_snapshot_block_list->rscc_regs);

	/*
	 * We want to capture these through AHB path here because we might skip them
	 * in the crashdumper path if GX is OFF, and these are needed for debug.
	 */
	if (!kgsl_regmap_valid_offset(&device->regmap, regs_ptr[0]))
		WARN_ONCE(1, "cx_misc registers are not defined in device tree");
	else
		kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS_V2,
			snapshot, adreno_snapshot_registers_v2, (void *)regs_ptr);

	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_GMU_MEMORY,
		snapshot, gen8_gmu_snapshot_dtcm, gmu);

	/* Capture GMU registers which are on CX domain and unsliced */
	kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_REGS_V2, snapshot,
		adreno_snapshot_registers_v2,
		(void *) gen8_snapshot_block_list->gmu_cx_unsliced_regs);

	if (!gen8_gmu_rpmh_pwr_state_is_active(device) ||
		!gen8_gmu_gx_is_on(adreno_dev))
		return;

	/* Set fence to ALLOW mode so registers can be read */
	kgsl_regwrite(device, GEN8_GMUAO_AHB_FENCE_CTRL, 0);

	/* Capture GMU registers which are on GX domain */
	for (i = 0 ; i < gen8_snapshot_block_list->num_gmu_gx_regs; i++) {
		struct gen8_reg_list *regs = &gen8_snapshot_block_list->gmu_gx_regs[i];

		slice = NUMBER_OF_SLICES(regs->slice_region, adreno_dev);
		for (j = 0 ; j < slice; j++) {
			info.regs = regs;
			info.slice_id = SLICE_ID(regs->slice_region, j);
			kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_MVC_V3, snapshot,
				gen8_legacy_snapshot_registers, &info);
		}
	}
}

void gen8_gmu_snapshot(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	/*
	 * Dump external register first to have GPUCC and other external
	 * register in snapshot to analyze the system state even in partial
	 * snapshot dump
	 */
	gen8_snapshot_external_core_regs(device, snapshot);

	gen8_gmu_device_snapshot(device, snapshot);

	gen8_snapshot(adreno_dev, snapshot);

	gmu_core_regwrite(device, GEN8_GMUCX_GMU2HOST_INTR_CLR, UINT_MAX);
	gmu_core_regwrite(device, GEN8_GMUCX_GMU2HOST_INTR_MASK, HFI_IRQ_MASK);
}
