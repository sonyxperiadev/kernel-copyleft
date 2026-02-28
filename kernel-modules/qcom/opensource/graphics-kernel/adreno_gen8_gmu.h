/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __ADRENO_GEN8_GMU_H
#define __ADRENO_GEN8_GMU_H

#include <linux/soc/qcom/qcom_aoss.h>

#include "adreno_gen8_hfi.h"
#include "kgsl_gmu_core.h"

struct gen8_dcvs_table {
	u32 gpu_level_num;
	u32 gmu_level_num;
	struct opp_gx_desc gx_votes[MAX_GX_LEVELS];
	struct opp_desc cx_votes[MAX_CX_LEVELS];
};

/**
 * struct gen8_gmu_device - GMU device structure
 * @ver: GMU Version information
 * @irq: GMU interrupt number
 * @fw_image: GMU FW image
 * @dump_mem: pointer to GMU debug dump memory
 * @gmu_log: gmu event log memory
 * @hfi: HFI controller
 * @clks: GPU subsystem clocks required for GMU functionality
 * @idle_level: Minimal GPU idle power level
 * @log_wptr_retention: Store the log wptr offset on slumber
 */
struct gen8_gmu_device {
	struct {
		u32 core;
		u32 core_dev;
		u32 pwr;
		u32 pwr_dev;
		u32 hfi;
	} ver;
	int irq;
	const struct firmware *fw_image;
	struct kgsl_memdesc *dump_mem;
	struct kgsl_memdesc *gmu_log;
	/** @vrb: GMU virtual register bank memory */
	struct kgsl_memdesc *vrb;
	/** @trace: gmu trace container */
	struct kgsl_gmu_trace trace;
	/** @gmu_init_scratch: Memory to store the initial HFI messages */
	struct kgsl_memdesc *gmu_init_scratch;
	/** @gpu_boot_scratch: Memory to store the bootup HFI messages */
	struct kgsl_memdesc *gpu_boot_scratch;
	struct gen8_hfi hfi;
	/** @pwrlevels: Array of GMU power levels */
	struct clk_bulk_data *clks;
	/** @num_clks: Number of entries in the @clks array */
	int num_clks;
	u32 idle_level;
	/** @freqs: Array of GMU frequencies */
	u32 freqs[GMU_MAX_PWRLEVELS];
	/** @vlvls: Array of GMU voltage levels */
	u32 vlvls[GMU_MAX_PWRLEVELS];
	/** @qmp: aoss_qmp handle */
	struct qmp *qmp;
	/** @gmu_globals: Array to store gmu global buffers */
	struct kgsl_memdesc gmu_globals[GMU_KERNEL_ENTRIES];
	/** @global_entries: To keep track of number of gmu buffers */
	u32 global_entries;
	struct gmu_vma_entry *vma;
	u32 log_wptr_retention;
	/** @cm3_fault: whether gmu received a cm3 fault interrupt */
	atomic_t cm3_fault;
	/**
	 * @itcm_shadow: Copy of the itcm block in firmware binary used for
	 * snapshot
	 */
	void *itcm_shadow;
	/** @flags: Internal gmu flags */
	unsigned long flags;
	/** @rscc_virt: Pointer where RSCC block is mapped */
	void __iomem *rscc_virt;
	/** @log_stream_enable: GMU log streaming enable. Disabled by default */
	bool log_stream_enable;
	/** @log_group_mask: Allows overriding default GMU log group mask */
	u32 log_group_mask;
	struct kobject log_kobj;
	/*
	 * @perf_ddr_bw: The lowest ddr bandwidth that puts CX at a corner at
	 * which GMU can run at higher frequency.
	 */
	u32 perf_ddr_bw;
	/** @rdpm_cx_virt: Pointer where the RDPM CX block is mapped */
	void __iomem *rdpm_cx_virt;
	/** @rdpm_mx_virt: Pointer where the RDPM MX block is mapped */
	void __iomem *rdpm_mx_virt;
	/** @num_oob_perfcntr: Number of active oob_perfcntr requests */
	u32 num_oob_perfcntr;
	/** @acd_debug_val: DVM value to calibrate ACD for a level */
	u32 acd_debug_val;
	/** @stats_enable: GMU stats feature enable */
	bool stats_enable;
	/** @stats_mask: GMU performance countables to enable */
	u32 stats_mask;
	/** @stats_interval: GMU performance counters sampling interval */
	u32 stats_interval;
	/** @stats_kobj: kernel object for GMU stats directory in sysfs */
	struct kobject stats_kobj;
	/** @cp_init_hdr: raw command header for cp_init */
	u32 cp_init_hdr;
	/** @switch_to_unsec_hdr: raw command header for switch to unsecure packet */
	u32 switch_to_unsec_hdr;
	/** @dcvs_table: Table for gpu dcvs levels */
	struct gen8_dcvs_table dcvs_table;
	/** @cur_freq: Tracks scaled frequency for GMU */
	u32 cur_freq;
};

/* Helper function to get to gen8 gmu device from adreno device */
struct gen8_gmu_device *to_gen8_gmu(struct adreno_device *adreno_dev);

/* Helper function to get to adreno device from gen8 gmu device */
struct adreno_device *gen8_gmu_to_adreno(struct gen8_gmu_device *gmu);

/**
 * gen8_reserve_gmu_kernel_block() - Allocate a global gmu buffer
 * @gmu: Pointer to the gen8 gmu device
 * @addr: Desired gmu virtual address
 * @size: Size of the buffer in bytes
 * @vma_id: Target gmu vma where this buffer should be mapped
 * @align: Alignment for the GMU VA and GMU mapping size
 *
 * This function allocates a global gmu buffer and maps it in
 * the desired gmu vma
 *
 * Return: Pointer to the memory descriptor or error pointer on failure
 */
struct kgsl_memdesc *gen8_reserve_gmu_kernel_block(struct gen8_gmu_device *gmu,
		u32 addr, u32 size, u32 vma_id, u32 align);

/**
 * gen8_reserve_gmu_kernel_block_fixed() - Maps phyical resource address to gmu
 * @gmu: Pointer to the gen8 gmu device
 * @addr: Desired gmu virtual address
 * @size: Size of the buffer in bytes
 * @vma_id: Target gmu vma where this buffer should be mapped
 * @resource: Name of the resource to get the size and address to allocate
 * @attrs: Attributes for the mapping
 * @align: Alignment for the GMU VA and GMU mapping size
 *
 * This function maps the physcial resource address to desired gmu vma
 *
 * Return: Pointer to the memory descriptor or error pointer on failure
 */
struct kgsl_memdesc *gen8_reserve_gmu_kernel_block_fixed(struct gen8_gmu_device *gmu,
	u32 addr, u32 size, u32 vma_id, const char *resource, int attrs, u32 align);

/**
 * gen8_alloc_gmu_kernel_block() - Allocate a gmu buffer
 * @gmu: Pointer to the gen8 gmu device
 * @md: Pointer to the memdesc
 * @size: Size of the buffer in bytes
 * @vma_id: Target gmu vma where this buffer should be mapped
 * @attrs: Attributes for the mapping
 *
 * This function allocates a buffer and maps it in the desired gmu vma
 *
 * Return: 0 on success or error code on failure
 */
int gen8_alloc_gmu_kernel_block(struct gen8_gmu_device *gmu,
	struct kgsl_memdesc *md, u32 size, u32 vma_id, int attrs);

/**
 * gen8_gmu_import_buffer() - Import a gmu buffer
 * @gmu: Pointer to the gen8 gmu device
 * @vma_id: Target gmu vma where this buffer should be mapped
 * @md: Pointer to the memdesc to be mapped
 * @attrs: Attributes for the mapping
 * @align: Alignment for the GMU VA and GMU mapping size
 *
 * This function imports and maps a buffer to a gmu vma
 *
 * Return: 0 on success or error code on failure
 */
int gen8_gmu_import_buffer(struct gen8_gmu_device *gmu, u32 vma_id,
			struct kgsl_memdesc *md, u32 attrs, u32 align);

/**
 * gen8_free_gmu_block() - Free a gmu buffer
 * @gmu: Pointer to the gen8 gmu device
 * @md: Pointer to the memdesc that is to be freed
 *
 * This function frees a gmu block allocated by gen8_reserve_gmu_kernel_block()
 */
void gen8_free_gmu_block(struct gen8_gmu_device *gmu, struct kgsl_memdesc *md);

/**
 * gen8_build_rpmh_tables - Build the rpmh tables
 * @adreno_dev: Pointer to the adreno device
 *
 * This function creates the gpu dcvs and bw tables
 *
 * Return: 0 on success and negative error on failure
 */
int gen8_build_rpmh_tables(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_gx_is_on - Check if GX is on
 * @adreno_dev: Pointer to the adreno device
 *
 * This function reads pwr status registers to check if GX
 * is on or off
 */
bool gen8_gmu_gx_is_on(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_device_probe - GEN8 GMU snapshot function
 * @pdev: Pointer to the platform device
 * @chipid: Chipid of the target
 * @gpucore: Pointer to the gpucore
 *
 * The target specific probe function for gmu based gen8 targets.
 */
int gen8_gmu_device_probe(struct platform_device *pdev,
		u32 chipid, const struct adreno_gpu_core *gpucore);

/**
 * gen8_gmu_reset - Reset and restart the gmu
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_reset(struct adreno_device *adreno_dev);

/**
 * gen8_enable_gpu_irq - Enable gpu interrupt
 * @adreno_dev: Pointer to the adreno device
 */
void gen8_enable_gpu_irq(struct adreno_device *adreno_dev);

/**
 * gen8_disable_gpu_irq - Disable gpu interrupt
 * @adreno_dev: Pointer to the adreno device
 */
void gen8_disable_gpu_irq(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_snapshot- Take snapshot for gmu targets
 * @adreno_dev: Pointer to the adreno device
 * @snapshot: Pointer to the snapshot structure
 *
 * Send an NMI to gmu if we hit a gmu fault. Then take gmu
 * snapshot and carry on with rest of the gen8 snapshot
 */
void gen8_gmu_snapshot(struct adreno_device *adreno_dev,
		struct kgsl_snapshot *snapshot);

/**
 * gen8_gmu_probe - Probe gen8 gmu resources
 * @device: Pointer to the kgsl device
 * @pdev: Pointer to the gmu platform device
 *
 * Probe the gmu and hfi resources
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_probe(struct kgsl_device *device,
		struct platform_device *pdev);

/**
 * gen8_gmu_parse_fw - Parse the gmu fw binary
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_parse_fw(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_memory_init - Allocate gmu memory
 * @adreno_dev: Pointer to the adreno device
 *
 * Allocates the gmu log buffer and others if ndeeded.
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_memory_init(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_aop_send_acd_state - Enable or disable acd feature in aop
 * @gmu: Pointer to the gen8 gmu device
 * @flag: Boolean to enable or disable acd in aop
 *
 * This function enables or disables gpu acd feature using qmp
 */
void gen8_gmu_aop_send_acd_state(struct gen8_gmu_device *gmu, bool flag);

/**
 * gen8_gmu_load_fw - Load gmu firmware
 * @adreno_dev: Pointer to the adreno device
 *
 * Loads the gmu firmware binary into TCMs and memory
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_load_fw(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_device_start - Bring gmu out of reset
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_device_start(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_hfi_start - Indicate hfi start to gmu
 * @device: Pointer to the kgsl device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_hfi_start(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_itcm_shadow - Create itcm shadow copy for snapshot
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_itcm_shadow(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_register_config - gmu register configuration
 * @adreno_dev: Pointer to the adreno device
 *
 * Program gmu regsiters based on features
 */
void gen8_gmu_register_config(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_version_info - Get gmu firmware version
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_version_info(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_irq_enable - Enable gmu interrupts
 * @adreno_dev: Pointer to the adreno device
 */
void gen8_gmu_irq_enable(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_irq_disable - Disaable gmu interrupts
 * @adreno_dev: Pointer to the adreno device
 */
void gen8_gmu_irq_disable(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_suspend - Hard reset the gpu and gmu
 * @adreno_dev: Pointer to the adreno device
 *
 * In case we hit a gmu fault, hard reset the gpu and gmu
 * to recover from the fault
 */
void gen8_gmu_suspend(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_oob_set - send gmu oob request
 * @device: Pointer to the kgsl device
 * @req: Type of oob request as defined in enum oob_request
 *
 * Request gmu to keep gpu powered up till the oob is cleared
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_oob_set(struct kgsl_device *device, enum oob_request oob);

/**
 * gen8_gmu_oob_clear - clear an asserted oob request
 * @device: Pointer to the kgsl device
 * @req: Type of oob request as defined in enum oob_request
 *
 * Clear a previously requested oob so that gmu can power
 * collapse the gpu
 */
void gen8_gmu_oob_clear(struct kgsl_device *device, enum oob_request oob);

/**
 * gen8_gmu_wait_for_lowest_idle - wait for gmu to complete ifpc
 * @adreno_dev: Pointer to the adreno device
 *
 * If ifpc is enabled, wait for gmu to put gpu into ifpc.
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_wait_for_lowest_idle(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_wait_for_idle - Wait for gmu to become idle
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_wait_for_idle(struct adreno_device *adreno_dev);

/**
 * gen8_rscc_sleep_sequence - Trigger rscc sleep sequence
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_rscc_sleep_sequence(struct adreno_device *adreno_dev);

/**
 * gen8_rscc_wakeup_sequence - Trigger rscc wakeup sequence
 * @adreno_dev: Pointer to the adreno device
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_rscc_wakeup_sequence(struct adreno_device *adreno_dev);

/**
 * gen8_halt_gbif - Halt CX and GX requests in GBIF
 * @adreno_dev: Pointer to the adreno device
 *
 * Clear any pending GX or CX transactions in GBIF and
 * deassert GBIF halt
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_halt_gbif(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_remove - Clean up gmu probed resources
 * @device: Pointer to the kgsl device
 */
void gen8_gmu_remove(struct kgsl_device *device);

/**
 * gen8_gmu_enable_clks - Enable gmu clocks
 * @adreno_dev: Pointer to the adreno device
 * @level: GMU frequency level
 *
 * Return: 0 on success or negative error on failure
 */
int gen8_gmu_enable_clks(struct adreno_device *adreno_dev, u32 level);

/**
 * gen8_gmu_handle_watchdog - Handle watchdog interrupt
 * @adreno_dev: Pointer to the adreno device
 */
void gen8_gmu_handle_watchdog(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_send_nmi - Send NMI to GMU
 * @device: Pointer to the kgsl device
 * @gf_policy: GMU fault panic setting policy
 * @force: Boolean to forcefully send NMI irrespective of GMU state
 */
void gen8_gmu_send_nmi(struct kgsl_device *device, bool force,
		       enum gmu_fault_panic_policy gf_policy);

/**
 * gen8_gmu_add_to_minidump - Register gen8_device with va minidump
 * @adreno_dev: Pointer to the adreno device
 */
int gen8_gmu_add_to_minidump(struct adreno_device *adreno_dev);

/**
 * gen8_gmu_clock_set_rate - Set the gmu clock rate
 * @adreno_dev: Handle to the adreno device
 * @req_freq: Requested freq to set gmu to
 *
 * Returns 0 on success or error on clock set rate failure
 */
int gen8_gmu_clock_set_rate(struct adreno_device *adreno_dev, u32 req_freq);

/**
 * gen8_gmu_rpmh_pwr_state_is_active - Check the state of GPU HW
 * @device: Pointer to the kgsl device
 *
 * Returns true on active or false otherwise
 */
bool gen8_gmu_rpmh_pwr_state_is_active(struct kgsl_device *device);
#endif
