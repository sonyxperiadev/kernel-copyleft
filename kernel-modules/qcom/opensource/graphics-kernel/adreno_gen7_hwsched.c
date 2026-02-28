// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/interconnect.h>

#include "adreno.h"
#include "adreno_gen7.h"
#include "adreno_gen7_hwsched.h"
#include "adreno_snapshot.h"
#include "kgsl_bus.h"
#include "kgsl_device.h"
#include "kgsl_trace.h"

static void _wakeup_hw_fence_waiters(struct adreno_device *adreno_dev, u32 fault)
{
	struct gen7_hwsched_hfi *hfi = to_gen7_hwsched_hfi(adreno_dev);
	bool lock = !in_interrupt();

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &adreno_dev->hwsched.flags))
		return;

	/*
	 * We could be in interrupt context here, which means we need to use spin_lock_irqsave
	 * (which disables interrupts) everywhere we take this lock. Instead of that, simply
	 * avoid taking this lock if we are recording a fault from an interrupt handler.
	 */
	if (lock)
		spin_lock(&hfi->hw_fence.lock);

	clear_bit(GEN7_HWSCHED_HW_FENCE_SLEEP_BIT, &hfi->hw_fence.flags);

	/* Avoid creating new hardware fences until recovery is complete */
	set_bit(GEN7_HWSCHED_HW_FENCE_ABORT_BIT, &hfi->hw_fence.flags);

	if (!lock)
		/*
		 * This barrier ensures that the above bitops complete before we wake up the waiters
		 */
		smp_wmb();
	else
		spin_unlock(&hfi->hw_fence.lock);

	wake_up_all(&hfi->hw_fence.unack_wq);

	del_timer_sync(&hfi->hw_fence_timer);
}

void gen7_hwsched_fault(struct adreno_device *adreno_dev, u32 fault)
{
	/*
	 * Wake up any threads that may be sleeping waiting for the hardware fence unack count to
	 * drop to a desired threshold.
	 */
	_wakeup_hw_fence_waiters(adreno_dev, fault);

	adreno_scheduler_fault(adreno_dev, fault);
}

static void gen7_hwsched_snapshot_preemption_records(struct kgsl_device *device,
	struct kgsl_snapshot *snapshot, struct kgsl_memdesc *md)
{
	u64 ctxt_record_size = md->size;
	u64 offset;

	do_div(ctxt_record_size, KGSL_PRIORITY_MAX_RB_LEVELS);

	/* All preemption records exist as a single mem alloc entry */
	for (offset = 0; offset < md->size; offset += ctxt_record_size)
		adreno_hwsched_snapshot_preemption_record(device, snapshot, md,
				offset, ctxt_record_size);
}

void gen7_hwsched_snapshot(struct adreno_device *adreno_dev,
	struct kgsl_snapshot *snapshot)
{
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_hwsched_hfi *hw_hfi = to_gen7_hwsched_hfi(adreno_dev);
	bool skip_memkind_rb = false;
	u32 i;
	bool parse_payload;

	gen7_gmu_snapshot(adreno_dev, snapshot);

	adreno_hwsched_parse_fault_cmdobj(adreno_dev, snapshot);

	/*
	 * First try to dump ringbuffers using context bad HFI payloads
	 * because they have all the ringbuffer parameters. If ringbuffer
	 * payloads are not present, fall back to dumping ringbuffers
	 * based on MEMKIND_RB
	 */
	if (GMU_VER_MINOR(gmu->ver.hfi) < 2)
		parse_payload = adreno_hwsched_parse_payload_rb_legacy(adreno_dev, snapshot);
	else
		parse_payload = adreno_hwsched_parse_payload_rb(adreno_dev, snapshot);

	if (parse_payload)
		skip_memkind_rb = true;

	for (i = 0; i < hw_hfi->mem_alloc_entries; i++) {
		struct hfi_mem_alloc_entry *entry = &hw_hfi->mem_alloc_table[i];

		if (entry->desc.mem_kind == HFI_MEMKIND_RB && !skip_memkind_rb)
			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_RB_V2,
				snapshot, adreno_hwsched_snapshot_rb,
				entry->md);

		if (entry->desc.mem_kind == HFI_MEMKIND_SCRATCH)
			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
				snapshot, adreno_snapshot_global,
				entry->md);

		if (entry->desc.mem_kind == HFI_MEMKIND_PROFILE)
			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
				snapshot, adreno_snapshot_global,
				entry->md);

		if (entry->desc.mem_kind == HFI_MEMKIND_CSW_SMMU_INFO)
			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
				snapshot, adreno_snapshot_global,
				entry->md);

		if (entry->desc.mem_kind == HFI_MEMKIND_CSW_PRIV_NON_SECURE)
			gen7_hwsched_snapshot_preemption_records(device, snapshot,
				entry->md);

		if (entry->desc.mem_kind == HFI_MEMKIND_PREEMPT_SCRATCH)
			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
				snapshot, adreno_snapshot_global,
				entry->md);

		if (entry->desc.mem_kind == HFI_MEMKIND_AQE_BUFFER)
			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_GPU_OBJECT_V2,
				snapshot, adreno_hwsched_snapshot_aqe_buffer,
				entry->md);

		if (entry->desc.mem_kind == HFI_MEMKIND_HW_FENCE) {
			struct gmu_mem_type_desc desc;

			desc.memdesc = entry->md;
			desc.type = SNAPSHOT_GMU_MEM_HW_FENCE;

			kgsl_snapshot_add_section(device,
				KGSL_SNAPSHOT_SECTION_GMU_MEMORY,
				snapshot, adreno_snapshot_gmu_mem, &desc);
		}

	}

	if (adreno_hwsched_context_queue_enabled(adreno_dev))
		adreno_hwsched_snapshot_context_queue(device, snapshot);
}

static int gen7_hwsched_gmu_first_boot(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int level, ret = 0;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_AWARE);

	gen7_gmu_aop_send_acd_state(gmu, adreno_dev->acd_enabled);

	ret = kgsl_pwrctrl_enable_cx_gdsc(device);
	if (ret)
		return ret;

	ret = gen7_gmu_enable_clks(adreno_dev, GMU_MAX_PWRLEVELS - 1);
	if (ret)
		goto gdsc_off;

	/*
	 * Enable AHB timeout detection to catch any register access taking longer
	 * time before NOC timeout gets detected. Enable this logic before any
	 * register access which happens to be just after enabling clocks.
	 */
	gen7_enable_ahb_timeout_detection(adreno_dev);

	/* Initialize the CX timer */
	gen7_cx_timer_init(adreno_dev);

	ret = gen7_gmu_load_fw(adreno_dev);
	if (ret)
		goto clks_gdsc_off;

	ret = gen7_gmu_itcm_shadow(adreno_dev);
	if (ret)
		goto clks_gdsc_off;

	if (!test_bit(GMU_PRIV_PDC_RSC_LOADED, &gmu->flags)) {
		ret = gen7_load_pdc_ucode(adreno_dev);
		if (ret)
			goto clks_gdsc_off;

		gen7_load_rsc_ucode(adreno_dev);
		set_bit(GMU_PRIV_PDC_RSC_LOADED, &gmu->flags);
	}

	ret = gen7_scm_gpu_init_cx_regs(adreno_dev);
	if (ret)
		goto clks_gdsc_off;

	gen7_gmu_register_config(adreno_dev);

	ret = gen7_gmu_version_info(adreno_dev);
	if (ret)
		goto clks_gdsc_off;

	if (GMU_VER_MINOR(gmu->ver.hfi) < 2)
		set_bit(ADRENO_HWSCHED_CTX_BAD_LEGACY, &adreno_dev->hwsched.flags);

	gen7_gmu_irq_enable(adreno_dev);

	/* Vote for minimal DDR BW for GMU to init */
	level = pwr->pwrlevels[pwr->default_pwrlevel].bus_min;

	/* From this GMU FW all RBBM interrupts are handled at GMU */
	if (gmu->ver.core >= GMU_VERSION(5, 01, 06))
		adreno_irq_free(adreno_dev);

	icc_set_bw(pwr->icc_path, 0, kBps_to_icc(pwr->ddr_table[level]));

	/* Clear any hwsched faults that might have been left over */
	adreno_clear_gpu_fault(adreno_dev);

	ret = gen7_gmu_device_start(adreno_dev);
	if (ret)
		goto err;

	gen7_get_gpu_feature_info(adreno_dev);

	ret = gen7_hwsched_hfi_start(adreno_dev);
	if (ret)
		goto err;

	ret = gen7_gmu_clock_set_rate(adreno_dev, gmu->freqs[0]);
	if (ret) {
		gen7_hwsched_hfi_stop(adreno_dev);
		goto err;
	}

	if (gen7_hwsched_hfi_get_value(adreno_dev, HFI_VALUE_GMU_AB_VOTE) == 1 &&
		!WARN_ONCE(!adreno_dev->gpucore->num_ddr_channels,
			"Number of DDR channel is not specified in gpu core")) {
		adreno_dev->gmu_ab = true;
		set_bit(ADRENO_DEVICE_GMU_AB, &adreno_dev->priv);
	}

	icc_set_bw(pwr->icc_path, 0, 0);

	device->gmu_fault = false;

	kgsl_pwrctrl_set_state(device, KGSL_STATE_AWARE);

	return 0;

err:
	gen7_gmu_irq_disable(adreno_dev);

	if (device->gmu_fault) {
		gen7_gmu_suspend(adreno_dev);

		return ret;
	}

clks_gdsc_off:
	clk_bulk_disable_unprepare(gmu->num_clks, gmu->clks);

gdsc_off:
	kgsl_pwrctrl_disable_cx_gdsc(device);

	gen7_rdpm_cx_freq_update(gmu, 0);

	return ret;
}

static int gen7_hwsched_gmu_boot(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret = 0;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_AWARE);

	ret = kgsl_pwrctrl_enable_cx_gdsc(device);
	if (ret)
		return ret;

	ret = gen7_gmu_enable_clks(adreno_dev, GMU_MAX_PWRLEVELS - 1);
	if (ret)
		goto gdsc_off;

	/*
	 * Enable AHB timeout detection to catch any register access taking longer
	 * time before NOC timeout gets detected. Enable this logic before any
	 * register access which happens to be just after enabling clocks.
	 */
	gen7_enable_ahb_timeout_detection(adreno_dev);

	/* Initialize the CX timer */
	gen7_cx_timer_init(adreno_dev);

	ret = gen7_rscc_wakeup_sequence(adreno_dev);
	if (ret)
		goto clks_gdsc_off;

	ret = gen7_gmu_load_fw(adreno_dev);
	if (ret)
		goto clks_gdsc_off;

	gen7_gmu_register_config(adreno_dev);

	gen7_gmu_irq_enable(adreno_dev);

	/* Clear any hwsched faults that might have been left over */
	adreno_clear_gpu_fault(adreno_dev);

	ret = gen7_gmu_device_start(adreno_dev);
	if (ret)
		goto err;

	ret = gen7_hwsched_hfi_start(adreno_dev);
	if (ret)
		goto err;

	ret = gen7_gmu_clock_set_rate(adreno_dev, gmu->freqs[0]);
	if (ret) {
		gen7_hwsched_hfi_stop(adreno_dev);
		goto err;
	}

	device->gmu_fault = false;

	kgsl_pwrctrl_set_state(device, KGSL_STATE_AWARE);

	return 0;
err:
	gen7_gmu_irq_disable(adreno_dev);

	if (device->gmu_fault) {
		gen7_gmu_suspend(adreno_dev);

		return ret;
	}

clks_gdsc_off:
	clk_bulk_disable_unprepare(gmu->num_clks, gmu->clks);

gdsc_off:
	kgsl_pwrctrl_disable_cx_gdsc(device);

	gen7_rdpm_cx_freq_update(gmu, 0);

	return ret;
}

static int gen7_hwsched_notify_slumber(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	struct hfi_prep_slumber_cmd req;
	int ret;

	ret = CMD_MSG_HDR(req, H2F_MSG_PREPARE_SLUMBER);
	if (ret)
		return ret;

	req.freq = gmu->dcvs_table.gpu_level_num - pwr->default_pwrlevel - 1;
	req.bw = pwr->pwrlevels[pwr->default_pwrlevel].bus_freq;

	req.bw |= adreno_gmu_bus_ab_quantize(adreno_dev, 0);
	/* Disable the power counter so that the GMU is not busy */
	gmu_core_regwrite(device, GEN7_GMU_CX_GMU_POWER_COUNTER_ENABLE, 0);

	ret = gen7_hfi_send_cmd_async(adreno_dev, &req, sizeof(req));

	/*
	 * GEMNOC can enter power collapse state during GPU power down sequence.
	 * This could abort CX GDSC collapse. Assert Qactive to avoid this.
	 */
	gmu_core_regwrite(device, GEN7_GPU_GMU_CX_GMU_CX_FALNEXT_INTF, 0x1);

	return ret;
}
static int gen7_hwsched_gmu_power_off(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret = 0;

	if (device->gmu_fault)
		goto error;

	/* Wait for the lowest idle level we requested */
	ret = gen7_gmu_wait_for_lowest_idle(adreno_dev);
	if (ret)
		goto error;

	ret = gen7_hwsched_notify_slumber(adreno_dev);
	if (ret)
		goto error;

	ret = gen7_gmu_wait_for_idle(adreno_dev);
	if (ret)
		goto error;

	ret = gen7_rscc_sleep_sequence(adreno_dev);

	gen7_rdpm_mx_freq_update(gmu, 0);

	/* Now that we are done with GMU and GPU, Clear the GBIF */
	ret = gen7_halt_gbif(adreno_dev);

	gen7_gmu_irq_disable(adreno_dev);

	gen7_hwsched_hfi_stop(adreno_dev);

	clk_bulk_disable_unprepare(gmu->num_clks, gmu->clks);

	kgsl_pwrctrl_disable_cx_gdsc(device);

	gen7_rdpm_cx_freq_update(gmu, 0);

	kgsl_pwrctrl_set_state(device, KGSL_STATE_NONE);

	return ret;

error:
	gen7_gmu_irq_disable(adreno_dev);
	gen7_hwsched_hfi_stop(adreno_dev);
	gen7_gmu_suspend(adreno_dev);

	return ret;
}

static void gen7_hwsched_init_ucode_regs(struct adreno_device *adreno_dev)
{
	struct adreno_firmware *fw = ADRENO_FW(adreno_dev, ADRENO_FW_SQE);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	/* Program the ucode base for CP */
	kgsl_regwrite(device, GEN7_CP_SQE_INSTR_BASE_LO,
		lower_32_bits(fw->memdesc->gpuaddr));
	kgsl_regwrite(device, GEN7_CP_SQE_INSTR_BASE_HI,
		upper_32_bits(fw->memdesc->gpuaddr));

	if (ADRENO_FEATURE(adreno_dev, ADRENO_AQE)) {
		fw = ADRENO_FW(adreno_dev, ADRENO_FW_AQE);

		/* Program the ucode base for AQE0 (BV coprocessor) */
		kgsl_regwrite(device, GEN7_CP_AQE_INSTR_BASE_LO_0,
			lower_32_bits(fw->memdesc->gpuaddr));
		kgsl_regwrite(device, GEN7_CP_AQE_INSTR_BASE_HI_0,
			upper_32_bits(fw->memdesc->gpuaddr));

		/* Program the ucode base for AQE1 (LPAC coprocessor) */
		if (adreno_dev->lpac_enabled) {
			kgsl_regwrite(device, GEN7_CP_AQE_INSTR_BASE_LO_1,
				      lower_32_bits(fw->memdesc->gpuaddr));
			kgsl_regwrite(device, GEN7_CP_AQE_INSTR_BASE_HI_1,
				      upper_32_bits(fw->memdesc->gpuaddr));
		}
	}
}

static int gen7_hwsched_gpu_boot(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	ret = kgsl_mmu_start(device);
	if (ret)
		goto err;

	ret = gen7_gmu_oob_set(device, oob_gpu);
	if (ret)
		goto err;

	/* Clear the busy_data stats - we're starting over from scratch */
	memset(&adreno_dev->busy_data, 0, sizeof(adreno_dev->busy_data));

	gen7_start(adreno_dev);

	/* Re-initialize the coresight registers if applicable */
	adreno_coresight_start(adreno_dev);

	adreno_perfcounter_start(adreno_dev);

	/* Clear FSR here in case it is set from a previous pagefault */
	kgsl_mmu_clear_fsr(&device->mmu);

	gen7_enable_gpu_irq(adreno_dev);

	gen7_hwsched_init_ucode_regs(adreno_dev);

	ret = gen7_hwsched_boot_gpu(adreno_dev);
	if (ret)
		goto err;

	/*
	 * At this point it is safe to assume that we recovered. Setting
	 * this field allows us to take a new snapshot for the next failure
	 * if we are prioritizing the first unrecoverable snapshot.
	 */
	if (device->snapshot)
		device->snapshot->recovered = true;

	device->reset_counter++;

	/*
	 * If warmboot is enabled and we switched a sysfs node, we will do a coldboot
	 * in the subseqent slumber exit. Once that is done we need to mark this bool
	 * as false so that in the next run we can do warmboot
	 */
	clear_bit(ADRENO_DEVICE_FORCE_COLDBOOT, &adreno_dev->priv);
err:
	gen7_gmu_oob_clear(device, oob_gpu);

	if (ret)
		gen7_hwsched_gmu_power_off(adreno_dev);

	return ret;
}

static void hwsched_idle_timer(struct timer_list *t)
{
	struct kgsl_device *device = container_of(t, struct kgsl_device,
					idle_timer);

	kgsl_schedule_work(&device->idle_check_ws);
}

static int gen7_gmu_warmboot_init(struct adreno_device *adreno_dev)
{
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret = 0;

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_GMU_WARMBOOT))
		return ret;

	if (IS_ERR_OR_NULL(gmu->gmu_init_scratch)) {
		gmu->gmu_init_scratch = gen7_reserve_gmu_kernel_block(gmu, 0,
				SZ_4K, GMU_CACHE, 0);
		ret = PTR_ERR_OR_ZERO(gmu->gmu_init_scratch);
		if (ret)
			return ret;
	}

	if (IS_ERR_OR_NULL(gmu->gpu_boot_scratch)) {
		gmu->gpu_boot_scratch = gen7_reserve_gmu_kernel_block(gmu, 0,
				SZ_4K, GMU_CACHE, 0);
		ret = PTR_ERR_OR_ZERO(gmu->gpu_boot_scratch);
	}

	return ret;
}

static int gen7_hwsched_gmu_memory_init(struct adreno_device *adreno_dev)
{
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret;

	/* GMU Virtual register bank */
	if (IS_ERR_OR_NULL(gmu->vrb)) {
		gmu->vrb = gen7_reserve_gmu_kernel_block(gmu, 0, GMU_VRB_SIZE,
						GMU_NONCACHED_KERNEL, 0);

		if (IS_ERR(gmu->vrb))
			return PTR_ERR(gmu->vrb);

		/* Populate size of the virtual register bank */
		ret = gmu_core_set_vrb_register(gmu->vrb, VRB_SIZE_IDX, gmu->vrb->size >> 2);
		if (ret)
			return ret;
	}

	/* GMU trace log */
	if (IS_ERR_OR_NULL(gmu->trace.md)) {
		gmu->trace.md = gen7_reserve_gmu_kernel_block(gmu, 0,
					GMU_TRACE_SIZE, GMU_NONCACHED_KERNEL, 0);

		if (IS_ERR(gmu->trace.md))
			return PTR_ERR(gmu->trace.md);

		/* Pass trace buffer address to GMU through the VRB */
		ret = gmu_core_set_vrb_register(gmu->vrb, VRB_TRACE_BUFFER_ADDR_IDX,
					gmu->trace.md->gmuaddr);
		if (ret)
			return ret;

		/* Initialize the GMU trace buffer header */
		gmu_core_trace_header_init(&gmu->trace);
	}

	return 0;
}

static int gen7_hwsched_gmu_init(struct adreno_device *adreno_dev)
{
	int ret;

	ret = gen7_gmu_parse_fw(adreno_dev);
	if (ret)
		return ret;

	ret = gen7_gmu_memory_init(adreno_dev);
	if (ret)
		return ret;

	ret = gen7_gmu_warmboot_init(adreno_dev);
	if (ret)
		return ret;

	ret = gen7_hwsched_gmu_memory_init(adreno_dev);
	if (ret)
		return ret;

	return gen7_hwsched_hfi_init(adreno_dev);
}

static void gen7_hwsched_touch_wakeup(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret;

	/*
	 * Do not wake up a suspended device or until the first boot sequence
	 * has been completed.
	 */
	if (test_bit(GMU_PRIV_PM_SUSPEND, &gmu->flags) ||
		!test_bit(GMU_PRIV_FIRST_BOOT_DONE, &gmu->flags))
		return;

	if (test_bit(GMU_PRIV_GPU_STARTED, &gmu->flags))
		goto done;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_ACTIVE);

	ret = gen7_hwsched_gmu_boot(adreno_dev);
	if (ret)
		return;

	ret = gen7_hwsched_gpu_boot(adreno_dev);
	if (ret)
		return;

	kgsl_pwrscale_wake(device);

	set_bit(GMU_PRIV_GPU_STARTED, &gmu->flags);

	device->pwrctrl.last_stat_updated = ktime_get();

	kgsl_pwrctrl_set_state(device, KGSL_STATE_ACTIVE);

done:
	/*
	 * When waking up from a touch event we want to stay active long enough
	 * for the user to send a draw command. The default idle timer timeout
	 * is shorter than we want so go ahead and push the idle timer out
	 * further for this special case
	 */
	mod_timer(&device->idle_timer, jiffies +
		msecs_to_jiffies(adreno_wake_timeout));
}

static int gen7_hwsched_boot(struct adreno_device *adreno_dev)
{
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	bool bcl_state = adreno_dev->bcl_enabled;
	int ret;

	if (test_bit(GMU_PRIV_GPU_STARTED, &gmu->flags))
		return 0;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_ACTIVE);

	adreno_hwsched_start(adreno_dev);

	if (IS_ENABLED(CONFIG_QCOM_KGSL_HIBERNATION) &&
		!test_bit(GMU_PRIV_PDC_RSC_LOADED, &gmu->flags)) {
		/*
		 * During hibernation entry ZAP was unloaded and
		 * CBCAST BCL register is in reset state.
		 * Set bcl_enabled to false to skip KMD's HFI request
		 * to GMU for BCL feature, send BCL feature request to
		 * GMU after ZAP load at GPU boot. This ensures that
		 * Central Broadcast register was programmed before
		 * enabling BCL.
		 */
		adreno_dev->bcl_enabled = false;
		ret = gen7_hwsched_gmu_first_boot(adreno_dev);
	} else {
		ret = gen7_hwsched_gmu_boot(adreno_dev);
	}

	if (ret)
		return ret;

	ret = gen7_hwsched_gpu_boot(adreno_dev);
	if (ret)
		return ret;

	kgsl_start_idle_timer(device);
	kgsl_pwrscale_wake(device);

	set_bit(GMU_PRIV_GPU_STARTED, &gmu->flags);

	if (IS_ENABLED(CONFIG_QCOM_KGSL_HIBERNATION))
		adreno_dev->bcl_enabled = bcl_state;

	device->pwrctrl.last_stat_updated = ktime_get();

	kgsl_pwrctrl_set_state(device, KGSL_STATE_ACTIVE);

	return ret;
}

static int gen7_aqe_microcode_read(struct adreno_device *adreno_dev)
{
	struct adreno_firmware *aqe_fw = ADRENO_FW(adreno_dev, ADRENO_FW_AQE);
	const struct adreno_gen7_core *gen7_core = to_gen7_core(adreno_dev);

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_AQE))
		return 0;

	return adreno_get_firmware(adreno_dev, gen7_core->aqefw_name, aqe_fw);
}

static int gen7_hwsched_first_boot(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret;

	if (test_bit(GMU_PRIV_FIRST_BOOT_DONE, &gmu->flags))
		return gen7_hwsched_boot(adreno_dev);

	adreno_hwsched_start(adreno_dev);

	ret = gen7_microcode_read(adreno_dev);
	if (ret)
		return ret;

	ret = gen7_aqe_microcode_read(adreno_dev);
	if (ret)
		return ret;

	ret = gen7_init(adreno_dev);
	if (ret)
		return ret;

	ret = gen7_hwsched_gmu_init(adreno_dev);
	if (ret)
		return ret;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_ACTIVE);

	ret = gen7_hwsched_gmu_first_boot(adreno_dev);
	if (ret)
		return ret;

	ret = gen7_hwsched_gpu_boot(adreno_dev);
	if (ret)
		return ret;

	adreno_get_bus_counters(adreno_dev);

	adreno_dev->cooperative_reset = ADRENO_FEATURE(adreno_dev,
						 ADRENO_COOP_RESET);

	set_bit(GMU_PRIV_FIRST_BOOT_DONE, &gmu->flags);
	set_bit(GMU_PRIV_GPU_STARTED, &gmu->flags);

	/*
	 * BCL needs respective Central Broadcast register to
	 * be programed from TZ. This programing happens only
	 * when zap shader firmware load is successful. Zap firmware
	 * load can fail in boot up path hence enable BCL only after we
	 * successfully complete first boot to ensure that Central
	 * Broadcast register was programed before enabling BCL.
	 */
	if (ADRENO_FEATURE(adreno_dev, ADRENO_BCL))
		adreno_dev->bcl_enabled = true;

	/*
	 * There is a possible deadlock scenario during kgsl firmware reading
	 * (request_firmware) and devfreq update calls. During first boot, kgsl
	 * device mutex is held and then request_firmware is called for reading
	 * firmware. request_firmware internally takes dev_pm_qos_mtx lock.
	 * Whereas in case of devfreq update calls triggered by thermal/bcl or
	 * devfreq sysfs, it first takes the same dev_pm_qos_mtx lock and then
	 * tries to take kgsl device mutex as part of get_dev_status/target
	 * calls. This results in deadlock when both thread are unable to acquire
	 * the mutex held by other thread. Enable devfreq updates now as we are
	 * done reading all firmware files.
	 */
	device->pwrscale.devfreq_enabled = true;

	device->pwrctrl.last_stat_updated = ktime_get();

	kgsl_pwrctrl_set_state(device, KGSL_STATE_ACTIVE);

	return 0;
}

/**
 * drain_ctx_hw_fences_cpu - Force trigger the hardware fences that
 * were not sent to TxQueue by the GMU
 */
static void drain_ctx_hw_fences_cpu(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct adreno_hw_fence_entry *entry, *tmp;

	spin_lock(&drawctxt->lock);
	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_inflight_list, node) {
		kgsl_hw_fence_trigger_cpu(KGSL_DEVICE(adreno_dev), entry->kfence);
		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}
	spin_unlock(&drawctxt->lock);
}

static void drain_hw_fences_cpu(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_context *context;
	int id;

	read_lock(&device->context_lock);
	idr_for_each_entry(&device->context_idr, context, id) {
		if (context->gmu_registered)
			drain_ctx_hw_fences_cpu(adreno_dev, ADRENO_CONTEXT(context));
	}
	read_unlock(&device->context_lock);
}

/**
 * check_inflight_hw_fences - During SLUMBER entry, we must make sure all hardware fences across
 * all registered contexts have been sent to TxQueue. If not, take a snapshot
 */
static int check_inflight_hw_fences(struct adreno_device *adreno_dev)
{
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_context *context;
	int id, ret = 0;

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &hwsched->flags))
		return 0;

	read_lock(&device->context_lock);
	idr_for_each_entry(&device->context_idr, context, id) {

		if (context->gmu_registered) {
			ret = gen7_hwsched_check_context_inflight_hw_fences(adreno_dev,
				ADRENO_CONTEXT(context));
			if (ret)
				break;
		}
	}
	read_unlock(&device->context_lock);

	if (ret)
		gmu_core_fault_snapshot(device, GMU_FAULT_HW_FENCE);

	return ret;
}

static int gen7_hwsched_power_off(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret = 0;
	bool drain_cpu = false;

	if (!test_bit(GMU_PRIV_GPU_STARTED, &gmu->flags))
		return 0;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_SLUMBER);

	ret = gen7_gmu_oob_set(device, oob_gpu);
	if (ret) {
		gen7_gmu_oob_clear(device, oob_gpu);
		goto no_gx_power;
	}

	kgsl_pwrscale_update_stats(device);

	/* Save active coresight registers if applicable */
	adreno_coresight_stop(adreno_dev);

	adreno_irqctrl(adreno_dev, 0);

	gen7_gmu_oob_clear(device, oob_gpu);

no_gx_power:
	kgsl_pwrctrl_irq(device, false);

	/* Make sure GMU has sent all hardware fences to TxQueue */
	if (check_inflight_hw_fences(adreno_dev))
		drain_cpu = true;

	gen7_hwsched_gmu_power_off(adreno_dev);

	/* Now that we are sure that GMU is powered off, drain pending fences */
	if (drain_cpu)
		drain_hw_fences_cpu(adreno_dev);

	adreno_hwsched_unregister_contexts(adreno_dev);

	adreno_llcc_slice_deactivate(adreno_dev);

	clear_bit(GMU_PRIV_GPU_STARTED, &gmu->flags);

	del_timer_sync(&device->idle_timer);

	kgsl_pwrscale_sleep(device);

	kgsl_pwrctrl_clear_l3_vote(device);

	kgsl_pwrctrl_set_state(device, KGSL_STATE_SLUMBER);

	return ret;
}

static void check_hw_fence_unack_count(struct adreno_device *adreno_dev)
{
	struct gen7_hwsched_hfi *hfi = to_gen7_hwsched_hfi(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 unack_count;

	if (!test_bit(ADRENO_HWSCHED_HW_FENCE, &adreno_dev->hwsched.flags))
		return;

	gen7_hwsched_process_msgq(adreno_dev);

	spin_lock(&hfi->hw_fence.lock);
	unack_count = hfi->hw_fence.unack_count;
	spin_unlock(&hfi->hw_fence.lock);

	if (!unack_count)
		return;

	dev_err(GMU_PDEV_DEV(device),
		"hardware fence unack_count(%d) isn't zero before SLUMBER\n",
		unack_count);
	gmu_core_fault_snapshot(device, GMU_FAULT_HW_FENCE);
}

static void hwsched_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work,
					struct kgsl_device, idle_check_ws);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	mutex_lock(&device->mutex);

	if (test_bit(GMU_DISABLE_SLUMBER, &device->gmu_core.flags))
		goto done;

	if (atomic_read(&device->active_cnt) || time_is_after_jiffies(device->idle_jiffies)) {
		kgsl_pwrscale_update(device);
		kgsl_start_idle_timer(device);
		goto done;
	}

	spin_lock(&device->submit_lock);
	if (device->submit_now) {
		spin_unlock(&device->submit_lock);
		kgsl_pwrscale_update(device);
		kgsl_start_idle_timer(device);
		goto done;
	}

	device->skip_inline_submit = true;
	spin_unlock(&device->submit_lock);

	if (!gen7_hw_isidle(adreno_dev)) {
		dev_err(device->dev, "GPU isn't idle before SLUMBER\n");
		gmu_core_fault_snapshot(device, GMU_FAULT_PANIC_NONE);
	}

	check_hw_fence_unack_count(adreno_dev);

	gen7_hwsched_power_off(adreno_dev);

done:
	mutex_unlock(&device->mutex);
}

static int gen7_hwsched_first_open(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	/*
	 * Do the one time settings that need to happen when we
	 * attempt to boot the gpu the very first time
	 */
	ret = gen7_hwsched_first_boot(adreno_dev);
	if (ret)
		return ret;

	/*
	 * A client that does a first_open but never closes the device
	 * may prevent us from going back to SLUMBER. So trigger the idle
	 * check by incrementing the active count and immediately releasing it.
	 */
	atomic_inc(&device->active_cnt);
	adreno_active_count_put(adreno_dev);

	return 0;
}

int gen7_hwsched_active_count_get(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret = 0;

	if (WARN_ON(!mutex_is_locked(&device->mutex)))
		return -EINVAL;

	if (test_bit(GMU_PRIV_PM_SUSPEND, &gmu->flags))
		return -EINVAL;

	if ((atomic_read(&device->active_cnt) == 0))
		ret = gen7_hwsched_boot(adreno_dev);

	if (ret == 0)
		atomic_inc(&device->active_cnt);

	trace_kgsl_active_count(device,
		(unsigned long) __builtin_return_address(0));

	return ret;
}

static int gen7_hwsched_dcvs_set(struct adreno_device *adreno_dev,
		int gpu_pwrlevel, int bus_level, u32 ab)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	struct gen7_dcvs_table *table = &gmu->dcvs_table;
	struct hfi_gx_bw_perf_vote_cmd req = {
		.ack_type = DCVS_ACK_BLOCK,
		.freq = INVALID_DCVS_IDX,
		.bw = INVALID_DCVS_IDX,
	};
	int ret;

	if (!test_bit(GMU_PRIV_HFI_STARTED, &gmu->flags))
		return 0;

	/* Do not set to XO and lower GPU clock vote from GMU */
	if ((gpu_pwrlevel != INVALID_DCVS_IDX) &&
			(gpu_pwrlevel >= table->gpu_level_num - 1)) {
		dev_err(GMU_PDEV_DEV(device), "Invalid gpu dcvs request: %d\n",
			gpu_pwrlevel);
		return -EINVAL;
	}

	if (gpu_pwrlevel < table->gpu_level_num - 1)
		req.freq = table->gpu_level_num - gpu_pwrlevel - 1;

	if (bus_level < pwr->ddr_table_count && bus_level > 0)
		req.bw = bus_level;

	req.bw |= adreno_gmu_bus_ab_quantize(adreno_dev, ab);

	/* GMU will vote for slumber levels through the sleep sequence */
	if ((req.freq == INVALID_DCVS_IDX) && (req.bw == INVALID_BW_VOTE))
		return 0;

	ret = CMD_MSG_HDR(req, H2F_MSG_GX_BW_PERF_VOTE);
	if (ret)
		return ret;

	ret = gen7_hfi_send_cmd_async(adreno_dev, &req, sizeof(req));

	if (ret) {
		dev_err_ratelimited(GMU_PDEV_DEV(device),
			"Failed to set GPU perf idx %u, bw idx %u\n",
			req.freq, req.bw);

		/*
		 * If this was a dcvs request along side an active gpu, request
		 * dispatcher based reset and recovery.
		 */
		if (test_bit(GMU_PRIV_GPU_STARTED, &gmu->flags))
			gen7_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
	}

	if (req.freq != INVALID_DCVS_IDX)
		gen7_rdpm_mx_freq_update(gmu, gmu->dcvs_table.gx_votes[req.freq].freq);

	return ret;
}

static int gen7_hwsched_clock_set(struct adreno_device *adreno_dev,
	u32 pwrlevel)
{
	return gen7_hwsched_dcvs_set(adreno_dev, pwrlevel, INVALID_DCVS_IDX, INVALID_AB_VALUE);
}

static void scale_gmu_frequency(struct adreno_device *adreno_dev, int buslevel)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	u32 cur_freq = gmu->cur_freq;
	u32 req_freq = gmu->freqs[0];

	if (!gmu->perf_ddr_bw)
		return;

	/*
	 * Scale the GMU if DDR is at a CX corner at which GMU can run at
	 * a higher frequency
	 */
	if (pwr->ddr_table[buslevel] >= gmu->perf_ddr_bw)
		req_freq = gmu->freqs[GMU_MAX_PWRLEVELS - 1];

	if (cur_freq == req_freq)
		return;

	gen7_gmu_clock_set_rate(adreno_dev, req_freq);
}

static int gen7_hwsched_bus_set(struct adreno_device *adreno_dev, int buslevel,
	u32 ab)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	int ret = 0;

	/* Target gen7_9_x votes for perfmode through ACV. Skip icc path for same */
	if (!adreno_is_gen7_9_x(adreno_dev))
		kgsl_icc_set_tag(pwr, buslevel);

	if (buslevel == pwr->cur_buslevel)
		buslevel = INVALID_DCVS_IDX;

	if ((ab == pwr->cur_ab) || ((ab == 0) && (adreno_dev->gmu_ab)))
		ab = INVALID_AB_VALUE;

	if ((ab == INVALID_AB_VALUE) && (buslevel == INVALID_DCVS_IDX))
		return 0;

	ret = gen7_hwsched_dcvs_set(adreno_dev, INVALID_DCVS_IDX,
			buslevel, ab);
	if (ret)
		return ret;

	if (buslevel != INVALID_DCVS_IDX) {
		scale_gmu_frequency(adreno_dev, buslevel);

		pwr->cur_buslevel = buslevel;
	}

	if (ab != INVALID_AB_VALUE) {
		if (!adreno_dev->gmu_ab)
			icc_set_bw(pwr->icc_path, MBps_to_icc(ab), 0);
		pwr->cur_ab = ab;
	}

	trace_kgsl_buslevel(device, pwr->active_pwrlevel, pwr->cur_buslevel, pwr->cur_ab);
	return ret;
}

static int gen7_hwsched_pm_suspend(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	int ret;

	if (test_bit(GMU_PRIV_PM_SUSPEND, &gmu->flags))
		return 0;

	kgsl_pwrctrl_request_state(device, KGSL_STATE_SUSPEND);

	/* Halt any new submissions */
	reinit_completion(&device->halt_gate);

	/*
	 * Wait for the dispatcher to retire everything by waiting
	 * for the active count to go to zero.
	 */
	ret = kgsl_active_count_wait(device, 0, msecs_to_jiffies(100));
	if (ret) {
		dev_err(device->dev, "Timed out waiting for the active count\n");
		goto err;
	}

	ret = adreno_hwsched_idle(adreno_dev);
	if (ret)
		goto err;

	gen7_hwsched_power_off(adreno_dev);

	adreno_get_gpu_halt(adreno_dev);

	set_bit(GMU_PRIV_PM_SUSPEND, &gmu->flags);

	kgsl_pwrctrl_set_state(device, KGSL_STATE_SUSPEND);

	return 0;

err:
	adreno_hwsched_start(adreno_dev);

	return ret;
}

static void gen7_hwsched_pm_resume(struct adreno_device *adreno_dev)
{
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);

	if (WARN(!test_bit(GMU_PRIV_PM_SUSPEND, &gmu->flags),
		"resume invoked without a suspend\n"))
		return;

	adreno_put_gpu_halt(adreno_dev);

	adreno_hwsched_start(adreno_dev);

	clear_bit(GMU_PRIV_PM_SUSPEND, &gmu->flags);
}

void gen7_hwsched_handle_watchdog(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 mask;

	/* Temporarily mask the watchdog interrupt to prevent a storm */
	gmu_core_regread(device, GEN7_GMU_AO_HOST_INTERRUPT_MASK,
		&mask);
	gmu_core_regwrite(device, GEN7_GMU_AO_HOST_INTERRUPT_MASK,
			(mask | GMU_INT_WDOG_BITE));

	gen7_gmu_send_nmi(device, false, GMU_FAULT_PANIC_NONE);

	dev_err_ratelimited(GMU_PDEV_DEV(device),
			"GMU watchdog expired interrupt received\n");

	gen7_hwsched_fault(adreno_dev, ADRENO_GMU_FAULT);
}

static void gen7_hwsched_drain_ctxt_unregister(struct adreno_device *adreno_dev)
{
	struct gen7_hwsched_hfi *hfi = to_gen7_hwsched_hfi(adreno_dev);
	struct pending_cmd *cmd = NULL;

	read_lock(&hfi->msglock);

	list_for_each_entry(cmd, &hfi->msglist, node) {
		if (MSG_HDR_GET_ID(cmd->sent_hdr) == H2F_MSG_UNREGISTER_CONTEXT)
			complete(&cmd->complete);
	}

	read_unlock(&hfi->msglock);
}

/**
 * process_context_hw_fences_after_reset - This function processes all hardware fences that were
 * sent to GMU prior to recovery. If a fence is not retired by the GPU, and the context is still
 * good, then move them to the reset list.
 */
static void process_context_hw_fences_after_reset(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt, struct list_head *reset_list)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_hw_fence_entry *entry, *tmp;

	spin_lock(&drawctxt->lock);
	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_inflight_list, node) {
		struct adreno_context *drawctxt = entry->drawctxt;
		struct gmu_context_queue_header *hdr = drawctxt->gmu_context_queue.hostptr;
		bool retired = kgsl_check_timestamp(device, &drawctxt->base, (u32)entry->cmd.ts);

		/* Delete the fences that GMU has sent to the TxQueue */
		if (timestamp_cmp(hdr->out_fence_ts, (u32)entry->cmd.ts) >= 0) {
			adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
			continue;
		}

		/*
		 * Force retire the fences if the corresponding submission is retired by GPU
		 * or if the context has gone bad
		 */
		if (retired || kgsl_context_is_bad(&drawctxt->base))
			entry->cmd.flags |= HW_FENCE_FLAG_SKIP_MEMSTORE;

		list_add_tail(&entry->reset_node, reset_list);
	}
	spin_unlock(&drawctxt->lock);
}

/**
 * process_inflight_hw_fences_after_reset - Send hardware fences from all contexts back to the GMU
 * after fault recovery. We must wait for ack when sending each of these fences to GMU so as to
 * avoid sending a large number of hardware fences in a short span of time.
 */
static int process_inflight_hw_fences_after_reset(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_context *context = NULL;
	int id, ret = 0;
	struct list_head hw_fence_list;
	struct adreno_hw_fence_entry *entry, *tmp;

	/*
	 * Since we need to wait for ack from GMU when sending each inflight fence back to GMU, we
	 * cannot send them from within atomic context. Hence, walk list of such hardware fences
	 * for each context and add it to this local list and then walk this list to send all these
	 * fences to GMU.
	 */
	INIT_LIST_HEAD(&hw_fence_list);

	read_lock(&device->context_lock);
	idr_for_each_entry(&device->context_idr, context, id) {
		process_context_hw_fences_after_reset(adreno_dev, ADRENO_CONTEXT(context),
			&hw_fence_list);
	}
	read_unlock(&device->context_lock);

	list_for_each_entry_safe(entry, tmp, &hw_fence_list, reset_node) {

		/*
		 * This is part of the reset sequence and any error in this path will be handled by
		 * the caller.
		 */
		ret = gen7_send_hw_fence_hfi_wait_ack(adreno_dev, entry, 0);
		if (ret)
			break;

		list_del_init(&entry->reset_node);
	}

	return ret;
}

/**
 * process_detached_hw_fences_after_reset - Send fences that couldn't be sent to GMU when a context
 * got detached. We must wait for ack when sending each of these fences to GMU so as to avoid
 * sending a large number of hardware fences in a short span of time.
 */
static int process_detached_hw_fences_after_reset(struct adreno_device *adreno_dev)
{
	struct adreno_hw_fence_entry *entry, *tmp;
	struct gen7_hwsched_hfi *hfi = to_gen7_hwsched_hfi(adreno_dev);
	struct kgsl_context *context = NULL;
	int ret = 0;

	list_for_each_entry_safe(entry, tmp, &hfi->detached_hw_fence_list, node) {

		/*
		 * This is part of the reset sequence and any error in this path will be handled by
		 * the caller.
		 */
		ret = gen7_send_hw_fence_hfi_wait_ack(adreno_dev, entry,
			HW_FENCE_FLAG_SKIP_MEMSTORE);
		if (ret)
			return ret;

		context = &entry->drawctxt->base;

		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);

		kgsl_context_put(context);
	}

	return ret;
}

static int gen7_hwsched_drain_context_hw_fences(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	struct adreno_hw_fence_entry *entry, *tmp;
	int ret = 0;

	/* We don't need the drawctxt lock here as this context has already been invalidated */
	list_for_each_entry_safe(entry, tmp, &drawctxt->hw_fence_list, node) {

		/* Any error here is fatal */
		ret = gen7_send_hw_fence_hfi_wait_ack(adreno_dev, entry,
			HW_FENCE_FLAG_SKIP_MEMSTORE);
		if (ret)
			break;

		adreno_hwsched_remove_hw_fence_entry(adreno_dev, entry);
	}

	return ret;
}

static struct adreno_context *_get_guilty_context(struct kgsl_device *device)
{
	struct kgsl_context *context = NULL;
	struct adreno_context *guilty = NULL;
	int id;

	read_lock(&device->context_lock);
	idr_for_each_entry(&device->context_idr, context, id) {
		if (test_and_clear_bit(KGSL_CONTEXT_PRIV_INVALID_DRAIN_HW_FENCE, &context->priv) &&
			_kgsl_context_get(context)) {
			guilty = ADRENO_CONTEXT(context);
			break;
		}
	}
	read_unlock(&device->context_lock);

	return guilty;
}

static int drain_guilty_context_hw_fences(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct adreno_context *guilty = _get_guilty_context(device);
	int ret = 0;

	/*
	 * We don't need drawctxt spinlock to signal these fences since the only other place
	 * which can access these fences is the context detach path and device mutex
	 * ensures mutual exclusion between recovery thread and detach thread.
	 */
	while (guilty) {
		ret = gen7_hwsched_drain_context_hw_fences(adreno_dev, guilty);

		kgsl_context_put(&guilty->base);

		if (ret)
			break;

		guilty = _get_guilty_context(device);
	}

	return ret;
}

static int handle_hw_fences_after_reset(struct adreno_device *adreno_dev)
{
	int ret;

	ret = drain_guilty_context_hw_fences(adreno_dev);
	if (ret)
		return ret;

	/*
	 * We must do this after adreno_hwsched_replay() so that context registration
	 * is done before we re-send the un-retired hardware fences to the GMU
	 */
	ret = process_inflight_hw_fences_after_reset(adreno_dev);
	if (ret)
		return ret;

	ret = process_detached_hw_fences_after_reset(adreno_dev);
	if (ret)
		return ret;

	return gen7_hwsched_disable_hw_fence_throttle(adreno_dev);
}

int gen7_hwsched_reset_replay(struct adreno_device *adreno_dev)
{
	struct gen7_gmu_device *gmu = to_gen7_gmu(adreno_dev);
	struct gen7_hwsched_hfi *hfi = to_gen7_hwsched_hfi(adreno_dev);
	int ret;

	/*
	 * Any pending context unregister packets will be lost
	 * since we hard reset the GMU. This means any threads waiting
	 * for context unregister hfi ack will timeout. Wake them
	 * to avoid false positive ack timeout messages later.
	 */
	gen7_hwsched_drain_ctxt_unregister(adreno_dev);

	if (!test_bit(GMU_PRIV_GPU_STARTED, &gmu->flags))
		return 0;

	gen7_disable_gpu_irq(adreno_dev);

	gen7_gmu_irq_disable(adreno_dev);

	gen7_hwsched_hfi_stop(adreno_dev);

	gen7_gmu_suspend(adreno_dev);

	adreno_hwsched_unregister_contexts(adreno_dev);

	adreno_llcc_slice_deactivate(adreno_dev);

	clear_bit(GMU_PRIV_GPU_STARTED, &gmu->flags);

	spin_lock(&hfi->hw_fence.lock);

	/* Reset the unack count back to zero as we start afresh */
	hfi->hw_fence.unack_count = 0;

	spin_unlock(&hfi->hw_fence.lock);

	/*
	 * When we reset, we want to coldboot incase any scratch corruption
	 * has occurred before we faulted.
	 */
	adreno_mark_for_coldboot(adreno_dev);

	ret = gen7_hwsched_boot(adreno_dev);
	if (ret)
		goto done;

	adreno_hwsched_replay(adreno_dev);

	ret = handle_hw_fences_after_reset(adreno_dev);
done:
	BUG_ON(ret);

	return ret;
}

const struct adreno_power_ops gen7_hwsched_power_ops = {
	.first_open = gen7_hwsched_first_open,
	.last_close = gen7_hwsched_power_off,
	.active_count_get = gen7_hwsched_active_count_get,
	.touch_wakeup = gen7_hwsched_touch_wakeup,
	.pm_suspend = gen7_hwsched_pm_suspend,
	.pm_resume = gen7_hwsched_pm_resume,
	.gpu_clock_set = gen7_hwsched_clock_set,
	.gpu_bus_set = gen7_hwsched_bus_set,
};

const struct adreno_hwsched_ops gen7_hwsched_ops = {
	.submit_drawobj = gen7_hwsched_submit_drawobj,
	.preempt_count = gen7_hwsched_preempt_count_get,
	.create_hw_fence = gen7_hwsched_create_hw_fence,
	.get_rb_hostptr = gen7_hwsched_get_rb_hostptr,
};

int gen7_hwsched_probe(struct platform_device *pdev,
		u32 chipid, const struct adreno_gpu_core *gpucore)
{
	struct adreno_device *adreno_dev;
	struct kgsl_device *device;
	struct gen7_hwsched_device *gen7_hwsched_dev;
	int ret;

	gen7_hwsched_dev = devm_kzalloc(&pdev->dev, sizeof(*gen7_hwsched_dev),
				GFP_KERNEL);
	if (!gen7_hwsched_dev)
		return -ENOMEM;

	adreno_dev = &gen7_hwsched_dev->gen7_dev.adreno_dev;

	adreno_dev->hwsched_enabled = true;

	adreno_dev->irq_mask = GEN7_HWSCHED_INT_MASK;

	ret = gen7_probe_common(pdev, adreno_dev, chipid, gpucore);
	if (ret)
		return ret;

	device = KGSL_DEVICE(adreno_dev);

	INIT_WORK(&device->idle_check_ws, hwsched_idle_check);

	timer_setup(&device->idle_timer, hwsched_idle_timer, 0);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_LPAC))
		adreno_dev->lpac_enabled = true;

	if (ADRENO_FEATURE(adreno_dev, ADRENO_DMS)) {
		set_bit(ADRENO_DEVICE_DMS, &adreno_dev->priv);
		adreno_dev->dms_enabled = true;
	}

	kgsl_mmu_set_feature(device, KGSL_MMU_PAGEFAULT_TERMINATE);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_HW_FENCE))
		device->max_syncobj_hw_fence_count = min_t(u32, HFI_SYNCOBJ_LEGACY_HW_FENCE_MAX,
			MAX_SYNCOBJ_QUERY_BITS);

	ret = adreno_hwsched_init(adreno_dev, &gen7_hwsched_ops);
	if (ret)
		dev_err(&pdev->dev, "adreno hardware scheduler init failed ret %d\n", ret);

	return ret;
}

int gen7_hwsched_add_to_minidump(struct adreno_device *adreno_dev)
{
	struct gen7_device *gen7_dev = container_of(adreno_dev,
					struct gen7_device, adreno_dev);
	struct gen7_hwsched_device *gen7_hwsched = container_of(gen7_dev,
					struct gen7_hwsched_device, gen7_dev);
	struct gen7_hwsched_hfi *hw_hfi = &gen7_hwsched->hwsched_hfi;
	int ret, i;

	ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev, KGSL_HWSCHED_DEVICE,
			(void *)(gen7_hwsched), sizeof(struct gen7_hwsched_device));
	if (ret)
		return ret;

	if (!IS_ERR_OR_NULL(gen7_dev->gmu.gmu_log)) {
		ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev,
					KGSL_GMU_LOG_ENTRY,
					gen7_dev->gmu.gmu_log->hostptr,
					gen7_dev->gmu.gmu_log->size);
		if (ret)
			return ret;
	}

	if (!IS_ERR_OR_NULL(gen7_dev->gmu.hfi.hfi_mem)) {
		ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev,
					KGSL_HFIMEM_ENTRY,
					gen7_dev->gmu.hfi.hfi_mem->hostptr,
					gen7_dev->gmu.hfi.hfi_mem->size);
		if (ret)
			return ret;
	}

	if (!IS_ERR_OR_NULL(gen7_dev->gmu.vrb)) {
		ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev,
					KGSL_GMU_VRB_ENTRY,
					gen7_dev->gmu.vrb->hostptr,
					gen7_dev->gmu.vrb->size);
		if (ret)
			return ret;
	}

	if (!IS_ERR_OR_NULL(gen7_dev->gmu.trace.md)) {
		ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev,
					KGSL_GMU_TRACE_ENTRY,
					gen7_dev->gmu.trace.md->hostptr,
					gen7_dev->gmu.trace.md->size);
		if (ret)
			return ret;
	}

	/* Dump HFI hwsched global mem alloc entries */
	for (i = 0; i < hw_hfi->mem_alloc_entries; i++) {
		struct hfi_mem_alloc_entry *entry = &hw_hfi->mem_alloc_table[i];
		char hfi_minidump_str[MAX_VA_MINIDUMP_STR_LEN] = {0};
		u32 rb_id = 0;

		if (!hfi_get_minidump_string(entry->desc.mem_kind,
					     &hfi_minidump_str[0],
					     sizeof(hfi_minidump_str), &rb_id)) {
			ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev,
						      hfi_minidump_str,
						      entry->md->hostptr,
						      entry->md->size);
			if (ret)
				return ret;
		}
	}

	if (!IS_ERR_OR_NULL(hw_hfi->big_ib)) {
		ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev,
					      KGSL_HFI_BIG_IB_ENTRY,
					      hw_hfi->big_ib->hostptr,
					      hw_hfi->big_ib->size);
		if (ret)
			return ret;
	}

	if (!IS_ERR_OR_NULL(hw_hfi->big_ib_recurring))
		ret = kgsl_add_va_to_minidump(adreno_dev->dev.dev,
					      KGSL_HFI_BIG_IB_REC_ENTRY,
					      hw_hfi->big_ib_recurring->hostptr,
					      hw_hfi->big_ib_recurring->size);

	return ret;
}
