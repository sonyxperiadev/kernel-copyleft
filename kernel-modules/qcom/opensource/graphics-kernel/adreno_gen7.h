/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _ADRENO_GEN7_H_
#define _ADRENO_GEN7_H_

#include <linux/delay.h>

#include "gen7_reg.h"
#include "adreno_gen7_gmu.h"

/* Forward struct declaration */
struct gen7_snapshot_block_list;

extern const struct adreno_power_ops gen7_gmu_power_ops;
extern const struct adreno_power_ops gen7_hwsched_power_ops;
extern const struct adreno_perfcounters adreno_gen7_perfcounters;
extern const struct adreno_perfcounters adreno_gen7_hwsched_perfcounters;
extern const struct adreno_perfcounters adreno_gen7_no_cb_perfcounters;
extern const struct adreno_perfcounters adreno_gen7_9_0_hwsched_perfcounters;

struct gen7_gpudev {
	struct adreno_gpudev base;
	int (*hfi_probe)(struct adreno_device *adreno_dev);
	void (*hfi_remove)(struct adreno_device *adreno_dev);
	void (*handle_watchdog)(struct adreno_device *adreno_dev);
};

extern const struct gen7_gpudev adreno_gen7_gmu_gpudev;
extern const struct gen7_gpudev adreno_gen7_hwsched_gpudev;
extern const struct gen7_gpudev adreno_gen7_9_0_hwsched_gpudev;

/**
 * struct gen7_device - Container for the gen7_device
 */
struct gen7_device {
	/** @gmu: Container for the gen7 GMU device */
	struct gen7_gmu_device gmu;
	/** @adreno_dev: Container for the generic adreno device */
	struct adreno_device adreno_dev;
	/** @aperture: The last value that the host aperture register was programmed to */
	u32 aperture;
	/** @ext_pwrup_list_len: External pwrup reglist length */
	u16 ext_pwrup_list_len;
};

/**
 * struct gen7_pwrup_extlist - container for powerup external reglist
 */
struct gen7_pwrup_extlist {
	/** offset: Dword offset of the register to write */
	u32 offset;
	/** pipelines: pipelines to write */
	u32 pipelines;
};

/**
 * struct gen7_protected_regs - container for a protect register span
 */
struct gen7_protected_regs {
	/** @reg: Physical protected mode register to write to */
	u32 reg;
	/** @start: Dword offset of the starting register in the range */
	u32 start;
	/**
	 * @end: Dword offset of the ending register in the range
	 * (inclusive)
	 */
	u32 end;
	/**
	 * @noaccess: 1 if the register should not be accessible from
	 * userspace, 0 if it can be read (but not written)
	 */
	u32 noaccess;
};

/**
 * struct adreno_gen7_core - gen7 specific GPU core definitions
 */
struct adreno_gen7_core {
	/** @base: Container for the generic GPU definitions */
	struct adreno_gpu_core base;
	/** @gmu_fw_version: Minimum firmware version required to support this core */
	u32 gmu_fw_version;
	/** @sqefw_name: Name of the SQE microcode file */
	const char *sqefw_name;
	/** @aqefw_name: Name of the AQE microcode file */
	const char *aqefw_name;
	/** @gmufw_name: Name of the GMU firmware file */
	const char *gmufw_name;
	/** @gmufw_name: Name of the backup GMU firmware file */
	const char *gmufw_bak_name;
	/** @zap_name: Name of the CPZ zap file */
	const char *zap_name;
	/** @hwcg: List of registers and values to write for HWCG */
	const struct kgsl_regmap_list *hwcg;
	/** @hwcg_count: Number of registers in @hwcg */
	u32 hwcg_count;
	/** @ao_hwcg: List of registers and values to write for HWCG in AO block */
	const struct kgsl_regmap_list *ao_hwcg;
	/** @ao_hwcg_count: Number of registers in @ao_hwcg */
	u32 ao_hwcg_count;
	/** @gbif: List of registers and values to write for GBIF */
	const struct kgsl_regmap_list *gbif;
	/** @gbif_count: Number of registers in @gbif */
	u32 gbif_count;
	/** @hang_detect_cycles: Hang detect counter timeout value */
	u32 hang_detect_cycles;
	/** @protected_regs: Array of protected registers for the target */
	const struct gen7_protected_regs *protected_regs;
	/** @ctxt_record_size: Size of the preemption record in bytes */
	u64 ctxt_record_size;
	/** @highest_bank_bit: Highest bank bit value */
	u32 highest_bank_bit;
	/** @gen7_snapshot_block_list: Device-specific blocks dumped in the snapshot */
	const struct gen7_snapshot_block_list *gen7_snapshot_block_list;
	/** @gmu_hub_clk_freq: Gmu hub interface clock frequency */
	u64 gmu_hub_clk_freq;
	/**
	 * @bcl_data: bit 0 contains response type for bcl alarms and bits 1:21 controls sid vals
	 * to configure throttle levels for bcl alarm levels 0-2. If sid vals are not set,
	 * gmu fw sets default throttle levels.
	 */
	u32 bcl_data;
	/** @preempt_level: Preemption level valid ranges [0 to 2] */
	u32 preempt_level;
	/** @qos_value: GPU qos value to set for each RB. */
	const u32 *qos_value;
	/**
	 * @acv_perfmode_ddr_freq: Vote perfmode when DDR frequency >= acv_perfmode_ddr_freq.
	 * If not specified, vote perfmode for highest DDR level only.
	 */
	u32 acv_perfmode_ddr_freq;
	/** @acv_perfmode_vote: ACV vote for GPU perfmode */
	u32 acv_perfmode_vote;
	/** @rt_bus_hint: IB level hint for real time clients i.e. RB-0 */
	const u32 rt_bus_hint;
	/** @fast_bus_hint: Whether or not to increase IB vote on high ddr stall */
	bool fast_bus_hint;
	/** @noc_timeout_us: GPU config NOC port timeout in usec */
	u32 noc_timeout_us;
};

/**
 * struct gen7_cp_preemption_record - CP context record for
 * preemption.
 * @magic: (00) Value at this offset must be equal to
 * GEN7_CP_CTXRECORD_MAGIC_REF.
 * @info: (04) Type of record. Written non-zero (usually) by CP.
 * we must set to zero for all ringbuffers.
 * @errno: (08) Error code. Initialize this to GEN7_CP_CTXRECORD_ERROR_NONE.
 * CP will update to another value if a preemption error occurs.
 * @data: (12) DATA field in YIELD and SET_MARKER packets.
 * Written by CP when switching out. Not used on switch-in. Initialized to 0.
 * @cntl: (16) RB_CNTL, saved and restored by CP. We must initialize this.
 * @rptr: (20) RB_RPTR, saved and restored by CP. We must initialize this.
 * @wptr: (24) RB_WPTR, saved and restored by CP. We must initialize this.
 * @_pad28: (28) Reserved/padding.
 * @rptr_addr: (32) RB_RPTR_ADDR_LO|HI saved and restored. We must initialize.
 * rbase: (40) RB_BASE_LO|HI saved and restored.
 * counter: (48) Pointer to preemption counter.
 * @bv_rptr_addr: (56) BV_RB_RPTR_ADDR_LO|HI save and restored. We must initialize.
 */
struct gen7_cp_preemption_record {
	u32 magic;
	u32 info;
	u32 errno;
	u32 data;
	u32 cntl;
	u32 rptr;
	u32 wptr;
	u32 _pad28;
	u64 rptr_addr;
	u64 rbase;
	u64 counter;
	u64 bv_rptr_addr;
};

/**
 * struct gen7_cp_smmu_info - CP preemption SMMU info.
 * @magic: (00) The value at this offset must be equal to
 * GEN7_CP_SMMU_INFO_MAGIC_REF
 * @_pad4: (04) Reserved/padding
 * @ttbr0: (08) Base address of the page table for the * incoming context
 * @asid: (16) Address Space IDentifier (ASID) of the incoming context
 * @context_idr: (20) Context Identification Register value
 * @context_bank: (24) Which Context Bank in SMMU to update
 */
struct gen7_cp_smmu_info {
	u32 magic;
	u32 _pad4;
	u64 ttbr0;
	u32 asid;
	u32 context_idr;
	u32 context_bank;
};

#define GEN7_CP_SMMU_INFO_MAGIC_REF		0x241350d5UL

#define GEN7_CP_CTXRECORD_MAGIC_REF		0xae399d6eUL
/* Size of each CP preemption record */
#define GEN7_CP_CTXRECORD_SIZE_IN_BYTES		(4192 * 1024)
/* Size of the performance counter save/restore block (in bytes) */
#define GEN7_CP_PERFCOUNTER_SAVE_RESTORE_SIZE	(4 * 1024)

#define GEN7_CP_RB_CNTL_DEFAULT \
	(FIELD_PREP(GENMASK(7, 0), ilog2(KGSL_RB_DWORDS >> 1)) | \
	 FIELD_PREP(GENMASK(12, 8), ilog2(4)))

/* Size of the CP_INIT pm4 stream in dwords */
#define GEN7_CP_INIT_DWORDS 10

#define GEN7_INT_MASK \
	((1 << GEN7_INT_AHBERROR) |			\
	 (1 << GEN7_INT_ATBASYNCFIFOOVERFLOW) |		\
	 (1 << GEN7_INT_GPCERROR) |			\
	 (1 << GEN7_INT_SWINTERRUPT) |			\
	 (1 << GEN7_INT_HWERROR) |			\
	 (1 << GEN7_INT_PM4CPINTERRUPT) |		\
	 (1 << GEN7_INT_RB_DONE_TS) |			\
	 (1 << GEN7_INT_CACHE_CLEAN_TS) |		\
	 (1 << GEN7_INT_ATBBUSOVERFLOW) |		\
	 (1 << GEN7_INT_HANGDETECTINTERRUPT) |		\
	 (1 << GEN7_INT_OUTOFBOUNDACCESS) |		\
	 (1 << GEN7_INT_UCHETRAPINTERRUPT) |		\
	 (1 << GEN7_INT_TSBWRITEERROR) |		\
	 (1 << GEN7_INT_SWFUSEVIOLATION))

#define GEN7_HWSCHED_INT_MASK \
	((1 << GEN7_INT_AHBERROR) |			\
	 (1 << GEN7_INT_ATBASYNCFIFOOVERFLOW) |		\
	 (1 << GEN7_INT_ATBBUSOVERFLOW) |		\
	 (1 << GEN7_INT_OUTOFBOUNDACCESS) |		\
	 (1 << GEN7_INT_UCHETRAPINTERRUPT) |		\
	 (1 << GEN7_INT_TSBWRITEERROR))

/**
 * to_gen7_core - return the gen7 specific GPU core struct
 * @adreno_dev: An Adreno GPU device handle
 *
 * Returns:
 * A pointer to the gen7 specific GPU core struct
 */
static inline const struct adreno_gen7_core *
to_gen7_core(struct adreno_device *adreno_dev)
{
	const struct adreno_gpu_core *core = adreno_dev->gpucore;

	return container_of(core, struct adreno_gen7_core, base);
}

/* Preemption functions */
void gen7_preemption_trigger(struct adreno_device *adreno_dev, bool atomic);
void gen7_preemption_schedule(struct adreno_device *adreno_dev);
void gen7_preemption_start(struct adreno_device *adreno_dev);
int gen7_preemption_init(struct adreno_device *adreno_dev);

u32 gen7_preemption_post_ibsubmit(struct adreno_device *adreno_dev,
		unsigned int *cmds);
u32 gen7_preemption_pre_ibsubmit(struct adreno_device *adreno_dev,
		struct adreno_ringbuffer *rb, struct adreno_context *drawctxt,
		u32 *cmds);

void gen7_preemption_callback(struct adreno_device *adreno_dev, int bit);

void gen7_preemption_context_destroy(struct kgsl_context *context);

void gen7_preemption_prepare_postamble(struct adreno_device *adreno_dev);

void gen7_snapshot(struct adreno_device *adreno_dev,
		struct kgsl_snapshot *snapshot);
void gen7_crashdump_init(struct adreno_device *adreno_dev);

/**
 * gen7_snapshot_external_core_regs - Dump external registers into snapshot
 * @device: Pointer to KGSL device
 * @snapshot: Pointer to the snapshot
 *
 * Dump external core registers like GPUCC, CPR into GPU snapshot.
 */
void gen7_snapshot_external_core_regs(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot);

/**
 * gen7_start - Program gen7 registers
 * @adreno_dev: An Adreno GPU handle
 *
 * This function does all gen7 register programming every
 * time we boot the gpu
 *
 * Return: 0 on success or negative on failure
 */
int gen7_start(struct adreno_device *adreno_dev);

/**
 * gen7_init - Initialize gen7 resources
 * @adreno_dev: An Adreno GPU handle
 *
 * This function does gen7 specific one time initialization
 * and is invoked when the very first client opens a
 * kgsl instance
 *
 * Return: Zero on success and negative error on failure
 */
int gen7_init(struct adreno_device *adreno_dev);

/**
 * gen7_cx_timer_init - Initialize the CX timer on Gen7 devices
 * @adreno_dev: Pointer to the adreno device
 *
 * Synchronize the GPU CX timer (if we have one) with the CPU timer
 */
void gen7_cx_timer_init(struct adreno_device *adreno_dev);

/**
 * gen7_get_gpu_feature_info - Get hardware supported feature info
 * @adreno_dev: Pointer to the adreno device
 *
 * Get HW supported feature info and update sofware feature configuration
 */
void gen7_get_gpu_feature_info(struct adreno_device *adreno_dev);

/**
 * gen7_rb_start - Gen7 specific ringbuffer setup
 * @adreno_dev: An Adreno GPU handle
 *
 * This function does gen7 specific ringbuffer setup and
 * attempts to submit CP INIT and bring GPU out of secure mode
 *
 * Return: Zero on success and negative error on failure
 */
int gen7_rb_start(struct adreno_device *adreno_dev);

/**
 * gen7_microcode_read - Get the cp microcode from the filesystem
 * @adreno_dev: An Adreno GPU handle
 *
 * This function gets the firmware from filesystem and sets up
 * the micorocode global buffer
 *
 * Return: Zero on success and negative error on failure
 */
int gen7_microcode_read(struct adreno_device *adreno_dev);

/**
 * gen7_probe_common - Probe common gen7 resources
 * @pdev: Pointer to the platform device
 * @adreno_dev: Pointer to the adreno device
 * @chipid: Chipid of the target
 * @gpucore: Pointer to the gpucore strucure
 *
 * This function sets up the gen7 resources common across all
 * gen7 targets
 */
int gen7_probe_common(struct platform_device *pdev,
	struct adreno_device *adreno_dev, u32 chipid,
	const struct adreno_gpu_core *gpucore);

/**
 * gen7_hw_isidle - Check whether gen7 gpu is idle or not
 * @adreno_dev: An Adreno GPU handle
 *
 * Return: True if gpu is idle, otherwise false
 */
bool gen7_hw_isidle(struct adreno_device *adreno_dev);

/**
 * gen7_spin_idle_debug - Debug logging used when gpu fails to idle
 * @adreno_dev: An Adreno GPU handle
 * @str: String describing the failure
 *
 * This function logs interesting registers and triggers a snapshot
 */
void gen7_spin_idle_debug(struct adreno_device *adreno_dev,
	const char *str);

/**
 * gen7_perfcounter_update - Update the IFPC perfcounter list
 * @adreno_dev: An Adreno GPU handle
 * @reg: Perfcounter reg struct to add/remove to the list
 * @update_reg: true if the perfcounter needs to be programmed by the CPU
 * @pipe: pipe id for CP aperture control
 * @flags: Flags set for requested perfcounter group
 *
 * Return: 0 on success or -EBUSY if the lock couldn't be taken
 */
int gen7_perfcounter_update(struct adreno_device *adreno_dev,
	struct adreno_perfcount_register *reg, bool update_reg, u32 pipe,
	unsigned long flags);

/*
 * gen7_ringbuffer_init - Initialize the ringbuffers
 * @adreno_dev: An Adreno GPU handle
 *
 * Initialize the ringbuffer(s) for a5xx.
 * Return: 0 on success or negative on failure
 */
int gen7_ringbuffer_init(struct adreno_device *adreno_dev);

/**
 * gen7_ringbuffer_submitcmd - Submit a user command to the ringbuffer
 * @adreno_dev: An Adreno GPU handle
 * @cmdobj: Pointer to a user command object
 * @flags: Internal submit flags
 * @time: Optional pointer to a adreno_submit_time container
 *
 * Return: 0 on success or negative on failure
 */
int gen7_ringbuffer_submitcmd(struct adreno_device *adreno_dev,
		struct kgsl_drawobj_cmd *cmdobj, u32 flags,
		struct adreno_submit_time *time);

/**
 * gen7_ringbuffer_submit - Submit a command to the ringbuffer
 * @rb: Ringbuffer pointer
 * @time: Optional pointer to a adreno_submit_time container
 *
 * Return: 0 on success or negative on failure
 */
int gen7_ringbuffer_submit(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time);

/**
 * gen7_fenced_write - Write to a fenced register
 * @adreno_dev: An Adreno GPU handle
 * @offset: Register offset
 * @value: Value to write
 * @mask: Expected FENCE_STATUS for successful write
 *
 * Return: 0 on success or negative on failure
 */
int gen7_fenced_write(struct adreno_device *adreno_dev, u32 offset,
		u32 value, u32 mask);

/**
 * gen77ringbuffer_addcmds - Wrap and submit commands to the ringbuffer
 * @adreno_dev: An Adreno GPU handle
 * @rb: Ringbuffer pointer
 * @drawctxt: Draw context submitting the commands
 * @flags: Submission flags
 * @in: Input buffer to write to ringbuffer
 * @dwords: Dword length of @in
 * @timestamp: Draw context timestamp for the submission
 * @time: Optional pointer to a adreno_submit_time container
 *
 * Return: 0 on success or negative on failure
 */
int gen7_ringbuffer_addcmds(struct adreno_device *adreno_dev,
		struct adreno_ringbuffer *rb, struct adreno_context *drawctxt,
		u32 flags, u32 *in, u32 dwords, u32 timestamp,
		struct adreno_submit_time *time);

/**
 * gen7_cp_init_cmds - Create the CP_INIT commands
 * @adreno_dev: An Adreno GPU handle
 * @cmd: Buffer to write the CP_INIT commands into
 */
void gen7_cp_init_cmds(struct adreno_device *adreno_dev, u32 *cmds);

/**
 * gen7_gmu_hfi_probe - Probe Gen7 HFI specific data
 * @adreno_dev: An Adreno GPU handle
 *
 * Return: 0 on success or negative on failure
 */
int gen7_gmu_hfi_probe(struct adreno_device *adreno_dev);

static inline const struct gen7_gpudev *
to_gen7_gpudev(const struct adreno_gpudev *gpudev)
{
	return container_of(gpudev, struct gen7_gpudev, base);
}

/**
 * gen7_reset_preempt_records - Reset the preemption buffers
 * @adreno_dev: Handle to the adreno device
 *
 * Reset the preemption records at the time of hard reset
 */
void gen7_reset_preempt_records(struct adreno_device *adreno_dev);

/**
 * gen7_enable_ahb_timeout_detection - Program AHB control registers
 * @adreno_dev: An Adreno GPU handle
 *
 * Program AHB control registers to enable AHB timeout detection.
 */
void gen7_enable_ahb_timeout_detection(struct adreno_device *adreno_dev);

/**
 * gen7_rdpm_mx_freq_update - Update the mx frequency
 * @gmu: An Adreno GMU handle
 * @freq: Frequency in KHz
 *
 * This function communicates GPU mx frequency(in Mhz) changes to rdpm.
 */
void gen7_rdpm_mx_freq_update(struct gen7_gmu_device *gmu, u32 freq);

/**
 * gen7_rdpm_cx_freq_update - Update the cx frequency
 * @gmu: An Adreno GMU handle
 * @freq: Frequency in KHz
 *
 * This function communicates GPU cx frequency(in Mhz) changes to rdpm.
 */
void gen7_rdpm_cx_freq_update(struct gen7_gmu_device *gmu, u32 freq);

/**
 * gen7_scm_gpu_init_cx_regs - Program gpu regs for feature support
 * @adreno_dev: Handle to the adreno device
 *
 * Program gpu regs for feature support. Scm call for the same
 * is added from kernel version 6.0 onwards.
 *
 * Return: 0 on success or negative on failure
 */
int gen7_scm_gpu_init_cx_regs(struct adreno_device *adreno_dev);

#ifdef CONFIG_QCOM_KGSL_CORESIGHT
void gen7_coresight_init(struct adreno_device *device);
#else
static inline void gen7_coresight_init(struct adreno_device *device) { }
#endif

#endif
