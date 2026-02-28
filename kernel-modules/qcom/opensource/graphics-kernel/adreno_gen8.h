/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _ADRENO_GEN8_H_
#define _ADRENO_GEN8_H_

#include <linux/delay.h>

#include "adreno_gen8_gmu.h"
#include "adreno_gen8_hwsched_hfi.h"
#include "gen8_reg.h"

#define GEN8_0_0_NUM_PHYSICAL_SLICES	3
#define GEN8_3_0_NUM_PHYSICAL_SLICES	1
#define GEN8_6_0_NUM_PHYSICAL_SLICES	2

/* Forward struct declaration */
struct gen8_snapshot_block_list;

extern const struct adreno_power_ops gen8_gmu_power_ops;
extern const struct adreno_power_ops gen8_hwsched_power_ops;
extern const struct adreno_perfcounters adreno_gen8_perfcounters;

struct gen8_gpudev {
	struct adreno_gpudev base;
	int (*hfi_probe)(struct adreno_device *adreno_dev);
	void (*hfi_remove)(struct adreno_device *adreno_dev);
	void (*handle_watchdog)(struct adreno_device *adreno_dev);
};

extern const struct gen8_gpudev adreno_gen8_gmu_gpudev;
extern const struct gen8_gpudev adreno_gen8_hwsched_gpudev;

struct gen8_nonctxt_overrides {
	/** offset: Dword offset of the register to write */
	u32 offset;
	/** pipelines: Pipelines to write */
	u32 pipelines;
	/** val: Value to be written to the register */
	u32 val;
	/** set: True for user override request */
	bool set;
	/**
	 * list_type: 0 If the register already present in any of exisiting static pwrup list
			1 if the register fits into IFPC static pwrup only list
			2 if the register fits into IFPC + preemption static list
			3 if the register fits into external powerup list
	 */
	u32 list_type;
};

/**
 * struct gen8_device - Container for the gen8_device
 */
struct gen8_device {
	/** @gmu: Container for the gen8 GMU device */
	struct gen8_gmu_device gmu;
	/** @adreno_dev: Container for the generic adreno device */
	struct adreno_device adreno_dev;
	/** @aperture: The last value that the host aperture register was programmed to */
	u32 aperture;
	/** @ext_pwrup_list_len: External pwrup reglist length */
	u16 ext_pwrup_list_len;
	/**
	 * @nc_overrides: Noncontext registers overrides whitelist if defined,
	 * must be null terminated
	 */
	struct gen8_nonctxt_overrides *nc_overrides;
	/** @nc_mutex: Mutex to protect nc_overrides updates */
	struct mutex nc_mutex;
	/** @nc_overrides_enabled: Set through debugfs path when any override is enabled */
	bool nc_overrides_enabled;
};

/**
 * struct gen8_pwrup_extlist - container for a powerup external reglist
 */
struct gen8_pwrup_extlist {
	/** offset: Dword offset of the register to write */
	u32 offset;
	/** pipelines: pipelines to write */
	u32 pipelines;
};

/**
 * struct gen8_protected_regs - container for a protect register span
 */
struct gen8_protected_regs {
	/** @reg: Physical protected mode register to write to */
	u32 reg;
	/** @start: Dword offset of the starting register in the range */
	u32 start;
	/** @end: Dword offset of the ending register in the range (inclusive) */
	u32 end;
	/**
	 * @noaccess: 1 if the register should not be accessible from
	 * userspace, 0 if it can be read (but not written)
	 */
	u32 noaccess;
};

/**
 * struct gen8_nonctxt_regs - Container for non context registers span
 */
struct gen8_nonctxt_regs {
	/** @offset: Dword offset of the register to write */
	u32 offset;
	/** @val: Value to write */
	u32 val;
	/** @pipelines: pipelines to write */
	u32 pipelines;
};

/**
 * struct adreno_gen8_core - gen8 specific GPU core definitions
 */
struct adreno_gen8_core {
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
	/** @zap_name: Name of the CPZ zap file */
	const char *zap_name;
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
	const struct gen8_protected_regs *protected_regs;
	/** @nonctxt_regs: Array of non context register list */
	const struct gen8_nonctxt_regs *nonctxt_regs;
	/** @ctxt_record_size: Size of the preemption record in bytes */
	u64 ctxt_record_size;
	/** @highest_bank_bit: Highest bank bit value */
	u32 highest_bank_bit;
	/** @gen8_snapshot_block_list: Device-specific blocks dumped in the snapshot */
	const struct gen8_snapshot_block_list *gen8_snapshot_block_list;
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
	/** @rt_bus_hint: IB level hint for real time clients i.e. RB-0 */
	const u32 rt_bus_hint;
	/** @fast_bus_hint: Whether or not to increase IB vote on high ddr stall */
	bool fast_bus_hint;
	/** @noc_timeout_us: GPU config NOC port timeout in usec */
	u32 noc_timeout_us;
	/** @cl_no_ft_timeout_ms: Use this timeout for CL NO_FT instead of infinite */
	u32 cl_no_ft_timeout_ms;
	/** @therm_profile: GMU thermal mitigation profile */
	const struct hfi_therm_profile_ctrl *therm_profile;
};

/**
 * struct gen8_cp_preemption_record - CP context record for
 * preemption.
 * @magic: (00) Value at this offset must be equal to
 * GEN8_CP_CTXRECORD_MAGIC_REF.
 * @info: (04) Type of record. Written non-zero (usually) by CP.
 * we must set to zero for all ringbuffers.
 * @errno: (08) Error code. Initialize this to GEN8_CP_CTXRECORD_ERROR_NONE.
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
struct gen8_cp_preemption_record {
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
 * struct gen8_cp_smmu_info - CP preemption SMMU info.
 * @magic: (00) The value at this offset must be equal to
 * GEN8_CP_SMMU_INFO_MAGIC_REF
 * @_pad4: (04) Reserved/padding
 * @ttbr0: (08) Base address of the page table for the * incoming context
 * @asid: (16) Address Space IDentifier (ASID) of the incoming context
 * @context_idr: (20) Context Identification Register value
 * @context_bank: (24) Which Context Bank in SMMU to update
 */
struct gen8_cp_smmu_info {
	u32 magic;
	u32 _pad4;
	u64 ttbr0;
	u32 asid;
	u32 context_idr;
	u32 context_bank;
};

#define GEN8_CP_SMMU_INFO_MAGIC_REF		0x241350d5UL

#define GEN8_CP_CTXRECORD_MAGIC_REF		0xae399d6eUL
/* Size of each CP preemption record */
#define GEN8_CP_CTXRECORD_SIZE_IN_BYTES		(13536 * SZ_1K)
/* Size of preemption record to be dumped in snapshot */
#define GEN8_SNAPSHOT_CTXRECORD_SIZE_IN_BYTES	(128 * 1024)
/* Size of the performance counter save/restore block (in bytes) */
#define GEN8_CP_PERFCOUNTER_SAVE_RESTORE_SIZE	(4 * 1024)

#define GEN8_CP_RB_CNTL_DEFAULT \
	(FIELD_PREP(GENMASK(7, 0), ilog2(KGSL_RB_DWORDS >> 1)) | \
	 FIELD_PREP(GENMASK(12, 8), ilog2(4)))

/* Size of the CP_INIT pm4 stream in dwords */
#define GEN8_CP_INIT_DWORDS 10

#define GEN8_INT_MASK \
	((1 << GEN8_INT_AHBERROR) |			\
	 (1 << GEN8_INT_ATBASYNCFIFOOVERFLOW) |		\
	 (1 << GEN8_INT_GPCERROR) |			\
	 (1 << GEN8_INT_SWINTERRUPT) |			\
	 (1 << GEN8_INT_HWERROR) |			\
	 (1 << GEN8_INT_PM4CPINTERRUPT) |		\
	 (1 << GEN8_INT_RB_DONE_TS) |			\
	 (1 << GEN8_INT_CACHE_CLEAN_TS) |		\
	 (1 << GEN8_INT_ATBBUSOVERFLOW) |		\
	 (1 << GEN8_INT_HANGDETECTINTERRUPT) |		\
	 (1 << GEN8_INT_OUTOFBOUNDACCESS) |		\
	 (1 << GEN8_INT_UCHETRAPINTERRUPT) |		\
	 (1 << GEN8_INT_TSBWRITEERROR) |		\
	 (1 << GEN8_INT_SWFUSEVIOLATION))

#define GEN8_HWSCHED_INT_MASK \
	((1 << GEN8_INT_AHBERROR) |			\
	 (1 << GEN8_INT_ATBASYNCFIFOOVERFLOW) |		\
	 (1 << GEN8_INT_ATBBUSOVERFLOW) |		\
	 (1 << GEN8_INT_OUTOFBOUNDACCESS) |		\
	 (1 << GEN8_INT_UCHETRAPINTERRUPT) |		\
	 (1 << GEN8_INT_TSBWRITEERROR))

/* GEN8 CX MISC interrupt bits */
#define GEN8_CX_MISC_GPU_CC_IRQ	31

#define GEN8_CX_MISC_INT_MASK	BIT(GEN8_CX_MISC_GPU_CC_IRQ)

/**
 * to_gen8_core - return the gen8 specific GPU core struct
 * @adreno_dev: An Adreno GPU device handle
 *
 * Returns:
 * A pointer to the gen8 specific GPU core struct
 */
static inline const struct adreno_gen8_core *
to_gen8_core(struct adreno_device *adreno_dev)
{
	const struct adreno_gpu_core *core = adreno_dev->gpucore;

	return container_of(core, struct adreno_gen8_core, base);
}

/* Preemption functions */
void gen8_preemption_trigger(struct adreno_device *adreno_dev, bool atomic);
void gen8_preemption_schedule(struct adreno_device *adreno_dev);
void gen8_preemption_start(struct adreno_device *adreno_dev);
int gen8_preemption_init(struct adreno_device *adreno_dev);

u32 gen8_preemption_post_ibsubmit(struct adreno_device *adreno_dev,
		u32 *cmds);
u32 gen8_preemption_pre_ibsubmit(struct adreno_device *adreno_dev,
		struct adreno_ringbuffer *rb, struct adreno_context *drawctxt,
		u32 *cmds);

void gen8_preemption_callback(struct adreno_device *adreno_dev, int bit);

void gen8_preemption_context_destroy(struct kgsl_context *context);

void gen8_preemption_prepare_postamble(struct adreno_device *adreno_dev);

void gen8_snapshot(struct adreno_device *adreno_dev,
		struct kgsl_snapshot *snapshot);
void gen8_crashdump_init(struct adreno_device *adreno_dev);

/**
 * gen8_snapshot_external_core_regs - Dump external registers into snapshot
 * @device: Pointer to KGSL device
 * @snapshot: Pointer to the snapshot
 *
 * Dump external core registers like GPUCC, CPR into GPU snapshot.
 */
void gen8_snapshot_external_core_regs(struct kgsl_device *device,
		struct kgsl_snapshot *snapshot);

/**
 * gen8_enable_ahb_timeout_detection - Program AHB control registers
 * @adreno_dev: An Adreno GPU handle
 *
 * Program AHB control registers to enable AHB timeout detection.
 *
 */
void gen8_enable_ahb_timeout_detection(struct adreno_device *adreno_dev);

/**
 * gen8_start - Program gen8 registers
 * @adreno_dev: An Adreno GPU handle
 *
 * This function does all gen8 register programming every
 * time we boot the gpu
 *
 * Return: 0 on success or negative on failure
 */
int gen8_start(struct adreno_device *adreno_dev);

/**
 * gen8_init - Initialize gen8 resources
 * @adreno_dev: An Adreno GPU handle
 *
 * This function does gen8 specific one time initialization
 * and is invoked when the very first client opens a
 * kgsl instance
 *
 * Return: Zero on success and negative error on failure
 */
int gen8_init(struct adreno_device *adreno_dev);

/**
 * gen8_cx_timer_init - Initialize the CX timer on Gen8 devices
 * @adreno_dev: Pointer to the adreno device
 *
 * Synchronize the GPU CX timer (if we have one) with the CPU timer
 */
void gen8_cx_timer_init(struct adreno_device *adreno_dev);

/**
 * gen8_get_gpu_feature_info - Get hardware supported feature info
 * @adreno_dev: Pointer to the adreno device
 *
 * Get HW supported feature info and update sofware feature configuration
 */
void gen8_get_gpu_feature_info(struct adreno_device *adreno_dev);

/**
 * gen8_rb_start - Gen8 specific ringbuffer setup
 * @adreno_dev: An Adreno GPU handle
 *
 * This function does gen8 specific ringbuffer setup and
 * attempts to submit CP INIT and bring GPU out of secure mode
 *
 * Return: Zero on success and negative error on failure
 */
int gen8_rb_start(struct adreno_device *adreno_dev);

/**
 * gen8_microcode_read - Get the cp microcode from the filesystem
 * @adreno_dev: An Adreno GPU handle
 *
 * This function gets the firmware from filesystem and sets up
 * the micorocode global buffer
 *
 * Return: Zero on success and negative error on failure
 */
int gen8_microcode_read(struct adreno_device *adreno_dev);

/**
 * gen8_probe_common - Probe common gen8 resources
 * @pdev: Pointer to the platform device
 * @adreno_dev: Pointer to the adreno device
 * @chipid: Chipid of the target
 * @gpucore: Pointer to the gpucore strucure
 *
 * This function sets up the gen8 resources common across all
 * gen8 targets
 */
int gen8_probe_common(struct platform_device *pdev,
	struct adreno_device *adreno_dev, u32 chipid,
	const struct adreno_gpu_core *gpucore);

/**
 * gen8_hw_isidle - Check whether gen8 gpu is idle or not
 * @adreno_dev: An Adreno GPU handle
 *
 * Return: True if gpu is idle, otherwise false
 */
bool gen8_hw_isidle(struct adreno_device *adreno_dev);

/**
 * gen8_spin_idle_debug - Debug logging used when gpu fails to idle
 * @adreno_dev: An Adreno GPU handle
 *
 * This function logs interesting registers and triggers a snapshot
 */
void gen8_spin_idle_debug(struct adreno_device *adreno_dev,
	const char *str);

/**
 * gen8_perfcounter_update - Update the IFPC perfcounter list
 * @adreno_dev: An Adreno GPU handle
 * @reg: Perfcounter reg struct to add/remove to the list
 * @update_reg: true if the perfcounter needs to be programmed by the CPU
 * @pipe: pipe id for CP aperture control
 * @flags: Flags set for requested perfcounter group
 *
 * Return: 0 on success or -EBUSY if the lock couldn't be taken
 */
int gen8_perfcounter_update(struct adreno_device *adreno_dev,
	struct adreno_perfcount_register *reg, bool update_reg, u32 pipe,
	unsigned long flags);

/*
 * gen8_ringbuffer_init - Initialize the ringbuffers
 * @adreno_dev: An Adreno GPU handle
 *
 * Initialize the ringbuffer(s) for a5xx.
 * Return: 0 on success or negative on failure
 */
int gen8_ringbuffer_init(struct adreno_device *adreno_dev);

/**
 * gen8_ringbuffer_submitcmd - Submit a user command to the ringbuffer
 * @adreno_dev: An Adreno GPU handle
 * @cmdobj: Pointer to a user command object
 * @flags: Internal submit flags
 * @time: Optional pointer to a adreno_submit_time container
 *
 * Return: 0 on success or negative on failure
 */
int gen8_ringbuffer_submitcmd(struct adreno_device *adreno_dev,
		struct kgsl_drawobj_cmd *cmdobj, u32 flags,
		struct adreno_submit_time *time);

/**
 * gen8_ringbuffer_submit - Submit a command to the ringbuffer
 * @rb: Ringbuffer pointer
 * @time: Optional pointer to a adreno_submit_time container
 *
 * Return: 0 on success or negative on failure
 */
int gen8_ringbuffer_submit(struct adreno_ringbuffer *rb,
		struct adreno_submit_time *time);

/**
 * gen8_fenced_write - Write to a fenced register
 * @adreno_dev: An Adreno GPU handle
 * @offset: Register offset
 * @value: Value to write
 * @mask: Expected FENCE_STATUS for successful write
 *
 * Return: 0 on success or negative on failure
 */
int gen8_fenced_write(struct adreno_device *adreno_dev, u32 offset,
		u32 value, u32 mask);

/**
 * gen87ringbuffer_addcmds - Wrap and submit commands to the ringbuffer
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
int gen8_ringbuffer_addcmds(struct adreno_device *adreno_dev,
		struct adreno_ringbuffer *rb, struct adreno_context *drawctxt,
		u32 flags, u32 *in, u32 dwords, u32 timestamp,
		struct adreno_submit_time *time);

/**
 * gen8_cp_init_cmds - Create the CP_INIT commands
 * @adreno_dev: An Adreno GPU handle
 * @cmd: Buffer to write the CP_INIT commands into
 */
void gen8_cp_init_cmds(struct adreno_device *adreno_dev, u32 *cmds);

/**
 * gen8_gmu_hfi_probe - Probe Gen8 HFI specific data
 * @adreno_dev: An Adreno GPU handle
 *
 * Return: 0 on success or negative on failure
 */
int gen8_gmu_hfi_probe(struct adreno_device *adreno_dev);

static inline const struct gen8_gpudev *
to_gen8_gpudev(const struct adreno_gpudev *gpudev)
{
	return container_of(gpudev, struct gen8_gpudev, base);
}

/**
 * gen8_reset_preempt_records - Reset the preemption buffers
 * @adreno_dev: Handle to the adreno device
 *
 * Reset the preemption records at the time of hard reset
 */
void gen8_reset_preempt_records(struct adreno_device *adreno_dev);

/**
 * gen8_rdpm_mx_freq_update - Update the mx frequency
 * @gmu: An Adreno GMU handle
 * @freq: Frequency in KHz
 *
 * This function communicates GPU mx frequency(in Mhz) changes to rdpm.
 */
void gen8_rdpm_mx_freq_update(struct gen8_gmu_device *gmu, u32 freq);

/**
 * gen8_rdpm_cx_freq_update - Update the cx frequency
 * @gmu: An Adreno GMU handle
 * @freq: Frequency in KHz
 *
 * This function communicates GPU cx frequency(in Mhz) changes to rdpm.
 */
void gen8_rdpm_cx_freq_update(struct gen8_gmu_device *gmu, u32 freq);

/**
 * gen8_scm_gpu_init_cx_regs - Program gpu regs for feature support
 * @adreno_dev: Handle to the adreno device
 *
 * Program gpu regs for feature support. Scm call for the same
 * is added from kernel version 6.0 onwards.
 *
 * Return: 0 on success or negative on failure
 */
int gen8_scm_gpu_init_cx_regs(struct adreno_device *adreno_dev);

/**
 * gen8_legacy_snapshot_registers - Dump registers for GPU/GMU
 * @device: Handle to the KGSL device
 * @buf: Target buffer to copy the data
 * @remain: Buffer size remaining for dump
 * @priv: Private data to dump the registers
 *
 * Return: Size of the section
 */
size_t gen8_legacy_snapshot_registers(struct kgsl_device *device,
		 u8 *buf, size_t remain, void *priv);

/**
 * gen8_regread64_aperture - Read 64 bit register values
 * @device: Handle to the KGSL device
 * @offsetwords_lo: Lower 32 bit address to read
 * @offsetwords_hi: Higher 32 bit address to read
 * @value: The value of register at offsetwords
 * @pipe: Pipe for which the register is to be read
 * @slice_id: Slice for which the register is to be read
 * @use_slice_id: Set if the value to be read is from a sliced register
 *
 * This function reads the 64 bit value for registers
 */
void gen8_regread64_aperture(struct kgsl_device *device,
	u32 offsetwords_lo, u32 offsetwords_hi, u64 *value, u32 pipe,
	u32 slice_id, u32 use_slice_id);

/**
 * gen8_regread_aperture - Read 32 bit register values
 * @device: Handle to the KGSL device
 * @offsetwords: 32 bit address to read
 * @value: The value of register at offsetwords
 * @pipe: Pipe for which the register is to be read
 * @slice_id: Slice for which the register is to be read
 * @use_slice_id: Set if the value to be read is from a sliced register
 *
 * This function reads the 32 bit value for registers
 */
void gen8_regread_aperture(struct kgsl_device *device,
	u32 offsetwords, u32 *value, u32 pipe, u32 slice_id, u32 use_slice_id);


/**
 * gen8_host_aperture_set - Program CP aperture register
 * @adreno_dev: Handle to the adreno device
 * @pipe_id: Pipe for which the register is to be set
 * @slice_id: Slice for which the register is to be set
 * @use_slice_id: Set if the value to be read is from a sliced register
 *
 * This function programs CP aperture register
 */
void gen8_host_aperture_set(struct adreno_device *adreno_dev, u32 pipe_id,
		u32 slice_id, u32 use_slice_id);

#if IS_ENABLED(CONFIG_QCOM_KGSL_CORESIGHT)
void gen8_coresight_init(struct adreno_device *device);
#else
static inline void gen8_coresight_init(struct adreno_device *device) { }
#endif

/**
 * gen8_get_num_slices - Get the number of physical slices for Gen8 GPUs
 * @adreno_dev: Handle to the adreno device
 *
 * Return: Number of physical slices available on Gen8 GPUs
 */
static inline u32 gen8_get_num_slices(struct adreno_device *adreno_dev)
{
	if (adreno_is_gen8_3_0(adreno_dev))
		return GEN8_3_0_NUM_PHYSICAL_SLICES;
	else if (adreno_is_gen8_6_0(adreno_dev))
		return GEN8_6_0_NUM_PHYSICAL_SLICES;
	else
		return GEN8_0_0_NUM_PHYSICAL_SLICES;
}
#endif
