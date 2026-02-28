// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2002,2007-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/interconnect.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/module.h>
#include <linux/msm_kgsl.h>
#include <linux/regulator/consumer.h>
#include <linux/nvmem-consumer.h>
#include <linux/pm_domain.h>
#include <linux/reset.h>
#include <linux/trace.h>
#include <linux/units.h>
#include <linux/version.h>
#include <soc/qcom/dcvs.h>
#include <soc/qcom/socinfo.h>
#include <linux/suspend.h>

#include "adreno.h"
#include "adreno_a5xx.h"
#include "adreno_a6xx.h"
#include "adreno_compat.h"
#include "adreno_pm4types.h"
#include "adreno_trace.h"
#include "kgsl_bus.h"
#include "kgsl_power_trace.h"
#include "kgsl_reclaim.h"
#include "kgsl_trace.h"
#include "kgsl_util.h"

/* Include the master list of GPU cores that are supported */
#include "adreno-gpulist.h"

static void adreno_unbind(struct device *dev);
static void adreno_input_work(struct work_struct *work);
static int adreno_soft_reset(struct kgsl_device *device);
static unsigned int counter_delta(struct kgsl_device *device,
	unsigned int reg, unsigned int *counter);
static struct device_node *
	adreno_get_gpu_model_node(struct platform_device *pdev);

static struct adreno_device device_3d0;
static bool adreno_preemption_enable;
static u32 kgsl_gpu_sku_override = U32_MAX;
static u32 kgsl_gpu_speed_bin_override = U32_MAX;

/* Nice level for the higher priority GPU start thread */
int adreno_wake_nice = -7;

/* Number of milliseconds to stay active after a wake on touch */
unsigned int adreno_wake_timeout = 100;

static u32 get_ucode_version(const u32 *data)
{
	u32 version;

	version = data[1];

	if ((version & 0xf) != 0xa)
		return version;

	version &= ~0xfff;
	return  version | ((data[3] & 0xfff000) >> 12);
}

int adreno_get_firmware(struct adreno_device *adreno_dev,
		const char *fwfile, struct adreno_firmware *firmware)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct firmware *fw = NULL;
	int ret;

	if (!IS_ERR_OR_NULL(firmware->memdesc))
		return 0;

	ret = request_firmware(&fw, fwfile, &device->pdev->dev);

	if (ret) {
		dev_err(device->dev, "request_firmware(%s) failed: %d\n",
				fwfile, ret);
		return ret;
	}

	firmware->memdesc = kgsl_allocate_global(device, fw->size - 4, 0,
				KGSL_MEMFLAGS_GPUREADONLY, KGSL_MEMDESC_UCODE,
				"ucode");

	ret = PTR_ERR_OR_ZERO(firmware->memdesc);
	if (!ret) {
		memcpy(firmware->memdesc->hostptr, &fw->data[4], fw->size - 4);
		firmware->size = (fw->size - 4) / sizeof(u32);
		firmware->version = get_ucode_version((u32 *)fw->data);
	}

	release_firmware(fw);
	return ret;
}


int adreno_zap_shader_load(struct adreno_device *adreno_dev,
		const char *name)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	if (!name || adreno_dev->zap_loaded)
		return 0;

	ret = kgsl_zap_shader_load(&device->pdev->dev, name);
	if (!ret)
		adreno_dev->zap_loaded = true;

	return ret;
}

#if (IS_ENABLED(CONFIG_QCOM_KGSL_HIBERNATION) || IS_ENABLED(CONFIG_DEEPSLEEP))
static void adreno_zap_shader_unload(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	if (adreno_dev->zap_loaded) {
		ret = kgsl_zap_shader_unload(&device->pdev->dev);
		if (!ret)
			adreno_dev->zap_loaded = false;
	}
}
#endif

/**
 * adreno_readreg64() - Read a 64bit register by getting its offset from the
 * offset array defined in gpudev node
 * @adreno_dev: Pointer to the adreno device
 * @lo:	lower 32bit register enum that is to be read
 * @hi:	higher 32bit register enum that is to be read
 * @val: 64 bit Register value read is placed here
 */
void adreno_readreg64(struct adreno_device *adreno_dev,
		enum adreno_regs lo, enum adreno_regs hi, uint64_t *val)
{
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	unsigned int val_lo = 0, val_hi = 0;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	if (adreno_checkreg_off(adreno_dev, lo))
		kgsl_regread(device, gpudev->reg_offsets[lo], &val_lo);
	if (adreno_checkreg_off(adreno_dev, hi))
		kgsl_regread(device, gpudev->reg_offsets[hi], &val_hi);

	*val = (val_lo | ((uint64_t)val_hi << 32));
}

/**
 * adreno_get_rptr() - Get the current ringbuffer read pointer
 * @rb: Pointer the ringbuffer to query
 *
 * Get the latest rptr
 */
unsigned int adreno_get_rptr(struct adreno_ringbuffer *rb)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	u32 rptr = 0;

	kgsl_sharedmem_readl(device->scratch, &rptr,
			     SCRATCH_RB_OFFSET(rb->id, rptr));

	return rptr;
}

static void adreno_touch_wakeup(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	/*
	 * Don't schedule adreno_start in a high priority workqueue, we are
	 * already in a workqueue which should be sufficient
	 */
	kgsl_pwrctrl_change_state(device, KGSL_STATE_ACTIVE);

	/*
	 * When waking up from a touch event we want to stay active long enough
	 * for the user to send a draw command.  The default idle timer timeout
	 * is shorter than we want so go ahead and push the idle timer out
	 * further for this special case
	 */
	mod_timer(&device->idle_timer,
		jiffies + msecs_to_jiffies(adreno_wake_timeout));

}

/*
 * A workqueue callback responsible for actually turning on the GPU after a
 * touch event. kgsl_pwrctrl_change_state(ACTIVE) is used without any
 * active_count protection to avoid the need to maintain state.  Either
 * somebody will start using the GPU or the idle timer will fire and put the
 * GPU back into slumber.
 */
static void adreno_input_work(struct work_struct *work)
{
	struct adreno_device *adreno_dev = container_of(work,
			struct adreno_device, input_work);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_power_ops *ops = ADRENO_POWER_OPS(adreno_dev);

	mutex_lock(&device->mutex);

	device->pwrctrl.wake_on_touch = true;

	ops->touch_wakeup(adreno_dev);

	mutex_unlock(&device->mutex);
}

/* Wake up the touch event kworker to initiate GPU wakeup */
void adreno_touch_wake(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	/*
	 * Don't do anything if anything hasn't been rendered since we've been
	 * here before
	 */

	if (device->pwrctrl.wake_on_touch)
		return;

	if (device->state == KGSL_STATE_SLUMBER)
		schedule_work(&adreno_dev->input_work);
}

/*
 * Process input events and schedule work if needed.  At this point we are only
 * interested in groking EV_ABS touchscreen events
 */
static void adreno_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	struct kgsl_device *device = handle->handler->private;

	/* Only consider EV_ABS (touch) events */
	if (type == EV_ABS)
		adreno_touch_wake(device);
}

#ifdef CONFIG_INPUT
static int adreno_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = handler->name;

	ret = input_register_handle(handle);
	if (ret) {
		kfree(handle);
		return ret;
	}

	ret = input_open_device(handle);
	if (ret) {
		input_unregister_handle(handle);
		kfree(handle);
	}

	return ret;
}

static void adreno_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}
#else
static int adreno_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	return 0;
}
static void adreno_input_disconnect(struct input_handle *handle) {}
#endif

/*
 * We are only interested in EV_ABS events so only register handlers for those
 * input devices that have EV_ABS events
 */
static const struct input_device_id adreno_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		/* assumption: MT_.._X & MT_.._Y are in the same long */
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
				BIT_MASK(ABS_MT_POSITION_X) |
				BIT_MASK(ABS_MT_POSITION_Y) },
	},
	{ },
};

static struct input_handler adreno_input_handler = {
	.event = adreno_input_event,
	.connect = adreno_input_connect,
	.disconnect = adreno_input_disconnect,
	.name = "kgsl",
	.id_table = adreno_input_ids,
};

/*
 * _soft_reset() - Soft reset GPU
 * @adreno_dev: Pointer to adreno device
 *
 * Soft reset the GPU by doing a AHB write of value 1 to RBBM_SW_RESET
 * register. This is used when we want to reset the GPU without
 * turning off GFX power rail. The reset when asserted resets
 * all the HW logic, restores GPU registers to default state and
 * flushes out pending VBIF transactions.
 */
static void _soft_reset(struct adreno_device *adreno_dev)
{
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);
	unsigned int reg;

	adreno_writereg(adreno_dev, ADRENO_REG_RBBM_SW_RESET_CMD, 1);
	/*
	 * Do a dummy read to get a brief read cycle delay for the
	 * reset to take effect
	 */
	adreno_readreg(adreno_dev, ADRENO_REG_RBBM_SW_RESET_CMD, &reg);
	adreno_writereg(adreno_dev, ADRENO_REG_RBBM_SW_RESET_CMD, 0);

	/* The SP/TP regulator gets turned off after a soft reset */

	clear_bit(ADRENO_DEVICE_GPU_REGULATOR_ENABLED, &adreno_dev->priv);
	if (gpudev->regulator_enable)
		gpudev->regulator_enable(adreno_dev);
}

/**
 * adreno_irqctrl() - Enables/disables the RBBM interrupt mask
 * @adreno_dev: Pointer to an adreno_device
 * @state: 1 for masked or 0 for unmasked
 * Power: The caller of this function must make sure to use OOBs
 * so that we know that the GPU is powered on
 */
void adreno_irqctrl(struct adreno_device *adreno_dev, int state)
{
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	if (!adreno_dev->irq_mask)
		return;

	adreno_writereg(adreno_dev, ADRENO_REG_RBBM_INT_0_MASK,
		state ? adreno_dev->irq_mask : 0);

	if (gpudev->swfuse_irqctrl)
		gpudev->swfuse_irqctrl(adreno_dev, state);
}

/*
 * adreno_hang_int_callback() - Isr for fatal interrupts that hang GPU
 * @adreno_dev: Pointer to device
 * @bit: Interrupt bit
 */
void adreno_hang_int_callback(struct adreno_device *adreno_dev, int bit)
{
	dev_crit_ratelimited(KGSL_DEVICE(adreno_dev)->dev,
				"MISC: GPU hang detected\n");
	adreno_irqctrl(adreno_dev, 0);

	/* Trigger a fault in the dispatcher - this will effect a restart */
	adreno_scheduler_fault(adreno_dev, ADRENO_HARD_FAULT);
}

/*
 * adreno_cp_callback() - CP interrupt handler
 * @adreno_dev: Adreno device pointer
 * @irq: irq number
 *
 * Handle the cp interrupt generated by GPU.
 */
void adreno_cp_callback(struct adreno_device *adreno_dev, int bit)
{
	adreno_scheduler_queue(adreno_dev);
}

static irqreturn_t adreno_irq_handler(int irq, void *data)
{
	struct kgsl_device *device = data;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	irqreturn_t ret;

	atomic_inc(&adreno_dev->pending_irq_refcnt);
	/* Ensure this increment is done before the IRQ status is updated */
	smp_mb__after_atomic();

	ret = gpudev->irq_handler(adreno_dev);

	/* Make sure the regwrites are done before the decrement */
	smp_mb__before_atomic();
	atomic_dec(&adreno_dev->pending_irq_refcnt);
	/* Ensure other CPUs see the decrement */
	smp_mb__after_atomic();

	return ret;
}

static irqreturn_t adreno_freq_limiter_irq_handler(int irq, void *data)
{
	struct kgsl_device *device = data;

	KGSL_PWRCTRL_LOG_FREQLIM(device);

	reset_control_reset(device->freq_limiter_irq_clear);

	return IRQ_HANDLED;
}

irqreturn_t adreno_irq_callbacks(struct adreno_device *adreno_dev,
		const struct adreno_irq_funcs *funcs, u32 status)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	irqreturn_t ret = IRQ_NONE;

	/* Loop through all set interrupts and call respective handlers */
	while (status) {
		int i = fls(status) - 1;

		if (funcs[i].func) {
			if (adreno_dev->irq_mask & BIT(i))
				funcs[i].func(adreno_dev, i);
		} else
			dev_crit_ratelimited(device->dev,
				"Unhandled interrupt bit %x\n", i);

		ret = IRQ_HANDLED;

		status &= ~BIT(i);
	}

	return ret;
}

static int adreno_get_chipid(struct platform_device *pdev, u32 *chipid);

static inline bool _rev_match(unsigned int id, unsigned int entry)
{
	return (entry == ANY_ID || entry == id);
}

static const struct adreno_gpu_core *
_get_gpu_core(struct platform_device *pdev, u32 *chipid)
{
	int i;
	struct device_node *node;

	/*
	 * When "qcom,gpu-models" is defined, use gpu model node to match
	 * on a compatible string, otherwise match using legacy way.
	 */
	node = adreno_get_gpu_model_node(pdev);
	if (!node || !of_find_property(node, "compatible", NULL))
		node = pdev->dev.of_node;

	*chipid = 0;

	/* Check to see if any of the entries match on a compatible string */
	for (i = 0; i < ARRAY_SIZE(adreno_gpulist); i++) {
		if (adreno_gpulist[i]->compatible &&
				of_device_is_compatible(node,
					adreno_gpulist[i]->compatible)) {
			/*
			 * We matched compat string, set chipid based on
			 * dtsi, else fail.
			 */
			if (!adreno_get_chipid(pdev, chipid))
				return adreno_gpulist[i];

			dev_crit(&pdev->dev,
					"No chipid associated with %s\n",
					adreno_gpulist[i]->compatible);
			return NULL;
		}
	}

	/* No compatible string so try and match on chipid */
	if (!adreno_get_chipid(pdev, chipid)) {
		unsigned int core = ADRENO_CHIPID_CORE(*chipid);
		unsigned int major = ADRENO_CHIPID_MAJOR(*chipid);
		unsigned int minor = ADRENO_CHIPID_MINOR(*chipid);
		unsigned int patchid = ADRENO_CHIPID_PATCH(*chipid);

		for (i = 0; i < ARRAY_SIZE(adreno_gpulist); i++) {
			if (core == adreno_gpulist[i]->core &&
				_rev_match(major, adreno_gpulist[i]->major) &&
				_rev_match(minor, adreno_gpulist[i]->minor) &&
				_rev_match(patchid, adreno_gpulist[i]->patchid))
				return adreno_gpulist[i];
		}
	}

	dev_crit(&pdev->dev, "Unknown GPU chip ID %8.8x\n", *chipid);
	return NULL;
}

static struct {
	unsigned int quirk;
	const char *prop;
} adreno_quirks[] = {
	 { ADRENO_QUIRK_TWO_PASS_USE_WFI, "qcom,gpu-quirk-two-pass-use-wfi" },
	 { ADRENO_QUIRK_CRITICAL_PACKETS, "qcom,gpu-quirk-critical-packets" },
	 { ADRENO_QUIRK_FAULT_DETECT_MASK, "qcom,gpu-quirk-fault-detect-mask" },
	 { ADRENO_QUIRK_DISABLE_RB_DP2CLOCKGATING,
			"qcom,gpu-quirk-dp2clockgating-disable" },
	 { ADRENO_QUIRK_DISABLE_LMLOADKILL,
			"qcom,gpu-quirk-lmloadkill-disable" },
	{ ADRENO_QUIRK_HFI_USE_REG, "qcom,gpu-quirk-hfi-use-reg" },
	{ ADRENO_QUIRK_SECVID_SET_ONCE, "qcom,gpu-quirk-secvid-set-once" },
	{ ADRENO_QUIRK_LIMIT_UCHE_GBIF_RW,
			"qcom,gpu-quirk-limit-uche-gbif-rw" },
	{ ADRENO_QUIRK_CX_GDSC, "qcom,gpu-quirk-cx-gdsc" },
};

static int adreno_get_chipid(struct platform_device *pdev, u32 *chipid)
{
	u32 id;

	if (!of_property_read_u32(pdev->dev.of_node, "qcom,chipid", chipid))
		return 0;

	id = socinfo_get_partinfo_chip_id(SOCINFO_PART_GPU);
	if (id)
		*chipid = id;

	return id ? 0 : -EINVAL;
}

static void
adreno_update_soc_hw_revision_quirks(struct adreno_device *adreno_dev,
		struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int i;

	/* update quirk */
	for (i = 0; i < ARRAY_SIZE(adreno_quirks); i++) {
		if (of_property_read_bool(node, adreno_quirks[i].prop))
			adreno_dev->quirks |= adreno_quirks[i].quirk;
	}
}

static const struct adreno_gpu_core *
adreno_identify_gpu(struct platform_device *pdev, u32 *chipid)
{
	const struct adreno_gpu_core *gpucore;

	gpucore = _get_gpu_core(pdev, chipid);
	if (!gpucore)
		return ERR_PTR(-ENODEV);

	/*
	 * Identify non-longer supported targets and spins and print a helpful
	 * message
	 */
	if (gpucore->features & ADRENO_DEPRECATED) {
		if (gpucore->compatible)
			dev_err(&pdev->dev,
				"Support for GPU %s has been deprecated\n",
				gpucore->compatible);
		else
			dev_err(&pdev->dev,
				"Support for GPU %x.%d.%x.%d has been deprecated\n",
				gpucore->core, gpucore->major,
				gpucore->minor, gpucore->patchid);
		return ERR_PTR(-ENODEV);
	}

	return gpucore;
}

static const struct of_device_id adreno_match_table[] = {
	{ .compatible = "qcom,kgsl-3d0", .data = &device_3d0 },
	{ },
};

MODULE_DEVICE_TABLE(of, adreno_match_table);

/* Dynamically build the OPP table for the GPU device */
static void adreno_build_opp_table(struct device *dev, struct kgsl_pwrctrl *pwr)
{
	int i;

	/* Skip if the table has already been populated */
	if (dev_pm_opp_get_opp_count(dev) > 0)
		return;

	/* Add all the supported frequencies into the tree */
	for (i = 0; i < pwr->num_pwrlevels; i++)
		dev_pm_opp_add(dev, pwr->pwrlevels[i].gpu_freq, 0);
}

static int adreno_of_parse_pwrlevels(struct adreno_device *adreno_dev,
		struct device_node *node)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct device_node *child;
	int ret;

	pwr->num_pwrlevels = 0;

	for_each_child_of_node(node, child) {
		u32 index, freq = 0, voltage, bus;
		struct kgsl_pwrlevel *level;

		ret = of_property_read_u32(child, "reg", &index);
		if (ret) {
			dev_err(device->dev, "%pOF: powerlevel index not found\n",
				child);
			goto out;
		}

		ret = of_property_read_u32(child, "qcom,gpu-freq", &freq);
		if (ret) {
			dev_err(device->dev, "%pOF: Unable to read qcom,gpu-freq\n",
				child);
			goto out;
		}

		/* Ignore "zero" powerlevels */
		if (!freq)
			continue;

		ret = of_property_read_u32(child, "qcom,level", &voltage);
		if (ret) {
			dev_err(device->dev, "%pOF: Unable to read qcom,level\n",
				child);
			goto out;
		}

		ret = kgsl_of_property_read_ddrtype(child, "qcom,bus-freq",
			&bus);
		if (ret) {
			dev_err(device->dev, "%pOF:Unable to read qcom,bus-freq\n",
				child);
			goto out;
		}

		if (index >= ARRAY_SIZE(pwr->pwrlevels)) {
			dev_err(device->dev, "%pOF: Pwrlevel index %d is out of range\n",
				child, index);
			continue;
		}

		if (index >= pwr->num_pwrlevels)
			pwr->num_pwrlevels = index + 1;

		level = &pwr->pwrlevels[index];

		level->gpu_freq = freq;
		level->bus_freq = bus;
		level->voltage_level = voltage;
		level->cx_level = 0xffffffff;

		of_property_read_u32(child, "qcom,acd-level",
			&level->acd_level);

		of_property_read_u32(child, "qcom,cx-level",
			&level->cx_level);

		level->bus_min = level->bus_freq;
		kgsl_of_property_read_ddrtype(child,
			"qcom,bus-min", &level->bus_min);

		level->bus_max = level->bus_freq;
		kgsl_of_property_read_ddrtype(child,
			"qcom,bus-max", &level->bus_max);
	}

	adreno_build_opp_table(&device->pdev->dev, pwr);
	return 0;
out:
	of_node_put(child);
	return ret;
}

static void adreno_of_get_initial_pwrlevels(struct kgsl_pwrctrl *pwr,
		struct device_node *node)
{
	int level;

	/* Get and set the initial power level */
	if (WARN_ON(of_property_read_u32(node, "qcom,initial-pwrlevel", &level) ||
		level < 0 || level >= pwr->num_pwrlevels))
		level = 1;

	pwr->active_pwrlevel = level;
	pwr->default_pwrlevel = level;

	/* Set the max power level */
	pwr->max_pwrlevel = 0;

	/* Get and set the min power level */
	if (of_property_read_u32(node, "qcom,initial-min-pwrlevel", &level))
		level = pwr->num_pwrlevels - 1;

	if (level < 0 || level >= pwr->num_pwrlevels || level < pwr->default_pwrlevel)
		level = pwr->num_pwrlevels - 1;

	pwr->min_render_pwrlevel = level;
	pwr->min_pwrlevel = level;
}

static void adreno_of_get_limits(struct adreno_device *adreno_dev,
		struct device_node *node)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwrctrl = &device->pwrctrl;
	unsigned int throttle_level;

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_LM) || of_property_read_u32(node,
				"qcom,throttle-pwrlevel", &throttle_level))
		return;

	throttle_level = min(throttle_level, pwrctrl->num_pwrlevels - 1);

	pwrctrl->throttle_mask = GENMASK(pwrctrl->num_pwrlevels - 1,
			pwrctrl->num_pwrlevels - 1 - throttle_level);

	adreno_dev->lm_enabled = true;
}

static int adreno_of_get_legacy_pwrlevels(struct adreno_device *adreno_dev,
		struct device_node *parent)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct device_node *node;
	int ret;

	node = of_find_node_by_name(parent, "qcom,gpu-pwrlevels");

	if (node == NULL) {
		dev_err(&device->pdev->dev,
			"Unable to find 'qcom,gpu-pwrlevels'\n");
		return -EINVAL;
	}

	ret = adreno_of_parse_pwrlevels(adreno_dev, node);

	if (!ret) {
		adreno_of_get_initial_pwrlevels(&device->pwrctrl, parent);
		adreno_of_get_limits(adreno_dev, parent);
	}

	of_node_put(node);
	return ret;
}

static int adreno_of_get_pwrlevels(struct adreno_device *adreno_dev,
		struct device_node *parent)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct device_node *node, *child;

	node = of_find_node_by_name(parent, "qcom,gpu-pwrlevel-bins");
	if (node == NULL)
		return adreno_of_get_legacy_pwrlevels(adreno_dev, parent);

	for_each_child_of_node(node, child) {
		bool match = false;
		int tbl_size;
		u32 bin = 0;

		/* Check if the bin has a speed-bin requirement */
		if (!of_property_read_u32(child, "qcom,speed-bin", &bin))
			match = (bin == device->speed_bin);

		/* Check if the bin has a sku-code requirement */
		if (of_get_property(child, "qcom,sku-codes", &tbl_size)) {
			int num_codes = tbl_size / sizeof(u32);
			int i;
			u32 sku_code;

			/*
			 * If we have a speed-bin requirement that did not match
			 * keep searching.
			 */
			if (bin && !match)
				continue;

			/* Check if the soc_code matches any of the sku codes */
			match = false;
			for (i = 0; i < num_codes; i++) {
				if (!of_property_read_u32_index(child, "qcom,sku-codes",
								i, &sku_code) &&
					(sku_code == 0 || device->soc_code == sku_code)) {
					match = true;
					break;
				}
			}
		}

		if (match) {
			int ret;

			ret = adreno_of_parse_pwrlevels(adreno_dev, child);
			if (ret) {
				of_node_put(child);
				return ret;
			}

			adreno_of_get_initial_pwrlevels(&device->pwrctrl, child);

			/*
			 * Check for global throttle-pwrlevel first and override
			 * with speedbin specific one if found.
			 */
			adreno_of_get_limits(adreno_dev, parent);
			adreno_of_get_limits(adreno_dev, child);

			of_node_put(child);
			return 0;
		}
	}

	dev_err(&device->pdev->dev,
		"No match for speed_bin:%d and soc_code:0x%x\n",
		device->speed_bin, device->soc_code);
	return -ENODEV;
}

static int register_l3_voter(struct kgsl_device *device)
{
	int ret = 0;

	/*
	 * The L3 vote setup is performed only once. Once set up is done, it is
	 * safe to access num_l3_pwrlevels without acquiring the device mutex.
	 * Therefore, an early check can be added without taking the mutex.
	 */
	if (READ_ONCE(device->num_l3_pwrlevels))
		return ret;

	mutex_lock(&device->mutex);

	if (!device->l3_vote)
		goto done;

	/* Verify again if the L3 vote is set up to handle races */
	if (device->num_l3_pwrlevels)
		goto done;

	memset(device->l3_freq, 0x0, sizeof(device->l3_freq));

	ret = qcom_dcvs_register_voter(KGSL_L3_DEVICE, DCVS_L3, DCVS_SLOW_PATH);
	if (ret) {
		dev_err_once(&device->pdev->dev,
			"Unable to register l3 dcvs voter: %d\n", ret);
		goto done;
	}

	ret = qcom_dcvs_hw_minmax_get(DCVS_L3, &device->l3_freq[1],
		&device->l3_freq[2]);
	if (ret) {
		dev_err_once(&device->pdev->dev,
			"Unable to get min/max for l3 dcvs: %d\n", ret);
		qcom_dcvs_unregister_voter(KGSL_L3_DEVICE, DCVS_L3,
			DCVS_SLOW_PATH);
		memset(device->l3_freq, 0x0, sizeof(device->l3_freq));
		goto done;
	}

	WRITE_ONCE(device->num_l3_pwrlevels, 3);

done:
	mutex_unlock(&device->mutex);

	return ret;
}

static int adreno_of_get_power(struct adreno_device *adreno_dev,
		struct platform_device *pdev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	ret = adreno_of_get_pwrlevels(adreno_dev, pdev->dev.of_node);
	if (ret)
		return ret;

	atomic64_set(&device->pwrctrl.interval_timeout, CONFIG_QCOM_KGSL_IDLE_TIMEOUT);

	/* Set default bus control to true on all targets */
	device->pwrctrl.bus_control = true;

	return 0;
}

/* Read the fuse through the new and fancy nvmem method */
static int adreno_read_speed_bin(struct platform_device *pdev)
{
	struct nvmem_cell *cell = nvmem_cell_get(&pdev->dev, "speed_bin");
	int ret = PTR_ERR_OR_ZERO(cell);
	void *buf;
	int val = 0;
	size_t len;

	if (ret) {
		if (ret == -ENOENT)
			return 0;

		return ret;
	}

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	memcpy(&val, buf, min(len, sizeof(val)));
	kfree(buf);

	return val;
}

static void adreno_read_soc_code(struct kgsl_device *device)
{
	int feature_code, pcode;
	bool internal_sku;

	feature_code = max_t(int, socinfo_get_feature_code(), SOCINFO_FC_UNKNOWN);
	internal_sku = (feature_code >= SOCINFO_FC_Y0) && (feature_code < SOCINFO_FC_INT_RESERVE);

	/* Pcode is significant only for internal SKUs */
	pcode = internal_sku ? max_t(int, socinfo_get_pcode(), SOCINFO_PCODE_UNKNOWN) :
			SOCINFO_PCODE_UNKNOWN;

	device->soc_code = FIELD_PREP(GENMASK(31, 16), pcode) |
				FIELD_PREP(GENMASK(15, 0), feature_code);

	/* Override soc_code and speed_bin for internal feature codes only */
	if (internal_sku) {
		if (kgsl_gpu_sku_override != U32_MAX)
			device->soc_code = kgsl_gpu_sku_override;

		if (kgsl_gpu_speed_bin_override != U32_MAX)
			device->speed_bin = kgsl_gpu_speed_bin_override;
	}
}

static int adreno_read_gpu_model_fuse(struct platform_device *pdev)
{
	struct nvmem_cell *cell = nvmem_cell_get(&pdev->dev, "gpu_model");
	void *buf;
	int val = 0;
	size_t len;

	if (IS_ERR(cell))
		return PTR_ERR(cell);

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf))
		return PTR_ERR(buf);

	memcpy(&val, buf, min(len, sizeof(val)));
	kfree(buf);

	return val;
}

static struct device_node *
adreno_get_gpu_model_node(struct platform_device *pdev)
{
	struct device_node *node, *child;
	int fuse_model = adreno_read_gpu_model_fuse(pdev);

	if (fuse_model < 0)
		return NULL;

	node = of_find_node_by_name(pdev->dev.of_node, "qcom,gpu-models");
	if (node == NULL)
		return NULL;

	for_each_child_of_node(node, child) {
		u32 model;

		if (of_property_read_u32(child, "qcom,gpu-model-id", &model))
			continue;

		if (model == fuse_model) {
			of_node_put(node);
			return child;
		}
	}

	of_node_put(node);

	return NULL;
}

const char *adreno_get_gpu_model(struct kgsl_device *device)
{
	struct device_node *node;
	static char gpu_model[32];
	const char *model;
	int ret;

	if (strlen(gpu_model))
		return gpu_model;

	node = adreno_get_gpu_model_node(device->pdev);
	if (!node)
		node = of_node_get(device->pdev->dev.of_node);

	ret = of_property_read_string(node, "qcom,gpu-model", &model);
	of_node_put(node);

	if (!ret)
		goto done;

	model = socinfo_get_partinfo_part_name(SOCINFO_PART_GPU);
	if (model)
		goto done;

	scnprintf(gpu_model, sizeof(gpu_model), "Adreno%u%u%uv%u",
		(u32)ADRENO_CHIPID_CORE(ADRENO_DEVICE(device)->chipid),
		(u32)ADRENO_CHIPID_MAJOR(ADRENO_DEVICE(device)->chipid),
		(u32)ADRENO_CHIPID_MINOR(ADRENO_DEVICE(device)->chipid),
		(u32)ADRENO_CHIPID_PATCH(ADRENO_DEVICE(device)->chipid) + 1);

	return gpu_model;

done:
	strscpy(gpu_model, model, sizeof(gpu_model));
	return gpu_model;
}

static u32 adreno_get_vk_device_id(struct kgsl_device *device)
{
	struct device_node *node;
	static u32 device_id;
	u32 vk_id;
	int ret;

	if (device_id)
		return device_id;

	node = adreno_get_gpu_model_node(device->pdev);
	if (!node)
		node = of_node_get(device->pdev->dev.of_node);

	ret = of_property_read_u32(node, "qcom,vk-device-id", &device_id);
	of_node_put(node);
	if (!ret)
		return device_id;

	vk_id = socinfo_get_partinfo_vulkan_id(SOCINFO_PART_GPU);
	device_id = vk_id ? vk_id : ADRENO_DEVICE(device)->chipid;

	return device_id;
}

#if IS_ENABLED(CONFIG_QCOM_LLCC)
static int adreno_probe_llcc(struct adreno_device *adreno_dev,
		struct platform_device *pdev)
{
	int ret;

	/* Get the system cache slice descriptor for GPU */
	adreno_dev->gpu_llc_slice = llcc_slice_getd(LLCC_GPU);
	ret = PTR_ERR_OR_ZERO(adreno_dev->gpu_llc_slice);

	if (ret) {
		/* Propagate EPROBE_DEFER back to the probe function */
		if (ret == -EPROBE_DEFER)
			return ret;

		if (ret != -ENOENT)
			dev_warn(&pdev->dev,
				"Unable to get the GPU LLC slice: %d\n", ret);
	} else
		adreno_dev->gpu_llc_slice_enable = true;

	/* Get the system cache slice descriptor for GPU pagetables */
	adreno_dev->gpuhtw_llc_slice = llcc_slice_getd(LLCC_GPUHTW);
	ret = PTR_ERR_OR_ZERO(adreno_dev->gpuhtw_llc_slice);
	if (ret) {
		if (ret == -EPROBE_DEFER) {
			llcc_slice_putd(adreno_dev->gpu_llc_slice);
			return ret;
		}

		if (ret != -ENOENT)
			dev_warn(&pdev->dev,
				"Unable to get GPU HTW LLC slice: %d\n", ret);
	} else
		adreno_dev->gpuhtw_llc_slice_enable = true;

	return 0;
}
#else
static int adreno_probe_llcc(struct adreno_device *adreno_dev,
		struct platform_device *pdev)
{
	return 0;
}
#endif

const char *hfi_feature_to_string(u32 feature)
{
	switch (feature) {
	case HFI_FEATURE_HWSCHED:
		return "HWSCHED";
	case HFI_FEATURE_PREEMPTION:
		return "PREEMPTION";
	case HFI_FEATURE_LM:
		return "LM";
	case HFI_FEATURE_IFPC:
		return "IFPC";
	case HFI_FEATURE_BCL:
		return "BCL";
	case HFI_FEATURE_ACD:
		return "ACD";
	case HFI_FEATURE_KPROF:
		return "KPROF";
	case HFI_FEATURE_BAIL_OUT_TIMER:
		return "BAIL_OUT_TIMER";
	case HFI_FEATURE_GMU_STATS:
		return "GMU_STATS";
	case HFI_FEATURE_CLX:
		return "CLX";
	case HFI_FEATURE_LSR:
		return "LSR";
	case HFI_FEATURE_LPAC:
		return "LPAC";
	case HFI_FEATURE_HW_FENCE:
		return "HW_FENCE";
	case HFI_FEATURE_PERF_NORETAIN:
		return "PERF_NORETAIN";
	case HFI_FEATURE_DMS:
		return "DMS";
	case HFI_FEATURE_AQE:
		return "AQE";
	}
	return "unknown";
}

static void adreno_regmap_op_preaccess(struct kgsl_regmap_region *region)
{
	struct kgsl_device *device = region->priv;
	/*
	 * kgsl panic notifier will be called in atomic context to get
	 * GPU snapshot. Also panic handler will skip snapshot dumping
	 * incase GPU is in SLUMBER state. So we can safely ignore the
	 * kgsl_pre_hwaccess().
	 */
	if (!device->snapshot_atomic && !in_interrupt())
		kgsl_pre_hwaccess(device);
}

static const struct kgsl_regmap_ops adreno_regmap_ops = {
	.preaccess = adreno_regmap_op_preaccess,
};

static const struct kgsl_functable adreno_functable;

static void adreno_setup_device(struct adreno_device *adreno_dev)
{
	u32 i;

	adreno_dev->dev.name = "kgsl-3d0";
	adreno_dev->dev.ftbl = &adreno_functable;

	init_completion(&adreno_dev->dev.hwaccess_gate);
	init_completion(&adreno_dev->dev.halt_gate);
	init_completion(&adreno_dev->suspend_recovery_gate);
	complete_all(&adreno_dev->suspend_recovery_gate);

	idr_init(&adreno_dev->dev.context_idr);

	mutex_init(&adreno_dev->dev.mutex);
	mutex_init(&adreno_dev->dev.file_mutex);
	mutex_init(&adreno_dev->fault_recovery_mutex);
	mutex_init(&adreno_dev->dcvs_tuning_mutex);
	INIT_LIST_HEAD(&adreno_dev->dev.globals);

	/* Set the fault tolerance policy to replay, skip, throttle */
	adreno_dev->ft_policy = BIT(KGSL_FT_REPLAY) |
		BIT(KGSL_FT_SKIPCMD) | BIT(KGSL_FT_THROTTLE);

	/* Enable command timeouts by default */
	adreno_dev->long_ib_detect = true;

	INIT_WORK(&adreno_dev->input_work, adreno_input_work);

	INIT_LIST_HEAD(&adreno_dev->active_list);
	spin_lock_init(&adreno_dev->active_list_lock);
	rwlock_init(&adreno_dev->fault_stats_lock);

	for (i = 0; i < ARRAY_SIZE(adreno_dev->ringbuffers); i++) {
		struct adreno_ringbuffer *rb = &adreno_dev->ringbuffers[i];

		INIT_LIST_HEAD(&rb->events.group);
	}

	/*
	 * Some GPUs needs specific alignment for UCHE GMEM base address.
	 * Configure UCHE GMEM base based on GMEM size and align it accordingly.
	 * This needs to be done based on GMEM size to avoid overlap between
	 * RB and UCHE GMEM range.
	 */
	if (adreno_dev->gpucore->uche_gmem_alignment)
		adreno_dev->uche_gmem_base =
			ALIGN(adreno_dev->gpucore->gmem_size,
				adreno_dev->gpucore->uche_gmem_alignment);
}

static const struct of_device_id adreno_component_match[] = {
	{ .compatible = "qcom,gen8-gmu" },
	{ .compatible = "qcom,gen7-gmu" },
	{ .compatible = "qcom,gpu-gmu" },
	{ .compatible = "qcom,gpu-rgmu" },
	{ .compatible = "qcom,kgsl-smmu-v2" },
	{ .compatible = "qcom,smmu-kgsl-cb" },
	{},
};

static int adreno_irq_setup(struct platform_device *pdev,
		struct adreno_device *adreno_dev)
{
	if (!adreno_dev->irq_mask)
		return 0;

	return kgsl_request_irq(pdev, "kgsl_3d0_irq", adreno_irq_handler, KGSL_DEVICE(adreno_dev));
}

static int adreno_pm_notifier(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct adreno_device *adreno_dev = container_of(nb, struct adreno_device, pm_nb);
	struct kgsl_pwrctrl *pwr = &adreno_dev->dev.pwrctrl;
	struct generic_pm_domain *pd = NULL;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	if ((event != PM_SUSPEND_PREPARE) && (event != PM_POST_SUSPEND))
		return NOTIFY_DONE;

	if (pwr->gx_pd) {
		pd = container_of(pwr->gx_pd->pm_domain, struct generic_pm_domain, domain);

		if (pd->prepared_count) {
			dev_err_ratelimited(device->dev,
				"unexpected gx pd prepared_count:%d event:%lu\n",
				pd->prepared_count, event);
			return NOTIFY_BAD;
		}
	}

	if (pwr->gmu_cx_pd) {
		pd = container_of(pwr->gmu_cx_pd->pm_domain, struct generic_pm_domain, domain);

		if (pd->prepared_count) {
			dev_err_ratelimited(device->dev,
				"unexpected cx pd prepared_count:%d event:%lu\n",
				pd->prepared_count, event);
			return NOTIFY_BAD;
		}
	}

	if (event == PM_SUSPEND_PREPARE) {
		/*
		 * In presence of a hardware fault, cancel system suspend (by returning NOTIFY_BAD)
		 * here to make sure system suspend doesn't increment the pd prepared_count. This
		 * ensures that cx and gx gdscs can be toggled successfully during fault recovery.
		 */
		if (adreno_gpu_fault(adreno_dev)) {
			dev_err_ratelimited(device->dev, "cancelling suspend because of fault\n");
			complete_all(&adreno_dev->suspend_recovery_gate);
			adreno_scheduler_queue(adreno_dev);
			return NOTIFY_BAD;
		}

		reinit_completion(&adreno_dev->suspend_recovery_gate);
		return NOTIFY_DONE;
	}

	/*
	 * We get PM_POST_SUSPEND if we failed kgsl suspend in the presence of a hardware fault,
	 * or when system resume finishes. In either case, this means the system has come out of
	 * suspend and has put back the power domain prepared_count. This means we are safe to
	 * perform fault recovery.
	 */
	if (event == PM_POST_SUSPEND) {
		complete_all(&adreno_dev->suspend_recovery_gate);
		/*
		 * Queue the gpu scheduler to proceed with fault recovery in case there was a
		 * fault
		 */
		adreno_scheduler_queue(adreno_dev);
	}

	return NOTIFY_DONE;
}

int adreno_device_probe(struct platform_device *pdev,
		struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct device *dev = &pdev->dev;
	unsigned int priv = 0;
	int status;
	u32 size;

	KGSL_BOOT_MARKER("GPU Init");

	/* Initialize the adreno device structure */
	adreno_setup_device(adreno_dev);

	dev_set_drvdata(dev, device);

	device->pdev = pdev;

	adreno_update_soc_hw_revision_quirks(adreno_dev, pdev);

	status = adreno_read_speed_bin(pdev);
	if (status < 0)
		goto err;

	device->speed_bin = status;

	adreno_read_soc_code(device);

	status = adreno_of_get_power(adreno_dev, pdev);
	if (status)
		goto err;

	status = kgsl_bus_init(device, pdev);
	if (status)
		goto err;

	status = kgsl_regmap_init(pdev, &device->regmap, "kgsl_3d0_reg_memory",
		&adreno_regmap_ops, device);
	if (status)
		goto err_bus_close;

	/*
	 * The SMMU APIs use unsigned long for virtual addresses which means
	 * that we cannot use 64 bit virtual addresses on a 32 bit kernel even
	 * though the hardware and the rest of the KGSL driver supports it.
	 */
	if (adreno_support_64bit(adreno_dev))
		kgsl_mmu_set_feature(device, KGSL_MMU_64BIT);

	/*
	 * Set the SMMU aperture on A6XX/Gen7 targets to use per-process
	 * pagetables.
	 */
	if (ADRENO_GPUREV(adreno_dev) >= 600)
		kgsl_mmu_set_feature(device, KGSL_MMU_SMMU_APERTURE);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_IOCOHERENT))
		kgsl_mmu_set_feature(device, KGSL_MMU_IO_COHERENT);

	/*
	 * Support VBOs on hardware where HLOS has access to PRR registers
	 * configuration.
	 */
	if (!adreno_is_a650(adreno_dev))
		kgsl_mmu_set_feature(device, KGSL_MMU_SUPPORT_VBO);

	if (adreno_preemption_enable)
		adreno_dev->preempt_override = true;

	device->pwrctrl.bus_width = adreno_dev->gpucore->bus_width;

	device->mmu.secured = (IS_ENABLED(CONFIG_QCOM_SECURE_BUFFER) &&
		ADRENO_FEATURE(adreno_dev, ADRENO_CONTENT_PROTECTION));

	/* Probe the LLCC - this could return -EPROBE_DEFER */
	status = adreno_probe_llcc(adreno_dev, pdev);
	if (status)
		goto err_bus_close;

	/*
	 * IF the GPU HTW slice was successsful set the MMU feature so the
	 * domain can set the appropriate attributes
	 */
	if (!IS_ERR_OR_NULL(adreno_dev->gpuhtw_llc_slice))
		kgsl_mmu_set_feature(device, KGSL_MMU_LLCC_ENABLE);

	/*
	 * Force no write allocate for A5x, A6x and all gen7 targets
	 * except gen_7_9_x and gen_7_14_0_family. gen_7_9_x and gen_7_14_0_family
	 * use write allocate.
	 */
	if (adreno_is_a5xx(adreno_dev) || adreno_is_a6xx(adreno_dev) ||
		(adreno_is_gen7(adreno_dev) && !adreno_is_gen7_9_x(adreno_dev) &&
		!adreno_is_gen7_14_0_family(adreno_dev)))
		kgsl_mmu_set_feature(device, KGSL_MMU_FORCE_LLCC_NWA);

	 /* Bind the components before doing the KGSL platform probe. */
	status = component_bind_all(dev, NULL);
	if (status)
		goto err_remove_llcc;

	status = adreno_irq_setup(pdev, adreno_dev);
	if (status < 0)
		goto err_unbind;

	device->pwrctrl.interrupt_num = status;

	device->freq_limiter_intr_num = kgsl_request_irq_optional(pdev, "freq_limiter_irq",
				adreno_freq_limiter_irq_handler, device);

	device->freq_limiter_irq_clear =
		devm_reset_control_get(&pdev->dev, "freq_limiter_irq_clear");

	status = kgsl_device_platform_probe(device);
	if (status)
		goto err_unbind;

	adreno_fence_trace_array_init(device);

	/* Add CX_DBGC block to the regmap*/
	kgsl_regmap_add_region(&device->regmap, pdev, "cx_dbgc", NULL, NULL);

	/* Probe for the optional CX_MISC block */
	kgsl_regmap_add_region(&device->regmap, pdev, "cx_misc", NULL, NULL);

	if (kgsl_regmap_add_region(&device->regmap, pdev, "isense_cntl", NULL, NULL) == 0)
		adreno_dev->isense_reg_mapped = true;

	/* Allocate the memstore for storing timestamps and other useful info */

	if (ADRENO_FEATURE(adreno_dev, ADRENO_APRIV))
		priv |= KGSL_MEMDESC_PRIVILEGED;

	device->memstore = kgsl_allocate_global(device,
		KGSL_MEMSTORE_SIZE, 0, 0, priv, "memstore");

	status = PTR_ERR_OR_ZERO(device->memstore);
	if (status) {
		trace_array_put(device->fence_trace_array);
		kgsl_device_platform_remove(device);
		goto err_unbind;
	}

	/* Initialize the snapshot engine */
	size = adreno_dev->gpucore->snapshot_size;

	/*
	 * Use a default size if one wasn't specified, but print a warning so
	 * the developer knows to fix it
	 */

	if (WARN(!size, "The snapshot size was not specified in the gpucore\n"))
		size = SZ_1M;

	kgsl_device_snapshot_probe(device, size);

	adreno_debugfs_init(adreno_dev);
	adreno_profile_init(adreno_dev);

	adreno_dev->perfcounter = false;

	adreno_sysfs_init(adreno_dev);

	/* Ignore return value, as driver can still function without pwrscale enabled */
	kgsl_pwrscale_init(device, pdev, CONFIG_QCOM_ADRENO_DEFAULT_GOVERNOR);

	if (ADRENO_FEATURE(adreno_dev, ADRENO_L3_VOTE))
		device->l3_vote = true;

#ifdef CONFIG_INPUT

	if (!of_property_read_bool(pdev->dev.of_node,
			"qcom,disable-wake-on-touch")) {
		adreno_input_handler.private = device;
		/*
		 * It isn't fatal if we cannot register the input handler.  Sad,
		 * perhaps, but not fatal
		 */
		if (input_register_handler(&adreno_input_handler)) {
			adreno_input_handler.private = NULL;
			dev_err(device->dev,
				     "Unable to register the input handler\n");
		}
	}
#endif

	/*
	 * With power domains, we cannot perform recovery during a concurrent system suspend because
	 * system suspend path increments power domain prepared_count, which prevents successful
	 * toggling of the power domain gdsc while system is in suspend path. Hence, get
	 * notifications when system has come out of suspend completely, so that we can perform
	 * fault recovery.
	 */
	if (device->pwrctrl.gx_pd || device->pwrctrl.gmu_cx_pd) {
		adreno_dev->pm_nb.notifier_call = adreno_pm_notifier;
		register_pm_notifier(&adreno_dev->pm_nb);
	}

	kgsl_qcom_va_md_register(device);

	KGSL_BOOT_MARKER("GPU Ready");

	return 0;

err_unbind:
	component_unbind_all(dev, NULL);

err_remove_llcc:
	if (!IS_ERR_OR_NULL(adreno_dev->gpu_llc_slice))
		llcc_slice_putd(adreno_dev->gpu_llc_slice);

	if (!IS_ERR_OR_NULL(adreno_dev->gpuhtw_llc_slice))
		llcc_slice_putd(adreno_dev->gpuhtw_llc_slice);

err_bus_close:
	kgsl_bus_close(device);

err:
	device->pdev = NULL;
	dev_err_probe(&pdev->dev, status, "adreno device probe failed\n");
	return status;
}

static int adreno_bind(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct adreno_gpu_core *gpucore;
	int ret;
	u32 chipid;

	gpucore = adreno_identify_gpu(pdev, &chipid);
	if (IS_ERR(gpucore))
		return PTR_ERR(gpucore);

	ret = gpucore->gpudev->probe(pdev, chipid, gpucore);

	if (!ret) {
		struct kgsl_device *device = dev_get_drvdata(dev);

		device->pdev_loaded = true;
		srcu_init_notifier_head(&device->nh);
	} else {
		/*
		 * Handle resource clean up through unbind, instead of a
		 * lengthy goto error path.
		 */
		adreno_unbind(dev);
	}

	return ret;
}

static void adreno_unbind(struct device *dev)
{
	struct adreno_device *adreno_dev;
	struct kgsl_device *device;
	const struct adreno_gpudev *gpudev;

	device = dev_get_drvdata(dev);
	if (!device)
		return;

	/* Return if cleanup happens in adreno_device_probe */
	if (!device->pdev)
		return;

	if (device->pdev_loaded) {
		srcu_cleanup_notifier_head(&device->nh);
		device->pdev_loaded = false;
	}

	adreno_dev = ADRENO_DEVICE(device);
	gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	trace_array_put(device->fence_trace_array);

	if (gpudev->remove != NULL)
		gpudev->remove(adreno_dev);

#ifdef CONFIG_INPUT
	if (adreno_input_handler.private)
		input_unregister_handler(&adreno_input_handler);
#endif

	kgsl_qcom_va_md_unregister(device);
	adreno_coresight_remove(adreno_dev);
	adreno_profile_close(adreno_dev);

	/* Release the system cache slice descriptor */
	if (!IS_ERR_OR_NULL(adreno_dev->gpu_llc_slice))
		llcc_slice_putd(adreno_dev->gpu_llc_slice);

	if (!IS_ERR_OR_NULL(adreno_dev->gpuhtw_llc_slice))
		llcc_slice_putd(adreno_dev->gpuhtw_llc_slice);

	kgsl_pwrscale_close(device);

	if (adreno_dev->dispatch_ops && adreno_dev->dispatch_ops->close)
		adreno_dev->dispatch_ops->close(adreno_dev);

	kgsl_device_platform_remove(device);

	component_unbind_all(dev, NULL);

	kgsl_bus_close(device);
	device->pdev = NULL;

	if (device->num_l3_pwrlevels != 0)
		qcom_dcvs_unregister_voter(KGSL_L3_DEVICE, DCVS_L3,
			DCVS_SLOW_PATH);

	clear_bit(ADRENO_DEVICE_PWRON_FIXUP, &adreno_dev->priv);
	clear_bit(ADRENO_DEVICE_INITIALIZED, &adreno_dev->priv);
}

static void adreno_resume(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	if (device->state == KGSL_STATE_SUSPEND) {
		adreno_put_gpu_halt(adreno_dev);
		kgsl_pwrctrl_change_state(device, KGSL_STATE_SLUMBER);
	} else if (device->state != KGSL_STATE_INIT) {
		/*
		 * This is an error situation so wait for the device to idle and
		 * then put the device in SLUMBER state.  This will get us to
		 * the right place when we resume.
		 */
		if (device->state == KGSL_STATE_ACTIVE)
			adreno_idle(device);
		kgsl_pwrctrl_change_state(device, KGSL_STATE_SLUMBER);
		dev_err(device->dev, "resume invoked without a suspend\n");
	}
}

static int adreno_pm_resume(struct device *dev)
{
	struct kgsl_device *device = dev_get_drvdata(dev);
	struct adreno_device *adreno_dev;
	const struct adreno_power_ops *ops;

	if (!device)
		return 0;

	adreno_dev = ADRENO_DEVICE(device);
	ops = ADRENO_POWER_OPS(adreno_dev);

#if IS_ENABLED(CONFIG_DEEPSLEEP)
	if (pm_suspend_via_firmware()) {
		struct kgsl_iommu *iommu = &device->mmu.iommu;
		int status = kgsl_set_smmu_aperture(device, &iommu->user_context);

		if (status)
			return status;

		status = kgsl_set_smmu_lpac_aperture(device, &iommu->lpac_context);
		if (status < 0)
			return status;
	}
#endif

	mutex_lock(&device->mutex);
	ops->pm_resume(adreno_dev);
	mutex_unlock(&device->mutex);

	kgsl_reclaim_start();
	return 0;
}

static int adreno_suspend(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int status = kgsl_pwrctrl_change_state(device, KGSL_STATE_SUSPEND);

	if (!status && device->state == KGSL_STATE_SUSPEND)
		adreno_get_gpu_halt(adreno_dev);

	return status;
}

static int adreno_pm_suspend(struct device *dev)
{
	struct kgsl_device *device = dev_get_drvdata(dev);
	struct adreno_device *adreno_dev;
	const struct adreno_power_ops *ops;
	int status;

	if (!device)
		return 0;

	adreno_dev = ADRENO_DEVICE(device);
	ops = ADRENO_POWER_OPS(adreno_dev);

	/* Return early if fault recovery is in progress */
	if (!mutex_trylock(&adreno_dev->fault_recovery_mutex))
		return -EDEADLK;

	mutex_lock(&device->mutex);
	status = ops->pm_suspend(adreno_dev);

#if IS_ENABLED(CONFIG_DEEPSLEEP)
	if (!status && pm_suspend_via_firmware())
		adreno_zap_shader_unload(adreno_dev);
#endif

	mutex_unlock(&device->mutex);
	mutex_unlock(&adreno_dev->fault_recovery_mutex);

	if (status)
		return status;

	/*
	 * When the device enters in suspend state, the CX can be collapsed causing
	 * the GPU CX timer to pause. Clear the ADRENO_DEVICE_CX_TIMER_INITIALIZED
	 * flag to ensure that the CX timer is reseeded during resume.
	 */
	clear_bit(ADRENO_DEVICE_CX_TIMER_INITIALIZED, &adreno_dev->priv);
	kgsl_reclaim_close();
	kthread_flush_worker(device->events_worker);
	flush_workqueue(kgsl_driver.lockless_workqueue);

	return status;
}

void adreno_create_profile_buffer(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	unsigned int priv = 0;

	if (ADRENO_FEATURE(adreno_dev, ADRENO_APRIV))
		priv = KGSL_MEMDESC_PRIVILEGED;

	adreno_allocate_global(device, &adreno_dev->profile_buffer,
		PAGE_SIZE, 0, 0, priv, "alwayson");

	adreno_dev->profile_index = 0;

	if (!IS_ERR(adreno_dev->profile_buffer))
		set_bit(ADRENO_DEVICE_DRAWOBJ_PROFILE,
			&adreno_dev->priv);
}

static int adreno_init(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int ret;

	ret = kgsl_pwrctrl_change_state(device, KGSL_STATE_INIT);
	if (ret)
		return ret;

	/*
	 * initialization only needs to be done once initially until
	 * device is shutdown
	 */
	if (test_bit(ADRENO_DEVICE_INITIALIZED, &adreno_dev->priv))
		return 0;

	ret = gpudev->init(adreno_dev);
	if (ret)
		return ret;

	set_bit(ADRENO_DEVICE_INITIALIZED, &adreno_dev->priv);

	return 0;
}

static bool gdscs_left_on(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (gmu_core_gpmu_isenabled(device))
		return false;

	if (pwr->cx_regulator)
		return regulator_is_enabled(pwr->cx_regulator);

	if (pwr->gx_regulator)
		return regulator_is_enabled(pwr->gx_regulator);

	if (pwr->gmu_cx_pd)
		return kgsl_genpd_is_enabled(pwr->gmu_cx_pd);

	if (pwr->gx_pd)
		return kgsl_genpd_is_enabled(pwr->gx_pd);

	return false;
}

void adreno_set_active_ctxs_null(struct adreno_device *adreno_dev)
{
	int i;
	struct adreno_ringbuffer *rb;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		if (rb->drawctxt_active)
			kgsl_context_put(&(rb->drawctxt_active->base));
		rb->drawctxt_active = NULL;

		kgsl_sharedmem_writel(device->scratch,
			SCRATCH_RB_OFFSET(rb->id, current_rb_ptname),
			0);
	}
}

static int adreno_open(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret;

	/*
	 * active_cnt special case: we are starting up for the first
	 * time, so use this sequence instead of the kgsl_pwrctrl_wake()
	 * which will be called by adreno_active_count_get().
	 */
	atomic_inc(&device->active_cnt);

	memset(device->memstore->hostptr, 0, device->memstore->size);

	ret = adreno_init(device);
	if (ret)
		goto err;

	ret = adreno_start(device, 0);
	if (ret)
		goto err;

	complete_all(&device->hwaccess_gate);
	kgsl_pwrctrl_change_state(device, KGSL_STATE_ACTIVE);
	adreno_active_count_put(adreno_dev);

	return 0;
err:
	kgsl_pwrctrl_change_state(device, KGSL_STATE_INIT);
	atomic_dec(&device->active_cnt);

	return ret;
}

static int adreno_first_open(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_power_ops *ops = ADRENO_POWER_OPS(adreno_dev);

	if (!device->pdev_loaded)
		return -ENODEV;

	return ops->first_open(adreno_dev);
}

static int adreno_close(struct adreno_device *adreno_dev)
{
	return kgsl_pwrctrl_change_state(KGSL_DEVICE(adreno_dev),
			KGSL_STATE_INIT);
}

static int adreno_last_close(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_power_ops *ops = ADRENO_POWER_OPS(adreno_dev);

	/*
	 * Wait up to 1 second for the active count to go low
	 * and then start complaining about it
	 */
	if (kgsl_active_count_wait(device, 0, HZ)) {
		dev_err(device->dev,
			"Waiting for the active count to become 0\n");

		while (kgsl_active_count_wait(device, 0, HZ))
			dev_err(device->dev,
				"Still waiting for the active count\n");
	}

	return ops->last_close(adreno_dev);
}

static int adreno_pwrctrl_active_count_get(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret = 0;

	if (WARN_ON(!mutex_is_locked(&device->mutex)))
		return -EINVAL;

	if ((atomic_read(&device->active_cnt) == 0) &&
		(device->state != KGSL_STATE_ACTIVE)) {
		mutex_unlock(&device->mutex);
		wait_for_completion(&device->hwaccess_gate);
		mutex_lock(&device->mutex);
		device->pwrctrl.superfast = true;
		ret = kgsl_pwrctrl_change_state(device, KGSL_STATE_ACTIVE);
	}
	if (ret == 0)
		atomic_inc(&device->active_cnt);
	trace_kgsl_active_count(device,
		(unsigned long) __builtin_return_address(0));
	return ret;
}

void adreno_active_count_put(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	if (WARN_ON(!mutex_is_locked(&device->mutex)))
		return;

	if (WARN(atomic_read(&device->active_cnt) == 0,
			"Unbalanced get/put calls to KGSL active count\n"))
		return;

	if (atomic_dec_and_test(&device->active_cnt)) {
		kgsl_pwrscale_update_stats(device);
		kgsl_pwrscale_update(device);

		kgsl_start_idle_timer(device);
	}

	trace_kgsl_active_count(device,
		(unsigned long) __builtin_return_address(0));

	wake_up(&device->active_cnt_wq);
}

int adreno_active_count_get(struct adreno_device *adreno_dev)
{
	const struct adreno_power_ops *ops = ADRENO_POWER_OPS(adreno_dev);

	return ops->active_count_get(adreno_dev);
}

void adreno_get_bus_counters(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int ret = 0;

	if (!device->pwrctrl.bus_control)
		return;

	/* VBIF waiting for RAM */
	ret |= adreno_perfcounter_kernel_get(adreno_dev,
		KGSL_PERFCOUNTER_GROUP_VBIF_PWR, 0,
		&adreno_dev->starved_ram_lo, NULL);

	/* Target has GBIF */
	if (adreno_is_gen8(adreno_dev) || adreno_is_gen7(adreno_dev) ||
		(adreno_is_a6xx(adreno_dev) && !adreno_is_a630(adreno_dev))) {
		ret |= adreno_perfcounter_kernel_get(adreno_dev,
			KGSL_PERFCOUNTER_GROUP_VBIF_PWR, 1,
			&adreno_dev->starved_ram_lo_ch1, NULL);

		ret |= adreno_perfcounter_kernel_get(adreno_dev,
			KGSL_PERFCOUNTER_GROUP_VBIF,
			GBIF_AXI0_READ_DATA_TOTAL_BEATS,
			&adreno_dev->ram_cycles_lo, NULL);

		ret |= adreno_perfcounter_kernel_get(adreno_dev,
			KGSL_PERFCOUNTER_GROUP_VBIF,
			GBIF_AXI1_READ_DATA_TOTAL_BEATS,
			&adreno_dev->ram_cycles_lo_ch1_read, NULL);

		ret |= adreno_perfcounter_kernel_get(adreno_dev,
			KGSL_PERFCOUNTER_GROUP_VBIF,
			GBIF_AXI0_WRITE_DATA_TOTAL_BEATS,
			&adreno_dev->ram_cycles_lo_ch0_write, NULL);

		ret |= adreno_perfcounter_kernel_get(adreno_dev,
			KGSL_PERFCOUNTER_GROUP_VBIF,
			GBIF_AXI1_WRITE_DATA_TOTAL_BEATS,
			&adreno_dev->ram_cycles_lo_ch1_write, NULL);
	} else {
		/* VBIF DDR cycles */
		ret |= adreno_perfcounter_kernel_get(adreno_dev,
			KGSL_PERFCOUNTER_GROUP_VBIF,
			VBIF_AXI_TOTAL_BEATS,
			&adreno_dev->ram_cycles_lo, NULL);
	}

	if (ret)
		dev_err(KGSL_DEVICE(adreno_dev)->dev,
			"Unable to get perf counters for bus DCVS\n");
}

#define ADRENO_AHB_MIN_TIMEOUT_VAL_USEC 1000

u32 adreno_get_ahb_timeout_val(struct adreno_device *adreno_dev, u32 noc_timeout_us)
{
	u64 cycles, hub_clk_freq = adreno_dev->gmu_hub_clk_freq;
	u32 timeout_val;

	if (!noc_timeout_us)
		return 0;

	do_div(hub_clk_freq, HZ_PER_MHZ);
	cycles = hub_clk_freq * noc_timeout_us;

	/*
	 * Get max possible AHB timeout value which is less than the GPU NOC timeout value.
	 * When cycles are exact power of two, the calculated AHB timeout value will be same
	 * as GPU config NOC timeout. Just reduce one cycle to make sure we do not program AHB
	 * timeout same as GPU config NOC timeout.
	 */
	if (is_power_of_2(cycles))
		cycles -= 1;

	timeout_val = ilog2(cycles);

	/*
	 * Make sure, AHB timeout value fits into bit fields and it is not too low
	 * which can cause false timeouts.
	 */
	if ((timeout_val > GENMASK(4, 0)) ||
		((ADRENO_AHB_MIN_TIMEOUT_VAL_USEC * hub_clk_freq) > (1 << timeout_val))) {
		dev_warn(adreno_dev->dev.dev, "Invalid AHB timeout_val %u\n", timeout_val);
		return 0;
	}

	/*
	 * Return (timeout_val - 1). Based on timeout_val programmed, a timeout will occur if
	 * an AHB transaction is not completed in 2 ^ (timeout_val + 1) cycles.
	 */
	return (timeout_val - 1);
}

/**
 * _adreno_start - Power up the GPU and prepare to accept commands
 * @adreno_dev: Pointer to an adreno_device structure
 *
 * The core function that powers up and initalizes the GPU.  This function is
 * called at init and after coming out of SLUMBER
 */
static int _adreno_start(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int status;
	unsigned int state = device->state;
	bool gdsc_left_on;

	/* make sure ADRENO_DEVICE_STARTED is not set here */
	WARN_ON(test_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv));

	gdsc_left_on = gdscs_left_on(device);

	/* Clear any GPU faults that might have been left over */
	adreno_clear_gpu_fault(adreno_dev);

	/* Put the GPU in a responsive state */
	status = kgsl_pwrctrl_change_state(device, KGSL_STATE_AWARE);
	if (status)
		goto error_pwr_off;

	/* Set any stale active contexts to NULL */
	adreno_set_active_ctxs_null(adreno_dev);

	/* Set the bit to indicate that we've just powered on */
	set_bit(ADRENO_DEVICE_PWRON, &adreno_dev->priv);

	/* Clear the busy_data stats - we're starting over from scratch */
	memset(&adreno_dev->busy_data, 0, sizeof(adreno_dev->busy_data));

	/* Soft reset the GPU if a regulator is stuck on*/
	if (gdsc_left_on)
		_soft_reset(adreno_dev);

	/* Start the GPU */
	status = gpudev->start(adreno_dev);
	if (status)
		goto error_pwr_off;

	/* Re-initialize the coresight registers if applicable */
	adreno_coresight_start(adreno_dev);

	adreno_irqctrl(adreno_dev, 1);

	adreno_perfcounter_start(adreno_dev);

	/* Clear FSR here in case it is set from a previous pagefault */
	kgsl_mmu_clear_fsr(&device->mmu);

	status = gpudev->rb_start(adreno_dev);
	if (status)
		goto error_pwr_off;

	/*
	 * At this point it is safe to assume that we recovered. Setting
	 * this field allows us to take a new snapshot for the next failure
	 * if we are prioritizing the first unrecoverable snapshot.
	 */
	if (device->snapshot)
		device->snapshot->recovered = true;

	/* Start the dispatcher */
	adreno_dispatcher_start(device);

	device->reset_counter++;

	set_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv);

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

	return 0;

error_pwr_off:
	/* set the state back to original state */
	kgsl_pwrctrl_change_state(device, state);

	return status;
}

/**
 * adreno_start() - Power up and initialize the GPU
 * @device: Pointer to the KGSL device to power up
 * @priority:  Boolean flag to specify of the start should be scheduled in a low
 * latency work queue
 *
 * Power up the GPU and initialize it.  If priority is specified then elevate
 * the thread priority for the duration of the start operation
 */
int adreno_start(struct kgsl_device *device, int priority)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int nice = task_nice(current);
	int ret;

	if (priority && (adreno_wake_nice < nice))
		set_user_nice(current, adreno_wake_nice);

	ret = _adreno_start(adreno_dev);

	if (priority)
		set_user_nice(current, nice);

	return ret;
}

static int adreno_stop(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int error = 0;

	if (!test_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv))
		return 0;

	kgsl_pwrscale_update_stats(device);

	adreno_irqctrl(adreno_dev, 0);

	/* Save active coresight registers if applicable */
	adreno_coresight_stop(adreno_dev);

	/* Save physical performance counter values before GPU power down*/
	adreno_perfcounter_save(adreno_dev);

	if (gpudev->clear_pending_transactions)
		gpudev->clear_pending_transactions(adreno_dev);

	adreno_dispatcher_stop(adreno_dev);

	adreno_ringbuffer_stop(adreno_dev);

	adreno_llcc_slice_deactivate(adreno_dev);

	adreno_set_active_ctxs_null(adreno_dev);

	clear_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv);

	return error;
}

/**
 * adreno_reset() - Helper function to reset the GPU
 * @device: Pointer to the KGSL device structure for the GPU
 * @fault: Type of fault. Needed to skip soft reset for MMU fault
 *
 * Try to reset the GPU to recover from a fault.  First, try to do a low latency
 * soft reset.  If the soft reset fails for some reason, then bring out the big
 * guns and toggle the footswitch.
 */
int adreno_reset(struct kgsl_device *device, int fault)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int ret = -EINVAL;
	int i;

	if (gpudev->reset)
		return gpudev->reset(adreno_dev);

	/*
	 * Try soft reset first Do not do soft reset for a IOMMU fault (because
	 * the IOMMU hardware needs a reset too)
	 */

	if (!(fault & ADRENO_IOMMU_STALL_ON_PAGE_FAULT))
		ret = adreno_soft_reset(device);

	if (ret) {
		/* If soft reset failed/skipped, then pull the power */
		kgsl_pwrctrl_change_state(device, KGSL_STATE_INIT);
		/* since device is officially off now clear start bit */
		clear_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv);

		/* Try to reset the device */
		ret = adreno_start(device, 0);

		for (i = 0; ret && i < 4; i++) {
			msleep(20);
			ret = adreno_start(device, 0);
		}

		if (ret)
			return ret;

		if (i != 0)
			dev_warn(device->dev,
			      "Device hard reset tried %d tries\n", i);
	}

	/*
	 * If active_cnt is non-zero then the system was active before
	 * going into a reset - put it back in that state
	 */

	if (atomic_read(&device->active_cnt))
		kgsl_pwrctrl_change_state(device, KGSL_STATE_ACTIVE);

	return ret;
}

static int copy_prop(struct kgsl_device_getproperty *param,
		void *src, size_t size)
{
	if (copy_to_user(param->value, src,
		min_t(u32, size, param->sizebytes)))
		return -EFAULT;

	return 0;
}

static int adreno_prop_device_info(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_devinfo devinfo = {
		.device_id = device->id + 1,
		.chip_id = adreno_dev->chipid,
		.mmu_enabled = kgsl_mmu_has_feature(device, KGSL_MMU_PAGED),
		.gmem_gpubaseaddr = 0,
		.gmem_sizebytes = adreno_dev->gpucore->gmem_size,
	};

	return copy_prop(param, &devinfo, sizeof(devinfo));
}

static int adreno_prop_gpu_model(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct kgsl_gpu_model model = {0};

	strscpy(model.gpu_model, adreno_get_gpu_model(device),
			sizeof(model.gpu_model));

	return copy_prop(param, &model, sizeof(model));
}

static int adreno_prop_device_shadow(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct kgsl_shadowprop shadowprop = { 0 };

	if (device->memstore->hostptr) {
		/* Pass a dummy address to identify memstore */
		shadowprop.gpuaddr =  KGSL_MEMSTORE_TOKEN_ADDRESS;
		shadowprop.size = device->memstore->size;

		shadowprop.flags = KGSL_FLAGS_INITIALIZED |
			KGSL_FLAGS_PER_CONTEXT_TIMESTAMPS;
	}

	return copy_prop(param, &shadowprop, sizeof(shadowprop));
}

static int adreno_prop_device_qdss_stm(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct kgsl_qdss_stm_prop qdssprop = {0};

	if (!IS_ERR_OR_NULL(device->qdss_desc)) {
		qdssprop.gpuaddr = device->qdss_desc->gpuaddr;
		qdssprop.size = device->qdss_desc->size;
	}

	return copy_prop(param, &qdssprop, sizeof(qdssprop));
}

static int adreno_prop_device_qtimer(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct kgsl_qtimer_prop qtimerprop = {0};

	if (!IS_ERR_OR_NULL(device->qtimer_desc)) {
		qtimerprop.gpuaddr = device->qtimer_desc->gpuaddr;
		qtimerprop.size = device->qtimer_desc->size;
	}

	return copy_prop(param, &qtimerprop, sizeof(qtimerprop));
}

static int adreno_prop_s32(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	int val = 0;

	if (param->type == KGSL_PROP_MMU_ENABLE)
		val = kgsl_mmu_has_feature(device, KGSL_MMU_PAGED);
	else if (param->type == KGSL_PROP_INTERRUPT_WAITS)
		val = 1;

	return copy_prop(param, &val, sizeof(val));
}

static int adreno_prop_uche_gmem_addr(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	return copy_prop(param, &adreno_dev->uche_gmem_base,
		sizeof(adreno_dev->uche_gmem_base));
}

static int adreno_prop_ucode_version(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_ucode_version ucode = {
		.pfp = adreno_dev->fw[ADRENO_FW_PFP].version,
		.pm4 = adreno_dev->fw[ADRENO_FW_PM4].version,
	};

	return copy_prop(param, &ucode, sizeof(ucode));
}

static int adreno_prop_gaming_bin(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	void *buf;
	size_t len;
	int ret;
	struct nvmem_cell *cell;

	cell = nvmem_cell_get(&device->pdev->dev, "gaming_bin");
	if (IS_ERR(cell))
		return -EINVAL;

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (!IS_ERR(buf)) {
		ret = copy_prop(param, buf, len);
		kfree(buf);
		return ret;
	}

	dev_err(device->dev, "failed to read gaming_bin nvmem cell\n");
	return -EINVAL;
}

static int adreno_prop_u32(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	u32 val = 0;

	if (param->type == KGSL_PROP_HIGHEST_BANK_BIT) {
		val = adreno_dev->highest_bank_bit;
	} else if (param->type == KGSL_PROP_MIN_ACCESS_LENGTH)
		of_property_read_u32(device->pdev->dev.of_node,
			"qcom,min-access-length", &val);
	else if (param->type == KGSL_PROP_UBWC_MODE)
		of_property_read_u32(device->pdev->dev.of_node,
			"qcom,ubwc-mode", &val);
	else if (param->type == KGSL_PROP_DEVICE_BITNESS)
		val = adreno_support_64bit(adreno_dev) ? 48 : 32;
	else if (param->type == KGSL_PROP_SPEED_BIN)
		val = device->speed_bin;
	else if (param->type == KGSL_PROP_VK_DEVICE_ID)
		val = adreno_get_vk_device_id(device);
	else if (param->type == KGSL_PROP_IS_LPAC_ENABLED)
		val = adreno_dev->lpac_enabled ? 1 : 0;
	else if (param->type == KGSL_PROP_IS_RAYTRACING_ENABLED)
		val =  adreno_dev->raytracing_enabled ? 1 : 0;
	else if (param->type == KGSL_PROP_IS_FASTBLEND_ENABLED)
		val = adreno_dev->fastblend_enabled ? 1 : 0;
	else if (param->type == KGSL_PROP_IS_AQE_ENABLED)
		val = ADRENO_FEATURE(adreno_dev, ADRENO_AQE) ? 1 : 0;

	return copy_prop(param, &val, sizeof(val));
}

static int adreno_prop_uche_trap_base(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	u64 val = 0;

	if (!gpudev->get_uche_trap_base)
		return -EINVAL;

	val = gpudev->get_uche_trap_base();

	return copy_prop(param, &val, sizeof(val));
}

static const struct {
	int type;
	int (*func)(struct kgsl_device *device,
		struct kgsl_device_getproperty *param);
} adreno_property_funcs[] = {
	{ KGSL_PROP_DEVICE_INFO, adreno_prop_device_info },
	{ KGSL_PROP_DEVICE_SHADOW, adreno_prop_device_shadow },
	{ KGSL_PROP_DEVICE_QDSS_STM, adreno_prop_device_qdss_stm },
	{ KGSL_PROP_DEVICE_QTIMER, adreno_prop_device_qtimer },
	{ KGSL_PROP_MMU_ENABLE, adreno_prop_s32 },
	{ KGSL_PROP_INTERRUPT_WAITS, adreno_prop_s32 },
	{ KGSL_PROP_UCHE_GMEM_VADDR, adreno_prop_uche_gmem_addr },
	{ KGSL_PROP_UCODE_VERSION, adreno_prop_ucode_version },
	{ KGSL_PROP_HIGHEST_BANK_BIT, adreno_prop_u32 },
	{ KGSL_PROP_MIN_ACCESS_LENGTH, adreno_prop_u32 },
	{ KGSL_PROP_UBWC_MODE, adreno_prop_u32 },
	{ KGSL_PROP_DEVICE_BITNESS, adreno_prop_u32 },
	{ KGSL_PROP_SPEED_BIN, adreno_prop_u32 },
	{ KGSL_PROP_GAMING_BIN, adreno_prop_gaming_bin },
	{ KGSL_PROP_GPU_MODEL, adreno_prop_gpu_model},
	{ KGSL_PROP_VK_DEVICE_ID, adreno_prop_u32},
	{ KGSL_PROP_IS_LPAC_ENABLED, adreno_prop_u32 },
	{ KGSL_PROP_IS_RAYTRACING_ENABLED, adreno_prop_u32},
	{ KGSL_PROP_IS_FASTBLEND_ENABLED, adreno_prop_u32},
	{ KGSL_PROP_UCHE_TRAP_BASE, adreno_prop_uche_trap_base },
	{ KGSL_PROP_IS_AQE_ENABLED, adreno_prop_u32 },
};

static int adreno_getproperty(struct kgsl_device *device,
		struct kgsl_device_getproperty *param)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adreno_property_funcs); i++) {
		if (param->type == adreno_property_funcs[i].type)
			return adreno_property_funcs[i].func(device, param);
	}

	return -ENODEV;
}

static int adreno_query_property_list(struct kgsl_device *device, u32 *list,
		u32 count)
{
	int i;

	if (!list)
		return ARRAY_SIZE(adreno_property_funcs);

	for (i = 0; i < count && i < ARRAY_SIZE(adreno_property_funcs); i++)
		list[i] = adreno_property_funcs[i].type;

	return i;
}

int adreno_set_constraint(struct kgsl_device *device,
				struct kgsl_context *context,
				struct kgsl_device_constraint *constraint)
{
	int status = 0;

	switch (constraint->type) {
	case KGSL_CONSTRAINT_PWRLEVEL: {
		struct kgsl_device_constraint_pwrlevel pwr;

		if (constraint->size != sizeof(pwr)) {
			status = -EINVAL;
			break;
		}

		if (copy_from_user(&pwr,
				(void __user *)constraint->data,
				sizeof(pwr))) {
			status = -EFAULT;
			break;
		}
		if (pwr.level >= KGSL_CONSTRAINT_PWR_MAXLEVELS) {
			status = -EINVAL;
			break;
		}

		context->pwr_constraint.type =
				KGSL_CONSTRAINT_PWRLEVEL;
		context->pwr_constraint.sub_type = pwr.level;
		trace_kgsl_user_pwrlevel_constraint(device,
			context->id,
			context->pwr_constraint.type,
			context->pwr_constraint.sub_type);
		}
		break;
	case KGSL_CONSTRAINT_NONE:
		if (context->pwr_constraint.type == KGSL_CONSTRAINT_PWRLEVEL)
			trace_kgsl_user_pwrlevel_constraint(device,
				context->id,
				KGSL_CONSTRAINT_NONE,
				context->pwr_constraint.sub_type);
		context->pwr_constraint.type = KGSL_CONSTRAINT_NONE;
		break;
	case KGSL_CONSTRAINT_L3_PWRLEVEL: {
		struct kgsl_device_constraint_pwrlevel pwr;

		if (constraint->size != sizeof(pwr)) {
			status = -EINVAL;
			break;
		}

		if (copy_from_user(&pwr, constraint->data, sizeof(pwr))) {
			status = -EFAULT;
			break;
		}

		status = register_l3_voter(device);
		if (status)
			break;

		if (pwr.level >= KGSL_CONSTRAINT_PWR_MAXLEVELS)
			pwr.level = KGSL_CONSTRAINT_PWR_MAXLEVELS - 1;

		context->l3_pwr_constraint.type = KGSL_CONSTRAINT_L3_PWRLEVEL;
		context->l3_pwr_constraint.sub_type = pwr.level;
		trace_kgsl_user_pwrlevel_constraint(device, context->id,
			context->l3_pwr_constraint.type,
			context->l3_pwr_constraint.sub_type);
		}
		break;
	case KGSL_CONSTRAINT_L3_NONE: {
		unsigned int type = context->l3_pwr_constraint.type;

		if (type == KGSL_CONSTRAINT_L3_PWRLEVEL)
			trace_kgsl_user_pwrlevel_constraint(device, context->id,
				KGSL_CONSTRAINT_L3_NONE,
				context->l3_pwr_constraint.sub_type);
		context->l3_pwr_constraint.type = KGSL_CONSTRAINT_L3_NONE;
		}
		break;
	default:
		status = -EINVAL;
		break;
	}

	/* If a new constraint has been set for a context, cancel the old one */
	if ((status == 0) &&
		(context->id == device->pwrctrl.constraint.owner_id)) {
		trace_kgsl_constraint(device, device->pwrctrl.constraint.type,
					device->pwrctrl.active_pwrlevel, 0);
		device->pwrctrl.constraint.type = KGSL_CONSTRAINT_NONE;
	}

	return status;
}

static int adreno_default_setproperty(struct kgsl_device_private *dev_priv,
		u32 type, void __user *value, u32 sizebytes)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	u32 enable;

	if (type != KGSL_PROP_PWRCTRL)
		return -ENODEV;

	if (sizebytes != sizeof(enable))
		return -EINVAL;

	if (copy_from_user(&enable, value, sizeof(enable)))
		return -EFAULT;

	mutex_lock(&device->mutex);

	if (enable) {
		if (gmu_core_isenabled(device))
			clear_bit(GMU_DISABLE_SLUMBER, &device->gmu_core.flags);
		else
			device->pwrctrl.ctrl_flags = 0;

		kgsl_pwrscale_enable(device);
	} else {
		if (gmu_core_isenabled(device)) {
			set_bit(GMU_DISABLE_SLUMBER, &device->gmu_core.flags);

			if (!adreno_active_count_get(adreno_dev))
				adreno_active_count_put(adreno_dev);
		} else {
			kgsl_pwrctrl_change_state(device, KGSL_STATE_ACTIVE);
			device->pwrctrl.ctrl_flags = KGSL_PWR_ON;
		}
		kgsl_pwrscale_disable(device, true);
	}

	mutex_unlock(&device->mutex);

	return 0;
}

static int adreno_setproperty(struct kgsl_device_private *dev_priv,
				unsigned int type,
				void __user *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;
	struct kgsl_device *device = dev_priv->device;

	switch (type) {
	case KGSL_PROP_PWR_CONSTRAINT:
	case KGSL_PROP_L3_PWR_CONSTRAINT: {
			struct kgsl_device_constraint constraint;
			struct kgsl_context *context;

			if (sizebytes != sizeof(constraint))
				break;

			if (copy_from_user(&constraint, value,
				sizeof(constraint))) {
				status = -EFAULT;
				break;
			}

			context = kgsl_context_get_owner(dev_priv,
							constraint.context_id);

			if (context == NULL)
				break;

			status = adreno_set_constraint(device, context,
								&constraint);

			kgsl_context_put(context);
		}
		break;
	default:
		status = adreno_default_setproperty(dev_priv, type, value, sizebytes);
		break;
	}

	return status;
}

/*
 * adreno_soft_reset -  Do a soft reset of the GPU hardware
 * @device: KGSL device to soft reset
 *
 * "soft reset" the GPU hardware - this is a fast path GPU reset
 * The GPU hardware is reset but we never pull power so we can skip
 * a lot of the standard adreno_stop/adreno_start sequence
 */
static int adreno_soft_reset(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);
	int ret;

	if (gpudev->clear_pending_transactions) {
		ret = gpudev->clear_pending_transactions(adreno_dev);
		if (ret)
			return ret;
	}

	kgsl_pwrctrl_change_state(device, KGSL_STATE_AWARE);
	adreno_set_active_ctxs_null(adreno_dev);

	adreno_irqctrl(adreno_dev, 0);

	adreno_clear_gpu_fault(adreno_dev);
	/* since device is oficially off now clear start bit */
	clear_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv);

	/* save physical performance counter values before GPU soft reset */
	adreno_perfcounter_save(adreno_dev);

	_soft_reset(adreno_dev);

	/* Clear the busy_data stats - we're starting over from scratch */
	adreno_dev->busy_data.gpu_busy = 0;
	adreno_dev->busy_data.bif_ram_cycles = 0;
	adreno_dev->busy_data.bif_ram_cycles_read_ch1 = 0;
	adreno_dev->busy_data.bif_ram_cycles_write_ch0 = 0;
	adreno_dev->busy_data.bif_ram_cycles_write_ch1 = 0;
	adreno_dev->busy_data.bif_starved_ram = 0;
	adreno_dev->busy_data.bif_starved_ram_ch1 = 0;

	/* Reinitialize the GPU */
	gpudev->start(adreno_dev);

	/* Re-initialize the coresight registers if applicable */
	adreno_coresight_start(adreno_dev);

	/* Enable IRQ */
	adreno_irqctrl(adreno_dev, 1);

	/* stop all ringbuffers to cancel RB events */
	adreno_ringbuffer_stop(adreno_dev);

	/* Start the ringbuffer(s) again */
	ret = gpudev->rb_start(adreno_dev);
	if (ret == 0) {
		device->reset_counter++;
		set_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv);
	}

	/* Restore physical performance counter values after soft reset */
	adreno_perfcounter_restore(adreno_dev);

	if (ret)
		dev_err(device->dev, "Device soft reset failed: %d\n", ret);

	return ret;
}

bool adreno_isidle(struct adreno_device *adreno_dev)
{
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);
	struct adreno_ringbuffer *rb;
	int i;

	if (!kgsl_state_is_awake(KGSL_DEVICE(adreno_dev)))
		return true;

	/*
	 * wptr is updated when we add commands to ringbuffer, add a barrier
	 * to make sure updated wptr is compared to rptr
	 */
	smp_mb();

	/*
	 * ringbuffer is truly idle when all ringbuffers read and write
	 * pointers are equal
	 */

	FOR_EACH_RINGBUFFER(adreno_dev, rb, i) {
		if (!adreno_rb_empty(rb))
			return false;
	}

	return gpudev->hw_isidle(adreno_dev);
}

/**
 * adreno_spin_idle() - Spin wait for the GPU to idle
 * @adreno_dev: Pointer to an adreno device
 * @timeout: milliseconds to wait before returning error
 *
 * Spin the CPU waiting for the RBBM status to return idle
 */
int adreno_spin_idle(struct adreno_device *adreno_dev, unsigned int timeout)
{
	unsigned long wait = jiffies + msecs_to_jiffies(timeout);

	do {
		/*
		 * If we fault, stop waiting and return an error. The dispatcher
		 * will clean up the fault from the work queue, but we need to
		 * make sure we don't block it by waiting for an idle that
		 * will never come.
		 */

		if (adreno_gpu_fault(adreno_dev) != 0)
			return -EDEADLK;

		if (adreno_isidle(adreno_dev))
			return 0;

	} while (time_before(jiffies, wait));

	/*
	 * Under rare conditions, preemption can cause the while loop to exit
	 * without checking if the gpu is idle. check one last time before we
	 * return failure.
	 */
	if (adreno_gpu_fault(adreno_dev) != 0)
		return -EDEADLK;

	if (adreno_isidle(adreno_dev))
		return 0;

	return -ETIMEDOUT;
}

/**
 * adreno_idle() - wait for the GPU hardware to go idle
 * @device: Pointer to the KGSL device structure for the GPU
 *
 * Wait up to ADRENO_IDLE_TIMEOUT milliseconds for the GPU hardware to go quiet.
 * Caller must hold the device mutex, and must not hold the dispatcher mutex.
 */

int adreno_idle(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret;

	/*
	 * Make sure the device mutex is held so the dispatcher can't send any
	 * more commands to the hardware
	 */

	if (WARN_ON(!mutex_is_locked(&device->mutex)))
		return -EDEADLK;

	/* Check if we are already idle before idling dispatcher */
	if (adreno_isidle(adreno_dev))
		return 0;
	/*
	 * Wait for dispatcher to finish completing commands
	 * already submitted
	 */
	ret = adreno_dispatcher_idle(adreno_dev);
	if (ret)
		return ret;

	return adreno_spin_idle(adreno_dev, ADRENO_IDLE_TIMEOUT);
}

static int adreno_drain_and_idle(struct kgsl_device *device)
{
	int ret;

	reinit_completion(&device->halt_gate);

	ret = kgsl_active_count_wait(device, 0, HZ);
	if (ret)
		return ret;

	return adreno_idle(device);
}

/* Caller must hold the device mutex. */
int adreno_suspend_context(struct kgsl_device *device)
{
	/* process any profiling results that are available */
	adreno_profile_process_results(ADRENO_DEVICE(device));

	/* Wait for the device to go idle */
	return adreno_idle(device);
}

bool adreno_gx_is_on(struct adreno_device *adreno_dev)
{
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);

	return gpudev->gx_is_on(adreno_dev);
}

void adreno_profile_submit_time(struct adreno_submit_time *time)
{
	struct kgsl_drawobj *drawobj;
	struct kgsl_drawobj_cmd *cmdobj;
	struct kgsl_mem_entry *entry;
	struct kgsl_drawobj_profiling_buffer *profile_buffer;

	if (!time)
		return;

	drawobj = time->drawobj;
	if (drawobj == NULL)
		return;

	cmdobj = CMDOBJ(drawobj);
	entry = cmdobj->profiling_buf_entry;
	if (!entry)
		return;

	profile_buffer = kgsl_gpuaddr_to_vaddr(&entry->memdesc,
			cmdobj->profiling_buffer_gpuaddr);

	if (profile_buffer == NULL)
		return;

	/* Return kernel clock time to the client if requested */
	if (drawobj->flags & KGSL_DRAWOBJ_PROFILING_KTIME) {
		u64 secs = time->ktime;

		profile_buffer->wall_clock_ns =
			do_div(secs, NSEC_PER_SEC);
		profile_buffer->wall_clock_s = secs;
	} else {
		profile_buffer->wall_clock_s = time->utime.tv_sec;
		profile_buffer->wall_clock_ns = time->utime.tv_nsec;
	}

	profile_buffer->gpu_ticks_queued = time->ticks;

	kgsl_memdesc_unmap(&entry->memdesc);
}

/**
 * adreno_waittimestamp - sleep while waiting for the specified timestamp
 * @device - pointer to a KGSL device structure
 * @context - pointer to the active kgsl context
 * @timestamp - GPU timestamp to wait for
 * @msecs - amount of time to wait (in milliseconds)
 *
 * Wait up to 'msecs' milliseconds for the specified timestamp to expire.
 */
static int adreno_waittimestamp(struct kgsl_device *device,
		struct kgsl_context *context,
		unsigned int timestamp,
		unsigned int msecs)
{
	int ret;

	if (context == NULL) {
		/* If they are doing then complain once */
		dev_WARN_ONCE(device->dev, 1,
			"IOCTL_KGSL_DEVICE_WAITTIMESTAMP is deprecated\n");
		return -ENOTTY;
	}

	/* Return -ENOENT if the context has been detached */
	if (kgsl_context_detached(context))
		return -ENOENT;

	ret = adreno_drawctxt_wait(ADRENO_DEVICE(device), context,
		timestamp, msecs);

	/* If the context got invalidated then return a specific error */
	if (kgsl_context_invalid(context))
		ret = -EDEADLK;

	/*
	 * Return -EPROTO if the device has faulted since the last time we
	 * checked.  Userspace uses this as a marker for performing post
	 * fault activities
	 */

	if (!ret && test_and_clear_bit(ADRENO_CONTEXT_FAULT, &context->priv))
		ret = -EPROTO;

	return ret;
}

/**
 * __adreno_readtimestamp() - Reads the timestamp from memstore memory
 * @adreno_dev: Pointer to an adreno device
 * @index: Index into the memstore memory
 * @type: Type of timestamp to read
 * @timestamp: The out parameter where the timestamp is read
 */
static int __adreno_readtimestamp(struct adreno_device *adreno_dev, int index,
				int type, unsigned int *timestamp)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int status = 0;

	switch (type) {
	case KGSL_TIMESTAMP_CONSUMED:
		kgsl_sharedmem_readl(device->memstore, timestamp,
			KGSL_MEMSTORE_OFFSET(index, soptimestamp));
		break;
	case KGSL_TIMESTAMP_RETIRED:
		kgsl_sharedmem_readl(device->memstore, timestamp,
			KGSL_MEMSTORE_OFFSET(index, eoptimestamp));
		break;
	default:
		status = -EINVAL;
		*timestamp = 0;
		break;
	}
	return status;
}

/**
 * adreno_rb_readtimestamp(): Return the value of given type of timestamp
 * for a RB
 * @adreno_dev: adreno device whose timestamp values are being queried
 * @priv: The object being queried for a timestamp (expected to be a rb pointer)
 * @type: The type of timestamp (one of 3) to be read
 * @timestamp: Pointer to where the read timestamp is to be written to
 *
 * CONSUMED and RETIRED type timestamps are sorted by id and are constantly
 * updated by the GPU through shared memstore memory. QUEUED type timestamps
 * are read directly from context struct.

 * The function returns 0 on success and timestamp value at the *timestamp
 * address and returns -EINVAL on any read error/invalid type and timestamp = 0.
 */
int adreno_rb_readtimestamp(struct adreno_device *adreno_dev,
		void *priv, enum kgsl_timestamp_type type,
		unsigned int *timestamp)
{
	int status = 0;
	struct adreno_ringbuffer *rb = priv;

	if (type == KGSL_TIMESTAMP_QUEUED)
		*timestamp = rb->timestamp;
	else
		status = __adreno_readtimestamp(adreno_dev,
				rb->id + KGSL_MEMSTORE_MAX,
				type, timestamp);

	return status;
}

/**
 * adreno_readtimestamp(): Return the value of given type of timestamp
 * @device: GPU device whose timestamp values are being queried
 * @priv: The object being queried for a timestamp (expected to be a context)
 * @type: The type of timestamp (one of 3) to be read
 * @timestamp: Pointer to where the read timestamp is to be written to
 *
 * CONSUMED and RETIRED type timestamps are sorted by id and are constantly
 * updated by the GPU through shared memstore memory. QUEUED type timestamps
 * are read directly from context struct.

 * The function returns 0 on success and timestamp value at the *timestamp
 * address and returns -EINVAL on any read error/invalid type and timestamp = 0.
 */
static int adreno_readtimestamp(struct kgsl_device *device,
		void *priv, enum kgsl_timestamp_type type,
		unsigned int *timestamp)
{
	int status = 0;
	struct kgsl_context *context = priv;

	if (type == KGSL_TIMESTAMP_QUEUED) {
		struct adreno_context *ctxt = ADRENO_CONTEXT(context);

		*timestamp = ctxt->timestamp;
	} else
		status = __adreno_readtimestamp(ADRENO_DEVICE(device),
				context->id, type, timestamp);

	return status;
}

/**
 * adreno_device_private_create(): Allocate an adreno_device_private structure
 */
static struct kgsl_device_private *adreno_device_private_create(void)
{
	struct adreno_device_private *adreno_priv =
			kzalloc(sizeof(*adreno_priv), GFP_KERNEL);

	if (adreno_priv) {
		INIT_LIST_HEAD(&adreno_priv->perfcounter_list);
		return &adreno_priv->dev_priv;
	}
	return NULL;
}

/**
 * adreno_device_private_destroy(): Destroy an adreno_device_private structure
 * and release the perfcounters held by the kgsl fd.
 * @dev_priv: The kgsl device private structure
 */
static void adreno_device_private_destroy(struct kgsl_device_private *dev_priv)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_device_private *adreno_priv =
		container_of(dev_priv, struct adreno_device_private,
		dev_priv);
	struct adreno_perfcounter_list_node *p, *tmp;

	mutex_lock(&device->mutex);
	list_for_each_entry_safe(p, tmp, &adreno_priv->perfcounter_list, node) {
		adreno_perfcounter_put(adreno_dev, p->groupid,
					p->countable, PERFCOUNTER_FLAG_NONE);
		list_del(&p->node);
		kfree(p);
	}
	mutex_unlock(&device->mutex);

	kfree(adreno_priv);
}

/**
 * adreno_power_stats() - Reads the counters needed for freq decisions
 * @device: Pointer to device whose counters are read
 * @stats: Pointer to stats set that needs updating
 * Power: The caller is expected to be in a clock enabled state as this
 * function does reg reads
 */
static void adreno_power_stats(struct kgsl_device *device,
				struct kgsl_power_stats *stats)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);

	return gpudev->power_stats(adreno_dev, stats);
}

static int adreno_regulator_enable(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);

	if (gpudev->regulator_enable)
		return gpudev->regulator_enable(adreno_dev);

	return 0;
}

static bool adreno_is_hw_collapsible(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);

	if (!gpudev->is_hw_collapsible(adreno_dev))
		return false;

	if (gpudev->clear_pending_transactions(adreno_dev))
		return false;

	return true;
}

static void adreno_regulator_disable(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);

	if (gpudev->regulator_disable)
		gpudev->regulator_disable(adreno_dev);
}

static void adreno_pwrlevel_change_settings(struct kgsl_device *device,
		unsigned int prelevel, unsigned int postlevel, bool post)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);

	if (gpudev->pwrlevel_change_settings)
		gpudev->pwrlevel_change_settings(adreno_dev, prelevel,
					postlevel, post);
}

static bool adreno_is_hwcg_on(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	return adreno_dev->hwcg_enabled;
}

static int adreno_queue_cmds(struct kgsl_device_private *dev_priv,
	struct kgsl_context *context, struct kgsl_drawobj *drawobj[],
	u32 count, u32 *timestamp)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (WARN_ON(!adreno_dev->dispatch_ops || !adreno_dev->dispatch_ops->queue_cmds))
		return -ENODEV;

	return adreno_dev->dispatch_ops->queue_cmds(dev_priv, context, drawobj,
		count, timestamp);
}

static inline bool _verify_ib(struct kgsl_device_private *dev_priv,
		struct kgsl_context *context, struct kgsl_memobj_node *ib)
{
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_process_private *private = dev_priv->process_priv;

	/* The maximum allowable size for an IB in the CP is 0xFFFFF dwords */
	if (ib->size == 0 || ((ib->size >> 2) > 0xFFFFF)) {
		pr_context(device, context, "ctxt %u invalid ib size %lld\n",
			context->id, ib->size);
		return false;
	}

	/* Make sure that the address is in range and dword aligned */
	if (!kgsl_mmu_gpuaddr_in_range(private->pagetable, ib->gpuaddr,
		ib->size) || !IS_ALIGNED(ib->gpuaddr, 4)) {
		pr_context(device, context, "ctxt %u invalid ib gpuaddr %llX\n",
			context->id, ib->gpuaddr);
		return false;
	}

	return true;
}

int adreno_verify_cmdobj(struct kgsl_device_private *dev_priv,
		struct kgsl_context *context, struct kgsl_drawobj *drawobj[],
		uint32_t count)
{
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_memobj_node *ib;
	unsigned int i;

	for (i = 0; i < count; i++) {
		/* Verify the IBs before they get queued */
		if (drawobj[i]->type == CMDOBJ_TYPE) {
			struct kgsl_drawobj_cmd *cmdobj = CMDOBJ(drawobj[i]);

			list_for_each_entry(ib, &cmdobj->cmdlist, node)
				if (!_verify_ib(dev_priv,
					&ADRENO_CONTEXT(context)->base, ib))
					return -EINVAL;

			/*
			 * Clear the wake on touch bit to indicate an IB has
			 * been submitted since the last time we set it.
			 * But only clear it when we have rendering commands.
			 */
			device->pwrctrl.wake_on_touch = false;
		}
	}

	return 0;
}

static int adreno_queue_recurring_cmd(struct kgsl_device_private *dev_priv,
	struct kgsl_context *context, struct kgsl_drawobj *drawobj)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);
	struct kgsl_drawobj_cmd *cmdobj = CMDOBJ(drawobj);
	int ret;

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_LSR))
		return -EOPNOTSUPP;

	if (!gpudev->send_recurring_cmdobj)
		return -ENODEV;

	ret = adreno_verify_cmdobj(dev_priv, context, &drawobj, 1);
	if (ret)
		return ret;

	mutex_lock(&device->mutex);

	/* Only one recurring command allowed */
	if (hwsched->recurring_cmdobj) {
		mutex_unlock(&device->mutex);
		return -EINVAL;
	}

	ret = kgsl_check_context_state(context);
	if (ret) {
		mutex_unlock(&device->mutex);
		return ret;
	}

	set_bit(CMDOBJ_RECURRING_START, &cmdobj->priv);

	ret = gpudev->send_recurring_cmdobj(adreno_dev, cmdobj);
	mutex_unlock(&device->mutex);

	if (!ret)
		srcu_notifier_call_chain(&device->nh, GPU_GMU_READY, NULL);

	return ret;
}

static int adreno_dequeue_recurring_cmd(struct kgsl_device *device,
	struct kgsl_context *context)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_hwsched *hwsched = &adreno_dev->hwsched;
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);
	struct kgsl_drawobj *recurring_drawobj;
	int ret;

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_LSR))
		return -EOPNOTSUPP;

	if (!gpudev->send_recurring_cmdobj)
		return -ENODEV;

	mutex_lock(&device->mutex);

	/* We can safely return here as recurring wokload is already untracked */
	if (hwsched->recurring_cmdobj == NULL) {
		mutex_unlock(&device->mutex);
		return -EINVAL;
	}

	recurring_drawobj = DRAWOBJ(hwsched->recurring_cmdobj);

	/* Check if the recurring command is for same context or not*/
	if (recurring_drawobj->context != context) {
		mutex_unlock(&device->mutex);
		return -EINVAL;
	}

	ret = kgsl_check_context_state(context);
	if (ret) {
		mutex_unlock(&device->mutex);
		return ret;
	}

	clear_bit(CMDOBJ_RECURRING_START, &hwsched->recurring_cmdobj->priv);
	set_bit(CMDOBJ_RECURRING_STOP, &hwsched->recurring_cmdobj->priv);

	ret = gpudev->send_recurring_cmdobj(adreno_dev, hwsched->recurring_cmdobj);

	mutex_unlock(&device->mutex);

	if (!ret)
		srcu_notifier_call_chain(&device->nh, GPU_GMU_STOP, NULL);

	return ret;
}

static void adreno_set_isdb_breakpoint_registers(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev  = ADRENO_GPU_DEVICE(adreno_dev);

	if (gpudev->set_isdb_breakpoint_registers)
		gpudev->set_isdb_breakpoint_registers(adreno_dev);
}

static void adreno_drawctxt_sched(struct kgsl_device *device,
		struct kgsl_context *context)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (WARN_ON(!adreno_dev->dispatch_ops || !adreno_dev->dispatch_ops->queue_context))
		return;

	adreno_dev->dispatch_ops->queue_context(adreno_dev,
		ADRENO_CONTEXT(context));
}

void adreno_mark_for_coldboot(struct adreno_device *adreno_dev)
{
	if (!adreno_dev->warmboot_enabled)
		return;

	set_bit(ADRENO_DEVICE_FORCE_COLDBOOT, &adreno_dev->priv);
}

bool adreno_smmu_is_stalled(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_mmu *mmu = &device->mmu;
	u32 fault, val;

	/*
	 * RBBM_STATUS3:SMMU_STALLED_ON_FAULT (BIT 24) to tells if GPU
	 * encountered a pagefault. Gen8 page fault status checked from
	 * the software condition as RBBM_STATUS3 is not available.
	 */
	if (ADRENO_GPUREV(adreno_dev) < ADRENO_REV_GEN8_0_0) {
		adreno_readreg(adreno_dev, ADRENO_REG_RBBM_STATUS3, &val);
		return (val & BIT(24));
	}

	fault = adreno_gpu_fault(adreno_dev);

	return ((fault & ADRENO_IOMMU_STALL_ON_PAGE_FAULT) &&
		test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE, &mmu->pfpolicy)) ? true : false;
}

int adreno_power_cycle(struct adreno_device *adreno_dev,
	void (*callback)(struct adreno_device *adreno_dev, void *priv),
	void *priv)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	const struct adreno_power_ops *ops = ADRENO_POWER_OPS(adreno_dev);
	int ret;

	mutex_lock(&device->mutex);
	ret = ops->pm_suspend(adreno_dev);

	if (!ret) {
		callback(adreno_dev, priv);
		adreno_mark_for_coldboot(adreno_dev);
		ops->pm_resume(adreno_dev);
	}

	mutex_unlock(&device->mutex);

	return ret;
}

struct cycle_data {
	void *ptr;
	void *val;
};

static void cycle_set_bool(struct adreno_device *adreno_dev, void *priv)
{
	struct cycle_data *data = priv;

	*((bool *) data->ptr) = *((bool *) data->val);
}

int adreno_power_cycle_bool(struct adreno_device *adreno_dev,
	bool *flag, bool val)
{
	struct cycle_data data = { .ptr = flag, .val = &val };

	return adreno_power_cycle(adreno_dev, cycle_set_bool, &data);
}

static void cycle_set_u32(struct adreno_device *adreno_dev, void *priv)
{
	struct cycle_data *data = priv;

	*((u32 *) data->ptr) = *((u32 *) data->val);
}

int adreno_power_cycle_u32(struct adreno_device *adreno_dev,
	u32 *flag, u32 val)
{
	struct cycle_data data = { .ptr = flag, .val = &val };

	return adreno_power_cycle(adreno_dev, cycle_set_u32, &data);
}

static int adreno_gpu_clock_set(struct kgsl_device *device, u32 pwrlevel)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_power_ops *ops = ADRENO_POWER_OPS(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct kgsl_pwrlevel *pl = &pwr->pwrlevels[pwrlevel];
	u32 prev_pwrlevel = pwr->previous_pwrlevel;
	int ret;

	if (ops->gpu_clock_set) {
		ret = ops->gpu_clock_set(adreno_dev, pwrlevel);
	} else {
		ret = clk_set_rate(pwr->grp_clks[0], pl->gpu_freq);
		if (ret)
			dev_err(device->dev, "GPU clk freq set failure: %d\n", ret);
	}

	if (ret)
		return ret;

	trace_kgsl_pwrlevel(device, pwrlevel, pl->gpu_freq,
		prev_pwrlevel, pwr->pwrlevels[prev_pwrlevel].gpu_freq);

	KGSL_TRACE_GPU_FREQ(pl->gpu_freq/1000, 0);
	return 0;
}

static int adreno_interconnect_bus_set(struct adreno_device *adreno_dev,
	int level, u32 ab)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if ((level == pwr->cur_buslevel) && (ab == pwr->cur_ab))
		return 0;

	kgsl_icc_set_tag(pwr, level);
	pwr->cur_buslevel = level;
	pwr->cur_ab = ab;

	icc_set_bw(pwr->icc_path, MBps_to_icc(ab),
		kBps_to_icc(pwr->ddr_table[level]));

	trace_kgsl_buslevel(device, pwr->active_pwrlevel, level, ab);

	return 0;
}

static int adreno_gpu_bus_set(struct kgsl_device *device, int level, u32 ab)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_power_ops *ops = ADRENO_POWER_OPS(adreno_dev);

	if (ops->gpu_bus_set)
		return ops->gpu_bus_set(adreno_dev, level, ab);

	return adreno_interconnect_bus_set(adreno_dev, level, ab);
}

u32 adreno_gmu_bus_ab_quantize(struct adreno_device *adreno_dev, u32 ab)
{
	u16 vote = 0;
	u32 max_bw, max_ab;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	if (!adreno_dev->gmu_ab || (ab == INVALID_AB_VALUE))
		return (FIELD_PREP(GENMASK(31, 16), INVALID_AB_VALUE));

	/*
	 * max ddr bandwidth (kbps) = (Max bw in kbps per channel * number of channel)
	 * max ab (Mbps) = max ddr bandwidth (kbps) / 1000
	 */
	max_bw = pwr->ddr_table[pwr->ddr_table_count - 1] * adreno_dev->gpucore->num_ddr_channels;
	max_ab = max_bw / 1000;

	/*
	 * If requested AB is higher than theoretical max bandwidth, set AB vote as max
	 * allowable quantized AB value.
	 *
	 * Power FW supports a 16 bit AB BW level. We can quantize the entire vote-able BW
	 * range to a 16 bit space and the quantized value can be used to vote for AB though
	 * GMU. Quantization can be performed as below.
	 *
	 * quantized_vote = (ab vote (kbps) * 2^16) / max ddr bandwidth (kbps)
	 */
	if (ab >= max_ab)
		vote = MAX_AB_VALUE;
	else
		vote = (u16)(((u64)ab * 1000 * (1 << 16)) / max_bw);

	/*
	 * Vote will be calculated as 0 for smaller AB values.
	 * Set a minimum non-zero vote in such cases.
	 */
	if (ab && !vote)
		vote = 0x1;

	/*
	 * Set ab enable mask and valid AB vote. req.bw is 32 bit value 0xABABENIB
	 * and with this return we want to set the upper 16 bits and EN field specifies
	 * if the AB vote is valid or not.
	 */
	return (FIELD_PREP(GENMASK(31, 16), vote) | FIELD_PREP(GENMASK(15, 8), 1));
}

static void adreno_deassert_gbif_halt(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	const struct adreno_gpudev *gpudev = ADRENO_GPU_DEVICE(adreno_dev);

	if (gpudev->deassert_gbif_halt)
		gpudev->deassert_gbif_halt(adreno_dev);
}

static void adreno_create_hw_fence(struct kgsl_device *device, struct kgsl_sync_fence *kfence)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (WARN_ON(!adreno_dev->dispatch_ops))
		return;

	if (adreno_dev->dispatch_ops->create_hw_fence)
		adreno_dev->dispatch_ops->create_hw_fence(adreno_dev, kfence);
}

u64 adreno_read_cx_timer(struct adreno_device *adreno_dev)
{
	/* Check if the CX timer is initialized */
	if (!test_bit(ADRENO_DEVICE_CX_TIMER_INITIALIZED, &adreno_dev->priv))
		return 0;

	/* Since the GPU CX and CPU timers are synchronized return the CPU timer */
	return arch_timer_read_counter();
}

static void add_proc_fault_list(struct adreno_device *adreno_dev,
			struct kgsl_process_private *proc_priv)
{
	int cur_idx = -1;
	int new_idx = -1;
	int i;

	/*
	 * Add the process to array of faulting processes. This array is sorted (processes with
	 * more faults are higher ranked).
	 */
	write_lock(&adreno_dev->fault_stats_lock);
	proc_priv->fault_count++;

	for (i = ARRAY_SIZE(adreno_dev->fault_procs) - 1; i >= 0; i--) {
		struct adreno_fault_proc *proc = &adreno_dev->fault_procs[i];

		/* Check whether this process is already tracked in the array */
		if (!strcmp(proc->comm, proc_priv->comm)) {
			proc_priv->fault_count = ++proc->fault_count;
			cur_idx = i;
			break;
		}

		/* Find left most entry with lower fault count than current process */
		if (proc->fault_count < proc_priv->fault_count)
			new_idx = i;
	}

	/*
	 * If the process is not currently tracked, we can place it at the left most index that has
	 * a fault count less than its fault count. If such an index does not exist, the array is
	 * already full.
	 */
	if (cur_idx < 0) {
		if (new_idx >= 0)
			goto write_new_idx;
		goto unlock;
	}

	/* Find the left most entry (if any) with a lower fault_count */
	for (new_idx = 0; new_idx < cur_idx; new_idx++) {
		if (adreno_dev->fault_procs[new_idx].fault_count < proc_priv->fault_count)
			break;
	}

	/* Check if the current process is already at its correct position */
	if (new_idx == cur_idx)
		goto unlock;

	/* Swap the two entries */
	adreno_dev->fault_procs[cur_idx].fault_count =
			adreno_dev->fault_procs[new_idx].fault_count;
	strscpy(adreno_dev->fault_procs[cur_idx].comm, adreno_dev->fault_procs[new_idx].comm,
			sizeof(adreno_dev->fault_procs[cur_idx].comm));

write_new_idx:
	adreno_dev->fault_procs[new_idx].fault_count = proc_priv->fault_count;
	strscpy(adreno_dev->fault_procs[new_idx].comm, proc_priv->comm,
		sizeof(adreno_dev->fault_procs[new_idx].comm));

unlock:
	write_unlock(&adreno_dev->fault_stats_lock);
}

void adreno_gpufault_stats(struct adreno_device *adreno_dev,
	struct kgsl_drawobj *drawobj, struct kgsl_drawobj *drawobj_lpac, int fault)
{
	int i;
	struct kgsl_process_private *proc_priv = NULL, *proc_priv_lpac;

	write_lock(&adreno_dev->fault_stats_lock);
	for (i = 0; i < ARRAY_SIZE(adreno_dev->fault_counts); i++)
		if (fault & BIT(i))
			adreno_dev->fault_counts[i]++;
	write_unlock(&adreno_dev->fault_stats_lock);

	if (drawobj) {
		proc_priv = drawobj->context->proc_priv;
		add_proc_fault_list(adreno_dev, proc_priv);
	}
	if (drawobj_lpac) {
		proc_priv_lpac = drawobj_lpac->context->proc_priv;
		if (proc_priv_lpac != proc_priv)
			add_proc_fault_list(adreno_dev, proc_priv_lpac);
	}
}

static const struct kgsl_functable adreno_functable = {
	/* Mandatory functions */
	.suspend_context = adreno_suspend_context,
	.first_open = adreno_first_open,
	.start = adreno_start,
	.stop = adreno_stop,
	.last_close = adreno_last_close,
	.getproperty = adreno_getproperty,
	.getproperty_compat = adreno_getproperty_compat,
	.waittimestamp = adreno_waittimestamp,
	.readtimestamp = adreno_readtimestamp,
	.queue_cmds = adreno_queue_cmds,
	.ioctl = adreno_ioctl,
	.compat_ioctl = adreno_compat_ioctl,
	.power_stats = adreno_power_stats,
	.snapshot = adreno_snapshot,
	.drain_and_idle = adreno_drain_and_idle,
	.device_private_create = adreno_device_private_create,
	.device_private_destroy = adreno_device_private_destroy,
	/* Optional functions */
	.drawctxt_create = adreno_drawctxt_create,
	.drawctxt_detach = adreno_drawctxt_detach,
	.drawctxt_destroy = adreno_drawctxt_destroy,
	.drawctxt_dump = adreno_drawctxt_dump,
	.setproperty = adreno_setproperty,
	.setproperty_compat = adreno_setproperty_compat,
	.drawctxt_sched = adreno_drawctxt_sched,
	.resume = adreno_dispatcher_start,
	.regulator_enable = adreno_regulator_enable,
	.is_hw_collapsible = adreno_is_hw_collapsible,
	.regulator_disable = adreno_regulator_disable,
	.pwrlevel_change_settings = adreno_pwrlevel_change_settings,
	.query_property_list = adreno_query_property_list,
	.is_hwcg_on = adreno_is_hwcg_on,
	.gpu_clock_set = adreno_gpu_clock_set,
	.gpu_bus_set = adreno_gpu_bus_set,
	.deassert_gbif_halt = adreno_deassert_gbif_halt,
	.queue_recurring_cmd = adreno_queue_recurring_cmd,
	.dequeue_recurring_cmd = adreno_dequeue_recurring_cmd,
	.set_isdb_breakpoint_registers = adreno_set_isdb_breakpoint_registers,
	.create_hw_fence = adreno_create_hw_fence,
};

static const struct component_master_ops adreno_ops = {
	.bind = adreno_bind,
	.unbind = adreno_unbind,
};

const struct adreno_power_ops adreno_power_operations = {
	.first_open = adreno_open,
	.last_close = adreno_close,
	.active_count_get = adreno_pwrctrl_active_count_get,
	.pm_suspend = adreno_suspend,
	.pm_resume = adreno_resume,
	.touch_wakeup = adreno_touch_wakeup,
};

static int _compare_of(struct device *dev, void *data)
{
	return (dev->of_node == data);
}

static void _release_of(struct device *dev, void *data)
{
	of_node_put(data);
}

static void adreno_add_components(struct device *dev,
		struct component_match **match)
{
	struct device_node *node;

	/*
	 * Add kgsl-smmu, context banks and gmu as components, if supported.
	 * Master bind (adreno_bind) will be called only once all added
	 * components are available.
	 */
	for_each_matching_node(node, adreno_component_match) {
		if (!of_device_is_available(node))
			continue;

		component_match_add_release(dev, match, _release_of, _compare_of, node);
	}
}

static int adreno_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;

	adreno_add_components(&pdev->dev, &match);

	if (!match)
		return -ENODEV;

	return component_master_add_with_match(&pdev->dev,
			&adreno_ops, match);
}

static int adreno_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &adreno_ops);

	return 0;
}

#if IS_ENABLED(CONFIG_QCOM_KGSL_HIBERNATION)
#if IS_ENABLED(CONFIG_QCOM_SECURE_BUFFER)
/*
 * Issue hyp_assign call to assign non-used internal/userspace secure
 * buffers to kernel.
 */
static int adreno_secure_pt_hibernate(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_process_private *process;
	struct kgsl_mem_entry *entry;
	struct kgsl_global_memdesc *md;
	struct kgsl_memdesc *memdesc;
	int ret, id;

	read_lock(&kgsl_driver.proclist_lock);
	list_for_each_entry(process, &kgsl_driver.process_list, list) {
		idr_for_each_entry(&process->mem_idr, entry, id) {
			memdesc = &entry->memdesc;
			if (!kgsl_memdesc_is_secured(memdesc) ||
				(memdesc->flags & KGSL_MEMFLAGS_USERMEM_ION) ||
				(memdesc->priv & KGSL_MEMDESC_HYPASSIGNED_HLOS))
				continue;

			read_unlock(&kgsl_driver.proclist_lock);

			if (kgsl_unlock_sgt(memdesc->sgt))
				dev_err(device->dev, "kgsl_unlock_sgt failed\n");

			memdesc->priv |= KGSL_MEMDESC_HYPASSIGNED_HLOS;

			read_lock(&kgsl_driver.proclist_lock);
		}
	}
	read_unlock(&kgsl_driver.proclist_lock);

	list_for_each_entry(md, &device->globals, node) {
		memdesc = &md->memdesc;
		if (kgsl_memdesc_is_secured(memdesc) &&
			!(memdesc->priv & KGSL_MEMDESC_HYPASSIGNED_HLOS)) {
			ret = kgsl_unlock_sgt(memdesc->sgt);
			if (ret) {
				dev_err(device->dev, "kgsl_unlock_sgt failed ret %d\n", ret);
				goto fail;
			}
			memdesc->priv |= KGSL_MEMDESC_HYPASSIGNED_HLOS;
		}
	}

	return 0;

fail:
	list_for_each_entry(md, &device->globals, node) {
		memdesc = &md->memdesc;
		if (kgsl_memdesc_is_secured(memdesc) &&
			(memdesc->priv & KGSL_MEMDESC_HYPASSIGNED_HLOS)) {
			kgsl_lock_sgt(memdesc->sgt, memdesc->size);
			memdesc->priv &= ~KGSL_MEMDESC_HYPASSIGNED_HLOS;
		}
	}

	return -EBUSY;
}

static int adreno_secure_pt_restore(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_process_private *process;
	struct kgsl_mem_entry *entry;
	struct kgsl_memdesc *memdesc;
	struct kgsl_global_memdesc *md;
	int ret, id;

	list_for_each_entry(md, &device->globals, node) {
		memdesc = &md->memdesc;
		if (kgsl_memdesc_is_secured(memdesc) &&
			(memdesc->priv & KGSL_MEMDESC_HYPASSIGNED_HLOS)) {
			ret = kgsl_lock_sgt(memdesc->sgt, memdesc->size);
			if (ret) {
				dev_err(device->dev, "kgsl_lock_sgt failed ret %d\n", ret);
				return ret;
			}
			memdesc->priv &= ~KGSL_MEMDESC_HYPASSIGNED_HLOS;
		}
	}

	read_lock(&kgsl_driver.proclist_lock);
	list_for_each_entry(process, &kgsl_driver.process_list, list) {
		idr_for_each_entry(&process->mem_idr, entry, id) {
			memdesc = &entry->memdesc;
			if (!kgsl_memdesc_is_secured(memdesc) ||
				(memdesc->flags & KGSL_MEMFLAGS_USERMEM_ION) ||
				!(memdesc->priv & KGSL_MEMDESC_HYPASSIGNED_HLOS))
				continue;

			read_unlock(&kgsl_driver.proclist_lock);

			ret = kgsl_lock_sgt(memdesc->sgt, memdesc->size);
			if (ret) {
				dev_err(device->dev, "kgsl_lock_sgt failed ret %d\n", ret);
				return ret;
			}
			memdesc->priv &= ~KGSL_MEMDESC_HYPASSIGNED_HLOS;

			read_lock(&kgsl_driver.proclist_lock);
		}
	}
	read_unlock(&kgsl_driver.proclist_lock);

	return 0;
}
#else
static int adreno_secure_pt_hibernate(struct adreno_device *adreno_dev)
{
	return 0;
}

static int adreno_secure_pt_restore(struct adreno_device *adreno_dev)
{
	return 0;
}
#endif /* IS_ENABLED(CONFIG_QCOM_SECURE_BUFFER) */

static int adreno_hibernation_suspend(struct device *dev)
{
	struct kgsl_device *device = dev_get_drvdata(dev);
	struct adreno_device *adreno_dev;
	const struct adreno_power_ops *ops;
	int status;

	if (!device)
		return 0;

	adreno_dev = ADRENO_DEVICE(device);
	ops = ADRENO_POWER_OPS(adreno_dev);

	mutex_lock(&device->mutex);

	status = ops->pm_suspend(adreno_dev);
	if (status)
		goto err;

	/*
	 * When the device enters in hibernation state, the CX will be collapsed causing
	 * the GPU CX timer to pause. Clear the ADRENO_DEVICE_CX_TIMER_INITIALIZED flag
	 * to ensure that the CX timer is reseeded during resume.
	 */
	clear_bit(ADRENO_DEVICE_CX_TIMER_INITIALIZED, &adreno_dev->priv);

	/*
	 * Unload zap shader during device hibernation and reload it
	 * during resume as there is possibility that TZ driver
	 * is not aware of the hibernation.
	 */
	adreno_zap_shader_unload(adreno_dev);
	status = adreno_secure_pt_hibernate(adreno_dev);

err:
	mutex_unlock(&device->mutex);
	return status;
}

static int adreno_hibernation_resume(struct device *dev)
{
	struct kgsl_device *device = dev_get_drvdata(dev);
	struct kgsl_iommu *iommu;
	struct kgsl_pwrscale *pwrscale;
	struct adreno_device *adreno_dev;
	const struct adreno_power_ops *ops;
	int ret;

	if (!device)
		return 0;

	iommu = &device->mmu.iommu;
	pwrscale = &device->pwrscale;
	adreno_dev = ADRENO_DEVICE(device);
	ops = ADRENO_POWER_OPS(adreno_dev);

	mutex_lock(&device->mutex);

	ret = adreno_secure_pt_restore(adreno_dev);
	if (ret)
		goto err;

	ret = kgsl_set_smmu_aperture(device, &iommu->user_context);
	if (ret)
		goto err;

	ret = kgsl_set_smmu_lpac_aperture(device, &iommu->lpac_context);
	if (ret < 0)
		goto err;

	gmu_core_dev_force_first_boot(device);

	msm_adreno_tz_reinit(pwrscale->devfreqptr);

	ops->pm_resume(adreno_dev);

err:
	mutex_unlock(&device->mutex);
	return ret;
}

static const struct dev_pm_ops adreno_pm_ops = {
	.suspend  = adreno_pm_suspend,
	.resume = adreno_pm_resume,
	.freeze = adreno_hibernation_suspend,
	.thaw = adreno_hibernation_resume,
	.poweroff = adreno_hibernation_suspend,
	.restore = adreno_hibernation_resume,
};
#else
static const struct dev_pm_ops adreno_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(adreno_pm_suspend, adreno_pm_resume)
};
#endif /* IS_ENABLED(CONFIG_QCOM_KGSL_HIBERNATION) */

static struct platform_driver adreno_platform_driver = {
	.probe = adreno_probe,
	.remove = adreno_remove,
	.driver = {
		.name = "kgsl-3d",
		.pm = &adreno_pm_ops,
		.of_match_table = of_match_ptr(adreno_match_table),
	}
};

static int __init kgsl_3d_init(void)
{
	int ret;

	ret = kgsl_core_init();
	if (ret)
		return ret;

	ret = kgsl_mmu_init();
	if (ret) {
		kgsl_core_exit();
		return ret;
	}

	gmu_core_register();
	ret = platform_driver_register(&adreno_platform_driver);
	if (ret) {
		gmu_core_unregister();
		kgsl_mmu_exit();
		kgsl_core_exit();
	}

	return ret;
}

static void __exit kgsl_3d_exit(void)
{
	platform_driver_unregister(&adreno_platform_driver);
	gmu_core_unregister();
	kgsl_mmu_exit();
	kgsl_core_exit();
}

module_param_named(preempt_enable, adreno_preemption_enable, bool, 0600);
MODULE_PARM_DESC(preempt_enable, "Enable GPU HW Preemption");

module_param_named(gpu_sku_override, kgsl_gpu_sku_override, uint, 0600);
MODULE_PARM_DESC(gpu_sku_override, "Override SKU code identifier for GPU driver");

module_param_named(gpu_speed_bin_override, kgsl_gpu_speed_bin_override, uint, 0600);
MODULE_PARM_DESC(gpu_speed_bin_override, "Override GPU speed bin");

module_init(kgsl_3d_init);
module_exit(kgsl_3d_exit);

MODULE_DESCRIPTION("3D Graphics driver");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: arm_smmu nvmem_qfprom socinfo governor_msm_adreno_tz governor_gpubw_mon");
#if (KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE)
MODULE_IMPORT_NS(DMA_BUF);
#endif
