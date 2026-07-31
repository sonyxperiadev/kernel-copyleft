// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2010,2015,2019 The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 Linaro Ltd.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
/*
* Copyright 2025 Sony Corporation
* NOTE: This file has been modified by Sony Corporation
* Modifications are licensed under the License.
*/
#define pr_fmt(fmt)     "qcom-scm: %s: " fmt, __func__

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/cpumask.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/reboot.h>
#include <linux/reset-controller.h>
#include <soc/qcom/qseecom_scm.h>
#include <linux/delay.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/gunyah/gh_rm_drv.h>
#include <linux/qti-lcp-ppddr.h>
#include <include/linux/arm-smccc.h>
#include <linux/qtee_shmbridge.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>

#include "qcom_scm.h"
#include "qcom_tzmem.h"
#include "qtee_shmbridge_internal.h"
#include "lcp-ppddr-internal.h"

static bool download_mode = IS_ENABLED(CONFIG_QCOM_SCM_DOWNLOAD_MODE_DEFAULT);
module_param(download_mode, bool, 0);

static unsigned int pas_shutdown_retry_delay_ms = 5000;
module_param(pas_shutdown_retry_delay_ms, uint, 0644);

struct qcom_scm_waitq {
	struct idr idr;
	spinlock_t idr_lock;
	struct work_struct scm_irq_work;
	u64 call_ctx_cnt;
	u64 irq;
	enum qcom_scm_wq_feature wq_feature;
};

struct qcom_scm {
	struct device *dev;
	struct clk *core_clk;
	struct clk *iface_clk;
	struct clk *bus_clk;
	struct icc_path *path;
	struct completion waitq_comp;
	struct reset_controller_dev reset;
	struct notifier_block restart_nb;
	struct qcom_scm_waitq waitq;

	/* control access to the interconnect path */
	struct mutex scm_bw_lock;
	int scm_vote_count;

	u64 dload_mode_addr;

	struct qcom_tzmem_pool *mempool;
};

static enum qcom_scm_custom_reset_type qcom_scm_custom_reset_type = QCOM_SCM_RST_NONE;

DEFINE_SEMAPHORE(qcom_scm_sem_lock, 1);

/**
 * struct qcom_scm_qseecom_resp - QSEECOM SCM call response.
 * @result:    Result or status of the SCM call. See &enum qcom_scm_qseecom_result.
 * @resp_type: Type of the response. See &enum qcom_scm_qseecom_resp_type.
 * @data:      Response data. The type of this data is given in @resp_type.
 */
struct qcom_scm_qseecom_resp {
	u64 result;
	u64 resp_type;
	u64 data;
};

enum qcom_scm_qseecom_result {
	QSEECOM_RESULT_SUCCESS			= 0,
	QSEECOM_RESULT_INCOMPLETE		= 1,
	QSEECOM_RESULT_BLOCKED_ON_LISTENER	= 2,
	QSEECOM_RESULT_FAILURE			= 0xFFFFFFFF,
};

enum qcom_scm_qseecom_resp_type {
	QSEECOM_SCM_RES_APP_ID			= 0xEE01,
	QSEECOM_SCM_RES_QSEOS_LISTENER_ID	= 0xEE02,
};

enum qcom_scm_qseecom_tz_owner {
	QSEECOM_TZ_OWNER_SIP			= 2,
	QSEECOM_TZ_OWNER_TZ_APPS		= 48,
	QSEECOM_TZ_OWNER_QSEE_OS		= 50
};

enum qcom_scm_qseecom_tz_svc {
	QSEECOM_TZ_SVC_APP_ID_PLACEHOLDER	= 0,
	QSEECOM_TZ_SVC_APP_MGR			= 1,
	QSEECOM_TZ_SVC_INFO			= 6,
};

enum qcom_scm_qseecom_tz_cmd_app {
	QSEECOM_TZ_CMD_APP_SEND			= 1,
	QSEECOM_TZ_CMD_APP_LOOKUP		= 3,
};

enum qcom_scm_qseecom_tz_cmd_info {
	QSEECOM_TZ_CMD_INFO_VERSION		= 3,
};

#define QSEECOM_MAX_APP_NAME_SIZE		64

#define SHMBRIDGE_RESULT_NOTSUPP		4

/* Each bit configures cold/warm boot address for one of the 4 CPUs */
static const u8 qcom_scm_cpu_cold_bits[QCOM_SCM_BOOT_MAX_CPUS] = {
	0, BIT(0), BIT(3), BIT(5)
};
static const u8 qcom_scm_cpu_warm_bits[QCOM_SCM_BOOT_MAX_CPUS] = {
	BIT(2), BIT(1), BIT(4), BIT(6)
};


#define QCOM_SCM_FLAG_COLDBOOT_CPU0	0x00
#define QCOM_SCM_FLAG_COLDBOOT_CPU1	0x01
#define QCOM_SCM_FLAG_COLDBOOT_CPU2	0x08
#define QCOM_SCM_FLAG_COLDBOOT_CPU3	0x20

#define QCOM_SCM_FLAG_WARMBOOT_CPU0	0x04
#define QCOM_SCM_FLAG_WARMBOOT_CPU1	0x02
#define QCOM_SCM_FLAG_WARMBOOT_CPU2	0x10
#define QCOM_SCM_FLAG_WARMBOOT_CPU3	0x40

#define QCOM_SMC_WAITQ_FLAG_WAKE_ONE	BIT(0)
#define QCOM_SMC_WAITQ_FLAG_WAKE_ALL	BIT(1)
#define QCOM_SCM_WAITQ_FLAG_WAKE_NONE   0x0

#define QCOM_DLOAD_MASK		GENMASK(5, 4)
#define QCOM_DLOAD_NODUMP	0
#define QCOM_DLOAD_FULLDUMP	1

static const char * const qcom_scm_convention_names[] = {
	[SMC_CONVENTION_UNKNOWN] = "unknown",
	[SMC_CONVENTION_ARM_32] = "smc arm 32",
	[SMC_CONVENTION_ARM_64] = "smc arm 64",
	[SMC_CONVENTION_LEGACY] = "smc legacy",
};

#define GIC_SPI_BASE        32
#define GIC_MAX_SPI       1019  // SPIs in GICv3 spec range from 32..1019
#define GIC_ESPI_BASE     4096
#define GIC_MAX_ESPI      5119 // ESPIs in GICv3 spec range from 4096..5119

static struct qcom_scm *__scm;

#define SCM_NOT_INITIALIZED()  (unlikely(!__scm) ? pr_err("SCM not initialized\n") : 0)

static int qcom_scm_clk_enable(void)
{
	int ret;

	ret = clk_prepare_enable(__scm->core_clk);
	if (ret)
		goto bail;

	ret = clk_prepare_enable(__scm->iface_clk);
	if (ret)
		goto disable_core;

	ret = clk_prepare_enable(__scm->bus_clk);
	if (ret)
		goto disable_iface;

	return 0;

disable_iface:
	clk_disable_unprepare(__scm->iface_clk);
disable_core:
	clk_disable_unprepare(__scm->core_clk);
bail:
	return ret;
}

static void qcom_scm_clk_disable(void)
{
	clk_disable_unprepare(__scm->core_clk);
	clk_disable_unprepare(__scm->iface_clk);
	clk_disable_unprepare(__scm->bus_clk);
}

static int qcom_scm_bw_enable(void)
{
	int ret = 0;

	if (!__scm->path)
		return 0;

	mutex_lock(&__scm->scm_bw_lock);
	if (!__scm->scm_vote_count) {
		ret = icc_set_bw(__scm->path, 0, UINT_MAX);
		if (ret < 0) {
			dev_err(__scm->dev, "failed to set bandwidth request\n");
			goto err_bw;
		}
	}
	__scm->scm_vote_count++;
err_bw:
	mutex_unlock(&__scm->scm_bw_lock);

	return ret;
}

static void qcom_scm_bw_disable(void)
{
	if (!__scm->path)
		return;

	mutex_lock(&__scm->scm_bw_lock);
	if (__scm->scm_vote_count-- == 1)
		icc_set_bw(__scm->path, 0, 0);
	mutex_unlock(&__scm->scm_bw_lock);
}

enum qcom_scm_convention qcom_scm_convention = SMC_CONVENTION_UNKNOWN;
static DEFINE_SPINLOCK(scm_query_lock);

struct qcom_tzmem_pool *qcom_scm_get_tzmem_pool(void)
{
	return __scm->mempool;
}

static enum qcom_scm_convention __get_convention(void)
{
	unsigned long flags;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_INFO,
		.cmd = QCOM_SCM_INFO_IS_CALL_AVAIL,
		.args[0] = SCM_SMC_FNID(QCOM_SCM_SVC_INFO,
					   QCOM_SCM_INFO_IS_CALL_AVAIL) |
			   (ARM_SMCCC_OWNER_SIP << ARM_SMCCC_OWNER_SHIFT),
		.arginfo = QCOM_SCM_ARGS(1),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	enum qcom_scm_convention probed_convention;
	int ret;
	bool forced = false;

	if (likely(qcom_scm_convention != SMC_CONVENTION_UNKNOWN))
		return qcom_scm_convention;

	/*
	 * Per the "SMC calling convention specification", the 64-bit calling
	 * convention can only be used when the client is 64-bit, otherwise
	 * system will encounter the undefined behaviour.
	 */
#if IS_ENABLED(CONFIG_ARM64)
	/*
	 * Device isn't required as there is only one argument - no device
	 * needed to dma_map_single to secure world
	 */
	probed_convention = SMC_CONVENTION_ARM_64;
	ret = __scm_smc_call(NULL, &desc, probed_convention, &res, QCOM_SCM_CALL_ATOMIC);
	if (!ret && res.result[0] == 1)
		goto found;

	/*
	 * Some SC7180 firmwares didn't implement the
	 * QCOM_SCM_INFO_IS_CALL_AVAIL call, so we fallback to forcing ARM_64
	 * calling conventions on these firmwares. Luckily we don't make any
	 * early calls into the firmware on these SoCs so the device pointer
	 * will be valid here to check if the compatible matches.
	 */
	if (of_device_is_compatible(__scm ? __scm->dev->of_node : NULL, "qcom,scm-sc7180")) {
		forced = true;
		goto found;
	}
#endif

	probed_convention = SMC_CONVENTION_ARM_32;
	ret = __scm_smc_call(NULL, &desc, probed_convention, &res, QCOM_SCM_CALL_ATOMIC);
	if (!ret && res.result[0] == 1)
		goto found;

	probed_convention = SMC_CONVENTION_LEGACY;
found:
	spin_lock_irqsave(&scm_query_lock, flags);
	if (probed_convention != qcom_scm_convention) {
		qcom_scm_convention = probed_convention;
		pr_info("qcom_scm: convention: %s%s\n",
			qcom_scm_convention_names[qcom_scm_convention],
			forced ? " (forced)" : "");
	}
	spin_unlock_irqrestore(&scm_query_lock, flags);

	return qcom_scm_convention;
}

/**
 * qcom_scm_call() - Invoke a syscall in the secure world
 * @dev:	device
 * @desc:	Descriptor structure containing arguments and return values
 * @res:        Structure containing results from SMC/HVC call
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 * This should *only* be called in pre-emptible context.
 */
static int qcom_scm_call(struct device *dev, const struct qcom_scm_desc *desc,
			 struct qcom_scm_res *res)
{
	might_sleep();
	switch (__get_convention()) {
	case SMC_CONVENTION_ARM_32:
	case SMC_CONVENTION_ARM_64:
		return scm_smc_call(dev, desc, res, QCOM_SCM_CALL_NORMAL);
	case SMC_CONVENTION_LEGACY:
		return scm_legacy_call(dev, desc, res);
	default:
		pr_err("Unknown current SCM calling convention.\n");
		return -EINVAL;
	}
}

/**
 * qcom_scm_call_atomic() - atomic variation of qcom_scm_call()
 * @dev:	device
 * @desc:	Descriptor structure containing arguments and return values
 * @res:	Structure containing results from SMC/HVC call
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 * This can be called in atomic context.
 */
static int qcom_scm_call_atomic(struct device *dev,
				const struct qcom_scm_desc *desc,
				struct qcom_scm_res *res)
{
	switch (__get_convention()) {
	case SMC_CONVENTION_ARM_32:
	case SMC_CONVENTION_ARM_64:
		return scm_smc_call(dev, desc, res, QCOM_SCM_CALL_ATOMIC);
	case SMC_CONVENTION_LEGACY:
		return scm_legacy_call_atomic(dev, desc, res);
	default:
		pr_err("Unknown current SCM calling convention.\n");
		return -EINVAL;
	}
}

/**
 * qcom_scm_call_noretry() - noretry variation of qcom_scm_call()
 * @dev:	device
 * @svc_id:	service identifier
 * @cmd_id:	command identifier
 * @desc:	Descriptor structure containing arguments and return values
 * @res:	Structure containing results from SMC/HVC call
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 */
static int qcom_scm_call_noretry(struct device *dev,
				const struct qcom_scm_desc *desc,
				struct qcom_scm_res *res)
{
	switch (__get_convention()) {
	case SMC_CONVENTION_ARM_32:
	case SMC_CONVENTION_ARM_64:
		return scm_smc_call(dev, desc, res, QCOM_SCM_CALL_NORETRY);
	case SMC_CONVENTION_LEGACY:
		BUG_ON(1); /* No current implementation */
	default:
		pr_err("Unknown current SCM calling convention.\n");
		return -EINVAL;
	}
}

static bool __qcom_scm_is_call_available(struct device *dev, u32 svc_id,
					 u32 cmd_id)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_INFO,
		.cmd = QCOM_SCM_INFO_IS_CALL_AVAIL,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	desc.arginfo = QCOM_SCM_ARGS(1);
	switch (__get_convention()) {
	case SMC_CONVENTION_ARM_32:
	case SMC_CONVENTION_ARM_64:
		desc.args[0] = SCM_SMC_FNID(svc_id, cmd_id) |
				(ARM_SMCCC_OWNER_SIP << ARM_SMCCC_OWNER_SHIFT);
		break;
	case SMC_CONVENTION_LEGACY:
		desc.args[0] = SCM_LEGACY_FNID(svc_id, cmd_id);
		break;
	default:
		pr_err("Unknown SMC convention being used\n");
		return false;
	}

	ret = qcom_scm_call(dev, &desc, &res);

	return ret ? false : !!res.result[0];
}

static int qcom_scm_set_boot_addr(void *entry, const u8 *cpu_bits)
{
	int cpu;
	unsigned int flags = 0;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_ADDR,
		.arginfo = QCOM_SCM_ARGS(2),
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	if (!__scm)
		return -EINVAL;

	for_each_present_cpu(cpu) {
		if (cpu >= QCOM_SCM_BOOT_MAX_CPUS)
			return -EINVAL;
		flags |= cpu_bits[cpu];
	}

	desc.args[0] = flags;
	desc.args[1] = virt_to_phys(entry);

	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}

static int qcom_scm_set_boot_addr_mc(void *entry, unsigned int flags)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_ADDR_MC,
		.owner = ARM_SMCCC_OWNER_SIP,
		.arginfo = QCOM_SCM_ARGS(6),
		.args = {
			virt_to_phys(entry),
			/* Apply to all CPUs in all affinity levels */
			~0ULL, ~0ULL, ~0ULL, ~0ULL,
			flags,
		},
	};

	/* Need a device for DMA of the additional arguments */
	if (!__scm || __get_convention() == SMC_CONVENTION_LEGACY)
		return -EOPNOTSUPP;

	return qcom_scm_call(__scm->dev, &desc, NULL);
}

/**
 * qcom_scm_set_warm_boot_addr() - Set the warm boot address for all cpus
 * @entry: Entry point function for the cpus
 *
 * Set the Linux entry point for the SCM to transfer control to when coming
 * out of a power down. CPU power down may be executed on cpuidle or hotplug.
 */
int qcom_scm_set_warm_boot_addr(void *entry)
{
	if (qcom_scm_set_boot_addr_mc(entry, QCOM_SCM_BOOT_MC_FLAG_WARMBOOT))
		/* Fallback to old SCM call */
		return qcom_scm_set_boot_addr(entry, qcom_scm_cpu_warm_bits);
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_set_warm_boot_addr);

/**
 * qcom_scm_set_cold_boot_addr() - Set the cold boot address for all cpus
 * @entry: Entry point function for the cpus
 */
int qcom_scm_set_cold_boot_addr(void *entry)
{
	if (qcom_scm_set_boot_addr_mc(entry, QCOM_SCM_BOOT_MC_FLAG_COLDBOOT))
		/* Fallback to old SCM call */
		return qcom_scm_set_boot_addr(entry, qcom_scm_cpu_cold_bits);
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_set_cold_boot_addr);

/**
 * qcom_scm_cpu_power_down() - Power down the cpu
 * @flags:	Flags to flush cache
 *
 * This is an end point to power down cpu. If there was a pending interrupt,
 * the control would return from this function, otherwise, the cpu jumps to the
 * warm boot entry point set for this cpu upon reset.
 */
void qcom_scm_cpu_power_down(u32 flags)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_TERMINATE_PC,
		.args[0] = flags & QCOM_SCM_FLUSH_FLAG_MASK,
		.arginfo = QCOM_SCM_ARGS(1),
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	qcom_scm_call_atomic(__scm ? __scm->dev : NULL, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_cpu_power_down);

/**
 * qcm_scm_sec_wdog_deactivate() - Deactivate secure watchdog
 */
int qcom_scm_sec_wdog_deactivate(void)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SEC_WDOG_DIS,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = 1,
		.arginfo = QCOM_SCM_ARGS(1),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_sec_wdog_deactivate);

/**
 * qcom_scm_sec_wdog_trigger() - Trigger secure watchdog
 */
int qcom_scm_sec_wdog_trigger(void)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SEC_WDOG_TRIGGER,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = 0,
		.arginfo = QCOM_SCM_ARGS(1),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_sec_wdog_trigger);

int qcom_scm_set_remote_state(u32 state, u32 id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_REMOTE_STATE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = state,
		.args[1] = id,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_set_remote_state);

int qcom_scm_spin_cpu(void)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SPIN_CPU,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = 0,
		.arginfo = QCOM_SCM_ARGS(1),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_spin_cpu);

int qcom_scm_disable_sdi(void)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SDI_CONFIG,
		.args[0] = 1, /* Disable watchdog debug */
		.args[1] = 0, /* Disable SDI */
		.arginfo = QCOM_SCM_ARGS(2),
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;
	ret = qcom_scm_call(__scm->dev, &desc, &res);

	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_disable_sdi);

static int __qcom_scm_set_dload_mode(struct device *dev, enum qcom_download_mode mode)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_DLOAD_MODE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = mode,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	desc.args[1] = 0;

	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}

void qcom_scm_set_download_mode(enum qcom_download_mode mode)
{
	int ret = 0;
	struct device *dev = __scm ? __scm->dev : NULL;

	if (__scm && __scm->dload_mode_addr) {
		ret = qcom_scm_io_writel(__scm->dload_mode_addr, mode);
	} else if (__qcom_scm_is_call_available(dev,
				QCOM_SCM_SVC_BOOT,
				QCOM_SCM_BOOT_SET_DLOAD_MODE)) {
		ret = __qcom_scm_set_dload_mode(dev, mode);
	} else {
		dev_err(dev,
			"No available mechanism for setting download mode\n");
	}

	if (ret)
		dev_err(dev, "failed to set download mode: %d\n", ret);
}
EXPORT_SYMBOL_GPL(qcom_scm_set_download_mode);

int qcom_scm_get_download_mode(unsigned int *mode)
{
	int ret = -EINVAL;
	struct device *dev = __scm ? __scm->dev : NULL;

	if (__scm && __scm->dload_mode_addr) {
		ret = qcom_scm_io_readl(__scm->dload_mode_addr, mode);
	} else {
		dev_err(dev,
			"No available mechanism for getting download mode\n");
	}

	if (ret)
		dev_err(dev, "failed to get download mode: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_get_download_mode);

int qcom_scm_config_cpu_errata(void)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_CONFIG_CPU_ERRATA,
		.owner = ARM_SMCCC_OWNER_SIP,
		.arginfo = 0xffffffff,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_config_cpu_errata);

/**
 * qcom_scm_pas_init_image() - Initialize peripheral authentication service
 *			       state machine for a given peripheral, using the
 *			       metadata
 * @peripheral: peripheral id
 * @metadata:	pointer to memory containing ELF header, program header table
 *		and optional blob of data used for authenticating the metadata
 *		and the rest of the firmware
 * @size:	size of the metadata
 * @ctx:	optional metadata context
 *
 * Return: 0 on success.
 *
 * Upon successful return, the PAS metadata context (@ctx) will be used to
 * track the metadata allocation, this needs to be released by invoking
 * qcom_scm_pas_metadata_release() by the caller.
 */

void qcom_scm_phy_update_scm_level_shifter(u32 val)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_QUSB2PHY_LVL_SHIFTER_CMD_ID,
		.owner = ARM_SMCCC_OWNER_SIP
	};

	if (SCM_NOT_INITIALIZED())
		return;

	desc.args[0] = val;
	desc.args[1] = 0;
	desc.arginfo = QCOM_SCM_ARGS(2);

	ret = qcom_scm_call(__scm->dev, &desc, NULL);
	if (ret)
		pr_err("Failed to update scm level shifter=0x%x\n", ret);

}
EXPORT_SYMBOL_GPL(qcom_scm_phy_update_scm_level_shifter);


int qcom_scm_pas_init_image(u32 peripheral, const void *metadata, size_t size,
			    struct qcom_scm_pas_metadata *ctx, struct device *dev_32bit)
{
	struct device *dma_dev = __scm->dev;
	dma_addr_t mdata_phys;
	void *mdata_buf;
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_INIT_IMAGE,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_VAL, QCOM_SCM_RW),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	/*
	 * Only use 32bit dma device for dma memory allocation but use
	 * Scm device for any scm calls.
	 */
	if (dev_32bit)
		dma_dev = dev_32bit;

	/*
	 * During the scm call memory protection will be enabled for the meta
	 * data blob, so make sure it's physically contiguous, 4K aligned and
	 * non-cachable to avoid XPU violations.
	 *
	 * For PIL calls the hypervisor creates SHM Bridges for the blob
	 * buffers on behalf of Linux so we must not do it ourselves hence
	 * not using the TZMem allocator here.
	 *
	 * If we pass a buffer that is already part of an SHM Bridge to this
	 * call, it will fail.
	 */
	mdata_buf = dma_alloc_coherent(dma_dev, size, &mdata_phys,
				       GFP_KERNEL);
	if (!mdata_buf) {
		dev_err(dma_dev, "Allocation of metadata buffer failed.\n");
		return -ENOMEM;
	}
	memcpy(mdata_buf, metadata, size);

	ret = qcom_scm_clk_enable();
	if (ret)
		goto out;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	desc.args[1] = mdata_phys;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

out:
	if (ret < 0 || !ctx) {
		dma_free_coherent(dma_dev, size, mdata_buf, mdata_phys);
	} else if (ctx) {
		ctx->ptr = mdata_buf;
		ctx->phys = mdata_phys;
		ctx->size = size;
	}

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_init_image);

/**
 * qcom_scm_pas_metadata_release() - release metadata context
 * @ctx:	metadata context
 */
void qcom_scm_pas_metadata_release(struct qcom_scm_pas_metadata *ctx,
				   struct device *dev_32bit)
{
	struct device *dma_dev = __scm->dev;

	if (!ctx || !ctx->ptr)
		return;

	if (dev_32bit)
		dma_dev = dev_32bit;

	dma_free_coherent(dma_dev, ctx->size, ctx->ptr, ctx->phys);

	ctx->ptr = NULL;
	ctx->phys = 0;
	ctx->size = 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_metadata_release);

/**
 * qcom_scm_pas_mem_setup() - Prepare the memory related to a given peripheral
 *			      for firmware loading
 * @peripheral:	peripheral id
 * @addr:	start address of memory area to prepare
 * @size:	size of the memory area to prepare
 *
 * Returns 0 on success.
 */
int qcom_scm_pas_mem_setup(u32 peripheral, phys_addr_t addr, phys_addr_t size)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_MEM_SETUP,
		.arginfo = QCOM_SCM_ARGS(3),
		.args[0] = peripheral,
		.args[1] = addr,
		.args[2] = size,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_mem_setup);

/**
 * qcom_scm_pas_auth_and_reset() - Authenticate the given peripheral firmware
 *				   and reset the remote processor
 * @peripheral:	peripheral id
 *
 * Return 0 on success.
 */
int qcom_scm_pas_auth_and_reset(u32 peripheral)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_AUTH_AND_RESET,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_auth_and_reset);

/**
 * qcom_scm_pas_shutdown() - Shut down the remote processor
 * @peripheral: peripheral id
 *
 * Returns 0 on success.
 */
int qcom_scm_pas_shutdown(u32 peripheral)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_SHUTDOWN,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_bw_enable();
	if (ret)
		goto disable_clk;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	qcom_scm_bw_disable();

disable_clk:
	qcom_scm_clk_disable();

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_shutdown);

int qcom_scm_pas_shutdown_retry(u32 peripheral)
{
	int ret;

	ret = qcom_scm_pas_shutdown(peripheral);
	/* No need to retry if the first try worked */
	if (!ret)
		return ret;

	pr_err("PAS Shutdown: First call to shutdown failed with error: %d\n", ret);
	pr_err("PAS Shutdown: Sleeping for: %u\n", pas_shutdown_retry_delay_ms);
	msleep(pas_shutdown_retry_delay_ms);

	pr_err("PAS Shutdown: Attempting to shutdown peripheral again\n");
	return qcom_scm_pas_shutdown(peripheral);
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_shutdown_retry);

/**
 * qcom_scm_pas_supported() - Check if the peripheral authentication service is
 *			      available for the given peripherial
 * @peripheral:	peripheral id
 *
 * Returns true if PAS is supported for this peripheral, otherwise false.
 */
bool qcom_scm_pas_supported(u32 peripheral)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_IS_SUPPORTED,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = peripheral,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	if (!__qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_PIL,
					  QCOM_SCM_PIL_PAS_IS_SUPPORTED))
		return false;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? false : !!res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_pas_supported);

static int __qcom_scm_pas_mss_reset(struct device *dev, bool reset)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_MSS_RESET,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = reset,
		.args[1] = 0,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}

static int qcom_scm_pas_reset_assert(struct reset_controller_dev *rcdev,
				     unsigned long idx)
{
	if (idx != 0)
		return -EINVAL;

	return __qcom_scm_pas_mss_reset(__scm->dev, 1);
}

static int qcom_scm_pas_reset_deassert(struct reset_controller_dev *rcdev,
				       unsigned long idx)
{
	if (idx != 0)
		return -EINVAL;

	return __qcom_scm_pas_mss_reset(__scm->dev, 0);
}

static const struct reset_control_ops qcom_scm_pas_reset_ops = {
	.assert = qcom_scm_pas_reset_assert,
	.deassert = qcom_scm_pas_reset_deassert,
};

int qcom_scm_get_sec_dump_state(u32 *dump_state)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_UTIL,
		.cmd = QCOM_SCM_UTIL_GET_SEC_DUMP_STATE,
		.owner = ARM_SMCCC_OWNER_SIP
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm ? __scm->dev : NULL, &desc, &res);

	if (dump_state)
		*dump_state = res.result[0];

	return ret;
}
EXPORT_SYMBOL(qcom_scm_get_sec_dump_state);

int qcom_scm_assign_dump_table_region(bool is_assign, phys_addr_t addr, size_t size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_UTIL,
		.cmd = QCOM_SCM_UTIL_DUMP_TABLE_ASSIGN,
		.arginfo = QCOM_SCM_ARGS(3),
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = is_assign,
		.args[1] = addr,
		.args[2] = size,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_assign_dump_table_region);

int qcom_scm_io_readl(phys_addr_t addr, unsigned int *val)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_IO,
		.cmd = QCOM_SCM_IO_READ,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = addr,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;


	ret = qcom_scm_call_atomic(__scm->dev, &desc, &res);
	if (ret >= 0)
		*val = res.result[0];

	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_io_readl);

int qcom_scm_io_writel(phys_addr_t addr, unsigned int val)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_IO,
		.cmd = QCOM_SCM_IO_WRITE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = addr,
		.args[1] = val,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_io_writel);

/**
 * qcom_scm_io_reset()
 */
int qcom_scm_io_reset(void)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_IO,
		.cmd = QCOM_SCM_IO_RESET,
		.owner = ARM_SMCCC_OWNER_SIP,
		.arginfo = QCOM_SCM_ARGS(2),
	};

	return qcom_scm_call_atomic(__scm ? __scm->dev : NULL, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_io_reset);

bool qcom_scm_is_secure_wdog_trigger_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_BOOT,
						QCOM_SCM_BOOT_SEC_WDOG_TRIGGER);
}
EXPORT_SYMBOL(qcom_scm_is_secure_wdog_trigger_available);

bool qcom_scm_is_mode_switch_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_BOOT,
						QCOM_SCM_BOOT_SWITCH_MODE);
}
EXPORT_SYMBOL(qcom_scm_is_mode_switch_available);

int __qcom_scm_get_feat_version(struct device *dev, u64 feat_id, u64 *version)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_INFO,
		.cmd = QCOM_SCM_INFO_GET_FEAT_VERSION_CMD,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = feat_id,
		.arginfo = QCOM_SCM_ARGS(1),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (version)
		*version = res.result[0];

	return ret;
}

/**
 * qcom_halt_spmi_pmic_arbiter() - Halt SPMI PMIC arbiter
 *
 * Force the SPMI PMIC arbiter to shutdown so that no more SPMI transactions
 * are sent from the MSM to the PMIC. This is required in order to avoid an
 * SPMI lockup on certain PMIC chips if PS_HOLD is lowered in the middle of
 * an SPMI transaction.
 */
void qcom_scm_halt_spmi_pmic_arbiter(void)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PWR,
		.cmd = QCOM_SCM_PWR_IO_DISABLE_PMIC_ARBITER,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = 0,
		.arginfo = QCOM_SCM_ARGS(1),
	};

	ret = qcom_scm_call_atomic(__scm->dev, &desc, NULL);
	if (ret)
		pr_debug("Failed to halt_spmi_pmic_arbiter=0x%x\n", ret);
}

int qcom_scm_spmi_permission_switch(u8 mode)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PWR,
		.cmd = QCOM_SCM_PWR_GPIO_TRANSFER_ACCESS,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = mode,
		.arginfo = QCOM_SCM_ARGS(3),
	};
	desc.args[1] = 0;
	desc.args[2] = 0;

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_spmi_permission_switch);

/**
 * qcom_scm_restore_sec_cfg_available() - Check if secure environment
 * supports restore security config interface.
 *
 * Return true if restore-cfg interface is supported, false if not.
 */
bool qcom_scm_restore_sec_cfg_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_MP,
					    QCOM_SCM_MP_RESTORE_SEC_CFG);
}
EXPORT_SYMBOL_GPL(qcom_scm_restore_sec_cfg_available);

int qcom_scm_restore_sec_cfg(u32 device_id, u32 spare)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_RESTORE_SEC_CFG,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = device_id,
		.args[1] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_restore_sec_cfg);

int qcom_scm_iommu_secure_ptbl_size(u32 spare, size_t *size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_IOMMU_SECURE_PTBL_SIZE,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (size)
		*size = res.result[0];

	return ret ? : res.result[1];
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_secure_ptbl_size);

int qcom_scm_iommu_secure_ptbl_init(u64 addr, u32 size, u32 spare)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_IOMMU_SECURE_PTBL_INIT,
		.arginfo = QCOM_SCM_ARGS(3, QCOM_SCM_RW, QCOM_SCM_VAL,
					 QCOM_SCM_VAL),
		.args[0] = addr,
		.args[1] = size,
		.args[2] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, NULL);

	/* the pg table has been initialized already, ignore the error */
	if (ret == -EPERM)
		ret = 0;

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_secure_ptbl_init);

int qcom_scm_iommu_set_cp_pool_size(u32 spare, u32 size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_IOMMU_SET_CP_POOL_SIZE,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = size,
		.args[1] = spare,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_set_cp_pool_size);

int qcom_scm_mem_protect_video_var(u32 cp_start, u32 cp_size,
				   u32 cp_nonpixel_start,
				   u32 cp_nonpixel_size)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_VIDEO_VAR,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_VAL, QCOM_SCM_VAL,
					 QCOM_SCM_VAL, QCOM_SCM_VAL),
		.args[0] = cp_start,
		.args[1] = cp_size,
		.args[2] = cp_nonpixel_start,
		.args[3] = cp_nonpixel_size,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_mem_protect_video_var);

int qcom_scm_mem_protect_region_id(phys_addr_t paddr, size_t size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_MEM_PROTECT_REGION_ID,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = paddr,
		.args[1] = size,
		.arginfo = QCOM_SCM_ARGS(2),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_mem_protect_region_id);

static int __qcom_scm_assign_mem(struct device *dev, phys_addr_t mem_region,
				 size_t mem_sz, phys_addr_t src, size_t src_sz,
				 phys_addr_t dest, size_t dest_sz)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_ASSIGN,
		.arginfo = QCOM_SCM_ARGS(7, QCOM_SCM_RO, QCOM_SCM_VAL,
					 QCOM_SCM_RO, QCOM_SCM_VAL, QCOM_SCM_RO,
					 QCOM_SCM_VAL, QCOM_SCM_VAL),
		.args[0] = mem_region,
		.args[1] = mem_sz,
		.args[2] = src,
		.args[3] = src_sz,
		.args[4] = dest,
		.args[5] = dest_sz,
		.args[6] = 0,
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(dev, &desc, &res);

	return ret ? : res.result[0];
}

/**
 * qcom_scm_assign_mem() - Make a secure call to reassign memory ownership
 * @mem_addr: mem region whose ownership need to be reassigned
 * @mem_sz:   size of the region.
 * @srcvm:    vmid for current set of owners, each set bit in
 *            flag indicate a unique owner
 * @newvm:    array having new owners and corresponding permission
 *            flags
 * @dest_cnt: number of owners in next set.
 *
 * Return negative errno on failure or 0 on success with @srcvm updated.
 */
int qcom_scm_assign_mem(phys_addr_t mem_addr, size_t mem_sz,
			u64 *srcvm,
			const struct qcom_scm_vmperm *newvm,
			unsigned int dest_cnt)
{
	struct qcom_scm_current_perm_info *destvm;
	struct qcom_scm_mem_map_info *mem_to_map;
	phys_addr_t mem_to_map_phys;
	phys_addr_t dest_phys;
	phys_addr_t ptr_phys;
	size_t mem_to_map_sz;
	size_t dest_sz;
	size_t src_sz;
	size_t ptr_sz;
	u64 next_vm;
	__le32 *src;
	int ret, i, b;
	u64 srcvm_bits = *srcvm;

	if (!gh_rm_needs_scm_assign(srcvm, newvm, dest_cnt))
		return 0;

	src_sz = hweight64(srcvm_bits) * sizeof(*src);
	mem_to_map_sz = sizeof(*mem_to_map);
	dest_sz = dest_cnt * sizeof(*destvm);
	ptr_sz = ALIGN(src_sz, SZ_64) + ALIGN(mem_to_map_sz, SZ_64) +
			ALIGN(dest_sz, SZ_64);

	void *ptr __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							ptr_sz, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	ptr_phys = qcom_tzmem_to_phys(ptr);

	/* Fill source vmid detail */
	src = ptr;
	i = 0;
	for (b = 0; b < BITS_PER_TYPE(u64); b++) {
		if (srcvm_bits & BIT(b))
			src[i++] = cpu_to_le32(b);
	}

	/* Fill details of mem buff to map */
	mem_to_map = ptr + ALIGN(src_sz, SZ_64);
	mem_to_map_phys = ptr_phys + ALIGN(src_sz, SZ_64);
	mem_to_map->mem_addr = cpu_to_le64(mem_addr);
	mem_to_map->mem_size = cpu_to_le64(mem_sz);

	next_vm = 0;
	/* Fill details of next vmid detail */
	destvm = ptr + ALIGN(mem_to_map_sz, SZ_64) + ALIGN(src_sz, SZ_64);
	dest_phys = ptr_phys + ALIGN(mem_to_map_sz, SZ_64) + ALIGN(src_sz, SZ_64);
	for (i = 0; i < dest_cnt; i++, destvm++, newvm++) {
		destvm->vmid = cpu_to_le32(newvm->vmid);
		destvm->perm = cpu_to_le32(newvm->perm);
		destvm->ctx = 0;
		destvm->ctx_size = 0;
		next_vm |= BIT(newvm->vmid);
	}

	ret = __qcom_scm_assign_mem(__scm->dev, mem_to_map_phys, mem_to_map_sz,
				    ptr_phys, src_sz, dest_phys, dest_sz);
	if (ret) {
		dev_err(__scm->dev,
			"Assign memory protection call failed %d\n", ret);
		return -EINVAL;
	}

	*srcvm = next_vm;
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_assign_mem);

/**
 * qcom_scm_assign_mem_regions() - Make a secure call to reassign memory
 *				   ownership of several memory regions
 * @mem_regions:    A buffer describing the set of memory regions that need to
 *		    be reassigned
 * @mem_regions_sz: The size of the buffer describing the set of memory
 *                  regions that need to be reassigned (in bytes)
 * @srcvms:	    A buffer populated with he vmid(s) for the current set of
 *		    owners
 * @src_sz:	    The size of the src_vms buffer (in bytes)
 * @newvms:	    A buffer populated with the new owners and corresponding
 *		    permission flags.
 * @newvms_sz:	    The size of the new_vms buffer (in bytes)
 *
 * NOTE: It is up to the caller to ensure that the buffers that will be accessed
 * by the secure world are cache aligned, and have been flushed prior to
 * invoking this call.
 *
 * Return negative errno on failure, 0 on success.
 */
int qcom_scm_assign_mem_regions(struct qcom_scm_mem_map_info *mem_regions,
				size_t mem_regions_sz, u32 *srcvms,
				size_t src_sz,
				struct qcom_scm_current_perm_info *newvms,
				size_t newvms_sz)
{
	return __qcom_scm_assign_mem(__scm ? __scm->dev : NULL,
				     virt_to_phys(mem_regions), mem_regions_sz,
				     virt_to_phys(srcvms), src_sz,
				     virt_to_phys(newvms), newvms_sz);
}
EXPORT_SYMBOL(qcom_scm_assign_mem_regions);

/**
 * qcom_scm_mem_protect_sd_ctrl() - SDE memory protect.
 *
 */
int qcom_scm_mem_protect_sd_ctrl(u32 devid, phys_addr_t mem_addr, u64 mem_size,
				u32 vmid)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_CMD_SD_CTRL,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = devid,
		.args[1] = mem_addr,
		.args[2] = mem_size,
		.args[3] = vmid,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_VAL, QCOM_SCM_RW,
					 QCOM_SCM_VAL, QCOM_SCM_VAL)
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_mem_protect_sd_ctrl);

#define CFG_PHYS_DDR_PROTECTIONS_FOR_REGIONS_API_VERSION	1

static int __qcom_scm_cfg_phys_ddr_protections_for_region(
			struct device *dev,
			phys_addr_t ppddr_set_phys,
			uint32_t ppddr_set_sz,
			uint32_t *resp,
			phys_addr_t resp_phys,
			uint32_t resp_sz,
			enum cfg_phys_ddr_protection_cmd cmd)
{
	struct qcom_scm_res res;
	int ret;

	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DDR,
		.cmd = QCOM_SCM_SVC_DDR_CFG_PHYS_DDR_PROTECTION_FOR_REGIONS,
		.arginfo = QCOM_SCM_ARGS(6,
					 QCOM_SCM_VAL,
					 QCOM_SCM_RW,
					 QCOM_SCM_VAL,
					 QCOM_SCM_RW,
					 QCOM_SCM_VAL,
					 QCOM_SCM_VAL),
		.args[0] = CFG_PHYS_DDR_PROTECTIONS_FOR_REGIONS_API_VERSION,
		.args[1] = ppddr_set_phys,
		.args[2] = ppddr_set_sz,
		.args[3] = resp_phys,
		.args[4] = resp_sz,
		.args[5] = cmd,
		.owner = QSEECOM_TZ_OWNER_SIP,
	};

	while (1) {
		ret = qcom_scm_call(__scm->dev, &desc, &res);
		if (ret)
			goto out;

		if (*resp != CFG_PHYS_DDR_PROTECTION_RSP_CMD_PROCESSING)
			break;

		/* after submitting the request, we just need to poll */
		desc.args[5] = CFG_PHYS_DDR_PROTECTION_CMD_GET_CMD_RESULT_FOR_REGIONS;
	}
	ret = res.result[0];

out:
	if (ret)
		pr_err("cfg_pddr_protected_region SCM call ret %d\n", ret);

	return ret;
}

static int map_to_linux_error(uint32_t resp)
{
	switch (resp) {
	case CFG_PHYS_DDR_PROTECTION_RSP_CMD_COMPLETE:
		return 0;
	case CFG_PHYS_DDR_PROTECTION_RSP_ERR_HW_IS_BUSY:
		return -EBUSY;
	case CFG_PHYS_DDR_PROTECTION_RSP_ERR_REGION_NOT_PROTECTABLE:
	case CFG_PHYS_DDR_PROTECTION_RSP_ERR_NO_CFG_ALLOWED:
		return -EINVAL;
	case CFG_PHYS_DDR_PROTECTION_RSP_ERR_REGION_ALREADY_IN_USE:
		/*
		 * Region is in use by another VM, so we shouldn't be
		 * reconfiguring it
		 */
		return -EINVAL;
	case CFG_PHYS_DDR_PROTECTION_RSP_ERR_PARTIAL_ENABLE_NOT_SUPPORTED:
		return -EOPNOTSUPP;
	case CFG_PHYS_DDR_PROTECTION_RSP_ERR_CMD_FAILED:
	case CFG_PHYS_DDR_PROTECTION_RSP_ERR_MAX:
	default:
		return -EIO;
	}
}

/*
 * Allocate memory from the shmbridge shared region and zero the region.
 * Return 0 on success or error otherwise.
 */
static int alloc_from_shmbridge_pool(struct device *dev, size_t len,
				     struct qtee_shm *shm)
{
	int ret;

	/* LCP-DARE TZ calls require shmbridge */
	if (!qtee_shmbridge_is_enabled())
		return -EOPNOTSUPP;

	ret = qtee_shmbridge_allocate_shm(len, shm);
	if (ret)
		return ret;

	memset(shm->vaddr, 0, len);

	pr_debug("%s() paddr %llu, size %lu, ret %d\n", __func__, shm->paddr,
				shm->size, ret);
	return 0;
}

/**
 * qcom_scm_cfg_pddr_protected_regions() - Make a secure call to configure
 *		 DDR protections for the region @cfg_region.
 *
 * It is not an error to try to reconfigure a region/sub-region as the
 * same type again. It just be might be inefficient if HW ends up
 * reinitializing the region, but TZ will check this and avoid going
 * to the hardware. There is no other entity changing the settings on
 * th regiones so we could cache the settings and avoid calling TZ, but
 * for now lets leave it to TZ.
 *
 * Return 0 on success or negative errno on failure.
 */
int qcom_scm_cfg_pddr_protected_region(struct ppddr_region *cfg_region)
{
	struct phys_protected_ddr_region *ppddr;
	enum cfg_phys_ddr_protection_cmd cmd;
	struct qtee_shm ppddr_shm = { 0 };
	struct qtee_shm resp_shm = { 0 };
	phys_addr_t ppddr_phys;
	phys_addr_t resp_phys;
	uint32_t ppddr_sz;
	uint32_t resp_sz;
	uint32_t *resp;
	uint32_t ret;

	if (cfg_region == NULL)
		return 0;

	cmd = cfg_region->lcp_mem_type;

	switch (cmd) {
	case CFG_PHYS_DDR_PROTECTION_CMD_GET_CMD_RESULT_FOR_REGIONS:
		/*
		 * This type is only to communicate with TZ. No reason
		 * for caller to use it at this time.
		 */
		return -EINVAL;

	case CFG_PHYS_DDR_PROTECTION_CMD_DISABLE_REGIONS:
	case CFG_PHYS_DDR_PROTECTION_CMD_ENABLE_DE:
	case CFG_PHYS_DDR_PROTECTION_CMD_ENABLE_AND_INIT_DAE:
	case CFG_PHYS_DDR_PROTECTION_CMD_ENABLE_AND_INIT_DARE:
	case CFG_PHYS_DDR_PROTECTION_CMD_ENABLE_MTE:
	case CFG_PHYS_DDR_PROTECTION_CMD_ENABLE_DE_AND_MTE:
		break;

	default:
		return -EINVAL;
	}

	/*
	 * Allocate memory for responses buffer and the scm arg buffer, ppddr
	 * These must be allocated in the region shared with shmbridge.
	 */
	resp_sz = sizeof(uint32_t);
	ret = alloc_from_shmbridge_pool(__scm->dev, resp_sz, &resp_shm);
	if (ret)
		return ret;

	ppddr_sz = sizeof(*ppddr);
	ret = alloc_from_shmbridge_pool(__scm->dev, ppddr_sz, &ppddr_shm);
	if (ret)
		goto out_free_resp;

	ppddr = ppddr_shm.vaddr;
	ppddr_phys = ppddr_shm.paddr;

	resp = resp_shm.vaddr;
	resp_phys = resp_shm.paddr;

	memset(resp, 0, resp_sz);

	if (!ppddr)
		goto out_free_resp;

	ppddr->ppddr_data_region.start_addr =
				cfg_region->data_region.start_addr;
	ppddr->ppddr_data_region.end_addr =
				cfg_region->data_region.end_addr;
	ppddr->ppddr_init_data_region.start_addr =
				cfg_region->data_region.start_addr;
	ppddr->ppddr_init_data_region.end_addr =
				cfg_region->data_region.end_addr;

	/* NOTE: TZ manages the tag regions, so ignore them for now */
	ppddr->ppddr_tag_regions_ptr = NULL;
	ppddr->ppddr_tag_regions_size = 0;
	ppddr->ppddr_ret_tag_regions_ptr = NULL;
	ppddr->ppddr_ret_tag_regions_len_ptr = 0;
	ppddr->ppddr_ret_tag_regions_len_ptr_size = 0;

	ret = __qcom_scm_cfg_phys_ddr_protections_for_region(__scm->dev,
					ppddr_phys, ppddr_sz, resp, resp_phys,
					resp_sz, cmd);
	if (ret)
		goto out;

	ret = map_to_linux_error(*resp);
out:
	qtee_shmbridge_free_shm(&ppddr_shm);

out_free_resp:
	if (ret)
		pr_err("%s(): resp %u ret %d\n", __func__, *resp, ret);

	qtee_shmbridge_free_shm(&resp_shm);

	pr_debug("%s() region [0x%llx, 0x%llx] type %d, ret %d\n", __func__,
		cfg_region->data_region.start_addr,
		cfg_region->data_region.end_addr, cfg_region->lcp_mem_type,
		ret);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_cfg_pddr_protected_region);

int qcom_scm_kgsl_set_smmu_aperture(unsigned int num_context_bank)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_CP_SMMU_APERTURE_ID,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = 0xffff0000
			   | ((QCOM_SCM_CP_APERTURE_REG & 0xff) << 8)
			   | (num_context_bank & 0xff),
		.args[1] = 0xffffffff,
		.args[2] = 0xffffffff,
		.args[3] = 0xffffffff,
		.arginfo = QCOM_SCM_ARGS(4),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_kgsl_set_smmu_aperture);

int qcom_scm_kgsl_set_smmu_lpac_aperture(unsigned int num_context_bank)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_CP_SMMU_APERTURE_ID,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = 0xffff0000
			   | ((QCOM_SCM_CP_LPAC_APERTURE_REG & 0xff) << 8)
			   | (num_context_bank & 0xff),
		.args[1] = 0xffffffff,
		.args[2] = 0xffffffff,
		.args[3] = 0xffffffff,
		.arginfo = QCOM_SCM_ARGS(4),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_kgsl_set_smmu_lpac_aperture);

int qcom_scm_kgsl_init_regs(u32 gpu_req)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_GPU,
		.cmd = QCOM_SCM_SVC_GPU_INIT_REGS,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = gpu_req,
		.arginfo = QCOM_SCM_ARGS(1),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_kgsl_init_regs);

int qcom_scm_kgsl_dcvs_tuning(u32 mingap, u32 penalty, u32 numbusy)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DCVS,
		.cmd = QCOM_SCM_DCVS_TUNING,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = mingap,
		.args[1] = penalty,
		.args[2] = numbusy,
		.arginfo = QCOM_SCM_ARGS(3),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_kgsl_dcvs_tuning);

int qcom_scm_enable_shm_bridge(void)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_ENABLE,
		.owner = ARM_SMCCC_OWNER_SIP
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_enable_shm_bridge);

int qcom_scm_delete_shm_bridge(u64 handle)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_DELETE,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = handle,
		.arginfo = QCOM_SCM_ARGS(1, QCOM_SCM_VAL),
	};

	return qcom_scm_call(__scm ? __scm->dev : NULL, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_delete_shm_bridge);

int qcom_scm_create_shm_bridge(u64 pfn_and_ns_perm_flags,
	u64 ipfn_and_s_perm_flags, u64 size_and_flags, u64 ns_vmids,
	u64 *handle)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_CREATE,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = pfn_and_ns_perm_flags,
		.args[1] = ipfn_and_s_perm_flags,
		.args[2] = size_and_flags,
		.args[3] = ns_vmids,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_VAL, QCOM_SCM_VAL,
					QCOM_SCM_VAL, QCOM_SCM_VAL),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (handle)
		*handle = res.result[1];

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_create_shm_bridge);

/**
 * qcom_scm_dcvs_core_available() - check if core DCVS operations are available
 */
bool qcom_scm_dcvs_core_available(void)
{
	struct device *dev = __scm ? __scm->dev : NULL;

	return __qcom_scm_is_call_available(dev, QCOM_SCM_SVC_DCVS,
					    QCOM_SCM_DCVS_INIT) &&
	       __qcom_scm_is_call_available(dev, QCOM_SCM_SVC_DCVS,
					    QCOM_SCM_DCVS_UPDATE) &&
	       __qcom_scm_is_call_available(dev, QCOM_SCM_SVC_DCVS,
					    QCOM_SCM_DCVS_RESET);
}
EXPORT_SYMBOL(qcom_scm_dcvs_core_available);

/**
 * qcom_scm_dcvs_ca_available() - check if context aware DCVS operations are
 * available
 */
bool qcom_scm_dcvs_ca_available(void)
{
	struct device *dev = __scm ? __scm->dev : NULL;

	return __qcom_scm_is_call_available(dev, QCOM_SCM_SVC_DCVS,
					    QCOM_SCM_DCVS_INIT_CA_V2) &&
	       __qcom_scm_is_call_available(dev, QCOM_SCM_SVC_DCVS,
					    QCOM_SCM_DCVS_UPDATE_CA_V2);
}
EXPORT_SYMBOL(qcom_scm_dcvs_ca_available);

/**
 * qcom_scm_dcvs_reset()
 */
int qcom_scm_dcvs_reset(void)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DCVS,
		.cmd = QCOM_SCM_DCVS_RESET,
		.owner = ARM_SMCCC_OWNER_SIP
	};

	return qcom_scm_call(__scm ? __scm->dev : NULL, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_dcvs_reset);

int qcom_scm_dcvs_init_v2(phys_addr_t addr, size_t size, int *version)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DCVS,
		.cmd = QCOM_SCM_DCVS_INIT_V2,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = addr,
		.args[1] = size,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_RW, QCOM_SCM_VAL),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (ret >= 0)
		*version = res.result[0];
	return ret;
}
EXPORT_SYMBOL(qcom_scm_dcvs_init_v2);

int qcom_scm_dcvs_init_ca_v2(phys_addr_t addr, size_t size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DCVS,
		.cmd = QCOM_SCM_DCVS_INIT_CA_V2,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = addr,
		.args[1] = size,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_RW, QCOM_SCM_VAL),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_dcvs_init_ca_v2);

int qcom_scm_dcvs_update(int level, s64 total_time, s64 busy_time)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DCVS,
		.cmd = QCOM_SCM_DCVS_UPDATE,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = level,
		.args[1] = total_time,
		.args[2] = busy_time,
		.arginfo = QCOM_SCM_ARGS(3),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call_atomic(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_dcvs_update);

int qcom_scm_dcvs_update_v2(int level, s64 total_time, s64 busy_time)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DCVS,
		.cmd = QCOM_SCM_DCVS_UPDATE_V2,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = level,
		.args[1] = total_time,
		.args[2] = busy_time,
		.arginfo = QCOM_SCM_ARGS(3),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_dcvs_update_v2);

int qcom_scm_dcvs_update_ca_v2(int level, s64 total_time, s64 busy_time,
			       int context_count)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_DCVS,
		.cmd = QCOM_SCM_DCVS_UPDATE_CA_V2,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = level,
		.args[1] = total_time,
		.args[2] = busy_time,
		.args[3] = context_count,
		.arginfo = QCOM_SCM_ARGS(4),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_dcvs_update_ca_v2);

/**
 * qcom_scm_ocmem_lock_available() - is OCMEM lock/unlock interface available
 */
bool qcom_scm_ocmem_lock_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_OCMEM,
					    QCOM_SCM_OCMEM_LOCK_CMD);
}
EXPORT_SYMBOL_GPL(qcom_scm_ocmem_lock_available);

/**
 * qcom_scm_ocmem_lock() - call OCMEM lock interface to assign an OCMEM
 * region to the specified initiator
 *
 * @id:     tz initiator id
 * @offset: OCMEM offset
 * @size:   OCMEM size
 * @mode:   access mode (WIDE/NARROW)
 */
int qcom_scm_ocmem_lock(enum qcom_scm_ocmem_client id, u32 offset, u32 size,
			u32 mode)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_OCMEM,
		.cmd = QCOM_SCM_OCMEM_LOCK_CMD,
		.args[0] = id,
		.args[1] = offset,
		.args[2] = size,
		.args[3] = mode,
		.arginfo = QCOM_SCM_ARGS(4),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_ocmem_lock);

/**
 * qcom_scm_ocmem_unlock() - call OCMEM unlock interface to release an OCMEM
 * region from the specified initiator
 *
 * @id:     tz initiator id
 * @offset: OCMEM offset
 * @size:   OCMEM size
 */
int qcom_scm_ocmem_unlock(enum qcom_scm_ocmem_client id, u32 offset, u32 size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_OCMEM,
		.cmd = QCOM_SCM_OCMEM_UNLOCK_CMD,
		.args[0] = id,
		.args[1] = offset,
		.args[2] = size,
		.arginfo = QCOM_SCM_ARGS(3),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_ocmem_unlock);

/**
 * qcom_scm_ice_available() - Is the ICE key programming interface available?
 *
 * Return: true iff the SCM calls wrapped by qcom_scm_ice_invalidate_key() and
 *	   qcom_scm_ice_set_key() are available.
 */
bool qcom_scm_ice_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					    QCOM_SCM_ES_INVALIDATE_ICE_KEY) &&
		__qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_ES,
					     QCOM_SCM_ES_CONFIG_SET_ICE_KEY);
}
EXPORT_SYMBOL_GPL(qcom_scm_ice_available);

/**
 * qcom_scm_ice_invalidate_key() - Invalidate an inline encryption key
 * @index: the keyslot to invalidate
 *
 * The UFSHCI and eMMC standards define a standard way to do this, but it
 * doesn't work on these SoCs; only this SCM call does.
 *
 * It is assumed that the SoC has only one ICE instance being used, as this SCM
 * call doesn't specify which ICE instance the keyslot belongs to.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_ice_invalidate_key(u32 index)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_INVALIDATE_ICE_KEY,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = index,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_ice_invalidate_key);

/**
 * qcom_scm_ice_set_key() - Set an inline encryption key
 * @index: the keyslot into which to set the key
 * @key: the key to program
 * @key_size: the size of the key in bytes
 * @cipher: the encryption algorithm the key is for
 * @data_unit_size: the encryption data unit size, i.e. the size of each
 *		    individual plaintext and ciphertext.  Given in 512-byte
 *		    units, e.g. 1 = 512 bytes, 8 = 4096 bytes, etc.
 *
 * Program a key into a keyslot of Qualcomm ICE (Inline Crypto Engine), where it
 * can then be used to encrypt/decrypt UFS or eMMC I/O requests inline.
 *
 * The UFSHCI and eMMC standards define a standard way to do this, but it
 * doesn't work on these SoCs; only this SCM call does.
 *
 * It is assumed that the SoC has only one ICE instance being used, as this SCM
 * call doesn't specify which ICE instance the keyslot belongs to.
 *
 * Return: 0 on success; -errno on failure.
 */
int qcom_scm_ice_set_key(u32 index, const u8 *key, u32 key_size,
			 enum qcom_scm_ice_cipher cipher, u32 data_unit_size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_CONFIG_SET_ICE_KEY,
		.arginfo = QCOM_SCM_ARGS(5, QCOM_SCM_VAL, QCOM_SCM_RW,
					 QCOM_SCM_VAL, QCOM_SCM_VAL,
					 QCOM_SCM_VAL),
		.args[0] = index,
		.args[2] = key_size,
		.args[3] = cipher,
		.args[4] = data_unit_size,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	int ret;

	void *keybuf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							   key_size,
							   GFP_KERNEL);
	if (!keybuf)
		return -ENOMEM;
	memcpy(keybuf, key, key_size);
	desc.args[1] = qcom_tzmem_to_phys(keybuf);

	ret = qcom_scm_call(__scm->dev, &desc, NULL);

	memzero_explicit(keybuf, key_size);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_ice_set_key);

int qcom_scm_config_set_ice_key(uint32_t index, phys_addr_t paddr, size_t size,
				uint32_t cipher, unsigned int data_unit,
				unsigned int ce)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_CONFIG_SET_ICE_KEY_V2,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = index,
		.args[1] = paddr,
		.args[2] = size,
		.args[3] = cipher,
		.args[4] = data_unit,
		.args[5] = ce,
		.arginfo = QCOM_SCM_ARGS(6, QCOM_SCM_VAL, QCOM_SCM_RW,
					QCOM_SCM_VAL, QCOM_SCM_VAL,
					QCOM_SCM_VAL, QCOM_SCM_VAL),
	};

	return qcom_scm_call_noretry(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_config_set_ice_key);

int qcom_scm_clear_ice_key(uint32_t index,  unsigned int ce)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_CLEAR_ICE_KEY,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = index,
		.args[1] = ce,
		.arginfo = QCOM_SCM_ARGS(2),
	};

	return qcom_scm_call_noretry(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_clear_ice_key);

int qcom_scm_derive_sw_secret(phys_addr_t paddr_key, size_t size_key,
			      phys_addr_t paddr_secret, size_t size_secret)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_ES,
		.cmd = QCOM_SCM_ES_DERIVE_RAW_SECRET,
		.owner = ARM_SMCCC_OWNER_SIP
	};

	desc.args[0] = paddr_key;
	desc.args[1] = size_key;
	desc.args[2] = paddr_secret;
	desc.args[3] = size_secret;
	desc.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_RW, QCOM_SCM_VAL,
					QCOM_SCM_RW, QCOM_SCM_VAL);
	return qcom_scm_call_noretry(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_derive_sw_secret);

/**
 * qcom_scm_hdcp_available() - Check if secure environment supports HDCP.
 *
 * Return true if HDCP is supported, false if not.
 */
bool qcom_scm_hdcp_available(void)
{
	bool avail;
	int ret = qcom_scm_clk_enable();

	if (ret)
		return ret;

	avail = __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_HDCP,
						QCOM_SCM_HDCP_INVOKE);

	qcom_scm_clk_disable();

	return avail;
}
EXPORT_SYMBOL_GPL(qcom_scm_hdcp_available);

/**
 * qcom_scm_hdcp_req() - Send HDCP request.
 * @req: HDCP request array
 * @req_cnt: HDCP request array count
 * @resp: response buffer passed to SCM
 *
 * Write HDCP register(s) through SCM.
 */
int qcom_scm_hdcp_req(struct qcom_scm_hdcp_req *req, u32 req_cnt, u32 *resp)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_HDCP,
		.cmd = QCOM_SCM_HDCP_INVOKE,
		.arginfo = QCOM_SCM_ARGS(10),
		.args = {
			req[0].addr,
			req[0].val,
			req[1].addr,
			req[1].val,
			req[2].addr,
			req[2].val,
			req[3].addr,
			req[3].val,
			req[4].addr,
			req[4].val
		},
		.owner = ARM_SMCCC_OWNER_SIP,
	};
	struct qcom_scm_res res;

	if (req_cnt > QCOM_SCM_HDCP_MAX_REQ_CNT)
		return -ERANGE;

	ret = qcom_scm_clk_enable();
	if (ret)
		return ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	*resp = res.result[0];

	qcom_scm_clk_disable();

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_hdcp_req);

int qcom_scm_lmh_fetch_data(u32 node_id, u32 debug_type, uint32_t *peak,
		uint32_t *avg)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_LMH,
		.cmd = QCOM_SCM_LMH_DEBUG_FETCH_DATA,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = node_id,
		.args[1] = debug_type,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_VAL, QCOM_SCM_VAL),
	};
	struct qcom_scm_res res;

	ret = __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_LMH,
					   QCOM_SCM_LMH_DEBUG_FETCH_DATA);
	if (ret <= 0)
		return ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (peak)
		*peak = res.result[0];
	if (avg)
		*avg = res.result[1];

	return ret;
}
EXPORT_SYMBOL(qcom_scm_lmh_fetch_data);

int qcom_scm_iommu_set_pt_format(u32 sec_id, u32 ctx_num, u32 pt_fmt)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMMU_PROGRAM,
		.cmd = QCOM_SCM_SMMU_PT_FORMAT,
		.arginfo = QCOM_SCM_ARGS(3),
		.args[0] = sec_id,
		.args[1] = ctx_num,
		.args[2] = pt_fmt, /* 0: LPAE AArch32 - 1: AArch64 */
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_iommu_set_pt_format);

int qcom_scm_qsmmu500_wait_safe_toggle(bool en)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMMU_PROGRAM,
		.cmd = QCOM_SCM_SMMU_CONFIG_ERRATA1,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = QCOM_SCM_SMMU_CONFIG_ERRATA1_CLIENT_ALL,
		.args[1] = en,
		.owner = ARM_SMCCC_OWNER_SIP,
	};


	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_qsmmu500_wait_safe_toggle);

int qcom_scm_smmu_notify_secure_lut(u64 dev_id, bool secure)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMMU_PROGRAM,
		.cmd = QCOM_SCM_SMMU_SECURE_LUT,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = dev_id,
		.args[1] = secure,
		.arginfo = QCOM_SCM_ARGS(2),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_smmu_notify_secure_lut);

int qcom_scm_camera_protect_all(uint32_t protect, uint32_t param)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_CAMERA,
		.cmd = QCOM_SCM_CAMERA_PROTECT_ALL,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = protect,
		.args[1] = param,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_VAL, QCOM_SCM_VAL),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_camera_protect_all);

int qcom_scm_camera_protect_phy_lanes(bool protect, u64 regmask)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_CAMERA,
		.cmd = QCOM_SCM_CAMERA_PROTECT_PHY_LANES,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = protect,
		.args[1] = regmask,
		.arginfo = QCOM_SCM_ARGS(2),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL(qcom_scm_camera_protect_phy_lanes);

int qcom_scm_camera_update_camnoc_qos(uint32_t use_case_id,
	uint32_t cam_qos_cnt, struct qcom_scm_camera_qos *cam_qos)
{
	int ret;
	dma_addr_t payload_phys;
	u32 *payload_buf = NULL;
	u32 payload_size = 0;

	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_CAMERA,
		.cmd = QCOM_SCM_CAMERA_UPDATE_CAMNOC_QOS,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = use_case_id,
		.args[2] = payload_size,
		.arginfo = QCOM_SCM_ARGS(3, QCOM_SCM_VAL, QCOM_SCM_RW, QCOM_SCM_VAL),
	};

	if ((cam_qos_cnt > QCOM_SCM_CAMERA_MAX_QOS_CNT) || (cam_qos_cnt && !cam_qos)) {
		pr_err("Invalid input SmartQoS count: %d\n", cam_qos_cnt);
		return -EINVAL;
	}

	payload_size = cam_qos_cnt * sizeof(struct qcom_scm_camera_qos);

	/* fill all required qos settings */
	if (use_case_id && payload_size && cam_qos) {
		payload_buf = dma_alloc_coherent(__scm->dev,
						 payload_size, &payload_phys, GFP_KERNEL);
		if (!payload_buf)
			return -ENOMEM;

		memcpy(payload_buf, cam_qos, payload_size);
		desc.args[1] = payload_phys;
		desc.args[2] = payload_size;
	}

	ret = qcom_scm_call(__scm->dev, &desc, NULL);

	if (payload_buf)
		dma_free_coherent(__scm->dev, payload_size, payload_buf, payload_phys);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_camera_update_camnoc_qos);

static int qcom_scm_reboot(struct device *dev)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_OEM_POWER,
		.cmd = QCOM_SCM_OEM_POWER_REBOOT,
		.owner = ARM_SMCCC_OWNER_OEM,
	};

	return qcom_scm_call_atomic(dev, &desc, NULL);
}

static int qcom_scm_custom_reboot(struct device *dev,
			enum qcom_scm_custom_reset_type reboot_type)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_OEM_POWER,
		.cmd = QCOM_SCM_OEM_POWER_CUSTOM_REBOOT,
		.owner = ARM_SMCCC_OWNER_OEM,
	};

	desc.args[0] = reboot_type;
	desc.arginfo = QCOM_SCM_ARGS(1);

	return qcom_scm_call_atomic(dev, &desc, NULL);
}

bool qcom_scm_lmh_dcvsh_available(void)
{
	return __qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_LMH, QCOM_SCM_LMH_LIMIT_DCVSH);
}
EXPORT_SYMBOL_GPL(qcom_scm_lmh_dcvsh_available);

int qcom_scm_shm_bridge_enable(void)
{
	int ret;

	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_ENABLE,
		.owner = ARM_SMCCC_OWNER_SIP
	};

	struct qcom_scm_res res;

	if (!__qcom_scm_is_call_available(__scm->dev, QCOM_SCM_SVC_MP,
					  QCOM_SCM_MP_SHM_BRIDGE_ENABLE))
		return -EOPNOTSUPP;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (ret)
		return ret;

	if (res.result[0] == SHMBRIDGE_RESULT_NOTSUPP)
		return -EOPNOTSUPP;

	return res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_shm_bridge_enable);

int qcom_scm_shm_bridge_create(u64 pfn_and_ns_perm_flags,
			       u64 ipfn_and_s_perm_flags, u64 size_and_flags,
			       u64 ns_vmids, u64 *handle)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_CREATE,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = pfn_and_ns_perm_flags,
		.args[1] = ipfn_and_s_perm_flags,
		.args[2] = size_and_flags,
		.args[3] = ns_vmids,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_VAL, QCOM_SCM_VAL,
					 QCOM_SCM_VAL, QCOM_SCM_VAL),
	};

	struct qcom_scm_res res;
	int ret;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	if (handle && !ret)
		*handle = res.result[1];

	return ret ?: res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_shm_bridge_create);

int qcom_scm_shm_bridge_delete(u64 handle)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_SHM_BRIDGE_DELETE,
		.owner = ARM_SMCCC_OWNER_SIP,
		.args[0] = handle,
		.arginfo = QCOM_SCM_ARGS(1, QCOM_SCM_VAL),
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_shm_bridge_delete);

int qcom_scm_lmh_profile_change(u32 profile_id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_LMH,
		.cmd = QCOM_SCM_LMH_LIMIT_PROFILE_CHANGE,
		.arginfo = QCOM_SCM_ARGS(1, QCOM_SCM_VAL),
		.args[0] = profile_id,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_lmh_profile_change);

int qcom_scm_lmh_dcvsh(u32 payload_fn, u32 payload_reg, u32 payload_val,
		       u64 limit_node, u32 node_id, u64 version)
{
	int ret, payload_size = 5 * sizeof(u32);

	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_LMH,
		.cmd = QCOM_SCM_LMH_LIMIT_DCVSH,
		.arginfo = QCOM_SCM_ARGS(5, QCOM_SCM_RO, QCOM_SCM_VAL, QCOM_SCM_VAL,
					QCOM_SCM_VAL, QCOM_SCM_VAL),
		.args[1] = payload_size,
		.args[2] = limit_node,
		.args[3] = node_id,
		.args[4] = version,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	u32 *payload_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							       payload_size,
							       GFP_KERNEL);
	if (!payload_buf)
		return -ENOMEM;

	payload_buf[0] = payload_fn;
	payload_buf[1] = 0;
	payload_buf[2] = payload_reg;
	payload_buf[3] = 1;
	payload_buf[4] = payload_val;

	desc.args[0] = qcom_tzmem_to_phys(payload_buf);

	ret = qcom_scm_call(__scm->dev, &desc, NULL);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_lmh_dcvsh);

int qcom_scm_get_tz_log_feat_id(u64 *version)
{
	return __qcom_scm_get_feat_version(__scm->dev, QCOM_SCM_FEAT_LOG_ID,
					   version);
}
EXPORT_SYMBOL(qcom_scm_get_tz_log_feat_id);

int qcom_scm_get_tz_feat_id_version(u64 feat_id, u64 *version)
{
	return __qcom_scm_get_feat_version(__scm->dev, feat_id,
					   version);
}
EXPORT_SYMBOL(qcom_scm_get_tz_feat_id_version);

int qcom_scm_register_qsee_log_buf(phys_addr_t buf, size_t len)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_QSEELOG,
		.cmd = QCOM_SCM_QSEELOG_REGISTER,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = buf,
		.args[1] = len,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_RW),
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_register_qsee_log_buf);

int qcom_scm_query_encrypted_log_feature(u64 *enabled)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_QSEELOG,
		.cmd = QCOM_SCM_QUERY_ENCR_LOG_FEAT_ID,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	if (!ret)
		*enabled = res.result[0];

	return ret;
}
EXPORT_SYMBOL(qcom_scm_query_encrypted_log_feature);

int qcom_scm_query_log_status(u64 *status)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_QSEELOG,
		.cmd = QCOM_SCM_QUERY_LOG_STATUS,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	if (!ret)
		*status = res.result[0];

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_query_log_status);

int qcom_scm_query_tz_time(u64 *ticks, u32 *frequency)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_QSEELOG,
		.cmd = QCOM_SCM_QUERY_TZ_TIME_ID,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call(__scm->dev, &desc, &res);
	if (!ret) {
		*ticks = ((uint64_t)res.result[0] << 32) | (uint64_t)res.result[1];
		*frequency = (uint32_t)res.result[2];
	}

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_query_tz_time);

int qcom_scm_request_encrypted_log(phys_addr_t buf,
				   size_t len,
				   uint32_t log_id,
				   bool is_full_tz_logs_supported,
				   bool is_full_tz_logs_enabled)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_QSEELOG,
		.cmd = QCOM_SCM_REQUEST_ENCR_LOG_ID,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = buf,
		.args[1] = len,
		.args[2] = log_id
	};
	struct qcom_scm_res res;

	if (is_full_tz_logs_supported) {
		if (is_full_tz_logs_enabled) {
			/* requesting full logs */
			desc.args[3] = 1;
		} else {
			/* requesting incremental logs */
			desc.args[3] = 0;
		}
		desc.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_RW);
	} else {
		desc.arginfo = QCOM_SCM_ARGS(3, QCOM_SCM_RW);
	}
	ret = qcom_scm_call(__scm->dev, &desc, &res);

	return ret ? : res.result[0];
}
EXPORT_SYMBOL(qcom_scm_request_encrypted_log);

int qcom_scm_invoke_smc_legacy(phys_addr_t in_buf, size_t in_buf_size,
		phys_addr_t out_buf, size_t out_buf_size, int32_t *result,
		u64 *response_type, unsigned int *data)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMCINVOKE,
		.cmd = QCOM_SCM_SMCINVOKE_INVOKE_LEGACY,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = in_buf,
		.args[1] = in_buf_size,
		.args[2] = out_buf,
		.args[3] = out_buf_size,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_RW, QCOM_SCM_VAL,
			QCOM_SCM_RW, QCOM_SCM_VAL),
		.multicall_allowed = true,
	};

	struct qcom_scm_res res;

	ret = qcom_scm_call_noretry(__scm->dev, &desc, &res);

	if (result)
		*result = res.result[1];

	if (response_type)
		*response_type = res.result[0];

	if (data)
		*data = res.result[2];

	return ret;
}
EXPORT_SYMBOL(qcom_scm_invoke_smc_legacy);

int qcom_scm_invoke_smc(phys_addr_t in_buf, size_t in_buf_size,
		phys_addr_t out_buf, size_t out_buf_size, int32_t *result,
		u64 *response_type, unsigned int *data)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMCINVOKE,
		.cmd = QCOM_SCM_SMCINVOKE_INVOKE,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = in_buf,
		.args[1] = in_buf_size,
		.args[2] = out_buf,
		.args[3] = out_buf_size,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_RW, QCOM_SCM_VAL,
					QCOM_SCM_RW, QCOM_SCM_VAL),
		.multicall_allowed = true,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call_noretry(__scm->dev, &desc, &res);

	if (result)
		*result = res.result[1];

	if (response_type)
		*response_type = res.result[0];

	if (data)
		*data = res.result[2];

	return ret;
}
EXPORT_SYMBOL(qcom_scm_invoke_smc);

int qcom_scm_invoke_smc_ffa(uint64_t ffa_handle, size_t offset,
		size_t in_buf_size, size_t out_buf_size, int32_t *result,
		u64 *response_type, unsigned int *data)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMCINVOKE,
		.cmd = QCOM_SCM_SMCINVOKE_INVOKE_FFA,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = ffa_handle,
		.args[1] = offset,
		.args[2] = in_buf_size,
		.args[3] = out_buf_size,
		.arginfo = QCOM_SCM_ARGS(4, QCOM_SCM_VAL, QCOM_SCM_VAL,
					QCOM_SCM_VAL, QCOM_SCM_VAL),
		.multicall_allowed = true,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call_noretry(__scm->dev, &desc, &res);

	if (result)
		*result = res.result[1];

	if (response_type)
		*response_type = res.result[0];

	if (data)
		*data = res.result[2];

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_invoke_smc_ffa);

int qcom_scm_invoke_callback_response(phys_addr_t out_buf,
	size_t out_buf_size, int32_t *result, u64 *response_type,
	unsigned int *data)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMCINVOKE,
		.cmd = QCOM_SCM_SMCINVOKE_CB_RSP,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = out_buf,
		.args[1] = out_buf_size,
		.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_RW, QCOM_SCM_VAL),
		.multicall_allowed = true,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call_noretry(__scm->dev, &desc, &res);

	if (result)
		*result = res.result[1];

	if (response_type)
		*response_type = res.result[0];

	if (data)
		*data = res.result[2];

	return ret;
}
EXPORT_SYMBOL(qcom_scm_invoke_callback_response);

int qcom_scm_invoke_callback_response_ffa(uint64_t ffa_handle,
	size_t out_offset, size_t out_buf_size, int32_t *result,
	u64 *response_type, unsigned int *data)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMCINVOKE,
		.cmd = QCOM_SCM_SMCINVOKE_CB_RSP_FFA,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = ffa_handle,
		.args[1] = out_offset,
		.args[2] = out_buf_size,
		.arginfo = QCOM_SCM_ARGS(3, QCOM_SCM_VAL, QCOM_SCM_VAL,
					QCOM_SCM_VAL),
		.multicall_allowed = true,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call_noretry(__scm->dev, &desc, &res);

	if (result)
		*result = res.result[1];

	if (response_type)
		*response_type = res.result[0];

	if (data)
		*data = res.result[2];

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_invoke_callback_response_ffa);

int qcom_scm_invoke_ack_doorbell(u32 doorbell_id, u32 msg_id)
{
	int ret;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_SMCINVOKE,
		.cmd = QCOM_SCM_SMCINVOKE_DOORBELL_ACK,
		.owner = ARM_SMCCC_OWNER_TRUSTED_OS,
		.args[0] = doorbell_id,
		.args[1] = msg_id,
		.arginfo = QCOM_SCM_ARGS(2),
		.multicall_allowed = true,
	};
	struct qcom_scm_res res;

	ret = qcom_scm_call_atomic(__scm->dev, &desc, &res);
	if (ret)
		return ret;

	return res.result[0];
}
EXPORT_SYMBOL_GPL(qcom_scm_invoke_ack_doorbell);

int qcom_scm_qseecom_call(u32 cmd_id, struct qseecom_scm_desc *desc, bool retry)
{
	int ret;
	struct device *dev = __scm ? __scm->dev : NULL;
	struct qcom_scm_desc _desc = {
		.svc = (cmd_id & 0xff00) >> 8,
		.cmd = (cmd_id & 0xff),
		.owner = (cmd_id & 0x3f000000) >> 24,
		.args[0] = desc->args[0],
		.args[1] = desc->args[1],
		.args[2] = desc->args[2],
		.args[3] = desc->args[3],
		.args[4] = desc->args[4],
		.args[5] = desc->args[5],
		.args[6] = desc->args[6],
		.args[7] = desc->args[7],
		.args[8] = desc->args[8],
		.args[9] = desc->args[9],
		.arginfo = desc->arginfo,
	};
	struct qcom_scm_res res;

	if (retry)
		ret = qcom_scm_call(dev, &_desc, &res);
	else
		ret = qcom_scm_call_noretry(dev, &_desc, &res);

	desc->ret[0] = res.result[0];
	desc->ret[1] = res.result[1];
	desc->ret[2] = res.result[2];

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_scm_qseecom_call);

int qcom_scm_gpu_init_regs(u32 gpu_req)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_GPU,
		.cmd = QCOM_SCM_SVC_GPU_INIT_REGS,
		.arginfo = QCOM_SCM_ARGS(1),
		.args[0] = gpu_req,
		.owner = ARM_SMCCC_OWNER_SIP,
	};

	return qcom_scm_call(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_gpu_init_regs);

static int qcom_scm_find_dload_address(struct device *dev, u64 *addr)
{
	struct device_node *tcsr;
	struct device_node *np = dev->of_node;
	struct resource res;
	u32 offset;
	int ret;

	tcsr = of_parse_phandle(np, "qcom,dload-mode", 0);
	if (!tcsr)
		return 0;

	ret = of_address_to_resource(tcsr, 0, &res);
	of_node_put(tcsr);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(np, "qcom,dload-mode", 1, &offset);
	if (ret < 0)
		return ret;

	*addr = res.start + offset;

	return 0;
}

#ifdef CONFIG_QCOM_QSEECOM

/* Lock for QSEECOM SCM call executions */
static DEFINE_MUTEX(qcom_scm_qseecom_call_lock);

static int __qcom_scm_qseecom_call(const struct qcom_scm_desc *desc,
				   struct qcom_scm_qseecom_resp *res)
{
	struct qcom_scm_res scm_res = {};
	int status;

	/*
	 * QSEECOM SCM calls should not be executed concurrently. Therefore, we
	 * require the respective call lock to be held.
	 */
	lockdep_assert_held(&qcom_scm_qseecom_call_lock);

	status = qcom_scm_call(__scm->dev, desc, &scm_res);

	res->result = scm_res.result[0];
	res->resp_type = scm_res.result[1];
	res->data = scm_res.result[2];

	if (status)
		return status;

	return 0;
}

/**
 * qcom_scm_qseecom_call() - Perform a QSEECOM SCM call.
 * @desc: SCM call descriptor.
 * @res:  SCM call response (output).
 *
 * Performs the QSEECOM SCM call described by @desc, returning the response in
 * @rsp.
 *
 * Return: Zero on success, nonzero on failure.
 */
static int qcom_scm_qseecom_call(const struct qcom_scm_desc *desc,
				 struct qcom_scm_qseecom_resp *res)
{
	int status;

	/*
	 * Note: Multiple QSEECOM SCM calls should not be executed same time,
	 * so lock things here. This needs to be extended to callback/listener
	 * handling when support for that is implemented.
	 */

	mutex_lock(&qcom_scm_qseecom_call_lock);
	status = __qcom_scm_qseecom_call(desc, res);
	mutex_unlock(&qcom_scm_qseecom_call_lock);

	dev_dbg(__scm->dev, "%s: owner=%x, svc=%x, cmd=%x, result=%lld, type=%llx, data=%llx\n",
		__func__, desc->owner, desc->svc, desc->cmd, res->result,
		res->resp_type, res->data);

	if (status) {
		dev_err(__scm->dev, "qseecom: scm call failed with error %d\n", status);
		return status;
	}

	/*
	 * TODO: Handle incomplete and blocked calls:
	 *
	 * Incomplete and blocked calls are not supported yet. Some devices
	 * and/or commands require those, some don't. Let's warn about them
	 * prominently in case someone attempts to try these commands with a
	 * device/command combination that isn't supported yet.
	 */
	WARN_ON(res->result == QSEECOM_RESULT_INCOMPLETE);
	WARN_ON(res->result == QSEECOM_RESULT_BLOCKED_ON_LISTENER);

	return 0;
}

/**
 * qcom_scm_qseecom_get_version() - Query the QSEECOM version.
 * @version: Pointer where the QSEECOM version will be stored.
 *
 * Performs the QSEECOM SCM querying the QSEECOM version currently running in
 * the TrustZone.
 *
 * Return: Zero on success, nonzero on failure.
 */
static int qcom_scm_qseecom_get_version(u32 *version)
{
	struct qcom_scm_desc desc = {};
	struct qcom_scm_qseecom_resp res = {};
	u32 feature = 10;
	int ret;

	desc.owner = QSEECOM_TZ_OWNER_SIP;
	desc.svc = QSEECOM_TZ_SVC_INFO;
	desc.cmd = QSEECOM_TZ_CMD_INFO_VERSION;
	desc.arginfo = QCOM_SCM_ARGS(1, QCOM_SCM_VAL);
	desc.args[0] = feature;

	ret = qcom_scm_qseecom_call(&desc, &res);
	if (ret)
		return ret;

	*version = res.result;
	return 0;
}

/**
 * qcom_scm_qseecom_app_get_id() - Query the app ID for a given QSEE app name.
 * @app_name: The name of the app.
 * @app_id:   The returned app ID.
 *
 * Query and return the application ID of the SEE app identified by the given
 * name. This returned ID is the unique identifier of the app required for
 * subsequent communication.
 *
 * Return: Zero on success, nonzero on failure, -ENOENT if the app has not been
 * loaded or could not be found.
 */
int qcom_scm_qseecom_app_get_id(const char *app_name, u32 *app_id)
{
	unsigned long name_buf_size = QSEECOM_MAX_APP_NAME_SIZE;
	unsigned long app_name_len = strlen(app_name);
	struct qcom_scm_desc desc = {};
	struct qcom_scm_qseecom_resp res = {};
	int status;

	if (app_name_len >= name_buf_size)
		return -EINVAL;

	char *name_buf __free(qcom_tzmem) = qcom_tzmem_alloc(__scm->mempool,
							     name_buf_size,
							     GFP_KERNEL);
	if (!name_buf)
		return -ENOMEM;

	memcpy(name_buf, app_name, app_name_len);

	desc.owner = QSEECOM_TZ_OWNER_QSEE_OS;
	desc.svc = QSEECOM_TZ_SVC_APP_MGR;
	desc.cmd = QSEECOM_TZ_CMD_APP_LOOKUP;
	desc.arginfo = QCOM_SCM_ARGS(2, QCOM_SCM_RW, QCOM_SCM_VAL);
	desc.args[0] = qcom_tzmem_to_phys(name_buf);
	desc.args[1] = app_name_len;

	status = qcom_scm_qseecom_call(&desc, &res);

	if (status)
		return status;

	if (res.result == QSEECOM_RESULT_FAILURE)
		return -ENOENT;

	if (res.result != QSEECOM_RESULT_SUCCESS)
		return -EINVAL;

	if (res.resp_type != QSEECOM_SCM_RES_APP_ID)
		return -EINVAL;

	*app_id = res.data;
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_qseecom_app_get_id);

/**
 * qcom_scm_qseecom_app_send() - Send to and receive data from a given QSEE app.
 * @app_id:   The ID of the target app.
 * @req:      Request buffer sent to the app (must be TZ memory)
 * @req_size: Size of the request buffer.
 * @rsp:      Response buffer, written to by the app (must be TZ memory)
 * @rsp_size: Size of the response buffer.
 *
 * Sends a request to the QSEE app associated with the given ID and read back
 * its response. The caller must provide two DMA memory regions, one for the
 * request and one for the response, and fill out the @req region with the
 * respective (app-specific) request data. The QSEE app reads this and returns
 * its response in the @rsp region.
 *
 * Return: Zero on success, nonzero on failure.
 */
int qcom_scm_qseecom_app_send(u32 app_id, void *req, size_t req_size,
			      void *rsp, size_t rsp_size)
{
	struct qcom_scm_qseecom_resp res = {};
	struct qcom_scm_desc desc = {};
	phys_addr_t req_phys;
	phys_addr_t rsp_phys;
	int status;

	req_phys = qcom_tzmem_to_phys(req);
	rsp_phys = qcom_tzmem_to_phys(rsp);

	desc.owner = QSEECOM_TZ_OWNER_TZ_APPS;
	desc.svc = QSEECOM_TZ_SVC_APP_ID_PLACEHOLDER;
	desc.cmd = QSEECOM_TZ_CMD_APP_SEND;
	desc.arginfo = QCOM_SCM_ARGS(5, QCOM_SCM_VAL,
				     QCOM_SCM_RW, QCOM_SCM_VAL,
				     QCOM_SCM_RW, QCOM_SCM_VAL);
	desc.args[0] = app_id;
	desc.args[1] = req_phys;
	desc.args[2] = req_size;
	desc.args[3] = rsp_phys;
	desc.args[4] = rsp_size;

	status = qcom_scm_qseecom_call(&desc, &res);

	if (status)
		return status;

	if (res.result != QSEECOM_RESULT_SUCCESS)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_scm_qseecom_app_send);

/*
 * We do not yet support re-entrant calls via the qseecom interface. To prevent
 + any potential issues with this, only allow validated machines for now.
 */
static const struct of_device_id qcom_scm_qseecom_allowlist[] __maybe_unused = {
	{ .compatible = "lenovo,flex-5g" },
	{ .compatible = "lenovo,thinkpad-x13s", },
	{ .compatible = "qcom,sc8180x-primus" },
	{ .compatible = "qcom,x1e80100-crd" },
	{ .compatible = "qcom,x1e80100-qcp" },
	{ }
};

static bool qcom_scm_qseecom_machine_is_allowed(void)
{
	struct device_node *np;
	bool match;

	np = of_find_node_by_path("/");
	if (!np)
		return false;

	match = of_match_node(qcom_scm_qseecom_allowlist, np);
	of_node_put(np);

	return match;
}

static void qcom_scm_qseecom_free(void *data)
{
	struct platform_device *qseecom_dev = data;

	platform_device_del(qseecom_dev);
	platform_device_put(qseecom_dev);
}

static int qcom_scm_qseecom_init(struct qcom_scm *scm)
{
	struct platform_device *qseecom_dev;
	u32 version;
	int ret;

	/*
	 * Note: We do two steps of validation here: First, we try to query the
	 * QSEECOM version as a check to see if the interface exists on this
	 * device. Second, we check against known good devices due to current
	 * driver limitations (see comment in qcom_scm_qseecom_allowlist).
	 *
	 * Note that we deliberately do the machine check after the version
	 * check so that we can log potentially supported devices. This should
	 * be safe as downstream sources indicate that the version query is
	 * neither blocking nor reentrant.
	 */
	ret = qcom_scm_qseecom_get_version(&version);
	if (ret)
		return 0;

	dev_info(scm->dev, "qseecom: found qseecom with version 0x%x\n", version);

	if (!qcom_scm_qseecom_machine_is_allowed()) {
		dev_info(scm->dev, "qseecom: untested machine, skipping\n");
		return 0;
	}

	/*
	 * Set up QSEECOM interface device. All application clients will be
	 * set up and managed by the corresponding driver for it.
	 */
	qseecom_dev = platform_device_alloc("qcom_qseecom", -1);
	if (!qseecom_dev)
		return -ENOMEM;

	qseecom_dev->dev.parent = scm->dev;

	ret = platform_device_add(qseecom_dev);
	if (ret) {
		platform_device_put(qseecom_dev);
		return ret;
	}

	return devm_add_action_or_reset(scm->dev, qcom_scm_qseecom_free, qseecom_dev);
}

#else /* CONFIG_QCOM_QSEECOM */

static int qcom_scm_qseecom_init(struct qcom_scm *scm)
{
	return 0;
}

#endif /* CONFIG_QCOM_QSEECOM */

/**
 * qcom_scm_is_available() - Checks if SCM is available
 */
bool qcom_scm_is_available(void)
{
	return !!READ_ONCE(__scm);
}
EXPORT_SYMBOL_GPL(qcom_scm_is_available);

static int qcom_scm_do_restart(struct notifier_block *this, unsigned long event,
			      void *ptr)
{
	struct qcom_scm *scm = container_of(this, struct qcom_scm, restart_nb);
	char *cmd = ptr ? ptr : "normal";

	if (reboot_mode == REBOOT_WARM &&
		qcom_scm_custom_reset_type == QCOM_SCM_RST_NONE)
		qcom_scm_reboot(scm->dev);

	else if (cmd && !strcmp(cmd, "rtc"))
		qcom_scm_custom_reset_type = QCOM_SCM_RST_SHUTDOWN_TO_RTC_MODE;

	else if (cmd && !strcmp(cmd, "twm"))
		qcom_scm_custom_reset_type = QCOM_SCM_RST_SHUTDOWN_TO_TWM_MODE;

	if (qcom_scm_custom_reset_type > QCOM_SCM_RST_NONE &&
		qcom_scm_custom_reset_type < QCOM_SCM_RST_MAX)
		qcom_scm_custom_reboot(scm->dev, qcom_scm_custom_reset_type);

	return NOTIFY_OK;
}

static int qcom_scm_fill_irq_fwspec_params(struct irq_fwspec *fwspec, u32 virq)
{
	if (virq >= GIC_SPI_BASE && virq <= GIC_MAX_SPI) {
		fwspec->param[0] = GIC_SPI;
		fwspec->param[1] = virq - GIC_SPI_BASE;
	} else if (virq >= GIC_ESPI_BASE && virq <= GIC_MAX_ESPI) {
		fwspec->param[0] = GIC_ESPI;
		fwspec->param[1] = virq - GIC_ESPI_BASE;
	} else {
		WARN(1, "Unexpected virq: %d\n", virq);
		return -ENXIO;
	}
	fwspec->param[2] = IRQ_TYPE_EDGE_RISING;
	fwspec->param_count = 3;

	return 0;
}

static int qcom_scm_query_wq_queue_info(struct qcom_scm *scm)
{
	int ret;
	u32 hwirq;
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_WAITQ,
		.cmd = QCOM_SCM_GET_WQ_QUEUE_INFO,
		.owner = ARM_SMCCC_OWNER_SIP
	};
	struct qcom_scm_res res;
	struct irq_fwspec fwspec;
	struct device_node *parent_irq_node;

	scm->waitq.wq_feature = QCOM_SCM_SINGLE_SMC_ALLOW;
	ret = qcom_scm_call_atomic(__scm->dev, &desc, &res);
	if (ret) {
		pr_err("%s: Failed to get wq queue info: %d\n", __func__, ret);
		return ret;
	}

	scm->waitq.call_ctx_cnt = res.result[0] & 0xFF;
	hwirq = res.result[1] & 0xFFFF;
	scm->waitq.wq_feature = QCOM_SCM_MULTI_SMC_WHITE_LIST_ALLOW;

	ret = qcom_scm_fill_irq_fwspec_params(&fwspec, hwirq);
	if (ret)
		return ret;
	parent_irq_node = of_irq_find_parent(__scm->dev->of_node);

	fwspec.fwnode = of_node_to_fwnode(parent_irq_node);

	scm->waitq.irq = irq_create_fwspec_mapping(&fwspec);

	pr_info("WQ Info, feature: %d call_ctx_cnt: %llu irq: %llu\n",
		scm->waitq.wq_feature, scm->waitq.call_ctx_cnt, scm->waitq.irq);

	return ret;
}

int qcom_scm_set_gic_cpuclass(u32 mpidr, u32 clss)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_GIC,
		.cmd =  QCOM_SCM_GIC_SET_CPUCLASS,
		.arginfo = QCOM_SCM_ARGS(2),
		.args[0] = mpidr,
		.args[1] = clss,
		.owner = ARM_SMCCC_OWNER_SIP
	};

	return qcom_scm_call_atomic(__scm->dev, &desc, NULL);
}
EXPORT_SYMBOL_GPL(qcom_scm_set_gic_cpuclass);

bool qcom_scm_multi_call_allow(struct device *dev, bool multicall_allowed)
{
	struct qcom_scm *scm;

	scm = dev_get_drvdata(dev);
	if (multicall_allowed &&
		scm->waitq.wq_feature == QCOM_SCM_MULTI_SMC_WHITE_LIST_ALLOW)
		return true;

	return false;
};

struct completion *qcom_scm_lookup_wq(struct qcom_scm *scm, u32 wq_ctx)
{
	struct completion *wq = NULL;
	unsigned long flags;
	int err;

	spin_lock_irqsave(&scm->waitq.idr_lock, flags);
	wq = idr_find(&scm->waitq.idr, wq_ctx);
	if (wq)
		goto out;

	wq = devm_kzalloc(scm->dev, sizeof(*wq), GFP_ATOMIC);
	if (!wq) {
		wq = ERR_PTR(-ENOMEM);
		goto out;
	}

	init_completion(wq);

	err = idr_alloc_u32(&scm->waitq.idr, wq, &wq_ctx, wq_ctx, GFP_ATOMIC);
	if (err < 0) {
		devm_kfree(scm->dev, wq);
		wq = ERR_PTR(err);
	}

out:
	spin_unlock_irqrestore(&scm->waitq.idr_lock, flags);
	return wq;
}

void scm_waitq_flag_handler(struct completion *wq, u32 flags)
{
	switch (flags) {
	case QCOM_SMC_WAITQ_FLAG_WAKE_ONE:
		complete(wq);
		break;
	case QCOM_SMC_WAITQ_FLAG_WAKE_ALL:
		complete_all(wq);
		reinit_completion(wq);
		break;
	default:
		pr_err("invalid flags: %u\n", flags);
	}
}

static void scm_irq_work(struct work_struct *work)
{
	int ret;
	u32 wq_ctx, flags, more_pending = 0;
	struct completion *wq_to_wake;
	struct qcom_scm_waitq *w = container_of(work, struct qcom_scm_waitq, scm_irq_work);
	struct qcom_scm *scm = container_of(w, struct qcom_scm, waitq);
	bool multi_smc = (scm->waitq.wq_feature == QCOM_SCM_MULTI_SMC_WHITE_LIST_ALLOW);

	if (qcom_scm_convention != SMC_CONVENTION_ARM_64) {
		/* Unsupported */
		return;
	}

	do {
		ret = scm_get_wq_ctx(&wq_ctx, &flags, &more_pending, multi_smc);
		if (ret) {
			pr_err("GET_WQ_CTX SMC call failed: %d\n", ret);
			return;
		}

		/* This happens if two wakeups occur in close succession */
		if (flags == QCOM_SCM_WAITQ_FLAG_WAKE_NONE)
			return;

		wq_to_wake = qcom_scm_lookup_wq(scm, wq_ctx);
		if (IS_ERR_OR_NULL(wq_to_wake)) {
			pr_err("No waitqueue found for wq_ctx %d: %ld\n",
					wq_ctx, PTR_ERR(wq_to_wake));
			return;
		}

		scm_waitq_flag_handler(wq_to_wake, flags);
	} while (more_pending);
}

static irqreturn_t qcom_scm_irq_handler(int irq, void *p)
{
	struct qcom_scm *scm = p;

	schedule_work(&scm->waitq.scm_irq_work);

	return IRQ_HANDLED;
}

static int __qcom_multi_smc_init(struct qcom_scm *__scm,
						struct platform_device *pdev)
{
	int ret = 0, irq;

	spin_lock_init(&__scm->waitq.idr_lock);
	idr_init(&__scm->waitq.idr);
	if (of_device_is_compatible(__scm->dev->of_node, "qcom,scm-v1.1")) {
		INIT_WORK(&__scm->waitq.scm_irq_work, scm_irq_work);

		/* Detect Multi SMC support present or not */
		ret = qcom_scm_query_wq_queue_info(__scm);
		if (!ret) {
			irq = __scm->waitq.irq;
			sema_init(&qcom_scm_sem_lock,
					(int)__scm->waitq.call_ctx_cnt);
		} else {
			irq = platform_get_irq(pdev, 0);
			if (irq < 0) {
				dev_err(__scm->dev, "WQ IRQ is not specified: %d\n", irq);
				return irq;
			}
		}
		ret = devm_request_irq(__scm->dev, irq,
				qcom_scm_irq_handler,
				IRQF_ONESHOT, "qcom-scm", __scm);
		if (ret < 0) {
			dev_err(__scm->dev, "Failed to request qcom-scm irq: %d\n", ret);
			return ret;
		}
		irq_set_irq_wake(irq, true);
	}

	return ret;
}

/**
 * scm_mem_protection_init_do() - Makes core kernel bootup milestone call
 *                                to Kernel Protect (KP) in Hypervisor
 *                                to start kernel memory protection. KP will
 *                                start protection on kernel sections like
 *                                .text, .rodata, .bss, .data with applying
 *                                permissions in EL2 page table.
 *
 * @pid_offset:       Offset of PID in task_struct structure to pass in
 *                    hypervisor syscall.
 * @task_name_offset: Offset of task name in task_struct structure to pass in
 *                    hypervisor syscall.
 *
 * Returns 0 on success.
 */
int  scm_mem_protection_init_do(void)
{
	int ret = 0, resp;
	uint32_t pid_offset = 0;
	uint32_t task_name_offset = 0;
	struct qcom_scm_desc desc = {
		.svc = SCM_SVC_RTIC,
		.cmd = TZ_HLOS_NOTIFY_CORE_KERNEL_BOOTUP,
		.owner = ARM_SMCCC_OWNER_SIP,
		.arginfo = QCOM_SCM_ARGS(2),
	};

	struct qcom_scm_res res;

	if (!__scm) {
		pr_err("SCM dev is not initialized\n");
		return -1;
	}

	/*
	 * Fetching offset of PID and task_name from task_struct.
	 * This will be used by fault handler of Kernel Protect (KP)
	 * in hypervisor to read PID and task name of process for
	 * which KP fault handler is triggered. This is required to
	 * record PID and task name in integrity report of kernel.
	 */
	pid_offset = offsetof(struct task_struct, pid);
	task_name_offset = offsetof(struct task_struct, comm);

	pr_debug("offset of pid is %u, offset of comm is %u\n",
			pid_offset, task_name_offset);
	desc.args[0] = pid_offset,
	desc.args[1] = task_name_offset,

	ret = qcom_scm_call(__scm ? __scm->dev : NULL, &desc, &res);
	resp = res.result[0];

	pr_debug("SCM call values: ret %d, resp %d\n",
			ret, resp);

	if (ret || resp) {
		pr_err("SCM call failed %d, resp %d\n", ret, resp);
		if (ret)
			return ret;
	}

	return resp;
}

static int qcom_scm_probe(struct platform_device *pdev)
{
	struct qcom_tzmem_pool_config pool_config;
	struct qcom_scm *scm;
	int ret;

	scm = devm_kzalloc(&pdev->dev, sizeof(*scm), GFP_KERNEL);
	if (!scm)
		return -ENOMEM;

	scm->dev = &pdev->dev;
	ret = qcom_scm_find_dload_address(&pdev->dev, &scm->dload_mode_addr);
	if (ret < 0)
		return ret;

	init_completion(&scm->waitq_comp);
	mutex_init(&scm->scm_bw_lock);

	scm->path = devm_of_icc_get(&pdev->dev, NULL);
	if (IS_ERR(scm->path))
		return dev_err_probe(&pdev->dev, PTR_ERR(scm->path),
				     "failed to acquire interconnect path\n");

	scm->core_clk = devm_clk_get_optional(&pdev->dev, "core");
	if (IS_ERR(scm->core_clk))
		return PTR_ERR(scm->core_clk);

	scm->iface_clk = devm_clk_get_optional(&pdev->dev, "iface");
	if (IS_ERR(scm->iface_clk))
		return PTR_ERR(scm->iface_clk);

	scm->bus_clk = devm_clk_get_optional(&pdev->dev, "bus");
	if (IS_ERR(scm->bus_clk))
		return PTR_ERR(scm->bus_clk);

	scm->reset.ops = &qcom_scm_pas_reset_ops;
	scm->reset.nr_resets = 1;
	scm->reset.of_node = pdev->dev.of_node;
	ret = devm_reset_controller_register(&pdev->dev, &scm->reset);
	if (ret)
		return ret;

	/* vote for max clk rate for highest performance */
	ret = clk_set_rate(scm->core_clk, INT_MAX);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	platform_set_drvdata(pdev, scm);

	/* Let all above stores be available after this */
	smp_store_release(&__scm, scm);

	/* unification to make sure scm transactions go over HAB channel */
	if (of_property_read_bool(pdev->dev.of_node, "qcom,scm-hab"))
		__qcom_scm_init();

	__get_convention();
	ret  = __qcom_multi_smc_init(scm, pdev);
	if (ret)
		return ret;

	scm->restart_nb.notifier_call = qcom_scm_do_restart;
	scm->restart_nb.priority = 130;
	register_restart_handler(&scm->restart_nb);

	if (scm->dload_mode_addr &&
	    IS_ERR(platform_device_register_data(&pdev->dev, "qcom-dload-mode",
						 PLATFORM_DEVID_NONE, NULL, 0)))
		dev_err(&pdev->dev, "failed to register qcom dload device\n");

	/*
	 * If requested enable "download mode", from this point on warmboot
	 * will cause the boot stages to enter download mode, unless
	 * disabled below by a clean shutdown/reboot.
	 */
	if (download_mode)
		qcom_scm_set_download_mode(QCOM_DOWNLOAD_FULLDUMP);

	/*
	 * Disable SDI if indicated by DT that it is enabled by default.
	 */
	if (of_property_read_bool(pdev->dev.of_node, "qcom,sdi-enabled"))
		qcom_scm_disable_sdi();

	ret = of_reserved_mem_device_init(__scm->dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(__scm->dev, ret,
				     "Failed to setup the reserved memory region for TZ mem\n");

	ret = qcom_tzmem_enable(__scm->dev);
	if (ret)
		return dev_err_probe(__scm->dev, ret,
				     "Failed to enable the TrustZone memory allocator\n");

	memset(&pool_config, 0, sizeof(pool_config));
	pool_config.initial_size = 0;
	pool_config.policy = QCOM_TZMEM_POLICY_ON_DEMAND;
	pool_config.max_size = SZ_256K;

	__scm->mempool = devm_qcom_tzmem_pool_new(__scm->dev, &pool_config);
	if (IS_ERR(__scm->mempool))
		return dev_err_probe(__scm->dev, PTR_ERR(__scm->mempool),
				     "Failed to create the SCM memory pool\n");

	/*
	 * Initialize the QSEECOM interface.
	 *
	 * Note: QSEECOM is fairly self-contained and this only adds the
	 * interface device (the driver of which does most of the heavy
	 * lifting). So any errors returned here should be either -ENOMEM or
	 * -EINVAL (with the latter only in case there's a bug in our code).
	 * This means that there is no need to bring down the whole SCM driver.
	 * Just log the error instead and let SCM live.
	 */
	ret = qcom_scm_qseecom_init(scm);
	WARN(ret < 0, "failed to initialize qseecom: %d\n", ret);

	return qtee_shmbridge_driver_init();
}

static void qcom_scm_shutdown(struct platform_device *pdev)
{
	idr_destroy(&__scm->waitq.idr);
	qcom_scm_disable_sdi();
	qcom_scm_halt_spmi_pmic_arbiter();
	/* Clean shutdown, disable download mode to allow normal restart */
	if (download_mode)
		qcom_scm_set_download_mode(QCOM_DOWNLOAD_NODUMP);
}

static const struct of_device_id qcom_scm_dt_match[] = {
	{ .compatible = "qcom,scm" },

	/* Legacy entries kept for backwards compatibility */
	{ .compatible = "qcom,scm-apq8064" },
	{ .compatible = "qcom,scm-apq8084" },
	{ .compatible = "qcom,scm-ipq4019" },
	{ .compatible = "qcom,scm-msm8953" },
	{ .compatible = "qcom,scm-msm8974" },
	{ .compatible = "qcom,scm-msm8996" },
	{ .compatible = "qcom,scm-v1.1" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_scm_dt_match);

static struct platform_driver qcom_scm_driver = {
	.driver = {
		.name	= "qcom_scm",
		.of_match_table = qcom_scm_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = qcom_scm_probe,
	.shutdown = qcom_scm_shutdown,
};

static int __init qcom_scm_init(void)
{
	return platform_driver_register(&qcom_scm_driver);
}
subsys_initcall(qcom_scm_init);

#if IS_MODULE(CONFIG_QCOM_SCM)
static void __exit qcom_scm_exit(void)
{
	__qcom_scm_qcpe_exit();
	qtee_shmbridge_driver_exit();
	platform_driver_unregister(&qcom_scm_driver);
}
module_exit(qcom_scm_exit);
#endif

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. SCM driver");
MODULE_LICENSE("GPL v2");
