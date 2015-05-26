/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTE: This file has been modified by Sony Mobile Communications AB.
 * Modifications are licensed under the License.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/i2c/i2c-qup.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/memory.h>
#include <linux/memblock.h>
#include <linux/regulator/cpr-regulator.h>
#include <linux/regulator/fan53555.h>
#include <linux/regulator/onsemi-ncp6335d.h>
#include <linux/regulator/qpnp-regulator.h>
#include <linux/msm_tsens.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <mach/board.h>
#include <mach/msm_bus.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#include <mach/restart.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/socinfo.h>
#include <mach/board.h>
#include <mach/clk-provider.h>
#include <mach/msm_smd.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/msm_smem.h>
#include <mach/msm_memory_dump.h>
#include <linux/msm_thermal.h>
#include "board-dt.h"
#include "clock.h"
#include "platsmp.h"
#include "spm.h"
#include "pm.h"
#include "modem_notifier.h"
#include "spm-regulator.h"

#ifdef CONFIG_RAMDUMP_TAGS
#include "board-rdtags.h"
#endif

static struct memtype_reserve msm8226_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static int msm8226_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct of_dev_auxdata msm_hsic_host_adata[] = {
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),
	{}
};

#ifdef CONFIG_RAMDUMP_TAGS
static struct resource rdtags_resources[] = {
	[0] = {
		.name   = "rdtags_mem",
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device rdtags_device = {
	.name           = "rdtags",
	.id             = -1,
	.dev = {
		.platform_data = &rdtags_platdata,
	},
};
#endif

#ifdef CONFIG_CRASH_LAST_LOGS

static struct resource lastlogs_resources[] = {
	[0] = {
		.name	= "last_kmsg",
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "last_amsslog",
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device lastlogs_device = {
	.name           = "last_logs",
	.id             = -1,
};
#endif

#define RDTAGS_MEM_SIZE (256 * SZ_1K)
#define RDTAGS_MEM_DESC_SIZE (256 * SZ_1K)
#define LAST_LOGS_OFFSET (RDTAGS_MEM_SIZE + RDTAGS_MEM_DESC_SIZE)

#ifdef CONFIG_CRASH_LAST_LOGS
#define LAST_LOG_HEADER_SIZE 4096
#define KMSG_LOG_SIZE ((1 << CONFIG_LOG_BUF_SHIFT) + LAST_LOG_HEADER_SIZE)
#define AMSS_LOG_SIZE ((16 * SZ_1K) + LAST_LOG_HEADER_SIZE)
#endif

#if defined(CONFIG_RAMDUMP_TAGS) || defined(CONFIG_CRASH_LAST_LOGS)
static void reserve_debug_memory(void)
{
	struct membank *mb = &meminfo.bank[meminfo.nr_banks - 1];
	unsigned long bank_end = mb->start + mb->size;
	/*Base address for rdtags*/
	unsigned long debug_mem_base = bank_end - SZ_1M;
	/*Base address for crash logs memory*/
	unsigned long lastlogs_base = debug_mem_base + LAST_LOGS_OFFSET;

	memblock_free(debug_mem_base, SZ_1M);
	memblock_remove(debug_mem_base, SZ_1M);
#ifdef CONFIG_RAMDUMP_TAGS
	rdtags_resources[0].start = debug_mem_base;
	rdtags_resources[0].end = debug_mem_base + RDTAGS_MEM_SIZE - 1;
	debug_mem_base += RDTAGS_MEM_SIZE;
	pr_info("Rdtags start %x end %x\n", \
		(unsigned int)rdtags_resources[0].start, \
		(unsigned int)rdtags_resources[0].end);
	rdtags_device.num_resources = ARRAY_SIZE(rdtags_resources);
	rdtags_device.resource = rdtags_resources;
#endif
#ifdef CONFIG_CRASH_LAST_LOGS
	lastlogs_resources[0].start = lastlogs_base;
	lastlogs_resources[0].end = lastlogs_base + KMSG_LOG_SIZE - 1;
	lastlogs_base += KMSG_LOG_SIZE;
	pr_info("last_kmsg start %x end %x\n", \
		(unsigned int)lastlogs_resources[0].start, \
		(unsigned int)lastlogs_resources[0].end);

	lastlogs_resources[1].start = lastlogs_base;
	lastlogs_resources[1].end = lastlogs_base + AMSS_LOG_SIZE - 1;
	lastlogs_device.num_resources = ARRAY_SIZE(lastlogs_resources);
	lastlogs_device.resource = lastlogs_resources;
	pr_info("last_amsslog start %x end %x\n", \
		(unsigned int)lastlogs_resources[1].start, \
		(unsigned int)lastlogs_resources[1].end);
#endif
}
#endif

static struct of_dev_auxdata msm8226_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),
	OF_DEV_AUXDATA("qcom,hsic-smsc-hub", 0, "msm_smsc_hub",
			msm_hsic_host_adata),

	{}
};

static struct reserve_info msm8226_reserve_info __initdata = {
	.memtype_reserve_table = msm8226_reserve_table,
	.paddr_to_memtype = msm8226_paddr_to_memtype,
};

static void __init msm8226_early_memory(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_hole, msm8226_reserve_table);
}

void __init msm8226_init_early(void)
{
	msm_reserve_last_regs();
}

void __init msm8226_add_devices(void)
{
#ifdef CONFIG_RAMDUMP_TAGS
	platform_device_register(&rdtags_device);
#endif
#ifdef CONFIG_CRASH_LAST_LOGS
	platform_device_register(&lastlogs_device);
#endif
}

static void __init msm8226_reserve(void)
{
#if defined(CONFIG_RAMDUMP_TAGS) || defined(CONFIG_CRASH_LAST_LOGS)
	reserve_debug_memory();
#endif
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_reserve, msm8226_reserve_table);
	msm_reserve();
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8226_add_drivers(void)
{
	msm_smem_init();
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	qpnp_regulator_init();
	spm_regulator_init();
	if (of_board_is_rumi())
		msm_clock_init(&msm8226_rumi_clock_init_data);
	else
		msm_clock_init(&msm8226_clock_init_data);
	msm_bus_fabric_init_driver();
	qup_i2c_init_driver();
	ncp6335d_regulator_init();
	fan53555_regulator_init();
	cpr_regulator_init();
	tsens_tm_init_driver();
	msm_thermal_device_init();
}

void __init msm8226_init(void)
{
	struct of_dev_auxdata *adata = msm8226_auxdata_lookup;

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	msm8226_init_gpiomux();
	board_dt_populate(adata);
	msm8226_add_devices();
	msm8226_add_drivers();
}

static const char *msm8226_dt_match[] __initconst = {
	"qcom,msm8226",
	"qcom,msm8926",
	"qcom,apq8026",
	NULL
};

DT_MACHINE_START(MSM8226_DT, "Qualcomm MSM 8x26 / MSM 8x28 (Flattened Device Tree)")
	.map_io = msm_map_msm8226_io,
	.init_irq = msm_dt_init_irq,
	.init_machine = msm8226_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = msm8226_dt_match,
	.reserve = msm8226_reserve,
	.init_very_early = msm8226_early_memory,
	.init_early = msm8226_init_early,
	.restart = msm_restart,
	.smp = &arm_smp_ops,
MACHINE_END
