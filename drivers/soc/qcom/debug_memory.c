 /*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/memory.h>
#include <linux/memblock.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#endif
#ifdef CONFIG_RAMDUMP_TAGS
#include <linux/rdtags.h>
#include "board-rdtags.h"
#endif

#define RAMDUMP_MEMDESC_SIZE (256 * SZ_1K)
#define RDTAGS_MEM_SIZE (256 * SZ_1K)
#define HEADER_SIZE (4 * SZ_1K)
#define SUBSYS_ERR_LOG_SIZE ((16 * SZ_1K) + HEADER_SIZE)

#define RDTAGS_OFFSET (RAMDUMP_MEMDESC_SIZE)
#define AMSS_LOGS_OFFSET (RAMDUMP_MEMDESC_SIZE + RDTAGS_MEM_SIZE)
#define ADSP_LOGS_OFFSET (AMSS_LOGS_OFFSET + SUBSYS_ERR_LOG_SIZE)

static int ramdump_mode;
static unsigned long debug_mem_base;
static unsigned long debug_mem_size;

static int is_aligned(unsigned long addr, unsigned long align)
{
	return !(addr & (align - 1));
}

static int __init warm_boot_setup(char *p)
{
	unsigned long res;

	if (!p || !*p)
		return 0;

	if (!kstrtoul(p, 0 , &res)) {
		if (res == 0xC0DEDEAD || res == 0xABADBABE)
			ramdump_mode = 1;
	}

	pr_info("board-ramdump: boot mode detected as %s\n",
		ramdump_mode ? "ramdump" : "normal");
	return 0;
}
early_param("warmboot", warm_boot_setup);

#ifdef CONFIG_OF
static const struct of_device_id ramdump_dt[] = {
		{ .compatible = "qcom,debug_memory" },
			{}
};
#endif

int __init alloc_debug_memory(void)
{
	struct device_node *node;
	uint32_t *regs = NULL;
	size_t cells;

	if (ramdump_mode)
		return 0;

	node = of_find_matching_node(NULL, ramdump_dt);
	if (!node) {
		pr_err("debug region node not found\n");
		return -EINVAL;
	}

	cells = of_n_addr_cells(node) + of_n_size_cells(node);
	regs = kcalloc(cells, sizeof(uint32_t), GFP_KERNEL);
	if (!regs) {
		pr_err("Failed to allocate memory for cells\n");
		return -ENOMEM;
	}

	if (of_property_read_u32_array(node, "reg", regs, cells)) {
		pr_err("unable to find base address of node in dtb\n");
		return -EINVAL;
	}

	if (cells == 4) {
		debug_mem_base = (unsigned long)regs[0] << 32 | regs[1];
		debug_mem_size = (unsigned long)regs[2] << 32 | regs[3];
	} else if (cells == 2) {
		debug_mem_base = regs[0];
		debug_mem_size = regs[1];
	} else
		pr_err("bad number of cells in the regs property\n");

	pr_info("board-ramdump: allocated debug memory at %lx-%lx\n",
		debug_mem_base, debug_mem_base + debug_mem_size - 1);
	of_node_put(node);
	kfree(regs);

	return 0;
}

static void __init ramdump_debug_memory_init(void)
{
	unsigned long addr;
	void __iomem *imem_base;
	struct device_node *imem_node;

	imem_node = of_find_compatible_node(NULL, NULL,
			     "qcom,msm-imem-debug_base");
	if (!imem_node) {
		pr_err("debug base in DT does not exist\n");
		return;
	}

	imem_base = of_iomap(imem_node, 0);
	if (!imem_base) {
		pr_err("debug base imem offset mapping failed\n");
		return;
	}

	addr = readl_relaxed(imem_base);

	if (ramdump_mode) {
		if (addr && is_aligned(addr, SZ_1M))
			debug_mem_base = addr;
		else
			debug_mem_base = 0;
		writel_relaxed(0, imem_base);
	} else
		writel_relaxed(debug_mem_base, imem_base);

	iounmap(imem_base);
}

#ifdef CONFIG_RAMDUMP_MEMDESC
static struct resource ramdump_memdesc_resources[] = {
	[0] = {
		.name   = "ramdump_memdesc",
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ramdump_memdesc_device = {
	.name           = "ramdump_memdesc",
	.id             = -1,
};
#endif

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

#ifdef CONFIG_SUBSYS_LAST_ERR_LOG
static struct resource last_subsyslog_resources[] = {
	[0] = {
		.name	= "amsslog",
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "adsplog",
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device last_subsyslog_device = {
	.name           = "last_subsyslog",
	.id             = -1,
};
#endif

static void __init ramdump_add_devices(void)
{
	if (!debug_mem_base)
		return;

#ifdef CONFIG_RAMDUMP_MEMDESC
	ramdump_memdesc_resources[0].start = debug_mem_base;
	ramdump_memdesc_resources[0].end = ramdump_memdesc_resources[0].start +
					RAMDUMP_MEMDESC_SIZE - 1;
	ramdump_memdesc_device.num_resources =
					ARRAY_SIZE(ramdump_memdesc_resources);
	ramdump_memdesc_device.resource = ramdump_memdesc_resources;
	ramdump_memdesc_device.dev.platform_data = &ramdump_mode;

	platform_device_register(&ramdump_memdesc_device);
#endif

#ifdef CONFIG_RAMDUMP_TAGS
	rdtags_resources[0].start = debug_mem_base + RDTAGS_OFFSET;
	rdtags_resources[0].end = rdtags_resources[0].start +
				  RDTAGS_MEM_SIZE - 1;
	rdtags_device.num_resources = ARRAY_SIZE(rdtags_resources);
	rdtags_device.resource = rdtags_resources;
	rdtags_platdata.ramdump_mode = ramdump_mode;

	platform_device_register(&rdtags_device);
#endif

#ifdef CONFIG_SUBSYS_LAST_ERR_LOG
	last_subsyslog_resources[0].start = debug_mem_base + AMSS_LOGS_OFFSET;
	last_subsyslog_resources[0].end = last_subsyslog_resources[0].start +
				    SUBSYS_ERR_LOG_SIZE - 1;

	last_subsyslog_resources[1].start = debug_mem_base + ADSP_LOGS_OFFSET;
	last_subsyslog_resources[1].end = last_subsyslog_resources[1].start +
					SUBSYS_ERR_LOG_SIZE - 1;

	last_subsyslog_device.num_resources =
			ARRAY_SIZE(last_subsyslog_resources);
	last_subsyslog_device.resource = last_subsyslog_resources;
	last_subsyslog_device.dev.platform_data = &ramdump_mode;
	platform_device_register(&last_subsyslog_device);
#endif
}

static int __init board_ramdump_init(void)
{
	if (alloc_debug_memory()) {
		pr_err("board_ramdump: failed to allocate memory\n");
		return -EINVAL;
	}

	ramdump_debug_memory_init();
	ramdump_add_devices();
	return 0;
}
core_initcall(board_ramdump_init);
