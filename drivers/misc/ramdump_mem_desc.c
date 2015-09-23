/*
 * Author: Nandhakumar Rangasamy<nandhakumar.x.rangasamy@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/of_fdt.h>
#include <linux/string.h>
#include <asm/setup.h>
#include <linux/delay.h>
#include <linux/ramdump_mem_desc.h>
#include <linux/sort.h>
#include <linux/genhd.h>
#include <linux/device.h>

#ifdef CONFIG_HAVE_MEMBLOCK
#include <linux/memblock.h>
#endif

static struct device *dev;
static void *mem_desc_base;
static size_t mem_desc_size;
static char *mem_desc_buf;
static unsigned int mem_desc_data_size;

/*Random number*/
#define RDTAGS_MEM_DESC_SIG 0x42972468
#define DUMP_TABLE_OFFSET 0x5014
#define MEM_DESC_PLATFORM 0
#define MEM_DESC_CORE 1
#define MEM_DESC_MAX 64
#define MEM_DESC_MASK 0xf

struct mem_desc_hdr {
	u32 sig;
	u32 version;
	u32 num_desc;
	u32 reserved;
};

struct mem_desc {
	u64 phys_addr;
	u64 size;
	u8  name[MEM_DESC_NAME_SIZE];
	u32 flags;
	u32 reserved;
} __attribute__ ((__packed__));

struct ramdump_mem_desc {
	struct mem_desc_hdr hdr;
	struct mem_desc desc[MEM_DESC_MAX];
};

#define MEM_DESC_FORMAT_SIZE 114
#define MEM_DESC_FORMAT "0x%016llx:0x%016llx:%s:%s:0x%08x\n"

static int match_dev_by_volname(struct device *pdev, const void *data)
{
	char *volname = "rddata";
	struct hd_struct *part = dev_to_part(pdev);

	if (!part->info)
		goto no_match;

	if (strncmp(volname, part->info->volname,
			sizeof(part->info->volname)))
		goto no_match;

	return 1;
no_match:
	return 0;
}

static int get_ramdump_partition_index(void)
{
	struct device *pdev = NULL;
	u8 uuid[16] = {0};
	int ret = -1;

	pdev = class_find_device(&block_class, NULL, &uuid,
					match_dev_by_volname);
	if (!pdev) {
		dev_info(dev, "Partition device not found");
		return ret;
	}

	return dev_to_part(pdev)->partno;
}

static u32 mem_desc_update_flags(u64 start, u64 end,
			struct ramdump_mem_desc *m_desc)
{
	int i, ret;
	u32 flags;

	ret = get_ramdump_partition_index();
	if (ret < 0)
		return 0;

	flags = ret << 9;
	for (i = 0; i < m_desc->hdr.num_desc; i++) {
		if (start >= m_desc->desc[i].phys_addr &&
			(end <= (m_desc->desc[i].phys_addr
				+ m_desc->desc[i].size)))
				flags |= m_desc->desc[i].flags;
	}

	return flags;
}

static void mem_desc_split_sect_format(
				u64 sort_buffer[],
				unsigned int sort_count,
				struct ramdump_mem_desc *m_desc)
{
	unsigned int i, j;

	for (j = 0; j < sort_count; j++) {
		u32 flags = 0; u64 size = 0;
		char *p_name = NULL, *c_name = NULL;

		if ((j+1) < sort_count) {
			if (sort_buffer[j] == sort_buffer[j+1])
				continue;
		}

		for (i = 0; i < m_desc->hdr.num_desc; i++) {
			if (sort_buffer[j] >= m_desc->desc[i].phys_addr &&
				(sort_buffer[j] < (m_desc->desc[i].phys_addr
					+ m_desc->desc[i].size))) {

				size = sort_buffer[j + 1] - sort_buffer[j];
				flags = mem_desc_update_flags(sort_buffer[j],
						sort_buffer[j+1],
						m_desc);

				switch (m_desc->desc[i].flags & MEM_DESC_MASK) {
				case MEM_DESC_PLATFORM:
					if (p_name == NULL)
						p_name = m_desc->desc[i].name;
					else if ((!strncmp(p_name,
						m_desc->desc[i].name,
						sizeof(m_desc->desc[i].name)))
						&&
						(m_desc->desc[i].flags >> 31))
						continue;
					else {
						mem_desc_data_size += snprintf(
							(mem_desc_buf +
							mem_desc_data_size),
							MEM_DESC_FORMAT_SIZE,
							MEM_DESC_FORMAT,
							sort_buffer[j],
							size,
							m_desc->desc[i].name,
							"NULL",
							flags);
					}
				break;
				case MEM_DESC_CORE:
					c_name = m_desc->desc[i].name;
				break;
				}
			}
		}
		if (p_name != NULL) {
			mem_desc_data_size += snprintf(
					(mem_desc_buf +
					mem_desc_data_size),
					MEM_DESC_FORMAT_SIZE,
					MEM_DESC_FORMAT,
					sort_buffer[j],
					size,
					p_name,
					c_name ? c_name : "NULL",
					flags);
			}
	}
}

static int mem_desc_add_flag_by_name(char *name)
{
	int i, ret = 0;
	char *core_name[] = {
			"CORE",
			"core"
		};
	int c_count = sizeof(core_name)/sizeof(core_name[0]);

	for (i = 0; i < c_count; i++) {
		if (strrchr(name, *core_name[i]) == NULL)
			continue;
		if (!strncmp(strrchr(name, *core_name[i]), core_name[i], 4)) {
			ret = 1;
			break;
		}
	}

	return ret;
}

void ramdump_add_mem_desc(u64 addr, u64 size, char *name)
{
	struct ramdump_mem_desc *m_desc = NULL;
	struct mem_desc mem_desc_tmp;
	unsigned int i, offset;

	if (!mem_desc_base) {
		dev_info(dev, "Adding mem_desc failed\n");
		return;
	}

	m_desc = (struct ramdump_mem_desc *)mem_desc_base;
	offset = sizeof(struct mem_desc_hdr) +
		m_desc->hdr.num_desc * sizeof(struct mem_desc);

	if ((offset + sizeof(struct mem_desc)) > mem_desc_size)
		return;

	memset(&mem_desc_tmp, 0, sizeof(struct mem_desc));
	mem_desc_tmp.phys_addr = addr;
	mem_desc_tmp.size = size;
	strlcpy(mem_desc_tmp.name, name, sizeof(mem_desc_tmp.name));
	mem_desc_tmp.flags = mem_desc_add_flag_by_name(name);

	if (!(mem_desc_tmp.flags & MEM_DESC_CORE))
		goto add_mem_desc;

	for (i = 0; i < m_desc->hdr.num_desc; i++) {
		u64 msize = m_desc->desc[i].phys_addr + m_desc->desc[i].size;
		if ((addr >= m_desc->desc[i].phys_addr && addr < msize) ||
			(addr < m_desc->desc[i].phys_addr &&
			(addr + size) > m_desc->desc[i].phys_addr)) {
			dev_err(dev, "overlaped 0x%016llx,0x%016llx,%s\n",
				addr, size, name);
			return;
		}
	}

add_mem_desc:
	memcpy_toio(((struct ramdump_mem_desc *)(((unsigned long)m_desc) + offset)),
					&mem_desc_tmp, sizeof(struct mem_desc));
	m_desc->hdr.num_desc++;
}
EXPORT_SYMBOL(ramdump_add_mem_desc);

static int cmp_sort_addr(const void *a, const void *b)
{
	u64 x = *(u64 *)a;
	u64 y = *(u64 *)b;
	int ret = 0;

	if (x < y)
		ret = -1;
	if (x > y)
		ret = 1;

	return ret;
}

static void get_mem_desc(void)
{
	unsigned int count = 0;
	struct ramdump_mem_desc *m_desc =
		(struct ramdump_mem_desc *)mem_desc_base;
	unsigned int i, num_desc = m_desc->hdr.num_desc;

	if (m_desc->hdr.sig == RDTAGS_MEM_DESC_SIG) {
		u64 sort_buffer[MEM_DESC_MAX << 1];
		for (i = 0; i < num_desc; i++) {
			u64 start, end;
			start = m_desc->desc[i].phys_addr;
			end = m_desc->desc[i].phys_addr + m_desc->desc[i].size;

			memcpy(&sort_buffer[count++], &start,
					sizeof(u64));
			memcpy(&sort_buffer[count++], &end,
					sizeof(u64));
		}

		sort(sort_buffer, count, sizeof(u64),
						cmp_sort_addr, NULL);
		mem_desc_split_sect_format(sort_buffer, count, m_desc);
	}
}

#ifdef CONFIG_HAVE_MEMBLOCK
static void add_memblock_info(void)
{
	int i;
	struct memblock_type *block = &memblock.memory;;

	for (i = 0; i < block->cnt; i++)
		ramdump_add_mem_desc((u64)block->regions[i].base,
					(u64)block->regions[i].size,
					"vmcore");
}
#else
static void add_ioresource(struct resource *res,
				char *iores_name,
				char *core_name)
{
	while (res != NULL) {
		if (res->child != NULL)
			add_ioresource(res->child, iores_name, core_name);

		if ((res->flags & IORESOURCE_MEM) &&
			strcmp(res->name, iores_name) == 0) {
			if (core_name != NULL) {
				ramdump_add_mem_desc(
					(u64)res->start,
					((u64)res->end - (u64)res->start + 1),
					core_name
					);
			}
		}
		res = res->sibling;
	}
}
#endif

static void mem_desc_add_linux_meminfo(void)
{
#ifdef CONFIG_HAVE_MEMBLOCK
	add_memblock_info();
#else
	add_ioresource(&iomem_resource, "System RAM", "vmcore");
#endif
}

static ssize_t mem_desc_read(struct file *file, char __user *buf,
				size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos == 0)
		get_mem_desc();

	if (pos >= mem_desc_data_size) {
		memset_io(mem_desc_base, 0x0, mem_desc_data_size);
		return 0;
	}

	count = min(len, (size_t)(mem_desc_data_size - pos));
	if (copy_to_user(buf, mem_desc_buf + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations mem_desc_file_ops = {
	.owner = THIS_MODULE,
	.read = mem_desc_read,
};

#define MAX_DESC_PER_TYPE 32
static int mem_desc_init(void)
{
	int ret = 0;
	struct proc_dir_entry *subentry_mem_desc;

	mem_desc_buf = kmalloc((MEM_DESC_FORMAT_SIZE * 3 * MAX_DESC_PER_TYPE),
			GFP_KERNEL);
	if (mem_desc_buf == NULL) {
		dev_err(dev, "Failed to allocte memory regs_buf\n");
		ret = -ENOMEM;
		goto exit;
	}

	subentry_mem_desc = proc_create_data("mem_desc",
				S_IFREG | S_IRUGO, NULL,
				&mem_desc_file_ops, NULL);
	if (!subentry_mem_desc) {
		dev_err(dev, "Failed to create proc subentry mem_desc\n");
		ret = -1;
		goto exit1;
	}

	return ret;

exit1:
	kfree(mem_desc_buf);
	mem_desc_buf = NULL;
exit:
	return ret;
}

static int is_ramdump_mode(void)
{
	return *((int *)(dev->platform_data));
}

static int mem_desc_driver_probe(struct platform_device *pdev)
{
	struct resource *res_mem_desc;
	struct ramdump_mem_desc *m_desc;
	void *mem_desc_end;
	int ret = 0;

	dev = &pdev->dev;

	res_mem_desc = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "ramdump_memdesc");
	if (!res_mem_desc || !res_mem_desc->start) {
		dev_err(dev,
			"Ramdump tags driver mem desc resource"
				" invalid/absent\n");
		ret = -ENODEV;
		goto exit;
	}

	mem_desc_size = res_mem_desc->end
				- res_mem_desc->start + 1;
	mem_desc_base = (void *)ioremap(res_mem_desc->start,
						mem_desc_size);
	if (!mem_desc_base) {
		dev_err(dev, "Failed to mem desc map %ld bytes at 0x%08llx\n",
				mem_desc_size, res_mem_desc->start);
		ret = -EINVAL;
		goto exit;
	}

	mem_desc_end = mem_desc_base + mem_desc_size;
	dev_info(dev, "res_mem_desc_base = 0x%08llx mem_desc size = %ld\n",
			res_mem_desc->start, mem_desc_size);
	m_desc = (struct ramdump_mem_desc *)mem_desc_base;

	dev_info(dev, "ramdump_mode = %s\n",
		is_ramdump_mode() ? "ramdump" : "normal");

	if (!is_ramdump_mode()) {
		memset_io(mem_desc_base, 0x0, mem_desc_size);
		m_desc->hdr.sig = RDTAGS_MEM_DESC_SIG;
		mem_desc_add_linux_meminfo();
	} else {
		if (m_desc->hdr.sig == RDTAGS_MEM_DESC_SIG) {
			dev_info(dev, "Found memory descriptors\n");
			if (mem_desc_init() != 0)
				goto exit;
		} else {
			dev_info(dev, "NO valid memory descriptors found!!\n");
			ret = -EINVAL;
			goto exit;
		}
	}

	return 0;
exit:
	if (mem_desc_base) {
		iounmap(mem_desc_base);
		mem_desc_base = NULL;
	}

	return ret;
}

static struct platform_driver mem_desc_driver = {
	.probe = mem_desc_driver_probe,
	.driver	= {
		.name = "ramdump_memdesc",
	},
};

static int __init mem_desc_core_init(void)
{
	return platform_driver_register(&mem_desc_driver);
}

static void __exit mem_desc_module_exit(void)
{
	if (mem_desc_base) {
		iounmap(mem_desc_base);
		mem_desc_base = NULL;
	}

	platform_driver_unregister(&mem_desc_driver);
}

core_initcall(mem_desc_core_init);
module_exit(mem_desc_module_exit);
