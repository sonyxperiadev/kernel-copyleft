/*
 * Author: Nandhakumar Rangasamy<nandhakumar.x.rangasamy@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
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
#include <linux/sort.h>
#include <linux/mount.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/blk_types.h>
#include <linux/blkdev.h>

#define MEM_DESC_NAME_SIZE 32

enum {
	MEM_DESC_PLATFORM = 0,
	MEM_DESC_CORE
};

struct mem_desc {
	u64 phys_addr;
	u64 size;
	u8  name[MEM_DESC_NAME_SIZE];
	u32 flags;
	u32 reserved;
} __attribute__ ((__packed__));

static struct device *dev;
static void *mem_desc_base;
static size_t mem_desc_size;
static char *mem_desc_buf;
static unsigned int mem_desc_data_size;
static char *mem_desc_work_buf;
static unsigned int mem_desc_work_size;
static int mem_desc_count;

static DEFINE_MUTEX(mem_desc_lock);

/*Random number*/
#define RDTAGS_MEM_DESC_SIG 0x42972468
#define DUMP_TABLE_OFFSET 0x5014
#define MEM_DESC_MAX 128
#define MEM_DESC_MASK 0xf

struct mem_desc_hdr {
	u32 sig;
	u32 version;
	u32 num_desc;
	u32 reserved;
};

struct ramdump_mem_desc {
	struct mem_desc_hdr hdr;
	struct mem_desc desc[MEM_DESC_MAX];
};

#define MEM_DESC_FORMAT_SIZE 114
#define MEM_DESC_FORMAT "0x%016llx:0x%016llx:%s:%s:0x%08x\n"

struct desc_buf {
	u64 addr;
	u64 size;
	u8 p_name[32];
	u8 c_name[32];
	u32 flags;
};

#define RAMDUMP_MEMDESC_SIZE (256 * SZ_1K)
static unsigned long debug_mem_base;
static unsigned long debug_mem_size;

static int get_ramdump_partition_index(void)
{
	int ret = -1;
	dev_t rdev;
	struct block_device *rd_device;
	char *ramdump_partition = "PARTLABEL=rddata";

	rdev = name_to_dev_t(ramdump_partition);
	if (!rdev) {
		dev_info(dev, "No matching partition device found\n");
		return ret;
	}

	rd_device = blkdev_get_by_dev(rdev, FMODE_READ, NULL);
	if (IS_ERR(rd_device)) {
		dev_info(dev, "Failed to get ramdump device\n");
		ret = PTR_ERR(rd_device);
		return ret;
	}

	return rd_device->bd_partno;

}

static u32 mem_desc_update_flags(u64 start, u64 end,
			struct ramdump_mem_desc *m_desc)
{
	int i, p_idx;

	struct desc_flags {
		u32 dump_type:4;
		u32 scn_index:6;
		u32 partition_index:8;
		u32 reserved:12;
		u32 nv:2;
	} __attribute__ ((__packed__));

	union u_desc_flags {
		struct desc_flags desc;
		u32 data;
	} flags = { {0} };

	int scn_match_found = 0;

	p_idx = get_ramdump_partition_index();
	if (p_idx < 0)
		return 0;

	for (i = 0; i < m_desc->hdr.num_desc; i++) {
		struct desc_flags desc_flags;

		memcpy(&desc_flags, &m_desc->desc[i].flags, sizeof(u32));
		/* Check if address in desc range */
		if (start >= m_desc->desc[i].phys_addr &&
		    end <= (m_desc->desc[i].phys_addr + m_desc->desc[i].size)) {

			if (desc_flags.dump_type & MEM_DESC_CORE)
				flags.desc.dump_type = desc_flags.dump_type;

			/* Populate section index, partition index and NV bits.
			 * We skip the condition if we already found a
			 * matching section in NV partition so that
			 * we do not overwrite the matching section index.
			 */
			if (desc_flags.nv && !scn_match_found) {
				flags.desc.nv = desc_flags.nv;
				flags.desc.scn_index = desc_flags.scn_index;
				flags.desc.partition_index = p_idx;

				if (start == m_desc->desc[i].phys_addr &&
				     end == (m_desc->desc[i].phys_addr +
					      m_desc->desc[i].size))
					scn_match_found = 1;
			}
		}
	}

	return flags.data;
}

static void mem_desc_populate_work_buf(u64 addr, u64 size, char *p_name,
					char *c_name, u32 flags)
{
	int i;
	struct desc_buf m_desc;
	struct desc_buf *buf;
	int desc_size = 0;

	if (!p_name)
		return;
	/* No mem_desc with same name & same addr */
	for (i = 0; i < mem_desc_count; i++) {
		buf = (struct desc_buf *)
			(((unsigned long)mem_desc_work_buf) + desc_size);
		if (!strncmp(p_name, buf->p_name, sizeof(m_desc.p_name))
				&& addr == buf->addr) {
			return;
		}

		desc_size += sizeof(m_desc);
	}

	memset(&m_desc, 0, sizeof(m_desc));
	m_desc.addr = addr;
	m_desc.size = size;
	m_desc.flags = flags;
	strscpy(m_desc.p_name, p_name, sizeof(m_desc.p_name));
	if (c_name)
		strscpy(m_desc.c_name, c_name, sizeof(m_desc.c_name));
	else
		strscpy(m_desc.c_name, "NULL", sizeof(m_desc.c_name));

	memcpy((void *)(((unsigned long)mem_desc_work_buf) +
			mem_desc_work_size), &m_desc, sizeof(m_desc));
	mem_desc_work_size += sizeof(m_desc);
	mem_desc_count++;
}

static void mem_desc_fill_format(void)
{
	int i, desc_size = 0;
	struct desc_buf *m_desc;

	for (i = 0; i < mem_desc_count; i++) {
		m_desc = (struct desc_buf *)(((unsigned long)mem_desc_work_buf)
				+ desc_size);
		mem_desc_data_size += snprintf(
				(mem_desc_buf + mem_desc_data_size),
				MEM_DESC_FORMAT_SIZE,
				MEM_DESC_FORMAT,
				m_desc->addr,
				m_desc->size,
				m_desc->p_name,
				m_desc->c_name,
				m_desc->flags);
		desc_size += sizeof(struct desc_buf);
	}
}

static void mem_desc_split(u64 *sort_buffer,
			unsigned int sort_count,
			struct ramdump_mem_desc *m_desc)
{
	unsigned int i, j;

	for (j = 0; j < sort_count; j++) {
		u32 flags = 0; u64 size = 0;
		char *p_name = NULL, *c_name = NULL;

		if ((j + 1) < sort_count) {
			if (sort_buffer[j] == sort_buffer[j + 1])
				continue;
		} else {
			break;
		}

		size = sort_buffer[j + 1] - sort_buffer[j];
		flags = mem_desc_update_flags(sort_buffer[j],
			  sort_buffer[j + 1], m_desc);

		for (i = 0; i < m_desc->hdr.num_desc; i++) {
			if (sort_buffer[j] >= m_desc->desc[i].phys_addr &&
				(sort_buffer[j] < (m_desc->desc[i].phys_addr
					+ m_desc->desc[i].size))) {
				switch (m_desc->desc[i].flags & MEM_DESC_MASK) {
				case MEM_DESC_PLATFORM:
					if (p_name == NULL)
						p_name = m_desc->desc[i].name;
					else if (!strncmp(p_name,
						m_desc->desc[i].name,
						sizeof(m_desc->desc[i].name)))
						continue;
					else
						mem_desc_populate_work_buf(sort_buffer[j],
							size,
							m_desc->desc[i].name,
							NULL,
							flags);
					break;
				case MEM_DESC_CORE:
					c_name = m_desc->desc[i].name;
					break;
				}
			}
		}

		if (p_name != NULL)
			mem_desc_populate_work_buf(sort_buffer[j],
					size, p_name, c_name, flags);
	}
}

void ramdump_add_mem_desc(struct mem_desc *desc)
{
	struct ramdump_mem_desc *m_desc = NULL;
	unsigned int offset;

	if (!mem_desc_base) {
		dev_info(dev, "Adding mem_desc failed\n");
		return;
	}

	mutex_lock(&mem_desc_lock);
	m_desc = (struct ramdump_mem_desc *)mem_desc_base;
	offset = sizeof(struct mem_desc_hdr) +
		m_desc->hdr.num_desc * sizeof(struct mem_desc);

	if (m_desc->hdr.num_desc >= MEM_DESC_MAX)
		goto exit;

	if ((offset + sizeof(struct mem_desc)) > mem_desc_size)
		goto exit;

	memcpy_toio(
		((struct ramdump_mem_desc *)(((unsigned long)m_desc) + offset)),
		desc, sizeof(struct mem_desc));

	m_desc->hdr.num_desc++;
exit:
	mutex_unlock(&mem_desc_lock);
}
EXPORT_SYMBOL(ramdump_add_mem_desc);

void ramdump_remove_mem_desc(struct mem_desc *desc)
{
	struct ramdump_mem_desc *m_desc = NULL;
	struct ramdump_mem_desc *m_desc_buf = NULL;
	unsigned int i, offset, m_desc_size;

	if (!mem_desc_base) {
		dev_info(dev, "Removing mem_desc failed\n");
		return;
	}

	mutex_lock(&mem_desc_lock);
	m_desc = (struct ramdump_mem_desc *)mem_desc_base;
	m_desc_size = sizeof(struct mem_desc_hdr) +
	m_desc->hdr.num_desc * sizeof(struct mem_desc);

	m_desc_buf = vmalloc(m_desc_size);
	if (!m_desc_buf) {
		dev_err(dev, "Failed to allocate mem_desc buffer\n");
		goto exit;
	}

	memcpy_fromio(m_desc_buf, mem_desc_base, m_desc_size);
	offset = sizeof(struct mem_desc_hdr) +
	m_desc_buf->hdr.num_desc * sizeof(struct mem_desc);

	for (i = 0; i < m_desc_buf->hdr.num_desc; i++) {
		if ((desc->phys_addr == m_desc_buf->desc[i].phys_addr) &&
				!strncmp(desc->name, m_desc_buf->desc[i].name,
				sizeof(m_desc_buf->desc[i].name))) {
			memset(&m_desc_buf->desc[i], 0,
					sizeof(struct mem_desc));
			memmove(&m_desc_buf->desc[i],
				&m_desc_buf->desc[i + 1],
				(m_desc_buf->hdr.num_desc - i - 1)
				* sizeof(struct mem_desc));
			m_desc_buf->hdr.num_desc--;
		}
	}

	memset_io(m_desc, 0, m_desc_size);
	memcpy_toio(m_desc, m_desc_buf, m_desc_size);
	vfree(m_desc_buf);
exit:
	mutex_unlock(&mem_desc_lock);
}
EXPORT_SYMBOL(ramdump_remove_mem_desc);

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
	unsigned int i, num_desc;

	num_desc = (m_desc->hdr.num_desc < MEM_DESC_MAX) ?
			m_desc->hdr.num_desc : MEM_DESC_MAX;

	if (m_desc->hdr.sig == RDTAGS_MEM_DESC_SIG) {
		u64 *sort_buffer;
		sort_buffer = kmalloc(sizeof(*sort_buffer)*num_desc*2,
					GFP_KERNEL);
		if (!sort_buffer) {
			pr_err("ramdumper: %s: failed to allocate memory\n",
				__FUNCTION__);
			return;
		}
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
		mem_desc_split(sort_buffer, count, m_desc);
		mem_desc_fill_format();
		kfree(sort_buffer);
	}
}

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
				struct mem_desc desc;

				desc.phys_addr = (u64)res->start;
				desc.size =
					((u64)res->end - (u64)res->start) + 1;
				desc.flags = MEM_DESC_CORE;
				strscpy(desc.name, core_name,
						sizeof(desc.name));
				ramdump_add_mem_desc(&desc);
			}
		}
		res = res->sibling;
	}
}

static void mem_desc_add_linux_meminfo(void)
{
	add_ioresource(&iomem_resource, "System RAM", "vmcore");
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

static const struct proc_ops mem_desc_proc_ops = {
	.proc_read = mem_desc_read,
};

#define MAX_DESC_PER_TYPE 32
static int mem_desc_init(void)
{
	int ret = 0;
	size_t alloc_size;
	struct proc_dir_entry *subentry_mem_desc;

	alloc_size = MEM_DESC_FORMAT_SIZE * 3 * MAX_DESC_PER_TYPE;
	mem_desc_buf = kmalloc(alloc_size, GFP_KERNEL);
	if (mem_desc_buf == NULL) {
		dev_err(dev, "Failed to allocate memory regs_buf\n");
		ret = -ENOMEM;
		goto exit;
	}

	mem_desc_work_buf = kmalloc(alloc_size, GFP_KERNEL);
	if (mem_desc_work_buf == NULL) {
		dev_err(dev, "Failed to allocate memory regs_buf\n");
		ret = -ENOMEM;
		goto exit;
	}

	subentry_mem_desc = proc_create_data("mem_desc",
				S_IFREG | S_IRUGO, NULL,
				&mem_desc_proc_ops, NULL);
	if (!subentry_mem_desc) {
		dev_err(dev, "Failed to create proc subentry mem_desc\n");
		ret = -1;
		goto exit1;
	}

	return ret;

exit1:
	kfree(mem_desc_buf);
	kfree(mem_desc_work_buf);
	mem_desc_buf = NULL;
exit:
	return ret;
}

static int mem_desc_driver_probe(struct platform_device *pdev)
{
	struct resource *res_mem_desc;
	struct ramdump_mem_desc *m_desc;
	void *mem_desc_end;
	int ret = 0;

	dev = &pdev->dev;

	res_mem_desc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
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

	if (m_desc->hdr.sig == RDTAGS_MEM_DESC_SIG) {
		dev_info(dev, "Found memory descriptors\n");
		mem_desc_add_linux_meminfo();
		if (mem_desc_init() != 0)
			goto exit;
	} else {
		dev_info(dev, "NO valid memory descriptors found!!\n");
		ret = -EINVAL;
		goto exit;
	}

	return 0;
exit:
	if (mem_desc_base) {
		iounmap(mem_desc_base);
		mem_desc_base = NULL;
	}

	return ret;
}


static const struct of_device_id ramdump_dt[] = {
		{ .compatible = "qcom,debug_memory" },
		{}
};

static struct resource ramdump_memdesc_resources;

static struct platform_driver mem_desc_driver = {
	.probe = mem_desc_driver_probe,
	.driver	= {
		.name = "ramdump_memdesc",
	},
};

static struct platform_device ramdump_memdesc_device = {
	.name           = "ramdump_memdesc",
	.id             = -1,
	.num_resources	= 1,
	.resource	= &ramdump_memdesc_resources,
};

static int __init mem_desc_core_init(void)
{
	struct device_node *node;
	uint32_t *regs = NULL;
	size_t cells;
	int ret;

	node = of_find_matching_node(NULL, ramdump_dt);
	if (!node) {
		pr_err("debug region node not found\n");
		return -EINVAL;
	}

	cells = of_n_addr_cells(node) + of_n_size_cells(node);
	regs = kcalloc(cells, sizeof(uint32_t), GFP_KERNEL);
	if (!regs) {
		pr_err("Failed to allocate memory for cells\n");
		of_node_put(node);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(node, "reg", regs, cells)) {
		pr_err("unable to find base address of node in dtb\n");
		goto failed;
	}

	if (cells == 4) {
		debug_mem_base = (unsigned long)regs[0] << 32 | regs[1];
		debug_mem_size = (unsigned long)regs[2] << 32 | regs[3];
	} else if (cells == 2) {
		debug_mem_base = regs[0];
		debug_mem_size = regs[1];
	} else {
		pr_err("bad number of cells in the regs property\n");
		goto failed;
	}

	pr_info("board-ramdump: Initialized debug memory at %lx-%lx\n",
		debug_mem_base, debug_mem_base + debug_mem_size - 1);
	of_node_put(node);
	kfree(regs);

	if (!debug_mem_base)
		return -EINVAL;

	ramdump_memdesc_resources.start = debug_mem_base;
	ramdump_memdesc_resources.end = ramdump_memdesc_resources.start +
					RAMDUMP_MEMDESC_SIZE - 1;
	ramdump_memdesc_resources.flags  = IORESOURCE_MEM;

	ret = platform_device_register(&ramdump_memdesc_device);
	if (ret) {
		pr_err("memdesc: platform device registration failed is %d\n", ret);
		return ret;
	}

	return platform_driver_register(&mem_desc_driver);

failed:
	of_node_put(node);
	kfree(regs);
	return -EINVAL;
}

static void __exit mem_desc_module_exit(void)
{
	if (mem_desc_base) {
		iounmap(mem_desc_base);
		mem_desc_base = NULL;
	}

	platform_driver_unregister(&mem_desc_driver);
}

module_init(mem_desc_core_init);
module_exit(mem_desc_module_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("lmk_ramdump_mem_desc driver");
