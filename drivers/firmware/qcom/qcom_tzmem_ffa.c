/*
 * NOTE: This file has been modified by Sony Corporation.
 * Modifications are Copyright 2025 Sony Corporation,
 * and licensed under the license of the file.
 */
// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "qcom_tzmem_ffa: [%s]: " fmt, __func__

#include <linux/arm_ffa.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include "qcom_tzmem.h"

/* We can have only one QTEE Secure Partition with the given UUID */
static struct ffa_device *qcom_tzmem_ffa_dev;
static struct device *qcom_tzmem_dev;

/* We can have only one QTEE Secure Partition with the given UUID */
#define QCOM_TZMEM_FFA_UUID						    \
	UUID_INIT(0x571217bb, 0x16d2, 0x543f, 0x91, 0x7e, 0xc4, 0xf0, 0x42, \
		  0x37, 0xa7, 0x74)

static int qcom_tzmem_ffa_mem_share_internal(struct sg_table *sgt, uint64_t *ffa_handle)
{
	int rc = 0;

	struct ffa_mem_region_attributes mem_attr = {
		.receiver = qcom_tzmem_ffa_dev->vm_id,
		.attrs = FFA_MEM_RW,
		.flag = 0,
	};

	struct ffa_mem_ops_args mem_args = {
		.attrs = &mem_attr,
		.use_txbuf = true,
		.nattrs = 1,
		.flags = 0,
		.sg = sgt->sgl,
	};

	rc = qcom_tzmem_ffa_dev->ops->mem_ops->memory_share(&mem_args);
	if (rc) {
		pr_err("Failed to share memory, ret: %d\n", rc);
		return rc;
	}

	pr_debug("Memory shared successfully, ffa_handle: 0x%llx\n", mem_args.g_handle);
	*ffa_handle = mem_args.g_handle;

	return 0;
}

int qcom_tzmem_ffa_mem_share(const void *vaddr, const dma_addr_t dma_addr,
			     size_t size, bool is_cached, uint64_t *ffa_handle)
{
	struct sg_table sgt;
	int rc;

	if (!qcom_tzmem_ffa_dev || !qcom_tzmem_dev)
		return -ENODEV;

	pr_debug("vaddr: 0x%p, , paddr: 0x%llx, size: 0x%zx\n", vaddr, dma_addr, size);

	if (!is_cached) {
		/* Try dma_get_sgtable() if dma_addr is valid */
		rc = dma_get_sgtable(qcom_tzmem_dev, &sgt, (void *)vaddr, dma_addr, size);
		if (rc) {
			pr_err("dma_get_sgtable failed, rc: %d\n", rc);
			return rc;
		}
	} else {
		size_t nr_pages = PAGE_ALIGN(size) / PAGE_SIZE;
		struct page **pages __free(kfree) = kcalloc(nr_pages, sizeof(*pages), GFP_KERNEL);
		if (!pages)
			return -ENOMEM;

		for (int i = 0; i < nr_pages; i++)
			pages[i] = virt_to_page((uint8_t *)vaddr + i * PAGE_SIZE);

		rc = sg_alloc_table_from_pages(&sgt, pages, nr_pages, 0,
						nr_pages * PAGE_SIZE, GFP_KERNEL);
		if (rc) {
			pr_err("Failed to allocate sg list, rc: %d\n", rc);
			return rc;
		}
	}

	/* Share memory via FFA */
	rc = qcom_tzmem_ffa_mem_share_internal(&sgt, ffa_handle);
	if (rc)
		pr_err("Failed to share memory, rc: %d\n", rc);

	sg_free_table(&sgt);
	return rc;
}

int qcom_tzmem_ffa_mem_reclaim(uint64_t ffa_handle)
{
	int rc = 0;

	if (!qcom_tzmem_ffa_dev)
		return -ENODEV;

	pr_debug("reclaim enter: ffa_handle: 0x%llx\n", ffa_handle);

	rc = qcom_tzmem_ffa_dev->ops->mem_ops->memory_reclaim(ffa_handle, 0);
	if (rc)
		pr_err("mem_reclaim failed: handle: 0x%llx, rc: %d\n", ffa_handle, rc);

	return rc;
}

static int qcom_tzmem_ffa_probe(struct ffa_device *ffa_dev)
{
	qcom_tzmem_ffa_dev = ffa_dev;
	return 0;
}

static void qcom_tzmem_ffa_remove(struct ffa_device *ffa_dev)
{
	qcom_tzmem_dev = NULL;
	qcom_tzmem_ffa_dev = NULL;
}

static const struct ffa_device_id qcom_tzmem_ffa_device_id[] = {
	{ QCOM_TZMEM_FFA_UUID },
	{}
};

static struct ffa_driver qcom_tzmem_ffa_driver = {
	.name = "qcom_tzmem_ffa",
	.probe = qcom_tzmem_ffa_probe,
	.remove = qcom_tzmem_ffa_remove,
	.id_table = qcom_tzmem_ffa_device_id,
};

int qcom_tzmem_ffa_register(struct device *dev)
{
	if (qcom_tzmem_dev)
		return -EBUSY;

	qcom_tzmem_dev = dev;

	return ffa_register(&qcom_tzmem_ffa_driver);
}
