/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_COMPAT_H_
#define _CAM_COMPAT_H_

#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/iommu.h>
#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
#include <linux/firmware/qcom/qcom_scm.h>
#else
#include <linux/qcom_scm.h>
#endif
#include <linux/list_sort.h>
#if KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE
#include <linux/dma-iommu.h>
#endif
#include <soc/qcom/of_common.h>
#include <linux/spi/spi.h>

#include "cam_csiphy_dev.h"
#include "cam_cpastop_hw.h"
#include "cam_smmu_api.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
#include <linux/ion.h>
#include <linux/msm_ion.h>
#define VFL_TYPE_VIDEO VFL_TYPE_GRABBER
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && \
	LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0)
#include <soc/qcom/of_common.h>
#include <linux/qcom-dma-mapping.h>
#endif

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
MODULE_IMPORT_NS(DMA_BUF);
#endif


#if IS_REACHABLE(CONFIG_INTERCONNECT_QCOM)
#include <linux/interconnect.h>
#endif

struct cam_fw_alloc_info {
	struct device *fw_dev;
	void          *fw_kva;
	uint64_t       fw_hdl;
};

int cam_reserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length);
void cam_unreserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length);
void cam_cpastop_scm_write(struct cam_cpas_hw_errata_wa *errata_wa);
int cam_ife_notify_safe_lut_scm(bool safe_trigger);
int camera_component_match_add_drivers(struct device *master_dev,
	struct component_match **match_list);
int cam_csiphy_notify_secure_mode(struct csiphy_device *csiphy_dev,
	bool protect, int32_t offset);
void cam_free_clear(const void *);
void cam_check_iommu_faults(struct iommu_domain *domain,
	struct cam_smmu_pf_info *pf_info);
static inline int cam_get_ddr_type(void) { return of_fdt_get_ddrtype(); }
int cam_compat_util_get_dmabuf_va(struct dma_buf *dmabuf, uintptr_t *vaddr);
void cam_compat_util_put_dmabuf_va(struct dma_buf *dmabuf, void *vaddr);
struct sg_table *cam_compat_dmabuf_map_attach(
	struct dma_buf_attachment *attach, enum dma_data_direction dma_dir);
void cam_compat_dmabuf_unmap_attach(struct dma_buf_attachment *attach,
	struct sg_table *table, enum dma_data_direction dma_dir);
void cam_smmu_util_iommu_custom(struct device *dev,
	dma_addr_t discard_start, size_t discard_length);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
int cam_req_mgr_ordered_list_cmp(void *priv,
	const struct list_head *head_1, const struct list_head *head_2);
#else
int cam_req_mgr_ordered_list_cmp(void *priv,
	struct list_head *head_1, struct list_head *head_2);
#endif

int cam_get_subpart_info(uint32_t *part_info, uint32_t max_num_cam);

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
void cam_eeprom_spi_driver_remove(struct spi_device *sdev);
#else
int cam_eeprom_spi_driver_remove(struct spi_device *sdev);
#endif

int cam_compat_util_get_irq(struct cam_hw_soc_info *soc_info);

int cam_iommu_map(struct iommu_domain *domain,
	size_t firmware_start, phys_addr_t fw_hdl, size_t firmware_len,
	int prot);

size_t cam_iommu_map_sg(struct iommu_domain *domain,
	dma_addr_t iova_start, struct scatterlist *sgl, uint64_t orig_nents,
	int prot);

int16_t cam_get_gpio_counts(struct cam_hw_soc_info *soc_info);

uint16_t cam_get_named_gpio(struct cam_hw_soc_info *soc_info,
	int index);

#if IS_REACHABLE(CONFIG_INTERCONNECT_QCOM)
inline struct icc_path *cam_icc_get_path(struct device *dev,
	const int src_id, const int dst_id, const char *path_name, bool use_path_name);
#endif

#endif /* _CAM_COMPAT_H_ */
