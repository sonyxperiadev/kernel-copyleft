// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "cam_compat.h"
#include "cam_debug_util.h"
#include "cam_cpas_api.h"
#include "camera_main.h"
#include "cam_eeprom_dev.h"
#include "cam_eeprom_core.h"

#if KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE
#include <soc/qcom/socinfo.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
int cam_reserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length)
{
	int rc = 0;
	struct device_node *of_node;
	struct device_node *mem_node;
	struct resource     res;

	of_node = (icp_fw->fw_dev)->of_node;
	mem_node = of_parse_phandle(of_node, "memory-region", 0);
	if (!mem_node) {
		rc = -ENOMEM;
		CAM_ERR(CAM_SMMU, "FW memory carveout not found");
		goto end;
	}
	rc = of_address_to_resource(mem_node, 0, &res);
	of_node_put(mem_node);
	if (rc < 0) {
		CAM_ERR(CAM_SMMU, "Unable to get start of FW mem carveout");
		goto end;
	}
	icp_fw->fw_hdl = res.start;
	icp_fw->fw_kva = ioremap_wc(icp_fw->fw_hdl, fw_length);
	if (!icp_fw->fw_kva) {
		CAM_ERR(CAM_SMMU, "Failed to map the FW.");
		rc = -ENOMEM;
		goto end;
	}
	memset_io(icp_fw->fw_kva, 0, fw_length);

end:
	return rc;
}

void cam_unreserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length)
{
	iounmap(icp_fw->fw_kva);
}

int cam_ife_notify_safe_lut_scm(bool safe_trigger)
{
	const uint32_t smmu_se_ife = 0;
	uint32_t camera_hw_version, rc = 0;

	rc = cam_cpas_get_cpas_hw_version(&camera_hw_version);
	if (!rc) {
		switch (camera_hw_version) {
		case CAM_CPAS_TITAN_170_V100:
		case CAM_CPAS_TITAN_170_V110:
		case CAM_CPAS_TITAN_175_V100:
			if (qcom_scm_smmu_notify_secure_lut(smmu_se_ife, safe_trigger)) {
				CAM_ERR(CAM_ISP, "scm call to enable safe failed");
				rc = -EINVAL;
			}
			break;
		default:
			break;
		}
	}

	return rc;
}

int cam_csiphy_notify_secure_mode(struct csiphy_device *csiphy_dev,
	bool protect, int32_t offset)
{
	int rc = 0;

	if (offset >= CSIPHY_MAX_INSTANCES_PER_PHY) {
		CAM_ERR(CAM_CSIPHY, "Invalid CSIPHY offset");
		rc = -EINVAL;
	} else if (qcom_scm_camera_protect_phy_lanes(protect,
			csiphy_dev->csiphy_info[offset]
				.csiphy_cpas_cp_reg_mask)) {
		CAM_ERR(CAM_CSIPHY, "SCM call to hypervisor failed");
		rc = -EINVAL;
	}

	return rc;
}

void cam_cpastop_scm_write(struct cam_cpas_hw_errata_wa *errata_wa)
{
	int reg_val;

	qcom_scm_io_readl(errata_wa->data.reg_info.offset, &reg_val);
	reg_val |= errata_wa->data.reg_info.value;
	qcom_scm_io_writel(errata_wa->data.reg_info.offset, reg_val);
}

static int camera_platform_compare_dev(struct device *dev, const void *data)
{
	return platform_bus_type.match(dev, (struct device_driver *) data);
}

static int camera_i2c_compare_dev(struct device *dev, const void *data)
{
	return i2c_bus_type.match(dev, (struct device_driver *) data);
}
#else
int cam_reserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length)
{
	int rc = 0;

	icp_fw->fw_kva = dma_alloc_coherent(icp_fw->fw_dev, fw_length,
		&icp_fw->fw_hdl, GFP_KERNEL);

	if (!icp_fw->fw_kva) {
		CAM_ERR(CAM_SMMU, "FW memory alloc failed");
		rc = -ENOMEM;
	}

	return rc;
}

void cam_unreserve_icp_fw(struct cam_fw_alloc_info *icp_fw, size_t fw_length)
{
	dma_free_coherent(icp_fw->fw_dev, fw_length, icp_fw->fw_kva,
		icp_fw->fw_hdl);
}

int cam_ife_notify_safe_lut_scm(bool safe_trigger)
{
	const uint32_t smmu_se_ife = 0;
	uint32_t camera_hw_version, rc = 0;
	struct scm_desc description = {
		.arginfo = SCM_ARGS(2, SCM_VAL, SCM_VAL),
		.args[0] = smmu_se_ife,
		.args[1] = safe_trigger,
	};

	rc = cam_cpas_get_cpas_hw_version(&camera_hw_version);
	if (!rc) {
		switch (camera_hw_version) {
		case CAM_CPAS_TITAN_170_V100:
		case CAM_CPAS_TITAN_170_V110:
		case CAM_CPAS_TITAN_175_V100:
			if (scm_call2(SCM_SIP_FNID(0x15, 0x3), &description)) {
				CAM_ERR(CAM_ISP, "scm call to enable safe failed");
				rc = -EINVAL;
			}
			break;
		default:
			break;
		}
	}

	return rc;
}

int cam_csiphy_notify_secure_mode(struct csiphy_device *csiphy_dev,
	bool protect, int32_t offset)
{
	int rc = 0;
	struct scm_desc description = {
		.arginfo = SCM_ARGS(2, SCM_VAL, SCM_VAL),
		.args[0] = protect,
		.args[1] = csiphy_dev->csiphy_info[offset]
			.csiphy_cpas_cp_reg_mask,
	};

	if (offset >= CSIPHY_MAX_INSTANCES_PER_PHY) {
		CAM_ERR(CAM_CSIPHY, "Invalid CSIPHY offset");
		rc = -EINVAL;
	} else if (scm_call2(SCM_SIP_FNID(0x18, 0x7), &description)) {
		CAM_ERR(CAM_CSIPHY, "SCM call to hypervisor failed");
		rc = -EINVAL;
	}

	return rc;
}

void cam_cpastop_scm_write(struct cam_cpas_hw_errata_wa *errata_wa)
{
	int reg_val;

	reg_val = scm_io_read(errata_wa->data.reg_info.offset);
	reg_val |= errata_wa->data.reg_info.value;
	scm_io_write(errata_wa->data.reg_info.offset, reg_val);
}

static int camera_platform_compare_dev(struct device *dev, void *data)
{
	return platform_bus_type.match(dev, (struct device_driver *) data);
}

static int camera_i2c_compare_dev(struct device *dev, void *data)
{
	return i2c_bus_type.match(dev, (struct device_driver *) data);
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
void cam_free_clear(const void * ptr)
{
	kfree_sensitive(ptr);
}
#else
void cam_free_clear(const void * ptr)
{
	kzfree(ptr);
}
#endif

/* Callback to compare device from match list before adding as component */
static inline int camera_component_compare_dev(struct device *dev, void *data)
{
	return dev == data;
}

/* Add component matches to list for master of aggregate driver */
int camera_component_match_add_drivers(struct device *master_dev,
	struct component_match **match_list)
{
	int i, rc = 0;
	struct platform_device *pdev = NULL;
	struct i2c_client *client = NULL;
	struct device *start_dev = NULL, *match_dev = NULL;

	if (!master_dev || !match_list) {
		CAM_ERR(CAM_UTIL, "Invalid parameters for component match add");
		rc = -EINVAL;
		goto end;
	}

	for (i = 0; i < ARRAY_SIZE(cam_component_platform_drivers); i++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		struct device_driver const *drv =
			&cam_component_platform_drivers[i]->driver;
		const void *drv_ptr = (const void *)drv;
#else
		struct device_driver *drv = &cam_component_platform_drivers[i]->driver;
		void *drv_ptr = (void *)drv;
#endif
		start_dev = NULL;
		while ((match_dev = bus_find_device(&platform_bus_type,
			start_dev, drv_ptr, &camera_platform_compare_dev))) {
			put_device(start_dev);
			pdev = to_platform_device(match_dev);
			CAM_DBG(CAM_UTIL, "Adding matched component:%s", pdev->name);
			component_match_add(master_dev, match_list,
				camera_component_compare_dev, match_dev);
			start_dev = match_dev;
		}
		put_device(start_dev);
	}

	for (i = 0; i < ARRAY_SIZE(cam_component_i2c_drivers); i++) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
		struct device_driver const *drv =
			&cam_component_i2c_drivers[i]->driver;
		const void *drv_ptr = (const void *)drv;
#else
		struct device_driver *drv = &cam_component_i2c_drivers[i]->driver;
		void *drv_ptr = (void *)drv;
#endif
		start_dev = NULL;
		while ((match_dev = bus_find_device(&i2c_bus_type,
			start_dev, drv_ptr, &camera_i2c_compare_dev))) {
			put_device(start_dev);
			client = to_i2c_client(match_dev);
			CAM_DBG(CAM_UTIL, "Adding matched component:%s", client->name);
			component_match_add(master_dev, match_list,
				camera_component_compare_dev, match_dev);
			start_dev = match_dev;
		}
		put_device(start_dev);
	}

end:
	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#include <linux/qcom-iommu-util.h>
void cam_check_iommu_faults(struct iommu_domain *domain,
	struct cam_smmu_pf_info *pf_info)
{
	struct qcom_iommu_fault_ids fault_ids = {0, 0, 0};

	if (qcom_iommu_get_fault_ids(domain, &fault_ids))
		CAM_ERR(CAM_SMMU, "Cannot get smmu fault ids");
	else
		CAM_ERR(CAM_SMMU, "smmu fault ids bid:%d pid:%d mid:%d",
			fault_ids.bid, fault_ids.pid, fault_ids.mid);

	pf_info->bid = fault_ids.bid;
	pf_info->pid = fault_ids.pid;
	pf_info->mid = fault_ids.mid;
}
#else
void cam_check_iommu_faults(struct iommu_domain *domain,
	struct cam_smmu_pf_info *pf_info)
{
	struct iommu_fault_ids fault_ids = {0, 0, 0};

	if (iommu_get_fault_ids(domain, &fault_ids))
		CAM_ERR(CAM_SMMU, "Error: Can not get smmu fault ids");

	CAM_ERR(CAM_SMMU, "smmu fault ids bid:%d pid:%d mid:%d",
		fault_ids.bid, fault_ids.pid, fault_ids.mid);

	pf_info->bid = fault_ids.bid;
	pf_info->pid = fault_ids.pid;
	pf_info->mid = fault_ids.mid;
}
#endif

static int inline cam_subdev_list_cmp(struct cam_subdev *entry_1, struct cam_subdev *entry_2)
{
	if (entry_1->close_seq_prior > entry_2->close_seq_prior)
		return 1;
	else if (entry_1->close_seq_prior < entry_2->close_seq_prior)
		return -1;
	else
		return 0;
}

#if (KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE)
int cam_compat_util_get_dmabuf_va(struct dma_buf *dmabuf, uintptr_t *vaddr)
{
	struct iosys_map mapping;
#if (KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE)
	int error_code = dma_buf_vmap_unlocked(dmabuf, &mapping);
#else
	int error_code = dma_buf_vmap(dmabuf, &mapping);
#endif
	if (error_code) {
		*vaddr = 0;
	} else {
		*vaddr = (mapping.is_iomem) ?
			(uintptr_t)mapping.vaddr_iomem : (uintptr_t)mapping.vaddr;
		CAM_DBG(CAM_MEM,
			"dmabuf=%p, *vaddr=%p, is_iomem=%d, vaddr_iomem=%p, vaddr=%p",
			dmabuf, *vaddr, mapping.is_iomem, mapping.vaddr_iomem, mapping.vaddr);
	}

	return error_code;
}

void cam_compat_util_put_dmabuf_va(struct dma_buf *dmabuf, void *vaddr)
{
	struct iosys_map mapping = IOSYS_MAP_INIT_VADDR(vaddr);
#if (KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE)
	dma_buf_vunmap_unlocked(dmabuf, &mapping);
#else
	dma_buf_vunmap(dmabuf, &mapping);
#endif
}

#elif (KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE)
int cam_compat_util_get_dmabuf_va(struct dma_buf *dmabuf, uintptr_t *vaddr)
{
	struct dma_buf_map mapping;
	int error_code = dma_buf_vmap(dmabuf, &mapping);

	if (error_code)
		*vaddr = 0;
	else
		*vaddr = (mapping.is_iomem) ?
			(uintptr_t)mapping.vaddr_iomem : (uintptr_t)mapping.vaddr;

	return error_code;
}

void cam_compat_util_put_dmabuf_va(struct dma_buf *dmabuf, void *vaddr)
{
	struct dma_buf_map mapping = DMA_BUF_MAP_INIT_VADDR(vaddr);

	dma_buf_vunmap(dmabuf, &mapping);
}

#else
int cam_compat_util_get_dmabuf_va(struct dma_buf *dmabuf, uintptr_t *vaddr)
{
	int error_code = 0;
	void *addr = dma_buf_vmap(dmabuf);

	if (!addr) {
		*vaddr = 0;
		error_code = -ENOSPC;
	} else {
		*vaddr = (uintptr_t)addr;
	}

	return error_code;
}

void cam_compat_util_put_dmabuf_va(struct dma_buf *dmabuf, void *vaddr)
{
	dma_buf_vunmap(dmabuf, vaddr);
}
#endif


struct sg_table *cam_compat_dmabuf_map_attach(struct dma_buf_attachment *attach,
	enum dma_data_direction dma_dir)
{
#if (KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE)
	return dma_buf_map_attachment_unlocked(attach, dma_dir);
#else
	return dma_buf_map_attachment(attach, dma_dir);
#endif
}

void cam_compat_dmabuf_unmap_attach(struct dma_buf_attachment *attach,
	struct sg_table *table, enum dma_data_direction dma_dir)
{
#if (KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE)
	dma_buf_unmap_attachment_unlocked(attach, table, dma_dir);
#else
	dma_buf_unmap_attachment(attach, table, dma_dir);
#endif
}


#if (KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE)
void cam_smmu_util_iommu_custom(struct device *dev,
	dma_addr_t discard_start, size_t discard_length)
{

}

int cam_req_mgr_ordered_list_cmp(void *priv,
	const struct list_head *head_1, const struct list_head *head_2)
{
	return cam_subdev_list_cmp(list_entry(head_1, struct cam_subdev, list),
		list_entry(head_2, struct cam_subdev, list));
}

#else
void cam_smmu_util_iommu_custom(struct device *dev,
	dma_addr_t discard_start, size_t discard_length)
{
	iommu_dma_enable_best_fit_algo(dev);

	if (discard_start)
		iommu_dma_reserve_iova(dev, discard_start, discard_length);

	return;
}

int cam_req_mgr_ordered_list_cmp(void *priv,
	struct list_head *head_1, struct list_head *head_2)
{
	return cam_subdev_list_cmp(list_entry(head_1, struct cam_subdev, list),
		list_entry(head_2, struct cam_subdev, list));
}

#endif

#if KERNEL_VERSION(5, 15, 0) <= LINUX_VERSION_CODE
int cam_get_subpart_info(uint32_t *part_info, uint32_t max_num_cam)
{
	int rc = 0;
	int num_cam;

	num_cam = socinfo_get_part_count(PART_CAMERA);
	CAM_DBG(CAM_CPAS, "number of cameras: %d", num_cam);
	if (num_cam != max_num_cam) {
		CAM_ERR(CAM_CPAS, "Unsupported number of parts: %d", num_cam);
		return -EINVAL;
	}

	/*
	 * If bit value in part_info is "0" then HW is available.
	 * If bit value in part_info is "1" then HW is unavailable.
	 */
	rc = socinfo_get_subpart_info(PART_CAMERA, part_info, num_cam);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Failed while getting subpart_info, rc = %d.", rc);
		return rc;
	}

	return 0;
}
#else
int cam_get_subpart_info(uint32_t *part_info, uint32_t max_num_cam)
{
	return 0;
}
#endif

#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
void cam_eeprom_spi_driver_remove(struct spi_device *sdev)
{
	struct v4l2_subdev             *sd = spi_get_drvdata(sdev);
	struct cam_eeprom_ctrl_t       *e_ctrl;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_hw_soc_info         *soc_info;

	if (!sd) {
		CAM_ERR(CAM_EEPROM, "Subdevice is NULL");
		return;
	}

	e_ctrl = (struct cam_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "eeprom device is NULL");
		return;
	}

	soc_info = &e_ctrl->soc_info;
	mutex_lock(&(e_ctrl->eeprom_mutex));
	cam_eeprom_shutdown(e_ctrl);
	mutex_unlock(&(e_ctrl->eeprom_mutex));
	mutex_destroy(&(e_ctrl->eeprom_mutex));
	cam_unregister_subdev(&(e_ctrl->v4l2_dev_str));
	kfree(e_ctrl->io_master_info.spi_client);
	e_ctrl->io_master_info.spi_client = NULL;
	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	if (soc_private) {
		kfree(soc_private->power_info.gpio_num_info);
		soc_private->power_info.gpio_num_info = NULL;
		kfree(soc_private);
		soc_private = NULL;
	}
	v4l2_set_subdevdata(&e_ctrl->v4l2_dev_str.sd, NULL);
	kfree(e_ctrl);
}

int cam_compat_util_get_irq(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	soc_info->irq_num = platform_get_irq(soc_info->pdev, 0);
	if (soc_info->irq_num < 0) {
		rc = soc_info->irq_num;
		return rc;
	}

	return rc;
}
#else
int cam_eeprom_spi_driver_remove(struct spi_device *sdev)
{
	struct v4l2_subdev             *sd = spi_get_drvdata(sdev);
	struct cam_eeprom_ctrl_t       *e_ctrl;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_hw_soc_info         *soc_info;

	if (!sd) {
		CAM_ERR(CAM_EEPROM, "Subdevice is NULL");
		return -EINVAL;
	}

	e_ctrl = (struct cam_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "eeprom device is NULL");
		return -EINVAL;
	}

	soc_info = &e_ctrl->soc_info;
	mutex_lock(&(e_ctrl->eeprom_mutex));
	cam_eeprom_shutdown(e_ctrl);
	mutex_unlock(&(e_ctrl->eeprom_mutex));
	mutex_destroy(&(e_ctrl->eeprom_mutex));
	cam_unregister_subdev(&(e_ctrl->v4l2_dev_str));
	kfree(e_ctrl->io_master_info.spi_client);
	e_ctrl->io_master_info.spi_client = NULL;
	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	if (soc_private) {
		kfree(soc_private->power_info.gpio_num_info);
		soc_private->power_info.gpio_num_info = NULL;
		kfree(soc_private);
		soc_private = NULL;
	}
	v4l2_set_subdevdata(&e_ctrl->v4l2_dev_str.sd, NULL);
	kfree(e_ctrl);

	return 0;
}

int cam_compat_util_get_irq(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	soc_info->irq_line =
		platform_get_resource_byname(soc_info->pdev,
		IORESOURCE_IRQ, soc_info->irq_name);
	if (!soc_info->irq_line) {
		rc = -ENODEV;
		return rc;
	}
	soc_info->irq_num = soc_info->irq_line->start;

	return rc;
}
#endif

#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
int cam_iommu_map(struct iommu_domain *domain,
	size_t firmware_start, phys_addr_t fw_hdl,
	size_t firmware_len, int prot)
{
	int rc = 0;

	rc = iommu_map(domain, firmware_start,
			fw_hdl,	firmware_len,
			prot, GFP_ATOMIC);
	return rc;
}
#else
int cam_iommu_map(struct iommu_domain *domain,
	size_t firmware_start, phys_addr_t fw_hdl,
	size_t firmware_len, int prot)
{
	int rc = 0;

	rc = iommu_map(domain, firmware_start,
			fw_hdl,	firmware_len,
			prot);
	return rc;
}
#endif

#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
size_t cam_iommu_map_sg(struct iommu_domain *domain,
	dma_addr_t iova_start, struct scatterlist *sgl,
	uint64_t orig_nents, int prot)
{
	size_t size = 0;

	size = iommu_map_sg(domain,
			iova_start,
			sgl, orig_nents,
			prot, GFP_ATOMIC);
	return size;
}
#else
size_t cam_iommu_map_sg(struct iommu_domain *domain,
	dma_addr_t iova_start, struct scatterlist *sgl,
	uint64_t orig_nents, int prot)
{
	size_t size = 0;

	size = iommu_map_sg(domain, iova_start,
			sgl, orig_nents,
			prot);
	return size;
}
#endif

int16_t cam_get_gpio_counts(struct cam_hw_soc_info *soc_info)
{
	struct device_node *of_node = NULL;
	int16_t gpio_array_size = 0;

	of_node = soc_info->dev->of_node;
#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
	gpio_array_size = of_count_phandle_with_args(
		of_node, "gpios", "#gpio-cells");
#else
	gpio_array_size = of_gpio_count(of_node);
#endif

	return gpio_array_size;
}

uint16_t cam_get_named_gpio(struct cam_hw_soc_info *soc_info,
	int index)
{
	struct device_node *of_node = NULL;
	uint16_t gpio_pin = 0;

	of_node = soc_info->dev->of_node;
#if KERNEL_VERSION(6, 2, 0) <= LINUX_VERSION_CODE
	gpio_pin = of_get_named_gpio(of_node, "gpios", index);
#else
	gpio_pin = of_get_gpio(of_node, index);
#endif

	return gpio_pin;
}

inline struct icc_path *cam_icc_get_path(struct device *dev,
		const int src_id, const int dst_id, const char *path_name, bool use_path_name)
{
	CAM_DBG(CAM_UTIL, "Get icc path name: %s src_id:%d dst_id:%d use_path_name:%s", path_name,
		src_id, dst_id, CAM_BOOL_TO_YESNO(use_path_name));

#if KERNEL_VERSION(6, 5, 0) <= LINUX_VERSION_CODE
	if (!use_path_name) {
		CAM_ERR(CAM_UTIL, "Must use path names to get icc path handle");
		return NULL;
	}

	return of_icc_get(dev, path_name);
#else
	if (use_path_name)
		return of_icc_get(dev, path_name);
	else
		return icc_get(dev, src_id, dst_id);
#endif
}


