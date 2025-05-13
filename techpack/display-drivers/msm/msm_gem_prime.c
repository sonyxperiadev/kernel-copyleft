/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_mmu.h"
#include "msm_kms.h"
#include <linux/module.h>

#include <drm/drm_drv.h>

#include <linux/qcom-dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/version.h>
#include <linux/mem-buf.h>
#include <soc/qcom/secure_buffer.h>
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
#include <linux/qti-smmu-proxy-callbacks.h>
#elif (KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE)
#include <linux/ion.h>
#include <linux/msm_ion.h>
#endif

struct sg_table *msm_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	int npages = obj->size >> PAGE_SHIFT;

	if (WARN_ON(!msm_obj->pages))  /* should have already pinned! */
		return NULL;

	return drm_prime_pages_to_sg(obj->dev, msm_obj->pages, npages);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
int msm_gem_prime_vmap(struct drm_gem_object *obj, struct iosys_map *map)
{
	map->vaddr = msm_gem_get_vaddr(obj);
	return IS_ERR_OR_NULL(map->vaddr);
}
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
int msm_gem_prime_vmap(struct drm_gem_object *obj, struct dma_buf_map *map)
{
	map->vaddr = msm_gem_get_vaddr(obj);
	return IS_ERR_OR_NULL(map->vaddr);
}
#else
void *msm_gem_prime_vmap(struct drm_gem_object *obj)
{
	return msm_gem_get_vaddr(obj);
}
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
void msm_gem_prime_vunmap(struct drm_gem_object *obj, struct iosys_map *map)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
void msm_gem_prime_vunmap(struct drm_gem_object *obj, struct dma_buf_map *map)
#else
void msm_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr)
#endif
{
	msm_gem_put_vaddr(obj);
}

int msm_gem_prime_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	if (ret < 0)
		return ret;

	return msm_gem_mmap_obj(vma->vm_private_data, vma);
}

struct drm_gem_object *msm_gem_prime_import_sg_table(struct drm_device *dev,
		struct dma_buf_attachment *attach, struct sg_table *sg)
{
	return msm_gem_import(dev, attach->dmabuf, sg);
}

int msm_gem_prime_pin(struct drm_gem_object *obj)
{
	if (!obj->import_attach)
		msm_gem_get_pages(obj);
	return 0;
}

void msm_gem_prime_unpin(struct drm_gem_object *obj)
{
	if (!obj->import_attach)
		msm_gem_put_pages(obj);
}

struct drm_gem_object *msm_gem_prime_import(struct drm_device *dev,
					    struct dma_buf *dma_buf)
{
	struct dma_buf_attachment *attach;
	struct sg_table *sgt = NULL;
	struct drm_gem_object *obj;
	struct device *attach_dev = NULL;
	struct msm_drm_private *priv;
	struct msm_kms *kms;
	bool lazy_unmap = true;
	bool is_vmid_tvm = false, is_vmid_cp_pixel = false;
	bool is_vmid_sec_display = false, is_vmid_cam_preview = false;
	int *vmid_list, *perms_list;
	int nelems = 0, i, ret;
	unsigned long dma_map_attrs = 0;

	if (!dma_buf || !dev->dev_private)
		return ERR_PTR(-EINVAL);

	priv = dev->dev_private;
	kms = priv->kms;

	if (dma_buf->priv && !dma_buf->ops->begin_cpu_access) {
		obj = dma_buf->priv;
		if (obj->dev == dev) {
			/*
			 * Importing dmabuf exported from out own gem increases
			 * refcount on gem itself instead of f_count of dmabuf.
			 */
			drm_gem_object_get(obj);
			return obj;
		}
	}

	if (!dev->driver->gem_prime_import_sg_table) {
		DRM_ERROR("NULL gem_prime_import_sg_table\n");
		return ERR_PTR(-EINVAL);
	}

	get_dma_buf(dma_buf);

	if (!kms || !kms->funcs->get_address_space_device) {
		DRM_ERROR("invalid kms ops\n");
		ret = -EINVAL;
		goto fail_put;
	}

	ret = mem_buf_dma_buf_copy_vmperm(dma_buf, &vmid_list, &perms_list, &nelems);
	if (ret) {
		DRM_ERROR("get vmid list failure, ret:%d", ret);
		goto fail_put;
	}

	for (i = 0; i < nelems; i++) {
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
		/* avoid VMID checks in trusted-vm, set flag in HLOS when only VMID_TVM is set */
		if ((vmid_list[i] == VMID_TVM) &&
				(!kms->funcs->in_trusted_vm || !kms->funcs->in_trusted_vm(kms))) {
			is_vmid_tvm = true;
			dma_map_attrs = DMA_ATTR_QTI_SMMU_PROXY_MAP;
		}
#endif
		if (vmid_list[i] == VMID_CP_PIXEL) {
			is_vmid_cp_pixel = true;
			is_vmid_tvm = false;
			dma_map_attrs = 0;
			break;
		} else if (vmid_list[i] == VMID_CP_CAMERA_PREVIEW) {
			is_vmid_cam_preview = true;
			break;
		} else if (vmid_list[i] == VMID_CP_SEC_DISPLAY) {
			is_vmid_sec_display = true;
			break;
		}

	}

	/* mem_buf_dma_buf_copy_vmperm uses kmemdup, do kfree to free up the memory */
	kfree(vmid_list);
	kfree(perms_list);

	/*
	 * - attach default drm device for VMID_TVM-only or when IOMMU is not available
	 * - avoid using lazy unmap feature as it doesn't add value without nested translations
	 */
	if (is_vmid_cp_pixel) {
		attach_dev = kms->funcs->get_address_space_device(kms, MSM_SMMU_DOMAIN_SECURE);
	} else if (!iommu_present(&platform_bus_type) || is_vmid_tvm || is_vmid_cam_preview
			|| is_vmid_sec_display) {
		attach_dev = dev->dev;
		lazy_unmap = false;
	} else {
		attach_dev = kms->funcs->get_address_space_device(kms, MSM_SMMU_DOMAIN_UNSECURE);
	}

	/*
	 * While transitioning from secure use-cases, the secure/non-secure
	 * context bank might still not be attached back, while the
	 * prime_fd_to_handle call is made for the next frame. Attach those
	 * buffers to default drm device and reattaching with the correct
	 * context-bank will be handled in msm_gem_delayed_import.
	 */
	if (!attach_dev) {
		DRM_DEBUG("attaching dma buf with default drm device\n");
		attach_dev = dev->dev;
	}

	attach = dma_buf_attach(dma_buf, attach_dev);
	if (IS_ERR(attach)) {
		DRM_ERROR("dma_buf_attach failure, err=%ld\n", PTR_ERR(attach));
		return ERR_CAST(attach);
	}

	/*
	 * For cached buffers where CPU access is required, dma_map_attachment
	 * must be called now to allow user-space to perform cpu sync begin/end
	 * otherwise do delayed mapping during the commit.
	 */
	if (lazy_unmap)
		attach->dma_map_attrs |= DMA_ATTR_DELAYED_UNMAP;

	attach->dma_map_attrs |= dma_map_attrs;

	/*
	 * avoid map_attachment for S2-only buffers and TVM buffers as it needs to be mapped
	 * after the SID switch scm_call and will be handled during msm_gem_get_dma_addr
	 */
	if (!is_vmid_tvm && !is_vmid_cam_preview && !is_vmid_sec_display) {
		sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
		if (IS_ERR(sgt)) {
			ret = PTR_ERR(sgt);
			DRM_ERROR("dma_buf_map_attachment failure, err=%d\n", ret);
			goto fail_detach;
		}
	} else {
		DRM_DEBUG("deferring dma_buf_map_attachment; tvm:%d, sec_cam:%d, sec_disp:%d\n",
				is_vmid_tvm, is_vmid_cam_preview, is_vmid_sec_display);
	}

	/*
	 * If importing a NULL sg table (i.e. for uncached buffers),
	 * create a drm gem object with only the dma buf attachment.
	 */
	obj = dev->driver->gem_prime_import_sg_table(dev, attach, sgt);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		DRM_ERROR("gem_prime_import_sg_table failure, err=%d\n", ret);
		goto fail_unmap;
	}

	obj->import_attach = attach;

	return obj;

fail_unmap:
	if (sgt)
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
fail_detach:
	dma_buf_detach(dma_buf, attach);
fail_put:
	dma_buf_put(dma_buf);

	return ERR_PTR(ret);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
MODULE_IMPORT_NS(DMA_BUF);
#endif
