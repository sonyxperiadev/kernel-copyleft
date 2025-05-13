/*
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <linux/qcom-dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/pfn_t.h>
#include <linux/version.h>
#include <linux/module.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
#include <linux/ion.h>
#endif

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_mmu.h"
#include "sde_dbg.h"

#define GUARD_BYTES (BIT(8) - 1)
#define ALIGNED_OFFSET (U32_MAX & ~(GUARD_BYTES))

static void msm_gem_vunmap_locked(struct drm_gem_object *obj);


static dma_addr_t physaddr(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_drm_private *priv = obj->dev->dev_private;
	return (((dma_addr_t)msm_obj->vram_node->start) << PAGE_SHIFT) +
			priv->vram.paddr;
}

static bool use_pages(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	return !msm_obj->vram_node;
}

/* allocate pages from VRAM carveout, used when no IOMMU: */
static struct page **get_pages_vram(struct drm_gem_object *obj, int npages)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_drm_private *priv = obj->dev->dev_private;
	dma_addr_t paddr;
	struct page **p;
	int ret, i;

	p = kvmalloc_array(npages, sizeof(struct page *), GFP_KERNEL);
	if (!p)
		return ERR_PTR(-ENOMEM);

	spin_lock(&priv->vram.lock);
	ret = drm_mm_insert_node(&priv->vram.mm, msm_obj->vram_node, npages);
	spin_unlock(&priv->vram.lock);
	if (ret) {
		kvfree(p);
		return ERR_PTR(ret);
	}

	paddr = physaddr(obj);
	for (i = 0; i < npages; i++) {
		p[i] = phys_to_page(paddr);
		paddr += PAGE_SIZE;
	}

	return p;
}

static struct page **get_pages(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct device *aspace_dev;

	if (obj->import_attach)
		return msm_obj->pages;

	if (!msm_obj->pages) {
		struct drm_device *dev = obj->dev;
		struct page **p;
		int npages = obj->size >> PAGE_SHIFT;

		if (use_pages(obj))
			p = drm_gem_get_pages(obj);
		else
			p = get_pages_vram(obj, npages);

		if (IS_ERR(p)) {
			DISP_DEV_ERR(dev->dev, "could not get pages: %ld\n",
					PTR_ERR(p));
			return p;
		}

		msm_obj->pages = p;

		msm_obj->sgt = drm_prime_pages_to_sg(dev, p, npages);
		if (IS_ERR(msm_obj->sgt)) {
			void *ptr = ERR_CAST(msm_obj->sgt);

			DISP_DEV_ERR(dev->dev, "failed to allocate sgt\n");
			msm_obj->sgt = NULL;
			return ptr;
		}

		if (msm_obj->vram_node) {
			goto end;
		/*
		 * For non-cached buffers, ensure the new pages are clean
		 * because display controller, GPU, etc. are not coherent
		 */
		} else if (msm_obj->flags & (MSM_BO_WC|MSM_BO_UNCACHED)) {
			aspace_dev = msm_gem_get_aspace_device(msm_obj->aspace);
			if (aspace_dev) {
				dma_map_sg(aspace_dev, msm_obj->sgt->sgl,
					msm_obj->sgt->nents, DMA_BIDIRECTIONAL);
				/* mark the buffer as external buffer */
				msm_obj->flags |= MSM_BO_EXTBUF;
			} else {
				DRM_ERROR("failed to get aspace_device\n");
			}
		}
	}

end:
	return msm_obj->pages;
}

static void put_pages_vram(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_drm_private *priv = obj->dev->dev_private;

	spin_lock(&priv->vram.lock);
	drm_mm_remove_node(msm_obj->vram_node);
	spin_unlock(&priv->vram.lock);

	kvfree(msm_obj->pages);
}

static void put_pages(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	if (msm_obj->pages) {
		if (msm_obj->sgt) {
			sg_free_table(msm_obj->sgt);
			kfree(msm_obj->sgt);
		}

		if (use_pages(obj))
			drm_gem_put_pages(obj, msm_obj->pages, true, false);
		else
			put_pages_vram(obj);

		msm_obj->pages = NULL;
	}
}

struct page **msm_gem_get_pages(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct page **p;

	mutex_lock(&msm_obj->lock);

	if (WARN_ON(msm_obj->madv != MSM_MADV_WILLNEED)) {
		mutex_unlock(&msm_obj->lock);
		return ERR_PTR(-EBUSY);
	}

	p = get_pages(obj);
	mutex_unlock(&msm_obj->lock);
	return p;
}

void msm_gem_put_pages(struct drm_gem_object *obj)
{
	/* when we start tracking the pin count, then do something here */
}

void msm_gem_sync(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj;
	struct device *aspace_dev;

	if (!obj)
		return;

	msm_obj = to_msm_bo(obj);

	if (msm_obj->vram_node)
		return;
	/*
	 * dma_sync_sg_for_device synchronises a single contiguous or
	 * scatter/gather mapping for the CPU and device.
	 */
	aspace_dev = msm_gem_get_aspace_device(msm_obj->aspace);
	if (aspace_dev)
		dma_sync_sg_for_device(aspace_dev, msm_obj->sgt->sgl,
				msm_obj->sgt->nents, DMA_BIDIRECTIONAL);
	else
		DRM_ERROR("failed to get aspace_device\n");
}


int msm_gem_mmap_obj(struct drm_gem_object *obj,
		struct vm_area_struct *vma)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 25))
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;
#else
	vm_flags_mod(vma, VM_MIXEDMAP, VM_PFNMAP);
#endif

	if (msm_obj->flags & MSM_BO_WC) {
		vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	} else if (msm_obj->flags & MSM_BO_UNCACHED) {
		vma->vm_page_prot = pgprot_noncached(vm_get_page_prot(vma->vm_flags));
	} else {
		/*
		 * Shunt off cached objs to shmem file so they have their own
		 * address_space (so unmap_mapping_range does what we want,
		 * in particular in the case of mmap'd dmabufs)
		 */
		fput(vma->vm_file);
		get_file(obj->filp);
		vma->vm_pgoff = 0;
		vma->vm_file  = obj->filp;

		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	}

	return 0;
}

int msm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret) {
		DBG("mmap failed: %d", ret);
		return ret;
	}

	return msm_gem_mmap_obj(vma->vm_private_data, vma);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
static vm_fault_t msm_gem_fault(struct vm_fault *vmf)
#else
vm_fault_t msm_gem_fault(struct vm_fault *vmf)
#endif
{
	struct vm_area_struct *vma = vmf->vma;
	struct drm_gem_object *obj = vma->vm_private_data;
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct page **pages;
	unsigned long pfn;
	pgoff_t pgoff;
	int err;
	vm_fault_t ret;

	/*
	 * vm_ops.open/drm_gem_mmap_obj and close get and put
	 * a reference on obj. So, we dont need to hold one here.
	 */
	err = mutex_lock_interruptible(&msm_obj->lock);
	if (err) {
		ret = VM_FAULT_NOPAGE;
		goto out;
	}

	if (WARN_ON(msm_obj->madv != MSM_MADV_WILLNEED)) {
		mutex_unlock(&msm_obj->lock);
		return VM_FAULT_SIGBUS;
	}

	/* make sure we have pages attached now */
	pages = get_pages(obj);
	if (IS_ERR(pages)) {
		ret = vmf_error(PTR_ERR(pages));
		goto out_unlock;
	}

	/* We don't use vmf->pgoff since that has the fake offset: */
	pgoff = (vmf->address - vma->vm_start) >> PAGE_SHIFT;

	pfn = page_to_pfn(pages[pgoff]);

	VERB("Inserting %pK pfn %lx, pa %lx", (void *)vmf->address,
			pfn, pfn << PAGE_SHIFT);

	ret = vmf_insert_mixed(vma, vmf->address, __pfn_to_pfn_t(pfn, PFN_DEV));
out_unlock:
	mutex_unlock(&msm_obj->lock);
out:
	return ret;
}

/** get mmap offset */
static uint64_t mmap_offset(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	int ret;

	WARN_ON(!mutex_is_locked(&msm_obj->lock));

	/* Make it mmapable */
	ret = drm_gem_create_mmap_offset(obj);

	if (ret) {
		DISP_DEV_ERR(dev->dev, "could not allocate mmap offset\n");
		return 0;
	}

	return drm_vma_node_offset_addr(&obj->vma_node);
}

uint64_t msm_gem_mmap_offset(struct drm_gem_object *obj)
{
	uint64_t offset;
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	mutex_lock(&msm_obj->lock);
	offset = mmap_offset(obj);
	mutex_unlock(&msm_obj->lock);
	return offset;
}

dma_addr_t msm_gem_get_dma_addr(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct sg_table *sgt;

	if (!msm_obj->sgt) {
		sgt = dma_buf_map_attachment(obj->import_attach,
						DMA_BIDIRECTIONAL);
		if (IS_ERR_OR_NULL(sgt)) {
			DRM_ERROR("dma_buf_map_attachment failure, err=%ld\n",
					PTR_ERR(sgt));
			return 0;
		}
		msm_obj->sgt = sgt;
	}

	return sg_dma_address(msm_obj->sgt->sgl);
}

static struct msm_gem_vma *add_vma(struct drm_gem_object *obj,
		struct msm_gem_address_space *aspace)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_gem_vma *vma;

	WARN_ON(!mutex_is_locked(&msm_obj->lock));

	vma = kzalloc(sizeof(*vma), GFP_KERNEL);
	if (!vma)
		return ERR_PTR(-ENOMEM);

	vma->aspace = aspace;
	msm_obj->aspace = aspace;

	list_add_tail(&vma->list, &msm_obj->vmas);

	return vma;
}

static struct msm_gem_vma *lookup_vma(struct drm_gem_object *obj,
		struct msm_gem_address_space *aspace)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_gem_vma *vma;

	WARN_ON(!mutex_is_locked(&msm_obj->lock));

	list_for_each_entry(vma, &msm_obj->vmas, list) {
		if (vma->aspace == aspace)
			return vma;
	}

	return NULL;
}

static void del_vma(struct msm_gem_vma *vma)
{
	if (!vma)
		return;

	list_del(&vma->list);
	kfree(vma);
}

/* Called with msm_obj->lock locked */
static void
put_iova(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_gem_vma *vma, *tmp;

	WARN_ON(!mutex_is_locked(&msm_obj->lock));

	list_for_each_entry_safe(vma, tmp, &msm_obj->vmas, list) {
		msm_gem_unmap_vma(vma->aspace, vma, msm_obj->sgt,
				msm_obj->flags);
		/*
		 * put_iova removes the domain connected to the obj which makes
		 * the aspace inaccessible. Store the aspace, as it is used to
		 * update the active_list during gem_free_obj and gem_purge.
		 */
		msm_obj->aspace = vma->aspace;
		del_vma(vma);
	}
}

/* get iova, taking a reference.  Should have a matching put */
static int msm_gem_get_iova_locked(struct drm_gem_object *obj,
		struct msm_gem_address_space *aspace, uint64_t *iova)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_gem_vma *vma;
	int ret = 0;

	WARN_ON(!mutex_is_locked(&msm_obj->lock));

	vma = lookup_vma(obj, aspace);

	if (!vma) {
		struct page **pages;
		struct device *dev;
		struct dma_buf *dmabuf;
		bool reattach = false;
		unsigned long dma_map_attrs;

		dev = msm_gem_get_aspace_device(aspace);
		if ((dev && obj->import_attach) &&
				((dev != obj->import_attach->dev) ||
				msm_obj->obj_dirty)) {

			if (of_device_is_compatible(dev->of_node, "qcom,smmu_sde_unsec") &&
				of_device_is_compatible(obj->import_attach->dev->of_node,
				"qcom,smmu_sde_sec")) {
				SDE_EVT32(obj->import_attach->dev, dev, msm_obj->sgt,
						 msm_obj->obj_dirty);
				DRM_ERROR("gem obj found mapped to %s, now requesting map on %s",
					dev_name(obj->import_attach->dev), dev_name(dev));
				return -EINVAL;
			}

			dmabuf = obj->import_attach->dmabuf;
			dma_map_attrs = obj->import_attach->dma_map_attrs;

			DRM_DEBUG("detach nsec-dev:%pK attach sec-dev:%pK\n",
					obj->import_attach->dev, dev);
			SDE_EVT32(obj->import_attach->dev, dev, msm_obj->sgt,
					 msm_obj->obj_dirty);

			if (msm_obj->sgt)
				dma_buf_unmap_attachment(obj->import_attach,
					msm_obj->sgt, DMA_BIDIRECTIONAL);
			dma_buf_detach(dmabuf, obj->import_attach);

			obj->import_attach = dma_buf_attach(dmabuf, dev);
			if (IS_ERR(obj->import_attach)) {
				DRM_ERROR("dma_buf_attach failure, err=%ld\n",
						PTR_ERR(obj->import_attach));
				ret = PTR_ERR(obj->import_attach);
				return ret;
			}
			/*
			 * obj->import_attach is created as part of dma_buf_attach.
			 * Re-apply the dma_map_attr in this case to be in sync
			 * with iommu_map attrs during map_attachment callback.
			 */
			obj->import_attach->dma_map_attrs |= dma_map_attrs;
			msm_obj->obj_dirty = false;
			reattach = true;
		}

		/* perform delayed import for buffers without existing sgt */
		if (((msm_obj->flags & MSM_BO_EXTBUF) && !(msm_obj->sgt))
				|| reattach) {
			ret = msm_gem_delayed_import(obj);
			if (ret) {
				DRM_ERROR("delayed dma-buf import failed %d\n",
						ret);
				msm_obj->obj_dirty = true;
				return ret;
			}
		}

		vma = add_vma(obj, aspace);
		if (IS_ERR(vma)) {
			ret = PTR_ERR(vma);
			return ret;
		}

		pages = get_pages(obj);
		if (IS_ERR(pages)) {
			ret = PTR_ERR(pages);
			goto fail;
		}

		ret = msm_gem_map_vma(aspace, vma, msm_obj->sgt,
				obj->size >> PAGE_SHIFT,
				msm_obj->flags);
		if (ret)
			goto fail;
	}

	*iova = vma->iova;

	if (aspace &&  !msm_obj->in_active_list) {
		mutex_lock(&aspace->list_lock);
		msm_gem_add_obj_to_aspace_active_list(aspace, obj);
		mutex_unlock(&aspace->list_lock);
	}

	return 0;

fail:
	del_vma(vma);
	return ret;
}

int msm_gem_get_iova(struct drm_gem_object *obj,
		struct msm_gem_address_space *aspace, uint64_t *iova)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	int ret;

	mutex_lock(&msm_obj->lock);
	ret = msm_gem_get_iova_locked(obj, aspace, iova);
	mutex_unlock(&msm_obj->lock);

	return ret;
}

/* get iova without taking a reference, used in places where you have
 * already done a 'msm_gem_get_iova()'.
 */
uint64_t msm_gem_iova(struct drm_gem_object *obj,
		struct msm_gem_address_space *aspace)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_gem_vma *vma;

	mutex_lock(&msm_obj->lock);
	vma = lookup_vma(obj, aspace);
	mutex_unlock(&msm_obj->lock);
	WARN_ON(!vma);

	return vma ? vma->iova : 0;
}

/*
 * Unpin a iova by updating the reference counts. The memory isn't actually
 * purged until something else (shrinker, mm_notifier, destroy, etc) decides
 * to get rid of it
 */
void msm_gem_unpin_iova(struct drm_gem_object *obj,
		struct msm_gem_address_space *aspace)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct msm_gem_vma *vma;

	mutex_lock(&msm_obj->lock);
	vma = lookup_vma(obj, aspace);

	if (!WARN_ON(!vma))
		msm_gem_unmap_vma(vma->aspace, vma, msm_obj->sgt,
				msm_obj->flags);

	mutex_unlock(&msm_obj->lock);
}

void msm_gem_put_iova(struct drm_gem_object *obj,
		struct msm_gem_address_space *aspace)
{
	// XXX TODO ..
	// NOTE: probably don't need a _locked() version.. we wouldn't
	// normally unmap here, but instead just mark that it could be
	// unmapped (if the iova refcnt drops to zero), but then later
	// if another _get_iova_locked() fails we can start unmapping
	// things that are no longer needed..
}

void msm_gem_aspace_domain_attach_detach_update(
		struct msm_gem_address_space *aspace,
		bool is_detach)
{
	struct msm_gem_object *msm_obj;
	struct drm_gem_object *obj;
	struct aspace_client *aclient;
	int ret;
	uint64_t iova;

	if (!aspace)
		return;

	mutex_lock(&aspace->list_lock);
	if (is_detach) {
		/* Indicate to clients domain is getting detached */
		list_for_each_entry(aclient, &aspace->clients, list) {
			if (aclient->cb)
				aclient->cb(aclient->cb_data,
						is_detach);
		}

		/**
		 * Unmap active buffers,
		 * typically clients should do this when the callback is called,
		 * but this needs to be done for the buffers which are not
		 * attached to any planes.
		 */
		list_for_each_entry(msm_obj, &aspace->active_list, iova_list) {
			obj = &msm_obj->base;
			if (obj->import_attach) {
				mutex_lock(&msm_obj->lock);
				put_iova(obj);
				msm_obj->obj_dirty = true;
				mutex_unlock(&msm_obj->lock);
			}
		}
	} else {
		/* map active buffers */
		list_for_each_entry(msm_obj, &aspace->active_list, iova_list) {
			obj = &msm_obj->base;
			ret = msm_gem_get_iova(obj, aspace, &iova);
			if (ret) {
				mutex_unlock(&aspace->list_lock);
				return;
			}
		}

		/* Indicate to clients domain is attached */
		list_for_each_entry(aclient, &aspace->clients, list) {
			if (aclient->cb)
				aclient->cb(aclient->cb_data,
						is_detach);
		}
	}
	mutex_unlock(&aspace->list_lock);
}

int msm_gem_dumb_create(struct drm_file *file, struct drm_device *dev,
		struct drm_mode_create_dumb *args)
{
	args->pitch = align_pitch(args->width, args->bpp);
	args->size  = PAGE_ALIGN(args->pitch * args->height);
	return msm_gem_new_handle(dev, file, args->size,
			MSM_BO_SCANOUT | MSM_BO_CACHED, &args->handle, "dumb");
}

int msm_gem_dumb_map_offset(struct drm_file *file, struct drm_device *dev,
		uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	/* GEM does all our handle to object mapping */
	obj = drm_gem_object_lookup(file, handle);
	if (obj == NULL) {
		ret = -ENOENT;
		goto fail;
	}

	*offset = msm_gem_mmap_offset(obj);

	drm_gem_object_put(obj);

fail:
	return ret;
}

static void *get_vaddr(struct drm_gem_object *obj, unsigned madv)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	struct iosys_map map;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct dma_buf_map map;
#endif
	int ret = 0;

	mutex_lock(&msm_obj->lock);

	if (WARN_ON(msm_obj->madv > madv)) {
		DISP_DEV_ERR(obj->dev->dev, "Invalid madv state: %u vs %u\n",
			msm_obj->madv, madv);
		mutex_unlock(&msm_obj->lock);
		return ERR_PTR(-EBUSY);
	}

	/* increment vmap_count *before* vmap() call, so shrinker can
	 * check vmap_count (is_vunmapable()) outside of msm_obj->lock.
	 * This guarantees that we won't try to msm_gem_vunmap() this
	 * same object from within the vmap() call (while we already
	 * hold msm_obj->lock)
	 */
	msm_obj->vmap_count++;

	if (!msm_obj->vaddr) {
		struct page **pages = get_pages(obj);
		if (IS_ERR(pages)) {
			ret = PTR_ERR(pages);
			goto fail;
		}

		if (obj->import_attach) {
			if (obj->dev && obj->dev->dev && !dev_is_dma_coherent(obj->dev->dev)) {
				ret = dma_buf_begin_cpu_access(
					obj->import_attach->dmabuf, DMA_BIDIRECTIONAL);
				if (ret)
					goto fail;
			}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
			ret = dma_buf_vmap(obj->import_attach->dmabuf, &map);
			if (ret)
				goto fail;
			msm_obj->vaddr = map.vaddr;
#else
			msm_obj->vaddr = dma_buf_vmap(obj->import_attach->dmabuf);
#endif
		} else {
			msm_obj->vaddr = vmap(pages, obj->size >> PAGE_SHIFT,
				VM_MAP, PAGE_KERNEL);
		}

		if (msm_obj->vaddr == NULL) {
			ret = -ENOMEM;
			goto fail;
		}
	}

	mutex_unlock(&msm_obj->lock);
	return msm_obj->vaddr;

fail:
	msm_obj->vmap_count--;
	mutex_unlock(&msm_obj->lock);
	return ERR_PTR(ret);
}

void *msm_gem_get_vaddr(struct drm_gem_object *obj)
{
	return get_vaddr(obj, MSM_MADV_WILLNEED);
}

void msm_gem_put_vaddr(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	mutex_lock(&msm_obj->lock);
	WARN_ON(msm_obj->vmap_count < 1);
	msm_obj->vmap_count--;
	mutex_unlock(&msm_obj->lock);
}

/* Update madvise status, returns true if not purged, else
 * false or -errno.
 */
int msm_gem_madvise(struct drm_gem_object *obj, unsigned madv)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	mutex_lock(&msm_obj->lock);

	WARN_ON(!mutex_is_locked(&obj->dev->struct_mutex));

	if (msm_obj->madv != __MSM_MADV_PURGED)
		msm_obj->madv = madv;

	madv = msm_obj->madv;

	mutex_unlock(&msm_obj->lock);

	return (madv != __MSM_MADV_PURGED);
}

static void msm_gem_vunmap_locked(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(msm_obj->vaddr);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct dma_buf_map map = DMA_BUF_MAP_INIT_VADDR(msm_obj->vaddr);
#endif

	WARN_ON(!mutex_is_locked(&msm_obj->lock));

	if (!msm_obj->vaddr || WARN_ON(!is_vunmapable(msm_obj)))
		return;

	if (obj->import_attach) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
		dma_buf_vunmap(obj->import_attach->dmabuf, &map);
#else
		dma_buf_vunmap(obj->import_attach->dmabuf, msm_obj->vaddr);
#endif
		if (obj->dev && obj->dev->dev && !dev_is_dma_coherent(obj->dev->dev))
			dma_buf_end_cpu_access(obj->import_attach->dmabuf, DMA_BIDIRECTIONAL);
	} else {
		vunmap(msm_obj->vaddr);
	}

	msm_obj->vaddr = NULL;
}

void msm_gem_vunmap(struct drm_gem_object *obj, enum msm_gem_lock subclass)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	mutex_lock_nested(&msm_obj->lock, subclass);
	msm_gem_vunmap_locked(obj);
	mutex_unlock(&msm_obj->lock);
}

int msm_gem_cpu_prep(struct drm_gem_object *obj, uint32_t op, ktime_t *timeout)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	bool write = !!(op & MSM_PREP_WRITE);
	unsigned long remain =
		op & MSM_PREP_NOSYNC ? 0 : timeout_to_jiffies(timeout);
	long ret;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	ret = dma_resv_wait_timeout(msm_obj->resv, write, true, remain);
#else
	ret = dma_resv_wait_timeout_rcu(msm_obj->resv, write, true, remain);
#endif
	if (ret == 0)
		return remain == 0 ? -EBUSY : -ETIMEDOUT;
	else if (ret < 0)
		return ret;

	/* TODO cache maintenance */

	return 0;
}

int msm_gem_cpu_fini(struct drm_gem_object *obj)
{
	/* TODO cache maintenance */
	return 0;
}

/* don't call directly!  Use drm_gem_object_put() and friends */
void msm_gem_free_object(struct drm_gem_object *obj)
{
	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	struct drm_device *dev = obj->dev;
	struct msm_drm_private *priv = dev->dev_private;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(msm_obj->vaddr);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct dma_buf_map map = DMA_BUF_MAP_INIT_VADDR(msm_obj->vaddr);
#endif

	/* object should not be on active list: */
	WARN_ON(is_active(msm_obj));

	mutex_lock(&priv->mm_lock);
	list_del(&msm_obj->mm_list);
	mutex_unlock(&priv->mm_lock);

	mutex_lock(&msm_obj->lock);

	put_iova(obj);
	if (msm_obj->aspace) {
		mutex_lock(&msm_obj->aspace->list_lock);
		msm_gem_remove_obj_from_aspace_active_list(msm_obj->aspace,
				obj);
		mutex_unlock(&msm_obj->aspace->list_lock);
	}

	if (obj->import_attach) {
		if (msm_obj->vaddr)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
			dma_buf_vunmap(obj->import_attach->dmabuf, &map);
#else
			dma_buf_vunmap(obj->import_attach->dmabuf, msm_obj->vaddr);
#endif

		/* Don't drop the pages for imported dmabuf, as they are not
		 * ours, just free the array we allocated:
		 */
		if (msm_obj->pages)
			kvfree(msm_obj->pages);

		drm_prime_gem_destroy(obj, msm_obj->sgt);
	} else {
		msm_gem_vunmap_locked(obj);
		put_pages(obj);
	}

	if (msm_obj->resv == &msm_obj->_resv)
		dma_resv_fini(msm_obj->resv);

	drm_gem_object_release(obj);

	mutex_unlock(&msm_obj->lock);
	kfree(msm_obj);
}

/* convenience method to construct a GEM buffer object, and userspace handle */
int msm_gem_new_handle(struct drm_device *dev, struct drm_file *file,
		uint32_t size, uint32_t flags, uint32_t *handle,
		char *name)
{
	struct drm_gem_object *obj;
	int ret;

	obj = msm_gem_new(dev, size, flags);

	if (IS_ERR(obj))
		return PTR_ERR(obj);

	if (name)
		msm_gem_object_set_name(obj, "%s", name);

	ret = drm_gem_handle_create(file, obj, handle);

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_put(obj);

	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
static const struct vm_operations_struct vm_ops = {
	.fault = msm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct drm_gem_object_funcs msm_gem_object_funcs = {
	.free = msm_gem_free_object,
	.pin = msm_gem_prime_pin,
	.unpin = msm_gem_prime_unpin,
	.get_sg_table = msm_gem_prime_get_sg_table,
	.vmap = msm_gem_prime_vmap,
	.vunmap = msm_gem_prime_vunmap,
	.vm_ops = &vm_ops,
};
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
static int msm_gem_new_impl(struct drm_device *dev,
		uint32_t size, uint32_t flags,
		struct dma_resv *resv,
		struct drm_gem_object **obj)
{
#else
static int msm_gem_new_impl(struct drm_device *dev,
		uint32_t size, uint32_t flags,
		struct dma_resv *resv,
		struct drm_gem_object **obj,
		bool struct_mutex_locked)
{
	struct msm_drm_private *priv = dev->dev_private;
#endif
	struct msm_gem_object *msm_obj;

	switch (flags & MSM_BO_CACHE_MASK) {
	case MSM_BO_UNCACHED:
	case MSM_BO_CACHED:
	case MSM_BO_WC:
		break;
	default:
		DISP_DEV_ERR(dev->dev, "invalid cache flag: %x\n",
				(flags & MSM_BO_CACHE_MASK));
		return -EINVAL;
	}

	msm_obj = kzalloc(sizeof(*msm_obj), GFP_KERNEL);
	if (!msm_obj)
		return -ENOMEM;

	mutex_init(&msm_obj->lock);

	msm_obj->flags = flags;
	msm_obj->madv = MSM_MADV_WILLNEED;

	if (resv) {
		msm_obj->resv = resv;
	} else {
		msm_obj->resv = &msm_obj->_resv;
		dma_resv_init(msm_obj->resv);
	}

	INIT_LIST_HEAD(&msm_obj->submit_entry);
	INIT_LIST_HEAD(&msm_obj->vmas);
	INIT_LIST_HEAD(&msm_obj->iova_list);
	msm_obj->aspace = msm_gem_smmu_address_space_get(dev,
			MSM_SMMU_DOMAIN_UNSECURE);
	if (IS_ERR(msm_obj->aspace))
		msm_obj->aspace = NULL;
	msm_obj->in_active_list = false;
	msm_obj->obj_dirty = false;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
	mutex_lock(&priv->mm_lock);
	list_add_tail(&msm_obj->mm_list, &priv->inactive_list);
	mutex_unlock(&priv->mm_lock);
#endif

	*obj = &msm_obj->base;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	(*obj)->funcs = &msm_gem_object_funcs;
#endif

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
struct drm_gem_object *msm_gem_new(struct drm_device *dev, uint32_t size, uint32_t flags)
#else
static struct drm_gem_object *_msm_gem_new(struct drm_device *dev,
		uint32_t size, uint32_t flags, bool struct_mutex_locked)
#endif
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gem_object *msm_obj;
	struct drm_gem_object *obj = NULL;
	bool use_vram = false;
	int ret;

	size = PAGE_ALIGN(size);

	if (!iommu_present(&platform_bus_type))
		use_vram = true;
	else if ((flags & (MSM_BO_STOLEN | MSM_BO_SCANOUT)) && priv->vram.size)
		use_vram = true;

	if (WARN_ON(use_vram && !priv->vram.size))
		return ERR_PTR(-EINVAL);

	/* Disallow zero sized objects as they make the underlying
	 * infrastructure grumpy
	 */
	if (size == 0)
		return ERR_PTR(-EINVAL);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	ret = msm_gem_new_impl(dev, size, flags, NULL, &obj);
#else
	ret = msm_gem_new_impl(dev, size, flags, NULL, &obj, struct_mutex_locked);
#endif
	if (ret)
		goto fail;

	msm_obj = to_msm_bo(obj);

	if (use_vram) {
		struct msm_gem_vma *vma;
		struct page **pages;

		mutex_lock(&msm_obj->lock);

		vma = add_vma(obj, NULL);
		mutex_unlock(&msm_obj->lock);
		if (IS_ERR(vma)) {
			ret = PTR_ERR(vma);
			goto fail;
		}

		to_msm_bo(obj)->vram_node = &vma->node;

		drm_gem_private_object_init(dev, obj, size);

		pages = get_pages(obj);
		if (IS_ERR(pages)) {
			ret = PTR_ERR(pages);
			goto fail;
		}

		vma->iova = physaddr(obj);
	} else {
		ret = drm_gem_object_init(dev, obj, size);
		if (ret)
			goto fail;

		/*
		 * Our buffers are kept pinned, so allocating them from the
		 * MOVABLE zone is a really bad idea, and conflicts with CMA.
		 * See comments above new_inode() why this is required _and_
		 * expected if you're going to pin these pages.
		 */
		mapping_set_gfp_mask(obj->filp->f_mapping, GFP_HIGHUSER);
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	mutex_lock(&priv->mm_lock);
	list_add_tail(&msm_obj->mm_list, &priv->inactive_list);
	mutex_unlock(&priv->mm_lock);
#endif

	return obj;

fail:
	drm_gem_object_put(obj);
	return ERR_PTR(ret);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
struct drm_gem_object *msm_gem_new_locked(struct drm_device *dev,
		uint32_t size, uint32_t flags)
{
	return _msm_gem_new(dev, size, flags, true);
}

struct drm_gem_object *msm_gem_new(struct drm_device *dev,
		uint32_t size, uint32_t flags)
{
	return _msm_gem_new(dev, size, flags, false);
}
#endif

int msm_gem_delayed_import(struct drm_gem_object *obj)
{
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct msm_gem_object *msm_obj;
	int ret = 0;

	if (!obj) {
		DRM_ERROR("NULL drm gem object\n");
		return -EINVAL;
	}

	msm_obj = to_msm_bo(obj);

	if (!obj->import_attach) {
		DRM_ERROR("NULL dma_buf_attachment in drm gem object\n");
		return -EINVAL;
	}

	attach = obj->import_attach;
	attach->dma_map_attrs |= DMA_ATTR_DELAYED_UNMAP;

	/*
	 * dma_buf_map_attachment will call dma_map_sg for ion buffer
	 * mapping, and iova will get mapped when the function returns.
	 */
	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		DRM_ERROR("dma_buf_map_attachment failure, err=%d\n",
				ret);
		goto fail_import;
	}
	msm_obj->sgt = sgt;
	msm_obj->pages = NULL;

fail_import:
	return ret;
}

struct drm_gem_object *msm_gem_import(struct drm_device *dev,
		struct dma_buf *dmabuf, struct sg_table *sgt)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct msm_drm_private *priv = dev->dev_private;
#endif
	struct msm_gem_object *msm_obj;
	struct drm_gem_object *obj = NULL;
	uint32_t size;
	int ret;

	size = PAGE_ALIGN(dmabuf->size);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	ret = msm_gem_new_impl(dev, size, MSM_BO_WC, dmabuf->resv, &obj);
#else
	ret = msm_gem_new_impl(dev, size, MSM_BO_WC, dmabuf->resv, &obj, false);
#endif
	if (ret)
		goto fail;

	drm_gem_private_object_init(dev, obj, size);

	msm_obj = to_msm_bo(obj);
	mutex_lock(&msm_obj->lock);
	msm_obj->sgt = sgt;
	msm_obj->pages = NULL;
	/*
	 * 1) If sg table is NULL, user should call msm_gem_delayed_import
	 * to add back the sg table to the drm gem object.
	 *
	 * 2) Add buffer flag unconditionally for all import cases.
	 *    # Cached buffer will be attached immediately hence sgt will
	 *      be available upon gem obj creation.
	 *    # Un-cached buffer will follow delayed attach hence sgt
	 *      will be NULL upon gem obj creation.
	 */
	msm_obj->flags |= MSM_BO_EXTBUF;

	mutex_unlock(&msm_obj->lock);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	mutex_lock(&priv->mm_lock);
	list_add_tail(&msm_obj->mm_list, &priv->inactive_list);
	mutex_unlock(&priv->mm_lock);
#endif

	return obj;

fail:
	drm_gem_object_put(obj);
	return ERR_PTR(ret);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
static void *_msm_gem_kernel_new(struct drm_device *dev, uint32_t size,
		uint32_t flags, struct msm_gem_address_space *aspace,
		struct drm_gem_object **bo, uint64_t *iova, bool locked)
{
	void *vaddr;
	struct drm_gem_object *obj = _msm_gem_new(dev, size, flags, locked);
	int ret;

	if (IS_ERR(obj))
		return ERR_CAST(obj);

	if (iova) {
		ret = msm_gem_get_iova(obj, aspace, iova);
		if (ret)
			goto err;
	}

	vaddr = msm_gem_get_vaddr(obj);
	if (IS_ERR(vaddr)) {
		msm_gem_put_iova(obj, aspace);
		ret = PTR_ERR(vaddr);
		goto err;
	}

	if (bo)
		*bo = obj;

	return vaddr;
err:
	if (locked)
		drm_gem_object_put_locked(obj);
	else
		drm_gem_object_put(obj);

	return ERR_PTR(ret);

}

void *msm_gem_kernel_new(struct drm_device *dev, uint32_t size,
		uint32_t flags, struct msm_gem_address_space *aspace,
		struct drm_gem_object **bo, uint64_t *iova)
{
	return _msm_gem_kernel_new(dev, size, flags, aspace, bo, iova, false);
}

void *msm_gem_kernel_new_locked(struct drm_device *dev, uint32_t size,
		uint32_t flags, struct msm_gem_address_space *aspace,
		struct drm_gem_object **bo, uint64_t *iova)
{
	return _msm_gem_kernel_new(dev, size, flags, aspace, bo, iova, true);
}

void msm_gem_kernel_put(struct drm_gem_object *bo,
		struct msm_gem_address_space *aspace, bool locked)
{
	if (IS_ERR_OR_NULL(bo))
		return;

	msm_gem_put_vaddr(bo);
	msm_gem_unpin_iova(bo, aspace);

	if (locked)
		drm_gem_object_put_locked(bo);
	else
		drm_gem_object_put(bo);
}
#endif

void msm_gem_object_set_name(struct drm_gem_object *bo, const char *fmt, ...)
{
	struct msm_gem_object *msm_obj = to_msm_bo(bo);
	va_list ap;

	if (!fmt)
		return;

	va_start(ap, fmt);
	vsnprintf(msm_obj->name, sizeof(msm_obj->name), fmt, ap);
	va_end(ap);
}

void msm_gem_put_buffer(struct drm_gem_object *gem)
{
	struct msm_gem_object *msm_gem;

	if (!gem)
		return;

	msm_gem = to_msm_bo(gem);

	msm_gem_put_iova(gem, msm_gem->aspace);
	msm_gem_put_vaddr(gem);
}

int msm_gem_get_buffer(struct drm_gem_object *gem,
		struct drm_device *dev, struct drm_framebuffer *fb,
		uint32_t align_size)
{
	struct msm_gem_object *msm_gem;
	uint32_t size;
	uint64_t iova_aligned;
	int ret = -EINVAL;

	if (!gem) {
		DRM_ERROR("invalid drm gem");
		return ret;
	}

	msm_gem = to_msm_bo(gem);

	size = PAGE_ALIGN(gem->size);
	if (size < (align_size + GUARD_BYTES)) {
		DRM_ERROR("invalid gem size");
		goto exit;
	}

	msm_gem_smmu_address_space_get(dev, MSM_SMMU_DOMAIN_UNSECURE);

	if (PTR_ERR(msm_gem->aspace) == -ENODEV) {
		DRM_DEBUG("IOMMU not present, relying on VRAM.");
	} else if (IS_ERR_OR_NULL(msm_gem->aspace)) {
		ret = PTR_ERR(msm_gem->aspace);
		DRM_ERROR("failed to get aspace");
		goto exit;
	}

	ret = msm_gem_get_iova(gem, msm_gem->aspace, &msm_gem->iova);
	if (ret) {
		DRM_ERROR("failed to get the iova ret %d", ret);
		goto exit;
	}

	msm_gem_get_vaddr(gem);
	if (IS_ERR_OR_NULL(msm_gem->vaddr)) {
		DRM_ERROR("failed to get vaddr");
		goto exit;
	}

	iova_aligned = (msm_gem->iova + GUARD_BYTES) & ALIGNED_OFFSET;
	msm_gem->offset = iova_aligned - msm_gem->iova;
	msm_gem->iova = msm_gem->iova + msm_gem->offset;

	return 0;

exit:
	msm_gem_put_buffer(gem);

	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
MODULE_IMPORT_NS(DMA_BUF);
#endif
