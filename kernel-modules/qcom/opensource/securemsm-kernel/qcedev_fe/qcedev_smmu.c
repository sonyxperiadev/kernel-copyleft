// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/dma-mapping.h>
#include <linux/qcom-dma-mapping.h>
#include <linux/list.h>
#include <linux/mem-buf.h>
#include <linux/slab.h>
#include "qcedev_smmu.h"
#include "soc/qcom/secure_buffer.h"

static int find_free_hab_handle(struct qce_fe_drv_hab_handles *drv_handles)
{
	int handle_id = -1;
	int i = 0;

	if (drv_handles == NULL) {
		pr_err("%s : invalid handle\n", __func__);
		return handle_id;
	}
	spin_lock(&(drv_handles->handle_lock));
		for (i = 0; i < HAB_HANDLE_NUM; i++) {
			if (!drv_handles->qce_fe_hab_handles[i].in_use) {
				drv_handles->qce_fe_hab_handles[i].in_use = true;
				handle_id = i;
				spin_unlock(&(drv_handles->handle_lock));
				return handle_id;
			}
		}
	spin_unlock(&(drv_handles->handle_lock));
	pr_err("%s : there is no available hab handle\n", __func__);
	return handle_id;
}

static void free_hab_handle(struct qce_fe_drv_hab_handles *drv_handles, int handle_id)
{
	spin_lock(&(drv_handles->handle_lock));
	drv_handles->qce_fe_hab_handles[handle_id].in_use = false;
	spin_unlock(&(drv_handles->handle_lock));
}

static int msm_gpce_ion_smmu_map(struct dma_buf *dma_buf,
		struct qcedev_fe_reg_buf_info *binfo, struct qce_fe_drv_hab_handles *drv_handles)
{
	int rc = -2;
	u32 export_id;
	u32 cmd_rsp_size;
	u32 is_secure;
	bool exported = false;
	struct msm_gpce_smmu_vm_map_cmd smmu_map_cmd;
	struct msm_gpce_smmu_vm_map_cmd_rsp cmd_rsp;
	unsigned long delay = jiffies + (HZ / 2);
	int handle_id = find_free_hab_handle(drv_handles);
	u32 msm_gpce_ion_hab_handle = 0;
	size_t len = 0;

	if (handle_id < 0) {
		pr_err("%s : there is no available hab handle\n", __func__);
		return -ENODEV;
	}
	if (IS_ERR_OR_NULL(dma_buf)) {
		pr_err("%s : Failed to get dma_buf\n", __func__);
		dma_buf = NULL;
		goto err;
	}

	msm_gpce_ion_hab_handle = (drv_handles->qce_fe_hab_handles[handle_id]).handle;
	len = dma_buf->size;
	//Check if Buffer is secure or not
	is_secure = !(mem_buf_dma_buf_exclusive_owner(dma_buf));
	/* Export the buffer to physical VM */
	rc = habmm_export(msm_gpce_ion_hab_handle, dma_buf, len,
		&export_id, HABMM_EXPIMP_FLAGS_DMABUF);
	if (rc) {
		pr_err("%s: habmm_export failed dma_buf = %pK, len = %zd, rc = %d\n",
			__func__, dma_buf, len, rc);
		goto err;
	}

	exported = true;
	smmu_map_cmd.cmd_id = MSM_GPCE_SMMU_VM_CMD_MAP;
	smmu_map_cmd.export_id = export_id;
	smmu_map_cmd.buf_size = len;
	smmu_map_cmd.mem_handle = NULL;
	smmu_map_cmd.is_secure = is_secure;

	rc = habmm_socket_send(msm_gpce_ion_hab_handle,
		(void *)&smmu_map_cmd, sizeof(smmu_map_cmd), 0);
	if (rc) {
		pr_err("%s: habmm_socket_send failed %d\n",
			__func__, rc);
		goto err;
	}

	do {
		cmd_rsp_size = sizeof(cmd_rsp);
		rc = habmm_socket_recv(msm_gpce_ion_hab_handle,
			(void *)&cmd_rsp,
			&cmd_rsp_size,
			0xFFFFFFFF,
			0);
	} while (time_before(jiffies, delay) && (rc == -EINTR) &&
			(cmd_rsp_size == 0));
	if (rc) {
		pr_err("%s: habmm_socket_recv failed %d\n",
			__func__, rc);
		goto err;
	}

	if (cmd_rsp_size != sizeof(cmd_rsp)) {
		pr_err("%s: invalid size for cmd rsp %u, expected %zu\n",
			__func__, cmd_rsp_size, sizeof(cmd_rsp));
		rc = -EIO;
		goto err;
	}

	if (cmd_rsp.status) {
		pr_err("%s: SMMU map command failed %d\n",
			__func__, cmd_rsp.status);
		rc = cmd_rsp.status;
		goto err;
	}
	binfo->ion_buf.export_id = export_id;
	binfo->ion_buf.smmu_device_address = cmd_rsp.addr;
	binfo->ion_buf.mapped_buf_size = len;
	binfo->ion_buf.is_secure = is_secure;
	binfo->ion_buf.dma_buf = dma_buf;

	free_hab_handle(drv_handles, handle_id);
	return 0;

err:
	if (exported)
		(void)habmm_unexport(msm_gpce_ion_hab_handle, export_id, 0);

	free_hab_handle(drv_handles, handle_id);
	return rc;
}

static int msm_gpce_ion_smmu_unmap(struct qcedev_fe_ion_buf_info *dma_buffer_info,
		struct qce_fe_drv_hab_handles *drv_handles)
{
	int rc;
	u32 cmd_rsp_size;
	struct msm_gpce_smmu_vm_unmap_cmd smmu_unmap_cmd;
	struct msm_gpce_smmu_vm_unmap_cmd_rsp cmd_rsp;
	unsigned long delay = jiffies + (HZ / 2);
	int handle_id = find_free_hab_handle(drv_handles);
	u32 msm_gpce_ion_hab_handle = 0;

	if (handle_id < 0) {
		pr_err("%s : there is no available hab handle\n", __func__);
		return -ENODEV;
	}
	msm_gpce_ion_hab_handle = drv_handles->qce_fe_hab_handles[handle_id].handle;

	smmu_unmap_cmd.cmd_id = MSM_GPCE_SMMU_VM_CMD_UNMAP;
	smmu_unmap_cmd.export_id = dma_buffer_info->export_id;
	smmu_unmap_cmd.is_secure = dma_buffer_info->is_secure;
	pr_debug("%s: export_id %u\n", __func__, smmu_unmap_cmd.export_id);

	rc = habmm_socket_send(msm_gpce_ion_hab_handle,
		(void *)&smmu_unmap_cmd,
		sizeof(smmu_unmap_cmd), 0);
	if (rc) {
		pr_err("%s: habmm_socket_send failed %d\n",
			__func__, rc);
		goto err;
	}

	do {
		cmd_rsp_size = sizeof(cmd_rsp);
		rc = habmm_socket_recv(msm_gpce_ion_hab_handle,
			(void *)&cmd_rsp,
			&cmd_rsp_size,
			0xFFFFFFFF,
			0);
	} while (time_before(jiffies, delay) &&
			(rc == -EINTR) && (cmd_rsp_size == 0));
	if (rc) {
		pr_err("%s: habmm_socket_recv failed %d\n",
			__func__, rc);
		goto err;
	}

	if (cmd_rsp_size != sizeof(cmd_rsp)) {
		pr_err("%s: invalid size for cmd rsp %u\n",
			__func__, cmd_rsp_size);
		rc = -EIO;
		goto err;
	}

	if (cmd_rsp.status) {
		pr_err("%s: SMMU unmap command failed %d\n", __func__, cmd_rsp.status);
		rc = cmd_rsp.status;
		goto err;
	}

	rc = habmm_unexport(msm_gpce_ion_hab_handle,
		dma_buffer_info->export_id, 0xFFFFFFFF);
	if (rc) {
		pr_err("%s: habmm_unexport failed export_id = %d, rc = %d\n",
			__func__, dma_buffer_info->export_id, rc);
	}
	free_hab_handle(drv_handles, handle_id);
	pr_err("%s: SMMU unmap command status %d\n", __func__, rc);
	return rc;

err:
	free_hab_handle(drv_handles, handle_id);
	return rc;
}

int qcedev_check_and_map_buffer(void *handle,
		int fd, unsigned int offset, unsigned int fd_size,
		unsigned long long *vaddr, struct qce_fe_drv_hab_handles *drv_handles)
{
	struct qcedev_fe_reg_buf_info *binfo = NULL, *temp = NULL;
	bool found = false;
	struct qcedev_fe_handle *qce_hndl = (struct qcedev_fe_handle *)handle;
	int rc = 0;
	struct dma_buf *buf = NULL;
	unsigned long mapped_size = 0;

	if (!handle || !vaddr || fd < 0 || offset >= fd_size) {
		pr_err("%s: err: invalid input arguments\n", __func__);
		return -EINVAL;
	}
	/* Check if the buffer fd is already mapped */
	mutex_lock(&qce_hndl->registeredbufs.lock);
	list_for_each_entry(temp, &qce_hndl->registeredbufs.list, list) {
		if (temp->ion_buf.buf_ion_fd == fd) {
			found = true;
			*vaddr = temp->ion_buf.smmu_device_address;
			mapped_size = temp->ion_buf.mapped_buf_size;
			atomic_inc(&temp->ref_count);
			break;
		}
	}
	mutex_unlock(&qce_hndl->registeredbufs.lock);

	/* If buffer fd is not mapped then create a fresh mapping */
	if (!found) {
		pr_debug("%s: info: ion fd not registered with driver\n",
			__func__);
		binfo = kzalloc(sizeof(*binfo), GFP_KERNEL);
		if (!binfo) {
			pr_err("%s: err: failed to allocate binfo\n",
				__func__);
			rc = -ENOMEM;
			goto error;
		}
		binfo->ion_buf.buf_ion_fd = fd;
		buf = dma_buf_get(fd);
		if (IS_ERR_OR_NULL(buf)) {
			rc = -EINVAL;
			goto error;
		}
		rc = msm_gpce_ion_smmu_map(buf, binfo, drv_handles);
		if (rc) {
			pr_err("%s: err: failed to map fd (%d) error = %d\n",
				__func__, fd, rc);
			goto error;
		}
		*vaddr = binfo->ion_buf.smmu_device_address;
		mapped_size = binfo->ion_buf.mapped_buf_size;
		atomic_inc(&binfo->ref_count);
	}
	/* Make sure the offset is within the mapped range */
	if (offset >= mapped_size) {
		pr_err(
			"%s: err: Offset (%u) exceeds mapped size(%lu) for fd: %d\n",
			__func__, offset, mapped_size, fd);
		rc = -ERANGE;
		goto unmap;
	}

	if (!found) {
		/* Add buffer mapping information to regd buffer list */
		mutex_lock(&qce_hndl->registeredbufs.lock);
		list_add_tail(&binfo->list, &qce_hndl->registeredbufs.list);
		mutex_unlock(&qce_hndl->registeredbufs.lock);
	}

	/* return the mapped virtual address adjusted by offset */
	*vaddr += offset;

	return 0;
unmap:
	if (!found) {
		msm_gpce_ion_smmu_unmap(&(binfo->ion_buf), drv_handles);
	}
error:
	kfree(binfo);
	return rc;
}

int qcedev_check_and_unmap_buffer(void *handle, int fd, struct qce_fe_drv_hab_handles *drv_handles)
{
	struct qcedev_fe_reg_buf_info *binfo = NULL, *dummy = NULL;
	struct qcedev_fe_handle *qce_hndl = handle;
	bool found = false;

	if (!handle || fd < 0) {
		pr_err("%s: err: invalid input arguments\n", __func__);
		return -EINVAL;
	}

	/* Check if the buffer fd is mapped and present in the regd list. */
	mutex_lock(&qce_hndl->registeredbufs.lock);
	list_for_each_entry_safe(binfo, dummy,
		&qce_hndl->registeredbufs.list, list) {
		if (binfo->ion_buf.buf_ion_fd == fd) {
			found = true;
			atomic_dec(&binfo->ref_count);

			/* Unmap only if there are no more references */
			if (atomic_read(&binfo->ref_count) == 0) {
				msm_gpce_ion_smmu_unmap(&(binfo->ion_buf), drv_handles);
				dma_buf_put(binfo->ion_buf.dma_buf);
				list_del(&binfo->list);
				kfree(binfo);
			}
			break;
		}
	}
	mutex_unlock(&qce_hndl->registeredbufs.lock);

	if (!found) {
		pr_err("%s: err: calling unmap on unknown fd %d\n",
			__func__, fd);
		return -EINVAL;
	}

	return 0;
}

int qcedev_unmap_all_buffers(void *handle, struct qce_fe_drv_hab_handles *drv_handles)
{
	struct qcedev_fe_reg_buf_info *binfo = NULL;
	struct qcedev_fe_handle *qce_hndl = handle;
	struct list_head *pos;

	if (!handle) {
		pr_err("%s: err: invalid input arguments\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&qce_hndl->registeredbufs.lock);
	while (!list_empty(&qce_hndl->registeredbufs.list)) {
		pos = qce_hndl->registeredbufs.list.next;
		binfo = list_entry(pos, struct qcedev_fe_reg_buf_info, list);
		if (binfo)
			msm_gpce_ion_smmu_unmap(&(binfo->ion_buf), drv_handles);
		list_del(pos);
		kfree(binfo);
	}
	mutex_unlock(&qce_hndl->registeredbufs.lock);
	pr_info("%s: Done successfully\n", __func__);
	return 0;
}

