// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kthread.h>

#include <linux/qcom-iommu-util.h>
#include <dt-bindings/arm/msm/qti-smmu-proxy-dt-ids.h>
#include "qti-smmu-proxy-common.h"

#define RECEIVER_COMPAT_STR "smmu-proxy-receiver"
#define CB_COMPAT_STR "smmu-proxy-cb"

static void *msgq_hdl;

struct smmu_proxy_buffer_cb_info {
	bool mapped;
	struct dma_buf_attachment *attachment;
	struct sg_table *sg_table;
};

struct smmu_proxy_buffer_state {
	bool locked;
	struct smmu_proxy_buffer_cb_info cb_info[QTI_SMMU_PROXY_CB_IDS_LEN];
	struct dma_buf *dmabuf;
};

static DEFINE_MUTEX(buffer_state_lock);
static DEFINE_XARRAY(buffer_state_arr);

static unsigned int cb_map_counts[QTI_SMMU_PROXY_CB_IDS_LEN] = { 0 };
struct device *cb_devices[QTI_SMMU_PROXY_CB_IDS_LEN] = { 0 };

struct task_struct *receiver_msgq_handler_thread;

static int zero_dma_buf(struct dma_buf *dmabuf)
{
	int ret;
	struct iosys_map vmap_struct = {0};

	ret = dma_buf_vmap(dmabuf, &vmap_struct);
	if (ret) {
		pr_err("%s: dma_buf_vmap() failed with %d\n", __func__, ret);
		return ret;
	}

	/* Use DMA_TO_DEVICE since we are not reading anything */
	ret = dma_buf_begin_cpu_access(dmabuf, DMA_TO_DEVICE);
	if (ret) {
		pr_err("%s: dma_buf_begin_cpu_access() failed with %d\n", __func__, ret);
		goto unmap;
	}

	memset(vmap_struct.vaddr, 0, dmabuf->size);
	ret = dma_buf_end_cpu_access(dmabuf, DMA_TO_DEVICE);
	if (ret)
		pr_err("%s: dma_buf_end_cpu_access() failed with %d\n", __func__, ret);
unmap:
	dma_buf_vunmap(dmabuf, &vmap_struct);

	if (ret)
		pr_err("%s: Failed to properly zero the DMA-BUF\n", __func__);

	return ret;
}

static int iommu_unmap_and_relinquish(u32 hdl)
{
	int cb_id, ret = 0;
	struct smmu_proxy_buffer_state *buf_state;

	mutex_lock(&buffer_state_lock);
	buf_state = xa_load(&buffer_state_arr, hdl);
	if (!buf_state) {
		pr_err("%s: handle 0x%x unknown to proxy driver!\n", __func__, hdl);
		ret = -EINVAL;
		goto out;
	}

	if (buf_state->locked) {
		pr_err("%s: handle 0x%x is locked!\n", __func__, hdl);
		ret = -EINVAL;
		goto out;
	}

	for (cb_id = 0; cb_id < QTI_SMMU_PROXY_CB_IDS_LEN; cb_id++) {
		if (buf_state->cb_info[cb_id].mapped) {
			dma_buf_unmap_attachment(buf_state->cb_info[cb_id].attachment,
						 buf_state->cb_info[cb_id].sg_table,
						 DMA_BIDIRECTIONAL);
			dma_buf_detach(buf_state->dmabuf,
				       buf_state->cb_info[cb_id].attachment);
			buf_state->cb_info[cb_id].mapped = false;

			/* If nothing left is mapped for this CB, unprogram its SMR */
			cb_map_counts[cb_id]--;
			if (!cb_map_counts[cb_id]) {
				ret = qcom_iommu_sid_switch(cb_devices[cb_id], SID_RELEASE);
				if (ret) {
					pr_err("%s: Failed to unprogram SMR for cb_id %d rc: %d\n",
					       __func__, cb_id, ret);
					break;
				}
			}
		}
	}

	ret = zero_dma_buf(buf_state->dmabuf);
	if (!ret) {
		dma_buf_put(buf_state->dmabuf);
		flush_delayed_fput();
	}

	xa_erase(&buffer_state_arr, hdl);
	kfree(buf_state);
out:
	mutex_unlock(&buffer_state_lock);

	return ret;
}

static int process_unmap_request(struct smmu_proxy_unmap_req *req, size_t size)
{
	struct smmu_proxy_unmap_resp *resp;
	int ret = 0;

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		pr_err("%s: Failed to allocate memory for response\n", __func__);
		return -ENOMEM;
	}

	ret = iommu_unmap_and_relinquish(req->hdl);

	resp->hdr.msg_type = SMMU_PROXY_UNMAP_RESP;
	resp->hdr.msg_size = sizeof(*resp);
	resp->hdr.ret = ret;

	ret = gh_msgq_send(msgq_hdl, resp, resp->hdr.msg_size, 0);
	if (ret < 0)
		pr_err("%s: failed to send response to mapping request rc: %d\n", __func__, ret);
	else
		pr_debug("%s: response to mapping request sent\n", __func__);

	kfree(resp);

	return ret;
}

static
inline
struct sg_table *retrieve_and_iommu_map(struct mem_buf_retrieve_kernel_arg *retrieve_arg,
					u32 cb_id)
{
	int ret;
	struct dma_buf *dmabuf;
	bool new_buf = false;
	struct smmu_proxy_buffer_state *buf_state;
	struct dma_buf_attachment *attachment;
	struct sg_table *table;

	if (cb_id >= QTI_SMMU_PROXY_CB_IDS_LEN) {
		pr_err("%s: CB ID %d too large\n", __func__, cb_id);
		return ERR_PTR(-EINVAL);
	}

	if (!cb_devices[cb_id]) {
		pr_err("%s: CB of ID %d not defined\n", __func__, cb_id);
		return ERR_PTR(-EINVAL);
	}

	mutex_lock(&buffer_state_lock);
	buf_state = xa_load(&buffer_state_arr, retrieve_arg->memparcel_hdl);
	if (buf_state) {
		if (buf_state->cb_info[cb_id].mapped) {
			table = buf_state->cb_info[cb_id].sg_table;
			goto unlock;
		}
		if (buf_state->locked) {
			pr_err("%s: handle 0x%x is locked!\n", __func__,
			       retrieve_arg->memparcel_hdl);
			ret = -EINVAL;
			goto unlock_err;
		}

		dmabuf = buf_state->dmabuf;
	} else {
		new_buf = true;
		dmabuf = mem_buf_retrieve(retrieve_arg);
		if (IS_ERR(dmabuf)) {
			ret = PTR_ERR(dmabuf);
			pr_err("%s: Failed to retrieve DMA-BUF rc: %d\n", __func__, ret);
			goto unlock_err;
		}

		ret = zero_dma_buf(dmabuf);
		if (ret) {
			pr_err("%s: Failed to zero the DMA-BUF rc: %d\n", __func__, ret);
			goto free_buf;
		}

		buf_state = kzalloc(sizeof(*buf_state), GFP_KERNEL);
		if (!buf_state) {
			pr_err("%s: Unable to allocate memory for buf_state\n",
			       __func__);
			ret = -ENOMEM;
			goto free_buf;
		}

		buf_state->dmabuf = dmabuf;
	}

	attachment = dma_buf_attach(dmabuf, cb_devices[cb_id]);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		pr_err("%s: Failed to attach rc: %d\n", __func__, ret);
		goto free_buf_state;
	}

	table = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(table)) {
		ret = PTR_ERR(table);
		pr_err("%s: Failed to map rc: %d\n", __func__, ret);
		goto detach;
	}

	if (table->nents != 1) {
		ret = -EINVAL;
		pr_err("%s: Buffer not mapped as one segment!\n", __func__);
		goto unmap;
	}

	buf_state->cb_info[cb_id].mapped = true;
	buf_state->cb_info[cb_id].attachment = attachment;
	buf_state->cb_info[cb_id].sg_table = table;

	if (!cb_map_counts[cb_id]) {
		ret = qcom_iommu_sid_switch(cb_devices[cb_id], SID_ACQUIRE);
		if (ret) {
			pr_err("%s: Failed to program SMRs for cb_id %d rc: %d\n", __func__,
			       cb_id, ret);
			goto unmap;
		}
	}
	cb_map_counts[cb_id]++;

	ret = xa_err(xa_store(&buffer_state_arr, retrieve_arg->memparcel_hdl, buf_state,
		     GFP_KERNEL));
	if (ret < 0) {
		pr_err("%s: Failed to store new buffer in xarray rc: %d\n", __func__,
		       ret);
		goto dec_cb_map_count;
	}

unlock:
	mutex_unlock(&buffer_state_lock);

	return table;

dec_cb_map_count:
	cb_map_counts[cb_id]--;
	if (!cb_map_counts[cb_id]) {
		ret = qcom_iommu_sid_switch(cb_devices[cb_id], SID_RELEASE);
		if (ret)
			pr_err("%s: Failed to unprogram SMR for cb_id %d rc: %d\n",
			       __func__, cb_id, ret);
	}
unmap:
	dma_buf_unmap_attachment(attachment, table, DMA_BIDIRECTIONAL);
detach:
	dma_buf_detach(dmabuf, attachment);
free_buf_state:
	if (new_buf)
		kfree(buf_state);
free_buf:
	if (new_buf)
		dma_buf_put(dmabuf);
unlock_err:
	mutex_unlock(&buffer_state_lock);

	return ERR_PTR(ret);
}

static int process_map_request(struct smmu_proxy_map_req *req, size_t size)
{
	struct smmu_proxy_map_resp *resp;
	int ret = 0;
	u32 n_acl_entries = req->acl_desc.n_acl_entries;
	size_t map_req_len = offsetof(struct smmu_proxy_map_req,
				      acl_desc.acl_entries[n_acl_entries]);
	struct mem_buf_retrieve_kernel_arg retrieve_arg = {0};
	int i;
	struct sg_table *table;

	/*
	 * Last entry of smmu_proxy_map_req is an array of arbitrary length.
	 * Validate that the number of entries fits within the buffer given
	 * to us by the message queue.
	 */
	if (map_req_len > size) {
		pr_err("%s: Reported size of smmu_proxy_map_request (%ld bytes) greater than message size given by message queue (%ld bytes)\n",
		       __func__, map_req_len, size);
		return -EINVAL;
	}

	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		pr_err("%s: Failed to allocate memory for response\n", __func__);
		return -ENOMEM;
	}

	retrieve_arg.vmids = kmalloc_array(n_acl_entries, sizeof(*retrieve_arg.vmids), GFP_KERNEL);
	if (!retrieve_arg.vmids) {
		ret = -ENOMEM;
		goto free_resp;
	}

	retrieve_arg.perms = kmalloc_array(n_acl_entries, sizeof(*retrieve_arg.perms), GFP_KERNEL);
	if (!retrieve_arg.perms) {
		ret = -ENOMEM;
		goto free_vmids;
	}

	retrieve_arg.fd_flags = O_RDWR;
	retrieve_arg.memparcel_hdl = req->hdl;
	retrieve_arg.sender_vmid = VMID_HLOS;
	retrieve_arg.nr_acl_entries = n_acl_entries;

	for (i = 0; i < n_acl_entries; i++) {
		retrieve_arg.vmids[i] = req->acl_desc.acl_entries[i].vmid;
		retrieve_arg.perms[i] = req->acl_desc.acl_entries[i].perms;
	}

	table = retrieve_and_iommu_map(&retrieve_arg, req->cb_id);
	if (IS_ERR(table)) {
		ret = PTR_ERR(table);
		goto free_perms;
	}

	resp->hdr.msg_type = SMMU_PROXY_MAP_RESP;
	resp->hdr.msg_size = sizeof(*resp);
	resp->hdr.ret = ret;
	resp->iova = sg_dma_address(table->sgl);
	resp->mapping_len = sg_dma_len(table->sgl);

	ret = gh_msgq_send(msgq_hdl, resp, resp->hdr.msg_size, 0);
	if (ret < 0) {
		pr_err("%s: failed to send response to mapping request rc: %d\n", __func__, ret);
		iommu_unmap_and_relinquish(req->hdl);
	} else {
		pr_debug("%s: response to mapping request sent\n", __func__);
	}

free_perms:
	kfree(retrieve_arg.perms);
free_vmids:
	kfree(retrieve_arg.vmids);
free_resp:
	kfree(resp);

	return ret;
}

static void smmu_proxy_process_msg(void *buf, size_t size)
{
	struct smmu_proxy_msg_hdr *msg_hdr = buf;
	struct smmu_proxy_resp_hdr *resp;
	int ret = -EINVAL;

	if (size < sizeof(*msg_hdr) || msg_hdr->msg_size != size) {
		pr_err("%s: message received is not of a proper size: 0x%lx, 0x:%x\n",
		       __func__, size, msg_hdr->msg_size);
		goto handle_err;
	}

	switch (msg_hdr->msg_type) {
	case SMMU_PROXY_MAP:
		ret = process_map_request(buf, size);
		break;
	case SMMU_PROXY_UNMAP:
		ret = process_unmap_request(buf, size);
		break;
	default:
		pr_err("%s: received message of unknown type: %d\n", __func__,
		       msg_hdr->msg_type);
	}

	if (!ret)
		return;

handle_err:
	resp = kzalloc(sizeof(resp), GFP_KERNEL);
	if (!resp) {
		pr_err("%s: Failed to allocate memory for response\n", __func__);
		return;
	}

	resp->msg_type = SMMU_PROXY_ERR_RESP;
	resp->msg_size = sizeof(resp);
	resp->ret = ret;

	ret = gh_msgq_send(msgq_hdl, resp, resp->msg_size, 0);
	if (ret < 0)
		pr_err("%s: failed to send error response rc: %d\n", __func__, ret);
	else
		pr_debug("%s: response to mapping request sent\n", __func__);

	kfree(resp);

}

static int receiver_msgq_handler(void *msgq_hdl)
{
	void *buf;
	size_t size;
	int ret;

	buf = kzalloc(GH_MSGQ_MAX_MSG_SIZE_BYTES, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	while (!kthread_should_stop()) {
		ret = gh_msgq_recv(msgq_hdl, buf, GH_MSGQ_MAX_MSG_SIZE_BYTES, &size, 0);
		if (ret < 0) {
			pr_err_ratelimited("%s failed to receive message rc: %d\n", __func__, ret);
		} else {
			smmu_proxy_process_msg(buf, size);
		}
	}

	kfree(buf);

	return 0;
}

static int smmu_proxy_ac_lock_toggle(int dma_buf_fd, bool lock)
{
	int ret = 0;
	struct smmu_proxy_buffer_state *buf_state;
	struct dma_buf *dmabuf;
	u32 handle;

	dmabuf = dma_buf_get(dma_buf_fd);
	if (IS_ERR(dmabuf)) {
		pr_err("%s: unable to get dma-buf from FD %d, rc: %ld\n", __func__,
		       dma_buf_fd, PTR_ERR(dmabuf));
		return PTR_ERR(dmabuf);
	}

	ret = mem_buf_dma_buf_get_memparcel_hdl(dmabuf, &handle);
	if (ret) {
		pr_err("%s: Failed to get memparcel handle rc: %d\n", __func__, ret);
		goto free_buf;
	}

	mutex_lock(&buffer_state_lock);
	buf_state = xa_load(&buffer_state_arr, handle);
	if (!buf_state) {
		pr_err("%s: handle 0x%x unknown to proxy driver!\n", __func__, handle);
		ret = -EINVAL;
		goto out;
	}

	if (buf_state->locked == lock) {
		pr_err("%s: handle 0x%x already %s!\n", __func__, handle,
		       lock ? "locked" : "unlocked");
		ret = -EINVAL;
		goto out;
	}

	buf_state->locked = lock;
out:
	mutex_unlock(&buffer_state_lock);
free_buf:
	dma_buf_put(dmabuf);

	return ret;
}

/*
 * Iterate over all buffers mapped to context bank @context_bank_id, and zero
 * out the buffers. If there is a single error for any buffer, we bail out with
 * an error and disregard the rest of the buffers mapped to @context_bank_id.
 */
int smmu_proxy_clear_all_buffers(void __user *context_bank_id_array,
				 __u32 num_cb_ids)
{
	unsigned long handle;
	struct smmu_proxy_buffer_state *buf_state;
	__u32 cb_ids[QTI_SMMU_PROXY_CB_IDS_LEN];
	int i, ret = 0;
	bool found_mapped_cb;

	/* Checking this allows us to keep cb_id_arr fixed in length */
	if (num_cb_ids > QTI_SMMU_PROXY_CB_IDS_LEN) {
		pr_err("%s: Invalid number of CB IDs: %u\n", __func__, num_cb_ids);
		return -EINVAL;
	}

	ret = copy_struct_from_user(&cb_ids, sizeof(cb_ids), context_bank_id_array,
				    sizeof(cb_ids));
	if (ret) {
		pr_err("%s: Failed to get CB IDs from user space rc %d\n", __func__, ret);
		return ret;
	}

	for (i = 0; i < num_cb_ids; i++) {
		if (cb_ids[i] >= QTI_SMMU_PROXY_CB_IDS_LEN) {
			pr_err("%s: Invalid CB ID of %u at pos %d\n", __func__, cb_ids[i], i);
			return -EINVAL;
		}
	}

	mutex_lock(&buffer_state_lock);
	xa_for_each(&buffer_state_arr, handle, buf_state) {
		found_mapped_cb = false;
		for (i = 0; i < num_cb_ids; i++) {
			if (buf_state->cb_info[cb_ids[i]].mapped) {
				found_mapped_cb = true;
				break;
			}
		}
		if (!found_mapped_cb)
			continue;

		ret = zero_dma_buf(buf_state->dmabuf);
		if (ret) {
			pr_err("%s: dma_buf_vmap() failed with %d\n", __func__, ret);
			break;
		}
	}

	mutex_unlock(&buffer_state_lock);
	return ret;
}

static int smmu_proxy_get_dma_buf(struct smmu_proxy_get_dma_buf_ctl *get_dma_buf_ctl)
{
	struct smmu_proxy_buffer_state *buf_state;
	int fd, ret = 0;

	mutex_lock(&buffer_state_lock);
	buf_state = xa_load(&buffer_state_arr, get_dma_buf_ctl->memparcel_hdl);
	if (!buf_state) {
		pr_err("%s: handle 0x%llx unknown to proxy driver!\n", __func__,
		       get_dma_buf_ctl->memparcel_hdl);
		ret = -EINVAL;
		goto out;
	}

	get_dma_buf(buf_state->dmabuf);
	fd = dma_buf_fd(buf_state->dmabuf, O_CLOEXEC);
	if (fd < 0) {
		ret = fd;
		pr_err("%s: Failed to install FD for dma-buf rc: %d\n", __func__,
		       ret);
		dma_buf_put(buf_state->dmabuf);
	} else {
		get_dma_buf_ctl->dma_buf_fd = fd;
	}
out:
	mutex_unlock(&buffer_state_lock);

	return ret;
}

static long smmu_proxy_dev_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	unsigned int dir = _IOC_DIR(cmd);
	union smmu_proxy_ioctl_arg ioctl_arg;
	int ret;

	if (_IOC_SIZE(cmd) > sizeof(ioctl_arg))
		return -EINVAL;

	if (copy_from_user(&ioctl_arg, (void __user *)arg, _IOC_SIZE(cmd)))
		return -EFAULT;

	if (!(dir & _IOC_WRITE))
		memset(&ioctl_arg, 0, sizeof(ioctl_arg));

	switch (cmd) {
	case QTI_SMMU_PROXY_AC_LOCK_BUFFER:
	{
		struct smmu_proxy_acl_ctl *acl_ctl =
			&ioctl_arg.acl_ctl;

		ret = smmu_proxy_ac_lock_toggle(acl_ctl->dma_buf_fd, true);
		if (ret)
			return ret;

		break;
	}
	case QTI_SMMU_PROXY_AC_UNLOCK_BUFFER:
	{
		struct smmu_proxy_acl_ctl *acl_ctl =
			&ioctl_arg.acl_ctl;

		ret = smmu_proxy_ac_lock_toggle(acl_ctl->dma_buf_fd, false);
		if (ret)
			return ret;

		break;
	}
	case QTI_SMMU_PROXY_WIPE_BUFFERS:
	{
		struct smmu_proxy_wipe_buf_ctl *wipe_buf_ctl =
			&ioctl_arg.wipe_buf_ctl;

		ret = smmu_proxy_clear_all_buffers((void *) wipe_buf_ctl->context_bank_id_array,
						   wipe_buf_ctl->num_cb_ids);
		break;
	}
	case QTI_SMMU_PROXY_GET_DMA_BUF:
	{
		ret = smmu_proxy_get_dma_buf(&ioctl_arg.get_dma_buf_ctl);
		break;
	}

	default:
		return -ENOTTY;
	}

	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &ioctl_arg,
				 _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	return 0;
}

static const struct file_operations smmu_proxy_dev_fops = {
	.unlocked_ioctl = smmu_proxy_dev_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static int receiver_probe_handler(struct device *dev)
{
	int ret = 0;

	msgq_hdl = gh_msgq_register(GH_MSGQ_LABEL_SMMU_PROXY);
	if (IS_ERR(msgq_hdl)) {
		ret = PTR_ERR(msgq_hdl);
		dev_err(dev, "Queue registration failed: %ld!\n", PTR_ERR(msgq_hdl));
		return ret;
	}

	receiver_msgq_handler_thread = kthread_run(receiver_msgq_handler, msgq_hdl,
						   "smmu_proxy_msgq_handler");
	if (IS_ERR(receiver_msgq_handler_thread)) {
		ret = PTR_ERR(receiver_msgq_handler_thread);
		dev_err(dev, "Failed to launch receiver_msgq_handler thread: %ld\n",
			PTR_ERR(receiver_msgq_handler_thread));
		goto free_msgq;
	}

	ret = smmu_proxy_create_dev(&smmu_proxy_dev_fops);
	if (ret) {
		pr_err("Failed to create character device with error %d\n", ret);
		goto free_kthread;
	}

	return 0;
free_kthread:
	kthread_stop(receiver_msgq_handler_thread);
free_msgq:
	gh_msgq_unregister(msgq_hdl);
	return ret;
}

static int proxy_fault_handler(struct iommu_domain *domain, struct device *dev,
			       unsigned long iova, int flags, void *token)
{
	dev_err(dev, "Context fault with IOVA %lx and fault flags %d\n", iova, flags);
	return -EINVAL;
}

static int cb_probe_handler(struct device *dev)
{
	int ret;
	unsigned int context_bank_id;
	struct iommu_domain *domain;

	ret = of_property_read_u32(dev->of_node, "qti,cb-id", &context_bank_id);
	if (ret) {
		dev_err(dev, "Failed to read qti,cb-id property for device\n");
		return -EINVAL;
	}

	if (context_bank_id >= QTI_SMMU_PROXY_CB_IDS_LEN) {
		dev_err(dev, "Invalid CB ID: %u\n", context_bank_id);
		return -EINVAL;
	}

	if (cb_devices[context_bank_id]) {
		dev_err(dev, "Context bank %u is already populated\n", context_bank_id);
		return -EINVAL;
	}

	ret = dma_set_max_seg_size(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "Failed to set segment size\n");
		return ret;
	}

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "Failed to set DMA-MASK\n");
		return ret;
	}

	domain = iommu_get_domain_for_dev(dev);
	if (IS_ERR_OR_NULL(domain)) {
		dev_err(dev, "%s: Failed to get iommu domain\n", __func__);
		return -EINVAL;
	}

	iommu_set_fault_handler(domain, proxy_fault_handler, NULL);
	cb_devices[context_bank_id] = dev;

	return 0;
}

static int smmu_proxy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (of_device_is_compatible(dev->of_node, CB_COMPAT_STR)) {
		return cb_probe_handler(dev);
	} else if (of_device_is_compatible(dev->of_node, RECEIVER_COMPAT_STR)) {
		return  receiver_probe_handler(dev);
	} else {
		return -EINVAL;
	}
}

static const struct of_device_id smmu_proxy_match_table[] = {
	{.compatible = RECEIVER_COMPAT_STR},
	{.compatible = CB_COMPAT_STR},
	{},
};

static struct platform_driver smmu_proxy_driver = {
	.probe = smmu_proxy_probe,
	.driver = {
		.name = "qti-smmu-proxy",
		.of_match_table = smmu_proxy_match_table,
	},
};

int __init init_smmu_proxy_driver(void)
{
	int ret;
	struct csf_version csf_version;

	ret = smmu_proxy_get_csf_version(&csf_version);
	if (ret) {
		pr_err("%s: Unable to get CSF version\n", __func__);
		return ret;
	}

	if (csf_version.arch_ver == 2 && csf_version.max_ver == 0) {
		pr_err("%s: CSF 2.5 not in use, not loading module\n", __func__);
		return -EINVAL;
	}

	return platform_driver_register(&smmu_proxy_driver);
}
module_init(init_smmu_proxy_driver);

MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL v2");
