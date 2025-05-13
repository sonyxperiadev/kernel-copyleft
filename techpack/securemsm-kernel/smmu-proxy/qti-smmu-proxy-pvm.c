// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "qti-smmu-proxy-common.h"

#include <linux/qti-smmu-proxy-callbacks.h>
#include <linux/qcom-dma-mapping.h>
#include <linux/of.h>

static void *msgq_hdl;

DEFINE_MUTEX(sender_mutex);

static const struct file_operations smmu_proxy_dev_fops;

int smmu_proxy_unmap(void *data)
{
	struct dma_buf *dmabuf;
	void *buf;
	size_t size;
	int ret;
	struct smmu_proxy_unmap_req *req;
	struct smmu_proxy_unmap_resp *resp;

	mutex_lock(&sender_mutex);
	buf = kzalloc(GH_MSGQ_MAX_MSG_SIZE_BYTES, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		pr_err("%s: Failed to allocate memory!\n", __func__);
		goto out;
	}

	req = buf;

	dmabuf = data;
	ret = mem_buf_dma_buf_get_memparcel_hdl(dmabuf, &req->hdl);
	if (ret) {
		pr_err("%s: Failed to get memparcel handle rc: %d\n", __func__, ret);
		goto free_buf;
	}

	req->hdr.msg_type = SMMU_PROXY_UNMAP;
	req->hdr.msg_size = sizeof(*req);

	ret = gh_msgq_send(msgq_hdl, (void *) req, req->hdr.msg_size, 0);
	if (ret < 0) {
		pr_err("%s: failed to send message rc: %d\n", __func__, ret);
		goto free_buf;
	}

	/*
	 * No need to validate size -  gh_msgq_recv() ensures that sizeof(*resp) <
	 * GH_MSGQ_MAX_MSG_SIZE_BYTES
	 */
	ret = gh_msgq_recv(msgq_hdl, buf, sizeof(*resp), &size, 0);
	if (ret < 0) {
		pr_err_ratelimited("%s: failed to receive message rc: %d\n", __func__, ret);
		goto free_buf;
	}

	resp = buf;
	if (resp->hdr.ret) {
		ret = resp->hdr.ret;
		pr_err("%s: Unmap call failed on remote VM, rc: %d\n", __func__,
		       resp->hdr.ret);
	}

free_buf:
	kfree(buf);
out:
	mutex_unlock(&sender_mutex);

	return ret;
}

int smmu_proxy_map(struct device *client_dev, struct sg_table *proxy_iova,
		   struct dma_buf *dmabuf)
{
	void *buf;
	size_t size;
	int ret = 0;
	int n_acl_entries, i;
	int vmids[2] = { VMID_TVM, VMID_OEMVM };
	int perms[2] = { PERM_READ | PERM_WRITE, PERM_READ | PERM_WRITE};
	struct csf_version csf_version;
	struct mem_buf_lend_kernel_arg arg = {0};
	struct smmu_proxy_map_req *req;
	struct smmu_proxy_map_resp *resp;

	ret = smmu_proxy_get_csf_version(&csf_version);
	if (ret) {
		return ret;
	}

	/*
	 * We enter this function iff the CSF version is 2.5.* . If CSF 2.5.1
	 * is in use, we set n_acl_entries to two, in order to assign this
	 * memory to the TVM and OEM VM. If CSF 2.5.0 is in use, we just assign
	 * it to the TVM.
	 */
	n_acl_entries = csf_version.min_ver == 1 ? 2 : 1;

	mutex_lock(&sender_mutex);
	buf = kzalloc(GH_MSGQ_MAX_MSG_SIZE_BYTES, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	if (mem_buf_dma_buf_exclusive_owner(dmabuf)) {
		arg.vmids = vmids;
		arg.perms = perms;
		arg.nr_acl_entries = n_acl_entries;

		ret = mem_buf_lend(dmabuf, &arg);
		if (ret) {
			pr_err("%s: Failed to lend buf rc: %d\n", __func__, ret);
			goto free_buf;
		}
	}

	/* Prepare the message */
	req = buf;
	req->acl_desc.n_acl_entries = n_acl_entries;
	for (i = 0; i < n_acl_entries; i++) {
		req->acl_desc.acl_entries[i].vmid = vmids[i];
		req->acl_desc.acl_entries[i].perms = perms[i];
	}

	ret = mem_buf_dma_buf_get_memparcel_hdl(dmabuf, &req->hdl);
	if (ret) {
		pr_err("%s: Failed to get memparcel handle rc: %d\n", __func__, ret);
		goto free_buf;
	}

	ret = of_property_read_u32(client_dev->of_node,
				   "qti,smmu-proxy-cb-id",
				   &req->cb_id);
	if (ret) {
		dev_err(client_dev, "%s: Err reading 'qti,smmu-proxy-cb-id' rc: %d\n",
			__func__, ret);
		goto free_buf;
	}

	req->hdr.msg_type = SMMU_PROXY_MAP;
	req->hdr.msg_size = offsetof(struct smmu_proxy_map_req,
				acl_desc.acl_entries[n_acl_entries]);

	ret = gh_msgq_send(msgq_hdl, (void *) req, req->hdr.msg_size, 0);
	if (ret < 0) {
		pr_err("%s: failed to send message rc: %d\n", __func__, ret);
		goto free_buf;
	}

	/*
	 * No need to validate size -  gh_msgq_recv() ensures that sizeof(*resp) <
	 * GH_MSGQ_MAX_MSG_SIZE_BYTES
	 */
	ret = gh_msgq_recv(msgq_hdl, buf, sizeof(*resp), &size, 0);
	if (ret < 0) {
		pr_err_ratelimited("%s: failed to receive message rc: %d\n", __func__, ret);
		goto free_buf;
	}

	resp = buf;

	if (resp->hdr.ret) {
		ret = resp->hdr.ret;
		pr_err_ratelimited("%s: Map call failed on remote VM, rc: %d\n", __func__,
				   resp->hdr.ret);
		goto free_buf;
	}

	ret = mem_buf_dma_buf_set_destructor(dmabuf, smmu_proxy_unmap, dmabuf);
	if (ret) {
		pr_err_ratelimited("%s: Failed to set vmperm destructor, rc: %d\n",
				   __func__, ret);
		goto free_buf;
	}

	sg_dma_address(proxy_iova->sgl) = resp->iova;
	sg_dma_len(proxy_iova->sgl) = resp->mapping_len;
	/*
	 * We set the number of entries to one here, as we only allow the mapping to go
	 * through on the TVM if the sg_table returned by dma_buf_map_attachment has one
	 * entry.
	 */
	proxy_iova->nents = 1;

free_buf:
	kfree(buf);
out:
	mutex_unlock(&sender_mutex);

	return ret;
}

void smmu_proxy_unmap_nop(struct device *client_dev, struct sg_table *table,
			  struct dma_buf *dmabuf)
{

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
	case QTI_SMMU_PROXY_GET_VERSION_IOCTL:
	{
		struct csf_version *csf_version =
			&ioctl_arg.csf_version;

		ret = smmu_proxy_get_csf_version(csf_version);
		if(ret)
			return ret;

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

static int sender_probe_handler(struct platform_device *pdev)
{
	int ret;
	struct csf_version csf_version;

	msgq_hdl = gh_msgq_register(GH_MSGQ_LABEL_SMMU_PROXY);
	if (IS_ERR(msgq_hdl)) {
		ret = PTR_ERR(msgq_hdl);
		pr_err("%s: Queue registration failed rc: %ld!\n", __func__, PTR_ERR(msgq_hdl));
		return ret;
	}

	ret = smmu_proxy_get_csf_version(&csf_version);
	if (ret) {
		pr_err("%s: Failed to get CSF version rc: %d\n", __func__, ret);
		goto free_msgq;
	}

	if (csf_version.arch_ver == 2 && csf_version.max_ver == 0) {
		ret = qti_smmu_proxy_register_callbacks(NULL, NULL);
	} else if (csf_version.arch_ver == 2 && csf_version.max_ver == 5) {
		ret = qti_smmu_proxy_register_callbacks(smmu_proxy_map, smmu_proxy_unmap_nop);
	} else {
		pr_err("%s: Invalid CSF version: %d.%d\n", __func__, csf_version.arch_ver,
			csf_version.max_ver);
		goto free_msgq;
	}

	if (ret) {
		pr_err("%s: Failed to set SMMU proxy callbacks rc: %d\n", __func__, ret);
		goto free_msgq;
	}

	ret = smmu_proxy_create_dev(&smmu_proxy_dev_fops);
	if (ret) {
		pr_err("%s: Failed to create character device rc: %d\n", __func__,
		       ret);
		goto set_callbacks_null;
	}

	return 0;

set_callbacks_null:
	qti_smmu_proxy_register_callbacks(NULL, NULL);
free_msgq:
	gh_msgq_unregister(msgq_hdl);
	return ret;
}

static const struct of_device_id smmu_proxy_match_table[] = {
	{.compatible = "smmu-proxy-sender"},
	{},
};

static struct platform_driver smmu_proxy_driver = {
	.probe = sender_probe_handler,
	.driver = {
		.name = "qti-smmu-proxy",
		.of_match_table = smmu_proxy_match_table,
	},
};

int __init init_smmu_proxy_driver(void)
{
	return platform_driver_register(&smmu_proxy_driver);
}
module_init(init_smmu_proxy_driver);

MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL v2");
