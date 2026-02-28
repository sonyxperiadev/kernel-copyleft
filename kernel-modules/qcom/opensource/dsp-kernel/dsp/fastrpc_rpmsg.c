// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2011-2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2018, Linaro Limited
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/rpmsg.h>
#include <linux/pm_qos.h>
#include "../include/uapi/misc/fastrpc.h"
#include <linux/of_reserved_mem.h>
#include "fastrpc_shared.h"
#include <linux/soc/qcom/pdr.h>
#include <linux/delay.h>

void fastrpc_channel_ctx_put(struct fastrpc_channel_ctx *cctx);
void fastrpc_update_gctx(struct fastrpc_channel_ctx *cctx, int flag);
void fastrpc_lowest_capacity_corecount(struct device *dev, struct fastrpc_channel_ctx *cctx);
int fastrpc_init_privileged_gids(struct device *dev, char *prop_name,
						struct gid_list *gidlist);
int fastrpc_setup_service_locator(struct fastrpc_channel_ctx *cctx, char *client_name,
					char *service_name, char *service_path, int spd_session);
void fastrpc_register_wakeup_source(struct device *dev,
	const char *client_name, struct wakeup_source **device_wake_source);
int fastrpc_mmap_remove_ssr(struct fastrpc_channel_ctx *cctx);
void fastrpc_queue_pd_status(struct fastrpc_user *fl, int domain, int status, int sessionid);

struct fastrpc_channel_ctx* get_current_channel_ctx(struct device *dev)
{
	return dev_get_drvdata(dev->parent);
}

static int fastrpc_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct device *rdev = &rpdev->dev;
	struct fastrpc_channel_ctx *data;
	int i, err, domain_id = -1, vmcount;
	const char *domain;
	bool secure_dsp;
	unsigned int vmids[FASTRPC_MAX_VMIDS];

	dev_info(rdev, "%s started\n", __func__);

	err = of_property_read_string(rdev->of_node, "label", &domain);
	if (err) {
		dev_info(rdev, "FastRPC Domain not specified in DT\n");
		return err;
	}

	for (i = 0; i <= CDSP_DOMAIN_ID; i++) {
		if (!strcmp(domains[i], domain)) {
			domain_id = i;
			break;
		}
	}

	if (domain_id < 0) {
		dev_info(rdev, "FastRPC Invalid Domain ID %d\n", domain_id);
		return -EINVAL;
	}

	if (of_reserved_mem_device_init_by_idx(rdev, rdev->of_node, 0))
		dev_info(rdev, "no reserved DMA memory for FASTRPC\n");

	vmcount = of_property_read_variable_u32_array(rdev->of_node,
				"qcom,vmids", &vmids[0], 0, FASTRPC_MAX_VMIDS);
	if (vmcount < 0)
		vmcount = 0;
	else if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	err = fastrpc_init_privileged_gids(rdev, "qcom,fastrpc-gids", &data->gidlist);
	if (err)
		dev_err(rdev, "Privileged gids init failed.\n");

	if (vmcount) {
		data->vmcount = vmcount;
		data->perms = BIT(QCOM_SCM_VMID_HLOS);
		for (i = 0; i < data->vmcount; i++) {
			data->vmperms[i].vmid = vmids[i];
			data->vmperms[i].perm = QCOM_SCM_PERM_RWX;
		}
	}

	atomic_set(&data->teardown, 0);
	secure_dsp = !(of_property_read_bool(rdev->of_node, "qcom,non-secure-domain"));
	data->secure = secure_dsp;

	of_property_read_u32(rdev->of_node, "qcom,rpc-latency-us",
			&data->qos_latency);

	fastrpc_lowest_capacity_corecount(rdev, data);
	if (data->lowest_capacity_core_count > 0 &&
	    of_property_read_bool(rdev->of_node, "qcom,single-core-latency-vote"))
		data->lowest_capacity_core_count = 1;

	kref_init(&data->refcount);
	dev_set_drvdata(&rpdev->dev, data);
	rdev->dma_mask = &data->dma_mask;
	dma_set_mask_and_coherent(rdev, DMA_BIT_MASK(32));
	INIT_LIST_HEAD(&data->users);
	INIT_LIST_HEAD(&data->gmaps);
	INIT_LIST_HEAD(&data->rootheap_bufs.list);
	mutex_init(&data->wake_mutex);
	spin_lock_init(&data->lock);
	spin_lock_init(&(data->gmsg_log.tx_lock));
	spin_lock_init(&(data->gmsg_log.rx_lock));
	idr_init(&data->ctx_idr);
	ida_init(&data->tgid_frpc_ida);
	init_completion(&data->ssr_complete);
	init_waitqueue_head(&data->ssr_wait_queue);
	data->domain_id = domain_id;
	data->max_sess_per_proc = FASTRPC_MAX_SESSIONS_PER_PROCESS;
	data->rpdev = rpdev;

	err = of_platform_populate(rdev->of_node, NULL, NULL, rdev);
	if (err)
		goto populate_error;

	switch (domain_id) {
	case ADSP_DOMAIN_ID:
	case MDSP_DOMAIN_ID:
	case SDSP_DOMAIN_ID:
		/* Unsigned PD offloading is only supported on CDSP*/
		data->unsigned_support = false;
		err = fastrpc_device_register(rdev, data, secure_dsp, domains[domain_id]);
		if (err)
			goto fdev_error;
		data->cpuinfo_todsp = FASTRPC_CPUINFO_DEFAULT;
		break;
	case CDSP_DOMAIN_ID:
		data->unsigned_support = true;
		/* Create both device nodes so that we can allow both Signed and Unsigned PD */
		err = fastrpc_device_register(rdev, data, true, domains[domain_id]);
		if (err)
			goto fdev_error;

		err = fastrpc_device_register(rdev, data, false, domains[domain_id]);
		if (err)
			goto fdev_error;
		data->cpuinfo_todsp = FASTRPC_CPUINFO_EARLY_WAKEUP;
		break;
	default:
		err = -EINVAL;
		goto fdev_error;
	}

	if (domain_id == ADSP_DOMAIN_ID) {
		err = fastrpc_setup_service_locator(data, AUDIO_PDR_SERVICE_LOCATION_CLIENT_NAME,
			AUDIO_PDR_ADSP_SERVICE_NAME, ADSP_AUDIOPD_NAME, 0);
		if (err)
			goto fdev_error;

		err = fastrpc_setup_service_locator(data, SENSORS_PDR_ADSP_SERVICE_LOCATION_CLIENT_NAME,
			SENSORS_PDR_ADSP_SERVICE_NAME, ADSP_SENSORPD_NAME, 1);
		if (err)
			goto fdev_error;

		err = fastrpc_setup_service_locator(data, OIS_PDR_ADSP_SERVICE_LOCATION_CLIENT_NAME,
			OIS_PDR_ADSP_SERVICE_NAME, ADSP_OISPD_NAME, 2);
		if (err)
			goto fdev_error;
	} else if (domain_id == SDSP_DOMAIN_ID) {
		err = fastrpc_setup_service_locator(data, SENSORS_PDR_SLPI_SERVICE_LOCATION_CLIENT_NAME,
			SENSORS_PDR_SLPI_SERVICE_NAME, SLPI_SENSORPD_NAME, 0);
		if (err)
			goto fdev_error;
	}

	mutex_lock(&data->wake_mutex);
	if(data->fdevice)
		fastrpc_register_wakeup_source(data->fdevice->miscdev.this_device,
			FASTRPC_NON_SECURE_WAKE_SOURCE_CLIENT_NAME, &data->wake_source);
	if(data->secure_fdevice)
		fastrpc_register_wakeup_source(data->secure_fdevice->miscdev.this_device,
			FASTRPC_SECURE_WAKE_SOURCE_CLIENT_NAME, &data->wake_source_secure);
	mutex_unlock(&data->wake_mutex);

	fastrpc_update_gctx(data, 1);

	dev_info(rdev, "Opened rpmsg channel for %s", domain);
	return 0;

fdev_error:
	if (data->fdevice)
		misc_deregister(&data->fdevice->miscdev);
	if (data->secure_fdevice)
		misc_deregister(&data->secure_fdevice->miscdev);
populate_error:
	kfree(data->gidlist.gids);
	data->gidlist.gids = NULL;
	kfree(data);
	return err;
}

static void fastrpc_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct fastrpc_channel_ctx *cctx = dev_get_drvdata(&rpdev->dev);
	struct fastrpc_user *user;
	unsigned long flags;
	int i = 0;

	dev_info(cctx->dev, "%s started", __func__);

	/* No invocations past this point */
	spin_lock_irqsave(&cctx->lock, flags);
	atomic_set(&cctx->teardown, 1);
	cctx->staticpd_status = false;
	list_for_each_entry(user, &cctx->users, user) {
		fastrpc_queue_pd_status(user, cctx->domain_id, FASTRPC_DSP_SSR, user->sessionid);
		fastrpc_notify_users(user);
		/* cancel all waiting dspsignals */
		fastrpc_ssr_dspsignal_cancel_wait(user);
	}
	spin_unlock_irqrestore(&cctx->lock, flags);

	if (cctx->fdevice)
		misc_deregister(&cctx->fdevice->miscdev);

	if (cctx->secure_fdevice)
		misc_deregister(&cctx->secure_fdevice->miscdev);

	for (i = 0; i < FASTRPC_MAX_SPD; i++) {
		if (cctx->spd[i].pdrhandle)
			pdr_handle_release(cctx->spd[i].pdrhandle);
	}

	spin_lock_irqsave(&cctx->lock, flags);
	/*
	 * If there are other ongoing remote invocations, wait for them to
	 * complete before cleaning up the channel resources, to avoid UAF.
	 */
	while (cctx->invoke_cnt > 0) {
		spin_unlock_irqrestore(&cctx->lock, flags);
		wait_event_interruptible(cctx->ssr_wait_queue,
				cctx->invoke_cnt == 0);
		spin_lock_irqsave(&cctx->lock, flags);
	}
	spin_unlock_irqrestore(&cctx->lock, flags);

	/*
	 * As remote channel is down, corresponding SMMU devices will also
	 * be removed. So free all SMMU mappings of every process using this
	 * channel to avoid any UAF later.
	 */
	list_for_each_entry(user, &cctx->users, user) {
		fastrpc_free_user(user);
	}

	mutex_lock(&cctx->wake_mutex);
	if (cctx->wake_source) {
		wakeup_source_unregister(cctx->wake_source);
		cctx->wake_source = NULL;
	}
	if (cctx->wake_source_secure) {
		wakeup_source_unregister(cctx->wake_source_secure);
		cctx->wake_source_secure = NULL;
	}
	mutex_unlock(&cctx->wake_mutex);

	dev_info(cctx->dev, "Closing rpmsg channel for %s", domains[cctx->domain_id]);
	kfree(cctx->gidlist.gids);
	of_platform_depopulate(&rpdev->dev);
	fastrpc_mmap_remove_ssr(cctx);
	cctx->dev = NULL;
	cctx->rpdev = NULL;
	// Wake up all process releases, if waiting for SSR to complete
	complete_all(&cctx->ssr_complete);
	fastrpc_update_gctx(cctx, 0);
	fastrpc_channel_ctx_put(cctx);
}

static int fastrpc_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
				  int len, void *priv, u32 addr)
{
	struct fastrpc_channel_ctx *cctx = dev_get_drvdata(&rpdev->dev);

	return fastrpc_handle_rpc_response(cctx, data, len);
}

static const struct of_device_id fastrpc_rpmsg_of_match[] = {
	{ .compatible = "qcom,fastrpc" },
	{ },
};
MODULE_DEVICE_TABLE(of, fastrpc_rpmsg_of_match);

static struct rpmsg_driver fastrpc_driver = {
	.probe = fastrpc_rpmsg_probe,
	.remove = fastrpc_rpmsg_remove,
	.callback = fastrpc_rpmsg_callback,
	.drv = {
		.name = "qcom,fastrpc",
		.of_match_table = fastrpc_rpmsg_of_match,
	},
};

int fastrpc_transport_send(struct fastrpc_channel_ctx *cctx, void *rpc_msg, uint32_t rpc_msg_size) {
	int err = 0;

	if (atomic_read(&cctx->teardown))
		return -EPIPE;

	err = rpmsg_send(cctx->rpdev->ept, rpc_msg, rpc_msg_size);
	if (err == -EIO) {
		pr_err("fastrpc: failed to send message due to SSR\n");
		err = -EPIPE;
	}
	return err;
}

int fastrpc_transport_init(void) {
	int ret;

	ret = register_rpmsg_driver(&fastrpc_driver);
	if (ret < 0) {
		pr_err("fastrpc: failed to register rpmsg driver\n");
		return ret;
	}

	return 0;
}

void fastrpc_transport_deinit(void) {
	unregister_rpmsg_driver(&fastrpc_driver);
}
