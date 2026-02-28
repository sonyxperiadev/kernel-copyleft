// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* rmnet_offload configuration handlers */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include "rmnet_map.h"
#include "rmnet_private.h"
#include "qmi_rmnet.h"
#include "rmnet_offload_state.h"
#include "rmnet_offload_engine.h"

MODULE_LICENSE("GPL v2");

#define DL_PRIO_RMNET_OFFLOAD 0

/* Our internal state */
static struct rmnet_offload_state *rmnet_offload;

static void rmnet_offload_state_deregister_cb(void)
{
	struct rmnet_port *port = rmnet_offload->core_port;
	struct rmnet_offload_dl_marker_state *dl_state;

	qmi_rmnet_ps_ind_deregister(port, &rmnet_offload->powersave_ind);
	dl_state = &rmnet_offload->dl_marker_state;
	if (dl_state->dl_marker_cb_registered)
		rmnet_map_dl_ind_deregister(port, &dl_state->dl_ind);
}

static void rmnet_offload_state_free(void)
{
	LIST_HEAD(flush_list);
	/* Nothing to free if it was never allocated */
	if (!rmnet_offload)
		return;

	/* Unhook ourselves from the other drivers */
	rmnet_offload_state_deregister_cb();
	rmnet_offload_unset_hooks();
	rmnet_offload_engine_disable_chain_flush();
	synchronize_rcu();

	/* Flush out before we destroy everything */
	rmnet_offload_engine_flush_all_flows(&flush_list);
	rmnet_offload_engine_exit();
	kfree(rmnet_offload);
	rmnet_offload = NULL;
}

/* Conditionally register for DL marker callbacks and powersave notifications
 * from the core and DFC drivers.
 */
static int rmnet_offload_state_register_cb(void)
{
	struct rmnet_port *port = rmnet_offload->core_port;
	struct qmi_rmnet_ps_ind *ps_ind;
	struct rmnet_offload_dl_marker_state *dl_state;
	struct rmnet_map_dl_ind *dl_ind;
	int rc = RMNET_OFFLOAD_MGMT_SUCCESS;

	/* Register for powersave indications */
	ps_ind = &rmnet_offload->powersave_ind;
	ps_ind->ps_on_handler = rmnet_offload_handle_powersave_on;
	ps_ind->ps_off_handler = rmnet_offload_handle_powersave_off;
	if (qmi_rmnet_ps_ind_register(port, ps_ind)) {
		/* These callbacks are a no-op currently, but we might as well
		 * warn about the failure, since that points to a problem in
		 * rmnet_core.ko.
		 */
		rc = RMNET_OFFLOAD_MGMT_PARTIAL;
		pr_warn("%s(): PS CB registration failed\n", __func__);
	}

	dl_state = &rmnet_offload->dl_marker_state;
	dl_ind = &dl_state->dl_ind;
	dl_ind->priority = DL_PRIO_RMNET_OFFLOAD;
	dl_ind->dl_hdr_handler_v2 =
		rmnet_offload_handle_dl_header;
	dl_ind->dl_trl_handler_v2 =
		rmnet_offload_handle_dl_trailer;
	if (rmnet_map_dl_ind_register(port, dl_ind)) {
		rc = RMNET_OFFLOAD_MGMT_PARTIAL;
		pr_warn("%s(): DL CB registratation failed\n",
			__func__);
	} else {
		dl_state->dl_marker_cb_registered = true;
	}

	/* Flush on the end of SKB chains as a backup for DL markers */
	rmnet_offload_engine_enable_chain_flush();
	return rc;
}

static int rmnet_offload_state_init(struct rmnet_port *port)
{
	int rc;

	rmnet_offload = kzalloc(sizeof(*rmnet_offload), GFP_KERNEL);
	if (!rmnet_offload) {
		pr_err("%s(): Resource allocation failed\n", __func__);
		return RMNET_OFFLOAD_MGMT_FAILURE;
	}

	/* Store the port struct for processing */
	rmnet_offload->core_port = port;

	/* Let the engine core initialize itself */
	rc = rmnet_offload_engine_init();
	if (rc < 0)
		goto fail;

	/* Register for callbacks */
	rc = rmnet_offload_state_register_cb();
	if (rc != RMNET_OFFLOAD_MGMT_SUCCESS) {
		/* Traditionally this has not been an error. We just
		 * warned about it.
		 */
		pr_warn("%s(): Callback registration failed\n", __func__);
	}

	rmnet_offload->rmnet_offload_vnd_count = 1;

	/* Everything is ready. Say hello to the core driver */
	rmnet_offload_set_hooks();
	return rc;

fail:
	kfree(rmnet_offload);
	rmnet_offload = NULL;
	return rc;
}

static int rmnet_offload_state_notifier(struct notifier_block *nb,
					unsigned long notify_event,
					void *notify_data)
{
	struct net_device *device = netdev_notifier_info_to_dev(notify_data);
	struct rmnet_port *port;
	struct rmnet_priv *priv;
	int rc;

	(void)nb;
	/* We only care about rmnet devices */
	if (!device || strncmp(device->name, "rmnet_data", 10))
		goto done;

	switch (notify_event) {
	case NETDEV_REGISTER:
		/* Don't initialze if we've already done so */
		if (rmnet_offload) {
			/* Increment the device count and we're done */
			rmnet_offload->rmnet_offload_vnd_count++;
			goto done;
		}

		priv = netdev_priv(device);
		port = rmnet_get_port(priv->real_dev);
		if (!port) {
			pr_err("%s(): Invalid rmnet configuration on %s\n",
			       __func__, device->name);
			goto done;
		}

		pr_info("%s(): Initializing on device %s\n", __func__,
			device->name);
		rc = rmnet_offload_state_init(port);
		if (rc == RMNET_OFFLOAD_MGMT_FAILURE) {
			pr_err("%s(): Initialization failed\n", __func__);
			goto done;
		}

		break;
	case NETDEV_UNREGISTER:
		/* Don't uninitialize if we never initialized */
		if (!rmnet_offload)
			goto done;

		/* Decrement vnd count and free if no more devices */
		if (--rmnet_offload->rmnet_offload_vnd_count)
			goto done;

		pr_info("%s(): Uninitializing on device %s\n", __func__,
			device->name);
		rmnet_offload_state_free();
		break;
	}

done:
	return NOTIFY_DONE;
}

static struct notifier_block rmnet_offload_state_notifier_block = {
	.notifier_call = rmnet_offload_state_notifier,
	.priority = 1,
};

static int __init rmnet_offload_init(void)
{
	pr_info("%s(): rmnet_offload initializing\n", __func__);
	return register_netdevice_notifier(&rmnet_offload_state_notifier_block);
}

static void __exit rmnet_offload_exit(void)
{
	pr_info("%s(): rmnet_offload exiting\n", __func__);
	unregister_netdevice_notifier(&rmnet_offload_state_notifier_block);
}

/* Internal state accessor */
struct rmnet_offload_state *rmnet_offload_state_get(void)
{
	return rmnet_offload;
}

module_init(rmnet_offload_init);
module_exit(rmnet_offload_exit);
