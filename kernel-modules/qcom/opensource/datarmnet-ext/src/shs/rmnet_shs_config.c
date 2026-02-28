// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/module.h>
#include "rmnet_map.h"
#include "rmnet_private.h"
#include "rmnet_shs_config.h"
#include "rmnet_shs.h"
#include "rmnet_shs_wq.h"
#include "rmnet_shs_ll.h"
#include "rmnet_shs_freq.h"
#include "rmnet_shs_wq_genl.h"
#include "rmnet_shs_common.h"
#include "rmnet_shs_modules.h"
#include "rmnet_module.h"

static int rmnet_shs_dev_notify_cb(struct notifier_block *nb,
				    unsigned long event, void *data);

static struct notifier_block rmnet_shs_dev_notifier __read_mostly = {
	.notifier_call = rmnet_shs_dev_notify_cb,
	.priority = 2,
};

/* Version array, version's should be inserted at the end */
static char *rmnet_shs_version[] = {"43a6b",
				    "a2ce6",
				    "d1ab1",
				    "d879b",
				    "ac626",
				    "5cff7",
				    "a586b"
				   };


module_param_array(rmnet_shs_version, charp, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_version, "Version of shs driver");


static const struct rmnet_module_hook_register_info
rmnet_shs_ll_entry_hook = {
		.hooknum = RMNET_MODULE_HOOK_SHS_SKB_LL_ENTRY,
		.func = rmnet_shs_ll_handler
};

static int rmnet_vnd_total;
/* Enable smart hashing capability upon call to initialize module*/
int __init rmnet_shs_module_init(void)
{
	pr_info("%s(): Loaded rmnet SHS module \n", __func__);
	trace_rmnet_shs_high(RMNET_SHS_MODULE, RMNET_SHS_MODULE_INIT,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	if (rmnet_shs_wq_genl_init()) {
		rm_err("%s", "SHS_GNL: Failed to init generic netlink");
	}

	return register_netdevice_notifier(&rmnet_shs_dev_notifier);
}

/* Remove smart hashing capability upon call to initialize module */
void __exit rmnet_shs_module_exit(void)
{
	trace_rmnet_shs_high(RMNET_SHS_MODULE, RMNET_SHS_MODULE_EXIT,
			    0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);
	unregister_netdevice_notifier(&rmnet_shs_dev_notifier);

	rmnet_shs_wq_genl_deinit();

	pr_info("%s(): Exiting rmnet SHS module\n", __func__);
}

static int rmnet_shs_dev_notify_cb(struct notifier_block *nb,
				    unsigned long event, void *data)
{

	struct net_device *dev = netdev_notifier_info_to_dev(data);
	struct rmnet_priv *priv;
	struct rmnet_port *port;
	int ret = 0;

	if (!dev) {
		rmnet_shs_crit_err[RMNET_SHS_NETDEV_ERR]++;
		return NOTIFY_DONE;
	}

	if (!(strncmp(dev->name, "rmnet_data", 10) == 0))
		return NOTIFY_DONE;

	switch (event) {

	case NETDEV_DOWN:
		rmnet_shs_wq_reset_ep_active(dev);
		break;
	case NETDEV_UNREGISTER:
		rmnet_vnd_total--;

		/* Deinitialize if last vnd is going down or if
		 * phy_dev is going down.
		 */
		if (!rmnet_vnd_total && rmnet_shs_cfg.rmnet_shs_init_complete) {
			unsigned int cpu_switch;

			pr_info("rmnet_shs deinit %s going down\n", dev->name);
			rmnet_shs_skb_entry_disable();
			rmnet_shs_switch_disable();

			rmnet_module_hook_unregister_no_sync(&rmnet_shs_ll_entry_hook, 1);
			qmi_rmnet_ps_ind_deregister(rmnet_shs_cfg.port,
						    &rmnet_shs_cfg.rmnet_idl_ind_cb);
			rmnet_map_dl_ind_deregister(rmnet_shs_cfg.port,
						    &rmnet_shs_cfg.dl_mrk_ind_cb);
			rmnet_map_pb_ind_deregister(rmnet_shs_cfg.port,
						    &rmnet_shs_cfg.pb_mrk_ind_cb);

			rmnet_shs_cancel_table();
			rmnet_shs_ll_deinit();
			cpu_switch = rmnet_shs_rx_wq_exit();
			/* Only Unhook vh if we registered in 1st place */
			if (rmnet_shs_cfg.is_reg_dl_mrk_ind)
				rmnet_shs_vh_unset();
			rmnet_shs_wq_exit();
			rmnet_shs_exit(cpu_switch);
			trace_rmnet_shs_high(RMNET_SHS_MODULE,
					     RMNET_SHS_MODULE_INIT_WQ,
					     0xDEF, 0xDEF, 0xDEF,
					     0xDEF, NULL, NULL);
		}
		break;

	case NETDEV_REGISTER:
		rmnet_vnd_total++;

		if (rmnet_vnd_total && !rmnet_shs_cfg.rmnet_shs_init_complete) {
			pr_info("rmnet_shs initializing %s\n", dev->name);
			priv = netdev_priv(dev);
			port = rmnet_get_port(priv->real_dev);
			if (!port) {
				pr_err("rmnet_shs: invalid rmnet_port\n");
				break;
			}
			rmnet_shs_init(priv->real_dev, dev);
			rmnet_shs_wq_init();
			rmnet_shs_rx_wq_init();
		}

		break;
	case NETDEV_UP:
		if (!rmnet_shs_cfg.is_reg_dl_mrk_ind &&
		    rmnet_shs_cfg.rmnet_shs_init_complete) {

			port = rmnet_shs_cfg.port;
			if (!port) {
				pr_err("rmnet_shs: invalid rmnet_cfg_port\n");
				break;
			}

			rmnet_shs_cfg.dl_mrk_ind_cb.priority = RMNET_SHS;
			rmnet_shs_cfg.pb_mrk_ind_cb.priority = RMNET_SHS;
			if (port->data_format & RMNET_INGRESS_FORMAT_DL_MARKER_V2) {
				rmnet_shs_cfg.dl_mrk_ind_cb.dl_hdr_handler_v2 =
					&rmnet_shs_dl_hdr_handler_v2;
				rmnet_shs_cfg.dl_mrk_ind_cb.dl_trl_handler_v2 =
					&rmnet_shs_dl_trl_handler_v2;
				rmnet_shs_cfg.pb_mrk_ind_cb.pb_ind_handler =
					&rmnet_shs_pb_hdr_handler;
			}
			rmnet_shs_cfg.rmnet_idl_ind_cb.ps_on_handler =
					&rmnet_shs_ps_on_hdlr;
			rmnet_shs_cfg.rmnet_idl_ind_cb.ps_off_handler =
					&rmnet_shs_ps_off_hdlr;

			ret = rmnet_map_dl_ind_register(port,
						        &rmnet_shs_cfg.dl_mrk_ind_cb);
			if (ret)
				pr_err("%s(): rmnet dl_ind registration fail\n",
				       __func__);

			ret = rmnet_map_pb_ind_register(port,
						        &rmnet_shs_cfg.pb_mrk_ind_cb);
			if (ret)
				pr_err("%s(): rmnet pb_ind registration fail\n",
				       __func__);

			ret = qmi_rmnet_ps_ind_register(port,
						        &rmnet_shs_cfg.rmnet_idl_ind_cb);
			if (ret)
				pr_err("%s(): rmnet ps_ind registration fail\n",
				       __func__);

			rmnet_shs_wq_set_ep_active(dev);
			rmnet_shs_wq_refresh_ep_masks();
			rmnet_shs_wq_refresh_new_flow_list();
			/* Mark active before RCU pointer */
			rmnet_shs_update_cfg_mask();
			trace_rmnet_shs_high(RMNET_SHS_MODULE,
					     RMNET_SHS_MODULE_INIT_WQ,
					     0xDEF, 0xDEF, 0xDEF,
					     0xDEF, NULL, NULL);

			rmnet_shs_switch_disable();
			rmnet_shs_skb_entry_enable();
			rmnet_module_hook_register(&rmnet_shs_ll_entry_hook, 1);
			rmnet_shs_vh_set();
			rmnet_shs_cfg.is_reg_dl_mrk_ind = 1;
			/* Needed so we don't mark active twice*/
			break;
		}
		rmnet_shs_wq_set_ep_active(dev);

		break;

	default:
		break;

	}
	return NOTIFY_DONE;
}

module_init(rmnet_shs_module_init);
module_exit(rmnet_shs_module_exit);
