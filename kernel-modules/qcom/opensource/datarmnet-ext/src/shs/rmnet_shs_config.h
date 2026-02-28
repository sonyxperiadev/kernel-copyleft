/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/netdevice.h>

#ifndef _RMNET_SHS_CONFIG_H_
#define _RMNET_SHS_CONFIG_H_

#define RMNET_SHS_LOG_LEVEL_ERROR	1
#define RMNET_SHS_LOG_LEVEL_INFO	2
#define RMNET_SHS_LOG_LEVEL_DEBUG	3

extern struct rmnet_shs_cfg_s rmnet_shs_cfg;
extern int rmnet_is_real_dev_registered(const struct net_device *real_dev);
extern rx_handler_result_t rmnet_rx_handler(struct sk_buff **pskb);
int __init rmnet_shs_module_init(void);
void __exit rmnet_shs_module_exit(void);

#endif /* _RMNET_SMHS_CONFIG_H_ */
