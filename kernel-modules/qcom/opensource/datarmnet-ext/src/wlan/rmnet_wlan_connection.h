/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN connection management framework */

#ifndef __RMNET_WLAN_CONNECTION_H__
#define __RMNET_WLAN_CONNECTION_H__

#include <linux/types.h>
#include <linux/in6.h>
#include "rmnet_wlan.h"

struct rmnet_wlan_connection_info {
	union {
		__be32 v4_saddr;
		struct in6_addr v6_saddr;
	};
	union {
		__be32 v4_daddr;
		struct in6_addr v6_daddr;
	};
	u8 ip_proto;
};

/* Lookup */
struct rmnet_wlan_fwd_info *
rmnet_wlan_connection_find(struct rmnet_wlan_connection_info *info);

/* Flush everything */
void rmnet_wlan_connection_flush(void);

/* External Hooks */
void rmnet_wlan_set_hooks(void);
void rmnet_wlan_unset_hooks(void);

/* Setup and teardown interface */
int rmnet_wlan_connection_init(void);
int rmnet_wlan_connection_deinit(void);

#endif
