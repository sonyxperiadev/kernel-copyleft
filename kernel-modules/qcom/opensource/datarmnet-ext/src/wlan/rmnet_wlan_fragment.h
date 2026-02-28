/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN fragment handler framework */

#ifndef __RMNET_WLAN_FRAGMENT_H__
#define __RMNET_WLAN_FRAGMENT_H__

#include <linux/skbuff.h>
#include "rmnet_wlan.h"

/* Fragment handling interface */
int rmnet_wlan_fragment_v4(struct sk_buff *skb, int ip_len,
			   struct rmnet_wlan_tuple *tuple,
			   struct rmnet_wlan_fwd_info *fwd_info);
int rmnet_wlan_fragment_v6(struct sk_buff *skb, int ip_len,
			   struct rmnet_wlan_tuple *tuple,
			   struct rmnet_wlan_fwd_info *fwd_info);

/* Initialize fragment handling */
int rmnet_wlan_fragment_init(void);
/* Purge all fragment information */
void rmnet_wlan_fragments_remove(void);

/* Handle FWD information removal */
void rmnet_wlan_fragment_del_fwd_info(struct rmnet_wlan_fwd_info *info);

#endif
