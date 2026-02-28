/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET PERF UDP framework */

#ifndef __RMNET_PERF_UDP_H__
#define __RMNET_PERF_UDP_H__

#include <linux/skbuff.h>

void rmnet_perf_ingress_handle_udp(struct sk_buff *skb);
void rmnet_perf_egress_handle_udp(struct sk_buff *skb);
int rmnet_perf_udp_init(void);
void rmnet_perf_udp_exit(void);

#endif
