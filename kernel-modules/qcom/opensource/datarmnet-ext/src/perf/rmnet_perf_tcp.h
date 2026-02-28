/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET PERF TCP framework */

#ifndef __RMNET_PERF_TCP_H__
#define __RMNET_PERF_TCP_H__

#include <linux/skbuff.h>

extern bool enable_tcp;
void rmnet_perf_ingress_handle_tcp(struct sk_buff *skb);
void rmnet_perf_ingress_rx_handler_tcp(struct sk_buff *skb);
void rmnet_perf_egress_handle_tcp(struct sk_buff *skb);
void rmnet_perf_tcp_update_quickack_thresh(u32 hash, u32 byte_thresh);
int rmnet_perf_tcp_init(void);
void rmnet_perf_tcp_exit(void);

#endif
