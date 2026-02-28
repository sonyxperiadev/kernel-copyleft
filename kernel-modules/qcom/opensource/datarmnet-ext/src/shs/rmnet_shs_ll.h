/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _RMNET_SHS_LL_H_
#define _RMNET_SHS_LL_H_

int rmnet_shs_ll_handler(struct sk_buff *skb, struct rmnet_shs_clnt_s *clnt_cfg);
void rmnet_shs_ll_init(void);
void rmnet_shs_ll_deinit(void);
void rmnet_shs_add_llflow(struct rmnet_shs_wq_flow_node  *node);
void rmnet_shs_remove_llflow(struct rmnet_shs_wq_flow_node  *node);
int rmnet_shs_is_filter_match(struct sk_buff *skb);

#endif /* _RMNET_SHS_LL_H_ */
