/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _RMNET_MEM_NL_H_
#define _RMNET_MEM_NL_H_

#include <net/genetlink.h>

enum {
	RMNET_MEM_ATTR_UNSPEC,
	RMNET_MEM_ATTR_MODE,
	RMNET_MEM_ATTR_POOL_SIZE,
	RMNET_MEM_ATTR_INT,
	__RMNET_MEM_ATTR_MAX,
};

struct rmnet_memzone_req {
        int zone;
        int valid;
};
struct rmnet_pool_update_req {
        unsigned poolsize[4];
        unsigned valid_mask;
};

int rmnet_mem_nl_register(void);
void rmnet_mem_nl_unregister(void);
int rmnet_mem_nl_cmd_update_mode(struct sk_buff *skb, struct genl_info *info);
int rmnet_mem_nl_cmd_update_pool_size(struct sk_buff *skb, struct genl_info *info);
int rmnet_mem_nl_cmd_peak_pool_size(struct sk_buff *skb, struct genl_info *info);
int rmnet_mem_genl_send_int_to_userspace_no_info(int val, struct genl_info *info);

#endif /* _RMNET_MEM_GENL_H_ */
