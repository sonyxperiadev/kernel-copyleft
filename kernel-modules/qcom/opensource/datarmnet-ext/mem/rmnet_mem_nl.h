/* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-only
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
int rmnet_mem_genl_send_int_to_userspace_no_info(int val, struct genl_info *info);

#endif /* _RMNET_MEM_GENL_H_ */

#ifndef DATARMNET7b1420e7bf
#define DATARMNET7b1420e7bf
#include <net/genetlink.h>
enum{DATARMNET2cc66aaa85,DATARMNETe5184c7a76,DATARMNETb0428b7575,
DATARMNETbd57d2442f,DATARMNETc820b7a9dc,};struct DATARMNET5d6175c98d{int 
DATARMNET3a4d9ad400;int valid;};struct DATARMNET5d23779a8f{unsigned 
DATARMNETe87b937bb6[(0xd11+230-0xdf3)];unsigned DATARMNET855b934a37;};int 
rmnet_mem_nl_register(void);void rmnet_mem_nl_unregister(void);int 
DATARMNET291f036d31(struct sk_buff*skb,struct genl_info*DATARMNET54338da2ff);int
 DATARMNET8e48a951e4(struct sk_buff*skb,struct genl_info*DATARMNET54338da2ff);
int DATARMNETe85d734d4f(int val,struct genl_info*DATARMNET54338da2ff);
#endif 

