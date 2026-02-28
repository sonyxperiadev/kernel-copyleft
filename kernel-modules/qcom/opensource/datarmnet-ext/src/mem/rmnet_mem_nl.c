// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "rmnet_mem_nl.h"
#include "rmnet_mem_priv.h"

#define RMNET_MEM_GENL_FAMILY_NAME "RMNET_MEM"
#define RMNET_MEM_GENL_VERSION 1

enum {
	RMNET_MEM_CMD_UNSPEC,
	RMNET_MEM_CMD_UPDATE_MODE,
	RMNET_MEM_CMD_UPDATE_POOL_SIZE,
	RMNET_MEM_CMD_UPDATE_PEAK_POOL_SIZE,
	__RMNET_MEM_GENL_CMD_MAX,
};

#define RMNET_MEM_ATTR_MAX (__RMNET_MEM_ATTR_MAX - 1)

uint32_t rmnet_shs_genl_seqnum;

static struct nla_policy rmnet_mem_nl_policy[RMNET_MEM_ATTR_MAX + 1] = {
    [RMNET_MEM_ATTR_MODE]  = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_memzone_req)),
    [RMNET_MEM_ATTR_POOL_SIZE]  = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_pool_update_req)),
};

static const struct genl_ops rmnet_mem_nl_ops[] = {
	{
		/* Deprecated, not used*/
		.cmd = RMNET_MEM_CMD_UPDATE_MODE,
		.doit = rmnet_mem_nl_cmd_update_mode,
	},
	{
		/* Adjust static pool size on the fly, set target_pool_size & start wq */
		.cmd = RMNET_MEM_CMD_UPDATE_POOL_SIZE,
		.doit = rmnet_mem_nl_cmd_update_pool_size,
	},
	{
		/* Set PB ind vote for what pool size will be adjusted to
		 * during active PB IND. Max(target_pool_size, pb_ind_max)
		 */
		.cmd = RMNET_MEM_CMD_UPDATE_PEAK_POOL_SIZE,
		.doit = rmnet_mem_nl_cmd_peak_pool_size,
	},
};

struct genl_family rmnet_aps_nl_family __ro_after_init = {
	.hdrsize = 0,
	.name = RMNET_MEM_GENL_FAMILY_NAME,
	.version = RMNET_MEM_GENL_VERSION,
	.maxattr = RMNET_MEM_ATTR_MAX,
	.policy = rmnet_mem_nl_policy,
	.ops = rmnet_mem_nl_ops,
	.n_ops = ARRAY_SIZE(rmnet_mem_nl_ops),
};

int rmnet_mem_genl_send_int_to_userspace_no_info(int val, struct genl_info *info)
{
	struct sk_buff *skb;
	void *msg_head;
	int rc;

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb == NULL)
		goto out;

	msg_head = genlmsg_put(skb, 0, 0, &rmnet_aps_nl_family,
			       0, RMNET_MEM_CMD_UPDATE_MODE);
	if (msg_head == NULL) {
		rc = -ENOMEM;
		rm_err("MEM_GNL: FAILED to msg_head %d\n", rc);
		kfree(skb);
		goto out;
	}
	rc = nla_put_u32(skb, RMNET_MEM_ATTR_INT, val);
	if (rc != 0) {
		rm_err("MEM_GNL: FAILED nla_put %d\n", rc);
		kfree(skb);
		goto out;
	}

	genlmsg_end(skb, msg_head);

	rc = genlmsg_reply(skb, info);
	if (rc != 0)
		goto out;

	rm_err("MEM_GNL: Successfully sent int %d\n", val);
	return 0;

out:
	rm_err("MEM_GNL: FAILED to send int %d\n", val);
	return -1;
}
int rmnet_mem_nl_register(void)
{
	return genl_register_family(&rmnet_aps_nl_family);
}

void rmnet_mem_nl_unregister(void)
{
	genl_unregister_family(&rmnet_aps_nl_family);
}
