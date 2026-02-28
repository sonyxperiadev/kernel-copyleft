// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "rmnet_shs_modules.h"
#include "rmnet_shs_common.h"
#include "rmnet_shs_ll.h"

#include "rmnet_shs_wq_genl.h"
#include <net/sock.h>
#include <linux/skbuff.h>
#include <linux/cpumask.h>

MODULE_LICENSE("GPL v2");

static struct net *last_net;
static u32 last_snd_portid;

static struct net *msg_last_net;
static u32 msg_last_snd_portid;

uint32_t rmnet_shs_genl_seqnum;
uint32_t rmnet_shs_genl_msg_seqnum;
int rmnet_shs_userspace_connected;

#define RMNET_SHS_GENL_MAX_STR_LEN	255

#define RMNET_SHS_GENL_SEC_TO_NSEC(x)   ((x) * 1000000000)

/* Static Functions and Definitions */
static struct nla_policy rmnet_shs_genl_attr_policy[RMNET_SHS_GENL_ATTR_MAX + 1] = {
	[RMNET_SHS_GENL_ATTR_INT]  = { .type = NLA_S32 },
	[RMNET_SHS_GENL_ATTR_SUGG] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_shs_wq_sugg_info)),
	[RMNET_SHS_GENL_ATTR_SEG]  = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_shs_wq_seg_info)),
	[RMNET_SHS_GENL_ATTR_FLOW] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_shs_wq_flow_info)),
	[RMNET_SHS_GENL_ATTR_QUICKACK]  = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_shs_wq_quickack_info)),
	[RMNET_SHS_GENL_ATTR_STR]  = { .type = NLA_NUL_STRING, .len = RMNET_SHS_GENL_MAX_STR_LEN},
	[RMNET_SHS_GENL_ATTR_BOOTUP] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_shs_bootup_info)),
};

#define RMNET_SHS_GENL_OP(_cmd, _func)			\
	{						\
		.cmd	= _cmd,				\
		.doit	= _func,			\
		.dumpit	= NULL,				\
		.flags	= 0,				\
	}

static const struct genl_ops rmnet_shs_genl_ops[] = {
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_CMD_INIT_SHSUSRD,
			  rmnet_shs_genl_dma_init),
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_CMD_TRY_TO_MOVE_FLOW,
			  rmnet_shs_genl_try_to_move_flow),
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_CMD_SET_FLOW_SEGMENTATION,
			  rmnet_shs_genl_set_flow_segmentation),
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_CMD_MEM_SYNC,
			  rmnet_shs_genl_mem_sync),
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_CMD_LL_FLOW,
			  rmnet_shs_genl_set_flow_ll),
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_CMD_QUICKACK,
			  rmnet_shs_genl_set_quickack_thresh),
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_CMD_BOOTUP,
			  rmnet_shs_genl_set_bootup_config),

};

/* Generic Netlink Message Channel policy and ops */
static struct nla_policy rmnet_shs_genl_msg_attr_policy[RMNET_SHS_GENL_ATTR_MAX + 1] = {
	[RMNET_SHS_GENL_MSG_ATTR_REQ]  = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_shs_msg_req)),
	[RMNET_SHS_GENL_MSG_ATTR_RESP]  = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_shs_msg_resp)),
};

static const struct genl_ops rmnet_shs_genl_msg_ops[] = {
	RMNET_SHS_GENL_OP(RMNET_SHS_GENL_MSG_WAIT_CMD,
			  rmnet_shs_genl_msg_req_hdlr),
};

struct genl_family rmnet_shs_genl_family = {
	.hdrsize = 0,
	.name    = RMNET_SHS_GENL_FAMILY_NAME,
	.version = RMNET_SHS_GENL_VERSION,
	.maxattr = RMNET_SHS_GENL_ATTR_MAX,
	.policy = rmnet_shs_genl_attr_policy,
	.ops     = rmnet_shs_genl_ops,
	.n_ops   = ARRAY_SIZE(rmnet_shs_genl_ops),
};

struct genl_family rmnet_shs_genl_msg_family = {
	.hdrsize = 0,
	.name    = RMNET_SHS_GENL_MSG_FAMILY_NAME,
	.version = RMNET_SHS_GENL_VERSION,
	.maxattr = RMNET_SHS_GENL_ATTR_MAX,
	.policy = rmnet_shs_genl_msg_attr_policy,
	.ops     = rmnet_shs_genl_msg_ops,
	.n_ops   = ARRAY_SIZE(rmnet_shs_genl_msg_ops),
};

int rmnet_shs_genl_send_int_to_userspace(struct genl_info *info, int val)
{
	struct sk_buff *skb;
	void *msg_head;
	int rc;

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb == NULL)
		goto out;

	msg_head = genlmsg_put(skb, 0, info->snd_seq+1, &rmnet_shs_genl_family,
			       0, RMNET_SHS_GENL_CMD_INIT_SHSUSRD);
	if (msg_head == NULL) {
		rc = -ENOMEM;
		rm_err("SHS_GNL: FAILED to msg_head %d\n", rc);
		kfree(skb);
		goto out;
	}
	rc = nla_put_u32(skb, RMNET_SHS_GENL_ATTR_INT, val);
	if (rc != 0) {
		rm_err("SHS_GNL: FAILED nla_put %d\n", rc);
		kfree(skb);
		goto out;
	}

	genlmsg_end(skb, msg_head);

	rc = genlmsg_unicast(genl_info_net(info), skb, info->snd_portid);
	if (rc != 0)
		goto out;

	rm_err("SHS_GNL: Successfully sent int %d\n", val);
	return 0;

out:
	/* TODO: Need to free skb?? */
	rm_err("SHS_GNL: FAILED to send int %d\n", val);
	return -1;
}

int rmnet_shs_genl_send_int_to_userspace_no_info(int val)
{
	struct sk_buff *skb;
	void *msg_head;
	int rc;

	if (last_net == NULL) {
		rm_err("SHS_GNL: FAILED to send int %d - last_net is NULL\n",
		       val);
		return -1;
	}

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb == NULL)
		goto out;

	msg_head = genlmsg_put(skb, 0, rmnet_shs_genl_seqnum++, &rmnet_shs_genl_family,
			       0, RMNET_SHS_GENL_CMD_INIT_SHSUSRD);
	if (msg_head == NULL) {
		rc = -ENOMEM;
		rm_err("SHS_GNL: FAILED to msg_head %d\n", rc);
		kfree(skb);
		goto out;
	}
	rc = nla_put_u32(skb, RMNET_SHS_GENL_ATTR_INT, val);
	if (rc != 0) {
		rm_err("SHS_GNL: FAILED nla_put %d\n", rc);
		kfree(skb);
		goto out;
	}

	genlmsg_end(skb, msg_head);

	rc = genlmsg_unicast(last_net, skb, last_snd_portid);
	if (rc != 0)
		goto out;

	rm_err("SHS_GNL: Successfully sent int %d\n", val);
	return 0;

out:
	/* TODO: Need to free skb?? */
	rm_err("SHS_GNL: FAILED to send int %d\n", val);
	rmnet_shs_userspace_connected = 0;
	return -1;
}

int rmnet_shs_genl_send_msg_to_userspace(void)
{
	struct sk_buff *skb;
	void *msg_head;
	int rc;
	int val = rmnet_shs_genl_seqnum++;

	rm_err("SHS_GNL: Trying to send msg %d\n", val);
	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb == NULL)
		goto out;

	msg_head = genlmsg_put(skb, 0, rmnet_shs_genl_seqnum++, &rmnet_shs_genl_family,
			       0, RMNET_SHS_GENL_CMD_INIT_SHSUSRD);
	if (msg_head == NULL) {
		rc = -ENOMEM;
		rm_err("SHS_GNL: FAILED to msg_head %d\n", rc);
		kfree(skb);
		goto out;
	}
	rc = nla_put_u32(skb, RMNET_SHS_GENL_ATTR_INT, val);
	if (rc != 0) {
		rm_err("SHS_GNL: FAILED nla_put %d\n", rc);
		kfree(skb);
		goto out;
	}

	genlmsg_end(skb, msg_head);

	genlmsg_multicast(&rmnet_shs_genl_family, skb, 0, 0, GFP_ATOMIC);

	rm_err("SHS_GNL: Successfully sent int %d\n", val);
	return 0;

out:
	/* TODO: Need to free skb?? */
	rm_err("SHS_GNL: FAILED to send int %d\n", val);
	rmnet_shs_userspace_connected = 0;
	return -1;
}

/* Currently unused - handles message from userspace to initialize the shared memory,
 * memory is inited by kernel wq automatically
 */
int rmnet_shs_genl_dma_init(struct sk_buff *skb_2, struct genl_info *info)
{
	rm_err("%s", "SHS_GNL: rmnet_shs_genl_dma_init: Clear LL");

	rmnet_shs_ll_deinit();

	if (info == NULL) {
		rm_err("%s", "SHS_GNL: an error occured - info is null");
		return -1;
	}

	return 0;
}

int rmnet_shs_genl_set_flow_ll(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_shs_wq_flow_node *flow_info;

	rm_err("%s", "SHS_GNL: rmnet_shs_genl_set_flow_ll");

	if (info == NULL) {
		rm_err("%s", "SHS_GNL: an error occured - info is null");
		return -1;
	}

	na = info->attrs[RMNET_SHS_GENL_ATTR_FLOW];
	if (na) {
        /* Dyanmically allocating filter/flow info which must be freed */
		flow_info = kzalloc(sizeof(*flow_info), GFP_ATOMIC);
		if (!flow_info) {
			rm_err("%s", "SHS_GNL: rmnet_shs_genl_set_flow_ll flow info failure");
			rmnet_shs_genl_send_int_to_userspace(info,
					RMNET_SHS_SEG_FAIL_RESP_INT);
			return 0;
		}

		if (nla_memcpy(&flow_info->info, na, sizeof(flow_info->info)) > 0) {
			if (flow_info->info.opcode == RMNET_SHS_LL_OPCODE_ADD)
				rmnet_shs_add_llflow(flow_info);
			else if (flow_info->info.opcode == RMNET_SHS_LL_OPCODE_DEL)
				rmnet_shs_remove_llflow(flow_info);
			rmnet_shs_genl_send_int_to_userspace(info,
						RMNET_SHS_SEG_SET_RESP_INT);
		} else {
			rm_err("SHS_GNL: nla_memcpy failed %d\n",
			       RMNET_SHS_GENL_ATTR_FLOW);
			rmnet_shs_genl_send_int_to_userspace(info,
					RMNET_SHS_SEG_FAIL_RESP_INT);
			return 0;
		}
	} else {
		rm_err("SHS_GNL: no info->attrs %d\n",
		       RMNET_SHS_GENL_ATTR_FLOW);
		rmnet_shs_genl_send_int_to_userspace(info,
				RMNET_SHS_SEG_FAIL_RESP_INT);
		return 0;
	}

	return 0;
}

int rmnet_shs_genl_set_bootup_config(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_shs_bootup_info bootup_info;
	int i;

	rm_err("%s %s", "SHS_GNL: ", __func__);

	if (info == NULL) {
		rm_err("%s", "SHS_GNL: an error occured - info is null");
		return -1;
	}

	na = info->attrs[RMNET_SHS_GENL_ATTR_BOOTUP];
	if (na) {
		if (nla_memcpy(&bootup_info, na, sizeof(bootup_info)) > 0) {

			rmnet_shs_cfg.non_perf_mask = bootup_info.non_perf_mask;
			rmnet_shs_cfg.max_s_cores = hweight_long(bootup_info.non_perf_mask);
			rmnet_shs_cfg.perf_mask = ~rmnet_shs_cfg.non_perf_mask;
			rmnet_shs_cfg.feature_mask = bootup_info.feature_mask;
			rmnet_shs_cfg.cpu_freq_boost_val = bootup_info.cpu_freq_boost_val;

			/* Requires shsusrd to enable */
			if (rmnet_shs_cfg.feature_mask & INST_RX_SWTCH_FEAT) {
				rmnet_shs_inst_rate_switch = 1;
			}

			rm_err("SHS_GNL: bootup req "
			       "feature_mask = 0x%x non_perfmaxk = 0x%x, perf_mask 0x%x",
			       bootup_info.feature_mask,
			       rmnet_shs_cfg.non_perf_mask,
				   rmnet_shs_cfg.perf_mask);
			for(i = 0; i < MAX_CPUS; i++)
			{
				rmnet_shs_cpu_rx_min_pps_thresh[i] = bootup_info.rx_min_pps_thresh[i];
				rmnet_shs_cpu_rx_max_pps_thresh[i] = bootup_info.rx_max_pps_thresh[i];
				rm_err("SHS_GNL: bootup %i req %llu %llu", i,
						rmnet_shs_cpu_rx_min_pps_thresh[i],
						rmnet_shs_cpu_rx_max_pps_thresh[i] );
			}
			rmnet_shs_genl_send_int_to_userspace(info,
					RMNET_SHS_BOOT_SET_RESP_INT);

		} else {
			rm_err("SHS_GNL: nla_memcpy failed %d\n",
			       RMNET_SHS_GENL_ATTR_BOOTUP);
			rmnet_shs_genl_send_int_to_userspace(info,
					RMNET_SHS_BOOT_FAIL_RESP_INT);
			return 0;
		}
	} else {
		rm_err("SHS_GNL: no info->attrs %d\n",
		       RMNET_SHS_GENL_ATTR_BOOTUP);
		rmnet_shs_genl_send_int_to_userspace(info,
				RMNET_SHS_BOOT_FAIL_RESP_INT);
		return 0;
	}

	return 0;
}
int rmnet_shs_genl_set_flow_segmentation(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_shs_wq_seg_info seg_info;
	int rc = 0;

	rm_err("%s", "SHS_GNL: rmnet_shs_genl_set_flow_segmentation");

	if (info == NULL) {
		rm_err("%s", "SHS_GNL: an error occured - info is null");
		return -1;
	}

	na = info->attrs[RMNET_SHS_GENL_ATTR_SEG];
	if (na) {
		if (nla_memcpy(&seg_info, na, sizeof(seg_info)) > 0) {
			rm_err("SHS_GNL: recv segmentation req "
			       "hash_to_set = 0x%x segs_per_skb = %u",
			       seg_info.hash_to_set,
			       seg_info.segs_per_skb);

			rc = rmnet_shs_wq_set_flow_segmentation(seg_info.hash_to_set,
								seg_info.segs_per_skb);

			if (rc == 1) {
				rmnet_shs_genl_send_int_to_userspace(info,
						RMNET_SHS_SEG_SET_RESP_INT);
				trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR,
					RMNET_SHS_WQ_FLOW_SEG_SET_PASS,
					seg_info.hash_to_set, seg_info.segs_per_skb,
					0xDEF, 0xDEF, NULL, NULL);
			} else {
				rmnet_shs_genl_send_int_to_userspace(info,
						RMNET_SHS_SEG_FAIL_RESP_INT);
				trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR,
					RMNET_SHS_WQ_FLOW_SEG_SET_FAIL,
					seg_info.hash_to_set, seg_info.segs_per_skb,
					0xDEF, 0xDEF, NULL, NULL);
				return 0;
			}
		} else {
			rm_err("SHS_GNL: nla_memcpy failed %d\n",
			       RMNET_SHS_GENL_ATTR_SEG);
			rmnet_shs_genl_send_int_to_userspace(info,
					RMNET_SHS_SEG_FAIL_RESP_INT);
			return 0;
		}
	} else {
		rm_err("SHS_GNL: no info->attrs %d\n",
		       RMNET_SHS_GENL_ATTR_SEG);
		rmnet_shs_genl_send_int_to_userspace(info,
				RMNET_SHS_SEG_FAIL_RESP_INT);
		return 0;
	}

	return 0;
}

int rmnet_shs_genl_set_quickack_thresh(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_shs_wq_quickack_info quickack_info;
	int rc = 0;

	rm_err("%s", "SHS_GNL: rmnet_shs_genl_set_quickack_thresh");

	if (info == NULL) {
		rm_err("%s", "SHS_GNL: an error occured - info is null");
		return -1;
	}

	na = info->attrs[RMNET_SHS_GENL_ATTR_QUICKACK];
	if (na) {
		if (nla_memcpy(&quickack_info, na, sizeof(quickack_info)) > 0) {
			rm_err("SHS_GNL: recv quickack req "
			       "hash_to_set = 0x%x thresh = %u",
			       quickack_info.hash_to_set,
			       quickack_info.ack_thresh);

			rc = rmnet_shs_wq_set_quickack_thresh(quickack_info.hash_to_set,
							      quickack_info.ack_thresh);

			if (rc == 1) {
				rmnet_shs_genl_send_int_to_userspace(info,
						RMNET_SHS_QUICKACK_SET_RESP_INT);
			} else {
				rmnet_shs_genl_send_int_to_userspace(info,
						RMNET_SHS_QUICKACK_FAIL_RESP_INT);
				return 0;
			}
		} else {
			rm_err("SHS_GNL: nla_memcpy failed %d\n",
			       RMNET_SHS_GENL_ATTR_QUICKACK);
			rmnet_shs_genl_send_int_to_userspace(info,
					RMNET_SHS_QUICKACK_FAIL_RESP_INT);
			return 0;
		}
	} else {
		rm_err("SHS_GNL: no info->attrs %d\n",
		       RMNET_SHS_GENL_ATTR_QUICKACK);
		rmnet_shs_genl_send_int_to_userspace(info,
				RMNET_SHS_QUICKACK_FAIL_RESP_INT);
		return 0;
	}

	return 0;
}


int rmnet_shs_genl_try_to_move_flow(struct sk_buff *skb_2, struct genl_info *info)
{
	struct nlattr *na;
	struct rmnet_shs_wq_sugg_info sugg_info;
	int rc = 0;

	rm_err("%s", "SHS_GNL: rmnet_shs_genl_try_to_move_flow");

	if (info == NULL) {
		rm_err("%s", "SHS_GNL: an error occured - info is null");
		return -1;
	}

	na = info->attrs[RMNET_SHS_GENL_ATTR_SUGG];
	if (na) {
		if (nla_memcpy(&sugg_info, na, sizeof(sugg_info)) > 0) {
			rm_err("SHS_GNL: cur_cpu =%u dest_cpu = %u "
			       "hash_to_move = 0x%x sugg_type = %u",
			       sugg_info.cur_cpu,
			       sugg_info.dest_cpu,
			       sugg_info.hash_to_move,
			       sugg_info.sugg_type);
			if (sugg_info.dest_cpu >= MAX_CPUS || sugg_info.cur_cpu >= MAX_CPUS) {
				rmnet_shs_mid_err[RMNET_SHS_MALFORM_MOVE]++;
				rmnet_shs_genl_send_int_to_userspace(info, RMNET_SHS_RMNET_MOVE_FAIL_RESP_INT);
				return -1;
			}

			if (sugg_info.sugg_type == RMNET_SHS_WQ_SUGG_LL_FLOW_CORE) {
				rmnet_shs_ll_flow_cpu = sugg_info.dest_cpu;
				trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR, RMNET_SHS_WQ_TRY_PASS,
				   sugg_info.cur_cpu, sugg_info.dest_cpu,
				   sugg_info.hash_to_move, sugg_info.sugg_type, NULL, NULL);

				rmnet_shs_genl_send_int_to_userspace(info,RMNET_SHS_RMNET_MOVE_DONE_RESP_INT);
				return 0;
			}
			if (sugg_info.sugg_type == RMNET_SHS_WQ_SUGG_LL_PHY_CORE) {
				rmnet_shs_ll_phy_cpu = sugg_info.dest_cpu;
				trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR, RMNET_SHS_WQ_TRY_PASS,
				   sugg_info.cur_cpu, sugg_info.dest_cpu,
				   sugg_info.hash_to_move, sugg_info.sugg_type, NULL, NULL);

				rmnet_shs_genl_send_int_to_userspace(info,RMNET_SHS_RMNET_MOVE_DONE_RESP_INT);
				return 0;
			}

			if (sugg_info.sugg_type == RMNET_SHS_WQ_SUGG_RMNET_TO_SILVER) {
				rmnet_shs_switch_reason[RMNET_SHS_PHY_SWITCH_GOLD_TO_S]++;

				/* Only drop to silver if given a gold core and phy core not already dropped*/
				if (!((1 << sugg_info.dest_cpu) & NONPERF_MASK)  ||
				    ((1 << rmnet_shs_cfg.phy_tcpu) & NONPERF_MASK )) {
						rmnet_shs_genl_send_int_to_userspace(info,RMNET_SHS_RMNET_MOVE_DONE_RESP_INT);
					return -1;
				}
				/* Dont move back down to core 1 if core 1 is reserved */
				if (((1 << sugg_info.dest_cpu) & rmnet_shs_halt_mask)) {
					rmnet_shs_switch_reason[RMNET_SHS_SUGG_R2S_FAIL1]++;
					rmnet_shs_genl_send_int_to_userspace(info, RMNET_SHS_RMNET_MOVE_FAIL_RESP_INT);
					return -1;
				}

				rmnet_shs_cfg.phy_tcpu = sugg_info.dest_cpu;
				rcu_read_lock();
				rmnet_shs_switch_enable();
				rcu_read_unlock();

				rmnet_shs_genl_send_int_to_userspace(info,RMNET_SHS_RMNET_MOVE_DONE_RESP_INT);
				return 0;
			}

			if (sugg_info.sugg_type == RMNET_SHS_WQ_SUGG_RMNET_TO_GOLD) {
				rmnet_shs_switch_reason[RMNET_SHS_PHY_SWITCH_SILVER_TO_G]++;

				/* Only ramp to gold  if given a gold core and phy cpu is not already ramped*/
				if (!((1 << sugg_info.dest_cpu) & PERF_MASK)) {
					rmnet_shs_genl_send_int_to_userspace(info,  RMNET_SHS_RMNET_MOVE_DONE_RESP_INT);
					return -1;
				}

				if (((1 << sugg_info.dest_cpu) & rmnet_shs_halt_mask)) {
					rmnet_shs_switch_reason[RMNET_SHS_SUGG_R2G_FAIL1]++;
					rmnet_shs_genl_send_int_to_userspace(info, RMNET_SHS_RMNET_MOVE_FAIL_RESP_INT);
					return -1;
				}

				/* If dest is already current cpu exit */
				if ((rmnet_shs_cfg.phy_acpu) == sugg_info.dest_cpu &&
				    (rmnet_shs_cfg.phy_tcpu) == sugg_info.dest_cpu) {
					rmnet_shs_genl_send_int_to_userspace(info,  RMNET_SHS_RMNET_MOVE_DONE_RESP_INT);
					return 0;
				}

				if (((1 << sugg_info.dest_cpu) & PERF_MASK) && ((1 << rmnet_shs_cfg.phy_acpu) & PERF_MASK)) {
					rmnet_shs_switch_reason[RMNET_SHS_RM2G_G2G_SWITCH]++;
				}


				rmnet_shs_cfg.phy_tcpu = sugg_info.dest_cpu;
				rcu_read_lock();
				rmnet_shs_switch_enable();
				rcu_read_unlock();

				rmnet_shs_genl_send_int_to_userspace(info,RMNET_SHS_RMNET_MOVE_DONE_RESP_INT);
				return 0;
			}

			rc = rmnet_shs_wq_try_to_move_flow(sugg_info.cur_cpu,
							   sugg_info.dest_cpu,
							   sugg_info.hash_to_move,
							   sugg_info.sugg_type);
			if (rc == 1) {
				rmnet_shs_genl_send_int_to_userspace(info,
						RMNET_SHS_MOVE_PASS_RESP_INT);
				trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR, RMNET_SHS_WQ_TRY_PASS,
				   sugg_info.cur_cpu, sugg_info.dest_cpu,
				   sugg_info.hash_to_move, sugg_info.sugg_type, NULL, NULL);

			} else {
				rmnet_shs_genl_send_int_to_userspace(info,
						RMNET_SHS_MOVE_FAIL_RESP_INT);
				trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR, RMNET_SHS_WQ_TRY_FAIL,
				   sugg_info.cur_cpu, sugg_info.dest_cpu,
				   sugg_info.hash_to_move, sugg_info.sugg_type, NULL, NULL);
				return 0;
			}
		} else {
			rm_err("SHS_GNL: nla_memcpy failed %d\n",
			       RMNET_SHS_GENL_ATTR_SUGG);
			rmnet_shs_genl_send_int_to_userspace(info,
					RMNET_SHS_MOVE_FAIL_RESP_INT);
			return 0;
		}
	} else {
		rm_err("SHS_GNL: no info->attrs %d\n",
		       RMNET_SHS_GENL_ATTR_SUGG);
		rmnet_shs_genl_send_int_to_userspace(info,
				RMNET_SHS_MOVE_FAIL_RESP_INT);
		return 0;
	}

	return 0;
}

int rmnet_shs_genl_mem_sync(struct sk_buff *skb_2, struct genl_info *info)
{
	rm_err("%s", "SHS_GNL: rmnet_shs_genl_mem_sync");

	if (!rmnet_shs_userspace_connected)
		rmnet_shs_userspace_connected = 1;

	/* Todo: detect when userspace is disconnected. If we dont get
	 * a sync message in the next 2 wq ticks, we got disconnected
	 */

	trace_rmnet_shs_wq_high(RMNET_SHS_WQ_SHSUSR, RMNET_SHS_WQ_SHSUSR_SYNC_START,
				0xDEF, 0xDEF, 0xDEF, 0xDEF, NULL, NULL);

	if (info == NULL) {
		rm_err("%s", "SHS_GNL: an error occured - info is null");
		return -1;
	}

	last_net = genl_info_net(info);
	last_snd_portid = info->snd_portid;

	rm_err("port_id = %u", last_snd_portid);
	return 0;
}


void rmnet_shs_create_ping_boost_msg_resp(uint32_t perf_duration,
					  struct rmnet_shs_msg_resp *msg_resp)
{
	struct rmnet_shs_ping_boost_payload ping_boost_msg;
	struct timespec64 time;

	if (msg_resp == NULL) {
		rm_err("%s", "SHS_MSG_GNL - invalid input");
		return;
	}

	memset(msg_resp, 0x0, sizeof(struct rmnet_shs_msg_resp));
	memset(&ping_boost_msg, 0x0, sizeof(ping_boost_msg));

	ktime_get_real_ts64(&time);
	msg_resp->timestamp = (RMNET_SHS_GENL_SEC_TO_NSEC(time.tv_sec) + time.tv_nsec);
	ping_boost_msg.perf_duration = perf_duration;
	ping_boost_msg.perf_acq = 1;

	/* Copy to boost info into to the payload of first msg */
	memcpy(&(msg_resp->list[0].payload),
	       &ping_boost_msg, sizeof(ping_boost_msg));
	msg_resp->list[0].msg_type = RMNET_SHS_GENL_PING_BOOST_MSG;

	msg_resp->valid = 1;
	msg_resp->list_len = 1;
}


void rmnet_shs_create_pause_msg_resp(uint8_t seq,
					  struct rmnet_shs_msg_resp *msg_resp)
{
	struct rmnet_shs_pause_payload pause_msg;

	if (msg_resp == NULL) {
		rm_err("%s", "SHS_MSG_GNL - invalid input");
		return;
	}

	memset(msg_resp, 0x0, sizeof(struct rmnet_shs_msg_resp));
	memset(&pause_msg, 0x0, sizeof(pause_msg));

	/* Copy to boost info into to the payload of first msg */
	memcpy(&(msg_resp->list[0].payload),
	       &pause_msg, sizeof(pause_msg));
	msg_resp->list[0].msg_type = RMNET_SHS_GENL_TRAFFIC_PAUSE_MSG;

	msg_resp->valid = 1;
	msg_resp->list_len = 1;
}

void rmnet_shs_create_phy_msg_resp(struct rmnet_shs_msg_resp *msg_resp,
                                  uint8_t ocpu, uint8_t ncpu)
{
	struct rmnet_shs_phy_change_payload phy_change_msg;
	struct timespec64 time;

	if (msg_resp == NULL) {
		rm_err("%s", "SHS_MSG_GNL - invalid input");
		return;
	}

	memset(msg_resp, 0x0, sizeof(struct rmnet_shs_msg_resp));
	memset(&phy_change_msg, 0x0, sizeof(phy_change_msg));

	ktime_get_real_ts64(&time);
	msg_resp->timestamp = (RMNET_SHS_GENL_SEC_TO_NSEC(time.tv_sec) + time.tv_nsec);
	phy_change_msg.old_cpu = ocpu;
	phy_change_msg.new_cpu = ncpu;

	/* Copy to boost info into to the payload of first msg */
	memcpy(&(msg_resp->list[0].payload),
	       &phy_change_msg, sizeof(phy_change_msg));
	msg_resp->list[0].msg_type = RMNET_SHS_GENL_PHY_CHANGE_MSG;

	msg_resp->valid = 1;
	msg_resp->list_len = 1;
}

int rmnet_shs_genl_msg_direct_send_to_userspace(struct rmnet_shs_msg_resp *msg_ptr)
{
	struct sk_buff *skb;
	void *msg_head;
	int rc;

	if (msg_last_net == NULL) {
		rm_err("%s", "SHS_GNL: FAILED to send msg_last_net is NULL\n");
		return -1;
	}

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (skb == NULL)
		goto out;

	msg_head = genlmsg_put(skb, 0, rmnet_shs_genl_msg_seqnum++,
			       &rmnet_shs_genl_msg_family,
			       0, RMNET_SHS_GENL_MSG_WAIT_CMD);
	if (msg_head == NULL) {
		rc = -ENOMEM;
		rm_err("SHS_GNL: FAILED to msg_head %d\n", rc);
		kfree(skb);
		goto out;
	}
	rc = nla_put(skb, RMNET_SHS_GENL_MSG_ATTR_RESP,
		     sizeof(struct rmnet_shs_msg_resp),
		     msg_ptr);
	if (rc != 0) {
		rm_err("SHS_GNL: FAILED nla_put %d\n", rc);
		kfree(skb);
		goto out;
	}

	genlmsg_end(skb, msg_head);

	rc = genlmsg_unicast(msg_last_net, skb, msg_last_snd_portid);
	if (rc != 0)
		goto out;

	rm_err("SHS_MSG_GNL: Successfully sent msg %d\n",
	       rmnet_shs_genl_msg_seqnum);
	return 0;

out:
	rm_err("%s", "SHS_GNL: FAILED to send to msg channel\n");
	return -1;
}

/* Handler for message channel to shsusrd */
int rmnet_shs_genl_msg_req_hdlr(struct sk_buff *skb_2,
				struct genl_info *info)
{
	rm_err("%s", "SHS_GNL_MSG: rmnet_shs_genl_msg_req");

	if (!rmnet_shs_userspace_connected) {
		rm_err("%s", "SHS_GNL_MSG: error: userspace not connected");
		return -1;
	}

	if (info == NULL) {
		rm_err("%s", "SHS_GNL_MSG: error: info is null");
		return -1;
	}

	msg_last_net = genl_info_net(info);
	msg_last_snd_portid = info->snd_portid;

	rm_err("msg_port_id = %u", msg_last_snd_portid);
	return 0;
}

/* register new generic netlink family */
int rmnet_shs_wq_genl_init(void)
{
	int ret;

	rmnet_shs_userspace_connected = 0;
	ret = genl_register_family(&rmnet_shs_genl_family);
	if (ret != 0) {
		rm_err("SHS_GNL: register family failed: %i", ret);
		genl_unregister_family(&rmnet_shs_genl_family);
		return -1;
	}

	rm_err("SHS_GNL: successfully registered generic netlink family: %s",
	       RMNET_SHS_GENL_FAMILY_NAME);

	ret = genl_register_family(&rmnet_shs_genl_msg_family);
	if (ret != 0) {
		rm_err("SHS_MSG_GNL: register family failed: %i", ret);
		genl_unregister_family(&rmnet_shs_genl_msg_family);
	} else {
		rm_err("SHS_MSG_GNL: successfully registered generic netlink family: %s",
		       RMNET_SHS_GENL_MSG_FAMILY_NAME);
	}

	return 0;
}

/* Unregister the generic netlink family */
int rmnet_shs_wq_genl_deinit(void)
{
	int ret;

	rmnet_shs_genl_send_int_to_userspace_no_info(RMNET_SHS_SYNC_WQ_EXIT);

	ret = genl_unregister_family(&rmnet_shs_genl_family);
	if(ret != 0){
		rm_err("SHS_GNL: unregister family failed: %i\n",ret);
	}
	rmnet_shs_userspace_connected = 0;

	ret = genl_unregister_family(&rmnet_shs_genl_msg_family);
	if(ret != 0){
		rm_err("SHS_GNL: unregister family failed: %i\n", ret);
	}
	return 0;
}
