// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "rmnet_aps_genl.h"
#include "rmnet_aps.h"

/* Static Functions and Definitions */
static struct nla_policy rmnet_aps_genl_attr_policy[RMNET_APS_GENL_ATTR_MAX +
						    1] = {
		[RMNET_APS_GENL_ATTR_FLOW_REQ] =
			NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_aps_flow_req)),
		[RMNET_APS_GENL_ATTR_FLOW_RESP] = NLA_POLICY_EXACT_LEN(
			sizeof(struct rmnet_aps_flow_resp)),
		[RMNET_APS_GENL_ATTR_PDN_CONFIG_REQ] = NLA_POLICY_EXACT_LEN(
			sizeof(struct rmnet_aps_pdn_config_req)),
		[RMNET_APS_GENL_ATTR_PDN_CONFIG_RESP] = NLA_POLICY_EXACT_LEN(
			sizeof(struct rmnet_aps_pdn_config_resp)),
		[RMNET_APS_GENL_ATTR_FILTER_REQ] = NLA_POLICY_EXACT_LEN(
			sizeof(struct rmnet_aps_filter_req)),
		[RMNET_APS_GENL_ATTR_FILTER_RESP] = NLA_POLICY_EXACT_LEN(
			sizeof(struct rmnet_aps_filter_resp)),
		[RMNET_APS_GENL_ATTR_DATA_REPORT] = NLA_POLICY_EXACT_LEN(
			sizeof(struct rmnet_aps_data_report)),
};

#define RMNET_APS_GENL_OP(_cmd, _func)                                         \
	{                                                                      \
		.cmd = _cmd, .doit = _func, .dumpit = NULL, .flags = 0,        \
	}

static const struct genl_ops rmnet_aps_genl_ops[] = {
	RMNET_APS_GENL_OP(RMNET_APS_GENL_CMD_FLOW, rmnet_aps_genl_flow_hdlr),
	RMNET_APS_GENL_OP(RMNET_APS_GENL_CMD_PDN_CONFIG,
			  rmnet_aps_genl_pdn_config_hdlr),
	RMNET_APS_GENL_OP(RMNET_APS_GENL_CMD_FILTER,
			  rmnet_aps_genl_filter_hdlr),
	RMNET_APS_GENL_OP(RMNET_APS_GENL_CMD_DATA_REPORT,
			  rmnet_aps_genl_data_report_hdlr),
};

struct genl_family rmnet_aps_genl_family = {
	.hdrsize = 0,
	.name = RMNET_APS_GENL_FAMILY_NAME,
	.version = RMNET_APS_GENL_VERSION,
	.maxattr = RMNET_APS_GENL_ATTR_MAX,
	.policy = rmnet_aps_genl_attr_policy,
	.ops = rmnet_aps_genl_ops,
	.n_ops = ARRAY_SIZE(rmnet_aps_genl_ops),
};

/* register new generic netlink family */
int rmnet_aps_genl_init(void)
{
	return genl_register_family(&rmnet_aps_genl_family);
}

/* Unregister the generic netlink family */
void rmnet_aps_genl_deinit(void)
{
	genl_unregister_family(&rmnet_aps_genl_family);
}
