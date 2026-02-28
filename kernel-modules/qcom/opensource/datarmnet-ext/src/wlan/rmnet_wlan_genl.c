// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN Generic Netlink */

#include <net/genetlink.h>
#include <net/netlink.h>
#include <linux/module.h>
#include <linux/if.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include "rmnet_wlan_genl.h"
#include "rmnet_wlan.h"
#include "rmnet_wlan_connection.h"
#include "rmnet_wlan_fragment.h"

/* Use integer 58 instead of ':' to avoid issues with scripts */
#define RMNET_WLAN_CHAR_COLON 58

static struct nla_policy
rmnet_wlan_genl_tuple_policy[RMNET_WLAN_GENL_TUPLE_ATTR_MAX + 1] = {
	[RMNET_WLAN_GENL_TUPLE_ATTR_TUPLE] =
		NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_wlan_tuple)),
};

static struct nla_policy
rmnet_wlan_genl_attr_policy[RMNET_WLAN_GENL_ATTR_MAX + 1] = {
	[RMNET_WLAN_GENL_ATTR_TUPLES] =
		NLA_POLICY_NESTED_ARRAY(rmnet_wlan_genl_tuple_policy),
	[RMNET_WLAN_GENL_ATTR_DEV] = {
		.type = NLA_NUL_STRING,
		.len = IFNAMSIZ - 1, /* Max len excluding NULL */
	},
	[RMNET_WLAN_GENL_ATTR_FWD_ADDR] = {
		.type = NLA_NUL_STRING,
		.len = INET6_ADDRSTRLEN,
	},
	[RMNET_WLAN_GENL_ATTR_FWD_DEV] = {
		.type = NLA_NUL_STRING,
		.len = IFNAMSIZ - 1, /* Max len excluding NULL */
	},
	[RMNET_WLAN_GENL_ATTR_ENCAP_PORT] = {
		.type = NLA_U16,
	},
	[RMNET_WLAN_GENL_ATTR_NET_TYPE] = {
		.type = NLA_U8,
	},
	[RMNET_WLAN_GENL_ATTR_LL_SRC_ADDR] = {
		.type = NLA_NUL_STRING,
		.len = INET6_ADDRSTRLEN,
	},
	[RMNET_WLAN_GENL_ATTR_LL_DST_ADDR] = {
		.type = NLA_NUL_STRING,
		.len = INET6_ADDRSTRLEN,
	},
	[RMNET_WLAN_GENL_ATTR_LL_SRC_PORT] = {
		.type = NLA_U16,
	},
	[RMNET_WLAN_GENL_ATTR_LL_DST_PORT] = {
		.type = NLA_U16,
	},
};

#define RMNET_WLAN_GENL_OP(_cmd, _func) \
	{ \
		.cmd = _cmd, \
		.doit = _func, \
	}

static struct genl_family rmnet_wlan_genl_family;

static int rmnet_wlan_genl_add_tuples(struct sk_buff *skb,
				      struct genl_info *info)
{
	struct nlattr *tb[RMNET_WLAN_GENL_TUPLE_ATTR_MAX + 1];
	struct rmnet_wlan_tuple *tuples;
	struct nlattr *nla;
	u32 tuple_count = 0;
	int tuple_len;
	int rc = 0;

	if (!info->attrs[RMNET_WLAN_GENL_ATTR_TUPLES]) {
		/* Help me help you! */
		GENL_SET_ERR_MSG(info, "Must supply tuple info");
		return -EINVAL;
	}

	nla_for_each_nested(nla, info->attrs[RMNET_WLAN_GENL_ATTR_TUPLES],
			    tuple_len)
		tuple_count++;

	tuples = kcalloc(tuple_count, sizeof(*tuples), GFP_KERNEL);
	if (!tuples) {
		GENL_SET_ERR_MSG(info, "Kernel OOM");
		return -ENOMEM;
	}

	tuple_count = 0;
	nla_for_each_nested(nla, info->attrs[RMNET_WLAN_GENL_ATTR_TUPLES],
			    tuple_len) {
		struct rmnet_wlan_tuple *tuple;

		rc = nla_parse_nested(tb, RMNET_WLAN_GENL_TUPLE_ATTR_MAX, nla,
				      rmnet_wlan_genl_tuple_policy,
				      info->extack);
		if (rc)
			goto out;

		if (!tb[RMNET_WLAN_GENL_TUPLE_ATTR_TUPLE]) {
			GENL_SET_ERR_MSG(info, "Must specify tuple entry");
			goto out;
		}

		/* Sanitize. It's 2020 after all.
		 *
		 * ...Too soon?
		 */
		tuple = nla_data(tb[RMNET_WLAN_GENL_TUPLE_ATTR_TUPLE]);
		if (tuple->ip_proto != 4 && tuple->ip_proto != 6) {
			GENL_SET_ERR_MSG(info, "Invalid IP protocol");
			goto out;
		}

		if (tuple->trans_proto != IPPROTO_TCP &&
		    tuple->trans_proto != IPPROTO_UDP &&
		    tuple->trans_proto != IPPROTO_ESP) {
			GENL_SET_ERR_MSG(info, "Invalid transport protocol");
			goto out;
		}

		memcpy(&tuples[tuple_count], tuple, sizeof(*tuple));
		tuple_count++;
	}

	rc = rmnet_wlan_add_tuples(tuples, tuple_count, info);

out:
	kfree(tuples);
	return rc;
}

static int rmnet_wlan_genl_del_tuples(struct sk_buff *skb,
				      struct genl_info *info)
{
	struct nlattr *tb[RMNET_WLAN_GENL_TUPLE_ATTR_MAX + 1];
	struct rmnet_wlan_tuple *tuples;
	struct nlattr *nla;
	u32 tuple_count = 0;
	int tuple_len;
	int rc;

	if (!info->attrs[RMNET_WLAN_GENL_ATTR_TUPLES]) {
		GENL_SET_ERR_MSG(info, "Must supply tuple info");
		return -EINVAL;
	}

	nla_for_each_nested(nla, info->attrs[RMNET_WLAN_GENL_ATTR_TUPLES],
			    tuple_len)
		tuple_count++;

	tuples = kcalloc(tuple_count, sizeof(*tuples), GFP_KERNEL);
	if (!tuples) {
		GENL_SET_ERR_MSG(info, "Kernel OOM");
		return -ENOMEM;
	}

	tuple_count = 0;
	nla_for_each_nested(nla, info->attrs[RMNET_WLAN_GENL_ATTR_TUPLES],
			    tuple_len) {
		struct rmnet_wlan_tuple *tuple;

		rc = nla_parse_nested(tb, RMNET_WLAN_GENL_TUPLE_ATTR_MAX, nla,
				      rmnet_wlan_genl_tuple_policy,
				      info->extack);
		if (rc)
			goto out;

		if (!tb[RMNET_WLAN_GENL_TUPLE_ATTR_TUPLE]) {
			GENL_SET_ERR_MSG(info, "Must specify tuple entry");
			rc = -EINVAL;
			goto out;
		}

		tuple = nla_data(tb[RMNET_WLAN_GENL_TUPLE_ATTR_TUPLE]);
		memcpy(&tuples[tuple_count], tuple, sizeof(*tuple));
		tuple_count++;
	}

	rc = rmnet_wlan_del_tuples(tuples, tuple_count, info);

out:
	kfree(tuples);
	return rc;
}

static int rmnet_wlan_genl_set_device(struct sk_buff *skb,
				      struct genl_info *info)
{
	struct nlattr *nla;
	int net_type;
	int err;

	if (!info->attrs[RMNET_WLAN_GENL_ATTR_DEV] ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]) {
		GENL_SET_ERR_MSG(info, "Must specify device and network info");
		return -EINVAL;
	}

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_DEV];
	net_type = nla_get_u8(info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]);

	if(net_type != DATA_PATH_PROXY_NET_WLAN &&
	   net_type != DATA_PATH_PROXY_NET_WWAN &&
	   net_type != DATA_PATH_PROXY_NET_LBO) {
		GENL_SET_ERR_MSG(info, "Network type not supported!");
		return -EINVAL;
	}

	if(net_type == DATA_PATH_PROXY_NET_WWAN) {
		err = rmnet_wwan_set_device(nla_data(nla), info);
	} else {
		err = rmnet_wlan_set_device(nla_data(nla), info);
	}

	return err;
}

static int rmnet_wlan_genl_unset_device(struct sk_buff *skb,
					struct genl_info *info)
{
	struct nlattr *nla;
	int net_type;
	int err;

	if(!info->attrs[RMNET_WLAN_GENL_ATTR_DEV] ||
	   !info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]) {
		GENL_SET_ERR_MSG(info,
				 "Kernel error, unregistering notifier failed");
		return -EINVAL;
	}
	net_type = nla_get_u8(info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]);

    /* Still don't care about you */
	nla = info->attrs[RMNET_WLAN_GENL_ATTR_DEV];

	if(net_type != DATA_PATH_PROXY_NET_WLAN &&
	   net_type != DATA_PATH_PROXY_NET_WWAN &&
	   net_type != DATA_PATH_PROXY_NET_LBO) {
		GENL_SET_ERR_MSG(info, "Network type not supported!");
		return -EINVAL;
	}

	if(net_type == DATA_PATH_PROXY_NET_WWAN) {
		err = rmnet_wwan_unset_device(nla_data(nla), info);
	} else {
		err = rmnet_wlan_unset_device(nla_data(nla), info);
	}

	if (err)
		GENL_SET_ERR_MSG(info,
				 "Kernel error, unregistering notifier failed");

	return err;
}

static int rmnet_wlan_genl_add_fwd_info(struct sk_buff *skb,
					struct genl_info *info)
{
	struct rmnet_wlan_fwd_info fwd_info = {};
	struct nlattr *nla;
	char *addr_str;
	int err;

	/* Must provide the address and device to forward to */
	if (!info->attrs[RMNET_WLAN_GENL_ATTR_FWD_ADDR] ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_FWD_DEV]  ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]) {
		GENL_SET_ERR_MSG(info,
				 "Must specify FWD device, address, and network");
		return -EINVAL;
	}

	fwd_info.net_type = nla_get_u8(info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]);

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_FWD_ADDR];
	addr_str = nla_data(nla);
	if (strnchr(addr_str, nla_len(nla), RMNET_WLAN_CHAR_COLON)) {
		if (in6_pton(addr_str, nla_len(nla),
			     fwd_info.v6_addr.s6_addr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "FWD address is invalid in IPv6");
			return -EINVAL;
		}

		fwd_info.ip_proto = 6;
	} else {
		if (in4_pton(addr_str, nla_len(nla),
			     (u8 *)&fwd_info.v4_addr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "FWD address is invalid in IPv4");
			return -EINVAL;
		}

		fwd_info.ip_proto = 4;
	}

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_FWD_DEV];
	fwd_info.fwd_dev = dev_get_by_name(genl_info_net(info), nla_data(nla));
	if (!fwd_info.fwd_dev) {
		GENL_SET_ERR_MSG(info, "Invalid FWD device name");
		return -EINVAL;
	}

	err = rmnet_wlan_add_fwd_info(&fwd_info, info);
	dev_put(fwd_info.fwd_dev);
	return err;
}

static int rmnet_wlan_genl_del_fwd_info(struct sk_buff *skb,
					struct genl_info *info)
{
	struct rmnet_wlan_fwd_info fwd_info = {};
	struct nlattr *nla;
	char *addr_str;
	int err;

	/* Must provide the address and device to forward to */
	if (!info->attrs[RMNET_WLAN_GENL_ATTR_FWD_ADDR] ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_FWD_DEV]  ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]) {
		GENL_SET_ERR_MSG(info,
				 "Must specify FWD device and address");
		return -EINVAL;
	}

	fwd_info.net_type = nla_get_u8(info->attrs[RMNET_WLAN_GENL_ATTR_NET_TYPE]);

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_FWD_ADDR];
	addr_str = nla_data(nla);
	if (strnchr(addr_str, nla_len(nla), RMNET_WLAN_CHAR_COLON)) {
		if (in6_pton(addr_str, nla_len(nla),
			     fwd_info.v6_addr.s6_addr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "FWD address is invalid in IPv6");
			return -EINVAL;
		}

		fwd_info.ip_proto = 6;
	} else {
		if (in4_pton(addr_str, nla_len(nla),
			     (u8 *)&fwd_info.v4_addr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "FWD address is invalid in IPv4");
			return -EINVAL;
		}

		fwd_info.ip_proto = 4;
	}

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_FWD_DEV];
	fwd_info.fwd_dev = dev_get_by_name(genl_info_net(info), nla_data(nla));
	if (!fwd_info.fwd_dev) {
		GENL_SET_ERR_MSG(info, "Invalid FWD device name");
		return -EINVAL;
	}

	err = rmnet_wlan_del_fwd_info(&fwd_info, info);
	dev_put(fwd_info.fwd_dev);
	return err;
}

static int rmnet_wlan_genl_set_encap_port(struct sk_buff *skb,
					  struct genl_info *info)
{
	struct nlattr *nla;

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_ENCAP_PORT];
	if (!nla) {
		GENL_SET_ERR_MSG(info, "Must specify encap port");
		return -EINVAL;
	}

	return rmnet_wlan_set_encap_port(nla_get_be16(nla), info);
}

static int rmnet_wlan_genl_unset_encap_port(struct sk_buff *skb,
					    struct genl_info *info)
{
	struct nlattr *nla;

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_ENCAP_PORT];
	if (!nla) {
		GENL_SET_ERR_MSG(info, "Must specify encap port");
		return -EINVAL;
	}

	return rmnet_wlan_unset_encap_port(nla_get_be16(nla), info);
}

static int rmnet_wlan_genl_act_encap_port_pass_through(struct sk_buff *skb,
						       struct genl_info *info)
{
	struct nlattr *nla;

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_ENCAP_PORT];
	if (!nla) {
		GENL_SET_ERR_MSG(info, "Must specify encap port");
		return -EINVAL;
	}

	return rmnet_wlan_act_encap_port_pass_through(nla_get_be16(nla), info);
}

static int rmnet_wlan_genl_act_encap_port_drop(struct sk_buff *skb,
					       struct genl_info *info)
{
	struct nlattr *nla;

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_ENCAP_PORT];
	if (!nla) {
		GENL_SET_ERR_MSG(info, "Must specify encap port");
		return -EINVAL;
	}

	return rmnet_wlan_act_encap_port_drop(nla_get_be16(nla), info);
}

static int rmnet_wlan_genl_reset(struct sk_buff *skb, struct genl_info *info)
{
	(void)skb;
	(void)info;

	rmnet_wlan_reset();
	return 0;
}

static int rmnet_wlan_genl_add_ll_tuple(struct sk_buff *skb,
					  struct genl_info *info)
{
	struct rmnet_wlan_ll_tuple tuple = {};
	struct nlattr *nla;
	char *addr_str;

	/* Must provide the saddr, daddr, sport, dport */
	if (!info->attrs[RMNET_WLAN_GENL_ATTR_LL_SRC_ADDR]  ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_LL_DST_ADDR]  ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_LL_SRC_PORT]  ||
	    !info->attrs[RMNET_WLAN_GENL_ATTR_LL_DST_PORT]) {
		GENL_SET_ERR_MSG(info,
				 "Must specify FWD device and address");
		return -EINVAL;
	}


	/* Set SRC address and set IPv4 or IPv6 Protocol */
	nla = info->attrs[RMNET_WLAN_GENL_ATTR_LL_SRC_ADDR];
	addr_str = nla_data(nla);
	if (strnchr(addr_str, nla_len(nla), RMNET_WLAN_CHAR_COLON)) {
		if (in6_pton(addr_str, nla_len(nla),
			     tuple.v6_saddr.s6_addr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "SRC address is invalid in IPv6");
			return -EINVAL;
		}

		tuple.ip_proto = 6;
	} else {
		if (in4_pton(addr_str, nla_len(nla),
			     (u8 *)&tuple.v4_saddr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "SRC address is invalid in IPv4");
			return -EINVAL;
		}

		tuple.ip_proto = 4;
	}

	/* Set DST address */
	nla = info->attrs[RMNET_WLAN_GENL_ATTR_LL_DST_ADDR];
	addr_str = nla_data(nla);
	if (strnchr(addr_str, nla_len(nla), RMNET_WLAN_CHAR_COLON)) {
		if (in6_pton(addr_str, nla_len(nla),
			     tuple.v6_daddr.s6_addr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "DST address is invalid in IPv6");
			return -EINVAL;
		}
	} else {
		if (in4_pton(addr_str, nla_len(nla),
			     (u8 *)&tuple.v4_daddr, -1, NULL) != 1) {
			GENL_SET_ERR_MSG(info,
					 "DST address is invalid in IPv4");
			return -EINVAL;
		}
	}

	/* Set Source and Destination Port */
	nla = info->attrs[RMNET_WLAN_GENL_ATTR_LL_SRC_PORT];
	tuple.sport = nla_get_be16(nla);

	nla = info->attrs[RMNET_WLAN_GENL_ATTR_LL_DST_PORT];
	tuple.dport = nla_get_be16(nla);

	rmnet_wlan_add_ll_tuple(&tuple);

	return 0;
}

static int rmnet_wlan_genl_del_ll_tuple(struct sk_buff *skb,
					  struct genl_info *info)
{
	(void)skb;
	(void)info;

	rmnet_wlan_del_ll_tuple();

	return 0;
}

static int rmnet_wlan_genl_get_tuple(struct sk_buff *skb,
				     struct genl_info *info)
{
	struct sk_buff *skb_out = NULL;
	int err = 0;

	/* Create a buffer and write the internal tuples */
	err = rmnet_wlan_get_tuples(&skb_out, &rmnet_wlan_genl_family, info);
	if (err)
		goto out;

	if (!skb_out) {
		err = -EINVAL;
		goto out;
	}

	genlmsg_reply(skb_out, info);
out:
	return err;
}

static const struct genl_ops rmnet_wlan_genl_ops[] = {
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_ADD_TUPLES,
			   rmnet_wlan_genl_add_tuples),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_DEL_TUPLES,
			   rmnet_wlan_genl_del_tuples),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_SET_DEV,
			   rmnet_wlan_genl_set_device),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_UNSET_DEV,
			   rmnet_wlan_genl_unset_device),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_ADD_FWD_INFO,
			   rmnet_wlan_genl_add_fwd_info),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_DEL_FWD_INFO,
			   rmnet_wlan_genl_del_fwd_info),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_SET_ENCAP_PORT,
			   rmnet_wlan_genl_set_encap_port),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_UNSET_ENCAP_PORT,
			   rmnet_wlan_genl_unset_encap_port),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_RESET,
			   rmnet_wlan_genl_reset),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_ENCAP_PORT_ACT_PASS_THROUGH,
			   rmnet_wlan_genl_act_encap_port_pass_through),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_ENCAP_PORT_ACT_DROP,
			   rmnet_wlan_genl_act_encap_port_drop),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_LL_ADDR_ADD,
			   rmnet_wlan_genl_add_ll_tuple),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_LL_ADDR_DEL,
			   rmnet_wlan_genl_del_ll_tuple),
	RMNET_WLAN_GENL_OP(RMNET_WLAN_GENL_CMD_GET_TUPLES,
			   rmnet_wlan_genl_get_tuple),
};

static struct genl_family rmnet_wlan_genl_family = {
	.name    = RMNET_WLAN_GENL_FAMILY_NAME,
	.version = RMNET_WLAN_GENL_VERSION,
	.maxattr = RMNET_WLAN_GENL_ATTR_MAX,
	.policy  = rmnet_wlan_genl_attr_policy,
	.ops     = rmnet_wlan_genl_ops,
	.n_ops   = ARRAY_SIZE(rmnet_wlan_genl_ops),
};

static int __init rmnet_wlan_genl_init(void)
{
	int ret = 0;

	pr_info("%s(): rmnet_wlan initializing\n", __func__);
	ret = genl_register_family(&rmnet_wlan_genl_family);
	if (ret) {
		pr_err("%s(): registering family failed: %i\n", __func__, ret);
		goto err0;
	}

	ret = rmnet_wlan_connection_init();
	if (ret) {
		pr_err("%s(): connection management init failed: %i\n", __func__, ret);
		goto err1;
	}

	ret = rmnet_wlan_fragment_init();
	if (ret) {
		pr_err("%s(): fragment management init failed: %i\n", __func__,
		       ret);
		goto err2;
	}

	rmnet_wlan_set_hooks();
	pr_info("%s(): rmnet_wlan_set_hooks set\n", __func__);

	return 0;
err2:
	rmnet_wlan_connection_deinit();
err1:
	genl_unregister_family(&rmnet_wlan_genl_family);
err0:
	return ret;
}

static void __exit rmnet_wlan_genl_exit(void)
{
	int ret;

	pr_info("%s(): rmnet_wlan exiting\n", __func__);
	ret = rmnet_wlan_connection_deinit();
	if (ret)
		pr_err("%s(): connection management de-init failed: %i\n", __func__, ret);

	rmnet_wlan_deinit();
	ret = genl_unregister_family(&rmnet_wlan_genl_family);
	if (ret)
		pr_err("%s(): unregister family failed: %i\n", __func__, ret);

	rmnet_wlan_unset_hooks();
	pr_info("%s(): rmnet_wlan_unset_hooks unset\n", __func__);
}


MODULE_LICENSE("GPL v2");
module_init(rmnet_wlan_genl_init);
module_exit(rmnet_wlan_genl_exit);
