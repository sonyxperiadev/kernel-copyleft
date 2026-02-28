// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET PERF framework */

#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/in.h>
#include <linux/udp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include "rmnet_module.h"
#include <net/ipv6.h>
#include <net/ip.h>

#include "rmnet_perf_tcp.h"
#include "rmnet_perf_udp.h"

#include "rmnet_descriptor.h"
#include "rmnet_map.h"
#include "rmnet_qmap.h"

#include <net/genetlink.h>

MODULE_LICENSE("GPL v2");

/* Insert newest first, last 4 bytes of the change id */
static char *verinfo[] = {
	"71b2019d",
	"1a5fa493",
	"58aa9bee",
	"8ab0a8ee",
	"f22bace0",
	"cc98f08a",
	"ce79321c",
	"5dcdd4c0",
	"4c9b5337",
	"a3babd40",
	"7f078f96"
};

#define RMNET_PERF_GENL_FAMILY_NAME "RMNET_PERF"
#define RMNET_PERF_GENL_MULTICAST_NAME_0 "RMNET_PERF_MC_0"
#define RMNET_PERF_GENL_MULTICAST_NAME_1 "RMNET_PERF_MC_1"
#define RMNET_PERF_GENL_MULTICAST_NAME_2 "RMNET_PERF_MC_2"
#define RMNET_PERF_GENL_MULTICAST_NAME_3 "RMNET_PERF_MC_3"
#define RMNET_PERF_GENL_VERSION 1

enum {
	RMNET_PERF_CMD_UNSPEC,
	RMNET_PERF_CMD_GET_STATS,
	RMNET_PERF_CMD_MAP_CMD,
	__RMNET_PERF_GENL_CMD_MAX,
};

enum {
	RMNET_PERF_ATTR_UNSPEC,
	RMNET_PERF_ATTR_STATS_REQ,
	RMNET_PERF_ATTR_STATS_RESP,
	RMNET_PERF_ATTR_MAP_CMD_REQ,
	RMNET_PERF_ATTR_MAP_CMD_RESP,
	RMNET_PERF_ATTR_MAP_CMD_IND,
	__RMNET_PERF_ATTR_MAX,
};

enum {
	RMNET_PERF_MULTICAST_GROUP_0,
	RMNET_PERF_MULTICAST_GROUP_1,
	RMNET_PERF_MULTICAST_GROUP_2,
	RMNET_PERF_MULTICAST_GROUP_3,
	__RMNET_PERF_MULTICAST_GROUP_MAX,
};

#define RMNET_PERF_ATTR_MAX (__RMNET_PERF_ATTR_MAX - 1)

struct rmnet_perf_stats_req {
	u8 mux_id;
} __aligned(1);

struct rmnet_perf_proto_stats {
	u64 tcpv4_pkts;
	u64 tcpv4_bytes;
	u64 udpv4_pkts;
	u64 udpv4_bytes;
	u64 tcpv6_pkts;
	u64 tcpv6_bytes;
	u64 udpv6_pkts;
	u64 udpv6_bytes;
} __aligned(1);

struct rmnet_perf_coal_common_stats {
	u64 csum_error;
	u64 pkt_recons;
	u64 close_non_coal;
	u64 l3_mismatch;
	u64 l4_mismatch;
	u64 nlo_limit;
	u64 pkt_limit;
	u64 byte_limit;
	u64 time_limit;
	u64 eviction;
	u64 close_coal;
} __aligned(1);

struct downlink_stats {
	struct rmnet_perf_coal_common_stats coal_common_stats;
	struct rmnet_perf_proto_stats coal_veid_stats[16];
	u64 non_coal_pkts;
	u64 non_coal_bytes;
} __aligned(1);

struct uplink_stats {
	struct rmnet_perf_proto_stats seg_proto_stats;
} __aligned(1);

struct rmnet_perf_stats_store {
	struct downlink_stats dl_stats;
	struct uplink_stats ul_stats;
} __aligned(1);

struct rmnet_perf_stats_resp {
	u16 error_code;
	struct rmnet_perf_stats_store stats;
} __aligned(1);

struct rmnet_perf_map_cmd_req {
	u16 cmd_len;
	u8 cmd_name;
	u8 ack;
	u8 cmd_content[16384];
} __aligned(1);

struct rmnet_perf_map_cmd_resp {
	u8 cmd_name;
	u16 error_code;
} __aligned(1);

struct rmnet_perf_map_cmd_ind {
	u16 cmd_len;
	u8 cmd_name;
	u8 ack;
	u8 cmd_content[4096];
} __aligned(1);

static struct nla_policy rmnet_perf_nl_policy[RMNET_PERF_ATTR_MAX + 1] = {
	[RMNET_PERF_ATTR_STATS_REQ] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_perf_stats_req)),
	[RMNET_PERF_ATTR_STATS_RESP] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_perf_stats_resp)),
	[RMNET_PERF_ATTR_MAP_CMD_REQ] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_perf_map_cmd_req)),
	[RMNET_PERF_ATTR_MAP_CMD_RESP] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_perf_map_cmd_resp)),
	[RMNET_PERF_ATTR_MAP_CMD_IND] = NLA_POLICY_EXACT_LEN(sizeof(struct rmnet_perf_map_cmd_ind)),
};

static const struct genl_multicast_group rmnet_perf_nl_mcgrps[] = {
	[RMNET_PERF_MULTICAST_GROUP_0] = { .name = RMNET_PERF_GENL_MULTICAST_NAME_0, },
	[RMNET_PERF_MULTICAST_GROUP_1] = { .name = RMNET_PERF_GENL_MULTICAST_NAME_1, },
	[RMNET_PERF_MULTICAST_GROUP_2] = { .name = RMNET_PERF_GENL_MULTICAST_NAME_2, },
	[RMNET_PERF_MULTICAST_GROUP_3] = { .name = RMNET_PERF_GENL_MULTICAST_NAME_3, },
};

int rmnet_perf_netlink_seq = 0;

module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

bool enable_tcp = true;
module_param_named(rmnet_perf_knob0, enable_tcp, bool, 0644);

static bool enable_udp = true;
module_param_named(rmnet_perf_knob1, enable_udp, bool, 0644);

#define RMNET_INGRESS_QUIC_PORT 443

struct rmnet_perf_stats_store stats_store[17];

static inline bool rmnet_perf_is_quic_packet(struct udphdr *uh)
{
	return be16_to_cpu(uh->source) == RMNET_INGRESS_QUIC_PORT ||
	       be16_to_cpu(uh->dest) == RMNET_INGRESS_QUIC_PORT;
}

static bool rmnet_perf_is_quic_initial_packet(struct sk_buff *skb, int ip_len)
{
	u8 *first_byte, __first_byte;
	struct udphdr *uh, __uh;

	uh = skb_header_pointer(skb, ip_len, sizeof(*uh), &__uh);

	if (!uh || !rmnet_perf_is_quic_packet(uh))
		return false;

	/* Length sanity check. Could check for the full QUIC header length if
	 * need be, but since all we really care about is the first byte, just
	 * make sure there is one.
	 */
	if (be16_to_cpu(uh->len) < sizeof(struct udphdr) + 1)
		return false;

	/* I am a very paranoid accessor of data at this point... */
	first_byte = skb_header_pointer(skb, ip_len + sizeof(struct udphdr),
					1, &__first_byte);
	if (!first_byte)
		return false;

	return ((*first_byte) & 0xC0) == 0xC0;
}

static int rmnet_perf_ingress_handle_quic(struct sk_buff *skb, int ip_len)
{
	if (rmnet_perf_is_quic_initial_packet(skb, ip_len)) {
		skb->hash = 0;
		skb->sw_hash = 1;
		return 0;
	}

	return -EINVAL;
}

int rmnet_perf_ingress_handle(struct sk_buff *skb)
{
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, __iph;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph || ip_is_fragment(iph))
			return -EINVAL;

		if (iph->protocol == IPPROTO_UDP) {
			if (enable_udp)
				rmnet_perf_ingress_handle_udp(skb);

			return rmnet_perf_ingress_handle_quic(skb,
							      iph->ihl * 4);
		}

		if (iph->protocol == IPPROTO_TCP) {
			if (enable_tcp)
				rmnet_perf_ingress_handle_tcp(skb);

			/* Don't skip SHS processing for TCP */
			return -EINVAL;
		}
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, __ip6h;
		int ip_len;
		__be16 frag_off;
		u8 proto;

		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			return -EINVAL;

		proto = ip6h->nexthdr;
		ip_len = ipv6_skip_exthdr(skb, sizeof(*ip6h), &proto,
					  &frag_off);
		if (ip_len < 0 || frag_off)
			return -EINVAL;

		if (proto == IPPROTO_UDP) {
			if (enable_udp)
				rmnet_perf_ingress_handle_udp(skb);

			return rmnet_perf_ingress_handle_quic(skb, ip_len);
		}

		if (proto == IPPROTO_TCP) {
			if (enable_tcp)
				rmnet_perf_ingress_handle_tcp(skb);

			return -EINVAL;
		}
	}

	return -EINVAL;
}

void rmnet_perf_ingress_rx_handler(struct sk_buff *skb)
{
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, __iph;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph || ip_is_fragment(iph))
			return;

		if (iph->protocol == IPPROTO_TCP) {
			if (enable_tcp)
				rmnet_perf_ingress_rx_handler_tcp(skb);
		}
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, __ip6h;
		int ip_len;
		__be16 frag_off;
		u8 proto;

		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			return;

		proto = ip6h->nexthdr;
		ip_len = ipv6_skip_exthdr(skb, sizeof(*ip6h), &proto,
					  &frag_off);
		if (ip_len < 0 || frag_off)
			return;

		if (proto == IPPROTO_TCP) {
			if (enable_tcp)
				rmnet_perf_ingress_rx_handler_tcp(skb);
		}
	}
}

static void rmnet_perf_egress_handle_quic(struct sk_buff *skb, int ip_len)
{
	if (rmnet_perf_is_quic_initial_packet(skb, ip_len))
		skb->priority = 0xDA001A;
}

void rmnet_perf_egress_handle(struct sk_buff *skb)
{
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, __iph;

		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		/* Potentially problematic, but the problem is secondary
		 * fragments have no transport header.
		 */
		if (!iph || ip_is_fragment(iph))
			return;

		if (iph->protocol == IPPROTO_UDP) {
			if (enable_udp)
				rmnet_perf_egress_handle_udp(skb);

			rmnet_perf_egress_handle_quic(skb, iph->ihl * 4);
			return;
		}

		if (iph->protocol == IPPROTO_TCP) {
			if (enable_tcp)
				rmnet_perf_egress_handle_tcp(skb);

			return;
		}
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, __ip6h;
		int ip_len;
		__be16 frag_off;
		u8 proto;

		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			return;

		proto = ip6h->nexthdr;
		ip_len = ipv6_skip_exthdr(skb, sizeof(*ip6h), &proto,
					  &frag_off);
		if (ip_len < 0 || frag_off)
			return;

		if (proto == IPPROTO_UDP) {
			if (enable_udp)
				rmnet_perf_egress_handle_udp(skb);

			rmnet_perf_egress_handle_quic(skb, ip_len);
			return;
		}

		if (proto == IPPROTO_TCP) {
			if (enable_tcp)
				rmnet_perf_egress_handle_tcp(skb);

			return;
		}
	}
}

void rmnet_perf_nl_map_cmd_multicast(struct sk_buff *skb);

/* skb will be freed by rmnet_qmap_cmd_handler() */
void rmnet_perf_cmd_ingress_handler(struct sk_buff *skb)
{
	if (skb_linearize(skb)) {
		pr_err("%s(): Linearization error\n", __func__);
		return;
	}

	rmnet_perf_nl_map_cmd_multicast(skb);
}

void rmnet_perf_coal_common_stat(uint8_t mux_id, uint32_t type)
{
	if (!mux_id || mux_id > 16)
		goto err0;

	switch (type) {
	case 0:
		stats_store[mux_id].dl_stats.coal_common_stats.csum_error++;
		break;
	case 1:
		stats_store[mux_id].dl_stats.coal_common_stats.pkt_recons++;
		break;
	case 2:
		stats_store[mux_id].dl_stats.coal_common_stats.close_non_coal++;
		break;
	case 3:
		stats_store[mux_id].dl_stats.coal_common_stats.l3_mismatch++;
		break;
	case 4:
		stats_store[mux_id].dl_stats.coal_common_stats.l4_mismatch++;
		break;
	case 5:
		stats_store[mux_id].dl_stats.coal_common_stats.nlo_limit++;
		break;
	case 6:
		stats_store[mux_id].dl_stats.coal_common_stats.pkt_limit++;
		break;
	case 7:
		stats_store[mux_id].dl_stats.coal_common_stats.byte_limit++;
		break;
	case 8:
		stats_store[mux_id].dl_stats.coal_common_stats.time_limit++;
		break;
	case 9:
		stats_store[mux_id].dl_stats.coal_common_stats.eviction++;
		break;
	case 10:
		stats_store[mux_id].dl_stats.coal_common_stats.close_coal++;
		break;
	default:
		break;
	}

err0:
	return;
}

void rmnet_perf_coal_stat(uint8_t mux_id, uint8_t veid, uint64_t len, uint32_t type)
{
	if (!mux_id || mux_id > 16)
		goto err0;

	if (veid >= 16)
		goto err0;

	switch (type) {
	case 0:
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].tcpv4_pkts++;
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].tcpv4_bytes += len;
		break;
	case 1:
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].udpv4_pkts++;
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].udpv4_bytes += len;
		break;
	case 2:
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].tcpv6_pkts++;
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].tcpv6_bytes += len;
		break;
	case 3:
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].udpv6_pkts++;
		stats_store[mux_id].dl_stats.coal_veid_stats[veid].udpv6_bytes += len;
		break;
	}

err0:
	return;
}

void rmnet_perf_seg_stat(uint8_t mux_id, struct sk_buff *skb)
{
	if (!mux_id || mux_id > 16)
		goto err0;

	if (skb->protocol == htons(ETH_P_IP)) {
		if (ip_hdr(skb)->protocol == IPPROTO_TCP) {
			stats_store[mux_id].ul_stats.seg_proto_stats.tcpv4_pkts++;
			stats_store[mux_id].ul_stats.seg_proto_stats.tcpv4_bytes += skb->len;
		} else if (ip_hdr(skb)->protocol == IPPROTO_UDP) {
			stats_store[mux_id].ul_stats.seg_proto_stats.udpv4_pkts++;
			stats_store[mux_id].ul_stats.seg_proto_stats.udpv4_bytes += skb->len;
		}
	}

	if (skb->protocol == htons(ETH_P_IPV6)) {
		if (ipv6_hdr(skb)->nexthdr == IPPROTO_TCP) {
			stats_store[mux_id].ul_stats.seg_proto_stats.tcpv6_pkts++;
			stats_store[mux_id].ul_stats.seg_proto_stats.tcpv6_bytes += skb->len;
		} else if (ipv6_hdr(skb)->nexthdr == IPPROTO_UDP) {
			stats_store[mux_id].ul_stats.seg_proto_stats.udpv6_pkts++;
			stats_store[mux_id].ul_stats.seg_proto_stats.udpv6_bytes += skb->len;
		}
	}

err0:
	return;
}

void rmnet_perf_non_coal_stat(uint8_t mux_id, uint64_t len)
{
	if (!mux_id || mux_id > 16)
		goto err0;

	stats_store[mux_id].dl_stats.non_coal_pkts++;
	stats_store[mux_id].dl_stats.non_coal_bytes += len;

err0:
	return;
}

static const struct rmnet_module_hook_register_info
rmnet_perf_module_hooks[] = {
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_INGRESS,
		.func = rmnet_perf_ingress_handle,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_EGRESS,
		.func = rmnet_perf_egress_handle,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_SET_THRESH,
		.func = rmnet_perf_tcp_update_quickack_thresh,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_INGRESS_RX_HANDLER,
		.func = rmnet_perf_ingress_rx_handler,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_CMD_INGRESS,
		.func = rmnet_perf_cmd_ingress_handler,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_COAL_COMMON_STAT,
		.func = rmnet_perf_coal_common_stat,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_COAL_STAT,
		.func = rmnet_perf_coal_stat,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_SEG_STAT,
		.func = rmnet_perf_seg_stat,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_NON_COAL_STAT,
		.func = rmnet_perf_non_coal_stat,
	},
};

void rmnet_perf_set_hooks(void)
{
	rmnet_module_hook_register(rmnet_perf_module_hooks,
				   ARRAY_SIZE(rmnet_perf_module_hooks));
}

void rmnet_perf_unset_hooks(void)
{
	rmnet_module_hook_unregister(rmnet_perf_module_hooks,
				     ARRAY_SIZE(rmnet_perf_module_hooks));
}

int rmnet_perf_nl_cmd_get_stats(struct sk_buff *skb, struct genl_info *info);
int rmnet_perf_nl_cmd_map_cmd_req(struct sk_buff *skb, struct genl_info *info);

static const struct genl_ops rmnet_perf_nl_ops[] = {
	{
		.cmd = RMNET_PERF_CMD_GET_STATS,
		.doit = rmnet_perf_nl_cmd_get_stats,
	},
	{
		.cmd = RMNET_PERF_CMD_MAP_CMD,
		.doit = rmnet_perf_nl_cmd_map_cmd_req,
	},
};

struct genl_family rmnet_perf_nl_family __ro_after_init = {
	.hdrsize = 0,
	.name = RMNET_PERF_GENL_FAMILY_NAME,
	.version = RMNET_PERF_GENL_VERSION,
	.maxattr = RMNET_PERF_ATTR_MAX,
	.policy = rmnet_perf_nl_policy,
	.ops = rmnet_perf_nl_ops,
	.n_ops = ARRAY_SIZE(rmnet_perf_nl_ops),
	.mcgrps = rmnet_perf_nl_mcgrps,
	.n_mcgrps = ARRAY_SIZE(rmnet_perf_nl_mcgrps),
};

int rmnet_perf_nl_cmd_get_stats(struct sk_buff *skb, struct genl_info *info)
{
	struct rmnet_perf_stats_resp *resp = NULL;
	struct rmnet_perf_stats_req req;
	int bytes = -1, ret = -ENOMEM;
	struct sk_buff *rskb = NULL;
	struct nlattr *na = NULL;
	void *hdrp = NULL;

	rskb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!rskb) {
		pr_err("%s(): Failed to allocate response skb\n", __func__);
		goto err0;
	}

	hdrp = genlmsg_put(rskb, 0, rmnet_perf_netlink_seq++,
			   &rmnet_perf_nl_family, 0,
			   RMNET_PERF_CMD_GET_STATS);
	if (!hdrp) {
		pr_err("%s(): Failed to set header pointer\n", __func__);
		goto err1;
	}

	resp = kzalloc(sizeof(struct rmnet_perf_stats_resp), GFP_ATOMIC);
	if (!resp) {
		pr_err("%s(): Failed to allocate response cmd\n", __func__);
		goto err1;
	}

	memset(&req, 0, sizeof(struct rmnet_perf_stats_req));
	ret = -EINVAL;
	na = info->attrs[RMNET_PERF_ATTR_STATS_REQ];
	if (!na) {
		pr_err("%s(): Failed to get cmd request attribute\n", __func__);
		goto err2;
	}

	bytes = nla_memcpy(&req, na, sizeof(struct rmnet_perf_stats_req));
	if (bytes <= 0) {
		pr_err("%s(): Failed to copy cmd request attribute\n", __func__);
		goto err2;
	}

	if (req.mux_id > 16) {
		pr_err("%s(): Unsupported mux id %u\n", __func__, req.mux_id);
		goto err2;
	}

	ret = 0;
	memcpy(&resp->stats, &stats_store[req.mux_id],
	       sizeof(struct rmnet_perf_stats_store));

err2:
	resp->error_code = abs(ret);
	if (!nla_put(rskb, RMNET_PERF_ATTR_STATS_RESP,
		     sizeof(struct rmnet_perf_stats_resp), resp)) {
		kfree(resp);
		genlmsg_end(rskb, hdrp);
		return genlmsg_reply(rskb, info);
	} else {
		pr_err("%s(): Failed to copy cmd response attribute\n", __func__);
	}
	kfree(resp);
err1:
	nlmsg_free(rskb);
err0:
	return ret;
}

void rmnet_perf_nl_map_cmd_multicast(struct sk_buff *skb)
{
	uint8_t offset = sizeof(struct qmap_cmd_hdr);
	struct rmnet_perf_map_cmd_ind *ind = NULL;
	struct qmap_cmd_hdr *cmd_hdr = NULL;
	struct sk_buff *iskb = NULL;
	void *hdrp = NULL;
	int rc = -EINVAL;

	iskb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!iskb) {
		pr_err("%s(): Failed to indication skb\n", __func__);
		goto err0;
	}

	hdrp = genlmsg_put(iskb, 0, rmnet_perf_netlink_seq++,
			   &rmnet_perf_nl_family, 0,
			   RMNET_PERF_CMD_MAP_CMD);
	if (!hdrp) {
		pr_err("%s(): Failed to set header pointer\n", __func__);
		goto err1;
	}

	ind = kzalloc(sizeof(struct rmnet_perf_map_cmd_ind), GFP_ATOMIC);
	if (!ind) {
		pr_err("%s(): Failed to allocate indication cmd\n", __func__);
		goto err1;
	}

	if (skb->len <= offset) {
		pr_err("%s(): Incoming cmd size is invalid\n", __func__);
		goto err2;
	}

	cmd_hdr = (struct qmap_cmd_hdr *)skb->data;
	ind->cmd_len = skb->len - offset;
	ind->cmd_name = cmd_hdr->cmd_name;
	ind->ack = cmd_hdr->cmd_type;
	memcpy(ind->cmd_content, skb->data + offset, ind->cmd_len);

	if (nla_put(iskb, RMNET_PERF_ATTR_MAP_CMD_IND,
		    sizeof(struct rmnet_perf_map_cmd_ind), ind)) {
		pr_err("%s(): Failed to copy cmd indication attribute\n", __func__);
		goto err2;
	}

	genlmsg_end(iskb, hdrp);
	kfree(ind);
	/* -EINVAL is the only error for which the skb is not freed */
	rc = genlmsg_multicast(&rmnet_perf_nl_family, iskb, 0,
			       RMNET_PERF_MULTICAST_GROUP_0, GFP_ATOMIC);
	if (rc == -EINVAL) {
		pr_err("%s(): Invalid group for multicast\n", __func__);
		goto err1;
	}
	return;

err2:
	kfree(ind);
err1:
	nlmsg_free(iskb);
err0:
	return;
}

int rmnet_perf_cmd_xmit(struct rmnet_perf_map_cmd_req *cmd)
{
	struct net_device *dev = dev_get_by_name(&init_net, "rmnet_ipa0");
	int cmd_len = sizeof(struct qmap_cmd_hdr) + cmd->cmd_len;
	struct qmap_cmd_hdr *cmd_hdr = NULL;
	struct sk_buff *skb = NULL;
	char *cmd_content = NULL;
	int ret = -ENODEV;

	if (!dev) {
		pr_err("%s(): Unable to get reference to device\n", __func__);
		goto err0;
	}

	skb = alloc_skb(cmd_len, GFP_ATOMIC);
	if (!skb) {
		pr_err("%s(): Unable to allocate memory for cmd\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	skb_put(skb, cmd_len);
	memset(skb->data, 0, cmd_len);

	cmd_hdr = (struct qmap_cmd_hdr *)skb->data;
	cmd_hdr->cd_bit = 1;
	cmd_hdr->mux_id = 0;
	cmd_hdr->pkt_len = htons(sizeof(struct rmnet_map_control_command_header) +
					cmd->cmd_len);
	cmd_hdr->cmd_name = cmd->cmd_name;
	cmd_hdr->cmd_type = cmd->ack;

	cmd_content = (char *)(skb->data + sizeof(struct qmap_cmd_hdr));
	memcpy(cmd_content, cmd->cmd_content, cmd->cmd_len);

	skb->dev = dev;
	skb->protocol = htons(ETH_P_MAP);

	ret = rmnet_qmap_send(skb, RMNET_CH_CTL, false);

err1:
	dev_put(dev);
err0:
	return ret;
}

int rmnet_perf_nl_cmd_map_cmd_req(struct sk_buff *skb, struct genl_info *info)
{
	struct rmnet_perf_map_cmd_req *req = NULL;
	struct rmnet_perf_map_cmd_resp resp;
	int bytes = -1, ret = -ENOMEM;
	struct sk_buff *rskb = NULL;
	struct nlattr *na = NULL;
	void *hdrp = NULL;

	rskb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!rskb) {
		pr_err("%s(): Failed to allocate response skb\n", __func__);
		goto err0;
	}

	hdrp = genlmsg_put(rskb, 0, rmnet_perf_netlink_seq++,
			   &rmnet_perf_nl_family, 0,
			   RMNET_PERF_CMD_MAP_CMD);
	if (!hdrp) {
		pr_err("%s(): Failed to set header pointer\n", __func__);
		goto err1;
	}

	memset(&resp, 0, sizeof(struct rmnet_perf_map_cmd_resp));
	req = kzalloc(sizeof(struct rmnet_perf_map_cmd_req), GFP_ATOMIC);
	if (!req) {
		pr_err("%s(): Failed to allocate request cmd\n", __func__);
		goto err2;
	}

	ret = -EINVAL;
	na = info->attrs[RMNET_PERF_ATTR_MAP_CMD_REQ];
	if (!na) {
		pr_err("%s(): Failed to get cmd request attribute\n", __func__);
		goto err3;
	}

	bytes = nla_memcpy(req, na, sizeof(struct rmnet_perf_map_cmd_req));
	if (bytes <= 0) {
		pr_err("%s(): Failed to copy cmd request attribute\n", __func__);
		goto err3;
	}

	switch (req->cmd_name) {
	case QMAP_CMD_31:
	case QMAP_CMD_32:
	case QMAP_CMD_40:
	case QMAP_CMD_42:
		break;
	default:
		pr_err("%s(): Unsupported command %u\n", __func__, req->cmd_name);
		goto err3;
	}

	if (!req->cmd_len || (req->cmd_len > 16000)) {
		pr_err("%s(): Unsupported length %u\n", __func__, req->cmd_len);
		goto err3;
	}

	resp.cmd_name = req->cmd_name;
	ret = rmnet_perf_cmd_xmit(req);

err3:
	kfree(req);
err2:
	resp.error_code = abs(ret);
	if (!nla_put(rskb, RMNET_PERF_ATTR_MAP_CMD_RESP,
		     sizeof(struct rmnet_perf_map_cmd_resp), &resp)) {
		genlmsg_end(rskb, hdrp);
		return genlmsg_reply(rskb, info);
	} else {
		pr_err("%s(): Failed to copy cmd response attribute\n", __func__);
	}
err1:
	nlmsg_free(rskb);
err0:
	return ret;
}

int rmnet_perf_nl_register(void)
{
	return genl_register_family(&rmnet_perf_nl_family);
}

void rmnet_perf_nl_unregister(void)
{
	genl_unregister_family(&rmnet_perf_nl_family);
}

static int __init rmnet_perf_init(void)
{
	int rc;

	pr_info("%s(): Loading\n", __func__);
	rc = rmnet_perf_tcp_init();
	if (rc)
		goto err0;

	rc = rmnet_perf_udp_init();
	if (rc)
		goto err1;

	rc = rmnet_perf_nl_register();
	if (rc) {
		pr_err("%s(): Failed to register generic netlink family\n", __func__);
		goto err2;
	}

	rmnet_perf_set_hooks();

err2:
	rmnet_perf_udp_exit();
err1:
	rmnet_perf_tcp_exit();
err0:
	return rc;
}

static void __exit rmnet_perf_exit(void)
{
	rmnet_perf_unset_hooks();
	rmnet_perf_nl_unregister();
	rmnet_perf_udp_exit();
	rmnet_perf_tcp_exit();
	pr_info("%s(): exiting\n", __func__);
}

module_init(rmnet_perf_init);
module_exit(rmnet_perf_exit);
