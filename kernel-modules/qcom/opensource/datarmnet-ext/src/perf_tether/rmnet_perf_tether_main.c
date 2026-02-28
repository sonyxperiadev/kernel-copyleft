// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET PERF TETHER framework */

#include <linux/module.h>
#include <net/tcp.h>
#include "rmnet_descriptor.h"
#include "rmnet_map.h"
#include "rmnet_qmap.h"
#include "rmnet_module.h"

MODULE_LICENSE("GPL v2");

/* Insert newest first, last 4 bytes of the change id */
static char *verinfo[] = {
	"58aa9bee",
	"e218f451",
	"648b7095",
	"7415921c",
	"49af9bd4"
};
module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

struct rmnet_perf_tether_state {
	u8 rmnet_perf_tether_vnd_count;
};

static struct rmnet_perf_tether_state *rmnet_perf_tether;

unsigned int configure_knob1 __read_mostly = 0;
module_param(configure_knob1, uint, 0644);

unsigned int knob1 __read_mostly = 0;
module_param(knob1, uint, 0644);

unsigned int configure_knob2 __read_mostly = 0;
module_param(configure_knob2, uint, 0644);

unsigned int knob2 __read_mostly = 0;
module_param(knob2, uint, 0644);

static DEFINE_SPINLOCK(rmnet_perf_tether_lock);

#define RMNET_PERF_TETHER_NUM_FLOWS (50)

#define RMNET_PERF_TETHER_HASH_TABLE_BITS \
	(const_ilog2(RMNET_PERF_TETHER_NUM_FLOWS))

static DEFINE_HASHTABLE(rmnet_perf_tether_flow_table,
			RMNET_PERF_TETHER_HASH_TABLE_BITS);

struct rmnet_perf_tether_node {
	struct list_head list;
	struct hlist_node hlist;
	u32 hash;

	/* instead of using headers, the values are stored in __be32 in the
	 * layout used by jhash2 below.
	 */

	__be32 pkt_five_tuple[11];
	u32 tuple_len;
};

struct list_head rmnet_perf_tether_free_list = \
	LIST_HEAD_INIT(rmnet_perf_tether_free_list);

#define RMNET_PERF_TYPE_TETHER_MESSAGE (1)
#define RMNET_PERF_TYPE_TETHER_LEN (12)
#define RMNET_PERF_TYPE_TETHER_CMD_NAME (27)
#define RMNET_PERF_TYPE_TETHER_CMD_MODE (1)

struct rmnet_map_tether_cmd_header
{
	u8 mode;
	u8 endp_count;
	u8 config;
	u8 reserved;
};

static u32 rmnet_perf_tether_get_hash_from_skb(struct sk_buff *skb, int *valid,
					       int syn_ack, int egress)
{
	__be32 pkt_five_tuple[11];
	u32 flow_hash_key_len;

	if (skb->protocol == htons(ETH_P_IP)) {
		/* We know that this is a TCP packet because of the core
		 * hook checks
		 */
		if (!tcp_hdr(skb)->syn)
			goto fail;
		if (syn_ack) {
			if (!tcp_hdr(skb)->ack)
				goto fail;
		} else {
			if (tcp_hdr(skb)->ack)
				goto fail;
		}

		pkt_five_tuple[0] = egress ? ip_hdr(skb)->daddr : ip_hdr(skb)->saddr;
		pkt_five_tuple[1] = egress ? ip_hdr(skb)->saddr : ip_hdr(skb)->daddr;
		pkt_five_tuple[2] = ip_hdr(skb)->protocol;
		pkt_five_tuple[3] = egress ? tcp_hdr(skb)->dest : tcp_hdr(skb)->source;
		pkt_five_tuple[4] = egress ? tcp_hdr(skb)->source : tcp_hdr(skb)->dest;
		flow_hash_key_len = 5;
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		/* We know that this is a TCP packet because of the core
		 * hook checks
		 */
		if (!tcp_hdr(skb)->syn)
			goto fail;
		if (syn_ack) {
			if (!tcp_hdr(skb)->ack)
				goto fail;
		} else {
			if (tcp_hdr(skb)->ack)
				goto fail;
		}

		memcpy(&pkt_five_tuple[0], egress ? ipv6_hdr(skb)->daddr.s6_addr :
		       ipv6_hdr(skb)->saddr.s6_addr, sizeof(struct in6_addr));
		memcpy(&pkt_five_tuple[5], egress ? ipv6_hdr(skb)->saddr.s6_addr :
		       ipv6_hdr(skb)->daddr.s6_addr, sizeof(struct in6_addr));
		pkt_five_tuple[8] = ipv6_hdr(skb)->nexthdr;
		pkt_five_tuple[9] = tcp_hdr(skb)->dest;
		pkt_five_tuple[10] = tcp_hdr(skb)->source;
		flow_hash_key_len = 11;
	} else {
		goto fail;
	}

	*valid = 1;
	return jhash2(pkt_five_tuple, flow_hash_key_len, 0);

fail:
	*valid = 0;
	return 0;
}

static void rmnet_perf_mangle_syn_ack(struct tcphdr *tp)
{
	if (tp->syn && tp->ack) {
		if (configure_knob1) {
			if (knob1 > 65535)
				knob1 = 65535;
			tp->window = cpu_to_be16(knob1);
		}

		if (configure_knob2) {
			unsigned char *ptr;
			u32 length;

			if (knob2 > TCP_MAX_WSCALE)
				knob2 = TCP_MAX_WSCALE;

			length = tp->doff * 4 - sizeof(struct tcphdr);
			ptr = (unsigned char *)(tp + 1);

			while (length > 0) {
				int opcode = *ptr++;
				int opsize;

				switch(opcode) {
				case TCPOPT_EOL:
					return;
				case TCPOPT_NOP:
					length--;
					continue;
				default:
					if (length < 2)
						return;
					opsize = *ptr++;
					if (opsize < 2)
						return;
					if (opsize > length)
						return;
					if (opcode == TCPOPT_WINDOW)
						*ptr = knob2;

					ptr += opsize-2;
					length -= opsize;
				}
			}
		}
	}
}

static int
rmnet_perf_compare_node(struct rmnet_perf_tether_node *node,
			struct sk_buff *skb)
{
	/* already checked for tcp syn earlier */
	if (skb->protocol == htons(ETH_P_IP)) {
		if ((node->pkt_five_tuple[0] == ip_hdr(skb)->saddr) &&
		    (node->pkt_five_tuple[1] == ip_hdr(skb)->daddr) &&
		    (node->pkt_five_tuple[2] == ip_hdr(skb)->protocol) &&
		    (node->pkt_five_tuple[3] == tcp_hdr(skb)->source) &&
		    (node->pkt_five_tuple[4] == tcp_hdr(skb)->dest) &&
		    (node->tuple_len == 5))
			return 0;

	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		if ((!memcmp(&node->pkt_five_tuple[0], ipv6_hdr(skb)->saddr.s6_addr,
		     sizeof(struct in6_addr))) &&
		    (!memcmp(&node->pkt_five_tuple[5], ipv6_hdr(skb)->daddr.s6_addr,
		     sizeof(struct in6_addr))) &&
		    (node->pkt_five_tuple[8] == ipv6_hdr(skb)->nexthdr) &&
		    (node->pkt_five_tuple[9] == tcp_hdr(skb)->source) &&
		    (node->pkt_five_tuple[10] == tcp_hdr(skb)->dest) &&
		    (node->tuple_len == 11))
			return 0;
	}

	return 1;
}

void rmnet_perf_tether_ingress(struct tcphdr *tp, struct sk_buff *skb)
{
	int valid = 0;
	u32 hash;
	unsigned long flags;
	struct rmnet_perf_tether_node *node, *tmp = NULL;

	if (!configure_knob1 && !configure_knob2)
		return;

	hash = rmnet_perf_tether_get_hash_from_skb(skb, &valid, 1, 0);
	if (!valid)
		return;

	spin_lock_irqsave(&rmnet_perf_tether_lock, flags);
	hash_for_each_possible(rmnet_perf_tether_flow_table, node, hlist, hash) {
		if (!rmnet_perf_compare_node(node, skb)) {
			tmp = node;
			break;
		}

		tmp = NULL;
	}

	if (!tmp) {
		spin_unlock_irqrestore(&rmnet_perf_tether_lock, flags);
		return;
	}

	if (node) {
		/* Remove from hashlist and add to free list in case
		 * of a match
		 */
		hash_del(&node->hlist);
		list_add_tail(&node->list, &rmnet_perf_tether_free_list);
	}
	spin_unlock_irqrestore(&rmnet_perf_tether_lock, flags);

	rmnet_perf_mangle_syn_ack(tp);
}

static void
rmnet_perf_populate_node(struct rmnet_perf_tether_node *node,
			 struct sk_buff *skb)
{
	/* already checked for tcp syn earlier */
	if (skb->protocol == htons(ETH_P_IP)) {
		node->pkt_five_tuple[0] = ip_hdr(skb)->daddr;
		node->pkt_five_tuple[1] = ip_hdr(skb)->saddr;
		node->pkt_five_tuple[2] = ip_hdr(skb)->protocol;
		node->pkt_five_tuple[3] = tcp_hdr(skb)->dest;
		node->pkt_five_tuple[4] = tcp_hdr(skb)->source;

		node->tuple_len = 5;
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		memcpy(&node->pkt_five_tuple[0], ipv6_hdr(skb)->daddr.s6_addr,
		       sizeof(struct in6_addr));
		memcpy(&node->pkt_five_tuple[5], ipv6_hdr(skb)->saddr.s6_addr,
		       sizeof(struct in6_addr));
		node->pkt_five_tuple[8] = ipv6_hdr(skb)->nexthdr;
		node->pkt_five_tuple[9] = tcp_hdr(skb)->dest;
		node->pkt_five_tuple[10] = tcp_hdr(skb)->source;

		node->tuple_len = 11;
	}
}

void rmnet_perf_tether_egress(struct sk_buff *skb)
{
	int valid = 0;
	u32 hash;
	unsigned long flags;
	struct rmnet_perf_tether_node *node;
	struct hlist_node *tmp;
	int bkt_cursor;

	/* Check for forwarded skbs */
	struct net_device *dev;

	if (!configure_knob1 && !configure_knob2)
		return;

	if (!skb->skb_iif)
		return;

	dev = __dev_get_by_index(&init_net, skb->skb_iif);
	if (!dev)
		return;

	hash = rmnet_perf_tether_get_hash_from_skb(skb, &valid, 0, 1);
	if (!valid)
		return;

	spin_lock_irqsave(&rmnet_perf_tether_lock, flags);
	/* Find a free node from the freelist and add to the hash list */
	node = list_first_entry_or_null(&rmnet_perf_tether_free_list,
					struct rmnet_perf_tether_node, list);
	if (node) {
		list_del(&node->list);
		node->hash = hash;

		rmnet_perf_populate_node(node, skb);

		hash_add(rmnet_perf_tether_flow_table, &node->hlist,
			 node->hash);
	} else {
		hash_for_each_safe(rmnet_perf_tether_flow_table, bkt_cursor, tmp,
				   node, hlist) {
			/* reuse first node, ideally this needs to be fifo */
			hash_del(&node->hlist);

			node->hash = hash;

			rmnet_perf_populate_node(node, skb);

			hash_add(rmnet_perf_tether_flow_table, &node->hlist,
				 node->hash);
			break;
		}
	}

	spin_unlock_irqrestore(&rmnet_perf_tether_lock, flags);
}

void rmnet_perf_tether_cmd(u8 message, u64 val)
{
	struct net_device *dev = dev_get_by_name(&init_net, "rmnet_ipa0");
	struct sk_buff *skb;

	if (!dev)
		return;

	if (message == RMNET_PERF_TYPE_TETHER_MESSAGE)
	{
		struct rmnet_map_control_command_header *cmdh;
		struct rmnet_map_tether_cmd_header *teth;
		struct rmnet_map_header *maph;

		skb = alloc_skb(16, GFP_ATOMIC);
		if (!skb)
			goto done;

		skb_put(skb, 16);
		memset(skb->data, 0, 16);

		maph = (struct rmnet_map_header *)skb->data;
		maph->cd_bit = 1;
		maph->pkt_len = htons(RMNET_PERF_TYPE_TETHER_LEN);

		cmdh = (struct rmnet_map_control_command_header *)(skb->data + sizeof(*maph));
		cmdh->command_name = RMNET_PERF_TYPE_TETHER_CMD_NAME;

		teth = (struct rmnet_map_tether_cmd_header *)(skb->data + sizeof(*maph) + sizeof(*cmdh));
		teth->mode = RMNET_PERF_TYPE_TETHER_CMD_MODE;
		teth->config = !val;

		skb->dev = dev;
		skb->protocol = htons(ETH_P_MAP);

		rmnet_qmap_send(skb, RMNET_CH_CTL, false);
	}

done:
	dev_put(dev);
}

static const struct rmnet_module_hook_register_info
rmnet_perf_tether_module_hooks[] = {
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_TETHER_INGRESS,
		.func = rmnet_perf_tether_ingress,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_TETHER_EGRESS,
		.func = rmnet_perf_tether_egress,
	},
	{
		.hooknum = RMNET_MODULE_HOOK_PERF_TETHER_CMD,
		.func = rmnet_perf_tether_cmd,
	},
};

void rmnet_perf_tether_set_hooks(void)
{
	rmnet_module_hook_register(rmnet_perf_tether_module_hooks,
				   ARRAY_SIZE(rmnet_perf_tether_module_hooks));
}

void rmnet_perf_tether_unset_hooks(void)
{
	rmnet_module_hook_unregister(rmnet_perf_tether_module_hooks,
				ARRAY_SIZE(rmnet_perf_tether_module_hooks));
}

static int rmnet_perf_tether_state_init(void)
{
	int i;

	rmnet_perf_tether = kzalloc(sizeof(*rmnet_perf_tether), GFP_KERNEL);
	if (!rmnet_perf_tether) {
		pr_err("%s(): Resource allocation failed\n", __func__);
		return -1;
	}

	rmnet_perf_tether->rmnet_perf_tether_vnd_count++;

	for (i = 0; i < RMNET_PERF_TETHER_NUM_FLOWS; i++) {
		struct rmnet_perf_tether_node *node;

		node = kzalloc(sizeof(*node), GFP_ATOMIC);
		if (!node)
			continue;

		INIT_LIST_HEAD(&node->list);
		INIT_HLIST_NODE(&node->hlist);

		list_add_tail(&node->list, &rmnet_perf_tether_free_list);
	}

	/* Everything is ready. Say hello to the core driver */
	rmnet_perf_tether_set_hooks();
	return 0;
}

static void rmnet_perf_tether_clear_flow_table(void)
{
	struct rmnet_perf_tether_node *node;
	struct hlist_node *tmp;
	int bkt_cursor;

	hash_for_each_safe(rmnet_perf_tether_flow_table, bkt_cursor, tmp,
			   node, hlist) {
		hash_del(&node->hlist);
		kfree(node);
	}
}

static void rmnet_perf_tether_clear_free_list(void)
{
	struct rmnet_perf_tether_node *node, *idx;

	list_for_each_entry_safe(node, idx, &rmnet_perf_tether_free_list,
				 list) {
		list_del(&node->list);
		kfree(node);
	}
}

static void rmnet_perf_tether_state_free(void)
{
	rmnet_perf_tether_unset_hooks();

	rmnet_perf_tether_clear_free_list();
	rmnet_perf_tether_clear_flow_table();

	kfree(rmnet_perf_tether);
	rmnet_perf_tether = NULL;
}

static int rmnet_perf_tether_state_notifier(struct notifier_block *nb,
					    unsigned long notify_event,
					    void *notify_data)
{
	struct net_device *device = netdev_notifier_info_to_dev(notify_data);
	int rc;

	(void)nb;
	/* We only care about rmnet devices */
	if (!device || strncmp(device->name, "rmnet_data", 10))
		goto done;

	switch (notify_event) {
	case NETDEV_REGISTER:
		/* Don't initialze if we've already done so */
		if (rmnet_perf_tether) {
			/* Increment the device count and we're done */
			rmnet_perf_tether->rmnet_perf_tether_vnd_count++;
			goto done;
		}

		pr_info("%s(): Initializing on device %s\n", __func__,
			device->name);
		rc = rmnet_perf_tether_state_init();
		if (rc) {
			pr_err("%s(): Initialization failed\n", __func__);
			goto done;
		}

		break;
	case NETDEV_UNREGISTER:
		/* Don't uninitialize if we never initialized */
		if (!rmnet_perf_tether)
			goto done;

		/* Decrement vnd count and free if no more devices */
		if (--rmnet_perf_tether->rmnet_perf_tether_vnd_count)
			goto done;

		pr_info("%s(): Uninitializing on device %s\n", __func__,
			device->name);
		rmnet_perf_tether_state_free();
		break;
	}

done:
	return NOTIFY_DONE;
}

static struct notifier_block rmnet_perf_tether_state_notifier_block = {
	.notifier_call = rmnet_perf_tether_state_notifier,
	.priority = 3,
};

static int __init rmnet_perf_tether_init(void)
{
	pr_info("%s(): Loading\n", __func__);
	return register_netdevice_notifier(&rmnet_perf_tether_state_notifier_block);
}

static void __exit rmnet_perf_tether_exit(void)
{
	pr_info("%s(): exiting\n", __func__);
	unregister_netdevice_notifier(&rmnet_perf_tether_state_notifier_block);
}

module_init(rmnet_perf_tether_init);
module_exit(rmnet_perf_tether_exit);
