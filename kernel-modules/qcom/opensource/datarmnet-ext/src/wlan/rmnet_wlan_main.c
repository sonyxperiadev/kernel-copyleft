// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* RMNET WLAN handler framework */

#include <linux/types.h>
#include <linux/skbuff.h>
#include <net/genetlink.h>
#include <net/netlink.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/tcp.h>
#include <linux/rcupdate.h>
#include <linux/list.h>
#include <linux/rculist.h>
#include <linux/mutex.h>
#include <linux/hashtable.h>
#include <linux/log2.h>
#include <linux/netdevice.h>
#include "rmnet_wlan_genl.h"
#include "rmnet_wlan.h"
#include "rmnet_wlan_stats.h"
#include "rmnet_wlan_fragment.h"
#include "rmnet_wlan_connection.h"

/* Insert newest first, last 4 bytes of the change id */
static char *verinfo[] = {
	"07c9a447",
	"20b1aeb1",
	"4c12af9c",
	"a586fa1f",
	"66dfa294",
	"c203e699",
	"b30ce266",
	"58aa9bee",
	"729ca737",
	"f45422bd",
	"02931fbf",
};
module_param_array(verinfo, charp, NULL, 0444);
MODULE_PARM_DESC(verinfo, "Version of the driver");

#define RMNET_WLAN_HASH_BKTS (16)
#define RMNET_WLAN_HASH_BITS (const_ilog2(RMNET_WLAN_HASH_BKTS))

struct rmnet_wlan_tuple_node {
	struct hlist_node hash;
	struct rcu_head rcu;
	struct rmnet_wlan_tuple info;
};

struct rmnet_wlan_encap_node {
	struct hlist_node hash;
	struct rcu_head rcu;
	__be16 port;
};

/* For hashtable protection */
static DEFINE_MUTEX(rmnet_wlan_tuple_mutex);
static DEFINE_HASHTABLE(rmnet_wlan_tuple_hash, RMNET_WLAN_HASH_BITS);

/* For forward information hashtable protection */
static DEFINE_MUTEX(rmnet_wlan_fwd_mutex);
static DEFINE_HASHTABLE(rmnet_wlan_fwd_hash, RMNET_WLAN_HASH_BITS);

/* For encap port hashtable protection */
static DEFINE_MUTEX(rmnet_wlan_encap_mutex);
static DEFINE_HASHTABLE(rmnet_wlan_encap_hash, RMNET_WLAN_HASH_BITS);
static DEFINE_HASHTABLE(rmnet_wlan_act_encap_drop_hash, RMNET_WLAN_HASH_BITS);

/* For low latency global structure protection */
static DEFINE_MUTEX(rmnet_wlan_ll_tuple_mutex);
static DEFINE_SPINLOCK(rmnet_wlan_tuple_lock);

/* wlan device */
static char rmnet_wlan_device[IFNAMSIZ];

/* rmnet device for cwlan */
static char rmnet_wwan_device[IFNAMSIZ];

/* Tuple Count */
static u32 rmnet_wlan_tuple_count;

/* Low latency tuple cache */
struct rmnet_wlan_ll_tuple * rmnet_wlan_ll_tuple_cache;

static bool rmnet_wlan_tuple_match(struct rmnet_wlan_tuple *t1,
				   struct rmnet_wlan_tuple *t2)
{
	/* Protocols must match */
	if (t1->ip_proto != t2->ip_proto ||
	    t1->trans_proto != t2->trans_proto)
		return false;

	if (t1->ip_proto == IPPROTO_ESP)
		/* Check SPI value */
		return t1->spi_val == t2->spi_val;

	/* Check port value */
	return t1->port == t2->port;
}

static int __rmnet_wlan_add_tuple(struct rmnet_wlan_tuple *tuple,
				  struct genl_info *info)
	__must_hold(&rmnet_wlan_tuple_mutex)
{
	struct rmnet_wlan_tuple_node *node;

	rcu_read_lock();
	if (rmnet_wlan_tuple_present(tuple)) {
		/* No duplicates */
		rcu_read_unlock();
		GENL_SET_ERR_MSG(info, "Tuple already present");
		return -EEXIST;
	}

	rcu_read_unlock();

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		GENL_SET_ERR_MSG(info, "Cannot allocate tuple memory");
		return -ENOMEM;
	}

	memcpy(&node->info, tuple, sizeof(*tuple));
	INIT_HLIST_NODE(&node->hash);
	/* Hash is the port value (first 2 bytes of SPI in ESP case) */
	hash_add_rcu(rmnet_wlan_tuple_hash, &node->hash, tuple->port);
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_TUPLE_ADD);
	return 0;
}

static int __rmnet_wlan_del_tuple(struct rmnet_wlan_tuple *tuple,
				  struct genl_info *info)
	__must_hold(&rmnet_wlan_tuple_mutex)
{
	struct rmnet_wlan_tuple_node *node;
	struct hlist_node *tmp;

	hash_for_each_possible_safe(rmnet_wlan_tuple_hash, node, tmp, hash,
				    tuple->port) {
		if (rmnet_wlan_tuple_match(&node->info, tuple)) {
			hash_del_rcu(&node->hash);
			kfree_rcu(node, rcu);
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_TUPLE_DEL);
			return 0;
		}
	}

	GENL_SET_ERR_MSG(info, "No such tuple");
	return -ESRCH;
}

static void rmnet_wlan_flush_tuples(void)
{
	struct rmnet_wlan_tuple_node *node;
	struct hlist_node *tmp;
	int bkt;

	mutex_lock(&rmnet_wlan_tuple_mutex);
	rmnet_wlan_tuple_count = 0;

	hash_for_each_safe(rmnet_wlan_tuple_hash, bkt, tmp, node, hash) {
		hash_del_rcu(&node->hash);
		kfree_rcu(node, rcu);
	}

	mutex_unlock(&rmnet_wlan_tuple_mutex);
}

static bool rmnet_wlan_fwd_info_match(struct rmnet_wlan_fwd_info *f1,
				      struct rmnet_wlan_fwd_info *f2)
{
	/* Reverse link matches modem prefix, but not IID */
	struct in6_addr mask = {
		.s6_addr32 = { 0xffffffff, 0xffffffff, 0, 0 },
	};

	if (f1->ip_proto != f2->ip_proto)
		return false;

	if (f1->ip_proto == 4)
		return f1->v4_addr == f2->v4_addr;

	return !ipv6_masked_addr_cmp(&f1->v6_addr, &mask, &f2->v6_addr);
}

struct rmnet_wlan_fwd_info_node *
rmnet_wlan_fwd_info_find(struct rmnet_wlan_fwd_info *info) __must_hold(RCU)
{
	struct rmnet_wlan_fwd_info_node *node;

	/* Hash is the first 4 bytes of the address. This prevents using any
	 * uninitialized data in the union for the IPv4 case.
	 */
	hash_for_each_possible_rcu(rmnet_wlan_fwd_hash, node, hash,
				   info->v4_addr) {
		if (rmnet_wlan_fwd_info_match(&node->fwd, info))
			return node;
	}

	return NULL;
}

static int rmnet_wlan_fwd_info_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct net_device *device = netdev_notifier_info_to_dev(data);
	struct rmnet_wlan_fwd_info_node *fwd_info;

	fwd_info = container_of(nb, struct rmnet_wlan_fwd_info_node, nb);
	/* Is this about our fowarding device? */
	if (!device || rmnet_wlan_strlcmp(device->name, fwd_info->dev_name, IFNAMSIZ))
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_GOING_DOWN:
		if (fwd_info->fwd.fwd_dev) {
			dev_put(fwd_info->fwd.fwd_dev);
			WRITE_ONCE(fwd_info->fwd.fwd_dev, NULL);
		}

		break;
	case NETDEV_UP:
		if (!fwd_info->fwd.fwd_dev) {
			dev_hold(device);
			WRITE_ONCE(fwd_info->fwd.fwd_dev, device);
		}

		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static int __rmnet_wlan_add_fwd_info(struct rmnet_wlan_fwd_info *fwd,
				     struct genl_info *info)
	__must_hold(&rmnet_wlan_fwd_mutex)
{
	struct rmnet_wlan_fwd_info_node *fwd_info;
	int err;

	rcu_read_lock();
	fwd_info = rmnet_wlan_fwd_info_find(fwd);
	if (fwd_info) {
		GENL_SET_ERR_MSG(info, "FWD information already present");
		rcu_read_unlock();
		return -EEXIST;
	}

	rcu_read_unlock();

	fwd_info = kzalloc(sizeof(*fwd_info), GFP_KERNEL);
	if (!fwd_info) {
		GENL_SET_ERR_MSG(info, "Cannot allocate FWD context");
		return -ENOMEM;
	}

	/* Don't add the device pointer itself into the forward node yet.
	 * The notifier will handle adding that and taking the appropriate
	 * device references.
	 */
	memcpy(&fwd_info->fwd, fwd, sizeof(*fwd));
	fwd_info->fwd.fwd_dev = NULL;
	/* Silence checker tools from complaining.
	 *
	 * Sure would be a shame if this buffer was guaranteeed to be the
	 * correct size, huh?
	 */
	strscpy(fwd_info->dev_name, fwd->fwd_dev->name, IFNAMSIZ);
	fwd_info->nb.notifier_call = rmnet_wlan_fwd_info_notifier;
	err = register_netdevice_notifier(&fwd_info->nb);
	if (err) {
		GENL_SET_ERR_MSG(info, "Registering FWD notifier failed");
		kfree(fwd_info);
		return err;
	}

	INIT_HLIST_NODE(&fwd_info->hash);
	hash_add_rcu(rmnet_wlan_fwd_hash, &fwd_info->hash, fwd->v4_addr);
	return 0;
}

static int
rmnet_wlan_fwd_info_release(struct rmnet_wlan_fwd_info_node *fwd_info)
	__must_hold(&rmnet_wlan_fwd_mutex)
{
	int err;

	err = unregister_netdevice_notifier(&fwd_info->nb);
	if (err)
		return err;

	hash_del_rcu(&fwd_info->hash);
	/* Make sure no fragments are still holding references to this */
	rmnet_wlan_fragment_del_fwd_info(&fwd_info->fwd);
	kfree_rcu(fwd_info, rcu);
	return err;
}

static int __rmnet_wlan_del_fwd_info(struct rmnet_wlan_fwd_info *fwd,
				     struct genl_info *info)
	__must_hold(&rmnet_wlan_fwd_mutex)
{
	struct rmnet_wlan_fwd_info_node *fwd_info;
	int err;

	rcu_read_lock();
	fwd_info = rmnet_wlan_fwd_info_find(fwd);
	rcu_read_unlock();
	if (!fwd_info) {
		GENL_SET_ERR_MSG(info, "No such FWD information");
		return -ESRCH;
	}

	if (rmnet_wlan_strlcmp(fwd_info->dev_name, fwd->fwd_dev->name, IFNAMSIZ)) {
		GENL_SET_ERR_MSG(info, "Incorrect FWD device");
		return -ENODEV;
	}

	err = rmnet_wlan_fwd_info_release(fwd_info);
	if (err) {
		GENL_SET_ERR_MSG(info, "Unregistering notifier failed");
		return err;
	}

	return err;
}

static void rmnet_wlan_flush_fwd_info(void)
{
	struct rmnet_wlan_fwd_info_node *info;
	struct hlist_node *tmp;
	int bkt;

	mutex_lock(&rmnet_wlan_fwd_mutex);
	hash_for_each_safe(rmnet_wlan_fwd_hash, bkt, tmp, info, hash)
		rmnet_wlan_fwd_info_release(info);

	mutex_unlock(&rmnet_wlan_fwd_mutex);
}

static bool rmnet_wlan_encap_port_present(__be16 port)
	__must_hold(RCU)
{
	struct rmnet_wlan_encap_node *node;

	hash_for_each_possible_rcu(rmnet_wlan_encap_hash, node, hash, port) {
		if (node->port == port)
			return true;
	}

	return false;
}

static int __rmnet_wlan_add_encap_port(__be16 port, struct genl_info *info)
	__must_hold(&rmnet_wlan_encap_mutex)
{
	struct rmnet_wlan_encap_node *node;

	rcu_read_lock();
	if (rmnet_wlan_encap_port_present(port)) {
		/* No duplicates */
		rcu_read_unlock();
		GENL_SET_ERR_MSG(info, "Encap port already present");
		return -EEXIST;
	}

	rcu_read_unlock();

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		GENL_SET_ERR_MSG(info, "Cannot allocate encap port memory");
		return -ENOMEM;
	}

	node->port = port;
	INIT_HLIST_NODE(&node->hash);
	hash_add_rcu(rmnet_wlan_encap_hash, &node->hash, port);
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_PORT_ADD);
	return 0;

}

static int __rmnet_wlan_del_encap_port(__be16 port, struct genl_info *info)
	__must_hold(&rmnet_wlan_encap_mutex)
{
	struct rmnet_wlan_encap_node *node;
	struct hlist_node *tmp;

	hash_for_each_possible_safe(rmnet_wlan_encap_hash, node, tmp, hash,
				    port) {
		if (node->port == port) {
			hash_del_rcu(&node->hash);
			kfree_rcu(node, rcu);
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_PORT_DEL);
			return 0;
		}
	}

	GENL_SET_ERR_MSG(info, "No such port value");
	return -ESRCH;

}

static bool rmnet_wlan_encap_port_drop_present(__be16 port)
	__must_hold(RCU)
{
	struct rmnet_wlan_encap_node *node;

	hash_for_each_possible_rcu(rmnet_wlan_act_encap_drop_hash, node, hash, port) {
		if (node->port == port)
			return true;
	}

	return false;
}

static int __rmnet_wlan_add_ll_tuple(struct rmnet_wlan_ll_tuple *tuple)
	__must_hold(&rmnet_wlan_ll_tuple_mutex)
{
	struct rmnet_wlan_ll_tuple *tuple_internal;
	struct rmnet_wlan_ll_tuple *old_tuple;
	unsigned long ht_flags;

	tuple_internal = kzalloc(sizeof(*tuple_internal), GFP_KERNEL);
	if (!tuple_internal) {
		return -ENOMEM;
	}

	memcpy(tuple_internal, tuple, sizeof(*tuple_internal));

	spin_lock_irqsave(&rmnet_wlan_tuple_lock, ht_flags);
	old_tuple = rcu_dereference_protected(rmnet_wlan_ll_tuple_cache,
					lockdep_is_held(&rmnet_wlan_tuple_lock));
	rcu_assign_pointer(rmnet_wlan_ll_tuple_cache, tuple_internal);
	spin_unlock_irqrestore(&rmnet_wlan_tuple_lock, ht_flags);
	synchronize_rcu();

	if (old_tuple) kfree(old_tuple);

	return 0;
}

static int __rmnet_wlan_del_ll_tuple(void)
	__must_hold(&rmnet_wlan_ll_tuple_mutex)
{
	struct rmnet_wlan_ll_tuple *old_tuple;
	unsigned long ht_flags;

	spin_lock_irqsave(&rmnet_wlan_tuple_lock, ht_flags);
	old_tuple = rcu_dereference_protected(rmnet_wlan_ll_tuple_cache,
					lockdep_is_held(&rmnet_wlan_tuple_lock));
	rcu_assign_pointer(rmnet_wlan_ll_tuple_cache, NULL);
	spin_unlock_irqrestore(&rmnet_wlan_tuple_lock, ht_flags);
	synchronize_rcu();

	if (old_tuple) kfree(old_tuple);

	return 0;
}

int rmnet_wlan_act_encap_port_pass_through(__be16 port, struct genl_info *info)
	__must_hold(&rmnet_wlan_encap_mutex)
{
	struct rmnet_wlan_encap_node *node;
	struct hlist_node *tmp;

	hash_for_each_possible_safe(rmnet_wlan_act_encap_drop_hash, node, tmp, hash,
				    port) {
		if (node->port == port) {
			hash_del_rcu(&node->hash);
			kfree_rcu(node, rcu);
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_PORT_DROP_DEL);
			return 0;
		}
	}

	/* not an error if there is no matching port in this hashtable */
	return 0;
}

int rmnet_wlan_act_encap_port_drop(__be16 port, struct genl_info *info)
	__must_hold(&rmnet_wlan_encap_mutex)
{
	struct rmnet_wlan_encap_node *node;

	rcu_read_lock();
	if (rmnet_wlan_encap_port_drop_present(port)) {
		/* No duplicates */
		rcu_read_unlock();
		GENL_SET_ERR_MSG(info, "Encap port already present");
		return -EEXIST;
	}

	rcu_read_unlock();

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		GENL_SET_ERR_MSG(info, "Cannot allocate encap port memory");
		return -ENOMEM;
	}

	node->port = port;
	INIT_HLIST_NODE(&node->hash);
	hash_add_rcu(rmnet_wlan_act_encap_drop_hash, &node->hash, port);
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_PORT_DROP_ADD);
	return 0;
}

static void rmnet_wlan_flush_encap_ports(void)
{
	struct rmnet_wlan_encap_node *node;
	struct hlist_node *tmp;
	int bkt;

	mutex_lock(&rmnet_wlan_encap_mutex);
	hash_for_each_safe(rmnet_wlan_encap_hash, bkt, tmp, node, hash) {
		hash_del_rcu(&node->hash);
		kfree_rcu(node, rcu);
	}

	mutex_unlock(&rmnet_wlan_encap_mutex);
}

rx_handler_result_t rmnet_wlan_rx_handler(struct sk_buff **pskb)
{
	struct rmnet_wlan_connection_info conn = {};
	struct rmnet_wlan_tuple tuple = {};
	struct rmnet_wlan_fwd_info_node *fwd_node;
	struct rmnet_wlan_fwd_info *fwd_info, info;
	struct sk_buff *skb = *pskb;
	rx_handler_result_t res = RX_HANDLER_PASS;
	int ip_len;

	/* Is this a touchable packet? */
	if (!skb || skb->pkt_type == PACKET_LOOPBACK)
		return res;

	rcu_read_lock();
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_PKT_RX);
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph = ip_hdr(skb);

		if (iph->protocol == IPPROTO_ICMP) {
			/* Check the connection info */
			conn.v4_saddr = iph->daddr;
			conn.v4_daddr = iph->saddr;
			conn.ip_proto = 4;
			fwd_info = rmnet_wlan_connection_find(&conn);
			if (fwd_info)
				goto send;
		}

		info.v4_addr = iph->daddr;
		info.ip_proto = 4;
		fwd_node = rmnet_wlan_fwd_info_find(&info);
		if (!fwd_node)
			goto out;

		fwd_info = &fwd_node->fwd;
		tuple.ip_proto = 4;
		tuple.trans_proto = iph->protocol;
		ip_len = iph->ihl * 4;
		if (ip_is_fragment(iph)) {
			if (!rmnet_wlan_fragment_v4(skb, ip_len, &tuple,
						    fwd_info))
				/* Fragment was handled */
				res = RX_HANDLER_CONSUMED;

			goto out;
		}
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h = ipv6_hdr(skb);
		__be16 frag_off;
		u8 proto;

		proto = ip6h->nexthdr;
		ip_len = ipv6_skip_exthdr(skb, sizeof(*ip6h), &proto,
					  &frag_off);
		if (ip_len < 0)
			goto out;

		if (proto == IPPROTO_ICMPV6) {
			/* Check the connection info */
			memcpy(&conn.v6_saddr, &ip6h->daddr,
			       sizeof(conn.v6_saddr));
			memcpy(&conn.v6_daddr, &ip6h->saddr,
			       sizeof(conn.v6_daddr));
			conn.ip_proto = 6;
			fwd_info = rmnet_wlan_connection_find(&conn);
			if (fwd_info)
				goto send;
		}

		memcpy(&info.v6_addr, &ip6h->daddr, sizeof(info.v6_addr));
		info.ip_proto = 6;
		fwd_node = rmnet_wlan_fwd_info_find(&info);
		if (!fwd_node)
			goto out;

		fwd_info = &fwd_node->fwd;
		tuple.ip_proto = 6;
		tuple.trans_proto = proto;
		if (frag_off) {
			if (!rmnet_wlan_fragment_v6(skb, ip_len, &tuple,
						    fwd_info))
				/* Fragment was handled */
				res = RX_HANDLER_CONSUMED;

			goto out;
		}
	} else {
		goto out;
	}

	if (tuple.trans_proto == IPPROTO_TCP ||
	    tuple.trans_proto == IPPROTO_UDP) {
		struct udphdr *up = (struct udphdr *)(skb->data + ip_len);

		/* Abusing the fact that the ports are in the same location
		 * for both TCP and UDP.
		 */
		tuple.port = up->dest;

		if (rmnet_wlan_udp_encap_check(skb, &tuple, ip_len)) {
			if (rmnet_wlan_udp_encap_drop_check(&tuple)) {
				kfree_skb(skb);

				res = RX_HANDLER_CONSUMED;
				rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_DROP);
				goto out;
			}

			/* Let the stack handle decapsulation */
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_DELIVER);
			goto out;
		}
	} else if (tuple.trans_proto == IPPROTO_ESP) {
		struct ip_esp_hdr *esph, __esph;

		esph = skb_header_pointer(skb, ip_len, sizeof(*esph), &__esph);
		if(!esph) {
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_ENCAP_HDRP_FAIL);
			goto out;
		}

		tuple.spi_val = esph->spi;
	} else {
		goto out;
	}

	/* Search the list for our tuple */
	if (rmnet_wlan_tuple_present(&tuple))
		/* Try to forward */
		goto send;

	/* Marked CIWLAN Traffic will be sent to dummy0 since it'll always have
	   IPv4v6 link local addresses*/
	if(skb->mark == 0x20002) {
		struct net_device *ddev = dev_get_by_name_rcu(&init_net, "dummy0");

		if(ddev) {
			skb->dev = ddev;
			skb->mark = 0;
		}
		else {
			rmnet_wlan_stats_update(RMNET_WLAN_STAT_CIWLAN_DDEV_GET_FAIL);
		}
	}


	/* No match. Let it go. Musical accompaniment is optional */
	goto out;

send:
	if (!rmnet_wlan_deliver_skb(skb, fwd_info))
		res = RX_HANDLER_CONSUMED;
out:
	rcu_read_unlock();
	return res;
}

static int rmnet_wlan_device_notifier(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct net_device *device = netdev_notifier_info_to_dev(data);

	/* Should we even care about this? Hopefully not. */
	if (!device || rmnet_wlan_strlcmp(device->name, rmnet_wlan_device, IFNAMSIZ))
		return NOTIFY_DONE;

	if (!rmnet_wlan_strlcmp(device->name, "rmnet_data", 10)) {
		/* Well, someone's playing a sick joke here and naming wlan
		 * devices "rmnet_data", but it would behoove us to not trash
		 * our own state. Don't set the RX handler here, and just
		 * let the module hook in the rmnet_rx handler call us for it.
		 */
		pr_info("%s(): the goofs! the gaffs! someone has quite the naming scheme!!\n",
			__func__);
		return NOTIFY_DONE;
	}

	switch (event) {
	case NETDEV_UNREGISTER:
		/* Johnny, we hardly knew ya~! */
		netdev_rx_handler_unregister(device);
		break;
	case NETDEV_REGISTER:
		if (netdev_rx_handler_register(device, rmnet_wlan_rx_handler,
					       NULL))
			pr_err("%s(): Registering handler failed\n", __func__);
		break;
	default:
		/* Pesky compilers! */
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block rmnet_wlan_notifier = {
	.notifier_call = rmnet_wlan_device_notifier,
};

static int __rmnet_wlan_unset_device(void)
{
	int err;

	if (!rmnet_wlan_device[0])
		return 0;

	err = unregister_netdevice_notifier(&rmnet_wlan_notifier);
	if (!err)
		/* Notifer brought down successfully. Clear the name */
		rmnet_wlan_device[0] = 0;

	return err;
}

static int __rmnet_wwan_unset_device(void)
{
	rmnet_wwan_device[0] = 0;
	return 0;
}

static void rmnet_wlan_cleanup(void)
{
	/* Bring down the notifier & rx_handler */
	__rmnet_wlan_unset_device();
	__rmnet_wwan_unset_device();

	/* Release all state */
	rmnet_wlan_flush_encap_ports();
	rmnet_wlan_fragments_remove();
	rmnet_wlan_flush_fwd_info();
	rmnet_wlan_flush_tuples();
	rmnet_wlan_del_ll_tuple();
}

static int rmnet_wlan_skb_ensure_writable(struct sk_buff *skb, int write_len)
{
	if (!pskb_may_pull(skb, write_len))
		return -ENOMEM;

	if (!skb_cloned(skb) || skb_clone_writable(skb, write_len))
		return 0;

	return pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
}

static void rmnet_wlan_inet_proto_csum_replace4(__sum16 *sum,
						struct sk_buff *skb,
						__be32 from, __be32 to,
						bool pseudohdr)
{
	if (skb->ip_summed != CHECKSUM_PARTIAL) {
		csum_replace4(sum, from, to);
		if (skb->ip_summed == CHECKSUM_COMPLETE && pseudohdr) {
			skb->csum = csum_sub(~(skb->csum), (__wsum)from);
			skb->csum = ~csum_add(skb->csum, (__wsum)to);
		}
	} else if (pseudohdr) {
		__wsum tmp = csum_unfold(*sum);

		tmp = csum_sub(tmp, (__wsum)from);
		tmp = csum_add(tmp, (__wsum)to);
		*sum = ~csum_fold(tmp);
	}
}

static void rmnet_wlan_inet_proto_csum_replace2(__sum16 *sum,
						struct sk_buff *skb,
						__be16 from, __be16 to,
						bool pseudohdr)
{
	rmnet_wlan_inet_proto_csum_replace4(sum, skb, (__be32)from, (__be32)to,
					    pseudohdr);
}

static unsigned int rmnet_wlan_tcp_optlen(u8 *opt, unsigned int off)
{
	if (opt[off] == TCPOPT_EOL || opt[off] == TCPOPT_NOP || !opt[off + 1])
		/* Skip over the byte */
		return 1;

	return opt[off + 1];
}

void rmnet_wlan_tcp_mss_clamp(struct sk_buff *skb, u32 tcp_flags)
{
	struct tcphdr *th;
	u8 __th[60];
	u8 *opt;
	__be16 mss_clamp = htons(1140);
	int ip_len;
	u16 tcp_len;
	unsigned int i;
	__be32 word;
	u8 ip_proto;

	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph, __iph;

		ip_proto = 4;
		iph = skb_header_pointer(skb, 0, sizeof(*iph), &__iph);
		if (!iph)
			goto err;

		if (iph->protocol != IPPROTO_TCP)
			goto skip;

		ip_len = iph->ihl * 4;
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		struct ipv6hdr *ip6h, __ip6h;
		__be16 frag_off;
		u8 prot;

		ip_proto = 6;
		ip6h = skb_header_pointer(skb, 0, sizeof(*ip6h), &__ip6h);
		if (!ip6h)
			goto err;

		prot = ip6h->nexthdr;
		ip_len = ipv6_skip_exthdr(skb, sizeof(*ip6h), &prot,
					  &frag_off);
		if (ip_len < 0)
			goto err;

		if (prot != IPPROTO_TCP)
			goto skip;
	} else {
		goto err;
	}

	th = skb_header_pointer(skb, ip_len, sizeof(*th), &__th);
	if (!th)
		goto err;

	if (!(tcp_flag_word(th) & tcp_flags))
		goto skip;

	tcp_len = th->doff * 4;
	/* We need to write data to this SKB. Make sure we can */
	if (rmnet_wlan_skb_ensure_writable(skb, ip_len + tcp_len))
		goto err;

	th = skb_header_pointer(skb, ip_len, tcp_len, &__th);
	if (!th)
		goto err;

	opt = (u8 *)th;
	for (i = sizeof(*th); i < tcp_len; i += rmnet_wlan_tcp_optlen(opt, i)) {
		__be16 *mss;

		/* MSS Option present && valid? */
		if (opt[i] != TCPOPT_MSS || opt[i + 1] != TCPOLEN_MSS)
			continue;

		mss = (__be16 *)(&opt[i + 2]);
		if (*mss <= mss_clamp)
			/* Already clamped */
			goto skip;

		/* Update tcp checksum and MSS */
		rmnet_wlan_inet_proto_csum_replace2(&th->check, skb, *mss,
						    mss_clamp, false);
		*mss = mss_clamp;
		rmnet_wlan_stats_update(RMNET_WLAN_STAT_MSS_CLAMP);
		return;
	}

	/* No MSS. We can add it only if there's no packet data and the TCP
	 * header isn't already full.
	 */
	if (skb->len > ip_len + tcp_len || tcp_len >= 60)
		goto skip;

	if (skb_tailroom(skb) < TCPOLEN_MSS) {
		if (pskb_expand_head(skb, 0, TCPOLEN_MSS - skb_tailroom(skb),
				     GFP_ATOMIC))
			goto err;

		/* expand head trashes headers. reload */
		th = skb_header_pointer(skb, ip_len, tcp_len, &__th);
		if (!th)
			goto err;
	}

	/* Update pesudoheader checksum for new packet length */
	rmnet_wlan_inet_proto_csum_replace2(&th->check, skb,
					    htons(skb->len - ip_len),
					    htons(skb->len - ip_len +
						  TCPOLEN_MSS),
					    true);
	opt = skb_put(skb, TCPOLEN_MSS);
	/* TCP packets with no MSS option are assumed to use the default MSS.
	 * Use appropriate minimum of our clamp and the protocol default.
	 */
	if (ip_proto == 4)
		mss_clamp = htons(min_t(u16, ntohs(mss_clamp), 536));
	else
		mss_clamp = htons(min_t(u16, ntohs(mss_clamp), 1220));

	/* Add option and update checksum */
	opt[0] = TCPOPT_MSS;
	opt[1] = TCPOLEN_MSS;
	*((__be16 *)(opt + 2)) = mss_clamp;
	rmnet_wlan_inet_proto_csum_replace4(&th->check, skb, 0,
					    *((__be32 *)opt), false);

	/* Update TCP data offset and checksum */
	word = tcp_flag_word(th);
	th->doff++;
	rmnet_wlan_inet_proto_csum_replace4(&th->check, skb, word,
					    tcp_flag_word(th), false);

	/* Update IP packet length (and checksum for v4) */
	if (ip_proto == 4) {
		struct iphdr *iph = ip_hdr(skb);

		csum_replace2(&iph->check, iph->tot_len, htons(skb->len));
		iph->tot_len = htons(skb->len);
	} else {
		struct ipv6hdr *ip6h = ipv6_hdr(skb);

		ip6h->payload_len = htons(ntohs(ip6h->payload_len) +
					  TCPOLEN_MSS);
	}

	rmnet_wlan_stats_update(RMNET_WLAN_STAT_MSS_CLAMP);
	return;

skip:
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_MSS_CLAMP_SKIP);
	return;

err:
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_MSS_CLAMP_ERR);
	return;
}

int rmnet_wlan_deliver_skb(struct sk_buff *skb,
			   struct rmnet_wlan_fwd_info *info)
{
	struct net_device *device;

	device = READ_ONCE(info->fwd_dev);
	if (!device) {
		rmnet_wlan_stats_update(RMNET_WLAN_STAT_PKT_FWD_NO_DEV);
		return -ENODEV;
	}

	/* clamp down any forwarded syn-acks */
	rmnet_wlan_tcp_mss_clamp(skb, TCP_FLAG_SYN | TCP_FLAG_ACK);
	skb->dev = device;
	dev_queue_xmit(skb);
	rmnet_wlan_stats_update(RMNET_WLAN_STAT_PKT_FWD);
	return 0;
}

bool rmnet_wlan_tuple_present(struct rmnet_wlan_tuple *tuple)
	__must_hold(RCU)
{
	struct rmnet_wlan_tuple_node *node;

	hash_for_each_possible_rcu(rmnet_wlan_tuple_hash, node, hash,
				   tuple->port) {
		if (rmnet_wlan_tuple_match(&node->info, tuple))
			return true;
	}

	return false;
}

int rmnet_wlan_add_tuples(struct rmnet_wlan_tuple *tuples, u32 tuple_count,
			  struct genl_info *info)
{
	int err = 0;
	u32 i;

	mutex_lock(&rmnet_wlan_tuple_mutex);
	for (i = 0; i < tuple_count; i++) {
		err = __rmnet_wlan_add_tuple(&tuples[i], info);
		if (err) {
			if (err == -EEXIST)
				/* Not fatal. Just skip over it */
				err = 0;
			else
				break;
		} else {
			rmnet_wlan_tuple_count++;
		}
	}

	mutex_unlock(&rmnet_wlan_tuple_mutex);
	return err;
}

int rmnet_wlan_del_tuples(struct rmnet_wlan_tuple *tuples, u32 tuple_count,
			  struct genl_info *info)
{
	int err = 0;
	u32 i;

	mutex_lock(&rmnet_wlan_tuple_mutex);
	for (i = 0; i < tuple_count; i++) {
		err = __rmnet_wlan_del_tuple(&tuples[i], info);
		if (err) {
			if (err == -ESRCH)
				/* Not fatal. Skip it and move on */
				err = 0;
			else
				break;
		} else {
			rmnet_wlan_tuple_count--;
		}
	}

	mutex_unlock(&rmnet_wlan_tuple_mutex);
	return err;
}

int rmnet_wlan_set_device(char *dev_name, struct genl_info *info)
{
	int err;

	if (!rmnet_wlan_strlcmp(rmnet_wlan_device, dev_name, IFNAMSIZ)) {
		GENL_SET_ERR_MSG(info, "Device is already set");
		/* Error? Technically I did what you told me, it just took no
		 * work on my part...
		 */
		return 0;
	}

	/* Bring down the old notifier. This forces the register events to be
	 * replayed for the new device if it's already up when we register
	 * a new notifier below.
	 */
	err = __rmnet_wlan_unset_device();
	if (err) {
		GENL_SET_ERR_MSG(info,
				 "Kernel error, notifier unregister failed");
		return err;
	}

	/* Begone, foul security CR!
	 * The power of NULL-termination compels you!
	 * *flailing arms uselessly*
	 */
	strscpy(rmnet_wlan_device, dev_name, IFNAMSIZ);
	err = register_netdevice_notifier(&rmnet_wlan_notifier);
	if (err) {
		GENL_SET_ERR_MSG(info, "Kernel error, notifier failed");
		rmnet_wlan_device[0] = 0;
	}

	return err;
}

int rmnet_wwan_set_device(char *dev_name, struct genl_info *info)
{
	int err;

	if (!rmnet_wlan_strlcmp(rmnet_wwan_device, dev_name, IFNAMSIZ)) {
		GENL_SET_ERR_MSG(info, "Device is already set");
		/* Error? Technically I did what you told me, it just took no
		 * work on my part...
		 */
		return 0;
	}

	/* Bring down the old notifier. This forces the register events to be
	 * replayed for the new device if it's already up when we register
	 * a new notifier below.
	 */
	err = __rmnet_wwan_unset_device();
	if (err) {
		GENL_SET_ERR_MSG(info,
				 "Kernel error, notifier unregister failed");
		return err;
	}

	/* Begone, foul security CR!
	 * The power of NULL-termination compels you!
	 * *flailing arms uselessly*
	 */
	strscpy(rmnet_wwan_device, dev_name, IFNAMSIZ);
	return err;
}

int rmnet_wlan_unset_device(char *dev_name, struct genl_info *info)
{
	return __rmnet_wlan_unset_device();
}

int rmnet_wwan_unset_device(char *dev_name, struct genl_info *info)
{
	return __rmnet_wwan_unset_device();
}

int rmnet_wlan_add_fwd_info(struct rmnet_wlan_fwd_info *fwd_info,
			    struct genl_info *info)
{
	int err;

	mutex_lock(&rmnet_wlan_fwd_mutex);
	err = __rmnet_wlan_add_fwd_info(fwd_info, info);
	mutex_unlock(&rmnet_wlan_fwd_mutex);
	return err;
}

int rmnet_wlan_del_fwd_info(struct rmnet_wlan_fwd_info *fwd_info,
			    struct genl_info *info)
{
	int err;

	mutex_lock(&rmnet_wlan_fwd_mutex);
	err = __rmnet_wlan_del_fwd_info(fwd_info, info);
	mutex_unlock(&rmnet_wlan_fwd_mutex);
	return err;
}

int rmnet_wlan_set_encap_port(__be16 port, struct genl_info *info)
{
	int err;

	mutex_lock(&rmnet_wlan_encap_mutex);
	err = __rmnet_wlan_add_encap_port(port, info);
	mutex_unlock(&rmnet_wlan_encap_mutex);
	return err;
}

int rmnet_wlan_unset_encap_port(__be16 port, struct genl_info *info)
{
	int err;

	mutex_lock(&rmnet_wlan_encap_mutex);
	err = __rmnet_wlan_del_encap_port(port, info);
	mutex_unlock(&rmnet_wlan_encap_mutex);
	return err;
}

bool rmnet_wlan_udp_encap_check(struct sk_buff *skb,
				struct rmnet_wlan_tuple *tuple,
				int ip_len)
{
	struct udphdr *up, __up;
	__be32 *payload, __payload;

	if (tuple->trans_proto != IPPROTO_UDP ||
	    !rmnet_wlan_encap_port_present(tuple->port))
		return false;

	up = skb_header_pointer(skb, ip_len, sizeof(*up), &__up);
	if (!up)
		return false;

	/* If SRC port is 500 and dest port matches the encap, this is
	 * a non-encap IKE response that must be forwarded to modem */
	if (up->source == htons(500))
		return false;

	payload = skb_header_pointer(skb, ip_len + sizeof(*up),
				     sizeof(*payload), &__payload);
	if (!payload)
		return false;

	return !!(*payload);
}

bool rmnet_wlan_udp_encap_drop_check(struct rmnet_wlan_tuple *tuple)
{
	return rmnet_wlan_encap_port_drop_present(tuple->port);
}

int rmnet_wlan_add_ll_tuple(struct rmnet_wlan_ll_tuple *tuple)
{
	int err;

	mutex_lock(&rmnet_wlan_ll_tuple_mutex);
	err = __rmnet_wlan_add_ll_tuple(tuple);
	mutex_unlock(&rmnet_wlan_ll_tuple_mutex);
	return err;
}

int rmnet_wlan_del_ll_tuple(void)
{
	int err;

	mutex_lock(&rmnet_wlan_ll_tuple_mutex);
	err = __rmnet_wlan_del_ll_tuple();
	mutex_unlock(&rmnet_wlan_ll_tuple_mutex);
	return err;
}

static int rmnet_wlan_write_tuple(struct sk_buff *skb, struct rmnet_wlan_tuple *tuple)
{
	struct nlattr *inner_attr;

	inner_attr = nla_nest_start(skb, RMNET_WLAN_GENL_ATTR_TUPLES);
	if (!inner_attr)
		return -EINVAL;

	if (nla_put(skb, RMNET_WLAN_GENL_TUPLE_ATTR_TUPLE, sizeof(*tuple), tuple)) {
		/* In the case we fail to put something cancel the nesting */
		nla_nest_cancel(skb, inner_attr);
		return -EINVAL;
	}

	nla_nest_end(skb, inner_attr);

	return 0;
}

int rmnet_wlan_get_tuples(struct sk_buff **pskb, struct genl_family *fam,
			  struct genl_info *info)
{
	struct rmnet_wlan_tuple_node *node;
	struct sk_buff *skb_out = NULL;
	struct nlattr *attr;
	int msg_size;
	int err = 0;
	void *hdr;
	int bkt;

	mutex_lock(&rmnet_wlan_tuple_mutex);

	msg_size = nla_total_size(nla_total_size(sizeof(node->info))
				  * rmnet_wlan_tuple_count);

	/* Create the skb based on the number tuples being sent */
	skb_out = genlmsg_new(msg_size, GFP_KERNEL);
	if (!skb_out) {
		err = -ENOMEM;
		goto err0;
	}

	hdr = genlmsg_put_reply(skb_out, info, fam, 0,
				RMNET_WLAN_GENL_CMD_GET_TUPLES);
	if (!hdr) {
		err = -EINVAL;
		goto err1;
	}

	attr = nla_nest_start(skb_out, RMNET_WLAN_GENL_ATTR_TUPLES);
	if (!attr) {
		err = -EINVAL;
		goto err1;
	}

	/* Iterate thru the hash table even if it's empty */
	hash_for_each(rmnet_wlan_tuple_hash, bkt, node, hash)
		rmnet_wlan_write_tuple(skb_out, &node->info);

	nla_nest_end(skb_out, attr);
	genlmsg_end(skb_out, hdr);

	*pskb = skb_out;
	goto err0;

err1:
	kfree(skb_out);
err0:
	mutex_unlock(&rmnet_wlan_tuple_mutex);

	return err;
}

int rmnet_wlan_reset(void)
{
	rmnet_wlan_cleanup();
	return 0;
}

void rmnet_wlan_deinit(void)
{
	rmnet_wlan_cleanup();
}

char *rmnet_wlan_get_dev(void)
{
	return rmnet_wlan_device;
}

char *rmnet_wwan_get_dev(void)
{
	return rmnet_wwan_device;
}

int rmnet_wlan_strlcmp(const char *string1, const char *string2,
		       size_t limit_bytes)
{
	while (limit_bytes--) {
		if (*string1 != *string2)
			return ((unsigned char)*string1 - (unsigned char)*string2);

		string1++;
		string2++;

		if (!(*string1))
			break;
	}

	return 0;
}
