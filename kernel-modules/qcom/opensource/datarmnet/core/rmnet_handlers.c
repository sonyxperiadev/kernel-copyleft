/* Copyright (c) 2013-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * RMNET Data ingress/egress handler
 *
 */

#include <linux/netdevice.h>
#include <linux/netdev_features.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/inet.h>
#include <net/sock.h>
#include <linux/tracepoint.h>
#include "rmnet_private.h"
#include "rmnet_config.h"
#include "rmnet_vnd.h"
#include "rmnet_map.h"
#include "rmnet_handlers.h"
#include "rmnet_descriptor.h"
#include "rmnet_ll.h"
#include "rmnet_module.h"


#include "rmnet_qmi.h"
#include "qmi_rmnet.h"

#define RMNET_IP_VERSION_4 0x40
#define RMNET_IP_VERSION_6 0x60
#define CREATE_TRACE_POINTS
#include "rmnet_trace.h"

EXPORT_TRACEPOINT_SYMBOL(rmnet_shs_low);
EXPORT_TRACEPOINT_SYMBOL(rmnet_shs_high);
EXPORT_TRACEPOINT_SYMBOL(rmnet_shs_err);
EXPORT_TRACEPOINT_SYMBOL(rmnet_shs_wq_low);
EXPORT_TRACEPOINT_SYMBOL(rmnet_shs_wq_high);
EXPORT_TRACEPOINT_SYMBOL(rmnet_shs_wq_err);
EXPORT_TRACEPOINT_SYMBOL(rmnet_perf_low);
EXPORT_TRACEPOINT_SYMBOL(rmnet_perf_high);
EXPORT_TRACEPOINT_SYMBOL(rmnet_perf_err);
EXPORT_TRACEPOINT_SYMBOL(rmnet_low);
EXPORT_TRACEPOINT_SYMBOL(rmnet_high);
EXPORT_TRACEPOINT_SYMBOL(rmnet_err);
EXPORT_TRACEPOINT_SYMBOL(rmnet_freq_update);
EXPORT_TRACEPOINT_SYMBOL(rmnet_freq_reset);
EXPORT_TRACEPOINT_SYMBOL(rmnet_freq_boost);
EXPORT_TRACEPOINT_SYMBOL(print_icmp_rx);



/* Helper Functions */

void rmnet_set_skb_proto(struct sk_buff *skb)
{
	switch (rmnet_map_data_ptr(skb)[0] & 0xF0) {
	case RMNET_IP_VERSION_4:
		skb->protocol = htons(ETH_P_IP);
		break;
	case RMNET_IP_VERSION_6:
		skb->protocol = htons(ETH_P_IPV6);
		break;
	default:
		skb->protocol = htons(ETH_P_MAP);
		break;
	}
}
EXPORT_SYMBOL(rmnet_set_skb_proto);

/* Generic handler */

void
rmnet_deliver_skb(struct sk_buff *skb, struct rmnet_port *port)
{
	trace_rmnet_low(RMNET_MODULE, RMNET_DLVR_SKB, 0xDEF, 0xDEF,
			0xDEF, 0xDEF, (void *)skb, NULL);
	skb_reset_network_header(skb);
	rmnet_vnd_rx_fixup(skb->dev, skb->len);

	skb->pkt_type = PACKET_HOST;
	skb_set_mac_header(skb, 0);

	/* Low latency packets use a different balancing scheme */
	if (skb->priority == 0xda1a)
		goto skip_shs;

	if (rmnet_module_hook_shs_skb_entry(NULL, skb, &port->shs_cfg))
		return;

skip_shs:
	if (rmnet_module_hook_shs_skb_ll_entry(NULL, skb, &port->shs_cfg))
		return;

	netif_receive_skb(skb);
}
EXPORT_SYMBOL(rmnet_deliver_skb);

/* Important to note, port cannot be used here if it has gone stale */
void
rmnet_deliver_skb_wq(struct sk_buff *skb, struct rmnet_port *port,
		     enum rmnet_packet_context ctx)
{
	struct rmnet_priv *priv = netdev_priv(skb->dev);

	trace_rmnet_low(RMNET_MODULE, RMNET_DLVR_SKB, 0xDEF, 0xDEF,
			0xDEF, 0xDEF, (void *)skb, NULL);
	skb_reset_transport_header(skb);
	skb_reset_network_header(skb);
	rmnet_vnd_rx_fixup(skb->dev, skb->len);

	skb->pkt_type = PACKET_HOST;
	skb_set_mac_header(skb, 0);

	/* packets coming from work queue context due to packet flush timer
	 * must go through the special workqueue path in SHS driver
	 */
	if (!ctx) {
		if (rmnet_module_hook_shs_skb_entry(NULL, skb, &port->shs_cfg))
			return;
	}

	if (ctx == RMNET_NET_RX_CTX)
		netif_receive_skb(skb);
	else
		gro_cells_receive(&priv->gro_cells, skb);
}
EXPORT_SYMBOL(rmnet_deliver_skb_wq);

/* Deliver a list of skbs after undoing coalescing */
static void rmnet_deliver_skb_list(struct sk_buff_head *head,
				   struct rmnet_port *port)
{
	struct sk_buff *skb;

	while ((skb = __skb_dequeue(head))) {
		rmnet_set_skb_proto(skb);
		rmnet_deliver_skb(skb, port);
	}
}

/* MAP handler */

static void
__rmnet_map_ingress_handler(struct sk_buff *skb,
			    struct rmnet_port *port)
{
	struct rmnet_map_header *qmap;
	struct rmnet_endpoint *ep;
	struct sk_buff_head list;
	u16 len, pad;
	u8 mux_id;

	/* We don't need the spinlock since only we touch this */
	__skb_queue_head_init(&list);

	qmap = (struct rmnet_map_header *)rmnet_map_data_ptr(skb);
	if (qmap->cd_bit) {
		qmi_rmnet_set_dl_msg_active(port);
		if (port->data_format & RMNET_INGRESS_FORMAT_DL_MARKER) {
			if (!rmnet_map_flow_command(skb, port, false))
				return;
		}

		if (port->data_format & RMNET_FLAGS_INGRESS_MAP_COMMANDS)
			return rmnet_map_command(skb, port);

		goto free_skb;
	}

	mux_id = qmap->mux_id;
	pad = qmap->pad_len;
	len = ntohs(qmap->pkt_len) - pad;

	if (mux_id >= RMNET_MAX_LOGICAL_EP)
		goto free_skb;

	ep = rmnet_get_endpoint(port, mux_id);
	if (!ep)
		goto free_skb;

	skb->dev = ep->egress_dev;

	/* Handle QMAPv5 packet */
	if (qmap->next_hdr &&
	    (port->data_format & (RMNET_FLAGS_INGRESS_COALESCE |
				  RMNET_PRIV_FLAGS_INGRESS_MAP_CKSUMV5))) {
		if (rmnet_map_process_next_hdr_packet(skb, &list, len))
			goto free_skb;
	} else {
		/* We only have the main QMAP header to worry about */
		pskb_pull(skb, sizeof(*qmap));

		rmnet_set_skb_proto(skb);

		if (port->data_format & RMNET_FLAGS_INGRESS_MAP_CKSUMV4) {
			if (!rmnet_map_checksum_downlink_packet(skb, len + pad))
				skb->ip_summed = CHECKSUM_UNNECESSARY;
		}

		pskb_trim(skb, len);

		/* Push the single packet onto the list */
		__skb_queue_tail(&list, skb);
	}

	if (port->data_format & RMNET_INGRESS_FORMAT_PS)
		qmi_rmnet_work_maybe_restart(port, NULL, skb_peek(&list));

	rmnet_deliver_skb_list(&list, port);
	return;

free_skb:
	kfree_skb(skb);
}

static void
rmnet_map_ingress_handler(struct sk_buff *skb,
			  struct rmnet_port *port)
{
	int is_chained_skb = 0;
	struct sk_buff *skbn, *head_skb;

	if (skb->dev->type == ARPHRD_ETHER) {
		if (pskb_expand_head(skb, ETH_HLEN, 0, GFP_ATOMIC)) {
			kfree_skb(skb);
			return;
		}

		skb_push(skb, ETH_HLEN);
	}

	if (port->data_format & (RMNET_FLAGS_INGRESS_COALESCE |
				 RMNET_PRIV_FLAGS_INGRESS_MAP_CKSUMV5)) {
		if (skb_is_nonlinear(skb)) {
			rmnet_frag_ingress_handler(skb, port);
			return;
		}
	}

	/* No aggregation. Pass the frame on as is */
	if (!(port->data_format & RMNET_FLAGS_INGRESS_DEAGGREGATION)) {
		__rmnet_map_ingress_handler(skb, port);
		return;
	}

	/* Deaggregation and freeing of HW originating
	 * buffers is done within here
	 */
	head_skb = skb;
	while (skb) {
		struct sk_buff *skb_frag = skb_shinfo(skb)->frag_list;
		struct sk_buff *skb_next = skb->next;

		skb_shinfo(skb)->frag_list = NULL;
		skb->next = NULL;
		while ((skbn = rmnet_map_deaggregate(skb, port)) != NULL) {
			__rmnet_map_ingress_handler(skbn, port);

			if (skbn == skb)
				goto next_skb;
		}

		consume_skb(skb);
next_skb:
		if (head_skb == skb) {
			skb = skb_frag;
			if (skb_frag) {
				is_chained_skb = 1;
				port->stats.chained_packets_recvd++;
			}
		} else  {
			skb = skb_next;
			/* No longer expected to use older skb chain format */
			WARN_ON(skb_frag != NULL);
		}

		if (is_chained_skb)
			port->stats.packets_chained++;
	}
}

static int rmnet_map_egress_handler(struct sk_buff *skb,
				    struct rmnet_port *port, u8 mux_id,
				    struct net_device *orig_dev,
				    bool low_latency)
{
	int required_headroom, additional_header_len, csum_type, tso = 0;
	struct rmnet_map_header *map_header;
	struct rmnet_aggregation_state *state;

	additional_header_len = 0;
	required_headroom = sizeof(struct rmnet_map_header);
	csum_type = 0;

	if (port->data_format & RMNET_FLAGS_EGRESS_MAP_CKSUMV4) {
		additional_header_len = sizeof(struct rmnet_map_ul_csum_header);
		csum_type = RMNET_FLAGS_EGRESS_MAP_CKSUMV4;
	} else if ((port->data_format & RMNET_PRIV_FLAGS_EGRESS_MAP_CKSUMV5) ||
		   (port->data_format & RMNET_EGRESS_FORMAT_PRIORITY)) {
		additional_header_len = sizeof(struct rmnet_map_v5_csum_header);
		csum_type = RMNET_PRIV_FLAGS_EGRESS_MAP_CKSUMV5;
	}

	required_headroom += additional_header_len;

	if (skb_headroom(skb) < required_headroom) {
		if (pskb_expand_head(skb, required_headroom, 0, GFP_ATOMIC))
			return -ENOMEM;
	}

	if (port->data_format & RMNET_INGRESS_FORMAT_PS)
		qmi_rmnet_work_maybe_restart(port, NULL, NULL);

	state = &port->agg_state[(low_latency) ? RMNET_LL_AGG_STATE :
				 RMNET_DEFAULT_AGG_STATE];

	if (csum_type &&
	    (skb_shinfo(skb)->gso_type & (SKB_GSO_UDP_L4 | SKB_GSO_TCPV4 | SKB_GSO_TCPV6)) &&
	     skb_shinfo(skb)->gso_size) {
		spin_lock_bh(&state->agg_lock);
		rmnet_map_send_agg_skb(state);

		if (rmnet_map_add_tso_header(skb, port, orig_dev))
			return -EINVAL;
		csum_type = 0;
		tso = 1;
	}

	if (csum_type)
		rmnet_map_checksum_uplink_packet(skb, port, orig_dev,
						 csum_type);

	map_header = rmnet_map_add_map_header(skb, additional_header_len, 0,
					      port);
	if (!map_header)
		return -ENOMEM;

	map_header->mux_id = mux_id;

	if (port->data_format & RMNET_EGRESS_FORMAT_AGGREGATION) {
		if (state->params.agg_count < 2 ||
		    rmnet_map_tx_agg_skip(skb, required_headroom) || tso)
			goto done;

		rmnet_map_tx_aggregate(skb, port, low_latency);
		return -EINPROGRESS;
	}

done:
	skb->protocol = htons(ETH_P_MAP);
	return 0;
}

static void
rmnet_bridge_handler(struct sk_buff *skb, struct net_device *bridge_dev)
{
	if (bridge_dev) {
		skb->dev = bridge_dev;
		dev_queue_xmit(skb);
	}
}

/* Ingress / Egress Entry Points */

/* Processes packet as per ingress data format for receiving device. Logical
 * endpoint is determined from packet inspection. Packet is then sent to the
 * egress device listed in the logical endpoint configuration.
 */
rx_handler_result_t rmnet_rx_handler(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	struct rmnet_port *port;
	struct net_device *dev;
	int consumed;

	if (!skb)
		goto done;

	if (skb->pkt_type == PACKET_LOOPBACK)
		return RX_HANDLER_PASS;

	trace_rmnet_low(RMNET_MODULE, RMNET_RCV_FROM_PND, 0xDEF,
			0xDEF, 0xDEF, 0xDEF, NULL, NULL);
	dev = skb->dev;
	port = rmnet_get_port(dev);
	if (unlikely(!port)) {
		dev_core_stats_rx_nohandler_inc(skb->dev);
		kfree_skb(skb);
		goto done;
	}

	switch (port->rmnet_mode) {
	case RMNET_EPMODE_VND:
		if (rmnet_module_hook_shs_switch(&consumed, skb,
						 &port->phy_shs_cfg)) {
			if (consumed)
				return RX_HANDLER_CONSUMED;
		}

		rmnet_map_ingress_handler(skb, port);
		break;
	case RMNET_EPMODE_BRIDGE:
		rmnet_bridge_handler(skb, port->bridge_ep);
		break;
	}

done:
	return RX_HANDLER_CONSUMED;
}
EXPORT_SYMBOL(rmnet_rx_handler);

rx_handler_result_t rmnet_rx_priv_handler(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	rx_handler_result_t rc = RX_HANDLER_PASS;

	rmnet_module_hook_wlan_ingress_rx_handler(&rc, pskb);
	if (rc != RX_HANDLER_PASS)
		return rc;

	rmnet_module_hook_perf_ingress_rx_handler(skb);

	return RX_HANDLER_PASS;
}

/* Modifies packet as per logical endpoint configuration and egress data format
 * for egress device configured in logical endpoint. Packet is then transmitted
 * on the egress device.
 */
void rmnet_egress_handler(struct sk_buff *skb, bool low_latency)
{
	struct net_device *orig_dev;
	struct rmnet_port *port;
	struct rmnet_priv *priv;
	u8 mux_id;
	int err;
	u32 skb_len;

	trace_rmnet_low(RMNET_MODULE, RMNET_TX_UL_PKT, 0xDEF, 0xDEF, 0xDEF,
			0xDEF, (void *)skb, NULL);
	sk_pacing_shift_update(skb->sk, 8);

	orig_dev = skb->dev;
	priv = netdev_priv(orig_dev);
	skb->dev = priv->real_dev;
	mux_id = priv->mux_id;

	port = rmnet_get_port(skb->dev);
	if (!port)
		goto drop;

	skb_len = skb->len;
	err = rmnet_map_egress_handler(skb, port, mux_id, orig_dev,
				       low_latency);
	if (err == -ENOMEM || err == -EINVAL) {
		goto drop;
	} else if (err == -EINPROGRESS) {
		rmnet_vnd_tx_fixup(orig_dev, skb_len);
		return;
	}

	rmnet_vnd_tx_fixup(orig_dev, skb_len);

	if (low_latency) {
		if (rmnet_ll_send_skb(skb)) {
			/* Drop but no need to free. Above API handles that */
			this_cpu_inc(priv->pcpu_stats->stats.tx_drops);
			return;
		}
	} else {
		dev_queue_xmit(skb);
	}

	return;

drop:
	this_cpu_inc(priv->pcpu_stats->stats.tx_drops);
	kfree_skb(skb);
}
