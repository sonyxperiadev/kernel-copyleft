/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __RMNET_OFFLOAD_H__
#define __RMNET_OFFLOAD_H__

#include <linux/types.h>
#include <asm/byteorder.h>
#include "rmnet_descriptor.h"
#include "rmnet_map.h"

#define RMNET_OFFLOAD_PROTO_TCP		6
#define RMNET_OFFLOAD_PROTO_UDP		17
#define RMNET_OFFLOAD_PROTO_FRAGMENT	44

struct rmnet_offload_iphdr {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8 roip_ihl:4;
	u8 roip_version:4;
#elif defined (__BIG_ENDIAN_BITFIELD)
	u8 roip_version:4;
	u8 roip_ihl:4;
#else
#error "<asm/byteorder.h> error"
#endif
	u8 roip_tos;
	__be16 roip_tot_len;
	__be16 roip_id;
	__be16 roip_frag_off;
	u8 roip_ttl;
	u8 roip_protocol;
	__be16 roip_check;
	__be32 roip_saddr;
	__be32 roip_daddr;
};

struct rmnet_offload_ipv6hdr {
	/* rmnet_offload doesn't care about the version field. So honestly,
	 * it's easier to just take the whole 32 bits as the flow label
	 */
	__be32 roipv6_flow_lbl;
	__be16 roipv6_payload_len;
	u8 roipv6_nexthdr;
	u8 roipv6_hop_limit;
	__be32 roipv6_saddr[4];
	__be32 roipv6_daddr[4];
};

struct rmnet_offload_tcphdr {
	__be16 rotcp_source;
	__be16 rotcp_dest;
	__be32 rotcp_seq;
	__be32 rotcp_ack;
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8 rotcp_res:4;
	u8 rotcp_doff:4;
#elif defined (__BIG_ENDIAN_BITFIELD)
	u8 rotcp_doff:4;
	u8 rotcp_res:4;
#else
#error "<asm/byteorder.h> error"
#endif
	u8 rotcp_flags;
	__be16 rotcp_window;
	__be16 rotcp_check;
	__be16 rotcp_urg;
};

struct rmnet_offload_udphdr {
	__be16 roudp_source;
	__be16 roudp_dest;
	__be16 roudp_len;
	__be16 roudp_check;
};

struct rmnet_offload_header_info {
	/* Port information */
	__be16 roh_sport;
	__be16 roh_dport;

	/* Address information */
	union {
		__be32 roh_saddr4;
		__be32 roh_saddr6[4];
	};

	union {
		__be32 roh_daddr4;
		__be32 roh_daddr6[4];
	};

	/* Header flags */
	union {
		struct {
			u8 roh_ip_ttl;
			u8 roh_ip_tos;
			__be16 roh_ip_frag_off;
		};
		__be32 roh_flag_word;
	};

	/* TCP sequence number. Both the flow and the pkt info structs need
	 * this value at various times, so it makes sense to put it in this
	 * shared struct.
	 */
	u32 roh_tcp_seq;

	/* Header lengths and protocols */
	u16 roh_ip_len;
	u16 roh_trans_len;
	u8 roh_ip_proto;
	u8 roh_trans_proto;
};

struct rmnet_offload_info {
	struct rmnet_frag_descriptor *roi_frag_desc;

	/* Packet headers */
	struct rmnet_offload_header_info roi_hdrs;

	/* 5 tuple hash key */
	u32 roi_hash_key;

	/* Payload length */
	u16 roi_payload_len;

	/* Packet meta information */
	bool roi_first_pkt;
	bool roi_skip_hash;
	bool roi_len_mismatch;
};

void rmnet_offload_lock(void);
void rmnet_offload_unlock(void);
void rmnet_offload_set_hooks(void);
void rmnet_offload_unset_hooks(void);
void rmnet_offload_deliver_descs(struct list_head *desc_list);
void rmnet_offload_flush_current_pkt(struct rmnet_offload_info *pkt_info,
				     struct list_head *flush_list);
void rmnet_offload_handle_powersave_on(void *port);
void rmnet_offload_handle_powersave_off(void *port);
void
rmnet_offload_handle_dl_header(struct rmnet_map_dl_ind_hdr *dlhdr,
			       struct rmnet_map_control_command_header *cmd);
void
rmnet_offload_handle_dl_trailer(struct rmnet_map_dl_ind_trl *dltrl,
				struct rmnet_map_control_command_header *cmd);

#endif
