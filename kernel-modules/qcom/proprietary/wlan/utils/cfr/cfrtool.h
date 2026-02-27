/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

/**
 * DOC: dump data for channel frequency response capture
 * This file provides tool to dump cfr capture data from driver
 */

#ifndef _CFR_TOOL_H
#define _CFR_TOOL_H


typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

#define MAC_ADDR_SIZE 6
#define MAX_CHAINS 8 /* Max Tx/Rx chains */
#define MAX_CFR_MU_USERS 4

struct cfr_metadata {
	uint8_t    status;
	uint8_t    capture_bw;
	uint8_t    channel_bw;
	uint8_t    phy_mode;
	uint16_t   prim20_chan;
	uint16_t   center_freq1;
	uint16_t   center_freq2;
	uint8_t    capture_mode; /* ack_capture_mode */
	uint8_t    capture_type; /* cfr_capture_type */
	uint8_t    sts_count;
	uint8_t    num_rx_chain;
	uint64_t   timestamp;
	uint32_t   length;
	uint8_t    is_mu_ppdu;
	uint8_t    num_mu_users;
	union {
		uint8_t    su_peer_addr[MAC_ADDR_SIZE];
		uint8_t    mu_peer_addr[MAX_CFR_MU_USERS][MAC_ADDR_SIZE];
	} peer_addr;
	uint32_t   chain_rssi[MAX_CHAINS];
	uint16_t   chain_phase[MAX_CHAINS];
} __attribute__ ((__packed__));

struct cfr_header {
	uint32_t   start_magic_num;
	uint32_t   vendorid;
	uint8_t    cfr_metadata_version;
	uint8_t    cfr_data_version;
	uint8_t    chip_type;
	uint8_t    pltform_type;
	uint32_t   Reserved;
	struct cfr_metadata meta;
} __attribute__ ((__packed__));

struct nl80211_ctx {
	struct nl_sock *cmd_sock;
	struct nl_sock *event_sock;
	int32_t netlink_familyid;
};

int startCapture();
int stopCapture();

#endif
