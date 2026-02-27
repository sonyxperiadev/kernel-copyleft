/**
  * Copyright (c) 2023 Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  */

#ifndef _QTI_WIFI_H_
#define _QTI_WIFI_H_

#define NUM_INTERFACES_MAX     4
#define LEN_INTERFACE_NAME_MAX 6

#define QTIWIFI_EVENT_THERMAL_CHANGE "CTRL-EVENT-THERMAL-CHANGED "
#define QTIWIFI_EVENT_CONGESTION_REPORT "CTRL-EVENT-CONGESTION-REPORT "

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define ETH_ALEN 6

struct qtiwifi_global {
	int ioctl_sock_fd;
	struct nl_sock *cmd_sock;
	struct nl_sock *event_sock;
	struct nl_cb *cb;
	int32_t nl80211_id;
	void (*ctrl_event_qtiwifi_aidl_cb)(const char *ctx, const char* msg);
};

struct qtiwifi_iface {
	uint32_t type;
	char name[LEN_INTERFACE_NAME_MAX];
};

struct sta_info {
	uint32_t reason;
};

struct family_data {
	const char *group;
	int id;
};

extern struct qtiwifi_iface qtiwifi_interfaces[NUM_INTERFACES_MAX];
extern int qtiwifi_num_ifaces;
extern struct qtiwifi_global *global;

/* common util func */

int ack_handler(struct nl_msg *msg, void *arg);

int finish_handler(struct nl_msg *msg, void *arg);

int no_seq_check(struct nl_msg *msg, void *arg);

int error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err, void *arg);

void nl80211_nlmsg_clear(struct nl_msg *msg);

int send_and_recv(struct qtiwifi_global* ctx, struct nl_msg* msg,
	int (*valid_handler)(struct nl_msg*, void*), void *valid_data);

/* qtiwifi func */

/* process commands from framework */
int qtiwifi_process_cmd(struct qtiwifi_global *global, const char* ifname,
	const char* cmd, char* reply, int reply_cap);

/* get available sta/ap interfaces */
int qtiwifi_get_interfaces(void);

/* process events reported from sta/ap interfaces */
int qtiwifi_process_driver_events(struct nl_msg *msg, void *arg);

/* receive events reported from sta/ap interfaces */
void qtiwifi_driver_nl80211_event_receive(int sock,
	void *eloop_ctx, void *handle);

/* process command with "DRIVER" prefix */
int qtiwifi_do_driver_cmd(struct qtiwifi_global *global, const char* ifname,
	const char* cmd, char* reply, int reply_cap);

#endif
