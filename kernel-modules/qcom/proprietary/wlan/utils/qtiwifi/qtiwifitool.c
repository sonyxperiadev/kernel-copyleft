/**
  * Copyright (c) 2023 Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  */
/* Qualcomm Technologies, Inc. has chosen to take qtiwifitool.c subject to
 * the BSD license and terms.
 */

/*
 * Driver interaction with extended Linux CFG8021
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <log/log.h>
#include <net/if.h>
#include <errno.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/rtnetlink.h>
#include <netpacket/packet.h>
#include <netlink/netlink.h>
#include <netlink/socket.h>

#include "qca-vendor_copy.h"
#include "nl80211_copy.h"
#include "eloop.h"
#include "aidl.h"
#include "qtiwifitool.h"

struct qtiwifi_global *global = NULL;
struct qtiwifi_iface qtiwifi_interfaces[NUM_INTERFACES_MAX];
int qtiwifi_num_ifaces = 0;

int ack_handler(struct nl_msg *msg, void *arg)
{
	int *err = (int *)arg;

	*err = 0;
	return NL_STOP;
}

int finish_handler(struct nl_msg *msg, void *arg)
{
	int *ret = (int *)arg;

	*ret = 0;
	return NL_SKIP;
}

int no_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

int error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err,
			 void *arg)
{
	int *ret = (int *)arg;

	*ret = err->error;
	ALOGI("%s received : %d - %s", __func__,
		   err->error, strerror(err->error));
	return NL_SKIP;
}

void nl80211_nlmsg_clear(struct nl_msg *msg)
{
	/*
	 * Clear nlmsg data, e.g., to make sure key material is not left in
	 * heap memory for unnecessarily long time.
	 */
	if (msg) {
		struct nlmsghdr *hdr = nlmsg_hdr(msg);
		void *data = nlmsg_data(hdr);
		/*
		 * This would use nlmsg_datalen() or the older nlmsg_len() if
		 * only libnl were to maintain a stable API.. Neither will work
		 * with all released versions, so just calculate the length
		 * here.
		 */
		int len = hdr->nlmsg_len - NLMSG_HDRLEN;

		memset(data, 0, len);
	}
}

int send_and_recv(struct qtiwifi_global *ctx,
			 struct nl_msg *msg,
			 int (*valid_handler)(struct nl_msg *, void *),
			 void *valid_data)
{
	struct nl_cb *cb;
	int err = -ENOMEM, opt;

	if (!msg)
		return -ENOMEM;

	cb = nl_cb_clone(ctx->cb);
	if (!cb)
		goto out;

	/* try to set NETLINK_EXT_ACK to 1, ignoring errors */
	opt = 1;
	setsockopt(nl_socket_get_fd(ctx->cmd_sock), SOL_NETLINK,
		   NETLINK_EXT_ACK, &opt, sizeof(opt));

	/* try to set NETLINK_CAP_ACK to 1, ignoring errors */
	opt = 1;
	setsockopt(nl_socket_get_fd(ctx->cmd_sock), SOL_NETLINK,
		   NETLINK_CAP_ACK, &opt, sizeof(opt));

	err = nl_send_auto_complete(ctx->cmd_sock, msg);
	if (err < 0) {
		ALOGE("nl80211: nl_send_auto_complete() failed: %s\n",
			nl_geterror(err));
		/* Need to convert libnl error code to an errno value. For now,
		 * just hardcode this to EBADF; the real error reason is shown
		 * in that error print above. */
		err = -EBADF;
		goto out;
	}

	err = 1;

	nl_cb_err(cb, NL_CB_CUSTOM, error_handler, &err);
	nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &err);
	nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &err);

	if (valid_handler) {
		nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM,
			  valid_handler, valid_data);
	} else {
		ALOGE("handler not valid");
	}

	while (err > 0) {
		int res = nl_recvmsgs(ctx->cmd_sock, cb);

		if (res == -NLE_DUMP_INTR) {
			/* Most likely one of the nl80211 dump routines hit a
			 * case where internal results changed while the dump
			 * was being sent. The most common known case for this
			 * is scan results fetching while associated were every
			 * received Beacon frame from the AP may end up
			 * incrementing bss_generation. This
			 * NL80211_CMD_GET_SCAN case tries again in the caller;
			 * other cases (of which there are no known common ones)
			 * will stop and return an error. */
			ALOGE("nl80211: %s; convert to -EAGAIN\n",
				   nl_geterror(res));
			err = -EAGAIN;
		} else if (res < 0) {
			ALOGE("nl80211: %s->nl_recvmsgs failed: %d (%s)\n",
				   __func__, res, nl_geterror(res));
		}
	}

 out:
	nl_cb_put(cb);
	if (!valid_handler && valid_data == (void *) -1)
		nl80211_nlmsg_clear(msg);
	nlmsg_free(msg);
	return err;
}

static struct nl_sock * qtiwifi_create_nl_socket(struct nl_cb* cb)
{
	struct nl_sock *sock = nl_socket_alloc_cb(cb);

	if (sock == NULL) {
		ALOGE("Failed to create NL socket");
		return NULL;
	}

	if (nl_connect(sock, NETLINK_GENERIC)) {
		ALOGE("Could not connect handle");
		nl_socket_free(sock);
		return NULL;
	}

	return sock;
}


static int family_handler(struct nl_msg *msg, void *arg)
{
	struct family_data *res = arg;
	struct nlattr *tb[CTRL_ATTR_MAX + 1];
	struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
	struct nlattr *mcgrp;
	int i;

	nla_parse(tb, CTRL_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
		  genlmsg_attrlen(gnlh, 0), NULL);
	if (!tb[CTRL_ATTR_MCAST_GROUPS])
		return NL_SKIP;

	nla_for_each_nested(mcgrp, tb[CTRL_ATTR_MCAST_GROUPS], i) {
		struct nlattr *tb2[CTRL_ATTR_MCAST_GRP_MAX + 1];
		nla_parse(tb2, CTRL_ATTR_MCAST_GRP_MAX, nla_data(mcgrp),
			  nla_len(mcgrp), NULL);
		if (!tb2[CTRL_ATTR_MCAST_GRP_NAME] ||
		    !tb2[CTRL_ATTR_MCAST_GRP_ID] ||
		    strncmp(nla_data(tb2[CTRL_ATTR_MCAST_GRP_NAME]),
			       res->group,
			       nla_len(tb2[CTRL_ATTR_MCAST_GRP_NAME])) != 0)
			continue;
		res->id = nla_get_u32(tb2[CTRL_ATTR_MCAST_GRP_ID]);
		break;
	};

	return NL_SKIP;
}

static int nl_get_multicast_id(struct qtiwifi_global *ctx,
			       const char *family, const char *group)
{
	struct nl_msg *msg;
	int ret;
	struct family_data res = { group, -ENOENT };

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;
	if (!genlmsg_put(msg, 0, 0, genl_ctrl_resolve(ctx->cmd_sock, "nlctrl"),
			 0, 0, CTRL_CMD_GETFAMILY, 0) ||
	    nla_put_string(msg, CTRL_ATTR_FAMILY_NAME, family)) {
		nlmsg_free(msg);
		return -1;
	}

	ret = send_and_recv(ctx, msg, family_handler, &res);
	if (ret == 0)
		ret = res.id;
	return ret;
}

static void qtiwifi_register_eloop_read(struct nl_sock** handle,
			eloop_sock_handler handler, void* eloop_data) {
	int err;
	err = nl_socket_set_buffer_size(*handle, 262144, 0);
	if (err < 0) {
		/* continue anyway with the default (smaller) buffer */
	}
	nl_socket_set_nonblocking(*handle);
	eloop_register_read_sock(nl_socket_get_fd(*handle), handler,
		eloop_data, *handle);
}

static void qtiwifi_nl80211_init(void) {
	global = (struct qtiwifi_global*) malloc (sizeof(struct qtiwifi_global));

	if (!global) {
		ALOGE("global init failure");
		goto out;
	}

	global->ioctl_sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
	if (global->ioctl_sock_fd < 0 ) {
		ALOGE("ioctl sock created failure");
		goto out;
	}

	global->cb = nl_cb_alloc(NL_CB_DEFAULT);
	if (!global->cb) {
		ALOGE("global cb allocate failure");
		goto out;
	}

	global->cmd_sock = qtiwifi_create_nl_socket(global->cb);
	if (!global->cmd_sock) {
		ALOGE("global cmd_sock allocate failure");
		goto out;
	}

	global->event_sock = qtiwifi_create_nl_socket(global->cb);
	if (!global->event_sock) {
		ALOGE("global event_sock allocate failure");
		goto out;
	}

	global->nl80211_id = genl_ctrl_resolve(global->cmd_sock, "nl80211");
	if (global->nl80211_id < 0) {
		ALOGE("global cmd id obtain failure");
		goto out;
	}

	global->ctrl_event_qtiwifi_aidl_cb = NULL;

	int err = 1;
	nl_cb_set(global->cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, no_seq_check, NULL);
	nl_cb_err(global->cb, NL_CB_CUSTOM, error_handler, &err);
	nl_cb_set(global->cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &err);
	nl_cb_set(global->cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &err);

	nl_cb_set(global->cb, NL_CB_VALID, NL_CB_CUSTOM,
			qtiwifi_process_driver_events, NULL);

	int status = nl_get_multicast_id(global, "nl80211", "vendor");
	if (status >= 0) {
		status = nl_socket_add_membership(global->event_sock, status);
	}
	if (status < 0) {
		ALOGE("nl80211: Could not add multicast "
			   "membership for vendor events: %d (%s)",
			   status, nl_geterror(status));
		/* Continue without vendor events */
	}
	qtiwifi_register_eloop_read(&global->event_sock,
		qtiwifi_driver_nl80211_event_receive, global->cb);

	return;

out:
	ALOGE("qtiwifi nl80211 init failed");
}

static void qtiwifi_nl80211_deinit(void)
{
	if (global->event_sock) {
		nl_socket_free(global->event_sock);
	}
	if (global->cmd_sock) {
		nl_socket_free(global->cmd_sock);
	}
	if (global->cb) {
		nl_cb_put(global->cb);
	}
	free(global);
}

static void qtiwifi_terminate(int sig, void* ignore, void* nothing)
{
	ALOGI("qtiwifi ternimating...");
	eloop_terminate();
}

int qtiwifi_process_cmd(struct qtiwifi_global *global, const char* ifname,
		const char* cmd, char* reply, int reply_cap)
{
	int res;
	if (strncmp(cmd, "DRIVER ", 7) == 0) {
		res = qtiwifi_do_driver_cmd(global, ifname,
				cmd + 7, reply, reply_cap);
		ALOGE("%s: cmd=%s ", __func__, cmd);
	} else {
		res = -1;
	}

	if (res == 0) {
		memcpy(reply, "OK\n", 3);
		res = 3;
	}
	return res;
}

static int obtain_iface_handler(struct nl_msg *msg, void *arg)
{
	struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
	struct nlattr *tb_msg[NL80211_ATTR_MAX + 1];
	const char *indent = "";

	nla_parse(tb_msg, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
		  genlmsg_attrlen(gnlh, 0), NULL);

	if (tb_msg[NL80211_ATTR_IFNAME]) {
		char* name = nla_get_string(tb_msg[NL80211_ATTR_IFNAME]);
		if (strncmp(name, "wlan", 4) == 0 ||
				strncmp(name, "swlan", 5) == 0) {
			struct ifreq ifr;
			strlcpy(ifr.ifr_name, name, strlen(name)+1);

			//wlan* present but might not UP, check for its status
			if (strncmp(name, "wlan", 4) == 0) {
				if (ioctl(global->ioctl_sock_fd,
						SIOCGIFFLAGS, &ifr) != 0) {
					ALOGE("failed obtain %s status", name);
					return NL_SKIP;
				} else {
					if (!(ifr.ifr_flags & IFF_UP)) {
						return NL_SKIP;
					}
				}
			}

			strlcpy(qtiwifi_interfaces[qtiwifi_num_ifaces].name,
			  nla_get_string(tb_msg[NL80211_ATTR_IFNAME]),
			  strlen(nla_get_string(tb_msg[NL80211_ATTR_IFNAME]))+1);
			qtiwifi_interfaces[qtiwifi_num_ifaces].type = -1;
		} else
			return NL_SKIP;
	} else {
		ALOGE("%sUnnamed/non-netdev interface", indent);
		return NL_SKIP;
	}

	if (tb_msg[NL80211_ATTR_IFTYPE]) {
		qtiwifi_interfaces[qtiwifi_num_ifaces].type =
			nla_get_u32(tb_msg[NL80211_ATTR_IFTYPE]);

		ALOGI("%s: obtain ifname=%s, type=%d", __func__,
			qtiwifi_interfaces[qtiwifi_num_ifaces].name,
			qtiwifi_interfaces[qtiwifi_num_ifaces].type);
	}

	++qtiwifi_num_ifaces;
	return NL_SKIP;
}

int qtiwifi_get_interfaces(void) {
	qtiwifi_num_ifaces = 0;
	struct nl_msg *msg;

	msg = nlmsg_alloc();
	if (!msg) {
		ALOGE("Alloc msg buffer failed.");
		return -ENOMEM;
	}

	if (!genlmsg_put(msg, 0, 0, global->nl80211_id, 0,
			NLM_F_DUMP, NL80211_CMD_GET_INTERFACE, 0)){
		nlmsg_free(msg);
		return -1;
	}

	int ret = send_and_recv(global, msg, obtain_iface_handler, NULL);
	return ret;
}

int main(int argc, char *argv[])
{
	ALOGI("start qtiwifi");

	eloop_init(NULL);

	eloop_register_signal(SIGTERM, qtiwifi_terminate, NULL);
	eloop_register_signal(SIGINT, qtiwifi_terminate, NULL);

	qtiwifi_nl80211_init();

	qtiwifi_aidl_init(global);

	eloop_run();

	qtiwifi_aidl_deinit(global);

	qtiwifi_nl80211_deinit();

	return 0;
}
