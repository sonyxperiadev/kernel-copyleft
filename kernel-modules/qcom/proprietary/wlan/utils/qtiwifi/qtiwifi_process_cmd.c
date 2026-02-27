/**
  * Copyright (c) 2023 Qualcomm Technologies, Inc.
  * All Rights Reserved.
  * Confidential and Proprietary - Qualcomm Technologies, Inc.
  */
/* Qualcomm Technologies, Inc. has chosen to take qtiwifi_process_cmd.c
 * subject to the BSD license and terms.
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
#include <log/log.h>
#include <net/if.h>
#include <stdint.h>
#include <errno.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/rtnetlink.h>
#include <netlink/object-api.h>
#include <netlink/netlink.h>
#include <netlink/socket.h>

#include "qca-vendor_copy.h"
#include "nl80211_copy.h"
#include "eloop.h"
#include "wpa_driver_common_lib.h"
#include "qtiwifitool.h"

static struct nl_msg* qtiwifi_nlmsg(struct qtiwifi_global* global,
	const char* ifname, int cmd, int subcmd)
{
	int res;
	struct nl_msg *nlmsg = nlmsg_alloc();
	int ifindex;

	if (nlmsg == NULL) {
		ALOGE("Out of memory");
		return NULL;
	}

	genlmsg_put(nlmsg, /* pid = */ 0, /* seq = */ 0,
		global->nl80211_id, 0, 0, cmd, 0);

	if (cmd == NL80211_CMD_VENDOR) {
		res = nla_put_u32(nlmsg, NL80211_ATTR_VENDOR_ID, OUI_QCA);
		if (res < 0) {
			ALOGE("Failed to put vendor id");
			goto cleanup;
		}
		res = nla_put_u32(nlmsg, NL80211_ATTR_VENDOR_SUBCMD, subcmd);
		if (res < 0) {
			ALOGE("Failed to put vendor sub command");
			goto cleanup;
		}
	}

	if (ifname && (strlen(ifname) > 0))
		ifindex = if_nametoindex(ifname);
	else
		ifindex = if_nametoindex("wlan0");

	if (nla_put_u32(nlmsg, NL80211_ATTR_IFINDEX, ifindex) != 0) {
		ALOGE("Failed to get iface index for iface: %s", ifname);
		goto cleanup;
	}
	return nlmsg;

cleanup:
	if (nlmsg)
		nlmsg_free(nlmsg);
	return NULL;
}

static uint32_t qtiwifi_get_iface_type(const char* ifname)
{
	int i = 0;
	int ret = qtiwifi_get_interfaces();

	if (ret)
		return -1;

	for (; i < qtiwifi_num_ifaces; ++i) {
		if (strncmp(ifname, qtiwifi_interfaces[i].name, 6) == 0) {
			return qtiwifi_interfaces[i].type;
		}
	}
	return -1;
}

static int thermal_info_handler(struct nl_msg *msg, void *arg)
{
	struct nlattr *tb[NL80211_ATTR_MAX + 1];
	struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
	int *param = arg;
	struct nlattr *nl_vendor;
	struct nlattr *tb_vendor[QCA_WLAN_VENDOR_ATTR_MAX + 1];

	nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
	    genlmsg_attrlen(gnlh, 0), NULL);

	nl_vendor = tb[NL80211_ATTR_VENDOR_DATA];
	if (!nl_vendor || nla_parse(tb_vendor, QCA_WLAN_VENDOR_ATTR_MAX,
	    nla_data(nl_vendor), nla_len(nl_vendor), NULL)) {
		return NL_SKIP;
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_THERMAL_GET_TEMPERATURE_DATA])
		*param = (int) nla_get_u32(tb_vendor[
		    QCA_WLAN_VENDOR_ATTR_THERMAL_GET_TEMPERATURE_DATA]);
	else if (tb_vendor[QCA_WLAN_VENDOR_ATTR_THERMAL_LEVEL])
		*param = (int) nla_get_u32(
		    tb_vendor[QCA_WLAN_VENDOR_ATTR_THERMAL_LEVEL]);
	else ;

	ALOGD("%s, receive thermal info %d", __func__, *param);
	return NL_SKIP;
}

void qtiwifi_driver_nl80211_event_receive(int sock, void *eloop_ctx,
	void *handle)
{
	struct nl_cb *cb = eloop_ctx;
	int res;

	res = nl_recvmsgs(handle, cb);
	if (res < 0) {
		ALOGE("nl80211: %s->nl_recvmsgs failed: %d",
		__func__, res);
	}
}



static void qtiwifi_nl80211_medium_access_event(int ifidx,
	uint8_t* data, size_t len)
{
	char name[LEN_INTERFACE_NAME_MAX] = {0};
	uint8_t type;
	uint32_t percent;
	char event_msg[1024] = {0};

	ALOGI("%s: received congestion report event", __func__);
	if (global->ctrl_event_qtiwifi_aidl_cb) {
		if_indextoname(ifidx, name);
	} else {
		return;
	}

	struct nlattr *tb[QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_MAX + 1];
	if (nla_parse(tb, QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_MAX,
		(struct nlattr *) data, len, NULL) ||
		!tb[QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_TYPE])
		return;

	type = nla_get_u8(tb[QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_TYPE]);
	if (type == QCA_WLAN_MEDIUM_ASSESS_CONGESTION_REPORT) {
		if (!tb[QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_CONGESTION_PERCENTAGE])
			return;
		percent = nla_get_u8(
		    tb[QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_CONGESTION_PERCENTAGE]);

		snprintf(event_msg, sizeof(event_msg),
		    QTIWIFI_EVENT_CONGESTION_REPORT "percentage=%d", percent);

		global->ctrl_event_qtiwifi_aidl_cb(name, event_msg);
	}
}

static void qtiwifi_nl80211_thermal_event(int ifidx, uint8_t* data, size_t len)
{
	char name[LEN_INTERFACE_NAME_MAX] = {0};
	char event_msg[1024] = {0};
	uint32_t level;

	ALOGI("%s: received thermal event", __func__);

	if (global->ctrl_event_qtiwifi_aidl_cb) {
		if_indextoname(ifidx, name);
	} else {
		return;
	}

	struct nlattr *tb[QCA_WLAN_VENDOR_ATTR_THERMAL_EVENT_MAX + 1];
	if (nla_parse(tb, QCA_WLAN_VENDOR_ATTR_THERMAL_EVENT_MAX,
		(struct nlattr *) data, len, NULL) ||
		!tb[QCA_WLAN_VENDOR_ATTR_THERMAL_EVENT_LEVEL])
		return;

	level = nla_get_u32(tb[QCA_WLAN_VENDOR_ATTR_THERMAL_EVENT_LEVEL]);

	snprintf(event_msg, sizeof(event_msg), QTIWIFI_EVENT_THERMAL_CHANGE
			"level=%d", level);
	global->ctrl_event_qtiwifi_aidl_cb(name, event_msg);
}

int qtiwifi_process_driver_events(struct nl_msg *msg, void *arg)
{
	uint32_t vendor_id = 0;
	int subcmd = 0;
	int ifidx = -1;

	struct genlmsghdr *genlh;
	struct nlmsghdr *nlh = nlmsg_hdr(msg);
	genlh = (struct genlmsghdr *)nlmsg_data(nlh);

	struct nlattr *nlattrs[NL80211_ATTR_MAX + 1];
	uint8_t* data = NULL;
	size_t len = 0;

	nla_parse(nlattrs, NL80211_ATTR_MAX, genlmsg_attrdata(genlh, 0),
		genlmsg_attrlen(genlh, 0), NULL);

	if (nlattrs[NL80211_ATTR_IFINDEX]) {
		ifidx = nla_get_u32(nlattrs[NL80211_ATTR_IFINDEX]);
	}

	if (nlattrs[NL80211_ATTR_VENDOR_DATA]) {
		vendor_id = nla_get_u32(nlattrs[NL80211_ATTR_VENDOR_ID]);
		subcmd = nla_get_u32(nlattrs[NL80211_ATTR_VENDOR_SUBCMD]);

		if (vendor_id != OUI_QCA) {
			ALOGI("ignore unsupported vendor: vendor_id=%u,"
				"subcmd=%d", vendor_id, subcmd);
			return 0;
		}

		data = nla_data(nlattrs[NL80211_ATTR_VENDOR_DATA]);
		len = nla_len(nlattrs[NL80211_ATTR_VENDOR_DATA]);
        } else {
		ALOGI("ignore event from ifidx=%d", ifidx);
		return 0;
	}
	switch (subcmd) {

	case QCA_NL80211_VENDOR_SUBCMD_THERMAL_EVENT:
		qtiwifi_nl80211_thermal_event(ifidx, data, len);
		break;

	case QCA_NL80211_VENDOR_SUBCMD_MEDIUM_ASSESS:
		qtiwifi_nl80211_medium_access_event(ifidx, data, len);
		break;

	default:
		ALOGI("ignore unsupported vendor: vendor_id=%u,"
			"subcmd=%d", vendor_id, subcmd);
		break;
	}

	return 0;
}

static int qtiwifi_driver_cmd_get_thermal_info(struct qtiwifi_global* global,
	const char* ifname, int *result, int attr)
{
	struct nl_msg *msg = NULL;
	struct nlattr *params = NULL;
	int ret = 0;

	if (!(msg = qtiwifi_nlmsg(global, ifname,
		NL80211_CMD_VENDOR, QCA_NL80211_VENDOR_SUBCMD_THERMAL_CMD)) ||
		!(params = nla_nest_start(msg, NL80211_ATTR_VENDOR_DATA)) ||
		nla_put_u32(msg, QCA_WLAN_VENDOR_ATTR_THERMAL_CMD_VALUE, attr)) {
		nlmsg_free(msg);
		return -1;
	}

	nla_nest_end(msg, params);
	ret = send_and_recv(global, msg, thermal_info_handler, result);
	if (!ret)
		return 0;
	ALOGE("%s: failed to get thermal info, ifname=%s", __func__, ifname);
	return ret;
}

static int qtiwifi_driver_cmd_set_ani_level(struct qtiwifi_global *global,
	const char* ifname, int mode, int ofdmlvl)
{
	struct nl_msg *msg = NULL;
	struct nlattr *params = NULL;
	int ret = 0;

	if (!(msg = qtiwifi_nlmsg(global, ifname, NL80211_CMD_VENDOR,
		QCA_NL80211_VENDOR_SUBCMD_SET_WIFI_CONFIGURATION)) ||
		!(params = nla_nest_start(msg, NL80211_ATTR_VENDOR_DATA)) ||
		nla_put_u8(msg, QCA_WLAN_VENDOR_ATTR_CONFIG_ANI_SETTING, mode)) {
		nlmsg_free(msg);
		return -1;
	}

	if(mode == QCA_WLAN_ANI_SETTING_FIXED) {
		if(nla_put(msg, QCA_WLAN_VENDOR_ATTR_CONFIG_ANI_LEVEL,
			sizeof(int32_t), &ofdmlvl)) {
			nlmsg_free(msg);
			return -1;
		}
	}

	nla_nest_end(msg, params);
	ret = send_and_recv(global, msg, NULL, NULL);
	if (!ret)
		return 0;
	ALOGE("%s: failed to set ani level, ifname=%s, mode=%d, ofdmlvl=%d,\
		ret=%d", __func__, ifname, mode, ofdmlvl, ret);

	return ret;
}

static int qtiwifi_driver_cmd_set_congestion_report(
	struct qtiwifi_global *global, const char* ifname,  const char *cmd)
{
	struct nl_msg *msg;
	struct nlattr *params = NULL;
	char *endptr = NULL;
	int ret;
	int enable = -1, threshold = -1, interval = -1;

	ALOGI("%s enter: ifname=%s, cmd=%s", __func__, ifname, cmd);

	enable = strtol(cmd, &endptr, 10);
	if (enable != 0 && enable != 1) {
		ALOGE("%s: invalid enable arg %d", __func__, enable);
		return -EINVAL;
	}

	if (!(msg = qtiwifi_nlmsg(global, ifname, NL80211_CMD_VENDOR,
		QCA_NL80211_VENDOR_SUBCMD_MEDIUM_ASSESS)) ||
		!(params = nla_nest_start(msg, NL80211_ATTR_VENDOR_DATA)) ||
		nla_put_u8(msg, QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_TYPE,
		  QCA_WLAN_MEDIUM_ASSESS_CONGESTION_REPORT) ||
		nla_put_u8(msg,
		  QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_CONGESTION_REPORT_ENABLE,
		  enable)) {
			nlmsg_free(msg);
			return -1;
	}

	if (enable == 1) {
		if (!(*endptr) ||
		  ((threshold = strtol(endptr, &endptr, 10)) < 0 ||
		  threshold > 100) ||
		  !(*endptr) || ((interval = strtol(endptr, &endptr, 10)) < 1 ||
		  interval > 30)) {
			ALOGE("%s: args less or invalid", __func__);
			nlmsg_free(msg);
			return -EINVAL;
		}
		if (nla_put_u8(msg,
		  QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_CONGESTION_REPORT_THRESHOLD,
		  threshold) || nla_put_u8(msg,
		  QCA_WLAN_VENDOR_ATTR_MEDIUM_ASSESS_CONGESTION_REPORT_INTERVAL,
		  interval)) {
			nlmsg_free(msg);
			return -1;
		}
	}

	nla_nest_end(msg, params);
	ret = send_and_recv(global, msg, NULL, NULL);

	if (!ret)
		return 0;
	ALOGE("%s: Failed set congestion report, ret=%d", __func__, ret);
	return ret;
}

static int qtiwifi_driver_cmd_set_tx_power(struct qtiwifi_global *global,
	const char* ifname, const char *cmd)
{
	struct nl_msg *msg;
	char *endptr = NULL;
	int ret;
	int dbm, mbm;

	ALOGI("%s enter: ifname=%s, cmd=%s", __func__, ifname, cmd);

	dbm = strtol(cmd, &endptr, 10);
	if (*endptr || dbm < 0) {
		ALOGE("%s: invalid dbm %d", __func__, dbm);
		return -1;
	}
	mbm = dbm * 100;
	if (mbm < 0) { // integer overflow
		ALOGE("%s: invalid mbm %d", __func__, mbm);
		return -1;
	}

	if (!(msg = qtiwifi_nlmsg(global, ifname, NL80211_CMD_SET_WIPHY, 0)) ||
	    nla_put_u32(msg, NL80211_ATTR_WIPHY_TX_POWER_SETTING,
		NL80211_TX_POWER_LIMITED) ||
	    nla_put_u32(msg, NL80211_ATTR_WIPHY_TX_POWER_LEVEL, mbm)) {
		nlmsg_free(msg);
		return -1;
	}

	ret = send_and_recv(global, msg, NULL, NULL);
	if (!ret)
		return 0;

	ALOGE("%s: Failed set_tx_power dbm=%d, ret=%d",
		   __func__, dbm, ret);
	return ret;
}

static int qtiwifi_sta_info_handler(struct nl_msg *msg, void *arg) {
	struct genlmsghdr *msg_hdr;
	struct nlattr *tb[NL80211_ATTR_MAX + 1];
	struct nlattr *tb_vendor[QCA_WLAN_VENDOR_ATTR_MAX + 1];
	struct nlattr *vendor_data, *attr_link_info;
	int vendor_len;
	struct sta_info  *info = (struct sta_info *)arg;

	if (!info) {
		ALOGE("%s: Invalid arg, it's null!", __func__);
		return -1;
	}

	msg_hdr = (struct genlmsghdr *)nlmsg_data(nlmsg_hdr(msg));
	nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(msg_hdr, 0),
		  genlmsg_attrlen(msg_hdr, 0), NULL);

	if (!tb[NL80211_ATTR_VENDOR_DATA]) {
		ALOGE("%s,NL80211_ATTR_VENDOR_DATA parse error", __func__);
		return -1;
	}

	vendor_data = nla_data(tb[NL80211_ATTR_VENDOR_DATA]);
	vendor_len = nla_len(tb[NL80211_ATTR_VENDOR_DATA]);

	if (nla_parse(tb_vendor, QCA_WLAN_VENDOR_ATTR_MAX,
		      vendor_data, vendor_len, NULL)) {
		ALOGE("%s: NL80211_ATTR_VENDOR_DATA not found", __func__);
		return -1;
	}

	attr_link_info = tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_LINK_INFO_ATTR];
	if (attr_link_info) {
		struct nlattr *tb_link_info[NL80211_ATTR_MAX + 1];

		nla_parse(tb_link_info, NL80211_ATTR_MAX, nla_data(attr_link_info),
			  nla_len(attr_link_info), NULL);

		if (tb_link_info[NL80211_ATTR_REASON_CODE]) {
			info->reason =
				nla_get_u32(tb_link_info[NL80211_ATTR_REASON_CODE]);
			ALOGI("%s: successfully obtained reason code: reason=%d",
	      			__func__, info->reason);
			return 0;
		}
	}

	ALOGE("%s: failed to obtain reason code", __func__);
	return -1;
}

static int chartohex(char c)
{
	int val = -1;

	if (c >= '0' && c <= '9')
		val = c - '0';
	else if (c >= 'a' && c <= 'f')
		val = c - 'a' + 10;
	else if (c >= 'A' && c <= 'F')
		val = c - 'A' + 10;

	return val;
}

static int convert_string_to_bytes(uint8_t *addr, const char *text, uint16_t max_bytes)
{
	uint16_t i = 0;
	int nibble;
	const char *temp = text;

	while (temp && *temp != '\0' && i < max_bytes) {
		nibble = chartohex(*temp++);
		if (nibble == -1)
			return -1;
		addr[i] = nibble << 4;
		nibble = chartohex(*temp++);
		if (nibble == -1)
			return -1;
		addr[i++] += nibble;
		if (*temp == ':')
			temp++;
	}

	return i;
}

static int qtiwifi_driver_cmd_get_sta_info(struct qtiwifi_global *global,
	const char* ifname, const char* cmd, struct sta_info *sinfo)
{
	uint8_t mac[ETH_ALEN];
	int ret;
	struct nl_msg *msg;
	struct nlattr *params;

	while (*cmd == ' ') ++cmd; //skip white space
	if (!(strlen(cmd) >= ETH_ALEN * 2 + ETH_ALEN - 1
			&& convert_string_to_bytes(mac, cmd, ETH_ALEN) > 0)) {
		ALOGE("%s: no mac provided.", __func__);
		return -1;
	}
	if (!(msg = qtiwifi_nlmsg(global, ifname, NL80211_CMD_VENDOR,
		QCA_NL80211_VENDOR_SUBCMD_GET_STATION)) ||
		!(params = nla_nest_start(msg, NL80211_ATTR_VENDOR_DATA)) ||
		nla_put(msg, QCA_WLAN_VENDOR_ATTR_GET_STATION_REMOTE,
		  ETH_ALEN, mac)) {
		nlmsg_free(msg);
		return -1;
	}
	nla_nest_end(msg, params);
	ret = send_and_recv(global, msg, qtiwifi_sta_info_handler, sinfo);

	if (!ret)
		return 0;

	ALOGE("%s: Failed to get sta info", __func__);
	return ret;
}

int qtiwifi_do_driver_cmd(struct qtiwifi_global *global, const char* ifname,
	const char* cmd, char* reply, int reply_cap)
{
	int res;

	if (global == NULL || ifname == NULL || cmd == NULL || reply == NULL) {
		ALOGE("%s: invalid args!", __func__);
		return -1;
	}

	if (strncmp(cmd, "GET_THERMAL_INFO", 16) == 0) {
		int temperature = -1;
		int thermal = -1;
		int ret, ret2;

		ret = qtiwifi_driver_cmd_get_thermal_info(global, ifname, &temperature,
		        QCA_WLAN_VENDOR_ATTR_THERMAL_CMD_TYPE_GET_TEMPERATURE);
		if (ret) {
			ALOGE(" %s, failed to obtain thermal temperature ", __func__);
			return -1;
		}

		ret2 = qtiwifi_driver_cmd_get_thermal_info(global, ifname, &thermal,
		        QCA_WLAN_VENDOR_ATTR_THERMAL_CMD_TYPE_GET_LEVEL);
		if (ret2) {
			ALOGE(" %s, failed to obtain thermal level ", __func__);
			return -1;
		}

		ALOGI("%s thermal cmd result: %d %d", __func__, temperature, thermal);
		snprintf(reply, reply_cap, "%d %d", temperature, thermal);
		res = strlen(reply);
	} else if (strncmp(cmd, "SET_ANI_LEVEL ", 14) == 0) {
		char* endptr = NULL;
		int mode = 0;
		int ofdmlvl = 0;

		mode = strtol(cmd + 14, &endptr, 10);
		if (mode == 1) {
			if(!(*endptr)) {
				ALOGE("%s: failed to set ani setting,\
					invalid cmd: %s\n", __func__, cmd);
				return -1;
			}
			ofdmlvl = strtol(endptr, NULL, 10);
		}

		res = qtiwifi_driver_cmd_set_ani_level(global, ifname, mode, ofdmlvl);
	} else if (strncmp(cmd, "SET_CONGESTION_REPORT ", 22) == 0) {
		uint32_t type = qtiwifi_get_iface_type(ifname);

		if (type != NL80211_IFTYPE_AP) {
			ALOGE("%s: this cmd only applies to AP iface: %s\
				type of %s is %d", __func__, cmd, ifname, type);
			return -1;
		}
		return qtiwifi_driver_cmd_set_congestion_report(global, ifname, cmd + 22);
	} else if (strncmp(cmd, "SET_TXPOWER ", 12) == 0) {
		return qtiwifi_driver_cmd_set_tx_power(global, ifname, cmd + 12);
	} else if (strncmp(cmd, "MACADDR", 7) == 0) {
		uint8_t mac[ETH_ALEN] = {};
		struct ifreq ifr;

		memset(&ifr, 0, sizeof(ifr));
		strlcpy(ifr.ifr_name, ifname, LEN_INTERFACE_NAME_MAX);

		if (ioctl(global->ioctl_sock_fd, SIOCGIFHWADDR, &ifr)) {
			ALOGE("%s: failed to obtain %s mac", __func__, ifname);
			return -1;
		}

		memcpy(mac, ifr.ifr_hwaddr.sa_data, ETH_ALEN);
		snprintf(reply, reply_cap, "Macaddr = " MACSTR "\n", MAC2STR(mac));
		res = strlen(reply);
	} else if (strncmp(cmd, "GETSTATSSTAINFO", 15) == 0) {
		struct sta_info sinfo;
		int ret;

		ret = qtiwifi_driver_cmd_get_sta_info(global, ifname, cmd + 15, &sinfo);
		if (ret) {
			ALOGE(" %s, failed to obtain sta info ", __func__);
			return -1;
		}
		snprintf(reply, reply_cap, "reason = %d", sinfo.reason);
		res = strlen(reply);
	} else {
		ALOGE("%s: unsupported driver cmd %s", __func__, cmd);
		res = -1;
	}
	return res;
}
