/*
 * Copyright (c) 2020, 2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

/**
 * DOC: dump data for channel frequency response capture
 * This file provides tool to dump cfr capture data from driver
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdint.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <dirent.h>
#include "cfrtool.h"
#ifndef CONFIG_WIFICFR_USE_AIDL
#include "hidl/1.0/hidl.h"
#else
#include "aidl/aidl.h"
#endif
#include <pthread.h>
#include <log/log.h>

#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/rtnetlink.h>
#include <netpacket/packet.h>
#include <linux/filter.h>
#include <linux/errqueue.h>
#include <linux-private/linux/fib_rules.h>
#include <linux/pkt_sched.h>
#include <netlink/object-api.h>
#include <netlink/netlink.h>
#include <netlink/socket.h>
#include <netlink-private/object-api.h>
#include <netlink-private/types.h>

#include "../../../../../../external/wpa_supplicant_8/src/drivers/nl80211_copy.h"
#include "../../../../../../hardware/qcom/wlan/qcwcn/wpa_supplicant_8_lib/qca-vendor_copy.h"

#define CFR_DESTIN_FILE_DIR    "/data/vendor/wifi/cfr"
#define CFR_DESTIN_FILE        "/cfr_dump_%s.bin"
#define CFR_DESTIN_FILE_START  "cfr_dump_"
#define CFR_DESTIN_FILE_SUFFIX ".bin"
#define CFR_SOURCE_FILE0       "/sys/kernel/debug/wlan/cfrwlan/cfr_dump0"
#define CFR_SOURCE_FILE1       "/sys/kernel/debug/qdf/cfrwlan/cfr_dump0"
#define MAX_FILE_NAME_SIZE     128
#define MAX_FILE_SIZE          (8 * 1024 * 1024)
#define MAX_CAPTURE_SIZE       (4096)
#define INVAL_FILE_HANDLE      (-1)
#define INVAL_STATUS           (-1)
#define CFRLOG(params ...)     printf(params)
#define MAX_CFR_FILE_COUNT     4
#define CFR_STOP_STR           "CFR-CAPTURE-STOPPED"
#define CFR_STOP_STR_SIZE      (sizeof(CFR_STOP_STR))
#define CFR_DESTIN_FILE_START_SIZE    (sizeof(CFR_DESTIN_FILE_START) - 1)
#define WIFI_CFRTOOL_EVENT_SOCK_PORT     645
#define WIFI_HAL_CMD_SOCK_PORT     644
#define DEFAULT_EVENT_CB_SIZE   (64)

struct cfrtool_params {
	uint8_t stop_capture;
	uint8_t reset_write_file;
	int src_fd;
	int dst_fd;
	int dst_file_len;
	char cur_file_name[MAX_FILE_NAME_SIZE];
	uint8_t read_buf[MAX_CAPTURE_SIZE];
	uint8_t send_data_to_hidl;
	uint8_t hidl_initialized;
	int cfr_src_fd;
	int cfr_dst_fd;
};

struct family_data {
	const char *group;
	int id;
};

static struct cfrtool_params g_cfrtool_param = { 0 };

static int ack_handler(struct nl_msg *msg, void *arg)
{
	int *err = (int *)arg;

	*err = 0;
	return NL_STOP;
}

static int finish_handler(struct nl_msg *msg, void *arg)
{
	int *ret = (int *)arg;

	*ret = 0;
	return NL_SKIP;
}

static int no_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

static void exit_handler(int signum)
{
	g_cfrtool_param.stop_capture = 1;
}

static int error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err,
			 void *arg)
{
	int *ret = (int *)arg;

	*ret = err->error;
	ALOGI("%s received : %d - %s", __func__,
		   err->error, strerror(err->error));
	return NL_SKIP;
}

static int check_files_cfr_folder()
{
	DIR *cfr_dir;
	struct dirent *cfr_entry;
	char file_name[MAX_FILE_NAME_SIZE];
	char earliest_file_name[MAX_FILE_NAME_SIZE] = "";
	int cfr_file_count = 0;
	int remove_file_count;
	int index;
	int loop_index;
	struct stat cfr_file_stat;
	time_t earliest_time;

	cfr_dir = opendir(CFR_DESTIN_FILE_DIR);
	if (!cfr_dir)
	{
		CFRLOG("open cfr folder failed\n");
		ALOGE("open cfr folder failed\n");
		return INVAL_STATUS;
	}

	while ((cfr_entry = readdir(cfr_dir)) != NULL)
	{
		if (strncmp(cfr_entry->d_name, CFR_DESTIN_FILE_START,
			    CFR_DESTIN_FILE_START_SIZE) == 0)
			cfr_file_count++;
	}
	closedir(cfr_dir);

	if (cfr_file_count <= MAX_CFR_FILE_COUNT)
		return 0;

	remove_file_count = cfr_file_count - MAX_CFR_FILE_COUNT;
	time(&earliest_time);
	for (index = 0; index < remove_file_count; index++) {
		cfr_dir = opendir(CFR_DESTIN_FILE_DIR);
		if (!cfr_dir)
		{
			CFRLOG("open cfr folder failed");
			ALOGI("open cfr folder failed");
			continue;
		}
		loop_index = 0;
		while (((cfr_entry = readdir(cfr_dir)) != NULL))
		{
			if (strncmp(cfr_entry->d_name,
				    CFR_DESTIN_FILE_START,
				    CFR_DESTIN_FILE_START_SIZE) == 0)
			{
				snprintf(file_name, MAX_FILE_NAME_SIZE, "%s/%s",
					 CFR_DESTIN_FILE_DIR, cfr_entry->d_name);
				stat(file_name, &cfr_file_stat);
				if (earliest_time >= cfr_file_stat.st_ctime ||
				    !loop_index) {
					memcpy(earliest_file_name,
					       file_name,
					       MAX_FILE_NAME_SIZE);
					earliest_time = cfr_file_stat.st_ctime;
				}
				loop_index++;
			}
		}
		closedir(cfr_dir);

		/* Output file list if current file is the earliest file */
		if (strncmp(earliest_file_name,
			    g_cfrtool_param.cur_file_name,
			    MAX_FILE_NAME_SIZE) == 0) {
			CFRLOG("Err, current file is the earliest\n");
			ALOGI("Err, current file is the earliest\n");
			cfr_dir = opendir(CFR_DESTIN_FILE_DIR);
			if (!cfr_dir)
			{
				CFRLOG("open cfr folder failed\n");
				ALOGE("open cfr folder failed\n");
				return INVAL_STATUS;
			}

			while ((cfr_entry = readdir(cfr_dir)) != NULL)
			{
				if (strncmp(cfr_entry->d_name,
					    CFR_DESTIN_FILE_START,
					    CFR_DESTIN_FILE_START_SIZE) == 0)
					CFRLOG("%s\n", cfr_entry->d_name);
					ALOGI("%s\n", cfr_entry->d_name);
			}
			closedir(cfr_dir);
			return 0;
		}

		CFRLOG("Remove old file %s\n", earliest_file_name);
		ALOGI("Remove old file %s\n", earliest_file_name);
		remove(earliest_file_name);
	}

	return 0;
}

static int open_cfr_src_file()
{
	int hfile = INVAL_FILE_HANDLE;
	char str_file[MAX_FILE_NAME_SIZE];

	snprintf(str_file, sizeof(str_file), CFR_SOURCE_FILE0);
	hfile = open(str_file, O_RDONLY);
	if (hfile == INVAL_FILE_HANDLE) {
		snprintf(str_file, sizeof(str_file), CFR_SOURCE_FILE1);
		hfile = open(str_file, O_RDONLY);
	}
	g_cfrtool_param.src_fd = hfile;

	return hfile;
}

static int create_cfr_dst_file()
{
	static char str_file[MAX_FILE_NAME_SIZE];
	char str_time[MAX_FILE_NAME_SIZE] = {0};
	struct timeval tv = {0};
	time_t cur_time = 0;
	struct tm *tm_val = NULL;
	int hfile = INVAL_FILE_HANDLE;

	mkdir(CFR_DESTIN_FILE_DIR, 0777);
	gettimeofday(&tv, NULL);
	cur_time = tv.tv_sec;
	tm_val = localtime(&cur_time);
	if (tm_val) {
		strftime(str_time, MAX_FILE_NAME_SIZE, "%Y_%m_%d_%H_%M_%S",
			 tm_val);
		snprintf(str_file, sizeof(str_file), "%s"CFR_DESTIN_FILE,
			 CFR_DESTIN_FILE_DIR, str_time);
		CFRLOG("create new file: %s\n", str_file);
		ALOGI("create new file: %s\n", str_file);
		hfile = open(str_file, O_WRONLY | O_CREAT, 0600);
		g_cfrtool_param.dst_file_len = 0;
		g_cfrtool_param.dst_fd = hfile;
		memcpy(g_cfrtool_param.cur_file_name, str_file, MAX_FILE_NAME_SIZE);
	} else {
		CFRLOG("failed to get current time\n");
		ALOGE("failed to get current time\n");
	}

	return hfile;
}

static void cfr_data_read_handler(int src_handle, int dst_handle)
{
	int length = 0, retval = 0;
	struct cfr_header *head;
	uint32_t end_magic;
	uint8_t *ptr;

	length = read(src_handle, g_cfrtool_param.read_buf, MAX_CAPTURE_SIZE);
	if (length <= 0)
		return;

	if (length == sizeof(CFR_STOP_STR) &&
	    (!memcmp(g_cfrtool_param.read_buf, CFR_STOP_STR,
		    CFR_STOP_STR_SIZE))) {
		g_cfrtool_param.reset_write_file = 1;
			return;
	}

	if (length == MAX_CAPTURE_SIZE) {
		CFRLOG("please start this tool before starting CSI\n");
		while (length == MAX_CAPTURE_SIZE) {
			if(g_cfrtool_param.send_data_to_hidl) {
				wificfr_hidl_notify_cfrdata(g_cfrtool_param.read_buf, length);
			} else {
			write(dst_handle, g_cfrtool_param.read_buf, length);
			g_cfrtool_param.dst_file_len += length;
			}
			length = read(src_handle, g_cfrtool_param.read_buf,
				      MAX_CAPTURE_SIZE);
		}

		if (length >= CFR_STOP_STR_SIZE) {
			ptr = &g_cfrtool_param.read_buf[
				length - CFR_STOP_STR_SIZE];
			if (!memcmp(ptr, CFR_STOP_STR,
				    CFR_STOP_STR_SIZE)) {
				g_cfrtool_param.reset_write_file = 1;
				length -= CFR_STOP_STR_SIZE;
			}
		}

		if (length)
		{
			if(g_cfrtool_param.send_data_to_hidl) {
				wificfr_hidl_notify_cfrdata(g_cfrtool_param.read_buf, length);
		} else

			write(dst_handle, g_cfrtool_param.read_buf, length);
		}
		return;
	}

	if (lseek(src_handle, 0, SEEK_CUR) + length > MAX_FILE_SIZE) {
		retval = lseek(src_handle, 0, SEEK_SET);
		if (retval < 0) {
			CFRLOG("lseek fail");
			ALOGE("lseek fail");
			return;
		}
	}

	head = (struct cfr_header *) g_cfrtool_param.read_buf;
	if(g_cfrtool_param.send_data_to_hidl) {
		//Hidl callback
		ALOGI("cfrtool: Notify data available on HIDL callback");
		wificfr_hidl_notify_cfrdata(g_cfrtool_param.read_buf, length);
	} else {
		write(dst_handle, g_cfrtool_param.read_buf, length);
		g_cfrtool_param.dst_file_len += length;
	}
	end_magic = *((uint32_t *)(g_cfrtool_param.read_buf + sizeof(*head) +
		      head->meta.length));
	CFRLOG("total length %d, freq %d, phymode %d, payload length %d, offset %d, end magic:%x\n",
	       length, head->meta.prim20_chan, head->meta.phy_mode,
	       head->meta.length, (int)sizeof(*head), end_magic);
	ALOGI("cfrtool:total length %d, freq %d, phymode %d, payload length %d, offset %d, end magic:%x\n",
	       length, head->meta.prim20_chan, head->meta.phy_mode,
	       head->meta.length, (int)sizeof(*head), end_magic);
}

static struct nl_sock * wifi_create_nl_socket(int port, int protocol)
{
    struct nl_sock *sock = nl_socket_alloc();
    if (sock == NULL) {
        ALOGE("Failed to create NL socket");
        return NULL;
    }

    if (nl_connect(sock, protocol)) {
        ALOGE("Could not connect handle");
        nl_socket_free(sock);
        return NULL;
    }

    return sock;
}

static int internal_valid_message_handler(struct nl_msg *msg, void *arg)
{

	uint32_t vendor_id = 0;
	int subcmd = 0;

	struct genlmsghdr *genlh;
	struct  nlmsghdr *nlh = nlmsg_hdr(msg);
	genlh = (struct genlmsghdr *)nlmsg_data(nlh);
	struct nlattr *nlattrs[NL80211_ATTR_MAX + 1];
	struct nlattr *tb[QCA_WLAN_VENDOR_ATTR_PEER_CFR_MAX + 1];

	nla_parse(nlattrs, NL80211_ATTR_MAX, genlmsg_attrdata(genlh, 0),
		genlmsg_attrlen(genlh, 0), NULL);
	if (nlattrs[NL80211_ATTR_VENDOR_DATA]) {
		vendor_id = nla_get_u32(nlattrs[NL80211_ATTR_VENDOR_ID]);
		subcmd = nla_get_u32(nlattrs[NL80211_ATTR_VENDOR_SUBCMD]);
		if (subcmd != QCA_NL80211_VENDOR_SUBCMD_PEER_CFR_CAPTURE_CFG) {
			ALOGI("event received %s, vendor_id = 0x%0x, subcmd = 0x%0x",
			(char *)nla_data(nlattrs[NL80211_ATTR_VENDOR_DATA]), vendor_id, subcmd);
		} else {

			nla_parse(tb, QCA_WLAN_VENDOR_ATTR_PEER_CFR_MAX, nla_data(nlattrs[NL80211_ATTR_VENDOR_DATA]),
				  nla_len(nlattrs[NL80211_ATTR_VENDOR_DATA]), NULL);
			if (!tb[QCA_WLAN_VENDOR_ATTR_PEER_CFR_RESP_DATA])
			{
				ALOGD("No CSI data");
				//return WIFI_SUCCESS;
				return 0;
			}
			if(g_cfrtool_param.send_data_to_hidl) {
				wificfr_hidl_notify_cfrdata(nla_data(tb[QCA_WLAN_VENDOR_ATTR_PEER_CFR_RESP_DATA]),
					nla_len(tb[QCA_WLAN_VENDOR_ATTR_PEER_CFR_RESP_DATA]));
			} else if (g_cfrtool_param.cfr_dst_fd != INVAL_FILE_HANDLE) {
				write(g_cfrtool_param.cfr_dst_fd, nla_data(tb[QCA_WLAN_VENDOR_ATTR_PEER_CFR_RESP_DATA]),
					nla_len(tb[QCA_WLAN_VENDOR_ATTR_PEER_CFR_RESP_DATA]));
				g_cfrtool_param.dst_file_len += nla_len(tb[QCA_WLAN_VENDOR_ATTR_PEER_CFR_RESP_DATA]);
			}
		}
	}
	return 0;
}

static int internal_pollin_handler(struct nl_sock *sock)
{
	struct nl_cb *cb = nl_socket_get_cb(sock);

	int res = nl_recvmsgs(sock, cb);
	if(res)
		ALOGE("Error :%d while reading nl msg", res);
	nl_cb_put(cb);
	return res;
}

static void internal_event_handler(int events,
                                   struct nl_sock *sock)
{
	if (events & POLLERR) {
		ALOGE("Error reading from socket");
		internal_pollin_handler(sock);
	} else if (events & POLLHUP) {
		ALOGE("Remote side hung up");
	} else if (events & POLLIN) {
		//ALOGI("Found some events!!!");
		internal_pollin_handler(sock);
	} else {
		ALOGE("Unknown event - %0x", events);
	}
}

static void nl80211_nlmsg_clear(struct nl_msg *msg)
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

static int send_and_recv(struct nl80211_ctx *ctx,
			 struct nl_msg *msg,
			 int (*valid_handler)(struct nl_msg *, void *),
			 void *valid_data)
{
	struct nl_cb *cb;
	int err = -ENOMEM, opt;

	if (!msg)
		return -ENOMEM;

	cb = nl_cb_alloc(NL_CB_DEFAULT);
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
		ALOGE("nl80211: nl_send_auto_complete() failed: %s",
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

	if (valid_handler)
		nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM,
			  valid_handler, valid_data);

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
			ALOGE("nl80211: %s; convert to -EAGAIN",
				   nl_geterror(res));
			err = -EAGAIN;
		} else if (res < 0) {
			ALOGE("nl80211: %s->nl_recvmsgs failed: %d (%s)",
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

static int nl_get_multicast_id(struct nl80211_ctx *ctx,
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


static int cfr_data_rx_handler()
{
	int ret = 0;
	int status = 0;
	g_cfrtool_param.cfr_src_fd = INVAL_FILE_HANDLE;
	g_cfrtool_param.cfr_dst_fd = INVAL_FILE_HANDLE;

	g_cfrtool_param.cfr_src_fd = open_cfr_src_file();
	if (g_cfrtool_param.cfr_src_fd == INVAL_FILE_HANDLE) {
		CFRLOG("open CFR source file handle failed");
		ALOGI("cfrtool:open CFR source file handle failed, "
		"%s use transport mode as nl_event only", strerror(errno));
	}
	g_cfrtool_param.cfr_dst_fd = create_cfr_dst_file();
	if (g_cfrtool_param.cfr_dst_fd == INVAL_FILE_HANDLE) {
		CFRLOG("open CFR dst file handle failed");
		ALOGI("cfrtool:open CFR dst file handle failed");
	}

	struct nl80211_ctx *ctx;
	struct nl_cb *cb = NULL;
	ctx = calloc(1, sizeof(struct nl80211_ctx));
	if (!ctx) {
		ALOGE("Failed to alloc nl80211_ctx");
		return -1;
	}

	ctx->cmd_sock = wifi_create_nl_socket(WIFI_HAL_CMD_SOCK_PORT,
			NETLINK_GENERIC);
	if (ctx->cmd_sock == NULL) {
		ALOGE("Failed to create command socket port");
		//ret = WIFI_ERROR_UNKNOWN;
		ret = -1;
		goto unload;
	}

	/* Set the socket buffer size */
	if (nl_socket_set_buffer_size(ctx->cmd_sock, (256*1024), 0) < 0) {
		ALOGE("Could not set nl_socket RX buffer size for cmd_sock: %s",
		strerror(errno));
		/* continue anyway with the default (smaller) buffer */
	}

	ctx->event_sock =
		wifi_create_nl_socket(WIFI_CFRTOOL_EVENT_SOCK_PORT, NETLINK_GENERIC);
	if (ctx->event_sock == NULL) {
		ALOGE("Failed to create event socket port");
		//ret = WIFI_ERROR_UNKNOWN;
		ret = -1;
		goto unload;
	}

	/* Set the socket buffer size */
	if (nl_socket_set_buffer_size(ctx->event_sock, (256*1024), 0) < 0) {
		ALOGE("Could not set nl_socket RX buffer size for event_sock: %s",
			strerror(errno));
		/* continue anyway with the default (smaller) buffer */
	}

	cb = nl_socket_get_cb(ctx->event_sock);
	if (cb == NULL) {
		ALOGE("Failed to get NL control block for event socket port");
		//ret = WIFI_ERROR_UNKNOWN;
		ret = -1;
		goto unload;
	}

	int err = 1;
	nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, no_seq_check, NULL);
	nl_cb_err(cb, NL_CB_CUSTOM, error_handler, &err);
	nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &err);
	nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &err);

	nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, internal_valid_message_handler,
		NULL);
	nl_cb_put(cb);

	ctx->netlink_familyid = genl_ctrl_resolve(ctx->cmd_sock, "nl80211");
	if (ctx->netlink_familyid < 0) {
		ALOGE("nl80211: 'nl80211' generic netlink not found");
		ret = -1;
		goto unload;
	}
	status = nl_get_multicast_id(ctx, "nl80211", "vendor");
	if (status >= 0) {
		status = nl_socket_add_membership(ctx->event_sock, status);
	}
	if (status < 0) {
		ALOGE("nl80211: Could not add multicast "
			   "membership for vendor events: %d (%s)",
			   status, nl_geterror(status));
		/* Continue without vendor events */
	}

	struct pollfd pfd[2];
	memset(&pfd, 0, 2*sizeof(pfd[0]));

	pfd[0].fd = nl_socket_get_fd(ctx->event_sock);
	pfd[0].events = POLLIN | POLLHUP | POLLERR;
	if (g_cfrtool_param.cfr_src_fd != INVAL_FILE_HANDLE) {
		pfd[1].fd = g_cfrtool_param.cfr_src_fd;
		pfd[1].events = POLLIN | POLLHUP | POLLERR;
	}
	int result;

	do {
		pfd[0].revents = 0;
		pfd[1].revents = 0;
		if (g_cfrtool_param.cfr_src_fd != INVAL_FILE_HANDLE)
			result = poll(pfd, 2, -1);
		else
			result = poll(pfd, 1, -1);
		if (result < 0) {
			ALOGE("Error polling socket, result %d", result);
		} else {
			if ((!g_cfrtool_param.send_data_to_hidl) &&
				(g_cfrtool_param.dst_file_len > MAX_FILE_SIZE))
				g_cfrtool_param.reset_write_file = 1;
			if (!g_cfrtool_param.send_data_to_hidl &&
				g_cfrtool_param.reset_write_file) {
				g_cfrtool_param.reset_write_file = 0;
				close(g_cfrtool_param.cfr_dst_fd);
				CFRLOG("Capture File Ready: %s\n",
					g_cfrtool_param.cur_file_name);
				ALOGI("Capture File Ready: %s\n",
					g_cfrtool_param.cur_file_name);
				check_files_cfr_folder();
				g_cfrtool_param.cfr_dst_fd = create_cfr_dst_file();
				if (g_cfrtool_param.cfr_dst_fd == INVAL_FILE_HANDLE &&
					g_cfrtool_param.cfr_src_fd != INVAL_FILE_HANDLE) {
					close(g_cfrtool_param.cfr_src_fd);
					CFRLOG("open CFR dst file handle failed");
					ALOGE("open CFR dst file handle failed");
					return INVAL_STATUS;
				}
			}
			if (pfd[0].revents & (POLLIN | POLLHUP | POLLERR)) {
				if (g_cfrtool_param.cfr_dst_fd == INVAL_FILE_HANDLE &&
					!g_cfrtool_param.send_data_to_hidl) {
					ALOGE("open CFR dst file handle failed, also don't send data to hidl");
				}
				internal_event_handler(pfd[0].revents, ctx->event_sock);
			}
			if (pfd[1].revents & (POLLIN | POLLHUP | POLLERR)) {
				ALOGI("polling for realyfs succeed");
				if (pfd[1].revents & POLLERR) {
					close(g_cfrtool_param.cfr_src_fd);
					close(g_cfrtool_param.cfr_dst_fd);
					if (!g_cfrtool_param.send_data_to_hidl &&
						!g_cfrtool_param.dst_file_len) {
						CFRLOG("Remove NULL file: %s\n",
							g_cfrtool_param.cur_file_name);
						ALOGE("Remove NULL file: %s\n",
							g_cfrtool_param.cur_file_name);
						remove(g_cfrtool_param.cur_file_name);
					}
					return INVAL_STATUS;
				}

				if (g_cfrtool_param.send_data_to_hidl ||
					(pfd[1].revents & POLLIN)) {
					if (g_cfrtool_param.cfr_dst_fd == INVAL_FILE_HANDLE &&
						!g_cfrtool_param.send_data_to_hidl) {
						ALOGE("open CFR dst file handle failed, also don't send data to hidl");
						return INVAL_STATUS;
					}
					cfr_data_read_handler(g_cfrtool_param.cfr_src_fd, g_cfrtool_param.cfr_dst_fd);
				}
			}
		}
	} while (!g_cfrtool_param.stop_capture);

	if (g_cfrtool_param.cfr_src_fd != INVAL_FILE_HANDLE)
		close(g_cfrtool_param.cfr_src_fd);
	if (g_cfrtool_param.cfr_dst_fd != INVAL_FILE_HANDLE)
		close(g_cfrtool_param.cfr_dst_fd);
	if (!g_cfrtool_param.dst_file_len || g_cfrtool_param.dst_file_len <= CFR_STOP_STR_SIZE) {
		CFRLOG("Remove NULL file: %s\n",
			g_cfrtool_param.cur_file_name);
		ALOGI("Remove NULL file: %s\n",
			g_cfrtool_param.cur_file_name);
		remove(g_cfrtool_param.cur_file_name);
	}

unload:
	if (ctx->event_sock) {
		nl_socket_free(ctx->event_sock);
	}
	if (ctx->cmd_sock) {
		nl_socket_free(ctx->cmd_sock);
	}
	free(ctx);
	return -1;
}

int startCapture() {
	if (g_cfrtool_param.hidl_initialized) {
		g_cfrtool_param.send_data_to_hidl = 1;
		return 1;
	}
	else
		return 0;
}


int stopCapture() {
	if (g_cfrtool_param.hidl_initialized) {
		g_cfrtool_param.send_data_to_hidl = 0;
		return 1;
	}
	else
		return 0;
}

void* initialize_hidl (void *arg)
{
	ALOGI("cfrtool:HIDL initialization");
	if (wificfr_hidl_process()) {
		ALOGE("cfrtool:HIDL initialization failed\n");
		g_cfrtool_param.hidl_initialized = 0;
	}
	return NULL;
}

int main(int argc, char *argv[])
{
	g_cfrtool_param.hidl_initialized = 1;
	//Create a thread for initializing the hidl for cfrtool
	pthread_t cfr_hidl_thread;
	pthread_create(&cfr_hidl_thread, NULL, initialize_hidl, NULL);

	if (argc > 1)
		CFRLOG("Needn't additional params\n");
	CFRLOG("Press Ctrl+C to Exit\n");
	signal(SIGINT, exit_handler);
	cfr_data_rx_handler();
	return 0;
}
