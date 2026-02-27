/*
 * Driver interaction with Linux nl80211/cfg80211
 * Copyright (c) 2002-2015, Jouni Malinen <j@w1.fi>
 * Copyright (c) 2003-2004, Instant802 Networks, Inc.
 * Copyright (c) 2005-2006, Devicescape Software, Inc.
 * Copyright (c) 2007, Johannes Berg <johannes@sipsolutions.net>
 * Copyright (c) 2009-2010, Atheros Communications
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <net/if.h>
#include "spectral_scan_fmt.h"
#include "spectraltool.h"
#include <inttypes.h>
#include "qca-vendor_copy.h"

struct nlIfaceInfo *info;
int config_params[MAX_NUM_OF_PARAMS];
char *sample_out_file = "/data/outFile";

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


int error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err, void *arg)
{
	int *ret = (int *)arg;
	*ret = err->error;

	printf("error_handler received : %d\n", err->error);
	return NL_SKIP;
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

static int send_and_recv(struct nlIfaceInfo *info,
			 struct nl_handle *nl_handle, struct nl_msg *msg,
			 int (*valid_handler)(struct nl_msg *, void *),
			 void *valid_data)
{
	struct nl_cb *cb;
	int err = -ENOMEM;

	if (!msg)
		return -ENOMEM;

	cb = nl_cb_clone(info->nl_cb);
	if (!cb)
		goto out;

	err = nl_send_auto_complete(nl_handle, msg);
	if (err < 0)
		goto out;

	err = 1;

	nl_cb_err(cb, NL_CB_CUSTOM, error_handler, &err);
	nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &err);
	nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &err);

	if (valid_handler)
		nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM,
			  valid_handler, valid_data);

	while (err > 0) {
		int res = nl_recvmsgs(nl_handle, cb);

		if (res < 0) {
			printf("nl80211: %s->nl_recvmsgs failed: %d",
				   __func__, res);
		}
	}
 out:
	nl_cb_put(cb);
	if (!valid_handler && valid_data == (void *) -1)
		nl80211_nlmsg_clear(msg);
	nlmsg_free(msg);
	return err;
}

void reset_spectral_configs(void)
{
	int i;

	for (i = 0; i < MAX_NUM_OF_PARAMS; i++)
		config_params[i] = -1;
}

void dump_samp_msg(void *data, uint32_t len)
{
	struct spectral_samp_msg_info *msg_legacy;
	struct spectral_samp_msg_info_umac_v0 *msg_umac_v0;
	struct spectral_samp_msg_info_umac_v1 *msg_umac_v1;
	uint32_t i;

	if (!data || !len) {
		printf("Invalid msg\n");
		return;
	}

	if (IS_UMAC_SPECTRAL_SAMPLE_MSG(info->version,
					info->sub_version,
					len)) {
		printf("***********dump converged samp msg**********\n");
		if (info->sub_version == SPECTRAL_SUB_VERSION_1) {
			msg_umac_v1 = (struct spectral_samp_msg_info_umac_v1 *)
					data;
			output_sample_msg(msg_umac_v1);
		} else {
			msg_umac_v0 = (struct spectral_samp_msg_info_umac_v0 *)
					data;
			output_sample_msg(msg_umac_v0);
		}
	} else {
		printf("***********dump samp msg**********\n");
		msg_legacy = (struct spectral_samp_msg_info *)data;
		output_sample_msg(msg_legacy);
	}
}

/*
 * Function    : get_free_mem_kB
 * Description : Get amount of free physical memory, in kB. We do not consider
 *		 cached memory since caching behaviour cannot be modelled by the
 *		 application and besides, we would like to avoid any actions
 *		 that result in cache flushes.
 * Input   : None
 * Output   : On error: -1, on success: amount of free physical memory in kB
 */
static int get_free_mem_kB(void)
{
	FILE *fp = NULL;
	char line[256];
	int free_mem = -1;

	fp = fopen("/proc/meminfo", "r");

	if (fp == NULL) {
		perror("fopen");
		return -EINVAL;
	}

	while (fgets(line, sizeof(line), fp)) {
		if (sscanf(line, "MemFree: %d kB", &free_mem) == 1)
		break;
	}

	fclose(fp);

	return free_mem;
}

int set_sock_opt(int num_raw_data_to_cap)
{
	/* Receive buffer size to be requested */
	int rbuff_sz_req = 0;
	/* Upper limit on receive buffer size to be requested */
	int rbuff_sz_req_limit = 0;
	/* Current receive buffer size */
	int rbuff_sz_curr = 0;
	/* Length of current receive buffer size datatype */
	socklen_t rbuff_sz_curr_len = 0;
	/* Free physical memory (not including caching */
	int free_mem = 0;
	int ret = 0;

	/* Get current receive buffer size */
	rbuff_sz_curr_len = sizeof(rbuff_sz_curr);
	ret = getsockopt(info->sock_fd, SOL_SOCKET, SO_RCVBUF,
				(void *)&rbuff_sz_curr,
				&rbuff_sz_curr_len);
	if (ret < 0) {
		perror("getsockopt\n");
		return ret;
	}

	/* Calculate upper limit on receive buffer size we'd like to request */
	ret = free_mem = get_free_mem_kB();
	if (ret < 0) {
		fprintf(stderr,
			"Could not determine amount of free physical memory\n");
		return ret;
	}
	rbuff_sz_req_limit = (int)(((float)free_mem * 1000) *
					QCA_SPECTOOL_MAX_FREEMEM_UTIL);

	/* Determine the receive buffer size to be requested */
	rbuff_sz_req = SAMPRECVBUF_SZ * sizeof(uint8_t) * num_raw_data_to_cap;

	if (rbuff_sz_req > rbuff_sz_req_limit)
		rbuff_sz_req = rbuff_sz_req_limit;

	if (rbuff_sz_req > rbuff_sz_curr) {
		/* We first try SO_RCVBUFFORCE. This is available since
		 * Linux 2.6.14, and if we have CAP_NET_ADMIN privileges.
		 *
		 * In case SO_RCVBUFFORCE is not available or we are not
		 * entitled to use it, then an error will be returned and
		 * we can fall back to SO_RCVBUF.
		 * If we use SO_RCVBUF, the kernel will cap our requested
		 * value as per rmem_max. We will have to survive with the
		 * possibility of a few netlink messages being lost under
		 * some circumstances.
		 */
		ret = setsockopt(info->sock_fd, SOL_SOCKET, SO_RCVBUFFORCE,
				(void *)&rbuff_sz_req, sizeof(rbuff_sz_req));

		if (ret < 0) {
			ret = setsockopt(info->sock_fd, SOL_SOCKET,
					SO_RCVBUF,
					(void *)&rbuff_sz_req,
					sizeof(rbuff_sz_req));
			if (ret < 0) {
				perror("setsockopt\n");
				return ret;
			}
		}
	}

	return 0;
}

#ifdef CONFIG_CLD80211_LIB
int spectral_scan_res_handler(struct nl_msg *msg, void *arg)
{
	struct nlattr *attrs[CLD80211_ATTR_MAX + 1];
	struct genlmsghdr *header;
	struct nlattr *tb_vendor[CLD80211_ATTR_MAX + 1];

	struct  nlmsghdr *nlh = nlmsg_hdr(msg);

	header = (struct genlmsghdr *)nlmsg_data(nlh);
	if (header->cmd == WLAN_NL_MSG_SPECTRAL_SCAN) {
		int result = nla_parse(attrs, CLD80211_ATTR_MAX,
				genlmsg_attrdata(header, 0),
				genlmsg_attrlen(header, 0), NULL);

		if (result || !attrs[CLD80211_ATTR_VENDOR_DATA]) {
			printf("No valid data received");
			return 0;
		}

		nla_parse(tb_vendor, CLD80211_ATTR_MAX,
			  (struct nlattr *)nla_data(
				attrs[CLD80211_ATTR_VENDOR_DATA]),
				nla_len(attrs[CLD80211_ATTR_VENDOR_DATA]),
						NULL);

		if (tb_vendor[CLD80211_ATTR_DATA]) {
			/* Compatible with converged host and old version */
			struct spectral_samp_msg_header *msg_header =
				nla_data(tb_vendor[CLD80211_ATTR_DATA]);
			if (msg_header->signature != SPECTRAL_SIGNATURE)
				return -1;

			dump_samp_msg(msg_header,
				      nla_len(tb_vendor[CLD80211_ATTR_DATA]));
		}
	}
	else
		printf("%s: CLD80211_ATTR_DATA not found\n",
					__FUNCTION__);

	return 0;
}

void recv_and_process_results(void)
{

	if (info->cldctx)
		cld80211_recv(info->cldctx, 500, true,
				spectral_scan_res_handler, NULL);
}
#else
void recv_and_process_results(void)
{
	struct timeval tv_timeout;
	fd_set readfds;
	struct sockaddr_nl src_addr, dest_addr;
	socklen_t fromlen;
	int ret = 0;
	int read_bytes = 0;
	uint8_t *samprecvbuf = NULL;
	struct spectral_samp_msg_header *msg_header;
	struct nlmsghdr *nlh = NULL;
	int num_rbuff_received = 0;

	samprecvbuf = (uint8_t *)malloc(SAMPRECVBUF_SZ * sizeof(uint8_t));
	if (samprecvbuf == NULL) {
		printf("Could not allocate buffer to receive SAMP data\n");
		ret = -1;
		goto out;
	}
	memset(samprecvbuf, 0, SAMPRECVBUF_SZ * sizeof(u_int8_t));

	printf("scan_count=%d\n", config_params[SPECTRAL_PARAM_SCAN_COUNT]);
	ret = set_sock_opt(config_params[SPECTRAL_PARAM_SCAN_COUNT]);
	if (ret < 0)
		goto out;

	FD_ZERO(&readfds);
	FD_SET(info->sock_fd, &readfds);

	while (num_rbuff_received < config_params[SPECTRAL_PARAM_SCAN_COUNT]) {
		tv_timeout.tv_sec = QCA_SPECTOOL_NL_TIMEOUT_SEC;
		tv_timeout.tv_usec = QCA_SPECTOOL_NL_TIMEOUT_USEC;

		ret = select(info->sock_fd + 1, &readfds, NULL, NULL,
				&tv_timeout);

		if (ret < 0) {
			perror("select\n");
			goto out;
		} else if (ret == 0) {
			printf("Warning - timed out waiting for messages.\n");
			break;
		} else if (!FD_ISSET(info->sock_fd, &readfds)) {
			fprintf(stderr, "Unexpected condition waiting for messages - no socket fd indicated\n");
			goto out;
		}

		fromlen = sizeof(src_addr);
		read_bytes = recvfrom(info->sock_fd, samprecvbuf,
				  SAMPRECVBUF_SZ * sizeof(uint8_t), MSG_WAITALL,
				  (struct sockaddr *) &src_addr, &fromlen);
		if (read_bytes < 0) {
			if (errno != ENOBUFS) {
				perror("recvfrom(netlink)\n");
				printf("Error reading netlink\n");
				goto out;
			}
		} else {
			printf("received %d bytes\n", read_bytes);

			nlh = (struct nlmsghdr *) samprecvbuf;
			msg_header = (struct spectral_samp_msg_header *)
					NLMSG_DATA(nlh);
			if (msg_header->signature != SPECTRAL_SIGNATURE)
				return;

			num_rbuff_received++;
			dump_samp_msg(msg_header, nlh->nlmsg_len);
		}
	}
out:
	if (samprecvbuf != NULL)
		free(samprecvbuf);

}
#endif

#define put_params(name) \
do { \
	if (config_params[SPECTRAL_PARAM_ ## name] != -1) { \
		printf("put %s value %d\n", # name, \
			config_params[SPECTRAL_PARAM_ ## name]); \
		nla_put_u32(msg, \
			QCA_WLAN_VENDOR_ATTR_SPECTRAL_SCAN_CONFIG_ ## name, \
			config_params[SPECTRAL_PARAM_ ## name]);  \
	} \
} while (0)

int spectral_scan_start(int ifidx)
{
	struct nl_msg *msg;
	int ret;
	struct nlattr *data;

	msg = nlmsg_alloc();
	if (!msg) {
		printf("alloc nl msg failed\n");
		return -ENOMEM;
	}
	genlmsg_put(msg, 0, 0, info->nl80211_family_id, 0, 0,
					NL80211_CMD_VENDOR, 0);
	nla_put_u32(msg, NL80211_ATTR_IFINDEX, ifidx);
	nla_put_u32(msg, NL80211_ATTR_VENDOR_ID, OUI_QCA);
	nla_put_u32(msg, NL80211_ATTR_VENDOR_SUBCMD,
			QCA_NL80211_VENDOR_SUBCMD_SPECTRAL_SCAN_START);
	data = nla_nest_start(msg, NL80211_ATTR_VENDOR_DATA);
	if (!data)
		return -EINVAL;

	put_params(SCAN_COUNT);
	put_params(SCAN_PERIOD);
	put_params(PRIORITY);
	put_params(FFT_SIZE);
	put_params(GC_ENA);
	put_params(RESTART_ENA);
	put_params(NOISE_FLOOR_REF);
	put_params(INIT_DELAY);
	put_params(NB_TONE_THR);
	put_params(STR_BIN_THR);
	put_params(WB_RPT_MODE);
	put_params(RSSI_RPT_MODE);
	put_params(RSSI_THR);
	put_params(PWR_FORMAT);
	put_params(RPT_MODE);
	put_params(BIN_SCALE);
	put_params(DBM_ADJ);
	put_params(CHN_MASK);
	nla_nest_end(msg, data);
	ret = send_and_recv(info, info->cmd_sock, msg, 0, 0);

	return ret;
}

int spectral_scan_start_and_recv(int ifidx)
{
	int ret;

	ret = spectral_scan_start(ifidx);
	if (ret < 0) {
		printf("Spectral scan start failed\n");
		return ret;
	}

	printf("wait for scan results...\n");
	recv_and_process_results();

	return ret;
}

int spectral_scan_stop(int ifidx)
{
	struct nl_msg *msg;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		printf("alloc nl msg failed\n");
		return -ENOMEM;
	}
	genlmsg_put(msg, 0, 0, info->nl80211_family_id, 0, 0,
				NL80211_CMD_VENDOR, 0);
	nla_put_u32(msg, NL80211_ATTR_IFINDEX, ifidx);
	nla_put_u32(msg, NL80211_ATTR_VENDOR_ID, OUI_QCA);
	nla_put_u32(msg, NL80211_ATTR_VENDOR_SUBCMD,
			QCA_NL80211_VENDOR_SUBCMD_SPECTRAL_SCAN_STOP);

	ret = send_and_recv(info, info->cmd_sock, msg, 0, 0);

	return ret;
}

struct spectral_samp_msg_info *g_msg_legacy;
struct spectral_samp_msg_info_umac_v0 *g_msg_umac_v0;
struct spectral_samp_msg_info_umac_v1 *g_msg_umac_v1;

#ifdef CONFIG_CLD80211_LIB
int getsample_handler(struct nl_msg *msg, void *arg)
{
	struct nlattr *attrs[CLD80211_ATTR_MAX + 1];
	struct genlmsghdr *header;
	struct nlattr *tb_vendor[CLD80211_ATTR_MAX + 1];
	struct spectral_samp_msg_header *msg_header;
	struct  nlmsghdr *nlh = nlmsg_hdr(msg);

	header = (struct genlmsghdr *)nlmsg_data(nlh);
	if (header->cmd == WLAN_NL_MSG_SPECTRAL_SCAN) {
		int result = nla_parse(attrs, CLD80211_ATTR_MAX,
				genlmsg_attrdata(header, 0),
				genlmsg_attrlen(header, 0), NULL);

		if (result || !attrs[CLD80211_ATTR_VENDOR_DATA]) {
			printf("No valid data received");
			return 0;
		}

		nla_parse(tb_vendor, CLD80211_ATTR_MAX,
			  (struct nlattr *)nla_data(
				attrs[CLD80211_ATTR_VENDOR_DATA]),
				nla_len(attrs[CLD80211_ATTR_VENDOR_DATA]),
						NULL);
		if (tb_vendor[CLD80211_ATTR_DATA]) {
			msg_header =
				(struct spectral_samp_msg_header *)
				nla_data(tb_vendor[CLD80211_ATTR_DATA]);
			if (msg_header->signature != SPECTRAL_SIGNATURE)
				return -1;

			if (IS_UMAC_SPECTRAL_SAMPLE_MSG(
				info->version,
				info->sub_version,
				nlh->nlmsg_len)) {
				if (info->sub_version == SPECTRAL_SUB_VERSION_1)
					g_msg_umac_v1 =
					(struct spectral_samp_msg_info_umac_v1 *)
					nla_data(tb_vendor[CLD80211_ATTR_DATA]);
				else
					g_msg_umac_v0 =
					(struct spectral_samp_msg_info_umac_v0 *)
					nla_data(tb_vendor[CLD80211_ATTR_DATA]);
			} else {
				g_msg_legacy = (struct spectral_samp_msg_info *)
				nla_data(tb_vendor[CLD80211_ATTR_DATA]);
			}
		}
	} else
		printf("%s: CLD80211_ATTR_DATA not found\n",
					__FUNCTION__);

	return 0;
}
#endif

int spectral_scan_getsample(int ifidx, int num_raw_data_to_cap)
{
	int ret = 0;
#ifndef CONFIG_CLD80211_LIB
	struct sockaddr_nl src_addr, dest_addr;
	socklen_t fromlen;
	struct nlmsghdr *nlh = NULL;
	int read_bytes = 0;
	struct msghdr msg;
#endif /* !CONFIG_CLD80211_LIB */
	uint8_t *samprecvbuf = NULL;
	uint16_t num_buf_written = 0;
	FILE *fp = NULL;
	uint8_t *bufSave = NULL, *buf_ptr = NULL;
	uint8_t *bufSave_sec80 = NULL, *buf_ptr_sec80 = NULL;
	int32_t *timeStp = NULL, *sampinfo = NULL;
	int32_t *rssi_nf_sec80 = NULL;
	uint8_t *is_160 = NULL;
	int is_pwr_format_enabled = 0;
	char delimiter = ' ';
	uint16_t num_rbuff_errors = 0;
#ifndef CONFIG_CLD80211_LIB
	struct timeval tv_timeout;
	fd_set readfds;
	struct spectral_samp_msg_header *msg_header;
#endif /* !CONFIG_CLD80211_LIB */

	/**
	 * Check if the user input is within the valid allowed range
	 * Note: scan_count is a 12-bit field => Range of 0-4095
	 * Allowed range is 1-4000 to account for the dropped packets
	 */
	if ((num_raw_data_to_cap < 0) ||
			(num_raw_data_to_cap > MAX_SIZE_CAPTURE)) {
		printf("Number of samples to capture out of range\n");
		printf("Enter valid input in the range 1 - %d\n",
					MAX_SIZE_CAPTURE);
		ret = -1;
		goto out;
	}

#ifndef CONFIG_CLD80211_LIB
	/* On some platforms and under some circumstances, our netlink message
	 * receive rate may not be able to keep up with the driver's send rate.
	 * This can result in receive buffer errors.
	 * To mitigate this, we increase the socket receive buffer size.
	 *
	 * An alternative considered is to have two threads, one purely for
	 * socket receive operations, the other for processing the received
	 * information.
	 * However, test results partially emulating this scenario showed that
	 * even with this, we can run into the receive buffer errors (due to
	 * the high rate at which the netlink messages arrive).
	 */
	ret = set_sock_opt(num_raw_data_to_cap);
	if (ret < 0)
		goto out;

	FD_ZERO(&readfds);
	FD_SET(info->sock_fd, &readfds);

#endif

	fp = fopen(sample_out_file, "wt");
	if (!fp) {
		printf("Could not open file to write\n");
		ret = -1;
		goto out;
	}

	samprecvbuf = (uint8_t *)malloc(SAMPRECVBUF_SZ * sizeof(uint8_t));
	if (samprecvbuf == NULL) {
		printf("Could not allocate buffer to receive SAMP data\n");
		ret = -1;
		goto out;
	}
	memset(samprecvbuf, 0, SAMPRECVBUF_SZ * sizeof(u_int8_t));

	bufSave = (uint8_t *)malloc(num_raw_data_to_cap *
					MAX_RAW_SPECT_DATA_SZ);
	if (bufSave == NULL) {
		printf("Could not allocate buffer to save Spectral bins\n");
		ret = -1;
		goto out;
	}
	memset(bufSave, 0, num_raw_data_to_cap * MAX_RAW_SPECT_DATA_SZ);

	bufSave_sec80 = (uint8_t *)malloc(num_raw_data_to_cap *
					   MAX_RAW_SPECT_DATA_SZ);
	if (bufSave_sec80 == NULL) {
		printf("Could not allocate buffer to save Spectral bins for secondary 80 MHz segment\n");
		ret = -1;
		goto out;
	}
	memset(bufSave_sec80, 0, num_raw_data_to_cap * MAX_RAW_SPECT_DATA_SZ);

	timeStp = (int32_t *)malloc(num_raw_data_to_cap * 3 * sizeof(int32_t));
	if (timeStp == NULL) {
		printf("Could not allocate buffers to save timestamp, RSSI and NF\n");
		ret = -1;
		goto out;
	}
	memset(timeStp, 0, num_raw_data_to_cap * 3 * sizeof(int32_t));

	rssi_nf_sec80 = (int32_t *)malloc(num_raw_data_to_cap * 2 *
						sizeof(int32_t));
	if (rssi_nf_sec80 == NULL) {
		printf("Could not allocate buffers to save RSSI and NF for secondary 80 MHz segment\n");
		ret = -1;
		goto out;
	}
	memset(rssi_nf_sec80, 0, num_raw_data_to_cap * 2 * sizeof(int32_t));

	sampinfo = (int32_t *)malloc(num_raw_data_to_cap * 2 * sizeof(int32_t));
	if (sampinfo == NULL) {
		printf("Could not allocate buffers to save sample info\n");
		ret = -1;
		goto out;
	}
	memset(sampinfo, 0, num_raw_data_to_cap * 2 * sizeof(int32_t));

	is_160 = (u_int8_t *)malloc(num_raw_data_to_cap * sizeof(u_int8_t));
	if (is_160 == NULL) {
		printf("Could not allocate buffers to save flag indicating if sample was taken at 160 MHz width\n");
		ret = -1;
		goto out;
	}
	memset(is_160, 0, num_raw_data_to_cap * sizeof(u_int8_t));

	buf_ptr = bufSave;
	buf_ptr_sec80 = bufSave_sec80;

	/**
	 * Setting scan count and starting spectral scan in case of N samples
	 * to capture
	 */
	config_params[SPECTRAL_PARAM_SCAN_COUNT] = num_raw_data_to_cap;
	spectral_scan_start(ifidx);

	printf("Waiting for message from kernel\n");

	while (num_buf_written < num_raw_data_to_cap) {
#ifdef CONFIG_CLD80211_LIB
		cld80211_recv(info->cldctx, 500, false, getsample_handler, NULL);
#else
		tv_timeout.tv_sec = QCA_SPECTOOL_NL_TIMEOUT_SEC;
		tv_timeout.tv_usec = QCA_SPECTOOL_NL_TIMEOUT_USEC;
		ret = select(info->sock_fd + 1, &readfds, NULL, NULL,
				&tv_timeout);

		if (ret < 0) {
			perror("select\n");
			goto out;
		} else if (0 == ret) {
			printf("Warning - timed out waiting for messages.\n");
			break;
		} else if (!FD_ISSET(info->sock_fd, &readfds)) {
			fprintf(stderr, "Unexpected condition waiting for messages - no socket fd indicated\n");
			ret = -EINVAL;
			goto out;
		}

		fromlen = sizeof(src_addr);
		read_bytes = recvfrom(info->sock_fd, samprecvbuf,
				  SAMPRECVBUF_SZ * sizeof(uint8_t), MSG_WAITALL,
				  (struct sockaddr *) &src_addr, &fromlen);
		if (read_bytes < 0) {
			if (ENOBUFS == errno)
				num_rbuff_errors++;
			else {
				perror("recvfrom(netlink)\n");
				printf("Error reading netlink\n");
				ret = read_bytes;
				goto out;
			}
		}

		nlh = (struct nlmsghdr *) samprecvbuf;
		msg_header = (struct spectral_samp_msg_header *) NLMSG_DATA(nlh);
		if (msg_header->signature != SPECTRAL_SIGNATURE)
			continue;

		if (IS_UMAC_SPECTRAL_SAMPLE_MSG(
			info->version,
			info->sub_version,
			nlh->nlmsg_len)) {
			if (info->sub_version == SPECTRAL_SUB_VERSION_1)
				g_msg_umac_v1 =
					(struct spectral_samp_msg_info_umac_v1 *)
					NLMSG_DATA(nlh);
			else
				g_msg_umac_v0 =
					(struct spectral_samp_msg_info_umac_v0 *)
					NLMSG_DATA(nlh);
		} else {
			g_msg_legacy = (struct spectral_samp_msg_info *)
				NLMSG_DATA(nlh);
		}
#endif
		if (IS_UMAC_SPECTRAL_SAMPLE_MSG(
			info->version,
			info->sub_version, 0)) {

			/* sampinfo will be re-used for secondary 80 MHz
			 * if applicable, since it is known that the number of
			 * samples will be the same for both 80 MHz segments.
			 *
			 * Similarly, time stamp will be reused.
			 *
			 * XXX: Remove this space optimization if the delivery
			 * format for future 160 MHz chipsets post QCA9984
			 * changes and a difference in
			 * number of samples/timestamps becomes possible.
			 */
			if (info->sub_version == SPECTRAL_SUB_VERSION_1) {
				if (g_msg_umac_v1) {
					save_msg(g_msg_umac_v1);
				} else {
					printf("g_msg_umac_v1 is NULL\n");
					num_buf_written++;
					continue;
				}
			} else {
				if (g_msg_umac_v0) {
					save_msg(g_msg_umac_v0);
				} else {
					printf("g_msg_umac_v0 is NULL\n");
					num_buf_written++;
					continue;
				}
			}
		} else {
			if (g_msg_legacy) {
				save_msg(g_msg_legacy);
			}
		}
		/* Irrespective of whether the current FFT bins were at
		 * 160 MHz or not, we skip to next position to be in
		 * sync with first (original) segment's FFT bin
		 * positions. Hence the increment outside of check of
		 * 160 MHz width.
		 */
		buf_ptr_sec80 += MAX_RAW_SPECT_DATA_SZ;

		num_buf_written++;
		g_msg_umac_v1 = NULL;
		g_msg_umac_v0 = NULL;
		g_msg_legacy = NULL;
	}

	/* Get current configurations */
	is_pwr_format_enabled = config_params[SPECTRAL_PARAM_PWR_FORMAT];

	/* Stopping spectral scan */
	spectral_scan_stop(ifidx);

	printf("Number of samples captured %d\n",
			num_buf_written);

	if (num_rbuff_errors)
		printf("Warning: %hu receive buffer errors. Some samples were lost due to receive-rate constraints\n",
				num_rbuff_errors);

	{
		uint16_t cnt, valCnt;

		buf_ptr = bufSave;
		buf_ptr_sec80 = bufSave_sec80;

		for (cnt = 0; cnt < num_buf_written; cnt++) {
			fprintf(fp, "%u %c ", (unsigned)sampinfo[cnt * 2],
							delimiter);
			fprintf(fp, "%u %c ", (unsigned)sampinfo[cnt * 2 + 1],
							delimiter);

			if (is_pwr_format_enabled)
				for (valCnt = 0; valCnt <
					(unsigned)sampinfo[cnt * 2 + 1];
					valCnt++)
					fprintf(fp, "%d %c ",
						(int8_t)(buf_ptr[valCnt]),
						delimiter);
			else
				for (valCnt = 0; valCnt <
						(unsigned)sampinfo[cnt * 2 + 1];
						valCnt++)
					fprintf(fp, "%u %c ",
						(u_int8_t)(buf_ptr[valCnt]),
						delimiter);

			fprintf(fp, "%u %c ", (unsigned)timeStp[cnt * 3],
						delimiter);
			fprintf(fp, "%d %c ", timeStp[cnt * 3 + 1], delimiter);
			fprintf(fp, "%d %c ", timeStp[cnt * 3 + 2], delimiter);
			fprintf(fp, "\n");
			buf_ptr += MAX_RAW_SPECT_DATA_SZ;

			if (is_160[cnt]) {
				fprintf(fp, "%u %c ",
					(unsigned)sampinfo[cnt * 2], delimiter);
				fprintf(fp, "%u %c ",
					(unsigned)sampinfo[cnt * 2 + 1],
					delimiter);

				for (valCnt = 0; valCnt <
						(unsigned)sampinfo[cnt * 2 + 1];
						valCnt++)
					fprintf(fp, "%u %c ",
						(u_int8_t)
						(buf_ptr_sec80[valCnt]),
						delimiter);

				fprintf(fp, "%u %c ",
						(unsigned)timeStp[cnt * 3],
						delimiter);
				fprintf(fp, "%d %c ",
					rssi_nf_sec80[cnt * 2], delimiter);
				fprintf(fp, "%d %c ",
					rssi_nf_sec80[cnt * 2 + 1], delimiter);
				fprintf(fp, "\n");
			}

			buf_ptr_sec80 += MAX_RAW_SPECT_DATA_SZ;
		}
	}

out:
	if (fp != NULL)
		fclose(fp);

	if (samprecvbuf != NULL)
		free(samprecvbuf);

	if (bufSave != NULL)
		free(bufSave);

	if (bufSave_sec80 != NULL)
		free(bufSave_sec80);

	if (sampinfo != NULL)
		free(sampinfo);

	if (timeStp != NULL)
		free(timeStp);

	if (rssi_nf_sec80 != NULL)
		free(rssi_nf_sec80);

	if (is_160 != NULL)
		free(is_160);

	return ret;
}

static int32_t create_nl_socket(void)
{
	int32_t ret;
	int32_t sock_fd;
	struct sockaddr_nl src_addr;

	sock_fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_USERSOCK);
	if (sock_fd < 0) {
		fprintf(stderr, "Socket creation failed sock_fd 0x%x\n",
			sock_fd);
		return -EINVAL;
	}

	memset(&src_addr, 0, sizeof(src_addr));
	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_groups = 0x0;
	src_addr.nl_pid = getpid(); /* self pid */

	ret = bind(sock_fd, (struct sockaddr *)&src_addr, sizeof(src_addr));
	if (ret < 0) {
		close(sock_fd);
		return ret;
	}
	return sock_fd;
}

struct nlIfaceInfo *initialize(void)
{
	struct nl_sock *cmd_sock = NULL;
	int ret;
	uint32_t pid = getpid() & 0x3FFFFF;

	struct nlIfaceInfo *info = (struct nlIfaceInfo *)malloc( sizeof(
							struct nlIfaceInfo));
	if (info == NULL) {
		printf("Could not allocate nlIfaceInfo\n");
		return NULL;
	}

	info->nl_cb = nl_cb_alloc(NL_CB_DEFAULT);
	if (info->nl_cb == NULL) {
		printf("Failed to allocate netlink callbacks");
		free(info);
		return NULL;
	}

	cmd_sock = nl_socket_alloc_cb(info->nl_cb);
	if (!cmd_sock) {
		free(info);
		return NULL;
	}

	nl_socket_set_local_port(cmd_sock, pid);
	if (genl_connect(cmd_sock)) {
		printf("Failed to connect to generic netlink cmd_sock");
		nl_socket_free(cmd_sock);
		free(info);
		return NULL;
	}

#ifdef CONFIG_CLD80211_LIB
	info->cldctx = cld80211_init();
	if (info->cldctx != NULL) {
		ret = cld80211_add_mcast_group(info->cldctx, "oem_msgs");
		if (ret) {
			printf("Failed to add mcast group: oem_msgs");
			goto cleanup;
		}
	} else
#endif /* CONFIG_CLD80211_LIB */
	{
		printf("Failed to initialize cld80211 library,"
		               " proceed with legacy procedure\n");
		info->sock_fd = create_nl_socket();
		if (info->sock_fd < 0)
			goto cleanup;
	}

	/* Set the socket buffer size */
	if (nl_socket_set_buffer_size(cmd_sock, (256*1024), 0) < 0)
		printf("Could not set nl_socket RX buffer size for cmd_sock: %s\n",
				strerror(errno));
	/* continue anyway with the default (smaller) buffer */

	info->cmd_sock = cmd_sock;

	info->nl80211_family_id = genl_ctrl_resolve(cmd_sock, "nl80211");
	if (info->nl80211_family_id < 0) {
		printf("Could not resolve nl80211 familty id\n");
		goto cleanup;
	}

	pthread_mutex_init(&info->cb_lock, NULL);

	return info;
cleanup:
#ifdef CONFIG_CLD80211_LIB
	cld80211_deinit(info->cldctx);
	info->cldctx = NULL;
#endif
	if (cmd_sock)
		nl_socket_free(cmd_sock);

	if (info)
		free(info);

	return NULL;
}

static int event_handler(struct nl_msg *msg, void *arg)
{
	struct nlattr *attrs[CLD80211_ATTR_MAX + 1];
	struct genlmsghdr *header;
	struct nlattr *tb_vendor[CLD80211_ATTR_MAX + 1];

	struct  nlmsghdr *nlh = nlmsg_hdr(msg);

	header = (struct genlmsghdr *)nlmsg_data(nlh);
	if (header->cmd == WLAN_NL_MSG_SPECTRAL_SCAN) {
		int result = nla_parse(attrs, CLD80211_ATTR_MAX,
				genlmsg_attrdata(header, 0),
				genlmsg_attrlen(header, 0), NULL);

		if (result || !attrs[CLD80211_ATTR_VENDOR_DATA]) {
			printf("No valid data received");
			return 0;
		}

		nla_parse(tb_vendor, CLD80211_ATTR_MAX,
			  (struct nlattr *)nla_data(
				attrs[CLD80211_ATTR_VENDOR_DATA]),
				nla_len(attrs[CLD80211_ATTR_VENDOR_DATA]),
						NULL);

		if (tb_vendor[CLD80211_ATTR_DATA]) {
			if (IS_INCLUDE_VERSION(nla_len(
				tb_vendor[CLD80211_ATTR_DATA]))) {
				struct spectral_scan_msg_v *ss_recvmsg_v =
					(struct spectral_scan_msg_v *)nla_data(
					tb_vendor[CLD80211_ATTR_DATA]);
				info->version = ss_recvmsg_v->version;
				info->sub_version = ss_recvmsg_v->sub_version;
				printf("received spectral scan reg response %d, version: %d.%d\n",
				       ss_recvmsg_v->msg_type, info->version,
				       info->sub_version);
				if (info->version > SPECTRAL_MAX_VERSION ||
				    ((info->version == SPECTRAL_MAX_VERSION) &&
				     (info->sub_version > SPECTRAL_MAX_SUB_VERSION))) {
					printf("spectraltool doesn't spport current host, tool support max version %d, sub version %d, host %d.%d\n",
					       SPECTRAL_MAX_VERSION,
					       SPECTRAL_MAX_SUB_VERSION,
					       info->version,
					       info->sub_version);
				}
			} else {
				struct spectral_scan_msg *ss_recvmsg =
			              (struct spectral_scan_msg *)nla_data(
					tb_vendor[CLD80211_ATTR_DATA]);
				info->version = SPECTRAL_VERSION_2;
				info->sub_version = SPECTRAL_SUB_VERSION_0;
				printf("received spectral scan reg response %d, force set to version: %d.%d\n",
				       ss_recvmsg->msg_type, info->version,
				       info->sub_version);
			}
		}
		else
			printf("%s: CLD80211_ATTR_DATA not found",
					__FUNCTION__);

	}

	return 0;
}

int send_cld80211_nlmsg(void *cldctx, int cmd, void *data,
			int len, int pid)
{
	int ret = -1;
	struct nlattr *nla_data = NULL;
	struct nl_msg *nlmsg;

	nlmsg = (struct nl_msg *)cld80211_msg_alloc(cldctx, cmd,
							&nla_data, pid);
	if (!nlmsg || !nla_data) {
		printf("%s: NL message allocation failed %d\n",
			__FUNCTION__, ret);
		goto end;
	}
	ret = nla_put(nlmsg, CLD80211_ATTR_DATA, len, data);
	if (ret)
		goto end;
	nla_nest_end(nlmsg, nla_data);

	ret = cld80211_send_recv_msg(cldctx, nlmsg, event_handler, NULL);
	if (ret)
		goto end;
	printf("%s: successfully sent cld80211 message for pid %d\n",
			__FUNCTION__, pid);

end:
	if (nlmsg)
	free(nlmsg);
	return ret;
}


int register_to_driver(struct nlIfaceInfo *info)
{
	int ret;
	struct spectral_scan_msg spectral_msg, *ss_recvmsg;
	struct spectral_scan_msg_v *ss_recvmsg_v;

	spectral_msg.msg_type = SPECTRAL_SCAN_REGISTER_REQ;
	spectral_msg.pid = getpid();

	if (info->cldctx != NULL) {
		ret = send_cld80211_nlmsg(info->cldctx,
			WLAN_NL_MSG_SPECTRAL_SCAN, &spectral_msg,
			sizeof(struct spectral_scan_msg), getpid());
		if (ret < 0) {
			printf("Failed to send WLAN_NL_MSG_SPECTRAL_SCAN"
				" to driver\n");
			return ret;
		}
	}
	else
	{
		struct sockaddr_nl dest_addr;
		struct nlmsghdr *nlh = NULL;
		struct msghdr msg;
		struct iovec iov;
		size_t len;

		memset(&dest_addr, 0, sizeof(dest_addr));
		dest_addr.nl_family = AF_NETLINK;
		dest_addr.nl_pid = 0; /* For Linux Kernel */
		dest_addr.nl_groups = 0; /* unicast */

		len = sizeof(struct spectral_scan_msg);
		nlh = malloc(NLMSG_SPACE(len));
		if (nlh == NULL) {
			fprintf(stderr, "Cannot allocate memory\n");
			return -ENOMEM;
		}
		memset(nlh, 0, NLMSG_SPACE(len));
		nlh->nlmsg_len = NLMSG_SPACE(len);
		nlh->nlmsg_pid = getpid();
		nlh->nlmsg_type = WLAN_NL_MSG_SPECTRAL_SCAN;
		nlh->nlmsg_flags = NLM_F_REQUEST;

		memcpy(NLMSG_DATA(nlh), &spectral_msg, len);

		memset(&msg, 0, sizeof(msg));
		iov.iov_base = (void *)nlh;
		iov.iov_len = nlh->nlmsg_len;
		msg.msg_name = (void *)&dest_addr;
		msg.msg_namelen = sizeof(dest_addr);
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;

		printf("registering with driver, pid=%d\n", nlh->nlmsg_pid);
			ret  = sendmsg(info->sock_fd, &msg, 0);

		nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(256));
		if (nlh == NULL) {
			printf("Cannot allocate memory!");
			return -ENOMEM;
		}
		memset(nlh, 0, NLMSG_SPACE(256));
		nlh->nlmsg_len = NLMSG_SPACE(256);
		nlh->nlmsg_pid = getpid();

		iov.iov_base = (void *)nlh;
		iov.iov_len = nlh->nlmsg_len;

		memset(&dest_addr, 0, sizeof(dest_addr));
		dest_addr.nl_family = AF_NETLINK;
		dest_addr.nl_pid = 0; /* For Linux Kernel */
		dest_addr.nl_groups = 1;

		memset(&msg, 0, sizeof(msg));
		msg.msg_name = (void *)&dest_addr;
		msg.msg_namelen = sizeof(dest_addr);
		msg.msg_iov = &iov;
		msg.msg_iovlen = 1;
		while (1) {
			ret = recvmsg(info->sock_fd, &msg, 0);
			if (ret < 0)
				continue;
			else
				break;
		}

		if (nlh) {
			if (nlh->nlmsg_type == WLAN_NL_MSG_SPECTRAL_SCAN) {
				if (IS_INCLUDE_VERSION(nlh->nlmsg_len)) {
					ss_recvmsg_v =
						(struct spectral_scan_msg_v *)
						(NLMSG_DATA(nlh));
					info->version = ss_recvmsg_v->version;
					info->sub_version =
						ss_recvmsg_v->sub_version;
					printf("received spectral scan reg response %d, version: %d.%d\n",
					       ss_recvmsg_v->msg_type,
					       info->version,
					       info->sub_version);
				} else {
					ss_recvmsg =
						(struct spectral_scan_msg *)
						(NLMSG_DATA(nlh));
					info->version = SPECTRAL_VERSION_2;
					info->sub_version =
							SPECTRAL_SUB_VERSION_0;
					printf("received spectral scan reg response %d, force set to version: %d.%d\n",
					       ss_recvmsg->msg_type,
					       info->version,
					       info->sub_version);
				}
			} else
				printf("msg_type invalid %d\n",
						nlh->nlmsg_type);
		} else
			printf("nlh is null\n");

		free(nlh);
	}
	return 0;
}

void strupper(char *str)
{
	while (*str) {
		*str = toupper(*str);
		str++;
	}
}

char *strnstr(const char *src, const char *dst, size_t len)
{
	size_t dst_len;

	dst_len = strlen(dst);
	if (!dst_len)
		return (char *)src;
	while (len >= dst_len) {
		len--;
		if (!memcmp(src, dst, dst_len))
			return (char *)src;
		src++;
	}

	return NULL;
}

#define set_params(name) \
do { \
	if (strnstr(token[i], #name, MAX_CONFIG_LEN)) { \
		tmp = strnstr(token[i], "=", MAX_CONFIG_LEN); \
		if (tmp) { \
			tmp++; \
			config_params[SPECTRAL_PARAM_ ## name] = \
				(int)strtol(tmp, &end_ptr, 0); \
		} else { \
			printf("Invalid param format %s\n", \
				 token[i]); \
			return -EINVAL; \
		} \
	} \
} while (0)

void usage(void)
{
	printf("spscan -i <ifname>\n");
}

int main(int argc, char **argv)
{
	int ret, ifidx, i;
	char *ifname = NULL;
	char *tmp = NULL;
	enum spectral_scan_command cmd;
	char cmd_str[MAX_CMD_LEN];
	char token[MAX_NUM_OF_PARAMS + 1][MAX_CONFIG_LEN];
	int token_num, num_of_samples;
	char *end_ptr;

	if (argc != 3) {
		usage();
		return 0;
	}

	/* get ifname */
	tmp = argv[1];
	if (!strcmp(tmp, "-i"))
		ifname = argv[2];
	else {
		printf("interface name is needed: -i <ifname>\n");
		return 0;
	}

	ifidx = if_nametoindex(ifname);
	if (ifidx == -1) {
		printf("Invalid ifname %s\n", argv[2]);
		return 0;
	}

	info = initialize();
	if (info == NULL) {
		printf("Failed to initialize sockets\n");
		return -EINVAL;
	}

	printf("register with driver...\n");
	ret = register_to_driver(info);
	if (ret) {
		printf("registration with driver failed\n");
		return ret;
	}

	/* get config parameters */
	reset_spectral_configs();
	while (1) {
		printf("spectral_scan_cmd>");
		fgets(cmd_str, MAX_CMD_LEN, stdin);
		if (!strcmp(cmd_str, "\n"))
			continue;
		tmp = &cmd_str[0];
		i = 0;
		int len = 0;
		char *tmp1;

		while (*tmp != '\0') {
			tmp1 = tmp;
			len = 0;
			while (*tmp1 != ' ' && *tmp1 != '\n' && len < (MAX_CONFIG_LEN - 1)) {
				len++;
				tmp1++;
			}
			strlcpy(token[i], tmp, sizeof(token[i]));
			token[i][len] = '\0';
			tmp += len;
			if (*tmp ==  '\n')
				break;
			tmp++;
			i++;
		}

		token_num = i + 1;

		/* get command */
		if (!strcmp(token[0], "start"))
			cmd = SPECTRAL_SCAN_START;
		else if (!strcmp(token[0], "stop"))
			cmd = SPECTRAL_SCAN_STOP;
		else if (!strcmp(token[0], "config"))
			cmd = SPECTRAL_SCAN_CONFIG;
		else if (!strcmp(token[0], "getsample"))
			cmd = SPECTRAL_SCAN_GETSAMPLE;
		else if (!strcmp(token[0], "quit"))
			cmd = SPECTRAL_SCAN_EXIT;
		else {
			printf("Invalid command %s\n", token[0]);
			return 0;
		}

		switch (cmd) {
		case SPECTRAL_SCAN_START:
			ret = spectral_scan_start_and_recv(ifidx);
			if (ret)
				printf("spectral scan start failed\n");
			break;
		case SPECTRAL_SCAN_STOP:
			ret = spectral_scan_stop(ifidx);
			if (ret)
				printf("spectral scan stop failed\n");
			break;
		case SPECTRAL_SCAN_CONFIG:
			for (i = 1; i < token_num; i++) {
				strupper(token[i]);
				set_params(SCAN_COUNT);
				set_params(SCAN_PERIOD);
				set_params(PRIORITY);
				set_params(FFT_SIZE);
				set_params(GC_ENA);
				set_params(RESTART_ENA);
				set_params(NOISE_FLOOR_REF);
				set_params(INIT_DELAY);
				set_params(NB_TONE_THR);
				set_params(STR_BIN_THR);
				set_params(WB_RPT_MODE);
				set_params(RSSI_RPT_MODE);
				set_params(RSSI_THR);
				set_params(PWR_FORMAT);
				set_params(RPT_MODE);
				set_params(BIN_SCALE);
				set_params(DBM_ADJ);
				set_params(CHN_MASK);
			}
			break;
		case SPECTRAL_SCAN_GETSAMPLE:
			if (token_num >= 3) {
				num_of_samples = atoi(token[1]);
				if (strnstr(token[2], "file_name",
						MAX_CONFIG_LEN)) {
					tmp = strnstr(token[2], "=",
						MAX_CONFIG_LEN);
					if (tmp) {
						tmp++;
						sample_out_file = tmp;
						printf("User specified file %s\n",
							sample_out_file);
					} else {
						printf("Invalid param format %s\n",
								token[2]);
					}
				} else
					printf("Invalid param format %s\n",
								token[2]);
			} else if (token_num == 2)
				num_of_samples = atoi(token[1]);
			else
				num_of_samples = 50;
			ret = spectral_scan_getsample(ifidx, num_of_samples);
			if (ret < 0)
				printf("spectral scan get sample failed\n");
			break;
		case SPECTRAL_SCAN_EXIT:
			goto out;
		default:
			printf("Invalid command\n");
		}
	}
out:
	if (info->sock_fd >= 0)
		close(info->sock_fd);

	if (info->cmd_sock)
		nl_socket_free(info->cmd_sock);

	if (info)
		free(info);

	return 0;
}
