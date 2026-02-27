/*
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef __NL_WRAPPER__
#define __NL_WRAPPER__

#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <netlink/object-api.h>
#include <linux/pkt_sched.h>

#include "nl80211_copy.h"
#ifdef CONFIG_CLD80211_LIB
#include <cld80211_lib.h>
#else
#include <netlink-types.h>
#endif

#define MAX_NUM_OF_PARAMS       (18)
#define SOCKET_BUFFER_SIZE      (32768U)
#define RECV_BUF_SIZE           (4096)
#define DEFAULT_EVENT_CB_SIZE   (64)
#define DEFAULT_CMD_SIZE        (64)
#define SPECTRAL_SIGNATURE  0xdeadbeef
#define MAX_CMD_LEN		256
#define MAX_CONFIG_LEN		128

/* libnl 2.0 compatibility code */
#define nl_handle nl_sock
#define nl80211_handle_alloc nl_socket_alloc_cb
#define nl80211_handle_destroy nl_socket_free

#define WLAN_NL_MSG_SPECTRAL_SCAN   29
#define MAX_SIZE_CAPTURE 50000
/*
 * Maximum portion of free physical memory we allow ourselves to request for
 * while setting socket receive buffer size. This does not include cached
 * memory.
 * This is a float on a scale of 0-1.
 *
 * Note that the kernel doubles the value we request for, to account for
 * bookkeeping overhead. Be mindful of this when changing the below.
 */
#define QCA_SPECTOOL_MAX_FREEMEM_UTIL       (.30f)
#define SAMPRECVBUF_SZ        (2048)
#define MAX_RAW_SPECT_DATA_SZ (600)
/* Netlink timeout specification (second and microsecond components) */
#define QCA_SPECTOOL_NL_TIMEOUT_SEC         (2)
#define QCA_SPECTOOL_NL_TIMEOUT_USEC        (0)

#define SPECTRAL_PARAM_SCAN_COUNT                       0
#define SPECTRAL_PARAM_SCAN_PERIOD                      1
#define SPECTRAL_PARAM_PRIORITY                         2
#define SPECTRAL_PARAM_FFT_SIZE                         3
#define SPECTRAL_PARAM_GC_ENA                           4
#define SPECTRAL_PARAM_RESTART_ENA                      5
#define SPECTRAL_PARAM_NOISE_FLOOR_REF                  6
#define SPECTRAL_PARAM_INIT_DELAY                       7
#define SPECTRAL_PARAM_NB_TONE_THR                      8
#define SPECTRAL_PARAM_STR_BIN_THR                      9
#define SPECTRAL_PARAM_WB_RPT_MODE                      10
#define SPECTRAL_PARAM_RSSI_RPT_MODE                    11
#define SPECTRAL_PARAM_RSSI_THR                         12
#define SPECTRAL_PARAM_PWR_FORMAT                       13
#define SPECTRAL_PARAM_RPT_MODE                         14
#define SPECTRAL_PARAM_BIN_SCALE                        15
#define SPECTRAL_PARAM_DBM_ADJ                          16
#define SPECTRAL_PARAM_CHN_MASK                         17

#define SPECTRAL_VERSION_2                              2
#define SPECTRAL_VERSION_3                              3
#define SPECTRAL_SUB_VERSION_0                          0
#define SPECTRAL_SUB_VERSION_1                          1
#define SPECTRAL_MAX_VERSION SPECTRAL_VERSION_3
#define SPECTRAL_MAX_SUB_VERSION SPECTRAL_SUB_VERSION_1
#define IS_INCLUDE_VERSION(len) ((len) > (2 * sizeof(uint32_t)))

#define IS_UMAC_SPECTRAL_SAMPLE_MSG(ver, sub, len) \
	(((ver) >= SPECTRAL_VERSION_3) || \
	 (((ver) == SPECTRAL_VERSION_2) && \
	 ((sub) == SPECTRAL_SUB_VERSION_1)) || \
	 ((len) > (sizeof(struct spectral_samp_msg_info) + sizeof(uint32_t))))

#define output_sample_msg(msg) \
	printf("signature=%x\n", msg->signature); \
	printf("freq=%d\n", msg->freq); \
	printf("vhtop_ch_freq_seg1=%d\n", msg->vhtop_ch_freq_seg1); \
	printf("vhtop_ch_freq_seg2=%d\n", msg->vhtop_ch_freq_seg2); \
	printf("freq_loading=%d\n", msg->freq_loading); \
	printf("int_type=%d\n", msg->int_type); \
	printf("bin_pwr_count=%d\n", msg->samp_data.bin_pwr_count); \
	printf("bin_pwr:\n"); \
	for (i = 0; i < msg->samp_data.bin_pwr_count; i++) \
		printf("%d ", msg->samp_data.bin_pwr[i]); \
	printf("\n"); \
	printf("timestamp=%d\n", msg->samp_data.spectral_tstamp); \
	printf("noise_floor=%d\n", msg->samp_data.noise_floor);

#define save_msg(msg) \
{ \
	sampinfo[num_buf_written * 2] = num_buf_written; \
	sampinfo[num_buf_written * 2 + 1] = \
			msg->samp_data.bin_pwr_count; \
 \
	memcpy(buf_ptr, msg->samp_data.bin_pwr, \
			msg->samp_data.bin_pwr_count); \
	buf_ptr += MAX_RAW_SPECT_DATA_SZ; \
 \
	timeStp[num_buf_written * 3] = \
			msg->samp_data.spectral_tstamp; \
	timeStp[num_buf_written * 3 + 1] = \
			msg->samp_data.spectral_rssi; \
	timeStp[num_buf_written * 3 + 2] = \
			msg->samp_data.noise_floor; \
	{ \
		static int nf_cnt; \
 \
		nf_cnt++; \
		if (nf_cnt == num_raw_data_to_cap) { \
			printf("Noise Floor %d\n", \
			msg->samp_data.noise_floor); \
			nf_cnt = 0; \
		} \
 \
	} \
 \
	if (msg->samp_data.ch_width == CH_WIDTH_160MHZ) { \
		is_160[num_buf_written] = 1; \
 \
		memcpy(buf_ptr_sec80, \
			   msg->samp_data.bin_pwr_sec80, \
			   msg->samp_data. \
			   bin_pwr_count_sec80); \
 \
		rssi_nf_sec80[num_buf_written * 2] = \
			msg->samp_data. \
			spectral_rssi_sec80; \
		rssi_nf_sec80[num_buf_written * 2 + 1] = \
			msg->samp_data.noise_floor_sec80; \
	} \
}

enum spectral_scan_command {
	SPECTRAL_SCAN_START,
	SPECTRAL_SCAN_STOP,
	SPECTRAL_SCAN_CONFIG,
	SPECTRAL_SCAN_GETSAMPLE,
	SPECTRAL_SCAN_EXIT,
};


struct nlIfaceInfo {
	struct nl_sock *cmd_sock;
	int sock_fd;
	struct nl_cb *nl_cb;
	int num_event_cb;
	int alloc_event_cb;
	pthread_mutex_t cb_lock;
	int nl80211_family_id;
	uint8_t clean_up;
#ifdef CONFIG_CLD80211_LIB
	void *cldctx;
#endif
	uint32_t version;
	uint32_t sub_version;
};

struct spectral_scan_msg {
	uint32_t msg_type;
	uint32_t pid;
};

struct spectral_scan_msg_v {
	uint32_t msg_type;
	uint32_t pid;
	uint32_t version;
	uint32_t sub_version;
};

enum spectral_scan_msg_type {
	SPECTRAL_SCAN_REGISTER_REQ,
	SPECTRAL_SCAN_REGISTER_RSP,
};

enum phy_ch_width {
	CH_WIDTH_20MHZ = 0,
	CH_WIDTH_40MHZ,
	CH_WIDTH_80MHZ,
	CH_WIDTH_160MHZ,
	CH_WIDTH_80P80MHZ,
	CH_WIDTH_5MHZ,
	CH_WIDTH_10MHZ,
	CH_WIDTH_INVALID,
	CH_WIDTH_MAX
};

#endif
