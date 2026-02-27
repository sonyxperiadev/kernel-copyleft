/*
 * Copyright (c) 2015-2017 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

/*
 * Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

#ifndef _CLD_DIAG_PARSER_H
#define _CLD_DIAG_PARSER_H

#define FEATURE_LOG_EXPOSED_HEADER

#include <stdint.h>
#ifndef __KERNEL__
#include <sys/socket.h>
#include <netinet/in.h>
#endif // __KERNEL__
#ifdef USE_GLIB
#include <glib.h>
#define strlcat g_strlcat
#define strlcpy g_strlcpy
#endif
#include <athdefs.h>
#include <a_types.h>
#include "dbglog_common.h"

#define INIT_WITH_SLEEP            15
#define CLD_NETLINK_USER           17

#define LOGFILE_FLAG               0x01
#define CONSOLE_FLAG               0x02
#define QXDM_FLAG                  0x04
#define SILENT_FLAG                0x08
#define DEBUG_FLAG                 0x10
#define PARSE_FLAG                 0x20
#define WITHOUT_RING_FLAG          0x01
#define HDRLEN                     16

#define RECLEN (HDRLEN + ATH6KL_FWLOG_PAYLOAD_SIZE)
#define READ_LEN     (2 * 1024 * 1024)

static inline unsigned int get_32(const unsigned char *pos)
{
	return pos[0] | (pos[1] << 8) | (pos[2] << 16) | (pos[3] << 24);
}

/* General purpose MACROS to handle the WNI Netlink msgs */
#define ANI_NL_MASK        3

/*
 * The following enum defines the target kernel module for which the netlink
 * message is intended for. Each kernel module along with its counterpart
 * in the user space, will then define a set of messages they recognize.
 * Each of this message will have an header of type tAniHdr define below.
 * Each Netlink message to/from a kernel module will contain only one
 * message which is preceded by a tAniHdr.
 *
 *         +------------+-------+-------+----------+
 *         |Netlink hdr | Align |tAniHdr| msg body |
 *         +------------+-------+-------|----------+
 */
#define ANI_NL_MSG_BASE     0x10    /* Some arbitrary base */
typedef enum eAniNlModuleTypes {
	ANI_NL_MSG_NETSIM = ANI_NL_MSG_BASE,// NetSim Messages (to the server)
	ANI_NL_MSG_PUMAC,       // Messages for/from the Upper MAC driver
	ANI_NL_MSG_WNS,         // Messages for the Wireless Networking
	//  Services module(s)
	ANI_NL_MSG_MACSW,       // Messages from MAC
	ANI_NL_MSG_ES,          // Messages from ES
	ANI_NL_MSG_WSM,         // Message from the WSM in user space
	ANI_NL_MSG_DVT,         // Message from the DVT application
	ANI_NL_MSG_PTT,         // Message from the PTT application
	ANI_NL_MSG_MAC_CLONE,     //Message from the Mac clone App
	ANI_NL_MSG_LOG = ANI_NL_MSG_BASE + 0x0C, // Message for WLAN logging
	ANI_NL_MSG_MAX
} tAniNlModTypes;

//All Netlink messages must contain this header
typedef struct sAniHdr {
	unsigned short type;
	unsigned short length;
} tAniHdr, tAniMsgHdr;

/*
 * This msg hdr will always follow tAniHdr in all the messages exchanged
 * between the Applications in userspace the Pseudo Driver, in either
 * direction.
 */
typedef struct sAniNlMsg {
	struct  nlmsghdr nlh;   // Netlink Header
	int radio;          // unit number of the radio
	tAniHdr wmsg;       // Airgo Message Header
} tAniNlHdr;

typedef struct sAniAppRegReq {
	tAniNlModTypes type;    /* The module id that the application is
			           registering for */
	int pid;            /* Pid returned in the nl_sockaddr structure
			    in the call getsockbyname after the
			    application opens and binds a netlink
			    socket */
} tAniNlAppRegReq;

static inline unsigned int aniNlAlign(unsigned int len)
{
	return ((len + ANI_NL_MASK) & ~ANI_NL_MASK);
}

/*
 * Determines the aligned length of the WNI MSG including the hdr
 * for a given payload of length 'len'.
 */
static inline unsigned int aniNlLen(unsigned int len)
{
	return  (aniNlAlign(sizeof(tAniHdr)) + len);
}

/* KERNEL DEFS END */

#ifdef CONFIG_ANDROID_LOG
#include <android/log.h>

#define FWDEBUG_LOG_NAME        "ROME"
#define FWDEBUG_NAME            "ROME_DEBUG"
#define android_printf(...) \
    __android_log_print(ANDROID_LOG_INFO, FWDEBUG_LOG_NAME, __VA_ARGS__);
#else
#define android_printf printf
#endif

#define WLAN_NL_MSG_CNSS_DIAG   27 /* Msg type between user space/wlan driver */
#define WLAN_NL_MSG_CNSS_HOST_MSG    28
#define WLAN_NL_MSG_CNSS_HOST_EVENT_LOG    17
#define CNSS_WLAN_DIAG          0x07
#define CNSS_WLAN_SSR_TYPE      0x01
#define CNSS_WLAN_LEVEL_TYPE    0x02
/* NL messgage Carries actual Logs from Driver */
#define ANI_NL_MSG_LOG_HOST_MSG_TYPE 89
#define ANI_NL_MSG_LOG_HOST_EVENT_LOG_TYPE 0x5050
/* NL message Registration Req/Response to and from Driver */
#define ANI_NL_MSG_LOG_REG_TYPE  0x0001
#define MAX_MSG_SIZE 50000
/* PKT SIZE for intf */
#define MAX_PKT_SIZE 8192

/* SPECIAL DIAG HANDLING */
#define DIAG_WLAN_MODULE_STA_PWRSAVE  34292
#define DIAG_WLAN_MODULE_WAL          42996
#define DIAG_NAN_MODULE_ID            56820
#define DIAG_WLAN_MODULE_IBSS_PWRSAVE 57332

#define RESTART_LEVEL     \
    "echo related > /sys/bus/msm_subsys/devices/subsys%d/restart_level"
#define DB_FILE_PATH        "Data.msc"
#define BUF_SIZ  256
#define MAX_TIME_LEN 32

#define WLAN_LOG_TO_DIAG(xx_ss_id, xx_ss_mask, xx_fmt) \
do { \
	if (xx_ss_mask & (MSG_BUILD_MASK_ ## xx_ss_id)) { \
		msg_const_type xx_msg = { \
		{__LINE__, (xx_ss_id), (xx_ss_mask)}, (NULL), msg_file}; \
		xx_msg.fmt = xx_fmt; \
		msg_send (&xx_msg); \
	} \
} while  (0); \

#ifndef ARRAY_LENGTH
#define ARRAY_LENGTH(a)         (sizeof(a) / sizeof((a)[0]))
#endif


int32_t parser_init();

int32_t
dbglog_parse_debug_logs(u_int8_t *datap, u_int16_t len, u_int16_t dropped);

void
diag_initialize();

void
process_diaghost_msg(uint8_t *datap, uint16_t len);

uint32_t
process_diagfw_msg(uint8_t *datap, uint16_t len, FILE *log_out);

int
diag_msg_handler(uint32_t id, char *payload,  uint16_t vdevid, uint64_t timestamp);

void miliseconds_to_time(uint64_t ts, char *time);

uint32_t parse_diagfw_msg(uint8_t *datap, uint32_t len, FILE *log_out,
			  uint16_t parse_flag);

#endif

