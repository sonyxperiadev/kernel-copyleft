/* SPDX-License-Identifier: GPL-2.0
 *
 * LXS touch raw-data debugging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*
 * Copyright 2024 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __LXS_TS_HAL_ABT_H
#define __LXS_TS_HAL_ABT_H

#include "lxs_ts.h"

#if defined(__LXS_SUPPORT_ABT)
#include <linux/signal.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/socket.h>
#include <linux/net.h>
#include <net/sock.h>
#include <net/tcp.h>

#define ABT_TAG			"abt: "
#define ABT_TAG_ERR		"abt(E): "
#define ABT_TAG_WARN	"abt(W): "
#define ABT_TAG_DBG		"abt(D): "

#define t_abt_info(_abt, fmt, args...)	\
		__t_dev_info(_abt->dev, ABT_TAG "[%d(%s)]: " fmt,	\
			_abt->abt_conn_tool, abt_conn_name(_abt->abt_conn_tool),	\
			##args)

#define t_abt_err(_abt, fmt, args...)	\
		__t_dev_err(_abt->dev, ABT_TAG_ERR "[%d(%s)] : " fmt,	\
			_abt->abt_conn_tool, abt_conn_name(_abt->abt_conn_tool),	\
			##args)

#define t_abt_warn(_abt, fmt, args...)	\
		__t_dev_warn(_abt->dev, ABT_TAG_WARN "[%d(%s)] : " fmt,	\
			_abt->abt_conn_tool, abt_conn_name(_abt->abt_conn_tool),	\
			##args)

#define t_abt_dbg(condition, _abt, fmt, args...)	\
		do {							\
			if (unlikely(t_abt_dbg_mask & (condition))) \
				__t_dev_info(_abt->dev, ABT_TAG_DBG "[%d(%s)] : " fmt,	\
					_abt->abt_conn_tool, abt_conn_name(_abt->abt_conn_tool), ##args);	\
		} while (0)

#define t_abt_dbg_base(_abt, fmt, args...)	\
		t_abt_dbg(DBG_BASE, _abt, fmt, ##args)

#define t_abt_dbg_trace(_abt, fmt, args...)	\
		t_abt_dbg(DBG_TRACE, _abt, fmt, ##args)

#define ABT_MAX_RW_SZ_POW	10
#define ABT_MAX_RW_SZ		(1<<ABT_MAX_RW_SZ_POW)

#define DEFAULT_IP		"127.0.0.1"
#define NONE_IP			"0.0.0.0"

#define DEFAULT_PORT	8095
#define TS_TCP_PORT		8097
#define SEND_PORT		8090
#define OMK_BUF			1000

#define ABT_COMM_SEND_DATA_SZ		(4<<10)

/* length limit for u16 size < (64<<10) */
#define ABT_COMM_PACKET_FRAME_MAX	(32<<10)

#define ABT_COMM_NAME_SZ		128
#define ABT_SEND_IP_SIZE		16

#define ABT_SOCK_TIME_OUT		10

#if defined(__DBG_ABT_DATA)
enum E_DATA_TYPE {
	DATA_TYPE_RAW = 0,
	DATA_TYPE_BASELINE,
	/* */
	DATA_TYPE_RN_ORG = 10,
	DATA_TYPE_SEG1 = 20,
	DATA_TYPE_SEG2,
	DATA_TYPE_MAX
};
#endif

enum {
	ABT_DEBUG_DATA = 0,
	ABT_DEBUG_MODE,
	ABT_DEBUG_DATA_RW_MODE,
	ABT_DEBUG_DATA_CAPTURE_MODE,
	ABT_DEBUG_DATA_CMD_MODE,
};

#if defined(__DBG_ABT_REPORT)
enum RW_TYPE {
	READ_TYPE = 55,
	WRITE_TYPE = 66,
};

enum DEBUG_REPORT_CMD {
	DEBUG_REPORT_POINT = 0x100,
	DEBUG_REPORT_OCD,
};
#endif

enum ABT_CONNECT_TOOL {
	ABT_CONN_NOTHING = 0,
	ABT_CONN_TOUCH,
	ABT_CONN_MAX
};

static const char * const __abt_conn_name_str[] = {
	[ABT_CONN_NOTHING]	= "CONN_NOTHING",
	[ABT_CONN_TOUCH]	= "CONN_TOUCH",
};

static inline const char *abt_conn_name(int idx)
{
	return (idx < ABT_CONN_MAX) ? __abt_conn_name_str[idx] : "(invalid)";
}

enum {
	ABT_RUNNING_OFF = 0,
	ABT_RUNNING_ON,
	ABT_RUNNING_EXIT,
};

/* TCP */
enum {
	TCP_REG_READ = 0x80,
	TCP_REG_WRITE,
	TCP_FRAME_START,
	TCP_REPORT_START,
	TCP_SYNC_START,
	TCP_SYNCDEBUG_START,
	TCP_CAPTURE_STOP,
	TCP_CONNECT_CMD,
	TCP_DISCONNECT_CMD
};

enum {
	COMM_RES_SUCCESS	= 0,
	COMM_RES_WR_FAILED	= 0x8001,
};

enum {
	ABT_SOCK_USE_NONE	= 0,
	ABT_SOCK_USE_CONN	= BIT(0),
	ABT_SOCK_USE_LISTEN	= BIT(1),
	ABT_SOCK_USE_BIND	= BIT(2),
	/* */
	ABT_SOCK_TOOL_OPT	= ABT_SOCK_USE_CONN,
};

enum {
	ABT_SOCK_STS_NONE	= 0,
	ABT_SOCK_STS_INIT	= 1,
	ABT_SOCK_STS_RUN	= 2,
	ABT_SOCK_STS_NG		= 9,
	ABT_SOCK_STS_MAX,
};

static const char * const __abt_sock_status_strs[] = {
	[ABT_SOCK_STS_NONE]	= "NONE",
	[ABT_SOCK_STS_INIT]	= "INIT",
	[ABT_SOCK_STS_RUN]	= "RUN",
	[ABT_SOCK_STS_NG]	= "NG",
};

static inline const char *abt_sock_status_str(int status)
{
	const char *str = (status < ABT_SOCK_STS_MAX) ?
						__abt_sock_status_strs[status] : NULL;

	return (str) ? str : "(invalid)";
}

#pragma pack(push, 1)
struct abt_send_data {
	u8 type;
	u8 mode;
	u8 flag;
	u8 touchCnt;
	u32 timestamp;
	u32 frame_num;
	u8 data[ABT_COMM_SEND_DATA_SZ];
} __packed;

struct abt_packet_hdr {
	u8       cmd;
	u16      addr;
	u16      size;
} __packed;

struct abt_comm_packet {
	struct abt_packet_hdr hdr;
	union {
		u32		value;
		u8		frame[ABT_COMM_PACKET_FRAME_MAX];
	} data;
} __packed;
#pragma pack(pop)

typedef int (*abt_sock_listener_t)(void *data, u8 *buf, u32 len);

struct abt_comm {
//	struct device *dev;

//	char name[ABT_COMM_NAME_SZ];

	struct task_struct *thread;

	int family;
	int type;
	int protocol;

	atomic_t running;

	struct socket *sock;
	struct sockaddr_in addr;

	struct socket *ts_sock;
	struct sockaddr_in ts_addr;

	struct socket *sock_send;
	struct sockaddr_in addr_send;

	struct socket *curr_sock;
	struct sockaddr_in *curr_addr;

	struct abt_send_data *data_send;

	struct abt_comm_packet *recv_packet;
	struct abt_comm_packet *send_packet;

	int send_connected;
	char send_ip[ABT_SEND_IP_SIZE+4];

	abt_sock_listener_t	sock_listener;
};

struct abt_data {
	void *ts;
	struct device *dev;
	/* */
	int dbg_mask;
	/* */
	struct mutex comm_lock;
	int close;

	struct abt_comm comm;

	long sock_time_out;

	u32 port_default;
	u32 port_tcp;
	u32 port_send;
	u32 port_buf_omk;

	u16 frame_num;
	int abt_report_mode;
	int abt_report_point;
	int abt_conn_tool;

	int client_connected;
	int client_connect_trying;

	u32 connect_error_count;

	atomic_t socket_status;

	int sysfs_done;

	void (*report_handler)(struct lxs_ts *ts, int release_all);
};

#define TS_ATTR_ABT(_name, _show, _store)	\
		TS_ATTR(_name, _show, _store)

#endif	/* __LXS_SUPPORT_ABT */

#endif /* __LXS_TS_HAL_ABT_H */


