/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */




/******************************************************************************
 * aniRtt.h
 *
 * This file contains preprocessor defintion and other datatype definitions
 * private to aniRtt
 *
 * Krishna Reddy, 09/05/2003
 *
 ******************************************************************************/

#ifndef _ANI_RTT_H_
#define _ANI_RTT_H_

#define RTT_MAX_MSG_SIZE	50000
#define USER_SPACE_DATA         8192

#define RTT_SERVER_PORT 1528
#define eANI_NIM_CRDT_SYSDBG_LOG_DUMP_RSP 0x2108
#define FEATURE_LOG_EXPOSED_HEADER
#define UNUSED(x)      (void)(x)

typedef enum {
	RTT_SME_MSG = 1,		// eANIAPI_AUTOTEST_ID_SME_MSG
	RTT_DRV_DISABLE = 7,	// eANIAPI_AUTOTEST_ID_DISABLE_ADAPTER
	RTT_DRV_ENABLE = 8,		// eANIAPI_AUTOTEST_ID_ENABLE_ADAPTER
	RTT_CFG_SET_REQ = 15,	// eANIAPI_AUTOTEST_ID_CFG_SET_REQ
} tRttMsgIds;

typedef enum {
	RTT_RSP_SME_MSG = 201,
	RTT_RSP_CFG_MSG = 202,
	RTT_RSP_HDD_MSG = 203,
	RTT_RSP_ALREADY_REGISTERED_MSG = 204,
	RTT_REBOOT = 0xffff,
} tRttRspMsgIds;

typedef struct sAniRttCmdRspMsg {
	int	msgLen;
	int	radio;
	ANI_U16	msgType;
} tAniRttCmdRspMsg;

#define PTT_WLAN_LOG_REGISTER       0x01
#define PTT_WLAN_LOG_MSG            0x59
#define PTT_WLAN_LOG_READY_IND_MSG  0x5A
#define PTT_FTM_CMDS_TYPE           0x4040
#define PTT_DIAG_CMDS_TYPE          0x5050
#define PTT_DIAG_TYPE_LOGS   1 
#define PTT_DIAG_TYPE_EVENTS 2

typedef struct sAniDiagMsg {
	int msg_len;
	char *pRespData;
	tAniBoolean diag_msg_received;
}tAniDiagMsg;

typedef struct sAniRttServerContext {
	int	radio;
	tAniIpc    *ipcs;	/* IPC struct for on which the server listens */
	tAniIpc    *ipcnl ;	/* IPC struct the Netlink socket that the
				 * server uses to pass messages back and
				 * forth from the Pseudo Driver kernel
				 * module.
				 */
	tAniIpc    *clIpc;	/* The accepted socket to the client */
	struct nlmsghdr nl;	/* A prebuilt and cached Netlink msg hdr */
	struct sockaddr_nl *snl;/* return from getsockname in aniAsfIpcOpen */
	tAniDiagMsg diag_msg;
} tAniRttCtxt ;

/* Static in-line type conversion functions */

typedef union u32ValAry{
        ANI_U32 val;
        char ary[sizeof(ANI_U32)];
} t32ValAry;

typedef union u16ValAry{
        ANI_U16 val;
        char ary[sizeof(ANI_U16)];
} t16ValAry;

static __inline__ ANI_U32 pttSocketRd32(char *cp)
{
        t32ValAry r;
        int i;

        i = sizeof(ANI_U32) - 1;
        r.ary[i--] = cp[3];
        r.ary[i--] = cp[2];
        r.ary[i--] = cp[1];
        r.ary[i] = cp[0];

        return r.val;
}

static __inline__ ANI_U16 pttSocketRd16(char *cp)
{
        t16ValAry  r;
        int i;

        i = sizeof(ANI_U16) - 1;
        r.ary[i--] = cp[1];
        r.ary[i] = cp[0];

        return r.val;
}

static inline ANI_U32 pttSocketSwapU32(ANI_U32 val)
{
    return((val << 24) |
           (val >> 24) |
           ((val & 0x0000FF00) << 8) |
           ((val & 0x00FF0000) >> 8));
}

static inline ANI_U16 pttSocketSwapU16(ANI_U16 val)
{
    return(((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8));
}

#endif // _ANI_RTT_H_
