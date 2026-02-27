/*
 * Copyright (c) 2013-2014 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/******************************************************************************
 * aniNlMsg.h
 *
 * This file contains generic macros to manipulate the WNI Netlink messages
 * exhanged between the WNI Applications and the WNI Kernel modules.
 *
 * Krishna Reddy, 05/20/2002
 *
 ******************************************************************************/

#ifndef _ANI_NL_MSG_H_
#define _ANI_NL_MSG_H_

#include <asm/types.h>
#ifndef __KERNEL__
#include <sys/socket.h>
#include <netinet/in.h>
#endif // __KERNEL__
#ifdef ANI_KERNEL_2_6
#define _LINUX_TYPES_H
#define __KERNEL_STRICT_NAMES
#endif
#include <linux/netlink.h>

#include "aniAsfHdr.h"

#define ANI_BRIDGE_NAME "br0"

#define ANI_RADIO_0     0
#define ANI_RADIO_1     1
#define ANI_RADIO_2     2

#define ANI_MAX_RADIOS	    3
#define ANI_MAX_ETHERNETS   2

#define ANI_NL_MSG_OK       0
#define ANI_NL_MSG_ERROR    -1

#define ANI_NL_MSG_OVERHEAD (NLMSG_SPACE(tAniHdr + 4))

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

/*
 * This msg hdr will always follow tAniHdr in all the messages exchanged
 * between the Applications in userspace the Pseudo Driver, in either 
 * direction.
 */
typedef struct sAniNlMsg {
    struct  nlmsghdr nlh;	// Netlink Header
    int radio;			// unit number of the radio 
    tAniHdr wmsg;		// Airgo Message Header
} tAniNlHdr;

typedef struct sAniAppRegReq {
    tAniNlModTypes type;	/* The module id that the application is 
					registering for */
    int pid;			/* Pid returned in the nl_sockaddr structure 
					in the call getsockbyname after the 
					application opens and binds a netlink 
					socket */
} tAniNlAppRegReq;

typedef struct sAniNlAppRegRsp {
    struct nlmsghdr nlh;	// NetLink Msg Header
    int radio;			// Radio unit associated with the message
    tAniHdr wniHdr;		// Generic WNI msg header
    tAniNlAppRegReq regReq;	// The original request msg
    int ret;			// Return code
} tAniNlAppRegRsp;

typedef enum eAniRegAccessCmd {
    ANI_MAC_REG_READ = 1,
    ANI_MAC_REG_WRITE,
} tAniRegAccCmd;

typedef struct sAniRegAccessMsg {
    unsigned int    addr;
    unsigned int    len;
    tAniRegAccCmd   cmd;
    unsigned char   data[16];
} tAniMacRegAccMsg;


#endif  /* _ANI_NL_MSG_H_ */
