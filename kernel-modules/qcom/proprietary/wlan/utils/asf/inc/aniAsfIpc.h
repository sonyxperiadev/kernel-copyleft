/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file aniAsfIpc.h is for the Inter-Process Communications Manager (Function declarations)
 * Author:  U. Loganathan
 * Date:    May 16th 2002
 * History:-
 * Date     Modified by Modification Information
 *
 */

#ifndef _ANI_ASF_IPC_H_
#define _ANI_ASF_IPC_H_

#define ANI_MAX_BUF_SIZE        1514

#define ANI_RCV_SOC_BUF_SIZE	16 * 1024
#define ANI_SND_SOC_BUF_SIZE	16 * 1024

#define ANI_DYNAMIC_PORT        0

#define ANI_NO_PORTMAP      	-1

#define AF_INET_EXT		AF_MAX

/*
 * Structure to handle Inter 
 * process communcation
 */
typedef struct ipc tAniIpc;

extern void aniAsfIpcInit(void);
extern int aniAsfIpcSize(void);
extern tAniIpc *aniAsfIpcOpen(int, int, int);
extern tAniIpc *aniAsfIpcOpenWithLocalAddr(int af, int type, int port, char *localAddr);
extern void aniAsfIpcClose(tAniIpc *);
#ifdef ASF_PORT_MAP_SUPPORTED
extern int aniAsfIpcListen(tAniIpc *, int, int);
#else
extern int aniAsfIpcListen(tAniIpc *, int);
#endif
extern tAniIpc *aniAsfIpcAccept(tAniIpc *);
extern int aniAsfIpcConnect(tAniIpc *, char *, int, int);
extern int aniAsfIpcRecv(tAniIpc *, char *, int);
extern int aniAsfIpcSend(tAniIpc *, char *, int);
extern void aniAsfIpcSetFd(tAniIpc *, void (*)(void *), void *);
extern void aniAsfIpcUnSetFd(tAniIpc *);
extern int aniAsfIpcCheck(tAniIpc *, long);
extern void aniAsfIpcDebug(tAniIpc *);
extern void aniAsfIpcBlockSelect(void);
extern int aniAsfIpcProcess(void);
extern struct sockaddr_nl *aniAsfIpcGetSnl(tAniIpc *);
extern int aniAsfIpcGetSock(tAniIpc *);
extern int aniAsfIpcGetAniMsgType(tAniIpc *);
extern int aniAsfIpcGetAniMsgLen(tAniIpc *);

extern int aniAsfIpcSetSockOpt(tAniIpc *ipc, int, int, void *, int);

#endif /* _ANI_ASF_IPC_H_ */
