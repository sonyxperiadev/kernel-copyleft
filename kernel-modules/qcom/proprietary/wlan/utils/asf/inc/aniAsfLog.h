/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file log.h is for the Log Manager (Function declarations)
 * Author:  U. Loganathan
 * Date:    May 16th 2002
 * History:-
 * Date     Modified by      Modification Information
 * 10/1/02  Mayank Upadhyay  Added formats to print function name
 * 11/12/02  Andy Chinmulgund Removed aniSyslog.h header include
 */

#ifndef _ANI_ASF_LOG_H_
#define _ANI_ASF_LOG_H_

#include <stdio.h>
#include <errno.h>
#include <syslog.h>
#include <string.h>

// Use this if you want to log fileName:lineNum
#define ANI_LOG_FILE        __FILE__, __LINE__, NULL
#define ANI_WHERE           ANI_LOG_FILE

// Use this if you want to log appName.funcName
#define ANI_LOG_FUNC        NULL, 0,  __FUNCTION__

// Use this if you don't want any extra headers
#define ANI_LOG_PLAIN       NULL, 0, NULL

#ifdef ANI_DEBUG
#define ANI_CONS_LOG        1
#else /* ANI_DEBUG */
#define ANI_CONS_LOG        0
#endif /* ANI_DEBUG */

// Log Manager Messages
#define ANI_LOG_INIT        LOG_NOTICE, ANI_WHERE, "Started %s with Pid %d"
#define ANI_LOG_CLOSE       LOG_NOTICE, ANI_WHERE, "Stopped task with pid %d"

// General Messages
#define ANI_MUTEX_ERR       LOG_ERR, ANI_WHERE, "Error(%d):Mutex error", errno
#define ANI_COND_ERR       LOG_ERR, ANI_WHERE, "Error(%d):Condition error", errno
#define ANI_MALLOC_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Malloc error", errno
#define ANI_FOPEN_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Fopen error", errno
#define ANI_IOCTL_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Ioctl error", errno
#define ANI_SEM_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Semaphore error", errno

// Timer Manager Messages
#define ANI_NULL_TIMER      	LOG_ERR, ANI_WHERE, "Null Timer"
#define ANI_MAX_TMRS      	LOG_ERR, ANI_WHERE, "Max Timers (%d)"
#define ANI_NULL_DURATION       LOG_ERR, ANI_WHERE, "Null Duration"
#define ANI_TIMER_NOT_RUNNING   LOG_INFO, ANI_WHERE, "Timer is not running"

// IPC Manager Messages
#define ANI_SOCKET_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Socket open failed, ipcPtr [0x%x]", errno
#define ANI_BIND_ERR        LOG_ERR, ANI_WHERE, "Error(%d):Bind failed, ipcPtr [0x%x], sd %d", errno
#define ANI_SETSOCK_ERR     LOG_ERR, ANI_WHERE, "Error(%d):Setsockopt for %s failed, ipcPtr [0x%x], sd %d", errno
#define ANI_GETPEERNAME_ERR     LOG_ERR, ANI_WHERE, "Error(%d):Getpeername failed, ipcPtr [0x%x], sd %d", errno
#define ANI_GETSOCKNAME_ERR     LOG_ERR, ANI_WHERE, "Error(%d):Getsockname failed, ipcPtr [0x%x], sd %d", errno
#define ANI_LISTEN_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Listen failed, ipcPtr [0x%x], sd %d", errno
#define ANI_CONNECT_ERR     LOG_ERR, ANI_WHERE, "Error(%d):Connect failed, ipcPtr [0x%x], sd %d", errno
#define ANI_ACCEPT_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Accept failed, ipcPtr [0x%x], sd %d", errno
#define ANI_SELECT_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Select failed", errno
#define ANI_SENDTO_ERR      LOG_ERR, ANI_WHERE, "Error(%d):Sendto failed, ipcPtr [0x%x], sd %d", errno
#define ANI_RECVFROM_ERR        LOG_ERR, ANI_WHERE, "Error(%d):RecvFrom failed, ipcPtr [0x%x], sd %d", errno
#define ANI_DUP_ERR        LOG_ERR, ANI_WHERE, "Error(%d):Dup failed, ipcPtr [0x%x], sd %d", errno
#define ANI_WRITE_ERR       LOG_ERR, ANI_WHERE, "Error(%d):Write failed, ipcPtr [0x%x], sd %d", errno
#define ANI_RECV_ERR        LOG_ERR, ANI_WHERE, "Error(%d):Recv failed, ipcPtr [0x%x], sd %d", errno
#define ANI_READ_ERR        LOG_ERR, ANI_WHERE, "Error(%d):Read failed, ipcPtr [0x%x], sd %d", errno
#define ANI_GETHOSTBYNAME_ERR   LOG_ERR, ANI_WHERE, "Error(%d):Gethostbyname failed, ipcPtr [0x%x], sd %d"
#define ANI_GETHOSTTYPE_ERR     LOG_ERR, ANI_WHERE, "Gethostbyname invalid type, ipcPtr [0x%x], sd %d"

#define ANI_PMSET_ERR       LOG_ERR, ANI_WHERE, "Port Map Set failed"
#define ANI_PMGET_ERR       LOG_ERR, ANI_WHERE, "Port Map Get failed"

// Port Map Library & Some General Messages
#define ANI_IPCOPEN_ERR     LOG_ERR, ANI_WHERE, "IPC Open failed"
#define ANI_IPCCONNECT_ERR      LOG_ERR, ANI_WHERE, "IPC Connect failed"
#define ANI_IPCACCEPT_ERR       LOG_ERR, ANI_WHERE, "IPC Accept failed"
#define ANI_IPCCHECK_ERR        LOG_ERR, ANI_WHERE, "IPC Reply timeout"
#define ANI_IPCSEND_ERR     LOG_ERR, ANI_WHERE, "IPC Send failed"
#define ANI_IPCRECV_ERR     LOG_ERR, ANI_WHERE, "IPC Recv failed"
#define ANI_IPCLISTEN_ERR       LOG_ERR, ANI_WHERE, "IPC Listen failed"

extern void aniAsfLogInit(const char *, int, int);
extern void aniAsfLogConsole(int);
extern void aniAsfLogClose(void);
extern void aniAsfLogSetLevel(int);
#ifndef ANI_ENTRY_LEVEL_AP
extern int aniAsfLogMsg(int, const char *, long, const char *, const char *, ...);
#else
#define aniAsfLogMsg(...)
#endif /* ANI_ENTRY_LEVEL_AP */
extern void aniAsfAddFacilityName(char *);
extern void aniAsfRemoveFacilityName(char *);
extern void aniAsfLogDinit(char *);
extern int aniFacilityEnabled(int);

#endif /* _ANI_ASF_LOG_H_ */
