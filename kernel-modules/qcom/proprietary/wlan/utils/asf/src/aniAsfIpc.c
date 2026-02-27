/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */




/*
 * This file ipc.c has the Inter-process communication Manager library code
 * Author:	U. Loganathan
 * Date:	May 16th 2002
 * History:-
 * Date		Modified by	Modification Information
 *
 */

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <asm/types.h>
#ifdef ANI_KERNEL_2_6
#define _LINUX_TYPES_H
#define __KERNEL_STRICT_NAMES
#endif
#include <linux/netlink.h>
#ifdef ANI_KERNEL_2_6
typedef __u16 __le16;
typedef __u16 __be16;
typedef __u32  __le32;
typedef __u32  __be32;
typedef __u16  __sum16;
typedef __u32  __wsum;
#endif
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <sys/times.h>
#include <pthread.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "aniTypes.h"
#include "aniAsfMem.h"
#include "aniAsfIpc.h"
#include "aniAsfLog.h"
#include "aniAsfHdr.h"
#ifdef ASF_PORT_MAP_SUPPORTED
#include "aniAsfPortMap.h"
#endif


#define ANI_ETH_P_EAPOL 0x888E

/*
 * Internal structure to handle
 * socket call backs
 */
typedef struct scb {

	// Socket descriptor
	int		sd;

	// Function to call if data available on socket
	void		(*func)(void *);

	// Arguments to be passed for call-back function
	void		*args;

	// Linked-list next pointer
	struct scb	*next;

}tScb;

/*
 * Structure to handle Inter 
 * process communcation
 */
struct ipc {

	// Socket descriptor
	int			sd;

	// Communication domain
	// Values currently supported are
	// AF_INET_EXT (Proprietary, For External Address), 
	// AF_INET (Only works for LoopBack Address)
	// AF_NETLINK, AF_PACKET
	int			domain;

	// Semantics of communication
	int			type;

	// Program Id
	int			prog;

	// Program Version Id
	int			vers;

	// Message Type (for UDP)
	short			msgType;

	// Message Length (for UDP)
	short			msgLen;

	// Holds the local address for sd
	union {
		struct sockaddr_in 	sin;
		struct sockaddr_nl	snl;
		struct sockaddr_ll	sll;
	}local;

	// Holds the remote address for sd
	union {
		struct sockaddr_in 	sin;
		struct sockaddr_nl	snl;
		struct sockaddr_ll	sll;
	}remote;

	// Hold the interface for Raw sockets
	int			interface;
	// Hold the call back info
	tScb 			scb;

	// Whether or not a callback function is set
	tAniBoolean callbackFlag;

};

static void scbAdd(tScb *scb);
static void scbRemove(tScb *scb);

// Linked list to hold the socket call back's
static tScb		*scblist = NULL;

// read file descriptor set for select 
static fd_set 		readfds;

// For multithread Mutex Lock
static pthread_mutex_t 	scbmutex;

// Flag to hold if ipc has been inited
static int 		ipcinited = 0;

// Flag to make select a blocking call
static int 		blocked = 0;


/**
 * aniAsfIpcInit
 *
 * FUNCTION:
 * 	Initialize function for IPC
 *
 * LOGIC:
 * 	Initialize the pthread mutex for locking critical sections
 * 	Zeroes out the global variable readfds
 *
 * @param None
 *
 * @param return - None
 *
 */

void aniAsfIpcInit()
{
	// Init the pthread mutex
	// mutex init always returns 0
	pthread_mutex_init(&scbmutex, NULL);

	// zero out the fd_set
	FD_ZERO(&readfds);

	// Flag for select block
	blocked = 0;

	// Flag that the ipc has been inited
	ipcinited = 1;
}

/**
 * aniAsfIpcSetSocks
 *
 * FUNCTION:
 * 	Initialize socket options
 *
 * LOGIC:
 * 	Initialize the all the socket setsock options
 *
 * @param *ipc - Ipc Pointer
 *
 * @param return - 0 on success and -1 on error
 *
 */

int aniAsfIpcSetSocks(tAniIpc *ipc)
{
	int val, len;
	struct linger ling;
	int nbio = 1;

	len = sizeof(int);

	// sets the maximum send buffer size to 16K for normal sockets
	// sets the maximum send buffer size to 128K for netlink sockets
	val = ANI_SND_SOC_BUF_SIZE;
	if (ipc->domain == AF_NETLINK) {
		val *= 8;
	}
	if (setsockopt(ipc->sd, SOL_SOCKET, SO_SNDBUF, 
			(char *)&val, len) < 0) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "SNDBUF",
				ipc, ipc->sd);
		return(-1);
	}
    
	// sets the maximum receive buffer size to 16K
	val = ANI_RCV_SOC_BUF_SIZE;
	if (setsockopt(ipc->sd, SOL_SOCKET, SO_RCVBUF, 
			(char *)&val, len) < 0) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "RCVBUF",
				ipc, ipc->sd);
		return(-1);
	}
    
	// allows bind to reuse local addresses
	val = 1;
	if (setsockopt(ipc->sd, SOL_SOCKET, SO_REUSEADDR, 
			(char *)&val, len) < 0) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "REUSEADR",
				ipc, ipc->sd);
		return(-1);
	}
    
	// Datagram sockets can send and receive packets using
	// broadcast addresses
	val = 1;
	if (setsockopt(ipc->sd, SOL_SOCKET, SO_BROADCAST, 
			(char *)&val, len) < 0) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "BROADCAST",
				ipc, ipc->sd);
		return(-1);
	}
    
	// setting the linger off makes the closing of
	// sockets done in the backgroud.
	ling.l_onoff = 0; 	// linger off
	ling.l_linger = 0;	// 0 seconds to linger for
	if (setsockopt(ipc->sd, SOL_SOCKET, SO_LINGER, 
			(char *)&ling, sizeof(ling)) < 0) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "LINGER",
				ipc, ipc->sd);
		return(-1);
	}

	// Make this socket non blocking
	if (ioctl(ipc->sd, FIONBIO, &nbio)) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "FIONBIO",
				ipc, ipc->sd);
		return(-1);
	}

	return 0;
}

int aniAsfIpcSetSockOpt(tAniIpc *ipc, int level, 
                        int optname, void *optval, int optlen)
{
    if (setsockopt(ipc->sd, level, optname, optval, optlen) < 0) {
        aniAsfLogMsg(ANI_SETSOCK_ERR, "SOCK_OPTS", ipc, ipc->sd, level, optname);
        return(-1);
    }

    return 0;
}


/**
 * aniAsfIpcOpenWithLocalAddr
 *
 * FUNCTION:
 * 	Function to open a inter process communications channel on the
 *  specified local address.
 *
 * LOGIC:
 * 	Will be used by both the Client and server
 * 	Initialize the ipc structure fields
 * 	Open a socket based on socket family and type
 * 	Set socket options 
 * 		Send Buffer size to 1024 * 16
 * 		Receive Buffer size to 1024 * 16
 * 		Re-use socket address
 * 		Support  Broadcast
 * 		Disable socket linger
 * 	Bind this socket to the local endpoint
 * 		Static port is used if a port parameter value > 0 or
 * 		Dynamic port assignment is used
 *      For INET protocol, the socket is bound to the specified local
 *      address or INADDR_ANY, if localAddr is NULL
 * 	Give a name to this socket
 *
 * @param af - socket domain
 * @param type - socket type
 * @param port - port to bind.  If zero is passed a dynamic port is used
 * @param localAddr - local address to bind to. If NULL is passed then
 * INADDR_ANY is used.
 *
 * @param return - NULL is returned if an error occurs or the New Ipc pointer 
 * 			of the open socket
 *
 */
tAniIpc *aniAsfIpcOpenWithLocalAddr(int af, int type, int port, char *localAddr)
{
	socklen_t len;
	tAniIpc *ipc;
	int proto;

	// allocate memory return NULL on error
	if ((ipc = (tAniIpc *)aniMalloc(sizeof(struct ipc))) == NULL) {
		aniAsfLogMsg(ANI_MALLOC_ERR);
		return(NULL);
	}

	// Initialize the ipc structure
	ipc->domain = af;
	ipc->type = type;
	ipc->prog = 0;
	ipc->vers = -1; // Don't register
	ipc->msgType = -1;
	ipc->msgLen = -1;

	switch (af) {
	case AF_NETLINK: /* For Netlink */
		proto = NETLINK_USERSOCK;
		break;
	case AF_PACKET: /* For Raw Socket */
	       // proto = htons(ANI_ETH_P_EAPOL); // Doesn't work on all distributions of Linux
	        proto = htons(ETH_P_ALL);
	        break;
	case AF_INET_EXT: /* Used for external ip Address */
		ipc->domain = AF_INET;
		// flow thru
	default:
		proto = 0;
		break;
	}

	// Open a socket, creates an endpoint for communication
	if ((ipc->sd = socket(ipc->domain, type, proto)) < 0) {
		aniAsfLogMsg(ANI_SOCKET_ERR, ipc);
		aniFree(ipc);
		return(NULL);
	}

	// Set all the default sock options
	if (aniAsfIpcSetSocks(ipc) < 0) {
		aniAsfIpcClose(ipc);
		return(NULL);
	}
    
	// prepare to bind
	switch (af) {
	case AF_NETLINK: /* For Netlink sockets */
		len = sizeof(struct sockaddr_nl);
		memset((void *)&ipc->local.snl, 0, len);
		ipc->local.snl.nl_family = af; /* Domain */
		ipc->local.snl.nl_groups = port; /* Subscriptions */
		break;
	case AF_PACKET: /* For Raw Sockets */
		len = sizeof(struct sockaddr_ll);
		memset((void *)&ipc->local.sll, 0, len);
		ipc->local.sll.sll_family = af; /* Domain */
		ipc->local.sll.sll_protocol = proto; /* Protocol */
		ipc->local.sll.sll_ifindex = port; /* interface */
		ipc->interface = port; /* Store the interface for future use */
		break;
	case AF_INET: /* For Inet Sockets */
	default: /* Default */
		len = sizeof(struct sockaddr_in);
		memset((void *)&ipc->local.sin, 0, len);
		/* Make is AF_INET as default */
		ipc->local.sin.sin_family = AF_INET; 
		if (af == AF_INET_EXT) {
            // For INET_EXT communications bind to the specified local address.
            if (localAddr == NULL) {
                ipc->local.sin.sin_addr.s_addr = htonl(INADDR_ANY);
            } else {
                if (inet_aton(localAddr, &ipc->local.sin.sin_addr) == 0) {
                    aniAsfLogMsg(ANI_BIND_ERR, ipc, ipc->sd);
                    aniAsfIpcClose(ipc);
                    return(NULL);
                }
            }
		} else {
			/* Default bind to the LOOPBACK addres */
			ipc->local.sin.sin_addr.s_addr = 
				htonl(INADDR_LOOPBACK);
		}
		ipc->local.sin.sin_port = htons(port); 
		break;
	}

	// Bind assigns a name to the socket. 
	if (bind(ipc->sd, (struct sockaddr *)&ipc->local, len) < 0) {
		aniAsfLogMsg(ANI_BIND_ERR, ipc, ipc->sd);
		aniAsfIpcClose(ipc);
		return(NULL);
	}

	// Get the socket name, address and port number of the socket
	switch (af) {
	case AF_NETLINK:
		len = sizeof(ipc->local.snl);
		break;
	case AF_PACKET:
		len = sizeof(ipc->local.sll);
		break;
	case AF_INET:
	default:
		len = sizeof(ipc->local.sin);
		break;
	}

	if (getsockname(ipc->sd,
			(struct sockaddr *)&ipc->local,
			&len) < 0) {
		aniAsfLogMsg(ANI_GETSOCKNAME_ERR, ipc, ipc->sd);
		aniAsfIpcClose(ipc);
		return(NULL);
	}

	// Initialize the callback flag
	ipc->callbackFlag = eANI_BOOLEAN_FALSE;

	// return the new ipc pointer
	return(ipc);
}

/**
 * aniAsfIpcOpen
 *
 * FUNCTION:
 * 	Function to open a inter process communications channel
 *
 * LOGIC:
 * 	Will be used by both the Client and server
 * 	Initialize the ipc structure fields
 * 	Open a socket based on socket family and type
 * 	Set socket options 
 * 		Send Buffer size to 1024 * 16
 * 		Receive Buffer size to 1024 * 16
 * 		Re-use socket address
 * 		Support  Broadcast
 * 		Disable socket linger
 * 	Bind this socket to a port
 * 		Static port is used if a port parameter value > 0 or
 * 		Dynamic port assignment is used
 *      For INET protocol, the socket is bound to the local address INADDR_ANY
 * 	Give a name to this socket
 *
 * @param af - socket domain
 * @param type - socket type
 * @param port - port to bind.  If zero is passed a dynamic port is used
 *
 * @param return - NULL is returned if an error occurs or the New Ipc pointer 
 * 			of the open socket
 *
 */

tAniIpc *aniAsfIpcOpen(int af, int type, int port)
{
    return aniAsfIpcOpenWithLocalAddr(af, type, port, NULL);
}

/**
 * aniAsfIpcClose
 *
 * FUNCTION:
 * 	Function to close the open communication channel
 *
 * LOGIC:
 * 	Will be used by both the Client and server
 * 	Closes the open socket descriptor
 * 	Cleans out the pmap settings
 * 	Frees the ipc malloc memory
 *
 * @param *ipc - The IPC pointer 
 *
 * @param return - None
 *
 */

void aniAsfIpcClose(tAniIpc *ipc)
{
	if (ipc != NULL) {

		// Unset the callback function if it is still set
		// Do this before closing the socket
		if (ipc->callbackFlag) {
			aniAsfIpcUnSetFd(ipc);
		}

		// Close the socket
		close(ipc->sd);

		// Unset port map entry if already set
		if (ipc->prog && (ipc->vers >= 0)) {
#ifdef ASF_PORT_MAP_SUPPORTED
			int proto = (ipc->type == SOCK_STREAM) ? IPPROTO_TCP : IPPROTO_UDP;
			aniAsfPmUnSet(ipc->prog, ipc->vers, proto);
#else
    aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Port Map not supported");
#endif
		}

		// Free ipc pointer
		aniFree(ipc);
	}
}

/**
 * aniAsfIpcDebug
 *
 * FUNCTION:
 * 	Print the socket details
 *
 * LOGIC:
 * 	Prints the sockets ip address and port numbers
 *
 * @param *ipc - Ipc Pointer
 *
 * @param return - None
 *
 */

void aniAsfIpcDebug(tAniIpc *ipc)
{
	if (ipc) {
		aniAsfLogMsg(LOG_NOTICE, ANI_WHERE,
			"Local - %d@%s Remote %d@%s",
			ntohs(ipc->local.sin.sin_port),
			inet_ntoa(ipc->local.sin.sin_addr), 
			ntohs(ipc->remote.sin.sin_port),
			inet_ntoa(ipc->remote.sin.sin_addr)); 
	}
}

/**
 * aniAsfIpcListen
 *
 * FUNCTION:
 * 	Function used by server to start listening for clients to connect
 *
 * LOGIC:
 * 	Will be used by server
 * 	Server's port number is registered to the port map layer using pmap_set
 * 	If the parameter vers is a negative number then pmap registry is not done
 * 	If the socket is of type SOCK_STREAM it calls the system call listen
 *
 * @param *ipc - The IPC pointer 
 * @param prog - Unique program id
 * @param vers - Programs version id
 *
 * @param return - ? -1 for error and errno will be set with actual error 
 * 				number; 0 on success
 *
 */

#ifdef ASF_PORT_MAP_SUPPORTED
int aniAsfIpcListen(tAniIpc *ipc, int prog, int vers)
#else
int aniAsfIpcListen(tAniIpc *ipc, int vers)
#endif
{
	// If vers is less than 0 don't register with portmap
	if (vers >= 0) {
#ifdef ASF_PORT_MAP_SUPPORTED
		int proto = (ipc->type == SOCK_STREAM) ? IPPROTO_TCP : IPPROTO_UDP;

		if (aniAsfPmSet(prog, vers, proto, ntohs(ipc->local.sin.sin_port)) < 0) {
			aniAsfLogMsg(ANI_PMSET_ERR);
			return(-1);
		}

		// Store the prog id, to be used by IpcClose to unset portmap entry
		ipc->prog = prog;
		ipc->vers = vers;
#else
    aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Port Map not supported");
    return -1;
#endif 
	}

	// Listen for sock_stream only.  backlog of 5 connections
	if (ipc->type == SOCK_STREAM) {
		if (listen(ipc->sd, 5) < 0) {
			aniAsfLogMsg(ANI_LISTEN_ERR, ipc, ipc->sd);
			return(-1);
		}
	}

	return(0);
}

/**
 * aniAsfIpcAccept
 *
 * FUNCTION:
 * 	Server Function to accept a client connection
 *
 * LOGIC:
 * 	For sockets of type SOCK_STREAM. It uses the system call accept
 * 	For sockets of type SOCK_DGRAM it uses the system call recvfrom
 * 	Dup's the socket descriptor for SOCK_DGRAM and returns the same 
 * 		socket descriptor in a new Ipc pointer
 *
 * @param *ipc - The IPC pointer 
 *
 * @param return - NULL is returned if an error occurs; or the New IPC pointer 
 * 		referencing the new socket
 *
 */

tAniIpc *aniAsfIpcAccept(tAniIpc *ipc)
{
	socklen_t len;
	long lbuf;
	char *cbuf = (char *)&lbuf;
	tAniIpc *nipc;

	// Allocate memory, return NULL on error
	if ((nipc = (tAniIpc *)aniMalloc(sizeof(struct ipc))) == NULL) {
		aniAsfLogMsg(ANI_MALLOC_ERR);
		return(NULL);
	}

	switch (ipc->domain) {
	case AF_NETLINK:
		len = sizeof(ipc->local.snl);
		break;
	case AF_PACKET:
		len = sizeof(ipc->local.sll);
		break;
	case AF_INET:
	default:
		len = sizeof(ipc->local.sin);
		break;
	}

	// Initialize values;
	nipc->domain = ipc->domain;
	nipc->type = ipc->type;
	nipc->prog = 0;
	nipc->vers = -1; // Don't register
	nipc->msgType = -1;
	nipc->msgLen = -1;

	// For TCP
	if (ipc->type == SOCK_STREAM) {

		// accepts the new client connection
		if ((nipc->sd = accept(ipc->sd,
				(struct sockaddr *)&nipc->remote, 
				&len)) < 0) {
			aniAsfLogMsg(ANI_ACCEPT_ERR, ipc, ipc->sd);
			aniFree(nipc);
			return(NULL);
		}

		// Set all the default sock options
		if (aniAsfIpcSetSocks(nipc) < 0) {
			close(nipc->sd);
			aniFree(nipc);
			return(NULL);
		}

		if (getpeername(nipc->sd,
				(struct sockaddr *)&nipc->remote, 
				&len) < 0) {
			aniAsfLogMsg(ANI_GETPEERNAME_ERR,
					nipc, nipc->sd);
			close(nipc->sd);
			aniFree(nipc);
			return(NULL);
		}

		if (getsockname(nipc->sd,
				(struct sockaddr *)&nipc->local,
				&len) < 0) {
			aniAsfLogMsg(ANI_GETSOCKNAME_ERR,
					nipc, nipc->sd);
			close(nipc->sd);
			aniFree(nipc);
			return(NULL);
		}

	} else { // For UDP & RAW

		// Tries to read with a MSG_PEEK option.  
		// Just get the connection details
		if (recvfrom(ipc->sd, (char *)&lbuf,
				sizeof(long), MSG_PEEK, 
				(struct sockaddr *)&nipc->remote,
				&len) < 0) {
			aniAsfLogMsg(ANI_RECVFROM_ERR,
					ipc, ipc->sd);
			aniFree(nipc);
			return(NULL);
		}

		// Copy the local from ipc to nipc
		memcpy(&nipc->local, &ipc->local, len);

		// First two bytes (Message Type)
		nipc->msgType = aniAsfGet16(cbuf);
		cbuf += 2;

		// Next two bytes (Message Length)
		nipc->msgLen = aniAsfGet16(cbuf);

		// Dup's the current socket just to mimic TCP
		if ((nipc->sd = dup(ipc->sd)) < 0) {
			aniAsfLogMsg(ANI_DUP_ERR, ipc, ipc->sd);
			aniFree(nipc);
			return(NULL);
		}

	}

	// Return the new ipc pointer
	return(nipc);
}

/**
 * aniAsfIpcConnect
 *
 * FUNCTION:
 * 	Client Function to connect to the server
 *
 * LOGIC:
 * 	Uses the gethostbyname to translate host name to host entry
 * 	For connections within the same host "localhost" can be used
 * 	If the version is a negative number than server port is used as is
 * 	If the version is positive, it uses pmap_getport to find the 
 * 			server's port number
 * 	Uses the remote servers ip address and port number to connect
 *
 * @param *ipc - The IPC pointer 
 * @param *host - Servers host name, "localhost" if the server is running 
 * 			on the same host as client
 * @param prog - Server program's unique id  or the actual servers 
 * 			port number if vers parameter is -1
 * @param vers - Server program's version number or -1 if the parameter 
 * 			prog passes the actual port number
 *
 * @param return - -1 for error and errno will be set with actual error number;
 * 			0 on success
 *
 */

int aniAsfIpcConnect(tAniIpc *ipc, char *host, int prog, int vers)
{
	struct hostent hp;
	socklen_t len;
	short rport;
	int nbio;
	char buf[1024];
	int buflen = 1024;
	struct hostent *result;
	int herrno = -1;
	char *tmphost = host;
	
	if (!host)
	{
		 aniAsfLogMsg(ANI_GETHOSTBYNAME_ERR,
				hstrerror(herrno), ipc, ipc->sd);
		return(-1);
	}

#define HOST_FILE_BUG 1
#ifdef HOST_FILE_BUG
	// There seems to be bug where the /etc/hosts
	// file looses the localhost entry causing all
	// applications to go into a spin loop.  This
	// is a tempororary workaround.
	if (strcmp(host, "localhost") == 0) {
		tmphost = "127.0.0.1";
	}
#endif // HOST_FILE_BUG
	memset(&hp, 0, sizeof(struct hostent));

	// Get the host entry; name to IP address
	if (gethostbyname_r(tmphost, &hp, buf, buflen,
			&result, &herrno) != 0) {
		aniAsfLogMsg(ANI_GETHOSTBYNAME_ERR, 
				hstrerror(herrno), ipc, ipc->sd);
		return(-1);
	}

	// if the type is not AF_INET then return error
	if ((hp.h_addrtype != AF_INET) || (hp.h_length != 4)) {
		aniAsfLogMsg(ANI_GETHOSTTYPE_ERR, ipc, ipc->sd);
		return(-1);
	}

	if (vers < 0) {
		// if vers is less than zero prog has the actual port number
		rport = (unsigned short)prog;
	} else {
#ifdef ASF_PORT_MAP_SUPPORTED
		int proto = (ipc->type == SOCK_STREAM) ? IPPROTO_TCP : IPPROTO_UDP;
		int rt;

		// if vers is positive numer then use aniAsfPmGetPort
		// to get the servers port number
		// Get the port number
 
		if ((rt = aniAsfPmGetPort(host, prog, vers, proto)) <= 0) {
			aniAsfLogMsg(ANI_PMGET_ERR);
			return(-1);
		}
		rport = (unsigned short) rt;
#else
    aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Port Map not supported");
    return -1;
#endif
	}

	// prepare of connection
	switch (ipc->domain) {
	case AF_NETLINK:
		len = sizeof(struct sockaddr_nl);
		memset((void *)&ipc->remote.snl, 0, len);
		ipc->remote.snl.nl_family = ipc->domain;
		ipc->remote.snl.nl_pid = 0;
		ipc->remote.snl.nl_groups = 0;
		break;
	case AF_PACKET:
		len = sizeof(struct sockaddr_ll);
		memset((void *)&ipc->remote.sll, 0, len);
		ipc->remote.sll.sll_family = ipc->domain;
		ipc->remote.sll.sll_protocol = ipc->type;
		ipc->remote.sll.sll_ifindex = ipc->interface;
		break;
	case AF_INET:
	default:
		len = sizeof(struct sockaddr_in);
		memset((void *)&ipc->remote.sin, 0, len);
		ipc->remote.sin.sin_family = ipc->domain;
		ipc->remote.sin.sin_port = htons(rport);
		memcpy(&ipc->remote.sin.sin_addr, hp.h_addr, hp.h_length);
		break;
	}

	nbio = 0;
	// Remove non blocking
	if (ioctl(ipc->sd, FIONBIO, &nbio)) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "FIONBIO",
				ipc, ipc->sd);
		return(-1);
	}

	// Connect to the server
	if (connect(ipc->sd, (struct sockaddr *)&ipc->remote, len) < 0) {
		aniAsfLogMsg(ANI_CONNECT_ERR, ipc, ipc->sd);
		return(-1);
	}

	nbio = 1;
	// Set socket again to non blocking
	if (ioctl(ipc->sd, FIONBIO, &nbio)) {
		aniAsfLogMsg(ANI_SETSOCK_ERR, "FIONBIO",
				ipc, ipc->sd);
		return(-1);
	}

	if (getpeername(ipc->sd, (struct sockaddr *)&ipc->remote, 
				&len) < 0) {
		aniAsfLogMsg(ANI_GETPEERNAME_ERR,
				ipc, ipc->sd);
		return(-1);
	}

	if (getsockname(ipc->sd, (struct sockaddr *)&ipc->local, 
				&len) < 0) {
		aniAsfLogMsg(ANI_GETSOCKNAME_ERR, ipc, ipc->sd);
		return(-1);
	}

	return(0);
}

/**
 * aniAsfIpcRecv
 *
 * FUNCTION:
 * 	Function to read data from a socket
 *
 * LOGIC:
 * 	Can be used to read both SOCK_STREAM & SOCK_DGRAM
 * 	For SOCK_STREAM uses the system call read to read until the required bytes 
 * 	are read, or there is nothing more available
 * 	For SOCK_DGRAM uses the system call recv to read a packet
 * 	Needs to be used after a select to make sure there is data to be read.
 * 	As aniAsfIpcRecv() Will NOT block if there is no data available on socket. 
 *
 * @param *ipc - The IPC pointer 
 * @param *ptr - buffer to hold the data read
 * @param bytes - total bytes to read
 *
 * @param return - -1 for error and errno will be set with actual error number;
 * 			total number of actual bytes read  on success
 *
 */

int aniAsfIpcRecv(tAniIpc *ipc, char *ptr, int bytes)
{
	if (ipc->type == SOCK_STREAM) {
		// For TCP connections
		int left, bread;

		left = bytes;
		while (left > 0) {

			// Read data from socket
			bread = read(ipc->sd, ptr, left);

			if (bread < 0) {
				if (errno == EINTR) {
					// if interrupted by a signal 
					// try again
					continue;
				} else if (errno == EAGAIN) {
					// If we are done reading what is available, let's get out 
					// with what we've got!
					return(bytes - left);
				} else {
					// It truly is a read error, so Return error
					aniAsfLogMsg(ANI_READ_ERR, 
							ipc, ipc->sd);
					return(-1);
				}
			} else if (bread == 0) {
				// Connection is closed
				break;
			}

			// Keep reading until byte count left
			left -= bread;
			ptr += bread;
		}

		// Return total bytes read
		// If the return value is less the 
		// bytes then the connection was closed
		return(bytes - left);
	} else {
		// For UDP connections read as pkts
		int rv;

		if ((rv = recv(ipc->sd, ptr, bytes, 0)) < 0) {
			aniAsfLogMsg(ANI_RECV_ERR, ipc,
					ipc->sd);
		}

		return(rv);
	}
}

/**
 * aniAsfIpcSend
 *
 * FUNCTION:
 * 	Function to write data to the socket
 *
 * LOGIC:
 * 	Uses the system call write to write data to the socket
 * 	Can be used for both SOCK_STREAM and SOCK_DGRAM
 *
 * @param *ipc - The IPC pointer 
 * @param *ptr - data buffer which holds the data to be written
 * @param bytes - total bytes to write
 *
 * @param return - -1 for error and errno will be set with actual error number;
 * 			total number of actual bytes written on success
 *
 */

int aniAsfIpcSend(tAniIpc *ipc, char *ptr, int bytes)
{
	if (ipc->type == SOCK_STREAM) {
		// For TCP connections
		int left, written;

		left = bytes;

		while (left > 0) {

			// Write data to socket
			written = write(ipc->sd, ptr, left);

			if (written < 0) {
				// if interrupted by a signal or
				// if resource temporarily unavailable
				// try again
				if ((errno == EINTR) ||
						(errno == EAGAIN)) {
					continue;
				}

				// Return error
				aniAsfLogMsg(ANI_WRITE_ERR,
						ipc, ipc->sd);
				return(-1);
			} else if (written == 0) {
				// Connection is closed
				break;
			}

			// Keep writing until byte count left
			left -= written;
			ptr += written;
		}

		// Return total bytes written
		// If the return value is less than 
		// bytes then the connection was closed
		return(bytes - left);
	} else {
		// For UDP & RAW connections write as pkts
		int rv;
		socklen_t len;
		struct sockaddr *remote;

		/* From man page
		 * If sendto() is used on a connection-mode
		 * (SOCK_STREAM, SOCK_SEQPACKET) socket,
		 * the arguments dest_addr and addrlen are ignored
		 * (and the error EISCONN may be returned
		 * when they are not NULL and 0),
		 * and the error ENOTCONN is returned
		 * when the socket was not actually connected
		 */
		remote = NULL;
		len = 0;

		if ((rv = sendto(ipc->sd, ptr, bytes, 0,
					remote, len)) < 0) {
			aniAsfLogMsg(ANI_SENDTO_ERR, ipc, ipc->sd);
		}

		return(rv);
	}
}

/**
 * aniAsfIpcSetFd
 *
 * FUNCTION:
 * 	Function to associate a call back function for every read socket descriptor
 *
 * LOGIC:
 * 	Set the corresponding socket descriptor bit to the global readfds
 * 	Adds this socket to the socket call back list
 *
 * @param *ipc - The IPC pointer 
 * @param (*func)(void *) - The call back function
 * @param *args - Call back functions context information
 *
 * @param return - None
 *
 */

void aniAsfIpcSetFd(tAniIpc *ipc, void (*func)(void *), void *args)
{
	// If IPC not initialized then init it
	if (!ipcinited) {
		aniAsfIpcInit();
	}

	// Sanity check the IPC structure
	if (ipc == NULL || ipc->sd >= FD_SETSIZE || ipc->sd < 0)
		return;

	if (!FD_ISSET(ipc->sd, &readfds)) {
		// Set the sd in the readfds list
		FD_SET(ipc->sd, &readfds);

		// preparations to add to the scblist
		ipc->scb.func = func;
		ipc->scb.args = args;
		ipc->scb.sd = ipc->sd;

		// Add to scblist
		scbAdd(&ipc->scb);

		ipc->callbackFlag = eANI_BOOLEAN_TRUE;
	}
}

/**
 * aniAsfIpcUnSetFd
 *
 * FUNCTION:
 * 	Function to disassociate a call back function for the read socket descriptor
 *
 * LOGIC:
 * 	Unsets the corresponding socket descriptor bit to the global readfds
 * 	Removes this socket from the socket call back list
 *
 *
 * @param *ipc - The IPC pointer 
 *
 * @param return - None
 *
 */

void aniAsfIpcUnSetFd(tAniIpc *ipc)
{
	// Sanity check the IPC structure
	if (ipc == NULL || ipc->sd >= FD_SETSIZE || ipc->sd < 0)
		return;

	if (FD_ISSET(ipc->sd, &readfds)) {
		// Unset the sd from the readfds list
		FD_CLR(ipc->sd, &readfds);

		// Remove from scblist
		scbRemove(&ipc->scb);
	}

	// Unset the callback flag always
	ipc->callbackFlag = eANI_BOOLEAN_FALSE;
}

/* Multiplication overflow detection */
int multiplyOverflow(long num1, long num2)
{
	long res;

	res = num1 * num2;
	if (num1 != 0 && res / num1 != num2)
		return -1;
	return 0;
}

/* Addition overflow detection */
int addOverflow(long num1, long num2)
{
	if ((num1 > 0) && (num1 > LONG_MAX - num2))
		return -1;
	if ((num1 < 0) && (num1 < LONG_MIN - num2))
		return -1;
	return 0;
}

/**
 * aniAsfIpcCheck
 *
 * FUNCTION:
 * 	Function to check if data is available in the socket
 *
 * LOGIC:
 * 	This is a function used to bypass aniAsfIpcProcess
 * 	Waits on select to check if any data arrrives in 20 msecs
 * 	Used by the portmap library
 *
 * @param *ipc - The IPC pointer 
 *
 * @param return - -1 on error with errno set with error value and
 * 0 on success
 *
 */

int aniAsfIpcCheck(tAniIpc *ipc, long msec)
{
	clock_t jiffies, prevjiffies, diffjiffies;
	clock_t tmp = 0;
	long ticksec = 1;
	struct timeval tmo;
	fd_set rfds;
	int rv = 0, n;

	n = ipc->sd + 1;

	memset(&rfds, 0, sizeof(fd_set));

	FD_SET(ipc->sd, &rfds);

	// Cache Timeout
	prevjiffies = times(NULL);
	// Get ticks/sec count
	ticksec = sysconf(_SC_CLK_TCK);
	if (ticksec < 0 || ticksec > LONG_MAX) {
		aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Invalid ticksec value");
		return -1;
	}

	while( msec > 0 )
	{
		// Set Timeout 
		tmo.tv_sec = msec / 1000;
		tmo.tv_usec = (msec % 1000) * 1000; // usecs

		if ((rv = select(n, &rfds, NULL, NULL, &tmo)) < 0) {
			// if interrupted by a signal or
			// if resource temporarily unavailable
			// return error
			if ((errno != EINTR) &&
					(errno != EAGAIN)) {
				aniAsfLogMsg(ANI_SELECT_ERR);
				return(-1);
			}
			else
			{
				// Determine the elapsed time
				// elapsed time => diffjiffies
				jiffies = times(NULL);

				// Watch out for the "wrap" scenario
				if( jiffies < prevjiffies )
					diffjiffies = jiffies + (~tmp - prevjiffies);
				else
					diffjiffies = jiffies - prevjiffies;

				// Update msec to contain the difference between
				// the desired timeout and elapsed time.
				// This new timeout will now be used by select to
				// complete the transaction
				if (multiplyOverflow((diffjiffies / ticksec), 1000)) {
					aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Invalid input provided");
					return -1;
				}
				if (addOverflow(((diffjiffies / ticksec) * 1000),
						(diffjiffies % ticksec))) {
					aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Invalid input provided");
					return -1;
				}
				msec -= (((diffjiffies / ticksec) * 1000) +
						(diffjiffies % ticksec));
				if (msec <= 0)
					break;

				// Update prevjiffies
				prevjiffies = jiffies;
			}
		}
		else
			break;
	}

	if (rv > 0) {
		if (FD_ISSET(ipc->sd, &rfds)) {
			return(1);
		}
	}

	return(0);
}

/**
 * aniAsfIpcBlockSelect
 *
 * FUNCTION:
 * 	Function to set the flag to make the select system to block
 *
 * LOGIC:
 * 	Function to set the flag to make the select system to block
 *
 * @param None
 *
 * @param return - None
 *
 */

void aniAsfIpcBlockSelect(void)
{
	// Make the select call as blocked
	blocked = 1;
}

/**
 * aniAsfIpcProcess
 *
 * FUNCTION:
 * 	Function to check for read activity on registered sockets
 *
 * LOGIC:
 * 	Walks through the socket call back list for any data to be read
 * 	Uses system call select with a timeout value of 100 msecs
 * 	If any data present in the read socket descriptor it then 
 * 		invokes its corresponding call back function
 *
 *
 * @param None
 *
 * @param return - errno on error
 *
 */

int aniAsfIpcProcess(void)
{
	struct timeval tmo;
	struct timeval *tmoptr;
	fd_set rfds;
	tScb *tscb, **ascb;
	int i, n, rv;

	if (scblist != NULL) {
		// Maximum fd's select has to serve 
		n = scblist->sd + 1;

		// restore readfds from the stored value
        rfds = readfds;
	} else {
		// default values
		n = 0;

	    FD_ZERO(&rfds);
	}

	// Timeout set to 100 msecs
	tmo.tv_sec = 0;
	tmo.tv_usec = 100000; // 100 msecs

	// If the select needs to be blocked then pass tmo as NULL
	if (blocked) {
		tmoptr = NULL;
	} else {
		tmoptr = &tmo;
	}

	// Wait for any activity on socket descriptors
	// Select could break out due to timeout or any signal
	if ((rv = select(n, &rfds, NULL, NULL, tmoptr)) < 0) {
		// if interrupted by a signal or
		// if resource temporarily unavailable
		// return error
		if ((errno != EINTR) &&
				(errno != EAGAIN)) {
			aniAsfLogMsg(ANI_SELECT_ERR);
			return(errno);
		}
	}

	if (rv > 0) {

		// Create memory to store the ready scb's
		if ((ascb = (tScb **)aniCalloc(rv, sizeof(tScb *))) == NULL) {
			aniAsfLogMsg(ANI_MALLOC_ERR);
			return(errno);
		}

		// lock the critical section
		if ((errno = pthread_mutex_lock(&scbmutex)) > 0) {
			aniFree(ascb);
			aniAsfLogMsg(ANI_MUTEX_ERR);
			return(errno);
		}

		i = 0;

		// Walk thru the socket descriptor list
		for (tscb = scblist; tscb != NULL; tscb = tscb->next) {
			if (FD_ISSET(tscb->sd, &rfds) && tscb->func) {
				// Put this scb's pointer into a lscb list 
				// so that socket call back functions can add 
				// and delete scblist (aniAsfIpcSetFd/aniAsfIpcUnSetFd) 
				// in multithreaded applications
				ascb[i++] = tscb;
			}
		}

		// unlock the critical section
		if ((errno = pthread_mutex_unlock(&scbmutex)) > 0) {
			aniFree(ascb);
			aniAsfLogMsg(ANI_MUTEX_ERR);
			return(errno);
		}

		// Walk thru the ready list
		for (i = 0; i < rv; i++) {
			tscb = (tScb *)ascb[i];
			if (tscb) {
				// invoke the callback function
				(*tscb->func)(tscb->args);
			}
		}

		// Free the ready scb't memory pointer
		aniFree(ascb);
	}
	return(0);
}

/**
 * aniAsfIpcGetSnl
 *
 * FUNCTION:
 * 	Function to return the local sockaddr_nl
 *
 * LOGIC:
 * 	The PID field of the sockaddr_nl struct is needed for
 * 	netlink socket communication with the kernel. 
 *
 * @param *ipc - The IPC pointer
 *
 * @param return - struct sockaddr_nl * or NULL
 *
 */

struct sockaddr_nl *aniAsfIpcGetSnl(tAniIpc *ipc)
{
	if (ipc && ipc->domain == AF_NETLINK)
		return &ipc->local.snl;

	return NULL;		
}

/**
 * aniAsfIpcGetSock
 *
 * FUNCTION:
 * 	Function to return the socket descriptor
 *
 * LOGIC:
 * 	Returns the socket descriptor if the ipc is not NULL
 *
 * @param *ipc - The IPC pointer
 *
 * @param return - socket descriptor or -1
 *
 */

int aniAsfIpcGetSock(tAniIpc *ipc)
{
	return (ipc ? ipc->sd : -1);
}

/**
 * aniAsfIpcGetAniMsgType
 *
 * FUNCTION:
 * 	Function to return the socket descriptor
 *
 * LOGIC:
 * 	Returns the Airgo Message Type if the ipc is not 
 * 	NULL and the socket type is not SOCK_STREAM
 *
 * @param *ipc - The IPC pointer
 *
 * @param return - Airgo Message Type or -1
 *
 */

int aniAsfIpcGetAniMsgType(tAniIpc *ipc)
{
	if (ipc && ipc->type != SOCK_STREAM)
		return ipc->msgType;

	return -1;		
}

/**
 * aniAsfIpcGetAniMsgLen
 *
 * FUNCTION:
 * 	Function to return the socket descriptor
 *
 * LOGIC:
 * 	Returns the Airgo Message Type if the ipc is not 
 * 	NULL and the socket type is not SOCK_STREAM
 *
 * @param *ipc - The IPC pointer
 *
 * @param return - Airgo Message Length or -1
 *
 */

int aniAsfIpcGetAniMsgLen(tAniIpc *ipc)
{
	if (ipc && ipc->type != SOCK_STREAM)
		return ipc->msgLen;

	return -1;		
}

/**
 * scbAdd
 *
 * FUNCTION:
 * 	Internal Socket call back add function
 *
 * LOGIC:
 * 	Adds the call back function to the scb list
 * 	Inserts in the descending order of the socket descriptor.
 * 	First in the list, is always the highest socket descriptor number 
 * 		used by select
 *
 *
 * @param *scb - Socket call back pointer
 *
 * @param return - None
 *
 */

static void scbAdd(tScb *scb)
{
	tScb *tnext = NULL, *tprev = NULL;

	// If NULL pointer, then return
	if (scb == NULL) {
		return;
	}

	// lock the critical section
	if ((errno = pthread_mutex_lock(&scbmutex)) > 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
		return;
	}

	// Search thru the list for the right spot to insert
	// in the descending order of socket descriptor
	for (tnext = scblist; tnext != NULL; tnext = tnext->next) {
		if (tnext->sd < scb->sd) {
			break;
		}
		tprev = tnext;
	}

 	// Insert to list
	if (tprev == NULL) {
		scblist = scb;
	} else {
		tprev->next = scb;
	}

	scb->next = tnext;

	// unlock the critical section
	if ((errno = pthread_mutex_unlock(&scbmutex)) > 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
	}
}

/**
 * scbRemove
 *
 * FUNCTION:
 * 	Internal Socket Call back Remove Function
 *
 * LOGIC:
 * 	Walks through the list and removes the socket descriptor from the list
 *
 * @param *scb - Socket call back pointer
 *
 * @param return - None
 *
 */

static void scbRemove(tScb *scb)
{
	tScb *tcur = NULL, *tlast = NULL;

	// If NULL pointer, then return
	if (scb == NULL) {
		return;
	}

	// lock the critical section
	if ((errno = pthread_mutex_lock(&scbmutex)) > 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
		return;
	}

	// Verify that entry is really on list
	for (tcur = scblist; tcur != NULL; tcur = tcur->next) {
		if (tcur == scb) {
			break;
		}
		tlast = tcur;
	}

	// Remove it from the list
	if (tcur != NULL) {
		if (tlast != NULL) {
			tlast->next = tcur->next;
		} else {
			scblist = tcur->next;
		}
	}

	// unlock the critical section
	if ((errno = pthread_mutex_unlock(&scbmutex)) > 0) {
		aniAsfLogMsg(ANI_MUTEX_ERR);
	}
}

