/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file asfportmap.c has the Port Map Library library code
 * Author:  U. Loganathan
 * Date:    May 31st 2002
 * History:-
 * Date     Modified by Modification Information
 *
 */

#ifdef ASF_PORT_MAP_SUPPORTED

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "aniAsfHdr.h"
#include "aniAsfLog.h"

#define ANI_PMAP_FILE		"/dev/null"
#define ANI_PMAP_MAX_VERS	4

#define ANI_PMAP_ENTRIES	(ANI_PROG_ID_END - ANI_PROG_ID_START)

typedef enum sAniProto {
	ANI_PMAP_UDP,
	ANI_PMAP_TCP,
	ANI_PMAP_MAX_PROTO
} tAniProto;

typedef struct sVersMap {
	int		versId;
	int		port[ANI_PMAP_MAX_PROTO];
} ANI_PACK_STRUC tVersMap;

typedef struct sPortMap {
	int		numVers;
	tVersMap	verMap[ANI_PMAP_MAX_VERS];
} ANI_PACK_STRUC tPortMap;

char *aniAsfAttachPortMapAddr()
{
	int shmId;
	int shmKey;
	char *shmAddr;
	int size = sizeof(tPortMap) * ANI_PMAP_ENTRIES;
	int created = 1;

	if ((shmKey = ftok(ANI_PMAP_FILE, 'l')) < 0) {
		aniAsfLogMsg(LOG_CRIT, ANI_WHERE,
			"Error(%d): Cannot Get Key", errno);
		return NULL;
	}

	errno = 0;
	// Create the shared memory only if it does not exist already
	if ((shmId = shmget(shmKey, size,
				(IPC_CREAT | IPC_EXCL | 0666))) < 0) {

#ifndef BRECIS
		if (errno == EEXIST)
#endif // BRECIS
		{
			// If already available just get the 
			// shared memory id
			shmId = shmget(shmKey, 0, 0);
			created = 0;
		}
		
		if (shmId < 0) {
			aniAsfLogMsg(LOG_CRIT, ANI_WHERE,
				"Error(%d): Cannot Access Shared Memory", errno);
			return NULL;
		}
	}

	// Attach to the shared memory segment
	if ((shmAddr = shmat(shmId, NULL, 0)) == NULL) {
		aniAsfLogMsg(LOG_CRIT, ANI_WHERE,
				"Error(%d): Cannot Attach to Shared Memory", errno);
		return NULL;
	}

	// When the shared memory segment gets created for 
	// the first time initialize it with zeros
	if (created == 1) {
		aniAsfLogMsg(LOG_NOTICE, ANI_WHERE,
				"!!!! Initializing Port Map Memory !!!!");
		memset(shmAddr, 0, size);
	}

	return(shmAddr);
}

void aniAsfDetachPortMapAddr(char *shmAddr)
{
	if (shmdt(shmAddr) < 0) {
		aniAsfLogMsg(LOG_CRIT, ANI_WHERE,
				"Cannot Detach Shared Memory");
		return;
	}
}

/**
 * aniAsfPmSet
 *
 * FUNCTION:
 *  Function to register the port map entry with the server
 *
 * LOGIC:
 *  Registers the program Id, Version Id, Protocol and port with the server
 *
 * @param progId - Program identification number
 * @param versId - Program version identification number
 * @param proto - Program protocol
 * @param port - Program's locally binded port
 *
 * @param return - -1 on error; or 0 on success
 *
 */

int aniAsfPmSet(int progId, int versId, int proto, int port)
{
	tPortMap *pMap;
	char *pMapAddr;
	int progIndex;
	int pMapProto;
	int numVers;
	int i;

	if ((progId < ANI_PROG_ID_START) || 
			(progId >= ANI_PROG_ID_END)) {
		return(-1);
	}

	switch (proto) {
	case IPPROTO_TCP:
		pMapProto = ANI_PMAP_TCP;
		break;
	case IPPROTO_UDP:
		pMapProto = ANI_PMAP_UDP;
		break;
	default:
		return(-1);
		break;
	}

	if ((pMapAddr = aniAsfAttachPortMapAddr()) == NULL) {
		return(-1);
	};

	progIndex = progId - ANI_PROG_ID_START;

	pMap = (tPortMap *)pMapAddr;
	pMap += progIndex;
	
	numVers = pMap->numVers;

	if (numVers >= ANI_PMAP_MAX_VERS) {
		aniAsfDetachPortMapAddr(pMapAddr);
		return(-1);
	}

	if (numVers == 0) {
		pMap->verMap[numVers].versId  = versId;
		pMap->verMap[numVers].port[pMapProto] = port;
		pMap->numVers++;
	} else {
		for (i = 0; i < numVers; i++) {
			// If already in the list update it
			if (versId == pMap->verMap[i].versId) {
				pMap->verMap[i].port[pMapProto] = port;
				break;
			}
		}

		// New version entry - add it
		if (i == numVers) {
			pMap->verMap[i].versId  = versId;
			pMap->verMap[i].port[pMapProto] = port;
			pMap->numVers++;
		}
	}

	aniAsfDetachPortMapAddr(pMapAddr);

	return(0);
}

/**
 * aniAsfPmUnSet
 *
 * FUNCTION:
 *  Function to unregister the port map entry from the server
 *
 * LOGIC:
 *  UnRegisters the program Id, Version Id, Protocol from the server
 *
 * @param progId - Program identification number
 * @param versId - Program version identification number
 * @param proto - Program protocol
 *
 * @param return - -1 on error; or 0 on success
 *
 */

int aniAsfPmUnSet(int progId, int versId, int proto)
{
	tPortMap *pMap;
	char *pMapAddr;
	int progIndex;
	int pMapProto;
	int numVers;
	int i;

	if ((progId < ANI_PROG_ID_START) || 
			(progId >= ANI_PROG_ID_END)) {
		return(-1);
	}

	switch (proto) {
	case IPPROTO_TCP:
		pMapProto = ANI_PMAP_TCP;
		break;
	case IPPROTO_UDP:
		pMapProto = ANI_PMAP_UDP;
		break;
	default:
		return(-1);
		break;
	}

	if ((pMapAddr = aniAsfAttachPortMapAddr()) == NULL) {
		return(-1);
	};

	progIndex = progId - ANI_PROG_ID_START;

	pMap = (tPortMap *)pMapAddr;
	pMap += progIndex;
	
	numVers = pMap->numVers;

	for (i = 0; i < numVers; i++) {
		if (versId == pMap->verMap[i].versId) {
			pMap->verMap[i].port[pMapProto] = 0;
			break;
		}
	}

	aniAsfDetachPortMapAddr(pMapAddr);

	return(0);
}

/**
 * aniAsfPmGetPort
 *
 * FUNCTION:
 *  Function to get the port number from the server
 *
 * LOGIC:
 *  Gets the port number for the program Id, Version Id, Protocol from the server
 *
 * @param *host - hostname of the portmap server; [TODO: this may only
 * accept "localhost" once the portmapd security analysis is done.]
 * @param progId - Program identification number
 * @param versId - Program version identification number
 * @param proto - Program protocol
 *
 * @param return - -1 on error; or 0 on success
 *
 */

int aniAsfPmGetPort(char *host, int progId, int versId, int proto)
{
	tPortMap *pMap;
	char *pMapAddr;
	int progIndex;
	int pMapProto;
	int port = 0;
	int i;

	if ((progId < ANI_PROG_ID_START) ||
			(progId >= ANI_PROG_ID_END)) {
		return(-1);
	}

	switch (proto) {
	case IPPROTO_TCP:
		pMapProto = ANI_PMAP_TCP;
		break;
	case IPPROTO_UDP:
		pMapProto = ANI_PMAP_UDP;
		break;
	default:
		return(-1);
		break;
	}

	if ((pMapAddr = aniAsfAttachPortMapAddr()) == NULL) {
		return(-1);
	};

	progIndex = progId - ANI_PROG_ID_START;

	pMap = (tPortMap *)pMapAddr;
	pMap += progIndex;
	
	for (i = 0; i < pMap->numVers; i++) {
		if (versId == pMap->verMap[i].versId) {
			port = pMap->verMap[i].port[pMapProto];
			break;
		}
	}

	aniAsfDetachPortMapAddr(pMapAddr);

	if (port > 0) {
		return(port);
	} else {
		return(-1);
	}
}

/**
 * aniAsfPmDump
 *
 * FUNCTION:
 *  Function to get all the port map entries from the server
 *
 * LOGIC:
 *  Get all the Registers port map entries 
 *      program Id, Version Id, Protocol and port from the server
 *
 * @param None
 *
 * @param return - -1 or 0 on error; or port number on success
 *
 */

int aniAsfPmDump(void)
{
	tPortMap *pMap, *tMap;
	char *pMapAddr;
	int proto;
	int i;
	int j;
	int k;

	if ((pMapAddr = aniAsfAttachPortMapAddr()) == NULL) {
		return(-1);
	};


	tMap = (tPortMap *)pMapAddr;

	for (i = 0; i < ANI_PMAP_ENTRIES; i++) { 

		pMap = (tMap + i);

		for (j = 0; j < pMap->numVers; j++) {

			for (k = 0; k < ANI_PMAP_MAX_PROTO; k++) {

				if (pMap->verMap[j].port[k]) {

					switch (k) {
					case ANI_PMAP_TCP:
						proto = IPPROTO_TCP;
						break;
					case ANI_PMAP_UDP:
						proto = IPPROTO_UDP;
						break;
					default:
						proto = 0;
						break;
					}

            				printf("%10d %5d %5d %10d\n",
						i + ANI_PROG_ID_START ,
						pMap->verMap[j].versId, 
                				proto, pMap->verMap[j].port[k]);

				}
			}
		}

	}

	aniAsfDetachPortMapAddr(pMapAddr);

	return(0);
}

#endif //ASF_PORT_MAP_SUPPORTED

