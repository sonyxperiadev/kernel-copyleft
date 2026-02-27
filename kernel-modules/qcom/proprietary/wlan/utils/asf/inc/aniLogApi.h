/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/* vi: set sw=2  ts=2 */

/***********************************************************************
 * aniLogApi.c
 *
 * This file contains the fault scanning implementation from syslog 
 * messages ile .
 *
 **********************************************************************/

#ifndef _ANI_SCAN_LOG_API_H
#define _ANI_SCAN_LOG_API_H

/* Includes */
#include "aniTypes.h"
#include "aniErrors.h"
#include "aniAsfLog.h"

#include <time.h>

/* Types Definitions and Defines */
/* error codes. This should ideally go into aniErrors.h */
#define ANI_E_END_ITERATION -1


/* for fault log scanning */
#define SCAN_ON     1
#define SCAN_OFF    0


/** structure of a Log Message, contents of a GetLogMsg() to be
 *  returned according to this structure
 */
typedef struct sAniLogMessage {
  time_t     aniLogTime;
  ANI_S8     aniHostAddress[255];
  ANI_S8     aniModuleName[255];
  ANI_U32    aniLogLevel;
  ANI_U32    aniEventId;
  ANI_U32    aniStringId;
  
  /* Event parameters. Interpret this according to event format */
  ANI_S8     aniEventParameters[1024]; 
} tAniLogMessage;



/* Filter structure to be passed to getLogMsg for event 
 * filtering
 */
typedef struct sAniLogFilter {
  /* Bit flags to indicate filter criteria */
#define ANI_LOG_NONE_FLAG      0x00
#define ANI_LOG_ALL_FLAG       0xff
#define ANI_LOG_TIME_FLAG      0x01
#define ANI_LOG_HOST_FLAG      0x02
#define ANI_LOG_MODULE_FLAG    0x04
#define ANI_LOG_EVENT_FLAG     0x08
#define ANI_LOG_LEVEL_FLAG     0x10
  ANI_U16    flags;
  time_t     aniStartTime;
  time_t     aniEndTime;
  ANI_S8     aniHostAddress[255];
  ANI_S8     aniModuleName[255];
  ANI_U32    aniEventId;
  ANI_U32    aniLogLevel;
} tAniLogFilter;


/* Context Handle, abstract for the calling module. 
 * Interpreted by Scan Library as defined in aniLogApi.c
 */
typedef ANI_U32  tAniLogHandle;

/* Log Scan API */

/* 1. First function to be called by the Scanning App
 * Input: Log file configured for use by syslog
 * Returns: a context on a successful Init or a NULL pointer 
 */
extern tAniLogHandle 
aniAsfLogInitScan(char *syslogFile);



/* 2. Cleans up resources allocated by the Init function.
 * Needs to be called after the log retrieval is done
 * Returns: None 
 */
extern void 
aniAsfLogStopScan(tAniLogHandle aniLogHandle);



/* 3. Log retrieval API
 * Accepts: filter, output buffer and scan handle, 
 * reads the log file, retrieves messages based on filter,
 * and returns either the requested number of records or 
 * the filtered contents whichever is less
 *
 * Return value: ANI_OK indicating more records.
 *               Generic ASF errors in case of error
 *               ANI_E_END_ITERATION on end of records
 */
extern aniError 
aniAsfLogGetMsg(tAniLogHandle  aniLogHandle,
    tAniLogFilter  aniLogFilter,
    tAniLogMessage *buffer,
    ANI_U32        sizeOfBuffer,
    ANI_U32        *noOfEntries);



/* 4. Event Transformation */
extern aniError 
aniAsfLogEventTransform (ANI_U32 aniEventId,
    ANI_U32 aniCurrentLogLevel,
    ANI_U32 aniTransformedLogLevel);
                               


#endif /* _ANI_SCAN_LOG_API_H */


/*
   Local Variables:
   c-file-style: "linux"
   c-basic-offset: 2
   tab-width: 2
End:
 */


