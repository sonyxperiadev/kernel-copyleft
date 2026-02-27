/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 *
 * This file contains the necessary data structures, prototypes and
 * definitions of message types required for the implementation of
 * the Device Configuration Layer (DCL)
 *
 * Author:  Ashok Ranganath
 * 
 * Date:  July 29 2002
 *
 * History:-
 * Date           Modified by      Modification Information
 * -------------------------------------------------------------------
 * 09/25/2002     Joseph L R       Updated to support TTLVs as a library
 *
 */

#ifndef _ANI_ASF_TTLV_H_
#define _ANI_ASF_TTLV_H_

#include "aniTypes.h"
#include "aniAsfIpc.h"
#include "aniAsfHdr.h"

/********************************************************************/

#define ANI_TTLV_HDR_SIZE  (sizeof(ANI_U32) + sizeof(ANI_U16))
#define ANI_MIN_TTLV_SIZE  ANI_TTLV_HDR_SIZE

typedef struct stAniTtlv {
    ANI_U32 paramType;		/* The Parameter Type of this TTLV */
    ANI_U16 length;		/* Length of the TTLV including hdr */
    u_char *pData;		/* The data associated with this TTLV */
} tAniTtlv;

/* Opaque TTLV Message, that holds an ASF Message */
typedef struct stAniTtlvMsg tAniTtlvMsg;

/**
 *
 * A Typical TTLV encoded message is structured as follows:
 * 
 * --------------------------------------------------------
 * |  MESSAGE TYPE (2 bytes)    | MESSAGE LENGTH (2 bytes)|
 * --------------------------------------------------------
 * |              MESSAGE PARAMETERS (TTLV 1)             |
 * | Data-Type(2 bytes)         |   Val-Id (2 bytes)      |
 * --------------------------------------------------------
 * |               Value(N bytes)                         |
 * --------------------------------------------------------
 * |              MESSAGE PARAMETERS (TTLV 2 )            |
 * | Data-Type(2 bytes)         |   Val-Id (2 bytes)      |
 * --------------------------------------------------------
 * |               Value(N bytes)                         |
 * --------------------------------------------------------
 *
 * 
 * Depending on the interface published by the subsytem
 * the MESSAGE-PARAMETERS might be absent altogether.
 *
 */

extern char *aniAsfPutTtlv(char *, ANI_U32, ANI_U32, char *);
extern tAniTtlv *aniAsfGetStructTtlv(char *, ANI_U32 *);
extern void aniAsfFreeStructTtlv(tAniTtlv *);

#endif /*  _ANI_ASF_TTLV_H_ */
