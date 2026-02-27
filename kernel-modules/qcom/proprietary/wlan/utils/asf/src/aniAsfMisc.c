/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file asfmisc.c has the  Manager library code
 * Author:  U. Loganathan
 * Date:    Jul 2nd 2002
 * History:-
 * Date     Modified by Modification Information
 *
 */

#include <stdio.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>

#include "aniTypes.h"
#include "aniAsfMem.h"
#include "aniAsfHdr.h"
#include "aniAsfIpc.h"
#include "aniAsfValIds.h"
#include "aniAsfTtlv.h"
#include "aniAsfLog.h"
#include "aniErrors.h"
#include "aniUtils.h"

/*
 * Machine-independent, alignment insensitive network-to-host long
 * conversion.
 * 
 */
ANI_U32 aniAsfGet32(char *cp)
{
	return(aniAsfRd32(cp));
}

/*
 * Put a long in host order into a char array in network order. 
 * 
 */
char *aniAsfPut32(char *cp, ANI_U32 x)
{
	return(aniAsfWr32(cp, x));
}

ANI_U16 aniAsfGet16(char *cp)
{
	return(aniAsfRd16(cp));
}

/* Put a short in host order into a char array in network order */
char *aniAsfPut16(char *cp, ANI_U16 x)
{
	return(aniAsfWr16(cp, x));
}

void aniAsfNtohHdr(char *start)
{
	aniAsfNtohAniHdr(start);
}

void aniAsfHtonWniHdr(tAniHdr *hdr)
{
	aniAsfHtonAniHdr(hdr);
}

/**
 * aniAsfPutTtlv
 *
 * FUNCTION:
 * 	This routine adds a new TLV to the end of the send packet buffer
 *
 * LOGIC:
 * 	This is a generic routine that appends a TLV to a given packet
 * 	buffer. It copies the number of bytes as specified in the TLV
 * 	length parameter
 *
 * @param pBuffer - pointer to the packet buffer. The new TLV will
 *                  be added at this offset.
 *
 * @param pTLV - pointer to a TLV struct buffer to be added
 *
 * @param return - OK, indicates that the routine executed successfully
 *                 ERROR, indicates there was an error
 *
 */
char *aniAsfPutTtlv(char *pBuffer, ANI_U32 paramType,
			ANI_U32 valLen, char *val)
{
	// Copy this TLV to the packet bufer
	// asfPut16 does the hton conversion
	// Copy the data type
	pBuffer = aniAsfPut32(pBuffer, paramType);

	// Copy the length
	pBuffer = aniAsfPut16(pBuffer, valLen + 6);

	// Copy the value buffer
	switch(((paramType >> 16) & 0xFFFF)) {
	case ANI_TYPE_U8:
	case ANI_TYPE_S8:
		*pBuffer = *val;
		pBuffer++;
		break;
	case ANI_TYPE_U16:
	case ANI_TYPE_S16:
		pBuffer = aniAsfPut16(pBuffer, *((ANI_U16 *)val));
		break;
	case ANI_TYPE_U32:
	case ANI_TYPE_S32:
	case ANI_TYPE_IPADDR:
		pBuffer = aniAsfPut32(pBuffer, *((ANI_U32 *)val));
		break;
	case ANI_TYPE_IP_PFX_V4:
		pBuffer = aniAsfPut32(pBuffer, *((ANI_U32 *)val));
		pBuffer = aniAsfPut32(pBuffer, *((ANI_U32 *)(val+4)));
		break;
	case ANI_TYPE_NONE:
		break;
	default:
		memcpy(pBuffer, val, valLen);
		pBuffer += valLen;
		break;
	}
	return pBuffer;
}

/**
 * aniAsfGetStructTtlv
 *
 * FUNCTION:
 * 	This routine extracts a TLV from a given receive packet buffer
 *
 * LOGIC:
 * 	This is a generic routine that extracts a TLV from a given packet
 * 	buffer. It copies the number of bytes as specified in the TLV
 * 	length parameter
 *
 * @param pBuffer - pointer to the packet buffer. The TLV will be
 *                  extracted at this offset.
 *
 * @param pTLV - pointer to a TLV struct buffer that contains the
 *               TLV that was just extracted
 *
 * @param return - OK, indicates that the routine executed successfully
 *                 ERROR, indicates there was an error
 *
 */
tAniTtlv *aniAsfGetStructTtlv(char *pBuffer, ANI_U32 *len)
{
  	tAniTtlv *ttlv;

	ttlv = aniMalloc(sizeof(tAniTtlv));

	if (ttlv == NULL) {
		printf("malloc error \n");
		return(NULL);
	}

	// Initialize the data member pData. This will
	// ensure that when aniAsfFreeStructTtlv is called,
	// aniFree( ttlv->pData ) is not invoked for an
	// invalid/uninitialized pointer
	ttlv->pData = NULL;

	// asfGet16 does the ntoh conversion
	// Get the param type
	ttlv->paramType = aniAsfGet32(pBuffer);
	pBuffer += 4;

	// Get the length of the value parameter
	ttlv->length = aniAsfGet16(pBuffer);
	pBuffer += 2;

	switch(((ttlv->paramType >> 16) & 0xFFFF)) {
	case ANI_TYPE_U8:
	case ANI_TYPE_S8:
	  	ttlv->pData = aniMalloc(sizeof(ANI_TYPE_U8));

		if (ttlv->pData == NULL) {
		  aniFree(ttlv);
		  return(NULL);
		}

		*ttlv->pData = *pBuffer;
		pBuffer++;
		break;
	case ANI_TYPE_U16:
	case ANI_TYPE_S16:
	  	ttlv->pData = aniMalloc(sizeof(ANI_TYPE_S16));

		if (ttlv->pData == NULL) {
		  aniFree(ttlv);
		  return(NULL);
		}

		*((ANI_U16 *) ttlv->pData) = aniAsfGet16(pBuffer);
		pBuffer += 2;
		break;
	case ANI_TYPE_U32:
	case ANI_TYPE_S32:
	case ANI_TYPE_IPADDR:
	  	ttlv->pData = aniMalloc(sizeof(ANI_TYPE_S32));

		if (ttlv->pData == NULL) {
		  aniFree(ttlv);
		  return(NULL);
		}

		*((ANI_U32 *) ttlv->pData) = aniAsfGet32(pBuffer);
		pBuffer += 4;
		break;
	case ANI_TYPE_STR:
		// Doing a malloc ONLY if value length greater than 0
		// This will ensure that the caller will at least get
		// the PT (4 bytes) and TTLV_LEN (2 bytes)
		if( ttlv->length > 6 )
		{
			ttlv->pData = aniMalloc((ttlv->length - 5));

			if (ttlv->pData == NULL) {
				aniFree(ttlv);
				return(NULL);
			}
			memcpy(ttlv->pData, pBuffer, ttlv->length - 6);
			ttlv->pData[ttlv->length - 6] = '\0';
			pBuffer += (ttlv->length - 6);
		}
		break;
	case ANI_TYPE_MACADDR:
	  	ttlv->pData = aniMalloc(sizeof(tAniMacAddr));

		if (ttlv->pData == NULL) {
		  aniFree(ttlv);
		  return(NULL);
		}

		memcpy(ttlv->pData, pBuffer, sizeof(tAniMacAddr));
		pBuffer += (ttlv->length - sizeof(tAniMacAddr));
		break;
	case ANI_TYPE_IP_PFX_V4:
	  	ttlv->pData = aniMalloc(2*sizeof(ANI_TYPE_S32));

		if (ttlv->pData == NULL) {
		  aniFree(ttlv);
		  return(NULL);
		}

		/* ipAddress */
		*((ANI_U32 *) ttlv->pData) = aniAsfGet32(pBuffer);
		/* prefixlen */
		*((ANI_U32 *) (ttlv->pData+4)) = aniAsfGet32(pBuffer+4);
		pBuffer += 8;
		break;
	default:
		// Doing a malloc ONLY if value length greater than 0
		// This will ensure that the caller will at least get
		// the PT (4 bytes) and TTLV_LEN (2 bytes)
		if( ttlv->length > 6 )
		{
		  ttlv->pData = aniMalloc((ttlv->length - 6));

			if (ttlv->pData == NULL) {
			  aniFree(ttlv);
			  return(NULL);
			}

			memcpy(ttlv->pData, pBuffer, ttlv->length - 6);
			pBuffer += (ttlv->length - 6);
		}
		break;
	}

	*len = ttlv->length;

	return(ttlv);
}

void aniAsfFreeStructTtlv(tAniTtlv *ttlv)
{
	if (ttlv) {

		if (ttlv->pData) {
			aniFree(ttlv->pData);
		}

		aniFree(ttlv);
	}
}



/** 
 * freeTLV
 *
 * FUNCTION:
 *  This routine frees any memory that was previouly allocated
 *  using this TLV object
 *  
 * LOGIC:
 *  This is a generic routine that frees the memory allocated using
 *  the TLV.value parameter. Should the TLV object itself have been
 *  allocated using memCreate?
 *
 * @param pTLV - pointer to a TLV struct buffer.
 *               NOTE: This routine only frees the value parameter
 *  
 * @param return - OK, indicates that the routine executed successfully
 *                 ERROR, indicates there was an error
 *
 */

int freeTTLV( tAniTtlv *pTTLV )
{
    if( NULL != pTTLV->pData)
        aniFree( pTTLV->pData);
    memset( pTTLV, 0, sizeof( tAniTtlv ));
    return ANI_OK;
}


//
// ASF HDR + LP HDR + TTL
//
/*
 * Here's how an ASF packet header will look like:
 * NOTE: The Time-To-Live has been reduced to a 16-bit field
 *
 *bit                        bit bit                     bit
 * 31.........................16.15.......................0
 * --------------------------------------------------------
 * |            MESG ID         |        MESG LENGTH      |
 * --------------------------------------------------------
 * |    Large Packet Header     |           TTL           |
 * --------------------------------------------------------
 * |                    MESSAGE PAYLOAD                   |
 * --------------------------------------------------------
 */

//
// Zooming in on the LP HDR
//
/*
 * Here's what an LP header will look like
 *
 *bit                        bit bit                      bit
 * 31.........................24.23.......................16
 * --------------------------------------------------------
 * |   RESERVED (CM signature)  | LP |s7|s6|s5|s4|s3|s2|s1|
 * --------------------------------------------------------
 *  
 * where,
 * 
 * LP - bit23 : Indicates if this packet is a fragment of a
 * Large Packet
 *
 * s1-s7 - bit16..bit22 : Identifies the sequence number of a
 * fragment of a Large Packet
 */


/**
 * aniAsfEncodeLpHdr
 *
 * This routine encodes the (L)arge (P)acket (nee TTL)
 * header. If the packet being sent is a fragment of a Large
 * Packet, then the lpBit is set and the sequence number is
 * also updated appropriately
 *
 * @param lpHdr - (output) The 4-byte LP header, which is the
 * 4-bytes immediately following the ASF HDR (also 4-bytes).
 * This is updated with the appropriate header parameters by
 * this routine
 *
 * @param lpBit - (input) Indicates if the LP bit (bit23)
 * needs to be set or not
 *
 * @param seqNum - (input) Indicates the sequence number to
 * be set for this fragment
 *
 * @return - the NEW (LP + TTL) Header (4-bytes) upon success
 *           ANI_ERROR upon encountering a failure
 */
int aniAsfEncodeLpHdr( ANI_U32 *lpHdr, int lpBit, int seqNum )
{
	if( NULL == lpHdr )
		return ANI_ERROR;

	if( lpBit )
		*lpHdr = *lpHdr | 0x800000;
	else
		*lpHdr = *lpHdr & 0xff00ffff;

	seqNum = seqNum % 0x80;
	*lpHdr = ((*lpHdr & 0xff80ffff) | ((seqNum & 0x7f) << 16));

	return *lpHdr;
}

/**
 * aniAsfSendMsg
 *
 * Sends an ASF message to the destination process identified
 * by the object ipcc. Takes care of handling large packets,
 * which are packets with payload exceeding 16384 bytes
 *
 * @param *ipcc - the tAniIpc pointer, which identifies the
 * destination process to which this message has to be sent
 *
 * @param buf - the send buffer
 *
 * @param len - the length of the send buffer
 *
 * @return - ANI_E_IPCSEND upon a failure
 *           Number of bytes sent otherwise
 */
int aniAsfSendMsg(tAniIpc *ipcc, char *buf, int len)
{
	int i, totalBytesSent = 0, sendBytes;
	int lpBit = 0, seqNum = 0;
	char *bufOffset = buf;
	ANI_U32 asfHdr = 0, lpHdr;

	/*
	 * Dump the contents of the send buffer irrespective of the
	 * DEBUG settings. Will help in analysing the packet contents
	 */
	aniAsfLogMsg(LOG_DEBUG, ANI_WHERE, "Send: %s",
			aniAsfHexStr((const u_char *)buf, len));

	// Determine if this is a large packet
	if( len > ANI_SND_SOC_BUF_SIZE )
	{
		lpBit = 1;
		// Cache the ASF and LP headers
		asfHdr = aniAsfGet32( buf );
		lpHdr = aniAsfGet32( buf + ANI_HDR_LEN );
	}

	do
	{
		if( lpBit )
		{
			if((len - (bufOffset - buf)) > ANI_SND_SOC_BUF_SIZE )
				sendBytes = ANI_SND_SOC_BUF_SIZE / 8;
			else
			{
				// Last fragment of the large packet
				lpBit = 0;
				sendBytes = (len - (bufOffset - buf));
			}

			// Copy the ASF header
			aniAsfPut32( bufOffset, asfHdr );
			// Adjust the send buffer length for "this" fragment
			aniAsfPut16( bufOffset + 2, sendBytes );
			// Update the LP header
			lpHdr = aniAsfEncodeLpHdr( &lpHdr, lpBit, seqNum );
			// Copy the LP header
			aniAsfPut32( bufOffset + ANI_HDR_LEN, lpHdr );
		}
		else
			sendBytes = len;

		if((i = aniAsfIpcSend(ipcc, bufOffset, sendBytes)) < 0)
		{
			aniAsfLogMsg(ANI_IPCSEND_ERR);

			// TODO - Need to try to reconnect?
			totalBytesSent = ANI_E_IPCSEND;
			lpBit = 0;
			//break;
		}
		else
		{
			totalBytesSent += i;
			seqNum++; // Next fragment/sequence number

			//
			// Offset the send buffer. Make sure there is enough
			// space left for ANI_HDR + LP_HDR for the next (or
			// last) fragment
			// In this case, CM_HDR_LEN = ANI_HDR_LEN + LP_HDR
			//
			// Just a NOTE - Apart from the very first fragment of
			// a large packet, the rest of them do NOT have the
			// ANI_RESULT_TTLV following the LP header
			//
			bufOffset += (i - (ANI_HDR_LEN + ANI_LP_HDR_LEN));
		}
	}
	while( lpBit );

	if( totalBytesSent != ANI_E_IPCSEND )
	{
		//
		// IMPORTANT NOTE:
		// Adjust totalBytesSent. Since the CM_HDR (ASF + LP HDR)
		// is repeated for every fragment, we need to discount
		// this size when we compute the total length
		// 
		// Effectively, this totalBytesSent should now be equal
		// to the value of "len"!
		//
		// FIXME: Do we sent this altered totalBytesSent or
		// should we send the actual number of bytes sent?
		//
		totalBytesSent -= ((seqNum - 1) * (ANI_HDR_LEN + ANI_LP_HDR_LEN));
	}

	return totalBytesSent;
}
