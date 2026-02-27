/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


#include <stdio.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>

#include "aniAsfHdr.h"
#include "aniAsfIpc.h"
#include "aniErrors.h"
#include "aniUtils.h"
#include "aniAsfLog.h"
#ifndef ANI_AP_SDK
#include "aniNmpSvcMsgs.h"
#endif // !ANI_AP_SDK
#include <sys/socket.h>

#define MAX_BYTES 1500

u_char *
aniAsfHexDump(const u_char *bytes, u_int len)
{
  return aniAsfHexStr(bytes, len);
}

/*
 * Converts a ASCII HEX character to an integer value.
 * Invalid characters should return 0.
 */
static inline char c2h(char c)
{
    if ((c >= '0') && (c <= '9'))
        return (c - '0');
    else if ((c >= 'a') && (c <= 'f'))
        return (c - 'a') + 10;
    else if ((c >= 'A') && (c <= 'F'))
        return (c - 'A') + 10;
    else
        return 0;
}

/**
 * Reads a ":" separated ASCII (Hex) representation of a binary value into 
 * a binary buffer. 
 * 
 * e.g. Reads in 01:02:03:04:15:a6:07:c8:69 into a buffer of length 9.
 * Note that single hex digits must be preceeded by a 0.
 * Further, this routine assumes you are passing in precisely the
 * correct length ascii string...no extra leading or trailing
 * whitespace allowed! 
 *
 * The function will also not overwrite memory beyond the size of
 * the destination buffer.
 *
 * @param str the string to read the the ASCII Hex data from.
 * @param hexData the data buffer to populate with binary data.
 * @param maxBytes the size of the buffer hexData. The function will
 * stop copying bytes once this limit has been reached.
 *
 * @return the number of bytes in the result.
 */
int
aniAsfAscii2Hex(const u_char *str, u_char *hexData, int maxBytes)
{
    int i = 0;
    int len = 0;
    int count =0;
    int tmp = 0;

    /* 
     * This next line assumes you are passing in precisely the correct length string...
     * no extra whitespace allowed 
     */
    len = (strlen((const char *)str) / 3) ; 
    if (len >= maxBytes) {
        // The real length is one greater than len because the last byte
        // is not ":" terminated
        len = maxBytes - 1;
    }

    while (i < len) {
        count = sscanf((const char *)str, "%x:", &tmp);
        //  if (count != 1) break;  /* Leave this check out */
        hexData[i] = tmp;
        str += 3;
        i++;
    }

    // Read the last byte which does not have a trailing ":"
    count = sscanf((const char *)str, "%x", &tmp);
    hexData[i] = tmp;

    return (i+1);
}


/**
 * str can be "00:0a:f5:aa:bb:cc" or "00-0a-f5-aa-bb-cc" or "00.0a.f5.aa.bb.cc"
 * or "00-0a:f5-aa.bb-cc" or "000af5aabbcc".
 */
inline void
aniAsfAscii2MacAddr(const u_char *str, tAniMacAddr macAddr)
{
	char *tok, *holder = NULL, macAddrStr[20] = {0} ;
	int i ;
	char *delimiter = ":.-" ;

	strlcpy(macAddrStr, (const char *)str, sizeof(macAddrStr)) ;

	tok = strtok_r(macAddrStr, delimiter, &holder);

	/* if macAddrStr = "00:03:9c:12:34:56"
	 * then "00\003:9c:12:34:56"
	 *       A   A
	 *       |   |
	 *       |   holder
	 *       tok
	 */
	if ( holder != NULL )
	{
	    for (i = 0 ; i < 6 && tok != NULL ; ++i)
	    {
		macAddr[i] = (unsigned char)strtoul(tok, (char**)NULL, 16) ;
		tok = strtok_r(NULL, delimiter, &holder);
	    }
	}
	else /* macAddrStr = "000af5aabbcc" */
	{
	    for (i = 0 ; i < 6 ; ++i)
	    {
		macAddr[i] = (unsigned char) (c2h(macAddrStr[2*i]) << 4 | c2h(macAddrStr[2*i+1])) ;
	    }
	}

	return ;
}

/**
 * ani_strlen - Wrapper around strlen()
 * @s: The string to be sized
 */
size_t ani_strlen(const char * s)
{
    if (s == NULL) 
        return 0;
    else
        return strlen(s);
}

void *ani_calloc(size_t nmemb, size_t size)
{
    void *ptr=calloc(nmemb, size);

    if (ptr)
        memset(ptr,0,size);
    return ptr;
}

void *ani_memcpy(void * dest, const void *src, size_t n)
{
    /* What to do if "dest" is NULL? */
    if (src == NULL) {
        memset(dest, 0, n);
        return dest;
    }
    return memcpy(dest,src,n);
}

// Each byte will be converted to hex digits followed by a
// punctuation (which is specified in the "delimiter" param. Thus 
// allocate three times the storage.
u_char *
aniAsfHexString(const u_char *bytes, u_int len, char delimiter)
{
  static u_char buf[MAX_BYTES*(2+1)];
  u_int i;
  char *ptr;

  len = ANI_MIN(len, MAX_BYTES);
  for (i = 0, ptr = (char *)buf; i < len; i++) {
    snprintf(ptr, (sizeof(buf) - (ptr - (char *)buf)), "%.2x%c",
             bytes[i], delimiter);
    ptr += 3;
  }

  // Delete the extra punctuation and null terminate the string
  if (len > 0)
      ptr--;
  *ptr = '\0';

  return buf;

}


/*
 * aniAsfBcastSysinitMesg
 *
 * Using the evNotifier as the broadcast medium, this API
 * will broadcast "sysinit" related messages. The recipient
 * of this broadcast is any service that has registered with
 * evNotifier
 * Currently, the following sysinit related messages can be
 * broadcast: -
 * eANI_BCAST_SYSREADY
 * eANI_BCAST_SYSCONFIG_START
 * eANI_BCAST_SYSCONFIG_COMPLETE
 * eANI_BCAST_SYSCONFIG_DONE
 * 
 * NOTE: This API makes the following assumptions -
 * 1) This API will always send a fixed size message (8 bytes).
 * This is not a generic API to send "any" broadcast message via
 * the evNotifier
 * 2) This API will always send the sysinit broadcast to the
 * evNotifier running on the "localhost"
 *
 * @param mesgId - BCAST sysinit message id to send
 *
 * @param param - The 4-byte PROG_ID associated with the
 * sysinit message
 *
 * @param return - An actual error value encountered
 *
 */
int aniAsfBcastSysinitMesg( ANI_U16 nMesgId, ANI_U32 nProgId )
{
tAniIpc *ipcc = NULL;
char sendBuf[8];
char *bufOffset = sendBuf;
int retVal = ANI_OK;

	memset( sendBuf, 0, 8 );
	// Add the send buffer packet Header
	bufOffset = aniAsfPut16( bufOffset, nMesgId );
	bufOffset = aniAsfPut16( bufOffset, 8 );
	// Add 4-byte message parameter - Program ID
	bufOffset = aniAsfPut32( bufOffset, nProgId );

	// Initiate the client connection to evNotifier
	if( NULL == (ipcc = aniAsfIpcOpen( PF_INET,
					SOCK_DGRAM,
					0 )))
		retVal = ANI_E_IPCOPEN;
	else
	{
		if( aniAsfIpcConnect( ipcc,
					"localhost",
					EV_NOTIFIER_PROG_ID,
					ANI_AP_VERS_ID ) < 0 )
			retVal = ANI_E_IPCCONNECT;
		else
		{
			if( aniAsfIpcSend( ipcc, sendBuf, (bufOffset - sendBuf)) < 0 )
				retVal = ANI_E_IPCSEND;
		}
	}

	if( NULL != ipcc )
		aniAsfIpcClose( ipcc );

	return retVal;
}

#ifndef ANI_AP_SDK
/*
 * aniApIsNmPortal() 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is a NM Portal, otherwise returns eANI_BOOLEAN_FALSE
 */

tAniBoolean aniApIsNmPortal(void)
{
	struct stat buf;

  	if (stat(NMP_NMPORTAL_CONF_FILE, &buf))	// file doesn't exist
	  	return eANI_BOOLEAN_FALSE;
	else
	  	return eANI_BOOLEAN_TRUE;
}

/*
 * aniApIsSecPortal() 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is a SEC Portal, otherwise returns eANI_BOOLEAN_FALSE
 */

tAniBoolean aniApIsSecPortal(void)
{
	struct stat buf;

  	if (stat(NMP_SEC_PORTAL_CONF_FILE, &buf))	// file doesn't exist
	  	return eANI_BOOLEAN_FALSE;
	else
	  	return eANI_BOOLEAN_TRUE;
}

/*
 * aniApIsEnrollmentPortal() 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is an enrollment Portal, otherwise returns eANI_BOOLEAN_FALSE
 */

tAniBoolean aniApIsEnrollmentPortal(void)
{
	struct stat buf;

  	if (stat(NMP_ENROLLMENT_PORTAL_CONF_FILE, &buf))	// file doesn't exist
	  	return eANI_BOOLEAN_FALSE;
	else
	  	return eANI_BOOLEAN_TRUE;
}

/*
 * aniApIsMobileAgent 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is a mobile agent, otherwise returns eANI_BOOLEAN_FALSE
 */

tAniBoolean aniApIsMobileAgent(void)
{
	struct stat buf;

  	if (stat(ANI_MA_FLAG_FILE, &buf))	// file doesn't exist
	  	return eANI_BOOLEAN_FALSE;
	else
	  	return eANI_BOOLEAN_TRUE;
}

/*
 * aniInvalidateApMobileAgent 
 *
 * makes the AP not a mobile-agent
 */
void aniInvalidateApMobileAgent(void)
{
  	unlink(ANI_MA_FLAG_FILE);
}

/*
 * aniMakeApMobileAgent 
 *
 * makes the AP a mobile agent.
 *
 * @returns ANI_OK or ANI_E_FAILED
 */
int aniMakeApMobileAgent(void)
{
	FILE *fp;

  	fp = fopen(ANI_MA_FLAG_FILE, "w");

	if (fp == NULL) {
		aniAsfLogMsg(LOG_ERR, ANI_WHERE, "Can't create flag file %s: %s", ANI_MA_FLAG_FILE, strerror(errno));
		return ANI_E_FAILED;
	}

	fclose(fp);

	return ANI_OK;
}
#endif // !ANI_AP_SDK
