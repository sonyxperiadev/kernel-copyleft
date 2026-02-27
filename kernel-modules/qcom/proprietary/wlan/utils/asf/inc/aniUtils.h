/*
 * Copyright (c) 2013, 2016 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


#ifndef _ANI_UTILS_H_
#define _ANI_UTILS_H_

#include <sys/types.h>
#include <aniTypes.h>

#define ANI_SECURE_DOWNLOAD_FILE_PATH "/ani/runconfig/securedownload/"
#define ANI_STATIC_CONFIG_DIR "/ani/config/static/"
#define ANI_DYN_CONFIG_DIR "/ani/config/dynamic/"
#define ANI_TMP_DIR "/ani/runconfig/dynamic/"

#define STARTUP_CONFIG_DIR_FLASH ANI_DYN_CONFIG_DIR"startup/"
#define STARTUP_CONFIG_FILE_FLASH STARTUP_CONFIG_DIR_FLASH"apconfig.xml"

#define ANI_PRISTINE_FLAG_FILE	ANI_DYN_CONFIG_DIR "pristine.conf"
#define ANI_MA_FLAG_FILE	ANI_DYN_CONFIG_DIR "ma.conf"
#define ANI_MGMT_IP_ADDR_FILE	ANI_DYN_CONFIG_DIR "ipAddr"

#define ANI_MIN(x, y) ((x) < (y) ? (x) : (y))
#define ANI_MAX(x, y) ((x) > (y) ? (x) : (y))
#define ANI_CHECK_RANGE(lower, x , upper) \
                ( ((lower) <= (x)) && ((x) <= (upper)) )

/**
 * Converts a sequence of arbitrary octets to an ascii printable null
 * terminated hexadecimal representation (e.g.,
 * "00-0d-4e-12-97-20"). The application should copy the returned
 * string to its own storage. The application can determine the delimiter char.
 */
u_char *
aniAsfHexString(const u_char *buf, u_int len, char delimiter);

/**
 * Converts a sequence of arbitrary octets to an ascii printable null
 * terminated hexadecimal representation (e.g.,
 * "00:0d:4e:12:97:20"). The application should copy the returned
 * string to its own storage.
 */
inline u_char *
aniAsfHexStr(const u_char *buf, u_int len)
{
  return aniAsfHexString(buf, len, ':');
}

/**
 * Converts a sequence of arbitrary octests to stdout in a hex
 * representation as well as ASCII representation.
 */
u_char *
aniAsfHexDump(const u_char *buf, u_int len);

/**
 * Reads a ":" separated ASCII MAC address from a string into a 6 byte
 * tAniMacAddr structure. The string should contain the MAC as shown in
 * this example: "00:A:12:12:32:00". Note that single hex digits need
 * not be preceeded by a 0.
 *
 * @param str the string to read the the ASCII MAC address from.
 * @param macAddr the tAniMacAddr that should be populated
 */
inline void
aniAsfAscii2MacAddr(const u_char *str, tAniMacAddr macAddr);

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
 * @return numBytes the number of bytes in the result.
 */
int
aniAsfAscii2Hex(const u_char *str, u_char *hexData, int maxBytes);


size_t ani_strlen(const char * s);

void *ani_calloc(size_t nmemb, size_t size);

void *ani_memcpy(void * dest, const void *src, size_t n);

int aniAsfBcastSysinitMesg( ANI_U16 , ANI_U32 );

/*
 * aniApIsNmPortal() 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is a NM Portal, otherwise returns eANI_BOOLEAN_FALSE
 */

tAniBoolean aniApIsNmPortal(void);

/*
 * aniApIsSecPortal() 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is a SEC Portal, otherwise returns eANI_BOOLEAN_FALSE
 */

tAniBoolean aniApIsSecPortal(void);

/*
 * aniApIsEnrollmentPortal() 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is an enrollment Portal, otherwise returns eANI_BOOLEAN_FALSE
 */

tAniBoolean aniApIsEnrollmentPortal(void);

/*
 * aniApIsMobileAgent 
 *
 * @returns eANI_BOOLEAN_TRUE if AP is a mobile agent, otherwise returns eANI_BOOLEAN_FALSE
 */
tAniBoolean aniApIsMobileAgent(void);

/*
 * aniInvalidateApMobileAgent 
 *
 * makes the AP NOT a mobile agent
 */
void aniInvalidateApMobileAgent(void);

/*
 * aniMakeApMobileAgent 
 *
 * makes the AP a mobile agent
 *
 * returns ANI_OK or ANI_E_FAILED
 */
int aniMakeApMobileAgent(void);

#endif //_ANI_UTILS_H_
