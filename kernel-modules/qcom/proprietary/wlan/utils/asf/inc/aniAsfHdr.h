/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * This file aniAsfEvent.h is for the Event Manager (Function declarations)
 * Author:  U. Loganathan
 * Date:    Jul 1st 2002
 * History:-
 * Date     Modified by Modification Information
 *
 */

#ifndef _ANI_ASF_HDR_H_
#define _ANI_ASF_HDR_H_

#ifdef __KERNEL__
#include <asm/byteorder.h>
#else // __KERNEL__ 
#include <netinet/in.h>
#endif // __KERNEL__
#include "aniTypes.h"
#include "aniResultCodes.h"
#include "aniAsfIpc.h"

/*
 * Below constants are defined as per android_filesystem_config.h
 * to resolve dependency issue of vendor modules on system modules.
 */
#define AID_WIFI 1010            /* wifi subsystem */
#define AID_MEDIA_RW 1023        /* internal media storage write access */
#define AID_SDCARD_R 1028        /* external storage read access */
#define AID_DIAG 2002  /* access to diagnostic resources */
#define AID_INET 3003         /* can create AF_INET and AF_INET6 sockets */
#define AID_NET_ADMIN 3005    /* can configure interfaces and routing tables. */

/*
 * This file will have to be included with all applications and also
 * by modules which communicate with the AP applications
 */

#define ANI_AP_VERS_ID      1001

/**
 * The program ID's used to register with PortMap
 */

#define ANI_PROG_ID_START	5000000

typedef enum eAniProgIds {
	ANI_BCAST_PROG_ID = ANI_PROG_ID_START,

	ANI_PMAP_PROG_ID,
	ANI_WSM_PROG_ID,
	ANI_AUTH_PROG_ID,
	ANI_IAPP_PROG_ID,
	ANI_DISC_PROG_ID,
	ANI_EZC_PROG_ID,
	ANI_DBM_PROG_ID,
	ANI_NSM_PROG_ID,
	EV_NOTIFIER_PROG_ID,
	ANI_CM_PROG_ID,
	ANI_LSM_PROG_ID,
	ANI_SWD_PROG_ID,
	ANI_DDS_PROG_ID,
	ANI_PRM_PROG_ID,
	ANI_NMP_PROG_ID,
	ANI_FM_PROG_ID, /* FM server's registered port */
	ANI_UM_PROG_ID, /* UM server registered port */
	ANI_PM_PROG_ID, /* Performance Manager registered port */
	ANI_NMP_DBS_PROG_ID, /* Copy Server registered port */
	ANI_RF_STATS_PROG_ID, /* RF Stats Server registered port */
	ANI_GPM_PROG_ID, /* Guest portal IPC program ID */
	ANI_GPM_REDIR_PROG_ID, /* Guest portal TCP/HTTP redirect prog ID */

    // Test programs and temporary servers
	ANI_NETSIM_SERV_PROG_ID,
	ANI_AA_NSIM_R0_PROG_ID,
	ANI_AA_NSIM_R1_PROG_ID,
    ANI_GENERIC_TEST_PROG_ID,
    ANI_GENERIC_TEST_PROG_ID_2,
	ANI_EVNOTIFY_PROG_ID,

	ANI_PROG_ID_END
} tAniProgIds;

/**
 * The version ID used to register with PortMap for each service
 */
#define ANI_DISC_PROG_VER_ID    ANI_AP_VERS_ID
#define ANI_EZC_PROG_VER_ID     1
#define ANI_FM_VERS_ID          ANI_AP_VERS_ID
#define ANI_UM_VERS_ID          ANI_AP_VERS_ID
#define ANI_PM_PROG_VER_ID      ANI_AP_VERS_ID
#define ANI_NMP_DBS_VERS_ID     ANI_AP_VERS_ID
#define ANI_RF_STATS_VERS_ID    ANI_AP_VERS_ID

#ifdef ANI_AP_SDK
#define ANI_WSM_PROG_PORT	5001
#define ANI_AUTH_PROG_PORT	5002
#endif /* ANI_AP_SDK */

/**
 * All Woodside Messages will start this as its header
 */
typedef struct sAniHdr {

    // Two byte wnihdr Message type
    ANI_U16     type;

    // Message length
    ANI_U16     length;

} tAniHdr;

/**
 * Definition for the TLV's
 */
typedef struct sAniParameter
{
    // Parameter Type
    ANI_U16     pType;

    // Parameter Length
    ANI_U16     pLength;

    // Parameter Data
    ANI_U32     *value;

} tAniParameter;

#define ANI_HDR_LEN sizeof(tAniHdr)
#define ANI_LP_HDR_LEN 4

/*
 * Currently defined modules are
 * COMMON(Portmap, Log), NM, WSM, AA, IAPP, NSM, DRIVER
 */

/* Message type definition - Common Pool  0x0001 - 0x00FF */
/* Driver messages 0x0001 - 0x000F */
#define ANI_DRIVER_MSG_START        0x0001 

#define ANI_MSG_APP_REG_REQ     (ANI_DRIVER_MSG_START + 0)
#define ANI_MSG_APP_REG_RSP     (ANI_DRIVER_MSG_START + 1)
#define ANI_MSG_EAPOL           (ANI_DRIVER_MSG_START + 2)
#define ANI_MSG_LINK_UP           (ANI_DRIVER_MSG_START + 3)
#define ANI_MSG_LINK_DOWN           (ANI_DRIVER_MSG_START + 4)


/* Driver messages 0x0010 - 0x001F */
#define ANI_NETSIM_MSG_START		0x0010

/* wni portmap messages  0x0020 - 0x002F */
#define ANI_PORTMAP_MSG_START		0x0020

/* Log messages 0x0030 - 0x003F */
#define ANI_LOG_MSG_START		0x0030
#define ANI_LOG_SET_LOG_LEVEL		0x0031
#define ANI_LOG_GET_LOG_LEVEL		0x0032

/* General messages 0x0030 - 0x003F */
#define ANI_SRV_SHUTDOWN		0x0040

/* Message between different modules */
/* NM <---> WSM mesages */
#define ANI_NM_WSM_MSG_START		0x0100
#define ANI_NM_WSM_MSG_START_BLK1	0x0120
#define ANI_NM_WSM_MSG_END		0x02ff

/* NM <---> IAPP mesages */
#define ANI_NM_IAPP_MSG_START		0x0300

/* NM <---> NSM mesages */
#define ANI_NM_NSM_MSG_START		0x0400
#define ANI_NM_BRIDGE_MSG_START     ANI_NM_NSM_MSG_START    /* deprecated */

/* NM <---> DRIVER mesages */
#define ANI_NM_DRIVER_MSG_START		0x0500

/* WSM <---> AA mesages */
#define ANI_WSM_AA_MSG_START		0x0600

/* WSM <---> IAPP mesages */
#define ANI_WSM_IAPP_MSG_START		0x0700

/* WSM <---> NSM mesages */
#define ANI_WSM_NSM_MSG_START		0x0800
#define ANI_WSM_BRIDGE_MSG_START    ANI_WSM_NSM_MSG_START   /* deprecated */

/* WSM <---> DRIVER mesages */
#define ANI_WSM_DRIVER_MSG_START	0x0900

/* AA <---> IAPP mesages */
#define ANI_AA_IAPP_MSG_START		0x0A00

/* AA <---> NSM mesages */
#define ANI_AA_NSM_MSG_START		0x0B00
#define ANI_AA_BRIDGE_MSG_START     ANI_AA_NSM_MSG_START

/* AA <---> DRIVER mesages */
#define ANI_AA_DRIVER_MSG_START		0x0C00

/* IAPP <---> NSM mesages */
#define ANI_IAPP_NSM_MSG_START		0x0D00
#define ANI_IAPP_BRIDGE_MSG_START   ANI_IAPP_NSM_MSG_START

/* IAPP <---> DRIVER mesages */
#define ANI_IAPP_DRIVER_MSG_START	0x0E00

/* NSM <---> DRIVER mesages */
#define ANI_NSM_DRIVER_MSG_START	0x0F00
#define ANI_BRIDGE_DRIVER_MSG_START ANI_NSM_DRIVER_MSG_START

/* Config messages 0x1100 - 0x11FF */
#define ANI_HOST_MSG_START		0x1100

/* Station Management messages 0x1200 - 0x12FF */
#define ANI_SME_MSG_START		0x1200

/* Network Services messages 0x1300 - 0x13FF */
#define ANI_NS_MSG_START		0x1300

/* NMPortal Discovery Server Messages 0x1400 - 0x14FF */
#define ANI_NMP_DISC_MSG_START		0x1400

/* DBM library Messages 0x1500 - 0x15FF */
#define ANI_DBM_MSG_START		0x1500

/* Global Broadcast Messages */
#define ANI_BCAST_MSG_START             0x1600

/* NMPortal Easy Config (EZC) Messages */
#define ANI_NMP_EZC_MSG_START		0x1700

/* Messages for services provided by NSM */
#define ANI_NSM_MSG_START		0x1800
#define ANI_NSM_MSG_BLK1_START		0x18a0

/* Messages for services provided by WSM */
#define ANI_WSM_SERVICES_START		0x1900

/* Messages for Config services provided by AA/SSM */
#define ANI_NM_SSM_MSG_START            0x1a00

/* Messages for Legacy AA/SSM Config services provided by CM(?) */
#define ANI_NM_SSM_LEGACY_MSG_START     0x1b00

/* Messages for Legacy dhcp, system, snmp config services provided by CM(?) */
#define ANI_NM_LSM_MSG_START		0x1c00

/* Messages for Software Download services */
#define ANI_SWD_MSG_START		0x1d00

/* Messages for Policy Registry */
#define ANI_PRM_MSG_START		0x1e00

/* Messages for Data Distribution Services (DDS) */
#define ANI_DDS_MSG_START		0x1f00

/* Messages for Fault Manager */
#define ANI_FM_MSG_START		0x2000

/* Messages for User Manager */
#define ANI_UM_MSG_START		0x2100

/* Messages for Configuration Manager (CM) */
#define ANI_CM_MSG_START		0x2200

/* Messages for NMPortal */
#define ANI_NMP_MSG_START		0x2300

/* Messages for Copy Server */
#define ANI_NMP_DBS_MSG_START  0x2400

/* NM <---> PRM mesages */
#define ANI_NM_PRM_MSG_START		0x2500

/* PM mesages */
#define ANI_PM_MSG_START		0x2600

/* Messages for RFS Manager */
#define ANI_RFS_MSG_START		0x2700

/* Message for GPM */
#define ANI_GPM_MSG_START		0x2800

/* Message for SSM libraries to send to the AA - the only real SSM process*/
#define ANI_SSM_AA_MSG_START		0x2900

#define ANI_UPNP_WSM_MSG_START	0x2a00

/**
 * Some of the common meesages
 * Change log level
 */
typedef struct tAniLogLevel {
    ANI_U16     messageType;
    ANI_U16     length;
    // To change the log level
    ANI_U32     loglevel;
}tAniLogLevel;

typedef enum eAniLogType {

    ANI_CHANGE_LOG_LEVEL = ANI_LOG_MSG_START

}tAniLogType;

/* Prototypes */
extern ANI_U32 aniAsfGet32(char *);
extern char *aniAsfPut32(char *, ANI_U32);
extern ANI_U16 aniAsfGet16(char *);
extern char *aniAsfPut16(char *, ANI_U16);
extern int aniAsfEncodeLpHdr( ANI_U32 *, int, int );
extern int aniAsfSendMsg(tAniIpc *ipcc, char *buf, int len);

extern void aniAsfNtohHdr(char *);
extern void aniAsfHtonWniHdr(tAniHdr *);

/* Static in-line type conversion functions */

typedef union uAniU32ValAry{
	ANI_U32 val;
	char ary[sizeof(ANI_U32)];
} tAniU32ValAry;

typedef union uAniU16ValAry{
	ANI_U16 val;
	char ary[sizeof(ANI_U16)];
} tAniU16ValAry;

/*
 * Machine-independent, alignment insensitive network-to-host long
 * conversion.
 * 
 */

static __inline__ ANI_U32 aniAsfRd32(char *cp)
{
	tAniU32ValAry r;
	int i;

	i = sizeof(ANI_U32) - 1;
	r.ary[i--] = cp[3];
	r.ary[i--] = cp[2];
	r.ary[i--] = cp[1];
	r.ary[i] = cp[0];

    return (ntohl(r.val));
}

/*
 * Put a long in host order into a char array in network order. 
 * 
 */
static __inline__ char *aniAsfWr32(char *cp, ANI_U32 x)
{
	tAniU32ValAry r;
	int i;

	r.val = htonl(x);
	i = sizeof(ANI_U32) - 1;
	cp[3] = r.ary[i--];
	cp[2] = r.ary[i--];
	cp[1] = r.ary[i--];
	cp[0] = r.ary[i];

    return (cp + sizeof(ANI_U32));
}

static __inline__ ANI_U16 aniAsfRd16(char *cp)
{
	tAniU16ValAry  r;
	int i;

	i = sizeof(ANI_U16) - 1;
	r.ary[i--] = cp[1];
	r.ary[i] = cp[0];

	return(ntohs(r.val));
}

/* Put a short in host order into a char array in network order */
static __inline__ char *aniAsfWr16(char *cp, ANI_U16 x)
{
	tAniU16ValAry r;
	int i;

	r.val = htons(x);
	i = sizeof(ANI_U16) - 1;
	cp[1] = r.ary[i--];
	cp[0] = r.ary[i];

    return (cp + sizeof(ANI_U16));
}

static __inline__ void aniAsfNtohAniHdr(char *start)
{
    tAniU16ValAry r;
	int i;

	// Convert the type field.
    r.val = aniAsfRd16(start);
	i = sizeof(ANI_U16) - 1;
	start[1] = r.ary[i--];
	start[0] = r.ary[i];

	// Convert the length field.
    r.val = aniAsfRd16(&start[sizeof(ANI_U16)]);
	i = sizeof(ANI_U16) - 1;
	start[sizeof(ANI_U16) + 1] = r.ary[i--];
	start[sizeof(ANI_U16) + 0] = r.ary[i];
}

static __inline__ void aniAsfHtonAniHdr(tAniHdr *hdr)
{
    aniAsfWr16((char *) &hdr->type, hdr->type);
    aniAsfWr16((char *) &hdr->length, hdr->length);
}

#endif /* _ANI_ASF_HDR_H_ */
