/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * File: $File: //depot/software/projects/feature_branches/gen5_phase1/os/linux/classic/ap/apps/include/aniNmpSvcMsgs.h $
 *
 * Contains interface messages supported by EZC server.
 *
 * Author:			Andy Phan
 * Date:			11/02/02
 * History:-
 * Date				Modified by		 Modification Information
 * ------------------------------------------------------
 *
 */
#ifndef _ANI_NMP_SVC_MSGS_H_
#define _ANI_NMP_SVC_MSGS_H_

#include <aniTypes.h>
#include <aniAsfHdr.h>
#include <aniAsfLog.h>

#define NMP_AP_START_UP_CFG_FILE_NAME "apconfig.xml"
#define NMP_AP_START_UP_CFG_PATH_N_FILE_NAME  ANI_DYN_CONFIG_DIR "startup/" NMP_AP_START_UP_CFG_FILE_NAME

#define NMP_DEFAULT_SUPPORT_LOGS_FILE_NAME "supportLogs.tar.gz"

#define NMP_AP_SYSTEM_VERSION_INFO_FILE_NAME "version.xml"
#define NMP_AP_SYSTEM_VERSION_INFO_PATH_N_FILE_NAME  ANI_DYN_CONFIG_DIR NMP_AP_SYSTEM_VERSION_INFO_FILE_NAME

#define NMP_NM_BACKUP_FILES_SCRIPT  "/etc/init.d/aniBackupFiles"
#define NMP_NM_BACKUP_ARG_SYS_LOGS  "sysLogs"
#define NMP_NM_BACKUP_ARG_SUPPORT_LOGS  "supportLogs"

#define NMP_AP_SYSLOG_BACKUP_FILE_NAME "syslog.tar.gz"
#define NMP_AP_SYSLOG_BACKUP_PATH_N_FILE_NAME  "/tmp/" NMP_AP_SYSLOG_BACKUP_FILE_NAME

#define NMP_AP_PORTAL_BACKUP_FILE_NAME "portalCfg.tar.gz"
#define NMP_AP_PORTAL_BACKUP_PATH_N_FILE_NAME  "/tmp/" NMP_AP_PORTAL_BACKUP_FILE_NAME
#define NMP_AP_POLICY_DB_DIR  ANI_DYN_CONFIG_DIR "policyDb/"

#define NMP_AP_BOOTSTRAP_POLICY_NAME  "defaultpolicy.xml"
#define NMP_AP_BOOTSTRAP_POLICY_FILE  NMP_AP_POLICY_DB_DIR NMP_AP_BOOTSTRAP_POLICY_NAME

#define NMP_CONFIG_DIR ANI_DYN_CONFIG_DIR "nmportal/"

// This flag file indicates whether the AP is a NMPortal AP
#define NMP_NMPORTAL_CONF_FILE NMP_CONFIG_DIR "nmportal.conf"

// This flag file indicates whether the AP is enrolled
#define NMP_AP_INIT_CONFIG_FILE NMP_CONFIG_DIR "enabled_ap.conf"

// This flag file indicates whether the AP is a SEC Portal
#define NMP_SEC_PORTAL_CONF_FILE NMP_CONFIG_DIR "secp.conf"

// This flag file indicates whether the AP is the enrollment Portal
#define NMP_ENROLLMENT_PORTAL_CONF_FILE NMP_CONFIG_DIR "enrollment.conf"

// This file contains the master agent data
#define NMP_MA_DATA_FILE NMP_CONFIG_DIR "ma.dat"

// This flag file indicates mobility service is enable
#define NMP_MOBILITY_ENABLE_CONF_FILE NMP_CONFIG_DIR "mobility.conf"

#define NMP_DEVICE_ID_LEN 80

#define NMP_MAX_COLUMN_SIZE 5120
#define NMP_MAX_PORTAL_DIST_COLUMN_SIZE 600

#define NMP_DIST_FLAG_REBOOT                   1
#define NMP_DIST_FLAG_SAVE_TO_STARTUP          2
#define NMP_DIST_FLAG_SAVE_RUNNING_CONFIG      4
#define NMP_DIST_FLAG_APPLY_TO_SELF            8
//
// EZC Enumerated Service Message Types
//
typedef enum eAniNmpSvcMessage {

		// Used by Discover Server to do SSL hand-shake
		eANI_NMP_GET_ENROLL_THUMB_PRINT_REQ = ANI_NMP_MSG_START + 200,
		eANI_NMP_GET_ENROLL_THUMB_PRINT_RSP,
		eANI_NMP_NMS_DISTRIBUT_POLICY_REQ,
		eANI_NMP_NMS_DISTRIBUT_POLICY_RSP,
		eANI_NMP_UPDATE_ENROLLMENT_INFO_REQ,
		eANI_NMP_UPDATE_ENROLLMENT_INFO_RSP,
		eANI_NMP_IP_CHANGE_NOTIF_RETRY_REQ,
		eANI_NMP_IP_CHANGE_NOTIF_RETRY_RSP

} tAniNmpSvcMessage;

// Subsystems:		DDS ==> NMP
// Request Type:	eANI_NMP_UPDATE_ENROLLMENT_INFO_REQ 
typedef struct sAniNmpUpdateEnrollmentInfoReq {
		tAniHdr	aniHdr;
		ANI_U32	ipAddress;	// IP address of the device to be updated
		char deviceIdStr[NMP_DEVICE_ID_LEN];
} ANI_PACK_STRUC tAniNmpUpdateEnrollmentInfoReq;

// Subsystems:		NMP ==> DDS
// Request Type:	eANI_NMP_UPDATE_ENROLLMENT_INFO_RSP
typedef struct sAniNmpUpdateEnrollmentInfoRsp {
		tAniHdr	aniHdr;
		ANI_U32	status;
} ANI_PACK_STRUC tAniNmpUpdateEnrollmentInfoRsp;


// Subsystems:		DISC ==> NMP
// Request Type:	eANI_NMP_GET_ENROLL_THUMB_PRINT_REQ 
typedef struct sAniNmpGetEnrollThumbPrintReq {
		tAniHdr	aniHdr;
		ANI_U32	ipAddress;	// IP address in 32 bit format, e.g.: 0xc0a84ba6
		char primaryKey[NMP_DEVICE_ID_LEN];
} ANI_PACK_STRUC tAniNmpGetEnrollThumbPrintReq;

// Subsystems:	NMP ==> DISC 
// Response Type:	eANI_NMP_GET_ENROLL_THUMB_PRINT_RSP 
typedef struct sAniNmpGetEnrollThumbPrintRsp {
		tAniHdr	aniHdr;
		ANI_U32	status;
} ANI_PACK_STRUC tAniNmpGetEnrollThumbPrintRsp;

// Subsystems:	NMP ==> Webui Policy plugin 
// Response Type:	eANI_NMP_NMS_DISTRIBUT_POLICY_RSP
typedef struct sAniNmpNmsDistPolicyRsp {
		tAniHdr	aniHdr;
		ANI_U32	status;
} ANI_PACK_STRUC tAniNmpNmsDistPolicyRsp;

#endif /* _ANI_NMP_SVC_MSGS_H_ */

