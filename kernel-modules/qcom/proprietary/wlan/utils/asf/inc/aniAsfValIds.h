/*
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 *
 */


/*
 * File: $Header: aniAsfParmTypes.h $
 *
 * Contents: Contains NMS Messaging definitions
 * Author:      Joseph L Raja
 * Date:        Sept 20, 2002
 * History:
 * Date         Modified by     Modification Information
 * ------------------------------------------------------
 *
 */

#ifndef __ANI_ASF_PARAM_TYPES_H_
#define __ANI_ASF_PARAM_TYPES_H_

/**
 * Macros to construct & extract Data Type and Value Identity from
 * a U32 Parameter Type (PT).
 * DT is U16, Val_Id is U16.
 */

/* Get DT from PT */
#define ANI_GET_DT(x)          ( ((x) & (0xFFFF0000)) >> 16)

/* Get Val ID from PT */
#define ANI_GET_VAL_ID(x)      ( (x) & (0xFFFF) )

/* Assign DT to PT */
#define ANI_SET_DT(x,typ)      ( ((typ) << 16) | (x) ) 

/* Set the ID in a PT */
#define ANI_SET_VAL_ID(x,val_id)  ( ((x) & (0xFFFF0000)) | (val_id) )

/* Make a PT from DT and Val ID */
#define ANI_MK_PT(typ,val_id)  ( ( ((typ) & 0xFFFF) << 16) | ((val_id) & 0xFFFF) )

/* The Data Types supported by the Messages */
#define ANI_TYPE_NONE       0x00    /* Data Type not defined            */
#define ANI_TYPE_U8         0x01    /* Unsigned 8 bit integer           */
#define ANI_TYPE_U16        0x02    /* Unsigned 16 bit integer          */
#define ANI_TYPE_U32        0x03    /* Unsigned 32 bit integer          */
#define ANI_TYPE_U64        0x04    /* Unsigned 64 bit integer          */
#define ANI_TYPE_S8         0x05    /* Signed 8 bit - same as character */
#define ANI_TYPE_S16        0x06    /* Signed 16 bit integer            */
#define ANI_TYPE_S32        0x07    /* Signed 32 bit integer            */
#define ANI_TYPE_STR        0x08    /* Null terminated String           */
#define ANI_TYPE_FLOAT      0x09    /* Single Precision Floating point  */
#define ANI_TYPE_DOUBLE     0x0A    /* Double Precision Floating Point  */
#define ANI_TYPE_IPADDR     0x0B    /* IP Address                       */
#define ANI_TYPE_MACADDR    0X0C    /* Medium Access Control Address    */
#define ANI_TYPE_FXD_DEC    0x0D    /* Fixed Point decimal              */
#define ANI_TYPE_DOMAIN_NAME 0x0E   /* Null terminated fully qualified
                                       Domain name string.              */
#define ANI_TYPE_HEX_STRING 0x0F    /* Null terminated Hex String       */
#define ANI_TYPE_N_U32      0x10    /* Array of U32                     */
#define ANI_TYPE_N_MACADDR  0x11    /* Array of MAC addresses           */
#define ANI_TYPE_N_STRING   0x12    /* Array of Null terminated strings */
#define ANI_TYPE_N_IPADDR   0x13    /* Array of IP Addresses            */
#define ANI_TYPE_N_DOMAIN_NAME  0x14    /* Array of Null terminted domain 
                                           names                        */
#define ANI_TYPE_N_PAR_ERR  0x15    /* Array of Parameter Error codes   */
#define ANI_TYPE_IP_PFX_V4  0x16    /* IPv4 address & num of mask  bits */
#define ANI_TYPE_IP_PRES_V4 0x17    /* IPv4 addr in presentation format
                                       i.e. A.B.C.D\0                   */
#define ANI_TYPE_RECORD     0x18    /* A related set of TTLVs called a
									   Record */

#define ANI_WSC_METHOD     0x19  /* WSC Method type, either PIN or PBC */



/**
 *
 * Every Subsystem is given a range of 512 parameters.
 * The Value Identifier ranges for each subsystem is defined below.
 * The Parameter type that gets sent out in the TTLV is composed as
 * follows:  PT = ANI_MK_PT(Data Type, Value-identity).
 *
 */

/* Reserved Parameter Identites 0x00-0x1FF = 512 */
#define ANI_VIDS_START  0

#define ANI_VID_COOKIE      (ANI_VIDS_START)
#define ANI_VID_ERR_CODE    (ANI_VIDS_START + 1)
#define ANI_VID_ERR_FIELD   (ANI_VIDS_START + 2)
#define ANI_VID_SENDER      (ANI_VIDS_START + 3)
#define ANI_VID_INDEX       (ANI_VIDS_START + 4)
#define ANI_VID_COUNT       (ANI_VIDS_START + 5)
#define ANI_VID_RESULT_CODE (ANI_VIDS_START + 6)
#define ANI_VID_SEPARATOR   (ANI_VIDS_START + 7)
#define ANI_VID_LOG_LEVEL   (ANI_VIDS_START + 8)

/* Parameter Types: For the Reserved Parameter Identities  */
#define ANI_COOKIE          ANI_MK_PT(ANI_TYPE_U16, ANI_VID_COOKIE)
#define ANI_ERR_CODE        ANI_MK_PT(ANI_TYPE_U16, ANI_VID_ERR_CODE)
#define ANI_ERR_FIELD       ANI_MK_PT(ANI_TYPE_U16, ANI_VID_ERR_FIELD)
#define ANI_SENDER          ANI_MK_PT(ANI_TYPE_U16, ANI_VID_SENDER)
#define ANI_INDEX           ANI_MK_PT(ANI_TYPE_U16, ANI_VID_INDEX)
#define ANI_COUNT           ANI_MK_PT(ANI_TYPE_U16, ANI_VID_COUNT)
#define ANI_RESULT_CODE     ANI_MK_PT(ANI_TYPE_S32, ANI_VID_RESULT_CODE)
#define ANI_SEPARATOR       ANI_MK_PT(ANI_TYPE_NONE, ANI_VID_SEPARATOR)
#define ANI_LOG_LEVEL       ANI_MK_PT(ANI_TYPE_U16, ANI_VID_LOG_LEVEL)

/* Value Identity Ranges for WSM 0x200 - 0x3FF = 512 */
#define ANI_VAL_ID_WSM_START    0x200

/* Value Identity Ranges for SSM 0x400 - 0x5FF = 512 */
#define ANI_VAL_ID_SSM_START    0x400

/* Value Identity Ranges for NSM 0x600 - 0x7FF = 512 */
#define ANI_VAL_ID_NSM_START         0x600
#define ANI_VAL_ID_NSM_BLK1_START    0x780

/* Value Identity Ranges for DISCOVERY 0x800 - 0x9FF = 512 */
#define ANI_VAL_ID_DISC_START   0x800

/* Value Identity Ranges for NMPortal 0x1000 - 0x11FF = 512 */
#define ANI_VAL_ID_NMP_START    0x1000

/* Value Identity Ranges for EZ Config Server 0x1200 - 0x13FF = 512 */
#define ANI_VAL_ID_EZCFG_START  0x1200

/* Value Identity Ranges for AA/SSM 0x1400 - 0x15FF = 512 */
#define ANI_VAL_ID_SSM_CLI_START    0x1400

/* Value Identity Ranges for AA/SSM 0x1600 - 0x17FF = 512 */
#define ANI_VAL_ID_SSM_LEGACY_CLI_START 0x1600

/* Value Identity Ranges for SWD Server 0x1800 - 0x19FF = 512 */
#define ANI_VAL_ID_SWD_START    0x1800

/* Value Identity Ranges for DDS Server 0x2000 - 0x21FF = 512 */
#define ANI_VAL_ID_DDS_START    0x2000

/* Value Identity Ranges for FM server 0x2200 - 0x23FF = 512 */
#define ANI_VAL_ID_FAULT_START  0x2200

/* Value Identity Ranges for UM server 0x2400 - 0x25FF = 512 */
#define ANI_VAL_ID_UM_START  0x2400

/* Value Identity Ranges for CM server 0x2600 - 0x27FF = 512 */
#define ANI_VAL_ID_CM_START  0x2600

/* Value Identity Ranges for PM server 0x2800 - 0x29FF = 512 */
#define ANI_VAL_ID_PM_START  0x2800

/* Value Identity Ranges for IAPP server 0x3000 - 0x31FF = 512 */
#define ANI_VAL_ID_IAPP_START  0x3000

/* Value Identity Ranges for PRM server 0x4000 - 0x41FF = 512 */
#define ANI_VAL_ID_PRM_START  0x4000

/* Value Identity Ranges for LSM server 0x5000 - 0x51FF = 512 */
#define ANI_VAL_ID_LSM_START  0x5000

/* Value Identity Ranges for RFS server 0x5200 - 0x53FF = 512 */
#define ANI_VAL_ID_RFS_START  0x5200

/* Value Identity Ranges for GPM CLI 0x5400 - 0x55FF = 512 */
#define ANI_VAL_ID_GPM_CLI_START    0x5400

#endif /* __ANI_ASF_PARM_TYPES_H_ */
