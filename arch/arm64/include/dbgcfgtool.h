/**+===========================================================================
  File: dbgcfgtool.h

  Description:


  Note:


  Author:
        Hebbel Chao Oct-20-2009

-------------------------------------------------------------------------------
** FIHSPEC CONFIDENTIAL
** Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved.
** Copyright(c) 2009 FIHSPEC Corporation. All Rights Reserved.
**
** The source code contained or described herein and all documents related
** to the source code (Material) are owned by Mobinnova Technology Corporation.
** The Material is protected by worldwide copyright and trade secret laws and
** treaty provisions. No part of the Material may be used, copied, reproduced,
** modified, published, uploaded, posted, transmitted, distributed, or disclosed
** in any way without Mobinnova prior express written permission.
============================================================================+*/
#ifndef __DBGCFGTOOL_H__
#define __DBGCFGTOOL_H__

#define DEV_IOCTLID 0xD1
#define DBG_IOCTL_CMD_HANDLE_DBGCFG _IOWR(DEV_IOCTLID, 0, dbgcfg_ioctl_arg)

typedef enum
{
    DBG_CMDLINE_PARAMETER_START = 0,
    DBG_UARTMSG_CFG = DBG_CMDLINE_PARAMETER_START,
    DBG_PRINTF_UARTMSG_CFG = 1,
    DBG_ANDROID_UARTMSG_CFG = 2,
    DBG_ANDROID_UARTMSG_MAIN_CFG = 3,
    DBG_ANDROID_UARTMSG_RADIO_CFG = 4,
    DBG_ANDROID_UARTMSG_EVENTS_CFG = 5,
    DBG_ANDROID_UARTMSG_SYSTEM_CFG = 6,
    DBG_FORCE_TRIGGER_PANIC_CFG = 7,
    DBG_RAMDUMP_TO_SDCARD_CFG = 8,  /*CORE-HC-RAMDUMP-00+*/
    DBG_SUBSYSTEM_RESTART_CFG = 9,	/*CORE-TH-SubSystemRestart-00+*/
    DBG_USB_ENG_MODE_CFG = 10,	/*CONN-JY-OpenUSBbyConfigTA-00+*/
    DBG_SUBSYSTEM_RAMDUMP_CFG = 11,/*CORE-TH-SubSystemRestart-00+*/
    DBG_SUBSYSTEM_RAMDUMP_RPM_LOG_CFG = 12, /*CORE-TH-SubSystemRestart-00+*/
    DBG_FORCE_TO_KEEP_DEBUG_CFG = 13,	//CORE-KH-KeepDebugSetting-00-a

	/* bits 28-31 will not be cleared when data wiped */
    DBG_EN_POWER_OFF_CHG_LOG = 31,			/* CORE-EL-POWER_OFF_CHG_LOG-00+ */
    DBG_CMDLINE_PARAMETER_MAX = DBG_EN_POWER_OFF_CHG_LOG,
    
    DBG_XML_PARAMETER_START = 32,
    DBG_CPU_USAGE_CFG = DBG_XML_PARAMETER_START,
    DBG_TCXOSD_DISPLAY_ON_CFG = 33,
    DBG_SENSOR_TEMP_RECORDING_CFG = 34,	//CORE-KH-DbgCfgTool-RecordingTemp-00-a
    DBG_DMESG_RESTRICT_CFG = 35,	//CORE-KH-DbgCfgTool-klogd-00-a
    DBG_XML_PARAMETER_MAX = 63
}dbgcfg_id;

typedef enum
{
    DBGCFG_READ = 0,
    DBGCFG_WRITE = 1
}dbgcfg_action;

typedef struct
{
    dbgcfg_id id;
    dbgcfg_action action;
    unsigned int value;
}dbgcfg_ioctl_arg;

/* Referenced from system/core/include/cutils/log.h */
typedef enum {
    LOG_ID_MAIN = 0,
    LOG_ID_RADIO = 1,
    LOG_ID_EVENTS = 2,
    LOG_ID_SYSTEM = 3,
    LOG_ID_MAX
} log_id_t;

#endif	//__DBGCFGTOOL_H__