/*
* Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved.
*/

#ifndef _LINUX_FIH_SW_INFO_H
#define _LINUX_FIH_SW_INFO_H

//BSP-REXER-SMEM-00+[
#include "fih_hw_info.h"

struct smem_oem_info
{
  unsigned int hwid;
  unsigned int hwid_verify; /*BSP-ELuo-HWID_VERIFY-00*/
  char   nonHLOS_version[32];
  char   nonHLOS_git_head[64];
  unsigned int power_on_cause;
  /*BSP-LC-Write_IMEI_MEID_to_NV-00 +[*/
  uint8_t imei_1[16];
  uint8_t imei_2[16];
  uint32_t imei_status;
  /*BSP-LC-Write_IMEI_MEID_to_NV-00 +]*/
};
#define PROJECT_ID_SHIFT_MASK 0
#define PHASE_ID_SHIFT_MASK 8
#define BAND_ID_SHIFT_MASK 16
#define SIM_ID_SHIFT_MASK 24
//BSP-REXER-SMEM-00+]
// BSP-JH-Reserve_Mem-00 +[
/*===========================================================================
                        FIH Reserved Memory Definition
===========================================================================*/
//CORE-KH-DebugToolPorting-01-m[
#define MTD_MEMORY_RESERVE_BASE 0xA0000000
#define MTD_MEMORY_RESERVE_SIZE 0x00400000
//CORE-KH-DebugToolPorting-01-m]
// BSP-JH-Reserve_Mem-00 +]


#define MTD_RAM_CONSOLE_ADDR  MTD_MEMORY_RESERVE_BASE
#define MTD_RAM_CONSOLE_SIZE  0x00100000
#define MTD_MLOG_ADDR         (MTD_MEMORY_RESERVE_BASE + MTD_RAM_CONSOLE_SIZE
#define MTD_MLOG_SIZE         0x00100000
#define MTD_FB_SPARE_ADDR     (MTD_MLOG_ADDR + MTD_MLOG_SIZE)
#define MTD_FB_SPARE_SIZE     0x00100000

#define SEMC_PartNumber		"S_PartNumber"
#define SEMC_SwRevison		"S_SwRevision"
#define SEMC_ReleaseFlag	"S_ReleaseFlag"
#define SEMC_BlobVersion	"S_BlobVersion"
#define SEMC_BpVersion		"S_BpVersion"
#define SEMC_SvnVersion		"S_SvnVersion"
#define SEMC_ProductName	"S_ProductModelName"

//MTD-KERNEL-DL-PWRON_CAUSE-00 +[
/*===========================================================================
                        FIH PWRON CAUSE
===========================================================================*/
#define MTD_PWR_ON_EVENT_MODEM_FATAL_ERROR      0x10000000 // modem fatal error
#define MTD_PWR_ON_EVENT_KERNEL_PANIC           0x20000000 // host panic
#define MTD_PWR_ON_EVENT_MODEM_SW_WD_RESET      0x40000000 // modem software watchdog reset
#define MTD_PWR_ON_EVENT_MODEM_FW_WD_RESET      0x80000000 // modem firmware watchdog reset
#define MTD_PWR_ON_EVENT_ABNORMAL_RESET         0x01000000 // abnormal reset
#define MTD_PWR_ON_EVENT_HW_WD_RESET            0x00100000 // hardware watchdog reset
#define MTD_PWR_ON_EVENT_SOFTWARE_RESET         0x00200000 // software reset //CORE-DL-AddPocForSwReset-00
#define MTD_PWR_ON_EVENT_PWR_OFF_CHG_REBOOT     0x00400000 // power off charging reset system //CORE-DL-FixForcePowerOn-00
#define MTD_PWR_ON_EVENT_RPM_WD_RESET           0x00800000 // rpm watchdog reset /* MTD-CORE-EL-AddPocForRPM-00+ */
#define MTD_PWR_ON_EVENT_FORCE_TRIGGER_PANIC    0x00010000 /*CORE-TH-manual_crash-00+*/
#define MTD_PWR_ON_EVENT_CLEAN_DATA             0x00000000 // data is clean
//MTD-KERNEL-DL-PWRON_CAUSE-00 +]

/*===========================================================================
                        FIH RAM console definition
===========================================================================*/
/*0x88100000*/
#define RAM_CONSOLE_PHYS MTD_RAM_CONSOLE_ADDR
#define RAM_CONSOLE_SIZE 0x00020000/*128KB */

/*0x88120000*/
#define ALOG_RAM_CONSOLE_PHYS_MAIN (RAM_CONSOLE_PHYS + RAM_CONSOLE_SIZE)
#define ALOG_RAM_CONSOLE_SIZE_MAIN 0x00020000 /*128KB */

/*0x88140000*/
#define ALOG_RAM_CONSOLE_PHYS_RADIO (ALOG_RAM_CONSOLE_PHYS_MAIN +  ALOG_RAM_CONSOLE_SIZE_MAIN)
#define ALOG_RAM_CONSOLE_SIZE_RADIO 0x00020000 /*128KB */

/*0x88160000*/
#define ALOG_RAM_CONSOLE_PHYS_EVENTS (ALOG_RAM_CONSOLE_PHYS_RADIO + ALOG_RAM_CONSOLE_SIZE_RADIO)
#define ALOG_RAM_CONSOLE_SIZE_EVENTS 0x00020000 /*128KB */

/*0x88180000*/
#define ALOG_RAM_CONSOLE_PHYS_SYSTEM (ALOG_RAM_CONSOLE_PHYS_EVENTS + ALOG_RAM_CONSOLE_SIZE_EVENTS)
#define ALOG_RAM_CONSOLE_SIZE_SYSTEM 0x00020000 /*128KB */

//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a[
#define ALOG_RAM_CONSOLE_PHYS_CRASH (ALOG_RAM_CONSOLE_PHYS_SYSTEM + ALOG_RAM_CONSOLE_SIZE_SYSTEM)
#define ALOG_RAM_CONSOLE_SIZE_CRASH 0x00020000 /*128KB */
//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-a]

/*===========================================================================
                        FIH Suspend definition
===========================================================================*/
/*0x881A0000*/
#define MTD_SUSPEND_LOG_BASE (ALOG_RAM_CONSOLE_PHYS_CRASH + ALOG_RAM_CONSOLE_SIZE_CRASH)	//CORE-KH-DbgCfgTool-LogcatCrashCmd-00-m
#define MTD_SUSPEND_LOG_SIZE 0x00020000 /* 128KB */

/*===========================================================================
                        FIH Other definition
===========================================================================*/
#define FIH_OTHER_DATA_BASE	(RAM_CONSOLE_PHYS + MTD_RAM_CONSOLE_SIZE)
#define CONFIG_FEATURE_FIH_SW3_PANIC_FILE

/*0x881FFFF0 Ramdump Time*/
#define   CRASH_TIME_RAMDUMP_LEN	0x10 /*16bytes*/
#define   CRASH_TIME_RAMDUMP_ADDR	(FIH_OTHER_DATA_BASE - CRASH_TIME_RAMDUMP_LEN)

/*0x881FFFEC power off charging*/
#define   SECOND_POWER_ON_LEN	0x04 /*4bytes*/
#define   SECOND_POWER_ON_ADDR	(CRASH_TIME_RAMDUMP_ADDR - SECOND_POWER_ON_LEN)
#define   SECOND_POWER_ON_SIGN	0x65971228

/*0x881FFE80 RAM panic data*/
#define PANIC_RAM_DATA_SIZE		0x600 //increase buffer size to 1.5k //CORE-KH-DebugTool_ForcePanic_forL-00-m
#define PANIC_RAM_DATA_BEGIN	(SECOND_POWER_ON_ADDR - PANIC_RAM_DATA_SIZE)
#define PANIC_RAM_SIGNATURE		0x19761105
struct fih_panic_ram_data{
  unsigned int				signature;
  unsigned int				length;
  char						data[1];
} ;

//MTD-KERNEL-DL-PWRON_CAUSE-00 +[
/*0x881FFFE8 Power on cause*/
#define FIH_PWRON_CAUSE_LEN      0x04 /*4bytes*/
#define FIH_PWRON_CAUSE_ADDR     (FIH_PRODUCTINFO_ADDR - FIH_PWRON_CAUSE_LEN)

/*0x881FFFE4 HW watch dog*/
#define FIH_HW_WD_LEN            0x04 /*4bytes*/
#define FIH_HW_WD_ADDR           (FIH_PWRON_CAUSE_ADDR - FIH_HW_WD_LEN)
#define FIH_HW_WD_SIGNATURE      0x19850118

#define SOFTWARE_RESET           99500 //CORE-DL-AddPocForSwReset-00
#define MODEM_FATAL_ERR          99501
#define MODEM_SW_WDOG_EXPIRED    99502
#define MODEM_FW_WDOG_EXPIRED    99503
#define HOST_KERNEL_PANIC        99504
#define PWR_OFF_CHG_REBOOT       99505 //CORE-DL-FixForcePowerOn-00
//MTD-KERNEL-DL-PWRON_CAUSE-00 +]

/*0x881FFDB0 RAM STORE_FATAL_ERROR_REASON*/
#define DIAG_BUFFER_LEN		0xD0
#define STORE_FATAL_ERROR_REASON (PANIC_RAM_DATA_BEGIN - DIAG_BUFFER_LEN)

/* CORE-HC-productinfo-01+[ */
/*0x200FFC84 productinfo*/
#define FIH_PRODUCTINFO_LEN      0x12C /*300bytes*/
#define FIH_PRODUCTINFO_ADDR     (STORE_FATAL_ERROR_REASON - FIH_PRODUCTINFO_LEN)
#define PRODUCTINFO_ITEM_LEN 30
enum productinfo_offset
{
    OFFICIAL_VER_OFFSET = 0,
    VENDER_VER_OFFSET = OFFICIAL_VER_OFFSET + PRODUCTINFO_ITEM_LEN,
    PRODUCT_MODEL_OFFSET = VENDER_VER_OFFSET + PRODUCTINFO_ITEM_LEN,
    BUILD_TYPE_OFFSET = PRODUCT_MODEL_OFFSET + PRODUCTINFO_ITEM_LEN
};
/* CORE-HC-productinfo-01+] */

/* MTD-CORE-EL-handle_SSR-01*[ */
#define MODEM_ERR_TAG			"ERR:"
#define MODEM_ERR_TAG_LEN		4 	/*equals to (strlen(MODEM_ERR_TAG))*/
void log_ss_failure_reason(const char* ss_name, u32 size, char* err_str);
/* MTD-CORE-EL-handle_SSR-01*] */

#endif

