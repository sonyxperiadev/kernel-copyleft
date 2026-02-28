/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_BLUETOOTH_POWER_H
#define __LINUX_BLUETOOTH_POWER_H

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/soc/qcom/qcom_aoss.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>

/*
 * voltage regulator information required for configuring the
 * bluetooth chipset
 */

enum power_modes {
	POWER_DISABLE = 0,
	POWER_ENABLE,
	POWER_RETENTION,
	POWER_DISABLE_RETENTION,
};

enum SubSystem {
	BLUETOOTH = 1,
	UWB,
};

enum FmdOperation {
	ENABLE_FMD,
	DISABLE_FMD,
	UPDATE_SOC_VER
};

enum power_states {
	IDLE = 0,
	BT_ON,
	UWB_ON,
	ALL_CLIENTS_ON,
};

enum retention_states {
	/* Default state */
	RETENTION_IDLE = 0,
	/* When BT is only client and it is in retention_state */
	BT_IN_RETENTION,
	/* BT is retention mode and UWB powered ON triggered */
	BT_OUT_OF_RETENTION,
	/* When UWB is only client and it is in retention_state */
	UWB_IN_RETENTION,
	/* UWB is retention mode and BT powered ON triggered */
	UWB_OUT_OF_RETENTION,
	/* Both clients are voted for retention */
	BOTH_CLIENTS_IN_RETENTION,
};

enum grant_return_values {
	ACCESS_GRANTED = 0,
	ACCESS_DENIED  = 1,
	ACCESS_RELEASED = 2,
	ACCESS_DISALLOWED = -1,
};

enum grant_states {
	/* Default state */
	NO_GRANT_FOR_ANY_SS = 0,
	NO_OTHER_CLIENT_WAITING_FOR_GRANT,
	BT_HAS_GRANT,
	UWB_HAS_GRANT,
	BT_WAITING_FOR_GRANT,
	UWB_WAITING_FOR_GRANT,
};

static inline char *ConvertGrantRetToString(enum grant_return_values state)
{
	switch (state) {
	case ACCESS_GRANTED:
		return "ACCESS_GRANTED";
	case ACCESS_DENIED:
		return "ACCESS_DENIED";
	case ACCESS_RELEASED:
		return "ACCESS_RELEASED";
	case ACCESS_DISALLOWED:
		return "ACCESS_DISALLOWED";
	default:
		return "INVALID State";
	}
}

static inline char *ConvertGrantToString(enum grant_states state)
{
	switch (state) {
	case NO_GRANT_FOR_ANY_SS:
		return "NO_GRANT_FOR_ANY_SS";
	case NO_OTHER_CLIENT_WAITING_FOR_GRANT:
		return "NO_OTHER_CLIENT_WAITING_FOR_GRANT";
	case BT_HAS_GRANT:
		return "BT_HAS_GRANT";
	case UWB_HAS_GRANT:
		return "UWB_HAS_GRANT";
	case BT_WAITING_FOR_GRANT:
		return "BT_WAITING_FOR_GRANT";
	case UWB_WAITING_FOR_GRANT:
		return "UWB_WAITING_FOR_GRANT";
	default:
		return "INVALID STATE";
	}
}

enum cores {
	BT_CORE = 0,
	UWB_CORE,
	PLATFORM_CORE
};

enum ssr_states {
	SUB_STATE_IDLE = 0,
	SSR_ON_BT,
	BT_SSR_COMPLETED,
	SSR_ON_UWB,
	UWB_SSR_COMPLETED,
	REG_BT_PID,
	REG_UWB_PID,
};

enum plt_pwr_state {
	POWER_ON_BT = 0,
	POWER_OFF_BT,
	POWER_ON_UWB,
	POWER_OFF_UWB,
	POWER_ON_BT_RETENION,
	POWER_ON_UWB_RETENION,
	BT_ACCESS_REQ,
	UWB_ACCESS_REQ,
	BT_RELEASE_ACCESS,
	UWB_RELEASE_ACCESS,
	BT_MAX_PWR_STATE,
};

enum {
	PWR_WAITING_RSP = -2,
	PWR_RSP_RECV = 0,
	PWR_FAIL_RSP_RECV = -1,
	PWR_CLIENT_KILLED,
};

static inline char *ConvertRetentionModeToString(int state)
{
	switch (state) {
	case IDLE:
		return "Both client not in Retention";
	case BT_IN_RETENTION:
		return "BT in Retention";
	case BT_OUT_OF_RETENTION:
		return "BT is out off Retention";
	case UWB_IN_RETENTION:
		return "UWB in Retention";
	case UWB_OUT_OF_RETENTION:
		return "UWB is out off Retention";
	case BOTH_CLIENTS_IN_RETENTION:
		return "Both client in Retention";
	default:
		return "Retention state = INVALID STATE";
	}
}

static inline char *ConvertClientReqToString(int arg)
{
	switch (arg) {
	case POWER_DISABLE:
		return "Power OFF";
	case POWER_ENABLE:
		return "Power ON";
	case POWER_RETENTION:
		return "Power Retention";
	default:
		return "INVALID STATE";
	}
}

static inline char *ConvertPowerStatusToString(int state)
{
	switch (state) {
	case IDLE:
		return "Current state is ALL Client OFF";
	case BT_ON:
		return "Current state is BT powered ON";
	case UWB_ON:
		return "Current state is UWB powered ON";
	case ALL_CLIENTS_ON:
		return "Current state is ALL Client ON";
	default:
		return "Current state is = INVALID STATE";
	}
}

static inline char *ConvertSsrStatusToString(int state)
{
	switch (state) {
	case SUB_STATE_IDLE:
		return "and No SSR";
	case SSR_ON_BT:
		return "and SSR on BT";
	case BT_SSR_COMPLETED:
		return "and BT SSR completed";
	case SSR_ON_UWB:
		return "and SSR on UWB";
	case UWB_SSR_COMPLETED:
		return "and UWB SSR completed";
	default:
		return "SSR STATE = INVALID STATE";
	}
}

static inline char *ConvertPowerReqToString(int arg)
{
	switch (arg) {
	case POWER_ON_BT:
		return "POWER_ON_BT";
	case POWER_OFF_BT:
		return "POWER_OFF_BT";
	case POWER_ON_UWB:
		return "POWER_ON_UWB";
	case POWER_OFF_UWB:
		return "POWER_OFF_UWB";
	case POWER_ON_BT_RETENION:
		return "POWER_ON_BT_RETENION";
	case POWER_ON_UWB_RETENION:
		return "POWER_ON_UWB_RETENION";
	case BT_ACCESS_REQ:
		return "BT_ACCESS_REQ";
	case UWB_ACCESS_REQ:
		return "UWB_ACCESS_REQ";
	case BT_RELEASE_ACCESS:
		return "BT_RELEASE_ACCESS";
	case UWB_RELEASE_ACCESS:
		return "UWB_RELEASE_ACCESS";
	case BT_MAX_PWR_STATE:
		return "BT_MAX_PWR_STATE";
	default:
		return "INVALID STATE";
	}
};

static inline char *ConvertRegisterModeToString(int reg_mode)
{
	switch (reg_mode) {
	case POWER_DISABLE:
		return "vote off";
	case POWER_ENABLE:
		return "vote on";
	case POWER_RETENTION:
		return "vote for retention";
	case POWER_DISABLE_RETENTION:
		return "vote offretention";
	default:
		return "INVALID STATE";
	}
}

enum UwbPrimaryReasonCode{
	UWB_HOST_REASON_DEFAULT_NONE  = 0x00,                         //INVALID REASON
	UWB_HOST_REASON_PERI_SOC_CRASHED = 0x01,                      //PERI SOC WAS CRASHED
	UWB_HOST_REASON_PERI_SOC_CRASHED_DIAG_SSR = 0x02,             //PERI SOC CRASHED DIAG INITIATED SSR
	UWB_HOST_REASON_INIT_FAILED = 0x03,                           //HOST INITIALIZATION FAILED
	UWB_HOST_REASON_CLOSE_RCVD_DURING_INIT = 0x04,                //CLOSE RECEIVED FROM STACK DURING SOC INIT
	UWB_HOST_REASON_ERROR_READING_DATA_FROM_Q2SPI = 0x05,         //ERROR READING DATA FROM Q2SPI
	UWB_HOST_REASON_WRITE_FAIL_SPCL_BUFF_CRASH_SOC = 0x06,        //FAILED TO WRITE SPECIAL BYTES TO CRASH SOC
	UWB_HOST_REASON_RX_THREAD_STUCK = 0x07,                       //RX THREAD STUCK
	UWB_HOST_REASON_SSR_CMD_TIMEDOUT = 0x08,                      //SSR DUE TO CMD TIMED OUT
	UWB_HOST_REASON_SSR_INVALID_BYTES_RCVD = 0x0A,                //INVALID HCI CMD TYPE RECEIVED
	UWB_HOST_REASON_SSR_RCVD_LARGE_PKT_FROM_SOC = 0x0B,           //SSR DUE TO LARGE PKT RECVIVED FROM SOC
	UWB_HOST_REASON_SSR_UNABLE_TO_WAKEUP_SOC = 0x0C,              //UNABLE TO WAKE UP SOC
	UWB_HOST_REASON_CMD_TIMEDOUT_SOC_WAIT_TIMEOUT = 0x0D,         //COMMAND TIMEOUT AND SOC CRASH WAIT TIMEOUT
	UWB_HOST_REASON_INV_BYTES_SOC_WAIT_TIMEOUT = 0x0F,            //INVALID BYTES AND SOC CRASH WAIT TIMEOUT
	UWB_HOST_REASON_SOC_WAKEUP_FAILED_SOC_WAIT_TIMEOUT = 0x10,    //SOC WAKEUP FAILURE AND SOC CRASH WAIT TIMEOUT
	UWB_HOST_REASON_SOC_CRASHED_DIAG_SSR_SOC_WAIT_TIMEOUT = 0x11, //SOC CRASHED DIAG INITIATED SSR CRASH WAIT TIMEOUT
	UWB_HOST_REASON_NONE_SOC_WAIT_TIMEOUT = 0x12,                 //INVALID FAILURE AND SOC CRASH WAIT TIMEOUT
	UWB_HOST_REASON_SOC_DEINIT_STUCK = 0x13,                      //SOC DEINIT STUCK
	UWB_HOST_REASON_SSR_INTERNAL_CMD_TIMEDOUT = 0x14,             //SSR DUE TO CMD INTERNAL TIMED OUT
	UWB_HOST_REASON_FAILED_TO_SEND_INTERNAL_CMD = 0x15,           //FAILED TO SEND INTERNAL CMD
	UWB_HOST_REASON_SSR_SLEEP_IND_NOT_RCVD = 0x16,                //SOC DID NOT RCVD SLEEP IND DURING CLOSE
	UWB_HOST_REASON_UWB_SOC_CRASHED = 0xC1,                       //UWB SOC WAS CRASHED
	UWB_HOST_REASON_UWB_SOC_CRASHED_DIAG_SSR = 0xC2,              //UWB SOC CRASHED DIAG INITIATED SSR
	UWB_HOST_REASON_DIAG_LOG_API_STUCK = 0x39,                    //DIAG log API stuck.
	UWB_HOST_REASON_PERI_CRASH_ON_OTHER_SS = 0x3A,                //Peripheral core crash detected in BT SS
	UWB_HOST_REASON_CRASH_EVT_INDUCED = 0x60,                     //Packet Type from SoC for inducing crash
};

enum UwbSecondaryReasonCode{
	UWB_SOC_REASON_DEFAULT                 =  0x00,
	UWB_SOC_REASON_TX_RX_INVALID_PKT       =  0x40,
	UWB_SOC_REASON_TX_RX_INVALID_PKT_LENE  =  0x41,
	UWB_SOC_REASON_TX_RX_OVERFLOW_BUFF     =  0x42,
	UWB_SOC_REASON_UNKNOWN                 =  0x81,
	UWB_SOC_REASON_SW_REQUESTED            =  0x82,
	UWB_SOC_REASON_STACK_OVERFLOW          =  0x83,
	UWB_SOC_REASON_EXCEPTION               =  0x84,
	UWB_SOC_REASON_ASSERT                  =  0x85,
	UWB_SOC_REASON_TRAP                    =  0x86,
	UWB_SOC_REASON_OS_FATAL                =  0x87,
	UWB_SOC_REASON_HCI_RESET               =  0x88,
	UWB_SOC_REASON_PATCH_RESET             =  0x89,
	UWB_SOC_REASON_ABT                     =  0x8A,
	UWB_SOC_REASON_RAMMASK                 =  0x8B,
	UWB_SOC_REASON_PREBARK                 =  0x8C,
	UWB_SOC_REASON_BUSERROR                =  0x8D,
	UWB_SOC_REASON_IO_FATAL                =  0x8E,
	UWB_SOC_REASON_SSR_CMD                 =  0x8F,
	UWB_SOC_REASON_POWERON                 =  0x90,
	UWB_SOC_REASON_WATCHDOG                =  0x91,
	UWB_SOC_REASON_RAMMASK_RGN1            =  0x92,
	UWB_SOC_REASON_RAMMASK_RGN0            =  0x93,
	UWB_SOC_REASON_Q6_WATCHDOG             =  0x94,
	UWB_SOC_REASON_ZEALIS_RAM_MASK_RGN0    =  0x95,
	UWB_SOC_REASON_ZEALIS_RAM_MASK_RGN1    =  0x96,
	UWB_SOC_REASON_APSS_RESET              =  0x97,
	UWB_SOC_REASON_TIME_RESET              =  0x98,
	UWB_SOC_REASON_AUDIOSS_RESET           =  0x99,
	UWB_SOC_REASON_HOST_WARMRESET          =  0x9A,
	UWB_SOC_REASON_HOST_NMI_INIT           =  0x9B,
	UWB_SOC_REASON_PANIC_FAULT             =  0x9C,
	UWB_SOC_REASON_EARLY_TRAP              =  0x9D,
	UWB_SOC_REASON_INSTR_ADDR_MISALGIN     =  0x9E,
	UWB_SOC_REASON_INSTR_ACCESS_FAULT      =  0x9F,
	UWB_SOC_REASON_ILLEGAL_INSTR           =  0xA0,
	UWB_SOC_REASON_BREAKPOINT_EXCEPTION    =  0xA1,
	UWB_SOC_REASON_LOAD_ADDR_MISALIGN      =  0xA2,
	UWB_SOC_REASON_LOAD_ACCESS_FAULT       =  0xA3,
	UWB_SOC_REASON_STORE_ADDR_MISALGN      =  0xA4,
	UWB_SOC_REASON_STORE_ACCESS_FAULT      =  0xA5,
	UWB_SOC_REASON_ECALL_UMODE             =  0xA6,
	UWB_SOC_REASON_ECALL_MMODE             =  0xA7,
	UWB_SOC_REASON_STACK_UNDERFLOW         =  0xA8,
	UWB_SOC_REASON_MACHINE_EXIT_INT        =  0xA9,
	UWB_SOC_REASON_PERF_MONITOR_OVERFLOW   =  0xAA,
	UWB_SOC_REASON_EXT_SUBSYS_RESET        =  0xAB,
	UWB_SOC_REASON_IPC_STALL               =  0xAC,
	UWB_SOC_REASON_PEER_CPU0_NMI           =  0xAD,
	UWB_SOC_REASON_PEER_CPU1_NMI           =  0xAE,
	UWB_SOC_REASON_PEER_CPU2_NMI           =  0xAF,
	UWB_SOC_REASON_TX_RX_INVALID_PKT_FATAL =  0xC0,
	UWB_SOC_REASON_TX_RX_INVALID_LEN_FATAL =  0xC1,
	UWB_SOC_REASON_TX_RX_OVERFLOW_FATAL    =  0xC2,
	UWB_SOC_REASON_INVALID_STACK           =  0xF0,
	UWB_SOC_REASON_INVALID_MCI_MSG_RCVD    =  0xF1,
	UWB_HOST_REASON_PERI_GETVER_SEND_STUCK          =  0x18,
	UWB_HOST_REASON_PERI_GETVER_NO_RSP_RCVD         =  0x19,
	UWB_HOST_REASON_PERI_PATCH_DNLD_STUCK           =  0x1B,
	UWB_HOST_REASON_PERI_GETBOARDID_CMD_STUCK       =  0x1C,
	UWB_HOST_REASON_PERI_NVM_DNLD_STUCK             =  0x1D,
	UWB_HOST_REASON_PERI_RESET_STUCK                =  0x1E,
	UWB_HOST_REASON_PERI_GETBLDINFO_CMD_STUCK       =  0x1F,
	UWB_HOST_REASON_PERI_ENHLOG_CMD_STUCK           =  0x21,
	UWB_HOST_REASON_DIAGINIT_STUCK                  =  0x22,
	UWB_HOST_REASON_DIAGDEINIT_STUCK                =  0x23,
	UWB_HOST_REASON_SECURE_BRIDGE_CMD_STUCK         =  0x26,
	UWB_HOST_REASON_FAILED_TO_SEND_CMD              =  0x27,
	UWB_HOST_REASON_PERI_RESET_CC_NOT_RCVD          =  0x28,
	UWB_HOST_REASON_HCI_PRE_SHUTDOWN_CC_NOT_RCVD    =  0x29,
	UWB_HOST_REASON_FAILED_TO_RECEIVE_SLEEP_IND     =  0x2B,
	UWB_HOST_REASON_POWER_ON_REGS_STUCK             =  0x2C,
	UWB_HOST_REASON_RX_THREAD_START_STUCK           =  0x2D,
	UWB_HOST_REASON_GET_LOCALADDR_STUCK             =  0x2E,
	UWB_HOST_REASON_OTP_INFO_GET_CMD_STUCK          =  0x2F,
	UWB_HOST_REASON_FILE_SYSTEM_CALL_STUCK          =  0x30,
	UWB_HOST_REASON_PROPERTY_GET_STUCK              =  0x31,
	UWB_HOST_REASON_PROPERTY_SET_STUCK              =  0x32,
	UWB_HOST_REASON_PERI_RAM_PATCH_READ_STUCK       =  0x33,
	UWB_HOST_REASON_PERI_NVM_PATCH_READ_STUCK       =  0x34,
	UWB_HOST_REASON_POWER_IOCTL_STUCK               =  0x36,
	UWB_HOST_REASON_PERI_PATCH_CONFIG_CMD_STUCK     =  0x37,
	UWB_HOST_REASON_PERI_PATCH_CONFIG_FAILED        =  0x38,
	UWB_HOST_REASON_UWB_GETVER_SEND_STUCK           =  0x39,
	UWB_HOST_REASON_UWB_GETVER_NO_RSP_RCVD          =  0x3A,
	UWB_HOST_REASON_SOC_NAME_UNKOWN                 =  0x3B,
	UWB_HOST_REASON_PERI_GETVER_CMD_FAILED          =  0x3C,
	UWB_HOST_REASON_BAUDRATE_CHANGE_FAILED          =  0x3D,
	UWB_HOST_REASON_PERI_TLV_DOWNLOAD_FAILED        =  0x3E,
	UWB_HOST_REASON_PERI_GETBLDINFO_CMD_FAILED      =  0x3F,
	UWB_HOST_REASON_PERI_RESET_CMD_FAILED           =  0x40,
	UWB_HOST_REASON_MEMORY_ALLOCATION_FAILED        =  0x42,
	UWB_HOST_REASON_READ_THREAD_START_FAILED        =  0x43,
	UWB_HOST_REASON_HW_FLOW_ON_FAILED               =  0x44,
	UWB_HOST_REASON_PERI_NVM_FILE_NOT_FOUND         =  0x45,
	UWB_HOST_REASON_UWB_RAM_PATCH_READ_STUCK        =  0x48,
	UWB_HOST_REASON_UWB_NVM_PATCH_READ_STUCK        =  0x49,
	UWB_HOST_REASON_UWB_NVM_FILE_NOT_FOUND          =  0x4A,
	UWB_HOST_REASON_UWB_GETBLDINFO_CMD_FAILED       =  0x4B,
	UWB_HOST_REASON_UWB_PATCH_DNLD_STUCK            =  0x4C,
	UWB_HOST_REASON_UWB_NVM_DNLD_STUCK              =  0x4D,
	UWB_HOST_REASON_UWB_GETBLDINFO_CMD_STUCK        =  0x4E,
	UWB_HOST_REASON_PERI_ACTIVATE_CMD_STUCK         =  0x4F,
	UWB_HOST_REASON_PERI_ARBITRATION_CMD_STUCK      =  0x50,
	UWB_HOST_REASON_PERI_ARBITRATION_NTF_STUCK      =  0x51,
	UWB_HOST_REASON_INITIALIZATION_FAILED           =  0x52,
	UWB_HOST_REASON_UWB_RESET_CC_NOT_RCVD           =  0x53,
	UWB_HOST_REASON_UWB_ACTIVATE_CC_NOT_RCVD        =  0x54,
	UWB_HOST_REASON_TME_ACTIVATE_CC_NOT_RCVD        =  0x55,
	UWB_HOST_REASON_Q2SPI_INIT_STUCK                =  0x56,
	UWB_HOST_REASON_Q2SPI_INIT_FAILED               =  0x57,
	UWB_HOST_REASON_UWB_TLV_DOWNLOAD_FAILED         =  0x58,
	UWB_HOST_REASON_UWB_ENHLOG_CMD_STUCK            =  0x59,
	UWB_HOST_REASON_UWB_GETVER_CMD_FAILED           =  0x5A,
	UWB_HOST_REASON_UWB_PATCH_CONFIG_CMD_STUCK      =  0x5B,
	UWB_HOST_REASON_UWB_PATCH_CONFIG_CMD_FAILED     =  0x5C,
	UWB_HOST_REASON_UWB_RESET_STUCK                 =  0x5D,
	UWB_HOST_REASON_PERI_ACTIVATE_NTF_STUCK         =  0x5E,
	UWB_HOST_REASON_UWB_CORE_RESET_CMD_FAILED       =  0x5F,
	UWB_HOST_REASON_TME_ARBITRATION_CMD_STUCK       =  0x60,
	UWB_HOST_REASON_TME_ARBITRATION_NTF_STUCK       =  0x61,
	UWB_HOST_REASON_TME_GETVER_SEND_STUCK           =  0x62,
	UWB_HOST_REASON_TME_GETVER_NO_RSP_RCVD          =  0x63,
	UWB_HOST_REASON_TME_GETVER_CMD_FAILED           =  0x64,
	UWB_HOST_REASON_TME_PATCH_DNLD_STUCK            =  0x65,
	UWB_HOST_REASON_TME_RESET_STUCK                 =  0x66,
	UWB_HOST_REASON_TME_GETBLDINFO_CMD_STUCK        =  0x67,
	UWB_HOST_REASON_TME_GETBLDINFO_CMD_FAILED       =  0x68,
	UWB_HOST_REASON_TME_RAM_PATCH_READ_STUCK        =  0x69,
	Q2SPI_REASON_DEFAULT                            =  0xFF
};

typedef struct {
  enum UwbSecondaryReasonCode reason;
  char reasonstr[50];
} UwbSecondaryReasonMap;

typedef struct {
  enum UwbPrimaryReasonCode reason;
  char reasonstr[100];
} UwbPrimaryReasonMap;

static UwbPrimaryReasonMap uwbPriReasonMap[] = {
	{UWB_HOST_REASON_DEFAULT_NONE, "Invalid reason"},
	{UWB_HOST_REASON_PERI_SOC_CRASHED, "Peri SOC crashed"},
	{UWB_HOST_REASON_UWB_SOC_CRASHED, "UWB SOC crashed"},
	{UWB_HOST_REASON_PERI_SOC_CRASHED_DIAG_SSR, "Peri SOC crashed with diag initiated SSR"},
	{UWB_HOST_REASON_UWB_SOC_CRASHED_DIAG_SSR, "UWB SOC crashed with diag initiated SSR"},
	{UWB_HOST_REASON_INIT_FAILED, "Init failed"},
	{UWB_HOST_REASON_CLOSE_RCVD_DURING_INIT, "Close received from stack during SOC init"},
	{UWB_HOST_REASON_ERROR_READING_DATA_FROM_Q2SPI, "Error reading data from Q2SPI"},
	{UWB_HOST_REASON_WRITE_FAIL_SPCL_BUFF_CRASH_SOC, "Failed to write special bytes to crash SOC"},
	{UWB_HOST_REASON_RX_THREAD_STUCK, "Rx Thread Stuck"},
	{UWB_HOST_REASON_SSR_CMD_TIMEDOUT, "SSR due to command timed out"},
	{UWB_HOST_REASON_SSR_RCVD_LARGE_PKT_FROM_SOC, "Large packet received from SOC"},
	{UWB_HOST_REASON_SSR_UNABLE_TO_WAKEUP_SOC, "Unable to wake SOC"},
	{UWB_HOST_REASON_CMD_TIMEDOUT_SOC_WAIT_TIMEOUT, "Command timedout and SOC crash wait timeout"},
	{UWB_HOST_REASON_INV_BYTES_SOC_WAIT_TIMEOUT,
		"Invalid bytes received and SOC crash wait timeout"},
	{UWB_HOST_REASON_SOC_WAKEUP_FAILED_SOC_WAIT_TIMEOUT,
		"SOC Wakeup failed and SOC crash wait timeout"},
	{UWB_HOST_REASON_SOC_CRASHED_DIAG_SSR_SOC_WAIT_TIMEOUT,
		"SOC crashed with diag initiated SSR and SOC wait timeout"},
	{UWB_HOST_REASON_NONE_SOC_WAIT_TIMEOUT, "Invalid Reason and SOC crash wait timeout"},
	{UWB_HOST_REASON_SOC_DEINIT_STUCK, "SOC Deinit Stuck"},
	{UWB_HOST_REASON_SSR_INTERNAL_CMD_TIMEDOUT, "SSR due to internal Command timeout"},
	{UWB_HOST_REASON_FAILED_TO_SEND_INTERNAL_CMD, "Failed to send internal command"},
	{UWB_HOST_REASON_SSR_SLEEP_IND_NOT_RCVD, "Failed to receive SLEEP IND during close"},
	{UWB_HOST_REASON_PERI_CRASH_ON_OTHER_SS, "Peri SOC crashed detected on BT SS"},
	{UWB_HOST_REASON_DIAG_LOG_API_STUCK, "DIAG log API stuck"}
};

static UwbSecondaryReasonMap uwbSecReasonMap[] = {
	{ UWB_SOC_REASON_DEFAULT, "Default"},
	{ UWB_SOC_REASON_TX_RX_INVALID_PKT, "Tx/Rx Inavlid Packet"},
	{ UWB_SOC_REASON_TX_RX_INVALID_PKT_LENE, "Tx/Rx Invalid Pkt Len"},
	{ UWB_SOC_REASON_TX_RX_OVERFLOW_BUFF, "Tx/Rx Overflow Buffer"},
	{ UWB_SOC_REASON_UNKNOWN, "Unknown"},
	{ UWB_SOC_REASON_TX_RX_INVALID_PKT_FATAL, "Tx/Rx invalid packet fatal error"},
	{ UWB_SOC_REASON_TX_RX_INVALID_LEN_FATAL, "Tx/Rx invalid length fatal error"},
	{ UWB_SOC_REASON_TX_RX_OVERFLOW_BUFF, "Tx/Rx Overflow Buffer"},
	{ UWB_SOC_REASON_SW_REQUESTED, "SW Requested"},
	{ UWB_SOC_REASON_STACK_OVERFLOW, "Stack Overflow"},
	{ UWB_SOC_REASON_EXCEPTION, "Exception"},
	{ UWB_SOC_REASON_ASSERT, "Assert"},
	{ UWB_SOC_REASON_TRAP, "Trap"},
	{ UWB_SOC_REASON_OS_FATAL, "OS Fatal"},
	{ UWB_SOC_REASON_HCI_RESET, "HCI Reset"},
	{ UWB_SOC_REASON_PATCH_RESET, "Patch Reset"},
	{ UWB_SOC_REASON_ABT, "SoC Abort"},
	{ UWB_SOC_REASON_RAMMASK, "RAM MASK"},
	{ UWB_SOC_REASON_PREBARK, "PREBARK"},
	{ UWB_SOC_REASON_BUSERROR, "Bus error"},
	{ UWB_SOC_REASON_IO_FATAL, "IO fatal eror"},
	{ UWB_SOC_REASON_SSR_CMD, "SSR CMD"},
	{ UWB_SOC_REASON_POWERON, "Power ON"},
	{ UWB_SOC_REASON_WATCHDOG, "Watchdog"},
	{ UWB_SOC_REASON_RAMMASK_RGN1, "RAMMASK RGN1"},
	{ UWB_SOC_REASON_RAMMASK_RGN0, "RAMMASK RGN0"},
	{ UWB_SOC_REASON_Q6_WATCHDOG, "Q6 Watchdog"},
	{ UWB_SOC_REASON_ZEALIS_RAM_MASK_RGN0, "ZEALIS RAM MASK RGN0"},
	{ UWB_SOC_REASON_ZEALIS_RAM_MASK_RGN1, "ZEALIS RAM MASK RGN1"},
	{ UWB_SOC_REASON_APSS_RESET, "APSS reset"},
	{ UWB_SOC_REASON_TIME_RESET, "Time reset"},
	{ UWB_SOC_REASON_AUDIOSS_RESET, "Audioss reset"},
	{ UWB_SOC_REASON_HOST_WARMRESET, "Host warm reset"},
	{ UWB_SOC_REASON_HOST_NMI_INIT, "Host NMI init"},
	{ UWB_SOC_REASON_PANIC_FAULT, "Panic Fault"},
	{ UWB_SOC_REASON_EARLY_TRAP, "Early Trap"},
	{ UWB_SOC_REASON_INSTR_ADDR_MISALGIN, "Instruction Address Misalign"},
	{ UWB_SOC_REASON_INSTR_ACCESS_FAULT, "Instruction Access Fault"},
	{ UWB_SOC_REASON_ILLEGAL_INSTR, "Illegal Instruction"},
	{ UWB_SOC_REASON_BREAKPOINT_EXCEPTION, "Breakpoint Exception"},
	{ UWB_SOC_REASON_LOAD_ADDR_MISALIGN, "Load Address Misalign"},
	{ UWB_SOC_REASON_LOAD_ACCESS_FAULT, "Load Access Fault"},
	{ UWB_SOC_REASON_STORE_ADDR_MISALGN, "Store Address Misalign"},
	{ UWB_SOC_REASON_STORE_ACCESS_FAULT, "Store Access Fault"},
	{ UWB_SOC_REASON_ECALL_UMODE, "Ecall Umode"},
	{ UWB_SOC_REASON_ECALL_MMODE, "Ecall Mmode"},
	{ UWB_SOC_REASON_STACK_UNDERFLOW, "Stack Underflow"},
	{ UWB_SOC_REASON_MACHINE_EXIT_INT, "Machine Exit Int"},
	{ UWB_SOC_REASON_PERF_MONITOR_OVERFLOW, "Perf Monitor Overflow"},
	{ UWB_SOC_REASON_EXT_SUBSYS_RESET, "Ext Subsystem Reset"},
	{ UWB_SOC_REASON_IPC_STALL, "IPC Stall"},
	{ UWB_SOC_REASON_PEER_CPU0_NMI, "Crash in Peri CPU"},
	{ UWB_SOC_REASON_PEER_CPU1_NMI, "Crash in BT CPU"},
	{ UWB_SOC_REASON_PEER_CPU2_NMI, "Crash in UWB CPU"},
	{ UWB_SOC_REASON_INVALID_STACK, "Invalid Stack"},
	{ UWB_SOC_REASON_INVALID_MCI_MSG_RCVD, "Invalid MCI message received"},
	{ UWB_HOST_REASON_PERI_GETVER_SEND_STUCK, "PeriGetVerSendStuck"},
	{ UWB_HOST_REASON_UWB_GETVER_SEND_STUCK, "UwbGetVerSendStuck"},
	{ UWB_HOST_REASON_TME_GETVER_SEND_STUCK, "TmeGetVerSendStuck"},
	{ UWB_HOST_REASON_PERI_GETVER_NO_RSP_RCVD, "PeriGetVerNoRspRcvd"},
	{ UWB_HOST_REASON_UWB_GETVER_NO_RSP_RCVD, "UwbGetVerNoRspRcvd"},
	{ UWB_HOST_REASON_TME_GETVER_NO_RSP_RCVD, "TmeGetVerNoRspRcvd"},
	{ UWB_HOST_REASON_PERI_PATCH_DNLD_STUCK, "PeriPatchDnldStuck"},
	{ UWB_HOST_REASON_UWB_PATCH_DNLD_STUCK, "UwbPatchDnldStuck"},
	{ UWB_HOST_REASON_TME_PATCH_DNLD_STUCK, "TmePatchDnldStuck"},
	{ UWB_HOST_REASON_PERI_GETBOARDID_CMD_STUCK, "PeriGetBoardIdStuck"},
	{ UWB_HOST_REASON_PERI_NVM_DNLD_STUCK, "PeriNvmDnldStuck"},
	{ UWB_HOST_REASON_UWB_NVM_DNLD_STUCK, "UwbNvmDnldStuck"},
	{ UWB_HOST_REASON_PERI_RESET_STUCK, "PeriResetStuck"},
	{ UWB_HOST_REASON_UWB_RESET_STUCK, "UwbResetStuck"},
	{ UWB_HOST_REASON_TME_RESET_STUCK, "TmeResetStuck"},
	{ UWB_HOST_REASON_PERI_GETBLDINFO_CMD_STUCK, "PeriGetBldInfoCmdStuck"},
	{ UWB_HOST_REASON_UWB_GETBLDINFO_CMD_STUCK, "UwbGetBldInfoCmdStuck"},
	{ UWB_HOST_REASON_TME_GETBLDINFO_CMD_STUCK, "TmeGetBldInfoCmdStuck"},
	{ UWB_HOST_REASON_PERI_ENHLOG_CMD_STUCK, "Peri EnhLogCmdStuck"},
	{ UWB_HOST_REASON_UWB_ENHLOG_CMD_STUCK, "Uwb EnhLogCmdStuck"},
	{ UWB_HOST_REASON_DIAGINIT_STUCK, "DiagInitStuck"},
	{ UWB_HOST_REASON_DIAGDEINIT_STUCK, "DiagDeinitStuck"},
	{ UWB_HOST_REASON_FAILED_TO_SEND_CMD, "Failed to send internal cmd"},
	{ UWB_HOST_REASON_PERI_RESET_CC_NOT_RCVD, "Peri Reset Cmd CC Not Rcvd"},
	{ UWB_HOST_REASON_UWB_RESET_CC_NOT_RCVD, "UWB Reset Cmd CC Not Rcvd"},
	{ UWB_HOST_REASON_UWB_ACTIVATE_CC_NOT_RCVD, "UWB Activate Cmd CC Not Rcvd"},
	{ UWB_HOST_REASON_TME_ACTIVATE_CC_NOT_RCVD, "TME DeActivate Cmd CC Not Rcvd"},
	{ UWB_HOST_REASON_POWER_ON_REGS_STUCK, "SoC Power ON Sequence stuck"},
	{ UWB_HOST_REASON_POWER_IOCTL_STUCK, "Power driver IOCTL stuck"},
	{ UWB_HOST_REASON_RX_THREAD_START_STUCK, "RX thread start stuck"},
	{ UWB_HOST_REASON_OTP_INFO_GET_CMD_STUCK, "Get OTP info. cmd stuck"},
	{ UWB_HOST_REASON_FILE_SYSTEM_CALL_STUCK, "FILE system call stuck"},
	{ UWB_HOST_REASON_PROPERTY_GET_STUCK, "Property get call stuck"},
	{ UWB_HOST_REASON_PROPERTY_SET_STUCK, "Property set call stuck"},
	{ UWB_HOST_REASON_PERI_RAM_PATCH_READ_STUCK, "Peri RAM patch open/read stuck"},
	{ UWB_HOST_REASON_UWB_RAM_PATCH_READ_STUCK, "UWB RAM patch open/read stuck"},
	{ UWB_HOST_REASON_PERI_NVM_PATCH_READ_STUCK, "Peri NVM file open/read stuck"},
	{ UWB_HOST_REASON_UWB_NVM_PATCH_READ_STUCK, "UWB NVM file open/read stuck"},
	{ UWB_HOST_REASON_PERI_PATCH_CONFIG_CMD_STUCK, "Peri Patch config cmd stuck"},
	{ UWB_HOST_REASON_PERI_PATCH_CONFIG_FAILED, "Peri Patch config cmd failed"},
	{ UWB_HOST_REASON_UWB_PATCH_CONFIG_CMD_STUCK, "Uwb Patch config cmd stuck"},
	{ UWB_HOST_REASON_UWB_PATCH_CONFIG_CMD_FAILED, "Uwb Patch config cmd stuck"},
	{ UWB_HOST_REASON_SOC_NAME_UNKOWN, "SoC name unkown"},
	{ UWB_HOST_REASON_PERI_TLV_DOWNLOAD_FAILED, "Peri TLV/NVM download failed"},
	{ UWB_HOST_REASON_PERI_GETBLDINFO_CMD_FAILED, "Peri FW build info. cmd failed"},
	{ UWB_HOST_REASON_UWB_GETBLDINFO_CMD_FAILED, "UWB build info. cmd failed"},
	{ UWB_HOST_REASON_PERI_RESET_CMD_FAILED, "HCI Peri RESET cmd failed"},
	{ UWB_HOST_REASON_UWB_CORE_RESET_CMD_FAILED, "UWB Core RESET cmd failed"},
	{ UWB_HOST_REASON_MEMORY_ALLOCATION_FAILED, "Memory allocation failed"},
	{ UWB_HOST_REASON_READ_THREAD_START_FAILED, "Read thread start failed"},
	{ UWB_HOST_REASON_HW_FLOW_ON_FAILED, "HW Flow ON failed"},
	{ UWB_HOST_REASON_PERI_ACTIVATE_CMD_STUCK, "Peri actvate cmd stuck"},
	{ UWB_HOST_REASON_PERI_ACTIVATE_NTF_STUCK, "Peri activate ntf stuck"},
	{ UWB_HOST_REASON_PERI_ARBITRATION_CMD_STUCK, "Peri arbitration cmd stuck"},
	{ UWB_HOST_REASON_PERI_ARBITRATION_NTF_STUCK, "Peri arbitration ntf stuck"},
	{ UWB_HOST_REASON_INITIALIZATION_FAILED, "Initialization Failed"},
	{ UWB_HOST_REASON_Q2SPI_INIT_STUCK, "Q2SPI Init stuck"},
	{ UWB_HOST_REASON_Q2SPI_INIT_FAILED, "Q2SPI Init Failed"},
	{ UWB_HOST_REASON_UWB_TLV_DOWNLOAD_FAILED, "Uwb TLV/NVM download failed"},
	{ Q2SPI_REASON_DEFAULT, "Q2SPI reason Default"},
};

struct log_index {
	int init;
	int crash;
};

struct fmdOperationStruct {
	unsigned char fmdOperation;
	unsigned char socFwVer;
	short int rebootStatus;
	short int fmdCycles;
};

struct vreg_data {
	struct regulator *reg;  /* voltage regulator handle */
	const char *name;       /* regulator name */
	u32 min_vol;            /* min voltage level */
	u32 max_vol;            /* max voltage level */
	u32 load_curr;          /* current */
	bool is_enabled;        /* is this regulator enabled? */
	bool is_retention_supp; /* does this regulator support retention mode */
	struct log_index indx;  /* Index for reg. w.r.t init & crash */
	bool fmd_mode_set;
};

struct pwr_data {
	char compatible[32];
	struct vreg_data *bt_vregs;
	int bt_num_vregs;
	struct vreg_data *uwb_vregs;
	int uwb_num_vregs;
	struct vreg_data *platform_vregs;
	int platform_num_vregs;
};

struct bt_power_clk_data {
	struct clk *clk;  /* clock regulator handle */
	const char *name; /* clock name */
	bool is_enabled;  /* is this clock enabled? */
};

struct btpower_state_machine {
	struct mutex state_machine_lock;
	enum power_states power_state;
	enum retention_states retention_mode;
	enum grant_states grant_state;
	enum grant_states grant_pending;
};

#define BTPWR_MAX_REQ         BT_MAX_PWR_STATE 
/*
 * Platform data for the bluetooth power driver.
 */
struct platform_pwr_data {
	struct platform_device *pdev;
	int bt_gpio_sys_rst;                   /* Bluetooth reset gpio */
	int wl_gpio_sys_rst;                   /* Wlan reset gpio */
	int bt_gpio_sw_ctrl;                   /* Bluetooth sw_ctrl gpio */
	int bt_gpio_fmd_clk_ctrl;              /* Bluetooth fmd_clk_ctrl gpio */
	int bt_gpio_debug;                     /* Bluetooth debug gpio */
	unsigned int wlan_sw_ctrl_gpio;        /* Wlan switch control gpio*/
	int bt_gpio_resetb;                    /* BT RESETB GPIO */
#ifdef CONFIG_MSM_BT_OOBS
	int bt_gpio_dev_wake;                  /* Bluetooth bt_wake */
	int bt_gpio_host_wake;                 /* Bluetooth bt_host_wake */
	int irq;                               /* Bluetooth host_wake IRQ */
#endif
	int sw_cntrl_gpio;
	int xo_gpio_clk;                       /* XO clock gpio*/
	struct device *slim_dev;
	struct vreg_data *bt_vregs;
	struct vreg_data *uwb_vregs;
	struct vreg_data *wlan_vregs;
	struct vreg_data *platform_vregs;
	struct bt_power_clk_data *bt_chip_clk; /* bluetooth reference clock */
	int (*power_setup)(int core, int id);  /* Bluetooth power setup function */
	char compatible[32];                   /*Bluetooth SoC name */
	int bt_num_vregs;
	int uwb_num_vregs;
	int platform_num_vregs;
	struct qmp *qmp;
	struct mbox_chan *mbox_chan;
	const char *vreg_ipa;
	bool is_ganges_dt;
	int pdc_init_table_len;
	const char **pdc_init_table;
	int bt_device_type;
	bool sec_peri_feature_disable;
	int bt_sec_hw_disable;
#ifdef CONFIG_MSM_BT_OOBS
	struct file *reffilp_obs;
	struct task_struct *reftask_obs;
#endif
	struct task_struct *reftask_bt;
	struct task_struct *reftask_uwb;
	struct btpower_state_machine btpower_state;
	enum ssr_states sub_state;
	enum ssr_states wrkq_signal_state;
	struct workqueue_struct *workq;
	struct device_node *bt_of_node;
	struct device_node *uwb_of_node;
	struct work_struct bt_wq;
	struct work_struct uwb_wq;
	wait_queue_head_t rsp_wait_q[BTPWR_MAX_REQ];
	int wait_status[BTPWR_MAX_REQ];
	struct work_struct wq_pwr_voting;
	struct sk_buff_head rxq;
	struct mutex pwr_mtx;
	bool is_fmd_mode_enable;
	struct nvmem_cell *nvmem_cell_fmd_set;
	struct nvmem_cell *nvmem_cell_fmd_chg_pon;
	struct nvmem_cell *nvmem_cell_fmd_cnt2_stop;
	u32 fmd_clk_gpio_id;
};

int btpower_register_slimdev(struct device *dev);
int btpower_get_chipset_version(void);
int btpower_aop_mbox_init(struct platform_pwr_data *pdata);
int bt_aop_pdc_reconfig(struct platform_pwr_data *pdata);

#define WLAN_SW_CTRL_GPIO           "qcom,wlan-sw-ctrl-gpio"
#define BT_CMD_SLIM_TEST            0xbfac
#define BT_CMD_PWR_CTRL             0xbfad
#define BT_CMD_CHIPSET_VERS         0xbfae
#define BT_CMD_GET_CHIPSET_ID       0xbfaf
#define BT_CMD_CHECK_SW_CTRL        0xbfb0
#define BT_CMD_GETVAL_POWER_SRCS    0xbfb1
#define BT_CMD_SET_IPA_TCS_INFO     0xbfc0
#define BT_CMD_KERNEL_PANIC         0xbfc1
#define UWB_CMD_KERNEL_PANIC        0xbfc2
#define UWB_CMD_PWR_CTRL            0xbfe1
#define BT_CMD_REGISTRATION         0xbfe2
#define UWB_CMD_REGISTRATION        0xbfe3
#define BT_CMD_ACCESS_CTRL          0xbfe4
#define UWB_CMD_ACCESS_CTRL         0xbfe5
#define UWB_GET_SSR_STATE           0xbfe6
#define BT_CMD_FMD_OPERATION        0xbfb2

#ifdef CONFIG_MSM_BT_OOBS
#define BT_CMD_OBS_VOTE_CLOCK		0xbfd1
/**
 * enum btpower_obs_param: OOBS low power param
 * @BTPOWER_OBS_CLK_OFF: Transport bus is no longer acquired
 * @BTPOWER_OBS_CLK_ON: Acquire transport bus for either transmitting or receiving
 * @BTPOWER_OBS_DEV_OFF: Bluetooth is released because of no more transmission
 * @BTPOWER_OBS_DEV_ON: Wake up the Bluetooth controller for transmission
 */
enum btpower_obs_param {
	BTPOWER_OBS_CLK_OFF = 0,
	BTPOWER_OBS_CLK_ON,
	BTPOWER_OBS_DEV_OFF,
	BTPOWER_OBS_DEV_ON,
};
#endif
#endif /* __LINUX_BLUETOOTH_POWER_H */
