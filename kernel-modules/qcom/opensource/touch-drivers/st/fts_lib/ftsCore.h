/* SPDX-License-Identifier: GPL-2.0-only */
/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				 **
  **************************************************************************
  *                                                                        *
  *			FTS Core definitions				 **
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
 * \file ftsCore.h
 * \brief Contains all the definitions and structs of Core functionalities
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef FTS_CORE_H
#define FTS_CORE_H

#include "ftsHardware.h"
#include "ftsSoftware.h"
#include "../fts.h"

/* HW DATA */
#define GPIO_NOT_DEFINED	-1	/* /< value assumed by reset_gpio when
					 * the reset pin of the IC is not
					 * connected */


#define ADDR_SIZE_HW_REG	BITS_32	/* /< value of AddrSize for Hw register
					 * in FTI @see AddrSize */


#define DATA_HEADER		4	/* /< size in byte of the header loaded
					 * with the data in the frambuffer */

/**
  * Type of CRC errors
  */
typedef enum {
	CRC_CODE	= 1,	/* /< CRC in the code section */
	CRC_CONFIG	= 2,	/* /< CRC in the config section */
	CRC_CX		= 3,	/* /< CRC in the cx section */
	CRC_PANEL	= 4	/* /< CRC in the panel section */
} CRC_Error;

/* RETRY MECHANISM */
#define RETRY_MAX_REQU_DATA		2	/* /< Max number of attemps
						 * performed
						  * when requesting data */
#define RETRY_SYSTEM_RESET		3	/* /< Max number of attemps
						 * performed
						  * to reset the IC */

int initCore(struct fts_ts_info *info);
void setResetGpio(int gpio);
int fts_system_reset(struct fts_ts_info *info);
int isSystemResettedUp(struct fts_ts_info *info);
int isSystemResettedDown(struct fts_ts_info *info);
void setSystemResetedUp(struct fts_ts_info *info, int val);
void setSystemResetedDown(struct fts_ts_info *info, int val);
int pollForEvent(struct fts_ts_info *info, int *event_to_search, int event_bytes, u8 *readData, int
		 time_to_wait);
int checkEcho(struct fts_ts_info *info, u8 *cmd, int size);
int setScanMode(struct fts_ts_info *info, u8 mode, u8 settings);
int setFeatures(struct fts_ts_info *info, u8 feat, u8 *settings, int size);
int defaultSysInfo(struct fts_ts_info *info, int i2cError);
int writeSysCmd(struct fts_ts_info *info, u8 sys_cmd, u8 *sett, int size);
int readSysInfo(struct fts_ts_info *info, int request);
int readConfig(struct fts_ts_info *info, u16 offset, u8 *outBuf, int len);
int writeConfig(struct fts_ts_info *info, u16 offset, u8 *data, int len);
int fts_disableInterrupt(struct fts_ts_info *info);
int fts_disableInterruptNoSync(struct fts_ts_info *info);
int fts_resetDisableIrqCount(struct fts_ts_info *info);
int fts_enableInterrupt(struct fts_ts_info *info);
int fts_crc_check(struct fts_ts_info *info);
int requestSyncFrame(struct fts_ts_info *info, u8 type);
int saveMpFlag(struct fts_ts_info *info, u8 mpflag);

#endif	/* FTS_CORE_H */
