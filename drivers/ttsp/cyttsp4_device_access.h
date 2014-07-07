/*
 * cyttsp4_device_access.h
 * Cypress TrueTouch(TM) Standard Product V4 Device Access module.
 * Configuration and Test command/status user interface.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2012 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#ifndef _LINUX_CYTTSP4_DEVICE_ACCESS_H
#define _LINUX_CYTTSP4_DEVICE_ACCESS_H

#define CYTTSP4_DEVICE_ACCESS_NAME "cyttsp4_device_access"

#define CYTTSP4_INPUT_ELEM_SZ (sizeof("0xHH") + 1)
#define CYTTSP4_TCH_PARAM_SIZE_BLK_SZ 128

/* Timeout values in milliseconds */
#define CY_DA_REQUEST_EXCLUSIVE_TIMEOUT	500
#define CY_DA_COMMAND_COMPLETE_TIMEOUT	500

/* Command Sizes and expected Return Sizes */
/*
 * 1: Command
 * 1: Parameter ID
 */
#define CY_CMD_OP_GET_PARA_CMD_SZ	2

/*
 * 1: Parameter ID
 * 1: Size of data type
 * 4: read data
 */
#define CY_CMD_OP_GET_PARA_RET_SZ	6

/*
 * 1: Command
 * 1: Parameter ID
 * 1: Size of data type
 * 4: write data
 */
#define CY_CMD_OP_SET_PARA_CMD_SZ	7

/*
 * 1: Command
 * 1: Parameter ID
 */
#define CY_CMD_OP_SET_PARA_RET_SZ	2

/*
 * 1: Command
 */
#define CY_CMD_CAT_GET_CFG_BLK_SZ_CMD_SZ 1

/*
 * 2: Row Size
 */
#define CY_CMD_CAT_GET_CFG_BLK_SZ_RET_SZ 2

/*
 * 1: Command
 * 2: Offset
 * 2: Length
 * 1: EBID
 */
#define CY_CMD_CAT_READ_CFG_BLK_CMD_SZ 6

/*
 * 1: Complete bit
 * 1: EBID
 * 2: Read Length
 * 1: Reserved
 * N: EBID Row Size
 * 2: CRC
 */
#define CY_CMD_CAT_READ_CFG_BLK_RET_SZ 7
#define CY_CMD_CAT_READ_CFG_BLK_RET_HDR_SZ 5

struct cyttsp4_device_access_platform_data {
	char const *device_access_dev_name;
};

/*
 * 1: Command
 */
#define CY_CMD_CAT_EXEC_SCAN_CMD_SZ 1

/*
 * 1: Status
 */
#define CY_CMD_CAT_EXEC_SCAN_RET_SZ 1

/*
 * 1: Command
 * 2: Read offset
 * 2: Num Element
 * 1: Data ID
 */
#define CY_CMD_CAT_RET_PANEL_DATA_CMD_SZ 6

/*
 * 1: Status
 * 1: Data ID
 * 2: Num Element
 * 1: Data ID
 */
#define CY_CMD_CAT_RET_PANEL_DATA_RET_SZ 5

/*
 * 1: Command
 * 1: Mode
 */
#define CY_CMD_CAT_IDACS_CMD_SZ 2

/*
 * 1: Status
 */
#define CY_CMD_CAT_IDACS_RET_SZ 1

/*
 * 1: Command
 * 2: Read offset
 * 2: Read length
 * 1: Data ID
 */
#define CY_CMD_CAT_RET_IDACS_DATA_CMD_SZ 6

/*
 * 1: Status
 * 1: Data ID
 * 2: Read length
 * 1: Data format
 */
#define CY_CMD_CAT_RET_IDACS_DATA_RET_SZ 5

#define CY_CMD_CAT_MUTUAL_CAP_IDAC_DATA_NUM 265
#define CY_CMD_CAT_SELF_CAP_IDAC_DATA_NUM 36
#define CY_CMD_CAT_RET_MUTUAL_CAP_IDAC_DATA 0
#define CY_CMD_CAT_RET_SELF_CAP_IDAC_DATA 1


#define CY_CMD_IN_DATA_OFFSET_VALUE 0

#define CY_CMD_OUT_STATUS_OFFSET 0
#define CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_H 2
#define CY_CMD_RET_PNL_OUT_ELMNT_SZ_OFFS_L 3
#define CY_CMD_RET_PNL_OUT_DATA_FORMAT_OFFS 4

#define CY_CMD_RET_PANEL_ELMNT_SZ_MASK 0x07

#define CY_CMD_STATUS_SUCCESS 0

#define I2C_BUF_MAX_SIZE 256

enum scanDataTypeList {
	CY_MUT_RAW,
	CY_MUT_BASE,
	CY_MUT_DIFF,
	CY_SELF_RAW,
	CY_SELF_BASE,
	CY_SELF_DIFF,
	CY_BAL_RAW,
	CY_BAL_BASE,
	CY_BAL_DIFF,
};

#endif /* _LINUX_CYTTSP4_DEVICE_ACCESS_H */
