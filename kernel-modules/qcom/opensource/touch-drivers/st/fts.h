/* SPDX-License-Identifier: GPL-2.0-only */
/*
  * fts.c
  *
  * FTS Capacitive touch screen controller (FingerTipS)
  *
  * Copyright (C) 2016-2019, STMicroelectronics
  * Authors: AMG(Analog Mems Group)
  *
  *             marco.cali@st.com
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
  * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
  * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
  *THE
  * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
  *
  * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
  */

/*!
 * \file fts.h
 * \brief Contains all the definitions and structs used generally by the driver
 */

#ifndef _LINUX_FTS_I2C_H_
#define _LINUX_FTS_I2C_H_

#include <linux/device.h>
#include "fts_lib/ftsSoftware.h"
#include "fts_lib/ftsHardware.h"


/****************** CONFIGURATION SECTION ******************/
/** @defgroup conf_section	 Driver Configuration Section
 * Settings of the driver code in order to suit the HW set up and the
 *application behavior
 * @{
 */
/* **** CODE CONFIGURATION **** */
#define FTS_TS_DRV_NAME		"fts"	/* /< driver name */
#define FTS_TS_DRV_VERSION	"5.2.20" /* /< driver version string format */
#define FTS_TS_DRV_VER		0x05021400	/* driver version u32 format */

#define DEBUG	/* /< define to print more logs in the kernel log and better
		 * follow the code flow */

#define DRIVER_TEST	/* /< if defined allow to use and test special functions
			 * of the driver and fts_lib from comand shell (usefull
			 * for enginering/debug operations) */


/* If both COMPUTE_INIT_METHOD and PRE_SAVED_METHOD are not defined,
 * driver will be automatically configured as GOLDEN_VALUE_METHOD */
#ifndef COMPUTE_INIT_METHOD
		#define PRE_SAVED_METHOD /* Pre-Saved Method used
					  * during production */
#endif

#ifdef FW_H_FILE
	#define FW_SIZE_NAME	myArray_size	/* /< name of the variable in
						 * the FW header file which
						 * specified the dimension of
						 * the FW data array */
	#define FW_ARRAY_NAME	myArray	/* /< name of the variable in the FW
					 * header file which specified the FW
					 * data array */
/* #define FW_UPDATE_ON_PROBE */
 /* if defined the FW update will be execute on the probe, if not it will be
 * executed EXP_FN_WORK_DELAY_MS ms after the probe is completed */
#endif

#ifndef FW_UPDATE_ON_PROBE
/* #define LIMITS_H_FILE */
/* include the Production Limit File as header file, can be commented to use a
 * .csv file instead */
#ifdef LIMITS_H_FILE
	#define LIMITS_SIZE_NAME	myArray2_size	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * dimension of the
							 * limits data array */
	#define LIMITS_ARRAY_NAME	myArray2	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * limits data array */
#endif
#else
/* if execute fw update in the probe the limit file must be a .h */
#define LIMITS_H_FILE	/* /< include the Production Limit File as header file,
			 * DO NOT COMMENT! */
#define LIMITS_SIZE_NAME		myArray2_size	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * dimension of the
							 * limits data array */
#define LIMITS_ARRAY_NAME		myArray2	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * limits data array */
#endif

#define USE_ONE_FILE_NODE
/* allow to enable/disable all the features just using one file node */

#ifndef FW_UPDATE_ON_PROBE
#define EXP_FN_WORK_DELAY_MS 1000	/* /< time in ms elapsed after the probe
					 * to start the work which execute FW
					 * update and the Initialization of the
					 * IC */
#endif

/* **** END **** */


/* **** FEATURES USED IN THE IC **** */
/* Enable the support of keys */
/* #define PHONE_KEY */

#ifdef GESTURE_MODE
	#define USE_GESTURE_MASK	/* /< the gestures to select are
					 * referred using a gesture bitmask
					 * instead of their gesture IDs */
#endif


#define CHARGER_MODE	/* /< enable the support to charger mode feature
			 * (comment to disable) */


/* **** END **** */


/* **** PANEL SPECIFICATION **** */
#define X_AXIS_MAX	1440	/* /< Max X coordinate of the display */
#define X_AXIS_MIN	0	/* /< min X coordinate of the display */
#define Y_AXIS_MAX	3200	/* /< Max Y coordinate of the display */
#define Y_AXIS_MIN	0	/* /< min Y coordinate of the display */

#define PRESSURE_MIN	0	/* /< min value of pressure reported */
#define PRESSURE_MAX	127	/* /< Max value of pressure reported */

#define DISTANCE_MIN	0	/* /< min distance between the tool and the
				 * display */
#define DISTANCE_MAX	127	/* /< Max distance between the tool and the
				 * display */

#define TOUCH_ID_MAX	10	/* /< Max number of simoultaneous touches
				 * reported */

#define AREA_MIN	PRESSURE_MIN	/* /< min value of Major/minor axis
					 * reported */
#define AREA_MAX	PRESSURE_MAX	/* /< Man value of Major/minor axis
					 * reported */
/* **** END **** */
/**@}*/
/*********************************************************/


/*
  * Configuration mode
  *
  * bitmask which can assume the value defined as features in ftsSoftware.h or
  * the following values
  */

/** @defgroup mode_section	 IC Status Mode
  * Bitmask which keeps track of the features and working mode enabled in the
  * IC.
  * The meaning of the the LSB of the bitmask must be interpreted considering
  * that the value defined in @link feat_opt Feature Selection Option @endlink
  * correspond to the position of the corresponding bit in the mask
  * @{
  */
#define MODE_NOTHING 0x00000000	/* /< nothing enabled (sense off) */
#define MODE_ACTIVE(_mask, _sett)	(_mask |= (SCAN_MODE_ACTIVE << 24) | \
						  (_sett << 16))
/* /< store the status of scan mode active and its setting */
#define MODE_LOW_POWER(_mask, _sett)   (_mask |= (SCAN_MODE_LOW_POWER << 24) | \
						  (_sett << 16))
/* /< store the status of scan mode low power and its setting */
#define IS_POWER_MODE(_mask, _mode)	((_mask&(_mode<<24)) != 0x00)
/* /< check the current mode of the IC */

/** @}*/

#define CMD_STR_LEN	32	/* /< max number of parameters that can accept
				 * the MP file node (stm_fts_cmd) */

#define TSP_BUF_SIZE	PAGE_SIZE	/* /< max number of bytes printable on
					 * the shell in the normal file nodes */

/* CHIP INFO */
/** @defgroup system_info	System Info
 * System Info Data collect the most important information about hw and fw
 * @{
 */
/* Size in bytes of System Info data */
#define SYS_INFO_SIZE			216	/* Num bytes of die info */
#define DIE_INFO_SIZE			16	/* Num bytes of external release
						 * in config
						 */
#define EXTERNAL_RELEASE_INFO_SIZE	8	/* Num bytes of release info in
						 * sys info
						 *  (first bytes are external
						 *release)
						 */
#define RELEASE_INFO_SIZE		(EXTERNAL_RELEASE_INFO_SIZE)

/** @}*/

/** @addtogroup system_info
 * @{
 */

/**
 * Struct which contains fundamental information about the chip and its
 *configuration
 */
struct SysInfo {
	u16 u16_apiVer_rev;	/* /< API revision version */
	u8 u8_apiVer_minor;	/* /< API minor version */
	u8 u8_apiVer_major;	/* /< API major version */
	u16 u16_chip0Ver;	/* /< Dev0 version */
	u16 u16_chip0Id;	/* /< Dev0 ID */
	u16 u16_chip1Ver;	/* /< Dev1 version */
	u16 u16_chip1Id;	/* /< Dev1 ID */
	u16 u16_fwVer;	/* /< Fw version */
	u16 u16_svnRev;	/* /< SVN Revision */
	u16 u16_cfgVer;	/* /< Config Version */
	u16 u16_cfgProjectId;	/* /< Config Project ID */
	u16 u16_cxVer;	/* /< Cx Version */
	u16 u16_cxProjectId;	/* /< Cx Project ID */
	u8 u8_cfgAfeVer;	/* /< AFE version in Config */
	u8 u8_cxAfeVer;	/* /< AFE version in CX */
	u8 u8_panelCfgAfeVer;	/* /< AFE version in PanelMem */
	u8 u8_protocol;	/* /< Touch Report Protocol */
	u8 u8_dieInfo[DIE_INFO_SIZE];	/* /< Die information */
	u8 u8_releaseInfo[RELEASE_INFO_SIZE];	/* /< Release information */
	u32 u32_fwCrc;	/* /< Crc of FW */
	u32 u32_cfgCrc;	/* /< Crc of config */
	u8 u8_mpFlag; /* /< MP Flag */
	u8 u8_ssDetScanSet; /* /< Type of Detect Scan Selected */

	u16 u16_scrResX;	/* /< X resolution on main screen */
	u16 u16_scrResY;	/* /< Y resolution on main screen */
	u8 u8_scrTxLen;	/* /< Tx length */
	u8 u8_scrRxLen;	/* /< Rx length */
	u8 u8_keyLen;	/* /< Key Len */
	u8 u8_forceLen;	/* /< Force Len */
	u32 u32_productionTimestamp;	/* /< Production Timestamp */

	u16 u16_dbgInfoAddr;	/* /< Offset of debug Info structure */

	u16 u16_msTchRawAddr;	/* /< Offset of MS touch raw frame */
	u16 u16_msTchFilterAddr;	/* /< Offset of MS touch filter frame */
	u16 u16_msTchStrenAddr;	/* /< Offset of MS touch strength frame */
	u16 u16_msTchBaselineAddr;	/* /< Offset of MS touch baseline frame
					 */

	u16 u16_ssTchTxRawAddr;	/* /< Offset of SS touch force raw frame */
	u16 u16_ssTchTxFilterAddr;	/* /< Offset of SS touch force filter
					 * frame
					 */
	u16 u16_ssTchTxStrenAddr;	/* /< Offset of SS touch force strength
					 * frame
					 */
	u16 u16_ssTchTxBaselineAddr;	/* /< Offset of SS touch force baseline
					 * frame
					 */

	u16 u16_ssTchRxRawAddr;	/* /< Offset of SS touch sense raw frame */
	u16 u16_ssTchRxFilterAddr;	/* /< Offset of SS touch sense filter
					 * frame
					 */
	u16 u16_ssTchRxStrenAddr;	/* /< Offset of SS touch sense strength
					 * frame
					 */
	u16 u16_ssTchRxBaselineAddr;	/* /< Offset of SS touch sense baseline
					 * frame
					 */

	u16 u16_keyRawAddr;	/* /< Offset of key raw frame */
	u16 u16_keyFilterAddr;	/* /< Offset of key filter frame */
	u16 u16_keyStrenAddr;	/* /< Offset of key strength frame */
	u16 u16_keyBaselineAddr;	/* /< Offset of key baseline frame */

	u16 u16_frcRawAddr;	/* /< Offset of force touch raw frame */
	u16 u16_frcFilterAddr;	/* /< Offset of force touch filter frame */
	u16 u16_frcStrenAddr;	/* /< Offset of force touch strength frame */
	u16 u16_frcBaselineAddr;	/* /< Offset of force touch baseline
					 * frame
					 */

	u16 u16_ssHvrTxRawAddr;	/* /< Offset of SS hover Force raw frame */
	u16 u16_ssHvrTxFilterAddr;	/* /< Offset of SS hover Force filter
					 * frame
					 */
	u16 u16_ssHvrTxStrenAddr;	/* /< Offset of SS hover Force strength
					 * frame
					 */
	u16 u16_ssHvrTxBaselineAddr;	/* /< Offset of SS hover Force baseline
					 * frame
					 */

	u16 u16_ssHvrRxRawAddr;	/* /< Offset of SS hover Sense raw frame */
	u16 u16_ssHvrRxFilterAddr;	/* /< Offset of SS hover Sense filter
					 * frame
					 */
	u16 u16_ssHvrRxStrenAddr;	/* /< Offset of SS hover Sense strength
					 * frame
					 */
	u16 u16_ssHvrRxBaselineAddr;	/* /< Offset of SS hover Sense baseline
					 * frame
					 */

	u16 u16_ssPrxTxRawAddr;	/* /< Offset of SS proximity force raw frame */
	u16 u16_ssPrxTxFilterAddr;	/* /< Offset of SS proximity force
					 * filter frame
					 */
	u16 u16_ssPrxTxStrenAddr;	/* /< Offset of SS proximity force
					 * strength frame
					 */
	u16 u16_ssPrxTxBaselineAddr;	/* /< Offset of SS proximity force
					 * baseline frame
					 */

	u16 u16_ssPrxRxRawAddr;	/* /< Offset of SS proximity sense raw frame */
	u16 u16_ssPrxRxFilterAddr;	/* /< Offset of SS proximity sense
					 * filter frame
					 */
	u16 u16_ssPrxRxStrenAddr;	/* /< Offset of SS proximity sense
					 * strength frame
					 */
	u16 u16_ssPrxRxBaselineAddr;	/* /< Offset of SS proximity sense
					 * baseline frame
					 */

	u16 u16_ssDetRawAddr;		/* /< Offset of SS detect raw frame */
	u16 u16_ssDetFilterAddr;	/* /< Offset of SS detect filter
					 * frame
					 */
	u16 u16_ssDetStrenAddr;		/* /< Offset of SS detect strength
					 * frame
					 */
	u16 u16_ssDetBaselineAddr;	/* /< Offset of SS detect baseline
					 * frame
					 */
};

/** @}*/

/**
 * Struct which contains information about the HW platform and set up
 */
struct fts_hw_platform_data {
	int (*power)(bool on);
	int irq_gpio;	/* /< number of the gpio associated to the interrupt pin */
	unsigned int irq_flags; /* /< irq trigger type */
	int reset_gpio;	/* /< number of the gpio associated to the reset pin */
	const char *vdd_reg_name;	/* /< name of the VDD regulator */
	const char *avdd_reg_name;	/* /< name of the AVDD regulator */
};

/*
 * Forward declaration
 */
struct fts_ts_info;
extern char tag[8];	/* /< forward the definition of the label used
			 * to print the log in the kernel log
			 */

/*
 * Dispatch event handler
 */
typedef void (*event_dispatch_handler_t)
	(struct fts_ts_info *info, unsigned char *data);

/**
 * FTS capacitive touch screen device information
 * - dev             Pointer to the structure device \n
 * - i2c_client      client structure for I2C\n
 * - spi_client      client structure for SPI\n
 * - input_dev       Input device structure \n
 * - work            Work thread \n
 * - event_wq        Event queue for work thread \n
 * - event_dispatch_table  Event dispatch table handlers \n
 * - attrs           SysFS attributes \n
 * - mode            Device operating mode (bitmask) \n
 * - touch_id        Bitmask for touch id (mapped to input slots) \n
 * - stylus_id       Bitmask for tracking the stylus touches (mapped using the touchId) \n
 * - timer           Timer when operating in polling mode \n
 * - power           Power on/off routine \n
 * - board           HW info retrieved from device tree \n
 * - vdd_reg         DVDD power regulator \n
 * - avdd_reg        AVDD power regulator \n
 * - resume_bit      Indicate if screen off/on \n
 * - fwupdate_stat   Store the result of a fw update triggered by the host \n
 * - fb_notifier     Used for be notified from a suspend/resume event \n
 * - notifier_cookie saved cookie during panel event notification \n
 * - sensor_sleep    true suspend was called, false resume was called \n
 * - wakesrc         Wakeup Source struct \n
 * - input_report_mutex  mutex for handling the pressure of keys \n
 * - series_of_switches  to store the enabling status of a particular feature from the host \n
 * - ready           Touch ready indicator \n
 * - bus_type        Indicate touch bus type \n
 * - irq             saved touch irq number \n
 */
struct fts_ts_info {
	struct device            *dev;	/* /< Pointer to the structure device */
	struct i2c_client        *i2c_client;	/* /< I2C client structure */
	struct spi_device        *spi_client;	/* /< SPI client structure */
	struct input_dev         *input_dev;	/* /< Input device structure */
	struct work_struct work;	/* /< Event work thread */
	struct work_struct suspend_work;	/* /< Suspend work thread */
	struct work_struct resume_work;	/* /< Resume work thread */
	struct workqueue_struct  *event_wq;	/* /< Workqueue used for event
						 * handler, suspend and resume
						 * work threads
						 */

#ifndef FW_UPDATE_ON_PROBE
	struct delayed_work fwu_work;	/* /< Delayed work thread for fw update
					 * process
					 */
	struct workqueue_struct    *fwu_workqueue;	/* /< Fw update work
							 * queue
							 */
#endif
	event_dispatch_handler_t *event_dispatch_table;	/* /< Event dispatch
							 * table handlers
							 */

	struct attribute_group attrs;	/* /< SysFS attributes */

	unsigned int mode;	/* /< Device operating mode (bitmask: msb
				 * indicate if active or lpm)
				 */
	unsigned long touch_id;	/* /< Bitmask for touch id (mapped to input
				 * slots)
				 */
#ifdef STYLUS_MODE
	unsigned long stylus_id;	/* /< Bitmask for tracking the stylus
					 * touches (mapped using the touchId)
					 */
#endif


	struct fts_hw_platform_data *board;	/* /< HW info retrieved from
						 * device tree
						 */
	struct regulator *vdd_reg;	/* /< DVDD power regulator */
	struct regulator *avdd_reg;	/* /< AVDD power regulator */


	int resume_bit;	/* /< Indicate if screen off/on */
	int fwupdate_stat;	/* /< Store the result of a fw update triggered
				 * by the host
				 */


#if defined(CONFIG_DRM)
	struct notifier_block fb_notifier;
	void *notifier_cookie;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notifier;	/* /< Used for be notified from a
						 * suspend/resume event
						 */
#endif

	bool sensor_sleep;	/* /< if true suspend was called while if false
				 * resume was called
				 */
	struct wakeup_source *wakesrc;	/* Wake Lock struct */

	/* input lock */
	struct mutex input_report_mutex;	/* /< mutex for handling the report
						 * of the pressure of keys
						 */

	/* switches for features */
	int gesture_enabled;	/* /< if set, the gesture mode will be enabled
				 * during the suspend
				 */
	int glove_enabled;	/* /< if set, the glove mode will be enabled
				 * when allowed
				 */
	int charger_enabled;	/* /< if set, the charger mode will be enabled
				 * when allowed
				 */
	int stylus_enabled;	/* /< if set, the stylus mode will be enabled
				 * when allowed
				 */
	int cover_enabled;	/* /< if set, the cover mode will be enabled
				 * when allowed
				 */
	int grip_enabled;	/* /< if set, the grip mode mode will be enabled
				 * when allowed
				 */
	bool qts_en;	/* /< indicate whether qts is enabled or not */
	struct mutex tui_transition_lock;	/* /< mutex for trusted input operation */

	bool ready;	/* /< indicate whether probe finished and touch is ready */
	u16 bus_type;	/* /< bus interface used for touch IC */
	bool is_primary; /* /< Indicates whether it is the primary touch */
	int irq;	/* /< saved irq number for touch */

	struct SysInfo systemInfo;	/* /< System Info variable */
	int reset_gpio;	/* /< gpio number of the rest
			 * pin, the value is
			 *  GPIO_NOT_DEFINED if the
			 * reset pin is not connected
			 */
	int system_reseted_up;	/* /< flag checked during resume to
				 * understand if there was a system
				 * reset and restore the proper state
				 */
	int system_reseted_down;	/* /< flag checked during suspend to
					 * understand if there was a system
					 * reset and restore the proper state
					 */
	int disable_irq_count;	/* /< count the number of call to
				 * disable_irq, start with 1 because at
				 * the boot IRQ are already disabled
				 */
	struct mutex fts_int;	/* /< mutex to control the access to the
				 * disable_irq_counter
				 */
	struct pinctrl *pinctrl;	/* /< pinctrl for level shifter and touch GPIOs */
	struct pinctrl_state *pins_active;	/* /< active pinctrl state */
	struct pinctrl_state *pins_suspend;	/* /< suspend pinctrl state */
};

enum DUAL_TOUCH_TYPE {
	PRIMARY_TOUCH_IDX,
	SECONDARY_TOUCH_IDX,
	MAX_SUPPORTED_TOUCH_PANELS,
};

int fts_get_touch_type(struct device_node *np);

int fts_chip_powercycle(struct fts_ts_info *info);
extern int input_register_notifier_client(struct notifier_block *nb);
extern int input_unregister_notifier_client(struct notifier_block *nb);

/* export declaration of functions in fts_proc.c */
extern int fts_proc_init(void);
extern int fts_proc_remove(void);

#endif
