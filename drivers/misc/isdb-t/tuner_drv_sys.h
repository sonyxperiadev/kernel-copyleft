/******************************************************************************
 *
 *  file name       : tuner_drv_sys.h
 *  brief note      : The Header for Driver Public Presentation
 *
 *  creation data   : 2011.08.01
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 322                        $ Revision of Last commit
 *  $Date:: 2011-10-26 13:33:02 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
 *              Copyright (C) 2012 Sony Mobile Communications AB.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/08/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/
#ifndef _TUNER_DRV_SYS_H
#define _TUNER_DRV_SYS_H

/* IOCTL setting parameter */
#define TUNER_IOC_MAGIC 'd'
#define TUNER_IOCTL_VALGET        _IOW(TUNER_IOC_MAGIC, 1, struct ioctl_cmd)
#define TUNER_IOCTL_VALSET        _IOR(TUNER_IOC_MAGIC, 2, struct ioctl_cmd)
#define TUNER_IOCTL_VALGET_EVENT  _IOR(TUNER_IOC_MAGIC, 3, struct ioctl_cmd)
#define TUNER_IOCTL_VALSET_POWER  _IOR(TUNER_IOC_MAGIC, 4, struct ioctl_cmd)
#define TUNER_IOCTL_VALSET_EVENT  _IOW(TUNER_IOC_MAGIC, 5, struct ioctl_cmd)
#define TUNER_IOCTL_VALREL_EVENT  _IOW(TUNER_IOC_MAGIC, 6, struct ioctl_cmd)
#define TUNER_IOCTL_ANT_SWITCH    _IOR(TUNER_IOC_MAGIC, 7, struct ioctl_cmd)
#define TUNER_IOCTL_ANT_SEARCH    _IOR(TUNER_IOC_MAGIC, 8, struct ioctl_cmd)
#define TUNER_IOCTL_CIRCUIT_INFO  _IOR(TUNER_IOC_MAGIC, 9, struct ioctl_cmd)
#define TUNER_IOCTL_GET_CURRENT_ANT  _IOR(TUNER_IOC_MAGIC, 10, struct ioctl_cmd)
/* Parameters for power control */
enum TUNER_DRV_CTL {
	TUNER_DRV_CTL_POWON,
	TUNER_DRV_CTL_POWOFF,
	TUNER_DRV_CTL_RESET
};

/* enabit enable setting */
#define SET_ENABIT 0xFF
#define AUTO_ANT_SEARCH_ON 1
#define AUTO_ANT_SEARCH_OFF 0

#define AUTO_ANT_SYNC 1
#define AUTO_ANT_NOT_SYNC 0

#define AUTO_ANT_PKT_ERR 1
#define AUTO_ANT_NO_PKT_ERR 0

/*antmode*/
enum ANTMODE {
	ANTMODE_WHIP,
	ANTMODE_EARPHONE,
	ANTMODE_USB,
	ANTMODE_CHARGER,
	ANTMODE_AUTO,
	ANTMODE_NOTUSE
};
enum ANTCONNECT {
	CONNECT_USB,
	CONNECT_WHIP,
	CONNECT_EARPHONE
};

/*tuner sync*/
enum TUNERSYNC {
	TUNER_SYNC,
	TUNER_SYNC_SEQ
};

/*Layer for fullseg*/
#define LAYER_A 1
#define LAYER_B 2
#define LAYER_C 3
/*ant_switch_status*/
#define PER_SEL 0
#define BER_SEL 1
#define ANT_SEARCH_AUTO_ON 1
#define ANT_SEARCH_AUTO_OFF 0
/*ant_switch_paramators*/
#define DEF_RSSI1 -100
#define DEF_RSSI2 -100
#define DEF_RSSI3 -100
#define ANT_NUM 2
#define ANT_SET_WAIT 500 /*ms*/
#define BPER_STABLE_WAIT1 1200 /*ms*/
#define BPER_STABLE_WAIT2 1200 /*ms*/
#define BPER_POLE_WAIT 200 /*ms*/
#define NON_ERROR_COUNT 5
#define ERROR_COUNT 5
#define ERROR_COMPARING_COUNT 10
#define BPER_TIME_OUT_COUNT 1
#define RSSI_MIN_THREATHOLD 0
#define F_LYR LAYER_B
#define INT_MASK_WAIT 0

/* Structure for register read/write */
struct _tuner_data_rw {
	unsigned short slave_adr;
	unsigned short adr;
	unsigned short sbit;
	unsigned short ebit;
	unsigned short param;
	unsigned short enabit;
};

/* Device IOCTL data structure */
struct ioctl_cmd {
	unsigned int reg;
	unsigned int offset;
	unsigned int val;
};

#endif/* _TUNER_DRV_SYS_H */
