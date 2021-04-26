/**************************************************************************//**
 *
 *  @file		tuner_drv_sys.h
 *  @brief		The public header for the mm_tuner55x driver
 *
 ****************************************************************************//*
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************
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
 ******************************************************************************/
/*
 * Copyright (C) 2019 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
/*..+....1....+....2....+....3....+....4....+....5....+....6....+....7....+...*/
#ifndef _TUNER_DRV_SYS_H
#define _TUNER_DRV_SYS_H

/******************************************************************************
 * include
 ******************************************************************************/
#include "tuner_drv_config.h"

/******************************************************************************
 * define
 ******************************************************************************/
/* IOCTL parameters */
#define TUNER_IOC_MAGIC 'd'
#define TUNER_IOCTL_VALGET	_IOWR(TUNER_IOC_MAGIC, 1, union _tuner_data_rw)
#define TUNER_IOCTL_VALSET	_IOW(TUNER_IOC_MAGIC, 2, union _tuner_data_rw)
#define TUNER_IOCTL_EVENT_GET _IOWR(TUNER_IOC_MAGIC, 5, union _tuner_data_event)
#define TUNER_IOCTL_EVENT_SET _IOW(TUNER_IOC_MAGIC, 6, union _tuner_data_event)
#define TUNER_IOCTL_EVENT_REL _IOW(TUNER_IOC_MAGIC, 7, union _tuner_data_event)
#define TUNER_IOCTL_CNTSET	_IOW(TUNER_IOC_MAGIC, 3, union _tuner_data_rw)
#define TUNER_IOCTL_CNTGET	_IOWR(TUNER_IOC_MAGIC, 4, union _tuner_data_rw)
#define TUNER_IOCTL_TSIF_START _IOW(TUNER_IOC_MAGIC, 10,\
							struct _tuner_data_tsif)
#define TUNER_IOCTL_TSIF_STOP _IO(TUNER_IOC_MAGIC,  11)
#define TUNER_IOCTL_TSIF_PKTSIZE _IOR(TUNER_IOC_MAGIC, 12, unsigned int)
#define TUNER_IOCTL_GETVER		_IOR(TUNER_IOC_MAGIC, 19, unsigned int)
#define TUNER_IOCTL_CPATHID     _IOW(TUNER_IOC_MAGIC, 20, enum _tuner_cpathid)
#define TUNER_IOCTL_NRST_CTL	_IOW(TUNER_IOC_MAGIC, 30, unsigned int)

#define TUNER_IOCTL_TSIF_BLKSIZE        _IOR(TUNER_IOC_MAGIC, 64, int)
#define TUNER_IOCTL_TSIF_GET_IV         _IOR(TUNER_IOC_MAGIC, 65, uint8_t)
#define TUNER_IOCTL_TSIF_INIT_IV        _IOW(TUNER_IOC_MAGIC, 66, uint8_t)

/******************************************************************************
 * enumerator type
 ******************************************************************************/
/** @addtogroup group_libtuner_API_public */

/** @brief Register bank enumerator */
enum _reg_bank {
	Sub = 0, /*!< Register bank in RF circuit */
	Main1 = 1, /*!< Register bank in the 13(full)-segment demodulator */
	Main2 = 2, /*!< Register bank in the 1-segment dedicated demodulator */
#ifdef __KERNEL__
	SSub = 3,
	SMain1 = 4,
	SMain2 = 5,
	END_SLVCFG = 255      /* End Mark */
#endif
};

/** @brief Event (interrupt) setting mode */
enum _evset_mode {
	/** Add the specified event definition to existence definitions */
	TUNER_EVENT_MODE_ADD = 0,
	/** Clear existence definitions and set specified definitions only */
	TUNER_EVENT_MODE_OVW = 1,
};

/** @brief OFDM demodulator circuit enumerators. */
enum _bw_seg {
	TUNER_DRV_BW13	= 0,	/*!< 13(full)-seg. reception. */
	TUNER_DRV_BW1		= 1,	/*!< 1-segment reception. */
	TUNER_DRV_BW3		= 2,	/*!< 3-segment reception. */
};

/** @brief TS packet type enumerator */
enum _ts_pkt_type {
	TUNER_DRV_TS_NORMAL = 0, /*!< 188 bytes length */
	TUNER_DRV_TS_ADDFEC = 1, /*!< 204 bytes length (with redundancy bits) */
	TUNER_DRV_TS_TSTAMP = 2, /*!< 192 bytes length (with Time-Stamp) */
};
/** @} */
enum _tuner_cpathid {
	DVR_MASTER = 0,
	DVR_SLAVE = 3,
};

/******************************************************************************
 * struct/union type
 ******************************************************************************/
/* for register read/write */
/* @brief Structure to write some (masked) bits of a register. */
struct snglreg {
	enum _reg_bank bank;	/*!< Register bank. */
	uint8_t adr;	/*!< Offset address in a bank. */
	uint8_t enabit;	/*!< bit-mask specifies some bits of a register. */
	uint8_t param;	/*!< read/write data. */
};

union _tuner_data_rw {
	struct snglreg sngl;
	struct {
		enum _reg_bank bank;	/* reg. bank */
		uint8_t adr;	/* reg. address */
		uint16_t len;	/* continuous length */
		uint8_t *buf;	/* buffer for continuous read/write */
	} cont;
};

/** @addtogroup group_libtuner_API_public */

/** @brief Union to set/get the event (interrupt) definitions/factors */
union _tuner_data_event {
	struct {
		unsigned intst:		8;	/*!< INTST register */
		unsigned intcnd:	8;	/*!< INTCND register */
		unsigned intset1:	8;	/*!< INTSET1 register */
		unsigned irqnum:	8;	/*!< Num of the IRQ */
	} get;		/*!< To access as the event (interrupt) factor */
	struct {
		unsigned intdef2:	4;	/*!< INTDEF2[3:0] register */
		unsigned reserve:	4;	/*!< reserve bits */
		unsigned intdef1:	8;	/*!< INTDEF1 register */
		unsigned intset1:	8;	/*!< INTSET1 register */
		unsigned mode:		4;	/*!< ::_evset_mode */
		unsigned reserve1:	4;	/*!< reserve bits */
	} set;		/*!< To access as the event (interrupt) definition */
	/** @brief For the Event Enumerator (::_tuner_event) */
	uint32_t pack;
};

/** @brief Event (interrupt) definition/factor type */

/** @brief TS slave I/F control parameters
 *
 * Control TS slave I/F sub-system of the tuner device.
 */
struct _tuner_data_tsif {
	uint8_t dwind[3];	/*!< Data window width */
	uint8_t thl[3];	    /*!< Water line lower threshold */

	enum _ts_pkt_type ts_pkt_type;	/*!< TS packet type */

	/* AES configuration */
	uint8_t aes_mode;

	/* depend on the SPI (HOST-Master) I/F */
	uint8_t spi_ts_bit_per_word;	/*!< Bits per a word of SPI I/F */
	/*1: calibration execute when tuner_drv_hw_tsif_get_pkts called. */
	/*0: calibration off. */
	uint8_t spi_calibration_flag;
};
/** @brief TS slave I/F control type */

/** @} */

#endif/* _TUNER_DRV_SYS_H */
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
