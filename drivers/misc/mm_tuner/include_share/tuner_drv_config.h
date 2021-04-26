/**************************************************************************//**
 *
 *  @file		tuner_drv_config.h
 *
 *  @brief		configuration header of the mm_tuner55x driver
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
#ifndef _TUNER_DRV_CONFIG_H
#define _TUNER_DRV_CONFIG_H

/******************************************************************************
 * define
 ******************************************************************************/
#define TUNER_SET_ON                     1       /* setting ON */
#define TUNER_SET_OFF                    0       /* setting OFF */

/* device driver file name */
#define TUNER_CONFIG_DRIVER_NAME		"mmtuner"

/* device number */
#define TUNER_CONFIG_DRV_MAJOR         100       /* MAJOR No. */
#define TUNER_CONFIG_DRV_MINOR         200       /* MINOR No. */

/* compile switch for IRQ */
/* #define TUNER_CONFIG_IRQ_PC_LINUX */

/* IRQ */
/* #define TUNER_CONFIG_IRQ_ENABLE */	/* System IRQ Enable */
extern int TUNER_CONFIG_INT;

/**
 * IRQ kernel thread priority (0-99)
 */
#define TUNER_CONFIG_KTH_PRI			95

/* TS I/F kernel thread priority (0-99)  */
#define TUNER_CONFIG_TSBTH_PRI			95

/* exclusive access control  */
/* #define TUNER_CONFIG_DRV_MULTI */

/**
 * @brief Compile switch for Device Tree.
 *
 * This definition should be enabled
 * if the kernel does NOT support Device Tree.
 */
/* #define TUNER_CONFIG_NO_DEVICE_TREE */

/**
 * @brief Compile switch for the SPI EDGE mode.
 *
 * Use SPI edge mode when using SPI slave I/F.
 * Please define this macro TUNER_CONFIG_SPI_EDGE
 * when you use 30MHz over SPI CLK.
 */
/* #define TUNER_CONFIG_SPI_EDGE */

/* @brief SPI calibration Timeout [ms]. */
#define TUNER_CONFIG_CALIBRATION_TIMEOUT		(1800000UL)

/**
 * @brief Compile switch for the SPI BREAK code at every SPI command.
 *
 * Send SPI BREAKCODE.
 * Insert SPI break pattern before every SPI command when defined.
 */
/* #define TUNER_CONFIG_SPI_BREAKCODE */

/**
 * @brief Compile switch for the SPI divide transaction mode.
 *
 * Divide SPI Packet Read command into 2 transactions
 * (command phase and data phase).
 */
/* #define TUNER_CONFIG_SPI_DIV_MSG */

/**
 * @brief Compile switch for the extend packet read mode.
 *
 * Use Extend Packet read with SPI slave I/F.
 * We DON'T need to use this option usually.
 */
/* #define TUNER_CONFIG_SPI_EXTREAD */

/* @brief TS-Read timeout limit [ms]. */
#define TUNER_CONFIG_TSREAD_TIMEOUT			(1200UL)


/* Using DMA transfer when using GPIF I/F */
/* #define TUNER_CONFIG_GPIF_DMA */

/* Enable AES on DPATH */
/* #define TUNER_CONFIG_AES_ENABLE */

/* Enable TS packet when AES is on */
/* #define TUNER_CONFIG_AES_TSCHK */

/* Enable PID NULL Packet filter  */
/* #define TUNER_CONFIG_PF_NULLFILTER */

/* Configurate byte order of TS stream via slave i/f */
/* Plase set the following MACRO when you need the byte order transform */
/* +-----------+------------+------------+-----------+------------* */
/* | Slave     |  Size      | MACRO      | BYTEORDER | CPU ENDIAN | */
/* +-----------+------------+------------+-----------+------------+ */
/* | SPI Slave |  8         | Don't care | MSB First | BIG/LITTLE | */
/* | SPI Slave |  32        | Undefined  | LSB First | LITTLE     | */
/* | SPI Slave |  32        | Defined    | MSB First | BIG        | */
/* +-----------+------------+------------+-----------+------------+ */
/* | SDIO      | Don't care | Don't care | MSB First | Don't care | */
/* +-----------+------------+------------+-----------+------------+ */
/* | GPIF      | Don't care | Undefined  | LSB First | Don't care | */
/* | GPIF      | Don't care | Defined    | MSB First | Don't care | */
/* +-----------+------------+------------+-----------+------------+ */
/*       LSB First Stream ( .. .. .. 0x47 .. .. ) */
/*       MSB First Stream ( 0x47 .. .. .. ) */
/* The order of GPIF depends on memory controller */
/* #define TUNER_CONFIG_SLV_MSBFIRST */


/* DMA Boundary */
/* Some SoC has the transfer size limitation to use DMA with SPI */
/* It's limitation is that the data size must be align with 1K */
/* So, when you want to use these SoC,please set the following MACRO */
/* When SoC has 512byte boundary limitation,set to 512 */
/* When SoC has 1024byte boundary limitation,set to 1024 */
/* #define TUNER_CONFIG_SPI_ALIGN 1024 */


#endif/* _TUNER_DRV_CONFIG_H */
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
