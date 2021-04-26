/**************************************************************************//**
 *
 *  @file		tuner_drv_hw.h
 *
 *  @brief		Common header file of the HardWare control layer.
 *
 ***************************************************************************//*
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
#ifndef _TUNER_DRV_HW_H
#define _TUNER_DRV_HW_H

/******************************************************************************
 * include
 ******************************************************************************/
#include "tuner_drv.h"

/******************************************************************************
 * prototype
 ******************************************************************************/

/* following functions are described in "tuner_drv_hw.c" */
int tuner_drv_hw_reqirq(void);
void tuner_drv_hw_freeirq(void);

#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER

void tuner_drv_hw_enable_interrupt(void);
void tuner_drv_hw_disable_interrupt(void);

#endif

/*
 * following functions are described in "tuner_drv_<if>.c"
 */
int tuner_drv_hw_read_reg(
		enum _reg_bank bank,
		uint8_t adr,
		uint16_t len,
		uint8_t *rd
		);
int tuner_drv_hw_write_reg(
		enum _reg_bank bank,
		uint8_t adr,
		uint16_t len,
		uint8_t *wd
		);
int tuner_drv_hw_rmw_reg(
		enum _reg_bank bank,
		uint8_t adr,
		uint8_t mask,
		uint8_t wd
		);
int tuner_drv_hw_setev(union _tuner_data_event *ev);
int tuner_drv_hw_relev(union _tuner_data_event *ev);

#ifndef CPATH_I2C
int tuner_drv_hw_write_prg(
		enum _reg_bank bank,
		uint8_t adr,
		uint16_t len,
		uint8_t *wd
		);
#endif
#ifdef CPATH_I2C
int tuner_drv_hw_set_id(enum _tuner_cpathid cpath_id);
#endif
extern enum _tuner_cpathid cpath_id;

/*
 * following functions are described in "tuner_drv_hw_<if>.c".
 * <if> is NOT "i2c".
 */
#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
int tuner_drv_hw_tsif_set_tpm(void);
int tuner_drv_hw_tsif_register(void);
void tuner_drv_hw_tsif_unregister(void);

int tuner_drv_hw_tsif_set_cntxt(struct _tsif_cntxt *tc);
int tuner_drv_hw_tsif_config(struct _tsif_cntxt *tc);
int tuner_drv_hw_tsif_get_pkts(struct _tsif_cntxt *tc);
int tuner_drv_hw_tsif_get_dready(void);
int tuner_drv_hw_tsif_sync_pkt(void);

/*
*   Common Setting for slave IF
*   These setting are only valid when DPATH is enabled.
*/

extern struct snglreg slvif_cfgregs[];
/* register offset is below */
#define SLVIF_CFG_SLVINTEN   (0)
#define SLVIF_CFG_ISEGSEL    (1)
#define SLVIF_CFG_DOSET4     (2)
#define SLVIF_CFG_WATERLINE  (3)
#define SLVIF_CFG_BYTEORDER  (4)
#define SLVIF_CFG_PKTSYNCC3  (5)
#define SLVIF_CFG_IFPWDSET1  (6)

#endif

extern int TUNER_CONFIG_INT;

#endif /* _TUNER_DRV_HW_H */
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
