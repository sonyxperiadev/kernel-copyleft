/**************************************************************************//**
 *
 *  @file		tuner_drv_hw.c
 *
 *  @brief		The HW Wrapping Layer for Tmm Tuner Driver
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
/******************************************************************************
 * include
 ******************************************************************************/
#include "tuner_drv.h"
#include "tuner_drv_hw.h"

#ifdef TUNER_CONFIG_IRQ_ENABLE
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#endif

/******************************************************************************
 * data
 ******************************************************************************/
static bool g_tuner_irq_flag;

#if defined(DPATH_SPI) || defined(DPATH_SDIO) || defined(DPATH_GPIF)
	/* Configuration registers list for slave i/f. */
	/* Don't Edit from here */
struct snglreg slvif_cfgregs[] = {
	{ Main2, 0x6B, 0x20, 0x20 },	/* #0  EXINTSET.SLVINTEN = 1 */
	{ Main1, 0xE2, 0xF0, 0x90 },	/* #1  INTSET5.ISEGSEL[3:0] = 9 */
	/* #2  STM_SYNCSEL[4:0] DOSET4[4:0] = 1 */
	{ Main1, 0xD9, 0x1F, 0x01 },
	/* #3  PKTWLSET1.DATA_WINDOW0[4:0], PKTWLSET2.LOWER_LEVEL0[2:0] */
	{ Main2, 0x66, 0xFF, 0x00 },
	/* #4  PKTBUFCTL.ECONFIG[1:0] Set byte order of slave i/f */
	{ Main2, 0x60, 0x40, 0x00 },
	/* #5  PKTSYNCC3[7:4] COFF2=ON NULLOFF2=ON */
	{ Main2, 0x65, 0x90, 0x90 },
	/* #6  IFPWDSET1[6].[2:1] PKTBUF0,INTGEN0,PKTPACK0 ON*/
	{ Main2, 0x70, 0x46, 0x00 },
#ifdef TUNER_CONFIG_PF_NULLFILTER
	{ Main2, 0x70, 0x01, 0x00 },	/* #7  IFPWDSET1[0] PF0 CLK ON */
	/* #8  PFTBLSET0A ADDR=0x41(PF0CONFIG) Set address */
	{ Main2, 0x59, 0xFF, 0x41 },
	/* #9  PFTBLSET0L DATA_LOW=0x02 Set Null Filter Enable */
	{ Main2, 0x57, 0x02, 0x02 },
	/* #10 PFTBLSET0U DATA_HI=0x00 */
	{ Main2, 0x58, 0xFF, 0x00 },
	/* #11 PFTBLSET0A ADDR=0x41(PF0CONFIG) Write Data */
	{ Main2, 0x59, 0xFF, 0xC1 },
#endif
	/* Don't Edit to here */

	/* If you want to add any setting the below */
	/* { Mainx , 0xxx, 0xxx, 0x00 }, */

	/* End Mark. Do not remove this line */
	{ END_SLVCFG, 0x00, 0x00, 0x00 }

};
#endif


/******************************************************************************
 * code area
 ******************************************************************************/

/**************************************************************************//**
 * interruption registration control of a driver
 *
 * @retval	0	normal
 * @retval		<0	error
 ******************************************************************************/
int tuner_drv_hw_reqirq(void)
{
	int ret = 0; /* function return */

	/* the sub-system of IRQ has been already activated */
	if (g_tuner_irq_flag == true) {
		pr_debug("IRQ (#%d) is already active, so do nothing\n",
				TUNER_CONFIG_INT);
		return 0;
	}

#ifdef TUNER_CONFIG_IRQ_ENABLE
	pr_debug("*** request IRQ ***\n");
	/* request IRQ */
	ret = request_irq(
			TUNER_CONFIG_INT,		/* number of IRQ */
			tuner_interrupt,		/* call-back function */
			IRQF_TRIGGER_RISING,
			"mm_tuner",			/* IRQ name */
			NULL			/* device ID is not specified */
			);
	if (ret != 0) {
		pr_err("request_irq() fail (return:%d)\n", ret);
		return -EINVAL;
	}
#else
#ifdef DEBUG
	pr_debug("TUNER_CONFIG_IRQ_ENABLE is not defined\n");
#endif
#endif
	/* IRQ status flag: on */
	g_tuner_irq_flag = true;

	return ret;
}

/**************************************************************************//**
 * interruption registration release control of a driver
 *
 ******************************************************************************/
void tuner_drv_hw_freeirq(void)
{
	if (g_tuner_irq_flag == false) {
		/* IRQ line is not active */
		pr_debug("IRQ (#%d) is not active, so do nothing.\n",
				TUNER_CONFIG_INT);
		return;
	}

#ifdef TUNER_CONFIG_IRQ_ENABLE
	/* disable IRQ line */
	pr_debug("*** FREE IRQ ***\n");
	free_irq(TUNER_CONFIG_INT, NULL);
#else
#ifdef DEBUG
	pr_debug("TUNER_CONFIG_IRQ_ENABLE is not defined.\n");
#endif
#endif

	/* flag off */
	g_tuner_irq_flag = false;
}

/**************************************************************************//**
 * Write masked bits of the Register. (Read and Modified Write)
 *
 * @retval 0					Normal end
 * @retval <0					error (refer the errno)
 *
 * @param [in] bank	register bank enumerator
 * @param [in] adr	start address for continuous write
 * @param [in] mask	continuous write length
 * @param [in] wd		write data
 ******************************************************************************/
int tuner_drv_hw_rmw_reg(enum _reg_bank bank, uint8_t adr,
						uint8_t mask, uint8_t wd)
{
	int ret;
	uint8_t data;

	if (mask == 0x00) {
		pr_warn("%s(): Bitmask is 0x00, so write nothing.\n", __func__);
		goto _out;
	}
	if (mask == 0xff) {
		data = wd;
	} else {
		ret = tuner_drv_hw_read_reg(bank, adr, 1, &data);
		if (ret)
			return ret;
		data = (data & ~mask) | wd;
		pr_debug("%s(): R,M(m:0x%02x,d:0x%02x),W(0x%02x).\n", __func__,
				mask, wd, data);
	}
	ret = tuner_drv_hw_write_reg(bank, adr, 1, &data);
	if (ret)
		return ret;

_out:
	return 0;
}

/**************************************************************************//**
 * @brief Set the event (interrupt) condition.
 *
 * This function set some specified interrupt (event) conditions,
 * and, be enable the interrupt sub system.
 *
 * @retval 0			normal
 * @retval <0			error
 ******************************************************************************/
int tuner_drv_hw_setev(union _tuner_data_event *ev)
{
	int ret;
	uint8_t buf[2] = { 0x00, 0x00 };

	pr_debug("mode:%u intset1:0x%02x intdef1:0x%02x intdef2:0x%01x",
	ev->set.mode, ev->set.intset1, ev->set.intdef1, ev->set.intdef2);

	if (ev->set.mode == TUNER_EVENT_MODE_ADD) {
		/* read INTDEF1 and INTDEF2 */
		ret = tuner_drv_hw_read_reg(Main1,	0xDC, 2, buf);
		if (ret) {
			pr_err("Read INTDEF1/2, failed\n");
			return ret;
		}
		buf[0] |= ev->set.intdef1;
		buf[1] |= ev->set.intdef2;
	} else {	/* Overwrite mode: TUNER_EVENT_MODE_OVW */
		buf[0] = ev->set.intdef1;
		buf[1] = ev->set.intdef2;
	}

	/* write INTDEF1 and INTDEF2 */
	ret = tuner_drv_hw_write_reg(Main1, 0xDC, 2, buf);
	if (ret) {
		pr_err("Write INTDEF1/2, fail.\n");
		return ret;
	}

	/* write INTSET1[3](NINTEN) and NITSET1[0](INTMD) */
	ret = tuner_drv_hw_rmw_reg(Main1, 0xDE, 0x09, ev->set.intset1);
	if (ret) {
		pr_err("Write INTSET1.NINTEN/INTMD, failed\n");
		return ret;
	}
	if ((buf[0] | (buf[1] & 0x0F)) != 0x00) {
		pr_debug("Enable system IRQ line.\n");
		ret = tuner_drv_hw_reqirq();
		if (ret) {
			pr_err("tuner_drv_hw_reqirq() failed.\n");
			return ret;
		}
	}

	return 0; /* normal exit */
}

/**************************************************************************//**
 * Clear the IRQ (interrupt) conditions.
 *
 * This function clear the specified interrupt conditions.
 * And, be disabled the IRQ sub-system, when the all interrupt conditions
 * are not active.
 *
 * @retval 0			normal
 * @retval <0			error
 ******************************************************************************/
int tuner_drv_hw_relev(union _tuner_data_event *ev)
{
	int ret;
	uint8_t buf[2] = { 0x00, 0x00 };

	/* read INTDEF1/2 */
	ret = tuner_drv_hw_read_reg(Main1, 0xDC, 2, buf);
	if (ret) {
		pr_err("Read INTDEF1/2, failed.\n");
		return ret;
	}

	/* clear specified bits */
	buf[0] &= ~(ev->set.intdef1);
	buf[1] &= ~(ev->set.intdef2);

	/* write INTDEF1/2 */
	ret = tuner_drv_hw_write_reg(Main1, 0xDC, 2, buf);
	if (ret) {
		pr_debug("Write INTDEF1/2, failed.\n");
		return ret;
	}

	if ((buf[0] | (buf[1] & 0x0F)) == 0x00) {
		pr_debug("Disable system IRQ line.\n");
		tuner_drv_hw_freeirq();
	}

	return 0;	/* normal return */
}

#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
/**************************************************************************//**
 * interruption registration enable control of a driver
 *
 ******************************************************************************/
void tuner_drv_hw_enable_interrupt(void)
{
	/* enabling interrupt */
	enable_irq(TUNER_CONFIG_INT, NULL);
}

/**************************************************************************//**
 * interruption registration disable control of a driver
 *
 ******************************************************************************/
void tuner_drv_hw_disable_interrupt(void)
{
	/* disabling interrupt */
	disable_irq(TUNER_CONFIG_INT, NULL);
}
#endif /* TUNER_CONFIG_IRQ_LEVELTRIGGER */
/*******************************************************************************
 * Copyright (c) 2015 Socionext Inc.
 ******************************************************************************/
