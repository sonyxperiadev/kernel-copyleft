/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CORESIGHT_CORESIGHT_TPDA_H
#define _CORESIGHT_CORESIGHT_TPDA_H

#define TPDA_CR			(0x000)
#define TPDA_Pn_CR(n)		(0x004 + (n * 4))
#define TPDA_FPID_CR		(0x084)
#define TPDA_FREQREQ_VAL	(0x088)
#define TPDA_SYNCR		(0x08C)
#define TPDA_FLUSH_CR		(0x090)
#define TPDA_FLUSH_SR		(0x094)

/* Cross trigger FREQ packets timestamp bit */
#define TPDA_CR_FREQTS		BIT(2)
/* Cross trigger FREQ packet request bit */
#define TPDA_CR_FRIE		BIT(3)
/* Cross trigger FLAG packet request interface bit */
#define TPDA_CR_FLRIE		BIT(4)
/* Cross trigger synchronization bit */
#define TPDA_CR_SRIE		BIT(5)
/* Packetize CMB/MCMB traffic bit */
#define TPDA_CR_CMBCHANMODE	BIT(20)

/* Aggregator port enable bit */
#define TPDA_Pn_CR_ENA		BIT(0)
/* Aggregator port CMB data set element size bit */
#define TPDA_Pn_CR_CMBSIZE		GENMASK(7, 6)
/* Aggregator port DSB data set element size bit */
#define TPDA_Pn_CR_DSBSIZE		BIT(8)
/* Mode control bit */
#define TPDA_MODE_CTRL			BIT(12)

#define TPDA_MAX_INPORTS	32

/* Bits 6 ~ 12 is for atid value */
#define TPDA_CR_ATID		GENMASK(12, 6)
/* Bits 13 ~ 19 is for mid value */
#define TPDA_CR_MID		GENMASK(19, 13)

#define IS_STATIC_TPDM		1

/**
 * struct tpda_drvdata - specifics associated to an TPDA component
 * @base:       memory mapped base address for this component.
 * @dev:        The device entity associated to this component.
 * @csdev:      component vitals needed by the framework.
 * @spinlock:   lock for the drvdata value.
 * @enable:     enable status of the component.
 * @dsb_esize   Record the DSB element size.
 * @cmb_esize   Record the CMB element size.
 */
struct tpda_drvdata {
	void __iomem		*base;
	struct device		*dev;
	struct coresight_device	*csdev;
	spinlock_t		spinlock;
	u8			atid;
	u32			dsb_esize;
	u32			cmb_esize;
	bool			trig_async;
	bool			trig_flag_ts;
	bool			trig_freq;
	bool			freq_ts;
	uint32_t		freq_req_val;
	bool			freq_req;
	bool			cmbchan_mode;
	struct clk		*atclk;
};

#endif  /* _CORESIGHT_CORESIGHT_TPDA_H */
