// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */


#ifndef BTFM_SWR_SLAVE_H
#define BTFM_SWR_SLAVE_H

#include "btfm_swr.h"

/* Registers Address */

/* Register Bit Setting */
#define SLAVE_ENABLE_OVERRUN_AUTO_RECOVERY	(0x1 << 1)
#define SLAVE_ENABLE_UNDERRUN_AUTO_RECOVERY	(0x1 << 0)
#define SLAVE_SB_PGD_PORT_ENABLE			(0x1 << 0)
#define SLAVE_SB_PGD_PORT_DISABLE		(0x0 << 0)


#define BTFM_INVALID_PORT 0xFF

extern struct soc_port_mapping slave_port[];

enum {
	QCA_GANGES_SOC_ID_0100 = 0x40210100,
	QCA_GANGES_SOC_ID_0200 = 0x40210200,
};


enum {
	QCA_EVROS_SOC_ID_0100 = 0x40200100,
	QCA_EVROS_SOC_ID_0200 = 0x40200200,
};


enum {
	EVROS_EA = 0x0108170220,
	GANGES_EA = 0x0208170220,
};

/* Specific defines for slave slimbus device */
#define SLAVE_SWR_REG_OFFSET		0x0800

#endif
