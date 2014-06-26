/* drivers/input/misc/apds9930.h
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *         Takashi Shiina <Takashi.Shiina@sonymobile.com>
 *         Masashi Shimizu <Masashi.X.Shimizu@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */


#ifndef __APDS9930_H
#define __APDS9930_H

struct device;

enum apds9930_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

enum taos_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_8        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_120      = (3 << 0),
	PGAIN_1        = (0 << 2),
	PGAIN_2        = (1 << 2),
	PGAIN_4        = (2 << 2),
	PGAIN_8        = (3 << 2),
	PDIOD_NO       = (0 << 4),
	PDIOD_CH0      = (1 << 4),
	PDIOD_CH1      = (2 << 4),
	PDIOD_DONT_USE = (3 << 4),
	PDRIVE_100MA   = (0 << 6),
	PDRIVE_50MA    = (1 << 6),
	PDRIVE_25MA    = (2 << 6),
	PDRIVE_12_5MA    = (3 << 6),
};

#define PRX_PERSIST(p) (((p) & 0xf) << 4)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

struct apds9930_parameters {
	u16 prox_th_min;
	u16 prox_th_max;
};

#endif /* __APDS9930_H */
