/* linux/drivers/misc/isdb-t/tuner_api.h
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Author: Tatsuya Ooka <Tatsuya.Ooka@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef _TUNER_API_H
#define _TUNER_API_H

#include "tuner_config.h"

enum TUNER_STATE {
	TUNER_STATE_FULLSEG,
	TUNER_STATE_FULL_ONE_SEG,
	TUNER_STATE_ONESEG,
};

/* Simplified BER/PER monitor */
struct _tune_data_easy_bper {
	unsigned char set_rank;
	unsigned int  info_data;
	unsigned char set_read;
	unsigned char set_mode;
	unsigned char set_time;
	unsigned char set_data[3];
	unsigned char set_set;
	unsigned char set_lay;
	unsigned int  info_berrd;
	unsigned int  info_berlenrd;
	unsigned char info_berd_st;
	unsigned char info_read1;
	unsigned char info_stm[3];
	unsigned char info_read2[3];
	unsigned char set_len;
	unsigned char set_sel;
	unsigned char info_i2c;
	unsigned char info_berrdy;
};

/* RSSI monitor               */
struct _tune_data_rssi {
	int info_rssi;
};

#endif/* _TUNER_API_H */
