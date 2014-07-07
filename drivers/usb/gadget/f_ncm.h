/* drivers/usb/gadget/f_ncm.h
 *
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * The file borrows code from f_ncm.c and f_mbim.c which is:
 *
 * Copyright (C) 2010 Nokia Corporation
 * Contact: Yauheni Kaliuta <yauheni.kaliuta@nokia.com>
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2012 Sony Mobile Communications AB.
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __F_NCM_H
#define __F_NCM_H

#include <linux/usb/cdc.h>

enum ncm_notify_state {
	NCM_NOTIFY_NONE,		/* don't notify */
	NCM_NOTIFY_CONNECT,		/* issue CONNECT next */
	NCM_NOTIFY_SPEED,		/* issue SPEED_CHANGE next */
};

#define LOG2_STATUS_INTERVAL_MSEC	5	/* 1 << 5 == 32 msec */
#define NCM_STATUS_BYTECOUNT		16	/* 8 byte header + data */

/*
 * Here are options for NCM Datagram Pointer table (NDP) parser.
 * There are 2 different formats: NDP16 and NDP32 in the spec (ch. 3),
 * in NDP16 offsets and sizes fields are 1 16bit word wide,
 * in NDP32 -- 2 16bit words wide. Also signatures are different.
 * To make the parser code the same, put the differences in the structure,
 * and switch pointers to the structures when the format is changed.
 */

struct ndp_parser_opts {
	u32		nth_sign;
	u32		ndp_sign;
	unsigned	nth_size;
	unsigned	ndp_size;
	unsigned	ndplen_align;
	/* sizes in u16 units */
	unsigned	dgram_item_len; /* index or length */
	unsigned	block_length;
	unsigned	fp_index;
	unsigned	reserved1;
	unsigned	reserved2;
	unsigned	next_fp_index;
};

#define INIT_NDP16_OPTS {					\
		.nth_sign = USB_CDC_NCM_NTH16_SIGN,		\
		.ndp_sign = USB_CDC_NCM_NDP16_NOCRC_SIGN,	\
		.nth_size = sizeof(struct usb_cdc_ncm_nth16),	\
		.ndp_size = sizeof(struct usb_cdc_ncm_ndp16),	\
		.ndplen_align = 4,				\
		.dgram_item_len = 1,				\
		.block_length = 1,				\
		.fp_index = 1,					\
		.reserved1 = 0,					\
		.reserved2 = 0,					\
		.next_fp_index = 1,				\
	}


#define INIT_NDP32_OPTS {					\
		.nth_sign = USB_CDC_NCM_NTH32_SIGN,		\
		.ndp_sign = USB_CDC_NCM_NDP32_NOCRC_SIGN,	\
		.nth_size = sizeof(struct usb_cdc_ncm_nth32),	\
		.ndp_size = sizeof(struct usb_cdc_ncm_ndp32),	\
		.ndplen_align = 8,				\
		.dgram_item_len = 2,				\
		.block_length = 2,				\
		.fp_index = 2,					\
		.reserved1 = 1,					\
		.reserved2 = 2,					\
		.next_fp_index = 2,				\
	}

#endif /* __F_NCM_H */
