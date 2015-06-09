/* include/linux/felica.h
 *
 * Copyright (C) 2010-2011 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SONY_FELICA_H
#define _SONY_FELICA_H

#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/switch.h>
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
#include <linux/wait.h>
#endif

#define FELICA_DEV_NAME "semc_felica"

/* FeliCa Device structures */
struct felica_dev {
	struct list_head	node;
	struct device	*dev;
	struct miscdevice	device_cen;
	struct miscdevice	device_pon;
	struct miscdevice	device_rfs;
	struct miscdevice	device_rws;
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	struct miscdevice	device_hsel;
	struct miscdevice	device_intu_poll;
	struct miscdevice	device_available_poll;
	struct miscdevice	device_snfc_cen;
#endif
	struct felica_cen_pfdata	*flcen;
	struct felica_pon_pfdata	*flpon;
	struct felica_rfs_pfdata	*flrfs;
	struct felica_int_pfdata	*flint;
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	struct snfc_intu_pfdata		*flintu;
	struct snfc_hsel_pfdata		*flhsel;
	struct snfc_ldo_pfdata		*flldo;
#endif
	struct switch_dev	swdev;
	unsigned int		st_usbcon;
	unsigned int		st_airplane;
	unsigned int		ta_rwusb;
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	unsigned int		available_poll_snfc;
	wait_queue_head_t	available_poll_wait;
	int			intu_curr_val;
	wait_queue_head_t	intu_wait_queue;
	int			hsel_ref;
#endif
};

/* FeliCa Platform Data structures */
struct felica_cen_pfdata {
	int	(*cen_init)(struct felica_dev *);
	int	(*cen_read)(u8 *, struct felica_dev *);
	int	(*cen_write)(u8, struct felica_dev *);
};

struct felica_pon_pfdata {
	int	(*pon_init)(struct felica_dev *);
	void	(*pon_write)(int, struct felica_dev *);
	void	(*pon_release)(struct felica_dev *);
	int	(*tvdd_on)(struct felica_dev *);
	void	(*tvdd_off)(struct felica_dev *);
};

struct felica_rfs_pfdata {
	int	(*rfs_init)(struct felica_dev *);
	int	(*rfs_read)(struct felica_dev *);
	void	(*rfs_release)(struct felica_dev *);
	unsigned int irq_rfs;
};

struct felica_int_pfdata {
	int	(*int_init)(struct felica_dev *);
	int	(*int_read)(struct felica_dev *);
	void	(*int_release)(struct felica_dev *);
	unsigned int irq_int;
};

#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
struct snfc_intu_pfdata {
	int	(*intu_init)(struct felica_dev *);
	int	(*intu_read)(struct felica_dev *);
	void	(*intu_release)(struct felica_dev *);
	unsigned int irq_intu;
};

struct snfc_hsel_pfdata {
	int	(*hsel_init)(struct felica_dev *);
	void	(*hsel_write)(int, struct felica_dev *);
	void	(*hsel_release)(struct felica_dev *);
};

struct snfc_ldo_pfdata {
	void	(*ldo_write)(int, struct felica_dev *);
};
#endif

struct felica_platform_data {
	struct felica_cen_pfdata	cen_pfdata;
	struct felica_pon_pfdata	pon_pfdata;
	struct felica_rfs_pfdata	rfs_pfdata;
	struct felica_int_pfdata	int_pfdata;
#ifdef CONFIG_SONY_FELICA_NFC_SUPPORT
	struct snfc_intu_pfdata	intu_pfdata;
	struct snfc_hsel_pfdata	hsel_pfdata;
	struct snfc_ldo_pfdata	ldo_pfdata;
#endif
	int (*gpio_init)(struct felica_dev *);
	int (*reg_release)(struct felica_dev *);
};

#endif
