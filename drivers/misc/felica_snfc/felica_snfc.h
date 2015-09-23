/* drivers/misc/felica_snfc/felica_snfc.h
 *
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Author: Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __FELICA_SNFC_H__
#define __FELICA_SNFC_H__

#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/switch.h>
#include <linux/wait.h>

struct felica_cen_ops {
	int	(*cen_init)(void *);
	int	(*cen_read)(u8 *, void *);
	int	(*cen_write)(u8, void *);
	int	(*cen_release)(void *);
};

struct felica_pon_ops {
	int	(*pon_init)(void *);
	void	(*pon_write)(int, void *);
	void	(*pon_release)(void *);
};

struct felica_rfs_ops {
	int	(*rfs_init)(void *);
	int	(*rfs_read)(void *);
	void	(*rfs_release)(void *);
};

struct felica_int_ops {
	int	(*int_init)(void *);
	int	(*int_read)(void *);
	void	(*int_release)(void *);
};

struct nfc_intu_ops {
	int	(*intu_init)(void *);
	int	(*intu_read)(void *);
	void	(*intu_release)(void *);
};

struct nfc_hsel_ops {
	int	(*hsel_init)(void *);
	void	(*hsel_write)(int, void *);
	void	(*hsel_release)(void *);
};

struct nfc_ldo_ops {
	void	(*ldo_write)(int, void *);
};

enum dev_type {
	FELICA,
	FELICA_SNFC,
};

struct felica_data {
	enum dev_type type;
	void *user;

	/* FeliCa */
	const struct felica_cen_ops *flcen;
	const struct felica_pon_ops *flpon;
	const struct felica_rfs_ops *flrfs;
	const struct felica_int_ops *flint;
	unsigned int irq_rfs;
	unsigned int irq_int;

	/* NFC */
	const struct nfc_intu_ops *flintu;
	const struct nfc_hsel_ops *flhsel;
	const struct nfc_ldo_ops *flldo;
	unsigned int irq_intu;
};

int felica_snfc_irq_start(struct device *dev);
int felica_snfc_irq_shutdown(struct device *dev);
int felica_snfc_unregister(struct device *dev);
int felica_snfc_register(struct device *dev, struct felica_data *felica_data);

#endif
