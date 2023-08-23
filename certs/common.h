/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef _CERT_COMMON_H
#define _CERT_COMMON_H

int load_certificate_list(const u8 cert_list[], const unsigned long list_size,
			  const struct key *keyring);

#endif
