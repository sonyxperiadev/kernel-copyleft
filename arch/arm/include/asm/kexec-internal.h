/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#ifndef _ARM_KEXEC_INTERNAL_H
#define _ARM_KEXEC_INTERNAL_H

struct kexec_relocate_data {
	unsigned long kexec_start_address;
	unsigned long kexec_indirection_page;
	unsigned long kexec_mach_type;
	unsigned long kexec_r2;
};

#endif
