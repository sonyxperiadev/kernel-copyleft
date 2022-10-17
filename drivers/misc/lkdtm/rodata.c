/*
 * NOTE: This file has been modified by Sony Corporation.
 * Modifications are Copyright 2021 Sony Corporation,
 * and licensed under the license of the file.
 */
// SPDX-License-Identifier: GPL-2.0
/*
 * This includes functions that are meant to live entirely in .rodata
 * (via objcopy tricks), to validate the non-executability of .rodata.
 */
#include "lkdtm.h"

void noinstr lkdtm_rodata_do_nothing(void)
{
	/* Does nothing. We just want an architecture agnostic "return". */
}
