// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#if !defined(_IPA_ACCESS_CONTROL_H_)
#define _IPA_ACCESS_CONTROL_H_

#include "ipa_reg_dump.h"

/*
 * AA_COMBO - actual read, actual write
 * AN_COMBO - actual read, no-op write
 * NA_COMBO - no-op read, actual write
 * NN_COMBO - no-op read, no-op write
 */

/*
 * The following is target specific.
 */
static struct reg_mem_access_map_t mem_access_map[] = {
	/*------------------------------------------------------------*/
	/*      Range               Use when              Use when    */
	/*  Begin    End           SD_ENABLED           SD_DISABLED   */
	/*------------------------------------------------------------*/
	{ 0x04000, 0x04FFF, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0xA8000, 0xB8000, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0x05000, 0x0F000, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0x0F000, 0x10000, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0x18000, 0x2A000, { &io_matrix[AA_COMBO], &io_matrix[AA_COMBO] } },
	{ 0x2A000, 0x3C000, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0x3C000, 0x4E000, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0x10000, 0x11000, { &io_matrix[AA_COMBO], &io_matrix[AA_COMBO] } },
	{ 0x11000, 0x12000, { &io_matrix[NN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x12000, 0x13000, { &io_matrix[NN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x14C000, 0x14D000, { &io_matrix[AA_COMBO], &io_matrix[AA_COMBO] } },
	{ 0x14D000, 0x14E000, { &io_matrix[NN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x14E000, 0x150000, { &io_matrix[NN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x140000, 0x148000, { &io_matrix[AA_COMBO], &io_matrix[AA_COMBO] } },
	{ 0x148000, 0x14C000, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0x150000, 0x160000, { &io_matrix[AA_COMBO], &io_matrix[AA_COMBO] } },
	{ 0x160000, 0x180000, { &io_matrix[AN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x180000, 0x181000, { &io_matrix[NN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x181000, 0x1A0000, { &io_matrix[AN_COMBO], &io_matrix[AN_COMBO] } },
	{ 0x1A0000, 0x1C0000, { &io_matrix[AN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x1C0000, 0x1C2000, { &io_matrix[NN_COMBO], &io_matrix[NN_COMBO] } },
	{ 0x1C2000, 0x1C4000, { &io_matrix[AA_COMBO], &io_matrix[AA_COMBO] } },
	{ 0x120000, 0x128000, { &io_matrix[NN_COMBO], &io_matrix[NN_COMBO] } },
};

#endif /* #if !defined(_IPA_ACCESS_CONTROL_H_) */
