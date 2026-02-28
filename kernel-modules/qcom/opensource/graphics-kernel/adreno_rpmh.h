// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _ADRENO_GMU_RPMH_H
#define _ADRENO_GMU_RPMH_H

struct rpmh_arc_vals {
	u32 num;
	const u16 *val;
};

struct bcm {
	const char *name;
	u32 buswidth;
	u32 channels;
	u32 unit;
	u16 width;
	u8 vcd;
	bool fixed;
};

struct bcm_data {
	__le32 unit;
	__le16 width;
	u8 vcd;
	u8 reserved;
};

struct rpmh_bw_votes {
	u32 wait_bitmask;
	u32 num_cmds;
	u32 *addrs;
	u32 num_levels;
	u32 **cmds;
};

#define ARC_VOTE_SET(pri, sec, vlvl) \
	(FIELD_PREP(GENMASK(31, 16), vlvl) | \
	 FIELD_PREP(GENMASK(15, 8), sec) | \
	 FIELD_PREP(GENMASK(7, 0), pri))

/*
 * List of Bus Control Modules (BCMs) that need to be configured for the GPU
 * to access DDR. For each bus level we will generate a vote each BC
 */
static struct bcm adreno_ddr_bcms[] = {
	{ .name = "SH0", .buswidth = 16 },
	{ .name = "MC0", .buswidth = 4 },
	{ .name = "ACV", .fixed = true },
};

/* Same as above, but for the CNOC BCMs */
static struct bcm adreno_cnoc_bcms[] = {
	{ .name = "CN0", .buswidth = 4 },
};

/**
 * adreno_rpmh_arc_cmds - Read RPMh ARC values from command DB
 * @arc: Pointer to the rpmh_arc_vals structure
 * @res_id: Resource ID for which ARC values are requested
 *
 * Return: 0 on success or negative error on failure
 */
int adreno_rpmh_arc_cmds(struct rpmh_arc_vals *arc, const char *res_id);

/**
 * adreno_rpmh_setup_volt_dependency_tbl - Set up voltage dependency table
 * @votes: Pointer to an array to store the resulting ARC votes
 * @pri_rail: Pointer to the primary rail ARC values
 * @sec_rail: Pointer to the secondary rail ARC values
 * @vlvl: Array of voltage levels corresponding to each frequency corner
 * @num_entries: Number of entries in the voltage dependency table
 *
 * Return: 0 on success or negative error on failure
 */
int adreno_rpmh_setup_volt_dependency_tbl(u32 *votes, struct rpmh_arc_vals *pri_rail,
		struct rpmh_arc_vals *sec_rail, u16 *vlvl, u32 num_entries);

/**
 * adreno_rpmh_free_bw_votes - Free memory associated with RPMh bandwidth votes
 * @votes: Pointer to the rpmh_bw_votes structure
 */
void adreno_rpmh_free_bw_votes(struct rpmh_bw_votes *votes);

/**
 * adreno_rpmh_build_bw_votes - Build the votes table from the specified
 * bandwidth levels
 * @bcms: Array of BCM structures representing BCMs
 * @bcm_count: Number of BCMs in the array
 * @levels: Array of performance levels
 * @levels_count: Number of performance levels
 * @perfmode_vote: Performance mode vote
 * @perfmode_lvl: Initial performance level for performance mode vote
 * @gmu_ab: Indicate if GMU supports AB vote
 *
 * Return: Pointer to the rpmh_bw_votes structure on success, or an error pointer failure
 */
struct rpmh_bw_votes *adreno_rpmh_build_bw_votes(struct bcm *bcms, int bcm_count,
	u32 *levels, int levels_count, u32 perfmode_vote, u32 perfmode_lvl, bool gmu_ab);

/**
 * adreno_rpmh_to_cx_hlvl - Convert RPMh VLVL to CX HLVL level
 * @cx_rail: Pointer to the RPMh ARC values for the CX rail
 * @vlvl: Voltage level to convert
 * @hlvl: Pointer to store the resulting CX level
 *
 * Return: 0 on success or negative error on failure
 */
int adreno_rpmh_to_cx_hlvl(struct rpmh_arc_vals *cx_rail, u32 vlvl, u32 *hlvl);

/**
 * adreno_rpmh_build_bw_table_cmd - Build bandwidth table command
 * @cmd: Pointer to the hfi_bwtable_cmd structure
 * @ddr: Pointer to the DDR RPMh bandwidth votes
 * @cnoc: Pointer to the CNOC RPMh bandwidth votes
 */
void adreno_rpmh_build_bw_table_cmd(struct hfi_bwtable_cmd *cmd, struct rpmh_bw_votes *ddr,
		struct rpmh_bw_votes *cnoc);
#endif
