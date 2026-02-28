// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/types.h>
#include <soc/qcom/cmd-db.h>
#include <soc/qcom/tcs.h>

#include "adreno.h"
#include "adreno_rpmh.h"

int adreno_rpmh_arc_cmds(struct rpmh_arc_vals *arc, const char *res_id)
{
	size_t len = 0;
	bool dummy_res = true;

	arc->val = cmd_db_read_aux_data(res_id, &len);

	/*
	 * cmd_db_read_aux_data() gives us a zero-padded table of
	 * size len that contains the arc values. To determine the
	 * number of arc values, we loop through the table and count
	 * them until we get to the end of the buffer or hit the
	 * zero padding.
	 */
	for (arc->num = 1; arc->num < (len >> 1); arc->num++) {
		if (arc->val[arc->num] != 0)
			dummy_res = false;

		if (arc->val[arc->num - 1] != 0 && arc->val[arc->num] == 0)
			break;
	}

	/* Dummy resource entry in cmd_db with all zeros */
	if (dummy_res)
		return -ENODATA;

	return 0;
}

int adreno_rpmh_setup_volt_dependency_tbl(u32 *votes, struct rpmh_arc_vals *pri_rail,
		struct rpmh_arc_vals *sec_rail, u16 *vlvl, u32 num_entries)
{
	int i, j, k;
	uint16_t cur_vlvl;
	bool found_match;

	/*
	 * i tracks current KGSL GPU frequency table entry
	 * j tracks secondary rail voltage table entry
	 * k tracks primary rail voltage table entry
	 */
	for (i = 0; i < num_entries; i++) {
		found_match = false;

		/* Look for a primary rail voltage that matches a VLVL level */
		for (k = 0; k < pri_rail->num; k++) {
			if (pri_rail->val[k] >= vlvl[i]) {
				cur_vlvl = pri_rail->val[k];
				found_match = true;
				break;
			}
		}

		/* If we did not find a matching VLVL level then abort */
		if (!found_match)
			return -EINVAL;

		/*
		 * Look for a secondary rail index whose VLVL value
		 * is greater than or equal to the VLVL value of the
		 * corresponding index of the primary rail
		 */
		for (j = 0; j < sec_rail->num; j++) {
			if (sec_rail->val[j] >= cur_vlvl ||
					j + 1 == sec_rail->num)
				break;
		}

		if (j == sec_rail->num)
			j = 0;

		votes[i] = ARC_VOTE_SET(k, j, cur_vlvl);
	}

	return 0;
}

/* Generate a set of bandwidth votes for the list of BCMs */
static void tcs_cmd_data(struct bcm *bcms, int count, u32 ab, u32 ib, u32 *data,
		u32 perfmode_vote)
{
	int i;

	for (i = 0; i < count; i++) {
		bool valid = true;
		bool commit = false;
		u64 avg, peak, x, y;

		if (i == count - 1 || bcms[i].vcd != bcms[i + 1].vcd)
			commit = true;

		if (bcms[i].fixed) {
			if (!ab && !ib)
				data[i] = BCM_TCS_CMD(commit, false, 0x0, 0x0);
			else
				data[i] = BCM_TCS_CMD(commit, true, 0x0, perfmode_vote);
			continue;
		}

		/* Multiple the bandwidth by the width of the connection */
		avg = ((u64) ab) * bcms[i].width;

		/* And then divide by the total width */
		do_div(avg, bcms[i].buswidth);

		peak = ((u64) ib) * bcms[i].width;
		do_div(peak, bcms[i].buswidth);

		/* Input bandwidth value is in KBps */
		x = avg * 1000ULL;
		do_div(x, bcms[i].unit);

		/* Input bandwidth value is in KBps */
		y = peak * 1000ULL;
		do_div(y, bcms[i].unit);

		/*
		 * If a bandwidth value was specified but the calculation ends
		 * rounding down to zero, set a minimum level
		 */
		if (ab && x == 0)
			x = 1;

		if (ib && y == 0)
			y = 1;

		x = min_t(u64, x, BCM_TCS_CMD_VOTE_MASK);
		y = min_t(u64, y, BCM_TCS_CMD_VOTE_MASK);

		if (!x && !y)
			valid = false;

		data[i] = BCM_TCS_CMD(commit, valid, x, y);
	}
}

void adreno_rpmh_free_bw_votes(struct rpmh_bw_votes *votes)
{
	int i;

	if (!votes)
		return;

	for (i = 0; votes->cmds && i < votes->num_levels; i++)
		kfree(votes->cmds[i]);

	kfree(votes->cmds);
	kfree(votes->addrs);
	kfree(votes);
}

struct rpmh_bw_votes *adreno_rpmh_build_bw_votes(struct bcm *bcms, int bcm_count,
		u32 *levels, int levels_count, u32 perfmode_vote, u32 perfmode_lvl,
		bool gmu_ab)
{
	struct rpmh_bw_votes *votes;
	int i;

	votes = kzalloc(sizeof(*votes), GFP_KERNEL);
	if (!votes)
		return ERR_PTR(-ENOMEM);

	votes->addrs = kcalloc(bcm_count, sizeof(*votes->cmds), GFP_KERNEL);
	if (!votes->addrs) {
		adreno_rpmh_free_bw_votes(votes);
		return ERR_PTR(-ENOMEM);
	}

	votes->cmds = kcalloc(levels_count, sizeof(*votes->cmds), GFP_KERNEL);
	if (!votes->cmds) {
		adreno_rpmh_free_bw_votes(votes);
		return ERR_PTR(-ENOMEM);
	}

	votes->num_cmds = bcm_count;
	votes->num_levels = levels_count;

	/* Get the cmd-db information for each BCM */
	for (i = 0; i < bcm_count; i++) {
		size_t l;
		const struct bcm_data *data;

		data = cmd_db_read_aux_data(bcms[i].name, &l);

		votes->addrs[i] = cmd_db_read_addr(bcms[i].name);

		bcms[i].unit = le32_to_cpu(data->unit);
		bcms[i].width = le16_to_cpu(data->width);
		bcms[i].vcd = data->vcd;
	}

	for (i = 0; i < bcm_count; i++) {
		if (i == (bcm_count - 1) || bcms[i].vcd != bcms[i + 1].vcd)
			votes->wait_bitmask |= (1 << i);
	}

	for (i = 0; i < levels_count; i++) {
                votes->cmds[i] = kcalloc(bcm_count, sizeof(u32), GFP_KERNEL);
                if (!votes->cmds[i]) {
                        adreno_rpmh_free_bw_votes(votes);
                        return ERR_PTR(-ENOMEM);
                }

                tcs_cmd_data(bcms, bcm_count, gmu_ab ? levels[i] : 0x0,
                        levels[i], votes->cmds[i],
                        (i >= perfmode_lvl) ? perfmode_vote : 0x0);
        }

        return votes;
}

int adreno_rpmh_to_cx_hlvl(struct rpmh_arc_vals *cx_rail, u32 vlvl, u32 *hlvl)
{
	u32 i;

	/*
	 * This means that the Gx level doesn't have a dependency on Cx level.
	 * Return the same value to disable cx voting at GMU.
	 */
	if (vlvl == 0xffffffff) {
		*hlvl = vlvl;
		return 0;
	}

	for (i = 0; i < cx_rail->num; i++) {
		if (cx_rail->val[i] >= vlvl) {
			*hlvl = i;
			return 0;
		}
	}

	return -EINVAL;
}

void adreno_rpmh_build_bw_table_cmd(struct hfi_bwtable_cmd *cmd, struct rpmh_bw_votes *ddr,
		struct rpmh_bw_votes *cnoc)
{
	u32 i, j;

	cmd->bw_level_num = ddr->num_levels;
	cmd->ddr_cmds_num = ddr->num_cmds;
	cmd->ddr_wait_bitmask = ddr->wait_bitmask;

	for (i = 0; i < ddr->num_cmds; i++)
		cmd->ddr_cmd_addrs[i] = ddr->addrs[i];

	for (i = 0; i < ddr->num_levels; i++)
		for (j = 0; j < ddr->num_cmds; j++)
			cmd->ddr_cmd_data[i][j] = (u32) ddr->cmds[i][j];

	if (!cnoc)
		return;

	cmd->cnoc_cmds_num = cnoc->num_cmds;
		cmd->cnoc_wait_bitmask = cnoc->wait_bitmask;

	for (i = 0; i < cnoc->num_cmds; i++)
		cmd->cnoc_cmd_addrs[i] = cnoc->addrs[i];

	for (i = 0; i < cnoc->num_levels; i++)
		for (j = 0; j < cnoc->num_cmds; j++)
			cmd->cnoc_cmd_data[i][j] = (u32) cnoc->cmds[i][j];
}
