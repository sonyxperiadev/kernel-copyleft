// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/types.h>
#include <soc/qcom/cmd-db.h>
#include <soc/qcom/tcs.h>

#include "adreno.h"
#include "adreno_gen8.h"
#include "adreno_rpmh.h"
#include "kgsl_bus.h"
#include "kgsl_device.h"

/*
 * setup_gmu_arc_votes - Build the gmu voting table
 * @gmu: Pointer to gmu device
 * @pri_rail: Pointer to primary power rail vlvl table
 * @sec_rail: Pointer to second/dependent power rail vlvl table
 *
 * This function initializes the cx votes for all gmu frequencies
 * for gmu dcvs
 */
static int setup_cx_arc_votes(struct gen8_gmu_device *gmu,
	struct rpmh_arc_vals *pri_rail, struct rpmh_arc_vals *sec_rail)
{
	/* Hardcoded values of GMU CX voltage levels */
	u16 gmu_cx_vlvl[MAX_CX_LEVELS];
	u32 cx_votes[MAX_CX_LEVELS];
	struct gen8_dcvs_table *table = &gmu->dcvs_table;
	u32 *freqs = gmu->freqs;
	u32 *vlvls = gmu->vlvls;
	int ret, i;

	gmu_cx_vlvl[0] = 0;
	gmu_cx_vlvl[1] = vlvls[0];
	gmu_cx_vlvl[2] = vlvls[1];

	table->gmu_level_num = 3;

	table->cx_votes[0].freq = 0;
	table->cx_votes[1].freq = freqs[0] / 1000;
	table->cx_votes[2].freq = freqs[1] / 1000;

	ret = adreno_rpmh_setup_volt_dependency_tbl(cx_votes, pri_rail,
			sec_rail, gmu_cx_vlvl, table->gmu_level_num);
	if (!ret) {
		for (i = 0; i < table->gmu_level_num; i++)
			table->cx_votes[i].vote = cx_votes[i];
	}

	return ret;
}

#define GEN8_DEP_VOTE_SET(cx, mx) \
	(FIELD_PREP(GENMASK(31, 14), 0x3FFFF) | \
	 FIELD_PREP(GENMASK(13, 8), mx) | \
	 FIELD_PREP(GENMASK(7, 0), cx))

static int setup_dependency_domain_tbl(u32 *votes,
		struct rpmh_arc_vals *dep_rail, struct rpmh_arc_vals *cx_rail,
		u16 *vlvl, u32 *cx_vlvl, u32 num_entries)
{
	u32 cx_vote;
	int i, j;

	for (i = 1; i < num_entries; i++) {
		bool found_match = false;

		if (cx_vlvl[i] == 0xffffffff) {
			/* This means that the Gx level doesn't have a dependency on Cx level */
			cx_vote = 0xff;
			found_match = true;
		} else {
			for (j = 0; j < cx_rail->num; j++) {
				if (cx_rail->val[j] >= cx_vlvl[i]) {
					cx_vote = j;
					found_match = true;
					break;
				}
			}
		}

		/* If we did not find a matching VLVL level then abort */
		if (!found_match) {
			pr_err("kgsl: Unsupported cx corner: %u\n", cx_vlvl[i]);
			return -EINVAL;
		}

		/*
		 * Set MX dependency domain votes for GX level. Look for indexes
		 * whose vlvl value is greater than or equal to the vlvl value
		 * of the corresponding index of dependency rail
		 */
		for (j = 0; j < dep_rail->num; j++) {
			if (dep_rail->val[j] >= vlvl[i] || j+1 == dep_rail->num)
				break;
		}

		votes[i] = GEN8_DEP_VOTE_SET(cx_vote, j);
	}

	return 0;
}

/*
 * setup_gx_arc_votes - Build the gpu dcvs voting table
 * @hfi: Pointer to hfi device
 * @pri_rail: Pointer to primary power rail vlvl table
 * @sec_rail: Pointer to second/dependent power rail vlvl table
 * @gmxc_rail: Pointer to MxG power rail vlvl table
 *
 * This function initializes the gx votes for all gpu frequencies
 * for gpu dcvs
 */
static int setup_gx_arc_votes(struct adreno_device *adreno_dev,
	struct rpmh_arc_vals *pri_rail, struct rpmh_arc_vals *sec_rail,
	struct rpmh_arc_vals *gmxc_rail, struct rpmh_arc_vals *cx_rail)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct gen8_dcvs_table *table = &gmu->dcvs_table;
	u16 vlvl_tbl[MAX_GX_LEVELS];
	u32 cx_vlvl_tbl[MAX_GX_LEVELS];
	u32 gx_votes[MAX_GX_LEVELS];
	u32 dep_votes[MAX_GX_LEVELS];
	int ret, i;

	table->gpu_level_num = pwr->num_pwrlevels + 1;

	if (table->gpu_level_num > ARRAY_SIZE(vlvl_tbl)) {
		dev_err(device->dev,
			"Defined more GPU DCVS levels than RPMh can support\n");
		return -ERANGE;
	}

	/* Initialize vlvl tables */
	memset(vlvl_tbl, 0, sizeof(vlvl_tbl));
	memset(cx_vlvl_tbl, 0, sizeof(cx_vlvl_tbl));

	/* Fill the vlvl tables. GMU power levels are in ascending order */
	for (i = 1; i < table->gpu_level_num; i++) {
		vlvl_tbl[i] = pwr->pwrlevels[pwr->num_pwrlevels - i].voltage_level;
		cx_vlvl_tbl[i] = pwr->pwrlevels[pwr->num_pwrlevels - i].cx_level;
	}

	/* If the target does not have a dedicated Mx rail, use secondary rail */
	if (gmxc_rail == NULL)
		ret = adreno_rpmh_setup_volt_dependency_tbl(gx_votes, pri_rail, sec_rail,
				vlvl_tbl, table->gpu_level_num);
	else
		ret = adreno_rpmh_setup_volt_dependency_tbl(gx_votes, pri_rail, gmxc_rail,
				vlvl_tbl, table->gpu_level_num);
	if (ret)
		return ret;

	ret = setup_dependency_domain_tbl(dep_votes, sec_rail, cx_rail,
			vlvl_tbl, cx_vlvl_tbl, table->gpu_level_num);
	if (ret)
		return ret;

	/* Populate DCVS table with all the votes */
	for (i = 1; i < table->gpu_level_num; i++) {
		table->gx_votes[i].freq = pwr->pwrlevels[pwr->num_pwrlevels - i].gpu_freq / 1000;
		table->gx_votes[i].vote = gx_votes[i];
		table->gx_votes[i].dep_vote = dep_votes[i];
	}

	/* Add the zero powerlevel for the perf table */
	table->gx_votes[0].freq = 0;
	table->gx_votes[0].vote = 0;
	table->gx_votes[0].dep_vote = 0xFFFFFFFF;

	return ret;
}

static int build_dcvs_table(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	struct rpmh_arc_vals gx_arc, cx_arc, mx_arc, gmxc_arc;
	int ret;

	ret = adreno_rpmh_arc_cmds(&gx_arc, "gfx.lvl");
	if (ret)
		return ret;

	ret = adreno_rpmh_arc_cmds(&cx_arc, "cx.lvl");
	if (ret)
		return ret;

	ret = adreno_rpmh_arc_cmds(&mx_arc, "mx.lvl");
	if (ret)
		return ret;

	ret = setup_cx_arc_votes(gmu, &cx_arc, &mx_arc);
	if (ret)
		return ret;

	/* If the target supports dedicated MxC rail, read the same */
	if (cmd_db_read_addr("gmxc.lvl")) {
		ret = adreno_rpmh_arc_cmds(&gmxc_arc, "gmxc.lvl");
		/* Dummy gMxC resource, treat as if no dedicated MxC */
		if (ret == -ENODATA)
			ret = setup_gx_arc_votes(adreno_dev, &gx_arc, &mx_arc, NULL, &cx_arc);
		else
			ret = setup_gx_arc_votes(adreno_dev, &gx_arc, &mx_arc, &gmxc_arc, &cx_arc);
	} else {
		/* No gMxC resource entry, treat as if no dedicated MxC */
		ret = setup_gx_arc_votes(adreno_dev, &gx_arc, &mx_arc, NULL, &cx_arc);
	}

	return ret;
}

/* BIT(2) is used to vote for GPU performance mode through GMU */
#define ACV_GPU_PERFMODE_VOTE	BIT(2)

static int build_bw_table(struct adreno_device *adreno_dev)
{
	struct gen8_gmu_device *gmu = to_gen8_gmu(adreno_dev);
	const struct adreno_gen8_core *gen8_core = to_gen8_core(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct rpmh_bw_votes *ddr, *cnoc = NULL;
	u32 perfmode_lvl = kgsl_pwrctrl_get_acv_perfmode_lvl(device,
			gen8_core->acv_perfmode_ddr_freq);
	u32 *cnoc_table;
	u32 count;
	int ret;

	ddr = adreno_rpmh_build_bw_votes(adreno_ddr_bcms, ARRAY_SIZE(adreno_ddr_bcms),
		pwr->ddr_table, pwr->ddr_table_count, ACV_GPU_PERFMODE_VOTE, perfmode_lvl,
		adreno_dev->gmu_ab);

	if (IS_ERR(ddr))
		return PTR_ERR(ddr);

	cnoc_table = kgsl_bus_get_table(device->pdev, "qcom,bus-table-cnoc",
		&count);

	if (count > 0)
		cnoc = adreno_rpmh_build_bw_votes(adreno_cnoc_bcms,
			ARRAY_SIZE(adreno_cnoc_bcms), cnoc_table, count, 0, 0,
			adreno_dev->gmu_ab);

	kfree(cnoc_table);

	if (IS_ERR(cnoc)) {
		adreno_rpmh_free_bw_votes(ddr);
		return PTR_ERR(cnoc);
	}

	ret = CMD_MSG_HDR(gmu->hfi.bw_table, H2F_MSG_BW_VOTE_TBL);
	if (ret)
		return ret;

	adreno_rpmh_build_bw_table_cmd(&gmu->hfi.bw_table, ddr, cnoc);

	adreno_rpmh_free_bw_votes(ddr);
	adreno_rpmh_free_bw_votes(cnoc);

	return 0;
}

int gen8_build_rpmh_tables(struct adreno_device *adreno_dev)
{
	int ret;

	ret = build_dcvs_table(adreno_dev);
	if (ret) {
		dev_err(adreno_dev->dev.dev, "Failed to build dcvs table\n");
		return ret;
	}

	ret = build_bw_table(adreno_dev);
	if (ret)
		dev_err(adreno_dev->dev.dev, "Failed to build bw table\n");

	return ret;
}
