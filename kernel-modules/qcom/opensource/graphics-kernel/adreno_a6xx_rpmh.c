// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/types.h>
#include <soc/qcom/cmd-db.h>
#include <soc/qcom/tcs.h>

#include "adreno.h"
#include "adreno_a6xx.h"
#include "adreno_hfi.h"
#include "adreno_rpmh.h"
#include "kgsl_bus.h"
#include "kgsl_device.h"

/*
 * setup_gmu_arc_votes - Build the gmu voting table
 * @adreno_dev: Pointer to adreno device
 * @pri_rail: Pointer to primary power rail vlvl table
 * @sec_rail: Pointer to second/dependent power rail vlvl table
 * @freqs: List of GMU frequencies
 * @vlvls: List of GMU voltage levels
 *
 * This function initializes the cx votes for all gmu frequencies
 * for gmu dcvs
 */
static int setup_cx_arc_votes(struct adreno_device *adreno_dev,
	struct rpmh_arc_vals *pri_rail, struct rpmh_arc_vals *sec_rail,
	u32 *freqs, u32 *vlvls)
{
	/* Hardcoded values of GMU CX voltage levels */
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct a6xx_hfi *hfi = &gmu->hfi;
	u16 gmu_cx_vlvl[MAX_CX_LEVELS];
	u32 cx_votes[MAX_CX_LEVELS];
	struct hfi_dcvstable_cmd *table = &hfi->dcvs_table;
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

/*
 * setup_gx_arc_votes - Build the gpu dcvs voting table
 * @hfi: Pointer to hfi device
 * @pri_rail: Pointer to primary power rail vlvl table
 * @sec_rail: Pointer to second/dependent power rail vlvl table
 *
 * This function initializes the gx votes for all gpu frequencies
 * for gpu dcvs
 */
static int setup_gx_arc_votes(struct adreno_device *adreno_dev,
	struct rpmh_arc_vals *pri_rail, struct rpmh_arc_vals *sec_rail,
	struct rpmh_arc_vals *cx_rail)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct hfi_dcvstable_cmd *table = &gmu->hfi.dcvs_table;
	u32 index;
	u16 vlvl_tbl[MAX_GX_LEVELS];
	u32 gx_votes[MAX_GX_LEVELS];
	int ret, i;

	/* Add the zero powerlevel for the perf table */
	table->gpu_level_num = device->pwrctrl.num_pwrlevels + 1;

	if (table->gpu_level_num > pri_rail->num ||
		table->gpu_level_num > ARRAY_SIZE(vlvl_tbl)) {
		dev_err(GMU_PDEV_DEV(device),
			"Defined more GPU DCVS levels than RPMh can support\n");
		return -ERANGE;
	}

	memset(vlvl_tbl, 0, sizeof(vlvl_tbl));

	table->gx_votes[0].freq = 0;
	table->gx_votes[0].dep_vote = 0;
	/* Disable cx vote in gmu dcvs table if it is not supported in DT */
	if (pwr->pwrlevels[0].cx_level == 0xffffffff)
		table->gx_votes[0].dep_vote = 0xffffffff;

	/* GMU power levels are in ascending order */
	for (index = 1, i = pwr->num_pwrlevels - 1; i >= 0; i--, index++) {
		u32 cx_vlvl = pwr->pwrlevels[i].cx_level;

		vlvl_tbl[index] = pwr->pwrlevels[i].voltage_level;
		table->gx_votes[index].freq = pwr->pwrlevels[i].gpu_freq / 1000;

		ret = adreno_rpmh_to_cx_hlvl(cx_rail, cx_vlvl,
				&table->gx_votes[index].dep_vote);
		if (ret) {
			dev_err(GMU_PDEV_DEV(device), "Unsupported cx corner: %u\n",
					cx_vlvl);
			return ret;
		}
	}

	ret = adreno_rpmh_setup_volt_dependency_tbl(gx_votes, pri_rail,
			sec_rail, vlvl_tbl, table->gpu_level_num);
	if (!ret) {
		for (i = 0; i < table->gpu_level_num; i++)
			table->gx_votes[i].vote = gx_votes[i];
	}

	return ret;

}

static int build_dcvs_table(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct a6xx_hfi *hfi = &gmu->hfi;
	struct rpmh_arc_vals gx_arc, cx_arc, mx_arc;
	int ret;

	ret = CMD_MSG_HDR(hfi->dcvs_table, H2F_MSG_PERF_TBL);
	if (ret)
		return ret;

	ret = adreno_rpmh_arc_cmds(&gx_arc, "gfx.lvl");
	if (ret)
		return ret;

	ret = adreno_rpmh_arc_cmds(&cx_arc, "cx.lvl");
	if (ret)
		return ret;

	ret = adreno_rpmh_arc_cmds(&mx_arc, "mx.lvl");
	if (ret)
		return ret;

	ret = setup_cx_arc_votes(adreno_dev, &cx_arc, &mx_arc,
					gmu->freqs, gmu->vlvls);
	if (ret)
		return ret;

	return setup_gx_arc_votes(adreno_dev, &gx_arc, &mx_arc, &cx_arc);
}

/* BIT(3) is used to vote for GPU performance mode through GMU */
#define ACV_GPU_PERFMODE_VOTE	BIT(3)

static int build_bw_table(struct adreno_device *adreno_dev)
{
	struct a6xx_gmu_device *gmu = to_a6xx_gmu(adreno_dev);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct rpmh_bw_votes *ddr, *cnoc = NULL;
	u32 *cnoc_table;
	u32 count;
	int ret;

	ddr = adreno_rpmh_build_bw_votes(adreno_ddr_bcms, ARRAY_SIZE(adreno_ddr_bcms),
		pwr->ddr_table, pwr->ddr_table_count, ACV_GPU_PERFMODE_VOTE, 0,
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

int a6xx_build_rpmh_tables(struct adreno_device *adreno_dev)
{
	int ret;

	ret = build_dcvs_table(adreno_dev);
	if (ret)
		return ret;

	return build_bw_table(adreno_dev);
}
