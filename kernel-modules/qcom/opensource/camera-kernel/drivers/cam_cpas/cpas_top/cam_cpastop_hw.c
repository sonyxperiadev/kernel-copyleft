// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>

#include "cam_compat.h"
#include "cam_cpas_hw_intf.h"
#include "cam_cpas_hw.h"
#include "cam_cpastop_hw.h"
#include "cam_io_util.h"
#include "cam_cpas_soc.h"
#include "cpastop100.h"
#include "cpastop_v150_100.h"
#include "cpastop_v170_200.h"
#include "cpastop_v170_110.h"
#include "cpastop_v175_100.h"
#include "cpastop_v175_101.h"
#include "cpastop_v175_120.h"
#include "cpastop_v175_130.h"
#include "cpastop_v480_100.h"
#include "cpastop_v480_custom.h"
#include "cpastop_v580_100.h"
#include "cpastop_v580_custom.h"
#include "cpastop_v540_100.h"
#include "cpastop_v520_100.h"
#include "cpastop_v545_100.h"
#include "cpastop_v570_100.h"
#include "cpastop_v570_200.h"
#include "cpastop_v680_100.h"
#include "cpastop_v680_110.h"
#include "cpastop_v165_100.h"
#include "cpastop_v780_100.h"
#include "cpastop_v640_200.h"
#include "cpastop_v880_100.h"
#include "cpastop_v980_100.h"
#include "cpastop_v860_100.h"
#include "cam_req_mgr_workq.h"
#include "cam_common_util.h"

struct cam_camnoc_info *camnoc_info[CAM_CAMNOC_HW_TYPE_MAX];
struct cam_cpas_info *cpas_info;

#if (defined(CONFIG_CAM_TEST_IRQ_LINE) && defined(CONFIG_CAM_TEST_IRQ_LINE_AT_PROBE))
	struct completion test_irq_hw_complete[CAM_CAMNOC_HW_TYPE_MAX];
#endif

#define CAMNOC_SLAVE_MAX_ERR_CODE 7
static const char * const camnoc_slave_err_code[] = {
	"Target Error",              /* err code 0 */
	"Address decode error",      /* err code 1 */
	"Unsupported request",       /* err code 2 */
	"Disconnected target",       /* err code 3 */
	"Security violation",        /* err code 4 */
	"Hidden security violation", /* err code 5 */
	"Timeout Error",             /* err code 6 */
	"Unknown Error",             /* unknown err code */
};

static const uint32_t cam_cpas_hw_version_map
	[CAM_CPAS_CAMERA_VERSION_ID_MAX][CAM_CPAS_VERSION_ID_MAX] = {
	/* for camera_150 */
	{
		CAM_CPAS_TITAN_150_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_170 */
	{
		CAM_CPAS_TITAN_170_V100,
		0,
		CAM_CPAS_TITAN_170_V110,
		CAM_CPAS_TITAN_170_V120,
		0,
		CAM_CPAS_TITAN_170_V200,
	},
	/* for camera_175 */
	{
		CAM_CPAS_TITAN_175_V100,
		CAM_CPAS_TITAN_175_V101,
		0,
		CAM_CPAS_TITAN_175_V120,
		CAM_CPAS_TITAN_175_V130,
		0,
	},
	/* for camera_480 */
	{
		CAM_CPAS_TITAN_480_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_580 */
	{
		CAM_CPAS_TITAN_580_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_520 */
	{
		CAM_CPAS_TITAN_520_V100,
		0,
		0,
		0,
		0,
		0,

	},
	/* for camera_540 */
	{
		CAM_CPAS_TITAN_540_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_545 */
	{
		CAM_CPAS_TITAN_545_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_570 */
	{
		CAM_CPAS_TITAN_570_V100,
		0,
		0,
		0,
		0,
		CAM_CPAS_TITAN_570_V200,
	},
	/* for camera_680 */
	{
		CAM_CPAS_TITAN_680_V100,
		0,
		CAM_CPAS_TITAN_680_V110,
		0,
		0,
		0,
	},
	/* for camera_165 */
	{
		CAM_CPAS_TITAN_165_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_780 */
	{
		CAM_CPAS_TITAN_780_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_640 */
	{
		0,
		0,
		0,
		0,
		0,
		CAM_CPAS_TITAN_640_V200,
	},
	/* for camera_880 */
	{
		CAM_CPAS_TITAN_880_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_980 */
	{
		CAM_CPAS_TITAN_980_V100,
		0,
		0,
		0,
		0,
		0,
	},
	/* for camera_860 */
	{
		CAM_CPAS_TITAN_860_V100,
		0,
		0,
		0,
		0,
		0,
	},
};

static char *cam_cpastop_get_camnoc_name(enum cam_camnoc_hw_type type)
{
	switch (type) {
	case CAM_CAMNOC_HW_COMBINED:
		return "CAMNOC_COMBINED";
	case CAM_CAMNOC_HW_RT:
		return "CAMNOC_RT";
	case CAM_CAMNOC_HW_NRT:
		return "CAMNOC_NRT";
	default:
		return "Invalid CAMNOC";
	}
}

static int cam_cpas_translate_camera_cpas_version_id(
	uint32_t cam_version,
	uint32_t cpas_version,
	uint32_t *cam_version_id,
	uint32_t *cpas_version_id)
{

	switch (cam_version) {

	case CAM_CPAS_CAMERA_VERSION_150:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_150;
		break;

	case CAM_CPAS_CAMERA_VERSION_170:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_170;
		break;

	case CAM_CPAS_CAMERA_VERSION_175:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_175;
		break;

	case CAM_CPAS_CAMERA_VERSION_480:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_480;
		break;

	case CAM_CPAS_CAMERA_VERSION_520:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_520;
		break;

	case CAM_CPAS_CAMERA_VERSION_540:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_540;
		break;

	case CAM_CPAS_CAMERA_VERSION_580:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_580;
		break;

	case CAM_CPAS_CAMERA_VERSION_545:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_545;
		break;

	case CAM_CPAS_CAMERA_VERSION_570:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_570;
		break;

	case CAM_CPAS_CAMERA_VERSION_680:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_680;
		break;

	case CAM_CPAS_CAMERA_VERSION_165:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_165;
		break;

	case CAM_CPAS_CAMERA_VERSION_780:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_780;
		break;
	case CAM_CPAS_CAMERA_VERSION_640:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_640;
		break;
	case CAM_CPAS_CAMERA_VERSION_880:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_880;
		break;
	case CAM_CPAS_CAMERA_VERSION_980:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_980;
		break;
	case CAM_CPAS_CAMERA_VERSION_860:
		*cam_version_id = CAM_CPAS_CAMERA_VERSION_ID_860;
		break;
	default:
		CAM_ERR(CAM_CPAS, "Invalid cam version %u",
			cam_version);
		return -EINVAL;
	}

	switch (cpas_version) {

	case CAM_CPAS_VERSION_100:
		*cpas_version_id = CAM_CPAS_VERSION_ID_100;
		break;

	case CAM_CPAS_VERSION_101:
		*cpas_version_id = CAM_CPAS_VERSION_ID_101;
		break;
	case CAM_CPAS_VERSION_110:
		*cpas_version_id = CAM_CPAS_VERSION_ID_110;
		break;

	case CAM_CPAS_VERSION_120:
		*cpas_version_id = CAM_CPAS_VERSION_ID_120;
		break;

	case CAM_CPAS_VERSION_130:
		*cpas_version_id = CAM_CPAS_VERSION_ID_130;
		break;

	case CAM_CPAS_VERSION_200:
		*cpas_version_id = CAM_CPAS_VERSION_ID_200;
		break;

	default:
		CAM_ERR(CAM_CPAS, "Invalid cpas version %u",
			cpas_version);
		return -EINVAL;
	}
	return 0;
}

static int cam_cpastop_get_hw_info(struct cam_hw_info *cpas_hw,
	struct cam_cpas_hw_caps *hw_caps)
{
	struct cam_cpas *cpas_core = (struct cam_cpas *) cpas_hw->core_info;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;
	int32_t reg_indx = cpas_core->regbase_index[CAM_CPAS_REG_CPASTOP];
	uint32_t cam_version, cpas_version;
	uint32_t cam_version_id, cpas_version_id;
	int rc;

	if (reg_indx == -1) {
		CAM_ERR(CAM_CPAS, "Invalid arguments");
		return -EINVAL;
	}

	hw_caps->camera_family = CAM_FAMILY_CPAS_SS;

	cam_version = cam_io_r_mb(soc_info->reg_map[reg_indx].mem_base + 0x0);
	hw_caps->camera_version.major =
		CAM_BITS_MASK_SHIFT(cam_version, 0xff0000, 0x10);
	hw_caps->camera_version.minor =
		CAM_BITS_MASK_SHIFT(cam_version, 0xff00, 0x8);
	hw_caps->camera_version.incr =
		CAM_BITS_MASK_SHIFT(cam_version, 0xff, 0x0);

	cpas_version = cam_io_r_mb(soc_info->reg_map[reg_indx].mem_base + 0x4);
	hw_caps->cpas_version.major =
		CAM_BITS_MASK_SHIFT(cpas_version, 0xf0000000, 0x1c);
	hw_caps->cpas_version.minor =
		CAM_BITS_MASK_SHIFT(cpas_version, 0xfff0000, 0x10);
	hw_caps->cpas_version.incr =
		CAM_BITS_MASK_SHIFT(cpas_version, 0xffff, 0x0);

	CAM_DBG(CAM_CPAS, "Family %d, version %d.%d.%d, cpas %d.%d.%d",
		hw_caps->camera_family, hw_caps->camera_version.major,
		hw_caps->camera_version.minor, hw_caps->camera_version.incr,
		hw_caps->cpas_version.major, hw_caps->cpas_version.minor,
		hw_caps->cpas_version.incr);

	soc_info->hw_version = CAM_CPAS_TITAN_NONE;
	rc = cam_cpas_translate_camera_cpas_version_id(cam_version,
		cpas_version, &cam_version_id, &cpas_version_id);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Invalid Version, Camera: 0x%x CPAS: 0x%x",
			cam_version, cpas_version);
		return -EINVAL;
	}

	soc_info->hw_version =
		cam_cpas_hw_version_map[cam_version_id][cpas_version_id];

	CAM_DBG(CAM_CPAS, "CPAS HW VERSION %x", soc_info->hw_version);

	return 0;
}

static int cam_cpastop_setup_regbase_indices(struct cam_hw_soc_info *soc_info,
	int32_t regbase_index[], int32_t num_reg_map)
{
	uint32_t index;
	int rc;

	if (num_reg_map > CAM_CPAS_REG_MAX) {
		CAM_ERR(CAM_CPAS, "invalid num_reg_map=%d", num_reg_map);
		return -EINVAL;
	}

	if (soc_info->num_mem_block > CAM_SOC_MAX_BLOCK) {
		CAM_ERR(CAM_CPAS, "invalid num_mem_block=%d",
			soc_info->num_mem_block);
		return -EINVAL;
	}

	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "cam_cpas_top", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[CAM_CPAS_REG_CPASTOP] = index;
	} else if (rc) {
		CAM_ERR(CAM_CPAS, "failed to get index for CPASTOP rc=%d", rc);
		return rc;
	} else {
		CAM_ERR(CAM_CPAS, "regbase not found for CPASTOP, rc=%d, %d %d",
			rc, index, num_reg_map);
		return -EINVAL;
	}

	/*
	 * camnoc reg base represents reg base for legacy camnoc, this becomes optional since
	 * some targets have camnoc split into RT/NRT
	 */
	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "cam_camnoc", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[CAM_CPAS_REG_CAMNOC] = index;
		CAM_DBG(CAM_CPAS, "regbase found for CAMNOC, rc=%d, %d %d",
			rc, index, num_reg_map);
	}  else {
		CAM_DBG(CAM_CPAS, "regbase not found for CAMNOC, rc=%d, %d %d",
			rc, index, num_reg_map);
		regbase_index[CAM_CPAS_REG_CAMNOC] = -1;
	}

	/* optional - reg base for a target where camnoc reg is split into two domains: rt/nrt */
	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "cam_camnoc_rt", &index);
	if ((rc == 0) &&  (index < num_reg_map)) {
		regbase_index[CAM_CPAS_REG_CAMNOC_RT] = index;
		CAM_DBG(CAM_CPAS, "regbase found for CAMNOC RT, rc=%d, %d %d",
			rc, index, num_reg_map);
	} else {
		CAM_DBG(CAM_CPAS, "regbase not found for CAMNOC RT, rc=%d, %d %d",
			rc, index, num_reg_map);
		regbase_index[CAM_CPAS_REG_CAMNOC_RT] = -1;
	}

	/* optional - reg base for a target where camnoc reg is split into two domains: rt/nrt */
	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "cam_camnoc_nrt", &index);
	if ((rc == 0) &&  (index < num_reg_map)) {
		regbase_index[CAM_CPAS_REG_CAMNOC_NRT] = index;
		CAM_DBG(CAM_CPAS, "regbase found for CAMNOC NRT, rc=%d, %d %d",
			rc, index, num_reg_map);
	} else {
		CAM_DBG(CAM_CPAS, "regbase not found for CAMNOC NRT, rc=%d, %d %d",
			rc, index, num_reg_map);
		regbase_index[CAM_CPAS_REG_CAMNOC_NRT] = -1;
	}

	/* optional - rpmh register map */
	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "cam_rpmh", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[CAM_CPAS_REG_RPMH] = index;
		CAM_DBG(CAM_CPAS, "regbase found for RPMH, rc=%d, %d %d",
			rc, index, num_reg_map);
	} else {
		CAM_DBG(CAM_CPAS, "regbase not found for RPMH, rc=%d, %d %d",
			rc, index, num_reg_map);
		regbase_index[CAM_CPAS_REG_RPMH] = -1;
	}

	/* optional - cesta register map */
	rc = cam_common_util_get_string_index(soc_info->mem_block_name,
		soc_info->num_mem_block, "cam_cesta", &index);
	if ((rc == 0) && (index < num_reg_map)) {
		regbase_index[CAM_CPAS_REG_CESTA] = index;
		CAM_DBG(CAM_CPAS, "regbase found for cesta, rc=%d, %d %d",
			rc, index, num_reg_map);
	} else {
		CAM_DBG(CAM_CPAS, "regbase not found for cesta, rc=%d, %d %d",
			rc, index, num_reg_map);
		regbase_index[CAM_CPAS_REG_CESTA] = -1;
	}

	return 0;
}

static int cam_cpastop_handle_errlogger(int camnoc_idx,
	struct cam_cpas *cpas_core, struct cam_hw_soc_info *soc_info,
	struct cam_camnoc_irq_slave_err_data *slave_err)
{
	int regbase_idx = cpas_core->regbase_index[camnoc_info[camnoc_idx]->reg_base];
	int err_code_index = 0;

	if (!camnoc_info[camnoc_idx]->err_logger) {
		CAM_ERR_RATE_LIMIT(CAM_CPAS, "Invalid err logger info");
		return -EINVAL;
	}

	slave_err->errlog0_low.value = cam_io_r_mb(
		soc_info->reg_map[regbase_idx].mem_base +
		camnoc_info[camnoc_idx]->err_logger->errlog0_low);

	err_code_index = slave_err->errlog0_low.err_code;
	if (err_code_index > CAMNOC_SLAVE_MAX_ERR_CODE)
		err_code_index = CAMNOC_SLAVE_MAX_ERR_CODE;

	CAM_ERR_RATE_LIMIT(CAM_CPAS,
		"[%s] Possible memory configuration issue, fault at SMMU raised as CAMNOC SLAVE_IRQ err_code=%d(%s)",
		camnoc_info[camnoc_idx]->camnoc_name, slave_err->errlog0_low.err_code,
		camnoc_slave_err_code[err_code_index]);

	return 0;
}

static int cam_cpastop_handle_ubwc_enc_err(int camnoc_idx,
	struct cam_cpas *cpas_core, struct cam_hw_soc_info *soc_info, int i,
	struct cam_camnoc_irq_ubwc_enc_data *enc_err)
{
	int regbase_idx = cpas_core->regbase_index[camnoc_info[camnoc_idx]->reg_base];

	enc_err->encerr_status.value =
		cam_io_r_mb(soc_info->reg_map[regbase_idx].mem_base +
			camnoc_info[camnoc_idx]->irq_err[i].err_status.offset);

	/* Let clients handle the UBWC errors */
	CAM_DBG(CAM_CPAS,
		"[%s] ubwc enc err [%d]: offset[0x%x] value[0x%x]",
		camnoc_info[camnoc_idx]->camnoc_name, i,
		camnoc_info[camnoc_idx]->irq_err[i].err_status.offset,
		enc_err->encerr_status.value);

	return 0;
}

static int cam_cpastop_handle_ubwc_dec_err(int camnoc_idx,
	struct cam_cpas *cpas_core, struct cam_hw_soc_info *soc_info, int i,
	struct cam_camnoc_irq_ubwc_dec_data *dec_err)
{
	int regbase_idx = cpas_core->regbase_index[camnoc_info[camnoc_idx]->reg_base];

	dec_err->decerr_status.value =
		cam_io_r_mb(soc_info->reg_map[regbase_idx].mem_base +
		camnoc_info[camnoc_idx]->irq_err[i].err_status.offset);

	/* Let clients handle the UBWC errors */
	CAM_DBG(CAM_CPAS,
		"[%s] ubwc dec err status [%d]: offset[0x%x] value[0x%x] thr_err=%d, fcl_err=%d, len_md_err=%d, format_err=%d",
		camnoc_info[camnoc_idx]->camnoc_name, i,
		camnoc_info[camnoc_idx]->irq_err[i].err_status.offset, dec_err->decerr_status.value,
		dec_err->decerr_status.thr_err, dec_err->decerr_status.fcl_err,
		dec_err->decerr_status.len_md_err, dec_err->decerr_status.format_err);

	return 0;
}

static int cam_cpastop_handle_ahb_timeout_err(int camnoc_idx,
	struct cam_hw_info *cpas_hw, struct cam_camnoc_irq_ahb_timeout_data *ahb_err)
{
	CAM_ERR_RATE_LIMIT(CAM_CPAS, "[%s] ahb timeout error",
		camnoc_info[camnoc_idx]->camnoc_name);

	return 0;
}

#if (defined(CONFIG_CAM_TEST_IRQ_LINE) && defined(CONFIG_CAM_TEST_IRQ_LINE_AT_PROBE))
static int cam_cpastop_enable_test_irq(int camnoc_idx,
	struct cam_hw_info *cpas_hw)
{
	int i;

	camnoc_info[camnoc_idx]->irq_sbm->sbm_enable.value |=
		camnoc_info[camnoc_idx]->test_irq_info.sbm_enable_mask;
	camnoc_info[camnoc_idx]->irq_sbm->sbm_clear.value |=
		camnoc_info[camnoc_idx]->test_irq_info.sbm_clear_mask;

	for (i = 0; i < camnoc_info[camnoc_idx]->irq_err_size; i++) {
		if (camnoc_info[camnoc_idx]->irq_err[i].irq_type ==
			CAM_CAMNOC_HW_IRQ_CAMNOC_TEST)
			camnoc_info[camnoc_idx]->irq_err[i].enable = true;
	}

	return 0;
}

static int cam_cpastop_disable_test_irq(int camnoc_idx,
	struct cam_hw_info *cpas_hw)
{
	int i;

	camnoc_info[camnoc_idx]->irq_sbm->sbm_enable.value &=
		~camnoc_info[camnoc_idx]->test_irq_info.sbm_enable_mask;
	camnoc_info[camnoc_idx]->irq_sbm->sbm_clear.value &=
		~camnoc_info[camnoc_idx]->test_irq_info.sbm_clear_mask;

	for (i = 0; i < camnoc_info[camnoc_idx]->irq_err_size; i++) {
		if (camnoc_info[camnoc_idx]->irq_err[i].irq_type ==
			CAM_CAMNOC_HW_IRQ_CAMNOC_TEST)
			camnoc_info[camnoc_idx]->irq_err[i].enable = false;
	}

	return 0;
}

static void cam_cpastop_check_test_irq(int camnoc_idx,
	struct cam_hw_info *cpas_hw, uint32_t irq_status)
{
	int i;

	for (i = 0; i < camnoc_info[camnoc_idx]->irq_err_size; i++) {
		if ((camnoc_info[camnoc_idx]->irq_err[i].irq_type ==
			CAM_CAMNOC_HW_IRQ_CAMNOC_TEST) &&
			(irq_status & camnoc_info[camnoc_idx]->irq_err[i].sbm_port) &&
			(camnoc_info[camnoc_idx]->irq_err[i].enable)) {
			complete(&test_irq_hw_complete[camnoc_idx]);
			CAM_INFO(CAM_CPAS, "[%s] Test IRQ triggerred",
				camnoc_info[camnoc_idx]->camnoc_name);
		}
	}
}
#endif

static void cam_cpastop_enable_camnoc_irqs(
	struct cam_hw_info *cpas_hw, int camnoc_idx)
{
	int i;
	struct cam_cpas *cpas_core = (struct cam_cpas *) cpas_hw->core_info;

	cpas_core->smmu_fault_handled = false;

	/* clear and enable all irq errors */
	for (i = 0; i < camnoc_info[camnoc_idx]->irq_err_size; i++) {
		if (camnoc_info[camnoc_idx]->irq_err[i].enable) {
			cam_cpas_util_reg_update(cpas_hw, camnoc_info[camnoc_idx]->reg_base,
				&camnoc_info[camnoc_idx]->irq_err[i].err_clear);
			cam_cpas_util_reg_update(cpas_hw, camnoc_info[camnoc_idx]->reg_base,
				&camnoc_info[camnoc_idx]->irq_err[i].err_enable);
		}
	}

	/* On poweron reset enable all error irqs applicable for the target */
	cam_cpas_util_reg_update(cpas_hw, camnoc_info[camnoc_idx]->reg_base,
		&camnoc_info[camnoc_idx]->irq_sbm->sbm_enable);
}

static void cam_cpastop_handle_camnoc_irqs(uint32_t irq_status,
	struct cam_hw_info *cpas_hw, int camnoc_idx)
{
	int i, regbase_idx;
	struct cam_cpas *cpas_core = cpas_hw->core_info;
	uint32_t updated_sbm_mask = 0;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;
	enum cam_cpas_reg_base reg_base;

	reg_base = camnoc_info[camnoc_idx]->reg_base;
	regbase_idx = cpas_core->regbase_index[reg_base];

	for (i = 0; i < camnoc_info[camnoc_idx]->irq_err_size; i++) {
		if ((camnoc_info[camnoc_idx]->irq_err[i].enable) &&
			(camnoc_info[camnoc_idx]->irq_err[i].sbm_port & irq_status)) {
			/* Clear the error status */
			cam_cpas_util_reg_update(cpas_hw, reg_base,
				&camnoc_info[camnoc_idx]->irq_err[i].err_clear);
			updated_sbm_mask |= camnoc_info[camnoc_idx]->irq_err[i].sbm_port;
		}
	}

	/* Disable all serviced irqs, all disabled irqs are enabled only when CPAS restarts */
	cam_io_w(((~updated_sbm_mask) & (camnoc_info[camnoc_idx]->irq_sbm->sbm_enable.value)),
		soc_info->reg_map[regbase_idx].mem_base +
		camnoc_info[camnoc_idx]->irq_sbm->sbm_enable.offset);
}

static int cam_cpastop_reset_irq(uint32_t irq_status,
	struct cam_hw_info *cpas_hw, int camnoc_idx)
{
#if (defined(CONFIG_CAM_TEST_IRQ_LINE) && defined(CONFIG_CAM_TEST_IRQ_LINE_AT_PROBE))
	static int counter[CAM_CAMNOC_HW_TYPE_MAX] = {0};
	bool wait_for_irq = false;

	if (counter[camnoc_idx] == 0)  {
		CAM_INFO(CAM_CPAS, "Enabling %s test irq",
			camnoc_info[camnoc_idx]->camnoc_name);
		cam_cpastop_enable_test_irq(camnoc_idx, cpas_hw);
		wait_for_irq = true;
		init_completion(&test_irq_hw_complete[camnoc_idx]);
		counter[camnoc_idx] = 1;
	} else if (counter[camnoc_idx] == 1) {
		CAM_INFO(CAM_CPAS, "Disabling %s test irq",
			camnoc_info[camnoc_idx]->camnoc_name);
		cam_cpastop_disable_test_irq(camnoc_idx, cpas_hw);
		counter[camnoc_idx] = 2;
	}
#endif

	if (!camnoc_info[camnoc_idx]->irq_sbm->sbm_enable.enable)
		return 0;

	cam_cpas_util_reg_update(cpas_hw, camnoc_info[camnoc_idx]->reg_base,
		&camnoc_info[camnoc_idx]->irq_sbm->sbm_clear);

	if (irq_status)
		cam_cpastop_handle_camnoc_irqs(irq_status, cpas_hw, camnoc_idx);
	else
		/* poweron reset */
		cam_cpastop_enable_camnoc_irqs(cpas_hw, camnoc_idx);

#if (defined(CONFIG_CAM_TEST_IRQ_LINE) && defined(CONFIG_CAM_TEST_IRQ_LINE_AT_PROBE))
	if (wait_for_irq) {
		if (!cam_common_wait_for_completion_timeout(&test_irq_hw_complete[camnoc_idx],
			msecs_to_jiffies(2000)))
			CAM_ERR(CAM_CPAS, "[%s] CAMNOC Test IRQ line verification timed out",
				camnoc_info[camnoc_idx]->camnoc_name);
		CAM_INFO(CAM_CPAS, "[%s] IRQ test Success",
			camnoc_info[camnoc_idx]->camnoc_name);
	}
#endif

	return 0;
}

static void cam_cpastop_notify_clients(struct cam_cpas *cpas_core,
	struct cam_cpas_irq_data *irq_data, bool force_notify)
{
	int i;
	struct cam_cpas_client *cpas_client;
	bool error_handled = false;

	CAM_DBG(CAM_CPAS,
		"Notify CB : num_clients=%d, registered=%d, started=%d",
		cpas_core->num_clients, cpas_core->registered_clients,
		cpas_core->streamon_clients);

	for (i = 0; i < cpas_core->num_clients; i++) {
		if (CAM_CPAS_CLIENT_STARTED(cpas_core, i)) {
			cpas_client = cpas_core->cpas_client[i];
			if (cpas_client->data.cam_cpas_client_cb) {
				CAM_DBG(CAM_CPAS,
					"Calling client CB %d : %d",
					i, irq_data->irq_type);
				error_handled =
					cpas_client->data.cam_cpas_client_cb(
					cpas_client->data.client_handle,
					cpas_client->data.userdata,
					irq_data);
				if (error_handled && !force_notify)
					break;
			}
		}
	}
}

static void cam_cpastop_work(struct work_struct *work)
{
	struct cam_cpas_work_payload *payload;
	struct cam_hw_info *cpas_hw;
	struct cam_cpas *cpas_core;
	struct cam_hw_soc_info *soc_info;
	int i;
	enum cam_camnoc_hw_irq_type irq_type;
	struct cam_cpas_irq_data irq_data;
	int camnoc_idx;

	payload = container_of(work, struct cam_cpas_work_payload, work);
	if (!payload) {
		CAM_ERR(CAM_CPAS, "NULL payload");
		return;
	}

	camnoc_idx = payload->camnoc_idx;
	cam_common_util_thread_switch_delay_detect(
		"cam_cpas_workq", "schedule", cam_cpastop_work,
		payload->workq_scheduled_ts,
		CAM_WORKQ_SCHEDULE_TIME_THRESHOLD);

	cpas_hw = payload->hw;
	cpas_core = (struct cam_cpas *) cpas_hw->core_info;
	soc_info = &cpas_hw->soc_info;

	if (!atomic_inc_not_zero(&cpas_core->soc_access_count)) {
		CAM_ERR(CAM_CPAS, "CPAS off");
		return;
	}

	for (i = 0; i < camnoc_info[camnoc_idx]->irq_err_size; i++) {
		if ((payload->irq_status & camnoc_info[camnoc_idx]->irq_err[i].sbm_port) &&
			(camnoc_info[camnoc_idx]->irq_err[i].enable)) {
			irq_type = camnoc_info[camnoc_idx]->irq_err[i].irq_type;
			CAM_ERR_RATE_LIMIT(CAM_CPAS,
				"Error occurred, irq type=%d", irq_type);
			memset(&irq_data, 0x0, sizeof(irq_data));
			irq_data.irq_type = (enum cam_camnoc_irq_type)irq_type;

			switch (irq_type) {
			case CAM_CAMNOC_HW_IRQ_SLAVE_ERROR:
				cam_cpastop_handle_errlogger(camnoc_idx,
					cpas_core, soc_info,
					&irq_data.u.slave_err);
				break;
			case CAM_CAMNOC_HW_IRQ_IFE_UBWC_STATS_ENCODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_IFE02_UBWC_ENCODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_IFE13_UBWC_ENCODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_IPE_BPS_UBWC_ENCODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_OFE_UBWC_WRITE_ENCODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_TFE_UBWC_ENCODE_ERROR:
				cam_cpastop_handle_ubwc_enc_err(camnoc_idx,
					cpas_core, soc_info, i,
					&irq_data.u.enc_err);
				break;
			case CAM_CAMNOC_HW_IRQ_IPE1_BPS_UBWC_DECODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_IPE0_UBWC_DECODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_IPE_BPS_UBWC_DECODE_ERROR:
			case CAM_CAMNOC_HW_IRQ_OFE_UBWC_READ_DECODE_ERROR:
				cam_cpastop_handle_ubwc_dec_err(camnoc_idx,
					cpas_core, soc_info, i,
					&irq_data.u.dec_err);
				break;
			case CAM_CAMNOC_HW_IRQ_AHB_TIMEOUT:
				cam_cpastop_handle_ahb_timeout_err(camnoc_idx,
					cpas_hw, &irq_data.u.ahb_err);
				break;
			case CAM_CAMNOC_HW_IRQ_CAMNOC_TEST:
				CAM_INFO(CAM_CPAS, "TEST IRQ for %s",
					camnoc_info[camnoc_idx]->camnoc_name);
				break;
			default:
				CAM_ERR(CAM_CPAS, "Invalid IRQ type: %u", irq_type);
				break;
			}

			cam_cpastop_notify_clients(cpas_core, &irq_data, false);

			payload->irq_status &=
				~camnoc_info[camnoc_idx]->irq_err[i].sbm_port;
		}
	}
	atomic_dec(&cpas_core->soc_access_count);
	wake_up(&cpas_core->soc_access_count_wq);
	CAM_DBG(CAM_CPAS, "soc_access_count=%d\n", atomic_read(&cpas_core->soc_access_count));

	if (payload->irq_status)
		CAM_ERR(CAM_CPAS, "%s IRQ not handled irq_status=0x%x",
			camnoc_info[camnoc_idx]->camnoc_name, payload->irq_status);

	kfree(payload);
}

static irqreturn_t cam_cpastop_handle_irq(int irq_num, void *data)
{
	struct cam_cpas_soc_irq_data *soc_irq_data = data;
	struct cam_hw_info *cpas_hw = soc_irq_data->cpas_hw;
	struct cam_cpas *cpas_core = (struct cam_cpas *) cpas_hw->core_info;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;
	int regbase_idx, slave_err_irq_idx, camnoc_idx;
	struct cam_cpas_work_payload *payload;
	struct cam_cpas_irq_data irq_data;
	enum cam_camnoc_hw_type camnoc_type;

	if (!atomic_inc_not_zero(&cpas_core->soc_access_count)) {
		CAM_ERR(CAM_CPAS, "CPAS off");
		return IRQ_HANDLED;
	}

	camnoc_type = soc_irq_data->camnoc_type;
	if ((camnoc_type < 0) || (camnoc_type >= CAM_CAMNOC_HW_TYPE_MAX)) {
		CAM_ERR(CAM_CPAS, "Invalid camnoc type: %d", camnoc_type);
		goto done;
	}

	camnoc_idx = cpas_core->camnoc_info_idx[camnoc_type];
	if ((camnoc_idx < 0) || (camnoc_idx >= cpas_core->num_valid_camnoc)) {
		CAM_ERR(CAM_CPAS, "Invalid camnoc idx: %d", camnoc_idx);
		goto done;
	}

	payload = kzalloc(sizeof(struct cam_cpas_work_payload), GFP_ATOMIC);
	if (!payload)
		goto done;

	regbase_idx = cpas_core->regbase_index[camnoc_info[camnoc_idx]->reg_base];

	payload->irq_status = cam_io_r_mb(soc_info->reg_map[regbase_idx].mem_base +
		camnoc_info[camnoc_idx]->irq_sbm->sbm_status.offset);
	payload->camnoc_idx = camnoc_idx;

	CAM_DBG(CAM_CPAS, "IRQ callback of %s irq_status=0x%x",
		camnoc_info[camnoc_idx]->camnoc_name, payload->irq_status);

#if (defined(CONFIG_CAM_TEST_IRQ_LINE) && defined(CONFIG_CAM_TEST_IRQ_LINE_AT_PROBE))
	cam_cpastop_check_test_irq(camnoc_idx, cpas_hw, payload->irq_status);
#endif

	/* Clear irq */
	cam_cpastop_reset_irq(payload->irq_status, cpas_hw, camnoc_idx);

	slave_err_irq_idx = cpas_core->slave_err_irq_idx[camnoc_idx];

	/* Check for slave error irq */
	if ((cpas_core->slave_err_irq_en[camnoc_idx]) && (payload->irq_status  &
		camnoc_info[camnoc_idx]->irq_err[slave_err_irq_idx].sbm_port)) {
		struct cam_camnoc_irq_slave_err_data *slave_err = &irq_data.u.slave_err;

		irq_data.irq_type = (enum cam_camnoc_irq_type)
			camnoc_info[camnoc_idx]->irq_err[slave_err_irq_idx].irq_type;
		slave_err->errlog0_low.value = cam_io_r_mb(
			soc_info->reg_map[regbase_idx].mem_base +
			camnoc_info[camnoc_idx]->err_logger->errlog0_low);

		/* Validate address decode error */
		if (slave_err->errlog0_low.err_code == CAM_CAMNOC_ADDRESS_DECODE_ERROR) {
			/* Notify clients about potential page fault */
			if (!cpas_core->smmu_fault_handled) {
				cam_cpastop_notify_clients(cpas_core, &irq_data, true);
				CAM_ERR_RATE_LIMIT(CAM_CPAS,
					"Fault at SMMU raised as CAMNOC SLAVE IRQ, address decode error");
			}

			cpas_core->smmu_fault_handled = true;
			/* Skip bh if no other irq is set */
			payload->irq_status &=
				~camnoc_info[camnoc_idx]->irq_err[slave_err_irq_idx].sbm_port;
			if (!payload->irq_status) {
				kfree(payload);
				goto done;
			}
		}
	}

	payload->hw = cpas_hw;
	INIT_WORK((struct work_struct *)&payload->work, cam_cpastop_work);

	payload->workq_scheduled_ts = ktime_get();
	queue_work(cpas_core->work_queue, &payload->work);

done:
	atomic_dec(&cpas_core->soc_access_count);
	wake_up(&cpas_core->soc_access_count_wq);

	return IRQ_HANDLED;
}

static int cam_cpastop_print_poweron_settings(struct cam_hw_info *cpas_hw)
{
	int i, j;
	enum cam_cpas_reg_base reg_base;
	struct cam_cpas *cpas_core = cpas_hw->core_info;

	for (i = 0; i < cpas_core->num_valid_camnoc; i++) {
		CAM_INFO(CAM_CPAS, "QOS settings for %s :",
			camnoc_info[i]->camnoc_name);
		for (j = 0; j < camnoc_info[i]->specific_size; j++) {
			if (camnoc_info[i]->specific[j].enable) {
				CAM_INFO(CAM_CPAS,
					"Reading QoS settings port: %d port name: %s",
					camnoc_info[i]->specific[j].port_type,
					camnoc_info[i]->specific[j].port_name);
				reg_base = camnoc_info[i]->reg_base;
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].priority_lut_low);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].priority_lut_high);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].urgency);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].danger_lut);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].safe_lut);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].ubwc_ctl);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].flag_out_set0_low);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].dynattr_mainctl);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].qosgen_mainctl);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].qosgen_shaping_low);
				cam_cpas_util_reg_read(cpas_hw, reg_base,
					&camnoc_info[i]->specific[j].qosgen_shaping_high);
			}
		}
	}

	return 0;
}

static int cam_cpastop_poweron(struct cam_hw_info *cpas_hw)
{
	int i, j;
	struct cam_cpas_hw_errata_wa_list *errata_wa_list;
	struct cam_cpas_hw_errata_wa *errata_wa;
	struct cam_cpas *cpas_core = cpas_hw->core_info;
	bool errata_enabled = false;

	for (i = 0; i < cpas_core->num_valid_camnoc; i++)
		cam_cpastop_reset_irq(0x0, cpas_hw, i);

	for (i = 0; i < cpas_core->num_valid_camnoc; i++) {
		CAM_DBG(CAM_CPAS, "QOS settings for %s :",
			camnoc_info[i]->camnoc_name);
		for (j = 0; j < camnoc_info[i]->specific_size; j++) {
			if (camnoc_info[i]->specific[j].enable) {
				CAM_DBG(CAM_CPAS,
					"Updating QoS settings port: %d prot name: %s",
					camnoc_info[i]->specific[j].port_type,
					camnoc_info[i]->specific[j].port_name);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].priority_lut_low);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].priority_lut_high);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].urgency);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].danger_lut);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].safe_lut);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].ubwc_ctl);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].flag_out_set0_low);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].dynattr_mainctl);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].qosgen_mainctl);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].qosgen_shaping_low);
				cam_cpas_util_reg_update(cpas_hw, camnoc_info[i]->reg_base,
					&camnoc_info[i]->specific[j].qosgen_shaping_high);
			}
		}

		if (!errata_enabled) {
			errata_wa_list = camnoc_info[i]->errata_wa_list;
			if (errata_wa_list) {
				errata_wa = &errata_wa_list->tcsr_camera_hf_sf_ares_glitch;
				if (errata_wa->enable) {
					cam_cpastop_scm_write(errata_wa);
					errata_enabled = true;
				}
			}
		}
	}

	return 0;
}

static int cam_cpastop_poweroff(struct cam_hw_info *cpas_hw)
{
	struct cam_cpas *cpas_core = (struct cam_cpas *) cpas_hw->core_info;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;
	int camnoc_index, rc = 0;
	struct cam_cpas_hw_errata_wa_list *errata_wa_list;
	struct cam_cpas_hw_errata_wa *errata_wa;

	errata_wa_list = camnoc_info[0]->errata_wa_list;
	if (!errata_wa_list)
		return 0;

	if (errata_wa_list->camnoc_flush_slave_pending_trans.enable) {
		errata_wa = &errata_wa_list->camnoc_flush_slave_pending_trans;
		camnoc_index = cpas_core->regbase_index[camnoc_info[0]->reg_base];

		rc = cam_io_poll_value_wmask(
			soc_info->reg_map[camnoc_index].mem_base +
			errata_wa->data.reg_info.offset,
			errata_wa->data.reg_info.value,
			errata_wa->data.reg_info.mask,
			CAM_CPAS_POLL_RETRY_CNT,
			CAM_CPAS_POLL_MIN_USECS, CAM_CPAS_POLL_MAX_USECS);
		if (rc) {
			CAM_DBG(CAM_CPAS,
				"camnoc flush slave pending trans failed");
			/* Do not return error, passthrough */
			rc = 0;
		}
	}

	return rc;
}

static int cam_cpastop_qchannel_handshake(struct cam_hw_info *cpas_hw,
	bool power_on, bool force_on)
{
	struct cam_cpas *cpas_core = (struct cam_cpas *) cpas_hw->core_info;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;
	int32_t reg_indx = cpas_core->regbase_index[CAM_CPAS_REG_CPASTOP];
	uint32_t mask = 0;
	uint32_t wait_data, qchannel_status, qdeny;
	int rc = 0, ret = 0, i;
	struct cam_cpas_private_soc *soc_private =
		(struct cam_cpas_private_soc *) cpas_hw->soc_info.soc_private;
	struct cam_cpas_hw_errata_wa_list *errata_wa_list;
	bool icp_clk_enabled = false;
	struct cam_cpas_camnoc_qchannel *qchannel_info;

	if (reg_indx == -1)
		return -EINVAL;

	for (i = 0; i < cpas_info->num_qchannel; i++) {
		qchannel_info = cpas_info->qchannel_info[i];

		if (!icp_clk_enabled) {
			errata_wa_list = camnoc_info[i]->errata_wa_list;
			if (errata_wa_list && errata_wa_list->enable_icp_clk_for_qchannel.enable) {
				CAM_DBG(CAM_CPAS, "Enabling ICP clk for qchannel handshake");

				if (soc_private->icp_clk_index == -1) {
					CAM_ERR(CAM_CPAS,
						"ICP clock not added as optional clk, qchannel handshake will fail");
				} else {
					rc = cam_soc_util_clk_enable(soc_info,
						CAM_CLK_SW_CLIENT_IDX, true,
						soc_private->icp_clk_index, -1);
					if (rc)
						CAM_ERR(CAM_CPAS,
							"Error enable icp clk failed rc=%d", rc);
					else
						icp_clk_enabled = true;
				}
			}
		}

		if (power_on) {
			if (force_on) {
				cam_io_w_mb(0x1,
					soc_info->reg_map[reg_indx].mem_base +
					qchannel_info->qchannel_ctrl);
				CAM_DBG(CAM_CPAS, "Force qchannel on for %s",
						camnoc_info[i]->camnoc_name);
			}
			/* wait for QACCEPTN in QCHANNEL status*/
			mask = BIT(0);
			wait_data = 1;
		} else {

			/* Clear the quiecience request in QCHANNEL ctrl*/
			cam_io_w_mb(0, soc_info->reg_map[reg_indx].mem_base +
				qchannel_info->qchannel_ctrl);
			/* wait for QACCEPTN and QDENY in QCHANNEL status*/
			mask = BIT(1) | BIT(0);
			wait_data = 0;
		}

		rc = cam_io_poll_value_wmask(
			soc_info->reg_map[reg_indx].mem_base +
			qchannel_info->qchannel_status,
			wait_data, mask, CAM_CPAS_POLL_QH_RETRY_CNT,
			CAM_CPAS_POLL_MIN_USECS, CAM_CPAS_POLL_MAX_USECS);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"CPAS_%s %s idle sequence failed, qstat 0x%x",
				power_on ? "START" : "STOP", camnoc_info[i]->camnoc_name,
			cam_io_r(soc_info->reg_map[reg_indx].mem_base +
				qchannel_info->qchannel_status));
			ret = rc;
			/* Do not return error, passthrough */
		}

		/* check if deny bit is set */
		qchannel_status = cam_io_r_mb(soc_info->reg_map[reg_indx].mem_base +
			qchannel_info->qchannel_status);
		CAM_DBG(CAM_CPAS,
			"CPAS_%s %s : qchannel status 0x%x", power_on ? "START" : "STOP",
			camnoc_info[i]->camnoc_name, qchannel_status);

		qdeny = (qchannel_status & BIT(1));
		if (!power_on && qdeny)
			ret = -EBUSY;
	}

	if (icp_clk_enabled) {
		rc = cam_soc_util_clk_disable(soc_info, CAM_CLK_SW_CLIENT_IDX, true,
			soc_private->icp_clk_index);
		if (rc)
			CAM_ERR(CAM_CPAS, "Error disable icp clk failed rc=%d", rc);
	}

	return ret;
}

static int cam_cpastop_set_up_camnoc_info(struct cam_cpas *cpas_core,
	struct cam_camnoc_info **alloc_camnoc)
{
	int i, j, camnoc_cnt = 0;

	for (i = 0; i < CAM_CAMNOC_HW_TYPE_MAX; i++) {
		if (alloc_camnoc[i]) {
			alloc_camnoc[i]->camnoc_name = cam_cpastop_get_camnoc_name(i);
			camnoc_info[camnoc_cnt] = alloc_camnoc[i];
			cpas_core->camnoc_info[camnoc_cnt] = alloc_camnoc[i];
			cpas_core->camnoc_info_idx[i] = camnoc_cnt;

			switch (i) {
			case CAM_CAMNOC_HW_COMBINED:
				camnoc_info[i]->reg_base = CAM_CPAS_REG_CAMNOC;
				break;
			case CAM_CAMNOC_HW_RT:
				camnoc_info[i]->reg_base = CAM_CPAS_REG_CAMNOC_RT;
				break;
			case CAM_CAMNOC_HW_NRT:
				camnoc_info[i]->reg_base = CAM_CPAS_REG_CAMNOC_NRT;
				break;
			default:
				CAM_ERR(CAM_CPAS, "Invalid camnoc type %u", i);
				return -EINVAL;
			}

			camnoc_cnt++;

			if (cpas_core->regbase_index[camnoc_info[i]->reg_base] == -1) {
				CAM_ERR(CAM_CPAS, "Regbase not set up for %s",
					camnoc_info[i]->camnoc_name);
				return -EINVAL;
			}
		} else {
			cpas_core->camnoc_info_idx[i] = -1;
		}
	}

	if (camnoc_cnt == 0) {
		CAM_ERR(CAM_CPAS, "No available camnoc header for binding");
		return -EINVAL;
	}

	if (cpas_info->num_qchannel && cpas_info->num_qchannel != camnoc_cnt) {
		CAM_ERR(CAM_CPAS, "Invalid number of qchannel: %u number of camnoc: %u",
			cpas_info->num_qchannel, camnoc_cnt);
		return -EINVAL;
	}

	cpas_core->num_valid_camnoc = camnoc_cnt;

	if (cpas_core->camnoc_info_idx[CAM_CAMNOC_HW_COMBINED] >= 0)
		cpas_core->camnoc_rt_idx = cpas_core->camnoc_info_idx[CAM_CAMNOC_HW_COMBINED];
	else if (cpas_core->camnoc_info_idx[CAM_CAMNOC_HW_RT] >= 0)
		cpas_core->camnoc_rt_idx = cpas_core->camnoc_info_idx[CAM_CAMNOC_HW_RT];
	else {
		cpas_core->camnoc_rt_idx = -1;
		CAM_ERR(CAM_CPAS, "No CAMNOC RT idx found");
		return -EINVAL;
	}

	/* Check if slave error irq is enabled */
	for (i = 0; i < cpas_core->num_valid_camnoc; i++) {
		for (j = 0; j < camnoc_info[i]->irq_err_size; j++) {
			if (camnoc_info[i]->irq_err[j].irq_type ==
				CAM_CAMNOC_HW_IRQ_SLAVE_ERROR) {
				if (camnoc_info[i]->irq_err[j].enable) {
					cpas_core->slave_err_irq_en[i] = true;
					cpas_core->slave_err_irq_idx[i] = j;
					break;
				}
			}
		}
	}

	return 0;
}

static int cam_cpastop_get_hw_capability(struct cam_hw_info *cpas_hw)
{
	int i, reg_idx;
	struct cam_cpas *cpas_core = cpas_hw->core_info;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;
	struct cam_cpas_hw_cap_info *hw_caps_info;
	struct cam_cpas_hw_caps *hw_caps = &cpas_core->hw_caps;

	hw_caps_info = &cpas_info->hw_caps_info;
	reg_idx = cpas_core->regbase_index[CAM_CPAS_REG_CPASTOP];

	/* At least one hw caps register must be present */
	if (!hw_caps_info->num_caps_registers ||
		hw_caps_info->num_caps_registers > CAM_CPAS_MAX_CAPS_REGS) {
		CAM_ERR(CAM_CPAS,
			"Invalid number of populated caps registers: %u",
			hw_caps_info->num_caps_registers);
		return -EINVAL;
	}

	hw_caps->num_capability_reg = hw_caps_info->num_caps_registers;
	for (i = 0; i < hw_caps_info->num_caps_registers; i++) {
		hw_caps->camera_capability[i] = cam_io_r_mb(soc_info->reg_map[reg_idx].mem_base +
			hw_caps_info->hw_caps_offsets[i]);
		CAM_DBG(CAM_CPAS, "camera_caps_%d = 0x%x", i, hw_caps->camera_capability[i]);
	}

	return 0;
}

static int cam_cpastop_init_hw_version(struct cam_hw_info *cpas_hw,
	struct cam_cpas_hw_caps *hw_caps)
{
	int rc = 0;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;
	struct cam_cpas *cpas_core = (struct cam_cpas *) cpas_hw->core_info;
	struct cam_cpas_cesta_info *cesta_info = NULL;
	struct cam_camnoc_info *alloc_camnoc_info[CAM_CAMNOC_HW_TYPE_MAX] = {0};

	CAM_DBG(CAM_CPAS,
		"hw_version=0x%x Camera Version %d.%d.%d, cpas version %d.%d.%d",
		soc_info->hw_version,
		hw_caps->camera_version.major,
		hw_caps->camera_version.minor,
		hw_caps->camera_version.incr,
		hw_caps->cpas_version.major,
		hw_caps->cpas_version.minor,
		hw_caps->cpas_version.incr);

	switch (soc_info->hw_version) {
	case CAM_CPAS_TITAN_170_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam170_cpas100_camnoc_info;
		cpas_info = &cam170_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_170_V110:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam170_cpas110_camnoc_info;
		cpas_info = &cam170_cpas110_cpas_info;
		break;
	case CAM_CPAS_TITAN_170_V200:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam170_cpas200_camnoc_info;
		cpas_info = &cam170_cpas200_cpas_info;
		break;
	case CAM_CPAS_TITAN_175_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam175_cpas100_camnoc_info;
		cpas_info = &cam175_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_175_V101:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam175_cpas101_camnoc_info;
		cpas_info = &cam175_cpas101_cpas_info;
		break;
	case CAM_CPAS_TITAN_175_V120:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam175_cpas120_camnoc_info;
		cpas_info = &cam175_cpas120_cpas_info;
		break;
	case CAM_CPAS_TITAN_175_V130:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam175_cpas130_camnoc_info;
		cpas_info = &cam175_cpas130_cpas_info;
		break;
	case CAM_CPAS_TITAN_150_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam150_cpas100_camnoc_info;
		cpas_info = &cam150_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_480_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam480_cpas100_camnoc_info;
		cpas_info = &cam480_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_580_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam580_cpas100_camnoc_info;
		cpas_info = &cam580_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_540_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam540_cpas100_camnoc_info;
		cpas_info = &cam540_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_520_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam520_cpas100_camnoc_info;
		cpas_info = &cam520_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_545_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam545_cpas100_camnoc_info;
		cpas_info = &cam545_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_570_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam570_cpas100_camnoc_info;
		cpas_info = &cam570_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_570_V200:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam570_cpas200_camnoc_info;
		cpas_info = &cam570_cpas200_cpas_info;
		break;
	case CAM_CPAS_TITAN_680_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam680_cpas100_camnoc_info;
		cpas_info = &cam680_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_680_V110:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam680_cpas110_camnoc_info;
		cpas_info = &cam680_cpas110_cpas_info;
		break;
	case CAM_CPAS_TITAN_165_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam165_cpas100_camnoc_info;
		cpas_info = &cam165_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_780_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam780_cpas100_camnoc_info;
		cpas_info = &cam780_cpas100_cpas_info;
		break;
	case CAM_CPAS_TITAN_640_V200:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam640_cpas200_camnoc_info;
		cpas_info = &cam640_cpas200_cpas_info;
		break;
	case CAM_CPAS_TITAN_880_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam880_cpas100_camnoc_info;
		cpas_info = &cam880_cpas100_cpas_info;
		cesta_info = &cam_v880_cesta_info;
		break;
	case CAM_CPAS_TITAN_980_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_RT] = &cam980_cpas100_camnoc_info_rt;
		alloc_camnoc_info[CAM_CAMNOC_HW_NRT] = &cam980_cpas100_camnoc_info_nrt;
		cpas_info = &cam980_cpas100_cpas_info;
		cesta_info = &cam_v980_cesta_info;
		break;
	case CAM_CPAS_TITAN_860_V100:
		alloc_camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam860_cpas100_camnoc_info;
		cpas_info = &cam860_cpas100_cpas_info;
		cesta_info = &cam_v860_cesta_info;
		break;
	default:
		CAM_ERR(CAM_CPAS, "Camera Version not supported %d.%d.%d",
			hw_caps->camera_version.major,
			hw_caps->camera_version.minor,
			hw_caps->camera_version.incr);
		return -EINVAL;
	}

	cpas_core->cesta_info = cesta_info;

	rc = cam_cpastop_set_up_camnoc_info(cpas_core, alloc_camnoc_info);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Failed to set up camnoc info rc=%d", rc);
		return rc;
	}

	rc = cam_cpastop_get_hw_capability(cpas_hw);
	if (rc) {
		CAM_ERR(CAM_CPAS, "Failed to get titan hw capability rc=%d", rc);
		return rc;
	}

	return rc;
}

static int cam_cpastop_setup_qos_settings(struct cam_hw_info *cpas_hw,
	uint32_t selection_mask)
{
	int rc = 0;
	struct cam_hw_soc_info *soc_info = &cpas_hw->soc_info;

	CAM_DBG(CAM_CPAS,
		"QoS selection : hw_version=0x%x selection_mask 0x%x",
		soc_info->hw_version,
		selection_mask);

	switch (soc_info->hw_version) {
	case CAM_CPAS_TITAN_480_V100:
		if (selection_mask & CAM_CPAS_QOS_CUSTOM_SETTINGS_MASK)
			camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam480_custom_camnoc_info;
		else if (selection_mask & CAM_CPAS_QOS_DEFAULT_SETTINGS_MASK)
			camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam480_cpas100_camnoc_info;
		else
			CAM_ERR(CAM_CPAS, "Invalid selection mask 0x%x",
				selection_mask);
		break;
	case CAM_CPAS_TITAN_580_V100:
		if (selection_mask & CAM_CPAS_QOS_CUSTOM_SETTINGS_MASK)
			camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam580_custom_camnoc_info;
		else if (selection_mask & CAM_CPAS_QOS_DEFAULT_SETTINGS_MASK)
			camnoc_info[CAM_CAMNOC_HW_COMBINED] = &cam580_cpas100_camnoc_info;
		else
			CAM_ERR(CAM_CPAS,
				"Invalid selection mask 0x%x for hw 0x%x",
				selection_mask, soc_info->hw_version);
		break;
	default:
		CAM_WARN(CAM_CPAS, "QoS selection not supported for 0x%x",
			soc_info->hw_version);
		rc = -EINVAL;
		break;
	}

	return rc;
}

int cam_cpastop_get_internal_ops(struct cam_cpas_internal_ops *internal_ops)
{
	if (!internal_ops) {
		CAM_ERR(CAM_CPAS, "invalid NULL param");
		return -EINVAL;
	}

	internal_ops->get_hw_info = cam_cpastop_get_hw_info;
	internal_ops->init_hw_version = cam_cpastop_init_hw_version;
	internal_ops->handle_irq = cam_cpastop_handle_irq;
	internal_ops->setup_regbase = cam_cpastop_setup_regbase_indices;
	internal_ops->power_on = cam_cpastop_poweron;
	internal_ops->power_off = cam_cpastop_poweroff;
	internal_ops->setup_qos_settings = cam_cpastop_setup_qos_settings;
	internal_ops->print_poweron_settings =
		cam_cpastop_print_poweron_settings;
	internal_ops->qchannel_handshake = cam_cpastop_qchannel_handshake;

	return 0;
}
