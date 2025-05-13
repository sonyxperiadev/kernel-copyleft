// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "tpg_hw.h"

#define BYTES_PER_REGISTER           4
#define NUM_REGISTER_PER_LINE        4
#define REG_OFFSET(__start, __i)    ((__start) + ((__i) * BYTES_PER_REGISTER))
#define CAM_TPG_HW_WAIT_TIMEOUT     msecs_to_jiffies(100)
#define MAX_ACTIVE_QUEUE_DEPTH      2
#define MAX_WAITING_QUEUE_DEPTH     32
#define TIMEOUT_MULTIPLIER_PRESIL   5
#define TIMEOUT_MULTIPLIER          1

static int cam_io_tpg_dump(void __iomem *base_addr,
	uint32_t start_offset, int size)
{
	char          line_str[128];
	char         *p_str;
	int           i;
	uint32_t      data;

	CAM_DBG(CAM_TPG, "addr=%pK offset=0x%x size=%d",
		base_addr, start_offset, size);

	if (!base_addr || (size <= 0))
		return -EINVAL;

	line_str[0] = '\0';
	p_str = line_str;
	for (i = 0; i < size; i++) {
		if (i % NUM_REGISTER_PER_LINE == 0) {
			snprintf(p_str, 12, "0x%08x: ",
				REG_OFFSET(start_offset, i));
			p_str += 11;
		}
		data = cam_io_r(base_addr + REG_OFFSET(start_offset, i));
		snprintf(p_str, 9, "%08x ", data);
		p_str += 8;
		if ((i + 1) % NUM_REGISTER_PER_LINE == 0) {
			CAM_DBG(CAM_TPG, "%s", line_str);
			line_str[0] = '\0';
			p_str = line_str;
		}
	}
	if (line_str[0] != '\0')
		CAM_ERR(CAM_TPG, "%s", line_str);

	return 0;
}

int32_t cam_tpg_mem_dmp(struct cam_hw_soc_info *soc_info)
{
	int32_t rc = 0;
	resource_size_t size = 0;
	void __iomem *addr = NULL;

	if (!soc_info) {
		rc = -EINVAL;
		CAM_ERR(CAM_TPG, "invalid input %d", rc);
		return rc;
	}
	addr = soc_info->reg_map[0].mem_base;
	size = resource_size(soc_info->mem_block[0]);
	rc = cam_io_tpg_dump(addr, 0, (size >> 2));
	if (rc < 0) {
		CAM_ERR(CAM_TPG, "generating dump failed %d", rc);
	}
	return rc;
}


#define __TPG_DEBUG_DUMP__
#ifdef __TPG_DEBUG_DUMP__
static const char * const tpg_phy_type_strings[] = {
	"TPG_PHY_TYPE_INVALID",
	"TPG_PHY_TYPE_DPHY",
	"TPG_PHY_TYPE_CPHY",
	"TPG_PHY_TYPE_MAX"
};

static const char * const tpg_interleaving_format_string[] = {
	"TPG_INTERLEAVING_FORMAT_INVALID",
	"TPG_INTERLEAVING_FORMAT_FRAME",
	"TPG_INTERLEAVING_FORMAT_LINE",
	"TPG_INTERLEAVING_FORMAT_SHDR",
	"TPG_INTERLEAVING_FORMAT_SparsePD",
	"TPG_INTERLEAVING_FORMAT_MAX"
};

static const char * const tpg_shutter_type_strings[] = {
	"TPG_SHUTTER_TYPE_INVALID",
	"TPG_SHUTTER_TYPE_ROLLING",
	"TPG_SHUTTER_TYPE_GLOBAL",
	"TPG_SHUTTER_TYPE_MAX"
};

static const char *const tpg_pattern_type_strings[] = {
	"TPG_PATTERN_INVALID",
	"TPG_PATTERN_REAL_IMAGE",
	"TPG_PATTERN_RANDOM_PIXL",
	"TPG_PATTERN_RANDOM_INCREMENTING_PIXEL",
	"TPG_PATTERN_COLOR_BAR",
	"TPG_PATTERN_ALTERNATING_55_AA",
	"TPG_PATTERN_ALTERNATING_USER_DEFINED",
	"TPG_PATTERN_MAX"
};

static const char *const tpg_color_bar_mode_strings[] = {
	"TPG_COLOR_BAR_MODE_INVALID",
	"TPG_COLOR_BAR_MODE_NORMAL",
	"TPG_COLOR_BAR_MODE_SPLIT",
	"TPG_COLOR_BAR_MODE_ROTATING",
	"TPG_COLOR_BAR_MODE_MAX"
};

static const char *const tpg_stream_type_strings[] = {
	"TPG_STREAM_TYPE_INVALID",
	"TPG_STREAM_TYPE_IMAGE",
	"TPG_STREAM_TYPE_PDAF",
	"TPG_STREAM_TYPE_META",
	"TPG_STREAM_TYPE_MAX"
};

static const char *const tpg_image_format_type_strings[] = {
	"TPG_IMAGE_FORMAT_INVALID",
	"TPG_IMAGE_FORMAT_BAYER",
	"TPG_IMAGE_FORMAT_QCFA",
	"TPG_IMAGE_FORMAT_YUV",
	"TPG_IMAGE_FORMAT_JPEG",
	"TPG_IMAGE_FORMAT_MAX"
};
#endif

int dump_global_configs(int idx,
		struct tpg_global_config_t *global)
{
#ifdef __TPG_DEBUG_DUMP__
	CAM_DBG(CAM_TPG, "TPG[%d] phy_type            : %s",
			idx,
			tpg_phy_type_strings[global->phy_type]);
	CAM_DBG(CAM_TPG, "TPG[%d] lane_count          : %d",
			idx,
			global->lane_count);
	CAM_DBG(CAM_TPG, "TPG[%d] interleaving_format : %s",
			idx,
			tpg_interleaving_format_string[global->interleaving_format]);
	CAM_DBG(CAM_TPG, "TPG[%d] phy_mode            : %d",
			idx,
			global->phy_mode);
	CAM_DBG(CAM_TPG, "TPG[%d] shutter_type        : %s",
			idx,
			tpg_shutter_type_strings[global->shutter_type]);
	CAM_DBG(CAM_TPG, "TPG[%d] skip pattern        : 0x%x",
			idx,
			global->skip_pattern);
	CAM_DBG(CAM_TPG, "TPG[%d] tpg clock           : %d",
			idx,
			global->tpg_clock);
#endif
	return 0;
}

int dump_stream_configs(int hw_idx,
		int stream_idx,
		struct tpg_stream_config_t *stream)
{
#ifdef __TPG_DEBUG_DUMP__
	CAM_DBG(CAM_TPG, "TPG[%d][%d] pattern_type    : %s",
			hw_idx,
			stream_idx,
			tpg_pattern_type_strings[stream->pattern_type]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] cb_mode         : %s",
			hw_idx,
			stream_idx,
			tpg_color_bar_mode_strings[stream->cb_mode]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] frame_count     : %d",
			hw_idx,
			stream_idx,
			stream->frame_count);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] stream_type     : %s",
			hw_idx,
			stream_idx,
			tpg_stream_type_strings[stream->stream_type]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] left            : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.left);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] top             : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.top);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] width           : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.width);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] height          : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.height);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] pixel_depth     : %d",
			hw_idx,
			stream_idx,
			stream->pixel_depth);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] cfa_arrangement : %d",
			hw_idx,
			stream_idx,
			stream->cfa_arrangement);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] output_format   : %s",
			hw_idx,
			stream_idx,
		tpg_image_format_type_strings[stream->output_format]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] vc              : 0x%x",
			hw_idx,
			stream_idx,
			stream->vc);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] dt              : 0x%x",
			hw_idx,
			stream_idx,
			stream->dt);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] hbi             : %d",
			hw_idx,
			stream_idx,
			stream->hbi);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] vbi             : %d",
			hw_idx,
			stream_idx,
			stream->vbi);
#endif
	return 0;
}

int dump_stream_configs_v3(int hw_idx,
		int stream_idx,
		struct tpg_stream_config_v3_t *stream)
{
#ifdef __TPG_DEBUG_DUMP__
	CAM_DBG(CAM_TPG, "TPG[%d][%d] pattern_type    : %s",
			hw_idx,
			stream_idx,
			tpg_pattern_type_strings[stream->pattern_type]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] cb_mode         : %s",
			hw_idx,
			stream_idx,
			tpg_color_bar_mode_strings[stream->cb_mode]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] frame_count     : %d",
			hw_idx,
			stream_idx,
			stream->frame_count);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] stream_type     : %s",
			hw_idx,
			stream_idx,
			tpg_stream_type_strings[stream->stream_type]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] left            : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.left);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] top             : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.top);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] width           : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.width);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] height          : %d",
			hw_idx,
			stream_idx,
			stream->stream_dimension.height);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] pixel_depth     : %d",
			hw_idx,
			stream_idx,
			stream->pixel_depth);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] cfa_arrangement : %d",
			hw_idx,
			stream_idx,
			stream->cfa_arrangement);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] output_format   : %s",
			hw_idx,
			stream_idx,
		tpg_image_format_type_strings[stream->output_format]);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] vc              : 0x%x",
			hw_idx,
			stream_idx,
			stream->vc);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] dt              : 0x%x",
			hw_idx,
			stream_idx,
			stream->dt);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] hbi             : %d",
			hw_idx,
			stream_idx,
			stream->hbi);
	CAM_DBG(CAM_TPG, "TPG[%d][%d] vbi             : %d",
			hw_idx,
			stream_idx,
			stream->vbi);
#endif
	return 0;
}
static int tpg_hw_release_vc_slots_locked(struct tpg_hw *hw,
		struct tpg_hw_request *req)
{
	struct list_head *pos = NULL, *pos_next = NULL;
	struct tpg_hw_stream *entry;
	struct tpg_hw_stream_v3 *entry_v3;
	int i = 0;

	if (!hw || !hw->hw_info || !req) {
		CAM_ERR(CAM_TPG, "Invalid Params");
		return -EINVAL;
	}

	CAM_DBG(CAM_TPG, "TPG[%d]  req[%lld] Freeing all the streams",
			hw->hw_idx, req->request_id);

	for (i = 0; i < hw->hw_info->max_vc_channels; i++) {
		req->vc_slots[i].slot_id      =  i;
		req->vc_slots[i].vc           = -1;
		req->vc_slots[i].stream_count =  0;
		if (hw->stream_version == 1) {
			list_for_each_safe(pos, pos_next, &req->vc_slots[i].head) {
				entry = list_entry(pos, struct tpg_hw_stream, list);
				list_del(pos);
				kfree(entry);
			}
		} else if (hw->stream_version == 3) {
			list_for_each_safe(pos, pos_next, &req->vc_slots[i].head) {
				entry_v3 = list_entry(pos, struct tpg_hw_stream_v3, list);
				list_del(pos);
				kfree(entry_v3);
			}
		}
		INIT_LIST_HEAD(&(req->vc_slots[i].head));
	}

	hw->vc_count = 0;
	kfree(req->vc_slots);
	kfree(req);

	return 0;
}

static int tpg_hw_free_waiting_requests_locked(struct tpg_hw *hw)
{
	struct list_head *pos = NULL, *pos_next = NULL;
	struct tpg_hw_request *req = NULL;

	if (!hw) {
		CAM_ERR(CAM_TPG, "Invalid Params");
		return -EINVAL;
	}

	/* free up the pending requests*/
	CAM_DBG(CAM_TPG, "TPG[%d]  freeing all waiting requests",
			hw->hw_idx);
	list_for_each_safe(pos, pos_next, &hw->waiting_request_q) {
		req = list_entry(pos, struct tpg_hw_request, list);
		CAM_DBG(CAM_TPG, "TPG[%d] freeing request[%lld] ",
				hw->hw_idx, req->request_id);
		list_del(pos);
		tpg_hw_release_vc_slots_locked(hw, req);
	}
	return 0;
}

static int tpg_hw_free_active_requests_locked(struct tpg_hw *hw)
{
	struct list_head *pos = NULL, *pos_next = NULL;
	struct tpg_hw_request *req = NULL;

	if (!hw) {
		CAM_ERR(CAM_TPG, "Invalid Params");
		return -EINVAL;
	}

	/* free up the active requests*/
	CAM_DBG(CAM_TPG, "TPG[%d]  freeing all active requests",
			hw->hw_idx);
	list_for_each_safe(pos, pos_next, &hw->active_request_q) {
		req = list_entry(pos, struct tpg_hw_request, list);
		CAM_DBG(CAM_TPG, "TPG[%d] freeing request[%lld] ",
				hw->hw_idx, req->request_id);
		list_del(pos);
		tpg_hw_release_vc_slots_locked(hw, req);
	}
	return 0;
}

static int tpg_hw_soc_disable(struct tpg_hw *hw)
{
	int rc = 0;
	unsigned long flags;

	if (!hw || !hw->soc_info) {
		CAM_ERR(CAM_TPG, "Error Invalid params");
		return -EINVAL;
	}

	rc = cam_soc_util_disable_platform_resource(hw->soc_info, CAM_CLK_SW_CLIENT_IDX, true,
		true);

	if (rc) {
		CAM_ERR(CAM_TPG, "TPG[%d] Disable platform failed %d",
			hw->hw_idx, rc);
		return rc;
	}
	if ((rc = cam_cpas_stop(hw->cpas_handle))) {
		CAM_ERR(CAM_TPG, "TPG[%d] CPAS stop failed",
			hw->hw_idx);
		return rc;
	} else {
		spin_lock_irqsave(&hw->hw_state_lock, flags);
		hw->state = TPG_HW_STATE_HW_DISABLED;
		spin_unlock_irqrestore(&hw->hw_state_lock, flags);
	}

	return rc;
}

static int tpg_hw_soc_enable(
	struct tpg_hw *hw,
	enum cam_vote_level clk_level,
	uint32_t enable_irq)
{
	int rc = 0;
	struct cam_ahb_vote ahb_vote;
	struct cam_axi_vote axi_vote = {0};
	unsigned long flags;

	ahb_vote.type = CAM_VOTE_ABSOLUTE;
	ahb_vote.vote.level = CAM_SVS_VOTE;
	axi_vote.num_paths = 1;
	axi_vote.axi_path[0].path_data_type = CAM_AXI_PATH_DATA_ALL;
	axi_vote.axi_path[0].transac_type = CAM_AXI_TRANSACTION_WRITE;

	axi_vote.axi_path[0].camnoc_bw = CAM_CPAS_DEFAULT_AXI_BW;
	axi_vote.axi_path[0].mnoc_ab_bw = CAM_CPAS_DEFAULT_AXI_BW;
	axi_vote.axi_path[0].mnoc_ib_bw = CAM_CPAS_DEFAULT_AXI_BW;

	CAM_DBG(CAM_TPG, "TPG[%d] camnoc_bw:%lld mnoc_ab_bw:%lld mnoc_ib_bw:%lld ",
		hw->hw_idx,
		axi_vote.axi_path[0].camnoc_bw,
		axi_vote.axi_path[0].mnoc_ab_bw,
		axi_vote.axi_path[0].mnoc_ib_bw);

	rc = cam_cpas_start(hw->cpas_handle, &ahb_vote, &axi_vote);
	if (rc) {
		CAM_ERR(CAM_TPG, "TPG[%d] CPAS start failed",
			hw->hw_idx);
		rc = -EFAULT;
		goto end;
	}

	rc = cam_soc_util_enable_platform_resource(hw->soc_info, CAM_CLK_SW_CLIENT_IDX, true,
		clk_level, enable_irq);

	if (rc) {
		CAM_ERR(CAM_TPG, "TPG[%d] enable platform failed",
			hw->hw_idx);
		goto stop_cpas;
	}

	spin_lock_irqsave(&hw->hw_state_lock, flags);
	/* might need to wait if in busy state */
	hw->state = TPG_HW_STATE_READY;
	spin_unlock_irqrestore(&hw->hw_state_lock, flags);

	return rc;
stop_cpas:
	cam_cpas_stop(hw->cpas_handle);
end:
	return rc;
}

static int tpg_hw_apply_settings_to_hw_locked(struct tpg_hw *hw,
	struct tpg_hw_request *req)
{
	int i = 0;
	uint32_t stream_idx = 0;
	int num_vcs = 0;
	struct global_config_args globalargs = {0};
	if (!hw ||
		!hw->hw_info ||
		!hw->hw_info->ops ||
		!hw->hw_info->ops->process_cmd ||
		!hw->soc_info ||
		!req) {
		CAM_ERR(CAM_TPG, "Invalid argument");
		return -EINVAL;
	}
	/* If nop then skip applying and return success*/
	if (req->request_type == TPG_HW_REQ_TYPE_NOP)
		goto end;

	dump_global_configs(hw->hw_idx, &req->global_config);
	if (hw->stream_version == 1) {
		for (i = 0; i < hw->hw_info->max_vc_channels; i++) {
			int dt_slot = 0;
			struct vc_config_args vc_config = {0};
			struct list_head *pos = NULL, *pos_next = NULL;
			struct tpg_hw_stream *entry = NULL, *vc_stream_entry = NULL;

			if (req->vc_slots[i].vc == -1)
				break;
			num_vcs++;
			vc_config.vc_slot = i;
			vc_config.num_dts = req->vc_slots[i].stream_count;
			vc_stream_entry = list_first_entry(&req->vc_slots[i].head,
				struct tpg_hw_stream, list);
			vc_config.stream  = &vc_stream_entry->stream;
			hw->hw_info->ops->process_cmd(hw,
					TPG_CONFIG_VC, &vc_config);

			list_for_each_safe(pos, pos_next, &req->vc_slots[i].head) {
				struct dt_config_args dt_config = {0};

				entry = list_entry(pos, struct tpg_hw_stream, list);
				dump_stream_configs(hw->hw_idx,
					stream_idx++,
					&entry->stream);
				dt_config.vc_slot = i;
				dt_config.dt_slot = dt_slot++;
				dt_config.stream  = &entry->stream;
				hw->hw_info->ops->process_cmd(hw, TPG_CONFIG_DT, &dt_config);
			}
		}
	} else if (hw->stream_version == 3) {
		for (i = 0; i < hw->hw_info->max_vc_channels; i++) {
			int dt_slot = 0;
			struct vc_config_args_v3 vc_config = {0};
			struct list_head *pos = NULL, *pos_next = NULL;
			struct tpg_hw_stream_v3 *entry = NULL, *vc_stream_entry = NULL;

			if (req->vc_slots[i].vc == -1)
				break;
			num_vcs++;
			vc_config.vc_slot = i;
			vc_config.num_dts = req->vc_slots[i].stream_count;
			vc_stream_entry = list_first_entry(&req->vc_slots[i].head,
				struct tpg_hw_stream_v3, list);
			vc_config.stream  = &vc_stream_entry->stream;
			hw->hw_info->ops->process_cmd(hw,
					TPG_CONFIG_VC, &vc_config);

			list_for_each_safe(pos, pos_next, &req->vc_slots[i].head) {
				struct dt_config_args_v3 dt_config = {0};

				entry = list_entry(pos, struct tpg_hw_stream_v3, list);
				dump_stream_configs_v3(hw->hw_idx,
					stream_idx++,
					&entry->stream);
				dt_config.vc_slot = i;
				dt_config.dt_slot = dt_slot++;
				dt_config.stream  = &entry->stream;
				hw->hw_info->ops->process_cmd(hw, TPG_CONFIG_DT, &dt_config);
			}
		}
	}
	globalargs.num_vcs      = num_vcs;
	globalargs.globalconfig = &req->global_config;
	hw->hw_info->ops->process_cmd(hw,
		TPG_CONFIG_CTRL, &globalargs);
	if (!cam_presil_mode_enabled())
		cam_tpg_mem_dmp(hw->soc_info);
end:
	return 0;
}

int tpg_hw_dump_status(struct tpg_hw *hw)
{
	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;
	switch (hw->hw_info->version) {
	case TPG_HW_VERSION_1_3:
	case TPG_HW_VERSION_1_3_1:
	case TPG_HW_VERSION_1_4:
		if (hw->hw_info->ops->dump_status)
			hw->hw_info->ops->dump_status(hw, NULL);
		break;
	default:
		CAM_WARN(CAM_TPG, "Hw version doesn't support status dump");
		break;
	}
	return 0;
}

static int tpg_hw_check_hw_state_and_apply_settings_locked(
	struct tpg_hw *hw,
	struct tpg_hw_request *req)
{
	int rc = 0;
	unsigned long wait_jiffies       = 0;
	unsigned long rem_jiffies        = 0;
	unsigned long flags;
	int32_t       timeout_multiplier = 0;

	if (!hw || !req) {
		CAM_ERR(CAM_TPG, "Invalid param");
		rc = -EINVAL;
		goto end;
	}
	if (req->request_type != TPG_HW_REQ_TYPE_NOP) {
		if (cam_presil_mode_enabled())
			timeout_multiplier = TIMEOUT_MULTIPLIER_PRESIL;
		else
			timeout_multiplier = TIMEOUT_MULTIPLIER;

		/* if hw state is not ready, wait for timeout.
		 * if hw state becomes ready handle in follow up if.
		 */
		if (hw->state == TPG_HW_STATE_BUSY) {
			wait_jiffies = CAM_TPG_HW_WAIT_TIMEOUT * timeout_multiplier;
			reinit_completion(&hw->complete_rup);
			rem_jiffies =
				cam_common_wait_for_completion_timeout(&hw->complete_rup,
					wait_jiffies);
			if (!rem_jiffies) {
				CAM_ERR(CAM_TPG, "TPG[%d] hw timeout %llu",
					hw->hw_idx, rem_jiffies);
				rc = -EBUSY;
				goto end;
			}
		}
		if (hw->state == TPG_HW_STATE_READY) {
			hw->settings_update = 1;
			spin_lock_irqsave(&hw->hw_state_lock, flags);
			hw->state = TPG_HW_STATE_BUSY;
			spin_unlock_irqrestore(&hw->hw_state_lock, flags);
			CAM_DBG(CAM_TPG, "HW State ready to busy");
		}
	}
	rc = tpg_hw_apply_settings_to_hw_locked(hw, req);
end:
	return rc;
}

static int tpg_hw_reapply_from_active_queue_locked(
	struct tpg_hw *hw,
	uint64_t request_id)
{
	int rc = 0;
	struct tpg_hw_request *reapply_req = NULL;
	struct tpg_hw_request *entry = NULL;
	struct list_head *pos = NULL, *pos_next = NULL;

	if (!hw) {
		CAM_ERR(CAM_TPG, "Invalid argument");
		return -EINVAL;
	}

	list_for_each_safe(pos, pos_next, &hw->active_request_q) {
		entry = list_entry(pos, struct tpg_hw_request, list);
		if (entry->request_id == request_id) {
			reapply_req = entry;
			break;
		}
	}

	if (reapply_req) {
		CAM_DBG(CAM_TPG, "TPG[%d] got reapply request %d", hw->hw_idx, request_id);
		rc = tpg_hw_check_hw_state_and_apply_settings_locked(hw, reapply_req);
	} else {
		CAM_ERR(CAM_TPG, "Could not find reapply request in active queue %d", request_id);
		return -EINVAL;
	}
	return rc;
}

static int tpg_hw_lookup_queues_and_apply_req_locked(
	struct tpg_hw *hw,
	uint64_t request_id)
{
	int rc = 0;
	struct tpg_hw_request *req = NULL, *active_req = NULL;

	if (!hw) {
		CAM_ERR(CAM_TPG, "Invalid argument");
		rc = -EINVAL;
		goto end;
	}

	if (!list_empty(&hw->waiting_request_q)) {
		req = list_first_entry(&hw->waiting_request_q,
			struct tpg_hw_request, list);
		if (req->request_id == request_id) {
			CAM_DBG(CAM_TPG, "TPG[%d] request %d got matched", hw->hw_idx, request_id);
			rc = tpg_hw_check_hw_state_and_apply_settings_locked(hw, req);

			if (rc == 0) {
				/* delete the request from waiting_q*/
				list_del(&req->list);
				hw->waiting_request_q_depth--;
				/* delete active request from active q*/
				if (!list_empty(&hw->active_request_q) &&
					(hw->active_request_q_depth >= MAX_ACTIVE_QUEUE_DEPTH)) {
					active_req = list_first_entry(&hw->active_request_q,
						struct tpg_hw_request, list);
					list_del(&active_req->list);
					hw->active_request_q_depth--;

					/* free the previously active request */
					tpg_hw_release_vc_slots_locked(hw, active_req);
				}

				/* Add the currently applied request to active queue*/
				list_add_tail(&req->list, &hw->active_request_q);
				hw->active_request_q_depth++;
			}
		} else {
			rc = tpg_hw_reapply_from_active_queue_locked(hw, request_id);
		}
	} else {
		CAM_DBG(CAM_TPG, "TPG[%d] got apply for request %d when waiting queue is empty",
			hw->hw_idx,
			request_id);
		rc = tpg_hw_reapply_from_active_queue_locked(hw, request_id);
	}

	if (rc == 0) {
		CAM_DBG(CAM_TPG, "TPG[%d] Hw Apply done for req %lld", hw->hw_idx, request_id);
	} else {
		CAM_ERR(CAM_TPG, "TPG[%d] Hw Apply Failed for req %lld", hw->hw_idx, request_id);
		rc = -EAGAIN;
	}

end:
	return rc;
}

/* apply all pending requests sequencially */
static int tpg_hw_apply_req_from_waiting_queue_locked(
	struct tpg_hw *hw)
{
	int rc = 0;
	struct list_head *pos = NULL, *pos_next = NULL;
	struct tpg_hw_request *req = NULL;
	struct tpg_hw_request *prev_req = NULL;

	if (!hw) {
		CAM_ERR(CAM_TPG, "Invalid Params");
		return -EINVAL;
	}

	list_for_each_safe(pos, pos_next, &hw->waiting_request_q) {
		req = list_entry(pos, struct tpg_hw_request, list);
		tpg_hw_apply_settings_to_hw_locked(hw, req);
		list_del(pos);
		hw->waiting_request_q_depth--;
		/* free previous req if any*/
		if (prev_req)
			tpg_hw_release_vc_slots_locked(hw, prev_req);
		prev_req = req;
	}

	/* last applied request in active */
	if (prev_req != NULL) {
		list_add_tail(&prev_req->list, &hw->active_request_q);
		hw->active_request_q_depth++;
	}
	return rc;
}

int tpg_hw_start(struct tpg_hw *hw)
{
	int rc = 0;
	struct tpg_reg_settings *reg_settings = NULL;
	struct tpg_settings_config_t *config = NULL;
	uint32_t settings_count = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;

	reg_settings = hw->register_settings;
	config = &hw->settings_config;
	settings_count = config->active_count;

	mutex_lock(&hw->mutex);
	switch (hw->hw_info->version) {
	case TPG_HW_VERSION_1_0:
	case TPG_HW_VERSION_1_1:
		if (hw->hw_info->ops->start)
			hw->hw_info->ops->start(hw, NULL);
		break;
	case TPG_HW_VERSION_1_2:
	case TPG_HW_VERSION_1_3:
	case TPG_HW_VERSION_1_3_1:
	case TPG_HW_VERSION_1_4:
		/* apply all initial requests */
		if (hw->hw_info->ops->start)
			hw->hw_info->ops->start(hw, NULL);

		if (settings_count != 0) {
			hw->hw_info->ops->write_settings(hw, config,
						reg_settings);
		} else {
			tpg_hw_apply_req_from_waiting_queue_locked(hw);
		}

		break;
	default:
		CAM_ERR(CAM_TPG, "TPG[%d] Unsupported HW Version",
			hw->hw_idx);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&hw->mutex);
	return rc;
}

int tpg_hw_stop(struct tpg_hw *hw)
{
	int rc = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;
	mutex_lock(&hw->mutex);
	switch (hw->hw_info->version) {
	case TPG_HW_VERSION_1_0:
	case TPG_HW_VERSION_1_1:
	case TPG_HW_VERSION_1_2:
	case TPG_HW_VERSION_1_3:
	case TPG_HW_VERSION_1_3_1:
	case TPG_HW_VERSION_1_4:
		if (hw->hw_info->ops->stop) {
			rc = hw->hw_info->ops->stop(hw, NULL);
			if (rc) {
				CAM_ERR(CAM_TPG, "TPG[%d] hw stop failed %d",
					hw->hw_idx, rc);
				break;
			}
		}
		rc = tpg_hw_soc_disable(hw);
		if (rc) {
			CAM_ERR(CAM_TPG, "TPG[%d] hw soc disable failed %d",
				hw->hw_idx, rc);
			break;
		}
		tpg_hw_free_waiting_requests_locked(hw);
		tpg_hw_free_active_requests_locked(hw);
		break;
	default:
		CAM_ERR(CAM_TPG, "TPG[%d] Unsupported HW Version",
			hw->hw_idx);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&hw->mutex);

	return rc;
}

int tpg_hw_acquire(struct tpg_hw *hw,
		struct tpg_hw_acquire_args *acquire)
{
	int rc = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;

	mutex_lock(&hw->mutex);
	switch (hw->hw_info->version) {
	case TPG_HW_VERSION_1_0:
	case TPG_HW_VERSION_1_1:
	case TPG_HW_VERSION_1_2:
	case TPG_HW_VERSION_1_3:
	case TPG_HW_VERSION_1_3_1:
	case TPG_HW_VERSION_1_4:
		// Start Cpas and enable required clocks
		break;
	default:
		CAM_ERR(CAM_TPG, "TPG[%d] Unsupported HW Version",
			hw->hw_idx);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&hw->mutex);
	return rc;
}

int tpg_hw_release(struct tpg_hw *hw)
{
	int rc = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;
	mutex_lock(&hw->mutex);
	switch (hw->hw_info->version) {
	case TPG_HW_VERSION_1_0:
	case TPG_HW_VERSION_1_1:
	case TPG_HW_VERSION_1_2:
	case TPG_HW_VERSION_1_3:
	case TPG_HW_VERSION_1_3_1:
	case TPG_HW_VERSION_1_4:
		break;
	default:
		CAM_ERR(CAM_TPG, "TPG[%d] Unsupported HW Version",
			hw->hw_idx);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&hw->mutex);
	return rc;
}

enum cam_vote_level get_tpg_clk_level(
		struct tpg_hw *hw)
{
	uint32_t cam_vote_level = 0;
	uint32_t last_valid_vote = 0;
	uint64_t clk = 0;
	struct cam_hw_soc_info *soc_info;

	if (!hw || !hw->soc_info)
		return -EINVAL;

	soc_info = hw->soc_info;
	clk = hw->tpg_clock;

	for (cam_vote_level = 0;
			cam_vote_level < CAM_MAX_VOTE; cam_vote_level++) {
		if (soc_info->clk_level_valid[cam_vote_level] != true)
			continue;

		if (soc_info->clk_rate[cam_vote_level]
			[soc_info->src_clk_idx] >= clk) {
			CAM_INFO(CAM_TPG,
				"match detected %s : %llu:%d level : %d",
				soc_info->clk_name[soc_info->src_clk_idx],
				clk,
				soc_info->clk_rate[cam_vote_level]
				[soc_info->src_clk_idx],
				cam_vote_level);
			return cam_vote_level;
		}
		last_valid_vote = cam_vote_level;
	}
	return last_valid_vote;
}

static int tpg_hw_configure_init_settings(
		struct tpg_hw *hw,
		struct tpg_hw_initsettings *settings)
{
	int rc = 0;
	int clk_level = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;
	mutex_lock(&hw->mutex);
	switch (hw->hw_info->version) {
	case TPG_HW_VERSION_1_0:
	case TPG_HW_VERSION_1_1:
	case TPG_HW_VERSION_1_2:
	case TPG_HW_VERSION_1_3:
	case TPG_HW_VERSION_1_3_1:
	case TPG_HW_VERSION_1_4:
		/* Need to handle the clock from the init config */
		/* Need to handle this latter */
		clk_level = get_tpg_clk_level(hw);
		rc = tpg_hw_soc_enable(hw, clk_level, true);

		if (rc) {
			CAM_ERR(CAM_TPG, "TPG[%d] hw soc enable failed %d",
				hw->hw_idx, rc);
			break;
		}

		if (hw->hw_info->ops->init)
			rc = hw->hw_info->ops->init(hw, settings);

		if (rc) {
			CAM_ERR(CAM_TPG, "TPG[%d] hw init failed %d",
				hw->hw_idx, rc);
		}
		break;
	default:
		CAM_ERR(CAM_TPG, "TPG[%d] Unsupported HW Version",
			hw->hw_idx);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&hw->mutex);
	return rc;
}

static int tpg_hw_configure_init_settings_v3(
		struct tpg_hw *hw,
		struct tpg_hw_initsettings_v3 *settings)
{
	int rc = 0;
	int clk_level = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;
	mutex_lock(&hw->mutex);
	switch (hw->hw_info->version) {
	case TPG_HW_VERSION_1_0:
	case TPG_HW_VERSION_1_1:
	case TPG_HW_VERSION_1_2:
	case TPG_HW_VERSION_1_3:
	case TPG_HW_VERSION_1_3_1:
	case TPG_HW_VERSION_1_4:
		clk_level = get_tpg_clk_level(hw);
		rc = tpg_hw_soc_enable(hw, clk_level, true);

		if (rc) {
			CAM_ERR(CAM_TPG, "TPG[%d] hw soc enable failed %d",
				hw->hw_idx, rc);
			break;
		}

		if (hw->hw_info->ops->init)
			rc = hw->hw_info->ops->init(hw, settings);

		if (rc) {
			CAM_ERR(CAM_TPG, "TPG[%d] hw init failed %d",
				hw->hw_idx, rc);
		}
		break;
	default:
		CAM_ERR(CAM_TPG, "TPG[%d] Unsupported HW Version",
			hw->hw_idx);
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&hw->mutex);
	return rc;
}


int tpg_hw_config(
	struct tpg_hw *hw,
	enum tpg_hw_cmd_t config_cmd,
	void *config_args)
{
	int rc = 0;

	if (!hw || !hw->hw_info || !hw->hw_info->ops)
		return -EINVAL;
	switch (config_cmd) {
	case TPG_HW_CMD_INIT_CONFIG:
		//validate_stream_list(hw);
		if (hw->settings_config.active_count != 0) {
			tpg_hw_configure_init_settings_v3(hw,
					(struct tpg_hw_initsettings_v3 *)config_args);
		} else {
			if (hw->stream_version == 1) {
				tpg_hw_configure_init_settings(hw,
					(struct tpg_hw_initsettings *)config_args);
			} else if (hw->stream_version == 3) {
				tpg_hw_configure_init_settings_v3(hw,
					(struct tpg_hw_initsettings_v3 *)config_args);
			}
		}
		break;
	default:
		CAM_ERR(CAM_TPG, "TPG[%d] Unsupported hw config command",
			hw->hw_idx);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int tpg_hw_copy_settings_config(
	struct tpg_hw *hw,
	struct tpg_settings_config_t *settings)
{
	struct tpg_reg_settings *reg_settings;

	if (!hw || !settings) {
		CAM_ERR(CAM_TPG, "invalid parameter");
		return -EINVAL;
	}

	hw->register_settings =
		kzalloc(sizeof(struct tpg_reg_settings) *
		settings->settings_array_size, GFP_KERNEL);

	if (hw->register_settings == NULL) {
		CAM_ERR(CAM_TPG, "unable to allocate memory");
		return -EINVAL;
	}

	reg_settings = (struct tpg_reg_settings *)
		((uint8_t *)settings + settings->settings_array_offset);

	mutex_lock(&hw->mutex);
	memcpy(&hw->settings_config,
		settings,
		sizeof(struct tpg_settings_config_t));
	memcpy(hw->register_settings,
		reg_settings,
		sizeof(struct tpg_reg_settings) * settings->settings_array_size);
	mutex_unlock(&hw->mutex);

	return 0;
}

static int assign_vc_slot(
	struct tpg_hw *hw,
	int  vc,
	struct tpg_hw_request *req,
	struct tpg_hw_stream *stream
	)
{
	int rc = -EINVAL, i = 0, slot_matched = 0;

	if (!hw || !stream || !req) {
		return -EINVAL;
	}

	for (i = 0; i < hw->hw_info->max_vc_channels; i++) {
		/* Found a matching slot */
		if (req->vc_slots[i].vc == vc) {
			slot_matched = 1;
			if (req->vc_slots[i].stream_count
					< hw->hw_info->max_dt_channels_per_vc) {
				list_add_tail(&stream->list, &req->vc_slots[i].head);
				req->vc_slots[i].stream_count++;
				req->vc_slots[i].vc = vc;
				rc = 0;
				CAM_DBG(CAM_TPG, "vc[%d]dt[%d]=>slot[%d]", vc, stream->stream.dt, i);
				break;
			} else {

				/**
				 * already slot was assigned for this vc
				 * however this slot have been filled with
				 * full streams
				 */
				rc = -EINVAL;
				CAM_ERR(CAM_TPG, "vc[%d]dt[%d]=>slot[%d] is overlfown",
						vc, stream->stream.dt, i);
				break;
			}
		}

		/**
		 * none of the above slots matched, and now found an empty slot
		 * so assigning stream to that slot
		 */
		if (req->vc_slots[i].vc == -1) {
			list_add_tail(&stream->list, &req->vc_slots[i].head);
			req->vc_slots[i].stream_count++;
			req->vc_slots[i].vc = vc;
			req->vc_count++;
			rc = 0;
			CAM_DBG(CAM_TPG, "vc[%d]dt[%d]=>slot[%d]", vc, stream->stream.dt, i);
			break;
		}
	}
	if ((slot_matched == 0) && (rc != 0)) {
		CAM_ERR(CAM_TPG, "No slot matched");
	}
	return rc;
}

static int assign_vc_slot_v3(
	struct tpg_hw *hw,
	int  vc,
	struct tpg_hw_request *req,
	struct tpg_hw_stream_v3 *stream
	)
{
	int rc = -EINVAL, i = 0, slot_matched = 0;

	if (!hw || !stream || !req)
		return -EINVAL;

	for (i = 0; i < hw->hw_info->max_vc_channels; i++) {
		/* Found a matching slot */
		if (req->vc_slots[i].vc == vc) {
			slot_matched = 1;
			if (req->vc_slots[i].stream_count
					< hw->hw_info->max_dt_channels_per_vc) {
				list_add_tail(&stream->list, &req->vc_slots[i].head);
				req->vc_slots[i].stream_count++;
				req->vc_slots[i].vc = vc;
				rc = 0;
				CAM_DBG(CAM_TPG, "vc[%d]dt[%d]=>slot[%d]",
					vc,
					stream->stream.dt,
					i);
			} else {

				/**
				 * already slot was assigned for this vc
				 * however this slot have been filled with
				 * full streams
				 */
				rc = -EINVAL;
				CAM_ERR(CAM_TPG, "vc[%d]dt[%d]=>slot[%d] is overlfown",
					vc, stream->stream.dt, i);
			}
			break;
		}

		/**
		 * none of the above slots matched, and now found an empty slot
		 * so assigning stream to that slot
		 */
		if (req->vc_slots[i].vc == -1) {
			list_add_tail(&stream->list, &req->vc_slots[i].head);
			req->vc_slots[i].stream_count++;
			req->vc_slots[i].vc = vc;
			req->vc_count++;
			rc = 0;
			CAM_DBG(CAM_TPG, "vc[%d]dt[%d]=>slot[%d]", vc, stream->stream.dt, i);
			break;
		}
	}
	if ((slot_matched == 0) && (rc != 0))
		CAM_ERR(CAM_TPG, "No slot matched");
	return rc;
}

int tpg_hw_free_request(
	struct tpg_hw *hw,
	struct tpg_hw_request *req)
{
	int rc = 0;

	if (!hw || !req)
		return -EINVAL;

	mutex_lock(&hw->mutex);
	rc = tpg_hw_release_vc_slots_locked(hw, req);
	mutex_unlock(&hw->mutex);
	return rc;
}

int tpg_hw_reset(struct tpg_hw *hw)
{
	int rc = 0;
	unsigned long flags;
	if (!hw)
		return -EINVAL;

	CAM_DBG(CAM_TPG, "TPG HW reset");
	/* disable the hw */
	mutex_lock(&hw->mutex);

	rc = tpg_hw_free_waiting_requests_locked(hw);
	if (rc)
		CAM_ERR(CAM_TPG, "TPG[%d] unable to free up the pending requests",
				hw->hw_idx);

	if (hw->state != TPG_HW_STATE_HW_DISABLED) {
		rc = cam_soc_util_disable_platform_resource(hw->soc_info, CAM_CLK_SW_CLIENT_IDX,
			true, false);
		if (rc) {
			CAM_ERR(CAM_TPG, "TPG[%d] Disable platform failed %d", hw->hw_idx, rc);
		}
		rc = cam_cpas_stop(hw->cpas_handle);
		if (rc) {
			CAM_ERR(CAM_TPG, "TPG[%d] CPAS stop failed", hw->hw_idx);
		}
		spin_lock_irqsave(&hw->hw_state_lock, flags);
		hw->state =  TPG_HW_STATE_HW_DISABLED;
		spin_unlock_irqrestore(&hw->hw_state_lock, flags);
	}

	rc = tpg_hw_free_active_requests_locked(hw);
	if (rc) {
		CAM_ERR(CAM_TPG, "TPG[%d] unable to free up the active requests",
				hw->hw_idx);
	}
	mutex_unlock(&hw->mutex);

	return rc;
}

int tpg_hw_add_stream(
	struct tpg_hw *hw,
	struct tpg_hw_request *req,
	struct tpg_stream_config_t *cmd)
{
	int rc = 0;
	struct tpg_hw_stream *stream = NULL;
	if (!hw || !req || !cmd) {
		CAM_ERR(CAM_TPG, "Invalid params");
		return -EINVAL;
	}

	hw->stream_version = 1;
	mutex_lock(&hw->mutex);
	stream = kzalloc(sizeof(struct tpg_hw_stream), GFP_KERNEL);
	if (!stream) {
		CAM_ERR(CAM_TPG, "TPG[%d] stream allocation failed",
			hw->hw_idx);
		mutex_unlock(&hw->mutex);
		return -ENOMEM;
	}
	memcpy(&stream->stream,
		cmd,
		sizeof(struct tpg_stream_config_t));

	rc = assign_vc_slot(hw, stream->stream.vc, req, stream);
	mutex_unlock(&hw->mutex);
	return rc;
}

int tpg_hw_add_request(struct tpg_hw *hw,
	struct tpg_hw_request *req)
{
	int rc = 0;

	if (!hw || !req) {
		CAM_ERR(CAM_TPG, "Invalid params");
		return -EINVAL;
	}
	mutex_lock(&hw->mutex);
	if (hw->waiting_request_q_depth <= MAX_WAITING_QUEUE_DEPTH) {
		list_add_tail(&req->list, &hw->waiting_request_q);
		hw->waiting_request_q_depth++;
	} else {
		CAM_ERR(CAM_TPG, "waiting queue size is exceed");
	}
	mutex_unlock(&hw->mutex);
	return rc;
}

struct tpg_hw_request *tpg_hw_create_request(
	struct tpg_hw *hw,
	uint64_t request_id)
{
	struct tpg_hw_request *req = NULL;
	uint32_t num_vc_channels = hw->hw_info->max_vc_channels;
	uint32_t i = 0;

	if (!hw) {
		CAM_ERR(CAM_TPG, "Invalid params");
		return NULL;
	}

	/* Allocate request */
	req = kzalloc(sizeof(struct tpg_hw_request),
			GFP_KERNEL);
	if (!req) {
		CAM_ERR(CAM_TPG, "TPG[%d] request allocation failed",
				hw->hw_idx);
		return NULL;
	}
	req->request_id = request_id;
	/* Allocate Vc slots in request */
	req->vc_slots = kcalloc(num_vc_channels, sizeof(struct tpg_vc_slot_info),
			GFP_KERNEL);

	req->vc_count = 0;
	if (!req->vc_slots) {
		CAM_ERR(CAM_TPG, "TPG[%d] vc slot allocation failed",
				hw->hw_idx);
		goto err_exit_1;
	}

	INIT_LIST_HEAD(&req->list);
	/* Initialize the slot variables */
	for (i = 0; i < hw->hw_info->max_vc_channels; i++) {
		req->vc_slots[i].slot_id      =  i;
		req->vc_slots[i].vc           = -1;
		req->vc_slots[i].stream_count =  0;
		INIT_LIST_HEAD(&(req->vc_slots[i].head));
	}
	CAM_DBG(CAM_TPG, "TPG[%d] request(%lld) allocated success",
			hw->hw_idx, request_id);
	return req;
err_exit_1:
	kfree(req);
	return NULL;
}

int tpg_hw_request_set_opcode(
	struct tpg_hw_request *req,
	uint32_t opcode)
{
	int rc = 0;

	if (!req) {
		CAM_ERR(CAM_TPG, "Invalid params");
		return -EINVAL;
	}
	switch (opcode) {
	case CAM_TPG_PACKET_OPCODE_INITIAL_CONFIG:
		req->request_type = TPG_HW_REQ_TYPE_INIT;
		break;
	case CAM_TPG_PACKET_OPCODE_NOP:
		req->request_type = TPG_HW_REQ_TYPE_NOP;
		break;
	case CAM_TPG_PACKET_OPCODE_UPDATE:
		req->request_type = TPG_HW_REQ_TYPE_UPDATE;
		break;
	default:
		req->request_type = TPG_HW_REQ_TYPE_NOP;
		break;
	}
	CAM_INFO(CAM_TPG, "req[%d] type = %d",
			req->request_id,
			req->request_type);
	return rc;
}

int tpg_hw_apply(
	struct tpg_hw *hw,
	uint64_t request_id)
{
	int rc = 0;

	if (!hw) {
		CAM_ERR(CAM_TPG, "Invalid params");
		return -EINVAL;
	}
	mutex_lock(&hw->mutex);
	rc = tpg_hw_lookup_queues_and_apply_req_locked(hw, request_id);
	mutex_unlock(&hw->mutex);
	return rc;
}
int tpg_hw_add_stream_v3(
	struct tpg_hw *hw,
	struct tpg_hw_request *req,
	struct tpg_stream_config_v3_t *cmd)
{
	int rc = 0;
	struct tpg_hw_stream_v3 *stream = NULL;

	if (!hw || !req || !cmd) {
		CAM_ERR(CAM_TPG, "Invalid params");
		return -EINVAL;
	}

	hw->stream_version = 3;
	mutex_lock(&hw->mutex);
	stream = kzalloc(sizeof(struct tpg_hw_stream_v3), GFP_KERNEL);
	if (!stream) {
		CAM_ERR(CAM_TPG, "TPG[%d] stream allocation failed",
			hw->hw_idx);
		mutex_unlock(&hw->mutex);
		return -ENOMEM;
	}
	memcpy(&stream->stream,
		cmd,
		sizeof(struct tpg_stream_config_v3_t));

	rc = assign_vc_slot_v3(hw, stream->stream.vc, req, stream);
	mutex_unlock(&hw->mutex);
	return rc;
}

