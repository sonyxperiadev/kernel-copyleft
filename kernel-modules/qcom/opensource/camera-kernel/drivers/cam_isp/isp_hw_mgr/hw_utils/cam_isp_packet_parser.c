// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <media/cam_defs.h>
#include <media/cam_isp.h>
#include "cam_mem_mgr.h"
#include "cam_isp_hw.h"
#include "cam_vfe_hw_intf.h"
#include "cam_isp_packet_parser.h"
#include "cam_debug_util.h"
#include "cam_isp_hw_mgr_intf.h"

static void cam_isp_add_update_entry(
	enum cam_isp_cdm_bl_type            cdm_bl_type,
	struct cam_hw_prepare_update_args  *prepare,
	struct cam_kmd_buf_info            *kmd_buf_info,
	uint32_t                            update_size)
{
	/* Validation is done from the caller */
	uint32_t num_ent;
	struct cam_hw_update_entry *curr_update_entry;

	num_ent = prepare->num_hw_update_entries;
	curr_update_entry = &(prepare->hw_update_entries[num_ent]);
	curr_update_entry->handle = kmd_buf_info->handle;
	curr_update_entry->len = update_size;
	curr_update_entry->offset = kmd_buf_info->offset;
	curr_update_entry->flags = cdm_bl_type;
	curr_update_entry->addr = (uintptr_t) (kmd_buf_info->cpu_addr +
		kmd_buf_info->used_bytes/4);

	num_ent++;
	kmd_buf_info->used_bytes += update_size;
	kmd_buf_info->offset     += update_size;
	prepare->num_hw_update_entries = num_ent;

	CAM_DBG(CAM_ISP,
		"Add new entry: num_ent=%d handle=0x%x, len=%u, offset=%u",
		num_ent - 1, curr_update_entry->handle,
		curr_update_entry->len, curr_update_entry->offset);
}

static void cam_isp_combine_update_entry(
	struct cam_hw_prepare_update_args  *prepare,
	struct cam_kmd_buf_info            *kmd_buf_info,
	uint32_t                            update_size)
{
	/* Validation is done from the caller */
	uint32_t num_ent;
	struct cam_hw_update_entry *prev_update_entry;

	num_ent = prepare->num_hw_update_entries;
	prev_update_entry = &(prepare->hw_update_entries[num_ent - 1]);
	prev_update_entry->len += update_size;

	CAM_DBG(CAM_ISP,
		"Combined with prev entry: num_ent=%d handle=0x%x, len=%u, offset=%u",
		num_ent - 1, prev_update_entry->handle,
		prev_update_entry->len, prev_update_entry->offset);

	kmd_buf_info->used_bytes += update_size;
	kmd_buf_info->offset += update_size;
}

void cam_isp_update_hw_entry(
	enum cam_isp_cdm_bl_type            cdm_bl_type,
	struct cam_hw_prepare_update_args  *prepare,
	struct cam_kmd_buf_info            *kmd_buf_info,
	uint32_t                            update_size,
	bool                                combine)
{
	if (combine)
		cam_isp_combine_update_entry(prepare,
			kmd_buf_info, update_size);
	else
		cam_isp_add_update_entry(cdm_bl_type, prepare,
			kmd_buf_info, update_size);
}

int cam_isp_add_change_base(
	struct cam_hw_prepare_update_args      *prepare,
	struct list_head                       *res_list_isp_src,
	struct cam_isp_change_base_args        *change_base_args,
	struct cam_kmd_buf_info                *kmd_buf_info)
{
	int rc = -EINVAL;
	struct cam_isp_hw_mgr_res       *hw_mgr_res;
	struct cam_isp_resource_node    *res;
	struct cam_isp_hw_get_cmd_update get_base;
	uint32_t                         num_ent, i;

	num_ent = prepare->num_hw_update_entries;

	/* Max one hw entries required for each base */
	if (num_ent + 1 >= prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
			num_ent, prepare->max_hw_update_entries);
		return -EINVAL;
	}

	list_for_each_entry(hw_mgr_res, res_list_isp_src, list) {
		if (hw_mgr_res->res_type == CAM_ISP_RESOURCE_UNINT)
			continue;

		for (i = 0; i < CAM_ISP_HW_SPLIT_MAX; i++) {
			if (!hw_mgr_res->hw_res[i])
				continue;

			res = hw_mgr_res->hw_res[i];
			if (res->hw_intf->hw_idx != change_base_args->base_idx)
				continue;

			get_base.res  = res;
			get_base.cdm_id = change_base_args->cdm_id;
			get_base.cmd_type = CAM_ISP_HW_CMD_GET_CHANGE_BASE;
			get_base.cmd.cmd_buf_addr = kmd_buf_info->cpu_addr +
				kmd_buf_info->used_bytes/4;
			get_base.cmd.size  = kmd_buf_info->size -
					kmd_buf_info->used_bytes;

			rc = res->hw_intf->hw_ops.process_cmd(
				res->hw_intf->hw_priv,
				CAM_ISP_HW_CMD_GET_CHANGE_BASE, &get_base,
				sizeof(struct cam_isp_hw_get_cmd_update));
			if (rc)
				return rc;

			cam_isp_add_update_entry(CAM_ISP_COMMON_CFG_BL, prepare,
				kmd_buf_info, get_base.cmd.used_bytes);

			/* return success */
			return 0;
		}
	}

	return rc;
}

static int cam_isp_update_dual_config(
	struct cam_cmd_buf_desc            *cmd_desc,
	uint32_t                            split_id,
	uint32_t                            base_idx,
	struct cam_isp_hw_mgr_res          *res_list_isp_out,
	uint8_t                            *out_map,
	uint32_t                            out_base,
	uint32_t                            out_max)
{
	int rc = -EINVAL;
	struct cam_isp_dual_config                 *dual_config;
	struct cam_isp_hw_mgr_res                  *hw_mgr_res;
	struct cam_isp_resource_node               *res;
	struct cam_isp_hw_dual_isp_update_args      dual_isp_update_args;
	uint32_t                                    port_id = 0;
	uint32_t                                    ports_plane_idx;
	size_t                                      len = 0, remain_len = 0;
	uint32_t                                   *cpu_addr;
	uint32_t                                    i, j;

	CAM_DBG(CAM_ISP, "cmd des size %d, length: %d",
		cmd_desc->size, cmd_desc->length);

	rc = cam_packet_util_get_cmd_mem_addr(
		cmd_desc->mem_handle, &cpu_addr, &len);
	if (rc)
		return rc;

	if ((len < sizeof(struct cam_isp_dual_config)) ||
		(cmd_desc->offset >=
		(len - sizeof(struct cam_isp_dual_config)))) {
		CAM_ERR(CAM_ISP, "not enough buffer provided");
		cam_mem_put_cpu_buf(cmd_desc->mem_handle);
		return -EINVAL;
	}
	remain_len = len - cmd_desc->offset;
	cpu_addr += (cmd_desc->offset / 4);
	dual_config = (struct cam_isp_dual_config *)cpu_addr;

	if ((dual_config->num_ports *
		sizeof(struct cam_isp_dual_stripe_config)) >
		(remain_len - offsetof(struct cam_isp_dual_config, stripes))) {
		CAM_ERR(CAM_ISP, "not enough buffer for all the dual configs");
		cam_mem_put_cpu_buf(cmd_desc->mem_handle);
		return -EINVAL;
	}
	for (i = 0; i < dual_config->num_ports; i++) {

		if (i >= (out_max & 0xFF)) {
			CAM_ERR(CAM_ISP,
				"failed update for i:%d > size_isp_out:%d",
				i, (out_max & 0xFF));
			rc = -EINVAL;
			goto end;
		}

		for (j = 0; j < CAM_ISP_HW_SPLIT_MAX; j++) {
			ports_plane_idx = (j * (dual_config->num_ports *
				CAM_PACKET_MAX_PLANES)) +
				(i * CAM_PACKET_MAX_PLANES);

			if (dual_config->stripes[ports_plane_idx].port_id == 0)
				continue;
			port_id = dual_config->stripes[ports_plane_idx].port_id;
			hw_mgr_res = &res_list_isp_out[out_map[port_id & 0xFF]];
			if (!hw_mgr_res) {
				CAM_ERR(CAM_ISP,
					"Invalid isp out resource i %d num_out_res %d",
					i, dual_config->num_ports);
				rc = -EINVAL;
				goto end;
			}

			if (!hw_mgr_res->hw_res[j])
				continue;

			if (hw_mgr_res->hw_res[j]->hw_intf->hw_idx != base_idx)
				continue;

			res = hw_mgr_res->hw_res[j];

			if (res->res_id < out_base ||
				res->res_id >= out_max)
				continue;

			dual_isp_update_args.split_id = j;
			dual_isp_update_args.res      = res;
			dual_isp_update_args.dual_cfg = dual_config;
			rc = res->hw_intf->hw_ops.process_cmd(
				res->hw_intf->hw_priv,
				CAM_ISP_HW_CMD_STRIPE_UPDATE,
				&dual_isp_update_args,
				sizeof(struct cam_isp_hw_dual_isp_update_args));
			if (rc)
				goto end;
		}
	}

end:
	cam_mem_put_cpu_buf(cmd_desc->mem_handle);
	return rc;
}

int cam_isp_add_cmd_buf_update(
	struct cam_isp_resource_node         *res,
	struct cam_hw_intf                   *hw_intf,
	uint32_t                              cmd_type,
	uint32_t                              hw_cmd_type,
	uint32_t                             *cmd_buf_addr,
	uint32_t                              kmd_buf_remain_size,
	void                                 *cmd_update_data,
	uint32_t                             *bytes_used)
{
	int rc = 0;
	struct cam_isp_hw_get_cmd_update    cmd_update;
	uint32_t                            total_used_bytes = 0;

	cmd_update.cmd_type = hw_cmd_type;
	cmd_update.cmd.cmd_buf_addr = cmd_buf_addr;
	cmd_update.cmd.size = kmd_buf_remain_size;
	cmd_update.cmd.used_bytes = 0;
	cmd_update.data = cmd_update_data;
	CAM_DBG(CAM_ISP, "cmd_type %u cmd buffer 0x%pK, size %d",
		cmd_update.cmd_type,
		cmd_update.cmd.cmd_buf_addr,
		cmd_update.cmd.size);

	cmd_update.res = res;
	rc = hw_intf->hw_ops.process_cmd(
		hw_intf->hw_priv,
		cmd_update.cmd_type, &cmd_update,
		sizeof(struct cam_isp_hw_get_cmd_update));

	if (rc) {
		CAM_ERR(CAM_ISP, "get buf cmd error:%d cmd %d", hw_intf->hw_idx,
		cmd_update.cmd_type);
		rc = -ENOMEM;
		return rc;
	}

	total_used_bytes += cmd_update.cmd.used_bytes;
	*bytes_used = total_used_bytes;
	CAM_DBG(CAM_ISP, "total_used_bytes %u", total_used_bytes);
	return rc;
}

int cam_isp_add_command_buffers(
	struct cam_hw_prepare_update_args  *prepare,
	struct cam_kmd_buf_info            *kmd_buf_info,
	struct cam_isp_ctx_base_info       *base_info,
	cam_packet_generic_blob_handler     blob_handler_cb,
	struct cam_isp_hw_mgr_res          *res_list_isp_out,
	uint8_t                            *out_map,
	uint32_t                            out_base,
	uint32_t                            out_max)
{
	int rc = 0;
	uint32_t                           cmd_meta_data, num_ent, i;
	uint32_t                           base_idx;
	enum cam_isp_hw_split_id           split_id;
	struct cam_cmd_buf_desc           *cmd_desc = NULL;
	struct cam_hw_update_entry        *hw_entry = NULL;

	split_id = base_info->split_id;
	base_idx = base_info->idx;
	hw_entry = prepare->hw_update_entries;

	/*
	 * set the cmd_desc to point the first command descriptor in the
	 * packet
	 */
	cmd_desc = (struct cam_cmd_buf_desc *)
			((uint8_t *)&prepare->packet->payload +
			prepare->packet->cmd_buf_offset);

	CAM_DBG(CAM_ISP, "split id = %d, number of command buffers:%d",
		split_id, prepare->packet->num_cmd_buf);

	for (i = 0; i < prepare->packet->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		num_ent = prepare->num_hw_update_entries;
		if (!cmd_desc[i].length)
			continue;

		/* One hw entry space required for left or right or common */
		if (num_ent + 1 >= prepare->max_hw_update_entries) {
			CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
				num_ent, prepare->max_hw_update_entries);
			return -EINVAL;
		}

		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		cmd_meta_data = cmd_desc[i].meta_data;

		CAM_DBG(CAM_ISP, "meta type: %d, split_id: %d",
			cmd_meta_data, split_id);

		switch (cmd_meta_data) {
		case CAM_ISP_PACKET_META_BASE:
		case CAM_ISP_PACKET_META_LEFT:
		case CAM_ISP_PACKET_META_DMI_LEFT:
			if (split_id == CAM_ISP_HW_SPLIT_LEFT) {
				hw_entry[num_ent].len = cmd_desc[i].length;
				hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
				hw_entry[num_ent].offset = cmd_desc[i].offset;
				CAM_DBG(CAM_ISP,
					"Meta_Left num_ent=%d handle=0x%x, len=%u, offset=%u",
					num_ent,
					hw_entry[num_ent].handle,
					hw_entry[num_ent].len,
					hw_entry[num_ent].offset);
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent++;
			}
			break;
		case CAM_ISP_PACKET_META_RIGHT:
		case CAM_ISP_PACKET_META_DMI_RIGHT:
			if (split_id == CAM_ISP_HW_SPLIT_RIGHT) {
				hw_entry[num_ent].len = cmd_desc[i].length;
				hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
				hw_entry[num_ent].offset = cmd_desc[i].offset;
				CAM_DBG(CAM_ISP,
					"Meta_Right num_ent=%d handle=0x%x, len=%u, offset=%u",
					num_ent,
					hw_entry[num_ent].handle,
					hw_entry[num_ent].len,
					hw_entry[num_ent].offset);
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent++;
			}
			break;
		case CAM_ISP_PACKET_META_COMMON:
		case CAM_ISP_PACKET_META_DMI_COMMON:
			hw_entry[num_ent].len = cmd_desc[i].length;
			hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
			hw_entry[num_ent].offset = cmd_desc[i].offset;
			CAM_DBG(CAM_ISP,
				"Meta_Common num_ent=%d handle=0x%x, len=%u, offset=%u",
				num_ent,
				hw_entry[num_ent].handle,
				hw_entry[num_ent].len,
				hw_entry[num_ent].offset);
			hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
			num_ent++;
			break;
		case CAM_ISP_PACKET_META_DUAL_CONFIG:
			rc = cam_isp_update_dual_config(&cmd_desc[i],
				split_id, base_idx, res_list_isp_out,
				out_map, out_base, out_max);
			if (rc)
				return rc;
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_LEFT:
			if (split_id == CAM_ISP_HW_SPLIT_LEFT) {
				struct cam_isp_generic_blob_info   blob_info;

				blob_info.prepare = prepare;
				blob_info.base_info = base_info;
				blob_info.kmd_buf_info = kmd_buf_info;
				blob_info.entry_added = false;

				rc = cam_packet_util_process_generic_cmd_buffer(
					&cmd_desc[i],
					blob_handler_cb,
					&blob_info);
				if (rc) {
					CAM_ERR(CAM_ISP,
						"Failed in processing blobs %d",
						rc);
					return rc;
				}
				num_ent = prepare->num_hw_update_entries;
			}
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_RIGHT:
			if (split_id == CAM_ISP_HW_SPLIT_RIGHT) {
				struct cam_isp_generic_blob_info   blob_info;

				blob_info.prepare = prepare;
				blob_info.base_info = base_info;
				blob_info.kmd_buf_info = kmd_buf_info;
				blob_info.entry_added = false;

				rc = cam_packet_util_process_generic_cmd_buffer(
					&cmd_desc[i],
					blob_handler_cb,
					&blob_info);
				if (rc) {
					CAM_ERR(CAM_ISP,
						"Failed in processing blobs %d",
						rc);
					return rc;
				}
				num_ent = prepare->num_hw_update_entries;
			}
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_COMMON: {
			struct cam_isp_generic_blob_info   blob_info;

			blob_info.prepare = prepare;
			blob_info.base_info = base_info;
			blob_info.kmd_buf_info = kmd_buf_info;
			blob_info.entry_added = false;

			rc = cam_packet_util_process_generic_cmd_buffer(
				&cmd_desc[i],
				blob_handler_cb,
				&blob_info);
			if (rc) {
				CAM_ERR(CAM_ISP,
					"Failed in processing blobs %d", rc);
				return rc;
			}
			num_ent = prepare->num_hw_update_entries;
		}
			break;
		case CAM_ISP_PACKET_META_REG_DUMP_ON_FLUSH:
		case CAM_ISP_PACKET_META_REG_DUMP_ON_ERROR:
		case CAM_ISP_PACKET_META_REG_DUMP_PER_REQUEST:
			if (split_id == CAM_ISP_HW_SPLIT_LEFT) {
				if (prepare->num_reg_dump_buf >=
					CAM_REG_DUMP_MAX_BUF_ENTRIES) {
					CAM_ERR(CAM_ISP,
					"Descriptor count out of bounds: %d",
					prepare->num_reg_dump_buf);
					return -EINVAL;
				}

				prepare->reg_dump_buf_desc[
					prepare->num_reg_dump_buf] =
					cmd_desc[i];
				prepare->num_reg_dump_buf++;
				CAM_DBG(CAM_ISP,
					"Added command buffer: %d desc_count: %d",
					cmd_desc[i].meta_data,
					prepare->num_reg_dump_buf);
			}
			break;
		case CAM_ISP_SFE_PACKET_META_LEFT:
		case CAM_ISP_SFE_PACKET_META_RIGHT:
		case CAM_ISP_SFE_PACKET_META_COMMON:
		case CAM_ISP_SFE_PACKET_META_DUAL_CONFIG:
			break;
		case CAM_ISP_PACKET_META_CSID_LEFT:
		case CAM_ISP_PACKET_META_CSID_RIGHT:
		case CAM_ISP_PACKET_META_CSID_COMMON:
			break;
		default:
			CAM_ERR(CAM_ISP, "invalid cdm command meta data %d",
				cmd_meta_data);
			return -EINVAL;
		}
		prepare->num_hw_update_entries = num_ent;
	}

	return rc;
}

int cam_sfe_add_command_buffers(
	struct cam_hw_prepare_update_args  *prepare,
	struct cam_kmd_buf_info            *kmd_buf_info,
	struct cam_isp_ctx_base_info       *base_info,
	cam_packet_generic_blob_handler     blob_handler_cb,
	struct cam_isp_hw_mgr_res          *res_list_sfe_out,
	uint8_t                            *out_map,
	uint32_t                            out_base,
	uint32_t                            out_max)
{
	int rc = 0;
	uint32_t                             cmd_meta_data, num_ent, i;
	enum cam_isp_hw_split_id             split_id;
	struct cam_cmd_buf_desc             *cmd_desc = NULL;
	struct cam_hw_update_entry          *hw_entry = NULL;

	split_id = base_info->split_id;
	hw_entry = prepare->hw_update_entries;

	/*
	 * set the cmd_desc to point the first command descriptor in the
	 * packet
	 */
	cmd_desc = (struct cam_cmd_buf_desc *)
			((uint8_t *)&prepare->packet->payload +
			prepare->packet->cmd_buf_offset);

	CAM_DBG(CAM_ISP, "split id = %d, number of command buffers:%d",
		split_id, prepare->packet->num_cmd_buf);

	for (i = 0; i < prepare->packet->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		num_ent = prepare->num_hw_update_entries;
		if (!cmd_desc[i].length)
			continue;

		/* One hw entry space required for left or right or common */
		if (num_ent + 1 >= prepare->max_hw_update_entries) {
			CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
				num_ent, prepare->max_hw_update_entries);
			return -EINVAL;
		}

		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		cmd_meta_data = cmd_desc[i].meta_data;

		CAM_DBG(CAM_ISP, "meta type: %d, split_id: %d",
			cmd_meta_data, split_id);

		switch (cmd_meta_data) {
		case CAM_ISP_SFE_PACKET_META_LEFT:
			if (split_id == CAM_ISP_HW_SPLIT_LEFT) {
				hw_entry[num_ent].len = cmd_desc[i].length;
				hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
				hw_entry[num_ent].offset = cmd_desc[i].offset;
				CAM_DBG(CAM_ISP,
					"Meta_Left num_ent=%d handle=0x%x, len=%u, offset=%u",
					num_ent,
					hw_entry[num_ent].handle,
					hw_entry[num_ent].len,
					hw_entry[num_ent].offset);
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent++;
			}
			break;
		case CAM_ISP_SFE_PACKET_META_RIGHT:
			if (split_id == CAM_ISP_HW_SPLIT_RIGHT) {
				hw_entry[num_ent].len = cmd_desc[i].length;
				hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
				hw_entry[num_ent].offset = cmd_desc[i].offset;
				CAM_DBG(CAM_ISP,
					"Meta_Right num_ent=%d handle=0x%x, len=%u, offset=%u",
					num_ent,
					hw_entry[num_ent].handle,
					hw_entry[num_ent].len,
					hw_entry[num_ent].offset);
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent++;
			}
			break;
		case CAM_ISP_SFE_PACKET_META_COMMON:
			hw_entry[num_ent].len = cmd_desc[i].length;
			hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
			hw_entry[num_ent].offset = cmd_desc[i].offset;
			CAM_DBG(CAM_ISP,
				"Meta_Common num_ent=%d handle=0x%x, len=%u, offset=%u",
				num_ent,
				hw_entry[num_ent].handle,
				hw_entry[num_ent].len,
				hw_entry[num_ent].offset);
			hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
			num_ent++;
			break;
		case CAM_ISP_SFE_PACKET_META_DUAL_CONFIG:
			rc = cam_isp_update_dual_config(&cmd_desc[i],
				split_id, base_info->idx,
				res_list_sfe_out, out_map, out_base, out_max);
			if (rc)
				return rc;
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_LEFT:
			if (split_id == CAM_ISP_HW_SPLIT_LEFT) {
				struct cam_isp_generic_blob_info   blob_info;

				blob_info.prepare = prepare;
				blob_info.base_info = base_info;
				blob_info.kmd_buf_info = kmd_buf_info;
				blob_info.entry_added = false;

				rc = cam_packet_util_process_generic_cmd_buffer(
					&cmd_desc[i],
					blob_handler_cb,
					&blob_info);
				if (rc) {
					CAM_ERR(CAM_ISP,
						"Failed in processing blobs %d",
						rc);
					return rc;
				}
				num_ent = prepare->num_hw_update_entries;
			}
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_RIGHT:
			if (split_id == CAM_ISP_HW_SPLIT_RIGHT) {
				struct cam_isp_generic_blob_info   blob_info;

				blob_info.prepare = prepare;
				blob_info.base_info = base_info;
				blob_info.kmd_buf_info = kmd_buf_info;
				blob_info.entry_added = false;

				rc = cam_packet_util_process_generic_cmd_buffer(
					&cmd_desc[i],
					blob_handler_cb,
					&blob_info);
				if (rc) {
					CAM_ERR(CAM_ISP,
						"Failed in processing blobs %d",
						rc);
					return rc;
				}
				num_ent = prepare->num_hw_update_entries;
			}
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_COMMON: {
			struct cam_isp_generic_blob_info   blob_info;

			blob_info.prepare = prepare;
			blob_info.base_info = base_info;
			blob_info.kmd_buf_info = kmd_buf_info;
			blob_info.entry_added = false;

			rc = cam_packet_util_process_generic_cmd_buffer(
				&cmd_desc[i],
				blob_handler_cb,
				&blob_info);
			if (rc) {
				CAM_ERR(CAM_ISP,
					"Failed in processing blobs %d", rc);
				return rc;
			}
			num_ent = prepare->num_hw_update_entries;
		}
			break;
		case CAM_ISP_PACKET_META_BASE:
		case CAM_ISP_PACKET_META_LEFT:
		case CAM_ISP_PACKET_META_RIGHT:
		case CAM_ISP_PACKET_META_COMMON:
		case CAM_ISP_PACKET_META_DMI_LEFT:
		case CAM_ISP_PACKET_META_DMI_RIGHT:
		case CAM_ISP_PACKET_META_DMI_COMMON:
		case CAM_ISP_PACKET_META_CLOCK:
		case CAM_ISP_PACKET_META_CSID:
		case CAM_ISP_PACKET_META_DUAL_CONFIG:
		case CAM_ISP_PACKET_META_REG_DUMP_PER_REQUEST:
		case CAM_ISP_PACKET_META_REG_DUMP_ON_FLUSH:
		case CAM_ISP_PACKET_META_REG_DUMP_ON_ERROR:
		case CAM_ISP_PACKET_META_CSID_LEFT:
		case CAM_ISP_PACKET_META_CSID_RIGHT:
		case CAM_ISP_PACKET_META_CSID_COMMON:
			break;
		default:
			CAM_ERR(CAM_ISP, "invalid cdm command meta data %d",
				cmd_meta_data);
			return -EINVAL;
		}
		prepare->num_hw_update_entries = num_ent;
	}

	return rc;
}

static void cam_isp_validate_for_sfe_scratch(
	struct cam_isp_sfe_scratch_buf_res_info *sfe_res_info,
	uint32_t res_type, uint32_t out_base)
{
	uint32_t res_id_out = res_type & 0xFF;

	if ((res_id_out) < ((out_base & 0xFF) +
		sfe_res_info->num_active_fe_rdis)) {
		CAM_DBG(CAM_ISP,
			"Buffer found for SFE port: 0x%x - skip scratch buffer",
			res_type);
		sfe_res_info->sfe_rdi_cfg_mask |= (1 << res_id_out);
	}
}

static void cam_isp_validate_for_ife_scratch(
	struct cam_isp_ife_scratch_buf_res_info *ife_res_info,
	uint32_t res_type)
{
	int i;

	for (i = 0; i < ife_res_info->num_ports; i++) {
		if (res_type == ife_res_info->ife_scratch_resources[i]) {
			CAM_DBG(CAM_ISP,
				"Buffer found for IFE port: 0x%x - skip scratch buffer",
				res_type);
			ife_res_info->ife_scratch_cfg_mask |= (1 << i);
		}
	}
}

static int cam_isp_io_buf_get_entries_util(
	struct cam_isp_io_buf_info              *buf_info,
	struct cam_buf_io_cfg                   *io_cfg,
	struct cam_isp_hw_mgr_res              **hw_mgr_res)
{
	uint32_t                                res_id;
	uint32_t                                num_entries;
	struct cam_hw_fence_map_entry          *map_entries  =  NULL;
	struct cam_isp_hw_mgr_res              *hw_mgr_res_temp;
	bool                                    found = false;

	CAM_DBG(CAM_REQ,
		"req_id %llu resource_type:%d fence:%d direction %d format %d",
		buf_info->prepare->packet->header.request_id,
		io_cfg->resource_type, io_cfg->fence,
		io_cfg->direction, io_cfg->format);

	if (io_cfg->direction == CAM_BUF_OUTPUT) {
		res_id = io_cfg->resource_type & 0xFF;

		if (io_cfg->resource_type < buf_info->out_base ||
			io_cfg->resource_type >= buf_info->out_max)
			return -ENOMSG;

		if ((buf_info->base->hw_type == CAM_ISP_HW_TYPE_SFE)  &&
				(buf_info->scratch_check_cfg->validate_for_sfe)) {
			cam_isp_validate_for_sfe_scratch(
				&buf_info->scratch_check_cfg->sfe_scratch_res_info,
				io_cfg->resource_type, buf_info->out_base);
		} else if ((buf_info->base->hw_type == CAM_ISP_HW_TYPE_VFE) &&
				(buf_info->scratch_check_cfg->validate_for_ife)) {
			cam_isp_validate_for_ife_scratch(
				&buf_info->scratch_check_cfg->ife_scratch_res_info,
				io_cfg->resource_type);
		}

		*hw_mgr_res = &buf_info->res_list_isp_out[buf_info->out_map[res_id]];
		if ((*hw_mgr_res)->res_type == CAM_ISP_RESOURCE_UNINT) {
			CAM_ERR(CAM_ISP, "io res id:%d not valid",
				io_cfg->resource_type);
			return -EINVAL;
		}
	} else if (io_cfg->direction == CAM_BUF_INPUT) {
		found = false;
		res_id = io_cfg->resource_type;
		if (!buf_info->res_list_in_rd) {
			CAM_DBG(CAM_ISP,
				"No BUS Read supported hw_type %d io_cfg %d req:%d type:%d fence:%d",
				buf_info->base->hw_type,
				buf_info->prepare->packet->num_io_configs,
				buf_info->prepare->packet->header.request_id,
				io_cfg->resource_type, io_cfg->fence);
			return -ENOMSG;
		}
		if (buf_info->base->hw_type != CAM_ISP_HW_TYPE_SFE)
			res_id = CAM_ISP_HW_VFE_IN_RD;

		list_for_each_entry_safe((*hw_mgr_res), hw_mgr_res_temp,
			buf_info->res_list_in_rd, list) {
			if ((*hw_mgr_res)->res_id == res_id) {
				found = true;
				break;
			}
		}

		if (!found) {
			CAM_ERR(CAM_ISP, "No Read resource");
			return -EINVAL;
		}

	} else {
		CAM_ERR(CAM_ISP, "Invalid io config direction :%d",
				io_cfg->direction);
		return -EINVAL;
	}

	if (buf_info->fill_fence) {
		if (io_cfg->direction == CAM_BUF_OUTPUT &&
			(buf_info->prepare->num_out_map_entries <
				 buf_info->prepare->max_out_map_entries)) {
			num_entries = buf_info->prepare->num_out_map_entries;
			map_entries = &buf_info->prepare->out_map_entries[num_entries];
			buf_info->prepare->num_out_map_entries++;
		} else if (io_cfg->direction == CAM_BUF_INPUT &&
			(buf_info->prepare->num_in_map_entries <
				 buf_info->prepare->max_in_map_entries)) {
			num_entries = buf_info->prepare->num_in_map_entries;
			map_entries = &buf_info->prepare->in_map_entries[num_entries];
			buf_info->prepare->num_in_map_entries++;

		} else {
			CAM_ERR(CAM_ISP, "dir: %d max_ln:%d max_out: %u in: %u out %u",
				io_cfg->direction,
				buf_info->prepare->max_in_map_entries,
				buf_info->prepare->max_out_map_entries,
				buf_info->prepare->num_in_map_entries,
				buf_info->prepare->num_out_map_entries);
			return -EINVAL;
		}
		map_entries->resource_handle = io_cfg->resource_type;
		map_entries->sync_id = io_cfg->fence;
	}

	return 0;
}

static int cam_isp_add_io_buffers_util(
	struct cam_isp_io_buf_info       *buf_info,
	struct cam_buf_io_cfg            *io_cfg,
	struct cam_isp_resource_node     *res)
{
	uint32_t                            secure_mode_cmd;
	uint32_t                            bus_update_cmd;
	int                                 rc = 0;
	dma_addr_t                          io_addr[CAM_PACKET_MAX_PLANES];
	struct cam_isp_hw_get_cmd_update    update_buf;
	struct cam_isp_hw_get_wm_update     bus_port_update;
	struct cam_hw_fence_map_entry      *out_map_entry = NULL;
	struct cam_smmu_buffer_tracker     *old_head_entry, *new_head_entry;
	uint32_t                            kmd_buf_remain_size;
	uint32_t                            plane_id, num_entries;
	dma_addr_t                         *image_buf_addr;
	uint32_t                           *image_buf_offset;
	size_t                              size;
	int                                 mmu_hdl;
	bool                                is_buf_secure;
	struct cam_isp_hw_get_cmd_update    secure_mode;
	uint32_t                            mode = 0;

	if (io_cfg->direction == CAM_BUF_OUTPUT) {
		secure_mode_cmd = CAM_ISP_HW_CMD_GET_WM_SECURE_MODE;
		bus_update_cmd = CAM_ISP_HW_CMD_GET_BUF_UPDATE;
	} else if (io_cfg->direction == CAM_BUF_INPUT) {
		secure_mode_cmd = CAM_ISP_HW_CMD_GET_RM_SECURE_MODE;
		bus_update_cmd = CAM_ISP_HW_CMD_GET_BUF_UPDATE_RM;

	} else {
		CAM_ERR(CAM_ISP, "Invalid dir %d", io_cfg->direction);
		return -EINVAL;
	}

	if (!res) {
		CAM_ERR(CAM_ISP, "invalid res, io res id:%d split_id :%d",
			io_cfg->resource_type, buf_info->base->split_id);
		return -EINVAL;
	}

	if (res->res_id != io_cfg->resource_type) {
		CAM_ERR(CAM_ISP, "err res id:%d io res id:%d",
			res->res_id, io_cfg->resource_type);
		return -EINVAL;
	}

	old_head_entry = list_first_entry_or_null(buf_info->prepare->buf_tracker,
		struct cam_smmu_buffer_tracker, list);

	memset(io_addr, 0, sizeof(io_addr));
	for (plane_id = 0; plane_id < CAM_PACKET_MAX_PLANES; plane_id++) {
		if (!io_cfg->mem_handle[plane_id])
			break;
		secure_mode.cmd_type = secure_mode_cmd;
		secure_mode.res = res;
		secure_mode.data = (void *)&mode;
		rc = res->hw_intf->hw_ops.process_cmd(
			res->hw_intf->hw_priv, secure_mode_cmd,
			&secure_mode, sizeof(struct cam_isp_hw_get_cmd_update));
		if (rc) {
			CAM_ERR(CAM_ISP, "Get secure mode failed cmd_type %d res_id %d",
				secure_mode_cmd, res->res_id);
			return -EINVAL;
		}

		is_buf_secure = cam_mem_is_secure_buf(io_cfg->mem_handle[plane_id]);

		if ((mode  == CAM_SECURE_MODE_SECURE) &&
			is_buf_secure) {
			mmu_hdl = buf_info->sec_iommu_hdl;
		} else if ((mode == CAM_SECURE_MODE_NON_SECURE) &&
			!is_buf_secure) {
			mmu_hdl = buf_info->iommu_hdl;
		} else {
			CAM_ERR_RATE_LIMIT(CAM_ISP,
				"Invalid hdl: port mode[%u], buf mode[%u]",
				mode, is_buf_secure);
			return -EINVAL;
		}

		rc = cam_mem_get_io_buf(io_cfg->mem_handle[plane_id],
			mmu_hdl, &io_addr[plane_id], &size, NULL,
			(!plane_id && (io_cfg->direction == CAM_BUF_OUTPUT)) ?
			buf_info->prepare->buf_tracker : NULL);
		if (rc) {
			CAM_ERR(CAM_ISP, "no io addr for plane%d", plane_id);
			rc = -ENOMEM;
			return rc;
		}

		/* need to update with offset */
		io_addr[plane_id] += io_cfg->offsets[plane_id];
		CAM_DBG(CAM_ISP, "get io_addr for plane %d: 0x%llx, mem_hdl=0x%x",
			plane_id, io_addr[plane_id], io_cfg->mem_handle[plane_id]);

		CAM_DBG(CAM_ISP, "mmu_hdl=0x%x, size=%d, end=0x%x",
			mmu_hdl, (int)size, io_addr[plane_id]+size);
	}

	if (!plane_id) {
		CAM_ERR(CAM_ISP, "No valid planes for res%d", res->res_id);
		rc = -ENOMEM;
		return rc;
	}

	if ((buf_info->kmd_buf_info->used_bytes) <
		buf_info->kmd_buf_info->size) {
		kmd_buf_remain_size = buf_info->kmd_buf_info->size -
			(buf_info->kmd_buf_info->used_bytes);
	} else {
		CAM_ERR(CAM_ISP, "no free kmd memory for base %d", buf_info->base->idx);
		rc = -ENOMEM;
		return rc;
	}
	update_buf.res = res;
	update_buf.cmd_type = bus_update_cmd;
	update_buf.cmd.cmd_buf_addr = buf_info->kmd_buf_info->cpu_addr +
		buf_info->kmd_buf_info->used_bytes/4;
	bus_port_update.image_buf = io_addr;
	bus_port_update.num_buf   = plane_id;
	bus_port_update.io_cfg    = io_cfg;
	bus_port_update.frame_header = 0;
	bus_port_update.fh_enabled = false;

	for (plane_id = 0; plane_id < CAM_PACKET_MAX_PLANES; plane_id++)
		bus_port_update.image_buf_offset[plane_id] = 0;

	if ((buf_info->frame_hdr->frame_header_enable) &&
			!(buf_info->frame_hdr->frame_header_res_id)) {
		bus_port_update.frame_header =
			buf_info->frame_hdr->frame_header_iova_addr;
		bus_port_update.local_id =
			buf_info->prepare->packet->header.request_id;
	}

	update_buf.cmd.size = kmd_buf_remain_size;
	if (io_cfg->direction == CAM_BUF_OUTPUT)
		update_buf.wm_update = &bus_port_update;
	else if (io_cfg->direction == CAM_BUF_INPUT)
		update_buf.rm_update = &bus_port_update;

	update_buf.use_scratch_cfg = false;

	CAM_DBG(CAM_ISP, "cmd buffer 0x%pK, size %d",
		update_buf.cmd.cmd_buf_addr, update_buf.cmd.size);
	rc = res->hw_intf->hw_ops.process_cmd(
		res->hw_intf->hw_priv,
		bus_update_cmd, &update_buf,
		sizeof(struct cam_isp_hw_get_cmd_update));

	if (rc) {
		CAM_ERR(CAM_ISP, "get buf cmd error:%d",
				res->res_id);
		rc = -ENOMEM;
		return rc;
	}

	if (bus_port_update.fh_enabled) {
		buf_info->frame_hdr->frame_header_res_id = res->res_id;
		CAM_DBG(CAM_ISP,
			"Frame header enabled for res: 0x%x iova: %pK",
			buf_info->frame_hdr->frame_header_res_id,
			bus_port_update.frame_header);
	}

	buf_info->kmd_buf_info->used_bytes += update_buf.cmd.used_bytes;
	buf_info->kmd_buf_info->offset += update_buf.cmd.used_bytes;

	if (io_cfg->direction == CAM_BUF_OUTPUT) {
		if (buf_info->fill_fence) {
			num_entries = buf_info->prepare->num_out_map_entries - 1;
			out_map_entry =
				&buf_info->prepare->out_map_entries[num_entries];
			if (!out_map_entry) {
				CAM_ERR(CAM_ISP, "out_map_entry is NULL");
				rc = -EINVAL;
				return rc;
			}

			image_buf_addr = out_map_entry->image_buf_addr;
			image_buf_offset = bus_port_update.image_buf_offset;
			if (buf_info->base->split_id == CAM_ISP_HW_SPLIT_LEFT) {
				for (plane_id = 0; plane_id < CAM_PACKET_MAX_PLANES;
					plane_id++)
					image_buf_addr[plane_id] = io_addr[plane_id] +
						image_buf_offset[plane_id];
			}

			new_head_entry =
				list_first_entry_or_null(buf_info->prepare->buf_tracker,
					struct cam_smmu_buffer_tracker, list);
			if (new_head_entry && old_head_entry != new_head_entry) {
				out_map_entry->buffer_tracker = new_head_entry;
				CAM_DBG(CAM_ISP,
					"[SMMU_BT] Tracking io_buf, buf_handle: 0x%x, fd: 0x%x, res_id: %d",
					io_cfg->mem_handle[0],
					out_map_entry->buffer_tracker->ion_fd, res->res_id);
			}
		}
	}

	return rc;
}

static int cam_isp_add_io_buffers_mc(
	uint64_t                   *mc_io_cfg,
	struct cam_isp_io_buf_info *io_info,
	uint8_t                     num_ports,
	uint32_t                    ctxt_id)
{
	uint32_t                        bytes_used;
	uint32_t                        kmd_buf_remain_size;
	uint32_t                       *cmd_buf_addr;
	struct cam_isp_hw_mgr_res      *hw_mgr_res = NULL;
	struct cam_isp_resource_node   *res = NULL;
	uint8_t                         max_out = 0;
	int                             rc = 0;
	int                             i;

	if (io_info->kmd_buf_info->used_bytes < io_info->kmd_buf_info->size) {
		kmd_buf_remain_size = io_info->kmd_buf_info->size -
			io_info->kmd_buf_info->used_bytes;
	} else {
		CAM_ERR(CAM_ISP,
			"no free kmd memory for base=%d bytes_used=%u buf_size=%u",
			io_info->base->idx, io_info->kmd_buf_info->used_bytes,
			io_info->kmd_buf_info->size);
		rc = -ENOMEM;
		return rc;
	}

	cmd_buf_addr = io_info->kmd_buf_info->cpu_addr +
		io_info->kmd_buf_info->used_bytes/4;
	rc = cam_isp_add_cmd_buf_update(
		NULL, io_info->hw_intf,
		CAM_ISP_HW_CMD_MC_CTXT_SEL,
		CAM_ISP_HW_CMD_MC_CTXT_SEL,
		(void *)cmd_buf_addr,
		kmd_buf_remain_size,
		(void *)(&ctxt_id),
		&bytes_used);

	if (rc) {
		CAM_ERR(CAM_ISP, "Adding MC context[%u] failed for base[%d]",
			ctxt_id, io_info->base->idx);
		return rc;
	}

	io_info->kmd_buf_info->used_bytes += bytes_used;
	io_info->kmd_buf_info->offset += bytes_used;

	max_out = io_info->out_max;
	for (i = 0; i < num_ports; i++) {
		rc = cam_isp_io_buf_get_entries_util(io_info,
			(struct cam_buf_io_cfg *)mc_io_cfg[(max_out * ctxt_id) + i], &hw_mgr_res);

		if (!hw_mgr_res) {
			CAM_ERR(CAM_ISP, "hw_mgr res is NULL");
			return -EINVAL;
		}

		res = hw_mgr_res->hw_res[io_info->base->split_id];

		if (!res)
			continue;

		rc = cam_isp_add_io_buffers_util(io_info,
			(struct cam_buf_io_cfg *)mc_io_cfg[(max_out * ctxt_id) + i], res);

		if (rc) {
			CAM_ERR(CAM_ISP, "ctxt[%d] io_cfg[%d] add buf failed rc %d",
				ctxt_id, i, rc);
			return rc;
		}
	}

	return rc;
}

int cam_isp_add_io_buffers(struct cam_isp_io_buf_info   *io_info)
{
	int                                 rc = 0;
	struct cam_buf_io_cfg              *io_cfg = NULL;
	struct cam_isp_hw_mgr_res          *hw_mgr_res = NULL;
	uint32_t                            i;
	uint32_t                            curr_used_bytes = 0;
	uint32_t                            bytes_updated = 0;
	struct cam_isp_resource_node       *res = NULL;
	int                                 ctxt_id = 0;
	uint8_t                             num_ports[CAM_ISP_MULTI_CTXT_MAX] = {0};
	uint8_t                             max_out_res = 0;
	uint64_t                           *mc_cfg = NULL;
	uint32_t                            major_version = 0;

	io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&io_info->prepare->packet->payload +
			io_info->prepare->packet->io_configs_offset);
	curr_used_bytes = io_info->kmd_buf_info->used_bytes;

	/* Max one hw entries required for each base */
	if (io_info->prepare->num_hw_update_entries + 1 >=
			io_info->prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
			io_info->prepare->num_hw_update_entries,
			io_info->prepare->max_hw_update_entries);
		return -EINVAL;
	}

	max_out_res = io_info->out_max & 0xFF;
	major_version = io_info->major_version;

	if (major_version == 3) {
		mc_cfg = vzalloc(sizeof(uint64_t) * CAM_ISP_MULTI_CTXT_MAX * (max_out_res));
		if (!mc_cfg) {
			CAM_ERR(CAM_ISP, "Memory allocation failed for MC cases");
			return -ENOMEM;
		}
	}

	for (i = 0; i < io_info->prepare->packet->num_io_configs; i++) {

		if (major_version == 3) {
			ctxt_id = ffs(io_cfg[i].flag) - 1;
			if (ctxt_id < 0) {
				CAM_ERR(CAM_ISP,
					"Invalid ctxt_id %d req_id %llu resource_type:%d",
					ctxt_id, io_info->prepare->packet->header.request_id,
					io_cfg[i].resource_type);
				rc = -EINVAL;
				goto err;
			}

			mc_cfg[(max_out_res * ctxt_id) + num_ports[ctxt_id]] = (uint64_t)&io_cfg[i];
			num_ports[ctxt_id]++;
		} else {
			rc = cam_isp_io_buf_get_entries_util(io_info, &io_cfg[i], &hw_mgr_res);
			if (rc == -ENOMSG) {
				rc = 0;
				continue;
			} else if (rc) {
				CAM_ERR(CAM_ISP, "io_cfg[%d] failed rc %d", i, rc);
				return rc;
			}

			if (!hw_mgr_res) {
				CAM_ERR(CAM_ISP, "hw_mgr res is NULL");
				return -EINVAL;
			}

			res = hw_mgr_res->hw_res[io_info->base->split_id];
			if (!res)
				continue;

			rc = cam_isp_add_io_buffers_util(io_info, &io_cfg[i], res);
			if (rc) {
				CAM_ERR(CAM_ISP, "io_cfg[%d] add buf failed rc %d", i, rc);
				return rc;
			}
		}
	}

	if (major_version == 3) {
		for (i = 0; i < CAM_ISP_MULTI_CTXT_MAX; i++) {
			if (!num_ports[i])
				continue;
			rc = cam_isp_add_io_buffers_mc(mc_cfg, io_info, num_ports[i], i);
			if (rc) {
				CAM_ERR(CAM_ISP, "MC context[%u] failed for base[%d]",
					i, io_info->base->idx);
				goto err;
			}
		}
		vfree(mc_cfg);
	}

	bytes_updated = io_info->kmd_buf_info->used_bytes - curr_used_bytes;
	CAM_DBG(CAM_ISP, "io_cfg_used_bytes %d, fill_fence %d",
		bytes_updated, io_info->fill_fence);

	if (bytes_updated) {
		/**
		 * Offset and used_bytes are already updated in the previous
		 * add_io_buffers_util function and now points to next empty
		 * block in kmd buffer, reset it here to align with generic
		 * update hw entry function
		 */
		io_info->kmd_buf_info->offset -= bytes_updated;
		io_info->kmd_buf_info->used_bytes -= bytes_updated;
		cam_isp_update_hw_entry(CAM_ISP_IOCFG_BL, io_info->prepare,
			io_info->kmd_buf_info, bytes_updated, false);
	}

	return rc;
err:
	vfree(mc_cfg);
	return rc;
}

int cam_isp_add_reg_update(
	struct cam_hw_prepare_update_args    *prepare,
	struct list_head                     *res_list_isp_src,
	uint32_t                              base_idx,
	struct cam_kmd_buf_info              *kmd_buf_info,
	bool                                  combine)
{
	int rc = -EINVAL;
	struct cam_isp_resource_node         *res;
	struct cam_isp_hw_mgr_res            *hw_mgr_res;
	struct cam_isp_hw_get_cmd_update      get_regup;
	uint32_t kmd_buf_remain_size, i, reg_update_size;

	/* Max one hw entries required for each base */
	if (prepare->num_hw_update_entries + 1 >=
		prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
			prepare->num_hw_update_entries,
			prepare->max_hw_update_entries);
		return -EINVAL;
	}

	reg_update_size = 0;
	list_for_each_entry(hw_mgr_res, res_list_isp_src, list) {
		if (hw_mgr_res->res_type == CAM_ISP_RESOURCE_UNINT)
			continue;

		for (i = 0; i < CAM_ISP_HW_SPLIT_MAX; i++) {
			if (!hw_mgr_res->hw_res[i])
				continue;

			res = hw_mgr_res->hw_res[i];
			if (res->hw_intf->hw_idx != base_idx)
				continue;

			if (kmd_buf_info->size > (kmd_buf_info->used_bytes +
				reg_update_size)) {
				kmd_buf_remain_size =  kmd_buf_info->size -
					(kmd_buf_info->used_bytes +
					reg_update_size);
			} else {
				CAM_ERR(CAM_ISP, "no free mem %d %d %d",
					base_idx, kmd_buf_info->size,
					kmd_buf_info->used_bytes +
					reg_update_size);
				rc = -EINVAL;
				return rc;
			}

			get_regup.cmd.cmd_buf_addr = kmd_buf_info->cpu_addr +
				kmd_buf_info->used_bytes/4 +
				reg_update_size/4;
			get_regup.cmd.size = kmd_buf_remain_size;
			get_regup.cmd_type = CAM_ISP_HW_CMD_GET_REG_UPDATE;
			get_regup.res = res;

			rc = res->hw_intf->hw_ops.process_cmd(
				res->hw_intf->hw_priv,
				CAM_ISP_HW_CMD_GET_REG_UPDATE, &get_regup,
				sizeof(struct cam_isp_hw_get_cmd_update));
			if (rc)
				return rc;

			CAM_DBG(CAM_ISP,
				"Reg update added for res %d hw_id %d cdm_idx %d",
				res->res_id, res->hw_intf->hw_idx, base_idx);
			reg_update_size += get_regup.cmd.used_bytes;
		}
	}

	if (reg_update_size) {
		cam_isp_update_hw_entry(CAM_ISP_COMMON_CFG_BL, prepare,
			kmd_buf_info, reg_update_size, combine);
		rc = 0;
	}

	return rc;
}

int cam_isp_add_go_cmd(
	struct cam_hw_prepare_update_args    *prepare,
	struct cam_isp_resource_node         *res,
	struct cam_kmd_buf_info              *kmd_buf_info,
	bool                                  combine)
{
	int rc;
	struct cam_isp_hw_get_cmd_update      get_regup;
	uint32_t kmd_buf_remain_size, reg_update_size;

	/* Max one hw entries required for each base */
	if (prepare->num_hw_update_entries + 1 >=
		prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
			prepare->num_hw_update_entries,
			prepare->max_hw_update_entries);
		return -EINVAL;
	}

	reg_update_size = 0;
	if (kmd_buf_info->size > (kmd_buf_info->used_bytes +
		reg_update_size)) {
		kmd_buf_remain_size =  kmd_buf_info->size -
			(kmd_buf_info->used_bytes +
			reg_update_size);
	} else {
		CAM_ERR(CAM_ISP, "no free mem %d %d",
			kmd_buf_info->size,
			kmd_buf_info->used_bytes +
			reg_update_size);
		return -EINVAL;
	}

	get_regup.cmd.cmd_buf_addr = kmd_buf_info->cpu_addr +
		kmd_buf_info->used_bytes/4 +
		reg_update_size/4;
	get_regup.cmd.size = kmd_buf_remain_size;
	get_regup.cmd_type = CAM_ISP_HW_CMD_FE_TRIGGER_CMD;
	get_regup.res = res;

	rc = res->process_cmd(res->res_priv,
		CAM_ISP_HW_CMD_FE_TRIGGER_CMD, &get_regup,
		sizeof(struct cam_isp_hw_get_cmd_update));
	if (rc)
		return rc;

	CAM_DBG(CAM_ISP, "GO_CMD added for RD res %d hw_id %d",
		res->res_type, res->hw_intf->hw_idx);
	reg_update_size += get_regup.cmd.used_bytes;

	/* Update HW entries */
	if (reg_update_size)
		cam_isp_update_hw_entry(CAM_ISP_COMMON_CFG_BL,
			prepare, kmd_buf_info, reg_update_size,
			combine);
	return 0;
}

int cam_isp_add_comp_wait(
	struct cam_hw_prepare_update_args    *prepare,
	struct list_head                     *res_list_isp_src,
	uint32_t                              base_idx,
	struct cam_kmd_buf_info              *kmd_buf_info)
{
	int rc = -EINVAL;
	struct cam_isp_hw_mgr_res            *hw_mgr_res;
	struct cam_isp_hw_get_cmd_update      add_wait;
	struct cam_hw_intf                   *hw_intf;
	bool                                  hw_res_valid = false;
	uint32_t kmd_buf_remain_size, num_ent, add_wait_size;

	/* Max one hw entries required for each base */
	if (prepare->num_hw_update_entries >= prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries");
		return -EINVAL;
	}

	add_wait_size = 0;
	list_for_each_entry(hw_mgr_res, res_list_isp_src, list) {
		if (hw_mgr_res->res_type == CAM_ISP_RESOURCE_UNINT)
			continue;
		if (hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_LEFT] &&
			hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_RIGHT]) {
			hw_res_valid = true;
			break;
		}
	}

	if (!hw_mgr_res || !hw_res_valid) {
		CAM_ERR(CAM_ISP, "No src with multi_vfe config");
		return -EINVAL;
	}

	hw_intf = hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_LEFT]->hw_intf;

	if (kmd_buf_info->size > (kmd_buf_info->used_bytes +
			add_wait_size)) {
		kmd_buf_remain_size =  kmd_buf_info->size -
			(kmd_buf_info->used_bytes + add_wait_size);
	} else {
		CAM_ERR(CAM_ISP, "no free mem %d %d %d",
			base_idx, kmd_buf_info->size,
			kmd_buf_info->used_bytes +
			add_wait_size);
		rc = -EINVAL;
		return rc;
	}

	add_wait.cmd.cmd_buf_addr = kmd_buf_info->cpu_addr +
		kmd_buf_info->used_bytes/4 +
		add_wait_size/4;
	add_wait.cmd.size = kmd_buf_remain_size;
	add_wait.cmd_type = CAM_ISP_HW_CMD_ADD_WAIT;
	add_wait.res = hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_LEFT];

	rc = hw_intf->hw_ops.process_cmd(
		hw_intf->hw_priv,
		CAM_ISP_HW_CMD_ADD_WAIT,
		&add_wait,
		sizeof(struct cam_isp_hw_get_cmd_update));

	if (rc) {
		CAM_ERR(CAM_ISP,
			"wait_comp_event addition failed for dual vfe");
		return rc;
	}

	add_wait_size += add_wait.cmd.used_bytes;
	if (add_wait_size) {
		/* Update the HW entries */
		num_ent = prepare->num_hw_update_entries;
		prepare->hw_update_entries[num_ent].handle =
			kmd_buf_info->handle;
		prepare->hw_update_entries[num_ent].len = add_wait_size;
		prepare->hw_update_entries[num_ent].offset =
			kmd_buf_info->offset;
		prepare->hw_update_entries[num_ent].flags = CAM_ISP_IOCFG_BL;
		CAM_DBG(CAM_ISP,
			"num_ent=%d handle=0x%x, len=%u, offset=%u",
			num_ent,
			prepare->hw_update_entries[num_ent].handle,
			prepare->hw_update_entries[num_ent].len,
			prepare->hw_update_entries[num_ent].offset);
		num_ent++;

		kmd_buf_info->used_bytes += add_wait_size;
		kmd_buf_info->offset     += add_wait_size;
		prepare->num_hw_update_entries = num_ent;
		/* add wait_comp_event is success return status 0 */
		rc = 0;
	}

	return rc;
}

int cam_isp_add_wait_trigger(
	struct cam_hw_prepare_update_args    *prepare,
	struct list_head                     *res_list_isp_src,
	uint32_t                              base_idx,
	struct cam_kmd_buf_info              *kmd_buf_info,
	bool                                  trigger_cdm_en)
{
	int rc = -EINVAL;
	struct cam_isp_hw_mgr_res            *hw_mgr_res;
	struct cam_isp_hw_get_cmd_update      add_trigger;
	struct cam_hw_intf                   *hw_intf;
	bool                                  hw_res_valid = false;
	uint32_t kmd_buf_remain_size, num_ent, add_trigger_size;

	/* Max one hw entries required for each base */
	if (prepare->num_hw_update_entries + 1 >=
				prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries");
		return -EINVAL;
	}

	add_trigger_size = 0;

	list_for_each_entry(hw_mgr_res, res_list_isp_src, list) {
		if (hw_mgr_res->res_type == CAM_ISP_RESOURCE_UNINT)
			continue;
		if (hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_LEFT] &&
			hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_RIGHT]) {
			hw_res_valid = true;
			break;
		}
	}

	if (!hw_mgr_res || !hw_res_valid) {
		CAM_ERR(CAM_ISP, "No src with multi_vfe config");
		return -EINVAL;
	}

	hw_intf = hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_RIGHT]->hw_intf;

	if (kmd_buf_info->size > (kmd_buf_info->used_bytes +
			add_trigger_size)) {
		kmd_buf_remain_size =  kmd_buf_info->size -
			(kmd_buf_info->used_bytes + add_trigger_size);
	} else {
		CAM_ERR(CAM_ISP, "no free mem %d %d %d",
			base_idx, kmd_buf_info->size,
			kmd_buf_info->used_bytes +
			add_trigger_size);
		rc = -EINVAL;
		return rc;
	}

	add_trigger.cmd.cmd_buf_addr = kmd_buf_info->cpu_addr +
		kmd_buf_info->used_bytes/4 +
		add_trigger_size/4;
	add_trigger.cmd.size = kmd_buf_remain_size;
	add_trigger.cmd_type = CAM_ISP_HW_CMD_ADD_WAIT_TRIGGER;
	add_trigger.res = hw_mgr_res->hw_res[CAM_ISP_HW_SPLIT_RIGHT];

	rc = hw_intf->hw_ops.process_cmd(
		hw_intf->hw_priv,
		CAM_ISP_HW_CMD_ADD_WAIT_TRIGGER,
		&add_trigger,
		sizeof(struct cam_isp_hw_get_cmd_update));

	if (rc) {
		CAM_ERR(CAM_ISP,
			"wait_trigger_event addition failed for dual vfe");
		return rc;
	}

	add_trigger_size += add_trigger.cmd.used_bytes;

	if (add_trigger_size) {
		/* Update the HW entries */
		num_ent = prepare->num_hw_update_entries;
		prepare->hw_update_entries[num_ent].handle =
			kmd_buf_info->handle;
		prepare->hw_update_entries[num_ent].len = add_trigger_size;
		prepare->hw_update_entries[num_ent].offset =
			kmd_buf_info->offset;
		prepare->hw_update_entries[num_ent].flags = CAM_ISP_IOCFG_BL;
		CAM_DBG(CAM_ISP,
			"num_ent=%d handle=0x%x, len=%u, offset=%u",
			num_ent,
			prepare->hw_update_entries[num_ent].handle,
			prepare->hw_update_entries[num_ent].len,
			prepare->hw_update_entries[num_ent].offset);
		num_ent++;

		kmd_buf_info->used_bytes += add_trigger_size;
		kmd_buf_info->offset     += add_trigger_size;
		prepare->num_hw_update_entries = num_ent;
		/* add wait trigger is success return status 0 */
		rc = 0;
	}

	return rc;
}

int cam_isp_add_csid_command_buffers(
	struct cam_hw_prepare_update_args   *prepare,
	struct cam_kmd_buf_info             *kmd_buf_info,
	cam_packet_generic_blob_handler     blob_handler_cb,
	struct cam_isp_ctx_base_info        *base_info)
{
	int rc = 0;
	uint32_t                           cmd_meta_data, num_ent, i;
	enum cam_isp_hw_split_id           split_id;
	struct cam_cmd_buf_desc           *cmd_desc = NULL;
	struct cam_hw_update_entry        *hw_entry = NULL;

	split_id = base_info->split_id;
	hw_entry = prepare->hw_update_entries;

	/*
	 * set the cmd_desc to point the first command descriptor in the
	 * packet
	 */
	cmd_desc = (struct cam_cmd_buf_desc *)
			((uint8_t *)&prepare->packet->payload +
			prepare->packet->cmd_buf_offset);

	CAM_DBG(CAM_ISP, "split id = %d, number of command buffers:%d",
		split_id, prepare->packet->num_cmd_buf);

	for (i = 0; i < prepare->packet->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		num_ent = prepare->num_hw_update_entries;
		if (!cmd_desc[i].length)
			continue;

		/* One hw entry space required for left or right or common */
		if (num_ent + 1 >= prepare->max_hw_update_entries) {
			CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
				num_ent, prepare->max_hw_update_entries);
			return -EINVAL;
		}

		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		cmd_meta_data = cmd_desc[i].meta_data;

		CAM_DBG(CAM_ISP, "meta type: %d, split_id: %d",
			cmd_meta_data, split_id);

		switch (cmd_meta_data) {
		case CAM_ISP_PACKET_META_BASE:
		case CAM_ISP_PACKET_META_LEFT:
		case CAM_ISP_PACKET_META_DMI_LEFT:
		case CAM_ISP_PACKET_META_RIGHT:
		case CAM_ISP_PACKET_META_DMI_RIGHT:
		case CAM_ISP_PACKET_META_COMMON:
		case CAM_ISP_PACKET_META_DMI_COMMON:
		case CAM_ISP_PACKET_META_DUAL_CONFIG:
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_LEFT:
			if (split_id == CAM_ISP_HW_SPLIT_LEFT) {
				struct cam_isp_generic_blob_info   blob_info;

				prepare->num_hw_update_entries = num_ent;
				blob_info.prepare = prepare;
				blob_info.base_info = base_info;
				blob_info.kmd_buf_info = kmd_buf_info;

				rc = cam_packet_util_process_generic_cmd_buffer(
					&cmd_desc[i],
					blob_handler_cb,
					&blob_info);
				if (rc) {
					CAM_ERR(CAM_ISP,
						"Failed in processing blobs %d",
						rc);
					return rc;
				}
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent = prepare->num_hw_update_entries;
			}
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_RIGHT:
			if (split_id == CAM_ISP_HW_SPLIT_RIGHT) {
				struct cam_isp_generic_blob_info   blob_info;

				prepare->num_hw_update_entries = num_ent;
				blob_info.prepare = prepare;
				blob_info.base_info = base_info;
				blob_info.kmd_buf_info = kmd_buf_info;

				rc = cam_packet_util_process_generic_cmd_buffer(
					&cmd_desc[i],
					blob_handler_cb,
					&blob_info);
				if (rc) {
					CAM_ERR(CAM_ISP,
						"Failed in processing blobs %d",
						rc);
					return rc;
				}
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent = prepare->num_hw_update_entries;
			}
			break;
		case CAM_ISP_PACKET_META_GENERIC_BLOB_COMMON: {
			struct cam_isp_generic_blob_info   blob_info;

			prepare->num_hw_update_entries = num_ent;
			blob_info.prepare = prepare;
			blob_info.base_info = base_info;
			blob_info.kmd_buf_info = kmd_buf_info;

			rc = cam_packet_util_process_generic_cmd_buffer(
				&cmd_desc[i],
				blob_handler_cb,
				&blob_info);
			if (rc) {
				CAM_ERR(CAM_ISP,
					"Failed in processing blobs %d", rc);
				return rc;
			}
			hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
			num_ent = prepare->num_hw_update_entries;
		}
			break;
		case CAM_ISP_PACKET_META_REG_DUMP_ON_FLUSH:
		case CAM_ISP_PACKET_META_REG_DUMP_ON_ERROR:
		case CAM_ISP_PACKET_META_REG_DUMP_PER_REQUEST:
		case CAM_ISP_SFE_PACKET_META_LEFT:
		case CAM_ISP_SFE_PACKET_META_RIGHT:
		case CAM_ISP_SFE_PACKET_META_COMMON:
		case CAM_ISP_SFE_PACKET_META_DUAL_CONFIG:
			break;
		case CAM_ISP_PACKET_META_CSID_LEFT:
			if (split_id == CAM_ISP_HW_SPLIT_LEFT) {
				hw_entry[num_ent].len = cmd_desc[i].length;
				hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
				hw_entry[num_ent].offset = cmd_desc[i].offset;
				CAM_DBG(CAM_ISP,
					"Meta_Left num_ent=%d handle=0x%x, len=%u, offset=%u",
					num_ent,
					hw_entry[num_ent].handle,
					hw_entry[num_ent].len,
					hw_entry[num_ent].offset);
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent++;
			}
			break;
		case CAM_ISP_PACKET_META_CSID_RIGHT:
			if (split_id == CAM_ISP_HW_SPLIT_RIGHT) {
				hw_entry[num_ent].len = cmd_desc[i].length;
				hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
				hw_entry[num_ent].offset = cmd_desc[i].offset;
				CAM_DBG(CAM_ISP,
					"Meta_Right num_ent=%d handle=0x%x, len=%u, offset=%u",
					num_ent,
					hw_entry[num_ent].handle,
					hw_entry[num_ent].len,
					hw_entry[num_ent].offset);
				hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
				num_ent++;
			}
			break;
		case CAM_ISP_PACKET_META_CSID_COMMON:
			hw_entry[num_ent].len = cmd_desc[i].length;
			hw_entry[num_ent].handle = cmd_desc[i].mem_handle;
			hw_entry[num_ent].offset = cmd_desc[i].offset;
			CAM_DBG(CAM_ISP,
				"Meta_Common num_ent=%d handle=0x%x, len=%u, offset=%u",
				num_ent,
				hw_entry[num_ent].handle,
				hw_entry[num_ent].len,
				hw_entry[num_ent].offset);
			hw_entry[num_ent].flags = CAM_ISP_IQ_BL;
			num_ent++;
			break;
		default:
			CAM_ERR(CAM_ISP, "invalid cdm command meta data %d",
				cmd_meta_data);
			return -EINVAL;
		}
		prepare->num_hw_update_entries = num_ent;
	}

	return rc;
}

int cam_isp_add_csid_reg_update(
	struct cam_hw_prepare_update_args    *prepare,
	struct cam_kmd_buf_info              *kmd_buf_info,
	void                                 *args,
	bool                                  combine)
{
	int rc;
	struct cam_isp_resource_node         *res;
	uint32_t kmd_buf_remain_size, reg_update_size = 0;
	struct cam_isp_csid_reg_update_args *rup_args = NULL;

	if (prepare->num_hw_update_entries + 1 >=
		prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
			prepare->num_hw_update_entries,
			prepare->max_hw_update_entries);
		return -EINVAL;
	}

	rup_args = (struct cam_isp_csid_reg_update_args *)args;

	if (!rup_args->num_res) {
		CAM_ERR(CAM_ISP, "No Res for Reg Update");
		return -EINVAL;
	}

	if (kmd_buf_info->size <=(kmd_buf_info->used_bytes +
		reg_update_size)) {
		CAM_ERR(CAM_ISP, "no free mem %u %u %u",
			kmd_buf_info->size,
			kmd_buf_info->used_bytes +
			reg_update_size);
		return -EINVAL;
	}

	kmd_buf_remain_size = kmd_buf_info->size -
		(kmd_buf_info->used_bytes +
		reg_update_size);

	rup_args->cmd.used_bytes = 0;
	rup_args->cmd.cmd_buf_addr = kmd_buf_info->cpu_addr +
		kmd_buf_info->used_bytes/4 +
		reg_update_size/4;
	rup_args->cmd.size = kmd_buf_remain_size;
	rup_args->reg_write = false;
	res = rup_args->res[0];

	rc = res->hw_intf->hw_ops.process_cmd(
		res->hw_intf->hw_priv,
		CAM_ISP_HW_CMD_GET_REG_UPDATE, rup_args,
		sizeof(struct cam_isp_csid_reg_update_args));
	if (rc)
		return rc;

	CAM_DBG(CAM_ISP,
		"Reg update added for res %d hw_id %d",
		res->res_id, res->hw_intf->hw_idx);
	reg_update_size += rup_args->cmd.used_bytes;

	/* Update hw entries */
	if (reg_update_size)
		cam_isp_update_hw_entry(CAM_ISP_COMMON_CFG_BL,
			prepare, kmd_buf_info, reg_update_size,
			combine);
	return 0;
}

int cam_isp_add_csid_offline_cmd(
	struct cam_hw_prepare_update_args    *prepare,
	struct cam_isp_resource_node         *res,
	struct cam_kmd_buf_info              *kmd_buf_info,
	bool                                  combine)
{
	int rc;
	uint32_t kmd_buf_remain_size, go_cmd_size;
	struct cam_ife_csid_offline_cmd_update_args go_args;

	if (prepare->num_hw_update_entries + 1 >=
		prepare->max_hw_update_entries) {
		CAM_ERR(CAM_ISP, "Insufficient  HW entries :%d %d",
			prepare->num_hw_update_entries,
			prepare->max_hw_update_entries);
		return -EINVAL;
	}

	go_cmd_size = 0;
	if (kmd_buf_info->size > (kmd_buf_info->used_bytes +
		go_cmd_size)) {
		kmd_buf_remain_size =  kmd_buf_info->size -
			(kmd_buf_info->used_bytes +
			go_cmd_size);
	} else {
		CAM_ERR(CAM_ISP, "no free mem %d %d",
			kmd_buf_info->size,
			kmd_buf_info->used_bytes +
			go_cmd_size);
		return -EINVAL;
	}

	go_args.cmd.cmd_buf_addr = kmd_buf_info->cpu_addr +
		kmd_buf_info->used_bytes / 4 +
		go_cmd_size / 4;
	go_args.cmd.size = kmd_buf_remain_size;
	go_args.res = res;

	rc = res->hw_intf->hw_ops.process_cmd(
			res->hw_intf->hw_priv,
			CAM_IFE_CSID_PROGRAM_OFFLINE_CMD, &go_args,
			sizeof(go_args));
	if (rc)
		return rc;

	CAM_DBG(CAM_ISP,
		"offline cmd update added for CSID: %u res: %d",
		res->hw_intf->hw_idx, res->res_id);
	go_cmd_size += go_args.cmd.used_bytes;

	/* Update HW entries */
	if (go_cmd_size)
		cam_isp_update_hw_entry(CAM_ISP_COMMON_CFG_BL,
			prepare, kmd_buf_info, go_cmd_size,
			combine);
	return 0;
}

int cam_isp_get_cmd_buf_count(
	struct cam_hw_prepare_update_args    *prepare,
	struct cam_isp_cmd_buf_count         *cmd_buf_count)
{
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uint32_t                        cmd_meta_data = 0;
	int                             i;
	int                             rc = 0;

	cmd_desc = (struct cam_cmd_buf_desc *)
			((uint8_t *)&prepare->packet->payload +
			prepare->packet->cmd_buf_offset);

	for (i = 0; i < prepare->packet->num_cmd_buf; i++) {
		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);
		if (rc)
			return rc;

		if (!cmd_desc[i].length)
			continue;

		rc = cam_packet_util_validate_cmd_desc(&cmd_desc[i]);

		if (rc)
			return rc;

		cmd_meta_data = cmd_desc[i].meta_data;

		switch (cmd_meta_data) {

		case CAM_ISP_PACKET_META_BASE:
		case CAM_ISP_PACKET_META_LEFT:
		case CAM_ISP_PACKET_META_DMI_LEFT:
		case CAM_ISP_PACKET_META_RIGHT:
		case CAM_ISP_PACKET_META_DMI_RIGHT:
		case CAM_ISP_PACKET_META_COMMON:
		case CAM_ISP_PACKET_META_DMI_COMMON:
			cmd_buf_count->isp_cnt++;
			break;
		case CAM_ISP_PACKET_META_CSID_LEFT:
		case CAM_ISP_PACKET_META_CSID_RIGHT:
		case CAM_ISP_PACKET_META_CSID_COMMON:
			cmd_buf_count->csid_cnt++;
			break;
		case CAM_ISP_SFE_PACKET_META_LEFT:
		case CAM_ISP_SFE_PACKET_META_RIGHT:
		case CAM_ISP_SFE_PACKET_META_COMMON:
			cmd_buf_count->sfe_cnt++;
			break;
		default:
			break;
		}
	}

	CAM_DBG(CAM_ISP, "Number of cmd buffers: isp:%u csid:%u sfe:%u",
		 cmd_buf_count->isp_cnt, cmd_buf_count->csid_cnt,
		 cmd_buf_count->sfe_cnt);
	return rc;
}
