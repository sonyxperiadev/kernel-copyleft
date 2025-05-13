// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "msm_vidc_internal.h"
#include "msm_vidc_driver.h"
#include "msm_venc.h"
#include "msm_vidc_platform.h"
#include "msm_vidc_debug.h"

extern struct msm_vidc_core *g_core;

static bool is_priv_ctrl(u32 id)
{
	bool private = false;

	if (IS_PRIV_CTRL(id))
		return true;

	/*
	 * Treat below standard controls as private because
	 * we have added custom values to the controls
	 */
	switch (id) {
	/*
	 * TODO: V4L2_CID_MPEG_VIDEO_HEVC_PROFILE is std ctrl. But
	 * V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10_STILL_PICTURE support is not
	 * available yet. Hence, make this as private ctrl for time being
	 */
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		private = true;
		break;
	default:
		private = false;
		break;
	}

	return private;
}

static const char *const mpeg_video_blur_types[] = {
	"Blur None",
	"Blur External",
	"Blur Adaptive",
	NULL,
};

static const char *const mpeg_video_hevc_profile[] = {
	"Main",
	"Main Still Picture",
	"Main 10",
	"Main 10 Still Picture",
	NULL,
};

static const char * const av1_profile[] = {
	"Main",
	"High",
	"Professional",
	NULL,
};

static const char * const av1_level[] = {
	"2.0",
	"2.1",
	"2.2",
	"2.3",
	"3.0",
	"3.1",
	"3.2",
	"3.3",
	"4.0",
	"4.1",
	"4.2",
	"4.3",
	"5.0",
	"5.1",
	"5.2",
	"5.3",
	"6.0",
	"6.1",
	"6.2",
	"6.3",
	"7.0",
	"7.1",
	"7.2",
	"7.3",
	NULL,
};

static const char * const av1_tier[] = {
	"Main",
	"High",
	NULL,
};

static const char *const mpeg_video_vidc_ir_type[] = {
	"Random",
	"Cyclic",
	NULL,
};

static const char * const *msm_vidc_get_qmenu_type(
		struct msm_vidc_inst *inst, u32 cap_id)
{
	switch (cap_id) {
	case BLUR_TYPES:
		return mpeg_video_blur_types;
	case PROFILE:
		if (inst->codec == MSM_VIDC_HEVC || inst->codec == MSM_VIDC_HEIC) {
			return mpeg_video_hevc_profile;
		} else if (inst->codec == MSM_VIDC_AV1) {
			return av1_profile;
		} else {
			i_vpr_e(inst, "%s: invalid codec type %d for cap id %d\n",
				__func__, inst->codec, cap_id);
			return NULL;
		}
	case LEVEL:
		if (inst->codec == MSM_VIDC_AV1) {
			return av1_level;
		} else {
			i_vpr_e(inst, "%s: invalid codec type %d for cap id %d\n",
				__func__, inst->codec, cap_id);
			return NULL;
		}
	case AV1_TIER:
		return av1_tier;
	case IR_TYPE:
		return mpeg_video_vidc_ir_type;
	default:
		i_vpr_e(inst, "%s: No available qmenu for cap id %d\n",
			__func__, cap_id);
		return NULL;
	}
}

static inline bool has_children(struct msm_vidc_inst_cap *cap)
{
	return !!cap->children[0];
}

static inline bool is_leaf(struct msm_vidc_inst_cap *cap)
{
	return !has_children(cap);
}

bool is_valid_cap_id(enum msm_vidc_inst_capability_type cap_id)
{
	return cap_id > INST_CAP_NONE && cap_id < INST_CAP_MAX;
}

bool is_valid_cap(struct msm_vidc_inst *inst,
		enum msm_vidc_inst_capability_type cap_id)
{
	if (cap_id <= INST_CAP_NONE || cap_id >= INST_CAP_MAX)
		return false;

	return !!inst->capabilities[cap_id].cap_id;
}

static inline bool is_all_childrens_visited(
	struct msm_vidc_inst_cap *cap, bool lookup[INST_CAP_MAX]) {
	bool found = true;
	int i;

	for (i = 0; i < MAX_CAP_CHILDREN; i++) {
		if (cap->children[i] == INST_CAP_NONE)
			continue;

		if (!lookup[cap->children[i]]) {
			found = false;
			break;
		}
	}
	return found;
}

static int add_node_list(struct list_head *list, enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst_cap_entry *entry = NULL;

	entry = vzalloc(sizeof(*entry));
	if (!entry) {
		d_vpr_e("%s: allocation failed\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&entry->list);
	entry->cap_id = cap_id;
	list_add(&entry->list, list);

	return rc;
}

static int add_node(
	struct list_head *list, struct msm_vidc_inst_cap *lcap, bool lookup[INST_CAP_MAX])
{
	int rc = 0;

	if (lookup[lcap->cap_id])
		return 0;

	rc = add_node_list(list, lcap->cap_id);
	if (rc)
		return rc;

	lookup[lcap->cap_id] = true;
	return 0;
}



static int msm_vidc_add_capid_to_fw_list(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id)
{
	struct msm_vidc_inst_cap_entry *entry = NULL;
	int rc = 0;

	/* skip adding if cap_id already present in firmware list */
	list_for_each_entry(entry, &inst->firmware_list, list) {
		if (entry->cap_id == cap_id) {
			i_vpr_l(inst,
				"%s: cap[%d] %s already present in fw list\n",
				__func__, cap_id, cap_name(cap_id));
			return 0;
		}
	}

	rc = add_node_list(&inst->firmware_list, cap_id);
	if (rc)
		return rc;

	return 0;
}

static int msm_vidc_add_children(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id)
{
	struct msm_vidc_inst_cap *cap;
	int i, rc = 0;

	cap = &inst->capabilities[cap_id];

	for (i = 0; i < MAX_CAP_CHILDREN; i++) {
		if (!cap->children[i])
			break;

		if (!is_valid_cap_id(cap->children[i]))
			continue;

		rc = add_node_list(&inst->children_list, cap->children[i]);
		if (rc)
			return rc;
	}

	return rc;
}

static int msm_vidc_adjust_cap(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id,
	struct v4l2_ctrl *ctrl, const char *func)
{
	struct msm_vidc_inst_cap *cap;
	int rc = 0;

	/* validate cap_id */
	if (!is_valid_cap_id(cap_id))
		return 0;

	/* validate cap */
	cap = &inst->capabilities[cap_id];
	if (!is_valid_cap(inst, cap->cap_id))
		return 0;

	/* check if adjust supported */
	if (!cap->adjust) {
		if (ctrl)
			msm_vidc_update_cap_value(inst, cap_id, ctrl->val, func);
		return 0;
	}

	/* call adjust */
	rc = cap->adjust(inst, ctrl);
	if (rc) {
		i_vpr_e(inst, "%s: adjust cap failed for %s\n", func, cap_name(cap_id));
		return rc;
	}

	return rc;
}

static int msm_vidc_set_cap(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id,
	const char *func)
{
	struct msm_vidc_inst_cap *cap;
	int rc = 0;

	/* validate cap_id */
	if (!is_valid_cap_id(cap_id))
		return 0;

	/* validate cap */
	cap = &inst->capabilities[cap_id];
	if (!is_valid_cap(inst, cap->cap_id))
		return 0;

	/* check if set supported */
	if (!cap->set)
		return 0;

	/* call set */
	rc = cap->set(inst, cap_id);
	if (rc) {
		i_vpr_e(inst, "%s: set cap failed for %s\n", func, cap_name(cap_id));
		return rc;
	}

	return rc;
}

static int msm_vidc_adjust_dynamic_property(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id, struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst_cap_entry *entry = NULL, *temp = NULL;
	struct msm_vidc_inst_cap *cap;
	s32 prev_value;
	int rc = 0;

	cap = &inst->capabilities[0];

	/* sanitize cap_id */
	if (!is_valid_cap_id(cap_id)) {
		i_vpr_e(inst, "%s: invalid cap_id %u\n", __func__, cap_id);
		return -EINVAL;
	}

	if (!(cap[cap_id].flags & CAP_FLAG_DYNAMIC_ALLOWED)) {
		i_vpr_h(inst,
			"%s: dynamic setting of cap[%d] %s is not allowed\n",
			__func__, cap_id, cap_name(cap_id));
		return -EBUSY;
	}
	i_vpr_h(inst, "%s: cap[%d] %s\n", __func__, cap_id, cap_name(cap_id));

	prev_value = cap[cap_id].value;
	rc = msm_vidc_adjust_cap(inst, cap_id, ctrl, __func__);
	if (rc)
		return rc;

	if (cap[cap_id].value == prev_value && cap_id == GOP_SIZE) {
		/*
		 * Ignore setting same GOP size value to firmware to avoid
		 * unnecessary generation of IDR frame.
		 */
		return 0;
	}

	/* add cap_id to firmware list always */
	rc = msm_vidc_add_capid_to_fw_list(inst, cap_id);
	if (rc)
		goto error;

	/* add children only if cap value modified */
	if (cap[cap_id].value == prev_value)
		return 0;

	rc = msm_vidc_add_children(inst, cap_id);
	if (rc)
		goto error;

	list_for_each_entry_safe(entry, temp, &inst->children_list, list) {
		if (!is_valid_cap_id(entry->cap_id)) {
			rc = -EINVAL;
			goto error;
		}

		if (!cap[entry->cap_id].adjust) {
			i_vpr_e(inst, "%s: child cap must have ajdust function %s\n",
				__func__, cap_name(entry->cap_id));
			rc = -EINVAL;
			goto error;
		}

		prev_value = cap[entry->cap_id].value;
		rc = msm_vidc_adjust_cap(inst, entry->cap_id, NULL, __func__);
		if (rc)
			goto error;

		/* add children if cap value modified */
		if (cap[entry->cap_id].value != prev_value) {
			/* add cap_id to firmware list always */
			rc = msm_vidc_add_capid_to_fw_list(inst, entry->cap_id);
			if (rc)
				goto error;

			rc = msm_vidc_add_children(inst, entry->cap_id);
			if (rc)
				goto error;
		}

		list_del_init(&entry->list);
		vfree(entry);
	}

	/* expecting children_list to be empty */
	if (!list_empty(&inst->children_list)) {
		i_vpr_e(inst, "%s: child_list is not empty\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	return 0;
error:
	list_for_each_entry_safe(entry, temp, &inst->children_list, list) {
		i_vpr_e(inst, "%s: child list: %s\n", __func__, cap_name(entry->cap_id));
		list_del_init(&entry->list);
		vfree(entry);
	}
	list_for_each_entry_safe(entry, temp, &inst->firmware_list, list) {
		i_vpr_e(inst, "%s: fw list: %s\n", __func__, cap_name(entry->cap_id));
		list_del_init(&entry->list);
		vfree(entry);
	}

	return rc;
}

static int msm_vidc_set_dynamic_property(struct msm_vidc_inst *inst)
{
	struct msm_vidc_inst_cap_entry *entry = NULL, *temp = NULL;
	int rc = 0;

	i_vpr_h(inst, "%s()\n", __func__);

	list_for_each_entry_safe(entry, temp, &inst->firmware_list, list) {
		rc = msm_vidc_set_cap(inst, entry->cap_id, __func__);
		if (rc)
			goto error;

		list_del_init(&entry->list);
		vfree(entry);
	}

	return 0;
error:
	list_for_each_entry_safe(entry, temp, &inst->firmware_list, list) {
		i_vpr_e(inst, "%s: fw list: %s\n", __func__, cap_name(entry->cap_id));
		list_del_init(&entry->list);
		vfree(entry);
	}

	return rc;
}

int msm_vidc_ctrl_handler_deinit(struct msm_vidc_inst *inst)
{
	i_vpr_h(inst, "%s(): num ctrls %d\n", __func__, inst->num_ctrls);
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
	memset(&inst->ctrl_handler, 0, sizeof(struct v4l2_ctrl_handler));

	return 0;
}

int msm_vidc_ctrl_handler_init(struct msm_vidc_inst *inst, bool init)
{
	int rc = 0;
	struct msm_vidc_inst_cap *cap;
	struct msm_vidc_core *core;
	int idx = 0;
	struct v4l2_ctrl_config ctrl_cfg = {0};
	int num_ctrls = 0, ctrl_idx = 0;
	u64 codecs_count, step_or_mask;

	core = inst->core;
	cap = &inst->capabilities[0];

	if (!core->v4l2_ctrl_ops) {
		i_vpr_e(inst, "%s: no control ops\n", __func__);
		return -EINVAL;
	}

	for (idx = 0; idx < INST_CAP_MAX; idx++) {
		if (cap[idx].v4l2_id)
			num_ctrls++;
	}
	if (!num_ctrls) {
		i_vpr_e(inst, "%s: no ctrls available in cap database\n",
			__func__);
		return -EINVAL;
	}

	if (init) {
		codecs_count = is_encode_session(inst) ?
			core->enc_codecs_count :
			core->dec_codecs_count;
		rc = v4l2_ctrl_handler_init(&inst->ctrl_handler,
			INST_CAP_MAX * codecs_count);
		if (rc) {
			i_vpr_e(inst, "control handler init failed, %d\n",
					inst->ctrl_handler.error);
			goto error;
		}
	}

	for (idx = 0; idx < INST_CAP_MAX; idx++) {
		struct v4l2_ctrl *ctrl;

		if (!cap[idx].v4l2_id)
			continue;

		if (ctrl_idx >= num_ctrls) {
			i_vpr_e(inst,
				"%s: invalid ctrl %#x, max allowed %d\n",
				__func__, cap[idx].v4l2_id,
				num_ctrls);
			rc = -EINVAL;
			goto error;
		}
		i_vpr_l(inst,
			"%s: cap[%d] %24s, value %d min %d max %d step_or_mask %#x flags %#x v4l2_id %#x hfi_id %#x\n",
			__func__, idx, cap_name(idx),
			cap[idx].value,
			cap[idx].min,
			cap[idx].max,
			cap[idx].step_or_mask,
			cap[idx].flags,
			cap[idx].v4l2_id,
			cap[idx].hfi_id);

		memset(&ctrl_cfg, 0, sizeof(struct v4l2_ctrl_config));

		/*
		 * few controls might have been already initialized in instance initialization,
		 * so modify the range values for them instead of initializing them again
		 */
		if (!init) {
			struct msm_vidc_ctrl_data ctrl_priv_data;

			ctrl = v4l2_ctrl_find(&inst->ctrl_handler, cap[idx].v4l2_id);
			if (ctrl) {
				step_or_mask = (cap[idx].flags & CAP_FLAG_MENU) ?
					~(cap[idx].step_or_mask) :
					cap[idx].step_or_mask;
				memset(&ctrl_priv_data, 0, sizeof(struct msm_vidc_ctrl_data));
				ctrl_priv_data.skip_s_ctrl = true;
				ctrl->priv = &ctrl_priv_data;
				v4l2_ctrl_modify_range(ctrl,
					cap[idx].min,
					cap[idx].max,
					step_or_mask,
					cap[idx].value);
				/* reset private data to null to ensure s_ctrl not skipped */
				ctrl->priv = NULL;
				continue;
			}
		}

		if (is_priv_ctrl(cap[idx].v4l2_id)) {
			/* add private control */
			ctrl_cfg.def = cap[idx].value;
			ctrl_cfg.flags = 0;
			ctrl_cfg.id = cap[idx].v4l2_id;
			ctrl_cfg.max = cap[idx].max;
			ctrl_cfg.min = cap[idx].min;
			ctrl_cfg.ops = core->v4l2_ctrl_ops;
			if (cap[idx].flags & CAP_FLAG_MENU)
				ctrl_cfg.type = V4L2_CTRL_TYPE_MENU;
			else if (cap[idx].flags & CAP_FLAG_BITMASK)
				ctrl_cfg.type = V4L2_CTRL_TYPE_BITMASK;
			else
				ctrl_cfg.type = V4L2_CTRL_TYPE_INTEGER;
			if (is_meta_cap(inst, idx)) {
				/* bitmask is expected to be enabled for meta controls */
				if (ctrl_cfg.type != V4L2_CTRL_TYPE_BITMASK) {
					i_vpr_e(inst,
						"%s: missing bitmask for cap %s\n",
						__func__, cap_name(idx));
					rc = -EINVAL;
					goto error;
				}
			}
			if (ctrl_cfg.type == V4L2_CTRL_TYPE_MENU) {
				ctrl_cfg.menu_skip_mask =
					~(cap[idx].step_or_mask);
				ctrl_cfg.qmenu = msm_vidc_get_qmenu_type(inst,
					cap[idx].cap_id);
			} else {
				ctrl_cfg.step =
					cap[idx].step_or_mask;
			}
			ctrl_cfg.name = cap_name(cap[idx].cap_id);
			if (!ctrl_cfg.name) {
				i_vpr_e(inst, "%s: %#x ctrl name is null\n",
					__func__, ctrl_cfg.id);
				rc = -EINVAL;
				goto error;
			}
			ctrl = v4l2_ctrl_new_custom(&inst->ctrl_handler,
					&ctrl_cfg, NULL);
		} else {
			if (cap[idx].flags & CAP_FLAG_MENU) {
				ctrl = v4l2_ctrl_new_std_menu(
					&inst->ctrl_handler,
					core->v4l2_ctrl_ops,
					cap[idx].v4l2_id,
					cap[idx].max,
					~(cap[idx].step_or_mask),
					cap[idx].value);
			} else {
				ctrl = v4l2_ctrl_new_std(&inst->ctrl_handler,
					core->v4l2_ctrl_ops,
					cap[idx].v4l2_id,
					cap[idx].min,
					cap[idx].max,
					cap[idx].step_or_mask,
					cap[idx].value);
			}
		}
		if (!ctrl) {
			i_vpr_e(inst, "%s: invalid ctrl %#x cap %24s\n", __func__,
				cap[idx].v4l2_id, cap_name(idx));
			rc = -EINVAL;
			goto error;
		}

		rc = inst->ctrl_handler.error;
		if (rc) {
			i_vpr_e(inst,
				"error adding ctrl (%#x) to ctrl handle, %d\n",
				cap[idx].v4l2_id,
				inst->ctrl_handler.error);
			goto error;
		}

		if (cap[idx].flags & CAP_FLAG_VOLATILE)
			ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

		ctrl->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
		ctrl_idx++;
	}
	inst->num_ctrls = num_ctrls;
	i_vpr_h(inst, "%s(): num ctrls %d\n", __func__, inst->num_ctrls);

	return 0;
error:
	msm_vidc_ctrl_handler_deinit(inst);

	return rc;
}

static int msm_vidc_update_buffer_count_if_needed(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	bool update_input_port = false, update_output_port = false;

	switch (cap_id) {
	case LAYER_TYPE:
	case ENH_LAYER_COUNT:
	case LAYER_ENABLE:
		update_input_port = true;
		break;
	case THUMBNAIL_MODE:
	case PRIORITY:
		update_input_port = true;
		update_output_port = true;
		break;
	default:
		update_input_port = false;
		update_output_port = false;
		break;
	}

	if (update_input_port) {
		rc = msm_vidc_update_buffer_count(inst, INPUT_PORT);
		if (rc)
			return rc;
	}
	if (update_output_port) {
		rc = msm_vidc_update_buffer_count(inst, OUTPUT_PORT);
		if (rc)
			return rc;
	}

	return rc;
}

static int msm_vidc_allow_secure_session(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_inst *i;
	struct msm_vidc_core *core;
	u32 count = 0;

	core = inst->core;

	core_lock(core, __func__);
	list_for_each_entry(i, &core->instances, list) {
		if (i->capabilities[SECURE_MODE].value)
			count++;
	}

	if (count > core->capabilities[MAX_SECURE_SESSION_COUNT].value) {
		i_vpr_e(inst,
			"%s: total secure sessions %d exceeded max limit %d\n",
			__func__, count,
			core->capabilities[MAX_SECURE_SESSION_COUNT].value);
		rc = -EINVAL;
	}
	core_unlock(core, __func__);

	return rc;
}

int msm_v4l2_op_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	int rc = 0;
	struct msm_vidc_inst *inst;

	if (!ctrl) {
		d_vpr_e("%s: invalid ctrl parameter\n", __func__);
		return -EINVAL;
	}

	inst = container_of(ctrl->handler,
			    struct msm_vidc_inst, ctrl_handler);
	inst = get_inst_ref(g_core, inst);
	if (!inst) {
		d_vpr_e("%s: could not find inst for ctrl %s id %#x\n",
			__func__, ctrl->name, ctrl->id);
		return -EINVAL;
	}
	client_lock(inst, __func__);
	inst_lock(inst, __func__);

	rc = msm_vidc_get_control(inst, ctrl);
	if (rc) {
		i_vpr_e(inst, "%s: failed for ctrl %s id %#x\n",
			__func__, ctrl->name, ctrl->id);
		goto unlock;
	} else {
		i_vpr_h(inst, "%s: ctrl %s id %#x, value %d\n",
			__func__, ctrl->name, ctrl->id, ctrl->val);
	}

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);
	return rc;
}

static int msm_vidc_update_static_property(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	if (cap_id == DRV_VERSION) {
		i_vpr_h(inst, "%s: driver version update not allowed\n",
			__func__);
		return 0;
	}

	/* update value to db */
	msm_vidc_update_cap_value(inst, cap_id, ctrl->val, __func__);

	if (cap_id == CLIENT_ID) {
		rc = msm_vidc_update_debug_str(inst);
		if (rc)
			return rc;
	}

	if (cap_id == SECURE_MODE) {
		if (ctrl->val) {
			rc = msm_vidc_allow_secure_session(inst);
			if (rc)
				return rc;
		}
	}

	if (cap_id == ROTATION) {
		struct v4l2_format *output_fmt;

		output_fmt = &inst->fmts[OUTPUT_PORT];
		rc = msm_venc_s_fmt_output(inst, output_fmt);
		if (rc)
			return rc;
	}

	if (cap_id == DELIVERY_MODE) {
		struct v4l2_format *output_fmt;

		output_fmt = &inst->fmts[OUTPUT_PORT];
		rc = msm_venc_s_fmt_output(inst, output_fmt);
		if (rc)
			return rc;
	}

	if (cap_id == BITSTREAM_SIZE_OVERWRITE) {
		rc = msm_vidc_update_bitstream_buffer_size(inst);
		if (rc)
			return rc;
	}

	/* call this explicitly to adjust client priority */
	if (cap_id == PRIORITY) {
		rc = msm_vidc_adjust_session_priority(inst, ctrl);
		if (rc)
			return rc;
	}

	if (cap_id == CRITICAL_PRIORITY)
		msm_vidc_update_cap_value(inst, PRIORITY, 0, __func__);

	if (cap_id == ENH_LAYER_COUNT && inst->codec == MSM_VIDC_HEVC) {
		u32 enable;

		/* enable LAYER_ENABLE cap if HEVC_HIER enh layers > 0 */
		if (ctrl->val > 0)
			enable = 1;
		else
			enable = 0;

		msm_vidc_update_cap_value(inst, LAYER_ENABLE, enable, __func__);
	}
	if (is_meta_cap(inst, cap_id)) {
		rc = msm_vidc_update_meta_port_settings(inst);
		if (rc)
			return rc;
	}

	rc = msm_vidc_update_buffer_count_if_needed(inst, cap_id);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_s_ctrl(struct msm_vidc_inst *inst, struct v4l2_ctrl *ctrl)
{
	enum msm_vidc_inst_capability_type cap_id;
	struct msm_vidc_inst_cap *cap;
	int rc = 0;
	u32 port;

	cap = &inst->capabilities[0];

	i_vpr_h(inst, FMT_STRING_SET_CTRL,
		__func__, state_name(inst->state), ctrl->name, ctrl->id, ctrl->val);

	cap_id = msm_vidc_get_cap_id(inst, ctrl->id);
	if (!is_valid_cap_id(cap_id)) {
		i_vpr_e(inst, "%s: invalid cap_id for ctrl %s\n", __func__, ctrl->name);
		return -EINVAL;
	}

	/* mark client set flag */
	cap[cap_id].flags |= CAP_FLAG_CLIENT_SET;

	port = is_encode_session(inst) ? OUTPUT_PORT : INPUT_PORT;
	if (!inst->bufq[port].vb2q->streaming) {
		/* static case */
		rc = msm_vidc_update_static_property(inst, cap_id, ctrl);
		if (rc)
			return rc;
	} else {
		/* dynamic case */
		rc = msm_vidc_adjust_dynamic_property(inst, cap_id, ctrl);
		if (rc)
			return rc;

		rc = msm_vidc_set_dynamic_property(inst);
		if (rc)
			return rc;
	}

	return rc;
}

int msm_v4l2_op_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct msm_vidc_inst *inst;
	struct msm_vidc_ctrl_data *priv_ctrl_data;
	int rc = 0;

	if (!ctrl) {
		d_vpr_e("%s: invalid ctrl parameter\n", __func__);
		return -EINVAL;
	}

	/*
	 * v4l2_ctrl_modify_range may internally call s_ctrl
	 * which will again try to acquire lock leading to deadlock,
	 * Add check to avoid such scenario.
	 */
	priv_ctrl_data = ctrl->priv ? ctrl->priv : NULL;
	if (priv_ctrl_data && priv_ctrl_data->skip_s_ctrl) {
		d_vpr_l("%s: skip s_ctrl (%s)\n", __func__, ctrl->name);
		return 0;
	}

	inst = container_of(ctrl->handler, struct msm_vidc_inst, ctrl_handler);
	inst = get_inst_ref(g_core, inst);
	if (!inst) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	rc = inst->event_handle(inst, MSM_VIDC_S_CTRL, ctrl);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);
	return rc;
}

int msm_vidc_prepare_dependency_list(struct msm_vidc_inst *inst)
{
	struct list_head leaf_list, opt_list;
	struct msm_vidc_inst_cap *cap, *lcap, *temp_cap;
	struct msm_vidc_inst_cap_entry *entry = NULL, *temp = NULL;
	bool leaf_visited[INST_CAP_MAX];
	bool opt_visited[INST_CAP_MAX];
	int tmp_count_total, tmp_count, num_nodes = 0;
	int i, rc = 0;

	cap = &inst->capabilities[0];

	if (!list_empty(&inst->caps_list)) {
		i_vpr_h(inst, "%s: dependency list already prepared\n", __func__);
		return 0;
	}

	/* init local list and lookup table entries */
	INIT_LIST_HEAD(&leaf_list);
	INIT_LIST_HEAD(&opt_list);
	memset(&leaf_visited, 0, sizeof(leaf_visited));
	memset(&opt_visited, 0, sizeof(opt_visited));

	/* populate leaf nodes first */
	for (i = 1; i < INST_CAP_MAX; i++) {
		lcap = &cap[i];
		if (!is_valid_cap(inst, lcap->cap_id))
			continue;

		/* sanitize cap value */
		if (i != lcap->cap_id) {
			i_vpr_e(inst, "%s: cap id mismatch. expected %s, actual %s\n",
				__func__, cap_name(i), cap_name(lcap->cap_id));
			rc = -EINVAL;
			goto error;
		}

		/* add all leaf nodes */
		if (is_leaf(lcap)) {
			rc = add_node(&leaf_list, lcap, leaf_visited);
			if (rc)
				goto error;
		} else {
			rc = add_node(&opt_list, lcap, opt_visited);
			if (rc)
				goto error;
		}
	}

	/* find total optional list entries */
	list_for_each_entry(entry, &opt_list, list)
		num_nodes++;

	/* used for loop detection */
	tmp_count_total = num_nodes;
	tmp_count = num_nodes;

	/* sort final outstanding nodes */
	list_for_each_entry_safe(entry, temp, &opt_list, list) {
		/* initially remove entry from opt list */
		list_del_init(&entry->list);
		opt_visited[entry->cap_id] = false;
		tmp_count--;
		temp_cap = &cap[entry->cap_id];

		/**
		 * if all child are visited then add this entry to
		 * leaf list else add it to the end of optional list.
		 */
		if (is_all_childrens_visited(temp_cap, leaf_visited)) {
			list_add(&entry->list, &leaf_list);
			leaf_visited[entry->cap_id] = true;
			tmp_count_total--;
		} else {
			list_add_tail(&entry->list, &opt_list);
			opt_visited[entry->cap_id] = true;
		}

		/* detect loop */
		if (!tmp_count) {
			if (num_nodes == tmp_count_total) {
				i_vpr_e(inst, "%s: loop detected in subgraph %d\n",
					__func__, num_nodes);
				rc = -EINVAL;
				goto error;
			}
			num_nodes = tmp_count_total;
			tmp_count = tmp_count_total;
		}
	}

	/* expecting opt_list to be empty */
	if (!list_empty(&opt_list)) {
		i_vpr_e(inst, "%s: opt_list is not empty\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	/* move elements to &inst->caps_list from local */
	list_replace_init(&leaf_list, &inst->caps_list);

	return 0;
error:
	list_for_each_entry_safe(entry, temp, &opt_list, list) {
		i_vpr_e(inst, "%s: opt_list: %s\n", __func__, cap_name(entry->cap_id));
		list_del_init(&entry->list);
		vfree(entry);
	}
	list_for_each_entry_safe(entry, temp, &leaf_list, list) {
		i_vpr_e(inst, "%s: leaf_list: %s\n", __func__, cap_name(entry->cap_id));
		list_del_init(&entry->list);
		vfree(entry);
	}
	return rc;
}

int msm_vidc_adjust_v4l2_properties(struct msm_vidc_inst *inst)
{
	struct msm_vidc_inst_cap_entry *entry = NULL, *temp = NULL;
	int rc = 0;

	i_vpr_h(inst, "%s()\n", __func__);

	/* adjust all possible caps from caps_list */
	list_for_each_entry_safe(entry, temp, &inst->caps_list, list) {
		i_vpr_l(inst, "%s: cap: id %3u, name %s\n", __func__,
			entry->cap_id, cap_name(entry->cap_id));

		rc = msm_vidc_adjust_cap(inst, entry->cap_id, NULL, __func__);
		if (rc)
			return rc;
	}

	return rc;
}

int msm_vidc_set_v4l2_properties(struct msm_vidc_inst *inst)
{
	struct msm_vidc_inst_cap_entry *entry = NULL, *temp = NULL;
	int rc = 0;

	i_vpr_h(inst, "%s()\n", __func__);

	/* set all caps from caps_list */
	list_for_each_entry_safe(entry, temp, &inst->caps_list, list) {
		rc = msm_vidc_set_cap(inst, entry->cap_id, __func__);
		if (rc)
			return rc;
	}

	return rc;
}
