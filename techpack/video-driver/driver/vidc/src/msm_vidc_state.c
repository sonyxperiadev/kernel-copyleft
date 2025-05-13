// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "msm_vidc_control.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_state.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_core.h"
#include "msm_vidc_vb2.h"
#include "msm_vidc.h"
#include "msm_vidc_events.h"
#include "venus_hfi.h"

bool core_in_valid_state(struct msm_vidc_core *core)
{
	return (core->state == MSM_VIDC_CORE_INIT ||
		core->state == MSM_VIDC_CORE_INIT_WAIT);
}

bool is_core_state(struct msm_vidc_core *core, enum msm_vidc_core_state state)
{
	return core->state == state;
}

bool is_drc_pending(struct msm_vidc_inst *inst)
{
	return is_sub_state(inst, MSM_VIDC_DRC) &&
		is_sub_state(inst, MSM_VIDC_DRC_LAST_BUFFER);
}

bool is_drain_pending(struct msm_vidc_inst *inst)
{
	return is_sub_state(inst, MSM_VIDC_DRAIN) &&
		is_sub_state(inst, MSM_VIDC_DRAIN_LAST_BUFFER);
}

static const char * const core_state_name_arr[] =
	FOREACH_CORE_STATE(GENERATE_STRING);

const char *core_state_name(enum msm_vidc_core_state state)
{
	const char *name = "UNKNOWN STATE";

	if (state >= ARRAY_SIZE(core_state_name_arr))
		goto exit;

	name = core_state_name_arr[state];

exit:
	return name;
}

static const char * const event_name_arr[] =
	FOREACH_EVENT(GENERATE_STRING);

static const char *event_name(enum msm_vidc_event event)
{
	const char *name = "UNKNOWN EVENT";

	if (event >= ARRAY_SIZE(event_name_arr))
		goto exit;

	name = event_name_arr[event];

exit:
	return name;
}

static int __strict_inst_check(struct msm_vidc_inst *inst, const char *function)
{
	bool fatal = !mutex_is_locked(&inst->lock);

	WARN_ON(fatal);

	return fatal ? -EINVAL : 0;
}

static int msm_vidc_core_deinit_state(struct msm_vidc_core *core,
	enum msm_vidc_core_event_type type,
	struct msm_vidc_event_data *data)
{
	int rc = 0;

	switch (type) {
	case CORE_EVENT_UPDATE_SUB_STATE:
	{
		u32 req_sub_state;
		u32 allow_mask = -1;

		req_sub_state = data->edata.uval;

		/* none of the requested substate supported */
		if (!(req_sub_state & allow_mask)) {
			d_vpr_e("%s: invalid substate update request %#x\n",
				__func__, req_sub_state);
			return -EINVAL;
		}

		/* update core substate */
		core->sub_state |= req_sub_state & allow_mask;
		return rc;
	}
	default: {
		d_vpr_e("%s: unexpected core event type %u\n",
			__func__, type);
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_core_init_wait_state(struct msm_vidc_core *core,
	enum msm_vidc_core_event_type type,
	struct msm_vidc_event_data *data)
{
	int rc = 0;

	switch (type) {
	case CORE_EVENT_UPDATE_SUB_STATE:
	{
		u32 req_sub_state;
		u32 allow_mask = -1;

		req_sub_state = data->edata.uval;

		/* none of the requested substate supported */
		if (!(req_sub_state & allow_mask)) {
			d_vpr_e("%s: invalid substate update request %#x\n",
				__func__, req_sub_state);
			return -EINVAL;
		}

		/* update core substate */
		core->sub_state |= req_sub_state & allow_mask;
		return rc;
	}
	default: {
		d_vpr_e("%s: unexpected core event type %u\n",
			__func__, type);
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_core_init_state(struct msm_vidc_core *core,
	enum msm_vidc_core_event_type type,
	struct msm_vidc_event_data *data)
{
	int rc = 0;

	switch (type) {
	case CORE_EVENT_UPDATE_SUB_STATE:
	{
		u32 req_sub_state;
		u32 allow_mask = -1;

		req_sub_state = data->edata.uval;

		/* none of the requested substate supported */
		if (!(req_sub_state & allow_mask)) {
			d_vpr_e("%s: invalid substate update request %#x\n",
				__func__, req_sub_state);
			return -EINVAL;
		}

		/* update core substate */
		core->sub_state |= req_sub_state & allow_mask;
		return rc;
	}
	default: {
		d_vpr_e("%s: unexpected core event type %u\n",
			__func__, type);
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_core_error_state(struct msm_vidc_core *core,
	enum msm_vidc_core_event_type type,
	struct msm_vidc_event_data *data)
{
	int rc = 0;

	switch (type) {
	case CORE_EVENT_UPDATE_SUB_STATE:
	{
		u32 req_sub_state;
		u32 allow_mask = -1;

		req_sub_state = data->edata.uval;

		/* none of the requested substate supported */
		if (!(req_sub_state & allow_mask)) {
			d_vpr_e("%s: invalid substate update request %#x\n",
				__func__, req_sub_state);
			return -EINVAL;
		}

		/* update core substate */
		core->sub_state |= req_sub_state & allow_mask;
		return rc;
	}
	default: {
		d_vpr_e("%s: unexpected core event type %u\n",
			__func__, type);
		return -EINVAL;
	}
	}

	return rc;
}

struct msm_vidc_core_state_handle {
	enum msm_vidc_core_state   state;
	int                      (*handle)(struct msm_vidc_core *core,
				   enum msm_vidc_core_event_type type,
				   struct msm_vidc_event_data *data);
};

static struct msm_vidc_core_state_handle *msm_vidc_get_core_state_handle(
	enum msm_vidc_core_state req_state)
{
	int cnt;
	struct msm_vidc_core_state_handle *core_state_handle = NULL;
	static struct msm_vidc_core_state_handle state_handle[] = {
		{MSM_VIDC_CORE_DEINIT,      msm_vidc_core_deinit_state      },
		{MSM_VIDC_CORE_INIT_WAIT,   msm_vidc_core_init_wait_state   },
		{MSM_VIDC_CORE_INIT,        msm_vidc_core_init_state        },
		{MSM_VIDC_CORE_ERROR,       msm_vidc_core_error_state       },
	};

	for (cnt = 0; cnt < ARRAY_SIZE(state_handle); cnt++) {
		if (state_handle[cnt].state == req_state) {
			core_state_handle = &state_handle[cnt];
			break;
		}
	}

	/* if req_state does not exist in the table */
	if (cnt == ARRAY_SIZE(state_handle)) {
		d_vpr_e("%s: invalid core state \"%s\" requested\n",
			__func__, core_state_name(req_state));
		return core_state_handle;
	}

	return core_state_handle;
}

int msm_vidc_update_core_state(struct msm_vidc_core *core,
	enum msm_vidc_core_state request_state, const char *func)
{
	struct msm_vidc_core_state_handle *state_handle = NULL;
	int rc = 0;

	/* get core state handler for requested state */
	state_handle = msm_vidc_get_core_state_handle(request_state);
	if (!state_handle)
		return -EINVAL;

	d_vpr_h("%s: core state changed to %s from %s\n", func,
		core_state_name(state_handle->state), core_state_name(core->state));

	/* finally update core state and handler */
	core->state = state_handle->state;
	core->state_handle = state_handle->handle;

	return rc;
}

struct msm_vidc_core_state_allow {
	enum msm_vidc_core_state   from;
	enum msm_vidc_core_state   to;
	enum msm_vidc_allow        allow;
};

enum msm_vidc_allow msm_vidc_allow_core_state_change(
	struct msm_vidc_core *core,
	enum msm_vidc_core_state req_state)
{
	int cnt;
	enum msm_vidc_allow allow = MSM_VIDC_DISALLOW;
	static struct msm_vidc_core_state_allow state[] = {
		/* from, to, allow */
		{MSM_VIDC_CORE_DEINIT,      MSM_VIDC_CORE_DEINIT,      MSM_VIDC_IGNORE    },
		{MSM_VIDC_CORE_DEINIT,      MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_ALLOW     },
		{MSM_VIDC_CORE_DEINIT,      MSM_VIDC_CORE_INIT,        MSM_VIDC_DISALLOW  },
		{MSM_VIDC_CORE_DEINIT,      MSM_VIDC_CORE_ERROR,       MSM_VIDC_IGNORE    },
		{MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_CORE_DEINIT,      MSM_VIDC_DISALLOW  },
		{MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_IGNORE    },
		{MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_CORE_INIT,        MSM_VIDC_ALLOW     },
		{MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_CORE_ERROR,       MSM_VIDC_ALLOW     },
		{MSM_VIDC_CORE_INIT,        MSM_VIDC_CORE_DEINIT,      MSM_VIDC_ALLOW     },
		{MSM_VIDC_CORE_INIT,        MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_DISALLOW  },
		{MSM_VIDC_CORE_INIT,        MSM_VIDC_CORE_INIT,        MSM_VIDC_IGNORE    },
		{MSM_VIDC_CORE_INIT,        MSM_VIDC_CORE_ERROR,       MSM_VIDC_ALLOW     },
		{MSM_VIDC_CORE_ERROR,       MSM_VIDC_CORE_DEINIT,      MSM_VIDC_ALLOW     },
		{MSM_VIDC_CORE_ERROR,       MSM_VIDC_CORE_INIT_WAIT,   MSM_VIDC_IGNORE    },
		{MSM_VIDC_CORE_ERROR,       MSM_VIDC_CORE_INIT,        MSM_VIDC_IGNORE    },
		{MSM_VIDC_CORE_ERROR,       MSM_VIDC_CORE_ERROR,       MSM_VIDC_IGNORE    },
	};

	for (cnt = 0; cnt < ARRAY_SIZE(state); cnt++) {
		if (state[cnt].from == core->state && state[cnt].to == req_state) {
			allow = state[cnt].allow;
			break;
		}
	}

	return allow;
}

int msm_vidc_change_core_state(struct msm_vidc_core *core,
	enum msm_vidc_core_state request_state, const char *func)
{
	enum msm_vidc_allow allow;
	int rc = 0;

	/* core must be locked */
	rc = __strict_check(core, func);
	if (rc) {
		d_vpr_e("%s(): core was not locked\n", func);
		return rc;
	}

	/* current and requested state is same */
	if (core->state == request_state)
		return 0;

	/* check if requested state movement is allowed */
	allow = msm_vidc_allow_core_state_change(core, request_state);
	if (allow == MSM_VIDC_IGNORE) {
		d_vpr_h("%s: %s core state change %s -> %s\n", func,
			allow_name(allow), core_state_name(core->state),
			core_state_name(request_state));
		return 0;
	} else if (allow == MSM_VIDC_DISALLOW) {
		d_vpr_e("%s: %s core state change %s -> %s\n", func,
			allow_name(allow), core_state_name(core->state),
			core_state_name(request_state));
		return -EINVAL;
	}

	/* go ahead and update core state */
	rc = msm_vidc_update_core_state(core, request_state, func);
	if (rc)
		return rc;

	return rc;
}

bool is_core_sub_state(struct msm_vidc_core *core,
	enum msm_vidc_core_sub_state sub_state)
{
	return !!(core->sub_state & sub_state);
}

const char *core_sub_state_name(enum msm_vidc_core_sub_state sub_state)
{
	switch (sub_state) {
	case CORE_SUBSTATE_NONE:                 return "NONE ";
	case CORE_SUBSTATE_GDSC_HANDOFF:         return "GDSC_HANDOFF ";
	case CORE_SUBSTATE_PM_SUSPEND:           return "PM_SUSPEND ";
	case CORE_SUBSTATE_FW_PWR_CTRL:          return "FW_PWR_CTRL ";
	case CORE_SUBSTATE_POWER_ENABLE:         return "POWER_ENABLE ";
	case CORE_SUBSTATE_PAGE_FAULT:           return "PAGE_FAULT ";
	case CORE_SUBSTATE_CPU_WATCHDOG:         return "CPU_WATCHDOG ";
	case CORE_SUBSTATE_VIDEO_UNRESPONSIVE:   return "VIDEO_UNRESPONSIVE ";
	case CORE_SUBSTATE_MAX:                  return "MAX ";
	}

	return "UNKNOWN ";
}

static int prepare_core_sub_state_name(enum msm_vidc_core_sub_state sub_state,
	char *buf, u32 size)
{
	int i = 0;

	if (!buf || !size)
		return -EINVAL;

	strscpy(buf, "\0", size);
	if (sub_state == CORE_SUBSTATE_NONE) {
		strscpy(buf, "CORE_SUBSTATE_NONE", size);
		return 0;
	}

	for (i = 0; BIT(i) < CORE_SUBSTATE_MAX; i++) {
		if (sub_state & BIT(i))
			strlcat(buf, core_sub_state_name(BIT(i)), size);
	}

	return 0;
}

static int msm_vidc_update_core_sub_state(struct msm_vidc_core *core,
	enum msm_vidc_core_sub_state sub_state, const char *func)
{
	struct msm_vidc_event_data data;
	char sub_state_name[MAX_NAME_LENGTH];
	int ret = 0, rc = 0;

	/* no substate update */
	if (!sub_state)
		return 0;

	/* invoke update core substate event */
	memset(&data, 0, sizeof(struct msm_vidc_event_data));
	data.edata.uval = sub_state;
	rc = core->state_handle(core, CORE_EVENT_UPDATE_SUB_STATE, &data);
	if (rc) {
		ret = prepare_core_sub_state_name(sub_state,
			 sub_state_name, sizeof(sub_state_name) - 1);
		if (!ret)
			d_vpr_e("%s: state %s, requested invalid core substate %s\n",
				func, core_state_name(core->state), sub_state_name);
		return rc;
	}

	return rc;
}

int msm_vidc_change_core_sub_state(struct msm_vidc_core *core,
		enum msm_vidc_core_sub_state clear_sub_state,
		enum msm_vidc_core_sub_state set_sub_state, const char *func)
{
	int rc = 0;
	enum msm_vidc_core_sub_state prev_sub_state;

	/* core must be locked */
	rc = __strict_check(core, func);
	if (rc) {
		d_vpr_e("%s(): core was not locked\n", func);
		return rc;
	}

	/* sanitize core state handler */
	if (!core->state_handle) {
		d_vpr_e("%s: invalid core state handle\n", __func__);
		return -EINVAL;
	}

	/* final value will not change */
	if (clear_sub_state == set_sub_state)
		return 0;

	/* sanitize clear & set value */
	if (set_sub_state > CORE_SUBSTATE_MAX ||
		clear_sub_state > CORE_SUBSTATE_MAX) {
		d_vpr_e("%s: invalid sub states. clear %#x or set %#x\n",
			func, clear_sub_state, set_sub_state);
		return -EINVAL;
	}

	prev_sub_state = core->sub_state;

	/* set sub state */
	rc = msm_vidc_update_core_sub_state(core, set_sub_state, func);
	if (rc)
		return rc;

	/* check if all core substates updated */
	if ((core->sub_state & set_sub_state) != set_sub_state)
		d_vpr_e("%s: all substates not updated %#x, expected %#x\n",
			func, core->sub_state & set_sub_state, set_sub_state);

	/* clear sub state */
	core->sub_state &= ~clear_sub_state;

	/* print substates only when there is a change */
	if (core->sub_state != prev_sub_state) {
		rc = prepare_core_sub_state_name(core->sub_state, core->sub_state_name,
			sizeof(core->sub_state_name) - 1);
		if (!rc)
			d_vpr_h("%s: core sub state changed to %s\n", func, core->sub_state_name);
	}

	return 0;
}

/* do not modify the state names as it is used in test scripts */
static const char * const state_name_arr[] =
	FOREACH_STATE(GENERATE_STRING);

const char *state_name(enum msm_vidc_state state)
{
	const char *name = "UNKNOWN STATE";

	if (state >= ARRAY_SIZE(state_name_arr))
		goto exit;

	name = state_name_arr[state];

exit:
	return name;
}

bool is_state(struct msm_vidc_inst *inst, enum msm_vidc_state state)
{
	return inst->state == state;
}

bool is_sub_state(struct msm_vidc_inst *inst, enum msm_vidc_sub_state sub_state)
{
	return (inst->sub_state & sub_state);
}

const char *sub_state_name(enum msm_vidc_sub_state sub_state)
{
	switch (sub_state) {
	case MSM_VIDC_DRAIN:               return "DRAIN ";
	case MSM_VIDC_DRC:                 return "DRC ";
	case MSM_VIDC_DRAIN_LAST_BUFFER:   return "DRAIN_LAST_BUFFER ";
	case MSM_VIDC_DRC_LAST_BUFFER:     return "DRC_LAST_BUFFER ";
	case MSM_VIDC_INPUT_PAUSE:         return "INPUT_PAUSE ";
	case MSM_VIDC_OUTPUT_PAUSE:        return "OUTPUT_PAUSE ";
	case MSM_VIDC_FIRST_IPSC:          return "FIRST_IPSC ";
	}

	return "SUB_STATE_NONE";
}

static int prepare_sub_state_name(enum msm_vidc_sub_state sub_state,
	char *buf, u32 size)
{
	int i = 0;

	if (!buf || !size)
		return -EINVAL;

	strscpy(buf, "\0", size);
	if (sub_state == MSM_VIDC_SUB_STATE_NONE) {
		strscpy(buf, "SUB_STATE_NONE", size);
		return 0;
	}

	for (i = 0; i < MSM_VIDC_MAX_SUB_STATES; i++) {
		if (sub_state & BIT(i))
			strlcat(buf, sub_state_name(BIT(i)), size);
	}

	return 0;
}

struct msm_vidc_state_allow {
	enum msm_vidc_state        from;
	enum msm_vidc_state        to;
	enum msm_vidc_allow        allow;
};

static enum msm_vidc_allow msm_vidc_allow_state_change(
	struct msm_vidc_inst *inst,
	enum msm_vidc_state req_state)
{
	int cnt;
	enum msm_vidc_allow allow = MSM_VIDC_DISALLOW;
	static struct msm_vidc_state_allow state[] = {
		/* from, to, allow */
		{MSM_VIDC_OPEN,             MSM_VIDC_OPEN,               MSM_VIDC_IGNORE    },
		{MSM_VIDC_OPEN,             MSM_VIDC_INPUT_STREAMING,    MSM_VIDC_ALLOW     },
		{MSM_VIDC_OPEN,             MSM_VIDC_OUTPUT_STREAMING,   MSM_VIDC_ALLOW     },
		{MSM_VIDC_OPEN,             MSM_VIDC_STREAMING,          MSM_VIDC_DISALLOW  },
		{MSM_VIDC_OPEN,             MSM_VIDC_CLOSE,              MSM_VIDC_ALLOW     },
		{MSM_VIDC_OPEN,             MSM_VIDC_ERROR,              MSM_VIDC_ALLOW     },

		{MSM_VIDC_INPUT_STREAMING,  MSM_VIDC_OPEN,               MSM_VIDC_ALLOW     },
		{MSM_VIDC_INPUT_STREAMING,  MSM_VIDC_INPUT_STREAMING,    MSM_VIDC_IGNORE    },
		{MSM_VIDC_INPUT_STREAMING,  MSM_VIDC_OUTPUT_STREAMING,   MSM_VIDC_DISALLOW  },
		{MSM_VIDC_INPUT_STREAMING,  MSM_VIDC_STREAMING,          MSM_VIDC_ALLOW     },
		{MSM_VIDC_INPUT_STREAMING,  MSM_VIDC_CLOSE,              MSM_VIDC_ALLOW     },
		{MSM_VIDC_INPUT_STREAMING,  MSM_VIDC_ERROR,              MSM_VIDC_ALLOW     },

		{MSM_VIDC_OUTPUT_STREAMING, MSM_VIDC_OPEN,               MSM_VIDC_ALLOW     },
		{MSM_VIDC_OUTPUT_STREAMING, MSM_VIDC_INPUT_STREAMING,    MSM_VIDC_DISALLOW  },
		{MSM_VIDC_OUTPUT_STREAMING, MSM_VIDC_OUTPUT_STREAMING,   MSM_VIDC_IGNORE    },
		{MSM_VIDC_OUTPUT_STREAMING, MSM_VIDC_STREAMING,          MSM_VIDC_ALLOW     },
		{MSM_VIDC_OUTPUT_STREAMING, MSM_VIDC_CLOSE,              MSM_VIDC_ALLOW     },
		{MSM_VIDC_OUTPUT_STREAMING, MSM_VIDC_ERROR,              MSM_VIDC_ALLOW     },

		{MSM_VIDC_STREAMING,        MSM_VIDC_OPEN,               MSM_VIDC_DISALLOW  },
		{MSM_VIDC_STREAMING,        MSM_VIDC_INPUT_STREAMING,    MSM_VIDC_ALLOW     },
		{MSM_VIDC_STREAMING,        MSM_VIDC_OUTPUT_STREAMING,   MSM_VIDC_ALLOW     },
		{MSM_VIDC_STREAMING,        MSM_VIDC_STREAMING,          MSM_VIDC_IGNORE    },
		{MSM_VIDC_STREAMING,        MSM_VIDC_CLOSE,              MSM_VIDC_ALLOW     },
		{MSM_VIDC_STREAMING,        MSM_VIDC_ERROR,              MSM_VIDC_ALLOW     },

		{MSM_VIDC_CLOSE,            MSM_VIDC_OPEN,               MSM_VIDC_DISALLOW  },
		{MSM_VIDC_CLOSE,            MSM_VIDC_INPUT_STREAMING,    MSM_VIDC_DISALLOW  },
		{MSM_VIDC_CLOSE,            MSM_VIDC_OUTPUT_STREAMING,   MSM_VIDC_DISALLOW  },
		{MSM_VIDC_CLOSE,            MSM_VIDC_STREAMING,          MSM_VIDC_DISALLOW  },
		{MSM_VIDC_CLOSE,            MSM_VIDC_CLOSE,              MSM_VIDC_IGNORE    },
		{MSM_VIDC_CLOSE,            MSM_VIDC_ERROR,              MSM_VIDC_IGNORE    },

		{MSM_VIDC_ERROR,            MSM_VIDC_OPEN,               MSM_VIDC_IGNORE    },
		{MSM_VIDC_ERROR,            MSM_VIDC_INPUT_STREAMING,    MSM_VIDC_IGNORE    },
		{MSM_VIDC_ERROR,            MSM_VIDC_OUTPUT_STREAMING,   MSM_VIDC_IGNORE    },
		{MSM_VIDC_ERROR,            MSM_VIDC_STREAMING,          MSM_VIDC_IGNORE    },
		{MSM_VIDC_ERROR,            MSM_VIDC_CLOSE,              MSM_VIDC_IGNORE    },
		{MSM_VIDC_ERROR,            MSM_VIDC_ERROR,              MSM_VIDC_IGNORE    },
	};

	for (cnt = 0; cnt < ARRAY_SIZE(state); cnt++) {
		if (state[cnt].from == inst->state && state[cnt].to == req_state) {
			allow = state[cnt].allow;
			break;
		}
	}

	return allow;
}

static int msm_vidc_open_state(struct msm_vidc_inst *inst,
	enum msm_vidc_event event, void *data)
{
	int rc = 0;

	/* inst must be locked */
	rc = __strict_inst_check(inst, __func__);
	if (rc) {
		i_vpr_e(inst, "%s(): inst was not locked\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case MSM_VIDC_TRY_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *)data;

		/* allow try_fmt request in open state */
		rc = msm_vidc_try_fmt(inst, f);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_S_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *)data;

		/* allow s_fmt request in open state */
		rc = msm_vidc_s_fmt(inst, f);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_S_CTRL:
	{
		struct v4l2_ctrl *ctrl = (struct v4l2_ctrl *)data;

		/* allow set_control request in open state */
		rc = msm_vidc_s_ctrl(inst, ctrl);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_REQBUFS:
	{
		struct v4l2_requestbuffers *b = (struct v4l2_requestbuffers *)data;

		/* allow reqbufs request in open state */
		rc = msm_vidc_reqbufs(inst, b);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_STREAMON:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		/* allow streamon request in open state */
		rc = msm_vidc_start_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_STREAMOFF:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		/* ignore streamoff request in open state */
		i_vpr_h(inst, "%s: streamoff of (%s) ignored in state (%s)\n",
			__func__, v4l2_type_name(q->type), state_name(inst->state));
		break;
	}
	case MSM_VIDC_CMD_START:
	{
		/* disallow start cmd request in open state */
		i_vpr_e(inst, "%s: (%s) not allowed, sub_state (%s)\n",
			__func__, event_name(event), inst->sub_state_name);

		return -EBUSY;
	}
	case MSM_VIDC_CMD_STOP:
	{
		/* ignore stop cmd request in open state */
		i_vpr_h(inst, "%s: (%s) ignored, sub_state (%s)\n",
			__func__, event_name(event), inst->sub_state_name);
		break;
	}
	case MSM_VIDC_BUF_QUEUE:
	{
		struct msm_vidc_buffer *buf = (struct msm_vidc_buffer *)data;

		/* defer qbuf request in open state */
		print_vidc_buffer(VIDC_LOW, "low ", "qbuf deferred", inst, buf);
		break;
	}
	default:
	{
		i_vpr_e(inst, "%s: unexpected event %s\n", __func__, event_name(event));
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_input_streaming_state(struct msm_vidc_inst *inst,
	enum msm_vidc_event event, void *data)
{
	int rc = 0;

	/* inst must be locked */
	rc = __strict_inst_check(inst, __func__);
	if (rc) {
		i_vpr_e(inst, "%s(): inst was not locked\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case MSM_VIDC_BUF_QUEUE:
	{
		struct msm_vidc_buffer *buf = (struct msm_vidc_buffer *)data;

		/* defer meta port */
		if (buf->type == MSM_VIDC_BUF_INPUT_META || buf->type == MSM_VIDC_BUF_OUTPUT_META) {
			print_vidc_buffer(VIDC_LOW, "low ", "qbuf deferred", inst, buf);
			return 0;
		}

		/* disallow */
		if (!is_input_buffer(buf->type) && !is_output_buffer(buf->type)) {
			i_vpr_e(inst, "%s: invalid buf type %u\n", __func__, buf->type);
			return -EINVAL;
		}

		/* defer output port */
		if (is_output_buffer(buf->type)) {
			print_vidc_buffer(VIDC_LOW, "low ", "qbuf deferred", inst, buf);
			return 0;
		}

		rc = msm_vidc_buf_queue(inst, buf);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_TRY_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *)data;

		/* disallow */
		if (f->type == INPUT_MPLANE || f->type == INPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) port\n",
				__func__, event_name(event), v4l2_type_name(f->type));
			return -EBUSY;
		}

		rc = msm_vidc_try_fmt(inst, f);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_S_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *)data;

		/* disallow */
		if (f->type == INPUT_MPLANE || f->type == INPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) port\n",
				__func__, event_name(event), v4l2_type_name(f->type));
			return -EBUSY;
		}

		rc = msm_vidc_s_fmt(inst, f);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_S_CTRL:
	{
		struct v4l2_ctrl *ctrl = (struct v4l2_ctrl *)data;
		u32 cap_id = msm_vidc_get_cap_id(inst, ctrl->id);

		if (cap_id == INST_CAP_NONE) {
			i_vpr_e(inst, "%s: invalid cap_id %u\n", __func__, cap_id);
			return -EINVAL;
		}

		/* disallow */
		if (is_decode_session(inst)) {
			/* check dynamic allowed if master port is streaming */
			if (!(inst->capabilities[cap_id].flags & CAP_FLAG_DYNAMIC_ALLOWED)) {
				i_vpr_e(inst, "%s: cap_id %#x (%s) not allowed in state %s\n",
					__func__, cap_id, cap_name(cap_id),
					state_name(inst->state));
				return -EINVAL;
			}
		}

		rc = msm_vidc_s_ctrl(inst, ctrl);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_REQBUFS:
	{
		struct v4l2_requestbuffers *b = (struct v4l2_requestbuffers *)data;

		/* disallow */
		if (b->type == INPUT_MPLANE || b->type == INPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) port\n",
				__func__, event_name(event), v4l2_type_name(b->type));
			return -EBUSY;
		}

		rc = msm_vidc_reqbufs(inst, b);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_STREAMON:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		/* disallow */
		if (q->type == INPUT_MPLANE || q->type == INPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) type\n",
				__func__, event_name(event), v4l2_type_name(q->type));
			return -EBUSY;
		}

		rc = msm_vidc_start_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_STREAMOFF:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		/* ignore */
		if (q->type == OUTPUT_MPLANE || q->type == OUTPUT_META_PLANE) {
			i_vpr_h(inst, "%s: streamoff of (%s) ignored in state (%s)\n",
				__func__, v4l2_type_name(q->type), state_name(inst->state));
			return 0;
		}

		/* disallow */
		if (q->type == INPUT_META_PLANE) {
			i_vpr_e(inst, "%s: streamoff of (%s) not allowed in state (%s)\n",
				__func__, v4l2_type_name(q->type), state_name(inst->state));
			return -EINVAL;
		}

		/* sanitize type field */
		if (q->type != INPUT_MPLANE) {
			i_vpr_e(inst, "%s: invalid type %d\n", __func__, q->type);
			return -EINVAL;
		}

		rc = msm_vidc_stop_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_CMD_START:
	{
		/* disallow if START called for non DRC/drain cases */
		if (!is_drc_pending(inst) && !is_drain_pending(inst)) {
			i_vpr_e(inst, "%s: (%s) not allowed, sub_state (%s)\n",
				__func__, event_name(event), inst->sub_state_name);
			return -EBUSY;
		}

		/* client would call start(resume) to complete DRC/drain sequence */
		rc = msm_vidc_start_cmd(inst);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_CMD_STOP:
	{
		/* back to back drain not allowed */
		if (is_sub_state(inst, MSM_VIDC_DRAIN)) {
			i_vpr_e(inst, "%s: drain (%s) not allowed, sub_state (%s)\n\n",
				__func__, event_name(event), inst->sub_state_name);
			return -EBUSY;
		}

		rc = msm_vidc_stop_cmd(inst);
		if (rc)
			return rc;
		break;
	}
	default:
	{
		i_vpr_e(inst, "%s: unexpected event %s\n", __func__, event_name(event));
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_output_streaming_state(struct msm_vidc_inst *inst,
	enum msm_vidc_event event, void *data)
{
	int rc = 0;

	/* inst must be locked */
	rc = __strict_inst_check(inst, __func__);
	if (rc) {
		i_vpr_e(inst, "%s(): inst was not locked\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case MSM_VIDC_BUF_QUEUE:
	{
		struct msm_vidc_buffer *buf = (struct msm_vidc_buffer *)data;

		/* defer meta port */
		if (buf->type == MSM_VIDC_BUF_INPUT_META || buf->type == MSM_VIDC_BUF_OUTPUT_META) {
			print_vidc_buffer(VIDC_LOW, "low ", "qbuf deferred", inst, buf);
			return 0;
		}

		/* disallow */
		if (!is_input_buffer(buf->type) && !is_output_buffer(buf->type)) {
			i_vpr_e(inst, "%s: invalid buf type %u\n", __func__, buf->type);
			return -EINVAL;
		}

		/* defer input port */
		if (is_input_buffer(buf->type)) {
			print_vidc_buffer(VIDC_LOW, "low ", "qbuf deferred", inst, buf);
			return 0;
		}

		rc = msm_vidc_buf_queue(inst, buf);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_TRY_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *)data;

		/* disallow */
		if (f->type == OUTPUT_MPLANE || f->type == OUTPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) port\n",
				__func__, event_name(event), v4l2_type_name(f->type));
			return -EBUSY;
		}

		rc = msm_vidc_try_fmt(inst, f);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_S_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *)data;

		/* disallow */
		if (f->type == OUTPUT_MPLANE || f->type == OUTPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) port\n",
				__func__, event_name(event), v4l2_type_name(f->type));
			return -EBUSY;
		}

		rc = msm_vidc_s_fmt(inst, f);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_S_CTRL:
	{
		struct v4l2_ctrl *ctrl = (struct v4l2_ctrl *)data;
		u32 cap_id = msm_vidc_get_cap_id(inst, ctrl->id);

		if (cap_id == INST_CAP_NONE) {
			i_vpr_e(inst, "%s: invalid cap_id %u\n", __func__, cap_id);
			return -EINVAL;
		}

		/* disallow */
		if (is_encode_session(inst)) {
			/* check dynamic allowed if master port is streaming */
			if (!(inst->capabilities[cap_id].flags & CAP_FLAG_DYNAMIC_ALLOWED)) {
				i_vpr_e(inst, "%s: cap_id %#x not allowed in state %s\n",
					__func__, cap_id, state_name(inst->state));
				return -EINVAL;
			}
		}

		rc = msm_vidc_s_ctrl(inst, ctrl);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_REQBUFS:
	{
		struct v4l2_requestbuffers *b = (struct v4l2_requestbuffers *)data;

		/* disallow */
		if (b->type == OUTPUT_MPLANE || b->type == OUTPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) port\n",
				__func__, event_name(event), v4l2_type_name(b->type));
			return -EBUSY;
		}

		rc = msm_vidc_reqbufs(inst, b);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_STREAMON:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		/* disallow */
		if (q->type == OUTPUT_MPLANE || q->type == OUTPUT_META_PLANE) {
			i_vpr_e(inst, "%s: (%s) not allowed for (%s) type\n",
				__func__, event_name(event), v4l2_type_name(q->type));
			return -EBUSY;
		}

		rc = msm_vidc_start_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_STREAMOFF:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		/* ignore */
		if (q->type == INPUT_MPLANE || q->type == INPUT_META_PLANE) {
			i_vpr_h(inst, "%s: streamoff of (%s) ignored in state (%s)\n",
				__func__, v4l2_type_name(q->type), state_name(inst->state));
			return 0;
		}

		/* disallow */
		if (q->type == OUTPUT_META_PLANE) {
			i_vpr_e(inst, "%s: streamoff of (%s) not allowed in state (%s)\n",
				__func__, v4l2_type_name(q->type), state_name(inst->state));
			return -EINVAL;
		}

		/* sanitize type field */
		if (q->type != OUTPUT_MPLANE) {
			i_vpr_e(inst, "%s: invalid type %d\n", __func__, q->type);
			return -EINVAL;
		}

		rc = msm_vidc_stop_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_CMD_START:
	{
		/* disallow if START called for non DRC/drain cases */
		if (!is_drc_pending(inst) && !is_drain_pending(inst)) {
			i_vpr_e(inst, "%s: (%s) not allowed, sub_state (%s)\n",
				__func__, event_name(event), inst->sub_state_name);
			return -EBUSY;
		}

		/* client would call start(resume) to complete DRC/drain sequence */
		rc = msm_vidc_start_cmd(inst);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_CMD_STOP:
	{
		/* drain not allowed as input is not streaming */
		i_vpr_e(inst, "%s: drain (%s) not allowed, sub state %s\n",
			__func__, event_name(event), inst->sub_state_name);
		return -EBUSY;
	}
	default: {
		i_vpr_e(inst, "%s: unexpected event %s\n", __func__, event_name(event));
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_streaming_state(struct msm_vidc_inst *inst,
	enum msm_vidc_event event, void *data)
{
	int rc = 0;

	/* inst must be locked */
	rc = __strict_inst_check(inst, __func__);
	if (rc) {
		i_vpr_e(inst, "%s(): inst was not locked\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case MSM_VIDC_BUF_QUEUE:
	{
		struct msm_vidc_buffer *buf = (struct msm_vidc_buffer *)data;

		/* defer meta port */
		if (buf->type == MSM_VIDC_BUF_INPUT_META || buf->type == MSM_VIDC_BUF_OUTPUT_META) {
			print_vidc_buffer(VIDC_LOW, "low ", "qbuf deferred", inst, buf);
			return 0;
		}

		/* disallow */
		if (!is_input_buffer(buf->type) && !is_output_buffer(buf->type)) {
			i_vpr_e(inst, "%s: invalid buf type %u\n", __func__, buf->type);
			return -EINVAL;
		}

		rc = msm_vidc_buf_queue(inst, buf);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_S_CTRL:
	{
		struct v4l2_ctrl *ctrl = (struct v4l2_ctrl *)data;
		u32 cap_id = msm_vidc_get_cap_id(inst, ctrl->id);

		if (cap_id == INST_CAP_NONE) {
			i_vpr_e(inst, "%s: invalid cap_id %u\n", __func__, cap_id);
			return -EINVAL;
		}

		/* disallow */
		if (!(inst->capabilities[cap_id].flags & CAP_FLAG_DYNAMIC_ALLOWED)) {
			i_vpr_e(inst, "%s: cap_id %#x not allowed in state %s\n",
				__func__, cap_id, state_name(inst->state));
			return -EINVAL;
		}

		rc = msm_vidc_s_ctrl(inst, ctrl);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_STREAMOFF:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		/* disallow */
		if (q->type == INPUT_META_PLANE || q->type == OUTPUT_META_PLANE) {
			i_vpr_e(inst, "%s: streamoff of (%s) not allowed in state (%s)\n",
				__func__, v4l2_type_name(q->type), state_name(inst->state));
			return -EINVAL;
		}

		/* sanitize type field */
		if (q->type != INPUT_MPLANE && q->type != OUTPUT_MPLANE) {
			i_vpr_e(inst, "%s: invalid type %d\n", __func__, q->type);
			return -EINVAL;
		}

		rc = msm_vidc_stop_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_CMD_START:
	{
		/* disallow if START called for non DRC/drain cases */
		if (!is_drc_pending(inst) && !is_drain_pending(inst)) {
			i_vpr_e(inst, "%s: (%s) not allowed, sub_state (%s)\n",
				__func__, event_name(event), inst->sub_state_name);
			return -EBUSY;
		}

		/* client would call start(resume) to complete DRC/drain sequence */
		rc = msm_vidc_start_cmd(inst);
		if (rc)
			return rc;
		break;
	}
	case MSM_VIDC_CMD_STOP:
	{
		/* back to back drain not allowed */
		if (is_sub_state(inst, MSM_VIDC_DRAIN)) {
			i_vpr_e(inst, "%s: drain (%s) not allowed, sub_state (%s)\n\n",
				__func__, event_name(event), inst->sub_state_name);
			return -EBUSY;
		}

		rc = msm_vidc_stop_cmd(inst);
		if (rc)
			return rc;
		break;
	}
	default: {
		i_vpr_e(inst, "%s: unexpected event %s\n", __func__, event_name(event));
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_close_state(struct msm_vidc_inst *inst,
	enum msm_vidc_event event, void *data)
{
	int rc = 0;

	/* inst must be locked */
	rc = __strict_inst_check(inst, __func__);
	if (rc) {
		i_vpr_e(inst, "%s(): inst was not locked\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case MSM_VIDC_STREAMOFF:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		rc = msm_vidc_stop_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	default: {
		i_vpr_e(inst, "%s: unexpected event %s\n", __func__, event_name(event));
		return -EINVAL;
	}
	}

	return rc;
}

static int msm_vidc_error_state(struct msm_vidc_inst *inst,
	enum msm_vidc_event event, void *data)
{
	int rc = 0;

	/* inst must be locked */
	rc = __strict_inst_check(inst, __func__);
	if (rc) {
		i_vpr_e(inst, "%s(): inst was not locked\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case MSM_VIDC_STREAMOFF:
	{
		struct vb2_queue *q = (struct vb2_queue *)data;

		rc = msm_vidc_stop_streaming(inst, q);
		if (rc)
			return rc;
		break;
	}
	default: {
		i_vpr_e(inst, "%s: unexpected event %s\n", __func__, event_name(event));
		return -EINVAL;
	}
	}

	return rc;
}

struct msm_vidc_state_handle {
	enum msm_vidc_state   state;
	int                 (*handle)(struct msm_vidc_inst *inst,
		enum msm_vidc_event event, void *data);
};

static struct msm_vidc_state_handle *msm_vidc_get_state_handle(
	struct msm_vidc_inst *inst,
	enum msm_vidc_state req_state)
{
	int cnt;
	struct msm_vidc_state_handle *inst_state_handle = NULL;
	static struct msm_vidc_state_handle state_handle[] = {
		{MSM_VIDC_OPEN,             msm_vidc_open_state             },
		{MSM_VIDC_INPUT_STREAMING,  msm_vidc_input_streaming_state  },
		{MSM_VIDC_OUTPUT_STREAMING, msm_vidc_output_streaming_state },
		{MSM_VIDC_STREAMING,        msm_vidc_streaming_state        },
		{MSM_VIDC_CLOSE,            msm_vidc_close_state            },
		{MSM_VIDC_ERROR,            msm_vidc_error_state            },
	};

	for (cnt = 0; cnt < ARRAY_SIZE(state_handle); cnt++) {
		if (state_handle[cnt].state == req_state) {
			inst_state_handle = &state_handle[cnt];
			break;
		}
	}

	/* check if req_state does not exist in the table */
	if (cnt == ARRAY_SIZE(state_handle)) {
		i_vpr_e(inst, "%s: invalid state %s\n", __func__, state_name(req_state));
		return inst_state_handle;
	}

	return inst_state_handle;
}

int msm_vidc_update_state(struct msm_vidc_inst *inst,
	enum msm_vidc_state request_state, const char *func)
{
	struct msm_vidc_state_handle *state_handle = NULL;
	int rc = 0;

	/* get inst state handler for requested state */
	state_handle = msm_vidc_get_state_handle(inst, request_state);
	if (!state_handle)
		return -EINVAL;

	if (request_state == MSM_VIDC_ERROR)
		i_vpr_e(inst, FMT_STRING_STATE_CHANGE,
		   func, state_name(request_state), state_name(inst->state));
	else
		i_vpr_h(inst, FMT_STRING_STATE_CHANGE,
		   func, state_name(request_state), state_name(inst->state));

	trace_msm_vidc_common_state_change(inst, func, state_name(inst->state),
			state_name(request_state));

	/* finally update inst state and handler */
	inst->state = state_handle->state;
	inst->event_handle = state_handle->handle;

	return rc;
}

int msm_vidc_change_state(struct msm_vidc_inst *inst,
		enum msm_vidc_state request_state, const char *func)
{
	enum msm_vidc_allow allow;
	int rc;

	if (is_session_error(inst)) {
		i_vpr_h(inst,
			"%s: inst is in bad state, can not change state to %s\n",
			func, state_name(request_state));
		return 0;
	}

	/* current and requested state is same */
	if (inst->state == request_state)
		return 0;

	/* check if requested state movement is allowed */
	allow = msm_vidc_allow_state_change(inst, request_state);
	if (allow != MSM_VIDC_ALLOW) {
		i_vpr_e(inst, "%s: %s state change %s -> %s\n", func,
			allow_name(allow), state_name(inst->state),
			state_name(request_state));
		return (allow == MSM_VIDC_DISALLOW ? -EINVAL : 0);
	}

	/* go ahead and update inst state */
	rc = msm_vidc_update_state(inst, request_state, func);
	if (rc)
		return rc;

	return 0;
}

struct msm_vidc_sub_state_allow {
	enum msm_vidc_state            state;
	enum msm_vidc_allow            allow;
	u32                            sub_state_mask;
};

static int msm_vidc_set_sub_state(struct msm_vidc_inst *inst,
	enum msm_vidc_sub_state sub_state, const char *func)
{
	char sub_state_name[MAX_NAME_LENGTH];
	int cnt, rc = 0;
	static struct msm_vidc_sub_state_allow sub_state_allow[] = {
		/* state, allow, sub_state */
		{MSM_VIDC_OPEN,              MSM_VIDC_DISALLOW,    MSM_VIDC_DRC                 |
								   MSM_VIDC_DRAIN               |
								   MSM_VIDC_DRC_LAST_BUFFER     |
								   MSM_VIDC_DRAIN_LAST_BUFFER   |
								   MSM_VIDC_INPUT_PAUSE         |
								   MSM_VIDC_OUTPUT_PAUSE         },

		{MSM_VIDC_INPUT_STREAMING,   MSM_VIDC_DISALLOW,    MSM_VIDC_DRC_LAST_BUFFER     |
								   MSM_VIDC_DRAIN_LAST_BUFFER   |
								   MSM_VIDC_OUTPUT_PAUSE         },
		{MSM_VIDC_INPUT_STREAMING,   MSM_VIDC_ALLOW,       MSM_VIDC_DRC                 |
								   MSM_VIDC_DRAIN               |
								   MSM_VIDC_INPUT_PAUSE         |
								   MSM_VIDC_FIRST_IPSC           },

		{MSM_VIDC_OUTPUT_STREAMING,  MSM_VIDC_DISALLOW,    MSM_VIDC_DRC                 |
								   MSM_VIDC_DRAIN               |
								   MSM_VIDC_INPUT_PAUSE          },
		{MSM_VIDC_OUTPUT_STREAMING,  MSM_VIDC_ALLOW,       MSM_VIDC_DRC_LAST_BUFFER     |
								   MSM_VIDC_DRAIN_LAST_BUFFER   |
								   MSM_VIDC_OUTPUT_PAUSE         },

		{MSM_VIDC_STREAMING,         MSM_VIDC_ALLOW,       MSM_VIDC_DRC                 |
								   MSM_VIDC_DRAIN               |
								   MSM_VIDC_DRC_LAST_BUFFER     |
								   MSM_VIDC_DRAIN_LAST_BUFFER   |
								   MSM_VIDC_INPUT_PAUSE         |
								   MSM_VIDC_OUTPUT_PAUSE         },

		{MSM_VIDC_CLOSE,             MSM_VIDC_ALLOW,       MSM_VIDC_DRC                 |
								   MSM_VIDC_DRAIN               |
								   MSM_VIDC_DRC_LAST_BUFFER     |
								   MSM_VIDC_DRAIN_LAST_BUFFER   |
								   MSM_VIDC_INPUT_PAUSE         |
								   MSM_VIDC_OUTPUT_PAUSE         },

		{MSM_VIDC_ERROR,             MSM_VIDC_ALLOW,       MSM_VIDC_DRC                 |
								   MSM_VIDC_DRAIN               |
								   MSM_VIDC_DRC_LAST_BUFFER     |
								   MSM_VIDC_DRAIN_LAST_BUFFER   |
								   MSM_VIDC_INPUT_PAUSE         |
								   MSM_VIDC_OUTPUT_PAUSE         },
	};

	/* no substate to update */
	if (!sub_state)
		return 0;

	/* check if any substate is disallowed */
	for (cnt = 0; cnt < ARRAY_SIZE(sub_state_allow); cnt++) {
		/* skip other states */
		if (sub_state_allow[cnt].state != inst->state)
			continue;

		/* continue if not disallowed */
		if (sub_state_allow[cnt].allow != MSM_VIDC_DISALLOW)
			continue;

		if (sub_state_allow[cnt].sub_state_mask & sub_state) {
			prepare_sub_state_name(sub_state, sub_state_name, sizeof(sub_state_name));
			i_vpr_e(inst, "%s: state (%s), disallow substate (%s)\n",
				func, state_name(inst->state), sub_state_name);
			return -EINVAL;
		}
	}

	/* remove ignorable substates from a given substate */
	for (cnt = 0; cnt < ARRAY_SIZE(sub_state_allow); cnt++) {
		/* skip other states */
		if (sub_state_allow[cnt].state != inst->state)
			continue;

		/* continue if not ignored */
		if (sub_state_allow[cnt].allow != MSM_VIDC_IGNORE)
			continue;

		if (sub_state_allow[cnt].sub_state_mask & sub_state) {
			prepare_sub_state_name(sub_state, sub_state_name, sizeof(sub_state_name));
			i_vpr_h(inst, "%s: state (%s), ignore substate (%s)\n",
				func, state_name(inst->state), sub_state_name);

			/* remove ignorable substate bits from actual */
			sub_state &= ~(sub_state_allow[cnt].sub_state_mask & sub_state);
			break;
		}
	}

	/* check if all substate bits are allowed */
	for (cnt = 0; cnt < ARRAY_SIZE(sub_state_allow); cnt++) {
		/* skip other states */
		if (sub_state_allow[cnt].state != inst->state)
			continue;

		/* continue if not allowed */
		if (sub_state_allow[cnt].allow != MSM_VIDC_ALLOW)
			continue;

		if ((sub_state_allow[cnt].sub_state_mask & sub_state) != sub_state) {
			prepare_sub_state_name(sub_state, sub_state_name, sizeof(sub_state_name));
			i_vpr_e(inst, "%s: state (%s), not all substates allowed (%s)\n",
				func, state_name(inst->state), sub_state_name);
			return -EINVAL;
		}
	}

	/* update substate */
	inst->sub_state |= sub_state;

	return rc;
}

int msm_vidc_change_sub_state(struct msm_vidc_inst *inst,
		enum msm_vidc_sub_state clear_sub_state,
		enum msm_vidc_sub_state set_sub_state, const char *func)
{
	enum msm_vidc_sub_state prev_sub_state;
	int rc = 0;

	if (is_session_error(inst)) {
		i_vpr_h(inst,
			"%s: inst is in bad state, can not change sub state\n", func);
		return 0;
	}

	/* final value will not change */
	if (!clear_sub_state && !set_sub_state)
		return 0;

	/* sanitize clear & set value */
	if ((clear_sub_state & set_sub_state) ||
		(set_sub_state > MSM_VIDC_MAX_SUB_STATE_VALUE) ||
		(clear_sub_state > MSM_VIDC_MAX_SUB_STATE_VALUE)) {
		i_vpr_e(inst, "%s: invalid sub states to clear %#x or set %#x\n",
			func, clear_sub_state, set_sub_state);
		return -EINVAL;
	}

	prev_sub_state = inst->sub_state;

	/* set sub state */
	rc = msm_vidc_set_sub_state(inst, set_sub_state, __func__);
	if (rc)
		return rc;

	/* clear sub state */
	inst->sub_state &= ~clear_sub_state;

	/* print substates only when there is a change */
	if (inst->sub_state != prev_sub_state) {
		rc = prepare_sub_state_name(inst->sub_state, inst->sub_state_name,
			sizeof(inst->sub_state_name));
		if (!rc)
			i_vpr_h(inst, "%s: state %s and sub state changed to %s\n",
				func, state_name(inst->state), inst->sub_state_name);
	}

	return 0;
}
