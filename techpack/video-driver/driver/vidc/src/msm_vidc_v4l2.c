// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "msm_vidc_v4l2.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_core.h"
#include "msm_vidc_inst.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_debug.h"
#include "msm_vidc.h"
#include "msm_vidc_events.h"

extern struct msm_vidc_core *g_core;

static struct msm_vidc_inst *get_vidc_inst(struct file *filp, void *fh)
{
	if (!filp || !filp->private_data)
		return NULL;
	return container_of(filp->private_data,
					struct msm_vidc_inst, fh);
}

unsigned int msm_v4l2_poll(struct file *filp, struct poll_table_struct *pt)
{
	int poll = 0;
	struct msm_vidc_inst *inst = get_vidc_inst(filp, NULL);

	inst = get_inst_ref(g_core, inst);
	if (!inst) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return POLLERR;
	}
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		poll = POLLERR;
		goto exit;
	}
	poll = msm_vidc_poll((void *)inst, filp, pt);
	if (poll)
		goto exit;

exit:
	put_inst(inst);
	return poll;
}

int msm_v4l2_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct msm_video_device *vid_dev =
		container_of(vdev, struct msm_video_device, vdev);
	struct msm_vidc_core *core = video_drvdata(filp);
	struct msm_vidc_inst *inst;

	trace_msm_v4l2_vidc_open("START", NULL);
	inst = msm_vidc_open(core, vid_dev->type);
	if (!inst) {
		d_vpr_e("Failed to create instance, type = %d\n",
			vid_dev->type);
		trace_msm_v4l2_vidc_open("END", NULL);
		return -ENOMEM;
	}
	filp->private_data = &(inst->fh);
	trace_msm_v4l2_vidc_open("END", inst);
	return 0;
}

int msm_v4l2_close(struct file *filp)
{
	int rc = 0;
	struct msm_vidc_inst *inst;

	inst = get_vidc_inst(filp, NULL);
	if (!inst) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	trace_msm_v4l2_vidc_close("START", inst);

	rc = msm_vidc_close(inst);
	filp->private_data = NULL;
	trace_msm_v4l2_vidc_close("END", NULL);
	return rc;
}

int msm_v4l2_querycap(struct file *filp, void *fh,
			struct v4l2_capability *cap)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !cap) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_querycap((void *)inst, cap);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_enum_fmt(struct file *filp, void *fh,
					struct v4l2_fmtdesc *f)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !f) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_enum_fmt((void *)inst, f);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_try_fmt(struct file *filp, void *fh, struct v4l2_format *f)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !f) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = inst->event_handle(inst, MSM_VIDC_TRY_FMT, f);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_s_fmt(struct file *filp, void *fh,
					struct v4l2_format *f)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !f) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = inst->event_handle(inst, MSM_VIDC_S_FMT, f);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_g_fmt(struct file *filp, void *fh,
					struct v4l2_format *f)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !f) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_g_fmt((void *)inst, f);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_s_selection(struct file *filp, void *fh,
					struct v4l2_selection *s)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !s) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_s_selection((void *)inst, s);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_g_selection(struct file *filp, void *fh,
					struct v4l2_selection *s)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !s) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_g_selection((void *)inst, s);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_s_parm(struct file *filp, void *fh,
					struct v4l2_streamparm *a)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !a) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_s_param((void *)inst, a);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_g_parm(struct file *filp, void *fh,
					struct v4l2_streamparm *a)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !a) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_g_param((void *)inst, a);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_reqbufs(struct file *filp, void *fh,
				struct v4l2_requestbuffers *b)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !b) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = inst->event_handle(inst, MSM_VIDC_REQBUFS, b);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_querybuf(struct file *filp, void *fh,
				struct v4l2_buffer *b)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !b) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_querybuf((void *)inst, b);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_create_bufs(struct file *filp, void *fh,
				struct v4l2_create_buffers *b)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !b) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_create_bufs((void *)inst, b);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_prepare_buf(struct file *filp, void *fh,
				struct v4l2_buffer *b)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	struct video_device *vdev = video_devdata(filp);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !b) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_prepare_buf((void *)inst, vdev->v4l2_dev->mdev, b);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_qbuf(struct file *filp, void *fh,
				struct v4l2_buffer *b)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	struct video_device *vdev = video_devdata(filp);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !b) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	/*
	 * [1] If request_fd enabled, msm_vb2_buf_queue() is not called from here.
	 *   instead it's called as part of msm_v4l2_request_queue().
	 *   Hence inst lock should be acquired in common function i.e
	 *   msm_vb2_buf_queue, to handle both requests and non-request
	 *   scenarios.
	 * [2] If request_fd is disabled, inst_lock can be acquired here.
	 *   Acquiring inst_lock from here will ensure RO list insertion
	 *   and deletion i.e. attach/map will happen under lock.
	 * Currently, request_fd is disabled. Therefore, acquire inst_lock
	 * from this function to ensure RO list insertion/updation is under
	 * lock to avoid stability usecase.
	 */
	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	rc = msm_vidc_qbuf(inst, vdev->v4l2_dev->mdev, b);
	if (rc)
		goto exit;

exit:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_dqbuf(struct file *filp, void *fh,
				struct v4l2_buffer *b)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !b) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	rc = msm_vidc_dqbuf(inst, b);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_streamon(struct file *filp, void *fh,
				enum v4l2_buf_type i)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto exit;
	}

	rc = msm_vidc_streamon((void *)inst, i);
	if (rc)
		goto exit;

exit:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_streamoff(struct file *filp, void *fh,
				enum v4l2_buf_type i)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	rc = msm_vidc_streamoff((void *)inst, i);
	if (rc)
		i_vpr_e(inst, "%s: msm_vidc_stramoff failed\n", __func__);

	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	struct msm_vidc_inst *inst;
	int rc = 0;

	inst = container_of(fh, struct msm_vidc_inst, fh);
	inst = get_inst_ref(g_core, inst);
	if (!inst || !sub) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_subscribe_event((void *)inst, sub);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_unsubscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	struct msm_vidc_inst *inst;
	int rc = 0;

	inst = container_of(fh, struct msm_vidc_inst, fh);
	inst = get_inst_ref(g_core, inst);
	if (!inst || !sub) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	rc = msm_vidc_unsubscribe_event((void *)inst, sub);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_try_decoder_cmd(struct file *filp, void *fh,
			     struct v4l2_decoder_cmd *dec)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !dec) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_try_cmd(inst, (union msm_v4l2_cmd *)dec);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_decoder_cmd(struct file *filp, void *fh,
				struct v4l2_decoder_cmd *dec)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	enum msm_vidc_event event;
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	if (!dec) {
		i_vpr_e(inst, "%s: invalid params\n", __func__);
		rc = -EINVAL;
		goto unlock;
	}
	if (dec->cmd != V4L2_DEC_CMD_START &&
		dec->cmd != V4L2_DEC_CMD_STOP) {
		i_vpr_e(inst, "%s: invalid cmd %#x\n", __func__, dec->cmd);
		rc = -EINVAL;
		goto unlock;
	}
	event = (dec->cmd == V4L2_DEC_CMD_START ? MSM_VIDC_CMD_START : MSM_VIDC_CMD_STOP);
	rc = inst->event_handle(inst, event, NULL);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_try_encoder_cmd(struct file *filp, void *fh,
			     struct v4l2_encoder_cmd *enc)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !enc) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_try_cmd(inst, (union msm_v4l2_cmd *)enc);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_encoder_cmd(struct file *filp, void *fh,
				struct v4l2_encoder_cmd *enc)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	enum msm_vidc_event event;
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	if (!enc) {
		i_vpr_e(inst, "%s: invalid params\n", __func__);
		rc = -EINVAL;
		goto unlock;
	}
	if (enc->cmd != V4L2_ENC_CMD_START &&
		enc->cmd != V4L2_ENC_CMD_STOP) {
		i_vpr_e(inst, "%s: invalid cmd %#x\n", __func__, enc->cmd);
		rc = -EINVAL;
		goto unlock;
	}
	event = (enc->cmd == V4L2_ENC_CMD_START ? MSM_VIDC_CMD_START : MSM_VIDC_CMD_STOP);
	rc = inst->event_handle(inst, event, NULL);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_enum_framesizes(struct file *filp, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !fsize) {
		d_vpr_e("%s: invalid params: %pK %pK\n",
				__func__, inst, fsize);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_enum_framesizes((void *)inst, fsize);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_enum_frameintervals(struct file *filp, void *fh,
				struct v4l2_frmivalenum *fival)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !fival) {
		d_vpr_e("%s: invalid params: %pK %pK\n",
			__func__, inst, fival);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_enum_frameintervals((void *)inst, fival);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_queryctrl(struct file *filp, void *fh,
	struct v4l2_queryctrl *ctrl)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !ctrl) {
		d_vpr_e("%s: invalid instance\n", __func__);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_query_ctrl((void *)inst, ctrl);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_querymenu(struct file *filp, void *fh,
	struct v4l2_querymenu *qmenu)
{
	struct msm_vidc_inst *inst = get_vidc_inst(filp, fh);
	int rc = 0;

	inst = get_inst_ref(g_core, inst);
	if (!inst || !qmenu) {
		d_vpr_e("%s: invalid params %pK %pK\n",
			__func__, inst, qmenu);
		return -EINVAL;
	}

	client_lock(inst, __func__);
	inst_lock(inst, __func__);
	if (is_session_error(inst)) {
		i_vpr_e(inst, "%s: inst in error state\n", __func__);
		rc = -EBUSY;
		goto unlock;
	}
	rc = msm_vidc_query_menu((void *)inst, qmenu);
	if (rc)
		goto unlock;

unlock:
	inst_unlock(inst, __func__);
	client_unlock(inst, __func__);
	put_inst(inst);

	return rc;
}

int msm_v4l2_request_validate(struct media_request *req)
{
	d_vpr_l("%s()\n", __func__);
	return vb2_request_validate(req);
}

void msm_v4l2_request_queue(struct media_request *req)
{
	d_vpr_l("%s()\n", __func__);
	v4l2_m2m_request_queue(req);
}

void msm_v4l2_m2m_device_run(void *priv)
{
	d_vpr_l("%s()\n", __func__);
}

void msm_v4l2_m2m_job_abort(void *priv)
{
	struct msm_vidc_inst *inst = priv;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return;
	}
	i_vpr_h(inst, "%s: m2m job aborted\n", __func__);
	v4l2_m2m_job_finish(inst->m2m_dev, inst->m2m_ctx);
}
