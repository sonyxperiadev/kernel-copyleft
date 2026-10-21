/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kthread.h>
#include "cvp_vm_msgq.h"
#include "msm_cvp_debug.h"

/**
 * cvp_msgq_receiver - thread function that receive msg from gunyah msgq
 * data: cvp_msgq_drv pointer
 *
 * Note: single thread. If the sub-function or global data used in this
 *       function is also used somehwere else, please add rx_lock.
 */
static int cvp_msgq_receiver(void *data)
{
	struct cvp_msgq_drv *msgq_drv = data;

	struct cvp_ipc_msg *msg_ptr;
	size_t size;
	bool is_resp;
	/**
	* true: response received from remote VM, cmd initiated from LOCAL VM;
	* false: cmd initiated from REMOTE VM;
	*/
	int rc = -1;
	if (IS_ERR_OR_NULL(msgq_drv))
		return -EINVAL;

	msg_ptr = kzalloc(sizeof(*msg_ptr), GFP_KERNEL);
	if (!msg_ptr) {
		dprintk(CVP_ERR, "%s: fail to allocate mem\n", __func__);
		return -ENOMEM;
	}

	while (true) {
		rc = gh_msgq_recv(msgq_drv->config.handle, msg_ptr,
			sizeof(*msg_ptr), &size, 0);

		if (rc != 0 ) {
			dprintk(CVP_ERR,
			"%s: gh_msgq_recv fail rc=%d handle=%#x msg_ptr=%#x\n",
			__func__, rc, msgq_drv->config.handle, msg_ptr);

			if (rc != -EAGAIN) {
				kfree(msg_ptr);
				return rc;
			}
			continue;
		}

		is_resp = (msg_ptr->type &
			CVP_IPC_MSG_TYPE_DIR_CHECK) ? true : false;

		if (is_resp == false) {
			dprintk(CVP_VM,
				"%s: gh_msgq_recv cmd from remote VM\n",
				__func__);

			if (msgq_drv->pending_local_cmd.type == 0) {

				/* copy ipc message to local cmd */
				memcpy(&msgq_drv->pending_local_cmd,
					msg_ptr, sizeof(struct cvp_ipc_msg));

				/* toggle the direction bit*/
				msgq_drv->pending_local_cmd.type ^=
					CVP_IPC_MSG_TYPE_DIR_CHECK;

				/* TODO: call client function ptr to process */

				memcpy(msg_ptr, &msgq_drv->pending_local_cmd,
					sizeof(struct cvp_ipc_msg));

				/* 4: elements before actual data in cvp_ipc_msg*/
				size = (4 + msgq_drv->pending_local_cmd.len)<<2;

				/* sanity check on size information */
				if (size > GH_MSGQ_MAX_MSG_SIZE_BYTES) {
					dprintk(CVP_ERR,
						"%s: msg size %d exceed max size supported %d \n",
						__func__, size, GH_MSGQ_MAX_MSG_SIZE_BYTES);
					rc = -E2BIG;
					msgq_drv->pending_local_cmd.type = 0;
					continue;
				}

				/* send it back to the remote VM as response */
				rc = gh_msgq_send(msgq_drv->config.handle,
				msg_ptr, size, GH_MSGQ_TX_PUSH);

				if (rc < 0) {
					dprintk(CVP_ERR,
					"%s: failed gh_msgq_send rc %d \n",
					__func__, rc);
				}

				/* flag the source is released */
				msgq_drv->pending_local_cmd.type = 0;
			}
			else {
				dprintk(CVP_ERR,
				"%s: Msg rejected, local cmd in use type %d\n",
				__func__, msgq_drv->pending_local_cmd.type);
			}
		}
		else {
			dprintk(CVP_VM,
			"%s: gh_msgq_recv respond type from remote VM\n",
			__func__);

			if ((msg_ptr->type & CVP_IPC_MSG_TYPE_ACT_CHECK) !=
				msgq_drv->pending_remote_rsp.type) {

				dprintk(CVP_ERR,
				"%s: Msg disgard,recv type %d, pend local %d\n",
				__func__, msg_ptr->type,
				msgq_drv->pending_remote_rsp.type);
			}
			else {
				/* memcpy received data to pending_remote_rsp */
				memcpy(&msgq_drv->pending_remote_rsp, msg_ptr,
					sizeof(struct cvp_ipc_msg));

				/* clear direction bit of pending_remote_rsp */
				msgq_drv->pending_remote_rsp.type &=
					(~CVP_IPC_MSG_TYPE_DIR_CHECK);

				/* complete for cmd initiated from local VM */
				complete(&msgq_drv->completions[
					msgq_drv->pending_remote_rsp.type - 1]);
			}
		}
	}
	return 0;
}

static int cvp_complete_msgq_init(struct cvp_msgq_drv *msgq_drv)
{
	int i;

	msgq_drv->receiver_thread = kthread_run(
			cvp_msgq_receiver,
			(void *)msgq_drv,
			"CVP msgq receiver");
	if (IS_ERR_OR_NULL(msgq_drv->receiver_thread)) {
		dprintk(CVP_ERR, "Failed to start msgq receiver thread\n");
		return -EINVAL;
	}

	mutex_init(&msgq_drv->ipc_lock);

	for (i = 0; i <= (CVP_MAX_IPC_CMD - 1); i++)
		init_completion(&msgq_drv->completions[i]);

	return 0;
}

#ifndef CONFIG_EVA_TVM
static int cvp_msgq_cb(struct notifier_block *nb,
		unsigned long cmd, void *data)
{
	struct gh_rm_notif_vm_status_payload *vm_status_payload;
	struct cvp_gh_msgq_config *msgq_config;
	struct cvp_msgq_drv *msgq_drv;
	gh_vmid_t peer_vmid;
	gh_vmid_t self_vmid;
	int rc;

	if (IS_ERR_OR_NULL(nb))
		return -EINVAL;

	msgq_config = container_of(nb, struct cvp_gh_msgq_config, rm_nb);
	msgq_drv = container_of(msgq_config, struct cvp_msgq_drv, config);

	if (cmd != GH_RM_NOTIF_VM_STATUS)
		return NOTIFY_DONE;

	/**
	 * Check VM status, only GH_TRUSTED_VM notification activate
	 * Gunyah msgq registration
	 */
	vm_status_payload = (struct gh_rm_notif_vm_status_payload *)data;

	if (vm_status_payload->vm_status != GH_RM_VM_STATUS_READY)
		return -12;

	if (gh_rm_get_vmid(msgq_config->peer_id, &peer_vmid))
		return -13;

	if (gh_rm_get_vmid(GH_PRIMARY_VM, &self_vmid))
		return -14;

	if (peer_vmid != vm_status_payload->vmid)
		return NOTIFY_DONE;

	dprintk(CVP_VM, "%s: vmid=%d, peer_vmid=%d\n",
			__func__, vm_status_payload->vmid, peer_vmid);

	if (msgq_config->handle)
		return -15;

	msgq_config->handle = gh_msgq_register(GH_MSGQ_LABEL_EVA);
	if (IS_ERR(msgq_config->handle)) {
		rc = PTR_ERR(msgq_drv->config.handle);
		dprintk(CVP_ERR, "PVM failed to register msgq %d\n", rc);
		return rc;
	}
	dprintk(CVP_VM, "%s: gh_msgq_register handle: %x\n",
			__func__, msgq_config->handle);

	rc = cvp_complete_msgq_init(msgq_drv);

	return rc;
}
#endif

static int cvp_msgq_init(struct cvp_msgq_drv *msgq_drv)
{
	int rc = 0;

	msgq_drv->config.label = GH_MSGQ_LABEL_EVA;
	msgq_drv->config.handle = NULL;
#ifndef CONFIG_EVA_TVM
	/* PVM init */
	msgq_drv->config.peer_id = GH_TRUSTED_VM;
	msgq_drv->config.rm_nb.notifier_call = cvp_msgq_cb;

	rc = gh_rm_register_notifier(&msgq_drv->config.rm_nb);
	if (rc) {
		dprintk(CVP_ERR, "PVM Fail register msgq notifier %d\n", rc);
		return rc;
	}
	dprintk(CVP_VM, "%s:  gh_rm_register_notifier\n", __func__);
#else
	/* TVM init */
	msgq_drv->config.handle = gh_msgq_register(GH_MSGQ_LABEL_EVA);
	if (IS_ERR(msgq_drv->config.handle)) {
		rc = PTR_ERR(msgq_drv->config.handle);
		dprintk(CVP_ERR, "TVM failed to register msgq %d\n", rc);
		return rc;
	}
	rc = cvp_complete_msgq_init(msgq_drv);
#endif
	return rc;
}

static int cvp_msgq_deinit(struct cvp_msgq_drv *msgq_drv)
{
	if (msgq_drv->receiver_thread)
		kthread_stop(msgq_drv->receiver_thread);

	return 0;
}

static int cvp_msgq_send_cmd(struct cvp_msgq_drv *msgq_drv,
		void *msg, size_t msg_size)
{
	int rc = -1;

	struct cvp_ipc_msg *msg_ptr = msg;

	if (!msgq_drv->config.handle) {
		dprintk(CVP_ERR, "%s: Invalid msgq handle\n", __func__);
		rc = -EINVAL;
		goto err_param_check;
	}

	if (msg_size > GH_MSGQ_MAX_MSG_SIZE_BYTES) {
		dprintk(CVP_ERR,
			"%s: msg size %d exceed max size supported %d \n",
			__func__, msg_size, GH_MSGQ_MAX_MSG_SIZE_BYTES);
		rc = -E2BIG;
		goto err_param_check;
	}

	mutex_lock(&msgq_drv->ipc_lock);

	/* init case: only allow sending msg sequentially */
	if (msgq_drv->pending_remote_rsp.type &
		CVP_IPC_MSG_TYPE_ACT_CHECK) {
		rc = -EPERM;
		dprintk(CVP_ERR,
			"%s: Msg rejected, local rsp occupied.\n",
			__func__);

		goto err_valid_check;
	}

	/* book keeping type bits in pending_remote_rsp */
	msgq_drv->pending_remote_rsp.type = msg_ptr->type;

	rc = gh_msgq_send(msgq_drv->config.handle,
			msg_ptr, msg_size, GH_MSGQ_TX_PUSH);
	if (rc < 0) {
		dprintk(CVP_ERR,
			"%s: failed with gh_msgq_send with rc %d \n",
			__func__, rc);
		goto err_gh_send;
	}

	/* wait for completion */
	if (!wait_for_completion_timeout(
		&msgq_drv->completions[msgq_drv->pending_remote_rsp.type - 1],
		msecs_to_jiffies(CVP_VM_RESPONSE_TIMEOUT))) {
		dprintk(CVP_ERR, "%s cvp ipc msg type %d timeout\n",
			__func__, msgq_drv->pending_remote_rsp.type-1);
		rc = -ETIMEDOUT;
	}

	/* copy pending_remote_rsp content to msg (inout param)*/
	memcpy(msg, &msgq_drv->pending_remote_rsp,
			sizeof(struct cvp_ipc_msg));

	/*  clear type bits to indicate resource is avaialbel */
	msgq_drv->pending_remote_rsp.type = 0;

	mutex_unlock(&msgq_drv->ipc_lock);

	return rc;

err_gh_send:
err_valid_check:
	mutex_unlock(&msgq_drv->ipc_lock);
err_param_check:
	return rc;
}

static struct cvp_msgq_ops msgq_ops = {
	.msgq_init = cvp_msgq_init,
	.msgq_deinit = cvp_msgq_deinit,
	.msgq_send = cvp_msgq_send_cmd,
	.msgq_receive = NULL,
};

struct cvp_msgq_drv cvp_ipc_msgq = {
	.ops = &msgq_ops,
};
