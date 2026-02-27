/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CVP_VM_MSGQ_H_
#define _CVP_VM_MSGQ_H_

#include <linux/types.h>
#include <linux/gunyah/gh_msgq.h>
#include <linux/gunyah/gh_rm_drv.h>
#include "cvp_comm_def.h"

#define MAX_CVP_IPC_LEN 16

#define CVP_VM_RESPONSE_TIMEOUT			300

#define CVP_IPC_MSG_TYPE_DIR_CHECK	0x10000000	/* direction check */
#define CVP_IPC_MSG_TYPE_ACT_CHECK	0x00000011  /* action check */

enum CVP_IPC_MSG_TYPE {
	REQUEST_SESS_CTRL = 1,
	RELEASE_SESS_CTRL = 2,
	REQUEST_EVA_RESET = 3,
	RECLAIM_SESS_CTRL = 4,	/* Only PVM can reclaim sesession control */
	CVP_MAX_IPC_CMD = 5,
};

struct cvp_ipc_msg {
	/* type format:
	 *	bit 31: 0->Initiated command; 1->Response to remote command
	 *	bit 2~0: CVP_IPC_MSG_TYPE
	 */
	uint32_t type;
	uint32_t ver;
	uint32_t len;
	uint32_t resv;
	uint32_t data[MAX_CVP_IPC_LEN];
};

struct cvp_gh_msgq_config {
	int  peer_id;
	int  label;
	void *handle;
	struct notifier_block rm_nb;
};

struct cvp_msgq_ops;

struct cvp_msgq_drv {
	struct mutex ipc_lock;	/* Mutex for sending MSG */
	struct cvp_gh_msgq_config config;
	struct task_struct *receiver_thread;
	struct completion completions[CVP_MAX_IPC_CMD + 1];
	/*
	 * pending_local_cmd: the command is being processed locally.
	 * The command is a request sent from remote VM
	 */
	struct cvp_ipc_msg pending_local_cmd;
	/*
	 * pending_remote_rsp: the command is being processing remotely.
	 * The command is a request sent by local VM
	 */
	struct cvp_ipc_msg pending_remote_rsp;
	struct cvp_msgq_ops *ops;
};

struct cvp_msgq_ops {
	int (*msgq_init)(struct cvp_msgq_drv *msgq_drv);
	int (*msgq_send)(struct cvp_msgq_drv *msgq_drv, void *msg,
			size_t msg_size);
	int (*msgq_receive)(struct cvp_msgq_drv *msgq_drv);
	int (*msgq_deinit)(struct cvp_msgq_drv *msgq_drv);
};

extern struct cvp_msgq_drv cvp_ipc_msgq;
#endif
