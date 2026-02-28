/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SMCI_IMPL_H__
#define __SMCI_IMPL_H__

#include <linux/sched.h>
#include <linux/qtee_shmbridge.h>
#include <linux/firmware/qcom/si_object.h>

#include "smcinvoke.h"

struct server_info {
	char comm[TASK_COMM_LEN];
	struct kref refcount;

	int dead;

	struct list_head cb_tx_list;

	/* Queue of server threads waiting for a new transaction. */
	wait_queue_head_t Q;
};

#define CONTEXT_ID_ANY 0

struct cb_txn {
	struct kref refcount;

	unsigned int context_id;

	/* ''Object Invocation'' */

	s64 u_handle;			/* Object's u_handle to invoke. */
	unsigned long op;		/* Operation to perform on object. */
	struct si_arg *args;	/* Arguments for the requested operation. */
	int errno;				/* Result of the operation. */

	enum state {
		XST_NEW = 0,		/* New transaction. */
		XST_PENDING = 1,	/* Waiting for server. */
		XST_PROCESSING = 2,	/* Being processed by server. */
		XST_PROCESSED = 3,	/* Done. */
		XST_TIMEDOUT = 4,
	} processing;

	/* ''Object Invocation'' as seen by userspace. */

	size_t u_args_sz;
	union smcinvoke_arg *u_args;

	struct list_head node;
	struct completion completion;
};

#define U_ARGS_SIZE(t) (size_of_arg((t)->args) * sizeof((t)->u_args[0]))

#endif /* __SMCI_IMPL_H__ */
