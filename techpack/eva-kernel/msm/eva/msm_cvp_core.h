/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 */

#ifndef _MSM_CVP_CORE_H_
#define _MSM_CVP_CORE_H_

#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/dma-buf.h>
#include <linux/refcount.h>
#include <media/msm_eva_private.h>
#include "msm_cvp_buf.h"
#include "msm_cvp_synx.h"

#define DDR_TYPE_LPDDR4 0x6
#define DDR_TYPE_LPDDR4X 0x7
#define DDR_TYPE_LPDDR4Y 0x8
#define DDR_TYPE_LPDDR5 0x9

enum session_type {
	MSM_CVP_USER = 1,
	MSM_CVP_KERNEL,
	MSM_CVP_BOOT,
	MSM_CVP_DSP,
	MSM_CVP_UNKNOWN,
	MSM_CVP_MAX_DEVICES = MSM_CVP_UNKNOWN,
};

struct msm_cvp_inst *msm_cvp_open(int session_type, struct task_struct *task);
int msm_cvp_close(void *instance);
int msm_cvp_suspend(void);
int msm_cvp_poll(void *instance, struct file *filp,
		struct poll_table_struct *pt);
int msm_cvp_private(void *cvp_inst, unsigned int cmd,
		struct eva_kmd_arg *arg);

#endif
