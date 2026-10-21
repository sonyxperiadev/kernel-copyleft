/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CVP_VM_H_
#define _CVP_VM_H_

#include <linux/types.h>
#include "cvp_comm_def.h"
#include "msm_cvp_core.h"
#include "msm_cvp_internal.h"
#include "cvp_core_hfi.h"
#include "cvp_vm_msgq.h"
#include "cvp_vm_resource.h"

enum cvp_vm_id {
	VM_PRIMARY = 1,
	VM_TRUSTED = 2,
	VM_INVALID = 3,
};

enum cvp_vm_state {
	VM_STATE_INIT = 1,
	VM_STATE_ACTIVE = 2,
	VM_STATE_ERROR = 3,
	VM_STATE_INVALID = 4,
};

struct msm_cvp_vm_ops {
	int (*vm_start)(struct msm_cvp_core *core);
	int (*vm_init_reg_and_irq)(struct iris_hfi_device *device,
			struct msm_cvp_platform_resources *res);
};

struct msm_cvp_vm_manager {
	enum cvp_vm_state vm_state;
	enum cvp_vm_id vm_id;
	struct cvp_msgq_drv *msgq_drv;
	struct cvp_vm_resource *vm_rm;
	struct msm_cvp_vm_ops *vm_ops;
};

extern struct msm_cvp_vm_manager vm_manager;
#endif
