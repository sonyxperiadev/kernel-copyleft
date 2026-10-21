/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CVP_VM_RESOURCE_H_
#define _CVP_VM_RESOURCE_H_

#include <linux/types.h>
#include "cvp_comm_def.h"

struct cvp_vm_resource {
	int reserved;
};

extern struct cvp_vm_resource cvp_vm_rm;
#endif
