/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SMCI_OPENER_H
#define __SMCI_OPENER_H

#include "smci_object.h"
#include "IOpener.h"


/** 0 is not a valid service ID. */
#define SMCI_OPENER_INVALID_ID UINT32_C(0)

#define SMCI_OPENER_ERROR_NOT_FOUND INT32_C(10)
#define SMCI_OPENER_ERROR_PRIVILEGE INT32_C(11)
#define SMCI_OPENER_ERROR_NOT_SUPPORTED INT32_C(12)

#define SMCI_OPENER_OP_OPEN 0

static inline int32_t
smci_opener_release(struct smci_object self)
{
	return IOpener_release(self);
}

static inline int32_t
smci_opener_retain(struct smci_object self)
{
	return IOpener_retain(self);
}

static inline int32_t
smci_opener_open(struct smci_object self, uint32_t id_val, struct smci_object *obj_ptr)
{
	return IOpener_open(self, id_val, obj_ptr);
}

#endif /* __SMCI_OPENER_H */
