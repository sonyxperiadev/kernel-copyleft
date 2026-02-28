/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SMCI_APPCONTROLLER_H
#define __SMCI_APPCONTROLLER_H

#include "smci_object.h"
#include "IAppController.h"

#define SMCI_APPCONTROLLER_CBO_INTERFACE_WAIT UINT32_C(1)

#define SMCI_APPCONTROLLER_ERROR_APP_SUSPENDED INT32_C(10)
#define SMCI_APPCONTROLLER_ERROR_APP_BLOCKED_ON_LISTENER INT32_C(11)
#define SMCI_APPCONTROLLER_ERROR_APP_UNLOADED INT32_C(12)
#define SMCI_APPCONTROLLER_ERROR_APP_IN_USE INT32_C(13)
#define SMCI_APPCONTROLLER_ERROR_NOT_SUPPORTED INT32_C(14)
#define SMCI_APPCONTROLLER_ERROR_CBO_UNKNOWN INT32_C(15)
#define SMCI_APPCONTROLLER_ERROR_APP_UNLOAD_NOT_ALLOWED INT32_C(16)
#define SMCI_APPCONTROLLER_ERROR_APP_DISCONNECTED INT32_C(17)
#define SMCI_APPCONTROLLER_ERROR_USER_DISCONNECT_REJECTED INT32_C(18)
#define SMCI_APPCONTROLLER_ERROR_STILL_RUNNING INT32_C(19)

#define SMCI_APPCONTROLLER_OP_OPENSESSION 0
#define SMCI_APPCONTROLLER_OP_UNLOAD 1
#define SMCI_APPCONTROLLER_OP_GETAPPOBJECT 2
#define SMCI_APPCONTROLLER_OP_INSTALLCBO 3
#define SMCI_APPCONTROLLER_OP_DISCONNECT 4
#define SMCI_APPCONTROLLER_OP_RESTART 5

static inline int32_t
smci_appcontroller_release(struct smci_object self)
{
	return IAppController_release(self);
}

static inline int32_t
smci_appcontroller_retain(struct smci_object self)
{
	return IAppController_retain(self);
}

static inline int32_t
smci_appcontroller_opensession(struct smci_object self, uint32_t cancel_code_val,
	uint32_t connection_method_val, uint32_t connection_data_val, uint32_t param_types_val,
	uint32_t ex_param_types_val, const void *i1_ptr, size_t i1_len, const void *i2_ptr,
	size_t i2_len, const void *i3_ptr, size_t i3_len, const void *i4_ptr, size_t i4_len,
	void *o1_ptr, size_t o1_len, size_t *o1_lenout, void *o2_ptr, size_t o2_len,
	size_t *o2_lenout, void *o3_ptr, size_t o3_len, size_t *o3_lenout, void *o4_ptr,
	size_t o4_len, size_t *o4_lenout, struct smci_object imem1_val,
	struct smci_object imem2_val, struct smci_object imem3_val, struct smci_object imem4_val,
	uint32_t *memref_out_sz1_ptr, uint32_t *memref_out_sz2_ptr, uint32_t *memref_out_sz3_ptr,
	uint32_t *memref_out_sz4_ptr, struct smci_object *session_ptr, uint32_t *ret_value_ptr,
	uint32_t *ret_origin_ptr)
{
	return IAppController_openSession(self, cancel_code_val,
		connection_method_val, connection_data_val, param_types_val,
		ex_param_types_val, i1_ptr, i1_len, i2_ptr,
		i2_len, i3_ptr, i3_len, i4_ptr, i4_len,
		o1_ptr, o1_len, o1_lenout, o2_ptr, o2_len,
		o2_lenout, o3_ptr, o3_len, o3_lenout, o4_ptr,
		o4_len, o4_lenout, imem1_val,
		imem2_val, imem3_val, imem4_val,
		memref_out_sz1_ptr, memref_out_sz2_ptr, memref_out_sz3_ptr,
		memref_out_sz4_ptr, session_ptr, ret_value_ptr,
		ret_origin_ptr);
}

static inline int32_t
smci_appcontroller_unload(struct smci_object self)
{
	return IAppController_unload(self);
}

static inline int32_t
smci_appcontroller_getappobject(struct smci_object self, struct smci_object *obj_ptr)
{
	return IAppController_getAppObject(self, obj_ptr);
}

static inline int32_t
smci_appcontroller_installcbo(struct smci_object self, uint32_t uid_val, struct smci_object obj_val)
{
	return IAppController_installCBO(self, uid_val, obj_val);
}

static inline int32_t
smci_appcontroller_disconnect(struct smci_object self)
{
	return IAppController_disconnect(self);
}

static inline int32_t
smci_appcontroller_restart(struct smci_object self)
{
	return IAppController_restart(self);
}

#endif /* __SMCI_APPCONTROLLER_H */
