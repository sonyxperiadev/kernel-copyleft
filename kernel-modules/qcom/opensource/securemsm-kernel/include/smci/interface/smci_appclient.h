/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SMCI_APPCLIENT_H
#define __SMCI_APPCLIENT_H

#include "smci_object.h"
#include "IAppClient.h"

#define SMCI_APPCLIENT_ERROR_APP_NOT_FOUND INT32_C(10)
#define SMCI_APPCLIENT_ERROR_APP_RESTART_FAILED INT32_C(11)
#define SMCI_APPCLIENT_ERROR_APP_UNTRUSTED_CLIENT INT32_C(12)
#define SMCI_APPCLIENT_ERROR_CLIENT_CRED_PARSING_FAILURE INT32_C(13)
#define SMCI_APPCLIENT_ERROR_APP_LOAD_FAILED INT32_C(14)

#define SMCI_APPCLIENT_UID (0x97)
#define SMCI_APPCLIENT_OP_GETAPPOBJECT 0

static inline int32_t
smci_appclient_release(struct smci_object self)
{
	return IAppClient_release(self);
}

static inline int32_t
smci_appclient_retain(struct smci_object self)
{
	return IAppClient_retain(self);
}

static inline int32_t
smci_appclient_getappobject(struct smci_object self, const void *app_dist_name_ptr,
			size_t app_dist_name_len, struct smci_object *obj_ptr)
{
	return IAppClient_getAppObject(self, app_dist_name_ptr,
		app_dist_name_len, obj_ptr);
}

#endif /* __SMCI_APPCLIENT_H */
