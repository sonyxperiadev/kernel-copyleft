/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SMCI_APPLOADER_H
#define __SMCI_APPLOADER_H

#include "smci_object.h"
#include "smci_appcontroller.h"
#include "IAppLoader.h"

#define SMCI_APPLOADER_ERROR_INVALID_BUFFER INT32_C(10)
#define SMCI_APPLOADER_ERROR_PIL_ROLLBACK_FAILURE INT32_C(11)
#define SMCI_APPLOADER_ERROR_ELF_SIGNATURE_ERROR INT32_C(12)
#define SMCI_APPLOADER_ERROR_METADATA_INVALID INT32_C(13)
#define SMCI_APPLOADER_ERROR_MAX_NUM_APPS INT32_C(14)
#define SMCI_APPLOADER_ERROR_NO_NAME_IN_METADATA INT32_C(15)
#define SMCI_APPLOADER_ERROR_ALREADY_LOADED INT32_C(16)
#define SMCI_APPLOADER_ERROR_EMBEDDED_IMAGE_NOT_FOUND INT32_C(17)
#define SMCI_APPLOADER_ERROR_TZ_HEAP_MALLOC_FAILURE INT32_C(18)
#define SMCI_APPLOADER_ERROR_TA_APP_REGION_MALLOC_FAILURE INT32_C(19)
#define SMCI_APPLOADER_ERROR_CLIENT_CRED_PARSING_FAILURE INT32_C(20)
#define SMCI_APPLOADER_ERROR_APP_UNTRUSTED_CLIENT INT32_C(21)
#define SMCI_APPLOADER_ERROR_APP_NOT_LOADED INT32_C(22)
#define SMCI_APPLOADER_ERROR_APP_MAX_CLIENT_CONNECTIONS INT32_C(23)
#define SMCI_APPLOADER_ERROR_APP_BLACKLISTED INT32_C(24)

#define SMCI_APPLOADER_OP_LOADFROMBUFFER 0
#define SMCI_APPLOADER_OP_LOADFROMREGION 1
#define SMCI_APPLOADER_OP_LOADEMBEDDED 2
#define SMCI_APPLOADER_OP_CONNECT 3
#define SMCI_APPLOADER_UID (0x3)

static inline int32_t
smci_apploader_release(struct smci_object self)
{
	return IAppLoader_release(self);
}

static inline int32_t
smci_apploader_retain(struct smci_object self)
{
	return IAppLoader_retain(self);
}

static inline int32_t
smci_apploader_loadfrombuffer(struct smci_object self, const void *appelf_ptr, size_t appelf_len,
		struct smci_object *appcontroller_ptr)
{
	return IAppLoader_loadFromBuffer(self, appelf_ptr, appelf_len,
		appcontroller_ptr);
}

static inline int32_t
smci_apploader_loadfromregion(struct smci_object self, struct smci_object appelf_val,
		struct smci_object *appcontroller_ptr)
{
	return IAppLoader_loadFromRegion(self, appelf_val,
		appcontroller_ptr);
}

static inline int32_t
smci_apploader_loadembedded(struct smci_object self, const void *appname_ptr, size_t appname_len,
		struct smci_object *appcontroller_ptr)
{
	return IAppLoader_loadEmbedded(self, appname_ptr, appname_len,
		appcontroller_ptr);
}

static inline int32_t
smci_apploader_connect(struct smci_object self, const void *appname_ptr, size_t appname_len,
		struct smci_object *appcontroller_ptr)
{
	return IAppLoader_connect(self, appname_ptr, appname_len,
		appcontroller_ptr);
}

#endif /* __SMCI_APPLOADER_H */
