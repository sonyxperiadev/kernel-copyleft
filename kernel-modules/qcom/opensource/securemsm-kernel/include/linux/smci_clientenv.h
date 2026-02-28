/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __SMCI_CLIENTENV_H
#define __SMCI_CLIENTENV_H

#include "smci_object.h"
#include "IClientEnv.h"

#define SMCI_CLIENTENV_OP_OPEN 0
#define SMCI_CLIENTENV_OP_REGISTERLEGACY 1
#define SMCI_CLIENTENV_OP_REGISTER 2
#define SMCI_CLIENTENV_OP_REGISTERWITHWHITELIST 3
#define SMCI_CLIENTENV_OP_NOTIFYDOMAINCHANGE 4
#define SMCI_CLIENTENV_OP_REGISTERWITHCREDENTIALS 5
#define SMCI_CLIENTENV_OP_LOADCMNLIBFROMBUFFER 6
#define SMCI_CLIENTENV_OP_CONFIGTAREGION 7
#define SMCI_CLIENTENV_OP_ADCIACCEPT 8
#define SMCI_CLIENTENV_OP_ADCISUTDOWN 9

static inline int32_t
smci_clientenv_release(struct smci_object self)
{
	return IClientEnv_release(self);
}

static inline int32_t
smci_clientenv_retain(struct smci_object self)
{
	return IClientEnv_retain(self);
}

static inline int32_t
smci_clientenv_open(struct smci_object self, uint32_t uid_val, struct smci_object *obj_ptr)
{
	return IClientEnv_open(self, uid_val, obj_ptr);
}

static inline int32_t
smci_clientenv_registerlegacy(struct smci_object self, const void *credentials_ptr,
		size_t credentials_len, struct smci_object *clientenv_ptr)
{
	return IClientEnv_registerLegacy(self, credentials_ptr,
		credentials_len, clientenv_ptr);
}

static inline int32_t
smci_clientenv_register(struct smci_object self, struct smci_object credentials_val,
			struct smci_object *clientenv_ptr)
{
	return IClientEnv_register(self, credentials_val,
			clientenv_ptr);
}

static inline int32_t
smci_clientenv_registerwithwhitelist(struct smci_object self,
		struct smci_object credentials_val, const uint32_t *uids_ptr,
		size_t uids_len, struct smci_object *clientenv_ptr)
{
	return IClientEnv_registerWithWhitelist(self,
		credentials_val, uids_ptr,
		uids_len, clientenv_ptr);
}

static inline int32_t
smc_clientenv_notifydomainchange(struct smci_object self)
{
	return IClientEnv_notifyDomainChange(self);
}

static inline int32_t
smci_clientenv_registerwithcredentials(struct smci_object self, struct smci_object
		credentials_val, struct smci_object *clientenv_ptr)
{
	return IClientEnv_registerWithCredentials(self,
		credentials_val, clientenv_ptr);
}

static inline int32_t
smci_clientenv_loadcmnlibfrombuffer(struct smci_object self, const void *cmnlibelf_ptr,
		size_t cmnlibelf_len)
{
	return IClientEnv_loadCmnlibFromBuffer(self, cmnlibelf_ptr, cmnlibelf_len);
}

static inline int32_t
smci_clientenv_configtaregion(struct smci_object self, uint64_t apprgnaddr_val,
		uint32_t apprgnsize_val)

{
	return IClientEnv_configTaRegion(self, apprgnaddr_val, apprgnsize_val);
}

static inline int32_t
smci_clientenv_adciaccept(struct smci_object self)
{
	return IClientEnv_adciAccept(self);
}

static inline int32_t
smci_clientenv_adcishutdown(struct smci_object self)
{
	return IClientEnv_adciShutdown(self);
}

#endif /* __SMCI_CLIENTENV_H */
