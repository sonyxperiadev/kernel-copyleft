/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "smcinvoke_object.h"

#define IQSEEComCompatAppLoader_MAX_FILENAME_LEN UINT32_C(64)
#define IQSEEComCompatAppLoader_MAX_DISTNAME_LEN UINT32_C(128)
#define IQSEEComCompatAppLoader_ELFCLASS32 UINT32_C(1)
#define IQSEEComCompatAppLoader_ELFCLASS64 UINT32_C(2)

#define IQSEEComCompatAppLoader_ERROR_INVALID_BUFFER INT32_C(10)
#define IQSEEComCompatAppLoader_ERROR_PIL_ROLLBACK_FAILURE INT32_C(11)
#define IQSEEComCompatAppLoader_ERROR_ELF_SIGNATURE_ERROR INT32_C(12)
#define IQSEEComCompatAppLoader_ERROR_METADATA_INVALID INT32_C(13)
#define IQSEEComCompatAppLoader_ERROR_MAX_NUM_APPS INT32_C(14)
#define IQSEEComCompatAppLoader_ERROR_NO_NAME_IN_METADATA INT32_C(15)
#define IQSEEComCompatAppLoader_ERROR_ALREADY_LOADED INT32_C(16)
#define IQSEEComCompatAppLoader_ERROR_EMBEDDED_IMAGE_NOT_FOUND INT32_C(17)
#define IQSEEComCompatAppLoader_ERROR_TZ_HEAP_MALLOC_FAILURE INT32_C(18)
#define IQSEEComCompatAppLoader_ERROR_TA_APP_REGION_MALLOC_FAILURE INT32_C(19)
#define IQSEEComCompatAppLoader_ERROR_CLIENT_CRED_PARSING_FAILURE INT32_C(20)
#define IQSEEComCompatAppLoader_ERROR_APP_UNTRUSTED_CLIENT INT32_C(21)
#define IQSEEComCompatAppLoader_ERROR_APP_BLACKLISTED INT32_C(22)
#define IQSEEComCompatAppLoader_ERROR_APP_NOT_LOADED INT32_C(23)
#define IQSEEComCompatAppLoader_ERROR_NOT_QSEECOM_COMPAT_APP INT32_C(24)
#define IQSEEComCompatAppLoader_ERROR_FILENAME_TOO_LONG INT32_C(25)
#define IQSEEComCompatAppLoader_ERROR_APP_ARCH_NOT_SUPPORTED INT32_C(26)
#define IQSEEComCompatAppLoader_ERROR_VM_NOT_WHITELISTED_TO_LOAD_TA INT32_C(27)
#define IQSEEComCompatAppLoader_ERROR_ELF_LOADING INT32_C(28)

#define IQSEEComCompatAppLoader_OP_loadFromRegion 0
#define IQSEEComCompatAppLoader_OP_loadFromBuffer 1
#define IQSEEComCompatAppLoader_OP_lookupTA 2

static inline int32_t
IQSEEComCompatAppLoader_release(struct Object self)
{
	return Object_invoke(self, Object_OP_release, 0, 0);
}

static inline int32_t
IQSEEComCompatAppLoader_retain(struct Object self)
{
	return Object_invoke(self, Object_OP_retain, 0, 0);
}

/*
 *
 *  Load a trusted application from a memory region.
 *  The application ELF binary is passed as an IMemRegion object.
 *
 *  @param[in]  appElf         Region containing the ELF image
 *  @param[in]  filename       Filename associated with TA (buffer can be empty)
 *  @param[out] appCompat      A QSEEComCompat object to access the trusted application
 *
 *  @return  Object_OK if successful.
 *
 */
static inline int32_t IQSEEComCompatAppLoader_loadFromRegion(struct Object self,
			struct Object appElf,
			const void *filename_ptr, size_t filename_len,
			struct Object *appCompat)
{
	union ObjectArg a[] = {
		{.bi = (struct ObjectBufIn) { filename_ptr, filename_len * sizeof(uint8_t) } },
		{.o = appElf },
		{.o = Object_NULL },
	};

	int32_t result = Object_invoke(self, IQSEEComCompatAppLoader_OP_loadFromRegion, a,
			ObjectCounts_pack(1, 0, 1, 1));

	*appCompat = a[2].o;
	return result;
}

/*
 *
 *  Load a trusted application from buffer.
 *  The application ELF binary is passed as a buffer.
 *
 *  @param[in]  appElf         Buffer containing the ELF image
 *  @param[in]  filename       Filename associated with TA (buffer can be empty)
 *  @param[out] distName       App distinguished name
 *  @param[out] appCompat      A QSEEComCompat object to access the trusted application
 *
 *  @return  Object_OK if successful.
 *
 */
static inline int32_t IQSEEComCompatAppLoader_loadFromBuffer(struct Object self,
			const void *appElf_ptr, size_t appElf_len,
			const void *filename_ptr, size_t filename_len,
			void *distName_ptr, size_t distName_len,
			size_t *distName_lenout, struct Object *appCompat)
{
	union ObjectArg a[] = {
		{.bi = (struct ObjectBufIn) { appElf_ptr, appElf_len * sizeof(uint8_t) } },
		{.bi = (struct ObjectBufIn) { filename_ptr, filename_len * sizeof(uint8_t) } },
		{.b = (struct ObjectBuf) { distName_ptr, distName_len * sizeof(uint8_t) } },
		{.o = Object_NULL },
	};

	int32_t result = Object_invoke(self, IQSEEComCompatAppLoader_OP_loadFromBuffer, a,
			ObjectCounts_pack(2, 1, 0, 1));

	*distName_lenout = a[2].b.size / sizeof(uint8_t);
	*appCompat = a[3].o;
	return result;
}

/*
 *
 *  Lookup an already loaded TA and return its QSEEComCompat object.
 *  The lookup will be done first by filename, and if a TA was
 *  not found, the lookup will then be done by distinguished name and appname.
 *
 *  @param[in]  appName        Filename/Appname/Distinguished name associated with TA
 *  @param[out] appCompat      A QSEEComCompat object to access the trusted application
 *  @param[out] appArchType    Loaded trusted application arch type
 *
 *  @return  Object_OK if successful.
 *
 */
static inline int32_t IQSEEComCompatAppLoader_lookupTA(struct Object self,
			const void *appName_ptr, size_t appName_len,
			struct Object *appCompat, uint32_t *appArchType_ptr)
{
	union ObjectArg a[] = {
		{.bi = (struct ObjectBufIn) { appName_ptr, appName_len * sizeof(uint8_t) } },
		{.b = (struct ObjectBuf) { appArchType_ptr, sizeof(uint32_t) } },
		{.o = Object_NULL },
	};

	int32_t result = Object_invoke(self, IQSEEComCompatAppLoader_OP_lookupTA, a,
			ObjectCounts_pack(1, 1, 0, 1));

	*appCompat = a[2].o;
	return result;
}

