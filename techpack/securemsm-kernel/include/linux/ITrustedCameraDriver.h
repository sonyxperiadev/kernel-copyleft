/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "smcinvoke_object.h"

/**
 * Struct containing values for programming of domain ID
 *
 * @version:            Version info
 * @protect:            To protect or reset the lanes
 * @csid_hw_idx_mask:   Bit position denoting CSID in use
 * @cdm_hw_idx_mask:    Bit position denoting CDM in use
 * @vc_mask:            VC mask for identifying domain
 * @phy_lane_sel_mask:  PHY lane info - contains CPHY, DPHY and PHY ID values
 *                      0-15 bits -- PHY index
 *                      16-23 bits -- CPHY lanes
 *                      24-31 bits -- DPHY lanes
 * @reserved:           Reserved bit
 */

typedef struct {
	uint32_t version;
	uint32_t protect;
	uint32_t csid_hw_idx_mask;
	uint32_t cdm_hw_idx_mask;
	uint64_t vc_mask;
	uint64_t phy_lane_sel_mask;
	uint64_t reserved;
} ITCDriverSensorInfo;

#define ITrustedCameraDriver_ERROR_NOT_ALLOWED 10

#define ITrustedCameraDriver_OP_dynamicProtectSensor 0
#define ITrustedCameraDriver_OP_getVersion 1
#define ITrustedCameraDriver_OP_dynamicConfigureFDPort 3

static inline int32_t
ITrustedCameraDriver_release(struct Object self)
{
	return Object_invoke(self, Object_OP_release, 0, 0);
}

static inline int32_t
ITrustedCameraDriver_retain(struct Object self)
{
	return Object_invoke(self, Object_OP_retain, 0, 0);
}

/*
 * Description: This method allows protecting a camera sensor based on the sensor
 *              information provided.
 *
 * In:          this - ITrustedCameraDriver object
 * In:          phy_info_ptr - Camera HW settings required for securing the usecase
 * Out:         void
 * Return:      Object_OK on success
 *              secure camera error codes from seccam_def on failure
 */

static inline int32_t
ITrustedCameraDriver_dynamicProtectSensor(struct Object self,
		const ITCDriverSensorInfo *phy_info_ptr)
{
	union ObjectArg a[1] = {{{0, 0}}};

	a[0].bi = (struct ObjectBufIn) { phy_info_ptr, sizeof(ITCDriverSensorInfo) };

	return Object_invoke(self, ITrustedCameraDriver_OP_dynamicProtectSensor, a,
			ObjectCounts_pack(1, 0, 0, 0));
}

/*
 * Description: Get the current version info
 *
 * In:         this - ITrustedCameraDriver object
 * Out:        arch_ver_ptr - the pointer of arch version number.
 * Out:        max_ver_ptr -  the pointer of the second part of the version number
 * Out:        min_ver_ptr -  the pointer of the third part of the version number
 * Return:     Object_OK on success
 */

static inline int32_t
ITrustedCameraDriver_getVersion(struct Object self, uint32_t *arch_ver_ptr,
		uint32_t *max_ver_ptr, uint32_t *min_ver_ptr)
{
	union ObjectArg a[1] = {{{0, 0}}};
	int32_t result;
	struct {
		uint32_t m_arch_ver;
		uint32_t m_max_ver;
		uint32_t m_min_ver;
	} o = {0};

	a[0].b = (struct ObjectBuf) { &o, 12 };

	result = Object_invoke(self, ITrustedCameraDriver_OP_getVersion, a,
			ObjectCounts_pack(0, 1, 0, 0));

	*arch_ver_ptr = o.m_arch_ver;
	*max_ver_ptr = o.m_max_ver;
	*min_ver_ptr = o.m_min_ver;

	return result;
}

/*
 * Description: Dynamic configuration to allow secure/non-secure FD port
 *              on all the CSIDs
 *
 * In:          this - ITrustedCameraDriver object
 * In:          protect - to secure or non-secure the port
 * Out:         void
 * Return:      Object_OK on success
 *              Object_ERROR on failure
 *              ITrustedCameraDriver_ERROR_NOT_ALLOWED on request to
 *              configure FD port even when disabled by OEM
 */

static inline int32_t
ITrustedCameraDriver_dynamicConfigureFDPort(struct Object self, uint32_t protect)
{
	union ObjectArg a[1] = {{{0, 0}}};

	a[0].b = (struct ObjectBuf) { &protect, sizeof(uint32_t) };

	return Object_invoke(self, ITrustedCameraDriver_OP_dynamicConfigureFDPort, a,
			ObjectCounts_pack(1, 0, 0, 0));
}
