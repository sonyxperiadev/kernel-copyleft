/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "smcinvoke_object.h"

//Maximum number of ports which can exist in a HWTYPE
#define PORT_MAX 65

#define PROTECT_PORT 1
#define UNPROTECT_PORT 0

//Camera HW types
#define ITRUSTEDCAMERADRIVER_IFE 0
#define ITRUSTEDCAMERADRIVER_IFE_LITE 1
#define ITRUSTEDCAMERADRIVER_IPE 2
#define ITRUSTEDCAMERADRIVER_TFE 3
#define ITRUSTEDCAMERADRIVER_HWType_MAX 4

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

struct tc_driver_sensor_info {
	uint32_t version;
	uint32_t protect;
	uint32_t csid_hw_idx_mask;
	uint32_t cdm_hw_idx_mask;
	uint64_t vc_mask;
	uint64_t phy_lane_sel_mask;
	uint64_t reserved;
};

/**
 * Struct containing values for configuration of ports
 *
 * @hw_type:            Type of device to be configured(IFE, IFE LITE, IPE,..)
 * @hw_id_mask:         Mask for IDs of the devices
 * @protect:            Protect or unprotect the ports
 * @phy_id:             Phy ID for real time engines only
 * @num_ports:          Number of ports that are going to get configured
 * @port_id:            To specify what all ports need to be configured
 *                      for the device(s) specified in the mask of type enum.
 *                      It's an array of size PORT_MAX, which indicates
 *                      the maximum number of ports available for any device.
 *                      The number of ports to be configured from this array is
 *                      defined by 'num_ports'
 */

struct port_info {
	uint32_t hw_type;
	uint32_t hw_id_mask;
	uint32_t protect;
	uint32_t phy_id;
	uint32_t num_ports;
	uint32_t port_id[PORT_MAX];
};


//This enumerates TFE device output port resources.
enum tfe_output_port_id {
	TFE_FULL = 0,
	TFE_DC4_Y,
	TFE_DC4_C,
	TFE_DC16_Y,
	TFE_DC16_C,
	TFE_DS2_Y,
	TFE_DS2_C,
	TFE_FD_Y,
	TFE_FD_C,
	TFE_IR_OUT,
	TFE_PDAF_0,
	TFE_PDAF_1,
	TFE_PDAF_2,
	TFE_PDAF_3,
	TFE_RDI_0,
	TFE_RDI_1,
	TFE_RDI_2,
	TFE_RDI_3,
	TFE_RDI_4,
	TFE_SECURE_FRAME_HEADER,
};

//This enumerates IFE_LITE device output port resources.
enum ife_lite_output_port_id {
	IFE_LITE_RDI_0_OUT = 0,
	IFE_LITE_RDI_1_OUT,
	IFE_LITE_RDI_2_OUT,
	IFE_LITE_RDI_3_OUT,
	IFE_LITE_GAMMA,
	IFE_LITE_SECURE_FRAME_HEADER,
	IFE_LITE_GAMMA_MNDS_Y_CP_EN,
	IFE_LITE_GAMMA_2,
	IFE_LITE_GAMMA_1,
};

//This enumerates IFE device output port resources.
enum ife_output_port_id {
	IFE_VIDEO_OUT = 0,
	IFE_FD_OUT,
	IFE_PIXEL_RAW_DUMP_OUT,
	IFE_PDAF_OUT,
	IFE_RDI_0_OUT,
	IFE_DISPLAY_FULL_OUT,
	IFE_CAMIF_PD,
	IFE_LCR,
	IFE_RDI_1_OUT,
	IFE_RDI_2_OUT,
	IFE_BAYER_LTM_OUT,
	IFE_SECURE_FRAME_HEADER,
};

//This enumerates IPE device output port resources.
enum ipe_output_port_id {
	IPE_DISP_C = 0,
	IPE_DISP_Y,
	IPE_VID_C,
	IPE_VID_Y,
	IPE_TF_DS4_YC,
	IPE_TF_C,
	IPE_TF_Y,
	IPE_TFI,
	IPE_PDI,
	IPE_BLUR_MAP,
	IPE_APP_Y,
	IPE_APP_C,
	IPE_HDR_LM1,
	IPE_HDR_LM2,
	IPE_LTM_MASK_OUT,
	IPE_LTM_MAX_MIN,
	IPE_GS,
	IPE_HDR_HLW1,
	IPE_HDR_HLW2,
	IPE_SECURE_FRAME_HEADER,
	IPE_DS_C,
	IPE_DS_Y,
};

#define ITRUSTEDCAMERADRIVER_ERROR_NOT_ALLOWED 10

#define ITRUSTEDCAMERADRIVER_OP_DYNAMICPROTECTSENSOR 0
#define ITRUSTEDCAMERADRIVER_OP_GETVERSION 1
#define ITRUSTEDCAMERADRIVER_OP_DYNAMICCONFIGUREFDPORT 3
#define ITRUSTEDCAMERADRIVER_OP_DYNAMICCONFIGUREPORTSV2 4

static inline int32_t
trusted_camera_driver_release(struct Object self)
{
	return Object_invoke(self, Object_OP_release, 0, 0);
}

static inline int32_t
trusted_camera_driver_retain(struct Object self)
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
trusted_camera_driver_dynamic_protect_sensor(struct Object self,
		const struct tc_driver_sensor_info *phy_info_ptr)
{
	union ObjectArg a[1] = {{{0, 0}}};

	a[0].bi = (struct ObjectBufIn) { phy_info_ptr, sizeof(struct tc_driver_sensor_info) };

	return Object_invoke(self, ITRUSTEDCAMERADRIVER_OP_DYNAMICPROTECTSENSOR, a,
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
trusted_camera_driver_get_version(struct Object self, uint32_t *arch_ver_ptr,
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

	result = Object_invoke(self, ITRUSTEDCAMERADRIVER_OP_GETVERSION, a,
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
trusted_camera_driver_dynamic_configure_fd_port(struct Object self, uint32_t protect)
{
	union ObjectArg a[1] = {{{0, 0}}};

	a[0].b = (struct ObjectBuf) { &protect, sizeof(uint32_t) };

	return Object_invoke(self, ITRUSTEDCAMERADRIVER_OP_DYNAMICCONFIGUREFDPORT, a,
			ObjectCounts_pack(1, 0, 0, 0));
}

/*
 * Description: Dynamic configuration for secure and non-secure port on a given HWTYPE
 *
 * In:          this - ITrustedCameraDriver object
 * In:          port_info_ptr - structure containing the port_info
 * In:          port_info_len - length of port_info structure
 * Out:         void
 * Return:      Object_OK on success
 *              Object_ERROR on failure
 *              ITrustedCameraDriver_ERROR_NOT_ALLOWED on request to
 *              configure ports even when disabled by OEM
 */

static inline int32_t
trusted_camera_driver_dynamic_configure_ports_v2(struct Object self,
		const struct port_info *port_info_ptr, size_t port_info_len)
{
	union ObjectArg a[1] = {{{0, 0}}};

	a[0].bi = (struct ObjectBufIn) { port_info_ptr, port_info_len * sizeof(struct port_info) };

	return Object_invoke(self, ITRUSTEDCAMERADRIVER_OP_DYNAMICCONFIGUREPORTSV2, a,
			ObjectCounts_pack(1, 0, 0, 0));
}
