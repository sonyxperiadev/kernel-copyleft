/**
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/**
 * This file contains definitions that are common to UMD and KMD
 * but shall not be added to the UAPI to allow for better UAPI
 * backward compatibility. Identical copies of this file must be
 * used by both UMD and KMD for desired functioning.
 */

#ifndef _EVA_SHARED_DEF_H_
#define _EVA_SHARED_DEF_H_

/**
 * Structure corresponding to HFI_CVP_BUFFER_TYPE
 */

struct cvp_buf_type {
	__s32 fd;
	__u32 size;
	__u32 offset;
	__u32 flags;
	__u32 reserved1;
	__u32 reserved2;
	__u32 fence_type;
	__u32 input_handle;
	__u32 output_handle;
	__u32 debug_flags;
	__u32 crc;
};

/**
 * Structures and macros for Warp-NCC Out-of-Band (OOB) buffer
 */

#define EVA_KMD_WNCC_MAX_LAYERS               4
#define EVA_KMD_WNCC_MAX_ADDRESSES            4095
#define EVA_KMD_WNCC_MAX_SRC_BUFS             2400
#define EVA_KMD_WNCC_SRC_BUF_ID_OFFSET        1

struct eva_kmd_wncc_metadata {
	__u64 loc_x_dec : 12;
	__u64 loc_x_frac : 9;
	__u64 loc_y_dec : 12;
	__u64 loc_y_frac : 9;
	__u64 iova_lsb : 22; /* Populated in KMD */
	__u64 iova_msb : 10; /* Populated in KMD */
	__u64 scale_idx : 2;
	__s64 aff_coeff_3 : 13;
	__s64 aff_coeff_2 : 13;
	__s64 aff_coeff_1 : 13;
	__s64 aff_coeff_0 : 13;
};

struct eva_kmd_oob_wncc {
	__u32 metadata_bufs_offset;
	__u32 num_layers;
	struct eva_kmd_wncc_layer {
		__u32 num_addrs;
		struct eva_kmd_wncc_addr {
			__u32 buffer_id;
			__u32 offset;
		} addrs[EVA_KMD_WNCC_MAX_ADDRESSES];
	} layers[EVA_KMD_WNCC_MAX_LAYERS];
};

/**
 * Structure and macros for Out-of-Band (OOB) buffer
 * that may accompany HFI packet data
 */

#define EVA_KMD_OOB_INVALID 0
#define EVA_KMD_OOB_WNCC    1

struct eva_kmd_oob_buf {
	__u32 oob_type;
	union {
		struct eva_kmd_oob_wncc wncc;
	};
};

#endif
