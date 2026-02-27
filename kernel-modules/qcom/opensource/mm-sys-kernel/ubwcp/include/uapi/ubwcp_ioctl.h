/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UBWCP_IOCTL_H_
#define __UBWCP_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define UBWCP_IOCTL_SET_BUF_ATTR _IOW('U', 1, struct ubwcp_ioctl_buffer_attrs)
#define UBWCP_IOCTL_GET_HW_VER   _IOR('U', 2, struct ubwcp_ioctl_hw_version)
#define UBWCP_IOCTL_GET_STRIDE_ALIGN  _IOWR('U', 3, struct ubwcp_ioctl_stride_align)
#define UBWCP_IOCTL_VALIDATE_STRIDE _IOWR('U', 4, struct ubwcp_ioctl_validate_stride)


enum ubwcp_image_format {
	UBWCP_LINEAR = 0,
	UBWCP_RGBA8888,
	UBWCP_NV12,
	UBWCP_NV12_Y,
	UBWCP_NV12_UV,
	UBWCP_NV124R,
	UBWCP_NV124R_Y,
	UBWCP_NV124R_UV,
	UBWCP_TP10,
	UBWCP_TP10_Y,
	UBWCP_TP10_UV,
	UBWCP_P010,
	UBWCP_P010_Y,
	UBWCP_P010_UV,
	UBWCP_P016,
	UBWCP_P016_Y,
	UBWCP_P016_UV,
};

enum ubwcp_compression_type {
	UBWCP_COMPRESSION_LOSSLESS = 0,
};

enum ubwcp_subsample {
	UBWCP_SUBSAMPLE_4_2_0 = 0,
};

#define UBWCP_SUBSYSTEM_TARGET_CPU  (1 << 0)

/**
 * @image_format: image format
 * @major_ubwc_ver: set to 0. This is not HW version.
 * @minor_ubwc_ver: set to 0. This is not HW version.
 * @compression_type: only lossless is supported.
 * @lossy_params: set to 0
 * @width: image width (pixels)
 * @height: image height (pixels)
 * @stride: image stride (bytes)
 * @scanlines: number of scanlines
 * @planar_padding: padding between Y and UV planes (bytes)
 * @subsample: only 4:2:0 is supported
 * @sub_system_target: only CPU is supported
 * @y_offset: set to 0
 * @batch_size: set to 1
 *
 * All pad[x] and unused[x] fields must be set to 0
 */
struct ubwcp_buffer_attrs {

	__u16 image_format;	/* enum ubwcp_image_format */
	__u16 major_ubwc_ver;	/* per-buffer version: must be set to 0 */
	__u16 minor_ubwc_ver;	/* per-buffer version: must be set to 0 */
	__u16 compression_type;	/* enum ubwcp_compression_type */

	__u64  lossy_params;	/* must be set to 0 */

	__u32 width;
	__u32 height;
	__u32 stride;
	__u32 scanlines;

	__u32 planar_padding;
	__u32 subsample;	/* enum enum ubwcp_subsample */
	__u32 sub_system_target;/* bit mask: UBWCP_SUBSYSTEM_TARGET_XXX */
	__u32 y_offset;		/* must be set to 0 */

	__u32 batch_size;	/* only size supported: 1 */
	__u32 unused1;

	__u32 unused2;
	__u32 unused3;
	__u32 unused4;
	__u32 unused5;

	__u32 unused6;
	__u32 unused7;
	__u32 unused8;
	__u32 unused9;
};

/**
 *  @fd: dma_buf file descriptor for the buffer whose
 *             attributes are specified
 *  @attr: ubwcp buffer attributes
 */
struct ubwcp_ioctl_buffer_attrs {
	__u32 fd;
	__u32 pad;
	struct ubwcp_buffer_attrs attr;
};

/**
 * ubwcp hardware version
 *  @major: major version
 *  @minor: minor version
 */
struct ubwcp_ioctl_hw_version {
	__u32 major;
	__u32 minor;
};

/**
 * Stride alignment for given format
 *  @image_format: image format
 *  @stride_align: stride alignment
 *  @unused: must be set to 0
 *  IOCTL will fail for linear image format
 */
struct ubwcp_ioctl_stride_align {
	__u16 image_format;
	__u16 stride_align;
	__u32 unused;
};

/**
 * validate stride
 *  @image_format: image format
 *  @width: image width in pixels
 *  @stride: image stride in bytes
 *  @valid: returns 0 (not valid), 1 (valid)
 *  @unusedX: must be set to 0
 *  IOCTL will fail for linear image format
 */
struct ubwcp_ioctl_validate_stride {
	__u16 image_format;
	__u32 width;
	__u32 stride;
	__u16 valid;
	__u16 unused1;
	__u16 unused2;
};

#endif /* __UBWCP_IOCTL_H_ */
