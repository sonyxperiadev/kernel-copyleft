/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __MSM_MEDIA_INFO_H__
#define __MSM_MEDIA_INFO_H__

#include "msm_vidc_internal.h"

/* Width and Height should be multiple of 16 */
#define INTERLACE_WIDTH_MAX 1920
#define INTERLACE_HEIGHT_MAX 1920
#define INTERLACE_MB_PER_FRAME_MAX ((1920 * 1088) / 256)

#ifndef MSM_MEDIA_ALIGN
#define MSM_MEDIA_ALIGN(__sz, __align) (((__align) & ((__align) - 1)) ?\
	((((__sz) + (__align) - 1) / (__align)) * (__align)) :\
	(((__sz) + (__align) - 1) & (~((__align) - 1))))
#endif

#ifndef MSM_MEDIA_ROUNDUP
#define MSM_MEDIA_ROUNDUP(__sz, __r) (((__sz) + ((__r) - 1)) / (__r))
#endif

/*
 * Function arguments:
 * @v4l2_fmt
 * @width
 * Progressive: width
 * Interlaced: width
 */
static inline unsigned int video_y_stride_bytes(unsigned int colorformat,
						unsigned int width)
{
	unsigned int alignment, stride = 0;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12:
	case MSM_VIDC_FMT_NV21:
	case MSM_VIDC_FMT_NV12C:
		alignment = 128;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	case MSM_VIDC_FMT_TP10C:
		alignment = 256;
		stride = MSM_MEDIA_ALIGN(width, 192);
		stride = MSM_MEDIA_ALIGN(stride * 4 / 3, alignment);
		break;
	case MSM_VIDC_FMT_P010:
		alignment = 256;
		stride = MSM_MEDIA_ALIGN(width * 2, alignment);
		break;
	default:
		break;
	}
invalid_input:
	return stride;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @width
 * Progressive: width
 * Interlaced: width
 */
static inline unsigned int video_y_stride_pix(unsigned int colorformat,
					      unsigned int width)
{
	unsigned int alignment, stride = 0;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12:
	case MSM_VIDC_FMT_NV21:
	case MSM_VIDC_FMT_NV12C:
	case MSM_VIDC_FMT_P010:
		alignment = 128;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	case MSM_VIDC_FMT_TP10C:
		alignment = 192;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	default:
		break;
	}

invalid_input:
	return stride;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @width
 * Progressive: width
 * Interlaced: width
 */
static inline unsigned int video_uv_stride_bytes(unsigned int colorformat,
						 unsigned int width)
{
	unsigned int alignment, stride = 0;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV21:
	case MSM_VIDC_FMT_NV12:
	case MSM_VIDC_FMT_NV12C:
		alignment = 128;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	case MSM_VIDC_FMT_TP10C:
		alignment = 256;
		stride = MSM_MEDIA_ALIGN(width, 192);
		stride = MSM_MEDIA_ALIGN(stride * 4 / 3, alignment);
		break;
	case MSM_VIDC_FMT_P010:
		alignment = 256;
		stride = MSM_MEDIA_ALIGN(width * 2, alignment);
		break;
	default:
		break;
	}
invalid_input:
	return stride;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @width
 * Progressive: width
 * Interlaced: width
 */
static inline unsigned int video_uv_stride_pix(unsigned int colorformat,
					       unsigned int width)
{
	unsigned int alignment, stride = 0;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV21:
	case MSM_VIDC_FMT_NV12:
	case MSM_VIDC_FMT_NV12C:
	case MSM_VIDC_FMT_P010:
		alignment = 128;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	case MSM_VIDC_FMT_TP10C:
		alignment = 192;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	default:
		break;
	}
invalid_input:
	return stride;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @height
 * Progressive: height
 * Interlaced: (height+1)>>1
 */
static inline unsigned int video_y_scanlines(unsigned int colorformat,
					     unsigned int height)
{
	unsigned int alignment, sclines = 0;

	if (!height)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12:
	case MSM_VIDC_FMT_NV21:
	case MSM_VIDC_FMT_NV12C:
	case MSM_VIDC_FMT_P010:
		alignment = 32;
		break;
	case MSM_VIDC_FMT_TP10C:
		alignment = 16;
		break;
	default:
		return 0;
	}
	sclines = MSM_MEDIA_ALIGN(height, alignment);
invalid_input:
	return sclines;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @height
 * Progressive: height
 * Interlaced: (height+1)>>1
 */
static inline unsigned int video_uv_scanlines(unsigned int colorformat,
					      unsigned int height)
{
	unsigned int alignment, sclines = 0;

	if (!height)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV21:
	case MSM_VIDC_FMT_NV12:
	case MSM_VIDC_FMT_TP10C:
	case MSM_VIDC_FMT_P010:
		alignment = 16;
		break;
	case MSM_VIDC_FMT_NV12C:
		alignment = 32;
		break;
	default:
		goto invalid_input;
	}

	sclines = MSM_MEDIA_ALIGN((height + 1) >> 1, alignment);

invalid_input:
	return sclines;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @width
 * Progressive: width
 * Interlaced: width
 */
static inline unsigned int video_y_meta_stride(unsigned int colorformat,
					       unsigned int width)
{
	int y_tile_width = 0, y_meta_stride = 0;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12C:
		y_tile_width = 32;
		break;
	case MSM_VIDC_FMT_TP10C:
		y_tile_width = 48;
		break;
	default:
		goto invalid_input;
	}

	y_meta_stride = MSM_MEDIA_ROUNDUP(width, y_tile_width);
	y_meta_stride = MSM_MEDIA_ALIGN(y_meta_stride, 64);

invalid_input:
	return y_meta_stride;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @height
 * Progressive: height
 * Interlaced: (height+1)>>1
 */
static inline unsigned int video_y_meta_scanlines(unsigned int colorformat,
						  unsigned int height)
{
	int y_tile_height = 0, y_meta_scanlines = 0;

	if (!height)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12C:
		y_tile_height = 8;
		break;
	case MSM_VIDC_FMT_TP10C:
		y_tile_height = 4;
		break;
	default:
		goto invalid_input;
	}

	y_meta_scanlines = MSM_MEDIA_ROUNDUP(height, y_tile_height);
	y_meta_scanlines = MSM_MEDIA_ALIGN(y_meta_scanlines, 16);

invalid_input:
	return y_meta_scanlines;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @width
 * Progressive: width
 * Interlaced: width
 */
static inline unsigned int video_uv_meta_stride(unsigned int colorformat,
						unsigned int width)
{
	int uv_tile_width = 0, uv_meta_stride = 0;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12C:
		uv_tile_width = 16;
		break;
	case MSM_VIDC_FMT_TP10C:
		uv_tile_width = 24;
		break;
	default:
		goto invalid_input;
	}

	uv_meta_stride = MSM_MEDIA_ROUNDUP((width + 1) >> 1, uv_tile_width);
	uv_meta_stride = MSM_MEDIA_ALIGN(uv_meta_stride, 64);

invalid_input:
	return uv_meta_stride;
}

/*
 * Function arguments:
 * @v4l2_fmt
 * @height
 * Progressive: height
 * Interlaced: (height+1)>>1
 */
static inline unsigned int video_uv_meta_scanlines(unsigned int colorformat,
						   unsigned int height)
{
	int uv_tile_height = 0, uv_meta_scanlines = 0;

	if (!height)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12C:
		uv_tile_height = 8;
		break;
	case MSM_VIDC_FMT_TP10C:
		uv_tile_height = 4;
		break;
	default:
		goto invalid_input;
	}

	uv_meta_scanlines = MSM_MEDIA_ROUNDUP((height + 1) >> 1, uv_tile_height);
	uv_meta_scanlines = MSM_MEDIA_ALIGN(uv_meta_scanlines, 16);

invalid_input:
	return uv_meta_scanlines;
}

static inline unsigned int video_rgb_stride_bytes(unsigned int colorformat,
						  unsigned int width)
{
	unsigned int alignment = 0, stride = 0, bpp = 4;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_RGBA8888C:
	case MSM_VIDC_FMT_RGBA8888:
		alignment = 256;
		break;
	default:
		goto invalid_input;
	}

	stride = MSM_MEDIA_ALIGN(width * bpp, alignment);

invalid_input:
	return stride;
}

static inline unsigned int video_rgb_stride_pix(unsigned int colorformat,
						unsigned int width)
{
	unsigned int bpp = 4;

	return video_rgb_stride_bytes(colorformat, width) / bpp;
}

static inline unsigned int video_rgb_scanlines(unsigned int colorformat,
					       unsigned int height)
{
	unsigned int alignment = 0, scanlines = 0;

	if (!height)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_RGBA8888C:
		alignment = 16;
		break;
	case MSM_VIDC_FMT_RGBA8888:
		alignment = 32;
		break;
	default:
		goto invalid_input;
	}

	scanlines = MSM_MEDIA_ALIGN(height, alignment);

invalid_input:
	return scanlines;
}

static inline unsigned int video_rgb_meta_stride(unsigned int colorformat,
						 unsigned int width)
{
	int rgb_tile_width = 0, rgb_meta_stride = 0;

	if (!width)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_RGBA8888C:
	case MSM_VIDC_FMT_RGBA8888:
		rgb_tile_width = 16;
		break;
	default:
		goto invalid_input;
	}

	rgb_meta_stride = MSM_MEDIA_ROUNDUP(width, rgb_tile_width);
	rgb_meta_stride = MSM_MEDIA_ALIGN(rgb_meta_stride, 64);

invalid_input:
	return rgb_meta_stride;
}

static inline unsigned int video_rgb_meta_scanlines(unsigned int colorformat,
						    unsigned int height)
{
	int rgb_tile_height = 0, rgb_meta_scanlines = 0;

	if (!height)
		goto invalid_input;

	switch (colorformat) {
	case MSM_VIDC_FMT_RGBA8888C:
	case MSM_VIDC_FMT_RGBA8888:
		rgb_tile_height = 4;
		break;
	default:
		goto invalid_input;
	}

	rgb_meta_scanlines = MSM_MEDIA_ROUNDUP(height, rgb_tile_height);
	rgb_meta_scanlines = MSM_MEDIA_ALIGN(rgb_meta_scanlines, 16);

invalid_input:
	return rgb_meta_scanlines;
}

static inline unsigned int video_buffer_size(unsigned int colorformat,
					     unsigned int pix_width,
					     unsigned int pix_height,
					     unsigned int interlace)
{
	unsigned int size = 0;
	unsigned int y_plane, uv_plane, y_stride,
		uv_stride, y_sclines, uv_sclines;
	unsigned int y_ubwc_plane = 0, uv_ubwc_plane = 0;
	unsigned int y_meta_stride = 0, y_meta_scanlines = 0;
	unsigned int uv_meta_stride = 0, uv_meta_scanlines = 0;
	unsigned int y_meta_plane = 0, uv_meta_plane = 0;
	unsigned int rgb_stride = 0, rgb_scanlines = 0;
	unsigned int rgb_plane = 0, rgb_ubwc_plane = 0, rgb_meta_plane = 0;
	unsigned int rgb_meta_stride = 0, rgb_meta_scanlines = 0;

	if (!pix_width || !pix_height)
		goto invalid_input;

	y_stride = video_y_stride_bytes(colorformat, pix_width);
	uv_stride = video_uv_stride_bytes(colorformat, pix_width);
	y_sclines = video_y_scanlines(colorformat, pix_height);
	uv_sclines = video_uv_scanlines(colorformat, pix_height);
	rgb_stride = video_rgb_stride_bytes(colorformat, pix_width);
	rgb_scanlines = video_rgb_scanlines(colorformat, pix_height);

	switch (colorformat) {
	case MSM_VIDC_FMT_NV21:
	case MSM_VIDC_FMT_NV12:
	case MSM_VIDC_FMT_P010:
		y_plane = y_stride * y_sclines;
		uv_plane = uv_stride * uv_sclines;
		size = y_plane + uv_plane;
		break;
	case MSM_VIDC_FMT_NV12C:
		y_meta_stride = video_y_meta_stride(colorformat, pix_width);
		uv_meta_stride = video_uv_meta_stride(colorformat, pix_width);
		if (!interlace && colorformat == MSM_VIDC_FMT_NV12C) {
			y_ubwc_plane = MSM_MEDIA_ALIGN(y_stride * y_sclines, 4096);
			uv_ubwc_plane = MSM_MEDIA_ALIGN(uv_stride * uv_sclines, 4096);
			y_meta_scanlines =
				video_y_meta_scanlines(colorformat, pix_height);
			y_meta_plane = MSM_MEDIA_ALIGN(y_meta_stride *
						       y_meta_scanlines, 4096);
			uv_meta_scanlines =
				video_uv_meta_scanlines(colorformat, pix_height);
			uv_meta_plane = MSM_MEDIA_ALIGN(uv_meta_stride *
							uv_meta_scanlines,
							4096);
			size = (y_ubwc_plane + uv_ubwc_plane + y_meta_plane +
				uv_meta_plane);
		} else {
			if (pix_width <= INTERLACE_WIDTH_MAX &&
				pix_height <= INTERLACE_HEIGHT_MAX &&
				(pix_height * pix_width) / 256 <= INTERLACE_MB_PER_FRAME_MAX) {
				y_sclines =
					video_y_scanlines(colorformat, (pix_height + 1) >> 1);
				y_ubwc_plane =
					MSM_MEDIA_ALIGN(y_stride * y_sclines, 4096);
				uv_sclines =
					video_uv_scanlines(colorformat, (pix_height + 1) >> 1);
				uv_ubwc_plane =
					MSM_MEDIA_ALIGN(uv_stride * uv_sclines, 4096);
				y_meta_scanlines =
				video_y_meta_scanlines(colorformat, (pix_height + 1) >> 1);
				y_meta_plane = MSM_MEDIA_ALIGN(y_meta_stride *
							       y_meta_scanlines,
							       4096);
				uv_meta_scanlines =
				video_uv_meta_scanlines(colorformat, (pix_height + 1) >> 1);
				uv_meta_plane = MSM_MEDIA_ALIGN(uv_meta_stride *
								uv_meta_scanlines,
								4096);
				size = (y_ubwc_plane + uv_ubwc_plane + y_meta_plane +
					uv_meta_plane)*2;
			} else {
				y_sclines = video_y_scanlines(colorformat, pix_height);
				y_ubwc_plane =
					MSM_MEDIA_ALIGN(y_stride * y_sclines, 4096);
				uv_sclines = video_uv_scanlines(colorformat, pix_height);
				uv_ubwc_plane =
					MSM_MEDIA_ALIGN(uv_stride * uv_sclines, 4096);
				y_meta_scanlines =
					video_y_meta_scanlines(colorformat, pix_height);
				y_meta_plane = MSM_MEDIA_ALIGN(y_meta_stride *
							       y_meta_scanlines,
							       4096);
				uv_meta_scanlines =
					video_uv_meta_scanlines(colorformat, pix_height);
				uv_meta_plane = MSM_MEDIA_ALIGN(uv_meta_stride *
								uv_meta_scanlines,
								4096);
				size = (y_ubwc_plane + uv_ubwc_plane + y_meta_plane +
					uv_meta_plane);
			}
		}
		break;
	case MSM_VIDC_FMT_TP10C:
		y_ubwc_plane = MSM_MEDIA_ALIGN(y_stride * y_sclines, 4096);
		uv_ubwc_plane = MSM_MEDIA_ALIGN(uv_stride * uv_sclines, 4096);
		y_meta_stride = video_y_meta_stride(colorformat, pix_width);
		y_meta_scanlines = video_y_meta_scanlines(colorformat, pix_height);
		y_meta_plane = MSM_MEDIA_ALIGN(y_meta_stride *
					       y_meta_scanlines, 4096);
		uv_meta_stride = video_uv_meta_stride(colorformat, pix_width);
		uv_meta_scanlines = video_uv_meta_scanlines(colorformat, pix_height);
		uv_meta_plane = MSM_MEDIA_ALIGN(uv_meta_stride *
						uv_meta_scanlines,
						4096);

		size = y_ubwc_plane + uv_ubwc_plane + y_meta_plane +
			uv_meta_plane;
		break;
	case MSM_VIDC_FMT_RGBA8888C:
		rgb_ubwc_plane = MSM_MEDIA_ALIGN(rgb_stride * rgb_scanlines,
						 4096);
		rgb_meta_stride = video_rgb_meta_stride(colorformat, pix_width);
		rgb_meta_scanlines = video_rgb_meta_scanlines(colorformat,
							      pix_height);
		rgb_meta_plane = MSM_MEDIA_ALIGN(rgb_meta_stride *
						 rgb_meta_scanlines, 4096);
		size = rgb_ubwc_plane + rgb_meta_plane;
		break;
	case MSM_VIDC_FMT_RGBA8888:
		rgb_plane = MSM_MEDIA_ALIGN(rgb_stride * rgb_scanlines, 4096);
		size = rgb_plane;
		break;
	default:
		break;
	}

invalid_input:
	size = MSM_MEDIA_ALIGN(size, 4096);
	return size;
}

#endif
