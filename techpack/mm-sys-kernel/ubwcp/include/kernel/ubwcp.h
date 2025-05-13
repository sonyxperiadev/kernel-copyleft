/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __UBWCP_H_
#define __UBWCP_H_

#include <linux/types.h>
#include <linux/dma-buf.h>

#include "../uapi/ubwcp_ioctl.h"


typedef int (*configure_mmap)(struct dma_buf *dmabuf, bool linear, phys_addr_t ula_pa_addr,
			      size_t ula_pa_size);

/**
 * Get UBWCP hardware version
 *
 * @param ver : ptr to ver struct where hw version will be
 *            copied
 *
 * @return int : 0 on success, otherwise error code
 */
int ubwcp_get_hw_version(struct ubwcp_ioctl_hw_version *ver);

/**
 * Configures ubwcp buffer with the provided buffer image
 * attributes. This call must be done at least once before
 * ubwcp_lock(). Attributes can be configured multiple times,
 * but only during unlocked state.
 *
 * Upon error, buffer will be in undefined state.
 * There is no guarantee that previously set attributes will be retained.
 * Caller could retry set attributes, but must not reuse buffer
 * until a successful set attribute call is done.
 *
 * @param dmabuf : ptr to the dma buf
 * @param attr   : buffer attributes to set
 *
 * @return int : 0 on success, otherwise error code
 */
int ubwcp_set_buf_attrs(struct dma_buf *dmabuf, struct ubwcp_buffer_attrs *attr);

/**
 * Get the currently configured attributes for the buffer
 *
 * @param dmabuf : ptr to the dma buf
 * @param attr   : pointer to location where image attributes
 *		   for this buffer will be copied to.
 *
 * @return int : 0 on success, otherwise error code
 */
int ubwcp_get_buf_attrs(struct dma_buf *dmabuf, struct ubwcp_buffer_attrs *attr);

/**
 * Set permanent range translation for the buffer. This reserves
 * ubwcp address translation resources for the buffer until free
 * is called. This may speed up lock()/unlock() calls as they
 * don't need to configure address translations for the buffer.
 *
 * @param dmabuf : ptr to the dma buf
 * @param enable : true == enable, false == disable
 *
 * @return int : 0 on success, otherwise error code
 */
int ubwcp_set_perm_range_translation(struct dma_buf *dmabuf, bool enable);

enum ubwcp_error {
	UBWCP_ENCODE_ERROR = 0,
	UBWCP_DECODE_ERROR,
	UBWCP_RANGE_TRANSLATION_ERROR,
	UBWCP_SMMU_FAULT,
	UBWCP_UNKNOWN_ERROR,
};

enum iommu_cb_id {
	UBWCP_DESC_CB_ID = 0,
	UBWCP_BUF_CB_ID,
	UBWCP_UNKNOWN_CB_ID,
};

struct ubwcp_enc_err_info {
	struct dma_buf *dmabuf;
	phys_addr_t ula_pa;
};

struct ubwcp_dec_err_info {
	struct dma_buf *dmabuf;
	phys_addr_t ula_pa;
};

struct ubwcp_translation_err_info {
	struct dma_buf *dmabuf;
	phys_addr_t ula_pa;
	bool read;
};

struct ubwcp_smmu_fault_err_info {
	struct dma_buf *dmabuf;
	unsigned long iova;
	enum iommu_cb_id iommu_dev_id;
	int iommu_fault_flags;
};

struct ubwcp_err_info {
	enum ubwcp_error err_code;
	union {
		struct ubwcp_enc_err_info enc_err;
		struct ubwcp_dec_err_info dec_err;
		struct ubwcp_translation_err_info translation_err;
		struct ubwcp_smmu_fault_err_info smmu_err;
	};
};

typedef void (*ubwcp_error_handler_t)(struct ubwcp_err_info *err, void *data);

/*
 * Register an error handler
 *
 * @param client_id : not currently supported (pass in -1)
 * @param handler   : the error handler function which will be called when an
 *		      error occurs
 * @param data      : data pointer provided with the error handler function
 *
 * @return int : 0 on success, otherwise error code
 */
int ubwcp_register_error_handler(u32 client_id, ubwcp_error_handler_t handler,
				void *data);

/*
 * Unregister an error handler
 *
 * @param client_id : client id of handler to unregister (pass in -1)
 *
 * @return int : 0 on success, otherwise error code
 */
int ubwcp_unregister_error_handler(u32 client_id);

#endif /* __UBWCP_H_ */
