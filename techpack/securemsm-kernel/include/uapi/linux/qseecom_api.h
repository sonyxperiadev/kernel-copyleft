/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2017, 2019, 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _QSEECOM_API_H_
#define _QSEECOM_API_H_

#pragma message("Warning: This header file will be deprecated in future")

#include <linux/types.h>
#include <linux/ioctl.h>
#include "qseecom.h"

/*
 * struct qseecom_register_listener_req -
 *      for register listener ioctl request
 * @listener_id - service id (shared between userspace and QSE)
 * @ifd_data_fd - ion handle
 * @virt_sb_base - shared buffer base in user space
 * @sb_size - shared buffer size
 */
struct qseecom_register_listener_req {
	__u32 listener_id; /* in */
	__s32 ifd_data_fd; /* in */
	void *virt_sb_base; /* in */
	__u32 sb_size; /* in */
};

/*
 * struct qseecom_send_cmd_req - for send command ioctl request
 * @cmd_req_len - command buffer length
 * @cmd_req_buf - command buffer
 * @resp_len - response buffer length
 * @resp_buf - response buffer
 */
struct qseecom_send_cmd_req {
	void *cmd_req_buf; /* in */
	unsigned int cmd_req_len; /* in */
	void *resp_buf; /* in/out */
	unsigned int resp_len; /* in/out */
};

/*
 * struct qseecom_send_modfd_cmd_req - for send command ioctl request
 * @cmd_req_len - command buffer length
 * @cmd_req_buf - command buffer
 * @resp_len - response buffer length
 * @resp_buf - response buffer
 * @ifd_data_fd - ion handle to memory allocated in user space
 * @cmd_buf_offset - command buffer offset
 */
struct qseecom_send_modfd_cmd_req {
	void *cmd_req_buf; /* in */
	unsigned int cmd_req_len; /* in */
	void *resp_buf; /* in/out */
	unsigned int resp_len; /* in/out */
	struct qseecom_ion_fd_info ifd_data[MAX_ION_FD];
};

/*
 * struct qseecom_load_img_data - for sending image length information and
 * ion file descriptor to the qseecom driver. ion file descriptor is used
 * for retrieving the ion file handle and in turn the physical address of
 * the image location.
 * @mdt_len - Length of the .mdt file in bytes.
 * @img_len - Length of the .mdt + .b00 +..+.bxx images files in bytes
 * @ion_fd - Ion file descriptor used when allocating memory.
 * @img_name - Name of the image.
 * @app_arch - Architecture of the image, i.e. 32bit or 64bit app
 */
struct qseecom_load_img_req {
	__u32 mdt_len; /* in */
	__u32 img_len; /* in */
	__s32  ifd_data_fd; /* in */
	char	 img_name[MAX_APP_NAME_SIZE]; /* in */
	__u32 app_arch; /* in */
	__u32 app_id; /* out*/
};

struct qseecom_set_sb_mem_param_req {
	__s32 ifd_data_fd; /* in */
	void *virt_sb_base; /* in */
	__u32 sb_len; /* in */
};

/*
 * struct qseecom_qseos_version_req - get qseos version
 * @qseos_version - version number
 */
struct qseecom_qseos_version_req {
	unsigned int qseos_version; /* in */
};

/*
 * struct qseecom_qseos_app_load_query - verify if app is loaded in qsee
 * @app_name[MAX_APP_NAME_SIZE]-  name of the app.
 * @app_id - app id.
 */
struct qseecom_qseos_app_load_query {
	char app_name[MAX_APP_NAME_SIZE]; /* in */
	__u32 app_id; /* out */
	__u32 app_arch;
};

struct qseecom_send_svc_cmd_req {
	__u32 cmd_id;
	void *cmd_req_buf; /* in */
	unsigned int cmd_req_len; /* in */
	void *resp_buf; /* in/out */
	unsigned int resp_len; /* in/out */
};

/*
 * struct qseecom_send_modfd_resp - for send command ioctl request
 * @req_len - command buffer length
 * @req_buf - command buffer
 * @ifd_data_fd - ion handle to memory allocated in user space
 * @cmd_buf_offset - command buffer offset
 */
struct qseecom_send_modfd_listener_resp {
	void *resp_buf_ptr; /* in */
	unsigned int resp_len; /* in */
	struct qseecom_ion_fd_info ifd_data[MAX_ION_FD]; /* in */
};

struct qseecom_sg_entry {
	__u32 phys_addr;
	__u32 len;
};

struct qseecom_sg_entry_64bit {
	__u64 phys_addr;
	__u32 len;
} __attribute__ ((packed));


struct file;


#define QSEECOM_IOC_MAGIC    0x97


#define QSEECOM_IOCTL_REGISTER_LISTENER_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 1, struct qseecom_register_listener_req)

#define QSEECOM_IOCTL_UNREGISTER_LISTENER_REQ \
	_IO(QSEECOM_IOC_MAGIC, 2)

#define QSEECOM_IOCTL_SEND_CMD_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 3, struct qseecom_send_cmd_req)

#define QSEECOM_IOCTL_SEND_MODFD_CMD_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 4, struct qseecom_send_modfd_cmd_req)

#define QSEECOM_IOCTL_RECEIVE_REQ \
	_IO(QSEECOM_IOC_MAGIC, 5)

#define QSEECOM_IOCTL_SEND_RESP_REQ \
	_IO(QSEECOM_IOC_MAGIC, 6)

#define QSEECOM_IOCTL_LOAD_APP_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 7, struct qseecom_load_img_req)

#define QSEECOM_IOCTL_SET_MEM_PARAM_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 8, struct qseecom_set_sb_mem_param_req)

#define QSEECOM_IOCTL_UNLOAD_APP_REQ \
	_IO(QSEECOM_IOC_MAGIC, 9)

#define QSEECOM_IOCTL_GET_QSEOS_VERSION_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 10, struct qseecom_qseos_version_req)

#define QSEECOM_IOCTL_LOAD_EXTERNAL_ELF_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 13, struct qseecom_load_img_req)

#define QSEECOM_IOCTL_UNLOAD_EXTERNAL_ELF_REQ \
	_IO(QSEECOM_IOC_MAGIC, 14)

#define QSEECOM_IOCTL_APP_LOADED_QUERY_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 15, struct qseecom_qseos_app_load_query)

#define QSEECOM_IOCTL_SEND_CMD_SERVICE_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 16, struct qseecom_send_svc_cmd_req)

#define QSEECOM_IOCTL_SEND_MODFD_RESP \
	_IOWR(QSEECOM_IOC_MAGIC, 21, struct qseecom_send_modfd_listener_resp)

#define QSEECOM_IOCTL_SEND_MODFD_CMD_64_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 35, struct qseecom_send_modfd_cmd_req)

#define QSEECOM_IOCTL_SEND_MODFD_RESP_64 \
	_IOWR(QSEECOM_IOC_MAGIC, 36, struct qseecom_send_modfd_listener_resp)

#endif /* _QSEECOM_API_H_ */
