/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2017, 2019, 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _QSEECOM_H_
#define _QSEECOM_H_

#pragma message("Warning: This header file will be deprecated in future")

#include <linux/types.h>
#include <linux/ioctl.h>

#define MAX_ION_FD  4
#define MAX_APP_NAME_SIZE  64
#define QSEECOM_HASH_SIZE  32

#define ICE_KEY_SIZE 32
#define ICE_SALT_SIZE 32

/*
 * struct qseecom_ion_fd_info - ion fd handle data information
 * @fd - ion handle to some memory allocated in user space
 * @cmd_buf_offset - command buffer offset
 */
struct qseecom_ion_fd_info {
	__s32 fd;
	__u32 cmd_buf_offset;
};

enum qseecom_key_management_usage_type {
	QSEOS_KM_USAGE_DISK_ENCRYPTION = 0x01,
	QSEOS_KM_USAGE_FILE_ENCRYPTION = 0x02,
	QSEOS_KM_USAGE_UFS_ICE_DISK_ENCRYPTION = 0x03,
	QSEOS_KM_USAGE_SDCC_ICE_DISK_ENCRYPTION = 0x04,
	QSEOS_KM_USAGE_MAX
};

struct qseecom_create_key_req {
	unsigned char hash32[QSEECOM_HASH_SIZE];
	enum qseecom_key_management_usage_type usage;
};

struct qseecom_wipe_key_req {
	enum qseecom_key_management_usage_type usage;
	int wipe_key_flag;/* 1->remove key from storage(alone with clear key) */
			  /* 0->do not remove from storage (clear key) */
};

struct qseecom_update_key_userinfo_req {
	unsigned char current_hash32[QSEECOM_HASH_SIZE];
	unsigned char new_hash32[QSEECOM_HASH_SIZE];
	enum qseecom_key_management_usage_type usage;
};

#define SHA256_DIGEST_LENGTH	(256/8)
/*
 * struct qseecom_save_partition_hash_req
 * @partition_id - partition id.
 * @hash[SHA256_DIGEST_LENGTH] -  sha256 digest.
 */
struct qseecom_save_partition_hash_req {
	int partition_id; /* in */
	char digest[SHA256_DIGEST_LENGTH]; /* in */
};

/*
 * struct qseecom_is_es_activated_req
 * @is_activated - 1=true , 0=false
 */
struct qseecom_is_es_activated_req {
	int is_activated; /* out */
};

/*
 * struct qseecom_mdtp_cipher_dip_req
 * @in_buf - input buffer
 * @in_buf_size - input buffer size
 * @out_buf - output buffer
 * @out_buf_size - output buffer size
 * @direction - 0=encrypt, 1=decrypt
 */
struct qseecom_mdtp_cipher_dip_req {
	__u8 *in_buf;
	__u32 in_buf_size;
	__u8 *out_buf;
	__u32 out_buf_size;
	__u32 direction;
};

struct qseecom_qteec_req {
	void    *req_ptr;
	__u32    req_len;
	void    *resp_ptr;
	__u32    resp_len;
};

struct qseecom_qteec_modfd_req {
	void    *req_ptr;
	__u32    req_len;
	void    *resp_ptr;
	__u32    resp_len;
	struct qseecom_ion_fd_info ifd_data[MAX_ION_FD];
};

#define MAX_CE_PIPE_PAIR_PER_UNIT 3

struct qseecom_ce_pipe_entry {
	int valid;
	unsigned int ce_num;
	unsigned int ce_pipe_pair;
};

struct qseecom_ice_data_t {
	int flag;
};

#define MAX_CE_INFO_HANDLE_SIZE 32
struct qseecom_ce_info_req {
	unsigned char handle[MAX_CE_INFO_HANDLE_SIZE];
	unsigned int usage;
	unsigned int unit_num;
	unsigned int num_ce_pipe_entries;
	struct qseecom_ce_pipe_entry ce_pipe_entry[MAX_CE_PIPE_PAIR_PER_UNIT];
};

struct qseecom_ice_key_data_t {
	__u8 key[ICE_KEY_SIZE];
	__u32 key_len;
	__u8 salt[ICE_SALT_SIZE];
	__u32 salt_len;
};

struct file;


#define QSEECOM_IOC_MAGIC    0x97


#define QSEECOM_IOCTL_CREATE_KEY_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 17, struct qseecom_create_key_req)

#define QSEECOM_IOCTL_WIPE_KEY_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 18, struct qseecom_wipe_key_req)

#define QSEECOM_IOCTL_SAVE_PARTITION_HASH_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 19, struct qseecom_save_partition_hash_req)

#define QSEECOM_IOCTL_IS_ES_ACTIVATED_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 20, struct qseecom_is_es_activated_req)

#define QSEECOM_IOCTL_UPDATE_KEY_USER_INFO_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 24, struct qseecom_update_key_userinfo_req)

#define QSEECOM_QTEEC_IOCTL_OPEN_SESSION_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 30, struct qseecom_qteec_modfd_req)

#define QSEECOM_QTEEC_IOCTL_CLOSE_SESSION_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 31, struct qseecom_qteec_req)

#define QSEECOM_QTEEC_IOCTL_INVOKE_MODFD_CMD_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 32, struct qseecom_qteec_modfd_req)

#define QSEECOM_QTEEC_IOCTL_REQUEST_CANCELLATION_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 33, struct qseecom_qteec_modfd_req)

#define QSEECOM_IOCTL_MDTP_CIPHER_DIP_REQ \
	_IOWR(QSEECOM_IOC_MAGIC, 34, struct qseecom_mdtp_cipher_dip_req)

#define QSEECOM_IOCTL_GET_CE_PIPE_INFO \
	_IOWR(QSEECOM_IOC_MAGIC, 40, struct qseecom_ce_info_req)

#define QSEECOM_IOCTL_FREE_CE_PIPE_INFO \
	_IOWR(QSEECOM_IOC_MAGIC, 41, struct qseecom_ce_info_req)

#define QSEECOM_IOCTL_QUERY_CE_PIPE_INFO \
	_IOWR(QSEECOM_IOC_MAGIC, 42, struct qseecom_ce_info_req)

#define QSEECOM_IOCTL_SET_ICE_INFO \
	_IOWR(QSEECOM_IOC_MAGIC, 43, struct qseecom_ice_data_t)

#define QSEECOM_IOCTL_FBE_CLEAR_KEY \
	_IOWR(QSEECOM_IOC_MAGIC, 44, struct qseecom_ice_key_data_t)

#endif /* _QSEECOM_H_ */
