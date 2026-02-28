// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2018, 2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/qseecom_api.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/scatterlist.h>
#include <linux/interconnect.h>
#include <linux/delay.h>
#include "qseecom_32bit_impl.h"
#include <linux/compat.h>


static int get_qseecom_register_listener_req_32bit(
		struct qseecom_register_listener_req_32bit __user *data32,
		struct qseecom_register_listener_req *data)
{
	int err;
	compat_ulong_t listener_id;
	compat_long_t ifd_data_fd;
	compat_uptr_t virt_sb_base;
	compat_ulong_t sb_size;

	err = get_user(listener_id, &data32->listener_id);
	memcpy(&data->listener_id, &listener_id, sizeof(listener_id));
	err |= get_user(ifd_data_fd, &data32->ifd_data_fd);
	memcpy(&data->ifd_data_fd, &ifd_data_fd, sizeof(ifd_data_fd));
	err |= get_user(virt_sb_base, &data32->virt_sb_base);
	data->virt_sb_base = NULL;
	memcpy(&data->virt_sb_base, &virt_sb_base, sizeof(virt_sb_base));
	err |= get_user(sb_size, &data32->sb_size);
	memcpy(&data->sb_size, &sb_size, sizeof(sb_size));
	return err;
}

static int get_qseecom_load_img_req_32bit(
		struct qseecom_load_img_req_32bit __user *data32,
		struct qseecom_load_img_req *data)
{
	int err;
	compat_ulong_t mdt_len;
	compat_ulong_t img_len;
	compat_long_t ifd_data_fd;
	compat_ulong_t app_arch;
	compat_uint_t app_id;
	char img_name;
	unsigned int i;

	err = get_user(mdt_len, &data32->mdt_len);
	memcpy(&data->mdt_len, &mdt_len, sizeof(mdt_len));
	err |= get_user(img_len, &data32->img_len);
	memcpy(&data->img_len, &img_len, sizeof(img_len));
	err |= get_user(ifd_data_fd, &data32->ifd_data_fd);
	memcpy(&data->ifd_data_fd, &ifd_data_fd, sizeof(ifd_data_fd));
	for (i = 0; i < MAX_APP_NAME_SIZE; i++) {
		err |= get_user(img_name, &(data32->img_name[i]));
		memcpy(&data->img_name[i], &img_name, sizeof(img_name));
	}
	err |= get_user(app_arch, &data32->app_arch);
	memcpy(&data->app_arch, &app_arch, sizeof(app_arch));
	err |= get_user(app_id, &data32->app_id);
	memcpy(&data->app_id, &app_id, sizeof(app_id));
	return err;
}

static int get_qseecom_send_cmd_req_32bit(
		struct qseecom_send_cmd_req_32bit __user *data32,
		struct qseecom_send_cmd_req *data)
{
	int err;
	compat_uptr_t cmd_req_buf;
	compat_uint_t cmd_req_len;
	compat_uptr_t resp_buf;
	compat_uint_t resp_len;

	err = get_user(cmd_req_buf, &data32->cmd_req_buf);
	data->cmd_req_buf = NULL;
	memcpy(&data->cmd_req_buf, &cmd_req_buf, sizeof(cmd_req_buf));
	err |= get_user(cmd_req_len, &data32->cmd_req_len);
	memcpy(&data->cmd_req_len, &cmd_req_len, sizeof(cmd_req_len));
	err |= get_user(resp_buf, &data32->resp_buf);
	data->resp_buf = NULL;
	memcpy(&data->resp_buf, &resp_buf, sizeof(resp_buf));
	err |= get_user(resp_len, &data32->resp_len);
	memcpy(&data->resp_len, &resp_len, sizeof(resp_len));
	return err;
}

static int get_qseecom_send_modfd_cmd_req_32bit(
		struct qseecom_send_modfd_cmd_req_32bit __user *data32,
		struct qseecom_send_modfd_cmd_req *data)
{
	int err;
	unsigned int i;
	compat_uptr_t cmd_req_buf;
	compat_uint_t cmd_req_len;
	compat_uptr_t resp_buf;
	compat_uint_t resp_len;
	compat_long_t fd;
	compat_ulong_t cmd_buf_offset;

	err = get_user(cmd_req_buf, &data32->cmd_req_buf);
	data->cmd_req_buf = NULL;
	memcpy(&data->cmd_req_buf, &cmd_req_buf, sizeof(cmd_req_buf));
	err |= get_user(cmd_req_len, &data32->cmd_req_len);
	memcpy(&data->cmd_req_len, &cmd_req_len, sizeof(cmd_req_len));
	err |= get_user(resp_buf, &data32->resp_buf);
	data->resp_buf = NULL;
	memcpy(&data->resp_buf, &resp_buf, sizeof(resp_buf));
	err |= get_user(resp_len, &data32->resp_len);
	memcpy(&data->resp_len, &resp_len, sizeof(resp_len));
	for (i = 0; i < MAX_ION_FD; i++) {
		err |= get_user(fd, &data32->ifd_data[i].fd);
		memcpy(&data->ifd_data[i].fd, &fd, sizeof(fd));
		err |= get_user(cmd_buf_offset,
				&data32->ifd_data[i].cmd_buf_offset);
		memcpy(&data->ifd_data[i].cmd_buf_offset, &cmd_buf_offset, sizeof(cmd_buf_offset));
	}
	return err;
}

static int get_qseecom_set_sb_mem_param_req_32bit(
		struct qseecom_set_sb_mem_param_req_32bit __user *data32,
		struct qseecom_set_sb_mem_param_req *data)
{
	int err;
	compat_long_t ifd_data_fd;
	compat_uptr_t virt_sb_base;
	compat_ulong_t sb_len;

	err = get_user(ifd_data_fd, &data32->ifd_data_fd);
	memcpy(&data->ifd_data_fd, &ifd_data_fd, sizeof(ifd_data_fd));
	err |= get_user(virt_sb_base, &data32->virt_sb_base);
	data->virt_sb_base = NULL;
	memcpy(&data->virt_sb_base, &virt_sb_base, sizeof(virt_sb_base));
	err |= get_user(sb_len, &data32->sb_len);
	memcpy(&data->sb_len, &sb_len, sizeof(sb_len));
	return err;
}

static int get_qseecom_qseos_version_req_32bit(
		struct qseecom_qseos_version_req_32bit __user *data32,
		struct qseecom_qseos_version_req *data)
{
	int err;
	compat_uint_t qseos_version;

	err = get_user(qseos_version, &data32->qseos_version);
	memcpy(&data->qseos_version, &qseos_version, sizeof(qseos_version));
	return err;
}

static int get_qseecom_qseos_app_load_query_32bit(
		struct qseecom_qseos_app_load_query_32bit __user *data32,
		struct qseecom_qseos_app_load_query *data)
{
	int err = 0;
	unsigned int i;
	compat_uint_t app_id;
	char app_name;
	compat_ulong_t app_arch;

	for (i = 0; i < MAX_APP_NAME_SIZE; i++) {
		err |= get_user(app_name, &(data32->app_name[i]));
		memcpy(&data->app_name[i], &app_name, sizeof(app_name));
	}
	err |= get_user(app_id, &data32->app_id);
	memcpy(&data->app_id, &app_id, sizeof(app_id));
	err |= get_user(app_arch, &data32->app_arch);
	memcpy(&data->app_arch, &app_arch, sizeof(app_arch));
	return err;
}

static int get_qseecom_send_svc_cmd_req_32bit(
		struct qseecom_send_svc_cmd_req_32bit __user *data32,
		struct qseecom_send_svc_cmd_req *data)
{
	int err;
	compat_ulong_t cmd_id;
	compat_uptr_t cmd_req_buf;
	compat_uint_t cmd_req_len;
	compat_uptr_t resp_buf;
	compat_uint_t resp_len;

	err = get_user(cmd_id, &data32->cmd_id);
	memcpy(&data->cmd_id, &cmd_id, sizeof(cmd_id));
	err |= get_user(cmd_req_buf, &data32->cmd_req_buf);
	data->cmd_req_buf = NULL;
	memcpy(&data->cmd_req_buf, &cmd_req_buf, sizeof(cmd_req_buf));
	err |= get_user(cmd_req_len, &data32->cmd_req_len);
	memcpy(&data->cmd_req_len, &cmd_req_len, sizeof(cmd_req_len));
	err |= get_user(resp_buf, &data32->resp_buf);
	data->resp_buf = NULL;
	memcpy(&data->resp_buf, &resp_buf, sizeof(resp_buf));
	err |= get_user(resp_len, &data32->resp_len);
	memcpy(&data->resp_len, &resp_len, sizeof(resp_len));
	return err;
}

static int get_qseecom_create_key_req_32bit(
		struct qseecom_create_key_req_32bit __user *data32,
		struct qseecom_create_key_req *data)
{
	int err = 0;
	compat_uint_t usage;
	unsigned char hash32;
	unsigned int i;

	for (i = 0; i < QSEECOM_HASH_SIZE; i++) {
		err |= get_user(hash32, &(data32->hash32[i]));
		memcpy(&data->hash32[i], &hash32, sizeof(hash32));
	}
	err = get_user(usage, &data32->usage);
	memcpy(&data->usage, &usage, sizeof(usage));

	return err;
}

static int get_qseecom_wipe_key_req_32bit(
		struct qseecom_wipe_key_req_32bit __user *data32,
		struct qseecom_wipe_key_req *data)
{
	int err;
	compat_uint_t usage;
	compat_int_t wipe_key_flag;

	err = get_user(usage, &data32->usage);
	memcpy(&data->usage, &usage, sizeof(usage));
	err |= get_user(wipe_key_flag, &data32->wipe_key_flag);
	memcpy(&data->wipe_key_flag, &wipe_key_flag, sizeof(wipe_key_flag));

	return err;
}

static int get_qseecom_update_key_userinfo_req_32bit(
		struct qseecom_update_key_userinfo_req_32bit __user *data32,
		struct qseecom_update_key_userinfo_req *data)
{
	int err = 0;
	compat_uint_t usage;
	unsigned char current_hash32;
	unsigned char new_hash32;
	unsigned int i;

	for (i = 0; i < QSEECOM_HASH_SIZE; i++) {
		err |= get_user(current_hash32, &(data32->current_hash32[i]));
		memcpy(&data->current_hash32[i], &current_hash32, sizeof(current_hash32));
	}
	for (i = 0; i < QSEECOM_HASH_SIZE; i++) {
		err |= get_user(new_hash32, &(data32->new_hash32[i]));
		memcpy(&data->new_hash32[i], &new_hash32, sizeof(new_hash32));
	}
	err = get_user(usage, &data32->usage);
	memcpy(&data->usage, &usage, sizeof(usage));

	return err;
}

static int get_qseecom_save_partition_hash_req_32bit(
		struct qseecom_save_partition_hash_req_32bit __user *data32,
		struct qseecom_save_partition_hash_req *data)
{
	int err;
	compat_int_t partition_id;
	char digest;
	unsigned int i;

	err = get_user(partition_id, &data32->partition_id);
	memcpy(&data->partition_id, &partition_id, sizeof(partition_id));
	for (i = 0; i < SHA256_DIGEST_LENGTH; i++) {
		err |= get_user(digest, &(data32->digest[i]));
		memcpy(&data->digest[i], &digest, sizeof(digest));
	}
	return err;
}

static int get_qseecom_is_es_activated_req_32bit(
		struct qseecom_is_es_activated_req_32bit __user *data32,
		struct qseecom_is_es_activated_req *data)
{
	compat_int_t is_activated;
	int err;

	err = get_user(is_activated, &data32->is_activated);
	memcpy(&data->is_activated, &is_activated, sizeof(is_activated));
	return err;
}

static int get_qseecom_mdtp_cipher_dip_req_32bit(
		struct qseecom_mdtp_cipher_dip_req_32bit __user *data32,
		struct qseecom_mdtp_cipher_dip_req *data)
{
	int err;
	compat_int_t in_buf_size;
	compat_uptr_t in_buf;
	compat_int_t out_buf_size;
	compat_uptr_t out_buf;
	compat_int_t direction;

	err = get_user(in_buf_size, &data32->in_buf_size);
	memcpy(&data->in_buf_size, &in_buf_size, sizeof(in_buf_size));
	err |= get_user(out_buf_size, &data32->out_buf_size);
	memcpy(&data->out_buf_size, &out_buf_size, sizeof(out_buf_size));
	err |= get_user(direction, &data32->direction);
	memcpy(&data->direction, &direction, sizeof(direction));
	err |= get_user(in_buf, &data32->in_buf);
	data->in_buf = NULL;
	memcpy(&data->in_buf, &in_buf, sizeof(in_buf));
	err |= get_user(out_buf, &data32->out_buf);
	data->out_buf = NULL;
	memcpy(&data->out_buf, &out_buf, sizeof(out_buf));

	return err;
}

static int get_qseecom_send_modfd_listener_resp_32bit(
		struct qseecom_send_modfd_listener_resp_32bit __user *data32,
		struct qseecom_send_modfd_listener_resp *data)
{
	int err;
	unsigned int i;
	compat_uptr_t resp_buf_ptr;
	compat_uint_t resp_len;
	compat_long_t fd;
	compat_ulong_t cmd_buf_offset;

	err = get_user(resp_buf_ptr, &data32->resp_buf_ptr);
	data->resp_buf_ptr = NULL;
	memcpy(&data->resp_buf_ptr, &resp_buf_ptr, sizeof(resp_buf_ptr));
	err |= get_user(resp_len, &data32->resp_len);
	memcpy(&data->resp_len, &resp_len, sizeof(resp_len));
	for (i = 0; i < MAX_ION_FD; i++) {
		err |= get_user(fd, &data32->ifd_data[i].fd);
		memcpy(&data->ifd_data[i].fd, &fd, sizeof(fd));
		err |= get_user(cmd_buf_offset,
				&data32->ifd_data[i].cmd_buf_offset);
		memcpy(&data->ifd_data[i].cmd_buf_offset, &cmd_buf_offset, sizeof(cmd_buf_offset));
	}
	return err;
}


static int get_qseecom_qteec_req_32bit(
		struct qseecom_qteec_req_32bit __user *data32,
		struct qseecom_qteec_req *data)
{
	compat_uptr_t req_ptr;
	compat_ulong_t req_len;
	compat_uptr_t resp_ptr;
	compat_ulong_t resp_len;
	int err;

	err = get_user(req_ptr, &data32->req_ptr);
	data->req_ptr = NULL;
	memcpy(&data->req_ptr, &req_ptr, sizeof(req_ptr));
	err |= get_user(req_len, &data32->req_len);
	memcpy(&data->req_len, &req_len, sizeof(req_len));
	err |= get_user(resp_ptr, &data32->resp_ptr);
	data->resp_ptr = NULL;
	memcpy(&data->resp_ptr, &resp_ptr, sizeof(resp_ptr));
	err |= get_user(resp_len, &data32->resp_len);
	memcpy(&data->resp_len, &resp_len, sizeof(resp_len));
	return err;
}

static int get_qseecom_qteec_modfd_req_32bit(
		struct qseecom_qteec_modfd_req_32bit __user *data32,
		struct qseecom_qteec_modfd_req *data)
{
	compat_uptr_t req_ptr;
	compat_ulong_t req_len;
	compat_uptr_t resp_ptr;
	compat_ulong_t resp_len;
	compat_long_t fd;
	compat_ulong_t cmd_buf_offset;
	int err, i;

	err = get_user(req_ptr, &data32->req_ptr);
	data->req_ptr = NULL;
	memcpy(&data->req_ptr, &req_ptr, sizeof(req_ptr));
	err |= get_user(req_len, &data32->req_len);
	memcpy(&data->req_len, &req_len, sizeof(req_len));

	err |= get_user(resp_ptr, &data32->resp_ptr);
	data->resp_ptr = NULL;
	memcpy(&data->resp_ptr, &resp_ptr, sizeof(resp_ptr));
	err |= get_user(resp_len, &data32->resp_len);
	memcpy(&data->resp_len, &resp_len, sizeof(resp_len));

	for (i = 0; i < MAX_ION_FD; i++) {
		err |= get_user(fd, &data32->ifd_data[i].fd);
		memcpy(&data->ifd_data[i].fd, &fd, sizeof(fd));
		err |= get_user(cmd_buf_offset,
				&data32->ifd_data[i].cmd_buf_offset);
		memcpy(&data->ifd_data[i].cmd_buf_offset, &cmd_buf_offset, sizeof(cmd_buf_offset));
	}
	return err;
}

static int put_qseecom_load_img_req_32bit(
		struct qseecom_load_img_req_32bit __user *data32,
		struct qseecom_load_img_req *data)
{
	int err;
	compat_ulong_t mdt_len;
	compat_ulong_t img_len;
	compat_long_t ifd_data_fd;
	compat_ulong_t app_arch;
	compat_int_t app_id;
	char img_name;
	unsigned int i;

	memcpy(&mdt_len, &data->mdt_len, sizeof(mdt_len));
	err = put_user(mdt_len, &data32->mdt_len);
	memcpy(&img_len, &data->img_len, sizeof(img_len));
	err |= put_user(img_len, &data32->img_len);
	memcpy(&ifd_data_fd, &data->ifd_data_fd, sizeof(ifd_data_fd));
	err |= put_user(ifd_data_fd, &data32->ifd_data_fd);
	for (i = 0; i < MAX_APP_NAME_SIZE; i++) {
		memcpy(&img_name, &data->img_name[i], sizeof(img_name));
		err |= put_user(img_name, &data32->img_name[i]);
	}
	memcpy(&app_arch, &data->app_arch, sizeof(app_arch));
	err |= put_user(app_arch, &data32->app_arch);
	memcpy(&app_id, &data->app_id, sizeof(app_id));
	err |= put_user(app_id, &data32->app_id);
	return err;
}

static int put_qseecom_qseos_version_req_32bit(
		struct qseecom_qseos_version_req_32bit __user *data32,
		struct qseecom_qseos_version_req *data)
{
	compat_uint_t qseos_version;
	int err;

	memcpy(&qseos_version, &data->qseos_version, sizeof(qseos_version));
	err = put_user(qseos_version, &data32->qseos_version);
	return err;
}

static int put_qseecom_qseos_app_load_query_32bit(
		struct qseecom_qseos_app_load_query_32bit __user *data32,
		struct qseecom_qseos_app_load_query *data)
{
	int err = 0;
	unsigned int i;
	compat_int_t app_id;
	compat_ulong_t app_arch;
	char app_name;

	for (i = 0; i < MAX_APP_NAME_SIZE; i++) {
		memcpy(&app_name, &data->app_name[i], sizeof(app_name));
		err |= put_user(app_name, &(data32->app_name[i]));
	}
	memcpy(&app_id, &data->app_id, sizeof(app_id));
	err |= put_user(app_id, &data32->app_id);
	memcpy(&app_arch, &data->app_arch, sizeof(app_arch));
	err |= put_user(app_arch, &data32->app_arch);

	return err;
}

static int put_qseecom_is_es_activated_req_32bit(
		struct qseecom_is_es_activated_req_32bit __user *data32,
		struct qseecom_is_es_activated_req *data)
{
	compat_int_t is_activated;
	int err;

	memcpy(&is_activated, &data->is_activated, sizeof(is_activated));
	err = put_user(is_activated, &data32->is_activated);
	return err;
}

static unsigned int convert_cmd(unsigned int cmd)
{
	switch (cmd) {
	case QSEECOM_IOCTL_REGISTER_LISTENER_REQ_32BIT:
		return QSEECOM_IOCTL_REGISTER_LISTENER_REQ;
	case QSEECOM_IOCTL_UNREGISTER_LISTENER_REQ_32BIT:
		return QSEECOM_IOCTL_UNREGISTER_LISTENER_REQ;
	case QSEECOM_IOCTL_LOAD_APP_REQ_32BIT:
		return QSEECOM_IOCTL_LOAD_APP_REQ;
	case QSEECOM_IOCTL_RECEIVE_REQ_32BIT:
		return QSEECOM_IOCTL_RECEIVE_REQ;
	case QSEECOM_IOCTL_SEND_RESP_REQ_32BIT:
		return QSEECOM_IOCTL_SEND_RESP_REQ;
	case QSEECOM_IOCTL_UNLOAD_APP_REQ_32BIT:
		return QSEECOM_IOCTL_UNLOAD_APP_REQ;
	case QSEECOM_IOCTL_UNLOAD_EXTERNAL_ELF_REQ_32BIT:
		return QSEECOM_IOCTL_UNLOAD_EXTERNAL_ELF_REQ;
	case QSEECOM_IOCTL_SEND_CMD_REQ_32BIT:
		return QSEECOM_IOCTL_SEND_CMD_REQ;
	case QSEECOM_IOCTL_SEND_MODFD_CMD_REQ_32BIT:
		return QSEECOM_IOCTL_SEND_MODFD_CMD_REQ;
	case QSEECOM_IOCTL_SET_MEM_PARAM_REQ_32BIT:
		return QSEECOM_IOCTL_SET_MEM_PARAM_REQ;
	case QSEECOM_IOCTL_GET_QSEOS_VERSION_REQ_32BIT:
		return QSEECOM_IOCTL_GET_QSEOS_VERSION_REQ;
	case QSEECOM_IOCTL_LOAD_EXTERNAL_ELF_REQ_32BIT:
		return QSEECOM_IOCTL_LOAD_EXTERNAL_ELF_REQ;
	case QSEECOM_IOCTL_APP_LOADED_QUERY_REQ_32BIT:
		return QSEECOM_IOCTL_APP_LOADED_QUERY_REQ;
	case QSEECOM_IOCTL_SEND_CMD_SERVICE_REQ_32BIT:
		return QSEECOM_IOCTL_SEND_CMD_SERVICE_REQ;
	case QSEECOM_IOCTL_CREATE_KEY_REQ_32BIT:
		return QSEECOM_IOCTL_CREATE_KEY_REQ;
	case QSEECOM_IOCTL_WIPE_KEY_REQ_32BIT:
		return QSEECOM_IOCTL_WIPE_KEY_REQ;
	case QSEECOM_IOCTL_UPDATE_KEY_USER_INFO_REQ_32BIT:
		return QSEECOM_IOCTL_UPDATE_KEY_USER_INFO_REQ;
	case QSEECOM_IOCTL_SAVE_PARTITION_HASH_REQ_32BIT:
		return QSEECOM_IOCTL_SAVE_PARTITION_HASH_REQ;
	case QSEECOM_IOCTL_IS_ES_ACTIVATED_REQ_32BIT:
		return QSEECOM_IOCTL_IS_ES_ACTIVATED_REQ;
	case QSEECOM_IOCTL_SEND_MODFD_RESP_32BIT:
		return QSEECOM_IOCTL_SEND_MODFD_RESP;
	case QSEECOM_QTEEC_IOCTL_OPEN_SESSION_REQ_32BIT:
		return QSEECOM_QTEEC_IOCTL_OPEN_SESSION_REQ;
	case QSEECOM_QTEEC_IOCTL_CLOSE_SESSION_REQ_32BIT:
		return QSEECOM_QTEEC_IOCTL_CLOSE_SESSION_REQ;
	case QSEECOM_QTEEC_IOCTL_INVOKE_MODFD_CMD_REQ_32BIT:
		return QSEECOM_QTEEC_IOCTL_INVOKE_MODFD_CMD_REQ;
	case QSEECOM_QTEEC_IOCTL_REQUEST_CANCELLATION_REQ_32BIT:
		return QSEECOM_QTEEC_IOCTL_REQUEST_CANCELLATION_REQ;
	case QSEECOM_IOCTL_MDTP_CIPHER_DIP_REQ_32BIT:
		return QSEECOM_IOCTL_MDTP_CIPHER_DIP_REQ;
	case QSEECOM_IOCTL_SEND_MODFD_CMD_64_REQ_32BIT:
		return QSEECOM_IOCTL_SEND_MODFD_CMD_64_REQ;
	case QSEECOM_IOCTL_SEND_MODFD_RESP_64_32BIT:
		return QSEECOM_IOCTL_SEND_MODFD_RESP_64;

	default:
		return cmd;
	}
}

long qseecom_ioctl_32bit(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	long ret;

	switch (cmd) {

	case QSEECOM_IOCTL_UNREGISTER_LISTENER_REQ_32BIT:
	case QSEECOM_IOCTL_RECEIVE_REQ_32BIT:
	case QSEECOM_IOCTL_SEND_RESP_REQ_32BIT:
	case QSEECOM_IOCTL_UNLOAD_APP_REQ_32BIT:
	case QSEECOM_IOCTL_UNLOAD_EXTERNAL_ELF_REQ_32BIT: {
		return qseecom_ioctl(file, convert_cmd(cmd), 0);
	}
	break;
	case QSEECOM_IOCTL_REGISTER_LISTENER_REQ_32BIT: {
		struct qseecom_register_listener_req_32bit __user *data32;
		struct qseecom_register_listener_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_register_listener_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_LOAD_APP_REQ_32BIT: {
		struct qseecom_load_img_req_32bit __user *data32;
		struct qseecom_load_img_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;
		err = get_qseecom_load_img_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		err = put_qseecom_load_img_req_32bit(data32, data);
		kfree(data);
		return ret ? ret : err;
	}
	break;
	case QSEECOM_IOCTL_SEND_CMD_REQ_32BIT: {
		struct qseecom_send_cmd_req_32bit __user *data32;
		struct qseecom_send_cmd_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_send_cmd_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_SEND_MODFD_CMD_REQ_32BIT:
	case QSEECOM_IOCTL_SEND_MODFD_CMD_64_REQ_32BIT: {
		struct qseecom_send_modfd_cmd_req_32bit __user *data32;
		struct qseecom_send_modfd_cmd_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_send_modfd_cmd_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_SET_MEM_PARAM_REQ_32BIT: {
		struct qseecom_set_sb_mem_param_req_32bit __user *data32;
		struct qseecom_set_sb_mem_param_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_set_sb_mem_param_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_GET_QSEOS_VERSION_REQ_32BIT: {
		struct qseecom_qseos_version_req_32bit __user *data32;
		struct qseecom_qseos_version_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_qseos_version_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		err = put_qseecom_qseos_version_req_32bit(data32, data);
		kfree(data);
		return ret ? ret : err;
	}
	break;
	case QSEECOM_IOCTL_LOAD_EXTERNAL_ELF_REQ_32BIT: {
		struct qseecom_load_img_req_32bit __user *data32;
		struct qseecom_load_img_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_load_img_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_APP_LOADED_QUERY_REQ_32BIT: {
		struct qseecom_qseos_app_load_query_32bit __user *data32;
		struct qseecom_qseos_app_load_query *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_qseos_app_load_query_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
					(unsigned long)data);
		err = put_qseecom_qseos_app_load_query_32bit(data32, data);
		kfree(data);
		return ret ? ret : err;
	}
	break;
	case QSEECOM_IOCTL_SEND_CMD_SERVICE_REQ_32BIT: {
		struct qseecom_send_svc_cmd_req_32bit __user *data32;
		struct qseecom_send_svc_cmd_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_send_svc_cmd_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_CREATE_KEY_REQ_32BIT: {
		struct qseecom_create_key_req_32bit __user *data32;
		struct qseecom_create_key_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_create_key_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_WIPE_KEY_REQ_32BIT: {
		struct qseecom_wipe_key_req_32bit __user *data32;
		struct qseecom_wipe_key_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_wipe_key_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_UPDATE_KEY_USER_INFO_REQ_32BIT: {
		struct qseecom_update_key_userinfo_req_32bit __user *data32;
		struct qseecom_update_key_userinfo_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_update_key_userinfo_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_SAVE_PARTITION_HASH_REQ_32BIT: {
		struct qseecom_save_partition_hash_req_32bit __user *data32;
		struct qseecom_save_partition_hash_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_save_partition_hash_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_IS_ES_ACTIVATED_REQ_32BIT: {
		struct qseecom_is_es_activated_req_32bit __user *data32;
		struct qseecom_is_es_activated_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_is_es_activated_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		err = put_qseecom_is_es_activated_req_32bit(data32, data);
		kfree(data);
		return ret ? ret : err;
	}
	break;
	case QSEECOM_IOCTL_MDTP_CIPHER_DIP_REQ_32BIT: {
		struct qseecom_mdtp_cipher_dip_req_32bit __user *data32;
		struct qseecom_mdtp_cipher_dip_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_mdtp_cipher_dip_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_IOCTL_SEND_MODFD_RESP_32BIT:
	case QSEECOM_IOCTL_SEND_MODFD_RESP_64_32BIT: {
		struct qseecom_send_modfd_listener_resp_32bit __user *data32;
		struct qseecom_send_modfd_listener_resp *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_send_modfd_listener_resp_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_QTEEC_IOCTL_CLOSE_SESSION_REQ_32BIT: {
		struct qseecom_qteec_req_32bit __user *data32;
		struct qseecom_qteec_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_qteec_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	case QSEECOM_QTEEC_IOCTL_OPEN_SESSION_REQ_32BIT:
	case QSEECOM_QTEEC_IOCTL_INVOKE_MODFD_CMD_REQ_32BIT:
	case QSEECOM_QTEEC_IOCTL_REQUEST_CANCELLATION_REQ_32BIT: {
		struct qseecom_qteec_modfd_req_32bit __user *data32;
		struct qseecom_qteec_modfd_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (data == NULL)
			return -EFAULT;

		err = get_qseecom_qteec_modfd_req_32bit(data32, data);
		if (err) {
			kfree(data);
			return err;
		}

		ret = qseecom_ioctl(file, convert_cmd(cmd),
						(unsigned long)data);
		kfree(data);
		return ret;
	}
	break;
	default:
		return -ENOIOCTLCMD;
	break;
	}
	return 0;
}
