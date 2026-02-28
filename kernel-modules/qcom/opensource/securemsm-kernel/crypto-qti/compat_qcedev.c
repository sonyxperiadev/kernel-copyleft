// SPDX-License-Identifier: GPL-2.0-only
/*
 * QTI CE 32-bit compatibility syscall for 64-bit systems
 *
 * Copyright (c) 2014-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include "linux/qcedev.h"
#include <linux/compat.h>
#include "compat_qcedev.h"
#include <linux/slab.h>

static int compat_get_qcedev_pmem_info(
		struct compat_qcedev_pmem_info __user *pmem32,
		struct qcedev_pmem_info *pmem)
{
	__u32 offset;
	__s32 fd_src;
	__s32 fd_dst;
	int err, i;
	uint32_t len;

	err = get_user(fd_src, &pmem32->fd_src);
	memcpy(&pmem->fd_src, &fd_src, sizeof(fd_src));

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		err |= get_user(offset, &pmem32->src[i].offset);
		memcpy(&pmem->src[i].offset, &offset, sizeof(offset));
		err |= get_user(len, &pmem32->src[i].len);
		memcpy(&pmem->src[i].len, &len, sizeof(len));
	}

	err |= get_user(fd_dst, &pmem32->fd_dst);
	memcpy(&pmem->fd_dst, &fd_dst, sizeof(fd_dst));

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		err |= get_user(offset, &pmem32->dst[i].offset);
		memcpy(&pmem->dst[i].offset, &offset, sizeof(offset));
		err |= get_user(len, &pmem32->dst[i].len);
		memcpy(&pmem->dst[i].len, &len, sizeof(len));
	}

	return err;
}

static int compat_put_qcedev_pmem_info(
		struct compat_qcedev_pmem_info __user *pmem32,
		struct qcedev_pmem_info *pmem)
{
	__u32 offset = 0;
	__s32 fd_src = 0;
	__s32 fd_dst;
	int err = 0, i;
	uint32_t len;

	memcpy(&fd_src, &pmem->fd_src, sizeof(fd_src));
	err |= put_user(fd_src, &pmem32->fd_src);

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		memcpy(&offset, &pmem->src[i].offset, sizeof(offset));
		err |= put_user(offset, &pmem32->src[i].offset);
		memcpy(&len, &pmem->src[i].len, sizeof(len));
		err |= put_user(len, &pmem32->src[i].len);
	}

	memcpy(&fd_dst, &pmem->fd_dst, sizeof(fd_dst));
	err |= put_user(fd_dst, &pmem32->fd_dst);

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		memcpy(&offset, &pmem->dst[i].offset, sizeof(offset));
		err |= put_user(offset, &pmem32->dst[i].offset);
		memcpy(&len, &pmem->dst[i].len, sizeof(len));
		err |= put_user(len, &pmem32->dst[i].len);
	}

	return err;
}

static int compat_get_qcedev_vbuf_info(
		struct compat_qcedev_vbuf_info __user *vbuf32,
		struct qcedev_vbuf_info *vbuf)
{
	__u32 vaddr;
	int err = 0, i;
	uint32_t len;

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		err |= get_user(vaddr, &vbuf32->src[i].vaddr);
		memcpy(&vbuf->src[i].vaddr, &vaddr, sizeof(vaddr));
		err |= get_user(len, &vbuf32->src[i].len);
		memcpy(&vbuf->src[i].len, &len, sizeof(len));
	}

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		err |= get_user(vaddr, &vbuf32->dst[i].vaddr);
		memcpy(&vbuf->dst[i].vaddr, &vaddr, sizeof(vaddr));
		err |= get_user(len, &vbuf32->dst[i].len);
		memcpy(&vbuf->dst[i].len, &len, sizeof(len));
	}
	return err;
}

static int compat_put_qcedev_vbuf_info(
		struct compat_qcedev_vbuf_info __user *vbuf32,
		struct qcedev_vbuf_info *vbuf)
{
	__u32 vaddr = 0;
	int err = 0, i;
	uint32_t len = 0;

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		memcpy(&vaddr, &vbuf->src[i].vaddr, sizeof(vaddr));
		err |= put_user(vaddr, &vbuf32->src[i].vaddr);
		memcpy(&len, &vbuf->src[i].len, sizeof(len));
		err |= put_user(len, &vbuf32->src[i].len);
	}

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		memcpy(&vaddr, &vbuf->dst[i].vaddr, sizeof(vaddr));
		err |= put_user(vaddr, &vbuf32->dst[i].vaddr);
		memcpy(&len, &vbuf->dst[i].len, sizeof(len));
		err |= put_user(len, &vbuf32->dst[i].len);
	}
	return err;
}

static int compat_get_qcedev_cipher_op_req(
		struct compat_qcedev_cipher_op_req __user *data32,
		struct qcedev_cipher_op_req *data)
{
	enum qcedev_cipher_mode_enum mode;
	enum qcedev_cipher_alg_enum alg;
	__u32 byteoffset;
	enum qcedev_oper_enum op;
	__u32 data_len;
	__u32 encklen;
	__u32 entries;
	__u32 ivlen;
	uint8_t in_place_op;
	int err, i;
	uint8_t use_pmem;
	uint8_t enckey;
	uint8_t iv;

	err = get_user(use_pmem, &data32->use_pmem);
	memcpy(&data->use_pmem, &use_pmem, sizeof(use_pmem));

	if (use_pmem)
		err |= compat_get_qcedev_pmem_info(&data32->pmem, &data->pmem);
	else
		err |= compat_get_qcedev_vbuf_info(&data32->vbuf, &data->vbuf);

	err |= get_user(entries, &data32->entries);
	memcpy(&data->entries, &entries, sizeof(entries));
	err |= get_user(data_len, &data32->data_len);
	memcpy(&data->data_len, &data_len, sizeof(data_len));
	err |= get_user(in_place_op, &data32->in_place_op);
	memcpy(&data->in_place_op, &in_place_op, sizeof(in_place_op));

	for (i = 0; i < QCEDEV_MAX_KEY_SIZE; i++) {
		err |= get_user(enckey, &(data32->enckey[i]));
		memcpy(&data->enckey[i], &enckey, sizeof(enckey));
	}

	err |= get_user(encklen, &data32->encklen);
	memcpy(&data->encklen, &encklen, sizeof(encklen));

	for (i = 0; i < QCEDEV_MAX_IV_SIZE; i++) {
		err |= get_user(iv, &(data32->iv[i]));
		memcpy(&data->iv[i], &iv, sizeof(iv));
	}

	err |= get_user(ivlen, &data32->ivlen);
	memcpy(&data->ivlen, &ivlen, sizeof(ivlen));
	err |= get_user(byteoffset, &data32->byteoffset);
	memcpy(&data->byteoffset, &byteoffset, sizeof(byteoffset));
	err |= get_user(alg, &data32->alg);
	memcpy(&data->alg, &alg, sizeof(alg));
	err |= get_user(mode, &data32->mode);
	memcpy(&data->mode, &mode, sizeof(mode));
	err |= get_user(op, &data32->op);
	memcpy(&data->op, &op, sizeof(op));

	return err;
}

static int compat_put_qcedev_cipher_op_req(
		struct compat_qcedev_cipher_op_req __user *data32,
		struct qcedev_cipher_op_req *data)
{
	enum qcedev_cipher_mode_enum mode;
	enum qcedev_cipher_alg_enum alg;
	__u32 byteoffset;
	enum qcedev_oper_enum op;
	__u32 data_len;
	__u32 encklen;
	__u32 entries;
	__u32 ivlen;
	__u8 in_place_op;
	int err = 0, i;
	__u8 use_pmem = 0;
	__u8 enckey;
	__u8 iv;

	memcpy(&use_pmem, &data->use_pmem, sizeof(use_pmem));
	err |= put_user(use_pmem, &data32->use_pmem);

	if (use_pmem)
		err |= compat_put_qcedev_pmem_info(&data32->pmem, &data->pmem);
	else
		err |= compat_put_qcedev_vbuf_info(&data32->vbuf, &data->vbuf);

	memcpy(&entries, &data->entries, sizeof(entries));
	err |= put_user(entries, &data32->entries);
	memcpy(&data_len, &data->data_len, sizeof(data_len));
	err |= put_user(data_len, &data32->data_len);
	memcpy(&in_place_op, &data->in_place_op, sizeof(in_place_op));
	err |= put_user(in_place_op, &data32->in_place_op);

	for (i = 0; i < QCEDEV_MAX_KEY_SIZE; i++) {
		memcpy(&enckey, &data->enckey[i], sizeof(enckey));
		err |= put_user(enckey, &(data32->enckey[i]));
	}

	memcpy(&encklen, &data->encklen, sizeof(encklen));
	err |= put_user(encklen, &data32->encklen);

	for (i = 0; i < QCEDEV_MAX_IV_SIZE; i++) {
		memcpy(&iv, &(data->iv[i]), sizeof(iv));
		err |= put_user(iv, &(data32->iv[i]));
	}

	memcpy(&ivlen, &data->ivlen, sizeof(ivlen));
	err |= put_user(ivlen, &data32->ivlen);
	memcpy(&byteoffset, &data->byteoffset, sizeof(byteoffset));
	err |= put_user(byteoffset, &data32->byteoffset);
	memcpy(&alg, &data->alg, sizeof(alg));
	err |= put_user(alg, &data32->alg);
	memcpy(&mode, &data->mode, sizeof(mode));
	err |= put_user(mode, &data32->mode);
	memcpy(&op, &data->op, sizeof(op));
	err |= put_user(op, &data32->op);

	return err;
}

static int compat_xfer_qcedev_map_buf_req(
			struct compat_qcedev_map_buf_req __user *data32,
			struct qcedev_map_buf_req *data, bool to_get)
{
	int rc = 0, i;
	__s32 fd = -1;
	__u32 num_fds;
	__u32 fd_size = 0;
	__u32 fd_offset;
	__u64 buf_vaddr;

	if (to_get) {
		/* copy from compat struct */
		for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
			rc |= get_user(fd, &data32->fd[i]);
			memcpy(&data->fd[i], &fd, sizeof(fd));
			rc |= get_user(fd_size, &data32->fd_size[i]);
			memcpy(&data->fd_size[i], &fd_size, sizeof(fd_size));
			rc |= get_user(fd_offset, &data32->fd_offset[i]);
			memcpy(&data->fd_offset[i], &fd_offset, sizeof(fd_offset));
			rc |= get_user(buf_vaddr, &data32->buf_vaddr[i]);
			memcpy(&data->buf_vaddr[i], &buf_vaddr, sizeof(buf_vaddr));
		}

		rc |= get_user(num_fds, &data32->num_fds);
		memcpy(&data->num_fds, &num_fds, sizeof(num_fds));
	} else {
		/* copy to compat struct */
		for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
			memcpy(&fd, &data->fd[i], sizeof(fd));
			rc |= put_user(fd, &data32->fd[i]);
			memcpy(&fd_size, &data->fd_size[i], sizeof(fd_size));
			rc |= put_user(fd_size, &data32->fd_size[i]);
			memcpy(&fd_offset, &data->fd_offset[i], sizeof(fd_offset));
			rc |= put_user(fd_offset, &data32->fd_offset[i]);
			memcpy(&buf_vaddr, &data->buf_vaddr[i], sizeof(buf_vaddr));
			rc |= put_user(buf_vaddr, &data32->buf_vaddr[i]);
		}
		memcpy(&num_fds, &data->num_fds, sizeof(num_fds));
		rc |= put_user(num_fds, &data32->num_fds);
	}

	return rc;
}

static int compat_xfer_qcedev_unmap_buf_req(
			struct compat_qcedev_unmap_buf_req __user *data32,
			struct qcedev_unmap_buf_req *data, bool to_get)
{
	int i, rc = 0;
	__s32 fd = -1;
	__u32 num_fds;

	if (to_get) {
		/* copy from compat struct */
		for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
			rc |= get_user(fd, &data32->fd[i]);
			memcpy(&data->fd[i], &fd, sizeof(fd));
		}
		rc |= get_user(num_fds, &data32->num_fds);
		memcpy(&data->num_fds, &num_fds, sizeof(num_fds));
	} else {
		/* copy to compat struct */
		for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
			memcpy(&fd, &data->fd[i], sizeof(fd));
			rc |= put_user(fd, &data32->fd[i]);
		}
		memcpy(&num_fds, &data->num_fds, sizeof(num_fds));
		rc |= put_user(num_fds, &data32->num_fds);
	}
	return rc;
}


static int compat_get_qcedev_sha_op_req(
		struct compat_qcedev_sha_op_req __user *data32,
		struct qcedev_sha_op_req *data)
{
	enum qcedev_sha_alg_enum alg;
	__u32 authklen;
	__u32 data_len;
	__u32 entries;
	__u32 diglen;
	__u32 authkey;
	__u32 vaddr;
	int err = 0, i;
	uint8_t digest;
	uint32_t len;

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		err |= get_user(vaddr, &data32->data[i].vaddr);
		memcpy(&data->data[i].vaddr, &vaddr, sizeof(vaddr));
		err |= get_user(len, &data32->data[i].len);
		memcpy(&data->data[i].len, &len, sizeof(len));
	}

	err |= get_user(entries, &data32->entries);
	memcpy(&data->entries, &entries, sizeof(entries));
	err |= get_user(data_len, &data32->data_len);
	memcpy(&data->data_len, &data_len, sizeof(data_len));

	for (i = 0; i < QCEDEV_MAX_SHA_DIGEST; i++) {
		err |= get_user(digest, &(data32->digest[i]));
		memcpy(&data->digest[i], &digest, sizeof(digest));
	}

	err |= get_user(diglen, &data32->diglen);
	memcpy(&data->diglen, &diglen, sizeof(diglen));
	err |= get_user(authkey, &data32->authkey);
	memcpy(&data->authkey, &authkey, sizeof(authkey));
	err |= get_user(authklen, &data32->authklen);
	memcpy(&data->authklen, &authklen, sizeof(authklen));
	err |= get_user(alg, &data32->alg);
	memcpy(&data->alg, &alg, sizeof(alg));

	return err;
}

static int compat_put_qcedev_sha_op_req(
		struct compat_qcedev_sha_op_req __user *data32,
		struct qcedev_sha_op_req *data)
{
	enum qcedev_sha_alg_enum alg;
	__u32 authklen;
	__u32 data_len;
	__u32 entries;
	__u32 diglen;
	__u32 authkey;
	__u32 vaddr = 0;
	int err = 0, i;
	uint8_t digest;
	uint32_t len = 0;

	for (i = 0; i < QCEDEV_MAX_BUFFERS; i++) {
		memcpy(&vaddr, &(data->data[i].vaddr), sizeof(vaddr));
		err |= put_user(vaddr, &data32->data[i].vaddr);
		memcpy(&len, &(data->data[i].len), sizeof(len));
		err |= put_user(len, &data32->data[i].len);
	}

	memcpy(&entries, &data->entries, sizeof(entries));
	err |= put_user(entries, &data32->entries);
	memcpy(&data_len, &data->data_len, sizeof(data_len));
	err |= put_user(data_len, &data32->data_len);

	for (i = 0; i < QCEDEV_MAX_SHA_DIGEST; i++) {
		memcpy(&digest, &(data->digest[i]), sizeof(digest));
		err |= put_user(digest, &(data32->digest[i]));
	}

	memcpy(&diglen, &data->diglen, sizeof(diglen));
	err |= put_user(diglen, &data32->diglen);
	memcpy(&authkey, &data->authkey, sizeof(authkey));
	err |= put_user(authkey, &data32->authkey);
	memcpy(&authklen, &data->authklen, sizeof(authklen));
	err |= put_user(authklen, &data32->authklen);
	memcpy(&alg, &data->alg, sizeof(alg));
	err |= put_user(alg, &data32->alg);

	return err;
}

static unsigned int convert_cmd(unsigned int cmd)
{
	switch (cmd) {
	case COMPAT_QCEDEV_IOCTL_ENC_REQ:
		return QCEDEV_IOCTL_ENC_REQ;
	case COMPAT_QCEDEV_IOCTL_DEC_REQ:
		return QCEDEV_IOCTL_DEC_REQ;
	case COMPAT_QCEDEV_IOCTL_SHA_INIT_REQ:
		return QCEDEV_IOCTL_SHA_INIT_REQ;
	case COMPAT_QCEDEV_IOCTL_SHA_UPDATE_REQ:
		return QCEDEV_IOCTL_SHA_UPDATE_REQ;
	case COMPAT_QCEDEV_IOCTL_SHA_FINAL_REQ:
		return QCEDEV_IOCTL_SHA_FINAL_REQ;
	case COMPAT_QCEDEV_IOCTL_GET_SHA_REQ:
		return QCEDEV_IOCTL_GET_SHA_REQ;
	case COMPAT_QCEDEV_IOCTL_GET_CMAC_REQ:
		return QCEDEV_IOCTL_GET_CMAC_REQ;
	case COMPAT_QCEDEV_IOCTL_MAP_BUF_REQ:
		return QCEDEV_IOCTL_MAP_BUF_REQ;
	case COMPAT_QCEDEV_IOCTL_UNMAP_BUF_REQ:
		return QCEDEV_IOCTL_UNMAP_BUF_REQ;
	default:
		return cmd;
	}

}

long compat_qcedev_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	long ret;

	switch (cmd) {
	case COMPAT_QCEDEV_IOCTL_ENC_REQ:
	case COMPAT_QCEDEV_IOCTL_DEC_REQ: {
		struct compat_qcedev_cipher_op_req __user *data32;
		struct qcedev_cipher_op_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (!data)
			return -EFAULT;

		err = compat_get_qcedev_cipher_op_req(data32, data);
		if (err) {
			kfree(data);
			return err;
		}
		ret = qcedev_ioctl(file, convert_cmd(cmd), (unsigned long)data);
		err = compat_put_qcedev_cipher_op_req(data32, data);
		kfree(data);
		return ret ? ret : err;
	}
	case COMPAT_QCEDEV_IOCTL_SHA_INIT_REQ:
	case COMPAT_QCEDEV_IOCTL_SHA_UPDATE_REQ:
	case COMPAT_QCEDEV_IOCTL_SHA_FINAL_REQ:
	case COMPAT_QCEDEV_IOCTL_GET_CMAC_REQ:
	case COMPAT_QCEDEV_IOCTL_GET_SHA_REQ: {
		struct compat_qcedev_sha_op_req __user *data32;
		struct qcedev_sha_op_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (!data)
			return -EFAULT;

		err = compat_get_qcedev_sha_op_req(data32, data);
		if (err) {
			kfree(data);
			return err;
		}
		ret = qcedev_ioctl(file, convert_cmd(cmd), (unsigned long)data);
		err = compat_put_qcedev_sha_op_req(data32, data);
		kfree(data);
		return ret ? ret : err;
	}
	case COMPAT_QCEDEV_IOCTL_MAP_BUF_REQ: {
		struct compat_qcedev_map_buf_req __user *data32;
		struct qcedev_map_buf_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (!data)
			return -EINVAL;

		err = compat_xfer_qcedev_map_buf_req(data32, data, true);
		if (err) {
			kfree(data);
			return err;
		}
		ret = qcedev_ioctl(file, convert_cmd(cmd), (unsigned long)data);
		err = compat_xfer_qcedev_map_buf_req(data32, data, false);
		kfree(data);
		return ret ? ret : err;
	}
	case COMPAT_QCEDEV_IOCTL_UNMAP_BUF_REQ: {
		struct compat_qcedev_unmap_buf_req __user *data32;
		struct qcedev_unmap_buf_req *data;
		int err;

		data32 = compat_ptr(arg);
		data = kmalloc(sizeof(*data), GFP_KERNEL);
		if (!data)
			return -EINVAL;

		err = compat_xfer_qcedev_unmap_buf_req(data32, data, true);
		if (err) {
			kfree(data);
			return err;
		}
		ret = qcedev_ioctl(file, convert_cmd(cmd), (unsigned long)data);
		err = compat_xfer_qcedev_unmap_buf_req(data32, data, false);
		kfree(data);
		return ret ? ret : err;
	}
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}
