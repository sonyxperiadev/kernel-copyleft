// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fdtable.h>
#include <linux/anon_inodes.h>
#include <linux/delay.h>
#include <linux/kref.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/elf.h>

#include "smcinvoke.h"
#include "smcinvoke_object.h"
#include "IClientEnv.h"

#if IS_ENABLED(CONFIG_QSEECOM_COMPAT)
#include "../IQSEEComCompat.h"
#include "../IQSEEComCompatAppLoader.h"
#include "linux/qseecom_api.h"
#if IS_ENABLED(CONFIG_QSEECOM_PROXY)
#include <linux/qseecom_kernel.h>
#else
#include "misc/qseecom_kernel.h"
#endif /* CONFIG_QSEECOM_PROXY */
#endif /* CONFIG_QSEECOM_COMPAT */

/* Device for firmware API :0.*/
extern struct device *smci_dev;

#define MAX_FW_APP_SIZE	256		/* Application name size. */
#define FILE_EXT_SIZE 5			/* File extension like .mbn etc. */

#if IS_ENABLED(CONFIG_QSEECOM_COMPAT)

const uint32_t CQSEEComCompatAppLoader_UID = 122;

struct qseecom_compat_context {
	void *dev; /* in/out */
	unsigned char *sbuf; /* in/out */
	u32 sbuf_len; /* in/out */
	struct qtee_shm shm;
	u8 app_arch;
	struct Object client_env;
	struct Object app_loader;
	struct Object app_controller;
};

char *firmware_request_from_smcinvoke(const char *appname,
	size_t *fw_size, struct qtee_shm *shm);

static int load_app(struct qseecom_compat_context *cxt, const char *app_name)
{
	size_t fw_size = 0;
	u8 *imgbuf_va = NULL;
	int ret = 0;
	char dist_name[MAX_FW_APP_SIZE] = {0};
	size_t dist_name_len = 0;
	struct qtee_shm shm = {0};
	uint32_t arch_type = 0;

	if (strnlen(app_name, MAX_FW_APP_SIZE) == MAX_FW_APP_SIZE) {
		pr_err("The app_name (%s) with length %zu is not valid\n",
			app_name, strnlen(app_name, MAX_FW_APP_SIZE));
		return -EINVAL;
	}

	ret = IQSEEComCompatAppLoader_lookupTA(cxt->app_loader,
		app_name, strlen(app_name), &cxt->app_controller, &arch_type);
	if (!ret) {
		pr_info("app %s exists\n", app_name);
		return ret;
	}

	imgbuf_va = firmware_request_from_smcinvoke(app_name, &fw_size, &shm);
	if (imgbuf_va == NULL) {
		pr_err("Failed on firmware_request_from_smcinvoke\n");
		return -EINVAL;
	}

	ret = IQSEEComCompatAppLoader_loadFromBuffer(
			cxt->app_loader, imgbuf_va, fw_size,
			app_name, strlen(app_name),
			dist_name, MAX_FW_APP_SIZE, &dist_name_len,
			&cxt->app_controller);
	if (ret) {
		pr_err("loadFromBuffer failed for app %s, ret = %d\n",
				app_name, ret);
		goto exit_release_shm;
	}
	cxt->app_arch = *(uint8_t *)(imgbuf_va + EI_CLASS);

	pr_info("%s %d, loaded app %s, dist_name %s, dist_name_len %zu\n",
		__func__, __LINE__, app_name, dist_name, dist_name_len);

exit_release_shm:
	qtee_shmbridge_free_shm(&shm);
	return ret;
}

static int __qseecom_start_app(struct qseecom_handle **handle,
	char *app_name, uint32_t size)
{
	int ret = 0;
	struct qseecom_compat_context *cxt = NULL;

	pr_warn("%s, start app %s, size %u\n",
		__func__, app_name, size);
	if (app_name == NULL || handle == NULL) {
		pr_err("app_name is null or invalid handle\n");
		return -EINVAL;
	}
	/* allocate qseecom_compat_context */
	cxt = kzalloc(sizeof(struct qseecom_compat_context), GFP_KERNEL);
	if (!cxt)
		return -ENOMEM;

	/* get client env */
	ret = get_client_env_object(&cxt->client_env);
	if (ret) {
		pr_err("failed to get clientEnv when loading app %s, ret %d\n",
			app_name, ret);
		ret = -EINVAL;
		goto exit_free_cxt;
	}
	/* get apploader with CQSEEComCompatAppLoader_UID */
	ret = IClientEnv_open(cxt->client_env, CQSEEComCompatAppLoader_UID,
				&cxt->app_loader);
	if (ret) {
		pr_err("failed to get apploader when loading app %s, ret %d\n",
			app_name, ret);
		ret = -EINVAL;
		goto exit_release_clientenv;
	}

	/* load app*/
	ret = load_app(cxt, app_name);
	if (ret) {
		pr_err("failed to load app %s, ret = %d\n",
			app_name, ret);
		ret = -EINVAL;
		goto exit_release_apploader;
	}

	/* Get the physical address of the req/resp buffer */
	ret = qtee_shmbridge_allocate_shm(size, &cxt->shm);

	if (ret) {
		pr_err("qtee_shmbridge_allocate_shm failed, ret :%d\n", ret);
		ret = -EINVAL;
		goto exit_release_appcontroller;
	}
	cxt->sbuf = cxt->shm.vaddr;
	cxt->sbuf_len = size;
	*handle = (struct qseecom_handle *)cxt;

	return ret;

exit_release_appcontroller:
	Object_release(cxt->app_controller);
exit_release_apploader:
	Object_release(cxt->app_loader);
exit_release_clientenv:
	Object_release(cxt->client_env);
exit_free_cxt:
	kfree(cxt);

	return ret;
}

static int __qseecom_shutdown_app(struct qseecom_handle **handle)
{
	struct qseecom_compat_context *cxt = NULL;

	if ((handle == NULL) || (*handle == NULL)) {
		pr_err("Handle is NULL\n");
		return -EINVAL;
	}

	cxt = (struct qseecom_compat_context *)(*handle);

	qtee_shmbridge_free_shm(&cxt->shm);
	Object_release(cxt->app_controller);
	Object_release(cxt->app_loader);
	Object_release(cxt->client_env);
	kfree(cxt);
	*handle = NULL;
	return 0;
}

static int __qseecom_send_command(struct qseecom_handle *handle,
	void *send_buf, uint32_t sbuf_len, void *resp_buf, uint32_t rbuf_len)
{
struct qseecom_compat_context *cxt =
		(struct qseecom_compat_context *)handle;
	size_t out_len = 0;

	pr_debug("%s, sbuf_len %u, rbuf_len %u\n",
		__func__, sbuf_len, rbuf_len);

	if (!handle || !send_buf || !resp_buf || !sbuf_len || !rbuf_len) {
		pr_err("One of params is invalid. %s, handle %p, send_buf %p,resp_buf %p,sbuf_len %u, rbuf_len %u\n",
			 __func__, handle, send_buf, resp_buf, sbuf_len, rbuf_len);
		return -EINVAL;
	}
	return IQSEEComCompat_sendRequest(cxt->app_controller,
				  send_buf, sbuf_len,
				  resp_buf, rbuf_len,
				  send_buf, sbuf_len, &out_len,
				  resp_buf, rbuf_len, &out_len,
				  NULL, 0, /* embedded offset array */
				  (cxt->app_arch == ELFCLASS64),
				  Object_NULL, Object_NULL,
				  Object_NULL, Object_NULL);
}

static int __qseecom_process_listener_from_smcinvoke(uint32_t *result,
		u64 *response_type, unsigned int *data)
{
	/* this API is not supported by compat */
	pr_debug("%s is not supported by compat\n", __func__);
	return -EOPNOTSUPP;
}

#if IS_ENABLED(CONFIG_QSEECOM_PROXY)
static const struct qseecom_drv_ops qseecom_driver_ops = {
	.qseecom_send_command = __qseecom_send_command,
	.qseecom_start_app = __qseecom_start_app,
	.qseecom_shutdown_app = __qseecom_shutdown_app,
	.qseecom_process_listener_from_smcinvoke = __qseecom_process_listener_from_smcinvoke,
};

int get_qseecom_kernel_fun_ops(void)
{
	return provide_qseecom_kernel_fun_ops(&qseecom_driver_ops);
}
#else /* CONFIG_QSEECOM_PROXY */

int qseecom_start_app(struct qseecom_handle **handle, char *app_name, uint32_t size)
{
	return __qseecom_start_app(handle, app_name, size);
}
EXPORT_SYMBOL_GPL(qseecom_start_app);

int qseecom_shutdown_app(struct qseecom_handle **handle)
{
	return __qseecom_shutdown_app(handle);
}
EXPORT_SYMBOL_GPL(qseecom_shutdown_app);

int qseecom_send_command(struct qseecom_handle *handle, void *send_buf,
	uint32_t sbuf_len, void *resp_buf, uint32_t rbuf_len)
{
	return __qseecom_send_command(handle, send_buf, sbuf_len, resp_buf, rbuf_len);
}
EXPORT_SYMBOL_GPL(qseecom_send_command);

int qseecom_process_listener_from_smcinvoke(uint32_t *result,
		u64 *response_type, unsigned int *data)
{
	return __qseecom_process_listener_from_smcinvoke(result,
		response_type, data);
}
EXPORT_SYMBOL_GPL(qseecom_process_listener_from_smcinvoke);

#endif /* CONFIG_QSEECOM_PROXY */
#endif /* CONFIG_QSEECOM_COMPAT */

char *firmware_request_from_smcinvoke(const char *appname, size_t *fw_size, struct qtee_shm *shm)
{
	int rc = 0;
	const struct firmware *fw_entry = NULL, *fw_entry00 = NULL, *fw_entrylast = NULL;
	char fw_name[MAX_FW_APP_SIZE + FILE_EXT_SIZE] = "\0";
	int num_images = 0, phi = 0;
	unsigned char app_arch = 0;
	u8 *img_data_ptr = NULL;
	size_t bufferOffset = 0, phdr_table_offset = 0;
	size_t *offset = NULL;
	Elf32_Phdr phdr32;
	Elf64_Phdr phdr64;
	struct elf32_hdr *ehdr = NULL;
	struct elf64_hdr *ehdr64 = NULL;


	/* load b00*/
	snprintf(fw_name, sizeof(fw_name), "%s.b00", appname);
	rc = firmware_request_nowarn(&fw_entry00, fw_name, smci_dev);
	if (rc) {
		pr_err("Load %s failed, ret:%d\n", fw_name, rc);
		return NULL;
	}

	app_arch = *(unsigned char *)(fw_entry00->data + EI_CLASS);

	/*Get the offsets for split images header*/
	if (app_arch == ELFCLASS32) {

		ehdr = (struct elf32_hdr *)fw_entry00->data;
		num_images = ehdr->e_phnum;
		offset = kcalloc(num_images, sizeof(size_t), GFP_KERNEL);
		if (offset == NULL)
			goto release_fw_entry00;
		phdr_table_offset = (size_t) ehdr->e_phoff;
		for (phi = 1; phi < num_images; ++phi) {
			bufferOffset = phdr_table_offset + phi * sizeof(Elf32_Phdr);
			phdr32 = *(Elf32_Phdr *)(fw_entry00->data + bufferOffset);
			offset[phi] = (size_t)phdr32.p_offset;
		}

	} else if (app_arch == ELFCLASS64) {

		ehdr64 = (struct elf64_hdr *)fw_entry00->data;
		num_images = ehdr64->e_phnum;
		offset = kcalloc(num_images, sizeof(size_t), GFP_KERNEL);
		if (offset == NULL)
			goto release_fw_entry00;
		phdr_table_offset = (size_t) ehdr64->e_phoff;
		for (phi = 1; phi < num_images; ++phi) {
			bufferOffset = phdr_table_offset + phi * sizeof(Elf64_Phdr);
			phdr64 = *(Elf64_Phdr *)(fw_entry00->data + bufferOffset);
			offset[phi] = (size_t)phdr64.p_offset;
		}

	} else {

		pr_err("QSEE %s app, arch %u is not supported\n", appname, app_arch);
		goto release_fw_entry00;
	}

	/*Find the size of last split bin image*/
	snprintf(fw_name, ARRAY_SIZE(fw_name), "%s.b%02d", appname, num_images-1);
	rc = firmware_request_nowarn(&fw_entrylast, fw_name, smci_dev);
	if (rc) {
		pr_err("Failed to locate blob %s\n", fw_name);
		goto release_fw_entry00;
	}

	/*Total size of image will be the offset of last image + the size of last split image*/
	*fw_size = fw_entrylast->size + offset[num_images-1];

	/*Allocate memory for the buffer that will hold the split image*/
	rc = qtee_shmbridge_allocate_shm((*fw_size), shm);
	if (rc) {
		pr_err("smbridge alloc failed for size: %zu\n", *fw_size);
		goto release_fw_entrylast;
	}
	img_data_ptr = shm->vaddr;
	/*
	 * Copy contents of split bins to the buffer
	 */
	memcpy(img_data_ptr, fw_entry00->data, fw_entry00->size);
	for (phi = 1; phi < num_images-1; phi++) {
		snprintf(fw_name, ARRAY_SIZE(fw_name), "%s.b%02d", appname, phi);
		rc = firmware_request_nowarn(&fw_entry, fw_name, smci_dev);
		if (rc) {
			pr_err("Failed to locate blob %s\n", fw_name);
			qtee_shmbridge_free_shm(shm);
			img_data_ptr = NULL;
			goto release_fw_entrylast;
		}
		memcpy(img_data_ptr + offset[phi], fw_entry->data, fw_entry->size);
		release_firmware(fw_entry);
		fw_entry = NULL;
	}
	memcpy(img_data_ptr + offset[phi], fw_entrylast->data, fw_entrylast->size);

release_fw_entrylast:
	release_firmware(fw_entrylast);
release_fw_entry00:
	release_firmware(fw_entry00);
	kfree(offset);
	return img_data_ptr;
}
EXPORT_SYMBOL_GPL(firmware_request_from_smcinvoke);
