// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/cdev.h>
#include <linux/version.h>
#include "qti-smmu-proxy-common.h"
#include "smcinvoke_object.h"
#include "../include/linux/ITrustedCameraDriver.h"
#include "../include/linux/CTrustedCameraDriver.h"
#include "../include/linux/IClientEnv.h"

#define SMMU_PROXY_MAX_DEVS 1
static dev_t smmu_proxy_dev_no;
static struct class *smmu_proxy_class;
static struct cdev smmu_proxy_char_dev;

static struct csf_version cached_csf_version;

int smmu_proxy_get_csf_version(struct csf_version *csf_version)
{
	int ret;
	struct Object client_env = {0};
	struct Object sc_object;

	/* Assumption is that cached_csf_version.arch_ver !=0 ==> other vals are set */
	if (cached_csf_version.arch_ver != 0) {
		csf_version->arch_ver = cached_csf_version.arch_ver;
		csf_version->max_ver = cached_csf_version.max_ver;
		csf_version->min_ver = cached_csf_version.min_ver;

		return 0;
	}

	ret = get_client_env_object(&client_env);
	if (ret) {
		pr_err("%s: Failed to get env object rc: %d\n", __func__,
		       ret);
		return ret;
	}

	ret = IClientEnv_open(client_env, CTrustedCameraDriver_UID, &sc_object);
	if (ret) {
		pr_err("%s: Failed to get seccam object rc: %d\n", __func__,
		       ret);
		return ret;
	}

	ret = ITrustedCameraDriver_getVersion(sc_object, &csf_version->arch_ver,
					      &csf_version->max_ver,
					      &csf_version->min_ver);

	Object_release(sc_object);
	Object_release(client_env);

	/*
	 * Once we set cached_csf_version.arch_ver, concurrent callers will get
	 * the cached value.
	 */
	cached_csf_version.min_ver = csf_version->min_ver;
	cached_csf_version.max_ver = csf_version->max_ver;
	cached_csf_version.arch_ver = csf_version->arch_ver;

	return ret;
}
EXPORT_SYMBOL(smmu_proxy_get_csf_version);

int smmu_proxy_create_dev(const struct file_operations *fops)
{
	int ret;
	struct device *class_dev;

	ret = alloc_chrdev_region(&smmu_proxy_dev_no, 0, SMMU_PROXY_MAX_DEVS,
				  "qti-smmu-proxy");
	if (ret < 0)
		return ret;

#if (KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE)
	smmu_proxy_class = class_create("qti-smmu-proxy");
#else
	smmu_proxy_class = class_create(THIS_MODULE, "qti-smmu-proxy");
#endif
	if (IS_ERR(smmu_proxy_class)) {
		ret = PTR_ERR(smmu_proxy_class);
		goto err_class_create;
	}

	cdev_init(&smmu_proxy_char_dev, fops);
	ret = cdev_add(&smmu_proxy_char_dev, smmu_proxy_dev_no,
		       SMMU_PROXY_MAX_DEVS);
	if (ret < 0)
		goto err_cdev_add;

	class_dev = device_create(smmu_proxy_class, NULL, smmu_proxy_dev_no, NULL,
				  "qti-smmu-proxy");
	if (IS_ERR(class_dev)) {
		ret = PTR_ERR(class_dev);
		goto err_dev_create;
	}

	return 0;

err_dev_create:
	cdev_del(&smmu_proxy_char_dev);
err_cdev_add:
	class_destroy(smmu_proxy_class);
err_class_create:
	unregister_chrdev_region(smmu_proxy_dev_no, SMMU_PROXY_MAX_DEVS);

	return ret;
}

