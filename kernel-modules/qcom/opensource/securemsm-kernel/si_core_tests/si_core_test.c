// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "si-core-tests: %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/qtee_shmbridge.h>
#if IS_ENABLED(CONFIG_QCOM_SI_CORE)
#include <linux/firmware/qcom/si_object.h>
#include <linux/firmware.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/types.h>
#include <linux/uaccess.h>

// Function prototypes
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static long device_ioctl(struct file *, unsigned int, unsigned long);
static ssize_t device_write(struct file *file, const char __user *buf,
							size_t count, loff_t *offset);

// Define IOCTL commands
#define IOCTL_RUN_TEST _IOW(1, 0, int)

#define SI_CORE_TEST_TIMED

static struct class *driver_class;
static dev_t si_core_device_no;
static struct cdev si_core_cdev;
struct device *si_core_dev;

// File operations structure
static const struct file_operations fops = {
	.open = device_open,
	.release = device_release,
	.unlocked_ioctl = device_ioctl,
	.write = device_write
};

#ifdef SI_CORE_TEST_TIMED
#define si_object_do_invoke_timed(oic, o, op, args, result, msg) ({             \
    int ret;                                                                    \
    ktime_t start_time, end_time, diff;                                         \
    start_time = ktime_get();                                                   \
    ret = si_object_do_invoke((oic), (o), (op), (args), (result));              \
    end_time = ktime_get();                                                     \
    if (!ret) {                                                                 \
        diff = ktime_to_ns(ktime_sub(end_time, start_time));                    \
        pr_info("%s took(ns): %llu", msg, diff);                                \
    }                                                                           \
    ret;                                                                        \
})
#else
#define si_object_do_invoke_timed(oic, o, op, args, result, msg) \
     si_object_do_invoke((oic), (o), (op), (args), (result))
#endif

// Device open function
static int device_open(struct inode *inode, struct file *file)
{
	pr_info("Device opened\n");
	return 0;
}

// Device release function
static int device_release(struct inode *inode, struct file *file)
{
	pr_info("Device closed\n");
	return 0;
}

enum si_core_kernel_test_case {
	SI_CORE_KERNEL_TEST_GET_SERVICE = 1,
	SI_CORE_KERNEL_TEST_LOAD_APP,
	SI_CORE_KERNEL_TEST_END,
};

struct heap_info {
	u32 total_size;
	u32 used_size;
	u32 free_size;
	u32 overhead_size;
	u32 wasted_size;
	u32 largest_free_block_size;
};

static struct si_object_invoke_ctx oic;

int get_client_env(struct si_object_invoke_ctx *oic, struct si_object **client_env)
{
	int ret, result;
	struct si_arg args[3] = { 0 };

	args[0].o = NULL_SI_OBJECT;
	args[0].type = SI_AT_IO;
	args[1].type = SI_AT_OO;
	args[2].type = SI_AT_END;

	/* IClientEnv_OP_registerWithCredentials  is 5. */
	ret = si_object_do_invoke_timed(oic, ROOT_SI_OBJECT, 5, args, &result,
				"IClientEnv_OP_registerWithCredentials");
	if (ret || result) {
		pr_err("failed with result %d(ret = %d).\n", result, ret);
		return -EINVAL;
	}

	if (args[1].o != NULL_SI_OBJECT)
		*client_env = args[1].o;

	return 0;
}

int client_env_open(struct si_object_invoke_ctx *oic, struct si_object *client_env,
	u32 uid_val, struct si_object **service)
{
	int ret, result;
	struct si_arg args[3] = { 0 };

	args[0].b = (struct si_buffer) { {&uid_val}, sizeof(uid_val) };
	args[0].type = SI_AT_IB;
	args[1].type = SI_AT_OO;
	args[2].type = SI_AT_END;

	/* IClientEnv_OP_open is 0. */
	ret = si_object_do_invoke_timed(oic, client_env, 0, args, &result,
				"IClientEnv_OP_open");
	if (ret || result) {
		pr_err("failed with result %d(ret = %d).\n", result, ret);
		return -EINVAL;
	}

	if (args[1].o != NULL_SI_OBJECT)
		*service = args[1].o;

	return 0;
}

static int query_heap_info(struct si_object_invoke_ctx *oic, struct si_object *service,
							struct heap_info *heap_info)
{
	int ret, result;
	struct si_arg args[2] = { 0 };

	args[0].b = (struct si_buffer) { {heap_info}, sizeof(*heap_info) };
	args[0].type = SI_AT_OB;
	args[1].type = SI_AT_END;

	/* IDiagnostics_OP_queryHeapInfo is 0. */
	ret = si_object_do_invoke_timed(oic, service, 0, args, &result,
				"IDiagnostics_OP_queryHeapInfo");
	if (ret || result) {
		pr_err("failed with result %d(ret = %d).\n", result, ret);
		return -EINVAL;
	}

	return 0;
}

static int load_app(struct si_object *appLoader, void *file, int len,
						struct si_object **appController,
						struct si_object **appObj,
						struct si_object_invoke_ctx *oic)
{
	int ret = -1;
	int result;
	void *buffer;
	ssize_t length;

	buffer = file;
	length = (ssize_t)len;

	struct si_arg args[3] = { 0 };

	args[0].type = SI_AT_IB;
	args[0].b = (struct si_buffer) { {buffer}, length };
	args[1].type = SI_AT_OO;
	args[2].type = SI_AT_END;

	/* IAppLoader_OP_loadFromBuffer is 0. */
	ret = si_object_do_invoke_timed(oic, appLoader, 0, args, &result,
				"IAppLoader_OP_loadFromBuffer");
	if (ret || result) {
		pr_err("failed IAppLoader_OP_loadFromBuffer with result %d(ret = %d).\n",
				result, ret);
		kfree(buffer);
		return -EINVAL;
	}
	*appController = args[1].o;

	struct si_arg app_ctr_args[2] = { 0 };

	app_ctr_args[0].type = SI_AT_OO;
	app_ctr_args[1].type = SI_AT_END;

	/* IAppController_OP_getAppObject is 2. */
	ret = si_object_do_invoke_timed(oic, *appController, 2, app_ctr_args, &result,
				"IAppController_OP_getAppObject");
	if (ret || result) {
		put_si_object(*appController);
		pr_err("failed appController with result %d(ret = %d).\n", result, ret);
		kfree(buffer);
		return -EINVAL;
	}
	*appObj = app_ctr_args[0].o;

	kfree(buffer);
	return ret;
}

static const char *stringFromHash(const uint8_t *inHash, size_t inHashLen)
{
	static char outString[(2 * 32) + 1];
	size_t index = 0;

	for (size_t i = 0; i < inHashLen; i++)
		index += scnprintf(outString + index, sizeof(outString) - index, "%02X", inHash[i]);

	return outString;
}

static int send_command(struct si_object *appObj, struct si_object_invoke_ctx *oic)
{
	int ret, result;
	struct si_arg args[4] = { 0 };
	char stringToHash[] = "String to hash";
	const char *printHashString;
	int hash_sha = 1;
	void *idPtr = &hash_sha;
	uint8_t digest[32] = {0};

	args[0].type = SI_AT_IB;
	args[0].b = (struct si_buffer) { {stringToHash}, sizeof(stringToHash)-1 };
	args[1].type = SI_AT_IB;
	args[1].b = (struct si_buffer) { {idPtr}, sizeof(hash_sha) };
	args[2].type = SI_AT_OB;
	args[2].b = (struct si_buffer) { {digest}, sizeof(digest) };
	args[3].type = SI_AT_END;

	// ISMCIExampleApp_computeHash
	ret = si_object_do_invoke_timed(oic, appObj, 1, args, &result,
				"ISMCIExampleApp_computeHash");
	if (ret || result) {
		pr_err("failed ISMCIExampleApp_computeHash with result %d(ret = %d).\n",
				result, ret);
		return -EINVAL;
	}

	printHashString = stringFromHash(digest, sizeof(digest));
	pr_info("Hash String: %s\n", printHashString);
	return result;
}

int si_core_get_service_test(struct si_object_invoke_ctx *oic)
{
	int ret;
	struct si_object *client_env, *service;
	struct heap_info heap_info;

	ret = get_client_env(oic, &client_env);
	if (ret) {
		pr_err("get_client_env failed (%d).\n", ret);
		return ret;
	}

	/* CDiagnostics_UID is 143. */
	ret = client_env_open(oic, client_env, 143, &service);
	if (ret) {
		pr_err("client_env_open failed (%d).\n", ret);
		put_si_object(client_env);
		return ret;
	}

	ret = query_heap_info(oic, service, &heap_info);
	if (ret) {
		pr_err("query_heap_info failed (%d).\n", ret);
		goto out;
	}

	pr_info("TEST SUCCESS.\n");

out:
	put_si_object(service);
	put_si_object(client_env);
	return ret;
}

int si_core_kernel_test_load_app(struct si_object_invoke_ctx *oic, void *file, int len)
{
	int ret;

	struct si_object *client_env, *appLoader, *appController, *appObj;

	ret = get_client_env(oic, &client_env);
	if (ret) {
		kfree(file);
		pr_err("get_client_env failed (%d).\n", ret);
		return ret;
	}

	/* CAppLoader_UID is 3. */
	ret = client_env_open(oic, client_env, 3, &appLoader);
	if (ret) {
		kfree(file);
		pr_err("client_env_open failed (%d).\n", ret);
		goto out_client;
	}

	ret = load_app(appLoader, file, len, &appController, &appObj, oic);
	if (ret) {
		pr_err("App loading failed (%d).\n", ret);
		goto out;
	}

	ret = send_command(appObj, oic);
	if (!ret) {
		pr_info("LOADING APP TEST SUCCESS.\n");
		put_si_object(appObj);
	} else
		pr_err("Sending command failed (%d).\n", ret);

	put_si_object(appController);

out:
	put_si_object(appLoader);
out_client:
	put_si_object(client_env);
	return ret;
}

struct ioctl_arguments {
	u32 test_num;
	u64 file;
	u32 len;
};

static ssize_t device_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	size_t maxdatalen = 1024*64;

	char *kernel_buffer;

	const struct firmware *fw_entry;
	char fw_name[300] = "\0";

	const char *appname = "smcinvoke_example_ta64";
	int rc = 0;

	snprintf(fw_name, sizeof(fw_name), "%s.mbn", appname);
	rc = firmware_request_nowarn(&fw_entry, fw_name, si_core_dev);
	if (rc) {
		pr_err("Load %s failed, ret:%d\n", fw_name, rc);
		return rc;
	}

	pr_info("Running test case 1: Direct Path\n");
	if (si_core_get_service_test(&oic) != 0)
		pr_err("SI_CORE_KERNEL_TEST_GET_SERVICE failed.\n");
	else
		pr_info("SI_CORE_KERNEL_TEST_GET_SERVICE succeed.\n");
	pr_info("Running test case 2: Loading TA/Sending command\n");

	kernel_buffer = kmalloc(maxdatalen, GFP_KERNEL);
	memcpy(kernel_buffer, fw_entry->data, fw_entry->size);

	if (si_core_kernel_test_load_app(&oic, kernel_buffer, fw_entry->size) != 0)
		pr_err("SI_CORE_KERNEL_TEST_LOAD_APP failed.\n");
	else
		pr_info("SI_CORE_KERNEL_TEST_LOAD_APP succeed.\n");

	return count;
}

// IOCTL function
static long device_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int test_case;
	struct ioctl_arguments msg;
	void __user *user_addr;
	char *kernel_buffer;

	switch (cmd) {
	case IOCTL_RUN_TEST:
		// Run the specified test case
		if (copy_from_user(&msg, (struct ioctl_arguments __user *)arg,
				sizeof(struct ioctl_arguments))) {
			pr_err("copy_from_user failed\n");
			return -EACCES;
		}
		test_case = (int)(msg.test_num);
		pr_info("test case selected: %d\n", test_case);
		switch (test_case) {
		case SI_CORE_KERNEL_TEST_GET_SERVICE:
			pr_info("Running test case 1: Direct Path\n");
			if (si_core_get_service_test(&oic) != 0)
				pr_err("SI_CORE_KERNEL_TEST_GET_SERVICE failed.\n");
			else
				pr_info("SI_CORE_KERNEL_TEST_GET_SERVICE succeed.\n");
			break;
		case SI_CORE_KERNEL_TEST_LOAD_APP:
			pr_info("Running test case 2: Loading TA/Sending command\n");

			kernel_buffer = kmalloc(msg.len, GFP_KERNEL);
			user_addr = u64_to_user_ptr(msg.file);
			if (copy_from_user(kernel_buffer, user_addr, msg.len)) {
				kfree(kernel_buffer);
				pr_err("copy_from_user failed\n");
				break;
			}

			if (si_core_kernel_test_load_app(&oic, kernel_buffer, msg.len) != 0)
				pr_err("SI_CORE_KERNEL_TEST_LOAD_APP failed.\n");
			else
				pr_info("SI_CORE_KERNEL_TEST_LOAD_APP succeed.\n");
			break;
		default:
			pr_err("Invalid test case\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int __init si_core_test_mod_init(void)
{
	int err;

	err = alloc_chrdev_region(&si_core_device_no, 0, 1, "si_core_test_client");
	if (err < 0)
		return err;

	driver_class = class_create("si_core_test_client");

	if (IS_ERR(driver_class)) {
		pr_err("class_create failed.\n");
		return -1;
	}

	cdev_init(&si_core_cdev, &fops);
	si_core_cdev.owner = THIS_MODULE;

	err = cdev_add(&si_core_cdev, MKDEV(MAJOR(si_core_device_no), 0), 1);
	if (err < 0) {
		pr_err("class_create failed.\n");
		return err;
	}

	si_core_dev = device_create(driver_class, NULL, si_core_device_no, NULL,
					"si_core_test_client");
	if (IS_ERR(si_core_dev)) {
		err = PTR_ERR(si_core_dev);
		pr_err("device_create failed %d.\n", err);
		return err;
	}

	return 0;
}

#else

static int __init si_core_test_mod_init(void)
{
	pr_info("CONFIG_QCOM_SI_CORE not enabled - skipping test\n");
	return 0;
}
#endif /* CONFIG_QCOM_SI_CORE */
module_init(si_core_test_mod_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SI-CORE Test Kernel Driver.");
