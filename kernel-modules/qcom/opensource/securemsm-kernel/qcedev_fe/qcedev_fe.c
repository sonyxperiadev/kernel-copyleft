// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/qcedev.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include "qcedev_smmu.h"
#include "qcedev_fe.h"

#define QCE_FE_FIRST_MINOR 0
#define QCE_FE_MINOR_CNT   1
#define QCEDEV_MAX_BUFFERS      16
#define QCE_MM_HAB_ID  MM_GPCE_1
#define HAB_OPEN_WAIT_TIMEOUT_MS (300)

static struct cdev cdev_qce_fe;
static dev_t dev_qce_fe;
static struct class *class_qce_fe;
static struct completion create_hab_channel_done;

struct qce_fe_drv_hab_handles   *drv_handles;

static int qce_fe_hab_open(uint32_t *handle);

static int qce_fe_open(struct inode *i, struct file *f)
{
	struct qcedev_fe_handle *handle;

	handle = kzalloc(sizeof(struct qcedev_fe_handle), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;
	f->private_data = handle;
	mutex_init(&handle->registeredbufs.lock);
	INIT_LIST_HEAD(&handle->registeredbufs.list);
	pr_info("%s : Done succesffuly\n", __func__);
	return 0;
}

static int qce_fe_close(struct inode *i, struct file *f)
{
	struct qcedev_fe_handle *handle;

	pr_info("%s: Driver close\n", __func__);
	handle =  f->private_data;

	if (qcedev_unmap_all_buffers(handle, drv_handles))
		pr_err("%s: failed to unmap all ion buffers\n", __func__);

	kfree(handle);
	f->private_data = NULL;
	return 0;
}

#if (KERNEL_VERSION(2, 6, 35) > LINUX_VERSION_CODE)
static int gce_fe_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg)
#else
static long gce_fe_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
#endif
{
	int err = 0;
	struct qcedev_fe_handle *handle;

	handle =  f->private_data;

	/* Verify user arguments. */
	if (_IOC_TYPE(cmd) != QCEDEV_IOC_MAGIC) {
		err = -ENOTTY;
		goto exit_qcedev;
	}

	switch (cmd) {
	case QCEDEV_IOCTL_MAP_BUF_REQ:
		{
			unsigned long long vaddr = 0;
			struct qcedev_map_buf_req map_buf = { {0} };
			int i = 0;

			if (copy_from_user(&map_buf,
					(void __user *)arg, sizeof(map_buf))) {
				err = -EFAULT;
				goto exit_qcedev;
			}

			if (map_buf.num_fds > ARRAY_SIZE(map_buf.fd)) {
				pr_err("%s: err: num_fds = %d exceeds max value\n",
							__func__, map_buf.num_fds);
				err = -EINVAL;
				goto exit_qcedev;
			}

			for (i = 0; i < map_buf.num_fds; i++) {
				err = qcedev_check_and_map_buffer(handle,
						map_buf.fd[i],
						map_buf.fd_offset[i],
						map_buf.fd_size[i],
						&vaddr,
						drv_handles);
				if (err) {
					pr_err(
						"%s: err: failed to map fd(%d) - %d\n",
						__func__, map_buf.fd[i], err);
					goto exit_qcedev;
				}
				map_buf.buf_vaddr[i] = vaddr;
				pr_info("%s: info: vaddr = %llx\n",
					__func__, vaddr);
			}

			if (copy_to_user((void __user *)arg, &map_buf,
					sizeof(map_buf))) {
				err = -EFAULT;
				goto exit_qcedev;
			}
			break;
		}

	case QCEDEV_IOCTL_UNMAP_BUF_REQ:
		{
			struct qcedev_unmap_buf_req unmap_buf = { { 0 } };
			int i = 0;

			if (copy_from_user(&unmap_buf,
				(void __user *)arg, sizeof(unmap_buf))) {
				err = -EFAULT;
				goto exit_qcedev;
			}
			if (unmap_buf.num_fds > ARRAY_SIZE(unmap_buf.fd)) {
				pr_err("%s: err: num_fds = %d exceeds max value\n",
							__func__, unmap_buf.num_fds);
				err = -EINVAL;
				goto exit_qcedev;
			}

			for (i = 0; i < unmap_buf.num_fds; i++) {
				err = qcedev_check_and_unmap_buffer(handle,
						unmap_buf.fd[i],
						drv_handles);
				if (err) {
					pr_err(
						"%s: err: failed to unmap fd(%d) - %d\n",
						 __func__,
						unmap_buf.fd[i], err);
					goto exit_qcedev;
				}
			}
			break;
		}
	default:
		pr_err("QCE_FE: Failed. Invalid  IOCTL cmd  0x%x\n", cmd);
		err = -EINVAL;
	}
exit_qcedev:
	return err;
}


static const struct file_operations qce_fe_fops = {
	.owner = THIS_MODULE,
	.open = qce_fe_open,
	.release = qce_fe_close,
#if (KERNEL_VERSION(2, 6, 35) > LINUX_VERSION_CODE)
	.ioctl = gce_fe_ioctl
#else
	.unlocked_ioctl = gce_fe_ioctl
#endif
};

static int qce_fe_hab_open(uint32_t *handle)
{
	int ret;

	if (handle == NULL || *handle != 0) {
		pr_err("Invalid parameters\n");
		return -EINVAL;
	}
	ret = habmm_socket_open(handle, QCE_MM_HAB_ID, 0, 0);
	if (ret) {
		pr_err("habmm_socket_open failed with ret = %d\n", ret);
		return ret;
	}
	return 0;
}

static int qce_fe_create_hab_channel(void *pv)
{
	int ret = 0;
	int i = 0;

	while (!kthread_should_stop()) {
		drv_handles = kzalloc(sizeof(struct qce_fe_drv_hab_handles), GFP_KERNEL);
		if (drv_handles == NULL)
			return -ENOMEM;
		spin_lock_init(&(drv_handles->handle_lock));
		/* open hab handles which will be used for communication with QCE backend */
		for (i = 0 ; i < HAB_HANDLE_NUM; i++) {
			ret = qce_fe_hab_open(&(drv_handles->qce_fe_hab_handles[i].handle));
			if (ret != 0)
				return ret;
			drv_handles->qce_fe_hab_handles[i].in_use = false;
			if (i == 0)
				drv_handles->initialized = true;
		}
		complete(&create_hab_channel_done);
		pr_info("%s: Create hab channel succeeded\n", __func__);
		break;
	}
	return ret;
}

static int qce_fe_hab_close(uint32_t handle)
{
	int ret;

	if (handle == 0)
		return 0;

	ret = habmm_socket_close(handle);
	if (ret) {
		pr_err("habmm_socket_close failed with ret = %d\n", ret);
		return ret;
	}
	return 0;
}

static void qce_fe_destroy_hab_channel(void)
{
	int i = 0;

	if (drv_handles == NULL)
		return;
	spin_lock(&(drv_handles->handle_lock));
	/* open hab handles which will be used for communication with QCE backend */
	for (i = 0 ; i < HAB_HANDLE_NUM; i++)
		qce_fe_hab_close(drv_handles->qce_fe_hab_handles[i].handle);
	spin_unlock(&(drv_handles->handle_lock));
	pr_info("%s: Close hab channel succeeded\n", __func__);
}

static int __init qce_fe_init(void)
{
	int ret;
	struct device *dev_ret;
	struct task_struct *create_channel_kthread_task;

	pr_info("%s:QCE FE driver init\n", __func__);
	/* Dynamically allocate device numbers */
	ret = alloc_chrdev_region(&dev_qce_fe, QCE_FE_FIRST_MINOR, QCE_FE_MINOR_CNT, "qce");
	if (ret < 0) {
		pr_err("%s: failed with error: %d\n", __func__, ret);
		return -EFAULT;
	}
	cdev_init(&cdev_qce_fe, &qce_fe_fops);
	ret = cdev_add(&cdev_qce_fe, dev_qce_fe, QCE_FE_MINOR_CNT);
	if (ret < 0) {
		pr_err("%s: cdev_add() failed with error: %d\n", __func__, ret);
		ret = -EFAULT;
		goto unregister_chrdev_region_error;
	}

	class_qce_fe = class_create(THIS_MODULE, "qce");
	if (IS_ERR_OR_NULL(class_qce_fe)) {
		pr_err("%s: class_create() failed\n", __func__);
		ret = -EFAULT;
		goto cdev_del_error;
	}
	/* create a device and registers it with sysfs */
	dev_ret = device_create(class_qce_fe, NULL, dev_qce_fe, NULL, "qce");
	if (IS_ERR_OR_NULL(dev_ret)) {
		pr_err("%s: device_create() failed\n", __func__);
		ret = -EFAULT;
		goto class_destroy_error;
	}
	init_completion(&create_hab_channel_done);
	create_channel_kthread_task = kthread_run(qce_fe_create_hab_channel, NULL,
											 "thread_create_channel");
	if (IS_ERR(create_channel_kthread_task)) {
		pr_err("fail to create kthread to create hab channels\n");
		ret = -EFAULT;
		goto device_destroy_error;
	}
	if (wait_for_completion_interruptible_timeout(
		&create_hab_channel_done, msecs_to_jiffies(HAB_OPEN_WAIT_TIMEOUT_MS)) <= 0) {
		pr_err("%s: timeout hit\n", __func__);
		if ((drv_handles != NULL) && (drv_handles->initialized)) {
			pr_info("%s:create hab channels partially succeeded\t"
					"performance might be affected\n", __func__);
			kthread_stop(create_channel_kthread_task);
			return 0;
		}
		pr_err("%s:create hab channels failed, unloading qce_fe\n", __func__);
		kthread_stop(create_channel_kthread_task);
		/*termporarily don't set ret value */
		//ret = -ETIME;
		ret = 0;
		goto device_destroy_error;
	}
	return 0;
device_destroy_error:
	device_destroy(class_qce_fe, dev_qce_fe);

class_destroy_error:
	class_destroy(class_qce_fe);

cdev_del_error:
	cdev_del(&cdev_qce_fe);

unregister_chrdev_region_error:
	unregister_chrdev_region(dev_qce_fe, QCE_FE_MINOR_CNT);

	return ret;
}

static void __exit qce_fe_exit(void)
{
	pr_info("%s: Unloading qce fe driver.\n", __func__);
	qce_fe_destroy_hab_channel();
	device_destroy(class_qce_fe, dev_qce_fe);
	class_destroy(class_qce_fe);
	cdev_del(&cdev_qce_fe);
	unregister_chrdev_region(dev_qce_fe, QCE_FE_MINOR_CNT);
}

module_init(qce_fe_init);
module_exit(qce_fe_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QCE FE");
MODULE_IMPORT_NS(DMA_BUF);
