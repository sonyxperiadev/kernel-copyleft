/* drivers/misc/felica_vdev.c
 *
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <asm/ioctls.h>

#define FELICA_VDEV_BUF_SIZE 4096
#define FELICA_VDEV_TIMEOUT 3000

#define FELICA_READ_CMD		_IOWR('f', 0, unsigned)
#define FELICA_READ_RESP_CMD	_IOWR('f', 1, unsigned)
#define FELICA_WRITE_CMD	_IOWR('f', 2, unsigned)
#define FELICA_WRITE_RESP_CMD	_IOWR('f', 3, unsigned)
#define FELICA_OTHER_CMD	_IOWR('f', 4, unsigned)
#define FELICA_OTHER_RESP_CMD	_IOWR('f', 5, unsigned)
#define FELICA_STS_CMD		_IOWR('f', 6, unsigned)

enum felica_cmd_type {
	FELICA_CMD_TYPE_READ = 0,
	FELICA_CMD_TYPE_WRITE,
	FELICA_CMD_TYPE_OTHER,
	FELICA_CMD_TYPE_SIZE,
};

enum felica_cmd {
	FELICA_CMD_NIL = 0,
	FELICA_CMD_OPEN,
	FELICA_CMD_READ,
	FELICA_CMD_WRITE,
	FELICA_CMD_CLOSE,
	FELICA_CMD_FIONREAD,
};

struct vdev_cmd {
	enum felica_cmd cmd;
	int count;
};

struct vdev_cmd_resp {
	int result;
	int resp;
};

struct vdev_data {
	int vdev_ref;
	int vsub_ref;

	char send_buff[FELICA_VDEV_BUF_SIZE];
	char recv_buff[FELICA_VDEV_BUF_SIZE];
	int send_size;
	struct vdev_cmd vdev_cmd[FELICA_CMD_TYPE_SIZE];
	struct vdev_cmd_resp resp[FELICA_CMD_TYPE_SIZE];

	struct semaphore sem[FELICA_CMD_TYPE_SIZE];
	struct semaphore sem_sub[FELICA_CMD_TYPE_SIZE];

	wait_queue_head_t waitq[FELICA_CMD_TYPE_SIZE];
	wait_queue_head_t waitq_sub[FELICA_CMD_TYPE_SIZE];
};

static char *vdev_cmd_name[FELICA_CMD_TYPE_SIZE] = {
	"TYPE_READ", "TYPE_WRITE", "TYPE_OTHER",
};

static struct vdev_data *vdev_data;

static ssize_t felica_vsub_read(struct file *file, char __user *buf,
			size_t count, loff_t *offset)
{
	ssize_t size = 0;

	pr_debug("%s:[s]\n", __func__);
	if (!vdev_data->vsub_ref) {
		size = -EBADF;
		goto out;
	}
	if (!vdev_data->send_size)
		goto out;

	if (count < vdev_data->send_size) {
		size = -EINVAL;
		goto out;
	}

	size = vdev_data->send_size;

	if (copy_to_user(buf, vdev_data->send_buff, size))
		size = -EFAULT;

out:
	pr_debug("%s:[e]%d %d\n", __func__, count, size);
	return size;

}

static ssize_t felica_vsub_write(struct file *file, const char __user *buf,
			size_t count, loff_t *offset)
{
	ssize_t size = 0;

	pr_debug("%s:[s]\n", __func__);
	if (!vdev_data->vsub_ref) {
		size = -EBADF;
		goto out;
	}

	if (count > FELICA_VDEV_BUF_SIZE) {
		size = -EINVAL;
		goto out;
	}

	size = count;
	if (copy_from_user(vdev_data->recv_buff, buf, size))
		size = -EFAULT;
out:
	pr_debug("%s:[e]%d %d\n", __func__, count, size);
	return size;
}

static void felica_vsub_cmd_resp(int cmd_type, unsigned long arg)
{
	pr_debug("%s:[s]%s\n", __func__, vdev_cmd_name[cmd_type]);
	down(&vdev_data->sem_sub[cmd_type]);
	vdev_data->resp[cmd_type].result = arg;
	vdev_data->resp[cmd_type].resp = 1;
	wake_up_interruptible(&vdev_data->waitq[cmd_type]);
	up(&vdev_data->sem_sub[cmd_type]);
	pr_debug("%s:[e]%s %d\n", __func__, vdev_cmd_name[cmd_type],
			vdev_data->resp[cmd_type].result);
}

static long felica_vsub_cmd_ioctl(enum felica_cmd_type cmd_type,
					unsigned long arg)
{
	int ret;
	int error = 0;

	pr_debug("%s:[s]%s\n", __func__, vdev_cmd_name[cmd_type]);
	down(&vdev_data->sem_sub[cmd_type]);
	ret = wait_event_interruptible(vdev_data->waitq_sub[cmd_type],
					(!vdev_data->vsub_ref ||
					vdev_data->vdev_cmd[cmd_type].cmd));
	if (ret) {
		error = -EFAULT;
		goto out;
	}
	if (copy_to_user((void *)arg, &vdev_data->vdev_cmd[cmd_type],
		sizeof(struct vdev_cmd))) {
		vdev_data->vdev_cmd[cmd_type].cmd = FELICA_CMD_NIL;
		vdev_data->vdev_cmd[cmd_type].count = 0;
		error = -EFAULT;
		vdev_data->resp[cmd_type].result = -EFAULT;
		vdev_data->resp[cmd_type].resp = 1;
		wake_up_interruptible(&vdev_data->waitq[cmd_type]);
		goto out;
	}
	vdev_data->vdev_cmd[cmd_type].cmd = FELICA_CMD_NIL;
	vdev_data->vdev_cmd[cmd_type].count = 0;

out:
	up(&vdev_data->sem_sub[cmd_type]);
	pr_debug("%s:[e]%s %d\n", __func__, vdev_cmd_name[cmd_type], error);
	return error;
}

static long felica_vsub_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	long error = 0;

	pr_debug("%s:[s]\n", __func__);

	switch (cmd) {
	case FELICA_READ_CMD:
		error = felica_vsub_cmd_ioctl(FELICA_CMD_TYPE_READ, arg);
		break;
	case FELICA_WRITE_CMD:
		error = felica_vsub_cmd_ioctl(FELICA_CMD_TYPE_WRITE, arg);
		break;
	case FELICA_OTHER_CMD:
		error = felica_vsub_cmd_ioctl(FELICA_CMD_TYPE_OTHER, arg);
		break;
	case FELICA_READ_RESP_CMD:
		felica_vsub_cmd_resp(FELICA_CMD_TYPE_READ, arg);
		break;
	case FELICA_WRITE_RESP_CMD:
		felica_vsub_cmd_resp(FELICA_CMD_TYPE_WRITE, arg);
		break;
	case FELICA_OTHER_RESP_CMD:
		felica_vsub_cmd_resp(FELICA_CMD_TYPE_OTHER, arg);
		break;
	case FELICA_STS_CMD:
		if (copy_to_user((void *)arg, &vdev_data->vdev_ref,
					sizeof(vdev_data->vdev_ref))) {
			error = -EFAULT;
			goto out;
		}
		break;
	default:
		error = -EINVAL;
		break;
	}
out:
	pr_debug("%s:[e]%d %ld\n", __func__, cmd, error);
	return error;
}

static int felica_vsub_open(struct inode *inode, struct file *file)
{
	int error = 0;

	pr_debug("%s:[s]\n", __func__);
	if (vdev_data->vsub_ref > 0) {
		vdev_data->vsub_ref++;
		goto out;
	}
	vdev_data->vsub_ref++;
out:
	pr_debug("%s:[e]%d\n", __func__, error);
	return error;

}

static int felica_vsub_release(struct inode *inode, struct file *file)
{
	int i;

	pr_debug("%s:[s]\n", __func__);
	vdev_data->vsub_ref--;
	if (vdev_data->vsub_ref <= 0) {
		vdev_data->vsub_ref = 0;
		for (i = 0; i < FELICA_CMD_TYPE_SIZE; i++)
			wake_up_interruptible(&vdev_data->waitq_sub[i]);
	}
	pr_debug("%s:[e]%d %d\n", __func__, vdev_data->vsub_ref, 0);
	return 0;
}

static ssize_t felica_vdev_read(struct file *file, char __user *buf,
			size_t count, loff_t *offset)
{
	ssize_t size = 0;
	int ret;
	enum felica_cmd_type type = FELICA_CMD_TYPE_READ;

	pr_debug("%s:[s]\n", __func__);
	down(&vdev_data->sem[type]);
	if (!vdev_data->vdev_ref) {
		size = -EBADF;
		goto out;
	}

	if (count > FELICA_VDEV_BUF_SIZE) {
		size = -EINVAL;
		goto out;
	}

	vdev_data->resp[type].resp = 0;
	vdev_data->resp[type].result = 0;
	vdev_data->vdev_cmd[type].cmd = FELICA_CMD_READ;
	vdev_data->vdev_cmd[type].count = count;
	wake_up_interruptible(&vdev_data->waitq_sub[type]);

	ret = wait_event_interruptible_timeout(vdev_data->waitq[type],
				(!vdev_data->vdev_ref ||
				vdev_data->resp[type].resp),
				msecs_to_jiffies(FELICA_VDEV_TIMEOUT));
	if (ret <= 0) {
		size = -EIO;
		goto out;
	}

	size = vdev_data->resp[type].result;
	if (vdev_data->resp[type].result <= 0)
		goto out;

	if (copy_to_user(buf, vdev_data->recv_buff, size))
		size = -EFAULT;

out:
	up(&vdev_data->sem[type]);
	pr_debug("%s:[e]%d %d\n", __func__, count, size);
	return size;

}

static ssize_t felica_vdev_write(struct file *file, const char __user *buf,
			size_t count, loff_t *offset)
{
	ssize_t size = 0;
	int ret;
	enum felica_cmd_type type = FELICA_CMD_TYPE_WRITE;

	pr_debug("%s:[s]\n", __func__);
	down(&vdev_data->sem[type]);
	if (!vdev_data->vdev_ref) {
		size = -EBADF;
		goto out;
	}

	if (count > FELICA_VDEV_BUF_SIZE) {
		size = -EINVAL;
		goto out;
	}

	vdev_data->send_size = 0;
	if (copy_from_user(vdev_data->send_buff, buf, count)) {
		size = -EFAULT;
		goto out;
	}
	vdev_data->send_size = count;

	vdev_data->resp[type].resp = 0;
	vdev_data->resp[type].result = 0;
	vdev_data->vdev_cmd[type].cmd = FELICA_CMD_WRITE;
	vdev_data->vdev_cmd[type].count = count;
	wake_up_interruptible(&vdev_data->waitq_sub[type]);

	ret = wait_event_interruptible_timeout(vdev_data->waitq[type],
				(!vdev_data->vdev_ref ||
				vdev_data->resp[type].resp),
				msecs_to_jiffies(FELICA_VDEV_TIMEOUT));
	if (ret <= 0) {
		size = -EIO;
		goto out;
	}

	size = vdev_data->resp[type].result;

out:
	up(&vdev_data->sem[type]);
	pr_debug("%s:[e]%d %d\n", __func__, count, size);
	return size;
}

static long felica_vdev_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int ret;
	long error = 0;
	enum felica_cmd_type type = FELICA_CMD_TYPE_OTHER;

	pr_debug("%s:[s]\n", __func__);
	down(&vdev_data->sem[type]);
	if (!vdev_data->vdev_ref) {
		error = -EBADF;
		goto out;
	}

	if (cmd != FIONREAD) {
		error = -ENOTTY;
		goto out;
	}

	vdev_data->resp[type].resp = 0;
	vdev_data->resp[type].result = 0;
	vdev_data->vdev_cmd[type].cmd = FELICA_CMD_FIONREAD;
	vdev_data->vdev_cmd[type].count = 0;
	wake_up_interruptible(&vdev_data->waitq_sub[type]);

	ret = wait_event_interruptible_timeout(vdev_data->waitq[type],
				(!vdev_data->vdev_ref ||
				vdev_data->resp[type].resp),
				msecs_to_jiffies(FELICA_VDEV_TIMEOUT));
	if (ret <= 0) {
		error = -EIO;
		goto out;
	}

	if (vdev_data->resp[type].result < 0) {
		error = vdev_data->resp[type].result;
		goto out;
	}
	pr_debug("%s:[m]%d\n", __func__, vdev_data->resp[type].result);
	if (copy_to_user((void *)arg, &vdev_data->resp[type].result,
				sizeof(int)))
		error = -EFAULT;

out:
	up(&vdev_data->sem[type]);
	pr_debug("%s:[e]%ld\n", __func__, error);
	return error;
}

static int felica_vdev_open(struct inode *inode, struct file *file)
{
	int ret;
	int error = 0;
	enum felica_cmd_type type = FELICA_CMD_TYPE_OTHER;

	pr_debug("%s:[s]\n", __func__);
	down(&vdev_data->sem[type]);
	if (vdev_data->vdev_ref > 0) {
		vdev_data->vdev_ref++;
		goto out;
	}

	vdev_data->resp[type].resp = 0;
	vdev_data->resp[type].result = 0;
	vdev_data->vdev_cmd[type].cmd = FELICA_CMD_OPEN;
	vdev_data->vdev_cmd[type].count = 0;
	wake_up_interruptible(&vdev_data->waitq_sub[type]);

	ret = wait_event_interruptible_timeout(vdev_data->waitq[type],
				(vdev_data->resp[type].resp),
				msecs_to_jiffies(FELICA_VDEV_TIMEOUT));
	if (ret <= 0) {
		error = -EIO;
		goto out;
	}

	if (vdev_data->resp[type].result) {
		error = vdev_data->resp[type].result;
		goto out;
	}

	vdev_data->vdev_ref++;
out:
	up(&vdev_data->sem[type]);
	pr_debug("%s:[e]%d %d\n", __func__, vdev_data->vdev_ref, error);
	return error;

}

static int felica_vdev_release(struct inode *inode, struct file *file)
{
	int i;
	enum felica_cmd_type type = FELICA_CMD_TYPE_OTHER;

	pr_debug("%s:[s]\n", __func__);
	vdev_data->vdev_ref--;
	if (vdev_data->vdev_ref <= 0) {
		vdev_data->vdev_ref = 0;
		for (i = 0; i < FELICA_CMD_TYPE_SIZE; i++)
			wake_up_interruptible(&vdev_data->waitq[i]);
		down(&vdev_data->sem[type]);
		vdev_data->resp[type].resp = 0;
		vdev_data->resp[type].result = 0;
		vdev_data->vdev_cmd[type].cmd = FELICA_CMD_CLOSE;
		vdev_data->vdev_cmd[type].count = 0;
		wake_up_interruptible(&vdev_data->waitq_sub[type]);
		up(&vdev_data->sem[type]);
	}
	pr_debug("%s:[e]%d %d\n", __func__, vdev_data->vdev_ref, 0);
	return 0;
}

static int felica_vdev_fsync(struct file *file, loff_t start, loff_t end,
				int datasync)
{
	pr_debug("%s:[s]\n", __func__);
	if (!vdev_data->vsub_ref)
		return -EBADF;

	return 0;
}

static const struct file_operations felica_vdev_fops = {
	.owner		= THIS_MODULE,
	.read		= felica_vdev_read,
	.write		= felica_vdev_write,
	.unlocked_ioctl	= felica_vdev_ioctl,
	.open		= felica_vdev_open,
	.release	= felica_vdev_release,
	.fsync		= felica_vdev_fsync,
};

static const struct file_operations felica_vsub_fops = {
	.owner		= THIS_MODULE,
	.read		= felica_vsub_read,
	.write		= felica_vsub_write,
	.unlocked_ioctl	= felica_vsub_ioctl,
	.open		= felica_vsub_open,
	.release	= felica_vsub_release,
};

static struct miscdevice felica_vdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "felica_vdev",
	.fops = &felica_vdev_fops,
};

static struct miscdevice felica_vsubdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "felica_vsub",
	.fops = &felica_vsub_fops,
};

static int __init felica_vdev_init(void)
{
	int i;

	vdev_data = kmalloc(sizeof(struct vdev_data), GFP_KERNEL);
	if (!vdev_data) {
		pr_err("%s: kmalloc failed.\n", __func__);
		return -ENOMEM;
	}

	if (misc_register(&felica_vdevice)) {
		pr_err("%s: misc_register failed.\n", __func__);
		kfree(vdev_data);
		return -ENODEV;
	}

	if (misc_register(&felica_vsubdevice)) {
		pr_err("%s: misc_register failed.\n", __func__);
		misc_deregister(&felica_vdevice);
		kfree(vdev_data);
		return -ENODEV;
	}

	memset(vdev_data, 0, sizeof(struct vdev_data));
	for (i = 0; i < FELICA_CMD_TYPE_SIZE; i++) {
		sema_init(&vdev_data->sem[i], 1);
		sema_init(&vdev_data->sem_sub[i], 1);
		init_waitqueue_head(&vdev_data->waitq[i]);
		init_waitqueue_head(&vdev_data->waitq_sub[i]);
	}
	return 0;
}

static void __exit felica_vdev_exit(void)
{
	misc_deregister(&felica_vsubdevice);
	misc_deregister(&felica_vdevice);
}

module_init(felica_vdev_init);
module_exit(felica_vdev_exit);

MODULE_DESCRIPTION("FeliCa virtual device driver");
MODULE_LICENSE("GPL");
