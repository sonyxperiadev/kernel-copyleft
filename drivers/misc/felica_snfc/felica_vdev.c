/* drivers/misc/felica_snfc/felica_vdev.c
 *
 * Copyright (C) 2012-2013 Sony Mobile Communications AB.
 *
 * Author: Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>
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

enum felica_cmd_type {
	FELICA_CMD_TYPE_READ = 0,
	FELICA_CMD_TYPE_WRITE,
	FELICA_CMD_TYPE_OTHER,
	FELICA_CMD_TYPE_GET_STS,
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

struct vsub_cmd {
	int cmd_type;
	int cmd;
	int result;
};

struct vdev_data {
	int vdev_ref;
	int vsub_ref;
	int vsub_cmd_ref;

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
	"TYPE_READ", "TYPE_WRITE", "TYPE_OTHER", "TYPE_GET_STS",
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

	if (FELICA_VDEV_BUF_SIZE < count) {
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

static int felica_vsub_cmd_resp(struct vsub_cmd *cmd)
{
	int ret = 0;
	int cmd_type = cmd->cmd_type;

	if (0 > cmd_type || FELICA_CMD_TYPE_SIZE <= cmd_type ||
		FELICA_CMD_TYPE_GET_STS == cmd_type) {
		ret = -EINVAL;
		goto out;
	}

	pr_debug("%s:[s]%s\n", __func__, vdev_cmd_name[cmd_type]);
	down(&vdev_data->sem_sub[cmd_type]);
	vdev_data->resp[cmd_type].result = cmd->result;
	vdev_data->resp[cmd_type].resp = 1;
	wake_up_interruptible(&vdev_data->waitq[cmd_type]);
	up(&vdev_data->sem_sub[cmd_type]);

out:
	pr_debug("%s:[e]%s %d\n", __func__, vdev_cmd_name[cmd_type],
			vdev_data->resp[cmd_type].result);
	return ret;
}

static int felica_vsub_cmd(struct vsub_cmd *cmd)
{
	int ret = 0;
	int cmd_type = cmd->cmd_type;

	if (0 > cmd_type || FELICA_CMD_TYPE_SIZE <= cmd_type) {
		ret = -EINVAL;
		goto out;
	}

	pr_debug("%s:[s]%s\n", __func__, vdev_cmd_name[cmd_type]);

	if (FELICA_CMD_TYPE_GET_STS == cmd_type) {
		cmd->result = vdev_data->vdev_ref;
		goto sts_out;
	}

	down(&vdev_data->sem_sub[cmd_type]);
	ret = wait_event_interruptible(vdev_data->waitq_sub[cmd_type],
					(!vdev_data->vsub_cmd_ref ||
					vdev_data->vdev_cmd[cmd_type].cmd));
	if (ret) {
		ret = -EFAULT;
		goto out;
	}

	cmd->cmd = vdev_data->vdev_cmd[cmd_type].cmd;
	cmd->result = vdev_data->vdev_cmd[cmd_type].count;

	vdev_data->vdev_cmd[cmd_type].cmd = FELICA_CMD_NIL;
	vdev_data->vdev_cmd[cmd_type].count = 0;

out:
	up(&vdev_data->sem_sub[cmd_type]);
sts_out:
	pr_debug("%s:[e]%s %d\n", __func__, vdev_cmd_name[cmd_type], ret);
	return ret;
}

static int felica_vsub_open(struct inode *inode, struct file *file)
{
	int error = 0;

	pr_debug("%s:[s]\n", __func__);
	if (0 < vdev_data->vsub_ref) {
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
	pr_debug("%s:[s]\n", __func__);

	vdev_data->vsub_ref--;
	if (0 >= vdev_data->vsub_ref)
		vdev_data->vsub_ref = 0;

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

	if (FELICA_VDEV_BUF_SIZE < count) {
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
	if (0 >= ret) {
		size = -EIO;
		goto out;
	}

	size = vdev_data->resp[type].result;
	if (0 >= vdev_data->resp[type].result)
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

	if (FELICA_VDEV_BUF_SIZE < count) {
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
	if (0 >= ret) {
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

	if (FIONREAD != cmd) {
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
	if (0 >= ret) {
		error = -EIO;
		goto out;
	}

	if (0 > vdev_data->resp[type].result) {
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
	if (0 < vdev_data->vdev_ref) {
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
	if (0 >= ret) {
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
	if (0 >= vdev_data->vdev_ref) {
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
	if (!vdev_data->vdev_ref)
		return -EBADF;

	return 0;
}

static ssize_t felica_vsub_cmd_read(struct file *file, char __user *buf,
				size_t count, loff_t *offset)
{
	struct vsub_cmd cmd;
	ssize_t ret;

	pr_debug("%s:[s]\n", __func__);

	if (!vdev_data->vsub_cmd_ref) {
		ret = -EBADF;
		goto out;
	}

	if (count != sizeof(cmd)) {
		ret = -EINVAL;
		goto out;
	}

	if (copy_from_user(&cmd, buf, sizeof(cmd))) {
		ret = -EFAULT;
		goto out;
	}

	ret = felica_vsub_cmd(&cmd);
	if (ret) {
		pr_err("%s: command failed: %d: %d\n", __func__,
			cmd.cmd_type, ret);
		goto out;
	}

	if (copy_to_user(buf, &cmd, sizeof(cmd))) {
		ret = -EFAULT;
		goto out;
	}

	ret = count;

out:
	pr_debug("%s:[e]%d\n", __func__, ret);
	return ret;
}

static ssize_t felica_vsub_cmd_write(struct file *file, const char __user *buf,
			size_t count, loff_t *offset)
{
	struct vsub_cmd cmd;
	ssize_t ret;

	pr_debug("%s:[s]\n", __func__);

	if (!vdev_data->vsub_cmd_ref) {
		ret = -EBADF;
		goto out;
	}

	if (count != sizeof(cmd)) {
		ret = -EINVAL;
		goto out;
	}

	if (copy_from_user(&cmd, buf, sizeof(cmd))) {
		ret = -EFAULT;
		goto out;
	}

	ret = felica_vsub_cmd_resp(&cmd);
	if (ret) {
		pr_err("%s: command failed: %d: %d\n", __func__,
			cmd.cmd_type, ret);
		goto out;
	}

	ret = count;

out:
	pr_debug("%s:[e]%d\n", __func__, ret);
	return ret;
}

static int felica_vsub_cmd_open(struct inode *inode, struct file *file)
{
	int error = 0;

	pr_debug("%s:[s]\n", __func__);
	if (0 < vdev_data->vsub_cmd_ref) {
		vdev_data->vsub_cmd_ref++;
		goto out;
	}
	vdev_data->vsub_cmd_ref++;
out:
	pr_debug("%s:[e]%d\n", __func__, error);
	return error;

}

static int felica_vsub_cmd_release(struct inode *inode, struct file *file)
{
	int i;

	pr_debug("%s:[s]\n", __func__);
	vdev_data->vsub_cmd_ref--;
	if (0 >= vdev_data->vsub_cmd_ref) {
		vdev_data->vsub_cmd_ref = 0;
		for (i = 0; i < FELICA_CMD_TYPE_SIZE; i++)
			wake_up_interruptible(&vdev_data->waitq_sub[i]);
	}
	pr_debug("%s:[e]%d %d\n", __func__, vdev_data->vsub_cmd_ref, 0);
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
	.open		= felica_vsub_open,
	.release	= felica_vsub_release,
};

static const struct file_operations felica_vsub_cmd_fops = {
	.owner		= THIS_MODULE,
	.read		= felica_vsub_cmd_read,
	.write		= felica_vsub_cmd_write,
	.open		= felica_vsub_cmd_open,
	.release	= felica_vsub_cmd_release,
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

static struct miscdevice felica_vsubcmddevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "felica_vsub_cmd",
	.fops = &felica_vsub_cmd_fops,
};

static int __init felica_vdev_init(void)
{
	int i;
	int ret;

	vdev_data = kmalloc(sizeof(struct vdev_data), GFP_KERNEL);
	if (!vdev_data) {
		pr_err("%s: kmalloc failed.\n", __func__);
		ret = -ENOMEM;
		goto err_kmalloc;
	}

	if (misc_register(&felica_vdevice)) {
		pr_err("%s: vdev misc_register failed.\n", __func__);
		ret = -ENODEV;
		goto err_vdev_register;
	}

	if (misc_register(&felica_vsubdevice)) {
		pr_err("%s: vsub misc_register failed.\n", __func__);
		ret = -ENODEV;
		goto err_vsub_register;
	}

	if (misc_register(&felica_vsubcmddevice)) {
		pr_err("%s: vsub_cmd misc_register failed.\n", __func__);
		ret = -ENODEV;
		goto err_vsub_cmd_register;
	}

	memset(vdev_data, 0, sizeof(struct vdev_data));
	for (i = 0; i < FELICA_CMD_TYPE_SIZE; i++) {
		sema_init(&vdev_data->sem[i], 1);
		sema_init(&vdev_data->sem_sub[i], 1);
		init_waitqueue_head(&vdev_data->waitq[i]);
		init_waitqueue_head(&vdev_data->waitq_sub[i]);
	}
	return 0;

err_vsub_cmd_register:
	misc_deregister(&felica_vsubdevice);
err_vsub_register:
	misc_deregister(&felica_vdevice);
err_vdev_register:
	kfree(vdev_data);
err_kmalloc:
	return ret;
}

static void __exit felica_vdev_exit(void)
{
	misc_deregister(&felica_vsubdevice);
	misc_deregister(&felica_vdevice);
}

module_init(felica_vdev_init);
module_exit(felica_vdev_exit);

MODULE_AUTHOR("Manabu Yoshida <Manabu.X.Yoshida@sonymobile.com>");
MODULE_DESCRIPTION("FeliCa virtual device driver");
MODULE_LICENSE("GPL");
