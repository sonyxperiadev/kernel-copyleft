// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "ipa_i.h"
#include "ipa_opt_log.h"
#include <linux/msm_ipa.h>
#include <linux/sched/signal.h>
#include <linux/poll.h>

struct ipa_opt_log_context *ipa3_opt_log_ctx;

static void delete_first_node(void)
{
	struct ipa3_push_msg_wdi *msg;

	if (!list_empty(&ipa3_opt_log_ctx->opt_log_msg_list)) {
		msg = list_first_entry(&ipa3_opt_log_ctx->opt_log_msg_list,
				struct ipa3_push_msg_wdi, link);
		if (msg) {
			list_del(&msg->link);
			kfree(msg->buff);
			kfree(msg);
			ipa3_opt_log_ctx->stats.opt_log_drop_pkt++;
			if (atomic_read(&ipa3_opt_log_ctx->stats.numer_in_queue))
				atomic_dec(&ipa3_opt_log_ctx->stats.numer_in_queue);
		}
	} else {
		IPADBG("List Empty\n");
	}
}
int ipa3_send_opt_log_msg(const char *input_string)
{
	struct ipa3_push_msg_wdi *msg;
	void *data;
	size_t str_len;

	if (!ipa3_opt_log_ctx->opt_log_state.opt_log_open)
		return -ENXIO;

	IPADBG_LOW("Processing wdi message data\n");
	if (input_string == NULL) {
		IPADBG("Input string is NULL\n");
		return -EINVAL;
	}
	str_len = strlen(input_string);

	msg = kzalloc(sizeof(struct ipa3_push_msg_wdi), GFP_KERNEL);
	if (msg == NULL) {
		IPADBG("Memory allocation failed\n");
		return -ENOMEM;
	}

	data = kmemdup(input_string, str_len + 1, GFP_KERNEL);
	if (data == NULL) {
		kfree(msg);
		return -ENOMEM;
	}

	msg->buff = data;
	msg->len = str_len;

	mutex_lock(&ipa3_opt_log_ctx->opt_log_msg_lock);
	if (atomic_read(&ipa3_opt_log_ctx->stats.numer_in_queue) >=
						MAX_QUEUE_TO_OPT_LOG)
		delete_first_node();
	list_add_tail(&msg->link, &ipa3_opt_log_ctx->opt_log_msg_list);
	atomic_inc(&ipa3_opt_log_ctx->stats.numer_in_queue);
	mutex_unlock(&ipa3_opt_log_ctx->opt_log_msg_lock);

	wake_up(&ipa3_opt_log_ctx->opt_log_msg_waitq);
	IPA_STATS_INC_CNT(ipa3_opt_log_ctx->stats.opt_log_rx_pkt);

	return 0;
}
static int ipa_opt_log_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	mutex_lock(&ipa3_opt_log_ctx->ctx_lock);

	if (ipa3_opt_log_ctx->opt_log_state.opt_log_init) {
		ipa3_opt_log_ctx->stats.opt_log_drop_pkt = 0;
		atomic_set(&ipa3_opt_log_ctx->stats.numer_in_queue, 0);
		ipa3_opt_log_ctx->stats.opt_log_rx_pkt = 0;
		ipa3_opt_log_ctx->stats.opt_log_tx_diag_pkt = 0;
		ipa3_opt_log_ctx->opt_log_state.opt_log_open = true;
		ret = 0;
	} else {
		IPAERR("Trying to open opt log char device before initializing it\n");
		ret = -ENODEV;
	}

	mutex_unlock(&ipa3_opt_log_ctx->ctx_lock);
	return ret;
}
static int ipa_opt_log_release(struct inode *inode, struct file *filp)
{
	mutex_lock(&ipa3_opt_log_ctx->ctx_lock);
	ipa3_opt_log_ctx->opt_log_state.opt_log_open = false;
	mutex_unlock(&ipa3_opt_log_ctx->ctx_lock);
	return 1;
}
void ipa3_opt_log_cleanup(void)
{
	ipa3_opt_log_ctx->opt_log_state.opt_log_open = false;
	memset(&ipa3_opt_log_ctx->opt_log_state, 0, sizeof(ipa3_opt_log_ctx->opt_log_state));

	ipa3_opt_log_ctx->stats.opt_log_drop_pkt = 0;
	atomic_set(&ipa3_opt_log_ctx->stats.numer_in_queue, 0);
	ipa3_opt_log_ctx->stats.opt_log_rx_pkt = 0;
	ipa3_opt_log_ctx->stats.opt_log_tx_diag_pkt = 0;
}

/**
 * ipa_opt_log_read() - read message from IPA device
 * @filp:	[in] file pointer
 * @buf:	[out] buffer to read into
 * @count:	[in] size of above buffer
 * @f_pos:	[inout] file position
 *
 * User-space should continually read from /dev/ipa_events,
 * read will block when there are no messages to read.
 * Upon return, user-space should read
 * Buffer supplied must be big enough to
 * hold the data.
 *
 * Returns:	how many bytes copied to buffer
 *
 * Note:	Should not be called from atomic context
 */
static ssize_t ipa_opt_log_read(struct file *filp, char __user *buf, size_t count,
		  loff_t *f_pos)
{
	int ret =  0;
	char __user *start = buf;
	struct ipa3_push_msg_wdi *msg;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	add_wait_queue(&ipa3_opt_log_ctx->opt_log_msg_waitq, &wait);
	while (1) {

		IPADBG_LOW("Writing message to opt log\n");
		if (!ipa3_opt_log_ctx->opt_log_state.opt_log_open)
			break;

		mutex_lock(&ipa3_opt_log_ctx->opt_log_msg_lock);
		msg = NULL;
		if (!list_empty(&ipa3_opt_log_ctx->opt_log_msg_list)) {
			msg = list_first_entry(&ipa3_opt_log_ctx->opt_log_msg_list,
					struct ipa3_push_msg_wdi, link);
			list_del(&msg->link);
			if (atomic_read(&ipa3_opt_log_ctx->stats.numer_in_queue))
				atomic_dec(&ipa3_opt_log_ctx->stats.numer_in_queue);
		}

		mutex_unlock(&ipa3_opt_log_ctx->opt_log_msg_lock);

		if (msg != NULL) {
			if (msg->len > count) {
				IPAERR("Message length greater than count\n");
				kfree(msg->buff);
				kfree(msg);
				msg = NULL;
				ret = -EAGAIN;
				break;
			}

			if (msg->buff) {
				if (copy_to_user(buf, msg->buff,
							msg->len)) {
					ret = -EFAULT;
					kfree(msg->buff);
					kfree(msg);
					msg = NULL;
					ret = -EAGAIN;
					break;
				}
				buf += msg->len;
				count -= msg->len;
				kfree(msg->buff);
			}
			IPA_STATS_INC_CNT(ipa3_opt_log_ctx->stats.opt_log_tx_diag_pkt);
			kfree(msg);
			msg = NULL;
		}

		ret = -EAGAIN;
		if (filp->f_flags & O_NONBLOCK)
			break;

		ret = -EINTR;
		if (signal_pending(current))
			break;

		if (start != buf)
			break;
		wait_woken(&wait, TASK_INTERRUPTIBLE, MAX_SCHEDULE_TIMEOUT);
	}
	remove_wait_queue(&ipa3_opt_log_ctx->opt_log_msg_waitq, &wait);
	if (start != buf && ret != -EFAULT)
		ret = buf - start;

	return ret;
}
static const struct file_operations ipa_opt_log_fops = {
	.owner = THIS_MODULE,
	.open = ipa_opt_log_open,
	.release = ipa_opt_log_release,
	.read = ipa_opt_log_read,
};
int opt_log_file_test(void)
{
	return 1;
}
int ipa_opt_log_init(void)
{
	int result = 0;
	struct cdev *cdev;
	struct ipa3_opt_log_char_device_context *opt_log_cdev;

	ipa3_opt_log_ctx = kzalloc(sizeof(*ipa3_opt_log_ctx), GFP_KERNEL);
	if (!ipa3_opt_log_ctx) {
		result = -ENOMEM;
		goto fail_mem_ctx;
	}

	opt_log_cdev = &ipa3_opt_log_ctx->opt_log_cdev;
	INIT_LIST_HEAD(&ipa3_opt_log_ctx->opt_log_msg_list);
	init_waitqueue_head(&ipa3_opt_log_ctx->opt_log_msg_waitq);
	mutex_init(&ipa3_opt_log_ctx->opt_log_msg_lock);
	mutex_init(&ipa3_opt_log_ctx->ctx_lock);

	opt_log_cdev->class = class_create("ipa_events");

	if (IS_ERR(opt_log_cdev->class)) {
		IPAERR("Error: opt_log_cdev->class NULL\n");
		result = -ENODEV;
		goto create_char_dev0_fail;
	}

	result = alloc_chrdev_region(&opt_log_cdev->dev_num, 0, 1, "ipa_events");
	if (result) {
		IPAERR("alloc_chrdev_region error for ipa opt log\n");
		result = -ENODEV;
		goto alloc_chrdev0_region_fail;
	}

	opt_log_cdev->dev = device_create(opt_log_cdev->class, NULL,
		 opt_log_cdev->dev_num, ipa3_ctx, "ipa_events");
	if (IS_ERR(opt_log_cdev->dev)) {
		IPAERR("device_create err:%ld\n", PTR_ERR(opt_log_cdev->dev));
		result = PTR_ERR(opt_log_cdev->dev);
		goto device0_create_fail;
	}

	cdev = &opt_log_cdev->cdev;
	cdev_init(cdev, &ipa_opt_log_fops);
	cdev->owner = THIS_MODULE;
	cdev->ops = &ipa_opt_log_fops;

	result = cdev_add(cdev, opt_log_cdev->dev_num, 1);
	if (result) {
		IPAERR("cdev_add err=%d\n", -result);
		goto cdev0_add_fail;
	}

	ipa3_opt_log_ctx->opt_log_state.opt_log_init = true;
	return 0;

cdev0_add_fail:
	device_destroy(opt_log_cdev->class, opt_log_cdev->dev_num);
device0_create_fail:
	unregister_chrdev_region(opt_log_cdev->dev_num, 1);
alloc_chrdev0_region_fail:
	class_destroy(opt_log_cdev->class);
create_char_dev0_fail:
	kfree(ipa3_opt_log_ctx);
	ipa3_opt_log_ctx = NULL;
fail_mem_ctx:
	return result;
}
