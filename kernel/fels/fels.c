/*
 * Copyright 2023 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <uapi/fels/fels_common_types.h>
#include <fels.h>

#define FELS_LOG_LEN 128
#define FELS_DEBUG_LOG_LEN 32

static DEFINE_SPINLOCK(log_lock);
static DEFINE_SPINLOCK(debug_lock);
static unsigned int log_count = FELS_MAX_LOG_COUNT;
static unsigned int debug_log_count = FELS_MAX_LOG_COUNT;

static struct kobject *fels_obj;

struct fels_msg_list {
	char *msg_log;
	struct list_head list;
};

static LIST_HEAD(head_log);
static LIST_HEAD(head_debug_log);

static unsigned int fels_debug_enable = 1;

static int fels_debug_enable_set(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	unsigned long irq_flags;

	spin_lock_irqsave(&debug_lock, irq_flags);
	ret = param_set_int(val, kp);
	if (ret)
		goto out;

	if (!fels_debug_enable) {
		if (!list_empty(&head_debug_log)) {
			struct fels_msg_list *cur, *temp;

			list_for_each_entry_safe(cur, temp, &head_debug_log, list) {
				list_del(&cur->list);
				debug_log_count++;
				kfree(cur->msg_log);
				kfree(cur);
			}
		}
	}
out:
	spin_unlock_irqrestore(&debug_lock, irq_flags);
	return ret;
}

const struct kernel_param_ops fels_debug_ops = {
	.set = fels_debug_enable_set,
	.get = param_get_int,
};

module_param_cb(fels_debug_enable, &fels_debug_ops, &fels_debug_enable, 0644);

int fels_log(fels_category_t category, fels_log_level_t log_level,
		fels_error_code_t error_code, int param1, int param2,
		int param3, int param4)
{
	unsigned long irq_flags;
	struct fels_msg_list *msg;

	spin_lock_irqsave(&log_lock, irq_flags);
	if (log_count == 0) {
		spin_unlock_irqrestore(&log_lock, irq_flags);
		return FELS_FULL_OF_LOGS;
	}
	spin_unlock_irqrestore(&log_lock, irq_flags);

	if (!fels_category_validate(category))
		return FELS_ILLEGAL_ARGUMENT;
	if (!fels_log_level_validate(log_level))
		return FELS_ILLEGAL_ARGUMENT;
	if (!fels_error_code_validate(error_code))
		return FELS_ILLEGAL_ARGUMENT;

	msg = kmalloc(sizeof(struct fels_msg_list),
			in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
	if (!msg)
		return FELS_NOMEM;

	msg->msg_log = kmalloc(FELS_LOG_LEN, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
	if (!msg->msg_log) {
		kfree(msg);
		return FELS_NOMEM;
	}

	snprintf(msg->msg_log, FELS_LOG_LEN, "%d,%d,%d,%d,%d,%d,%d", category, log_level,
		error_code, param1, param2, param3, param4);
	spin_lock_irqsave(&log_lock, irq_flags);
	list_add_tail(&msg->list, &head_log);
	log_count--;
	spin_unlock_irqrestore(&log_lock, irq_flags);

	sysfs_notify(fels_obj, NULL, "log");
	return FELS_SUCCESS;
}
EXPORT_SYMBOL(fels_log);

int fels_debug_log(fels_category_t category, fels_log_level_t log_level, const char *debug_msg)
{
	size_t len;
	unsigned long irq_flags;
	struct fels_msg_list *msg;

	spin_lock_irqsave(&debug_lock, irq_flags);
	if (!fels_debug_enable) {
		spin_unlock_irqrestore(&debug_lock, irq_flags);
		return FELS_ERROR;
	}

	if (debug_log_count == 0) {
		spin_unlock_irqrestore(&debug_lock, irq_flags);
		return FELS_FULL_OF_LOGS;
	}
	spin_unlock_irqrestore(&debug_lock, irq_flags);

	if (!debug_msg)
		return FELS_ILLEGAL_ARGUMENT;
	if (!fels_category_validate(category))
		return FELS_ILLEGAL_ARGUMENT;
	if (!fels_log_level_validate(log_level))
		return FELS_ILLEGAL_ARGUMENT;

	len = strnlen(debug_msg, FELS_MAX_DEBUG_MSG_LEN + 1);
	if (len > FELS_MAX_DEBUG_MSG_LEN)
		return FELS_ILLEGAL_ARGUMENT;

	msg = kmalloc(sizeof(struct fels_msg_list),
			in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
	if (!msg)
		return FELS_NOMEM;

	msg->msg_log = kmalloc((len + FELS_DEBUG_LOG_LEN),
			in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
	if (!msg->msg_log) {
		kfree(msg);
		return FELS_NOMEM;
	}

	snprintf(msg->msg_log, (len + FELS_DEBUG_LOG_LEN), "%d,%d,%s", category, log_level,
			debug_msg);

	spin_lock_irqsave(&debug_lock, irq_flags);
	if (!fels_debug_enable) {
		spin_unlock_irqrestore(&debug_lock, irq_flags);
		kfree(msg->msg_log);
		kfree(msg);
		return FELS_ERROR;
	}
	list_add_tail(&msg->list, &head_debug_log);
	debug_log_count--;
	spin_unlock_irqrestore(&debug_lock, irq_flags);

	sysfs_notify(fels_obj, NULL, "debug_log");

	return FELS_SUCCESS;
}
EXPORT_SYMBOL(fels_debug_log);

static ssize_t fels_log_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	unsigned long irq_flags;

	spin_lock_irqsave(&log_lock, irq_flags);
	if (!list_empty(&head_log)) {
		struct fels_msg_list *cur = list_first_entry(&head_log, struct fels_msg_list, list);

		len += sysfs_emit_at(buf, len, "%s\n", cur->msg_log);
		list_del(&cur->list);
		log_count++;
		kfree(cur->msg_log);
		kfree(cur);
	}
	spin_unlock_irqrestore(&log_lock, irq_flags);
	return len;
}

static ssize_t fels_debug_log_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	unsigned long irq_flags;

	spin_lock_irqsave(&debug_lock, irq_flags);
	if (!list_empty(&head_debug_log)) {
		struct fels_msg_list *cur = list_first_entry(&head_debug_log,
				struct fels_msg_list, list);

		len += sysfs_emit_at(buf, len, "%s\n", cur->msg_log);
		list_del(&cur->list);
		debug_log_count++;
		kfree(cur->msg_log);
		kfree(cur);
	}
	spin_unlock_irqrestore(&debug_lock, irq_flags);
	return len;
}

static struct kobj_attribute fels_log_attr =
		__ATTR(log, 0440, fels_log_show, NULL);
static struct kobj_attribute fels_debug_log_attr =
		__ATTR(debug_log, 0440, fels_debug_log_show, NULL);

static struct attribute *fels_attrs[] = {
	&fels_log_attr.attr,
	&fels_debug_log_attr.attr,
	NULL,
};

static struct attribute_group fels_group = {
	.attrs = fels_attrs,
};

static int __init fels_init(void)
{
	int ret;

	fels_obj = kobject_create_and_add("fels", kernel_kobj);
	if (!fels_obj)
		return -ENOMEM;

	ret = sysfs_create_group(fels_obj, &fels_group);
	if (ret)
		goto r_kobj;

	return 0;
r_kobj:
	kobject_put(fels_obj);
	return ret;
}

static void __exit fels_exit(void)
{
	sysfs_remove_group(fels_obj, &fels_group);
	kobject_put(fels_obj);
}
module_init(fels_init);
module_exit(fels_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Fatal error logging driver");
