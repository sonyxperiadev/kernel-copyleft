// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "gunyah_trace: " fmt

#include <asm/gunyah.h>
#include <linux/arm-smccc.h>
#include <linux/debugfs.h>
#include <linux/gunyah.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/wait.h>

#include "gunyah_trace.h"
#include "hcall_trace.h"

/**
 * struct gunyah_trace_buf - Local structure for basic Gunyah trace buffer information
 * @entries: Pointer to the trace buffer entries
 * @header: Pointer to the trace buffer header
 * @size: Size of the trace buffer
 * @entry_num: Number of entries in the trace buffer
 */
struct gunyah_trace_buf {
	void __iomem *entries;
	struct gunyah_trace_header *header;
	size_t size;
	uint32_t entry_num;
};

/**
 * struct gunyah_trace_data - Main data structure for Gunyah trace
 * @base: Base address of the Gunyah trace
 * @size: Total size of the Gunyah trace
 * @irq: IRQ number associated with Gunyah trace
 * @enabled_events: Bitmask of enabled trace event classes
 * @poll_waiters: Wait queue for poll() system call
 * @user_cnt: Reference count for users of the trace debugfs file
 * @trace_buf_cnt: Number of trace buffers
 * @parent_fwnode: Parent firmware node handle
 * @debugfs_dir: Debugfs directory for trace buffer
 * @debugfs_trace: Debugfs file for trace buffer output
 * @debugfs_set_event: Debugfs file for setting trace event classes
 * @debugfs_available_events: Debugfs file for output available trace events
 * @buf: Array pointer of Gunyah trace buffers
 */
struct gunyah_trace_data {
	void __iomem *base;
	size_t size;
	int irq;
	u64 enabled_events;
	wait_queue_head_t poll_waiters;
	atomic_t user_cnt;
	uint16_t trace_buf_cnt;
	struct fwnode_handle *parent_fwnode;
	struct dentry *debugfs_dir;
	struct dentry *debugfs_trace;
	struct dentry *debugfs_set_event;
	struct dentry *debugfs_available_events;
	struct gunyah_trace_buf *buf;
};

/**
 * struct gunyah_trace_buf_status - Represent the status of current debugfs output
 * @last_index: Last index has copied to userspace
 * @last_wrap_cnt: Last wrap count has copied to userspace
 * @lock: Spinlock for synchronizing access to the trace buffer status
 */
struct gunyah_trace_buf_status {
	uint32_t last_index;
	uint32_t last_wrap_cnt;
	spinlock_t lock;
};

/**
 * struct gunyah_trace_file_data - Private data for debugfs trace file
 * @trace_buf_data: Pointer to the main trace data structure
 * @trace_buf_status: Pointer to the trace buffer status structure
 */
struct gunyah_trace_file_data {
	struct gunyah_trace_data *trace_buf_data;
	struct gunyah_trace_buf_status *trace_buf_status;
};

static struct gunyah_trace_data gunyah_trace_data;

static irqreturn_t gunyah_trace_buf_irq_handler(int irq, void *data)
{
	struct gunyah_trace_data *trace_buf = data;

	wake_up_all(&trace_buf->poll_waiters);

	return IRQ_HANDLED;
}

/**
 * init_trace_buf_header - Initialize the trace buffer headers
 * @data: Pointer to the gunyah_trace_data structure containing trace buffer information
 *
 * This function initialize basic information of Gunyah trace by parsing the trace
 * buffer header one by one. It also check if the trace buffer header is valid
 * or not.
 *
 * Return:
 * * 0 on success
 * * -EINVAL if a trace buffer is invalid
 * * -ENOMEM if memory allocation for buffer structures fails
 */
static int init_trace_buf_header(struct gunyah_trace_data *data)
{
	struct gunyah_trace_header *trace_buf;
	int i, buf_cnt = 0;

	trace_buf = data->base;

	/* go through buffer to get trace buf count */
	do {
		if (le32_to_cpu(trace_buf->buf_magic) !=
		    GUNYAH_TRACE_BUF_MAGIC) {
			pr_err("trace buffer magic not match %x\n",
			       le32_to_cpu(trace_buf->buf_magic));
			return -EINVAL;
		}
		buf_cnt++;
		trace_buf = (void *)trace_buf + sizeof(*trace_buf) +
			    ENTRY_SIZE * le32_to_cpu(trace_buf->entry_num);
		if ((void *)trace_buf > data->base + data->size) {
			pr_err("Invalid trace buf address %p\n", trace_buf);
			return -EINVAL;
		}
	} while ((void *)trace_buf < data->base + data->size);

	data->buf = kmalloc_array(buf_cnt, sizeof(*data->buf), GFP_KERNEL);
	if (!data->buf)
		return -ENOMEM;
	data->trace_buf_cnt = buf_cnt;

	trace_buf = data->base;
	for (i = 0; i < buf_cnt; i++) {
		data->buf[i].header = trace_buf;
		data->buf[i].entries =
			(void *)trace_buf + sizeof(struct gunyah_trace_header);
		data->buf[i].entry_num = le32_to_cpu(trace_buf->entry_num);
		data->buf[i].size =
			le32_to_cpu(trace_buf->entry_num) * ENTRY_SIZE;
		trace_buf = (void *)trace_buf + sizeof(*trace_buf) +
			    ENTRY_SIZE * le32_to_cpu(trace_buf->entry_num);
	}

	return 0;
}

static int gunyah_trace_open(struct inode *inode, struct file *file)
{
	int ret, i;
	struct gunyah_trace_data *data = inode->i_private;
	struct gunyah_trace_header *header;
	struct gunyah_trace_file_data *file_data;

	ret = gunyah_hypercall_config_trace_buf_notify(true);
	if (ret)
		pr_err("Failed to enable trace buffer notify:%d\n", ret);

	file_data = kmalloc(sizeof(*file_data), GFP_KERNEL);
	if (!file_data)
		return -ENOMEM;
	file_data->trace_buf_status =
		kmalloc_array(data->trace_buf_cnt,
			      sizeof(*file_data->trace_buf_status), GFP_KERNEL);
	if (!file_data->trace_buf_status) {
		ret = -ENOMEM;
		goto free_data;
	}

	/* Mark the start of tracing */
	for (i = 0; i < data->trace_buf_cnt; i++) {
		spin_lock_init(&file_data->trace_buf_status[i].lock);
		header = data->buf[i].header;
		file_data->trace_buf_status[i].last_wrap_cnt =
			le32_to_cpu(header->wrap_count);
		file_data->trace_buf_status[i].last_index =
			le32_to_cpu(header->index);
	}

	file->private_data = file_data;
	atomic_inc(&gunyah_trace_data.user_cnt);

	return 0;

free_data:
	kfree(file_data);

	return ret;
}

static ssize_t gunyah_trace_read(struct file *file, char __user *user_buf,
				 size_t count, loff_t *pos)
{
	struct gunyah_trace_file_data *file_data = file->private_data;
	struct gunyah_trace_buf *buf;
	ssize_t buf_remain_size, buf_tocopy_size;
	uint32_t index, wrap_cnt, last_index, last_wrap_cnt, new_entry_number;
	ssize_t new_entry_size;
	int i;
	unsigned long flags;

	count = ALIGN_DOWN(count, ENTRY_SIZE);
	buf_remain_size = count;
	for (i = 0; i < gunyah_trace_data.trace_buf_cnt && buf_remain_size > 0; i++) {
		buf = &gunyah_trace_data.buf[i];

		index = le32_to_cpu(buf->header->index);
		wrap_cnt = le32_to_cpu(buf->header->wrap_count);
		last_index = file_data->trace_buf_status[i].last_index;
		last_wrap_cnt = file_data->trace_buf_status[i].last_wrap_cnt;

		while (buf_remain_size > 0) {
			if (wrap_cnt - last_wrap_cnt > 1) {
				/* buffer overflow */
				return -EINVAL;
			} else if (wrap_cnt - last_wrap_cnt == 1) {
				if (index >= last_index)
					/* buffer overflow */
					return -EINVAL;
				new_entry_number =
					buf->entry_num - last_index + index;
			} else {
				new_entry_number = index - last_index;
			}
			if (new_entry_number == 0)
				break;
			new_entry_size = new_entry_number * ENTRY_SIZE;
			/* if reach the end of buffer */
			buf_tocopy_size =
				min((ssize_t)(buf->entry_num - last_index - 1) *
					    ENTRY_SIZE, new_entry_size);
			/* check if buffer is enough */
			buf_tocopy_size = min(buf_tocopy_size, buf_remain_size);
			if (copy_to_user(user_buf,
					 (void *)buf->entries +
						 last_index * ENTRY_SIZE,
					 buf_tocopy_size) == buf_tocopy_size)
				return -EFAULT;

			user_buf += buf_tocopy_size;
			buf_remain_size -= buf_tocopy_size;
			last_wrap_cnt +=
				(last_index + buf_tocopy_size / ENTRY_SIZE) /
				(buf->entry_num - 1);
			last_index =
				(last_index + buf_tocopy_size / ENTRY_SIZE) %
				(buf->entry_num - 1);
		}
		spin_lock_irqsave(&file_data->trace_buf_status[i].lock, flags);
		file_data->trace_buf_status[i].last_wrap_cnt = last_wrap_cnt;
		file_data->trace_buf_status[i].last_index = last_index;
		spin_unlock_irqrestore(&file_data->trace_buf_status[i].lock,
				       flags);
	}

	return count - buf_remain_size;
}

/**
 * gunyah_trace_available - Check if new trace entry is available
 * @file_data: Pointer to the gunyah_trace_file_data structure containing file-specific trace data
 *
 * This function checks if there is new trace entry available in any of the trace buffers.
 *
 * Return:
 * * true if new trace entry is available
 * * false if no new trace entry is available
 */
static bool gunyah_trace_available(struct gunyah_trace_file_data *file_data)
{
	struct gunyah_trace_buf *buf;
	int i;
	unsigned long flags;
	bool updated;

	for (i = 0; i < gunyah_trace_data.trace_buf_cnt; i++) {
		buf = &gunyah_trace_data.buf[i];
		spin_lock_irqsave(&file_data->trace_buf_status[i].lock, flags);
		updated =
			buf->header->wrap_count >
				file_data->trace_buf_status[i].last_wrap_cnt ||
			buf->header->index >
				file_data->trace_buf_status[i].last_index;
		spin_unlock_irqrestore(&file_data->trace_buf_status[i].lock,
				       flags);
		if (updated)
			return true;
	}

	return false;
}

static __poll_t gunyah_trace_poll(struct file *file, poll_table *poll_table)
{
	struct gunyah_trace_file_data *data = file->private_data;

	poll_wait(file, &gunyah_trace_data.poll_waiters, poll_table);

	if (gunyah_trace_available(data))
		return EPOLLIN | EPOLLRDNORM;
	else
		return 0;
}

static int gunyah_trace_release(struct inode *inode, struct file *file)
{
	int ret;

	if (atomic_dec_and_test(&gunyah_trace_data.user_cnt)) {
		ret = gunyah_hypercall_config_trace_buf_notify(false);
		if (ret)
			pr_err("Failed to disable trace buffer notify:%d\n",
			       ret);
	}
	kfree(file->private_data);

	return 0;
}

static const struct file_operations gunyah_trace_debugfs_fops = {
	.open = gunyah_trace_open,
	.read = gunyah_trace_read,
	.poll = gunyah_trace_poll,
	.release = gunyah_trace_release,
};

static int gunyah_set_event_show(struct seq_file *m, void *v)
{
	uint64_t enabled_flags;
	int ret, i;

	ret = gunyah_hypercall_update_trace_flag(0, 0, &enabled_flags);
	if (ret != GUNYAH_ERROR_OK) {
		pr_err("Failed to update trace flag %d\n", ret);
		return -EINVAL;
	}

	for (i = 0; i < GUNYAH_TRACE_CLASS_MAX; i++) {
		if ((1LLU << gunyah_trace_class_bitmap[i].bit) &
		    enabled_flags) {
			seq_printf(m, "%s%s", i ? " " : "",
				   gunyah_trace_class_bitmap[i].name);
		}
	}
	seq_putc(m, '\n');

	return 0;
}

static ssize_t gunyah_set_event_write(struct file *file,
				      const char __user *user_buf, size_t count,
				      loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct gunyah_trace_data *data = m->private;
	char buf[MAX_CLASS_SIZE];
	char *class_str, *str;
	u64 set_flags = 0, enabled_flags;
	int i, ret;

	if (count > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;
	str = strstrip(buf);

	while ((class_str = strsep(&str, " "))) {
		if (strlen(class_str) == 0)
			continue;
		for (i = 0; i < GUNYAH_TRACE_CLASS_MAX; i++) {
			if (!strcmp(class_str,
				    gunyah_trace_class_bitmap[i].name)) {
				set_flags |=
					(1 << gunyah_trace_class_bitmap[i].bit);
				break;
			}
		}
		/* did not found the class name */
		if (i == GUNYAH_TRACE_CLASS_MAX)
			return -EINVAL;
	}
	set_flags |= data->enabled_events;

	ret = gunyah_hypercall_update_trace_flag(set_flags, ~set_flags,
						 &enabled_flags);
	if (ret != GUNYAH_ERROR_OK) {
		pr_err("Failed to update trace flag %d set %llx clear %llx enabled %llx\n",
		       ret, set_flags, ~set_flags, enabled_flags);
		return -EBUSY;
	}
	data->enabled_events = set_flags;

	return count;
}

static int gunyah_set_event_open(struct inode *inode, struct file *file)
{
	struct gunyah_trace_data *data = inode->i_private;

	if ((file->f_mode & FMODE_WRITE) && (file->f_flags & O_TRUNC))
		data->enabled_events = 0;

	return single_open(file, gunyah_set_event_show, inode->i_private);
}

static const struct file_operations set_event_debugfs_fops = {
	.open = gunyah_set_event_open,
	.write = gunyah_set_event_write,
	.read = seq_read,
	.llseek = seq_lseek,
};

static int gunyah_available_event_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < GUNYAH_TRACE_CLASS_MAX; i++) {
		seq_printf(m, "%s%s", i ? " " : "",
			   gunyah_trace_class_bitmap[i].name);
	}
	seq_putc(m, '\n');

	return 0;
}

static int gunyah_available_event_open(struct inode *inode, struct file *file)
{
	return single_open(file, gunyah_available_event_show, inode->i_private);
}

static const struct file_operations available_debugfs_fops = {
	.open = gunyah_available_event_open,
	.read = seq_read,
};

static int gunyah_create_debugfs_entries(struct gunyah_trace_data *trace_data)
{
	trace_data->debugfs_dir = debugfs_create_dir("gunyah_trace", NULL);
	if (IS_ERR_OR_NULL(trace_data->debugfs_dir))
		return -ENOENT;

	trace_data->debugfs_trace =
		debugfs_create_file("trace", 0440, trace_data->debugfs_dir,
				    trace_data, &gunyah_trace_debugfs_fops);
	if (IS_ERR_OR_NULL(trace_data->debugfs_trace))
		return -ENOENT;

	trace_data->debugfs_set_event =
		debugfs_create_file("set_event", 0640, trace_data->debugfs_dir,
				    trace_data, &set_event_debugfs_fops);
	if (IS_ERR_OR_NULL(trace_data->debugfs_set_event))
		return -ENOENT;

	trace_data->debugfs_available_events = debugfs_create_file(
		"available_event", 0440, trace_data->debugfs_dir, trace_data,
		&available_debugfs_fops);
	if (IS_ERR_OR_NULL(trace_data->debugfs_available_events))
		return -ENOENT;

	return 0;
}

static int __init gunyah_trace_init(void)
{
	int ret;
	size_t info_size;
	struct gunyah_trace_info *info;
	struct device_node *parent_irq_node;
	struct irq_fwspec fwspec;

	info = gunyah_get_info(GUNYAH_INFO_OWNER_ROOTVM,
			       ADDRSPACE_INFO_TRACE_INFO_ID, &info_size);
	if (IS_ERR(info)) {
		ret = PTR_ERR(info);
		/* Secure device may not support gunyah trace */
		pr_err("Gunyah trace not supported ret=%d\n", ret);
		return 0;
	}
	if (info_size != sizeof(*info)) {
		pr_err("Unexpected gunyah info size: %zu, Expected: %zu\n",
		       info_size, sizeof(*info));
		ret = -EINVAL;
		return ret;
	}

	gunyah_trace_data.size = le64_to_cpu(info->trace_size);
	gunyah_trace_data.base = ioremap_cache(le64_to_cpu(info->trace_ipa),
					       gunyah_trace_data.size);
	if (!gunyah_trace_data.base) {
		pr_err("Failed to ioremap: base=%p size=%lu\n",
		       (void *)le64_to_cpu(info->trace_ipa),
		       gunyah_trace_data.size);
		ret = -ENXIO;
		return ret;
	}
	pr_debug("Gunyan trace buf: base=%p virt=%p\n",
		 (void *)le64_to_cpu(info->trace_ipa), gunyah_trace_data.base);

	of_node_get(of_root);
	parent_irq_node = of_irq_find_parent(of_root);
	of_node_put(of_root);
	if (!parent_irq_node) {
		pr_err("Failed to find interrupt parent of gunyah trace buffer\n");
		ret = -ENODEV;
		goto unmap_mem;
	}
	gunyah_trace_data.parent_fwnode = of_node_to_fwnode(parent_irq_node);
	if (!gunyah_trace_data.parent_fwnode) {
		pr_err("Failed to find interrupt parent domain of resource manager\n");
		ret = -ENODEV;
		goto unmap_mem;
	}
	fwspec.fwnode = gunyah_trace_data.parent_fwnode;
	ret = arch_gunyah_fill_irq_fwspec_params(
		le32_to_cpu(info->trace_dbl_irq.irq), &fwspec);
	if (ret) {
		pr_err("Failed to translate interrupt: %d\n", ret);
		goto unmap_mem;
	}
	gunyah_trace_data.irq = irq_create_fwspec_mapping(&fwspec);
	if (gunyah_trace_data.irq < 0) {
		pr_err("Failed to allocate irq mapping: %d\n", ret);
		goto unmap_mem;
	}

	ret = request_irq(gunyah_trace_data.irq, gunyah_trace_buf_irq_handler,
			  IRQF_TRIGGER_RISING, "gunyah_trace_irq",
			  &gunyah_trace_data);

	if (ret) {
		pr_err("Failed to request trace buf IRQ %d ret=%d\n",
		       gunyah_trace_data.irq, ret);
		ret = -ENODEV;
		goto unmap_mem;
	}

	ret = init_trace_buf_header(&gunyah_trace_data);
	if (ret) {
		pr_err("Failed to init trace buf header ret=%d\n", ret);
		ret = -EINVAL;
		goto free_irq;
	}

	init_waitqueue_head(&gunyah_trace_data.poll_waiters);

	ret = gunyah_create_debugfs_entries(&gunyah_trace_data);
	if (ret) {
		pr_err("Failed to create debugfs entries\n");
		goto remove_debugfs;
	}
	atomic_set(&gunyah_trace_data.user_cnt, 0);

	return 0;

remove_debugfs:
	debugfs_remove(gunyah_trace_data.debugfs_dir);
	kfree(gunyah_trace_data.buf);
free_irq:
	free_irq(gunyah_trace_data.irq, &gunyah_trace_data);
unmap_mem:
	iounmap(gunyah_trace_data.base);

	return ret;
}
module_init(gunyah_trace_init);

static void __exit gunyah_trace_exit(void)
{
	debugfs_remove(gunyah_trace_data.debugfs_dir);
	free_irq(gunyah_trace_data.irq, &gunyah_trace_data);
	irq_dispose_mapping(gunyah_trace_data.irq);
	kfree(gunyah_trace_data.buf);
	iounmap(gunyah_trace_data.base);
}
module_exit(gunyah_trace_exit);

MODULE_DESCRIPTION("Gunyah Trace Driver");
MODULE_LICENSE("GPL");
