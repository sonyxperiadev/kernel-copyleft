// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019, 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>

#include "synx_api.h"
#include "synx_debugfs.h"
#include "synx_util.h"
#include "synx_global.h"
#include "synx_debugfs_util.h"

#define MAX_DBG_BUF_SIZE (64 * SYNX_MAX_OBJS)
#ifdef ENABLE_DEBUGFS
#define MAX_HELP_BUF_SIZE (4096)
#define BUF_SIZE 16
#endif

struct dentry *my_direc;
u32 lower_handle_id = GLOBAL_HANDLE_STARTING_ID, upper_handle_id = GLOBAL_HANDLE_STARTING_ID;
long synx_columns = STATUS_COLUMN | ID_COLUMN | REF_CNT_COLUMN |
	NUM_CHILD_COLUMN | SUBSCRIBERS_COLUMN | WAITERS_COLUMN | PARENTS_COLUMN | GLOBAL_SHARED_MEM;
EXPORT_SYMBOL(synx_columns);

int synx_debug = SYNX_ERR | SYNX_WARN |
	SYNX_INFO;
EXPORT_SYMBOL(synx_debug);

void populate_bound_rows(
	struct synx_coredata *row, char *cur, char *end)
{
	int j;

	for (j = 0; j < row->num_bound_synxs; j++)
		SYNX_CONSOLE_LOG(cur, end, "\n\tID: %d",
		row->bound_synxs[j].external_desc.id);
}

static ssize_t synx_table_read(struct file *file,
		char *buf,
		size_t count,
		loff_t *ppos)
{
	struct synx_device *dev = file->private_data;
	struct error_node *err_node, *err_node_tmp;
	char *dbuf, *cur, *end;
	ssize_t len = 0;

	dbuf = kzalloc(MAX_DBG_BUF_SIZE, GFP_KERNEL);
	if (!dbuf)
		return -ENOMEM;

	cur = dbuf;
	end = cur + MAX_DBG_BUF_SIZE;
#ifdef ENABLE_DEBUGFS
	SYNX_CONSOLE_LOG(cur, end, "\n\tHandle ID start value : %d", lower_handle_id);
	SYNX_CONSOLE_LOG(cur, end, "\n\tHandle ID end value : %d\n", upper_handle_id);

	if (synx_columns & GLOBAL_HASHTABLE)
		synx_debugfs_util_print_hash_table(&cur, &end, true);
	if (synx_columns & LOCAL_HASHTABLE)
		synx_debugfs_util_print_hash_table(&cur, &end, false);
	if (synx_columns & CLIENT_HASHTABLE)
		synx_debugfs_util_print_client_table(&cur, &end);
	if (synx_columns & GLOBAL_SHARED_MEM)
		synx_debugfs_util_print_global_shared_memory(&cur, &end);
	if (synx_columns & DMA_FENCE_MAP)
		synx_debugfs_util_print_dma_fence(&cur, &end);
#endif

	if (synx_columns & ERROR_CODES && !list_empty(&dev->error_list)) {
		SYNX_CONSOLE_LOG(cur, end, "\nError(s): ");
		mutex_lock(&dev->error_lock);
		list_for_each_entry_safe(
			 err_node, err_node_tmp,
			 &dev->error_list, node) {
			SYNX_CONSOLE_LOG(cur, end, "\n\tTime: %s - ", err_node->timestamp);
			SYNX_CONSOLE_LOG(cur, end, "ID: %d - ", err_node->h_synx);
			SYNX_CONSOLE_LOG(cur, end, "Code: %d", err_node->error_code);
			list_del(&err_node->node);
			kfree(err_node);
		}
		mutex_unlock(&dev->error_lock);
	}
	len = simple_read_from_buffer(buf, count, ppos,
		dbuf, cur - dbuf);
	kfree(dbuf);
	return len;
}

#ifdef ENABLE_DEBUGFS
static ssize_t synx_table_write(struct file *file,
		const char __user *buf,
		size_t count,
		loff_t *ppos)
{
	u32 stat = -1;
	u32 i = 0, base = 10, num = 0;
	bool invalid_val = false;
	char *kbuffer = kzalloc(BUF_SIZE, GFP_KERNEL);

	if (!kbuffer)
		return -ENOMEM;
	stat = copy_from_user(kbuffer, buf, BUF_SIZE);
	if (stat != 0) {
		kfree(kbuffer);
		return -EFAULT;
	}
	if (kbuffer[i] == '0' && (kbuffer[i+1] == 'x' || kbuffer[i+1] == 'X')) {
		base = 16;
		i += 2;
	}
	for ( ; (i < BUF_SIZE / 2 && kbuffer[i] != '-' && kbuffer[i] != '\n'); i++)
		SYNX_READ_CHAR(kbuffer, num, base, i);
	if (!invalid_val)
		lower_handle_id = num;

	if (kbuffer[i] == '-') {
		num = 0;
		i++;
		for ( ; i < BUF_SIZE && kbuffer[i] != '\n'; i++)
			SYNX_READ_CHAR(kbuffer, num, base, i);
		if (!invalid_val)
			upper_handle_id = num;
	} else if (kbuffer[i] == '\n')
		upper_handle_id = lower_handle_id;
	kfree(kbuffer);

	return count;
}
#endif

static const struct file_operations synx_table_fops = {
	.owner = THIS_MODULE,
	.read = synx_table_read,
#ifdef ENABLE_DEBUGFS
	.write = synx_table_write,
#endif
	.open = simple_open,
};

#ifdef ENABLE_DEBUGFS
static ssize_t synx_help_read(struct file *file,
		char *buf,
		size_t count,
		loff_t *ppos)
{
	char *dbuf, *cur, *end;
	ssize_t len = 0;

	dbuf = kzalloc(MAX_HELP_BUF_SIZE, GFP_KERNEL);
	if (!dbuf)
		return -ENOMEM;

	cur = dbuf;
	end = cur + MAX_HELP_BUF_SIZE;
	synx_debugfs_util_load_help_content(&cur, &end);
	len = simple_read_from_buffer(buf, count, ppos, dbuf, cur - dbuf);
	kfree(dbuf);
	return len;
}
static const struct file_operations synx_help_fops = {
	.owner = THIS_MODULE,
	.read = synx_help_read,
};
#endif
struct dentry *synx_init_debugfs_dir(struct synx_device *dev)
{
	struct dentry *dir = NULL;
	dir = debugfs_create_dir("synx_debug", NULL);
	if (!dir) {
		dprintk(SYNX_ERR, "Failed to create debugfs for synx\n");
		return NULL;
	}
	debugfs_create_u32("debug_level", 0644, dir, &synx_debug);
	debugfs_create_ulong("column_level", 0644, dir, &synx_columns);

	if (!debugfs_create_file("synx_table",
		0644, dir, dev, &synx_table_fops)) {
		dprintk(SYNX_ERR, "Failed to create debugfs file for synx\n");
		return NULL;
	}
#ifdef ENABLE_DEBUGFS
	if (!debugfs_create_file("help",
		0444, dir, dev, &synx_help_fops)) {
		dprintk(SYNX_ERR, "Failed to create debugfs help file for synx\n");
		return NULL;
	}
#endif
	return dir;
}

void synx_remove_debugfs_dir(struct synx_device *dev)
{
	debugfs_remove_recursive(dev->debugfs_root);
}
