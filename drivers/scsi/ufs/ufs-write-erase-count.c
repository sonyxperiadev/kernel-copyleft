/*
 * Copyright (C) 2017 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <scsi/scsi_host.h>
#include <linux/debugfs.h>

#include "ufshcd.h"
#include "ufs_quirks.h"

static int host_num;
module_param(host_num, int, 0644);
MODULE_PARM_DESC(host_num, "UFS SCSI host number");

static struct dentry *ufs_debugfs_dir;

struct ufs_data_in_upiu {
	u8 byte0;
	u8 byte1;
	u16 byte2;
	u32 max_enhanced_area;
	u32 reserved0;
	u32 ave_enhanced_area;
	u32 max_normal_area;
	u32 reserved1;
	u32 ave_normal_area;
	u8 reserved[484];
};

#define vendor_name "TOSHIBA"
static bool is_ufs_vendor_toshiba(struct scsi_device *sdev)
{
	if (!strncmp(sdev->vendor, vendor_name, strlen(vendor_name)))
		return true;

	return false;
}

static struct scsi_device *get_scsi_device(void)
{
	struct Scsi_Host *shost;
	struct scsi_device *sdev = NULL;

	shost = scsi_host_lookup(host_num);
	if (!shost) {
		pr_err("%s: Failed to get scsi_host\n", __func__);
		goto out;
	}

	sdev = scsi_device_lookup(shost, 0, 0, 0);
	if (!sdev)
		pr_err("%s: Failed to get scsi device\n", __func__);
out:
	if (shost)
		scsi_host_put(shost);

	return sdev;
}

#define EVPD 0x1
#define PAGE_CODE 0xC0
#define CONTROL 0x00
#define MASK 0xFFFFFFFF
static int write_erase_count_show(struct seq_file *file, void *data)
{
	struct scsi_device *sdev;
	unsigned char cmd[MAX_COMMAND_SIZE];
	uint32_t max_enhanced_count = 0;
	uint32_t ave_enhanced_count = 0;
	uint32_t max_normal_count = 0;
	uint32_t ave_normal_count = 0;
	struct ufs_data_in_upiu *buf;
	int buf_sz;
	int result;

	sdev = get_scsi_device();
	if (!sdev) {
		seq_puts(file, "scsi device not found\n");
		return 0;
	}

	if (!is_ufs_vendor_toshiba(sdev)) {
		seq_puts(file, "Support only for Toshiba UFS\n");
		scsi_device_put(sdev);
		return 0;
	}

	buf_sz =  sizeof(struct ufs_data_in_upiu);
	buf = kzalloc(buf_sz, GFP_KERNEL);
	memset(cmd, 0, MAX_COMMAND_SIZE);
	cmd[0] = INQUIRY;
	cmd[1] = (0x1A << 2) | EVPD;
	cmd[2] = PAGE_CODE;
	cmd[3] = (buf_sz >> 8) & 0xFF;
	cmd[4] = buf_sz & 0xFF;
	cmd[5] = CONTROL;
	result = scsi_execute_req(sdev, cmd, DMA_FROM_DEVICE,
			(void *)buf, buf_sz, NULL, HZ, 1, NULL);
	scsi_device_put(sdev);
	if (result) {
		seq_puts(file, "cmd failed\n");
		goto exit;
	}

	if (buf->byte0 == 0 && buf->byte1 == 0xC0 &&
		ntohs(buf->byte2) == 0x01FC) {
		max_enhanced_count = ntohl(buf->max_enhanced_area);
		ave_enhanced_count = ntohl(buf->ave_enhanced_area);
		max_normal_count = ntohl(buf->max_normal_area);
		ave_normal_count = ntohl(buf->ave_normal_area);
	}

	seq_printf(file,
		"W/E count Max in Enhanced area is %d\n", max_enhanced_count);
	seq_printf(file,
		"W/E count ave in Enhanced area is %d\n", ave_enhanced_count);
	seq_printf(file,
		"W/E count Max in Normal area is %d\n", max_normal_count);
	seq_printf(file,
		"W/E count ave in Normal area is %d\n", ave_normal_count);

exit:
	kfree(buf);
	return 0;
}

static int ufsdbg_write_erase_count_open(struct inode *inode, struct file *file)
{
	return single_open(file,
			write_erase_count_show, inode->i_private);
}

static const struct file_operations ufsdbg_write_erase_count_fops = {
	.open = ufsdbg_write_erase_count_open,
	.read = seq_read,
};

static int __init ufs_write_erase_count_init(void)
{
	struct dentry *write_erase_count;

	ufs_debugfs_dir = debugfs_create_dir("ufs_debug_dir", NULL);
	if (!ufs_debugfs_dir) {
		pr_err("%s: Failed to create ufs_debug_dir\n", __func__);
		return -ENOMEM;
	}

	write_erase_count = debugfs_create_file("write_erase_count",
			S_IRUSR, ufs_debugfs_dir, NULL,
			&ufsdbg_write_erase_count_fops);
	if (!write_erase_count) {
		pr_err("%s: Failed to create write_erase_count file\n",
			__func__);
		return -ENOMEM;
	}
	return 0;
}

static void __exit ufs_write_erase_count_exit(void)
{
	debugfs_remove_recursive(ufs_debugfs_dir);
}

module_init(ufs_write_erase_count_init);
module_exit(ufs_write_erase_count_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dhamodharan Nallasivam <dhamodharan.x.nallasivam@sony.com>");
MODULE_DESCRIPTION("UFS write/erase count via debugfs for Toshiba UFS");
