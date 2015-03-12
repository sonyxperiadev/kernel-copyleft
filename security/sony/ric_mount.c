/*
 * Sony RIC Mount
 *
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * Author: Srinavasa, Nagaraju <srinavasa.x.nagaraju@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/security.h>
#include <linux/fs.h>
#include <linux/namei.h>
#include <linux/sched.h>
#include <linux/nsproxy.h>
#include <linux/genhd.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif
#include "../../fs/mount.h"
#include "ric.h"

struct ric_part_info {
	char name[64];
	char mnt_path[256];
	unsigned long mnt_flags;
};

static struct ric_part_info *bdev_info;
static struct ric_part_info default_bdev_info[] = {
	{"system", "/system", MS_RDONLY},
	{"userdata", "/data", MS_NOSUID | MS_NODEV},
	{"cache", "/cache", MS_NOSUID | MS_NODEV}
};
static u32 num_partitions;

static int sony_ric_bdev_mount_perm(const char *volname, struct path *path,
					unsigned long flags)
{
	struct ric_part_info *part;
	int i, match = 0;
	struct path mnt_path;

	pr_debug("RIC: volname: %s\n", volname);

	if (!bdev_info)
		return 0;

	for (i = 0; i < num_partitions; i++) {
		part = &bdev_info[i];
		if (part->name && !strncmp(part->name, volname, 64)) {
			match = 1;
			break;
		}
	}

	if (match) {
		if ((flags & part->mnt_flags) != part->mnt_flags)
			return -EPERM;

		if (kern_path(part->mnt_path, LOOKUP_FOLLOW, &mnt_path))
			goto done;

		if (path->dentry != mnt_path.dentry)
			return -EPERM;
	}

done:
	return 0;
}

static int match_device_by_devt(struct device *dev, void *data)
{
	dev_t *devt = data;

	return dev->devt == *devt;
}

static int sony_ric_bdev_mount(const char *dev_name, struct path *path,
				struct inode *inode, unsigned long flags)
{
	struct device *dev;
	struct hd_struct *part;
	int ret = 0;

	dev = class_find_device(&block_class, NULL, &inode->i_rdev,
				&match_device_by_devt);
	if (!dev)
		return ret;

	part = dev_to_part(dev);
	if (!part->info) {
		pr_debug("RIC: No partition info for %s\n", dev_name);
		goto done;
	}

	ret = sony_ric_bdev_mount_perm(part->info->volname, path, flags);
done:
	put_device(dev);
	return ret;
}

static int sony_ric_cdev_mount(const char *dev_name, struct path *path,
				struct inode *inode, unsigned long flags)
{
	if (inode->i_rdev ==  MKDEV(MISC_MAJOR, FUSE_MINOR)) {
		unsigned long fuse_flags = MS_NOSUID | MS_NODEV | MS_NOEXEC;

		if ((flags & fuse_flags) != fuse_flags)
			return -EPERM;
	}

	return 0;
}

static int sony_ric_dev_mount(const char *dev_name, struct path *path,
				unsigned long flags)
{
	struct path dev_path;
	struct inode *inode;
	int ret;

	ret = kern_path(dev_name, LOOKUP_FOLLOW, &dev_path);
	if (ret)
		return 0;

	inode = dev_path.dentry->d_inode;
	if (S_ISBLK(inode->i_mode))
		ret = sony_ric_bdev_mount(dev_name, path, inode, flags);
	else if (S_ISCHR(inode->i_mode))
		ret = sony_ric_cdev_mount(dev_name, path, inode, flags);

	path_put(&dev_path);
	return ret;
}

static int sony_ric_root_mount(unsigned long flags)
{
	if (!(flags & MS_RDONLY))
		return -EPERM;
	else
		return 0;
}

static inline int mount_point_is_rootfs(struct path *path)
{
	struct file_system_type *fs_type;

	if (path && path->dentry && path->dentry->d_sb) {
		fs_type = path->dentry->d_sb->s_type;
		return !strncmp(fs_type->name, "rootfs", 6);
	}

	return 0;
}

static int sony_ric_remount(struct path *path, unsigned long flags)
{
	struct mount *mnt = real_mount(path->mnt);

	if (mnt->mnt_ns != current->nsproxy->mnt_ns)
		goto done;

	if (path->dentry != path->mnt->mnt_root)
		goto done;

	if (mount_point_is_rootfs(path))
		return sony_ric_root_mount(flags);

	if (mnt->mnt_devname)
		return sony_ric_dev_mount(mnt->mnt_devname, path, flags);
done:
	return 0;
}

int sony_ric_mount(char *dev_name, struct path *path,
			  char *type, unsigned long flags, void *data)
{
	pr_debug("RIC: mount dev_name %s, type %s flags %lu\n",
			dev_name ? dev_name : "NULL",
			type ? type : "NULL", flags);

	if (!sony_ric_enabled())
		return 0;

	/* Ignore change type mounts */
	if (flags & (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE))
		return 0;

	/* Check for remounts */
	if (flags & MS_REMOUNT)
		return sony_ric_remount(path, flags);

	/* Check for new mounts while ignoring pseudo fs mounts */
	return sony_ric_dev_mount(dev_name, path, flags);
}
EXPORT_SYMBOL(sony_ric_mount);

#ifdef CONFIG_OF
static const struct of_device_id ric_dt_info[] = {
	{ .compatible = "sony_ric" },
	{}
};

static int sony_ric_populate_bdev_info_dt(void)
{
	struct device_node *node;
	int ret = 0, i;
	u32 *mnt_flags;

	node = of_find_matching_node(NULL, ric_dt_info);
	if (!node) {
		pr_err("RIC: Sony ric device node not found\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "sony,num-partition",
			&num_partitions);
	if (ret) {
		pr_err("RIC: Failed to read num-partition node\n");
		return -EINVAL;
	}

	bdev_info = kzalloc(num_partitions * (sizeof(struct ric_part_info)),
			GFP_KERNEL);
	if (!bdev_info)
		return -ENOMEM;

	mnt_flags = kzalloc(num_partitions * (sizeof(u32)), GFP_KERNEL);
	if (!mnt_flags) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = of_property_read_u32_array(node, "sony,mount-flags",
			mnt_flags, num_partitions);
	if (ret) {
		pr_err("RIC: Failed to read partition flags\n");
		ret = -EINVAL;
		goto exit1;
	}

	for (i = 0; i < num_partitions; i++) {
		const char *part_name;
		const char *mnt_path;

		ret = of_property_read_string_index(node,
			"sony,partition-names", i, &part_name);
		if (ret) {
			pr_err("RIC: Failed to read partition name\n");
			ret = -EINVAL;
			goto exit1;
		}

		ret = of_property_read_string_index(node,
			"sony,mount_paths", i, &mnt_path);
		if (ret) {
			pr_err("RIC: Failed to read partition name\n");
			ret = -EINVAL;
			goto exit1;
		}

		strlcpy(bdev_info[i].name, part_name, 64);
		strlcpy(bdev_info[i].mnt_path, mnt_path, 256);
		bdev_info[i].mnt_flags = mnt_flags[i];
	}

	of_node_put(node);
	kfree(mnt_flags);
	return ret;

exit1:
	kfree(mnt_flags);
exit:
	kfree(bdev_info);
	bdev_info = NULL;
	of_node_put(node);
	return ret;
}
#else
static int sony_ric_populate_bdev_info_dt(void)
{
	return -ENODEV;
}
#endif

static __init int sony_ric_bdev_info_init(void)
{
	int ret;

	ret = sony_ric_populate_bdev_info_dt();
	if (ret) {
		bdev_info = default_bdev_info;
		num_partitions = ARRAY_SIZE(default_bdev_info);
	}

	return 0;
}

early_initcall(sony_ric_bdev_info_init);
