/*
 * Sony RIC Mount
 *
 * Author: Srinavasa, Nagaraju <srinavasa.x.nagaraju@sonymobile.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */
/*
 * Copyright (C) 2013 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
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
	int i, ret, match = 0;
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
		if ((flags & part->mnt_flags) != part->mnt_flags) {
			pr_warning("RIC: %s mount denied, mnt_flags:0x%lx\n",
				    volname, flags);
			return -EPERM;
		}

		ret = kern_path(part->mnt_path, LOOKUP_FOLLOW, &mnt_path);
		if (ret) {
			pr_warning("RIC: kern_path() failed for %s ret = %d\n",
				    part->mnt_path, ret);
			return ret;
		}

		if (path->dentry != mnt_path.dentry) {
			path_put(&mnt_path);
			return -EPERM;
		}

		path_put(&mnt_path);
	}

	return 0;
}

static int match_device_by_devt(struct device *dev, const void *data)
{
	const dev_t *devt = data;

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

static int sony_ric_dev_mount(const char *dev_name, struct path *path,
				unsigned long flags)
{
	struct path dev_path;
	struct inode *inode;
	int ret;

	if (!dev_name || kern_path(dev_name, LOOKUP_FOLLOW, &dev_path))
		return -ENOENT;

	inode = dev_path.dentry->d_inode;
	ret = sony_ric_bdev_mount(dev_name, path, inode, flags);

	path_put(&dev_path);
	return ret;
}

static struct ric_part_info *find_matching_bdev_mountpoint(struct path *path)
{
	struct ric_part_info *part;
	struct path mnt_path;
	int i, ret, match = 0;

	for (i = 0; i < num_partitions; i++) {
		part = &bdev_info[i];
		ret = kern_path(part->mnt_path, LOOKUP_FOLLOW, &mnt_path);
		if (ret)
			continue;

		if ((path->dentry == mnt_path.dentry) ||
		     is_subdir(path->dentry, mnt_path.dentry)) {
			match = 1;
			path_put(&mnt_path);
			break;
		}

		path_put(&mnt_path);
	}

	if (match)
		return part;

	return NULL;
}

static int sony_ric_bind_mount(const char *dev_name, unsigned long flags)
{
	struct ric_part_info *part;
	struct path dev_path;

	if (!dev_name || kern_path(dev_name, LOOKUP_FOLLOW, &dev_path))
		return -ENOENT;

	part = find_matching_bdev_mountpoint(&dev_path);
	if (part) {
		pr_warning("RIC: %s bind mount denied, mnt_flags:0x%lx\n",
			    part->mnt_path, flags);
		path_put(&dev_path);
		return -EPERM;
	}

	path_put(&dev_path);
	return 0;
}

static int sony_ric_root_mount(unsigned long flags)
{
	if (!(flags & MS_RDONLY)) {
		pr_warning("RIC: rootfs remount denied, mnt_flags:0x%lx\n",
			    flags);
		return -EPERM;
	}

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
	struct ric_part_info *part;

	if (mount_point_is_rootfs(path))
		return sony_ric_root_mount(flags);

	part = find_matching_bdev_mountpoint(path);
	if (part) {
		if ((flags & part->mnt_flags) != part->mnt_flags) {
			pr_warning("RIC: %s remount denied, mnt_flags:0x%lx\n",
				    part->mnt_path, flags);
			return -EPERM;
		}
	}

	return 0;
}

int sony_ric_mount(const char *dev_name, struct path *path,
			const char *type, unsigned long flags, void *data)
{
	struct file_system_type *fstype = NULL;

	pr_debug("RIC: mount dev_name %s, type %s flags %lu\n",
			dev_name ? dev_name : "NULL",
			type ? type : "NULL", flags);

	if (!sony_ric_enabled())
		return 0;

	/* Check for remounts */
	if (flags & MS_REMOUNT)
		return sony_ric_remount(path, flags);

	/* Check for bind mounts */
	if (flags & (MS_BIND | MS_MOVE))
		return sony_ric_bind_mount(dev_name, flags);

	/* Ignore change type mounts */
	if (flags & (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE))
		return 0;

	if (type)
		fstype = get_fs_type(type);

	if (!fstype) {
		pr_err("RIC: unknown filesystem\n");
		return -ENODEV;
	}

	/* Check for new mounts while ignoring pseudo fs mounts */
	if (fstype->fs_flags & FS_REQUIRES_DEV) {
		put_filesystem(fstype);
		return sony_ric_dev_mount(dev_name, path, flags);
	}

	put_filesystem(fstype);

	return 0;
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
