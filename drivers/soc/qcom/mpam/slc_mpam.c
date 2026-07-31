// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
 #define pr_fmt(fmt) "mpam_slc: " fmt

#include <linux/io.h>
#include <linux/of.h>
#include <linux/configfs.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <soc/qcom/mpam.h>
#include <soc/qcom/mpam_msc.h>
#include <soc/qcom/mpam_slc.h>

#define NO_PARTID		(-1)
#define MAX_RETRY_CNT		500

struct slc_mpam_item {
	struct config_group group;
	int part_id;
	int client_id;
	bool cap_mon_support;
	bool cap_mon_enabled;
	bool miss_mon_support;
	bool miss_mon_enabled;
	bool fe_mon_support;
	bool fe_mon_enabled;
	bool be_mon_support;
	bool be_mon_enabled;
};

static uint32_t slc_firmware_ver;
static struct config_group *root_group;

static inline struct slc_mpam_item *get_pm_item(
					   struct config_item *item)
{
	return container_of(to_config_group(item),
				struct slc_mpam_item, group);
}

static inline int set_msc_query(struct msc_query *query,
					   struct slc_mpam_item *pm_item)
{
	struct qcom_mpam_msc *qcom_mpam_msc;

	qcom_mpam_msc = qcom_msc_lookup(SLC);
	if (!qcom_mpam_msc)
		return -ENODEV;

	query->qcom_msc_id.qcom_msc_type =
		qcom_mpam_msc->qcom_msc_id.qcom_msc_type;
	query->qcom_msc_id.qcom_msc_class =
		qcom_mpam_msc->qcom_msc_id.qcom_msc_class;
	query->qcom_msc_id.idx =
		qcom_mpam_msc->qcom_msc_id.idx;

	query->client_id = pm_item->client_id;
	query->part_id = pm_item->part_id;

	return 0;
}

static ssize_t slc_mpam_schemata_show(struct config_item *item,
		char *page)
{
	int ret;
	struct msc_query query;
	struct qcom_slc_gear_val gear_config;

	set_msc_query(&query, get_pm_item(item));

	ret = msc_system_get_partition(SLC, &query, &gear_config);
	if (ret)
		return scnprintf(page, PAGE_SIZE,
			"failed to get schemata %d\n", ret);

	return scnprintf(page, PAGE_SIZE, "gear=%d\n",
		gear_config.gear_val);
}

static ssize_t slc_mpam_schemata_store(struct config_item *item,
		const char *page, size_t count)
{
	int ret, input;
	char *token, *param_name;
	struct msc_query query;
	struct qcom_slc_gear_val gear_config;

	set_msc_query(&query, get_pm_item(item));

	while ((token = strsep((char **)&page, ",")) != NULL) {
		param_name = strsep(&token, "=");
		if (param_name == NULL || token == NULL)
			continue;
		if (kstrtouint(token, 0, &input) < 0) {
			pr_err("invalid argument for %s\n", param_name);
			return -EINVAL;
		}

		if (!strcmp("gear", param_name))
			gear_config.gear_val = input;
	}

	ret = msc_system_set_partition(SLC, &query, &gear_config);
	if (ret) {
		pr_err("set slc cache partition failed, ret=%d\n", ret);
		return -EBUSY;
	}

	return count;
}
CONFIGFS_ATTR(slc_mpam_, schemata);

static ssize_t slc_mpam_monitor_schemata_show(struct config_item *item,
		char *page)
{
	size_t len = 0;
	struct slc_mpam_item *pm_item = get_pm_item(item);

	if (pm_item->cap_mon_support)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"cap=%d,", pm_item->cap_mon_enabled);
	if (pm_item->miss_mon_support)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"miss=%d,", pm_item->miss_mon_enabled);
	if (pm_item->fe_mon_support)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"fe=%d,", pm_item->fe_mon_enabled);
	if (pm_item->be_mon_support)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"be=%d,", pm_item->be_mon_enabled);
	if (len)
		len += scnprintf(page + len - 1, PAGE_SIZE - len, "\n");

	return len;
}

static ssize_t slc_mpam_monitor_schemata_store(struct config_item *item,
		const char *page, size_t count)
{
	int ret, need_set;
	bool input;
	char *token, *param_name;
	struct msc_query query;
	struct slc_mon_config_val mon_cfg_val;
	struct slc_mpam_item *pm_item = get_pm_item(item);

	set_msc_query(&query, pm_item);

	while ((token = strsep((char **)&page, ",")) != NULL) {
		need_set = 0;
		param_name = strsep(&token, "=");
		if (param_name == NULL || token == NULL)
			continue;

		if (kstrtobool(token, &input) < 0) {
			pr_err("invalid argument for %s\n", param_name);
			return -EINVAL;
		}

		if (!strcmp("cap", param_name) && pm_item->cap_mon_support &&
				(pm_item->cap_mon_enabled != input)) {
			mon_cfg_val.slc_mon_function = CACHE_CAPACITY_CONFIG;
			mon_cfg_val.enable = input;
			need_set = 1;
		} else if (!strcmp("miss", param_name) && pm_item->miss_mon_support &&
				(pm_item->miss_mon_enabled != input)) {
			mon_cfg_val.slc_mon_function = CACHE_READ_MISS_CONFIG;
			mon_cfg_val.enable = input;
			need_set = 1;
		} else if (!strcmp("fe", param_name) && pm_item->fe_mon_support  &&
				(pm_item->fe_mon_enabled != input)) {
			mon_cfg_val.slc_mon_function = CACHE_FE_MON_CONFIG;
			mon_cfg_val.enable = input;
			need_set = 1;
		} else if (!strcmp("be", param_name)  && pm_item->be_mon_support  &&
				(pm_item->be_mon_enabled != input)) {
			mon_cfg_val.slc_mon_function = CACHE_BE_MON_CONFIG;
			mon_cfg_val.enable = input;
			need_set = 1;
		}

		if (need_set) {
			ret = msc_system_mon_config(SLC, &query, &mon_cfg_val);
			if (ret) {
				pr_err("%s monitor %s failed %d\n", param_name,
					(input) ? "enable" : "disable", ret);
				return -EBUSY;
			}

			if (mon_cfg_val.slc_mon_function == CACHE_CAPACITY_CONFIG)
				pm_item->cap_mon_enabled = input;
			else if (mon_cfg_val.slc_mon_function == CACHE_READ_MISS_CONFIG)
				pm_item->miss_mon_enabled = input;
			else if (mon_cfg_val.slc_mon_function == CACHE_FE_MON_CONFIG)
				pm_item->fe_mon_enabled = input;
			else if (mon_cfg_val.slc_mon_function == CACHE_BE_MON_CONFIG)
				pm_item->be_mon_enabled = input;
		}
	}

	return count;
}
CONFIGFS_ATTR(slc_mpam_, monitor_schemata);

static ssize_t slc_mpam_monitor_data_show(struct config_item *item,
		char *page)
{
	ssize_t len = 0;
	struct msc_query query;
	union mon_values mon_data;
	struct slc_mpam_item *pm_item = get_pm_item(item);

	set_msc_query(&query, pm_item);

	if (!pm_item->cap_mon_enabled && !pm_item->miss_mon_enabled &&
			!pm_item->fe_mon_enabled && !pm_item->be_mon_enabled)
		return 0;

	msc_system_mon_stats_read(SLC, &query, &mon_data);

	len = scnprintf(page, PAGE_SIZE,
			"timestamp=%llu,", mon_data.mon_stats.last_capture_time);
	if (pm_item->cap_mon_enabled)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"cap_cnt=%u,", mon_data.mon_stats.num_cache_lines);
	if (pm_item->miss_mon_enabled)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"miss_cnt=%llu,", mon_data.mon_stats.num_rd_misses);
	if (pm_item->fe_mon_enabled)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"fe_bytes=%llu,", mon_data.mon_stats.slc_fe_bytes);
	if (pm_item->be_mon_enabled)
		len += scnprintf(page + len, PAGE_SIZE - len,
			"be_bytes=%llu,", mon_data.mon_stats.slc_be_bytes);

	len -= 1;
	len += scnprintf(page + len, PAGE_SIZE - len, "\n");

	return len;
}
CONFIGFS_ATTR_RO(slc_mpam_, monitor_data);

static ssize_t slc_mpam_available_gear_show(struct config_item *item,
		char *page)
{
	int i, ret, gear_num;
	ssize_t len = 0;
	struct msc_query query;
	union slc_partid_capability_def partid_cap = { 0 };

	set_msc_query(&query, get_pm_item(item));

	ret = msc_system_get_device_capability(SLC, &query, &partid_cap);
	if (ret)
		return scnprintf(page, PAGE_SIZE,
			"failed to get available gear %d\n", ret);

	if (slc_firmware_ver == SLC_MPAM_VERSION_0) {
		for (i = 0; i < partid_cap.v0_cap.num_gears; i++) {
			gear_num = partid_cap.v0_cap.part_id_gears[i];
			len += scnprintf(page + len, PAGE_SIZE - len,
				"%d - %s\n", gear_num, gear_index[gear_num]);
		}
	} else {
		for (gear_num = 0, i = 0; gear_num < partid_cap.v1_cap.num_gears &&
				i < sizeof(partid_cap.v1_cap.cap_cfg.gear_flds_bitmap) * 8; i++) {
			if (((1 << i) & partid_cap.v1_cap.cap_cfg.gear_flds_bitmap) == 0)
				continue;

			len += scnprintf(page + len, PAGE_SIZE - len,
					"%d - %d\n", gear_num++,
					i * partid_cap.v1_cap.cap_cfg.slc_bitfield_capacity);
		}
	}

	return len;
}
CONFIGFS_ATTR_RO(slc_mpam_, available_gear);

static struct configfs_attribute *slc_mpam_attrs[] = {
	&slc_mpam_attr_schemata,
	&slc_mpam_attr_monitor_data,
	&slc_mpam_attr_monitor_schemata,
	&slc_mpam_attr_available_gear,
	NULL,
};

static const struct config_item_type slc_mpam_item_type = {
	.ct_attrs	= slc_mpam_attrs,
};

static ssize_t slc_mpam_monitors_data_show(struct config_item *item,
		char *page)
{
	struct msc_query query;

	set_msc_query(&query, get_pm_item(item));

	return msc_system_mon_read_all(SLC, &query, page);
}
CONFIGFS_ATTR_RO(slc_mpam_, monitors_data);

static struct configfs_attribute *slc_mpam_root_attrs[] = {
	&slc_mpam_attr_monitors_data,
	NULL,
};

static const struct config_item_type slc_mpam_root_type = {
	.ct_attrs	= slc_mpam_root_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct configfs_attribute *slc_mpam_client_monitor_attrs[] = {
	&slc_mpam_attr_monitor_schemata,
	&slc_mpam_attr_monitor_data,
	NULL,
};

static const struct config_item_type slc_mpam_client_monitor_type = {
	.ct_attrs	= slc_mpam_client_monitor_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct slc_mpam_item *slc_mpam_make_group(
		struct device *dev, const char *name,
		const struct config_item_type *item_type)
{
	struct slc_mpam_item *item;

	item = devm_kzalloc(dev, sizeof(struct slc_mpam_item), GFP_KERNEL);
	if (!item)
		return ERR_PTR(-ENOMEM);

	config_group_init_type_name(&item->group, name,
				   item_type);

	return item;
}

static struct config_group *create_config_node(
		const char *name, struct device *dev,
		int client_id, int part_id,
		struct config_group *parent_group,
		const struct config_item_type *item_type)
{
	int ret;
	struct msc_query query;
	struct slc_mpam_item *new_item;
	struct device_node *np = dev->of_node;
	union slc_partid_capability_def partid_cap = { 0 };

	new_item = slc_mpam_make_group(dev, name, item_type);
	if (IS_ERR(new_item)) {
		pr_err("Error create group %s\n", name);
		return ERR_PTR(-ENOMEM);
	}

	new_item->client_id = client_id;
	new_item->part_id = part_id;
	if (slc_firmware_ver != SLC_MPAM_VERSION_0) {
		if (part_id != NO_PARTID) {
			set_msc_query(&query, new_item);
			msc_system_get_device_capability(SLC, &query, &partid_cap);
			if (partid_cap.v1_cap.mon_support & (1 << cap_mon_support))
				new_item->cap_mon_support = true;
			if (partid_cap.v1_cap.mon_support & (1 << read_miss_mon_support))
				new_item->miss_mon_support = true;
			if (partid_cap.v1_cap.mon_support & (1 << fe_mon_support))
				new_item->fe_mon_support = true;
			if (partid_cap.v1_cap.mon_support & (1 << be_mon_support))
				new_item->be_mon_support = true;
		}
	} else {
		new_item->cap_mon_support = true;
		new_item->miss_mon_support = true;
	}

	if (client_id == 0 && of_property_read_bool(np, "qcom,client-level-mon")) {
		if (part_id == NO_PARTID) {
			new_item->part_id = 0;
			new_item->fe_mon_support = true;
			new_item->be_mon_support = true;
		} else {
			new_item->fe_mon_support = false;
			new_item->be_mon_support = false;
		}
	}

	ret = configfs_register_group(parent_group, &new_item->group);
	if (ret) {
		pr_err("Error register group %s\n", name);
		return ERR_PTR(-EBUSY);
	}

	return &new_item->group;
}

static int slc_config_fs_register(struct  device *dev)
{
	int clientid, partid;
	char buf[CONFIGFS_ITEM_NAME_LEN];
	const char *msc_name_dt;
	struct qcom_mpam_msc *qcom_msc;
	struct config_group *p_group, *sub_group;
	struct device_node *np = dev->of_node;
	struct qcom_slc_capability *slc_capability;
	struct slc_client_capability *slc_client_cap;

	p_group = platform_mpam_get_root_group();
	if (!p_group)
		return -EINVAL;

	of_property_read_string(np, "qcom,msc-name", &msc_name_dt);

	root_group = configfs_register_default_group(p_group, msc_name_dt, &slc_mpam_root_type);
	if (IS_ERR(root_group)) {
		dev_err(dev, "Error register group %s\n", msc_name_dt);
		return PTR_ERR(root_group);
	}

	qcom_msc = qcom_msc_lookup(SLC);
	if (!qcom_msc || qcom_msc->qcom_msc_id.qcom_msc_type != SLC)
		return -EINVAL;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	for (clientid = 0; clientid < slc_capability->num_clients; clientid++) {
		slc_client_cap = &(slc_capability->slc_client_cap[clientid]);

		if (slc_client_cap->enabled == false)
			continue;

		if (slc_client_cap->client_info.num_part_id > 1) {
			sub_group = create_config_node(slc_client_cap->client_name,
				dev, clientid, NO_PARTID, root_group,
				&slc_mpam_client_monitor_type);
			if (IS_ERR(sub_group))
				break;
			for (partid = 0; partid < slc_client_cap->client_info.num_part_id;
					partid++) {
				snprintf(buf, sizeof(buf), "partid%d", partid);
				create_config_node(buf, dev, clientid,
						partid, sub_group, &slc_mpam_item_type);
			}
		} else {
			create_config_node(slc_client_cap->client_name, dev,
					clientid, 0, root_group, &slc_mpam_item_type);
		}
	}

	return 0;
}

static int slc_mpam_probe(struct platform_device *pdev)
{
	int client_cnt;
	struct qcom_mpam_msc *qcom_msc;
	struct config_group *p_group;
	struct qcom_slc_capability *slc_capability;
	struct slc_client_capability *slc_client_cap;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *node;
	int clientid, ret = -EINVAL;
	const char *msc_name_dt;

	p_group = platform_mpam_get_root_group();
	if (!p_group)
		return -EPROBE_DEFER;

	/* Wait for mpam_msc_slc probe to finish */
	qcom_msc = qcom_msc_lookup(SLC);
	if (!qcom_msc ||
		qcom_msc->mpam_available != MPAM_MONITRS_AVAILABLE)
		return -EPROBE_DEFER;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	msc_system_get_mpam_version(SLC, &slc_firmware_ver);
	if (slc_firmware_ver != SLC_MPAM_VERSION_0)
		return slc_config_fs_register(&pdev->dev);

	client_cnt = of_get_child_count(np);
	if (!client_cnt) {
		dev_err(&pdev->dev, "No client found\n");
		return -ENODEV;
	}

	clientid = 0;
	for_each_child_of_node(np, node) {
		slc_client_cap = &(slc_capability->slc_client_cap[clientid]);
		ret = of_property_read_u32(node, "qcom,client-id", &clientid);
		of_property_read_string(node, "qcom,client-name", &msc_name_dt);
		if (ret || IS_ERR_OR_NULL(msc_name_dt))
			continue;

		slc_client_cap->client_name = devm_kzalloc(&pdev->dev, CLIENT_NAME_LEN, GFP_KERNEL);
		if (slc_client_cap->client_name == NULL)
			return -ENOMEM;

		strscpy(slc_client_cap->client_name, msc_name_dt, strlen(msc_name_dt) + 1);
		clientid++;
	}

	slc_config_fs_register(&pdev->dev);
	return 0;
}

static void slc_mpam_remove(struct platform_device *pdev)
{
	configfs_unregister_group(root_group);
	kfree(root_group);
}

static const struct of_device_id slc_mpam_table[] = {
	{ .compatible = "qcom,mpam-slc" },
	{}
};
MODULE_DEVICE_TABLE(of, slc_mpam_table);

static struct platform_driver slc_mpam_driver = {
	.driver = {
		.name = "mpam-slc",
		.of_match_table = slc_mpam_table,
		.suppress_bind_attrs = true,
	},
	.probe = slc_mpam_probe,
	.remove = slc_mpam_remove,
};

module_platform_driver(slc_mpam_driver);

MODULE_SOFTDEP("pre: mpam");
MODULE_DESCRIPTION("QCOM SLC MPAM driver");
MODULE_LICENSE("GPL");
