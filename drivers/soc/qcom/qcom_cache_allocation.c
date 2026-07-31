// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/units.h>
#include <soc/qcom/mpam.h>
#include <soc/qcom/mpam_msc.h>

#define CREATE_TRACE_POINTS
#include "trace-cache_allocation.h"

#define CONFIG_QTI_GPU_RESOURCE_ENABLED 0
#define K_F 1831  /* 19200000/1024/1024 * 100 */
#define SAMPLING_MS 10
#define RETRY_COUNT 3
#define boost_dection(ptr, idx) ({ \
	(ptr->cpu_freq_curr[idx] - ptr->cpu_freq_prev[idx]) \
		> ptr->config->freq_cfg[idx].cpu_boost_thresh ? 1 : 0; })

#define cache_get(ptr, member) ({	\
	(mpam_version.version >= 0x10002) ?	\
	ptr->V2.member : ptr->V1.member; })

struct freq_mon_config {
	int cluster_id;
	u32 cpu_boost_thresh;
	u32 cpu_restore_thresh;
	u32 cpu_instant_thresh[2];
	u32 gpu_instant_thresh;
};

struct client_ratio_gear {
	int cpu_gear;
	int gpu_gear;
};

struct bw_ratio_config {
	u64 bw_mon_ratio_thresh[4];
	struct client_ratio_gear bw_gear[5];
};

struct soc_config {
	struct freq_mon_config *freq_cfg;
	struct bw_ratio_config *bw_ratio_cfg;
};

struct bw_monitor_data {
	u64 last_timestamp;
	u64 last_byte_cnt;
	u64 timestamp;
	u64 byte_cnt;
};

struct cache_allocation {
	struct devfreq *df;
	struct device_node *gpu_np;
	struct soc_config *config;
	u32 *cpu_freq_prev;
	u32 *cpu_freq_curr;
	u32 gpu_freq_prev;
	u32 gpu_freq_curr;
	int freq_mon_status;
	bool enable_monitor;
	bool running_flag;
	int cluster_num;
	int *client_input;
	int *client_input_ext;
	int sampling_time_ms;
	struct delayed_work work;
	struct mutex lock;
	void __iomem *mon_base;
	struct bw_monitor_data *bw_monitor_data;
	int bw_mon_ratio_status;
	int win_count_config;
	int win_count;
	bool win_active;
	int current_governor;
};

struct slc_monitor_cfg {
	int msc_id;
	int client_id;
	int monitor_index;
};

enum {
	GEAR_BYPASS = 0,
	GEAR_LVL_1,
	GEAR_LVL_2,
	GEAR_LVL_3,
	GEAR_LVL_4,
	GEAR_LVL_5,
	GEAR_LVL_6,
	GEAR_LVL_7,
	GEAR_LVL_8,
	GEAR_LVL_9,
	GEAR_LVL_10,
	GEAR_LVL_11,
	GEAR_LVL_12,
	GEAR_LVL_13,
};

enum {
	APPS = 0,
	GPU,
	CLIENT_MAX,
};

static struct msc_query msc_queries[] = {
	[APPS] = {
		.qcom_msc_id = {0, 0, 2},
		.part_id = 0,
		.client_id = 0,
	},
	[GPU] = {
		.qcom_msc_id = {0, 0, 2},
		.part_id = 0,
		.client_id = 1,
	},
};

static struct slc_monitor_cfg slc_config[] = {
	[0] = {
		.msc_id = 3,
		.client_id = 0x1,
		.monitor_index = 0,
	},
	[1] = {
		.msc_id = 3,
		.client_id = 0x2,
		.monitor_index = 1,
	},
	[2] = {
		.msc_id = 3,
		.client_id = 0x10,
		.monitor_index = 2,
	},
};

static char *gov_list[] = {
	"FREQ_MON_GOV",
	"BW_MON_RATIO_GOV",
};

#define QCOM_CACHE_ALLOCATION_ATTR_STORE(name)							\
static ssize_t name##_store(struct device *dev,						\
		struct device_attribute *attr, const char *buf, size_t count)			\
{												\
	struct cache_allocation *pd = dev_get_drvdata(dev);					\
	int val;										\
												\
	if (kstrtoint(buf, 0, &val))								\
		return -EINVAL;									\
												\
	if (val < 0) {										\
		pr_err("input error\n");							\
		return -EINVAL;									\
	}											\
												\
	mutex_lock(&pd->lock);									\
	if (pd->enable_monitor && pd->running_flag) {						\
		pd->running_flag = false;							\
		cancel_delayed_work_sync(&pd->work);						\
	}											\
												\
	pd->name = val;										\
	if (pd->enable_monitor && !pd->running_flag) {						\
		pd->running_flag = true;							\
		schedule_delayed_work(&pd->work, msecs_to_jiffies(pd->sampling_time_ms));	\
	}											\
	mutex_unlock(&pd->lock);								\
	return count;										\
}												\

struct mpam_ver_ret mpam_version;

static void cpu_gpu_freq_update(struct cache_allocation *pdev)
{
	int i;

	for (i = 0; i < pdev->cluster_num; i++) {
		pdev->cpu_freq_prev[i] = pdev->cpu_freq_curr[i];
		pdev->cpu_freq_curr[i] =
				cpufreq_quick_get(pdev->config->freq_cfg[i].cluster_id);
		trace_cache_alloc_cpu_update(i, pdev->cpu_freq_prev[i],
							pdev->cpu_freq_curr[i]);
	}

#if IS_ENABLED(CONFIG_QTI_GPU_RESOURCE_ENABLED)
	struct devfreq *devfreq = pdev->df;
	unsigned long cur_freq = 0;
	pdev->gpu_freq_prev = pdev->gpu_freq_curr;

	mutex_lock(&devfreq->lock);
	devfreq->profile->get_cur_freq(devfreq->dev.parent, &cur_freq);
	mutex_unlock(&devfreq->lock);

	pdev->gpu_freq_curr = DIV_ROUND_UP(cur_freq, HZ_PER_MHZ);
	trace_cache_alloc_gpu_update(pdev->gpu_freq_prev, pdev->gpu_freq_curr);
#endif
}

static int save_gear_for_client(struct cache_allocation *pdev)
{
	int ret = 0;

	ret = msc_system_get_partition(SLC, &msc_queries[APPS],
					&pdev->client_input_ext[APPS]);
	if (ret < 0) {
		pr_err("fail to get cpu cache partition, ret=%d\n", ret);
		return ret;
	}

	ret = msc_system_get_partition(SLC, &msc_queries[GPU],
					&pdev->client_input_ext[GPU]);
	if (ret < 0) {
		pr_err("fail to set gpu cache partition, ret=%d\n", ret);
		return ret;
	}

	return ret;
}

static int cache_allocation_configure(struct cache_allocation *pdev)
{
	int ret = 0, cpu_gear_val;

	ret = msc_system_get_partition(SLC, &msc_queries[APPS], &cpu_gear_val);
	if (ret < 0) {
		pr_err("fail to get cpu cache partition, ret=%d\n", ret);
		return ret;
	}

	ret = msc_system_set_partition(SLC, &msc_queries[APPS], &pdev->client_input[APPS]);
	if (ret < 0) {
		if (ret == -EREMOTEIO) {
			for (int i = 0; i < RETRY_COUNT; i++) {
				ret = msc_system_set_partition(SLC, &msc_queries[APPS],
							&pdev->client_input[APPS]);
				if (ret < 0)
					pr_warn("set APPS Retry %d failed, ret=%d\n",
									i + 1, ret);
				else
					break;
			}
		} else
			pr_err("fail to set cpu cache partition, ret=%d\n", ret);
		return ret;
	}

	ret = msc_system_set_partition(SLC, &msc_queries[GPU], &pdev->client_input[GPU]);
	if (ret < 0) {
		if (ret == -EREMOTEIO) {
			for (int i = 0; i < RETRY_COUNT; i++) {
				ret = msc_system_set_partition(SLC, &msc_queries[GPU],
							&pdev->client_input[GPU]);
				if (ret < 0)
					pr_warn("set GPU Retry %d failed, ret=%d\n",
									i + 1, ret);
				else
					break;
			}
		} else {
			msc_system_set_partition(SLC, &msc_queries[APPS], &cpu_gear_val);
			pr_err("fail to set gpu cache partition, ret=%d\n", ret);
		}
		return ret;
	}

	trace_cache_alloc_config_update(pdev->client_input[APPS], pdev->client_input[GPU]);
	return ret;
}

static void module_bw_monitor(struct cache_allocation *pdev,
				struct slc_monitor_cfg *cfg, u64 *mbps, int idx)
{
	int index;
	union platform_monitor_value data, *pdata;

	index = cfg->monitor_index;
	pdata = pdev->mon_base + index * ((mpam_version.version >= 0x10002) ?
				sizeof(data.V2) : sizeof(data.V1));

	pdev->bw_monitor_data[idx].last_timestamp = pdev->bw_monitor_data[idx].timestamp;
	pdev->bw_monitor_data[idx].last_byte_cnt = pdev->bw_monitor_data[idx].byte_cnt;
	pdev->bw_monitor_data[idx].timestamp = cache_get(pdata, last_capture_time);
	pdev->bw_monitor_data[idx].byte_cnt = cache_get(pdata, bwmon_byte_count);

	trace_cache_alloc_timestamp(pdev->bw_monitor_data[idx].timestamp, idx);
	trace_cache_alloc_byte_cnt(pdev->bw_monitor_data[idx].byte_cnt, idx);

	if (pdev->bw_monitor_data[idx].timestamp ==
			pdev->bw_monitor_data[idx].last_timestamp) {
		*mbps = 0;
		return;
	}

	*mbps = (pdev->bw_monitor_data[idx].byte_cnt -
				pdev->bw_monitor_data[idx].last_byte_cnt) * K_F /
				((pdev->bw_monitor_data[idx].timestamp -
					pdev->bw_monitor_data[idx].last_timestamp) * 100);
}

static void platform_bw_thresh_monitor(struct cache_allocation *pdev, u64 *ratio)
{
	u64 m_mbps = 0;
	u64 l_mbps = 0;
	u64 cpu_mbps = 0;
	u64 gpu_mbps = 0;

	pdev->win_count++;
	if (pdev->win_count_config == pdev->win_count) {
		module_bw_monitor(pdev, &slc_config[0], &m_mbps, 0);
		module_bw_monitor(pdev, &slc_config[1], &l_mbps, 1);
		cpu_mbps = m_mbps + l_mbps;
		module_bw_monitor(pdev, &slc_config[2], &gpu_mbps, 2);

		if (gpu_mbps == 0)
			*ratio = pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[3] + 1;
		else
			*ratio = 100 * cpu_mbps / gpu_mbps;
		trace_cache_alloc_bw_ratio(m_mbps, l_mbps, cpu_mbps, gpu_mbps, *ratio);
		pdev->win_count = 0;
		pdev->win_active = true;

		return;
	}

	pdev->win_active = false;
}

static void freq_mon_prefer_cpu_gov(struct cache_allocation *pdev)
{
	int ret = 0;

	if (pdev->freq_mon_status != 1 &&
			(boost_dection(pdev, 0) || boost_dection(pdev, 1))) {
		pdev->freq_mon_status = 1;
	} else if (pdev->freq_mon_status != 2 &&
			((pdev->cpu_freq_curr[0] >= pdev->cpu_freq_prev[0]) ||
			(pdev->cpu_freq_curr[1] >= pdev->cpu_freq_prev[1]))) {
		pdev->freq_mon_status = 2;
		pdev->client_input[APPS] = GEAR_LVL_10;
		pdev->client_input[GPU] = GEAR_LVL_6;
		ret = cache_allocation_configure(pdev);
		if (ret < 0) {
			WARN_ON(1);
			return;
		}
	} else if (pdev->freq_mon_status != 3 &&
			((pdev->cpu_freq_curr[0] <=
					pdev->config->freq_cfg[0].cpu_restore_thresh) &&
			(pdev->cpu_freq_curr[1] <=
					pdev->config->freq_cfg[1].cpu_restore_thresh))) {
		pdev->freq_mon_status = 3;
		pdev->client_input[APPS] = GEAR_LVL_10;
		pdev->client_input[GPU] = GEAR_LVL_11;
		ret = cache_allocation_configure(pdev);
		if (ret < 0) {
			WARN_ON(1);
			return;
		}
	} else if (pdev->freq_mon_status != 4 &&
			pdev->cpu_freq_curr[0] < pdev->cpu_freq_prev[0] &&
			pdev->cpu_freq_curr[1] < pdev->cpu_freq_prev[1]) {
		pdev->freq_mon_status = 4;
	}
}

static void freq_mon_prefer_gpu_gov(struct cache_allocation *pdev)
{
	int ret = 0;

	if (pdev->freq_mon_status != 5 &&
	   (pdev->cpu_freq_curr[0] < pdev->config->freq_cfg[0].cpu_instant_thresh[0] ||
	   pdev->cpu_freq_curr[1] < pdev->config->freq_cfg[1].cpu_instant_thresh[0])) {
		pdev->freq_mon_status = 5;
		pdev->client_input[APPS] = GEAR_LVL_5;
		pdev->client_input[GPU] = GEAR_LVL_11;
		ret = cache_allocation_configure(pdev);
	} else if (pdev->freq_mon_status != 6 &&
			((pdev->cpu_freq_curr[0] >=
					pdev->config->freq_cfg[0].cpu_instant_thresh[0] &&
			pdev->cpu_freq_curr[0] <
					pdev->config->freq_cfg[0].cpu_instant_thresh[1]) ||
			(pdev->cpu_freq_curr[0] >=
					pdev->config->freq_cfg[1].cpu_instant_thresh[0] &&
			pdev->cpu_freq_curr[1] <
					pdev->config->freq_cfg[1].cpu_instant_thresh[1]))) {
		pdev->freq_mon_status = 6;
		pdev->client_input[APPS] = GEAR_LVL_10;
		pdev->client_input[GPU] = GEAR_LVL_11;
		ret = cache_allocation_configure(pdev);
	} else if (pdev->freq_mon_status != 7 &&
			(pdev->cpu_freq_curr[0] >=
					pdev->config->freq_cfg[0].cpu_instant_thresh[1] ||
			pdev->cpu_freq_curr[1] >=
					pdev->config->freq_cfg[1].cpu_instant_thresh[1])){
		pdev->freq_mon_status = 7;
		pdev->client_input[APPS] = GEAR_LVL_10;
		pdev->client_input[GPU] = GEAR_LVL_6;
		ret = cache_allocation_configure(pdev);
	}

	if (ret < 0) {
		WARN_ON(1);
		return;
	}
}

static void freq_mon_gov(struct cache_allocation *pdev)
{
	if (pdev->gpu_freq_curr < pdev->config->freq_cfg[0].gpu_instant_thresh)
		freq_mon_prefer_cpu_gov(pdev);
	else if (pdev->gpu_freq_curr > pdev->config->freq_cfg[1].gpu_instant_thresh)
		freq_mon_prefer_gpu_gov(pdev);
}

static void bw_mon_ratio_gov(struct cache_allocation *pdev)
{
	int ret = 0;
	u64 ratio = 0;

	platform_bw_thresh_monitor(pdev, &ratio);
	if (!pdev->win_active)
		return;

	if (pdev->bw_mon_ratio_status != 1 &&
			ratio < pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[0]) {
		pdev->bw_mon_ratio_status = 1;
		pdev->client_input[APPS] = pdev->config->bw_ratio_cfg->bw_gear[0].cpu_gear;
		pdev->client_input[GPU] = pdev->config->bw_ratio_cfg->bw_gear[0].gpu_gear;
		ret = cache_allocation_configure(pdev);
	} else if (pdev->bw_mon_ratio_status != 2 &&
			ratio >= pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[0] &&
			ratio < pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[1]) {
		pdev->bw_mon_ratio_status = 2;
		pdev->client_input[APPS] = pdev->config->bw_ratio_cfg->bw_gear[1].cpu_gear;
		pdev->client_input[GPU] = pdev->config->bw_ratio_cfg->bw_gear[1].gpu_gear;
		ret = cache_allocation_configure(pdev);
	} else if (pdev->bw_mon_ratio_status != 3 &&
			ratio >= pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[1] &&
			ratio < pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[2]) {
		pdev->bw_mon_ratio_status = 3;
		pdev->client_input[APPS] = pdev->config->bw_ratio_cfg->bw_gear[2].cpu_gear;
		pdev->client_input[GPU] = pdev->config->bw_ratio_cfg->bw_gear[2].gpu_gear;
		ret = cache_allocation_configure(pdev);
	} else if (pdev->bw_mon_ratio_status != 4 &&
			ratio >= pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[2] &&
			ratio < pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[3]) {
		pdev->bw_mon_ratio_status = 4;
		pdev->client_input[APPS] = pdev->config->bw_ratio_cfg->bw_gear[3].cpu_gear;
		pdev->client_input[GPU] = pdev->config->bw_ratio_cfg->bw_gear[3].gpu_gear;
		ret = cache_allocation_configure(pdev);
	} else if (pdev->bw_mon_ratio_status != 5 &&
			ratio >= pdev->config->bw_ratio_cfg->bw_mon_ratio_thresh[3]) {
		pdev->bw_mon_ratio_status = 5;
		pdev->client_input[APPS] = pdev->config->bw_ratio_cfg->bw_gear[4].cpu_gear;
		pdev->client_input[GPU] = pdev->config->bw_ratio_cfg->bw_gear[4].gpu_gear;
		ret = cache_allocation_configure(pdev);
	}

	if (ret < 0) {
		WARN_ON(1);
		return;
	}
}

static void cache_allocation_gov_select(struct cache_allocation *pdev)
{
	mutex_lock(&pdev->lock);
	switch (pdev->current_governor) {
	case 0:
		freq_mon_gov(pdev);
		break;
	case 1:
		bw_mon_ratio_gov(pdev);
		break;
	default:
		return;
	}
	mutex_unlock(&pdev->lock);
}

static ssize_t available_governors_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i, cnt = 0;

	for (i = 0; i < ARRAY_SIZE(gov_list); i++) {
		if (i != ARRAY_SIZE(gov_list) - 1)
			cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%s ", gov_list[i]);
		else
			cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%s\n", gov_list[i]);
	}

	return cnt;
}
static DEVICE_ATTR_RO(available_governors);

static ssize_t current_governor_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);
	int i;
	int cnt = 0;

	for (i = 0; i < ARRAY_SIZE(gov_list); i++) {
		if (i == pd->current_governor)
			cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%s\n", gov_list[i]);
	}

	return cnt;
}

QCOM_CACHE_ALLOCATION_ATTR_STORE(current_governor);
static DEVICE_ATTR_RW(current_governor);

static ssize_t win_count_config_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", pd->win_count_config);
}

QCOM_CACHE_ALLOCATION_ATTR_STORE(win_count_config);
static DEVICE_ATTR_RW(win_count_config);

static ssize_t bw_mon_ratio_thresh_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);
	int i, cnt = 0;

	for (i = 0; i < 4; i++)
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%llu ",
					pd->config->bw_ratio_cfg->bw_mon_ratio_thresh[i]);
	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "\n");

	return cnt;
}

static ssize_t bw_mon_ratio_thresh_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);
	int var[4] = {0};
	int ret, i = 0;
	char *kbuf, *s, *str;

	kbuf = kstrdup(buf, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	s = kbuf;
	while (((str = strsep(&s, " ")) != NULL) && i < 4) {
		ret = kstrtoint(str, 10, &var[i]);
		if (ret < 0) {
			pr_err("Invalid value :%d\n", ret);
			goto out;
		}
		i++;
	}

	mutex_lock(&pd->lock);
	for (i = 0; i < 4; i++)
		pd->config->bw_ratio_cfg->bw_mon_ratio_thresh[i] = var[i];
	mutex_unlock(&pd->lock);

out:
	kfree(kbuf);
	return count;
}
static DEVICE_ATTR_RW(bw_mon_ratio_thresh);

static ssize_t freq_mon_thresh_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);
	int i, cnt = 0;

	for (i = 0; i < pd->cluster_num; i++) {
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%d ",
					pd->config->freq_cfg[i].cpu_boost_thresh);
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%d ",
					pd->config->freq_cfg[i].cpu_restore_thresh);
	}

	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%d\n",
				pd->config->freq_cfg[0].gpu_instant_thresh);

	for (i = 0; i < pd->cluster_num; i++) {
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%d ",
					pd->config->freq_cfg[i].cpu_instant_thresh[0]);
		cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%d ",
					pd->config->freq_cfg[i].cpu_instant_thresh[1]);
	}

	cnt += scnprintf(buf + cnt, PAGE_SIZE - cnt, "%d\n",
				pd->config->freq_cfg[1].gpu_instant_thresh);

	return cnt;
}

static ssize_t freq_mon_thresh_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);
	int var[10] = {0};
	int ret, i = 0;
	char *kbuf, *s, *str;

	kbuf = kstrdup(buf, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	s = kbuf;
	while (((str = strsep(&s, " ")) != NULL) && i < 10) {
		ret = kstrtoint(str, 10, &var[i]);
		if (ret < 0) {
			pr_err("Invalid value :%d\n", ret);
			goto out;
		}
		i++;
	}

	mutex_lock(&pd->lock);
	for (i = 0; i < pd->cluster_num; i++) {
		pd->config->freq_cfg[i].cpu_boost_thresh = var[i * 2];
		pd->config->freq_cfg[i].cpu_restore_thresh = var[i * 2 + 1];
		pd->config->freq_cfg[i].cpu_instant_thresh[0] = var[i * 2 + 5];
		pd->config->freq_cfg[i].cpu_instant_thresh[1] = var[i * 2 + 6];
		pd->config->freq_cfg[i].gpu_instant_thresh = var[i * 5 + 4];
	}
	mutex_unlock(&pd->lock);

out:
	kfree(kbuf);
	return count;
}
static DEVICE_ATTR_RW(freq_mon_thresh);

static ssize_t enable_monitor_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", pd->enable_monitor);
}

static ssize_t enable_monitor_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);
	bool val;
	int ret = 0;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (pd->enable_monitor == val)
		return count;

	mutex_lock(&pd->lock);
	pd->enable_monitor = val;
	pd->running_flag = val;
	mutex_unlock(&pd->lock);

	if (pd->enable_monitor) {
		save_gear_for_client(pd);
		schedule_delayed_work(&pd->work, msecs_to_jiffies(pd->sampling_time_ms));
	} else {
		mutex_lock(&pd->lock);
		pd->freq_mon_status = 0;
		pd->bw_mon_ratio_status = 0;
		pd->client_input[APPS] = pd->client_input_ext[APPS];
		pd->client_input[GPU] = pd->client_input_ext[GPU];
		ret = cache_allocation_configure(pd);
		mutex_unlock(&pd->lock);
		if (ret < 0)
			WARN_ON(1);

		cancel_delayed_work_sync(&pd->work);
	}

	return count;
}
static DEVICE_ATTR_RW(enable_monitor);

static ssize_t sampling_time_ms_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cache_allocation *pd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", pd->sampling_time_ms);
}

QCOM_CACHE_ALLOCATION_ATTR_STORE(sampling_time_ms);
static DEVICE_ATTR_RW(sampling_time_ms);

static void cache_allocation_monitor_work(struct work_struct *work)
{
	struct cache_allocation *pdev = container_of(work, struct cache_allocation,
						work.work);

	cpu_gpu_freq_update(pdev);
	cache_allocation_gov_select(pdev);
	trace_cache_alloc_config_check(pdev->current_governor,
					pdev->client_input[APPS], pdev->client_input[GPU]);

	mutex_lock(&pdev->lock);
	pdev->running_flag = true;
	mutex_unlock(&pdev->lock);

	schedule_delayed_work(&pdev->work, msecs_to_jiffies(pdev->sampling_time_ms));
}

static int cache_allocation_probe(struct platform_device *pdev)
{
	struct cache_allocation *pd;
	struct soc_config *config;
	int ret = 0;
	struct resource *res;

	config = (struct soc_config *)of_device_get_match_data(&pdev->dev);
	if (!config)
		return -ENODEV;

	pd = devm_kzalloc(&pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	ret = of_property_read_u32(pdev->dev.of_node,
				"qcom,cluster-num", &pd->cluster_num);
	if (ret)
		return -EINVAL;

	pd->config = devm_kzalloc(&pdev->dev, sizeof(*pd->config), GFP_KERNEL);
	if (!pd->config)
		return -ENOMEM;

	pd->config->freq_cfg = devm_kcalloc(&pdev->dev, pd->cluster_num,
						sizeof(struct freq_mon_config), GFP_KERNEL);
	if (!pd->config->freq_cfg)
		return -ENOMEM;

	pd->config->bw_ratio_cfg = devm_kzalloc(&pdev->dev,
						sizeof(struct bw_ratio_config), GFP_KERNEL);
	if (!pd->config->bw_ratio_cfg)
		return -ENOMEM;

	pd->cpu_freq_prev = devm_kcalloc(&pdev->dev, pd->cluster_num,
						sizeof(u32), GFP_KERNEL);
	if (!pd->cpu_freq_prev)
		return -ENOMEM;

	pd->cpu_freq_curr = devm_kcalloc(&pdev->dev, pd->cluster_num,
						sizeof(u32), GFP_KERNEL);
	if (!pd->cpu_freq_curr)
		return -ENOMEM;

	pd->client_input = devm_kcalloc(&pdev->dev, ARRAY_SIZE(msc_queries),
					sizeof(*pd->client_input), GFP_KERNEL);
	if (!pd->client_input)
		return -ENOMEM;

	pd->client_input_ext = devm_kcalloc(&pdev->dev, ARRAY_SIZE(msc_queries),
					sizeof(*pd->client_input_ext), GFP_KERNEL);
	if (!pd->client_input_ext)
		return -ENOMEM;

#if IS_ENABLED(CONFIG_QTI_GPU_RESOURCE_ENABLED)
	struct platform_device *gpu_pdev;
	struct device_link *link;

	pd->gpu_np = of_parse_phandle(pdev->dev.of_node, "qcom,devfreq", 0);
	if (IS_ERR(pd->gpu_np))
		return PTR_ERR(pd->gpu_np);

	gpu_pdev = of_find_device_by_node(pd->gpu_np);
	if (!gpu_pdev) {
		pr_err("Cannot find device node %s\n",
			pd->gpu_np->name);
		of_node_put(pd->gpu_np);
		return -ENODEV;
	}

	link = device_link_add(&pdev->dev, &gpu_pdev->dev,
		DL_FLAG_AUTOPROBE_CONSUMER);

	put_device(&gpu_pdev->dev);
	of_node_put(pd->gpu_np);

	if (!link) {
		pr_err("add gpu device_link fail\n");
		return -ENODEV;
	}

	if (link->status == DL_STATE_DORMANT) {
		pr_warn("kgsl not probed yet\n");
		device_link_del(link);
		return -EPROBE_DEFER;
	}

	device_link_del(link);
	pd->df = devfreq_get_devfreq_by_node(pd->gpu_np);
	if (IS_ERR(pd->df)) {
		of_node_put(pd->gpu_np);
		return PTR_ERR(pd->df);
	}
#endif

	ret = qcom_mpam_get_version(&mpam_version);
	if (ret || mpam_version.version < 0x10000) {
		dev_err(&pdev->dev, "Platform MPAM is not available\n");
		return -ENODEV;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mon-base");
	pd->mon_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(pd->mon_base)) {
		dev_err(&pdev->dev, "Error ioremap mpam_mon_base\n");
		return -ENODEV;
	}

	pd->bw_monitor_data = devm_kcalloc(&pdev->dev, ARRAY_SIZE(slc_config),
					sizeof(*pd->bw_monitor_data), GFP_KERNEL);
	if (!pd->bw_monitor_data)
		return -ENOMEM;

	memcpy(pd->config->freq_cfg, config->freq_cfg,
				pd->cluster_num * sizeof(struct freq_mon_config));
	memcpy(pd->config->bw_ratio_cfg, config->bw_ratio_cfg, sizeof(struct bw_ratio_config));
	pd->enable_monitor = false;
	pd->running_flag =  false;
	pd->sampling_time_ms = SAMPLING_MS;

	pd->current_governor = 1;
	pd->freq_mon_status = 0;
	pd->bw_mon_ratio_status = 0;
	pd->win_count_config = 16;
	pd->win_active = false;
	pd->client_input[APPS] = 0;
	pd->client_input[GPU] = 0;

	ret = device_create_file(&pdev->dev, &dev_attr_enable_monitor);
	if (ret) {
		dev_err(&pdev->dev, "failed: create cache alloc enable monitor entry\n");
		goto fail_create_enable_monitor;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_sampling_time_ms);
	if (ret) {
		dev_err(&pdev->dev, "failed: create cache alloc sampling time entry\n");
		goto fail_create_sampling_time_ms;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_freq_mon_thresh);
	if (ret) {
		dev_err(&pdev->dev, "failed: create cache alloc freq_mon_thresh entry\n");
		goto fail_create_freq_mon_thresh;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_win_count_config);
	if (ret) {
		dev_err(&pdev->dev, "failed: create slc allocation win_count_config entry\n");
		goto fail_create_bw_win_count;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_bw_mon_ratio_thresh);
	if (ret) {
		dev_err(&pdev->dev, "failed: create slc allocation bw_mon_ratio_thresh entry\n");
		goto fail_create_bw_ratio;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_current_governor);
	if (ret) {
		dev_err(&pdev->dev, "failed: create slc allocation current_governor entry\n");
		goto fail_create_current_governor;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_available_governors);
	if (ret) {
		dev_err(&pdev->dev, "failed: create slc allocation available_governors entry\n");
		goto fail_create_cavailable_governors;
	}

	mutex_init(&pd->lock);
	INIT_DELAYED_WORK(&pd->work, cache_allocation_monitor_work);
	platform_set_drvdata(pdev, pd);

	return 0;

fail_create_cavailable_governors:
	device_remove_file(&pdev->dev, &dev_attr_current_governor);
fail_create_current_governor:
	device_remove_file(&pdev->dev, &dev_attr_bw_mon_ratio_thresh);
fail_create_bw_ratio:
	device_remove_file(&pdev->dev, &dev_attr_win_count_config);
fail_create_bw_win_count:
	device_remove_file(&pdev->dev, &dev_attr_freq_mon_thresh);
fail_create_freq_mon_thresh:
	device_remove_file(&pdev->dev, &dev_attr_sampling_time_ms);
fail_create_sampling_time_ms:
	device_remove_file(&pdev->dev, &dev_attr_enable_monitor);
fail_create_enable_monitor:
#if IS_ENABLED(CONFIG_QTI_GPU_RESOURCE_ENABLED)
	of_node_put(pd->gpu_np);
#endif
	return ret;
}

static void cache_allocation_remove(struct platform_device *pdev)
{
	struct cache_allocation *pd = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&pd->work);
	device_remove_file(&pdev->dev, &dev_attr_enable_monitor);
	device_remove_file(&pdev->dev, &dev_attr_sampling_time_ms);
	device_remove_file(&pdev->dev, &dev_attr_freq_mon_thresh);
	device_remove_file(&pdev->dev, &dev_attr_win_count_config);
	device_remove_file(&pdev->dev, &dev_attr_bw_mon_ratio_thresh);
	device_remove_file(&pdev->dev, &dev_attr_current_governor);
	device_remove_file(&pdev->dev, &dev_attr_available_governors);
}

static struct freq_mon_config canoe_cpu_gpu_freq_cfg[] = {
	[0] = {
		.cluster_id = 0,
		.cpu_boost_thresh = 1500000,
		.cpu_restore_thresh = 1000000,
		.cpu_instant_thresh = { 115200, 1996800 },
		.gpu_instant_thresh = 443,
	},
	[1] = {
		.cluster_id = 6,
		.cpu_boost_thresh = 1500000,
		.cpu_restore_thresh = 1400000,
		.cpu_instant_thresh = { 1689600, 3072000 },
		.gpu_instant_thresh = 607,
	},
};

static struct freq_mon_config alor_cpu_gpu_freq_cfg[] = {
	[0] = {
		.cluster_id = 0,
		.cpu_boost_thresh = 1500000,
		.cpu_restore_thresh = 1000000,
		.cpu_instant_thresh = { 115200, 1996800 },
		.gpu_instant_thresh = 443,
	},
	[1] = {
		.cluster_id = 6,
		.cpu_boost_thresh = 1500000,
		.cpu_restore_thresh = 1400000,
		.cpu_instant_thresh = { 1689600, 3072000 },
		.gpu_instant_thresh = 607,
	},
};

static struct bw_ratio_config canoe_bw_ratio_cfg = {
	.bw_mon_ratio_thresh = { 10, 50, 150, 1000},
	.bw_gear = {
		{GEAR_BYPASS, GEAR_LVL_11},
		{GEAR_LVL_5, GEAR_LVL_11},
		{GEAR_LVL_10, GEAR_LVL_11},
		{GEAR_LVL_10, GEAR_LVL_6},
		{GEAR_LVL_10, GEAR_BYPASS},
	},
};

static struct bw_ratio_config alor_bw_ratio_cfg = {
	.bw_mon_ratio_thresh = { 10, 50, 150, 1000},
	.bw_gear = {
		{GEAR_BYPASS, GEAR_LVL_9},
		{GEAR_LVL_4, GEAR_LVL_9},
		{GEAR_LVL_9, GEAR_LVL_9},
		{GEAR_LVL_9,  GEAR_LVL_4},
		{GEAR_LVL_9, GEAR_BYPASS},
	},
};

struct soc_config canoe_config = {
	.freq_cfg = canoe_cpu_gpu_freq_cfg,
	.bw_ratio_cfg = &canoe_bw_ratio_cfg,
};

struct soc_config alor_config = {
	.freq_cfg = alor_cpu_gpu_freq_cfg,
	.bw_ratio_cfg = &alor_bw_ratio_cfg,
};

static const struct of_device_id cache_allocation_table[] = {
	{ .compatible = "qcom,cache-allocation-canoe", .data = &canoe_config },
	{ .compatible = "qcom,cache-allocation-alor", .data = &alor_config },
	{ },
};
MODULE_DEVICE_TABLE(of, cache_allocation_table);

static struct platform_driver cache_allocation_driver = {
	.probe = cache_allocation_probe,
	.remove = cache_allocation_remove,
	.driver = {
		.name = "cache allocation",
		.of_match_table = cache_allocation_table,
	},
};
module_platform_driver(cache_allocation_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm Technologies, Inc. (QTI) cache allocation driver");
