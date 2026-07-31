// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) "qcom_mpam_slc: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/qcom_scmi_vendor.h>
#include <linux/scmi_protocol.h>
#include <soc/qcom/mpam_msc.h>
#include <soc/qcom/mpam_slc.h>
#include <linux/clk.h>

#define SCMI_GET_PARAM_LEN	64
/* Older firmware Client details */
static uint16_t slc_client_id[] = {
	APPS,
	GPU,
	NSP,
	SLC_CLIENT_MAX,
};

static struct clk *llcc_perfmon_clock;
static int perf_clk_enable;

static int llcc_perf_clk_enable(bool enable)
{
	if (llcc_perfmon_clock == NULL)
		return -ENOENT;

	if (enable)
		clk_prepare_enable(llcc_perfmon_clock);
	else
		clk_disable_unprepare(llcc_perfmon_clock);

	return 0;
}

static int mpam_msc_slc_set_params(struct device *dev, void *param, int param_len,
		uint32_t param_id)
{
	struct qcom_mpam_msc *qcom_msc;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_msc == NULL)
		return -EINVAL;

	return qcom_msc->scmi_ops->set_param(qcom_msc->ph, param, QCOM_SLC_MPAM_SCMI_STR,
				param_id, param_len);
}

static int mpam_msc_slc_get_params(struct device *dev, void *param_in, int param_in_len,
		void *param_out, int param_out_len, uint32_t param_id)
{
	int ret = -EPERM;
	uint8_t buf[SCMI_GET_PARAM_LEN];
	struct qcom_mpam_msc *qcom_msc;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_msc == NULL)
		return ret;

	if (param_in_len && param_in_len <= SCMI_GET_PARAM_LEN) {
		memcpy(buf, param_in, param_in_len);
		ret = qcom_msc->scmi_ops->get_param(qcom_msc->ph, buf,
			QCOM_SLC_MPAM_SCMI_STR, param_id, param_in_len, param_out_len);
	}

	if (!ret)
		memcpy(param_out, buf, param_out_len);

	return ret;
}

static int qcom_mpam_msc_enabled(struct qcom_mpam_msc *qcom_msc)
{
	if ((qcom_msc == NULL) || (qcom_msc->msc_capability == NULL) ||
			 (qcom_msc->mpam_available & MPAM_AVAILABLE) == 0)
		return -EINVAL;

	return 0;
}

static struct qcom_mpam_msc *slc_capability_check(struct device *dev, struct msc_query *query)
{
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;
	struct slc_client_capability *slc_client_cap;
	int client_idx, partid_idx;

	client_idx = query->client_id;
	partid_idx = query->part_id;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_mpam_msc_enabled(qcom_msc))
		return NULL;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	if (client_idx >= slc_capability->num_clients) {
		dev_err(dev, "Invalid Client ID %d\n", client_idx);
		return NULL;
	}

	slc_client_cap = &(slc_capability->slc_client_cap[client_idx]);

	if (slc_client_cap->enabled == false) {
		dev_err(dev, "Client not enabled for configuration %d\n", client_idx);
		return NULL;
	}

	if (partid_idx >= slc_client_cap->client_info.num_part_id) {
		dev_err(dev, "Invalid PART id %d\n", partid_idx);
		return NULL;
	}

	return qcom_msc;
}

static int slc_mpam_version(struct device *dev, void *val)
{
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;
	uint32_t *firmware_ver = val;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_mpam_msc_enabled(qcom_msc))
		return -EINVAL;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	*firmware_ver = slc_capability->firmware_ver.firmware_version;
	return 0;
}

static int slc_set_cache_partition(struct device *dev, void *msc_partid, void *msc_partconfig)
{
	struct qcom_mpam_msc *qcom_msc;
	struct slc_partid_config slc_part_config;
	struct qcom_slc_capability *slc_capability;
	struct slc_client_capability *slc_client_cap;
	struct slc_partid_capability *partid_cap;
	int client_idx, partid_idx, gear_idx;

	qcom_msc = slc_capability_check(dev, &slc_part_config.query);
	if (qcom_msc == NULL)
		return -EINVAL;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	memcpy(&slc_part_config.query, msc_partid, sizeof(struct msc_query));
	slc_part_config.gear_config.gear_val =
		((struct qcom_slc_gear_val *)msc_partconfig)->gear_val;

	if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0) {
		client_idx = slc_part_config.query.client_id;
		partid_idx = slc_part_config.query.part_id;
		slc_client_cap = &(slc_capability->slc_client_cap[client_idx]);
		partid_cap = (struct slc_partid_capability *)
			&(slc_client_cap->slc_partid_cap[partid_idx]);

		for (gear_idx = 0; gear_idx < partid_cap->num_gears; gear_idx++) {
			if (partid_cap->part_id_gears[gear_idx] ==
					slc_part_config.gear_config.gear_val)
				break;
		}

		if (gear_idx == partid_cap->num_gears) {
			dev_err(dev, "GEAR config not valid!\n");
			return -EINVAL;
		}
	}

	return mpam_msc_slc_set_params(dev, &slc_part_config, sizeof(struct slc_partid_config),
			PARAM_SET_CACHE_PARTITION_MSC);
}

static int slc_reset_cache_partition(struct device *dev, void *msc_partid, void *msc_partconfig)
{
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;
	struct slc_partid_config slc_part_config = {0};

	qcom_msc = slc_capability_check(dev, &slc_part_config.query);
	if (qcom_msc == NULL)
		return -EINVAL;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	memcpy(&slc_part_config.query, msc_partid, sizeof(struct msc_query));

	slc_part_config.gear_config.gear_val =
		((struct qcom_slc_gear_val *)msc_partconfig)->gear_val;

	return mpam_msc_slc_set_params(dev, &slc_part_config, sizeof(struct slc_partid_config),
			PARAM_RESET_CACHE_PARTITION_MSC);
}

static int slc_firmware_version_query(struct device *dev, void *msc_partid, void *version)
{
	struct qcom_mpam_msc *qcom_msc;
	struct msc_query query;
	union qcom_slc_firmware_version *firmware_ver;

	firmware_ver = (union qcom_slc_firmware_version *)version;
	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_msc == NULL)
		return -EINVAL;

	return mpam_msc_slc_get_params(dev, &query, sizeof(struct msc_query), firmware_ver,
			sizeof(union qcom_slc_firmware_version), PARAM_GET_SLC_MPAM_VERSION);
}

static int slc_client_query(struct device *dev, void *msc_partid, void *msc_partconfig)
{
	struct qcom_mpam_msc *qcom_msc;
	struct msc_query *query;
	struct slc_client_info *client_info;

	query = (struct msc_query *)msc_partid;
	client_info = (struct slc_client_info *)msc_partconfig;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_msc == NULL)
		return -EINVAL;

	return mpam_msc_slc_get_params(dev, query, sizeof(struct msc_query), client_info,
			sizeof(struct slc_client_info), PARAM_GET_CLIENT_INFO_MSC);
}

static int slc_get_cache_partition(struct device *dev, void *msc_partid, void *msc_partconfig)
{
	struct qcom_mpam_msc *qcom_msc;
	struct msc_query *query;
	struct qcom_slc_capability *slc_capability;
	struct slc_client_capability *slc_client_cap;
	struct qcom_slc_gear_val *gear_config;

	query = (struct msc_query *)msc_partid;
	qcom_msc = slc_capability_check(dev, query);
	if (qcom_msc == NULL)
		return -EINVAL;

	gear_config = (struct qcom_slc_gear_val *)msc_partconfig;
	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	slc_client_cap = &slc_capability->slc_client_cap[query->client_id];
	if (slc_client_cap->enabled == false)
		return -EINVAL;

	return mpam_msc_slc_get_params(dev, query, sizeof(struct msc_query), gear_config,
			sizeof(struct qcom_slc_gear_val), PARAM_GET_CACHE_PARTITION_MSC);
}

static int slc_get_cache_partition_capability(struct device *dev, void *msc_partid,
		void *msc_partconfig)
{
	struct qcom_mpam_msc *qcom_msc;
	struct msc_query *query;
	struct qcom_slc_capability *slc_capability;
	union slc_partid_capability_def *partid_cap;
	struct slc_client_capability *slc_client_cap;

	query = (struct msc_query *)msc_partid;
	qcom_msc = slc_capability_check(dev, query);
	if (qcom_msc == NULL)
		return -EINVAL;

	partid_cap = (union slc_partid_capability_def *)msc_partconfig;
	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	slc_client_cap = &slc_capability->slc_client_cap[query->client_id];
	if (slc_client_cap->enabled == false)
		return -EINVAL;

	return mpam_msc_slc_get_params(dev, query, sizeof(struct msc_query),
			partid_cap, sizeof(union slc_partid_capability_def),
			PARAM_GET_CACHE_CAPABILITY_MSC);
}

static int mon_idx_lookup(void __iomem *mem, int client_id, int part_id,
		struct qcom_slc_capability *slc_capability)
{
	int mon_idx;
	union qcom_slc_monitor_memory *mon_mem = (union qcom_slc_monitor_memory *)mem;
	struct slc_partid_info *part_info;
	struct qcom_slc_mon_data *data;
	struct qcom_slc_mon_data_v1 *data_v1;
	struct slc_client_capability *slc_client_cap;
	union slc_partid_capability_def *slc_partid_cap;
	int client_idx, part_idx;

	mon_idx = 0;
	slc_client_cap = slc_capability->slc_client_cap;
	if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0)
		data = &(mon_mem->mem_v0.data[0]);
	else
		data_v1 = &(mon_mem->mem_v1.data[0]);

	for (client_idx = 0; client_idx < slc_capability->num_clients; client_idx++) {
		slc_partid_cap = slc_client_cap->slc_partid_cap;
		for (part_idx = 0; part_idx < slc_client_cap->client_info.num_part_id; part_idx++) {
			if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0) {
				part_info = &(data->part_info);
				data++;
			} else {
				if (slc_partid_cap->v1_cap.mon_support == 0) {
					slc_partid_cap++;
					continue;
				}

				part_info = &(data_v1->part_info);
				data_v1++;
			}

			if ((client_id == part_info->client_id) &&
					(part_id == part_info->part_id))
				return mon_idx;

			mon_idx++;
			slc_partid_cap++;
		}

		slc_client_cap++;
	}

	return -EINVAL;
}

static struct qcom_mpam_msc *slc_config_request_check(struct device *dev, struct msc_query *query,
		struct slc_mon_config_val *mon_cfg)
{
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;
	int mon_idx;

	qcom_msc = slc_capability_check(dev, query);
	if (qcom_msc == NULL)
		return NULL;

	slc_capability =  (struct qcom_slc_capability *)qcom_msc->msc_capability;
	mon_idx = mon_idx_lookup(qcom_msc->mon_base, query->client_id, query->part_id,
			slc_capability);
	if (mon_idx < 0) {
		pr_err("Failed to get monitor index\n");
		return NULL;
	}

	switch (mon_cfg->slc_mon_function) {
	default:
		break;

	case CACHE_CAPACITY_CONFIG:
		switch (mon_cfg->enable) {
		default:
			break;

		case 0:
			if (slc_capability->slc_mon_configured.capacity_configured <= 0)
				return NULL;
			break;

		case 1:
			if (slc_capability->slc_mon_configured.capacity_configured >=
					slc_capability->slc_mon_list.capacity_config_available)
				return NULL;
			break;
		}
		break;

	case CACHE_READ_MISS_CONFIG:
		switch (mon_cfg->enable) {
		default:
			break;

		case 0:
			if (slc_capability->slc_mon_configured.read_miss_configured <= 0)
				return NULL;
			break;

		case 1:
			if (slc_capability->slc_mon_configured.read_miss_configured >=
					slc_capability->slc_mon_list.read_miss_config_available)
				return NULL;
			break;
		}
		break;

	case CACHE_FE_MON_CONFIG:
	case CACHE_BE_MON_CONFIG:
		/* FE, BE Monitors enabled only for clients, not for Part Id basis */
		if (query->part_id != 0) {
			dev_err(dev, "Invalid Part ID %d\n", query->part_id);
				return NULL;
		}
		break;
	}

	return qcom_msc;

}

static int update_mon_stats(struct device *dev, struct msc_query *query,
		struct slc_mon_config_val *mon_cfg)
{
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;
	int mon_idx;

	qcom_msc = slc_capability_check(dev, query);
	if (qcom_msc == NULL)
		return -EINVAL;

	slc_capability =  (struct qcom_slc_capability *)qcom_msc->msc_capability;
	mon_idx = mon_idx_lookup(qcom_msc->mon_base, query->client_id, query->part_id,
			slc_capability);
	if (mon_idx < 0)
		return -EINVAL;

	switch (mon_cfg->slc_mon_function) {
	default:
		break;

	case CACHE_CAPACITY_CONFIG:
		switch (mon_cfg->enable) {
		default:
			break;

		case 0:
			slc_capability->slc_mon_configured.capacity_configured--;
			slc_capability->slc_mon_configured.mon_cfgd--;
			break;

		case 1:
			slc_capability->slc_mon_configured.capacity_configured++;
			slc_capability->slc_mon_configured.mon_cfgd++;
			break;

		}
		break;

	case CACHE_READ_MISS_CONFIG:
		switch (mon_cfg->enable) {
		default:
			break;

		case 0:
			slc_capability->slc_mon_configured.read_miss_configured--;
			slc_capability->slc_mon_configured.mon_cfgd--;
			break;

		case 1:
			slc_capability->slc_mon_configured.read_miss_configured++;
			slc_capability->slc_mon_configured.mon_cfgd++;
			break;

		}
		break;

	case CACHE_FE_MON_CONFIG:
	case CACHE_BE_MON_CONFIG:
		switch (mon_cfg->enable) {
		default:
			break;

		case 0:
			slc_capability->slc_mon_configured.mon_cfgd--;
			break;

		case 1:
			slc_capability->slc_mon_configured.mon_cfgd++;
			break;

		}
		break;
	}

	if (mon_cfg->enable) {
		if (slc_capability->slc_mon_configured.mon_cfgd == 1)
			__module_get(THIS_MODULE);
	} else {
		if (slc_capability->slc_mon_configured.mon_cfgd == 0)
			module_put(THIS_MODULE);
	}

	return 0;
}

static int slc_mon_config(struct device *dev, void *msc_partid, void *msc_partconfig)
{
	struct qcom_mpam_msc *qcom_msc;
	struct msc_query *query;
	struct slc_mon_config_val *mon_cfg_val;
	struct slc_partid_capability_v1 partid_cap = { 0 };
	struct slc_mon_config mon_cfg = {0};
	int ret = -EINVAL;
	uint32_t firmware_ver;

	query = (struct msc_query *)msc_partid;
	mon_cfg_val = (struct slc_mon_config_val *)msc_partconfig;

	qcom_msc = slc_config_request_check(dev, query, mon_cfg_val);
	if (qcom_msc == NULL)
		return ret;

	firmware_ver = SLC_MPAM_VERSION_0;
	msc_system_get_mpam_version(SLC, &firmware_ver);
	if (firmware_ver != SLC_MPAM_VERSION_0) {
		ret = msc_system_get_device_capability(SLC, query, &partid_cap);
		if (ret) {
			pr_err("Failed to Config SLC Mon\n");
			return ret;
		}

		switch (mon_cfg_val->slc_mon_function) {
		default:
			break;

		case CACHE_CAPACITY_CONFIG:
			if ((partid_cap.mon_support & (1 << cap_mon_support)) == 0)
				return -EPERM;
			break;

		case CACHE_READ_MISS_CONFIG:
			if ((partid_cap.mon_support & (1 << read_miss_mon_support)) == 0)
				return -EPERM;
			break;

		case CACHE_FE_MON_CONFIG:
			if (firmware_ver == SLC_MPAM_VERSION_0)
				return -EPERM;

			if ((partid_cap.mon_support & (1 << fe_mon_support)) == 0)
				return -EPERM;

			if (mon_cfg_val->enable && !perf_clk_enable)
				if (llcc_perf_clk_enable(true))
					return -EPERM;
			break;

		case CACHE_BE_MON_CONFIG:
			if (firmware_ver == SLC_MPAM_VERSION_0)
				return -EPERM;

			if ((partid_cap.mon_support & (1 << be_mon_support)) == 0)
				return -EPERM;

			if (mon_cfg_val->enable && !perf_clk_enable)
				if (llcc_perf_clk_enable(true))
					return -EPERM;
			break;

		}
	}

	memcpy(&mon_cfg.query, msc_partid, sizeof(struct msc_query));
	memcpy(&mon_cfg.config, msc_partconfig, sizeof(struct slc_mon_config_val));
	ret = mpam_msc_slc_set_params(dev, &mon_cfg, sizeof(struct  slc_mon_config),
			PARAM_SET_CONFIG_MON_MSC);
	if (ret) {
		if ((mon_cfg_val->slc_mon_function == CACHE_FE_MON_CONFIG) ||
			(mon_cfg_val->slc_mon_function == CACHE_BE_MON_CONFIG)) {
			if (mon_cfg_val->enable && !perf_clk_enable)
				llcc_perf_clk_enable(false);
		}

		pr_err("Failed to Config SLC Mon\n");
	} else {
		if ((mon_cfg_val->slc_mon_function == CACHE_FE_MON_CONFIG) ||
			(mon_cfg_val->slc_mon_function == CACHE_BE_MON_CONFIG)) {
			if (mon_cfg_val->enable)
				perf_clk_enable++;
			else
				perf_clk_enable--;

			if (!mon_cfg_val->enable && !perf_clk_enable)
				llcc_perf_clk_enable(false);
		}

		update_mon_stats(dev, query, mon_cfg_val);
	}

	return ret;
};

#define MAX_SHARED_MEM_RETRY_CNT	5000
#define MAX_SHARED_MATCH_SEQ_CNT	10

static int slc_mon_stats_read(struct device *dev, void *msc_partid, void *data)
{
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;
	struct msc_query *query;
	union qcom_slc_monitor_memory *mon_mem;
	union mon_values *mon_data;
	struct qcom_slc_mon_data *data_mem;
	struct qcom_slc_mon_data_v1 *data_mem_v1;
	int mon_idx;
	uint32_t match_seq, retry_cnt, match_seq_cnt = 0;
	volatile uint32_t *match_seq_ptr;
	uint64_t *last_capture_time;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_mpam_msc_enabled(qcom_msc))
		return -EINVAL;

	query = (struct msc_query *)msc_partid;
	mon_data = (union mon_values *)data;
	slc_capability =  (struct qcom_slc_capability *)qcom_msc->msc_capability;
	mon_idx = mon_idx_lookup(qcom_msc->mon_base, query->client_id, query->part_id,
			slc_capability);
	if (mon_idx < 0)
		return -EINVAL;

	mon_mem = (union qcom_slc_monitor_memory *)qcom_msc->mon_base;
	if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0) {
		data_mem = &(mon_mem->mem_v0.data[mon_idx]);
		match_seq_ptr = &(mon_mem->mem_v0.match_seq);
		last_capture_time = &(mon_mem->mem_v0.last_capture_time);
	} else {
		data_mem_v1 = &(mon_mem->mem_v1.data[mon_idx]);
		match_seq_ptr = &(mon_mem->mem_v1.match_seq);
		last_capture_time = &(mon_mem->mem_v1.last_capture_time);
	}

	do {
		retry_cnt = 0;
		while (unlikely((match_seq = *match_seq_ptr) % 2) &&
				(retry_cnt++ < MAX_SHARED_MEM_RETRY_CNT))
			;

		if (retry_cnt >= MAX_SHARED_MEM_RETRY_CNT) {
			pr_err("SLC MPAM Monitor failed. FW agent updating the same memory\n");
			return -EIO;
		}

		/* Read as zero if monitor not enabled */
		mon_data->mon_stats.num_cache_lines = 0;
		mon_data->mon_stats.num_rd_misses = 0;
		mon_data->mon_stats.slc_fe_bytes = 0;
		mon_data->mon_stats.slc_be_bytes = 0;
		if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0) {
			switch (mon_data->ref.slc_mon_function) {
			case CACHE_CAPACITY_CONFIG:
				if (data_mem->cap_stats.cap_enabled)
					mon_data->capacity.num_cache_lines =
						data_mem->cap_stats.num_cache_lines;
				break;

			case CACHE_READ_MISS_CONFIG:
				if (data_mem->rd_miss_stats.miss_enabled)
					mon_data->misses.num_rd_misses =
						data_mem->rd_miss_stats.rd_misses;
				break;

			default:
				break;
			}

		} else {
			switch (mon_data->ref.slc_mon_function) {
			case CACHE_CAPACITY_CONFIG:
				if (data_mem_v1->mon_enabled & (1 << cap_mon_support))
					mon_data->capacity.num_cache_lines =
						data_mem_v1->num_cache_lines;

				break;

			case CACHE_READ_MISS_CONFIG:
				if (data_mem_v1->mon_enabled & (1 << read_miss_mon_support))
					mon_data->misses.num_rd_misses = data_mem_v1->rd_misses;

				break;

			case CACHE_FE_MON_CONFIG:
				if (data_mem_v1->mon_enabled & (1 << fe_mon_support))
					mon_data->fe_stats.slc_fe_bytes = data_mem_v1->fe_rd_bytes +
						data_mem_v1->fe_wr_bytes;

				break;

			case CACHE_BE_MON_CONFIG:
				if (data_mem_v1->mon_enabled & (1 << be_mon_support))
					mon_data->be_stats.slc_be_bytes = data_mem_v1->be_rd_bytes +
						data_mem_v1->be_wr_bytes;

				break;

			case CACHE_MON_STATS_READ:
				if (data_mem_v1->mon_enabled & (1 << cap_mon_support))
					mon_data->mon_stats.num_cache_lines =
						data_mem_v1->num_cache_lines;

				if (data_mem_v1->mon_enabled & (1 << read_miss_mon_support))
					mon_data->mon_stats.num_rd_misses = data_mem_v1->rd_misses;

				if (data_mem_v1->mon_enabled & (1 << fe_mon_support))
					mon_data->mon_stats.slc_fe_bytes = data_mem_v1->fe_rd_bytes
						+ data_mem_v1->fe_wr_bytes;

				if (data_mem_v1->mon_enabled & (1 << be_mon_support))
					mon_data->mon_stats.slc_be_bytes = data_mem_v1->be_rd_bytes
						+ data_mem_v1->be_wr_bytes;

				break;

			default:
				break;
			}
		}

		mon_data->ref.last_capture_time = *last_capture_time;
	} while ((match_seq != *match_seq_ptr) &&
			(match_seq_cnt++ < MAX_SHARED_MATCH_SEQ_CNT));

	if (match_seq_cnt == MAX_SHARED_MATCH_SEQ_CNT)
		return -EIO;

	return 0;
}

static ssize_t slc_mon_stats_print_data_v1(void *buf, struct qcom_slc_capability *slc_capability,
		volatile struct qcom_slc_mon_data_v1 *data_v1,  uint64_t last_capture_time)
{
	ssize_t len;
	int client_idx, part_idx;
	struct slc_client_capability *slc_client_cap;

	slc_client_cap = slc_capability->slc_client_cap;
	len = scnprintf(buf, PAGE_SIZE, "timestamp=%llu\n", last_capture_time);
	for (client_idx = 0; client_idx < slc_capability->num_clients; client_idx++) {
		switch (slc_client_cap->client_info.num_part_id) {
		case 1:
			if (!data_v1->mon_enabled) {
				data_v1++;
				break;
			}

			len += scnprintf(buf + len, PAGE_SIZE - len, "%s:\n",
					slc_client_cap->client_name);

			if (data_v1->mon_enabled & (1 << cap_mon_support))
				len += scnprintf(buf + len, PAGE_SIZE - len, "cap_cnt=%d,",
						data_v1->num_cache_lines);

			if (data_v1->mon_enabled & (1 << read_miss_mon_support))
				len += scnprintf(buf + len, PAGE_SIZE - len,
						"miss_cnt=%llu,", data_v1->rd_misses);

			if (data_v1->mon_enabled & (1 << fe_mon_support))
				len += scnprintf(buf + len, PAGE_SIZE - len, "fe_bytes=%llu,",
						data_v1->fe_rd_bytes + data_v1->fe_wr_bytes);

			if (data_v1->mon_enabled & (1 << be_mon_support))
				len += scnprintf(buf + len, PAGE_SIZE - len, "be_bytes=%llu,",
						data_v1->be_rd_bytes + data_v1->be_wr_bytes);

			len -= 1;
			len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
			data_v1++;
			break;

		default:
			/* Handling More than 1 Part ID's */
			for (part_idx = 0; part_idx < slc_client_cap->client_info.num_part_id;
					part_idx++) {
				if (slc_client_cap->slc_partid_cap[part_idx].v1_cap.mon_support
						== 0)
					continue;

				if (!data_v1->mon_enabled) {
					data_v1++;
					continue;
				}

				if (!part_idx) {
					if (data_v1->mon_enabled & ((1 << fe_mon_support) |
							(1 << be_mon_support)))
						len += scnprintf(buf + len, PAGE_SIZE - len,
								"%s:\n",
								slc_client_cap->client_name);

					if (data_v1->mon_enabled & (1 << fe_mon_support))
						len += scnprintf(buf + len, PAGE_SIZE - len,
							"fe_bytes=%llu,",
							data_v1->fe_rd_bytes +
							data_v1->fe_wr_bytes);

					if (data_v1->mon_enabled & (1 << be_mon_support))
						len += scnprintf(buf + len, PAGE_SIZE - len,
							"be_bytes=%llu,",
							data_v1->be_rd_bytes +
							data_v1->be_wr_bytes);

					len -= 1;
					len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
				}

				if (data_v1->mon_enabled & ((1 << cap_mon_support) |
							(1 << read_miss_mon_support)))
					len += scnprintf(buf + len, PAGE_SIZE - len,
							"%s part %d:\n",
							slc_client_cap->client_name,
							part_idx);

				if (data_v1->mon_enabled & (1 << cap_mon_support))
					len += scnprintf(buf + len, PAGE_SIZE - len, "cap_cnt=%d,",
							data_v1->num_cache_lines);

				if (data_v1->mon_enabled & (1 << read_miss_mon_support))
					len += scnprintf(buf + len, PAGE_SIZE - len,
							"miss_cnt=%llu,", data_v1->rd_misses);

				len -= 1;
				len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
				data_v1++;
			}
			break;
		}

		slc_client_cap++;
	}

	return len;
}

static ssize_t slc_mon_stats_print_data(void *buf, struct qcom_slc_capability *slc_capability,
		volatile struct qcom_slc_mon_data *data,  uint64_t last_capture_time)
{
	ssize_t len;
	int client_idx, part_idx;
	struct slc_client_capability *slc_client_cap;
	volatile struct slc_capacity *cap_stats;
	volatile struct slc_read_miss_cntr *rd_miss_stats;

	slc_client_cap = slc_capability->slc_client_cap;
	len = scnprintf(buf, PAGE_SIZE, "timestamp=%llu\n", last_capture_time);
	for (client_idx = 0; client_idx < slc_capability->num_clients;
			client_idx++) {
		switch (slc_client_cap->client_info.num_part_id) {
		case 1:
			cap_stats = &data->cap_stats;
			rd_miss_stats = &data->rd_miss_stats;
			if (!cap_stats->cap_enabled &&
					!rd_miss_stats->miss_enabled) {
				data++;
				break;
			}

			len += scnprintf(buf + len, PAGE_SIZE - len, "%s:\n",
					slc_client_cap->client_name);

			if (cap_stats->cap_enabled)
				len += scnprintf(buf + len, PAGE_SIZE - len, "cap_cnt=%d,",
						cap_stats->num_cache_lines);


			if (rd_miss_stats->miss_enabled)
				len += scnprintf(buf + len, PAGE_SIZE - len, "miss_cnt=%llu,",
						rd_miss_stats->rd_misses);

			len -= 1;
			len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
			data++;
			break;
		default:
			/* Handling More than 1 Part ID's */
			for (part_idx = 0; part_idx < slc_client_cap->client_info.num_part_id;
					part_idx++) {
				cap_stats = &data->cap_stats;
				rd_miss_stats = &data->rd_miss_stats;
				if (!cap_stats->cap_enabled &&
						!rd_miss_stats->miss_enabled) {
					data++;
					continue;
				}

				len += scnprintf(buf + len, PAGE_SIZE - len,
						"%s part %d:\n",
						slc_client_cap->client_name,
						part_idx);

				if (cap_stats->cap_enabled)
					len += scnprintf(buf + len, PAGE_SIZE - len,
							"cap_cnt=%d,",
							cap_stats->num_cache_lines);

				if (rd_miss_stats->miss_enabled)
					len += scnprintf(buf + len, PAGE_SIZE - len,
							"miss_cnt=%llu,",
							rd_miss_stats->rd_misses);

				len -= 1;
				len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
				data++;
			}

			break;
		}

		slc_client_cap++;
	}

	return len;
}

static int slc_mon_stats_print(struct device *dev, void *msc_partid, void *buf)
{
	ssize_t len;
	uint32_t match_seq, retry_cnt, match_seq_cnt = 0;
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;
	union qcom_slc_monitor_memory *mon_mem;
	volatile struct qcom_slc_mon_data *data;
	volatile struct qcom_slc_mon_data_v1 *data_v1;
	volatile uint32_t *match_seq_ptr;
	volatile uint64_t *last_capture_time;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_mpam_msc_enabled(qcom_msc))
		return -EINVAL;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	mon_mem = (union qcom_slc_monitor_memory *)qcom_msc->mon_base;

	if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0) {
		match_seq_ptr = &(mon_mem->mem_v0.match_seq);
		last_capture_time = &(mon_mem->mem_v0.last_capture_time);
	} else {
		match_seq_ptr = &(mon_mem->mem_v1.match_seq);
		last_capture_time = &(mon_mem->mem_v1.last_capture_time);
	}

	do {
		retry_cnt = 0;
		if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0)
			data = &(mon_mem->mem_v0.data[0]);
		else
			data_v1 = &(mon_mem->mem_v1.data[0]);

		while (unlikely((match_seq = *match_seq_ptr) % 2)
				&& (retry_cnt++ < MAX_SHARED_MEM_RETRY_CNT))
			;

		if (unlikely(retry_cnt >= MAX_SHARED_MEM_RETRY_CNT)) {
			pr_err("SLC MPAM Monitor failed. FW agent updating the same memory\n");
			return -EIO;
		}

		if (slc_capability->firmware_ver.firmware_version == SLC_MPAM_VERSION_0)
			len = slc_mon_stats_print_data(buf, slc_capability, data,
					*last_capture_time);
		else
			len = slc_mon_stats_print_data_v1(buf, slc_capability, data_v1,
					*last_capture_time);

	} while ((match_seq != *match_seq_ptr) && (match_seq_cnt++ < MAX_SHARED_MATCH_SEQ_CNT));

	if (match_seq_cnt == MAX_SHARED_MATCH_SEQ_CNT)
		return -EIO;

	return len;
}

static struct mpam_msc_ops slc_msc_ops = {
	.get_firmware_version = slc_mpam_version,
	.set_cache_partition = slc_set_cache_partition,
	.get_cache_partition = slc_get_cache_partition,
	.get_cache_partition_capability = slc_get_cache_partition_capability,
	.reset_cache_partition = slc_reset_cache_partition,
	.mon_config = slc_mon_config,
	.mon_stats_read = slc_mon_stats_read,
	.mon_stats_print = slc_mon_stats_print,
};

static int slc_client_info_read(struct device *dev, struct device_node *node)
{
	int client_idx, partid_idx, num_part_ids, ret = -EINVAL;
	struct msc_query query;
	struct qcom_mpam_msc *qcom_msc;
	struct slc_client_capability *client_cap;
	struct qcom_slc_capability *slc_capability;
	struct qcom_slc_mon_mem_v1 *mon_mem;
	struct slc_sct_client_info *client_info;
	struct slc_client_details *client_details;

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_msc->qcom_msc_id.qcom_msc_type != SLC)
		return -EINVAL;

	slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	query.qcom_msc_id.qcom_msc_type = qcom_msc->qcom_msc_id.qcom_msc_type;
	query.qcom_msc_id.qcom_msc_class = qcom_msc->qcom_msc_id.qcom_msc_class;
	query.qcom_msc_id.idx = qcom_msc->qcom_msc_id.idx;
	if (slc_capability->firmware_ver.firmware_version != SLC_MPAM_VERSION_0) {
		mon_mem = (struct qcom_slc_mon_mem_v1 *)qcom_msc->mon_base;
		client_info = (struct slc_sct_client_info *)&(mon_mem->data);
		slc_capability->num_clients = client_info->num_clients;
		slc_capability->slc_mon_list.capacity_config_available =
			client_info->slc_mon_info.num_cap_monitor;
		slc_capability->slc_mon_list.read_miss_config_available =
			client_info->slc_mon_info.num_miss_monitor;
		if ((slc_capability->num_clients == 0) ||
				(client_info->slc_mon_info.num_cap_monitor == 0) ||
				(client_info->slc_mon_info.num_miss_monitor == 0)) {
			pr_err("SLC Client info population Failed!\n");
			return -EINVAL;
		}

		pr_info("SLC Monitor size %lld\n", mon_mem->slc_mpam_monitor_size);
		pr_info("Number of SLC MPAM clients %d\n", slc_capability->num_clients);
		slc_capability->slc_client_cap = devm_kcalloc(dev, slc_capability->num_clients,
				sizeof(struct slc_client_capability), GFP_KERNEL);
		if (slc_capability->slc_client_cap == NULL)
			return -ENOMEM;

		qcom_msc->mpam_available = MPAM_AVAILABLE;
		client_details = &(client_info->client);
		slc_capability->num_partids = 0;
		for (client_idx = 0; client_idx < slc_capability->num_clients; client_idx++,
				client_details++) {
			client_cap = &(slc_capability->slc_client_cap[client_idx]);
			query.client_id = client_idx;
			query.part_id = 0;
			client_cap->enabled = false;
			if (slc_client_query(dev, &query, &(client_cap->client_info))) {
				pr_err("SLC MAPM ClientID %d Query Failed!\n", query.client_id);
				continue;
			}

			num_part_ids = client_cap->client_info.num_part_id;
			if ((num_part_ids == 0) || (num_part_ids == SLC_INVALID_PARTID)) {
				pr_err("SLC MAPM ClientID %d, has no valid PART ID!\n",
						query.client_id);
				continue;
			}

			client_cap->client_info.client_id = query.client_id;
			client_cap->client_name = devm_kzalloc(dev, CLIENT_NAME_LEN, GFP_KERNEL);

			client_cap->slc_partid_cap = devm_kcalloc(dev, num_part_ids,
					sizeof(union slc_partid_capability_def), GFP_KERNEL);
			if ((client_cap->client_name == NULL) ||
					(client_cap->slc_partid_cap == NULL))
				return -ENOMEM;

			strscpy(client_cap->client_name, client_details->client_name,
					CLIENT_NAME_LEN);
			client_cap->enabled = true;
			slc_capability->num_partids += num_part_ids;
			for (partid_idx = 0; partid_idx < num_part_ids; partid_idx++) {
				query.part_id = partid_idx;
				ret = msc_system_get_device_capability(qcom_msc->msc_id, &query,
						&(client_cap->slc_partid_cap[partid_idx]));
				if (ret) {
					client_cap->enabled = false;
					pr_err("SLC MAPM ClientID %d, Part ID %d, failed!\n",
							client_cap->client_info.client_id,
							query.part_id);
					continue;
				}

				pr_info("Client Name:%-12s Idx:%d, PartID:%d Monitor support %x\n",
					client_details->client_name,
					client_cap->client_info.client_id,
					query.part_id,
					client_cap->slc_partid_cap[partid_idx].v1_cap.mon_support);
			}

		}
	} else {
		slc_capability->num_clients = of_property_count_strings(node, "qcom,slc-clients");
		pr_info("Number of SLC MPAM clients %d\n", slc_capability->num_clients);
		slc_capability->slc_client_cap = devm_kcalloc(dev,
				slc_capability->num_clients, sizeof(struct slc_client_capability),
				GFP_KERNEL);
		if (slc_capability->slc_client_cap == NULL)
			return -ENOMEM;

		if (of_property_read_u32(node, "qcom,num-read-miss-cfg",
					&slc_capability->slc_mon_list.read_miss_config_available))
			return -EINVAL;

		if (of_property_read_u32(node, "qcom,num-cap-cfg",
					&slc_capability->slc_mon_list.capacity_config_available))
			return -EINVAL;

		qcom_msc->mpam_available = MPAM_AVAILABLE;
		for (client_idx = 0; client_idx < slc_capability->num_clients; client_idx++) {
			client_cap = &(slc_capability->slc_client_cap[client_idx]);
			client_cap->client_info.client_id = slc_client_id[client_idx];

			client_cap->enabled = false;
			query.client_id = slc_client_id[client_idx];
			query.part_id = 0;
			if (slc_client_query(dev, &query, &(client_cap->client_info)))
				continue;

			num_part_ids = client_cap->client_info.num_part_id;
			if ((num_part_ids == 0) || (num_part_ids == SLC_INVALID_PARTID))
				continue;

			client_cap->enabled = true;
			client_cap->slc_partid_cap = devm_kcalloc(dev, num_part_ids,
					sizeof(union slc_partid_capability_def), GFP_KERNEL);
			if (client_cap->slc_partid_cap == NULL) {
				ret = -ENOMEM;
				break;
			}

			for (partid_idx = 0; partid_idx < num_part_ids; partid_idx++) {
				query.part_id = partid_idx;
				ret = msc_system_get_device_capability(qcom_msc->msc_id, &query,
						&(client_cap->slc_partid_cap[partid_idx]));
				if (ret)
					continue;

				pr_info("SLC Client Idx:%d, PartID Idx:%d enabled\n",
						client_cap->client_info.client_id,
						query.part_id);
			}
		}

		slc_capability->num_partids = SLC_NUM_PARTIDS;
	}

	return ret;
}

static int slc_mpam_enable_disable(struct device *dev, void *msc_partid, void *msc_partconfig)
{
	struct qcom_slc_mpam_enable_cfg mon_cfg = {0};

	memcpy(&mon_cfg.query, msc_partid, sizeof(struct msc_query));
	memcpy(&mon_cfg.enable, msc_partconfig, sizeof(struct mpam_enable));

	return mpam_msc_slc_set_params(dev, &mon_cfg, sizeof(struct qcom_slc_mpam_enable_cfg),
			PARAM_SET_CONFIG_SLC_MPAM_START_STOP);
}

static int slc_mpam_start_stop(struct device *dev, uint32_t start)
{
	struct qcom_mpam_msc *qcom_msc;
	struct msc_query query = {0};
	struct mpam_enable enable = {0};

	qcom_msc = (struct qcom_mpam_msc *)dev_get_drvdata(dev);
	if (qcom_msc == NULL)
		return -EINVAL;

	query.qcom_msc_id.qcom_msc_type = qcom_msc->qcom_msc_id.qcom_msc_type;
	query.qcom_msc_id.qcom_msc_class = qcom_msc->qcom_msc_id.qcom_msc_class;
	query.qcom_msc_id.idx = qcom_msc->qcom_msc_id.idx;
	query.client_id = 0;
	query.part_id = 0;
	enable.value = start;
	return slc_mpam_enable_disable(dev, &query, &enable);
}

static int mpam_msc_slc_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;
	uint32_t val, firmware_ver;
	struct device_node *node;
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *qcom_slc_capability;
	union qcom_slc_monitor_memory *mon_mem;
	struct resource *res;

	qcom_msc = devm_kzalloc(&pdev->dev, sizeof(struct qcom_mpam_msc), GFP_KERNEL);
	if (qcom_msc == NULL)
		return -ENOMEM;

	qcom_msc->mpam_available = MPAM_UNINITIALIZAED;
	qcom_msc->sdev = get_qcom_scmi_device();
	if (IS_ERR(qcom_msc->sdev))
		return PTR_ERR(qcom_msc->sdev);

	qcom_msc->scmi_ops = qcom_msc->sdev->handle->devm_protocol_get(qcom_msc->sdev,
			QCOM_SCMI_VENDOR_PROTOCOL, &qcom_msc->ph);
	if (IS_ERR(qcom_msc->scmi_ops))
		return dev_err_probe(&pdev->dev, PTR_ERR(qcom_msc->scmi_ops),
				"Error getting vendor protocol ops\n");

	llcc_perfmon_clock = devm_clk_get(&pdev->dev, "qdss_clk");
	if (IS_ERR_OR_NULL(llcc_perfmon_clock)) {
		pr_warn("failed to get qdss clock node\n");
		llcc_perfmon_clock = NULL;
	}

	qcom_msc->qcom_msc_id.qcom_msc_type = SLC;
	qcom_msc->qcom_msc_id.qcom_msc_class = CACHE_TYPE;
	qcom_msc->ops = &slc_msc_ops;
	qcom_msc->dev = &pdev->dev;

	node = pdev->dev.of_node;
	ret = of_property_read_u32(node, "qcom,dev-index", &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "qcom,dev-index Node failed!\n");

	if (val > U8_MAX)
		return dev_err_probe(&pdev->dev, -EINVAL, "dev-index wrong value failed!\n");

	qcom_msc->qcom_msc_id.idx = val;
	ret = of_property_read_string(node, "qcom,msc-name", &qcom_msc->msc_name);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "qcom,msc-name Node failed!\n");

	ret = of_property_read_u32(node, "qcom,msc-id", &qcom_msc->msc_id);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "qcom,msc-id Node failed!\n");

	platform_set_drvdata(pdev, qcom_msc);
	/* SCMI firmware query firmware version details */
	if (slc_firmware_version_query(&pdev->dev, NULL, &firmware_ver))
		firmware_ver = SLC_MPAM_VERSION_0;

	/* Depending on firmware version SLC Monitor may memory program here */
	val = mpam_slc_mpam_init_v0;
	if (firmware_ver)
		val = mpam_slc_client_info_v1;

	if (slc_mpam_start_stop(&pdev->dev, val))
		return dev_err_probe(&pdev->dev, -EINVAL, "MAPM SCMI failure!\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mon-base");
	qcom_msc->mon_base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR_OR_NULL(qcom_msc->mon_base))
		return dev_err_probe(&pdev->dev, PTR_ERR(qcom_msc->mon_base),
				"ioremap slc mpam_mon_base failed!\n");

	mon_mem = (union qcom_slc_monitor_memory *) qcom_msc->mon_base;
	if (mon_mem->mem_v1.msc_id != qcom_msc->msc_id)
		return dev_err_probe(&pdev->dev, -EINVAL, "IMEM Init failure!\n");

	ret = attach_mpam_msc(&pdev->dev, qcom_msc, SLC);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "MPAM MSC attach failed!\n");

	qcom_msc->msc_capability = devm_kzalloc(&pdev->dev, sizeof(struct qcom_slc_capability),
			GFP_KERNEL);
	if (qcom_msc->msc_capability == NULL) {
		ret = -ENOMEM;
		goto err_detach;
	}

	qcom_slc_capability = (struct qcom_slc_capability *)qcom_msc->msc_capability;
	qcom_slc_capability->firmware_ver.firmware_version = firmware_ver;
	ret = slc_client_info_read(&pdev->dev, node);
	if (ret) {
		dev_err(&pdev->dev, "Failed to detect SLC device\n");
		goto err_detach;
	}

	if (qcom_slc_capability->firmware_ver.firmware_version != SLC_MPAM_VERSION_0) {
		/* SLC Monitor memory programmed here on updated firmware */
		ret = slc_mpam_start_stop(&pdev->dev, mpam_slc_mon_init_v1);
		if (ret) {
			dev_err(&pdev->dev, "MON Memory init Failure!\n");
			goto err_detach;
		}
	}

	dev_info(&pdev->dev, "SLC MPAM Firmware version %d.%d.%d\n",
			qcom_slc_capability->firmware_ver.ver.major,
			qcom_slc_capability->firmware_ver.ver.minor,
			qcom_slc_capability->firmware_ver.ver.fixes);
	qcom_slc_capability->slc_mon_configured.read_miss_configured = 0;
	qcom_slc_capability->slc_mon_configured.capacity_configured = 0;
	qcom_msc->mpam_available = MPAM_MONITRS_AVAILABLE;
	return 0;

err_detach:
	detach_mpam_msc(&pdev->dev, qcom_msc, SLC);
	return dev_err_probe(&pdev->dev, ret, "MPAM SLC driver probe failed!\n");
}

static void mpam_msc_slc_remove(struct platform_device *pdev)
{
	struct qcom_mpam_msc *qcom_msc;
	struct qcom_slc_capability *slc_capability;

	qcom_msc = (struct qcom_mpam_msc *)platform_get_drvdata(pdev);
	if (qcom_msc != NULL) {
		slc_capability =  (struct qcom_slc_capability *)qcom_msc->msc_capability;

		if (slc_capability->slc_mon_configured.mon_cfgd) {
			dev_err(&pdev->dev, "Monitors are configured!\n");
			return;
		}

		qcom_msc->mpam_available = MPAM_AVAILABLE;

		if (slc_mpam_start_stop(&pdev->dev, mpam_slc_reset))
			dev_err(&pdev->dev, "Failed to stop SLC Monitor thread\n");

		detach_mpam_msc(&pdev->dev, qcom_msc, SLC);
		qcom_msc->mpam_available = MPAM_UNINITIALIZAED;
	}

	platform_set_drvdata(pdev, NULL);
}
static const struct of_device_id mpam_msc_slc_table[] = {
	{ .compatible = "qcom,slc-mpam" },
	{}
};
MODULE_DEVICE_TABLE(of, mpam_msc_slc_table);

static struct platform_driver mpam_msc_slc_driver = {
	.driver = {
		.name = "mpam-msc-slc",
		.of_match_table = mpam_msc_slc_table,
		.suppress_bind_attrs = true,
	},
	.probe = mpam_msc_slc_probe,
	.remove = mpam_msc_slc_remove,
};

module_platform_driver(mpam_msc_slc_driver);

MODULE_SOFTDEP("pre: llcc_qcom");
MODULE_SOFTDEP("pre: qcom_scmi_client");
MODULE_DESCRIPTION("QCOM MPAM MSC SLC driver");
MODULE_LICENSE("GPL");
