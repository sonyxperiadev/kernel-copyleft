/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _QCOM_MPAM_SLC_H
#define _QCOM_MPAM_SLC_H

enum slc_clients_id {
	APPS,
	GPU,
	NSP,
	SLC_CLIENT_MAX,
};

enum slc_gears {
	SLC_GEAR_HIGH,
	SLC_GEAR_LOW,
	SLC_GEAR_BYPASS,
	MAX_NUM_GEARS,
};

static char gear_index[][25] = {
	"SLC_GEAR_HIGH",
	"SLC_GEAR_LOW",
	"SLC_GEAR_BYPASS",
	"",
};

#define CLIENT_NAME_LEN         16

#define QCOM_SLC_MPAM_SCMI_STR	0x534c434d50414d /* SLCMPAM */
#define SLC_INVALID_PARTID      ((1 << 16) - 1)

#define SLC_MPAM_VERSION_0	0x00000000	/* Base firmware */

enum mpam_slc_get_param_ids {
	PARAM_GET_CLIENT_INFO_MSC = 1,
	PARAM_GET_CACHE_CAPABILITY_MSC = 2,
	PARAM_GET_CACHE_PARTITION_MSC = 3,
	PARAM_GET_SLC_MPAM_VERSION = 4,
};

enum mpam_slc_set_param_ids {
	PARAM_SET_CACHE_PARTITION_MSC = 1,
	PARAM_RESET_CACHE_PARTITION_MSC = 2,
	PARAM_SET_CONFIG_MON_MSC = 3,
	PARAM_SET_CONFIG_SLC_MPAM_START_STOP = 4,
};

/* GET_PARAM */
/* PARAM_GET_CLIENT_INFO_MSC  */
struct slc_client_info {
	uint16_t client_id;
	uint16_t num_part_id;
} __packed;

/* PARAM_GET_CACHE_CAPABILITY_MSC */
struct slc_partid_capability {
	uint8_t part_id;
	uint8_t num_gears;
	uint8_t part_id_gears[MAX_NUM_GEARS];
} __packed;

struct slc_partid_capacity_config {
	uint32_t gear_flds_bitmap;
	uint16_t dflt_bitmap;
	uint32_t slc_bitfield_capacity;
} __packed;

enum slc_mintor_support {
	cap_mon_support,
	read_miss_mon_support,
	fe_mon_support,
	be_mon_support,
};

struct slc_partid_capability_v1 {
	uint8_t part_id;
	uint8_t num_gears;
	struct slc_partid_capacity_config cap_cfg;
	uint32_t mon_support;
} __packed;

union slc_partid_capability_def {
	struct slc_partid_capability v0_cap;
	struct slc_partid_capability_v1 v1_cap;
} __packed;

/* PARAM_GET_CACHE_PARTITION_MSC */
struct qcom_slc_gear_val {
	uint32_t gear_val;
} __packed;

/* PARAM_GET_SLC_MPAM_VERSION */
struct qcom_slc_firmware_version_flds {
	uint32_t fixes :8;
	uint32_t minor :8;
	uint32_t major :16;
};
union qcom_slc_firmware_version {
	uint32_t firmware_version;
	struct qcom_slc_firmware_version_flds ver;
} __packed;

/* SET_PARAM */
/* PARAM_SET_CACHE_PARTITION_MSC  */
/* PARAM_RESET_CACHE_PARTITION_MSC */
struct slc_partid_config {
	struct msc_query query;
	struct qcom_slc_gear_val gear_config;
} __packed;

/* PARAM_SET_CONFIG_MON_MSC */
enum slc_mon_function {
	CACHE_CAPACITY_CONFIG,
	CACHE_READ_MISS_CONFIG,
	CACHE_FE_MON_CONFIG,
	CACHE_BE_MON_CONFIG,
	CACHE_MON_STATS_READ,
};

struct slc_mon_config_val {
	uint32_t slc_mon_function;
	uint32_t enable;
} __packed;

struct slc_mon_config {
	struct msc_query query;
	struct slc_mon_config_val config;
} __packed;

/* PARAM_SET_CONFIG_SLC_MPAM_START_STOP */
enum mpam_enable_val {
	mpam_slc_reset = 0,
	mpam_slc_mpam_init_v0 = 1,
	mpam_slc_client_info_v1 = 2,
	mpam_slc_mon_init_v1 = 3,
};

struct mpam_enable {
	uint32_t value;
} __packed;

struct qcom_slc_mpam_enable_cfg {
	struct msc_query query;
	struct mpam_enable enable;
} __packed;

/* shared memory SLC monitor */
struct slc_capacity {
	uint32_t num_cache_lines;
	uint32_t cap_enabled;
} __packed;

struct slc_read_miss_cntr {
	uint64_t rd_misses;
	uint32_t cntr_index;
	uint32_t miss_enabled;
} __packed;

struct slc_fe_bw_info {
	uint64_t slc_fe_bytes;
	uint32_t cap_enabled;
} __packed;

struct slc_be_bw_info {
	uint64_t slc_be_bytes;
	uint32_t cap_enabled;
} __packed;

struct slc_partid_info {
	uint32_t client_id;
	uint32_t part_id;
} __packed;

struct slc_client_details {
	uint16_t client_id;
	char client_name[CLIENT_NAME_LEN];
} __packed;

struct slc_mon_details {
	uint32_t num_cap_monitor;
	uint32_t num_miss_monitor;
	uint32_t num_slc_fe_bw_mnitor;
	uint32_t num_slc_be_bw_mnitor;
} __packed;

struct slc_sct_client_info {
	uint32_t num_clients;
	struct slc_mon_details slc_mon_info;
	struct slc_client_details client;
} __packed;


struct qcom_slc_mon_data {
	struct slc_partid_info part_info;
	struct slc_capacity cap_stats;
	struct slc_read_miss_cntr rd_miss_stats;
} __packed;

struct qcom_slc_mon_data_v1 {
	struct slc_partid_info part_info;
	uint16_t cntr_index;
	uint16_t mon_enabled;
	uint32_t num_cache_lines;
	uint64_t rd_misses;
	uint64_t fe_rd_bytes;
	uint64_t fe_wr_bytes;
	uint64_t be_rd_bytes;
	uint64_t be_wr_bytes;
	uint64_t rd_hits;
	uint64_t rd_incr;
	uint64_t wr_hits;
	uint64_t wr_incr;
} __packed;

#define SLC_NUM_PARTIDS		5
struct qcom_slc_mon_mem_v0 {
	uint32_t match_seq;
	uint16_t msc_id;
	uint16_t num_active_mon;
	struct qcom_slc_mon_data data[SLC_NUM_PARTIDS];
	uint64_t last_capture_time;
} __packed;

struct qcom_slc_mon_mem_v1 {
	uint32_t match_seq;
	uint16_t msc_id;
	uint16_t num_active_mon;
	uint64_t last_capture_time;
	uint64_t slc_mpam_monitor_size;
	struct qcom_slc_mon_data_v1 data[];
} __packed;

union qcom_slc_monitor_memory {
	struct qcom_slc_mon_mem_v0 mem_v0;
	struct qcom_slc_mon_mem_v1 mem_v1;
};

/* slc Monitor capability */
struct slc_mon_capability {
	uint32_t read_miss_config_available;
	uint32_t capacity_config_available;
};

struct slc_mon_configured {
	uint32_t read_miss_configured;
	uint32_t capacity_configured;
	uint32_t mon_cfgd;
};

/* msc slc capability */
struct slc_client_capability {
	struct slc_client_info client_info;
	union slc_partid_capability_def *slc_partid_cap;
	uint8_t enabled;
	char *client_name;
} __packed;

struct qcom_slc_capability {
	uint32_t num_clients;
	struct slc_client_capability *slc_client_cap;
	struct slc_mon_capability slc_mon_list;
	struct slc_mon_configured slc_mon_configured;
	union qcom_slc_firmware_version firmware_ver;
	uint32_t num_partids;
} __packed;

/* slc mon API parameters */
struct capacity_info  {
	uint32_t slc_mon_function;
	uint64_t last_capture_time;
	uint32_t num_cache_lines;
} __packed;

struct miss_info  {
	uint32_t slc_mon_function;
	uint64_t last_capture_time;
	uint64_t num_rd_misses;
} __packed;

struct slc_fe_stats  {
	uint32_t slc_mon_function;
	uint64_t last_capture_time;
	uint64_t slc_fe_bytes;
} __packed;

struct slc_be_stats  {
	uint32_t slc_mon_function;
	uint64_t last_capture_time;
	uint64_t slc_be_bytes;
} __packed;

struct slc_partid_mon_stats  {
	uint32_t slc_mon_function;
	uint64_t last_capture_time;
	uint32_t num_cache_lines;
	uint64_t num_rd_misses;
	uint64_t slc_fe_bytes;
	uint64_t slc_be_bytes;
} __packed;

struct mon_ref {
	uint32_t slc_mon_function;
	uint64_t last_capture_time;
	uint64_t mon_data;
} __packed;

union mon_values {
	struct capacity_info capacity;
	struct miss_info misses;
	struct slc_fe_stats fe_stats;
	struct slc_be_stats be_stats;
	struct slc_partid_mon_stats mon_stats;
	struct mon_ref ref;
} __packed;

#endif /* _QCOM_MPAM_SLC_H */
