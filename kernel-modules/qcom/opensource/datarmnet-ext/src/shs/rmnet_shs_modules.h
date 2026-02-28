/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include  "rmnet_shs_wq.h"
#include  "rmnet_shs_common.h"
#include  "rmnet_shs.h"

#ifndef _RMNET_SHS_MODULES_H_
#define _RMNET_SHS_MODULES_H_

extern unsigned int rmnet_shs_wq_interval_ms;
extern unsigned long rmnet_shs_max_flow_inactivity_sec;
extern unsigned int rmnet_shs_wq_tuning __read_mostly;
extern unsigned long long rmnet_shs_cpu_rx_max_pps_thresh[MAX_CPUS];
extern unsigned long long rmnet_shs_cpu_rx_min_pps_thresh[MAX_CPUS];
extern unsigned int rmnet_shs_cpu_rx_flows[MAX_CPUS];
extern unsigned int rmnet_shs_cpu_rx_filter_flows[MAX_CPUS];
extern unsigned long long rmnet_shs_cpu_rx_bytes[MAX_CPUS];
extern unsigned long long rmnet_shs_cpu_rx_pkts[MAX_CPUS];
extern unsigned long long rmnet_shs_cpu_rx_bps[MAX_CPUS];
extern unsigned long long rmnet_shs_cpu_rx_pps[MAX_CPUS];
extern unsigned long long rmnet_shs_cpu_qhead_diff[MAX_CPUS];
extern unsigned long long rmnet_shs_cpu_qhead_total[MAX_CPUS];
extern unsigned long rmnet_shs_flow_hash[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long rmnet_shs_flow_proto[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_inactive_tsec[MAX_SUPPORTED_FLOWS_DEBUG];
extern int rmnet_shs_flow_cpu[MAX_SUPPORTED_FLOWS_DEBUG];
extern int rmnet_shs_flow_cpu_recommended[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_rx_bytes[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_rx_pkts[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_rx_bps[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_rx_pps[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_silver_to_gold[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_gold_to_silver[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long long rmnet_shs_flow_gold_balance[MAX_SUPPORTED_FLOWS_DEBUG];
extern unsigned long rmnet_shs_switch_reason[RMNET_SHS_SWITCH_MAX_REASON];
extern unsigned long rmnet_shs_flush_reason[RMNET_SHS_FLUSH_MAX_REASON];
extern unsigned int rmnet_shs_byte_store_limit;
extern unsigned int rmnet_shs_pkts_store_limit;
extern unsigned int rmnet_shs_max_core_wait;
extern unsigned int rmnet_shs_inst_rate_interval;
extern unsigned int rmnet_shs_inst_rate_switch;
extern unsigned int rmnet_shs_fall_back_timer;
extern unsigned int rmnet_shs_cpu_backlog_max_pkts[MAX_CPUS];
extern unsigned int rmnet_shs_cpu_inst_rate_max_pkts[MAX_CPUS];
extern unsigned int rmnet_shs_timeout;
extern unsigned int rmnet_shs_switch_cores;
extern unsigned int rmnet_shs_cpu_max_qdiff[MAX_CPUS];
extern unsigned int rmnet_shs_cpu_ooo_count[MAX_CPUS];
extern unsigned int rmnet_shs_cpu_max_coresum[MAX_CPUS];
extern unsigned int rmnet_shs_cpu_prio_dur;
extern unsigned int rmnet_oom_pkt_limit;
extern unsigned int rmnet_shs_debug;
extern unsigned int rmnet_shs_stats_enabled __read_mostly;
extern unsigned long rmnet_shs_crit_err[RMNET_SHS_CRIT_ERR_MAX];
extern unsigned long rmnet_shs_mid_err[RMNET_SHS_MID_ERR_MAX];
extern unsigned int rmnet_shs_ll_flow_cpu;
extern unsigned int rmnet_shs_ll_phy_cpu;
extern unsigned int rmnet_shs_halt_mask;
extern unsigned int rmnet_shs_reserve_mask;
extern unsigned int rmnet_shs_wq_tick;
extern unsigned int rmnet_shs_pause_count;
extern unsigned int rmnet_shs_restart_count;
extern unsigned int rmnet_shs_no_sync_packets;
extern unsigned int rmnet_shs_no_sync_off;
extern unsigned int rmnet_shs_reserve_on;
#endif
