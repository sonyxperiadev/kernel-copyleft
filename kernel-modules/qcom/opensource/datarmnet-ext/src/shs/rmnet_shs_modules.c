// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "rmnet_shs.h"

unsigned int rmnet_shs_wq_interval_ms __read_mostly = RMNET_SHS_WQ_INTERVAL_MS;
module_param(rmnet_shs_wq_interval_ms, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_wq_interval_ms, "Interval between wq runs (ms)");

unsigned long rmnet_shs_max_flow_inactivity_sec __read_mostly =
						RMNET_SHS_MAX_SKB_INACTIVE_TSEC;
module_param(rmnet_shs_max_flow_inactivity_sec, ulong, 0644);
MODULE_PARM_DESC(rmnet_shs_max_flow_inactivity_sec,
		 "Max flow inactive time before clean up");

unsigned int rmnet_shs_wq_tuning __read_mostly = 80;
module_param(rmnet_shs_wq_tuning, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_wq_tuning, "moving average weightage");

unsigned long long rmnet_shs_cpu_rx_max_pps_thresh[MAX_CPUS]__read_mostly = {
			 RMNET_SHS_UDP_PPS_LPWR_CPU0_UTHRESH,
			 RMNET_SHS_UDP_PPS_LPWR_CPU_UTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH,
			 RMNET_SHS_UDP_PPS_LPWR_CPU_UTHRESH,
			 RMNET_SHS_UDP_PPS_LPWR_CPU_UTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_UTHRESH
			};
module_param_array(rmnet_shs_cpu_rx_max_pps_thresh, ullong, NULL , 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_max_pps_thresh, "Max pkts core can handle");

unsigned long long rmnet_shs_cpu_rx_min_pps_thresh[MAX_CPUS]__read_mostly = {
			 RMNET_SHS_UDP_PPS_LPWR_CPU_LTHRESH,
			 RMNET_SHS_UDP_PPS_LPWR_CPU_LTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_LTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_LTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_LTHRESH,
			 RMNET_SHS_UDP_PPS_LPWR_CPU_LTHRESH,
			 RMNET_SHS_UDP_PPS_LPWR_CPU_LTHRESH,
			 RMNET_SHS_UDP_PPS_PERF_CPU_LTHRESH
			};
module_param_array(rmnet_shs_cpu_rx_min_pps_thresh, ullong, NULL , 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_min_pps_thresh, "Min pkts core can handle");

unsigned int rmnet_shs_cpu_rx_flows[MAX_CPUS];
module_param_array(rmnet_shs_cpu_rx_flows, uint, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_flows, "Num flows processed per core");

unsigned int rmnet_shs_cpu_rx_filter_flows[MAX_CPUS];
module_param_array(rmnet_shs_cpu_rx_filter_flows, uint, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_filter_flows, "Num filtered flows per core");

unsigned long long rmnet_shs_cpu_rx_bytes[MAX_CPUS];
module_param_array(rmnet_shs_cpu_rx_bytes, ullong, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_bytes, "SHS stamp bytes per CPU");

unsigned long long rmnet_shs_cpu_rx_pkts[MAX_CPUS];
module_param_array(rmnet_shs_cpu_rx_pkts, ullong, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_pkts, "SHS stamp total pkts per CPU");

unsigned long long rmnet_shs_cpu_rx_bps[MAX_CPUS];
module_param_array(rmnet_shs_cpu_rx_bps, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_bps, "SHS stamp enq rate per CPU");

unsigned long long rmnet_shs_cpu_rx_pps[MAX_CPUS];
module_param_array(rmnet_shs_cpu_rx_pps, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_rx_pps, "SHS stamp pkt enq rate per CPU");

unsigned long long rmnet_shs_cpu_qhead_diff[MAX_CPUS];
module_param_array(rmnet_shs_cpu_qhead_diff, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_qhead_diff, "SHS nw stack queue processed diff");

unsigned long long rmnet_shs_cpu_qhead_total[MAX_CPUS];
module_param_array(rmnet_shs_cpu_qhead_total, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_cpu_qhead_total, "SHS nw queue processed total");

unsigned long rmnet_shs_flow_hash[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_hash, ulong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_hash, "SHS stamp hash flow");

unsigned long rmnet_shs_flow_proto[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_proto, ulong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_proto, "SHS stamp hash transport protocol");

unsigned long long rmnet_shs_flow_inactive_tsec[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_inactive_tsec, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_inactive_tsec, "SHS stamp inactive flow time");

int rmnet_shs_flow_cpu[MAX_SUPPORTED_FLOWS_DEBUG] = {
			-1, -1, -1, -1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1, -1, -1, -1};
module_param_array(rmnet_shs_flow_cpu, int, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_cpu, "SHS stamp flow processing CPU");

int rmnet_shs_flow_cpu_recommended[MAX_SUPPORTED_FLOWS_DEBUG] = {
			 -1, -1, -1, -1, -1, -1, -1, -1,
			 -1, -1, -1, -1, -1, -1, -1, -1
			 };
module_param_array(rmnet_shs_flow_cpu_recommended, int, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_flow_cpu_recommended, "SHS stamp flow proc CPU");

unsigned long long rmnet_shs_flow_rx_bytes[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_rx_bytes, ullong, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_flow_rx_bytes, "SHS stamp bytes per flow");

unsigned long long rmnet_shs_flow_rx_pkts[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_rx_pkts, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_rx_pkts, "SHS stamp total pkts per flow");

unsigned long long rmnet_shs_flow_rx_bps[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_rx_bps, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_rx_bps, "SHS stamp enq rate per flow");

unsigned long long rmnet_shs_flow_rx_pps[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_rx_pps, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_rx_pps, "SHS stamp pkt enq rate per flow");

/* Counters for suggestions made by wq */
unsigned long long rmnet_shs_flow_silver_to_gold[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_silver_to_gold, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_silver_to_gold, "SHS Suggest Silver to Gold");

unsigned long long rmnet_shs_flow_gold_to_silver[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_gold_to_silver, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_gold_to_silver, "SHS Suggest Gold to Silver");

unsigned long long rmnet_shs_flow_gold_balance[MAX_SUPPORTED_FLOWS_DEBUG];
module_param_array(rmnet_shs_flow_gold_balance, ullong, NULL , 0444);
MODULE_PARM_DESC(rmnet_shs_flow_gold_balance, "SHS Suggest Gold Balance");

unsigned long rmnet_shs_switch_reason[RMNET_SHS_SWITCH_MAX_REASON];
module_param_array(rmnet_shs_switch_reason, ulong, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_switch_reason, "rmnet shs skb core swtich type");

unsigned long rmnet_shs_flush_reason[RMNET_SHS_FLUSH_MAX_REASON];
module_param_array(rmnet_shs_flush_reason, ulong, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_flush_reason, "rmnet shs skb flush trigger type");

unsigned int rmnet_shs_byte_store_limit __read_mostly = 30144000;
module_param(rmnet_shs_byte_store_limit, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_byte_store_limit, "Maximum byte module will park");

unsigned int rmnet_shs_pkts_store_limit __read_mostly = 24000;
module_param(rmnet_shs_pkts_store_limit, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_pkts_store_limit, "Maximum pkts module will park");

unsigned int rmnet_shs_max_core_wait __read_mostly = 55;
module_param(rmnet_shs_max_core_wait, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_max_core_wait,
		 "Max wait module will wait during move to perf core in ms");

unsigned int rmnet_shs_inst_rate_interval __read_mostly = 20;
module_param(rmnet_shs_inst_rate_interval, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_inst_rate_interval,
		 "Max interval we sample for instant burst prioritizing");

unsigned int rmnet_shs_inst_rate_switch __read_mostly = 0;
module_param(rmnet_shs_inst_rate_switch, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_inst_rate_switch,
		 "Configurable option to enable rx rate cpu switching");

unsigned int rmnet_shs_fall_back_timer __read_mostly = 1;
module_param(rmnet_shs_fall_back_timer, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_fall_back_timer,
		 "Option to enable fall back limit for parking");

unsigned int rmnet_shs_cpu_backlog_max_pkts[MAX_CPUS] = {
			900, 1100, 1100, 1100,  1100, 1100, 1100, 1100};
module_param_array(rmnet_shs_cpu_backlog_max_pkts, uint, NULL, 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_backlog_max_pkts,
		 "Max pkts in backlog prioritizing");

/*
unsigned int rmnet_shs_inst_rate_max_pkts __read_mostly = 2500;
module_param(rmnet_shs_inst_rate_max_pkts, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_inst_rate_max_pkts,
		 "Max pkts in a instant burst interval before prioritizing");
*/
unsigned int rmnet_shs_cpu_inst_rate_max_pkts[MAX_CPUS] = {
			3100, 3100, 3100, 3100,  3100, 3100, 3100, 3100};
module_param_array(rmnet_shs_cpu_inst_rate_max_pkts, uint, NULL, 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_inst_rate_max_pkts, "Max pkts in a burst before prioritizing");

unsigned int rmnet_shs_timeout __read_mostly = 6;
module_param(rmnet_shs_timeout, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_timeout, "Option to configure fall back duration");

unsigned int rmnet_shs_switch_cores __read_mostly = 1;
module_param(rmnet_shs_switch_cores, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_switch_cores, "Switch core upon hitting threshold");

unsigned int rmnet_shs_cpu_max_qdiff[MAX_CPUS];
module_param_array(rmnet_shs_cpu_max_qdiff, uint, NULL, 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_max_qdiff, "Max queue length seen of each core");

unsigned int rmnet_shs_cpu_ooo_count[MAX_CPUS];
module_param_array(rmnet_shs_cpu_ooo_count, uint, NULL, 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_ooo_count, "OOO count for each cpu");

unsigned int rmnet_shs_cpu_max_coresum[MAX_CPUS];
module_param_array(rmnet_shs_cpu_max_coresum, uint, NULL, 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_max_coresum, "Max coresum seen of each core");

unsigned int rmnet_shs_cpu_prio_dur __read_mostly = 3;
module_param(rmnet_shs_cpu_prio_dur, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_cpu_prio_dur, "Staying duration for netrx switch");

unsigned int rmnet_oom_pkt_limit __read_mostly = 5000;
module_param(rmnet_oom_pkt_limit, uint, 0644);
MODULE_PARM_DESC(rmnet_oom_pkt_limit, "Max rmnet pre-backlog");

/* Reserve mask is now only indicating audio reservations we are honoring*/
unsigned int rmnet_shs_reserve_mask __read_mostly = 0;
module_param(rmnet_shs_reserve_mask, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_resever_mask, "rmnet_shs_reserve_mask");

/* Halt mask is now a super set of reserve mask */
unsigned int rmnet_shs_halt_mask __read_mostly = 0;
module_param(rmnet_shs_halt_mask, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_halt_mask, "rmnet_shs_halt_mask");

unsigned int rmnet_shs_debug __read_mostly;
module_param(rmnet_shs_debug, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_debug, "rmnet_shs_debug");

unsigned int rmnet_shs_stats_enabled __read_mostly = 1;
module_param(rmnet_shs_stats_enabled, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_stats_enabled, "Enable Disable stats collection");

unsigned long rmnet_shs_mid_err[RMNET_SHS_MID_ERR_MAX];
module_param_array(rmnet_shs_mid_err, ulong, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_mid_err, "rmnet shs mid error type");

unsigned long rmnet_shs_crit_err[RMNET_SHS_CRIT_ERR_MAX];
module_param_array(rmnet_shs_crit_err, ulong, NULL, 0444);
MODULE_PARM_DESC(rmnet_shs_crit_err, "rmnet shs crtical error type");

unsigned int rmnet_shs_ll_flow_cpu = DEF_LL_CORE;
module_param(rmnet_shs_ll_flow_cpu, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_ll_flow_cpu, "Current LL flow cpu ");

unsigned int rmnet_shs_ll_phy_cpu = 2;
module_param(rmnet_shs_ll_phy_cpu, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_ll_phy_cpu, "Current LL phy cpu ");

unsigned int rmnet_shs_wq_tick = 0;
module_param(rmnet_shs_wq_tick, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_wq_tick, "rmnet_shs_Wq execution tick ");

unsigned int rmnet_shs_pause_count = 0;
module_param(rmnet_shs_pause_count, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_pause_count, "rmnet shs wq pause count");

unsigned int rmnet_shs_restart_count = 0;
module_param(rmnet_shs_restart_count, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_restart_count, "rmnet shs wq restart count");

unsigned int rmnet_shs_reserve_on = 1;
module_param(rmnet_shs_reserve_on, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_reserve_on, "reserve mask feature flag");

unsigned int rmnet_shs_no_sync_off  __read_mostly = 0;
module_param(rmnet_shs_no_sync_off, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_no_sync_off, "rmnet no sync feature toggle");

unsigned int rmnet_shs_no_sync_packets = 0;
module_param(rmnet_shs_no_sync_packets, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_no_sync_packets, "rmnet shs async packet count");
