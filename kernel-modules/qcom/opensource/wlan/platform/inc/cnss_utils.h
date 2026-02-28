/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017, 2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CNSS_UTILS_H_
#define _CNSS_UTILS_H_

#include <linux/types.h>

#define CNSS_SSR_DRIVER_DUMP_MAX_REGIONS 32

struct device;

enum cnss_utils_cc_src {
	CNSS_UTILS_SOURCE_CORE,
	CNSS_UTILS_SOURCE_11D,
	CNSS_UTILS_SOURCE_USER
};

enum cnss_utils_device_type {
	CNSS_UNSUPPORETD_DEVICE_TYPE = -1,
	CNSS_HMT_DEVICE_TYPE,
	CNSS_HSP_DEVICE_TYPE
};

enum cnss_status_type {
	CNSS_UTILS_FMD_STATUS,
	CNSS_UTILS_MAX_STATUS_TYPE
};

typedef void (*cnss_utils_status_update)(void *cb_ctx, bool status);

struct cnss_ssr_driver_dump_entry {
	char region_name[CNSS_SSR_DRIVER_DUMP_MAX_REGIONS];
	void *buffer_pointer;
	size_t buffer_size;
};

enum cnss_host_dump_type {
	CNSS_HOST_WLAN_LOGS		 = 0,
	CNSS_HOST_HTC_CREDIT		 = 1,
	CNSS_HOST_WMI_TX_CMP		 = 2,
	CNSS_HOST_WMI_COMMAND_LOG	 = 3,
	CNSS_HOST_WMI_EVENT_LOG		 = 4,
	CNSS_HOST_WMI_RX_EVENT		 = 5,
	CNSS_HOST_HAL_SOC		 = 6,
	CNSS_HOST_GWLAN_LOGGING		 = 7,
	CNSS_HOST_WMI_DEBUG_LOG_INFO	 = 8,
	CNSS_HOST_HTC_CREDIT_IDX	 = 9,
	CNSS_HOST_HTC_CREDIT_LEN	 = 10,
	CNSS_HOST_WMI_TX_CMP_IDX	 = 11,
	CNSS_HOST_WMI_COMMAND_LOG_IDX	 = 12,
	CNSS_HOST_WMI_EVENT_LOG_IDX	 = 13,
	CNSS_HOST_WMI_RX_EVENT_IDX	 = 14,
	CNSS_HOST_HIF_CE_DESC_HISTORY_BUFF = 15,
	CNSS_HOST_HANG_EVENT_DATA	 = 16,
	CNSS_HOST_CE_DESC_HIST		 = 17,
	CNSS_HOST_CE_COUNT_MAX		 = 18,
	CNSS_HOST_CE_HISTORY_MAX	 = 19,
	CNSS_HOST_ONLY_FOR_CRIT_CE	 = 20,
	CNSS_HOST_HIF_EVENT_HISTORY	 = 21,
	CNSS_HOST_HIF_EVENT_HIST_MAX	 = 22,
	CNSS_HOST_DP_WBM_DESC_REL	 = 23,
	CNSS_HOST_DP_WBM_DESC_REL_HANDLE = 24,
	CNSS_HOST_DP_TCL_CMD		 = 25,
	CNSS_HOST_DP_TCL_CMD_HANDLE	 = 26,
	CNSS_HOST_DP_TCL_STATUS		 = 27,
	CNSS_HOST_DP_TCL_STATUS_HANDLE	 = 28,
	CNSS_HOST_DP_REO_REINJ		 = 29,
	CNSS_HOST_DP_REO_REINJ_HANDLE	 = 30,
	CNSS_HOST_DP_RX_REL		 = 31,
	CNSS_HOST_DP_RX_REL_HANDLE	 = 32,
	CNSS_HOST_DP_REO_EXP		 = 33,
	CNSS_HOST_DP_REO_EXP_HANDLE	 = 34,
	CNSS_HOST_DP_REO_CMD		 = 35,
	CNSS_HOST_DP_REO_CMD_HANDLE	 = 36,
	CNSS_HOST_DP_REO_STATUS		 = 37,
	CNSS_HOST_DP_REO_STATUS_HANDLE	 = 38,
	CNSS_HOST_DP_TCL_DATA_0		 = 39,
	CNSS_HOST_DP_TCL_DATA_0_HANDLE	 = 40,
	CNSS_HOST_DP_TX_COMP_0		 = 41,
	CNSS_HOST_DP_TX_COMP_0_HANDLE	 = 42,
	CNSS_HOST_DP_TCL_DATA_1		 = 43,
	CNSS_HOST_DP_TCL_DATA_1_HANDLE	 = 44,
	CNSS_HOST_DP_TX_COMP_1		 = 45,
	CNSS_HOST_DP_TX_COMP_1_HANDLE	 = 46,
	CNSS_HOST_DP_TCL_DATA_2		 = 47,
	CNSS_HOST_DP_TCL_DATA_2_HANDLE	 = 48,
	CNSS_HOST_DP_TX_COMP_2		 = 49,
	CNSS_HOST_DP_TX_COMP_2_HANDLE	 = 50,
	CNSS_HOST_DP_REO_DST_0		 = 51,
	CNSS_HOST_DP_REO_DST_0_HANDLE	 = 52,
	CNSS_HOST_DP_REO_DST_1		 = 53,
	CNSS_HOST_DP_REO_DST_1_HANDLE	 = 54,
	CNSS_HOST_DP_REO_DST_2		 = 55,
	CNSS_HOST_DP_REO_DST_2_HANDLE	 = 56,
	CNSS_HOST_DP_REO_DST_3		 = 57,
	CNSS_HOST_DP_REO_DST_3_HANDLE	 = 58,
	CNSS_HOST_DP_REO_DST_4		 = 59,
	CNSS_HOST_DP_REO_DST_4_HANDLE	 = 60,
	CNSS_HOST_DP_REO_DST_5		 = 61,
	CNSS_HOST_DP_REO_DST_5_HANDLE	 = 62,
	CNSS_HOST_DP_REO_DST_6		 = 63,
	CNSS_HOST_DP_REO_DST_6_HANDLE	 = 64,
	CNSS_HOST_DP_REO_DST_7		 = 65,
	CNSS_HOST_DP_REO_DST_7_HANDLE	 = 66,
	CNSS_HOST_DP_PDEV_0		 = 67,
	CNSS_HOST_DP_WLAN_CFG_CTX	 = 68,
	CNSS_HOST_DP_SOC		 = 69,
	CNSS_HOST_HAL_RX_FST		 = 70,
	CNSS_HOST_DP_FISA		 = 71,
	CNSS_HOST_DP_FISA_HW_FSE_TABLE	 = 72,
	CNSS_HOST_DP_FISA_SW_FSE_TABLE	 = 73,
	CNSS_HOST_HIF			 = 74,
	CNSS_HOST_QDF_NBUF_HIST		 = 75,
	CNSS_HOST_TCL_WBM_MAP		 = 76,
	CNSS_HOST_RX_MAC_BUF_RING_0	 = 77,
	CNSS_HOST_RX_MAC_BUF_RING_0_HANDLE = 78,
	CNSS_HOST_RX_MAC_BUF_RING_1	 = 79,
	CNSS_HOST_RX_MAC_BUF_RING_1_HANDLE = 80,
	CNSS_HOST_RX_REFILL_0		 = 81,
	CNSS_HOST_RX_REFILL_0_HANDLE	 = 82,
	CNSS_HOST_CE_0			 = 83,
	CNSS_HOST_CE_0_SRC_RING		 = 84,
	CNSS_HOST_CE_0_SRC_RING_CTX	 = 85,
	CNSS_HOST_CE_1			 = 86,
	CNSS_HOST_CE_1_STATUS_RING	 = 87,
	CNSS_HOST_CE_1_STATUS_RING_CTX	 = 88,
	CNSS_HOST_CE_1_DEST_RING	 = 89,
	CNSS_HOST_CE_1_DEST_RING_CTX	 = 90,
	CNSS_HOST_CE_2			 = 91,
	CNSS_HOST_CE_2_STATUS_RING	 = 92,
	CNSS_HOST_CE_2_STATUS_RING_CTX	 = 93,
	CNSS_HOST_CE_2_DEST_RING	 = 94,
	CNSS_HOST_CE_2_DEST_RING_CTX	 = 95,
	CNSS_HOST_CE_3			 = 96,
	CNSS_HOST_CE_3_SRC_RING		 = 97,
	CNSS_HOST_CE_3_SRC_RING_CTX	 = 98,
	CNSS_HOST_CE_4			 = 99,
	CNSS_HOST_CE_4_SRC_RING		 = 100,
	CNSS_HOST_CE_4_SRC_RING_CTX	 = 101,
	CNSS_HOST_CE_5			 = 102,
	CNSS_HOST_CE_6			 = 103,
	CNSS_HOST_CE_7			 = 104,
	CNSS_HOST_CE_7_STATUS_RING	 = 105,
	CNSS_HOST_CE_7_STATUS_RING_CTX	 = 106,
	CNSS_HOST_CE_7_DEST_RING	 = 107,
	CNSS_HOST_CE_7_DEST_RING_CTX	 = 108,
	CNSS_HOST_CE_8			 = 109,
	CNSS_HOST_DP_TCL_DATA_3		 = 110,
	CNSS_HOST_DP_TCL_DATA_3_HANDLE	 = 111,
	CNSS_HOST_DP_TX_COMP_3		 = 112,
	CNSS_HOST_DP_TX_COMP_3_HANDLE	 = 113,
	CNSS_HOST_DUMP_TYPE_MAX		 = 114,
};

extern int cnss_utils_set_wlan_unsafe_channel(struct device *dev,
					      u16 *unsafe_ch_list,
					      u16 ch_count);
extern int cnss_utils_get_wlan_unsafe_channel(struct device *dev,
					      u16 *unsafe_ch_list,
					      u16 *ch_count, u16 buf_len);
extern enum cnss_utils_device_type cnss_utils_update_device_type(
				enum cnss_utils_device_type device_type);
extern int cnss_utils_fmd_status(int is_enabled);
extern int
cnss_utils_register_status_notifier(enum cnss_status_type status_type,
				    cnss_utils_status_update status_update_cb,
				    void *cb_ctx);
extern int cnss_utils_wlan_set_dfs_nol(struct device *dev,
				       const void *info, u16 info_len);
extern int cnss_utils_wlan_get_dfs_nol(struct device *dev,
				       void *info, u16 info_len);
extern int cnss_utils_get_driver_load_cnt(struct device *dev);
extern void cnss_utils_increment_driver_load_cnt(struct device *dev);
extern int cnss_utils_set_wlan_mac_address(const u8 *in, uint32_t len);
extern u8 *cnss_utils_get_wlan_mac_address(struct device *dev, uint32_t *num);
extern int cnss_utils_set_wlan_derived_mac_address(const u8 *in, uint32_t len);
extern u8 *cnss_utils_get_wlan_derived_mac_address(struct device *dev,
							uint32_t *num);
extern void cnss_utils_set_cc_source(struct device *dev,
				     enum cnss_utils_cc_src cc_source);
extern enum cnss_utils_cc_src cnss_utils_get_cc_source(struct device *dev);

#ifdef CONFIG_FEATURE_SMEM_MAILBOX
extern int cnss_utils_smem_mailbox_write(struct device *dev, int flags,
					 const __u8 *data, uint32_t len);
#endif

#endif
