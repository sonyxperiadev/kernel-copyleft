/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _NET_CNSS2_H
#define _NET_CNSS2_H

#include <linux/pci.h>

#define CNSS_MAX_FILE_NAME		20
#define CNSS_MAX_TIMESTAMP_LEN		32
#define CNSS_WLFW_MAX_BUILD_ID_LEN	128
#define CNSS_MAX_DEV_MEM_NUM		4
#define CNSS_CHIP_VER_ANY		0

#define CNSS_SSR_DRIVER_DUMP_MAX_REGIONS 32

enum cnss_bus_width_type {
	CNSS_BUS_WIDTH_NONE,
	CNSS_BUS_WIDTH_IDLE,
	CNSS_BUS_WIDTH_LOW,
	CNSS_BUS_WIDTH_MEDIUM,
	CNSS_BUS_WIDTH_HIGH,
	CNSS_BUS_WIDTH_VERY_HIGH,
	CNSS_BUS_WIDTH_LOW_LATENCY
};

enum cnss_platform_cap_flag {
	CNSS_HAS_EXTERNAL_SWREG = 0x01,
	CNSS_HAS_UART_ACCESS = 0x02,
	CNSS_HAS_DRV_SUPPORT = 0x04,
};

struct cnss_platform_cap {
	u32 cap_flag;
};

struct cnss_fw_files {
	char image_file[CNSS_MAX_FILE_NAME];
	char board_data[CNSS_MAX_FILE_NAME];
	char otp_data[CNSS_MAX_FILE_NAME];
	char utf_file[CNSS_MAX_FILE_NAME];
	char utf_board_data[CNSS_MAX_FILE_NAME];
	char epping_file[CNSS_MAX_FILE_NAME];
	char evicted_data[CNSS_MAX_FILE_NAME];
};

struct cnss_device_version {
	u32 family_number;
	u32 device_number;
	u32 major_version;
	u32 minor_version;
};

struct cnss_dev_mem_info {
	u64 start;
	u64 size;
};

struct cnss_soc_info {
	void __iomem *va;
	phys_addr_t pa;
	uint32_t chip_id;
	uint32_t chip_family;
	uint32_t board_id;
	uint32_t soc_id;
	uint32_t fw_version;
	char fw_build_timestamp[CNSS_MAX_TIMESTAMP_LEN + 1];
	struct cnss_device_version device_version;
	struct cnss_dev_mem_info dev_mem_info[CNSS_MAX_DEV_MEM_NUM];
	char fw_build_id[CNSS_WLFW_MAX_BUILD_ID_LEN + 1];
};

struct cnss_wlan_runtime_ops {
	int (*runtime_suspend)(struct pci_dev *pdev);
	int (*runtime_resume)(struct pci_dev *pdev);
};

enum cnss_driver_status {
	CNSS_UNINITIALIZED,
	CNSS_INITIALIZED,
	CNSS_LOAD_UNLOAD,
	CNSS_RECOVERY,
	CNSS_FW_DOWN,
	CNSS_HANG_EVENT,
	CNSS_BUS_EVENT,
	CNSS_SYS_REBOOT,
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

enum cnss_bus_event_type {
	BUS_EVENT_PCI_LINK_DOWN = 0,
	BUS_EVENT_PCI_LINK_RESUME_FAIL = 1,

	BUS_EVENT_INVALID = 0xFFFF,
};

enum cnss_wfc_mode {
	CNSS_WFC_MODE_OFF,
	CNSS_WFC_MODE_ON,
};

struct cnss_wfc_cfg {
	enum cnss_wfc_mode mode;
};

struct cnss_hang_event {
	void *hang_event_data;
	u16 hang_event_data_len;
};

struct cnss_bus_event {
	enum cnss_bus_event_type etype;
	void *event_data;
};

struct cnss_uevent_data {
	enum cnss_driver_status status;
	void *data;
};

struct cnss_ssr_driver_dump_entry {
	char region_name[CNSS_SSR_DRIVER_DUMP_MAX_REGIONS];
	void *buffer_pointer;
	size_t buffer_size;
};


struct cnss_wlan_driver {
	char *name;
	int  (*probe)(struct pci_dev *pdev, const struct pci_device_id *id);
	void (*remove)(struct pci_dev *pdev);
	int (*idle_restart)(struct pci_dev *pdev,
			    const struct pci_device_id *id);
	int  (*idle_shutdown)(struct pci_dev *pdev);
	int  (*reinit)(struct pci_dev *pdev, const struct pci_device_id *id);
	void (*shutdown)(struct pci_dev *pdev);
	void (*crash_shutdown)(struct pci_dev *pdev);
	int  (*suspend)(struct pci_dev *pdev, pm_message_t state);
	int  (*resume)(struct pci_dev *pdev);
	int  (*suspend_noirq)(struct pci_dev *pdev);
	int  (*resume_noirq)(struct pci_dev *pdev);
	void (*modem_status)(struct pci_dev *pdev, int state);
	void (*update_status)(struct pci_dev *pdev, uint32_t status);
	int  (*update_event)(struct pci_dev *pdev,
			     struct cnss_uevent_data *uevent);
	struct cnss_wlan_runtime_ops *runtime_ops;
	const struct pci_device_id *id_table;
	u32 chip_version;
	enum cnss_driver_mode (*get_driver_mode)(void);
	int (*collect_driver_dump)(struct pci_dev *pdev,
				   struct cnss_ssr_driver_dump_entry *input_array,
				   size_t *num_entries_loaded);
	int (*set_therm_cdev_state)(struct pci_dev *pci_dev,
				    unsigned long thermal_state,
				    int tcdev_id);
};

struct cnss_ce_tgt_pipe_cfg {
	u32 pipe_num;
	u32 pipe_dir;
	u32 nentries;
	u32 nbytes_max;
	u32 flags;
	u32 reserved;
};

struct cnss_ce_svc_pipe_cfg {
	u32 service_id;
	u32 pipe_dir;
	u32 pipe_num;
};

struct cnss_shadow_reg_cfg {
	u16 ce_id;
	u16 reg_offset;
};

struct cnss_shadow_reg_v2_cfg {
	u32 addr;
};

struct cnss_rri_over_ddr_cfg {
	u32 base_addr_low;
	u32 base_addr_high;
};

struct cnss_shadow_reg_v3_cfg {
	u32 addr;
};

struct cnss_wlan_enable_cfg {
	u32 num_ce_tgt_cfg;
	struct cnss_ce_tgt_pipe_cfg *ce_tgt_cfg;
	u32 num_ce_svc_pipe_cfg;
	struct cnss_ce_svc_pipe_cfg *ce_svc_cfg;
	u32 num_shadow_reg_cfg;
	struct cnss_shadow_reg_cfg *shadow_reg_cfg;
	u32 num_shadow_reg_v2_cfg;
	struct cnss_shadow_reg_v2_cfg *shadow_reg_v2_cfg;
	bool rri_over_ddr_cfg_valid;
	struct cnss_rri_over_ddr_cfg rri_over_ddr_cfg;
	u32 num_shadow_reg_v3_cfg;
	struct cnss_shadow_reg_v3_cfg *shadow_reg_v3_cfg;
	bool send_msi_ce;
};

enum cnss_driver_mode {
	CNSS_MISSION,
	CNSS_FTM,
	CNSS_EPPING,
	CNSS_WALTEST,
	CNSS_OFF,
	CNSS_CCPM,
	CNSS_QVIT,
	CNSS_CALIBRATION,
	CNSS_DRIVER_MODE_MAX,
};

enum cnss_recovery_reason {
	CNSS_REASON_DEFAULT,
	CNSS_REASON_LINK_DOWN,
	CNSS_REASON_RDDM,
	CNSS_REASON_TIMEOUT,
};

enum cnss_fw_caps {
	CNSS_FW_CAP_DIRECT_LINK_SUPPORT,
	CNSS_FW_CAP_AUX_UC_SUPPORT,
};

enum cnss_remote_mem_type {
	CNSS_REMOTE_MEM_TYPE_FW,
	CNSS_REMOTE_MEM_TYPE_QDSS,
	CNSS_REMOTE_MEM_TYPE_MAX,
};

struct cnss_mem_segment {
	size_t size;
	void *va;
	phys_addr_t pa;
};

extern int cnss_wlan_register_driver(struct cnss_wlan_driver *driver);
extern void cnss_wlan_unregister_driver(struct cnss_wlan_driver *driver);
extern void cnss_device_crashed(struct device *dev);
extern int cnss_pci_prevent_l1(struct device *dev);
extern void cnss_pci_allow_l1(struct device *dev);
extern int cnss_pci_link_down(struct device *dev);
extern int cnss_pci_is_device_down(struct device *dev);
extern void cnss_schedule_recovery(struct device *dev,
				   enum cnss_recovery_reason reason);
extern int cnss_self_recovery(struct device *dev,
			      enum cnss_recovery_reason reason);
extern int cnss_force_fw_assert(struct device *dev);
extern int cnss_force_collect_rddm(struct device *dev);
extern int cnss_qmi_send_get(struct device *dev);
extern int cnss_qmi_send_put(struct device *dev);
extern int cnss_qmi_send(struct device *dev, int type, void *cmd,
			 int cmd_len, void *cb_ctx,
			 int (*cb)(void *ctx, void *event, int event_len));
extern void *cnss_get_virt_ramdump_mem(struct device *dev, unsigned long *size);
extern int cnss_get_fw_files_for_target(struct device *dev,
					struct cnss_fw_files *pfw_files,
					u32 target_type, u32 target_version);
extern int cnss_get_platform_cap(struct device *dev,
				 struct cnss_platform_cap *cap);
extern struct iommu_domain *cnss_smmu_get_domain(struct device *dev);
extern int cnss_smmu_map(struct device *dev,
			 phys_addr_t paddr, uint32_t *iova_addr, size_t size);
extern int cnss_smmu_unmap(struct device *dev, uint32_t iova_addr, size_t size);
extern int cnss_get_soc_info(struct device *dev, struct cnss_soc_info *info);
extern int cnss_request_bus_bandwidth(struct device *dev, int bandwidth);
extern int cnss_power_up(struct device *dev);
extern int cnss_power_down(struct device *dev);
extern int cnss_idle_restart(struct device *dev);
extern int cnss_idle_shutdown(struct device *dev);
extern void cnss_request_pm_qos(struct device *dev, u32 qos_val);
extern void cnss_remove_pm_qos(struct device *dev);
extern void cnss_lock_pm_sem(struct device *dev);
extern void cnss_release_pm_sem(struct device *dev);
extern void cnss_pci_lock_reg_window(struct device *dev, unsigned long *flags);
extern void cnss_pci_unlock_reg_window(struct device *dev,
				       unsigned long *flags);
extern int cnss_wlan_pm_control(struct device *dev, bool vote);
extern int cnss_auto_suspend(struct device *dev);
extern int cnss_auto_resume(struct device *dev);
extern int cnss_pci_is_drv_connected(struct device *dev);
extern int cnss_pci_force_wake_request_sync(struct device *dev, int timeout);
extern int cnss_pci_force_wake_request(struct device *dev);
extern int cnss_pci_is_device_awake(struct device *dev);
extern int cnss_pci_force_wake_release(struct device *dev);
extern int cnss_get_user_msi_assignment(struct device *dev, char *user_name,
					int *num_vectors,
					uint32_t *user_base_data,
					uint32_t *base_vector);
extern int cnss_get_msi_irq(struct device *dev, unsigned int vector);
extern bool cnss_is_one_msi(struct device *dev);
extern void cnss_get_msi_address(struct device *dev, uint32_t *msi_addr_low,
				 uint32_t *msi_addr_high);
extern int cnss_wlan_hw_enable(void);
extern int cnss_wlan_enable(struct device *dev,
			    struct cnss_wlan_enable_cfg *config,
			    enum cnss_driver_mode mode,
			    const char *host_version);
extern int cnss_wlan_disable(struct device *dev, enum cnss_driver_mode mode);
extern unsigned int cnss_get_boot_timeout(struct device *dev);
extern int cnss_athdiag_read(struct device *dev, uint32_t offset,
			     uint32_t mem_type, uint32_t data_len,
			     uint8_t *output);
extern int cnss_athdiag_write(struct device *dev, uint32_t offset,
			      uint32_t mem_type, uint32_t data_len,
			      uint8_t *input);
extern int cnss_set_fw_log_mode(struct device *dev, uint8_t fw_log_mode);
extern int cnss_set_pcie_gen_speed(struct device *dev, u8 pcie_gen_speed);
extern int cnss_get_mem_seg_count(enum cnss_remote_mem_type type, u32 *seg);
extern int cnss_get_mem_segment_info(enum cnss_remote_mem_type type,
				     struct cnss_mem_segment segment[],
				     u32 segment_count);
extern int cnss_audio_smmu_map(struct device *dev, phys_addr_t paddr,
			       dma_addr_t iova, size_t size);
extern void cnss_audio_smmu_unmap(struct device *dev, dma_addr_t iova,
				 size_t size);
extern int cnss_get_fw_lpass_shared_mem(struct device *dev, dma_addr_t *iova,
					size_t *size);
extern int cnss_get_pci_slot(struct device *dev);
extern int cnss_pci_get_reg_dump(struct device *dev, uint8_t *buffer,
				 uint32_t len);
extern struct kobject *cnss_get_wifi_kobj(struct device *dev);
extern int cnss_send_buffer_to_afcmem(struct device *dev, const uint8_t *afcdb,
				      uint32_t len, uint8_t slotid);
extern int cnss_reset_afcmem(struct device *dev, uint8_t slotid);
extern bool cnss_get_fw_cap(struct device *dev, enum cnss_fw_caps fw_cap);
extern int cnss_set_wfc_mode(struct device *dev, struct cnss_wfc_cfg cfg);
extern int cnss_thermal_cdev_register(struct device *dev,
				      unsigned long max_state,
				      int tcdev_id);
extern void cnss_thermal_cdev_unregister(struct device *dev, int tcdev_id);
extern int cnss_get_curr_therm_cdev_state(struct device *dev,
					  unsigned long *thermal_state,
					  int tcdev_id);
extern int cnss_update_time_sync_period(struct device *dev,
					 uint32_t time_sync_period);
extern int cnss_reset_time_sync_period(struct device *dev);
#endif /* _NET_CNSS2_H */
