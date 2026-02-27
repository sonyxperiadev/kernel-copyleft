// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/devcoredump.h>
#include <linux/elf.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/reboot.h>
#include <linux/rwsem.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/thermal.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0))
#include <linux/panic_notifier.h>
#endif
#if IS_ENABLED(CONFIG_QCOM_MINIDUMP)
#include <soc/qcom/minidump.h>
#endif

#include "cnss_plat_ipc_qmi.h"
#include "cnss_utils.h"
#include "main.h"
#include "bus.h"
#include "debug.h"
#include "genl.h"
#include "reg.h"

#ifdef CONFIG_CNSS_HW_SECURE_DISABLE
#ifdef CONFIG_CNSS_HW_SECURE_SMEM
#include <linux/soc/qcom/smem.h>
#define PERISEC_SMEM_ID 651
#define HW_WIFI_UID 0x508
#else
#include "smcinvoke.h"
#include "smcinvoke_object.h"
#include "IClientEnv.h"
#define HW_STATE_UID 0x108
#define HW_OP_GET_STATE 1
#define HW_WIFI_UID 0x508
#define FEATURE_NOT_SUPPORTED 12
#define PERIPHERAL_NOT_FOUND 10
#endif
#endif

#define CNSS_DUMP_FORMAT_VER		0x11
#define CNSS_DUMP_FORMAT_VER_V2		0x22
#define CNSS_DUMP_MAGIC_VER_V2		0x42445953
#define CNSS_DUMP_NAME			"CNSS_WLAN"
#define CNSS_DUMP_DESC_SIZE		0x1000
#define CNSS_DUMP_SEG_VER		0x1
#define FILE_SYSTEM_READY		1
#define FW_READY_TIMEOUT		20000
#define FW_ASSERT_TIMEOUT		5000
#define CNSS_EVENT_PENDING		2989
#define POWER_RESET_MIN_DELAY_MS	100

#define CNSS_QUIRKS_DEFAULT		0
#ifdef CONFIG_CNSS_EMULATION
#define CNSS_MHI_TIMEOUT_DEFAULT	90000
#define CNSS_MHI_M2_TIMEOUT_DEFAULT	2000
#define CNSS_QMI_TIMEOUT_DEFAULT	90000
#else
#define CNSS_MHI_TIMEOUT_DEFAULT	0
#define CNSS_MHI_M2_TIMEOUT_DEFAULT	25
#define CNSS_QMI_TIMEOUT_DEFAULT	10000
#endif
#define CNSS_BDF_TYPE_DEFAULT		CNSS_BDF_ELF
#define CNSS_TIME_SYNC_PERIOD_DEFAULT	900000
#define CNSS_MIN_TIME_SYNC_PERIOD	2000
#define CNSS_DMS_QMI_CONNECTION_WAIT_MS 50
#define CNSS_DMS_QMI_CONNECTION_WAIT_RETRY 200
#define CNSS_DAEMON_CONNECT_TIMEOUT_MS  30000
#define CNSS_CAL_DB_FILE_NAME "wlfw_cal_db.bin"
#define CNSS_CAL_START_PROBE_WAIT_RETRY_MAX 100
#define CNSS_CAL_START_PROBE_WAIT_MS	500
#define CNSS_TIME_SYNC_PERIOD_INVALID	0xFFFFFFFF

enum cnss_cal_db_op {
	CNSS_CAL_DB_UPLOAD,
	CNSS_CAL_DB_DOWNLOAD,
	CNSS_CAL_DB_INVALID_OP,
};

enum cnss_recovery_type {
	CNSS_WLAN_RECOVERY = 0x1,
	CNSS_PCSS_RECOVERY = 0x2,
};

#ifdef CONFIG_CNSS_SUPPORT_DUAL_DEV
#define CNSS_MAX_DEV_NUM		2
static struct cnss_plat_data *plat_env[CNSS_MAX_DEV_NUM];
static int plat_env_count;
#else
static struct cnss_plat_data *plat_env;
#endif

static bool cnss_allow_driver_loading;

static struct cnss_fw_files FW_FILES_QCA6174_FW_3_0 = {
	"qwlan30.bin", "bdwlan30.bin", "otp30.bin", "utf30.bin",
	"utfbd30.bin", "epping30.bin", "evicted30.bin"
};

static struct cnss_fw_files FW_FILES_DEFAULT = {
	"qwlan.bin", "bdwlan.bin", "otp.bin", "utf.bin",
	"utfbd.bin", "epping.bin", "evicted.bin"
};

struct cnss_driver_event {
	struct list_head list;
	enum cnss_driver_event_type type;
	bool sync;
	struct completion complete;
	int ret;
	void *data;
};

bool cnss_check_driver_loading_allowed(void)
{
	return cnss_allow_driver_loading;
}

#ifdef CONFIG_CNSS_SUPPORT_DUAL_DEV
static void cnss_set_plat_priv(struct platform_device *plat_dev,
			       struct cnss_plat_data *plat_priv)
{
	cnss_pr_dbg("Set plat_priv at %d", plat_env_count);
	if (plat_priv) {
		plat_priv->plat_idx = plat_env_count;
		plat_env[plat_priv->plat_idx] = plat_priv;
		plat_env_count++;
	}
}

struct cnss_plat_data *cnss_get_plat_priv(struct platform_device
						 *plat_dev)
{
	int i;

	if (!plat_dev)
		return NULL;

	for (i = 0; i < plat_env_count; i++) {
		if (plat_env[i]->plat_dev == plat_dev)
			return plat_env[i];
	}
	return NULL;
}

struct cnss_plat_data *cnss_get_first_plat_priv(struct platform_device
						 *plat_dev)
{
	int i;

	if (!plat_dev) {
		for (i = 0; i < plat_env_count; i++) {
			if (plat_env[i])
				return plat_env[i];
		}
	}
	return NULL;
}

static void cnss_clear_plat_priv(struct cnss_plat_data *plat_priv)
{
	cnss_pr_dbg("Clear plat_priv at %d", plat_priv->plat_idx);
	plat_env[plat_priv->plat_idx] = NULL;
	plat_env_count--;
}

static int cnss_set_device_name(struct cnss_plat_data *plat_priv)
{
	snprintf(plat_priv->device_name, sizeof(plat_priv->device_name),
		 "wlan_%d", plat_priv->plat_idx);

	return 0;
}

static int cnss_plat_env_available(void)
{
	int ret = 0;

	if (plat_env_count >= CNSS_MAX_DEV_NUM) {
		cnss_pr_err("ERROR: No space to store plat_priv\n");
		ret = -ENOMEM;
	}
	return ret;
}

int cnss_get_plat_env_count(void)
{
	return plat_env_count;
}

struct cnss_plat_data *cnss_get_plat_env(int index)
{
	return plat_env[index];
}

struct cnss_plat_data *cnss_get_plat_priv_by_rc_num(int rc_num)
{
	int i;

	for (i = 0; i < plat_env_count; i++) {
		if (plat_env[i]->rc_num == rc_num)
			return plat_env[i];
	}
	return NULL;
}

static inline int
cnss_get_qrtr_node_id(struct cnss_plat_data *plat_priv)
{
	return of_property_read_u32(plat_priv->dev_node,
		"qcom,qrtr_node_id", &plat_priv->qrtr_node_id);
}

void cnss_get_qrtr_info(struct cnss_plat_data *plat_priv)
{
	int ret = 0;

	ret = cnss_get_qrtr_node_id(plat_priv);
	if (ret) {
		cnss_pr_warn("Failed to find qrtr_node_id err=%d\n", ret);
		plat_priv->qrtr_node_id = 0;
		plat_priv->wlfw_service_instance_id = 0;
	} else {
		plat_priv->wlfw_service_instance_id = plat_priv->qrtr_node_id +
						      QRTR_NODE_FW_ID_BASE;
		cnss_pr_dbg("service_instance_id=0x%x\n",
			    plat_priv->wlfw_service_instance_id);
	}
}

static inline int
cnss_get_pld_bus_ops_name(struct cnss_plat_data *plat_priv)
{
	return of_property_read_string(plat_priv->plat_dev->dev.of_node,
				       "qcom,pld_bus_ops_name",
				       &plat_priv->pld_bus_ops_name);
}

#else
static void cnss_set_plat_priv(struct platform_device *plat_dev,
			       struct cnss_plat_data *plat_priv)
{
	plat_env = plat_priv;
}

struct cnss_plat_data *cnss_get_plat_priv(struct platform_device *plat_dev)
{
	return plat_env;
}

static void cnss_clear_plat_priv(struct cnss_plat_data *plat_priv)
{
	plat_env = NULL;
}

static int cnss_set_device_name(struct cnss_plat_data *plat_priv)
{
	snprintf(plat_priv->device_name, sizeof(plat_priv->device_name),
		 "wlan");
	return 0;
}

static int cnss_plat_env_available(void)
{
	return 0;
}

struct cnss_plat_data *cnss_get_plat_priv_by_rc_num(int rc_num)
{
	return cnss_bus_dev_to_plat_priv(NULL);
}

void cnss_get_qrtr_info(struct cnss_plat_data *plat_priv)
{
}

static int
cnss_get_pld_bus_ops_name(struct cnss_plat_data *plat_priv)
{
	return 0;
}
#endif

void cnss_get_sleep_clk_supported(struct cnss_plat_data *plat_priv)
{
	plat_priv->sleep_clk = of_property_read_bool(plat_priv->dev_node,
						     "qcom,sleep-clk-support");
	cnss_pr_dbg("qcom,sleep-clk-support is %d\n",
		    plat_priv->sleep_clk);
}

void cnss_get_bwscal_info(struct cnss_plat_data *plat_priv)
{
	plat_priv->no_bwscale = of_property_read_bool(plat_priv->dev_node,
						      "qcom,no-bwscale");
}

static inline int
cnss_get_rc_num(struct cnss_plat_data *plat_priv)
{
	return of_property_read_u32(plat_priv->plat_dev->dev.of_node,
		"qcom,wlan-rc-num", &plat_priv->rc_num);
}

bool cnss_is_dual_wlan_enabled(void)
{
	return IS_ENABLED(CONFIG_CNSS_SUPPORT_DUAL_DEV);
}

/**
 * cnss_get_mem_seg_count - Get segment count of memory
 * @type: memory type
 * @seg: segment count
 *
 * Return: 0 on success, negative value on failure
 */
int cnss_get_mem_seg_count(enum cnss_remote_mem_type type, u32 *seg)
{
	struct cnss_plat_data *plat_priv;

	plat_priv = cnss_get_plat_priv(NULL);
	if (!plat_priv)
		return -ENODEV;

	switch (type) {
	case CNSS_REMOTE_MEM_TYPE_FW:
		*seg = plat_priv->fw_mem_seg_len;
		break;
	case CNSS_REMOTE_MEM_TYPE_QDSS:
		*seg = plat_priv->qdss_mem_seg_len;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(cnss_get_mem_seg_count);

/**
 * cnss_get_wifi_kobject -return wifi kobject
 * Return: Null, to maintain driver comnpatibilty
 */
struct kobject *cnss_get_wifi_kobj(struct device *dev)
{
	struct cnss_plat_data *plat_priv;

	plat_priv = cnss_get_plat_priv(NULL);
	if (!plat_priv)
		return NULL;

	return plat_priv->wifi_kobj;
}
EXPORT_SYMBOL(cnss_get_wifi_kobj);

/**
 * cnss_get_mem_segment_info - Get memory info of different type
 * @type: memory type
 * @segment: array to save the segment info
 * @seg: segment count
 *
 * Return: 0 on success, negative value on failure
 */
int cnss_get_mem_segment_info(enum cnss_remote_mem_type type,
			      struct cnss_mem_segment segment[],
			      u32 segment_count)
{
	struct cnss_plat_data *plat_priv;
	u32 i;

	plat_priv = cnss_get_plat_priv(NULL);
	if (!plat_priv)
		return -ENODEV;

	switch (type) {
	case CNSS_REMOTE_MEM_TYPE_FW:
		if (segment_count > plat_priv->fw_mem_seg_len)
			segment_count = plat_priv->fw_mem_seg_len;
		for (i = 0; i < segment_count; i++) {
			segment[i].size = plat_priv->fw_mem[i].size;
			segment[i].va = plat_priv->fw_mem[i].va;
			segment[i].pa = plat_priv->fw_mem[i].pa;
		}
		break;
	case CNSS_REMOTE_MEM_TYPE_QDSS:
		if (segment_count > plat_priv->qdss_mem_seg_len)
			segment_count = plat_priv->qdss_mem_seg_len;
		for (i = 0; i < segment_count; i++) {
			segment[i].size = plat_priv->qdss_mem[i].size;
			segment[i].va = plat_priv->qdss_mem[i].va;
			segment[i].pa = plat_priv->qdss_mem[i].pa;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(cnss_get_mem_segment_info);

static int cnss_get_audio_iommu_domain(struct cnss_plat_data *plat_priv)
{
	struct device_node *audio_ion_node;
	struct platform_device *audio_ion_pdev;

	audio_ion_node = of_find_compatible_node(NULL, NULL,
						 "qcom,msm-audio-ion");
	if (!audio_ion_node) {
		cnss_pr_err("Unable to get Audio ion node");
		return -EINVAL;
	}

	audio_ion_pdev = of_find_device_by_node(audio_ion_node);
	of_node_put(audio_ion_node);
	if (!audio_ion_pdev) {
		cnss_pr_err("Unable to get Audio ion platform device");
		return -EINVAL;
	}

	plat_priv->audio_iommu_domain =
				iommu_get_domain_for_dev(&audio_ion_pdev->dev);
	put_device(&audio_ion_pdev->dev);
	if (!plat_priv->audio_iommu_domain) {
		cnss_pr_err("Unable to get Audio ion iommu domain");
		return -EINVAL;
	}

	return 0;
}

int cnss_set_feature_list(struct cnss_plat_data *plat_priv,
			  enum cnss_feature_v01 feature)
{
	if (unlikely(!plat_priv || feature >= CNSS_MAX_FEATURE_V01))
		return -EINVAL;

	plat_priv->feature_list |= 1 << feature;
	return 0;
}

int cnss_clear_feature_list(struct cnss_plat_data *plat_priv,
			    enum cnss_feature_v01 feature)
{
	if (unlikely(!plat_priv || feature >= CNSS_MAX_FEATURE_V01))
		return -EINVAL;

	plat_priv->feature_list &= ~(1 << feature);
	return 0;
}

int cnss_get_feature_list(struct cnss_plat_data *plat_priv,
			  u64 *feature_list)
{
	if (unlikely(!plat_priv))
		return -EINVAL;

	*feature_list = plat_priv->feature_list;
	return 0;
}

size_t cnss_get_platform_name(struct cnss_plat_data *plat_priv,
			      char *buf, const size_t buf_len)
{
	if (unlikely(!plat_priv || !buf || !buf_len))
		return 0;

	if (of_property_read_bool(plat_priv->plat_dev->dev.of_node,
				  "platform-name-required")) {
		struct device_node *root;

		root = of_find_node_by_path("/");
		if (root) {
			const char *model;
			size_t model_len;

			model = of_get_property(root, "model", NULL);
			if (model) {
				model_len = strlcpy(buf, model, buf_len);
				cnss_pr_dbg("Platform name: %s (%zu)\n",
					    buf, model_len);

				return model_len;
			}
		}
	}

	return 0;
}

void cnss_pm_stay_awake(struct cnss_plat_data *plat_priv)
{
	if (atomic_inc_return(&plat_priv->pm_count) != 1)
		return;

	cnss_pr_dbg("PM stay awake, state: 0x%lx, count: %d\n",
		    plat_priv->driver_state,
		    atomic_read(&plat_priv->pm_count));
	pm_stay_awake(&plat_priv->plat_dev->dev);
}

void cnss_pm_relax(struct cnss_plat_data *plat_priv)
{
	int r = atomic_dec_return(&plat_priv->pm_count);

	WARN_ON(r < 0);

	if (r != 0)
		return;

	cnss_pr_dbg("PM relax, state: 0x%lx, count: %d\n",
		    plat_priv->driver_state,
		    atomic_read(&plat_priv->pm_count));
	pm_relax(&plat_priv->plat_dev->dev);
}

int cnss_get_fw_files_for_target(struct device *dev,
				 struct cnss_fw_files *pfw_files,
				 u32 target_type, u32 target_version)
{
	if (!pfw_files)
		return -ENODEV;

	switch (target_version) {
	case QCA6174_REV3_VERSION:
	case QCA6174_REV3_2_VERSION:
		memcpy(pfw_files, &FW_FILES_QCA6174_FW_3_0, sizeof(*pfw_files));
		break;
	default:
		memcpy(pfw_files, &FW_FILES_DEFAULT, sizeof(*pfw_files));
		cnss_pr_err("Unknown target version, type: 0x%X, version: 0x%X",
			    target_type, target_version);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(cnss_get_fw_files_for_target);

int cnss_get_platform_cap(struct device *dev, struct cnss_platform_cap *cap)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv)
		return -ENODEV;

	if (!cap)
		return -EINVAL;

	*cap = plat_priv->cap;
	cnss_pr_dbg("Platform cap_flag is 0x%x\n", cap->cap_flag);

	return 0;
}
EXPORT_SYMBOL(cnss_get_platform_cap);

/**
 * cnss_get_fw_cap - Check whether FW supports specific capability or not
 * @dev: Device
 * @fw_cap: FW Capability which needs to be checked
 *
 * Return: TRUE if supported, FALSE on failure or if not supported
 */
bool cnss_get_fw_cap(struct device *dev, enum cnss_fw_caps fw_cap)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	bool is_supported = false;

	if (!plat_priv)
		return is_supported;

	if (!plat_priv->fw_caps)
		return is_supported;

	switch (fw_cap) {
	case CNSS_FW_CAP_DIRECT_LINK_SUPPORT:
		is_supported = !!(plat_priv->fw_caps &
				  QMI_WLFW_DIRECT_LINK_SUPPORT_V01);
		if (is_supported && cnss_get_audio_iommu_domain(plat_priv))
			is_supported = false;
		break;
	default:
		cnss_pr_err("Invalid FW Capability: 0x%x\n", fw_cap);
	}

	cnss_pr_dbg("FW Capability 0x%x is %s\n", fw_cap,
		    is_supported ? "supported" : "not supported");
	return is_supported;
}
EXPORT_SYMBOL(cnss_get_fw_cap);

void cnss_request_pm_qos(struct device *dev, u32 qos_val)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv)
		return;

	cpu_latency_qos_add_request(&plat_priv->qos_request, qos_val);
}
EXPORT_SYMBOL(cnss_request_pm_qos);

void cnss_remove_pm_qos(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv)
		return;

	cpu_latency_qos_remove_request(&plat_priv->qos_request);
}
EXPORT_SYMBOL(cnss_remove_pm_qos);

int cnss_wlan_enable(struct device *dev,
		     struct cnss_wlan_enable_cfg *config,
		     enum cnss_driver_mode mode,
		     const char *host_version)
{
	int ret = 0;
	struct cnss_plat_data *plat_priv;

	if (!dev) {
		cnss_pr_err("Invalid dev pointer\n");
		return -EINVAL;
	}

	plat_priv = cnss_bus_dev_to_plat_priv(dev);
	if (!plat_priv)
		return -ENODEV;

	if (plat_priv->device_id == QCA6174_DEVICE_ID)
		return 0;

	if (test_bit(QMI_BYPASS, &plat_priv->ctrl_params.quirks))
		return 0;

	if (!config || !host_version) {
		cnss_pr_err("Invalid config or host_version pointer\n");
		return -EINVAL;
	}

	cnss_pr_dbg("Mode: %d, config: %pK, host_version: %s\n",
		    mode, config, host_version);

	if (mode == CNSS_WALTEST || mode == CNSS_CCPM)
		goto skip_cfg;

	if (plat_priv->device_id == QCN7605_DEVICE_ID)
		config->send_msi_ce = true;

	ret = cnss_wlfw_wlan_cfg_send_sync(plat_priv, config, host_version);
	if (ret)
		goto out;

skip_cfg:
	ret = cnss_wlfw_wlan_mode_send_sync(plat_priv, mode);
out:
	return ret;
}
EXPORT_SYMBOL(cnss_wlan_enable);

int cnss_wlan_disable(struct device *dev, enum cnss_driver_mode mode)
{
	int ret = 0;
	struct cnss_plat_data *plat_priv;

	if (!dev) {
		cnss_pr_err("Invalid dev pointer\n");
		return -EINVAL;
	}

	plat_priv = cnss_bus_dev_to_plat_priv(dev);
	if (!plat_priv)
		return -ENODEV;

	if (plat_priv->device_id == QCA6174_DEVICE_ID)
		return 0;

	if (test_bit(QMI_BYPASS, &plat_priv->ctrl_params.quirks))
		return 0;

	ret = cnss_wlfw_wlan_mode_send_sync(plat_priv, CNSS_OFF);
	cnss_bus_free_qdss_mem(plat_priv);

	return ret;
}
EXPORT_SYMBOL(cnss_wlan_disable);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 2, 0))
int cnss_iommu_map(struct iommu_domain *domain,
		   unsigned long iova, phys_addr_t paddr, size_t size, int prot)
{
	return iommu_map(domain, iova, paddr, size, prot);
}
#else
int cnss_iommu_map(struct iommu_domain *domain,
		   unsigned long iova, phys_addr_t paddr, size_t size, int prot)
{
	return iommu_map(domain, iova, paddr, size, prot, GFP_KERNEL);
}
#endif

int cnss_audio_smmu_map(struct device *dev, phys_addr_t paddr,
			dma_addr_t iova, size_t size)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	uint32_t page_offset;

	if (!plat_priv)
		return -ENODEV;

	if (!plat_priv->audio_iommu_domain)
		return -EINVAL;

	page_offset = iova & (PAGE_SIZE - 1);
	if (page_offset + size > PAGE_SIZE)
		size += PAGE_SIZE;

	iova -= page_offset;
	paddr -= page_offset;

	return cnss_iommu_map(plat_priv->audio_iommu_domain, iova, paddr,
			      roundup(size, PAGE_SIZE), IOMMU_READ |
			      IOMMU_WRITE | IOMMU_CACHE);
}
EXPORT_SYMBOL(cnss_audio_smmu_map);

void cnss_audio_smmu_unmap(struct device *dev, dma_addr_t iova, size_t size)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	uint32_t page_offset;

	if (!plat_priv)
		return;

	if (!plat_priv->audio_iommu_domain)
		return;

	page_offset = iova & (PAGE_SIZE - 1);
	if (page_offset + size > PAGE_SIZE)
		size += PAGE_SIZE;

	iova -= page_offset;

	iommu_unmap(plat_priv->audio_iommu_domain, iova,
		    roundup(size, PAGE_SIZE));
}
EXPORT_SYMBOL(cnss_audio_smmu_unmap);

int cnss_get_fw_lpass_shared_mem(struct device *dev, dma_addr_t *iova,
				 size_t *size)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	uint8_t i;

	if (!plat_priv)
		return -EINVAL;

	for (i = 0; i < plat_priv->fw_mem_seg_len; i++) {
		if (plat_priv->fw_mem[i].type ==
		    QMI_WLFW_MEM_LPASS_SHARED_V01) {
			*iova = plat_priv->fw_mem[i].pa;
			*size = plat_priv->fw_mem[i].size;
			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL(cnss_get_fw_lpass_shared_mem);

int cnss_athdiag_read(struct device *dev, u32 offset, u32 mem_type,
		      u32 data_len, u8 *output)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	int ret = 0;

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL!\n");
		return -EINVAL;
	}

	if (plat_priv->device_id == QCA6174_DEVICE_ID)
		return 0;

	if (!test_bit(CNSS_FW_READY, &plat_priv->driver_state)) {
		cnss_pr_err("Invalid state for athdiag read: 0x%lx\n",
			    plat_priv->driver_state);
		ret = -EINVAL;
		goto out;
	}

	ret = cnss_wlfw_athdiag_read_send_sync(plat_priv, offset, mem_type,
					       data_len, output);

out:
	return ret;
}
EXPORT_SYMBOL(cnss_athdiag_read);

int cnss_athdiag_write(struct device *dev, u32 offset, u32 mem_type,
		       u32 data_len, u8 *input)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	int ret = 0;

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL!\n");
		return -EINVAL;
	}

	if (plat_priv->device_id == QCA6174_DEVICE_ID)
		return 0;

	if (!test_bit(CNSS_FW_READY, &plat_priv->driver_state)) {
		cnss_pr_err("Invalid state for athdiag write: 0x%lx\n",
			    plat_priv->driver_state);
		ret = -EINVAL;
		goto out;
	}

	ret = cnss_wlfw_athdiag_write_send_sync(plat_priv, offset, mem_type,
						data_len, input);

out:
	return ret;
}
EXPORT_SYMBOL(cnss_athdiag_write);

int cnss_set_fw_log_mode(struct device *dev, u8 fw_log_mode)
{
	struct cnss_plat_data *plat_priv;

	if (!dev) {
		cnss_pr_err("Invalid dev pointer\n");
		return -EINVAL;
	}

	plat_priv = cnss_bus_dev_to_plat_priv(dev);
	if (!plat_priv)
		return -ENODEV;

	if (plat_priv->device_id == QCA6174_DEVICE_ID)
		return 0;

	return cnss_wlfw_ini_send_sync(plat_priv, fw_log_mode);
}
EXPORT_SYMBOL(cnss_set_fw_log_mode);

int cnss_set_pcie_gen_speed(struct device *dev, u8 pcie_gen_speed)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv)
		return -EINVAL;

	if (!plat_priv->fw_pcie_gen_switch) {
		cnss_pr_err("Firmware does not support PCIe gen switch\n");
		return -EOPNOTSUPP;
	}

	if (pcie_gen_speed < QMI_PCIE_GEN_SPEED_1_V01 ||
	    pcie_gen_speed > QMI_PCIE_GEN_SPEED_3_V01)
		return -EINVAL;

	cnss_pr_dbg("WLAN provided PCIE gen speed: %d\n", pcie_gen_speed);
	plat_priv->pcie_gen_speed = pcie_gen_speed;
	return 0;
}
EXPORT_SYMBOL(cnss_set_pcie_gen_speed);

static bool cnss_is_aux_support_enabled(struct cnss_plat_data *plat_priv)
{
	switch (plat_priv->device_id) {
	case PEACH_DEVICE_ID:
		if (!plat_priv->fw_aux_uc_support) {
			cnss_pr_dbg("FW does not support aux uc capability\n");
			return false;
		}
		break;
	default:
		cnss_pr_dbg("Host does not support aux uc capability\n");
		return false;
	}

	return true;
}

static int cnss_fw_mem_ready_hdlr(struct cnss_plat_data *plat_priv)
{
	int ret = 0;

	if (!plat_priv)
		return -ENODEV;

	set_bit(CNSS_FW_MEM_READY, &plat_priv->driver_state);

	ret = cnss_wlfw_tgt_cap_send_sync(plat_priv);
	if (ret)
		goto out;

	cnss_bus_load_tme_patch(plat_priv);

	cnss_wlfw_tme_patch_dnld_send_sync(plat_priv,
					   WLFW_TME_LITE_PATCH_FILE_V01);

	if (plat_priv->hds_enabled)
		cnss_wlfw_bdf_dnld_send_sync(plat_priv, CNSS_BDF_HDS);

	cnss_wlfw_bdf_dnld_send_sync(plat_priv, CNSS_BDF_REGDB);

	if (plat_priv->device_id == QCN7605_DEVICE_ID)
		plat_priv->ctrl_params.bdf_type = CNSS_BDF_BIN;

	ret = cnss_wlfw_bdf_dnld_send_sync(plat_priv,
					   plat_priv->ctrl_params.bdf_type);
	if (ret)
		goto out;

	if (plat_priv->device_id == QCN7605_DEVICE_ID)
		return 0;

	ret = cnss_bus_load_m3(plat_priv);
	if (ret)
		goto out;

	ret = cnss_wlfw_m3_dnld_send_sync(plat_priv);
	if (ret)
		goto out;

	if (cnss_is_aux_support_enabled(plat_priv)) {
		ret = cnss_bus_load_aux(plat_priv);
		if (ret)
			goto out;

		ret = cnss_wlfw_aux_dnld_send_sync(plat_priv);
		if (ret)
			goto out;
	}

	cnss_wlfw_qdss_dnld_send_sync(plat_priv);

	return 0;
out:
	return ret;
}

static int cnss_request_antenna_sharing(struct cnss_plat_data *plat_priv)
{
	int ret = 0;

	if (!plat_priv->antenna) {
		ret = cnss_wlfw_antenna_switch_send_sync(plat_priv);
		if (ret)
			goto out;
	}

	if (test_bit(CNSS_COEX_CONNECTED, &plat_priv->driver_state)) {
		ret = coex_antenna_switch_to_wlan_send_sync_msg(plat_priv);
		if (ret)
			goto out;
	}

	ret = cnss_wlfw_antenna_grant_send_sync(plat_priv);
	if (ret)
		goto out;

	return 0;

out:
	return ret;
}

static void cnss_release_antenna_sharing(struct cnss_plat_data *plat_priv)
{
	if (test_bit(CNSS_COEX_CONNECTED, &plat_priv->driver_state))
		coex_antenna_switch_to_mdm_send_sync_msg(plat_priv);
}

static int cnss_setup_dms_mac(struct cnss_plat_data *plat_priv)
{
	u32 i;
	int ret = 0;
	struct cnss_plat_ipc_daemon_config *cfg;

	ret = cnss_qmi_get_dms_mac(plat_priv);
	if (ret == 0 && plat_priv->dms.mac_valid)
		goto qmi_send;

	/* DTSI property use-nv-mac is used to force DMS MAC address for WLAN.
	 * Thus assert on failure to get MAC from DMS even after retries
	 */
	if (plat_priv->use_nv_mac) {
		/* Check if Daemon says platform support DMS MAC provisioning */
		cfg = cnss_plat_ipc_qmi_daemon_config();
		if (cfg) {
			if (!cfg->dms_mac_addr_supported) {
				cnss_pr_err("DMS MAC address not supported\n");
				CNSS_ASSERT(0);
				return -EINVAL;
			}
		}
		for (i = 0; i < CNSS_DMS_QMI_CONNECTION_WAIT_RETRY; i++) {
			if (plat_priv->dms.mac_valid)
				break;

			ret = cnss_qmi_get_dms_mac(plat_priv);
			if (ret == 0)
				break;
			msleep(CNSS_DMS_QMI_CONNECTION_WAIT_MS);
		}
		if (!plat_priv->dms.mac_valid) {
			cnss_pr_err("Unable to get MAC from DMS after retries\n");
			CNSS_ASSERT(0);
			return -EINVAL;
		}
	}
qmi_send:
	if (plat_priv->dms.mac_valid)
		ret =
		cnss_wlfw_wlan_mac_req_send_sync(plat_priv, plat_priv->dms.mac,
						 ARRAY_SIZE(plat_priv->dms.mac));

	return ret;
}

static int cnss_cal_db_mem_update(struct cnss_plat_data *plat_priv,
				  enum cnss_cal_db_op op, u32 *size)
{
	int ret = 0;
	u32 timeout = cnss_get_timeout(plat_priv,
				       CNSS_TIMEOUT_DAEMON_CONNECTION);
	enum cnss_plat_ipc_qmi_client_id_v01 client_id =
					CNSS_PLAT_IPC_DAEMON_QMI_CLIENT_V01;

	if (op >= CNSS_CAL_DB_INVALID_OP)
		return -EINVAL;

	if (!plat_priv->cbc_file_download) {
		cnss_pr_info("CAL DB file not required as per BDF\n");
		return 0;
	}
	if (*size == 0) {
		cnss_pr_err("Invalid cal file size\n");
		return -EINVAL;
	}
	if (!test_bit(CNSS_DAEMON_CONNECTED, &plat_priv->driver_state)) {
		cnss_pr_info("Waiting for CNSS Daemon connection\n");
		ret = wait_for_completion_timeout(&plat_priv->daemon_connected,
						  msecs_to_jiffies(timeout));
		if (!ret) {
			cnss_pr_err("Daemon not yet connected\n");
			CNSS_ASSERT(0);
			return ret;
		}
	}
	if (!plat_priv->cal_mem->va) {
		cnss_pr_err("CAL DB Memory not setup for FW\n");
		return -EINVAL;
	}

	/* Copy CAL DB file contents to/from CAL_TYPE_DDR mem allocated to FW */
	if (op == CNSS_CAL_DB_DOWNLOAD) {
		cnss_pr_dbg("Initiating Calibration file download to mem\n");
		ret = cnss_plat_ipc_qmi_file_download(client_id,
						      CNSS_CAL_DB_FILE_NAME,
						      plat_priv->cal_mem->va,
						      size);
	} else {
		cnss_pr_dbg("Initiating Calibration mem upload to file\n");
		ret = cnss_plat_ipc_qmi_file_upload(client_id,
						    CNSS_CAL_DB_FILE_NAME,
						    plat_priv->cal_mem->va,
						    *size);
	}

	if (ret)
		cnss_pr_err("Cal DB file %s %s failure\n",
			    CNSS_CAL_DB_FILE_NAME,
			    op == CNSS_CAL_DB_DOWNLOAD ? "download" : "upload");
	else
		cnss_pr_dbg("Cal DB file %s %s size %d done\n",
			    CNSS_CAL_DB_FILE_NAME,
			    op == CNSS_CAL_DB_DOWNLOAD ? "download" : "upload",
			    *size);

	return ret;
}

static int cnss_cal_mem_upload_to_file(struct cnss_plat_data *plat_priv)
{
	if (plat_priv->cal_file_size > plat_priv->cal_mem->size) {
		cnss_pr_err("Cal file size is larger than Cal DB Mem size\n");
		return -EINVAL;
	}
	return cnss_cal_db_mem_update(plat_priv, CNSS_CAL_DB_UPLOAD,
				      &plat_priv->cal_file_size);
}

static int cnss_cal_file_download_to_mem(struct cnss_plat_data *plat_priv,
					 u32 *cal_file_size)
{
	/* To download pass the total size of cal DB mem allocated.
	 * After cal file is download to mem, its size is updated in
	 * return pointer
	 */
	*cal_file_size = plat_priv->cal_mem->size;
	return cnss_cal_db_mem_update(plat_priv, CNSS_CAL_DB_DOWNLOAD,
				      cal_file_size);
}

static int cnss_fw_ready_hdlr(struct cnss_plat_data *plat_priv)
{
	int ret = 0;
	u32 cal_file_size = 0;

	if (!plat_priv)
		return -ENODEV;

	if (test_bit(CNSS_IN_REBOOT, &plat_priv->driver_state)) {
		cnss_pr_err("Reboot is in progress, ignore FW ready\n");
		return -EINVAL;
	}

	cnss_pr_dbg("Processing FW Init Done..\n");
	del_timer(&plat_priv->fw_boot_timer);
	set_bit(CNSS_FW_READY, &plat_priv->driver_state);
	clear_bit(CNSS_DEV_ERR_NOTIFY, &plat_priv->driver_state);

	cnss_wlfw_send_pcie_gen_speed_sync(plat_priv);
	cnss_send_subsys_restart_level_msg(plat_priv);

	if (test_bit(CNSS_FW_BOOT_RECOVERY, &plat_priv->driver_state)) {
		clear_bit(CNSS_FW_BOOT_RECOVERY, &plat_priv->driver_state);
		clear_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state);
	}

	if (test_bit(ENABLE_WALTEST, &plat_priv->ctrl_params.quirks)) {
		ret = cnss_wlfw_wlan_mode_send_sync(plat_priv,
						    CNSS_WALTEST);
	} else if (test_bit(CNSS_IN_COLD_BOOT_CAL, &plat_priv->driver_state)) {
		cnss_request_antenna_sharing(plat_priv);
		cnss_cal_file_download_to_mem(plat_priv, &cal_file_size);
		cnss_wlfw_cal_report_req_send_sync(plat_priv, cal_file_size);
		plat_priv->cal_time = jiffies;
		ret = cnss_wlfw_wlan_mode_send_sync(plat_priv,
						    CNSS_CALIBRATION);
	} else {
		ret = cnss_setup_dms_mac(plat_priv);
		ret = cnss_bus_call_driver_probe(plat_priv);
	}

	if (ret && test_bit(CNSS_DEV_ERR_NOTIFY, &plat_priv->driver_state))
		goto out;
	else if (ret)
		goto shutdown;

	cnss_vreg_unvote_type(plat_priv, CNSS_VREG_PRIM);

	return 0;

shutdown:
	cnss_bus_dev_shutdown(plat_priv);

	clear_bit(CNSS_FW_READY, &plat_priv->driver_state);
	clear_bit(CNSS_FW_MEM_READY, &plat_priv->driver_state);

out:
	return ret;
}

static char *cnss_driver_event_to_str(enum cnss_driver_event_type type)
{
	switch (type) {
	case CNSS_DRIVER_EVENT_SERVER_ARRIVE:
		return "SERVER_ARRIVE";
	case CNSS_DRIVER_EVENT_SERVER_EXIT:
		return "SERVER_EXIT";
	case CNSS_DRIVER_EVENT_REQUEST_MEM:
		return "REQUEST_MEM";
	case CNSS_DRIVER_EVENT_FW_MEM_READY:
		return "FW_MEM_READY";
	case CNSS_DRIVER_EVENT_FW_READY:
		return "FW_READY";
	case CNSS_DRIVER_EVENT_COLD_BOOT_CAL_START:
		return "COLD_BOOT_CAL_START";
	case CNSS_DRIVER_EVENT_COLD_BOOT_CAL_DONE:
		return "COLD_BOOT_CAL_DONE";
	case CNSS_DRIVER_EVENT_REGISTER_DRIVER:
		return "REGISTER_DRIVER";
	case CNSS_DRIVER_EVENT_UNREGISTER_DRIVER:
		return "UNREGISTER_DRIVER";
	case CNSS_DRIVER_EVENT_RECOVERY:
		return "RECOVERY";
	case CNSS_DRIVER_EVENT_FORCE_FW_ASSERT:
		return "FORCE_FW_ASSERT";
	case CNSS_DRIVER_EVENT_POWER_UP:
		return "POWER_UP";
	case CNSS_DRIVER_EVENT_POWER_DOWN:
		return "POWER_DOWN";
	case CNSS_DRIVER_EVENT_IDLE_RESTART:
		return "IDLE_RESTART";
	case CNSS_DRIVER_EVENT_IDLE_SHUTDOWN:
		return "IDLE_SHUTDOWN";
	case CNSS_DRIVER_EVENT_IMS_WFC_CALL_IND:
		return "IMS_WFC_CALL_IND";
	case CNSS_DRIVER_EVENT_WLFW_TWT_CFG_IND:
		return "WLFW_TWC_CFG_IND";
	case CNSS_DRIVER_EVENT_QDSS_TRACE_REQ_MEM:
		return "QDSS_TRACE_REQ_MEM";
	case CNSS_DRIVER_EVENT_FW_MEM_FILE_SAVE:
		return "FW_MEM_FILE_SAVE";
	case CNSS_DRIVER_EVENT_QDSS_TRACE_FREE:
		return "QDSS_TRACE_FREE";
	case CNSS_DRIVER_EVENT_QDSS_TRACE_REQ_DATA:
		return "QDSS_TRACE_REQ_DATA";
	case CNSS_DRIVER_EVENT_MAX:
		return "EVENT_MAX";
	}

	return "UNKNOWN";
};

int cnss_driver_event_post(struct cnss_plat_data *plat_priv,
			   enum cnss_driver_event_type type,
			   u32 flags, void *data)
{
	struct cnss_driver_event *event;
	unsigned long irq_flags;
	int gfp = GFP_KERNEL;
	int ret = 0;

	if (!plat_priv)
		return -ENODEV;

	cnss_pr_dbg("Posting event: %s(%d)%s, state: 0x%lx flags: 0x%0x\n",
		    cnss_driver_event_to_str(type), type,
		    flags ? "-sync" : "", plat_priv->driver_state, flags);

	if (type >= CNSS_DRIVER_EVENT_MAX) {
		cnss_pr_err("Invalid Event type: %d, can't post", type);
		return -EINVAL;
	}

	if (in_interrupt() || irqs_disabled())
		gfp = GFP_ATOMIC;

	event = kzalloc(sizeof(*event), gfp);
	if (!event)
		return -ENOMEM;

	cnss_pm_stay_awake(plat_priv);

	event->type = type;
	event->data = data;
	init_completion(&event->complete);
	event->ret = CNSS_EVENT_PENDING;
	event->sync = !!(flags & CNSS_EVENT_SYNC);

	spin_lock_irqsave(&plat_priv->event_lock, irq_flags);
	list_add_tail(&event->list, &plat_priv->event_list);
	spin_unlock_irqrestore(&plat_priv->event_lock, irq_flags);

	queue_work(plat_priv->event_wq, &plat_priv->event_work);

	if (!(flags & CNSS_EVENT_SYNC))
		goto out;

	if (flags & CNSS_EVENT_UNKILLABLE)
		wait_for_completion(&event->complete);
	else if (flags & CNSS_EVENT_UNINTERRUPTIBLE)
		ret = wait_for_completion_killable(&event->complete);
	else
		ret = wait_for_completion_interruptible(&event->complete);

	cnss_pr_dbg("Completed event: %s(%d), state: 0x%lx, ret: %d/%d\n",
		    cnss_driver_event_to_str(type), type,
		    plat_priv->driver_state, ret, event->ret);
	spin_lock_irqsave(&plat_priv->event_lock, irq_flags);
	if (ret == -ERESTARTSYS && event->ret == CNSS_EVENT_PENDING) {
		event->sync = false;
		spin_unlock_irqrestore(&plat_priv->event_lock, irq_flags);
		ret = -EINTR;
		goto out;
	}
	spin_unlock_irqrestore(&plat_priv->event_lock, irq_flags);

	ret = event->ret;
	kfree(event);

out:
	cnss_pm_relax(plat_priv);
	return ret;
}

/**
 * cnss_get_timeout - Get timeout for corresponding type.
 * @plat_priv: Pointer to platform driver context.
 * @cnss_timeout_type: Timeout type.
 *
 * Return: Timeout in milliseconds.
 */
unsigned int cnss_get_timeout(struct cnss_plat_data *plat_priv,
			      enum cnss_timeout_type timeout_type)
{
	unsigned int qmi_timeout = cnss_get_qmi_timeout(plat_priv);

	switch (timeout_type) {
	case CNSS_TIMEOUT_QMI:
		return qmi_timeout;
	case CNSS_TIMEOUT_POWER_UP:
		return (qmi_timeout << 2);
	case CNSS_TIMEOUT_IDLE_RESTART:
		/* In idle restart power up sequence, we have fw_boot_timer to
		 * handle FW initialization failure.
		 * It uses WLAN_MISSION_MODE_TIMEOUT, so setup 3x that time to
		 * account for FW dump collection and FW re-initialization on
		 * retry.
		 */
		return (qmi_timeout + WLAN_MISSION_MODE_TIMEOUT * 3);
	case CNSS_TIMEOUT_CALIBRATION:
		/* Similar to mission mode, in CBC if FW init fails
		 * fw recovery is tried. Thus return 2x the CBC timeout.
		 */
		return (qmi_timeout + WLAN_COLD_BOOT_CAL_TIMEOUT * 2);
	case CNSS_TIMEOUT_WLAN_WATCHDOG:
		return ((qmi_timeout << 1) + WLAN_WD_TIMEOUT_MS);
	case CNSS_TIMEOUT_RDDM:
		return CNSS_RDDM_TIMEOUT_MS;
	case CNSS_TIMEOUT_RECOVERY:
		return RECOVERY_TIMEOUT;
	case CNSS_TIMEOUT_DAEMON_CONNECTION:
		return qmi_timeout + CNSS_DAEMON_CONNECT_TIMEOUT_MS;
	default:
		return qmi_timeout;
	}
}

unsigned int cnss_get_boot_timeout(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return 0;
	}

	return cnss_get_timeout(plat_priv, CNSS_TIMEOUT_QMI);
}
EXPORT_SYMBOL(cnss_get_boot_timeout);

int cnss_power_up(struct device *dev)
{
	int ret = 0;
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	unsigned int timeout;

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	cnss_pr_dbg("Powering up device\n");

	ret = cnss_driver_event_post(plat_priv,
				     CNSS_DRIVER_EVENT_POWER_UP,
				     CNSS_EVENT_SYNC, NULL);
	if (ret)
		goto out;

	if (plat_priv->device_id == QCA6174_DEVICE_ID)
		goto out;

	timeout = cnss_get_timeout(plat_priv, CNSS_TIMEOUT_POWER_UP);

	reinit_completion(&plat_priv->power_up_complete);
	ret = wait_for_completion_timeout(&plat_priv->power_up_complete,
					  msecs_to_jiffies(timeout));
	if (!ret) {
		cnss_pr_err("Timeout (%ums) waiting for power up to complete\n",
			    timeout);
		ret = -EAGAIN;
		goto out;
	}

	return 0;

out:
	return ret;
}
EXPORT_SYMBOL(cnss_power_up);

int cnss_power_down(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	cnss_pr_dbg("Powering down device\n");

	return cnss_driver_event_post(plat_priv,
				      CNSS_DRIVER_EVENT_POWER_DOWN,
				      CNSS_EVENT_SYNC, NULL);
}
EXPORT_SYMBOL(cnss_power_down);

int cnss_idle_restart(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	unsigned int timeout;
	int ret = 0;

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	if (!mutex_trylock(&plat_priv->driver_ops_lock)) {
		cnss_pr_dbg("Another driver operation is in progress, ignore idle restart\n");
		return -EBUSY;
	}

	cnss_pr_dbg("Doing idle restart\n");

	reinit_completion(&plat_priv->power_up_complete);

	if (test_bit(CNSS_IN_REBOOT, &plat_priv->driver_state)) {
		cnss_pr_dbg("Reboot or shutdown is in progress, ignore idle restart\n");
		ret = -EINVAL;
		goto out;
	}

	ret = cnss_driver_event_post(plat_priv,
				     CNSS_DRIVER_EVENT_IDLE_RESTART,
				     CNSS_EVENT_SYNC_UNINTERRUPTIBLE, NULL);
	if (ret == -EINTR && plat_priv->device_id != QCA6174_DEVICE_ID)
		cnss_pr_err("Idle restart has been interrupted but device power up is still in progress");
	else if (ret)
		goto out;

	if (plat_priv->device_id == QCA6174_DEVICE_ID) {
		ret = cnss_bus_call_driver_probe(plat_priv);
		goto out;
	}

	timeout = cnss_get_timeout(plat_priv, CNSS_TIMEOUT_IDLE_RESTART);
	ret = wait_for_completion_timeout(&plat_priv->power_up_complete,
					  msecs_to_jiffies(timeout));
	if (plat_priv->power_up_error) {
		ret = plat_priv->power_up_error;
		clear_bit(CNSS_DRIVER_IDLE_RESTART, &plat_priv->driver_state);
		cnss_pr_dbg("Power up error:%d, exiting\n",
			    plat_priv->power_up_error);
		goto out;
	}

	if (!ret) {
		/* This exception occurs after attempting retry of FW recovery.
		 * Thus we can safely power off the device.
		 */
		cnss_fatal_err("Timeout (%ums) waiting for idle restart to complete\n",
			       timeout);
		ret = -ETIMEDOUT;
		cnss_power_down(dev);
		CNSS_ASSERT(0);
		goto out;
	}

	if (test_bit(CNSS_IN_REBOOT, &plat_priv->driver_state)) {
		cnss_pr_dbg("Reboot or shutdown is in progress, ignore idle restart\n");
		del_timer(&plat_priv->fw_boot_timer);
		ret = -EINVAL;
		goto out;
	}

	/* In non-DRV mode, remove MHI satellite configuration. Switching to
	 * non-DRV is supported only once after device reboots and before wifi
	 * is turned on. We do not allow switching back to DRV.
	 * To bring device back into DRV, user needs to reboot device.
	 */
	if (test_bit(DISABLE_DRV, &plat_priv->ctrl_params.quirks)) {
		cnss_pr_dbg("DRV is disabled\n");
		cnss_bus_disable_mhi_satellite_cfg(plat_priv);
	}

	mutex_unlock(&plat_priv->driver_ops_lock);
	return 0;

out:
	mutex_unlock(&plat_priv->driver_ops_lock);
	return ret;
}
EXPORT_SYMBOL(cnss_idle_restart);

int cnss_idle_shutdown(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	if (test_bit(CNSS_IN_SUSPEND_RESUME, &plat_priv->driver_state)) {
		cnss_pr_dbg("System suspend or resume in progress, ignore idle shutdown\n");
		return -EAGAIN;
	}

	cnss_pr_dbg("Doing idle shutdown\n");

	if (test_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state) ||
	    test_bit(CNSS_DEV_ERR_NOTIFY, &plat_priv->driver_state)) {
		cnss_pr_dbg("Recovery in progress. Ignore IDLE Shutdown\n");
		return -EBUSY;
	}

	return cnss_driver_event_post(plat_priv,
				      CNSS_DRIVER_EVENT_IDLE_SHUTDOWN,
				      CNSS_EVENT_SYNC_UNINTERRUPTIBLE, NULL);
}
EXPORT_SYMBOL(cnss_idle_shutdown);

static int cnss_get_resources(struct cnss_plat_data *plat_priv)
{
	int ret = 0;

	ret = cnss_get_vreg_type(plat_priv, CNSS_VREG_PRIM);
	if (ret < 0) {
		cnss_pr_err("Failed to get vreg, err = %d\n", ret);
		goto out;
	}

	ret = cnss_get_clk(plat_priv);
	if (ret) {
		cnss_pr_err("Failed to get clocks, err = %d\n", ret);
		goto put_vreg;
	}

	ret = cnss_get_pinctrl(plat_priv);
	if (ret) {
		cnss_pr_err("Failed to get pinctrl, err = %d\n", ret);
		goto put_clk;
	}

	return 0;

put_clk:
	cnss_put_clk(plat_priv);
put_vreg:
	cnss_put_vreg_type(plat_priv, CNSS_VREG_PRIM);
out:
	return ret;
}

static void cnss_put_resources(struct cnss_plat_data *plat_priv)
{
	cnss_put_clk(plat_priv);
	cnss_put_vreg_type(plat_priv, CNSS_VREG_PRIM);
}

#if IS_ENABLED(CONFIG_ESOC) && IS_ENABLED(CONFIG_MSM_SUBSYSTEM_RESTART)
static int cnss_modem_notifier_nb(struct notifier_block *nb,
				  unsigned long code,
				  void *ss_handle)
{
	struct cnss_plat_data *plat_priv =
		container_of(nb, struct cnss_plat_data, modem_nb);
	struct cnss_esoc_info *esoc_info;

	cnss_pr_dbg("Modem notifier: event %lu\n", code);

	if (!plat_priv)
		return NOTIFY_DONE;

	esoc_info = &plat_priv->esoc_info;

	if (code == SUBSYS_AFTER_POWERUP)
		esoc_info->modem_current_status = 1;
	else if (code == SUBSYS_BEFORE_SHUTDOWN)
		esoc_info->modem_current_status = 0;
	else
		return NOTIFY_DONE;

	if (!cnss_bus_call_driver_modem_status(plat_priv,
					       esoc_info->modem_current_status))
		return NOTIFY_DONE;

	return NOTIFY_OK;
}

static int cnss_register_esoc(struct cnss_plat_data *plat_priv)
{
	int ret = 0;
	struct device *dev;
	struct cnss_esoc_info *esoc_info;
	struct esoc_desc *esoc_desc;
	const char *client_desc;

	dev = &plat_priv->plat_dev->dev;
	esoc_info = &plat_priv->esoc_info;

	esoc_info->notify_modem_status =
		of_property_read_bool(dev->of_node,
				      "qcom,notify-modem-status");

	if (!esoc_info->notify_modem_status)
		goto out;

	ret = of_property_read_string_index(dev->of_node, "esoc-names", 0,
					    &client_desc);
	if (ret) {
		cnss_pr_dbg("esoc-names is not defined in DT, skip!\n");
	} else {
		esoc_desc = devm_register_esoc_client(dev, client_desc);
		if (IS_ERR_OR_NULL(esoc_desc)) {
			ret = PTR_RET(esoc_desc);
			cnss_pr_err("Failed to register esoc_desc, err = %d\n",
				    ret);
			goto out;
		}
		esoc_info->esoc_desc = esoc_desc;
	}

	plat_priv->modem_nb.notifier_call = cnss_modem_notifier_nb;
	esoc_info->modem_current_status = 0;
	esoc_info->modem_notify_handler =
		subsys_notif_register_notifier(esoc_info->esoc_desc ?
					       esoc_info->esoc_desc->name :
					       "modem", &plat_priv->modem_nb);
	if (IS_ERR(esoc_info->modem_notify_handler)) {
		ret = PTR_ERR(esoc_info->modem_notify_handler);
		cnss_pr_err("Failed to register esoc notifier, err = %d\n",
			    ret);
		goto unreg_esoc;
	}

	return 0;
unreg_esoc:
	if (esoc_info->esoc_desc)
		devm_unregister_esoc_client(dev, esoc_info->esoc_desc);
out:
	return ret;
}

static void cnss_unregister_esoc(struct cnss_plat_data *plat_priv)
{
	struct device *dev;
	struct cnss_esoc_info *esoc_info;

	dev = &plat_priv->plat_dev->dev;
	esoc_info = &plat_priv->esoc_info;

	if (esoc_info->notify_modem_status)
		subsys_notif_unregister_notifier
		(esoc_info->modem_notify_handler,
		 &plat_priv->modem_nb);
	if (esoc_info->esoc_desc)
		devm_unregister_esoc_client(dev, esoc_info->esoc_desc);
}
#else
static inline int cnss_register_esoc(struct cnss_plat_data *plat_priv)
{
	return 0;
}

static inline void cnss_unregister_esoc(struct cnss_plat_data *plat_priv) {}
#endif

int cnss_enable_dev_sol_irq(struct cnss_plat_data *plat_priv)
{
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;
	int ret = 0;

	if (sol_gpio->dev_sol_gpio < 0 || sol_gpio->dev_sol_irq <= 0)
		return 0;

	enable_irq(sol_gpio->dev_sol_irq);
	ret = enable_irq_wake(sol_gpio->dev_sol_irq);
	if (ret)
		cnss_pr_err("Failed to enable device SOL as wake IRQ, err = %d\n",
			    ret);

	return ret;
}

int cnss_disable_dev_sol_irq(struct cnss_plat_data *plat_priv)
{
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;
	int ret = 0;

	if (sol_gpio->dev_sol_gpio < 0 || sol_gpio->dev_sol_irq <= 0)
		return 0;

	ret = disable_irq_wake(sol_gpio->dev_sol_irq);
	if (ret)
		cnss_pr_err("Failed to disable device SOL as wake IRQ, err = %d\n",
			    ret);
	disable_irq(sol_gpio->dev_sol_irq);

	return ret;
}

int cnss_get_dev_sol_value(struct cnss_plat_data *plat_priv)
{
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;

	if (sol_gpio->dev_sol_gpio < 0)
		return -EINVAL;

	return gpio_get_value(sol_gpio->dev_sol_gpio);
}

static irqreturn_t cnss_dev_sol_handler(int irq, void *data)
{
	struct cnss_plat_data *plat_priv = data;
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;

	sol_gpio->dev_sol_counter++;
	cnss_pr_dbg("WLAN device SOL IRQ (%u) is asserted #%u\n",
		    irq, sol_gpio->dev_sol_counter);

	/* Make sure abort current suspend */
	cnss_pm_stay_awake(plat_priv);
	cnss_pm_relax(plat_priv);
	pm_system_wakeup();

	cnss_bus_handle_dev_sol_irq(plat_priv);

	return IRQ_HANDLED;
}

static int cnss_init_dev_sol_gpio(struct cnss_plat_data *plat_priv)
{
	struct device *dev = &plat_priv->plat_dev->dev;
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;
	int ret = 0;

	sol_gpio->dev_sol_gpio = of_get_named_gpio(dev->of_node,
						   "wlan-dev-sol-gpio", 0);
	if (sol_gpio->dev_sol_gpio < 0)
		goto out;

	cnss_pr_dbg("Get device SOL GPIO (%d) from device node\n",
		    sol_gpio->dev_sol_gpio);

	ret = gpio_request(sol_gpio->dev_sol_gpio, "wlan_dev_sol_gpio");
	if (ret) {
		cnss_pr_err("Failed to request device SOL GPIO, err = %d\n",
			    ret);
		goto out;
	}

	gpio_direction_input(sol_gpio->dev_sol_gpio);
	sol_gpio->dev_sol_irq = gpio_to_irq(sol_gpio->dev_sol_gpio);

	ret = request_irq(sol_gpio->dev_sol_irq, cnss_dev_sol_handler,
			  IRQF_TRIGGER_FALLING, "wlan_dev_sol_irq", plat_priv);
	if (ret) {
		cnss_pr_err("Failed to request device SOL IRQ, err = %d\n", ret);
		goto free_gpio;
	}

	return 0;

free_gpio:
	gpio_free(sol_gpio->dev_sol_gpio);
out:
	return ret;
}

static void cnss_deinit_dev_sol_gpio(struct cnss_plat_data *plat_priv)
{
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;

	if (sol_gpio->dev_sol_gpio < 0)
		return;

	free_irq(sol_gpio->dev_sol_irq, plat_priv);
	gpio_free(sol_gpio->dev_sol_gpio);
}

int cnss_set_host_sol_value(struct cnss_plat_data *plat_priv, int value)
{
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;

	if (sol_gpio->host_sol_gpio < 0)
		return -EINVAL;

	if (value)
		cnss_pr_dbg("Assert host SOL GPIO\n");
	gpio_set_value(sol_gpio->host_sol_gpio, value);

	return 0;
}

int cnss_get_host_sol_value(struct cnss_plat_data *plat_priv)
{
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;

	if (sol_gpio->host_sol_gpio < 0)
		return -EINVAL;

	return gpio_get_value(sol_gpio->host_sol_gpio);
}

static int cnss_init_host_sol_gpio(struct cnss_plat_data *plat_priv)
{
	struct device *dev = &plat_priv->plat_dev->dev;
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;
	int ret = 0;

	sol_gpio->host_sol_gpio = of_get_named_gpio(dev->of_node,
						    "wlan-host-sol-gpio", 0);
	if (sol_gpio->host_sol_gpio < 0)
		goto out;

	cnss_pr_dbg("Get host SOL GPIO (%d) from device node\n",
		    sol_gpio->host_sol_gpio);

	ret = gpio_request(sol_gpio->host_sol_gpio, "wlan_host_sol_gpio");
	if (ret) {
		cnss_pr_err("Failed to request host SOL GPIO, err = %d\n",
			    ret);
		goto out;
	}

	gpio_direction_output(sol_gpio->host_sol_gpio, 0);

	return 0;

out:
	return ret;
}

static void cnss_deinit_host_sol_gpio(struct cnss_plat_data *plat_priv)
{
	struct cnss_sol_gpio *sol_gpio = &plat_priv->sol_gpio;

	if (sol_gpio->host_sol_gpio < 0)
		return;

	gpio_free(sol_gpio->host_sol_gpio);
}

static int cnss_init_sol_gpio(struct cnss_plat_data *plat_priv)
{
	int ret;

	ret = cnss_init_dev_sol_gpio(plat_priv);
	if (ret)
		goto out;

	ret = cnss_init_host_sol_gpio(plat_priv);
	if (ret)
		goto deinit_dev_sol;

	return 0;

deinit_dev_sol:
	cnss_deinit_dev_sol_gpio(plat_priv);
out:
	return ret;
}

static void cnss_deinit_sol_gpio(struct cnss_plat_data *plat_priv)
{
	cnss_deinit_host_sol_gpio(plat_priv);
	cnss_deinit_dev_sol_gpio(plat_priv);
}

#if IS_ENABLED(CONFIG_MSM_SUBSYSTEM_RESTART)
static int cnss_subsys_powerup(const struct subsys_desc *subsys_desc)
{
	struct cnss_plat_data *plat_priv;
	int ret = 0;

	if (!subsys_desc->dev) {
		cnss_pr_err("dev from subsys_desc is NULL\n");
		return -ENODEV;
	}

	plat_priv = dev_get_drvdata(subsys_desc->dev);
	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	if (!plat_priv->driver_state) {
		cnss_pr_dbg("subsys powerup is ignored\n");
		return 0;
	}

	ret = cnss_bus_dev_powerup(plat_priv);
	if (ret)
		__pm_relax(plat_priv->recovery_ws);
	return ret;
}

static int cnss_subsys_shutdown(const struct subsys_desc *subsys_desc,
				bool force_stop)
{
	struct cnss_plat_data *plat_priv;

	if (!subsys_desc->dev) {
		cnss_pr_err("dev from subsys_desc is NULL\n");
		return -ENODEV;
	}

	plat_priv = dev_get_drvdata(subsys_desc->dev);
	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	if (!plat_priv->driver_state) {
		cnss_pr_dbg("subsys shutdown is ignored\n");
		return 0;
	}

	return cnss_bus_dev_shutdown(plat_priv);
}

void cnss_device_crashed(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	struct cnss_subsys_info *subsys_info;

	if (!plat_priv)
		return;

	subsys_info = &plat_priv->subsys_info;
	if (subsys_info->subsys_device) {
		set_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state);
		subsys_set_crash_status(subsys_info->subsys_device, true);
		subsystem_restart_dev(subsys_info->subsys_device);
	}
}
EXPORT_SYMBOL(cnss_device_crashed);

static void cnss_subsys_crash_shutdown(const struct subsys_desc *subsys_desc)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(subsys_desc->dev);

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return;
	}

	cnss_bus_dev_crash_shutdown(plat_priv);
}

static int cnss_subsys_ramdump(int enable,
			       const struct subsys_desc *subsys_desc)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(subsys_desc->dev);

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	if (!enable)
		return 0;

	return cnss_bus_dev_ramdump(plat_priv);
}

static void cnss_recovery_work_handler(struct work_struct *work)
{
}
#else
void cnss_recovery_handler(struct cnss_plat_data *plat_priv)
{
	int ret;

	set_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state);

	if (!plat_priv->recovery_enabled)
		panic("subsys-restart: Resetting the SoC wlan crashed\n");

	cnss_bus_dev_shutdown(plat_priv);
	cnss_bus_dev_ramdump(plat_priv);

	/* If recovery is triggered before Host driver registration,
	 * avoid device power up because eventually device will be
	 * power up as part of driver registration.
	 */
	if (!test_bit(CNSS_DRIVER_REGISTER, &plat_priv->driver_state) ||
	    !test_bit(CNSS_DRIVER_REGISTERED, &plat_priv->driver_state)) {
		cnss_pr_dbg("Host driver not registered yet, ignore Device Power Up, 0x%lx\n",
			    plat_priv->driver_state);
		return;
	}

	msleep(POWER_RESET_MIN_DELAY_MS);

	ret = cnss_bus_dev_powerup(plat_priv);
	if (ret) {
		__pm_relax(plat_priv->recovery_ws);
		clear_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state);
	}

	return;
}

static void cnss_recovery_work_handler(struct work_struct *work)
{
	struct cnss_plat_data *plat_priv =
		container_of(work, struct cnss_plat_data, recovery_work);

	cnss_recovery_handler(plat_priv);
}

void cnss_device_crashed(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv)
		return;

	set_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state);
	schedule_work(&plat_priv->recovery_work);
}
EXPORT_SYMBOL(cnss_device_crashed);
#endif /* CONFIG_MSM_SUBSYSTEM_RESTART */

void *cnss_get_virt_ramdump_mem(struct device *dev, unsigned long *size)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	struct cnss_ramdump_info *ramdump_info;

	if (!plat_priv)
		return NULL;

	ramdump_info = &plat_priv->ramdump_info;
	*size = ramdump_info->ramdump_size;

	return ramdump_info->ramdump_va;
}
EXPORT_SYMBOL(cnss_get_virt_ramdump_mem);

static const char *cnss_recovery_reason_to_str(enum cnss_recovery_reason reason)
{
	switch (reason) {
	case CNSS_REASON_DEFAULT:
		return "DEFAULT";
	case CNSS_REASON_LINK_DOWN:
		return "LINK_DOWN";
	case CNSS_REASON_RDDM:
		return "RDDM";
	case CNSS_REASON_TIMEOUT:
		return "TIMEOUT";
	}

	return "UNKNOWN";
};

static int cnss_do_recovery(struct cnss_plat_data *plat_priv,
			    enum cnss_recovery_reason reason)
{
	int ret;

	plat_priv->recovery_count++;

	if (plat_priv->device_id == QCA6174_DEVICE_ID)
		goto self_recovery;

	if (test_bit(SKIP_RECOVERY, &plat_priv->ctrl_params.quirks)) {
		cnss_pr_dbg("Skip device recovery\n");
		return 0;
	}

	/* FW recovery sequence has multiple steps and firmware load requires
	 * linux PM in awake state. Thus hold the cnss wake source until
	 * WLAN MISSION enabled. CNSS_TIMEOUT_RECOVERY option should cover all
	 * time taken in this process.
	 */
	pm_wakeup_ws_event(plat_priv->recovery_ws,
			   cnss_get_timeout(plat_priv, CNSS_TIMEOUT_RECOVERY),
			   true);

	switch (reason) {
	case CNSS_REASON_LINK_DOWN:
		if (!cnss_bus_check_link_status(plat_priv)) {
			cnss_pr_dbg("Skip link down recovery as link is already up\n");
			return 0;
		}
		if (test_bit(LINK_DOWN_SELF_RECOVERY,
			     &plat_priv->ctrl_params.quirks))
			goto self_recovery;
		if (!cnss_bus_recover_link_down(plat_priv)) {
			/* clear recovery bit here to avoid skipping
			 * the recovery work for RDDM later
			 */
			clear_bit(CNSS_DRIVER_RECOVERY,
				  &plat_priv->driver_state);
			return 0;
		}
		break;
	case CNSS_REASON_RDDM:
		cnss_bus_collect_dump_info(plat_priv, false);
		break;
	case CNSS_REASON_DEFAULT:
	case CNSS_REASON_TIMEOUT:
		break;
	default:
		cnss_pr_err("Unsupported recovery reason: %s(%d)\n",
			    cnss_recovery_reason_to_str(reason), reason);
		break;
	}
	cnss_bus_device_crashed(plat_priv);

	return 0;

self_recovery:
	cnss_pr_dbg("Going for self recovery\n");
	cnss_bus_dev_shutdown(plat_priv);

	if (test_bit(LINK_DOWN_SELF_RECOVERY, &plat_priv->ctrl_params.quirks))
		clear_bit(LINK_DOWN_SELF_RECOVERY,
			  &plat_priv->ctrl_params.quirks);

	/* If link down self recovery is triggered before Host driver
	 * registration, avoid device power up because eventually device
	 * will be power up as part of driver registration.
	 */

	if (!test_bit(CNSS_DRIVER_REGISTER, &plat_priv->driver_state) ||
	    !test_bit(CNSS_DRIVER_REGISTERED, &plat_priv->driver_state)) {
		cnss_pr_dbg("Host driver not registered yet, ignore Device Power Up, 0x%lx\n",
			    plat_priv->driver_state);
		return 0;
	}

	ret = cnss_bus_dev_powerup(plat_priv);
	if (ret)
		clear_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state);

	return 0;
}

static int cnss_driver_recovery_hdlr(struct cnss_plat_data *plat_priv,
				     void *data)
{
	struct cnss_recovery_data *recovery_data = data;
	int ret = 0;

	cnss_pr_dbg("Driver recovery is triggered with reason: %s(%d)\n",
		    cnss_recovery_reason_to_str(recovery_data->reason),
		    recovery_data->reason);

	snprintf(plat_priv->crash_reason_buf, sizeof(plat_priv->crash_reason_buf),
		"Driver recovery is triggered with reason: %s(%d)\n",
		cnss_recovery_reason_to_str(recovery_data->reason),
		recovery_data->reason);

	sysfs_notify(&plat_priv->plat_dev->dev.kobj, NULL, "crash_reason");

	if (!plat_priv->driver_state) {
		cnss_pr_err("Improper driver state, ignore recovery\n");
		ret = -EINVAL;
		goto out;
	}

	if (test_bit(CNSS_IN_REBOOT, &plat_priv->driver_state)) {
		cnss_pr_err("Reboot is in progress, ignore recovery\n");
		ret = -EINVAL;
		goto out;
	}

	if (test_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state)) {
		cnss_pr_err("Recovery is already in progress\n");
		CNSS_ASSERT(0);
		ret = -EINVAL;
		goto out;
	}

	if (test_bit(CNSS_DRIVER_UNLOADING, &plat_priv->driver_state) ||
	    test_bit(CNSS_DRIVER_IDLE_SHUTDOWN, &plat_priv->driver_state)) {
		cnss_pr_err("Driver unload or idle shutdown is in progress, ignore recovery\n");
		ret = -EINVAL;
		goto out;
	}

	switch (plat_priv->device_id) {
	case QCA6174_DEVICE_ID:
		if (test_bit(CNSS_DRIVER_LOADING, &plat_priv->driver_state) ||
		    test_bit(CNSS_DRIVER_IDLE_RESTART,
			     &plat_priv->driver_state)) {
			cnss_pr_err("Driver load or idle restart is in progress, ignore recovery\n");
			ret = -EINVAL;
			goto out;
		}
		break;
	default:
		if (!test_bit(CNSS_FW_READY, &plat_priv->driver_state)) {
			set_bit(CNSS_FW_BOOT_RECOVERY,
				&plat_priv->driver_state);
		}
		break;
	}

	set_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state);
	ret = cnss_do_recovery(plat_priv, recovery_data->reason);

out:
	kfree(data);
	return ret;
}

int cnss_self_recovery(struct device *dev,
		       enum cnss_recovery_reason reason)
{
	cnss_schedule_recovery(dev, reason);
	return 0;
}
EXPORT_SYMBOL(cnss_self_recovery);

void cnss_schedule_recovery(struct device *dev,
			    enum cnss_recovery_reason reason)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	struct cnss_recovery_data *data;
	int gfp = GFP_KERNEL;

	if (!test_bit(CNSS_DEV_ERR_NOTIFY, &plat_priv->driver_state))
		cnss_bus_update_status(plat_priv, CNSS_FW_DOWN);

	if (test_bit(CNSS_DRIVER_UNLOADING, &plat_priv->driver_state) ||
	    test_bit(CNSS_DRIVER_IDLE_SHUTDOWN, &plat_priv->driver_state)) {
		cnss_pr_dbg("Driver unload or idle shutdown is in progress, ignore schedule recovery\n");
		return;
	}

	if (in_interrupt() || irqs_disabled())
		gfp = GFP_ATOMIC;

	data = kzalloc(sizeof(*data), gfp);
	if (!data)
		return;

	data->reason = reason;
	cnss_driver_event_post(plat_priv,
			       CNSS_DRIVER_EVENT_RECOVERY,
			       0, data);
}
EXPORT_SYMBOL(cnss_schedule_recovery);

int cnss_force_fw_assert(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	if (plat_priv->device_id == QCA6174_DEVICE_ID) {
		cnss_pr_info("Forced FW assert is not supported\n");
		return -EOPNOTSUPP;
	}

	if (cnss_bus_is_device_down(plat_priv)) {
		cnss_pr_info("Device is already in bad state, ignore force assert\n");
		return 0;
	}

	if (test_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state)) {
		cnss_pr_info("Recovery is already in progress, ignore forced FW assert\n");
		return 0;
	}

	if (in_interrupt() || irqs_disabled())
		cnss_driver_event_post(plat_priv,
				       CNSS_DRIVER_EVENT_FORCE_FW_ASSERT,
				       0, NULL);
	else
		cnss_bus_force_fw_assert_hdlr(plat_priv);

	return 0;
}
EXPORT_SYMBOL(cnss_force_fw_assert);

int cnss_force_collect_rddm(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	unsigned int timeout;
	int ret = 0;

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return -ENODEV;
	}

	if (plat_priv->device_id == QCA6174_DEVICE_ID) {
		cnss_pr_info("Force collect rddm is not supported\n");
		return -EOPNOTSUPP;
	}

	if (cnss_bus_is_device_down(plat_priv)) {
		cnss_pr_info("Device is already in bad state, wait to collect rddm\n");
		goto wait_rddm;
	}

	if (test_bit(CNSS_DRIVER_RECOVERY, &plat_priv->driver_state)) {
		cnss_pr_info("Recovery is already in progress, wait to collect rddm\n");
		goto wait_rddm;
	}

	if (test_bit(CNSS_DRIVER_LOADING, &plat_priv->driver_state) ||
	    test_bit(CNSS_DRIVER_UNLOADING, &plat_priv->driver_state) ||
	    test_bit(CNSS_DRIVER_IDLE_RESTART, &plat_priv->driver_state) ||
	    test_bit(CNSS_DRIVER_IDLE_SHUTDOWN, &plat_priv->driver_state)) {
		cnss_pr_info("Loading/Unloading/idle restart/shutdown is in progress, ignore forced collect rddm\n");
		return 0;
	}

	ret = cnss_bus_force_fw_assert_hdlr(plat_priv);
	if (ret)
		return ret;

wait_rddm:
	reinit_completion(&plat_priv->rddm_complete);
	timeout = cnss_get_timeout(plat_priv, CNSS_TIMEOUT_RDDM);
	ret = wait_for_completion_timeout(&plat_priv->rddm_complete,
					  msecs_to_jiffies(timeout));
	if (!ret) {
		cnss_pr_err("Timeout (%ums) waiting for RDDM to complete\n",
			    timeout);
		ret = -ETIMEDOUT;
	} else if (ret > 0) {
		ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL(cnss_force_collect_rddm);

int cnss_qmi_send_get(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!test_bit(CNSS_QMI_WLFW_CONNECTED, &plat_priv->driver_state))
		return 0;

	return cnss_bus_qmi_send_get(plat_priv);
}
EXPORT_SYMBOL(cnss_qmi_send_get);

int cnss_qmi_send_put(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!test_bit(CNSS_QMI_WLFW_CONNECTED, &plat_priv->driver_state))
		return 0;

	return cnss_bus_qmi_send_put(plat_priv);
}
EXPORT_SYMBOL(cnss_qmi_send_put);

int cnss_qmi_send(struct device *dev, int type, void *cmd,
		  int cmd_len, void *cb_ctx,
		  int (*cb)(void *ctx, void *event, int event_len))
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	int ret;

	if (!plat_priv)
		return -ENODEV;

	if (!test_bit(CNSS_QMI_WLFW_CONNECTED, &plat_priv->driver_state))
		return -EINVAL;

	plat_priv->get_info_cb = cb;
	plat_priv->get_info_cb_ctx = cb_ctx;

	ret = cnss_wlfw_get_info_send_sync(plat_priv, type, cmd, cmd_len);
	if (ret) {
		plat_priv->get_info_cb = NULL;
		plat_priv->get_info_cb_ctx = NULL;
	}

	return ret;
}
EXPORT_SYMBOL(cnss_qmi_send);

static int cnss_cold_boot_cal_start_hdlr(struct cnss_plat_data *plat_priv)
{
	int ret = 0;
	u32 retry = 0, timeout;

	if (test_bit(CNSS_COLD_BOOT_CAL_DONE, &plat_priv->driver_state)) {
		cnss_pr_dbg("Calibration complete. Ignore calibration req\n");
		goto out;
	} else if (test_bit(CNSS_IN_COLD_BOOT_CAL, &plat_priv->driver_state)) {
		cnss_pr_dbg("Calibration in progress. Ignore new calibration req\n");
		goto out;
	} else if (test_bit(CNSS_WLAN_HW_DISABLED, &plat_priv->driver_state)) {
		cnss_pr_dbg("Calibration deferred as WLAN device disabled\n");
		goto out;
	}

	if (test_bit(CNSS_DRIVER_LOADING, &plat_priv->driver_state) ||
	    test_bit(CNSS_DRIVER_PROBED, &plat_priv->driver_state) ||
	    test_bit(CNSS_FW_READY, &plat_priv->driver_state)) {
		cnss_pr_err("WLAN in mission mode before cold boot calibration\n");
		CNSS_ASSERT(0);
		return -EINVAL;
	}

	while (retry++ < CNSS_CAL_START_PROBE_WAIT_RETRY_MAX) {
		if (test_bit(CNSS_PCI_PROBE_DONE, &plat_priv->driver_state))
			break;
		msleep(CNSS_CAL_START_PROBE_WAIT_MS);

		if (retry == CNSS_CAL_START_PROBE_WAIT_RETRY_MAX) {
			cnss_pr_err("Calibration start failed as PCI probe not complete\n");
			CNSS_ASSERT(0);
			ret = -EINVAL;
			goto mark_cal_fail;
		}
	}

	switch (plat_priv->device_id) {
	case QCA6290_DEVICE_ID:
	case QCA6390_DEVICE_ID:
	case QCA6490_DEVICE_ID:
	case KIWI_DEVICE_ID:
	case MANGO_DEVICE_ID:
	case PEACH_DEVICE_ID:
		break;
	default:
		cnss_pr_err("Not supported for device ID 0x%lx\n",
			    plat_priv->device_id);
		ret = -EINVAL;
		goto mark_cal_fail;
	}

	set_bit(CNSS_IN_COLD_BOOT_CAL, &plat_priv->driver_state);
	if (test_bit(CNSS_DRIVER_REGISTER, &plat_priv->driver_state)) {
		timeout = cnss_get_timeout(plat_priv,
					   CNSS_TIMEOUT_CALIBRATION);
		cnss_pr_dbg("Restarting calibration %ds timeout\n",
			    timeout / 1000);
		if (cancel_delayed_work_sync(&plat_priv->wlan_reg_driver_work))
			schedule_delayed_work(&plat_priv->wlan_reg_driver_work,
					      msecs_to_jiffies(timeout));
	}
	reinit_completion(&plat_priv->cal_complete);
	ret = cnss_bus_dev_powerup(plat_priv);
mark_cal_fail:
	if (ret) {
		complete(&plat_priv->cal_complete);
		clear_bit(CNSS_IN_COLD_BOOT_CAL, &plat_priv->driver_state);
		/* Set CBC done in driver state to mark attempt and note error
		 * since calibration cannot be retried at boot.
		 */
		plat_priv->cal_done = CNSS_CAL_FAILURE;
		set_bit(CNSS_COLD_BOOT_CAL_DONE, &plat_priv->driver_state);

		if (plat_priv->device_id == QCA6174_DEVICE_ID ||
		    plat_priv->device_id == QCN7605_DEVICE_ID) {
			if (!test_bit(CNSS_DRIVER_REGISTER, &plat_priv->driver_state))
				goto out;

			cnss_pr_info("Schedule WLAN driver load\n");

			if (cancel_delayed_work_sync(&plat_priv->wlan_reg_driver_work))
				schedule_delayed_work(&plat_priv->wlan_reg_driver_work,
						      0);
		}
	}

out:
	return ret;
}

static int cnss_cold_boot_cal_done_hdlr(struct cnss_plat_data *plat_priv,
					void *data)
{
	struct cnss_cal_info *cal_info = data;

	if (!test_bit(CNSS_IN_COLD_BOOT_CAL, &plat_priv->driver_state) ||
	    test_bit(CNSS_COLD_BOOT_CAL_DONE, &plat_priv->driver_state))
		goto out;

	switch (cal_info->cal_status) {
	case CNSS_CAL_DONE:
		cnss_pr_dbg("Calibration completed successfully\n");
		plat_priv->cal_done = true;
		break;
	case CNSS_CAL_TIMEOUT:
	case CNSS_CAL_FAILURE:
		cnss_pr_dbg("Calibration failed. Status: %d, force shutdown\n",
			    cal_info->cal_status);
		break;
	default:
		cnss_pr_err("Unknown calibration status: %u\n",
			    cal_info->cal_status);
		break;
	}

	cnss_wlfw_wlan_mode_send_sync(plat_priv, CNSS_OFF);
	cnss_bus_free_qdss_mem(plat_priv);
	cnss_release_antenna_sharing(plat_priv);

	if (plat_priv->device_id == QCN7605_DEVICE_ID)
		goto skip_shutdown;

	cnss_bus_dev_shutdown(plat_priv);
	msleep(POWER_RESET_MIN_DELAY_MS);

skip_shutdown:
	complete(&plat_priv->cal_complete);
	clear_bit(CNSS_IN_COLD_BOOT_CAL, &plat_priv->driver_state);
	set_bit(CNSS_COLD_BOOT_CAL_DONE, &plat_priv->driver_state);

	if (cal_info->cal_status == CNSS_CAL_DONE) {
		cnss_cal_mem_upload_to_file(plat_priv);
		if (!test_bit(CNSS_DRIVER_REGISTER, &plat_priv->driver_state))
			goto out;

		cnss_pr_dbg("Schedule WLAN driver load\n");
		if (cancel_delayed_work_sync(&plat_priv->wlan_reg_driver_work))
			schedule_delayed_work(&plat_priv->wlan_reg_driver_work,
					      0);
	}
out:
	kfree(data);
	return 0;
}

static int cnss_power_up_hdlr(struct cnss_plat_data *plat_priv)
{
	int ret;

	ret = cnss_bus_dev_powerup(plat_priv);
	if (ret)
		clear_bit(CNSS_DRIVER_IDLE_RESTART, &plat_priv->driver_state);

	return ret;
}

static int cnss_power_down_hdlr(struct cnss_plat_data *plat_priv)
{
	cnss_bus_dev_shutdown(plat_priv);

	return 0;
}

static int cnss_qdss_trace_req_mem_hdlr(struct cnss_plat_data *plat_priv)
{
	int ret = 0;

	ret = cnss_bus_alloc_qdss_mem(plat_priv);
	if (ret < 0)
		return ret;

	return cnss_wlfw_qdss_trace_mem_info_send_sync(plat_priv);
}

static void *cnss_get_fw_mem_pa_to_va(struct cnss_fw_mem *fw_mem,
				      u32 mem_seg_len, u64 pa, u32 size)
{
	int i = 0;
	u64 offset = 0;
	void *va = NULL;
	u64 local_pa;
	u32 local_size;

	for (i = 0; i < mem_seg_len; i++) {
		if (i == QMI_WLFW_MEM_LPASS_SHARED_V01)
			continue;

		local_pa = (u64)fw_mem[i].pa;
		local_size = (u32)fw_mem[i].size;
		if (pa == local_pa && size <= local_size) {
			va = fw_mem[i].va;
			break;
		}
		if (pa > local_pa &&
		    pa < local_pa + local_size &&
		    pa + size <= local_pa + local_size) {
			offset = pa - local_pa;
			va = fw_mem[i].va + offset;
			break;
		}
	}
	return va;
}

static int cnss_fw_mem_file_save_hdlr(struct cnss_plat_data *plat_priv,
				      void *data)
{
	struct cnss_qmi_event_fw_mem_file_save_data *event_data = data;
	struct cnss_fw_mem *fw_mem_seg;
	int ret = 0L;
	void *va = NULL;
	u32 i, fw_mem_seg_len;

	switch (event_data->mem_type) {
	case QMI_WLFW_MEM_TYPE_DDR_V01:
		if (!plat_priv->fw_mem_seg_len)
			goto invalid_mem_save;

		fw_mem_seg = plat_priv->fw_mem;
		fw_mem_seg_len = plat_priv->fw_mem_seg_len;
		break;
	case QMI_WLFW_MEM_QDSS_V01:
		if (!plat_priv->qdss_mem_seg_len)
			goto invalid_mem_save;

		fw_mem_seg = plat_priv->qdss_mem;
		fw_mem_seg_len = plat_priv->qdss_mem_seg_len;
		break;
	default:
		goto invalid_mem_save;
	}

	for (i = 0; i < event_data->mem_seg_len; i++) {
		va = cnss_get_fw_mem_pa_to_va(fw_mem_seg, fw_mem_seg_len,
					      event_data->mem_seg[i].addr,
					      event_data->mem_seg[i].size);
		if (!va) {
			cnss_pr_err("Fail to find matching va of pa %pa for mem type: %d\n",
				    &event_data->mem_seg[i].addr,
				    event_data->mem_type);
			ret = -EINVAL;
			break;
		}
		ret = cnss_genl_send_msg(va, CNSS_GENL_MSG_TYPE_QDSS,
					 event_data->file_name,
					 event_data->mem_seg[i].size);
		if (ret < 0) {
			cnss_pr_err("Fail to save fw mem data: %d\n",
				    ret);
			break;
		}
	}
	kfree(data);
	return ret;

invalid_mem_save:
	cnss_pr_err("FW Mem type %d not allocated. Invalid save request\n",
		    event_data->mem_type);
	kfree(data);
	return -EINVAL;
}

static int cnss_qdss_trace_free_hdlr(struct cnss_plat_data *plat_priv)
{
	cnss_bus_free_qdss_mem(plat_priv);

	return 0;
}

static int cnss_qdss_trace_req_data_hdlr(struct cnss_plat_data *plat_priv,
					 void *data)
{
	int ret = 0;
	struct cnss_qmi_event_fw_mem_file_save_data *event_data = data;

	if (!plat_priv)
		return -ENODEV;

	ret = cnss_wlfw_qdss_data_send_sync(plat_priv, event_data->file_name,
					    event_data->total_size);

	kfree(data);
	return ret;
}

static void cnss_driver_event_work(struct work_struct *work)
{
	struct cnss_plat_data *plat_priv =
		container_of(work, struct cnss_plat_data, event_work);
	struct cnss_driver_event *event;
	unsigned long flags;
	int ret = 0;

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL!\n");
		return;
	}

	cnss_pm_stay_awake(plat_priv);

	spin_lock_irqsave(&plat_priv->event_lock, flags);

	while (!list_empty(&plat_priv->event_list)) {
		event = list_first_entry(&plat_priv->event_list,
					 struct cnss_driver_event, list);
		list_del(&event->list);
		spin_unlock_irqrestore(&plat_priv->event_lock, flags);

		cnss_pr_dbg("Processing driver event: %s%s(%d), state: 0x%lx\n",
			    cnss_driver_event_to_str(event->type),
			    event->sync ? "-sync" : "", event->type,
			    plat_priv->driver_state);

		switch (event->type) {
		case CNSS_DRIVER_EVENT_SERVER_ARRIVE:
			ret = cnss_wlfw_server_arrive(plat_priv, event->data);
			break;
		case CNSS_DRIVER_EVENT_SERVER_EXIT:
			ret = cnss_wlfw_server_exit(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_REQUEST_MEM:
			ret = cnss_bus_alloc_fw_mem(plat_priv);
			if (ret)
				break;
			ret = cnss_wlfw_respond_mem_send_sync(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_FW_MEM_READY:
			ret = cnss_fw_mem_ready_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_FW_READY:
			ret = cnss_fw_ready_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_COLD_BOOT_CAL_START:
			ret = cnss_cold_boot_cal_start_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_COLD_BOOT_CAL_DONE:
			ret = cnss_cold_boot_cal_done_hdlr(plat_priv,
							   event->data);
			break;
		case CNSS_DRIVER_EVENT_REGISTER_DRIVER:
			ret = cnss_bus_register_driver_hdlr(plat_priv,
							    event->data);
			break;
		case CNSS_DRIVER_EVENT_UNREGISTER_DRIVER:
			ret = cnss_bus_unregister_driver_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_RECOVERY:
			ret = cnss_driver_recovery_hdlr(plat_priv,
							event->data);
			break;
		case CNSS_DRIVER_EVENT_FORCE_FW_ASSERT:
			ret = cnss_bus_force_fw_assert_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_IDLE_RESTART:
			set_bit(CNSS_DRIVER_IDLE_RESTART,
				&plat_priv->driver_state);
			fallthrough;
		case CNSS_DRIVER_EVENT_POWER_UP:
			ret = cnss_power_up_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_IDLE_SHUTDOWN:
			set_bit(CNSS_DRIVER_IDLE_SHUTDOWN,
				&plat_priv->driver_state);
			fallthrough;
		case CNSS_DRIVER_EVENT_POWER_DOWN:
			ret = cnss_power_down_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_IMS_WFC_CALL_IND:
			ret = cnss_process_wfc_call_ind_event(plat_priv,
							      event->data);
			break;
		case CNSS_DRIVER_EVENT_WLFW_TWT_CFG_IND:
			ret = cnss_process_twt_cfg_ind_event(plat_priv,
							     event->data);
			break;
		case CNSS_DRIVER_EVENT_QDSS_TRACE_REQ_MEM:
			ret = cnss_qdss_trace_req_mem_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_FW_MEM_FILE_SAVE:
			ret = cnss_fw_mem_file_save_hdlr(plat_priv,
							 event->data);
			break;
		case CNSS_DRIVER_EVENT_QDSS_TRACE_FREE:
			ret = cnss_qdss_trace_free_hdlr(plat_priv);
			break;
		case CNSS_DRIVER_EVENT_QDSS_TRACE_REQ_DATA:
			ret = cnss_qdss_trace_req_data_hdlr(plat_priv,
							    event->data);
			break;
		default:
			cnss_pr_err("Invalid driver event type: %d",
				    event->type);
			kfree(event);
			spin_lock_irqsave(&plat_priv->event_lock, flags);
			continue;
		}

		spin_lock_irqsave(&plat_priv->event_lock, flags);
		if (event->sync) {
			event->ret = ret;
			complete(&event->complete);
			continue;
		}
		spin_unlock_irqrestore(&plat_priv->event_lock, flags);

		kfree(event);

		spin_lock_irqsave(&plat_priv->event_lock, flags);
	}
	spin_unlock_irqrestore(&plat_priv->event_lock, flags);

	cnss_pm_relax(plat_priv);
}

#if IS_ENABLED(CONFIG_MSM_SUBSYSTEM_RESTART)
int cnss_register_subsys(struct cnss_plat_data *plat_priv)
{
	int ret = 0;
	struct cnss_subsys_info *subsys_info;

	subsys_info = &plat_priv->subsys_info;

	subsys_info->subsys_desc.name = plat_priv->device_name;
	subsys_info->subsys_desc.owner = THIS_MODULE;
	subsys_info->subsys_desc.powerup = cnss_subsys_powerup;
	subsys_info->subsys_desc.shutdown = cnss_subsys_shutdown;
	subsys_info->subsys_desc.ramdump = cnss_subsys_ramdump;
	subsys_info->subsys_desc.crash_shutdown = cnss_subsys_crash_shutdown;
	subsys_info->subsys_desc.dev = &plat_priv->plat_dev->dev;

	subsys_info->subsys_device = subsys_register(&subsys_info->subsys_desc);
	if (IS_ERR(subsys_info->subsys_device)) {
		ret = PTR_ERR(subsys_info->subsys_device);
		cnss_pr_err("Failed to register subsys, err = %d\n", ret);
		goto out;
	}

	subsys_info->subsys_handle =
		subsystem_get(subsys_info->subsys_desc.name);
	if (!subsys_info->subsys_handle) {
		cnss_pr_err("Failed to get subsys_handle!\n");
		ret = -EINVAL;
		goto unregister_subsys;
	} else if (IS_ERR(subsys_info->subsys_handle)) {
		ret = PTR_ERR(subsys_info->subsys_handle);
		cnss_pr_err("Failed to do subsystem_get, err = %d\n", ret);
		goto unregister_subsys;
	}

	return 0;

unregister_subsys:
	subsys_unregister(subsys_info->subsys_device);
out:
	return ret;
}

void cnss_unregister_subsys(struct cnss_plat_data *plat_priv)
{
	struct cnss_subsys_info *subsys_info;

	subsys_info = &plat_priv->subsys_info;
	subsystem_put(subsys_info->subsys_handle);
	subsys_unregister(subsys_info->subsys_device);
}

static void *cnss_create_ramdump_device(struct cnss_plat_data *plat_priv)
{
	struct cnss_subsys_info *subsys_info = &plat_priv->subsys_info;

	return create_ramdump_device(subsys_info->subsys_desc.name,
				     subsys_info->subsys_desc.dev);
}

static void cnss_destroy_ramdump_device(struct cnss_plat_data *plat_priv,
					void *ramdump_dev)
{
	destroy_ramdump_device(ramdump_dev);
}

int cnss_do_ramdump(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info *ramdump_info = &plat_priv->ramdump_info;
	struct ramdump_segment segment;

	memset(&segment, 0, sizeof(segment));
	segment.v_address = (void __iomem *)ramdump_info->ramdump_va;
	segment.size = ramdump_info->ramdump_size;

	return qcom_ramdump(ramdump_info->ramdump_dev, &segment, 1);
}

int cnss_do_elf_ramdump(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info_v2 *info_v2 = &plat_priv->ramdump_info_v2;
	struct cnss_dump_data *dump_data = &info_v2->dump_data;
	struct cnss_dump_seg *dump_seg = info_v2->dump_data_vaddr;
	struct ramdump_segment *ramdump_segs, *s;
	struct cnss_dump_meta_info meta_info = {0};
	int i, ret = 0;

	ramdump_segs = kcalloc(dump_data->nentries + 1,
			       sizeof(*ramdump_segs),
			       GFP_KERNEL);
	if (!ramdump_segs)
		return -ENOMEM;

	s = ramdump_segs + 1;
	for (i = 0; i < dump_data->nentries; i++) {
		if (dump_seg->type >= CNSS_FW_DUMP_TYPE_MAX) {
			cnss_pr_err("Unsupported dump type: %d",
				    dump_seg->type);
			continue;
		}

		if (meta_info.entry[dump_seg->type].entry_start == 0) {
			meta_info.entry[dump_seg->type].type = dump_seg->type;
			meta_info.entry[dump_seg->type].entry_start = i + 1;
		}
		meta_info.entry[dump_seg->type].entry_num++;

		s->address = dump_seg->address;
		s->v_address = (void __iomem *)dump_seg->v_address;
		s->size = dump_seg->size;
		s++;
		dump_seg++;
	}

	meta_info.magic = CNSS_RAMDUMP_MAGIC;
	meta_info.version = CNSS_RAMDUMP_VERSION;
	meta_info.chipset = plat_priv->device_id;
	meta_info.total_entries = CNSS_FW_DUMP_TYPE_MAX;

	ramdump_segs->v_address = (void __iomem *)(&meta_info);
	ramdump_segs->size = sizeof(meta_info);

	ret = qcom_elf_ramdump(info_v2->ramdump_dev, ramdump_segs,
			       dump_data->nentries + 1);
	kfree(ramdump_segs);

	return ret;
}
#else
static int cnss_panic_handler(struct notifier_block *nb, unsigned long action,
			      void *data)
{
	struct cnss_plat_data *plat_priv =
		container_of(nb, struct cnss_plat_data, panic_nb);

	cnss_bus_dev_crash_shutdown(plat_priv);

	return NOTIFY_DONE;
}

int cnss_register_subsys(struct cnss_plat_data *plat_priv)
{
	int ret;

	if (!plat_priv)
		return -ENODEV;

	plat_priv->panic_nb.notifier_call = cnss_panic_handler;
	ret = atomic_notifier_chain_register(&panic_notifier_list,
					     &plat_priv->panic_nb);
	if (ret) {
		cnss_pr_err("Failed to register panic handler\n");
		return -EINVAL;
	}

	return 0;
}

void cnss_unregister_subsys(struct cnss_plat_data *plat_priv)
{
	int ret;

	ret = atomic_notifier_chain_unregister(&panic_notifier_list,
					       &plat_priv->panic_nb);
	if (ret)
		cnss_pr_err("Failed to unregister panic handler\n");
}

#if IS_ENABLED(CONFIG_QCOM_MEMORY_DUMP_V2)
static void *cnss_create_ramdump_device(struct cnss_plat_data *plat_priv)
{
	return &plat_priv->plat_dev->dev;
}

static void cnss_destroy_ramdump_device(struct cnss_plat_data *plat_priv,
					void *ramdump_dev)
{
}
#endif

#if IS_ENABLED(CONFIG_QCOM_RAMDUMP)
int cnss_do_ramdump(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info *ramdump_info = &plat_priv->ramdump_info;
	struct qcom_dump_segment segment;
	struct list_head head;

	INIT_LIST_HEAD(&head);
	memset(&segment, 0, sizeof(segment));
	segment.va = ramdump_info->ramdump_va;
	segment.size = ramdump_info->ramdump_size;
	list_add(&segment.node, &head);

	return qcom_dump(&head, ramdump_info->ramdump_dev);
}
#else
int cnss_do_ramdump(struct cnss_plat_data *plat_priv)
{
	return 0;
}

/* Using completion event inside dynamically allocated ramdump_desc
 * may result a race between freeing the event after setting it to
 * complete inside dev coredump free callback and the thread that is
 * waiting for completion.
 */
DECLARE_COMPLETION(dump_done);
#define TIMEOUT_SAVE_DUMP_MS 30000

#define SIZEOF_ELF_STRUCT(__xhdr)					\
static inline size_t sizeof_elf_##__xhdr(unsigned char class)		\
{									\
	if (class == ELFCLASS32)					\
		return sizeof(struct elf32_##__xhdr);			\
	else								\
		return sizeof(struct elf64_##__xhdr);			\
}

SIZEOF_ELF_STRUCT(phdr)
SIZEOF_ELF_STRUCT(hdr)

#define set_xhdr_property(__xhdr, arg, class, member, value)		\
do {									\
	if (class == ELFCLASS32)					\
		((struct elf32_##__xhdr *)arg)->member = value;		\
	else								\
		((struct elf64_##__xhdr *)arg)->member = value;		\
} while (0)

#define set_ehdr_property(arg, class, member, value) \
	set_xhdr_property(hdr, arg, class, member, value)
#define set_phdr_property(arg, class, member, value) \
	set_xhdr_property(phdr, arg, class, member, value)

/* These replace qcom_ramdump driver APIs called from common API
 * cnss_do_elf_dump() by the ones defined here.
 */
#define qcom_dump_segment cnss_qcom_dump_segment
#define qcom_elf_dump cnss_qcom_elf_dump
#define dump_enabled cnss_dump_enabled

struct cnss_qcom_dump_segment {
	struct list_head node;
	dma_addr_t da;
	void *va;
	size_t size;
};

struct cnss_qcom_ramdump_desc {
	void *data;
	struct completion dump_done;
};

static ssize_t cnss_qcom_devcd_readv(char *buffer, loff_t offset, size_t count,
				     void *data, size_t datalen)
{
	struct cnss_qcom_ramdump_desc *desc = data;

	return memory_read_from_buffer(buffer, count, &offset, desc->data,
				       datalen);
}

static void cnss_qcom_devcd_freev(void *data)
{
	struct cnss_qcom_ramdump_desc *desc = data;

	cnss_pr_dbg("Free dump data for dev coredump\n");

	complete(&dump_done);
	vfree(desc->data);
	kfree(desc);
}

static int cnss_qcom_devcd_dump(struct device *dev, void *data, size_t datalen,
				gfp_t gfp)
{
	struct cnss_qcom_ramdump_desc *desc;
	unsigned int timeout = TIMEOUT_SAVE_DUMP_MS;
	int ret;

	desc = kmalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->data = data;
	reinit_completion(&dump_done);

	dev_coredumpm(dev, NULL, desc, datalen, gfp,
		      cnss_qcom_devcd_readv, cnss_qcom_devcd_freev);

	ret = wait_for_completion_timeout(&dump_done,
					  msecs_to_jiffies(timeout));
	if (!ret)
		cnss_pr_err("Timeout waiting (%dms) for saving dump to file system\n",
			    timeout);

	return ret ? 0 : -ETIMEDOUT;
}

/* Since the elf32 and elf64 identification is identical apart from
 * the class, use elf32 by default.
 */
static void init_elf_identification(struct elf32_hdr *ehdr, unsigned char class)
{
	memcpy(ehdr->e_ident, ELFMAG, SELFMAG);
	ehdr->e_ident[EI_CLASS] = class;
	ehdr->e_ident[EI_DATA] = ELFDATA2LSB;
	ehdr->e_ident[EI_VERSION] = EV_CURRENT;
	ehdr->e_ident[EI_OSABI] = ELFOSABI_NONE;
}

int cnss_qcom_elf_dump(struct list_head *segs, struct device *dev,
		       unsigned char class)
{
	struct cnss_qcom_dump_segment *segment;
	void *phdr, *ehdr;
	size_t data_size, offset;
	int phnum = 0;
	void *data;
	void __iomem *ptr;

	if (!segs || list_empty(segs))
		return -EINVAL;

	data_size = sizeof_elf_hdr(class);
	list_for_each_entry(segment, segs, node) {
		data_size += sizeof_elf_phdr(class) + segment->size;
		phnum++;
	}

	data = vmalloc(data_size);
	if (!data)
		return -ENOMEM;

	cnss_pr_dbg("Creating ELF file with size %d\n", data_size);

	ehdr = data;
	memset(ehdr, 0, sizeof_elf_hdr(class));
	init_elf_identification(ehdr, class);
	set_ehdr_property(ehdr, class, e_type, ET_CORE);
	set_ehdr_property(ehdr, class, e_machine, EM_NONE);
	set_ehdr_property(ehdr, class, e_version, EV_CURRENT);
	set_ehdr_property(ehdr, class, e_phoff, sizeof_elf_hdr(class));
	set_ehdr_property(ehdr, class, e_ehsize, sizeof_elf_hdr(class));
	set_ehdr_property(ehdr, class, e_phentsize, sizeof_elf_phdr(class));
	set_ehdr_property(ehdr, class, e_phnum, phnum);

	phdr = data + sizeof_elf_hdr(class);
	offset = sizeof_elf_hdr(class) + sizeof_elf_phdr(class) * phnum;
	list_for_each_entry(segment, segs, node) {
		memset(phdr, 0, sizeof_elf_phdr(class));
		set_phdr_property(phdr, class, p_type, PT_LOAD);
		set_phdr_property(phdr, class, p_offset, offset);
		set_phdr_property(phdr, class, p_vaddr, segment->da);
		set_phdr_property(phdr, class, p_paddr, segment->da);
		set_phdr_property(phdr, class, p_filesz, segment->size);
		set_phdr_property(phdr, class, p_memsz, segment->size);
		set_phdr_property(phdr, class, p_flags, PF_R | PF_W | PF_X);
		set_phdr_property(phdr, class, p_align, 0);

		if (segment->va) {
			memcpy(data + offset, segment->va, segment->size);
		} else {
			ptr = devm_ioremap(dev, segment->da, segment->size);
			if (!ptr) {
				cnss_pr_err("Invalid coredump segment (%pad, %zu)\n",
					    &segment->da, segment->size);
				memset(data + offset, 0xff, segment->size);
			} else {
				memcpy_fromio(data + offset, ptr,
					      segment->size);
			}
		}

		offset += segment->size;
		phdr += sizeof_elf_phdr(class);
	}

	return cnss_qcom_devcd_dump(dev, data, data_size, GFP_KERNEL);
}

/* Saving dump to file system is always needed in this case. */
static bool cnss_dump_enabled(void)
{
	return true;
}
#endif /* CONFIG_QCOM_RAMDUMP */

int cnss_do_elf_ramdump(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info_v2 *info_v2 = &plat_priv->ramdump_info_v2;
	struct cnss_dump_data *dump_data = &info_v2->dump_data;
	struct cnss_dump_seg *dump_seg = info_v2->dump_data_vaddr;
	struct qcom_dump_segment *seg;
	struct cnss_dump_meta_info meta_info = {0};
	struct list_head head;
	int i, ret = 0;

	if (!dump_enabled()) {
		cnss_pr_info("Dump collection is not enabled\n");
		return ret;
	}

	INIT_LIST_HEAD(&head);
	for (i = 0; i < dump_data->nentries; i++) {
		if (dump_seg->type >= CNSS_FW_DUMP_TYPE_MAX) {
			cnss_pr_err("Unsupported dump type: %d",
				    dump_seg->type);
			continue;
		}

		seg = kcalloc(1, sizeof(*seg), GFP_KERNEL);
		if (!seg) {
			cnss_pr_err("%s: Failed to allocate mem for seg %d\n",
				    __func__, i);
			continue;
		}

		if (meta_info.entry[dump_seg->type].entry_start == 0) {
			meta_info.entry[dump_seg->type].type = dump_seg->type;
			meta_info.entry[dump_seg->type].entry_start = i + 1;
		}
		meta_info.entry[dump_seg->type].entry_num++;
		seg->da = dump_seg->address;
		seg->va = dump_seg->v_address;
		seg->size = dump_seg->size;
		list_add_tail(&seg->node, &head);
		dump_seg++;
	}

	seg = kcalloc(1, sizeof(*seg), GFP_KERNEL);
	if (!seg) {
		cnss_pr_err("%s: Failed to allocate mem for elf ramdump seg\n",
			    __func__);
		goto skip_elf_dump;
	}

	meta_info.magic = CNSS_RAMDUMP_MAGIC;
	meta_info.version = CNSS_RAMDUMP_VERSION;
	meta_info.chipset = plat_priv->device_id;
	meta_info.total_entries = CNSS_FW_DUMP_TYPE_MAX;
	seg->va = &meta_info;
	seg->size = sizeof(meta_info);
	list_add(&seg->node, &head);

	ret = qcom_elf_dump(&head, info_v2->ramdump_dev, ELF_CLASS);

skip_elf_dump:
	while (!list_empty(&head)) {
		seg = list_first_entry(&head, struct qcom_dump_segment, node);
		list_del(&seg->node);
		kfree(seg);
	}

	return ret;
}

#ifdef CONFIG_CNSS2_SSR_DRIVER_DUMP
/**
 * cnss_host_ramdump_dev_release() - callback function for device release
 * @dev: device to be released
 *
 * Return: None
 */
static void cnss_host_ramdump_dev_release(struct device *dev)
{
	cnss_pr_dbg("free host ramdump device\n");
	kfree(dev);
}

int cnss_do_host_ramdump(struct cnss_plat_data *plat_priv,
			 struct cnss_ssr_driver_dump_entry *ssr_entry,
			 size_t num_entries_loaded)
{
	struct qcom_dump_segment *seg;
	struct cnss_host_dump_meta_info meta_info = {0};
	struct list_head head;
	int dev_ret = 0;
	struct device *new_device;
	static const char * const wlan_str[] = {
		[CNSS_HOST_WLAN_LOGS] = "wlan_logs",
		[CNSS_HOST_HTC_CREDIT] = "htc_credit",
		[CNSS_HOST_WMI_TX_CMP] = "wmi_tx_cmp",
		[CNSS_HOST_WMI_COMMAND_LOG] = "wmi_command_log",
		[CNSS_HOST_WMI_EVENT_LOG] = "wmi_event_log",
		[CNSS_HOST_WMI_RX_EVENT] = "wmi_rx_event",
		[CNSS_HOST_HAL_SOC] = "hal_soc",
		[CNSS_HOST_GWLAN_LOGGING] = "gwlan_logging",
		[CNSS_HOST_WMI_DEBUG_LOG_INFO] = "wmi_debug_log_info",
		[CNSS_HOST_HTC_CREDIT_IDX] = "htc_credit_history_idx",
		[CNSS_HOST_HTC_CREDIT_LEN] = "htc_credit_history_length",
		[CNSS_HOST_WMI_TX_CMP_IDX] = "wmi_tx_cmp_idx",
		[CNSS_HOST_WMI_COMMAND_LOG_IDX] = "wmi_command_log_idx",
		[CNSS_HOST_WMI_EVENT_LOG_IDX] = "wmi_event_log_idx",
		[CNSS_HOST_WMI_RX_EVENT_IDX] = "wmi_rx_event_idx",
		[CNSS_HOST_HIF_CE_DESC_HISTORY_BUFF] = "hif_ce_desc_history_buff",
		[CNSS_HOST_HANG_EVENT_DATA] = "hang_event_data",
		[CNSS_HOST_CE_DESC_HIST] = "hif_ce_desc_hist",
		[CNSS_HOST_CE_COUNT_MAX] = "hif_ce_count_max",
		[CNSS_HOST_CE_HISTORY_MAX] = "hif_ce_history_max",
		[CNSS_HOST_ONLY_FOR_CRIT_CE] = "hif_ce_only_for_crit",
		[CNSS_HOST_HIF_EVENT_HISTORY] = "hif_event_history",
		[CNSS_HOST_HIF_EVENT_HIST_MAX] = "hif_event_hist_max",
		[CNSS_HOST_DP_WBM_DESC_REL] = "wbm_desc_rel_ring",
		[CNSS_HOST_DP_WBM_DESC_REL_HANDLE] = "wbm_desc_rel_ring_handle",
		[CNSS_HOST_DP_TCL_CMD] = "tcl_cmd_ring",
		[CNSS_HOST_DP_TCL_CMD_HANDLE] = "tcl_cmd_ring_handle",
		[CNSS_HOST_DP_TCL_STATUS] = "tcl_status_ring",
		[CNSS_HOST_DP_TCL_STATUS_HANDLE] = "tcl_status_ring_handle",
		[CNSS_HOST_DP_REO_REINJ] = "reo_reinject_ring",
		[CNSS_HOST_DP_REO_REINJ_HANDLE] = "reo_reinject_ring_handle",
		[CNSS_HOST_DP_RX_REL] = "rx_rel_ring",
		[CNSS_HOST_DP_RX_REL_HANDLE] = "rx_rel_ring_handle",
		[CNSS_HOST_DP_REO_EXP] = "reo_exception_ring",
		[CNSS_HOST_DP_REO_EXP_HANDLE] = "reo_exception_ring_handle",
		[CNSS_HOST_DP_REO_CMD] = "reo_cmd_ring",
		[CNSS_HOST_DP_REO_CMD_HANDLE] = "reo_cmd_ring_handle",
		[CNSS_HOST_DP_REO_STATUS] = "reo_status_ring",
		[CNSS_HOST_DP_REO_STATUS_HANDLE] = "reo_status_ring_handle",
		[CNSS_HOST_DP_TCL_DATA_0] = "tcl_data_ring_0",
		[CNSS_HOST_DP_TCL_DATA_0_HANDLE] = "tcl_data_ring_0_handle",
		[CNSS_HOST_DP_TX_COMP_0] = "tx_comp_ring_0",
		[CNSS_HOST_DP_TX_COMP_0_HANDLE] = "tx_comp_ring_0_handle",
		[CNSS_HOST_DP_TCL_DATA_1] = "tcl_data_ring_1",
		[CNSS_HOST_DP_TCL_DATA_1_HANDLE] = "tcl_data_ring_1_handle",
		[CNSS_HOST_DP_TX_COMP_1] = "tx_comp_ring_1",
		[CNSS_HOST_DP_TX_COMP_1_HANDLE] = "tx_comp_ring_1_handle",
		[CNSS_HOST_DP_TCL_DATA_2] = "tcl_data_ring_2",
		[CNSS_HOST_DP_TCL_DATA_2_HANDLE] = "tcl_data_ring_2_handle",
		[CNSS_HOST_DP_TX_COMP_2] = "tx_comp_ring_2",
		[CNSS_HOST_DP_TX_COMP_2_HANDLE] = "tx_comp_ring_2_handle",
		[CNSS_HOST_DP_REO_DST_0] = "reo_dest_ring_0",
		[CNSS_HOST_DP_REO_DST_0_HANDLE] = "reo_dest_ring_0_handle",
		[CNSS_HOST_DP_REO_DST_1] = "reo_dest_ring_1",
		[CNSS_HOST_DP_REO_DST_1_HANDLE] = "reo_dest_ring_1_handle",
		[CNSS_HOST_DP_REO_DST_2] = "reo_dest_ring_2",
		[CNSS_HOST_DP_REO_DST_2_HANDLE] = "reo_dest_ring_2_handle",
		[CNSS_HOST_DP_REO_DST_3] = "reo_dest_ring_3",
		[CNSS_HOST_DP_REO_DST_3_HANDLE] = "reo_dest_ring_3_handle",
		[CNSS_HOST_DP_REO_DST_4] = "reo_dest_ring_4",
		[CNSS_HOST_DP_REO_DST_4_HANDLE] = "reo_dest_ring_4_handle",
		[CNSS_HOST_DP_REO_DST_5] = "reo_dest_ring_5",
		[CNSS_HOST_DP_REO_DST_5_HANDLE] = "reo_dest_ring_5_handle",
		[CNSS_HOST_DP_REO_DST_6] = "reo_dest_ring_6",
		[CNSS_HOST_DP_REO_DST_6_HANDLE] = "reo_dest_ring_6_handle",
		[CNSS_HOST_DP_REO_DST_7] = "reo_dest_ring_7",
		[CNSS_HOST_DP_REO_DST_7_HANDLE] = "reo_dest_ring_7_handle",
		[CNSS_HOST_DP_PDEV_0] = "dp_pdev_0",
		[CNSS_HOST_DP_WLAN_CFG_CTX] = "wlan_cfg_ctx",
		[CNSS_HOST_DP_SOC] = "dp_soc",
		[CNSS_HOST_HAL_RX_FST] = "hal_rx_fst",
		[CNSS_HOST_DP_FISA] = "dp_fisa",
		[CNSS_HOST_DP_FISA_HW_FSE_TABLE] = "dp_fisa_hw_fse_table",
		[CNSS_HOST_DP_FISA_SW_FSE_TABLE] = "dp_fisa_sw_fse_table",
		[CNSS_HOST_HIF] = "hif",
		[CNSS_HOST_QDF_NBUF_HIST] = "qdf_nbuf_history",
		[CNSS_HOST_TCL_WBM_MAP] = "tcl_wbm_map_array",
		[CNSS_HOST_RX_MAC_BUF_RING_0] = "rx_mac_buf_ring_0",
		[CNSS_HOST_RX_MAC_BUF_RING_0_HANDLE] = "rx_mac_buf_ring_0_handle",
		[CNSS_HOST_RX_MAC_BUF_RING_1] = "rx_mac_buf_ring_1",
		[CNSS_HOST_RX_MAC_BUF_RING_1_HANDLE] = "rx_mac_buf_ring_1_handle",
		[CNSS_HOST_RX_REFILL_0] = "rx_refill_buf_ring_0",
		[CNSS_HOST_RX_REFILL_0_HANDLE] = "rx_refill_buf_ring_0_handle",
		[CNSS_HOST_CE_0] = "ce_0",
		[CNSS_HOST_CE_0_SRC_RING] = "ce_0_src_ring",
		[CNSS_HOST_CE_0_SRC_RING_CTX] = "ce_0_src_ring_ctx",
		[CNSS_HOST_CE_1] = "ce_1",
		[CNSS_HOST_CE_1_STATUS_RING] = "ce_1_status_ring",
		[CNSS_HOST_CE_1_STATUS_RING_CTX] = "ce_1_status_ring_ctx",
		[CNSS_HOST_CE_1_DEST_RING] = "ce_1_dest_ring",
		[CNSS_HOST_CE_1_DEST_RING_CTX] = "ce_1_dest_ring_ctx",
		[CNSS_HOST_CE_2] = "ce_2",
		[CNSS_HOST_CE_2_STATUS_RING] = "ce_2_status_ring",
		[CNSS_HOST_CE_2_STATUS_RING_CTX] = "ce_2_status_ring_ctx",
		[CNSS_HOST_CE_2_DEST_RING] = "ce_2_dest_ring",
		[CNSS_HOST_CE_2_DEST_RING_CTX] = "ce_2_dest_ring_ctx",
		[CNSS_HOST_CE_3] = "ce_3",
		[CNSS_HOST_CE_3_SRC_RING] = "ce_3_src_ring",
		[CNSS_HOST_CE_3_SRC_RING_CTX] = "ce_3_src_ring_ctx",
		[CNSS_HOST_CE_4] = "ce_4",
		[CNSS_HOST_CE_4_SRC_RING] = "ce_4_src_ring",
		[CNSS_HOST_CE_4_SRC_RING_CTX] = "ce_4_src_ring_ctx",
		[CNSS_HOST_CE_5] = "ce_5",
		[CNSS_HOST_CE_6] = "ce_6",
		[CNSS_HOST_CE_7] = "ce_7",
		[CNSS_HOST_CE_7_STATUS_RING] = "ce_7_status_ring",
		[CNSS_HOST_CE_7_STATUS_RING_CTX] = "ce_7_status_ring_ctx",
		[CNSS_HOST_CE_7_DEST_RING] = "ce_7_dest_ring",
		[CNSS_HOST_CE_7_DEST_RING_CTX] = "ce_7_dest_ring_ctx",
		[CNSS_HOST_CE_8] = "ce_8",
		[CNSS_HOST_DP_TCL_DATA_3] = "tcl_data_ring_3",
		[CNSS_HOST_DP_TCL_DATA_3_HANDLE] = "tcl_data_ring_3_handle",
		[CNSS_HOST_DP_TX_COMP_3] = "tx_comp_ring_3",
		[CNSS_HOST_DP_TX_COMP_3_HANDLE] = "tx_comp_ring_3_handle"
	};
	int i;
	int ret = 0;
	enum cnss_host_dump_type j;

	if (!dump_enabled()) {
		cnss_pr_info("Dump collection is not enabled\n");
		return ret;
	}

	new_device = kcalloc(1, sizeof(*new_device), GFP_KERNEL);
	if (!new_device) {
		cnss_pr_err("Failed to alloc device mem\n");
		return -ENOMEM;
	}

	new_device->release = cnss_host_ramdump_dev_release;
	device_initialize(new_device);
	dev_set_name(new_device, "wlan_driver");
	dev_ret = device_add(new_device);
	if (dev_ret) {
		cnss_pr_err("Failed to add new device\n");
		goto put_device;
	}

	INIT_LIST_HEAD(&head);
	for (i = 0; i < num_entries_loaded; i++) {
		/* If region name registered by driver is not present in
		 * wlan_str. type for that entry will not be set, but entry will
		 * be added. Which will result in entry type being 0. Currently
		 * entry type 0 is for wlan_logs, which will result in parsing
		 * issue for wlan_logs as parsing is done based upon type field.
		 * So initialize type with -1(Invalid) to avoid such issues.
		 */
		meta_info.entry[i].type = -1;
		seg = kcalloc(1, sizeof(*seg), GFP_KERNEL);
		if (!seg) {
			cnss_pr_err("Failed to alloc seg entry %d\n", i);
			continue;
		}

		seg->va = ssr_entry[i].buffer_pointer;
		seg->da = (dma_addr_t)ssr_entry[i].buffer_pointer;
		seg->size = ssr_entry[i].buffer_size;

		for (j = 0; j < CNSS_HOST_DUMP_TYPE_MAX; j++) {
			if (strcmp(ssr_entry[i].region_name, wlan_str[j]) == 0) {
				meta_info.entry[i].type = j;
			}
		}
		meta_info.entry[i].entry_start = i + 1;
		meta_info.entry[i].entry_num++;

		list_add_tail(&seg->node, &head);
	}

	seg = kcalloc(1, sizeof(*seg), GFP_KERNEL);

	if (!seg) {
		cnss_pr_err("%s: Failed to allocate mem for host dump seg\n",
			    __func__);
		goto skip_host_dump;
	}

	meta_info.magic = CNSS_RAMDUMP_MAGIC;
	meta_info.version = CNSS_RAMDUMP_VERSION;
	meta_info.chipset = plat_priv->device_id;
	meta_info.total_entries = num_entries_loaded;
	seg->va = &meta_info;
	seg->da = (dma_addr_t)&meta_info;
	seg->size = sizeof(meta_info);
	list_add(&seg->node, &head);

	ret = qcom_elf_dump(&head, new_device, ELF_CLASS);

skip_host_dump:
	while (!list_empty(&head)) {
		seg = list_first_entry(&head, struct qcom_dump_segment, node);
		list_del(&seg->node);
		kfree(seg);
	}
	device_del(new_device);
put_device:
	put_device(new_device);
	cnss_pr_dbg("host ramdump result %d\n", ret);
	return ret;
}
#endif
#endif /* CONFIG_MSM_SUBSYSTEM_RESTART */

#if IS_ENABLED(CONFIG_QCOM_MEMORY_DUMP_V2)
static int cnss_init_dump_entry(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info *ramdump_info;
	struct msm_dump_entry dump_entry;

	ramdump_info = &plat_priv->ramdump_info;
	ramdump_info->dump_data.addr = ramdump_info->ramdump_pa;
	ramdump_info->dump_data.len = ramdump_info->ramdump_size;
	ramdump_info->dump_data.version = CNSS_DUMP_FORMAT_VER;
	ramdump_info->dump_data.magic = CNSS_DUMP_MAGIC_VER_V2;
	strlcpy(ramdump_info->dump_data.name, CNSS_DUMP_NAME,
		sizeof(ramdump_info->dump_data.name));
	dump_entry.id = MSM_DUMP_DATA_CNSS_WLAN;
	dump_entry.addr = virt_to_phys(&ramdump_info->dump_data);

	return msm_dump_data_register_nominidump(MSM_DUMP_TABLE_APPS,
						&dump_entry);
}

static int cnss_register_ramdump_v1(struct cnss_plat_data *plat_priv)
{
	int ret = 0;
	struct device *dev;
	struct cnss_ramdump_info *ramdump_info;
	u32 ramdump_size = 0;

	dev = &plat_priv->plat_dev->dev;
	ramdump_info = &plat_priv->ramdump_info;

	if (plat_priv->dt_type != CNSS_DTT_MULTIEXCHG) {
		/* dt type: legacy or converged */
		ret = of_property_read_u32(dev->of_node,
					   "qcom,wlan-ramdump-dynamic",
					   &ramdump_size);
	} else {
		ret = of_property_read_u32(plat_priv->dev_node,
					   "qcom,wlan-ramdump-dynamic",
					   &ramdump_size);
	}
	if (ret == 0) {
		ramdump_info->ramdump_va =
			dma_alloc_coherent(dev, ramdump_size,
					   &ramdump_info->ramdump_pa,
					   GFP_KERNEL);

		if (ramdump_info->ramdump_va)
			ramdump_info->ramdump_size = ramdump_size;
	}

	cnss_pr_dbg("ramdump va: %pK, pa: %pa\n",
		    ramdump_info->ramdump_va, &ramdump_info->ramdump_pa);

	if (ramdump_info->ramdump_size == 0) {
		cnss_pr_info("Ramdump will not be collected");
		goto out;
	}

	ret = cnss_init_dump_entry(plat_priv);
	if (ret) {
		cnss_pr_err("Failed to setup dump table, err = %d\n", ret);
		goto free_ramdump;
	}

	ramdump_info->ramdump_dev = cnss_create_ramdump_device(plat_priv);
	if (!ramdump_info->ramdump_dev) {
		cnss_pr_err("Failed to create ramdump device!");
		ret = -ENOMEM;
		goto free_ramdump;
	}

	return 0;
free_ramdump:
	dma_free_coherent(dev, ramdump_info->ramdump_size,
			  ramdump_info->ramdump_va, ramdump_info->ramdump_pa);
out:
	return ret;
}

static void cnss_unregister_ramdump_v1(struct cnss_plat_data *plat_priv)
{
	struct device *dev;
	struct cnss_ramdump_info *ramdump_info;

	dev = &plat_priv->plat_dev->dev;
	ramdump_info = &plat_priv->ramdump_info;

	if (ramdump_info->ramdump_dev)
		cnss_destroy_ramdump_device(plat_priv,
					    ramdump_info->ramdump_dev);

	if (ramdump_info->ramdump_va)
		dma_free_coherent(dev, ramdump_info->ramdump_size,
				  ramdump_info->ramdump_va,
				  ramdump_info->ramdump_pa);
}

/**
 * cnss_ignore_dump_data_reg_fail - Ignore Ramdump table register failure
 * @ret: Error returned by msm_dump_data_register_nominidump
 *
 * For Lahaina GKI boot, we dont have support for mem dump feature. So
 * ignore failure.
 *
 * Return: Same given error code if mem dump feature enabled, 0 otherwise
 */
static int cnss_ignore_dump_data_reg_fail(int ret)
{
	return ret;
}

static int cnss_register_ramdump_v2(struct cnss_plat_data *plat_priv)
{
	int ret = 0;
	struct cnss_ramdump_info_v2 *info_v2;
	struct cnss_dump_data *dump_data;
	struct msm_dump_entry dump_entry;
	struct device *dev = &plat_priv->plat_dev->dev;
	u32 ramdump_size = 0;

	info_v2 = &plat_priv->ramdump_info_v2;
	dump_data = &info_v2->dump_data;

	if (plat_priv->dt_type != CNSS_DTT_MULTIEXCHG) {
		/* dt type: legacy or converged */
		ret = of_property_read_u32(dev->of_node,
					   "qcom,wlan-ramdump-dynamic",
					   &ramdump_size);
	} else {
		ret = of_property_read_u32(plat_priv->dev_node,
					   "qcom,wlan-ramdump-dynamic",
					   &ramdump_size);
	}
	if (ret == 0)
		info_v2->ramdump_size = ramdump_size;

	cnss_pr_dbg("Ramdump size 0x%lx\n", info_v2->ramdump_size);

	info_v2->dump_data_vaddr = kzalloc(CNSS_DUMP_DESC_SIZE, GFP_KERNEL);
	if (!info_v2->dump_data_vaddr)
		return -ENOMEM;

	dump_data->paddr = virt_to_phys(info_v2->dump_data_vaddr);
	dump_data->version = CNSS_DUMP_FORMAT_VER_V2;
	dump_data->magic = CNSS_DUMP_MAGIC_VER_V2;
	dump_data->seg_version = CNSS_DUMP_SEG_VER;
	strlcpy(dump_data->name, CNSS_DUMP_NAME,
		sizeof(dump_data->name));
	dump_entry.id = MSM_DUMP_DATA_CNSS_WLAN;
	dump_entry.addr = virt_to_phys(dump_data);

	ret = msm_dump_data_register_nominidump(MSM_DUMP_TABLE_APPS,
						&dump_entry);
	if (ret) {
		ret = cnss_ignore_dump_data_reg_fail(ret);
		cnss_pr_err("Failed to setup dump table, %s (%d)\n",
			    ret ? "Error" : "Ignoring", ret);
		goto free_ramdump;
	}

	info_v2->ramdump_dev = cnss_create_ramdump_device(plat_priv);
	if (!info_v2->ramdump_dev) {
		cnss_pr_err("Failed to create ramdump device!\n");
		ret = -ENOMEM;
		goto free_ramdump;
	}

	return 0;

free_ramdump:
	kfree(info_v2->dump_data_vaddr);
	info_v2->dump_data_vaddr = NULL;
	return ret;
}

static void cnss_unregister_ramdump_v2(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info_v2 *info_v2;

	info_v2 = &plat_priv->ramdump_info_v2;

	if (info_v2->ramdump_dev)
		cnss_destroy_ramdump_device(plat_priv, info_v2->ramdump_dev);

	kfree(info_v2->dump_data_vaddr);
	info_v2->dump_data_vaddr = NULL;
	info_v2->dump_data_valid = false;
}

int cnss_register_ramdump(struct cnss_plat_data *plat_priv)
{
	int ret = 0;

	switch (plat_priv->device_id) {
	case QCA6174_DEVICE_ID:
		ret = cnss_register_ramdump_v1(plat_priv);
		break;
	case QCA6290_DEVICE_ID:
	case QCA6390_DEVICE_ID:
	case QCN7605_DEVICE_ID:
	case QCA6490_DEVICE_ID:
	case KIWI_DEVICE_ID:
	case MANGO_DEVICE_ID:
	case PEACH_DEVICE_ID:
		ret = cnss_register_ramdump_v2(plat_priv);
		break;
	default:
		cnss_pr_err("Unknown device ID: 0x%lx\n", plat_priv->device_id);
		ret = -ENODEV;
		break;
	}
	return ret;
}

void cnss_unregister_ramdump(struct cnss_plat_data *plat_priv)
{
	switch (plat_priv->device_id) {
	case QCA6174_DEVICE_ID:
		cnss_unregister_ramdump_v1(plat_priv);
		break;
	case QCA6290_DEVICE_ID:
	case QCA6390_DEVICE_ID:
	case QCN7605_DEVICE_ID:
	case QCA6490_DEVICE_ID:
	case KIWI_DEVICE_ID:
	case MANGO_DEVICE_ID:
	case PEACH_DEVICE_ID:
		cnss_unregister_ramdump_v2(plat_priv);
		break;
	default:
		cnss_pr_err("Unknown device ID: 0x%lx\n", plat_priv->device_id);
		break;
	}
}
#else
int cnss_register_ramdump(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info_v2 *info_v2 = &plat_priv->ramdump_info_v2;
	struct cnss_dump_data *dump_data = dump_data = &info_v2->dump_data;
	struct device *dev = &plat_priv->plat_dev->dev;
	u32 ramdump_size = 0;

	if (of_property_read_u32(dev->of_node, "qcom,wlan-ramdump-dynamic",
				 &ramdump_size) == 0)
		info_v2->ramdump_size = ramdump_size;

	cnss_pr_dbg("Ramdump size 0x%lx\n", info_v2->ramdump_size);

	info_v2->dump_data_vaddr = kzalloc(CNSS_DUMP_DESC_SIZE, GFP_KERNEL);
	if (!info_v2->dump_data_vaddr)
		return -ENOMEM;

	dump_data->paddr = virt_to_phys(info_v2->dump_data_vaddr);
	dump_data->version = CNSS_DUMP_FORMAT_VER_V2;
	dump_data->magic = CNSS_DUMP_MAGIC_VER_V2;
	dump_data->seg_version = CNSS_DUMP_SEG_VER;
	strlcpy(dump_data->name, CNSS_DUMP_NAME,
		sizeof(dump_data->name));

	info_v2->ramdump_dev = dev;

	return 0;
}

void cnss_unregister_ramdump(struct cnss_plat_data *plat_priv)
{
	struct cnss_ramdump_info_v2 *info_v2 = &plat_priv->ramdump_info_v2;

	info_v2->ramdump_dev = NULL;
	kfree(info_v2->dump_data_vaddr);
	info_v2->dump_data_vaddr = NULL;
	info_v2->dump_data_valid = false;
}
#endif /* CONFIG_QCOM_MEMORY_DUMP_V2 */

#if IS_ENABLED(CONFIG_QCOM_MINIDUMP)
int cnss_va_to_pa(struct device *dev, size_t size, void *va, dma_addr_t dma,
		  phys_addr_t *pa, unsigned long attrs)
{
	struct sg_table sgt;
	int ret;

	ret = dma_get_sgtable_attrs(dev, &sgt, va, dma, size, attrs);
	if (ret) {
		cnss_pr_err("Failed to get sgtable for va: 0x%pK, dma: %pa, size: 0x%zx, attrs: 0x%x\n",
			    va, &dma, size, attrs);
		return -EINVAL;
	}

	*pa = page_to_phys(sg_page(sgt.sgl));
	sg_free_table(&sgt);

	return 0;
}

int cnss_minidump_add_region(struct cnss_plat_data *plat_priv,
			     enum cnss_fw_dump_type type, int seg_no,
			     void *va, phys_addr_t pa, size_t size)
{
	struct md_region md_entry;
	int ret;

	switch (type) {
	case CNSS_FW_IMAGE:
		snprintf(md_entry.name, sizeof(md_entry.name), "FBC_%X",
			 seg_no);
		break;
	case CNSS_FW_RDDM:
		snprintf(md_entry.name, sizeof(md_entry.name), "RDDM_%X",
			 seg_no);
		break;
	case CNSS_FW_REMOTE_HEAP:
		snprintf(md_entry.name, sizeof(md_entry.name), "RHEAP_%X",
			 seg_no);
		break;
	default:
		cnss_pr_err("Unknown dump type ID: %d\n", type);
		return -EINVAL;
	}

	md_entry.phys_addr = pa;
	md_entry.virt_addr = (uintptr_t)va;
	md_entry.size = size;
	md_entry.id = MSM_DUMP_DATA_CNSS_WLAN;

	cnss_pr_dbg("Mini dump region: %s, va: %pK, pa: %pa, size: 0x%zx\n",
		    md_entry.name, va, &pa, size);

	ret = msm_minidump_add_region(&md_entry);
	if (ret < 0)
		cnss_pr_err("Failed to add mini dump region, err = %d\n", ret);

	return ret;
}

int cnss_minidump_remove_region(struct cnss_plat_data *plat_priv,
				enum cnss_fw_dump_type type, int seg_no,
				void *va, phys_addr_t pa, size_t size)
{
	struct md_region md_entry;
	int ret;

	switch (type) {
	case CNSS_FW_IMAGE:
		snprintf(md_entry.name, sizeof(md_entry.name), "FBC_%X",
			 seg_no);
		break;
	case CNSS_FW_RDDM:
		snprintf(md_entry.name, sizeof(md_entry.name), "RDDM_%X",
			 seg_no);
		break;
	case CNSS_FW_REMOTE_HEAP:
		snprintf(md_entry.name, sizeof(md_entry.name), "RHEAP_%X",
			 seg_no);
		break;
	default:
		cnss_pr_err("Unknown dump type ID: %d\n", type);
		return -EINVAL;
	}

	md_entry.phys_addr = pa;
	md_entry.virt_addr = (uintptr_t)va;
	md_entry.size = size;
	md_entry.id = MSM_DUMP_DATA_CNSS_WLAN;

	cnss_pr_vdbg("Remove mini dump region: %s, va: %pK, pa: %pa, size: 0x%zx\n",
		     md_entry.name, va, &pa, size);

	ret = msm_minidump_remove_region(&md_entry);
	if (ret)
		cnss_pr_err("Failed to remove mini dump region, err = %d\n",
			    ret);

	return ret;
}
#else
int cnss_va_to_pa(struct device *dev, size_t size, void *va, dma_addr_t dma,
		  phys_addr_t *pa, unsigned long attrs)
{
	return 0;
}

int cnss_minidump_add_region(struct cnss_plat_data *plat_priv,
			     enum cnss_fw_dump_type type, int seg_no,
			     void *va, phys_addr_t pa, size_t size)
{
	return 0;
}

int cnss_minidump_remove_region(struct cnss_plat_data *plat_priv,
				enum cnss_fw_dump_type type, int seg_no,
				void *va, phys_addr_t pa, size_t size)
{
	return 0;
}
#endif /* CONFIG_QCOM_MINIDUMP */

int cnss_request_firmware_direct(struct cnss_plat_data *plat_priv,
				 const struct firmware **fw_entry,
				 const char *filename)
{
	if (IS_ENABLED(CONFIG_CNSS_REQ_FW_DIRECT))
		return request_firmware_direct(fw_entry, filename,
					       &plat_priv->plat_dev->dev);
	else
		return firmware_request_nowarn(fw_entry, filename,
					       &plat_priv->plat_dev->dev);
}

#if IS_ENABLED(CONFIG_INTERCONNECT)
/**
 * cnss_register_bus_scale() - Setup interconnect voting data
 * @plat_priv: Platform data structure
 *
 * For different interconnect path configured in device tree setup voting data
 * for list of bandwidth requirements.
 *
 * Result: 0 for success. -EINVAL if not configured
 */
static int cnss_register_bus_scale(struct cnss_plat_data *plat_priv)
{
	int ret = -EINVAL;
	u32 idx, i, j, cfg_arr_size, *cfg_arr = NULL;
	struct cnss_bus_bw_info *bus_bw_info, *tmp;
	struct device *dev = &plat_priv->plat_dev->dev;

	INIT_LIST_HEAD(&plat_priv->icc.list_head);
	ret = of_property_read_u32(dev->of_node,
				   "qcom,icc-path-count",
				   &plat_priv->icc.path_count);
	if (ret) {
		cnss_pr_dbg("Platform Bus Interconnect path not configured\n");
		return 0;
	}
	ret = of_property_read_u32(plat_priv->plat_dev->dev.of_node,
				   "qcom,bus-bw-cfg-count",
				   &plat_priv->icc.bus_bw_cfg_count);
	if (ret) {
		cnss_pr_err("Failed to get Bus BW Config table size\n");
		goto cleanup;
	}
	cfg_arr_size = plat_priv->icc.path_count *
			 plat_priv->icc.bus_bw_cfg_count * CNSS_ICC_VOTE_MAX;
	cfg_arr = kcalloc(cfg_arr_size, sizeof(*cfg_arr), GFP_KERNEL);
	if (!cfg_arr) {
		cnss_pr_err("Failed to alloc cfg table mem\n");
		ret = -ENOMEM;
		goto cleanup;
	}

	ret = of_property_read_u32_array(plat_priv->plat_dev->dev.of_node,
					 "qcom,bus-bw-cfg", cfg_arr,
					 cfg_arr_size);
	if (ret) {
		cnss_pr_err("Invalid Bus BW Config Table\n");
		goto cleanup;
	}

	cnss_pr_dbg("ICC Path_Count: %d BW_CFG_Count: %d\n",
		    plat_priv->icc.path_count, plat_priv->icc.bus_bw_cfg_count);

	for (idx = 0; idx < plat_priv->icc.path_count; idx++) {
		bus_bw_info = devm_kzalloc(dev, sizeof(*bus_bw_info),
					   GFP_KERNEL);
		if (!bus_bw_info) {
			ret = -ENOMEM;
			goto out;
		}
		ret = of_property_read_string_index(dev->of_node,
						    "interconnect-names", idx,
						    &bus_bw_info->icc_name);
		if (ret)
			goto out;

		bus_bw_info->icc_path =
			of_icc_get(&plat_priv->plat_dev->dev,
				   bus_bw_info->icc_name);

		if (IS_ERR(bus_bw_info->icc_path))  {
			ret = PTR_ERR(bus_bw_info->icc_path);
			if (ret != -EPROBE_DEFER) {
				cnss_pr_err("Failed to get Interconnect path for %s. Err: %d\n",
					    bus_bw_info->icc_name, ret);
				goto out;
			}
		}

		bus_bw_info->cfg_table =
			devm_kcalloc(dev, plat_priv->icc.bus_bw_cfg_count,
				     sizeof(*bus_bw_info->cfg_table),
				     GFP_KERNEL);
		if (!bus_bw_info->cfg_table) {
			ret = -ENOMEM;
			goto out;
		}
		cnss_pr_dbg("ICC Vote CFG for path: %s\n",
			    bus_bw_info->icc_name);
		for (i = 0, j = (idx * plat_priv->icc.bus_bw_cfg_count *
		     CNSS_ICC_VOTE_MAX);
		     i < plat_priv->icc.bus_bw_cfg_count;
		     i++, j += 2) {
			bus_bw_info->cfg_table[i].avg_bw = cfg_arr[j];
			bus_bw_info->cfg_table[i].peak_bw = cfg_arr[j + 1];
			cnss_pr_dbg("ICC Vote BW: %d avg: %d peak: %d\n",
				    i, bus_bw_info->cfg_table[i].avg_bw,
				    bus_bw_info->cfg_table[i].peak_bw);
		}
		list_add_tail(&bus_bw_info->list,
			      &plat_priv->icc.list_head);
	}
	kfree(cfg_arr);
	return 0;
out:
	list_for_each_entry_safe(bus_bw_info, tmp,
				 &plat_priv->icc.list_head, list) {
		list_del(&bus_bw_info->list);
	}
cleanup:
	kfree(cfg_arr);
	memset(&plat_priv->icc, 0, sizeof(plat_priv->icc));
	return ret;
}

static void cnss_unregister_bus_scale(struct cnss_plat_data *plat_priv)
{
	struct cnss_bus_bw_info *bus_bw_info, *tmp;

	list_for_each_entry_safe(bus_bw_info, tmp,
				 &plat_priv->icc.list_head, list) {
		list_del(&bus_bw_info->list);
		if (bus_bw_info->icc_path)
			icc_put(bus_bw_info->icc_path);
	}
	memset(&plat_priv->icc, 0, sizeof(plat_priv->icc));
}
#else
static int cnss_register_bus_scale(struct cnss_plat_data *plat_priv)
{
	return 0;
}

static void cnss_unregister_bus_scale(struct cnss_plat_data *plat_priv) {}
#endif /* CONFIG_INTERCONNECT */

void cnss_daemon_connection_update_cb(void *cb_ctx, bool status)
{
	struct cnss_plat_data *plat_priv = cb_ctx;

	if (!plat_priv) {
		cnss_pr_err("%s: Invalid context\n", __func__);
		return;
	}
	if (status) {
		cnss_pr_info("CNSS Daemon connected\n");
		set_bit(CNSS_DAEMON_CONNECTED, &plat_priv->driver_state);
		complete(&plat_priv->daemon_connected);
	} else {
		cnss_pr_info("CNSS Daemon disconnected\n");
		reinit_completion(&plat_priv->daemon_connected);
		clear_bit(CNSS_DAEMON_CONNECTED, &plat_priv->driver_state);
	}
}

static ssize_t enable_hds_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	unsigned int enable_hds = 0;

	if (!plat_priv)
		return -ENODEV;

	if (sscanf(buf, "%du", &enable_hds) != 1) {
		cnss_pr_err("Invalid enable_hds sysfs command\n");
		return -EINVAL;
	}

	if (enable_hds)
		plat_priv->hds_enabled = true;
	else
		plat_priv->hds_enabled = false;

	cnss_pr_dbg("%s HDS file download, count is %zu\n",
		    plat_priv->hds_enabled ? "Enable" : "Disable", count);

	return count;
}

static ssize_t recovery_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	u32 buf_size = PAGE_SIZE;
	u32 curr_len = 0;
	u32 buf_written = 0;

	if (!plat_priv)
		return -ENODEV;

	buf_written = scnprintf(buf, buf_size,
				"Usage: echo [recovery_bitmap] > /sys/kernel/cnss/recovery\n"
				"BIT0 -- wlan fw recovery\n"
				"BIT1 -- wlan pcss recovery\n"
				"---------------------------------\n");
	curr_len += buf_written;

	buf_written = scnprintf(buf + curr_len, buf_size - curr_len,
				"WLAN recovery %s[%d]\n",
				plat_priv->recovery_enabled ? "Enabled" : "Disabled",
				plat_priv->recovery_enabled);
	curr_len += buf_written;

	buf_written = scnprintf(buf + curr_len, buf_size - curr_len,
				"WLAN PCSS recovery %s[%d]\n",
				plat_priv->recovery_pcss_enabled ? "Enabled" : "Disabled",
				plat_priv->recovery_pcss_enabled);
	curr_len += buf_written;

	/*
	 * Now size of curr_len is not over page size for sure,
	 * later if new item or none-fixed size item added, need
	 * add check to make sure curr_len is not over page size.
	 */
	return curr_len;
}

static ssize_t time_sync_period_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u ms\n",
			plat_priv->ctrl_params.time_sync_period);
}

/**
 * cnss_get_min_time_sync_period_by_vote() - Get minimum time sync period
 * @plat_priv: Platform data structure
 *
 * Result: return minimum time sync period present in vote from wlan and sys
 */
uint32_t cnss_get_min_time_sync_period_by_vote(struct cnss_plat_data *plat_priv)
{
	unsigned int i, min_time_sync_period = CNSS_TIME_SYNC_PERIOD_INVALID;
	unsigned int time_sync_period;

	for (i = 0; i < TIME_SYNC_VOTE_MAX; i++) {
		time_sync_period = plat_priv->ctrl_params.time_sync_period_vote[i];
		if (min_time_sync_period > time_sync_period)
			min_time_sync_period = time_sync_period;
	}

	return min_time_sync_period;
}

static ssize_t time_sync_period_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	unsigned int time_sync_period = 0;

	if (!plat_priv)
		return -ENODEV;

	if (sscanf(buf, "%du", &time_sync_period) != 1) {
		cnss_pr_err("Invalid time sync sysfs command\n");
		return -EINVAL;
	}

	if (time_sync_period < CNSS_MIN_TIME_SYNC_PERIOD) {
		cnss_pr_err("Invalid time sync value\n");
		return -EINVAL;
	}
	plat_priv->ctrl_params.time_sync_period_vote[TIME_SYNC_VOTE_CNSS] =
		time_sync_period;
	time_sync_period = cnss_get_min_time_sync_period_by_vote(plat_priv);

	if (time_sync_period == CNSS_TIME_SYNC_PERIOD_INVALID) {
		cnss_pr_err("Invalid min time sync value\n");
		return -EINVAL;
	}

	cnss_bus_update_time_sync_period(plat_priv, time_sync_period);

	return count;
}

/**
 * cnss_update_time_sync_period() - Set time sync period given by driver
 * @dev: device structure
 * @time_sync_period: time sync period value
 *
 * Update time sync period vote of driver and set minimum of time sync period
 * from stored vote through wlan and sys config
 * Result: return 0 for success, error in case of invalid value and no dev
 */
int cnss_update_time_sync_period(struct device *dev, uint32_t time_sync_period)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);

	if (!plat_priv)
		return -ENODEV;

	if (time_sync_period < CNSS_MIN_TIME_SYNC_PERIOD) {
		cnss_pr_err("Invalid time sync value\n");
		return -EINVAL;
	}

	plat_priv->ctrl_params.time_sync_period_vote[TIME_SYNC_VOTE_WLAN] =
		time_sync_period;
	time_sync_period = cnss_get_min_time_sync_period_by_vote(plat_priv);

	if (time_sync_period == CNSS_TIME_SYNC_PERIOD_INVALID) {
		cnss_pr_err("Invalid min time sync value\n");
		return -EINVAL;
	}

	cnss_bus_update_time_sync_period(plat_priv, time_sync_period);
	return 0;
}
EXPORT_SYMBOL(cnss_update_time_sync_period);

/**
 * cnss_reset_time_sync_period() - Reset time sync period
 * @dev: device structure
 *
 * Update time sync period vote of driver as invalid
 * and reset minimum of time sync period from
 * stored vote through wlan and sys config
 * Result: return 0 for success, error in case of no dev
 */
int cnss_reset_time_sync_period(struct device *dev)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	unsigned int time_sync_period = 0;

	if (!plat_priv)
		return -ENODEV;

	/* Driver vote is set to invalid in case of reset
	 * In this case, only vote valid to check is sys config
	 */
	plat_priv->ctrl_params.time_sync_period_vote[TIME_SYNC_VOTE_WLAN] =
		CNSS_TIME_SYNC_PERIOD_INVALID;
	time_sync_period = cnss_get_min_time_sync_period_by_vote(plat_priv);

	if (time_sync_period == CNSS_TIME_SYNC_PERIOD_INVALID) {
		cnss_pr_err("Invalid min time sync value\n");
		return -EINVAL;
	}

	cnss_bus_update_time_sync_period(plat_priv, time_sync_period);

	return 0;
}
EXPORT_SYMBOL(cnss_reset_time_sync_period);

static ssize_t recovery_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	unsigned int recovery = 0;

	if (!plat_priv)
		return -ENODEV;

	if (sscanf(buf, "%du", &recovery) != 1) {
		cnss_pr_err("Invalid recovery sysfs command\n");
		return -EINVAL;
	}

	plat_priv->recovery_enabled = !!(recovery & CNSS_WLAN_RECOVERY);
	plat_priv->recovery_pcss_enabled = !!(recovery & CNSS_PCSS_RECOVERY);

	cnss_pr_dbg("%s WLAN recovery, count is %zu\n",
		    plat_priv->recovery_enabled ? "Enable" : "Disable", count);
	cnss_pr_dbg("%s PCSS recovery, count is %zu\n",
		    plat_priv->recovery_pcss_enabled ? "Enable" : "Disable", count);

	cnss_send_subsys_restart_level_msg(plat_priv);
	return count;
}

static ssize_t shutdown_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);

	cnss_pr_dbg("Received shutdown notification\n");
	if (plat_priv) {
		set_bit(CNSS_IN_REBOOT, &plat_priv->driver_state);
		cnss_bus_update_status(plat_priv, CNSS_SYS_REBOOT);
		del_timer(&plat_priv->fw_boot_timer);
		complete_all(&plat_priv->power_up_complete);
		complete_all(&plat_priv->cal_complete);
		cnss_pr_dbg("Shutdown notification handled\n");
	}

	return count;
}

static ssize_t fs_ready_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int fs_ready = 0;
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);

	if (sscanf(buf, "%du", &fs_ready) != 1)
		return -EINVAL;

	cnss_pr_dbg("File system is ready, fs_ready is %d, count is %zu\n",
		    fs_ready, count);

	if (!plat_priv) {
		cnss_pr_err("plat_priv is NULL\n");
		return count;
	}

	if (test_bit(QMI_BYPASS, &plat_priv->ctrl_params.quirks)) {
		cnss_pr_dbg("QMI is bypassed\n");
		return count;
	}

	set_bit(CNSS_FS_READY, &plat_priv->driver_state);
	if (fs_ready == FILE_SYSTEM_READY && plat_priv->cbc_enabled) {
		cnss_driver_event_post(plat_priv,
				       CNSS_DRIVER_EVENT_COLD_BOOT_CAL_START,
				       0, NULL);
	}

	return count;
}

static ssize_t qdss_trace_start_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);

	wlfw_qdss_trace_start(plat_priv);
	cnss_pr_dbg("Received QDSS start command\n");
	return count;
}

static ssize_t qdss_trace_stop_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	u32 option = 0;

	if (sscanf(buf, "%du", &option) != 1)
		return -EINVAL;

	wlfw_qdss_trace_stop(plat_priv, option);
	cnss_pr_dbg("Received QDSS stop command\n");
	return count;
}

static ssize_t qdss_conf_download_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);

	cnss_wlfw_qdss_dnld_send_sync(plat_priv);
	cnss_pr_dbg("Received QDSS download config command\n");
	return count;
}

static ssize_t hw_trace_override_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	int tmp = 0;

	if (sscanf(buf, "%du", &tmp) != 1)
		return -EINVAL;

	plat_priv->hw_trc_override = tmp;
	cnss_pr_dbg("Received QDSS hw_trc_override indication\n");
	return count;
}

static ssize_t charger_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	int tmp = 0;

	if (sscanf(buf, "%du", &tmp) != 1)
		return -EINVAL;

	plat_priv->charger_mode = tmp;
	cnss_pr_dbg("Received Charger Mode: %d\n", tmp);
	return count;
}

static ssize_t crash_reason_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cnss_plat_data *plat_priv = dev_get_drvdata(dev);
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "%s\n", plat_priv->crash_reason_buf);
	memset(plat_priv->crash_reason_buf, 0, sizeof(plat_priv->crash_reason_buf));

	return r;
}

static DEVICE_ATTR_RO(crash_reason);
static DEVICE_ATTR_WO(fs_ready);
static DEVICE_ATTR_WO(shutdown);
static DEVICE_ATTR_RW(recovery);
static DEVICE_ATTR_WO(enable_hds);
static DEVICE_ATTR_WO(qdss_trace_start);
static DEVICE_ATTR_WO(qdss_trace_stop);
static DEVICE_ATTR_WO(qdss_conf_download);
static DEVICE_ATTR_WO(hw_trace_override);
static DEVICE_ATTR_WO(charger_mode);
static DEVICE_ATTR_RW(time_sync_period);

static struct attribute *cnss_attrs[] = {
	&dev_attr_fs_ready.attr,
	&dev_attr_shutdown.attr,
	&dev_attr_recovery.attr,
	&dev_attr_enable_hds.attr,
	&dev_attr_qdss_trace_start.attr,
	&dev_attr_qdss_trace_stop.attr,
	&dev_attr_qdss_conf_download.attr,
	&dev_attr_hw_trace_override.attr,
	&dev_attr_charger_mode.attr,
	&dev_attr_time_sync_period.attr,
	&dev_attr_crash_reason.attr,
	NULL,
};

static struct attribute_group cnss_attr_group = {
	.attrs = cnss_attrs,
};

static int cnss_create_sysfs_link(struct cnss_plat_data *plat_priv)
{
	struct device *dev = &plat_priv->plat_dev->dev;
	int ret;
	char cnss_name[CNSS_FS_NAME_SIZE];
	char shutdown_name[32];

	if (cnss_is_dual_wlan_enabled()) {
		snprintf(cnss_name, CNSS_FS_NAME_SIZE,
			 CNSS_FS_NAME "_%d", plat_priv->plat_idx);
		snprintf(shutdown_name, sizeof(shutdown_name),
			 "shutdown_wlan_%d", plat_priv->plat_idx);
	} else {
		snprintf(cnss_name, CNSS_FS_NAME_SIZE, CNSS_FS_NAME);
		snprintf(shutdown_name, sizeof(shutdown_name),
			 "shutdown_wlan");
	}

	ret = sysfs_create_link(kernel_kobj, &dev->kobj, cnss_name);
	if (ret) {
		cnss_pr_err("Failed to create cnss link, err = %d\n",
			    ret);
		goto out;
	}

	/* This is only for backward compatibility. */
	ret = sysfs_create_link(kernel_kobj, &dev->kobj, shutdown_name);
	if (ret) {
		cnss_pr_err("Failed to create shutdown_wlan link, err = %d\n",
			    ret);
		goto rm_cnss_link;
	}

	return 0;

rm_cnss_link:
	sysfs_remove_link(kernel_kobj, cnss_name);
out:
	return ret;
}

static void cnss_remove_sysfs_link(struct cnss_plat_data *plat_priv)
{
	char cnss_name[CNSS_FS_NAME_SIZE];
	char shutdown_name[32];

	if (cnss_is_dual_wlan_enabled()) {
		snprintf(cnss_name, CNSS_FS_NAME_SIZE,
			 CNSS_FS_NAME "_%d", plat_priv->plat_idx);
		snprintf(shutdown_name, sizeof(shutdown_name),
			 "shutdown_wlan_%d", plat_priv->plat_idx);
	} else {
		snprintf(cnss_name, CNSS_FS_NAME_SIZE, CNSS_FS_NAME);
		snprintf(shutdown_name, sizeof(shutdown_name),
			 "shutdown_wlan");
	}

	sysfs_remove_link(kernel_kobj, shutdown_name);
	sysfs_remove_link(kernel_kobj, cnss_name);
}

static int cnss_create_sysfs(struct cnss_plat_data *plat_priv)
{
	int ret = 0;

	ret = devm_device_add_group(&plat_priv->plat_dev->dev,
				    &cnss_attr_group);
	if (ret) {
		cnss_pr_err("Failed to create cnss device group, err = %d\n",
			    ret);
		goto out;
	}

	cnss_create_sysfs_link(plat_priv);

	return 0;
out:
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0))
union cnss_device_group_devres {
	const struct attribute_group *group;
};

static void devm_cnss_group_remove(struct device *dev, void *res)
{
	union cnss_device_group_devres *devres = res;
	const struct attribute_group *group = devres->group;

	cnss_pr_dbg("%s: removing group %p\n", __func__, group);
	sysfs_remove_group(&dev->kobj, group);
}

static int devm_cnss_group_match(struct device *dev, void *res, void *data)
{
	return ((union cnss_device_group_devres *)res) == data;
}

static void cnss_remove_sysfs(struct cnss_plat_data *plat_priv)
{
	cnss_remove_sysfs_link(plat_priv);
	WARN_ON(devres_release(&plat_priv->plat_dev->dev,
			       devm_cnss_group_remove, devm_cnss_group_match,
			       (void *)&cnss_attr_group));
}
#else
static void cnss_remove_sysfs(struct cnss_plat_data *plat_priv)
{
	cnss_remove_sysfs_link(plat_priv);
	devm_device_remove_group(&plat_priv->plat_dev->dev, &cnss_attr_group);
}
#endif

static int cnss_event_work_init(struct cnss_plat_data *plat_priv)
{
	spin_lock_init(&plat_priv->event_lock);
	plat_priv->event_wq = alloc_workqueue("cnss_driver_event",
					      WQ_UNBOUND, 1);
	if (!plat_priv->event_wq) {
		cnss_pr_err("Failed to create event workqueue!\n");
		return -EFAULT;
	}

	INIT_WORK(&plat_priv->event_work, cnss_driver_event_work);
	INIT_LIST_HEAD(&plat_priv->event_list);

	return 0;
}

static void cnss_event_work_deinit(struct cnss_plat_data *plat_priv)
{
	destroy_workqueue(plat_priv->event_wq);
}

static int cnss_reboot_notifier(struct notifier_block *nb,
				unsigned long action,
				void *data)
{
	struct cnss_plat_data *plat_priv =
		container_of(nb, struct cnss_plat_data, reboot_nb);

	set_bit(CNSS_IN_REBOOT, &plat_priv->driver_state);
	cnss_bus_update_status(plat_priv, CNSS_SYS_REBOOT);
	del_timer(&plat_priv->fw_boot_timer);
	complete_all(&plat_priv->power_up_complete);
	complete_all(&plat_priv->cal_complete);
	cnss_pr_dbg("Reboot is in progress with action %d\n", action);

	return NOTIFY_DONE;
}

#ifdef CONFIG_CNSS_HW_SECURE_DISABLE
#ifdef CONFIG_CNSS_HW_SECURE_SMEM
int cnss_wlan_hw_disable_check(struct cnss_plat_data *plat_priv)
{
	uint32_t *peripheralStateInfo = NULL;
	size_t size = 0;

	/* Once this flag is set, secure peripheral feature
	 * will not be supported till next reboot
	 */
	if (plat_priv->sec_peri_feature_disable)
		return 0;

	peripheralStateInfo = qcom_smem_get(QCOM_SMEM_HOST_ANY, PERISEC_SMEM_ID, &size);
	if (IS_ERR_OR_NULL(peripheralStateInfo)) {
		if (PTR_ERR(peripheralStateInfo) != -ENOENT)
			CNSS_ASSERT(0);

		cnss_pr_dbg("Secure HW feature not enabled. ret = %d\n",
			    PTR_ERR(peripheralStateInfo));
		plat_priv->sec_peri_feature_disable = true;
		return 0;
	}

	cnss_pr_dbg("Secure HW state: %d\n", *peripheralStateInfo);
	if ((*peripheralStateInfo >> (HW_WIFI_UID - 0x500)) & 0x1)
		set_bit(CNSS_WLAN_HW_DISABLED,
			&plat_priv->driver_state);
	else
		clear_bit(CNSS_WLAN_HW_DISABLED,
			  &plat_priv->driver_state);

	return 0;
}
#else
int cnss_wlan_hw_disable_check(struct cnss_plat_data *plat_priv)
{
	struct Object client_env;
	struct Object app_object;
	u32 wifi_uid = HW_WIFI_UID;
	union ObjectArg obj_arg[2] = {{{0, 0}}};
	int ret;
	u8 state = 0;

	/* Once this flag is set, secure peripheral feature
	 * will not be supported till next reboot
	 */
	if (plat_priv->sec_peri_feature_disable)
		return 0;

	/* get rootObj */
	ret = get_client_env_object(&client_env);
	if (ret) {
		cnss_pr_dbg("Failed to get client_env_object, ret: %d\n", ret);
		goto end;
	}
	ret = IClientEnv_open(client_env, HW_STATE_UID, &app_object);
	if (ret) {
		cnss_pr_dbg("Failed to get app_object, ret: %d\n",  ret);
		if (ret == FEATURE_NOT_SUPPORTED) {
			ret = 0; /* Do not Assert */
			plat_priv->sec_peri_feature_disable = true;
			cnss_pr_dbg("Secure HW feature not supported\n");
		}
		goto exit_release_clientenv;
	}

	obj_arg[0].b = (struct ObjectBuf) {&wifi_uid, sizeof(u32)};
	obj_arg[1].b = (struct ObjectBuf) {&state, sizeof(u8)};
	ret = Object_invoke(app_object, HW_OP_GET_STATE, obj_arg,
			    ObjectCounts_pack(1, 1, 0, 0));

	cnss_pr_dbg("SMC invoke ret: %d state: %d\n", ret, state);
	if (ret) {
		if (ret == PERIPHERAL_NOT_FOUND) {
			ret = 0; /* Do not Assert */
			plat_priv->sec_peri_feature_disable = true;
			cnss_pr_dbg("Secure HW mode is not updated. Peripheral not found\n");
		}
		goto exit_release_app_obj;
	}

	if (state == 1)
		set_bit(CNSS_WLAN_HW_DISABLED,
			&plat_priv->driver_state);
	else
		clear_bit(CNSS_WLAN_HW_DISABLED,
			  &plat_priv->driver_state);

exit_release_app_obj:
	Object_release(app_object);
exit_release_clientenv:
	Object_release(client_env);
end:
	if (ret) {
		cnss_pr_err("Unable to get HW disable status\n");
		CNSS_ASSERT(0);
	}
	return ret;
}
#endif
#else
int cnss_wlan_hw_disable_check(struct cnss_plat_data *plat_priv)
{
	return 0;
}
#endif

#ifdef CONFIG_DISABLE_CNSS_SRAM_DUMP
static void cnss_sram_dump_init(struct cnss_plat_data *plat_priv)
{
}
#else
static void cnss_sram_dump_init(struct cnss_plat_data *plat_priv)
{
	if (plat_priv->device_id == QCA6490_DEVICE_ID &&
	    cnss_get_host_build_type() == QMI_HOST_BUILD_TYPE_PRIMARY_V01)
		plat_priv->sram_dump = kcalloc(SRAM_DUMP_SIZE, 1, GFP_KERNEL);
}
#endif

#if IS_ENABLED(CONFIG_WCNSS_MEM_PRE_ALLOC)
static void cnss_initialize_mem_pool(unsigned long device_id)
{
	cnss_initialize_prealloc_pool(device_id);
}
static void cnss_deinitialize_mem_pool(void)
{
	cnss_deinitialize_prealloc_pool();
}
#else
static void cnss_initialize_mem_pool(unsigned long device_id)
{
}
static void cnss_deinitialize_mem_pool(void)
{
}
#endif

static int cnss_misc_init(struct cnss_plat_data *plat_priv)
{
	int ret;

	ret = cnss_init_sol_gpio(plat_priv);
	if (ret)
		return ret;

	timer_setup(&plat_priv->fw_boot_timer,
		    cnss_bus_fw_boot_timeout_hdlr, 0);

	ret = device_init_wakeup(&plat_priv->plat_dev->dev, true);
	if (ret)
		cnss_pr_err("Failed to init platform device wakeup source, err = %d\n",
			    ret);

	INIT_WORK(&plat_priv->recovery_work, cnss_recovery_work_handler);
	init_completion(&plat_priv->power_up_complete);
	init_completion(&plat_priv->cal_complete);
	init_completion(&plat_priv->rddm_complete);
	init_completion(&plat_priv->recovery_complete);
	init_completion(&plat_priv->daemon_connected);
	mutex_init(&plat_priv->dev_lock);
	mutex_init(&plat_priv->driver_ops_lock);

	plat_priv->reboot_nb.notifier_call = cnss_reboot_notifier;
	ret = register_reboot_notifier(&plat_priv->reboot_nb);
	if (ret)
		cnss_pr_err("Failed to register reboot notifier, err = %d\n",
			    ret);

	plat_priv->recovery_ws =
		wakeup_source_register(&plat_priv->plat_dev->dev,
				       "CNSS_FW_RECOVERY");
	if (!plat_priv->recovery_ws)
		cnss_pr_err("Failed to setup FW recovery wake source\n");

	ret = cnss_plat_ipc_register(CNSS_PLAT_IPC_DAEMON_QMI_CLIENT_V01,
				     cnss_daemon_connection_update_cb,
				     plat_priv);
	if (ret)
		cnss_pr_err("QMI IPC connection call back register failed, err = %d\n",
			    ret);

	cnss_sram_dump_init(plat_priv);

	if (of_property_read_bool(plat_priv->plat_dev->dev.of_node,
				  "qcom,rc-ep-short-channel"))
		cnss_set_feature_list(plat_priv, CNSS_RC_EP_ULTRASHORT_CHANNEL_V01);
	if (plat_priv->device_id == PEACH_DEVICE_ID)
		cnss_set_feature_list(plat_priv, CNSS_AUX_UC_SUPPORT_V01);

	return 0;
}

#ifdef CONFIG_DISABLE_CNSS_SRAM_DUMP
static void cnss_sram_dump_deinit(struct cnss_plat_data *plat_priv)
{
}
#else
static void cnss_sram_dump_deinit(struct cnss_plat_data *plat_priv)
{
	if (plat_priv->device_id == QCA6490_DEVICE_ID &&
	    cnss_get_host_build_type() == QMI_HOST_BUILD_TYPE_PRIMARY_V01)
		kfree(plat_priv->sram_dump);
}
#endif

static void cnss_misc_deinit(struct cnss_plat_data *plat_priv)
{
	cnss_plat_ipc_unregister(CNSS_PLAT_IPC_DAEMON_QMI_CLIENT_V01,
				 plat_priv);
	complete_all(&plat_priv->recovery_complete);
	complete_all(&plat_priv->rddm_complete);
	complete_all(&plat_priv->cal_complete);
	complete_all(&plat_priv->power_up_complete);
	complete_all(&plat_priv->daemon_connected);
	device_init_wakeup(&plat_priv->plat_dev->dev, false);
	unregister_reboot_notifier(&plat_priv->reboot_nb);
	del_timer(&plat_priv->fw_boot_timer);
	wakeup_source_unregister(plat_priv->recovery_ws);
	cnss_deinit_sol_gpio(plat_priv);
	cnss_sram_dump_deinit(plat_priv);
	kfree(plat_priv->on_chip_pmic_board_ids);
}

static void cnss_init_time_sync_period_default(struct cnss_plat_data *plat_priv)
{
	plat_priv->ctrl_params.time_sync_period_vote[TIME_SYNC_VOTE_WLAN] =
		CNSS_TIME_SYNC_PERIOD_INVALID;
	plat_priv->ctrl_params.time_sync_period_vote[TIME_SYNC_VOTE_CNSS] =
		CNSS_TIME_SYNC_PERIOD_DEFAULT;
}

static void cnss_init_control_params(struct cnss_plat_data *plat_priv)
{
	plat_priv->ctrl_params.quirks = CNSS_QUIRKS_DEFAULT;

	plat_priv->cbc_enabled = !IS_ENABLED(CONFIG_CNSS_EMULATION) &&
		of_property_read_bool(plat_priv->plat_dev->dev.of_node,
				      "qcom,wlan-cbc-enabled");

	plat_priv->ctrl_params.mhi_timeout = CNSS_MHI_TIMEOUT_DEFAULT;
	plat_priv->ctrl_params.mhi_m2_timeout = CNSS_MHI_M2_TIMEOUT_DEFAULT;
	plat_priv->ctrl_params.qmi_timeout = CNSS_QMI_TIMEOUT_DEFAULT;
	plat_priv->ctrl_params.bdf_type = CNSS_BDF_TYPE_DEFAULT;
	plat_priv->ctrl_params.time_sync_period = CNSS_TIME_SYNC_PERIOD_DEFAULT;
	cnss_init_time_sync_period_default(plat_priv);
	/* Set adsp_pc_enabled default value to true as ADSP pc is always
	 * enabled by default
	 */
	plat_priv->adsp_pc_enabled = true;
}

static void cnss_get_pm_domain_info(struct cnss_plat_data *plat_priv)
{
	struct device *dev = &plat_priv->plat_dev->dev;

	plat_priv->use_pm_domain =
		of_property_read_bool(dev->of_node, "use-pm-domain");

	cnss_pr_dbg("use-pm-domain is %d\n", plat_priv->use_pm_domain);
}

static void cnss_get_wlaon_pwr_ctrl_info(struct cnss_plat_data *plat_priv)
{
	struct device *dev = &plat_priv->plat_dev->dev;

	plat_priv->set_wlaon_pwr_ctrl =
		of_property_read_bool(dev->of_node, "qcom,set-wlaon-pwr-ctrl");

	cnss_pr_dbg("set_wlaon_pwr_ctrl is %d\n",
		    plat_priv->set_wlaon_pwr_ctrl);
}

static bool cnss_use_fw_path_with_prefix(struct cnss_plat_data *plat_priv)
{
	return (of_property_read_bool(plat_priv->plat_dev->dev.of_node,
				      "qcom,converged-dt") ||
		of_property_read_bool(plat_priv->plat_dev->dev.of_node,
				      "qcom,same-dt-multi-dev") ||
		of_property_read_bool(plat_priv->plat_dev->dev.of_node,
				      "qcom,multi-wlan-exchg"));
}

static const struct platform_device_id cnss_platform_id_table[] = {
	{ .name = "qca6174", .driver_data = QCA6174_DEVICE_ID, },
	{ .name = "qca6290", .driver_data = QCA6290_DEVICE_ID, },
	{ .name = "qca6390", .driver_data = QCA6390_DEVICE_ID, },
	{ .name = "qca6490", .driver_data = QCA6490_DEVICE_ID, },
	{ .name = "kiwi", .driver_data = KIWI_DEVICE_ID, },
	{ .name = "mango", .driver_data = MANGO_DEVICE_ID, },
	{ .name = "peach", .driver_data = PEACH_DEVICE_ID, },
	{ .name = "qcaconv", .driver_data = 0, },
	{ },
};

static const struct of_device_id cnss_of_match_table[] = {
	{
		.compatible = "qcom,cnss",
		.data = (void *)&cnss_platform_id_table[0]},
	{
		.compatible = "qcom,cnss-qca6290",
		.data = (void *)&cnss_platform_id_table[1]},
	{
		.compatible = "qcom,cnss-qca6390",
		.data = (void *)&cnss_platform_id_table[2]},
	{
		.compatible = "qcom,cnss-qca6490",
		.data = (void *)&cnss_platform_id_table[3]},
	{
		.compatible = "qcom,cnss-kiwi",
		.data = (void *)&cnss_platform_id_table[4]},
	{
		.compatible = "qcom,cnss-mango",
		.data = (void *)&cnss_platform_id_table[5]},
	{
		.compatible = "qcom,cnss-peach",
		.data = (void *)&cnss_platform_id_table[6]},
	{
		.compatible = "qcom,cnss-qca-converged",
		.data = (void *)&cnss_platform_id_table[7]},
	{ },
};
MODULE_DEVICE_TABLE(of, cnss_of_match_table);

static inline bool
cnss_use_nv_mac(struct cnss_plat_data *plat_priv)
{
	return of_property_read_bool(plat_priv->plat_dev->dev.of_node,
				     "use-nv-mac");
}

static int cnss_get_dev_cfg_node(struct cnss_plat_data *plat_priv)
{
	struct device_node *child;
	u32 id, i;
	int id_n,  device_identifier_gpio, ret;
	u8 gpio_value;


	if (plat_priv->dt_type != CNSS_DTT_CONVERGED)
		return 0;

	/* Parses the wlan_sw_ctrl gpio which is used to identify device */
	ret = cnss_get_wlan_sw_ctrl(plat_priv);
	if (ret) {
		cnss_pr_dbg("Failed to parse wlan_sw_ctrl gpio, error:%d", ret);
		return ret;
	}

	device_identifier_gpio = plat_priv->pinctrl_info.wlan_sw_ctrl_gpio;

	gpio_value = gpio_get_value(device_identifier_gpio);
	cnss_pr_dbg("Value of Device Identifier GPIO: %d\n", gpio_value);

	for_each_available_child_of_node(plat_priv->plat_dev->dev.of_node,
					 child) {
		if (strcmp(child->name, "chip_cfg"))
			continue;

		id_n = of_property_count_u32_elems(child, "supported-ids");
		if (id_n <= 0) {
			cnss_pr_err("Device id is NOT set\n");
			return -EINVAL;
		}

		for (i = 0; i < id_n; i++) {
			ret = of_property_read_u32_index(child,
							 "supported-ids",
							 i, &id);
			if (ret) {
				cnss_pr_err("Failed to read supported ids\n");
				return -EINVAL;
			}

			if (gpio_value && id == QCA6490_DEVICE_ID) {
				plat_priv->plat_dev->dev.of_node = child;
				plat_priv->device_id = QCA6490_DEVICE_ID;
				cnss_utils_update_device_type(CNSS_HSP_DEVICE_TYPE);
				cnss_pr_dbg("got node[%s@%d] for device[0x%x]\n",
					    child->name, i, id);
				return 0;
			} else if (!gpio_value && id == KIWI_DEVICE_ID) {
				plat_priv->plat_dev->dev.of_node = child;
				plat_priv->device_id = KIWI_DEVICE_ID;
				cnss_utils_update_device_type(CNSS_HMT_DEVICE_TYPE);
				cnss_pr_dbg("got node[%s@%d] for device[0x%x]\n",
					    child->name, i, id);
				return 0;
			}
		}
	}

	return -EINVAL;
}

static inline u32
cnss_dt_type(struct cnss_plat_data *plat_priv)
{
	bool is_converged_dt = of_property_read_bool(
		plat_priv->plat_dev->dev.of_node, "qcom,converged-dt");
	bool is_multi_wlan_xchg;

	if (is_converged_dt)
		return CNSS_DTT_CONVERGED;

	is_multi_wlan_xchg = of_property_read_bool(
		plat_priv->plat_dev->dev.of_node, "qcom,multi-wlan-exchg");

	if (is_multi_wlan_xchg)
		return CNSS_DTT_MULTIEXCHG;
	return CNSS_DTT_LEGACY;
}

static int cnss_wlan_device_init(struct cnss_plat_data *plat_priv)
{
	int ret = 0;
	int retry = 0;

	if (test_bit(SKIP_DEVICE_BOOT, &plat_priv->ctrl_params.quirks))
		return 0;

retry:
	ret = cnss_power_on_device(plat_priv, true);
	if (ret)
		goto end;

	ret = cnss_bus_init(plat_priv);
	if (ret) {
		if ((ret != -EPROBE_DEFER) &&
		    retry++ < POWER_ON_RETRY_MAX_TIMES) {
			cnss_power_off_device(plat_priv);
			cnss_pr_dbg("Retry cnss_bus_init #%d\n", retry);
			msleep(POWER_ON_RETRY_DELAY_MS * retry);
			goto retry;
		}
		goto power_off;
	}
	return 0;

power_off:
	cnss_power_off_device(plat_priv);
end:
	return ret;
}

int cnss_wlan_hw_enable(void)
{
	struct cnss_plat_data *plat_priv;
	int ret = 0;

	if (cnss_is_dual_wlan_enabled())
		plat_priv = cnss_get_first_plat_priv(NULL);
	else
		plat_priv = cnss_get_plat_priv(NULL);

	if (!plat_priv)
		return -ENODEV;

	clear_bit(CNSS_WLAN_HW_DISABLED, &plat_priv->driver_state);

	if (test_bit(CNSS_PCI_PROBE_DONE, &plat_priv->driver_state))
		goto register_driver;
	ret = cnss_wlan_device_init(plat_priv);
	if (ret) {
		if (!test_bit(CNSS_WLAN_HW_DISABLED, &plat_priv->driver_state))
			CNSS_ASSERT(0);
		return ret;
	}

	if (test_bit(CNSS_FS_READY, &plat_priv->driver_state))
		cnss_driver_event_post(plat_priv,
				       CNSS_DRIVER_EVENT_COLD_BOOT_CAL_START,
				       0, NULL);

register_driver:
	if (plat_priv->driver_ops)
		ret = cnss_wlan_register_driver(plat_priv->driver_ops);

	return ret;
}
EXPORT_SYMBOL(cnss_wlan_hw_enable);

int cnss_set_wfc_mode(struct device *dev, struct cnss_wfc_cfg cfg)
{
	struct cnss_plat_data *plat_priv = cnss_bus_dev_to_plat_priv(dev);
	int ret = 0;

	if (!plat_priv)
		return -ENODEV;

	/* If IMS server is connected, return success without QMI send */
	if (test_bit(CNSS_IMS_CONNECTED, &plat_priv->driver_state)) {
		cnss_pr_dbg("Ignore host request as IMS server is connected");
		return ret;
	}

	ret = cnss_wlfw_send_host_wfc_call_status(plat_priv, cfg);

	return ret;
}
EXPORT_SYMBOL(cnss_set_wfc_mode);

static int cnss_tcdev_get_max_state(struct thermal_cooling_device *tcdev,
				    unsigned long *thermal_state)
{
	struct cnss_thermal_cdev *cnss_tcdev = NULL;

	if (!tcdev || !tcdev->devdata) {
		cnss_pr_err("tcdev or tcdev->devdata is null!\n");
		return -EINVAL;
	}

	cnss_tcdev = tcdev->devdata;
	*thermal_state = cnss_tcdev->max_thermal_state;

	return 0;
}

static int cnss_tcdev_get_cur_state(struct thermal_cooling_device *tcdev,
				    unsigned long *thermal_state)
{
	struct cnss_thermal_cdev *cnss_tcdev = NULL;

	if (!tcdev || !tcdev->devdata) {
		cnss_pr_err("tcdev or tcdev->devdata is null!\n");
		return -EINVAL;
	}

	cnss_tcdev = tcdev->devdata;
	*thermal_state = cnss_tcdev->curr_thermal_state;

	return 0;
}

static int cnss_tcdev_set_cur_state(struct thermal_cooling_device *tcdev,
				    unsigned long thermal_state)
{
	struct cnss_thermal_cdev *cnss_tcdev = NULL;
	struct cnss_plat_data *plat_priv =  cnss_get_plat_priv(NULL);
	int ret = 0;

	if (!tcdev || !tcdev->devdata) {
		cnss_pr_err("tcdev or tcdev->devdata is null!\n");
		return -EINVAL;
	}

	cnss_tcdev = tcdev->devdata;

	if (thermal_state > cnss_tcdev->max_thermal_state)
		return -EINVAL;

	cnss_pr_vdbg("Cooling device set current state: %ld,for cdev id %d",
		     thermal_state, cnss_tcdev->tcdev_id);

	mutex_lock(&plat_priv->tcdev_lock);
	ret = cnss_bus_set_therm_cdev_state(plat_priv,
					    thermal_state,
					    cnss_tcdev->tcdev_id);
	if (!ret)
		cnss_tcdev->curr_thermal_state = thermal_state;
	mutex_unlock(&plat_priv->tcdev_lock);
	if (ret) {
		cnss_pr_err("Setting Current Thermal State Failed: %d,for cdev id %d",
			    ret, cnss_tcdev->tcdev_id);
		return ret;
	}

	return 0;
}

static struct thermal_cooling_device_ops cnss_cooling_ops = {
	.get_max_state = cnss_tcdev_get_max_state,
	.get_cur_state = cnss_tcdev_get_cur_state,
	.set_cur_state = cnss_tcdev_set_cur_state,
};

int cnss_thermal_cdev_register(struct device *dev, unsigned long max_state,
			       int tcdev_id)
{
	struct cnss_plat_data *priv = cnss_get_plat_priv(NULL);
	struct cnss_thermal_cdev *cnss_tcdev = NULL;
	char cdev_node_name[THERMAL_NAME_LENGTH] = "";
	struct device_node *dev_node;
	int ret = 0;

	if (!priv) {
		cnss_pr_err("Platform driver is not initialized!\n");
		return -ENODEV;
	}

	cnss_tcdev = kzalloc(sizeof(*cnss_tcdev), GFP_KERNEL);
	if (!cnss_tcdev) {
		cnss_pr_err("Failed to allocate cnss_tcdev object!\n");
		return -ENOMEM;
	}

	cnss_tcdev->tcdev_id = tcdev_id;
	cnss_tcdev->max_thermal_state = max_state;

	snprintf(cdev_node_name, THERMAL_NAME_LENGTH,
		 "qcom,cnss_cdev%d", tcdev_id);

	dev_node = of_find_node_by_name(NULL, cdev_node_name);
	if (!dev_node) {
		cnss_pr_err("Failed to get cooling device node\n");
		kfree(cnss_tcdev);
		return -EINVAL;
	}

	cnss_pr_dbg("tcdev node->name=%s\n", dev_node->name);

	if (of_find_property(dev_node, "#cooling-cells", NULL)) {
		cnss_tcdev->tcdev = thermal_of_cooling_device_register(dev_node,
								       cdev_node_name,
								       cnss_tcdev,
								       &cnss_cooling_ops);
		if (IS_ERR_OR_NULL(cnss_tcdev->tcdev)) {
			ret = PTR_ERR(cnss_tcdev->tcdev);
			cnss_pr_err("Cooling device register failed: %d, for cdev id %d\n",
				    ret, cnss_tcdev->tcdev_id);
			kfree(cnss_tcdev);
		} else {
			cnss_pr_dbg("Cooling device registered for cdev id %d",
				    cnss_tcdev->tcdev_id);
			mutex_lock(&priv->tcdev_lock);
			list_add(&cnss_tcdev->tcdev_list,
				 &priv->cnss_tcdev_list);
			mutex_unlock(&priv->tcdev_lock);
		}
	} else {
		cnss_pr_dbg("Cooling device registration not supported");
		kfree(cnss_tcdev);
		ret = -EOPNOTSUPP;
	}

	return ret;
}
EXPORT_SYMBOL(cnss_thermal_cdev_register);

void cnss_thermal_cdev_unregister(struct device *dev, int tcdev_id)
{
	struct cnss_plat_data *priv = cnss_get_plat_priv(NULL);
	struct cnss_thermal_cdev *cnss_tcdev = NULL;

	if (!priv) {
		cnss_pr_err("Platform driver is not initialized!\n");
		return;
	}

	mutex_lock(&priv->tcdev_lock);
	while (!list_empty(&priv->cnss_tcdev_list)) {
		cnss_tcdev = list_first_entry(&priv->cnss_tcdev_list,
					      struct cnss_thermal_cdev,
					      tcdev_list);
		thermal_cooling_device_unregister(cnss_tcdev->tcdev);
		list_del(&cnss_tcdev->tcdev_list);
		kfree(cnss_tcdev);
	}
	mutex_unlock(&priv->tcdev_lock);
}
EXPORT_SYMBOL(cnss_thermal_cdev_unregister);

int cnss_get_curr_therm_cdev_state(struct device *dev,
				   unsigned long *thermal_state,
				   int tcdev_id)
{
	struct cnss_plat_data *priv = cnss_get_plat_priv(NULL);
	struct cnss_thermal_cdev *cnss_tcdev = NULL;

	if (!priv) {
		cnss_pr_err("Platform driver is not initialized!\n");
		return -ENODEV;
	}

	mutex_lock(&priv->tcdev_lock);
	list_for_each_entry(cnss_tcdev, &priv->cnss_tcdev_list, tcdev_list) {
		if (cnss_tcdev->tcdev_id != tcdev_id)
			continue;

		*thermal_state = cnss_tcdev->curr_thermal_state;
		mutex_unlock(&priv->tcdev_lock);
		cnss_pr_dbg("Cooling device current state: %ld, for cdev id %d",
			    cnss_tcdev->curr_thermal_state, tcdev_id);
		return 0;
	}
	mutex_unlock(&priv->tcdev_lock);
	cnss_pr_dbg("Cooling device ID not found: %d", tcdev_id);
	return -EINVAL;
}
EXPORT_SYMBOL(cnss_get_curr_therm_cdev_state);

static int cnss_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct cnss_plat_data *plat_priv;
	const struct of_device_id *of_id;
	const struct platform_device_id *device_id;

	if (cnss_get_plat_priv(plat_dev)) {
		cnss_pr_err("Driver is already initialized!\n");
		ret = -EEXIST;
		goto out;
	}

	ret = cnss_plat_env_available();
	if (ret)
		goto out;

	of_id = of_match_device(cnss_of_match_table, &plat_dev->dev);
	if (!of_id || !of_id->data) {
		cnss_pr_err("Failed to find of match device!\n");
		ret = -ENODEV;
		goto out;
	}

	device_id = of_id->data;

	plat_priv = devm_kzalloc(&plat_dev->dev, sizeof(*plat_priv),
				 GFP_KERNEL);
	if (!plat_priv) {
		ret = -ENOMEM;
		goto out;
	}

	plat_priv->plat_dev = plat_dev;
	plat_priv->dev_node = NULL;
	plat_priv->device_id = device_id->driver_data;
	plat_priv->dt_type = cnss_dt_type(plat_priv);
	cnss_pr_dbg("Probing platform driver from dt type: %d\n",
		    plat_priv->dt_type);

	plat_priv->use_fw_path_with_prefix =
		cnss_use_fw_path_with_prefix(plat_priv);

	ret = cnss_get_dev_cfg_node(plat_priv);
	if (ret) {
		cnss_pr_err("Failed to get device cfg node, err = %d\n", ret);
		goto reset_plat_dev;
	}

	cnss_initialize_mem_pool(plat_priv->device_id);

	ret = cnss_get_pld_bus_ops_name(plat_priv);
	if (ret)
		cnss_pr_vdbg("Failed to find bus ops name, err = %d\n",
			     ret);

	ret = cnss_get_rc_num(plat_priv);

	if (ret)
		cnss_pr_err("Failed to find PCIe RC number, err = %d\n", ret);

	cnss_pr_dbg("rc_num=%d\n", plat_priv->rc_num);

	plat_priv->bus_type = cnss_get_bus_type(plat_priv);
	plat_priv->use_nv_mac = cnss_use_nv_mac(plat_priv);
	plat_priv->driver_mode = CNSS_DRIVER_MODE_MAX;
	cnss_set_plat_priv(plat_dev, plat_priv);
	cnss_set_device_name(plat_priv);
	platform_set_drvdata(plat_dev, plat_priv);
	INIT_LIST_HEAD(&plat_priv->vreg_list);
	INIT_LIST_HEAD(&plat_priv->clk_list);

	cnss_get_pm_domain_info(plat_priv);
	cnss_get_wlaon_pwr_ctrl_info(plat_priv);
	cnss_power_misc_params_init(plat_priv);
	cnss_get_tcs_info(plat_priv);
	cnss_get_cpr_info(plat_priv);
	cnss_aop_interface_init(plat_priv);
	cnss_init_control_params(plat_priv);

	ret = cnss_get_resources(plat_priv);
	if (ret)
		goto reset_ctx;

	ret = cnss_register_esoc(plat_priv);
	if (ret)
		goto free_res;

	ret = cnss_register_bus_scale(plat_priv);
	if (ret)
		goto unreg_esoc;

	ret = cnss_create_sysfs(plat_priv);
	if (ret)
		goto unreg_bus_scale;

	ret = cnss_event_work_init(plat_priv);
	if (ret)
		goto remove_sysfs;

	ret = cnss_dms_init(plat_priv);
	if (ret)
		goto deinit_event_work;

	ret = cnss_debugfs_create(plat_priv);
	if (ret)
		goto deinit_dms;

	ret = cnss_misc_init(plat_priv);
	if (ret)
		goto destroy_debugfs;

	ret = cnss_wlan_hw_disable_check(plat_priv);
	if (ret)
		goto deinit_misc;

	/* Make sure all platform related init are done before
	 * device power on and bus init.
	 */
	if (!test_bit(CNSS_WLAN_HW_DISABLED, &plat_priv->driver_state)) {
		ret = cnss_wlan_device_init(plat_priv);
		if (ret)
			goto deinit_misc;
	} else {
		cnss_pr_info("WLAN HW Disabled. Defer PCI enumeration\n");
	}
	cnss_register_coex_service(plat_priv);
	cnss_register_ims_service(plat_priv);

	mutex_init(&plat_priv->tcdev_lock);
	INIT_LIST_HEAD(&plat_priv->cnss_tcdev_list);

	cnss_pr_info("Platform driver probed successfully.\n");

	return 0;

deinit_misc:
	cnss_misc_deinit(plat_priv);
destroy_debugfs:
	cnss_debugfs_destroy(plat_priv);
deinit_dms:
	cnss_dms_deinit(plat_priv);
deinit_event_work:
	cnss_event_work_deinit(plat_priv);
remove_sysfs:
	cnss_remove_sysfs(plat_priv);
unreg_bus_scale:
	cnss_unregister_bus_scale(plat_priv);
unreg_esoc:
	cnss_unregister_esoc(plat_priv);
free_res:
	cnss_put_resources(plat_priv);
reset_ctx:
	cnss_aop_interface_deinit(plat_priv);
	platform_set_drvdata(plat_dev, NULL);
	cnss_deinitialize_mem_pool();
reset_plat_dev:
	cnss_clear_plat_priv(plat_priv);
out:
	return ret;
}

static int cnss_remove(struct platform_device *plat_dev)
{
	struct cnss_plat_data *plat_priv = platform_get_drvdata(plat_dev);

	plat_priv->audio_iommu_domain = NULL;
	cnss_genl_exit();
	cnss_unregister_ims_service(plat_priv);
	cnss_unregister_coex_service(plat_priv);
	cnss_bus_deinit(plat_priv);
	cnss_misc_deinit(plat_priv);
	cnss_debugfs_destroy(plat_priv);
	cnss_dms_deinit(plat_priv);
	cnss_qmi_deinit(plat_priv);
	cnss_event_work_deinit(plat_priv);
	cnss_cancel_dms_work();
	cnss_remove_sysfs(plat_priv);
	cnss_unregister_bus_scale(plat_priv);
	cnss_unregister_esoc(plat_priv);
	cnss_put_resources(plat_priv);
	cnss_aop_interface_deinit(plat_priv);
	cnss_deinitialize_mem_pool();
	platform_set_drvdata(plat_dev, NULL);
	cnss_clear_plat_priv(plat_priv);

	return 0;
}

static struct platform_driver cnss_platform_driver = {
	.probe  = cnss_probe,
	.remove = cnss_remove,
	.driver = {
		.name = "cnss2",
		.of_match_table = cnss_of_match_table,
#ifdef CONFIG_CNSS_ASYNC
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#endif
	},
};

static bool cnss_check_compatible_node(void)
{
	struct device_node *dn = NULL;

	for_each_matching_node(dn, cnss_of_match_table) {
		if (of_device_is_available(dn)) {
			cnss_allow_driver_loading = true;
			return true;
		}
	}

	return false;
}

/**
 * cnss_is_valid_dt_node_found - Check if valid device tree node present
 *
 * Valid device tree node means a node with "compatible" property from the
 * device match table and "status" property is not disabled.
 *
 * Return: true if valid device tree node found, false if not found
 */
static bool cnss_is_valid_dt_node_found(void)
{
	struct device_node *dn = NULL;

	for_each_matching_node(dn, cnss_of_match_table) {
		if (of_device_is_available(dn))
			break;
	}

	if (dn)
		return true;

	return false;
}

static int __init cnss_initialize(void)
{
	int ret = 0;

	if (!cnss_is_valid_dt_node_found())
		return -ENODEV;

	if (!cnss_check_compatible_node())
		return ret;

	cnss_debug_init();
	ret = platform_driver_register(&cnss_platform_driver);
	if (ret)
		cnss_debug_deinit();

	ret = cnss_genl_init();
	if (ret < 0)
		cnss_pr_err("CNSS genl init failed %d\n", ret);

	return ret;
}

static void __exit cnss_exit(void)
{
	cnss_genl_exit();
	platform_driver_unregister(&cnss_platform_driver);
	cnss_debug_deinit();
}

module_init(cnss_initialize);
module_exit(cnss_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CNSS2 Platform Driver");
