// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of_platform.h>
#include "hw_fence_drv_priv.h"
#include "hw_fence_drv_utils.h"
#include "hw_fence_drv_ipc.h"
#include "hw_fence_drv_debug.h"

/*
 * Max size of base table with ipc mappings, with one mapping per client type with configurable
 * number of subclients
 */
#define HW_FENCE_IPC_MAP_MAX (HW_FENCE_MAX_STATIC_CLIENTS_INDEX + \
	HW_FENCE_MAX_CLIENT_TYPE_CONFIGURABLE)

/**
 * struct hw_fence_client_ipc_map - map client id with ipc signal for trigger.
 * @ipc_client_id_virt: virtual ipc client id for the hw-fence client.
 * @ipc_client_id_phys: physical ipc client id for the hw-fence client.
 * @ipc_signal_id: ipc signal id for the hw-fence client.
 * @update_rxq: bool to indicate if clinet uses rx-queue.
 * @send_ipc: bool to indicate if client requires ipc interrupt for signaled fences
 */
struct hw_fence_client_ipc_map {
	int ipc_client_id_virt;
	int ipc_client_id_phys;
	int ipc_signal_id;
	bool update_rxq;
	bool send_ipc;
};

/**
 * struct hw_fence_clients_ipc_map - Table makes the 'client to signal' mapping, which is
 *		used by the hw fence driver to trigger ipc signal when hw fence is already
 *		signaled.
 *		This version is for targets that support dpu client id.
 *
 * Note that the index of this struct must match the enum hw_fence_client_id
 */
struct hw_fence_client_ipc_map hw_fence_clients_ipc_map[HW_FENCE_IPC_MAP_MAX] = {
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 1, true, true},/*ctrl q*/
	{HW_FENCE_IPC_CLIENT_ID_GPU_VID,  HW_FENCE_IPC_CLIENT_ID_GPU_VID, 0, false, false},/*ctx0 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 0, false, true},/* ctl0 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 1, false, true},/* ctl1 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 2, false, true},/* ctl2 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 3, false, true},/* ctl3 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 4, false, true},/* ctl4 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 5, false, true},/* ctl5 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 21, true, false},/*val0*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 22, true, false},/*val1*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 23, true, false},/*val2*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 24, true, false},/*val3*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 25, true, false},/*val4*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 26, true, false},/*val5*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 27, true, false},/*val6*/
#else
	{0, 0, 0, false, false}, /* val0 */
	{0, 0, 0, false, false}, /* val1 */
	{0, 0, 0, false, false}, /* val2 */
	{0, 0, 0, false, false}, /* val3 */
	{0, 0, 0, false, false}, /* val4 */
	{0, 0, 0, false, false}, /* val5 */
	{0, 0, 0, false, false}, /* val6 */
#endif /* CONFIG_DEBUG_FS */
	{HW_FENCE_IPC_CLIENT_ID_IPE_VID, HW_FENCE_IPC_CLIENT_ID_IPE_VID, 0, true, true}, /* ipe */
	{HW_FENCE_IPC_CLIENT_ID_VPU_VID, HW_FENCE_IPC_CLIENT_ID_VPU_VID, 0, true, true}, /* vpu */
};

/**
 * struct hw_fence_clients_ipc_map_v2 - Table makes the 'client to signal' mapping, which is
 *		used by the hw fence driver to trigger ipc signal when hw fence is already
 *		signaled.
 *		This version is for targets that support dpu client id and IPC v2.
 *
 * Note that the index of this struct must match the enum hw_fence_client_id for clients ids less
 * than HW_FENCE_MAX_STATIC_CLIENTS_INDEX.
 * For clients with configurable sub-clients, the index of this struct matches
 * HW_FENCE_MAX_STATIC_CLIENTS_INDEX + (client type index - HW_FENCE_MAX_CLIENT_TYPE_STATIC).
 */
struct hw_fence_client_ipc_map hw_fence_clients_ipc_map_v2[HW_FENCE_IPC_MAP_MAX] = {
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 1, true, true},/*ctrlq */
	{HW_FENCE_IPC_CLIENT_ID_GPU_VID,  HW_FENCE_IPC_CLIENT_ID_GPU_PID, 0, false, false},/* ctx0*/
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 0, false, true},/* ctl0 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 1, false, true},/* ctl1 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 2, false, true},/* ctl2 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 3, false, true},/* ctl3 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 4, false, true},/* ctl4 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 5, false, true},/* ctl5 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 21, true, false},/*val0*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 22, true, false},/*val1*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 23, true, false},/*val2*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 24, true, false},/*val3*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 25, true, false},/*val4*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 26, true, false},/*val5*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 27, true, false},/*val6*/
#else
	{0, 0, 0, false, false}, /* val0 */
	{0, 0, 0, false, false}, /* val1 */
	{0, 0, 0, false, false}, /* val2 */
	{0, 0, 0, false, false}, /* val3 */
	{0, 0, 0, false, false}, /* val4 */
	{0, 0, 0, false, false}, /* val5 */
	{0, 0, 0, false, false}, /* val6 */
#endif /* CONFIG_DEBUG_FS */
	{HW_FENCE_IPC_CLIENT_ID_IPE_VID, HW_FENCE_IPC_CLIENT_ID_IPE_PID, 0, true, true}, /* ipe */
	{HW_FENCE_IPC_CLIENT_ID_VPU_VID, HW_FENCE_IPC_CLIENT_ID_VPU_PID, 0, true, true}, /* vpu */
	{HW_FENCE_IPC_CLIENT_ID_IFE0_VID, HW_FENCE_IPC_CLIENT_ID_IFE0_PID, 0, false, true},/* ife0*/
	{HW_FENCE_IPC_CLIENT_ID_IFE1_VID, HW_FENCE_IPC_CLIENT_ID_IFE1_PID, 0, false, true},/* ife1*/
	{HW_FENCE_IPC_CLIENT_ID_IFE2_VID, HW_FENCE_IPC_CLIENT_ID_IFE2_PID, 0, false, true},/* ife2*/
	{HW_FENCE_IPC_CLIENT_ID_IFE3_VID, HW_FENCE_IPC_CLIENT_ID_IFE3_PID, 0, false, true},/* ife3*/
	{HW_FENCE_IPC_CLIENT_ID_IFE4_VID, HW_FENCE_IPC_CLIENT_ID_IFE4_PID, 0, false, true},/* ife4*/
	{HW_FENCE_IPC_CLIENT_ID_IFE5_VID, HW_FENCE_IPC_CLIENT_ID_IFE5_PID, 0, false, true},/* ife5*/
	{HW_FENCE_IPC_CLIENT_ID_IFE6_VID, HW_FENCE_IPC_CLIENT_ID_IFE6_PID, 0, false, true},/* ife6*/
	{HW_FENCE_IPC_CLIENT_ID_IFE7_VID, HW_FENCE_IPC_CLIENT_ID_IFE7_PID, 0, false, true},/* ife7*/
};

int hw_fence_ipcc_get_client_virt_id(struct hw_fence_driver_data *drv_data, u32 client_id)
{
	if (!drv_data || client_id >= drv_data->clients_num)
		return -EINVAL;

	return drv_data->ipc_clients_table[client_id].ipc_client_id_virt;
}

int hw_fence_ipcc_get_client_phys_id(struct hw_fence_driver_data *drv_data, u32 client_id)
{
	if (!drv_data || client_id >= drv_data->clients_num)
		return -EINVAL;

	return drv_data->ipc_clients_table[client_id].ipc_client_id_phys;
}

int hw_fence_ipcc_get_signal_id(struct hw_fence_driver_data *drv_data, u32 client_id)
{
	if (!drv_data || client_id >= drv_data->clients_num)
		return -EINVAL;

	return drv_data->ipc_clients_table[client_id].ipc_signal_id;
}

bool hw_fence_ipcc_needs_rxq_update(struct hw_fence_driver_data *drv_data, int client_id)
{
	if (!drv_data || client_id >= drv_data->clients_num)
		return false;

	return drv_data->ipc_clients_table[client_id].update_rxq;
}

bool hw_fence_ipcc_needs_ipc_irq(struct hw_fence_driver_data *drv_data, int client_id)
{
	if (!drv_data || client_id >= HW_FENCE_CLIENT_MAX)
		return false;

	return drv_data->ipc_clients_table[client_id].send_ipc;
}

/**
 * _get_ipc_phys_client_name() - Returns ipc client name from its physical id, used for debugging.
 */
static inline char *_get_ipc_phys_client_name(u32 client_id)
{
	switch (client_id) {
	case HW_FENCE_IPC_CLIENT_ID_APPS_PID:
		return "APPS_PID";
	case HW_FENCE_IPC_CLIENT_ID_GPU_PID:
		return "GPU_PID";
	case HW_FENCE_IPC_CLIENT_ID_DPU_PID:
		return "DPU_PID";
	case HW_FENCE_IPC_CLIENT_ID_IPE_PID:
		return "IPE_PID";
	case HW_FENCE_IPC_CLIENT_ID_VPU_PID:
		return "VPU_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE0_PID:
		return "IFE0_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE1_PID:
		return "IFE1_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE2_PID:
		return "IFE2_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE3_PID:
		return "IFE3_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE4_PID:
		return "IFE4_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE5_PID:
		return "IFE5_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE6_PID:
		return "IFE6_PID";
	case HW_FENCE_IPC_CLIENT_ID_IFE7_PID:
		return "IFE7_PID";
	}

	return "UNKNOWN_PID";
}

/**
 * _get_ipc_virt_client_name() - Returns ipc client name from its virtual id, used for debugging.
 */
static inline char *_get_ipc_virt_client_name(u32 client_id)
{
	switch (client_id) {
	case HW_FENCE_IPC_CLIENT_ID_APPS_VID:
		return "APPS_VID";
	case HW_FENCE_IPC_CLIENT_ID_GPU_VID:
		return "GPU_VID";
	case HW_FENCE_IPC_CLIENT_ID_DPU_VID:
		return "DPU_VID";
	case HW_FENCE_IPC_CLIENT_ID_IPE_VID:
		return "IPE_VID";
	case HW_FENCE_IPC_CLIENT_ID_VPU_VID:
		return "VPU_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE0_VID:
		return "IFE0_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE1_VID:
		return "IFE1_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE2_VID:
		return "IFE2_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE3_VID:
		return "IFE3_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE4_VID:
		return "IFE4_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE5_VID:
		return "IFE5_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE6_VID:
		return "IFE6_VID";
	case HW_FENCE_IPC_CLIENT_ID_IFE7_VID:
		return "IFE7_VID";
	}

	return "UNKNOWN_VID";
}

void hw_fence_ipcc_trigger_signal(struct hw_fence_driver_data *drv_data,
	u32 tx_client_pid, u32 rx_client_vid, u32 signal_id)
{
	void __iomem *ptr;
	u32 val;

	/* Send signal */
	ptr = IPC_PROTOCOLp_CLIENTc_SEND(drv_data->ipcc_io_mem, drv_data->protocol_id,
		tx_client_pid);
	val = (rx_client_vid << 16) | signal_id;

	HWFNC_DBG_IRQ("Sending ipcc from %s (%d) to %s (%d) signal_id:%d [wr:0x%x to off:0x%pK]\n",
		_get_ipc_phys_client_name(tx_client_pid), tx_client_pid,
		_get_ipc_virt_client_name(rx_client_vid), rx_client_vid,
		signal_id, val, ptr);
	HWFNC_DBG_H("Write:0x%x to RegOffset:0x%pK\n", val, ptr);
	writel_relaxed(val, ptr);

	/* Make sure value is written */
	wmb();
}

static int _hw_fence_ipcc_init_map_with_configurable_clients(struct hw_fence_driver_data *drv_data,
	struct hw_fence_client_ipc_map *base_table)
{
	int i, j, map_idx;
	size_t size;

	size = drv_data->clients_num * sizeof(struct hw_fence_client_ipc_map);
	drv_data->ipc_clients_table = kzalloc(size, GFP_KERNEL);

	if (!drv_data->ipc_clients_table)
		return -ENOMEM;

	/* copy mappings for static hw fence clients */
	size = HW_FENCE_MAX_STATIC_CLIENTS_INDEX * sizeof(struct hw_fence_client_ipc_map);
	memcpy(drv_data->ipc_clients_table, base_table, size);

	/* initialize mappings for ipc clients with configurable number of hw fence clients */
	map_idx = HW_FENCE_MAX_STATIC_CLIENTS_INDEX;
	for (i = 0; i < HW_FENCE_MAX_CLIENT_TYPE_CONFIGURABLE; i++) {
		int client_type = HW_FENCE_MAX_CLIENT_TYPE_STATIC + i;
		int clients_num = drv_data->hw_fence_client_types[client_type].clients_num;

		for (j = 0; j < clients_num; j++) {
			/* this should never happen if drv_data->clients_num is correct */
			if (map_idx >= drv_data->clients_num) {
				HWFNC_ERR("%s clients_num:%lu exceeds drv_data->clients_num:%lu\n",
					drv_data->hw_fence_client_types[client_type].name,
					clients_num, drv_data->clients_num);
				return -EINVAL;
			}
			drv_data->ipc_clients_table[map_idx] =
				base_table[HW_FENCE_MAX_STATIC_CLIENTS_INDEX + i];
			drv_data->ipc_clients_table[map_idx].ipc_signal_id = j;
			map_idx++;
		}
	}

	return 0;
}

/**
 * _hw_fence_ipcc_hwrev_init() - Initializes internal driver struct with corresponding ipcc data,
 *		according to the ipcc hw revision.
 * @drv_data: driver data.
 * @hwrev: ipcc hw revision.
 */
static int _hw_fence_ipcc_hwrev_init(struct hw_fence_driver_data *drv_data, u32 hwrev)
{
	int ret = 0;

	switch (hwrev) {
	case HW_FENCE_IPCC_HW_REV_170:
		drv_data->ipcc_client_vid = HW_FENCE_IPC_CLIENT_ID_APPS_VID;
		drv_data->ipcc_client_pid = HW_FENCE_IPC_CLIENT_ID_APPS_VID;
		drv_data->protocol_id = HW_FENCE_IPC_COMPUTE_L1_PROTOCOL_ID_KALAMA;
		drv_data->ipc_clients_table = hw_fence_clients_ipc_map;
		HWFNC_DBG_INIT("ipcc protocol_id: Kalama\n");
		break;
	case HW_FENCE_IPCC_HW_REV_203:
		drv_data->ipcc_client_vid = HW_FENCE_IPC_CLIENT_ID_APPS_VID;
		drv_data->ipcc_client_pid = HW_FENCE_IPC_CLIENT_ID_APPS_PID;
		drv_data->protocol_id = HW_FENCE_IPC_FENCE_PROTOCOL_ID_PINEAPPLE; /* Fence */
		ret = _hw_fence_ipcc_init_map_with_configurable_clients(drv_data,
			hw_fence_clients_ipc_map_v2);
		HWFNC_DBG_INIT("ipcc protocol_id: Pineapple\n");
		break;
	default:
		return -1;
	}

	return ret;
}

int hw_fence_ipcc_enable_signaling(struct hw_fence_driver_data *drv_data)
{
	void __iomem *ptr;
	u32 val;
	int ret;

	HWFNC_DBG_H("enable ipc +\n");

	ret = of_property_read_u32(drv_data->dev->of_node, "qcom,hw-fence-ipc-ver", &val);
	if (ret || !val) {
		HWFNC_ERR("missing hw fences ipc-ver entry or invalid ret:%d val:%d\n", ret, val);
		return -EINVAL;
	}

	if (_hw_fence_ipcc_hwrev_init(drv_data, val)) {
		HWFNC_ERR("ipcc protocol id not supported\n");
		return -EINVAL;
	}

	/* Enable compute l1 (protocol_id = 2) */
	val = 0x00000000;
	ptr = IPC_PROTOCOLp_CLIENTc_CONFIG(drv_data->ipcc_io_mem, drv_data->protocol_id,
		drv_data->ipcc_client_pid);
	HWFNC_DBG_H("Write:0x%x to RegOffset:0x%pK\n", val, ptr);
	writel_relaxed(val, ptr);

	/* Enable Client-Signal pairs from APPS(NS) (0x8) to APPS(NS) (0x8) */
	val = 0x000080000;
	ptr = IPC_PROTOCOLp_CLIENTc_RECV_SIGNAL_ENABLE(drv_data->ipcc_io_mem, drv_data->protocol_id,
		drv_data->ipcc_client_pid);
	HWFNC_DBG_H("Write:0x%x to RegOffset:0x%pK\n", val, ptr);
	writel_relaxed(val, ptr);

	HWFNC_DBG_H("enable ipc -\n");

	return 0;
}

int hw_fence_ipcc_enable_dpu_signaling(struct hw_fence_driver_data *drv_data)
{
	struct hw_fence_client_ipc_map *hw_fence_client;
	bool protocol_enabled = false;
	void __iomem *ptr;
	u32 val;
	int i;

	HWFNC_DBG_H("enable dpu ipc +\n");

	if (!drv_data || !drv_data->protocol_id || !drv_data->ipc_clients_table) {
		HWFNC_ERR("invalid drv data\n");
		return -1;
	}

	HWFNC_DBG_H("ipcc_io_mem:0x%lx\n", (u64)drv_data->ipcc_io_mem);

	HWFNC_DBG_H("Initialize dpu signals\n");
	/* Enable Client-Signal pairs from DPU (25) to APPS(NS) (8) */
	for (i = 0; i < drv_data->clients_num; i++) {
		hw_fence_client = &drv_data->ipc_clients_table[i];

		/* skip any client that is not a dpu client */
		if (hw_fence_client->ipc_client_id_virt != HW_FENCE_IPC_CLIENT_ID_DPU_VID)
			continue;

		if (!protocol_enabled) {
			/*
			 * First DPU client will enable the protocol for dpu, e.g. compute l1
			 * (protocol_id = 2) or fencing protocol, depending on the target, for the
			 * dpu client (vid = 25, pid = 9).
			 * Sets bit(1) to clear when RECV_ID is read
			 */
			val = 0x00000001;
			ptr = IPC_PROTOCOLp_CLIENTc_CONFIG(drv_data->ipcc_io_mem,
				drv_data->protocol_id, hw_fence_client->ipc_client_id_phys);
			HWFNC_DBG_H("Write:0x%x to RegOffset:0x%lx\n", val, (u64)ptr);
			writel_relaxed(val, ptr);

			protocol_enabled = true;
		}

		/* Enable signals for dpu client */
		HWFNC_DBG_H("dpu client:%d vid:%d pid:%d signal:%d\n", i,
			hw_fence_client->ipc_client_id_virt, hw_fence_client->ipc_client_id_phys,
			hw_fence_client->ipc_signal_id);

		/* Enable input apps-signal for dpu */
		val = (HW_FENCE_IPC_CLIENT_ID_APPS_VID << 16) |
				(hw_fence_client->ipc_signal_id & 0xFFFF);
		ptr = IPC_PROTOCOLp_CLIENTc_RECV_SIGNAL_ENABLE(drv_data->ipcc_io_mem,
			drv_data->protocol_id, hw_fence_client->ipc_client_id_phys);
		HWFNC_DBG_H("Write:0x%x to RegOffset:0x%lx\n", val, (u64)ptr);
		writel_relaxed(val, ptr);
	}

	HWFNC_DBG_H("enable dpu ipc -\n");

	return 0;
}
