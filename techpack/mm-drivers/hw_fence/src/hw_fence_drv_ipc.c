// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/of_platform.h>
#include "hw_fence_drv_priv.h"
#include "hw_fence_drv_utils.h"
#include "hw_fence_drv_ipc.h"
#include "hw_fence_drv_debug.h"

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
 * struct hw_fence_clients_ipc_map_no_dpu - Table makes the 'client to signal' mapping, which
 *		is used by the hw fence driver to trigger ipc signal when the hw fence is already
 *		signaled.
 *		This no_dpu version is for targets that do not support dpu client id
 *
 * Notes:
 * The index of this struct must match the enum hw_fence_client_id.
 * To change to a loopback signal instead of GMU, change ctx0 row to use:
 *   {HW_FENCE_IPC_CLIENT_ID_APPS, 20}.
 */
struct hw_fence_client_ipc_map hw_fence_clients_ipc_map_no_dpu[HW_FENCE_CLIENT_MAX] = {
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 1, true, true},/* ctrlq*/
	{HW_FENCE_IPC_CLIENT_ID_GPU_VID, HW_FENCE_IPC_CLIENT_ID_GPU_VID, 0, false, false},/* ctx0 */
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 14, false, true},/*ctl0*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 15, false, true},/*ctl1*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 16, false, true},/*ctl2*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 17, false, true},/*ctl3*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 18, false, true},/*ctl4*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 19, false, true},/*ctl5*/
#if IS_ENABLED(CONFIG_DEBUG_FS)
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 21, true, true},/* val0*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 22, true, true},/* val1*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 23, true, true},/* val2*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 24, true, true},/* val3*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 25, true, true},/* val4*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 26, true, true},/* val5*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 27, true, true},/* val6*/
#endif /* CONFIG_DEBUG_FS */
};

/**
 * struct hw_fence_clients_ipc_map - Table makes the 'client to signal' mapping, which is
 *		used by the hw fence driver to trigger ipc signal when hw fence is already
 *		signaled.
 *		This version is for targets that support dpu client id.
 *
 * Note that the index of this struct must match the enum hw_fence_client_id
 */
struct hw_fence_client_ipc_map hw_fence_clients_ipc_map[HW_FENCE_CLIENT_MAX] = {
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 1, true, true},/*ctrl q*/
	{HW_FENCE_IPC_CLIENT_ID_GPU_VID,  HW_FENCE_IPC_CLIENT_ID_GPU_VID, 0, false, false},/*ctx0 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 0, false, true},/* ctl0 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 1, false, true},/* ctl1 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 2, false, true},/* ctl2 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 3, false, true},/* ctl3 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 4, false, true},/* ctl4 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_VID, 5, false, true},/* ctl5 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 21, true, true},/* val0*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 22, true, true},/* val1*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 23, true, true},/* val2*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 24, true, true},/* val3*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 25, true, true},/* val4*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 26, true, true},/* val5*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_VID, 27, true, true},/* val6*/
#endif /* CONFIG_DEBUG_FS */
};

/**
 * struct hw_fence_clients_ipc_map_v2 - Table makes the 'client to signal' mapping, which is
 *		used by the hw fence driver to trigger ipc signal when hw fence is already
 *		signaled.
 *		This version is for targets that support dpu client id and IPC v2.
 *
 * Note that the index of this struct must match the enum hw_fence_client_id
 */
struct hw_fence_client_ipc_map hw_fence_clients_ipc_map_v2[HW_FENCE_CLIENT_MAX] = {
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 1, true, true},/*ctrlq */
	{HW_FENCE_IPC_CLIENT_ID_GPU_VID,  HW_FENCE_IPC_CLIENT_ID_GPU_PID, 0, false, false},/* ctx0*/
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 0, false, true},/* ctl0 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 1, false, true},/* ctl1 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 2, false, true},/* ctl2 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 3, false, true},/* ctl3 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 4, false, true},/* ctl4 */
	{HW_FENCE_IPC_CLIENT_ID_DPU_VID,  HW_FENCE_IPC_CLIENT_ID_DPU_PID, 5, false, true},/* ctl5 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 21, true, true},/* val0*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 22, true, true},/* val1*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 23, true, true},/* val2*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 24, true, true},/* val3*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 25, true, true},/* val4*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 26, true, true},/* val5*/
	{HW_FENCE_IPC_CLIENT_ID_APPS_VID, HW_FENCE_IPC_CLIENT_ID_APPS_PID, 27, true, true},/* val6*/
#endif /* CONFIG_DEBUG_FS */
};

int hw_fence_ipcc_get_client_virt_id(struct hw_fence_driver_data *drv_data, u32 client_id)
{
	if (!drv_data || client_id >= HW_FENCE_CLIENT_MAX)
		return -EINVAL;

	return drv_data->ipc_clients_table[client_id].ipc_client_id_virt;
}

int hw_fence_ipcc_get_client_phys_id(struct hw_fence_driver_data *drv_data, u32 client_id)
{
	if (!drv_data || client_id >= HW_FENCE_CLIENT_MAX)
		return -EINVAL;

	return drv_data->ipc_clients_table[client_id].ipc_client_id_phys;
}

int hw_fence_ipcc_get_signal_id(struct hw_fence_driver_data *drv_data, u32 client_id)
{
	if (!drv_data || client_id >= HW_FENCE_CLIENT_MAX)
		return -EINVAL;

	return drv_data->ipc_clients_table[client_id].ipc_signal_id;
}

bool hw_fence_ipcc_needs_rxq_update(struct hw_fence_driver_data *drv_data, int client_id)
{
	if (!drv_data || client_id >= HW_FENCE_CLIENT_MAX)
		return -EINVAL;

	return drv_data->ipc_clients_table[client_id].update_rxq;
}

bool hw_fence_ipcc_needs_ipc_irq(struct hw_fence_driver_data *drv_data, int client_id)
{
	if (!drv_data || client_id >= HW_FENCE_CLIENT_MAX)
		return -EINVAL;

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

/**
 * _hw_fence_ipcc_hwrev_init() - Initializes internal driver struct with corresponding ipcc data,
 *		according to the ipcc hw revision.
 * @drv_data: driver data.
 * @hwrev: ipcc hw revision.
 */
static int _hw_fence_ipcc_hwrev_init(struct hw_fence_driver_data *drv_data, u32 hwrev)
{
	switch (hwrev) {
	case HW_FENCE_IPCC_HW_REV_100:
		drv_data->ipcc_client_vid = HW_FENCE_IPC_CLIENT_ID_APPS_VID;
		drv_data->ipcc_client_pid = HW_FENCE_IPC_CLIENT_ID_APPS_VID;
		drv_data->protocol_id = HW_FENCE_IPC_COMPUTE_L1_PROTOCOL_ID_LAHAINA;
		drv_data->ipc_clients_table = hw_fence_clients_ipc_map_no_dpu;
		HWFNC_DBG_INIT("ipcc protocol_id: Lahaina\n");
		break;
	case HW_FENCE_IPCC_HW_REV_110:
		drv_data->ipcc_client_vid = HW_FENCE_IPC_CLIENT_ID_APPS_VID;
		drv_data->ipcc_client_pid = HW_FENCE_IPC_CLIENT_ID_APPS_VID;
		drv_data->protocol_id = HW_FENCE_IPC_COMPUTE_L1_PROTOCOL_ID_WAIPIO;
		drv_data->ipc_clients_table = hw_fence_clients_ipc_map_no_dpu;
		HWFNC_DBG_INIT("ipcc protocol_id: Waipio\n");
		break;
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
		drv_data->protocol_id = HW_FENCE_IPC_COMPUTE_L1_PROTOCOL_ID_PINEAPPLE; /* Fence */
		drv_data->ipc_clients_table = hw_fence_clients_ipc_map_v2;
		HWFNC_DBG_INIT("ipcc protocol_id: Pineapple\n");
		break;
	default:
		return -1;
	}

	return 0;
}

int hw_fence_ipcc_enable_signaling(struct hw_fence_driver_data *drv_data)
{
	void __iomem *ptr;
	u32 val;
	int ret;

	HWFNC_DBG_H("enable ipc +\n");

	/**
	 * Attempt to read the ipc version from dt, if not available, then attempt
	 * to read from the registers.
	 */
	ret = of_property_read_u32(drv_data->dev->of_node, "qcom,hw-fence-ipc-ver", &val);
	if (ret || !val) {
		/* if no device tree prop, attempt to get the version from the registers*/
		HWFNC_DBG_H("missing hw fences ipc-ver entry or invalid ret:%d val:%d\n", ret, val);

		/* Read IPC Version from Client=0x8 (apps) for protocol=2 (compute_l1) */
		val = readl_relaxed(IPC_PROTOCOLp_CLIENTc_VERSION(drv_data->ipcc_io_mem,
			HW_FENCE_IPC_COMPUTE_L1_PROTOCOL_ID_LAHAINA,
			HW_FENCE_IPC_CLIENT_ID_APPS_VID));
		HWFNC_DBG_INIT("ipcc version:0x%x\n", val);
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

#ifdef HW_DPU_IPCC
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
	for (i = 0; i < HW_FENCE_CLIENT_MAX; i++) {
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
#endif /* HW_DPU_IPCC */
