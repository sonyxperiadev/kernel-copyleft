/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __HW_FENCE_DRV_IPC_H
#define __HW_FENCE_DRV_IPC_H

/* ipc clients virtual client-id */
#define HW_FENCE_IPC_CLIENT_ID_APPS_VID 8
#define HW_FENCE_IPC_CLIENT_ID_GPU_VID 9
#define HW_FENCE_IPC_CLIENT_ID_IPE_VID 11
#define HW_FENCE_IPC_CLIENT_ID_VPU_VID 12
#define HW_FENCE_IPC_CLIENT_ID_DPU_VID 25
#define HW_FENCE_IPC_CLIENT_ID_IFE0_VID 128
#define HW_FENCE_IPC_CLIENT_ID_IFE1_VID 129
#define HW_FENCE_IPC_CLIENT_ID_IFE2_VID 130
#define HW_FENCE_IPC_CLIENT_ID_IFE3_VID 131
#define HW_FENCE_IPC_CLIENT_ID_IFE4_VID 132
#define HW_FENCE_IPC_CLIENT_ID_IFE5_VID 133
#define HW_FENCE_IPC_CLIENT_ID_IFE6_VID 134
#define HW_FENCE_IPC_CLIENT_ID_IFE7_VID 135

/* ipc clients physical client-id */
#define HW_FENCE_IPC_CLIENT_ID_APPS_PID 3
#define HW_FENCE_IPC_CLIENT_ID_GPU_PID 4
#define HW_FENCE_IPC_CLIENT_ID_IPE_PID 5
#define HW_FENCE_IPC_CLIENT_ID_VPU_PID 8
#define HW_FENCE_IPC_CLIENT_ID_DPU_PID 9
#define HW_FENCE_IPC_CLIENT_ID_IFE0_PID 11
#define HW_FENCE_IPC_CLIENT_ID_IFE1_PID 12
#define HW_FENCE_IPC_CLIENT_ID_IFE2_PID 13
#define HW_FENCE_IPC_CLIENT_ID_IFE3_PID 14
#define HW_FENCE_IPC_CLIENT_ID_IFE4_PID 15
#define HW_FENCE_IPC_CLIENT_ID_IFE5_PID 16
#define HW_FENCE_IPC_CLIENT_ID_IFE6_PID 17
#define HW_FENCE_IPC_CLIENT_ID_IFE7_PID 18

#define HW_FENCE_IPC_COMPUTE_L1_PROTOCOL_ID_KALAMA 2
#define HW_FENCE_IPC_COMPUTE_L1_PROTOCOL_ID_PINEAPPLE 2
#define HW_FENCE_IPC_FENCE_PROTOCOL_ID_PINEAPPLE 4

#define HW_FENCE_IPCC_HW_REV_170 0x00010700  /* Kalama */
#define HW_FENCE_IPCC_HW_REV_203 0x00020003  /* Pineapple */

#define IPC_PROTOCOLp_CLIENTc_VERSION(base, p, c) (base + (0x40000*p) + (0x1000*c))
#define IPC_PROTOCOLp_CLIENTc_CONFIG(base, p, c) (base + 0x8 + (0x40000*p) + (0x1000*c))
#define IPC_PROTOCOLp_CLIENTc_RECV_SIGNAL_ENABLE(base, p, c) \
	(base + 0x14 + (0x40000*p) + (0x1000*c))
#define IPC_PROTOCOLp_CLIENTc_SEND(base, p, c) ((base + 0xc) + (0x40000*p) + (0x1000*c))

/**
 * hw_fence_ipcc_trigger_signal() - Trigger ipc signal for the requested client/signal pair.
 * @drv_data: driver data.
 * @tx_client_id: ipc client id that sends the ipc signal.
 * @rx_client_id: ipc client id that receives the ipc signal.
 * @signal_id: signal id to send.
 *
 * This API triggers the ipc 'signal_id' from the 'tx_client_id' to the 'rx_client_id'
 */
void hw_fence_ipcc_trigger_signal(struct hw_fence_driver_data *drv_data,
	u32 tx_client_id, u32 rx_client_id, u32 signal_id);

/**
 * hw_fence_ipcc_enable_signaling() - Enable ipcc signaling for hw-fence driver.
 * @drv_data: driver data.
 *
 * Return: 0 on success or negative errno (-EINVAL)
 */
int hw_fence_ipcc_enable_signaling(struct hw_fence_driver_data *drv_data);

/**
 * hw_fence_ipcc_enable_dpu_signaling() - Enable ipcc signaling for dpu client.
 * @drv_data: driver data.
 *
 * Return: 0 on success or negative errno (-EINVAL)
 */
int hw_fence_ipcc_enable_dpu_signaling(struct hw_fence_driver_data *drv_data);

/**
 * hw_fence_ipcc_get_client_virt_id() - Returns the ipc client virtual id that corresponds to the
 *		hw fence driver client.
 * @drv_data: driver data.
 * @client_id: hw fence driver client id.
 *
 * The ipc client id returned by this API is used by the hw fence driver when signaling the fence.
 *
 * Return: client_id on success or negative errno (-EINVAL)
 */
int hw_fence_ipcc_get_client_virt_id(struct hw_fence_driver_data *drv_data, u32 client_id);

/**
 * hw_fence_ipcc_get_client_phys_id() - Returns the ipc client physical id that corresponds to the
 *		hw fence driver client.
 * @drv_data: driver data.
 * @client_id: hw fence driver client id.
 *
 * The ipc client id returned by this API is used by the hw fence driver when signaling the fence.
 *
 * Return: client_id on success or negative errno (-EINVAL)
 */
int hw_fence_ipcc_get_client_phys_id(struct hw_fence_driver_data *drv_data, u32 client_id);

/**
 * hw_fence_ipcc_get_signal_id() - Returns the ipc signal id that corresponds to the hw fence
 *		driver client.
 * @drv_data: driver data.
 * @client_id: hw fence driver client id.
 *
 * The ipc signal id returned by this API is used by the hw fence driver when signaling the fence.
 *
 * Return: client_id on success or negative errno (-EINVAL)
 */
int hw_fence_ipcc_get_signal_id(struct hw_fence_driver_data *drv_data, u32 client_id);

/**
 * hw_fence_ipcc_needs_rxq_update() - Returns bool to indicate if client uses rx-queue.
 * @drv_data: driver data.
 * @client_id: hw fence driver client id.
 *
 * Return: true if client needs to update rxq, false otherwise
 */
bool hw_fence_ipcc_needs_rxq_update(struct hw_fence_driver_data *drv_data, int client_id);

/**
 * hw_fence_ipcc_needs_ipc_irq() - Returns bool to indicate if client needs ipc interrupt for
 *		already signaled fences
 * @drv_data: driver data.
 * @client_id: hw fence driver client id.
 *
 * Return: true if client needs ipc interrupt for signaled fences, false otherwise
 */
bool hw_fence_ipcc_needs_ipc_irq(struct hw_fence_driver_data *drv_data, int client_id);

#endif /* __HW_FENCE_DRV_IPC_H */
