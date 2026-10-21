/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_PRESIL_HW_ACCESS_H_
#define _CAM_PRESIL_HW_ACCESS_H_

#include <linux/interrupt.h>

#define CAM_PRESIL_CLIENT_ID_CAMERA 0x1
#define CAM_PRESIL_CLIENT_ID_EVA    0x2

/* presil events to carry shared values from HW-KMD to PC-HOST CSim Wrapper */
#define CAM_PRESIL_EVENT_HFI_REG_BASE                                0x600
#define CAM_PRESIL_EVENT_HFI_REG(n) (CAM_PRESIL_EVENT_HFI_REG_BASE + (n * 4))
#define CAM_PRESIL_EVENT_HFI_REG_CMD_Q_IOVA                          CAM_PRESIL_EVENT_HFI_REG(1)
#define CAM_PRESIL_EVENT_HFI_REG_MSG_Q_IOVA                          CAM_PRESIL_EVENT_HFI_REG(2)
#define CAM_PRESIL_EVENT_HFI_REG_DBG_Q_IOVA                          CAM_PRESIL_EVENT_HFI_REG(3)
#define CAM_PRESIL_EVENT_HFI_REG_SFR_LEN                             CAM_PRESIL_EVENT_HFI_REG(4)
#define CAM_PRESIL_EVENT_HFI_REG_ICP_V1_HW_VERSION_TO_START_HFI_INIT CAM_PRESIL_EVENT_HFI_REG(13)
#define CAM_PRESIL_EVENT_HFI_REG_ON_FIRST_REG_START_FW_DOWNLOAD      0x638   /* write FF to start */
#define CAM_PRESIL_EVENT_IFE_FRAME_RUN                               0x123   /* write FF to start */


/*
 * enum cam_presil_err - return code from presil apis
 *
 * @CAM_PRESIL_SUCCESS         : Success
 * @CAM_PRESIL_FAILED          : Failed
 * @CAM_PRESIL_BLOCKED         : not presil hw
 * @CAM_PRESIL_BLOCKED_BOOTUP  : presil hw but at boot presil not connected
 *
 */
enum cam_presil_err {
	CAM_PRESIL_SUCCESS = 0x0,
	CAM_PRESIL_FAILED,
	CAM_PRESIL_BLOCKED,
	CAM_PRESIL_BLOCKED_BOOTUP,
};

/**
 * struct cam_presil_intr_reginfo - register received with irq
 * callback
 */
struct cam_presil_intr_reginfo
{
	uint32_t intr_en_off;
	uint32_t intr_status_off;
	uint32_t intr_en_val;
	uint32_t intr_status_val;
};

/**
 * struct cam_presil_intr_regwrinfo - reg val pair from pchost
 */
struct cam_presil_intr_regwrinfo
{
	void *   reg_off;
	uint32_t reg_val;
};

#define CAM_MODE_MAX_REG_CNT 25

/**
 * struct cam_presil_irq_data - data received along with irq cb
 * from pchost
 */
struct cam_presil_irq_data
{
	uint32_t                         irq_num;
	uint32_t                         irq_reg_count;
	struct cam_presil_intr_reginfo   intr_reg[CAM_MODE_MAX_REG_CNT];
	uint32_t                         irq_wr_count;
	struct cam_presil_intr_regwrinfo intr_wr_reg[CAM_MODE_MAX_REG_CNT];
	uint32_t                         magic;
};

/*
 *  cam_presil_subscribe_device_irq()
 *
 * @brief         :  Register for irq from presil framework.
 *
 * @irq_num       :  Unique irq number
 * @irq_handler   :  handler callback
 * @irq_priv_data :  Callback data
 * @irq_name      :  Irq name
 *
 * @return true or false.
 */
bool cam_presil_subscribe_device_irq(int irq_num, irq_handler_t irq_handler,
	void* irq_priv_data, const char *irq_name);

/*
 *  cam_presil_subscribe_device_irq()
 *
 * @brief   :  Un-Register for irq from presil framework.
 *
 * @irq_num :  Unique irq number
 *
 * @return true or false.
 */
bool cam_presil_unsubscribe_device_irq(int irq_num);

/*
 *  cam_presil_register_read()
 *
 * @brief   :  register read from presil hw.
 *
 * @addr    :  Register offset
 * @pValue  :  Value read from hw
 *
 * @return:  Success or Failure
 */
int cam_presil_register_read(void *addr, uint32_t *pValue);

/*
 *  cam_presil_register_write()
 *
 * @brief   :  register write to presil hw.
 *
 * @addr    :  Register offset
 * @pValue  :  Value to write to hw
 * @flags   :  Flags
 *
 * @return:  Success or Failure
 */
int cam_presil_register_write(void *addr, uint32_t value, uint32_t flags);

/*
 *  cam_presil_send_buffer()
 *
 * @brief        :  Copy buffer content to presil hw memory.
 *
 * @dma_buf_uint :  Not fd , it is dma_buf ptr to be sent to
 * 	presil umd daemon
 * @mmu_hdl      :  Iommu handle
 * @offset       :  Offset to start copy
 * @size         :  Size of copy
 * @addr32       :  Iova to start copy at
 *
 * @return:  Success or Failure
 */
int cam_presil_send_buffer(uint64_t dma_buf_uint, int mmu_hdl, uint32_t offset,
	uint32_t size, uint32_t addr32);

/*
 *  cam_presil_retrieve_buffer()
 *
 * @brief        :  Copy buffer content back from presil hw memory.
 *
 * @dma_buf_uint :  Not fd , it is dma_buf ptr to be sent to
 * 	presil umd daemon
 * @mmu_hdl      :  Iommu handle
 * @offset       :  Offset to start copy
 * @size         :  Size of copy
 * @addr32       :  Iova to start copy at
 *
 * @return:  Success or Failure
 */
int cam_presil_retrieve_buffer(uint64_t dma_buf_uint,
	int mmu_hdl, uint32_t offset, uint32_t size, uint32_t addr32);

/*
 *  cam_presil_readl_poll_timeout()
 *
 * @brief         :  Custom register poll function for presil hw.
 *
 * @mem_address   :  Reg offset to poll
 * @val           :  Value to compare
 * @max_try_count :  Max number of tries
 * @interval_msec :  Interval between tries
 *
 * @return:  Success or Failure
 */
int cam_presil_readl_poll_timeout(void __iomem *mem_address, uint32_t val,
	int max_try_count, int interval_msec);

/*
 *  cam_presil_hfi_write_cmd()
 *
 * @brief   :  Write HFI command to presil hw.
 *
 * @addr       :  Pointer to HFI command
 * @cmdlen     :  Length
 * @client_id  :  client Id of caller
 *
 * @return:  Success or Failure
 */
int cam_presil_hfi_write_cmd(void *addr, uint32_t cmdlen, uint32_t client_id);

/*
 *  cam_presil_hfi_read_message()
 *
 * @brief         :  Read HFI response message from presil hw.
 *
 * @pmsg          :  Pointer to HFI message buffer
 * @q_id          :  Length
 * @words_read    :  Response message
 * @client_id     :  client Id of caller
 *
 * @return:  Success or Failure
 */
int cam_presil_hfi_read_message(uint32_t *pmsg, uint8_t q_id,
	uint32_t *words_read, uint32_t client_id);

/**
 * @brief : API to check if camera driver running in presil
 *        enabled mode
 *
 * @return true or false.
 */
bool cam_presil_mode_enabled(void);

/*
 *  cam_presil_send_event()
 *
 * @brief   :  send event to pchost.
 *
 * @event_id :  Event Id
 * @value  :  Value with additional information
 *
 * @return:  Success or Failure
 */
int cam_presil_send_event(uint32_t event_id, uint32_t value);

#endif /* _CAM_PRESIL_HW_ACCESS_H_ */
