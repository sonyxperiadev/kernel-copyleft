// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_BTFM_CODEC_H
#define __LINUX_BTFM_CODEC_H

#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/printk.h>
#include <linux/cdev.h>
#include <linux/skbuff.h>
#include "btfm_codec_hw_interface.h"

#define BTM_BTFMCODEC_DEFAULT_LOG_LVL        0x03
#define BTM_BTFMCODEC_DEBUG_LOG_LVL          0x04
#define BTM_BTFMCODEC_INFO_LOG_LVL           0x08

static uint8_t log_lvl = BTM_BTFMCODEC_DEFAULT_LOG_LVL;

#define BTFMCODEC_ERR(fmt, arg...)  pr_err("%s: " fmt "\n", __func__, ## arg)
#define BTFMCODEC_WARN(fmt, arg...) pr_warn("%s: " fmt "\n", __func__, ## arg)
#define BTFMCODEC_DBG(fmt, arg...)  { if(log_lvl >= BTM_BTFMCODEC_DEBUG_LOG_LVL) \
	                                pr_err("%s: " fmt "\n", __func__, ## arg); \
				      else \
	                                pr_debug("%s: " fmt "\n", __func__, ## arg); \
	                            }
#define BTFMCODEC_INFO(fmt, arg...) { if(log_lvl >= BTM_BTFMCODEC_INFO_LOG_LVL) \
					pr_err("%s: " fmt "\n", __func__, ## arg);\
				      else \
					pr_info("%s: " fmt "\n", __func__, ## arg);\
				    }

#define DEVICE_NAME_MAX_LEN	64
#define BTM_CP_UPDATE           0xbfaf

typedef enum btfmcodec_states {
	/*Default state of kernel proxy driver */
	IDLE = 0,
	/* Waiting for BT bearer indication after configuring HW ports */
	BT_Connecting = 1,
	/* When BT is active transport */
	BT_Connected = 2,
	/* Waiting for BTADV Audio bearer switch indications */
	BTADV_AUDIO_Connecting = 3,
	/* When BTADV audio is active transport */
	BTADV_AUDIO_Connected = 4
} btfmcodec_state;

enum btfm_pkt_type {
	BTM_PKT_TYPE_PREPARE_REQ = 0,
	BTM_PKT_TYPE_MASTER_CONFIG_RSP,
	BTM_PKT_TYPE_MASTER_SHUTDOWN_RSP,
	BTM_PKT_TYPE_BEARER_SWITCH_IND,
	BTM_PKT_TYPE_HWEP_SHUTDOWN,
	BTM_PKT_TYPE_HWEP_CONFIG,
	BTM_PKT_TYPE_DMA_CONFIG_RSP,
	BTM_PKT_TYPE_MAX,
};


char *coverttostring(enum btfmcodec_states);
struct btfmcodec_state_machine {
	struct mutex state_machine_lock;
	btfmcodec_state prev_state;
	btfmcodec_state current_state;
	btfmcodec_state next_state;
};

struct btfmcodec_char_device {
	struct cdev cdev;
	refcount_t active_clients;
	struct mutex lock;
	int reuse_minor;
	char dev_name[DEVICE_NAME_MAX_LEN];
	struct workqueue_struct *workqueue;
	struct sk_buff_head rxq;
	struct work_struct rx_work;
	struct work_struct wq_hwep_shutdown;
	struct work_struct wq_prepare_bearer;
	struct work_struct wq_hwep_configure;
	wait_queue_head_t readq;
	spinlock_t tx_queue_lock;
	struct sk_buff_head txq;
	wait_queue_head_t rsp_wait_q[BTM_PKT_TYPE_MAX];
	uint8_t status[BTM_PKT_TYPE_MAX];
	void *btfmcodec;
};

struct adsp_notifier {
	void *notifier;
	struct notifier_block nb;
};

struct btfmcodec_data {
	struct device dev;
	struct btfmcodec_state_machine states;
	struct btfmcodec_char_device *btfmcodec_dev;
	struct hwep_data *hwep_info;
	struct list_head config_head;
	struct adsp_notifier notifier;
	struct mutex hwep_drv_lock;
};

struct btfmcodec_data *btfm_get_btfmcodec(void);
bool isCpSupported(void);
#endif /*__LINUX_BTFM_CODEC_H */
