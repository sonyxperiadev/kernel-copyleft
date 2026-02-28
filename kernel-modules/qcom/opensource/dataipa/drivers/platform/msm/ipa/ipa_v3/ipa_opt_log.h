/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/cdev.h>

#ifndef _IPA3_OPT_LOG_H_
#define _IPA3_OPT_LOG_H_

#define MAX_QUEUE_TO_OPT_LOG 128

#define EVENT_LOG_NAME "ipa-event-log"

struct ipa3_opt_log_stats {
	u32 opt_log_rx_pkt;
	u32 opt_log_tx_diag_pkt;
	u32 opt_log_drop_pkt;
	atomic_t numer_in_queue;
};

struct opt_log_state_bit_mask {
	u32 opt_log_init:1;
	u32 opt_log_open:1;
	u32:0;
};

/**
 * struct ipa3_opt_log_char_device_context - IPA WDI OPT DPATH character device
 * @class: pointer to the struct class
 * @dev_num: device number
 * @dev: the dev_t of the device
 * @cdev: cdev of the device
 */
struct ipa3_opt_log_char_device_context {
	struct class *class;
	dev_t dev_num;
	struct device *dev;
	struct cdev cdev;
};

struct ipa_opt_log_context {
	struct ipa3_opt_log_char_device_context opt_log_cdev;
	struct list_head opt_log_msg_list;
	struct mutex opt_log_msg_lock;
	struct mutex ctx_lock;
	struct opt_log_state_bit_mask opt_log_state;
	struct ipa3_opt_log_stats stats;
	wait_queue_head_t opt_log_msg_waitq;
};

struct ipa3_push_msg_wdi {
	void *buff;
	int len;
	struct list_head link;
};

extern struct ipa_opt_log_context *ipa3_opt_log_ctx;

int opt_log_file_test(void);
int ipa_opt_log_init(void);
int ipa3_send_opt_log_msg(const char *input_string);
void ipa3_opt_log_cleanup(void);

#endif /* _IPA3_OPT_LOG_H_ */
