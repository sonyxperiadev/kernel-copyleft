// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/moduleparam.h>
#include "cam_common_util.h"
#include "cam_debug_util.h"
#include "cam_presil_hw_access.h"
#include "cam_hw.h"
#if IS_REACHABLE(CONFIG_QCOM_VA_MINIDUMP)
#include <soc/qcom/minidump.h>
static  struct cam_common_mini_dump_dev_info g_minidump_dev_info;
#endif

#define CAM_PRESIL_POLL_DELAY 20

static struct cam_common_err_inject_info g_err_inject_info;

static uint timeout_multiplier = 1;
module_param(timeout_multiplier, uint, 0644);

int cam_common_util_get_string_index(const char **strings,
	uint32_t num_strings, const char *matching_string, uint32_t *index)
{
	int i;

	for (i = 0; i < num_strings; i++) {
		if (strnstr(strings[i], matching_string, strlen(strings[i]))) {
			CAM_DBG(CAM_UTIL, "matched %s : %d\n",
				matching_string, i);
			*index = i;
			return 0;
		}
	}

	return -EINVAL;
}

uint32_t cam_common_util_remove_duplicate_arr(int32_t *arr, uint32_t num)
{
	int i, j;
	uint32_t wr_idx = 1;

	if (!arr) {
		CAM_ERR(CAM_UTIL, "Null input array");
		return 0;
	}

	for (i = 1; i < num; i++) {
		for (j = 0; j < wr_idx ; j++) {
			if (arr[i] == arr[j])
				break;
		}
		if (j == wr_idx)
			arr[wr_idx++] = arr[i];
	}

	return wr_idx;
}

unsigned long cam_common_wait_for_completion_timeout(
	struct completion   *complete,
	unsigned long        timeout_jiffies)
{
	unsigned long wait_jiffies;
	unsigned long rem_jiffies;

	if (!complete) {
		CAM_ERR(CAM_UTIL, "Null complete pointer");
		return 0;
	}

	if (timeout_multiplier < 1)
		timeout_multiplier = 1;

	wait_jiffies = timeout_jiffies * timeout_multiplier;
	rem_jiffies = wait_for_completion_timeout(complete, wait_jiffies);

	return rem_jiffies;
}

int cam_common_read_poll_timeout(
	void __iomem        *addr,
	unsigned long        delay,
	unsigned long        timeout,
	uint32_t             mask,
	uint32_t             check_val,
	uint32_t            *status)
{
	unsigned long wait_time_us;
	int rc = -EINVAL;

	if (!addr || !status) {
		CAM_ERR(CAM_UTIL, "Invalid param addr: %pK status: %pK",
			addr, status);
		return rc;
	}

	if (timeout_multiplier < 1)
		timeout_multiplier = 1;

	wait_time_us = timeout * timeout_multiplier;

	if (false == cam_presil_mode_enabled()) {
		rc = readl_poll_timeout(addr, *status, (*status & mask) == check_val, delay,
			wait_time_us);
	} else {
		rc = cam_presil_readl_poll_timeout(addr, mask,
			wait_time_us/(CAM_PRESIL_POLL_DELAY * 1000), CAM_PRESIL_POLL_DELAY);
	}

	return rc;
}

int cam_common_modify_timer(struct timer_list *timer, int32_t timeout_val)
{
	if (!timer) {
		CAM_ERR(CAM_UTIL, "Invalid reference to system timer");
		return -EINVAL;
	}

	if (timeout_multiplier < 1)
		timeout_multiplier = 1;

	CAM_DBG(CAM_UTIL, "Starting timer to fire in %d ms. (jiffies=%lu)\n",
		(timeout_val * timeout_multiplier), jiffies);
	mod_timer(timer,
		(jiffies + msecs_to_jiffies(timeout_val * timeout_multiplier)));

	return 0;
}

void cam_common_util_thread_switch_delay_detect(
	const char *token, ktime_t scheduled_time, uint32_t threshold)
{
	uint64_t                         diff;
	ktime_t                          cur_time;
	struct timespec64                cur_ts;
	struct timespec64                scheduled_ts;

	cur_time = ktime_get();
	diff = ktime_ms_delta(cur_time, scheduled_time);

	if (diff > threshold) {
		scheduled_ts  = ktime_to_timespec64(scheduled_time);
		cur_ts = ktime_to_timespec64(cur_time);
		CAM_WARN_RATE_LIMIT_CUSTOM(CAM_UTIL, 1, 1,
			"%s delay detected %ld:%06ld cur %ld:%06ld diff %ld: threshold %d",
			token, scheduled_ts.tv_sec,
			scheduled_ts.tv_nsec/NSEC_PER_USEC,
			cur_ts.tv_sec, cur_ts.tv_nsec/NSEC_PER_USEC,
			diff, threshold);
	}
}

#if IS_REACHABLE(CONFIG_QCOM_VA_MINIDUMP)
static void cam_common_mini_dump_handler(void *dst, unsigned long len)
{
	int                               i = 0;
	uint8_t                          *waddr;
	unsigned long                     bytes_written = 0;
	unsigned long                     remain_len = len;
	struct cam_common_mini_dump_data *md;

	if (len < sizeof(*md)) {
	    CAM_WARN(CAM_UTIL, "Insufficient len %lu", len);
	    return;
	}

	md = (struct cam_common_mini_dump_data *)dst;
	waddr = (uint8_t *)md + sizeof(*md);
	remain_len -= sizeof(*md);

	for (i = 0; i < CAM_COMMON_MINI_DUMP_DEV_NUM; i++) {
		if (!g_minidump_dev_info.dump_cb[i])
			continue;

		memcpy(md->name[i], g_minidump_dev_info.name[i],
			strlen(g_minidump_dev_info.name[i]));
		md->waddr[i] = (void *)waddr;
		bytes_written = g_minidump_dev_info.dump_cb[i](
			(void *)waddr, remain_len);
		md->size[i] = bytes_written;
		if (bytes_written >= len) {
			CAM_WARN(CAM_UTIL, "No more space to dump");
			goto nomem;
		}

		remain_len -= bytes_written;
		waddr += bytes_written;
	}

	return;
nomem:
    for (; i >=0; i--)
	    CAM_WARN(CAM_UTIL, "%s: Dumped len: %lu", md->name[i], md->size[i]);
}

static int cam_common_md_notify_handler(struct notifier_block *this,
	unsigned long event, void *ptr)
{
	struct va_md_entry cbentry;
	int rc = 0;

	cbentry.vaddr = 0x0;
	strlcpy(cbentry.owner, "Camera", sizeof(cbentry.owner));
	cbentry.size = CAM_COMMON_MINI_DUMP_SIZE;
	cbentry.cb = cam_common_mini_dump_handler;
	rc = qcom_va_md_add_region(&cbentry);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Va Region add falied %d", rc);
		return NOTIFY_STOP_MASK;
	}

	return NOTIFY_OK;
}

static struct notifier_block cam_common_md_notify_blk = {
	.notifier_call = cam_common_md_notify_handler,
	.priority = INT_MAX,
};

int cam_common_register_mini_dump_cb(
	cam_common_mini_dump_cb mini_dump_cb,
	uint8_t *dev_name)
{
	int rc = 0;

	if (g_minidump_dev_info.num_devs >= CAM_COMMON_MINI_DUMP_DEV_NUM) {
		CAM_ERR(CAM_UTIL, "No free index available");
		return -EINVAL;
	}

	if (!mini_dump_cb || !dev_name) {
		CAM_ERR(CAM_UTIL, "Invalid params");
		return -EINVAL;
	}

	g_minidump_dev_info.dump_cb[g_minidump_dev_info.num_devs] =
		mini_dump_cb;
	scnprintf(g_minidump_dev_info.name[g_minidump_dev_info.num_devs],
		CAM_COMMON_MINI_DUMP_DEV_NAME_LEN, dev_name);
	g_minidump_dev_info.num_devs++;
	if (!g_minidump_dev_info.is_registered) {
		rc = qcom_va_md_register("Camera", &cam_common_md_notify_blk);
		if (rc) {
			CAM_ERR(CAM_UTIL, "Camera VA minidump register failed");
			goto end;
		}
		g_minidump_dev_info.is_registered = true;
	}
end:
	return rc;
}
#endif

void *cam_common_user_dump_clock(
	void *dump_struct, uint8_t *addr_ptr)
{
	struct cam_hw_info  *hw_info = NULL;
	uint64_t            *addr = NULL;

	hw_info = (struct cam_hw_info *)dump_struct;

	if (!hw_info || !addr_ptr) {
		CAM_ERR(CAM_ISP, "HW info or address pointer NULL");
		return addr;
	}

	addr = (uint64_t *)addr_ptr;
	*addr++ = hw_info->soc_info.applied_src_clk_rate;
	return addr;
}

int cam_common_user_dump_helper(
	void *cmd_args,
	void *(*func)(void *dump_struct, uint8_t *addr_ptr),
	void *dump_struct,
	size_t size,
	const char *tag, ...)
{

	uint8_t                                   *dst;
	uint8_t                                   *addr, *start;
	void                                      *returned_ptr;
	struct cam_common_hw_dump_args            *dump_args;
	struct cam_common_hw_dump_header          *hdr;
	va_list                                    args;
	void*(*func_ptr)(void *dump_struct, uint8_t *addr_ptr);

	dump_args = (struct cam_common_hw_dump_args *)cmd_args;
	if (!dump_args->cpu_addr || !dump_args->buf_len) {
		CAM_ERR(CAM_UTIL,
			"Invalid params %pK %zu",
			(void *)dump_args->cpu_addr,
			dump_args->buf_len);
		return -EINVAL;
	}
	if (dump_args->buf_len <= dump_args->offset) {
		CAM_WARN(CAM_UTIL,
			"Dump offset overshoot offset %zu buf_len %zu",
			dump_args->offset, dump_args->buf_len);
		return -ENOSPC;
	}

	dst = (uint8_t *)dump_args->cpu_addr + dump_args->offset;
	hdr = (struct cam_common_hw_dump_header *)dst;

	va_start(args, tag);
	vscnprintf(hdr->tag, CAM_COMMON_HW_DUMP_TAG_MAX_LEN, tag, args);
	va_end(args);

	hdr->word_size = size;

	addr = (uint8_t *)(dst + sizeof(struct cam_common_hw_dump_header));
	start = addr;

	func_ptr = func;
	returned_ptr = func_ptr(dump_struct, addr);

	if (IS_ERR(returned_ptr))
		return PTR_ERR(returned_ptr);

	addr = (uint8_t *)returned_ptr;
	hdr->size = addr - start;
	CAM_DBG(CAM_UTIL, "hdr size: %d, word size: %d, addr: %x, start: %x",
		hdr->size, hdr->word_size, addr, start);
	dump_args->offset += hdr->size +
		sizeof(struct cam_common_hw_dump_header);

	return 0;
}

int cam_common_register_err_inject_cb(
	cam_common_err_inject_cb err_inject_cb,
	enum cam_common_err_inject_hw_id hw_id)
{
	int rc = 0;

	if (g_err_inject_info.num_hw_registered >= CAM_COMMON_ERR_INJECT_HW_MAX) {
		CAM_ERR(CAM_UTIL, "No free index available");
		return -EINVAL;
	}

	if (!err_inject_cb || hw_id >= CAM_COMMON_ERR_INJECT_HW_MAX) {
		CAM_ERR(CAM_UTIL, "Invalid params");
		return -EINVAL;
	}

	g_err_inject_info.err_inject_cb[hw_id] = err_inject_cb;
	g_err_inject_info.num_hw_registered++;
	CAM_DBG(CAM_UTIL, "err inject cb registered for HW_id:%d, total registered: %d", hw_id,
		g_err_inject_info.num_hw_registered);
	return rc;
}

int cam_common_release_err_params(uint64_t dev_hdl)
{
	int rc = 0;
	struct list_head *pos = NULL, *pos_next = NULL;
	struct cam_err_inject_param *err_param;

	if (!g_err_inject_info.is_list_initialised)
		return -EINVAL;

	if (list_empty(&g_err_inject_info.active_err_ctx_list)) {
		CAM_ERR(CAM_UTIL, "list is empty");
		return -EINVAL;
	}
	list_for_each_safe(pos, pos_next, &g_err_inject_info.active_err_ctx_list) {
		err_param = list_entry(pos, struct cam_err_inject_param, list);
		if (err_param->dev_hdl == dev_hdl) {
			CAM_INFO(CAM_UTIL, "entry deleted for %llu dev hdl", dev_hdl);
			list_del(pos);
			kfree(err_param);
		}
	}
	return rc;
}

static int cam_err_inject_set(const char *kmessage,
	const struct kernel_param *kp)
{
	int     rc                                         = 0;
	char    tmp_buff[CAM_COMMON_ERR_INJECT_BUFFER_LEN] = {'\0'};
	char    *token_start, *token_end;
	struct  cam_err_inject_param *err_params           = NULL;
	uint8_t param_counter                              = 0;

	err_params = kzalloc(sizeof(struct cam_err_inject_param), GFP_KERNEL);
	if (!err_params) {
		CAM_ERR(CAM_UTIL, "no free memory");
		return -ENOMEM;
	}

	memset(tmp_buff, 0, CAM_COMMON_ERR_INJECT_BUFFER_LEN);
	rc = strscpy(tmp_buff, kmessage, CAM_COMMON_ERR_INJECT_BUFFER_LEN);
	if (rc == -E2BIG)
		goto free;

	token_start = tmp_buff;
	token_end   = tmp_buff;

	CAM_INFO(CAM_UTIL, "parsing input param for cam_err_inject: %s", tmp_buff);
	while (token_start != NULL) {
		strsep(&token_end, ":");
		switch (param_counter) {
		case HW_NAME:
			if (strcmp(token_start, CAM_COMMON_IFE_NODE) == 0)
				err_params->hw_id = CAM_COMMON_ERR_INJECT_HW_ISP;
			else if (strcmp(token_start, CAM_COMMON_ICP_NODE) == 0)
				err_params->hw_id = CAM_COMMON_ERR_INJECT_HW_ICP;
			else if (strcmp(token_start, CAM_COMMON_JPEG_NODE) == 0)
				err_params->hw_id = CAM_COMMON_ERR_INJECT_HW_JPEG;
			else {
				CAM_ERR(CAM_UTIL, "invalid camera hardware [ %s ]", token_start);
					goto free;
			}
			break;
		case REQ_ID:
			if (kstrtou64(token_start, 0, &(err_params->req_id)))
				goto free;
			break;
		case ERR_TYPE:
			if (kstrtou32(token_start, 0, &(err_params->err_type)))
				goto free;
			break;
		case ERR_CODE:
			if (kstrtou32(token_start, 0, &(err_params->err_code)))
				goto free;
			break;
		case DEV_HDL:
			if (kstrtou64(token_start, 0, &(err_params->dev_hdl)))
				goto free;
			break;
		default:
			CAM_ERR(CAM_UTIL, "Insuffiecient parameter count [%d] ", param_counter);
			goto free;
		}

		param_counter++;
		token_start = token_end;
	}

	if (param_counter < CAM_COMMON_ERR_INJECT_PARAM_NUM) {
		CAM_ERR(CAM_UTIL, "Insuffiecient parameter count [%d]", param_counter);
		goto free;
	}

	CAM_INFO(CAM_UTIL, "parsed params: req_id: %llu err_type: %u, err_code: %u dev_hdl: %llu",
		err_params->req_id, err_params->err_type, err_params->err_code,
		err_params->dev_hdl);

	if (g_err_inject_info.err_inject_cb[err_params->hw_id]) {
		rc = g_err_inject_info.err_inject_cb[err_params->hw_id](err_params);
		if (rc)
			goto free;
		else {
			if (!g_err_inject_info.is_list_initialised) {
				INIT_LIST_HEAD(&g_err_inject_info.active_err_ctx_list);
				g_err_inject_info.is_list_initialised = true;
			}

			list_add(&err_params->list, &g_err_inject_info.active_err_ctx_list);
		}
	} else {
		CAM_ERR(CAM_UTIL, "Handler for HW_id [%d] not registered", err_params->hw_id);
		goto free;
	}

	if (rc)
		CAM_ERR(CAM_UTIL, "No Dev_hdl found: [%d]", err_params->dev_hdl);

	return rc;
free:
	kfree(err_params);
	return -EINVAL;
}

static int cam_err_inject_get(char *buffer,
	const struct kernel_param *kp)
{
	uint8_t hw_name[10];
	uint16_t buff_max_size = CAM_COMMON_ERR_MODULE_PARAM_MAX_LENGTH;
	struct cam_err_inject_param *err_param;
	int ret = 0;

	if (!g_err_inject_info.is_list_initialised)
		return scnprintf(buffer, buff_max_size, "uninitialised");

	else if (!list_empty(&g_err_inject_info.active_err_ctx_list)) {
		list_for_each_entry(err_param, &g_err_inject_info.active_err_ctx_list, list) {
			switch (err_param->hw_id) {
			case CAM_COMMON_ERR_INJECT_HW_ISP:
				strscpy(hw_name, CAM_COMMON_IFE_NODE, 10);
				break;
			case CAM_COMMON_ERR_INJECT_HW_ICP:
				strscpy(hw_name, CAM_COMMON_ICP_NODE, 10);
				break;
			case CAM_COMMON_ERR_INJECT_HW_JPEG:
				strscpy(hw_name, CAM_COMMON_JPEG_NODE, 10);
				break;
			default:
				strscpy(hw_name, "undef", 10);
			}
			ret += scnprintf(buffer+ret, buff_max_size,
				"hw_name: %s req_id: %u err_type: %u err_code: %u dev_hdl: %d\n",
				hw_name, err_param->req_id, err_param->err_type,
				err_param->err_code, err_param->dev_hdl);

			CAM_DBG(CAM_UTIL, "output buffer: %s", buffer);

			if (ret < buff_max_size) {
				buff_max_size = buff_max_size - ret;
			} else {
				CAM_WARN(CAM_UTIL, "out buff max limit reached");
				break;
			}
		}
		return ret;
	}

	return scnprintf(buffer, buff_max_size, "uninitialised");
}

static const struct kernel_param_ops cam_error_inject_ops = {
	.set = cam_err_inject_set,
	.get = cam_err_inject_get
};

module_param_cb(cam_error_inject, &cam_error_inject_ops, NULL, 0644);
