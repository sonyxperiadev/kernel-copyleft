// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
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
static struct cam_common_mini_dump_dev_info g_minidump_dev_info;
#endif

#define CAM_PRESIL_POLL_DELAY 20

static struct cam_common_inject_evt_info g_inject_evt_info;

static uint timeout_multiplier = 1;
module_param(timeout_multiplier, uint, 0644);
typedef int (*cam_common_evt_inject_cmd_parse_handler)(
	struct cam_common_inject_evt_param *inject_params,
	uint32_t param_counter, char *token);

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

void cam_common_util_thread_switch_delay_detect(char *wq_name, const char *state,
	void *cb, ktime_t scheduled_time, uint32_t threshold)
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
			"%s cb: %ps delay in %s detected %ld:%06ld cur %ld:%06ld\n"
			"diff %ld: threshold %d",
			wq_name, cb, state, scheduled_ts.tv_sec,
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
			(void *)waddr, remain_len, g_minidump_dev_info.priv_data[i]);
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
	uint8_t *dev_name, void *priv_data)
{
	int rc = 0;
	uint32_t idx;

	if (g_minidump_dev_info.num_devs >= CAM_COMMON_MINI_DUMP_DEV_NUM) {
		CAM_ERR(CAM_UTIL, "No free index available");
		return -EINVAL;
	}

	if (!mini_dump_cb || !dev_name) {
		CAM_ERR(CAM_UTIL, "Invalid params");
		return -EINVAL;
	}

	idx = g_minidump_dev_info.num_devs;
	g_minidump_dev_info.dump_cb[idx] =
		mini_dump_cb;
	scnprintf(g_minidump_dev_info.name[idx],
		CAM_COMMON_MINI_DUMP_DEV_NAME_LEN, dev_name);
	g_minidump_dev_info.priv_data[idx] = priv_data;
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
	*addr++ = cam_soc_util_get_applied_src_clk(&hw_info->soc_info, true);

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

int cam_common_register_evt_inject_cb(cam_common_evt_inject_cb evt_inject_cb,
	enum cam_common_evt_inject_hw_id hw_id)
{
	int rc = 0;

	if (g_inject_evt_info.num_hw_registered >= CAM_COMMON_EVT_INJECT_HW_MAX) {
		CAM_ERR(CAM_UTIL, "No free index available");
		return -EINVAL;
	}

	if (!evt_inject_cb || hw_id >= CAM_COMMON_EVT_INJECT_HW_MAX) {
		CAM_ERR(CAM_UTIL, "Invalid params evt_inject_cb %s hw_id: %d",
			CAM_IS_NULL_TO_STR(evt_inject_cb), hw_id);
		return -EINVAL;
	}

	g_inject_evt_info.evt_inject_cb[hw_id] = evt_inject_cb;
	g_inject_evt_info.num_hw_registered++;
	CAM_DBG(CAM_UTIL, "Evt inject cb registered for HW_id: %d, total registered: %d", hw_id,
		g_inject_evt_info.num_hw_registered);
	return rc;
}

void cam_common_release_evt_params(int32_t dev_hdl)
{
	struct list_head *pos = NULL, *pos_next = NULL;
	struct cam_common_inject_evt_param *inject_params;

	if (!g_inject_evt_info.is_list_initialised)
		return;

	if (list_empty(&g_inject_evt_info.active_evt_ctx_list)) {
		CAM_DBG(CAM_UTIL, "Event injection list is initialized but empty");
		return;
	}

	list_for_each_safe(pos, pos_next, &g_inject_evt_info.active_evt_ctx_list) {
		inject_params = list_entry(pos, struct cam_common_inject_evt_param, list);
		if (inject_params->dev_hdl == dev_hdl) {
			CAM_INFO(CAM_UTIL, "entry deleted for %d dev hdl", dev_hdl);
			list_del(pos);
			kfree(inject_params);
		}
	}
}

static inline int cam_common_evt_inject_get_hw_id(uint8_t *hw_id, char *token)
{
	if (strcmp(token, CAM_COMMON_IFE_NODE) == 0)
		*hw_id = CAM_COMMON_EVT_INJECT_HW_ISP;
	else if (strcmp(token, CAM_COMMON_ICP_NODE) == 0)
		*hw_id = CAM_COMMON_EVT_INJECT_HW_ICP;
	else if (strcmp(token, CAM_COMMON_JPEG_NODE) == 0)
		*hw_id = CAM_COMMON_EVT_INJECT_HW_JPEG;
	else {
		CAM_ERR(CAM_UTIL, "Invalid camera hardware [ %s ]", token);
		return -EINVAL;
	}

	return 0;
}

static inline int cam_common_evt_inject_get_str_id_type(uint8_t *id_type, char *token)
{
	if (!strcmp(token, CAM_COMMON_EVT_INJECT_BUFFER_ERROR))
		*id_type = CAM_COMMON_EVT_INJECT_BUFFER_ERROR_TYPE;
	else if (!strcmp(token, CAM_COMMON_EVT_INJECT_NOTIFY_EVENT))
		*id_type = CAM_COMMON_EVT_INJECT_NOTIFY_EVENT_TYPE;
	else {
		CAM_ERR(CAM_UTIL, "Invalid string id: %s", token);
		return -EINVAL;
	}

	return 0;
}

static int cam_common_evt_inject_parse_buffer_error_evt_params(
	struct cam_common_inject_evt_param *inject_params,
	uint32_t param_counter, char *token)
{
	struct cam_hw_inject_buffer_error_param *buf_err_params =
		&inject_params->evt_params.u.buf_err_evt;
	int rc = 0;

	switch (param_counter) {
	case SYNC_ERROR_CAUSE:
		if (kstrtou32(token, 0, &buf_err_params->sync_error)) {
			CAM_ERR(CAM_UTIL, "Invalid event type %s", token);
			rc = -EINVAL;
		}
		break;
	default:
		CAM_ERR(CAM_UTIL, "Invalid extra parameters: %s", token);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_common_evt_inject_parse_node_evt_params(
	struct cam_common_inject_evt_param *inject_params,
	uint32_t param_counter, char *token)
{
	struct cam_hw_inject_node_evt_param *node_params =
		&inject_params->evt_params.u.evt_notify.u.node_evt_params;
	int rc = 0;

	switch (param_counter) {
	case EVENT_TYPE:
		if (kstrtou32(token, 0, &node_params->event_type)) {
			CAM_ERR(CAM_UTIL, "Invalid event type %s", token);
			rc = -EINVAL;
		}
		break;
	case EVENT_CAUSE:
		if (kstrtou32(token, 0, &node_params->event_cause)) {
			CAM_ERR(CAM_UTIL, "Invalid event cause %s", token);
			rc = -EINVAL;
		}
		break;
	default:
		CAM_ERR(CAM_UTIL, "Invalid extra parameters: %s", token);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_common_evt_inject_parse_pf_params(
	struct cam_common_inject_evt_param *inject_params,
	uint32_t param_counter, char *token)
{
	struct cam_hw_inject_pf_evt_param *pf_params =
		&inject_params->evt_params.u.evt_notify.u.pf_evt_params;
	int rc = 0;

	switch (param_counter) {
	case PF_PARAM_CTX_FOUND:
		if (kstrtobool(token, &pf_params->ctx_found)) {
			CAM_ERR(CAM_UTIL, "Invalid context found value %s", token);
			rc = -EINVAL;
		}
		break;
	default:
		CAM_ERR(CAM_UTIL, "Invalid extra parameters %s", token);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_common_evt_inject_parse_err_evt_params(
	struct cam_common_inject_evt_param *inject_params,
	uint32_t param_counter, char *token)
{
	struct cam_hw_inject_err_evt_param *err_params =
		&inject_params->evt_params.u.evt_notify.u.err_evt_params;
	int rc = 0;

	switch (param_counter) {
	case ERR_PARAM_ERR_TYPE:
		if (kstrtou32(token, 0, &err_params->err_type)) {
			CAM_ERR(CAM_UTIL, "Invalid error type %s", token);
			rc = -EINVAL;
		}
		break;
	case ERR_PARAM_ERR_CODE:
		if (kstrtou32(token, 0, &err_params->err_code)) {
			CAM_ERR(CAM_UTIL, "Invalid error code %s", token);
			rc = -EINVAL;
		}
		break;
	default:
		CAM_ERR(CAM_UTIL, "Invalid extra parameters: %s", token);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_common_evt_inject_parse_event_notify(
	struct cam_common_inject_evt_param *inject_params,
	uint32_t param_counter, char *token)
{
	int rc = 0;

	switch (param_counter) {
	case EVT_NOTIFY_TYPE:
		if (kstrtou32(token, 0,
			&inject_params->evt_params.u.evt_notify.evt_notify_type)) {
			CAM_ERR(CAM_UTIL, "Invalid Event notify type %s", token);
			rc = -EINVAL;
		}
		break;
	default:
		CAM_ERR(CAM_UTIL, "Invalid extra parameters: %s", token);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_common_evt_inject_parse_common_params(
	struct cam_common_inject_evt_param *inject_params,
	uint32_t param_counter, char *token)
{
	int rc = 0;
	struct cam_hw_inject_evt_param *evt_param = &inject_params->evt_params;

	switch (param_counter) {
	case STRING_ID:
		rc = cam_common_evt_inject_get_str_id_type(&evt_param->inject_id, token);
		break;
	case HW_NAME:
		rc = cam_common_evt_inject_get_hw_id(&inject_params->hw_id, token);
		break;
	case DEV_HDL:
		if (kstrtos32(token, 0, &inject_params->dev_hdl)) {
			CAM_ERR(CAM_UTIL, "Invalid device handle %s", token);
			rc = -EINVAL;
		}
		break;
	case REQ_ID:
		if (kstrtou64(token, 0, &evt_param->req_id)) {
			CAM_ERR(CAM_UTIL, "Invalid request id %s", token);
			rc = -EINVAL;
		}
		break;
	default:
		 CAM_ERR(CAM_UTIL, "Invalid extra parameter: %s", token);
		 rc = -EINVAL;
	}

	return rc;
}

static int cam_common_evt_inject_generic_command_parser(
	struct cam_common_inject_evt_param *inject_params,
	char **msg, uint32_t max_params, cam_common_evt_inject_cmd_parse_handler cmd_parse_cb)
{
	char *token = NULL;
	int rc = 0, param_counter = 0;

	token = strsep(msg, ":");
	while (token != NULL) {
		rc = cmd_parse_cb(inject_params, param_counter, token);
		if (rc) {
			CAM_ERR(CAM_UTIL, "Parsed Command failed rc: %d", rc);
			return rc;
		}

		param_counter++;
		if (param_counter == max_params)
			break;
		token = strsep(msg, ":");
	}

	if (param_counter < max_params) {
		CAM_ERR(CAM_UTIL,
			"Insufficient parameters passed for total parameters: %u",
			param_counter);
		return -EINVAL;
	}

	return rc;
}

static int cam_common_evt_inject_set(const char *kmessage,
	const struct kernel_param *kp)
{
	struct   cam_common_inject_evt_param *inject_params   = NULL;
	struct   cam_hw_inject_evt_param *hw_evt_params       = NULL;
	cam_common_evt_inject_cmd_parse_handler parse_handler = NULL;
	int      rc                                           = 0;
	char     tmp_buff[CAM_COMMON_EVT_INJECT_BUFFER_LEN];
	char    *msg                                          = NULL;
	uint32_t param_output                                 = 0;

	inject_params = kzalloc(sizeof(struct cam_common_inject_evt_param), GFP_KERNEL);
	if (!inject_params) {
		CAM_ERR(CAM_UTIL, "no free memory");
		return -ENOMEM;
	}

	rc = strscpy(tmp_buff, kmessage, CAM_COMMON_EVT_INJECT_BUFFER_LEN);
	if (rc == -E2BIG)
		goto free;

	CAM_INFO(CAM_UTIL, "parsing input param for cam event injection: %s", tmp_buff);

	msg = tmp_buff;
	hw_evt_params = &inject_params->evt_params;

	rc = cam_common_evt_inject_generic_command_parser(inject_params, &msg,
		COMMON_PARAM_MAX, cam_common_evt_inject_parse_common_params);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Fail to parse common params %d", rc);
		goto free;
	}

	switch (hw_evt_params->inject_id) {
	case CAM_COMMON_EVT_INJECT_NOTIFY_EVENT_TYPE:
		rc = cam_common_evt_inject_generic_command_parser(inject_params, &msg,
			EVT_NOTIFY_PARAM_MAX, cam_common_evt_inject_parse_event_notify);
		if (rc) {
			CAM_ERR(CAM_UTIL, "Fail to parse event notify type param %d", rc);
			goto free;
		}

		switch (hw_evt_params->u.evt_notify.evt_notify_type) {
		case V4L_EVENT_CAM_REQ_MGR_ERROR:
			parse_handler = cam_common_evt_inject_parse_err_evt_params;
			param_output = ERR_PARAM_MAX;
			break;
		case V4L_EVENT_CAM_REQ_MGR_NODE_EVENT:
			parse_handler = cam_common_evt_inject_parse_node_evt_params;
			param_output = NODE_PARAM_MAX;
			break;
		case V4L_EVENT_CAM_REQ_MGR_PF_ERROR:
			parse_handler = cam_common_evt_inject_parse_pf_params;
			param_output = PF_PARAM_MAX;
			break;
		default:
			CAM_ERR(CAM_UTIL, "Invalid event notification type: %u",
				hw_evt_params->u.evt_notify.evt_notify_type);
			goto free;
		}
		break;
	case CAM_COMMON_EVT_INJECT_BUFFER_ERROR_TYPE:
		parse_handler = cam_common_evt_inject_parse_buffer_error_evt_params;
		param_output = BUFFER_ERROR_PARAM_MAX;
		break;
	default:
		CAM_ERR(CAM_UTIL, "Invalid Injection id: %u", hw_evt_params->inject_id);
	}

	rc = cam_common_evt_inject_generic_command_parser(inject_params, &msg,
		param_output, parse_handler);
	if (rc) {
		CAM_ERR(CAM_UTIL, "Command Parsed failed with Inject id: %u rc: %d",
			hw_evt_params->inject_id, rc);
		goto free;
	}

	if (g_inject_evt_info.evt_inject_cb[inject_params->hw_id]) {
		rc = g_inject_evt_info.evt_inject_cb[inject_params->hw_id](inject_params);
		if (rc)
			goto free;
	} else {
		CAM_ERR(CAM_UTIL, "Handler for HW_id [%hhu] not registered", inject_params->hw_id);
		goto free;
	}

	if (!g_inject_evt_info.is_list_initialised) {
		INIT_LIST_HEAD(&g_inject_evt_info.active_evt_ctx_list);
		g_inject_evt_info.is_list_initialised = true;
	}

	list_add(&inject_params->list, &g_inject_evt_info.active_evt_ctx_list);

	return rc;

free:
	kfree(inject_params);
	return rc;
}

static int cam_common_evt_inject_get(char *buffer,
	const struct kernel_param *kp)
{
	uint8_t hw_name[16], string_id[16];
	uint16_t buff_max_size = CAM_COMMON_EVT_INJECT_MODULE_PARAM_MAX_LENGTH;
	struct cam_common_inject_evt_param *inject_params = NULL;
	struct cam_hw_inject_evt_param *evt_params = NULL;
	uint32_t  ret = 0;

	if (!g_inject_evt_info.is_list_initialised)
		return scnprintf(buffer, buff_max_size, "uninitialised");

	if (list_empty(&g_inject_evt_info.active_evt_ctx_list))
		return scnprintf(buffer, buff_max_size, "Active err inject list is empty");

	list_for_each_entry(inject_params, &g_inject_evt_info.active_evt_ctx_list, list) {
		evt_params = &inject_params->evt_params;

		switch (inject_params->hw_id) {
		case CAM_COMMON_EVT_INJECT_HW_ISP:
			strscpy(hw_name, CAM_COMMON_IFE_NODE, sizeof(hw_name));
			break;
		case CAM_COMMON_EVT_INJECT_HW_ICP:
			strscpy(hw_name, CAM_COMMON_ICP_NODE, sizeof(hw_name));
			break;
		case CAM_COMMON_EVT_INJECT_HW_JPEG:
			strscpy(hw_name, CAM_COMMON_JPEG_NODE, sizeof(hw_name));
			break;
		default:
			ret += scnprintf(buffer+ret, buff_max_size, "Undefined HW id\n");
			goto undefined_param;
		}

		switch (evt_params->inject_id) {
		case CAM_COMMON_EVT_INJECT_BUFFER_ERROR_TYPE:
			strscpy(string_id, CAM_COMMON_EVT_INJECT_BUFFER_ERROR, sizeof(string_id));
			break;
		case CAM_COMMON_EVT_INJECT_NOTIFY_EVENT_TYPE:
			strscpy(string_id, CAM_COMMON_EVT_INJECT_NOTIFY_EVENT, sizeof(string_id));
			break;
		default:
			ret += scnprintf(buffer+ret, buff_max_size, "Undefined string id\n");
			goto undefined_param;
		}

		ret += scnprintf(buffer+ret, buff_max_size,
			"string_id: %s hw_name: %s dev_hdl: %d req_id: %llu ",
			string_id, hw_name,
			inject_params->dev_hdl, evt_params->req_id);

		if (buff_max_size > ret) {
			buff_max_size -= ret;
		} else {
			CAM_WARN(CAM_UTIL, "out buff max limit reached");
			break;
		}

		if (evt_params->inject_id ==
			CAM_COMMON_EVT_INJECT_BUFFER_ERROR_TYPE) {
			ret += scnprintf(buffer+ret, buff_max_size,
				"sync_error: %u\n", evt_params->u.buf_err_evt.sync_error);
		} else {
			switch (evt_params->u.evt_notify.evt_notify_type) {
			case V4L_EVENT_CAM_REQ_MGR_ERROR: {
				struct cam_hw_inject_err_evt_param *err_evt_params =
					&evt_params->u.evt_notify.u.err_evt_params;
				ret += scnprintf(buffer+ret, buff_max_size,
					"Error event: error type: %u error code: %u\n",
					err_evt_params->err_type, err_evt_params->err_code);
				break;
			}
			case V4L_EVENT_CAM_REQ_MGR_NODE_EVENT: {
				struct cam_hw_inject_node_evt_param *node_evt_params =
					&evt_params->u.evt_notify.u.node_evt_params;
				ret += scnprintf(buffer+ret, buff_max_size,
					"Node event: event type: %u event cause: %u\n",
					node_evt_params->event_type, node_evt_params->event_cause);
				break;
			}
			case V4L_EVENT_CAM_REQ_MGR_PF_ERROR: {
				struct cam_hw_inject_pf_evt_param *pf_evt_params =
					&evt_params->u.evt_notify.u.pf_evt_params;
				ret += scnprintf(buffer+ret, buff_max_size,
					"PF event: ctx found %hhu\n",
					pf_evt_params->ctx_found);
				break;
			}
			default:
				ret += scnprintf(buffer+ret, buff_max_size,
					"Undefined notification event\n");
			}
		}

undefined_param:
		CAM_DBG(CAM_UTIL, "output buffer: %s", buffer);

		if (buff_max_size > ret) {
			buff_max_size -= ret;
		} else {
			CAM_WARN(CAM_UTIL, "out buff max limit reached");
			break;
		}
	}

	return ret;
}

static const struct kernel_param_ops cam_common_evt_inject = {
	.set = cam_common_evt_inject_set,
	.get = cam_common_evt_inject_get
};

module_param_cb(cam_event_inject, &cam_common_evt_inject, NULL, 0644);
