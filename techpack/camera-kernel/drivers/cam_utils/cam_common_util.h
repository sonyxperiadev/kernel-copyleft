/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CAM_COMMON_UTIL_H_
#define _CAM_COMMON_UTIL_H_

#include <linux/types.h>
#include <linux/kernel.h>

#define CAM_BITS_MASK_SHIFT(x, mask, shift) (((x) & (mask)) >> shift)
#define CAM_36BIT_INTF_GET_IOVA_BASE(iova) ((iova) >> 8)
#define CAM_36BIT_INTF_GET_IOVA_OFFSET(iova) ((iova) & 0xff)

#define CAM_COMMON_MINI_DUMP_DEV_NUM      6
#define CAM_COMMON_MINI_DUMP_DEV_NAME_LEN 16
#define CAM_COMMON_MINI_DUMP_SIZE         10 * 1024 * 1024

#define CAM_COMMON_HW_DUMP_TAG_MAX_LEN 64
#define CAM_MAX_NUM_CCI_PAYLOAD_BYTES     11

#define CAM_COMMON_ERR_MODULE_PARAM_MAX_LENGTH  4096
#define CAM_COMMON_ERR_INJECT_BUFFER_LEN  200
#define CAM_COMMON_ERR_INJECT_DEV_MAX     5
#define CAM_COMMON_ERR_INJECT_PARAM_NUM   5
#define CAM_COMMON_IFE_NODE "IFE"
#define CAM_COMMON_ICP_NODE "IPE"
#define CAM_COMMON_JPEG_NODE "JPEG"

#define CAM_COMMON_NS_PER_MS              1000000ULL

#define PTR_TO_U64(ptr) ((uint64_t)(uintptr_t)ptr)
#define U64_TO_PTR(ptr) ((void *)(uintptr_t)ptr)

#define CAM_TRIGGER_PANIC(format, args...) panic("CAMERA - " format "\n", ##args)

#define CAM_GET_TIMESTAMP(timestamp) ktime_get_real_ts64(&(timestamp))
#define CAM_GET_TIMESTAMP_DIFF_IN_MICRO(ts_start, ts_end, diff_microsec)                     \
({                                                                                           \
	diff_microsec = 0;                                                                   \
	if (ts_end.tv_nsec >= ts_start.tv_nsec) {                                            \
		diff_microsec =                                                              \
			(ts_end.tv_nsec - ts_start.tv_nsec) / 1000;                          \
		diff_microsec +=                                                             \
			(ts_end.tv_sec - ts_start.tv_sec) * 1000 * 1000;                     \
	} else {                                                                             \
		diff_microsec =                                                              \
			(ts_end.tv_nsec +                                                    \
			(1000*1000*1000 - ts_start.tv_nsec)) / 1000;                         \
		diff_microsec +=                                                             \
			(ts_end.tv_sec - ts_start.tv_sec - 1) * 1000 * 1000;                 \
	}                                                                                    \
})

#define CAM_CONVERT_TIMESTAMP_FORMAT(ts, hrs, min, sec, ms)                                  \
({                                                                                           \
	uint64_t tmp = ((ts).tv_sec);                                                        \
	(ms) = ((ts).tv_nsec) / 1000000;                                                     \
	(sec) = do_div(tmp, 60);                                                             \
	(min) = do_div(tmp, 60);                                                             \
	(hrs) = do_div(tmp, 24);                                                             \
})

#define CAM_COMMON_WAIT_FOR_COMPLETION_TIMEOUT_ERRMSG(complete, timeout_jiffies, module_id,  \
	fmt, args...)                                                                        \
({                                                                                           \
	struct timespec64 start_time, end_time;                                              \
	unsigned long rem_jiffies;                                                           \
	start_time = ktime_to_timespec64(ktime_get());                                       \
	rem_jiffies = cam_common_wait_for_completion_timeout((complete), (timeout_jiffies)); \
	if (!rem_jiffies) {                                                                  \
		end_time = ktime_to_timespec64(ktime_get());                                 \
		CAM_ERR(module_id, fmt " (timeout: %ums start: %llu.%llu end: %llu.%llu)",   \
		##args, jiffies_to_msecs(timeout_jiffies),                                   \
		start_time.tv_sec, (start_time.tv_nsec/NSEC_PER_USEC),                       \
		end_time.tv_sec, (end_time.tv_nsec/NSEC_PER_USEC));                          \
	}                                                                                    \
	rem_jiffies;                                                                         \
})

typedef unsigned long (*cam_common_mini_dump_cb) (void *dst, unsigned long len);

/**
 * struct cam_common_mini_dump_dev_info
 * @dump_cb       : address of data dumped
 * @name          : Name of driver
 * @num_devs      : Number of device registerd
 * @is_registered : Bool to indicate if registered
 */
struct cam_common_mini_dump_dev_info {
	cam_common_mini_dump_cb  dump_cb[CAM_COMMON_MINI_DUMP_DEV_NUM];
	uint8_t                  name[CAM_COMMON_MINI_DUMP_DEV_NUM]
				    [CAM_COMMON_MINI_DUMP_DEV_NAME_LEN];
	uint8_t                  num_devs;
	bool                     is_registered;
};

/**
 * struct cam_common_mini_dump_data
 * @link         : address of data dumped
 * @name         : Name of driver
 * @size         : Size dumped
 */
struct cam_common_mini_dump_data {
	void          *waddr[CAM_COMMON_MINI_DUMP_DEV_NUM];
	uint8_t        name[CAM_COMMON_MINI_DUMP_DEV_NUM][CAM_COMMON_MINI_DUMP_DEV_NAME_LEN];
	unsigned long  size[CAM_COMMON_MINI_DUMP_DEV_NUM];
};


typedef int (*cam_common_err_inject_cb) (void *err_param);
int cam_common_release_err_params(uint64_t dev_hdl);

enum cam_common_err_inject_hw_id {
	CAM_COMMON_ERR_INJECT_HW_ISP,
	CAM_COMMON_ERR_INJECT_HW_ICP,
	CAM_COMMON_ERR_INJECT_HW_JPEG,
	CAM_COMMON_ERR_INJECT_HW_MAX
};

enum cam_common_err_inject_input_param_pos {
	HW_NAME = 0,
	REQ_ID,
	ERR_TYPE,
	ERR_CODE,
	DEV_HDL
};

/**
 * @req_id  : req id for err to be injected
 * @dev_hdl : dev_hdl for the context
 * @err_type: error type for error request
 * @err_code: error code for error request
 * @hw_id   : hw id representing hw nodes of type cam_common_err_inject_hw_id
 */
struct cam_err_inject_param {
	struct list_head  list;
	uint64_t          req_id;
	uint64_t          dev_hdl;
	uint32_t          err_type;
	uint32_t          err_code;
	uint8_t           hw_id;
};
/**
 * struct cam_common_err_inject_info
 * @err_inject_cb      : address of callback
 * @active_err_ctx_list: list containing active err inject requests
 * @num_hw_registered  : number of callbacks registered
 * @is_list_initialised: bool to check init for err_inject list
 */
struct cam_common_err_inject_info {
	cam_common_err_inject_cb    err_inject_cb[CAM_COMMON_ERR_INJECT_HW_MAX];
	struct list_head            active_err_ctx_list;
	uint8_t                     num_hw_registered;
	bool                        is_list_initialised;
};

/**
 * struct cam_common_hw_dump_args
 * @req_id         : request id
 * @cpu_addr       : address where dumping will start from
 * @buf_len        : length of buffer where data is being dumped to
 * @offset         : buffer offset from cpu_addr after each item dump
 * @ctxt_to_hw_map : context to hw map
 * @is_dump_all    : flag to indicate if all information or just bw/clk rate
 * @
 */
struct cam_common_hw_dump_args {
	uint64_t                req_id;
	uintptr_t               cpu_addr;
	size_t                  buf_len;
	size_t                  offset;
	void                   *ctxt_to_hw_map;
	bool                    is_dump_all;
};

/**
 * struct cam_common_hw_dump_header
 * @tag        : string used by the parser to call parse functions
 * @size       : size of the header in the buffer
 * @word_size  : word size of the header
 * @
 */
struct cam_common_hw_dump_header {
	uint8_t   tag[CAM_COMMON_HW_DUMP_TAG_MAX_LEN];
	uint64_t  size;
	uint32_t  word_size;
};

/**
 * cam_common_util_get_string_index()
 *
 * @brief                  Match the string from list of strings to return
 *                         matching index
 *
 * @strings:               Pointer to list of strings
 * @num_strings:           Number of strings in 'strings'
 * @matching_string:       String to match
 * @index:                 Pointer to index to return matching index
 *
 * @return:                0 for success
 *                         -EINVAL for Fail
 */
int cam_common_util_get_string_index(const char **strings,
	uint32_t num_strings, const char *matching_string, uint32_t *index);

/**
 * cam_common_util_remove_duplicate_arr()
 *
 * @brief                  Move all the unique integers to the start of
 *                         the array and return the number of unique integers
 *
 * @array:                 Pointer to the first integer of array
 * @num:                   Number of elements in array
 *
 * @return:                Number of unique integers in array
 */
uint32_t cam_common_util_remove_duplicate_arr(int32_t *array,
	uint32_t num);

/**
 * cam_common_wait_for_completion_timeout()
 *
 * @brief                  common interface to implement wait for completion
 *                         for slow environment like presil, single debug
 *                         timeout variable can take care
 *
 * @complete:              Pointer to the first integer of array
 * @timeout_jiffies:       Timeout value in jiffie
 *
 * @return:                Remaining jiffies, non-zero for success, zero
 *                         in case of failure
 */
unsigned long cam_common_wait_for_completion_timeout(
	struct completion   *complete,
	unsigned long        timeout_jiffies);
/**
 * cam_common_read_poll_timeout()
 *
 * @brief                  common interface to read poll timeout
 *
 * @addr:                  Address of IO register
 * @delay:                 Delay interval of poll
 * @timeout:               Timeout for poll
 * @mask:                  Mask to be checked
 * @check_val:             Value to be compared to break poll
 * @status:                Status of register of IO
 *
 * @return:                0 if success and negative if fail
 * */
int cam_common_read_poll_timeout(
	void __iomem        *addr,
	unsigned long        delay,
	unsigned long        timeout,
	uint32_t             mask,
	uint32_t             check_val,
	uint32_t            *status);

/**
 * cam_common_modify_timer()
 *
 * @brief                  common interface to modify timer,
 *
 * @timer:                 reference to system timer
 * @timeout_val:           timeout value for timer
 *
 * @return:                0 if success and negative if fail
 */
int cam_common_modify_timer(struct timer_list *timer, int32_t timeout_val);

/**
 * cam_common_util_thread_switch_delay_detect()
 *
 * @brief                  Detect if there is any scheduling delay
 *
 * @token:                 String identifier to print workq name or tasklet
 * @scheduled_time:        Time when workq or tasklet was scheduled
 * @threshold:             Threshold time
 *
 */
void cam_common_util_thread_switch_delay_detect(const char *token,
	ktime_t scheduled_time, uint32_t threshold);

/**
 * cam_common_register_mini_dump_cb()
 *
 * @brief                  common interface to register mini dump cb
 *
 * @mini_dump_cb:          Pointer to the mini_dump_cb
 * @name:                  name of device registering
 *
 * @return:                0 if success in register non-zero if failes
 */
#if IS_REACHABLE(CONFIG_QCOM_VA_MINIDUMP)
int cam_common_register_mini_dump_cb(
	cam_common_mini_dump_cb mini_dump_cb, uint8_t *name);
#else
static inline int cam_common_register_mini_dump_cb(
	cam_common_mini_dump_cb mini_dump_cb,
	uint8_t *dev_name)
{
	return 0;
}
#endif

/**
 * cam_common_user_dump_clock()
 *
 * @brief                  Handles clock rate dump
 *
 * @dump_struct:           Struct holding dump info
 * @addr_ptr:              Pointer to buffer address pointer
 */
void *cam_common_user_dump_clock(
	void     *dump_struct,
	uint8_t  *addr_ptr);

/**
 * cam_common_user_dump_helper()
 *
 * @brief                  Handles buffer addressing and dumping for user dump
 *
 * @cmd_args:              Holds cam_common_hw_dump_args pointer
 * @func:                  Function pointer for dump function
 * @dump_struct:           Struct holding dump info
 * @size:                  Size_t value used for header word size
 * @tag:                   Tag for header, used by parser
 * @...:                   Variadic arguments, appended to tag if given
 */
int cam_common_user_dump_helper(
	void        *cmd_args,
	void        *(*func)(void *, uint8_t *),
	void        *dump_struct,
	size_t       size,
	const char  *tag,
	...);

/**
 * cam_common_register_err_inject_cb()
 *
 * @brief                  common interface to register error inject cb
 *
 * @err_inject_cb:         Pointer to err_inject_cb
 * @hw_id:                 HW id of the HW driver registering
 *
 * @return:                0 if success in register non-zero if failes
 */
int cam_common_register_err_inject_cb(
	cam_common_err_inject_cb err_inject_cb,
	enum cam_common_err_inject_hw_id hw_id);

#endif /* _CAM_COMMON_UTIL_H_ */
