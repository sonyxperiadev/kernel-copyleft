// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/kthread.h>
#include <linux/string.h>
#include <linux/bits.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include "ipclite_test.h"

struct kobject *sysfs_dir;

static int threads_started, threads_completed, cores_completed;
static bool ssr_complete;
/* data_lock spinlock is used to increment ping counters in thread safe manner.
 * core_wq to ensure all the cores have completed the test before next step.
 * ssr_wq to wait during ssr operation.
 * reply_wq to wait on replies to ping sent.
 * thread_wq to wait on all threads local to APPS to complete
 * test_done is a completion barrier which ensures test case is completed
 * crash_done is a completion barrier which ensures ssr crash is completed
 */
DEFINE_SPINLOCK(data_lock);
DECLARE_WAIT_QUEUE_HEAD(core_wq);
DECLARE_WAIT_QUEUE_HEAD(ssr_wq);
DECLARE_WAIT_QUEUE_HEAD(reply_wq);
DECLARE_WAIT_QUEUE_HEAD(thread_wq);
DECLARE_COMPLETION(test_done);
DECLARE_COMPLETION(crash_done);

static struct ipclite_thread_data wakeup_check, bg_pings;
static struct ipclite_thread_data thread_data;

struct handle_t *handle_ptr;
static int handle_data[512];
static struct ipclite_test_data *data;

static void init_test_params(void)
{
	data->test_params.wait = 1;
	data->test_params.num_pings = 1000;
	data->test_params.num_itr = 1;
	data->test_params.selected_senders = 1;
	data->test_params.selected_receivers = 1;
	data->test_params.enabled_cores = IPCLITE_TEST_ALL_CORES;
	data->test_params.selected_test_case = 0;
	data->test_params.num_thread = 1;
	data->test_params.num_senders = 1;
	data->test_params.num_receivers = 1;
}
/* Function to pack the different fields into one 64 bit message value
 * 1 byte header of constant patter 01010101
 * 1 byte to store the parameter type
 * 1 byte to store the test case id
 * 3 bytes to store the value of parameter in payload
 * 1 byte to store test start/stop information
 * 1 byte to store test pass/fail information
 */
static uint64_t get_param_macro(uint64_t parameter_info, uint64_t test_info,
				uint64_t payload_info, uint64_t start_stop_info,
				uint64_t pass_fail_info)
{
	uint64_t param_macro = 0;

	parameter_info &= GENMASK_ULL(7, 0);
	test_info &= GENMASK_ULL(7, 0);
	payload_info &= GENMASK_ULL(23, 0);
	start_stop_info &= GENMASK_ULL(7, 0);
	pass_fail_info &= GENMASK_ULL(7, 0);

	param_macro = ((uint64_t)IPCLITE_TEST_HEADER) << 56;
	param_macro |= parameter_info << 48;
	param_macro |= test_info << 40;
	param_macro |= payload_info << 16;
	param_macro |= start_stop_info << 8;
	param_macro |= pass_fail_info;

	return param_macro;
}

static inline bool is_enabled_core(int core_id)
{
	return (data->test_params.enabled_cores & BIT(core_id)) ? true : false;
}

static inline bool is_selected_receiver(int core_id)
{
	return (data->test_params.selected_receivers & BIT(core_id)) ? true : false;
}

static inline bool is_selected_sender(int core_id)
{
	return (data->test_params.selected_senders & BIT(core_id)) ? true : false;
}

static void ping_receive(struct ipclite_test_data *data)
{
	pr_debug("Successfully received a ping\n");
	data->pings_received[data->client_id]++;
	wake_up_interruptible(&reply_wq);
}

static int check_pings(struct ipclite_test_data *data)
{
	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (!is_selected_receiver(i))
			continue;
		if (data->pings_sent[i] != data->pings_received[i])
			return -IPCLITE_TEST_FAIL;
	}
	return 0;
}

static void ping_all_enabled_cores(u64 msg)
{
	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (i == IPCMEM_APPS || !is_enabled_core(i))
			continue;
		ipclite_test_msg_send(i, msg);
	}
}

static void ping_sel_senders(uint64_t msg)
{
	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (i == IPCMEM_APPS || !(data->test_params.selected_senders & BIT(i)))
			continue;
		ipclite_test_msg_send(i, msg);
	}
}

static int thread_init(struct ipclite_thread_data *th_data, void *data_ptr, void *fptr)
{
	th_data->data = data_ptr;
	th_data->run = false;
	init_waitqueue_head(&th_data->wq);
	th_data->thread = kthread_run(fptr, th_data, "test thread");
	if (IS_ERR(th_data->thread)) {
		pr_err("Thread creation failed\n");
		return -EINVAL;
	}
	return 0;
}

static int ping_selected_receivers(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	struct ipclite_test_data *data = t_data->data;
	int ret = 0;
	uint64_t macro_to_ping = get_param_macro(TEST_CASE,
						data->test_params.selected_test_case,
						PING_SEND, 0, 0);
	bool fail = false;

	while (!kthread_should_stop()) {

		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;

		for (int i = 0; i < data->test_params.num_pings/data->test_params.num_thread; ++i) {
			for (int j = 0; j < IPCMEM_NUM_HOSTS; ++j) {
				if (!is_selected_receiver(j))
					continue;
				ret = ipclite_test_msg_send(j, macro_to_ping);
				if (ret == 0) {
					spin_lock(&data_lock);
					data->pings_sent[j]++;
					spin_unlock(&data_lock);
				} else
					fail = true;
				/* If wait is enabled and number of pings to wait on is sent,
				 * Wait for replies or timeout
				 */
				if (data->test_params.wait != 0 &&
							(i+1) % data->test_params.wait == 0) {
					ret = wait_event_interruptible_timeout(reply_wq,
								check_pings(data) == 0,
								msecs_to_jiffies(1000));
					if (ret < 1)
						pr_err("Timeout occurred\n");
				}
			}
		}
		pr_debug("Completed iteration. Marking thread as completed\n");
		spin_lock(&data_lock);
		threads_completed++;
		wake_up_interruptible(&thread_wq);
		spin_unlock(&data_lock);
	}

	return fail ? -IPCLITE_TEST_FAIL : 0;
}

static int negative_tests(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	int ret = 0, fail = 0;
	uint64_t param;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;
		pr_info("Test 1: Sending messages to disabled cores\n");
		for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
			if (!is_selected_receiver(i))
				continue;
			param = get_param_macro(TEST_CASE, NEGATIVE,
						PING_SEND, 0, 0);
			ret = ipclite_test_msg_send(i, param);
			if (ret == 0) {
				pr_err("TEST FAILED\n");
				fail++;
			}
		}
		if (!fail)
			pr_info("TEST PASSED\n");

		pr_info("Test 2: Passing NULL to get_global_parition_info\n");
		ret = get_global_partition_info(NULL);
		if (ret == 0) {
			pr_err("TEST FAILED\n");
			fail++;
		} else
			pr_info("TEST PASSED\n");

		if (fail != 0)
			pr_err("Negative TEST FAILED\n");
		else
			pr_info("Negative TEST PASSED\n");

		param = get_param_macro(TEST_CASE, NEGATIVE, 0,
					IPCLITE_TEST_STOP, 0);
		ipclite_test_msg_send(IPCMEM_APPS, param);
		wait_event_interruptible_timeout(core_wq,
						cores_completed == data->test_params.num_senders,
						msecs_to_jiffies(1000));
		complete(&test_done);
	}
	return fail == 0 ? 0 : -IPCLITE_TEST_FAIL;
}

static int hw_unlock_test(void *hw_mutex_byte)
{
	int ret = 0;
	uint64_t param;

	if (!hw_mutex_byte) {
		pr_err("Byte for hardware mutex testing is not initialized.\n");
		return -EFAULT;
	}

	pr_info("Testing HW Mutex Lock Acquire Functionality\n");
	*((int *)(hw_mutex_byte)) = -1;
	pr_debug("The initial value of the byte is %d\n", *((int *)(hw_mutex_byte)));
	pr_debug("Locking the mutex from APPS Side\n");

	ret = ipclite_hw_mutex_acquire();
	if (ret != 0) {
		pr_err("Could not acquire hw mutex from APPS side\n");
		return ret;
	}

	pr_debug("Setting the value of the byte to %d\n", IPCMEM_APPS);
	*((int *)(hw_mutex_byte)) = IPCMEM_APPS;
	pr_debug("The new value of the byte is %d\n", *((int *)(hw_mutex_byte)));

	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (i == IPCMEM_APPS || !is_selected_receiver(i))
			continue;
		pr_debug("Pinging %s to try and release the locked mutex\n",
						core_name[i]);
		param = get_param_macro(TEST_CASE, HW_MUTEX,
					HW_MUTEX_RELEASE,
					IPCLITE_TEST_START, 0);
		ipclite_test_msg_send(i, param);
		// Wait for timeout here
		udelay(1000);
	}

	if (*((int *)(hw_mutex_byte)) != IPCMEM_APPS)
		return -IPCLITE_TEST_FAIL;

	ret = ipclite_hw_mutex_release();
	if (ret != 0)
		pr_err("Could not release mutex lock successfully\n");
	return ret;
}

static int hw_mutex_test(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	struct ipclite_test_data *data = t_data->data;
	int ret = 0;
	void *addr = data->global_memory->virt_base;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;

		ret = hw_unlock_test(addr);

		if (ret == 0)
			pr_info("HW Unlock Test Passed.\n");
		else
			pr_info("HW Unlock Test Failed.\n");

		complete(&test_done);
	}
	return  ret;
}
/* Ping cores which are not selected for ssr in the background */
static int send_bg_pings(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	struct ipclite_test_data *data = t_data->data;
	int ret;
	uint64_t param;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;

		while (!ssr_complete && !kthread_should_stop()) {
			for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
				if (i == data->ssr_client || !is_selected_receiver(i))
					continue;
				param = get_param_macro(TEST_CASE,
							SSR,
							PING_SEND, 0, 0);
				ret = ipclite_test_msg_send(i, param);
				if (ret != 0)
					pr_err("Unable to ping core %d\n", i);
			}
			wait_event_interruptible_timeout(ssr_wq,
							ssr_complete,
							msecs_to_jiffies(1000));
		}
		pr_debug("SSR recovery of core %d completed. Exiting thread\n",
								data->ssr_client);
	}
	return 0;
}
/* Wait for 30s and then send pings one to by one to see if core wakeup
 *   is completed
 */
static int ssr_wakeup_check(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	struct ipclite_test_data *data = t_data->data;
	int count = 0, ret = 0;
	uint64_t param;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;

		ssr_complete = false;
		msleep_interruptible(30000);
		while (count < 10) {
			pr_debug("Sent ping number %d to check if wakeup is completed\n",
							count);
			param = get_param_macro(TEST_CASE, SSR,
						SSR_WAKEUP,
						IPCLITE_TEST_START, 0);
			ret = ipclite_test_msg_send(data->ssr_client, param);
			++count;
			wait_event_interruptible_timeout(ssr_wq,
							ssr_complete,
							msecs_to_jiffies(1000));
		}
		if (count == 10 && !ssr_complete) {
			pr_info("FW Core wakeup failed.\n");
			return -IPCLITE_TEST_FAIL;
		}
		pr_info("FW Core wakeup completed successfully.\n");
		pr_info("Going for non crashing testing.\n");
		param = get_param_macro(TEST_CASE, PING, 0,
					IPCLITE_TEST_START, 0);
		ipclite_test_msg_send(data->ssr_client, param);
		complete(&crash_done);
	}
	return 0;
}

static int ssr_test(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	struct ipclite_test_data *data = t_data->data;
	uint64_t param = 0;
	int ret = 0;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;

		ssr_complete = false;
		ret = thread_init(&wakeup_check, data, ssr_wakeup_check);

		if (ret != 0) {
			pr_err("Thread creation failed\n");
			return -EINVAL;
		}

		ret = thread_init(&bg_pings, data, send_bg_pings);
		if (ret != 0) {
			pr_err("Thread creation failed\n");
			kthread_stop(wakeup_check.thread);
			return -EINVAL;
		}
		pr_info("Starting on SSR test for core %d\n", data->ssr_client);
		memset(data->pings_sent, 0, sizeof(data->pings_sent));
		memset(data->pings_received, 0, sizeof(data->pings_received));
		param = get_param_macro(TEST_CASE, SSR,
					SSR_CRASHING, IPCLITE_TEST_START, 0);
		ipclite_test_msg_send(data->ssr_client, param);
		wait_for_completion(&crash_done);
		kthread_stop(wakeup_check.thread);
		kthread_stop(bg_pings.thread);
		complete(&test_done);
	}
	return 0;
}

static int inc_byte(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	ipclite_atomic_uint32_t *addr = t_data->data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;
		for (int i = 0; i < data->test_params.num_itr; ++i)
			ipclite_global_atomic_inc(addr);
		threads_completed++;
		wake_up_interruptible(&thread_wq);
	}
	return 0;
}

static int dec_byte(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	ipclite_atomic_uint32_t *addr = t_data->data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;
		for (int i = 0; i < data->test_params.num_itr; ++i)
			ipclite_global_atomic_dec(addr);
		threads_completed++;
		wake_up_interruptible(&thread_wq);
	}
	return 0;
}

static int global_atomics_test(void *byte, int test_number)
{
	int ret = 0;
	int total_increment = 0;
	uint64_t param;
	bool fail = false;
	struct ipclite_thread_data ga_t1, ga_t2;


	if (!byte) {
		pr_err("Byte not initialized. Test Failed\n");
		return -EFAULT;
	}
	pr_debug("The initial value of the byte is %x\n", *((int *)byte));

	threads_completed = 0;
	threads_started = 0;

	switch (test_number) {
	case GLOBAL_ATOMICS_INC:
		ret = thread_init(&ga_t1, byte, inc_byte);
		if (ret != 0) {
			pr_err("Thread creation failed\n");
			return -EINVAL;
		}
		ret = thread_init(&ga_t2, byte, inc_byte);
		if (ret != 0) {
			pr_err("Thread creation failed\n");
			kthread_stop(ga_t1.thread);
			return -EINVAL;
		}
		break;
	case GLOBAL_ATOMICS_DEC:
		ret = thread_init(&ga_t1, byte, dec_byte);
		if (ret != 0) {
			pr_err("Thread creation failed\n");
			return -EINVAL;
		}
		ret = thread_init(&ga_t2, byte, dec_byte);
		if (ret != 0) {
			pr_err("Thread creation failed\n");
			kthread_stop(ga_t1.thread);
			return -EINVAL;
		}
		break;
	case GLOBAL_ATOMICS_INC_DEC:
		ret = thread_init(&ga_t1, byte, inc_byte);
		if (ret != 0) {
			pr_err("Thread creation failed\n");
			return -EINVAL;
		}
		ret = thread_init(&ga_t2, byte, dec_byte);
		if (ret != 0) {
			pr_err("Thread creation failed\n");
			kthread_stop(ga_t1.thread);
			return -EINVAL;
		}
		break;
	default:
		pr_err("Wrong input provided\n");
		return -EINVAL;
	}
	param = get_param_macro(TEST_CASE,
				GLOBAL_ATOMIC,
				test_number,
				IPCLITE_TEST_START, 0);

	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (i == IPCMEM_APPS || !is_selected_receiver(i))
			continue;
		ret = ipclite_test_msg_send(i, param);
		if (ret == 0)
			threads_started += 2;
	}
	if (is_selected_receiver(IPCMEM_APPS)) {
		ga_t1.run = true;
		wake_up_interruptible(&ga_t1.wq);
		ga_t2.run = true;
		wake_up_interruptible(&ga_t2.wq);
		threads_started += 2;
	}
	/* Wait for all threads to complete or timeout */
	ret = wait_event_interruptible_timeout(thread_wq,
					threads_started == 2 * data->test_params.num_receivers &&
					threads_completed == 2 * data->test_params.num_receivers,
					msecs_to_jiffies(1000));
	if (ret < 1)
		pr_err("Threads could not complete successfully\n");

	pr_debug("The value of the byte is %x\n", *((int *)byte));
	/* Stopping threads if they have not already completed before evaluation */
	kthread_stop(ga_t1.thread);
	kthread_stop(ga_t2.thread);

	total_increment = 2 * data->test_params.num_receivers * data->test_params.num_itr;

	switch (test_number) {
	case GLOBAL_ATOMICS_INC:
		if (*((int *)byte) == total_increment)
			pr_info("Increment Successful.\n");
		else {
			pr_err("Increment Failed.\n");
			fail = true;
		}
		break;
	case GLOBAL_ATOMICS_DEC:
		if (*((int *)byte) == 0)
			pr_info("Decrement Successful\n");
		else {
			pr_err("Decrement Failed\n");
			fail = true;
		}
		break;
	case GLOBAL_ATOMICS_INC_DEC:
		if (*((int *)byte) == 0)
			pr_info("Increment and Decrement Successful\n");
		else {
			pr_err("Increment and Decrement Failed\n");
			fail = true;
		}
		break;
	default:
		pr_err("Wrong input provided\n");
		return -EINVAL;
	}

	return fail ? -IPCLITE_TEST_FAIL : 0;
}

static inline uint32_t bitops_count_trailing_one(uint32_t x)
{
	uint32_t mask = 0;

	for (int i = 0; i < BITS(ipclite_atomic_uint32_t); i++) {
		mask = 1 << i;
		if (!(x & mask))
			return i;
	}
	return BITS(ipclite_atomic_uint32_t);
}

/**
 * @brief Finds the first zero in the bitmap
 *
 * @param bmap_addr pointer to bitmap
 * @param size the size of the bitmap indicated in number of bits
 * @return uint32_t index of the first zero
 */
static uint32_t bitops_util_find_first_zero(uint32_t *bmap_addr, uint32_t size)
{
	uint32_t res = 0;

	for (int i = 0; i * BITS(ipclite_atomic_uint32_t) < size; i++) {
		if (bmap_addr[i] != ~(uint32_t)0) {
			res = i * BITS(ipclite_atomic_uint32_t) +
				bitops_count_trailing_one(bmap_addr[i]);
			return res < size ? res : size;
		}
	}
	return size;
}

static int alloc_index(int *bitmap_base)
{
	uint32_t prev = 0, index = 0;

	do {
		index = bitops_util_find_first_zero((unsigned int *) bitmap_base,
							NUM_HANDLES);
		if (index > NUM_HANDLES) {
			pr_err("No Memory Error. Exiting\n");
			break;
		}
		prev = ipclite_global_test_and_set_bit(index % 32,
					(ipclite_atomic_uint32_t *)(bitmap_base + index/32));
		if ((prev & (1UL << (index % 32))) == 0)
			break;
	} while (true);
	return index;
}

void clear_index(int *bitmap_base, uint32_t index)
{
	uint32_t addr_idx = index/32, ii = index % 32;

	if (bitmap_base == NULL) {
		pr_err("Invalid pointer passed\n");
		return;
	}
	ipclite_global_test_and_clear_bit(ii, (ipclite_atomic_uint32_t *)(bitmap_base + addr_idx));
}

static int global_atomics_test_set_clear(struct ipclite_test_data *data)
{
	int index = 0, ret = 0;
	bool fail = false;
	uint64_t param;

	handle_ptr = data->global_memory->virt_base;
	pr_info("Starting global atomics Test 4. Starting allocation of index\n");
	pr_debug("The total number of handles is %d\n", NUM_HANDLES);
	pr_debug("Global Base : %p\n", handle_ptr);
	for (int itr = 0; itr < data->test_params.num_itr; itr++) {
		threads_started = 0;
		threads_completed = 0;
		for (int j = 0; j < IPCMEM_NUM_HOSTS; ++j) {
			if (j == IPCMEM_APPS || !is_selected_receiver(j))
				continue;
			param = get_param_macro(TEST_CASE,
						GLOBAL_ATOMIC,
						GLOBAL_ATOMICS_SET_CLR,
						IPCLITE_TEST_START, 0);
			ret = ipclite_test_msg_send(j, param);
			if (ret == 0)
				threads_started++;
		}
		if (is_selected_receiver(IPCMEM_APPS)) {
			threads_started++;
			for (int i = 0; i < 512; ++i) {
				index = alloc_index((int *)handle_ptr);
				handle_data[i] = index;
				handle_ptr->handle_data[index] = IPCMEM_APPS;
			}

			for (int i = 0; i < 512; ++i) {
				index = handle_data[i];
				if (handle_ptr->handle_data[index] != IPCMEM_APPS) {
					pr_err("Handle data has been overwritten.\n");
					pr_err("This is a bug : Core : %d Index : %d\n",
						handle_ptr->handle_data[index], index);
					fail = true;
				}
			}

			for (int i = 0; i < 512; ++i) {
				index = handle_data[i];
				clear_index((int *)handle_ptr, index);
			}
			threads_completed++;
			if (fail)
				break;
		}
		wait_event_interruptible_timeout(thread_wq,
				threads_started == data->test_params.num_receivers &&
				threads_completed == data->test_params.num_receivers,
				msecs_to_jiffies(1000));
	}
	if (!fail)
		pr_info("Global Atomics Set and Clear test passed successfully\n");
	return fail ? -IPCLITE_TEST_FAIL  : 0;
}

static int global_atomics_test_wrapper(void *data_ptr)
{
	int result = 0, ret = 0;
	struct ipclite_thread_data *t_data = data_ptr;
	struct ipclite_test_data *data = t_data->data;
	void *addr = data->global_memory->virt_base;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;
		*((int *)addr) = 0;
		result = global_atomics_test(addr, GLOBAL_ATOMICS_INC);
		result &= global_atomics_test(addr, GLOBAL_ATOMICS_DEC);
		result &= global_atomics_test(addr, GLOBAL_ATOMICS_INC_DEC);
		result &= global_atomics_test_set_clear(data);
		if (result != 0) {
			pr_err("Global Atomics TEST FAILED\n");
			ret = -IPCLITE_TEST_FAIL;
		} else {
			pr_info("Global Atomics TEST PASSED\n");
			ret = 0;
		}
		complete(&test_done);
	}
	return ret;
}

static int ping_test(void *data_ptr)
{
	int ret = 0;
	uint64_t param_macro;
	struct ipclite_test_data *data = data_ptr;
	struct ipclite_thread_data th_arr[IPCLITE_TEST_MAX_THREADS];
	int count;

	memset(data->pings_sent, 0, sizeof(data->pings_sent));
	memset(data->pings_received, 0, sizeof(data->pings_received));
	threads_completed = 0;
	param_macro = 0;
	for (count = 0; count < data->test_params.num_thread; ++count) {
		ret = thread_init(&th_arr[count], data, ping_selected_receivers);
		if (ret != 0)
			break;
	}
	if (count != data->test_params.num_thread)
		while (count > 0) {
			kthread_stop(th_arr[count-1].thread);
			--count;
		}
	if (ret != 0) {
		pr_err("Threads could not be initialized. Ping Test Failed\n");
		return ret;
	}
	for (threads_started = 0; threads_started < data->test_params.num_thread;
							++threads_started) {
		th_arr[threads_started].run = true;
		wake_up_interruptible(&th_arr[threads_started].wq);
	}
	ret = wait_event_interruptible_timeout(thread_wq,
				threads_started == data->test_params.num_thread &&
				threads_completed == data->test_params.num_thread,
				msecs_to_jiffies(1000) * data->test_params.num_thread);
	if (ret < 1) {
		pr_err("Threads not completed successfully. Only completed %d threads\n",
						threads_completed);
		return ret;

	}
	pr_info("All threads completed successfully.\n");
	pr_debug("Going for checking\n");
	/*Wait for the queue to get processed before checking if all replies are received*/
	if (!data->test_params.wait)
		msleep_interruptible(1000);
	ret = check_pings(data);

	if (ret == 0)
		pr_debug("All replies received successfully.\n");
	else
		pr_debug("All replies not received successfully.\n");

	while (count > 0) {
		kthread_stop(th_arr[count-1].thread);
		--count;
	}
	param_macro = get_param_macro(TEST_CASE, PING, 0,
					IPCLITE_TEST_STOP, 0);
	ipclite_test_msg_send(IPCMEM_APPS, param_macro);
	return ret;
}

static int wrapper_ping_test(void *data_ptr)
{
	int ret = 0;
	uint64_t param_macro;
	struct ipclite_thread_data *t_data = data_ptr;
	struct ipclite_test_data *data = t_data->data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;

		for (int i = 0; i < data->test_params.num_itr; ++i) {
			cores_completed = 0;
			param_macro = get_param_macro(TEST_CASE,
							PING,
							0, IPCLITE_TEST_START, 0);
			/* Ping all senders to start sending messages.
			 *  If APPS is one of the senders start sending
			 */
			ping_sel_senders(param_macro);
			if (is_selected_sender(IPCMEM_APPS))
				ping_test(data);
			wait_event_interruptible_timeout(core_wq,
						cores_completed == data->test_params.num_senders,
						msecs_to_jiffies(1000));
			ret = check_pings(data);
			if (ret != 0)
				pr_info("Iteration %d of ping test failed\n", i+1);
			else
				pr_info("Iteration %d of ping test passed\n", i+1);
		}
	if (is_selected_sender(IPCMEM_APPS))
		complete(&test_done);
	}
	return 0;
}

static int debug_tests(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	uint64_t param;
	int disabled_core = ffz(data->test_params.enabled_cores);

	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;
		param = get_param_macro(TEST_CASE, DEBUG,
					PING_SEND, 0, 0);
		if (disabled_core == IPCMEM_NUM_HOSTS)
			pr_err("All cores are enabled. No Disabled cores\n");
		/* Pinging one enabled and disabled cores to get the error and dbg prints */
		if (disabled_core < IPCMEM_NUM_HOSTS)
			ipclite_test_msg_send(disabled_core, param);

		param = get_param_macro(TEST_CASE, PING, 0,
						IPCLITE_TEST_STOP, 0);
		ipclite_test_msg_send(IPCMEM_APPS, param);
		wait_event_interruptible_timeout(core_wq,
						cores_completed == data->test_params.num_senders,
						msecs_to_jiffies(1000));
		complete(&test_done);
	}
	return 0;
}

static void ipclite_test_set_enabled_cores(void)
{
	if (data->test_params.enabled_cores < 0 ||
					data->test_params.enabled_cores > IPCLITE_TEST_ALL_CORES) {
		pr_err("Invalid parameter value given to enabled cores\n");
		data->test_params.enabled_cores = IPCLITE_TEST_ALL_CORES;
		return;
	}
	pr_info("Enabled cores set to %d\n", data->test_params.enabled_cores);
}

static void ipclite_test_set_wait(void)
{
	uint64_t param;

	if (data->test_params.wait < 0) {
		pr_err("Invalid parameter value given to wait\n");
		data->test_params.wait = 1;
		return;
	}

	pr_info("wait set to %d\n", data->test_params.wait);

	param = get_param_macro(WAIT, 0, data->test_params.wait, 0, 0);
	ping_all_enabled_cores(param);
}

static void ipclite_test_set_num_pings(void)
{
	uint64_t param;

	pr_info("num_pings set to %d\n", data->test_params.num_pings);

	param = get_param_macro(NUM_PINGS, 0,
				data->test_params.num_pings, 0, 0);
	ping_all_enabled_cores(param);
}

static void ipclite_test_set_num_itr(void)
{
	uint64_t param;

	pr_info("num_itr set to %d\n", data->test_params.num_itr);

	param = get_param_macro(NUM_ITR, 1,
				data->test_params.num_itr, 0, 0);
	ping_all_enabled_cores(param);
}

static void ipclite_test_set_receivers(void)
{
	uint64_t param;

	if (data->test_params.selected_receivers < 0 ||
		data->test_params.selected_receivers > IPCLITE_TEST_ALL_CORES) {
		pr_err("Invalid parameter value given to selected_receivers\n");
		data->test_params.selected_receivers = 1;
		data->test_params.num_receivers = 1;
		return;
	}
	/* Check number of 1s using hamming weight function.
	 * Number of 1s is number of receivers
	 */
	data->test_params.num_receivers = hweight_long(data->test_params.selected_receivers);

	pr_info("selected_receivers set to %d\n", data->test_params.selected_receivers);

	param = get_param_macro(RECEIVER_LIST, 0,
				data->test_params.selected_receivers, 0, 0);
	ping_all_enabled_cores(param);
}

static void ipclite_test_set_senders(void)
{
	if (data->test_params.selected_senders < 0 ||
		data->test_params.selected_senders > IPCLITE_TEST_ALL_CORES) {
		pr_err("Invalid parameter value given to selected_senders\n");
		data->test_params.selected_senders = 1;
		data->test_params.num_senders = 1;
		return;
	}

	/* Check number of 1s using hamming weight function. */
	data->test_params.num_senders = hweight_long(data->test_params.selected_senders);

	pr_info("selected_senders set to %d\n", data->test_params.selected_senders);
}

static void ipclite_test_set_num_threads(void)
{
	uint64_t param;

	if (data->test_params.num_thread < 0 ||
					data->test_params.num_thread > IPCLITE_TEST_MAX_THREADS) {
		pr_err("Invalid parameter value given to num_thread\n");
		data->test_params.num_thread = 1;
		return;
	}

	pr_info("num_thread set to %d\n", data->test_params.num_thread);

	param = get_param_macro(NUM_THREADS, 0,
				data->test_params.num_thread, 0, 0);
	ping_all_enabled_cores(param);
}

static void ipclite_test_set_test(void)
{
	uint64_t param;
	int ret = 0;

	if (data->test_params.selected_test_case < 0 || data->test_params.selected_test_case > 8) {
		pr_err("Invalid parameter value given to test_case\n");
		data->test_params.selected_test_case = 0;
		return;
	}

	pr_info("selected_test_case set to %d\n", data->test_params.selected_test_case);
	param = get_param_macro(TEST_CASE,
				data->test_params.selected_test_case, 0,
				IPCLITE_TEST_START, 0);

	switch (data->test_params.selected_test_case) {
	case PING:
		ret = thread_init(&thread_data, data, wrapper_ping_test);
		if (ret != 0) {
			pr_err("Could not create thread for testing\n");
			return;
		}
		thread_data.run = true;
		wake_up_interruptible(&thread_data.wq);
		break;
	case NEGATIVE:
		ping_sel_senders(param);
		if (is_selected_sender(IPCMEM_APPS)) {
			pr_info("Starting test %d for core %s\n",
				NEGATIVE, core_name[IPCMEM_APPS]);
			ret = thread_init(&thread_data, data, negative_tests);
			if (ret != 0) {
				pr_err("Could not create thread for testing\n");
				return;
			}
			thread_data.run = true;
			wake_up_interruptible(&thread_data.wq);
		}
		break;
	case GLOBAL_ATOMIC:
		ret = thread_init(&thread_data, data, global_atomics_test_wrapper);
		if (ret != 0) {
			pr_err("Could not create thread for testing\n");
			return;
		}
		thread_data.run = true;
		wake_up_interruptible(&thread_data.wq);
		break;
	case DEBUG:
		ping_sel_senders(param);
		if (is_selected_sender(IPCMEM_APPS)) {
			ret = thread_init(&thread_data, data, debug_tests);
			if (ret != 0) {
				pr_err("Could not create thread for testing\n");
				return;
			}
			thread_data.run = true;
			wake_up_interruptible(&thread_data.wq);
		}
		break;
	case SSR:
		if (data->test_params.num_senders != 1) {
			pr_err("SSR Testing requires only 1 core to be selected\n");
			return;
		}
		/* Find first set (ffs) to get the bit position/index of sender */
		data->ssr_client = ffs(data->test_params.selected_senders) - 1;
		if (data->ssr_client == 0 || !is_enabled_core(data->ssr_client)) {
			pr_err("Invalid core selected for SSR Testing\n");
			return;
		}
		pr_info("Starting test %d for core %s\n",
			SSR, core_name[data->ssr_client]);
		ret = thread_init(&thread_data, data, ssr_test);
		if (ret != 0) {
			pr_err("Could not create thread for testing\n");
			return;
		}
		thread_data.run = true;
		wake_up_interruptible(&thread_data.wq);
		break;
	case HW_MUTEX:
		if (data->test_params.num_senders != 1) {
			pr_err("HW Mutex Testing requires only 1 core to be selected\n");
			return;
		}

		if (is_selected_sender(IPCMEM_APPS)) {
			pr_info("Starting test %d for core %s\n",
				HW_MUTEX, core_name[IPCMEM_APPS]);
			ret = thread_init(&thread_data, data, hw_mutex_test);
			if (ret != 0) {
				pr_err("Could not create thread for testing\n");
				return;
			}
			thread_data.run = true;
			wake_up_interruptible(&thread_data.wq);
		} else
			ping_sel_senders(param);
		break;
	default:
		pr_err("Wrong input provided\n");
		return;
	}
	wait_for_completion(&test_done);
	if (thread_data.thread != NULL)
		ret = kthread_stop(thread_data.thread);
	if (ret != 0)
		pr_err("Test did not complete successfully\n");
	else
		pr_info("Test completed successfully\n");
}

static int parse_param(char **temp_buf, int *addr)
{
	char *token;
	int ret;

	token = strsep(temp_buf, " ");
	if (!token) {
		pr_err("Token value is NULL in parse param\n");
		return -EINVAL;
	}
	ret = kstrtoint(token, 0, addr);
	if (ret < 0) {
		pr_err("Parameter value not read correctly\n");
		return ret;
	}
	return 0;
}

static ssize_t ipclite_test_params_write(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	char *temp_buf = kmalloc(strlen(buf)+1, GFP_KERNEL);
	char *temp_ptr = temp_buf;
	int ret, param = 0;

	if (!temp_buf) {
		pr_err("Memory not allocated\n");
		return -EINVAL;
	}

	ret = strscpy(temp_buf, buf, strlen(buf)+1);

	if (ret < 0) {
		pr_err("User input is too large\n");
		goto exit;
	}

	ret = parse_param(&temp_buf, &param);
	if (ret != 0)
		goto exit;

	if (param  == ENABLED_CORES) {
		ret = parse_param(&temp_buf, &data->test_params.enabled_cores);
		if (ret == 0)
			ipclite_test_set_enabled_cores();
		goto exit;
	} else
		data->test_params.selected_test_case = param;

	switch (data->test_params.selected_test_case) {
	case PING:
		ret = parse_param(&temp_buf, &data->test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &data->test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		ret = parse_param(&temp_buf, &data->test_params.num_pings);
		if (ret != 0)
			break;
		ipclite_test_set_num_pings();
		ret = parse_param(&temp_buf, &data->test_params.wait);
		if (ret != 0)
			break;
		ipclite_test_set_wait();
		ret = parse_param(&temp_buf, &data->test_params.num_itr);
		if (ret != 0)
			break;
		ipclite_test_set_num_itr();
		ret = parse_param(&temp_buf, &data->test_params.num_thread);
		if (ret != 0)
			break;
		ipclite_test_set_num_threads();
		break;
	case NEGATIVE:
		ret = parse_param(&temp_buf, &data->test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &data->test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		break;
	case GLOBAL_ATOMIC:
		ret = parse_param(&temp_buf, &data->test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		ret = parse_param(&temp_buf, &data->test_params.num_itr);
		if (ret != 0)
			break;
		ipclite_test_set_num_itr();
		break;
	case DEBUG:
		ret = parse_param(&temp_buf, &data->test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		break;
	case SSR:
		ret = parse_param(&temp_buf, &data->test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &data->test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		ret = parse_param(&temp_buf, &data->test_params.num_pings);
		if (ret != 0)
			break;
		ipclite_test_set_num_pings();
		break;
	case HW_MUTEX:
		ret = parse_param(&temp_buf, &data->test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &data->test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		break;
	default:
		pr_err("Wrong input provided\n");
		goto exit;
	}
	if (ret == 0)
		ipclite_test_set_test();
exit:
	kfree(temp_ptr);
	return count;
}



static int ipclite_test_callback_fn(unsigned int client_id, long long msg,
					void *data_ptr)
{
	struct ipclite_test_data *data = data_ptr;
	uint64_t header, parameter_info, test_info, payload_info,
			start_stop_info, pass_fail_info;
	uint64_t reply_macro;
	int ret = 0;

	/* Unpack the different bit fields from message value */
	header = (msg & GENMASK(63, 56))>>56;
	parameter_info = (msg & GENMASK(55, 48))>>48;
	test_info = (msg & GENMASK(47, 40))>>40;
	payload_info = (msg & GENMASK(39, 16))>>16;
	start_stop_info = (msg & GENMASK(15, 8))>>8;
	pass_fail_info = (msg & GENMASK(7, 0));

	if (!data) {
		pr_err("Callback data pointer not loaded successfully\n");
		return -EFAULT;
	}

	data->client_id = client_id;

	if (header != IPCLITE_TEST_HEADER) {
		pr_err("Corrupted message packed received\n");
		return -EINVAL;
	}

	pr_debug("The message received is %lx\n", msg);

	switch (test_info) {
	case PING:
	case NEGATIVE:
	case DEBUG:
		if (payload_info == PING_SEND) {
			reply_macro = get_param_macro(TEST_CASE,
							test_info,
							PING_REPLY,
							0, 0);
			ipclite_test_msg_send(client_id, reply_macro);
			break;
		}
		if (payload_info == PING_REPLY) {
			ping_receive(data);
			break;
		}
		if (pass_fail_info == IPCLITE_TEST_PASS)
			pr_info("Test passed on core %s\n", core_name[client_id]);
		else if (pass_fail_info == IPCLITE_TEST_FAIL)
			pr_info("Test failed on core %s\n", core_name[client_id]);
		if (start_stop_info == IPCLITE_TEST_STOP) {
			++cores_completed;
			if (cores_completed == data->test_params.num_senders)
				pr_info("Test completed on all cores\n");
			if (is_selected_sender(IPCMEM_APPS))
				wake_up_interruptible(&core_wq);
			else
				complete(&test_done);
		}
		break;
	case HW_MUTEX:
		if (start_stop_info == IPCLITE_TEST_START) {
			ret = ipclite_hw_mutex_release();
			if (ret == 0)
				*((int *)data->global_memory->virt_base) = IPCMEM_APPS;
			reply_macro = get_param_macro(TEST_CASE,
							test_info,
							HW_MUTEX_RELEASE,
							IPCLITE_TEST_STOP, 0);
			ipclite_test_msg_send(client_id, reply_macro);

		}
		if (pass_fail_info == IPCLITE_TEST_PASS)
			pr_info("HW Unlock Test passed on core %s\n",
					core_name[client_id]);
		else if (pass_fail_info == IPCLITE_TEST_FAIL)
			pr_info("HW Unlock Test failed on core %s\n",
					core_name[client_id]);
		if (start_stop_info == IPCLITE_TEST_STOP)
			complete(&test_done);
		break;
	case SSR:
		if (payload_info == PING_SEND) {
			reply_macro = get_param_macro(TEST_CASE,
							test_info,
							PING_REPLY,
							0, 0);
			data->pings_received[client_id]++;
			ipclite_test_msg_send(client_id, reply_macro);
			if (data->pings_received[client_id] == data->test_params.num_pings) {
				pr_info("Waking up ssr_wakeup_check_thread.\n");
				pr_info("Signaling other cores to make sure there is no other crash\n");
				wakeup_check.run = true;
				wake_up_interruptible(&wakeup_check.wq);
				bg_pings.run = true;
				wake_up_interruptible(&bg_pings.wq);
			}
		}
		if (payload_info == SSR_WAKEUP) {
			if (start_stop_info == IPCLITE_TEST_STOP) {
				ssr_complete = true;
				pr_info("%s wakeup completed\n",
						core_name[client_id]);
				wake_up_interruptible(&ssr_wq);
			}
		}
		if (pass_fail_info == IPCLITE_TEST_PASS)
			pr_info("Test %d passed on core %s\n",
						test_info, core_name[client_id]);
		else if (pass_fail_info == IPCLITE_TEST_FAIL)
			pr_info("Test %d failed on core %s\n",
					test_info, core_name[client_id]);
		break;
	case GLOBAL_ATOMIC:
		if (start_stop_info == IPCLITE_TEST_STOP) {
			pr_debug("%s completed Global Atomics Test.\n",
						core_name[client_id]);
			if (payload_info == GLOBAL_ATOMICS_SET_CLR)
				threads_completed++;
			else
				threads_completed += 2;
			wake_up_interruptible(&thread_wq);
		}
		break;
	default:
		pr_info("Wrong input given\n");
	}
	return 0;
}

struct kobj_attribute ipclite_test_params = __ATTR(ipclite_test_params,
							0660,
							NULL,
							ipclite_test_params_write);

static int ipclite_test_sysfs_node_setup(void)
{
	int ret = 0;

	sysfs_dir = kobject_create_and_add("ipclite_test", kernel_kobj);
	if (sysfs_dir == NULL) {
		pr_err("Cannot create sysfs directory\n");
		return -ENOENT;
	}

	ret = sysfs_create_file(sysfs_dir, &ipclite_test_params.attr);
	if (ret) {
		pr_err("Cannot create sysfs file for ipclite test module. Error - %d\n",
			ret);
		return -ENOENT;
	}
	return 0;
}

static int __init ipclite_test_init(void)
{
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->global_memory = kzalloc(sizeof(*(data->global_memory)),
						GFP_KERNEL);
	if (!data->global_memory) {
		kfree(data);
		data = NULL;
		return -ENOMEM;
	}
	ret = get_global_partition_info(data->global_memory);
	if (ret != 0) {
		pr_err("Unable to load global partition information\n");
		goto bail;
	}

	ret = ipclite_register_test_client(ipclite_test_callback_fn, data);
	if (ret != 0) {
		pr_err("Could not register client\n");
		goto bail;
	}

	ret = ipclite_test_sysfs_node_setup();
	if (ret != 0) {
		pr_err("Failed to create sysfs interface\n");
		goto bail;
	}

	init_test_params();
	return 0;
bail:
	kfree(data->global_memory);
	kfree(data);
	data = NULL;
	return ret;
}

static void __exit ipclite_test_exit(void)
{
	pr_info("Removing IPCLite Test Module\n");
	sysfs_remove_file(sysfs_dir, &ipclite_test_params.attr);
	kobject_put(sysfs_dir);
	kfree(data->global_memory);
	kfree(data);
	data = NULL;
}

module_init(ipclite_test_init);
module_exit(ipclite_test_exit);

MODULE_LICENSE("GPL v2");
