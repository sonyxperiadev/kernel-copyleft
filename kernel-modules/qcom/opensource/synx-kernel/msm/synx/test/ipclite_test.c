// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/kthread.h>
#include <linux/string.h>
#include <linux/bits.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include "ipclite_test.h"

struct kobject *sysfs_dir;

static int threads_completed, cores_completed;
static unsigned int pingsend_fail;
static bool corestatus;
/* thread_wq to wait on all threads local to APPS to complete
 * test_done is a completion barrier which ensures test case is completed
 */
DECLARE_WAIT_QUEUE_HEAD(thread_wq);
DECLARE_COMPLETION(test_done);

static struct ipclite_thread_data m_thread, wakeup_check, bg_pings;
static struct ipclite_thread_data *th_arr;
static struct global_region_info global_memory;
static int ssr_client;
static struct ipclite_test_params test_params;

static void init_test_params(void)
{
	test_params.wait = 1;
	test_params.num_pings = 1000;
	test_params.num_itr = 1;
	test_params.selected_senders = 1;
	test_params.selected_receivers = 1;
	test_params.selected_test_case = 0;
	test_params.num_thread = 1;
	test_params.num_senders = 1;
	test_params.num_receivers = 1;
}
/* Function to pack the different fields into one 64 bit message value
 * 1 byte header of constant patter 01010101
 * 1 byte to store the parameter type
 * 1 byte to store the test case id
 * 4 bits to store the thread id
 * 20 bits to store the value of parameter in payload
 * 1 byte to store test start/stop information
 * 1 byte to store test pass/fail information
 */
static uint64_t get_test_macro(uint64_t test_info,
				uint64_t thread_count, uint64_t payload_info,
				uint64_t start_stop_info, uint64_t pass_fail_info)
{
	uint64_t param_macro = 0, parameter_info = TEST_CASE;

	param_macro = (uint64_t)IPCLITE_TEST_HEADER << 56;
	param_macro |= (parameter_info & GENMASK_ULL(7, 0)) << 48;
	param_macro |= (test_info & GENMASK_ULL(7, 0)) << 40;
	param_macro |= (thread_count & GENMASK_ULL(3, 0)) << 36;
	param_macro |= (payload_info & GENMASK_ULL(19, 0)) << 16;
	param_macro |= (start_stop_info & GENMASK_ULL(7, 0)) << 8;
	param_macro |= (pass_fail_info & GENMASK_ULL(7, 0));

	return param_macro;
}
/* Function to pack the different fields into one 64 bit message value
 * 1 byte to store the wait
 * 1 byte to store the parameter_info
 * 20 bits to store the number of pings
 * 12 bits to store the number of iterations
 * 12 bits to store the selected receivers
 * 4 bits to store the numbers of threads
 */
static uint64_t get_param_macro(void)
{
	uint64_t param_macro = 0, parameter_info = PARAM;

	param_macro = (test_params.wait & GENMASK_ULL(7, 0)) << 56;
	param_macro |= (parameter_info & GENMASK_ULL(7, 0)) << 48;
	param_macro |= (test_params.num_pings & GENMASK_ULL(19, 0)) << 28;
	param_macro |= (test_params.num_itr & GENMASK_ULL(11, 0)) << 16;
	param_macro |= (test_params.selected_receivers & GENMASK_ULL(11, 0)) << 4;
	param_macro |= (test_params.num_thread & GENMASK_ULL(3, 0));
	return param_macro;
}

static inline bool is_enabled_core(int core_id)
{
	return (test_params.enabled_cores & BIT(core_id)) ? true : false;
}

static inline bool is_selected_receiver(int core_id)
{
	return (test_params.selected_receivers & BIT(core_id)) ? true : false;
}

static inline bool is_selected_sender(int core_id)
{
	return (test_params.selected_senders & BIT(core_id)) ? true : false;
}

static void ping_receive(void *data, int client_id)
{
	int pings;
	struct ipclite_thread_data *th_data = data;

	pr_debug("Successfully received a ping\n");
	th_data->pings_received[client_id]++;
	pings = th_data->pings_received[client_id];
	if (test_params.wait == 0)
		return;
	if (pings % test_params.wait == 0 || pings == th_data->num_pings)
		wake_up_interruptible(&th_data->wq);
}

static int check_pings(void)
{
	bool fail = false;
	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (!is_selected_receiver(i))
			continue;
		for (int id = 0; id < test_params.num_thread; id++) {
			if (th_arr[id].pings_received[i] != th_arr[id].num_pings) {
				pr_err("fail: host :%d, thread :%d, send:%d, receive:%d\n",
				i, id, th_arr[id].pings_sent[i], th_arr[id].pings_received[i]);
				fail = true;
			} else
				pr_debug("pass: host :%d, thread :%d, send:%d, receive:%d\n",
				i, id, th_arr[id].pings_sent[i], th_arr[id].pings_received[i]);
		}
	}
	return fail ? -IPCLITE_TEST_FAIL : 0;
}

static void ping_sel_senders(uint64_t msg)
{
	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (i == IPCMEM_APPS || !is_selected_sender(i))
			continue;
		ipclite_test_msg_send(i, msg);
	}
}

static int thread_init(struct ipclite_thread_data *th_data, void *fptr)
{
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
	int ret = 0;
	bool fail = false;
	uint64_t macro_to_ping = get_test_macro(test_params.selected_test_case,
						t_data->t_id, PING_SEND, 0, 0);
	t_data->num_pings = test_params.num_pings/test_params.num_thread;
	if (t_data->t_id == 0)
		t_data->num_pings += test_params.num_pings%test_params.num_thread;
	while (!kthread_should_stop()) {
		wait_event_interruptible(t_data->wq, t_data->run);
		if (kthread_should_stop())
			break;
		t_data->run = false;
		for (int i = 0; i < t_data->num_pings; ++i) {
			for (int host = 0; host < IPCMEM_NUM_HOSTS; ++host) {
				if (!is_selected_receiver(host))
					continue;
retry_ping:
				ret = ipclite_test_msg_send(host, macro_to_ping);
				if (ret == 0)
					t_data->pings_sent[host]++;
				else if (ret == -EAGAIN) {
					msleep_interruptible(RETRY_DELAY);
					goto retry_ping;
				} else {
					fail = true;
					break;
				}
				/* If wait is enabled and number of pings to wait on is sent,
				 * Wait for replies or timeout
				 */
				if (test_params.wait == 0)
					continue;
				if ((i+1) % test_params.wait == 0 ||
								(i+1 == t_data->num_pings)) {
					ret = wait_event_interruptible_timeout(t_data->wq,
					t_data->pings_sent[host] == t_data->pings_received[host],
					msecs_to_jiffies((SEC_DELAY * 2) + test_params.wait * 500));
					if (ret < 1) {
						pr_err("Timeout occurred\n");
						fail = true;
						break;
					}
				}
			}
			if (fail)
				break;
		}
		pr_debug("Completed iteration. Marking thread as completed\n");
		ipclite_global_atomic_inc((ipclite_atomic_int32_t *) &threads_completed);
		wake_up_interruptible(&thread_wq);
	}
	return 0;
}

static int negative_tests(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	int ret = 0;
	bool fail = false;
	uint64_t macro;

	wait_event_interruptible(t_data->wq, t_data->run);
	if (is_selected_sender(IPCMEM_APPS)) {
		pr_info("Test 1: Sending messages to disabled cores\n");
		macro = get_test_macro(NEGATIVE, 0, PING_SEND, 0, 0);
		for (int host = 0; host < IPCMEM_NUM_HOSTS; ++host) {
			if (!is_selected_receiver(host))
				continue;
			ret = ipclite_test_msg_send(host, macro);
			if (ret == 0) {
				pr_err("Disabled core %d test failed\n", host);
				fail = true;
			}
		}

		pr_info("Test 2: Passing NULL to get_global_parition_info\n");
		ret = get_global_partition_info(NULL);
		if (ret == 0) {
			pr_err("Passing NULL test failed\n");
			fail = true;
		}
		if (!fail)
			pr_info("Negative test - pass\n");
		++cores_completed;
	}
	ret = wait_event_interruptible_timeout(t_data->wq,
					cores_completed == test_params.num_senders,
						msecs_to_jiffies(SEC_DELAY));
	if (ret < 1)
		pr_err("Timeout - other cores not completed\n");
	else
		pr_info("Test completed on all cores\n");
	complete(&test_done);
	return fail ? -IPCLITE_TEST_FAIL : 0;
}

static int hw_unlock_test(void *hw_mutex_byte, void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	int ret = 0;
	uint64_t macro;

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

	macro = get_test_macro(HW_MUTEX, 0, HW_MUTEX_RELEASE,
					IPCLITE_TEST_START, 0);
	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (i == IPCMEM_APPS || !is_selected_receiver(i))
			continue;
		t_data->run = false;
		pr_debug("Pinging %s to try and release the locked mutex\n",
						core_name[i]);
		ipclite_test_msg_send(i, macro);
		// Wait for timeout here
		mdelay(100);
		if (t_data->run == false)
			pr_debug("Timeout\n");

		if (*((int *)(hw_mutex_byte)) != IPCMEM_APPS) {
			pr_err("hw_mutex acquired by %s core\n", core_name[i]);
			return -IPCLITE_TEST_FAIL;
		}
	}
	ret = ipclite_hw_mutex_release();
	if (ret != 0)
		pr_err("Could not release mutex lock successfully\n");
	return ret;
}

static int hw_mutex_test(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	int ret = 0;
	uint64_t macro = get_test_macro(HW_MUTEX, 0,
					0, IPCLITE_TEST_START, 0);

	wait_event_interruptible(t_data->wq, t_data->run);
	if (is_selected_sender(IPCMEM_APPS)) {
		ret = hw_unlock_test(global_memory.virt_base, data_ptr);
		if (ret != 0)
			pr_err("HW Unlock test failed.\n");
	} else {
		t_data->run = false;
		ping_sel_senders(macro);
	}

	ret = wait_event_interruptible_timeout(t_data->wq,
					t_data->run, msecs_to_jiffies(SEC_DELAY));
	if (ret < 1)
		pr_err("Timeout - other core not completed\n");
	else
		pr_info("Test completed on all cores\n");

	complete(&test_done);
	return  ret;
}
/* Ping cores which are not selected for ssr in the background */
static int send_bg_pings(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	int ret, failed_hosts[IPCMEM_NUM_HOSTS] = {0};
	uint64_t macro = get_test_macro(SSR, 0, PING_SEND, 0, 0);

	wait_event_interruptible(t_data->wq, t_data->run);
	while (!kthread_should_stop()) {
		t_data->run = false;
		for (int host = 0; host < IPCMEM_NUM_HOSTS; ++host) {
			if (host == IPCMEM_APPS ||
				!is_selected_receiver(host) || failed_hosts[host])
				continue;
			ret = ipclite_test_msg_send(host, macro);
			if (ret != 0) {
				pr_err("Unable to ping core %d\n", host);
				failed_hosts[host] = 1;
				continue;
			}
			ret = wait_event_interruptible_timeout(t_data->wq,
						t_data->run, msecs_to_jiffies(SEC_DELAY));
			if (kthread_should_stop())
				break;
			if (ret < 1)
				pr_err("Timeout - waiting for %d core\n", host);
		}
	}
	pr_debug("SSR recovery of core %d completed. Exiting thread\n",
							ssr_client);
	return 0;
}
/* Wait for 30s and then send pings one to by one to see if core wakeup
 *   is completed
 */
static int ssr_wakeup_check(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	int count = 0, ret = 0;
	uint64_t macro = get_test_macro(SSR, 0, SSR_WAKEUP, IPCLITE_TEST_START, 0);

	wait_event_interruptible(t_data->wq, t_data->run);
	t_data->run = false;
	msleep_interruptible(SSR_DELAY);
	while (count < 10) {
		pr_debug("Sent ping number %d to check if wakeup is completed\n",
						count);
		ipclite_test_msg_send(ssr_client, macro);
		ret = wait_event_interruptible_timeout(t_data->wq,
						t_data->run, msecs_to_jiffies(SEC_DELAY));
		if (ret > 0)
			break;
		++count;
	}
	kthread_stop(bg_pings.thread);
	if (count == 10) {
		pr_info("FW Core wakeup failed.\n");
		goto exit;
	}
	t_data->run = false;
	pr_info("FW Core wakeup completed successfully.\n");
	pr_debug("Going for non crashing testing.\n");
	macro = get_param_macro();
	ipclite_test_msg_send(ssr_client, macro);
	macro = get_test_macro(PING, 0, 0, IPCLITE_TEST_START, 0);
	ipclite_test_msg_send(ssr_client, macro);
	wait_event_interruptible_timeout(t_data->wq,
			t_data->run, msecs_to_jiffies(SEC_DELAY + test_params.num_pings/10));
	if (!t_data->run)
		pr_info("SSR ping test failed\n");
exit:
	m_thread.run = true;
	wake_up_interruptible(&m_thread.wq);
	return 0;
}

static int ssr_test(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	uint64_t macro = 0;
	int ret = 0;

	wait_event_interruptible(t_data->wq, t_data->run);
	t_data->run = false;
	ret = thread_init(&wakeup_check, ssr_wakeup_check);
	if (ret != 0)
		return -EINVAL;

	ret = thread_init(&bg_pings, send_bg_pings);
	if (ret != 0) {
		kthread_stop(wakeup_check.thread);
		return -EINVAL;
	}
	memset(m_thread.pings_received, 0, sizeof(m_thread.pings_received));
	macro = get_test_macro(SSR, 0, SSR_CRASHING, IPCLITE_TEST_START, 0);
	ipclite_test_msg_send(ssr_client, macro);
	ret = wait_event_interruptible_timeout(t_data->wq, t_data->run,
					msecs_to_jiffies(CRASH_DELAY + test_params.num_pings/10));
	if (ret < 1)
		pr_err("Timeout - SSR\n");
	complete(&test_done);
	return 0;
}

static int inc_byte(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	ipclite_atomic_uint32_t *addr = global_memory.virt_base;

	wait_event_interruptible(t_data->wq, t_data->run);
	if (kthread_should_stop())
		return 0;
	for (int i = 0; i < test_params.num_itr; ++i)
		ipclite_global_atomic_inc(addr);
	threads_completed++;
	wake_up_interruptible(&thread_wq);
	return 0;
}

static int dec_byte(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	ipclite_atomic_uint32_t *addr = global_memory.virt_base;

	wait_event_interruptible(t_data->wq, t_data->run);
	if (kthread_should_stop())
		return 0;
	for (int i = 0; i < test_params.num_itr; ++i)
		ipclite_global_atomic_dec(addr);
	threads_completed++;
	wake_up_interruptible(&thread_wq);
	return 0;
}

static int global_atomics_test(int test_number)
{
	int ret, threads_started;
	int total_increment = 0;
	uint64_t macro;
	bool fail = false;
	struct ipclite_thread_data ga_t1, ga_t2;
	void *thread_1, *thread_2;
	void *gmem = global_memory.virt_base;

	if (!gmem) {
		pr_err("Error: gmem not initialized.\n");
		return -EFAULT;
	}
	pr_debug("The initial value of the gmem is %x\n", *((int *)gmem));

	threads_completed = 0;
	threads_started = 0;
	if (is_selected_sender(IPCMEM_APPS)) {
		switch (test_number) {
		case GLOBAL_ATOMICS_INC:
			thread_1 = inc_byte;
			thread_2 = inc_byte;
			break;
		case GLOBAL_ATOMICS_DEC:
			thread_1 = dec_byte;
			thread_2 = dec_byte;
			break;
		case GLOBAL_ATOMICS_INC_DEC:
			thread_1 = inc_byte;
			thread_2 = dec_byte;
			break;
		default:
			pr_err("Wrong input provided\n");
			return -EINVAL;
		}
		ret = thread_init(&ga_t1, thread_1);
		if (ret != 0)
			return -EINVAL;
		ret = thread_init(&ga_t2, thread_2);
		if (ret != 0) {
			kthread_stop(ga_t1.thread);
			return -EINVAL;
		}
	}
	macro = get_test_macro(GLOBAL_ATOMIC, 0, test_number,
				IPCLITE_TEST_START, 0);

	for (int i = 0; i < IPCMEM_NUM_HOSTS; ++i) {
		if (i == IPCMEM_APPS || !is_selected_sender(i))
			continue;
		ret = ipclite_test_msg_send(i, macro);
		if (ret == 0)
			threads_started += 2;
		else
			pr_err("failed to start test in %s core\n", core_name[i]);
	}
	if (is_selected_sender(IPCMEM_APPS)) {
		ga_t1.run = true;
		wake_up_interruptible(&ga_t1.wq);
		ga_t2.run = true;
		wake_up_interruptible(&ga_t2.wq);
		threads_started += 2;
	}
	/* Wait for all threads to complete or timeout */
	ret = wait_event_interruptible_timeout(thread_wq,
					threads_completed == threads_started,
					msecs_to_jiffies(SEC_DELAY));
	if (ret < 1) {
		pr_err("Timeout - not all threads completed\n");
		return -IPCLITE_TEST_FAIL;
	}

	pr_debug("The value of the gmem is %x\n", *((int *)gmem));
	/* Stopping threads if they have not already completed before evaluation */

	total_increment = 2 * test_params.num_senders * test_params.num_itr;

	switch (test_number) {
	case GLOBAL_ATOMICS_INC:
		if (*((int *)gmem) == total_increment)
			pr_info("Increment Successful.\n");
		else {
			pr_err("Increment Failed.\n");
			fail = true;
		}
		break;
	case GLOBAL_ATOMICS_DEC:
		if (*((int *)gmem) == 0)
			pr_info("Decrement Successful\n");
		else {
			pr_err("Decrement Failed\n");
			fail = true;
		}
		break;
	case GLOBAL_ATOMICS_INC_DEC:
		if (*((int *)gmem) == 0)
			pr_info("Increment and Decrement Successful\n");
		else {
			pr_err("Increment and Decrement Failed\n");
			fail = true;
		}
		break;
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

static int global_atomics_test_set_clear(void)
{
	int index = 0, ret = 0, threads_started, hosts_created;
	bool fail = false;
	struct handle_t *handle_ptr;
	uint64_t macro;
	int *handle_data = kcalloc(512, sizeof(int), GFP_KERNEL);

	if (!handle_data)
		return -ENOMEM;
	macro = get_test_macro(GLOBAL_ATOMIC, 0,
					GLOBAL_ATOMICS_SET_CLR, IPCLITE_TEST_CREATE, 0);
	for (hosts_created = 0; hosts_created < IPCMEM_NUM_HOSTS; ++hosts_created) {
		if (hosts_created == IPCMEM_APPS || !is_selected_sender(hosts_created))
			continue;
		ret = ipclite_test_msg_send(hosts_created, macro);
		if (ret != 0) {
			pr_err("failed to start test in core %s\n", core_name[hosts_created]);
			goto exit;
		}
	}
	handle_ptr = global_memory.virt_base;
	pr_info("Starting global atomics Test 4.\n");
	pr_debug("Starting allocation of index\n");
	pr_debug("The total number of handles is %d\n", NUM_HANDLES);
	pr_debug("Global Base : %p\n", handle_ptr);
	macro = get_test_macro(GLOBAL_ATOMIC, 0, GLOBAL_ATOMICS_SET_CLR,
						IPCLITE_TEST_START, 0);
	for (int itr = 0; itr < test_params.num_itr; itr++) {
		threads_started = 0;
		threads_completed = 0;
		for (int host = 0; host < IPCMEM_NUM_HOSTS; ++host) {
			if (host == IPCMEM_APPS || !is_selected_sender(host))
				continue;
			ret = ipclite_test_msg_send(host, macro);
			if (ret == 0)
				threads_started++;
		}
		if (is_selected_sender(IPCMEM_APPS)) {
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
			if (fail)
				pr_err("%d iteration failed\n", itr);
			else
				threads_completed++;
		}
		ret = wait_event_interruptible_timeout(thread_wq,
					threads_completed == threads_started,
					msecs_to_jiffies(SEC_DELAY));
		if (ret < 1) {
			pr_err("Timeout - not all cores completed\n");
			break;
		}
	}
	if (is_selected_sender(IPCMEM_APPS) && !fail)
		pr_info("APPS - Global Atomics Set and Clear test passed\n");
exit:
	macro = get_test_macro(GLOBAL_ATOMIC, 0, GLOBAL_ATOMICS_SET_CLR,
						IPCLITE_TEST_DESTROY, 0);
	for (int host = 0; host < hosts_created; ++host) {
		if (host == IPCMEM_APPS || !is_selected_sender(host))
			continue;
		ipclite_test_msg_send(host, macro);
	}
	kfree(handle_data);
	return fail ? -IPCLITE_TEST_FAIL  : 0;
}

static int global_atomics_test_wrapper(void *data_ptr)
{
	int result = 0, ret = 0;
	struct ipclite_thread_data *t_data = data_ptr;

	wait_event_interruptible(t_data->wq, t_data->run);
	*((int *)global_memory.virt_base) = 0;
	result = global_atomics_test(GLOBAL_ATOMICS_INC);
	msleep_interruptible(10);
	result |= global_atomics_test(GLOBAL_ATOMICS_DEC);
	msleep_interruptible(10);
	result |= global_atomics_test(GLOBAL_ATOMICS_INC_DEC);
	msleep_interruptible(10);
	result |= global_atomics_test_set_clear();
	if (result != 0) {
		pr_err("Global Atomics test failed\n");
		ret = -IPCLITE_TEST_FAIL;
	} else {
		pr_info("Global Atomics test passed\n");
		ret = 0;
	}
	complete(&test_done);
	return ret;
}

static int ping_test(void)
{
	int ret = 0;
	threads_completed = 0;
	for (int id = 0; id < test_params.num_thread; ++id) {
		memset(th_arr[id].pings_sent, 0, sizeof(th_arr[id].pings_sent));
		memset(th_arr[id].pings_received, 0, sizeof(th_arr[id].pings_received));
		th_arr[id].run = true;
		wake_up_interruptible(&th_arr[id].wq);
	}
	ret = wait_event_interruptible(thread_wq,
				threads_completed == test_params.num_thread);
	if (ret < 0) {
		pr_err("Timeout - All threads not completed, completed %d\n",
						threads_completed);
		goto stop;
	}
	pr_debug("All threads completed successfully.\n");
	pr_debug("Going for checking\n");
	/*Wait for the queue to get processed before checking if all replies are received*/
	if (!test_params.wait)
		msleep_interruptible(WAIT_DELAY * test_params.num_receivers);
	ret = check_pings();
	if (ret == 0)
		pr_info("Ping test passed on IPCMEM_APPS\n");
	else {
		pr_err("PING_SEND failed :%d\n", pingsend_fail);
		pr_err("Ping test failed on IPCMEM_APPS\n");
	}

stop:
	++cores_completed;
	return ret;
}

static int wrapper_ping_test(void *data_ptr)
{
	int ret = 0, id;
	struct ipclite_thread_data *t_data = data_ptr;
	uint64_t param_macro;

	wait_event_interruptible(t_data->wq, t_data->run);
	if (is_selected_sender(IPCMEM_APPS)) {
		for (id = 0; id < test_params.num_thread; ++id) {
			th_arr[id].t_id = id;
			ret = thread_init(&th_arr[id], ping_selected_receivers);
			if (ret != 0)
				goto exit;
		}
	}

	param_macro = get_test_macro(PING, 0, 0, IPCLITE_TEST_START, 0);
	for (int i = 0; i < test_params.num_itr; ++i) {
		cores_completed = 0;
		pingsend_fail = 0;
		/* Ping all senders to start sending messages.
		 *  If APPS is one of the senders start sending
		 */
		ping_sel_senders(param_macro);
		if (is_selected_sender(IPCMEM_APPS))
			ping_test();
		ret = wait_event_interruptible_timeout(t_data->wq,
			cores_completed == test_params.num_senders,
			msecs_to_jiffies((SEC_DELAY + test_params.num_pings/4)
				* (test_params.num_senders + !test_params.wait)));
		if (ret < 1) {
			pr_err("Timeout - Iteration %d of ping test failed\n", i+1);
			break;
		}
		pr_info("Iteration %d of ping test passed\n", i+1);
		if (test_params.num_itr > 1)
			msleep_interruptible(REFRESH_DELAY);
	}

exit:
	if (is_selected_sender(IPCMEM_APPS))
		while (id-- > 0)
			kthread_stop(th_arr[id].thread);
	complete(&test_done);
	return 0;
}

static int debug_tests(void *data_ptr)
{
	struct ipclite_thread_data *t_data = data_ptr;
	uint64_t macro;
	int ret;
	int disabled_core = ffz(test_params.enabled_cores);

	wait_event_interruptible(t_data->wq, t_data->run);
	if (is_selected_sender(IPCMEM_APPS)) {
		macro = get_test_macro(DEBUG, 0, PING_SEND, 0, 0);
		if (disabled_core == IPCMEM_NUM_HOSTS)
			pr_err("All cores are enabled. No Disabled cores\n");
		/* Pinging one enabled and disabled cores to get the error and dbg prints */
		if (disabled_core < IPCMEM_NUM_HOSTS) {
			ret = ipclite_test_msg_send(disabled_core, macro);
			if (ret == 0)
				pr_err("Debug test failed\n");
			else
				pr_info("Debug test passed\n");
		}
		++cores_completed;
	}

	ret = wait_event_interruptible_timeout(t_data->wq,
					cores_completed == test_params.num_senders,
						msecs_to_jiffies(SEC_DELAY));
	if (ret < 1)
		pr_err("Timeout - other cores not completed\n");
	else
		pr_info("Test completed on all cores\n");
	complete(&test_done);
	return 0;
}

static void ipclite_test_set_receivers(void)
{
	if (test_params.selected_receivers > IPCLITE_TEST_ALL_CORES) {
		pr_err("Invalid value given to selected_receivers\n");
		test_params.selected_receivers = 1;
	}
	/* Check number of 1s using hamming weight function.
	 * Number of 1s is number of receivers
	 */
	test_params.num_receivers = hweight_long(test_params.selected_receivers);
	pr_info("selected_receivers set to %d\n", test_params.selected_receivers);
}

static void ipclite_test_set_senders(void)
{
	if (test_params.selected_senders > IPCLITE_TEST_ALL_CORES) {
		pr_err("Invalid value given to selected_senders\n");
		test_params.selected_senders = 1;
	}
	/* Check number of 1s using hamming weight function. */
	test_params.num_senders = hweight_long(test_params.selected_senders);
	pr_info("selected_senders set to %d\n", test_params.selected_senders);
}

static int main_thread_create(void *fptr)
{
	int ret = 0;

	ret = thread_init(&m_thread, fptr);
	if (ret != 0)
		return ret;
	m_thread.run = true;
	wake_up_interruptible(&m_thread.wq);
	return 0;
}

static void ipclite_test_set_test(void)
{
	int ret = 0, receiver;
	uint64_t macro;

	if (test_params.selected_test_case > 8) {
		pr_err("Error: Invalid value given to test_case\n");
		return;
	}

	pr_info("selected_test_case set to %s\n",
				test_name[test_params.selected_test_case - 1]);
	macro = get_test_macro(test_params.selected_test_case, 0,
				0, IPCLITE_TEST_START, 0);

	switch (test_params.selected_test_case) {
	case PING:
		th_arr = kcalloc(test_params.num_thread, sizeof(*th_arr), GFP_KERNEL);
		if (!th_arr)
			return;
		ret = main_thread_create(wrapper_ping_test);
		break;
	case NEGATIVE:
		cores_completed = 0;
		receiver = ffs(test_params.selected_receivers) - 1;
		if (is_enabled_core(receiver)) {
			pr_err("Error: selected receiver can't be a enabled core\n");
			return;
		}
		ping_sel_senders(macro);
		ret = main_thread_create(negative_tests);
		break;
	case GLOBAL_ATOMIC:
		ret = main_thread_create(global_atomics_test_wrapper);
		break;
	case DEBUG:
		cores_completed = 0;
		ping_sel_senders(macro);
		ret = main_thread_create(debug_tests);
		break;
	case SSR:
		if (test_params.num_senders != 1) {
			pr_err("Error: SSR Testing requires only 1 core to be selected\n");
			return;
		}
		if (test_params.selected_senders & test_params.selected_receivers) {
			pr_err("Error: SSR Testing can't be done within the same core\n");
			return;
		}
		if (!is_selected_receiver(IPCMEM_APPS)) {
			pr_err("Error: SSR Testing need apps to be one of the receiver\n");
			return;
		}
		/* Find first set (ffs) to get the bit position/index of sender */
		ssr_client = ffs(test_params.selected_senders) - 1;
		if (ssr_client == 0 || !is_enabled_core(ssr_client)) {
			pr_err("Error: Invalid core selected for SSR Testing\n");
			return;
		}
		pr_info("Starting SSR test for core %s\n", core_name[ssr_client]);
		ret = main_thread_create(ssr_test);
		break;
	case HW_MUTEX:
		if (test_params.num_senders != 1) {
			pr_err("Error: HW Mutex Testing requires only 1 core to be selected\n");
			return;
		}
		if (test_params.selected_senders & test_params.selected_receivers) {
			pr_err("Error: HW Mutex Testing can't be done within the same core\n");
			return;
		}
		ret = main_thread_create(hw_mutex_test);
		break;
	default:
		pr_err("Error: Wrong input provided\n");
		return;
	}
	if (ret == 0)
		wait_for_completion(&test_done);
	if (test_params.selected_test_case == PING) {
		kfree(th_arr);
		th_arr = NULL;
	}
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

static int basic_ping_test(void)
{
	int ret;
	bool fail = false;
	uint64_t macro = get_test_macro(PING, 0, BASIC_PING, 0, 0);

	for (int core = 0; core < IPCMEM_NUM_HOSTS; ++core) {
		if (core == IPCMEM_APPS || !is_enabled_core(core))
			continue;
		corestatus = false;
		ipclite_test_msg_send(core, macro);
		ret = wait_event_interruptible_timeout(thread_wq,
							corestatus, msecs_to_jiffies(250));
		if (ret < 1) {
			pr_err("Timeout - core %d not alive\n", core);
			fail = true;
		}
	}
	if (fail)
		return -IPCLITE_TEST_FAIL;
	return IPCLITE_TEST_PASS;
}

static ssize_t ipclite_test_params_write(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	char *temp_buf = kmalloc(strlen(buf)+1, GFP_KERNEL);
	char *temp_ptr = temp_buf;
	int ret, common_cores;
	uint64_t param_macro;

	if (!temp_buf) {
		pr_err("Error: Memory not allocated\n");
		return -EINVAL;
	}

	ret = strscpy(temp_buf, buf, strlen(buf)+1);

	if (ret < 0) {
		pr_err("Error: User input is too large\n");
		goto exit;
	}

	init_test_params();
	ret = parse_param(&temp_buf, &test_params.selected_test_case);
	if (ret != 0)
		goto exit;

	switch (test_params.selected_test_case) {
	case PING:
		ret = parse_param(&temp_buf, &test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		ret = parse_param(&temp_buf, &test_params.num_pings);
		if (ret != 0)
			break;
		ret = parse_param(&temp_buf, &test_params.wait);
		if (ret != 0)
			break;
		ret = parse_param(&temp_buf, &test_params.num_itr);
		if (ret != 0)
			break;
		ret = parse_param(&temp_buf, &test_params.num_thread);
		if (ret != 0)
			break;
		if (test_params.num_pings > 200000) {
			pr_err("Invalid value given to pings\n");
			test_params.num_pings = 1;
		}
		if (test_params.num_thread > IPCLITE_TEST_MAX_THREADS) {
			pr_err("Invalid value given to num_thread\n");
			test_params.num_thread = 1;
		} else if (test_params.num_thread > test_params.num_pings) {
			pr_err("Invalid num_thread for given number of pings\n");
			test_params.num_thread = test_params.num_pings;
		}
		common_cores = test_params.selected_senders
								& test_params.selected_receivers;
		common_cores = common_cores ? hweight_long(common_cores) : 1;
		if (test_params.wait * test_params.num_thread * common_cores > 1000) {
			pr_err("Overall wait value is more then queue size. Setting max.\n");
			test_params.wait = 1000/(test_params.num_thread * common_cores);
		} else if (test_params.wait > test_params.num_pings) {
			pr_err("Invalid value given to wait\n");
			test_params.wait = 1;
		}
		if (test_params.num_itr > 4000) {
			pr_err("Invalid value given to itr\n");
			test_params.num_itr = 1;
		}
		pr_info("num_pings set to %d\n", test_params.num_pings);
		pr_info("num_itr set to %d\n", test_params.num_itr);
		pr_info("wait set to %d\n", test_params.wait);
		pr_info("num_thread set to %d\n", test_params.num_thread);
		break;
	case NEGATIVE:
	case HW_MUTEX:
		ret = parse_param(&temp_buf, &test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		break;
	case GLOBAL_ATOMIC:
		ret = parse_param(&temp_buf, &test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &test_params.num_itr);
		if (ret != 0)
			break;
		if (test_params.num_itr > 4000) {
			pr_err("Invalid value given to itr\n");
			test_params.num_itr = 1;
		}
		pr_info("num_itr set to %d\n", test_params.num_itr);
		break;
	case DEBUG:
		ret = parse_param(&temp_buf, &test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		break;
	case SSR:
		ret = parse_param(&temp_buf, &test_params.selected_senders);
		if (ret != 0)
			break;
		ipclite_test_set_senders();
		ret = parse_param(&temp_buf, &test_params.selected_receivers);
		if (ret != 0)
			break;
		ipclite_test_set_receivers();
		ret = parse_param(&temp_buf, &test_params.num_pings);
		if (ret != 0)
			break;
		if (test_params.num_pings > 20000) {
			pr_err("Invalid value given to pings\n");
			test_params.num_pings = 1;
		}
		pr_info("num_pings set to %d\n", test_params.num_pings);
		break;
	default:
		pr_err("Error: Wrong input provided\n");
		goto exit;
	}
	if (ret != 0)
		goto exit;
	test_params.enabled_cores = test_params.selected_senders;
	if (test_params.selected_test_case != NEGATIVE)
		test_params.enabled_cores |= test_params.selected_receivers;
	ret = basic_ping_test();
	if (ret != IPCLITE_TEST_PASS)
		goto exit;
	param_macro = get_param_macro();
	ping_sel_senders(param_macro);
	ipclite_test_set_test();
exit:
	kfree(temp_ptr);
	return count;
}

static void ping_callback(int test_info, int t_id, int payload_info, int start_stop_info,
						int pass_fail_info, int client_id)
{
	uint64_t reply_macro;
	int ret = 0;

	if (payload_info == PING_SEND) {
		reply_macro = get_test_macro(test_info, t_id,
						PING_REPLY, 0, 0);
		ret = ipclite_test_msg_send(client_id, reply_macro);
		if (ret == -EAGAIN)
			++pingsend_fail;
		return;
	}
	if (payload_info == PING_REPLY) {
		if (test_info == PING)
			ping_receive(&th_arr[t_id], client_id);
		return;
	}
	if (pass_fail_info == IPCLITE_TEST_PASS)
		pr_info("Test passed on core %s\n", core_name[client_id]);
	else if (pass_fail_info == IPCLITE_TEST_FAIL)
		pr_err("Test failed on core %s\n", core_name[client_id]);
	if (start_stop_info == IPCLITE_TEST_STOP) {
		if (test_params.selected_test_case == SSR) {
			wakeup_check.run = true;
			wake_up_interruptible(&wakeup_check.wq);
			return;
		}
		++cores_completed;
		if (cores_completed == test_params.num_senders)
			wake_up_interruptible(&m_thread.wq);
		return;
	}
	if (payload_info == BASIC_PING) {
		corestatus = true;
		wake_up_interruptible(&thread_wq);
	}
}

static void hw_mutex_callback(int test_info, int start_stop_info,
						int pass_fail_info, int client_id)
{
	uint64_t reply_macro;
	int ret = 0;

	if (start_stop_info == IPCLITE_TEST_START) {
		ret = ipclite_hw_mutex_release();
		if (ret == 0)
			*((int *)global_memory.virt_base) = IPCMEM_APPS;
		reply_macro = get_test_macro(test_info, 0, HW_MUTEX_RELEASE,
						IPCLITE_TEST_STOP, 0);
		ipclite_test_msg_send(client_id, reply_macro);
	}
	if (pass_fail_info == IPCLITE_TEST_PASS)
		pr_info("HW Unlock Test passed on core %s\n",
				core_name[client_id]);
	else if (pass_fail_info == IPCLITE_TEST_FAIL)
		pr_err("HW Unlock Test failed on core %s\n",
				core_name[client_id]);
	if (start_stop_info == IPCLITE_TEST_STOP) {
		m_thread.run = true;
		if (!is_selected_sender(IPCMEM_APPS))
			wake_up_interruptible(&m_thread.wq);
	}
}

static void ssr_callback(int payload_info, int start_stop_info, int client_id)
{
	if (payload_info == PING_SEND) {
		m_thread.pings_received[client_id]++;
		if (m_thread.pings_received[client_id] == test_params.num_pings) {
			pr_info("Waking up ssr_wakeup_check_thread.\n");
			pr_info("Signaling other cores to make sure there is no other crash\n");
			wakeup_check.run = true;
			wake_up_interruptible(&wakeup_check.wq);
			bg_pings.run = true;
			wake_up_interruptible(&bg_pings.wq);
		}
		return;
	}
	if (payload_info == SSR_WAKEUP) {
		if (start_stop_info == IPCLITE_TEST_STOP) {
			wakeup_check.run = true;
			pr_info("%s wakeup completed\n",
					core_name[client_id]);
			wake_up_interruptible(&wakeup_check.wq);
		}
		return;
	}
	if (payload_info == PING_REPLY) {
		bg_pings.run = true;
		wake_up_interruptible(&bg_pings.wq);
	}
}

static void atomic_callback(int payload_info, int pass_fail_info, int client_id)
{
	if (payload_info == GLOBAL_ATOMICS_SET_CLR) {
		if (pass_fail_info == IPCLITE_TEST_PASS) {
			threads_completed++;
			if (threads_completed == test_params.num_senders)
				wake_up_interruptible(&thread_wq);
		} else
			pr_err("%s Global Atomics test failed\n",
								core_name[client_id]);
	} else {
		threads_completed += 2;
		wake_up_interruptible(&thread_wq);
	}
}

static int ipclite_test_callback_fn(uint32_t client_id, int64_t msg, void *data)
{
	uint64_t header, test_info, t_id, payload_info,
				start_stop_info, pass_fail_info;

	/* Unpack the different bit fields from message value */
	header = (msg & GENMASK(63, 56))>>56;
	//parameter_info = (msg & GENMASK(55, 48))>>48;
	test_info = (msg & GENMASK(47, 40))>>40;
	t_id = (msg & GENMASK(39, 36))>>36;
	payload_info = (msg & GENMASK(35, 16))>>16;
	start_stop_info = (msg & GENMASK(15, 8))>>8;
	pass_fail_info = (msg & GENMASK(7, 0));

	if (header != IPCLITE_TEST_HEADER) {
		pr_err("Corrupted message - client_id:%d, msg:%llx\n", client_id, msg);
		return -EINVAL;
	}

	pr_debug("The message received is %lld\n", msg);

	switch (test_info) {
	case PING:
	case NEGATIVE:
	case DEBUG:
		ping_callback(test_info, t_id, payload_info, start_stop_info,
						pass_fail_info, client_id);
		break;
	case HW_MUTEX:
		hw_mutex_callback(test_info, start_stop_info, pass_fail_info, client_id);
		break;
	case SSR:
		ssr_callback(payload_info, start_stop_info, client_id);
		break;
	case GLOBAL_ATOMIC:
		atomic_callback(payload_info, pass_fail_info, client_id);
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

	ret = get_global_partition_info(&global_memory);
	if (ret != 0) {
		pr_err("Unable to load global partition information\n");
		goto bail;
	}

	ret = ipclite_register_test_client(ipclite_test_callback_fn, NULL);
	if (ret != 0) {
		pr_err("Could not register client\n");
		goto bail;
	}

	ret = ipclite_test_sysfs_node_setup();
	if (ret != 0) {
		pr_err("Failed to create sysfs interface\n");
		goto bail;
	}

bail:
	return ret;
}

static void __exit ipclite_test_exit(void)
{
	pr_info("Removing IPCLite Test Module\n");
	sysfs_remove_file(sysfs_dir, &ipclite_test_params.attr);
	kobject_put(sysfs_dir);
}

module_init(ipclite_test_init);
module_exit(ipclite_test_exit);

MODULE_LICENSE("GPL v2");
