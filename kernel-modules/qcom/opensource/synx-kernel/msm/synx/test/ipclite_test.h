/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023-2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include "../ipclite_client.h"
#include "../ipclite.h"

/* General testing related configurations */
#define IPCLITE_TEST_MAX_THREADS 5
#define IPCLITE_TEST_HEADER 0xaa
#define IPCLITE_TEST_ALL_CORES GENMASK(IPCMEM_NUM_HOSTS - 1, 0)

/* Synx Usecase related definitions */
#define NUM_HANDLES   4096
#define BITMAP_SIZE   (NUM_HANDLES/32)
#define BITS(x) (sizeof(x)*8)

struct handle_t {
	int handle_bitmap[BITMAP_SIZE];
	int handle_data[NUM_HANDLES];
};

/* Flags for Pass, Fail, Start, and Stop */
#define IPCLITE_TEST_PASS 2
#define IPCLITE_TEST_FAIL 1

#define IPCLITE_TEST_START 2
#define IPCLITE_TEST_STOP 1

#define IPCLITE_TEST_CREATE 3
#define IPCLITE_TEST_DESTROY 4

#define SEC_DELAY 1000

#define RETRY_DELAY 50
#define REFRESH_DELAY 500
#define WAIT_DELAY 2000

#define SSR_DELAY 30000
#define CRASH_DELAY 45000

/* List of Cases Available for Testing */
enum ipclite_test_type {
	PING		= 1,
	NEGATIVE	= 2,
	GLOBAL_ATOMIC	= 3,
	DEBUG		= 4,
	SSR		= 5,
	HW_MUTEX	= 6,
	NUM_TESTS	= 6,
};

/* List of sysfs parameters */
enum ipclite_test_param {
	TEST_CASE	= 1,
	PARAM	= 2,
};

/* List of subtests for HW Mutex Test */
enum ipclite_test_hw_mutex_subtest {
	HW_MUTEX_RELEASE	= 1,
};

/* List of messages for SSR Testing */
enum ipclite_test_ssr_subtest {
	SSR_CRASHING	= 1,
	SSR_WAKEUP	= 2,
};

/* List of subtest for Global Atomics Testing */
enum ipclite_test_global_atomics_subtest {
	GLOBAL_ATOMICS_INC	= 1,
	GLOBAL_ATOMICS_DEC	= 2,
	GLOBAL_ATOMICS_INC_DEC	= 3,
	GLOBAL_ATOMICS_SET_CLR	= 4,
};

/* Types of pings and replies to be sent and received */
enum ipclite_test_ping {
	PING_SEND	= 10,
	PING_REPLY	= 11,
	BASIC_PING	= 12,
};

static char core_name[IPCMEM_NUM_HOSTS][13] = {
					"IPCMEM_APPS",
					"IPCMEM_MODEM",
					"IPCMEM_LPASS",
					"IPCMEM_SLPI",
					"IPCMEM_GPU",
					"IPCMEM_CDSP",
					"IPCMEM_CVP",
					"IPCMEM_CAM",
					"IPCMEM_VPU"
};

static char test_name[NUM_TESTS][14] = {
					"PING",
					"NEGATIVE",
					"GLOBAL_ATOMIC",
					"DEBUG",
					"SSR",
					"HW_MUTEX",
};

struct ipclite_test_params {
	unsigned int wait;
	unsigned int num_pings;
	unsigned int num_itr;
	unsigned int selected_senders;
	unsigned int selected_receivers;
	unsigned int selected_test_case;
	unsigned int enabled_cores;
	unsigned int num_thread;
	unsigned int num_senders;
	unsigned int num_receivers;
};

struct ipclite_thread_data {
	struct task_struct *thread;
	int t_id;
	int num_pings;
	int pings_sent[IPCMEM_NUM_HOSTS];
	int pings_received[IPCMEM_NUM_HOSTS];
	wait_queue_head_t wq;
	bool run;
};

static int ipclite_test_callback_fn(uint32_t client_id, int64_t msg, void *data);
