/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
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

/* List of Cases Available for Testing */
enum ipclite_test_type {
	PING		= 1,
	NEGATIVE	= 2,
	GLOBAL_ATOMIC	= 3,
	DEBUG		= 4,
	SSR		= 5,
	HW_MUTEX	= 6,
};

/* List of sysfs parameters */
enum ipclite_test_param {
	TEST_CASE	= 1,
	SENDER_LIST	= 2,
	RECEIVER_LIST	= 3,
	NUM_PINGS	= 4,
	WAIT		= 5,
	NUM_ITR		= 6,
	NUM_THREADS	= 7,
	ENABLED_CORES	= 8,
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

struct ipclite_test_params {
	int wait;
	int num_pings;
	int num_itr;
	int selected_senders;
	int selected_receivers;
	int selected_test_case;
	int enabled_cores;
	int num_thread;
	int num_senders;
	int num_receivers;
};

struct ipclite_test_data {
	int pings_sent[IPCMEM_NUM_HOSTS];
	int pings_received[IPCMEM_NUM_HOSTS];
	int client_id;
	struct global_region_info *global_memory;
	struct ipclite_test_params test_params;
	int ssr_client;
};

struct ipclite_thread_data {
	struct task_struct *thread;
	void *data;
	wait_queue_head_t wq;
	bool run;
};

static int ipclite_test_callback_fn(unsigned int client_id, long long  msg, void *d);
