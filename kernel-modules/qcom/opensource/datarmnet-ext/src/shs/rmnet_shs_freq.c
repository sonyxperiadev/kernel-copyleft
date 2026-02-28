// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include "rmnet_shs.h"
#include "rmnet_shs_freq.h"
#include "rmnet_shs_modules.h"
#include "rmnet_shs_config.h"

#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/pm_qos.h>

#define MAX_FREQ INT_MAX
#define MIN_FREQ 0
#define BOOST_FREQ rmnet_shs_cfg.cpu_freq_boost_val

/* CPU1 is doing important work, dont do freq boost work on it */
#define WORK_CPU 2
struct cpu_freq {
	unsigned int freq_floor;
	unsigned int freq_ceil;
};

unsigned int rmnet_shs_freq_enable __read_mostly = 1;
module_param(rmnet_shs_freq_enable, uint, 0644);
MODULE_PARM_DESC(rmnet_shs_freq_enable, "Enable/disable freq boost feature");

/* Shared workqueue between existing boosting and pb marker boosting*/
struct workqueue_struct *shs_boost_wq;

struct rmnet_shs_cpu_boosts {
	struct cpu_freq __percpu *cpu_boosts;
};
static struct rmnet_shs_cpu_boosts shs_cpu_boosts;

static struct work_struct boost_cpu;
static DEFINE_PER_CPU(struct freq_qos_request, boost_req);

/* PB Burst Marker has its own work struct and cpe_freqs */
struct rmnet_shs_pb_cpu_boosts {
	struct cpu_freq __percpu *pb_cpu_boosts;
};
static struct rmnet_shs_pb_cpu_boosts shs_pb_cpu_boosts;

static struct work_struct pb_boost_worker;
static DEFINE_PER_CPU(struct freq_qos_request, pb_boost_req);

static void shs_update_cpu_policy(struct work_struct *work)
{
	struct cpu_freq *boost;
	unsigned int i;
	int ret;
	struct freq_qos_request *req;

	cpus_read_lock();
	for_each_online_cpu(i) {
		boost = per_cpu_ptr(shs_cpu_boosts.cpu_boosts, i);
		req = &per_cpu(boost_req, i);

		ret = freq_qos_update_request(req, boost->freq_floor);

	}
	cpus_read_unlock();
}

static void shs_update_pb_cpu_policy(struct work_struct *work)
{
	struct cpu_freq *boost;
	unsigned int i;
	int ret;
	struct freq_qos_request *req;



	cpus_read_lock();
	for_each_online_cpu(i) {
		boost = per_cpu_ptr(shs_pb_cpu_boosts.pb_cpu_boosts, i);
		req = &per_cpu(pb_boost_req, i);

		ret = freq_qos_update_request(req, boost->freq_floor);
		SHS_TRACE_LOW(RMNET_SHS_PB_BOOST_CPU, RMNET_SHS_PB_BOOST_CPU_UPDATE,
					  boost->freq_floor, boost->freq_ceil, 0xDEF, 0xDEF, NULL,
					  NULL);
	}
	cpus_read_unlock();
}

void rmnet_shs_reset_freq(void)
{
	struct cpu_freq *boost;
	int i;

	for_each_possible_cpu(i) {
		boost = per_cpu_ptr(shs_cpu_boosts.cpu_boosts, i);
		boost->freq_floor = MIN_FREQ;
		boost->freq_ceil = MAX_FREQ;
	}

	for_each_possible_cpu(i) {
		boost = per_cpu_ptr(shs_pb_cpu_boosts.pb_cpu_boosts, i);
		boost->freq_floor = MIN_FREQ;
		boost->freq_ceil = MAX_FREQ;
	}
}

/* Does not queue it's own work, must be called before boost_cpus, and
 * reset_cpus must be called afterwards to ramp it down.
 */
void rmnet_shs_boost_gold_cpu(int cpu)
{
	struct cpu_freq *boost;
	int i = cpu;

	if (cpu < 0 || cpu >= MAX_CPUS)
		return;

	if ((1 << i) & NONPERF_MASK)
		return;
	boost = per_cpu_ptr(shs_cpu_boosts.cpu_boosts, i);
	boost->freq_floor = BOOST_FREQ;
	boost->freq_ceil = MAX_FREQ;
	trace_rmnet_freq_boost(i, BOOST_FREQ);
}

void rmnet_shs_boost_cpus(void)
{
	struct cpu_freq *boost;
	int i;
	for_each_possible_cpu(i) {

		if ((1 << i) & PERF_MASK)
			continue;
		boost = per_cpu_ptr(shs_cpu_boosts.cpu_boosts, i);
		boost->freq_floor = BOOST_FREQ;
		boost->freq_ceil = MAX_FREQ;
		trace_rmnet_freq_boost(i, BOOST_FREQ);
	}

	if (work_pending(&boost_cpu))
		return;

	if (shs_boost_wq) {
		queue_work_on(WORK_CPU, shs_boost_wq, &boost_cpu);
	}
}

void rmnet_shs_pb_boost_cpus(void)
{
	struct cpu_freq *boost;
	int i;
	for_each_possible_cpu(i) {

		if ((1 << i) & PERF_MASK)
			continue;
		boost = per_cpu_ptr(shs_pb_cpu_boosts.pb_cpu_boosts, i);
		boost->freq_floor = BOOST_FREQ;
		boost->freq_ceil = MAX_FREQ;
		trace_rmnet_freq_boost(i, BOOST_FREQ);
	}

	if (work_pending(&pb_boost_worker))
		return;

	if (shs_boost_wq) {
		queue_work_on(WORK_CPU, shs_boost_wq, &pb_boost_worker);
	}
}

void rmnet_shs_reset_cpus(void)
{
	struct cpu_freq *boost;
	int i;

	for_each_possible_cpu(i) {
		boost = per_cpu_ptr(shs_cpu_boosts.cpu_boosts, i);
		boost->freq_floor = MIN_FREQ;
		boost->freq_ceil = MAX_FREQ;
		trace_rmnet_freq_reset(i, MIN_FREQ);
	}

	if (work_pending(&boost_cpu))
		return;

	if (shs_boost_wq)
		queue_work_on(WORK_CPU, shs_boost_wq, &boost_cpu);
}

void rmnet_shs_pb_reset_cpus(void)
{
	struct cpu_freq *boost;
	int i;

	for_each_possible_cpu(i) {
		boost = per_cpu_ptr(shs_pb_cpu_boosts.pb_cpu_boosts, i);
		boost->freq_floor = MIN_FREQ;
		boost->freq_ceil = MAX_FREQ;
		trace_rmnet_freq_reset(i, MIN_FREQ);
	}

	if (work_pending(&pb_boost_worker))
		return;

	if (shs_boost_wq)
		queue_work_on(WORK_CPU, shs_boost_wq, &pb_boost_worker);
}

static void rmnet_shs_remove_qos_reqs(void)
{
	struct freq_qos_request *req;
	int i;

	for_each_possible_cpu(i) {
		req = &per_cpu(boost_req, i);
		if (req &&  freq_qos_request_active(req)) {
			freq_qos_remove_request(req);
		}

		req = &per_cpu(pb_boost_req, i);
		if (req &&  freq_qos_request_active(req)) {
			freq_qos_remove_request(req);
		}
	}
}

int rmnet_shs_freq_init(void)
{
	struct cpu_freq *boost;
	int i;
	int ret = 0;
	struct freq_qos_request *req;
	struct cpufreq_policy *policy;

	shs_cpu_boosts.cpu_boosts = alloc_percpu(struct cpu_freq);
	if (!shs_cpu_boosts.cpu_boosts)
		return -ENOMEM;

	shs_pb_cpu_boosts.pb_cpu_boosts = alloc_percpu(struct cpu_freq);
	if (!shs_pb_cpu_boosts.pb_cpu_boosts) {
		free_percpu(shs_cpu_boosts.cpu_boosts);
		return -ENOMEM;
	}

	if (!shs_boost_wq)
		shs_boost_wq = alloc_workqueue("shs_boost_wq", WQ_HIGHPRI, 0);

	if (!shs_boost_wq) {
		ret = -ENOMEM;
		goto err;
	}

	for_each_possible_cpu(i) {
		boost = per_cpu_ptr(shs_cpu_boosts.cpu_boosts, i);
		req = &per_cpu(boost_req, i);
		policy = cpufreq_cpu_get(i);
		if (!policy) {
			pr_err("%s: cpufreq policy not found for cpu%d\n",
							__func__, i);
			return -ESRCH;
		}

		ret = freq_qos_add_request(&policy->constraints, req,
				FREQ_QOS_MIN, MIN_FREQ);
		if (ret < 0) {
			pr_err("%s: Failed to add freq constraint (%d)\n",
						__func__, ret);
			return ret;
		}

		req = &per_cpu(pb_boost_req, i);
		policy = cpufreq_cpu_get(i);
		if (!policy) {
			pr_err("%s: cpufreq policy not found for pb cpu%d\n",
							__func__, i);
			return -ESRCH;
		}

		ret = freq_qos_add_request(&policy->constraints, req,
				FREQ_QOS_MIN, MIN_FREQ);
		if (ret < 0) {
			pr_err("%s: Failed to add pb freq constraint (%d)\n",
						__func__, ret);
			return ret;
		}
	}
	INIT_WORK(&boost_cpu, shs_update_cpu_policy);
	INIT_WORK(&pb_boost_worker, shs_update_pb_cpu_policy);

	rmnet_shs_reset_freq();
	return 0;
err:
	/* this resetting of frequencies is redundant when cpu_boosts is dynamic
	 * but will be leaving it in here for if we switch back to static
	 */
	rmnet_shs_reset_freq();
	free_percpu(shs_cpu_boosts.cpu_boosts);
	free_percpu(shs_pb_cpu_boosts.pb_cpu_boosts);
	if (shs_boost_wq) {
		destroy_workqueue(shs_boost_wq);
		shs_boost_wq = NULL;
	}

	return ret;
}

int rmnet_shs_freq_exit(void)
{
	/* No need to cancel work as it will be drained and not re-queued */
	/* No need to call reset_freq as removing qos freqs will do that for us */
	rmnet_shs_remove_qos_reqs();

	if (shs_boost_wq) {
		destroy_workqueue(shs_boost_wq);
		shs_boost_wq = NULL;
	}

	free_percpu(shs_cpu_boosts.cpu_boosts);
	free_percpu(shs_pb_cpu_boosts.pb_cpu_boosts);
	return 0;
}
