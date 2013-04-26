/* arch/arm/mach-msm/cpufreq.c
 *
 * MSM architecture cpufreq driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2013, The Linux Foundation. All rights reserved.
 * Author: Mike A. Chan <mikechan@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <mach/socinfo.h>
#include <mach/cpufreq.h>

#include "acpuclock.h"

struct cpufreq_work_struct {
	struct work_struct work;
	struct completion complete;
	uint32_t new_freq;
	uint32_t cur_freq;
	int cpu;
	int status;
};

static DEFINE_PER_CPU(struct cpufreq_work_struct, cpufreq_work);
static struct workqueue_struct *msm_cpufreq_wq;

struct cpufreq_suspend_t {
	struct mutex suspend_mutex;
	int device_suspended;
};

static DEFINE_PER_CPU(struct cpufreq_suspend_t, cpufreq_suspend);

struct cpu_freq_limit {
	uint32_t allowed_max;
	uint32_t allowed_min;
	uint32_t cpu_mask;
};

static struct cpu_freq_limit cpu_freq_limits[NR_LIMITS];
static uint32_t cpuinfo_max, cpuinfo_min;
static DEFINE_MUTEX(limits_mutex);

static int limit_turbo = 2;
module_param_named(limit_turbo, limit_turbo, int, 0644);

static void get_limits(uint32_t cpu, uint32_t *min, uint32_t *max)
{
	uint32_t hi = cpuinfo_max;
	uint32_t low = cpuinfo_min;
	struct cpu_freq_limit *limit = NULL;
	int i = 0;

	mutex_lock(&limits_mutex);
	for (i = 0; i < NR_LIMITS; i++) {
		limit = &cpu_freq_limits[i];
		if (!(limit->cpu_mask & BIT(cpu)))
			continue;
		if (hi > limit->allowed_max)
			hi = limit->allowed_max;
		if (low < limit->allowed_min)
			low = limit->allowed_min;
	}

	*min = low;
	*max = hi;
	mutex_unlock(&limits_mutex);
}

static int set_cpu_freq(unsigned int cpu,
		unsigned int cur_freq, unsigned int new_freq)
{
	int ret = 0;
	int saved_sched_policy = -EINVAL;
	int saved_sched_rt_prio = -EINVAL;
	struct cpufreq_freqs freqs;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
	uint32_t allowed_max = 0;
	uint32_t allowed_min = 0;

	get_limits(cpu, &allowed_min, &allowed_max);

	if (new_freq > allowed_max) {
		new_freq = allowed_max;
		pr_debug("msm: cpufreq limiting freq to %d\n", new_freq);
	}

	if (new_freq < allowed_min) {
		new_freq = allowed_min;
		pr_debug("msm: cpufreq: min freq set to %d\n", new_freq);
	}

	freqs.old = cur_freq;
	freqs.new = new_freq;
	freqs.cpu = cpu;

	/*
	 * Put the caller into SCHED_FIFO priority to avoid cpu starvation
	 * in the acpuclk_set_rate path while increasing frequencies
	 */

	if (freqs.new > freqs.old && current->policy != SCHED_FIFO) {
		saved_sched_policy = current->policy;
		saved_sched_rt_prio = current->rt_priority;
		sched_setscheduler_nocheck(current, SCHED_FIFO, &param);
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	ret = acpuclk_set_rate(cpu, new_freq, SETRATE_CPUFREQ);
	if (!ret)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	/* Restore priority after clock ramp-up */
	if (freqs.new > freqs.old && saved_sched_policy >= 0) {
		param.sched_priority = saved_sched_rt_prio;
		sched_setscheduler_nocheck(current, saved_sched_policy, &param);
	}
	return ret;
}

static void set_cpu_work(struct work_struct *work)
{
	struct cpufreq_work_struct *cpu_work =
		container_of(work, struct cpufreq_work_struct, work);

	cpu_work->status = set_cpu_freq(cpu_work->cpu,
			cpu_work->cur_freq, cpu_work->new_freq);
	complete(&cpu_work->complete);
}

static int msm_cpufreq_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int ret = -EFAULT;
	int index;
	struct cpufreq_frequency_table *table;

	struct cpufreq_work_struct *cpu_work = NULL;
	cpumask_var_t mask;

	if (!cpu_active(policy->cpu)) {
		pr_info("cpufreq: cpu %d is not active.\n", policy->cpu);
		return -ENODEV;
	}

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	mutex_lock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);

	if (per_cpu(cpufreq_suspend, policy->cpu).device_suspended) {
		pr_debug("cpufreq: cpu%d scheduling frequency change "
				"in suspend.\n", policy->cpu);
		ret = -EFAULT;
		goto done;
	}

	table = cpufreq_frequency_get_table(policy->cpu);
	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto done;
	}

	pr_debug("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
		policy->cpu, target_freq, relation,
		policy->min, policy->max, table[index].frequency);

	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	cpu_work->new_freq = table[index].frequency;
	cpu_work->cur_freq = policy->cur;
	cpu_work->cpu = policy->cpu;
	cpu_work->status = -ENODEV;

	cpumask_clear(mask);
	cpumask_set_cpu(policy->cpu, mask);
	if (cpumask_equal(mask, &current->cpus_allowed)) {
		ret = set_cpu_freq(cpu_work->cpu,
				cpu_work->cur_freq, cpu_work->new_freq);
		goto done;
	} else {
		cancel_work_sync(&cpu_work->work);
		INIT_COMPLETION(cpu_work->complete);
		queue_work_on(policy->cpu, msm_cpufreq_wq, &cpu_work->work);
		wait_for_completion(&cpu_work->complete);
	}

	ret = cpu_work->status;

done:
	free_cpumask_var(mask);
	mutex_unlock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);
	return ret;
}

static int msm_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int msm_cpufreq_get_freq(unsigned int cpu)
{
	return acpuclk_get_rate(cpu);
}

int msm_cpufreq_set_freq_limits(enum cpufreq_limits_enum le,
		uint32_t cpu, uint32_t min, uint32_t max)
{
	struct cpu_freq_limit *limit = &cpu_freq_limits[le];
	struct cpufreq_work_struct *cpu_work = NULL;
	struct cpufreq_policy *policy = NULL;
	int i;


	mutex_lock(&limits_mutex);
	if ((min != MSM_CPUFREQ_NO_LIMIT) &&
			min >= cpuinfo_min && min <= cpuinfo_max)
		limit->allowed_min = min;
	else
		limit->allowed_min = cpuinfo_min;


	if ((max != MSM_CPUFREQ_NO_LIMIT) &&
			max <= cpuinfo_max && max >= cpuinfo_min)
		limit->allowed_max = max;
	else
		limit->allowed_max = cpuinfo_max;

	limit->cpu_mask = cpu;
	mutex_unlock(&limits_mutex);

	pr_debug("%s: Limiting mode: %d cpumask = %d min = %d, max = %d\n",
			__func__, le, cpu,
			limit->allowed_min, limit->allowed_max);

	for_each_online_cpu(i) {
		policy = cpufreq_cpu_get(i);
		if (!policy)
			continue;
		mutex_lock(&per_cpu(cpufreq_suspend, i).suspend_mutex);
		cpu_work = &per_cpu(cpufreq_work, i);
		cancel_work_sync(&cpu_work->work);
		INIT_COMPLETION(cpu_work->complete);
		cpu_work->new_freq = policy->cur;
		queue_work_on(i, msm_cpufreq_wq, &cpu_work->work);
		wait_for_completion(&cpu_work->complete);
		mutex_unlock(&per_cpu(cpufreq_suspend, i).suspend_mutex);
		cpufreq_cpu_put(policy);
	}

	return 0;
}
EXPORT_SYMBOL(msm_cpufreq_set_freq_limits);

static inline int msm_cpufreq_limits_init(void)
{
	int i = 0;
	struct cpu_freq_limit *limit = NULL;
	struct cpufreq_frequency_table *table = NULL;
	uint32_t min = (uint32_t) -1;
	uint32_t max = 0;

	table = cpufreq_frequency_get_table(0);
	if (table == NULL) {
		pr_err("%s: error reading cpufreq table for cpu\n", __func__);
		return -ENODEV;
	}

	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
		if (table[i].frequency > max)
			max = table[i].frequency;
		if (table[i].frequency < min)
			min = table[i].frequency;
	}

	cpuinfo_min = min;
	cpuinfo_max = max;

	mutex_lock(&limits_mutex);
	for (i = 0; i < NR_LIMITS; i++) {
		limit = &cpu_freq_limits[i];
		limit->allowed_min = cpuinfo_min;
		limit->allowed_max = cpuinfo_max;
		limit->cpu_mask = 0;
	}

	if (limit_turbo && (num_online_cpus() > limit_turbo)) {
		limit = &cpu_freq_limits[HOTPLUG_LIMIT];
		limit->allowed_min = cpuinfo_min;
		limit->allowed_max = TURBO_LIMIT;
		limit->cpu_mask = 0xFFFF;
	}
	mutex_unlock(&limits_mutex);

	return 0;
}

static int __cpuinit msm_cpufreq_init(struct cpufreq_policy *policy)
{
	int cur_freq;
	int index;
	struct cpufreq_frequency_table *table;
	struct cpufreq_work_struct *cpu_work = NULL;

	table = cpufreq_frequency_get_table(policy->cpu);
	if (table == NULL)
		return -ENODEV;
	/*
	 * In 8625, 8610, and 8226 both cpu core's frequency can not
	 * be changed independently. Each cpu is bound to
	 * same frequency. Hence set the cpumask to all cpu.
	 */
	if (cpu_is_msm8625() || cpu_is_msm8625q() || cpu_is_msm8226()
		|| cpu_is_msm8610())
		cpumask_setall(policy->cpus);

	if (cpufreq_frequency_table_cpuinfo(policy, table)) {
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
		policy->cpuinfo.min_freq = CONFIG_MSM_CPU_FREQ_MIN;
		policy->cpuinfo.max_freq = CONFIG_MSM_CPU_FREQ_MAX;
#endif
	}
#ifdef CONFIG_MSM_CPU_FREQ_SET_MIN_MAX
	policy->min = CONFIG_MSM_CPU_FREQ_MIN;
	policy->max = CONFIG_MSM_CPU_FREQ_MAX;
#endif

	cur_freq = acpuclk_get_rate(policy->cpu);
	if (cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_H, &index) &&
	    cpufreq_frequency_table_target(policy, table, cur_freq,
	    CPUFREQ_RELATION_L, &index)) {
		pr_info("cpufreq: cpu%d at invalid freq: %d\n",
				policy->cpu, cur_freq);
		return -EINVAL;
	}

	if (cur_freq != table[index].frequency) {
		int ret = 0;
		ret = set_cpu_freq(policy->cpu, cur_freq,
					table[index].frequency);
		if (ret)
			return ret;
		pr_debug("cpufreq: cpu%d init at %d switching to %d\n",
				policy->cpu, cur_freq, table[index].frequency);
		cur_freq = table[index].frequency;
	}

	policy->cur = cur_freq;

	policy->cpuinfo.transition_latency =
		acpuclk_get_switch_time() * NSEC_PER_USEC;

	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	INIT_WORK(&cpu_work->work, set_cpu_work);
	init_completion(&cpu_work->complete);

	return 0;
}

static int __cpuinit msm_cpufreq_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		if (limit_turbo && (num_online_cpus() == limit_turbo))
			msm_cpufreq_set_freq_limits(HOTPLUG_LIMIT, 0xFFFF,
				MSM_CPUFREQ_NO_LIMIT, TURBO_LIMIT);
		break;
	case CPU_DEAD:
		if (limit_turbo && (num_online_cpus() == limit_turbo))
			msm_cpufreq_set_freq_limits(HOTPLUG_LIMIT, 0xFFFF,
				MSM_CPUFREQ_NO_LIMIT, MSM_CPUFREQ_NO_LIMIT);
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
		break;
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		mutex_lock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
		mutex_unlock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		break;
	case CPU_DOWN_FAILED:
	case CPU_DOWN_FAILED_FROZEN:
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_cpufreq_cpu_notifier = {
	.notifier_call = msm_cpufreq_cpu_callback,
};

/*
 * Define suspend/resume for cpufreq_driver. Kernel will call
 * these during suspend/resume with interrupts disabled. This
 * helps the suspend/resume variable get's updated before cpufreq
 * governor tries to change the frequency after coming out of suspend.
 */
static int msm_cpufreq_suspend(struct cpufreq_policy *policy)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
	}

	return 0;
}

static int msm_cpufreq_resume(struct cpufreq_policy *policy)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	return 0;
}

static struct freq_attr *msm_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver msm_cpufreq_driver = {
	/* lps calculations are handled here. */
	.flags		= CPUFREQ_STICKY | CPUFREQ_CONST_LOOPS,
	.init		= msm_cpufreq_init,
	.verify		= msm_cpufreq_verify,
	.target		= msm_cpufreq_target,
	.get		= msm_cpufreq_get_freq,
	.suspend	= msm_cpufreq_suspend,
	.resume		= msm_cpufreq_resume,
	.name		= "msm",
	.attr		= msm_freq_attr,
};

static int __init msm_cpufreq_early_init(void)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		mutex_init(&(per_cpu(cpufreq_suspend, cpu).suspend_mutex));
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	return 0;
}
early_initcall(msm_cpufreq_early_init);

static int __init msm_cpufreq_register(void)
{
	int ret = 0;

	msm_cpufreq_wq = create_workqueue("msm-cpufreq");
	ret = msm_cpufreq_limits_init();
	if (ret)
		return ret;
	register_hotcpu_notifier(&msm_cpufreq_cpu_notifier);
	return cpufreq_register_driver(&msm_cpufreq_driver);
}

late_initcall(msm_cpufreq_register);
