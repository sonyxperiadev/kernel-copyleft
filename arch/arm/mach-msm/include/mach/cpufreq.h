/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_MACH_CPUFREQ_H
#define __ARCH_ARM_MACH_MSM_MACH_CPUFREQ_H

#define MSM_CPUFREQ_NO_LIMIT 0xFFFFFFFF
#define TURBO_LIMIT 1958400

enum cpufreq_limits_enum {
	THERMAL_LIMIT,
	HOTPLUG_LIMIT,
	NR_LIMITS,
};

#ifdef CONFIG_CPU_FREQ_MSM

/**
 * msm_cpufreq_set_freq_limit() - Set max/min freq limits on cpu
 *
 * @le: The enum for which this limits are being set
 * @cpu_mask: The cpu cores for which the limits apply
 * @max: The max frequency allowed
 * @min: The min frequency allowed
 *
 * If the @max or @min is set to MSM_CPUFREQ_NO_LIMIT, the limit
 * will default to the CPUFreq limit.
 *
 * returns 0 on success, errno on failure
 */
extern int msm_cpufreq_set_freq_limits(enum cpufreq_limits_enum le,
		uint32_t cpu_mask, uint32_t min, uint32_t max);
#else
static inline int msm_cpufreq_set_freq_limits(enum cpufreq_limits_enum le,
		uint32_t cpu_mask, uint32_t min, uint32_t max)
{
	return -ENOSYS;
}
#endif

#endif /* __ARCH_ARM_MACH_MSM_MACH_CPUFREQ_H */
