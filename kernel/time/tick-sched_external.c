/*
 * Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved.
 * Copyright (C) 2011 Foxconn.  All rights reserved.
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/percpu.h>
#include <linux/profile.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/rq_stats.h>

#include <asm/irq_regs.h>

#include "tick-internal.h"

#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/kallsyms.h>



/* total cpu number*/
/* CORE-HC-CPU_Usage-00+[ */
#ifdef CONFIG_NR_CPUS
#define CPU_NUMS CONFIG_NR_CPUS
#else
#define CPU_NUMS 4
#endif
/* CORE-HC-CPU_Usage-00+] */

u64 Last_checked_jiffies = 0;

u64 LastCheckedTime[CPU_NUMS] = {0};
u64 LastBusyTime[CPU_NUMS]  = {0};

u64 LastIdleTime[CPU_NUMS]  = {0};
u64 LastIoWaitTime[CPU_NUMS]  = {0};


extern unsigned int debug_cpu_usage_enable;

#define CPU_USAGE_CHECK_INTERVAL_MS 1000  /* 1000ms */

/* CORE-HC-CPU_Usage-00+[ */
long LastCpuUsage[CPU_NUMS]  = {0};
long LastIowaitUsage[CPU_NUMS]  = {0};
/* CORE-HC-CPU_Usage-00+] */

/*Kernel-SC-add-cpuFreq-info-01+[*/
/* acpuclk_get_rate(): get REAL cpu frequency*/
extern unsigned long acpuclk_get_rate(int cpu);
/* cpufreq_quick_get(): get cpu governor's current cpu frequency. Return value will be same as what is shown in scaling_cur_freq in sysfs.*/
unsigned int cpufreq_quick_get(unsigned int cpu);
/*Kernel-SC-add-cpuFreq-info-01+]*/

long get_cpu_usage(int cpu, long*  IoWaitUsage)
{
	u64 CurrTime = 0;
	u64 CurrBusyTime = 0;
	u64 CurrIoWaitTime = 0;
	
	long TotalTickCount = 0;
	long BusyTickCount = 0;
	long IoWaitTickCount = 0;

	long Usage = 0;
	
	*IoWaitUsage = 0;

	if(cpu >= CPU_NUMS)
	{
		return Usage;
	}
	
	/* get the current time */
	CurrTime = jiffies_to_usecs(jiffies_64);

	/* get this cpu's busy time */
	CurrBusyTime  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	CurrBusyTime += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	CurrBusyTime += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	CurrBusyTime += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	CurrBusyTime += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	CurrBusyTime += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	CurrBusyTime =  jiffies_to_usecs(CurrBusyTime);
	CurrIoWaitTime =  jiffies_to_usecs(kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT]);
		
	/* Calculate the time interval */
	TotalTickCount = CurrTime - LastCheckedTime[cpu];
	BusyTickCount = CurrBusyTime - LastBusyTime[cpu];
	IoWaitTickCount = CurrIoWaitTime - LastIoWaitTime[cpu];

	/* record last current and busy time */
	LastBusyTime[cpu] = CurrBusyTime;
	LastCheckedTime[cpu] = CurrTime;
	LastIoWaitTime[cpu] = CurrIoWaitTime;

	Last_checked_jiffies = jiffies_64;
	
	/* Calculate the busy rate */
	if (TotalTickCount >= BusyTickCount && TotalTickCount > 0)
	{
		BusyTickCount = 100 * BusyTickCount;
		do_div(BusyTickCount, TotalTickCount);
		Usage = BusyTickCount;

		IoWaitTickCount = 100 * IoWaitTickCount;
		do_div(IoWaitTickCount, TotalTickCount);
		*IoWaitUsage = IoWaitTickCount;
	}
	else
	{
		Usage = 0;
	}

	return Usage;
}

/* CORE-HC-CPU_Usage-00+[ */
#define MAX_REG_LOOP_CHAR	512
static int get_cpu_usage_param(char *buf, struct kernel_param *kp)
{
	int cpu = 0, count = 0, total_count = 0;
	for_each_present_cpu(cpu)
	{
		count = snprintf(buf + total_count, MAX_REG_LOOP_CHAR - total_count, 
			"CPU%d%s:%3ld%%,IOW=%3ld%%\n", 
			cpu, 
			(cpu_is_offline(cpu) ? "(off)" : "(on) "), 
			LastCpuUsage[cpu], 
			LastIowaitUsage[cpu]);
		
		total_count += count;
	}
	
	return total_count;
}
module_param_call(cpu_usage, NULL, get_cpu_usage_param, NULL, 0644);
/* CORE-HC-CPU_Usage-00+] */


/* CORE-EL-CPU_Freq-00+ */
extern void cpufreq_quick_get_infos(unsigned int cpu, unsigned int *min, 
											unsigned int *max, unsigned int *cur);


/*Kernel-SC-add-cpuFreq-info-02+[*/

/*
* cpu_info_msg: output message
* msg_len: the size of output message.
* output message is included cpu's usage, real cpu frequecy and cpu governor's current cpu frequency.
*/
/* CORE-EL-CPU_Freq-00*[ */
void show_cpu_usage_and_freq(char * cpu_info_msg, int msg_len)
{	
	
	unsigned int cpu_freq_min = 0;
	unsigned int cpu_freq_max = 0;
	unsigned int cpu_curr_freq = 0;
	long cpu_usage = 0;

	long iowait_usage = 0;

	int cpu;

	int len = msg_len; 
	int str_len = 0;
	int tmp_len = 0;

	if(!cpu_info_msg || msg_len <= 0)
	{
		return;
	}

	cpufreq_quick_get_infos(0, &cpu_freq_min, &cpu_freq_max, &cpu_curr_freq);
	cpu_freq_max /= 1000;
	cpu_freq_min /= 1000;
	cpu_curr_freq /= 1000; /*cpu governor's current cpu frequency*/

	tmp_len = snprintf(cpu_info_msg, len, "[m=%u M=%u F=%u]", cpu_freq_min, cpu_freq_max, cpu_curr_freq);	
	
	str_len += tmp_len;
	len -= tmp_len;
	
	for_each_present_cpu(cpu)
	{
		cpu_usage = get_cpu_usage(cpu, &iowait_usage); /*get cpu usage*/

		/* CORE-HC-CPU_Usage-00+[ */
		LastCpuUsage[cpu]  = cpu_usage;
		LastIowaitUsage[cpu]  = iowait_usage;
		/* CORE-HC-CPU_Usage-00+] */

		tmp_len = snprintf((cpu_info_msg + str_len), len, "[C%d%s:%3ld%% IOW=%3ld%%]", cpu, (cpu_is_offline(cpu) ? " off" : ""), cpu_usage, iowait_usage);	
		
		str_len += tmp_len;
		len -= tmp_len;
		
		if(len <= 0 || str_len >= msg_len)
		{
			break;
		}
	}

	if(len > 0 && str_len < msg_len)
	{
		snprintf((cpu_info_msg + str_len), len, "C%d:", smp_processor_id()); /*this cpu is handling this timer interrupt*/
	}
	
}
/* CORE-EL-CPU_Freq-00*] */
/*Kernel-SC-add-cpuFreq-info-02+]*/


void count_cpu_time(void)
{
	
	if (unlikely(debug_cpu_usage_enable == 1 && (1000*(jiffies_64 - Last_checked_jiffies)/HZ >= CPU_USAGE_CHECK_INTERVAL_MS)))
	{
		struct task_struct * p = current;
		struct pt_regs *regs = get_irq_regs();
		
		char cpu_info_msg[128] = {0};
		int len = sizeof(cpu_info_msg);
		
		show_cpu_usage_and_freq(cpu_info_msg, len);

		if (likely(p != 0))
		{
			if (regs != 0)
			{
				if (regs->ARM_pc <= TASK_SIZE)  /* User space */
				{
					struct mm_struct *mm = p->mm;
					struct vm_area_struct *vma;
					struct file *map_file = NULL;

					/* Parse vma information */
					vma = find_vma(mm, regs->ARM_pc);

					if (vma != NULL)
					{
						map_file = vma->vm_file;
					
						if (map_file)  /* memory-mapped file */
						{
							printk(KERN_INFO "%sLR=0x%08lx PC=0x%08lx[U][%4d][%s][%s+0x%lx]\r\n",
								cpu_info_msg,
								regs->ARM_lr, 
								regs->ARM_pc, 
								(unsigned int) p->pid,
								p->comm,
								map_file->f_path.dentry->d_iname, 
								regs->ARM_pc - vma->vm_start);/*Kernel-SC-add-cpuFreq-info-02**/
						}
						else 
						{
							const char *name = arch_vma_name(vma);
							if (!name) 
							{
								if (mm) 
								{
									if (vma->vm_start <= mm->start_brk &&
										vma->vm_end >= mm->brk) 
									{
										name = "heap";
									} 
									else if (vma->vm_start <= mm->start_stack &&
										vma->vm_end >= mm->start_stack) 
									{
										name = "stack";
									}
								}
								else 
								{
									name = "vdso";
								}
							}
							printk(KERN_INFO "%sLR=0x%08lx PC=0x%08lx[U][%4d][%s][%s]\r\n",
								cpu_info_msg,
								regs->ARM_lr, 
								regs->ARM_pc, 
								(unsigned int) p->pid,
								p->comm, 
								name);/*Kernel-SC-add-cpuFreq-info-02**/
						}
					}
				}
				else /* Kernel space */
				{
					printk(KERN_INFO "%sLR=0x%08lx PC=0x%08lx[K][%4d][%s]", 
						cpu_info_msg,
						regs->ARM_lr, 
						regs->ARM_pc, 
						(unsigned int) p->pid,
						p->comm);/*Kernel-SC-add-cpuFreq-info-02**/
					
					#ifdef CONFIG_KALLSYMS
					print_symbol("[%s]\r\n", regs->ARM_pc);
					#else
					printk(KERN_INFO "\r\n"); /* KERNEL-SC-debug-msg-00* */
					#endif
				}
			}
			else  /* Can't get PC & RA address */
			{
				printk(KERN_INFO "%s[%s]\r\n", 
					cpu_info_msg, 
					p->comm);/*Kernel-SC-add-cpuFreq-info-02**/
			}
		}
		else  /* Can't get process information */
		{
			printk(KERN_INFO "%sERROR: p=0x%08lx regs=0x%08lx\r\n", 
				cpu_info_msg, 
				(long)p, 
				(long)regs);/*Kernel-SC-add-cpuFreq-info-02**/
		}
	}
}
