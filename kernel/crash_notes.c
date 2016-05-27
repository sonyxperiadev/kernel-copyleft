/* kernel/crash_notes.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/capability.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/slab.h>
#include <linux/nmi.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/crash_notes.h>
#include <asm/cacheflush.h>

#define CRASH_NOTE_NAME "CORE"

#define CRASH_NOTE_SIZE (ALIGN(sizeof(struct elf_note), 4) + \
			 ALIGN(sizeof(CRASH_NOTE_NAME), 4) + \
			 ALIGN(sizeof(struct elf_prstatus), 4))

#define CRASH_NOTE_BYTES (2 * CRASH_NOTE_SIZE)

typedef u32 note_buf_t[CRASH_NOTE_BYTES / 4];
note_buf_t *crash_notes;

static atomic_t crash_notes_ipi;

static inline void crash_notes_save_this_cpu(enum crash_note_save_type type,
			       unsigned int cpu)
{
	struct elf_prstatus prstatus;
	struct pt_regs regs;
	struct elf_note *note;
	u32 *buf;
	u32 *start;

	buf = (u32 *)per_cpu_ptr(crash_notes, cpu);
	if (!buf)
		return;

	start = buf;
	memset(&prstatus, 0, sizeof(prstatus));
	prstatus.pr_pid = current->pid;

	if (type != CRASH_NOTE_INIT) {
		crash_notes_save_regs(&regs);
		elf_core_copy_regs(&prstatus.pr_reg, &regs);
	}

	note = (struct elf_note *)buf;
	note->n_namesz = strnlen(CRASH_NOTE_NAME, 5) + 1;
	note->n_descsz = sizeof(prstatus);
	note->n_type = NT_PRSTATUS;
	buf += (sizeof(struct elf_note) + 3) / 4;
	memcpy(buf, CRASH_NOTE_NAME, note->n_namesz);
	buf += (note->n_namesz + 3) / 4;
	memcpy(buf, &prstatus, sizeof(prstatus));
	buf += (note->n_descsz + 3) / 4;

	note = (struct elf_note *)buf;
	note->n_namesz = 0;
	note->n_descsz = 0;
	note->n_type   = 0;

	flush_cache_all();
}

static DEFINE_RAW_SPINLOCK(print_trace_lock);

static void crash_notes_save_nopanic_cpu(void *unused)
{
	unsigned int cpu = smp_processor_id();

	raw_spin_lock(&print_trace_lock);
	pr_crit("CPU%u: stopping\n", cpu);
	dump_stack();
	raw_spin_unlock(&print_trace_lock);

	crash_notes_save_this_cpu(CRASH_NOTE_STOPPING, cpu);
	atomic_dec(&crash_notes_ipi);

	set_cpu_active(cpu, false);
	flush_cache_all();

	while (1)
		cpu_relax();
}

void crash_notes_save_cpus(void)
{
	unsigned long msecs;

	local_irq_disable();

	atomic_set(&crash_notes_ipi, num_online_cpus() - 1);
	smp_call_function(crash_notes_save_nopanic_cpu, NULL, false);
	msecs = 1000;
	while ((atomic_read(&crash_notes_ipi) > 0) && msecs) {
		mdelay(1);
		msecs--;
	}

	if (atomic_read(&crash_notes_ipi) > 0)
		pr_warn("Non-crashing CPUs did not react to IPI\n");
}
EXPORT_SYMBOL(crash_notes_save_cpus);

static int crash_notes_save_panic_cpu(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	long i;

	crash_notes_save_this_cpu(CRASH_NOTE_CRASHING, smp_processor_id());

	bust_spinlocks(0);

	if (panic_timeout > 0) {
		/*
		 * Delay timeout seconds before rebooting the machine.
		 * We can't use the "normal" timers since we just panicked.
		 */
		pr_emerg("Rebooting in %d seconds..", panic_timeout);

		for (i = 0; i < panic_timeout; i++) {
			touch_nmi_watchdog();
			mdelay(MSEC_PER_SEC);
		}
		/*
		 * This will not be a clean reboot, with everything
		 * shutting down.  But if there is a chance of
		 * rebooting the system it will be rebooted.
		 */
		emergency_restart();
	}

	local_irq_enable();
	while (1) {
		touch_softlockup_watchdog();
		mdelay(MSEC_PER_SEC);
	}

	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = crash_notes_save_panic_cpu,
	.priority = INT_MIN /* will not return; must be done last */
};

static int __init crash_notes_init(void)
{
	int i;

	/* Allocate memory for saving cpu registers. */
	crash_notes = alloc_percpu(note_buf_t);
	if (!crash_notes) {
		pr_err("allocation of percpu crash notes failed\n");
		return -ENOMEM;
	}

	/* Initialize memory with something that the tools pick up on */
	/* It will NOT be useful register info, but it's something at least */
	for_each_possible_cpu(i) {
		crash_notes_save_this_cpu(CRASH_NOTE_INIT, i);
	}

	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	return 0;
}
module_init(crash_notes_init)
