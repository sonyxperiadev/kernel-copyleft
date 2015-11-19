/*
 * Based on arch/arm/kernel/process.c
 *
 *  Copyright(C) 2011-2014 Foxconn International Holdings, Ltd. All rights reserved.
 * Original Copyright (C) 1995  Linus Torvalds
 * Copyright (C) 1996-2000 Russell King - Converted to ARM.
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdarg.h>

#include <linux/compat.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
/* FIH-CORE-TH-DebugToolPorting-01+[ */
#ifdef CONFIG_FEATURE_FIH_SW3_PANIC_FILE
#include <linux/slab.h>	
#endif
/* FIH-CORE-TH-DebugToolPorting-01+] */
#include <linux/user.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/elfcore.h>
#include <linux/pm.h>
#include <linux/tick.h>
#include <linux/utsname.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/hw_breakpoint.h>
#include <linux/personality.h>
#include <linux/notifier.h>

#include <asm/compat.h>
#include <asm/cacheflush.h>
#include <asm/fpsimd.h>
#include <asm/mmu_context.h>
#include <asm/processor.h>
#include <asm/stacktrace.h>
/* FIH-CORE-TH-DebugToolPorting-01+[ */ 
#include <linux/fs.h>
#include <linux/file.h> 
#include <linux/fih_sw_info.h>
int send_mtbf = 1;	/* determine if send mtbf report */
/* FIH-CORE-TH-DebugToolPorting-01-] */ 
//CORE-KH-DebugTool_ForcePanic_forL-00-a[
#ifdef CONFIG_FEATURE_FIH_SW3_PANIC_FILE    
static void fih_write_panic_into_buffer(struct pt_regs *regs);
#endif
//CORE-KH-DebugTool_ForcePanic_forL-00-a]

static void setup_restart(void)
{
	/*
	 * Tell the mm system that we are going to reboot -
	 * we may need it to insert some 1:1 mappings so that
	 * soft boot works.
	 */
	setup_mm_for_reboot();

	/* Clean and invalidate caches */
	flush_cache_all();

	/* Turn D-cache off */
	cpu_cache_off();

	/* Push out any further dirty data, and ensure cache is empty */
	flush_cache_all();
}

void soft_restart(unsigned long addr)
{
	typedef void (*phys_reset_t)(unsigned long);
	phys_reset_t phys_reset;

	setup_restart();

	/* Switch to the identity mapping */
	phys_reset = (phys_reset_t)virt_to_phys(cpu_reset);
	phys_reset(addr);

	/* Should never get here */
	BUG();
}

/*
 * Function pointers to optional machine specific functions
 */
void (*pm_power_off)(void);
EXPORT_SYMBOL_GPL(pm_power_off);

void (*arm_pm_restart)(enum reboot_mode reboot_mode, const char *cmd);
EXPORT_SYMBOL_GPL(arm_pm_restart);

/*
 * This is our default idle handler.
 */
void arch_cpu_idle(void)
{
	/*
	 * This should do all the clock switching and wait for interrupt
	 * tricks
	 */
	if (cpuidle_idle_call()) {
		cpu_do_idle();
		local_irq_enable();
	}
}

void arch_cpu_idle_enter(void)
{
	idle_notifier_call_chain(IDLE_START);
}

void arch_cpu_idle_exit(void)
{
	idle_notifier_call_chain(IDLE_END);
}

#ifdef CONFIG_HOTPLUG_CPU
void arch_cpu_idle_dead(void)
{
       cpu_die();
}
#endif

/*
 * Called by kexec, immediately prior to machine_kexec().
 *
 * This must completely disable all secondary CPUs; simply causing those CPUs
 * to execute e.g. a RAM-based pin loop is not sufficient. This allows the
 * kexec'd kernel to use any and all RAM as it sees fit, without having to
 * avoid any code or data used by any SW CPU pin loop. The CPU hotplug
 * functionality embodied in disable_nonboot_cpus() to achieve this.
 */
void machine_shutdown(void)
{
	disable_nonboot_cpus();
}

/*
 * Halting simply requires that the secondary CPUs stop performing any
 * activity (executing tasks, handling interrupts). smp_send_stop()
 * achieves this.
 */
void machine_halt(void)
{
	local_irq_disable();
	smp_send_stop();
	while (1);
}

/*
 * Power-off simply requires that the secondary CPUs stop performing any
 * activity (executing tasks, handling interrupts). smp_send_stop()
 * achieves this. When the system power is turned off, it will take all CPUs
 * with it.
 */
void machine_power_off(void)
{
	local_irq_disable();
	smp_send_stop();
	if (pm_power_off)
		pm_power_off();
}

/*
 * Restart requires that the secondary CPUs stop performing any activity
 * while the primary CPU resets the system. Systems with a single CPU can
 * use soft_restart() as their machine descriptor's .restart hook, since that
 * will cause the only available CPU to reset. Systems with multiple CPUs must
 * provide a HW restart implementation, to ensure that all CPUs reset at once.
 * This is required so that any code running after reset on the primary CPU
 * doesn't have to co-ordinate with other CPUs to ensure they aren't still
 * executing pre-reset code, and using RAM that the primary CPU's code wishes
 * to use. Implementing such co-ordination would be essentially impossible.
 */
void machine_restart(char *cmd)
{
	/* Disable interrupts first */
	local_irq_disable();
	smp_send_stop();

	/* Now call the architecture specific reboot code. */
	if (arm_pm_restart)
		arm_pm_restart(REBOOT_HARD, cmd);

	/*
	 * Whoops - the architecture was unable to reboot.
	 */
	printk("Reboot failed -- System halted\n");
	while (1);
}

/*
 * dump a block of kernel memory from around the given address
 */
static void show_data(unsigned long addr, int nbytes, const char *name)
{
	int	i, j;
	int	nlines;
	u64	*p;

	/*
	 * don't attempt to dump non-kernel addresses or
	 * values that are probably just small negative numbers
	 */
	if (addr < PAGE_OFFSET || addr > -256UL)
		return;

	printk("\n%s: %#lx:\n", name, addr);

	/*
	 * round address down to a 64 bit boundary
	 * and always dump a multiple of 64 bytes
	 */
	p = (u64 *)(addr & ~(sizeof(u64) - 1));
	nbytes += (addr & (sizeof(u64) - 1));
	nlines = (nbytes + 63) / 64;

	for (i = 0; i < nlines; i++) {
		/*
		 * just display low 16 bits of address to keep
		 * each line of the dump < 80 characters
		 */
		printk("%04lx ", (unsigned long)p & 0xffff);
		for (j = 0; j < 8; j++) {
			u64 data;
			/*
			 * vmalloc addresses may point to
			 * memory-mapped peripherals
			 */
			if (!virt_addr_valid(p) ||
				 probe_kernel_address(p, data)) {
				printk(" ********");
			} else {
				printk(KERN_CONT " %016llx", data);
			}
			++p;
		}
		printk(KERN_CONT "\n");
	}
}

static void show_extra_register_data(struct pt_regs *regs, int nbytes)
{
	show_data(regs->pc - nbytes, nbytes * 2, "PC");
	show_data(regs->regs[30] - nbytes, nbytes * 2, "LR");
	show_data(regs->sp - nbytes, nbytes * 2, "SP");
}

void __show_regs(struct pt_regs *regs)
{
	int i, top_reg;
	u64 lr, sp;

	if (compat_user_mode(regs)) {
		lr = regs->compat_lr;
		sp = regs->compat_sp;
		top_reg = 12;
	} else {
		lr = regs->regs[30];
		sp = regs->sp;
		top_reg = 29;
	}

	show_regs_print_info(KERN_DEFAULT);
	print_symbol("PC is at %s\n", instruction_pointer(regs));
	print_symbol("LR is at %s\n", lr);
	printk("pc : [<%016llx>] lr : [<%016llx>] pstate: %08llx\n",
	       regs->pc, lr, regs->pstate);
	printk("sp : %016llx\n", sp);
	for (i = top_reg; i >= 0; i--) {
		printk("x%-2d: %016llx ", i, regs->regs[i]);
		if (i % 2 == 0)
			printk("\n");
	}
	/* Dump only kernel mode */
	if (get_fs() == get_ds())
		show_extra_register_data(regs, 256);
	printk("\n");

//CORE-KH-DebugTool_ForcePanic_forL-00-a[
#ifdef CONFIG_FEATURE_FIH_SW3_PANIC_FILE    
	/* determine if send mtbf report */
	if (send_mtbf == 1)	
		fih_write_panic_into_buffer(regs);
#endif
//CORE-KH-DebugTool_ForcePanic_forL-00-a]
}

/* FIH-CORE-TH-DebugToolPorting-01+[ */ 
/*====================================
 * write kernel panic data into txt
 *===================================*/ 
#ifdef CONFIG_FEATURE_FIH_SW3_PANIC_FILE 
#define KSYM_NAME_LEN_FIH 128

void __print_symbol_fih(char *buffer_panic, int buffer_size, unsigned long address) /*FIH-KERNEL-SC-fix_coverity-issues-03**/
{
	int strlen_char = 0;
	
	memset(buffer_panic, 0, buffer_size); /*FIH-KERNEL-SC-fix_coverity-issues-03**/
	sprint_symbol(buffer_panic, address);
	strlen_char = strlen(buffer_panic);
	strlcpy((buffer_panic + strlen_char), "\n", buffer_size);/*FIH-KERNEL-SC-fix_coverity-issues-03**/
}

void print_symbol_fih(unsigned long addr, char *buffer_panic, int buffer_size) /*FIH-KERNEL-SC-fix_coverity-issues-03**/
{
	__print_symbol_fih( buffer_panic, buffer_size, (unsigned long) __builtin_extract_return_addr((void *)addr));/*FIH-KERNEL-SC-fix_coverity-issues-03**/
}

/* MTD-CORE-EL-power_on_cause-00+[ */
void * get_hw_wd_virt_addr(void)
{
	static void *hw_wd_virt_addr = 0;

	if (unlikely(hw_wd_virt_addr == 0)){
		hw_wd_virt_addr = ioremap(FIH_HW_WD_ADDR, FIH_HW_WD_LEN);
		if (hw_wd_virt_addr == NULL)
			printk(KERN_ERR "hw_wd_virt_addr iormap failed\n");
	}

	return hw_wd_virt_addr;
}

EXPORT_SYMBOL(get_hw_wd_virt_addr);

void * get_pwron_cause_virt_addr(void)
{
	static void *pwron_cause_virt_addr = 0;

	if (unlikely(pwron_cause_virt_addr == 0)){
		pwron_cause_virt_addr = ioremap(FIH_PWRON_CAUSE_ADDR, FIH_PWRON_CAUSE_LEN);
		if (pwron_cause_virt_addr == NULL)
			printk(KERN_ERR "pwron_cause_virt_addr iormap failed\n");
	}

	return pwron_cause_virt_addr;
}

EXPORT_SYMBOL(get_pwron_cause_virt_addr);

/* MTD-CORE-EL-handle_SSR-00+[ */
void clear_all_modem_pwron_cause (void) {
	unsigned int *pwron_cause_ptr;

	pwron_cause_ptr = (unsigned int*) get_pwron_cause_virt_addr();
	if (pwron_cause_ptr == NULL)
		return;

	*pwron_cause_ptr &= ~MTD_PWR_ON_EVENT_MODEM_FATAL_ERROR;
	*pwron_cause_ptr &= ~MTD_PWR_ON_EVENT_MODEM_SW_WD_RESET;
	*pwron_cause_ptr &= ~MTD_PWR_ON_EVENT_MODEM_FW_WD_RESET;

	printk(KERN_ERR "%s called\n ", __func__);
}

EXPORT_SYMBOL(clear_all_modem_pwron_cause);
/* MTD-CORE-EL-handle_SSR-00+] */

/* MTD-CORE-EL-handle_SSR-00*[ */
unsigned int latest_modem_err = 0; 

void write_pwron_cause (int pwron_cause)
{
	unsigned int *pwron_cause_ptr;

	pwron_cause_ptr = (unsigned int*) get_pwron_cause_virt_addr();
	if (pwron_cause_ptr == NULL)
		return;

	switch (pwron_cause) {
	case HOST_KERNEL_PANIC:
		*pwron_cause_ptr |= MTD_PWR_ON_EVENT_KERNEL_PANIC;
		break;
	case MODEM_FATAL_ERR:
		*pwron_cause_ptr |= MTD_PWR_ON_EVENT_MODEM_FATAL_ERROR;
		/* MTD-CORE-EL-handle_SSR-00+ */
		latest_modem_err = MTD_PWR_ON_EVENT_MODEM_FATAL_ERROR;
		break;
	case MODEM_SW_WDOG_EXPIRED:
		*pwron_cause_ptr |= MTD_PWR_ON_EVENT_MODEM_SW_WD_RESET;
		/* MTD-CORE-EL-handle_SSR-00+ */
		latest_modem_err = MTD_PWR_ON_EVENT_MODEM_SW_WD_RESET;
		break;
	case MODEM_FW_WDOG_EXPIRED:
		*pwron_cause_ptr |= MTD_PWR_ON_EVENT_MODEM_FW_WD_RESET;
		/* MTD-CORE-EL-handle_SSR-00+ */
		latest_modem_err = MTD_PWR_ON_EVENT_MODEM_FW_WD_RESET;
		break;
	case SOFTWARE_RESET:
		if (*pwron_cause_ptr & MTD_PWR_ON_EVENT_PWR_OFF_CHG_REBOOT)
			printk("PWR_OFF_CHG_REBOOT is detected. Keep POC as it is.\n");
		else
			*pwron_cause_ptr |= MTD_PWR_ON_EVENT_SOFTWARE_RESET;
		break;
	case PWR_OFF_CHG_REBOOT:
		*pwron_cause_ptr |= MTD_PWR_ON_EVENT_PWR_OFF_CHG_REBOOT;
		break;
	default:
		printk(KERN_ERR "%d Unknown reboot_reason!\n", pwron_cause);
	}
}

/* MTD-CORE-EL-handle_SSR-00*] */


EXPORT_SYMBOL(write_pwron_cause);
/* MTD-CORE-EL-power_on_cause-00+] */

void * get_alog_buffer_virt_addr(void){
	static void *alog_buffer_virt_addr = 0;

	if (unlikely(alog_buffer_virt_addr == 0)){
		alog_buffer_virt_addr = ioremap(PANIC_RAM_DATA_BEGIN, PANIC_RAM_DATA_SIZE);
	}

	return alog_buffer_virt_addr;
}

EXPORT_SYMBOL(get_alog_buffer_virt_addr);

void * get_timestamp_buffer_virt_addr(void){
	static void *buffer_virt_addr = 0;

	if (unlikely(buffer_virt_addr == 0)){
		buffer_virt_addr = ioremap(CRASH_TIME_RAMDUMP_ADDR, CRASH_TIME_RAMDUMP_LEN);
	}
	return buffer_virt_addr;
}

EXPORT_SYMBOL(get_timestamp_buffer_virt_addr);

//CORE-KH-DebugTool_ForcePanic_forL-00-m[
static void fih_write_panic_into_buffer(struct pt_regs *regs)
{
	int strlen_char = 0;
	char *panic_string = NULL;
	char pointer_data[300] = {0};
	char buffer_panic[128] = {0};
	u64 lr, sp;
//CORE-KH-DebugTool_ForcePanic_forL-00-d
//	int panic_string_len = PANIC_RAM_DATA_SIZE - sizeof(struct fih_panic_ram_data);/*FIH-KERNEL-SC-fix_coverity-issues-03*/
	int	buf_str_len = 0, total_size=0;

	struct fih_panic_ram_data *fih_panic_ram_data_ptr = 
		(struct fih_panic_ram_data *) get_alog_buffer_virt_addr();
        
	/* Lookup function in PC */
	if (fih_panic_ram_data_ptr == NULL) {
		printk("ioremap PANIC_RAM_DATA_BEGIN fail\n");
		return;
	}

	panic_string = fih_panic_ram_data_ptr->data;
	printk("panic_ram_data addr= %016llx\n", (u64)(char *)panic_string);
	memset_io(panic_string, 0, PANIC_RAM_DATA_SIZE);
//	strlcpy(panic_string, "PC is at ", panic_string_len);
	memcpy_toio(panic_string, "PC is at ", 9);
	strlen_char= strlen(panic_string);
	total_size += strlen_char;
	print_symbol_fih(instruction_pointer(regs), buffer_panic, sizeof(buffer_panic));
	buf_str_len= strlen(buffer_panic);
	total_size += buf_str_len;
//	strlcpy(panic_string + strlen_char, buffer_panic, panic_string_len);
	memcpy_toio(panic_string + strlen_char, buffer_panic, (size_t)buf_str_len);
	strlen_char= strlen(panic_string);
	total_size += strlen_char;
	/* Lookup function in LR */
//	strlcpy(panic_string + strlen_char, "LR is at ", panic_string_len);
	memcpy_toio(panic_string + strlen_char, "LR is at ", 9);
	strlen_char= strlen(panic_string);

	if (compat_user_mode(regs)) {
		lr = regs->compat_lr;
		sp = regs->compat_sp;
	} else {
		lr = regs->regs[30];
		sp = regs->sp;
	}
	print_symbol_fih((unsigned long)lr, buffer_panic, sizeof(buffer_panic));
//	strlcpy(panic_string + strlen_char, buffer_panic, panic_string_len);
	buf_str_len= strlen(buffer_panic);
	total_size += buf_str_len;
	memcpy_toio(panic_string + strlen_char, buffer_panic, (size_t)buf_str_len);
	strlen_char= strlen(panic_string);
	total_size += strlen_char;
	
	/* Get pointer value: pc, lr, psr, sp, ip, fp */        
	memset(pointer_data, 0, sizeof(pointer_data));
	snprintf(pointer_data, sizeof(pointer_data), "pc : [<%016llx>]    lr : [<%016llx>]    psr: %08llx\n"
		"sp : %016llx  ip : %016llx  fp : %016llx\n", regs->pc, lr, regs->pstate,
		sp, regs->compat_usr(12), regs->compat_fp);
//	strlcpy(panic_string + strlen_char, pointer_data, panic_string_len);
	buf_str_len= strlen(pointer_data);
	total_size += buf_str_len;
	memcpy_toio(panic_string + strlen_char, pointer_data, (size_t)buf_str_len);
	strlen_char= strlen(panic_string);
	total_size += strlen_char;

   
	/* Get r0 -r10 value */  
	memset(pointer_data, 0, sizeof(pointer_data));
	snprintf(pointer_data, sizeof(pointer_data), "r10: %016llx  r9 : %016llx  r8 : %016llx\n"
		"r7 : %016llx  r6 : %016llx  r5 : %016llx  r4 : %016llx\n"
		"r3 : %016llx  r2 : %016llx  r1 : %016llx  r0 : %016llx\n", 
		regs->compat_usr(10), regs->compat_usr(9),regs->compat_usr(8),
		regs->compat_usr(7), regs->compat_usr(6),regs->compat_usr(5), 
		regs->compat_usr(4), regs->compat_usr(3), regs->compat_usr(2),
		regs->compat_usr(1), regs->compat_usr(0));
//	strlcpy(panic_string + strlen_char, pointer_data, panic_string_len);
	buf_str_len= strlen(pointer_data);
	total_size += buf_str_len;
	if (total_size >= PANIC_RAM_DATA_SIZE)
        goto oversize;
	memcpy_toio(panic_string + (u64)strlen_char, pointer_data, (size_t)buf_str_len);
	strlen_char= strlen(panic_string);
	total_size += strlen_char;

oversize:
	printk("actually panic_ram_data size= 0x%x\n", total_size);
	fih_panic_ram_data_ptr->length = strlen_char;
	fih_panic_ram_data_ptr->signature = PANIC_RAM_SIGNATURE;	
}
//CORE-KH-DebugTool_ForcePanic_forL-00-m]
#endif

void show_regs(struct pt_regs * regs)
{
	printk("\n");
	__show_regs(regs);
}

/*
 * Free current thread data structures etc..
 */
void exit_thread(void)
{
}

void flush_thread(void)
{
	fpsimd_flush_thread();
	flush_ptrace_hw_breakpoint(current);
}

void release_thread(struct task_struct *dead_task)
{
}

int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src)
{
	fpsimd_preserve_current_state();
	*dst = *src;
	return 0;
}

asmlinkage void ret_from_fork(void) asm("ret_from_fork");

int copy_thread(unsigned long clone_flags, unsigned long stack_start,
		unsigned long stk_sz, struct task_struct *p)
{
	struct pt_regs *childregs = task_pt_regs(p);
	unsigned long tls = p->thread.tp_value;

	memset(&p->thread.cpu_context, 0, sizeof(struct cpu_context));

	if (likely(!(p->flags & PF_KTHREAD))) {
		*childregs = *current_pt_regs();
		childregs->regs[0] = 0;
		if (is_compat_thread(task_thread_info(p))) {
			if (stack_start)
				childregs->compat_sp = stack_start;
		} else {
			/*
			 * Read the current TLS pointer from tpidr_el0 as it may be
			 * out-of-sync with the saved value.
			 */
			asm("mrs %0, tpidr_el0" : "=r" (tls));
			if (stack_start) {
				/* 16-byte aligned stack mandatory on AArch64 */
				if (stack_start & 15)
					return -EINVAL;
				childregs->sp = stack_start;
			}
		}
		/*
		 * If a TLS pointer was passed to clone (4th argument), use it
		 * for the new thread.
		 */
		if (clone_flags & CLONE_SETTLS)
			tls = childregs->regs[3];
	} else {
		memset(childregs, 0, sizeof(struct pt_regs));
		childregs->pstate = PSR_MODE_EL1h;
		p->thread.cpu_context.x19 = stack_start;
		p->thread.cpu_context.x20 = stk_sz;
	}
	p->thread.cpu_context.pc = (unsigned long)ret_from_fork;
	p->thread.cpu_context.sp = (unsigned long)childregs;
	p->thread.tp_value = tls;

	ptrace_hw_copy_thread(p);

	return 0;
}

static void tls_thread_switch(struct task_struct *next)
{
	unsigned long tpidr, tpidrro;

	if (!is_compat_task()) {
		asm("mrs %0, tpidr_el0" : "=r" (tpidr));
		current->thread.tp_value = tpidr;
	}

	if (is_compat_thread(task_thread_info(next))) {
		tpidr = 0;
		tpidrro = next->thread.tp_value;
	} else {
		tpidr = next->thread.tp_value;
		tpidrro = 0;
	}

	asm(
	"	msr	tpidr_el0, %0\n"
	"	msr	tpidrro_el0, %1"
	: : "r" (tpidr), "r" (tpidrro));
}

/*
 * Thread switching.
 */
struct task_struct *__switch_to(struct task_struct *prev,
				struct task_struct *next)
{
	struct task_struct *last;

	fpsimd_thread_switch(next);
	tls_thread_switch(next);
	hw_breakpoint_thread_switch(next);
	contextidr_thread_switch(next);

	/*
	 * Complete any pending TLB or cache maintenance on this CPU in case
	 * the thread migrates to a different CPU.
	 */
	dsb(ish);

	/* the actual thread switch */
	last = cpu_switch_to(prev, next);

	return last;
}

unsigned long get_wchan(struct task_struct *p)
{
	struct stackframe frame;
	unsigned long stack_page;
	int count = 0;
	if (!p || p == current || p->state == TASK_RUNNING)
		return 0;

	frame.fp = thread_saved_fp(p);
	frame.sp = thread_saved_sp(p);
	frame.pc = thread_saved_pc(p);
	stack_page = (unsigned long)task_stack_page(p);
	do {
		if (frame.sp < stack_page ||
		    frame.sp >= stack_page + THREAD_SIZE ||
		    unwind_frame(&frame))
			return 0;
		if (!in_sched_functions(frame.pc))
			return frame.pc;
	} while (count ++ < 16);
	return 0;
}

unsigned long arch_align_stack(unsigned long sp)
{
	if (!(current->personality & ADDR_NO_RANDOMIZE) && randomize_va_space)
		sp -= get_random_int() & ~PAGE_MASK;
	return sp & ~0xf;
}

static unsigned long randomize_base(unsigned long base)
{
	unsigned long range_end = base + (STACK_RND_MASK << PAGE_SHIFT) + 1;
	return randomize_range(base, range_end, 0) ? : base;
}

unsigned long arch_randomize_brk(struct mm_struct *mm)
{
	return randomize_base(mm->brk);
}

unsigned long randomize_et_dyn(unsigned long base)
{
	return randomize_base(base);
}
