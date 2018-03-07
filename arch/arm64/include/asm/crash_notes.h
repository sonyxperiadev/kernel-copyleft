/* arch/arm64/include/asm/crash_notes.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef __ARM64_CRASH_NOTES_H
#define __ARM64_CRASH_NOTES_H

#include <asm/ptrace.h>

#define PSTATE_NZCV_MASK (PSR_N_BIT | PSR_Z_BIT | PSR_C_BIT | PSR_V_BIT)
#define PSTATE_DAIF_MASK (PSR_D_BIT | PSR_A_BIT | PSR_I_BIT | PSR_F_BIT)

static inline void crash_notes_save_regs(struct pt_regs *regs)
{
	struct pstate {
		u64 nzcv;
		u64 daif;
		u64 current_el;
		u64 sp_sel;
	} pstate;

	/* 31 General purpose registers x0-x30 */
	__asm__ __volatile__("str x0, %0" : "=m"(regs->regs[0]));
	__asm__ __volatile__("str x1, %0" : "=m"(regs->regs[1]));
	__asm__ __volatile__("str x2, %0" : "=m"(regs->regs[2]));
	__asm__ __volatile__("str x3, %0" : "=m"(regs->regs[3]));
	__asm__ __volatile__("str x4, %0" : "=m"(regs->regs[4]));
	__asm__ __volatile__("str x5, %0" : "=m"(regs->regs[5]));
	__asm__ __volatile__("str x6, %0" : "=m"(regs->regs[6]));
	__asm__ __volatile__("str x7, %0" : "=m"(regs->regs[7]));
	__asm__ __volatile__("str x8, %0" : "=m"(regs->regs[8]));
	__asm__ __volatile__("str x9, %0" : "=m"(regs->regs[9]));
	__asm__ __volatile__("str x10, %0" : "=m"(regs->regs[10]));
	__asm__ __volatile__("str x11, %0" : "=m"(regs->regs[11]));
	__asm__ __volatile__("str x12, %0" : "=m"(regs->regs[12]));
	__asm__ __volatile__("str x13, %0" : "=m"(regs->regs[13]));
	__asm__ __volatile__("str x14, %0" : "=m"(regs->regs[14]));
	__asm__ __volatile__("str x15, %0" : "=m"(regs->regs[15]));
	__asm__ __volatile__("str x16, %0" : "=m"(regs->regs[16]));
	__asm__ __volatile__("str x17, %0" : "=m"(regs->regs[17]));
	__asm__ __volatile__("str x18, %0" : "=m"(regs->regs[18]));
	__asm__ __volatile__("str x19, %0" : "=m"(regs->regs[19]));
	__asm__ __volatile__("str x20, %0" : "=m"(regs->regs[20]));
	__asm__ __volatile__("str x21, %0" : "=m"(regs->regs[21]));
	__asm__ __volatile__("str x22, %0" : "=m"(regs->regs[22]));
	__asm__ __volatile__("str x23, %0" : "=m"(regs->regs[23]));
	__asm__ __volatile__("str x24, %0" : "=m"(regs->regs[24]));
	__asm__ __volatile__("str x25, %0" : "=m"(regs->regs[25]));
	__asm__ __volatile__("str x26, %0" : "=m"(regs->regs[26]));
	__asm__ __volatile__("str x27, %0" : "=m"(regs->regs[27]));
	__asm__ __volatile__("str x28, %0" : "=m"(regs->regs[28]));
	__asm__ __volatile__("str x29, %0" : "=m"(regs->regs[29]));
	__asm__ __volatile__("str x30, %0" : "=m"(regs->regs[30]));

	/* Save program counter & stack pointer here */
	__asm__ __volatile__(
		"mov %[_ARM_sp], sp\n\t"
		"adr %[_ARM_pc], 1f\n\t"
	"1:"
		: [_ARM_pc] "=r" (regs->pc),
		  [_ARM_sp] "=r" (regs->sp)
	);

	/* Obtain pstate through system registers */
	__asm__ __volatile__("mrs %0, nzcv" : "=&r"(pstate.nzcv));
	__asm__ __volatile__("mrs %0, daif" : "=&r"(pstate.daif));
	__asm__ __volatile__("mrs %0, currentel" : "=&r"(pstate.current_el));
	__asm__ __volatile__("mrs %0, spsel" : "=&r"(pstate.sp_sel));
	regs->pstate = pstate.nzcv & PSTATE_NZCV_MASK;
	regs->pstate |= pstate.daif & PSTATE_DAIF_MASK;
	regs->pstate |= pstate.current_el & PSR_MODE_MASK;
	regs->pstate |= pstate.sp_sel & 1;
}

#endif
