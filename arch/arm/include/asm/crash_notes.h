/* arch/arm/include/asm/crash_notes.h
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

#ifndef __ARM_CRASH_NOTES_H
#define __ARM_CRASH_NOTES_H

#define ARCH_HAS_CRASH_NOTES

static inline void crash_notes_save_regs(struct pt_regs *regs)
{
	__asm__ __volatile__ (
		"stmia	%[regs_base], {r0-r12}\n\t"
		"mov	%[_ARM_sp], sp\n\t"
		"str	lr, %[_ARM_lr]\n\t"
		"adr	%[_ARM_pc], 1f\n\t"
		"mrs	%[_ARM_cpsr], cpsr\n\t"
	"1:"
		: [_ARM_pc] "=r" (regs->ARM_pc),
		[_ARM_cpsr] "=r" (regs->ARM_cpsr),
		[_ARM_sp] "=r" (regs->ARM_sp),
		[_ARM_lr] "=o" (regs->ARM_lr)
		: [regs_base] "r" (&regs->ARM_r0)
		: "memory"
	);
}

#endif
