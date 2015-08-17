/* include/linux/crash_notes.h
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

#ifndef __CRASH_NOTES_H
#define __CRASH_NOTES_H

#ifdef CONFIG_CRASH_NOTES
#include <asm/crash_notes.h>

enum crash_note_save_type {
	CRASH_NOTE_INIT,
	CRASH_NOTE_STOPPING,
	CRASH_NOTE_CRASHING
};

extern void crash_notes_save_cpus(void);

#ifndef ARCH_HAS_CRASH_NOTES
static inline void crash_notes_save_regs(struct pt_regs *regs) { }
#endif

#else /*!CONFIG_CRASH_NOTES*/
static inline void crash_notes_save_cpus(void) { }
#endif /*CONFIG_CRASH_NOTES*/
#endif /*__CRASH_NOTES_H*/
