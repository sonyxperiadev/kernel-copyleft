/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * NOTE: This file has been modified by Sony Corporation.
 * Modifications are Copyright 2021 Sony Corporation,
 * and licensed under the license of the file.
 */
#ifndef _ASM_POWERPC_ERRNO_H
#define _ASM_POWERPC_ERRNO_H

#undef	EDEADLOCK
#include <asm-generic/errno.h>

#undef	EDEADLOCK
#define	EDEADLOCK	58	/* File locking deadlock error */

#endif	/* _ASM_POWERPC_ERRNO_H */
