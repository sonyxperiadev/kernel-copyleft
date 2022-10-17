/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Sony Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_BARRIER_H
#define __ASM_BARRIER_H

#define mb() asm volatile ("l.msync" ::: "memory")

#include <asm-generic/barrier.h>

#endif /* __ASM_BARRIER_H */
