/*
 * Copyright (C) 2014 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _SELINUX_TRAP_H_
#define _SELINUX_TRAP_H_

#include <linux/stddef.h>
#include <linux/semaphore.h>
#include <linux/audit.h>
#include <linux/sched.h>
#include "avc.h"

enum trap_mask_type {
	TRAP_MASK_TYPE_POLARITY,
	TRAP_MASK_TYPE_SCONTEXT,
	TRAP_MASK_TYPE_TCONTEXT,
	TRAP_MASK_TYPE_TCLASS,
	TRAP_MASK_TYPE_PNAME,
	TRAP_MASK_TYPE_PNAME_PARENT,
	TRAP_MASK_TYPE_PNAME_PGL,
	TRAP_MASK_TYPE_PATH,
	TRAP_MASK_TYPE_NAME,
	TRAP_MASK_TYPE_ACTION,
	TRAP_MASK_TYPE_MAX
};
enum trap_loglevel_type {
	TRAP_LOGLEVEL_MINIMUM = 0,
	TRAP_LOGLEVEL_NORMAL,
	TRAP_LOGLEVEL_DEVELOPER,
	TRAP_LOGLEVEL_MAX
};

#define TRAP_MASK_TYPE_BEGIN TRAP_MASK_TYPE_POLARITY

struct selinux_trap_list {
	char *item_array[TRAP_MASK_TYPE_MAX];
	struct list_head list;
	struct rcu_head rcu;
};

struct selinux_trap_process_list {
	struct list_head list;
	pid_t pid ;
	char *msg ;
	struct inode *inode;
	struct dentry *ldentry;
};

void trap_selinux_error(struct common_audit_data *ad);
extern int selinux_trap_enable;
extern int selinux_trap_debug;
extern struct selinux_trap_list selinux_trap_list_head;
extern struct selinux_trap_process_list selinux_trap_process_list_head;
extern struct semaphore selinux_trap_list_sem;

#define trap_devel_log(fmt, ...) \
	do {		/* Multhi-statement Macro for semicolon */ \
		if (selinux_trap_debug >= TRAP_LOGLEVEL_DEVELOPER) { \
			pr_devel(fmt, ##__VA_ARGS__); \
		} \
	} while (0)	/* Multhi-statement Macro for semicolon */

#endif /* _SELINUX_TRAP_H_ */
